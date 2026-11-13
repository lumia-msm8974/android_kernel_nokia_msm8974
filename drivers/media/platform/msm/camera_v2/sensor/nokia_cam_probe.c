/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Nokia Lumia (martini/bandit) camera sensor identification probe.
 *
 * A read-only, on-demand helper for bringing up the rear Karma and front
 * Skype_HD SMIA++ modules whose silicon die, CCI address and MIPI lane count
 * are not yet known (see CAMERA-MARTINI-PIANO.md). It reuses the MSM8974
 * camera_v2 CCI stack and the device-tree power/clock/regulator/GPIO parser,
 * so the power-up sequence is exactly the one described in the DT node
 * (derived from the Windows PEP). On a debugfs trigger it:
 *
 *   1. powers the sensor up with that sequence (MCLK 9.6 MHz by default),
 *   2. scans the candidate 7-bit CCI slave addresses,
 *   3. at each address that ACKs, reads the SMIA++ identification block
 *      (0x0000 model id, 0x0002 revision, 0x0003 manufacturer, 0x0004 SMIA
 *      version, 0x0016/0x0018/0x0019 die model/revision/manufacturer) and the
 *      capability / limit region 0x1000-0x11FF (clock, PLL, frame/line length
 *      and binning limits used to compute the mode programs),
 *   4. prints everything to dmesg,
 *   5. powers the sensor back down.
 *
 * It never writes a sensor register and always powers down afterwards, so it
 * is safe to run on the phone. It does nothing until explicitly triggered and
 * is disabled in the device tree by default.
 */
#define pr_fmt(fmt) "nokia-camprobe: " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>
#include <mach/msm_iomap.h>
#include <mach/gpiomux.h>
#include <media/msm_cam_sensor.h>
#include "msm_sensor.h"
#include "cci/msm_cci.h"
#include "io/msm_camera_i2c.h"
#include "io/msm_camera_dt_util.h"

/* Candidate 7-bit CCI addresses, overridable via qcom,probe-addr in the DT. */
static const u16 default_addrs[] = { 0x10, 0x1a, 0x20, 0x36, 0x37, 0x6c, 0x6d };

/* One debugfs directory for all probe nodes, one trigger file per node. */
static struct dentry *nokia_camprobe_root;
#define MAX_ADDRS 16

/* Local copy: the camera_v2 clk table is static to msm_sensor.c. */
static struct msm_cam_clk_info cam_8974_clk_info_local[] = {
	[SENSOR_CAM_MCLK] = { "cam_src_clk", 9600000 },
	[SENSOR_CAM_CLK] = { "cam_clk", 0 },
};

/* Read-only CCI access: only the functions this probe uses are populated. */
static struct msm_camera_i2c_fn_t nokia_camprobe_cci_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_util = msm_sensor_cci_i2c_util,
};

struct nokia_camprobe {
	struct platform_device *pdev;
	struct device_node *of_node;
	struct msm_camera_power_ctrl_t power_info;
	struct msm_camera_i2c_client i2c_client;
	struct msm_camera_cci_client cci_client;
	u32 cci_master;
	u16 addrs[MAX_ADDRS];
	u32 num_addrs;
	struct dentry *dir;
	struct mutex lock;
	struct delayed_work dump_work;
};

/*
 * Bring-up diagnostics: TLMM state of the camera pads and the CCI master 0
 * registers, read while a transfer is pending. Read-only.
 */
#define LUMIA_CCI_PHYS	0xfda0c000
#define TLMM_CFG(n)	(MSM_TLMM_BASE + 0x1000 + 0x10 * (n))
#define TLMM_INOUT(n)	(MSM_TLMM_BASE + 0x1004 + 0x10 * (n))
static void __iomem *lumia_cci_base;
static const u16 lumia_pads[] = { 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 27,
				  62, 83, 84, 89, 90, 91, 92, 100, 101, 116 };
static const u16 lumia_cci_regs[] = { 0x000, 0x100, 0x104, 0x108, 0x10c,
				      0x110, 0x11c, 0x300, 0x304, 0x308,
				      0xc04, 0xc0c };

static void nokia_camprobe_dump_hw(const char *when)
{
	int i;

	pr_info("hw state (%s):\n", when);
	for (i = 0; i < ARRAY_SIZE(lumia_pads); i++) {
		u32 cfg = __raw_readl(TLMM_CFG(lumia_pads[i]));
		u32 io = __raw_readl(TLMM_INOUT(lumia_pads[i]));

		pr_info("  gpio %3u cfg %03x (func %u pull %u drv %u oe %u) in %u out %u\n",
			lumia_pads[i], cfg, (cfg >> 2) & 0xf, cfg & 3,
			(cfg >> 6) & 7, (cfg >> 9) & 1, io & 1, (io >> 1) & 1);
	}
	if (!lumia_cci_base)
		lumia_cci_base = ioremap(LUMIA_CCI_PHYS, 0x1000);
	if (!lumia_cci_base)
		return;
	for (i = 0; i < ARRAY_SIZE(lumia_cci_regs); i++)
		pr_info("  cci[%03x] = %08x\n", lumia_cci_regs[i],
			__raw_readl(lumia_cci_base + lumia_cci_regs[i]));
}

/*
 * scan_all: besides the DT candidates, try every 7-bit address 0x08-0x77
 * (address phase plus a 16-bit register pointer, then a read; nothing is
 * written to any register). extra_high: TLMM GPIOs to drive high after
 * the DT power sequence and low again before power down, to find the
 * rails the PEP does not show (the probe DTB frees GPIO 21/22 from CCI).
 */
static bool scan_all = true;
module_param(scan_all, bool, 0644);
static int extra_high[8];
static int n_extra_high;
module_param_array(extra_high, int, &n_extra_high, 0644);

/* settle_ms: wait after power up before scanning; standby_level: value the
 * DT power sequence drives on the standby GPIO (1 = as in the DT). */
static int settle_ms;
module_param(settle_ms, int, 0644);
static int standby_level = 1;
module_param(standby_level, int, 0644);

/* cci_master: -1 = as in the DT; 1 = scan CCI master 1, with GPIO 21/22
 * muxed to function 1 (pull-up) for the duration of the scan. */
static int cci_master = -1;
module_param(cci_master, int, 0644);

/* hold_ms: keep the module powered this long after its scan, so another
 * module can be scanned with these rails on too. */
static int hold_ms;
module_param(hold_ms, int, 0644);

/* no_rst_stby: like Windows, never drive the reset/standby GPIOs (they are
 * requested as inputs and their power sequence steps become no-ops). */
static bool no_rst_stby;
module_param(no_rst_stby, bool, 0644);

static void nokia_camprobe_set_rst_stby(struct nokia_camprobe *p, bool skip)
{
	struct msm_camera_power_ctrl_t *pw = &p->power_info;
	struct msm_camera_gpio_num_info *ni = pw->gpio_conf->gpio_num_info;
	struct msm_sensor_power_setting *tbl[2] = { pw->power_setting,
						    pw->power_down_setting };
	int sizes[2] = { pw->power_setting_size, pw->power_down_setting_size };
	int t, i;

	for (t = 0; t < 2; t++)
		for (i = 0; i < sizes[t]; i++) {
			struct msm_sensor_power_setting *ps = &tbl[t][i];

			if ((ps->seq_type == SENSOR_GPIO ||
			     ps->seq_type == SENSOR_I2C_MUX) &&
			    (ps->seq_val == SENSOR_GPIO_RESET ||
			     ps->seq_val == SENSOR_GPIO_STANDBY))
				ps->seq_type = skip ? SENSOR_I2C_MUX : SENSOR_GPIO;
		}
	for (i = 0; ni && i < pw->gpio_conf->cam_gpio_req_tbl_size; i++) {
		struct gpio *g = &pw->gpio_conf->cam_gpio_req_tbl[i];

		if (g->gpio == ni->gpio_num[SENSOR_GPIO_RESET] ||
		    g->gpio == ni->gpio_num[SENSOR_GPIO_STANDBY])
			g->flags = skip ? GPIOF_DIR_IN : GPIOF_OUT_INIT_LOW;
	}
}

/*
 * win_order: power up in the Windows PEP order, platform first (CAMP F0:
 * cam_vio/L12 and the vdig GPIO 83 or 18), then cam_vana, then MCLK, then
 * reset/standby. The order is applied once per trigger on a copy.
 */
static bool win_order;
module_param(win_order, bool, 0644);

static int nokia_camprobe_rank(struct nokia_camprobe *p,
			       struct msm_sensor_power_setting *ps)
{
	struct camera_vreg_t *v = p->power_info.cam_vreg;

	if (ps->seq_type == SENSOR_VREG && v &&
	    !strcmp(v[ps->seq_val].reg_name, "cam_vio"))
		return 0;
	if (ps->seq_type == SENSOR_GPIO && ps->seq_val == SENSOR_GPIO_VDIG)
		return 1;
	if (ps->seq_type == SENSOR_VREG)
		return 2;
	if (ps->seq_type == SENSOR_CLK)
		return 3;
	return 4;
}

static void nokia_camprobe_reorder(struct nokia_camprobe *p,
				   struct msm_sensor_power_setting *saved)
{
	struct msm_camera_power_ctrl_t *pw = &p->power_info;
	int n = pw->power_setting_size, i, j, k = 0, r;

	memcpy(saved, pw->power_setting, n * sizeof(*saved));
	if (!win_order)
		return;
	for (r = 0; r <= 4; r++)
		for (i = 0; i < n; i++)
			if (nokia_camprobe_rank(p, &saved[i]) == r)
				pw->power_setting[k++] = saved[i];
	for (j = 0; j < n; j++)
		pr_info("  seq[%d] type %d val %d cfg %ld delay %u\n", j,
			pw->power_setting[j].seq_type,
			pw->power_setting[j].seq_val,
			pw->power_setting[j].config_val,
			pw->power_setting[j].delay);
}

/*
 * extra_vreg: regulators (by name, e.g. 8941_l3) to enable at their DT
 * voltage before the power sequence and disable after power down. The
 * Windows sleep config (\_SB.SDFR F3) turns L3 and L10 off, so they are on
 * while Windows runs although no device votes them; Linux drops them as
 * unused.
 */
static char *extra_vreg[6];
static int n_extra_vreg;
module_param_array(extra_vreg, charp, &n_extra_vreg, 0644);
static struct regulator *extra_reg[6];
/* extra_uv: optional voltage (uV) for each extra_vreg entry, 0 = as is */
static int extra_uv[6];
static int n_extra_uv;
module_param_array(extra_uv, int, &n_extra_uv, 0644);

static void nokia_camprobe_extra_vregs(bool on)
{
	int i;

	for (i = 0; i < n_extra_vreg; i++) {
		if (on) {
			extra_reg[i] = regulator_get(NULL, extra_vreg[i]);
			if (IS_ERR(extra_reg[i])) {
				pr_info("extra vreg %s: get failed %ld\n",
					extra_vreg[i], PTR_ERR(extra_reg[i]));
				extra_reg[i] = NULL;
				continue;
			}
			if (i < n_extra_uv && extra_uv[i] > 0)
				pr_info("extra vreg %s set %d uV: %d\n",
					extra_vreg[i], extra_uv[i],
					regulator_set_voltage(extra_reg[i],
							      extra_uv[i],
							      extra_uv[i]));
			pr_info("extra vreg %s on: %d (%d uV)\n", extra_vreg[i],
				regulator_enable(extra_reg[i]),
				regulator_get_voltage(extra_reg[i]));
		} else if (extra_reg[i]) {
			regulator_disable(extra_reg[i]);
			regulator_put(extra_reg[i]);
			extra_reg[i] = NULL;
		}
	}
}

/* extra_after: raise the extra_high GPIOs after the power sequence (rails
 * and MCLK on), as a reset line would be released, instead of before. */
static bool extra_after;
module_param(extra_after, bool, 0644);

static void nokia_camprobe_extra_gpios_high(void)
{
	int i, rc;

	for (i = 0; i < n_extra_high; i++) {
		rc = gpio_request_one(extra_high[i], GPIOF_OUT_INIT_HIGH,
				      "camprobe_extra");
		pr_info("extra gpio %d high: %s (%d)\n", extra_high[i],
			rc ? "request failed" : "ok", rc);
		if (rc)
			extra_high[i] = -1;
	}
	if (n_extra_high)
		msleep(20);
}

/*
 * spare: value for TLMM_SPARE_REG (0xFD512024) during the scan, -1 = leave.
 * The Windows camera platform driver owns this register (third memory
 * resource of \_SB.CAMP) and writes 0 to it in PrepareHardware; Linux
 * writes 0xf in msm_8974_init_gpiomux().
 */
static int spare = -1;
module_param(spare, int, 0644);

static void nokia_camprobe_dump_work(struct work_struct *work)
{
	nokia_camprobe_dump_hw("transfer pending");
}

static int nokia_camprobe_read(struct nokia_camprobe *p, u32 addr, u16 *val,
			       enum msm_camera_i2c_data_type dt)
{
	return msm_camera_cci_i2c_read(&p->i2c_client, addr, val, dt);
}

static int nokia_camprobe_dump_ids(struct nokia_camprobe *p, u16 sid,
				   bool quiet)
{
	u16 v, model, die_model = 0;
	int rc;

	rc = nokia_camprobe_read(p, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		if (!quiet)
			pr_info("  0x%02x: no ACK (rc %d)\n", sid, rc);
		return rc;
	}
	pr_info("  0x%02x: ACK, module model_id 0x%04x\n", sid, model);

#define RD8(reg, name) do {						\
	if (nokia_camprobe_read(p, (reg), &v,				\
				MSM_CAMERA_I2C_BYTE_DATA) >= 0)		\
		pr_info("    %-14s [0x%04x] = 0x%02x\n", name, (reg), v & 0xff); \
	} while (0)
#define RD16(reg, name) do {						\
	if (nokia_camprobe_read(p, (reg), &v,				\
				MSM_CAMERA_I2C_WORD_DATA) >= 0)		\
		pr_info("    %-14s [0x%04x] = 0x%04x\n", name, (reg), v); \
	} while (0)

	RD8(0x0002, "rev_major");
	RD8(0x0003, "manufacturer");
	RD8(0x0004, "smia_version");
	RD8(0x0010, "rev_minor");
	RD8(0x0011, "smiapp_version");
	/* Die identification (distinct from the module id above). */
	RD16(0x0016, "die_model");
	if (nokia_camprobe_read(p, 0x0016, &die_model,
				MSM_CAMERA_I2C_WORD_DATA) >= 0 && die_model)
		pr_info("    die model_id 0x%04x (silicon, != module)\n",
			die_model);
	RD8(0x0018, "die_rev");
	RD8(0x0019, "die_manufacturer");
	RD16(0x0040, "frame_count");
	/* Current CSI configuration the module boots with. */
	RD8(0x0112, "ccp_data_fmt_h");
	RD8(0x0113, "ccp_data_fmt_l");
	RD8(0x0114, "csi_lane_mode");

	pr_info("    capability/limit region 0x1000-0x11ff:\n");
	{
		u32 base;
		for (base = 0x1000; base < 0x1200; base += 16) {
			u8 line[16];
			int i, any = 0;
			for (i = 0; i < 16; i++) {
				if (nokia_camprobe_read(p, base + i, &v,
						MSM_CAMERA_I2C_BYTE_DATA) < 0) {
					line[i] = 0;
					continue;
				}
				line[i] = v & 0xff;
				if (line[i])
					any = 1;
			}
			if (any)
				pr_info("      %04x: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
					base, line[0], line[1], line[2], line[3],
					line[4], line[5], line[6], line[7],
					line[8], line[9], line[10], line[11],
					line[12], line[13], line[14], line[15]);
		}
	}
#undef RD8
#undef RD16
	return 0;
}

static int nokia_camprobe_run(struct nokia_camprobe *p)
{
	u32 cfg21 = 0, cfg22 = 0;
	int spare_old;
	struct msm_sensor_power_setting saved_seq[16];
	int rc, i;

	mutex_lock(&p->lock);
	for (i = 0; i < p->power_info.power_setting_size; i++) {
		struct msm_sensor_power_setting *ps =
			&p->power_info.power_setting[i];

		if (ps->seq_type == SENSOR_GPIO &&
		    ps->seq_val == SENSOR_GPIO_STANDBY)
			ps->config_val = standby_level ? 1 : 0;
	}
	nokia_camprobe_set_rst_stby(p, no_rst_stby);
	pr_info("power up (%u candidate address(es), cci master %u, standby %d, settle %d ms, rst/stby %s)\n",
		p->num_addrs, p->cci_master, standby_level, settle_ms,
		no_rst_stby ? "not driven" : "driven");
	if (!extra_after)
		nokia_camprobe_extra_gpios_high();
	spare_old = msm_tlmm_misc_reg_read(TLMM_SPARE_REG);
	if (spare >= 0) {
		msm_tlmm_misc_reg_write(TLMM_SPARE_REG, spare);
		mb();
	}
	pr_info("TLMM_SPARE_REG was 0x%x, now 0x%x\n", spare_old,
		msm_tlmm_misc_reg_read(TLMM_SPARE_REG));
	nokia_camprobe_extra_vregs(true);
	if (n_extra_vreg)
		msleep(5);
	nokia_camprobe_reorder(p, saved_seq);
	rc = msm_camera_power_up(&p->power_info, MSM_CAMERA_PLATFORM_DEVICE,
				 &p->i2c_client);
	if (rc < 0) {
		pr_err("power up failed: %d\n", rc);
		msm_tlmm_misc_reg_write(TLMM_SPARE_REG, spare_old);
		nokia_camprobe_extra_vregs(false);
		memcpy(p->power_info.power_setting, saved_seq,
		       p->power_info.power_setting_size * sizeof(saved_seq[0]));
		for (i = 0; i < n_extra_high; i++)
			if (extra_high[i] >= 0)
				gpio_free(extra_high[i]);
		mutex_unlock(&p->lock);
		return rc;
	}

	/* msm_camera_power_up() already ran MSM_CCI_INIT for the platform device. */

	if (extra_after)
		nokia_camprobe_extra_gpios_high();
	if (cci_master >= 0)
		p->cci_client.cci_i2c_master = cci_master;
	if (cci_master == 1) {
		cfg21 = __raw_readl(TLMM_CFG(21));
		cfg22 = __raw_readl(TLMM_CFG(22));
		__raw_writel((1 << 2) | 3, TLMM_CFG(21));
		__raw_writel((1 << 2) | 3, TLMM_CFG(22));
		mb();
		pr_info("cci master 1: gpio 21/22 function 1, pull-up\n");
	}
	if (settle_ms > 0)
		msleep(settle_ms);

	nokia_camprobe_dump_hw("after power up");
	for (i = 0; i < p->num_addrs; i++) {
		p->cci_client.sid = p->addrs[i];      /* 7-bit */
		p->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		nokia_camprobe_dump_ids(p, p->addrs[i], false);
	}
	if (scan_all) {
		int found = 0;

		/*
		 * One pointer byte (register 0x00) and a one-byte read: no data
		 * byte ever reaches a device we do not know, whatever its
		 * register width.
		 */
		pr_info("full scan 0x08-0x77 (8-bit pointer 0x00, read only):\n");
		for (i = 0x08; i <= 0x77; i++) {
			u16 v;

			p->cci_client.sid = i;
			p->i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
			if (nokia_camprobe_read(p, 0x00, &v,
						MSM_CAMERA_I2C_BYTE_DATA) < 0)
				continue;
			pr_info("  0x%02x: ACK, [0x00] = 0x%02x\n", i, v & 0xff);
			found++;
		}
		p->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		pr_info("full scan done: %d address(es) answered\n", found);
	}

	if (cci_master == 1) {
		__raw_writel(cfg21, TLMM_CFG(21));
		__raw_writel(cfg22, TLMM_CFG(22));
		mb();
	}
	p->cci_client.cci_i2c_master = p->cci_master;
	if (hold_ms > 0) {
		pr_info("holding power for %d ms\n", hold_ms);
		msleep(hold_ms);
	}
	msm_camera_power_down(&p->power_info, MSM_CAMERA_PLATFORM_DEVICE,
			      &p->i2c_client);
	for (i = 0; i < n_extra_high; i++) {
		if (extra_high[i] < 0)
			continue;
		gpio_set_value(extra_high[i], 0);
		gpio_free(extra_high[i]);
	}
	memcpy(p->power_info.power_setting, saved_seq,
	       p->power_info.power_setting_size * sizeof(saved_seq[0]));
	nokia_camprobe_extra_vregs(false);
	msm_tlmm_misc_reg_write(TLMM_SPARE_REG, spare_old);
	pr_info("power down done\n");
	mutex_unlock(&p->lock);
	return 0;
}

static ssize_t nokia_camprobe_trigger(struct file *file,
				      const char __user *buf,
				      size_t len, loff_t *ppos)
{
	struct nokia_camprobe *p = file->private_data;
	int rc = nokia_camprobe_run(p);

	return rc < 0 ? rc : len;
}

static const struct file_operations nokia_camprobe_fops = {
	.open = simple_open,
	.write = nokia_camprobe_trigger,
	.llseek = noop_llseek,
};

static int nokia_camprobe_parse_dt(struct nokia_camprobe *p)
{
	struct device_node *np = p->of_node;
	struct msm_camera_power_ctrl_t *pw = &p->power_info;
	struct msm_camera_gpio_conf *gconf;
	u16 *gpio_array = NULL;
	u16 gpio_array_size, i;
	u32 tmp[MAX_ADDRS];
	struct property *prop;
	int rc, cnt = 0;

	if (of_property_read_u32(np, "qcom,cci-master", &p->cci_master))
		p->cci_master = MASTER_0;

	prop = of_find_property(np, "qcom,probe-addr", NULL);
	if (prop)
		cnt = prop->length / sizeof(u32);
	if (cnt > 0 && cnt <= MAX_ADDRS &&
	    !of_property_read_u32_array(np, "qcom,probe-addr", tmp, cnt)) {
		for (i = 0; i < cnt; i++)
			p->addrs[i] = tmp[i];
		p->num_addrs = cnt;
	} else {
		for (i = 0; i < ARRAY_SIZE(default_addrs); i++)
			p->addrs[i] = default_addrs[i];
		p->num_addrs = ARRAY_SIZE(default_addrs);
	}

	rc = msm_camera_get_dt_vreg_data(np, &pw->cam_vreg, &pw->num_vreg);
	if (rc < 0)
		return rc;
	rc = msm_camera_get_dt_power_setting_data(np, pw->cam_vreg,
						  pw->num_vreg, pw);
	if (rc < 0)
		goto free_vreg;

	pw->gpio_conf = kzalloc(sizeof(*gconf), GFP_KERNEL);
	if (!pw->gpio_conf) {
		rc = -ENOMEM;
		goto free_ps;
	}
	gconf = pw->gpio_conf;
	gpio_array_size = of_gpio_count(np);
	if (gpio_array_size) {
		gpio_array = kcalloc(gpio_array_size, sizeof(u16), GFP_KERNEL);
		if (!gpio_array) {
			rc = -ENOMEM;
			goto free_gconf;
		}
		for (i = 0; i < gpio_array_size; i++)
			gpio_array[i] = of_get_gpio(np, i);
		rc = msm_camera_get_dt_gpio_req_tbl(np, gconf, gpio_array,
						    gpio_array_size);
		if (rc < 0)
			goto free_gpio;
		rc = msm_camera_get_dt_gpio_set_tbl(np, gconf, gpio_array,
						    gpio_array_size);
		if (rc < 0)
			goto free_gpio;
		rc = msm_camera_init_gpio_pin_tbl(np, gconf, gpio_array,
						  gpio_array_size);
		if (rc < 0)
			goto free_gpio;
	}
	kfree(gpio_array);

	/* MCLK 9.6 MHz by default; qcom,mclk-rate overrides it (e.g. 24 MHz). */
	pw->clk_info = kzalloc(sizeof(cam_8974_clk_info_local), GFP_KERNEL);
	if (!pw->clk_info) {
		rc = -ENOMEM;
		goto free_gconf;
	}
	memcpy(pw->clk_info, cam_8974_clk_info_local,
	       sizeof(cam_8974_clk_info_local));
	pw->clk_info_size = ARRAY_SIZE(cam_8974_clk_info_local);
	if (of_property_read_u32(np, "qcom,mclk-rate",
				 (u32 *)&pw->clk_info[0].clk_rate))
		pw->clk_info[0].clk_rate = 9600000;
	pw->dev = &p->pdev->dev;
	return 0;

free_gpio:
	kfree(gpio_array);
free_gconf:
	kfree(pw->gpio_conf);
free_ps:
	kfree(pw->power_setting);
	kfree(pw->power_down_setting);
free_vreg:
	kfree(pw->cam_vreg);
	return rc;
}

static int nokia_camprobe_probe(struct platform_device *pdev)
{
	struct nokia_camprobe *p;
	const char *name;
	int rc;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->pdev = pdev;
	p->of_node = pdev->dev.of_node;
	mutex_init(&p->lock);
	INIT_DELAYED_WORK(&p->dump_work, nokia_camprobe_dump_work);
	platform_set_drvdata(pdev, p);

	rc = nokia_camprobe_parse_dt(p);
	if (rc < 0) {
		dev_err(&pdev->dev, "DT parse failed: %d\n", rc);
		return rc;
	}

	p->i2c_client.cci_client = &p->cci_client;
	p->cci_client.cci_subdev = msm_cci_get_subdev();
	p->cci_client.cci_i2c_master = p->cci_master;
	p->cci_client.retries = 3;
	p->cci_client.id_map = 0;
	p->i2c_client.i2c_func_tbl = &nokia_camprobe_cci_tbl;
	p->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	if (of_property_read_string(p->of_node, "qcom,sensor-name", &name))
		name = dev_name(&pdev->dev);
	if (IS_ERR_OR_NULL(nokia_camprobe_root))
		nokia_camprobe_root = debugfs_create_dir("nokia_camprobe", NULL);
	if (!IS_ERR_OR_NULL(nokia_camprobe_root))
		p->dir = debugfs_create_file(name, 0200, nokia_camprobe_root,
					     p, &nokia_camprobe_fops);

	dev_info(&pdev->dev,
		 "ready: echo 1 > /sys/kernel/debug/nokia_camprobe/%s\n", name);
	return 0;
}

static int nokia_camprobe_remove(struct platform_device *pdev)
{
	struct nokia_camprobe *p = platform_get_drvdata(pdev);

	debugfs_remove_recursive(p->dir);
	return 0;
}

static const struct of_device_id nokia_camprobe_dt_match[] = {
	{ .compatible = "nokia,camera-probe" },
	{ }
};
MODULE_DEVICE_TABLE(of, nokia_camprobe_dt_match);

static struct platform_driver nokia_camprobe_driver = {
	.probe = nokia_camprobe_probe,
	.remove = nokia_camprobe_remove,
	.driver = {
		.name = "nokia-camprobe",
		.owner = THIS_MODULE,
		.of_match_table = nokia_camprobe_dt_match,
	},
};
module_platform_driver(nokia_camprobe_driver);

MODULE_DESCRIPTION("Nokia Lumia MSM8974 camera sensor identification probe");
MODULE_LICENSE("GPL v2");
