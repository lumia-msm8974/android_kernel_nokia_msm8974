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
#include <media/msm_cam_sensor.h>
#include "msm_sensor.h"
#include "cci/msm_cci.h"
#include "io/msm_camera_i2c.h"
#include "io/msm_camera_dt_util.h"

/* Candidate 7-bit CCI addresses, overridable via qcom,probe-addr in the DT. */
static const u16 default_addrs[] = { 0x10, 0x1a, 0x20, 0x36, 0x37, 0x6c, 0x6d };
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
};

static int nokia_camprobe_read(struct nokia_camprobe *p, u32 addr, u16 *val,
			       enum msm_camera_i2c_data_type dt)
{
	return msm_camera_cci_i2c_read(&p->i2c_client, addr, val, dt);
}

static void nokia_camprobe_dump_ids(struct nokia_camprobe *p, u16 sid)
{
	u16 v, model, die_model = 0;
	int rc;

	rc = nokia_camprobe_read(p, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_info("  0x%02x: no ACK (rc %d)\n", sid, rc);
		return;
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
}

static int nokia_camprobe_run(struct nokia_camprobe *p)
{
	int rc, i;

	mutex_lock(&p->lock);
	pr_info("power up (%u candidate address(es), cci master %u)\n",
		p->num_addrs, p->cci_master);
	rc = msm_camera_power_up(&p->power_info, MSM_CAMERA_PLATFORM_DEVICE,
				 &p->i2c_client);
	if (rc < 0) {
		pr_err("power up failed: %d\n", rc);
		mutex_unlock(&p->lock);
		return rc;
	}

	/* The CCI itself is brought up through the i2c_util INIT command. */
	rc = p->i2c_client.i2c_func_tbl->i2c_util(&p->i2c_client, MSM_CCI_INIT);
	if (rc < 0)
		pr_err("CCI init failed: %d (addresses may still answer)\n", rc);

	for (i = 0; i < p->num_addrs; i++) {
		p->cci_client.sid = p->addrs[i];      /* 7-bit */
		p->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		nokia_camprobe_dump_ids(p, p->addrs[i]);
	}

	p->i2c_client.i2c_func_tbl->i2c_util(&p->i2c_client, MSM_CCI_RELEASE);
	msm_camera_power_down(&p->power_info, MSM_CAMERA_PLATFORM_DEVICE,
			      &p->i2c_client);
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
	int rc;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->pdev = pdev;
	p->of_node = pdev->dev.of_node;
	mutex_init(&p->lock);
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

	p->dir = debugfs_create_dir("nokia_camprobe", NULL);
	if (!IS_ERR_OR_NULL(p->dir))
		debugfs_create_file("scan", 0200, p->dir, p,
				    &nokia_camprobe_fops);

	dev_info(&pdev->dev,
		 "ready: echo 1 > /sys/kernel/debug/nokia_camprobe/scan\n");
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
