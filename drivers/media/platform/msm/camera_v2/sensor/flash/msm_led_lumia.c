/*
 * Rear flash of the MSM8974 Lumias (930 "martini", 1520 "bandit") for the
 * msm camera daemon (MSM_CAMERA_SUBDEV_LED_FLASH).
 *
 * The flash driver IC sits on CCI master 0 at 0x30 (7-bit) and is powered
 * with the camera (GPIO 83, the sensor's "vdig" GPIO). Register 0x00 reads
 * 0x64 (0x34 on other units), 0x01 is its revision. The module settings
 * program it with one burst of registers 0x02-0x07:
 *   torch (video light):  00 00 00 ff 01 0a
 *   flash, low (preflash): 04 00 00 TT 01 0b
 *   flash, main:          0b b0 LL TT 01 0b
 * TT is the timeout, (ms - 2) / 2 clamped to 0..255; LL depends on the LED
 * module read from the IC memory (0x74 for module type 3, 0x7c for type 2,
 * 0 otherwise). Register 0x06 bits 7:6 select the strobe source (0 = start
 * on the register write, used here). 0x07 = 0x03 puts the IC back to
 * standby; 0x08 holds the fault flags. Chips 0x6x first get 0x07 = 0x02.
 * GPIO 84 is the IC's torch/strobe input; it is not driven here.
 *
 * The camera daemon keeps LED_HIGH until the snapshot frame is out, so the
 * flash timeout (flash_ms, default 200 ms) is much longer than on the
 * original platform, which fires synchronised pulses of at most 40 ms
 * (main) and 80 ms (pre). For that reason the main currents are off by
 * default (flash_main=0): LED_HIGH uses the preflash setting until the
 * currents have been measured.
 *
 * Copyright (c) 2026, Lorenzo Giannoccaro. GPLv2 only.
 */

#define pr_fmt(fmt) "lumia-flash: " fmt

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <media/v4l2-subdev.h>
#include <media/msm_cam_sensor.h>
#include <mach/camera2.h>
#include "msm_camera_i2c.h"
#include "msm_cci.h"
#include "msm_sd.h"

#define FLASH_NAME "nokia,lumia-camera-flash"

static bool flash_main;
module_param(flash_main, bool, 0644);
MODULE_PARM_DESC(flash_main, "LED_HIGH with the main flash currents (else preflash)");
static int flash_ms = 200;
module_param(flash_ms, int, 0644);
MODULE_PARM_DESC(flash_ms, "flash timeout for LED_HIGH, ms (2-512)");

struct lumia_flash {
	struct msm_sd_subdev msm_sd;
	struct platform_device *pdev;
	struct msm_camera_i2c_client i2c;
	struct msm_camera_cci_client cci;
	struct mutex lock;
	int state;
	u8 id, rev, led_type, led_info_read;
	bool cci_on;
};

static struct lumia_flash lf;

static struct msm_camera_i2c_fn_t lumia_flash_cci_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll = msm_camera_cci_i2c_poll,
};

static int wr(u8 reg, u8 val)
{
	return lf.i2c.i2c_func_tbl->i2c_write(&lf.i2c, reg, val,
		MSM_CAMERA_I2C_BYTE_DATA);
}

static int rd(u8 reg, u8 *val)
{
	uint16_t v = 0;
	int rc = lf.i2c.i2c_func_tbl->i2c_read(&lf.i2c, reg, &v,
		MSM_CAMERA_I2C_BYTE_DATA);

	*val = v;
	return rc;
}

static int burst(u8 reg, const u8 *val, int n)
{
	return lf.i2c.i2c_func_tbl->i2c_write_seq(&lf.i2c, reg, (u8 *)val, n);
}

/*
 * LED module type from the IC memory, as the original driver does at every
 * power-up: 0x07 = 1 then 9 (memory access), 0x0c-0x0f = {0x10, 0x11, 0,
 * 0xa7}, wait, 0x09 until bits 3:0 clear, data in 0x10/0x11, 0x07 = 3.
 */
static void lumia_flash_read_led_info(void)
{
	static const u8 cmd[4] = { 0x10, 0x11, 0x00, 0xa7 };
	u8 v = 0, d0 = 0, d1 = 0;
	int i, rc;

	rc = wr(0x06, 0x00);
	if (rc >= 0)
		rc = rd(0x07, &v);
	if (rc >= 0 && v != 0x09) {
		rc = wr(0x07, 0x01);
		if (rc >= 0)
			rc = wr(0x07, 0x09);
	}
	if (rc >= 0)
		rc = burst(0x0c, cmd, 4);
	usleep_range(2200, 2400);
	for (i = 0; rc >= 0 && i < 20; i++) {
		rc = rd(0x09, &v);
		if (rc < 0 || !(v & 0x0f) || (v & 0x07))
			break;
		usleep_range(500, 600);
	}
	if (rc >= 0 && !(v & 0x0f)) {
		rd(0x10, &d0);
		rd(0x11, &d1);
		lf.led_type = d1 & 0x0f;
	}
	wr(0x07, 0x03);
	lf.led_info_read = 1;
	pr_info("IC 0x%02x rev 0x%02x, LED data %02x %02x (type %u, status 0x%02x)\n",
		lf.id, lf.rev, d0, d1, lf.led_type, v);
}

/* The daemon sends LED_INIT before the sensor powers up: the CCI master has
 * to be brought up here, and dropped again on release. */
static int lumia_flash_cci(bool on)
{
	int rc;

	if (lf.cci_on == on)
		return 0;
	rc = msm_sensor_cci_i2c_util(&lf.i2c, on ? MSM_CCI_INIT : MSM_CCI_RELEASE);
	if (rc < 0) {
		pr_err("CCI %s failed: %d\n", on ? "init" : "release", rc);
		return rc;
	}
	lf.cci_on = on;
	return 0;
}

static int lumia_flash_init(void)
{
	u8 v = 0;
	int rc;

	rc = lumia_flash_cci(true);
	if (rc < 0)
		return rc;
	rc = rd(0x00, &lf.id);
	if (rc < 0)
		return rc;
	if (lf.id != 0x64 && lf.id != 0x34) {
		pr_err("unknown flash IC id 0x%02x\n", lf.id);
		return -ENODEV;
	}
	rd(0x01, &lf.rev);
	if (!lf.led_info_read)
		lumia_flash_read_led_info();
	rd(0x08, &v);			/* clears the fault flags */
	if (v)
		pr_info("faults at init: 0x%02x\n", v);
	return 0;
}

static int lumia_flash_fire(int torch)
{
	u8 r[6];
	int ms = clamp(flash_ms, 2, 512);
	int rc = 0;

	if ((lf.id & 0xf0) == 0x60)
		rc = wr(0x07, 0x02);
	if (torch) {
		r[0] = 0x00; r[1] = 0x00; r[2] = 0x00;
		r[3] = 0xff; r[4] = 0x01; r[5] = 0x0a;
	} else {
		r[0] = flash_main ? 0x0b : 0x04;
		r[1] = flash_main ? 0xb0 : 0x00;
		r[2] = !flash_main ? 0 :
			lf.led_type == 3 ? 0x74 : lf.led_type == 2 ? 0x7c : 0;
		r[3] = ms <= 2 ? 0 : min((ms - 2) / 2, 255);
		r[4] = 0x01;
		r[5] = 0x0b;
	}
	if (rc >= 0)
		rc = burst(0x02, r, 6);
	return rc;
}

static int lumia_flash_off(void)
{
	u8 v = 0;
	int rc = wr(0x07, 0x03);

	if (rd(0x08, &v) >= 0 && v)
		pr_info("faults: 0x%02x\n", v);
	return rc;
}

static int lumia_flash_config(struct msm_camera_led_cfg_t *cfg)
{
	int rc = 0;

	mutex_lock(&lf.lock);
	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_INIT:
		rc = lumia_flash_init();
		break;
	case MSM_CAMERA_LED_LOW:
		rc = lumia_flash_cci(true);
		if (rc >= 0)
			rc = lumia_flash_fire(1);
		break;
	case MSM_CAMERA_LED_HIGH:
		rc = lumia_flash_cci(true);
		if (rc >= 0)
			rc = lumia_flash_fire(0);
		break;
	case MSM_CAMERA_LED_OFF:
	case MSM_CAMERA_LED_RELEASE:
		if (lf.cci_on && (lf.state == MSM_CAMERA_LED_LOW ||
		    lf.state == MSM_CAMERA_LED_HIGH ||
		    cfg->cfgtype == MSM_CAMERA_LED_OFF))
			rc = lumia_flash_off();
		if (cfg->cfgtype == MSM_CAMERA_LED_RELEASE)
			lumia_flash_cci(false);
		break;
	default:
		rc = -EINVAL;
	}
	if (rc < 0)
		pr_err("cfg %d: %d\n", cfg->cfgtype, rc);
	else
		lf.state = cfg->cfgtype;
	mutex_unlock(&lf.lock);
	/* The daemon treats a failing flash as fatal: report, do not fail. */
	return 0;
}

static long lumia_flash_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
	void *arg)
{
	struct msm_camera_led_cfg_t cfg;

	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		*(uint32_t *)arg = lf.pdev->id;
		return 0;
	case VIDIOC_MSM_FLASH_LED_DATA_CFG:
		return lumia_flash_config(arg);
	case MSM_SD_SHUTDOWN:
		memset(&cfg, 0, sizeof(cfg));
		cfg.cfgtype = MSM_CAMERA_LED_RELEASE;
		return lumia_flash_config(&cfg);
	default:
		return -ENOIOCTLCMD;
	}
}

static struct v4l2_subdev_core_ops lumia_flash_core_ops = {
	.ioctl = lumia_flash_ioctl,
};

static struct v4l2_subdev_ops lumia_flash_ops = {
	.core = &lumia_flash_core_ops,
};

static const struct v4l2_subdev_internal_ops lumia_flash_internal_ops;

static int lumia_flash_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	u32 master = 0, sid = 0x30;
	int rc;

	rc = of_property_read_u32(np, "cell-index", &pdev->id);
	if (rc < 0)
		return rc;
	of_property_read_u32(np, "qcom,cci-master", &master);
	of_property_read_u32(np, "nokia,cci-sid", &sid);

	lf.pdev = pdev;
	mutex_init(&lf.lock);
	lf.cci.cci_subdev = msm_cci_get_subdev();
	lf.cci.cci_i2c_master = master;
	lf.cci.sid = sid;
	lf.cci.retries = 2;
	lf.i2c.cci_client = &lf.cci;
	lf.i2c.i2c_func_tbl = &lumia_flash_cci_tbl;
	lf.i2c.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;

	v4l2_subdev_init(&lf.msm_sd.sd, &lumia_flash_ops);
	v4l2_set_subdevdata(&lf.msm_sd.sd, &lf);
	lf.msm_sd.sd.internal_ops = &lumia_flash_internal_ops;
	lf.msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(lf.msm_sd.sd.name, ARRAY_SIZE(lf.msm_sd.sd.name), "msm_flash");
	media_entity_init(&lf.msm_sd.sd.entity, 0, NULL, 0);
	lf.msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	lf.msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LED_FLASH;
	lf.msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x1;
	msm_sd_register(&lf.msm_sd);
	pr_info("registered, CCI master %u at 0x%02x\n", master, sid);
	return 0;
}

static const struct of_device_id lumia_flash_dt_match[] = {
	{ .compatible = FLASH_NAME },
	{ }
};
MODULE_DEVICE_TABLE(of, lumia_flash_dt_match);

static struct platform_driver lumia_flash_driver = {
	.probe = lumia_flash_probe,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lumia_flash_dt_match,
	},
};

static int __init lumia_flash_add_driver(void)
{
	return platform_driver_register(&lumia_flash_driver);
}
module_init(lumia_flash_add_driver);
MODULE_DESCRIPTION("Lumia MSM8974 rear camera flash");
MODULE_LICENSE("GPL v2");
