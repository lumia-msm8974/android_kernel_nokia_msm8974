/*
 * Rear camera module of the MSM8974 Lumias (930 "martini", 1520 "bandit"):
 * focus lens behind the sensor's serial master, optical stabilisation
 * controller on the CCI.
 *
 * Focus. The VCM driver of the module is not on the CCI. The sensor (SMIA++,
 * module 0x4541) relays one-byte writes to it through its serial master:
 *   0x3484 = slave address (8-bit, 0x1c)   0x3480 = 0x14 (start)
 *   0x3485 = register   0x3486 = data   0x3487 = 1 (send)   per byte
 *   0x3485 = 0, 0x3480 = 0 (stop)
 * The 10-bit position goes to registers 3 (bits 9:8) and 4 (bits 7:0), MSB
 * first. With "nokia,af-sensor-bridge" every DAC code of msm_actuator becomes
 * these ten sensor writes; the camera daemon library describes a plain 10-bit
 * DAC on the sensor address (word address, byte data).
 *
 * Stabilisation. The controller (CCI 0x3e) needs a firmware download through
 * a SMIA++-style data transfer window (control 0x0581, status 0x0582, page
 * 0x0583, 64 data bytes from 0x0584), then the module's init block, then one
 * of the mode blocks. Everything comes from a firmware file built from the
 * module settings (see make_ois_blob.py); this file only replays it. A record
 * with op 1 polls its register until it reads non-zero (100 x 1 ms).
 * The download runs in a work item, in parallel with the first frames, like
 * on the original platform; until it ends the lens is left as it is.
 *
 * Copyright (c) 2026, Lorenzo Giannoccaro. GPLv2 only.
 */

#define pr_fmt(fmt) "lumia-af: " fmt

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/slab.h>
#include "msm_sd.h"
#include "msm_actuator.h"

#define LUMIA_BRIDGE_WRITES	10
#define LUMIA_DAC_MAX		1023

/* 0: leave the controller alone, 1: download and run the viewfinder mode. */
static int lumia_ois = 1;
module_param(lumia_ois, int, 0644);
MODULE_PARM_DESC(lumia_ois, "rear OIS: 0 off, 1 on (viewfinder mode)");

/* Mode block applied once the controller is ready: 1 vf, 2 still, 3 video, 4 off. */
static int lumia_ois_mode = 1;
module_param(lumia_ois_mode, int, 0644);

enum { OIS_COLD, OIS_LOADING, OIS_READY, OIS_FAILED };
enum { BLK_INIT, BLK_VF, BLK_STILL, BLK_VIDEO, BLK_OFF, BLK_DEINIT, BLK_NUM };

struct lois_hdr {
	char magic[4];
	__le16 version, hdr_size;
	u8 dev_addr, reg_bytes;
	__le16 dti_ctrl, dti_data, page_size;
	__le16 first_page, delay_before, delay_after, rsvd0;
	__le32 fw_off, fw_size, module_id, rsvd1;
	struct {
		__le32 off;
		__le16 count, rsvd;
	} blk[BLK_NUM];
} __packed;

struct lois_rec {
	__le16 addr;
	u8 val, op;
} __packed;

void msm_actuator_lumia_parse_dt(struct msm_actuator_ctrl_t *a,
	struct device_node *of_node)
{
	u32 val;

	if (!of_property_read_u32(of_node, "nokia,af-sensor-bridge", &val)) {
		a->lumia_bridge = 1;
		a->lumia_bridge_slave = val;
		a->lumia_sensor_sid = 0x10;
		if (!of_property_read_u32(of_node, "nokia,af-sensor-sid", &val))
			a->lumia_sensor_sid = val;
		pr_info("lens driver 0x%02x behind sensor 0x%02x\n",
			a->lumia_bridge_slave, a->lumia_sensor_sid);
	}
	if (of_property_read_bool(of_node, "nokia,ois")) {
		a->ois_present = 1;
		a->ois_client.i2c_func_tbl = a->i2c_client.i2c_func_tbl;
		a->ois_cci.cci_subdev = msm_cci_get_subdev();
		a->ois_cci.cci_i2c_master = a->cci_master;
		a->ois_cci.retries = 3;
		a->ois_cci.id_map = 0;
		a->ois_client.cci_client = &a->ois_cci;
		a->ois_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		mutex_init(&a->ois_lock);
		INIT_WORK(&a->ois_work, msm_actuator_lumia_ois_work);
	}
}

/* Entries msm_actuator must allocate per lens code. */
uint32_t msm_actuator_lumia_tbl_mult(struct msm_actuator_ctrl_t *a)
{
	return a->lumia_bridge ? LUMIA_BRIDGE_WRITES : 1;
}

/* Called once the set_info of the daemon has been copied in. */
void msm_actuator_lumia_fixup(struct msm_actuator_ctrl_t *a)
{
	if (!a->lumia_bridge)
		return;
	if (a->act_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		a->i2c_client.cci_client->sid = a->lumia_sensor_sid;
	a->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	a->i2c_data_type = MSM_ACTUATOR_BYTE_DATA;
}

void msm_actuator_lumia_bridge_append(struct msm_actuator_ctrl_t *a,
	int16_t code, uint16_t delay)
{
	struct msm_camera_i2c_reg_array *t;
	uint16_t c = code < 0 ? 0 : (code > LUMIA_DAC_MAX ? LUMIA_DAC_MAX : code);
	const uint16_t seq[LUMIA_BRIDGE_WRITES][2] = {
		{ 0x3484, a->lumia_bridge_slave }, { 0x3480, 0x14 },
		{ 0x3485, 0x03 }, { 0x3486, c >> 8 }, { 0x3487, 0x01 },
		{ 0x3485, 0x04 }, { 0x3486, c & 0xff }, { 0x3487, 0x01 },
		{ 0x3485, 0x00 }, { 0x3480, 0x00 },
	};
	int i;

	if (!a->i2c_reg_tbl ||
	    a->i2c_tbl_index + LUMIA_BRIDGE_WRITES > a->i2c_tbl_size) {
		pr_err("lens table full (%u/%u)\n", a->i2c_tbl_index,
			a->i2c_tbl_size);
		return;
	}
	t = &a->i2c_reg_tbl[a->i2c_tbl_index];
	for (i = 0; i < LUMIA_BRIDGE_WRITES; i++) {
		t[i].reg_addr = seq[i][0];
		t[i].reg_data = seq[i][1];
		t[i].delay = 0;
	}
	t[LUMIA_BRIDGE_WRITES - 1].delay = delay;
	a->i2c_tbl_index += LUMIA_BRIDGE_WRITES;
	a->lumia_last_code = c;
}

/* Stabilisation controller --------------------------------------------- */

static int ois_wr(struct msm_actuator_ctrl_t *a, u16 reg, u8 val)
{
	return a->ois_client.i2c_func_tbl->i2c_write(&a->ois_client, reg, val,
		MSM_CAMERA_I2C_BYTE_DATA);
}

static int ois_rd(struct msm_actuator_ctrl_t *a, u16 reg, u8 *val)
{
	uint16_t v = 0;
	int rc = a->ois_client.i2c_func_tbl->i2c_read(&a->ois_client, reg, &v,
		MSM_CAMERA_I2C_BYTE_DATA);

	*val = v;
	return rc;
}

/* Polls a register until it reads non-zero, 100 x 1 ms like the module driver. */
static int ois_poll_ready(struct msm_actuator_ctrl_t *a, u16 reg)
{
	int i, rc;
	u8 v = 0;

	for (i = 0; i < 100; i++) {
		rc = ois_rd(a, reg, &v);
		if (rc < 0)
			return rc;
		if (v)
			return 0;
		usleep_range(1000, 1100);
	}
	pr_err("0x%04x stayed 0\n", reg);
	return -ETIMEDOUT;
}

static int ois_run_block(struct msm_actuator_ctrl_t *a, const struct firmware *fw,
	int blk)
{
	const struct lois_hdr *h = (const struct lois_hdr *)fw->data;
	const struct lois_rec *r;
	u32 off = le32_to_cpu(h->blk[blk].off);
	u16 n = le16_to_cpu(h->blk[blk].count), i;
	int rc = 0;

	if (!n)
		return 0;
	r = (const struct lois_rec *)(fw->data + off);
	for (i = 0; i < n && rc >= 0; i++) {
		u16 reg = le16_to_cpu(r[i].addr);

		if (r[i].op == 1)
			rc = ois_poll_ready(a, reg);
		else
			rc = ois_wr(a, reg, r[i].val);
	}
	if (rc < 0)
		pr_err("block %d failed at record %u: %d\n", blk, i - 1, rc);
	return rc;
}

static int ois_dti_wait(struct msm_actuator_ctrl_t *a, u16 status)
{
	int i, rc;
	u8 v = 0;

	for (i = 0; i < 200; i++) {
		rc = ois_rd(a, status, &v);
		if (rc < 0)
			return rc;
		if (v & 0x02)
			break;
		usleep_range(1000, 1100);
	}
	if (v & 0x0c) {
		pr_err("data transfer error, status 0x%02x\n", v);
		return -EIO;
	}
	return (v & 0x02) ? 0 : -ETIMEDOUT;
}

/* Firmware download through the data transfer window (0x0580-0x05c3). */
static int ois_download(struct msm_actuator_ctrl_t *a, const struct firmware *fw)
{
	const struct lois_hdr *h = (const struct lois_hdr *)fw->data;
	const u8 *p = fw->data + le32_to_cpu(h->fw_off);
	u32 left = le32_to_cpu(h->fw_size);
	u16 ctrl = le16_to_cpu(h->dti_ctrl), data = le16_to_cpu(h->dti_data);
	u16 psz = le16_to_cpu(h->page_size), page = le16_to_cpu(h->first_page);
	bool poll;
	u8 cap = 0;
	int rc;

	if (h->delay_before)
		msleep(le16_to_cpu(h->delay_before));
	rc = ois_rd(a, ctrl - 1, &cap);
	if (rc < 0)
		return rc;
	poll = cap & 0x08;
	rc = ois_wr(a, ctrl, 0x03);		/* enable, write */
	while (rc >= 0 && left) {
		u32 n = min_t(u32, left, psz), done = 0;

		if (poll && page != le16_to_cpu(h->first_page)) {
			rc = ois_dti_wait(a, ctrl + 1);
			if (rc < 0)
				break;
		}
		rc = ois_wr(a, ctrl + 2, page);
		while (rc >= 0 && done < n) {
			u32 k = min_t(u32, n - done, 16);

			rc = a->ois_client.i2c_func_tbl->i2c_write_seq(
				&a->ois_client, data + done, (u8 *)p + done, k);
			done += k;
		}
		p += n;
		left -= n;
		page++;
	}
	if (rc >= 0)
		rc = ois_dti_wait(a, ctrl + 1);
	ois_wr(a, ctrl, 0x04);			/* clear, interface off */
	if (rc >= 0 && h->delay_after)
		msleep(le16_to_cpu(h->delay_after));
	return rc;
}

static int ois_fw_ok(const struct firmware *fw)
{
	const struct lois_hdr *h = (const struct lois_hdr *)fw->data;
	int i;

	if (fw->size < sizeof(*h) || memcmp(h->magic, "LOIS", 4) ||
	    le16_to_cpu(h->version) != 1 || h->reg_bytes != 2 ||
	    !h->page_size || le16_to_cpu(h->page_size) > 64 ||
	    le32_to_cpu(h->fw_off) + le32_to_cpu(h->fw_size) > fw->size)
		return 0;
	for (i = 0; i < BLK_NUM; i++)
		if (le32_to_cpu(h->blk[i].off) +
		    4 * le16_to_cpu(h->blk[i].count) > fw->size)
			return 0;
	return 1;
}

/* Module id as in the settings file names: manufacturer, model, revision. */
static int lumia_module_id(struct msm_actuator_ctrl_t *a, u32 *id)
{
	struct msm_camera_i2c_client c = a->ois_client;
	struct msm_camera_cci_client cci = a->ois_cci;
	uint16_t model = 0, rev = 0, mfr = 0;
	int rc;

	cci.sid = a->lumia_sensor_sid ? a->lumia_sensor_sid : 0x10;
	c.cci_client = &cci;
	rc = c.i2c_func_tbl->i2c_read(&c, 0x0000, &model, MSM_CAMERA_I2C_WORD_DATA);
	if (rc >= 0)
		rc = c.i2c_func_tbl->i2c_read(&c, 0x0002, &rev, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc >= 0)
		rc = c.i2c_func_tbl->i2c_read(&c, 0x0003, &mfr, MSM_CAMERA_I2C_BYTE_DATA);
	*id = (mfr & 0xff) << 24 | model << 8 | (rev & 0xff);
	return rc;
}

void msm_actuator_lumia_ois_work(struct work_struct *work)
{
	struct msm_actuator_ctrl_t *a =
		container_of(work, struct msm_actuator_ctrl_t, ois_work);
	const struct firmware *fw = NULL;
	char name[32];
	u32 id = 0;
	int rc, mode = lumia_ois_mode;

	mutex_lock(&a->ois_lock);
	if (a->ois_state != OIS_LOADING)
		goto out;
	rc = lumia_module_id(a, &id);
	if (rc < 0) {
		pr_err("cannot read the module id: %d\n", rc);
		goto fail;
	}
	snprintf(name, sizeof(name), "lumia-ois-%08x.bin", id);
	rc = request_firmware(&fw, name, &a->pdev->dev);
	if (rc < 0) {
		pr_err("%s: %d\n", name, rc);
		goto fail;
	}
	if (!ois_fw_ok(fw) ||
	    le32_to_cpu(((const struct lois_hdr *)fw->data)->module_id) != id) {
		pr_err("%s: bad file\n", name);
		goto fail;
	}
	a->ois_cci.sid = ((const struct lois_hdr *)fw->data)->dev_addr >> 1;

	rc = ois_download(a, fw);
	if (rc >= 0)
		rc = ois_poll_ready(a, 0x0524);
	if (rc >= 0)
		rc = ois_wr(a, 0x0520, 0x01);
	if (rc >= 0)
		rc = ois_poll_ready(a, 0x0524);
	if (rc >= 0)
		rc = ois_run_block(a, fw, BLK_INIT);
	if (rc < 0) {
		pr_err("init failed: %d\n", rc);
		goto fail;
	}
	if (mode >= 1 && mode <= 4)
		rc = ois_run_block(a, fw, mode == 4 ? BLK_OFF : mode);
	pr_info("module %08x ready, mode %d (%d)\n", id, mode, rc);
	a->ois_fw = fw;
	a->ois_state = OIS_READY;
	/*
	 * The original platform re-sends the last lens position after an OIS
	 * mode change. Not done here: the next CFG_MOVE_FOCUS of the AF loop
	 * rewrites it anyway, and this work must not take actuator_mutex
	 * (msm_actuator_close waits for it).
	 */
	goto out;
fail:
	release_firmware(fw);
	a->ois_state = OIS_FAILED;
out:
	mutex_unlock(&a->ois_lock);
}

/* After CFG_SET_ACTUATOR_INFO: the sensor (and the controller) are powered. */
void msm_actuator_lumia_ois_start(struct msm_actuator_ctrl_t *a)
{
	if (!a->ois_present || !lumia_ois)
		return;
	/*
	 * The daemon sends CFG_SET_ACTUATOR_INFO again while the firmware
	 * download of the first one is running, and the download holds
	 * ois_lock for about a second. Busy means a download (or a power
	 * down) is running: nothing to start, and the second call, with the
	 * preview start behind it, must not wait for it.
	 */
	if (!mutex_trylock(&a->ois_lock))
		return;
	if (a->ois_state == OIS_COLD) {
		a->ois_state = OIS_LOADING;
		schedule_work(&a->ois_work);
	}
	mutex_unlock(&a->ois_lock);
}

/* Before the sensor powers down: OIS off, lens parked, controller to sleep. */
void msm_actuator_lumia_ois_stop(struct msm_actuator_ctrl_t *a)
{
	if (!a->ois_present)
		return;
	cancel_work_sync(&a->ois_work);
	mutex_lock(&a->ois_lock);
	if (a->ois_state == OIS_READY && a->ois_fw) {
		ois_run_block(a, a->ois_fw, BLK_OFF);
		if (ois_wr(a, 0x0520, 0x01) >= 0)
			ois_poll_ready(a, 0x0524);
		ois_run_block(a, a->ois_fw, BLK_DEINIT);
	}
	release_firmware(a->ois_fw);
	a->ois_fw = NULL;
	a->ois_state = OIS_COLD;
	mutex_unlock(&a->ois_lock);
}
