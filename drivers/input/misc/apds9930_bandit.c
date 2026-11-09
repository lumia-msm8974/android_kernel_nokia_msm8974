/*
 * apds9930_bandit.c - Avago APDS-9930 ambient light + proximity sensor
 *                     driver for the Nokia Lumia 1520 (bandit)
 *
 * Copyright (c) 2026, Lorenzo Giannoccaro.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * The chip sits on i2c-4 (BLSP1 QUP3) at 0x39, powered by PM8941 L18
 * ("vdd", 2.85 V) and LVS3 ("vio", 1.8 V). Its interrupt is TLMM GPIO 74,
 * not used yet: both sensors are polled with delayed work.
 *
 * Interface for the sensors HAL (two input devices):
 *
 *   "bandit-light"  EV_ABS ABS_MISC     = illuminance in lux (integer)
 *   "bandit-prox"   EV_ABS ABS_DISTANCE = 0 (near) or 5 (far), in cm
 *
 * each followed by EV_SYN. Every input device has, in
 * /sys/class/input/inputN/:
 *
 *   enable      0/1, read/write: starts/stops the sensor
 *   poll_delay  milliseconds, read/write, clamped to [min, 10000];
 *               min = one full chip cycle for light (~107 ms with the
 *               default ATIME/PTIME/WTIME), 10 ms for proximity
 *
 * The chip is put to sleep (PON = 0) when both sensors are disabled and
 * only the engines actually needed (AEN, PEN) are enabled otherwise.
 *
 * Lux is computed with the APDS-9930 datasheet formula, in integer math:
 *
 *   IAC1 = CH0 - B * CH1,  IAC2 = C * CH0 - D * CH1
 *   IAC  = max(IAC1, IAC2, 0)
 *   LPC  = GA * DF / (ALSIT * AGAIN),  LUX = IAC * LPC
 *
 * with DF = 52, B = 1.862, C = 0.746, D = 1.291, ALSIT in ms. The same
 * coefficients (x52) appear in the apds9930 lux table of the mainline
 * tsl2772 driver (drivers/iio/light/tsl2772.c, J. August Brenner / TAOS,
 * Brian Masney). The default proximity settings (PTIME 0xFF, 8 pulses,
 * 100 mA, CH1 diode, 1x gain) are the mainline tsl2772 defaults, so that
 * the thresholds below match the raw values measured on this phone by the
 * lumia-1520-mainline port (about 270 uncovered, 1023 covered), which also
 * found that ALS gain 1x reads zero indoors through the dark glass.
 * No code was copied from those drivers.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/math64.h>
#include <linux/pm.h>

#define APDS_DRV_NAME		"apds9930_bandit"

/* Command byte: 0x80 = register access, 0xA0 = auto-increment, 0xE0 = special */
#define APDS_CMD		0x80
#define APDS_CMD_AUTOINC	0xA0
#define APDS_CMD_CLR_ALL_INT	0xE7

/* Registers */
#define APDS_ENABLE		0x00
#define APDS_ATIME		0x01
#define APDS_PTIME		0x02
#define APDS_WTIME		0x03
#define APDS_AILTL		0x04	/* AILTL, AILTH, AIHTL, AIHTH */
#define APDS_PILTL		0x08	/* PILTL, PILTH, PIHTL, PIHTH */
#define APDS_PERS		0x0C
#define APDS_CONFIG		0x0D
#define APDS_PPULSE		0x0E
#define APDS_CONTROL		0x0F
#define APDS_ID			0x12
#define APDS_STATUS		0x13	/* then CH0 (0x14), CH1 (0x16), PDATA (0x18) */

/* ENABLE bits */
#define APDS_EN_PON		BIT(0)
#define APDS_EN_AEN		BIT(1)
#define APDS_EN_PEN		BIT(2)
#define APDS_EN_WEN		BIT(3)

/* STATUS bits */
#define APDS_ST_AVALID		BIT(0)
#define APDS_ST_PVALID		BIT(1)

/* CONTROL fields */
#define APDS_PDRIVE_100MA	(0 << 6)
#define APDS_PDIODE_CH1		(2 << 4)	/* the only valid setting */
#define APDS_PGAIN_1X		(0 << 2)

/* Timing: every ATIME/PTIME/WTIME step is 2.73 ms (273 x 10 us) */
#define APDS_STEP_10US		273
#define APDS_ATIME_VAL		0xDB	/* 37 cycles = 101 ms, max count 37888 */
#define APDS_PTIME_VAL		0xFF	/* 1 cycle, PDATA max 1023 */
#define APDS_WTIME_VAL		0xFF	/* 2.73 ms wait between cycles */
#define APDS_PPULSE_VAL		8
#define APDS_PERS_VAL		0x11	/* PPERS 1, APERS 1 (unused while polled) */

#define APDS_ALS_CYCLES		(256 - APDS_ATIME_VAL)
#define APDS_ALS_MAX_COUNT	min(65535, 1024 * APDS_ALS_CYCLES)
#define APDS_ALS_SAT		(APDS_ALS_MAX_COUNT * 9 / 10)
#define APDS_CYCLE_STEPS	(APDS_ALS_CYCLES + (256 - APDS_PTIME_VAL) + \
				 (256 - APDS_WTIME_VAL))

/*
 * Datasheet lux coefficients, scaled x1000. B, C and D can be replaced by
 * the board (avago,lux-coeff-b/c/d), for a different window over the
 * sensor.
 */
#define APDS_DF			52
static unsigned int apds_b = 1862;
static unsigned int apds_c = 746;
static unsigned int apds_d = 1291;

#define APDS_LUX_MAX		200000
#define APDS_PROX_NEAR_CM	0
#define APDS_PROX_FAR_CM	5

#define APDS_DELAY_MAX_MS	10000
#define APDS_LIGHT_DELAY_MS	200
#define APDS_PROX_DELAY_MS	100
#define APDS_PROX_MIN_MS	10
#define APDS_LIGHT_MIN_MS	DIV_ROUND_UP(APDS_CYCLE_STEPS * APDS_STEP_10US, 100)

/* AGAIN field value -> gain */
static const unsigned int apds_again_tbl[] = { 1, 8, 16, 120 };

/*
 * Glass attenuation x1000. The Lumia 1520 dark glass passes very little
 * light: calibrated on an RM-937 against a reference phone at 22 lux
 * (CH0 163, CH1 34 at 120x gain, 101 ms), giving GA about 51.4. Other
 * boards set it with avago,glass-attenuation; the parameter still
 * overrides it at run time.
 */
static unsigned int als_ga = 51400;
module_param(als_ga, uint, 0644);
MODULE_PARM_DESC(als_ga, "Glass attenuation factor x1000");

/* Initial ALS gain: 1, 8, 16 or 120. 1x reads zero indoors on this phone. */
static unsigned int als_gain = 8;
module_param(als_gain, uint, 0444);
MODULE_PARM_DESC(als_gain, "Initial ALS gain (1, 8, 16, 120)");

static bool als_autogain = true;
module_param(als_autogain, bool, 0644);
MODULE_PARM_DESC(als_autogain, "Switch ALS gain automatically on saturation/low light");

/*
 * Raw PDATA thresholds (hysteresis): ~270 uncovered, 1023 covered on the
 * 1520. Boards can set them with avago,prox-near and avago,prox-far.
 */
static unsigned int prox_near = 600;
module_param(prox_near, uint, 0644);
MODULE_PARM_DESC(prox_near, "PDATA above which the object is near");

static unsigned int prox_far = 400;
module_param(prox_far, uint, 0644);
MODULE_PARM_DESC(prox_far, "PDATA below which the object is far again");

enum apds_type { APDS_LIGHT, APDS_PROX };

struct apds9930;

struct apds_sensor {
	struct apds9930 *chip;
	struct input_dev *input;
	struct delayed_work work;
	enum apds_type type;
	unsigned int delay_ms;
	bool enabled;
	bool first;		/* force a report of the next sample */
};

struct apds9930 {
	struct i2c_client *client;
	struct regulator *vdd;
	struct regulator *vio;
	struct mutex lock;	/* chip registers and sensor state */
	struct apds_sensor light;
	struct apds_sensor prox;
	u8 enable;		/* cached ENABLE register */
	unsigned int gain_idx;	/* current AGAIN field */
	bool skip_als;		/* discard one sample after a gain change */
	bool near;
	unsigned int invalid;	/* polls in a row without valid data */
};

static int apds_write(struct apds9930 *chip, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(chip->client, APDS_CMD | reg, val);

	if (ret < 0)
		dev_err(&chip->client->dev, "write 0x%02x failed: %d\n", reg, ret);
	return ret;
}

static int apds_write16(struct apds9930 *chip, u8 reg, u16 val)
{
	int ret = apds_write(chip, reg, val & 0xff);

	return ret < 0 ? ret : apds_write(chip, reg + 1, val >> 8);
}

static u8 apds_control(struct apds9930 *chip)
{
	return APDS_PDRIVE_100MA | APDS_PDIODE_CH1 | APDS_PGAIN_1X |
	       chip->gain_idx;
}

/* Program timing, pulses, gains and thresholds (chip must be powered). */
static int apds_configure(struct apds9930 *chip)
{
	int ret;

	ret = apds_write(chip, APDS_ENABLE, 0);
	ret = ret ? : apds_write(chip, APDS_ATIME, APDS_ATIME_VAL);
	ret = ret ? : apds_write(chip, APDS_PTIME, APDS_PTIME_VAL);
	ret = ret ? : apds_write(chip, APDS_WTIME, APDS_WTIME_VAL);
	ret = ret ? : apds_write(chip, APDS_CONFIG, 0); /* no AGL/WLONG/PDL */
	ret = ret ? : apds_write(chip, APDS_PPULSE, APDS_PPULSE_VAL);
	ret = ret ? : apds_write(chip, APDS_CONTROL, apds_control(chip));
	ret = ret ? : apds_write(chip, APDS_PERS, APDS_PERS_VAL);
	/* Thresholds only matter once the IRQ is wired up (TODO below). */
	ret = ret ? : apds_write16(chip, APDS_AILTL, 0);
	ret = ret ? : apds_write16(chip, APDS_AILTL + 2, 0xffff);
	ret = ret ? : apds_write16(chip, APDS_PILTL, prox_far);
	ret = ret ? : apds_write16(chip, APDS_PILTL + 2, prox_near);
	if (ret)
		return ret;
	return i2c_smbus_write_byte(chip->client, APDS_CMD_CLR_ALL_INT);
}

/* Apply the enable state of both sensors to the chip. Lock held. */
static int apds_update_enable(struct apds9930 *chip)
{
	u8 val = 0;
	int ret;

	if (chip->light.enabled || chip->prox.enabled) {
		val = APDS_EN_PON | APDS_EN_WEN;
		if (chip->light.enabled)
			val |= APDS_EN_AEN;
		if (chip->prox.enabled)
			val |= APDS_EN_PEN;
	}
	if (val == chip->enable)
		return 0;

	if (!(chip->enable & APDS_EN_PON) && val) {
		/* Waking up: reprogram everything, then PON alone first. */
		ret = apds_configure(chip);
		ret = ret ? : apds_write(chip, APDS_ENABLE, APDS_EN_PON);
		if (ret)
			return ret;
		usleep_range(3000, 4000);
	}
	ret = apds_write(chip, APDS_ENABLE, val);
	if (ret)
		return ret;
	chip->enable = val;
	return 0;
}

/* Read STATUS, CH0, CH1 and PDATA in one auto-increment transfer. */
static int apds_read_data(struct apds9930 *chip, u8 *status, u16 *ch0,
			  u16 *ch1, u16 *pdata)
{
	u8 buf[7];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(chip->client,
					    APDS_CMD_AUTOINC | APDS_STATUS,
					    sizeof(buf), buf);
	if (ret < 0)
		return ret;
	if (ret != sizeof(buf))
		return -EIO;
	*status = buf[0];
	*ch0 = buf[1] | (buf[2] << 8);
	*ch1 = buf[3] | (buf[4] << 8);
	*pdata = buf[5] | (buf[6] << 8);
	return 0;
}

static unsigned int apds_calc_lux(unsigned int ch0, unsigned int ch1,
				  unsigned int gain)
{
	s64 iac1 = (s64)ch0 * 1000 - (s64)apds_b * ch1;
	s64 iac2 = (s64)apds_c * ch0 - (s64)apds_d * ch1;
	s64 iac = max3(iac1, iac2, (s64)0);	/* x1000 */
	u64 num, den, lux;

	/*
	 * lux = (iac / 1000) * (GA / 1000) * DF / (ALSIT_ms * gain)
	 * with ALSIT_ms = cycles * 273 / 100:
	 * lux = iac * GA * DF * 100 / (1e6 * cycles * 273 * gain)
	 */
	num = (u64)iac * als_ga * APDS_DF * 100;
	den = 1000000ULL * APDS_ALS_CYCLES * APDS_STEP_10US * gain;
	lux = div64_u64(num + den / 2, den);
	return lux > APDS_LUX_MAX ? APDS_LUX_MAX : (unsigned int)lux;
}

/*
 * Pick the gain for the next integration. Returns true when it changed,
 * in which case the current sample (taken with the old gain) is still
 * valid but the next one may straddle the change.
 */
static bool apds_autorange(struct apds9930 *chip, u16 ch0, u16 ch1)
{
	unsigned int idx = chip->gain_idx;
	unsigned int max = APDS_ALS_MAX_COUNT;

	if ((ch0 >= APDS_ALS_SAT || ch1 >= APDS_ALS_SAT) && idx > 0)
		idx--;
	else if (idx < ARRAY_SIZE(apds_again_tbl) - 1 &&
		 (u32)ch0 * apds_again_tbl[idx + 1] <
		 (max / 2) * apds_again_tbl[idx])
		idx++;

	if (idx == chip->gain_idx)
		return false;
	chip->gain_idx = idx;
	if (apds_write(chip, APDS_CONTROL, apds_control(chip)))
		return false;
	dev_dbg(&chip->client->dev, "ALS gain %ux\n", apds_again_tbl[idx]);
	return true;
}

/* Make the input core deliver the next value even if it is unchanged. */
static void apds_force_next(struct input_dev *input, unsigned int code)
{
	unsigned long flags;

	spin_lock_irqsave(&input->event_lock, flags);
	input->absinfo[code].value = -1;
	spin_unlock_irqrestore(&input->event_lock, flags);
}

static void apds_poll_light(struct apds9930 *chip, u8 status, u16 ch0, u16 ch1)
{
	struct apds_sensor *s = &chip->light;
	unsigned int gain = apds_again_tbl[chip->gain_idx];
	unsigned int lux;

	if (!(status & APDS_ST_AVALID))
		return;
	if (chip->skip_als) {
		chip->skip_als = false;
		return;
	}

	lux = apds_calc_lux(ch0, ch1, gain);
	if (als_autogain)
		chip->skip_als = apds_autorange(chip, ch0, ch1);

	if (s->first) {
		apds_force_next(s->input, ABS_MISC);
		s->first = false;
	}
	input_report_abs(s->input, ABS_MISC, lux);
	input_sync(s->input);
	dev_dbg(&chip->client->dev, "ch0 %u ch1 %u gain %ux lux %u\n",
		ch0, ch1, gain, lux);
}

static void apds_poll_prox(struct apds9930 *chip, u8 status, u16 pdata)
{
	struct apds_sensor *s = &chip->prox;

	if (!(status & APDS_ST_PVALID))
		return;

	if (!chip->near && pdata > prox_near)
		chip->near = true;
	else if (chip->near && pdata < prox_far)
		chip->near = false;

	if (s->first) {
		apds_force_next(s->input, ABS_DISTANCE);
		s->first = false;
	}
	input_report_abs(s->input, ABS_DISTANCE,
			 chip->near ? APDS_PROX_NEAR_CM : APDS_PROX_FAR_CM);
	input_sync(s->input);
	dev_dbg(&chip->client->dev, "pdata %u near %d\n", pdata, chip->near);
}

static void apds_work(struct work_struct *work)
{
	struct apds_sensor *s = container_of(to_delayed_work(work),
					     struct apds_sensor, work);
	struct apds9930 *chip = s->chip;
	u16 ch0, ch1, pdata;
	u8 status;
	int ret;

	mutex_lock(&chip->lock);
	if (!s->enabled) {
		mutex_unlock(&chip->lock);
		return;
	}

	ret = apds_read_data(chip, &status, &ch0, &ch1, &pdata);
	if (ret)
		dev_err_ratelimited(&chip->client->dev, "read failed: %d\n", ret);
	else if (s->type == APDS_LIGHT)
		apds_poll_light(chip, status, ch0, ch1);
	else
		apds_poll_prox(chip, status, pdata);

	/*
	 * Self-heal: the chip was seen asleep (ENABLE 0) while both the
	 * cached state and the sysfs enable said it was running, so the
	 * sensor reported 0 lux for ever. After a few polls without valid
	 * data, forget the cached state and power the engines up again.
	 */
	if (!ret && (status & (s->type == APDS_LIGHT ?
			       APDS_ST_AVALID : APDS_ST_PVALID))) {
		chip->invalid = 0;
	} else if (++chip->invalid >= 3) {
		dev_warn_ratelimited(&chip->client->dev,
				     "no valid data, re-enabling the chip\n");
		chip->invalid = 0;
		chip->enable = 0;
		apds_update_enable(chip);
	}

	queue_delayed_work(system_freezable_wq, &s->work,
			   msecs_to_jiffies(s->delay_ms));
	mutex_unlock(&chip->lock);
}

static unsigned int apds_min_delay(struct apds_sensor *s)
{
	return s->type == APDS_LIGHT ? APDS_LIGHT_MIN_MS : APDS_PROX_MIN_MS;
}

static int apds_set_enable(struct apds_sensor *s, bool on)
{
	struct apds9930 *chip = s->chip;
	int ret = 0;

	mutex_lock(&chip->lock);
	if (s->enabled != on) {
		s->enabled = on;
		if (on) {
			s->first = true;
			if (s->type == APDS_PROX)
				chip->near = false;
			else
				chip->skip_als = false;
		}
		ret = apds_update_enable(chip);
		if (ret) {
			s->enabled = !on;
		} else if (on) {
			/* First sample after one full chip cycle. */
			queue_delayed_work(system_freezable_wq, &s->work,
				msecs_to_jiffies(apds_min_delay(s)));
		}
	}
	mutex_unlock(&chip->lock);

	if (!on)
		cancel_delayed_work_sync(&s->work);
	return ret;
}

static ssize_t apds_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds_sensor *s = input_get_drvdata(to_input_dev(dev));

	return sprintf(buf, "%d\n", s->enabled ? 1 : 0);
}

static ssize_t apds_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct apds_sensor *s = input_get_drvdata(to_input_dev(dev));
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	if (val > 1)
		return -EINVAL;
	ret = apds_set_enable(s, val);
	return ret ? ret : count;
}

static ssize_t apds_delay_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct apds_sensor *s = input_get_drvdata(to_input_dev(dev));

	return sprintf(buf, "%u\n", s->delay_ms);
}

static ssize_t apds_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct apds_sensor *s = input_get_drvdata(to_input_dev(dev));
	struct apds9930 *chip = s->chip;
	unsigned int val;
	bool requeue;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	val = clamp_t(unsigned int, val, apds_min_delay(s), APDS_DELAY_MAX_MS);

	mutex_lock(&chip->lock);
	s->delay_ms = val;
	requeue = s->enabled;
	mutex_unlock(&chip->lock);

	/* Apply the new period now instead of after the old one expires. */
	if (requeue) {
		cancel_delayed_work_sync(&s->work);
		mutex_lock(&chip->lock);
		if (s->enabled)
			queue_delayed_work(system_freezable_wq, &s->work,
					   msecs_to_jiffies(s->delay_ms));
		mutex_unlock(&chip->lock);
	}
	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		   apds_enable_show, apds_enable_store);
static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   apds_delay_show, apds_delay_store);

static struct attribute *apds_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static const struct attribute_group apds_attr_group = {
	.attrs = apds_attrs,
};

static int apds_sensor_init(struct apds9930 *chip, struct apds_sensor *s,
			    enum apds_type type)
{
	struct input_dev *input;
	int ret;

	s->chip = chip;
	s->type = type;
	s->delay_ms = type == APDS_LIGHT ? APDS_LIGHT_DELAY_MS :
					   APDS_PROX_DELAY_MS;
	INIT_DELAYED_WORK(&s->work, apds_work);

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	input->id.bustype = BUS_I2C;
	input->dev.parent = &chip->client->dev;
	__set_bit(EV_ABS, input->evbit);
	if (type == APDS_LIGHT) {
		input->name = "bandit-light";
		input_set_abs_params(input, ABS_MISC, 0, APDS_LUX_MAX, 0, 0);
	} else {
		input->name = "bandit-prox";
		input_set_abs_params(input, ABS_DISTANCE, APDS_PROX_NEAR_CM,
				     APDS_PROX_FAR_CM, 0, 0);
	}
	input_set_drvdata(input, s);

	ret = input_register_device(input);
	if (ret) {
		input_free_device(input);
		return ret;
	}

	ret = sysfs_create_group(&input->dev.kobj, &apds_attr_group);
	if (ret) {
		input_unregister_device(input);
		return ret;
	}
	s->input = input;
	return 0;
}

static void apds_sensor_remove(struct apds_sensor *s)
{
	if (!s->input)
		return;
	sysfs_remove_group(&s->input->dev.kobj, &apds_attr_group);
	input_unregister_device(s->input);
	s->input = NULL;
}

/*
 * Optional supply: skipped with a warning when the DT has no "<id>-supply"
 * property (regulator_get would otherwise return -EPROBE_DEFER forever on
 * a fully constrained board). Returns NULL, a regulator, or ERR_PTR.
 */
static struct regulator *apds_get_supply(struct device *dev, const char *id)
{
	char prop[32];
	struct regulator *reg;
	int ret;

	snprintf(prop, sizeof(prop), "%s-supply", id);
	if (dev->of_node && !of_find_property(dev->of_node, prop, NULL)) {
		dev_warn(dev, "no %s supply in DT, assuming always on\n", id);
		return NULL;
	}

	reg = regulator_get(dev, id);
	if (IS_ERR(reg)) {
		ret = PTR_ERR(reg);
		if (ret == -EPROBE_DEFER)
			return reg;
		dev_warn(dev, "cannot get %s supply (%d), continuing\n", id, ret);
		return NULL;
	}

	ret = regulator_enable(reg);
	if (ret) {
		dev_warn(dev, "cannot enable %s supply (%d), continuing\n",
			 id, ret);
		regulator_put(reg);
		return NULL;
	}
	return reg;
}

static void apds_put_supply(struct regulator *reg)
{
	if (!reg)
		return;
	regulator_disable(reg);
	regulator_put(reg);
}

/*
 * Board calibration from the device tree. There is one sensor per phone,
 * so the values replace the module-wide defaults, which stay writable
 * through the parameters.
 */
static void __devinit apds_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 val;

	if (!np)
		return;
	if (!of_property_read_u32(np, "avago,glass-attenuation", &val) && val)
		als_ga = val;
	if (!of_property_read_u32(np, "avago,lux-coeff-b", &val))
		apds_b = val;
	if (!of_property_read_u32(np, "avago,lux-coeff-c", &val))
		apds_c = val;
	if (!of_property_read_u32(np, "avago,lux-coeff-d", &val))
		apds_d = val;
	if (!of_property_read_u32(np, "avago,prox-near", &val))
		prox_near = val;
	if (!of_property_read_u32(np, "avago,prox-far", &val))
		prox_far = val;
	if (prox_far >= prox_near) {
		dev_warn(dev, "prox-far %u not below prox-near %u, using 400/600\n",
			 prox_far, prox_near);
		prox_near = 600;
		prox_far = 400;
	}
	dev_info(dev, "GA %u, B %u, C %u, D %u, prox %u/%u\n", als_ga,
		 apds_b, apds_c, apds_d, prox_near, prox_far);
}

static int __devinit apds_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct apds9930 *chip;
	unsigned int i;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -ENODEV;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);
	apds_parse_dt(&client->dev);

	chip->vdd = apds_get_supply(&client->dev, "vdd");
	if (IS_ERR(chip->vdd))
		return PTR_ERR(chip->vdd);
	chip->vio = apds_get_supply(&client->dev, "vio");
	if (IS_ERR(chip->vio)) {
		ret = PTR_ERR(chip->vio);
		chip->vio = NULL;
		goto err_supply;
	}
	usleep_range(5000, 6000);	/* supply ramp + chip power-on reset */

	ret = i2c_smbus_read_byte_data(client, APDS_CMD | APDS_ID);
	if (ret < 0) {
		dev_err(&client->dev, "cannot read ID: %d\n", ret);
		goto err_supply;
	}
	/* 0x39 on this phone; mainline tsl2772 accepts the 0x3x family. */
	if ((ret & 0xf0) != 0x30) {
		dev_err(&client->dev, "unexpected ID 0x%02x\n", ret);
		ret = -ENODEV;
		goto err_supply;
	}
	dev_info(&client->dev, "APDS-9930 ID 0x%02x\n", ret);

	chip->gain_idx = 1;	/* 8x unless the parameter says otherwise */
	for (i = 0; i < ARRAY_SIZE(apds_again_tbl); i++)
		if (apds_again_tbl[i] == als_gain)
			chip->gain_idx = i;

	ret = apds_configure(chip);	/* leaves the chip asleep */
	if (ret)
		goto err_supply;

	/*
	 * TODO: the INT pin is TLMM GPIO 74 (active low, open drain). With it,
	 * use AIEN/PIEN and the thresholds programmed in apds_configure()
	 * instead of polling, and make proximity a wake-up source.
	 */
	ret = apds_sensor_init(chip, &chip->light, APDS_LIGHT);
	if (ret)
		goto err_supply;
	ret = apds_sensor_init(chip, &chip->prox, APDS_PROX);
	if (ret)
		goto err_light;
	return 0;

err_light:
	apds_sensor_remove(&chip->light);
err_supply:
	apds_put_supply(chip->vio);
	apds_put_supply(chip->vdd);
	return ret;
}

static int __devexit apds_remove(struct i2c_client *client)
{
	struct apds9930 *chip = i2c_get_clientdata(client);

	apds_set_enable(&chip->light, false);
	apds_set_enable(&chip->prox, false);
	apds_sensor_remove(&chip->prox);
	apds_sensor_remove(&chip->light);
	apds_put_supply(chip->vio);
	apds_put_supply(chip->vdd);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
/* Without the IRQ nothing can be sensed in suspend: stop and sleep. */
static int apds_suspend(struct device *dev)
{
	struct apds9930 *chip = i2c_get_clientdata(to_i2c_client(dev));

	cancel_delayed_work_sync(&chip->light.work);
	cancel_delayed_work_sync(&chip->prox.work);
	mutex_lock(&chip->lock);
	if (chip->enable && !apds_write(chip, APDS_ENABLE, 0))
		chip->enable = 0;
	mutex_unlock(&chip->lock);
	return 0;
}

static int apds_resume(struct device *dev)
{
	struct apds9930 *chip = i2c_get_clientdata(to_i2c_client(dev));
	struct apds_sensor *sensors[] = { &chip->light, &chip->prox };
	unsigned int i;

	mutex_lock(&chip->lock);
	apds_update_enable(chip);
	for (i = 0; i < ARRAY_SIZE(sensors); i++) {
		if (!sensors[i]->enabled)
			continue;
		sensors[i]->first = true;
		queue_delayed_work(system_freezable_wq, &sensors[i]->work,
				   msecs_to_jiffies(apds_min_delay(sensors[i])));
	}
	chip->skip_als = false;
	mutex_unlock(&chip->lock);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(apds_pm_ops, apds_suspend, apds_resume);

static const struct i2c_device_id apds_id[] = {
	{ APDS_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds_id);

static const struct of_device_id apds_of_match[] = {
	{ .compatible = "avago,apds9930" },
	{ }
};
MODULE_DEVICE_TABLE(of, apds_of_match);

static struct i2c_driver apds_driver = {
	.driver = {
		.name = APDS_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = apds_of_match,
		.pm = &apds_pm_ops,
	},
	.probe = apds_probe,
	.remove = __devexit_p(apds_remove),
	.id_table = apds_id,
};

module_i2c_driver(apds_driver);

MODULE_AUTHOR("Lorenzo Giannoccaro");
MODULE_DESCRIPTION("Avago APDS-9930 light/proximity driver for Lumia 1520");
MODULE_LICENSE("GPL v2");
