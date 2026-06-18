// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022-2025 Arcturus Industries
 *
 * AW9106C RGB LED control.
 *
 * The sysfs ABI is private to this BSP. Writes cache desired LED state and
 * update the AW9106 when the module is powered. Probe diagnostics can
 * temporarily override the cached state.
 *
 * One AW9106 drives both LEDs: right on OUT0..2, left on OUT3..5.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <media/v4l2-subdev.h>

#include "arcimx616.h"

/* AW9106C register map. */
#define AW9106_REG_P0DO		0x02	/* OUT4..5 output / smart-fade trigger */
#define AW9106_REG_P1DO		0x03	/* OUT0..3 output / smart-fade trigger */
#define AW9106_REG_P0DIR	0x04	/* OUT4..5 dir / blink|smart-fade select */
#define AW9106_REG_P1DIR	0x05	/* OUT0..3 dir / blink|smart-fade select */
#define AW9106_REG_ID		0x10
#define AW9106_REG_GCR		0x11	/* bit7 = GO */
#define AW9106_REG_P0WKMD	0x12	/* OUT4..5 LED(0)/GPIO(1) mode */
#define AW9106_REG_P1WKMD	0x13	/* OUT0..3 LED(0)/GPIO(1) mode */
#define AW9106_REG_PATEN	0x14	/* per-OUT breathing enable */
#define AW9106_REG_FDTMR	0x15	/* fade-on [2:0] / fade-off [5:3] */
#define AW9106_REG_FLTMR	0x16	/* full-on [2:0] / full-off [5:3] */
#define AW9106_REG_DIM0		0x20	/* DIM0..DIM5 at 0x20..0x25 */
#define AW9106_REG_RESET	0x7F	/* write 0x00 to reset */

#define AW9106_ID_VALUE		0x23
#define AW9106_GCR_GO		BIT(7)

/* Logical channel to OUT/DIM index. */
#define AW9106_RIGHT_R_OUT	0
#define AW9106_RIGHT_G_OUT	1
#define AW9106_RIGHT_B_OUT	2
#define AW9106_LEFT_R_OUT	3
#define AW9106_LEFT_G_OUT	4
#define AW9106_LEFT_B_OUT	5
#define AW9106_NUM_OUT		6

#define AW9106_DIM_REG(out)	(AW9106_REG_DIM0 + (out))

/* DIR/DO use P1 for OUT0..3 and P0 for OUT4..5. */
#define AW9106_PATEN_BIT(out)	BIT(out)
#define AW9106_OUT_IN_P0(out)	((out) >= 4)

static inline unsigned long aw9106_out_group_bit(unsigned int out)
{
	return out <= 3 ? BIT(out) : BIT(out - 4);
}

struct arcimx616_led_ctx {
	struct mutex lock; /* guards this context and per-sensor LED state */

	enum arc_led_effect effect;
	u16 fade_on_ms, full_on_ms, fade_off_ms, full_off_ms;

	bool internal_override_active;
	enum arc_led_override_reason override_reason;
	struct arcimx616_led_override ov;

	bool hw_go_active;

	int last_error;
};

static struct arcimx616_led_ctx g_led = {
	.lock = __MUTEX_INITIALIZER(g_led.lock),
	.effect = ARC_LED_EFFECT_STATIC,
	.fade_on_ms = 630,
	.full_on_ms = 315,
	.fade_off_ms = 630,
	.full_off_ms = 315,
};

/* One master sysfs group per AW9106. */
static bool g_led_master_created;

static int arcimx616_aw9106_write_reg(struct arcimx616_dev *sensor, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	if (!g_aw9106_i2c_client) {
		dev_err(&sensor->i2c_client->dev,
			"%s: AW9106 i2c client not initialized\n", __func__);
		return -ENODEV;
	}

	buf[0] = reg;
	buf[1] = val;

	msg.addr  = g_aw9106_i2c_client->addr;
	msg.flags = 0;  /* Write */
	msg.len   = 2;
	msg.buf   = buf;

	ret = i2c_transfer(g_aw9106_i2c_client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&sensor->i2c_client->dev,
			"AW9106 reg=0x%02x val=0x%02x write failed: %d\n",
			reg, val, ret);
		return ret;
	}
	return 0;
}

static int arcimx616_aw9106_read_reg(struct arcimx616_dev *sensor, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf;
	int ret;

	if (!g_aw9106_i2c_client) {
		dev_err(&sensor->i2c_client->dev,
			"%s: AW9106 i2c client not initialized\n", __func__);
		return -ENODEV;
	}

	buf = reg;

	msg[0].addr  = g_aw9106_i2c_client->addr;
	msg[0].flags = 0;  /* Write */
	msg[0].buf   = &buf;
	msg[0].len   = 1;

	msg[1].addr  = g_aw9106_i2c_client->addr;
	msg[1].flags = I2C_M_RD;  /* Read */
	msg[1].buf   = &buf;
	msg[1].len   = 1;

	ret = i2c_transfer(g_aw9106_i2c_client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&sensor->i2c_client->dev,
			"AW9106 reg=0x%02x read failed: %d\n", reg, ret);
		return ret;
	}

	*val = buf;
	return 0;
}

int arcimx616_aw9106_check_chip_id(struct arcimx616_dev *sensor)
{
	int ret;
	u8 led_chip_id = 0;

	ret = arcimx616_aw9106_read_reg(sensor, AW9106_REG_ID, &led_chip_id);
	if (!ret && led_chip_id != AW9106_ID_VALUE)
		ret = -ENXIO;

	return ret;
}

static struct arcimx616_dev *arcimx616_dev_from_dev(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	return to_arcimx616_dev(sd);
}

static struct arcimx616_dev *arcimx616_led_find(bool want_left)
{
	int i;

	for (i = 0; i < g_arcimx616_sensor_count; i++) {
		struct arcimx616_dev *s = g_arcimx616_sensors[i];

		if (s && s->is_left_sensor == want_left)
			return s;
	}
	return NULL;
}

static struct arcimx616_dev *arcimx616_led_anchor(void)
{
	int i;

	for (i = 0; i < g_arcimx616_sensor_count; i++) {
		if (g_arcimx616_sensors[i])
			return g_arcimx616_sensors[i];
	}
	return NULL;
}

static const char *arcimx616_led_effect_name(enum arc_led_effect effect)
{
	switch (effect) {
	case ARC_LED_EFFECT_BLINK:
		return "blink";
	case ARC_LED_EFFECT_SMART_FADE:
		return "smart_fade";
	case ARC_LED_EFFECT_STATIC:
	default:
		return "static";
	}
}

/* Per-sensor brightness scale. */
static u8 arcimx616_led_scale(u8 component, u8 brightness)
{
	return (u8)(((unsigned int)component * brightness) / 255);
}

/* AW9106 timing values in ms; 0 disables a phase. */
static bool arcimx616_led_timing_valid(unsigned int ms)
{
	switch (ms) {
	case 0:
	case 315:
	case 630:
	case 1260:
	case 2520:
	case 5040:
		return true;
	default:
		return false;
	}
}

static u8 arcimx616_led_time_code(u16 ms)
{
	switch (ms) {
	case 315:
		return 1;
	case 630:
		return 2;
	case 1260:
		return 3;
	case 2520:
		return 4;
	case 5040:
		return 5;
	case 0:
	default:
		return 0;
	}
}

/* Build the state to program into the AW9106. */
static void arcimx616_led_compute(u8 dim[AW9106_NUM_OUT],
				  bool active[AW9106_NUM_OUT],
				  enum arc_led_effect *effect,
				  u16 *fade_on, u16 *full_on,
				  u16 *fade_off, u16 *full_off)
{
	int i;

	for (i = 0; i < AW9106_NUM_OUT; i++) {
		dim[i] = 0;
		active[i] = false;
	}

	if (g_led.internal_override_active) {
		const struct arcimx616_led_override *o = &g_led.ov;

		if (o->right_enable) {
			dim[AW9106_RIGHT_R_OUT] =
				arcimx616_led_scale(o->right_r, o->right_brightness);
			dim[AW9106_RIGHT_G_OUT] =
				arcimx616_led_scale(o->right_g, o->right_brightness);
			dim[AW9106_RIGHT_B_OUT] =
				arcimx616_led_scale(o->right_b, o->right_brightness);
		}
		if (o->left_enable) {
			dim[AW9106_LEFT_R_OUT] =
				arcimx616_led_scale(o->left_r, o->left_brightness);
			dim[AW9106_LEFT_G_OUT] =
				arcimx616_led_scale(o->left_g, o->left_brightness);
			dim[AW9106_LEFT_B_OUT] =
				arcimx616_led_scale(o->left_b, o->left_brightness);
		}

		*effect = o->effect;
		*fade_on = o->fade_on_ms;
		*full_on = o->full_on_ms;
		*fade_off = o->fade_off_ms;
		*full_off = o->full_off_ms;
	} else {
		struct arcimx616_dev *left = arcimx616_led_find(true);
		struct arcimx616_dev *right = arcimx616_led_find(false);

		if (right && right->led_enable) {
			dim[AW9106_RIGHT_R_OUT] =
				arcimx616_led_scale(right->led_r, right->led_brightness);
			dim[AW9106_RIGHT_G_OUT] =
				arcimx616_led_scale(right->led_g, right->led_brightness);
			dim[AW9106_RIGHT_B_OUT] =
				arcimx616_led_scale(right->led_b, right->led_brightness);
		}
		if (left && left->led_enable) {
			dim[AW9106_LEFT_R_OUT] =
				arcimx616_led_scale(left->led_r, left->led_brightness);
			dim[AW9106_LEFT_G_OUT] =
				arcimx616_led_scale(left->led_g, left->led_brightness);
			dim[AW9106_LEFT_B_OUT] =
				arcimx616_led_scale(left->led_b, left->led_brightness);
		}

		*effect = g_led.effect;
		*fade_on = g_led.fade_on_ms;
		*full_on = g_led.full_on_ms;
		*fade_off = g_led.fade_off_ms;
		*full_off = g_led.full_off_ms;
	}

	for (i = 0; i < AW9106_NUM_OUT; i++)
		active[i] = dim[i] > 0;
}

/* Effect-specific timing rules. */
static int arcimx616_led_validate(enum arc_led_effect effect,
				  u16 fade_on, u16 full_on,
				  u16 fade_off, u16 full_off)
{
	switch (effect) {
	case ARC_LED_EFFECT_STATIC:
		return 0;
	case ARC_LED_EFFECT_BLINK:
		if (!fade_on || !full_on || !fade_off || !full_off)
			return -EINVAL;
		return 0;
	case ARC_LED_EFFECT_SMART_FADE:
		if (full_on != 0 || full_off != 0)
			return -EINVAL;
		if (fade_on == 0 && fade_off == 0)
			return -EINVAL;
		return 0;
	default:
		return -EINVAL;
	}
}

static int arcimx616_aw9106_program(struct arcimx616_dev *anchor,
				    const u8 dim[AW9106_NUM_OUT],
				    const bool active[AW9106_NUM_OUT],
				    enum arc_led_effect effect,
				    u16 fade_on, u16 full_on,
				    u16 fade_off, u16 full_off)
{
	u8 paten = 0, p0dir = 0, p1dir = 0, p0do = 0, p1do = 0;
	bool any_active = false;
	bool need_reset;
	int ret, i;

	for (i = 0; i < AW9106_NUM_OUT; i++) {
		if (active[i])
			any_active = true;
	}

	/* GO is one-shot. Reset to re-arm blink or clear a latched GO. */
	need_reset = (effect == ARC_LED_EFFECT_BLINK) || g_led.hw_go_active;
	if (need_reset) {
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_RESET, 0x00);
		if (ret)
			goto out;
		g_led.hw_go_active = false;
	}

	/* OUT0..5 in LED mode. */
	ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_P0WKMD, 0x00);
	if (ret)
		goto out;
	ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_P1WKMD, 0x00);
	if (ret)
		goto out;

	switch (effect) {
	case ARC_LED_EFFECT_STATIC:
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_PATEN, 0x00);
		if (ret)
			goto out;
		for (i = 0; i < AW9106_NUM_OUT; i++) {
			ret = arcimx616_aw9106_write_reg(anchor, AW9106_DIM_REG(i), dim[i]);
			if (ret)
				goto out;
		}
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_GCR, 0x00);
		if (ret)
			goto out;
		break;

	case ARC_LED_EFFECT_BLINK:
		for (i = 0; i < AW9106_NUM_OUT; i++) {
			if (!active[i])
				continue;
			paten |= AW9106_PATEN_BIT(i);
			if (AW9106_OUT_IN_P0(i))
				p0dir |= aw9106_out_group_bit(i);
			else
				p1dir |= aw9106_out_group_bit(i);
		}
		for (i = 0; i < AW9106_NUM_OUT; i++) {
			ret = arcimx616_aw9106_write_reg(anchor, AW9106_DIM_REG(i), dim[i]);
			if (ret)
				goto out;
		}
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_PATEN, paten);
		if (ret)
			goto out;
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_P1DIR, p1dir);
		if (ret)
			goto out;
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_P0DIR, p0dir);
		if (ret)
			goto out;
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_FDTMR,
						 (arcimx616_led_time_code(fade_off) << 3) |
						 arcimx616_led_time_code(fade_on));
		if (ret)
			goto out;
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_FLTMR,
						 (arcimx616_led_time_code(full_off) << 3) |
						 arcimx616_led_time_code(full_on));
		if (ret)
			goto out;
		if (any_active) {
			ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_GCR,
							 AW9106_GCR_GO);
			if (ret)
				goto out;
			g_led.hw_go_active = true;
		}
		break;

	case ARC_LED_EFFECT_SMART_FADE:
		for (i = 0; i < AW9106_NUM_OUT; i++) {
			if (!active[i])
				continue;
			paten |= AW9106_PATEN_BIT(i);
			if (AW9106_OUT_IN_P0(i))
				p0do |= aw9106_out_group_bit(i);
			else
				p1do |= aw9106_out_group_bit(i);
		}
		for (i = 0; i < AW9106_NUM_OUT; i++) {
			ret = arcimx616_aw9106_write_reg(anchor, AW9106_DIM_REG(i), dim[i]);
			if (ret)
				goto out;
		}
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_PATEN, paten);
		if (ret)
			goto out;
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_P1DIR, 0x00);
		if (ret)
			goto out;
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_P0DIR, 0x00);
		if (ret)
			goto out;
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_FDTMR,
						 (arcimx616_led_time_code(fade_off) << 3) |
						 arcimx616_led_time_code(fade_on));
		if (ret)
			goto out;
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_P1DO, p1do);
		if (ret)
			goto out;
		ret = arcimx616_aw9106_write_reg(anchor, AW9106_REG_P0DO, p0do);
		if (ret)
			goto out;
		break;

	default:
		ret = -EINVAL;
		goto out;
	}

	ret = 0;
out:
	return ret;
}

/* Call with g_led.lock held. */
static int arcimx616_led_apply_effective_locked(void)
{
	struct arcimx616_dev *anchor = arcimx616_led_anchor();
	u8 dim[AW9106_NUM_OUT];
	bool active[AW9106_NUM_OUT];
	enum arc_led_effect effect;
	u16 fade_on, full_on, fade_off, full_off;
	int ret;

	if (!anchor) {
		g_led.last_error = -ENODEV;
		return -ENODEV;
	}

	arcimx616_led_compute(dim, active, &effect,
			      &fade_on, &full_on, &fade_off, &full_off);

	/* Cache while the shared rail is off. */
	if (arcimx616_power_on_count <= 0)
		return 0;

	ret = arcimx616_led_validate(effect, fade_on, full_on, fade_off, full_off);
	if (ret) {
		g_led.last_error = ret;
		return ret;
	}

	ret = arcimx616_aw9106_program(anchor, dim, active, effect,
				       fade_on, full_on, fade_off, full_off);
	g_led.last_error = ret;
	return ret;
}

int arcimx616_led_apply_effective(void)
{
	int ret;

	mutex_lock(&g_led.lock);
	ret = arcimx616_led_apply_effective_locked();
	mutex_unlock(&g_led.lock);
	return ret;
}

static int
arcimx616_led_set_internal_override_locked(enum arc_led_override_reason reason,
					   const struct arcimx616_led_override *ov)
{
	g_led.internal_override_active = true;
	g_led.override_reason = reason;
	g_led.ov = *ov;

	return arcimx616_led_apply_effective_locked();
}

static int arcimx616_led_clear_internal_override_locked(void)
{
	g_led.internal_override_active = false;
	g_led.override_reason = ARC_LED_OVERRIDE_NONE;

	return arcimx616_led_apply_effective_locked();
}

int arcimx616_led_set_internal_override(enum arc_led_override_reason reason,
					const struct arcimx616_led_override *ov)
{
	int ret;

	mutex_lock(&g_led.lock);
	ret = arcimx616_led_set_internal_override_locked(reason, ov);
	mutex_unlock(&g_led.lock);
	return ret;
}

int arcimx616_led_clear_internal_override(void)
{
	int ret;

	mutex_lock(&g_led.lock);
	ret = arcimx616_led_clear_internal_override_locked();
	mutex_unlock(&g_led.lock);
	return ret;
}

/* Per-sensor private BSP attributes. */
static ssize_t led_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct arcimx616_dev *sensor = arcimx616_dev_from_dev(dev);
	ssize_t ret;

	mutex_lock(&g_led.lock);
	ret = sysfs_emit(buf, "%d\n", sensor->led_enable ? 1 : 0);
	mutex_unlock(&g_led.lock);
	return ret;
}

static ssize_t led_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct arcimx616_dev *sensor = arcimx616_dev_from_dev(dev);
	bool enable;
	int ret;

	if (sysfs_streq(buf, "1") || sysfs_streq(buf, "on"))
		enable = true;
	else if (sysfs_streq(buf, "0") || sysfs_streq(buf, "off"))
		enable = false;
	else
		return -EINVAL;

	mutex_lock(&g_led.lock);
	sensor->led_enable = enable;
	ret = arcimx616_led_apply_effective_locked();
	mutex_unlock(&g_led.lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(led_enable);

static ssize_t led_rgb_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct arcimx616_dev *sensor = arcimx616_dev_from_dev(dev);
	ssize_t ret;

	mutex_lock(&g_led.lock);
	ret = sysfs_emit(buf, "%u %u %u\n",
			 sensor->led_r, sensor->led_g, sensor->led_b);
	mutex_unlock(&g_led.lock);
	return ret;
}

static ssize_t led_rgb_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct arcimx616_dev *sensor = arcimx616_dev_from_dev(dev);
	unsigned int r, g, b, extra;
	int n, ret;

	n = sscanf(buf, "%u %u %u %u", &r, &g, &b, &extra);
	if (n != 3)
		return -EINVAL;
	if (r > 255 || g > 255 || b > 255)
		return -EINVAL;

	mutex_lock(&g_led.lock);
	sensor->led_r = r;
	sensor->led_g = g;
	sensor->led_b = b;
	ret = arcimx616_led_apply_effective_locked();
	mutex_unlock(&g_led.lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(led_rgb);

static ssize_t led_brightness_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct arcimx616_dev *sensor = arcimx616_dev_from_dev(dev);
	ssize_t ret;

	mutex_lock(&g_led.lock);
	ret = sysfs_emit(buf, "%u\n", sensor->led_brightness);
	mutex_unlock(&g_led.lock);
	return ret;
}

static ssize_t led_brightness_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct arcimx616_dev *sensor = arcimx616_dev_from_dev(dev);
	u8 brightness;
	int ret;

	ret = kstrtou8(buf, 10, &brightness);
	if (ret)
		return -EINVAL;

	mutex_lock(&g_led.lock);
	sensor->led_brightness = brightness;
	ret = arcimx616_led_apply_effective_locked();
	mutex_unlock(&g_led.lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(led_brightness);

/* Shared AW9106 attributes, exposed on one sensor. */
static ssize_t led_effect_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	mutex_lock(&g_led.lock);
	ret = sysfs_emit(buf, "%s\n", arcimx616_led_effect_name(g_led.effect));
	mutex_unlock(&g_led.lock);
	return ret;
}

static ssize_t led_effect_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	enum arc_led_effect effect;
	int ret;

	if (sysfs_streq(buf, "static"))
		effect = ARC_LED_EFFECT_STATIC;
	else if (sysfs_streq(buf, "blink"))
		effect = ARC_LED_EFFECT_BLINK;
	else if (sysfs_streq(buf, "smart_fade"))
		effect = ARC_LED_EFFECT_SMART_FADE;
	else
		return -EINVAL;

	mutex_lock(&g_led.lock);
	ret = arcimx616_led_validate(effect, g_led.fade_on_ms, g_led.full_on_ms,
				     g_led.fade_off_ms, g_led.full_off_ms);
	if (!ret) {
		g_led.effect = effect;
		ret = arcimx616_led_apply_effective_locked();
	}
	mutex_unlock(&g_led.lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(led_effect);

static ssize_t led_timing_ms_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	ssize_t ret;

	mutex_lock(&g_led.lock);
	ret = sysfs_emit(buf, "%u %u %u %u\n",
			 g_led.fade_on_ms, g_led.full_on_ms,
			 g_led.fade_off_ms, g_led.full_off_ms);
	mutex_unlock(&g_led.lock);
	return ret;
}

static ssize_t led_timing_ms_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned int fade_on, full_on, fade_off, full_off, extra;
	int n, ret;

	n = sscanf(buf, "%u %u %u %u %u",
		   &fade_on, &full_on, &fade_off, &full_off, &extra);
	if (n != 4)
		return -EINVAL;
	if (!arcimx616_led_timing_valid(fade_on) ||
	    !arcimx616_led_timing_valid(full_on) ||
	    !arcimx616_led_timing_valid(fade_off) ||
	    !arcimx616_led_timing_valid(full_off))
		return -EINVAL;

	mutex_lock(&g_led.lock);
	ret = arcimx616_led_validate(g_led.effect, fade_on, full_on,
				     fade_off, full_off);
	if (!ret) {
		g_led.fade_on_ms = fade_on;
		g_led.full_on_ms = full_on;
		g_led.fade_off_ms = fade_off;
		g_led.full_off_ms = full_off;
		ret = arcimx616_led_apply_effective_locked();
	}
	mutex_unlock(&g_led.lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(led_timing_ms);

static ssize_t led_status_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct arcimx616_dev *master = arcimx616_dev_from_dev(dev);
	struct arcimx616_dev *left, *right;
	ssize_t len = 0;

	mutex_lock(&g_led.lock);
	left = arcimx616_led_find(true);
	right = arcimx616_led_find(false);

	len += sysfs_emit_at(buf, len, "present=%d powered=%d effect=%s ",
			     master->connected ? 1 : 0,
			     arcimx616_power_on_count > 0 ? 1 : 0,
			     arcimx616_led_effect_name(g_led.effect));
	len += sysfs_emit_at(buf, len,
			     "left_enable=%d left_rgb=%u %u %u left_brightness=%u ",
			     (left && left->led_enable) ? 1 : 0,
			     left ? left->led_r : 0,
			     left ? left->led_g : 0,
			     left ? left->led_b : 0,
			     left ? left->led_brightness : 0);
	len += sysfs_emit_at(buf, len,
			     "right_enable=%d right_rgb=%u %u %u right_brightness=%u ",
			     (right && right->led_enable) ? 1 : 0,
			     right ? right->led_r : 0,
			     right ? right->led_g : 0,
			     right ? right->led_b : 0,
			     right ? right->led_brightness : 0);
	len += sysfs_emit_at(buf, len,
			     "timing_ms=%u %u %u %u last_error=%d override=%d\n",
			     g_led.fade_on_ms, g_led.full_on_ms,
			     g_led.fade_off_ms, g_led.full_off_ms,
			     g_led.last_error,
			     g_led.internal_override_active ? 1 : 0);
	mutex_unlock(&g_led.lock);
	return len;
}
static DEVICE_ATTR_RO(led_status);

static struct attribute *arcimx616_led_common_attrs[] = {
	&dev_attr_led_enable.attr,
	&dev_attr_led_rgb.attr,
	&dev_attr_led_brightness.attr,
	NULL,
};

static const struct attribute_group arcimx616_led_common_group = {
	.attrs = arcimx616_led_common_attrs,
};

static struct attribute *arcimx616_led_master_attrs[] = {
	&dev_attr_led_effect.attr,
	&dev_attr_led_timing_ms.attr,
	&dev_attr_led_status.attr,
	NULL,
};

static const struct attribute_group arcimx616_led_master_group = {
	.attrs = arcimx616_led_master_attrs,
};

void arcimx616_led_init_state(struct arcimx616_dev *sensor)
{
	mutex_lock(&g_led.lock);
	sensor->led_enable = false;
	sensor->led_r = 0;
	sensor->led_g = 0;
	sensor->led_b = 0;
	sensor->led_brightness = 255;
	mutex_unlock(&g_led.lock);
}

int arcimx616_led_create_sysfs(struct arcimx616_dev *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret;

	ret = device_add_group(dev, &arcimx616_led_common_group);
	if (ret)
		return ret;

	if (sensor->is_led_master) {
		if (g_led_master_created) {
			dev_warn(dev,
				 "LED master already assigned; not exposing master attrs here\n");
			sensor->is_led_master = false;
		} else {
			ret = device_add_group(dev, &arcimx616_led_master_group);
			if (ret) {
				device_remove_group(dev, &arcimx616_led_common_group);
				return ret;
			}
			g_led_master_created = true;
		}
	}

	return 0;
}

void arcimx616_led_remove_sysfs(struct arcimx616_dev *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;

	if (sensor->is_led_master) {
		device_remove_group(dev, &arcimx616_led_master_group);
		g_led_master_created = false;
	}
	device_remove_group(dev, &arcimx616_led_common_group);
}
