// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 * 
 * Based on OV6211 driver. Adapted by arcturus.industries
 */

#include <linux/unaligned.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define OV6211_LINK_FREQUENCY 240000000

#define OV6211_REG_CHIP_ID		0x300a

#define OV6211_REG_TIMING_DVPHO		0x3808
#define OV6211_REG_TIMING_DVPVO		0x380a
#define OV6211_REG_TIMING_HTS		0x380c
#define OV6211_REG_TIMING_VTS		0x380e

#define VIDIOC_SENSOR_PRV_G_REGISTER _IOWR('V', BASE_VIDIOC_PRIVATE + 8, struct sensor_register)

struct sensor_register {
	u16 reg_addr;
	u32 reg_value;
	u8 len;
};

static unsigned int virtual_channel;

/* regulator supplies */
static const char * const ov6211_supply_name[] = {
	"DOVDD", /* Digital I/O (1.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
	"DVDD",  /* Digital Core (1.5V) supply */
};

#define OV6211_NUM_SUPPLIES ARRAY_SIZE(ov6211_supply_name)

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

struct ov6211_mode_info {
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
	const struct reg_value *reg_data;
	u32 reg_data_size;
};

struct ov6211_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to OV6211 */

	struct regulator_bulk_data supplies[OV6211_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;

	/* lock to protect all members below */
	struct mutex lock;

	int power_count;

	struct v4l2_ctrl_handler ctrl_handler;
	struct mutex mutex;

	struct v4l2_mbus_framefmt fmt;

	bool streaming;
};

static inline struct ov6211_dev *to_ov6211_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov6211_dev, sd);
}

static const struct reg_value ov6211_setting[] = {

{0x0103, 0x01, 0, 0}, {0x0100, 0x00, 0, 0}, {0x3005, 0x08, 0, 0},
{0x3013, 0x12, 0, 0}, {0x3014, 0x04, 0, 0}, {0x3016, 0x10, 0, 0},
{0x3017, 0x00, 0, 0}, {0x3018, 0x00, 0, 0}, {0x301a, 0x00, 0, 0},
{0x301b, 0x00, 0, 0}, {0x301c, 0x00, 0, 0}, {0x3037, 0xf0, 0, 0},
{0x3080, 0x01, 0, 0}, {0x3081, 0x00, 0, 0}, {0x3082, 0x01, 0, 0},
{0x3098, 0x04, 0, 0}, {0x3099, 0x28, 0, 0}, {0x309a, 0x06, 0, 0},
{0x309b, 0x04, 0, 0}, {0x309c, 0x00, 0, 0}, {0x309d, 0x00, 0, 0},
{0x309e, 0x01, 0, 0}, {0x309f, 0x00, 0, 0}, {0x30b0, 0x08, 0, 0},
{0x30b1, 0x02, 0, 0}, {0x30b2, 0x00, 0, 0}, {0x30b3, 0x28, 0, 0},
{0x30b4, 0x02, 0, 0}, {0x30b5, 0x00, 0, 0}, {0x3106, 0xd9, 0, 0},
{0x3500, 0x00, 0, 0}, {0x3501, 0x0d, 0, 0}, {0x3502, 0x20, 0, 0},
{0x3503, 0x07, 0, 0}, {0x3509, 0x10, 0, 0}, {0x350b, 0xa0, 0, 0},
{0x3600, 0xfc, 0, 0}, {0x3620, 0xb7, 0, 0}, {0x3621, 0x05, 0, 0},
{0x3626, 0x31, 0, 0}, {0x3627, 0x40, 0, 0}, {0x3632, 0xa3, 0, 0},
{0x3633, 0x34, 0, 0}, {0x3634, 0x40, 0, 0}, {0x3636, 0x00, 0, 0},
{0x3660, 0x80, 0, 0}, {0x3662, 0x03, 0, 0}, {0x3664, 0xf0, 0, 0},
{0x366a, 0x10, 0, 0}, {0x366b, 0x06, 0, 0}, {0x3680, 0xf4, 0, 0},
{0x3681, 0x50, 0, 0}, {0x3682, 0x00, 0, 0}, {0x3708, 0x20, 0, 0},
{0x3709, 0x40, 0, 0}, {0x370d, 0x03, 0, 0}, {0x373b, 0x02, 0, 0},
{0x373c, 0x08, 0, 0}, {0x3742, 0x00, 0, 0}, {0x3744, 0x16, 0, 0},
{0x3745, 0x08, 0, 0}, {0x3781, 0xfc, 0, 0}, {0x3788, 0x00, 0, 0},
{0x3800, 0x00, 0, 0}, {0x3801, 0x04, 0, 0}, {0x3802, 0x00, 0, 0},
{0x3803, 0x04, 0, 0}, {0x3804, 0x01, 0, 0}, {0x3805, 0x9b, 0, 0},
{0x3806, 0x01, 0, 0}, {0x3807, 0x9b, 0, 0}, {0x3808, 0x01, 0, 0},
{0x3809, 0x90, 0, 0}, {0x380a, 0x01, 0, 0}, {0x380b, 0x90, 0, 0},
{0x380c, 0x05, 0, 0}, {0x380d, 0xf2, 0, 0}, {0x380e, 0x01, 0, 0},
{0x380f, 0xb6, 0, 0}, {0x3810, 0x00, 0, 0}, {0x3811, 0x04, 0, 0},
{0x3812, 0x00, 0, 0}, {0x3813, 0x04, 0, 0}, {0x3814, 0x11, 0, 0},
{0x3815, 0x11, 0, 0}, {0x3820, 0x00, 0, 0}, {0x3821, 0x00, 0, 0},
{0x382b, 0xfa, 0, 0}, {0x382f, 0x04, 0, 0}, {0x3832, 0x00, 0, 0},
{0x3833, 0x05, 0, 0}, {0x3834, 0x00, 0, 0}, {0x3835, 0x05, 0, 0},
{0x3882, 0x04, 0, 0}, {0x3883, 0x00, 0, 0}, {0x38a4, 0x10, 0, 0},
{0x38a5, 0x00, 0, 0}, {0x38b1, 0x03, 0, 0}, {0x3b80, 0x00, 0, 0},
{0x3b81, 0xff, 0, 0}, {0x3b82, 0x10, 0, 0}, {0x3b83, 0x00, 0, 0},
{0x3b84, 0x08, 0, 0}, {0x3b85, 0x00, 0, 0}, {0x3b86, 0x01, 0, 0},
{0x3b87, 0x00, 0, 0}, {0x3b88, 0x00, 0, 0}, {0x3b89, 0x00, 0, 0},
{0x3b8a, 0x00, 0, 0}, {0x3b8b, 0x05, 0, 0}, {0x3b8c, 0x00, 0, 0},
{0x3b8d, 0x00, 0, 0}, {0x3b8e, 0x01, 0, 0}, {0x3b8f, 0xb2, 0, 0},
{0x3b94, 0x05, 0, 0}, {0x3b95, 0xf2, 0, 0}, {0x3b96, 0xc0, 0, 0},
{0x4004, 0x04, 0, 0}, {0x404e, 0x01, 0, 0}, {0x4801, 0x0f, 0, 0},
{0x4806, 0x0f, 0, 0}, {0x4837, 0x43, 0, 0}, {0x5a08, 0x00, 0, 0},
{0x5a01, 0x00, 0, 0}, {0x5a03, 0x00, 0, 0}, {0x5a04, 0x10, 0, 0},
{0x5a05, 0xa0, 0, 0}, {0x5a06, 0x0c, 0, 0}, {0x5a07, 0x78, 0, 0},

};

static const struct ov6211_mode_info ov6211_mode_data = {
	1280, 1456 / 2, 800, 914, ov6211_setting, ARRAY_SIZE(ov6211_setting)
	// 1280, 1456, 800, 3676, ov6211_setting, ARRAY_SIZE(ov6211_setting)
};

static const s64 link_freq_menu_items[] = {
	OV6211_LINK_FREQUENCY
};

static int ov6211_write_reg(struct ov6211_dev *sensor, u16 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x, val=%x ret=%d\n",
			__func__, reg, val, ret);
		return ret;
	}

	return 0;
}

static int ov6211_read_reg(struct ov6211_dev *sensor, u16 reg, u8 *val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=%x\n",
			__func__, reg);
		return ret;
	}

	*val = buf[0];
	return 0;
}

static int ov6211_read_reg_len(struct ov6211_dev *sensor, u16 reg, u16 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[4] = {0};
	int ret;

	put_unaligned_be16(reg, addr_buf);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr_buf);
	msgs[0].buf = addr_buf;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

static int ov6211_write_reg16(struct ov6211_dev *sensor, u16 reg, u16 val)
{
	int ret;

	ret = ov6211_write_reg(sensor, reg, val >> 8);
	if (ret)
		return ret;

	return ov6211_write_reg(sensor, reg + 1, val & 0xff);
}

/* download ov6211 settings to sensor through i2c */
static int ov6211_set_timings(struct ov6211_dev *sensor,
			      const struct ov6211_mode_info *mode)
{
	int ret;

	ret = ov6211_write_reg16(sensor, OV6211_REG_TIMING_DVPHO, mode->hact);
	if (ret < 0)
		return ret;

	ret = ov6211_write_reg16(sensor, OV6211_REG_TIMING_DVPVO, mode->vact);
	if (ret < 0)
		return ret;

	ret = ov6211_write_reg16(sensor, OV6211_REG_TIMING_HTS, mode->htot);
	if (ret < 0)
		return ret;

	return ov6211_write_reg16(sensor, OV6211_REG_TIMING_VTS, mode->vtot);
}

static int ov6211_load_regs(struct ov6211_dev *sensor,
			    const struct ov6211_mode_info *mode)
{
	const struct reg_value *regs = mode->reg_data;
	unsigned int i;
	u32 delay_ms;
	u16 reg_addr;
	u8 mask, val;
	int ret = 0;

	struct i2c_client *client = sensor->i2c_client;

	u16 chip_id;
	ret = ov6211_read_reg_len(sensor, OV6211_REG_CHIP_ID, 2, (u32 *)&chip_id);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read chip identifier\n",
			__func__);
		return ret;
	}

	if (chip_id != 0x6710) {
		dev_err(&client->dev, "%s: wrong chip identifier won't stream!, expected 0x6710, got 0x%x\n",
			__func__, chip_id);
		ret = -ENXIO;
		return ret;
	}


	for (i = 0; i < mode->reg_data_size; ++i, ++regs) {
		delay_ms = regs->delay_ms;
		reg_addr = regs->reg_addr;
		val = regs->val;
		mask = regs->mask;

		ret = ov6211_write_reg(sensor, reg_addr, val);
		if (ret)
			break;

		if (delay_ms)
			usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
	}

	dev_err(&client->dev, "%s: REGISTER MAP SENT\n",
			__func__);
	return 0;
	//return ov6211_set_timings(sensor, mode);
}

/* restore the last set video mode after chip power-on */
static int ov6211_restore_mode(struct ov6211_dev *sensor)
{
	int ret;
	u8 val;

	struct i2c_client *client = sensor->i2c_client;

	/* first load the initial register values */
	ret = ov6211_load_regs(sensor, &ov6211_mode_data);
	if (ret < 0)
		return ret;

	//ret = ov6211_read_reg(sensor, 0x3662, &val);
	// if (ret < 0 || val != 0b111) {
	// 	struct i2c_client *client = sensor->i2c_client;
	// 	dev_err(&client->dev, "verify failed val=%.2x ret=%d\n", val, ret);
	// 	return ret;
	// }

	dev_err(&client->dev, "%s:ov6211_restore_mode complete\n",
			__func__);
	return 0;//ov6211_set_timings(sensor, &ov6211_mode_data);
}

static void ov6211_reset(struct ov6211_dev *sensor)
{
	if (!sensor->reset_gpio)
		return;

	usleep_range(20000, 25000);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(20000, 25000);

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(20000, 25000);
}

uint8_t ov6211_power_on;

static int ov6211_set_power_on(struct ov6211_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	clk_set_rate(sensor->xclk, 24000000);

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		return ret;
	}

	ret = regulator_bulk_enable(OV6211_NUM_SUPPLIES,
				    sensor->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		goto xclk_off;
	}

	ov6211_reset(sensor);

	usleep_range(5000,5500);

    ov6211_power_on = 1;

	return 0;

power_off:
	regulator_bulk_disable(OV6211_NUM_SUPPLIES, sensor->supplies);
xclk_off:
	clk_disable_unprepare(sensor->xclk);
	return ret;
}

static void ov6211_set_power_off(struct ov6211_dev *sensor)
{
	int i;
	int ret;
    // reset pin is setup as active low
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(20000, 25000);
    // individually shut down regulators instead of bulk, because
    // bulk will fail out if one of the regulators has already been shut off
	for (i = OV6211_NUM_SUPPLIES - 1; i >= 0; --i) {
		regulator_disable(sensor->supplies[i].consumer);
	}
	clk_disable_unprepare(sensor->xclk);
    ov6211_power_on = 0;
}

static int ov6211_set_power(struct ov6211_dev *sensor, bool on)
{
	int ret = 0;

	if (on) {
		ret = ov6211_set_power_on(sensor);
		if (ret)
			return ret;

		ret = ov6211_restore_mode(sensor);
		if (ret)
			goto power_off;

	} else {
		ov6211_set_power_off(sensor);
	}

	return 0;

power_off:
	//ov6211_set_power_off(sensor);
	return ret;
}

static ssize_t 
enable_show(struct device *dev, struct device_attribute *attr,
    char *buf)
{
	buf[0] = ov6211_power_on + '0';
	buf[1] = '\n';
	return 2;
}

static ssize_t
enable_store(struct device *dev, struct device_attribute *attr,
	  const char *buf, size_t count)
{
    int ret;
	int enable;
    struct ov6211_dev *sensor = dev_get_drvdata(dev);
	ret = sscanf(buf, "%d", &enable);

    if ( ret != 1){
        pr_debug("Invalid number of params for enable\n");
        return -1;
    }

    ov6211_set_power(sensor, !!enable);

	return count;
}
static DEVICE_ATTR_RW(enable);

static struct attribute *ov6211_attrs[] = {
    &dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group ov6211_attr_group = {
	.name = "ov6211",
    .attrs = ov6211_attrs,
};

static int ov6211_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov6211_dev *sensor =
			container_of(ctrl->handler, struct ov6211_dev, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
	int ret = 0;

	/* V4L2 controls values will be applied only when power is already up */
	//if (!pm_runtime_get_if_in_use(&client->dev))
	//	return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		/* note:
		 * gain_prec16_en = 1,
		 * dac_finegain_highbit = 1,
		 * gain shift = 3 (multiply by 8?)
		 *
		 * value in 6.4 fractional format
		 */
		ret = ov6211_write_reg(sensor, 0x350A, (ctrl->val >> 8) & 0x03);
		
		ret = ov6211_write_reg(sensor, 0x350B, (ctrl->val) & 0xff);
		break;

	case V4L2_CID_EXPOSURE:
		/* note : value in row periods, max is (vts - 4) */
		ret = ov6211_write_reg(sensor, 0x3500, (ctrl->val >> 12) &0x0f);
		ret = ov6211_write_reg(sensor, 0x3501, ctrl->val >> 4);
		ret = ov6211_write_reg(sensor, 0x3502, ctrl->val << 4);
		break;
	case V4L2_CID_BRIGHTNESS:
		ret = ov6211_write_reg(sensor, ctrl->val & 0xffff, ctrl->val >> 16 & 0xff);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	//pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov6211_ctrl_ops = {
	.s_ctrl = ov6211_set_ctrl,
};

static int ov6211_init_controls(struct ov6211_dev *sensor)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	int ret;

	ctrl_hdlr = &sensor->ctrl_handler;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 2);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &sensor->mutex;

	v4l2_ctrl_new_std(ctrl_hdlr, &ov6211_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			0, 0x3ff, 1, 0xa0);
	v4l2_ctrl_new_std(ctrl_hdlr, &ov6211_ctrl_ops, V4L2_CID_EXPOSURE,
			1, 6944, 1, 250);

	/* hack control for custom i2c writes */
	v4l2_ctrl_new_std(ctrl_hdlr, &ov6211_ctrl_ops, V4L2_CID_BRIGHTNESS, 0, 0xffffff, 1, 0);

	int size = ARRAY_SIZE(link_freq_menu_items);

	struct v4l2_ctrl *link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &ov6211_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   size - 1, 0,
						   link_freq_menu_items);
	if (link_freq)
		link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	sensor->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

/* --------------- Subdev Operations --------------- */

static int ov6211_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (sensor->power_count == !on) {
		ret = ov6211_set_power(sensor, !!on);
		if (ret)
			goto out;
	}

	/* Update the power count. */
	sensor->power_count += on ? 2 : -1; // XXX 2 avoids ever powering down
	WARN_ON(sensor->power_count < 0);
out:
	mutex_unlock(&sensor->lock);

	return ret;
}

static int ov6211_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	dev_err(&client->dev, "%s: s_stream called with enable:%d\n",
			__func__,enable);

	ov6211_write_reg(sensor, 0x100, !!enable);

	return 0;
}

static long ov6211_private_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
				 void *arg)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;
	u32 *a = arg;

	switch (cmd) {
	case VIDIOC_SENSOR_PRV_G_REGISTER: {
		struct sensor_register *reg = arg;
		u32 regval = 0;
		ret = ov6211_read_reg_len(sensor, reg->reg_addr, reg->len, &regval);
		if (ret) {
			dev_err(&sensor->i2c_client->dev, "Failed to read register 0x%04x: %d\n", reg->reg_addr, ret);
		}
		reg->reg_value = regval;
		break;
	}
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct v4l2_subdev_core_ops ov6211_core_ops = {
	.s_power = ov6211_s_power,
	.ioctl = ov6211_private_ioctl,
};

static const struct v4l2_subdev_video_ops ov6211_video_ops = {
	.s_stream = ov6211_s_stream,
};

static int ov6211_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	code->code = MEDIA_BUS_FMT_Y8_1X8;
	return 0;
}

static int ov6211_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct ov6211_dev *sensor = to_ov6211_dev(sd);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	format->format = sensor->fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static const struct v4l2_subdev_pad_ops ov6211_pad_ops = {
	.enum_mbus_code = ov6211_enum_mbus_code,
	.get_fmt = ov6211_get_fmt,
};

static const struct v4l2_subdev_ops ov6211_subdev_ops = {
	.core = &ov6211_core_ops,
	.video = &ov6211_video_ops,
	.pad = &ov6211_pad_ops,
};

static int ov6211_get_regulators(struct ov6211_dev *sensor)
{
	int i;

	for (i = 0; i < OV6211_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = ov6211_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       OV6211_NUM_SUPPLIES,
				       sensor->supplies);
}

static int ov6211_check_chip_id(struct ov6211_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;
	u16 chip_id;

	ret = ov6211_set_power_on(sensor);
	if (ret)
		return ret;

	ret = ov6211_read_reg_len(sensor, OV6211_REG_CHIP_ID, 2, (u32 *)&chip_id);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read chip identifier\n",
			__func__);
		goto power_off;
	}

	if (chip_id != 0x6710) {
		dev_err(&client->dev, "%s: FORCE ON wrong chip identifier, expected 0x6710, got 0x%x\n",
			__func__, chip_id);
		ret = 0;//-ENXIO;
	}
	else {
		dev_err(&client->dev, "%s: AAA Got correct chip identifier, expected 06710, got 0x%x\n",
			__func__, chip_id);
	}

power_off:
	ov6211_set_power_off(sensor);
	return ret;
}

static int ov6211_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct ov6211_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	u32 rotation;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_Y8_1X8;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 400;
	fmt->height = 400;
	fmt->field = V4L2_FIELD_NONE;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	/* get system clock (xclk) */
	sensor->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}

	/* request optional reset pin */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return PTR_ERR(sensor->reset_gpio);

	v4l2_i2c_subdev_init(&sensor->sd, client, &ov6211_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	ret = ov6211_get_regulators(sensor);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	ret = ov6211_check_chip_id(sensor);
	if (ret) 
	{
		dev_err(&client->dev, "Chip probe failed with err: %d but we will continue anyway to ensure v4l2 is setup.", ret);
	}
	mutex_init(&sensor->mutex);
	ret = ov6211_init_controls(sensor);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto entity_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto free_ctrls;

    dev_set_drvdata(dev, sensor);

    ret = sysfs_create_group(&dev->kobj, &ov6211_attr_group);
    if (ret < 0) {
        dev_err(dev, "failed to create ov6211 sysfs files\n");
        return ret;
    }

	return 0;

free_ctrls:
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static void ov6211_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov6211_dev *sensor = to_ov6211_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
}

static const struct i2c_device_id ov6211_id[] = {
	{"ov6211", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ov6211_id);

static const struct of_device_id ov6211_dt_ids[] = {
	{ .compatible = "ovti,ov6211x" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov6211_dt_ids);

static struct i2c_driver ov6211_i2c_driver = {
	.driver = {
		.name  = "ov6211",
		.of_match_table	= ov6211_dt_ids,
	},
	.id_table = ov6211_id,
	.probe = ov6211_probe,
	.remove   = ov6211_remove,
};

module_i2c_driver(ov6211_i2c_driver);

MODULE_DESCRIPTION("OV6211 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
