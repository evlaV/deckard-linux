// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Linaro Ltd

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define OV6710_REG_VALUE_08BIT		1
#define OV6710_REG_VALUE_16BIT		2
#define OV6710_REG_VALUE_24BIT		3

#define OV6710_LINK_FREQ_240MHZ		240000000ULL
#define OV6710_MCLK			24000000
#define OV6710_DATA_LANES		1

#define OV6710_REG_CHIP_ID		0x300a
#define OV6710_CHIP_ID			0x6710

#define OV6710_REG_MODE_SELECT		0x0100
#define OV6710_MODE_STANDBY		0x00
#define OV6710_MODE_STREAMING		0x01

/* Exposure controls from sensor */
#define OV6710_REG_EXPOSURE		0x3500
#define OV6710_EXPOSURE_MIN		1
#define OV6710_EXPOSURE_MAX_MARGIN	4
#define OV6710_EXPOSURE_STEP		1
#define OV6710_EXPOSURE_DEFAULT		210

/* Analogue gain controls from sensor */
#define OV6710_REG_ANALOGUE_GAIN	0x350a
#define OV6710_ANALOGUE_GAIN_MIN	1
#define OV6710_ANALOGUE_GAIN_MAX	0x3ff
#define OV6710_ANALOGUE_GAIN_STEP	1
#define OV6710_ANALOGUE_GAIN_DEFAULT	0xa0

#define to_ov6710(_sd)			container_of(_sd, struct ov6710, sd)

struct ov6710_reg {
	u16 address;
	u8 val;
};

struct ov6710_reg_list {
	u32 num_of_regs;
	const struct ov6710_reg *regs;
};

struct ov6710_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	/* Horizontal timining size */
	u32 hts;

	/* Default vertical timining size */
	u32 vts;

	/* Sensor register settings for this resolution */
	const struct ov6710_reg_list reg_list;
};

static const s64 link_freq_menu_items[] = {
	OV6710_LINK_FREQ_240MHZ,
};

static const struct ov6710_reg mode_400x400_regs[] = {
	{ 0x0103, 0x01 },
	{ 0x0100, 0x00 },
	{ 0x3005, 0x08 },
	{ 0x3013, 0x12 },
	{ 0x3014, 0x04 },
	{ 0x3016, 0x10 },
	{ 0x3017, 0x00 },
	{ 0x3018, 0x00 },
	{ 0x301a, 0x00 },
	{ 0x301b, 0x00 },
	{ 0x301c, 0x00 },
	{ 0x3037, 0xf0 },
	{ 0x3080, 0x01 },
	{ 0x3081, 0x00 },
	{ 0x3082, 0x01 },
	{ 0x3098, 0x04 },
	{ 0x3099, 0x28 },
	{ 0x309a, 0x06 },
	{ 0x309b, 0x04 },
	{ 0x309c, 0x00 },
	{ 0x309d, 0x00 },
	{ 0x309e, 0x01 },
	{ 0x309f, 0x00 },
	{ 0x30b0, 0x08 },
	{ 0x30b1, 0x02 },
	{ 0x30b2, 0x00 },
	{ 0x30b3, 0x28 },
	{ 0x30b4, 0x02 },
	{ 0x30b5, 0x00 },
	{ 0x3106, 0xd9 },
	{ 0x3500, 0x00 },
	{ 0x3501, 0x0d },
	{ 0x3502, 0x20 },
	{ 0x3503, 0x07 },
	{ 0x3509, 0x10 },
	{ 0x350b, 0xa0 },
	{ 0x3600, 0xfc },
	{ 0x3620, 0xb7 },
	{ 0x3621, 0x05 },
	{ 0x3626, 0x31 },
	{ 0x3627, 0x40 },
	{ 0x3632, 0xa3 },
	{ 0x3633, 0x34 },
	{ 0x3634, 0x40 },
	{ 0x3636, 0x00 },
	{ 0x3660, 0x80 },
	{ 0x3662, 0x03 },
	{ 0x3664, 0xf0 },
	{ 0x366a, 0x10 },
	{ 0x366b, 0x06 },
	{ 0x3680, 0xf4 },
	{ 0x3681, 0x50 },
	{ 0x3682, 0x00 },
	{ 0x3708, 0x20 },
	{ 0x3709, 0x40 },
	{ 0x370d, 0x03 },
	{ 0x373b, 0x02 },
	{ 0x373c, 0x08 },
	{ 0x3742, 0x00 },
	{ 0x3744, 0x16 },
	{ 0x3745, 0x08 },
	{ 0x3781, 0xfc },
	{ 0x3788, 0x00 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x04 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x04 },
	{ 0x3804, 0x01 },
	{ 0x3805, 0x9b },
	{ 0x3806, 0x01 },
	{ 0x3807, 0x9b },
	{ 0x3808, 0x01 },
	{ 0x3809, 0x90 },
	{ 0x380a, 0x01 },
	{ 0x380b, 0x90 },
	{ 0x380c, 0x05 },
	{ 0x380d, 0xf2 },
	{ 0x380e, 0x01 },
	{ 0x380f, 0xb6 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x04 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x04 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3820, 0x00 },
	{ 0x3821, 0x00 },
	{ 0x382b, 0xfa },
	{ 0x382f, 0x04 },
	{ 0x3832, 0x00 },
	{ 0x3833, 0x05 },
	{ 0x3834, 0x00 },
	{ 0x3835, 0x05 },
	{ 0x3882, 0x04 },
	{ 0x3883, 0x00 },
	{ 0x38a4, 0x10 },
	{ 0x38a5, 0x00 },
	{ 0x38b1, 0x03 },
	{ 0x3b80, 0x00 },
	{ 0x3b81, 0xff },
	{ 0x3b82, 0x10 },
	{ 0x3b83, 0x00 },
	{ 0x3b84, 0x08 },
	{ 0x3b85, 0x00 },
	{ 0x3b86, 0x01 },
	{ 0x3b87, 0x00 },
	{ 0x3b88, 0x00 },
	{ 0x3b89, 0x00 },
	{ 0x3b8a, 0x00 },
	{ 0x3b8b, 0x05 },
	{ 0x3b8c, 0x00 },
	{ 0x3b8d, 0x00 },
	{ 0x3b8e, 0x01 },
	{ 0x3b8f, 0xb2 },
	{ 0x3b94, 0x05 },
	{ 0x3b95, 0xf2 },
	{ 0x3b96, 0xc0 },
	{ 0x4004, 0x04 },
	{ 0x404e, 0x01 },
	{ 0x4801, 0x0f },
	{ 0x4806, 0x0f },
	{ 0x4837, 0x43 },
	{ 0x5a08, 0x00 },
	{ 0x5a01, 0x00 },
	{ 0x5a03, 0x00 },
	{ 0x5a04, 0x10 },
	{ 0x5a05, 0xa0 },
	{ 0x5a06, 0x0c },
	{ 0x5a07, 0x78 },
};

static const struct ov6710_mode supported_modes[] = {
	{
		.width = 400,
		.height = 400,
		.hts = 1522,
		.vts = 438,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_400x400_regs),
			.regs = mode_400x400_regs,
		},
	},
};

struct ov6710 {
	struct clk *xvclk;
	struct gpio_desc *reset_gpio;
	struct regulator *avdd;
	struct regulator *dovdd;
	struct regulator *dvdd;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;

	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;

	/* Current mode */
	const struct ov6710_mode *cur_mode;

	/* To serialize asynchronous callbacks */
	struct mutex mutex;
};

static int ov6710_read_reg(struct ov6710 *ov6710, u16 reg, u16 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov6710->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[4] = {0};
	int ret;

	if (len > 4)
		return -EINVAL;

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

static int ov6710_write_reg(struct ov6710 *ov6710, u16 reg, u16 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov6710->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << 8 * (4 - len), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int ov6710_write_reg_list(struct ov6710 *ov6710,
				 const struct ov6710_reg_list *r_list)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov6710->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < r_list->num_of_regs; i++) {
		ret = ov6710_write_reg(ov6710, r_list->regs[i].address, 1,
					   r_list->regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
						"failed to write reg 0x%4.4x. error = %d\n",
						r_list->regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static int ov6710_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov6710 *ov6710 = container_of(ctrl->handler, struct ov6710,
						 ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ov6710->sd);
	int ret;

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov6710_write_reg(ov6710, OV6710_REG_ANALOGUE_GAIN,
					   OV6710_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ov6710_write_reg(ov6710, OV6710_REG_EXPOSURE,
					   OV6710_REG_VALUE_24BIT, ctrl->val << 4);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov6710_ctrl_ops = {
	.s_ctrl = ov6710_set_ctrl,
};

static int ov6710_init_controls(struct ov6710 *ov6710)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 exposure_max;
	int ret;

	ctrl_hdlr = &ov6710->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 3);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &ov6710->mutex;

	ov6710->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &ov6710_ctrl_ops,
				V4L2_CID_LINK_FREQ,
				ARRAY_SIZE(link_freq_menu_items) - 1,
				0, link_freq_menu_items);
	if (ov6710->link_freq)
		ov6710->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(ctrl_hdlr, &ov6710_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
		OV6710_ANALOGUE_GAIN_MIN, OV6710_ANALOGUE_GAIN_MAX,
		OV6710_ANALOGUE_GAIN_STEP, OV6710_ANALOGUE_GAIN_DEFAULT);

	exposure_max = (ov6710->cur_mode->vts - OV6710_EXPOSURE_MAX_MARGIN);
	ov6710->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &ov6710_ctrl_ops,
				V4L2_CID_EXPOSURE,
				OV6710_EXPOSURE_MIN, exposure_max,
				OV6710_EXPOSURE_STEP, OV6710_EXPOSURE_DEFAULT);

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	ov6710->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static void ov6710_update_pad_format(const struct ov6710_mode *mode,
					 struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_SGRBG8_1X8;
	fmt->field = V4L2_FIELD_NONE;
}

static int ov6710_start_streaming(struct ov6710 *ov6710)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov6710->sd);
	const struct ov6710_reg_list *reg_list;
	int ret;

	reg_list = &ov6710->cur_mode->reg_list;
	ret = ov6710_write_reg_list(ov6710, reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to set mode\n");
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(ov6710->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = ov6710_write_reg(ov6710, OV6710_REG_MODE_SELECT,
				   OV6710_REG_VALUE_08BIT,
				   OV6710_MODE_STREAMING);
	if (ret) {
		dev_err(&client->dev, "failed to start streaming\n");
		return ret;
	}

	return 0;
}

static void ov6710_stop_streaming(struct ov6710 *ov6710)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov6710->sd);

	if (ov6710_write_reg(ov6710, OV6710_REG_MODE_SELECT,
				 OV6710_REG_VALUE_08BIT, OV6710_MODE_STANDBY))
		dev_err(&client->dev, "failed to stop streaming\n");
}

static int ov6710_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov6710 *ov6710 = to_ov6710(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&ov6710->mutex);

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret) {
			mutex_unlock(&ov6710->mutex);
			return ret;
		}

		ret = ov6710_start_streaming(ov6710);
		if (!ret) {
			mutex_unlock(&ov6710->mutex);
			return 0;
		}
	}

	ov6710_stop_streaming(ov6710);
	pm_runtime_put(&client->dev);

	mutex_unlock(&ov6710->mutex);

	return ret;
}

static int ov6710_set_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct ov6710 *ov6710 = to_ov6710(sd);
	const struct ov6710_mode *mode;

	mode = v4l2_find_nearest_size(supported_modes,
					  ARRAY_SIZE(supported_modes), width, height,
					  fmt->format.width, fmt->format.height);

	mutex_lock(&ov6710->mutex);

	ov6710_update_pad_format(mode, &fmt->format);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;
	else
		ov6710->cur_mode = mode;

	mutex_unlock(&ov6710->mutex);

	return 0;
}

static int ov6710_get_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct ov6710 *ov6710 = to_ov6710(sd);

	mutex_lock(&ov6710->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state, fmt->pad);
	else
		ov6710_update_pad_format(ov6710->cur_mode, &fmt->format);

	mutex_unlock(&ov6710->mutex);

	return 0;
}

static int ov6710_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG8_1X8;

	return 0;
}

static int ov6710_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG8_1X8)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov6710_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov6710 *ov6710 = to_ov6710(sd);

	mutex_lock(&ov6710->mutex);

	ov6710_update_pad_format(&supported_modes[0],
				 v4l2_subdev_state_get_format(fh->state, 0));

	mutex_unlock(&ov6710->mutex);

	return 0;
}

static const struct v4l2_subdev_video_ops ov6710_video_ops = {
	.s_stream = ov6710_set_stream,
};

static const struct v4l2_subdev_pad_ops ov6710_pad_ops = {
	.set_fmt = ov6710_set_format,
	.get_fmt = ov6710_get_format,
	.enum_mbus_code = ov6710_enum_mbus_code,
	.enum_frame_size = ov6710_enum_frame_size,
};

static const struct v4l2_subdev_ops ov6710_subdev_ops = {
	.video = &ov6710_video_ops,
	.pad = &ov6710_pad_ops,
};

static const struct media_entity_operations ov6710_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops ov6710_internal_ops = {
	.open = ov6710_open,
};

static int ov6710_identify_module(struct ov6710 *ov6710)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov6710->sd);
	int ret;
	u32 val;

	ret = ov6710_read_reg(ov6710, OV6710_REG_CHIP_ID,
				  OV6710_REG_VALUE_16BIT, &val);
	if (ret)
		return ret;

	if (val != OV6710_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			OV6710_CHIP_ID, val);
		return -ENXIO;
	} else {
		dev_err(&client->dev, "chip id match: %x=%x\n",
		OV6710_CHIP_ID, val);
	}

	return 0;
}

static int ov6710_check_hwcfg(struct ov6710 *ov6710)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ov6710->sd);
	struct device *dev = &client->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	u32 mclk;
	int ret;

	if (!fwnode)
		return -ENXIO;

	ret = fwnode_property_read_u32(fwnode, "clock-frequency", &mclk);
	if (!ret) {
		if (mclk != OV6710_MCLK) {
			dev_err(dev, "clock frequency %u is not supported\n",
				mclk);
			return -EINVAL;
		}
	}

	clk_set_rate(ov6710->xvclk, OV6710_MCLK);

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OV6710_DATA_LANES) {
		dev_err(dev, "number of CSI2 data lanes %d is not supported\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto check_hwcfg_error;
	}

check_hwcfg_error:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int ov6710_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov6710 *ov6710 = to_ov6710(sd);
	int ret;

	if (ov6710->avdd) {
		ret = regulator_enable(ov6710->avdd);
		if (ret)
			return ret;
	}

	if (ov6710->dovdd) {
		ret = regulator_enable(ov6710->dovdd);
		if (ret)
			goto avdd_disable;
	}

	if (ov6710->dvdd) {
		ret = regulator_enable(ov6710->dvdd);
		if (ret)
			goto dovdd_disable;
	}

	gpiod_set_value_cansleep(ov6710->reset_gpio, 0);
	usleep_range(10 * USEC_PER_MSEC, 15 * USEC_PER_MSEC);

	ret = clk_prepare_enable(ov6710->xvclk);
	if (ret)
		goto dvdd_disable;

	return 0;

dvdd_disable:
	if (ov6710->dvdd)
		regulator_disable(ov6710->dvdd);
dovdd_disable:
	if (ov6710->dovdd)
		regulator_disable(ov6710->dovdd);
avdd_disable:
	if (ov6710->avdd)
		regulator_disable(ov6710->avdd);

	return ret;
}

static int ov6710_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov6710 *ov6710 = to_ov6710(sd);

	clk_disable_unprepare(ov6710->xvclk);

	gpiod_set_value_cansleep(ov6710->reset_gpio, 1);

	if (ov6710->dvdd)
		regulator_disable(ov6710->dvdd);

	if (ov6710->dovdd)
		regulator_disable(ov6710->dovdd);

	if (ov6710->avdd)
		regulator_disable(ov6710->avdd);

	return 0;
}

static void ov6710_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov6710 *ov6710 = to_ov6710(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(&client->dev);
	mutex_destroy(&ov6710->mutex);
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov6710 *ov6710 = to_ov6710(sd);
	buf[0] = regulator_is_enabled(ov6710->dvdd) ? '1' : '0';
	buf[1] = '\n';
	return 2;
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
	  const char *buf, size_t count)
{
	int ret;
	int enable;

	ret = sscanf(buf, "%d", &enable);

	if ( ret != 1){
		pr_debug("Invalid number of params for enable\n");
		return -1;
	}

	if ( enable ) {
		ov6710_power_on(dev);
	} else {
		ov6710_power_off(dev);
	}

	return count;
}
static DEVICE_ATTR_RW(enable);

static struct attribute *ov6710_attrs[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group ov6710_attr_group = {
	.name = "ov6710",
	.attrs = ov6710_attrs,
};

static int ov6710_probe(struct i2c_client *client)
{
	struct ov6710 *ov6710;
	int ret;

	ov6710 = devm_kzalloc(&client->dev, sizeof(*ov6710), GFP_KERNEL);
	if (!ov6710)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&ov6710->sd, client, &ov6710_subdev_ops);

	ov6710->xvclk = devm_clk_get_optional(&client->dev, NULL);
	if (IS_ERR(ov6710->xvclk)) {
		ret = PTR_ERR(ov6710->xvclk);
		dev_err(&client->dev, "failed to get xvclk clock: %d\n", ret);
		return ret;
	}

	ret = ov6710_check_hwcfg(ov6710);
	if (ret) {
		dev_err(&client->dev, "failed to check HW configuration: %d",
			ret);
		return ret;
	}

	ov6710->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(ov6710->reset_gpio)) {
		dev_err(&client->dev, "cannot get reset GPIO\n");
		return PTR_ERR(ov6710->reset_gpio);
	}

	ov6710->avdd = devm_regulator_get_optional(&client->dev, "avdd");
	if (IS_ERR(ov6710->avdd)) {
		ret = PTR_ERR(ov6710->avdd);
		if (ret != -ENODEV) {
			dev_err_probe(&client->dev, ret,
					  "Failed to get 'avdd' regulator\n");
			return ret;
		}

		ov6710->avdd = NULL;
	}

	ov6710->dovdd = devm_regulator_get_optional(&client->dev, "dovdd");
	if (IS_ERR(ov6710->dovdd)) {
		ret = PTR_ERR(ov6710->dovdd);
		if (ret != -ENODEV) {
			dev_err_probe(&client->dev, ret,
					  "Failed to get 'dovdd' regulator\n");
			return ret;
		}

		ov6710->dovdd = NULL;
	}

	ov6710->dvdd = devm_regulator_get_optional(&client->dev, "dvdd");
	if (IS_ERR(ov6710->dvdd)) {
		ret = PTR_ERR(ov6710->dvdd);
		if (ret != -ENODEV) {
			dev_err_probe(&client->dev, ret,
					  "Failed to get 'dvdd' regulator\n");
			return ret;
		}

		ov6710->dvdd = NULL;
	}

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = ov6710_power_on(&client->dev);
	if (ret)
		return ret;

	ret = ov6710_identify_module(ov6710);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d\n", ret);
		goto power_off;
	}

	mutex_init(&ov6710->mutex);
	ov6710->cur_mode = &supported_modes[0];
	ret = ov6710_init_controls(ov6710);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d\n", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ov6710->sd.internal_ops = &ov6710_internal_ops;
	ov6710->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov6710->sd.entity.ops = &ov6710_subdev_entity_ops;
	ov6710->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov6710->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ov6710->sd.entity, 1, &ov6710->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init entity pads: %d\n", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&ov6710->sd);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d\n",
			ret);
		goto probe_error_media_entity_cleanup;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	dev_set_drvdata(&client->dev, ov6710);

	ret = sysfs_create_group(&client->dev.kobj, &ov6710_attr_group);
	if (ret < 0) {
		dev_err(&client->dev, "failed to create ov6710 sysfs files\n");
		return ret;
	}

	return 0;

probe_error_media_entity_cleanup:
	media_entity_cleanup(&ov6710->sd.entity);

probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(ov6710->sd.ctrl_handler);
	mutex_destroy(&ov6710->mutex);

power_off:
	ov6710_power_off(&client->dev);

	return ret;
}

static const struct dev_pm_ops ov6710_pm_ops = {
	SET_RUNTIME_PM_OPS(ov6710_power_off, ov6710_power_on, NULL)
};

static const struct of_device_id ov6710_of_match[] = {
	{ .compatible = "ovti,ov6710" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov6710_of_match);

static struct i2c_driver ov6710_i2c_driver = {
	.driver = {
		.name = "ov6710",
		.pm = &ov6710_pm_ops,
		.of_match_table = ov6710_of_match,
	},
	.probe = ov6710_probe,
	.remove = ov6710_remove,
};

module_i2c_driver(ov6710_i2c_driver);

MODULE_AUTHOR("Vladimir Zapolskiy <vladimir.zapolskiy@linaro.org>");
MODULE_DESCRIPTION("OmniVision OV6710 sensor driver");
MODULE_LICENSE("GPL");
