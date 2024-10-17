// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Linaro

#include <linux/acpi.h>
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

#define OG0VE1B_REG_VALUE_08BIT		1
#define OG0VE1B_REG_VALUE_16BIT		2
#define OG0VE1B_REG_VALUE_24BIT		3

#define OG0VE1B_LINK_FREQ_500MHZ	500000000ULL
#define OG0VE1B_SCLK			120000000LL
#define OG0VE1B_MCLK			19200000
#define OG0VE1B_DATA_LANES		1
#define OG0VE1B_RGB_DEPTH		8

#define OG0VE1B_REG_CHIP_ID		0x300a
#define OG0VE1B_CHIP_ID			0xc75645

#define OG0VE1B_REG_MODE_SELECT		0x0100
#define OG0VE1B_MODE_STANDBY		0x00
#define OG0VE1B_MODE_STREAMING		0x01

#if 0
/* vertical-timings from sensor */
#define OG0VE1B_REG_VTS			0x380e
#endif

#define OG0VE1B_VTS_120FPS		0x0498
#define OG0VE1B_VTS_120FPS_MIN		0x0498
#define OG0VE1B_VTS_MAX			0x7fff

/* horizontal-timings from sensor */
#define OG0VE1B_REG_HTS			0x380c

/* Exposure controls from sensor */
#if 0
#define OG0VE1B_REG_EXPOSURE		0x3501
#endif
#define	OG0VE1B_EXPOSURE_MIN		1
#define OG0VE1B_EXPOSURE_MAX_MARGIN	14
#define	OG0VE1B_EXPOSURE_STEP		1

/* Analog gain controls from sensor */
#if 0
#define OG0VE1B_REG_ANALOG_GAIN		0x3508
#endif
#define	OG0VE1B_ANAL_GAIN_MIN		16
#define	OG0VE1B_ANAL_GAIN_MAX		248 /* Max = 15.5x */
#define	OG0VE1B_ANAL_GAIN_STEP		1

/* Digital gain controls from sensor */
#if 0
#define OG0VE1B_REG_DIG_GAIN		0x350a
#endif
#define OG0VE1B_DGTL_GAIN_MIN		1024
#define OG0VE1B_DGTL_GAIN_MAX		16384 /* Max = 16x */
#define OG0VE1B_DGTL_GAIN_STEP		1
#define OG0VE1B_DGTL_GAIN_DEFAULT	1024

/* Group Access */
#define OG0VE1B_REG_GROUP_ACCESS	0x3208
#define OG0VE1B_GROUP_HOLD_START	0x0
#define OG0VE1B_GROUP_HOLD_END		0x10
#define OG0VE1B_GROUP_HOLD_LAUNCH	0xa0

/* Test Pattern Control */
#define OG0VE1B_REG_TEST_PATTERN	0x5100
#define OG0VE1B_TEST_PATTERN_ENABLE	BIT(7)
#define OG0VE1B_TEST_PATTERN_BAR_SHIFT	2

#define to_og0ve1b(_sd)			container_of(_sd, struct og0ve1b, sd)

enum {
	OG0VE1B_LINK_FREQ_1000MBPS,
};

struct og0ve1b_reg {
	u16 address;
	u8 val;
};

struct og0ve1b_reg_list {
	u32 num_of_regs;
	const struct og0ve1b_reg *regs;
};

struct og0ve1b_link_freq_config {
	const struct og0ve1b_reg_list reg_list;
};

struct og0ve1b_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	/* Horizontal timining size */
	u32 hts;

	/* Default vertical timining size */
	u32 vts_def;

	/* Min vertical timining size */
	u32 vts_min;

	/* Link frequency needed for this resolution */
	u32 link_freq_index;

	/* Sensor register settings for this resolution */
	const struct og0ve1b_reg_list reg_list;
};

/* CHECK ME */
static const struct og0ve1b_reg mipi_data_rate_1000mbps[] = {
	{ 0x0103, 0x01 },
	{ 0x0100, 0x00 },
	/*
	{ 0x0303, 0x02 },
	{ 0x0304, 0x00 },
	{ 0x0305, 0xd2 },
	{ 0x0323, 0x02 },
	{ 0x0324, 0x01 },
	{ 0x0325, 0x77 },
	*/
};

static const struct og0ve1b_reg mode_640x480_regs[] = {
        {0x0103, 0x01},
	{0x0100, 0x00},
	{0x30a0, 0x02},
	{0x30a1, 0x00},
	{0x30a2, 0x48},
	{0x30a3, 0x34},
	{0x30a4, 0xf7},
	{0x30a5, 0x00},
	{0x3082, 0x32},
	{0x3083, 0x01},
	{0x301c, 0xf0},
	{0x301e, 0x0b},
	{0x3106, 0x10},
	{0x3708, 0x77},
	{0x3709, 0xf8},
	{0x3717, 0x00},
	{0x3782, 0x00},
	{0x3783, 0x47},
	{0x37a2, 0x00},
	{0x3500, 0x00},
	{0x3501, 0x00},
	{0x3502, 0x40},
	{0x3503, 0x07},
	{0x3509, 0x10},
	{0x350b, 0x10},
	{0x3600, 0x83},
	{0x3601, 0x21},
	{0x3602, 0xf1},
	{0x360a, 0x18},
	{0x360e, 0xb3},
	{0x3613, 0x20},
	{0x366a, 0x78},
	{0x3706, 0x63},
	{0x3713, 0x00},
	{0x3716, 0xb0},
	{0x37a1, 0x38},
	{0x3800, 0x00},
	{0x3801, 0x04},
	{0x3802, 0x00},
	{0x3803, 0x04},
	{0x3804, 0x02},
	{0x3805, 0x8b},
	{0x3806, 0x01},
	{0x3807, 0xeb},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x03},
	{0x380d, 0x18},
	{0x380e, 0x02},
	{0x380f, 0x38},
	{0x3811, 0x04},
	{0x3813, 0x04},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x00},
	{0x3821, 0x00},
	{0x3823, 0x04},
	{0x382a, 0x00},
	{0x382b, 0x03},
	{0x3840, 0x00},
	{0x389e, 0x00},
	{0x3c05, 0x08},
	{0x3c26, 0x02},
	{0x3c27, 0xc0},
	{0x3c28, 0x00},
	{0x3c29, 0x40},
	{0x3c2c, 0x00},
	{0x3c2d, 0x50},
	{0x3c2e, 0x02},
	{0x3c2f, 0x66},
	{0x3c33, 0x08},
	{0x3c35, 0x00},
	{0x3c36, 0x00},
	{0x3c37, 0x00},
	{0x3f52, 0x9b},
	{0x4001, 0x42},
	{0x4004, 0x08},
	{0x4005, 0x00},
	{0x4007, 0x28},
	{0x4009, 0x40},
	{0x4307, 0x30},
	{0x4500, 0x80},
	{0x4501, 0x02},
	{0x4502, 0x47},
	{0x4504, 0x7f},
	{0x4601, 0x48},
	{0x4800, 0x64},
	{0x4801, 0x0f},
	{0x4806, 0x2f},
	{0x4819, 0xaa},
	{0x4823, 0x3e},
	{0x5000, 0x85},
	{0x5e00, 0x0c},
	{0x3899, 0x09},
	{0x4f00, 0x64},
	{0x4f02, 0x0a},
	{0x4f05, 0x0e},
	{0x4f06, 0x11},
	{0x4f08, 0x0b},
	{0x4f0a, 0xc4},
	{0x4f20, 0x1f},
	{0x4f25, 0x10},
	{0x3016, 0x10},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x3019, 0x00},
	{0x301a, 0x00},
	{0x301b, 0x00},
	{0x301c, 0x72},
	{0x3037, 0x40},
	{0x4f2c, 0x00},
	{0x4f21, 0x00},
	{0x4f23, 0x00},
	{0x4f2a, 0x00},
	{0x3665, 0xe7},
	{0x3668, 0x48},
	{0x3671, 0x3c},
	{0x389a, 0x02},
	{0x389b, 0x00},
	{0x303c, 0xa0},
	{0x300f, 0xf0},
	{0x304b, 0x0f},
	{0x3662, 0x24},
	{0x3006, 0x40},
	{0x4f26, 0x45},
	{0x3607, 0x34},
	{0x3608, 0x01},
	{0x360a, 0x0c},
	{0x360b, 0x86},
	{0x360c, 0xcc},
	{0x3013, 0x00},
	{0x3083, 0x02},
	{0x3084, 0x12},
	{0x4601, 0x38},
	{0x366f, 0x3a},
	{0x3713, 0x19},
	{0x37a2, 0x00},
	{0x3f43, 0x27},
	{0x3f45, 0x27},
	{0x3f47, 0x32},
	{0x3f49, 0x3e},
	{0x3f4b, 0x20},
	{0x3f4d, 0x30},
	{0x4300, 0x3f},
	{0x4009, 0x10},
	{0x3f02, 0x68},
	{0x3700, 0x8c},
	{0x370b, 0x7e},
	{0x3f47, 0x35},
	{0x350b, 0x10},
	{0x3500, 0x00},
	{0x3501, 0x22},
	{0x3502, 0xa0},
};

static const char * const og0ve1b_test_pattern_menu[] = {
	"Disabled",
	"Standard Color Bar",
	"Top-Bottom Darker Color Bar",
	"Right-Left Darker Color Bar",
	"Bottom-Top Darker Color Bar"
};

static const s64 link_freq_menu_items[] = {
	OG0VE1B_LINK_FREQ_500MHZ,
};

static const struct og0ve1b_link_freq_config link_freq_configs[] = {
	[OG0VE1B_LINK_FREQ_1000MBPS] = {
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mipi_data_rate_1000mbps),
			.regs = mipi_data_rate_1000mbps,
		}
	}
};

static const struct og0ve1b_mode supported_modes[] = {
	{
		.width = 640,
		.height = 480,
		.hts = 848,
		// vts 1176
		.vts_def = OG0VE1B_VTS_120FPS,
		.vts_min = OG0VE1B_VTS_120FPS_MIN,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640x480_regs),
			.regs = mode_640x480_regs,
		},
		.link_freq_index = OG0VE1B_LINK_FREQ_1000MBPS,
	},
};

struct og0ve1b {
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
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;

	/* Current mode */
	const struct og0ve1b_mode *cur_mode;

	/* To serialize asynchronus callbacks */
	struct mutex mutex;
};

static u64 to_pixel_rate(u32 f_index)
{
	u64 pixel_rate = link_freq_menu_items[f_index] * 2 * OG0VE1B_DATA_LANES;

	do_div(pixel_rate, OG0VE1B_RGB_DEPTH);

	return pixel_rate;
}

static u64 to_pixels_per_line(u32 hts, u32 f_index)
{
	u64 ppl = hts * to_pixel_rate(f_index);

	do_div(ppl, OG0VE1B_SCLK);

	return ppl;
}

static int og0ve1b_read_reg(struct og0ve1b *og0ve1b, u16 reg, u16 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og0ve1b->sd);
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

static int og0ve1b_write_reg(struct og0ve1b *og0ve1b, u16 reg, u16 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og0ve1b->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << 8 * (4 - len), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int og0ve1b_write_reg_list(struct og0ve1b *og0ve1b,
				  const struct og0ve1b_reg_list *r_list)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og0ve1b->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < r_list->num_of_regs; i++) {
		ret = og0ve1b_write_reg(og0ve1b, r_list->regs[i].address, 1,
					r_list->regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "failed to write reg 0x%4.4x. error = %d",
					    r_list->regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static int og0ve1b_test_pattern(struct og0ve1b *og0ve1b, u32 pattern)
{
	if (pattern)
		pattern = (pattern - 1) << OG0VE1B_TEST_PATTERN_BAR_SHIFT |
			  OG0VE1B_TEST_PATTERN_ENABLE;

	return og0ve1b_write_reg(og0ve1b, OG0VE1B_REG_TEST_PATTERN,
				 OG0VE1B_REG_VALUE_08BIT, pattern);
}

static int og0ve1b_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct og0ve1b *og0ve1b = container_of(ctrl->handler,
					       struct og0ve1b, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&og0ve1b->sd);
	s64 exposure_max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Update max exposure while meeting expected vblanking */
		exposure_max = og0ve1b->cur_mode->height + ctrl->val -
			       OG0VE1B_EXPOSURE_MAX_MARGIN;
		__v4l2_ctrl_modify_range(og0ve1b->exposure,
					 og0ve1b->exposure->minimum,
					 exposure_max, og0ve1b->exposure->step,
					 exposure_max);
	}

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
		/*
	case V4L2_CID_ANALOGUE_GAIN:
		ret = og0ve1b_write_reg(og0ve1b, OG0VE1B_REG_ANALOG_GAIN,
					OG0VE1B_REG_VALUE_16BIT,
					ctrl->val << 4);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		ret = og0ve1b_write_reg(og0ve1b, OG0VE1B_REG_DIG_GAIN,
					OG0VE1B_REG_VALUE_24BIT,
					ctrl->val << 6);
		break;

	case V4L2_CID_EXPOSURE:
		ret = og0ve1b_write_reg(og0ve1b, OG0VE1B_REG_EXPOSURE,
					OG0VE1B_REG_VALUE_16BIT, ctrl->val);
		break;

	case V4L2_CID_VBLANK:
		ret = og0ve1b_write_reg(og0ve1b, OG0VE1B_REG_VTS,
					OG0VE1B_REG_VALUE_16BIT,
					og0ve1b->cur_mode->height + ctrl->val);
		break;
		*/

	case V4L2_CID_TEST_PATTERN:
		ret = og0ve1b_test_pattern(og0ve1b, ctrl->val);
		break;

	default:
		//ret = -EINVAL;
		ret = 0;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops og0ve1b_ctrl_ops = {
	.s_ctrl = og0ve1b_set_ctrl,
};

static int og0ve1b_init_controls(struct og0ve1b *og0ve1b)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 exposure_max, h_blank;
	int ret;

	ctrl_hdlr = &og0ve1b->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &og0ve1b->mutex;
	og0ve1b->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						    &og0ve1b_ctrl_ops,
						    V4L2_CID_LINK_FREQ,
						    ARRAY_SIZE
						    (link_freq_menu_items) - 1,
						    0, link_freq_menu_items);
	if (og0ve1b->link_freq)
		og0ve1b->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	og0ve1b->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops,
						V4L2_CID_PIXEL_RATE, 0,
						to_pixel_rate
						(OG0VE1B_LINK_FREQ_1000MBPS),
						1,
						to_pixel_rate
						(OG0VE1B_LINK_FREQ_1000MBPS));
	og0ve1b->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops,
					    V4L2_CID_VBLANK,
					    og0ve1b->cur_mode->vts_min -
					    og0ve1b->cur_mode->height,
					    OG0VE1B_VTS_MAX -
					    og0ve1b->cur_mode->height, 1,
					    og0ve1b->cur_mode->vts_def -
					    og0ve1b->cur_mode->height);
	h_blank = to_pixels_per_line(og0ve1b->cur_mode->hts,
				     og0ve1b->cur_mode->link_freq_index) -
				     og0ve1b->cur_mode->width;
	og0ve1b->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops,
					    V4L2_CID_HBLANK, h_blank, h_blank,
					    1, h_blank);
	if (og0ve1b->hblank)
		og0ve1b->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OG0VE1B_ANAL_GAIN_MIN, OG0VE1B_ANAL_GAIN_MAX,
			  OG0VE1B_ANAL_GAIN_STEP, OG0VE1B_ANAL_GAIN_MIN);
	v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  OG0VE1B_DGTL_GAIN_MIN, OG0VE1B_DGTL_GAIN_MAX,
			  OG0VE1B_DGTL_GAIN_STEP, OG0VE1B_DGTL_GAIN_DEFAULT);
	exposure_max = (og0ve1b->cur_mode->vts_def -
			OG0VE1B_EXPOSURE_MAX_MARGIN);
	og0ve1b->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      OG0VE1B_EXPOSURE_MIN,
					      exposure_max,
					      OG0VE1B_EXPOSURE_STEP,
					      exposure_max);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &og0ve1b_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(og0ve1b_test_pattern_menu) - 1,
				     0, 0, og0ve1b_test_pattern_menu);

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	og0ve1b->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static void og0ve1b_update_pad_format(const struct og0ve1b_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_Y8_1X8;
	fmt->field = V4L2_FIELD_NONE;
}

static int og0ve1b_start_streaming(struct og0ve1b *og0ve1b)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og0ve1b->sd);
	const struct og0ve1b_reg_list *reg_list;
	int link_freq_index, ret;

	link_freq_index = og0ve1b->cur_mode->link_freq_index;
	reg_list = &link_freq_configs[link_freq_index].reg_list;

	ret = og0ve1b_write_reg_list(og0ve1b, reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to set plls");
		return ret;
	}

	reg_list = &og0ve1b->cur_mode->reg_list;
	ret = og0ve1b_write_reg_list(og0ve1b, reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to set mode");
		return ret;
	}

	ret = __v4l2_ctrl_handler_setup(og0ve1b->sd.ctrl_handler);
	if (ret)
		return ret;

	ret = og0ve1b_write_reg(og0ve1b, OG0VE1B_REG_MODE_SELECT,
				OG0VE1B_REG_VALUE_08BIT,
				OG0VE1B_MODE_STREAMING);
	if (ret) {
		dev_err(&client->dev, "failed to set stream");
		return ret;
	}

	return 0;
}

static void og0ve1b_stop_streaming(struct og0ve1b *og0ve1b)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og0ve1b->sd);

	if (og0ve1b_write_reg(og0ve1b, OG0VE1B_REG_MODE_SELECT,
			      OG0VE1B_REG_VALUE_08BIT, OG0VE1B_MODE_STANDBY))
		dev_err(&client->dev, "failed to set stream");
}

static int og0ve1b_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&og0ve1b->mutex);
	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret) {
			mutex_unlock(&og0ve1b->mutex);
			return ret;
		}

		ret = og0ve1b_start_streaming(og0ve1b);
		if (ret) {
			enable = 0;
			og0ve1b_stop_streaming(og0ve1b);
			pm_runtime_put(&client->dev);
		}
	} else {
		og0ve1b_stop_streaming(og0ve1b);
		pm_runtime_put(&client->dev);
	}

	mutex_unlock(&og0ve1b->mutex);

	return ret;
}

static int og0ve1b_set_format(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);
	const struct og0ve1b_mode *mode;
	s32 vblank_def, h_blank;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width,
				      height, fmt->format.width,
				      fmt->format.height);

	mutex_lock(&og0ve1b->mutex);
	og0ve1b_update_pad_format(mode, &fmt->format);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;
	} else {
		og0ve1b->cur_mode = mode;
		__v4l2_ctrl_s_ctrl(og0ve1b->link_freq, mode->link_freq_index);
		__v4l2_ctrl_s_ctrl_int64(og0ve1b->pixel_rate,
					 to_pixel_rate(mode->link_freq_index));

		/* Update limits and set FPS to default */
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(og0ve1b->vblank,
					 mode->vts_min - mode->height,
					 OG0VE1B_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(og0ve1b->vblank, vblank_def);
		h_blank = to_pixels_per_line(mode->hts, mode->link_freq_index) -
			  mode->width;
		__v4l2_ctrl_modify_range(og0ve1b->hblank, h_blank, h_blank, 1,
					 h_blank);
	}

	mutex_unlock(&og0ve1b->mutex);

	return 0;
}

static int og0ve1b_get_format(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);

	mutex_lock(&og0ve1b->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt->format = *v4l2_subdev_state_get_format(sd_state,
							    fmt->pad);
	else
		og0ve1b_update_pad_format(og0ve1b->cur_mode, &fmt->format);

	mutex_unlock(&og0ve1b->mutex);

	return 0;
}

static int og0ve1b_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG8_1X8;

	return 0;
}

static int og0ve1b_enum_frame_size(struct v4l2_subdev *sd,
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

static int og0ve1b_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);

	mutex_lock(&og0ve1b->mutex);
	og0ve1b_update_pad_format(&supported_modes[0],
				  v4l2_subdev_state_get_format(fh->state, 0));
	mutex_unlock(&og0ve1b->mutex);

	return 0;
}

static const struct v4l2_subdev_video_ops og0ve1b_video_ops = {
	.s_stream = og0ve1b_set_stream,
};

static const struct v4l2_subdev_pad_ops og0ve1b_pad_ops = {
	.set_fmt = og0ve1b_set_format,
	.get_fmt = og0ve1b_get_format,
	.enum_mbus_code = og0ve1b_enum_mbus_code,
	.enum_frame_size = og0ve1b_enum_frame_size,
};

static const struct v4l2_subdev_ops og0ve1b_subdev_ops = {
	.video = &og0ve1b_video_ops,
	.pad = &og0ve1b_pad_ops,
};

static const struct media_entity_operations og0ve1b_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops og0ve1b_internal_ops = {
	.open = og0ve1b_open,
};

static int og0ve1b_identify_module(struct og0ve1b *og0ve1b)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og0ve1b->sd);
	int ret;
	u32 val;

	ret = og0ve1b_read_reg(og0ve1b, OG0VE1B_REG_CHIP_ID,
			       OG0VE1B_REG_VALUE_24BIT, &val);
	if (ret)
		return ret;

	if (val != OG0VE1B_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x",
			OG0VE1B_CHIP_ID, val);
		return -ENXIO;
	}

	return 0;
}

static int og0ve1b_check_hwcfg(struct og0ve1b *og0ve1b)
{
	struct i2c_client *client = v4l2_get_subdevdata(&og0ve1b->sd);
	struct device *dev = &client->dev;
	struct fwnode_handle *ep;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	u32 mclk;
	int ret;
	unsigned int i, j;

	if (!fwnode)
		return -ENXIO;

	ret = fwnode_property_read_u32(fwnode, "clock-frequency", &mclk);
	if (ret) {
		if (!og0ve1b->xvclk) {
			dev_err(dev, "can't get clock frequency");
			return ret;
		}

		mclk = clk_get_rate(og0ve1b->xvclk);
	}

	clk_set_rate(og0ve1b->xvclk, 24000000);

	/*
	if (mclk != OG0VE1B_MCLK) {
		dev_err(dev, "external clock %d is not supported", mclk);
		return -EINVAL;
	}
	*/

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OG0VE1B_DATA_LANES) {
		dev_err(dev, "number of CSI2 data lanes %d is not supported",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto check_hwcfg_error;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(dev, "no link frequencies defined");
		ret = -EINVAL;
		goto check_hwcfg_error;
	}

	for (i = 0; i < ARRAY_SIZE(link_freq_menu_items); i++) {
		for (j = 0; j < bus_cfg.nr_of_link_frequencies; j++) {
			if (link_freq_menu_items[i] ==
				bus_cfg.link_frequencies[j])
				break;
		}

		if (j == bus_cfg.nr_of_link_frequencies) {
			dev_err(dev, "no link frequency %lld supported",
				link_freq_menu_items[i]);
			ret = -EINVAL;
			goto check_hwcfg_error;
		}
	}

check_hwcfg_error:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* Power/clock management functions */
static int og0ve1b_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);
	int ret;

	if (og0ve1b->avdd) {
		ret = regulator_enable(og0ve1b->avdd);
		if (ret)
			return ret;
	}

	if (og0ve1b->dovdd) {
		ret = regulator_enable(og0ve1b->dovdd);
		if (ret)
			goto avdd_disable;
	}

	if (og0ve1b->dvdd) {
		ret = regulator_enable(og0ve1b->dvdd);
		if (ret)
			goto dovdd_disable;
	}

	gpiod_set_value_cansleep(og0ve1b->reset_gpio, 0);
	usleep_range(USEC_PER_MSEC, 2 * USEC_PER_MSEC);

	ret = clk_prepare_enable(og0ve1b->xvclk);
	if (ret)
		goto dvdd_disable;

	return 0;

dvdd_disable:
	if (og0ve1b->dvdd)
		regulator_disable(og0ve1b->dvdd);
dovdd_disable:
	if (og0ve1b->dovdd)
		regulator_disable(og0ve1b->dovdd);
avdd_disable:
	if (og0ve1b->avdd)
		regulator_disable(og0ve1b->avdd);

	return ret;
}

static int og0ve1b_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);

	clk_disable_unprepare(og0ve1b->xvclk);

	gpiod_set_value_cansleep(og0ve1b->reset_gpio, 1);

	if (og0ve1b->dvdd)
		regulator_disable(og0ve1b->dvdd);

	if (og0ve1b->dovdd)
		regulator_disable(og0ve1b->dovdd);

	if (og0ve1b->avdd)
		regulator_disable(og0ve1b->avdd);

	return 0;
}

static void og0ve1b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(&client->dev);
	mutex_destroy(&og0ve1b->mutex);
}

static int og0ve1b_probe(struct i2c_client *client)
{
	struct og0ve1b *og0ve1b;
	int ret;

	og0ve1b = devm_kzalloc(&client->dev, sizeof(*og0ve1b), GFP_KERNEL);
	if (!og0ve1b)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&og0ve1b->sd, client, &og0ve1b_subdev_ops);

	og0ve1b->xvclk = devm_clk_get_optional(&client->dev, NULL);
	if (IS_ERR(og0ve1b->xvclk)) {
		ret = PTR_ERR(og0ve1b->xvclk);
		dev_err(&client->dev, "failed to get xvclk clock: %d\n", ret);
		return ret;
	}

	ret = og0ve1b_check_hwcfg(og0ve1b);
	if (ret) {
		dev_err(&client->dev, "failed to check HW configuration: %d",
			ret);
		return ret;
	}

	og0ve1b->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						      GPIOD_OUT_LOW);
	if (IS_ERR(og0ve1b->reset_gpio)) {
		dev_err(&client->dev, "cannot get reset GPIO\n");
		return PTR_ERR(og0ve1b->reset_gpio);
	}

	og0ve1b->avdd = devm_regulator_get_optional(&client->dev, "avdd");
	if (IS_ERR(og0ve1b->avdd)) {
		ret = PTR_ERR(og0ve1b->avdd);
		if (ret != -ENODEV) {
			dev_err_probe(&client->dev, ret,
				      "Failed to get 'avdd' regulator\n");
			return ret;
		}

		og0ve1b->avdd = NULL;
	}

	og0ve1b->dovdd = devm_regulator_get_optional(&client->dev, "dovdd");
	if (IS_ERR(og0ve1b->dovdd)) {
		ret = PTR_ERR(og0ve1b->dovdd);
		if (ret != -ENODEV) {
			dev_err_probe(&client->dev, ret,
				      "Failed to get 'dovdd' regulator\n");
			return ret;
		}

		og0ve1b->dovdd = NULL;
	}

	og0ve1b->dvdd = devm_regulator_get_optional(&client->dev, "dvdd");
	if (IS_ERR(og0ve1b->dvdd)) {
		ret = PTR_ERR(og0ve1b->dvdd);
		if (ret != -ENODEV) {
			dev_err_probe(&client->dev, ret,
				      "Failed to get 'dvdd' regulator\n");
			return ret;
		}

		og0ve1b->dvdd = NULL;
	}

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = og0ve1b_power_on(&client->dev);
	if (ret)
		return ret;

	ret = og0ve1b_identify_module(og0ve1b);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		goto power_off;
	}

	mutex_init(&og0ve1b->mutex);
	og0ve1b->cur_mode = &supported_modes[0];
	ret = og0ve1b_init_controls(og0ve1b);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	og0ve1b->sd.internal_ops = &og0ve1b_internal_ops;
	og0ve1b->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	og0ve1b->sd.entity.ops = &og0ve1b_subdev_entity_ops;
	og0ve1b->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	og0ve1b->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&og0ve1b->sd.entity, 1, &og0ve1b->pad);
	if (ret) {
		dev_err(&client->dev, "failed to init entity pads: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&og0ve1b->sd);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d",
			ret);
		goto probe_error_media_entity_cleanup;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

probe_error_media_entity_cleanup:
	media_entity_cleanup(&og0ve1b->sd.entity);

probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(og0ve1b->sd.ctrl_handler);
	mutex_destroy(&og0ve1b->mutex);

power_off:
	og0ve1b_power_off(&client->dev);

	return ret;
}

static const struct dev_pm_ops og0ve1b_pm_ops = {
	SET_RUNTIME_PM_OPS(og0ve1b_power_off, og0ve1b_power_on, NULL)
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id og0ve1b_acpi_ids[] = {
	{"OVTI01AC"},
	{}
};

MODULE_DEVICE_TABLE(acpi, og0ve1b_acpi_ids);
#endif

static const struct of_device_id og0ve1b_of_match[] = {
	{ .compatible = "ovti,og0ve1b" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, og0ve1b_of_match);

static struct i2c_driver og0ve1b_i2c_driver = {
	.driver = {
		.name = "og0ve1b",
		.pm = &og0ve1b_pm_ops,
		.acpi_match_table = ACPI_PTR(og0ve1b_acpi_ids),
		.of_match_table = og0ve1b_of_match,
	},
	.probe = og0ve1b_probe,
	.remove = og0ve1b_remove,
};

module_i2c_driver(og0ve1b_i2c_driver);

MODULE_AUTHOR("Shawn Tu");
MODULE_DESCRIPTION("OmniVision OG0VE1B sensor driver");
MODULE_LICENSE("GPL v2");
