// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024-2025 Linaro Ltd

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/units.h>
#include <linux/workqueue.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define OG0VE1B_LINK_FREQ_500MHZ	(500 * HZ_PER_MHZ)
#define OG0VE1B_MCLK_FREQ_24MHZ		(24 * HZ_PER_MHZ)

#define OG0VE1B_REG_CHIP_ID		CCI_REG24(0x300a)
#define OG0VE1B_CHIP_ID			0xc75645

#define OG0VE1B_REG_MODE_SELECT		CCI_REG8(0x0100)
#define OG0VE1B_MODE_STANDBY		0x00
#define OG0VE1B_MODE_STREAMING		BIT(0)

#define OG0VE1B_REG_SOFTWARE_RST	CCI_REG8(0x0103)
#define OG0VE1B_SOFTWARE_RST		BIT(0)

/* Exposure controls from sensor */
#define OG0VE1B_REG_EXPOSURE		CCI_REG24(0x3500)
#define OG0VE1B_EXPOSURE_MIN		1
#define OG0VE1B_EXPOSURE_MAX_MARGIN	14
#define OG0VE1B_EXPOSURE_STEP		1
#define OG0VE1B_EXPOSURE_DEFAULT	554

/* Analogue gain controls from sensor */
#define OG0VE1B_REG_ANALOGUE_GAIN	CCI_REG16(0x350a)
#define OG0VE1B_ANALOGUE_GAIN_MIN	1
#define OG0VE1B_ANALOGUE_GAIN_MAX	0x1ff
#define OG0VE1B_ANALOGUE_GAIN_STEP	1
#define OG0VE1B_ANALOGUE_GAIN_DEFAULT	16

/* Vertical timing size */
#define OG0VE1B_REG_VTS			CCI_REG16(0x380e)
#define OG0VE1B_VTS_MAX			0xffff

/* Test pattern */
#define OG0VE1B_REG_PRE_ISP		CCI_REG8(0x5e00)
#define OG0VE1B_TEST_PATTERN_ENABLE	BIT(7)

#define to_og0ve1b(_sd)			container_of(_sd, struct og0ve1b, sd)

static const s64 og0ve1b_link_freq_menu[] = {
	OG0VE1B_LINK_FREQ_500MHZ,
};

struct og0ve1b_reg_list {
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct og0ve1b_mode {
	u32 width;	/* Frame width in pixels */
	u32 height;	/* Frame height in pixels */
	u32 hts;	/* Horizontal timing size */
	u32 vts;	/* Default vertical timing size */
	u32 bpp;	/* Bits per pixel */

	const struct og0ve1b_reg_list reg_list;	/* Sensor register setting */
};

static const char * const og0ve1b_test_pattern_menu[] = {
	"Disabled",
	"Vertical Colour Bars",
};

static const char * const og0ve1b_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OG0VE1B_NUM_SUPPLIES	ARRAY_SIZE(og0ve1b_supply_names)

#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
/*
 * Vendor interface, compatible with the out-of-tree og0ve10-a deckard driver.
 * The ioctl numbers, the structure layouts and the control IDs are an ABI
 * shared with the deckard userspace and the ISP, they
 * are the same as the ones of the og01a1b deckard interface.
 */

/*
 * Workqueues of the register group updates, provided by the og01a1b vendor
 * interface or by the og01a1b-a deckard driver.
 */
extern struct workqueue_struct *sensor_workqueues[2];

#ifndef V4L2_CID_METADATA_CB_ADDR
#define V4L2_CID_METADATA_CB_ADDR	(V4L2_CID_IMAGE_PROC_CLASS_BASE + 6)
#endif
#ifndef V4L2_CID_METADATA_SIZE
#define V4L2_CID_METADATA_SIZE		(V4L2_CID_IMAGE_PROC_CLASS_BASE + 7)
#endif
#ifndef V4L2_CID_SENSOR_SOF_CB_ADDR
#define V4L2_CID_SENSOR_SOF_CB_ADDR	(V4L2_CID_IMAGE_PROC_CLASS_BASE + 8)
#endif

struct og0ve1b_deckard_register {
	u16 reg_addr;
	u32 reg_value;
	u8 len;
};

struct og0ve1b_deckard_timing_adjustment {
	__u16 new_rows;
	__u16 rows_to_restore;
	__u16 new_global_row;
	__u16 global_row_to_restore;
	__u8 group_to_adjust;
	__u8 number_of_frames_to_hold_adjustment;
};

#define VIDIOC_OG0VE1B_PRV_G_METADATA_VERSION \
	_IOR('V', BASE_VIDIOC_PRIVATE + 1, u32)
/* Argument: register address in bits [15:0], value in bits [23:16] */
#define VIDIOC_OG0VE1B_PRV_S_REG \
	_IO('V', BASE_VIDIOC_PRIVATE + 3)
/* Argument: as above, and group in bits [27:24] */
#define VIDIOC_OG0VE1B_PRV_S_GROUP_REG \
	_IO('V', BASE_VIDIOC_PRIVATE + 4)
/* Argument: group mode */
#define VIDIOC_OG0VE1B_PRV_S_GROUP_MODE \
	_IO('V', BASE_VIDIOC_PRIVATE + 5)
#define VIDIOC_OG0VE1B_PRV_G_DRIVER_VERSION \
	_IOR('V', BASE_VIDIOC_PRIVATE + 6, u32)
#define VIDIOC_OG0VE1B_PRV_S_TIMING_ADJUSTMENT \
	_IOW('V', BASE_VIDIOC_PRIVATE + 7, \
	     struct og0ve1b_deckard_timing_adjustment)
#define VIDIOC_OG0VE1B_PRV_G_REGISTER \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 8, struct og0ve1b_deckard_register)
/* Argument: 1 if the sensors are in sync, 0 otherwise */
#define VIDIOC_OG0VE1B_PRV_S_IN_SYNC \
	_IO('V', BASE_VIDIOC_PRIVATE + 9)
/* Argument: 1 if the streams of two sensors are interleaved, 0 otherwise */
#define VIDIOC_OG0VE1B_PRV_S_VC_INT \
	_IO('V', BASE_VIDIOC_PRIVATE + 10)
/* Argument: 1 to restart streaming on all sensors */
#define VIDIOC_OG0VE1B_PRV_S_SYNCHRONIZED_RESET \
	_IO('V', BASE_VIDIOC_PRIVATE + 11)

#define OG0VE1B_METADATA_VERSION	((3 << 16) | 1)
#define OG0VE1B_DRIVER_VERSION	((1 << 16) | 1)

#define OG0VE1B_NUM_GROUPS	3
#define OG0VE1B_GROUP_MAX_REGS	20
#define OG0VE1B_HISTORY		5

/*
 * Frame metadata buffer of the deckard driver. Only the history of the
 * register group updates is filled in, the rest of the buffer is zero.
 */
#define OG0VE1B_METADATA_SIZE		1208
#define OG0VE1B_METADATA_HISTORY_OFFSET	24

#define OG0VE1B_REG_GLOBAL_ROW		CCI_REG16(0x382c)

/* Broadcast address of all sensors on the bus */
#define OG0VE1B_DUAL_I2C_ADDR		0x70

/* Sensor of virtual channel 0 in the interleaved mode */
#define OG0VE1B_LEFT_SENSOR_ID		"upper_left"

/* Groups 0 and 1 are written alternately in this group mode */
#define OG0VE1B_GROUP_MODE_2_GROUPS	2

struct og0ve1b_deckard_register_set {
	u16 addr[OG0VE1B_GROUP_MAX_REGS];
	u8 val[OG0VE1B_GROUP_MAX_REGS];
	u16 num_regs;
};

struct og0ve1b_deckard_groups {
	int group_written;
	u64 pre_group_write_timestamp;
	u64 post_group_write_timestamp;
	u64 sof_ts;
	u64 sof_isr_ts;
	struct og0ve1b_deckard_register_set sets[OG0VE1B_NUM_GROUPS];
};

enum og0ve1b_deckard_timing_state {
	OG0VE1B_TIMING_NOT_IN_PROGRESS = 0,
	OG0VE1B_TIMING_READY_FOR_GROUP_RESTORE = 1,
	OG0VE1B_TIMING_REQUESTED = 4,
};

/* Layouts of the deckard driver on LP64 architectures */
static_assert(sizeof(struct og0ve1b_deckard_register) == 12);
static_assert(sizeof(struct og0ve1b_deckard_timing_adjustment) == 10);
static_assert(sizeof(struct og0ve1b_deckard_register_set) == 62);
static_assert(sizeof(struct og0ve1b_deckard_groups) == 232);
static_assert(offsetof(struct og0ve1b_deckard_groups, sets) == 40);
static_assert(OG0VE1B_METADATA_HISTORY_OFFSET +
	      OG0VE1B_HISTORY * sizeof(struct og0ve1b_deckard_groups) <=
	      OG0VE1B_METADATA_SIZE);

#define OG0VE1B_MAX_PENDING	256

struct og0ve1b_deckard {
	/* Serialises group data and register access */
	struct mutex lock;
	struct work_struct work;

	u32 queue_index;
	bool dual_i2c_main;
	bool dual_i2c_available;
	bool vc_interleaved;
	bool ctrl_setup;
	bool streaming;

	u32 group_mode;
	unsigned int last_group;
	struct og0ve1b_deckard_groups active;

	struct og0ve1b_deckard_timing_adjustment timing;
	enum og0ve1b_deckard_timing_state timing_state;

	u64 last_sof_ts;
	u64 last_sof_isr_ts;

	/* Protects metadata against the receiver driver callbacks */
	spinlock_t metadata_lock;
	struct og0ve1b_deckard_groups history[OG0VE1B_HISTORY];
	u8 metadata[OG0VE1B_METADATA_SIZE] __aligned(8);

	/* Single menu items of the callback and metadata size controls */
	s64 metadata_cb_address;
	s64 metadata_size;
	s64 sof_cb_address;

	/* Raw register writes done before stream start */
	struct cci_reg_sequence pending[OG0VE1B_MAX_PENDING];
	unsigned int num_pending;
};
#endif

struct og0ve1b {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xvclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[OG0VE1B_NUM_SUPPLIES];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl_handler ctrl_handler;

	/* Saved register value */
	u64 pre_isp;

#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
	struct og0ve1b_deckard v;
#endif
};

static const struct cci_reg_sequence og0ve1b_640x480_120fps_mode[] = {
	{ CCI_REG8(0x30a0), 0x02 },
	{ CCI_REG8(0x30a1), 0x00 },
	{ CCI_REG8(0x30a2), 0x48 },
	{ CCI_REG8(0x30a3), 0x34 },
	{ CCI_REG8(0x30a4), 0xf7 },
	{ CCI_REG8(0x30a5), 0x00 },
	{ CCI_REG8(0x3082), 0x32 },
	{ CCI_REG8(0x3083), 0x01 },
	{ CCI_REG8(0x301c), 0xf0 },
	{ CCI_REG8(0x301e), 0x0b },
	{ CCI_REG8(0x3106), 0x10 },
	{ CCI_REG8(0x3708), 0x77 },
	{ CCI_REG8(0x3709), 0xf8 },
	{ CCI_REG8(0x3717), 0x00 },
	{ CCI_REG8(0x3782), 0x00 },
	{ CCI_REG8(0x3783), 0x47 },
	{ CCI_REG8(0x37a2), 0x00 },
	{ CCI_REG8(0x3503), 0x07 },
	{ CCI_REG8(0x3509), 0x10 },
	{ CCI_REG8(0x3600), 0x83 },
	{ CCI_REG8(0x3601), 0x21 },
	{ CCI_REG8(0x3602), 0xf1 },
	{ CCI_REG8(0x360a), 0x18 },
	{ CCI_REG8(0x360e), 0xb3 },
	{ CCI_REG8(0x3613), 0x20 },
	{ CCI_REG8(0x366a), 0x78 },
	{ CCI_REG8(0x3706), 0x63 },
	{ CCI_REG8(0x3713), 0x00 },
	{ CCI_REG8(0x3716), 0xb0 },
	{ CCI_REG8(0x37a1), 0x38 },
	{ CCI_REG8(0x3800), 0x00 },
	{ CCI_REG8(0x3801), 0x04 },
	{ CCI_REG8(0x3802), 0x00 },
	{ CCI_REG8(0x3803), 0x04 },
	{ CCI_REG8(0x3804), 0x02 },
	{ CCI_REG8(0x3805), 0x8b },
	{ CCI_REG8(0x3806), 0x01 },
	{ CCI_REG8(0x3807), 0xeb },
	{ CCI_REG8(0x3808), 0x02 },	/* output width */
	{ CCI_REG8(0x3809), 0x80 },
	{ CCI_REG8(0x380a), 0x01 },	/* output height */
	{ CCI_REG8(0x380b), 0xe0 },
	{ CCI_REG8(0x380c), 0x03 },	/* horizontal timing size */
	{ CCI_REG8(0x380d), 0x18 },
	{ CCI_REG8(0x3811), 0x04 },
	{ CCI_REG8(0x3813), 0x04 },
	{ CCI_REG8(0x3814), 0x11 },
	{ CCI_REG8(0x3815), 0x11 },
	{ CCI_REG8(0x3820), 0x00 },
	{ CCI_REG8(0x3821), 0x00 },
	{ CCI_REG8(0x3823), 0x04 },
	{ CCI_REG8(0x382a), 0x00 },
	{ CCI_REG8(0x382b), 0x03 },
	{ CCI_REG8(0x3840), 0x00 },
	{ CCI_REG8(0x389e), 0x00 },
	{ CCI_REG8(0x3c05), 0x08 },
	{ CCI_REG8(0x3c26), 0x02 },
	{ CCI_REG8(0x3c27), 0xc0 },
	{ CCI_REG8(0x3c28), 0x00 },
	{ CCI_REG8(0x3c29), 0x40 },
	{ CCI_REG8(0x3c2c), 0x00 },
	{ CCI_REG8(0x3c2d), 0x50 },
	{ CCI_REG8(0x3c2e), 0x02 },
	{ CCI_REG8(0x3c2f), 0x66 },
	{ CCI_REG8(0x3c33), 0x08 },
	{ CCI_REG8(0x3c35), 0x00 },
	{ CCI_REG8(0x3c36), 0x00 },
	{ CCI_REG8(0x3c37), 0x00 },
	{ CCI_REG8(0x3f52), 0x9b },
	{ CCI_REG8(0x4001), 0x42 },
	{ CCI_REG8(0x4004), 0x08 },
	{ CCI_REG8(0x4005), 0x00 },
	{ CCI_REG8(0x4007), 0x28 },
	{ CCI_REG8(0x4009), 0x40 },
	{ CCI_REG8(0x4307), 0x30 },
	{ CCI_REG8(0x4500), 0x80 },
	{ CCI_REG8(0x4501), 0x02 },
	{ CCI_REG8(0x4502), 0x47 },
	{ CCI_REG8(0x4504), 0x7f },
	{ CCI_REG8(0x4601), 0x48 },
	{ CCI_REG8(0x4800), 0x64 },
	{ CCI_REG8(0x4801), 0x0f },
	{ CCI_REG8(0x4806), 0x2f },
	{ CCI_REG8(0x4819), 0xaa },
	{ CCI_REG8(0x4823), 0x3e },
	{ CCI_REG8(0x5000), 0x85 },
	{ CCI_REG8(0x5e00), 0x0c },
	{ CCI_REG8(0x3899), 0x09 },
	{ CCI_REG8(0x4f00), 0x64 },
	{ CCI_REG8(0x4f02), 0x0a },
	{ CCI_REG8(0x4f05), 0x0e },
	{ CCI_REG8(0x4f06), 0x11 },
	{ CCI_REG8(0x4f08), 0x0b },
	{ CCI_REG8(0x4f0a), 0xc4 },
	{ CCI_REG8(0x4f20), 0x1f },
	{ CCI_REG8(0x4f25), 0x10 },
	{ CCI_REG8(0x3016), 0x10 },
	{ CCI_REG8(0x3017), 0x00 },
	{ CCI_REG8(0x3018), 0x00 },
	{ CCI_REG8(0x3019), 0x00 },
	{ CCI_REG8(0x301a), 0x00 },
	{ CCI_REG8(0x301b), 0x00 },
	{ CCI_REG8(0x301c), 0x72 },
	{ CCI_REG8(0x3037), 0x40 },
	{ CCI_REG8(0x4f2c), 0x00 },
	{ CCI_REG8(0x4f21), 0x00 },
	{ CCI_REG8(0x4f23), 0x00 },
	{ CCI_REG8(0x4f2a), 0x00 },
	{ CCI_REG8(0x3665), 0xe7 },
	{ CCI_REG8(0x3668), 0x48 },
	{ CCI_REG8(0x3671), 0x3c },
	{ CCI_REG8(0x389a), 0x02 },
	{ CCI_REG8(0x389b), 0x00 },
	{ CCI_REG8(0x303c), 0xa0 },
	{ CCI_REG8(0x300f), 0xf0 },
	{ CCI_REG8(0x304b), 0x0f },
	{ CCI_REG8(0x3662), 0x24 },
	{ CCI_REG8(0x3006), 0x40 },
	{ CCI_REG8(0x4f26), 0x45 },
	{ CCI_REG8(0x3607), 0x34 },
	{ CCI_REG8(0x3608), 0x01 },
	{ CCI_REG8(0x360a), 0x0c },
	{ CCI_REG8(0x360b), 0x86 },
	{ CCI_REG8(0x360c), 0xcc },
	{ CCI_REG8(0x3013), 0x00 },
	{ CCI_REG8(0x3083), 0x02 },
	{ CCI_REG8(0x3084), 0x12 },
	{ CCI_REG8(0x4601), 0x38 },
	{ CCI_REG8(0x366f), 0x3a },
	{ CCI_REG8(0x3713), 0x19 },
	{ CCI_REG8(0x37a2), 0x00 },
	{ CCI_REG8(0x3f43), 0x27 },
	{ CCI_REG8(0x3f45), 0x27 },
	{ CCI_REG8(0x3f47), 0x32 },
	{ CCI_REG8(0x3f49), 0x3e },
	{ CCI_REG8(0x3f4b), 0x20 },
	{ CCI_REG8(0x3f4d), 0x30 },
	{ CCI_REG8(0x4300), 0x3f },
	{ CCI_REG8(0x4009), 0x10 },
	{ CCI_REG8(0x3f02), 0x68 },
	{ CCI_REG8(0x3700), 0x8c },
	{ CCI_REG8(0x370b), 0x7e },
	{ CCI_REG8(0x3f47), 0x35 },
};

static const struct og0ve1b_mode supported_modes[] = {
	{
		.width = 640,
		.height = 480,
		.hts = 792,
		.vts = 568,
		.bpp = 8,
		.reg_list = {
			.regs = og0ve1b_640x480_120fps_mode,
			.num_regs = ARRAY_SIZE(og0ve1b_640x480_120fps_mode),
		},
	},
};

static int og0ve1b_enable_test_pattern(struct og0ve1b *og0ve1b, u32 pattern)
{
	u64 val = og0ve1b->pre_isp;

	if (pattern)
		val |= OG0VE1B_TEST_PATTERN_ENABLE;
	else
		val &= ~OG0VE1B_TEST_PATTERN_ENABLE;

	return cci_write(og0ve1b->regmap, OG0VE1B_REG_PRE_ISP, val, NULL);
}

#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
static void og0ve1b_deckard_lock(struct og0ve1b *og0ve1b)
{
	mutex_lock(&og0ve1b->v.lock);
}

static void og0ve1b_deckard_unlock(struct og0ve1b *og0ve1b)
{
	mutex_unlock(&og0ve1b->v.lock);
}

static void og0ve1b_deckard_ctrl_setup(struct og0ve1b *og0ve1b, bool setup)
{
	og0ve1b->v.ctrl_setup = setup;
}

/* Protects the sensor count and the dual client */
static DEFINE_MUTEX(og0ve1b_deckard_mutex);
static unsigned int og0ve1b_deckard_num_sensors;
static struct i2c_client *og0ve1b_dual_client;
static struct regmap *og0ve1b_dual_regmap;

/* Sensors of virtual channels 0 and 1 in the interleaved mode */
static struct og0ve1b *og0ve1b_deckard_vc_sensor[2];

/* Shared by all sensors, as in the deckard driver */
static u64 og0ve1b_last_time_in_sync;
static u64 og0ve1b_last_dual_write_time;

static const struct regmap_config og0ve1b_dual_regmap_config = {
	.name = "og0ve1b-dual",
	.reg_bits = 16,
	.val_bits = 8,
};

static void og0ve1b_deckard_reset_groups(struct og0ve1b_deckard *v)
{
	unsigned long flags;
	unsigned int i;

	v->active.pre_group_write_timestamp = 0;
	v->active.post_group_write_timestamp = 0;
	memset(v->active.sets, 0, sizeof(v->active.sets));

	spin_lock_irqsave(&v->metadata_lock, flags);
	for (i = 0; i < OG0VE1B_HISTORY; i++)
		v->history[i] = v->active;
	spin_unlock_irqrestore(&v->metadata_lock, flags);
}

static int og0ve1b_deckard_set_group_reg(struct og0ve1b *og0ve1b, u32 arg)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;
	u16 addr = arg & 0xffff;
	u8 val = (arg >> 16) & 0xff;
	u8 group = (arg >> 24) & 0xf;
	struct og0ve1b_deckard_register_set *set;
	unsigned int i;

	if (group >= OG0VE1B_NUM_GROUPS)
		return -EINVAL;

	set = &v->active.sets[group];

	for (i = 0; i < set->num_regs; i++)
		if (set->addr[i] == addr)
			break;

	if (i == set->num_regs) {
		if (set->num_regs >= OG0VE1B_GROUP_MAX_REGS) {
			dev_err(og0ve1b->dev,
				"Group register map exceeded maximum size\n");
			return -ENOSPC;
		}

		set->addr[i] = addr;
		set->num_regs++;
	}

	set->val[i] = val;

	return 0;
}

/*
 * As the deckard driver, set the requested frame timing with the next write
 * of the group to adjust, and restore it with the following one.
 */
static int og0ve1b_deckard_adjust_timing(struct og0ve1b *og0ve1b,
					struct regmap *regmap)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;
	int ret = 0;

	switch (v->timing_state) {
	case OG0VE1B_TIMING_REQUESTED:
		cci_write(regmap, OG0VE1B_REG_VTS, v->timing.new_rows, &ret);
		cci_write(regmap, OG0VE1B_REG_GLOBAL_ROW,
			  v->timing.new_global_row, &ret);
		if (!ret)
			v->timing_state = OG0VE1B_TIMING_READY_FOR_GROUP_RESTORE;
		break;

	case OG0VE1B_TIMING_READY_FOR_GROUP_RESTORE:
		cci_write(regmap, OG0VE1B_REG_VTS, v->timing.rows_to_restore,
			  &ret);
		cci_write(regmap, OG0VE1B_REG_GLOBAL_ROW,
			  v->timing.global_row_to_restore, &ret);
		if (!ret)
			v->timing_state = OG0VE1B_TIMING_NOT_IN_PROGRESS;
		break;

	default:
		break;
	}

	return ret;
}

/*
 * Write a register group, as the deckard driver without the group hold
 * function, adjacent registers in one transfer.
 */
static int og0ve1b_deckard_write_group(struct og0ve1b *og0ve1b,
				      unsigned int group, struct regmap *regmap)
{
	const struct og0ve1b_deckard_register_set *set =
		&og0ve1b->v.active.sets[group];
	unsigned int i;
	int ret;

	for (i = 0; i < set->num_regs; i++) {
		if (i + 1 < set->num_regs &&
		    set->addr[i + 1] == set->addr[i] + 1) {
			ret = cci_write(regmap, CCI_REG16(set->addr[i]),
					set->val[i] << 8 | set->val[i + 1], NULL);
			i++;
		} else {
			ret = cci_write(regmap, CCI_REG8(set->addr[i]),
					set->val[i], NULL);
		}
		if (ret)
			return ret;
	}

	/*
	 * The frame timing is adjusted on the sensor itself, in the interleaved
	 * mode by the main sensor on all sensors
	 */
	if (group == og0ve1b->v.timing.group_to_adjust)
		return og0ve1b_deckard_adjust_timing(og0ve1b,
						    og0ve1b->v.vc_interleaved ?
						    regmap : og0ve1b->regmap);

	return 0;
}

static void og0ve1b_deckard_work(struct work_struct *work)
{
	struct og0ve1b_deckard *v = container_of(work, struct og0ve1b_deckard,
						work);
	struct og0ve1b *og0ve1b = container_of(v, struct og0ve1b, v);
	u64 pre_write_ts, post_write_ts, in_sync;
	unsigned long flags;
	bool vc_interleaved;
	int ret = 0;

	mutex_lock(&v->lock);

	pre_write_ts = ktime_get_raw_ns();

	if (READ_ONCE(v->group_mode) == OG0VE1B_GROUP_MODE_2_GROUPS)
		v->last_group = !v->last_group;
	else
		v->last_group = 0;

	/* The groups are written at every start of frame, changed or not */
	if (v->streaming) {
		in_sync = READ_ONCE(og0ve1b_last_time_in_sync);
		vc_interleaved = READ_ONCE(v->vc_interleaved);
		if (!in_sync && !vc_interleaved) {
			/* Not in sync, each sensor writes its own groups */
			ret = og0ve1b_deckard_write_group(og0ve1b, v->last_group,
							 og0ve1b->regmap);
		} else if (v->dual_i2c_main) {
			/* The main sensor writes its groups to all sensors */
			ret = og0ve1b_dual_regmap ?
			      og0ve1b_deckard_write_group(og0ve1b,
							 v->last_group,
							 og0ve1b_dual_regmap) :
			      -ENODEV;
			WRITE_ONCE(og0ve1b_last_dual_write_time,
				   ktime_get_raw_ns());
		} else if (!vc_interleaved &&
			   READ_ONCE(og0ve1b_last_dual_write_time) < in_sync) {
			/* No write by the main sensor since getting in sync */
			ret = og0ve1b_deckard_write_group(og0ve1b, v->last_group,
							 og0ve1b->regmap);
		}
		if (ret)
			dev_err_ratelimited(og0ve1b->dev,
					    "failed to write groups: %d\n", ret);
	}

	post_write_ts = ktime_get_raw_ns();

	/* As the deckard driver, record the metadata only while powered */
	if (v->streaming) {
		/* The group counter of the deckard driver is never updated */
		v->active.group_written = 0;
		v->active.pre_group_write_timestamp = pre_write_ts;
		v->active.sof_ts = v->last_sof_ts;
		v->active.sof_isr_ts = v->last_sof_isr_ts;
		v->active.post_group_write_timestamp = post_write_ts;

		spin_lock_irqsave(&v->metadata_lock, flags);
		memmove(&v->history[1], &v->history[0],
			(OG0VE1B_HISTORY - 1) *
			sizeof(v->history[0]));
		v->history[0] = v->active;
		spin_unlock_irqrestore(&v->metadata_lock, flags);
	}

	mutex_unlock(&v->lock);
}

/*
 * In the interleaved mode, the receiver driver callbacks of a sensor are for
 * the sensor of the virtual channel.
 */
static struct og0ve1b *og0ve1b_deckard_vc_target(struct v4l2_subdev *sd, u8 vc)
{
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);

	if (!READ_ONCE(og0ve1b->v.vc_interleaved))
		return og0ve1b;

	if (vc >= ARRAY_SIZE(og0ve1b_deckard_vc_sensor))
		return NULL;

	return READ_ONCE(og0ve1b_deckard_vc_sensor[vc]);
}

/* Called by the ISP for each frame */
static void *og0ve1b_deckard_get_metadata(struct v4l2_subdev *sd, u8 vc)
{
	struct og0ve1b *og0ve1b = og0ve1b_deckard_vc_target(sd, vc);
	struct og0ve1b_deckard *v;
	unsigned long flags;

	if (!og0ve1b)
		return NULL;

	v = &og0ve1b->v;

	spin_lock_irqsave(&v->metadata_lock, flags);
	memcpy(v->metadata + OG0VE1B_METADATA_HISTORY_OFFSET, v->history,
	       sizeof(v->history));
	spin_unlock_irqrestore(&v->metadata_lock, flags);

	return v->metadata;
}

/* Called by the ISP at each start of frame */
static void og0ve1b_deckard_sof(struct v4l2_subdev *sd, u64 sof_timestamp,
			       u64 sof_isr_timestamp, u8 vc)
{
	struct og0ve1b *og0ve1b = og0ve1b_deckard_vc_target(sd, vc);
	struct workqueue_struct *wq;
	struct og0ve1b_deckard *v;

	if (!og0ve1b)
		return;

	v = &og0ve1b->v;

	v->last_sof_ts = sof_timestamp;
	v->last_sof_isr_ts = sof_isr_timestamp;

	wq = READ_ONCE(sensor_workqueues[v->queue_index]);
	if (wq)
		queue_work(wq, &v->work);
}

static int og0ve1b_deckard_s_reg(struct og0ve1b *og0ve1b, u32 arg)
{
	return cci_write(og0ve1b->regmap, CCI_REG8(arg & 0xffff),
			 (arg >> 16) & 0xff, NULL);
}

/*
 * The deckard userspace configures the sensor with raw register writes before
 * stream start, the deckard driver keeps the sensor powered for them with
 * .s_power. Here the writes done before stream start are queued, and written
 * in order at stream start, after the register settings and the controls.
 * Called with the deckard lock held.
 */
static int og0ve1b_deckard_queue_reg(struct og0ve1b *og0ve1b, u32 arg)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;
	u32 reg = CCI_REG8(arg & 0xffff);

	/* Streaming can't be started or stopped before stream start */
	if (reg == OG0VE1B_REG_MODE_SELECT)
		return -EAGAIN;

	if (v->num_pending >= ARRAY_SIZE(v->pending))
		return -ENOSPC;

	v->pending[v->num_pending].reg = reg;
	v->pending[v->num_pending].val = (arg >> 16) & 0xff;
	v->num_pending++;

	return 0;
}

/*
 * Called with the deckard lock held. While streaming, the sensor is powered
 * and disabling the stream waits for the deckard lock.
 */
static int og0ve1b_deckard_write_reg(struct og0ve1b *og0ve1b, u32 arg)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;

	/* Keep the order of the queued writes */
	if (!v->streaming || v->num_pending)
		return og0ve1b_deckard_queue_reg(og0ve1b, arg);

	return og0ve1b_deckard_s_reg(og0ve1b, arg);
}

/* Called at stream start, while the sensor is powered */
static int og0ve1b_deckard_write_pending(struct og0ve1b *og0ve1b)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;
	int ret = 0;

	mutex_lock(&v->lock);
	cci_multi_reg_write(og0ve1b->regmap, v->pending, v->num_pending, &ret);
	v->num_pending = 0;
	mutex_unlock(&v->lock);

	if (ret)
		dev_err(og0ve1b->dev, "failed to write queued registers: %d\n",
			ret);

	return ret;
}

static int og0ve1b_deckard_g_reg(struct og0ve1b *og0ve1b,
				struct og0ve1b_deckard_register *reg)
{
	static const u32 width[] = {
		CCI_REG8(0), CCI_REG16(0), CCI_REG24(0), CCI_REG32(0),
	};
	u64 val = 0;
	int ret;

	if (!reg->len || reg->len > ARRAY_SIZE(width)) {
		ret = -EINVAL;
	} else {
		ret = cci_read(og0ve1b->regmap, width[reg->len - 1] |
			       reg->reg_addr, &val, NULL);
		if (ret)
			dev_err(og0ve1b->dev,
				"Failed to read register 0x%04x: %d\n",
				reg->reg_addr, ret);
	}

	reg->reg_value = val;

	return ret;
}

static void og0ve1b_deckard_set_in_sync(u32 in_sync)
{
	u64 now = ktime_get_raw_ns();
	u64 last = READ_ONCE(og0ve1b_last_time_in_sync);

	if (in_sync != 1)
		WRITE_ONCE(og0ve1b_last_time_in_sync, 0);
	else
		WRITE_ONCE(og0ve1b_last_time_in_sync,
			   last ? min(last, now) : now);
}

/* Restart streaming on all sensors at once through the broadcast address */
static int og0ve1b_deckard_synchronized_reset(struct og0ve1b *og0ve1b,
					     u32 reset)
{
	int ret = 0;

	if (reset != 1)
		return 0;

	if (!og0ve1b->v.dual_i2c_available)
		return -EINVAL;

	if (!og0ve1b_dual_regmap)
		return -ENODEV;

	cci_write(og0ve1b_dual_regmap, OG0VE1B_REG_MODE_SELECT,
		  OG0VE1B_MODE_STANDBY, &ret);
	/* Let streaming stop completely */
	fsleep(4000);
	cci_write(og0ve1b_dual_regmap, OG0VE1B_REG_MODE_SELECT,
		  OG0VE1B_MODE_STREAMING, &ret);

	return ret;
}

static long og0ve1b_deckard_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
				 void *arg)
{
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);
	struct og0ve1b_deckard *v = &og0ve1b->v;
	u32 val = (uintptr_t)arg;
	long ret = 0;

	switch (cmd) {
	case VIDIOC_OG0VE1B_PRV_G_METADATA_VERSION:
		*(u32 *)arg = OG0VE1B_METADATA_VERSION;
		break;

	case VIDIOC_OG0VE1B_PRV_G_DRIVER_VERSION:
		*(u32 *)arg = OG0VE1B_DRIVER_VERSION;
		break;

	case VIDIOC_OG0VE1B_PRV_S_REG:
		mutex_lock(&v->lock);
		ret = og0ve1b_deckard_write_reg(og0ve1b, val);
		mutex_unlock(&v->lock);
		break;

	case VIDIOC_OG0VE1B_PRV_S_GROUP_REG:
		mutex_lock(&v->lock);
		ret = og0ve1b_deckard_set_group_reg(og0ve1b, val);
		mutex_unlock(&v->lock);
		break;

	case VIDIOC_OG0VE1B_PRV_S_GROUP_MODE:
		WRITE_ONCE(v->group_mode, val);
		break;

	case VIDIOC_OG0VE1B_PRV_S_TIMING_ADJUSTMENT:
		mutex_lock(&v->lock);
		v->timing = *(struct og0ve1b_deckard_timing_adjustment *)arg;
		v->timing_state = OG0VE1B_TIMING_REQUESTED;
		mutex_unlock(&v->lock);
		break;

	case VIDIOC_OG0VE1B_PRV_G_REGISTER:
		/* Reads can't be deferred */
		if (pm_runtime_get_if_active(og0ve1b->dev) <= 0) {
			ret = -EAGAIN;
			break;
		}
		mutex_lock(&v->lock);
		ret = og0ve1b_deckard_g_reg(og0ve1b, arg);
		mutex_unlock(&v->lock);
		pm_runtime_put(og0ve1b->dev);
		break;

	case VIDIOC_OG0VE1B_PRV_S_IN_SYNC:
		og0ve1b_deckard_set_in_sync(val);
		break;

	case VIDIOC_OG0VE1B_PRV_S_VC_INT:
		WRITE_ONCE(v->vc_interleaved, val);
		break;

	case VIDIOC_OG0VE1B_PRV_S_SYNCHRONIZED_RESET:
		ret = og0ve1b_deckard_synchronized_reset(og0ve1b, val);
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

/* Called with the deckard lock held, while the sensor is powered */
static int og0ve1b_deckard_s_ctrl(struct og0ve1b *og0ve1b,
				 struct v4l2_ctrl *ctrl)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;

	switch (ctrl->id) {
	case V4L2_CID_METADATA_CB_ADDR:
	case V4L2_CID_SENSOR_SOF_CB_ADDR:
		return 0;

	/*
	 * Legacy controls for register access, they act on user writes only,
	 * not when the controls are applied at stream start.
	 */
	case V4L2_CID_BRIGHTNESS:
		if (v->ctrl_setup)
			return 0;
		return og0ve1b_deckard_write_reg(og0ve1b, ctrl->val);

	case V4L2_CID_HUE:
		if (v->ctrl_setup)
			return 0;
		return og0ve1b_deckard_set_group_reg(og0ve1b, ctrl->val);

	case V4L2_CID_SATURATION:
		if (!v->ctrl_setup)
			WRITE_ONCE(v->group_mode, ctrl->val);
		return 0;

	default:
		return -EINVAL;
	}
}

/* Legacy controls set by userspace while the sensor isn't powered */
static int og0ve1b_deckard_s_ctrl_unpowered(struct og0ve1b *og0ve1b,
					   struct v4l2_ctrl *ctrl)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;
	int ret = 0;

	mutex_lock(&v->lock);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = og0ve1b_deckard_queue_reg(og0ve1b, ctrl->val);
		break;

	/* The groups are written while streaming */
	case V4L2_CID_HUE:
		ret = og0ve1b_deckard_set_group_reg(og0ve1b, ctrl->val);
		break;

	case V4L2_CID_SATURATION:
		WRITE_ONCE(v->group_mode, ctrl->val);
		break;

	default:
		break;
	}

	mutex_unlock(&v->lock);

	return ret;
}

static void og0ve1b_deckard_init_controls(struct og0ve1b *og0ve1b,
					 struct v4l2_ctrl_handler *hdl,
					 const struct v4l2_ctrl_ops *ops)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;
	struct v4l2_ctrl_config cfg = {
		.ops = ops,
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	};

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BRIGHTNESS, 0, 0xffffff, 1, 0);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HUE, S32_MIN, S32_MAX, 1, 0);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION, 0, 2, 1, 0);

	v->metadata_cb_address = (s64)(uintptr_t)og0ve1b_deckard_get_metadata;
	v->metadata_size = OG0VE1B_METADATA_SIZE;
	v->sof_cb_address = (s64)(uintptr_t)og0ve1b_deckard_sof;

	cfg.id = V4L2_CID_METADATA_CB_ADDR;
	cfg.name = "Metadata Callback";
	cfg.qmenu_int = &v->metadata_cb_address;
	v4l2_ctrl_new_custom(hdl, &cfg, NULL);

	cfg.id = V4L2_CID_METADATA_SIZE;
	cfg.name = "Metadata Size";
	cfg.qmenu_int = &v->metadata_size;
	cfg.flags = V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_custom(hdl, &cfg, NULL);

	cfg.id = V4L2_CID_SENSOR_SOF_CB_ADDR;
	cfg.name = "Start of Frame Callback";
	cfg.qmenu_int = &v->sof_cb_address;
	cfg.flags = 0;
	v4l2_ctrl_new_custom(hdl, &cfg, NULL);
}

/* In the interleaved mode, streaming is started and stopped on all sensors */
static struct regmap *og0ve1b_deckard_stream_regmap(struct og0ve1b *og0ve1b)
{
	if (READ_ONCE(og0ve1b->v.vc_interleaved) && og0ve1b_dual_regmap)
		return og0ve1b_dual_regmap;

	return og0ve1b->regmap;
}

/* Called with the state lock held */
static void og0ve1b_deckard_s_stream(struct og0ve1b *og0ve1b, bool enable)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;

	WRITE_ONCE(og0ve1b_last_time_in_sync, 0);

	mutex_lock(&v->lock);
	v->streaming = enable;
	/* The groups set by userspace for the next stream start are kept */
	if (!enable)
		og0ve1b_deckard_reset_groups(v);
	mutex_unlock(&v->lock);
}

static void og0ve1b_deckard_create_dual_client(struct og0ve1b *og0ve1b,
					      struct i2c_client *client)
{
	struct i2c_board_info info = {
		I2C_BOARD_INFO("og0ve1b-dual", OG0VE1B_DUAL_I2C_ADDR),
	};
	struct i2c_client *dual;
	struct regmap *regmap;

	dual = i2c_new_client_device(client->adapter, &info);
	if (IS_ERR(dual)) {
		dev_err(og0ve1b->dev, "failed to create dual i2c client: %pe\n",
			dual);
		return;
	}

	regmap = regmap_init_i2c(dual, &og0ve1b_dual_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(og0ve1b->dev, "failed to init dual i2c regmap: %pe\n",
			regmap);
		i2c_unregister_device(dual);
		return;
	}

	og0ve1b_dual_client = dual;
	og0ve1b_dual_regmap = regmap;
}

static int og0ve1b_deckard_probe(struct og0ve1b *og0ve1b,
				struct i2c_client *client)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;
	const char *id;
	u32 val;

	mutex_init(&v->lock);
	spin_lock_init(&v->metadata_lock);
	INIT_WORK(&v->work, og0ve1b_deckard_work);

	if (device_property_read_u32(og0ve1b->dev, "sensor-queue-index",
				     &val) || val >= ARRAY_SIZE(sensor_workqueues))
		val = 0;
	v->queue_index = val;

	v->dual_i2c_main =
		!device_property_read_u32(og0ve1b->dev, "dual-i2c-main", &val) &&
		val;
	v->dual_i2c_available =
		!device_property_read_u32(og0ve1b->dev, "dual-i2c-available",
					  &val) && val;

	mutex_lock(&og0ve1b_deckard_mutex);

	/* The dual client is not needed for the sensor to work */
	if (!og0ve1b_dual_client)
		og0ve1b_deckard_create_dual_client(og0ve1b, client);

	og0ve1b_deckard_num_sensors++;

	/* The left sensor has virtual channel 0, the other one 1 */
	if (!device_property_read_string(og0ve1b->dev, "id", &id) &&
	    !strcmp(id, OG0VE1B_LEFT_SENSOR_ID))
		WRITE_ONCE(og0ve1b_deckard_vc_sensor[0], og0ve1b);
	else
		WRITE_ONCE(og0ve1b_deckard_vc_sensor[1], og0ve1b);

	mutex_unlock(&og0ve1b_deckard_mutex);

	return 0;
}

static void og0ve1b_deckard_remove(struct og0ve1b *og0ve1b)
{
	struct og0ve1b_deckard *v = &og0ve1b->v;
	unsigned int i;

	mutex_lock(&og0ve1b_deckard_mutex);

	for (i = 0; i < ARRAY_SIZE(og0ve1b_deckard_vc_sensor); i++)
		if (og0ve1b_deckard_vc_sensor[i] == og0ve1b)
			WRITE_ONCE(og0ve1b_deckard_vc_sensor[i], NULL);

	mutex_unlock(&og0ve1b_deckard_mutex);

	/* Callbacks of the other sensor may still queue the work */
	cancel_work_sync(&v->work);

	mutex_lock(&og0ve1b_deckard_mutex);

	if (!--og0ve1b_deckard_num_sensors && og0ve1b_dual_client) {
		regmap_exit(og0ve1b_dual_regmap);
		i2c_unregister_device(og0ve1b_dual_client);
		og0ve1b_dual_regmap = NULL;
		og0ve1b_dual_client = NULL;
	}

	mutex_unlock(&og0ve1b_deckard_mutex);

	mutex_destroy(&v->lock);
}
#else
static void og0ve1b_deckard_lock(struct og0ve1b *og0ve1b)
{
}

static void og0ve1b_deckard_unlock(struct og0ve1b *og0ve1b)
{
}

static void og0ve1b_deckard_ctrl_setup(struct og0ve1b *og0ve1b, bool setup)
{
}

static int og0ve1b_deckard_write_pending(struct og0ve1b *og0ve1b)
{
	return 0;
}

static int og0ve1b_deckard_s_ctrl(struct og0ve1b *og0ve1b,
				 struct v4l2_ctrl *ctrl)
{
	return -EINVAL;
}

static int og0ve1b_deckard_s_ctrl_unpowered(struct og0ve1b *og0ve1b,
					   struct v4l2_ctrl *ctrl)
{
	return 0;
}

static void og0ve1b_deckard_init_controls(struct og0ve1b *og0ve1b,
					 struct v4l2_ctrl_handler *hdl,
					 const struct v4l2_ctrl_ops *ops)
{
}

static void og0ve1b_deckard_s_stream(struct og0ve1b *og0ve1b, bool enable)
{
}

static struct regmap *og0ve1b_deckard_stream_regmap(struct og0ve1b *og0ve1b)
{
	return og0ve1b->regmap;
}

static int og0ve1b_deckard_probe(struct og0ve1b *og0ve1b,
				struct i2c_client *client)
{
	return 0;
}

static void og0ve1b_deckard_remove(struct og0ve1b *og0ve1b)
{
}
#endif

static int og0ve1b_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct og0ve1b *og0ve1b = container_of(ctrl->handler, struct og0ve1b,
					       ctrl_handler);
	const struct og0ve1b_mode *mode = &supported_modes[0];
	s64 exposure_max;
	int ret;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		exposure_max = ctrl->val + mode->height -
			OG0VE1B_EXPOSURE_MAX_MARGIN;
		ret = __v4l2_ctrl_modify_range(og0ve1b->exposure,
					og0ve1b->exposure->minimum,
					exposure_max,
					og0ve1b->exposure->step,
					og0ve1b->exposure->default_value);
		if (ret)
			return ret;
	}

	/* V4L2 controls are applied, when sensor is powered up for streaming */
	if (!pm_runtime_get_if_active(og0ve1b->dev))
		return og0ve1b_deckard_s_ctrl_unpowered(og0ve1b, ctrl);

	/* Serialise with the register access of the deckard interface */
	og0ve1b_deckard_lock(og0ve1b);

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(og0ve1b->regmap, OG0VE1B_REG_ANALOGUE_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(og0ve1b->regmap, OG0VE1B_REG_EXPOSURE,
				ctrl->val << 4, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(og0ve1b->regmap, OG0VE1B_REG_VTS,
				ctrl->val + mode->height, NULL);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = og0ve1b_enable_test_pattern(og0ve1b, ctrl->val);
		break;
	default:
		ret = og0ve1b_deckard_s_ctrl(og0ve1b, ctrl);
		break;
	}

	og0ve1b_deckard_unlock(og0ve1b);

	pm_runtime_put(og0ve1b->dev);

	return ret;
}

static const struct v4l2_ctrl_ops og0ve1b_ctrl_ops = {
	.s_ctrl = og0ve1b_set_ctrl,
};

static int og0ve1b_init_controls(struct og0ve1b *og0ve1b)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &og0ve1b->ctrl_handler;
	const struct og0ve1b_mode *mode = &supported_modes[0];
	s64 exposure_max, pixel_rate, h_blank, v_blank;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(ctrl_hdlr, 14);

	ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr, &og0ve1b_ctrl_ops,
				      V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(og0ve1b_link_freq_menu) - 1,
				      0, og0ve1b_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	pixel_rate = og0ve1b_link_freq_menu[0] / mode->bpp;
	v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  0, pixel_rate, 1, pixel_rate);

	h_blank = mode->hts - mode->width;
	ctrl = v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops, V4L2_CID_HBLANK,
				 h_blank, h_blank, 1, h_blank);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v_blank = mode->vts - mode->height;
	og0ve1b->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops,
					    V4L2_CID_VBLANK, v_blank,
					    OG0VE1B_VTS_MAX - mode->height, 1,
					    v_blank);

	v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OG0VE1B_ANALOGUE_GAIN_MIN, OG0VE1B_ANALOGUE_GAIN_MAX,
			  OG0VE1B_ANALOGUE_GAIN_STEP,
			  OG0VE1B_ANALOGUE_GAIN_DEFAULT);

	exposure_max = mode->vts - OG0VE1B_EXPOSURE_MAX_MARGIN;
	og0ve1b->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &og0ve1b_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      OG0VE1B_EXPOSURE_MIN,
					      exposure_max,
					      OG0VE1B_EXPOSURE_STEP,
					      OG0VE1B_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &og0ve1b_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(og0ve1b_test_pattern_menu) - 1,
				     0, 0, og0ve1b_test_pattern_menu);

	og0ve1b_deckard_init_controls(og0ve1b, ctrl_hdlr, &og0ve1b_ctrl_ops);

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	ret = v4l2_fwnode_device_parse(og0ve1b->dev, &props);
	if (ret)
		goto error_free_hdlr;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &og0ve1b_ctrl_ops,
					      &props);
	if (ret)
		goto error_free_hdlr;

	og0ve1b->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error_free_hdlr:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static void og0ve1b_update_pad_format(const struct og0ve1b_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = MEDIA_BUS_FMT_Y8_1X8;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int og0ve1b_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	const struct og0ve1b_reg_list *reg_list = &supported_modes[0].reg_list;
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);
	int ret;

	ret = pm_runtime_resume_and_get(og0ve1b->dev);
	if (ret)
		return ret;

	/* Skip a step of explicit entering into the standby mode */
	ret = cci_write(og0ve1b->regmap, OG0VE1B_REG_SOFTWARE_RST,
			OG0VE1B_SOFTWARE_RST, NULL);
	if (ret) {
		dev_err(og0ve1b->dev, "failed to software reset: %d\n", ret);
		goto error;
	}

	ret = cci_multi_reg_write(og0ve1b->regmap, reg_list->regs,
				  reg_list->num_regs, NULL);
	if (ret) {
		dev_err(og0ve1b->dev, "failed to set mode: %d\n", ret);
		goto error;
	}

	og0ve1b_deckard_ctrl_setup(og0ve1b, true);
	ret = __v4l2_ctrl_handler_setup(og0ve1b->sd.ctrl_handler);
	og0ve1b_deckard_ctrl_setup(og0ve1b, false);
	if (ret)
		goto error;

	ret = og0ve1b_deckard_write_pending(og0ve1b);
	if (ret)
		goto error;

	og0ve1b_deckard_s_stream(og0ve1b, true);

	ret = cci_write(og0ve1b_deckard_stream_regmap(og0ve1b),
			OG0VE1B_REG_MODE_SELECT, OG0VE1B_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(og0ve1b->dev, "failed to start streaming: %d\n", ret);
		goto error;
	}

	return 0;

error:
	og0ve1b_deckard_s_stream(og0ve1b, false);
	pm_runtime_put_autosuspend(og0ve1b->dev);

	return ret;
}

static int og0ve1b_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);
	int ret;

	og0ve1b_deckard_s_stream(og0ve1b, false);

	ret = cci_write(og0ve1b_deckard_stream_regmap(og0ve1b),
			OG0VE1B_REG_MODE_SELECT, OG0VE1B_MODE_STANDBY, NULL);
	if (ret)
		dev_err(og0ve1b->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(og0ve1b->dev);

	return ret;
}

static int og0ve1b_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *format;
	const struct og0ve1b_mode *mode;

	format = v4l2_subdev_state_get_format(state, 0);

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width,
				      fmt->format.height);

	og0ve1b_update_pad_format(mode, &fmt->format);
	*format = fmt->format;

	return 0;
}

static int og0ve1b_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_Y8_1X8;

	return 0;
}

static int og0ve1b_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_Y8_1X8)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int og0ve1b_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = MEDIA_BUS_FMT_Y8_1X8,
			.width = supported_modes[0].width,
			.height = supported_modes[0].height,
		},
	};

	og0ve1b_set_pad_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops og0ve1b_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops og0ve1b_pad_ops = {
	.set_fmt = og0ve1b_set_pad_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.enum_mbus_code = og0ve1b_enum_mbus_code,
	.enum_frame_size = og0ve1b_enum_frame_size,
	.enable_streams = og0ve1b_enable_streams,
	.disable_streams = og0ve1b_disable_streams,
};

#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
static const struct v4l2_subdev_core_ops og0ve1b_core_ops = {
	.ioctl = og0ve1b_deckard_ioctl,
};
#endif

static const struct v4l2_subdev_ops og0ve1b_subdev_ops = {
#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
	.core = &og0ve1b_core_ops,
#endif
	.video = &og0ve1b_video_ops,
	.pad = &og0ve1b_pad_ops,
};

static const struct v4l2_subdev_internal_ops og0ve1b_internal_ops = {
	.init_state = og0ve1b_init_state,
};

static const struct media_entity_operations og0ve1b_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int og0ve1b_identify_sensor(struct og0ve1b *og0ve1b)
{
	u64 val;
	int ret;

	ret = cci_read(og0ve1b->regmap, OG0VE1B_REG_CHIP_ID, &val, NULL);
	if (ret) {
		dev_err(og0ve1b->dev, "failed to read chip id: %d\n", ret);
		return ret;
	}

	if (val != OG0VE1B_CHIP_ID) {
		dev_err(og0ve1b->dev, "chip id mismatch: %x!=%llx\n",
			OG0VE1B_CHIP_ID, val);
		return -ENODEV;
	}

	ret = cci_read(og0ve1b->regmap, OG0VE1B_REG_PRE_ISP,
		       &og0ve1b->pre_isp, NULL);
	if (ret)
		dev_err(og0ve1b->dev, "failed to read pre_isp: %d\n", ret);

	return ret;
}

static int og0ve1b_check_hwcfg(struct og0ve1b *og0ve1b)
{
	struct fwnode_handle *fwnode = dev_fwnode(og0ve1b->dev), *ep;
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	unsigned long freq_bitmap;
	int ret;

	if (!fwnode)
		return -ENODEV;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -EINVAL;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	ret = v4l2_link_freq_to_bitmap(og0ve1b->dev,
				       bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       og0ve1b_link_freq_menu,
				       ARRAY_SIZE(og0ve1b_link_freq_menu),
				       &freq_bitmap);

	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int og0ve1b_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);
	int ret;

	ret = regulator_bulk_enable(OG0VE1B_NUM_SUPPLIES, og0ve1b->supplies);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(og0ve1b->reset_gpio, 0);
	usleep_range(10 * USEC_PER_MSEC, 15 * USEC_PER_MSEC);

	ret = clk_prepare_enable(og0ve1b->xvclk);
	if (ret)
		goto reset_gpio;

	return 0;

reset_gpio:
	gpiod_set_value_cansleep(og0ve1b->reset_gpio, 1);

	regulator_bulk_disable(OG0VE1B_NUM_SUPPLIES, og0ve1b->supplies);

	return ret;
}

static int og0ve1b_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);

	clk_disable_unprepare(og0ve1b->xvclk);

	gpiod_set_value_cansleep(og0ve1b->reset_gpio, 1);

	regulator_bulk_disable(OG0VE1B_NUM_SUPPLIES, og0ve1b->supplies);

	return 0;
}

static int og0ve1b_probe(struct i2c_client *client)
{
	struct og0ve1b *og0ve1b;
	unsigned long freq;
	unsigned int i;
	int ret;

	og0ve1b = devm_kzalloc(&client->dev, sizeof(*og0ve1b), GFP_KERNEL);
	if (!og0ve1b)
		return -ENOMEM;

	og0ve1b->dev = &client->dev;

	v4l2_i2c_subdev_init(&og0ve1b->sd, client, &og0ve1b_subdev_ops);

	og0ve1b->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(og0ve1b->regmap))
		return dev_err_probe(og0ve1b->dev, PTR_ERR(og0ve1b->regmap),
				     "failed to init CCI\n");

	og0ve1b->xvclk = devm_v4l2_sensor_clk_get(og0ve1b->dev, NULL);
	if (IS_ERR(og0ve1b->xvclk))
		return dev_err_probe(og0ve1b->dev, PTR_ERR(og0ve1b->xvclk),
				     "failed to get XVCLK clock\n");

	freq = clk_get_rate(og0ve1b->xvclk);
	if (freq && freq != OG0VE1B_MCLK_FREQ_24MHZ)
		return dev_err_probe(og0ve1b->dev, -EINVAL,
				     "XVCLK clock frequency %lu is not supported\n",
				     freq);

	ret = og0ve1b_check_hwcfg(og0ve1b);
	if (ret)
		return dev_err_probe(og0ve1b->dev, ret,
				     "failed to check HW configuration\n");

	og0ve1b->reset_gpio = devm_gpiod_get_optional(og0ve1b->dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(og0ve1b->reset_gpio))
		return dev_err_probe(og0ve1b->dev, PTR_ERR(og0ve1b->reset_gpio),
				     "cannot get reset GPIO\n");

	for (i = 0; i < OG0VE1B_NUM_SUPPLIES; i++)
		og0ve1b->supplies[i].supply = og0ve1b_supply_names[i];

	ret = devm_regulator_bulk_get(og0ve1b->dev, OG0VE1B_NUM_SUPPLIES,
				      og0ve1b->supplies);
	if (ret)
		return dev_err_probe(og0ve1b->dev, ret,
				     "failed to get supply regulators\n");

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = og0ve1b_power_on(og0ve1b->dev);
	if (ret)
		return ret;

	ret = og0ve1b_identify_sensor(og0ve1b);
	if (ret) {
		dev_err_probe(og0ve1b->dev, ret, "failed to find sensor\n");
		goto power_off;
	}

	ret = og0ve1b_deckard_probe(og0ve1b, client);
	if (ret)
		goto power_off;

	ret = og0ve1b_init_controls(og0ve1b);
	if (ret) {
		dev_err_probe(og0ve1b->dev, ret, "failed to init controls\n");
		goto vendor_remove;
	}

	og0ve1b->sd.state_lock = og0ve1b->ctrl_handler.lock;
	og0ve1b->sd.internal_ops = &og0ve1b_internal_ops;
	og0ve1b->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	og0ve1b->sd.entity.ops = &og0ve1b_subdev_entity_ops;
	og0ve1b->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	og0ve1b->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&og0ve1b->sd.entity, 1, &og0ve1b->pad);
	if (ret) {
		dev_err_probe(og0ve1b->dev, ret,
			      "failed to init media entity pads\n");
		goto v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&og0ve1b->sd);
	if (ret < 0) {
		dev_err_probe(og0ve1b->dev, ret,
			      "failed to init media entity pads\n");
		goto media_entity_cleanup;
	}

	pm_runtime_set_active(og0ve1b->dev);
	pm_runtime_enable(og0ve1b->dev);

	ret = v4l2_async_register_subdev_sensor(&og0ve1b->sd);
	if (ret < 0) {
		dev_err_probe(og0ve1b->dev, ret,
			      "failed to register V4L2 subdev\n");
		goto subdev_cleanup;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_idle(og0ve1b->dev);
	pm_runtime_set_autosuspend_delay(og0ve1b->dev, 1000);
	pm_runtime_use_autosuspend(og0ve1b->dev);

	return 0;

subdev_cleanup:
	v4l2_subdev_cleanup(&og0ve1b->sd);
	pm_runtime_disable(og0ve1b->dev);
	pm_runtime_set_suspended(og0ve1b->dev);

media_entity_cleanup:
	media_entity_cleanup(&og0ve1b->sd.entity);

v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(og0ve1b->sd.ctrl_handler);

vendor_remove:
	og0ve1b_deckard_remove(og0ve1b);

power_off:
	og0ve1b_power_off(og0ve1b->dev);

	return ret;
}

static void og0ve1b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og0ve1b *og0ve1b = to_og0ve1b(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	og0ve1b_deckard_remove(og0ve1b);
	pm_runtime_disable(og0ve1b->dev);

	if (!pm_runtime_status_suspended(og0ve1b->dev)) {
		og0ve1b_power_off(og0ve1b->dev);
		pm_runtime_set_suspended(og0ve1b->dev);
	}
}

static const struct dev_pm_ops og0ve1b_pm_ops = {
	SET_RUNTIME_PM_OPS(og0ve1b_power_off, og0ve1b_power_on, NULL)
};

static const struct of_device_id og0ve1b_of_match[] = {
	{ .compatible = "ovti,og0ve1b" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, og0ve1b_of_match);

static struct i2c_driver og0ve1b_i2c_driver = {
	.driver = {
		.name = "og0ve1b",
		.pm = &og0ve1b_pm_ops,
		.of_match_table = og0ve1b_of_match,
	},
	.probe = og0ve1b_probe,
	.remove = og0ve1b_remove,
};

module_i2c_driver(og0ve1b_i2c_driver);

MODULE_AUTHOR("Vladimir Zapolskiy <vladimir.zapolskiy@linaro.org>");
MODULE_DESCRIPTION("OmniVision OG0VE1B sensor driver");
MODULE_LICENSE("GPL");
