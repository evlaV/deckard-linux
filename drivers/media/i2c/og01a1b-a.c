// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 *
 * Based on OV9282 driver and some register settings from intel OG01A1B driver. Adapted by arcturus.industries
 */

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
#include <linux/gpio.h>
#include <linux/platform_device.h>

#define V4L2_CID_TEMPERATURE	(V4L2_CID_USER_OG01A1B_BASE + 0)

#define OG01A1B_REG_CHIP_ID		0x300a

#define OG01A1B_LINK_FREQUENCY 500000000

#define OG01A1B_REG_TPM_TEMPERATURE		0x4d17

#define OG01A1B_REG_TIMING_DVPHO		0x3808
#define OG01A1B_REG_TIMING_DVPVO		0x380a
#define OG01A1B_REG_TIMING_HTS		0x380c
#define OG01A1B_REG_TIMING_VTS		0x380e
#define NUM_REGISTER_GROUPS			2
#define GROUP_REGMAP_MAX_SIZE		10
#define LATE_GROUP_WRITE_FLAG 		0x1

DEFINE_SPINLOCK(group_spinlock);

DEFINE_SPINLOCK(metadata_spinlock);
DEFINE_SPINLOCK(ir_spinlock);

#define VIDIOC_OG01A1B_PRV_G_METADATA_VERSION _IOR('V', BASE_VIDIOC_PRIVATE + 1, u32)

#define VIDIOC_OG01A1B_PRV_S_IR_EMITTER_MODE _IOW('V', BASE_VIDIOC_PRIVATE + 2, u32)

// 32-bit argument: sensor register address in bits [16, 0], value in bits [24, 16]
#define VIDIOC_OG01A1B_PRV_S_REG        _IO('V', BASE_VIDIOC_PRIVATE + 3)
// 32-bit argument: sensor register address in bits [16, 0], value in bits [24, 16], group ID in bits [31, 24]
#define VIDIOC_OG01A1B_PRV_S_GROUP_REG  _IO('V', BASE_VIDIOC_PRIVATE + 4) // Argument is sensor register to set, address in bits [16, 0] value in bits [24, 16]
 // Integer argument: ID of the group mode
#define VIDIOC_OG01A1B_PRV_S_GROUP_MODE _IO('V', BASE_VIDIOC_PRIVATE + 5)

// Version 1.2 support histogram streaming.

#define METADATA_MAJOR_VERSION 1
#define METADATA_MINOR_VERSION 2


/* regulator supplies */
static const char * const og01a1b_supply_name[] = {
	"DOVDD", /* Digital I/O (1.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
	"DVDD",  /* Digital Core (1.5V) supply */
};

#define og01a1b_NUM_SUPPLIES ARRAY_SIZE(og01a1b_supply_name)

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

struct og01a1b_mode_info {
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
	const struct reg_value *reg_data;
	u32 reg_data_size;
};

struct register_set {
	u16 group_reg_addresses[GROUP_REGMAP_MAX_SIZE];
	u8 group_reg_values[GROUP_REGMAP_MAX_SIZE];
	u16 num_group_reg;
};

struct group_and_registers {
	int group_written;
	u64 pre_group_write_timestamp;
	u64 post_group_write_timestamp;
	u64 sof_ts;
	u64 sof_isr_ts;
	struct register_set group_register_sets[NUM_REGISTER_GROUPS];
};

struct SensorMetaData {
	u64 lastSOE;
	u64 lastEOE;
	u64 currentSOE;
	struct group_and_registers group_and_reg[5];
	int flags; // for outputing status flags
	u64 lastGroupChangePostWriteTimestamp;
	u8 padding[8];
};


enum og01a1b_group_mode {
	MODE_NOGROUPS = 0,
	MODE_1GROUPs = 1,
	MODE_2GROUPS =  2
};

enum og01a1b_ir_emitter_mode {
	IR_EMITTER_MODE_OFF = 0,
	IR_EMITTER_MODE_GROUP_1_ONLY = 1,
	IR_EMITTER_MODE_GROUP_0_ONLY =  2,
	IR_EMITTER_MODE_ALL_FRAMES
};

struct og01a1b_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to og01a1b */
	struct regulator_bulk_data supplies[og01a1b_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *strobe_gpio;
	struct gpio_desc *ir_gpio[2];

	/* lock to protect all members below */
	struct mutex lock;

	int power_count;
	u8 last_set_led_state;

	struct v4l2_ctrl_handler ctrl_handler;
	struct mutex mutex;

	struct v4l2_mbus_framefmt fmt;
	struct work_struct work;
	bool is_sync_sensor;
	struct og01a1b_dev *other_sensor;

	//sensorMetaData is modified by the group update
	//outputSensorMetaData is a copy for output
	struct SensorMetaData sensorMetaData;
	struct SensorMetaData outputSensorMetaData;
	s64 metadata_cb_address[1];
	s64 metadata_size[1];
	s64 sof_cb_address[1];
	u64 last_sof_ts;
	u64 last_sof_isr_ts;
	u64 last_requested_groupwrite_isp_ts;
	u64 last_emitter_on_ts;

	struct group_and_registers active_group_registers;
	bool updateGroupRegisters;
	enum og01a1b_ir_emitter_mode ir_emitter_mode;
	bool emittersPresent;
	enum og01a1b_group_mode group_mode;
	int sensor_index;

	const struct og01a1b_mode_info* current_mode;
	bool is_streaming;
	bool is_og0VE10;
};

struct og01a1b_dev* sensors[4];
int numSensors = 0;
int sync_sensor_index = -1;

static inline struct og01a1b_dev *to_og01a1b_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct og01a1b_dev, sd);
}

static const struct reg_value og0VE10_setting[] = {
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
{0x0100, 0x01}
};

static const struct reg_value og01a1b_setting[] = {

{0x0103, 0x01, 0, 0},
{0x0100, 0x00, 0, 0},
{0x0300, 0x0a, 0, 0},
{0x0301, 0x29, 0, 0},
{0x0302, 0x31, 0, 0},
{0x0303, 0x04, 0, 0},
{0x0304, 0x00, 0, 0},
{0x0305, 0xfa, 0, 0},
{0x0306, 0x00, 0, 0},
{0x0307, 0x01, 0, 0},
{0x0308, 0x02, 0, 0},
{0x0309, 0x00, 0, 0},
{0x0310, 0x00, 0, 0},
{0x0311, 0x00, 0, 0},
{0x0312, 0x07, 0, 0},
{0x0313, 0x00, 0, 0},
{0x0314, 0x00, 0, 0},
{0x0315, 0x00, 0, 0},
{0x0320, 0x02, 0, 0},
{0x0321, 0x01, 0, 0},
{0x0322, 0x01, 0, 0},
{0x0323, 0x04, 0, 0},
{0x0324, 0x01, 0, 0},
{0x0325, 0xc2, 0, 0},
{0x0326, 0xce, 0, 0},
{0x0327, 0x04, 0, 0},
{0x0329, 0x02, 0, 0},
{0x032a, 0x04, 0, 0},
{0x032b, 0x04, 0, 0},
{0x032c, 0x02, 0, 0},
{0x032d, 0x01, 0, 0},
{0x032e, 0x00, 0, 0},
{0x300d, 0x02, 0, 0},
{0x300e, 0x04, 0, 0},
{0x3021, 0x08, 0, 0},
{0x301e, 0x03, 0, 0},
{0x3103, 0x00, 0, 0},
{0x3106, 0x08, 0, 0},
{0x3107, 0x40, 0, 0},
{0x3216, 0x01, 0, 0},
{0x3217, 0x00, 0, 0},
{0x3218, 0xc0, 0, 0},
{0x3219, 0x55, 0, 0},
{0x3500, 0x00, 0, 0},
{0x3501, 0x04, 0, 0},
{0x3502, 0x8a, 0, 0},
{0x3506, 0x01, 0, 0},
{0x3507, 0x72, 0, 0},
{0x3508, 0x01, 0, 0},
{0x3509, 0x00, 0, 0},
{0x350a, 0x01, 0, 0},
{0x350b, 0x00, 0, 0},
{0x350c, 0x00, 0, 0},
{0x3541, 0x00, 0, 0},
{0x3542, 0x40, 0, 0},
{0x3605, 0xe0, 0, 0},
{0x3606, 0x41, 0, 0},
{0x3614, 0x20, 0, 0},
{0x3620, 0x0b, 0, 0},
{0x3630, 0x07, 0, 0},
{0x3636, 0xa0, 0, 0},
{0x3637, 0xf9, 0, 0},
{0x3638, 0x09, 0, 0},
{0x3639, 0x38, 0, 0},
{0x363f, 0x09, 0, 0},
{0x3640, 0x17, 0, 0},
{0x3662, 0x06, 0, 0}, // enables 8 bit mode
{0x3665, 0x80, 0, 0},
{0x3670, 0x68, 0, 0},
{0x3674, 0x00, 0, 0},
{0x3677, 0x3f, 0, 0},
{0x3679, 0x00, 0, 0},
{0x369f, 0x19, 0, 0},
{0x36a0, 0x03, 0, 0},
{0x36a2, 0x19, 0, 0},
{0x36a3, 0x03, 0, 0},
{0x370d, 0x66, 0, 0},
{0x370f, 0x00, 0, 0},
{0x3710, 0x03, 0, 0},
{0x3715, 0x03, 0, 0},
{0x3716, 0x03, 0, 0},
{0x3717, 0x06, 0, 0},
{0x3733, 0x00, 0, 0},
{0x3778, 0x00, 0, 0},
{0x37a8, 0x0f, 0, 0},
{0x37a9, 0x01, 0, 0},
{0x37aa, 0x07, 0, 0},
{0x37bd, 0x1c, 0, 0},
{0x37c1, 0x2f, 0, 0},
{0x37c3, 0x09, 0, 0},
{0x37c8, 0x1d, 0, 0},
{0x37ca, 0x30, 0, 0},
{0x37df, 0x00, 0, 0},
{0x3800, 0x00, 0, 0},
{0x3801, 0x00, 0, 0},
{0x3802, 0x00, 0, 0},
{0x3803, 0x00, 0, 0},
{0x3804, 0x05, 0, 0},
{0x3805, 0x0f, 0, 0},
{0x3806, 0x04, 0, 0},
{0x3807, 0x0f, 0, 0},
{0x3808, 0x05, 0, 0},
{0x3809, 0x00, 0, 0},
{0x380a, 0x04, 0, 0},
{0x380b, 0x00, 0, 0},
{0x380c, 0x03, 0, 0},
{0x380d, 0x50, 0, 0},
{0x380e, 0x04, 0, 0},
{0x380f, 0x98, 0, 0},
{0x3810, 0x00, 0, 0},
{0x3811, 0x08, 0, 0},
{0x3812, 0x00, 0, 0},
{0x3813, 0x08, 0, 0},
{0x3814, 0x11, 0, 0},
{0x3815, 0x11, 0, 0},
{0x3820, 0x40, 0, 0},
{0x3821, 0x04, 0, 0},
{0x3826, 0x00, 0, 0},
{0x3827, 0x00, 0, 0},
{0x382a, 0x08, 0, 0},
{0x382b, 0x52, 0, 0},
{0x382d, 0xba, 0, 0},
{0x383d, 0x14, 0, 0},
{0x384a, 0xa2, 0, 0},
{0x3866, 0x0e, 0, 0},
{0x3867, 0x07, 0, 0},
{0x3884, 0x00, 0, 0},
{0x3885, 0x08, 0, 0},
{0x3893, 0x68, 0, 0},
{0x3894, 0x2a, 0, 0},
{0x3898, 0x00, 0, 0},
{0x3899, 0x31, 0, 0},
{0x389a, 0x04, 0, 0},
{0x389b, 0x00, 0, 0},
{0x389c, 0x0b, 0, 0},
{0x389d, 0xad, 0, 0},
{0x389f, 0x08, 0, 0},
{0x38a0, 0x00, 0, 0},
{0x38a1, 0x00, 0, 0},
{0x38a8, 0x70, 0, 0},
{0x38ac, 0xea, 0, 0},
{0x38b2, 0x00, 0, 0},
{0x38b3, 0x08, 0, 0},
{0x38bc, 0x20, 0, 0},
{0x38c4, 0x0c, 0, 0},
{0x38c5, 0x3a, 0, 0},
{0x38c7, 0x3a, 0, 0},
{0x38e1, 0xc0, 0, 0},
{0x38ec, 0x3c, 0, 0},
{0x38f0, 0x09, 0, 0},
{0x38f1, 0x6f, 0, 0},
{0x38fe, 0x3c, 0, 0},
{0x391e, 0x00, 0, 0},
{0x391f, 0x00, 0, 0},
{0x3920, 0xa5, 0, 0},
{0x3921, 0x00, 0, 0},
{0x3922, 0x00, 0, 0},
{0x3923, 0x00, 0, 0},
{0x3924, 0x05, 0, 0},
{0x3925, 0x00, 0, 0},
{0x3926, 0x00, 0, 0},
{0x3927, 0x00, 0, 0},
{0x3928, 0x1a, 0, 0},
{0x3929, 0x01, 0, 0},
{0x392a, 0xb4, 0, 0},
{0x392b, 0x00, 0, 0},
{0x392c, 0x10, 0, 0},
{0x392f, 0x40, 0, 0},
{0x4000, 0xcf, 0, 0},
{0x4003, 0x40, 0, 0},
{0x4008, 0x00, 0, 0},
{0x4009, 0x07, 0, 0},
{0x400a, 0x02, 0, 0},
{0x400b, 0x54, 0, 0},
{0x400c, 0x00, 0, 0},
{0x400d, 0x07, 0, 0},
{0x4010, 0xc0, 0, 0},
{0x4012, 0x02, 0, 0},
{0x4014, 0x04, 0, 0},
{0x4015, 0x04, 0, 0},
{0x4017, 0x02, 0, 0},
{0x4042, 0x01, 0, 0},
{0x4306, 0x04, 0, 0},
{0x4307, 0x12, 0, 0},
{0x4509, 0x00, 0, 0},
{0x450b, 0x83, 0, 0},
{0x4604, 0x68, 0, 0},
{0x4608, 0x0a, 0, 0},
{0x4700, 0x06, 0, 0},
{0x4800, 0x64, 0, 0},
{0x481b, 0x3c, 0, 0},
{0x4825, 0x32, 0, 0},
{0x4833, 0x18, 0, 0},
{0x4837, 0x10, 0, 0},
{0x4850, 0x40, 0, 0},
{0x4860, 0x00, 0, 0},
{0x4861, 0xec, 0, 0},
{0x4864, 0x00, 0, 0},
{0x4883, 0x00, 0, 0},
{0x4888, 0x90, 0, 0},
{0x4889, 0x05, 0, 0},
{0x488b, 0x04, 0, 0},
{0x4f00, 0x04, 0, 0},
{0x4f10, 0x04, 0, 0},
{0x4f21, 0x01, 0, 0},
{0x4f22, 0x40, 0, 0},
{0x4f23, 0x44, 0, 0},
{0x4f24, 0x51, 0, 0},
{0x4f25, 0x41, 0, 0},
{0x5000, 0x1f, 0, 0},
{0x500a, 0x00, 0, 0},
{0x5100, 0x00, 0, 0},
{0x5111, 0x20, 0, 0},

//enable strobe for testing
{0x3009, 0b1000,0, 0},
{0x3017, 0xf2, 0, 0},
{0x3920, 0xff, 0, 0},
{0x3924, 0x00, 0, 0},

//for temperature sensor
{0x3667, 0x80,0 ,0},
{0x4d00, 0x4a,0 ,0},
{0x4d01, 0x18,0 ,0},
{0x4d02, 0xbb,0 ,0},
{0x4d03, 0xde,0 ,0},
{0x4d04, 0x93,0 ,0},
{0x4d05, 0xff,0 ,0},
{0x4d09, 0x0a,0 ,0},

};

static const struct reg_value og01a1b_setting_640x512[] = {

{0x0103, 0x01, 0, 0},
{0x0100, 0x00, 0, 0},
{0x0300, 0x0a, 0, 0},
{0x0301, 0x29, 0, 0},
{0x0302, 0x31, 0, 0},
{0x0303, 0x04, 0, 0},
{0x0304, 0x00, 0, 0},
{0x0305, 0xfa, 0, 0},
{0x0306, 0x00, 0, 0},
{0x0307, 0x01, 0, 0},
{0x0308, 0x02, 0, 0},
{0x0309, 0x00, 0, 0},
{0x0310, 0x00, 0, 0},
{0x0311, 0x00, 0, 0},
{0x0312, 0x07, 0, 0},
{0x0313, 0x00, 0, 0},
{0x0314, 0x00, 0, 0},
{0x0315, 0x00, 0, 0},
{0x0320, 0x02, 0, 0},
{0x0321, 0x01, 0, 0},
{0x0322, 0x01, 0, 0},
{0x0323, 0x04, 0, 0},
{0x0324, 0x01, 0, 0},
{0x0325, 0xc2, 0, 0},
{0x0326, 0xce, 0, 0},
{0x0327, 0x04, 0, 0},
{0x0329, 0x02, 0, 0},
{0x032a, 0x04, 0, 0},
{0x032b, 0x04, 0, 0},
{0x032c, 0x02, 0, 0},
{0x032d, 0x01, 0, 0},
{0x032e, 0x00, 0, 0},
{0x300d, 0x02, 0, 0},
{0x300e, 0x04, 0, 0},
{0x3021, 0x08, 0, 0},
{0x301e, 0x03, 0, 0},
{0x3103, 0x00, 0, 0},
{0x3106, 0x08, 0, 0},
{0x3107, 0x40, 0, 0},
{0x3216, 0x01, 0, 0},
{0x3217, 0x00, 0, 0},
{0x3218, 0xc0, 0, 0},
{0x3219, 0x55, 0, 0},
{0x3500, 0x00, 0, 0},
{0x3501, 0x04, 0, 0},
{0x3502, 0x8a, 0, 0},
{0x3506, 0x01, 0, 0},
{0x3507, 0x72, 0, 0},
{0x3508, 0x01, 0, 0},
{0x3509, 0x00, 0, 0},
{0x350a, 0x01, 0, 0},
{0x350b, 0x00, 0, 0},
{0x350c, 0x00, 0, 0},
{0x3541, 0x00, 0, 0},
{0x3542, 0x40, 0, 0},
{0x3605, 0xe0, 0, 0},
{0x3606, 0x41, 0, 0},
{0x3614, 0x20, 0, 0},
{0x3620, 0x0b, 0, 0},
{0x3630, 0x07, 0, 0},
{0x3636, 0xa0, 0, 0},
{0x3637, 0xf9, 0, 0},
{0x3638, 0x09, 0, 0},
{0x3639, 0x38, 0, 0},
{0x363f, 0x09, 0, 0},
{0x3640, 0x17, 0, 0},
{0x3662, 0x06, 0, 0}, // enables 8 bit mode
{0x3665, 0x80, 0, 0},
{0x3670, 0x68, 0, 0},
{0x3674, 0x00, 0, 0},
{0x3677, 0x3f, 0, 0},
{0x3679, 0x00, 0, 0},
{0x369f, 0x19, 0, 0},
{0x36a0, 0x03, 0, 0},
{0x36a2, 0x19, 0, 0},
{0x36a3, 0x03, 0, 0},
{0x370d, 0x66, 0, 0},
{0x370f, 0x00, 0, 0},
{0x3710, 0x03, 0, 0},
{0x3715, 0x03, 0, 0},
{0x3716, 0x03, 0, 0},
{0x3717, 0x06, 0, 0},
{0x3733, 0x00, 0, 0},
{0x3778, 0x00, 0, 0},
{0x37a8, 0x0f, 0, 0},
{0x37a9, 0x01, 0, 0},
{0x37aa, 0x07, 0, 0},
{0x37bd, 0x1c, 0, 0},
{0x37c1, 0x2f, 0, 0},
{0x37c3, 0x09, 0, 0},
{0x37c8, 0x1d, 0, 0},
{0x37ca, 0x30, 0, 0},
{0x37df, 0x00, 0, 0},
{0x3800, 0x00, 0, 0},
{0x3801, 0x00, 0, 0},
{0x3802, 0x00, 0, 0},
{0x3803, 0x00, 0, 0},
{0x3804, 0x05, 0, 0},
{0x3805, 0x0f, 0, 0},
{0x3806, 0x04, 0, 0},
{0x3807, 0x0f, 0, 0},
{0x3808, 0x05, 0, 0},
{0x3809, 0x00, 0, 0},
{0x380a, 0x04, 0, 0},
{0x380b, 0x00, 0, 0},
{0x380c, 0x03, 0, 0},
{0x380d, 0x50, 0, 0},
{0x380e, 0x04, 0, 0},
{0x380f, 0x98, 0, 0},
{0x3810, 0x00, 0, 0},
{0x3811, 0x08, 0, 0},
{0x3812, 0x00, 0, 0},
{0x3813, 0x08, 0, 0},
{0x3814, 0x11, 0, 0},
{0x3815, 0x11, 0, 0},
{0x3820, 0x40, 0, 0},
{0x3821, 0x04, 0, 0},
{0x3826, 0x00, 0, 0},
{0x3827, 0x00, 0, 0},
{0x382a, 0x08, 0, 0},
{0x382b, 0x52, 0, 0},
{0x382d, 0xba, 0, 0},
{0x383d, 0x14, 0, 0},
{0x384a, 0xa2, 0, 0},
{0x3866, 0x0e, 0, 0},
{0x3867, 0x07, 0, 0},
{0x3884, 0x00, 0, 0},
{0x3885, 0x08, 0, 0},
{0x3893, 0x68, 0, 0},
{0x3894, 0x2a, 0, 0},
{0x3898, 0x00, 0, 0},
{0x3899, 0x31, 0, 0},
{0x389a, 0x04, 0, 0},
{0x389b, 0x00, 0, 0},
{0x389c, 0x0b, 0, 0},
{0x389d, 0xad, 0, 0},
{0x389f, 0x08, 0, 0},
{0x38a0, 0x00, 0, 0},
{0x38a1, 0x00, 0, 0},
{0x38a8, 0x70, 0, 0},
{0x38ac, 0xea, 0, 0},
{0x38b2, 0x00, 0, 0},
{0x38b3, 0x08, 0, 0},
{0x38bc, 0x20, 0, 0},
{0x38c4, 0x0c, 0, 0},
{0x38c5, 0x3a, 0, 0},
{0x38c7, 0x3a, 0, 0},
{0x38e1, 0xc0, 0, 0},
{0x38ec, 0x3c, 0, 0},
{0x38f0, 0x09, 0, 0},
{0x38f1, 0x6f, 0, 0},
{0x38fe, 0x3c, 0, 0},
{0x391e, 0x00, 0, 0},
{0x391f, 0x00, 0, 0},
{0x3920, 0xa5, 0, 0},
{0x3921, 0x00, 0, 0},
{0x3922, 0x00, 0, 0},
{0x3923, 0x00, 0, 0},
{0x3924, 0x05, 0, 0},
{0x3925, 0x00, 0, 0},
{0x3926, 0x00, 0, 0},
{0x3927, 0x00, 0, 0},
{0x3928, 0x1a, 0, 0},
{0x3929, 0x01, 0, 0},
{0x392a, 0xb4, 0, 0},
{0x392b, 0x00, 0, 0},
{0x392c, 0x10, 0, 0},
{0x392f, 0x40, 0, 0},
{0x4000, 0xcf, 0, 0},
{0x4003, 0x40, 0, 0},
{0x4008, 0x00, 0, 0},
{0x4009, 0x07, 0, 0},
{0x400a, 0x02, 0, 0},
{0x400b, 0x54, 0, 0},
{0x400c, 0x00, 0, 0},
{0x400d, 0x07, 0, 0},
{0x4010, 0xc0, 0, 0},
{0x4012, 0x02, 0, 0},
{0x4014, 0x04, 0, 0},
{0x4015, 0x04, 0, 0},
{0x4017, 0x02, 0, 0},
{0x4042, 0x01, 0, 0},
{0x4306, 0x04, 0, 0},
{0x4307, 0x12, 0, 0},
{0x4509, 0x00, 0, 0},
{0x450b, 0x83, 0, 0},
{0x4604, 0x68, 0, 0},
{0x4608, 0x0a, 0, 0},
{0x4700, 0x06, 0, 0},
{0x4800, 0x64, 0, 0},
{0x481b, 0x3c, 0, 0},
{0x4825, 0x32, 0, 0},
{0x4833, 0x18, 0, 0},
{0x4837, 0x10, 0, 0},
{0x4850, 0x40, 0, 0},
{0x4860, 0x00, 0, 0},
{0x4861, 0xec, 0, 0},
{0x4864, 0x00, 0, 0},
{0x4883, 0x00, 0, 0},
{0x4888, 0x90, 0, 0},
{0x4889, 0x05, 0, 0},
{0x488b, 0x04, 0, 0},
{0x4f00, 0x04, 0, 0},
{0x4f10, 0x04, 0, 0},
{0x4f21, 0x01, 0, 0},
{0x4f22, 0x40, 0, 0},
{0x4f23, 0x44, 0, 0},
{0x4f24, 0x51, 0, 0},
{0x4f25, 0x41, 0, 0},
{0x5000, 0x1f, 0, 0},
{0x500a, 0x00, 0, 0},
{0x5100, 0x00, 0, 0},
{0x5111, 0x20, 0, 0},

//enable strobe for testing
{0x3009, 0b1000,0, 0},
{0x3017, 0xf2, 0, 0},
{0x3920, 0xff, 0, 0},
{0x3924, 0x00, 0, 0},

//for temperature sensor
{0x3667, 0x80,0 ,0},
{0x4d00, 0x4a,0 ,0},
{0x4d01, 0x18,0 ,0},
{0x4d02, 0xbb,0 ,0},
{0x4d03, 0xde,0 ,0},
{0x4d04, 0x93,0 ,0},
{0x4d05, 0xff,0 ,0},
{0x4d09, 0x0a,0 ,0},

{0x3501, 0x02, 0, 0},
{0x3502, 0x73, 0, 0},
{0x3778, 0x00, 0, 0},
{0x3803, 0x00, 0, 0},
{0x3806, 0x04, 0, 0},
{0x3807, 0x0f, 0, 0},
{0x3808, 0x02, 0, 0},
{0x3809, 0x80, 0, 0},
{0x380a, 0x02, 0, 0},
{0x380b, 0x00, 0, 0},
{0x380c, 0x03, 0, 0},
{0x380d, 0x0c, 0, 0},
{0x380e, 0x02, 0, 0},
{0x380f, 0x81, 0, 0},
{0x3810, 0x00, 0, 0},
{0x3811, 0x04, 0, 0},
{0x3812, 0x00, 0, 0},
{0x3813, 0x04, 0, 0},
{0x3814, 0x22, 0, 0},
{0x3815, 0x22, 0, 0},
{0x3820, 0x40, 0, 0},
{0x3821, 0x04, 0, 0},
{0x4509, 0x80, 0, 0},

};

static const struct reg_value og01a1b_setting_640x480[] = {

{0x0103, 0x01, 0, 0},
{0x0100, 0x00, 0, 0},
{0x0300, 0x0a, 0, 0},
{0x0301, 0x29, 0, 0},
{0x0302, 0x31, 0, 0},
{0x0303, 0x04, 0, 0},
{0x0304, 0x00, 0, 0},
{0x0305, 0xfa, 0, 0},
{0x0306, 0x00, 0, 0},
{0x0307, 0x01, 0, 0},
{0x0308, 0x02, 0, 0},
{0x0309, 0x00, 0, 0},
{0x0310, 0x00, 0, 0},
{0x0311, 0x00, 0, 0},
{0x0312, 0x07, 0, 0},
{0x0313, 0x00, 0, 0},
{0x0314, 0x00, 0, 0},
{0x0315, 0x00, 0, 0},
{0x0320, 0x02, 0, 0},
{0x0321, 0x01, 0, 0},
{0x0322, 0x01, 0, 0},
{0x0323, 0x04, 0, 0},
{0x0324, 0x01, 0, 0},
{0x0325, 0xc2, 0, 0},
{0x0326, 0xce, 0, 0},
{0x0327, 0x04, 0, 0},
{0x0329, 0x02, 0, 0},
{0x032a, 0x04, 0, 0},
{0x032b, 0x04, 0, 0},
{0x032c, 0x02, 0, 0},
{0x032d, 0x01, 0, 0},
{0x032e, 0x00, 0, 0},

{0x300d, 0x02, 0, 0},
{0x300e, 0x04, 0, 0},
{0x3021, 0x08, 0, 0},
{0x301e, 0x03, 0, 0},
{0x3103, 0x00, 0, 0},
{0x3106, 0x08, 0, 0},
{0x3107, 0x40, 0, 0},
{0x3216, 0x01, 0, 0},
{0x3217, 0x00, 0, 0},
{0x3218, 0xc0, 0, 0},
{0x3219, 0x55, 0, 0},
{0x3500, 0x00, 0, 0},
{0x3501, 0x02, 0, 0},
{0x3502, 0x73, 0, 0},
{0x3506, 0x01, 0, 0},
{0x3507, 0x72, 0, 0},
{0x3508, 0x01, 0, 0},
{0x3509, 0x00, 0, 0},
{0x350a, 0x01, 0, 0},
{0x350b, 0x00, 0, 0},
{0x350c, 0x00, 0, 0},
{0x3541, 0x00, 0, 0},
{0x3542, 0x40, 0, 0},
{0x3605, 0xe0, 0, 0},
{0x3606, 0x41, 0, 0},
{0x3614, 0x20, 0, 0},
{0x3620, 0x0b, 0, 0},
{0x3630, 0x07, 0, 0},
{0x3636, 0xa0, 0, 0},
{0x3637, 0xf9, 0, 0},
{0x3638, 0x09, 0, 0},
{0x3639, 0x38, 0, 0},
{0x363f, 0x09, 0, 0},
{0x3640, 0x17, 0, 0},
{0x3662, 0x06, 0, 0},
{0x3665, 0x80, 0, 0},
{0x3670, 0x68, 0, 0},
{0x3674, 0x00, 0, 0},
{0x3677, 0x3f, 0, 0},
{0x3679, 0x00, 0, 0},
{0x369f, 0x19, 0, 0},
{0x36a0, 0x03, 0, 0},
{0x36a2, 0x19, 0, 0},
{0x36a3, 0x03, 0, 0},
{0x370d, 0x66, 0, 0},
{0x370f, 0x00, 0, 0},
{0x3710, 0x03, 0, 0},
{0x3715, 0x03, 0, 0},
{0x3716, 0x03, 0, 0},
{0x3717, 0x06, 0, 0},
{0x3733, 0x00, 0, 0},
{0x3778, 0x10, 0, 0},
{0x37a8, 0x0f, 0, 0},
{0x37a9, 0x01, 0, 0},
{0x37aa, 0x07, 0, 0},
{0x37bd, 0x1c, 0, 0},
{0x37c1, 0x2f, 0, 0},
{0x37c3, 0x09, 0, 0},
{0x37c8, 0x1d, 0, 0},
{0x37ca, 0x30, 0, 0},
{0x37df, 0x00, 0, 0},
{0x3800, 0x00, 0, 0},
{0x3801, 0x00, 0, 0},
{0x3802, 0x00, 0, 0},
{0x3803, 0x20, 0, 0},
{0x3804, 0x05, 0, 0},
{0x3805, 0x0f, 0, 0},
{0x3806, 0x03, 0, 0},
{0x3807, 0xef, 0, 0},
{0x3808, 0x02, 0, 0},
{0x3809, 0x80, 0, 0},
{0x380a, 0x01, 0, 0},
{0x380b, 0xe0, 0, 0},
{0x380c, 0x03, 0, 0},
{0x380d, 0x0c, 0, 0},
{0x380e, 0x02, 0, 0},
{0x380f, 0x81, 0, 0},
{0x3810, 0x00, 0, 0},
{0x3811, 0x04, 0, 0},
{0x3812, 0x00, 0, 0},
{0x3813, 0x04, 0, 0},
{0x3814, 0x22, 0, 0},
{0x3815, 0x22, 0, 0},
{0x3820, 0x60, 0, 0},
{0x3821, 0x05, 0, 0},
{0x3826, 0x00, 0, 0},
{0x3827, 0x00, 0, 0},
{0x382a, 0x08, 0, 0},
{0x382b, 0x52, 0, 0},
{0x382d, 0xba, 0, 0},
{0x383d, 0x14, 0, 0},
{0x384a, 0xa2, 0, 0},
{0x3866, 0x0e, 0, 0},
{0x3867, 0x07, 0, 0},
{0x3884, 0x00, 0, 0},
{0x3885, 0x08, 0, 0},
{0x3893, 0x68, 0, 0},
{0x3894, 0x2a, 0, 0},
{0x3898, 0x00, 0, 0},
{0x3899, 0x31, 0, 0},
{0x389a, 0x04, 0, 0},
{0x389b, 0x00, 0, 0},
{0x389c, 0x0b, 0, 0},
{0x389d, 0xad, 0, 0},
{0x389f, 0x08, 0, 0},
{0x38a0, 0x00, 0, 0},
{0x38a1, 0x00, 0, 0},
{0x38a8, 0x70, 0, 0},
{0x38ac, 0xea, 0, 0},
{0x38b2, 0x00, 0, 0},
{0x38b3, 0x08, 0, 0},
{0x38bc, 0x20, 0, 0},
{0x38c4, 0x0c, 0, 0},
{0x38c5, 0x3a, 0, 0},
{0x38c7, 0x3a, 0, 0},
{0x38e1, 0xc0, 0, 0},
{0x38ec, 0x3c, 0, 0},
{0x38f0, 0x09, 0, 0},
{0x38f1, 0x6f, 0, 0},
{0x38fe, 0x3c, 0, 0},
{0x391e, 0x00, 0, 0},
{0x391f, 0x00, 0, 0},
{0x3920, 0xa5, 0, 0},
{0x3921, 0x00, 0, 0},
{0x3922, 0x00, 0, 0},
{0x3923, 0x00, 0, 0},
{0x3924, 0x05, 0, 0},
{0x3925, 0x00, 0, 0},
{0x3926, 0x00, 0, 0},
{0x3927, 0x00, 0, 0},
{0x3928, 0x1a, 0, 0},
{0x3929, 0x01, 0, 0},
{0x392a, 0xb4, 0, 0},
{0x392b, 0x00, 0, 0},
{0x392c, 0x10, 0, 0},
{0x392f, 0x40, 0, 0},
{0x4000, 0xcf, 0, 0},
{0x4003, 0x40, 0, 0},
{0x4008, 0x00, 0, 0},
{0x4009, 0x07, 0, 0},
{0x400a, 0x02, 0, 0},
{0x400b, 0x54, 0, 0},
{0x400c, 0x00, 0, 0},
{0x400d, 0x07, 0, 0},
{0x4010, 0xc0, 0, 0},
{0x4012, 0x02, 0, 0},
{0x4014, 0x04, 0, 0},
{0x4015, 0x04, 0, 0},
{0x4017, 0x02, 0, 0},
{0x4042, 0x01, 0, 0},
{0x4306, 0x04, 0, 0},
{0x4307, 0x12, 0, 0},
{0x4509, 0x80, 0, 0},
{0x450b, 0x83, 0, 0},
{0x4604, 0x68, 0, 0},
{0x4608, 0x0a, 0, 0},
{0x4700, 0x06, 0, 0},
{0x4800, 0x64, 0, 0},
{0x481b, 0x3c, 0, 0},
{0x4825, 0x32, 0, 0},
{0x4833, 0x18, 0, 0},
{0x4837, 0x10, 0, 0},
{0x4850, 0x40, 0, 0},
{0x4860, 0x00, 0, 0},
{0x4861, 0xec, 0, 0},
{0x4864, 0x00, 0, 0},
{0x4883, 0x00, 0, 0},
{0x4888, 0x90, 0, 0},
{0x4889, 0x05, 0, 0},
{0x488b, 0x04, 0, 0},
{0x4f00, 0x04, 0, 0},
{0x4f10, 0x04, 0, 0},
{0x4f21, 0x01, 0, 0},
{0x4f22, 0x40, 0, 0},
{0x4f23, 0x44, 0, 0},
{0x4f24, 0x51, 0, 0},
{0x4f25, 0x41, 0, 0},
{0x5000, 0x1f, 0, 0},
{0x500a, 0x00, 0, 0},
{0x5100, 0x00, 0, 0},
{0x5111, 0x20, 0, 0},

};

static const struct og01a1b_mode_info og01a1b_mode_data_1280x1024 = {
	1280, 848, 1024, 1176, og01a1b_setting, ARRAY_SIZE(og01a1b_setting)
};

static const struct og01a1b_mode_info og01a1b_mode_data_640x512 = {
	640, 848, 512, 1176, og01a1b_setting_640x512, ARRAY_SIZE(og01a1b_setting_640x512)
};

static const struct og01a1b_mode_info og01a1b_mode_data_640x480 = {
	640, 848, 480, 1176, og0VE10_setting, ARRAY_SIZE(og0VE10_setting)
};

static const s64 link_freq_menu_items[] = {
	OG01A1B_LINK_FREQUENCY,
};

static int og01a1b_write_reg(struct og01a1b_dev *sensor, u16 reg, u8 val)
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

static int og01a1b_add_or_set_group_reg(struct og01a1b_dev *sensor, int ctrlval)
{

	u16 address = ctrlval & 0xffff;
	u8 val = ctrlval >> 16 & 0xff;
	u8 group = ctrlval >> 24 & 0xf;

	struct register_set *reg_set =  &sensor->active_group_registers.group_register_sets[group];
	int index = -1;
	for(int i = 0; i < reg_set->num_group_reg; i++) 
	{
		if(reg_set->group_reg_addresses[i] == address)
		{
			index = i;
			break;
		}
	}
	if(index == -1) 
	{
		if(reg_set->num_group_reg >= GROUP_REGMAP_MAX_SIZE) 
		{
			struct i2c_client *client = sensor->i2c_client;
			dev_err(&client->dev, "Group register map exceeded maximum size");
			return -1;
		}
		index = reg_set->num_group_reg;
		reg_set->num_group_reg++;
		reg_set->group_reg_addresses[index] = address;
		sensor->updateGroupRegisters = true;
	}
	if(reg_set->group_reg_values[index] != val)
	{
		reg_set->group_reg_values[index] = val;
		sensor->updateGroupRegisters = true;
	}


	return 0;
}

int og01a1b_write_group_registers_if_needed(struct og01a1b_dev *sensor)
{
	int ret;
	if(!sensor->updateGroupRegisters)
		return 0;
	sensor->updateGroupRegisters = false;

	for(int group = 0; group < NUM_REGISTER_GROUPS; group++) 
	{
		ret = og01a1b_write_reg(sensor, 0x3208, 0x00 | group);
		if(ret != 0) {
			return ret;
		}
		for(int i = 0; i < sensor->active_group_registers.group_register_sets[group].num_group_reg; i++) 
		{
			ret = og01a1b_write_reg(sensor, sensor->active_group_registers.group_register_sets[group].group_reg_addresses[i],sensor->active_group_registers.group_register_sets[group].group_reg_values[i]);
			if(ret != 0) 
			{
				return ret;
			}
		}
		ret = og01a1b_write_reg(sensor, 0x3208, 0x10 | group);
		if(ret!= 0)
		{
			return ret;
		}
	}
	return 0;

}

static int og01a1b_read_reg(struct og01a1b_dev *sensor, u16 reg, u8 *val)
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

static int og01a1b_read_reg16(struct og01a1b_dev *sensor, u16 reg, u16 *val)
{
	u8 hi, lo;
	int ret;

	ret = og01a1b_read_reg(sensor, reg, &hi);
	if (ret)
		return ret;
	ret = og01a1b_read_reg(sensor, reg + 1, &lo);
	if (ret)
		return ret;

	*val = ((u16)hi << 8) | (u16)lo;
	return 0;
}

static int og01a1b_write_reg16(struct og01a1b_dev *sensor, u16 reg, u16 val)
{
	int ret;

	ret = og01a1b_write_reg(sensor, reg, val >> 8);
	if (ret)
		return ret;

	return og01a1b_write_reg(sensor, reg + 1, val & 0xff);
}

/* download og01a1b settings to sensor through i2c */
static int og01a1b_set_timings(struct og01a1b_dev *sensor,
			      const struct og01a1b_mode_info *mode)
{
	int ret;

	return 0;
	// ret = og01a1b_write_reg16(sensor, OG01A1B_REG_TIMING_DVPHO, mode->hact);
	// if (ret < 0)
	// 	return ret;

	// ret = og01a1b_write_reg16(sensor, OG01A1B_REG_TIMING_DVPVO, mode->vact);
	// if (ret < 0)
	// 	return ret;

	// if(sensor->ep.bus.mipi_csi2.num_data_lanes == 1) {
	// 	ret = og01a1b_write_reg16(sensor, OG01A1B_REG_TIMING_HTS, mode->htot*2);
	// }
	// else if(sensor->ep.bus.mipi_csi2.num_data_lanes == 2) {
	// 	ret = og01a1b_write_reg16(sensor, OG01A1B_REG_TIMING_HTS, mode->htot);
	// }
	// else {
	// 	struct i2c_client *client = sensor->i2c_client;
	// 	dev_err(&client->dev, "Unsupported lane configuration. Must be 1 or 2 lane MIPI");
	// 	return -EINVAL;
	// }
	// if (ret < 0)
	// 	return ret;

	// return og01a1b_write_reg16(sensor, OG01A1B_REG_TIMING_VTS, mode->vtot);
}

static int og01a1b_load_regs(struct og01a1b_dev *sensor,
			    const struct og01a1b_mode_info *mode)
{
	const struct reg_value *regs = mode->reg_data;
	unsigned int i;
	u32 delay_ms;
	u16 reg_addr;
	u8 mask, val;
	int ret = 0;

	for (i = 0; i < mode->reg_data_size; ++i, ++regs) {
		delay_ms = regs->delay_ms;
		reg_addr = regs->reg_addr;
		val = regs->val;
		mask = regs->mask;

		ret = og01a1b_write_reg(sensor, reg_addr, val);
		if (ret)
			break;

		if (delay_ms)
			usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
	}

	return og01a1b_set_timings(sensor, mode);
}

/* restore the last set video mode after chip power-on */
static int og01a1b_restore_mode(struct og01a1b_dev *sensor)
{
	int ret;
	u8 val;

	/* first load the initial register values */
	ret = og01a1b_load_regs(sensor, (sensor->current_mode));
	if (ret < 0)
		return ret;

	ret = og01a1b_read_reg(sensor, 0x3662, &val);
	// Verify that register map was succesfully changed from the default value
	// (Power on default:0b100 )
	// if (ret < 0 || val != 0x24) {
	// 	struct i2c_client *client = sensor->i2c_client;
	// 	const char *id;
	// 	dev_err(&client->dev, "Register map not verified  as written. Likely hardware problem with %s\n", of_property_read_string( client->dev.of_node, "id", &id ) ? "unknown camera" : id );
	// 	return -EINVAL;
	// }
	struct i2c_client *client = sensor->i2c_client;
	dev_err(&client->dev, "got 0x%x from 0x%x", val, 0x3662);

	//set registers for 1 lane mode if necessary
	// if(sensor->ep.bus.mipi_csi2.num_data_lanes == 1) {
	// 	og01a1b_write_reg(sensor, 0x3662, 0b010);
	// 	og01a1b_write_reg(sensor, 0x3039, 0x32);

	// }

	return og01a1b_set_timings(sensor, (sensor->current_mode));
}

static void og01a1b_reset(struct og01a1b_dev *sensor)
{
	if (!sensor->reset_gpio)
		return;

	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 5100);
}

static int og01a1b_set_power_on(struct og01a1b_dev *sensor)
{
	
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	// hard code clock rate to 24 mhz
	clk_set_rate(sensor->xclk, 24000000);

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		return ret;
	}

	ret = regulator_bulk_enable(og01a1b_NUM_SUPPLIES,
				    sensor->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		goto xclk_off;
	}

	og01a1b_reset(sensor);

	return 0;

xclk_off:
	clk_disable_unprepare(sensor->xclk);
	return ret;
}

static void og01a1b_set_power_off(struct og01a1b_dev *sensor)
{
	regulator_bulk_disable(og01a1b_NUM_SUPPLIES, sensor->supplies);
	clk_disable_unprepare(sensor->xclk);
}

static int og01a1b_set_power(struct og01a1b_dev *sensor, bool on)
{
	int ret = 0;

	if (on) {
		ret = og01a1b_set_power_on(sensor);
		if (ret)
			return ret;

		ret = og01a1b_restore_mode(sensor);
		if (ret)
			goto power_off;

	} else {
		og01a1b_set_power_off(sensor);
	}

	return 0;

power_off:
	og01a1b_set_power_off(sensor);
	return ret;
}

static long og01a1b_private_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg);

static int og01a1b_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct og01a1b_dev *sensor =
			container_of(ctrl->handler, struct og01a1b_dev, ctrl_handler);
	int ret = 0;

	/* V4L2 controls values will be applied only when power is already up */
	//if (!pm_runtime_get_if_in_use(&client->dev))
	//	return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		/* note:
		 * value in 5.4 fractional format
		 */
		ret = og01a1b_write_reg(sensor, 0x3508, (ctrl->val >> 4) & 0x1f);
		ret = og01a1b_write_reg(sensor, 0x3509, (ctrl->val << 4) & 0xf0);

		break;

    // "Hack controls" that are kept for backward compatibility
	case V4L2_CID_BRIGHTNESS:
        ret = og01a1b_private_ioctl(&sensor->sd, VIDIOC_OG01A1B_PRV_S_REG, (void*)(uintptr_t)ctrl->val);
		break;
	case V4L2_CID_HUE:
        ret = og01a1b_private_ioctl(&sensor->sd, VIDIOC_OG01A1B_PRV_S_GROUP_REG, (void*)(uintptr_t)ctrl->val);
		break;
	case V4L2_CID_SATURATION:
        ret = og01a1b_private_ioctl(&sensor->sd, VIDIOC_OG01A1B_PRV_S_GROUP_MODE, (void*)(uintptr_t)ctrl->val);
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int og01a1b_get_ctrl(struct v4l2_ctrl *ctrl)
{
	u16 temperature;
	struct og01a1b_dev *sensor =
			container_of(ctrl->handler, struct og01a1b_dev, ctrl_handler);

	og01a1b_read_reg16(sensor, OG01A1B_REG_TPM_TEMPERATURE, &temperature);
	ctrl->val = temperature;
	ctrl->has_changed = 1;
	ctrl->cur.val = temperature;
	return 0;
}

static const struct v4l2_ctrl_ops og01a1b_ctrl_ops = {
	.s_ctrl = og01a1b_set_ctrl,
};

static const struct v4l2_ctrl_ops og01a1b_ctrl_read_ops = {
	.g_volatile_ctrl = og01a1b_get_ctrl,
};

static const struct v4l2_ctrl_config og01a1b_ctrl_temperature = {
	.ops = &og01a1b_ctrl_read_ops,
	.id = V4L2_CID_TEMPERATURE,
	.name = "Temperature",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.flags = V4L2_CTRL_FLAG_VOLATILE,
	.min = 0,
	.max = 0xffffffff,
	.step = 1,
};

static void* get_metadata(struct v4l2_subdev *sd) {
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	unsigned long flags;
	spin_lock_irqsave(&metadata_spinlock, flags);
	memcpy(&sensor->outputSensorMetaData,&sensor->sensorMetaData, sizeof(struct SensorMetaData));
	sensor->sensorMetaData.flags = 0;
	spin_unlock_irqrestore(&metadata_spinlock, flags);
	
	return &sensor->outputSensorMetaData;

}

void set_ir_leds(struct og01a1b_dev *sensor, int state) {
	unsigned long flags;
	spin_lock_irqsave(&ir_spinlock, flags);
	for(int i = 0; i < 2; i++) {
		if(sensor->ir_gpio[i]!=NULL && sensor->ir_emitter_mode != IR_EMITTER_MODE_OFF) {
			gpiod_set_value(sensor->ir_gpio[i], state);
		}
	}

	// uncomment this to debug how long emitters were on for
	// if(sensor->last_set_led_state == 1 && state ==0) {
	// 	u64 ts = ktime_get_ns();
	// 	printk("emitters were on for %lu ns\n", ts - sensor->last_emitter_on_ts);
	// }
	sensor->last_set_led_state = state;
	spin_unlock_irqrestore(&ir_spinlock, flags);
}

bool emitter_enabled_for_group(struct og01a1b_dev *sensor, int group) {
	//printk("sensor->ir_emitter_mode:%d group:%d",sensor->ir_emitter_mode,group);
	if(sensor->ir_emitter_mode == IR_EMITTER_MODE_ALL_FRAMES) {
		return true;
	}
	else if(sensor->ir_emitter_mode == IR_EMITTER_MODE_GROUP_1_ONLY) {
		return group == 1;
	}
	else if(sensor->ir_emitter_mode == IR_EMITTER_MODE_GROUP_0_ONLY) {
		return group == 0;
	}
	return false;
}

void handle_exposure_event(struct og01a1b_dev *sensor, bool is_gpio_event) {
	// todo: don't turn it on if it's been turned on within a millisecond
    u64 ts = ktime_get_ns();
	int currentGroup = -1;
		if(sensor->is_sync_sensor && sensor->group_mode != MODE_NOGROUPS) {
			//find the most recent group switch before the last sof timestamp
			for(int i = 0; i < 4; i++) {
				if(sensor->last_sof_ts > sensor->sensorMetaData.group_and_reg[i].post_group_write_timestamp )
				{
					currentGroup = sensor->sensorMetaData.group_and_reg[i].group_written;
				}
			}
		}
		if(sensor->is_sync_sensor && sensor->group_mode == MODE_NOGROUPS) {
			currentGroup = 0;
		}

	//int state = gpiod_get_value(sensor->strobe_gpio);
	if(is_gpio_event) {
		sensor->sensorMetaData.currentSOE = ts;
		// only turn the emitters on if they are enabled, present and the last_emitter_on_ts is more than 4 ms ago
		// this prevents a problem when the group 0 exposure  is long and prevents a nearly 50% duty cycle for some safety
		//if(sensor->is_sync_sensor && ((ts - sensor->last_emitter_on_ts )> 4000000)) {
		if(sensor->is_sync_sensor) {
			if(emitter_enabled_for_group(sensor, currentGroup)) {
				if(sensor->power_count > 0 && sensor->last_set_led_state == 0 && ((ts - sensor->last_emitter_on_ts ) > 4000000)) {
					set_ir_leds(sensor, 1);
					sensor->last_emitter_on_ts = ts;
					//printk("gpio edge on at :%lu\n", ts);
				}
				else {
					set_ir_leds(sensor, 0);
					//printk("gpio edge off at :%lu\n", ts);
				}
			}
			else {
				set_ir_leds(sensor, 0);
			}
		}
	}
	else
	{
		if(sensor->is_sync_sensor ) {
			set_ir_leds(sensor, 0);
		}
		// we use this spinlock to prevent reading corrupted data because these are on different cores
		// We use the non irqsave version because this is always called from an atomic context and we
		// Just need to synchronize across cores.
		spin_lock(&group_spinlock);
		sensor->sensorMetaData.lastEOE = ts;
		sensor->sensorMetaData.lastSOE = sensor->sensorMetaData.currentSOE;
		spin_unlock(&group_spinlock);
	}
}

static void do_post_sof_work(struct v4l2_subdev *sd, u64 sof_timestamp, u64 sof_isr_timestamp) {
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	handle_exposure_event(sensor, false);
	if(sensor->is_sync_sensor ) {
		u64 ts = ktime_get_ns();
		//printk("sof off at :%lu\n", ts);
		set_ir_leds(sensor, 0);
	}
	sensor->last_sof_ts = sof_timestamp;
	sensor->last_sof_isr_ts = sof_isr_timestamp;

	if(numSensors ==2) 
	{	
		//we need to check the other sensors timestamp vs this one because it is possible that the other sensor fired the IRQ first but this sensor finished the SOF first
		if(((sof_timestamp - sensor->other_sensor->last_sof_ts) < 3000000 || (sensor->other_sensor->last_sof_ts - sof_timestamp) < 3000000) && sensor->group_mode != MODE_NOGROUPS)
		{
			if(sensor->is_sync_sensor) {
				sensor->last_requested_groupwrite_isp_ts = sof_timestamp;
				schedule_work(&sensor->work);
			}
			else 
			{
				sensor->other_sensor->last_requested_groupwrite_isp_ts = sof_timestamp;
				schedule_work(&sensor->other_sensor->work);
			}
		}
		return;

	}
	else {
		
		if(sensor->group_mode != MODE_NOGROUPS) 
		{
			for(int i = 0; i < numSensors; i++) {
				u64 start = sof_timestamp - 3000000;
				u64 end = sof_timestamp + 3000000;
				//printk("sensor_index:%d i:%d start: %lu, end: %lu, last_sof_ts: %lu\n", sensor->sensor_index, i, start, end, sensors[i]->last_sof_ts);
				if(!sensors[i]->is_streaming ) {
					continue;
				}
				if(sensors[i]->last_sof_ts < start || sensors[i]->last_sof_ts > end)
				{
					return;
				}
			}
			if(sensor->is_sync_sensor) {
				schedule_work(&sensor->work);
			}
			else {
				schedule_work(&sensor->other_sensor->work);
			}
		}
		
		return;
	}
}

static int og01a1b_init_controls(struct og01a1b_dev *sensor)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	int ret;
    int size = 0;
    struct v4l2_ctrl *link_freq = NULL;
    struct v4l2_ctrl *size_control = NULL;    

	ctrl_hdlr = &sensor->ctrl_handler;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 2);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &sensor->mutex;

	v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			0, 30000, 1, 0x10);
	// v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops, V4L2_CID_EXPOSURE,
	// 		0, 800, 1, 250);

	/* hack control for custom i2c writes */
	v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops, V4L2_CID_BRIGHTNESS, 0, 0xffffff, 1, 0);

	v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops, V4L2_CID_HUE, -2147483648, 2147483647, 1, 0);

	v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops, V4L2_CID_SATURATION, 0, 2, 1, 0);
	
	v4l2_ctrl_new_std_compound(ctrl_hdlr, &og01a1b_ctrl_ops,
				   V4L2_CID_COLORIMETRY_HDR10_CLL_INFO,
				   v4l2_ctrl_ptr_create(NULL));
	

	size = ARRAY_SIZE(link_freq_menu_items);

	link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &og01a1b_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   size - 1, 0,
						   link_freq_menu_items);
	if (link_freq)
		link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	size = ARRAY_SIZE(sensor->metadata_cb_address);

	v4l2_ctrl_new_int_menu(ctrl_hdlr, &og01a1b_ctrl_ops,
						   V4L2_CID_METADATA_CB_ADDR,
						   size - 1, 0,
						   sensor->metadata_cb_address);
	sensor->metadata_cb_address[0] = (s64)(get_metadata);
	size = ARRAY_SIZE(sensor->metadata_size);
	size_control = v4l2_ctrl_new_int_menu(ctrl_hdlr, &og01a1b_ctrl_ops,
						   V4L2_CID_METADATA_SIZE,
						   size - 1, 0,
						   sensor->metadata_size);
	sensor->sof_cb_address[0] = (s64)(do_post_sof_work);
	size = ARRAY_SIZE(sensor->sof_cb_address);
	v4l2_ctrl_new_int_menu(ctrl_hdlr, &og01a1b_ctrl_ops,
						   V4L2_CID_SENSOR_SOF_CB_ADDR,
						   size - 1, 0,
						   sensor->sof_cb_address);

	sensor->metadata_size[0] = sizeof(struct SensorMetaData);
	if (size_control)
		size_control->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	//add custom control for temperature sensor
	v4l2_ctrl_new_custom(ctrl_hdlr, &og01a1b_ctrl_temperature, NULL);

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	sensor->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

/* --------------- Subdev Operations --------------- */

static int og01a1b_s_power(struct v4l2_subdev *sd, int on)
{
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	int ret = 0;
	static char *gpionames[] = {"ira", "irb"};

	mutex_lock(&sensor->lock);

	if(sensor->is_sync_sensor) {
		struct i2c_client *client = sensor->i2c_client;
		sensor->ir_emitter_mode = false;
		sensor->last_emitter_on_ts = 0;
		unsigned long flags;
		sensor->last_emitter_on_ts = 0;
		spin_lock_irqsave(&ir_spinlock, flags);
		if(!!on) {
			sensor->emittersPresent = true;

			for(int i = 0; i < 2; i++)
			{
				sensor->ir_gpio[i] = devm_gpiod_get_optional(&client->dev, gpionames[i],
									GPIOD_OUT_LOW);
				
				
				if(sensor->ir_gpio[i] == NULL)
				{
					//if we don't acquire the gpio assume emitters are not present
					sensor->emittersPresent = false;
					break;
				}
			}
			//if an emitter is not found make sure both gpio_desc* are NULL
			//and return the GPIO so userspace can use it
			if(!sensor->emittersPresent) {
				for(int i = 0; i < 2; i++)
				{
					if(sensor->ir_gpio[i] != NULL) {
						devm_gpiod_put(&client->dev,sensor->ir_gpio[i]);
					}
					sensor->ir_gpio[i] = NULL;
				}
			}
				
		}
		else 
		{
			if(sensor->emittersPresent) {
			
				gpiod_set_value(sensor->ir_gpio[0], 0);
				gpiod_set_value(sensor->ir_gpio[1], 0);

				for(int i = 0; i < 2; i++)
				{
					devm_gpiod_put(&client->dev,sensor->ir_gpio[i]);
					sensor->ir_gpio[i] = NULL;
				}
			}
		}
		spin_unlock_irqrestore(&ir_spinlock, flags);	
	}

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (sensor->power_count == !on) {
		if(!!on == 0) 
		{
			sensor->updateGroupRegisters = true;
		}
		ret = og01a1b_set_power(sensor, !!on);
		if (ret)
			goto out;
	}

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);
out:
	mutex_unlock(&sensor->lock);

	return ret;
}

void reset_groups(struct og01a1b_dev *sensor) 
{
	sensor->active_group_registers.pre_group_write_timestamp = 0;
	sensor->active_group_registers.post_group_write_timestamp = 0;
	for(int i = 0; i < NUM_REGISTER_GROUPS; i++) {
		sensor->active_group_registers.group_register_sets[i].num_group_reg = 0;
		for(int j = 0; j < GROUP_REGMAP_MAX_SIZE; j++) {
			sensor->active_group_registers.group_register_sets[i].group_reg_addresses[j] = 0x00;
			sensor->active_group_registers.group_register_sets[i].group_reg_values[j] = 0x00;
		}
	}
	sensor->active_group_registers.group_register_sets->num_group_reg = 0;
	
	memcpy(&sensor->sensorMetaData.group_and_reg[4], &sensor->active_group_registers,sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[3], &sensor->active_group_registers,sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[2], &sensor->active_group_registers,sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[1], &sensor->active_group_registers,sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[0], &sensor->active_group_registers,sizeof(struct group_and_registers));
}




static irqreturn_t gpio_handler(int irq, void * ident)
{
	struct og01a1b_dev *sensor = (struct og01a1b_dev *)ident;
	u64 ts = ktime_get_ns();
	handle_exposure_event(sensor, true);
	
	return IRQ_HANDLED;
}

static int og01a1b_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	sensor->updateGroupRegisters = true;
	reset_groups(sensor);
	sensor->is_streaming = !!enable;

	og01a1b_write_reg(sensor, 0x100, !!enable);
	return 0;
}

// This ioctl is used to get the metadata version and enable / disable IR LED controls
// TODO replace some of the misused ioctl (CID_SATURATION etc) with this IOCTL
static long og01a1b_private_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
    u32 ret = 0;
	u32 *a = arg;
	unsigned long flags;
    switch (cmd)
    {
        case VIDIOC_OG01A1B_PRV_G_METADATA_VERSION:
        {
            *a = ((METADATA_MAJOR_VERSION << 16) & 0xffff0000) | (METADATA_MINOR_VERSION & 0xffff);
            break;
        }
        case VIDIOC_OG01A1B_PRV_S_IR_EMITTER_MODE:
        {
            //only allow this to be enabled if emitters are present. Else return -1
            if(sensor->emittersPresent) {
                sensor->ir_emitter_mode = *a;

                // Turn off IR emitters whenever this is called.
                // This ensures that even if this GPIO is called repeatedly
                // That it will always leave the IR leds off
                // And if the emitters are disabled they won't be turned back on because the check
                // for sensor->ir_emitter_mode is inside the section which is exclusive to ir_spinlock
                spin_lock_irqsave(&ir_spinlock, flags);
                for(int i = 0; i < 2; i++) {
                    if(sensor->ir_gpio[i]!=NULL) {
                        gpiod_set_value(sensor->ir_gpio[i], 0);
                    }
                }
                spin_unlock_irqrestore(&ir_spinlock, flags);
            }
            else 
            {
                if(sensor->is_sync_sensor) {
                    dev_err(&client->dev, "%s: Unable to enable emitters because they were not detected as connected\n", __func__);
                    ret = -1;
                }
            }
            break;
        }
        case VIDIOC_OG01A1B_PRV_S_REG:
        {
            u32 val = (uintptr_t)arg; // argument is passed as an integer value
            if(sensor->is_sync_sensor) {
                mutex_lock(&sensor->lock);
            }
            else {
                if(sensor->other_sensor != NULL) {
                    mutex_lock(&sensor->other_sensor->lock);
                }
            }
            ret = og01a1b_write_reg(sensor, val & 0xffff, val >> 16 & 0xff);
            if(sensor->is_sync_sensor) {
                    mutex_unlock(&sensor->lock);
            }
            else {
                if(sensor->other_sensor != NULL) {
                    mutex_unlock(&sensor->other_sensor->lock);
                }
            }
            break;
        }
        case VIDIOC_OG01A1B_PRV_S_GROUP_REG:
        {
            //ret = og01a1b_queue_reg(sensor, val & 0xffff, val >> 16 & 0xff);
            mutex_lock(&sensor->lock);
            ret = og01a1b_add_or_set_group_reg(sensor, (uintptr_t)arg); // argument is passed as an integer value
            mutex_unlock(&sensor->lock);
            break;
        }
        case VIDIOC_OG01A1B_PRV_S_GROUP_MODE:
        {
            sensor->group_mode = (uintptr_t)arg; // argument is passed as an integer value
            break;
        }
        default:
        {
            ret = -EINVAL;
        }
    }

	return ret;
}

static const struct v4l2_subdev_core_ops og01a1b_core_ops = {
	.s_power = og01a1b_s_power,
	.ioctl = og01a1b_private_ioctl,
};

static const struct v4l2_subdev_video_ops og01a1b_video_ops = {
	.s_stream = og01a1b_s_stream,
};

static int og01a1b_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	code->code = MEDIA_BUS_FMT_Y8_1X8;
	return 0;
}

static int og01a1b_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);

	mutex_lock(&sensor->lock);

	if(fmt->format.width == 1280 && fmt->format.height == 1024) 
	{
		sensor->current_mode = &og01a1b_mode_data_1280x1024;
		sensor->is_og0VE10 = false;
	}
	else if (fmt->format.width == 640 && fmt->format.height == 512) 
	{
		sensor->current_mode = &og01a1b_mode_data_640x512;
	}
	else if (fmt->format.width == 640 && fmt->format.height == 480) 
	{
		sensor->is_og0VE10 = true;
		sensor->current_mode = &og01a1b_mode_data_640x480;
	}
	else {
		mutex_unlock(&sensor->lock);
		return -1;
	}

	sensor->fmt.width = fmt->format.width;
	sensor->fmt.height = fmt->format.height;

	//TODO: return non zero value if invalid format


	mutex_unlock(&sensor->lock);

	return 0;
}

static int og01a1b_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	format->format = sensor->fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}



static const struct v4l2_subdev_pad_ops og01a1b_pad_ops = {
	.enum_mbus_code = og01a1b_enum_mbus_code,
	.get_fmt = og01a1b_get_fmt,
	.set_fmt = og01a1b_set_fmt,
};

static const struct v4l2_subdev_ops og01a1b_subdev_ops = {
	.core = &og01a1b_core_ops,
	.video = &og01a1b_video_ops,
	.pad = &og01a1b_pad_ops,
};

static int og01a1b_get_regulators(struct og01a1b_dev *sensor)
{
	int i;

	for (i = 0; i < og01a1b_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = og01a1b_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       og01a1b_NUM_SUPPLIES,
				       sensor->supplies);
}

static int og01a1b_check_chip_id(struct og01a1b_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device *dev = &client->dev;
	const char *id;
	int ret = 0;
	u16 chip_id;

	ret = og01a1b_set_power_on(sensor);
	if (ret)
		return ret;

	ret = og01a1b_read_reg16(sensor, OG01A1B_REG_CHIP_ID, &chip_id);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read chip identifier\n",
			__func__);
		goto power_off;
	}

	if (chip_id != 0x4701) {
		dev_err(&client->dev, "%s: wrong chip identifier, expected 0x4701, got this 0x%x. Likely hardware problem with %s\n",
			__func__, chip_id, of_property_read_string( dev->of_node, "id", &id ) ? "unknown camera" : id);
		ret = -ENXIO;
		ret = 0;
	}
	else {
		dev_err(&client->dev, "%s: Camera connection verified ( chip ID 0x%x ) for %s\n",
			__func__, chip_id, of_property_read_string( dev->of_node, "id", &id ) ? "unknown camera" : id);
	}

power_off:
	og01a1b_set_power_off(sensor);
	return ret;
}


int last_set_group = 0;

void update_groups(struct og01a1b_dev *sensor)
{
    u64 last_group_pre_write_timestamp = 0;
    u64 lastGroupPostWriteTimestamp = 0;
    unsigned long flags;

	mutex_lock(&sensor->lock);

	last_group_pre_write_timestamp = ktime_get_ns();
		for(int i = 0; i < numSensors; i++) {
			if(sensors[i]->power_count < 1 ) {
				continue;
			}
			og01a1b_write_group_registers_if_needed(sensors[i]);
		}

  if(sensor->group_mode == MODE_2GROUPS) {
	last_set_group = !last_set_group;
  }
  else
  {
	last_set_group = 0;
  }
  
  for(int i = 0; i < numSensors; i++) {
	if(sensors[i]->power_count < 1 ) {
		continue;
	}
	og01a1b_write_reg(sensors[i], 0x3208, 0xa0 | last_set_group );
  }
  lastGroupPostWriteTimestamp = ktime_get_ns();

  for(int i = 0; i < numSensors; i++) {
	if(sensors[i]->power_count < 1 ) {
		continue;
	}
	sensors[i]->active_group_registers.group_written = last_set_group;
	sensors[i]->active_group_registers.pre_group_write_timestamp = last_group_pre_write_timestamp;
	sensors[i]->active_group_registers.sof_ts = sensor->last_sof_ts;
	sensors[i]->active_group_registers.sof_isr_ts = sensor->last_sof_isr_ts;
	sensors[i]->active_group_registers.post_group_write_timestamp = lastGroupPostWriteTimestamp;
  }
  spin_lock_irqsave(&metadata_spinlock, flags);
  for(int i = 0; i < numSensors; i++) {
	if(sensors[i]->power_count < 1 ) {
		continue;
	}
	memcpy(&sensors[i]->sensorMetaData.group_and_reg[4], &sensors[i]->sensorMetaData.group_and_reg[3],sizeof(struct group_and_registers));
	memcpy(&sensors[i]->sensorMetaData.group_and_reg[3], &sensors[i]->sensorMetaData.group_and_reg[2],sizeof(struct group_and_registers));
	memcpy(&sensors[i]->sensorMetaData.group_and_reg[2], &sensors[i]->sensorMetaData.group_and_reg[1],sizeof(struct group_and_registers));
	memcpy(&sensors[i]->sensorMetaData.group_and_reg[1], &sensors[i]->sensorMetaData.group_and_reg[0],sizeof(struct group_and_registers));
	memcpy(&sensors[i]->sensorMetaData.group_and_reg[0], &sensors[i]->active_group_registers,sizeof(struct group_and_registers));
  }
  spin_unlock_irqrestore(&metadata_spinlock, flags);

  mutex_unlock(&sensor->lock);
}






static void update_groups_work_fn(struct work_struct *Wq)
{
	struct og01a1b_dev *sensor = container_of(Wq, struct og01a1b_dev, work);
	update_groups(sensor);

}


static int og01a1b_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct og01a1b_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;
    const char *id;
    bool skip_check_chip_id = false;
	static int num = 0; 

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	sensor->is_streaming = false;
	sensor->is_og0VE10 = false;
	sensor->sensor_index = numSensors;
	sensor->last_set_led_state = 0;
	if (of_property_read_string(dev->of_node, "id", &id)) {
        dev_err(dev, "Failed to read 'id' property\n");
        return -EINVAL;
    }
    if (strcmp(id, "slam_left") == 0) {
		sensor->is_sync_sensor = true;
		INIT_WORK(&sensor->work,update_groups_work_fn);
		sync_sensor_index = sensor->sensor_index;
		for(int i = 0; i < numSensors; i++) {
			sensors[i]->other_sensor = sensor;

			// We keep this in for the 2 camera compatibility case
			sensor->other_sensor = sensors[i];
		}
    } else {
		sensor->is_sync_sensor = false;
		if(sync_sensor_index != -1) {
			sensor->other_sensor = sensors[sync_sensor_index];
			// We keep this in for the 2 camera compatibility case
			sensors[sync_sensor_index]->other_sensor = sensor;
		}
    }
	
	sensors[numSensors] = sensor;
	numSensors++;
	sensor->group_mode = MODE_NOGROUPS;

	sensor->i2c_client = client;

	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_Y8_1X8;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 1280;
	fmt->height = 1024;
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

	sensor->strobe_gpio = devm_gpiod_get_optional(dev, "strobe",
						     GPIOD_IN);
	if(sensor->strobe_gpio  == NULL) {
		return -1;
	}

	for(int i = 0; i < 2; i++) {
		sensor->ir_gpio[i] = NULL;
	}
	v4l2_i2c_subdev_init(&sensor->sd, client, &og01a1b_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	ret = og01a1b_get_regulators(sensor);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	skip_check_chip_id  = of_property_read_bool( dev->of_node, "skip-check-id" );
	if(!skip_check_chip_id) {
		u32 also_check_index;
		ret = og01a1b_check_chip_id(sensor);
			if (ret)
			{
				dev_err(&client->dev, "Failed to get chip ID for %s\n", of_property_read_string( dev->of_node, "id", &id ) ? "unknown camera" : id );
				//Continue because 2 of these cameras share a power pin and this can prevent this from working at probe
				//goto entity_cleanup;
			}

		if (of_property_read_u32( dev->of_node, "also-check-index", &also_check_index) == 0) {
			printk("also_check_index:%d sensor->sensor_index:%d\n",also_check_index,sensor->sensor_index);
			if(also_check_index < sensor->sensor_index) {
				struct og01a1b_dev *other_sensor = sensors[also_check_index];
				//printk("Would have checked chip id of 0x%x",other_sensor);
				og01a1b_check_chip_id(other_sensor);
			}
		}
		else {
			printk("also_check_index:%d sensor->sensor_index:%d\n",also_check_index,sensor->sensor_index);
		}
	}

	mutex_init(&sensor->mutex);
	ret = og01a1b_init_controls(sensor);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto entity_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto free_ctrls;

	if(num < 4) {
		int gpio = desc_to_gpio(sensor->strobe_gpio);
		int irq = gpiod_to_irq(sensor->strobe_gpio);
		int err;
		u32 core_affinity;

		if((err = request_irq      (irq, gpio_handler, IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "og01a1b", sensor)) != 0)
		{
			dev_err(&client->dev, "Error %d: could not request irq: %d\n", err, gpio);
			return err;
		}

		if (!of_property_read_u32( dev->of_node, "irq-core-affinity", &core_affinity)) {
			// Only set the core affinity if it is not 0. This is because 0 is the default and GPIOs that can only be zero will fail in the call to irq_set_affinity
			if(core_affinity != 0) {
				int ret = irq_set_affinity(irq,cpumask_of(core_affinity));
				if (ret < 0)
				{
					dev_err(&client->dev, "Failed to set core affinity for irq:%d ret:%d core_affinity:%d",irq, ret, core_affinity);
				}
			}
		}
		num++;
	}

	sensor->current_mode = &og01a1b_mode_data_1280x1024;

	return 0;

free_ctrls:
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static void og01a1b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
}

static const struct i2c_device_id og01a1b_id[] = {
	{"og01a1b", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, og01a1b_id);

static const struct of_device_id og01a1b_dt_ids[] = {
	{ .compatible = "ovti,og01a1bx" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, og01a1b_dt_ids);

static struct i2c_driver og01a1b_i2c_driver = {
	.driver = {
		.name  = "og01a1b",
		.of_match_table	= og01a1b_dt_ids,
	},
	.id_table = og01a1b_id,
	.probe = og01a1b_probe,
	.remove   = og01a1b_remove,
};

module_i2c_driver(og01a1b_i2c_driver);

MODULE_DESCRIPTION("og01a1b MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
