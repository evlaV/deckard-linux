// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 *
 * Based on OV9282 driver and some register settings from intel OG01A1B driver. Adapted by arcturus.industries
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
#include <linux/pm_runtime.h>
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

// #define V4L2_CID_TEMPERATURE	(V4L2_CID_USER_OG01A1B_BASE + 0)
// #define V4L2_CID_READ_OG01A1B   (V4L2_CID_USER_OG01A1B_BASE + 1)

#define OG01A1B_REG_CHIP_ID		0x300a

#define OG01A1B_LINK_FREQUENCY 500000000

#define OG01A1B_REG_TPM_TEMPERATURE		0x4d17

#define OG01A1B_REG_TIMING_DVPHO		0x3808
#define OG01A1B_REG_TIMING_DVPVO		0x380a
#define OG01A1B_REG_TIMING_HTS		0x380c
#define OG01A1B_REG_TIMING_VTS		0x380e
#define NUM_REGISTER_GROUPS			3
#define GROUP_REGMAP_MAX_SIZE		20
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

#define VIDIOC_OG01A1B_PRV_G_DRIVER_VERSION _IOR('V', BASE_VIDIOC_PRIVATE + 6, u32)

#define VIDIOC_SENSOR_PRV_S_TIMING_ADJUSTMENT _IOW('V', BASE_VIDIOC_PRIVATE + 7, struct private_timing_adjustment)

#define VIDIOC_SENSOR_PRV_G_REGISTER _IOWR('V', BASE_VIDIOC_PRIVATE + 8, struct sensor_register)

#define VIDIOC_SENSOR_PRV_S_IN_SYNC _IO('V', BASE_VIDIOC_PRIVATE + 9)

// Version 1.2 support histogram streaming.

#define METADATA_MAJOR_VERSION 3
#define METADATA_MINOR_VERSION 1

#define DRIVER_MAJOR_VERSION 1
#define DRIVER_MINOR_VERSION 1


/* regulator supplies */
static const char * const og01a1b_supply_name[] = {
	"DOVDD", /* Digital I/O (1.8V) supply */
	"AVDD",  /* Analog (2.8V) supply */
	"DVDD",  /* Digital Core (1.5V) supply */
};

#define og01a1b_NUM_SUPPLIES ARRAY_SIZE(og01a1b_supply_name)

struct sensor_register {
	u16 reg_addr;
	u32 reg_value;
	u8 len;
};
struct sensor_private_group_reg {
	u16 reg_addr;
	u8 reg_value;
	u8 group;
};

struct private_timing_adjustment {
	__u16 new_rows;
	__u16 rows_to_restore;
	__u16 new_global_row;
	__u16 global_row_to_restore;
	__u8 group_to_adjust;
	__u8 number_of_frames_to_hold_adjustment; // This depends on the group setup
};

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
	MODE_2GROUPS =  2,
	MODE_3GROUPS =  3
};

enum og01a1b_ir_emitter_mode {
	IR_EMITTER_MODE_OFF = 0,
	IR_EMITTER_MODE_GROUP_1_ONLY = 1,
	IR_EMITTER_MODE_GROUP_0_AND_2_ONLY =  2,
	IR_EMITTER_MODE_ALL_FRAMES
};

enum og01a1b_timing_adjustment_state {
	TIMING_ADJUSTMENT_NOT_IN_PROGRESS = 0,
	TIMING_READY_FOR_GROUP_RESTORE = 1,
	TIMING_ADJUSTMENT_IN_PROGRESS = 2,
	TIMING_READY_FOR_INITIAL_GROUP_WRITE = 3,
	TIMING_ADJUSTMENT_REQUESTED = 4
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
	struct gpio_desc *ir_control_gpio;

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
	int last_vc;

	struct group_and_registers active_group_registers;
	bool updateGroupRegisters[NUM_REGISTER_GROUPS];
	enum og01a1b_ir_emitter_mode ir_emitter_mode;
	bool emittersPresent;
	bool ir_control_gpio_present;
	enum og01a1b_group_mode group_mode;
	int sensor_index;

	const struct og01a1b_mode_info* current_mode;
	bool is_streaming;
	bool is_left_sensor;
	bool dual_i2c_available;
	bool dual_i2c_main;
	struct private_timing_adjustment current_timing_adjustment;
	enum og01a1b_timing_adjustment_state timing_adjustment_state;
	int queue_index;

	bool otp_valid;
	u8 otp_data[256];
	bool skip_regmap;
};

struct og01a1b_dev* sensors[4];
int numSensors = 0;
int sync_sensor_index = -1;

/* Global i2c_client for dual writes */
static struct i2c_client *g_dual_i2c_client = NULL;

// This is used to determine whether or not we can use dual i2c mode. If this is non zero and more than 30 ms ago we can
// Because that means we do not have a danger of missing an update to one sensor
u64 last_time_in_sync = 0;
u64 last_dual_write_time = 0;



// These workqueues are shared between multiple sensors that are either og01a1b or og0ve10
// They should be assigned the same workqueue if they share an i2c bus
struct workqueue_struct *sensor_workqueues[2] = {NULL, NULL};
EXPORT_SYMBOL(sensor_workqueues);

static inline struct og01a1b_dev *to_og01a1b_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct og01a1b_dev, sd);
}

static const struct reg_value og01a1b_setting[] = {

	{ 0x0103, 0x01 },
	{ 0x0100, 0x00 },
	{ 0x0300, 0x0a },
	{ 0x0301, 0x29 },
	{ 0x0302, 0x31 },
	{ 0x0303, 0x04 },
	{ 0x0304, 0x00 },
	{ 0x0305, 0xfa },
	{ 0x0306, 0x00 },
	{ 0x0307, 0x01 },
	{ 0x0308, 0x02 },
	{ 0x0309, 0x00 },
	{ 0x0310, 0x00 },
	{ 0x0311, 0x00 },
	{ 0x0312, 0x07 },
	{ 0x0313, 0x00 },
	{ 0x0314, 0x00 },
	{ 0x0315, 0x00 },
	{ 0x0320, 0x02 },
	{ 0x0321, 0x01 },
	{ 0x0322, 0x01 },
	{ 0x0323, 0x04 },
	{ 0x0324, 0x01 },
	{ 0x0325, 0xc2 },
	{ 0x0326, 0xce },
	{ 0x0327, 0x04 },
	{ 0x0329, 0x02 },
	{ 0x032a, 0x04 },
	{ 0x032b, 0x04 },
	{ 0x032c, 0x02 },
	{ 0x032d, 0x01 },
	{ 0x032e, 0x00 },
	{ 0x300d, 0x02 },
	{ 0x300e, 0x04 },
	{ 0x3021, 0x08 },
	{ 0x301e, 0x03 },
	{ 0x3103, 0x00 },
	{ 0x3106, 0x08 },
	{ 0x3107, 0x40 },
	{ 0x3216, 0x01 },
	{ 0x3217, 0x00 },
	{ 0x3218, 0xc0 },
	{ 0x3219, 0x55 },
	{ 0x3500, 0x00 },
	{ 0x3501, 0x04 },
	{ 0x3502, 0x8a },
	{ 0x3506, 0x01 },
	{ 0x3507, 0x72 },
	{ 0x3508, 0x01 },
	{ 0x3509, 0x00 },
	{ 0x350a, 0x01 },
	{ 0x350b, 0x00 },
	{ 0x350c, 0x00 },
	{ 0x3541, 0x00 },
	{ 0x3542, 0x40 },
	{ 0x3605, 0xe0 },
	{ 0x3606, 0x41 },
	{ 0x3614, 0x20 },
	{ 0x3620, 0x0b },
	{ 0x3630, 0x07 },
	{ 0x3636, 0xa0 },
	{ 0x3637, 0xf9 },
	{ 0x3638, 0x09 },
	{ 0x3639, 0x38 },
	{ 0x363f, 0x09 },
	{ 0x3640, 0x17 },
	{ 0x3662, 0x06 }, // enables 8 bit mode
	{ 0x3665, 0x80 },
	{ 0x3670, 0x68 },
	{ 0x3674, 0x00 },
	{ 0x3677, 0x3f },
	{ 0x3679, 0x00 },
	{ 0x369f, 0x19 },
	{ 0x36a0, 0x03 },
	{ 0x36a2, 0x19 },
	{ 0x36a3, 0x03 },
	{ 0x370d, 0x66 },
	{ 0x370f, 0x00 },
	{ 0x3710, 0x03 },
	{ 0x3715, 0x03 },
	{ 0x3716, 0x03 },
	{ 0x3717, 0x06 },
	{ 0x3733, 0x00 },
	{ 0x3778, 0x00 },
	{ 0x37a8, 0x0f },
	{ 0x37a9, 0x01 },
	{ 0x37aa, 0x07 },
	{ 0x37bd, 0x1c },
	{ 0x37c1, 0x2f },
	{ 0x37c3, 0x09 },
	{ 0x37c8, 0x1d },
	{ 0x37ca, 0x30 },
	{ 0x37df, 0x00 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x00 },
	{ 0x3804, 0x05 },
	{ 0x3805, 0x0f },
	{ 0x3806, 0x04 },
	{ 0x3807, 0x0f },
	{ 0x3808, 0x05 },
	{ 0x3809, 0x00 },
	{ 0x380a, 0x04 },
	{ 0x380b, 0x00 },
	{ 0x380c, 0x03 },
	{ 0x380d, 0x50 },
	{ 0x380e, 0x04 },
	{ 0x380f, 0x98 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x08 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x08 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3820, 0x40 },
	{ 0x3821, 0x04 },
	{ 0x3826, 0x00 },
	{ 0x3827, 0x00 },
	{ 0x382a, 0x08 },
	{ 0x382b, 0x52 },
	{ 0x382d, 0xba },
	{ 0x383d, 0x14 },
	{ 0x384a, 0xa2 },
	{ 0x3866, 0x0e },
	{ 0x3867, 0x07 },
	{ 0x3884, 0x00 },
	{ 0x3885, 0x08 },
	{ 0x3893, 0x68 },
	{ 0x3894, 0x2a },
	{ 0x3898, 0x00 },
	{ 0x3899, 0x31 },
	{ 0x389a, 0x04 },
	{ 0x389b, 0x00 },
	{ 0x389c, 0x0b },
	{ 0x389d, 0xad },
	{ 0x389f, 0x08 },
	{ 0x38a0, 0x00 },
	{ 0x38a1, 0x00 },
	{ 0x38a8, 0x70 },
	{ 0x38ac, 0xea },
	{ 0x38b2, 0x00 },
	{ 0x38b3, 0x08 },
	{ 0x38bc, 0x20 },
	{ 0x38c4, 0x0c },
	{ 0x38c5, 0x3a },
	{ 0x38c7, 0x3a },
	{ 0x38e1, 0xc0 },
	{ 0x38ec, 0x3c },
	{ 0x38f0, 0x09 },
	{ 0x38f1, 0x6f },
	{ 0x38fe, 0x3c },
	{ 0x391e, 0x00 },
	{ 0x391f, 0x00 },
	{ 0x3920, 0xa5 },
	{ 0x3921, 0x00 },
	{ 0x3922, 0x00 },
	{ 0x3923, 0x00 },
	{ 0x3924, 0x05 },
	{ 0x3925, 0x00 },
	{ 0x3926, 0x00 },
	{ 0x3927, 0x00 },
	{ 0x3928, 0x1a },
	{ 0x3929, 0x01 },
	{ 0x392a, 0xb4 },
	{ 0x392b, 0x00 },
	{ 0x392c, 0x10 },
	{ 0x392f, 0x40 },
	{ 0x4000, 0xcf },
	{ 0x4003, 0x40 },
	{ 0x4008, 0x00 },
	{ 0x4009, 0x07 },
	{ 0x400a, 0x02 },
	{ 0x400b, 0x54 },
	{ 0x400c, 0x00 },
	{ 0x400d, 0x07 },
	{ 0x4010, 0xc0 },
	{ 0x4012, 0x02 },
	{ 0x4014, 0x04 },
	{ 0x4015, 0x04 },
	{ 0x4017, 0x02 },
	{ 0x4042, 0x01 },
	{ 0x4306, 0x04 },
	{ 0x4307, 0x12 },
	{ 0x4509, 0x00 },
	{ 0x450b, 0x83 },
	{ 0x4604, 0x68 },
	{ 0x4608, 0x0a },
	{ 0x4700, 0x06 },
	{ 0x4800, 0x64 },
	{ 0x481b, 0x3c },
	{ 0x4825, 0x32 },
	{ 0x4833, 0x18 },
	{ 0x4837, 0x10 },
	{ 0x4850, 0x40 },
	{ 0x4860, 0x00 },
	{ 0x4861, 0xec },
	{ 0x4864, 0x00 },
	{ 0x4883, 0x00 },
	{ 0x4888, 0x90 },
	{ 0x4889, 0x05 },
	{ 0x488b, 0x04 },
	{ 0x4f00, 0x04 },
	{ 0x4f10, 0x04 },
	{ 0x4f21, 0x01 },
	{ 0x4f22, 0x40 },
	{ 0x4f23, 0x44 },
	{ 0x4f24, 0x51 },
	{ 0x4f25, 0x41 },
	{ 0x5000, 0x1f },
	{ 0x500a, 0x00 },
	{ 0x5100, 0x00 },
	{ 0x5111, 0x20 },

	//enable strobe for testing
	{ 0x3009, 0b1000},
	{ 0x3017, 0xf2 },
	{ 0x3920, 0xff },
	{ 0x3924, 0x00 },

	//for temperature sensor
	{ 0x3667, 0x80},
	{ 0x4d00, 0x4a},
	{ 0x4d01, 0x18},
	{ 0x4d02, 0xbb},
	{ 0x4d03, 0xde},
	{ 0x4d04, 0x93},
	{ 0x4d05, 0xff},
	{ 0x4d09, 0x0a},

};

static const struct reg_value og01a1b_setting_640x512[] = {

	{ 0x0103, 0x01 },
	{ 0x0100, 0x00 },
	{ 0x0300, 0x0a },
	{ 0x0301, 0x29 },
	{ 0x0302, 0x31 },
	{ 0x0303, 0x04 },
	{ 0x0304, 0x00 },
	{ 0x0305, 0xfa },
	{ 0x0306, 0x00 },
	{ 0x0307, 0x01 },
	{ 0x0308, 0x02 },
	{ 0x0309, 0x00 },
	{ 0x0310, 0x00 },
	{ 0x0311, 0x00 },
	{ 0x0312, 0x07 },
	{ 0x0313, 0x00 },
	{ 0x0314, 0x00 },
	{ 0x0315, 0x00 },
	{ 0x0320, 0x02 },
	{ 0x0321, 0x01 },
	{ 0x0322, 0x01 },
	{ 0x0323, 0x04 },
	{ 0x0324, 0x01 },
	{ 0x0325, 0xc2 },
	{ 0x0326, 0xce },
	{ 0x0327, 0x04 },
	{ 0x0329, 0x02 },
	{ 0x032a, 0x04 },
	{ 0x032b, 0x04 },
	{ 0x032c, 0x02 },
	{ 0x032d, 0x01 },
	{ 0x032e, 0x00 },
	{ 0x300d, 0x02 },
	{ 0x300e, 0x04 },
	{ 0x3021, 0x08 },
	{ 0x301e, 0x03 },
	{ 0x3103, 0x00 },
	{ 0x3106, 0x08 },
	{ 0x3107, 0x40 },
	{ 0x3216, 0x01 },
	{ 0x3217, 0x00 },
	{ 0x3218, 0xc0 },
	{ 0x3219, 0x55 },
	{ 0x3500, 0x00 },
	{ 0x3501, 0x04 },
	{ 0x3502, 0x8a },
	{ 0x3506, 0x01 },
	{ 0x3507, 0x72 },
	{ 0x3508, 0x01 },
	{ 0x3509, 0x00 },
	{ 0x350a, 0x01 },
	{ 0x350b, 0x00 },
	{ 0x350c, 0x00 },
	{ 0x3541, 0x00 },
	{ 0x3542, 0x40 },
	{ 0x3605, 0xe0 },
	{ 0x3606, 0x41 },
	{ 0x3614, 0x20 },
	{ 0x3620, 0x0b },
	{ 0x3630, 0x07 },
	{ 0x3636, 0xa0 },
	{ 0x3637, 0xf9 },
	{ 0x3638, 0x09 },
	{ 0x3639, 0x38 },
	{ 0x363f, 0x09 },
	{ 0x3640, 0x17 },
	{ 0x3662, 0x06 }, // enables 8 bit mode
	{ 0x3665, 0x80 },
	{ 0x3670, 0x68 },
	{ 0x3674, 0x00 },
	{ 0x3677, 0x3f },
	{ 0x3679, 0x00 },
	{ 0x369f, 0x19 },
	{ 0x36a0, 0x03 },
	{ 0x36a2, 0x19 },
	{ 0x36a3, 0x03 },
	{ 0x370d, 0x66 },
	{ 0x370f, 0x00 },
	{ 0x3710, 0x03 },
	{ 0x3715, 0x03 },
	{ 0x3716, 0x03 },
	{ 0x3717, 0x06 },
	{ 0x3733, 0x00 },
	{ 0x3778, 0x00 },
	{ 0x37a8, 0x0f },
	{ 0x37a9, 0x01 },
	{ 0x37aa, 0x07 },
	{ 0x37bd, 0x1c },
	{ 0x37c1, 0x2f },
	{ 0x37c3, 0x09 },
	{ 0x37c8, 0x1d },
	{ 0x37ca, 0x30 },
	{ 0x37df, 0x00 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x00 },
	{ 0x3804, 0x05 },
	{ 0x3805, 0x0f },
	{ 0x3806, 0x04 },
	{ 0x3807, 0x0f },
	{ 0x3808, 0x05 },
	{ 0x3809, 0x00 },
	{ 0x380a, 0x04 },
	{ 0x380b, 0x00 },
	{ 0x380c, 0x03 },
	{ 0x380d, 0x50 },
	{ 0x380e, 0x04 },
	{ 0x380f, 0x98 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x08 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x08 },
	{ 0x3814, 0x11 },
	{ 0x3815, 0x11 },
	{ 0x3820, 0x40 },
	{ 0x3821, 0x04 },
	{ 0x3826, 0x00 },
	{ 0x3827, 0x00 },
	{ 0x382a, 0x08 },
	{ 0x382b, 0x52 },
	{ 0x382d, 0xba },
	{ 0x383d, 0x14 },
	{ 0x384a, 0xa2 },
	{ 0x3866, 0x0e },
	{ 0x3867, 0x07 },
	{ 0x3884, 0x00 },
	{ 0x3885, 0x08 },
	{ 0x3893, 0x68 },
	{ 0x3894, 0x2a },
	{ 0x3898, 0x00 },
	{ 0x3899, 0x31 },
	{ 0x389a, 0x04 },
	{ 0x389b, 0x00 },
	{ 0x389c, 0x0b },
	{ 0x389d, 0xad },
	{ 0x389f, 0x08 },
	{ 0x38a0, 0x00 },
	{ 0x38a1, 0x00 },
	{ 0x38a8, 0x70 },
	{ 0x38ac, 0xea },
	{ 0x38b2, 0x00 },
	{ 0x38b3, 0x08 },
	{ 0x38bc, 0x20 },
	{ 0x38c4, 0x0c },
	{ 0x38c5, 0x3a },
	{ 0x38c7, 0x3a },
	{ 0x38e1, 0xc0 },
	{ 0x38ec, 0x3c },
	{ 0x38f0, 0x09 },
	{ 0x38f1, 0x6f },
	{ 0x38fe, 0x3c },
	{ 0x391e, 0x00 },
	{ 0x391f, 0x00 },
	{ 0x3920, 0xa5 },
	{ 0x3921, 0x00 },
	{ 0x3922, 0x00 },
	{ 0x3923, 0x00 },
	{ 0x3924, 0x05 },
	{ 0x3925, 0x00 },
	{ 0x3926, 0x00 },
	{ 0x3927, 0x00 },
	{ 0x3928, 0x1a },
	{ 0x3929, 0x01 },
	{ 0x392a, 0xb4 },
	{ 0x392b, 0x00 },
	{ 0x392c, 0x10 },
	{ 0x392f, 0x40 },
	{ 0x4000, 0xcf },
	{ 0x4003, 0x40 },
	{ 0x4008, 0x00 },
	{ 0x4009, 0x07 },
	{ 0x400a, 0x02 },
	{ 0x400b, 0x54 },
	{ 0x400c, 0x00 },
	{ 0x400d, 0x07 },
	{ 0x4010, 0xc0 },
	{ 0x4012, 0x02 },
	{ 0x4014, 0x04 },
	{ 0x4015, 0x04 },
	{ 0x4017, 0x02 },
	{ 0x4042, 0x01 },
	{ 0x4306, 0x04 },
	{ 0x4307, 0x12 },
	{ 0x4509, 0x00 },
	{ 0x450b, 0x83 },
	{ 0x4604, 0x68 },
	{ 0x4608, 0x0a },
	{ 0x4700, 0x06 },
	{ 0x4800, 0x64 },
	{ 0x481b, 0x3c },
	{ 0x4825, 0x32 },
	{ 0x4833, 0x18 },
	{ 0x4837, 0x10 },
	{ 0x4850, 0x40 },
	{ 0x4860, 0x00 },
	{ 0x4861, 0xec },
	{ 0x4864, 0x00 },
	{ 0x4883, 0x00 },
	{ 0x4888, 0x90 },
	{ 0x4889, 0x05 },
	{ 0x488b, 0x04 },
	{ 0x4f00, 0x04 },
	{ 0x4f10, 0x04 },
	{ 0x4f21, 0x01 },
	{ 0x4f22, 0x40 },
	{ 0x4f23, 0x44 },
	{ 0x4f24, 0x51 },
	{ 0x4f25, 0x41 },
	{ 0x5000, 0x1f },
	{ 0x500a, 0x00 },
	{ 0x5100, 0x00 },
	{ 0x5111, 0x20 },

	//enable strobe for testing
	{ 0x3009, 0b1000},
	{ 0x3017, 0xf2 },
	{ 0x3920, 0xff },
	{ 0x3924, 0x00 },

	//for temperature sensor
	{ 0x3667, 0x80},
	{ 0x4d00, 0x4a},
	{ 0x4d01, 0x18},
	{ 0x4d02, 0xbb},
	{ 0x4d03, 0xde},
	{ 0x4d04, 0x93},
	{ 0x4d05, 0xff},
	{ 0x4d09, 0x0a},

	{ 0x3501, 0x02 },
	{ 0x3502, 0x73 },
	{ 0x3778, 0x00 },
	{ 0x3803, 0x00 },
	{ 0x3806, 0x04 },
	{ 0x3807, 0x0f },
	{ 0x3808, 0x02 },
	{ 0x3809, 0x80 },
	{ 0x380a, 0x02 },
	{ 0x380b, 0x00 },
	{ 0x380c, 0x03 },
	{ 0x380d, 0x0c },
	{ 0x380e, 0x02 },
	{ 0x380f, 0x81 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x04 },
	{ 0x3812, 0x00 },
	{ 0x3813, 0x04 },
	{ 0x3814, 0x22 },
	{ 0x3815, 0x22 },
	{ 0x3820, 0x40 },
	{ 0x3821, 0x04 },
	{ 0x4509, 0x80 },

};

static const struct og01a1b_mode_info og01a1b_mode_data_1280x1024 = {
	1280, 848, 1024, 1176, og01a1b_setting, ARRAY_SIZE(og01a1b_setting)
};

static const struct og01a1b_mode_info og01a1b_mode_data_640x512 = {
	640, 848, 512, 1176, og01a1b_setting_640x512, ARRAY_SIZE(og01a1b_setting_640x512)
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

static int og01a1b_write_reg_dual(struct og01a1b_dev *sensor, u16 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	if (!g_dual_i2c_client) {
		dev_err(&sensor->i2c_client->dev, "%s: dual i2c client not initialized\n", __func__);
		return -ENODEV;
	}

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msg.addr = g_dual_i2c_client->addr;
	msg.flags = g_dual_i2c_client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(g_dual_i2c_client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&sensor->i2c_client->dev, "%s: error: reg=%x, val=%x ret=%d\n",
			__func__, reg, val, ret);
		return ret;
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

static int og01a1b_read_reg_len(struct og01a1b_dev *sensor, u16 reg, u16 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);
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

static int og01a1b_write_reg16(struct og01a1b_dev *sensor, u16 reg, u16 val)
{
	// Use the og01a1b sequential write feature to use a single i2c transfer
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val >> 8; // written at reg
	buf[3] = val & 0xff; // written at reg+1

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

static int og01a1b_write_reg16_dual(struct og01a1b_dev *sensor, u16 reg, u16 val)
{
	// Use the og01a1b sequential write feature to use a single i2c transfer
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	if (!g_dual_i2c_client) {
		dev_err(&sensor->i2c_client->dev, "%s: dual i2c client not initialized\n", __func__);
		return -ENODEV;
	}

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val >> 8; // written at reg
	buf[3] = val & 0xff; // written at reg+1

	msg.addr = g_dual_i2c_client->addr;
	msg.flags = g_dual_i2c_client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(g_dual_i2c_client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&sensor->i2c_client->dev, "%s: error: reg=%x, val=%x ret=%d\n",
			__func__, reg, val, ret);
		return ret;
	}
	return 0;
}



static int og01a1b_add_or_set_group_reg(struct og01a1b_dev *sensor, u16 address, u8 val, u8 group)
{

	

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
		sensor->updateGroupRegisters[group] = true;
	}
	if(reg_set->group_reg_values[index] != val)
	{
		reg_set->group_reg_values[index] = val;
		sensor->updateGroupRegisters[group] = true;
	}
	return 0;
}

static int og01a1b_add_or_set_group_reg_from_ctrlval(struct og01a1b_dev *sensor, int ctrlval)
{
	u16 address = ctrlval & 0xffff;
	u8 val = ctrlval >> 16 & 0xff;
	u8 group = ctrlval >> 24 & 0xf;

	return og01a1b_add_or_set_group_reg(sensor, address, val, group);
}

static bool emitter_enabled_for_group(struct og01a1b_dev *sensor, int group) {
	//printk("sensor->ir_emitter_mode:%d group:%d",sensor->ir_emitter_mode,group);
	if(sensor->ir_emitter_mode == IR_EMITTER_MODE_ALL_FRAMES) {
		return true;
	}
	else if(sensor->ir_emitter_mode == IR_EMITTER_MODE_GROUP_1_ONLY) {
		return group == 1;
	}
	else if(sensor->ir_emitter_mode == IR_EMITTER_MODE_GROUP_0_AND_2_ONLY) {
		return (group == 0) || (group == 2);
	}
	return false;
}

static int og01a1b_write_group_registers_if_needed(struct og01a1b_dev *sensor, bool use_dual_i2c)
{
	int ret;

	for(int group = 0; group < NUM_REGISTER_GROUPS; group++) 
	{	
		bool doingTimingAdjustment = ((sensor->timing_adjustment_state == TIMING_READY_FOR_INITIAL_GROUP_WRITE) || (sensor->timing_adjustment_state == TIMING_READY_FOR_GROUP_RESTORE)) && group == sensor->current_timing_adjustment.group_to_adjust;
		//printk("doing timing adjustment:%d group:%d sensor->current_timing_adjustment.group_to_adjust:%d\n",doingTimingAdjustment,group,sensor->current_timing_adjustment.group_to_adjust);
		if(!sensor->updateGroupRegisters[group] && !doingTimingAdjustment)
		{
			continue;
		}
		sensor->updateGroupRegisters[group] = false;
		ret = use_dual_i2c ? og01a1b_write_reg_dual(sensor, 0x3208, 0x00 | group) : og01a1b_write_reg(sensor, 0x3208, 0x00 | group);
		if(ret != 0) {
			return ret;
		}
		for(int i = 0; i < sensor->active_group_registers.group_register_sets[group].num_group_reg; i++) 
		{
			if (i < sensor->active_group_registers.group_register_sets[group].num_group_reg - 1 &&
				sensor->active_group_registers.group_register_sets[group].group_reg_addresses[i + 1] == 
				sensor->active_group_registers.group_register_sets[group].group_reg_addresses[i] + 1) {
				// Sequential registers, use og01a1b_write_reg16
				u16 val = (sensor->active_group_registers.group_register_sets[group].group_reg_values[i] << 8) |
						   sensor->active_group_registers.group_register_sets[group].group_reg_values[i + 1];
				ret = use_dual_i2c ? og01a1b_write_reg16_dual(sensor, 
					sensor->active_group_registers.group_register_sets[group].group_reg_addresses[i], val) :
					og01a1b_write_reg16(sensor, 
					sensor->active_group_registers.group_register_sets[group].group_reg_addresses[i], val);
				i++; // Skip the next register as we've already written it
			} else {
				// Non-sequential registers, use og01a1b_write_reg
				ret = use_dual_i2c ? og01a1b_write_reg_dual(sensor, 
					sensor->active_group_registers.group_register_sets[group].group_reg_addresses[i],
					sensor->active_group_registers.group_register_sets[group].group_reg_values[i]) :
					og01a1b_write_reg(sensor, 
					sensor->active_group_registers.group_register_sets[group].group_reg_addresses[i],
					sensor->active_group_registers.group_register_sets[group].group_reg_values[i]);
			}
			if(ret != 0) 
			{
				return ret;
			}
		}
		if(emitter_enabled_for_group(sensor, group))
		{
			ret = use_dual_i2c ? og01a1b_write_reg_dual(sensor, 0x3920, 0xff) : og01a1b_write_reg(sensor, 0x3920, 0xff);
			if(ret != 0) 
			{
				return ret;
			}
		}
		else{
			ret = use_dual_i2c ? og01a1b_write_reg_dual(sensor, 0x3920, 0x00) : og01a1b_write_reg(sensor, 0x3920, 0x00);
			if(ret != 0) 
			{
				return ret;
			}
		}
		if(sensor->timing_adjustment_state == TIMING_READY_FOR_INITIAL_GROUP_WRITE && sensor->current_timing_adjustment.group_to_adjust == group)
		{
			ret = use_dual_i2c ? og01a1b_write_reg16_dual(sensor, 0x380e, sensor->current_timing_adjustment.new_rows) :
				og01a1b_write_reg16(sensor, 0x380e, sensor->current_timing_adjustment.new_rows);
			if(ret != 0) 
			{
				return ret;
			}
			ret = use_dual_i2c ? og01a1b_write_reg16_dual(sensor, 0x382c, sensor->current_timing_adjustment.new_global_row) :
				og01a1b_write_reg16(sensor, 0x382c, sensor->current_timing_adjustment.new_global_row);
			if(ret != 0) 
			{
				return ret;
			}
			sensor->timing_adjustment_state = TIMING_ADJUSTMENT_IN_PROGRESS;
		}
		// WHAT happens is we update group registers during a timing adjustment? 
		if(sensor->timing_adjustment_state == TIMING_READY_FOR_GROUP_RESTORE && sensor->current_timing_adjustment.group_to_adjust == group) 
		{
			// TODO: add register writes to be more explicit, the restored rows should be part of the group regs though
			// ( we could also get rid of the restore )
			sensor->timing_adjustment_state = TIMING_ADJUSTMENT_NOT_IN_PROGRESS;
		}
		ret = use_dual_i2c ? og01a1b_write_reg_dual(sensor, 0x3208, 0x10 | group) : og01a1b_write_reg(sensor, 0x3208, 0x10 | group);
		if(ret!= 0)
		{
			return ret;
		}
	}
	if(sensor->current_timing_adjustment.number_of_frames_to_hold_adjustment > 0)
	{
		sensor->current_timing_adjustment.number_of_frames_to_hold_adjustment--;
	}
	return 0;
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

	return 0;
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


	return 0;
}

static void og01a1b_reset(struct og01a1b_dev *sensor)
{
	if (!sensor->reset_gpio)
		return;

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 5100);
}

static int og01a1b_set_power_on(struct og01a1b_dev *sensor)
{
	
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable power\n",
			__func__);
		return ret;
	}
	
	// hard code clock rate to 24 mhz
	clk_set_rate(sensor->xclk, 24000000);

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto power_off;
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
power_off:
	pm_runtime_put_sync(&client->dev);
	return ret;
}

static void og01a1b_set_power_off(struct og01a1b_dev *sensor)
{
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	regulator_bulk_disable(og01a1b_NUM_SUPPLIES, sensor->supplies);
	clk_disable_unprepare(sensor->xclk);
	pm_runtime_put_sync(&sensor->i2c_client->dev);
}

static int og01a1b_read_otp_once(struct og01a1b_dev *sensor, u8 *otp_data);

/* Read OTP into cache if not already valid. Non-fatal on failure. */
static void og01a1b_fill_otp_cache_locked(struct og01a1b_dev *sensor)
{
	int ret;

	if (sensor->otp_valid)
		return;

	ret = og01a1b_write_reg(sensor, 0x100, 0x01);
	if (ret) {
		dev_err(&sensor->i2c_client->dev,
			"OTP: failed to start sensor: %d\n", ret);
		return;
	}
	usleep_range(5000, 5100);

	ret = og01a1b_read_otp_once(sensor, sensor->otp_data);
	if (ret) {
		dev_err(&sensor->i2c_client->dev,
			"OTP: read failed: %d\n", ret);
		return;
	}

	sensor->otp_valid = true;
}

static int og01a1b_set_power(struct og01a1b_dev *sensor, bool on)
{
	int ret = 0;

	if (on) {
		ret = og01a1b_set_power_on(sensor);
		if (ret)
			return ret;

		/* Fill OTP cache before restore_mode() touches the register map */
		og01a1b_fill_otp_cache_locked(sensor);

		if (!sensor->skip_regmap) {
			dev_dbg(&sensor->i2c_client->dev, "Applying register map from kernel\n");
			ret = og01a1b_restore_mode(sensor);
			if (ret)
				goto power_off;
		} else {
			dev_info(&sensor->i2c_client->dev, "skip_regmap set, skipping kernel register map\n");
		}

	} else {
		og01a1b_set_power_off(sensor);
	}

	return 0;

power_off:
	og01a1b_set_power_off(sensor);
	return ret;
}

static int og01a1b_read_otp_once(struct og01a1b_dev *sensor, u8 *otp_data)
{
	int ret;
	
	/* Clear OTP buffer registers 0x7000-0x70FF */
	for (int i = 0x7000; i <= 0x70FF; i++) {
		ret = og01a1b_write_reg(sensor, i, 0x00);
		if (ret)
			return ret;
	}

	/* Configure PLL and set register 0x100 to 1 if not yet set */
	ret = og01a1b_write_reg(sensor, 0x100, 0x01);
	if (ret)
		return ret;

	/* Set register 0x3DB1 to 0x01 to start OTP read */
	ret = og01a1b_write_reg(sensor, 0x3d81, 0x01);
	if (ret)
		return ret;

	/* Wait 15 ms for OTP read to complete */
	usleep_range(15000, 15100);

	/* Read OTP data from 0x7000-0x70FF (256 bytes) */
	for (int k = 0; k < 256; k++) {
		ret = og01a1b_read_reg(sensor, 0x7000 + k, &otp_data[k]);
		if (ret)
			return ret;
	}

	return 0;
}


static long og01a1b_private_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg);

static u16 og01a1b_read_reg_addr;
static int og01a1b_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct og01a1b_dev *sensor =
			container_of(ctrl->handler, struct og01a1b_dev, ctrl_handler);
	int ret = 0;
    u16 addr;

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(&sensor->i2c_client->dev))
		return 0;

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
		addr = ctrl->val & 0xffff;
		if (addr == 0x0000)
		{
			og01a1b_read_reg_addr = ctrl->val >> 16 & 0xff;
		}
		else if (addr == 0x0001)
		{
			og01a1b_read_reg_addr = og01a1b_read_reg_addr | ( ctrl->val >> 16 & 0xff ) << 8;
		}
		else
		{
			ret = og01a1b_private_ioctl(&sensor->sd, VIDIOC_OG01A1B_PRV_S_REG, (void*)(uintptr_t)ctrl->val);
		}
		break;
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

	pm_runtime_put(&sensor->i2c_client->dev);

	return ret;
}

static int og01a1b_get_ctrl(struct v4l2_ctrl *ctrl)
{
	u16 temperature;
	struct og01a1b_dev *sensor =
			container_of(ctrl->handler, struct og01a1b_dev, ctrl_handler);

	og01a1b_read_reg_len(sensor, OG01A1B_REG_TPM_TEMPERATURE, 2, (u32 *)&temperature);
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

// static const struct v4l2_ctrl_config og01a1b_ctrl_temperature = {
// 	.ops = &og01a1b_ctrl_read_ops,
// 	.id = V4L2_CID_TEMPERATURE,
// 	.name = "Temperature",
// 	.type = V4L2_CTRL_TYPE_INTEGER,
// 	.flags = V4L2_CTRL_FLAG_VOLATILE,
// 	.min = 0,
// 	.max = 0xffffffff,
// 	.step = 1,
// };

static int og01a1b_get_ctrl_read(struct v4l2_ctrl *ctrl)
{
	u8 val;
	int ret;

	struct og01a1b_dev *sensor =
			container_of(ctrl->handler, struct og01a1b_dev, ctrl_handler);

	if(og01a1b_read_reg_addr == 0)
	{
		og01a1b_read_reg_addr = 0x3500;
	}
	ret = og01a1b_read_reg(sensor, og01a1b_read_reg_addr, &val);
	ctrl->val = val;
	ctrl->has_changed = 1;
	ctrl->cur.val = val;

	return 0;
}

static const struct v4l2_ctrl_ops og01a1b_ctrl_read_ops_custom = {
	.g_volatile_ctrl = og01a1b_get_ctrl_read,
};

// static const struct v4l2_ctrl_config og01a1b_ctrl_read = {
// 	.ops = &og01a1b_ctrl_read_ops_custom,
// 	.id = V4L2_CID_READ_OG01A1B,
// 	.name = "ReadReg",
// 	.type = V4L2_CTRL_TYPE_INTEGER,
// 	.flags = V4L2_CTRL_FLAG_VOLATILE,
// 	.min = 0,
// 	.max = 0xffffffff,
// 	.step = 1,
// };

static void* get_metadata(struct v4l2_subdev *sd, u8 vc) {
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	unsigned long flags;
	spin_lock_irqsave(&metadata_spinlock, flags);
	memcpy(&sensor->outputSensorMetaData,&sensor->sensorMetaData, sizeof(struct SensorMetaData));
	sensor->sensorMetaData.flags = 0;
	spin_unlock_irqrestore(&metadata_spinlock, flags);
	
	return &sensor->outputSensorMetaData;

}

static void do_post_sof_work(struct v4l2_subdev *sd, u64 sof_timestamp, u64 sof_isr_timestamp, u8 vc) {
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);	
	sensor->last_sof_ts = sof_timestamp;
	sensor->last_sof_isr_ts = sof_isr_timestamp;

	if(/*vc == 1 && */ sensor->timing_adjustment_state == TIMING_ADJUSTMENT_REQUESTED) {
		sensor->timing_adjustment_state = TIMING_READY_FOR_INITIAL_GROUP_WRITE;
		//printk("Asked for it here:%d\n", sensor->sensor_index);
	}
	else if( /*vc == 1 && */ sensor->timing_adjustment_state == TIMING_ADJUSTMENT_IN_PROGRESS && sensor->current_timing_adjustment.number_of_frames_to_hold_adjustment == 0) {
		sensor->timing_adjustment_state = TIMING_READY_FOR_GROUP_RESTORE;
	}
	sensor->last_vc = vc;
	if(sensor_workqueues[sensor->queue_index] != NULL) {
		queue_work(sensor_workqueues[sensor->queue_index], &sensor->work);
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
				   v4l2_ctrl_ptr_create(NULL),
				   v4l2_ctrl_ptr_create(NULL),
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

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	sensor->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static void reset_groups(struct og01a1b_dev *sensor) 
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

/* --------------- Subdev Operations --------------- */

static int og01a1b_s_power(struct v4l2_subdev *sd, int on)
{
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);
	reset_groups(sensor);
	if(sensor->is_sync_sensor) {
		struct i2c_client *client = sensor->i2c_client;
		sensor->ir_emitter_mode = IR_EMITTER_MODE_OFF;
		unsigned long flags;
		spin_lock_irqsave(&ir_spinlock, flags);
		if(!!on) {
			sensor->emittersPresent = true;

			sensor->ir_control_gpio = devm_gpiod_get_optional(&client->dev, "ircontrol",
									GPIOD_OUT_LOW);
			if(sensor->ir_control_gpio == NULL) {
				sensor->ir_control_gpio_present = false;
			}
			else
			{
				sensor->ir_control_gpio_present = true;
			}				
		}
		else 
		{
			if(sensor->ir_control_gpio_present) 
			{
				devm_gpiod_put(&client->dev,sensor->ir_control_gpio);
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
			for(int i = 0; i < NUM_REGISTER_GROUPS; i++) {
				sensor->updateGroupRegisters[i] = true;
			}
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

static int og01a1b_s_stream(struct v4l2_subdev *sd, int enable)
{
    // Reset the sync state
    last_time_in_sync = 0;
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	for(int i = 0; i < NUM_REGISTER_GROUPS; i++) {
		sensor->updateGroupRegisters[i] = true;
	}
	
	sensor->is_streaming = !!enable;
	og01a1b_write_reg(sensor, 0x100, !!enable);
	// Only disable ir_control_gpio when stopping streaming since it can be configured before streamon
	if(!enable && sensor->ir_control_gpio_present) {
		gpiod_set_value(sensor->ir_control_gpio, 0);
	}
	return 0;
}

// This ioctl is used to get the metadata version and enable / disable IR LED controls
// TODO replace some of the misused ioctl (CID_SATURATION etc) with this IOCTL
static long og01a1b_private_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
    u32 ret = 0;
	u32 *a = arg;
    switch (cmd)
    {
        case VIDIOC_OG01A1B_PRV_G_METADATA_VERSION:
        {
            *a = ((METADATA_MAJOR_VERSION << 16) & 0xffff0000) | (METADATA_MINOR_VERSION & 0xffff);
            break;
        }
		case VIDIOC_OG01A1B_PRV_G_DRIVER_VERSION:
		{
			*a = ((DRIVER_MAJOR_VERSION << 16) & 0xffff0000) | (DRIVER_MINOR_VERSION & 0xffff);
			break;
		}
        case VIDIOC_OG01A1B_PRV_S_IR_EMITTER_MODE:
        {
			
			sensor->ir_emitter_mode = *a;
			// strobe configuration is done as part of the group registers update so we need
			// to re-update these when updating IR emitter mode
			for(int i = 0; i < NUM_REGISTER_GROUPS; i++) {
				sensor->updateGroupRegisters[i] = true;
			}
			// If the ir control GPIO is on this camera, enable it if any emitters are on.
			if(sensor->ir_control_gpio_present) {
				if(*a != 0)
				{
					gpiod_set_value(sensor->ir_control_gpio, 1);
				}
				else {
					gpiod_set_value(sensor->ir_control_gpio, 0);
				}
			}
            break;
        }
        case VIDIOC_OG01A1B_PRV_S_REG:
        {
            u32 val = (uintptr_t)arg; // argument is passed as an integer value
            mutex_lock(&sensor->lock);
			if((val & 0xffff) == 0x100) {
				ret = og01a1b_write_reg_dual(sensor, val & 0xffff, val >> 16 & 0xff);
			}
			else {
				ret = og01a1b_write_reg(sensor, val & 0xffff, val >> 16 & 0xff);
			}
           
            mutex_unlock(&sensor->lock);
            break;
        }
        case VIDIOC_OG01A1B_PRV_S_GROUP_REG:
        {
            //ret = og01a1b_queue_reg(sensor, val & 0xffff, val >> 16 & 0xff);
            mutex_lock(&sensor->lock);
            ret = og01a1b_add_or_set_group_reg_from_ctrlval(sensor, (uintptr_t)arg); // argument is passed as an integer value
            mutex_unlock(&sensor->lock);
            break;
        }
        case VIDIOC_OG01A1B_PRV_S_GROUP_MODE:
        {
            sensor->group_mode = (uintptr_t)arg; // argument is passed as an integer value
            break;
        }
		case VIDIOC_SENSOR_PRV_S_TIMING_ADJUSTMENT:
		{
			struct private_timing_adjustment *timing_adjustment = arg;
			//printk("new_rows: %d, rows_to_restore: %d, new_global_row: %d, global_row_to_restore: %d, group_to_adjust: %d sensor id:%d\n", timing_adjustment->new_rows, timing_adjustment->rows_to_restore, timing_adjustment->new_global_row, timing_adjustment->global_row_to_restore, timing_adjustment->group_to_adjust, sensor->sensor_index);
			//ret = og01a1b_set_timing_adjustment(sensor, timing_adjustment);
			sensor->current_timing_adjustment = *timing_adjustment;
			sensor->timing_adjustment_state = TIMING_ADJUSTMENT_REQUESTED;
			break;
		}
		case VIDIOC_SENSOR_PRV_G_REGISTER:
		{
			struct sensor_register *reg = arg;
			u32 regval = 0;
			ret = og01a1b_read_reg_len(sensor, reg->reg_addr, reg->len, &regval);
			if (ret) {
				dev_err(&sensor->i2c_client->dev, "Failed to read register 0x%04x: %d\n", reg->reg_addr, ret);
			}
			reg->reg_value = regval;
			break;
		}
		case VIDIOC_SENSOR_PRV_S_IN_SYNC:
		{
			u32 val = (uintptr_t)arg; // argument is passed as an integer value
			if(val == 1) {
				if(last_time_in_sync == 0) {
					last_time_in_sync = ktime_get_raw_ns();
				}
				else {
					last_time_in_sync = min_t(u64, last_time_in_sync, ktime_get_raw_ns());
				}
			}
			else
			{
				// Means we can't use dual i2c mode right now
				last_time_in_sync = 0;
			}
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
	}
	else if (fmt->format.width == 640 && fmt->format.height == 512) 
	{
		sensor->current_mode = &og01a1b_mode_data_640x512;
	}
	else {
		// Use the 1280x1024 register map
		// Note that this will only work if cropping registers are set up appropriately before streaming
	    sensor->current_mode = &og01a1b_mode_data_1280x1024;
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

	ret = og01a1b_read_reg_len(sensor, OG01A1B_REG_CHIP_ID, 2, (u32 *)&chip_id);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read chip identifier\n",
			__func__);
		goto power_off;
	}

	if (chip_id != 0x4701 && chip_id != 0xc756) {
		dev_err(&client->dev, "%s: wrong chip identifier, expected 0x4701 or 0xc756 (upper cams), got this 0x%x. Likely hardware problem with %s\n",
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

static void update_groups(struct og01a1b_dev *sensor)
{
    u64 last_group_pre_write_timestamp = 0;
    u64 lastGroupPostWriteTimestamp = 0;
    unsigned long flags;

	mutex_lock(&sensor->lock);
	

	last_group_pre_write_timestamp = ktime_get_raw_ns();
		//printk("updating groups for sensor:%d\n", sensor->sensor_index);
	if(last_time_in_sync != 0) {
		if(sensor->dual_i2c_main) {
			og01a1b_write_group_registers_if_needed(sensor, true);
			//printk("dual write on main sensor\n");
			last_dual_write_time = ktime_get_raw_ns();
		}
		// If there is not a dual write after the last time in sync, we need to write on the non main sensor to be safe
		else if(last_dual_write_time < last_time_in_sync) {
			//printk("dual write safety write on non main sensor\n");
			og01a1b_write_group_registers_if_needed(sensor, false);
		}
	}
	// If not in sync both sensors need to be updated
	else {
		//printk("non dual write write on non main sensor\n");
		og01a1b_write_group_registers_if_needed(sensor, false);
	}

	if(sensor->group_mode != MODE_NOGROUPS)
	{
		last_set_group = (last_set_group + 1) % sensor->group_mode;
	}
	else
	{
		last_set_group = 0;
	}

	// Fix the last set group based on the last VC received if that's getting
	// out of sync during group_mode updates
	if(sensor->last_vc == 1 && last_set_group != 1)
	{
		last_set_group = 1;
	}
  
  for(int i = 0; i < numSensors; i++) {
	if(sensors[i]->power_count < 1 ) {
		continue;
	}
	//og01a1b_write_reg(sensors[i], 0x3208, 0xa0 | last_set_group );
  }
  lastGroupPostWriteTimestamp = ktime_get_raw_ns();

	sensor->active_group_registers.group_written = last_set_group;
	sensor->active_group_registers.pre_group_write_timestamp = last_group_pre_write_timestamp;
	sensor->active_group_registers.sof_ts = sensor->last_sof_ts;
	sensor->active_group_registers.sof_isr_ts = sensor->last_sof_isr_ts;
	sensor->active_group_registers.post_group_write_timestamp = lastGroupPostWriteTimestamp;
  spin_lock_irqsave(&metadata_spinlock, flags);

	memcpy(&sensor->sensorMetaData.group_and_reg[4], &sensor->sensorMetaData.group_and_reg[3],sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[3], &sensor->sensorMetaData.group_and_reg[2],sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[2], &sensor->sensorMetaData.group_and_reg[1],sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[1], &sensor->sensorMetaData.group_and_reg[0],sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[0], &sensor->active_group_registers,sizeof(struct group_and_registers));

  spin_unlock_irqrestore(&metadata_spinlock, flags);

  mutex_unlock(&sensor->lock);
}

static void update_groups_work_fn(struct work_struct *Wq)
{
	struct og01a1b_dev *sensor = container_of(Wq, struct og01a1b_dev, work);
	update_groups(sensor);

}

static ssize_t otp_show(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
        struct i2c_client *client = to_i2c_client(dev);
		struct v4l2_subdev *sd = i2c_get_clientdata(client);
		struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
		int ret;

#if 0
	/* DEBUG: OTP read loop for testing - always runs when enabled */
	dev_info(dev, "DEBUG: Running OTP read loop test (200 iterations)\n");
	
	ret = og01a1b_set_power_on(sensor);
	if (ret) {
		dev_err(dev, "DEBUG: Failed to power on sensor for OTP loop: %d\n", ret);
	} else {
		og01a1b_write_reg(sensor, 0x100, 0x01);
		usleep_range(5000, 5100);
		
		u8 val[256];
		u8 otp_data_first_read[256];
		bool first_read = false;
		bool mismatch = false;

		for(int j = 0; j < 200; j++) {
			memset(val, 0, sizeof(val));
			dev_info(dev, "DEBUG: OTP read iteration %d\n", j);
			
			/* Use the single-read function */
			if (og01a1b_read_otp_once(sensor, val) != 0) {
				dev_err(dev, "DEBUG: Failed to read OTP on iteration %d\n", j);
				continue;
			}

			if(first_read == false) {
				memcpy(otp_data_first_read, val, 256);
				first_read = true;
			}
			/* Compare current read with first read */
			if (memcmp(val, otp_data_first_read, 256) != 0) {
				mismatch = true;
				dev_err(dev, "DEBUG: OTP data mismatch detected on iteration %d\n", j);
			} else {
				dev_info(dev, "DEBUG: OTP data matches first read on iteration %d\n", j);
			}
			dev_info(dev, "DEBUG: OTP data first 32 bytes: %*phN\n", 32, val);
		}

		if(mismatch) {
			dev_err(dev, "DEBUG: OTP data mismatch detected after 200 iterations\n");
		} else {
			dev_info(dev, "DEBUG: OTP data matches first read after 200 iterations\n");
		}
		
		og01a1b_set_power_off(sensor);
	}
	dev_info(dev, "DEBUG: OTP read loop test completed\n");
#endif

		mutex_lock(&sensor->lock);

        /* If OTP is already cached, return it immediately */
        if (sensor->otp_valid) {
			memcpy(buf, sensor->otp_data, sizeof(sensor->otp_data));
			mutex_unlock(&sensor->lock);
            return sizeof(sensor->otp_data);
		}

		/* Can't do a clean OTP read while the register map is loaded */
		if (sensor->power_count > 0) {
			dev_err(dev, "Cannot read OTP while sensor is powered on (power_count=%d)\n",
				sensor->power_count);
			mutex_unlock(&sensor->lock);
			return -EBUSY;
		}

		ret = og01a1b_set_power_on(sensor);
		if (ret) {
			dev_err(dev, "Failed to power on sensor for OTP read: %d\n", ret);
			mutex_unlock(&sensor->lock);
			return ret;
		}

		ret = og01a1b_write_reg(sensor, 0x100, 0x01);
		if (ret) {
			dev_err(dev, "Failed to start sensor for OTP read: %d\n", ret);
			goto power_off;
		}

		usleep_range(5000, 5100);

		ret = og01a1b_read_otp_once(sensor, sensor->otp_data);
		if (ret) {
			dev_err(dev, "Failed to read OTP data: %d\n", ret);
			goto power_off;
		}

		sensor->otp_valid = true;
		og01a1b_set_power_off(sensor);
		mutex_unlock(&sensor->lock);

		memcpy(buf, sensor->otp_data, sizeof(sensor->otp_data));
		return sizeof(sensor->otp_data);

power_off:
		og01a1b_set_power_off(sensor);
		mutex_unlock(&sensor->lock);
		return ret;
}

static ssize_t otp_valid_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	bool valid;

	mutex_lock(&sensor->lock);
	valid = sensor->otp_valid;
	mutex_unlock(&sensor->lock);

	return sysfs_emit(buf, "%d\n", valid);
}

static ssize_t otp_valid_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);
	sensor->otp_valid = false;
	memset(sensor->otp_data, 0, sizeof(sensor->otp_data));
	mutex_unlock(&sensor->lock);

	return count;
}

static ssize_t skip_regmap_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);

	return sysfs_emit(buf, "%d\n", sensor->skip_regmap);
}

static ssize_t skip_regmap_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct og01a1b_dev *sensor = to_og01a1b_dev(sd);
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	mutex_lock(&sensor->lock);
	sensor->skip_regmap = val;
	mutex_unlock(&sensor->lock);

	return count;
}

static DEVICE_ATTR_RO(otp);
static DEVICE_ATTR_RW(otp_valid);
static DEVICE_ATTR_RW(skip_regmap);

static struct attribute *og01a1b_dev_attrs[] = {
	&dev_attr_otp.attr,
	&dev_attr_otp_valid.attr,
	&dev_attr_skip_regmap.attr,
	NULL,
};
static const struct attribute_group og01a1b_attr_group = {
    .attrs = og01a1b_dev_attrs,
};

static int og01a1b_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct og01a1b_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;
    const char *id;
    bool skip_check_chip_id = false;
	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;


	/* Expose the sysfs attributes declared above */
	ret = devm_device_add_group(&client->dev, &og01a1b_attr_group);
	if (ret) {
		dev_err(&client->dev,
			"failed to create sysfs files for serial/OTP: %d\n",
			ret);
		goto entity_cleanup;
	}

	sensor->is_streaming = false;
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
		sensor->is_left_sensor = true;
    } else {
		sensor->is_sync_sensor = false;
		INIT_WORK(&sensor->work,update_groups_work_fn);
		if(sync_sensor_index != -1) {
			sensor->other_sensor = sensors[sync_sensor_index];
			// We keep this in for the 2 camera compatibility case
			sensors[sync_sensor_index]->other_sensor = sensor;
		}
		sensor->is_left_sensor = false;
    }

	if(numSensors == 0) {
		for (int i = 0; i < 2; i++) {
            sensor_workqueues[i] = alloc_workqueue("sensor_workqueue%d", WQ_UNBOUND | WQ_HIGHPRI | WQ_SYSFS, 1, i);

            // Define workqueue attributes
            struct workqueue_attrs *attrs;
            attrs = alloc_workqueue_attrs();
            if (!attrs) {
                // Handle allocation failure
                return -ENOMEM;
            }
            cpumask_set_cpu(i + 2, attrs->cpumask); // Set CPU affinity to CPU 2 and 3

            // Apply workqueue attributes
            if (apply_workqueue_attrs(sensor_workqueues[i], attrs)) {
                // Handle attribute application failure
                free_workqueue_attrs(attrs);
                return -EINVAL;
            }
            free_workqueue_attrs(attrs);
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
						     GPIOD_OUT_LOW);
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
		dev_err(&client->dev, "failed to init control: %d", ret);
		goto entity_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto free_ctrls;

	sensor->current_mode = &og01a1b_mode_data_1280x1024;

	// Read the sensor queue index from the device tree
    ret = of_property_read_u32(dev->of_node, "sensor-queue-index", &sensor->queue_index);
    if (ret) {
        dev_err(dev, "Failed to read 'sensor-queue-index' property. Defaulting to 0.\n");
		sensor->queue_index = 0;
    }
	if (sensor->queue_index > 1) {
		dev_err(dev, "Invalid sensor-queue-index value. Defaulting to 0.\n");
		sensor->queue_index = 0;
	}

	u32 dual_i2c_available_val, dual_i2c_main_val;

	ret = of_property_read_u32(dev->of_node, "dual-i2c-available", &dual_i2c_available_val);
	if (ret) {
		dev_err(dev, "Failed to read 'dual-i2c-available' property. Defaulting to 0.\n");
		sensor->dual_i2c_available = false;
	} else {
		sensor->dual_i2c_available = (dual_i2c_available_val != 0);
	}

	ret = of_property_read_u32(dev->of_node, "dual-i2c-main", &dual_i2c_main_val);
	if (ret) {
		dev_err(dev, "Failed to read 'dual-i2c-main' property. Defaulting to 0.\n");
		sensor->dual_i2c_main = false;
	} else {
		sensor->dual_i2c_main = (dual_i2c_main_val != 0);
	}

	/* Initialize dual i2c client on first probe */
	if (!g_dual_i2c_client) {
		struct i2c_board_info dual_info = {
			I2C_BOARD_INFO("og01a1b-dual", 0x70)
		};
		g_dual_i2c_client = i2c_new_client_device(client->adapter, &dual_info);
		if (IS_ERR(g_dual_i2c_client)) {
			dev_err(&client->dev, "Failed to create dual i2c client: %ld\n", 
				PTR_ERR(g_dual_i2c_client));
			g_dual_i2c_client = NULL;
		} else {
			dev_dbg(&client->dev, "Created dual i2c client at address 0x70\n");
		}
	}

	last_time_in_sync = 0;

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

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

	/* Remove from sensors array */
	for (int i = 0; i < numSensors; i++) {
		if (sensors[i] == sensor) {
			/* Shift remaining sensors down */
			for (int j = i; j < numSensors - 1; j++) {
				sensors[j] = sensors[j + 1];
			}
			numSensors--;
			break;
		}
	}

	/* Clean up dual i2c client when last sensor is removed */
	if (numSensors == 0 && g_dual_i2c_client) {
		i2c_unregister_device(g_dual_i2c_client);
		g_dual_i2c_client = NULL;
		dev_dbg(&client->dev, "Cleaned up dual i2c client\n");
	}

	v4l2_async_unregister_subdev(&sensor->sd);
	pm_runtime_disable(&client->dev);
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
