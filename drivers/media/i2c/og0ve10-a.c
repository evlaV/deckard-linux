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

#define V4L2_CID_TEMPERATURE (V4L2_CID_USER_OG01A1B_BASE + 0)

#define OG01A1B_REG_CHIP_ID 0x300a

#define OG01A1B_LINK_FREQUENCY 500000000

#define OG01A1B_REG_TPM_TEMPERATURE 0x4d17

#define NUM_REGISTER_GROUPS 3
#define GROUP_REGMAP_MAX_SIZE 20

DEFINE_SPINLOCK(metadata_spinlock_og0VE10);

#define VIDIOC_OG01A1B_PRV_G_METADATA_VERSION \
	_IOR('V', BASE_VIDIOC_PRIVATE + 1, u32)

#define VIDIOC_OG01A1B_PRV_S_IR_EMITTER_MODE \
	_IOW('V', BASE_VIDIOC_PRIVATE + 2, u32)

// 32-bit argument: sensor register address in bits [16, 0], value in bits [24, 16]
#define VIDIOC_OG01A1B_PRV_S_REG _IO('V', BASE_VIDIOC_PRIVATE + 3)
// 32-bit argument: sensor register address in bits [16, 0], value in bits [24, 16], group ID in bits [31, 24]
#define VIDIOC_OG01A1B_PRV_S_GROUP_REG \
	_IO('V',                       \
	    BASE_VIDIOC_PRIVATE +      \
		    4) // Argument is sensor register to set, address in bits [16, 0] value in bits [24, 16]
// Integer argument: ID of the group mode
#define VIDIOC_OG01A1B_PRV_S_GROUP_MODE _IO('V', BASE_VIDIOC_PRIVATE + 5)

#define VIDIOC_OG01A1B_PRV_G_DRIVER_VERSION \
	_IOR('V', BASE_VIDIOC_PRIVATE + 6, u32)

#define VIDIOC_SENSOR_PRV_S_TIMING_ADJUSTMENT \
	_IOW('V', BASE_VIDIOC_PRIVATE + 7, struct private_timing_adjustment)

#define VIDIOC_SENSOR_PRV_G_REGISTER _IOWR('V', BASE_VIDIOC_PRIVATE + 8, struct sensor_register)

#define VIDIOC_SENSOR_PRV_S_IN_SYNC _IO('V', BASE_VIDIOC_PRIVATE + 9)

struct sensor_register {
	u16 reg_addr;
	u32 reg_value;
	u8 len;
};
// Version 1.2 support histogram streaming.

#define METADATA_MAJOR_VERSION 3
#define METADATA_MINOR_VERSION 0

#define DRIVER_MAJOR_VERSION 1
#define DRIVER_MINOR_VERSION 1

/* regulator supplies */
static const char *const og0ve10_supply_name[] = {
	"DOVDD", /* Digital I/O (1.8V) supply */
	"AVDD", /* Analog (2.8V) supply */
	"DVDD", /* Digital Core (1.5V) supply */
};

#define og0ve10_NUM_SUPPLIES ARRAY_SIZE(og0ve10_supply_name)

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

struct og0ve10_mode_info {
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

enum og0ve10_group_mode {
	MODE_NOGROUPS = 0,
	MODE_1GROUPs = 1,
	MODE_2GROUPS = 2
};

enum og0ve10_timing_adjustment_state {
	TIMING_ADJUSTMENT_NOT_IN_PROGRESS = 0,
	TIMING_READY_FOR_GROUP_RESTORE = 1,
	TIMING_ADJUSTMENT_IN_PROGRESS = 2,
	TIMING_READY_FOR_INITIAL_GROUP_WRITE = 3,
	TIMING_ADJUSTMENT_REQUESTED = 4
};
u64 last_time_in_sync_og0VE10 = 0;
u64 last_dual_write_time_og0VE10 = 0;


struct og0ve10_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to og0ve10 */
	struct regulator_bulk_data supplies[og0ve10_NUM_SUPPLIES];
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
	struct og0ve10_dev *other_sensor;

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

	struct group_and_registers active_group_registers;
	bool updateGroupRegisters;
	enum og0ve10_group_mode group_mode;
	int sensor_index;

	const struct og0ve10_mode_info *current_mode;
	bool is_streaming;
	bool is_left_sensor;

	struct private_timing_adjustment current_timing_adjustment;
	enum og0ve10_timing_adjustment_state timing_adjustment_state;
	int lastGroup;

	int queue_index;

	bool dual_i2c_available;
	bool dual_i2c_main;
};

struct og0ve10_dev *sensors_og0VE10[4];
int numSensors_og0VE10 = 0;
int sync_sensor_index_og0VE10 = -1;

extern struct workqueue_struct *sensor_workqueues[2];

extern struct workqueue_struct *left_sensor_workqueue;
extern struct workqueue_struct *right_sensor_workqueue;

static inline struct og0ve10_dev *to_og0ve10_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct og0ve10_dev, sd);
}

static const struct reg_value og0VE10_setting[] = {
	{ 0x0103, 0x01 }, { 0x0100, 0x00 }, { 0x30a0, 0x02 }, { 0x30a1, 0x00 },
	{ 0x30a2, 0x48 }, { 0x30a3, 0x34 }, { 0x30a4, 0xf7 }, { 0x30a5, 0x00 },
	{ 0x3082, 0x32 }, { 0x3083, 0x01 }, { 0x301c, 0xf0 }, { 0x301e, 0x0b },
	{ 0x3106, 0x10 }, { 0x3708, 0x77 }, { 0x3709, 0xf8 }, { 0x3717, 0x00 },
	{ 0x3782, 0x00 }, { 0x3783, 0x47 }, { 0x37a2, 0x00 }, { 0x3500, 0x00 },
	{ 0x3501, 0x00 }, { 0x3502, 0x40 }, { 0x3503, 0x07 }, { 0x3509, 0x10 },
	{ 0x350b, 0x10 }, { 0x3600, 0x83 }, { 0x3601, 0x21 }, { 0x3602, 0xf1 },
	{ 0x360a, 0x18 }, { 0x360e, 0xb3 }, { 0x3613, 0x20 }, { 0x366a, 0x78 },
	{ 0x3706, 0x63 }, { 0x3713, 0x00 }, { 0x3716, 0xb0 }, { 0x37a1, 0x38 },
	{ 0x3800, 0x00 }, { 0x3801, 0x04 }, { 0x3802, 0x00 }, { 0x3803, 0x04 },
	{ 0x3804, 0x02 }, { 0x3805, 0x8b }, { 0x3806, 0x01 }, { 0x3807, 0xeb },
	{ 0x3808, 0x02 }, { 0x3809, 0x80 }, { 0x380a, 0x01 }, { 0x380b, 0xe0 },
	{ 0x380c, 0x03 }, { 0x380d, 0x18 }, { 0x380e, 0x02 }, { 0x380f, 0x38 },
	{ 0x3811, 0x04 }, { 0x3813, 0x04 }, { 0x3814, 0x11 }, { 0x3815, 0x11 },
	{ 0x3820, 0x00 }, { 0x3821, 0x00 }, { 0x3823, 0x04 }, { 0x382a, 0x00 },
	{ 0x382b, 0x03 }, { 0x3840, 0x00 }, { 0x389e, 0x00 }, { 0x3c05, 0x08 },
	{ 0x3c26, 0x02 }, { 0x3c27, 0xc0 }, { 0x3c28, 0x00 }, { 0x3c29, 0x40 },
	{ 0x3c2c, 0x00 }, { 0x3c2d, 0x50 }, { 0x3c2e, 0x02 }, { 0x3c2f, 0x66 },
	{ 0x3c33, 0x08 }, { 0x3c35, 0x00 }, { 0x3c36, 0x00 }, { 0x3c37, 0x00 },
	{ 0x3f52, 0x9b }, { 0x4001, 0x42 }, { 0x4004, 0x08 }, { 0x4005, 0x00 },
	{ 0x4007, 0x28 }, { 0x4009, 0x40 }, { 0x4307, 0x30 }, { 0x4500, 0x80 },
	{ 0x4501, 0x02 }, { 0x4502, 0x47 }, { 0x4504, 0x7f }, { 0x4601, 0x48 },
	{ 0x4800, 0x64 }, { 0x4801, 0x0f }, { 0x4806, 0x2f }, { 0x4819, 0xaa },
	{ 0x4823, 0x3e }, { 0x5000, 0x85 }, { 0x5e00, 0x0c }, { 0x3899, 0x09 },
	{ 0x4f00, 0x64 }, { 0x4f02, 0x0a }, { 0x4f05, 0x0e }, { 0x4f06, 0x11 },
	{ 0x4f08, 0x0b }, { 0x4f0a, 0xc4 }, { 0x4f20, 0x1f }, { 0x4f25, 0x10 },
	{ 0x3016, 0x10 }, { 0x3017, 0x00 }, { 0x3018, 0x00 }, { 0x3019, 0x00 },
	{ 0x301a, 0x00 }, { 0x301b, 0x00 }, { 0x301c, 0x72 }, { 0x3037, 0x40 },
	{ 0x4f2c, 0x00 }, { 0x4f21, 0x00 }, { 0x4f23, 0x00 }, { 0x4f2a, 0x00 },
	{ 0x3665, 0xe7 }, { 0x3668, 0x48 }, { 0x3671, 0x3c }, { 0x389a, 0x02 },
	{ 0x389b, 0x00 }, { 0x303c, 0xa0 }, { 0x300f, 0xf0 }, { 0x304b, 0x0f },
	{ 0x3662, 0x24 }, { 0x3006, 0x40 }, { 0x4f26, 0x45 }, { 0x3607, 0x34 },
	{ 0x3608, 0x01 }, { 0x360a, 0x0c }, { 0x360b, 0x86 }, { 0x360c, 0xcc },
	{ 0x3013, 0x00 }, { 0x3083, 0x02 }, { 0x3084, 0x12 }, { 0x4601, 0x38 },
	{ 0x366f, 0x3a }, { 0x3713, 0x19 }, { 0x37a2, 0x00 }, { 0x3f43, 0x27 },
	{ 0x3f45, 0x27 }, { 0x3f47, 0x32 }, { 0x3f49, 0x3e }, { 0x3f4b, 0x20 },
	{ 0x3f4d, 0x30 }, { 0x4300, 0x3f }, { 0x4009, 0x10 }, { 0x3f02, 0x68 },
	{ 0x3700, 0x8c }, { 0x370b, 0x7e }, { 0x3f47, 0x35 }, { 0x350b, 0x10 },
	{ 0x3500, 0x00 }, { 0x3501, 0x22 }, { 0x3502, 0xa0 },
};

static const struct og0ve10_mode_info og0ve10_mode_data_640x480 = {
	640, 848, 480, 1176, og0VE10_setting, ARRAY_SIZE(og0VE10_setting)
};

static const s64 link_freq_menu_items[] = {
	OG01A1B_LINK_FREQUENCY,
};

static int og0ve10_write_reg(struct og0ve10_dev *sensor, u16 reg, u8 val)
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

static int og0ve10_add_or_set_group_reg(struct og0ve10_dev *sensor, int ctrlval)
{
	u16 address = ctrlval & 0xffff;
	u8 val = ctrlval >> 16 & 0xff;
	u8 group = ctrlval >> 24 & 0xf;

	struct register_set *reg_set =
		&sensor->active_group_registers.group_register_sets[group];
	int index = -1;
	for (int i = 0; i < reg_set->num_group_reg; i++) {
		if (reg_set->group_reg_addresses[i] == address) {
			index = i;
			break;
		}
	}
	if (index == -1) {
		if (reg_set->num_group_reg >= GROUP_REGMAP_MAX_SIZE) {
			struct i2c_client *client = sensor->i2c_client;
			dev_err(&client->dev,
				"Group register map exceeded maximum size");
			return -1;
		}
		index = reg_set->num_group_reg;
		reg_set->num_group_reg++;
		reg_set->group_reg_addresses[index] = address;
		sensor->updateGroupRegisters = true;
	}
	if (reg_set->group_reg_values[index] != val) {
		reg_set->group_reg_values[index] = val;
		sensor->updateGroupRegisters = true;
	}

	return 0;
}

static int og0ve10_read_reg(struct og0ve10_dev *sensor, u16 reg, u8 *val)
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
		dev_err(&client->dev, "%s: error: reg=%x\n", __func__, reg);
		return ret;
	}

	*val = buf[0];
	return 0;
}

static int og0ve10_read_reg_len(struct og0ve10_dev *sensor, u16 reg, u16 len, u32 *val)
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

static int og0ve10_write_reg16(struct og0ve10_dev *sensor, u16 reg, u16 val)
{
	// Use the og0ve10 sequential write feature to use a single i2c transfer
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

static int og0ve10_write_reg_dual(struct og0ve10_dev *sensor, u16 reg, u8 val)
{
    struct i2c_client *client = sensor->i2c_client;
    unsigned short old_addr = client->addr;
    client->addr = 0x70;
    int ret = og0ve10_write_reg(sensor, reg, val);
    client->addr = old_addr;
    return ret;
}

static int og0ve10_write_reg16_dual(struct og0ve10_dev *sensor, u16 reg, u16 val)
{
    struct i2c_client *client = sensor->i2c_client;
    unsigned short old_addr = client->addr;
    client->addr = 0x70;
    int ret = og0ve10_write_reg16(sensor, reg, val);
    client->addr = old_addr;
    return ret;
}

static int og0ve10_load_regs(struct og0ve10_dev *sensor,
			     const struct og0ve10_mode_info *mode)
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

		ret = og0ve10_write_reg(sensor, reg_addr, val);
		if (ret)
			break;

		if (delay_ms)
			usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
	}

	return 0;
}

/* restore the last set video mode after chip power-on */
static int og0ve10_restore_mode(struct og0ve10_dev *sensor)
{
	int ret;
	u8 val;

	/* first load the initial register values */
	ret = og0ve10_load_regs(sensor, (sensor->current_mode));
	if (ret < 0)
		return ret;

	ret = og0ve10_read_reg(sensor, 0x3662, &val);

	struct i2c_client *client = sensor->i2c_client;
	return 0;
}

static void og0ve10_reset(struct og0ve10_dev *sensor)
{
	if (!sensor->reset_gpio)
		return;

	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(1000, 1100);

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	usleep_range(5000, 5100);
}

static int og0ve10_set_power_on(struct og0ve10_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	// hard code clock rate to 24 mhz
	clk_set_rate(sensor->xclk, 24000000);

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n", __func__);
		//return ret;
	}

	ret = regulator_bulk_enable(og0ve10_NUM_SUPPLIES, sensor->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		goto xclk_off;
	}

	og0ve10_reset(sensor);
	dev_info(&client->dev, "og0ve10_set_power_on: completed successfully\n");
	return 0;

xclk_off:
	clk_disable_unprepare(sensor->xclk);
	return ret;
}

static void og0ve10_set_power_off(struct og0ve10_dev *sensor)
{
	regulator_bulk_disable(og0ve10_NUM_SUPPLIES, sensor->supplies);
	//clk_disable_unprepare(sensor->xclk);
}

static int og0ve10_set_power(struct og0ve10_dev *sensor, bool on)
{
	dev_info(&sensor->i2c_client->dev, "og0ve10_set_power: on = %d\n", on);
	int ret = 0;

	if (on) {
		ret = og0ve10_set_power_on(sensor);
		if (ret) {
			dev_err(&sensor->i2c_client->dev, "og0ve10_set_power: og0ve10_set_power_on failed\n");
			return ret;
		}

		ret = og0ve10_restore_mode(sensor);
		if (ret) {
			dev_err(&sensor->i2c_client->dev, "og0ve10_set_power: og0ve10_restore_mode failed\n");
			goto power_off;
		}

	} else {
		dev_info(&sensor->i2c_client->dev, "og0ve10_set_power: power off\n");
		og0ve10_set_power_off(sensor);
	}

	return 0;

power_off:
	og0ve10_set_power_off(sensor);
	return ret;
}

static long og0ve10_private_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
				  void *arg);

static int og0ve10_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct og0ve10_dev *sensor =
		container_of(ctrl->handler, struct og0ve10_dev, ctrl_handler);
	int ret = 0;

	/* V4L2 controls values will be applied only when power is already up */
	//if (!pm_runtime_get_if_in_use(&client->dev))
	//	return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		/* note:
		 * value in 5.4 fractional format
		 */
		ret = og0ve10_write_reg(sensor, 0x3508,
					(ctrl->val >> 4) & 0x1f);
		ret = og0ve10_write_reg(sensor, 0x3509,
					(ctrl->val << 4) & 0xf0);

		break;

		// "Hack controls" that are kept for backward compatibility
	case V4L2_CID_BRIGHTNESS:
		ret = og0ve10_private_ioctl(&sensor->sd,
					    VIDIOC_OG01A1B_PRV_S_REG,
					    (void *)(uintptr_t)ctrl->val);
		break;
	case V4L2_CID_HUE:
		ret = og0ve10_private_ioctl(&sensor->sd,
					    VIDIOC_OG01A1B_PRV_S_GROUP_REG,
					    (void *)(uintptr_t)ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = og0ve10_private_ioctl(&sensor->sd,
					    VIDIOC_OG01A1B_PRV_S_GROUP_MODE,
					    (void *)(uintptr_t)ctrl->val);
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct v4l2_ctrl_ops og0ve10_ctrl_ops = {
	.s_ctrl = og0ve10_set_ctrl,
};

static void *get_metadata(struct v4l2_subdev *sd)
{
	struct og0ve10_dev *sensor = to_og0ve10_dev(sd);
	unsigned long flags;
	spin_lock_irqsave(&metadata_spinlock_og0VE10, flags);
	memcpy(&sensor->outputSensorMetaData, &sensor->sensorMetaData,
	       sizeof(struct SensorMetaData));
	sensor->sensorMetaData.flags = 0;
	spin_unlock_irqrestore(&metadata_spinlock_og0VE10, flags);

	return &sensor->outputSensorMetaData;
}

// The group parameter is unused here because virtual channel are not supported
static void do_post_sof_work(struct v4l2_subdev *sd, u64 sof_timestamp,
			     u64 sof_isr_timestamp, u8 group)
{
	struct og0ve10_dev *sensor = to_og0ve10_dev(sd);
	sensor->last_sof_ts = sof_timestamp;
	sensor->last_sof_isr_ts = sof_isr_timestamp;
	if (sensor_workqueues[sensor->queue_index] != NULL) {
		queue_work(sensor_workqueues[sensor->queue_index], &sensor->work);
	} else {
		printk("Failed to get queue for sensor\n");
	}
}

static int og0ve10_init_controls(struct og0ve10_dev *sensor)
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

	v4l2_ctrl_new_std(ctrl_hdlr, &og0ve10_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  0, 30000, 1, 0x10);

	v4l2_ctrl_new_std(ctrl_hdlr, &og0ve10_ctrl_ops, V4L2_CID_BRIGHTNESS, 0,
			  0xffffff, 1, 0);

	v4l2_ctrl_new_std(ctrl_hdlr, &og0ve10_ctrl_ops, V4L2_CID_HUE,
			  -2147483648, 2147483647, 1, 0);

	v4l2_ctrl_new_std(ctrl_hdlr, &og0ve10_ctrl_ops, V4L2_CID_SATURATION, 0,
			  2, 1, 0);

	v4l2_ctrl_new_std_compound(ctrl_hdlr, &og0ve10_ctrl_ops,
				   V4L2_CID_COLORIMETRY_HDR10_CLL_INFO,
				   v4l2_ctrl_ptr_create(NULL),
				   v4l2_ctrl_ptr_create(NULL),
				   v4l2_ctrl_ptr_create(NULL));

	size = ARRAY_SIZE(link_freq_menu_items);

	link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &og0ve10_ctrl_ops,
					   V4L2_CID_LINK_FREQ, size - 1, 0,
					   link_freq_menu_items);
	if (link_freq)
		link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	size = ARRAY_SIZE(sensor->metadata_cb_address);

	v4l2_ctrl_new_int_menu(ctrl_hdlr, &og0ve10_ctrl_ops,
			       V4L2_CID_METADATA_CB_ADDR, size - 1, 0,
			       sensor->metadata_cb_address);
	sensor->metadata_cb_address[0] = (s64)(get_metadata);
	size = ARRAY_SIZE(sensor->metadata_size);
	size_control = v4l2_ctrl_new_int_menu(ctrl_hdlr, &og0ve10_ctrl_ops,
					      V4L2_CID_METADATA_SIZE, size - 1,
					      0, sensor->metadata_size);
	sensor->sof_cb_address[0] = (s64)(do_post_sof_work);
	size = ARRAY_SIZE(sensor->sof_cb_address);
	v4l2_ctrl_new_int_menu(ctrl_hdlr, &og0ve10_ctrl_ops,
			       V4L2_CID_SENSOR_SOF_CB_ADDR, size - 1, 0,
			       sensor->sof_cb_address);

	sensor->metadata_size[0] = sizeof(struct SensorMetaData);
	if (size_control)
		size_control->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (ctrl_hdlr->error) {
		return ctrl_hdlr->error;
	}

	sensor->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

void reset_groups_og0VE10(struct og0ve10_dev *sensor)
{
	sensor->active_group_registers.pre_group_write_timestamp = 0;
	sensor->active_group_registers.post_group_write_timestamp = 0;
	for (int i = 0; i < NUM_REGISTER_GROUPS; i++) {
		sensor->active_group_registers.group_register_sets[i]
			.num_group_reg = 0;
		for (int j = 0; j < GROUP_REGMAP_MAX_SIZE; j++) {
			sensor->active_group_registers.group_register_sets[i]
				.group_reg_addresses[j] = 0x00;
			sensor->active_group_registers.group_register_sets[i]
				.group_reg_values[j] = 0x00;
		}
	}
	sensor->active_group_registers.group_register_sets->num_group_reg = 0;

	memcpy(&sensor->sensorMetaData.group_and_reg[4],
	       &sensor->active_group_registers,
	       sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[3],
	       &sensor->active_group_registers,
	       sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[2],
	       &sensor->active_group_registers,
	       sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[1],
	       &sensor->active_group_registers,
	       sizeof(struct group_and_registers));
	memcpy(&sensor->sensorMetaData.group_and_reg[0],
	       &sensor->active_group_registers,
	       sizeof(struct group_and_registers));
}

/* --------------- Subdev Operations --------------- */

static int og0ve10_s_power(struct v4l2_subdev *sd, int on)
{
	struct og0ve10_dev *sensor = to_og0ve10_dev(sd);
	int ret = 0;
	static char *gpionames[] = { "ira", "irb" };
	mutex_lock(&sensor->lock);
	reset_groups_og0VE10(sensor);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (sensor->power_count == !on) {
		if (!!on == 0) {
			sensor->updateGroupRegisters = true;
		}
		ret = og0ve10_set_power(sensor, !!on);
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

static int og0ve10_s_stream(struct v4l2_subdev *sd, int enable)
{
	last_time_in_sync_og0VE10 = 0;
	struct og0ve10_dev *sensor = to_og0ve10_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	sensor->updateGroupRegisters = true;

	sensor->is_streaming = !!enable;

	og0ve10_write_reg(sensor, 0x100, !!enable);
	return 0;
}

// This ioctl is used to get the metadata version and enable / disable IR LED controls
// TODO replace some of the misused ioctl (CID_SATURATION etc) with this IOCTL
static long og0ve10_private_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
				  void *arg)
{
	struct og0ve10_dev *sensor = to_og0ve10_dev(sd);
	struct i2c_client *client = sensor->i2c_client;
	u32 ret = 0;
	u32 *a = arg;
	unsigned long flags;
	switch (cmd) {
	case VIDIOC_OG01A1B_PRV_G_METADATA_VERSION: {
		*a = ((METADATA_MAJOR_VERSION << 16) & 0xffff0000) |
		     (METADATA_MINOR_VERSION & 0xffff);
		break;
	}
	case VIDIOC_OG01A1B_PRV_G_DRIVER_VERSION: {
		*a = ((DRIVER_MAJOR_VERSION << 16) & 0xffff0000) |
		     (DRIVER_MINOR_VERSION & 0xffff);
		break;
	}
	case VIDIOC_OG01A1B_PRV_S_REG: {
		u32 val = (uintptr_t)
		arg; // argument is passed as an integer value
		mutex_lock(&sensor->lock);
		
		ret = og0ve10_write_reg(sensor, val & 0xffff, val >> 16 & 0xff);
		mutex_unlock(&sensor->lock);	
		break;
	}
	case VIDIOC_OG01A1B_PRV_S_GROUP_REG: {
		//ret = og0ve10_queue_reg(sensor, val & 0xffff, val >> 16 & 0xff);
		mutex_lock(&sensor->lock);
		ret = og0ve10_add_or_set_group_reg(
			sensor,
			(uintptr_t)
				arg); // argument is passed as an integer value
		mutex_unlock(&sensor->lock);
		break;
	}
	case VIDIOC_OG01A1B_PRV_S_GROUP_MODE: {
		sensor->group_mode = (uintptr_t)
			arg; // argument is passed as an integer value
		break;
	}
	case VIDIOC_SENSOR_PRV_S_TIMING_ADJUSTMENT: {
		struct private_timing_adjustment *timing_adjustment = arg;
		//printk("new_rows: %d, rows_to_restore: %d, new_global_row: %d, global_row_to_restore: %d, group_to_adjust: %d sensor id:%d\n", timing_adjustment->new_rows, timing_adjustment->rows_to_restore, timing_adjustment->new_global_row, timing_adjustment->global_row_to_restore, timing_adjustment->group_to_adjust, sensor->sensor_index);
		//ret = og01a1b_set_timing_adjustment(sensor, timing_adjustment);
		sensor->current_timing_adjustment = *timing_adjustment;
		sensor->timing_adjustment_state = TIMING_ADJUSTMENT_REQUESTED;
		break;
	}
	case VIDIOC_SENSOR_PRV_G_REGISTER: {
		struct sensor_register *reg = arg;
		u32 regval = 0;
		ret = og0ve10_read_reg_len(sensor, reg->reg_addr, reg->len, &regval);
		if (ret) {
			dev_err(&sensor->i2c_client->dev, "Failed to read register 0x%04x: %d\n", reg->reg_addr, ret);
		}
		reg->reg_value = regval;
		break;
	}
	case VIDIOC_SENSOR_PRV_S_IN_SYNC:
	{
		u32 val = (uintptr_t)arg;
		if (val == 1) {
			if (last_time_in_sync_og0VE10 == 0) {
				last_time_in_sync_og0VE10 = ktime_get_raw_ns();
			} else {
				last_time_in_sync_og0VE10 = min_t(u64, last_time_in_sync_og0VE10, ktime_get_raw_ns());
			}
		} else {
			last_time_in_sync_og0VE10 = 0;
		}
		break;
	}
	default: {
		ret = -EINVAL;
	}
	}

	return ret;
}

static const struct v4l2_subdev_core_ops og0ve10_core_ops = {
	.s_power = og0ve10_s_power,
	.ioctl = og0ve10_private_ioctl,
};

static const struct v4l2_subdev_video_ops og0ve10_video_ops = {
	.s_stream = og0ve10_s_stream,
};

static int og0ve10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	code->code = MEDIA_BUS_FMT_Y8_1X8;
	return 0;
}

static int og0ve10_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	int ret = 0;
	struct og0ve10_dev *sensor = to_og0ve10_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

	mutex_lock(&sensor->lock);

	if (fmt->format.width == 640 && fmt->format.height == 480) {
		sensor->current_mode = &og0ve10_mode_data_640x480;
	} else {
		dev_err(&client->dev, "%s: Unsupported format.\n", __func__);
		sensor->current_mode = &og0ve10_mode_data_640x480;
		// ret = -1;
		// goto set_fmt_cleanup;
	}

	sensor->fmt.width = fmt->format.width;
	sensor->fmt.height = fmt->format.height;

set_fmt_cleanup:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int og0ve10_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *format)
{
	struct og0ve10_dev *sensor = to_og0ve10_dev(sd);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	format->format = sensor->fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static const struct v4l2_subdev_pad_ops og0ve10_pad_ops = {
	.enum_mbus_code = og0ve10_enum_mbus_code,
	.get_fmt = og0ve10_get_fmt,
	.set_fmt = og0ve10_set_fmt,
};

static const struct v4l2_subdev_ops og0ve10_subdev_ops = {
	.core = &og0ve10_core_ops,
	.video = &og0ve10_video_ops,
	.pad = &og0ve10_pad_ops,
};

static int og0ve10_get_regulators(struct og0ve10_dev *sensor)
{
	int i;

	for (i = 0; i < og0ve10_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = og0ve10_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       og0ve10_NUM_SUPPLIES, sensor->supplies);
}

static int og0ve10_check_chip_id(struct og0ve10_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct device *dev = &client->dev;
	const char *id;
	int ret = 0;
	u16 chip_id;

	ret = og0ve10_set_power_on(sensor);
	if (ret)
		return ret;

	ret = og0ve10_read_reg_len(sensor, OG01A1B_REG_CHIP_ID, 2, (u32 *)&chip_id);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read chip identifier\n",
			__func__);
		goto power_off;
	}

	if (chip_id != 0x4701 && chip_id != 0xc756) {
		dev_err(&client->dev,
			"%s: wrong chip identifier, expected 0x4701 or 0xc756 (upper cams), got this 0x%x. Likely hardware problem with %s\n",
			__func__, chip_id,
			of_property_read_string(dev->of_node, "id", &id) ?
				"unknown camera" :
				id);
		ret = -ENXIO;
		ret = 0;
	} else {
		dev_err(&client->dev,
			"%s: Camera connection verified ( chip ID 0x%x ) for %s\n",
			__func__, chip_id,
			of_property_read_string(dev->of_node, "id", &id) ?
				"unknown camera" :
				id);
	}

power_off:
	og0ve10_set_power_off(sensor);
	return ret;
}

int og0VE10_write_registers_for_group(struct og0ve10_dev *sensor, int group, bool use_dual_i2c)
{
	int ret;

	for (int i = 0;
	     i < sensor->active_group_registers.group_register_sets[group]
			 .num_group_reg;
	     i++) {
		// Check the next group register to detect sequential write and save an i2c transfer by using og0ve10_write_reg16
		bool is_sequential_write = false;
		if (i <
		    (sensor->active_group_registers.group_register_sets[group]
			     .num_group_reg -
		     1)) {
			is_sequential_write =
				(sensor->active_group_registers
					 .group_register_sets[group]
					 .group_reg_addresses[i + 1] ==
				 (sensor->active_group_registers
					  .group_register_sets[group]
					  .group_reg_addresses[i] +
				  1));
		}

		if (is_sequential_write) {
			u16 reg = sensor->active_group_registers
					  .group_register_sets[group]
					  .group_reg_addresses[i];
			// Put the current group in the high 8 bits since they are written first
			u16 val = (sensor->active_group_registers
					   .group_register_sets[group]
					   .group_reg_values[i]
				   << 8) |
				  sensor->active_group_registers
					  .group_register_sets[group]
					  .group_reg_values[i + 1];
			ret = use_dual_i2c ? og0ve10_write_reg16_dual(sensor, reg, val) : og0ve10_write_reg16(sensor, reg, val);
			i++; // bump i since we wrote two registers
		} else {
			ret = use_dual_i2c ? og0ve10_write_reg_dual(sensor, 
				sensor->active_group_registers
					.group_register_sets[group]
					.group_reg_addresses[i],
				sensor->active_group_registers
					.group_register_sets[group]
					.group_reg_values[i]) :
				og0ve10_write_reg(sensor, 
				sensor->active_group_registers
					.group_register_sets[group]
					.group_reg_addresses[i],
				sensor->active_group_registers
					.group_register_sets[group]
					.group_reg_values[i]);
		}

		if (ret != 0) {
			return ret;
		}
		//printk("wrote 0x%x val:0x%x for camera %d",sensor->active_group_registers.group_register_sets[group].group_reg_addresses[i], sensor->active_group_registers.group_register_sets[group].group_reg_values[i], sensor->sensor_index);
	}
	if (group == sensor->current_timing_adjustment.group_to_adjust) {
		if (sensor->timing_adjustment_state ==
		    TIMING_ADJUSTMENT_REQUESTED) {
			ret = og0ve10_write_reg16(
				sensor, 0x380e,
				sensor->current_timing_adjustment.new_rows);
			if (ret != 0) {
				return ret;
			}
			ret = og0ve10_write_reg16(
				sensor, 0x382c,
				sensor->current_timing_adjustment
					.new_global_row);
			if (ret != 0) {
				return ret;
			}
			sensor->timing_adjustment_state =
				TIMING_READY_FOR_GROUP_RESTORE;
		} else if (sensor->timing_adjustment_state ==
			   TIMING_READY_FOR_GROUP_RESTORE) {
			ret = og0ve10_write_reg16(
				sensor, 0x380e,
				sensor->current_timing_adjustment
					.rows_to_restore);
			if (ret != 0) {
				return ret;
			}
			ret = og0ve10_write_reg16(
				sensor, 0x382c,
				sensor->current_timing_adjustment
					.global_row_to_restore);
			if (ret != 0) {
				return ret;
			}
			sensor->timing_adjustment_state =
				TIMING_ADJUSTMENT_NOT_IN_PROGRESS;
		}
	}
	if (ret != 0) {
		return ret;
	}

	return 0;
}

int last_set_group_og0VE10 = 0;

void update_groups_og0VE10(struct og0ve10_dev *sensor)
{
	u64 last_group_pre_write_timestamp = 0;
	u64 lastGroupPostWriteTimestamp = 0;
	unsigned long flags;

	mutex_lock(&sensor->lock);

	last_group_pre_write_timestamp = ktime_get_raw_ns();

	if (sensor->group_mode == MODE_2GROUPS) {
		sensor->lastGroup = !sensor->lastGroup;
	} else {
		sensor->lastGroup = 0;
	}

	if (last_time_in_sync_og0VE10 != 0) {
		if (sensor->dual_i2c_main) {
			og0VE10_write_registers_for_group(sensor, sensor->lastGroup, true);
			last_dual_write_time_og0VE10 = ktime_get_raw_ns();
		} else if (last_dual_write_time_og0VE10 < last_time_in_sync_og0VE10) {
			//printk("og0ve10 dual write safety write on non main sensor\n");
			og0VE10_write_registers_for_group(sensor, sensor->lastGroup, false);
		}
	} else {
		//printk("og0ve10 non dual write write on non main sensor\n");
		og0VE10_write_registers_for_group(sensor, sensor->lastGroup, false);
	}

	lastGroupPostWriteTimestamp = ktime_get_raw_ns();

	//for(int i = 0; i < numSensors_og0VE10; i++)
	{
		if (sensor->power_count < 1) {
			goto update_groups_og0VE10_cleanup;
		}
		sensor->active_group_registers.group_written =
			last_set_group_og0VE10;
		sensor->active_group_registers.pre_group_write_timestamp =
			last_group_pre_write_timestamp;
		sensor->active_group_registers.sof_ts = sensor->last_sof_ts;
		sensor->active_group_registers.sof_isr_ts =
			sensor->last_sof_isr_ts;
		sensor->active_group_registers.post_group_write_timestamp =
			lastGroupPostWriteTimestamp;
	}
	spin_lock_irqsave(&metadata_spinlock_og0VE10, flags);
	//for(int i = 0; i < numSensors_og0VE10; i++)
	{
		if (sensor->power_count < 1) {
			goto update_groups_og0VE10_cleanup;
		}
		memcpy(&sensor->sensorMetaData.group_and_reg[4],
		       &sensor->sensorMetaData.group_and_reg[3],
		       sizeof(struct group_and_registers));
		memcpy(&sensor->sensorMetaData.group_and_reg[3],
		       &sensor->sensorMetaData.group_and_reg[2],
		       sizeof(struct group_and_registers));
		memcpy(&sensor->sensorMetaData.group_and_reg[2],
		       &sensor->sensorMetaData.group_and_reg[1],
		       sizeof(struct group_and_registers));
		memcpy(&sensor->sensorMetaData.group_and_reg[1],
		       &sensor->sensorMetaData.group_and_reg[0],
		       sizeof(struct group_and_registers));
		memcpy(&sensor->sensorMetaData.group_and_reg[0],
		       &sensor->active_group_registers,
		       sizeof(struct group_and_registers));
	}
update_groups_og0VE10_cleanup:
	spin_unlock_irqrestore(&metadata_spinlock_og0VE10, flags);

	mutex_unlock(&sensor->lock);
}

static void update_groups_og0VE10_work_fn(struct work_struct *Wq)
{
	struct og0ve10_dev *sensor = container_of(Wq, struct og0ve10_dev, work);
	update_groups_og0VE10(sensor);
}

static int og0ve10_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct og0ve10_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;
	const char *id;
	bool skip_check_chip_id = false;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	sensor->is_streaming = false;
	sensor->sensor_index = numSensors_og0VE10;
	sensor->last_set_led_state = 0;
	if (of_property_read_string(dev->of_node, "id", &id)) {
		dev_err(dev, "Failed to read 'id' property\n");
		return -EINVAL;
	}
	if (strcmp(id, "upper_left") == 0) {
		sensor->is_left_sensor = true;
		sensor->lastGroup = 0;
	} else {
		sensor->is_left_sensor = false;
		sensor->lastGroup = 0;
	}

	INIT_WORK(&sensor->work, update_groups_og0VE10_work_fn);

	sensors_og0VE10[numSensors_og0VE10] = sensor;
	numSensors_og0VE10++;
	sensor->group_mode = MODE_NOGROUPS;

	sensor->i2c_client = client;

	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_Y8_1X8;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 640;
	fmt->height = 480;
	fmt->field = V4L2_FIELD_NONE;

	endpoint =
		fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
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
	sensor->reset_gpio =
		devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return PTR_ERR(sensor->reset_gpio);

	sensor->strobe_gpio = devm_gpiod_get_optional(dev, "strobe", GPIOD_IN);
	if (sensor->strobe_gpio == NULL) {
		return -1;
	}

	for (int i = 0; i < 2; i++) {
		sensor->ir_gpio[i] = NULL;
	}
	v4l2_i2c_subdev_init(&sensor->sd, client, &og0ve10_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	ret = og0ve10_get_regulators(sensor);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	skip_check_chip_id =
		of_property_read_bool(dev->of_node, "skip-check-id");
	if (!skip_check_chip_id) {
		u32 also_check_index;
		ret = og0ve10_check_chip_id(sensor);
		if (ret) {
			dev_err(&client->dev, "Failed to get chip ID for %s\n",
				of_property_read_string(dev->of_node, "id",
							&id) ?
					"unknown camera" :
					id);
			//Continue because 2 of these cameras share a power pin and this can prevent this from working at probe
			//goto entity_cleanup;
		}

		if (of_property_read_u32(dev->of_node, "also-check-index",
					 &also_check_index) == 0) {
			printk("also_check_index:%d sensor->sensor_index:%d\n",
			       also_check_index, sensor->sensor_index);
			if (also_check_index < sensor->sensor_index) {
				struct og0ve10_dev *other_sensor =
					sensors_og0VE10[also_check_index];
				//printk("Would have checked chip id of 0x%x",other_sensor);
				og0ve10_check_chip_id(other_sensor);
			}
		} else {
			printk("also_check_index:%d sensor->sensor_index:%d\n",
			       also_check_index, sensor->sensor_index);
		}
	}

	mutex_init(&sensor->mutex);
	ret = og0ve10_init_controls(sensor);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto entity_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto free_ctrls;

	sensor->current_mode = &og0ve10_mode_data_640x480;

	// Read the sensor queue index from the device tree
	ret = of_property_read_u32(dev->of_node, "sensor-queue-index", &sensor->queue_index);
	if (ret) {
		dev_err(dev, "Failed to read 'sensor-queue-index' property. Defaulting to 0.\n");
		sensor->queue_index = 0;
		//return ret;
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

	last_time_in_sync_og0VE10 = 0;

	// TODO: remove that, this is handled by og01a1b-a as a shared workqueue
	// if(sensor->queue_index == 0) {
	// 	for (int i = 0; i < 2; i++) {
    //         sensor_workqueues[i] = alloc_workqueue("sensor_workqueue%d", WQ_UNBOUND | WQ_HIGHPRI | WQ_SYSFS, 1, i);

    //         // Define workqueue attributes
    //         struct workqueue_attrs *attrs;
    //         attrs = alloc_workqueue_attrs();
    //         if (!attrs) {
    //             // Handle allocation failure
    //             return -ENOMEM;
    //         }
    //         cpumask_set_cpu(i + 2, attrs->cpumask); // Set CPU affinity to CPU 2 and 3

    //         // Apply workqueue attributes
    //         if (apply_workqueue_attrs(sensor_workqueues[i], attrs)) {
    //             // Handle attribute application failure
    //             free_workqueue_attrs(attrs);
    //             return -EINVAL;
    //         }
    //         free_workqueue_attrs(attrs);
    //     }
	// }

	return 0;

free_ctrls:
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static void og0ve10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og0ve10_dev *sensor = to_og0ve10_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
}

static const struct i2c_device_id og0ve10_id[] = {
	{ "og0ve10", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, og0ve10_id);

static const struct of_device_id og0ve10_dt_ids[] = {
	{ .compatible = "ovti,og0ve10x" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, og0ve10_dt_ids);

static struct i2c_driver og0ve10_i2c_driver = {
	.driver = {
		.name  = "og0ve10",
		.of_match_table	= og0ve10_dt_ids,
	},
	.id_table = og0ve10_id,
	.probe = og0ve10_probe,
	.remove   = og0ve10_remove,
};

module_i2c_driver(og0ve10_i2c_driver);

MODULE_DESCRIPTION("og0ve10 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
