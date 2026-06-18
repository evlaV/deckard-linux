/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 * Copyright (C) 2022-2025 Arcturus Industries
 *
 * Shared definitions for the arcimx616 sensor module.
 */

#ifndef _ARCIMX616_H_
#define _ARCIMX616_H_

#include <linux/clk.h>
#include <linux/container_of.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Shared camera rail. */
#define arcimx616_NUM_SUPPLIES		1

#define ARCIMX616_EEPROM_SIZE		(8 * 1024)

#define ARCIMX616_MAX_SENSORS		2

struct arcimx616_mode_info;

enum arc_led_effect {
	ARC_LED_EFFECT_STATIC = 0,
	ARC_LED_EFFECT_BLINK,
	ARC_LED_EFFECT_SMART_FADE,
};

enum arc_led_override_reason {
	ARC_LED_OVERRIDE_NONE = 0,
	ARC_LED_OVERRIDE_PROBE,
};

struct arcimx616_led_override {
	bool left_enable;
	bool right_enable;
	u8 left_r, left_g, left_b;
	u8 right_r, right_g, right_b;
	u8 left_brightness;
	u8 right_brightness;
	enum arc_led_effect effect;
	u16 fade_on_ms, full_on_ms, fade_off_ms, full_off_ms;
};

struct arcimx616_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep;
	struct clk *xclk;
	struct clk *xclk2;

	struct regulator_bulk_data supplies[arcimx616_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *detect_gpio;
	struct gpio_desc *mux_enable;
	struct gpio_desc *mux_select;
	struct gpio_desc *strobe;
	int irq;

	struct delayed_work connect_work;
	spinlock_t irq_lock; /* protects detect IRQ state */
	bool connect_work_scheduled;
	bool connected;
	bool accessory_power_latched;

	int debounce_cnt;
	bool debounce_sensor0_ok;
	bool debounce_sensor1_ok;
	bool debounce_led_ok;
	bool debounce_settling;

	/* Protects V4L2 state below. */
	struct mutex lock;

	const struct arcimx616_mode_info *current_mode;

	struct v4l2_ctrl_handler ctrl_handler;
	struct mutex mutex; /* protects control handler state */

	struct v4l2_mbus_framefmt fmt;

	bool is_left_sensor;

	/* Owns shared AW9106 attrs. */
	bool is_led_master;

	/* Guarded by the LED mutex. */
	bool led_enable;
	u8 led_r, led_g, led_b;
	u8 led_brightness;

	u8 eeprom_buffer[ARCIMX616_EEPROM_SIZE];
	struct i2c_client *eeprom_client;
};

static inline struct arcimx616_dev *to_arcimx616_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct arcimx616_dev, sd);
}

extern int arcimx616_power_on_count;
extern struct arcimx616_dev *g_arcimx616_sensors[ARCIMX616_MAX_SENSORS];
extern int g_arcimx616_sensor_count;
extern struct i2c_client *g_aw9106_i2c_client;

int arcimx616_aw9106_check_chip_id(struct arcimx616_dev *sensor);

void arcimx616_led_init_state(struct arcimx616_dev *sensor);

int arcimx616_led_create_sysfs(struct arcimx616_dev *sensor);
void arcimx616_led_remove_sysfs(struct arcimx616_dev *sensor);

int arcimx616_led_apply_effective(void);

int arcimx616_led_set_internal_override(enum arc_led_override_reason reason,
					const struct arcimx616_led_override *ov);
int arcimx616_led_clear_internal_override(void);

#endif /* _ARCIMX616_H_ */
