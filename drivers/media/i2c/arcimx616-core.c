// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2014-2017 Mentor Graphics Inc.
 * Copyright (C) 2022-2025 Arcturus Industries
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
#include <linux/nvmem-consumer.h>
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

#include "arcimx616.h"

#define VIVID_CID_ARCIMX616_BASE	(V4L2_CID_USER_BASE | 0xf000)
#define V4L2_CID_TEMPERATURE	(VIVID_CID_ARCIMX616_BASE + 0)

#define arcimx616_REG_CHIP_ID		0x16

#define ARCIMX616_LINK_FREQUENCY  2200000000

#define IMX_REG_TEMP_SEN_OUT		0x013A

/* Private ioctls. */
struct eeprom_access_info {
	u32 offset;			/* EEPROM offset */
	u32 size;			/* bytes copied */
	void __user *buffer;		/* userspace data buffer */
};
#define VIDIOC_PRIV_G_CONNECTED _IOR('V', BASE_VIDIOC_PRIVATE + 1, u32)
/* BASE_VIDIOC_PRIVATE + 2 reserved (formerly VIDIOC_PRIV_S_CONNECTED) */
#define VIDIOC_PRIV_S_DUAL_REG _IO('V', BASE_VIDIOC_PRIVATE + 3)
#define VIDIOC_PRIV_G_EEPROM_DATA _IOWR('V', BASE_VIDIOC_PRIVATE + 4, struct eeprom_access_info) 
#define VIDIOC_PRIV_S_EEPROM_DATA _IOW('V', BASE_VIDIOC_PRIVATE + 5, struct eeprom_access_info) 

/* Private events. */
#define V4L2_EVENT_SENSOR_CONNECTED   (V4L2_EVENT_PRIVATE_START + 2)
#define V4L2_EVENT_SENSOR_DISCONNECTED   (V4L2_EVENT_PRIVATE_START + 3)

/* regulator supplies */
static const char * const arcimx616_supply_name[] = {
	"DOVDD", /* Entire camera system supply */
};
static_assert(ARRAY_SIZE(arcimx616_supply_name) == arcimx616_NUM_SUPPLIES);

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

struct arcimx616_mode_info {
	u32 hact;
	u32 htot;
	u32 vact;
	u32 vtot;
	const struct reg_value *reg_data;
	u32 reg_data_size;
};

int arcimx616_power_on_count;

struct arcimx616_find_ctx {
	struct i2c_adapter *adap;
	u16 addr;
	struct i2c_client *found;
};

static int arcimx616_match_client(struct device *dev, void *data)
{
	struct arcimx616_find_ctx *ctx = data;
	struct i2c_client *c = i2c_verify_client(dev);

	return c && c->adapter == ctx->adap && c->addr == ctx->addr
	       ? (ctx->found = c, 1) : 0;
}

/*
 * Ensure an at24 (24c64) client exists *and* its nvmem provider is ready,
 * so that nvmem_device_get() will succeed without returning -EPROBE_DEFER.
 */
#define ARCIMX616_NVMEM_NAME_LEFT      "passthrough_cams_eeprom_left"
#define ARCIMX616_NVMEM_NAME_RIGHT     "passthrough_cams_eeprom_right"
#define ARCIMX616_EEPROM_TPOR_MS  10

static int arcimx616_ensure_eeprom_client(struct arcimx616_dev *sensor)
{
	struct i2c_client  *master = sensor->i2c_client;
	struct i2c_adapter *adap   = master->adapter;
	struct i2c_client  *cl;
	
	bool is_left = sensor->is_left_sensor;
	u8 addr = (is_left) ? 0x50 : 0x53;
	struct i2c_board_info info = { I2C_BOARD_INFO("24c64", addr) };
	/* already cached? */
	if (sensor->eeprom_client && sensor->eeprom_client->dev.driver)
		return 0;                       /* ready to use */

	/* make sure the device had time to wake up after rail-on */
	msleep(ARCIMX616_EEPROM_TPOR_MS);

	/* locate an existing client, create one, or reuse the dummy */
	{
		struct arcimx616_find_ctx ctx = { .adap = adap, .addr = addr };
		i2c_for_each_dev(&ctx, arcimx616_match_client);
		cl = ctx.found;
	}

	if (!cl) {                             /* none at all -> create */
		cl = i2c_new_client_device(adap, &info);
		if (IS_ERR(cl))
			return PTR_ERR(cl);        /* -EBUSY should not happen here */
	}

	sensor->eeprom_client = cl;
	struct nvmem_device *nvmem;

	if (cl->dev.driver) {
		/* driver already bound – provider should exist */
		nvmem = nvmem_device_get(&cl->dev, is_left ? ARCIMX616_NVMEM_NAME_LEFT : ARCIMX616_NVMEM_NAME_RIGHT);
		if (!IS_ERR(nvmem)) {
			nvmem_device_put(nvmem);
			return 0;             /* success */
		}
		if (PTR_ERR(nvmem) != -EPROBE_DEFER) {
			dev_err(&master->dev, "nvmem device not found %ld\n", PTR_ERR(nvmem));
			return PTR_ERR(nvmem);
		}
	} else {
		/* No driver yet – try again */
		int ret = device_reprobe(&cl->dev);
		if(ret == 0) {
			return 0;
		}
		else {
			dev_err(&master->dev, "device re-probe failed %d\n", ret);
			return ret;
		}
	}
	dev_err(&master->dev, "failed to find nvmem device returning -ETIMEDOUT\n");
	return -ETIMEDOUT;
}



struct arcimx616_dev *g_arcimx616_sensors[ARCIMX616_MAX_SENSORS];
int g_arcimx616_sensor_count;

static struct i2c_client *g_dual_i2c_client = NULL;

struct i2c_client *g_aw9106_i2c_client;

/* Probe-only clients for the custom peripheral path. */
static struct i2c_client *g_temp_i2c_client = NULL;
static struct i2c_client *g_eeprom_i2c_client = NULL;

static int arcimx616_set_power(struct arcimx616_dev *sensor, bool on);

/*
 * Force power off on ALL known ARCIMX616 sensors. Synchronous: callers
 * can rely on the rails being down by the time this returns. Must be
 * invoked from sleeping context (process / threaded IRQ) and without
 * any spinlock held, since arcimx616_set_power() uses regulators and
 * usleep_range().
 */
static void arcimx616_power_off_all_sensors(void)
{
	int i;

	pr_info("arcimx616: Forcing power off on all sensors\n");

	for (i = 0; i < g_arcimx616_sensor_count; i++) {
		if (g_arcimx616_sensors[i])
			arcimx616_set_power(g_arcimx616_sensors[i], false);
	}
}

static const struct reg_value arcimx616_global_settings[] = {
{0x0136, 0x18,0 ,0}, {0x0137, 0x00,0 ,5}, {0x3c7e, 0x08,0 ,0},
{0x3c7f, 0x15,0 ,0}, {0x0111, 0x02,0 ,0},

/* Global settings. */
{0x380C, 0x00, 0, 0}, {0x3C00, 0x01, 0, 0}, {0x3C01, 0x00, 0, 0},
{0x3C02, 0x00, 0, 0}, {0x3C03, 0x03, 0, 0}, {0x3C04, 0xFF, 0, 0},
{0x3C05, 0x01, 0, 0}, {0x3C06, 0x00, 0, 0}, {0x3C07, 0x00, 0, 0},
{0x3C08, 0x03, 0, 0}, {0x3C09, 0xFF, 0, 0}, {0x3C0A, 0x00, 0, 0},
{0x3C0B, 0x00, 0, 0}, {0x3C0C, 0x10, 0, 0}, {0x3C0D, 0x10, 0, 0},
{0x3C0E, 0x10, 0, 0}, {0x3C0F, 0x10, 0, 0}, {0x3C10, 0x10, 0, 0},
{0x3C11, 0x20, 0, 0}, {0x3C15, 0x00, 0, 0}, {0x3C16, 0x00, 0, 0},
{0x3C17, 0x01, 0, 0}, {0x3C18, 0x00, 0, 0}, {0x3C19, 0x01, 0, 0},
{0x3C1A, 0x00, 0, 0}, {0x3C1B, 0x01, 0, 0}, {0x3C1C, 0x00, 0, 0},
{0x3C1D, 0x01, 0, 0}, {0x3C1E, 0x00, 0, 0}, {0x3C1F, 0x00, 0, 0},
{0x3F89, 0x01, 0, 0}, {0x3F8F, 0x01, 0, 0}, {0x53B9, 0x01, 0, 0},
{0x62C4, 0x04, 0, 0}, {0x658F, 0x07, 0, 0}, {0x6590, 0x05, 0, 0},
{0x6591, 0x07, 0, 0}, {0x6592, 0x05, 0, 0}, {0x6593, 0x07, 0, 0},
{0x6594, 0x05, 0, 0}, {0x6595, 0x07, 0, 0}, {0x6596, 0x05, 0, 0},
{0x6597, 0x05, 0, 0}, {0x6598, 0x05, 0, 0}, {0x6599, 0x05, 0, 0},
{0x659A, 0x05, 0, 0}, {0x659B, 0x05, 0, 0}, {0x659C, 0x05, 0, 0},
{0x659D, 0x05, 0, 0}, {0x659E, 0x07, 0, 0}, {0x659F, 0x05, 0, 0},
{0x65A0, 0x07, 0, 0}, {0x65A1, 0x05, 0, 0}, {0x65A2, 0x07, 0, 0},
{0x65A3, 0x05, 0, 0}, {0x65A4, 0x07, 0, 0}, {0x65A5, 0x05, 0, 0},
{0x65A6, 0x05, 0, 0}, {0x65A7, 0x05, 0, 0}, {0x65A8, 0x05, 0, 0},
{0x65A9, 0x05, 0, 0}, {0x65AA, 0x05, 0, 0}, {0x65AB, 0x05, 0, 0},
{0x65AC, 0x05, 0, 0}, {0x65AD, 0x07, 0, 0}, {0x65AE, 0x07, 0, 0},
{0x65AF, 0x07, 0, 0}, {0x65B0, 0x05, 0, 0}, {0x65B1, 0x05, 0, 0},
{0x65B2, 0x05, 0, 0}, {0x65B3, 0x05, 0, 0}, {0x65B4, 0x07, 0, 0},
{0x65B5, 0x07, 0, 0}, {0x65B6, 0x07, 0, 0}, {0x65B7, 0x07, 0, 0},
{0x65B8, 0x05, 0, 0}, {0x65B9, 0x05, 0, 0}, {0x65BA, 0x05, 0, 0},
{0x65BB, 0x05, 0, 0}, {0x65BC, 0x05, 0, 0}, {0x65BD, 0x05, 0, 0},
{0x65BE, 0x05, 0, 0}, {0x65BF, 0x05, 0, 0}, {0x65C0, 0x05, 0, 0},
{0x65C1, 0x05, 0, 0}, {0x65C2, 0x05, 0, 0}, {0x65C3, 0x05, 0, 0},
{0x65C4, 0x05, 0, 0}, {0x65C5, 0x05, 0, 0}, {0x6E1C, 0x00, 0, 0},
{0x6E1D, 0x00, 0, 0}, {0x6E25, 0x00, 0, 0}, {0x6E38, 0x03, 0, 0},
{0x895C, 0x01, 0, 0}, {0x895D, 0x00, 0, 0}, {0x8966, 0x00, 0, 0},
{0x8967, 0x4E, 0, 0}, {0x896A, 0x00, 0, 0}, {0x896B, 0x24, 0, 0},
{0x896F, 0x34, 0, 0}, {0x8976, 0x00, 0, 0}, {0x8977, 0x00, 0, 0},
{0x9004, 0x1F, 0, 0}, {0x9200, 0xB7, 0, 0}, {0x9201, 0x34, 0, 0},
{0x9202, 0xB7, 0, 0}, {0x9203, 0x36, 0, 0}, {0x9204, 0xB7, 0, 0},
{0x9205, 0x37, 0, 0}, {0x9206, 0xB7, 0, 0}, {0x9207, 0x38, 0, 0},
{0x9208, 0xB7, 0, 0}, {0x9209, 0x39, 0, 0}, {0x920A, 0xB7, 0, 0},
{0x920B, 0x3A, 0, 0}, {0x920C, 0xB7, 0, 0}, {0x920D, 0x3C, 0, 0},
{0x920E, 0xB7, 0, 0}, {0x920F, 0x3D, 0, 0}, {0x9210, 0xB7, 0, 0},
{0x9211, 0x3E, 0, 0}, {0x9212, 0xB7, 0, 0}, {0x9213, 0x3F, 0, 0},
{0x9214, 0xF6, 0, 0}, {0x9215, 0x13, 0, 0}, {0x9216, 0xF6, 0, 0},
{0x9217, 0x34, 0, 0}, {0x9218, 0xF4, 0, 0}, {0x9219, 0xA7, 0, 0},
{0x921A, 0xF4, 0, 0}, {0x921B, 0xAA, 0, 0}, {0x921C, 0xF4, 0, 0},
{0x921D, 0xAD, 0, 0}, {0x921E, 0xF4, 0, 0}, {0x921F, 0xB0, 0, 0},
{0x9220, 0xF4, 0, 0}, {0x9221, 0xB3, 0, 0}, {0x9222, 0x85, 0, 0},
{0x9223, 0x77, 0, 0}, {0x9224, 0xC4, 0, 0}, {0x9225, 0x4B, 0, 0},
{0x9226, 0xC4, 0, 0}, {0x9227, 0x4C, 0, 0}, {0x9228, 0xC4, 0, 0},
{0x9229, 0x4D, 0, 0}, {0x922A, 0xF5, 0, 0}, {0x922B, 0x5E, 0, 0},
{0x922C, 0xF5, 0, 0}, {0x922D, 0x5F, 0, 0}, {0x922E, 0xF5, 0, 0},
{0x922F, 0x64, 0, 0}, {0x9230, 0xF5, 0, 0}, {0x9231, 0x65, 0, 0},
{0x9232, 0xF5, 0, 0}, {0x9233, 0x6A, 0, 0}, {0x9234, 0xF5, 0, 0},
{0x9235, 0x6B, 0, 0}, {0x9236, 0xF5, 0, 0}, {0x9237, 0x70, 0, 0},
{0x9238, 0xF5, 0, 0}, {0x9239, 0x71, 0, 0}, {0x923A, 0xF5, 0, 0},
{0x923B, 0x76, 0, 0}, {0x923C, 0xF5, 0, 0}, {0x923D, 0x77, 0, 0},
{0x9810, 0x14, 0, 0}, {0x9814, 0x14, 0, 0}, {0xC020, 0x00, 0, 0},
{0xC026, 0x00, 0, 0}, {0xC027, 0x00, 0, 0}, {0xC448, 0x01, 0, 0},
{0xC44F, 0x01, 0, 0}, {0xC450, 0x00, 0, 0}, {0xC451, 0x00, 0, 0},
{0xC452, 0x01, 0, 0}, {0xC455, 0x00, 0, 0}, {0xE186, 0x36, 0, 0},
{0xE206, 0x35, 0, 0}, {0xE226, 0x33, 0, 0}, {0xE266, 0x34, 0, 0},
{0xE2A6, 0x31, 0, 0}, {0xE2C6, 0x37, 0, 0}, {0xE2E6, 0x32, 0, 0},



{0xA569, 0x06, 0, 0}, {0xA56A, 0x13, 0, 0}, {0xA56B, 0x13, 0, 0},
{0xA56C, 0x01, 0, 0}, {0xA678, 0x00, 0, 0}, {0xA679, 0x20, 0, 0},
{0xA812, 0x00, 0, 0}, {0xA813, 0x3F, 0, 0}, {0xA814, 0x3F, 0, 0},
{0xA830, 0x68, 0, 0}, {0xA831, 0x56, 0, 0}, {0xA832, 0x2B, 0, 0},
{0xA833, 0x55, 0, 0}, {0xA834, 0x55, 0, 0}, {0xA835, 0x16, 0, 0},
{0xA837, 0x51, 0, 0}, {0xA838, 0x34, 0, 0}, {0xA854, 0x4F, 0, 0},
{0xA855, 0x48, 0, 0}, {0xA856, 0x45, 0, 0}, {0xA857, 0x02, 0, 0},
{0xA85A, 0x23, 0, 0}, {0xA85B, 0x16, 0, 0}, {0xA85C, 0x12, 0, 0},
{0xA85D, 0x02, 0, 0}, {0xAA55, 0x00, 0, 0}, {0xAA56, 0x01, 0, 0},
{0xAA57, 0x30, 0, 0}, {0xAA58, 0x01, 0, 0}, {0xAA59, 0x30, 0, 0},
{0xAC72, 0x01, 0, 0}, {0xAC73, 0x26, 0, 0}, {0xAC74, 0x01, 0, 0},
{0xAC75, 0x26, 0, 0}, {0xAC76, 0x00, 0, 0}, {0xAC77, 0xC4, 0, 0},
{0xAE09, 0xFF, 0, 0}, {0xAE0A, 0xFF, 0, 0}, {0xAE12, 0x58, 0, 0},
{0xAE13, 0x58, 0, 0}, {0xAE15, 0x10, 0, 0}, {0xAE16, 0x10, 0, 0},
{0xAF05, 0x48, 0, 0}, {0xB069, 0x02, 0, 0}, {0xEA4B, 0x00, 0, 0},
{0xEA4C, 0x00, 0, 0}, {0xEA4D, 0x00, 0, 0}, {0xEA4E, 0x00, 0, 0}

};

static const struct reg_value arcimx616_setting_3820x2464[] = {

/* Timing settings. */
{0x0112, 0x0A, 0, 0}, {0x0113, 0x0A, 0, 0}, {0x0114, 0x03, 0, 0},
{0x0342, 0x0E, 0, 0}, {0x0343, 0xB8, 0, 0}, {0x0340, 0x1C, 0, 0},
{0x0341, 0xE4, 0, 0}, {0x0344, 0x00, 0, 0}, {0x0345, 0x00, 0, 0},
{0x0346, 0x00, 0, 0}, {0x0347, 0x00, 0, 0}, {0x0348, 0x19, 0, 0},
{0x0349, 0x9F, 0, 0}, {0x034A, 0x13, 0, 0}, {0x034B, 0x3F, 0, 0},
{0x0220, 0x62, 0, 0}, {0x0222, 0x01, 0, 0}, {0x0900, 0x01, 0, 0},
{0x0901, 0x22, 0, 0}, {0x0902, 0x08, 0, 0}, {0x3140, 0x00, 0, 0},
{0x3246, 0x81, 0, 0}, {0x3247, 0x81, 0, 0}, {0x0401, 0x00, 0, 0},
{0x0404, 0x00, 0, 0}, {0x0405, 0x10, 0, 0}, {0x0408, 0x00, 0, 0},
{0x0409, 0x00, 0, 0}, {0x040A, 0x00, 0, 0}, {0x040B, 0x00, 0, 0},
{0x040C, 0x0C, 0, 0}, {0x040D, 0xC0, 0, 0}, {0x040E, 0x09, 0, 0},
{0x040F, 0xA0, 0, 0}, {0x034C, 0x0C, 0, 0}, {0x034D, 0xC0, 0, 0},
{0x034E, 0x09, 0, 0}, {0x034F, 0xA0, 0, 0}, {0x0301, 0x05, 0, 0},
{0x0303, 0x02, 0, 0}, {0x0305, 0x04, 0, 0}, {0x0306, 0x01, 0, 0},
{0x0307, 0x68, 0, 0}, {0x030B, 0x01, 0, 0}, {0x030D, 0x04, 0, 0},
{0x030E, 0x01, 0, 0}, {0x030F, 0x72, 0, 0}, {0x0310, 0x01, 0, 0},
{0x3620, 0x00, 0, 0}, {0x3621, 0x00, 0, 0}, {0x3C12, 0x56, 0, 0},
{0x3C13, 0x52, 0, 0}, {0x3C14, 0x3E, 0, 0}, {0x3F0C, 0x00, 0, 0},
{0x3F14, 0x01, 0, 0}, {0x3F80, 0x00, 0, 0}, {0x3F81, 0xA0, 0, 0},
{0x3F8C, 0x00, 0, 0}, {0x3F8D, 0x00, 0, 0}, {0x3FFC, 0x00, 0, 0},
{0x3FFD, 0x1E, 0, 0}, {0x3FFE, 0x00, 0, 0}, {0x3FFF, 0xDC, 0, 0},
{0x0202, 0x1C, 0, 0}, {0x0203, 0xB4, 0, 0}, {0x0224, 0x01, 0, 0},
{0x0225, 0xF4, 0, 0}, {0x3FE0, 0x01, 0, 0}, {0x3FE1, 0xF4, 0, 0}, 
{0x0204, 0x00, 0, 0}, {0x0205, 0x70, 0, 0}, {0x0216, 0x00, 0, 0},
{0x0217, 0x70, 0, 0}, {0x0218, 0x01, 0, 0}, {0x0219, 0x00, 0, 0},
{0x020E, 0x01, 0, 0}, {0x020F, 0x00, 0, 0}, {0x0210, 0x01, 0, 0},
{0x0211, 0x00, 0, 0}, {0x0212, 0x01, 0, 0}, {0x0213, 0x00, 0, 0},
{0x0214, 0x01, 0, 0}, {0x0215, 0x00, 0, 0}, {0x3FE2, 0x00, 0, 0},
{0x3FE3, 0x70, 0, 0}, {0x3FE4, 0x01, 0, 0}, {0x3FE5, 0x00, 0, 0},
{0x3E20, 0x02, 0, 0}, {0x3E3B, 0x00, 0, 0}, {0x4034, 0x00, 0, 0},
{0x4035, 0xCC, 0, 0}
};

static const struct reg_value arcimx616_setting_1640x1232_binned[] = {
{0x0112, 0x0A, 0, 0}, {0x0113, 0x0A, 0, 0}, {0x0114, 0x03, 0, 0},
{0x0342, 0x08, 0, 0}, {0x0343, 0xC8, 0, 0}, {0x0340, 0x32, 0, 0},
{0x0341, 0x0A, 0, 0}, {0x0344, 0x00, 0, 0}, {0x0345, 0x00, 0, 0},
{0x0346, 0x00, 0, 0}, {0x0347, 0x00, 0, 0}, {0x0348, 0x19, 0, 0},
{0x0349, 0x9F, 0, 0}, {0x034A, 0x13, 0, 0}, {0x034B, 0x3F, 0, 0},
{0x0220, 0x62, 0, 0}, {0x0222, 0x01, 0, 0}, {0x0900, 0x01, 0, 0},
{0x0901, 0x44, 0, 0}, {0x0902, 0x08, 0, 0}, {0x3140, 0x00, 0, 0},
{0x3246, 0x89, 0, 0}, {0x3247, 0x89, 0, 0}, {0x0401, 0x00, 0, 0},
{0x0404, 0x00, 0, 0}, {0x0405, 0x10, 0, 0}, {0x0408, 0x00, 0, 0},
{0x0409, 0x00, 0, 0}, {0x040A, 0x00, 0, 0}, {0x040B, 0x00, 0, 0},
{0x040C, 0x06, 0, 0}, {0x040D, 0x68, 0, 0}, {0x040E, 0x04, 0, 0},
{0x040F, 0xD0, 0, 0}, {0x034C, 0x06, 0, 0}, {0x034D, 0x68, 0, 0},
{0x034E, 0x04, 0, 0}, {0x034F, 0xD0, 0, 0}, {0x0301, 0x05, 0, 0},
{0x0303, 0x02, 0, 0}, {0x0305, 0x04, 0, 0}, {0x0306, 0x01, 0, 0},
{0x0307, 0x68, 0, 0}, {0x030B, 0x01, 0, 0}, {0x030D, 0x04, 0, 0},
{0x030E, 0x01, 0, 0}, {0x030F, 0x72, 0, 0}, {0x0310, 0x01, 0, 0},
{0x3620, 0x00, 0, 0}, {0x3621, 0x00, 0, 0}, {0x3C12, 0x3E, 0, 0},
{0x3C13, 0x3A, 0, 0}, {0x3C14, 0x22, 0, 0}, {0x3F0C, 0x00, 0, 0},
{0x3F14, 0x00, 0, 0}, {0x3F80, 0x00, 0, 0}, {0x3F81, 0x00, 0, 0},
{0x3F8C, 0x00, 0, 0}, {0x3F8D, 0x00, 0, 0}, {0x3FFC, 0x00, 0, 0},
{0x3FFD, 0x55, 0, 0}, {0x3FFE, 0x00, 0, 0}, {0x3FFF, 0x78, 0, 0},
{0x0202, 0x31, 0, 0}, {0x0203, 0xDA, 0, 0}, {0x0224, 0x01, 0, 0},
{0x0225, 0xF4, 0, 0}, {0x3FE0, 0x01, 0, 0}, {0x3FE1, 0xF4, 0, 0},
{0x0204, 0x00, 0, 0}, {0x0205, 0x70, 0, 0}, {0x0216, 0x00, 0, 0},
{0x0217, 0x70, 0, 0}, {0x0218, 0x01, 0, 0}, {0x0219, 0x00, 0, 0},
{0x020E, 0x01, 0, 0}, {0x020F, 0x00, 0, 0}, {0x0210, 0x01, 0, 0},
{0x0211, 0x00, 0, 0}, {0x0212, 0x01, 0, 0}, {0x0213, 0x00, 0, 0},
{0x0214, 0x01, 0, 0}, {0x0215, 0x00, 0, 0}, {0x3FE2, 0x00, 0, 0},
{0x3FE3, 0x70, 0, 0}, {0x3FE4, 0x01, 0, 0}, {0x3FE5, 0x00, 0, 0},
{0x3E20, 0x01, 0, 0}, {0x3E3B, 0x00, 0, 0}, {0x4034, 0x00, 0, 0},
{0x4035, 0xCC, 0, 0}
};

static const struct arcimx616_mode_info arcimx616_mode_data_3820x2464 = {
	2688, 0, 2688, 0, arcimx616_setting_3820x2464, ARRAY_SIZE(arcimx616_setting_3820x2464)
};

static const struct arcimx616_mode_info arcimx616_mode_data_1640x1232_binned = {
	1920, 0, 1520, 0, arcimx616_setting_1640x1232_binned, ARRAY_SIZE(arcimx616_setting_1640x1232_binned)
};

static const s64 link_freq_menu_items[] = {
	ARCIMX616_LINK_FREQUENCY
};

static int arcimx616_write_reg_dual(struct arcimx616_dev *sensor, u16 reg, u8 val)
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

static int arcimx616_write_reg(struct arcimx616_dev *sensor, u16 reg, u8 val)
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

static int arcimx616_read_reg(struct arcimx616_dev *sensor, u16 reg, u8 *val)
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

static int arcimx616_read_reg16(struct arcimx616_dev *sensor, u16 reg, u16 *val)
{
	u8 hi, lo;
	int ret;

	ret = arcimx616_read_reg(sensor, reg, &hi);
	if (ret)
		return ret;
	ret = arcimx616_read_reg(sensor, reg + 1, &lo);
	if (ret)
		return ret;

	*val = ((u16)hi << 8) | (u16)lo;
	return 0;
}

#define AW9106C_ADDR  0x5B
#define ARCIMX616_I2C_ADDR_TEMP_P3T1084   0x48
#define ARCIMX616_I2C_ADDR_EEPROM_CAT24   0x51

static int arcimx616_nvmem_match(struct device *dev, const void *data)
{
	const char* expected_prefix = data;
	const char* name = dev_name(dev);
	int result;

	if(strlen(name) < strlen(expected_prefix)) {
		return 0;
	}

	result = memcmp(expected_prefix, name, strlen(expected_prefix)) == 0;
	return result;
}


static int arcimx616_copy_eeprom_data(struct arcimx616_dev *sensor, struct eeprom_access_info *access_info, bool from_eeprom)
{
	int ret = 0;

	arcimx616_set_power(sensor, true);
	ret = arcimx616_ensure_eeprom_client(sensor);
	if (ret) {
		dev_err(&sensor->i2c_client->dev,
			"failed to ensure eeprom client: %d\n", ret);
	}
	ret = 0;


	/* Match the synthetic nvmem provider name. */
	bool is_left = sensor->is_left_sensor;
	struct nvmem_device* nvmem_dev;
	int retries = 5;
	
	while (retries--) {
		nvmem_dev = nvmem_device_find(is_left ? ARCIMX616_NVMEM_NAME_LEFT : ARCIMX616_NVMEM_NAME_RIGHT, arcimx616_nvmem_match);
		if (!IS_ERR(nvmem_dev))
			break;
			
		if (retries > 0) {
			msleep(10);
			continue;
		}
		
		dev_err(&sensor->i2c_client->dev, "Failed to find a matching nvmem device after 5 retries\n");
		ret = PTR_ERR(nvmem_dev);
		goto out_power;
	}

	/* Validate the AT24 size. */
	size_t nvmem_size = nvmem_dev_size(nvmem_dev);
	if(nvmem_size != ARCIMX616_EEPROM_SIZE)
	{
		dev_err(&sensor->i2c_client->dev, "Unexpected EEPROM data size (nvmem cell size is %zu but expected %d\n", nvmem_size, ARCIMX616_EEPROM_SIZE);
		nvmem_device_put(nvmem_dev);
		ret = -EINVAL;
		goto out_power;
	}

	u32 eeprom_offset = access_info->offset;

	if(eeprom_offset >= ARCIMX616_EEPROM_SIZE || (eeprom_offset + access_info->size) > ARCIMX616_EEPROM_SIZE)
	{
		dev_err(&sensor->i2c_client->dev, "Invalid EEPROM access range [0x%x, 0x%x]. The available EEPROM size is 0x%x. \n",
			eeprom_offset, eeprom_offset+access_info->size, ARCIMX616_EEPROM_SIZE);
		nvmem_device_put(nvmem_dev);
		ret = -EINVAL;
		goto out_power;
	}

	/* Manufacturer range. */
	const u32 protected_range_end = 0x800;
	if(!from_eeprom && (eeprom_offset < protected_range_end))
	{
		dev_err(&sensor->i2c_client->dev, "Invalid EEPROM access range [0x%x, 0x%x] would overwrite the protected range [0x0, 0x%x] \n",
			eeprom_offset, eeprom_offset+access_info->size, protected_range_end);
		nvmem_device_put(nvmem_dev);
		ret = -EINVAL;
		goto out_power;
	}

	
	if(from_eeprom) {
		int nb_read = nvmem_device_read(nvmem_dev, eeprom_offset, access_info->size, sensor->eeprom_buffer);
		if(nb_read != access_info->size)
		{
			dev_err(&sensor->i2c_client->dev, "Failed EEPROM read (ret=%d)\n", nb_read);
			ret = -ENXIO;
			goto out_nvmem;
		}

		if(copy_to_user(access_info->buffer, sensor->eeprom_buffer, access_info->size) != 0)
		{
			dev_err(&sensor->i2c_client->dev, "Failed to copy data to user pointer \n");
			ret = -EFAULT;
			goto out_nvmem;
		}
	}
	else {
		if(copy_from_user(sensor->eeprom_buffer, access_info->buffer, access_info->size) != 0)
		{
			dev_err(&sensor->i2c_client->dev, "Failed to copy data from user pointer \n");
			ret = -EFAULT;
			goto out_nvmem;
		}

		int nb_read = nvmem_device_write(nvmem_dev, eeprom_offset, access_info->size, sensor->eeprom_buffer);
		if(nb_read != access_info->size)
		{
			dev_err(&sensor->i2c_client->dev, "Failed EEPROM write (ret=%d)\n", nb_read);
			ret = -ENXIO;
			goto out_nvmem;
		}
	}

out_nvmem:
	nvmem_device_put(nvmem_dev);

out_power:
	arcimx616_set_power(sensor, false);
	return ret;
}

static int arcimx616_load_regs(struct arcimx616_dev *sensor,
			    const struct arcimx616_mode_info *mode)
{
	{
		const struct reg_value *regs = arcimx616_global_settings;
		unsigned int i;
		u32 delay_ms;
		u16 reg_addr;
		u8 mask, val;
		int ret = 0;

		for (i = 0; i < ARRAY_SIZE(arcimx616_global_settings); ++i, ++regs) {
			delay_ms = regs->delay_ms;
			reg_addr = regs->reg_addr;
			val = regs->val;
			mask = regs->mask;

			ret = arcimx616_write_reg(sensor, reg_addr, val);
			if (ret)
				break;

			if (delay_ms) {
				usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
			}
		}
	}

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

			ret = arcimx616_write_reg(sensor, reg_addr, val);
			if (ret)
				break;

			if (delay_ms) {
				usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
			}
		}
	}
	return 0;
}

/* restore the last set video mode after chip power-on */
static int arcimx616_restore_mode(struct arcimx616_dev *sensor)
{
	int ret;
	u8 val;
	struct i2c_client *client = sensor->i2c_client;

	/* first load the initial register values */
	ret = arcimx616_load_regs(sensor, (sensor->current_mode));
	if (ret < 0)
		return ret;

	ret = arcimx616_read_reg(sensor, 0x114, &val);
	if (ret < 0 || val != 0x03) {
		dev_err(&client->dev, "verify failed val=%.2x ret=%d\n", val, ret);
		return ret;
	}

	return 0;
}

static int arcimx616_set_power_on(struct arcimx616_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable power\n",
			__func__);
		return ret;
	}

	/* Only the reset owner controls the shared rails. */
	if(sensor->reset_gpio == NULL) {
		return 0;
	}

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	/* Reset low for at least 1 ms. */
	usleep_range(1000, 1500);

	ret = regulator_bulk_enable(arcimx616_NUM_SUPPLIES,
		sensor->supplies);

	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		goto power_off;
	}



	clk_set_rate(sensor->xclk, 24000000);

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto regulator_off;
	}

	clk_set_rate(sensor->xclk2, 24000000);

	ret = clk_prepare_enable(sensor->xclk2);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto xclk_off;
	}

	/* Rail settle. */
	usleep_range(5000, 5500);

	/* Release reset. */
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);

	/* Sensor command delay. */
	usleep_range(13000, 13500);

	return 0;

xclk_off:
	clk_disable_unprepare(sensor->xclk);
regulator_off:
	regulator_bulk_disable(arcimx616_NUM_SUPPLIES, sensor->supplies);
power_off:
	pm_runtime_put_sync(&client->dev);
	return ret;
}

static void arcimx616_set_power_off(struct arcimx616_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;

	/*
	 * Sensors that don't own the reset gpio never enabled regulators
	 * or clocks in set_power_on, so only balance the pm_runtime ref.
	 */
	if (sensor->reset_gpio == NULL) {
		pm_runtime_put_sync(&client->dev);
		return;
	}

	regulator_bulk_disable(arcimx616_NUM_SUPPLIES, sensor->supplies);

	clk_disable_unprepare(sensor->xclk);
	clk_disable_unprepare(sensor->xclk2);

	dev_dbg(&client->dev, "%s: Powered off\n",
			__func__);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);

	/* Let rails settle. */
	usleep_range(1000, 1500);

	pm_runtime_put_sync(&client->dev);
}

static int arcimx616_set_power(struct arcimx616_dev *sensor, bool on)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	bool was_on = arcimx616_power_on_count > 0;

	if (on) {
		arcimx616_power_on_count++;
	} else {
		if (arcimx616_power_on_count > 0)
			arcimx616_power_on_count--;
		else
			return 0;
	}

	if (on && !was_on) {
		for (int i = 0; i < g_arcimx616_sensor_count; i++) {
			ret = arcimx616_set_power_on(g_arcimx616_sensors[i]);
			if (ret)
				return ret;
		}

		for (int i = 0; i < g_arcimx616_sensor_count; i++) {
			ret = arcimx616_restore_mode(g_arcimx616_sensors[i]);
			if (ret)
				dev_err(&client->dev, "Error from arcimx616_restore_mode");
		}
	} else {
		if (arcimx616_power_on_count == 0) {
			for (int i = 0; i < g_arcimx616_sensor_count; i++)
				arcimx616_set_power_off(g_arcimx616_sensors[i]);
		}
	}

	return 0;
}

static int arcimx616_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct arcimx616_dev *sensor =
			container_of(ctrl->handler, struct arcimx616_dev, ctrl_handler);
	int ret = 0;

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(&sensor->i2c_client->dev))
		return 0;

	switch (ctrl->id) {

	case V4L2_CID_BRIGHTNESS:
		ret = arcimx616_write_reg(sensor, ctrl->val & 0xffff, ctrl->val >> 16 & 0xff);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&sensor->i2c_client->dev);

	return ret;
}

static int arcimx616_get_ctrl(struct v4l2_ctrl *ctrl)
{
	u8 temperature;
	struct arcimx616_dev *sensor =
			container_of(ctrl->handler, struct arcimx616_dev, ctrl_handler);

	int ret = arcimx616_read_reg(sensor, IMX_REG_TEMP_SEN_OUT, &temperature);
	if(ret != 0) {
		dev_err(&sensor->i2c_client->dev, "Failed to read temperature\n");
		return ret;
	}
	ctrl->val = temperature;
	ctrl->has_changed = 1;
	ctrl->cur.val = temperature;
	return 0;
}

static const struct v4l2_ctrl_ops arcimx616_ctrl_ops = {
	.s_ctrl = arcimx616_set_ctrl,
};

static const struct v4l2_ctrl_ops arcimx616_ctrl_read_ops = {
	.g_volatile_ctrl = arcimx616_get_ctrl,
};

static const struct v4l2_ctrl_config arcimx616_ctrl_temperature = {
	.ops = &arcimx616_ctrl_read_ops,
	.id = V4L2_CID_TEMPERATURE,
	.name = "Temperature",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.flags = V4L2_CTRL_FLAG_VOLATILE,
	.min = 0,
	.max = 0xffffffff,
	.step = 1,
};

static int arcimx616_init_controls(struct arcimx616_dev *sensor)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	int ret;

	ctrl_hdlr = &sensor->ctrl_handler;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 2);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &sensor->mutex;

	/* hack control for custom i2c writes */
	v4l2_ctrl_new_std(ctrl_hdlr, &arcimx616_ctrl_ops, V4L2_CID_BRIGHTNESS, 0, 0xffffff, 1, 0);
	
	int size = ARRAY_SIZE(link_freq_menu_items);

	struct v4l2_ctrl *link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &arcimx616_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   size - 1, 0,
						   link_freq_menu_items);

	if (link_freq)
		link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	
	/* Temperature control. */
	v4l2_ctrl_new_custom(ctrl_hdlr, &arcimx616_ctrl_temperature, NULL);

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	sensor->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static int arcimx616_s_power(struct v4l2_subdev *sd, int on)
{
	struct arcimx616_dev *sensor = to_arcimx616_dev(sd);
	int ret = 0;

	struct i2c_client *client = sensor->i2c_client;
	
	dev_dbg(&client->dev, "calling s_power with on:%d",on);
	mutex_lock(&sensor->lock);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	
	ret = arcimx616_set_power(sensor, !!on);
	mutex_unlock(&sensor->lock);
	return 0;
}

static int arcimx616_check_chip_id(struct arcimx616_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;
	u16 chip_id;

	ret = arcimx616_read_reg16(sensor, arcimx616_REG_CHIP_ID, &chip_id);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read chip identifier\n",
			__func__);
	}

	if (chip_id != 0x616) {
		dev_err(&client->dev, "%s: wrong chip identifier, expected 0x616, got 0x%x\n",
			__func__, chip_id);
		ret = -ENXIO;
	}
	else { 
		dev_dbg(&client->dev, "%s: Correct chip identifier, expected 0x616, got 0x%x\n",
			__func__, chip_id);
	}
	return ret;
}

struct arcimx616_probe_result {
	bool sensor0_ok;
	bool sensor1_ok;
	bool led_ok;
	bool strobe_active;
	bool custom_peripheral_ok;
};

static bool arcimx616_i2c_client_acks(struct i2c_client *client)
{
	struct i2c_msg msg;
	u8 dummy;

	if (!client)
		return false;

	msg.addr  = client->addr;
	msg.flags = client->flags | I2C_M_RD;
	msg.buf   = &dummy;
	msg.len   = 1;

	return i2c_transfer(client->adapter, &msg, 1) == 1;
}

static bool arcimx616_probe_custom_peripheral(struct arcimx616_dev *sensor)
{
	bool temp_ok   = arcimx616_i2c_client_acks(g_temp_i2c_client);
	bool eeprom_ok = arcimx616_i2c_client_acks(g_eeprom_i2c_client);

	if (temp_ok || eeprom_ok) {
		dev_info(&sensor->i2c_client->dev,
			 "Custom peripheral detected on I2C (temp=%d eeprom=%d)\n",
			 temp_ok, eeprom_ok);
		return true;
	}
	return false;
}

static void arcimx616_probe_sources(struct arcimx616_dev *sensor,
				    struct arcimx616_probe_result *out)
{
	int sensor0Connected = -ENODEV;
	int sensor1Connected = -ENODEV;

	if (g_arcimx616_sensor_count > 0 && g_arcimx616_sensors[0])
		sensor0Connected = arcimx616_check_chip_id(g_arcimx616_sensors[0]);
	if (g_arcimx616_sensor_count > 1 && g_arcimx616_sensors[1])
		sensor1Connected = arcimx616_check_chip_id(g_arcimx616_sensors[1]);

	out->sensor0_ok = (sensor0Connected == 0);
	out->sensor1_ok = (sensor1Connected == 0);
	out->led_ok     = (arcimx616_aw9106_check_chip_id(sensor) == 0);
	out->strobe_active = false;
	out->custom_peripheral_ok = false;

	bool any_connected = out->led_ok || out->sensor0_ok || out->sensor1_ok;

	if (any_connected) {
		if (sensor->mux_select) {
			gpiod_set_value_cansleep(sensor->mux_select, 0);
		}
	} else {
		if (sensor->mux_select) {
			gpiod_set_value_cansleep(sensor->mux_select, 1);
		}

		/*
		 * Custom peripheral check before the strobe loopback path.
		 */
		if (arcimx616_probe_custom_peripheral(sensor)) {
			out->custom_peripheral_ok = true;
		} else if (sensor->strobe) {
			bool reset_high = sensor->reset_gpio &&
					  gpiod_get_value(sensor->reset_gpio) == 1;
			if (reset_high && gpiod_get_value(sensor->strobe)) {
				out->strobe_active = true;
			}
		}
	}
}

static int arcimx616_s_stream(struct v4l2_subdev *sd, int enable)
{

	struct arcimx616_dev *sensor = to_arcimx616_dev(sd);
	struct i2c_client *client = sensor->i2c_client;

			u8 enabled_check = 0;

	int ret = 0;

	ret = arcimx616_write_reg(sensor, 0x0100, !!enable);

	ret = arcimx616_read_reg(sensor, 0x0100, &enabled_check);
	if (ret) {
		dev_err(&client->dev, "%s: failed to read STREAM_ON\n",
			__func__);
	}

	if (!!enable == 1 && enabled_check != 1) {
		dev_err(&client->dev, "%s: wrong STREAM_ON, expected 0x%x, got 0x%x\n",
			__func__, enable, enabled_check);
		ret = -ENXIO;
	}

	/* Apply cached LED state. */
	if (enable)
		arcimx616_led_apply_effective();

	/* Keep the rest of the pipeline bring-up tolerant. */
	return 0;
}

static int arcimx616_subscribe_event(struct v4l2_subdev *sd,
                                  struct v4l2_fh *fh,
                                  struct v4l2_event_subscription *sub)
{
    switch (sub->type) {
    case V4L2_EVENT_SENSOR_CONNECTED:
    case V4L2_EVENT_SENSOR_DISCONNECTED:
        return v4l2_event_subscribe(fh, sub, 0, NULL);
    default:
        return -EINVAL;
    }
}

static long arcimx616_private_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct arcimx616_dev *sensor = to_arcimx616_dev(sd);
	int ret = 0;
	u32 *a = arg;

	switch(cmd)
	{
		case VIDIOC_PRIV_G_CONNECTED:
			if (sensor->detect_gpio &&
			    gpiod_get_value(sensor->detect_gpio) != 0)
				sensor->connected = false;
			*a = sensor->connected;
			break;
		case VIDIOC_PRIV_S_DUAL_REG:
			{
				u32 val = (uintptr_t)arg;
				ret = arcimx616_write_reg_dual(sensor, val & 0xffff, (val >> 16) & 0xff);
			}
			break;
		case VIDIOC_PRIV_G_EEPROM_DATA:
			{
				ret = arcimx616_copy_eeprom_data(sensor, arg, true /* from_eeprom */);
			}
			break;
		case VIDIOC_PRIV_S_EEPROM_DATA:
		{
			ret = arcimx616_copy_eeprom_data(sensor, arg, false /* from_eeprom */);
		}
		break;
		default:
			ret = -ENOTTY;
			break;
	}

	return ret;
}

static const struct v4l2_subdev_core_ops arcimx616_core_ops = {
	.s_power = arcimx616_s_power,
	.subscribe_event = arcimx616_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl = arcimx616_private_ioctl,
};

static const struct v4l2_subdev_video_ops arcimx616_video_ops = {
	.s_stream = arcimx616_s_stream,
};

static int arcimx616_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	code->code = MEDIA_BUS_FMT_Y10_1X10;
	return 0;
}

static int arcimx616_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct arcimx616_dev *sensor = to_arcimx616_dev(sd);

	mutex_lock(&sensor->lock);

	if(fmt->format.width == 3264 && fmt->format.height == 2464) 
	{
		sensor->current_mode = &arcimx616_mode_data_3820x2464;	
	}
	else if (fmt->format.width == 1640 && fmt->format.height == 1232) 
	{
		sensor->current_mode = &arcimx616_mode_data_1640x1232_binned;
	}
	else {
		/* Other resolutions use manual crop changes. */
		sensor->current_mode = &arcimx616_mode_data_3820x2464;	
	}

	sensor->fmt.width = fmt->format.width;
	sensor->fmt.height = fmt->format.height;


	mutex_unlock(&sensor->lock);

	return 0;
}

static int arcimx616_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *format)
{
	struct arcimx616_dev *sensor = to_arcimx616_dev(sd);

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	format->format = sensor->fmt;

	mutex_unlock(&sensor->lock);


	return 0;
}

static const struct v4l2_subdev_pad_ops arcimx616_pad_ops = {
	.enum_mbus_code = arcimx616_enum_mbus_code,
	.get_fmt = arcimx616_get_fmt,
	.set_fmt = arcimx616_set_fmt

};

static const struct v4l2_subdev_ops arcimx616_subdev_ops = {
	.core = &arcimx616_core_ops,
	.video = &arcimx616_video_ops,
	.pad = &arcimx616_pad_ops,
};

static int arcimx616_get_regulators(struct arcimx616_dev *sensor)
{
	int i;

	for (i = 0; i < arcimx616_NUM_SUPPLIES; i++)
		sensor->supplies[i].supply = arcimx616_supply_name[i];

	return devm_regulator_bulk_get(&sensor->i2c_client->dev,
				       arcimx616_NUM_SUPPLIES,
				       sensor->supplies);
}



static void arcimx616_reset_debounce_state(struct arcimx616_dev *sensor)
{
	sensor->debounce_cnt = 0;
	sensor->debounce_sensor0_ok = false;
	sensor->debounce_sensor1_ok = false;
	sensor->debounce_led_ok = false;
	sensor->debounce_settling = false;
}

static void arcimx616_connect_work_fn(struct work_struct *work)
{
	struct arcimx616_dev *sensor =
		container_of(to_delayed_work(work), struct arcimx616_dev, connect_work);

	const int debounce_num_connection_tests = 5;
	const int debounce_connection_tests_delay = HZ/10;
	const int debounce_settle_display_delay = HZ/2;
	struct arcimx616_probe_result cur;
	bool latch_accessory_power = false;

	if (!sensor->detect_gpio) {
		dev_dbg(&sensor->i2c_client->dev, "No detect GPIO found\n");
		return;
	}

	if (gpiod_get_value(sensor->detect_gpio) != 0) {
		dev_info(&sensor->i2c_client->dev, "Accessory canceled connect during debounce\n");
		goto halt_connection_work;
	}

	if (sensor->debounce_settling) {
		bool i2c_ever_seen = sensor->debounce_led_ok ||
				     sensor->debounce_sensor0_ok ||
				     sensor->debounce_sensor1_ok;

		/* Clear the probe blink before powering down. */
		arcimx616_led_clear_internal_override();
		arcimx616_set_power(sensor, false);

		if (i2c_ever_seen) {
			sensor->connected = true;
			struct v4l2_event ev = {0};
			ev.type = V4L2_EVENT_SENSOR_CONNECTED;
			v4l2_event_queue(sensor->sd.devnode, &ev);
		}
		goto halt_connection_work;
	}

	/* Pulse rails for each debounce probe. */
	arcimx616_set_power(sensor, true);
	if (sensor->mux_enable)
		gpiod_set_value_cansleep(sensor->mux_enable, 0);

	arcimx616_probe_sources(sensor, &cur);
	sensor->debounce_sensor0_ok |= cur.sensor0_ok;
	sensor->debounce_sensor1_ok |= cur.sensor1_ok;
	sensor->debounce_led_ok     |= cur.led_ok;

	/* Non-camera accessories latch rails until unplug. */
	if (cur.custom_peripheral_ok) {
		dev_info(&sensor->i2c_client->dev,
			 "Custom I2C peripheral detected; leaving power on until unplug\n");
		latch_accessory_power = true;
		goto halt_connection_work;
	}
	if (cur.strobe_active) {
		dev_info(&sensor->i2c_client->dev,
			 "Frunky loopback detected; leaving sensor power on until unplug\n");
		latch_accessory_power = true;
		goto halt_connection_work;
	}

	bool both_sensors_available = cur.sensor0_ok && cur.sensor1_ok;
	bool probe_window_done  = sensor->debounce_cnt >= debounce_num_connection_tests;

	if (!both_sensors_available && !probe_window_done) {
		arcimx616_set_power(sensor, false);
		sensor->debounce_cnt++;
		schedule_delayed_work(&sensor->connect_work, debounce_connection_tests_delay);
		return;
	}

	bool any_i2c = sensor->debounce_sensor0_ok ||
		       sensor->debounce_sensor1_ok ||
		       sensor->debounce_led_ok;

	if (!any_i2c) {
		arcimx616_set_power(sensor, false);
		goto halt_connection_work;
	}

	/* Probe blink: green = seen, red = missing. */
	{
		int left_idx  = g_arcimx616_sensors[0]->is_left_sensor ? 0 : 1;
		int right_idx = left_idx == 0 ? 1 : 0;
		bool left_ok  = (left_idx  == 0) ? sensor->debounce_sensor0_ok
						 : sensor->debounce_sensor1_ok;
		bool right_ok = (right_idx == 0) ? sensor->debounce_sensor0_ok
						 : sensor->debounce_sensor1_ok;

		if (sensor->debounce_led_ok) {
			struct arcimx616_led_override ov = {
				.left_enable = true,
				.right_enable = true,
				.left_brightness = 255,
				.right_brightness = 255,
				.left_r = left_ok ? 0 : 255,
				.left_g = left_ok ? 255 : 0,
				.left_b = 0,
				.right_r = right_ok ? 0 : 255,
				.right_g = right_ok ? 255 : 0,
				.right_b = 0,
				.effect = ARC_LED_EFFECT_BLINK,
				.fade_on_ms = 630,
				.full_on_ms = 315,
				.fade_off_ms = 630,
				.full_off_ms = 315,
			};

			arcimx616_led_set_internal_override(ARC_LED_OVERRIDE_PROBE, &ov);
		}
	}

	sensor->debounce_settling = true;
	schedule_delayed_work(&sensor->connect_work, debounce_settle_display_delay);
	return;

halt_connection_work:
	arcimx616_reset_debounce_state(sensor);
	spin_lock(&sensor->irq_lock);
	if (latch_accessory_power)
		sensor->accessory_power_latched = true;
	sensor->connect_work_scheduled = false;
	spin_unlock(&sensor->irq_lock);
}


static irqreturn_t gpio_handler(int irq, void *ident)
{
	struct arcimx616_dev *sensor = ident;
	int state = gpiod_get_value(sensor->detect_gpio);

	spin_lock(&sensor->irq_lock);

	if (state == 0) {
		if (!sensor->connected && !sensor->accessory_power_latched &&
		    !sensor->connect_work_scheduled) {
			sensor->connect_work_scheduled = true;
			schedule_delayed_work(&sensor->connect_work, HZ / 4);
		}
	} else {
		if (sensor->connect_work_scheduled) {
			spin_unlock(&sensor->irq_lock);
			cancel_delayed_work_sync(&sensor->connect_work);
			spin_lock(&sensor->irq_lock);
			sensor->connect_work_scheduled = false;
			sensor->accessory_power_latched = false;
			arcimx616_reset_debounce_state(sensor);
			dev_info(&sensor->i2c_client->dev, "Debounce: canceled connect.\n");
		}

		sensor->accessory_power_latched = false;
		spin_unlock(&sensor->irq_lock);
		arcimx616_power_off_all_sensors();
		spin_lock(&sensor->irq_lock);

		if (sensor->connected) {
			struct v4l2_event ev = {0};

			sensor->connected = false;
			ev.type = V4L2_EVENT_SENSOR_DISCONNECTED;
			v4l2_event_queue(sensor->sd.devnode, &ev);
			dev_info(&sensor->i2c_client->dev, "Accessory disconnected\n");
		}
	}

	spin_unlock(&sensor->irq_lock);

	return IRQ_HANDLED;
}

static int arcimx616_probe(struct i2c_client *client)
{
	bool added_global = false;
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct arcimx616_dev *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;
	

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	spin_lock_init(&sensor->irq_lock);
    INIT_DELAYED_WORK(&sensor->connect_work, arcimx616_connect_work_fn);

	sensor->i2c_client = client;

	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_Y10_1X10;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
	fmt->width = 2464;
	fmt->height = 2464;
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
		/* Shared with the sibling sensor. */
		dev_dbg(dev, "failed to get xclk\n");
	}
	sensor->xclk2 = devm_clk_get(dev, "xclk2");
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_LOW);
	sensor->detect_gpio = devm_gpiod_get_optional(dev, "detect",
						     GPIOD_IN);
	sensor->mux_enable = devm_gpiod_get_optional(dev, "mux-enable",
						     GPIOD_OUT_HIGH);
	sensor->mux_select = devm_gpiod_get_optional(dev, "mux-select",
						     GPIOD_OUT_HIGH);
	sensor->strobe = devm_gpiod_get_optional(dev, "strobe",
						     GPIOD_IN);
	if(sensor->detect_gpio) {
		dev_dbg(&client->dev, "detect GPIO found\n");
		sensor->irq = gpiod_to_irq(sensor->detect_gpio);
		ret = devm_request_threaded_irq(dev, sensor->irq, NULL, gpio_handler,
                                        IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                        "arcimx616-detect", sensor);
		if (ret) {
			dev_err(&client->dev, "failed to request IRQ for detect GPIO\n");
		}
		else {
			dev_dbg(&client->dev, "IRQ for detect GPIO requested successfully\n");
		}
	}
	else {
		dev_dbg(&client->dev, "no detect GPIO found\n");
	}

	struct v4l2_subdev *sd;
	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &arcimx616_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	ret = arcimx616_get_regulators(sensor);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	mutex_init(&sensor->mutex);
	
	/* Determine if this is left or right sensor based on device tree 'id' property */
	{
		const char *id;
		if (of_property_read_string(dev->of_node, "id", &id)) {
			dev_err(dev, "Failed to read 'id' property\n");
			ret = -EINVAL;
			goto entity_cleanup;
		}
		if (strcmp(id, "passthrough_left") == 0) {
			sensor->is_left_sensor = true;
			dev_dbg(dev, "Identified as left sensor\n");
		} else {
			sensor->is_left_sensor = false;
			dev_dbg(dev, "Identified as right sensor\n");
		}
	}

	/*
	 * Prefer DT for shared AW9106 attrs; otherwise use the detect owner.
	 */
	sensor->is_led_master =
		of_property_read_bool(dev->of_node, "arcturus,led-master") ||
		(sensor->detect_gpio != NULL);

	arcimx616_led_init_state(sensor);

	ret = arcimx616_init_controls(sensor);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto entity_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto free_ctrls;

	sensor->current_mode = &arcimx616_mode_data_3820x2464;

	{
		int idx = g_arcimx616_sensor_count;
		if (idx < ARCIMX616_MAX_SENSORS) {
			g_arcimx616_sensors[idx] = sensor;
			g_arcimx616_sensor_count++;
			added_global = true;

			/* Initialize dual and aw9106 i2c clients on first probe */
			if (g_arcimx616_sensor_count == 1 && !g_dual_i2c_client) {
				struct i2c_board_info dual_info = {
					I2C_BOARD_INFO("arcimx616-dual", 0x36)
				};
				g_dual_i2c_client = i2c_new_client_device(client->adapter, &dual_info);
				if (IS_ERR(g_dual_i2c_client)) {
					dev_err(&client->dev, "Failed to create dual i2c client: %ld\n", 
						PTR_ERR(g_dual_i2c_client));
					g_dual_i2c_client = NULL;
				} else {
					dev_dbg(&client->dev, "Created dual i2c client at address 0x36\n");
				}

				struct i2c_board_info aw9106_info = {
					I2C_BOARD_INFO("aw9106", AW9106C_ADDR)
				};
				g_aw9106_i2c_client = i2c_new_client_device(client->adapter, &aw9106_info);
				if (IS_ERR(g_aw9106_i2c_client)) {
					dev_err(&client->dev, "Failed to create AW9106 i2c client: %ld\n",
						PTR_ERR(g_aw9106_i2c_client));
					g_aw9106_i2c_client = NULL;
				} else {
					dev_dbg(&client->dev, "Created AW9106 i2c client at address 0x%02x\n", AW9106C_ADDR);
				}

				struct i2c_board_info temp_info = {
					I2C_BOARD_INFO("arcimx616-temp-probe", ARCIMX616_I2C_ADDR_TEMP_P3T1084)
				};
				g_temp_i2c_client = i2c_new_client_device(client->adapter, &temp_info);
				if (IS_ERR(g_temp_i2c_client)) {
					dev_err(&client->dev, "Failed to create temp-probe i2c client: %ld\n",
						PTR_ERR(g_temp_i2c_client));
					g_temp_i2c_client = NULL;
				} else {
					dev_dbg(&client->dev, "Created temp-probe i2c client at address 0x%02x\n",
						ARCIMX616_I2C_ADDR_TEMP_P3T1084);
				}

				struct i2c_board_info eeprom_info = {
					I2C_BOARD_INFO("arcimx616-eeprom-pr", ARCIMX616_I2C_ADDR_EEPROM_CAT24)
				};
				g_eeprom_i2c_client = i2c_new_client_device(client->adapter, &eeprom_info);
				if (IS_ERR(g_eeprom_i2c_client)) {
					dev_err(&client->dev, "Failed to create eeprom-probe i2c client: %ld\n",
						PTR_ERR(g_eeprom_i2c_client));
					g_eeprom_i2c_client = NULL;
				} else {
					dev_dbg(&client->dev, "Created eeprom-probe i2c client at address 0x%02x\n",
						ARCIMX616_I2C_ADDR_EEPROM_CAT24);
				}
			}
		} else {
			dev_warn(&client->dev, "arcimx616: Reached max sensor count!\n");
		}
	}

	/* Needs the sensor in the global array first; non-fatal on failure. */
	if (added_global) {
		ret = arcimx616_led_create_sysfs(sensor);
		if (ret) {
			dev_warn(&client->dev,
				 "failed to create LED sysfs attrs: %d\n", ret);
			ret = 0;
		}
	}

	/* Handle accessories already present at probe. */
	if (sensor->detect_gpio && gpiod_get_value(sensor->detect_gpio) == 0) {
		dev_dbg(&client->dev, "Accessory detected during probe, scheduling async connect\n");
		spin_lock(&sensor->irq_lock);
		if (!sensor->connect_work_scheduled) {
			sensor->connect_work_scheduled = true;
			schedule_delayed_work(&sensor->connect_work, HZ/4);
		}
		spin_unlock(&sensor->irq_lock);
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	return 0;

free_ctrls:
entity_cleanup:
	/* Clean up global tracking if we added this sensor */
	if (added_global) {
		g_arcimx616_sensor_count--;
		/* Clean up i2c clients if this was the only sensor */
		if (g_arcimx616_sensor_count == 0) {
			if (g_dual_i2c_client) {
				i2c_unregister_device(g_dual_i2c_client);
				g_dual_i2c_client = NULL;
			}
			if (g_aw9106_i2c_client) {
				i2c_unregister_device(g_aw9106_i2c_client);
				g_aw9106_i2c_client = NULL;
			}
			if (g_temp_i2c_client) {
				i2c_unregister_device(g_temp_i2c_client);
				g_temp_i2c_client = NULL;
			}
			if (g_eeprom_i2c_client) {
				i2c_unregister_device(g_eeprom_i2c_client);
				g_eeprom_i2c_client = NULL;
			}
		}
	}
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	dev_dbg(&client->dev, "Returning %d from arcimx616_probe",ret);

	return ret;
}

static void arcimx616_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct arcimx616_dev *sensor = to_arcimx616_dev(sd);

	arcimx616_led_remove_sysfs(sensor);

       /* Drop the synthetic AT24 client if we created one */
       if (sensor->eeprom_client) {
               i2c_unregister_device(sensor->eeprom_client);
               sensor->eeprom_client = NULL;
       }

	/* Remove from global array and cleanup dual client if last sensor */
	{
		int i;
		for (i = 0; i < g_arcimx616_sensor_count; i++) {
			if (g_arcimx616_sensors[i] == sensor) {
				/* Shift remaining sensors down */
				for (int j = i; j < g_arcimx616_sensor_count - 1; j++) {
					g_arcimx616_sensors[j] = g_arcimx616_sensors[j + 1];
				}
				g_arcimx616_sensor_count--;
				break;
			}
		}

		/* Clean up dual i2c client if this was the last sensor */
		if (g_arcimx616_sensor_count == 0) {
			if (g_dual_i2c_client) {
				i2c_unregister_device(g_dual_i2c_client);
				g_dual_i2c_client = NULL;
				dev_dbg(&client->dev, "Cleaned up dual i2c client\n");
			}
			if (g_aw9106_i2c_client) {
				i2c_unregister_device(g_aw9106_i2c_client);
				g_aw9106_i2c_client = NULL;
				dev_dbg(&client->dev, "Cleaned up AW9106 i2c client\n");
			}
			if (g_temp_i2c_client) {
				i2c_unregister_device(g_temp_i2c_client);
				g_temp_i2c_client = NULL;
				dev_dbg(&client->dev, "Cleaned up temp-probe i2c client\n");
			}
			if (g_eeprom_i2c_client) {
				i2c_unregister_device(g_eeprom_i2c_client);
				g_eeprom_i2c_client = NULL;
				dev_dbg(&client->dev, "Cleaned up eeprom-probe i2c client\n");
			}
		}
	}

	v4l2_async_unregister_subdev(&sensor->sd);
	pm_runtime_disable(&client->dev);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
}

/* Suspend drains detect work; resume re-probes the accessory state. */
static int arcimx616_pm_prepare(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct arcimx616_dev *sensor = to_arcimx616_dev(sd);
	unsigned long flags;

	/* The detect owner handles global teardown. */
	if (!sensor->detect_gpio)
		return 0;

	if (sensor->irq > 0)
		disable_irq(sensor->irq);

	cancel_delayed_work_sync(&sensor->connect_work);

	spin_lock_irqsave(&sensor->irq_lock, flags);
	sensor->connect_work_scheduled = false;
	arcimx616_reset_debounce_state(sensor);
	spin_unlock_irqrestore(&sensor->irq_lock, flags);

	arcimx616_power_off_all_sensors();

	return 0;
}

static void arcimx616_pm_complete(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct arcimx616_dev *sensor = to_arcimx616_dev(sd);
	unsigned long flags;
	bool plugged_in;
	bool was_connected;
	bool emit_disconnected = false;
	bool emit_connected = false;
	bool schedule_probe = false;
	bool keep_power_on = false;
	struct arcimx616_probe_result probe = {0};

	if (!sensor->detect_gpio) {
		return;
	}

	/* Rebuild detect state from a fresh probe. */
	spin_lock_irqsave(&sensor->irq_lock, flags);
	was_connected = sensor->connected;
	sensor->connected = false;
	sensor->accessory_power_latched = false;
	sensor->connect_work_scheduled = false;
	arcimx616_reset_debounce_state(sensor);
	spin_unlock_irqrestore(&sensor->irq_lock, flags);

	plugged_in = (gpiod_get_value(sensor->detect_gpio) == 0);

	if (!plugged_in) {
		if (was_connected)
			emit_disconnected = true;
		goto out;
	}

	arcimx616_set_power(sensor, true);
	if (sensor->mux_enable)
		gpiod_set_value_cansleep(sensor->mux_enable, 0);
	arcimx616_probe_sources(sensor, &probe);

	if (probe.custom_peripheral_ok || probe.strobe_active) {
		spin_lock_irqsave(&sensor->irq_lock, flags);
		sensor->accessory_power_latched = true;
		spin_unlock_irqrestore(&sensor->irq_lock, flags);
		keep_power_on = true;
		if (was_connected)
			emit_disconnected = true;
	} else if (probe.sensor0_ok && probe.sensor1_ok) {
		spin_lock_irqsave(&sensor->irq_lock, flags);
		sensor->connected = true;
		spin_unlock_irqrestore(&sensor->irq_lock, flags);
		if (!was_connected)
			emit_connected = true;
	} else {
		if (was_connected)
			emit_disconnected = true;
		spin_lock_irqsave(&sensor->irq_lock, flags);
		sensor->connect_work_scheduled = true;
		spin_unlock_irqrestore(&sensor->irq_lock, flags);
		schedule_probe = true;
	}

	if (!keep_power_on) {
		arcimx616_set_power(sensor, false);
	} else {
		arcimx616_led_apply_effective();
	}

out:
	if (schedule_probe)
		schedule_delayed_work(&sensor->connect_work, HZ/4);
	if (emit_disconnected) {
		struct v4l2_event ev = { .type = V4L2_EVENT_SENSOR_DISCONNECTED };
		v4l2_event_queue(sensor->sd.devnode, &ev);
	}
	if (emit_connected) {
		struct v4l2_event ev = { .type = V4L2_EVENT_SENSOR_CONNECTED };
		v4l2_event_queue(sensor->sd.devnode, &ev);
	}

	if (sensor->irq > 0)
		enable_irq(sensor->irq);
}

static const struct dev_pm_ops arcimx616_pm_ops = {
	.prepare  = arcimx616_pm_prepare,
	.complete = arcimx616_pm_complete,
};

static const struct i2c_device_id arcimx616_id[] = {
	{"arcimx616", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, arcimx616_id);

static const struct of_device_id arcimx616_dt_ids[] = {
	{ .compatible = "arcturus,arcimx616" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, arcimx616_dt_ids);

static struct i2c_driver arcimx616_i2c_driver = {
	.driver = {
		.name  = "arcimx616",
		.of_match_table	= arcimx616_dt_ids,
		.pm = &arcimx616_pm_ops,
	},
	.id_table = arcimx616_id,
	.probe = arcimx616_probe,
	.remove   = arcimx616_remove,
};

module_i2c_driver(arcimx616_i2c_driver);

MODULE_DESCRIPTION("arcimx616 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
