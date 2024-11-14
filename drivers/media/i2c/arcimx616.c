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
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

bool connected_override = true;


#define VIVID_CID_ARCIMX616_BASE	(V4L2_CID_USER_BASE | 0xf000)
#define V4L2_CID_TEMPERATURE	(VIVID_CID_ARCIMX616_BASE + 0)

/* min/typical/max system clock (xclk) frequencies */
#define arcimx616_XCLK_MIN  6000000
#define arcimx616_XCLK_MAX 54000000

#define arcimx616_DEFAULT_SLAVE_ID 0x1a

#define arcimx616_REG_CHIP_ID		0x16

#define ARCIMX616_LINK_FREQUENCY  2200000000


#define arcimx616_REG_TIMING_DVPHO		0x3808
#define arcimx616_REG_TIMING_DVPVO		0x380a
#define arcimx616_REG_TIMING_HTS		0x380c
#define arcimx616_REG_TIMING_VTS		0x380e

#define IMX_REG_TEMP_SEN_OUT		0x013A

// Private ioctls
struct eeprom_access_info {
	u32 offset; 			// offset into the eeprom memory range
	u32 size;				// number of bytes copied
	void __user *buffer;	// src buffer when using VIDIOC_PRIV_S_EEPROM_DATA, or dst when using VIDIOC_PRIV_G_EEPROM_DATA
};
#define VIDIOC_PRIV_G_CONNECTED _IOR('V', BASE_VIDIOC_PRIVATE + 1, u32)
#define VIDIOC_PRIV_S_CONNECTED _IO('V', BASE_VIDIOC_PRIVATE + 2)
#define VIDIOC_PRIV_S_DUAL_REG _IO('V', BASE_VIDIOC_PRIVATE + 3) 
#define VIDIOC_PRIV_G_EEPROM_DATA _IOWR('V', BASE_VIDIOC_PRIVATE + 4, struct eeprom_access_info) 
#define VIDIOC_PRIV_S_EEPROM_DATA _IOW('V', BASE_VIDIOC_PRIVATE + 5, struct eeprom_access_info) 

// Private events
#define V4L2_EVENT_SENSOR_CONNECTED   (V4L2_EVENT_PRIVATE_START + 2)
#define V4L2_EVENT_SENSOR_DISCONNECTED   (V4L2_EVENT_PRIVATE_START + 3)

#define ARCIMX616_EEPROM_SIZE (8*1024)

/* regulator supplies */
static const char * const arcimx616_supply_name[] = {
	"DOVDD", /* Entire camera system supply */
};

#define arcimx616_NUM_SUPPLIES ARRAY_SIZE(arcimx616_supply_name)

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

static int power_on_count = 0;

struct arcimx616_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to arcimx616 */
	struct clk *xclk2; /* system clock to arcimx616 */

	struct regulator_bulk_data supplies[arcimx616_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	struct gpio_desc *detect_gpio;
	int irq;

	struct delayed_work connect_work;      // For delayed "connected" notification
    spinlock_t irq_lock;                  // For state updates in the IRQ handler
    bool connect_work_scheduled;          // Whether we have scheduled the delayed work
    bool connected;                       // Whether we are currently in the "connected" state

	/* lock to protect all members below */
	struct mutex lock;

	void __iomem *base;
	u32 offset;

	const struct arcimx616_mode_info* current_mode;

	struct v4l2_ctrl_handler ctrl_handler;
	struct mutex mutex;

	struct v4l2_mbus_framefmt fmt;

	bool streaming;

	// nvmem_device_read/nvmem_device_write cannot work directly with user pointers to we need
	// to copy to/from this buffer
	u8 eeprom_buffer[ARCIMX616_EEPROM_SIZE];

	/* on-demand AT24 client */
	struct i2c_client *eeprom_client;

};

/* ------------------------------------------------------------------ */
/* Context + match helper (unchanged)                                 */
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
/* ------------------------------------------------------------------ */
/*
 * Ensure an at24 (24c64) client exists *and* its nvmem provider is ready,
 * so that nvmem_device_get() will succeed without returning -EPROBE_DEFER.
 */
#define ARCIMX616_NVMEM_NAME_LEFT      "passthrough_cams_eeprom_left"
#define ARCIMX616_NVMEM_NAME_RIGHT     "passthrough_cams_eeprom_right"
#define ARCIMX616_NVMEM_TIMEOUT   1000      /* total 1 s */
#define ARCIMX616_NVMEM_POLL_MS   50
#define ARCIMX616_EEPROM_TPOR_MS  10 

static int arcimx616_ensure_eeprom_client(struct arcimx616_dev *sensor)
{
	struct i2c_client  *master = sensor->i2c_client;
	struct i2c_adapter *adap   = master->adapter;
	struct i2c_client  *cl;
	bool is_left = (master->addr == 0x10) ? true : false;
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



/* Maximum number of ARCIMX616 sensors we expect */
#define ARCIMX616_MAX_SENSORS 2

/* Global array tracking all probed sensors */
static struct arcimx616_dev *g_arcimx616_sensors[ARCIMX616_MAX_SENSORS];
static int g_arcimx616_sensor_count = 0;

static int arcimx616_set_power(struct arcimx616_dev *sensor, bool on);

/* Work struct for powering off all sensors */
static struct work_struct power_off_work;

/* Work function to power off all sensors */
static void arcimx616_power_off_all_work_fn(struct work_struct *work)
{
    int i;
    pr_info("arcimx616: Forcing power off on all sensors\n");

    for (i = 0; i < g_arcimx616_sensor_count; i++) {
        if (g_arcimx616_sensors[i]) {
            /* Call the normal set_power() with 'false' to turn them off. */
            arcimx616_set_power(g_arcimx616_sensors[i], false);
        }
    }
}

/*
 * Helper to force power off on ALL known ARCIMX616 sensors.
 * This is triggered whenever one sensor sees a disconnect event.
 */
static void arcimx616_power_off_all_sensors(void)
{
    /* Queue the power off work on a high priority work queue */
    queue_work(system_highpri_wq, &power_off_work);
}

static inline struct arcimx616_dev *to_arcimx616_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct arcimx616_dev, sd);
}

static const struct reg_value arcimx616_global_settings[] = {
{0x0136, 0x18,0 ,0}, {0x0137, 0x00,0 ,5}, {0x3c7e, 0x08,0 ,0},
{0x3c7f, 0x15,0 ,0}, {0x0111, 0x02,0 ,0},

// global settings
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



// Image quality settings {0x88D6, 0x60, 0, 0}, {0x9852, 0x00, 0, 0},
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

//timing settings:
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

static const struct reg_value arcimx616_setting_1280x1024_binned[] = {

//enable 2v2h binnging
{0x0900, 0x01,0, 0},
{0x0901, 0x22,0, 0},
{0x0902, 0x02,0, 0}, //additional average weighting binning
{0xE04C, 0x00,0, 0},
{0xE04D, 0x87,0, 0},
{0xE04E, 0x00,0, 0},
{0xE04F, 0x1F,0, 0},
{0x3E20, 0x01,0, 0},
{0x3E37, 0x00,0, 0},
{0x3F50, 0x00,0, 0},
{0x3F56, 0x01,0, 0},
{0x3F57, 0x02,0, 0},
{0X3FF9, 0x00,0, 0}, //use per-channel digital gain

{0X0204,	0x01,0},//gain
{0X0205,	0xFF,0},//gain

{0X0202,	0x0B,0},
{0X0203,	0xe8,0}, //exposure

//image quality register 
{0x9827, 0x20, 0, 0}, {0x9830, 0x0A, 0, 0}, {0x9833, 0x0A, 0, 0},
{0x9834, 0x32, 0, 0}, {0x9837, 0x22, 0, 0}, {0x983C, 0x04, 0, 0},
{0x983F, 0x0A, 0, 0}, {0x994F, 0x00, 0, 0}, {0x9A48, 0x06, 0, 0},
{0x9A49, 0x06, 0, 0}, {0x9A4A, 0x06, 0, 0}, {0x9A4B, 0x06, 0, 0},
{0x9A4E, 0x03, 0, 0}, {0x9A4F, 0x03, 0, 0}, {0x9A54, 0x03, 0, 0},
{0x9A66, 0x03, 0, 0}, {0x9A67, 0x03, 0, 0}, {0xA2C9, 0x02, 0, 0},
{0xA2CB, 0x02, 0, 0}, {0xA2CD, 0x02, 0, 0}, {0xB249, 0x3F, 0, 0},
{0xB24F, 0x3F, 0, 0}, {0xB290, 0x3F, 0, 0}, {0xB293, 0x3F, 0, 0},
{0xB296, 0x3F, 0, 0}, {0xB299, 0x3F, 0, 0}, {0xB2A2, 0x3F, 0, 0},
{0xB2A8, 0x3F, 0, 0}, {0xB2A9, 0x0D, 0, 0}, {0xB2AA, 0x0D, 0, 0},
{0xB2AB, 0x3F, 0, 0}, {0xB2BA, 0x2F, 0, 0}, {0xB2BB, 0x2F, 0, 0},
{0xB2BC, 0x2F, 0, 0}, {0xB2BD, 0x10, 0, 0}, {0xB2C0, 0x3F, 0, 0},
{0xB2C3, 0x3F, 0, 0}, {0xB2D2, 0x3F, 0, 0}, {0xB2DE, 0x20, 0, 0},
{0xB2DF, 0x20, 0, 0}, {0xB2E0, 0x20, 0, 0}, {0xB2EA, 0x3F, 0, 0},
{0xB2ED, 0x3F, 0, 0}, {0xB2EE, 0x3F, 0, 0}, {0xB2EF, 0x3F, 0, 0},
{0xB2F0, 0x2F, 0, 0}, {0xB2F1, 0x2F, 0, 0}, {0xB2F2, 0x2F, 0, 0},
{0xB2F9, 0x0E, 0, 0}, {0xB2FA, 0x0E, 0, 0}, {0xB2FB, 0x0E, 0, 0},
{0xB759, 0x01, 0, 0}, {0xB765, 0x3F, 0, 0}, {0xB76B, 0x3F, 0, 0},
{0xB7B3, 0x03, 0, 0}, {0xB7B5, 0x03, 0, 0}, {0xB7B7, 0x03, 0, 0},
{0xB7BF, 0x03, 0, 0}, {0xB7C1, 0x03, 0, 0}, {0xB7C3, 0x03, 0, 0},
{0xB7EF, 0x02, 0, 0}, {0xB7F5, 0x1F, 0, 0}, {0xB7F7, 0x1F, 0, 0},
{0xB7F9, 0x1F, 0, 0}, {0xBCF1, 0x0, 0, 0},

//other register block for binning 
{0x3C0A, 0x73, 0, 0}, {0x3C0B, 0x64, 0, 0}, {0x3C0C, 0x5F, 0, 0},
{0x3C0D, 0x00, 0, 0}, {0x3C0E, 0x00, 0, 0}, {0x3C0F, 0x00, 0, 0},
{0x3C10, 0xA4, 0, 0}, {0x3C11, 0x02, 0, 0}, {0x3C12, 0x00, 0, 0},
{0x3C13, 0x03, 0, 0}, {0x3C14, 0x80, 0, 0}, {0x3C15, 0x04, 0, 0},
{0x3C16, 0x15, 0, 0}, {0x3C17, 0x15, 0, 0}, {0x3C18, 0x15, 0, 0},
{0x3C19, 0x15, 0, 0}, {0x3C1A, 0x15, 0, 0}, {0x3C1B, 0x15, 0, 0},
{0x3C1C, 0x06, 0, 0}, {0x3C1D, 0x06, 0, 0}, {0x3C1E, 0x06, 0, 0},
{0x3C1F, 0x06, 0, 0}, {0x3C20, 0x06, 0, 0}, {0x3C21, 0x06, 0, 0},
{0x3C22, 0x3F, 0, 0}, {0x3C23, 0x0A, 0, 0}, {0x3E35, 0x01, 0, 0},
{0x3F4A, 0x01, 0, 0}, {0x3F4B, 0x7F, 0, 0}, {0x3F26, 0x00, 0, 0},

{0x0344, 0x00,0, 0}, //analog horizontal cropping start
{0x0345, 0x00,0, 0},
{0x0346, 0x00,0, 0}, //vertical direction analog cropping start
{0x0347, 0x00,0, 0},
{0x0348, 0x0F,0, 0},
{0x0349, 0xD7,0, 0}, //anolog crop end
{0x034A, 0x0B,0, 0},
{0x034B, 0xdf,0, 0},


{0x0348, 0x0f,0, 0},
{0x0349, 0xd7,0, 0}, //anolog crop end

{0x034C, 0x05,0, 0}, //output width
{0x034D, 0x00,0, 0},

{0x034E, 0x04,0, 0},
{0x034F, 0x00,0, 0}, //output height

{0x0408, 0x00,0, 0},
{0x0409, 0x36,0, 0}, //digital crop left
{0x040A, 0x00,0, 0},
{0x040B, 0x00,0, 0},
{0x040C, 0x05,0, 0}, //digital width
{0x040D, 0x00,0, 0},
{0x040E, 0x04,0, 0}, //digital height
{0x040F, 0x00,0, 0},	
};


static const struct arcimx616_mode_info arcimx616_mode_data_3820x2464 = {
	2688, 0, 2688, 0, arcimx616_setting_3820x2464, ARRAY_SIZE(arcimx616_setting_3820x2464)
	// 1280, 1456, 800, 3676, arcimx616_setting_3820x2464_2688_crop, ARRAY_SIZE(arcimx616_setting_3820x2464)
};

static const struct arcimx616_mode_info arcimx616_mode_data_1640x1232_binned = {
	1920, 0, 1520, 0, arcimx616_setting_1640x1232_binned, ARRAY_SIZE(arcimx616_setting_1640x1232_binned)
	// 1280, 1456, 800, 3676, arcimx616_setting_3820x2464_2688_crop, ARRAY_SIZE(arcimx616_setting_3820x2464)
};

static const struct arcimx616_mode_info arcimx616_mode_data_1280x1024_binned = {
	1920, 0, 1520, 0, arcimx616_setting_1280x1024_binned, ARRAY_SIZE(arcimx616_setting_1280x1024_binned)
	// 1280, 1456, 800, 3676, arcimx616_setting_3820x2464_2688_crop, ARRAY_SIZE(arcimx616_setting_3820x2464)
};

static const s64 link_freq_menu_items[] = {
	ARCIMX616_LINK_FREQUENCY
};

static int arcimx616_write_reg_dual(struct arcimx616_dev *sensor, u16 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	unsigned short old_addr = client->addr;
	client->addr = 0x36;  // Set dual write address
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
		client->addr = old_addr;
		return ret;
	}

	client->addr = old_addr;
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

static int arcimx616_nvmem_match(struct device *dev, const void *data)
{
	const char* expected_prefix = data;
	const char* name = dev_name(dev);

	if(strlen(name) < strlen(expected_prefix)) {
		return 0; // name too short to hold the prefix
	}

	return memcmp(expected_prefix, name, strlen(expected_prefix)) == 0;
}


static int arcimx616_copy_eeprom_data(struct arcimx616_dev *sensor, struct eeprom_access_info *access_info, bool from_eeprom)
{
	int ret = 0;

	arcimx616_set_power(sensor, true);
	ret = arcimx616_ensure_eeprom_client(sensor);
	if (ret) {
		dev_err(&sensor->i2c_client->dev,
			"failed to ensure eeprom client: %d\n", ret);
		//goto out_power;
	}
	ret = 0;

	// Try to find an nvmem device with a name starting with this expected prefix
	// TODO: consider making that an of property
	bool is_left = (sensor->i2c_client->addr == 0x10) ? true : false;
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

	// Make sure we get the expected EEPROM size (8KB)
	size_t nvmem_size = nvmem_dev_size(nvmem_dev);
	if(nvmem_size != ARCIMX616_EEPROM_SIZE)
	{
		dev_err(&sensor->i2c_client->dev, "Unexpected EEPROM data size (nvmem cell size is %zu but expected %d\n", nvmem_size, ARCIMX616_EEPROM_SIZE);
		// Release nvmem_device before early return
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

	const u32 protected_range_end = 0x800; // [0x0, 0x800] is written by manufacturer
	if(!from_eeprom && (eeprom_offset < protected_range_end))
	{
		dev_err(&sensor->i2c_client->dev, "Invalid EEPROM access range [0x%x, 0x%x] would overwrite the protected range [0x0, 0x%x] \n",
			eeprom_offset, eeprom_offset+access_info->size, protected_range_end);
		nvmem_device_put(nvmem_dev);
		ret = -EINVAL;
		goto out_power;
	}

	
	if(from_eeprom) {
		// Copy from EEPROM to user buffer
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
		// Copy from user buffer to EEPROM
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

/**
 * arcimx616_aw9106_write_reg - Write one byte to a single-byte register on AW9106C
 * Returns 0 on success, negative error code on failure.
 */
static int arcimx616_aw9106_write_reg(struct arcimx616_dev *sensor, u8 reg, u8 val)
{
    struct i2c_client *client = sensor->i2c_client;
	unsigned short old_addr = client->addr;
	client->addr = AW9106C_ADDR;
    struct i2c_msg msg;
    u8 buf[2];
    int ret;

    if (!client)
        return -ENODEV;

    buf[0] = reg;
    buf[1] = val;

    msg.addr  = client->addr;
    msg.flags = 0;  /* Write */
    msg.len   = 2;
    msg.buf   = buf;

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0) {
        dev_err(&client->dev, "AW9106 reg=0x%02x val=0x%02x write failed: %d\n",
                reg, val, ret);
		client->addr = old_addr;
        return ret;
    }
	client->addr = old_addr;
    return 0;
}

/**
 * arcimx616_aw9106_set_leds - Example function to control two "RGB groups" of AW9106C
 * @sensor:        The arcimx616_dev
 * @left_r/g/b:    Brightness (0..255) for left LED's R/G/B
 * @right_r/g/b:   Brightness (0..255) for right LED's R/G/B
 * @left_breathe:  True => left side breathes (blink), else steady
 * @right_breathe: True => right side breathes (blink), else steady
 * @enable:        True => turn them on, False => turn them off
 */
static int arcimx616_aw9106_set_leds(struct arcimx616_dev *sensor,
                                  u8 left_r, u8 left_g, u8 left_b,
                                  u8 right_r, u8 right_g, u8 right_b,
                                  bool left_breathe, bool right_breathe,
                                  bool enable)
{
    int ret = 0;
    u8 paten_val = 0;
    u8 p1dir = 0;
    u8 p0dir = 0;

    if (!sensor->i2c_client)
        return -ENODEV;

    /* If disable => just zero everything out. */
    if (!enable) {
        ret |= arcimx616_aw9106_write_reg(sensor, 0x20, 0x00); // DIM0
        ret |= arcimx616_aw9106_write_reg(sensor, 0x21, 0x00); // DIM1
        ret |= arcimx616_aw9106_write_reg(sensor, 0x22, 0x00); // DIM2
        ret |= arcimx616_aw9106_write_reg(sensor, 0x23, 0x00); // DIM3
        ret |= arcimx616_aw9106_write_reg(sensor, 0x24, 0x00); // DIM4
        ret |= arcimx616_aw9106_write_reg(sensor, 0x25, 0x00); // DIM5
        /* Turn off breathing enable bits. */
        ret |= arcimx616_aw9106_write_reg(sensor, 0x14, 0x00);
        /* Clear GO bit in GCR. */
        ret |= arcimx616_aw9106_write_reg(sensor, 0x11, 0x00);

		ret |= arcimx616_aw9106_write_reg(sensor, 0x7f, 0x00); // reset
        return ret;
    }

    /*
     *    Switch all pins to LED mode:
     *    P0WKMD=0x00 => OUT4..5 LED
     *    P1WKMD=0x00 => OUT0..3 LED
     */
    ret |= arcimx616_aw9106_write_reg(sensor, 0x12, 0x00); // P0WKMD
    ret |= arcimx616_aw9106_write_reg(sensor, 0x13, 0x00); // P1WKMD

    /* Write brightness registers (DIM0..5) */
    ret |= arcimx616_aw9106_write_reg(sensor, 0x20, left_r);   // out0
    ret |= arcimx616_aw9106_write_reg(sensor, 0x21, left_g);   // out1
    ret |= arcimx616_aw9106_write_reg(sensor, 0x22, left_b);   // out2
    ret |= arcimx616_aw9106_write_reg(sensor, 0x23, right_r);  // out3
    ret |= arcimx616_aw9106_write_reg(sensor, 0x24, right_g);  // out4
    ret |= arcimx616_aw9106_write_reg(sensor, 0x25, right_b);  // out5

    /*
     * Enable breathing on whichever pins are meant to breathe:
     */
    if (left_breathe) {
        /* left => out0/out1/out2 => bits 0..2 */
        if (left_r > 0) paten_val |= BIT(0);
        if (left_g > 0) paten_val |= BIT(1); 
        if (left_b > 0) paten_val |= BIT(2);
    }
    if (right_breathe) {
        /* right => out3/out4/out5 => bits 3..5 */
        if (right_r > 0) paten_val |= BIT(3);
        if (right_g > 0) paten_val |= BIT(4);
        if (right_b > 0) paten_val |= BIT(5);
    }
    ret |= arcimx616_aw9106_write_reg(sensor, 0x14, paten_val);

    /*
     *   For BLINK vs SMART-FADE:
     *    In LED mode + PATEN=1, setting the corresponding DIR bit=1 => BLINK
     *    bit=0 => SMART-FADE. 
     *    P1DIR(0x05) => bits [3..0] => OUT3..OUT0
     *    P0DIR(0x04) => bits [1..0] => OUT5..OUT4
     */
    if (left_breathe) {
        /* out0..2 => bits 0..2 => set them to 1 => blink */
        p1dir |= 0x07;  /* sets bits 2..0 */
    }
    if (right_breathe) {
        /* out3 => bit3 in P1DIR, out4=>bit0 in P0DIR, out5=>bit1 in P0DIR */
        p1dir |= 0x08;  /* set bit3 => out3 => blink */
        p0dir |= 0x03;  /* set bits 1..0 => out5..4 => blink */
    }
    ret |= arcimx616_aw9106_write_reg(sensor, 0x05, p1dir); // P1DIR
    ret |= arcimx616_aw9106_write_reg(sensor, 0x04, p0dir); // P0DIR

    /*
     *    If breathing, set fade/hold times in FDTMR(0x15) & FLTMR(0x16).
     */
    if (left_breathe || right_breathe) {
        ret |= arcimx616_aw9106_write_reg(sensor, 0x15, 0x1A);
        ret |= arcimx616_aw9106_write_reg(sensor, 0x16, 0x1A);
    }

    /*  For BLINK mode, set GO=1 in GCR(0x11) => 0x80 */
    if (left_breathe || right_breathe) {
        ret |= arcimx616_aw9106_write_reg(sensor, 0x11, 0x80);
    } else {
        /* For steady on, we do not enable GO. */
        ret |= arcimx616_aw9106_write_reg(sensor, 0x11, 0x00);
    }
    return ret;
}

static int arcimx616_load_regs(struct arcimx616_dev *sensor,
			    const struct arcimx616_mode_info *mode)
{
	//load default regs
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
				// Apply the delay in the register map
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
				// Apply the delay in the register map
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

	//dev_err(&client->dev, "Trying to restore mode - regmap");
	/* first load the initial register values */
	ret = arcimx616_load_regs(sensor, (sensor->current_mode));
	if (ret < 0)
		return ret;

	ret = arcimx616_read_reg(sensor, 0x114, &val);
	if (ret < 0 || val != 0x03) {
		//struct i2c_client *client = sensor->i2c_client;
		dev_err(&client->dev, "verify failed val=%.2x ret=%d\n", val, ret);
		return ret;
	}
	//dev_err(&client->dev, "Mode restored - regmap");


	return 0;
}

static int arcimx616_set_power_on(struct arcimx616_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	ret = regulator_bulk_enable(arcimx616_NUM_SUPPLIES,
				    sensor->supplies);

	//  If this is not the sensor that owns the reset gpio, skip it
	if(sensor->reset_gpio == NULL) {
		return 0;
	}

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	// Make sure the reset is low for at least 1ms
	usleep_range(1000, 1500);

	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}



	clk_set_rate(sensor->xclk, 24000000);

	ret = clk_prepare_enable(sensor->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
	}

	clk_set_rate(sensor->xclk2, 24000000);

	ret = clk_prepare_enable(sensor->xclk2);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
	}

	// Wait for power to stabilize
	usleep_range(5000, 5500);

	// Set the reset high to release the sensor from reset
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);

	// Wait for the sensor to be ready to receive commands
	usleep_range(13000, 13500);

	return 0;

}

static void arcimx616_set_power_off(struct arcimx616_dev *sensor)
{
	struct i2c_client *client = sensor->i2c_client;

	regulator_bulk_disable(arcimx616_NUM_SUPPLIES, sensor->supplies);

	clk_disable_unprepare(sensor->xclk);

	if(sensor->reset_gpio == NULL) {
		return ;
	}
	dev_err(&client->dev, "%s: Powered off\n",
			__func__);
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);

	// Make sure everything is off before returning
	usleep_range(1000, 1500);

	
}

static int arcimx616_set_power(struct arcimx616_dev *sensor, bool on)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

    bool was_on = power_on_count > 0;

	if(on) {
		power_on_count++;
	}
	else {
		if(power_on_count > 0) {
			power_on_count --;
		}
		else {
			// already powered off
			return 0;
		}
	}
	
	if (on && !was_on) {

		for(int i = 0; i < g_arcimx616_sensor_count; i++) {
			ret = arcimx616_set_power_on(g_arcimx616_sensors[i]);
			if (ret)
				return ret;
		}
	
		//stuff we always when on
		for(int i = 0; i < g_arcimx616_sensor_count; i++) {
			ret = arcimx616_restore_mode(g_arcimx616_sensors[i]);
			if (ret)
				dev_err(&client->dev, "Error from arcimx616_restore_mode");
		}
	} else {

		if(power_on_count == 0)
		{
			for(int i = 0; i < g_arcimx616_sensor_count; i++) {
				arcimx616_set_power_off(g_arcimx616_sensors[i]);
			}
		}
	}

	return 0;
}

static int arcimx616_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct arcimx616_dev *sensor =
			container_of(ctrl->handler, struct arcimx616_dev, ctrl_handler);
	int ret = 0;

	switch (ctrl->id) {

	case V4L2_CID_BRIGHTNESS:
		ret = arcimx616_write_reg(sensor, ctrl->val & 0xffff, ctrl->val >> 16 & 0xff);
		break;
	default:
		ret = -EINVAL;
		break;
	}
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
	
	//add custom control for temperature sensor
	v4l2_ctrl_new_custom(ctrl_hdlr, &arcimx616_ctrl_temperature, NULL);

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	sensor->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

/* --------------- Subdev Operations --------------- */

static int arcimx616_s_power(struct v4l2_subdev *sd, int on)
{
	struct arcimx616_dev *sensor = to_arcimx616_dev(sd);
	int ret = 0;

	struct i2c_client *client = sensor->i2c_client;
	
	if(client->addr == 0x10) {
		sensor->base = devm_ioremap(sd->dev, 0xacc3000, 0xacc3000 + 0xd000);
	}
	else {
		sensor->base = devm_ioremap(sd->dev, 0xacb4000, 0xacb4000 + 0xd000);	
	}
	if (IS_ERR(sensor->base)) {
		dev_err(&client->dev, "could not map memory\n");
	}
	
	dev_err(&client->dev, "calling s_power with on:%d",on);
	mutex_lock(&sensor->lock);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	
	ret = arcimx616_set_power(sensor, !!on);
	mutex_unlock(&sensor->lock);
	return 0;
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
	return 0;

	return ret;
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
			if (sensor->detect_gpio) {
				*a = (gpiod_get_value(sensor->detect_gpio) == 0);
				sensor->connected = *a;
			} else {
				*a = sensor->connected;
			}
			if(connected_override == false) {
				*a = false;
				sensor->connected = false;
			}
			break;
		case VIDIOC_PRIV_S_CONNECTED:
			if(sensor->connected != (unsigned long)arg) {
				sensor->connected = (unsigned long)arg;
				if(!sensor->connected) {
					arcimx616_power_off_all_sensors();
					connected_override = false;
				}
				else {
					connected_override = true;
				}

				struct v4l2_event ev = {0};
				ev.type = sensor->connected ? V4L2_EVENT_SENSOR_CONNECTED : V4L2_EVENT_SENSOR_DISCONNECTED;
				v4l2_event_queue(sd->devnode, &ev);
			}
			break;
		case VIDIOC_PRIV_S_DUAL_REG:
			{
				u32 val = (uintptr_t)arg; // argument contains reg addr and value
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
		// This map is used for other resolutions that only modify the cropping manually
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
		dev_err(&client->dev, "%s: Correct chip identifier, expected 0x616, got 0x%x\n",
			__func__, chip_id);
	}
	return ret;
}

static void arcimx616_connect_work_fn(struct work_struct *work)
{
    struct arcimx616_dev *sensor =
        container_of(to_delayed_work(work), struct arcimx616_dev, connect_work);

    static bool first_stage = true;

    if (!sensor->detect_gpio) {
        dev_err(&sensor->i2c_client->dev, "No detect GPIO found\n");
        return;
    }


    /* Check if still connected */
    if (gpiod_get_value(sensor->detect_gpio) == 0) {
        if (first_stage) {
            // First stage - turn on power and LEDs
			arcimx616_set_power(sensor, true);
			int sensor0Connected = arcimx616_check_chip_id(g_arcimx616_sensors[0]);
			int sensor1Connected = arcimx616_check_chip_id(g_arcimx616_sensors[1]);

			

            
            arcimx616_aw9106_set_leds(sensor, 	sensor0Connected != 0 ? 255 : 0, 
											sensor0Connected == 0 ? 255 : 0, 
											0, 
											sensor1Connected != 0 ? 255 : 0, 
											sensor1Connected == 0 ? 255 : 0,
											0,
											true, true, true);
            
            // Schedule LED power off and user space notification after 500ms
            first_stage = false;
            schedule_delayed_work(&sensor->connect_work, HZ/2);
            return;
        } else {
            // Second stage - turn off LEDs and power, then signal connected if still plugged
            arcimx616_aw9106_set_leds(sensor, 0, 0, 0, 0, 0, 0, false, false, false);
            arcimx616_set_power(sensor, false);

			sensor->connected = true;
			struct v4l2_event ev = {0};
			ev.type = V4L2_EVENT_SENSOR_CONNECTED;
			v4l2_event_queue(sensor->sd.devnode, &ev);            
        }
    } else {
        dev_info(&sensor->i2c_client->dev, "Accessory canceled connect during debounce\n");
    }
    // Reset for next time and clear scheduled flag
    first_stage = true;
    spin_lock(&sensor->irq_lock);
    sensor->connect_work_scheduled = false;
    spin_unlock(&sensor->irq_lock);
}


static irqreturn_t gpio_handler(int irq, void *ident)
{
    struct arcimx616_dev *sensor = (struct arcimx616_dev *) ident;
    int state = gpiod_get_value(sensor->detect_gpio);

    spin_lock(&sensor->irq_lock);

    if (state == 0) {
        // Plugged in (pull-down). Start the 1/4 second timer if not already connected
        if (!sensor->connected && !sensor->connect_work_scheduled) {
            sensor->connect_work_scheduled = true;
            schedule_delayed_work(&sensor->connect_work, HZ/4); // 250ms
        }
    } else {

		/* Immediately turn off power on ALL ARCIMX616 sensors: */
    	arcimx616_power_off_all_sensors();
        // state == 1 => unplugged
        // Cancel any scheduled connect
        if (sensor->connect_work_scheduled) {
            cancel_delayed_work_sync(&sensor->connect_work);
            sensor->connect_work_scheduled = false;
            dev_info(&sensor->i2c_client->dev, "Debounce: canceled connect.\n");
        }

        // If we were connected, send disconnected event immediately
        if (sensor->connected) {
            sensor->connected = false;

            // Send the V4L2 disconnected event
            {
                struct v4l2_event ev = {0};
                ev.type = V4L2_EVENT_SENSOR_DISCONNECTED;
                v4l2_event_queue(sensor->sd.devnode, &ev);
            }
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
	//dev_err(dev, "starting arcimx616 driver\n");
	//dev_info(dev, "probe arcimx616 driver\n");
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
		dev_err(dev, "failed to get xclk\n");
	}
	sensor->xclk2 = devm_clk_get(dev, "xclk2");
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	sensor->detect_gpio = devm_gpiod_get_optional(dev, "detect",
						     GPIOD_IN);
	if(sensor->detect_gpio) {
		dev_err(&client->dev, "detect GPIO found\n");
		sensor->irq = gpiod_to_irq(sensor->detect_gpio);
		ret = devm_request_threaded_irq(dev, sensor->irq, NULL, gpio_handler,
                                        IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                        "arcimx616-detect", sensor);
		if (ret) {
			dev_err(&client->dev, "failed to request IRQ for detect GPIO\n");
		}
		else {
			dev_err(&client->dev, "IRQ for detect GPIO requested successfully\n");
		}
	}
	else {
		dev_err(&client->dev, "no detect GPIO found\n");
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
		} else {
			dev_warn(&client->dev, "arcimx616: Reached max sensor count!\n");
		}
	}

	/* Initialize the power off work */
	INIT_WORK(&power_off_work, arcimx616_power_off_all_work_fn);


	return 0;

free_ctrls:
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	dev_err(&client->dev, "Returning %d from arcimx616_probe",ret);

	return ret;
}

static void arcimx616_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct arcimx616_dev *sensor = to_arcimx616_dev(sd);

       /* Drop the synthetic AT24 client if we created one */
       if (sensor->eeprom_client) {
               i2c_unregister_device(sensor->eeprom_client);
               sensor->eeprom_client = NULL;
       }

	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
}

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
	},
	.id_table = arcimx616_id,
	.probe = arcimx616_probe,
	.remove   = arcimx616_remove,
};

module_i2c_driver(arcimx616_i2c_driver);

MODULE_DESCRIPTION("arcimx616 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
