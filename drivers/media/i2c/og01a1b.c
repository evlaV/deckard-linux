// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Intel Corporation.

#include <linux/acpi.h>
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
#include <linux/workqueue.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define OG01A1B_LINK_FREQ_500MHZ	500000000ULL
#define OG01A1B_SCLK			120000000LL
#define OG01A1B_DATA_LANES		2

#define OG01A1B_REG_CHIP_ID		CCI_REG24(0x300a)
#define OG01A1B_CHIP_ID			0x470141

#define OG01A1B_REG_MODE_SELECT		CCI_REG8(0x0100)
#define OG01A1B_MODE_STANDBY		0x00
#define OG01A1B_MODE_STREAMING		0x01

/* vertical-timings from sensor */
#define OG01A1B_REG_VTS			CCI_REG16(0x380e)
#define OG01A1B_VTS_120FPS		0x0498
#define OG01A1B_VTS_120FPS_MIN		0x0498
#define OG01A1B_VTS_240FPS		0x0281
#define OG01A1B_VTS_240FPS_MIN		0x0281
#define OG01A1B_VTS_40FPS		0x0d6a
#define OG01A1B_VTS_MAX			0x7fff

/* horizontal-timings from sensor */
#define OG01A1B_REG_HTS			CCI_REG16(0x380c)

/* Exposure controls from sensor */
#define OG01A1B_REG_EXPOSURE		CCI_REG16(0x3501)
#define	OG01A1B_EXPOSURE_MIN		1
#define OG01A1B_EXPOSURE_MAX_MARGIN	14
#define	OG01A1B_EXPOSURE_STEP		1

/* Analog gain controls from sensor */
#define OG01A1B_REG_ANALOG_GAIN		CCI_REG16(0x3508)
#define	OG01A1B_ANAL_GAIN_MIN		16
#define	OG01A1B_ANAL_GAIN_MAX		248 /* Max = 15.5x */
#define	OG01A1B_ANAL_GAIN_STEP		1

/* Digital gain controls from sensor */
#define OG01A1B_REG_DIG_GAIN		CCI_REG24(0x350a)
#define OG01A1B_DGTL_GAIN_MIN		1024
#define OG01A1B_DGTL_GAIN_MAX		16384 /* Max = 16x */
#define OG01A1B_DGTL_GAIN_STEP		1
#define OG01A1B_DGTL_GAIN_DEFAULT	1024

/* Test Pattern Control */
#define OG01A1B_REG_TEST_PATTERN	CCI_REG8(0x5100)
#define OG01A1B_TEST_PATTERN_ENABLE	BIT(7)
#define OG01A1B_TEST_PATTERN_BAR_SHIFT	2

/* Strobe output */
#define OG01A1B_REG_OUTPUT_ENABLE	CCI_REG8(0x3009)
#define OG01A1B_OUTPUT_ENABLE_STROBE	BIT(3)

#define to_og01a1b(_sd)			container_of(_sd, struct og01a1b, sd)

enum {
	OG01A1B_LINK_FREQ_1000MBPS,
};

struct og01a1b_reg_list {
	const struct cci_reg_sequence *regs;
	u32 num_of_regs;
};

struct og01a1b_mclk_config {
	/* External clock frequency in Hz */
	unsigned long mclk;

	/* PLL and MIPI timing settings for the 1000 Mbps data rate */
	const struct og01a1b_reg_list reg_list;
};

struct og01a1b_mode {
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
	const struct og01a1b_reg_list reg_list;
};

static const u32 og01a1b_mbus_formats[] = {
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_Y8_1X8,
};

static const struct cci_reg_sequence mipi_data_rate_1000mbps_mclk_19_2mhz[] = {
	{ CCI_REG8(0x0103), 0x01 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0304), 0x00 },
	{ CCI_REG8(0x0305), 0xd2 },
	{ CCI_REG8(0x0323), 0x02 },
	{ CCI_REG8(0x0324), 0x01 },
	{ CCI_REG8(0x0325), 0x77 },
	{ CCI_REG8(0x4837), 0x0f },
};

static const struct cci_reg_sequence mipi_data_rate_1000mbps_mclk_24mhz[] = {
	{ CCI_REG8(0x0103), 0x01 },
	{ CCI_REG8(0x0303), 0x04 },
	{ CCI_REG8(0x0304), 0x00 },
	{ CCI_REG8(0x0305), 0xfa },
	{ CCI_REG8(0x0323), 0x04 },
	{ CCI_REG8(0x0324), 0x01 },
	{ CCI_REG8(0x0325), 0xc2 },
	{ CCI_REG8(0x4837), 0x10 },
};

static const struct cci_reg_sequence og01a1b_common_regs[] = {
	{ CCI_REG8(0x0300), 0x0a },
	{ CCI_REG8(0x0301), 0x29 },
	{ CCI_REG8(0x0302), 0x31 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x01 },
	{ CCI_REG8(0x0308), 0x02 },
	{ CCI_REG8(0x0309), 0x00 },
	{ CCI_REG8(0x0310), 0x00 },
	{ CCI_REG8(0x0311), 0x00 },
	{ CCI_REG8(0x0312), 0x07 },
	{ CCI_REG8(0x0313), 0x00 },
	{ CCI_REG8(0x0314), 0x00 },
	{ CCI_REG8(0x0315), 0x00 },
	{ CCI_REG8(0x0320), 0x02 },
	{ CCI_REG8(0x0321), 0x01 },
	{ CCI_REG8(0x0322), 0x01 },
	{ CCI_REG8(0x0326), 0xce },
	{ CCI_REG8(0x0327), 0x04 },
	{ CCI_REG8(0x0329), 0x02 },
	{ CCI_REG8(0x032a), 0x04 },
	{ CCI_REG8(0x032b), 0x04 },
	{ CCI_REG8(0x032c), 0x02 },
	{ CCI_REG8(0x032d), 0x01 },
	{ CCI_REG8(0x032e), 0x00 },
	{ CCI_REG8(0x300d), 0x02 },
	{ CCI_REG8(0x300e), 0x04 },
	{ CCI_REG8(0x3021), 0x08 },
	{ CCI_REG8(0x301e), 0x03 },
	{ CCI_REG8(0x3103), 0x00 },
	{ CCI_REG8(0x3106), 0x08 },
	{ CCI_REG8(0x3107), 0x40 },
	{ CCI_REG8(0x3216), 0x01 },
	{ CCI_REG8(0x3217), 0x00 },
	{ CCI_REG8(0x3218), 0xc0 },
	{ CCI_REG8(0x3219), 0x55 },
	{ CCI_REG8(0x3500), 0x00 },
	{ CCI_REG8(0x3501), 0x04 },
	{ CCI_REG8(0x3502), 0x8a },
	{ CCI_REG8(0x3506), 0x01 },
	{ CCI_REG8(0x3507), 0x72 },
	{ CCI_REG8(0x3508), 0x01 },
	{ CCI_REG8(0x3509), 0x00 },
	{ CCI_REG8(0x350a), 0x01 },
	{ CCI_REG8(0x350b), 0x00 },
	{ CCI_REG8(0x350c), 0x00 },
	{ CCI_REG8(0x3541), 0x00 },
	{ CCI_REG8(0x3542), 0x40 },
	{ CCI_REG8(0x3605), 0xe0 },
	{ CCI_REG8(0x3606), 0x41 },
	{ CCI_REG8(0x3614), 0x20 },
	{ CCI_REG8(0x3620), 0x0b },
	{ CCI_REG8(0x3630), 0x07 },
	{ CCI_REG8(0x3636), 0xa0 },
	{ CCI_REG8(0x3637), 0xf9 },
	{ CCI_REG8(0x3638), 0x09 },
	{ CCI_REG8(0x3639), 0x38 },
	{ CCI_REG8(0x363f), 0x09 },
	{ CCI_REG8(0x3640), 0x17 },
	{ CCI_REG8(0x3665), 0x80 },
	{ CCI_REG8(0x3670), 0x68 },
	{ CCI_REG8(0x3674), 0x00 },
	{ CCI_REG8(0x3677), 0x3f },
	{ CCI_REG8(0x3679), 0x00 },
	{ CCI_REG8(0x369f), 0x19 },
	{ CCI_REG8(0x36a0), 0x03 },
	{ CCI_REG8(0x36a2), 0x19 },
	{ CCI_REG8(0x36a3), 0x03 },
	{ CCI_REG8(0x370d), 0x66 },
	{ CCI_REG8(0x370f), 0x00 },
	{ CCI_REG8(0x3710), 0x03 },
	{ CCI_REG8(0x3715), 0x03 },
	{ CCI_REG8(0x3716), 0x03 },
	{ CCI_REG8(0x3717), 0x06 },
	{ CCI_REG8(0x3733), 0x00 },
	{ CCI_REG8(0x3778), 0x00 },
	{ CCI_REG8(0x37a8), 0x0f },
	{ CCI_REG8(0x37a9), 0x01 },
	{ CCI_REG8(0x37aa), 0x07 },
	{ CCI_REG8(0x37bd), 0x1c },
	{ CCI_REG8(0x37c1), 0x2f },
	{ CCI_REG8(0x37c3), 0x09 },
	{ CCI_REG8(0x37c8), 0x1d },
	{ CCI_REG8(0x37ca), 0x30 },
	{ CCI_REG8(0x37df), 0x00 },
	{ CCI_REG8(0x3800), 0x00 },
	{ CCI_REG8(0x3801), 0x00 },
	{ CCI_REG8(0x3802), 0x00 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x05 },
	{ CCI_REG8(0x3805), 0x0f },
	{ CCI_REG8(0x3806), 0x04 },
	{ CCI_REG8(0x3807), 0x0f },
	{ CCI_REG8(0x3808), 0x05 },
	{ CCI_REG8(0x3809), 0x00 },
	{ CCI_REG8(0x380a), 0x04 },
	{ CCI_REG8(0x380b), 0x00 },
	{ CCI_REG8(0x380c), 0x03 },
	{ CCI_REG8(0x380d), 0x50 },
	{ CCI_REG8(0x380e), 0x04 },
	{ CCI_REG8(0x380f), 0x98 },
	{ CCI_REG8(0x3810), 0x00 },
	{ CCI_REG8(0x3811), 0x08 },
	{ CCI_REG8(0x3812), 0x00 },
	{ CCI_REG8(0x3813), 0x08 },
	{ CCI_REG8(0x3814), 0x11 },
	{ CCI_REG8(0x3815), 0x11 },
	{ CCI_REG8(0x3820), 0x40 },
	{ CCI_REG8(0x3821), 0x04 },
	{ CCI_REG8(0x3826), 0x00 },
	{ CCI_REG8(0x3827), 0x00 },
	{ CCI_REG8(0x382a), 0x08 },
	{ CCI_REG8(0x382b), 0x52 },
	{ CCI_REG8(0x382d), 0xba },
	{ CCI_REG8(0x383d), 0x14 },
	{ CCI_REG8(0x384a), 0xa2 },
	{ CCI_REG8(0x3866), 0x0e },
	{ CCI_REG8(0x3867), 0x07 },
	{ CCI_REG8(0x3884), 0x00 },
	{ CCI_REG8(0x3885), 0x08 },
	{ CCI_REG8(0x3893), 0x68 },
	{ CCI_REG8(0x3894), 0x2a },
	{ CCI_REG8(0x3898), 0x00 },
	{ CCI_REG8(0x3899), 0x31 },
	{ CCI_REG8(0x389a), 0x04 },
	{ CCI_REG8(0x389b), 0x00 },
	{ CCI_REG8(0x389c), 0x0b },
	{ CCI_REG8(0x389d), 0xad },
	{ CCI_REG8(0x389f), 0x08 },
	{ CCI_REG8(0x38a0), 0x00 },
	{ CCI_REG8(0x38a1), 0x00 },
	{ CCI_REG8(0x38a8), 0x70 },
	{ CCI_REG8(0x38ac), 0xea },
	{ CCI_REG8(0x38b2), 0x00 },
	{ CCI_REG8(0x38b3), 0x08 },
	{ CCI_REG8(0x38bc), 0x20 },
	{ CCI_REG8(0x38c4), 0x0c },
	{ CCI_REG8(0x38c5), 0x3a },
	{ CCI_REG8(0x38c7), 0x3a },
	{ CCI_REG8(0x38e1), 0xc0 },
	{ CCI_REG8(0x38ec), 0x3c },
	{ CCI_REG8(0x38f0), 0x09 },
	{ CCI_REG8(0x38f1), 0x6f },
	{ CCI_REG8(0x38fe), 0x3c },
	{ CCI_REG8(0x391e), 0x00 },
	{ CCI_REG8(0x391f), 0x00 },
	{ CCI_REG8(0x3920), 0xa5 },
	{ CCI_REG8(0x3921), 0x00 },
	{ CCI_REG8(0x3922), 0x00 },
	{ CCI_REG8(0x3923), 0x00 },
	{ CCI_REG8(0x3924), 0x05 },
	{ CCI_REG8(0x3925), 0x00 },
	{ CCI_REG8(0x3926), 0x00 },
	{ CCI_REG8(0x3927), 0x00 },
	{ CCI_REG8(0x3928), 0x1a },
	{ CCI_REG8(0x3929), 0x01 },
	{ CCI_REG8(0x392a), 0xb4 },
	{ CCI_REG8(0x392b), 0x00 },
	{ CCI_REG8(0x392c), 0x10 },
	{ CCI_REG8(0x392f), 0x40 },
	{ CCI_REG8(0x4000), 0xcf },
	{ CCI_REG8(0x4003), 0x40 },
	{ CCI_REG8(0x4008), 0x00 },
	{ CCI_REG8(0x4009), 0x07 },
	{ CCI_REG8(0x400a), 0x02 },
	{ CCI_REG8(0x400b), 0x54 },
	{ CCI_REG8(0x400c), 0x00 },
	{ CCI_REG8(0x400d), 0x07 },
	{ CCI_REG8(0x4010), 0xc0 },
	{ CCI_REG8(0x4012), 0x02 },
	{ CCI_REG8(0x4014), 0x04 },
	{ CCI_REG8(0x4015), 0x04 },
	{ CCI_REG8(0x4017), 0x02 },
	{ CCI_REG8(0x4042), 0x01 },
	{ CCI_REG8(0x4306), 0x04 },
	{ CCI_REG8(0x4307), 0x12 },
	{ CCI_REG8(0x4509), 0x00 },
	{ CCI_REG8(0x450b), 0x83 },
	{ CCI_REG8(0x4604), 0x68 },
	{ CCI_REG8(0x4608), 0x0a },
	{ CCI_REG8(0x4700), 0x06 },
	{ CCI_REG8(0x4800), 0x64 },
	{ CCI_REG8(0x481b), 0x3c },
	{ CCI_REG8(0x4825), 0x32 },
	{ CCI_REG8(0x4833), 0x18 },
	{ CCI_REG8(0x4850), 0x40 },
	{ CCI_REG8(0x4860), 0x00 },
	{ CCI_REG8(0x4861), 0xec },
	{ CCI_REG8(0x4864), 0x00 },
	{ CCI_REG8(0x4883), 0x00 },
	{ CCI_REG8(0x4888), 0x90 },
	{ CCI_REG8(0x4889), 0x05 },
	{ CCI_REG8(0x488b), 0x04 },
	{ CCI_REG8(0x4f00), 0x04 },
	{ CCI_REG8(0x4f10), 0x04 },
	{ CCI_REG8(0x4f21), 0x01 },
	{ CCI_REG8(0x4f22), 0x40 },
	{ CCI_REG8(0x4f23), 0x44 },
	{ CCI_REG8(0x4f24), 0x51 },
	{ CCI_REG8(0x4f25), 0x41 },
	{ CCI_REG8(0x5000), 0x1f },
	{ CCI_REG8(0x500a), 0x00 },
	{ CCI_REG8(0x5100), 0x00 },
	{ CCI_REG8(0x5111), 0x20 },
	{ CCI_REG8(0x3020), 0x20 },
	{ CCI_REG8(0x3613), 0x03 },
	{ CCI_REG8(0x38c9), 0x02 },
	{ CCI_REG8(0x5304), 0x01 },
	{ CCI_REG8(0x3620), 0x08 },
	{ CCI_REG8(0x3639), 0x58 },
	{ CCI_REG8(0x363a), 0x10 },
	{ CCI_REG8(0x3674), 0x04 },
	{ CCI_REG8(0x3780), 0xff },
	{ CCI_REG8(0x3781), 0xff },
	{ CCI_REG8(0x3782), 0x00 },
	{ CCI_REG8(0x3783), 0x01 },
	{ CCI_REG8(0x3798), 0xa3 },
	{ CCI_REG8(0x37aa), 0x10 },
	{ CCI_REG8(0x38a8), 0xf0 },
	{ CCI_REG8(0x38c4), 0x09 },
	{ CCI_REG8(0x38c5), 0xb0 },
	{ CCI_REG8(0x38df), 0x80 },
	{ CCI_REG8(0x38ff), 0x05 },
	{ CCI_REG8(0x4010), 0xf1 },
	{ CCI_REG8(0x4011), 0x70 },
	{ CCI_REG8(0x3667), 0x80 },
	{ CCI_REG8(0x4d00), 0x4a },
	{ CCI_REG8(0x4d01), 0x18 },
	{ CCI_REG8(0x4d02), 0xbb },
	{ CCI_REG8(0x4d03), 0xde },
	{ CCI_REG8(0x4d04), 0x93 },
	{ CCI_REG8(0x4d05), 0xff },
	{ CCI_REG8(0x4d09), 0x0a },
	{ CCI_REG8(0x37aa), 0x16 },
	{ CCI_REG8(0x3606), 0x42 },
	{ CCI_REG8(0x3605), 0x00 },
	{ CCI_REG8(0x36a2), 0x17 },
	{ CCI_REG8(0x300d), 0x0a },
	{ CCI_REG8(0x4d00), 0x4d },
	{ CCI_REG8(0x4d01), 0x95 },
	{ CCI_REG8(0x3d8c), 0x70 },
	{ CCI_REG8(0x3d8d), 0xe9 },
	{ CCI_REG8(0x5300), 0x00 },
	{ CCI_REG8(0x5301), 0x10 },
	{ CCI_REG8(0x5302), 0x00 },
	{ CCI_REG8(0x5303), 0xe3 },
	{ CCI_REG8(0x3d88), 0x00 },
	{ CCI_REG8(0x3d89), 0x10 },
	{ CCI_REG8(0x3d8a), 0x00 },
	{ CCI_REG8(0x3d8b), 0xe3 },
	{ CCI_REG8(0x4f22), 0x00 },
};

static const struct cci_reg_sequence mode_1280x1024_regs[] = {
	{ CCI_REG8(0x3501), 0x04 },
	{ CCI_REG8(0x3502), 0x8a },
	{ CCI_REG8(0x3808), 0x05 },
	{ CCI_REG8(0x3809), 0x00 },
	{ CCI_REG8(0x380a), 0x04 },
	{ CCI_REG8(0x380b), 0x00 },
	{ CCI_REG8(0x380c), 0x03 },
	{ CCI_REG8(0x380d), 0x50 },
	{ CCI_REG8(0x380e), 0x04 },
	{ CCI_REG8(0x380f), 0x98 },
	{ CCI_REG8(0x3811), 0x08 },
	{ CCI_REG8(0x3813), 0x08 },
	{ CCI_REG8(0x3814), 0x11 },
	{ CCI_REG8(0x3815), 0x11 },
	{ CCI_REG8(0x4509), 0x00 },
};

/* 2x2 binning */
static const struct cci_reg_sequence mode_640x512_regs[] = {
	{ CCI_REG8(0x3501), 0x02 },
	{ CCI_REG8(0x3502), 0x73 },
	{ CCI_REG8(0x3808), 0x02 },
	{ CCI_REG8(0x3809), 0x80 },
	{ CCI_REG8(0x380a), 0x02 },
	{ CCI_REG8(0x380b), 0x00 },
	{ CCI_REG8(0x380c), 0x03 },
	{ CCI_REG8(0x380d), 0x0c },
	{ CCI_REG8(0x380e), 0x02 },
	{ CCI_REG8(0x380f), 0x81 },
	{ CCI_REG8(0x3811), 0x04 },
	{ CCI_REG8(0x3813), 0x04 },
	{ CCI_REG8(0x3814), 0x22 },
	{ CCI_REG8(0x3815), 0x22 },
	{ CCI_REG8(0x4509), 0x80 },
};

/*
 * 1056x1024 cropped from the center of the pixel array, the window and timing
 * settings and the other settings below are the ones configured by the vendor
 * userspace with the og01a1b-a deckard driver.
 */
static const struct cci_reg_sequence mode_1056x1024_regs[] = {
	{ CCI_REG8(0x0106), 0x00 },
	{ CCI_REG8(0x3107), 0x00 },
	{ CCI_REG8(0x3501), 0x04 },
	{ CCI_REG8(0x3502), 0x8a },
	{ CCI_REG8(0x3506), 0x00 },
	{ CCI_REG8(0x3507), 0x00 },
	{ CCI_REG8(0x3800), 0x00 },
	{ CCI_REG8(0x3801), 0x78 },
	{ CCI_REG8(0x3802), 0x00 },
	{ CCI_REG8(0x3803), 0x00 },
	{ CCI_REG8(0x3804), 0x04 },
	{ CCI_REG8(0x3805), 0x97 },
	{ CCI_REG8(0x3806), 0x04 },
	{ CCI_REG8(0x3807), 0x0f },
	{ CCI_REG8(0x3808), 0x04 },
	{ CCI_REG8(0x3809), 0x20 },
	{ CCI_REG8(0x380a), 0x04 },
	{ CCI_REG8(0x380b), 0x00 },
	{ CCI_REG8(0x380c), 0x03 },
	{ CCI_REG8(0x380d), 0x64 },
	{ CCI_REG8(0x380e), 0x0d },
	{ CCI_REG8(0x380f), 0x6a },
	{ CCI_REG8(0x3810), 0x00 },
	{ CCI_REG8(0x3811), 0x00 },
	{ CCI_REG8(0x3812), 0x00 },
	{ CCI_REG8(0x3813), 0x08 },
	{ CCI_REG8(0x3814), 0x11 },
	{ CCI_REG8(0x3815), 0x11 },
	{ CCI_REG8(0x3820), 0x60 },
	{ CCI_REG8(0x4003), 0x10 },
	{ CCI_REG8(0x4011), 0x00 },
	{ CCI_REG8(0x4509), 0x00 },
};

/* Strobe pulse configuration for driving an IR illuminator */
static const struct cci_reg_sequence og01a1b_strobe_regs[] = {
	{ CCI_REG8(0x3017), 0xf2 },
	{ CCI_REG8(0x3920), 0xff },
	{ CCI_REG8(0x3924), 0x00 },
	{ CCI_REG8(0x3923), 0x00 },
	{ CCI_REG8(0x3922), 0x00 },
	{ CCI_REG8(0x3921), 0x00 },
	{ CCI_REG8(0x392f), 0x42 },
};

static const char * const og01a1b_test_pattern_menu[] = {
	"Disabled",
	"Standard Color Bar",
	"Top-Bottom Darker Color Bar",
	"Right-Left Darker Color Bar",
	"Bottom-Top Darker Color Bar"
};

static const s64 link_freq_menu_items[] = {
	OG01A1B_LINK_FREQ_500MHZ,
};

static const struct og01a1b_mclk_config mclk_configs[] = {
	{
		.mclk = 19200000,
		.reg_list = {
			.num_of_regs =
				ARRAY_SIZE(mipi_data_rate_1000mbps_mclk_19_2mhz),
			.regs = mipi_data_rate_1000mbps_mclk_19_2mhz,
		},
	},
	{
		.mclk = 24000000,
		.reg_list = {
			.num_of_regs =
				ARRAY_SIZE(mipi_data_rate_1000mbps_mclk_24mhz),
			.regs = mipi_data_rate_1000mbps_mclk_24mhz,
		},
	},
};

static const struct og01a1b_mode supported_modes[] = {
	{
		.width = 1280,
		.height = 1024,
		.hts = 848,
		.vts_def = OG01A1B_VTS_120FPS,
		.vts_min = OG01A1B_VTS_120FPS_MIN,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1280x1024_regs),
			.regs = mode_1280x1024_regs,
		},
		.link_freq_index = OG01A1B_LINK_FREQ_1000MBPS,
	},
	{
		.width = 1056,
		.height = 1024,
		.hts = 868,
		.vts_def = OG01A1B_VTS_40FPS,
		.vts_min = OG01A1B_VTS_120FPS_MIN,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1056x1024_regs),
			.regs = mode_1056x1024_regs,
		},
		.link_freq_index = OG01A1B_LINK_FREQ_1000MBPS,
	},
	{
		.width = 640,
		.height = 512,
		.hts = 780,
		.vts_def = OG01A1B_VTS_240FPS,
		.vts_min = OG01A1B_VTS_240FPS_MIN,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640x512_regs),
			.regs = mode_640x512_regs,
		},
		.link_freq_index = OG01A1B_LINK_FREQ_1000MBPS,
	},
};

#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
/*
 * Vendor interface, compatible with the og01a1b-a deckard driver.
 * The ioctl numbers, the structure layouts and the control IDs are an ABI
 * shared with the deckard userspace and the ISP.
 */

#ifndef V4L2_CID_METADATA_CB_ADDR
#define V4L2_CID_METADATA_CB_ADDR	(V4L2_CID_IMAGE_PROC_CLASS_BASE + 6)
#endif
#ifndef V4L2_CID_METADATA_SIZE
#define V4L2_CID_METADATA_SIZE		(V4L2_CID_IMAGE_PROC_CLASS_BASE + 7)
#endif
#ifndef V4L2_CID_SENSOR_SOF_CB_ADDR
#define V4L2_CID_SENSOR_SOF_CB_ADDR	(V4L2_CID_IMAGE_PROC_CLASS_BASE + 8)
#endif

struct og01a1b_deckard_register {
	u16 reg_addr;
	u32 reg_value;
	u8 len;
};

#define VIDIOC_OG01A1B_PRV_G_METADATA_VERSION \
	_IOR('V', BASE_VIDIOC_PRIVATE + 1, u32)
#define VIDIOC_OG01A1B_PRV_S_IR_EMITTER_MODE \
	_IOW('V', BASE_VIDIOC_PRIVATE + 2, u32)
/* Argument: register address in bits [15:0], value in bits [23:16] */
#define VIDIOC_OG01A1B_PRV_S_REG \
	_IO('V', BASE_VIDIOC_PRIVATE + 3)
/* Argument: as above, and group in bits [27:24] */
#define VIDIOC_OG01A1B_PRV_S_GROUP_REG \
	_IO('V', BASE_VIDIOC_PRIVATE + 4)
/* Argument: group mode */
#define VIDIOC_OG01A1B_PRV_S_GROUP_MODE \
	_IO('V', BASE_VIDIOC_PRIVATE + 5)
#define VIDIOC_OG01A1B_PRV_G_DRIVER_VERSION \
	_IOR('V', BASE_VIDIOC_PRIVATE + 6, u32)
#define VIDIOC_SENSOR_PRV_G_REGISTER \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 8, struct og01a1b_deckard_register)
/* Argument: 1 if the sensors are in sync, 0 otherwise */
#define VIDIOC_SENSOR_PRV_S_IN_SYNC \
	_IO('V', BASE_VIDIOC_PRIVATE + 9)

#define OG01A1B_METADATA_VERSION	((3 << 16) | 1)
#define OG01A1B_DRIVER_VERSION	((1 << 16) | 1)

#define OG01A1B_NUM_GROUPS	3
#define OG01A1B_GROUP_MAX_REGS	20
#define OG01A1B_HISTORY		5

/*
 * Frame metadata buffer of the deckard driver. Only the history of the
 * register group updates is filled in, the rest of the buffer is zero.
 */
#define OG01A1B_METADATA_SIZE		1208
#define OG01A1B_METADATA_HISTORY_OFFSET	24

/* Group hold */
#define OG01A1B_REG_GROUP_ACCESS	CCI_REG8(0x3208)
#define OG01A1B_GROUP_HOLD_START	0x00
#define OG01A1B_GROUP_HOLD_END		0x10

#define OG01A1B_REG_STROBE		CCI_REG8(0x3920)
#define OG01A1B_STROBE_ON		0xff
#define OG01A1B_STROBE_OFF		0x00

/* Broadcast address of all sensors on the bus */
#define OG01A1B_DUAL_I2C_ADDR		0x70

#define OG01A1B_SYNC_SENSOR_ID		"slam_left"

struct og01a1b_deckard_register_set {
	u16 addr[OG01A1B_GROUP_MAX_REGS];
	u8 val[OG01A1B_GROUP_MAX_REGS];
	u16 num_regs;
};

struct og01a1b_deckard_groups {
	int group_written;
	u64 pre_group_write_timestamp;
	u64 post_group_write_timestamp;
	u64 sof_ts;
	u64 sof_isr_ts;
	struct og01a1b_deckard_register_set sets[OG01A1B_NUM_GROUPS];
};

/* Layouts of the deckard driver on LP64 architectures */
static_assert(sizeof(struct og01a1b_deckard_register) == 12);
static_assert(sizeof(struct og01a1b_deckard_register_set) == 62);
static_assert(sizeof(struct og01a1b_deckard_groups) == 232);
static_assert(offsetof(struct og01a1b_deckard_groups, sets) == 40);
static_assert(OG01A1B_METADATA_HISTORY_OFFSET +
	      OG01A1B_HISTORY * sizeof(struct og01a1b_deckard_groups) <=
	      OG01A1B_METADATA_SIZE);

enum og01a1b_deckard_ir_mode {
	OG01A1B_IR_MODE_OFF = 0,
	OG01A1B_IR_MODE_GROUP_1_ONLY = 1,
	OG01A1B_IR_MODE_GROUP_0_AND_2_ONLY = 2,
	OG01A1B_IR_MODE_ALL_FRAMES = 3,
};

#define OG01A1B_MAX_PENDING	256

struct og01a1b_deckard {
	/* Serialises group data and register writes around group holds */
	struct mutex lock;
	struct work_struct work;

	bool is_sync_sensor;
	bool dual_i2c_main;
	u32 queue_index;
	bool ctrl_setup;

	u32 ir_mode;
	bool ir_gpio_requested;

	u32 group_mode;
	bool update_group[OG01A1B_NUM_GROUPS];
	struct og01a1b_deckard_groups active;

	u64 last_sof_ts;
	u64 last_sof_isr_ts;
	int last_vc;

	/* Protects metadata against the receiver driver callbacks */
	spinlock_t metadata_lock;
	struct og01a1b_deckard_groups history[OG01A1B_HISTORY];
	u8 metadata[OG01A1B_METADATA_SIZE] __aligned(8);

	/* Single menu items of the callback and metadata size controls */
	s64 metadata_cb_address;
	s64 metadata_size;
	s64 sof_cb_address;

	/* Raw register writes done while the sensor isn't powered */
	struct cci_reg_sequence pending[OG01A1B_MAX_PENDING];
	unsigned int num_pending;
};
#endif

struct og01a1b {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xvclk;
	const struct og01a1b_mclk_config *mclk_cfg;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *ircontrol_gpio;
	bool strobe_oe;
	bool streaming;
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
	const struct og01a1b_mode *cur_mode;

	/* Selected media bus format output */
	u32 code;

#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
	struct og01a1b_deckard v;
#endif
};

#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
static bool og01a1b_deckard_ir_enabled(struct og01a1b *og01a1b)
{
	return READ_ONCE(og01a1b->v.ir_mode) != OG01A1B_IR_MODE_OFF;
}

static bool og01a1b_deckard_ir_gpio_requested(struct og01a1b *og01a1b)
{
	return READ_ONCE(og01a1b->v.ir_gpio_requested);
}

static void og01a1b_deckard_lock(struct og01a1b *og01a1b)
{
	mutex_lock(&og01a1b->v.lock);
}

static void og01a1b_deckard_unlock(struct og01a1b *og01a1b)
{
	mutex_unlock(&og01a1b->v.lock);
}

static void og01a1b_deckard_ctrl_setup(struct og01a1b *og01a1b, bool setup)
{
	og01a1b->v.ctrl_setup = setup;
}
#else
static bool og01a1b_deckard_ir_enabled(struct og01a1b *og01a1b)
{
	return false;
}

static bool og01a1b_deckard_ir_gpio_requested(struct og01a1b *og01a1b)
{
	return false;
}

static void og01a1b_deckard_lock(struct og01a1b *og01a1b)
{
}

static void og01a1b_deckard_unlock(struct og01a1b *og01a1b)
{
}

static void og01a1b_deckard_ctrl_setup(struct og01a1b *og01a1b, bool setup)
{
}
#endif

static u64 to_pixel_rate(u32 f_index, u32 bpp)
{
	u64 pixel_rate = link_freq_menu_items[f_index] * 2 * OG01A1B_DATA_LANES;

	do_div(pixel_rate, bpp);

	return pixel_rate;
}

static u64 to_pixels_per_line(u32 hts, u32 f_index, u32 bpp)
{
	u64 ppl = hts * to_pixel_rate(f_index, bpp);

	do_div(ppl, OG01A1B_SCLK);

	return ppl;
}

static int og01a1b_test_pattern(struct og01a1b *og01a1b, u32 pattern)
{
	if (pattern)
		pattern = (pattern - 1) << OG01A1B_TEST_PATTERN_BAR_SHIFT |
			  OG01A1B_TEST_PATTERN_ENABLE;

	return cci_write(og01a1b->regmap, OG01A1B_REG_TEST_PATTERN,
			 pattern, NULL);
}

static int og01a1b_update_strobe(struct og01a1b *og01a1b)
{
	bool enable = og01a1b->strobe_oe || og01a1b_deckard_ir_enabled(og01a1b);
	int ret = 0;

	if (enable)
		cci_multi_reg_write(og01a1b->regmap, og01a1b_strobe_regs,
				    ARRAY_SIZE(og01a1b_strobe_regs), &ret);

	cci_update_bits(og01a1b->regmap, OG01A1B_REG_OUTPUT_ENABLE,
			OG01A1B_OUTPUT_ENABLE_STROBE,
			enable ? OG01A1B_OUTPUT_ENABLE_STROBE : 0, &ret);

	return ret;
}

static void og01a1b_update_ircontrol(struct og01a1b *og01a1b)
{
	bool enable = (og01a1b->strobe_oe && og01a1b->streaming) ||
		      og01a1b_deckard_ir_gpio_requested(og01a1b);

	gpiod_set_value_cansleep(og01a1b->ircontrol_gpio, enable);
}

#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
/* Protects the sensor count, the dual client and the workqueues */
static DEFINE_MUTEX(og01a1b_deckard_mutex);
static unsigned int og01a1b_deckard_num_sensors;
static struct i2c_client *og01a1b_dual_client;
static struct regmap *og01a1b_dual_regmap;

/* Shared by all sensors, as in the deckard driver */
static DEFINE_SPINLOCK(og01a1b_last_set_group_lock);
static int og01a1b_last_set_group;
static u64 og01a1b_last_time_in_sync;
static u64 og01a1b_last_dual_write_time;

/* Shared with the og0ve10-a deckard driver, which queues its work here */
struct workqueue_struct *sensor_workqueues[2];
EXPORT_SYMBOL(sensor_workqueues);

static const struct regmap_config og01a1b_dual_regmap_config = {
	.name = "og01a1b-dual",
	.reg_bits = 16,
	.val_bits = 8,
};

static void og01a1b_deckard_mark_groups(struct og01a1b_deckard *v)
{
	unsigned int i;

	for (i = 0; i < OG01A1B_NUM_GROUPS; i++)
		v->update_group[i] = true;
}

static void og01a1b_deckard_reset_groups(struct og01a1b_deckard *v)
{
	unsigned long flags;
	unsigned int i;

	v->active.pre_group_write_timestamp = 0;
	v->active.post_group_write_timestamp = 0;
	memset(v->active.sets, 0, sizeof(v->active.sets));

	spin_lock_irqsave(&v->metadata_lock, flags);
	for (i = 0; i < OG01A1B_HISTORY; i++)
		v->history[i] = v->active;
	spin_unlock_irqrestore(&v->metadata_lock, flags);
}

static int og01a1b_deckard_set_group_reg(struct og01a1b *og01a1b, u32 arg)
{
	struct og01a1b_deckard *v = &og01a1b->v;
	u16 addr = arg & 0xffff;
	u8 val = (arg >> 16) & 0xff;
	u8 group = (arg >> 24) & 0xf;
	struct og01a1b_deckard_register_set *set;
	unsigned int i;

	if (group >= OG01A1B_NUM_GROUPS)
		return -EINVAL;

	set = &v->active.sets[group];

	for (i = 0; i < set->num_regs; i++)
		if (set->addr[i] == addr)
			break;

	if (i == set->num_regs) {
		if (set->num_regs >= OG01A1B_GROUP_MAX_REGS) {
			dev_err(og01a1b->dev,
				"Group register map exceeded maximum size\n");
			return -ENOSPC;
		}

		set->addr[i] = addr;
		set->num_regs++;
		v->update_group[group] = true;
	}

	if (set->val[i] != val) {
		set->val[i] = val;
		v->update_group[group] = true;
	}

	return 0;
}

static bool og01a1b_deckard_group_strobe(struct og01a1b_deckard *v,
					unsigned int group)
{
	switch (READ_ONCE(v->ir_mode)) {
	case OG01A1B_IR_MODE_ALL_FRAMES:
		return true;
	case OG01A1B_IR_MODE_GROUP_1_ONLY:
		return group == 1;
	case OG01A1B_IR_MODE_GROUP_0_AND_2_ONLY:
		return group == 0 || group == 2;
	default:
		return false;
	}
}

/*
 * Write the changed register groups with the group hold function. With
 * use_dual set, the groups are written to all sensors on the bus.
 */
static int og01a1b_deckard_write_groups(struct og01a1b *og01a1b, bool use_dual)
{
	struct og01a1b_deckard *v = &og01a1b->v;
	struct regmap *regmap = use_dual ? og01a1b_dual_regmap : og01a1b->regmap;
	unsigned int group, i;
	int ret;

	if (!regmap)
		return -ENODEV;

	for (group = 0; group < OG01A1B_NUM_GROUPS; group++) {
		const struct og01a1b_deckard_register_set *set =
			&v->active.sets[group];

		if (!v->update_group[group])
			continue;

		v->update_group[group] = false;

		ret = cci_write(regmap, OG01A1B_REG_GROUP_ACCESS,
				OG01A1B_GROUP_HOLD_START | group, NULL);
		if (ret)
			return ret;

		for (i = 0; i < set->num_regs; i++) {
			if (i + 1 < set->num_regs &&
			    set->addr[i + 1] == set->addr[i] + 1) {
				/* Write adjacent registers in one transfer */
				ret = cci_write(regmap, CCI_REG16(set->addr[i]),
						set->val[i] << 8 | set->val[i + 1],
						NULL);
				i++;
			} else {
				ret = cci_write(regmap, CCI_REG8(set->addr[i]),
						set->val[i], NULL);
			}
			if (ret)
				return ret;
		}

		ret = cci_write(regmap, OG01A1B_REG_STROBE,
				og01a1b_deckard_group_strobe(v, group) ?
				OG01A1B_STROBE_ON : OG01A1B_STROBE_OFF, NULL);
		if (ret)
			return ret;

		ret = cci_write(regmap, OG01A1B_REG_GROUP_ACCESS,
				OG01A1B_GROUP_HOLD_END | group, NULL);
		if (ret)
			return ret;
	}

	return 0;
}

static void og01a1b_deckard_work(struct work_struct *work)
{
	struct og01a1b_deckard *v = container_of(work, struct og01a1b_deckard,
						work);
	struct og01a1b *og01a1b = container_of(v, struct og01a1b, v);
	u64 pre_write_ts, post_write_ts, in_sync;
	unsigned long flags;
	bool powered;
	int group, ret = 0;

	powered = pm_runtime_get_if_in_use(og01a1b->dev) > 0;

	mutex_lock(&v->lock);

	pre_write_ts = ktime_get_raw_ns();

	if (powered) {
		in_sync = READ_ONCE(og01a1b_last_time_in_sync);
		if (!in_sync) {
			/* Not in sync, each sensor writes its own groups */
			ret = og01a1b_deckard_write_groups(og01a1b, false);
		} else if (v->dual_i2c_main) {
			/* The main sensor writes the groups of all sensors */
			ret = og01a1b_deckard_write_groups(og01a1b, true);
			WRITE_ONCE(og01a1b_last_dual_write_time,
				   ktime_get_raw_ns());
		} else if (READ_ONCE(og01a1b_last_dual_write_time) < in_sync) {
			/* No write by the main sensor since getting in sync */
			ret = og01a1b_deckard_write_groups(og01a1b, false);
		}
		if (ret)
			dev_err_ratelimited(og01a1b->dev,
					    "failed to write groups: %d\n", ret);
	}

	spin_lock(&og01a1b_last_set_group_lock);
	group = 0;
	if (v->group_mode)
		group = (og01a1b_last_set_group + 1) % v->group_mode;
	/* Resynchronise with the virtual channel of the last frame */
	if (v->last_vc == 1)
		group = 1;
	og01a1b_last_set_group = group;
	spin_unlock(&og01a1b_last_set_group_lock);

	post_write_ts = ktime_get_raw_ns();

	v->active.group_written = group;
	v->active.pre_group_write_timestamp = pre_write_ts;
	v->active.sof_ts = v->last_sof_ts;
	v->active.sof_isr_ts = v->last_sof_isr_ts;
	v->active.post_group_write_timestamp = post_write_ts;

	spin_lock_irqsave(&v->metadata_lock, flags);
	memmove(&v->history[1], &v->history[0],
		(OG01A1B_HISTORY - 1) * sizeof(v->history[0]));
	v->history[0] = v->active;
	spin_unlock_irqrestore(&v->metadata_lock, flags);

	mutex_unlock(&v->lock);

	if (powered)
		pm_runtime_put(og01a1b->dev);
}

/* Called by the ISP for each frame */
static void *og01a1b_deckard_get_metadata(struct v4l2_subdev *sd, u8 vc)
{
	struct og01a1b_deckard *v = &to_og01a1b(sd)->v;
	unsigned long flags;

	spin_lock_irqsave(&v->metadata_lock, flags);
	memcpy(v->metadata + OG01A1B_METADATA_HISTORY_OFFSET, v->history,
	       sizeof(v->history));
	spin_unlock_irqrestore(&v->metadata_lock, flags);

	return v->metadata;
}

/* Called by the ISP at each start of frame */
static void og01a1b_deckard_sof(struct v4l2_subdev *sd, u64 sof_timestamp,
			       u64 sof_isr_timestamp, u8 vc)
{
	struct og01a1b_deckard *v = &to_og01a1b(sd)->v;
	struct workqueue_struct *wq;

	v->last_sof_ts = sof_timestamp;
	v->last_sof_isr_ts = sof_isr_timestamp;

	v->last_vc = vc;

	wq = READ_ONCE(sensor_workqueues[v->queue_index]);
	if (wq)
		queue_work(wq, &v->work);
}

static int og01a1b_deckard_s_reg(struct og01a1b *og01a1b, u32 arg)
{
	u32 reg = CCI_REG8(arg & 0xffff);
	u8 val = (arg >> 16) & 0xff;

	/* Streaming is started and stopped on all sensors at once */
	if (reg == OG01A1B_REG_MODE_SELECT) {
		if (!og01a1b_dual_regmap)
			return -ENODEV;

		return cci_write(og01a1b_dual_regmap, reg, val, NULL);
	}

	return cci_write(og01a1b->regmap, reg, val, NULL);
}

/*
 * The deckard userspace configures the sensor with raw register writes before
 * stream start, the deckard driver keeps the sensor powered for them with
 * .s_power. Here the writes done while the sensor isn't powered are queued,
 * and written in order at stream start, after the register settings and the
 * controls. Called with the deckard lock held.
 */
static int og01a1b_deckard_queue_reg(struct og01a1b *og01a1b, u32 arg)
{
	struct og01a1b_deckard *v = &og01a1b->v;
	u32 reg = CCI_REG8(arg & 0xffff);

	/* Streaming can't be started or stopped while the sensor is off */
	if (reg == OG01A1B_REG_MODE_SELECT)
		return -EAGAIN;

	if (v->num_pending >= ARRAY_SIZE(v->pending))
		return -ENOSPC;

	v->pending[v->num_pending].reg = reg;
	v->pending[v->num_pending].val = (arg >> 16) & 0xff;
	v->num_pending++;

	return 0;
}

/* Called with the deckard lock held */
static int og01a1b_deckard_write_reg(struct og01a1b *og01a1b, u32 arg)
{
	int ret;

	/* Keep the order of the queued writes */
	if (og01a1b->v.num_pending ||
	    pm_runtime_get_if_in_use(og01a1b->dev) <= 0)
		return og01a1b_deckard_queue_reg(og01a1b, arg);

	ret = og01a1b_deckard_s_reg(og01a1b, arg);
	pm_runtime_put(og01a1b->dev);

	return ret;
}

/* Called at stream start, while the sensor is powered */
static int og01a1b_deckard_write_pending(struct og01a1b *og01a1b)
{
	struct og01a1b_deckard *v = &og01a1b->v;
	int ret = 0;

	mutex_lock(&v->lock);
	cci_multi_reg_write(og01a1b->regmap, v->pending, v->num_pending, &ret);
	v->num_pending = 0;
	mutex_unlock(&v->lock);

	if (ret)
		dev_err(og01a1b->dev, "failed to write queued registers: %d\n",
			ret);

	return ret;
}

static int og01a1b_deckard_g_reg(struct og01a1b *og01a1b,
				struct og01a1b_deckard_register *reg)
{
	static const u32 width[] = {
		CCI_REG8(0), CCI_REG16(0), CCI_REG24(0), CCI_REG32(0),
	};
	u64 val = 0;
	int ret;

	if (!reg->len || reg->len > ARRAY_SIZE(width)) {
		ret = -EINVAL;
	} else {
		ret = cci_read(og01a1b->regmap, width[reg->len - 1] |
			       reg->reg_addr, &val, NULL);
		if (ret)
			dev_err(og01a1b->dev,
				"Failed to read register 0x%04x: %d\n",
				reg->reg_addr, ret);
	}

	reg->reg_value = val;

	return ret;
}

static int og01a1b_deckard_set_ir_mode(struct og01a1b *og01a1b, u32 mode)
{
	struct og01a1b_deckard *v = &og01a1b->v;
	int ret = 0;

	mutex_lock(&v->lock);

	WRITE_ONCE(v->ir_mode, mode);
	WRITE_ONCE(v->ir_gpio_requested, mode != OG01A1B_IR_MODE_OFF);

	/* The strobe is set in the register groups, rewrite all of them */
	og01a1b_deckard_mark_groups(v);

	og01a1b_update_ircontrol(og01a1b);

	if (pm_runtime_get_if_in_use(og01a1b->dev) > 0) {
		ret = og01a1b_update_strobe(og01a1b);
		pm_runtime_put(og01a1b->dev);
	}

	mutex_unlock(&v->lock);

	return ret;
}

static void og01a1b_deckard_set_in_sync(u32 in_sync)
{
	u64 now = ktime_get_raw_ns();
	u64 last = READ_ONCE(og01a1b_last_time_in_sync);

	if (in_sync != 1)
		WRITE_ONCE(og01a1b_last_time_in_sync, 0);
	else
		WRITE_ONCE(og01a1b_last_time_in_sync,
			   last ? min(last, now) : now);
}

static long og01a1b_deckard_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
				 void *arg)
{
	struct og01a1b *og01a1b = to_og01a1b(sd);
	struct og01a1b_deckard *v = &og01a1b->v;
	u32 val = (uintptr_t)arg;
	long ret = 0;

	switch (cmd) {
	case VIDIOC_OG01A1B_PRV_G_METADATA_VERSION:
		*(u32 *)arg = OG01A1B_METADATA_VERSION;
		break;

	case VIDIOC_OG01A1B_PRV_G_DRIVER_VERSION:
		*(u32 *)arg = OG01A1B_DRIVER_VERSION;
		break;

	case VIDIOC_OG01A1B_PRV_S_IR_EMITTER_MODE:
		ret = og01a1b_deckard_set_ir_mode(og01a1b, *(u32 *)arg);
		break;

	case VIDIOC_OG01A1B_PRV_S_REG:
		mutex_lock(&v->lock);
		ret = og01a1b_deckard_write_reg(og01a1b, val);
		mutex_unlock(&v->lock);
		break;

	case VIDIOC_OG01A1B_PRV_S_GROUP_REG:
		mutex_lock(&v->lock);
		ret = og01a1b_deckard_set_group_reg(og01a1b, val);
		mutex_unlock(&v->lock);
		break;

	case VIDIOC_OG01A1B_PRV_S_GROUP_MODE:
		WRITE_ONCE(v->group_mode, val);
		break;

	case VIDIOC_SENSOR_PRV_G_REGISTER:
		/* Reads can't be deferred */
		if (pm_runtime_get_if_in_use(og01a1b->dev) <= 0) {
			ret = -EAGAIN;
			break;
		}
		mutex_lock(&v->lock);
		ret = og01a1b_deckard_g_reg(og01a1b, arg);
		mutex_unlock(&v->lock);
		pm_runtime_put(og01a1b->dev);
		break;

	case VIDIOC_SENSOR_PRV_S_IN_SYNC:
		og01a1b_deckard_set_in_sync(val);
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

/* Called with the deckard lock held, while the sensor is powered */
static int og01a1b_deckard_s_ctrl(struct og01a1b *og01a1b,
				 struct v4l2_ctrl *ctrl)
{
	struct og01a1b_deckard *v = &og01a1b->v;

	switch (ctrl->id) {
	case V4L2_CID_METADATA_CB_ADDR:
	case V4L2_CID_SENSOR_SOF_CB_ADDR:
		return 0;

	/*
	 * Legacy controls for register access, they act on user writes only,
	 * not when the controls are applied at power up.
	 */
	case V4L2_CID_BRIGHTNESS:
		/* Addresses 0 and 1 set a read address, which is unused */
		if (v->ctrl_setup || (ctrl->val & 0xffff) <= 1)
			return 0;
		/* Keep the order of the queued writes */
		if (v->num_pending)
			return og01a1b_deckard_queue_reg(og01a1b, ctrl->val);
		return og01a1b_deckard_s_reg(og01a1b, ctrl->val);

	case V4L2_CID_HUE:
		if (v->ctrl_setup)
			return 0;
		return og01a1b_deckard_set_group_reg(og01a1b, ctrl->val);

	case V4L2_CID_SATURATION:
		if (!v->ctrl_setup)
			WRITE_ONCE(v->group_mode, ctrl->val);
		return 0;

	default:
		return -EINVAL;
	}
}

/* Legacy controls set by userspace while the sensor isn't powered */
static int og01a1b_deckard_s_ctrl_unpowered(struct og01a1b *og01a1b,
					   struct v4l2_ctrl *ctrl)
{
	struct og01a1b_deckard *v = &og01a1b->v;
	int ret = 0;

	mutex_lock(&v->lock);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if ((ctrl->val & 0xffff) > 1)
			ret = og01a1b_deckard_queue_reg(og01a1b, ctrl->val);
		break;

	/* The groups are written while streaming */
	case V4L2_CID_HUE:
		ret = og01a1b_deckard_set_group_reg(og01a1b, ctrl->val);
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

static void og01a1b_deckard_init_controls(struct og01a1b *og01a1b,
					 struct v4l2_ctrl_handler *hdl,
					 const struct v4l2_ctrl_ops *ops)
{
	struct og01a1b_deckard *v = &og01a1b->v;
	struct v4l2_ctrl_config cfg = {
		.ops = ops,
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
	};

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BRIGHTNESS, 0, 0xffffff, 1, 0);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HUE, S32_MIN, S32_MAX, 1, 0);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION, 0, 2, 1, 0);

	v->metadata_cb_address = (s64)(uintptr_t)og01a1b_deckard_get_metadata;
	v->metadata_size = OG01A1B_METADATA_SIZE;
	v->sof_cb_address = (s64)(uintptr_t)og01a1b_deckard_sof;

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

static void og01a1b_deckard_s_stream(struct og01a1b *og01a1b, bool enable)
{
	struct og01a1b_deckard *v = &og01a1b->v;

	WRITE_ONCE(og01a1b_last_time_in_sync, 0);

	mutex_lock(&v->lock);
	/*
	 * The groups and the IR emitter mode set by userspace for the next
	 * stream start are kept.
	 */
	if (!enable) {
		og01a1b_deckard_reset_groups(v);
		if (v->is_sync_sensor)
			WRITE_ONCE(v->ir_mode, OG01A1B_IR_MODE_OFF);
	}
	og01a1b_deckard_mark_groups(v);
	if (!enable)
		WRITE_ONCE(v->ir_gpio_requested, false);
	mutex_unlock(&v->lock);
}

static void og01a1b_deckard_pin_workqueue(struct device *dev,
					 struct workqueue_struct *wq,
					 unsigned int cpu)
{
	/* The workqueue attributes API isn't exported to modules */
#if IS_BUILTIN(CONFIG_VIDEO_OG01A1B)
	struct workqueue_attrs *attrs;

	if (cpu >= nr_cpu_ids) {
		dev_warn(dev, "no CPU %u for sensor workqueue\n", cpu);
		return;
	}

	attrs = alloc_workqueue_attrs();
	if (!attrs)
		return;

	cpumask_clear(attrs->cpumask);
	cpumask_set_cpu(cpu, attrs->cpumask);
	if (apply_workqueue_attrs(wq, attrs))
		dev_warn(dev, "failed to set sensor workqueue affinity\n");

	free_workqueue_attrs(attrs);
#endif
}

static int og01a1b_deckard_alloc_workqueues(struct device *dev)
{
	struct workqueue_struct *wq;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sensor_workqueues); i++) {
		if (sensor_workqueues[i])
			continue;

		wq = alloc_workqueue("sensor_workqueue%u",
				     WQ_UNBOUND | WQ_HIGHPRI | WQ_SYSFS, 1, i);
		if (!wq)
			return -ENOMEM;

		/* CPUs 2 and 3, as in the deckard driver */
		og01a1b_deckard_pin_workqueue(dev, wq, i + 2);

		WRITE_ONCE(sensor_workqueues[i], wq);
	}

	return 0;
}

static void og01a1b_deckard_create_dual_client(struct og01a1b *og01a1b,
					      struct i2c_client *client)
{
	struct i2c_board_info info = {
		I2C_BOARD_INFO("og01a1b-dual", OG01A1B_DUAL_I2C_ADDR),
	};
	struct i2c_client *dual;
	struct regmap *regmap;

	dual = i2c_new_client_device(client->adapter, &info);
	if (IS_ERR(dual)) {
		dev_err(og01a1b->dev, "failed to create dual i2c client: %pe\n",
			dual);
		return;
	}

	regmap = regmap_init_i2c(dual, &og01a1b_dual_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(og01a1b->dev, "failed to init dual i2c regmap: %pe\n",
			regmap);
		i2c_unregister_device(dual);
		return;
	}

	og01a1b_dual_client = dual;
	og01a1b_dual_regmap = regmap;
}

static int og01a1b_deckard_probe(struct og01a1b *og01a1b,
				struct i2c_client *client)
{
	struct og01a1b_deckard *v = &og01a1b->v;
	const char *id;
	u32 val;
	int ret;

	mutex_init(&v->lock);
	spin_lock_init(&v->metadata_lock);
	INIT_WORK(&v->work, og01a1b_deckard_work);

	v->is_sync_sensor =
		!device_property_read_string(og01a1b->dev, "id", &id) &&
		!strcmp(id, OG01A1B_SYNC_SENSOR_ID);

	if (device_property_read_u32(og01a1b->dev, "sensor-queue-index",
				     &val) || val >= ARRAY_SIZE(sensor_workqueues))
		val = 0;
	v->queue_index = val;

	v->dual_i2c_main =
		!device_property_read_u32(og01a1b->dev, "dual-i2c-main", &val) &&
		val;

	mutex_lock(&og01a1b_deckard_mutex);

	ret = og01a1b_deckard_alloc_workqueues(og01a1b->dev);
	if (!ret) {
		/* The dual client is not needed for the sensor to work */
		if (!og01a1b_dual_client)
			og01a1b_deckard_create_dual_client(og01a1b, client);

		og01a1b_deckard_num_sensors++;
	}

	mutex_unlock(&og01a1b_deckard_mutex);

	if (ret)
		mutex_destroy(&v->lock);

	return ret;
}

static void og01a1b_deckard_remove(struct og01a1b *og01a1b)
{
	struct og01a1b_deckard *v = &og01a1b->v;

	cancel_work_sync(&v->work);

	mutex_lock(&og01a1b_deckard_mutex);

	if (!--og01a1b_deckard_num_sensors && og01a1b_dual_client) {
		regmap_exit(og01a1b_dual_regmap);
		i2c_unregister_device(og01a1b_dual_client);
		og01a1b_dual_regmap = NULL;
		og01a1b_dual_client = NULL;
	}

	mutex_unlock(&og01a1b_deckard_mutex);

	mutex_destroy(&v->lock);
}

/* The workqueues may be in use by og0ve10-a until the module is unloaded */
static void og01a1b_deckard_exit(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sensor_workqueues); i++) {
		if (sensor_workqueues[i])
			destroy_workqueue(sensor_workqueues[i]);
		sensor_workqueues[i] = NULL;
	}
}
#else
static int og01a1b_deckard_s_ctrl(struct og01a1b *og01a1b,
				 struct v4l2_ctrl *ctrl)
{
	return -EINVAL;
}

static int og01a1b_deckard_s_ctrl_unpowered(struct og01a1b *og01a1b,
					   struct v4l2_ctrl *ctrl)
{
	return 0;
}

static int og01a1b_deckard_write_pending(struct og01a1b *og01a1b)
{
	return 0;
}

static void og01a1b_deckard_init_controls(struct og01a1b *og01a1b,
					 struct v4l2_ctrl_handler *hdl,
					 const struct v4l2_ctrl_ops *ops)
{
}

static void og01a1b_deckard_s_stream(struct og01a1b *og01a1b, bool enable)
{
}

static int og01a1b_deckard_probe(struct og01a1b *og01a1b,
				struct i2c_client *client)
{
	return 0;
}

static void og01a1b_deckard_remove(struct og01a1b *og01a1b)
{
}
#endif

static int og01a1b_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct og01a1b *og01a1b = container_of(ctrl->handler,
					       struct og01a1b, ctrl_handler);
	s64 exposure_max;
	int ret = 0;

	/* Propagate change of current control to all related controls */
	if (ctrl->id == V4L2_CID_VBLANK) {
		/* Update max exposure while meeting expected vblanking */
		exposure_max = og01a1b->cur_mode->height + ctrl->val -
			       OG01A1B_EXPOSURE_MAX_MARGIN;
		__v4l2_ctrl_modify_range(og01a1b->exposure,
					 og01a1b->exposure->minimum,
					 exposure_max, og01a1b->exposure->step,
					 exposure_max);
	}

	if (ctrl->id == V4L2_CID_FLASH_STROBE_OE) {
		og01a1b->strobe_oe = ctrl->val;
		og01a1b_update_ircontrol(og01a1b);
	}

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(og01a1b->dev))
		return og01a1b_deckard_s_ctrl_unpowered(og01a1b, ctrl);

	/* Don't interfere with register group writes */
	og01a1b_deckard_lock(og01a1b);

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(og01a1b->regmap, OG01A1B_REG_ANALOG_GAIN,
				ctrl->val << 4, NULL);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		ret = cci_write(og01a1b->regmap, OG01A1B_REG_DIG_GAIN,
				ctrl->val << 6, NULL);
		break;

	case V4L2_CID_EXPOSURE:
		ret = cci_write(og01a1b->regmap, OG01A1B_REG_EXPOSURE,
				ctrl->val, NULL);
		break;

	case V4L2_CID_VBLANK:
		ret = cci_write(og01a1b->regmap, OG01A1B_REG_VTS,
				og01a1b->cur_mode->height + ctrl->val, NULL);
		break;

	case V4L2_CID_TEST_PATTERN:
		ret = og01a1b_test_pattern(og01a1b, ctrl->val);
		break;

	case V4L2_CID_FLASH_STROBE_OE:
		ret = og01a1b_update_strobe(og01a1b);
		break;

	default:
		ret = og01a1b_deckard_s_ctrl(og01a1b, ctrl);
		break;
	}

	og01a1b_deckard_unlock(og01a1b);

	pm_runtime_put(og01a1b->dev);

	return ret;
}

static const struct v4l2_ctrl_ops og01a1b_ctrl_ops = {
	.s_ctrl = og01a1b_set_ctrl,
};

static int og01a1b_init_controls(struct og01a1b *og01a1b)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	s64 exposure_max, h_blank;
	u32 bpp;
	int ret;

	ctrl_hdlr = &og01a1b->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 15);
	if (ret)
		return ret;

	og01a1b->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
						    &og01a1b_ctrl_ops,
						    V4L2_CID_LINK_FREQ,
						    ARRAY_SIZE
						    (link_freq_menu_items) - 1,
						    0, link_freq_menu_items);
	if (og01a1b->link_freq)
		og01a1b->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	bpp = (og01a1b->code == MEDIA_BUS_FMT_Y10_1X10 ? 10 : 8);
	og01a1b->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops,
				V4L2_CID_PIXEL_RATE, 0,
				to_pixel_rate(OG01A1B_LINK_FREQ_1000MBPS, bpp),
				1,
				to_pixel_rate(OG01A1B_LINK_FREQ_1000MBPS, bpp));
	og01a1b->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops,
					    V4L2_CID_VBLANK,
					    og01a1b->cur_mode->vts_min -
					    og01a1b->cur_mode->height,
					    OG01A1B_VTS_MAX -
					    og01a1b->cur_mode->height, 1,
					    og01a1b->cur_mode->vts_def -
					    og01a1b->cur_mode->height);
	h_blank = to_pixels_per_line(og01a1b->cur_mode->hts,
				     og01a1b->cur_mode->link_freq_index, bpp) -
				     og01a1b->cur_mode->width;
	og01a1b->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops,
					    V4L2_CID_HBLANK, h_blank, h_blank,
					    1, h_blank);
	if (og01a1b->hblank)
		og01a1b->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OG01A1B_ANAL_GAIN_MIN, OG01A1B_ANAL_GAIN_MAX,
			  OG01A1B_ANAL_GAIN_STEP, OG01A1B_ANAL_GAIN_MIN);
	v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  OG01A1B_DGTL_GAIN_MIN, OG01A1B_DGTL_GAIN_MAX,
			  OG01A1B_DGTL_GAIN_STEP, OG01A1B_DGTL_GAIN_DEFAULT);
	exposure_max = (og01a1b->cur_mode->vts_def -
			OG01A1B_EXPOSURE_MAX_MARGIN);
	og01a1b->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops,
					      V4L2_CID_EXPOSURE,
					      OG01A1B_EXPOSURE_MIN,
					      exposure_max,
					      OG01A1B_EXPOSURE_STEP,
					      exposure_max);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &og01a1b_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(og01a1b_test_pattern_menu) - 1,
				     0, 0, og01a1b_test_pattern_menu);
	v4l2_ctrl_new_std(ctrl_hdlr, &og01a1b_ctrl_ops,
			  V4L2_CID_FLASH_STROBE_OE, 0, 1, 1, 0);
	og01a1b_deckard_init_controls(og01a1b, ctrl_hdlr, &og01a1b_ctrl_ops);

	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	og01a1b->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static void og01a1b_update_pad_format(const struct og01a1b_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
}

static int og01a1b_load_regs(struct og01a1b *og01a1b)
{
	const struct og01a1b_reg_list *reg_list;
	int ret;

	reg_list = &og01a1b->mclk_cfg->reg_list;
	ret = cci_multi_reg_write(og01a1b->regmap, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(og01a1b->dev, "failed to set plls: %d\n", ret);
		return ret;
	}

	ret = cci_multi_reg_write(og01a1b->regmap, og01a1b_common_regs,
				  ARRAY_SIZE(og01a1b_common_regs), NULL);
	if (ret) {
		dev_err(og01a1b->dev, "failed to set common registers: %d\n",
			ret);
		return ret;
	}

	reg_list = &og01a1b->cur_mode->reg_list;
	ret = cci_multi_reg_write(og01a1b->regmap, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(og01a1b->dev, "failed to set mode: %d\n", ret);
		return ret;
	}

	ret = cci_write(og01a1b->regmap, CCI_REG8(0x3662),
			(og01a1b->code == MEDIA_BUS_FMT_Y10_1X10 ? 0x4 : 0x6),
			NULL);
	if (ret)
		dev_err(og01a1b->dev, "failed to set output format: %d\n", ret);

	return ret;
}

static int og01a1b_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct og01a1b *og01a1b = to_og01a1b(sd);
	int ret;

	ret = pm_runtime_resume_and_get(og01a1b->dev);
	if (ret)
		return ret;

	ret = og01a1b_load_regs(og01a1b);
	if (ret)
		goto error;

	og01a1b_deckard_ctrl_setup(og01a1b, true);
	ret = __v4l2_ctrl_handler_setup(og01a1b->sd.ctrl_handler);
	og01a1b_deckard_ctrl_setup(og01a1b, false);
	if (ret)
		goto error;

	ret = og01a1b_deckard_write_pending(og01a1b);
	if (ret)
		goto error;

	og01a1b_deckard_s_stream(og01a1b, true);

	ret = cci_write(og01a1b->regmap, OG01A1B_REG_MODE_SELECT,
			OG01A1B_MODE_STREAMING, NULL);
	if (ret) {
		dev_err(og01a1b->dev, "failed to start streaming: %d\n", ret);
		goto error;
	}

	og01a1b->streaming = true;
	og01a1b_update_ircontrol(og01a1b);

	return 0;

error:
	og01a1b_update_ircontrol(og01a1b);
	pm_runtime_put_autosuspend(og01a1b->dev);

	return ret;
}

static int og01a1b_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct og01a1b *og01a1b = to_og01a1b(sd);
	int ret;

	og01a1b->streaming = false;
	og01a1b_deckard_s_stream(og01a1b, false);
	og01a1b_update_ircontrol(og01a1b);

	ret = cci_write(og01a1b->regmap, OG01A1B_REG_MODE_SELECT,
			OG01A1B_MODE_STANDBY, NULL);
	if (ret)
		dev_err(og01a1b->dev, "failed to stop streaming: %d\n", ret);

	pm_runtime_put_autosuspend(og01a1b->dev);

	return ret;
}

#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
static const struct v4l2_subdev_core_ops og01a1b_core_ops = {
	.ioctl = og01a1b_deckard_ioctl,
};
#endif

static int og01a1b_set_format(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *fmt)
{
	struct og01a1b *og01a1b = to_og01a1b(sd);
	const struct og01a1b_mode *mode;
	s32 vblank_def, h_blank, bpp;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width,
				      height, fmt->format.width,
				      fmt->format.height);

	og01a1b_update_pad_format(mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		goto set_format;

	bpp = (fmt->format.code == MEDIA_BUS_FMT_Y10_1X10 ? 10 : 8);
	__v4l2_ctrl_s_ctrl(og01a1b->link_freq, mode->link_freq_index);
	__v4l2_ctrl_s_ctrl_int64(og01a1b->pixel_rate,
				 to_pixel_rate(mode->link_freq_index, bpp));

	/* Update limits and set FPS to default */
	vblank_def = mode->vts_def - mode->height;
	__v4l2_ctrl_modify_range(og01a1b->vblank,
				 mode->vts_min - mode->height,
				 OG01A1B_VTS_MAX - mode->height, 1, vblank_def);
	__v4l2_ctrl_s_ctrl(og01a1b->vblank, vblank_def);
	h_blank = to_pixels_per_line(mode->hts, mode->link_freq_index,
				     bpp) - mode->width;
	__v4l2_ctrl_modify_range(og01a1b->hblank, h_blank, h_blank, 1, h_blank);

	og01a1b->cur_mode = mode;
	og01a1b->code = fmt->format.code;

set_format:
	*v4l2_subdev_state_get_format(sd_state, fmt->pad) = fmt->format;

	return 0;
}

static int og01a1b_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > ARRAY_SIZE(og01a1b_mbus_formats) - 1)
		return -EINVAL;

	code->code = og01a1b_mbus_formats[code->index];

	return 0;
}

static int og01a1b_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_Y10_1X10 &&
	    fse->code != MEDIA_BUS_FMT_Y8_1X8)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int og01a1b_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct og01a1b *og01a1b = to_og01a1b(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = og01a1b->cur_mode->width,
			.height = og01a1b->cur_mode->height,
			.code = og01a1b->code,
		},
	};

	og01a1b_set_format(sd, state, &fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops og01a1b_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops og01a1b_pad_ops = {
	.set_fmt = og01a1b_set_format,
	.get_fmt = v4l2_subdev_get_fmt,
	.enum_mbus_code = og01a1b_enum_mbus_code,
	.enum_frame_size = og01a1b_enum_frame_size,
	.enable_streams = og01a1b_enable_streams,
	.disable_streams = og01a1b_disable_streams,
};

static const struct v4l2_subdev_ops og01a1b_subdev_ops = {
#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
	.core = &og01a1b_core_ops,
#endif
	.video = &og01a1b_video_ops,
	.pad = &og01a1b_pad_ops,
};

static const struct media_entity_operations og01a1b_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops og01a1b_internal_ops = {
	.init_state = og01a1b_init_state,
};

static int og01a1b_identify_module(struct og01a1b *og01a1b)
{
	int ret;
	u64 val;

	ret = cci_read(og01a1b->regmap, OG01A1B_REG_CHIP_ID, &val, NULL);
	if (ret)
		return ret;

	if (val != OG01A1B_CHIP_ID) {
		dev_err(og01a1b->dev, "chip id mismatch: %x!=%llx",
			OG01A1B_CHIP_ID, val);
		return -ENXIO;
	}

	return 0;
}

static int og01a1b_check_hwcfg(struct og01a1b *og01a1b)
{
	struct device *dev = og01a1b->dev;
	struct fwnode_handle *ep;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret;
	unsigned int i, j;

	if (!fwnode)
		return -ENXIO;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OG01A1B_DATA_LANES) {
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
static int og01a1b_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct og01a1b *og01a1b = to_og01a1b(sd);
	unsigned long delay = DIV_ROUND_UP(8192UL * USEC_PER_SEC,
					   og01a1b->mclk_cfg->mclk);
	int ret;

	if (og01a1b->avdd) {
		ret = regulator_enable(og01a1b->avdd);
		if (ret)
			return ret;
	}

	if (og01a1b->dovdd) {
		ret = regulator_enable(og01a1b->dovdd);
		if (ret)
			goto avdd_disable;
	}

	if (og01a1b->dvdd) {
		ret = regulator_enable(og01a1b->dvdd);
		if (ret)
			goto dovdd_disable;
	}

	ret = clk_prepare_enable(og01a1b->xvclk);
	if (ret)
		goto dvdd_disable;

	gpiod_set_value_cansleep(og01a1b->reset_gpio, 0);

	if (og01a1b->reset_gpio)
		usleep_range(5 * USEC_PER_MSEC, 6 * USEC_PER_MSEC);
	else if (og01a1b->xvclk)
		usleep_range(delay, 2 * delay);

	return 0;

dvdd_disable:
	if (og01a1b->dvdd)
		regulator_disable(og01a1b->dvdd);
dovdd_disable:
	if (og01a1b->dovdd)
		regulator_disable(og01a1b->dovdd);
avdd_disable:
	if (og01a1b->avdd)
		regulator_disable(og01a1b->avdd);

	return ret;
}

static int og01a1b_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct og01a1b *og01a1b = to_og01a1b(sd);
	unsigned long delay = DIV_ROUND_UP(512 * USEC_PER_SEC,
					   og01a1b->mclk_cfg->mclk);

	if (og01a1b->xvclk)
		usleep_range(delay, 2 * delay);

	clk_disable_unprepare(og01a1b->xvclk);

	gpiod_set_value_cansleep(og01a1b->reset_gpio, 1);

	if (og01a1b->dvdd)
		regulator_disable(og01a1b->dvdd);

	if (og01a1b->dovdd)
		regulator_disable(og01a1b->dovdd);

	if (og01a1b->avdd)
		regulator_disable(og01a1b->avdd);

	return 0;
}

static void og01a1b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct og01a1b *og01a1b = to_og01a1b(sd);

	v4l2_async_unregister_subdev(sd);
	og01a1b_deckard_remove(og01a1b);
	v4l2_subdev_cleanup(&og01a1b->sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(og01a1b->dev);
}

static int og01a1b_probe(struct i2c_client *client)
{
	struct og01a1b *og01a1b;
	unsigned long freq;
	unsigned int i;
	int ret;

	og01a1b = devm_kzalloc(&client->dev, sizeof(*og01a1b), GFP_KERNEL);
	if (!og01a1b)
		return -ENOMEM;

	og01a1b->dev = &client->dev;

	v4l2_i2c_subdev_init(&og01a1b->sd, client, &og01a1b_subdev_ops);

	og01a1b->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(og01a1b->regmap))
		return dev_err_probe(og01a1b->dev, PTR_ERR(og01a1b->regmap),
				     "failed to init CCI\n");

	og01a1b->xvclk = devm_v4l2_sensor_clk_get(og01a1b->dev, NULL);
	if (IS_ERR(og01a1b->xvclk))
		return dev_err_probe(og01a1b->dev, PTR_ERR(og01a1b->xvclk),
				     "failed to get xvclk clock\n");

	freq = clk_get_rate(og01a1b->xvclk);
	for (i = 0; i < ARRAY_SIZE(mclk_configs); i++) {
		if (freq == mclk_configs[i].mclk) {
			og01a1b->mclk_cfg = &mclk_configs[i];
			break;
		}
	}
	if (!og01a1b->mclk_cfg)
		return dev_err_probe(og01a1b->dev, -EINVAL,
				     "external clock %lu is not supported",
				     freq);

	ret = og01a1b_check_hwcfg(og01a1b);
	if (ret) {
		dev_err(og01a1b->dev, "failed to check HW configuration: %d",
			ret);
		return ret;
	}

	og01a1b->reset_gpio = devm_gpiod_get_optional(og01a1b->dev, "reset",
						      GPIOD_OUT_LOW);
	if (IS_ERR(og01a1b->reset_gpio)) {
		dev_err(og01a1b->dev, "cannot get reset GPIO\n");
		return PTR_ERR(og01a1b->reset_gpio);
	}

	og01a1b->ircontrol_gpio = devm_gpiod_get_optional(og01a1b->dev,
							  "ircontrol",
							  GPIOD_OUT_LOW);
	if (IS_ERR(og01a1b->ircontrol_gpio))
		return dev_err_probe(og01a1b->dev,
				     PTR_ERR(og01a1b->ircontrol_gpio),
				     "cannot get IR control GPIO\n");

	og01a1b->avdd = devm_regulator_get_optional(og01a1b->dev, "avdd");
	if (IS_ERR(og01a1b->avdd)) {
		ret = PTR_ERR(og01a1b->avdd);
		if (ret != -ENODEV) {
			dev_err_probe(og01a1b->dev, ret,
				      "Failed to get 'avdd' regulator\n");
			return ret;
		}

		og01a1b->avdd = NULL;
	}

	og01a1b->dovdd = devm_regulator_get_optional(og01a1b->dev, "dovdd");
	if (IS_ERR(og01a1b->dovdd)) {
		ret = PTR_ERR(og01a1b->dovdd);
		if (ret != -ENODEV) {
			dev_err_probe(og01a1b->dev, ret,
				      "Failed to get 'dovdd' regulator\n");
			return ret;
		}

		og01a1b->dovdd = NULL;
	}

	og01a1b->dvdd = devm_regulator_get_optional(og01a1b->dev, "dvdd");
	if (IS_ERR(og01a1b->dvdd)) {
		ret = PTR_ERR(og01a1b->dvdd);
		if (ret != -ENODEV) {
			dev_err_probe(og01a1b->dev, ret,
				      "Failed to get 'dvdd' regulator\n");
			return ret;
		}

		og01a1b->dvdd = NULL;
	}

	/* The sensor must be powered on to read the CHIP_ID register */
	ret = og01a1b_power_on(og01a1b->dev);
	if (ret)
		return ret;

	ret = og01a1b_identify_module(og01a1b);
	if (ret) {
		dev_err(og01a1b->dev, "failed to find sensor: %d", ret);
		goto power_off;
	}

	og01a1b->cur_mode = &supported_modes[0];
	og01a1b->code = og01a1b_mbus_formats[0];

	ret = og01a1b_deckard_probe(og01a1b, client);
	if (ret)
		goto power_off;

	ret = og01a1b_init_controls(og01a1b);
	if (ret) {
		dev_err(og01a1b->dev, "failed to init controls: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	og01a1b->sd.state_lock = og01a1b->ctrl_handler.lock;
	og01a1b->sd.internal_ops = &og01a1b_internal_ops;
	og01a1b->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	og01a1b->sd.entity.ops = &og01a1b_subdev_entity_ops;
	og01a1b->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	og01a1b->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&og01a1b->sd.entity, 1, &og01a1b->pad);
	if (ret) {
		dev_err(og01a1b->dev, "failed to init entity pads: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	ret = v4l2_subdev_init_finalize(&og01a1b->sd);
	if (ret < 0) {
		dev_err_probe(og01a1b->dev, ret,
			      "failed to finalize subdevice init\n");
		goto probe_error_media_entity_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&og01a1b->sd);
	if (ret < 0) {
		dev_err(og01a1b->dev, "failed to register V4L2 subdev: %d",
			ret);
		goto probe_error_v4l2_subdev_cleanup;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(og01a1b->dev);
	pm_runtime_enable(og01a1b->dev);
	pm_runtime_idle(og01a1b->dev);

	return 0;

probe_error_v4l2_subdev_cleanup:
	v4l2_subdev_cleanup(&og01a1b->sd);

probe_error_media_entity_cleanup:
	media_entity_cleanup(&og01a1b->sd.entity);

probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(og01a1b->sd.ctrl_handler);
	og01a1b_deckard_remove(og01a1b);

power_off:
	og01a1b_power_off(og01a1b->dev);

	return ret;
}

static const struct dev_pm_ops og01a1b_pm_ops = {
	SET_RUNTIME_PM_OPS(og01a1b_power_off, og01a1b_power_on, NULL)
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id og01a1b_acpi_ids[] = {
	{"OVTI01AC"},
	{}
};

MODULE_DEVICE_TABLE(acpi, og01a1b_acpi_ids);
#endif

static const struct of_device_id og01a1b_of_match[] = {
	{ .compatible = "ovti,og01a1b" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, og01a1b_of_match);

static struct i2c_driver og01a1b_i2c_driver = {
	.driver = {
		.name = "og01a1b",
		.pm = &og01a1b_pm_ops,
		.acpi_match_table = ACPI_PTR(og01a1b_acpi_ids),
		.of_match_table = og01a1b_of_match,
	},
	.probe = og01a1b_probe,
	.remove = og01a1b_remove,
};

#if IS_ENABLED(CONFIG_VIDEO_DECKARD_API)
static int __init og01a1b_init(void)
{
	return i2c_add_driver(&og01a1b_i2c_driver);
}
module_init(og01a1b_init);

static void __exit og01a1b_exit(void)
{
	i2c_del_driver(&og01a1b_i2c_driver);
	og01a1b_deckard_exit();
}
module_exit(og01a1b_exit);
#else
module_i2c_driver(og01a1b_i2c_driver);
#endif

MODULE_AUTHOR("Shawn Tu");
MODULE_DESCRIPTION("OmniVision OG01A1B sensor driver");
MODULE_LICENSE("GPL v2");
