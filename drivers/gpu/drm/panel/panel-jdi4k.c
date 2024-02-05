// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>

#define DPUOVERCLOCKED

// instead of the defined pm8150 analog rails we use the
// TPS65132 to general the analog rails.  it is used
// its default config and therefore just has an enable pin ( enp_gpio )
static const char * const regulator_names[] = {
	"vdda",
	//"lab", pm8150 avdd
	//"ibb", pm8150 avee
};

// struct for the backlight drivers is created so that they can be enabled
// separately and not affect the panel on time sequencing
static const char * const backlight_driver_names[] = {
	"wled1",
	"wled2",
};

typedef enum
{
	RAD_MODE_120 = 0,
	RAD_MODE_90,
	RAD_MODE_80,
} rad_modes_t;

#define PANEL_MFG_INFO_NUM_BYTES 40

struct rad {
	struct device *dev;
	struct drm_panel panel;

	struct regulator_bulk_data power_supplies[ARRAY_SIZE(regulator_names)];
	struct regulator_bulk_data backlight_drivers[ARRAY_SIZE(backlight_driver_names)];

	struct gpio_desc *reset_gpio;
	struct gpio_desc *reset2_gpio;
	//struct gpio_desc *backlight_gpio;
	//struct gpio_desc *backlight2_gpio;
	struct gpio_desc *enp_gpio;

	struct backlight_device *backlight;

	struct mipi_dsi_device *dsi[2];

	bool prepared;
	bool enabled;
	u8 panel_mfg_info[PANEL_MFG_INFO_NUM_BYTES*2];
	u8 panel_read_data[512];
	int panel_read_id, panel_read_reg_addr, panel_read_num_bytes;
	rad_modes_t current_mode;

	struct drm_dsc_config dsc_cfg;
};

static inline struct rad *panel_to_ctx(struct drm_panel *panel)
{
	return container_of(panel, struct rad, panel);
}
	
// Modeset params: terms are defined per panel.
#define H_PIXELS           2160
#define H_FRONT_PORCH      24
#define H_BACK_PORCH       20
#define H_SYNC             20
#define V_LINES            2160
#define V_FRONT_PORCH_120  404
#define V_FRONT_PORCH_90   1265
#define V_FRONT_PORCH_80   1270
#define V_BACK_PORCH       20
#define V_SYNC             1
#define NUM_PANELS         2

// Brightness settings
#define BRIGHTNESS_DEFAULT 256
#define BRIGHTNESS_MAX_120 380
#define BRIGHTNESS_MAX_90  508
#define BRIGHTNESS_MAX_80  508

// DSI DCS command related
#define PANEL_VFP_80  1500
#define PANEL_VFP_90  1500
#define PANEL_VFP_120 700
#define PANEL_VBP_80  800
#define PANEL_VBP_90  800
#define PANEL_VBP_120 800

// Camera captuer signal on panel tester is GPIO2 routed out TE pin
// Need 100usec of offest on either side, so we use 50 lines at 2usec / internal line time
#define CAMERA_TRIGGER_OFFSET 50

// TODO adjust DPU overclocked settings for 90 Hz mode
#ifdef DPUOVERCLOCKED
	#define PANEL_VID_VS_DELAY_80  400
	#define PANEL_VID_VS_DELAY_90  500
	#define PANEL_VID_VS_DELAY_120 400
	#define PIXEL_CLOCK_KHZ_80     613000
	#define PIXEL_CLOCK_KHZ_90     690000
	#define PIXEL_CLOCK_KHZ_120    690000
#else
	#define PANEL_VID_VS_DELAY_90  315
	#define PANEL_VID_VS_DELAY_120 315	
	#define PIXEL_CLOCK_KHZ_90     717574
	#define PIXEL_CLOCK_KHZ_120    717574
#endif

// Total frame time can be calculated as 
// ( htotal*vtotal ) / PIXEL_CLOCK_KHZ
// example: ( (2160 + 24 + 20 + 20) * ( 2160 + 1404 + 1 + 20))/( 717574000 ) = 0.011 sec = 90 Hz
// The simplest way to create a new video mode is change vblank, which means changing V_FRONT_PORCH
// 120 Hz: ( (2160 + 24 + 20 + 20) * ( 2160 + 401 + 1 + 40))/( 690000000 ) = 0.083 sec = 120 Hz
static const struct drm_display_mode modes[3] = {
	{
		.name = "4320x2160_120",
		.clock = PIXEL_CLOCK_KHZ_120 * NUM_PANELS,
		.hdisplay = H_PIXELS * NUM_PANELS,
		.hsync_start = (H_PIXELS + H_FRONT_PORCH) * NUM_PANELS,
		.hsync_end = (H_PIXELS + H_FRONT_PORCH + H_SYNC) * NUM_PANELS,
		.htotal = (H_PIXELS + H_FRONT_PORCH + H_SYNC + H_BACK_PORCH) * NUM_PANELS,
		.vdisplay = V_LINES,
		.vsync_start = V_LINES + V_FRONT_PORCH_120,
		.vsync_end = V_LINES + V_FRONT_PORCH_120 + V_SYNC,
		.vtotal = V_LINES + V_FRONT_PORCH_120 + V_SYNC + V_BACK_PORCH,
	},
	{
		.name = "4320x2160_90",
		.clock = PIXEL_CLOCK_KHZ_90 * NUM_PANELS,
		.hdisplay = H_PIXELS * NUM_PANELS,
		.hsync_start = (H_PIXELS + H_FRONT_PORCH) * NUM_PANELS,
		.hsync_end = (H_PIXELS + H_FRONT_PORCH + H_SYNC) * NUM_PANELS,
		.htotal = (H_PIXELS + H_FRONT_PORCH + H_SYNC + H_BACK_PORCH) * NUM_PANELS,
		.vdisplay = V_LINES,
		.vsync_start = V_LINES + V_FRONT_PORCH_90,
		.vsync_end = V_LINES + V_FRONT_PORCH_90 + V_SYNC,
		.vtotal = V_LINES + V_FRONT_PORCH_90 + V_SYNC + V_BACK_PORCH,
	},
	{
		.name = "4320x2160_80",
		.clock = PIXEL_CLOCK_KHZ_80 * NUM_PANELS,
		.hdisplay = H_PIXELS * NUM_PANELS,
		.hsync_start = (H_PIXELS + H_FRONT_PORCH) * NUM_PANELS,
		.hsync_end = (H_PIXELS + H_FRONT_PORCH + H_SYNC) * NUM_PANELS,
		.htotal = (H_PIXELS + H_FRONT_PORCH + H_SYNC + H_BACK_PORCH) * NUM_PANELS,
		.vdisplay = V_LINES,
		.vsync_start = V_LINES + V_FRONT_PORCH_80,
		.vsync_end = V_LINES + V_FRONT_PORCH_80 + V_SYNC,
		.vtotal = V_LINES + V_FRONT_PORCH_80 + V_SYNC + V_BACK_PORCH,
	},
};

void _get_per_mode_settings(rad_modes_t mode, unsigned * panel_vfp, unsigned * vid_vs_delay, unsigned * vbp)
{
	switch(mode)
	{
		case RAD_MODE_120:
			*panel_vfp = PANEL_VFP_120;
			*vid_vs_delay = PANEL_VID_VS_DELAY_120;
			*vbp = PANEL_VBP_120;
		break;
		case RAD_MODE_90:
			*panel_vfp = PANEL_VFP_90;
			*vid_vs_delay = PANEL_VID_VS_DELAY_90;
			*vbp = PANEL_VBP_90;
		break;
		case RAD_MODE_80:
			*panel_vfp = PANEL_VFP_80;
			*vid_vs_delay = PANEL_VID_VS_DELAY_80;
			*vbp = PANEL_VBP_80;
		break;
		default:
			*panel_vfp = PANEL_VFP_120;
			*vid_vs_delay = PANEL_VID_VS_DELAY_120;
			*vbp = PANEL_VBP_120;
		break;
	}
}

// Important init parameters
// Reg 0xC0, param 1 is the RTN value.  This is the what determines the line load time for the panel
// From the DDIC datasheet:
// RCLK = 32.5 MHz [ds.429 JDI spec]
// RCLK_period = .031 usec
// SASPC RTN ( number of RCLKs in line time ): 0x42 = 66 ( default value we are using )
// internal line time: RTN * RCLK_period = 66 * .031 usec = 2.046 usec
// panel load ( calculated ) : internal line time * number of lines = 2.046 usec * 2160 = 4419 usec

// Reg 0xEC sets the number of lines that are loaded into the FIFO before the DDIC starts writing to the panel.
//          the timing of the fifi offset ( vid_vs_delay in the spec ) is offset lines * external line time,
//          the time it takes for one line to transmit over the MIPI port
//          max FIFO offset for 1/2 DSC = 1072
//          max FIFO offset for 1/3 DSC = 1608
// Reg 0xCF sets the compression modes, do not change ( set by HW config of 1 port, using DSC )
// Reg 0xB9 sets the backlight start line and width ( in internal line times ), note that it gets modified by the set brightness function

// Notes for aligning backlight pulse start with FIFO mode panel load:
//          the backlight pulse start = internal vsync + backlight pulse start ( in internal line times )
//          internal vysnc = external vsync + fifo offset ( in mipi line times )
//          panel load start = internal vsync + VBP setting in register 0xC0 ( in internal line times )


#define _DSI_CMD(payload) ({ \
	int ret, i; \
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) { \
		if (payload[0] >= 0xb0) \
			ret = mipi_dsi_generic_write(ctx->dsi[i], payload, ARRAY_SIZE(payload)); \
		else \
			ret = mipi_dsi_dcs_write_buffer(ctx->dsi[i], payload, ARRAY_SIZE(payload)); \
		if (ret < 0) \
			DRM_DEV_ERROR(ctx->dev, "failed to tx cmd [%.2x], err: %d\n", payload[0], ret); \
	} \
})
#define DSI_CMD(_payload...) ({ u8 payload[] = {_payload}; _DSI_CMD(payload); })
#define SLEEP(x) usleep_range((x) * 1000, (x) * 1000 + 1000)

#define BE16(x) (x)>>8,(x)&0xff

#if 1
static int dsi_populate_dsc_params(struct drm_dsc_config *dsc)
{
	int ret;

	dsc->simple_422 = 0;
	dsc->convert_rgb = 1;
	dsc->vbr_enable = 0;

	drm_dsc_set_const_params(dsc);
	drm_dsc_set_rc_buf_thresh(dsc);

	/* handle only bpp = bpc = 8, pre-SCR panels */
	ret = drm_dsc_setup_rc_params(dsc, DRM_DSC_1_1_PRE_SCR);

	dsc->initial_scale_value = drm_dsc_initial_scale_value(dsc);
	dsc->line_buf_depth = dsc->bits_per_component + 1;

	return drm_dsc_compute_rc_parameters(dsc);
}
#endif

static void rad_init_dsc_config(struct rad *ctx)
{
	ctx->dsc_cfg = (struct drm_dsc_config) {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x2,
		.slice_height = 48,
		.slice_width = 540,
		.slice_count = 4,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,

		.pic_width = 2160,
		.pic_height = 2160,
	};
	dsi_populate_dsc_params(&ctx->dsc_cfg);
}

static void dsi_write_panel_on(struct drm_panel *panel)
{
	struct rad *ctx = panel_to_ctx(panel);

	u8 pps_cmd[1 + sizeof(struct drm_dsc_picture_parameter_set)];
	unsigned RTN = 66, NL = 2160;
	unsigned VFP, VBP, VID_VS_DELAY;
	unsigned GPO1_TES1 = 233, GPO1_TEW1 = 17;
    _get_per_mode_settings(ctx->current_mode, &VFP, &VID_VS_DELAY, &VBP);


	pps_cmd[0] = 0xe6;
	drm_dsc_pps_payload_pack((void*) &pps_cmd[1], &ctx->dsc_cfg);

	// note:
	// - changed VFP from 2013 to 221 (for 120Hz mode)
	// - changed last byte of 0xb6 command to 0x70 (based on datasheet)
	// - disabled broken 0xc6 command (has wrong parameters based on datasheet)

	DSI_CMD(0xb0, 0x04), // mfg register unlock
	DSI_CMD(0xd6, 0x00), // skip loading register defaults out of sleep
	// DSI-2 Control Setting (C option): SELDL=2 (3 lane), PN2PTXR=1 (one port)
	DSI_CMD(0xb6, 0x20, 0x6b, 0x80, 0x06, 0x33, 0x8a, 0x00, 0x1a, 0x7a), //note the 0x7a last byte is needed to avoid corruption
	// Display Mode: VRM=0b010, DM=1 (FIFO mode), TE_ON_VIDEO=1 (TE follows TE_MODE setting)
	DSI_CMD(0xb7, 0x54, 0x00, 0x00, 0x00),
	// Generic pin output setting: GPO1_TES1=233, GPO1_TEW1=17
	// GPO2 is routed out TE, and is set to trigger 50 lines before, and last 50 lines longer, to trigger camera capture on the panel tester
	DSI_CMD(0xb9, BE16(GPO1_TES1), BE16(GPO1_TEW1), 0x00, 0x00, 0x00, 0x00, BE16(GPO1_TES1-CAMERA_TRIGGER_OFFSET), BE16(GPO1_TEW1+CAMERA_TRIGGER_OFFSET*2));
	// Source Setting
	DSI_CMD(0xf1,0x1e); // set offset for next write ( 0xc6 )
	DSI_CMD(0xc6,0x70,0x08,0xd0,0x02,0x21,0x6f,0x08,0x5a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // video mode timings
	// Display Timing
	DSI_CMD(0xc0, RTN, 0x86, VBP&0xff, VBP>>8, BE16(NL), BE16(VFP)),
	// Scaling Mode Setting: RTSON=0
	DSI_CMD(0xcd, 0x00),
	// Compression Mode Setting:
	// COMPV_METHOD=0x8b (DSC 1/3 for DSI)
	// DATA_SWAP=1 (MSB first),
	// COMPSYNA_METHOD=0x8b (DSC 1/3 for partial frame memory)
	DSI_CMD(0xcf, 0x8b, 0x00, 0x80, 0x46, 0x61, 0x00, 0x8b),
	// VID VS delay Setting: to shift when frame internal scanout starts 
	DSI_CMD(0xec, BE16(VID_VS_DELAY), 0x00, 0x00, 0x00),
	
	//TE and TE2 settings
	DSI_CMD(0xbe, 0x00, 0x6A, 0x02 ); //TE=GPIO2 0x6A, TE2=internal vsync 0x02

	// PPS Setting
	_DSI_CMD(pps_cmd);
	DSI_CMD(MIPI_DCS_GET_COMPRESSION_MODE, 0x01), // compression enabled
	DSI_CMD(MIPI_DCS_SET_TEAR_SCANLINE, 0x00, 0x00), // turn on TE at line 0
	DSI_CMD(MIPI_DCS_SET_TEAR_ON, 0x00), // turn on TE
	DSI_CMD(MIPI_DCS_SET_ADDRESS_MODE, 0x00), // Set scan out order ( default )
	DSI_CMD(MIPI_DCS_SET_PIXEL_FORMAT, 0x77), // 24 bits / pixel ( default )
	DSI_CMD(MIPI_DCS_EXIT_SLEEP_MODE),
	SLEEP(170),
	DSI_CMD(MIPI_DCS_SET_DISPLAY_ON),
	DSI_CMD(0xd6, 0x80), // load defaults after sleep out
	SLEEP(200),
	DSI_CMD(0xb0, 0x03),
	SLEEP(200);
}

static void dsi_write_panel_off(struct drm_panel *panel)
{
	struct rad *ctx = panel_to_ctx(panel);

	DSI_CMD(MIPI_DCS_SET_DISPLAY_OFF),
	DSI_CMD(MIPI_DCS_SET_TEAR_OFF),
	DSI_CMD(MIPI_DCS_ENTER_SLEEP_MODE),
	SLEEP(200);
}

/* -----------------------------------------------------------------------------
 * sysfs
 */

// uncomment this line to print out panel mfg info to dmesg
#define PRINT_PANEL_MFG_INFO
static void get_panel_info(struct drm_panel *panel)
{
	struct rad *ctx = panel_to_ctx(panel);
	u8 panel_info_regs[] = { 0xBF };
	u8 *panel_mfg_info = ctx->panel_mfg_info;
	int ret;
	unsigned int id, offset;

	// read panel info from register 0xBF
	// only DVT+ panels will have data other than synaptics driver IDs populated
	for (id = 0; id < ARRAY_SIZE(ctx->dsi); id++) 
	{ 
		offset =  id*PANEL_MFG_INFO_NUM_BYTES;
		ret = mipi_dsi_generic_read(ctx->dsi[id], panel_info_regs, 1, 
									&panel_mfg_info[offset], PANEL_MFG_INFO_NUM_BYTES);
		if ( ret < 0 )
		{
						DRM_DEV_ERROR(ctx->dev, "failed to read panel %d mfg info reg\n", id);
		}

        #ifdef PRINT_PANEL_MFG_INFO
		printk("printing jdi4k Panel info for %s panel : \n", id ? "right" : "left" );
		// parse the results
		printk(" synapticsSupplierID  %x\n",  panel_mfg_info[1+offset] << 8 | panel_mfg_info[0+offset] );
		printk(" synapticsDriverICProductCode  %x\n",  panel_mfg_info[3+offset] << 8 | panel_mfg_info[2+offset] );
		printk(" brightnessDataW255  %x\n",  panel_mfg_info[5+offset] );
		printk(" colorPointofWhiteX  %x\n",  panel_mfg_info[6+offset] );
		printk(" colorPointofWhiteY  %x\n",  panel_mfg_info[7+offset] );
		printk(" contrast  %x\n",  panel_mfg_info[8+offset] );
		printk(" gamma  %x\n",  panel_mfg_info[9+offset] );
		printk(" brightnessDataW32  %x\n",  panel_mfg_info[12+offset] );
		printk(" vcom1  %x\n",  panel_mfg_info[13+offset] );
		printk(" vcom2  %x\n",  panel_mfg_info[14+offset] );
		printk(" colorPointofRedX  %x\n",  panel_mfg_info[21+offset] );
		printk(" colorPointofRedY  %x\n",  panel_mfg_info[22+offset] );
		printk(" colorPointofGreenX  %x\n",  panel_mfg_info[23+offset] );
		printk(" colorPointofGreenY  %x\n",  panel_mfg_info[24+offset] );
		printk(" colorPointofBlueX  %x\n",  panel_mfg_info[25+offset] );
		printk(" colorPointofBlueY  %x\n",  panel_mfg_info[26+offset] );
		printk(" format  %x\n",  panel_mfg_info[27+offset] );
		printk(" vendor1  %x\n",  panel_mfg_info[28+offset] );
		printk(" vendor2  %x\n",  panel_mfg_info[29+offset] );
		printk(" mdlBaseCode1  %x\n",  panel_mfg_info[30+offset] );
		printk(" mdlBaseCode2  %x\n",  panel_mfg_info[31+offset] );
		printk(" mdlBaseCode3  %x\n",  panel_mfg_info[32+offset] );
		printk(" id  %x\n",  panel_mfg_info[33+offset] );
		printk(" serialization3  %x\n",  panel_mfg_info[34+offset] );
		printk(" serialization2  %x\n",  panel_mfg_info[35+offset] );
		printk(" serialization1  %x\n",  panel_mfg_info[36+offset] );
		printk(" year  %x\n",  panel_mfg_info[37+offset] );
		printk(" week  %x\n",  panel_mfg_info[38+offset] );
		printk(" day  %x\n",  panel_mfg_info[39+offset] );
		#endif
	}
}

#if 1
static ssize_t panel_mfg_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
       unsigned int i;
       int ret;
       ssize_t len = 0;
	   struct rad *ctx = dev_get_drvdata(dev);
       for (i = 0; i < ARRAY_SIZE(ctx->panel_mfg_info); i++) {
               ret = scnprintf(buf + len, PAGE_SIZE - len, "%u ",
                              ctx->panel_mfg_info[i]);
               if (ret < 0)
                       return ret;
               len += ret;
       }
       buf[len - 1] = '\n';
       return len;
}

static ssize_t panel_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	ssize_t len = 0;
	unsigned int i;
	struct rad *ctx = dev_get_drvdata(dev);
	ret = mipi_dsi_generic_read(ctx->dsi[ctx->panel_read_id], &ctx->panel_read_reg_addr, 1, ctx->panel_read_data, ctx->panel_read_num_bytes);
	if ( ret < 0 )
	{
		DRM_DEV_ERROR(ctx->dev, "failed to read panel %d reg %u\n", ctx->panel_read_id, ctx->panel_read_reg_addr);
		return ret;
	}
	else
	{
		for (i = 0; i < ctx->panel_read_num_bytes; i++) {
				ret = scnprintf(buf + len, PAGE_SIZE - len, "%x ",
								ctx->panel_read_data[i]);
				if (ret < 0)
						return ret;
				len += ret;
		}
		buf[len - 1] = '\n';
		return len;
	}
}

static ssize_t panel_read_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int ret;
	struct rad *ctx = dev_get_drvdata(dev);
	ret = sscanf(buf, "%d %2x %d", &ctx->panel_read_id, &ctx->panel_read_reg_addr, &ctx->panel_read_num_bytes);
	if(ret != 3)
	{
		printk("jdi4k panel read got wrong number of input parameters from sscanf");
		return -EINVAL;
	}
	if( ctx->panel_read_id != 0 && ctx->panel_read_id != 1)
	{
		// invalid panel id
		printk("panel read found invalid id value %u", ctx->panel_read_id);
		ctx->panel_read_id = 0;
		return -EINVAL;
	}
	return count;
}

static ssize_t panel_write_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int ret;
	int id, addr, len, i, v;
	char char_data[256];
	u8 payload[256];
	struct rad *ctx = dev_get_drvdata(dev);
	// get the base parameters
	ret = sscanf(buf, "%d %2x %d %s", &id, &addr, &len, char_data);
	if(ret != 4)
	{
		printk("jdi4k panel write got wrong number of input parameters (%d) from sscanf", ret);
		return -EINVAL;
	}
	if(id != 0 && id != 1)
	{
		// invalid panel id
		printk("panel read found invalid id value %u", ctx->panel_read_id);
		ctx->panel_read_id = 0;
		return -EINVAL;
	}
	// we limit writes to a total of 256 bytes
	if(len >256)
	{
		printk("jdi4k panel write exceed max length %d\n", len);
		return -EINVAL;
	}
	payload[0] = addr;
	// get the data payload
	for(i=0;i<len;i++)
	{
	    if(sscanf(char_data + i * 2, "%2x", &v) != 1) break;
        payload[i+1] = (unsigned char)v;
	}

	if (addr >= 0xb0)
		ret = mipi_dsi_generic_write(ctx->dsi[id], payload, ARRAY_SIZE(payload));
	else
		ret = mipi_dsi_dcs_write_buffer(ctx->dsi[id], payload, ARRAY_SIZE(payload));
	if (ret < 0)
	{
		DRM_DEV_ERROR(ctx->dev, "failed to tx cmd [%.2x], err: %d\n", payload[0], ret);
	}

	printk("jdi4k wrote %s to panel %d reg %x\n", char_data, id, addr);

	return count;
}

static ssize_t panel_available_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rad *ctx = dev_get_drvdata(dev);
	buf[0] = ctx->prepared ? '1' : '0';
	buf[1] = '\n';
	return 2;
}

static DEVICE_ATTR_RO(panel_mfg_info);
static DEVICE_ATTR_RW(panel_read);
static DEVICE_ATTR_WO(panel_write);
static DEVICE_ATTR_RO(panel_available);

static struct attribute *jdi4k_attrs[] = {
    &dev_attr_panel_mfg_info.attr,
	&dev_attr_panel_read.attr,
	&dev_attr_panel_write.attr,
	&dev_attr_panel_available.attr,
	NULL,
};

static const struct attribute_group jdi4k_attr_group = {
	.name = "jdi4k",
    .attrs = jdi4k_attrs,
};

#endif

/* ----------------------------------------------------------------------------- */

static int set_brightness(struct rad *ctx, u16 brightness, u16 pulse_offset_rows)
{
	unsigned GPO1_TES1 = pulse_offset_rows; // same as on init
	unsigned GPO1_TEW1 = brightness;
	signed CAM_TRIGGER_START, CAM_TRIGGER_WIDTH = 0;

	// limit max brigthness based on mode
	if (ctx->current_mode == RAD_MODE_120 && GPO1_TEW1 > BRIGHTNESS_MAX_120)
	{
		GPO1_TEW1 = BRIGHTNESS_MAX_120;
	}
	else if (ctx->current_mode == RAD_MODE_90 && GPO1_TEW1 > BRIGHTNESS_MAX_90)
	{
		GPO1_TEW1 = BRIGHTNESS_MAX_90;
	}
	else if (ctx->current_mode == RAD_MODE_80 && GPO1_TEW1 > BRIGHTNESS_MAX_80 )
	{
		GPO1_TEW1 = BRIGHTNESS_MAX_80;
	}

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	if ( brightness != 0 )
	{
		CAM_TRIGGER_START = (GPO1_TES1 > CAMERA_TRIGGER_OFFSET) ? (GPO1_TES1 - CAMERA_TRIGGER_OFFSET) : 0;
		CAM_TRIGGER_WIDTH = GPO1_TEW1 + CAMERA_TRIGGER_OFFSET*2;
	}

	DSI_CMD(0xb9, BE16(GPO1_TES1), BE16(GPO1_TEW1), 0x00, 0x00, 0x00, 0x00, BE16(CAM_TRIGGER_START), BE16(CAM_TRIGGER_WIDTH));

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int truly_35597_power_on(struct rad *ctx)
{
	int ret;

    // turn on vdd
	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->power_supplies), ctx->power_supplies);
	if (ret < 0)
		return ret;

    // wait > 1 msec
	usleep_range(1000, 2000);

    // turn on AVDD
	gpiod_set_value(ctx->enp_gpio, 1);
	// AVEE is enabled when 1V8 is enabled?

	//gpiod_set_value(ctx->reset_gpio, 0);
	//gpiod_set_value(ctx->reset2_gpio, 0);
	// wait > 3 msec
	usleep_range(20000, 30000);
	gpiod_set_value(ctx->reset_gpio, 1);
	gpiod_set_value(ctx->reset2_gpio, 1);
	//usleep_range(10000, 20000);

	return 0;
}

static int rad_power_off(struct rad *ctx)
{
	int ret = 0;

    // Put panels into reset
	gpiod_set_value(ctx->reset_gpio, 0);
	gpiod_set_value(ctx->reset2_gpio, 0);

    // wait > 25 msec
    usleep_range(25000, 26000);

    // turn off AVDD
	gpiod_set_value(ctx->enp_gpio, 0);

    // wait > 1 msec
	usleep_range(10000, 20000);

    // turn off vdd
	ret = regulator_bulk_disable(ARRAY_SIZE(ctx->power_supplies), ctx->power_supplies);
	if (ret) {
		DRM_DEV_ERROR(ctx->dev,
			"regulator_bulk_disable failed %d\n", ret);
	}
	return ret;
}

static int rad_disable(struct drm_panel *panel)
{
	struct rad *ctx = panel_to_ctx(panel);

	if (!ctx->enabled)
		return 0;

	backlight_disable(ctx->backlight);

	ctx->enabled = false;
	return 0;
}

static int rad_unprepare(struct drm_panel *panel)
{
	struct rad *ctx = panel_to_ctx(panel);
	int ret = 0;

	if (!ctx->prepared)
		return 0;

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

    // disable backlight drivers
	ret = regulator_bulk_disable(ARRAY_SIZE(ctx->backlight_drivers), ctx->backlight_drivers);
	if (ret) {
		DRM_DEV_ERROR(ctx->dev,
			"regulator_bulk_disable failed %d\n", ret);
	}

	dsi_write_panel_off(panel);

	ret = rad_power_off(ctx);
	if (ret < 0)
		DRM_DEV_ERROR(ctx->dev, "power_off failed ret = %d\n", ret);

	ctx->prepared = false;
	return ret;
}

static int rad_prepare(struct drm_panel *panel)
{
	struct rad *ctx = panel_to_ctx(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = truly_35597_power_on(ctx);
	if (ret < 0)
	{
		DRM_DEV_ERROR(ctx->dev, "power_on failed\n");
		goto power_off;
	}

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

    // wait > 20 msec after power on before initial setting 
	usleep_range(20000, 21000);
	dsi_write_panel_on(panel);

    get_panel_info(panel);

	// turn on backlight drivers
	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->backlight_drivers), ctx->backlight_drivers);
	if (ret < 0)
	{
		DRM_DEV_ERROR(ctx->dev, "backlight driver on failed\n");
		goto power_off;
	}

	ctx->backlight->props.pulse_offset_rows = 233;
	set_brightness(ctx, ctx->backlight->props.brightness, ctx->backlight->props.pulse_offset_rows);

	ctx->prepared = true;

	return 0;

power_off:
	if (rad_power_off(ctx))
		DRM_DEV_ERROR(ctx->dev, "power_off failed\n");
	return ret;
}

static int rad_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int rad_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct rad *rad = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if (!rad->prepared)
		return 0;

	ret = set_brightness(rad, bl->props.brightness, bl->props.pulse_offset_rows);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops rad_bl_ops = {
	.update_status = rad_bl_update_status,
	.get_brightness = rad_bl_get_brightness,
};

static struct drm_panel *_panel;

int panel_mode_set(const struct drm_display_mode *mode)
{
	struct rad *ctx = panel_to_ctx(_panel);
	unsigned RTN = 66, NL = 2160;
	unsigned VFP, VBP, VID_VS_DELAY;
	rad_modes_t new_mode;
	// use the vtotal to determine the mode
	if (mode->vtotal == modes[RAD_MODE_120].vtotal){
		new_mode = RAD_MODE_120;
	}else if (mode->vtotal == modes[RAD_MODE_90].vtotal){
		new_mode = RAD_MODE_90;
	}else if (mode->vtotal == modes[RAD_MODE_80].vtotal){
		new_mode = RAD_MODE_80;
	}else{
		DRM_DEV_ERROR(ctx->dev, "Invalid vtotal in mode set %d", mode->vtotal);
		return -EINVAL;
	}

    // check if we need to update settings
	if (ctx->current_mode!=new_mode)
	{
		ctx->current_mode = new_mode;
        // only reset the panel parematers if we already initialized the panels
		if (ctx->prepared)
		{
			_get_per_mode_settings(ctx->current_mode, &VFP, &VID_VS_DELAY, &VBP);
			// Display Timing
			DSI_CMD(0xc0, RTN, 0x86, VBP&0xff, VBP>>8, BE16(NL), BE16(VFP));
			// VID VS delay Setting: to shift when frame internal scanout starts
			DSI_CMD(0xec, BE16(VID_VS_DELAY), 0x00, 0x00, 0x00);
		}
	}

	return 0;
}

static int rad_enable(struct drm_panel *panel)
{
	struct rad *ctx = panel_to_ctx(panel);
	int ret;

	if (ctx->enabled)
		return 0;

	ret = backlight_enable(ctx->backlight);

	ctx->enabled = true;

	return 0;
}


static int rad_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct rad *ctx = panel_to_ctx(panel);
	struct drm_display_mode *mode;
	int i;

	connector->display_info.width_mm = 100;
	connector->display_info.height_mm = 50;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		mode = drm_mode_create(connector->dev);
		if (!mode) {
			DRM_DEV_ERROR(ctx->dev,
				"failed to create a new display mode\n");
			return i;
		}

		drm_mode_copy(mode, &modes[i]);
		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
	}

	return i;
}

static const struct drm_panel_funcs rad_drm_funcs = {
	.disable = rad_disable,
	.unprepare = rad_unprepare,
	.prepare = rad_prepare,
	.enable = rad_enable,
	.get_modes = rad_get_modes,
};

static int rad_panel_add(struct rad *ctx)
{
	struct device *dev = ctx->dev;
	struct backlight_properties bl_props;
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(ctx->power_supplies); i++)
		ctx->power_supplies[i].supply = regulator_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->power_supplies),
				      ctx->power_supplies);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ctx->backlight_drivers); i++)
		ctx->backlight_drivers[i].supply = backlight_driver_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->backlight_drivers),
				      ctx->backlight_drivers);
	if (ret < 0)
		return ret;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		DRM_DEV_ERROR(dev, "cannot get reset gpio %ld\n",
			PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	};

	ctx->reset2_gpio = devm_gpiod_get(dev, "reset2", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset2_gpio)) {
		DRM_DEV_ERROR(dev, "cannot get reset gpio %ld\n",
			PTR_ERR(ctx->reset2_gpio));
		return PTR_ERR(ctx->reset2_gpio);
	};

#if 0
	ctx->backlight_gpio = devm_gpiod_get_optional(dev, "backlight", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->backlight_gpio)) {
		DRM_DEV_ERROR(dev, "cannot get reset gpio %ld\n",
			PTR_ERR(ctx->backlight_gpio));
		return PTR_ERR(ctx->backlight_gpio);
	};

	ctx->backlight2_gpio = devm_gpiod_get_optional(dev, "backlight2", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->backlight2_gpio)) {
		DRM_DEV_ERROR(dev, "cannot get reset gpio %ld\n",
			PTR_ERR(ctx->backlight2_gpio));
		return PTR_ERR(ctx->backlight2_gpio);
	};
#endif

	ctx->enp_gpio = devm_gpiod_get(dev, "enp", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enp_gpio)) {
		DRM_DEV_ERROR(dev, "cannot get enp gpio %ld\n",
			PTR_ERR(ctx->enp_gpio));
		return PTR_ERR(ctx->enp_gpio);
	}

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = BRIGHTNESS_DEFAULT;
	// we set this based on 120 as the default mode as several scripts use it
	bl_props.max_brightness = BRIGHTNESS_MAX_120;

	ctx->backlight = devm_backlight_device_register(dev, dev_name(dev),
							  dev, ctx->dsi[0], &rad_bl_ops,
							  &bl_props);
	if (IS_ERR(ctx->backlight)) {
		ret = PTR_ERR(ctx->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_init(&ctx->panel, dev, &rad_drm_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	drm_panel_add(&ctx->panel);

	return 0;
}

static int rad_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct rad *ctx;
	struct mipi_dsi_device *dsi1_device;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct mipi_dsi_device *dsi_dev;
	int ret = 0;
	int i;

	const struct mipi_dsi_device_info info = {
		.type = "jdi4k",
		.channel = 0,
		.node = NULL,
	};

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);

	if (!ctx)
		return -ENOMEM;

	ctx->panel.prepare_prev_first = true;

	dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
	if (!dsi1) {
		DRM_DEV_ERROR(dev,
			"failed to get remote node for dsi1_device\n");
		return -ENODEV;
	}

	dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
	of_node_put(dsi1);
	if (!dsi1_host) {
		DRM_DEV_ERROR(dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	/* register the second DSI device */
	dsi1_device = mipi_dsi_device_register_full(dsi1_host, &info);
	if (IS_ERR(dsi1_device)) {
		DRM_DEV_ERROR(dev, "failed to create dsi device\n");
		return PTR_ERR(dsi1_device);
	}

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	ctx->dsi[0] = dsi;
	ctx->dsi[1] = dsi1_device;
	ctx->current_mode = RAD_MODE_120; // default to 120

	rad_init_dsc_config(ctx);
	ctx->dsi[0]->dsc = &ctx->dsc_cfg;
	ctx->dsi[1]->dsc = &ctx->dsc_cfg;
	ctx->dsi[0]->dsc_slice_per_pkt = 4;
	ctx->dsi[1]->dsc_slice_per_pkt = 4;

	ret = rad_panel_add(ctx);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to add panel\n");
		goto err_panel_add;
	}

	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		dsi_dev = ctx->dsi[i];
		dsi_dev->lanes = 3;
		dsi_dev->format = MIPI_DSI_FMT_RGB888;
		dsi_dev->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM |
			MIPI_DSI_CLOCK_NON_CONTINUOUS;
		ret = mipi_dsi_attach(dsi_dev);
		if (ret < 0) {
			DRM_DEV_ERROR(dev,
				"dsi attach failed i = %d\n", i);
			goto err_dsi_attach;
		}
	}

	_panel = &ctx->panel; /* for panel_mode_set hack */

    ret = sysfs_create_group(&dsi->dev.kobj, &jdi4k_attr_group);
    if (ret < 0) {
        dev_err(&dsi->dev, "failed to create jdi4k sysfs files\n");
        return ret;
    }

	return 0;

err_dsi_attach:
	drm_panel_remove(&ctx->panel);
err_panel_add:
	mipi_dsi_device_unregister(dsi1_device);
	return ret;
}

static void rad_remove(struct mipi_dsi_device *dsi)
{
	struct rad *ctx = mipi_dsi_get_drvdata(dsi);

	if (ctx->dsi[0])
		mipi_dsi_detach(ctx->dsi[0]);
	if (ctx->dsi[1]) {
		mipi_dsi_detach(ctx->dsi[1]);
		mipi_dsi_device_unregister(ctx->dsi[1]);
	}

	drm_panel_remove(&ctx->panel);

#if 1
	sysfs_remove_group(&dsi->dev.kobj, &jdi4k_attr_group);
#endif
}

static const struct of_device_id rad_of_match[] = {
	{ .compatible = "jdi4k" },
	{ }
};
MODULE_DEVICE_TABLE(of, rad_of_match);

static struct mipi_dsi_driver rad_driver = {
	.driver = {
		.name = "panel-jdi4k",
		.of_match_table = rad_of_match,
	},
	.probe = rad_probe,
	.remove = rad_remove,
};
module_mipi_dsi_driver(rad_driver);

MODULE_DESCRIPTION("Raydium R63455 DSI Panel Driver");
MODULE_LICENSE("GPL v2");
