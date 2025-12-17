// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023-2024, The Valve Ltd. All rights reserved.
 * Copyright (c) 2024, The Linaro Ltd. All rights reserved.
 */

#include <linux/backlight.h>
#include <linux/debugfs.h>
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

//#define NO_DSC

/* panel specific MIPI DSI CMD */
#define LPM02648C_MF_CMD_ACCESS_PROTECT		0xb0
#define LPM02648C_SEQ_CTL			0xd6
#define LPM02648C_DSI_CTL			0xb6
#define LPM02648C_DISP_MODE			0xb7
#define LPM02648C_GEN_OUTPIN_SET		0xb9
#define LPM02648C_DISP_SET1			0xc0
#define LPM02648C_DISP_SET2			0xf1
#define LPM02648C_DISP_SET3			0xc6
#define LPM02648C_DISP_SET3_2			0xcd
#define LPM02648C_DISP_SET4			0xcf
#define LPM02648C_DISP_SET5			0xec
#define LPM02648C_DISP_SET6			0xef
#define LPM02648C_TE_GPIO_CTL			0xbe
#define LPM02648C_PPS_SET			0xe6
#define LPM02648C_MFG_INFO			0xbf
#define 	VENDOR_OFFSET			27
#define 	VCOM120_OFFSET			13
#define 	VCOM90_OFFSET			15

// Brightness settings
#define BRIGHTNESS_DEFAULT 256
#define BRIGHTNESS_MAX_120 367

// Panel settings
#define VFP		700
#define VID_VS_DELAY	1112
#define VBP		23
#define GPO1_TES1_144_SHARP	150
#define GPO1_TES1_120_SHARP	100
#define GPO1_TES1_144_BOE	3435
#define GPO1_TES1_120_BOE	4068
#define GPO1_TES1_144_JDI	GPO1_TES1_144_BOE
#define GPO1_TES1_120_JDI	GPO1_TES1_120_BOE
#define GPO1_TES1_90		5446
#define GPO1_TEW1			0
#define RTN 59

// Panel Vendor types
#define VENDOR_SHARP	'S'
#define VENDOR_JDI		'J'
#define VENDOR_BOE		'B'

/*
 * Camera capture signal on panel tester is GPIO2 routed out TE pin
 * Need 100usec of offest on either side, so we use 50 lines at
 * 2usec / internal line time
 */
#define CAMERA_TRIGGER_OFFSET 50

static const char * const regulator_names[] = {
	"vdda",
};

static const char * const preboost_driver_names[] = {
	"preboost",
};

static const char * const backlight_driver_names[] = {
	"backlight-left",
	"backlight-right",
};

static const char * const lcd_bias_names[] = {
	"left-pos",
	"left-neg",
	"right-pos",
	"right-neg"
};

typedef enum
{
	LEFT_POS = 0,
	LEFT_NEG,
	RIGHT_POS,
	RIGHT_NEG,
} lpm026m648c_bias_t;

#define PANEL_MFG_INFO_NUM_BYTES 40

typedef enum
{
	MODE_120 = 0,
	MODE_90,
	MODE_80,
	MODE_144,
	MODE_72,
} lpm026m648c_modes_t;

struct lpm026m648c_ctx {
	struct device *dev;
	struct drm_panel panel;

	struct regulator_bulk_data power_supplies[ARRAY_SIZE(regulator_names)];
	struct regulator_bulk_data preboost_drivers[ARRAY_SIZE(preboost_driver_names)];
	struct regulator_bulk_data backlight_drivers[ARRAY_SIZE(backlight_driver_names)];
	struct regulator_bulk_data lcd_bias_regulators[ARRAY_SIZE(lcd_bias_names)];

	struct gpio_descs *reset_gpios;
	struct gpio_descs *te_gpios;
	struct gpio_desc *enp_gpio;
	struct gpio_desc *enn_gpio;

	int te_left_gpio_irq;
	int te_right_gpio_irq;

	struct backlight_device *backlight;
	struct mipi_dsi_device *dsi[2];
	u8 panel_mfg_info[PANEL_MFG_INFO_NUM_BYTES*2];
	u8 panel_read_data[512];
	int panel_read_id, panel_read_reg_addr, panel_read_num_bytes;

	struct drm_dsc_config dsc_cfg;
	bool is_dual_panel;
	bool dphy;
	bool low_fps_support;
	bool prepared;
	bool enabled;

	char panel_vendor;

	uint16_t vcom120[2];
	uint16_t vcom90[2];

	lpm026m648c_modes_t current_mode;
};

static inline struct lpm026m648c_ctx *panel_to_ctx(struct drm_panel *panel)
{
	return container_of(panel, struct lpm026m648c_ctx, panel);
}

static const struct drm_display_mode modes[1] = {
#ifdef NO_DSC
	{
		.name = "1776x1920_60",
		.clock =  (1776 + 20 + 8 + 28) * (1920 + 7 + 1 + 17) * 60 / 1000,
		.hdisplay = 1776,
		.hsync_start = (1776 + 20),
		.hsync_end = (1776 + 20 + 8),
		.htotal = (1776 + 20 + 8 + 28),
		.vdisplay = 1920,
		.vsync_start = 1920 + 7,
		.vsync_end = 1920 + 7 + 1,
		.vtotal = 1920 + 7 + 1 + 17,
		.width_mm = 50,
		.height_mm = 50,
	}
#else
	{
		.name = "1776x1920_90",
		.clock =  (1776 + 20 + 8 + 28) * (1920 + 7 + 1 + 17) * 90 / 1000,
		.hdisplay = 1776,
		.hsync_start = (1776 + 20),
		.hsync_end = (1776 + 20 + 8),
		.htotal = (1776 + 20 + 8 + 28),
		.vdisplay = 1920,
		.vsync_start = 1920 + 7,
		.vsync_end = 1920 + 7 + 1,
		.vtotal = 1920 + 7 + 1 + 17,
		.width_mm = 50,
		.height_mm = 50,
	}
#endif
};

#define lpm026m648c_dsi_write_seq(ctx, cmd, seq...)					\
	do {										\
		u8 d[] = {cmd, seq};							\
		struct mipi_dsi_multi_context dsi_ctx0 = { .dsi = ctx->dsi[0] };	\
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx0, d, ARRAY_SIZE(d));		\
		if (ctx->is_dual_panel) {						\
			struct mipi_dsi_multi_context dsi_ctx1 = { .dsi = ctx->dsi[1] };\
			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx1, d, ARRAY_SIZE(d));	\
		}									\
	} while (0)

#define lpm026m648c_dsi_write_buffer(ctx, d)						\
	do {										\
		struct mipi_dsi_multi_context dsi_ctx0 = { .dsi = ctx->dsi[0] };	\
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx0, d, ARRAY_SIZE(d));		\
		if (ctx->is_dual_panel) {						\
			struct mipi_dsi_multi_context dsi_ctx1 = { .dsi = ctx->dsi[1] };\
			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx1, d, ARRAY_SIZE(d));	\
		}									\
	} while (0)

#define BE16(x) (x)>>8, (x)&0xff
#define LE16(x) (x)&0xff, (x)>>8

static int dsi_populate_dsc_params(struct lpm026m648c_ctx *ctx)
{
	int ret;
	struct drm_dsc_config *dsc = &ctx->dsc_cfg;

	dsc->simple_422 = 0;
	dsc->convert_rgb = 1;
	dsc->vbr_enable = 0;

	drm_dsc_set_const_params(dsc);
	drm_dsc_set_rc_buf_thresh(dsc);

	/* handle only bpp = bpc = 8, pre-SCR panels */
	ret = drm_dsc_setup_rc_params(dsc, DRM_DSC_1_1_PRE_SCR);
	if (ret < 0)
		DRM_DEV_ERROR(ctx->dev, "failed to setup dsc params\n");

	dsc->initial_scale_value = drm_dsc_initial_scale_value(dsc);
    DRM_DEV_ERROR(ctx->dev, "dsc scale value %d\n", dsc->initial_scale_value);
	dsc->line_buf_depth = dsc->bits_per_component + 1;

	return drm_dsc_compute_rc_parameters(dsc);
}

static int lpm026m648c_init_dsc_config(struct lpm026m648c_ctx *ctx)
{
	ctx->dsc_cfg = (struct drm_dsc_config) {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = 24,
		.slice_width = 444,
		.slice_count = 4,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,

		.pic_width = 1776,
		.pic_height = 1920,
	};

	return dsi_populate_dsc_params(ctx);
}

// should be called after get_panel_info
static int update_vcom(struct lpm026m648c_ctx *ctx)
{
	u16 vcom_left, vcom_right;
	u8 payload[3] = {0xd5, 0, 0};
	int ret = 0;
	// TODO interpolate for other modes?
	if ( ctx->current_mode == MODE_120 ) {
		vcom_left = ctx->vcom120[0];
		vcom_right = ctx->vcom120[1];
	} else {
		// 90/80/72 use same settings
		vcom_left = ctx->vcom90[0];
		vcom_right = ctx->vcom90[1];
	}

	if ( vcom_left ) {
		payload[1] = ( vcom_left & 0xFF00 ) >> 8;
		payload[2] = vcom_left & 0x00FF;
		ret |= mipi_dsi_generic_write(ctx->dsi[0], payload, 3);
	}
	if ( vcom_right ) {
		payload[1] = ( vcom_right & 0xFF00 ) >> 8;
		payload[2] = vcom_right & 0x00FF;
		ret |= mipi_dsi_generic_write(ctx->dsi[1], payload, 3);
	}
	if ( ret < 0 ) {
		DRM_DEV_ERROR(ctx->dev, "Failed to set vcom ( %d )\n", ret);
	}
	return ret;
}



static void dsi_write_oled_panel_on(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

    dev_err(ctx->dev, "Running OLED init sequence");

	u8 pps_cmd[1 + sizeof(struct drm_dsc_picture_parameter_set)];

	drm_dsc_pps_payload_pack((void *)&pps_cmd[1], &ctx->dsc_cfg);
	pps_cmd[0] = 0xF6;

#ifdef NO_DSC
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_GET_COMPRESSION_MODE, 0x00); // no compression
#else
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_GET_COMPRESSION_MODE, 0x11); // enable dsc compression
#endif
	lpm026m648c_dsi_write_buffer(ctx, pps_cmd); // dsc
    //lpm026m648c_dsi_write_seq(ctx, 0xF6, 0x11, 0x00, 0x00, 0xAB, 0x10, 0xA0, 0x0A, 0x00, 0x05, 0xA0, 0x00, 0x20, 0x02, 0xD0, 0x03, 0x84, 0x01, 0x9A, 0x02, 0x68, 0x00, 0x19, 0x02, 0xD9, 0x00, 0x0E, 0x00, 0x0F, 0x03, 0xDf, 0x02, 0xEB, 0x16, 0x00, 0x10, 0xEC, 0x07, 0x10, 0x20, 0x00, 0x06, 0x0F, 0x0F, 0x33, 0x0E, 0x1C, 0x2A, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7B, 0x7D, 0x7E, 0x01, 0xC2, 0x22, 0x00, 0x2A, 0x40, 0x2A, 0xBE, 0x3A, 0xFC, 0x3A, 0xFA, 0x3A, 0xF8, 0x3B, 0x38, 0x3B, 0x78, 0x3B, 0x76, 0x4B, 0xB6, 0x4B, 0xB6, 0x4B, 0xF4, 0x63, 0xF4, 0x7C, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 );
	// try forcing the panel dsc params ( but this won't match local settings )
    //Panel_VBP * 1/2 - 4
    // 8*1/2-4
	lpm026m648c_dsi_write_seq(ctx, 0x3B, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x06, 0x17, 0x05, 0x9F, 0x04, 0xC7, 0x03, 0xAF); // video mode
	lpm026m648c_dsi_write_seq(ctx, 0x51, 0xFF); // full brightness
	lpm026m648c_dsi_write_seq(ctx, 0x53, 0x20); // manual brightness, dimming off
	lpm026m648c_dsi_write_seq(ctx, 0x26, 0x01); // default gamma set
	lpm026m648c_dsi_write_seq(ctx, 0x65, 0xB0); // scaling
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(170);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_ON);
}

static void dsi_write_panel_off(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_OFF);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(200);
}

static int set_brightness(struct lpm026m648c_ctx *ctx, u16 brightness,
			  u16 pulse_offset_rows)
{

	if (!ctx->dphy) {
		ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
		if (ctx->is_dual_panel)
			ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	}

	lpm026m648c_dsi_write_seq(ctx, 0x51, brightness); // full brightness

	if (!ctx->dphy) {
		ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
		if (ctx->is_dual_panel)
			ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;
	}
	return 0;
}

static int lpm026m648c_power_on(struct lpm026m648c_ctx *ctx)
{
	int ret;

    // VDDI 1.8V
	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->power_supplies),
				    ctx->power_supplies);
	if (ret < 0)
		return ret;

	usleep_range(10000, 20000);

	// turn on AVDD
	ret = regulator_enable(ctx->lcd_bias_regulators[LEFT_POS].consumer);
	ret |= regulator_enable(ctx->lcd_bias_regulators[RIGHT_POS].consumer);
	if ( ret ) {
		dev_err(ctx->dev, "Could not enable AVDD\n");
		return ret;
	}

	// set voltages here since we can't talk until after they are enabled
	// make sure we set the bias voltage here
	ret = regulator_set_voltage(ctx->lcd_bias_regulators[LEFT_POS].consumer, 5500000, 5500000);
	ret |= regulator_set_voltage(ctx->lcd_bias_regulators[RIGHT_POS].consumer, 5500000, 5500000);
	if ( ret ) {
		dev_err(ctx->dev, "Could not set AVDD\n");
		return ret;
	}
    // MIPI 11
	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ctx->is_dual_panel)
		ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	usleep_range(10000, 20000);
	
	gpiod_set_value(ctx->reset_gpios->desc[0], 1);
	gpiod_set_value(ctx->reset_gpios->desc[1], 1);

    usleep_range(10000, 20000);

	// interrupt callback to turn on AVEE/ELVSS when TE gets set
	return 0;
}

static int lpm026m648c_power_off(struct lpm026m648c_ctx *ctx)
{
	int ret;
	gpiod_set_value(ctx->reset_gpios->desc[0], 0);
	gpiod_set_value(ctx->reset_gpios->desc[1], 0);
	msleep(25);

	// turn off AVEE
	ret = regulator_disable(ctx->lcd_bias_regulators[LEFT_NEG].consumer);
	ret |= regulator_disable(ctx->lcd_bias_regulators[RIGHT_NEG].consumer);
	if ( ret ) {
		dev_err(ctx->dev, "Could not disable AVEE\n");
	}
	usleep_range(1000, 2000);
	// turn off AVDD
	ret = regulator_disable(ctx->lcd_bias_regulators[LEFT_POS].consumer);
	ret |= regulator_disable(ctx->lcd_bias_regulators[RIGHT_POS].consumer);
	if ( ret ) {
		dev_err(ctx->dev, "Could not disable AVDD\n");
	}

	msleep(20);

	return regulator_bulk_disable(ARRAY_SIZE(ctx->power_supplies),
				      ctx->power_supplies);
}

static int lpm026m648c_disable(struct drm_panel *panel)
{
    int ret;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	if (!ctx->enabled)
		return 0;

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->is_dual_panel)
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	dsi_write_panel_off(panel);

	ret = lpm026m648c_power_off(ctx);
	if (ret < 0)
		DRM_DEV_ERROR(ctx->dev, "power_off failed ret = %d\n", ret);

	backlight_disable(ctx->backlight);

	ctx->enabled = false;
	return 0;
}

static int lpm026m648c_unprepare(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	int ret = 0;

	if (!ctx->prepared)
		return 0;

	// disable backlight drivers
	ret = regulator_bulk_disable(ARRAY_SIZE(ctx->backlight_drivers), ctx->backlight_drivers);
	if (ret) {
		DRM_DEV_ERROR(ctx->dev,
			"regulator_bulk_disable failed for backlight %d\n", ret);
	}

	// disable preboost drivers
	ret = regulator_bulk_disable(ARRAY_SIZE(ctx->preboost_drivers), ctx->preboost_drivers);
	if (ret) {
		DRM_DEV_ERROR(ctx->dev,
			"regulator_bulk_disable failed for preboost%d\n", ret);
	}
	
	ctx->prepared = false;
	return ret;
}

static int lpm026m648c_prepare(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	if (ctx->prepared)
		return 0;

// OLED display does not use backlight drivers
#if 0
	// turn on preboost drivers
	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->preboost_drivers), ctx->preboost_drivers);
	if (ret < 0)
	{
		DRM_DEV_ERROR(ctx->dev, "preboost on failed\n");
		goto power_off;
	}

	//we need to delay to wait for hiccup mode to exit on EV1 hardware --
	//at a minimum it is about 70ms we need to wait here...
	usleep_range(400000, 450000);

	// turn on backlight drivers
	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->backlight_drivers), ctx->backlight_drivers);
	if (ret < 0)
	{
		DRM_DEV_ERROR(ctx->dev, "backlight driver on failed\n");
		goto power_off;
	}
#endif
	// clear out panel manufacturing info
	memset(ctx->panel_mfg_info, 0, ARRAY_SIZE(ctx->panel_mfg_info));

	ctx->prepared = true;

	return 0;
}

static int lpm026m648c_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int lpm026m648c_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct lpm026m648c_ctx *lpm026m648c_ctx = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if (!lpm026m648c_ctx->prepared)
		return 0;

	ret = set_brightness(lpm026m648c_ctx, bl->props.brightness,
			     bl->props.pulse_offset_rows);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops lpm026m648c_bl_ops = {
	.update_status = lpm026m648c_bl_update_status,
	.get_brightness = lpm026m648c_bl_get_brightness,
};

static int lpm026m648c_enable(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	int ret;

	if (ctx->enabled)
		return 0;

	ret = lpm026m648c_power_on(ctx);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "power_on failed\n");
		goto power_off;
	}

	dsi_write_oled_panel_on(panel);

	ctx->enabled = true;

	return ret;

power_off:
	if (lpm026m648c_power_off(ctx))
		DRM_DEV_ERROR(ctx->dev, "power_off failed\n");
	return ret;
}

static int lpm026m648c_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	struct drm_display_mode *mode;
	int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		// only add 72 Hz if the backlight driver supports it
		if ( i == MODE_72 ) {
			if ( !ctx->low_fps_support ) {
				continue;
			}
		}

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

		if (ctx->is_dual_panel) {
			mode->name[0] = '2';
			mode->name[1] = '*';
			mode->clock *= 2;
			mode->hdisplay *= 2;
			mode->hsync_start *= 2;
			mode->hsync_end *= 2;
			mode->htotal *= 2;
			mode->width_mm *= 2;
		}
		drm_mode_probed_add(connector, mode);
	}

	return i;
}

static int lpm026m648c_mode_set(struct drm_panel *panel,
			       const struct drm_display_mode *mode,
			       const struct drm_display_mode *adjusted_mode)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	lpm026m648c_modes_t new_mode;

	// use the vtotal to determine the mode
	if (mode->vtotal == modes[MODE_120].vtotal){
			new_mode = MODE_120;
	}else if (mode->vtotal == modes[MODE_90].vtotal){
			new_mode = MODE_90;
	}else if (mode->vtotal == modes[MODE_80].vtotal){
			new_mode = MODE_80;
	}else if (mode->vtotal == modes[MODE_72].vtotal){
			new_mode = MODE_72;
	}else if (mode->vtotal == modes[MODE_144].vtotal){
			new_mode = MODE_144;
	}else{
			DRM_DEV_ERROR(ctx->dev, "Invalid vtotal in mode set %d", mode->vtotal);
			return -EINVAL;
	}

	// check if we need to update settings
	if (ctx->current_mode != new_mode)
	{
		ctx->current_mode = new_mode;
		// only reset the panel parematers if we already initialized the panels
		if (ctx->enabled)
		{
			// update vcom
			update_vcom(ctx);
			// update brightness
			set_brightness(ctx, ctx->backlight->props.brightness, ctx->backlight->props.pulse_offset_rows);
		}
	}

	return 0;
}

static int panel_info_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	unsigned int id, offset, panel_num;
	u8 *info = ctx->panel_mfg_info;

	panel_num = ctx->is_dual_panel ? 2 : 1;
	for (id = 0; id < panel_num; id++) {
		offset =  id*PANEL_MFG_INFO_NUM_BYTES;

		seq_printf(s, "****** JDI Panel info for %s panel:\n",
			   id ? "right" : "left");
		seq_printf(s, " SupplierID  %x\n",
			   info[1+offset] << 8 | info[0+offset]);
		seq_printf(s, " DriverICProductCode  %x\n",
			   info[3+offset] << 8 | info[2+offset]);
		seq_printf(s, " brightnessDataW255  %x\n", info[5+offset]);
		seq_printf(s, " colorPointofWhiteX  %x\n",  info[6+offset]);
		seq_printf(s, " colorPointofWhiteY  %x\n",  info[7+offset]);
		seq_printf(s, " contrast  %x\n",  info[8+offset]);
		seq_printf(s, " gamma  %x\n",  info[9+offset]);
		seq_printf(s, " brightnessDataW32  %x\n",  info[12+offset]);
		seq_printf(s, " 120 vcom1  %x\n",  info[13+offset]);
		seq_printf(s, " 120 vcom2  %x\n",  info[14+offset]);
		seq_printf(s, " 90 vcom1  %x\n",  info[15+offset]);
		seq_printf(s, " 90 vcom2  %x\n",  info[16+offset]);
		seq_printf(s, " colorPointofRedX  %x\n",  info[21+offset]);
		seq_printf(s, " colorPointofRedY  %x\n",  info[22+offset]);
		seq_printf(s, " colorPointofGreenX  %x\n",  info[23+offset]);
		seq_printf(s, " colorPointofGreenY  %x\n",  info[24+offset]);
		seq_printf(s, " colorPointofBlueX  %x\n",  info[25+offset]);
		seq_printf(s, " colorPointofBlueY  %x\n",  info[26+offset]);
		seq_printf(s, " format  %x\n",  info[27+offset]);
		seq_printf(s, " vendor1  %x\n",  info[28+offset]);
		seq_printf(s, " vendor2  %x\n",  info[29+offset]);
		seq_printf(s, " mdlBaseCode1  %x\n",  info[30+offset]);
		seq_printf(s, " mdlBaseCode2  %x\n",  info[31+offset]);
		seq_printf(s, " mdlBaseCode3  %x\n",  info[32+offset]);
		seq_printf(s, " id  %x\n",  info[33+offset]);
		seq_printf(s, " serialization3  %x\n",  info[34+offset]);
		seq_printf(s, " serialization2  %x\n",  info[35+offset]);
		seq_printf(s, " serialization1  %x\n",  info[36+offset]);
		seq_printf(s, " year  %x\n",  info[37+offset]);
		seq_printf(s, " week  %x\n",  info[38+offset]);
		seq_printf(s, " day  %x\n",  info[39+offset]);
	}
	return 0;
}

static int panel_mfg_info_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ctx->panel_mfg_info); i++) {
		seq_printf(s, "%u ",  ctx->panel_mfg_info[i]);
	}
	seq_printf(s, "\n");
	return 0;
}

static int panel_read_show(struct seq_file *s, void *data)
{
	int ret;
	unsigned int i;
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	if (ctx->panel_read_reg_addr >= 0xb0) {
		ret = mipi_dsi_generic_read(ctx->dsi[ctx->panel_read_id], &ctx->panel_read_reg_addr, 1, ctx->panel_read_data, ctx->panel_read_num_bytes);
	} else {
		ret = mipi_dsi_dcs_read(ctx->dsi[ctx->panel_read_id], ctx->panel_read_reg_addr, ctx->panel_read_data, ctx->panel_read_num_bytes);
	}
	
	if ( ret < 0 )
	{
		DRM_DEV_ERROR(ctx->dev, "failed to read panel %d reg %u\n", ctx->panel_read_id, ctx->panel_read_reg_addr);
		return ret;
	}
	else
	{
		for (i = 0; i < ctx->panel_read_num_bytes; i++) {
			seq_printf(s, "%02x ",  ctx->panel_read_data[i]);
		}
		seq_printf(s, "\n");
		return 0;
	}
}

static ssize_t panel_read_write(struct file *file,
					  const char __user *userbuf,
					  size_t count, loff_t *ppos)
{
	int ret;
	struct seq_file *s = file->private_data;
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	u8 buf[32];

	if (copy_from_user(buf, userbuf, count))
		return -EOVERFLOW;

	ret = sscanf(buf, "%d %2x %d", &ctx->panel_read_id, &ctx->panel_read_reg_addr, &ctx->panel_read_num_bytes);
	if(ret != 3)
	{
		DRM_DEV_ERROR(ctx->dev, "panel read got wrong number of input parameters from sscanf");
		return -EINVAL;
	}
	if( ctx->panel_read_id != 0 && ctx->panel_read_id != 1)
	{
		// invalid panel id
		DRM_DEV_ERROR(ctx->dev, "panel read found invalid id value %u", ctx->panel_read_id);
		ctx->panel_read_id = 0;
		return -EINVAL;
	}
	return count;
}

static ssize_t panel_write_write(struct file *file,
					  const char __user *userbuf,
					  size_t count, loff_t *ppos)
{
	int ret;
	int id, addr, len, i, v;
	char char_data[256];
	u8 payload[256];
	u8 buf[256];
	struct seq_file *s = file->private_data;
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	if (copy_from_user(buf, userbuf, count))
		return -EOVERFLOW;

	// get the base parameters
	ret = sscanf(buf, "%d %2x %d %s", &id, &addr, &len, char_data);
	if(ret != 4)
	{
		DRM_DEV_ERROR(ctx->dev, "panel write got wrong number of input parameters (%d) from sscanf", ret);
		return -EINVAL;
	}
	if(id != 0 && id != 1)
	{
		// invalid panel id
		DRM_DEV_ERROR(ctx->dev, "panel read found invalid id value %u", ctx->panel_read_id);
		ctx->panel_read_id = 0;
		return -EINVAL;
	}
	// we limit writes to a total of 256 bytes
	if(len >256)
	{
		DRM_DEV_ERROR(ctx->dev, "jdi4k panel write exceed max length %d\n", len);
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
		ret = mipi_dsi_generic_write(ctx->dsi[id], payload, len+1);
	else if (len == 0)
		ret = mipi_dsi_dcs_write(ctx->dsi[id], addr, NULL, 0);
	else
		ret = mipi_dsi_dcs_write_buffer(ctx->dsi[id], payload, len+1);
	if (ret < 0)
	{
		DRM_DEV_ERROR(ctx->dev, "failed to tx cmd [%.2x], err: %d\n", payload[0], ret);
	}

	return count;
}

static int panel_write_show(struct seq_file *s, void *data)
{
	return 0;
}

static int panel_available_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	seq_printf(s, "%d\n",  ctx->enabled );
	return 0;
}

// RCLK = 32.5 MHz
#define RCLK_PERIOD_NSEC ( 1000000/32500 )
#define INTERNAL_LINE_TIME_NSEC  ( RTN * RCLK_PERIOD_NSEC )
#define EXTERNAL_LINE_TIME_NSEC ( ( 1000000 * (uint32_t)modes[ctx->current_mode].htotal ) / ( (uint32_t)modes[ctx->current_mode].clock ) )

static int panel_vsync_to_photons_usec_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	uint32_t vsync_to_photons_nsec, gpo1_tes1, frame_period_nsec;

	// set offset based on frame rate to optimize settling time vs latency
	if ( modes[ctx->current_mode].vtotal == modes[MODE_144].vtotal ) {
		gpo1_tes1 = ctx->panel_vendor == VENDOR_SHARP ? GPO1_TES1_144_SHARP : GPO1_TES1_144_BOE;
	} else if ( modes[ctx->current_mode].vtotal == modes[MODE_120].vtotal ) {
		gpo1_tes1 = ctx->panel_vendor == VENDOR_SHARP ? GPO1_TES1_120_SHARP : GPO1_TES1_120_BOE;
	} else {
		gpo1_tes1 = GPO1_TES1_90;
	}

	if ( ctx->panel_vendor == VENDOR_JDI 
		|| ctx->panel_vendor == VENDOR_BOE
		|| ctx->current_mode == MODE_72
		|| ctx->current_mode == MODE_80
		|| ctx->current_mode == MODE_90 ) {
		// VID_VS_DELAY + TES
		vsync_to_photons_nsec = VID_VS_DELAY * EXTERNAL_LINE_TIME_NSEC + gpo1_tes1 * INTERNAL_LINE_TIME_NSEC;
	} else { // SHARP 120/144
		switch ( ctx->current_mode )
		{
			case MODE_144:
				frame_period_nsec = 1000000000 / 144;
				break;
			case MODE_120:
				frame_period_nsec = 1000000000 / 120;
				break;
			default:
				frame_period_nsec = 1000000000 / 120;
		}
		// 1/FPS + TES
		vsync_to_photons_nsec = VID_VS_DELAY * EXTERNAL_LINE_TIME_NSEC + frame_period_nsec + gpo1_tes1 * INTERNAL_LINE_TIME_NSEC;
	}
	seq_printf(s, "%u\n", vsync_to_photons_nsec / 1000);
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(panel_info);
DEFINE_SHOW_ATTRIBUTE(panel_mfg_info);
DEFINE_SHOW_ATTRIBUTE(panel_available);
DEFINE_SHOW_ATTRIBUTE(panel_vsync_to_photons_usec);
DEFINE_SHOW_STORE_ATTRIBUTE(panel_read);
DEFINE_SHOW_STORE_ATTRIBUTE(panel_write);

static void lpm026m648c_debugfs_init(struct drm_panel *panel, struct dentry *root)
{
	debugfs_create_file("panel_info", 0600, root, panel, &panel_info_fops);
	debugfs_create_file("panel_mfg_info", 0600, root, panel, &panel_mfg_info_fops);
	debugfs_create_file("panel_available", 0600, root, panel, &panel_available_fops);
	debugfs_create_file("panel_vsync_to_photons_usec", 0600, root, panel, &panel_vsync_to_photons_usec_fops);
	debugfs_create_file("panel_write", 0600, root, panel, &panel_write_fops);
	debugfs_create_file("panel_read", 0600, root, panel, &panel_read_fops);
}

static const struct drm_panel_funcs lpm026m648c_drm_funcs = {
	.disable = lpm026m648c_disable,
	.enable = lpm026m648c_enable,
	.unprepare = lpm026m648c_unprepare,
	.prepare = lpm026m648c_prepare,
	.get_modes = lpm026m648c_get_modes,
	.debugfs_init = lpm026m648c_debugfs_init,
};

static irqreturn_t te_irq_handler_top(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t te_irq_handler(int irq, void *dev_id)
{
	int ret = 0;
	struct lpm026m648c_ctx *ctx = dev_id;

	// turn on AVEE/ELVSS ( left / right is tied together so we just turn them both on )
	ret = regulator_enable(ctx->lcd_bias_regulators[LEFT_NEG].consumer);
	ret |= regulator_enable(ctx->lcd_bias_regulators[RIGHT_NEG].consumer);
	if ( ret ) {
		dev_err(ctx->dev, "Could not enable  AVEE/ELVSS \n");
		return ret;
	}

	// make sure we set the bias voltage here
	ret = regulator_set_voltage(ctx->lcd_bias_regulators[RIGHT_NEG].consumer, 6300000, 6300000);
	ret |= regulator_set_voltage(ctx->lcd_bias_regulators[LEFT_NEG].consumer, 6300000, 6300000);

	if ( ret ) {
		dev_err(ctx->dev, "Could not set AVEE\n");
		return ret;
	}

	DRM_DEV_INFO(ctx->dev, "ELVSS enabled\n");

	return IRQ_HANDLED;
}


static int lpm026m648c_panel_add(struct lpm026m648c_ctx *ctx)
{
	struct device *dev = ctx->dev;
	struct backlight_properties bl_props;
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(ctx->power_supplies); i++)
		ctx->power_supplies[i].supply = regulator_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->power_supplies),
				      ctx->power_supplies);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "Failed to get power supplies\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ctx->preboost_drivers); i++)
						ctx->preboost_drivers[i].supply = preboost_driver_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->preboost_drivers),
						ctx->preboost_drivers);
	if (ret < 0){
		DRM_DEV_ERROR(ctx->dev, "Failed to get preboost supplies\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ctx->backlight_drivers); i++)
		ctx->backlight_drivers[i].supply = backlight_driver_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->backlight_drivers),
				      ctx->backlight_drivers);
	if (ret < 0){
		DRM_DEV_ERROR(ctx->dev, "Failed to get backlight supplies\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(ctx->lcd_bias_regulators); i++)
		ctx->lcd_bias_regulators[i].supply = lcd_bias_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->lcd_bias_regulators),
		ctx->lcd_bias_regulators);
	if (ret < 0){
		DRM_DEV_ERROR(ctx->dev, "Failed to get lcd bias supplies\n");
		return ret;
	}

	ctx->reset_gpios = devm_gpiod_get_array(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpios)){
		DRM_DEV_ERROR(ctx->dev, "Failed to get reset gpio\n");
		return PTR_ERR(ctx->reset_gpios);
	}

	ctx->te_gpios = devm_gpiod_get_array(dev, "te", GPIOD_IN);
	if (IS_ERR(ctx->te_gpios)){
		DRM_DEV_ERROR(ctx->dev, "Failed to get te gpios\n");
		return PTR_ERR(ctx->te_gpios);
	}

	ctx->te_left_gpio_irq = gpiod_to_irq(ctx->te_gpios->desc[0]);
	ctx->te_right_gpio_irq = gpiod_to_irq(ctx->te_gpios->desc[1]);

	ret = devm_request_threaded_irq(
		dev, ctx->te_left_gpio_irq,
		te_irq_handler_top,
		te_irq_handler, IRQF_TRIGGER_RISING,
		"TE_LEFT", ctx);

	if (ret) {
		DRM_DEV_ERROR(dev,
			"request te left alert interrupt failed with %d\n",
			ret);
		return ret;
	}
	
	ret = devm_request_threaded_irq(
		dev, ctx->te_right_gpio_irq,
		te_irq_handler_top,
		te_irq_handler, IRQF_TRIGGER_RISING,
		"TE_RIGHT", ctx);

	if (ret) {
		DRM_DEV_ERROR(dev,
			"request te right alert interrupt failed with %d\n",
			ret);
		return ret;
	}

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 0xFF;
	bl_props.max_brightness = 0xFF;

	ctx->backlight = devm_backlight_device_register(dev, dev_name(dev),
							dev, ctx->dsi[0],
							&lpm026m648c_bl_ops,
							&bl_props);
	if (IS_ERR(ctx->backlight)) {
		ret = PTR_ERR(ctx->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_init(&ctx->panel, dev, &lpm026m648c_drm_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	drm_panel_add(&ctx->panel);

	return 0;
}

static int lpm026m648c_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lpm026m648c_ctx *ctx;
	struct mipi_dsi_device *dsi1_device;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct mipi_dsi_device *dsi_dev;
	int ret = 0, dsi_num = 1;
	int i;

	const struct mipi_dsi_device_info info = {
		.type = "deckard4k",
		.channel = 0,
		.node = NULL,
	};

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);

	if (!ctx)
		return -ENOMEM;

	ctx->panel.prepare_prev_first = true;

	ctx->dphy = of_property_read_bool(dev->of_node, "dphy");

	ctx->is_dual_panel = of_property_read_bool(dev->of_node, "dual-panel");
	if (!ctx->is_dual_panel)
		goto skip_panel2;

	dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
	if (!dsi1) {
		DRM_DEV_ERROR(dev, "failed to get secondary dsi\n");
		return -ENODEV;
	}

	dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
	of_node_put(dsi1);
	if (!dsi1_host) {
		DRM_DEV_ERROR(dev, "failed to find secondary dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi1_device = mipi_dsi_device_register_full(dsi1_host, &info);
	if (IS_ERR(dsi1_device)) {
		DRM_DEV_ERROR(dev, "failed to create dsi device\n");
		return PTR_ERR(dsi1_device);
	}
	ctx->dsi[1] = dsi1_device;
	dsi_num = 2;

skip_panel2:
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	ctx->dsi[0] = dsi;

#ifndef NO_DSC
	ret = lpm026m648c_init_dsc_config(ctx);
	if (ret)
		goto err_panel_add;

	ctx->dsi[0]->dsc = &ctx->dsc_cfg;
	ctx->dsi[0]->dsc_slice_per_pkt = 4;
	if (ctx->is_dual_panel) {
		ctx->dsi[1]->dsc = &ctx->dsc_cfg;
		ctx->dsi[1]->dsc_slice_per_pkt = 4;
	}
#endif

	ctx->low_fps_support = of_property_read_bool(dev->of_node, "low-fps-support");

	ret = lpm026m648c_panel_add(ctx);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to add panel %d\n", ret);
		goto err_panel_add;
	}

	for (i = 0; i < dsi_num; i++) {
		dsi_dev = ctx->dsi[i];
		if (!ctx->dphy ) {
			dsi_dev->lanes = 3;
		} else {
			dsi_dev->lanes = 4;
		}
		dsi_dev->format = MIPI_DSI_FMT_RGB888;
		dsi_dev->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM |
			MIPI_DSI_CLOCK_NON_CONTINUOUS;
		ret = mipi_dsi_attach(dsi_dev);
		if (ret < 0) {
			DRM_DEV_ERROR(dev, "dsi attach failed i = %d\n", i);
			goto err_dsi_attach;
		}
	}

	return 0;

err_dsi_attach:
	drm_panel_remove(&ctx->panel);
err_panel_add:
	mipi_dsi_device_unregister(dsi1_device);
	return -EPROBE_DEFER;;
}

static void lpm026m648c_remove(struct mipi_dsi_device *dsi)
{
	struct lpm026m648c_ctx *ctx = mipi_dsi_get_drvdata(dsi);

	if (ctx->dsi[0])
		mipi_dsi_detach(ctx->dsi[0]);
	if (ctx->dsi[1]) {
		mipi_dsi_detach(ctx->dsi[1]);
		mipi_dsi_device_unregister(ctx->dsi[1]);
	}

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id lpm026m648c_of_match[] = {
	{ .compatible = "deckard-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, lpm026m648c_of_match);

static struct mipi_dsi_driver lpm026m648c_driver = {
	.driver = {
		.name = "deckard-panel",
		.of_match_table = lpm026m648c_of_match,
	},
	.probe = lpm026m648c_probe,
	.remove = lpm026m648c_remove,
};
module_mipi_dsi_driver(lpm026m648c_driver);

MODULE_DESCRIPTION("Sharp LS0DAS7232 DSI Panel Driver with BOE support");
MODULE_LICENSE("GPL v2");
