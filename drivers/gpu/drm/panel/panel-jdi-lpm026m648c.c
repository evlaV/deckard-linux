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
#define LPM02648C_TE_GPIO_CTL			0xbe
#define LPM02648C_PPS_SET			0xe6

// Brightness settings
#define BRIGHTNESS_DEFAULT 256
#define BRIGHTNESS_MAX_120 380

#define VFP		700
#define VID_VS_DELAY	400
#define VBP		800
#define GPO1_TES1	233
#define GPO1_TEW1	17

/*
 * Camera capture signal on panel tester is GPIO2 routed out TE pin
 * Need 100usec of offest on either side, so we use 50 lines at
 * 2usec / internal line time
 */
#define CAMERA_TRIGGER_OFFSET 50

static const char * const regulator_names[] = {
	"vdda",
};

static const char * const bl_driver_names[] = {
	"wled1",
};

#define PANEL_MFG_INFO_NUM_BYTES 40

struct lpm026m648c_ctx {
	struct device *dev;
	struct drm_panel panel;

	struct regulator_bulk_data power_supplies[ARRAY_SIZE(regulator_names)];
	struct regulator_bulk_data backlight_drivers[ARRAY_SIZE(bl_driver_names)];

	struct gpio_descs *reset_gpios;
	struct gpio_desc *enp_gpio;

	struct backlight_device *backlight;
	struct mipi_dsi_device *dsi[2];
	u8 panel_mfg_info[PANEL_MFG_INFO_NUM_BYTES*2];
	u8 panel_read_data[512];

	struct drm_dsc_config dsc_cfg;
	bool is_dual_panel;
	bool prepared;
	bool enabled;
};

static inline struct lpm026m648c_ctx *panel_to_ctx(struct drm_panel *panel)
{
	return container_of(panel, struct lpm026m648c_ctx, panel);
}

static const struct drm_display_mode modes[3] = {
	{
		.name = "  2160x2160_120",
		.clock = (2160 + 24 + 20 + 20) * (2160 + 404 + 1 + 20) * 120 / 1000,
		.hdisplay = 2160,
		.hsync_start = (2160 + 24),
		.hsync_end = (2160 + 24 + 20),
		.htotal = (2160 + 24 + 20 + 20),
		.vdisplay = 2160,
		.vsync_start = 2160 + 404,
		.vsync_end = 2160 + 404 + 1,
		.vtotal = 2160 + 404 + 1 + 20,
		.width_mm = 50,
		.height_mm = 50,

	},
	{
		.name = "  2160x2160_90",
		.clock = (2160 + 24 + 20 + 20) * (2160 + 1265 + 1 + 20) * 90 / 1000,
		.hdisplay = 2160,
		.hsync_start = (2160 + 24),
		.hsync_end = (2160 + 24 + 20),
		.htotal = (2160 + 24 + 20 + 20),
		.vdisplay = 2160,
		.vsync_start = 2160 + 1265,
		.vsync_end = 2160 + 1265 + 1,
		.vtotal = 2160 + 1265 + 1 + 20,
		.width_mm = 50,
		.height_mm = 50,
	},
	{
		.name = "  2160x2160_80",
		.clock = (2160 + 24 + 20 + 20) * (2160 + 1697 + 1 + 20) * 80 / 1000,
		.hdisplay = 2160,
		.hsync_start = (2160 + 24),
		.hsync_end = (2160 + 24 + 20),
		.htotal = (2160 + 24 + 20 + 20),
		.vdisplay = 2160,
		.vsync_start = 2160 + 1697,
		.vsync_end = 2160 + 1697 + 1,
		.vtotal = 2160 + 1697 + 1 + 20,
		.width_mm = 50,
		.height_mm = 50,
	},
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
	dsc->line_buf_depth = dsc->bits_per_component + 1;

	return drm_dsc_compute_rc_parameters(dsc);
}

static int lpm026m648c_init_dsc_config(struct lpm026m648c_ctx *ctx)
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

	return dsi_populate_dsc_params(ctx);
}

static void dsi_write_panel_on(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	u8 pps_cmd[1 + sizeof(struct drm_dsc_picture_parameter_set)];

	drm_dsc_pps_payload_pack((void *)&pps_cmd[1], &ctx->dsc_cfg);
	pps_cmd[0] = LPM02648C_PPS_SET;

	lpm026m648c_dsi_write_seq(ctx, LPM02648C_MF_CMD_ACCESS_PROTECT, 0x04);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_SEQ_CTL, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DSI_CTL,
				  0x20, 0x6b, 0x80, 0x06, 0x33, 0x8a, 0x00,
				  0x1a, 0x7a);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_MODE, 0x54, 0x00, 0x00,
				  0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_GEN_OUTPIN_SET,
				  BE16(GPO1_TES1), BE16(GPO1_TEW1), 0x00, 0x00,
				  0x00, 0x00,
				  BE16(GPO1_TES1-CAMERA_TRIGGER_OFFSET),
				  BE16(CAMERA_TRIGGER_OFFSET * 2 + GPO1_TEW1));
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET2, 0x1e);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET3, 0x70, 0x08, 0xd0,
				  0x02, 0x21, 0x6f, 0x08, 0x5a, 0x00, 0x00,
				  0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET1, 0x42, 0x86,
				  LE16(VBP), 0x08, 0x70, BE16(VFP));
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET3_2, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET4, 0x8b, 0x00, 0x80,
				  0x46, 0x61, 0x00, 0x8b);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_DISP_SET5, BE16(VID_VS_DELAY),
				  0x00, 0x00, 0x00);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_TE_GPIO_CTL, 0x00, 0x6A, 0x02);
	lpm026m648c_dsi_write_buffer(ctx, pps_cmd);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_GET_COMPRESSION_MODE, 0x01);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_SCANLINE, 0x00, 0x00);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_ON, 0x00);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_ADDRESS_MODE, 0x00);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_PIXEL_FORMAT, 0x77);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(170);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_ON);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_SEQ_CTL, 0x80);
	msleep(200);
	lpm026m648c_dsi_write_seq(ctx, LPM02648C_MF_CMD_ACCESS_PROTECT, 0x03);
	msleep(200);
}

static void dsi_write_panel_off(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_OFF);
	lpm026m648c_dsi_write_seq(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(200);
}

static void get_panel_info(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	int ret;
	unsigned int id, offset, panel_num;
	u8 panel_info_regs[] = { 0xBF };
	u8 *info = ctx->panel_mfg_info;

	panel_num = ctx->is_dual_panel ? 2 : 1;
	for (id = 0; id < panel_num; id++) {
		offset =  id*PANEL_MFG_INFO_NUM_BYTES;
		ret = mipi_dsi_generic_read(ctx->dsi[id], panel_info_regs, 1,
				&info[offset], PANEL_MFG_INFO_NUM_BYTES);
		if (ret < 0)
			DRM_DEV_ERROR(ctx->dev, "failed to read panel%d info\n", id);
	}
}

static int set_brightness(struct lpm026m648c_ctx *ctx, u16 brightness,
			  u16 pulse_offset_rows)
{
	u16 gpo1_tes1 = pulse_offset_rows;
	u16 gpo1_tew1;
	u16 cam_trig_start = 0, cam_trig_width = 0;

	gpo1_tew1 = brightness > BRIGHTNESS_MAX_120 ?
		    BRIGHTNESS_MAX_120 : brightness;

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->is_dual_panel)
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	if (brightness) {
		cam_trig_start = (gpo1_tes1 > CAMERA_TRIGGER_OFFSET) ?
				 (gpo1_tes1 - CAMERA_TRIGGER_OFFSET) : 0;
		cam_trig_width = gpo1_tew1 + CAMERA_TRIGGER_OFFSET*2;
	}

	lpm026m648c_dsi_write_seq(ctx, LPM02648C_GEN_OUTPIN_SET,
				  BE16(gpo1_tes1), BE16(gpo1_tew1), 0x00, 0x00,
				  0x00, 0x00, BE16(cam_trig_start),
				  BE16(cam_trig_width));

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ctx->is_dual_panel)
		ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;
	return 0;
}

static int lpm026m648c_power_on(struct lpm026m648c_ctx *ctx)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->power_supplies),
				    ctx->power_supplies);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);
	gpiod_set_value(ctx->enp_gpio, 1);
	msleep(20);
	gpiod_set_value(ctx->reset_gpios->desc[0], 1);
	gpiod_set_value(ctx->reset_gpios->desc[1], 1);
	return 0;
}

static int lpm026m648c_power_off(struct lpm026m648c_ctx *ctx)
{
	gpiod_set_value(ctx->reset_gpios->desc[0], 0);
	gpiod_set_value(ctx->reset_gpios->desc[1], 0);
	msleep(25);
	gpiod_set_value(ctx->enp_gpio, 0);
	msleep(20);

	return regulator_bulk_disable(ARRAY_SIZE(ctx->power_supplies),
				      ctx->power_supplies);
}

static int lpm026m648c_disable(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);

	if (!ctx->enabled)
		return 0;

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

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->is_dual_panel)
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = regulator_bulk_disable(ARRAY_SIZE(ctx->backlight_drivers),
				     ctx->backlight_drivers);
	if (ret)
		DRM_DEV_ERROR(ctx->dev,
			"failed to disable backlight %d\n", ret);

	dsi_write_panel_off(panel);

	ret = lpm026m648c_power_off(ctx);
	if (ret < 0)
		DRM_DEV_ERROR(ctx->dev, "power_off failed ret = %d\n", ret);

	ctx->prepared = false;
	return ret;
}

static int lpm026m648c_prepare(struct drm_panel *panel)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = lpm026m648c_power_on(ctx);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "power_on failed\n");
		goto power_off;
	}

	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ctx->is_dual_panel)
		ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

	msleep(20);
	dsi_write_panel_on(panel);
	get_panel_info(panel);

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->backlight_drivers),
				    ctx->backlight_drivers);
	if (ret < 0)
		goto power_off;

	ctx->backlight->props.pulse_offset_rows = GPO1_TES1;
	set_brightness(ctx, ctx->backlight->props.brightness,
		       ctx->backlight->props.pulse_offset_rows);

	ctx->prepared = true;

	return 0;

power_off:
	if (lpm026m648c_power_off(ctx))
		DRM_DEV_ERROR(ctx->dev, "power_off failed\n");
	return ret;
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

	ret = backlight_enable(ctx->backlight);

	ctx->enabled = true;

	return ret;
}

static int lpm026m648c_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	struct lpm026m648c_ctx *ctx = panel_to_ctx(panel);
	struct drm_display_mode *mode;
	int i;

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
		seq_printf(s, " vcom1  %x\n",  info[13+offset]);
		seq_printf(s, " vcom2  %x\n",  info[14+offset]);
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

DEFINE_SHOW_ATTRIBUTE(panel_info);

static void lpm026m648c_debugfs_init(struct drm_panel *panel, struct dentry *root)
{
	debugfs_create_file("panel_info", 0600, root, panel, &panel_info_fops);
}

static const struct drm_panel_funcs lpm026m648c_drm_funcs = {
	.disable = lpm026m648c_disable,
	.enable = lpm026m648c_enable,
	.unprepare = lpm026m648c_unprepare,
	.prepare = lpm026m648c_prepare,
	.get_modes = lpm026m648c_get_modes,
	.debugfs_init = lpm026m648c_debugfs_init,
};

static int lpm026m648c_panel_add(struct lpm026m648c_ctx *ctx)
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
		ctx->backlight_drivers[i].supply = bl_driver_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->backlight_drivers),
				      ctx->backlight_drivers);
	if (ret < 0)
		return ret;

	ctx->reset_gpios = devm_gpiod_get_array(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpios))
		return PTR_ERR(ctx->reset_gpios);

	ctx->enp_gpio = devm_gpiod_get(dev, "enp", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enp_gpio)) {
		DRM_DEV_ERROR(dev, "cannot get enp gpio %ld\n",
			PTR_ERR(ctx->enp_gpio));
		return PTR_ERR(ctx->enp_gpio);
	}

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = BRIGHTNESS_DEFAULT;
	bl_props.max_brightness = BRIGHTNESS_MAX_120;

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
		.type = "jdi4k",
		.channel = 0,
		.node = NULL,
	};

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);

	if (!ctx)
		return -ENOMEM;

	ctx->panel.prepare_prev_first = true;

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

	ret = lpm026m648c_init_dsc_config(ctx);
	if (ret)
		goto err_panel_add;

	ctx->dsi[0]->dsc = &ctx->dsc_cfg;
	ctx->dsi[0]->dsc_slice_per_pkt = 4;
	if (ctx->is_dual_panel) {
		ctx->dsi[1]->dsc = &ctx->dsc_cfg;
		ctx->dsi[1]->dsc_slice_per_pkt = 4;
	}

	ret = lpm026m648c_panel_add(ctx);
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to add panel\n");
		goto err_panel_add;
	}

	for (i = 0; i < dsi_num; i++) {
		dsi_dev = ctx->dsi[i];
		dsi_dev->lanes = 3;
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
	return ret;
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
	{ .compatible = "pocf,jdi-lpm026m648c" },
	{ }
};
MODULE_DEVICE_TABLE(of, lpm026m648c_of_match);

static struct mipi_dsi_driver lpm026m648c_driver = {
	.driver = {
		.name = "panel-jdi-lpm026m648c",
		.of_match_table = lpm026m648c_of_match,
	},
	.probe = lpm026m648c_probe,
	.remove = lpm026m648c_remove,
};
module_mipi_dsi_driver(lpm026m648c_driver);

MODULE_DESCRIPTION("JDI LPM02648C DSI Panel Driver");
MODULE_LICENSE("GPL v2");
