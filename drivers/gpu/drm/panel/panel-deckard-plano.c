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
#include <linux/of.h>
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

//#define CPHY_DEBUG

// Panel settings
#define BACKPLANE_HAA	3552
#define BACKPLANE_VAA	3840

#define RTN 59

//TODO: Get camera trigger working w/ Plano
/*
 * Camera capture signal on panel tester is GPIO2 routed out TE pin
 * Need 100usec of offest on either side, so we use 50 lines at
 * 2usec / internal line time
 */
#define CAMERA_TRIGGER_OFFSET 50

static const char * const regulator_names[] = {
	"vdda",
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
} plano_bias_t;

//TODO: Update for Plano
#define PANEL_MFG_INFO_NUM_BYTES 40

/*
typedef enum
{
	MODE_G2X_90 = 0,
	MODE_G2X_60,
	MODE_G1_5X_90,
	MODE_G1_5X_60,
	MODE_TEST,
	MODE_F2_5X_45,
	MODE_F2_5X_60,
	MODE_F2_5X_90,
	MODE_F3_25X_60,
	MODE_F3_25XP_60,
	MODE_F3_25X_90,
	MODE_F3_25XP_90,
} plano_modes_t;
*/
//TODO: For now, don't bother trying to keep enum updated as try out different modes
typedef uint8_t plano_modes_t;

struct plano_ctx {
	//TODO: clean up unneeded fields
	struct device *dev;
	struct drm_panel panel;

	struct regulator_bulk_data power_supplies[ARRAY_SIZE(regulator_names)];
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
	//bool low_fps_support;
	bool prepared;
	bool enabled;

	char panel_vendor;

	uint16_t vcom120[2];
	uint16_t vcom90[2];

	plano_modes_t current_mode;
};

static inline struct plano_ctx *panel_to_ctx(struct drm_panel *panel)
{
	return container_of(panel, struct plano_ctx, panel);
}


//Global scaling (applied by DDIC)
typedef enum
{
	GSCALE_1X = 0,
	GSCALE_1_33X,
	GSCALE_1_5X,
	GSCALE_2X,
} plano_gscale_t;

//Foviated transport scaling
typedef enum
{
	FSCALE_1X = 0,	//Fovial transport disabled
	FSCALE_2_5X,
	FSCALE_2_75X,
	FSCALE_3_25X,
	FSCALE_4_25X,
} plano_fscale_t;

//Extended mode
struct extmode_s {
	struct drm_display_mode	mode;
	char 		name[DRM_DISPLAY_MODE_LEN];
	bool		enabled;
	u16			fps;
	u16			haa;
	u16			hfp;
	u8			hs;
	u16			hbp;
	u16			vaa;
	u16			vfp_p;
	u16			vfp_m;
	u8			vs;
	u16			vbp_p;
	u16			vbp_m;
	bool		dsc_en;
	u16			dsc_vslice_height;
	u16			dsc_hslice_count;
	u16			vid_vs_delay;
	u16			h_dummy;
	u16			v_dummy;
	plano_gscale_t	gscale;
	plano_fscale_t	fscale;

};

#define MODE_CNT			16

//Each enable define may enable multiple modes, but they must all share the same DSC settings
//TODO:  Currenly only the DSC settings of the first enabled mode are used when checking
//  validity of all enabled modes. Modes requiring different DSC settings will be rejected
//  after plano_get_modes tries to add them.
//  So, for now, must be selective about which modes to enable at the same time
#define GLOBAL_2X_MODE_EN			true
#define GLOBAL_2X_NODSC_MODE_EN		false
#define GLOBAL_1_5X_MODE_EN			false
#define GLOBAL_1_5X_NODSC_MODE_EN	false
#define FOVIAL_2_5X_MODE_EN			false
#define FOVIAL_2_75X_MODE_EN		false
#define FOVIAL_3_25X_MODE_EN		false
#define TEST_MODE_EN				false
#define LOW_BW_TEST_MODE_EN			false

//Will get converted to drm_display_mode later
static struct extmode_s extmodes[MODE_CNT] = {
	//2X global scaling; 90Hz; DSC
	{
		.enabled =				GLOBAL_2X_MODE_EN,
		.name =					"1776x1920_90",
		.fps =					90,
		.haa =					1776,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vaa =					1920,
		.vfp_p =				17,
		.vfp_m =				21,
		.vs =					1,
		.vbp_p =				17,
		.vbp_m =				8,
		.dsc_en =				true,
		.dsc_vslice_height =	48,
		.dsc_hslice_count =		4,
		.vid_vs_delay =			2, //dsc_hslice_count / 2,
		.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_2X,
		.fscale =				FSCALE_1X,
	},
	//2X global scaling; 72Hz; DSC
	{
		.enabled =				GLOBAL_2X_MODE_EN,
		.name =					"1776x1920_72",
		.fps =					72,
		.haa =					1776,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vaa =					1920,
		.vfp_p =				17,
		.vfp_m =				21,
		.vs =					1,
		.vbp_p =				17,
		.vbp_m =				8,
		.dsc_en =				true,
		.dsc_vslice_height =	48,
		.dsc_hslice_count =		4,
		.vid_vs_delay =			2,	//dsc_hslice_count / 2,
		.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_2X,
		.fscale =				FSCALE_1X,
	},
	//2X global scaling; 60Hz; DSC
	{
		.enabled =				GLOBAL_2X_MODE_EN,
		.name =					"1776x1920_60",
		.fps =					60,
		.haa =					1776,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vaa =					1920,
		.vfp_p =				17,
		.vfp_m =				21,
		.vs =					1,
		.vbp_p =				17,
		.vbp_m =				8,
		.dsc_en =				true,
		.dsc_vslice_height =	48,
		.dsc_hslice_count =		4,
		.vid_vs_delay =			2,	//dsc_hslice_count / 2,
		.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_2X,
		.fscale =				FSCALE_1X,
	},
	//2X global scaling; 45Hz; DSC
	{
		.enabled =				GLOBAL_2X_MODE_EN,
		.name =					"1776x1920_45",
		.fps =					45,
		.haa =					1776,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vaa =					1920,
		.vfp_p =				17,
		.vfp_m =				21,
		.vs =					1,
		.vbp_p =				17,
		.vbp_m =				8,
		.dsc_en =				true,
		.dsc_vslice_height =	48,
		.dsc_hslice_count =		4,
		.vid_vs_delay =			2,	//dsc_hslice_count / 2,
		.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_2X,
		.fscale =				FSCALE_1X,
	},
	//2X global scaling; 30Hz; DSC
	{
		//.enabled =				GLOBAL_2X_MODE_EN,
		.enabled =				LOW_BW_TEST_MODE_EN,
		.name =					"1776x1920_30",
		.fps =					30,
		.haa =					1776,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vaa =					1920,
		.vfp_p =				17,
		.vfp_m =				21,
		.vs =					1,
		.vbp_p =				17,
		.vbp_m =				8,
		.dsc_en =				true,
		.dsc_vslice_height =	48,
		.dsc_hslice_count =		4,
		.vid_vs_delay =			2,	//dsc_hslice_count / 2,
		.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_2X,
		.fscale =				FSCALE_1X,
	},
	//2X global scaling; 60Hz; No DSC
	{
		.enabled =				GLOBAL_2X_NODSC_MODE_EN,
		.name =					"1776x1920_60",
		.fps =					60,
		.haa =					1776,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vaa =					1920,
		.vfp_p =				17,
		.vfp_m =				21,
		.vs =					1,
		.vbp_p =				17,
		.vbp_m =				8,
		.dsc_en =				false,
		.dsc_vslice_height =	48,
		.dsc_hslice_count =		4,
		.vid_vs_delay =			0,
		.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_2X,
		.fscale =				FSCALE_1X,
	},
	//Test mode:
	//1.5X global scaling; 60Hz; No DSC
	//Include prior to modes under development to provide a safe default
	{
		.enabled =				TEST_MODE_EN,
		.name =					"2368x2560_60",
		.fps =					60,
		.haa =					2368,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vaa =					2560,
		.vfp_p =				8,
		.vfp_m =				22,
		.vs =					1,
		.vbp_p =				8,
		.vbp_m =				10,
		.dsc_en =				false,
		.dsc_vslice_height =	48,
		.dsc_hslice_count =		4,
		.vid_vs_delay =			0,
		.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_1_5X,
		.fscale =				FSCALE_1X,
	},
	//1.5X global scaling; 90Hz; DSC
	//WIP:  Needs changes in drivers/gpu/drm/msm/disp/dpu1/dpu_encoder_phys_vid.c to handle fractional DSC width
	{
		.enabled =				GLOBAL_1_5X_MODE_EN,
		//.enabled =				false,
		.name =					"2368x2560_90",
		.fps =					90,
		.haa =					2368,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vaa =					2560,
		//App note recommendation:
		//TODO: Works w/ NO_DSC... not for DSC, but that might be due to other complications (like slice height)
		.vfp_p =				8,
		.vfp_m =				22,
		.vs =					1,
		//.vbp_p =				8,
		//.vbp_m =				10,
		.dsc_en =				true,

		//Doesn't work: dmesg: msm_dsi ae94000.dsi: [drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.dsc_vslice_height =	40,
		//.v_dummy =				0,

		//Doesn't work: dmesg: msm_dsi ae94000.dsi: [drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.dsc_vslice_height =	48,
		//.v_dummy =				32,
		//.vid_vs_delay =			0,

		//Doesn't work: dmesg: msm_dsi ae94000.dsi: [drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//W/ fractional correction in dpu_encoder_phys_vid.c, works w/ manual override of vid_vs_delay to 1, but w/ additional overlapping lines
		//.dsc_vslice_height =	48,
		//.v_dummy =				32,
		//.vid_vs_delay =			2,

		//Works W/ fractional correction in dpu_encoder_phys_vid.c, works w/ manual override of vid_vs_delay to 1, but w/ additional overlapping lines
		//Overriding display_color vertical lines to 2560 didn't help
		//Also, dummy lines need to be added to mode.vdisplay, otherwise get mode validation errors about pic_height not being a multiple of slice height...
		//  DSI: pic_height 2568 has to be multiple of slice 48
		//TODO:  But, pic_height should already account for dummy lines, so is error miscalculated?  
		//.dsc_vslice_height =	48,
		//.v_dummy =				32,
		//.vid_vs_delay =			1,

		//No difference in vertical overlap...
		//.dsc_vslice_height =	24,
		//.v_dummy =				8,
		//.vid_vs_delay =			1,

		//Moved dummy addition to just total, and changed mode validation to check dsc->pic_height rather than vdisplay for slice-height multiple
		//Works w/ vid_vs_delay = 9
		//display_color -v 2058 replaces overlap w/ black, so overlap is 502 input lines (753 display lines)...
		.dsc_vslice_height =	24,
		.v_dummy =				8,
		//.vid_vs_delay =			9,

		//.vbp_p =				8,
		//.vbp_m =				4,
		//.vid_vs_delay =			3,

		//.vbp_p =				17,
		//.vbp_m =				8,
		//.vid_vs_delay =			7,

		//Works w/ dummy moved to VFP
		.vbp_p =				17,
		.vbp_m =				8,
		.vid_vs_delay =			0,

		//Try original timings again...  nope.  Needs vid_vs_delay=1, and overlapped lines at top (away from DDIC) are back
		//.vbp_p =				8,
		//.vbp_m =				10,
		//.vid_vs_delay =			0,

		.dsc_hslice_count =		4,
		//.vid_vs_delay =			0,
		.h_dummy =				0,
		//.v_dummy =				0,
		.gscale =				GSCALE_1_5X,
		.fscale =				FSCALE_1X,
	},
	//1.5X global scaling; 60Hz; No DSC
	{
		.enabled =				GLOBAL_1_5X_NODSC_MODE_EN,
		.name =					"2368x2560_60",
		.fps =					60,
		.haa =					2368,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vaa =					2560,
//App note recommendation:
//Works w/ NO_DSC... not for DSC, but that might be due to other complications (like slice height)
		.vfp_p =				8,
		.vfp_m =				22,
		.vs =					1,
		.vbp_p =				8,
		.vbp_m =				10,
		//Doesn't work: dmesg: msm_dsi ae94000.dsi: [drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.vfp_p =				17,
		//.vfp_m =				21,
		//.vbp_p =				17,
		//.vbp_m =				8,
		//Doesn't work: dmesg: msm_dsi ae94000.dsi: [drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.vfp_p =				22,
		//.vfp_m =				26,
		//.vbp_p =				22,
		//.vbp_m =				12,
		.dsc_en =				false,
		//.dsc_vslice_height =	48,
		.dsc_vslice_height =	40,
		.dsc_hslice_count =		4,
		.vid_vs_delay =			0,
		.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_1_5X,
		.fscale =				FSCALE_1X,
	},
	//2.5X Fovial scale; 45Hz; DSC
	//Foveated and background size (per-panel): 1424x1536
	//TODO: WIP -- Not working yet
	{
		//.enabled =				FOVIAL_2_5X_MODE_EN,
		.enabled =				false, //TEMP TEST
		.name =					"1424x3072_45", //Name for actual image area, or image area w/ dummy lines?
		.fps =					45,
		.haa =					1424,
		.hfp =					20,
		//.hfp =					28,
		.hs =					8,
		.hbp =					28,
		.vaa =					1536*2,		//fovial + background
		.vfp_m =				24+768,	//768 extra for foveal image transport + buffer margin
		.vfp_p =				24,
		.vs =					1,
		.vbp_p =				23,		//"in FOV mode, vbp_p and vbp_m should be the same"
		.vbp_m =				23,
		.dsc_en =				true,
		//.dsc_vslice_height =	24,
		.dsc_vslice_height =	48,
		.dsc_hslice_count =		2,		//only 2, 4 supported
		.vid_vs_delay =			1536+23,	//vaa/2 + vbp + v_dummy/2
		.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_1X,
		.fscale =				FSCALE_2_5X,
	},
	//2.5X Fovial scale; 60Hz; DSC
	//Foveated and background size (per-panel): 1424x1536
	//TODO: WIP -- Not working yet
	{
		.enabled =				FOVIAL_2_5X_MODE_EN,
		//.enabled =				false, //TEMP TEST
		.name =					"1424x3072_60",	//Name for actual image area, or image area w/ dummy lines?
		.fps =					60,
		//.haa =					1424,
		.hfp =					20,
		//.hfp =					28,
		.hs =					8,
		.hbp =					28,
		.vaa =					1536*2,		//fovial + background
		//.vfp_m =				24+768,	//768 extra for foveal image transport + buffer margin
		//.vfp_p =				24,
		.vs =					1,
		//.vbp_p =				23,		//"in FOV mode, vbp_p and vbp_m should be the same"
		//.vbp_m =				23,

		.dsc_en =				true,
		//.dsc_vslice_height =	24,
		//.dsc_vslice_height =	48,
		//.dsc_hslice_count =		1,	//only 2, 4 supported
		//.vid_vs_delay =			1536+23,	//vaa/2 + vbp + v_dummy/2
		//.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_1X,
		.fscale =				FSCALE_2_5X,

		//[drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.vfp_m =				24+768,
		//.vfp_p =				24,
		//.vbp_p =				23,
		//.vbp_m =				23-9,
		//.vid_vs_delay =			1536+23,

		//[drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.vfp_m =				24+768+4,
		//.vfp_p =				24,
		//.vbp_p =				23,
		//.vbp_m =				23-9,
		//.vid_vs_delay =			1536+23,

		//[drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.vfp_m =				24+768+4,
		//.vfp_p =				24,
		//.vbp_p =				24,
		//.vbp_m =				23-9,
		//.vid_vs_delay =			1536+23,

		//[drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.vfp_m =				24+768+4,
		//.vfp_p =				24,
		//.vbp_p =				25,
		//.vbp_m =				23-9,
		//.vid_vs_delay =			1536+23,

		//[drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.vfp_m =				768,
		//.vfp_p =				24,
		//.vbp_p =				23,
		//.vbp_m =				23,
		//.vid_vs_delay =			1536+23,

		//[drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.vfp_m =				768,
		//.vfp_p =				17,
		//.vbp_p =				17,
		//.vbp_m =				8,
		//.vid_vs_delay =			1536+23,

		//[drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.vfp_m =				768,
		//.vfp_p =				17,
		//.vbp_p =				17,
		//.vbp_m =				8,
		//.vid_vs_delay =			1536,

		//[drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//.vfp_m =				21,
		//.vfp_p =				17,
		//.vbp_p =				17,
		//.vbp_m =				8,
		//.vid_vs_delay =			2,

		//[drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//w/ test mdoe: no obvious diff from default
		//.vfp_m =				768,
		//.vfp_p =				24,
		//.vbp_p =				23,
		//.vbp_m =				23,
		//.vid_vs_delay =			1536,

		//.vfp_m =				768,
		//.vfp_p =				24,
		//.vbp_p =				23,
		//.vbp_m =				23,
		//.vid_vs_delay =			1536,
		//.haa =					1421,
		//.h_dummy =				3,
		//.dsc_vslice_height =	24,
		//.dsc_hslice_count =		2,

		//[drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
		//  Also tried all 4 combos of width and xres remainder handling in dpu_encoder_phys_vid.c
		.vfp_m =				24+768,
		.vfp_p =				24,
		.vbp_p =				23,
		.vbp_m =				23,
		.vid_vs_delay =			1536,
		.haa =					1421,
		.h_dummy =				3,
		.dsc_vslice_height =	24,
		.dsc_hslice_count =		2,
	},
	//2.5X Fovial scale; 90Hz; DSC
	//Foveated and background size (per-panel): 1424x1536
	//TODO: WIP -- Not working yet
	{
		.enabled =				FOVIAL_2_5X_MODE_EN,
		//.enabled =				false, //TEMP TEST
		.name =					"1424x3072_90",	//Name for actual image area, or image area w/ dummy lines?
		.fps =					90,
		//.haa =					1424,
		//.hfp =					20,

		.hfp =					28,

		.hs =					8,
		.hbp =					28,
		.vaa =					1536*2,		//fovial + background
		//.vfp_m =				24+768,	//768 extra for foveal image transport + buffer margin
		//.vfp_p =				24,
		.vs =					1,
		//.vbp_p =				23,		//"in FOV mode, vbp_p and vbp_m should be the same"
		//.vbp_m =				23,

		.dsc_en =				true,
		//.dsc_vslice_height =	24,
		//.dsc_vslice_height =	48,
		//.dsc_hslice_count =		2,		//only 2, 4 supported
		//.vid_vs_delay =			1536+23,	//vaa/2 + vbp + v_dummy/2
		//.h_dummy =				0,
		.v_dummy =				0,
		.gscale =				GSCALE_1X,
		.fscale =				FSCALE_2_5X,

		//.vfp_m =				21+768,
		//.vfp_p =				17,
		//.vs =					1,
		//.vbp_p =				17,
		//.vbp_m =				8,

		//w/ test mode; extra overlayed rows at far-side (from DDIC)
		//.vfp_m =				21,
		//.vfp_p =				17,
		//.vbp_p =				17,
		//.vbp_m =				8,
		//.vid_vs_delay =			2,

		.vfp_m =				24+768,
		.vfp_p =				24,
		.vbp_p =				23,
		.vbp_m =				23,
		.vid_vs_delay =			1536+23,
		.haa =					1421,
		.h_dummy =				3,
		.dsc_vslice_height =	24,
		.dsc_hslice_count =		2,
	},
	//2.75X Fovial scale; 90Hz; DSC
	//Foveated and background size (per-panel): 1292x1397
	//TODO: WIP -- Not working yet
	{
		//.enabled =				FOVIAL_2_75X_MODE_EN,
		.enabled =				false,
		.name =					"f1297x1397_90",	//Name for actual image area, or image area w/ dummy lines?
		.fps =					90,
		//.haa =					1292,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vaa =					1397*2,		//fovial + background
		.vfp_m =				24+1008,	//1008 extra for foveal image transport + buffer margin
		.vfp_p =				24,
		.vs =					1,
		.vbp_p =				23,		//"in FOV mode, vbp_p and vbp_m should be the same"
		.vbp_m =				23,

		.dsc_en =				true,
		.dsc_vslice_height =	24,
		.dsc_hslice_count =		2,		//only 2, 4 supported
		//.vid_vs_delay =			1397+23+19,	//vaa/2 + vbp + v_dummy/2
		//.h_dummy =				0,
		.v_dummy =				19*2,
		.gscale =				GSCALE_1X,
		.fscale =				FSCALE_2_75X,

		//DSI errors
		//.haa =					1292,
		//.h_dummy =				0,

		//video timeout
		//.haa =					1292+1,	//+1 to be divisible by 3 for DSC compression
		//.h_dummy =				1,	//+3 to be divisible by 2 for hslice

		//video timeout
		//.haa =					1292+1,	//+1 to be divisible by 3 for DSC compression
		//.h_dummy =				1,	//+3 to be divisible by 2 for hslice
		//.vid_vs_delay =			1397+23+19+1,	//vaa/2 + vbp + v_dummy/2 + 1 extra

		//video timeout
		//.haa =					1292+1,	//+1 to be divisible by 3 for DSC compression
		//.h_dummy =				1,	//+3 to be divisible by 2 for hslice
		//.vid_vs_delay =			1397+23+19+2,	//vaa/2 + vbp + v_dummy/2 + 1 extra
	},
	//2.75X Fovial scale; 60Hz; DSC
	//Foveated and background size (per-panel): 1292x1397
	//TODO: WIP -- Not working yet
	{
		.enabled =				FOVIAL_2_75X_MODE_EN,
		.name =					"f1297x1397_60",	//Name for actual image area, or image area w/ dummy lines?
		.fps =					60,
		.haa =					1292,
		//.hfp =					20,
		//.hs =					8,
		//.hbp =					28,
		//.vaa =					1397*2,		//fovial + background
		//.vfp_m =				24+1008,	//1008 extra for foveal image transport + buffer margin
		//.vfp_p =				24,
		.vs =					1,
		//.vbp_p =				23,		//"in FOV mode, vbp_p and vbp_m should be the same"
		//.vbp_m =				23,

		.dsc_en =				true,
		.dsc_vslice_height =	24,
		.dsc_hslice_count =		2,		//only 2, 4 supported
		.vid_vs_delay =			1397+23+19,	//vaa/2 + vbp + v_dummy/2
		.h_dummy =				0,
		//.v_dummy =				19*2,
		.gscale =				GSCALE_1X,
		.fscale =				FSCALE_2_75X,

		//DSI errors
		//.vbp_p =				23,		//"in FOV mode, vbp_p and vbp_m should be the same"
		//.vbp_m =				23,
		//.hfp =					20,
		//.hbp =					28,

		//DSI errors
		//.vbp_p =				23,		//"in FOV mode, vbp_p and vbp_m should be the same"
		//.vbp_m =				23,
		//.hfp =					20+8,
		//.hbp =					28,

		//DSI errors
		//.vbp_p =				23,		//"in FOV mode, vbp_p and vbp_m should be the same"
		//.vbp_m =				23,
		//.hfp =					20,
		//.hbp =					28+8

		//DSI errors
		//.vbp_p =				23,		//"in FOV mode, vbp_p and vbp_m should be the same"
		//.vbp_m =				23,
		//.hfp =					20,
		//.hbp =					28,
		//.vfp_m =				24,

		//DSI errors
		//.vbp_p =				17,
		//.vbp_m =				8,
		//.hfp =					20,
		//.hbp =					28,
		//.vfp_m =				24,
		//.vfp_m =				24+1008,

		//DSI errors
		//.vbp_p =				23,
		//.vbp_m =				23,
		//.hfp =					20*2,
		//.hs =					8*2,
		//.hbp =					28*2,
		//.vfp_m =				24,
		//.vfp_m =				24+1008,

		.vaa =					1398*2,		//fovial + background
		.v_dummy =				18*2,
		.vbp_p =				23,
		.vbp_m =				23,
		.hfp =					20,
		.hs =					8,
		.hbp =					28,
		.vfp_p =				24,
		.vfp_m =				24+1008,
	},
	//3.25X Fovial scale; 90Hz; DSC; horiz test; with padding
	//Foveated and background size (per-panel): 1094x1182
	//TODO: WIP -- Not working yet
	{
		//.enabled =				FOVIAL_3_25X_MODE_EN,
		.enabled =				false,	//TEMP TEST
		.name =					"1094x2364+36_90_E",	//Name for actual image area, or image area w/ dummy lines?
		.fps =					90,
		//.haa =					1093,
		.haa =					1094,
		.hfp =					20,
		//.hfp =					28,
		.hs =					8,
		.hbp =					28,
		//.vaa =					(1182+18)*2,		//fovial + dummy + background + dummy
		//.vfp_p =				24+1440,	//1440 extra for foveal image transport + buffer margin
		//.vfp_m =				24+1440,
		.vaa =					1182*2,		//fovial + background image
		//.vfp_p =				24+1422,
		.vfp_m =				24+1440,
		.vfp_p =				24,
		//.vfp_m =				24,
		.vs =					1,
		//.vbp_p =				23,
		//.vbp_m =				23,
		.vbp_p =				17,
		.vbp_m =				8,
		.dsc_en =				true,
		.dsc_vslice_height =	24,
		.dsc_hslice_count =		2,		//only 2, 4 supported
		//.vid_vs_delay =			1182+23+18,	//vaa/2 + vbp + v_dummy
		.vid_vs_delay =			1182+17+18,	//vaa/2 + vbp + v_dummy
		//.vid_vs_delay =			1182+23,	//vaa/2 + vbp + v_dummy
		.h_dummy =				0,
		//.h_dummy =				1,
		.v_dummy =				18*2,
		//.v_dummy =				0,
		.gscale =				GSCALE_1X,
		.fscale =				FSCALE_3_25X,
	},
	//3.25X Fovial scale; 90Hz; DSC; with padding
	//Foveated and background size (per-panel): 1094x1182
	//Add padding to be multiple of DSC slice height: 1094x1200
	//TODO: WIP -- Not working yet
	{
		.enabled =				FOVIAL_3_25X_MODE_EN,
		//.name =					"1094x2364+36_90",	//Name for actual image area, or image area w/ dummy lines?
		.fps =					90,
		//.haa =					1094,
		.hfp =					20,
		//.hfp =					28,
		.hs =					8,
		.hbp =					28,
		//.vaa =					1182*2,		//fovial + background
		.vfp_m =				24+1440,	//1440 extra for foveal image transport + buffer margin
		.vfp_p =				24,
		.vs =					1,
		.vbp_p =				23,		//"in FOV mode, vbp_p and vbp_m should be the same"
		.vbp_m =				23,

		.name =					"1093+3x2364+36_90",	//Name for actual image area, or image area w/ dummy lines?
		.haa =					1093,
		.vaa =					1182*2,		//fovial + background
		.h_dummy =				3,
		.v_dummy =				6*2,	//Padding to insure integer multiple of DSC slice
		.vid_vs_delay =			1182+23+12,	//vaa/2 + vbp + v_dummy/2

		.dsc_en =				true,
		.dsc_vslice_height =	24,
		.dsc_hslice_count =		2,		//only 2, 4 supported
		//.vid_vs_delay =			1182+23+18,	//vaa/2 + vbp + v_dummy/2
		//.vid_vs_delay =			1182+23,	//vaa/2 + vbp
		//.h_dummy =				0,
		//.v_dummy =				18*2,	//Padding to insure integer multiple of DSC slice
										//"in FOV mode v_dummy/2 needs to be smaller than vfp_p to prevent tearing issue"
		//.v_dummy =				0,
		.gscale =				GSCALE_1X,
		.fscale =				FSCALE_3_25X,
	},
};

static int panel_read_show(struct seq_file *s, void *data);

#define plano_dsi_write_seq(ctx, cmd, seq...)					\
	do {										\
		u8 d[] = {cmd, seq};							\
		struct mipi_dsi_multi_context dsi_ctx0 = { .dsi = ctx->dsi[0] };	\
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx0, d, ARRAY_SIZE(d));		\
		if (ctx->is_dual_panel) {						\
			struct mipi_dsi_multi_context dsi_ctx1 = { .dsi = ctx->dsi[1] };\
			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx1, d, ARRAY_SIZE(d));	\
		}									\
	} while (0)

#define plano_dsi_write_buffer(ctx, d)						\
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

static int dsi_populate_dsc_params(struct plano_ctx *ctx)
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
	//dsc->line_buf_depth = dsc->bits_per_component + 2;	//Doesn't change reg value
	//gets overwritten by drm_dsc_compute_rc_parameters()
	//
	//hrd_delay = DIV_ROUND_UP((rbs_min * 16), vdsc_cfg->bits_per_pixel);
	//vdsc_cfg->rc_bits = (hrd_delay * vdsc_cfg->bits_per_pixel) / 16;
	//vdsc_cfg->initial_dec_delay = hrd_delay - vdsc_cfg->initial_xmit_delay;

	return drm_dsc_compute_rc_parameters(dsc);
	//ret = drm_dsc_compute_rc_parameters(dsc);
	//dsc->initial_dec_delay = 380;		//still didn't take... reg still reading back 0x1de
	//return ret;
}

static int plano_init_dsc_config(struct plano_ctx *ctx)
{
	ctx->dsc_cfg = (struct drm_dsc_config) {
		.dsc_version_major = 0x1,
		.dsc_version_minor = 0x1,
		.slice_height = extmodes[ctx->current_mode].dsc_vslice_height,
		//.slice_width = 444,
		//.slice_count = 4,
		//.slice_width = 888,	//Note: need to also change dsc_slice_per_pkt below
		//.slice_count = 2,	//Larger slide width results in initial_dec_delay = 2BC = 700, which satisfies requirement of <= slice_wdith - 64, but still displaying garbage
		//.slice_width = extmodes[ctx->current_mode].haa / extmodes[ctx->current_mode].dsc_hslice_count,
		.slice_width = ( extmodes[ctx->current_mode].haa + extmodes[ctx->current_mode].h_dummy ) / extmodes[ctx->current_mode].dsc_hslice_count,
		.slice_count = extmodes[ctx->current_mode].dsc_hslice_count,
		.bits_per_component = 8,
		.bits_per_pixel = 8 << 4,
		.block_pred_enable = true,

		//.line_buf_depth = 8,
		//.initial_dec_delay = 380,	//<= slice_with - 64 (444 - 64 = 380 (0x17C)) //can't set here... gets overwritten later

		//.pic_width = extmodes[ctx->current_mode].haa,
		.pic_width = extmodes[ctx->current_mode].haa + extmodes[ctx->current_mode].h_dummy,
		//.pic_height = extmodes[ctx->current_mode].vaa,
		.pic_height = extmodes[ctx->current_mode].vaa + extmodes[ctx->current_mode].v_dummy,
	};

	//TODO: get_modes getting called often, spamming dmesg... disable for now
	//DRM_DEV_INFO(ctx->dev, "Initialized dsc_cfg w/ extmode[%d] %s\n", ctx->current_mode, extmodes[ctx->current_mode].name);   

	return dsi_populate_dsc_params(ctx);
}

static int plano_set_dsi_dsc(struct plano_ctx *ctx, plano_modes_t mode_sel)
{

	if(extmodes[mode_sel].dsc_en)
	{

		int ret = plano_init_dsc_config(ctx);
		if (ret)
		{
			DRM_DEV_ERROR(ctx->dev, "Failed to change dsc config to mode set %s", extmodes[mode_sel].name);
			return -EINVAL;
		}
		ctx->dsi[0]->dsc = &ctx->dsc_cfg;
		ctx->dsi[0]->dsc_slice_per_pkt = extmodes[mode_sel].dsc_hslice_count;
		if (ctx->is_dual_panel) {
			ctx->dsi[1]->dsc = &ctx->dsc_cfg;
			ctx->dsi[1]->dsc_slice_per_pkt = extmodes[mode_sel].dsc_hslice_count;
		}
	}
	else
	{
		ctx->dsi[0]->dsc = NULL;
		ctx->dsi[0]->dsc_slice_per_pkt = 0;
		if (ctx->is_dual_panel) {
			ctx->dsi[1]->dsc = NULL;
			ctx->dsi[1]->dsc_slice_per_pkt = 0;
		}
	}

	return 0;
}


// should be called after get_panel_info
/*	Not needed for plano?
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
*/


static void dsi_write_plano_panel_dsc(struct plano_ctx *ctx)
{

	u8 pps_cmd[1 + sizeof(struct drm_dsc_picture_parameter_set)];

	if(extmodes[ctx->current_mode].dsc_en)
	{
		drm_dsc_pps_payload_pack((void *)&pps_cmd[1], &ctx->dsc_cfg);
		pps_cmd[0] = 0xF6;
		plano_dsi_write_buffer(ctx, pps_cmd); // dsc
		plano_dsi_write_seq(ctx, MIPI_DCS_GET_COMPRESSION_MODE, 0x11); // enable dsc compression
	}
	else
	{
		plano_dsi_write_seq(ctx, MIPI_DCS_GET_COMPRESSION_MODE, 0x10); // no compression
	}
	DRM_DEV_INFO(ctx->dev, "Updated panel DSC registers (DSC %s)\n", extmodes[ctx->current_mode].dsc_en ? "Enabled" : "Disabled" );
}

static void dsi_write_plano_panel_scaling(struct plano_ctx *ctx)
{
	u8 val;
	char fscale_s[5][8] = {{"1"},{"2.5"},{"2.75"},{"3.25"},{"4.25"}};

	if( extmodes[ctx->current_mode].gscale > 0 )
	{
		switch ( extmodes[ctx->current_mode].gscale )
		{
			case GSCALE_1_33X: 	val = 0x90;
				break;
			case GSCALE_1_5X: 	val = 0xA0;
				break;
			case GSCALE_2X: 	val = 0xB0;
				break;
			case GSCALE_1X:
			default: 			val = 0x00;	//No scaling
		}
	}
	else if ( extmodes[ctx->current_mode].fscale > 0 )
	{
		switch ( extmodes[ctx->current_mode].fscale )
		{
			case FSCALE_2_5X: 	val = 0x08;
				break;
			case FSCALE_2_75X:	val = 0x09;
				break;
			case FSCALE_3_25X:	val = 0x0A;
				break;
			case FSCALE_4_25X:	val = 0x0B;
				break;
			case FSCALE_1X:
			default:			val = 0x00;	//No scaling
		}
		DRM_DEV_INFO(ctx->dev, "Fovial scale: %s; Fovial and background image size: %d x %d\n", fscale_s[extmodes[ctx->current_mode].fscale], extmodes[ctx->current_mode].haa, extmodes[ctx->current_mode].vaa/2);   

		//For now, use register to position fovial region in center
		// in units of backplane pixels, not input pixels
		u16 x_pos, y_pos;
		x_pos = BACKPLANE_HAA / 2;
		y_pos = BACKPLANE_VAA / 2;
		plano_dsi_write_seq(ctx, 0x8a, 0x80 | (x_pos >> 8 & 0x0F), x_pos & 0xFF, y_pos >> 8 & 0x0F, y_pos & 0XFF);
		DRM_DEV_INFO(ctx->dev, "Set fov pos via reg 0x8A to 0x%03x, 0x%03x (%d, %d)\n", x_pos, y_pos, x_pos, y_pos);
	}
	else
	{
		val = 0x00;	//No scaling
	}
	plano_dsi_write_seq(ctx, 0x65, val);
	DRM_DEV_INFO(ctx->dev, "Wrote scaling reg to 0x%02x\n", val);
}

static void dsi_write_plano_panel_video_control(struct plano_ctx *ctx)
{
	uint8_t vbp_p[2], vfp_p[2], vid_vs_delay[2];

	vbp_p[1] = extmodes[ctx->current_mode].vbp_p >> 8 & 0xFF;
	vbp_p[0] = extmodes[ctx->current_mode].vbp_p >> 0 & 0xFF;

	vfp_p[1] = extmodes[ctx->current_mode].vfp_p >> 8 & 0xFF;
	vfp_p[0] = extmodes[ctx->current_mode].vfp_p >> 0 & 0xFF;

	vid_vs_delay[1] = extmodes[ctx->current_mode].vid_vs_delay >> 8 & 0xFF;
	vid_vs_delay[0] = extmodes[ctx->current_mode].vid_vs_delay >> 0 & 0xFF;

	switch ( extmodes[ctx->current_mode].fscale )
	{
		case FSCALE_2_5X:
			plano_dsi_write_seq(ctx, 0x3B, vbp_p[1], vbp_p[0], vfp_p[1], vfp_p[0], 0x00, 0x00, vid_vs_delay[1], vid_vs_delay[0], 0x05, 0x9F, 0x04, 0xC7, 0x03, 0xAF); // video mode
			break;
		case FSCALE_2_75X:
			plano_dsi_write_seq(ctx, 0x3B, vbp_p[1], vbp_p[0], vfp_p[1], vfp_p[0], 0x00, 0x00, 0x06, 0x17, vid_vs_delay[1], vid_vs_delay[0], 0x04, 0xC7, 0x03, 0xAF); // video mode
			break;
		case FSCALE_3_25X:
			plano_dsi_write_seq(ctx, 0x3B, vbp_p[1], vbp_p[0], vfp_p[1], vfp_p[0], 0x00, 0x00, 0x06, 0x17, 0x05, 0x9F, vid_vs_delay[1], vid_vs_delay[0], 0x03, 0xAF); // video mode
			break;
		case FSCALE_4_25X:
			plano_dsi_write_seq(ctx, 0x3B, vbp_p[1], vbp_p[0], vfp_p[1], vfp_p[0], 0x00, 0x00, 0x06, 0x17, 0x05, 0x9F, 0x04, 0xC7, vid_vs_delay[1], vid_vs_delay[0]); // video mode
			break;
		case FSCALE_1X:
		default:
			plano_dsi_write_seq(ctx, 0x3B, vbp_p[1], vbp_p[0], vfp_p[1], vfp_p[0], vid_vs_delay[1], vid_vs_delay[0], 0x06, 0x17, 0x05, 0x9F, 0x04, 0xC7, 0x03, 0xAF); // video mode
	}
}

static int dsi_write_plano_panel_setup(struct drm_panel *panel)
{
	struct plano_ctx *ctx = panel_to_ctx(panel);

    //dev_err(ctx->dev, "Running Plano init sequence");
	DRM_DEV_INFO(ctx->dev, "Configuring panel registers\n");

#ifdef CPHY_DEBUG
	//TODO: Is this right?  not working...
	//  Using &panel instead crashes kernel and keeps headset from booting
	struct seq_file s = {.private = panel};


	ctx->panel_read_id = 0;
	ctx->panel_read_reg_addr = 0x04;	//Display ID
	ctx->panel_read_num_bytes = 1;
	panel_read_show(&s, NULL);

	u8 panel_id_exp = 0xE3;	//TODO: if continue to check, move to ctx
	if(ctx->panel_read_data[0] != panel_id_exp)
	{
		DRM_DEV_ERROR(ctx->dev, "PanelID read of 0x%02X doesn't match expected 0x%02X \n", ctx->panel_read_data[0], panel_id_exp);
		return -EINVAL;
	}
#endif

	dsi_write_plano_panel_dsc(ctx);

#ifdef CPHY_DEBUG
	ctx->panel_read_id = 0;
	ctx->panel_read_reg_addr = 0x05;	//DSI error count
	ctx->panel_read_num_bytes = 1;
	panel_read_show(&s, NULL);

	ctx->panel_read_id = 0;
	ctx->panel_read_reg_addr = 0xF6;	//PPS
	ctx->panel_read_num_bytes = 94;
	panel_read_show(&s, NULL);
#endif

	dsi_write_plano_panel_video_control(ctx);

	plano_dsi_write_seq(ctx, 0x51, 0xFF); // full brightness
	plano_dsi_write_seq(ctx, 0x53, 0x20); // manual brightness, dimming off
	plano_dsi_write_seq(ctx, 0x26, 0x01); // default gamma set

	dsi_write_plano_panel_scaling(ctx);


	//  Moved to plano_enable -- need to start video stream before sleep-out
	/*
	plano_dsi_write_seq(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	//Wait at least three frames
	msleep(170);
	plano_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_ON);
	*/

	return 0;
}

static void dsi_write_panel_on(struct drm_panel *panel)
{
	struct plano_ctx *ctx = panel_to_ctx(panel);

	plano_dsi_write_seq(ctx, MIPI_DCS_EXIT_SLEEP_MODE);
	//Wait at least three frames
	//msleep(170);
	msleep(100);	//Good down to 30Hz
	plano_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_ON);
}

static void dsi_write_panel_off(struct drm_panel *panel)
{
	struct plano_ctx *ctx = panel_to_ctx(panel);

	plano_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	//plano_dsi_write_seq(ctx, MIPI_DCS_SET_TEAR_OFF);
	plano_dsi_write_seq(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(200);	//Wait at least 3 frames
}

//TODO: brightness changed from u16 -> u8... was there a reason for u16? does it matter?
//  Didn't fix the dmesg errors:
//    msm_dsi ae94000.dsi: [drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
//    dsi_cmds2buf_tx: cmd dma tx failed, type=0x15, data0=0x51, len=4, ret=-110
//  Is len=4 in nibbles, not bytes?
//  It's the DSI packet length, including header and footer
static int set_brightness(struct plano_ctx *ctx, u8 brightness,
			  u16 pulse_offset_rows)
{

	if (!ctx->prepared)
	{
		dev_err(ctx->dev, "Ignored attempt to set brightness to 0x%02X (%d) while panel unprepared\n", brightness, brightness);
		return -EINVAL;	//TODO: better return val?
	}

	//Does forcing LPM mode to write the brightness register help prevent these errors when timing is out of whack?
	//  msm_dsi ae94000.dsi: [drm:dsi_cmds2buf_tx] *ERROR* wait for video done timed out
	//  dsi_cmds2buf_tx: cmd dma tx failed, type=0x15, data0=0x51, len=4, ret=-110
	//  deckard-panel-plano ae94000.dsi.0: sending dcs data 51 00 failed: -110
	//No, doesn't seem to
	/* 
	u32 mode_flags[2];

	mode_flags[0] = ctx->dsi[0]->mode_flags;
	ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
	if (ctx->is_dual_panel)
	{
		mode_flags[1] = ctx->dsi[0]->mode_flags;
		ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;
	}

	*/

	//if (!ctx->dphy) {
		ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
		if (ctx->is_dual_panel)
			ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	//}

	plano_dsi_write_seq(ctx, 0x51, brightness);

	/*
	ctx->dsi[0]->mode_flags = mode_flags[0];
	if (ctx->is_dual_panel)
		ctx->dsi[1]->mode_flags = mode_flags[1];

	DRM_DEV_INFO(ctx->dev, "Brightness set to 0x%02X (%d) using LPM\n", brightness, brightness);
	*/

	//if (!ctx->dphy) {
		ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
		if (ctx->is_dual_panel)
			ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;
	//}
	DRM_DEV_INFO(ctx->dev, "Brightness set to 0x%02X (%d)\n", brightness, brightness);
	return 0;
}

static int plano_power_on(struct plano_ctx *ctx)
{
	int ret;

	DRM_DEV_INFO(ctx->dev, "Panel power on start\n");

    // VDDI 1.8V
	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->power_supplies),
				    ctx->power_supplies);
	if (ret < 0) {
		dev_err(ctx->dev, "Could not enable 1.8V\n");
		return ret;
	}

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
	
	//NOTE: boards currently wired to use reset_gpios->desc[0] for both displays
	gpiod_set_value(ctx->reset_gpios->desc[0], 1);
	gpiod_set_value(ctx->reset_gpios->desc[1], 1);

    usleep_range(10000, 20000);
	DRM_DEV_INFO(ctx->dev, "Panel power on end\n");

	// interrupt callback to turn on AVEE/ELVSS when TE gets set
	return 0;
}

static int plano_power_off(struct plano_ctx *ctx)
{
	int ret;

	DRM_DEV_INFO(ctx->dev, "Panel power off start\n");

	//NOTE: boards currently wired to use reset_gpios->desc[0] for both displays
	gpiod_set_value(ctx->reset_gpios->desc[0], 0);
	gpiod_set_value(ctx->reset_gpios->desc[1], 0);
	msleep(25);

	// turn off AVEE
	// should already been turned off by interrupt
	/*
	ret = regulator_disable(ctx->lcd_bias_regulators[LEFT_NEG].consumer);
	ret |= regulator_disable(ctx->lcd_bias_regulators[RIGHT_NEG].consumer);
	if ( ret ) {
		dev_err(ctx->dev, "Could not disable AVEE\n");
	}
	usleep_range(1000, 2000);
	*/
	// turn off AVDD
	ret = regulator_disable(ctx->lcd_bias_regulators[LEFT_POS].consumer);
	ret |= regulator_disable(ctx->lcd_bias_regulators[RIGHT_POS].consumer);
	if ( ret ) {
		dev_err(ctx->dev, "Could not disable AVDD\n");
	}

	msleep(20);
	DRM_DEV_INFO(ctx->dev, "Panel power off end\n");

	return regulator_bulk_disable(ARRAY_SIZE(ctx->power_supplies),
				      ctx->power_supplies);
}

static int plano_disable(struct drm_panel *panel)
{
	struct plano_ctx *ctx = panel_to_ctx(panel);

	DRM_DEV_INFO(ctx->dev, "Panel disable start\n");

	if (!ctx->enabled)
	{
		DRM_DEV_INFO(ctx->dev, "Panel disable abort -- already disabled\n");
		return 0;
	}

	ctx->dsi[0]->mode_flags &= ~MIPI_DSI_MODE_LPM;
	if (ctx->is_dual_panel)
		ctx->dsi[1]->mode_flags &= ~MIPI_DSI_MODE_LPM;

	dsi_write_panel_off(panel);

	ctx->enabled = false;
	DRM_DEV_INFO(ctx->dev, "Panel disable end\n");
	return 0;
}

static int plano_unprepare(struct drm_panel *panel)
{
	struct plano_ctx *ctx = panel_to_ctx(panel);
	int ret = 0;

	DRM_DEV_INFO(ctx->dev, "Panel unprepare start\n");

	if (!ctx->prepared)
	{
		DRM_DEV_INFO(ctx->dev, "Panel unprepare abort -- already unprepared\n");
		return 0;
	}


	//msleep(10);	//Wait at least 10ms
	msleep(100);	//Wait at least 3 frames. Good for down to 30fps
	ret = plano_power_off(ctx);
	if (ret < 0)
		DRM_DEV_ERROR(ctx->dev, "power_off failed ret = %d\n", ret);

	ctx->prepared = false;
	DRM_DEV_INFO(ctx->dev, "Panel unprepare end\n");
	return ret;
}

static int plano_prepare(struct drm_panel *panel)
{
	struct plano_ctx *ctx = panel_to_ctx(panel);
	int ret;

	DRM_DEV_INFO(ctx->dev, "Panel prepare start\n");

	if (ctx->prepared)
	{
		DRM_DEV_INFO(ctx->dev, "Panel prepare abort -- already prepared\n");
		return 0;
	}

	// clear out panel manufacturing info
	memset(ctx->panel_mfg_info, 0, ARRAY_SIZE(ctx->panel_mfg_info));


	//Moved here from plano_enable
	ret = plano_power_on(ctx);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "power_on failed\n");
		goto power_off;
	}

	ret = plano_set_dsi_dsc(ctx, ctx->current_mode);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "DSI DSC set failed\n");
		goto power_off;
	}

	/*
	if(extmodes[ctx->current_mode].dsc_en)
	{
		int ret = plano_init_dsc_config(ctx);
		if (ret)
		{
			DRM_DEV_ERROR(ctx->dev, "Failed to change dsc config to mode set %s", extmodes[ctx->current_mode].name);
			return -EINVAL;
		}
		ctx->dsi[0]->dsc = &ctx->dsc_cfg;
		ctx->dsi[0]->dsc_slice_per_pkt = extmodes[ctx->current_mode].dsc_hslice_count;
		//DRM_DEV_INFO(ctx->dev, "Wrote dsc_cfg to dsi[0] %s: pic_width = %d; pic_height = %d\n", ctx->dsi[0]->name, ctx->dsc_cfg->pic_width, ctx->dsc_cfg->pic_height);
		DRM_DEV_INFO(ctx->dev, "Wrote dsc_cfg to dsi[0] %s: pic_width = %d; pic_height = %d\n", ctx->dsi[0]->name, ctx->dsi[0]->dsc->pic_width, ctx->dsi[0]->dsc->pic_height);
		if (ctx->is_dual_panel) {
			ctx->dsi[1]->dsc = &ctx->dsc_cfg;
			ctx->dsi[1]->dsc_slice_per_pkt = extmodes[ctx->current_mode].dsc_hslice_count;
			//DRM_DEV_INFO(ctx->dev, "Wrote dsc_cfg to dsi[1] %s: pic_width = %d; pic_height = %d\n", ctx->dsi[1]->name, ctx->dsc_cfg->pic_width, ctx->dsc_cfg->pic_height);
			DRM_DEV_INFO(ctx->dev, "Wrote dsc_cfg to dsi[1] %s: pic_width = %d; pic_height = %d\n", ctx->dsi[1]->name, ctx->dsi[1]->dsc->pic_width, ctx->dsi[1]->dsc->pic_height);
		}
	}
	else
	{
		ctx->dsi[0]->dsc = NULL;
		ctx->dsi[0]->dsc_slice_per_pkt = 0;
		if (ctx->is_dual_panel) {
			ctx->dsi[1]->dsc = NULL;
			ctx->dsi[1]->dsc_slice_per_pkt = 0;
		}
	}
	*/

	ret = dsi_write_plano_panel_setup(panel);

	DRM_DEV_INFO(ctx->dev, "mode[%d] %s (%d x %d @ %dHz)\n", ctx->current_mode, extmodes[ctx->current_mode].name, extmodes[ctx->current_mode].haa, extmodes[ctx->current_mode].vaa, extmodes[ctx->current_mode].fps); 
	DRM_DEV_INFO(ctx->dev, "haa: %d; hfp: %d; hs: %d; hbp: %d; hdummy: %d; hdisplay: %d; htot: %d\n", extmodes[ctx->current_mode].haa, extmodes[ctx->current_mode].hfp, extmodes[ctx->current_mode].hs, extmodes[ctx->current_mode].hbp, extmodes[ctx->current_mode].h_dummy, extmodes[ctx->current_mode].mode.hdisplay, extmodes[ctx->current_mode].mode.htotal);
	DRM_DEV_INFO(ctx->dev, "vaa: %d; vfp: %d; vs: %d; vbp: %d; vdummy: %d; vdisplay: %d; vtot: %d\n", extmodes[ctx->current_mode].vaa, extmodes[ctx->current_mode].vfp_m, extmodes[ctx->current_mode].vs, extmodes[ctx->current_mode].vbp_m, extmodes[ctx->current_mode].v_dummy, extmodes[ctx->current_mode].mode.vdisplay, extmodes[ctx->current_mode].mode.vtotal);

	DRM_DEV_INFO(ctx->dev, "hdisplay: %d; hsync_start: %d; hsync_end: %d; htotal: %d\n", extmodes[ctx->current_mode].mode.hdisplay, extmodes[ctx->current_mode].mode.hsync_start, extmodes[ctx->current_mode].mode.hsync_end, extmodes[ctx->current_mode].mode.htotal);
	DRM_DEV_INFO(ctx->dev, "vdisplay: %d; vsync_start: %d; vsync_end: %d; vtotal: %d; clock: %d\n", extmodes[ctx->current_mode].mode.vdisplay, extmodes[ctx->current_mode].mode.vsync_start, extmodes[ctx->current_mode].mode.vsync_end, extmodes[ctx->current_mode].mode.vtotal, extmodes[ctx->current_mode].mode.clock);

	DRM_DEV_INFO(ctx->dev, "dsc_en: %s; dsc_slice_height: %d; dsc_slice_count: %d\n", extmodes[ctx->current_mode].dsc_en ? "true" : "false", extmodes[ctx->current_mode].dsc_vslice_height, extmodes[ctx->current_mode].dsc_hslice_count);
	DRM_DEV_INFO(ctx->dev, "Panel vfp: %d; vbp: %d; vid_vs_delay: %d\n", extmodes[ctx->current_mode].vfp_p, extmodes[ctx->current_mode].vbp_p, extmodes[ctx->current_mode].vid_vs_delay);




	ctx->prepared = true;
	DRM_DEV_INFO(ctx->dev, "Panel prepare end\n");

	if(ret != 0)
	{
		DRM_DEV_ERROR(ctx->dev, "Panel setup failed\n");
		//TODO: temp debug -- bail w/ panel on to allow for additional register transactions
#ifndef CPHY_DEBUG
		goto power_off;
#endif
		return ret;
	}

	return 0;

power_off:
	if (plano_power_off(ctx))
		DRM_DEV_ERROR(ctx->dev, "power_off failed\n");
	return ret;
}

static int plano_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int plano_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct plano_ctx *plano_ctx = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if (!plano_ctx->prepared)
		return 0;

	ret = set_brightness(plano_ctx, bl->props.brightness,
			     bl->props.pulse_offset_rows);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops plano_bl_ops = {
	.update_status = plano_bl_update_status,
	.get_brightness = plano_bl_get_brightness,
};

static int plano_enable(struct drm_panel *panel)
{
	struct plano_ctx *ctx = panel_to_ctx(panel);

	dsi_write_panel_on(panel);

	ctx->enabled = true;
	DRM_DEV_INFO(ctx->dev, "Panel enabled\n");

	return 0;
}

static plano_modes_t plano_init_modes(struct plano_ctx *ctx)
{
	bool default_set = false;
	plano_modes_t default_mode = 0;

	for (int i = 0; i < ARRAY_SIZE(extmodes); i++)
	{
		//Choose first enabled mode as default
		if(!default_set && extmodes[i].enabled)
		{
			default_mode = i;
			default_set = true;
		}

		strncpy(extmodes[i].mode.name, extmodes[i].name, DRM_DISPLAY_MODE_LEN);

		extmodes[i].mode.hdisplay = extmodes[i].haa;
		//extmodes[i].mode.hdisplay = extmodes[i].haa + extmodes[i].h_dummy;
		//extmodes[i].mode.hsync_start = extmodes[i].mode.hdisplay + extmodes[i].hfp;
		extmodes[i].mode.hsync_start = extmodes[i].mode.hdisplay + extmodes[i].hfp + extmodes[i].h_dummy;	//TODO: Verify dummy should be added to HFP
		extmodes[i].mode.hsync_end = extmodes[i].mode.hsync_start + extmodes[i].hs;
		extmodes[i].mode.htotal = extmodes[i].mode.hsync_end + extmodes[i].hbp;
		//extmodes[i].mode.htotal = extmodes[i].mode.hsync_end + extmodes[i].hbp + extmodes[i].h_dummy;

		extmodes[i].mode.vdisplay = extmodes[i].vaa;
		//extmodes[i].mode.vdisplay = extmodes[i].vaa + extmodes[i].v_dummy;	//TODO: add dummy here or total?	//If don't add to vdisplay, then get "DSI: pic_height 2560 has to be multiple of slice 48" type errors?
		//extmodes[i].mode.vsync_start = extmodes[i].mode.vdisplay + extmodes[i].vfp_m;
		extmodes[i].mode.vsync_start = extmodes[i].mode.vdisplay + extmodes[i].vfp_m + extmodes[i].v_dummy;		//Adding dummy to FP seems to work for g1.5x scaling
		extmodes[i].mode.vsync_end = extmodes[i].mode.vsync_start + extmodes[i].vs;
		extmodes[i].mode.vtotal = extmodes[i].mode.vsync_end + extmodes[i].vbp_m;
		//extmodes[i].mode.vtotal = extmodes[i].mode.vsync_end + extmodes[i].vbp_m + extmodes[i].v_dummy;
		//extmodes[i].mode.clock =  (extmodes[i].haa + 20 + 8 + 28) * (extmodes[i].vaa + extmodes[i].vfp_m + 1 + extmodes[i].vbp_m) * extmodes[i].fps / 1000;
		extmodes[i].mode.clock =  extmodes[i].mode.htotal * extmodes[i].mode.vtotal * extmodes[i].fps / 1000;


		extmodes[i].mode.width_mm = 50;
		extmodes[i].mode.height_mm = 50;
	
	}

	if(!default_set)
	{
		DRM_DEV_ERROR(ctx->dev, "Error initializing modes: No modes enabled");
		return -EINVAL;
	}

	return default_mode;
}

static int plano_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	struct plano_ctx *ctx = panel_to_ctx(panel);
	struct drm_display_mode *mode;
	int i, n = 0;
	int ret;

	plano_modes_t current_mode = ctx->current_mode;

	for (i = 0; i < ARRAY_SIZE(extmodes); i++) {

		ctx->current_mode = i;
		if(extmodes[i].enabled)
		{
			mode = drm_mode_create(connector->dev);
			if (!mode) {
				DRM_DEV_ERROR(ctx->dev,
					"failed to create a new display mode\n");
				return i;
			}

			drm_mode_copy(mode, &extmodes[i].mode);
			mode->type = DRM_MODE_TYPE_DRIVER;
			if (n == 0)
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

			//TODO: modes rejected if aren't compatible w/ DSC settings... using default mode dsc settings though?
			//TODO: Ok to change dsc config here?  Need to support multiple dsc configs in ctx?
			//	TODO: Supporting different DSC configs needs more work, if even possible
			//get_modes doesn't appear to be called while a mode is active, so should be ok to change dsc config in order to add mode?
			//but, mode validity not checked until later, so ctx may have changed?

			ret = plano_set_dsi_dsc(ctx, i);
			if (ret < 0) {
				DRM_DEV_ERROR(ctx->dev, "DSI DSC set failed\n");
			}

			/*
			if(extmodes[i].dsc_en)
			{

				int ret = plano_init_dsc_config(ctx);
				if (ret)
				{
					DRM_DEV_ERROR(ctx->dev, "Failed to change dsc config to mode set %s", mode->name);
					return -EINVAL;
				}
				ctx->dsi[0]->dsc = &ctx->dsc_cfg;
				ctx->dsi[0]->dsc_slice_per_pkt = extmodes[i].dsc_hslice_count;
				if (ctx->is_dual_panel) {
					ctx->dsi[1]->dsc = &ctx->dsc_cfg;
					ctx->dsi[1]->dsc_slice_per_pkt = extmodes[i].dsc_hslice_count;
				}
			}
			else
			{
				ctx->dsi[0]->dsc = NULL;
				ctx->dsi[0]->dsc_slice_per_pkt = 0;
				if (ctx->is_dual_panel) {
					ctx->dsi[1]->dsc = NULL;
					ctx->dsi[1]->dsc_slice_per_pkt = 0;
				}
			}
			*/

			drm_mode_probed_add(connector, mode);
			//TODO: This is getting called many times and periodically?
			//  disabled for now... too spammy
			//DRM_DEV_INFO(ctx->dev, "Added mode[%d] %s (extmode[%d] %s): %d x %d @ %dHz; dsc: %s \n", n, mode->name, i, extmodes[i].mode.name, mode->hdisplay, mode->vdisplay, extmodes[i].fps, extmodes[i].dsc_en ? "on" : "off");
			n++;
		}
	}

	ctx->current_mode = current_mode;

	//Restore dsi config to current_mode
	ret = plano_set_dsi_dsc(ctx, current_mode);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "DSI DSC restore failed\n");
	}

	return n;
}

static int plano_mode_set(struct drm_panel *panel,
			       const struct drm_display_mode *mode,
			       const struct drm_display_mode *adjusted_mode)
{
	struct plano_ctx *ctx = panel_to_ctx(panel);
	plano_modes_t new_mode;
	int i;
	char mod_mode_name[DRM_DISPLAY_MODE_LEN+1];
	int ret;

	//for(i = 0; i < MODE_CNT; i++)
	for(i = 0; i < ARRAY_SIZE(extmodes); i++)
	{
		if(extmodes[i].enabled)
		{
			strncpy(mod_mode_name, extmodes[i].mode.name, DRM_DISPLAY_MODE_LEN);
			if (ctx->is_dual_panel) {
				mod_mode_name[0] = '2';
				mod_mode_name[1] = '*';
			}
			//DRM_DEV_INFO(ctx->dev, "plano_mode_set: trying to match %s to extmode[%d] (%s -> %s)\n", mode->name, i, extmodes[i].mode.name, mod_mode_name);
			//Matching by name doesn't work because name is modified when expanded for dual panels
			//Start matching at 3rd char, as first 2 chars are munged
			//if (strcmp(mode->name, extmodes[i].mode.name)){
			if (strcmp(mode->name, mod_mode_name) == 0){
				new_mode = i;
				break;
			}
		}
	}

	DRM_DEV_INFO(ctx->dev, "current extmode[%d]: %s; requested mode: %s; new extmode[%d]: %s\n", ctx->current_mode, extmodes[ctx->current_mode].mode.name, mode->name, new_mode, extmodes[new_mode].mode.name);


	//if(i >= MODE_CNT)
	if(i >= ARRAY_SIZE(extmodes))
	{
		DRM_DEV_ERROR(ctx->dev, "Invalid name in mode set %s", mode->name);
		return -EINVAL;
	}

	/*
	if (strcmp(mode->name, extmodes[MODE_G2X_90].mode.name)){
			new_mode = MODE_G2X_90;
	}else if (strcmp(mode->name, extmodes[MODE_G2X_60].mode.name)){
			new_mode = MODE_G2X_60;
	}else if (strcmp(mode->name, extmodes[MODE_G1_5X_90].mode.name)){
			new_mode = MODE_G1_5X_90;
	}else if (strcmp(mode->name, extmodes[MODE_G1_5X_60].mode.name)){
			new_mode = MODE_G1_5X_60;
	}else if (strcmp(mode->name, extmodes[MODE_F3_25X_60].mode.name)){
			new_mode = MODE_F3_25X_60;
	}else {
			DRM_DEV_ERROR(ctx->dev, "Invalid name in mode set %s", mode->name);
			return -EINVAL;
	}
	*/

	// check if we need to update settings
	if (ctx->current_mode != new_mode)
	{
		DRM_DEV_INFO(ctx->dev, "Changed from extmode[%d] (%s) to extmode[%d] (%s)\n", ctx->current_mode, extmodes[ctx->current_mode].mode.name, new_mode, extmodes[new_mode].mode.name);
		ctx->current_mode = new_mode;
		// only reset the panel parematers if we already initialized the panels
		//if (ctx->enabled)
		if (ctx->prepared)
		{
			DRM_DEV_INFO(ctx->dev, "Changing modes while panels already prepared\n");
			plano_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_OFF);
			msleep(200);	//TODO: Needed?
			//	TODO: Supporting different DSC configs needs more work, if even possible

			ret = plano_set_dsi_dsc(ctx, ctx->current_mode);
			if (ret < 0) {
				DRM_DEV_ERROR(ctx->dev, "DSI DSC set failed\n");
			}

			/*
			if(extmodes[ctx->current_mode].dsc_en)
			{
				int ret = plano_init_dsc_config(ctx);
				if (ret)
				{
					DRM_DEV_ERROR(ctx->dev, "Failed to change dsc config to mode set %s", mode->name);
					return -EINVAL;
				}
				ctx->dsi[0]->dsc = &ctx->dsc_cfg;
				ctx->dsi[0]->dsc_slice_per_pkt = extmodes[ctx->current_mode].dsc_hslice_count;
				if (ctx->is_dual_panel) {
					ctx->dsi[1]->dsc = &ctx->dsc_cfg;
					ctx->dsi[1]->dsc_slice_per_pkt = extmodes[ctx->current_mode].dsc_hslice_count;
				}
			}
			else
			{
				ctx->dsi[0]->dsc = NULL;
				ctx->dsi[0]->dsc_slice_per_pkt = 0;
				if (ctx->is_dual_panel) {
					ctx->dsi[1]->dsc = NULL;
					ctx->dsi[1]->dsc_slice_per_pkt = 0;
				}
			}
			*/

			//Update panel settings
			dsi_write_plano_panel_dsc(ctx);
			dsi_write_plano_panel_scaling(ctx);
			dsi_write_plano_panel_video_control(ctx);
			msleep(200);	//TODO: Needed?
			plano_dsi_write_seq(ctx, MIPI_DCS_SET_DISPLAY_ON);
			//TODO: Finish
			// update vcom
			//update_vcom(ctx);
			// update brightness
			set_brightness(ctx, ctx->backlight->props.brightness, ctx->backlight->props.pulse_offset_rows);
		}
		else
		{
			DRM_DEV_INFO(ctx->dev, "Panels not prepared yet... will change modes upon prepare \n");
		}
	}

	return 0;
}

//TODO: update for plano
static int panel_info_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct plano_ctx *ctx = panel_to_ctx(panel);
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

//TODO: update for plano
static int panel_mfg_info_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct plano_ctx *ctx = panel_to_ctx(panel);
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
	struct plano_ctx *ctx = panel_to_ctx(panel);

	/*	Want to use during panel prep, so can't rely on panel already being prepared
	if (!ctx->prepared)
	{
		DRM_DEV_ERROR(ctx->dev, "Panel not prepared... skipping read of panel %d reg 0x%02X\n", ctx->panel_read_id, ctx->panel_read_reg_addr);
		return -EINVAL;	//TODO: better return val?
	}
	*/

	//DRM_DEV_INFO(ctx->dev, "Attempting to read %d bytes from panel %d reg 0x%02X into buf at %d\n", ctx->panel_read_num_bytes, ctx->panel_read_id, ctx->panel_read_reg_addr, &(ctx->panel_read_data[0]));
	DRM_DEV_INFO(ctx->dev, "Attempting to read %d bytes from panel %d reg 0x%02X\n", ctx->panel_read_num_bytes, ctx->panel_read_id, ctx->panel_read_reg_addr);
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
			//TODO: where does seq_printf output?  Need to enable??
			//seq_printf(s, "%02x ",  ctx->panel_read_data[i]);
			DRM_DEV_INFO(ctx->dev, "%02x ",  ctx->panel_read_data[i]);
		}
		//seq_printf(s, "\n");
		DRM_DEV_INFO(ctx->dev, "\n");
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
	struct plano_ctx *ctx = panel_to_ctx(panel);
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
	struct plano_ctx *ctx = panel_to_ctx(panel);

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
		DRM_DEV_ERROR(ctx->dev, "panel write exceed max length %d\n", len);
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
	struct plano_ctx *ctx = panel_to_ctx(panel);
	seq_printf(s, "%d\n",  ctx->enabled );
	return 0;
}

// RCLK = 32.5 MHz
#define RCLK_PERIOD_NSEC ( 1000000/32500 )
#define INTERNAL_LINE_TIME_NSEC  ( RTN * RCLK_PERIOD_NSEC )
#define EXTERNAL_LINE_TIME_NSEC ( ( 1000000 * (uint32_t)extmodes[ctx->current_mode].mode.htotal ) / ( (uint32_t)extmodes[ctx->current_mode].mode.clock ) )

static int panel_vsync_to_photons_usec_show(struct seq_file *s, void *data)
{
	struct drm_panel *panel = s->private;
	struct plano_ctx *ctx = panel_to_ctx(panel);
	uint32_t vsync_to_photons_nsec, gpo1_tes1, frame_period_nsec;

	// set offset based on frame rate to optimize settling time vs latency
	frame_period_nsec = 1000000000 / extmodes[ctx->current_mode].fps;
	// 1/FPS + TES
	//TODO: revisit... for now, just zero out
	//vsync_to_photons_nsec = VID_VS_DELAY * EXTERNAL_LINE_TIME_NSEC + frame_period_nsec + gpo1_tes1 * INTERNAL_LINE_TIME_NSEC;
	vsync_to_photons_nsec = 0;

	seq_printf(s, "%u\n", vsync_to_photons_nsec / 1000);
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(panel_info);
DEFINE_SHOW_ATTRIBUTE(panel_mfg_info);
DEFINE_SHOW_ATTRIBUTE(panel_available);
DEFINE_SHOW_ATTRIBUTE(panel_vsync_to_photons_usec);
DEFINE_SHOW_STORE_ATTRIBUTE(panel_read);
DEFINE_SHOW_STORE_ATTRIBUTE(panel_write);

static void plano_debugfs_init(struct drm_panel *panel, struct dentry *root)
{
	debugfs_create_file("panel_info", 0600, root, panel, &panel_info_fops);
	debugfs_create_file("panel_mfg_info", 0600, root, panel, &panel_mfg_info_fops);
	debugfs_create_file("panel_available", 0600, root, panel, &panel_available_fops);
	debugfs_create_file("panel_vsync_to_photons_usec", 0600, root, panel, &panel_vsync_to_photons_usec_fops);
	debugfs_create_file("panel_write", 0600, root, panel, &panel_write_fops);
	debugfs_create_file("panel_read", 0600, root, panel, &panel_read_fops);
}

static const struct drm_panel_funcs plano_drm_funcs = {
	.disable = plano_disable,
	.enable = plano_enable,
	.unprepare = plano_unprepare,
	.prepare = plano_prepare,
	.get_modes = plano_get_modes,
    .mode_set = plano_mode_set,
	.debugfs_init = plano_debugfs_init,
};

static irqreturn_t te_irq_handler_top(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t te_left_irq_handler(int irq, void *dev_id)
{
	int ret = 0;
	struct plano_ctx *ctx = dev_id;

	if(gpiod_get_value(ctx->te_gpios->desc[0]))
	{
		// turn on AVEE/ELVSS
		ret = regulator_enable(ctx->lcd_bias_regulators[LEFT_NEG].consumer);
		if ( ret ) {
			dev_err(ctx->dev, "Could not enable left AVEE/ELVSS \n");
			return ret;
		}

		// make sure we set the bias voltage here
		ret = regulator_set_voltage(ctx->lcd_bias_regulators[LEFT_NEG].consumer, 6300000, 6300000);

		if ( ret ) {
			dev_err(ctx->dev, "Could not set left AVEE\n");
			return ret;
		}

		DRM_DEV_INFO(ctx->dev, "Left ELVSS enabled\n");
	}
	else
	{
		// turn off AVEE/ELVSS 
		ret = regulator_disable(ctx->lcd_bias_regulators[LEFT_NEG].consumer);
		if ( ret ) {
			dev_err(ctx->dev, "Could not disable left AVEE\n");
			return ret;
		}

		DRM_DEV_INFO(ctx->dev, "Left ELVSS disabled\n");
	}


	return IRQ_HANDLED;
}

static irqreturn_t te_right_irq_handler(int irq, void *dev_id)
{
	int ret = 0;
	struct plano_ctx *ctx = dev_id;

	if(gpiod_get_value(ctx->te_gpios->desc[1]))
	{
		// turn on AVEE/ELVSS
		ret = regulator_enable(ctx->lcd_bias_regulators[RIGHT_NEG].consumer);
		if ( ret ) {
			dev_err(ctx->dev, "Could not enable right AVEE/ELVSS \n");
			return ret;
		}

		// make sure we set the bias voltage here
		ret = regulator_set_voltage(ctx->lcd_bias_regulators[RIGHT_NEG].consumer, 6300000, 6300000);

		if ( ret ) {
			dev_err(ctx->dev, "Could not set right AVEE\n");
			return ret;
		}

		DRM_DEV_INFO(ctx->dev, "Right ELVSS enabled\n");
	}
	else
	{
		// turn off AVEE/ELVSS 
		ret = regulator_disable(ctx->lcd_bias_regulators[RIGHT_NEG].consumer);
		if ( ret ) {
			dev_err(ctx->dev, "Could not disable right AVEE\n");
			return ret;
		}

		DRM_DEV_INFO(ctx->dev, "Right ELVSS disabled\n");
	}

	return IRQ_HANDLED;
}

static int plano_panel_add(struct plano_ctx *ctx)
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
		te_left_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
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
		te_right_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"TE_RIGHT", ctx);

	if (ret) {
		DRM_DEV_ERROR(dev,
			"request te right alert interrupt failed with %d\n",
			ret);
		return ret;
	}

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 0x00;
	bl_props.max_brightness = 0xFF;

	//TODO: refactor for plano?
	ctx->backlight = devm_backlight_device_register(dev, dev_name(dev),
							dev, ctx->dsi[0],
							&plano_bl_ops,
							&bl_props);
	if (IS_ERR(ctx->backlight)) {
		ret = PTR_ERR(ctx->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_init(&ctx->panel, dev, &plano_drm_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	drm_panel_add(&ctx->panel);

	DRM_DEV_INFO(ctx->dev, "Panel added\n");

	return 0;
}

static int plano_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct plano_ctx *ctx;
	struct mipi_dsi_device *dsi1_device;
	struct device_node *dsi1, *dsi0;
	struct mipi_dsi_host *dsi1_host;
	struct mipi_dsi_device *dsi_dev;
	struct device_node *dpu1, *dpu0;
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

	ctx->current_mode = plano_init_modes(ctx);

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

	dpu1 = of_graph_get_remote_node(dsi1, 0, -1);
	if (!dpu1) {
		DRM_DEV_ERROR(dev, "failed to get secondary dpu\n");
		return -ENODEV;
	}

	DRM_DEV_INFO(ctx->dev, "dsi[1] name: %s; remote: %s; dpu[1]: %s\n", ctx->dsi[1]->name, dsi1->full_name, dpu1->full_name);

skip_panel2:
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	ctx->dsi[0] = dsi;

	dsi0 = of_graph_get_remote_node(dsi->dev.of_node, 0, -1);
	if (!dsi0) {
		DRM_DEV_ERROR(dev, "failed to get primary dsi\n");
		return -ENODEV;
	}

	dpu0 = of_graph_get_remote_node(dsi0, 0, -1);
	if (!dpu0) {
		DRM_DEV_ERROR(dev, "failed to get primary dpu\n");
		return -ENODEV;
	}

	DRM_DEV_INFO(ctx->dev, "dsi[0] name: %s; remote: %s; dpu[0]: %s\n", ctx->dsi[0]->name, dsi0->full_name, dpu0->full_name);

	//TODO: config DSC based on mode?  probe done prior to mode selection...
	//Can DSC be configured later?  Or need to re-probe if change DSC?

	ret = plano_set_dsi_dsc(ctx, ctx->current_mode);
	if (ret < 0) {
		DRM_DEV_ERROR(ctx->dev, "DSI DSC set failed\n");
	}

	/*
	//if(extmodes[ctx->current_mode].dsc_en)
	if(extmodes[ctx->current_mode].dsc_en)
	{
		ret = plano_init_dsc_config(ctx);
		if (ret)
			goto err_panel_add;

		ctx->dsi[0]->dsc = &ctx->dsc_cfg;
		ctx->dsi[0]->dsc_slice_per_pkt = extmodes[ctx->current_mode].dsc_hslice_count;
		if (ctx->is_dual_panel) {
			ctx->dsi[1]->dsc = &ctx->dsc_cfg;
			ctx->dsi[1]->dsc_slice_per_pkt = extmodes[ctx->current_mode].dsc_hslice_count;
		}
	}
	else
	{
		ctx->dsi[0]->dsc = NULL;
		ctx->dsi[0]->dsc_slice_per_pkt = 0;
		if (ctx->is_dual_panel) {
			ctx->dsi[1]->dsc = NULL;
			ctx->dsi[1]->dsc_slice_per_pkt = 0;
		}
	}
	*/

	//ctx->low_fps_support = of_property_read_bool(dev->of_node, "low-fps-support");

	ret = plano_panel_add(ctx);
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

	DRM_DEV_INFO(ctx->dev, "Probed as %s w/ extmode[%d] %s (%d x %d @ %dHz\n", ctx->dphy ? "DPHY" : "CPHY", ctx->current_mode, extmodes[ctx->current_mode].name, extmodes[ctx->current_mode].mode.hdisplay, extmodes[ctx->current_mode].mode.vdisplay, extmodes[ctx->current_mode].fps);   

#ifdef CPHY_DEBUG
	//TODO: TEMP TEST -- added to allow for panel reg access w/o also starting video
	DRM_DEV_ERROR(ctx->dev, "TEMP DEBUG: Preparing panel for immediate register read/write for debug\n");
	plano_prepare(&ctx->panel);
#endif

	return 0;

err_dsi_attach:
	drm_panel_remove(&ctx->panel);
err_panel_add:
	mipi_dsi_device_unregister(dsi1_device);
	return -EPROBE_DEFER;;
}

static void plano_remove(struct mipi_dsi_device *dsi)
{
	struct plano_ctx *ctx = mipi_dsi_get_drvdata(dsi);

	if (ctx->dsi[0])
		mipi_dsi_detach(ctx->dsi[0]);
	if (ctx->dsi[1]) {
		mipi_dsi_detach(ctx->dsi[1]);
		mipi_dsi_device_unregister(ctx->dsi[1]);
	}

	drm_panel_remove(&ctx->panel);

	DRM_DEV_INFO(ctx->dev, "Panel removed\n");

}

static const struct of_device_id plano_of_match[] = {
	{ .compatible = "deckard-panel-plano" },
	{ }
};
MODULE_DEVICE_TABLE(of, plano_of_match);

static struct mipi_dsi_driver plano_driver = {
	.driver = {
		.name = "deckard-panel-plano",
		.of_match_table = plano_of_match,
	},
	.probe = plano_probe,
	.remove = plano_remove,
};
module_mipi_dsi_driver(plano_driver);

MODULE_DESCRIPTION("Plano DSI Panel Driver");
MODULE_LICENSE("GPL v2");
