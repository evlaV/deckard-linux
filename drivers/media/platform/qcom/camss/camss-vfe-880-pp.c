// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm MSM Camera Subsystem - VFE 880 (SM8650) pixel pipe setup
 *
 * Static pixel pipe configuration for NV12 output, one function per
 * block. Values derived from floating-point parameters are kept as their
 * fixed-point register values; geometry comes from the active format.
 * Tuning (WB, CC, gamma, CST) is static until the params buffer takes it
 * over.
 */

#include <linux/io.h>
#include <linux/kernel.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-880-pp.h"

/* Register offsets, named as in the IFE register map */
#define IFE_TOP_CORE_CFG_0                                       0x00024
#define IFE_WB_GAIN_MODULE_CFG                                   0x04560
#define IFE_WB_GAIN_WB_CFG_0                                     0x04568
#define IFE_WB_GAIN_WB_CFG_1                                     0x0456c
#define IFE_WB_GAIN_WB_CFG_2                                     0x04570
#define IFE_WB_GAIN_WB_OFFSET_CFG_0                              0x04574
#define IFE_WB_GAIN_WB_OFFSET_CFG_1                              0x04578
#define IFE_WB_GAIN_WB_OFFSET_CFG_2                              0x0457c
#define IFE_WB_GAIN_WB_OFFSET_CFG_3                              0x04580
#define IFE_WB_GAIN_WB_OFFSET_CFG_4                              0x04584
#define IFE_WB_GAIN_WB_OFFSET_CFG_5                              0x04588
#define IFE_DEMOSAIC_MODULE_CFG                                  0x05660
#define IFE_DEMOSAIC_INTERP_COEFF_CFG_0                          0x05668
#define IFE_DEMOSAIC_INTERP_COEFF_CFG_1                          0x0566c
#define IFE_DEMOSAIC_INTERP_CLASSIFIER_CFG_0                     0x05670
#define IFE_DEMOSAIC_INTERP_CLASSIFIER_CFG_1                     0x05674
#define IFE_COLOR_CORRECT_MODULE_CFG                             0x05860
#define IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_A_CFG_0            0x05868
#define IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_A_CFG_1            0x0586c
#define IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_B_CFG_0            0x05870
#define IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_B_CFG_1            0x05874
#define IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_C_CFG_0            0x05878
#define IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_C_CFG_1            0x0587c
#define IFE_COLOR_CORRECT_COLOR_CORRECT_SHIFT_M_CFG              0x0588c
#define IFE_GLUT_DMI_CFG                                         0x05f08
#define IFE_GLUT_DMI_LUT_CFG                                     0x05f0c
#define IFE_GLUT_DMI_DATA                                        0x05f10
#define IFE_GLUT_DMI_LUT_BANK_CFG                                0x05f58
#define IFE_GLUT_MODULE_LUT_BANK_CFG                             0x05f5c
#define IFE_GLUT_MODULE_CFG                                      0x05f60
#define IFE_COLOR_XFORM_MODULE_CFG                               0x06160
#define IFE_COLOR_XFORM_COLOR_XFORM_CH0_COEFF_CFG_0              0x06168
#define IFE_COLOR_XFORM_COLOR_XFORM_CH0_COEFF_CFG_1              0x0616c
#define IFE_COLOR_XFORM_COLOR_XFORM_CH0_ADDOFFSET_CFG            0x06170
#define IFE_COLOR_XFORM_COLOR_XFORM_CH0_SUBOFFSET_CFG            0x06174
#define IFE_COLOR_XFORM_COLOR_XFORM_CH0_CLAMP0_CFG               0x06178
#define IFE_COLOR_XFORM_COLOR_XFORM_CH0_CLAMP1_CFG               0x0617c
#define IFE_COLOR_XFORM_COLOR_XFORM_CH1_COEFF_CFG_0              0x06180
#define IFE_COLOR_XFORM_COLOR_XFORM_CH1_COEFF_CFG_1              0x06184
#define IFE_COLOR_XFORM_COLOR_XFORM_CH1_ADDOFFSET_CFG            0x06188
#define IFE_COLOR_XFORM_COLOR_XFORM_CH1_SUBOFFSET_CFG            0x0618c
#define IFE_COLOR_XFORM_COLOR_XFORM_CH1_CLAMP0_CFG               0x06190
#define IFE_COLOR_XFORM_COLOR_XFORM_CH1_CLAMP1_CFG               0x06194
#define IFE_COLOR_XFORM_COLOR_XFORM_CH2_COEFF_CFG_0              0x06198
#define IFE_COLOR_XFORM_COLOR_XFORM_CH2_COEFF_CFG_1              0x0619c
#define IFE_COLOR_XFORM_COLOR_XFORM_CH2_ADDOFFSET_CFG            0x061a0
#define IFE_COLOR_XFORM_COLOR_XFORM_CH2_SUBOFFSET_CFG            0x061a4
#define IFE_COLOR_XFORM_COLOR_XFORM_CH2_CLAMP0_CFG               0x061a8
#define IFE_COLOR_XFORM_COLOR_XFORM_CH2_CLAMP1_CFG               0x061ac
#define IFE_DOWNSCALE_MN_C_VID_OUT_MODULE_CFG                    0x09a60
#define IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_CFG            0x09a64
#define IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_IMAGE_SIZE_CFG 0x09a68
#define IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_H_CFG          0x09a6c
#define IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_H_PHASE_CFG    0x09a70
#define IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_V_CFG          0x09a74
#define IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_V_PHASE_CFG    0x09a78
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_MODULE_CFG 0x09c60
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_CLAMP_MIN 0x09c70
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_ROUNDING_CFG 0x09c74
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH1_CLAMP_MIN 0x09c78
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH1_ROUNDING_CFG 0x09c7c
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH2_CLAMP_MIN 0x09c80
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH2_ROUNDING_CFG 0x09c84
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_CLAMP_MAX 0x09c88
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH1_CLAMP_MAX 0x09c8c
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH2_CLAMP_MAX 0x09c90
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_MODULE_CFG 0x09e60
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_CLAMP_MIN 0x09e70
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_ROUNDING_CFG 0x09e74
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH1_CLAMP_MIN 0x09e78
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH1_ROUNDING_CFG 0x09e7c
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH2_CLAMP_MIN 0x09e80
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH2_ROUNDING_CFG 0x09e84
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_CLAMP_MAX 0x09e88
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH1_CLAMP_MAX 0x09e8c
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH2_CLAMP_MAX 0x09e90
#define IFE_DSX_Y_VID_OUT_DMI_CFG                                0x0a008
#define IFE_DSX_Y_VID_OUT_DMI_LUT_CFG                            0x0a00c
#define IFE_DSX_Y_VID_OUT_DMI_DATA                               0x0a010
#define IFE_DSX_Y_VID_OUT_DMI_DATA_1                             0x0a014
#define IFE_DSX_Y_VID_OUT_DMI_LUT_BANK_CFG                       0x0a058
#define IFE_DSX_Y_VID_OUT_MODULE_LUT_BANK_CFG                    0x0a05c
#define IFE_DSX_Y_VID_OUT_MODULE_CFG                             0x0a060
#define IFE_DSX_Y_VID_OUT_MODE                                   0x0a064
#define IFE_DSX_Y_VID_OUT_START_LOCATION_X                       0x0a068
#define IFE_DSX_Y_VID_OUT_START_LOCATION_Y                       0x0a06c
#define IFE_DSX_Y_VID_OUT_SCALE_RATIO_X                          0x0a070
#define IFE_DSX_Y_VID_OUT_SCALE_RATIO_Y                          0x0a074
#define IFE_DSX_Y_VID_OUT_OUT_SIZE                               0x0a078
#define IFE_DSX_Y_VID_OUT_IN_SIZE                                0x0a07c
#define IFE_DSX_C_VID_OUT_DMI_CFG                                0x0a208
#define IFE_DSX_C_VID_OUT_DMI_LUT_CFG                            0x0a20c
#define IFE_DSX_C_VID_OUT_DMI_DATA                               0x0a210
#define IFE_DSX_C_VID_OUT_DMI_DATA_1                             0x0a214
#define IFE_DSX_C_VID_OUT_DMI_LUT_BANK_CFG                       0x0a258
#define IFE_DSX_C_VID_OUT_MODULE_LUT_BANK_CFG                    0x0a25c
#define IFE_DSX_C_VID_OUT_MODULE_CFG                             0x0a260
#define IFE_DSX_C_VID_OUT_MODE                                   0x0a264
#define IFE_DSX_C_VID_OUT_START_LOCATION_X                       0x0a268
#define IFE_DSX_C_VID_OUT_START_LOCATION_Y                       0x0a26c
#define IFE_DSX_C_VID_OUT_SCALE_RATIO_X                          0x0a270
#define IFE_DSX_C_VID_OUT_SCALE_RATIO_Y                          0x0a274
#define IFE_DSX_C_VID_OUT_OUT_SIZE                               0x0a278
#define IFE_DSX_C_VID_OUT_IN_SIZE                                0x0a27c
#define IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_MODULE_CFG         0x0a460
#define IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CROP_LINE_CFG      0x0a468
#define IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CROP_PIXEL_CFG     0x0a46c
#define IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH0_CLAMP_MIN      0x0a470
#define IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH1_CLAMP_MIN      0x0a478
#define IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH2_CLAMP_MIN      0x0a480
#define IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH0_CLAMP_MAX      0x0a488
#define IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH1_CLAMP_MAX      0x0a48c
#define IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH2_CLAMP_MAX      0x0a490
#define IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_MODULE_CFG         0x0a660
#define IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CROP_LINE_CFG      0x0a668
#define IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CROP_PIXEL_CFG     0x0a66c
#define IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH0_CLAMP_MIN      0x0a670
#define IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH1_CLAMP_MIN      0x0a678
#define IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH2_CLAMP_MIN      0x0a680
#define IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH0_CLAMP_MAX      0x0a688
#define IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH1_CLAMP_MAX      0x0a68c
#define IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH2_CLAMP_MAX      0x0a690
#define IFE_DOWNSCALE_4TO1_Y_VID_DS16_OUT_MODULE_CFG             0x0a860
#define IFE_DOWNSCALE_4TO1_Y_VID_DS16_OUT_DS_COEFF               0x0a868
#define IFE_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CROP_LINE_CFG          0x0a904
#define IFE_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CROP_PIXEL_CFG         0x0a908
#define IFE_DOWNSCALE_4TO1_C_VID_DS16_OUT_MODULE_CFG             0x0aa60
#define IFE_DOWNSCALE_4TO1_C_VID_DS16_OUT_CROP_LINE_CFG          0x0ab04
#define IFE_DOWNSCALE_4TO1_C_VID_DS16_OUT_CROP_PIXEL_CFG         0x0ab08
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_MODULE_CFG 0x0ac60
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CROP_LINE_CFG 0x0ac68
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CROP_PIXEL_CFG 0x0ac6c
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH0_CLAMP_MIN 0x0ac70
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH1_CLAMP_MIN 0x0ac78
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH2_CLAMP_MIN 0x0ac80
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH0_CLAMP_MAX 0x0ac88
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH1_CLAMP_MAX 0x0ac8c
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH2_CLAMP_MAX 0x0ac90
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_MODULE_CFG 0x0ae60
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CROP_LINE_CFG 0x0ae68
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CROP_PIXEL_CFG 0x0ae6c
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH0_CLAMP_MIN 0x0ae70
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH1_CLAMP_MIN 0x0ae78
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH2_CLAMP_MIN 0x0ae80
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH0_CLAMP_MAX 0x0ae88
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH1_CLAMP_MAX 0x0ae8c
#define IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH2_CLAMP_MAX 0x0ae90
#define IFE_STATS_AEC_BHIST_DMI_CFG                              0x0b208
#define IFE_STATS_AEC_BHIST_DMI_LUT_CFG                          0x0b20c
#define IFE_STATS_AEC_BHIST_DMI_DATA                             0x0b210
#define IFE_STATS_AEC_BHIST_DMI_LUT_BANK_CFG                     0x0b258
#define IFE_STATS_AEC_BHIST_MODULE_LUT_BANK_CFG                  0x0b25c
#define IFE_STATS_AEC_BHIST_MODULE_CFG                           0x0b260
#define IFE_STATS_AEC_BHIST_BLACK_LEVEL_SUB_CFG_0                0x0b264
#define IFE_STATS_AEC_BHIST_RGN_OFFSET_CFG                       0x0b268
#define IFE_STATS_AEC_BHIST_RGN_NUM_CFG                          0x0b26c
#define IFE_STATS_AEC_BHIST_SEGMENT_SLOPE_CFG_0                  0x0b290
#define IFE_STATS_AEC_BHIST_SEGMENT_SLOPE_CFG_1                  0x0b294
#define IFE_STATS_AEC_BHIST_SEGMENT_START_BIN_CFG_0              0x0b298
#define IFE_STATS_AEC_BHIST_SEGMENT_START_BIN_CFG_1              0x0b29c
#define IFE_STATS_AEC_BHIST_SEGMENT_START_BIN_CFG_2              0x0b2a0
#define IFE_STATS_AEC_BHIST_Y_CONV_COEFF_CFG                     0x0b2a4
#define IFE_STATS_AEC_BHIST_RGN_OFFSET_CFG_1                     0x0b2a8
#define IFE_STATS_AEC_BHIST_RGN_OFFSET_CFG_2                     0x0b2ac
#define IFE_STATS_AEC_BHIST_RGN_OFFSET_CFG_3                     0x0b2b0
#define IFE_STATS_AEC_BHIST_RGN_NUM_CFG_1                        0x0b2b4
#define IFE_STATS_AEC_BHIST_RGN_NUM_CFG_2                        0x0b2b8
#define IFE_STATS_AEC_BHIST_RGN_NUM_CFG_3                        0x0b2bc
#define IFE_STATS_AEC_BHIST_RGN_WEIGHT_CFG                       0x0b2c0
#define IFE_COLOR_CORRECT_COLOR_CORRECT_OFFSET_K_CFG(x)          (0x05880 + 4 * (x))
#define IFE_STATS_AEC_BHIST_RGN_0_SLOPE_CFG(x)                   (0x0b2c4 + 4 * (x))
#define IFE_STATS_AEC_BHIST_RGN_1_SLOPE_CFG(x)                   (0x0b2d4 + 4 * (x))
#define IFE_STATS_AEC_BHIST_RGN_2_SLOPE_CFG(x)                   (0x0b2e4 + 4 * (x))
#define IFE_STATS_AEC_BHIST_SEGMENT_BASE_CFG(x)                  (0x0b270 + 4 * (x))

/* gamma 1/2.2: value and slope packed per entry */
static const u32 vfe_pp_gamma_lut[256] = {
	0x52000, 0x1e148, 0x161c0, 0x1221c, 0x10268, 0x0e2ac,
	0x0d2e4, 0x0c31c, 0x0b34c, 0x0a37c, 0x0a3a8, 0x093d0,
	0x093f8, 0x09420, 0x08444, 0x08464, 0x08488, 0x074a8,
	0x074c8, 0x074e4, 0x07504, 0x07520, 0x0653c, 0x06558,
	0x06574, 0x0658c, 0x065a8, 0x065c0, 0x065d8, 0x055f0,
	0x05608, 0x05620, 0x05634, 0x0564c, 0x05664, 0x05678,
	0x0568c, 0x056a4, 0x056b8, 0x056cc, 0x046e0, 0x046f4,
	0x04708, 0x0471c, 0x0472c, 0x04740, 0x04754, 0x04764,
	0x04778, 0x04788, 0x0479c, 0x047ac, 0x047c0, 0x047d0,
	0x047e0, 0x047f4, 0x04804, 0x04814, 0x04824, 0x04834,
	0x03844, 0x03854, 0x03864, 0x03874, 0x03884, 0x03894,
	0x038a0, 0x038b0, 0x038c0, 0x038d0, 0x038dc, 0x038ec,
	0x038fc, 0x03908, 0x03918, 0x03928, 0x03934, 0x03944,
	0x03950, 0x03960, 0x0396c, 0x03978, 0x03988, 0x03994,
	0x039a4, 0x039b0, 0x039bc, 0x039c8, 0x039d8, 0x039e4,
	0x039f0, 0x039fc, 0x03a0c, 0x03a18, 0x03a24, 0x03a30,
	0x03a3c, 0x03a48, 0x03a54, 0x03a60, 0x03a6c, 0x03a78,
	0x02a84, 0x02a90, 0x02a9c, 0x02aa8, 0x02ab4, 0x02ac0,
	0x02acc, 0x02ad8, 0x02ae4, 0x02af0, 0x02afc, 0x02b08,
	0x02b10, 0x02b1c, 0x02b28, 0x02b34, 0x02b40, 0x02b48,
	0x02b54, 0x02b60, 0x02b6c, 0x02b74, 0x02b80, 0x02b8c,
	0x02b94, 0x02ba0, 0x02bac, 0x02bb4, 0x02bc0, 0x02bcc,
	0x02bd4, 0x02be0, 0x02be8, 0x02bf4, 0x02c00, 0x02c08,
	0x02c14, 0x02c1c, 0x02c28, 0x02c30, 0x02c3c, 0x02c44,
	0x02c50, 0x02c58, 0x02c64, 0x02c6c, 0x02c78, 0x02c80,
	0x02c8c, 0x02c94, 0x02c9c, 0x02ca8, 0x02cb0, 0x02cbc,
	0x02cc4, 0x02ccc, 0x02cd8, 0x02ce0, 0x02cec, 0x02cf4,
	0x02cfc, 0x02d08, 0x02d10, 0x02d18, 0x02d20, 0x02d2c,
	0x02d34, 0x02d3c, 0x02d48, 0x02d50, 0x02d58, 0x02d60,
	0x02d6c, 0x02d74, 0x02d7c, 0x02d84, 0x02d90, 0x02d98,
	0x02da0, 0x02da8, 0x02db0, 0x02dbc, 0x02dc4, 0x02dcc,
	0x02dd4, 0x02ddc, 0x02de4, 0x02df0, 0x02df8, 0x02e00,
	0x02e08, 0x02e10, 0x02e18, 0x02e20, 0x02e28, 0x02e34,
	0x02e3c, 0x02e44, 0x02e4c, 0x02e54, 0x02e5c, 0x02e64,
	0x02e6c, 0x02e74, 0x02e7c, 0x02e84, 0x02e8c, 0x02e94,
	0x02e9c, 0x02ea4, 0x02eac, 0x02eb4, 0x02ebc, 0x01ec4,
	0x01ecc, 0x01ed4, 0x01edc, 0x01ee4, 0x01eec, 0x01ef4,
	0x01efc, 0x01f04, 0x01f0c, 0x01f14, 0x01f1c, 0x01f24,
	0x01f2c, 0x01f34, 0x01f3c, 0x01f44, 0x01f4c, 0x01f54,
	0x01f5c, 0x01f60, 0x01f68, 0x01f70, 0x01f78, 0x01f80,
	0x01f88, 0x01f90, 0x01f98, 0x01fa0, 0x01fa4, 0x01fac,
	0x01fb4, 0x01fbc, 0x01fc4, 0x01fcc, 0x01fd4, 0x01fd8,
	0x01fe0, 0x01fe8, 0x01ff0, 0x01ff8,
};

/* downscaler filter coefficients: { hi, lo } */
static const u32 vfe_pp_ds4_y_coeff[96][2] = {
	{ 0x99099098, 0x00000000 }, { 0x99099099, 0x00000000 }, { 0x99099099, 0x00000000 },
	{ 0x99098099, 0x00000000 }, { 0x99098099, 0x00000000 }, { 0x98099099, 0x00000000 },
	{ 0x98098098, 0x00000000 }, { 0x98098098, 0x00000000 }, { 0x98098098, 0x00000000 },
	{ 0x98098098, 0x00000000 }, { 0x98098098, 0x00000000 }, { 0x98097098, 0x00000000 },
	{ 0x98097098, 0x00000000 }, { 0x98098098, 0x00000000 }, { 0x97098098, 0x00000000 },
	{ 0x97097097, 0x00000000 }, { 0x97097097, 0x00000000 }, { 0x97097097, 0x00000000 },
	{ 0x96096097, 0x00000000 }, { 0x95096096, 0x00000000 }, { 0x94095095, 0x00000000 },
	{ 0x93094094, 0x00000000 }, { 0x92093093, 0x00000000 }, { 0x90091092, 0x00000000 },
	{ 0x8e08f090, 0x00000000 }, { 0x8b08c08e, 0x00000000 }, { 0x8808908b, 0x00000000 },
	{ 0x84086088, 0x00000000 }, { 0x80082084, 0x00000000 }, { 0x7c07e080, 0x00000000 },
	{ 0x7707907c, 0x00000000 }, { 0x73075077, 0x00000000 }, { 0x6f071073, 0x00000000 },
	{ 0x6a06d06f, 0x00000000 }, { 0x6706906a, 0x00000000 }, { 0x63065067, 0x00000000 },
	{ 0x60062063, 0x00000000 }, { 0x5e05f060, 0x00000000 }, { 0x5c05d05e, 0x00000000 },
	{ 0x5b05b05c, 0x00000000 }, { 0x5b05b05b, 0x00000000 }, { 0x5c05b05b, 0x00000000 },
	{ 0x5e05d05c, 0x00000000 }, { 0x6106005e, 0x00000000 }, { 0x65063061, 0x00000000 },
	{ 0x68066065, 0x00000000 }, { 0x6c06a068, 0x00000000 }, { 0x7106e06c, 0x00000000 },
	{ 0x74073071, 0x00000000 }, { 0x78076074, 0x00000000 }, { 0x7b07a078, 0x00000000 },
	{ 0x7e07d07b, 0x00000000 }, { 0x7f07f07e, 0x00000000 }, { 0x8008007f, 0x00000000 },
	{ 0x7f080080, 0x00000000 }, { 0x7d07e07f, 0x00000000 }, { 0x7907b07d, 0x00000000 },
	{ 0x74077079, 0x00000000 }, { 0x6e071074, 0x00000000 }, { 0x6606a06e, 0x00000000 },
	{ 0x5d062066, 0x00000000 }, { 0x5405905d, 0x00000000 }, { 0x4b04f054, 0x00000000 },
	{ 0x4104604b, 0x00000000 }, { 0x3703c041, 0x00000000 }, { 0x2d032037, 0x00000000 },
	{ 0x2302802d, 0x00000000 }, { 0x1a01f023, 0x00000000 }, { 0x1201601a, 0x00000000 },
	{ 0x0b00e012, 0x00000000 }, { 0x0500800b, 0x00000000 }, { 0x00002005, 0x00000000 },
	{ 0xfdffe000, 0x0000000f }, { 0xfaffbffd, 0x0000000f }, { 0xf8ff9ffa, 0x0000000f },
	{ 0xf7ff8ff8, 0x0000000f }, { 0xf7ff7ff7, 0x0000000f }, { 0xf7ff7ff7, 0x0000000f },
	{ 0xf7ff7ff7, 0x0000000f }, { 0xf8ff8ff7, 0x0000000f }, { 0xf9ff9ff8, 0x0000000f },
	{ 0xfbffaff9, 0x0000000f }, { 0xfcffbffb, 0x0000000f }, { 0xfdffcffc, 0x0000000f },
	{ 0xfeffeffd, 0x0000000f }, { 0xfffffffe, 0x0000000f }, { 0x00000fff, 0x00000000 },
	{ 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 },
	{ 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 },
	{ 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 },
};

/* downscaler filter coefficients: { hi, lo } */
static const u32 vfe_pp_ds4_c_coeff[48][2] = {
	{ 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 },
	{ 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 },
	{ 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 },
	{ 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 },
	{ 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 },
	{ 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 },
	{ 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 },
	{ 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 }, { 0x00100100, 0x00000001 },
	{ 0xf00f8100, 0x00000000 }, { 0xe00e80f0, 0x00000000 }, { 0xd00d80e0, 0x00000000 },
	{ 0xc00c80d0, 0x00000000 }, { 0xb00b80c0, 0x00000000 }, { 0xa00a80b0, 0x00000000 },
	{ 0x900980a0, 0x00000000 }, { 0x80088090, 0x00000000 }, { 0x70078080, 0x00000000 },
	{ 0x60068070, 0x00000000 }, { 0x50058060, 0x00000000 }, { 0x40048050, 0x00000000 },
	{ 0x30038040, 0x00000000 }, { 0x20028030, 0x00000000 }, { 0x10018020, 0x00000000 },
	{ 0x00008010, 0x00000000 }, { 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 },
	{ 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 },
	{ 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 }, { 0x00000000, 0x00000000 },
};

/* TOP CORE_CFG: pixel pipe input format and enables */
static void vfe_setup_core_config(struct vfe_device *vfe)
{
	writel_relaxed(0x00006000, vfe->base + IFE_TOP_CORE_CFG_0);
}

/* Chroma 2:1 for NV12 */
static void vfe_setup_chroma_downscale_nv12(struct vfe_device *vfe, u32 w, u32 h)
{
	writel_relaxed(0x00000001, vfe->base + IFE_DOWNSCALE_MN_C_VID_OUT_MODULE_CFG);
	writel_relaxed(0x00000600, vfe->base + IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_CFG);
	writel_relaxed(((w - 1) << 16) | (h - 1), vfe->base + IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_IMAGE_SIZE_CFG);
	writel_relaxed(0xc0400000, vfe->base + IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_H_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_H_PHASE_CFG);
	writel_relaxed(0xc0400000, vfe->base + IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_V_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_DOWNSCALE_MN_C_VID_OUT_DOWNSCALE_MN_C_V_PHASE_CFG);
}

/* Demosaic + WB: sub offsets 0, gains 4.0, add offsets 0 */
static void vfe_setup_demosaic(struct vfe_device *vfe)
{
	writel_relaxed(0x00000001, vfe->base + IFE_DEMOSAIC_MODULE_CFG);
	writel_relaxed(0x00000080, vfe->base + IFE_DEMOSAIC_INTERP_COEFF_CFG_0);
	writel_relaxed(0x00000000, vfe->base + IFE_DEMOSAIC_INTERP_COEFF_CFG_1);
	writel_relaxed(0x00000040, vfe->base + IFE_DEMOSAIC_INTERP_CLASSIFIER_CFG_0);
	writel_relaxed(0x00000066, vfe->base + IFE_DEMOSAIC_INTERP_CLASSIFIER_CFG_1);
	writel_relaxed(0x00000001, vfe->base + IFE_WB_GAIN_MODULE_CFG);
	writel_relaxed(0x00001000, vfe->base + IFE_WB_GAIN_WB_CFG_0);
	writel_relaxed(0x00001000, vfe->base + IFE_WB_GAIN_WB_CFG_1);
	writel_relaxed(0x00001000, vfe->base + IFE_WB_GAIN_WB_CFG_2);
	writel_relaxed(0x00000000, vfe->base + IFE_WB_GAIN_WB_OFFSET_CFG_0);
	writel_relaxed(0x00000000, vfe->base + IFE_WB_GAIN_WB_OFFSET_CFG_1);
	writel_relaxed(0x00000000, vfe->base + IFE_WB_GAIN_WB_OFFSET_CFG_2);
	writel_relaxed(0x00000000, vfe->base + IFE_WB_GAIN_WB_OFFSET_CFG_3);
	writel_relaxed(0x00000000, vfe->base + IFE_WB_GAIN_WB_OFFSET_CFG_4);
	writel_relaxed(0x00000000, vfe->base + IFE_WB_GAIN_WB_OFFSET_CFG_5);
}

/* Colour correction: channel permutation, no offsets */
static void vfe_setup_color_correction(struct vfe_device *vfe)
{
	writel_relaxed(0x00000001, vfe->base + IFE_COLOR_CORRECT_MODULE_CFG);
	writel_relaxed(0x00800000, vfe->base + IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_A_CFG_0);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_A_CFG_1);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_B_CFG_0);
	writel_relaxed(0x00000080, vfe->base + IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_B_CFG_1);
	writel_relaxed(0x00000080, vfe->base + IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_C_CFG_0);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_CORRECT_COLOR_CORRECT_COEFF_C_CFG_1);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_CORRECT_COLOR_CORRECT_OFFSET_K_CFG(0));
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_CORRECT_COLOR_CORRECT_OFFSET_K_CFG(1));
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_CORRECT_COLOR_CORRECT_OFFSET_K_CFG(2));
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_CORRECT_COLOR_CORRECT_SHIFT_M_CFG);
}

/* Per-channel gamma LUTs: 1/2.2, 256 entries, same table for R, G and B */
static void vfe_setup_gamma_luts(struct vfe_device *vfe)
{
	int i;

	writel_relaxed(0x00000001, vfe->base + IFE_GLUT_MODULE_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_GLUT_MODULE_LUT_BANK_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_GLUT_DMI_LUT_BANK_CFG);
	writel_relaxed(0x00000001, vfe->base + IFE_GLUT_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_GLUT_DMI_CFG);
	for (i = 0; i < 256; i++)
		writel_relaxed(0, vfe->base + IFE_GLUT_DMI_DATA);
	writel_relaxed(0x00000002, vfe->base + IFE_GLUT_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_GLUT_DMI_CFG);
	for (i = 0; i < 256; i++)
		writel_relaxed(0, vfe->base + IFE_GLUT_DMI_DATA);
	writel_relaxed(0x00000003, vfe->base + IFE_GLUT_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_GLUT_DMI_CFG);
	for (i = 0; i < 256; i++)
		writel_relaxed(0, vfe->base + IFE_GLUT_DMI_DATA);
	writel_relaxed(0x00000000, vfe->base + IFE_GLUT_DMI_LUT_CFG);
	writel_relaxed(0x00000001, vfe->base + IFE_GLUT_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_GLUT_DMI_CFG);
	for (i = 0; i < ARRAY_SIZE(vfe_pp_gamma_lut); i++)
		writel_relaxed(vfe_pp_gamma_lut[i], vfe->base + IFE_GLUT_DMI_DATA);
	writel_relaxed(0x00000002, vfe->base + IFE_GLUT_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_GLUT_DMI_CFG);
	for (i = 0; i < ARRAY_SIZE(vfe_pp_gamma_lut); i++)
		writel_relaxed(vfe_pp_gamma_lut[i], vfe->base + IFE_GLUT_DMI_DATA);
	writel_relaxed(0x00000003, vfe->base + IFE_GLUT_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_GLUT_DMI_CFG);
	for (i = 0; i < ARRAY_SIZE(vfe_pp_gamma_lut); i++)
		writel_relaxed(vfe_pp_gamma_lut[i], vfe->base + IFE_GLUT_DMI_DATA);
	writel_relaxed(0x00000000, vfe->base + IFE_GLUT_DMI_LUT_CFG);
}

/* RGB->YUV: BT.601, Cb/Cr offset 512, clamp 0..1023 */
static void vfe_setup_color_transform(struct vfe_device *vfe)
{
	writel_relaxed(0x00000001, vfe->base + IFE_COLOR_XFORM_MODULE_CFG);
	writel_relaxed(0x02590132, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH0_COEFF_CFG_0);
	writel_relaxed(0x00000074, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH0_COEFF_CFG_1);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH0_ADDOFFSET_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH0_SUBOFFSET_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH0_CLAMP0_CFG);
	writel_relaxed(0xffc00000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH0_CLAMP1_CFG);
	writel_relaxed(0x1ead1f54, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH1_COEFF_CFG_0);
	writel_relaxed(0x00000200, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH1_COEFF_CFG_1);
	writel_relaxed(0x40000000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH1_ADDOFFSET_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH1_SUBOFFSET_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH1_CLAMP0_CFG);
	writel_relaxed(0xffc00000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH1_CLAMP1_CFG);
	writel_relaxed(0x1e540200, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH2_COEFF_CFG_0);
	writel_relaxed(0x00001fad, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH2_COEFF_CFG_1);
	writel_relaxed(0x40000000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH2_ADDOFFSET_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH2_SUBOFFSET_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH2_CLAMP0_CFG);
	writel_relaxed(0xffc00000, vfe->base + IFE_COLOR_XFORM_COLOR_XFORM_CH2_CLAMP1_CFG);
}

/* AEC BHIST: Y histogram over the full frame, DMI cleared */
static void vfe_setup_aec_bhist(struct vfe_device *vfe, u32 w, u32 h)
{
	int i;

	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_DMI_LUT_BANK_CFG);
	writel_relaxed(0x00000001, vfe->base + IFE_STATS_AEC_BHIST_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_DMI_CFG);
	for (i = 0; i < 1024; i++)
		writel_relaxed(0, vfe->base + IFE_STATS_AEC_BHIST_DMI_DATA);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_DMI_LUT_CFG);
	writel_relaxed(0x00000001, vfe->base + IFE_STATS_AEC_BHIST_MODULE_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_BLACK_LEVEL_SUB_CFG_0);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_OFFSET_CFG);
	writel_relaxed(((h / 2 - 1) << 16) | (w / 2 - 1), vfe->base + IFE_STATS_AEC_BHIST_RGN_NUM_CFG);
	writel_relaxed(0xffc00000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_BASE_CFG(0));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_BASE_CFG(1));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_BASE_CFG(2));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_BASE_CFG(3));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_BASE_CFG(4));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_BASE_CFG(5));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_BASE_CFG(6));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_BASE_CFG(7));
	writel_relaxed(0x00000008, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_SLOPE_CFG_0);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_SLOPE_CFG_1);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_START_BIN_CFG_0);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_START_BIN_CFG_1);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_SEGMENT_START_BIN_CFG_2);
	writel_relaxed(0x00081205, vfe->base + IFE_STATS_AEC_BHIST_Y_CONV_COEFF_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_OFFSET_CFG_1);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_OFFSET_CFG_2);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_OFFSET_CFG_3);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_NUM_CFG_1);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_NUM_CFG_2);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_NUM_CFG_3);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_WEIGHT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_0_SLOPE_CFG(0));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_1_SLOPE_CFG(0));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_2_SLOPE_CFG(0));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_0_SLOPE_CFG(1));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_1_SLOPE_CFG(1));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_2_SLOPE_CFG(1));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_0_SLOPE_CFG(2));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_1_SLOPE_CFG(2));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_2_SLOPE_CFG(2));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_0_SLOPE_CFG(3));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_1_SLOPE_CFG(3));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_RGN_2_SLOPE_CFG(3));
	writel_relaxed(0x00000000, vfe->base + IFE_STATS_AEC_BHIST_MODULE_LUT_BANK_CFG);
}

/* Main output round/clamp: round mode 2, clamp 0..255 */
static void vfe_setup_main_output_round_clamp(struct vfe_device *vfe)
{
	writel_relaxed(0x0000fc01, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_MODULE_CFG);
	writel_relaxed(0x0000fc01, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_MODULE_CFG);
	writel_relaxed(0x00000010, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_ROUNDING_CFG);
	writel_relaxed(0x00000010, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH1_ROUNDING_CFG);
	writel_relaxed(0x00000010, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH2_ROUNDING_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH1_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH2_CLAMP_MIN);
	writel_relaxed(0x000000ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH0_CLAMP_MAX);
	writel_relaxed(0x000000ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH1_CLAMP_MAX);
	writel_relaxed(0x000000ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_Y_VID_OUT_CH2_CLAMP_MAX);
	writel_relaxed(0x00000011, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_ROUNDING_CFG);
	writel_relaxed(0x00000011, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH1_ROUNDING_CFG);
	writel_relaxed(0x00000011, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH2_ROUNDING_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH1_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH2_CLAMP_MIN);
	writel_relaxed(0x000000ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH0_CLAMP_MAX);
	writel_relaxed(0x000000ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH1_CLAMP_MAX);
	writel_relaxed(0x000000ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_MN_C_VID_OUT_CH2_CLAMP_MAX);
}

/* DS4: 4:1 downscale of Y and C, with crops */
static void vfe_setup_ds4(struct vfe_device *vfe, u32 w, u32 h)
{
	const u32 ds4_w = ALIGN(w, 8) / 4;
	const u32 ds4_h = ALIGN(h, 8) / 4;
	int i;

	writel_relaxed(0x00000001, vfe->base + IFE_DSX_Y_VID_OUT_MODULE_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_Y_VID_OUT_MODULE_LUT_BANK_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_Y_VID_OUT_MODE);
	writel_relaxed(0x00030000, vfe->base + IFE_DSX_Y_VID_OUT_START_LOCATION_X);
	writel_relaxed(0x00030000, vfe->base + IFE_DSX_Y_VID_OUT_START_LOCATION_Y);
	writel_relaxed(0x00080000, vfe->base + IFE_DSX_Y_VID_OUT_SCALE_RATIO_X);
	writel_relaxed(0x00080000, vfe->base + IFE_DSX_Y_VID_OUT_SCALE_RATIO_Y);
	writel_relaxed((h << 16) | w, vfe->base + IFE_DSX_Y_VID_OUT_IN_SIZE);
	writel_relaxed((ds4_h << 16) | ds4_w, vfe->base + IFE_DSX_Y_VID_OUT_OUT_SIZE);
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_Y_VID_OUT_DMI_LUT_BANK_CFG);
	writel_relaxed(0x00000001, vfe->base + IFE_DSX_Y_VID_OUT_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_Y_VID_OUT_DMI_CFG);
	for (i = 0; i < ARRAY_SIZE(vfe_pp_ds4_y_coeff); i++) {
		writel_relaxed(vfe_pp_ds4_y_coeff[i][0], vfe->base + IFE_DSX_Y_VID_OUT_DMI_DATA_1);
		writel_relaxed(vfe_pp_ds4_y_coeff[i][1], vfe->base + IFE_DSX_Y_VID_OUT_DMI_DATA);
	}
	writel_relaxed(0x00000002, vfe->base + IFE_DSX_Y_VID_OUT_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_Y_VID_OUT_DMI_CFG);
	for (i = 0; i < ARRAY_SIZE(vfe_pp_ds4_y_coeff); i++) {
		writel_relaxed(vfe_pp_ds4_y_coeff[i][0], vfe->base + IFE_DSX_Y_VID_OUT_DMI_DATA_1);
		writel_relaxed(vfe_pp_ds4_y_coeff[i][1], vfe->base + IFE_DSX_Y_VID_OUT_DMI_DATA);
	}
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_Y_VID_OUT_DMI_LUT_CFG);
	writel_relaxed(0x00000001, vfe->base + IFE_DSX_C_VID_OUT_MODULE_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_C_VID_OUT_MODULE_LUT_BANK_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_C_VID_OUT_MODE);
	writel_relaxed(0x00030000, vfe->base + IFE_DSX_C_VID_OUT_START_LOCATION_X);
	writel_relaxed(0x00030000, vfe->base + IFE_DSX_C_VID_OUT_START_LOCATION_Y);
	writel_relaxed(0x00080000, vfe->base + IFE_DSX_C_VID_OUT_SCALE_RATIO_X);
	writel_relaxed(0x00080000, vfe->base + IFE_DSX_C_VID_OUT_SCALE_RATIO_Y);
	writel_relaxed(((h / 2) << 16) | (w / 2), vfe->base + IFE_DSX_C_VID_OUT_IN_SIZE);
	writel_relaxed(((ds4_h / 2) << 16) | (ds4_w / 2), vfe->base + IFE_DSX_C_VID_OUT_OUT_SIZE);
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_C_VID_OUT_DMI_LUT_BANK_CFG);
	writel_relaxed(0x00000001, vfe->base + IFE_DSX_C_VID_OUT_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_C_VID_OUT_DMI_CFG);
	for (i = 0; i < ARRAY_SIZE(vfe_pp_ds4_c_coeff); i++) {
		writel_relaxed(vfe_pp_ds4_c_coeff[i][0], vfe->base + IFE_DSX_C_VID_OUT_DMI_DATA_1);
		writel_relaxed(vfe_pp_ds4_c_coeff[i][1], vfe->base + IFE_DSX_C_VID_OUT_DMI_DATA);
	}
	writel_relaxed(0x00000002, vfe->base + IFE_DSX_C_VID_OUT_DMI_LUT_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_C_VID_OUT_DMI_CFG);
	for (i = 0; i < ARRAY_SIZE(vfe_pp_ds4_c_coeff); i++) {
		writel_relaxed(vfe_pp_ds4_c_coeff[i][0], vfe->base + IFE_DSX_C_VID_OUT_DMI_DATA_1);
		writel_relaxed(vfe_pp_ds4_c_coeff[i][1], vfe->base + IFE_DSX_C_VID_OUT_DMI_DATA);
	}
	writel_relaxed(0x00000000, vfe->base + IFE_DSX_C_VID_OUT_DMI_LUT_CFG);
	writel_relaxed(0x0000fe01, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_MODULE_CFG);
	writel_relaxed(ds4_h - 1, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CROP_LINE_CFG);
	writel_relaxed(ds4_w - 1, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CROP_PIXEL_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH0_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH1_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH2_CLAMP_MIN);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH0_CLAMP_MAX);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH1_CLAMP_MAX);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_Y_VID_OUT_CH2_CLAMP_MAX);
	writel_relaxed(0x0000fe01, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_MODULE_CFG);
	writel_relaxed(ds4_h / 2 - 1, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CROP_LINE_CFG);
	writel_relaxed(ds4_w / 2 - 1, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CROP_PIXEL_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH0_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH1_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH2_CLAMP_MIN);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH0_CLAMP_MAX);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH1_CLAMP_MAX);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DSX_C_VID_OUT_CH2_CLAMP_MAX);
}

/* DS16: 4:1 downscale of DS4 output, with crops */
static void vfe_setup_ds16(struct vfe_device *vfe, u32 w, u32 h)
{
	const u32 ds4_w = ALIGN(w, 8) / 4;
	const u32 ds4_h = ALIGN(h, 8) / 4;
	const u32 ds16_w = (ds4_w / 4) & ~1u;
	const u32 ds16_h = (ds4_h / 4) & ~1u;

	writel_relaxed(0x00000f05, vfe->base + IFE_DOWNSCALE_4TO1_Y_VID_DS16_OUT_MODULE_CFG);
	writel_relaxed(0x09016c7d, vfe->base + IFE_DOWNSCALE_4TO1_Y_VID_DS16_OUT_DS_COEFF);
	writel_relaxed(ds4_h - 1, vfe->base + IFE_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CROP_LINE_CFG);
	writel_relaxed(ds4_w - 1, vfe->base + IFE_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CROP_PIXEL_CFG);
	writel_relaxed(0x00000f05, vfe->base + IFE_DOWNSCALE_4TO1_C_VID_DS16_OUT_MODULE_CFG);
	writel_relaxed(ds4_h / 2 - 1, vfe->base + IFE_DOWNSCALE_4TO1_C_VID_DS16_OUT_CROP_LINE_CFG);
	writel_relaxed(ds4_w / 2 - 1, vfe->base + IFE_DOWNSCALE_4TO1_C_VID_DS16_OUT_CROP_PIXEL_CFG);
	writel_relaxed(0x0000fe01, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_MODULE_CFG);
	writel_relaxed(ds16_h - 1, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CROP_LINE_CFG);
	writel_relaxed(ds16_w - 1, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CROP_PIXEL_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH0_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH1_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH2_CLAMP_MIN);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH0_CLAMP_MAX);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH1_CLAMP_MAX);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_Y_VID_DS16_OUT_CH2_CLAMP_MAX);
	writel_relaxed(0x0000fe01, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_MODULE_CFG);
	writel_relaxed(ds16_h / 2 - 1, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CROP_LINE_CFG);
	writel_relaxed(ds16_w / 2 - 1, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CROP_PIXEL_CFG);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH0_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH1_CLAMP_MIN);
	writel_relaxed(0x00000000, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH2_CLAMP_MIN);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH0_CLAMP_MAX);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH1_CLAMP_MAX);
	writel_relaxed(0x000003ff, vfe->base + IFE_CROP_RND_CLAMP_POST_DOWNSCALE_4TO1_C_VID_DS16_OUT_CH2_CLAMP_MAX);
}

/* Called from vfe_wm_start() for NV12, ahead of the write masters and the IPP reg update */
void vfe_880_setup_pixel_pipe(struct vfe_device *vfe, u32 w, u32 h)
{
	vfe_setup_core_config(vfe);
	vfe_setup_chroma_downscale_nv12(vfe, w, h);
	vfe_setup_demosaic(vfe);
	vfe_setup_color_correction(vfe);
	vfe_setup_gamma_luts(vfe);
	vfe_setup_color_transform(vfe);
	vfe_setup_aec_bhist(vfe, w, h);
	vfe_setup_main_output_round_clamp(vfe);
	vfe_setup_ds4(vfe, w, h);
	vfe_setup_ds16(vfe, w, h);

	/* make the whole configuration visible before the IPP is resumed */
	wmb();
}
