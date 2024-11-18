// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include "dpu_common.h"

#include <drm/drm_fourcc.h>
#include <drm/drm_blend.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>

static struct dpu_kms *
plane_to_dpu_kms(struct drm_plane *plane)
{
	struct msm_drm_private *priv = plane->dev->dev_private;

	return to_dpu_kms(priv->kms);
}

static bool
dpu_format_mod_supported(struct drm_plane *plane, uint32_t format, uint64_t modifier)
{
	if (modifier == DRM_FORMAT_MOD_LINEAR)
		return true;

	if (modifier == DRM_FORMAT_MOD_QCOM_COMPRESSED)
		return true;

	return false;
}

static int
dpu_plane_prepare_fb(struct drm_plane *plane, struct drm_plane_state *state)
{
	struct dpu_kms *dpu = plane_to_dpu_kms(plane);
	int ret;

	if (!state->fb)
		return 0;

	/*
	 * TODO: Need to sort out the msm_framebuffer_prepare() call below so
	 *       we can use msm_atomic_prepare_fb() instead of doing the
	 *       implicit fence and fb prepare by hand here.
	 */
	drm_gem_plane_helper_prepare_fb(plane, state);

	if (dpu->base.aspace) {
		ret = msm_framebuffer_prepare(state->fb, dpu->base.aspace, false);
		if (ret) {
			DRM_ERROR("failed to prepare framebuffer\n");
			return ret;
		}
	}

	return 0;
}

static void
dpu_plane_cleanup_fb(struct drm_plane *plane, struct drm_plane_state *state)
{
	struct dpu_kms *dpu = plane_to_dpu_kms(plane);

	if (!state || !state->fb)
		return;

	msm_framebuffer_cleanup(state->fb, dpu->base.aspace, false);
}

static int dpu_plane_atomic_check(struct drm_plane *plane,
				  struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	const struct drm_crtc_state *crtc_state = NULL;

	if (new_plane_state->crtc)
		crtc_state = drm_atomic_get_new_crtc_state(state,
							   new_plane_state->crtc);

	// TODO: use the actual min/max scaling instead of 0/INF
	return drm_atomic_helper_check_plane_state(new_plane_state, crtc_state, 0,
						   INT_MAX, true, true);

}

void set_framebuffer(struct dpu_kms *dpu, u32 reg, struct drm_framebuffer *fb)
{
	u32 addr[4] = {};
	u32 pitch[4] = {};
	u32 meta_height;

	addr[0] = msm_framebuffer_iova(fb, dpu->base.aspace, 0);
	pitch[0] = fb->pitches[0];

#if 0
	if (fb->format->format == DRM_FORMAT_NV12) {
		addr[1] = msm_framebuffer_iova(fb, dpu->base.aspace, 1);
		pitch[1] = fb->pitches[1];

		if (fb->modifier == DRM_FORMAT_MOD_QCOM_COMPRESSED) {
			addr[2] = addr[0];
			pitch[2] = ALIGN(DIV_ROUND_UP(fb->width, 32), 64);

			meta_height = ALIGN(DIV_ROUND_UP(fb->height, 8), 16);
			addr[0] += ALIGN(pitch[2] * meta_height, 4096);

			addr[3] = addr[1];
			pitch[3] = ALIGN(DIV_ROUND_UP((fb->width+1)>>1, 16), 64);

			meta_height = ALIGN(DIV_ROUND_UP((fb->height+1)>>1, 8), 16);
			addr[1] += ALIGN(pitch[3] * meta_height, 4096);
		}
	} else
#endif
 if (fb->modifier == DRM_FORMAT_MOD_QCOM_COMPRESSED) {
		/*  always hit in CAC test */
		/*
		 * UBWC plane in addr[2]/pitch[2], but comes before in memory
		 *
		 * note: the UBWC pitch should probably be calculated from the
		 * framebuffer pitch and not framebuffer width..
		 */

		addr[2] = addr[0];
		pitch[2] = ALIGN(DIV_ROUND_UP(fb->width, 16), 64);

		meta_height = ALIGN(DIV_ROUND_UP(fb->height, 4), 16);
		addr[0] += ALIGN(pitch[2] * meta_height, 4096);
	}

	// for multirect on CAC unpack planes
	if (reg == SSPP_SRC0_ADDR(16) || reg == SSPP_SRC0_ADDR(17)) {
		/*  always hit in CAC test */
		addr[1] = addr[0];
		addr[3] = addr[2];
		pitch[1] = pitch[0];
		pitch[3] = pitch[2];
	}

	/* write to SSPP_SRC0_ADDR and SRC1/2/3 */
	dpu_write(dpu, reg + 0,  addr[0]);
	dpu_write(dpu, reg + 4,  addr[1]);
	dpu_write(dpu, reg + 8,  addr[2]);
	dpu_write(dpu, reg + 12, addr[3]);

	/* tested that pitch value: 0/1:17408 2/3:320 */
	/* write to SSPP_SRC_YSTRIDE0 and SSPP_SRC_YSTRIDE1 */
	dpu_write(dpu, reg + 16, pitch[0] | pitch[1] << 16);
	dpu_write(dpu, reg + 20, pitch[2] | pitch[3] << 16);
}

void
sspp_set_coords(struct dpu_kms *dpu, u8 i, struct drm_rect *dst, struct drm_rect *src, bool nv12)
{
	u32 src_size = (drm_rect_height(src) & 0xffff0000) | drm_rect_width(src) >> 16;
	u32 src_size_uv = (drm_rect_height(src)/2 & 0xffff0000) | drm_rect_width(src)/2 >> 16;

printk("set coords2----\n");
	dpu_write(dpu, SSPP_SRC_SIZE(i), src_size);
	dpu_write(dpu, SSPP_SRC_SIZE(i)+4, src_size);
	dpu_write(dpu, SSPP_SRC_XY(i), (src->y1 & 0xffff0000) | src->x1 >> 16);
	dpu_write(dpu, SSPP_OUT_SIZE(i),
		  drm_rect_height(dst) << 16 | drm_rect_width(dst));
	dpu_write(dpu, SSPP_OUT_XY(i), dst->y1 << 16 | dst->x1);

	printk("------SSPP_SW_PIX_EXT_C0_REQ_PIXELS and SSPP_SW_PIX_EXT_C3_REQ_PIXELS(%d) 0x%x\n", i, src_size);
	dpu_write(dpu, SSPP_SW_PIX_EXT_C0_REQ_PIXELS(i), src_size);
	printk("------SSPP_SW_PIX_EXT_C1C2_REQ_PIXELS(%d) uv 0x%x size 0x%x\n", i, src_size_uv, src_size);
	dpu_write(dpu, SSPP_SW_PIX_EXT_C1C2_REQ_PIXELS(i), nv12 ? src_size_uv : src_size);
	dpu_write(dpu, SSPP_SW_PIX_EXT_C3_REQ_PIXELS(i), src_size);

}

#define SRC_FORMAT(b, g, r, a) (BIT(17) | (a ? 3 : 2) << 12 | ((b + g + r + a) / 8 - 1) << 9 |  \
	COLOR_##g##BIT | COLOR_##b##BIT << 2 | COLOR_##r##BIT << 4 | COLOR_ALPHA_##a##BIT << 6)

enum {
	COLOR_4BIT,
	COLOR_5BIT,
	COLOR_6BIT,
	COLOR_8BIT,
};

enum {
	COLOR_ALPHA_1BIT,
	COLOR_ALPHA_4BIT,
	COLOR_ALPHA_6BIT,
	COLOR_ALPHA_8BIT,
	COLOR_ALPHA_0BIT = 0, /* for SRC_FORMAT macro only */
};

static inline void
format_info(u32 fourcc, u32 cpp0, u32 *out_format, u32 *out_unpack_pattern)
{
	u32 format, unpack_pattern;
	switch (fourcc >> 16) {
	case DRM_FORMAT_ABGR8888 >> 16:
	default:
		format = SRC_FORMAT(8, 8, 8, 8);
		if (cpp0 == 3)
			format = SRC_FORMAT(8, 8, 8, 0);
		break;
	case DRM_FORMAT_BGR565 >> 16:
		format = SRC_FORMAT(5, 6, 5, 0);
		break;
	case DRM_FORMAT_ABGR1555 >> 16:
		format = SRC_FORMAT(5, 5, 5, 1);
		break;
	case DRM_FORMAT_ABGR4444 >> 16:
		format = SRC_FORMAT(4, 4, 4, 4);
		break;
	}

	switch (fourcc & 0xffff) {
	case DRM_FORMAT_ABGR8888 & 0xffff:
	case DRM_FORMAT_XBGR8888 & 0xffff:
	case DRM_FORMAT_BGR888   & 0xffff:
	default:
		unpack_pattern = 0x03010002;
		break;
	case DRM_FORMAT_ARGB8888 & 0xffff:
	case DRM_FORMAT_XRGB8888 & 0xffff:
	case DRM_FORMAT_RGB888   & 0xffff:
		unpack_pattern = 0x03020001;
		break;
	case DRM_FORMAT_RGBX8888 & 0xffff:
	case DRM_FORMAT_RGBA8888 & 0xffff:
		unpack_pattern = 0x01000203;
		break;
	case DRM_FORMAT_BGRX8888 & 0xffff:
	case DRM_FORMAT_BGRA8888 & 0xffff:
		unpack_pattern = 0x02000103;
		break;
	}
	if (fourcc == DRM_FORMAT_NV12) {
		// XXX: should bit 9 be set or not?
		format = 0x192923f;
		unpack_pattern = 0x0201;
	}

	*out_format = format;
	*out_unpack_pattern = unpack_pattern;
}

void
sspp_set_format(struct dpu_kms *dpu, u8 i, const struct drm_format_info *fmt,
		uint64_t modifier, unsigned rotation)
{
	u32 src_format, unpack_pattern, op_mode;
	u32 cdp_cntl, danger_lut;
	u64 creq_lut;

	format_info(fmt->format, fmt->cpp[0], &src_format, &unpack_pattern);

	op_mode = MDSS_MDP_OP_PE_OVERRIDE;
	cdp_cntl = BIT(0) | BIT(3); /* enable | preload_ahead */
	creq_lut = dpu->creq_lut_linear;
	danger_lut = 0xf;

	if (modifier == DRM_FORMAT_MOD_QCOM_COMPRESSED) {
		op_mode |= MDSS_MDP_OP_BWC_EN;
		src_format |= 2 << 30; /* UBWC fetch */
		unpack_pattern = 0x03010002; /* no swap for UBWC formats */
		cdp_cntl |= BIT(1) | BIT(2); /* ubwc_meta | tile_amortize */
		creq_lut = dpu->creq_lut_macrotile;
		danger_lut = 0xffff;
	}

#if 0
	if (rotation & DRM_MODE_ROTATE_270) {
		src_format |= BIT(11); /* ROT90 */
	}
#endif
	if (rotation & DRM_MODE_REFLECT_X)
		op_mode |= BIT(13); /* FLIP_LR */
	if (rotation & DRM_MODE_REFLECT_Y)
		op_mode |= BIT(14); /* FLIP_UD */

	dpu_write(dpu, SSPP_SRC_FORMAT(i), src_format);
	dpu_write(dpu, SSPP_SRC_UNPACK_PATTERN(i), unpack_pattern);
	dpu_write(dpu, SSPP_SRC_OP_MODE(i), op_mode);

printk("===sspp %d op_mode 0x%x with bit4 set for rec1\n", i, op_mode);
	if (i==16||i==17) { // for CAC unpack planes using multirect
	dpu_write(dpu, SSPP_SRC_FORMAT_REC1(i), src_format);
	dpu_write(dpu, SSPP_SRC_UNPACK_PATTERN_REC1(i), unpack_pattern);
	dpu_write(dpu, SSPP_SRC_OP_MODE_REC1(i), op_mode | BIT(4));
	dpu_write(dpu, SSPP_MULTIRECT_OPMODE(i), 3);
	dpu_write(dpu, SSPP_CDP_CNTL_REC1(i), cdp_cntl);
	} else {
	dpu_write(dpu, SSPP_MULTIRECT_OPMODE(i), 0);
	}

	dpu_write(dpu, SSPP_CDP_CNTL(i), cdp_cntl);
	dpu_write(dpu, SSPP_CREQ_LUT_0(i), creq_lut);
	dpu_write(dpu, SSPP_CREQ_LUT_1(i), creq_lut >> 32);
	dpu_write(dpu, SSPP_DANGER_LUT(i), danger_lut);
	dpu_write(dpu, SSPP_SAFE_LUT(i), 0);
}

void wb_set_framebuffer(struct dpu_kms *dpu, unsigned i, struct drm_framebuffer *fb, int cdm_mux)
{
	const struct drm_format_info *fmt = fb->format;
	u32 dst_format, unpack_pattern, opmode = 0;

printk("===================== wb set fb ================\n");
	format_info(fmt->format, fmt->cpp[0], &dst_format, &unpack_pattern);

	if (dst_format & BIT(12)) {
		dst_format |= BIT(8); /* DSTC3_EN */
		dst_format |= BIT(14); /* DST_ALPHA_X */
	}

	if (fmt->format == DRM_FORMAT_NV12 && i == 0) {
		// CSC, settings from downstream driver, coefficients for bt 601 limited range
		dpu_write(dpu, CDM_CSC_10_OPMODE, BIT(0) | BIT(2)); // dst format=1, src_format=0, enable=1
		dpu_write(dpu, CDM_CSC_10_MATRIX_COEFF_0, 0x0083 | 0x0102 << 16);
		dpu_write(dpu, CDM_CSC_10_MATRIX_COEFF_1, 0x0032 | 0x1fb5 << 16);
		dpu_write(dpu, CDM_CSC_10_MATRIX_COEFF_2, 0x1f6c | 0x00e1 << 16);
		dpu_write(dpu, CDM_CSC_10_MATRIX_COEFF_3, 0x00e1 | 0x1f45 << 16);
		dpu_write(dpu, CDM_CSC_10_MATRIX_COEFF_4, 0x1fdc);

		// pre-clamp: none (full 10-bit range)
		dpu_write(dpu, CDM_CSC_10_COMP_0_PRE_CLAMP, 0x000 << 16 | 0x3ff);
		dpu_write(dpu, CDM_CSC_10_COMP_1_PRE_CLAMP, 0x000 << 16 | 0x3ff);
		dpu_write(dpu, CDM_CSC_10_COMP_2_PRE_CLAMP, 0x000 << 16 | 0x3ff);

		// post-clamp: clamp for limited range
		dpu_write(dpu, CDM_CSC_10_COMP_0_POST_CLAMP, 0x040 << 16 | 0x3ac);
		dpu_write(dpu, CDM_CSC_10_COMP_1_POST_CLAMP, 0x040 << 16 | 0x3c0);
		dpu_write(dpu, CDM_CSC_10_COMP_2_POST_CLAMP, 0x040 << 16 | 0x3c0);

		// pre-bias: none
		dpu_write(dpu, CDM_CSC_10_COMP_0_PRE_BIAS, 0);
		dpu_write(dpu, CDM_CSC_10_COMP_1_PRE_BIAS, 0);
		dpu_write(dpu, CDM_CSC_10_COMP_2_PRE_BIAS, 0);

		// post-bias: CSC coefficients
		dpu_write(dpu, CDM_CSC_10_COMP_0_POST_BIAS, 0x0040); // +0.625
		dpu_write(dpu, CDM_CSC_10_COMP_1_POST_BIAS, 0x0200); // +0.5
		dpu_write(dpu, CDM_CSC_10_COMP_2_POST_BIAS, 0x0200); // +0.5

		// chroma down for NV12, using average method
		dpu_write(dpu, CDM_CDWN2_OUT_SIZE, fb->height << 16 | fb->width);
		dpu_write(dpu, CDM_CDWN2_OP_MODE, 0x87);//BIT(7) | BIT(1) | BIT(0) | 1 << 3 | 1 << 5);
		dpu_write(dpu, CDM_CDWN2_CLAMP_OUT, ((0x3FF << 16) | 0x0));

		dpu_write(dpu, CDM_HDMI_PACK_OP_MODE, 0); // ???
		dpu_write(dpu, CDM_MUX, cdm_mux);
	} else if (i == 0) {
		dpu_write(dpu, CDM_CDWN2_OP_MODE, 0);
		dpu_write(dpu, CDM_CSC_10_OPMODE, 0);
	}

	if (fb->modifier == DRM_FORMAT_MOD_QCOM_COMPRESSED) {
		opmode |= BIT(0);
		dst_format |= BIT(31);
	}

	dpu_write(dpu, WB_DST_FORMAT(i), dst_format);
	dpu_write(dpu, WB_DST_OP_MODE(i), opmode);
	dpu_write(dpu, WB_DST_PACK_PATTERN(i), unpack_pattern);
	// dpu_write(dpu, WB_DST_WRITE_CONFIG(i), write_config); 8650 doesn't have the bits that downstream sets here

	set_framebuffer(dpu, WB_DST0_ADDR(i), fb);

	dpu_write(dpu, WB_OUT_XY(i), 0);
	dpu_write(dpu, WB_OUT_SIZE(i), fb->height << 16 | fb->width);
	dpu_write(dpu, WB_OUT_IMAGE_SIZE(i), fb->height << 16 | fb->width);
}

void
dpu_plane_update(struct dpu_kms *dpu, u32 i, struct drm_plane_state *state)
{
	/* convert ROTATE_180 to REFLECT_X|REFLECT_Y: */
	unsigned rotation = drm_rotation_simplify(state->rotation,
		 DRM_MODE_ROTATE_0 | DRM_MODE_REFLECT_X |  DRM_MODE_REFLECT_Y);
	struct drm_framebuffer *fb = state->fb;
	if (!fb)
		return;

	if (i >= 4) i += 12; // DMA sspp starts at 0x20000 offset instead of 0x8000

printk("-----set_framebuffer....\n");
	set_framebuffer(dpu, SSPP_SRC0_ADDR(i), fb);
printk("-----sspp_set_coords....\n");
	//sspp_set_coords(dpu, i, &state->dst, &state->src, fb->format->format == DRM_FORMAT_NV12);
printk("set coords3--------\n");
	sspp_set_format(dpu, i, fb->format, fb->modifier, rotation);
}

static void
dpu_plane_atomic_update(struct drm_plane *plane,
			struct drm_atomic_state *old_state)
{
	/* there isn't a 1:1 mapping to HW planes, update in crtc update */
}

static const struct drm_plane_funcs dpu_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
	.format_mod_supported   = dpu_format_mod_supported,
};

static const struct drm_plane_helper_funcs dpu_plane_helper_funcs = {
	.prepare_fb = dpu_plane_prepare_fb,
	.cleanup_fb = dpu_plane_cleanup_fb,
	.atomic_check = dpu_plane_atomic_check,
	.atomic_update = dpu_plane_atomic_update,
};

static const uint32_t plane_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_ABGR1555,
	DRM_FORMAT_RGBA5551,
	DRM_FORMAT_BGRA5551,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_XBGR1555,
	DRM_FORMAT_RGBX5551,
	DRM_FORMAT_BGRX5551,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_ABGR4444,
	DRM_FORMAT_RGBA4444,
	DRM_FORMAT_BGRA4444,
	DRM_FORMAT_XRGB4444,
	DRM_FORMAT_XBGR4444,
	DRM_FORMAT_RGBX4444,
	DRM_FORMAT_BGRX4444,
	DRM_FORMAT_NV12,
};

static const uint64_t supported_modifiers[] = {
	DRM_FORMAT_MOD_QCOM_COMPRESSED,
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

int dpu_plane_init(struct dpu_kms *dpu, struct drm_plane *plane, uint32_t possible_crtcs)
{
	int ret;

	ret = drm_universal_plane_init(dpu->dev, plane, possible_crtcs, &dpu_plane_funcs,
			plane_formats, ARRAY_SIZE(plane_formats),
			supported_modifiers, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ret;

	drm_plane_create_zpos_property(plane, 0, 0, dpu->num_blendstages - 1);
	drm_plane_create_rotation_property(plane, DRM_MODE_ROTATE_0, DRM_MODE_ROTATE_0 |
		// TODO: test and re-enable 90/270 rotation
		// DRM_MODE_ROTATE_270 |
		DRM_MODE_ROTATE_0 |
		DRM_MODE_ROTATE_180 |
		DRM_MODE_REFLECT_X |
		DRM_MODE_REFLECT_Y);
	drm_plane_helper_add(plane, &dpu_plane_helper_funcs);

	return 0;
}
