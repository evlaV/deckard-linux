// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include "dpu_common.h"

// TODO
// primary vs overlay planes (vkcube doesn't UBWC...)
// UBWC setting in top
// enable async commit
// irq stuff
// dma plane crash
// weston mode change problem
// 4k mode needs larger vporch than expected (not related to dpu)

/* flush bits table for 4 vig and 4 dma config */
static const u8 sspp_flush_tbl[10] = {0, 1, 2, 18, 11, 12, 24, 25, 13, 14};
#define CTL_FLUSH_SSPP(i) BIT(sspp_flush_tbl[i])

static void
intf_timings(struct dpu_kms *dpu, u32 i, const struct drm_display_mode *mode,
	     bool is_dual_dsi, bool is_dp, bool is_dsc)
{
	/* from dsi_host.c */
	u32 h_total = mode->htotal;
	u32 v_total = mode->vtotal;
	u32 hs_end = mode->hsync_end - mode->hsync_start; /* h_pulse_width */
	u32 vs_end = mode->vsync_end - mode->vsync_start; /* v_pulse_width */
	u32 ha_start = h_total - mode->hsync_start;
	u32 ha_end = ha_start + mode->hdisplay;
	u32 va_start = v_total - mode->vsync_start;
	u32 va_end = va_start + mode->vdisplay;
	u32 hdisplay = mode->hdisplay;

	if (is_dual_dsi) {
		h_total /= 2;
		hs_end /= 2;
		ha_start /= 2;
		ha_end /= 2;
		hdisplay /= 2;
	}

	if (is_dsc) {
		h_total -= hdisplay;
		hdisplay /= 3;
		h_total += hdisplay;
		ha_end = ha_start + hdisplay;
	}

	if (is_dp) {
		ha_start = h_total - mode->hdisplay;
		ha_end = h_total;
		va_start = v_total - mode->vdisplay;
		va_end = v_total;
	}

	va_start = va_start * h_total + mode->hskew;
	va_end = va_end * h_total + mode->hskew - 1;
	ha_end = ha_end - 1;

	dpu_write(dpu, INTF_HSYNC_CTL(i), h_total << 16 | hs_end);
	dpu_write(dpu, INTF_VSYNC_PERIOD_F0(i), h_total * v_total);
	dpu_write(dpu, INTF_VSYNC_PULSE_WIDTH_F0(i), h_total * vs_end);
	dpu_write(dpu, INTF_DISPLAY_HCTL(i), ha_end << 16 | ha_start);

	if (is_dp) {
		dpu_write(dpu, INTF_DISPLAY_V_START_F0(i), va_start + ha_start);
		dpu_write(dpu, INTF_DISPLAY_V_END_F0(i), va_end);
		dpu_write(dpu, INTF_ACTIVE_HCTL(i), ha_end << 16 | ha_start);
		dpu_write(dpu, INTF_ACTIVE_V_START_F0(i), va_start);
		dpu_write(dpu, INTF_ACTIVE_V_END_F0(i), va_end);
	} else {
		dpu_write(dpu, INTF_DISPLAY_V_START_F0(i), va_start);
		dpu_write(dpu, INTF_DISPLAY_V_END_F0(i), va_end);
		dpu_write(dpu, INTF_ACTIVE_HCTL(i), 0);
		dpu_write(dpu, INTF_ACTIVE_V_START_F0(i), 0);
		dpu_write(dpu, INTF_ACTIVE_V_END_F0(i), 0);
	}

	dpu_write(dpu, INTF_BORDER_COLOR(i), 0);
	dpu_write(dpu, INTF_UNDERFLOW_COLOR(i), 0xff);
	dpu_write(dpu, INTF_HSYNC_SKEW(i), mode->hskew);
	dpu_write(dpu, INTF_POLARITY_CTL(i),
		  COND(mode->flags & DRM_MODE_FLAG_NHSYNC, 1) |
		  COND(mode->flags & DRM_MODE_FLAG_NVSYNC, 2));
	dpu_write(dpu, INTF_FRAME_LINE_COUNT_EN(i), 0x3);
	dpu_write(dpu, INTF_PANEL_FORMAT(i), 0x213f);

	/* XXX */
	if (!is_dp) {
		u32 data_width = hdisplay / 2; // widebus
		u32 hsync_data_end_x = ha_start + data_width - 1;

		dpu_write(dpu, INTF_PROG_FETCH_START(i), 0);//is_dsc ? 0x1A0CC1 : 0x1B2731);
		dpu_write(dpu, INTF_CONFIG(i), 0x00800000);
		dpu_write(dpu, INTF_CONFIG2(i), (is_dsc ? BIT(12) : 0) |
			BIT(4) | BIT(0)); // widebus
		dpu_write(dpu, INTF_DISPLAY_DATA_HCTL(i), hsync_data_end_x << 16 | ha_start); // widebus
	} else {
		dpu_write(dpu, INTF_CONFIG(i), 0x60800000);
		dpu_write(dpu, INTF_CONFIG2(i), 0x10);
		dpu_write(dpu, INTF_DISPLAY_DATA_HCTL(i), ha_end << 16 | ha_start);
	}

#if 0
	{
	u32 cfg = 0;
	spinlock_t tearcheck_spinlock;

	spin_lock_init(&tearcheck_spinlock);

	if (0) //(te->hw_vsync_mode)
		cfg |= BIT(20);

	cfg |= (19200000 / ((2400 + 20 + 2 + 18) * 144));

#define DEFAULT_TEARCHECK_SYNC_THRESH_CONTINUE	4
#define DEFAULT_TEARCHECK_SYNC_THRESH_START	4
	/*
	 * Local spinlock is acquired here to avoid pre-emption
	 * as below register programming should be completed in
	 * less than 2^16 vsync clk cycles.
	 */
	spin_lock(&tearcheck_spinlock);
	dpu_write(dpu, INTF_TEAR_SYNC_WRCOUNT(i),
			(2400 + DEFAULT_TEARCHECK_SYNC_THRESH_START + 1));
	dpu_write(dpu, INTF_TEAR_SYNC_CONFIG_VSYNC(i), cfg);
	dpu_write(dpu, INTF_TEAR_SYNC_CONFIG_HEIGHT(i), 0xFFF0);
	dpu_write(dpu, INTF_TEAR_VSYNC_INIT_VAL(i), 2400);
	dpu_write(dpu, INTF_TEAR_RD_PTR_IRQ(i), 2400 + 1);
	dpu_write(dpu, INTF_TEAR_WR_PTR_IRQ(i), 1);
	dpu_write(dpu, INTF_TEAR_START_POS(i), 2400);
	dpu_write(dpu, INTF_TEAR_SYNC_THRESH(i),
			((DEFAULT_TEARCHECK_SYNC_THRESH_CONTINUE << 16) |
			 DEFAULT_TEARCHECK_SYNC_THRESH_START));
	cfg |= BIT(19); /* VSYNC_COUNTER_EN */
	dpu_write(dpu, INTF_TEAR_SYNC_CONFIG_VSYNC(i), cfg);

	}
#endif
}

#define DSC_CMN_MAIN_CNF(i)	(0x80000 + 0x1000 * (i))

/* SDE_DSC_ENC regsiter offsets */
#define ENC_DF_CTRL                0x00
#define ENC_GENERAL_STATUS         0x04
#define ENC_HSLICE_STATUS          0x08
#define ENC_OUT_STATUS             0x0C
#define ENC_INT_STAT               0x10
#define ENC_INT_CLR                0x14
#define ENC_INT_MASK               0x18
#define DSC_MAIN_CONF              0x30
#define DSC_PICTURE_SIZE           0x34
#define DSC_SLICE_SIZE             0x38
#define DSC_MISC_SIZE              0x3C
#define DSC_HRD_DELAYS             0x40
#define DSC_RC_SCALE               0x44
#define DSC_RC_SCALE_INC_DEC       0x48
#define DSC_RC_OFFSETS_1           0x4C
#define DSC_RC_OFFSETS_2           0x50
#define DSC_RC_OFFSETS_3           0x54
#define DSC_RC_OFFSETS_4           0x58
#define DSC_FLATNESS_QP            0x5C
#define DSC_RC_MODEL_SIZE          0x60
#define DSC_RC_CONFIG              0x64
#define DSC_RC_BUF_THRESH_0        0x68
#define DSC_RC_BUF_THRESH_1        0x6C
#define DSC_RC_BUF_THRESH_2        0x70
#define DSC_RC_BUF_THRESH_3        0x74
#define DSC_RC_MIN_QP_0            0x78
#define DSC_RC_MIN_QP_1            0x7C
#define DSC_RC_MIN_QP_2            0x80
#define DSC_RC_MAX_QP_0            0x84
#define DSC_RC_MAX_QP_1            0x88
#define DSC_RC_MAX_QP_2             0x8C
#define DSC_RC_RANGE_BPG_OFFSETS_0  0x90
#define DSC_RC_RANGE_BPG_OFFSETS_1  0x94
#define DSC_RC_RANGE_BPG_OFFSETS_2  0x98

static inline void dsc_write(struct dpu_kms *dpu, u32 i, u32 reg, u32 val)
{
	u32 base = 0x80100 + 0x1000 * (i>>1) + 0x100 * (i&1);
	dpu_write(dpu, base + reg, val);
}

/* SDE_DSC_CTL regsiter offsets */
#define DSC_CTL                    0x00
#define DSC_CFG                    0x04
#define DSC_DATA_IN_SWAP           0x08
#define DSC_CLK_CTRL               0x0C
#define DSC_4HS_MERGE_EN           0x10
#define DSC_4HS_MERGE_CFG          0x14

static inline void dsc_write_ctl(struct dpu_kms *dpu, u32 i, u32 reg, u32 val)
{
	u32 base = 0x80f00 + 0x1000 * (i>>1) + 0x80 * (i&1);
	dpu_write(dpu, base + reg, val);
}

static void
set_dsc_config(struct dpu_kms *dpu, u32 i, const struct drm_dsc_config *dsc)
{
	u32 initial_lines = 3; // check this. see _dce_dsc_initial_line_calc in sde
	u32 num_active_ss_per_enc = 2;//2; // 2 slices per encoder (540 slice width, 2 encoders per panel)
	if (!dpu->crtc[0].is_dual_dsi)
		num_active_ss_per_enc = 1; // vtdr6130 test

	u32 ob_max_addr = 599; // see _dsc_calc_ob_max_addr
	u32 det_thresh_flatness = 2 << (dsc->bits_per_component - 8);
	u32 j, k;

	dpu_write(dpu, DSC_CMN_MAIN_CNF(i>>1),
		  BIT(0) | // split panel
		  BIT(1) | // multiplex
		  num_active_ss_per_enc << 7);

	dsc_write(dpu, i, ENC_DF_CTRL, initial_lines |
		BIT(9) | // VIDEO_MODE
		// ich_reset_override : set bit 10 and 11
		// config.bits_per_component > 8: bit 12
		ob_max_addr << 18);

	// native_420/native_422 only set for minor version ?
	// bits_per_pixel is doubled if native_420|native_422 ?
	dsc_write(dpu, i, DSC_MAIN_CONF,
		dsc->bits_per_component |
		dsc->convert_rgb << 4 |
		dsc->simple_422 << 5 |
		dsc->line_buf_depth  << 6 |
		dsc->bits_per_pixel << 10 |
		dsc->block_pred_enable << 20 |
		dsc->native_420 << 21 |
		dsc->native_422 << 22 |
		dsc->dsc_version_minor << 28);

	dsc_write(dpu, i, DSC_PICTURE_SIZE, dsc->pic_width | dsc->pic_height << 16);
	dsc_write(dpu, i, DSC_SLICE_SIZE, dsc->slice_width | dsc->slice_height << 16);
	dsc_write(dpu, i, DSC_MISC_SIZE, dsc->slice_chunk_size);
	dsc_write(dpu, i, DSC_HRD_DELAYS, dsc->initial_xmit_delay | dsc->initial_dec_delay << 16);
	dsc_write(dpu, i, DSC_RC_SCALE, dsc->initial_scale_value);
	dsc_write(dpu, i, DSC_RC_SCALE_INC_DEC, dsc->scale_increment_interval | dsc->scale_decrement_interval << 16);
	dsc_write(dpu, i, DSC_RC_OFFSETS_1, dsc->first_line_bpg_offset | dsc->second_line_bpg_offset << 5);
	dsc_write(dpu, i, DSC_RC_OFFSETS_2, dsc->nfl_bpg_offset | dsc->slice_bpg_offset << 16);
	dsc_write(dpu, i, DSC_RC_OFFSETS_3, dsc->initial_offset | dsc->final_offset << 16);
	dsc_write(dpu, i, DSC_RC_OFFSETS_4, dsc->nsl_bpg_offset | dsc->second_line_offset_adj << 16);
	dsc_write(dpu, i, DSC_FLATNESS_QP, dsc->flatness_min_qp | dsc->flatness_max_qp << 5 | det_thresh_flatness << 10);
	dsc_write(dpu, i, DSC_RC_MODEL_SIZE, dsc->rc_model_size);
	dsc_write(dpu, i, DSC_RC_CONFIG,
		dsc->rc_edge_factor |
		dsc->rc_quant_incr_limit0 << 8 |
		dsc->rc_quant_incr_limit1 << 13 |
		dsc->rc_tgt_offset_high << 20 |
		dsc->rc_tgt_offset_low << 24);
	for (j = 0; j < 12; j += 4) {
		dsc_write(dpu, i, DSC_RC_BUF_THRESH_0 + j,
			dsc->rc_buf_thresh[j + 0] |
			dsc->rc_buf_thresh[j + 1] << 8 |
			dsc->rc_buf_thresh[j + 2] << 16 |
			dsc->rc_buf_thresh[j + 3] << 24);
	}
	dsc_write(dpu, i, DSC_RC_BUF_THRESH_3, dsc->rc_buf_thresh[j + 0] | dsc->rc_buf_thresh[j + 1] << 8);

	for (j = 0, k = 0; j < DSC_NUM_BUF_RANGES; j += 5, k += 4) {
		dsc_write(dpu, i, DSC_RC_MIN_QP_0 + k,
			dsc->rc_range_params[j + 0].range_min_qp |
			dsc->rc_range_params[j + 1].range_min_qp  << 5 |
			dsc->rc_range_params[j + 2].range_min_qp  << 10 |
			dsc->rc_range_params[j + 3].range_min_qp  << 15 |
			dsc->rc_range_params[j + 4].range_min_qp  << 20);
		dsc_write(dpu, i, DSC_RC_MAX_QP_0 + k,
			dsc->rc_range_params[j + 0].range_max_qp |
			dsc->rc_range_params[j + 1].range_max_qp  << 5 |
			dsc->rc_range_params[j + 2].range_max_qp  << 10 |
			dsc->rc_range_params[j + 3].range_max_qp  << 15 |
			dsc->rc_range_params[j + 4].range_max_qp  << 20);
		dsc_write(dpu, i, DSC_RC_RANGE_BPG_OFFSETS_0 + k,
			dsc->rc_range_params[j + 0].range_bpg_offset |
			dsc->rc_range_params[j + 1].range_bpg_offset  << 6 |
			dsc->rc_range_params[j + 2].range_bpg_offset  << 12 |
			dsc->rc_range_params[j + 3].range_bpg_offset  << 18 |
			dsc->rc_range_params[j + 4].range_bpg_offset  << 24);
	}

	dsc_write_ctl(dpu, i, DSC_CFG,
		BIT(0) | // enable
		BIT(11) | // 8 bpc
		BIT(12) | // split panel
		BIT(13)); // multiplex
	// no 4hs merge
	dsc_write_ctl(dpu, i, DSC_4HS_MERGE_CFG, 0);
	dsc_write_ctl(dpu, i, DSC_4HS_MERGE_EN, 0);

	dsc_write_ctl(dpu, i, DSC_CTL, i);
}

static void
set_dsc_disable(struct dpu_kms *dpu, u32 i)
{
	dsc_write_ctl(dpu, i, DSC_CFG, 0);
}

static void
dpu_crtc_disable(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	drm_crtc_vblank_off(crtc);
}

static void
dpu_crtc_enable(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	drm_crtc_vblank_on(crtc);
}

static int
dpu_crtc_atomic_check(struct drm_crtc *crtc, struct drm_atomic_state *state)
{
	/* TODO: actually perform checks */
	return 0;
}

static void
dpu_crtc_atomic_begin(struct drm_crtc *crtc, struct drm_atomic_state *old_state)
{
	/* nothing */
}

static const u8 rt_pri_lvl[] = {3, 3, 4, 4, 5, 5, 6, 6};
static const u8 sspp_xin_id[] = {0, 4, 8, 12, 1, 5, 9, 13};

enum {
	/* alpha source */
	ALPHA_FG_CONST,
	ALPHA_BG_CONST,
	ALPHA_FG_PIXEL,
	ALPHA_BG_PIXEL,
	/* modifiers */
	INV_ALPHA = BIT(2),
	MOD_ALPHA = BIT(3),
	INV_MOD_ALPHA = BIT(4),
	TRANSP_EN = BIT(5),
};

#define CTL_MIXER_BORDER_OUT            BIT(24)

static void
mixer_set_blendfunc(struct dpu_kms *dpu, u32 i, u32 stage)
{
#ifdef SIXPLANEMODE
	u32 alpha_const = 0xff << 16 | 0xff; /* fg_const = 0xff, bg_const = 0 */
#else
	u32 alpha_const = 0xff << 16 | 0; /* fg_const = 0xff, bg_const = 0 */
#endif
	u32 fg_func = ALPHA_FG_CONST; /* solid fg */
	u32 bg_func = ALPHA_BG_CONST; /* invisible bg */

	/* TODO: blending for format with alpha channel */
	if (0) {
		fg_func = ALPHA_FG_PIXEL;
		bg_func = ALPHA_FG_PIXEL | INV_ALPHA;
	}

	dpu_write(dpu, LM_BLEND0_OP(i, stage), fg_func | bg_func << 8);
	dpu_write(dpu, LM_BLEND0_CONST_ALPHA(i, stage), alpha_const);
}

struct mixercfg {
	u32 cfg[5];
};

static void
mixercfg_set_sspp_stage(struct mixercfg *cfg, u32 sspp, u32 stage)
{
	/* assumes sspp index is 0-3 VIG, 4-7 DMA */
	static const u8 stage_bits_lo[10] = {0, 3, 6, 26, 18, 21, 0, 4, 0, 4};
	static const u8 stage_bits_hi[6] = {0, 2, 4,  6, 16, 18};

	/* offset to skip (0 = no stage, 1 = base stage) */
	stage += 2;

	if (sspp >= 8) {
		cfg->cfg[4] |= stage << stage_bits_lo[sspp];
	} else if (sspp < 6) {
		cfg->cfg[0] |= (stage & 7) << stage_bits_lo[sspp];
		cfg->cfg[1] |= (stage >= 8) << stage_bits_hi[sspp];
	} else {
		cfg->cfg[2] |= stage << stage_bits_lo[sspp];
	}
}

static void
ctl_set_mixercfg(struct dpu_kms *dpu, u32 ctl, u32 i, struct mixercfg *cfg)
{
	dpu_write(dpu, CTL_LAYER(ctl, i), cfg->cfg[0] | CTL_MIXER_BORDER_OUT);
	dpu_write(dpu, CTL_LAYER_EXT(ctl, i), cfg->cfg[1]);
	dpu_write(dpu, CTL_LAYER_EXT2(ctl, i), cfg->cfg[2]);
	dpu_write(dpu, CTL_LAYER_EXT3(ctl, i), cfg->cfg[3]);
	dpu_write(dpu, CTL_LAYER_EXT4(ctl, i), cfg->cfg[4]);
}

static void
dpu_crtc_atomic_flush(struct drm_crtc *crtc, struct drm_atomic_state *old_state)
{
	struct drm_crtc_state *state = crtc->state;
	struct drm_plane *plane;
	const struct drm_display_mode *mode = &state->mode;
	struct msm_drm_private *priv = crtc->dev->dev_private;
	struct dpu_kms *dpu = to_dpu_kms(priv->kms);
	struct dpu_crtc *c = to_dpu_crtc(crtc);
	unsigned long flags;
	bool is_dsc = dpu->dsc->dsc_version_major && !c->is_dp;
	u32 fetch_pipe_active = 0;

	struct mixercfg mixer_cfg[4] = {};
	struct drm_rect mixer_rect[4];
	u32 intf_active = 0;
	u32 merge3d_active = 0;
	u32 ctl_flush = CTL_FLUSH_MASK_CTL;
#ifdef SIXPLANEMODE
	u32 mixer_op_mode[2] = {};
	u32 num_planes = 0;
#else
	u32 mixer_op_mode = 0;
#endif

	unsigned i, num_mixers, num_mixercfg;

	/*
	 * DSC has a max width of 2048, so for dual 2160x2160 panels with DSC,
	 * 4 mixers are required even though mixer max width is 2560.
	 * this is a "QUADPIPE_DSCMERGE" topology
	 *
	 * for this configuration, we need to use 2 HW planes per plane because
	 * 1) plane max width is 4096, lower than 4320
	 * 2) a single plane can only be used by one mixer pair
	 * 3) plane can only read 1 pixel/clock, but max DPU clock is 460Mhz
	 *
	 * 3) is also a problem with DP (2 mixer merge3d configuration),
	 * because for example 3840x2160@60 needs more than 460Mhz DPU clock
	 * so 2 HW plane mechanism is used with DP too to avoid overclocking
	 *
	 * this 2 plane thing might result in slightly incorrect scaling when
	 * the plane is split across two HW planes, but this driver doesn't
	 * support scaling..
	 */

	if (c->is_dual_dsi && is_dsc)
		num_mixers = 4;
	else if (c->is_dp || c->is_dual_dsi)
		num_mixers = 2;
	else
		num_mixers = 1;
#ifdef SIXPLANEMODE
	num_mixercfg = num_mixers >= 2 ? 2 : 1;
#else
	num_mixercfg = num_mixers;
#endif

	if (num_mixercfg == 4) {
		mixer_rect[0] = (struct drm_rect) {
			0, 0, 1080, mode->vdisplay,
		};
		mixer_rect[1] = (struct drm_rect) {
			1080, 0, 2160, mode->vdisplay,
		};
		mixer_rect[2] = (struct drm_rect) {
			2160, 0, 3240, mode->vdisplay,
		};
		mixer_rect[3] = (struct drm_rect) {
			3240, 0, 4320, mode->vdisplay,
		};
	} else if (num_mixercfg == 2) {
		mixer_rect[0] = (struct drm_rect) {
			0, 0, mode->hdisplay / 2, mode->vdisplay,
		};
		mixer_rect[1] = (struct drm_rect) {
			mode->hdisplay / 2, 0, mode->hdisplay, mode->vdisplay,
		};
	} else {
		mixer_rect[0] = (struct drm_rect) {
			0, 0, mode->hdisplay, mode->vdisplay,
		};
	}

	if (!state->enable) {
		c->event = state->event;
		state->event = NULL;
		/*
		 * timing engine is disabled so vblank irq will never come
		 * complete the vblank event immediately
		 */
		dpu_complete_flip(dpu, c);
		/*
		 * skip crtc state update because it will be updated with new state
		 * before the timing engine is enabled again
		 */
		return;
	}


#ifdef SIXPLANEMODE
	drm_atomic_crtc_for_each_plane(plane, crtc)
		num_planes++;

	// support "normal" single plane by faking 2 planes
	if (num_planes == 1 && c->is_dual_dsi) {
		drm_atomic_crtc_for_each_plane(plane, crtc) {
			u32 stage = plane->state->normalized_zpos;

			mixer_op_mode[0] |= 1 << (stage + 1); /* + 1 to skip base stage */
			mixer_op_mode[1] |= 1 << (stage + 1); /* + 1 to skip base stage */

			for (i = c->lm; i < c->lm + num_mixers; i++)
				mixer_set_blendfunc(dpu, i, stage);

			for (i = 0; i < num_mixercfg; i++) {
				u32 id = to_dpu_plane(plane)->id + i;
				struct drm_plane_state s = *plane->state;
				if (!drm_rect_clip_scaled(&s.src, &s.dst, &mixer_rect[i]))
					continue;

				s.dst.x1 -= mixer_rect[i].x1;
				s.dst.x2 -= mixer_rect[i].x1;

				/* mapping sspp to mixer stages */
				mixercfg_set_sspp_stage(&mixer_cfg[i], id, stage);

				/* sspp state */
				dpu_plane_update(dpu, id, &s, false);
				ctl_flush |= CTL_FLUSH_SSPP(id);

				if (id >= 4)
					fetch_pipe_active |= BIT(id - 4);
				else
					fetch_pipe_active |= BIT(id + 16);
			}
		}
	} else {
		drm_atomic_crtc_for_each_plane(plane, crtc) {
			u32 stage = plane->state->normalized_zpos;
			u32 mixercfg_id = 0;
			if (c->is_dual_dsi && plane->state->dst.x1 >= mixer_rect[!mixercfg_id].x1)
				mixercfg_id = !mixercfg_id;

			mixer_op_mode[mixercfg_id] |= 1 << (stage + 1); /* + 1 to skip base stage */

			if (c->is_dual_dsi) {
				if (mixercfg_id) {
					mixer_set_blendfunc(dpu, 2, stage);
					mixer_set_blendfunc(dpu, 3, stage);
				} else {
					mixer_set_blendfunc(dpu, 0, stage);
					mixer_set_blendfunc(dpu, 1, stage);
				}
			} else {
				for (i = c->lm; i < c->lm + num_mixers; i++)
					mixer_set_blendfunc(dpu, i, stage);
			}

			u32 id = to_dpu_plane(plane)->id;
			struct drm_plane_state s = *plane->state;
			if (!drm_rect_clip_scaled(&s.src, &s.dst, &mixer_rect[mixercfg_id]))
				continue;

			s.dst.x1 -= mixer_rect[mixercfg_id].x1;
			s.dst.x2 -= mixer_rect[mixercfg_id].x1;

			/* mapping sspp to mixer stages */
			mixercfg_set_sspp_stage(&mixer_cfg[mixercfg_id], id, stage);

			/* sspp state */
			dpu_plane_update(dpu, id, &s, c->is_dual_dsi);
			ctl_flush |= CTL_FLUSH_SSPP(id);

			if (id >= 4)
				fetch_pipe_active |= BIT(id - 4);
			else
				fetch_pipe_active |= BIT(id + 16);
		}
	}
#else
	drm_atomic_crtc_for_each_plane(plane, crtc) {
		u32 stage = plane->state->normalized_zpos;

		// TODO: should be per mixer for dual HW planes
		mixer_op_mode |= 1 << (stage + 1); /* + 1 to skip base stage */

		for (i = c->lm; i < c->lm + num_mixers; i++)
			mixer_set_blendfunc(dpu, i, stage);

		for (i = 0; i < num_mixercfg; i++) {
			u32 id = to_dpu_plane(plane)->id * 4 + i;
			struct drm_plane_state s = *plane->state;
			if (!drm_rect_clip_scaled(&s.src, &s.dst, &mixer_rect[i]))
				continue;

			s.dst.x1 -= mixer_rect[i].x1;
			s.dst.x2 -= mixer_rect[i].x1;

			/* mapping sspp to mixer stages */
			mixercfg_set_sspp_stage(&mixer_cfg[i], id, stage);

			/* sspp state */
			dpu_plane_update(dpu, id, &s, false);
			ctl_flush |= CTL_FLUSH_SSPP(id);

			if (id >= 4)
				fetch_pipe_active |= BIT(id - 4);
			else
				fetch_pipe_active |= BIT(id + 16);
		}
	}
#endif

	// XXX skip this for CWB, doesn't use a mixer
	for (i = c->lm; i < c->lm + num_mixers; i++) {
	#ifdef SIXPLANEMODE
		u32 cfg_index = num_mixercfg > 1 ? (i >= c->lm + num_mixers/2) : 0;
		u32 op_mode = mixer_op_mode[cfg_index];
	#else
		u32 cfg_index = i - c->lm;//num_mixercfg > 1 ? (i >= c->lm + num_mixers/2) : 0;
		u32 op_mode = mixer_op_mode;
	#endif

		/* right mixer bit for paired mixers */
		if ((i & 1) && (num_mixers != num_mixercfg))
			op_mode |= BIT(31);

		dpu_write(dpu, LM_OUT_SIZE(i), mode->vdisplay << 16 | (mode->hdisplay / num_mixers));
		dpu_write(dpu, LM_OP_MODE(i), op_mode);

		ctl_set_mixercfg(dpu, c->ctl, i, &mixer_cfg[cfg_index]);

		ctl_flush |= CTL_FLUSH_LM(i);
	}

	if (num_mixers == 2 && !c->is_dp && !c->is_wb) {
		struct mixercfg cfg = {};
		ctl_set_mixercfg(dpu, c->ctl, c->lm + 2, &cfg);
		ctl_set_mixercfg(dpu, c->ctl, c->lm + 3, &cfg);

		dpu_write(dpu, LM_OUT_SIZE(c->lm + 2), 0);
		dpu_write(dpu, LM_OUT_SIZE(c->lm + 3), 0);
		dpu_write(dpu, LM_OP_MODE(c->lm + 2), 0);
		dpu_write(dpu, LM_OP_MODE(c->lm + 3), 0);

		ctl_flush |= CTL_FLUSH_LM(c->lm + 2);
		ctl_flush |= CTL_FLUSH_LM(c->lm + 3);

	}

	if (c->is_wb) {
		u32 i = (c == &dpu->crtc[3] ? 1 : 0);
		struct drm_connector_state *conn_state = dpu->writeback[i].base.state;
#ifdef SIXPLANEMODE
		bool is_cwb = (num_planes == 0 && mode->hdisplay == 2160 && mode->vdisplay == 2160);
#else
		bool is_cwb = true;
#endif

		if (is_cwb) {
			WARN_ON(!conn_state->writeback_job);
			conn_state->writeback_job->priv = (void*) (size_t) 1; // any non-zero value
			drm_writeback_queue_job(&dpu->writeback[i], conn_state);
		}

		if (!conn_state->writeback_job) { // note this includes cwb case
			spin_lock_irqsave(&dpu->dev->event_lock, flags);
			c->event = state->event;
			state->event = NULL;
			spin_unlock_irqrestore(&dpu->dev->event_lock, flags);

			dpu_complete_flip(dpu, c);
			return;
		}


		WARN_ON(!conn_state->writeback_job);

		dpu_write(dpu, WB_DST_WRITE_CONFIG(i), 0); // ubwc
		dpu_write(dpu, WB_DST_ADDR_SW_STATUS(i), 0); // secure
		dpu_write(dpu, WB_ALPHA_X_VALUE(i), 0xff);

		dpu_write(dpu, WB_DANGER_LUT(i), 0);
		dpu_write(dpu, WB_SAFE_LUT(i), 0xffff);
		dpu_write(dpu, WB_CREQ_LUT_0(i), 0);
		dpu_write(dpu, WB_CREQ_LUT_1(i), 0);
		dpu_write(dpu, WB_QOS_CTRL(i), 0);

		//printk("writeback %dx%d %d\n", mode->hdisplay, mode->vdisplay, !!conn_state->writeback_job->fb);

		wb_set_framebuffer(dpu, i, conn_state->writeback_job->fb, c->lm);

		dpu_write(dpu, WB_MUX(i), c->lm);
		dpu_write(dpu, CTL_WB_ACTIVE(c->ctl), BIT(1+i));
		dpu_write(dpu, CTL_WB_FLUSH(c->ctl), BIT(1+i));
		ctl_flush |= BIT(16); // flush WB
		if (i == 0) {
			dpu_write(dpu, CTL_CDM_ACTIVE(c->ctl), BIT(0));
			dpu_write(dpu, CTL_CDM_FLUSH(c->ctl), BIT(0));
			ctl_flush |= BIT(26); // flush CDM
		}

		drm_writeback_queue_job(&dpu->writeback[i], conn_state);
	} else if (state->mode_changed) {
		intf_timings(dpu, c->intf, mode, c->is_dual_dsi, c->is_dp, is_dsc);
		/* low 4 bits idx of pp block, or 0xf disable */
		dpu_write(dpu, INTF_MUX(c->intf), 0xF0000 | c->lm);
		intf_active = BIT(c->intf);
		if (c->is_dual_dsi) {
			intf_timings(dpu, c->intf + 1, mode, c->is_dual_dsi, c->is_dp, is_dsc);
			dpu_write(dpu, INTF_MUX(c->intf + 1), 0xF0000 | (c->lm + num_mixers/2)); // XXX
			intf_active |= BIT(c->intf + 1);
		}
		dpu_write(dpu, CTL_INTF_ACTIVE(c->ctl), intf_active);
		dpu_write(dpu, CTL_INTF_MASTER(c->ctl), BIT(c->intf));

		dpu_write(dpu, CTL_INTF_FLUSH(c->ctl), 0xf); /* bitmask of INTF modified */

		ctl_flush |= CTL_FLUSH_INTF;

		if (is_dsc) {
			set_dsc_config(dpu, 0, dpu->dsc);
			set_dsc_config(dpu, 1, dpu->dsc);
			set_dsc_config(dpu, 2, dpu->dsc);
			set_dsc_config(dpu, 3, dpu->dsc);

			dpu_write(dpu, CTL_DSC_ACTIVE(c->ctl), 0xf); /* bitmask of DSC active */
			dpu_write(dpu, CTL_DSC_FLUSH(c->ctl), 0xf); /* bitmask of DSC modified */
			ctl_flush |= CTL_FLUSH_DSC;
		} else if (!c->is_dp) {
			set_dsc_disable(dpu, 0);
			set_dsc_disable(dpu, 1);
			set_dsc_disable(dpu, 2);
			set_dsc_disable(dpu, 3);
			dpu_write(dpu, CTL_DSC_ACTIVE(c->ctl), 0);
			dpu_write(dpu, CTL_DSC_FLUSH(c->ctl), 0xf); /* bitmask of DSC modified */
			ctl_flush |= CTL_FLUSH_DSC;
		}

		/* always use merge3d for DP */
		if (c->is_dp) {
			dpu_write(dpu, MERGE_3D_MODE(c->lm / 2), num_mixers > 1 ? 3 : 0);
			dpu_write(dpu, MERGE_3D_MUX(c->lm / 2), 0);
			merge3d_active |= BIT(c->lm / 2);

			ctl_flush |= CTL_FLUSH_MERGE_3D;
		}
		dpu_write(dpu, CTL_MERGE_3D_ACTIVE(c->ctl), merge3d_active);
		dpu_write(dpu, CTL_MERGE_3D_FLUSH(c->ctl), 0x7); /* bitmask of MERGE3D flushed */
	}

	for (i = 0; i < 2; i++) {
		if (c->is_dual_dsi && !list_empty(&dpu->writeback[i].job_queue)) {
			// check for a pending CWB job
			struct drm_writeback_job *job = list_first_entry(&dpu->writeback[i].job_queue, struct drm_writeback_job, list_entry);
			// non-NULL "priv" indicates CWB job
			if (!job->priv) {
				// also check the next entry (required not to skip frame)
				if (list_is_last(&job->list_entry, &dpu->writeback[i].job_queue))
					goto not_cwb;

				job = list_next_entry(job, list_entry);
				if (!job->priv)
					goto not_cwb;
			}

			dpu_write(dpu, WB_DST_WRITE_CONFIG(i), 0); // ubwc
			dpu_write(dpu, WB_DST_ADDR_SW_STATUS(i), 0); // secure
			dpu_write(dpu, WB_ALPHA_X_VALUE(i), 0xff);

			dpu_write(dpu, WB_DANGER_LUT(i), 0);
			dpu_write(dpu, WB_SAFE_LUT(i), 0xffff);
			dpu_write(dpu, WB_CREQ_LUT_0(i), 0);
			dpu_write(dpu, WB_CREQ_LUT_1(i), 0);
			dpu_write(dpu, WB_QOS_CTRL(i), 0);

			wb_set_framebuffer(dpu, i, job->fb, i ? 0xb : 0xd);

			dpu_write(dpu, WB_MUX(i), i ? 0xb : 0xd); // DCWB
			dpu_write(dpu, CTL_WB_ACTIVE(c->ctl), BIT(1+i));
			dpu_write(dpu, CTL_WB_FLUSH(c->ctl), BIT(1+i));
			ctl_flush |= BIT(16); // flush WB
			if (i == 0) {
				dpu_write(dpu, CTL_CDM_ACTIVE(c->ctl), BIT(0));
				dpu_write(dpu, CTL_CDM_FLUSH(c->ctl), BIT(0));
				ctl_flush |= BIT(26); // flush CDM
			}

			dpu_write(dpu, MERGE_3D_CWB_MODE(i), 3);
			dpu_write(dpu, MERGE_3D_CWB_MUX(i), 0);

			dpu_write(dpu, CWB_MUX(i, 0), c->lm + i*2);
			dpu_write(dpu, CWB_MODE(i, 0), 0); // capture mixer output (see sde_crtc_output_capture_point)

			dpu_write(dpu, CWB_MUX(i, 1), c->lm + i*2+1);
			dpu_write(dpu, CWB_MODE(i, 1), 0);

			dpu_write(dpu, CTL_CWB_ACTIVE(c->ctl), 3 << (i*2));
			dpu_write(dpu, CTL_CWB_FLUSH(c->ctl), 3 << (i*2));
			ctl_flush |= BIT(28); // flush CWB

			job->priv = NULL;
		not_cwb:;
		}
	}

	dpu_write(dpu, CTL_FETCH_PIPE_ACTIVE(c->ctl), fetch_pipe_active);
	dpu_write(dpu, CTL_TOP(c->ctl), 0xf0000000); /* BIT(17) for command mode */

	dpu_toggle_start_irq(dpu, c->ctl, true);
	dpu_write(dpu, CTL_FLUSH(c->ctl), ctl_flush);

	if (c->is_wb)
		dpu_write(dpu, CTL_START(c->ctl), 1);


	// XXX race condition with irq??
	WARN_ON(c->event);

	spin_lock_irqsave(&dpu->dev->event_lock, flags);
	c->event = state->event;
	state->event = NULL;
	spin_unlock_irqrestore(&dpu->dev->event_lock, flags);
}

static const struct drm_crtc_funcs dpu_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = msm_crtc_enable_vblank,
	.disable_vblank = msm_crtc_disable_vblank,
};

static const struct drm_crtc_helper_funcs dpu_crtc_helper_funcs = {
	.atomic_disable = dpu_crtc_disable,
	.atomic_enable = dpu_crtc_enable,
	.atomic_check = dpu_crtc_atomic_check,
	.atomic_begin = dpu_crtc_atomic_begin,
	.atomic_flush = dpu_crtc_atomic_flush,
};

int dpu_crtc_init(struct dpu_kms *dpu, struct drm_crtc *crtc, struct drm_plane *plane)
{
	drm_crtc_init_with_planes(dpu->dev, crtc, plane, NULL, &dpu_crtc_funcs, NULL);
	drm_crtc_helper_add(crtc, &dpu_crtc_helper_funcs);

	return 0;
}

