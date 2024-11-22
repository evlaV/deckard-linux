// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt)	"[drm:%s] " fmt, __func__
#include "dpu_kms.h"
#include "dpu_hw_lm.h"
#include "dpu_hw_ctl.h"
#include "dpu_hw_cdm.h"
#include "dpu_hw_pingpong.h"
#include "dpu_hw_sspp.h"
#include "dpu_hw_intf.h"
#include "dpu_hw_wb.h"
#include "dpu_hw_dspp.h"
#include "dpu_hw_merge3d.h"
#include "dpu_hw_dsc.h"
#include "dpu_encoder.h"
#include "dpu_trace.h"


static inline bool reserved_by_other(uint32_t *res_map, int idx,
				     uint32_t crtc_id)
{
	return res_map[idx] && res_map[idx] != crtc_id;
}

/**
 * struct dpu_rm_requirements - Reservation requirements parameter bundle
 * @topology:  selected topology for the display
 */
struct dpu_rm_requirements {
	struct msm_display_topology topology;
};

int dpu_rm_init(struct drm_device *dev,
		struct dpu_rm *rm,
		const struct dpu_mdss_cfg *cat,
		const struct msm_mdss_data *mdss_data,
		void __iomem *mmio)
{
	int rc, i;

	if (!rm || !cat || !mmio) {
		DPU_ERROR("invalid kms\n");
		return -EINVAL;
	}

	/* Clear, setup lists */
	memset(rm, 0, sizeof(*rm));

	/* Interrogate HW catalog and create tracking items for hw blocks */
	for (i = 0; i < cat->mixer_count; i++) {
		struct dpu_hw_mixer *hw;
		const struct dpu_lm_cfg *lm = &cat->mixer[i];

		hw = dpu_hw_lm_init(dev, lm, mmio);
		if (IS_ERR(hw)) {
			rc = PTR_ERR(hw);
			DPU_ERROR("failed lm object creation: err %d\n", rc);
			goto fail;
		}
		rm->mixer_blks[lm->id - LM_0] = &hw->base;
	}

	for (i = 0; i < cat->merge_3d_count; i++) {
		struct dpu_hw_merge_3d *hw;
		const struct dpu_merge_3d_cfg *merge_3d = &cat->merge_3d[i];

		hw = dpu_hw_merge_3d_init(dev, merge_3d, mmio);
		if (IS_ERR(hw)) {
			rc = PTR_ERR(hw);
			DPU_ERROR("failed merge_3d object creation: err %d\n",
				rc);
			goto fail;
		}
		rm->merge_3d_blks[merge_3d->id - MERGE_3D_0] = &hw->base;
	}

	for (i = 0; i < cat->pingpong_count; i++) {
		struct dpu_hw_pingpong *hw;
		const struct dpu_pingpong_cfg *pp = &cat->pingpong[i];

		hw = dpu_hw_pingpong_init(dev, pp, mmio, cat->mdss_ver);
		if (IS_ERR(hw)) {
			rc = PTR_ERR(hw);
			DPU_ERROR("failed pingpong object creation: err %d\n",
				rc);
			goto fail;
		}
		if (pp->merge_3d && pp->merge_3d < MERGE_3D_MAX)
			hw->merge_3d = to_dpu_hw_merge_3d(rm->merge_3d_blks[pp->merge_3d - MERGE_3D_0]);
		rm->pingpong_blks[pp->id - PINGPONG_0] = &hw->base;
	}

	for (i = 0; i < cat->intf_count; i++) {
		struct dpu_hw_intf *hw;
		const struct dpu_intf_cfg *intf = &cat->intf[i];

		hw = dpu_hw_intf_init(dev, intf, mmio, cat->mdss_ver);
		if (IS_ERR(hw)) {
			rc = PTR_ERR(hw);
			DPU_ERROR("failed intf object creation: err %d\n", rc);
			goto fail;
		}
		rm->hw_intf[intf->id - INTF_0] = hw;
	}

	for (i = 0; i < cat->wb_count; i++) {
		struct dpu_hw_wb *hw;
		const struct dpu_wb_cfg *wb = &cat->wb[i];

		hw = dpu_hw_wb_init(dev, wb, mmio, cat->mdss_ver);
		if (IS_ERR(hw)) {
			rc = PTR_ERR(hw);
			DPU_ERROR("failed wb object creation: err %d\n", rc);
			goto fail;
		}
		rm->hw_wb[wb->id - WB_0] = hw;
	}

	for (i = 0; i < cat->ctl_count; i++) {
		struct dpu_hw_ctl *hw;
		const struct dpu_ctl_cfg *ctl = &cat->ctl[i];

		hw = dpu_hw_ctl_init(dev, ctl, mmio, cat->mixer_count, cat->mixer);
		if (IS_ERR(hw)) {
			rc = PTR_ERR(hw);
			DPU_ERROR("failed ctl object creation: err %d\n", rc);
			goto fail;
		}
		rm->ctl_blks[ctl->id - CTL_0] = &hw->base;
	}

	rm->has_active_ctls = (cat->mdss_ver->core_major_ver >= 5);

	for (i = 0; i < cat->dspp_count; i++) {
		struct dpu_hw_dspp *hw;
		const struct dpu_dspp_cfg *dspp = &cat->dspp[i];

		hw = dpu_hw_dspp_init(dev, dspp, mmio);
		if (IS_ERR(hw)) {
			rc = PTR_ERR(hw);
			DPU_ERROR("failed dspp object creation: err %d\n", rc);
			goto fail;
		}
		rm->dspp_blks[dspp->id - DSPP_0] = &hw->base;
	}

	for (i = 0; i < cat->dsc_count; i++) {
		struct dpu_hw_dsc *hw;
		const struct dpu_dsc_cfg *dsc = &cat->dsc[i];

		if (test_bit(DPU_DSC_HW_REV_1_2, &dsc->features))
			hw = dpu_hw_dsc_init_1_2(dev, dsc, mmio);
		else
			hw = dpu_hw_dsc_init(dev, dsc, mmio);

		if (IS_ERR(hw)) {
			rc = PTR_ERR(hw);
			DPU_ERROR("failed dsc object creation: err %d\n", rc);
			goto fail;
		}
		rm->dsc_blks[dsc->id - DSC_0] = &hw->base;
	}

	for (i = 0; i < cat->sspp_count; i++) {
		struct dpu_hw_sspp *hw;
		const struct dpu_sspp_cfg *sspp = &cat->sspp[i];

		hw = dpu_hw_sspp_init(dev, sspp, mmio, mdss_data, cat->mdss_ver);
		if (IS_ERR(hw)) {
			rc = PTR_ERR(hw);
			DPU_ERROR("failed sspp object creation: err %d\n", rc);
			goto fail;
		}
		rm->hw_sspp[sspp->id - SSPP_NONE] = hw;
	}

	if (cat->cdm) {
		struct dpu_hw_cdm *hw;

		hw = dpu_hw_cdm_init(dev, cat->cdm, mmio, cat->mdss_ver);
		if (IS_ERR(hw)) {
			rc = PTR_ERR(hw);
			DPU_ERROR("failed cdm object creation: err %d\n", rc);
			goto fail;
		}
		rm->cdm_blk = &hw->base;
	}

	return 0;

fail:
	return rc ? rc : -EFAULT;
}

static bool _dpu_rm_needs_split_display(const struct msm_display_topology *top)
{
	return top->num_intf > 1;
}

/**
 * _dpu_rm_get_lm_peer - get the id of a mixer which is a peer of the primary
 * @rm: dpu resource manager handle
 * @primary_idx: index of primary mixer in rm->mixer_blks[]
 *
 * Returns: lm peer mixed id on success or %-EINVAL on error
 */
static int _dpu_rm_get_lm_peer(struct dpu_rm *rm, int primary_idx)
{
	const struct dpu_lm_cfg *prim_lm_cfg;

	prim_lm_cfg = to_dpu_hw_mixer(rm->mixer_blks[primary_idx])->cap;

	if (prim_lm_cfg->lm_pair >= LM_0 && prim_lm_cfg->lm_pair < LM_MAX)
		return prim_lm_cfg->lm_pair - LM_0;
	return -EINVAL;
}

/**
 * _dpu_rm_check_lm_and_get_connected_blks - check if proposed layer mixer meets
 *	proposed use case requirements, incl. hardwired dependent blocks like
 *	pingpong
 * @rm: dpu resource manager handle
 * @global_state: resources shared across multiple kms objects
 * @crtc_id: crtc id requesting for allocation
 * @lm_idx: index of proposed layer mixer in rm->mixer_blks[], function checks
 *      if lm, and all other hardwired blocks connected to the lm (pp) is
 *      available and appropriate
 * @pp_idx: output parameter, index of pingpong block attached to the layer
 *      mixer in rm->pingpong_blks[].
 * @dspp_idx: output parameter, index of dspp block attached to the layer
 *      mixer in rm->dspp_blks[].
 * @reqs: input parameter, rm requirements for HW blocks needed in the
 *      datapath.
 * Return: true if lm matches all requirements, false otherwise
 */
static bool _dpu_rm_check_lm_and_get_connected_blks(struct dpu_rm *rm,
		struct dpu_global_state *global_state,
		uint32_t crtc_id, int lm_idx, int *pp_idx, int *dspp_idx,
		struct dpu_rm_requirements *reqs)
{
	const struct dpu_lm_cfg *lm_cfg;
	int idx;

	/* Already reserved? */
	if (reserved_by_other(global_state->mixer_to_crtc_id, lm_idx, crtc_id)) {
		DPU_DEBUG("lm %d already reserved\n", lm_idx + LM_0);
		return false;
	}

	lm_cfg = to_dpu_hw_mixer(rm->mixer_blks[lm_idx])->cap;
	idx = lm_cfg->pingpong - PINGPONG_0;
	if (idx < 0 || idx >= ARRAY_SIZE(rm->pingpong_blks)) {
		DPU_ERROR("failed to get pp on lm %d\n", lm_cfg->pingpong);
		return false;
	}

	if (reserved_by_other(global_state->pingpong_to_crtc_id, idx, crtc_id)) {
		DPU_DEBUG("lm %d pp %d already reserved\n", lm_cfg->id,
				lm_cfg->pingpong);
		return false;
	}
	*pp_idx = idx;

	if (!reqs->topology.num_dspp)
		return true;

	idx = lm_cfg->dspp - DSPP_0;
	if (idx < 0 || idx >= ARRAY_SIZE(rm->dspp_blks)) {
		DPU_ERROR("failed to get dspp on lm %d\n", lm_cfg->dspp);
		return false;
	}

	if (reserved_by_other(global_state->dspp_to_crtc_id, idx, crtc_id)) {
		DPU_DEBUG("lm %d dspp %d already reserved\n", lm_cfg->id,
				lm_cfg->dspp);
		return false;
	}
	*dspp_idx = idx;

	return true;
}

static int _dpu_rm_reserve_lms(struct dpu_rm *rm,
			       struct dpu_global_state *global_state,
			       uint32_t crtc_id,
			       struct dpu_rm_requirements *reqs)

{
	int lm_idx[MAX_BLOCKS];
	int pp_idx[MAX_BLOCKS];
	int dspp_idx[MAX_BLOCKS] = {0};
	int i, lm_count = 0;

	if (!reqs->topology.num_lm) {
		DPU_ERROR("invalid number of lm: %d\n", reqs->topology.num_lm);
		return -EINVAL;
	}

	/* Find a primary mixer */
	for (i = 0; i < ARRAY_SIZE(rm->mixer_blks) &&
			lm_count < reqs->topology.num_lm; i++) {
		if (!rm->mixer_blks[i])
			continue;

		/*
		 * Clear the last bit to drop the previous primary mixer if
		 * fail to find its peer.
		 */
		lm_count &= 0xfe;
		lm_idx[lm_count] = i;

		if (!_dpu_rm_check_lm_and_get_connected_blks(rm, global_state,
				crtc_id, i, &pp_idx[lm_count],
				&dspp_idx[lm_count], reqs)) {
			continue;
		}

		++lm_count;

		/* Valid primary mixer found, find matching peers */
		if (lm_count < reqs->topology.num_lm) {
			int j = _dpu_rm_get_lm_peer(rm, i);

			/* ignore the peer if there is an error or if the peer was already processed */
			if (j < 0 || j < i)
				continue;

			if (!rm->mixer_blks[j])
				continue;

			if (!_dpu_rm_check_lm_and_get_connected_blks(rm,
					global_state, crtc_id, j,
					&pp_idx[lm_count], &dspp_idx[lm_count],
					reqs)) {
				continue;
			}

			lm_idx[lm_count] = j;
			++lm_count;
		}
	}

	if (lm_count != reqs->topology.num_lm) {
		DPU_DEBUG("unable to find appropriate mixers\n");
		return -ENAVAIL;
	}

	for (i = 0; i < lm_count; i++) {
		global_state->mixer_to_crtc_id[lm_idx[i]] = crtc_id;
		global_state->pingpong_to_crtc_id[pp_idx[i]] = crtc_id;
		global_state->dspp_to_crtc_id[dspp_idx[i]] =
			reqs->topology.num_dspp ? crtc_id : 0;

		trace_dpu_rm_reserve_lms(lm_idx[i] + LM_0, crtc_id,
					 pp_idx[i] + PINGPONG_0);

		DPU_DEBUG("reserve lm[%d]:%d, pp_idx[%d]:%d, dspp[%d]:%d for crtc_id %d\n",
			  i, lm_idx[i], i, pp_idx[i], i, dspp_idx[i], crtc_id);
	}

	return 0;
}

static int _dpu_rm_reserve_ctls(
		struct dpu_rm *rm,
		struct dpu_global_state *global_state,
		uint32_t crtc_id,
		const struct msm_display_topology *top)
{
	int ctl_idx[MAX_BLOCKS];
	int i = 0, j, num_ctls;
	bool needs_split_display;

	if (rm->has_active_ctls) {
		num_ctls = 1;
		needs_split_display = false;
	} else {
		/* each hw_intf needs its own hw_ctrl to program its control path */
		num_ctls = top->num_intf;

		needs_split_display = _dpu_rm_needs_split_display(top);
	}

	for (j = 0; j < ARRAY_SIZE(rm->ctl_blks); j++) {
		const struct dpu_hw_ctl *ctl;
		unsigned long features;
		bool has_split_display;

		if (!rm->ctl_blks[j])
			continue;
		if (reserved_by_other(global_state->ctl_to_crtc_id, j, crtc_id))
			continue;

		ctl = to_dpu_hw_ctl(rm->ctl_blks[j]);
		features = ctl->caps->features;
		has_split_display = BIT(DPU_CTL_SPLIT_DISPLAY) & features;

		DPU_DEBUG("ctl %d caps 0x%lX\n", j + CTL_0, features);

		if (!rm->has_active_ctls && needs_split_display != has_split_display)
			continue;

		ctl_idx[i] = j;
		DPU_DEBUG("ctl %d match\n", j + CTL_0);

		if (++i == num_ctls)
			break;

	}

	if (i != num_ctls)
		return -ENAVAIL;

	for (i = 0; i < ARRAY_SIZE(ctl_idx) && i < num_ctls; i++) {
		global_state->ctl_to_crtc_id[ctl_idx[i]] = crtc_id;
		trace_dpu_rm_reserve_ctls(i + CTL_0, crtc_id);
	}

	return 0;
}

static int _dpu_rm_pingpong_next_index(struct dpu_global_state *global_state,
				       int start,
				       uint32_t crtc_id)
{
	int i;

	for (i = start; i < (PINGPONG_MAX - PINGPONG_0); i++) {
		if (global_state->pingpong_to_crtc_id[i] == crtc_id)
			return i;
	}

	return -ENAVAIL;
}

static int _dpu_rm_pingpong_dsc_check(int dsc_idx, int pp_idx)
{
	/*
	 * DSC with even index must be used with the PINGPONG with even index
	 * DSC with odd index must be used with the PINGPONG with odd index
	 */
	if ((dsc_idx & 0x01) != (pp_idx & 0x01))
		return -ENAVAIL;

	return 0;
}

static int _dpu_rm_dsc_alloc(struct dpu_rm *rm,
			     struct dpu_global_state *global_state,
			     uint32_t crtc_id,
			     const struct msm_display_topology *top)
{
	int num_dsc = 0;
	int pp_idx = 0;
	int dsc_idx;
	int ret;

	for (dsc_idx = 0; dsc_idx < ARRAY_SIZE(rm->dsc_blks) &&
	     num_dsc < top->num_dsc; dsc_idx++) {
		if (!rm->dsc_blks[dsc_idx])
			continue;

		if (reserved_by_other(global_state->dsc_to_crtc_id, dsc_idx, crtc_id))
			continue;

		pp_idx = _dpu_rm_pingpong_next_index(global_state, pp_idx, crtc_id);
		if (pp_idx < 0)
			return -ENAVAIL;

		ret = _dpu_rm_pingpong_dsc_check(dsc_idx, pp_idx);
		if (ret)
			return -ENAVAIL;

		global_state->dsc_to_crtc_id[dsc_idx] = crtc_id;
		num_dsc++;
		pp_idx++;
	}

	if (num_dsc < top->num_dsc) {
		DPU_ERROR("DSC allocation failed num_dsc=%d required=%d\n",
			   num_dsc, top->num_dsc);
		return -ENAVAIL;
	}

	return 0;
}

static int _dpu_rm_dsc_alloc_pair(struct dpu_rm *rm,
				  struct dpu_global_state *global_state,
				  uint32_t crtc_id,
				  const struct msm_display_topology *top)
{
	int num_dsc = 0;
	int dsc_idx, pp_idx = 0;
	int ret;

	/* only start from even dsc index */
	for (dsc_idx = 0; dsc_idx < ARRAY_SIZE(rm->dsc_blks) &&
	     num_dsc < top->num_dsc; dsc_idx += 2) {
		if (!rm->dsc_blks[dsc_idx] ||
		    !rm->dsc_blks[dsc_idx + 1])
			continue;

		/* consective dsc index to be paired */
		if (reserved_by_other(global_state->dsc_to_crtc_id, dsc_idx, crtc_id) ||
		    reserved_by_other(global_state->dsc_to_crtc_id, dsc_idx + 1, crtc_id))
			continue;

		pp_idx = _dpu_rm_pingpong_next_index(global_state, pp_idx, crtc_id);
		if (pp_idx < 0)
			return -ENAVAIL;

		ret = _dpu_rm_pingpong_dsc_check(dsc_idx, pp_idx);
		if (ret) {
			pp_idx = 0;
			continue;
		}

		pp_idx = _dpu_rm_pingpong_next_index(global_state, pp_idx + 1, crtc_id);
		if (pp_idx < 0)
			return -ENAVAIL;

		ret = _dpu_rm_pingpong_dsc_check(dsc_idx + 1, pp_idx);
		if (ret) {
			pp_idx = 0;
			continue;
		}

		global_state->dsc_to_crtc_id[dsc_idx] = crtc_id;
		global_state->dsc_to_crtc_id[dsc_idx + 1] = crtc_id;
		num_dsc += 2;
		pp_idx++;	/* start for next pair */
	}

	if (num_dsc < top->num_dsc) {
		DPU_ERROR("DSC allocation failed num_dsc=%d required=%d\n",
			   num_dsc, top->num_dsc);
		return -ENAVAIL;
	}

	return 0;
}

static int _dpu_rm_reserve_dsc(struct dpu_rm *rm,
			       struct dpu_global_state *global_state,
			       uint32_t crtc_id,
			       const struct msm_display_topology *top)
{
	if (!top->num_dsc || !top->num_intf)
		return 0;

	/*
	 * Facts:
	 * 1) no pingpong split (two layer mixers shared one pingpong)
	 * 2) DSC pair starts from even index, such as index(0,1), (2,3), etc
	 * 3) even PINGPONG connects to even DSC
	 * 4) odd PINGPONG connects to odd DSC
	 * 5) pair: crtc    +--> pp_idx_0 --> dsc_idx_0
	 *                  +--> pp_idx_1 --> dsc_idx_1
	 */

	/* num_dsc should be either 1, 2 or 4 */
	if (top->num_dsc > top->num_intf)	/* merge mode */
		return _dpu_rm_dsc_alloc_pair(rm, global_state, crtc_id, top);
	else
		return _dpu_rm_dsc_alloc(rm, global_state, crtc_id, top);

	return 0;
}

static int _dpu_rm_reserve_cdm(struct dpu_rm *rm,
			       struct dpu_global_state *global_state,
			       uint32_t crtc_id)
{
	/* try allocating only one CDM block */
	if (!rm->cdm_blk) {
		DPU_ERROR("CDM block does not exist\n");
		return -EIO;
	}

	if (global_state->cdm_to_crtc_id) {
		DPU_ERROR("CDM_0 is already allocated\n");
		return -EIO;
	}

	global_state->cdm_to_crtc_id = crtc_id;

	return 0;
}

static int _dpu_rm_make_reservation(
		struct dpu_rm *rm,
		struct dpu_global_state *global_state,
		uint32_t crtc_id,
		struct dpu_rm_requirements *reqs)
{
	int ret;

	ret = _dpu_rm_reserve_lms(rm, global_state, crtc_id, reqs);
	if (ret) {
		DPU_ERROR("unable to find appropriate mixers\n");
		return ret;
	}

	ret = _dpu_rm_reserve_ctls(rm, global_state, crtc_id,
				&reqs->topology);
	if (ret) {
		DPU_ERROR("unable to find appropriate CTL\n");
		return ret;
	}

	ret  = _dpu_rm_reserve_dsc(rm, global_state, crtc_id, &reqs->topology);
	if (ret)
		return ret;

	if (reqs->topology.needs_cdm) {
		ret = _dpu_rm_reserve_cdm(rm, global_state, crtc_id);
		if (ret) {
			DPU_ERROR("unable to find CDM blk\n");
			return ret;
		}
	}

	return ret;
}

static int _dpu_rm_populate_requirements(
		struct drm_crtc *crtc,
		struct dpu_rm_requirements *reqs,
		struct msm_display_topology req_topology)
{
	reqs->topology = req_topology;

	DRM_DEBUG_KMS("num_lm: %d num_dsc: %d num_intf: %d cdm: %d\n",
		      reqs->topology.num_lm, reqs->topology.num_dsc,
		      reqs->topology.num_intf, reqs->topology.needs_cdm);

	return 0;
}

static void _dpu_rm_clear_mapping(uint32_t *res_mapping, int cnt,
				  uint32_t crtc_id)
{
	int i;

	for (i = 0; i < cnt; i++) {
		if (res_mapping[i] == crtc_id)
			res_mapping[i] = 0;
	}
}

void dpu_rm_release(struct dpu_global_state *global_state,
		    struct drm_crtc *crtc)
{
	uint32_t crtc_id = crtc->base.id;

	_dpu_rm_clear_mapping(global_state->pingpong_to_crtc_id,
		ARRAY_SIZE(global_state->pingpong_to_crtc_id), crtc_id);
	_dpu_rm_clear_mapping(global_state->mixer_to_crtc_id,
		ARRAY_SIZE(global_state->mixer_to_crtc_id), crtc_id);
	_dpu_rm_clear_mapping(global_state->ctl_to_crtc_id,
		ARRAY_SIZE(global_state->ctl_to_crtc_id), crtc_id);
	_dpu_rm_clear_mapping(global_state->dsc_to_crtc_id,
		ARRAY_SIZE(global_state->dsc_to_crtc_id), crtc_id);
	_dpu_rm_clear_mapping(global_state->dspp_to_crtc_id,
		ARRAY_SIZE(global_state->dspp_to_crtc_id), crtc_id);
}

int dpu_rm_reserve(
		struct dpu_rm *rm,
		struct dpu_global_state *global_state,
		struct drm_crtc *crtc,
		struct msm_display_topology topology)
{
	struct dpu_rm_requirements reqs;
	int ret;

	if (IS_ERR(global_state)) {
		DPU_ERROR("failed to global state\n");
		return PTR_ERR(global_state);
	}

	DRM_DEBUG_KMS("reserving hw for crtc %d\n", crtc->base.id);

	ret = _dpu_rm_populate_requirements(crtc, &reqs, topology);
	if (ret) {
		DPU_ERROR("failed to populate hw requirements\n");
		return ret;
	}

	ret = _dpu_rm_make_reservation(rm, global_state, crtc->base.id, &reqs);
	if (ret)
		DPU_ERROR("failed to reserve hw resources: %d\n", ret);

	return ret;
}

struct dpu_hw_sspp *dpu_rm_reserve_sspp(struct dpu_rm *rm,
					struct dpu_global_state *global_state,
					struct drm_crtc *crtc,
					struct dpu_rm_sspp_requirements *reqs)
{
	uint32_t crtc_id = crtc->base.id;
	unsigned int weight, best_weight = UINT_MAX;
	struct dpu_hw_sspp *hw_sspp;
	unsigned long mask = 0;
	int i, best_idx = -1;

	/*
	 * Don't take cursor feature into consideration until there is proper support for SSPP_CURSORn.
	 */
	mask |= BIT(DPU_SSPP_CURSOR);

	if (reqs->scale)
		mask |= BIT(DPU_SSPP_SCALER_RGB) |
			BIT(DPU_SSPP_SCALER_QSEED2) |
			BIT(DPU_SSPP_SCALER_QSEED3_COMPATIBLE);

	if (reqs->yuv)
		mask |= BIT(DPU_SSPP_CSC) |
			BIT(DPU_SSPP_CSC_10BIT);

	if (reqs->rot90)
		mask |= BIT(DPU_SSPP_INLINE_ROTATION);

	for (i = 0; i < ARRAY_SIZE(rm->hw_sspp); i++) {
		if (!rm->hw_sspp[i])
			continue;

		if (global_state->sspp_to_crtc_id[i])
			continue;

		hw_sspp = rm->hw_sspp[i];

		/* skip incompatible planes */
		if (reqs->scale && !hw_sspp->cap->sblk->scaler_blk.len)
			continue;

		if (reqs->yuv && !hw_sspp->cap->sblk->csc_blk.len)
			continue;

		if (reqs->rot90 && !(hw_sspp->cap->features & DPU_SSPP_INLINE_ROTATION))
			continue;

		/*
		 * For non-yuv, non-scaled planes prefer simple (DMA or RGB)
		 * plane, falling back to VIG only if there are no such planes.
		 *
		 * This way we'd leave VIG sspps to be later used for YUV formats.
		 */
		weight = hweight64(hw_sspp->cap->features & ~mask);
		if (weight < best_weight) {
			best_weight = weight;
			best_idx = i;
		}
	}

	if (best_idx < 0)
		return NULL;

	global_state->sspp_to_crtc_id[best_idx] = crtc_id;

	return rm->hw_sspp[best_idx];
}

int dpu_rm_reserve_cac_sspp(struct dpu_kms *dpu_kms)
{
	struct dpu_global_state *global_state = dpu_kms_get_existing_global_state(dpu_kms);
	const struct dpu_rm *rm = global_state->rm;
	int i;

	/* RECT0/RECT1 are used for below hidden SSPPs in planar CAC mode */
	/* RECT1 for blue in VIG0~4 are not used due to content is combined with red */
	/* VIG0/2 are for the lm0/1 and VIG1/3 are for lm2/3 */
	/* DMA2 are for the lm0/1 and DMA3 are for lm2/3 */
	u32 csspp[] = {SSPP_VIG0, SSPP_VIG1, SSPP_VIG2, SSPP_VIG3, SSPP_DMA2, SSPP_DMA3, SSPP_DMA0, SSPP_DMA1};

printk("========== reserving CAC hidden SSPP done\n");
	for (i = 0; i < PLANAR_CAC_SSPP; i++) {
		global_state->sspp_to_crtc_id[csspp[i] - SSPP_NONE] = HIDEN_CAC_SSPP_CRTC;
		dpu_kms->cac_pipes[i * 2].sspp = rm->hw_sspp[csspp[i] - SSPP_NONE];
		dpu_kms->cac_pipes[i * 2 + 1].sspp = rm->hw_sspp[csspp[i] - SSPP_NONE];
printk("set SSPP ptr %p\n", rm->hw_sspp[csspp[i] - SSPP_NONE]);
	}
	for (i = PLANAR_CAC_SSPP; i < PLANAR_CAC_SSPP_WITH_FETCH; i++) {
		dpu_kms->cac_pipes[i * 2].sspp = rm->hw_sspp[csspp[i] - SSPP_NONE];
		dpu_kms->cac_pipes[i * 2 + 1].sspp = rm->hw_sspp[csspp[i] - SSPP_NONE];
	}

	return 0;
}

void dpu_rm_release_all_sspp(struct dpu_global_state *global_state,
			     struct drm_crtc *crtc)
{
	uint32_t crtc_id = crtc->base.id;

	_dpu_rm_clear_mapping(global_state->sspp_to_crtc_id,
		ARRAY_SIZE(global_state->sspp_to_crtc_id), crtc_id);
}

static char *dpu_hw_blk_type_name[] = {
	[DPU_HW_BLK_TOP] = "TOP",
	[DPU_HW_BLK_SSPP] = "SSPP",
	[DPU_HW_BLK_LM] = "LM",
	[DPU_HW_BLK_CTL] = "CTL",
	[DPU_HW_BLK_PINGPONG] = "pingpong",
	[DPU_HW_BLK_INTF] = "INTF",
	[DPU_HW_BLK_WB] = "WB",
	[DPU_HW_BLK_DSPP] = "DSPP",
	[DPU_HW_BLK_MERGE_3D] = "merge_3d",
	[DPU_HW_BLK_DSC] = "DSC",
	[DPU_HW_BLK_CDM] = "CDM",
	[DPU_HW_BLK_MAX] = "none",
};

int dpu_rm_get_assigned_resources(struct dpu_rm *rm,
	struct dpu_global_state *global_state, struct drm_crtc *crtc,
	enum dpu_hw_blk_type type, struct dpu_hw_blk **blks, int blks_size)
{
	uint32_t crtc_id = crtc->base.id;
	struct dpu_hw_blk **hw_blks;
	uint32_t *hw_to_crtc_id;
	int i, num_blks, max_blks;

	switch (type) {
	case DPU_HW_BLK_PINGPONG:
		hw_blks = rm->pingpong_blks;
		hw_to_crtc_id = global_state->pingpong_to_crtc_id;
		max_blks = ARRAY_SIZE(rm->pingpong_blks);
		break;
	case DPU_HW_BLK_LM:
		hw_blks = rm->mixer_blks;
		hw_to_crtc_id = global_state->mixer_to_crtc_id;
		max_blks = ARRAY_SIZE(rm->mixer_blks);
		break;
	case DPU_HW_BLK_CTL:
		hw_blks = rm->ctl_blks;
		hw_to_crtc_id = global_state->ctl_to_crtc_id;
		max_blks = ARRAY_SIZE(rm->ctl_blks);
		break;
	case DPU_HW_BLK_DSPP:
		hw_blks = rm->dspp_blks;
		hw_to_crtc_id = global_state->dspp_to_crtc_id;
		max_blks = ARRAY_SIZE(rm->dspp_blks);
		break;
	case DPU_HW_BLK_DSC:
		hw_blks = rm->dsc_blks;
		hw_to_crtc_id = global_state->dsc_to_crtc_id;
		max_blks = ARRAY_SIZE(rm->dsc_blks);
		break;
	case DPU_HW_BLK_CDM:
		hw_blks = &rm->cdm_blk;
		hw_to_crtc_id = &global_state->cdm_to_crtc_id;
		max_blks = 1;
		break;
	default:
		DPU_ERROR("blk type %d not managed by rm\n", type);
		return 0;
	}

	num_blks = 0;
	for (i = 0; i < max_blks; i++) {
		if (hw_to_crtc_id[i] != crtc_id)
			continue;

		if (num_blks == blks_size) {
			DPU_ERROR("More than %d resources assigned to crtc %d\n",
				  blks_size, crtc_id);
			break;
		}
		if (!hw_blks[i]) {
			DPU_ERROR("%s unavailable to assign to crtc %d\n",
				  dpu_hw_blk_type_name[type], crtc_id);
			break;
		}
		blks[num_blks++] = hw_blks[i];
	}

	return num_blks;
}

static void dpu_rm_print_state_helper(struct drm_printer *p,
					    struct dpu_hw_blk *blk,
					    uint32_t mapping)
{
	if (!blk)
		drm_puts(p, "- ");
	else if (!mapping)
		drm_puts(p, "# ");
	else
		drm_printf(p, "%d ", mapping);
}


void dpu_rm_print_state(struct drm_printer *p,
			const struct dpu_global_state *global_state)
{
	const struct dpu_rm *rm = global_state->rm;
	int i;

	drm_puts(p, "resource mapping:\n");
	drm_puts(p, "\tpingpong=");
	for (i = 0; i < ARRAY_SIZE(global_state->pingpong_to_crtc_id); i++)
		dpu_rm_print_state_helper(p, rm->pingpong_blks[i],
					  global_state->pingpong_to_crtc_id[i]);
	drm_puts(p, "\n");

	drm_puts(p, "\tmixer=");
	for (i = 0; i < ARRAY_SIZE(global_state->mixer_to_crtc_id); i++)
		dpu_rm_print_state_helper(p, rm->mixer_blks[i],
					  global_state->mixer_to_crtc_id[i]);
	drm_puts(p, "\n");

	drm_puts(p, "\tctl=");
	for (i = 0; i < ARRAY_SIZE(global_state->ctl_to_crtc_id); i++)
		dpu_rm_print_state_helper(p, rm->ctl_blks[i],
					  global_state->ctl_to_crtc_id[i]);
	drm_puts(p, "\n");

	drm_puts(p, "\tdspp=");
	for (i = 0; i < ARRAY_SIZE(global_state->dspp_to_crtc_id); i++)
		dpu_rm_print_state_helper(p, rm->dspp_blks[i],
					  global_state->dspp_to_crtc_id[i]);
	drm_puts(p, "\n");

	drm_puts(p, "\tdsc=");
	for (i = 0; i < ARRAY_SIZE(global_state->dsc_to_crtc_id); i++)
		dpu_rm_print_state_helper(p, rm->dsc_blks[i],
					  global_state->dsc_to_crtc_id[i]);
	drm_puts(p, "\n");

	drm_puts(p, "\tcdm=");
	dpu_rm_print_state_helper(p, rm->cdm_blk,
				  global_state->cdm_to_crtc_id);
	drm_puts(p, "\n");

	drm_puts(p, "\tsspp=");
	/* skip SSPP_NONE and start from the next index */
	for (i = SSPP_NONE + 1; i < ARRAY_SIZE(global_state->sspp_to_crtc_id); i++)
		dpu_rm_print_state_helper(p, rm->hw_sspp[i] ? &rm->hw_sspp[i]->base : NULL,
					  global_state->sspp_to_crtc_id[i]);
	drm_puts(p, "\n");
}
