// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include "dpu_common.h"

#include <drm/drm_fourcc.h>

#include <linux/of_irq.h>
#include <linux/delay.h>

static void
dpu_encoder_mode_set(struct drm_encoder *encoder,
		     struct drm_display_mode *mode,
		     struct drm_display_mode *adj_mode)
{
	struct msm_drm_private *priv = encoder->dev->dev_private;

	if (encoder->encoder_type == DRM_MODE_ENCODER_TMDS)
		msm_dp_display_mode_set(priv->dp[0], encoder, mode, adj_mode);
}

static void
dpu_encoder_enable(struct drm_encoder *encoder)
{
	struct msm_drm_private *priv = encoder->dev->dev_private;

	if (encoder->encoder_type == DRM_MODE_ENCODER_VIRTUAL) {
	} else if (encoder->encoder_type == DRM_MODE_ENCODER_TMDS) {
		msleep(100); /* XXX avoids broken state after changing DP mode */
		dpu_write(to_dpu_kms(priv->kms), INTF_TIMING_ENGINE_EN(0), 1);
	} else {
		dpu_write(to_dpu_kms(priv->kms), INTF_TIMING_ENGINE_EN(1), 1);
	}
}

static void
dpu_encoder_disable(struct drm_encoder *encoder)
{
	struct msm_drm_private *priv = encoder->dev->dev_private;

	if (encoder->encoder_type == DRM_MODE_ENCODER_VIRTUAL) {
	} else if (encoder->encoder_type == DRM_MODE_ENCODER_TMDS) {
		dpu_write(to_dpu_kms(priv->kms), INTF_TIMING_ENGINE_EN(0), 0);
	} else {
		dpu_write(to_dpu_kms(priv->kms), INTF_TIMING_ENGINE_EN(1), 0);
	}
}

static const struct drm_encoder_helper_funcs dpu_encoder_helper_funcs = {
	.mode_set = dpu_encoder_mode_set,
	.disable = dpu_encoder_disable,
	.enable = dpu_encoder_enable,
};

static const struct drm_encoder_funcs dpu_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int
dpu_encoder_init(struct dpu_kms *dpu, struct drm_encoder *encoder, int type)
{
	int ret;

	ret = drm_encoder_init(dpu->dev, encoder, &dpu_encoder_funcs, type, NULL);
	if (ret)
		return ret;

	drm_encoder_helper_add(encoder, &dpu_encoder_helper_funcs);
	return 0;
}

static int dpu_kms_mmu_init(struct dpu_kms *dpu_kms)
{
	struct msm_gem_address_space *aspace;
	struct msm_mmu *mmu;

	mmu = msm_iommu_new(dpu_kms->dev->dev->parent, 0);
	aspace = msm_gem_address_space_create(mmu, "dpu1", 0x1000, 0xfffffff);
	if (IS_ERR(aspace))
		return PTR_ERR(aspace);

	dpu_kms->base.aspace = aspace;
	return 0;
}

static const u32 writeback_formats[] = {
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

static const struct drm_connector_funcs wb_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int wb_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;

	return drm_add_modes_noedid(connector, 2560, 2560);
}

static int wb_prepare_job(struct drm_writeback_connector *connector, struct drm_writeback_job *job)
{
	struct msm_drm_private *priv = connector->base.dev->dev_private;
	struct dpu_kms *dpu = to_dpu_kms(priv->kms);
	int ret;

	if (!job->fb)
		return 0;

	ret = msm_framebuffer_prepare(job->fb, dpu->base.aspace, false);
	if (ret) {
		DRM_ERROR("failed to prepare framebuffer\n");
		return ret;
	}

	return 0;
}

static void wb_cleanup_job(struct drm_writeback_connector *connector, struct drm_writeback_job *job)
{
	struct msm_drm_private *priv = connector->base.dev->dev_private;
	struct dpu_kms *dpu = to_dpu_kms(priv->kms);

	if (!job->fb)
		return;

	msm_framebuffer_cleanup(job->fb, dpu->base.aspace, false);
}

static void wb_atomic_commit(struct drm_connector *conn, struct drm_atomic_state *state)
{
}

static const struct drm_connector_helper_funcs wb_conn_helper_funcs = {
	.get_modes = wb_connector_get_modes,
	.prepare_writeback_job = wb_prepare_job,
	.cleanup_writeback_job = wb_cleanup_job,
	.atomic_commit = wb_atomic_commit,
};

static int dpu_kms_hw_init(struct msm_kms *kms)
{
	struct dpu_kms *dpu = to_dpu_kms(kms);
	struct drm_device *dev;
	struct msm_drm_private *priv;
	int i, rc = -EINVAL;

	dev = dpu->dev;
	priv = dev->dev_private;

	dpu->mmio = devm_platform_ioremap_resource_byname(dpu->pdev, "mdp");
	if (IS_ERR(dpu->mmio)) {
		rc = PTR_ERR(dpu->mmio);
		DRM_ERROR("mdp register memory map failed: %d\n", rc);
		dpu->mmio = NULL;
		goto error;
	}

	dpu->vbif = devm_platform_ioremap_resource_byname(dpu->pdev, "vbif");
	if (IS_ERR(dpu->vbif)) {
		rc = PTR_ERR(dpu->vbif);
		DRM_ERROR("vbif register memory map failed: %d\n", rc);
		dpu->vbif = NULL;
		goto error;
	}

	dpu->core_rev = readl_relaxed(dpu->mmio + 0x0);

	printk("dpu->core_rev=%.8x\n", dpu->core_rev);

	dpu->num_blendstages = 11;
	dpu->hbb = 3; /* 2 for DDR4 */
	dpu->ubwc_version = 4;
	dpu->num_sspp = 8;
	dpu->qos_enable = 0;
	dpu->qseed_filter = 0x03; //FILTER_BILINEAR
	dpu->cwb_plane_select = 0;

	dpu->creq_lut_linear = 0x0011222222335777;
	dpu->creq_lut_macrotile = 0x0011223344556677;

	rc = dpu_kms_mmu_init(dpu);
	if (rc) {
		DRM_ERROR("dpu_kms_mmu_init failed: %d\n", rc);
		goto power_error;
	}

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 2560 * 2; // or 4096 max plane width??
	dev->mode_config.max_height = 4096;
	//dev->mode_config.allow_fb_modifiers = true;

	rc = dpu_encoder_init(dpu, &dpu->encoder[0], DRM_MODE_ENCODER_DSI);
	if (rc) {
		DRM_ERROR("encoder init failed, rc = %d\n", rc);
		return rc;
	}

	rc = dpu_encoder_init(dpu, &dpu->encoder[1], DRM_MODE_ENCODER_TMDS);
	if (rc) {
		DRM_ERROR("encoder init failed, rc = %d\n", rc);
		return rc;
	}

	for (i = 0; i < 2; i++) {
		drm_connector_helper_add(&dpu->writeback[i].base, &wb_conn_helper_funcs);

		rc = drm_writeback_connector_init(dev, &dpu->writeback[i], &wb_connector_funcs,
			&dpu_encoder_helper_funcs, writeback_formats, ARRAY_SIZE(writeback_formats), BIT(2+i));
		if (rc) {
			DRM_ERROR("writeback init failed, rc = %d\n", rc);
			return rc;
		}
	}

	for (i = 0; i < ARRAY_SIZE(priv->dsi); i++) {
		if (!priv->dsi[i])
			continue;

		// assume same DSC config for both panels
		if (i == 0)
			dpu->dsc = msm_dsi_get_dsc_config(priv->dsi[i]);

		rc = msm_dsi_modeset_init(priv->dsi[i], dev, &dpu->encoder[0]);
		if (rc) {
			DRM_ERROR("modeset_init failed for dsi[%d], rc = %d\n",
				i, rc);
			return rc; // XXX proper free path
		}
	}

	rc = msm_dp_modeset_init(priv->dp[0], dev, &dpu->encoder[1]);
	if (rc) {
		DRM_ERROR("modeset_init failed for DP, rc = %d\n", rc);
		return rc; // XXX proper free path
	}

	for (i = 0; i < 2; i++) {
		rc = dpu_plane_init(dpu, &dpu->plane[i].base, i ? 0xe : BIT(0));
		if (rc) {
			DRM_ERROR("plane init failed, rc = %d\n", rc);
			return rc; // XXX proper free path
		}
		dpu->plane[i].id = i;
	}

	for (i = 0; i < 4; i++) {
		rc = dpu_crtc_init(dpu, &dpu->crtc[i].base, &dpu->plane[i ? 1 : 0].base);
		if (rc) {
			DRM_ERROR("crtc init failed, rc = %d\n", rc);
			return rc;
		}
	}

	dpu->encoder[0].possible_crtcs |= 1;
	dpu->encoder[1].possible_crtcs |= 2;

	init_waitqueue_head(&dpu->crtc[0].pending_kickoff_wq);
	init_waitqueue_head(&dpu->crtc[1].pending_kickoff_wq);
	init_waitqueue_head(&dpu->crtc[2].pending_kickoff_wq);
	init_waitqueue_head(&dpu->crtc[3].pending_kickoff_wq);

	vbif_write(dpu, VBIF_OUT_AXI_AMEMTYPE_CONF0, 0x33333333);
	vbif_write(dpu, VBIF_OUT_AXI_AMEMTYPE_CONF1, 0x33333333);

	dpu->intr_en = INTF_WB_1_DONE | INTF_WB_2_DONE;
	dpu->intr2_en = 0;//INTR2_CTL_START(0) | INTR2_CTL_START(2);

	spin_lock_init(&dpu->irq_lock);

	dpu_write(dpu, INTR2_EN, dpu->intr2_en);
	dpu_write(dpu, INTR_EN, dpu->intr_en);

	dpu_write(dpu, INTR_CLEAR, ~0u);
	dpu_write(dpu, INTR2_CLEAR, ~0u);

	dpu_set_static_regs(dpu);

	{
		dpu->cac_prop = drm_property_create(dev, DRM_MODE_PROP_BLOB, "CAC_BLOB", 0);
		WARN_ON(!dpu->cac_prop);
		drm_object_attach_property(&dpu->crtc[0].base.base, dpu->cac_prop, -1);
		dpu->cac_blob_id = -1;
	}

	//reset_ubwc(priv);

	/* assign resources */
	dpu->crtc[0].ctl = 0;
	dpu->crtc[0].intf = 1;
	dpu->crtc[0].lm = 0;
	dpu->crtc[0].is_dual_dsi = msm_dsi_is_bonded_dsi(priv->dsi[0]);

	dpu->crtc[1].ctl = 2;
	dpu->crtc[1].intf = 0;
	dpu->crtc[1].lm = 4;
	dpu->crtc[1].is_dp = true;

	dpu->crtc[2].ctl = 4;
	dpu->crtc[2].intf = 0xff;
	dpu->crtc[2].lm = 4;
	dpu->crtc[2].is_wb = true;

	dpu->crtc[3].ctl = 5;
	dpu->crtc[3].intf = 0xff;
	dpu->crtc[3].lm = 5;
	dpu->crtc[3].is_wb = true;

	/* used by msm_drv.c */
	priv->num_crtcs = 4;

	//pm_runtime_put_sync(&dpu_kms->pdev->dev);

	printk("done gpu init\n");

	return 0;

power_error:
error:
	//_dpu_kms_hw_destroy(dpu);

	return rc;
}

static void dpu_irq_preinstall(struct msm_kms *kms)
{
}

static void dpu_irq_uninstall(struct msm_kms *kms)
{
}

void dpu_complete_flip(struct dpu_kms *dpu, struct dpu_crtc *c)
{
	unsigned long flags;

	spin_lock_irqsave(&dpu->dev->event_lock, flags);
	if (c->event) {
		drm_crtc_send_vblank_event(&c->base, c->event);
		c->event = NULL;
	}
	spin_unlock_irqrestore(&dpu->dev->event_lock, flags);
}

void dpu_vblank(struct dpu_kms *dpu, struct dpu_crtc *c)
{
	drm_crtc_handle_vblank(&c->base);
}

static irqreturn_t dpu_irq(struct msm_kms *kms)
{
	struct dpu_kms *dpu = to_dpu_kms(kms);
	u32 status1, status2;

	status1 = dpu_read(dpu, INTR_STATUS);
	status2 = dpu_read(dpu, INTR2_STATUS);
	dpu_write(dpu, INTR_CLEAR, status1);
	dpu_write(dpu, INTR2_CLEAR, status2);

	//printk("dpu_irq: %.8x %.8x\n", status1, status2);

	if (status1 & INTF_VSYNC(0)) {
		dpu_vblank(dpu, &dpu->crtc[1]);
	}

	if (status1 & INTF_VSYNC(1)) {
		dpu_vblank(dpu, &dpu->crtc[0]);
	}

	if (status1 & INTF_WB_1_DONE) {
		//printk("WB_1 done\n");
		drm_writeback_signal_completion(&dpu->writeback[0], 0);
	}

	if (status1 & INTF_WB_2_DONE) {
		//printk("WB_2 done\n");
		drm_writeback_signal_completion(&dpu->writeback[1], 0);
	}

	if (status2 & INTR2_CTL_START(0)) {
		wake_up_all(&dpu->crtc[0].pending_kickoff_wq);
		dpu_toggle_start_irq(dpu, 0, false);
		dpu_complete_flip(dpu, &dpu->crtc[0]);
	}

	if (status2 & INTR2_CTL_START(2)) {
		wake_up_all(&dpu->crtc[1].pending_kickoff_wq);
		dpu_toggle_start_irq(dpu, 2, false);
		dpu_complete_flip(dpu, &dpu->crtc[1]);
	}

	if (status2 & INTR2_CTL_START(4)) {
		wake_up_all(&dpu->crtc[2].pending_kickoff_wq);
		dpu_toggle_start_irq(dpu, 4, false);
		dpu_complete_flip(dpu, &dpu->crtc[2]);
	}

	if (status2 & INTR2_CTL_START(5)) {
		wake_up_all(&dpu->crtc[3].pending_kickoff_wq);
		dpu_toggle_start_irq(dpu, 5, false);
		dpu_complete_flip(dpu, &dpu->crtc[3]);
	}

	return IRQ_HANDLED;
}

static void dpu_kms_enable_commit(struct msm_kms *kms)
{
	/* nothing to do */
}

static void dpu_kms_disable_commit(struct msm_kms *kms)
{
	/* nothing to do */
}

static ktime_t
dpu_kms_vsync_time(struct msm_kms *kms, struct drm_crtc *crtc)
{
	struct dpu_kms *dpu = to_dpu_kms(kms);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct dpu_crtc *c = to_dpu_crtc(crtc);
	u64 tmp;
	u32 line;

	line = dpu_read(dpu, INTF_LINE_COUNT(c->intf));

	if (line >= mode->vtotal) {
		DRM_WARN("line (%d) > vtotal (%d)\n", line, mode->vtotal);
		line = 0;
	}

	tmp = (u64) mode->htotal * (mode->vtotal - line) * 1000000000;
	tmp = DIV_ROUND_UP_ULL(tmp, mode->clock * 1000);

	return ktime_add_ns(ktime_get(), tmp);
}

static void
dpu_kms_prepare_commit(struct msm_kms *kms, struct drm_atomic_state *state)
{
	/* nothing */
}

static void
dpu_kms_flush_commit(struct msm_kms *kms, unsigned crtc_mask)
{
	/* nothing */
}

static void dpu_kms_wait_flush(struct msm_kms *kms, unsigned crtc_mask)
{
	struct dpu_kms *dpu = to_dpu_kms(kms);
	struct drm_crtc *crtc;
	long ret;

	for_each_crtc_mask(dpu->dev, crtc, crtc_mask) {
		struct dpu_crtc *c = to_dpu_crtc(crtc);

		ret = wait_event_timeout(c->pending_kickoff_wq,
			(dpu_read(dpu, CTL_FLUSH(c->ctl)) == 0),
			msecs_to_jiffies(50));
		if (!ret) {
			DRM_ERROR("flush timeout\n");
			dpu_complete_flip(dpu, c); // XXX
			break;
		}
	}

	return;
}

static void dpu_kms_complete_commit(struct msm_kms *kms, unsigned crtc_mask)
{
	/* nothing to do */
}

void dpu_toggle_start_irq(struct dpu_kms *dpu, u32 ctl, bool enable)
{
	unsigned long flags;

	spin_lock_irqsave(&dpu->irq_lock, flags);

	if (enable) {
		dpu->intr2_en |= INTR2_CTL_START(ctl);
		dpu_write(dpu, INTR2_CLEAR, INTR2_CTL_START(ctl));
	} else
		dpu->intr2_en &= ~INTR2_CTL_START(ctl);
	dpu_write(dpu, INTR2_EN, dpu->intr2_en);

	spin_unlock_irqrestore(&dpu->irq_lock, flags);
}

static int dpu_kms_enable_vblank(struct msm_kms *kms, struct drm_crtc *crtc)
{
	struct dpu_kms *dpu = to_dpu_kms(kms);
	struct dpu_crtc *c = to_dpu_crtc(crtc);
	unsigned long flags;

	spin_lock_irqsave(&dpu->irq_lock, flags);

	dpu->intr_en |= INTF_VSYNC(c->intf);
	dpu_write(dpu, INTR_CLEAR, INTF_VSYNC(c->intf));
	dpu_write(dpu, INTR_EN, dpu->intr_en);

	spin_unlock_irqrestore(&dpu->irq_lock, flags);

	return 0;
}

static void dpu_kms_disable_vblank(struct msm_kms *kms, struct drm_crtc *crtc)
{
	struct dpu_kms *dpu = to_dpu_kms(kms);
	struct dpu_crtc *c = to_dpu_crtc(crtc);
	unsigned long flags;

	spin_lock_irqsave(&dpu->irq_lock, flags);

	dpu->intr_en &= ~INTF_VSYNC(c->intf);
	dpu_write(dpu, INTR_EN, dpu->intr_en);
	wmb();

	spin_unlock_irqrestore(&dpu->irq_lock, flags);
}

static const struct msm_format dummy_format;

static const struct msm_format *
dpu_get_msm_format(struct msm_kms *kms, uint32_t format, uint64_t modifier)
{
	/* return non-NULL to say format is supported */
	return &dummy_format;
}

static long
dpu_kms_round_pixclk(struct msm_kms *kms,
		     unsigned long rate,
		     struct drm_encoder *encoder)
{
	return rate;
}

static void dpu_kms_destroy(struct msm_kms *kms)
{
	DRM_ERROR("TODO KMS DESTROY");
}

static int dpu_kms_debugfs_init(struct msm_kms *kms, struct drm_minor *minor)
{
	return 0;
}

static const struct msm_kms_funcs kms_funcs = {
	.hw_init         = dpu_kms_hw_init,
	.irq_preinstall  = dpu_irq_preinstall,
	.irq_uninstall   = dpu_irq_uninstall,
	.irq             = dpu_irq,
	.enable_commit   = dpu_kms_enable_commit,
	.disable_commit  = dpu_kms_disable_commit,
	// .vsync_time      = dpu_kms_vsync_time, enable for async commit
	.prepare_commit  = dpu_kms_prepare_commit,
	.flush_commit    = dpu_kms_flush_commit,
	.wait_flush      = dpu_kms_wait_flush,
	.complete_commit = dpu_kms_complete_commit,
	.enable_vblank   = dpu_kms_enable_vblank,
	.disable_vblank  = dpu_kms_disable_vblank,
	.get_format      = dpu_get_msm_format,
	.round_pixclk    = dpu_kms_round_pixclk,
	.destroy         = dpu_kms_destroy,
#ifdef CONFIG_DEBUG_FS
	.debugfs_init    = dpu_kms_debugfs_init,
#endif
};

static int dpu_kms_init(struct drm_device *ddev)
{
	struct msm_drm_private *priv = ddev->dev_private;
	struct device *dev = ddev->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);
	int ret;

	ret = msm_kms_init(&dpu_kms->base, &kms_funcs);
	if (ret) {
		DRM_ERROR("failed to init kms, ret=%d\n", ret);
		return ret;
	}

	dpu_kms->dev = ddev;

	return 0;
}



static ssize_t cwb_plane_select_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_drm_private *priv = platform_get_drvdata(pdev);
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);

	if (dpu_kms) return sprintf(buf, "%d\n", dpu_kms->cwb_plane_select);
	else         return sprintf(buf, "error\n");
}

static ssize_t cwb_plane_select_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bool v;
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_drm_private *priv = platform_get_drvdata(pdev);
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);

	if (kstrtobool(buf, &v))
		return -EINVAL;

	if (dpu_kms) {
		printk("dpu2: cwb_plane_select = %d\n",v);
		dpu_kms->cwb_plane_select = v;
	} 
	else {
		return -ENODEV;
	}

	return count;
}

static DEVICE_ATTR_RW(cwb_plane_select);

static ssize_t qos_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_drm_private *priv = platform_get_drvdata(pdev);
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);

	if (dpu_kms) return sprintf(buf, "%d\n", dpu_kms->qos_enable);
	else         return sprintf(buf, "error\n");
}

static ssize_t qos_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	bool v;
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_drm_private *priv = platform_get_drvdata(pdev);
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);
	int i, pipe;

	if (kstrtobool(buf, &v))
		return -EINVAL;

	if (dpu_kms) {
		printk("dpu2: qos_enable = %d\n",v);
		dpu_kms->qos_enable = v;
		for (pipe = 0; pipe < 10; pipe++) {
			i = pipe + (pipe >= 4 ? 12 : 0); // index into register macros (extra offset for DMA pipes)
			dpu_write(dpu_kms, SSPP_QOS_CTRL(i), dpu_kms->qos_enable); // XXX
		}
	} 
	else {
		return -ENODEV;
	}

	return count;
}

static DEVICE_ATTR_RW(qos_enable);


static ssize_t qseed_filter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_drm_private *priv = platform_get_drvdata(pdev);
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);
	
	if (dpu_kms) return sprintf(buf, "%d\n", dpu_kms->qseed_filter);
	else         return sprintf(buf, "error\n");
}

static ssize_t qseed_filter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int v = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_drm_private *priv = platform_get_drvdata(pdev);
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);
	int i; 

	if (kstrtoint(buf, 0, &v))
		return -EINVAL;

	if (dpu_kms) {
		printk("dpu2: qseed_filter = %d\n",v);
		dpu_kms->qseed_filter = v;
		for (i = 0; i < 4; i++) {
			dpu_write(dpu_kms, QSEED3_OP_MODE(i), dpu_kms->qseed_filter << 24 | dpu_kms->qseed_filter << 16 | 0x4003); // BIT_WIDTH=1, CAC_MODE=1, EN=1,
		}
	} 
	else {
		return -ENODEV;
	}

	return count;
}

static DEVICE_ATTR_RW(qseed_filter);

static struct attribute *dpu_plane_attrs[] = {
	&dev_attr_qos_enable.attr,
	&dev_attr_qseed_filter.attr,
	&dev_attr_cwb_plane_select.attr,
	NULL,
};

static const struct attribute_group dpu_plane_attr_group = {
	.name = "dpu2",
	.attrs = dpu_plane_attrs,
};

static int dpu_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dpu_kms *dpu_kms;
	int irq;
	int ret;

	dpu_kms = devm_kzalloc(&pdev->dev, sizeof(*dpu_kms), GFP_KERNEL);
	if (!dpu_kms)
		return -ENOMEM;

	dpu_kms->pdev = pdev;

	ret = devm_clk_bulk_get_all(&pdev->dev, &dpu_kms->clocks);
	if (ret < 0) {
		DRM_ERROR("failed to parse clocks, ret=%d\n", ret);
		return ret;
	}
	dpu_kms->num_clocks = ret;

	clk_bulk_prepare_enable(dpu_kms->num_clocks, dpu_kms->clocks);

	ret = sysfs_create_group(&dev->kobj, &dpu_plane_attr_group);

	irq = irq_of_parse_and_map(dpu_kms->pdev->dev.of_node, 0);
	if (irq < 0) {
		DRM_ERROR("failed to get irq: %d\n", irq);
		return irq;
	}
	dpu_kms->base.irq = irq;

	return msm_drv_probe(&pdev->dev, dpu_kms_init, &dpu_kms->base);
}

static int dpu_dev_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &msm_drm_ops);

	return 0;
}

const struct of_device_id dpu_dt_match[] = {
	{ .compatible = "qcom,sm8150-dpu", },
	{ .compatible = "qcom,sm8250-dpu", },
	{ .compatible = "qcom,sm8350-dpu", },
	{}
};
MODULE_DEVICE_TABLE(of, dpu_dt_match);

static struct platform_driver dpu_driver = {
	.probe = dpu_dev_probe,
	.remove = dpu_dev_remove,
	.driver = {
		.name = "msm_dpu",
		.of_match_table = dpu_dt_match,
	},
};

void __init msm_dpu_register(void)
{
	platform_driver_register(&dpu_driver);
}

void __exit msm_dpu_unregister(void)
{
	platform_driver_unregister(&dpu_driver);
}
