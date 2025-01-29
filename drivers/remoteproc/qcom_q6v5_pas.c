// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm ADSP/SLPI Peripheral Image Loader for MSM8974 and MSM8996
 *
 * Copyright (C) 2016 Linaro Ltd
 * Copyright (C) 2014 Sony Mobile Communications AB
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/regulator/consumer.h>
#include <linux/remoteproc.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/iommu.h>

#include "qcom_common.h"
#include "qcom_pil_info.h"
#include "qcom_q6v5.h"
#include "remoteproc_internal.h"

#define ADSP_DECRYPT_SHUTDOWN_DELAY_MS	100

#define MAX_ASSIGN_COUNT 3

struct adsp_data {
	int crash_reason_smem;
	const char *firmware_name;
	const char *dtb_firmware_name;
	int pas_id;
	int dtb_pas_id;
	int lite_pas_id;
	unsigned int minidump_id;
	bool auto_boot;
	bool decrypt_shutdown;

	char **proxy_pd_names;

	const char *load_state;
	const char *ssr_name;
	const char *sysmon_name;
	int ssctl_id;
	unsigned int smem_host_id;

	int region_assign_idx;
	int region_assign_count;
	bool region_assign_shared;
	int region_assign_vmid;
};

struct qcom_adsp {
	struct device *dev;
	struct rproc *rproc;

	struct qcom_q6v5 q6v5;

	struct clk *xo;
	struct clk *aggre2_clk;

	struct regulator *cx_supply;
	struct regulator *px_supply;

	struct device *proxy_pds[3];

	int proxy_pd_count;

	const char *dtb_firmware_name;
	int pas_id;
	int dtb_pas_id;
	int lite_pas_id;
	unsigned int minidump_id;
	int crash_reason_smem;
	unsigned int smem_host_id;
	bool decrypt_shutdown;
	const char *info_name;

	const struct firmware *firmware;
	const struct firmware *dtb_firmware;

	struct completion start_done;
	struct completion stop_done;

	phys_addr_t mem_phys;
	phys_addr_t dtb_mem_phys;
	phys_addr_t mem_reloc;
	phys_addr_t dtb_mem_reloc;
	phys_addr_t region_assign_phys[MAX_ASSIGN_COUNT];
	void *mem_region;
	void *dtb_mem_region;
	size_t mem_size;
	size_t dtb_mem_size;
	size_t region_assign_size[MAX_ASSIGN_COUNT];

	int region_assign_idx;
	int region_assign_count;
	bool region_assign_shared;
	int region_assign_vmid;
	u64 region_assign_owners[MAX_ASSIGN_COUNT];

	struct qcom_rproc_glink glink_subdev;
	struct qcom_rproc_subdev smd_subdev;
	struct qcom_rproc_pdm pdm_subdev;
	struct qcom_rproc_ssr ssr_subdev;
	struct qcom_sysmon *sysmon;

	//struct qcom_scm_pas_metadata pas_metadata;
	//struct qcom_scm_pas_metadata dtb_pas_metadata;

	void __iomem *base, *evb_sel, *turingcc;
	struct iommu_domain *iommu;
};


#define write32(x, y) writel(y, x)

#define PLL_MODE           0x00
#if 0
#define PLL_ACTIVE_FLAG    BIT(30)
#define PLL_ACK_LATCH      BIT(29)
#define PLL_UPDATE         BIT(22)
#define PLL_VOTE_FSM_RESET BIT(21)
#define PLL_VOTE_FSM_ENA   BIT(20)
#define PLL_RESET_N        BIT(2)
#endif

#define PLL_OPMODE 0x04
#define PLL_STATE 0x08
#define PLL_STATUS 0x0c
#define PLL_L_VAL 0x10
#define PLL_ALPHA_VAL 0x14
#define PLL_USER_CTL 0x18
#define PLL_USER_CTL_U 0x1c
#define PLL_CONFIG_CTL 0x20
#define PLL_CONFIG_CTL_U 0x24
#define PLL_CONFIG_CTL_U1 0x28
#define PLL_TEST_CTL 0x2c
#define PLL_TEST_CTL_U 0x30
#define PLL_TEST_CTL_U1 0x34
#define PLL_TEST_CTL_U2 0x38
#define PLL_SSC 0x3c

static inline void
clk_pll_configure_lucid_ole(void __iomem *pll_reg, u32 freq)
{
   u16 l = freq / 19200000;
   // u16 alpha = (freq % 19200000) * 32 / 9375;

   printk("PLL_MODE: 0x%.8x L_VAL:%.8x\n", readl(pll_reg + PLL_MODE), readl(pll_reg + PLL_L_VAL));

   write32(pll_reg + PLL_L_VAL, 0x4444 << 16 | l);
   write32(pll_reg + PLL_USER_CTL, 0);
   write32(pll_reg + PLL_ALPHA_VAL, 0);

   write32(pll_reg + PLL_MODE, 0x4004);
   while ((readl(pll_reg + PLL_MODE) & 0x2000) == 0);
   write32(pll_reg + PLL_MODE, 0x0004);
   while ((readl(pll_reg + PLL_MODE) & 0x2000) != 0);
   usleep_range(1000,1000); // XXX: busywait(1)

   // HAL_clk_LucidolePLLSetRegSettings
   write32(pll_reg + PLL_CONFIG_CTL, 0x20485699);
   write32(pll_reg + PLL_CONFIG_CTL_U, 0x00182261);
   write32(pll_reg + PLL_CONFIG_CTL_U1, 0x82AA299C);
   write32(pll_reg + PLL_TEST_CTL, 0);
   write32(pll_reg + PLL_TEST_CTL_U, 0x00000003);
   write32(pll_reg + PLL_TEST_CTL_U1, 0x00009000);
   write32(pll_reg + PLL_TEST_CTL_U2, 0x00000034);
   // write32(pll_reg + PLL_USER_CTL, 0);
   write32(pll_reg + PLL_USER_CTL_U, 0x00000005);

   // HAL_clk_LucidolePLLSetCalConfig
   // set CAL_L to 0x44 (already set)

   // HAL_clk_LucidolePLLEnable
   write32(pll_reg + PLL_OPMODE, 1);
   write32(pll_reg + PLL_MODE, 5);
   write32(pll_reg + PLL_USER_CTL, 1);
}

// need to copy correct bring up / tear down sequences from hypervisor
// for now I just copied what I had in my dsp driver
// make a separate driver for this ?

#define QDSP6SS_VERSION 0x00

#define QDSP6SS_RST_EVB 0x10
#define QDSP6SS_RET_CFG 0x1c

#define QDSP6SS_BOOT_PARAMS 0x60

#define QDSP6SS_CORE_CBCR 0x48040
#define QDSP6SS_CORE_CMD_RCGR 0x48020
#define QDSP6SS_CORE_CFG_RCGR 0x48024
#define QDSP6SS_XO_CBCR 0x48120 // ACD_XO
#define QDSP6SS_SLEEP_CBCR 0x48060 // XXX

#define QDSP6SS_BOOT_CORE_START 0x400
#define QDSP6SS_BOOT_CMD 0x404
#define QDSP6SS_BOOT_STATUS 0x408

#define QDSP6SS_RESET_CFG 0x2440
#define QDSP6SS_RESET_CMD 0x2444
#define QDSP6SS_RESET_STATUS 0x2448

static int pas_auth_and_reset(struct qcom_adsp *dsp)
{
	int status;
	u32 boot_addr = dsp->mem_phys; // assume boot address is always the start of the image
	// (note: assuming images won't be relocated)

	// dtb ptr
	writel(dsp->dtb_mem_phys, dsp->base + QDSP6SS_BOOT_PARAMS);
	writel(0, dsp->base + QDSP6SS_BOOT_PARAMS + 4);
	// chip family << 16 | chip id (lanai)
	writel(0x008A022D, dsp->base + QDSP6SS_BOOT_PARAMS + 8);
	// chip version (2.0)
	writel(0x00000200, dsp->base + QDSP6SS_BOOT_PARAMS + 12);
	// platform info
	// this is "dragonboard 1.0" ? doesn't seem important
	writel(0x0A000100, dsp->base + QDSP6SS_BOOT_PARAMS + 16);
	// dtb size
	writel(dsp->dtb_mem_size, dsp->base + QDSP6SS_BOOT_PARAMS + 20);

	if (dsp->pas_id == 1) {// adsp
		// ??? SMC call
		// enable GCC_CFG_NOC_LPASS (on at boot already)
		// if (LPICX_NOC_LPASS_LPICX_SBCONTROL_SBM_FLAGOUTSTATUS0_LOW.PORT2)
		//   XXX do something different (when would this condition be true?)

		// enable LPASS_AON_CC_LPI_NOC_HS (already doing this)
		// assume LPASS_AON_CC_LPI_NOC_HS as hw ctl bit == 0
		// MISSING XXX: set enable bit on LPASS_AON_CC_Q6_AXIM (clock is on by default but the enable bit isn't set)
		// MISSING XXX: set enable bit in LPASS_QDSP6SS_CORE (doesn't seem needed for sm8650)
		// MISSING XXX: set LPASS_QDSP6SS_DBG_CFG to 0 (?)
		// MISSING XXX: set LPASS_RSC_WAIT_EVENT_OVRD_MASK to 8
		// MISSING XXX: set enable bit on LPASS_LPI_TCM_AXIS_HS
		// MISSING XXX: set 0x4001 mask in LPASS_LPI_TCM_256KB_BLOCKn (n from 0 to 7)

		writel(0, dsp->evb_sel);

		// rest can be shared with CDSP. note fw uses 960000000 init clock instead of 979200000 for adsp
	} else { // cdsp
		// turn on GCC_TURING_CFG_AHB
		// assume GCC_DDRSS_TURING_Q6_AXI as hw ctl bit == 1
		// MISSING XXX: set hw ctl bit on TURING_Q6SS_Q6_AXIM, TURING_NSPNOC
		// MISSING XXX: set enable bit on TURING_Q6SS_AHBS_AON (clock is on by default but the enable bit isn't set)
		// MISSING XXX: set TURING_QDSP6SS_DBG_CFG to 0 (?)
		// MISSING XXX: set enable bit in TURING_QDSP6SS_CORE (doesn't seem needed for sm8650)
		// MISSING XXX: configure ACD (read fuse value and set registers)
	}

	// note: ADSP init in fw has a different order for these writes, but it shouldn't matter

	writel(boot_addr >> 4, dsp->base + QDSP6SS_RST_EVB);
	writel(1, dsp->base + QDSP6SS_BOOT_CMD);
	usleep_range(1000, 2000);
	status = readl(dsp->base + QDSP6SS_BOOT_STATUS);
	if (!status)
		dev_warn(dsp->dev, "failed to boot\n");

	clk_pll_configure_lucid_ole(dsp->base + 0x40000, 979200000);

	writel(0x201, dsp->base + QDSP6SS_CORE_CFG_RCGR);
	writel(1, dsp->base + QDSP6SS_CORE_CMD_RCGR);
	usleep_range(1000, 2000);
	status = readl(dsp->base + QDSP6SS_CORE_CMD_RCGR);
	if (status & 1)
		dev_warn(dsp->dev, "failed to update rcg\n");

	writel(0, dsp->base + QDSP6SS_RET_CFG);
	writel(1, dsp->base + QDSP6SS_BOOT_CORE_START);

	return 0;
}

static int pas_shutdown(struct qcom_adsp *dsp)
{
	// need to fill this out so stop works

	return 0;
}

static void adsp_segment_dump(struct rproc *rproc, struct rproc_dump_segment *segment,
		       void *dest, size_t offset, size_t size)
{
	struct qcom_adsp *adsp = rproc->priv;
	int total_offset;

	total_offset = segment->da + segment->offset + offset - adsp->mem_phys;
	if (total_offset < 0 || total_offset + size > adsp->mem_size) {
		dev_err(adsp->dev,
			"invalid copy request for segment %pad with offset %zu and size %zu)\n",
			&segment->da, offset, size);
		memset(dest, 0xff, size);
		return;
	}

	memcpy_fromio(dest, adsp->mem_region + total_offset, size);
}

static void adsp_minidump(struct rproc *rproc)
{
	struct qcom_adsp *adsp = rproc->priv;

	if (rproc->dump_conf == RPROC_COREDUMP_DISABLED)
		return;

	qcom_minidump(rproc, adsp->minidump_id, adsp_segment_dump);
}

static int adsp_pds_enable(struct qcom_adsp *adsp, struct device **pds,
			   size_t pd_count)
{
	int ret;
	int i;

	for (i = 0; i < pd_count; i++) {
		dev_pm_genpd_set_performance_state(pds[i], INT_MAX);
		ret = pm_runtime_get_sync(pds[i]);
		if (ret < 0) {
			pm_runtime_put_noidle(pds[i]);
			dev_pm_genpd_set_performance_state(pds[i], 0);
			goto unroll_pd_votes;
		}
	}

	return 0;

unroll_pd_votes:
	for (i--; i >= 0; i--) {
		dev_pm_genpd_set_performance_state(pds[i], 0);
		pm_runtime_put(pds[i]);
	}

	return ret;
};

static void adsp_pds_disable(struct qcom_adsp *adsp, struct device **pds,
			     size_t pd_count)
{
	int i;

	for (i = 0; i < pd_count; i++) {
		dev_pm_genpd_set_performance_state(pds[i], 0);
		pm_runtime_put(pds[i]);
	}
}

static int adsp_shutdown_poll_decrypt(struct qcom_adsp *adsp)
{
	unsigned int retry_num = 50;
	int ret;

	do {
		msleep(ADSP_DECRYPT_SHUTDOWN_DELAY_MS);
		ret = pas_shutdown(adsp);
	} while (ret == -EINVAL && --retry_num);

	return ret;
}

static int adsp_unprepare(struct rproc *rproc)
{
	return 0;
}

static int adsp_load(struct rproc *rproc, const struct firmware *fw)
{
	struct qcom_adsp *adsp = rproc->priv;
	int ret;

	/* Store firmware handle to be used in adsp_start() */
	adsp->firmware = fw;

	if (adsp->lite_pas_id)
		ret = pas_shutdown(adsp);

	if (adsp->dtb_pas_id) {
		ret = request_firmware(&adsp->dtb_firmware, adsp->dtb_firmware_name, adsp->dev);
		if (ret) {
			dev_err(adsp->dev, "request_firmware failed for %s: %d\n",
				adsp->dtb_firmware_name, ret);
			return ret;
		}

		ret = qcom_mdt_load_no_init(adsp->dev, adsp->dtb_firmware, adsp->dtb_firmware_name,
					    adsp->dtb_pas_id, adsp->dtb_mem_region,
					    adsp->dtb_mem_phys, adsp->dtb_mem_size,
					    &adsp->dtb_mem_reloc);
		if (ret)
			goto release_dtb_firmware;
	}

	return 0;

release_dtb_firmware:
	release_firmware(adsp->dtb_firmware);

	return ret;
}

static int adsp_start(struct rproc *rproc)
{
	struct qcom_adsp *adsp = rproc->priv;
	int ret;

	ret = qcom_q6v5_prepare(&adsp->q6v5);
	if (ret)
		return ret;

	ret = adsp_pds_enable(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
	if (ret < 0)
		goto disable_irqs;

	ret = clk_prepare_enable(adsp->xo);
	if (ret)
		goto disable_proxy_pds;

	ret = clk_prepare_enable(adsp->aggre2_clk);
	if (ret)
		goto disable_xo_clk;

	if (adsp->cx_supply) {
		ret = regulator_enable(adsp->cx_supply);
		if (ret)
			goto disable_aggre2_clk;
	}

	if (adsp->px_supply) {
		ret = regulator_enable(adsp->px_supply);
		if (ret)
			goto disable_cx_supply;
	}

	ret = qcom_mdt_load_no_init(adsp->dev, adsp->firmware, rproc->firmware, adsp->pas_id,
				    adsp->mem_region, adsp->mem_phys, adsp->mem_size,
				    &adsp->mem_reloc);
	if (ret)
		goto disable_px_supply;

	qcom_pil_info_store(adsp->info_name, adsp->mem_phys, adsp->mem_size);

	ret = pas_auth_and_reset(adsp);
	if (ret) {
		dev_err(adsp->dev,
			"failed to authenticate image and release reset\n");
		goto disable_px_supply;
	}

	ret = qcom_q6v5_wait_for_start(&adsp->q6v5, msecs_to_jiffies(5000));
	if (ret == -ETIMEDOUT) {
		dev_err(adsp->dev, "start timed out\n");
		pas_shutdown(adsp);
		goto disable_px_supply;
	}

	/* Remove pointer to the loaded firmware, only valid in adsp_load() & adsp_start() */
	adsp->firmware = NULL;

	return 0;

disable_px_supply:
	if (adsp->px_supply)
		regulator_disable(adsp->px_supply);
disable_cx_supply:
	if (adsp->cx_supply)
		regulator_disable(adsp->cx_supply);
disable_aggre2_clk:
	clk_disable_unprepare(adsp->aggre2_clk);
disable_xo_clk:
	clk_disable_unprepare(adsp->xo);
disable_proxy_pds:
	adsp_pds_disable(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
disable_irqs:
	qcom_q6v5_unprepare(&adsp->q6v5);

	/* Remove pointer to the loaded firmware, only valid in adsp_load() & adsp_start() */
	adsp->firmware = NULL;

	return ret;
}

static void qcom_pas_handover(struct qcom_q6v5 *q6v5)
{
	struct qcom_adsp *adsp = container_of(q6v5, struct qcom_adsp, q6v5);

	if (adsp->px_supply)
		regulator_disable(adsp->px_supply);
	if (adsp->cx_supply)
		regulator_disable(adsp->cx_supply);
	clk_disable_unprepare(adsp->aggre2_clk);
	clk_disable_unprepare(adsp->xo);
	adsp_pds_disable(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
}

static int adsp_stop(struct rproc *rproc)
{
	struct qcom_adsp *adsp = rproc->priv;
	int handover;
	int ret;

	ret = qcom_q6v5_request_stop(&adsp->q6v5, adsp->sysmon);
	if (ret == -ETIMEDOUT)
		dev_err(adsp->dev, "timed out on wait\n");

	ret = pas_shutdown(adsp);
	if (ret && adsp->decrypt_shutdown)
		ret = adsp_shutdown_poll_decrypt(adsp);

	if (ret)
		dev_err(adsp->dev, "failed to shutdown: %d\n", ret);

	handover = qcom_q6v5_unprepare(&adsp->q6v5);
	if (handover)
		qcom_pas_handover(&adsp->q6v5);

	if (adsp->smem_host_id)
		ret = qcom_smem_bust_hwspin_lock_by_host(adsp->smem_host_id);

	return ret;
}

static void *adsp_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	struct qcom_adsp *adsp = rproc->priv;
	int offset;

	offset = da - adsp->mem_reloc;
	if (offset < 0 || offset + len > adsp->mem_size)
		return NULL;

	if (is_iomem)
		*is_iomem = true;

	return adsp->mem_region + offset;
}

static unsigned long adsp_panic(struct rproc *rproc)
{
	struct qcom_adsp *adsp = rproc->priv;

	return qcom_q6v5_panic(&adsp->q6v5);
}

static const struct rproc_ops adsp_ops = {
	.unprepare = adsp_unprepare,
	.start = adsp_start,
	.stop = adsp_stop,
	.da_to_va = adsp_da_to_va,
	.parse_fw = qcom_register_dump_segments,
	.load = adsp_load,
	.panic = adsp_panic,
};

static const struct rproc_ops adsp_minidump_ops = {
	.unprepare = adsp_unprepare,
	.start = adsp_start,
	.stop = adsp_stop,
	.da_to_va = adsp_da_to_va,
	.parse_fw = qcom_register_dump_segments,
	.load = adsp_load,
	.panic = adsp_panic,
	.coredump = adsp_minidump,
};

static int adsp_init_clock(struct qcom_adsp *adsp)
{
	int ret;

	adsp->xo = devm_clk_get(adsp->dev, "xo");
	if (IS_ERR(adsp->xo)) {
		ret = PTR_ERR(adsp->xo);
		if (ret != -EPROBE_DEFER)
			dev_err(adsp->dev, "failed to get xo clock");
		return ret;
	}

	adsp->aggre2_clk = devm_clk_get_optional(adsp->dev, "aggre2");
	if (IS_ERR(adsp->aggre2_clk)) {
		ret = PTR_ERR(adsp->aggre2_clk);
		if (ret != -EPROBE_DEFER)
			dev_err(adsp->dev,
				"failed to get aggre2 clock");
		return ret;
	}

	return 0;
}

static int adsp_init_regulator(struct qcom_adsp *adsp)
{
	adsp->cx_supply = devm_regulator_get_optional(adsp->dev, "cx");
	if (IS_ERR(adsp->cx_supply)) {
		if (PTR_ERR(adsp->cx_supply) == -ENODEV)
			adsp->cx_supply = NULL;
		else
			return PTR_ERR(adsp->cx_supply);
	}

	if (adsp->cx_supply)
		regulator_set_load(adsp->cx_supply, 100000);

	adsp->px_supply = devm_regulator_get_optional(adsp->dev, "px");
	if (IS_ERR(adsp->px_supply)) {
		if (PTR_ERR(adsp->px_supply) == -ENODEV)
			adsp->px_supply = NULL;
		else
			return PTR_ERR(adsp->px_supply);
	}

	return 0;
}

static int adsp_pds_attach(struct device *dev, struct device **devs,
			   char **pd_names)
{
	size_t num_pds = 0;
	int ret;
	int i;

	if (!pd_names)
		return 0;

	/* Handle single power domain */
	if (dev->pm_domain) {
		devs[0] = dev;
		pm_runtime_enable(dev);
		return 1;
	}

	while (pd_names[num_pds])
		num_pds++;

	for (i = 0; i < num_pds; i++) {
		devs[i] = dev_pm_domain_attach_by_name(dev, pd_names[i]);
		if (IS_ERR_OR_NULL(devs[i])) {
			ret = PTR_ERR(devs[i]) ? : -ENODATA;
			goto unroll_attach;
		}
	}

	return num_pds;

unroll_attach:
	for (i--; i >= 0; i--)
		dev_pm_domain_detach(devs[i], false);

	return ret;
};

static void adsp_pds_detach(struct qcom_adsp *adsp, struct device **pds,
			    size_t pd_count)
{
	struct device *dev = adsp->dev;
	int i;

	/* Handle single power domain */
	if (dev->pm_domain && pd_count) {
		pm_runtime_disable(dev);
		return;
	}

	for (i = 0; i < pd_count; i++)
		dev_pm_domain_detach(pds[i], false);
}

static int adsp_alloc_memory_region(struct qcom_adsp *adsp)
{
	struct reserved_mem *rmem;
	struct device_node *node;

	node = of_parse_phandle(adsp->dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(adsp->dev, "no memory-region specified\n");
		return -EINVAL;
	}

	rmem = of_reserved_mem_lookup(node);
	of_node_put(node);
	if (!rmem) {
		dev_err(adsp->dev, "unable to resolve memory-region\n");
		return -EINVAL;
	}

	adsp->mem_phys = adsp->mem_reloc = rmem->base;
	adsp->mem_size = rmem->size;
	adsp->mem_region = devm_ioremap_wc(adsp->dev, adsp->mem_phys, adsp->mem_size);
	if (!adsp->mem_region) {
		dev_err(adsp->dev, "unable to map memory region: %pa+%zx\n",
			&rmem->base, adsp->mem_size);
		return -EBUSY;
	}

	if (!adsp->dtb_pas_id)
		return 0;

	node = of_parse_phandle(adsp->dev->of_node, "memory-region", 1);
	if (!node) {
		dev_err(adsp->dev, "no dtb memory-region specified\n");
		return -EINVAL;
	}

	rmem = of_reserved_mem_lookup(node);
	of_node_put(node);
	if (!rmem) {
		dev_err(adsp->dev, "unable to resolve dtb memory-region\n");
		return -EINVAL;
	}

	adsp->dtb_mem_phys = adsp->dtb_mem_reloc = rmem->base;
	adsp->dtb_mem_size = rmem->size;
	adsp->dtb_mem_region = devm_ioremap_wc(adsp->dev, adsp->dtb_mem_phys, adsp->dtb_mem_size);
	if (!adsp->dtb_mem_region) {
		dev_err(adsp->dev, "unable to map dtb memory region: %pa+%zx\n",
			&rmem->base, adsp->dtb_mem_size);
		return -EBUSY;
	}

	return 0;
}

#if 0
struct iommu_entry {
	u32 base, size;
} cdsp_iommu[] = {
	{ 0x405000, 0x1000 }, // ipcc protocol 0 client 6
	{ 0x10ca000, 0x1000 }, // rng ee10
	{ 0x1fc8000, 0x1000 }, // tcsr
	{ 0x1f43000, 0x1000 }, // tcsr mutex 3
	{ 0x1fd4000, 0x1000 }, // tcsr wonce
	{ 0xb2a0000, 0x10000 }, // pdc_compute
	{ 0x100000, 0x100000 }, // gcc
	{ 0x221c2000, 0x1000 }, // qfprom_corr
};
#endif

static int adsp_assign_memory_region(struct qcom_adsp *adsp)
{
	struct device_node *node;
	int i, ret;

#if 1
	// just give it access to the whole hwio region for testing
	// (excluding first page to catch null pointers)
	iommu_map(adsp->iommu, 0x1000, 0x1000, 0x80000000u-0x1000, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
#else
	for (i = 0; i < ARRAY_SIZE(cdsp_iommu); i++) {
		const struct iommu_entry *mem = &cdsp_iommu[i];
		ret = iommu_map(adsp->iommu, mem->base, mem->base, mem->size, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		WARN_ON(ret);
	}
#endif

	for (i = 0; i < 10; i++) { // XXX: for each memory-region
		struct reserved_mem *rmem = NULL;

		node = of_parse_phandle(adsp->dev->of_node, "memory-region", i);
		if (node)
			rmem = of_reserved_mem_lookup(node);
		of_node_put(node);
		if (!rmem)
			continue;

		ret = iommu_map(adsp->iommu, rmem->base, rmem->base, rmem->size, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		WARN_ON(ret);
	}

	return 0;
}

static void adsp_unassign_memory_region(struct qcom_adsp *adsp)
{
	struct device_node *node;
	int i;

#if 1
	iommu_unmap(adsp->iommu, 0x1000, 0x80000000u-0x1000);
#else
	for (i = 0; i < ARRAY_SIZE(cdsp_iommu); i++) {
		const struct iommu_entry *mem = &cdsp_iommu[i];
		iommu_unmap(adsp->iommu, mem->base, mem->size);
	}
#endif

	for (i = 0; i < 10; i++) { // XXX: for each memory-region
		struct reserved_mem *rmem = NULL;

		node = of_parse_phandle(adsp->dev->of_node, "memory-region", i);
		if (node)
			rmem = of_reserved_mem_lookup(node);
		of_node_put(node);
		if (!rmem)
			continue;

		iommu_unmap(adsp->iommu, rmem->base, rmem->size);
	}
}

static int adsp_probe(struct platform_device *pdev)
{
	const struct adsp_data *desc;
	struct qcom_adsp *adsp;
	struct rproc *rproc;
	const char *fw_name, *dtb_fw_name = NULL;
	const struct rproc_ops *ops = &adsp_ops;
	int ret;

	desc = of_device_get_match_data(&pdev->dev);
	if (!desc)
		return -EINVAL;

	fw_name = desc->firmware_name;
	ret = of_property_read_string(pdev->dev.of_node, "firmware-name",
				      &fw_name);
	if (ret < 0 && ret != -EINVAL)
		return ret;

	if (desc->dtb_firmware_name) {
		dtb_fw_name = desc->dtb_firmware_name;
		ret = of_property_read_string_index(pdev->dev.of_node, "firmware-name", 1,
						    &dtb_fw_name);
		if (ret < 0 && ret != -EINVAL)
			return ret;
	}

	if (desc->minidump_id)
		ops = &adsp_minidump_ops;

	rproc = devm_rproc_alloc(&pdev->dev, pdev->name, ops, fw_name, sizeof(*adsp));

	if (!rproc) {
		dev_err(&pdev->dev, "unable to allocate remoteproc\n");
		return -ENOMEM;
	}

	rproc->auto_boot = desc->auto_boot;
	rproc_coredump_set_elf_info(rproc, ELFCLASS32, EM_NONE);

	adsp = rproc->priv;
	adsp->dev = &pdev->dev;
	adsp->rproc = rproc;
	adsp->minidump_id = desc->minidump_id;
	adsp->pas_id = desc->pas_id;
	adsp->lite_pas_id = desc->lite_pas_id;
	adsp->info_name = desc->sysmon_name;
	adsp->smem_host_id = desc->smem_host_id;
	adsp->decrypt_shutdown = desc->decrypt_shutdown;
	adsp->region_assign_idx = desc->region_assign_idx;
	adsp->region_assign_count = min_t(int, MAX_ASSIGN_COUNT, desc->region_assign_count);
	adsp->region_assign_vmid = desc->region_assign_vmid;
	adsp->region_assign_shared = desc->region_assign_shared;
	if (dtb_fw_name) {
		adsp->dtb_firmware_name = dtb_fw_name;
		adsp->dtb_pas_id = desc->dtb_pas_id;
	}
	platform_set_drvdata(pdev, adsp);

	ret = device_init_wakeup(adsp->dev, true);
	if (ret)
		goto free_rproc;

	adsp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adsp->base)) {
		ret = PTR_ERR(adsp->base);
		goto free_rproc;
	}

	if (platform_get_resource_byname(pdev, IORESOURCE_MEM, "evb_sel")) {
		adsp->evb_sel = devm_platform_ioremap_resource_byname(pdev, "evb_sel");
		if (IS_ERR(adsp->evb_sel))
			return PTR_ERR(adsp->evb_sel);
	}

	dma_set_mask_and_coherent(adsp->dev, 0xffffffffful); /* 36-bit VA */
	adsp->iommu = iommu_domain_alloc(&platform_bus_type);
	ret = iommu_attach_device(adsp->iommu, adsp->dev);
	WARN_ON(ret);

	ret = adsp_alloc_memory_region(adsp);
	if (ret)
		goto free_rproc;

	ret = adsp_assign_memory_region(adsp);
	if (ret)
		goto free_rproc;

	ret = adsp_init_clock(adsp);
	if (ret)
		goto free_rproc;

	ret = adsp_init_regulator(adsp);
	if (ret)
		goto free_rproc;

	ret = adsp_pds_attach(&pdev->dev, adsp->proxy_pds,
			      desc->proxy_pd_names);
	if (ret < 0)
		goto free_rproc;
	adsp->proxy_pd_count = ret;

	ret = qcom_q6v5_init(&adsp->q6v5, pdev, rproc, desc->crash_reason_smem, desc->load_state,
			     qcom_pas_handover);
	if (ret)
		goto detach_proxy_pds;

	qcom_add_glink_subdev(rproc, &adsp->glink_subdev, desc->ssr_name);
	qcom_add_smd_subdev(rproc, &adsp->smd_subdev);
	qcom_add_pdm_subdev(rproc, &adsp->pdm_subdev);
	adsp->sysmon = qcom_add_sysmon_subdev(rproc,
					      desc->sysmon_name,
					      desc->ssctl_id);
	if (IS_ERR(adsp->sysmon)) {
		ret = PTR_ERR(adsp->sysmon);
		goto detach_proxy_pds;
	}

	qcom_add_ssr_subdev(rproc, &adsp->ssr_subdev, desc->ssr_name);
	ret = rproc_add(rproc);
	if (ret)
		goto detach_proxy_pds;

	return 0;

detach_proxy_pds:
	adsp_pds_detach(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
free_rproc:
	if (adsp->iommu)

	device_init_wakeup(adsp->dev, false);

	return ret;
}

static void adsp_remove(struct platform_device *pdev)
{
	struct qcom_adsp *adsp = platform_get_drvdata(pdev);

	rproc_del(adsp->rproc);

	qcom_q6v5_deinit(&adsp->q6v5);
	adsp_unassign_memory_region(adsp);
	qcom_remove_glink_subdev(adsp->rproc, &adsp->glink_subdev);
	qcom_remove_sysmon_subdev(adsp->sysmon);
	qcom_remove_smd_subdev(adsp->rproc, &adsp->smd_subdev);
	qcom_remove_pdm_subdev(adsp->rproc, &adsp->pdm_subdev);
	qcom_remove_ssr_subdev(adsp->rproc, &adsp->ssr_subdev);
	adsp_pds_detach(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
	device_init_wakeup(adsp->dev, false);
}

static const struct adsp_data adsp_resource_init = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.auto_boot = true,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data sa8775p_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mbn",
	.pas_id = 1,
	.minidump_id = 5,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.load_state = "adsp",
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data sdm845_adsp_resource_init = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.auto_boot = true,
	.load_state = "adsp",
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data sm6350_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.load_state = "adsp",
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data sm6375_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.minidump_id = 3,
	.auto_boot = false,
	.proxy_pd_names = (char*[]){
		"cx",
		NULL
	},
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data sm8150_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		NULL
	},
	.load_state = "adsp",
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data sm8250_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.load_state = "adsp",
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data sm8350_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.load_state = "adsp",
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data msm8996_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		NULL
	},
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data cdsp_resource_init = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.auto_boot = true,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sa8775p_cdsp0_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp0.mbn",
	.pas_id = 18,
	.minidump_id = 7,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mxc",
		"nsp",
		NULL
	},
	.load_state = "cdsp",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sa8775p_cdsp1_resource = {
	.crash_reason_smem = 633,
	.firmware_name = "cdsp1.mbn",
	.pas_id = 30,
	.minidump_id = 20,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mxc",
		"nsp",
		NULL
	},
	.load_state = "nsp",
	.ssr_name = "cdsp1",
	.sysmon_name = "cdsp1",
	.ssctl_id = 0x20,
};

static const struct adsp_data sdm845_cdsp_resource_init = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.auto_boot = true,
	.load_state = "cdsp",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sm6350_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mx",
		NULL
	},
	.load_state = "cdsp",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sm8150_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		NULL
	},
	.load_state = "cdsp",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sm8250_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		NULL
	},
	.load_state = "cdsp",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sc8280xp_nsp0_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"nsp",
		NULL
	},
	.ssr_name = "cdsp0",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sc8280xp_nsp1_resource = {
	.crash_reason_smem = 633,
	.firmware_name = "cdsp.mdt",
	.pas_id = 30,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"nsp",
		NULL
	},
	.ssr_name = "cdsp1",
	.sysmon_name = "cdsp1",
	.ssctl_id = 0x20,
};

static const struct adsp_data x1e80100_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.dtb_firmware_name = "adsp_dtb.mdt",
	.pas_id = 1,
	.dtb_pas_id = 0x24,
	.lite_pas_id = 0x1f,
	.minidump_id = 5,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.load_state = "adsp",
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data x1e80100_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.dtb_firmware_name = "cdsp_dtb.mdt",
	.pas_id = 18,
	.dtb_pas_id = 0x25,
	.minidump_id = 7,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mxc",
		"nsp",
		NULL
	},
	.load_state = "cdsp",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sm8350_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mxc",
		NULL
	},
	.load_state = "cdsp",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sa8775p_gpdsp0_resource = {
	.crash_reason_smem = 640,
	.firmware_name = "gpdsp0.mbn",
	.pas_id = 39,
	.minidump_id = 21,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mxc",
		NULL
	},
	.load_state = "gpdsp0",
	.ssr_name = "gpdsp0",
	.sysmon_name = "gpdsp0",
	.ssctl_id = 0x21,
};

static const struct adsp_data sa8775p_gpdsp1_resource = {
	.crash_reason_smem = 641,
	.firmware_name = "gpdsp1.mbn",
	.pas_id = 40,
	.minidump_id = 22,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mxc",
		NULL
	},
	.load_state = "gpdsp1",
	.ssr_name = "gpdsp1",
	.sysmon_name = "gpdsp1",
	.ssctl_id = 0x22,
};

static const struct adsp_data mpss_resource_init = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.minidump_id = 3,
	.auto_boot = false,
	.proxy_pd_names = (char*[]){
		"cx",
		"mss",
		NULL
	},
	.load_state = "modem",
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data sc8180x_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.auto_boot = false,
	.proxy_pd_names = (char*[]){
		"cx",
		NULL
	},
	.load_state = "modem",
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data msm8996_slpi_resource_init = {
	.crash_reason_smem = 424,
	.firmware_name = "slpi.mdt",
	.pas_id = 12,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"ssc_cx",
		NULL
	},
	.ssr_name = "dsps",
	.sysmon_name = "slpi",
	.ssctl_id = 0x16,
};

static const struct adsp_data sdm845_slpi_resource_init = {
	.crash_reason_smem = 424,
	.firmware_name = "slpi.mdt",
	.pas_id = 12,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.load_state = "slpi",
	.ssr_name = "dsps",
	.sysmon_name = "slpi",
	.ssctl_id = 0x16,
};

static const struct adsp_data wcss_resource_init = {
	.crash_reason_smem = 421,
	.firmware_name = "wcnss.mdt",
	.pas_id = 6,
	.auto_boot = true,
	.ssr_name = "mpss",
	.sysmon_name = "wcnss",
	.ssctl_id = 0x12,
};

static const struct adsp_data sdx55_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mss",
		NULL
	},
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x22,
};

static const struct adsp_data sm8450_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.minidump_id = 3,
	.auto_boot = false,
	.decrypt_shutdown = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mss",
		NULL
	},
	.load_state = "modem",
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data sm8550_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.dtb_firmware_name = "adsp_dtb.mdt",
	.pas_id = 1,
	.dtb_pas_id = 0x24,
	.minidump_id = 5,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.load_state = "adsp",
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
	.smem_host_id = 2,
};

static const struct adsp_data sm8550_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.dtb_firmware_name = "cdsp_dtb.mdt",
	.pas_id = 18,
	.dtb_pas_id = 0x25,
	.minidump_id = 7,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mxc",
		"nsp",
		NULL
	},
	.load_state = "cdsp",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
	.smem_host_id = 5,
};

static const struct adsp_data sm8550_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.dtb_firmware_name = "modem_dtb.mdt",
	.pas_id = 4,
	.dtb_pas_id = 0x26,
	.minidump_id = 3,
	.auto_boot = false,
	.decrypt_shutdown = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mss",
		NULL
	},
	.load_state = "modem",
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x12,
	.smem_host_id = 1,
	.region_assign_idx = 2,
	.region_assign_count = 1,
	.region_assign_vmid = QCOM_SCM_VMID_MSS_MSA,
};

static const struct adsp_data sc7280_wpss_resource = {
	.crash_reason_smem = 626,
	.firmware_name = "wpss.mdt",
	.pas_id = 6,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mx",
		NULL
	},
	.load_state = "wpss",
	.ssr_name = "wpss",
	.sysmon_name = "wpss",
	.ssctl_id = 0x19,
};

static const struct adsp_data sm8650_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.dtb_firmware_name = "cdsp_dtb.mdt",
	.pas_id = 18,
	.dtb_pas_id = 0x25,
	.minidump_id = 7,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mxc",
		"nsp",
		NULL
	},
	.load_state = "cdsp",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
	.smem_host_id = 5,
	.region_assign_idx = 2,
	.region_assign_count = 1,
	.region_assign_shared = true,
	.region_assign_vmid = QCOM_SCM_VMID_CDSP,
};

static const struct adsp_data sm8650_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.dtb_firmware_name = "modem_dtb.mdt",
	.pas_id = 4,
	.dtb_pas_id = 0x26,
	.minidump_id = 3,
	.auto_boot = false,
	.decrypt_shutdown = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mss",
		NULL
	},
	.load_state = "modem",
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x12,
	.smem_host_id = 1,
	.region_assign_idx = 2,
	.region_assign_count = 3,
	.region_assign_vmid = QCOM_SCM_VMID_MSS_MSA,
};

static const struct of_device_id adsp_of_match[] = {
	{ .compatible = "qcom,msm8226-adsp-pil", .data = &adsp_resource_init},
	{ .compatible = "qcom,msm8953-adsp-pil", .data = &msm8996_adsp_resource},
	{ .compatible = "qcom,msm8974-adsp-pil", .data = &adsp_resource_init},
	{ .compatible = "qcom,msm8996-adsp-pil", .data = &msm8996_adsp_resource},
	{ .compatible = "qcom,msm8996-slpi-pil", .data = &msm8996_slpi_resource_init},
	{ .compatible = "qcom,msm8998-adsp-pas", .data = &msm8996_adsp_resource},
	{ .compatible = "qcom,msm8998-slpi-pas", .data = &msm8996_slpi_resource_init},
	{ .compatible = "qcom,qcs404-adsp-pas", .data = &adsp_resource_init },
	{ .compatible = "qcom,qcs404-cdsp-pas", .data = &cdsp_resource_init },
	{ .compatible = "qcom,qcs404-wcss-pas", .data = &wcss_resource_init },
	{ .compatible = "qcom,sa8775p-adsp-pas", .data = &sa8775p_adsp_resource},
	{ .compatible = "qcom,sa8775p-cdsp0-pas", .data = &sa8775p_cdsp0_resource},
	{ .compatible = "qcom,sa8775p-cdsp1-pas", .data = &sa8775p_cdsp1_resource},
	{ .compatible = "qcom,sa8775p-gpdsp0-pas", .data = &sa8775p_gpdsp0_resource},
	{ .compatible = "qcom,sa8775p-gpdsp1-pas", .data = &sa8775p_gpdsp1_resource},
	{ .compatible = "qcom,sc7180-adsp-pas", .data = &sm8250_adsp_resource},
	{ .compatible = "qcom,sc7180-mpss-pas", .data = &mpss_resource_init},
	{ .compatible = "qcom,sc7280-adsp-pas", .data = &sm8350_adsp_resource},
	{ .compatible = "qcom,sc7280-cdsp-pas", .data = &sm6350_cdsp_resource},
	{ .compatible = "qcom,sc7280-mpss-pas", .data = &mpss_resource_init},
	{ .compatible = "qcom,sc7280-wpss-pas", .data = &sc7280_wpss_resource},
	{ .compatible = "qcom,sc8180x-adsp-pas", .data = &sm8150_adsp_resource},
	{ .compatible = "qcom,sc8180x-cdsp-pas", .data = &sm8150_cdsp_resource},
	{ .compatible = "qcom,sc8180x-mpss-pas", .data = &sc8180x_mpss_resource},
	{ .compatible = "qcom,sc8280xp-adsp-pas", .data = &sm8250_adsp_resource},
	{ .compatible = "qcom,sc8280xp-nsp0-pas", .data = &sc8280xp_nsp0_resource},
	{ .compatible = "qcom,sc8280xp-nsp1-pas", .data = &sc8280xp_nsp1_resource},
	{ .compatible = "qcom,sdm660-adsp-pas", .data = &adsp_resource_init},
	{ .compatible = "qcom,sdm845-adsp-pas", .data = &sdm845_adsp_resource_init},
	{ .compatible = "qcom,sdm845-cdsp-pas", .data = &sdm845_cdsp_resource_init},
	{ .compatible = "qcom,sdm845-slpi-pas", .data = &sdm845_slpi_resource_init},
	{ .compatible = "qcom,sdx55-mpss-pas", .data = &sdx55_mpss_resource},
	{ .compatible = "qcom,sdx75-mpss-pas", .data = &sm8650_mpss_resource},
	{ .compatible = "qcom,sm6115-adsp-pas", .data = &adsp_resource_init},
	{ .compatible = "qcom,sm6115-cdsp-pas", .data = &cdsp_resource_init},
	{ .compatible = "qcom,sm6115-mpss-pas", .data = &sc8180x_mpss_resource},
	{ .compatible = "qcom,sm6350-adsp-pas", .data = &sm6350_adsp_resource},
	{ .compatible = "qcom,sm6350-cdsp-pas", .data = &sm6350_cdsp_resource},
	{ .compatible = "qcom,sm6350-mpss-pas", .data = &mpss_resource_init},
	{ .compatible = "qcom,sm6375-adsp-pas", .data = &sm6350_adsp_resource},
	{ .compatible = "qcom,sm6375-cdsp-pas", .data = &sm8150_cdsp_resource},
	{ .compatible = "qcom,sm6375-mpss-pas", .data = &sm6375_mpss_resource},
	{ .compatible = "qcom,sm8150-adsp-pas", .data = &sm8150_adsp_resource},
	{ .compatible = "qcom,sm8150-cdsp-pas", .data = &sm8150_cdsp_resource},
	{ .compatible = "qcom,sm8150-mpss-pas", .data = &mpss_resource_init},
	{ .compatible = "qcom,sm8150-slpi-pas", .data = &sdm845_slpi_resource_init},
	{ .compatible = "qcom,sm8250-adsp-pas", .data = &sm8250_adsp_resource},
	{ .compatible = "qcom,sm8250-cdsp-pas", .data = &sm8250_cdsp_resource},
	{ .compatible = "qcom,sm8250-slpi-pas", .data = &sdm845_slpi_resource_init},
	{ .compatible = "qcom,sm8350-adsp-pas", .data = &sm8350_adsp_resource},
	{ .compatible = "qcom,sm8350-cdsp-pas", .data = &sm8350_cdsp_resource},
	{ .compatible = "qcom,sm8350-slpi-pas", .data = &sdm845_slpi_resource_init},
	{ .compatible = "qcom,sm8350-mpss-pas", .data = &mpss_resource_init},
	{ .compatible = "qcom,sm8450-adsp-pas", .data = &sm8350_adsp_resource},
	{ .compatible = "qcom,sm8450-cdsp-pas", .data = &sm8350_cdsp_resource},
	{ .compatible = "qcom,sm8450-slpi-pas", .data = &sdm845_slpi_resource_init},
	{ .compatible = "qcom,sm8450-mpss-pas", .data = &sm8450_mpss_resource},
	{ .compatible = "qcom,sm8550-adsp-pas", .data = &sm8550_adsp_resource},
	{ .compatible = "qcom,sm8550-cdsp-pas", .data = &sm8550_cdsp_resource},
	{ .compatible = "qcom,sm8550-mpss-pas", .data = &sm8550_mpss_resource},
	{ .compatible = "qcom,sm8650-adsp-pas", .data = &sm8550_adsp_resource},
	{ .compatible = "qcom,sm8650-cdsp-pas", .data = &sm8650_cdsp_resource},
	{ .compatible = "qcom,sm8650-mpss-pas", .data = &sm8650_mpss_resource},
	{ .compatible = "qcom,x1e80100-adsp-pas", .data = &x1e80100_adsp_resource},
	{ .compatible = "qcom,x1e80100-cdsp-pas", .data = &x1e80100_cdsp_resource},
	{ },
};
MODULE_DEVICE_TABLE(of, adsp_of_match);

static struct platform_driver adsp_driver = {
	.probe = adsp_probe,
	.remove_new = adsp_remove,
	.driver = {
		.name = "qcom_q6v5_pas",
		.of_match_table = adsp_of_match,
	},
};

module_platform_driver(adsp_driver);
MODULE_DESCRIPTION("Qualcomm Hexagon v5 Peripheral Authentication Service driver");
MODULE_LICENSE("GPL v2");
