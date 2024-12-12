// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm Technology Inc. ADSP Peripheral Image Loader for SDM845.
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>

#include "qcom_common.h"
#include "qcom_pil_info.h"
#include "qcom_q6v5.h"
#include "remoteproc_internal.h"

/* time out value */
#define ACK_TIMEOUT			1000
#define ACK_TIMEOUT_US			1000000
#define BOOT_FSM_TIMEOUT		10000
/* mask values */
#define EVB_MASK			GENMASK(27, 4)
/*QDSP6SS register offsets*/
#define RST_EVB_REG			0x10
#define BOOT_PARAMS(i)			(0x60 + ((i) * 4))
#define CORE_START_REG			0x400
#define BOOT_CMD_REG			0x404
#define BOOT_STATUS_REG			0x408
#define RET_CFG_REG			0x1C
/*TCSR register offsets*/
#define LPASS_MASTER_IDLE_REG		0x8
#define LPASS_HALTACK_REG		0x4
//#define LPASS_PWR_ON_REG		0x10
#define LPASS_PWR_ON_REG		0xc
#define LPASS_HALTREQ_REG		0x0

#define SID_MASK_DEFAULT        0xF

#define QDSP6SS_XO_CBCR		0x38
#define QDSP6SS_CORE_CBCR	0x20
#define QDSP6SS_SLEEP_CBCR	0x3c

#define LPASS_BOOT_CORE_START	BIT(0)
#define LPASS_BOOT_CMD_START	BIT(0)
#define LPASS_EFUSE_Q6SS_EVB_SEL 0x0

struct qcom_adsp;

struct adsp_pil_data {
	int crash_reason_smem;
	const char *firmware_name;
	const char *dtb_firmware_name;
	const char *rsc_firmware_name;

	const char *ssr_name;
	const char *sysmon_name;
	int ssctl_id;
	bool is_wpss;
	bool has_iommu;
	bool auto_boot;

	const char **clk_ids;
	int num_clks;
	const char **pd_names;
	unsigned int num_pds;
	const char *load_state;

	int (*clk_setup)(struct qcom_adsp *adsp);
};

struct qcom_adsp {
	struct device *dev;
	struct rproc *rproc;

	struct qcom_q6v5 q6v5;

	struct clk *xo;

	int num_clks;
	struct clk_bulk_data *clks;

	void __iomem *qdsp6ss_base;
	void __iomem *lpass_efuse;

	struct reset_control *pdc_sync_reset;
	struct reset_control *restart;

	struct regmap *halt_map;
	unsigned int halt_lpass;

	int crash_reason_smem;
	const char *info_name;

	const char *dtb_firmware_name;
	const struct firmware *dtb_firmware;
	const char *rsc_firmware_name;
	const struct firmware *rsc_firmware;

	struct completion start_done;
	struct completion stop_done;

	phys_addr_t mem_phys;
	phys_addr_t dtb_mem_phys;	
	phys_addr_t mem_reloc;
	phys_addr_t dtb_mem_reloc;	
	void *mem_region;
	void *dtb_mem_region;	
	size_t mem_size;
	size_t dtb_mem_size;	
	bool has_iommu;

	struct dev_pm_domain_list *pd_list;

	struct qcom_rproc_glink glink_subdev;
	struct qcom_rproc_pdm pdm_subdev;
	struct qcom_rproc_ssr ssr_subdev;
	struct qcom_sysmon *sysmon;

	int (*clk_setup)(struct qcom_adsp *adsp);
	int (*shutdown)(struct qcom_adsp *adsp);
};

static int qcom_rproc_pds_attach(struct qcom_adsp *adsp, const char **pd_names,
				 unsigned int num_pds)
{
	struct device *dev = adsp->dev;
	struct dev_pm_domain_attach_data pd_data = {
		.pd_names = pd_names,
		.num_pd_names = num_pds,
	};
	int ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	/* Handle single power domain */
	if (dev->pm_domain)
		goto out;

	if (!pd_names)
		return 0;

	ret = dev_pm_domain_attach_list(dev, &pd_data, &adsp->pd_list);
	if (ret < 0)
		return ret;

out:
	pm_runtime_enable(dev);
	return 0;
}

static void qcom_rproc_pds_detach(struct qcom_adsp *adsp)
{
	struct device *dev = adsp->dev;
	struct dev_pm_domain_list *pds = adsp->pd_list;

	pr_info("%s:%d\n", __func__, __LINE__);

	dev_pm_domain_detach_list(pds);

	if (dev->pm_domain || pds)
		pm_runtime_disable(adsp->dev);
}

static int qcom_rproc_pds_enable(struct qcom_adsp *adsp)
{
	struct device *dev = adsp->dev;
	struct dev_pm_domain_list *pds = adsp->pd_list;
	int ret, i = 0;

	pr_info("%s:%d\n", __func__, __LINE__);

	if (!dev->pm_domain && !pds)
		return 0;

	if (dev->pm_domain)
		dev_pm_genpd_set_performance_state(dev, INT_MAX);

	while (pds && i < pds->num_pds) {
		dev_pm_genpd_set_performance_state(pds->pd_devs[i], INT_MAX);
		i++;
	}

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		while (pds && i > 0) {
			i--;
			dev_pm_genpd_set_performance_state(pds->pd_devs[i], 0);
		}

		if (dev->pm_domain)
			dev_pm_genpd_set_performance_state(dev, 0);
	}

	return ret;
}

static void qcom_rproc_pds_disable(struct qcom_adsp *adsp)
{
	struct device *dev = adsp->dev;
	struct dev_pm_domain_list *pds = adsp->pd_list;
	int i = 0;

	pr_info("%s:%d\n", __func__, __LINE__);

	if (!dev->pm_domain && !pds)
		return;

	if (dev->pm_domain)
		dev_pm_genpd_set_performance_state(dev, 0);

	while (pds && i < pds->num_pds) {
		dev_pm_genpd_set_performance_state(pds->pd_devs[i], 0);
		i++;
	}

	pm_runtime_put(dev);
}

static int qcom_wpss_shutdown(struct qcom_adsp *adsp)
{
	unsigned int val;

	regmap_write(adsp->halt_map, adsp->halt_lpass + LPASS_HALTREQ_REG, 1);

	/* Wait for halt ACK from QDSP6 */
	regmap_read_poll_timeout(adsp->halt_map,
				 adsp->halt_lpass + LPASS_HALTACK_REG, val,
				 val, 1000, ACK_TIMEOUT_US);

	/* Assert the WPSS PDC Reset */
	reset_control_assert(adsp->pdc_sync_reset);

	/* Place the WPSS processor into reset */
	reset_control_assert(adsp->restart);

	/* wait after asserting subsystem restart from AOSS */
	usleep_range(200, 205);

	/* Remove the WPSS reset */
	reset_control_deassert(adsp->restart);

	/* De-assert the WPSS PDC Reset */
	reset_control_deassert(adsp->pdc_sync_reset);

	usleep_range(100, 105);

	clk_bulk_disable_unprepare(adsp->num_clks, adsp->clks);

	regmap_write(adsp->halt_map, adsp->halt_lpass + LPASS_HALTREQ_REG, 0);

	/* Wait for halt ACK from QDSP6 */
	regmap_read_poll_timeout(adsp->halt_map,
				 adsp->halt_lpass + LPASS_HALTACK_REG, val,
				 !val, 1000, ACK_TIMEOUT_US);

	return 0;
}

static int qcom_adsp_shutdown(struct qcom_adsp *adsp)
{
	unsigned long timeout;
	unsigned int val;
	int ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	/* Reset the retention logic */
	val = readl(adsp->qdsp6ss_base + RET_CFG_REG);
	val |= 0x1;
	writel(val, adsp->qdsp6ss_base + RET_CFG_REG);

	clk_bulk_disable_unprepare(adsp->num_clks, adsp->clks);

	/* QDSP6 master port needs to be explicitly halted */
	ret = regmap_read(adsp->halt_map,
			adsp->halt_lpass + LPASS_PWR_ON_REG, &val);
	if (ret || !val)
		goto reset;

	ret = regmap_read(adsp->halt_map,
			adsp->halt_lpass + LPASS_MASTER_IDLE_REG,
			&val);
	if (ret || val)
		goto reset;

	regmap_write(adsp->halt_map,
			adsp->halt_lpass + LPASS_HALTREQ_REG, 1);

	/* Wait for halt ACK from QDSP6 */
	timeout = jiffies + msecs_to_jiffies(ACK_TIMEOUT);
	for (;;) {
		ret = regmap_read(adsp->halt_map,
			adsp->halt_lpass + LPASS_HALTACK_REG, &val);
		if (ret || val || time_after(jiffies, timeout))
			break;

		usleep_range(1000, 1100);
	}

	ret = regmap_read(adsp->halt_map,
			adsp->halt_lpass + LPASS_MASTER_IDLE_REG, &val);
	if (ret || !val)
		dev_err(adsp->dev, "port failed halt\n");

reset:
	/* Assert the LPASS PDC Reset */
	reset_control_assert(adsp->pdc_sync_reset);
	/* Place the LPASS processor into reset */
	reset_control_assert(adsp->restart);
	/* wait after asserting subsystem restart from AOSS */
	usleep_range(200, 300);

	/* Clear the halt request for the AXIM and AHBM for Q6 */
	regmap_write(adsp->halt_map, adsp->halt_lpass + LPASS_HALTREQ_REG, 0);

	/* De-assert the LPASS PDC Reset */
	reset_control_deassert(adsp->pdc_sync_reset);
	/* Remove the LPASS reset */
	reset_control_deassert(adsp->restart);
	/* wait after de-asserting subsystem restart from AOSS */
	usleep_range(200, 300);

	return 0;
}

static int adsp_load(struct rproc *rproc, const struct firmware *fw)
{
	struct qcom_adsp *adsp = rproc->priv;
	int ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	if (adsp->dtb_firmware_name) {
		ret = request_firmware(&adsp->dtb_firmware, adsp->dtb_firmware_name, adsp->dev);
		if (ret) {
			dev_err(adsp->dev, "request_firmware failed for %s: %d\n",
				adsp->dtb_firmware_name, ret);
			return ret;
		}

		ret = qcom_mdt_load_no_init(adsp->dev, adsp->dtb_firmware,
					    adsp->dtb_firmware_name, 0,
					    adsp->dtb_mem_region,
					    adsp->dtb_mem_phys,
					    adsp->dtb_mem_size,
					    &adsp->dtb_mem_reloc);
		if (ret)
			return ret;
	}


	pr_info("%s:%d\n", __func__, __LINE__);

	ret = qcom_mdt_load_no_init(adsp->dev, fw, rproc->firmware, 0,
				    adsp->mem_region, adsp->mem_phys,
				    adsp->mem_size, &adsp->mem_reloc);
	if (ret)
		return ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	qcom_pil_info_store(adsp->info_name, adsp->mem_phys, adsp->mem_size);

	pr_info("%s:%d\n", __func__, __LINE__);

	return 0;
}

static void adsp_unmap_carveout(struct rproc *rproc)
{
	struct qcom_adsp *adsp = rproc->priv;

	pr_info("%s:%d\n", __func__, __LINE__);

	if (adsp->has_iommu)
		iommu_unmap(rproc->domain, adsp->mem_phys, adsp->mem_size);
}

static int adsp_map_carveout(struct rproc *rproc)
{
	struct qcom_adsp *adsp = rproc->priv;
	struct of_phandle_args args;
	long long sid;
	unsigned long iova;
	int ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	if (!adsp->has_iommu)
		return 0;

	if (!rproc->domain)
		return -EINVAL;

	ret = of_parse_phandle_with_args(adsp->dev->of_node, "iommus", "#iommu-cells", 0, &args);
	if (ret < 0)
		return ret;

	sid = args.args[0] & SID_MASK_DEFAULT;

	/* Add SID configuration for ADSP Firmware to SMMU */
	iova =  adsp->mem_phys | (sid << 32);

	ret = iommu_map(rproc->domain, iova, adsp->mem_phys,
			adsp->mem_size,	IOMMU_READ | IOMMU_WRITE,
			GFP_KERNEL);
	if (ret) {
		dev_err(adsp->dev, "Unable to map ADSP Physical Memory\n");
		return ret;
	}

	if (adsp->dtb_firmware_name) {
		/* Add SID configuration for ADSP Firmware to SMMU */
		iova =  adsp->dtb_mem_phys | (sid << 32);

		ret = iommu_map(rproc->domain, iova, adsp->dtb_mem_phys,
				adsp->dtb_mem_size, IOMMU_READ | IOMMU_WRITE,
				GFP_KERNEL);
		if (ret) {
			dev_err(adsp->dev, "Unable to map ADSP DTB Physical Memory\n");
			return ret;
		}
	}

	pr_info("%s:%d\n", __func__, __LINE__);

	return 0;
}

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

#define write32(x, y) writel(y, x)

static inline void
clk_pll_configure_lucid_ole(void __iomem *pll_reg, u32 freq)
{
   u16 l = freq / 19200000;
   u16 alpha = (freq % 19200000) * 32 / 9375;

   write32(pll_reg + PLL_MODE, 0x00);

   // Add RINGOSC_CAL_L extra shift?
   write32(pll_reg + PLL_L_VAL, 0x44 << 24 | 0x44 << 16 | l);
   write32(pll_reg + PLL_ALPHA_VAL, alpha);
   write32(pll_reg + PLL_USER_CTL, 0x00008400 | 1);
   write32(pll_reg + PLL_USER_CTL_U, 0x00000005);
   write32(pll_reg + PLL_CONFIG_CTL, 0x20485699);
   write32(pll_reg + PLL_CONFIG_CTL_U, 0x00182261);
   write32(pll_reg + PLL_CONFIG_CTL_U1, 0x82AA299C);
   write32(pll_reg + PLL_TEST_CTL, 0);
   write32(pll_reg + PLL_TEST_CTL_U, 0x00000003);
   write32(pll_reg + PLL_TEST_CTL_U1, 0x00009000);
   write32(pll_reg + PLL_TEST_CTL_U2, 0x00000034);

#define PLL_UPDATE_BYPASS	BIT(23)
	write32(pll_reg + PLL_MODE, PLL_UPDATE_BYPASS);

#define PLL_STANDBY		0x0
#define PLL_RUN			0x1

#define PLL_LOCK_DET BIT(31)
#define OUT_MAIN_HW_CTL BIT(3)
#define PLL_RESET_N  BIT(2)
#define PLL_OUTCTRL  BIT(0)

	write32(pll_reg + PLL_OPMODE, PLL_STANDBY);
	write32(pll_reg + PLL_MODE, PLL_RESET_N | PLL_UPDATE_BYPASS);

	write32(pll_reg + PLL_OPMODE, PLL_RUN);

	//usleep_range(1000, 2000);
	//printk("PLL_MODE= 0x%.8x\n", readl(pll_reg + PLL_MODE));
	while ((readl(pll_reg + PLL_MODE) & PLL_LOCK_DET) == 0);

	write32(pll_reg + PLL_MODE, PLL_RESET_N | PLL_OUTCTRL);// | OUT_MAIN_HW_CTL);
}

static int cdsp_sm8650_clk_setup(struct qcom_adsp *adsp)
{
	void __iomem *turingcc = devm_ioremap(adsp->dev, 0x32008000, 0x20000);
	WARN_ON(IS_ERR_OR_NULL(turingcc));

	// 0x32008404 ?
	writel_relaxed(1, turingcc + 0x404); // TURING_Q6SS_Q6_AXIM_CBCR
	//writel_relaxed(1, turingcc + 0x220); // TURING_CENG_CBCR
	writel_relaxed(1, turingcc + 0x22c); // TURING_NSPNOC_CBCR
	writel_relaxed(1, turingcc + 0x414); // TURING_Q6SS_AHBS_AON_CBCR
	writel_relaxed(1, turingcc + 0x248);
	writel_relaxed(1, turingcc + 0x410);

	// TURING_QDSP6SS_PLL_PLL_CM_PLL_LUCID_OLE
	// Set it to use TURBO_L3 frequency - 1843.2 MHz
	clk_pll_configure_lucid_ole(adsp->qdsp6ss_base + 0x40000, 1843200000);

#define CDSP_QDSP6SS_CORE_CBCR 0x48040
#define CDSP_QDSP6SS_CORE_CMD_RCGR 0x48020
#define CDSP_QDSP6SS_CORE_CFG_RCGR 0x48024
#define CDSP_QDSP6SS_XO_CBCR 0x48120 // ACD_XO
#define CDSP_QDSP6SS_SLEEP_CBCR 0x48060 // XXX

	writel(0x200, adsp->qdsp6ss_base + CDSP_QDSP6SS_CORE_CFG_RCGR);
	writel(1, adsp->qdsp6ss_base + CDSP_QDSP6SS_CORE_CMD_RCGR);

	writel(1, adsp->qdsp6ss_base + CDSP_QDSP6SS_XO_CBCR);
	writel(1, adsp->qdsp6ss_base + CDSP_QDSP6SS_SLEEP_CBCR);
	writel(1, adsp->qdsp6ss_base + CDSP_QDSP6SS_CORE_CBCR);

	usleep_range(1000, 2000);

	return 0;
}

static int adsp_start(struct rproc *rproc)
{
	struct qcom_adsp *adsp = rproc->priv;
	int ret;
	unsigned int val;

	pr_info("%s:%d\n", __func__, __LINE__);

	ret = qcom_q6v5_prepare(&adsp->q6v5);
	if (ret)
		return ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	ret = adsp_map_carveout(rproc);
	if (ret) {
		dev_err(adsp->dev, "ADSP smmu mapping failed\n");
		goto disable_irqs;
	}

	pr_info("%s:%d\n", __func__, __LINE__);

	ret = clk_prepare_enable(adsp->xo);
	if (ret)
		goto adsp_smmu_unmap;

	ret = qcom_rproc_pds_enable(adsp);
	if (ret < 0)
		goto disable_xo_clk;

	pr_info("%s:%d\n", __func__, __LINE__);

	ret = clk_bulk_prepare_enable(adsp->num_clks, adsp->clks);
	if (ret) {
		dev_err(adsp->dev, "adsp clk_enable failed\n");
		goto disable_power_domain;
	}

	pr_info("%s:%d\n", __func__, __LINE__);

	if (adsp->clk_setup) {
		ret = adsp->clk_setup(adsp);
		if (ret)
			goto disable_adsp_clks;
	} else {
		/* Enable the XO clock */
		writel(1, adsp->qdsp6ss_base + QDSP6SS_XO_CBCR);

		/* Enable the QDSP6SS sleep clock */
		writel(1, adsp->qdsp6ss_base + QDSP6SS_SLEEP_CBCR);

		/* Enable the QDSP6 core clock */
		writel(1, adsp->qdsp6ss_base + QDSP6SS_CORE_CBCR);
	}

	pr_info("%s:%d\n", __func__, __LINE__);

	/* Program boot address */
	writel(adsp->mem_phys >> 4, adsp->qdsp6ss_base + RST_EVB_REG);

	if (adsp->dtb_mem_phys) {
		writel(adsp->dtb_mem_phys, adsp->qdsp6ss_base + BOOT_PARAMS(0));
		writel(0, adsp->qdsp6ss_base + BOOT_PARAMS(1));
		writel(0x008A022D, adsp->qdsp6ss_base + BOOT_PARAMS(2)); // chip_family << 16 | soc_id
		writel(0x00000200, adsp->qdsp6ss_base + BOOT_PARAMS(3)); // (SOCINFO_MAJOR(ver) >> 16) << 8) | SOCINFO_MINOR(ver) & 0xff
		//writel(0x0B000100, adsp->qdsp6ss_base + BOOT_PARAMS(4)); // hw_plat << 16 | (plat_ver >> 16) << 8) | plat_ver & 0xff
		writel(0x0A000100, adsp->qdsp6ss_base + BOOT_PARAMS(4)); // hw_plat << 16 | (plat_ver >> 16) << 8) | plat_ver & 0xff
		writel(adsp->dtb_mem_size, adsp->qdsp6ss_base + BOOT_PARAMS(5));
	}

	pr_info("%s:%d\n", __func__, __LINE__);

	if (adsp->lpass_efuse)
		writel(LPASS_EFUSE_Q6SS_EVB_SEL, adsp->lpass_efuse);

	pr_info("%s:%d\n", __func__, __LINE__);

	/* De-assert QDSP6 stop core. QDSP6 will execute after out of reset */
	writel(LPASS_BOOT_CORE_START, adsp->qdsp6ss_base + CORE_START_REG);

	usleep_range(1000, 2000);

	/* Trigger boot FSM to start QDSP6 */
	writel(LPASS_BOOT_CMD_START, adsp->qdsp6ss_base + BOOT_CMD_REG);

	pr_info("%s:%d\n", __func__, __LINE__);

	usleep_range(1000, 2000);

	/* Wait for core to come out of reset */
	ret = readl_poll_timeout(adsp->qdsp6ss_base + BOOT_STATUS_REG,
			val, (val & BIT(0)) != 0, 10, BOOT_FSM_TIMEOUT);
	if (ret) {
		dev_err(adsp->dev, "failed to bootup adsp\n");
		goto disable_adsp_clks;
	}

	pr_info("%s:%d\n", __func__, __LINE__);

	ret = qcom_q6v5_wait_for_start(&adsp->q6v5, msecs_to_jiffies(5 * HZ));
	if (ret == -ETIMEDOUT) {
		dev_err(adsp->dev, "start timed out\n");
		goto disable_adsp_clks;
	}

	pr_info("%s:%d\n", __func__, __LINE__);

	return 0;

disable_adsp_clks:
	clk_bulk_disable_unprepare(adsp->num_clks, adsp->clks);
disable_power_domain:
	qcom_rproc_pds_disable(adsp);
disable_xo_clk:
	clk_disable_unprepare(adsp->xo);
adsp_smmu_unmap:
	adsp_unmap_carveout(rproc);
disable_irqs:
	qcom_q6v5_unprepare(&adsp->q6v5);

	return ret;
}

static void qcom_adsp_pil_handover(struct qcom_q6v5 *q6v5)
{
	struct qcom_adsp *adsp = container_of(q6v5, struct qcom_adsp, q6v5);

	pr_info("%s:%d\n", __func__, __LINE__);

	clk_disable_unprepare(adsp->xo);
	qcom_rproc_pds_disable(adsp);
}

static int adsp_stop(struct rproc *rproc)
{
	struct qcom_adsp *adsp = rproc->priv;
	int handover;
	int ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	ret = qcom_q6v5_request_stop(&adsp->q6v5, adsp->sysmon);
	if (ret == -ETIMEDOUT)
		dev_err(adsp->dev, "timed out on wait\n");

	pr_info("%s:%d\n", __func__, __LINE__);

	ret = adsp->shutdown(adsp);
	if (ret)
		dev_err(adsp->dev, "failed to shutdown: %d\n", ret);

	pr_info("%s:%d\n", __func__, __LINE__);

	adsp_unmap_carveout(rproc);

	pr_info("%s:%d\n", __func__, __LINE__);

	handover = qcom_q6v5_unprepare(&adsp->q6v5);
	if (handover)
		qcom_adsp_pil_handover(&adsp->q6v5);

	return ret;
}

static void *adsp_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	struct qcom_adsp *adsp = rproc->priv;
	int offset;

	offset = da - adsp->mem_reloc;
	if (offset < 0 || offset + len > adsp->mem_size)
		return NULL;

	return adsp->mem_region + offset;
}

static int adsp_parse_firmware(struct rproc *rproc, const struct firmware *fw)
{
	struct qcom_adsp *adsp = rproc->priv;
	int ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	ret = qcom_register_dump_segments(rproc, fw);
	if (ret) {
		dev_err(&rproc->dev, "Error in registering dump segments\n");
		return ret;
	}

	if (adsp->has_iommu) {
		if (adsp->rsc_firmware_name) {
			ret = request_firmware(&adsp->rsc_firmware, adsp->rsc_firmware_name,
					       adsp->dev);
			if (ret) {
				dev_err(adsp->dev, "request_firmware failed for %s: %d\n",
						adsp->rsc_firmware_name, ret);
				return ret;
			}

			rproc->cached_table = kmemdup(adsp->rsc_firmware->data,
						      adsp->rsc_firmware->size,
						      GFP_KERNEL);
			if (!rproc->cached_table)
				return -ENOMEM;

			rproc->table_ptr = rproc->cached_table;
			rproc->table_sz = adsp->rsc_firmware->size;

			release_firmware(adsp->rsc_firmware);
		} else {
			ret = rproc_elf_load_rsc_table(rproc, fw);
			if (ret) {
				dev_err(&rproc->dev, "Error in loading resource table\n");
			}
		}
	}
	return 0;
}

static unsigned long adsp_panic(struct rproc *rproc)
{
	struct qcom_adsp *adsp = rproc->priv;

	pr_info("%s:%d\n", __func__, __LINE__);

	return qcom_q6v5_panic(&adsp->q6v5);
}

static const struct rproc_ops adsp_ops = {
	.start = adsp_start,
	.stop = adsp_stop,
	.da_to_va = adsp_da_to_va,
	.parse_fw = adsp_parse_firmware,
	.load = adsp_load,
	.panic = adsp_panic,
};

static int adsp_init_clock(struct qcom_adsp *adsp, const char **clk_ids)
{
	int num_clks = 0;
	int i, ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	adsp->xo = devm_clk_get(adsp->dev, "xo");
	if (IS_ERR(adsp->xo)) {
		ret = PTR_ERR(adsp->xo);
		if (ret != -EPROBE_DEFER)
			dev_err(adsp->dev, "failed to get xo clock");
		return ret;
	}

	for (i = 0; clk_ids[i]; i++)
		num_clks++;

	adsp->num_clks = num_clks;
	adsp->clks = devm_kcalloc(adsp->dev, adsp->num_clks,
				sizeof(*adsp->clks), GFP_KERNEL);
	if (!adsp->clks)
		return -ENOMEM;

	for (i = 0; i < adsp->num_clks; i++)
		adsp->clks[i].id = clk_ids[i];

	return devm_clk_bulk_get(adsp->dev, adsp->num_clks, adsp->clks);
}

static int adsp_init_reset(struct qcom_adsp *adsp)
{

	pr_info("%s:%d\n", __func__, __LINE__);

	adsp->pdc_sync_reset = devm_reset_control_get_optional_exclusive(adsp->dev,
			"pdc_sync");
	if (IS_ERR(adsp->pdc_sync_reset)) {
		dev_err(adsp->dev, "failed to acquire pdc_sync reset\n");
		return PTR_ERR(adsp->pdc_sync_reset);
	}

	adsp->restart = devm_reset_control_get_optional_exclusive(adsp->dev, "restart");

	/* Fall back to the  old "cc_lpass" if "restart" is absent */
	if (!adsp->restart)
		adsp->restart = devm_reset_control_get_exclusive(adsp->dev, "cc_lpass");

	if (IS_ERR(adsp->restart)) {
		dev_err(adsp->dev, "failed to acquire restart\n");
		return PTR_ERR(adsp->restart);
	}

	return 0;
}

static int adsp_init_mmio(struct qcom_adsp *adsp,
				struct platform_device *pdev)
{
	struct resource *efuse_region;
	struct device_node *syscon;
	int ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	adsp->qdsp6ss_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adsp->qdsp6ss_base)) {
		dev_err(adsp->dev, "failed to map QDSP6SS registers\n");
		return PTR_ERR(adsp->qdsp6ss_base);
	}

	efuse_region = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!efuse_region) {
		adsp->lpass_efuse = NULL;
		dev_dbg(adsp->dev, "failed to get efuse memory region\n");
	} else {
		adsp->lpass_efuse = devm_ioremap_resource(&pdev->dev, efuse_region);
		if (IS_ERR(adsp->lpass_efuse)) {
			dev_err(adsp->dev, "failed to map efuse registers\n");
			return PTR_ERR(adsp->lpass_efuse);
		}
	}
	syscon = of_parse_phandle(pdev->dev.of_node, "qcom,halt-regs", 0);
	if (!syscon) {
		dev_err(&pdev->dev, "failed to parse qcom,halt-regs\n");
		return -EINVAL;
	}

	adsp->halt_map = syscon_node_to_regmap(syscon);
	of_node_put(syscon);
	if (IS_ERR(adsp->halt_map))
		return PTR_ERR(adsp->halt_map);

	ret = of_property_read_u32_index(pdev->dev.of_node, "qcom,halt-regs",
			1, &adsp->halt_lpass);
	if (ret < 0) {
		dev_err(&pdev->dev, "no offset in syscon\n");
		return ret;
	}

	return 0;
}

static int adsp_alloc_memory_region(struct qcom_adsp *adsp)
{
	struct reserved_mem *rmem = NULL;
	struct device_node *node;

	pr_info("%s:%d\n", __func__, __LINE__);

	node = of_parse_phandle(adsp->dev->of_node, "memory-region", 0);
	if (node)
		rmem = of_reserved_mem_lookup(node);
	of_node_put(node);

	if (!rmem) {
		dev_err(adsp->dev, "unable to resolve memory-region\n");
		return -EINVAL;
	}

	adsp->mem_phys = adsp->mem_reloc = rmem->base;
	adsp->mem_size = rmem->size;
	adsp->mem_region = devm_ioremap_wc(adsp->dev,
				adsp->mem_phys, adsp->mem_size);
	if (!adsp->mem_region) {
		dev_err(adsp->dev, "unable to map memory region: %pa+%zx\n",
			&rmem->base, adsp->mem_size);
		return -EBUSY;
	}

	if (!adsp->dtb_firmware_name)
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
	adsp->dtb_mem_region = devm_ioremap_wc(adsp->dev, adsp->dtb_mem_phys,
					       adsp->dtb_mem_size);
	if (!adsp->dtb_mem_region) {
		dev_err(adsp->dev, "unable to map dtb memory region: %pa+%zx\n",
			&rmem->base, adsp->dtb_mem_size);
		return -EBUSY;
	}

	return 0;
}

static int adsp_probe(struct platform_device *pdev)
{
	const struct adsp_pil_data *desc;
	const char *firmware_name;
	const char *dtb_fw_name = NULL;
	const char *rsc_fw_name = NULL;
	struct qcom_adsp *adsp;
	struct rproc *rproc;
	int ret;

	pr_info("%s:%d\n", __func__, __LINE__);

	desc = of_device_get_match_data(&pdev->dev);
	if (!desc)
		return -EINVAL;

	firmware_name = desc->firmware_name;
	ret = of_property_read_string(pdev->dev.of_node, "firmware-name",
				      &firmware_name);
	if (ret < 0 && ret != -EINVAL) {
		dev_err(&pdev->dev, "unable to read firmware-name\n");
		return ret;
	}

	if (desc->dtb_firmware_name) {
		dtb_fw_name = desc->dtb_firmware_name;
		ret = of_property_read_string_index(pdev->dev.of_node, "firmware-name", 1,
						    &dtb_fw_name);
		if (ret < 0 && ret != -EINVAL)
			return ret;
	}

	if (desc->rsc_firmware_name) {
		rsc_fw_name = desc->rsc_firmware_name;
		ret = of_property_read_string_index(pdev->dev.of_node, "firmware-name", 2,
						    &rsc_fw_name);
		if (ret < 0 && ret != -EINVAL)
			return ret;
	}

	rproc = devm_rproc_alloc(&pdev->dev, pdev->name, &adsp_ops,
				 firmware_name, sizeof(*adsp));
	if (!rproc) {
		dev_err(&pdev->dev, "unable to allocate remoteproc\n");
		return -ENOMEM;
	}

	rproc->auto_boot = desc->auto_boot;
	rproc->has_iommu = desc->has_iommu;
	rproc_coredump_set_elf_info(rproc, ELFCLASS32, EM_NONE);

	adsp = rproc->priv;
	adsp->dev = &pdev->dev;
	adsp->rproc = rproc;
	adsp->info_name = desc->sysmon_name;
	adsp->has_iommu = desc->has_iommu;
	adsp->clk_setup = desc->clk_setup;
	if (dtb_fw_name)
		adsp->dtb_firmware_name = dtb_fw_name;
	if (rsc_fw_name)
		adsp->rsc_firmware_name = rsc_fw_name;

	platform_set_drvdata(pdev, adsp);

	if (desc->is_wpss)
		adsp->shutdown = qcom_wpss_shutdown;
	else
		adsp->shutdown = qcom_adsp_shutdown;

	ret = adsp_alloc_memory_region(adsp);
	if (ret)
		return ret;

	ret = adsp_init_clock(adsp, desc->clk_ids);
	if (ret)
		return ret;

	ret = qcom_rproc_pds_attach(adsp, desc->pd_names, desc->num_pds);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to attach proxy power domains\n");
		return ret;
	}

	ret = adsp_init_reset(adsp);
	if (ret)
		goto disable_pm;

	ret = adsp_init_mmio(adsp, pdev);
	if (ret)
		goto disable_pm;

	ret = qcom_q6v5_init(&adsp->q6v5, pdev, rproc, desc->crash_reason_smem,
			     desc->load_state, qcom_adsp_pil_handover);
	if (ret)
		goto disable_pm;

	qcom_add_glink_subdev(rproc, &adsp->glink_subdev, desc->ssr_name);
	qcom_add_pdm_subdev(rproc, &adsp->pdm_subdev);
	qcom_add_ssr_subdev(rproc, &adsp->ssr_subdev, desc->ssr_name);
	adsp->sysmon = qcom_add_sysmon_subdev(rproc,
					      desc->sysmon_name,
					      desc->ssctl_id);
	if (IS_ERR(adsp->sysmon)) {
		ret = PTR_ERR(adsp->sysmon);
		goto disable_pm;
	}

	ret = rproc_add(rproc);
	if (ret)
		goto disable_pm;

	return 0;

disable_pm:
	qcom_rproc_pds_detach(adsp);

	return ret;
}

static void adsp_remove(struct platform_device *pdev)
{
	struct qcom_adsp *adsp = platform_get_drvdata(pdev);

	rproc_del(adsp->rproc);

	qcom_q6v5_deinit(&adsp->q6v5);
	qcom_remove_glink_subdev(adsp->rproc, &adsp->glink_subdev);
	qcom_remove_pdm_subdev(adsp->rproc, &adsp->pdm_subdev);
	qcom_remove_sysmon_subdev(adsp->sysmon);
	qcom_remove_ssr_subdev(adsp->rproc, &adsp->ssr_subdev);
	qcom_rproc_pds_detach(adsp);
}

static const struct adsp_pil_data adsp_resource_init = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
	.is_wpss = false,
	.auto_boot = true,
	.clk_ids = (const char*[]) {
		"sway_cbcr", "lpass_ahbs_aon_cbcr", "lpass_ahbm_aon_cbcr",
		"qdsp6ss_xo", "qdsp6ss_sleep", "qdsp6ss_core", NULL
	},
	.num_clks = 7,
	.pd_names = (const char*[]) { "cx" },
	.num_pds = 1,
};

static const struct adsp_pil_data adsp_sc7280_resource_init = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.pbn",
	.load_state = "adsp",
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
	.has_iommu = true,
	.auto_boot = true,
	.clk_ids = (const char*[]) {
		"gcc_cfg_noc_lpass", NULL
	},
	.num_clks = 1,
};

static const struct adsp_pil_data cdsp_sm8650_resource_init = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.dtb_firmware_name = "cdsp_dtb.mdt",
	.rsc_firmware_name = "cdsp_rsc.bin",
	.load_state = "cdsp",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
	.has_iommu = true,
	//.auto_boot = true,
	.clk_ids = (const char*[]) {
		"ahb",
		NULL
	},
	.num_clks = 3,
	.clk_setup = cdsp_sm8650_clk_setup,
	.pd_names = (const char*[]) { "cx", "mxc", "nsp" },
	.num_pds = 3,
};

static const struct adsp_pil_data cdsp_resource_init = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
	.is_wpss = false,
	.auto_boot = true,
	.clk_ids = (const char*[]) {
		"sway", "tbu", "bimc", "ahb_aon", "q6ss_slave", "q6ss_master",
		"q6_axim", NULL
	},
	.num_clks = 7,
	.pd_names = (const char*[]) { "cx" },
	.num_pds = 1,
};

static const struct adsp_pil_data wpss_resource_init = {
	.crash_reason_smem = 626,
	.firmware_name = "wpss.mdt",
	.ssr_name = "wpss",
	.sysmon_name = "wpss",
	.ssctl_id = 0x19,
	.is_wpss = true,
	.auto_boot = false,
	.load_state = "wpss",
	.clk_ids = (const char*[]) {
		"ahb_bdg", "ahb", "rscp", NULL
	},
	.num_clks = 3,
	.pd_names = (const char*[]) { "cx", "mx" },
	.num_pds = 2,
};

static const struct of_device_id adsp_of_match[] = {
	{ .compatible = "qcom,qcs404-cdsp-pil", .data = &cdsp_resource_init },
	{ .compatible = "qcom,sc7280-adsp-pil", .data = &adsp_sc7280_resource_init },
	{ .compatible = "qcom,sc7280-wpss-pil", .data = &wpss_resource_init },
	{ .compatible = "qcom,sdm845-adsp-pil", .data = &adsp_resource_init },
	{ .compatible = "qcom,sm8650-cdsp-pil", .data = &cdsp_sm8650_resource_init },
	{ },
};
MODULE_DEVICE_TABLE(of, adsp_of_match);

static struct platform_driver adsp_pil_driver = {
	.probe = adsp_probe,
	.remove_new = adsp_remove,
	.driver = {
		.name = "qcom_q6v5_adsp",
		.of_match_table = adsp_of_match,
	},
};

module_platform_driver(adsp_pil_driver);
MODULE_DESCRIPTION("QTI SDM845 ADSP Peripheral Image Loader");
MODULE_LICENSE("GPL v2");
