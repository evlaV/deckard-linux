/*
 * Qualcomm Computer Vision Processor (CVP) Engine for Video Analytics (EVA)
 * processor driver.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>
#include <linux/scatterlist.h>
#include <linux/dma-buf.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/iommu.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#define CVP_TOP_BASE_OFFS			0x00000000
#define CVP_SS_IDLE_STATUS		(CVP_TOP_BASE_OFFS + 0x50)

#define CVP_CPU_WD_OFFS				0x00090000

#define CVP_CPU_WD_WDOGCONTROL		(CVP_CPU_WD_OFFS + 0x8)

#define CVP_CPU_BASE_OFFS			0x000A0000
#define CVP_AON_BASE_OFFS			0x000E0000

#define CVP_CPU_CS_A2HSOFTINTEN		(CVP_CPU_BASE_OFFS + 0x10)
#define CVP_CPU_CS_A2HSOFTINTENCLR	(CVP_CPU_BASE_OFFS + 0x14)
#define CVP_CPU_CS_A2HSOFTINT		(CVP_CPU_BASE_OFFS + 0x18)
#define CVP_CPU_CS_A2HSOFTINTCLR	(CVP_CPU_BASE_OFFS + 0x1C)
#define CVP_CPU_CS_VMIMSG		(CVP_CPU_BASE_OFFS + 0x34)
#define CVP_CPU_CS_VMIMSGAG0		(CVP_CPU_BASE_OFFS + 0x38)
#define CVP_CPU_CS_VMIMSGAG1		(CVP_CPU_BASE_OFFS + 0x3C)
#define CVP_CPU_CS_VMIMSGAG2		(CVP_CPU_BASE_OFFS + 0x40)
#define CVP_CPU_CS_VMIMSGAG3		(CVP_CPU_BASE_OFFS + 0x44)
#define CVP_CPU_CS_SCIACMD		(CVP_CPU_BASE_OFFS + 0x48)
#define CVP_CPU_CS_AXI4_QOS		(CVP_CPU_BASE_OFFS + 0x13C)
#define CVP_CPU_CS_H2XSOFTINTEN		(CVP_CPU_BASE_OFFS + 0x148)
#define CVP_CPU_CS_H2XSOFTINT		(CVP_CPU_BASE_OFFS + 0x150)

/* CVP_CTRL_STATUS */
#define CVP_CPU_CS_SCIACMDARG0		(CVP_CPU_BASE_OFFS + 0x4C)
#define CVP_CPU_CS_SCIACMDARG0_BMSK	0xff
#define CVP_CPU_CS_SCIACMDARG0_SHFT	0x0
#define CVP_CPU_CS_SCIACMDARG0_HFI_CTRL_ERROR_STATUS_BMSK	0xfe
#define CVP_CPU_CS_SCIACMDARG0_HFI_CTRL_ERROR_STATUS_SHFT	0x1
#define CVP_CPU_CS_SCIACMDARG0_HFI_CTRL_INIT_STATUS_BMSK	0x1
#define CVP_CPU_CS_SCIACMDARG0_HFI_CTRL_INIT_STATUS_SHFT	0x0
#define CVP_CPU_CS_SCIACMDARG0_HFI_CTRL_PC_READY           0x100

/* HFI_QTBL_INFO */
#define CVP_CPU_CS_SCIACMDARG1		(CVP_CPU_BASE_OFFS + 0x50)

/* HFI_QTBL_ADDR */
#define CVP_CPU_CS_SCIACMDARG2		(CVP_CPU_BASE_OFFS + 0x54)

/* HFI_VERSION_INFO */
#define CVP_CPU_CS_SCIACMDARG3		(CVP_CPU_BASE_OFFS + 0x58)

/* CVP_SFR_ADDR */
#define CVP_CPU_CS_SCIBCMD		(CVP_CPU_BASE_OFFS + 0x5C)

/* CVP_MMAP_ADDR */
#define CVP_CPU_CS_SCIBCMDARG0		(CVP_CPU_BASE_OFFS + 0x60)

/* CVP_UC_REGION_ADDR */
#define CVP_CPU_CS_SCIBARG1		(CVP_CPU_BASE_OFFS + 0x64)

/* CVP_UC_REGION_ADDR */
#define CVP_CPU_CS_SCIBARG2		(CVP_CPU_BASE_OFFS + 0x68)

#define CVP_CPU_CS_SCIBARG3		(CVP_CPU_BASE_OFFS + 0x6C)

#define CVP_CPU_CS_H2ASOFTINTEN		(CVP_CPU_BASE_OFFS + 0x148)
#define CVP_CPU_CS_H2ASOFTINTENCLR	(CVP_CPU_BASE_OFFS + 0x14c)
#define CVP_CPU_CS_H2ASOFTINT		(CVP_CPU_BASE_OFFS + 0x150)
#define CVP_CPU_CS_H2ASOFTINTCLR	(CVP_CPU_BASE_OFFS + 0x154)

#define CVP_AHB_BRIDGE_SYNC_RESET	(CVP_CPU_BASE_OFFS + 0x160)

/* FAL10 Feature Control */
#define CVP_CPU_CS_X2RPMh		(CVP_CPU_BASE_OFFS + 0x168)
#define CVP_CPU_CS_X2RPMh_MASK0_BMSK	0x1
#define CVP_CPU_CS_X2RPMh_MASK0_SHFT	0x0
#define CVP_CPU_CS_X2RPMh_MASK1_BMSK	0x2
#define CVP_CPU_CS_X2RPMh_MASK1_SHFT	0x1
#define CVP_CPU_CS_X2RPMh_SWOVERRIDE_BMSK	0x4
#define CVP_CPU_CS_X2RPMh_SWOVERRIDE_SHFT	0x3

#define CVP_CPU_CS_X2RPMh_STATUS	(CVP_CPU_BASE_OFFS + 0x170)

/*
 * --------------------------------------------------------------------------
 * MODULE: cvp_wrapper
 * --------------------------------------------------------------------------
 */
#define CVP_WRAPPER_BASE_OFFS		0x000B0000

#define CVP_WRAPPER_HW_VERSION		(CVP_WRAPPER_BASE_OFFS + 0x00)
#define CVP_WRAPPER_HW_VERSION_MAJOR_VERSION_MASK  0x78000000
#define CVP_WRAPPER_HW_VERSION_MAJOR_VERSION_SHIFT 28
#define CVP_WRAPPER_HW_VERSION_MINOR_VERSION_MASK  0xFFF0000
#define CVP_WRAPPER_HW_VERSION_MINOR_VERSION_SHIFT 16
#define CVP_WRAPPER_HW_VERSION_STEP_VERSION_MASK   0xFFFF

#define CVP_WRAPPER_INTR_STATUS	(CVP_WRAPPER_BASE_OFFS + 0x0C)
#define CVP_WRAPPER_INTR_STATUS_A2HWD_BMSK	0x8
#define CVP_WRAPPER_INTR_STATUS_A2H_BMSK	0x4

#define CVP_SS_IRQ_MASK   		(CVP_TOP_BASE_OFFS + 0x04)
#define CVP_SS_INTR_BMASK  		(0x100)
#define CVP_WRAPPER_INTR_MASK		(CVP_WRAPPER_BASE_OFFS + 0x10)
#define CVP_FATAL_INTR_BMSK		(CVP_WRAPPER_INTR_MASK_CPU_NOC_BMSK | \
					 CVP_WRAPPER_INTR_MASK_CORE_NOC_BMSK )
#define CVP_WRAPPER_INTR_MASK_CPU_NOC_BMSK	0x40
#define CVP_WRAPPER_INTR_MASK_CORE_NOC_BMSK	0x20
#define CVP_WRAPPER_INTR_MASK_A2HWD_BMSK	0x8
#define CVP_WRAPPER_INTR_MASK_A2HCPU_BMSK	0x4
#define CVP_WRAPPER_INTR_MASK_A2HCPU_SHFT	0x2

#define CVP_WRAPPER_INTR_CLEAR		(CVP_WRAPPER_BASE_OFFS + 0x14)

#define CVP_WRAPPER_TZ_BASE_OFFS		0x000C0000

#define CVP_WRAPPER_TZ_CPU_CLOCK_CONFIG	(CVP_WRAPPER_TZ_BASE_OFFS)
#define CVP_WRAPPER_INTR_CLEAR_A2HWD_BMSK	0x10
#define CVP_WRAPPER_INTR_CLEAR_A2HWD_SHFT	0x4
#define CVP_WRAPPER_INTR_CLEAR_A2H_BMSK		0x4
#define CVP_WRAPPER_INTR_CLEAR_A2H_SHFT		0x2
#define CVP_WRAPPER_CPU_STATUS		(CVP_WRAPPER_TZ_BASE_OFFS + 0x10)
#define CVP_WRAPPER_AXI_CLOCK_CONFIG	(CVP_WRAPPER_TZ_BASE_OFFS + 0x14)
#define CVP_WRAPPER_QNS4PDXFIFO_RESET	(CVP_WRAPPER_TZ_BASE_OFFS + 0x18)
#define CVP_WRAPPER_CPU_CGC_DIS		(CVP_WRAPPER_BASE_OFFS + 0x2010)

#define CVP_WRAPPER_CPU_CLOCK_CONFIG	(CVP_WRAPPER_BASE_OFFS + 0x50)
#define CVP_WRAPPER_DEBUG_BRIDGE_LPI_CONTROL	(CVP_WRAPPER_BASE_OFFS + 0x54)
#define CVP_WRAPPER_DEBUG_BRIDGE_LPI_STATUS	(CVP_WRAPPER_BASE_OFFS + 0x58)
#define CVP_WRAPPER_CPU_NOC_LPI_CONTROL		(CVP_WRAPPER_BASE_OFFS + 0x5C)
#define CVP_WRAPPER_CPU_NOC_LPI_STATUS		(CVP_WRAPPER_BASE_OFFS + 0x60)
#define CVP_WRAPPER_CORE_POWER_STATUS		(CVP_WRAPPER_BASE_OFFS + 0x80)
#define CVP_WRAPPER_CORE_POWER_CONTROL		(CVP_WRAPPER_BASE_OFFS + 0x84)
#define CVP_WRAPPER_CORE_CLOCK_CONFIG		(CVP_WRAPPER_BASE_OFFS + 0x88)

#define CVP_WRAPPER_TZ_XTSS_SW_RESET		(CVP_WRAPPER_TZ_BASE_OFFS + 0x1000)
#define CVP_WRAPPER_TZ_SEC_CPA_START_ADDR	(CVP_WRAPPER_TZ_BASE_OFFS + 0x1020)
#define CVP_WRAPPER_TZ_SEC_CPA_END_ADDR		(CVP_WRAPPER_TZ_BASE_OFFS + 0x1024)
#define CVP_WRAPPER_TZ_SEC_FW_START_ADDR	(CVP_WRAPPER_TZ_BASE_OFFS + 0x1028)
#define CVP_WRAPPER_TZ_SEC_FW_END_ADDR 		(CVP_WRAPPER_TZ_BASE_OFFS + 0x102c)
#define CVP_WRAPPER_TZ_SEC_NONPIX_START_ADDR 	(CVP_WRAPPER_TZ_BASE_OFFS + 0x1030)
#define CVP_WRAPPER_TZ_SEC_NONPIX_END_ADDR 	(CVP_WRAPPER_TZ_BASE_OFFS + 0x1034)

#define CVP_CTRL_INIT			CVP_CPU_CS_SCIACMD

#define CVP_CTRL_STATUS	CVP_CPU_CS_SCIACMDARG0
#define CVP_CTRL_INIT_STATUS__M		CVP_CPU_CS_SCIACMDARG0_HFI_CTRL_INIT_STATUS_BMSK
#define CVP_CTRL_ERROR_STATUS__M	CVP_CPU_CS_SCIACMDARG0_HFI_CTRL_ERROR_STATUS_BMSK
#define CVP_CTRL_INIT_IDLE_MSG_BMSK	CVP_CPU_CS_SCIACMDARG0_HFI_CTRL_INIT_IDLE_MSG_BMSK
#define CVP_CTRL_STATUS_PC_READY	CVP_CPU_CS_SCIACMDARG0_HFI_CTRL_PC_READY

#define CVP_QTBL_INFO			CVP_CPU_CS_SCIACMDARG1

#define CVP_QTBL_ADDR			CVP_CPU_CS_SCIACMDARG2

#define CVP_VERSION_INFO		CVP_CPU_CS_SCIACMDARG3

#define CVP_SFR_ADDR			CVP_CPU_CS_SCIBCMD
#define CVP_MMAP_ADDR			CVP_CPU_CS_SCIBCMDARG0
#define CVP_UC_REGION_ADDR		CVP_CPU_CS_SCIBARG1
#define CVP_UC_REGION_SIZE		CVP_CPU_CS_SCIBARG2

/* HFI_DSP_QTBL_ADDR
 * 31:3 - HFI_DSP_QTBL_ADDR
 *        4-byte aligned Address
 */
#define HFI_DSP_QTBL_ADDR		CVP_CPU_CS_VMIMSG

/* HFI_DSP_UC_REGION_ADDR
 * 31:20 - HFI_DSP_UC_REGION_ADDR
 *         1MB aligned address.
 *         Uncached Region start Address. This region covers
 *         HFI DSP QTable,
 *         HFI DSP Queue Headers,
 *         HFI DSP Queues,
 */
#define HFI_DSP_UC_REGION_ADDR		CVP_CPU_CS_VMIMSGAG0

/* HFI_DSP_UC_REGION_SIZE
 * 31:20 - HFI_DSP_UC_REGION_SIZE
 *         Multiples of 1MB.
 *         Size of the DSP_UC_REGION Uncached Region
 */
#define HFI_DSP_UC_REGION_SIZE		CVP_CPU_CS_VMIMSGAG1

/*
 * --------------------------------------------------------------------------
 * MODULE: vcodec noc error log registers
 * --------------------------------------------------------------------------
 */
#define CVP_NOC_BASE_OFFS		0x000D0000
#define CVP_NOC_ERR_SWID_LOW_OFFS	(CVP_NOC_BASE_OFFS + 0x0)
#define CVP_NOC_ERR_SWID_HIGH_OFFS	(CVP_NOC_BASE_OFFS + 0x4)
#define CVP_NOC_ERR_MAINCTL_LOW_OFFS	(CVP_NOC_BASE_OFFS + 0x8)
#define CVP_NOC_ERR_ERRVLD_LOW_OFFS	(CVP_NOC_BASE_OFFS + 0x10)
#define CVP_NOC_ERR_ERRCLR_LOW_OFFS	(CVP_NOC_BASE_OFFS + 0x18)
#define CVP_NOC_ERR_ERRLOG0_LOW_OFFS	(CVP_NOC_BASE_OFFS + 0x20)
#define CVP_NOC_ERR_ERRLOG0_HIGH_OFFS	(CVP_NOC_BASE_OFFS + 0x24)
#define CVP_NOC_ERR_ERRLOG1_LOW_OFFS	(CVP_NOC_BASE_OFFS + 0x28)
#define CVP_NOC_ERR_ERRLOG1_HIGH_OFFS	(CVP_NOC_BASE_OFFS + 0x2C)
#define CVP_NOC_ERR_ERRLOG2_LOW_OFFS	(CVP_NOC_BASE_OFFS + 0x30)
#define CVP_NOC_ERR_ERRLOG2_HIGH_OFFS	(CVP_NOC_BASE_OFFS + 0x34)
#define CVP_NOC_ERR_ERRLOG3_LOW_OFFS	(CVP_NOC_BASE_OFFS + 0x38)
#define CVP_NOC_ERR_ERRLOG3_HIGH_OFFS	(CVP_NOC_BASE_OFFS + 0x3C)
#define CVP_NOC_SBM_SENSELN0_LOW	(CVP_NOC_BASE_OFFS + 0x300)

#define CVP_NOC_CORE_BASE_OFFS			0x00010000
#define CVP_NOC_RGE_NIU_DECCTL_LOW	(CVP_NOC_CORE_BASE_OFFS + 0x3108)
#define CVP_NOC_RGE_NIU_ENCCTL_LOW 	(CVP_NOC_CORE_BASE_OFFS + 0x3188)
#define CVP_NOC_GCE_VADL_TOF_NIU_DECCTL_LOW (CVP_NOC_CORE_BASE_OFFS + 0x3508)
#define CVP_NOC_GCE_VADL_TOF_NIU_ENCCTL_LOW (CVP_NOC_CORE_BASE_OFFS + 0x3588)
#define CVP_NOC_MAIN_SIDEBANDMANAGER_FAULTINEN0_LOW	(CVP_NOC_CORE_BASE_OFFS + 0x7040)
#define CVP_NOC_MAIN_SIDEBANDMANAGER_SENSELN0_LOW	(CVP_NOC_CORE_BASE_OFFS + 0x7100)
#define CVP_NOC_MAIN_SIDEBANDMANAGER_SENSELN0_HIGH	(CVP_NOC_CORE_BASE_OFFS + 0x7104)
#define CVP_NOC_MAIN_SIDEBANDMANAGER_SENSELN1_HIGH	(CVP_NOC_CORE_BASE_OFFS + 0x710C)
#define CVP_NOC_CORE_ERR_SWID_LOW_OFFS		(CVP_NOC_CORE_BASE_OFFS + 0xA000)
#define CVP_NOC_CORE_ERR_SWID_HIGH_OFFS		(CVP_NOC_CORE_BASE_OFFS + 0xA004)
#define CVP_NOC_CORE_ERR_MAINCTL_LOW_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA008)
#define CVP_NOC_CORE_ERR_ERRVLD_LOW_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA010)
#define CVP_NOC_CORE_ERR_ERRCLR_LOW_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA018)
#define CVP_NOC_CORE_ERR_ERRLOG0_LOW_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA020)
#define CVP_NOC_CORE_ERR_ERRLOG0_HIGH_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA024)
#define CVP_NOC_CORE_ERR_ERRLOG1_LOW_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA028)
#define CVP_NOC_CORE_ERR_ERRLOG1_HIGH_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA02C)
#define CVP_NOC_CORE_ERR_ERRLOG2_LOW_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA030)
#define CVP_NOC_CORE_ERR_ERRLOG2_HIGH_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA034)
#define CVP_NOC_CORE_ERR_ERRLOG3_LOW_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA038)
#define CVP_NOC_CORE_ERR_ERRLOG3_HIGH_OFFS	(CVP_NOC_CORE_BASE_OFFS + 0xA03C)

#define CVP_NOC_RCG_VNOC_NOC_CLK_FORCECLOCKON_LOW	(CVP_NOC_CORE_BASE_OFFS + 0x2018)

/* NoC QoS registers */
#define CVP_NOC_RGE_PRIORITYLUT_LOW	(CVP_NOC_CORE_BASE_OFFS + 0x3030)
#define CVP_NOC_RGE_PRIORITYLUT_HIGH 	(CVP_NOC_CORE_BASE_OFFS + 0x3034)
#define CVP_NOC_RGE_URGENCY_LOW		(CVP_NOC_CORE_BASE_OFFS + 0x3038)
#define CVP_NOC_RGE_DANGERLUT_LOW	(CVP_NOC_CORE_BASE_OFFS + 0x3040)
#define CVP_NOC_RGE_SAFELUT_LOW		(CVP_NOC_CORE_BASE_OFFS + 0x3048)
#define CVP_NOC_GCE_PRIORITYLUT_LOW	(CVP_NOC_CORE_BASE_OFFS + 0x3430)
#define CVP_NOC_GCE_PRIORITYLUT_HIGH	(CVP_NOC_CORE_BASE_OFFS + 0x3434)
#define CVP_NOC_GCE_URGENCY_LOW		(CVP_NOC_CORE_BASE_OFFS + 0x3438)
#define CVP_NOC_GCE_DANGERLUT_LOW	(CVP_NOC_CORE_BASE_OFFS + 0x3440)
#define CVP_NOC_GCE_SAFELUT_LOW		(CVP_NOC_CORE_BASE_OFFS + 0x3448)
#define CVP_NOC_CDM_PRIORITYLUT_LOW	(CVP_NOC_CORE_BASE_OFFS + 0x3830)
#define CVP_NOC_CDM_PRIORITYLUT_HIGH	(CVP_NOC_CORE_BASE_OFFS + 0x3834)
#define CVP_NOC_CDM_URGENCY_LOW		(CVP_NOC_CORE_BASE_OFFS + 0x3838)
#define CVP_NOC_CDM_DANGERLUT_LOW	(CVP_NOC_CORE_BASE_OFFS + 0x3840)
#define CVP_NOC_CDM_SAFELUT_LOW		(CVP_NOC_CORE_BASE_OFFS + 0x3848)


/* End of NoC Qos */

#define CVP_NOC_RCGCONTROLLER_MAINCTL_LOW \
		(CVP_NOC_CORE_BASE_OFFS + 0xC008)
#define CVP_NOC_RCGCONTROLLER_HYSTERESIS_LOW \
		(CVP_NOC_CORE_BASE_OFFS + 0xC010)
#define CVP_NOC_RESET_REQ \
		(CVP_NOC_CORE_BASE_OFFS + 0xf000)
#define CVP_NOC_RESET_ACK \
		(CVP_NOC_CORE_BASE_OFFS + 0xf004)


#define CVP_AON_WRAPPER_CVP_NOC_LPI_CONTROL	(CVP_AON_BASE_OFFS + 0x8)
#define CVP_AON_WRAPPER_CVP_NOC_LPI_STATUS	(CVP_AON_BASE_OFFS + 0xC)
#define CVP_AON_WRAPPER_CVP_NOC_ARCG_CONTROL	(CVP_AON_BASE_OFFS + 0x14)
#define CVP_AON_WRAPPER_CVP_NOC_CORE_CLK_CONTROL	(CVP_AON_BASE_OFFS + 0x24)
#define CVP_AON_WRAPPER_CVP_NOC_CORE_SW_RESET	(CVP_AON_BASE_OFFS + 0x1C)
#define CVP_AON_WRAPPER_SPARE		(CVP_AON_BASE_OFFS + 0x28)

/* IOCTLs */

/* note: cvp fd will have POLLIN event if there are any pending ipcc signals */
#define CVP_IOCTL_IRQ_STATUS 	_IO('C', 0) /* returns mask of pending ipcc signals and clears */
#define CVP_IOCTL_IRQ_SEND  	_IO('C', 1) /* send an ipcc signal, argument is integer in 0-3 range */
#define CVP_IOCTL_SHUTDOWN 	_IO('C', 3) /* shutdown DSP. no argument */
#define CVP_IOCTL_MAP_DMABUF 	_IO('C', 4) /* 64-bit argument: low 32-bits dmabuf fd, high 32-bits VA */
#define CVP_IOCTL_MAP_DIRECT 	_IOW('C', 5, struct map_direct)
#define CVP_IOCTL_UNMAP 	_IO('C', 6) /* 32 bit va argument */
#define CVP_IOCTL_BOOT 		_IO('C', 7) /* argument ignored */

enum {
	PM_DOMAIN_MXC = 0,
	PM_DOMAIN_MMCX,
	PM_DOMAIN_CTRL,
	PM_DOMAIN_HW,
	PM_DOMAIN_COUNT,
};

enum {
	RESET_BUS = 0,
	RESET_XO,
	RESET_CORE,
	RESET_COUNT
};

enum {
	CLK_IFACE = 0,
	CLK_CORE,
	CLK_HW,
	CLK_COUNT
};

struct cvp {
	struct miscdevice miscdev;
	wait_queue_head_t waitqueue;
	atomic_t pending;
	spinlock_t lock;
	struct list_head maps;
	atomic_t open, booted;
	struct iommu_domain *iommu;
	void __iomem *base;
	u32 mem_phys;
	u32 mem_size;
	struct device *pds[PM_DOMAIN_COUNT];
	int perf[PM_DOMAIN_COUNT];
	struct reset_control *resets[RESET_COUNT];
	struct clk *clks[CLK_COUNT];
};

struct dma_map {
	struct list_head node;
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *table;
	u32 iova;
	u32 size;
};

struct map_direct {
	u32 va;
	u32 pa;
	u32 size;
};

#define CVP_FZ_SIZE	(7 * SZ_1M)
#define QTLB_BUF_ADDR	0x80000000
#define QTLB_BUF_SIZE	(3 * SZ_1M)	// Corresponds to SHARED_QSIZE which is 3Mb

static int cvp_interrupts_init(struct cvp *cvp)
{
	u32 mask_val;

	/* All interrupts should be disabled initially 0x1F6 : Reset value */
	mask_val = readl(cvp->base + CVP_WRAPPER_INTR_MASK);

	/* Write 0 to unmask CPU and WD interrupts */
	mask_val &= ~(CVP_WRAPPER_INTR_MASK_A2HWD_BMSK |
		      CVP_WRAPPER_INTR_MASK_A2HCPU_BMSK);
	writel(mask_val, cvp->base + CVP_WRAPPER_INTR_MASK);

#if 0
	mask_val = 0;
	mask_val = readl(cvp->base + CVP_SS_IRQ_MASK);
	mask_val &= ~(CVP_SS_INTR_BMASK);
	writel(mask_val, cvp->base + CVP_SS_IRQ_MASK);
#endif

	return 0;
}

static int cvp_boot_fw(struct cvp *cvp)
{
	int ret;
	u32 val;

	// TODO Hand off control of regulators to h/w

	// BIT(3) disables SYNX
	writel(BIT(0) | BIT(3), cvp->base + CVP_CPU_CS_SCIACMD);

	ret = cvp_interrupts_init(cvp);
	if (ret)
		return ret;

	ret = read_poll_timeout(readl, val, (val & BIT(0)), 500, 500000,
				false, cvp->base + CVP_CPU_CS_SCIACMDARG0);

	if (ret)
		dev_warn(cvp->miscdev.parent, "CVP boot failed with boot status 0x%x (interrupt status: 0x%x) \n",
			val, readl(cvp->base + CVP_WRAPPER_INTR_STATUS));

	writel(1, cvp->base + CVP_CPU_CS_H2XSOFTINTEN);
	writel(0, cvp->base + CVP_CPU_CS_X2RPMh);

	return 0;
}

static int cvp_load_fw(struct cvp *cvp)
{
	writel(0, cvp->base + CVP_WRAPPER_TZ_SEC_FW_START_ADDR);
	writel(CVP_FZ_SIZE, cvp->base + CVP_WRAPPER_TZ_SEC_FW_END_ADDR);
	writel(0, cvp->base + CVP_WRAPPER_TZ_SEC_CPA_START_ADDR);
	writel(0x4b000000, cvp->base + CVP_WRAPPER_TZ_SEC_CPA_END_ADDR);
	writel(0x01000000, cvp->base + CVP_WRAPPER_TZ_SEC_NONPIX_START_ADDR);
	writel(0x26800000, cvp->base + CVP_WRAPPER_TZ_SEC_NONPIX_END_ADDR);

	return 0;
}

static int cvp_setup_cvp_uc_memmap(struct cvp *cvp)
{
	u32 addr = QTLB_BUF_ADDR + QTLB_BUF_SIZE + SZ_1M;

	/* initialize DSP QTBL & UCREGION with CPU queues */
	writel(addr, cvp->base + HFI_DSP_QTBL_ADDR);
	writel(addr, cvp->base + HFI_DSP_UC_REGION_ADDR);
	writel(QTLB_BUF_SIZE, cvp->base + HFI_DSP_UC_REGION_SIZE);

	return 0;
}

static int cvp_setup_cvp_uc_region(struct cvp *cvp)
{
	writel(0, cvp->base + CVP_CTRL_STATUS);

	writel(QTLB_BUF_ADDR, cvp->base + CVP_UC_REGION_ADDR);
	writel(QTLB_BUF_SIZE, cvp->base + CVP_UC_REGION_SIZE);
	writel(QTLB_BUF_ADDR, cvp->base + CVP_QTBL_ADDR);
	writel(1, cvp->base + CVP_QTBL_INFO);
	writel(1, cvp->base + CVP_VERSION_INFO);
	writel(QTLB_BUF_ADDR + QTLB_BUF_SIZE, cvp->base + CVP_SFR_ADDR);
	// CVP_MMAP_ADDR ?

	return cvp_setup_cvp_uc_memmap(cvp);
}

static int cvp_set_registers(struct cvp *cvp)
{
	int ret;

	writel(0, cvp->base + CVP_WRAPPER_CORE_CLOCK_CONFIG);

#if 0
	// RESET_XO is shared with iris, but this causes the iris fw to shutdown
	// keep this commented until we figure out how to use this
	ret = reset_control_acquire(cvp->resets[RESET_XO]);
	if (ret) {
		dev_warn(cvp->miscdev.parent, "%s: failed to acquire XO reset\n", __func__);
		return 0;
	}

	reset_control_assert(cvp->resets[RESET_XO]);

	writel(0x99, cvp->base + CVP_CPU_CS_AXI4_QOS);
	writel(0x33333333, cvp->base + CVP_NOC_RGE_PRIORITYLUT_LOW);
	writel(0x33333333, cvp->base + CVP_NOC_RGE_PRIORITYLUT_HIGH);
	writel(0x1033, cvp->base + CVP_NOC_RGE_URGENCY_LOW);
	writel(0x0, cvp->base + CVP_NOC_RGE_DANGERLUT_LOW);
	writel(0xffff, cvp->base + CVP_NOC_RGE_SAFELUT_LOW);
	writel(0x33333333, cvp->base + CVP_NOC_GCE_PRIORITYLUT_LOW);
	writel(0x33333333, cvp->base + CVP_NOC_GCE_PRIORITYLUT_HIGH);
	writel(0x1033, cvp->base + CVP_NOC_GCE_URGENCY_LOW);
	writel(0x0, cvp->base + CVP_NOC_GCE_DANGERLUT_LOW);
	writel(0xffff, cvp->base + CVP_NOC_GCE_SAFELUT_LOW);
	writel(0x33333333, cvp->base + CVP_NOC_CDM_PRIORITYLUT_LOW);
	writel(0x33333333, cvp->base + CVP_NOC_CDM_PRIORITYLUT_HIGH);
	writel(0x1033, cvp->base + CVP_NOC_CDM_URGENCY_LOW);
	writel(0x0, cvp->base + CVP_NOC_CDM_DANGERLUT_LOW);
	writel(0xffff, cvp->base + CVP_NOC_CDM_SAFELUT_LOW);

	/* Below registers write moved from FW to SW to enable UBWC */
	writel(0x1, cvp->base + CVP_NOC_RGE_NIU_DECCTL_LOW);
	writel(0x1, cvp->base + CVP_NOC_RGE_NIU_ENCCTL_LOW);
	writel(0x1, cvp->base + CVP_NOC_GCE_VADL_TOF_NIU_DECCTL_LOW);
	writel(0x1, cvp->base + CVP_NOC_GCE_VADL_TOF_NIU_ENCCTL_LOW);
	writel(0x3, cvp->base + CVP_NOC_CORE_ERR_MAINCTL_LOW_OFFS);
	writel(0x1, cvp->base + CVP_NOC_MAIN_SIDEBANDMANAGER_FAULTINEN0_LOW);

	reset_control_deassert(cvp->resets[RESET_XO]);
	reset_control_release(cvp->resets[RESET_XO]);
#endif

	return 0;
}

static int cvp_power_on_core(struct cvp *cvp)
{
	int ret;

	ret = pm_runtime_get_sync(cvp->pds[PM_DOMAIN_HW]);
	if (ret)
		dev_err(cvp->miscdev.parent, "%s: failed to enable PM_DOMAIN_HW (%d)\n", __func__, ret);

	ret = clk_enable(cvp->clks[CLK_HW]);
	if (ret)
		dev_err(cvp->miscdev.parent, "%s: failed to enable CLK_HW (%d)\n", __func__, ret);

	dev_info(cvp->miscdev.parent, "%s: EVA core powered on\n", __func__);

	return 0;
}

static int cvp_power_on_controller(struct cvp *cvp)
{
	int ret;

	ret = pm_runtime_get_sync(cvp->pds[PM_DOMAIN_CTRL]);
	if (ret)
		dev_err(cvp->miscdev.parent, "%s: failed to enable PM_DOMAIN_CTRL (%d)\n", __func__, ret);

	// sleep_clk ?

	reset_control_assert(cvp->resets[RESET_BUS]);
	reset_control_assert(cvp->resets[RESET_CORE]);

	usleep_range(300, 400);

	reset_control_deassert(cvp->resets[RESET_CORE]);
	reset_control_deassert(cvp->resets[RESET_BUS]);

	ret = clk_enable(cvp->clks[CLK_IFACE]);
	if (ret)
		dev_err(cvp->miscdev.parent, "%s: failed to enable CLK_IFACE (%d)\n", __func__, ret);

	ret = clk_enable(cvp->clks[CLK_CORE]);
	if (ret)
		dev_err(cvp->miscdev.parent, "%s: failed to enable CLK_CORE (%d)\n", __func__, ret);

	dev_dbg(cvp->miscdev.parent, "%s: EVA controller powered on\n", __func__);

	return 0;
}

static int cvp_power_on(struct cvp *cvp)
{
	int ret;
	u32 reg;

	ret = pm_runtime_get_sync(cvp->pds[PM_DOMAIN_MXC]);
	if (ret) {
		dev_err(cvp->miscdev.parent, "%s: failed to enable PM_DOMAIN_MXC (%d)\n", __func__, ret);
		return ret;
	}
	ret = pm_runtime_get_sync(cvp->pds[PM_DOMAIN_MMCX]);
	if (ret) {
		dev_err(cvp->miscdev.parent, "%s: failed to enable PM_DOMAIN_MMCX (%d)\n", __func__, ret);
		return ret;
	}
	ret = dev_pm_genpd_set_performance_state(cvp->pds[PM_DOMAIN_MXC], cvp->perf[PM_DOMAIN_MXC]);
	if (ret) {
		dev_err(cvp->miscdev.parent, "%s: failed to set perf of PM_DOMAIN_MXC (%d)\n", __func__, ret);
		return ret;
	}
	ret = dev_pm_genpd_set_performance_state(cvp->pds[PM_DOMAIN_MMCX], cvp->perf[PM_DOMAIN_MMCX]);
	if (ret) {
		dev_err(cvp->miscdev.parent, "%s: failed to set perf of DOMAIN_CTRL (%d)\n", __func__, ret);
		return ret;
	}

	ret = cvp_power_on_controller(cvp);
	if (ret)
		return ret;

	ret = cvp_power_on_core(cvp);
	if (ret)
		return ret;

	// msm_cvp_scale_clocks ??

	ret = cvp_set_registers(cvp);
	if (ret)
		return ret;

	reg = readl(cvp->base + CVP_AON_WRAPPER_SPARE);
	if ((reg & BIT(1)))
		dev_warn(cvp->miscdev.parent, "%s: WRAPPER_SPARE non-zero %x\n", __func__, reg);

	writel(0x7, cvp->base + CVP_WRAPPER_DEBUG_BRIDGE_LPI_CONTROL);

	dev_info(cvp->miscdev.parent, "%s: cvp (eva) powered on\n", __func__);

	return 0;
}

static int cvp_power_on_init(struct cvp *cvp)
{
	int ret;

	ret = cvp_power_on(cvp);
	if (ret)
		return ret;

	/* Bring XTSS out of reset */
	writel(0, cvp->base + CVP_WRAPPER_TZ_XTSS_SW_RESET);

	writel(1, cvp->base + CVP_CPU_CS_A2HSOFTINTCLR);
	writel(1, cvp->base + CVP_CPU_CS_H2ASOFTINTCLR);

	ret = cvp_setup_cvp_uc_region(cvp);
	if (ret)
		return ret;

	ret = cvp_load_fw(cvp);
	if (ret)
		return ret;

	ret = cvp_boot_fw(cvp);
	if (ret)
		return ret;

	writel(0, cvp->base + CVP_WRAPPER_CORE_POWER_CONTROL); // CORE_PWR_DISABLE=0

	ret = readl(cvp->base + CVP_CPU_CS_SCIACMDARG0);

	dev_info(cvp->miscdev.parent, "%s: status %x\n", __func__, ret);

	atomic_set(&cvp->booted, 1);

	return ret;
}

static int cvp_power_off_controller(struct cvp *cvp)
{
	int ret;
	u32 reg;

	writel(0x3, cvp->base + CVP_CPU_CS_X2RPMh);

	reg = readl(cvp->base + CVP_WRAPPER_CPU_NOC_LPI_CONTROL);
	writel(reg | BIT(0), cvp->base + CVP_WRAPPER_CPU_NOC_LPI_CONTROL);

	ret = read_poll_timeout(readl, reg, reg & BIT(0), 100, 2000, false,
				cvp->base + CVP_WRAPPER_CPU_NOC_LPI_STATUS);
	if (ret) {
		dev_warn(cvp->miscdev.parent, "CVP_WRAPPER_IRIS_CPU_NOC_LPI_STATUS timeout (%x)", reg);
		goto disable_controller_power;
	}

	writel(0x0, cvp->base + CVP_WRAPPER_DEBUG_BRIDGE_LPI_CONTROL);

	ret = read_poll_timeout(readl, reg, reg == 0, 100, 2000, false,
				cvp->base + CVP_WRAPPER_DEBUG_BRIDGE_LPI_STATUS);
	if (ret) {
		dev_warn(cvp->miscdev.parent, "CVP_WRAPPER_DEBUG_BRIDGE_LPI_STATUS timeout (%x)", reg);
		goto disable_controller_power;
	}

	writel(0x3, cvp->base + CVP_WRAPPER_AXI_CLOCK_CONFIG);
	writel(0x1, cvp->base + CVP_WRAPPER_QNS4PDXFIFO_RESET);
	writel(0x0, cvp->base + CVP_WRAPPER_QNS4PDXFIFO_RESET);
	writel(0x0, cvp->base + CVP_WRAPPER_AXI_CLOCK_CONFIG);

	reset_control_assert(cvp->resets[RESET_BUS]);
	reset_control_assert(cvp->resets[RESET_CORE]);

	usleep_range(1000, 1100);

	reset_control_deassert(cvp->resets[RESET_CORE]);
	reset_control_deassert(cvp->resets[RESET_BUS]);

	/* Disable CVP NoC clock */
	reg = readl(cvp->base + CVP_AON_WRAPPER_CVP_NOC_CORE_CLK_CONTROL);
	writel(reg | BIT(0), cvp->base + CVP_AON_WRAPPER_CVP_NOC_CORE_CLK_CONTROL);

	/* enable CVP NoC reset */
	reg = readl(cvp->base + CVP_AON_WRAPPER_CVP_NOC_CORE_SW_RESET);
	writel(reg | BIT(0), cvp->base + CVP_AON_WRAPPER_CVP_NOC_CORE_SW_RESET);

#if 0
	// RESET_XO is shared with iris, but this causes the iris fw to shutdown
	// keep this commented until we figure out how to use this
	ret = reset_control_acquire(cvp->resets[RESET_XO]);
	if (ret) {
		dev_warn(cvp->miscdev.parent, "%s: failed to acquire XO reset\n", __func__);
		goto skip_xo_reset;
	}
#endif

	/* poll AON spare register bit1 to become zero with 50ms timeout */
	ret = read_poll_timeout(readl, reg, (reg & BIT(1)) == 0, 1000, 50000, false,
				cvp->base + CVP_AON_WRAPPER_SPARE);
	if (ret) {
		dev_warn(cvp->miscdev.parent, "AON_WRAPPER_SPARE timeout (%x)", reg);
		reset_control_release(cvp->resets[RESET_XO]);
		goto skip_xo_reset;
	}

	reg = readl(cvp->base + CVP_AON_WRAPPER_SPARE);
	writel(reg | BIT(0), cvp->base + CVP_AON_WRAPPER_SPARE);

	//reset_control_assert(cvp->resets[RESET_XO]);

	/* De-assert CVP NoC reset */
	reg = readl(cvp->base + CVP_AON_WRAPPER_CVP_NOC_CORE_SW_RESET);
	writel(reg & ~BIT(0), cvp->base + CVP_AON_WRAPPER_CVP_NOC_CORE_SW_RESET);

	/* de-assert EVA video_cc XO reset and enable video_cc XO clock after 80us */
	usleep_range(80, 100);
	//reset_control_deassert(cvp->resets[RESET_XO]);

	/* clear XO mask bit - this step was missing in previous sequence */
	reg = readl(cvp->base + CVP_AON_WRAPPER_SPARE);
	writel(reg & ~BIT(0), cvp->base + CVP_AON_WRAPPER_SPARE);

	//reset_control_release(cvp->resets[RESET_XO]);

skip_xo_reset:

	/* Enable CVP NoC clock */
	reg = readl(cvp->base + CVP_AON_WRAPPER_CVP_NOC_CORE_CLK_CONTROL);
	writel(reg & ~BIT(0), cvp->base + CVP_AON_WRAPPER_CVP_NOC_CORE_CLK_CONTROL);

	usleep_range(400, 500);

disable_controller_power:
	clk_disable(cvp->clks[CLK_CORE]);

	/* power down process */
	pm_runtime_put_sync(cvp->pds[PM_DOMAIN_CTRL]);

	clk_disable(cvp->clks[CLK_IFACE]);

	dev_dbg(cvp->miscdev.parent, "%s: EVA controller powered off\n", __func__);

	return 0;
}

static int cvp_power_off_hardware(struct cvp *cvp)
{
	int ret;
	u32 reg;

	reg = readl(cvp->base + CVP_WRAPPER_CORE_POWER_STATUS);
	if ((reg & BIT(1)) == 0) // CORE_PWR_ON
		goto disable_hw_power;

	reg = readl(cvp->base + CVP_WRAPPER_CORE_CLOCK_CONFIG);
	if (reg)
		writel(0, cvp->base + CVP_WRAPPER_CORE_CLOCK_CONFIG);

	ret = read_poll_timeout(readl, reg, reg & BIT(22), 1000, 10000, false,
				cvp->base + CVP_SS_IDLE_STATUS);
	if (ret)
		dev_warn(cvp->miscdev.parent, "CVP core fail to go idle timeout (%x)", reg);

	reg = readl(cvp->base + CVP_AON_WRAPPER_CVP_NOC_LPI_CONTROL);
	writel(reg | BIT(0), cvp->base + CVP_AON_WRAPPER_CVP_NOC_LPI_CONTROL);

	ret = read_poll_timeout(readl, reg, reg & BIT(0), 100, 2000, false,
				cvp->base + CVP_AON_WRAPPER_CVP_NOC_LPI_STATUS);
	if (ret) {
		dev_warn(cvp->miscdev.parent, "CVP AON_WRAPPER_CVP_NOC_LPI_STATUS timeout (%x)", reg);
		goto disable_hw_power;
	}

	reg = readl(cvp->base + CVP_AON_WRAPPER_CVP_NOC_LPI_CONTROL);
	writel(reg & ~BIT(0), cvp->base + CVP_AON_WRAPPER_CVP_NOC_LPI_CONTROL);

	/* Reset both sides of 2 ahb2ahb_bridges (TZ and non-TZ) */
	writel(0x3, cvp->base + CVP_AHB_BRIDGE_SYNC_RESET);
	writel(0x2, cvp->base + CVP_AHB_BRIDGE_SYNC_RESET);
	writel(0x0, cvp->base + CVP_AHB_BRIDGE_SYNC_RESET);

disable_hw_power:
	pm_runtime_put_sync(cvp->pds[PM_DOMAIN_HW]);

	clk_disable(cvp->clks[CLK_HW]);

	dev_info(cvp->miscdev.parent, "%s: EVA core powered off\n", __func__);

	return 0;
}

static int cvp_set_hw_state_reset(struct cvp *cvp)
{
	writel(1, cvp->base + CVP_WRAPPER_TZ_XTSS_SW_RESET);

	return 0;
}

static int cvp_prepare_pc(struct cvp *cvp)
{
	// Make sure the following is set so we know it's safe to reset cores and remove dma mappings
	//	- IDLE status: BIT(30) in CPU_CS_SCIACMDARG0
	//	- CPU Wait For Interrupt: BIT(0) in CVP_WRAPPER_CPU_STATUS
	// (this state may not be reached if the hardware enters an error state, for example after an invalid iova access)
	int ret;
	u32 cpu_status, arg0;

	ret = poll_timeout_us((cpu_status = readl(cvp->base + CVP_WRAPPER_CPU_STATUS),
			       arg0 = readl(cvp->base + CVP_CPU_CS_SCIACMDARG0)),
			      (cpu_status & 0x1) && (arg0 & BIT(30)), 1000, 20000, false);
	if (ret) {
		dev_warn(cvp->miscdev.parent, "CVP failed to enter idle state during shutdown. (CPU_STATUS=0x%x SCIACMDARG0=0x%x)",
			 cpu_status, arg0);
	}

	return 0;
}

static int cvp_shutdown(struct cvp *cvp)
{
	int ret;

	if (atomic_read(&cvp->booted) == 0)
		return -EBUSY;

	ret = cvp_prepare_pc(cvp);
	if(ret)
		return ret;

	ret = cvp_set_hw_state_reset(cvp);
	if (ret)
		return ret;

	// disable CPU watchdog while off
	writel(0, cvp->base + CVP_CPU_WD_WDOGCONTROL);

	ret = cvp_power_off_hardware(cvp);
	if (ret)
		return ret;

	ret = cvp_power_off_controller(cvp);
	if (ret)
		return ret;

	dev_info(cvp->miscdev.parent, "%s: cvp (eva) powered off\n", __func__);

	dev_pm_genpd_set_performance_state(cvp->pds[PM_DOMAIN_MXC], 0);
	dev_pm_genpd_set_performance_state(cvp->pds[PM_DOMAIN_MMCX], 0);

	pm_runtime_put(cvp->pds[PM_DOMAIN_MXC]);
	pm_runtime_put(cvp->pds[PM_DOMAIN_MMCX]);

	atomic_set(&cvp->booted, 0);

	return 0;
}

static int cvp_open(struct inode *inode, struct file *file)
{
	struct cvp *cvp = file->private_data;

	if (atomic_xchg(&cvp->open, 1)) /* only open one at a time */
		return -EPERM;

	return 0;
}

static void cvp_unmap(struct cvp *cvp, struct dma_map *map);

static int cvp_release(struct inode *inode, struct file *file)
{
	struct cvp *cvp = file->private_data;
	struct dma_map *map, *n;

	cvp_shutdown(cvp);

	/* no lock - ioctl() should not be ongoing when release() is called */
	list_for_each_entry_safe(map, n, &cvp->maps, node)
		cvp_unmap(cvp, map);
	INIT_LIST_HEAD(&cvp->maps);

	atomic_set(&cvp->open, 0);
	return 0;
}

static __poll_t cvp_poll(struct file *file, struct poll_table_struct *wait)
{
	struct cvp *cvp = file->private_data;

	poll_wait(file, &cvp->waitqueue, wait);

	if (atomic_read(&cvp->pending))
		return POLLIN | POLLRDNORM;
	return 0;
}

static long cvp_map_dmabuf(struct cvp *cvp, int fd, u32 iova)
{
	struct dma_map *map;
	int ret;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	map->buf = dma_buf_get(fd);
	if (IS_ERR(map->buf)) {
		ret = PTR_ERR(map->buf);
		goto get_err;
	}

	map->attach = dma_buf_attach(map->buf, cvp->miscdev.parent);
	if (IS_ERR(map->attach)) {
		ret = PTR_ERR(map->attach);
		goto attach_err;
	}

	map->table = dma_buf_map_attachment_unlocked(map->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(map->table)) {
		ret = PTR_ERR(map->table);
		goto map_err;
	}

	// (CVP firmware needs IOMMU_PRIV, use 0 address to detect?)
	ret = iommu_map_sgtable(cvp->iommu, iova, map->table, IOMMU_READ | IOMMU_WRITE | (iova ? 0 : IOMMU_PRIV));
	WARN_ON(!ret);
	if (ret < 0)
		goto iommu_err;

	map->iova = iova;
	map->size = ret;

	spin_lock(&cvp->lock);
	list_add_tail(&map->node, &cvp->maps);
	spin_unlock(&cvp->lock);

	return ret;

iommu_err:
	dma_buf_unmap_attachment_unlocked(map->attach, map->table, DMA_BIDIRECTIONAL);
map_err:
	dma_buf_detach(map->buf, map->attach);
attach_err:
	dma_buf_put(map->buf);
get_err:
	kfree(map);

	return ret;
}

static int cvp_map_direct(struct cvp *cvp, u32 iova, u32 phys, u32 size)
{
	struct dma_map *map;
	int ret;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	ret = iommu_map(cvp->iommu, iova, phys, size, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		kfree(map);
		return ret;
	}

	map->buf = NULL;
	map->attach = NULL;
	map->table = NULL;
	map->iova = iova;
	map->size = size;

	spin_lock(&cvp->lock);
	list_add_tail(&map->node, &cvp->maps);
	spin_unlock(&cvp->lock);
	return 0;
}

static void cvp_unmap(struct cvp *cvp, struct dma_map *map)
{
	iommu_unmap(cvp->iommu, map->iova, map->size);
	if (map->buf) {
		dma_buf_unmap_attachment_unlocked(map->attach, map->table, DMA_BIDIRECTIONAL);
		dma_buf_detach(map->buf, map->attach);
		dma_buf_put(map->buf);
	}
	kfree(map);
}

static long cvp_unmap_by_va(struct cvp *cvp, u32 va)
{
	struct dma_map *map, *n;

	spin_lock(&cvp->lock);
	list_for_each_entry_safe(map, n, &cvp->maps, node) {
		if (map->iova != va)
			continue;

		list_del(&map->node);
		spin_unlock(&cvp->lock);
		cvp_unmap(cvp, map);
		return 0;
	}
	spin_unlock(&cvp->lock);
	return -EINVAL;
}

static long cvp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct cvp *cvp = file->private_data;
	char __user *argp = (char __user *)arg;

	switch (cmd) {
		case CVP_IOCTL_IRQ_STATUS:
			return atomic_xchg(&cvp->pending, 0);
		case CVP_IOCTL_IRQ_SEND:
			writel(1, cvp->base + CVP_CPU_CS_H2XSOFTINT);
			return 0;
		case CVP_IOCTL_BOOT:
			if (atomic_xchg(&cvp->booted, 1))
				return -EBUSY;

			return cvp_power_on_init(cvp);
		case CVP_IOCTL_SHUTDOWN:
			return cvp_shutdown(cvp);
		case CVP_IOCTL_MAP_DMABUF:
			return cvp_map_dmabuf(cvp, arg & 0xffffffffu, arg >> 32);
		case CVP_IOCTL_MAP_DIRECT: {
			struct map_direct map;
			if (copy_from_user(&map, argp, sizeof(map)))
				return -EFAULT;
			return cvp_map_direct(cvp, map.va, map.pa, map.size);
		}
		case CVP_IOCTL_UNMAP:
			return cvp_unmap_by_va(cvp, arg);
		default:
			return -ENOTTY;
	}
}

static int cvp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct cvp *cvp = file->private_data;
	unsigned long user_count = vma_pages(vma);
	unsigned long count = cvp->mem_size >> PAGE_SHIFT;
	unsigned long pfn = PHYS_PFN(cvp->mem_phys);
	unsigned long off = vma->vm_pgoff;

	/* note: only used with CVP region which is a device region */
	vma->vm_page_prot = pgprot_device(vma->vm_page_prot);

	if (off >= count || user_count > count - off)
		return -ENXIO;

	return remap_pfn_range(vma, vma->vm_start, pfn + off,
			user_count << PAGE_SHIFT, vma->vm_page_prot);
}

static const struct file_operations cvp_fops = {
	.open = cvp_open,
	.release = cvp_release,
	.poll = cvp_poll,
	.unlocked_ioctl = cvp_ioctl,
	.compat_ioctl = cvp_ioctl,
	.mmap = cvp_mmap,
};

static irqreturn_t cvp_irq(int irq, void *data)
{
	struct cvp *cvp = data;
	u32 status;

	status = readl(cvp->base + CVP_WRAPPER_INTR_STATUS);
	writel(1, cvp->base + CVP_CPU_CS_A2HSOFTINTCLR);

	atomic_or(status, &cvp->pending);
	wake_up_interruptible(&cvp->waitqueue);
	return IRQ_HANDLED;
}

static int cvp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct cvp *cvp;
	int irq, ret;

	cvp = devm_kzalloc(dev, sizeof(*cvp), GFP_KERNEL);
	if (!cvp)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	cvp->mem_phys = res->start;
	cvp->mem_size = resource_size(res);

	cvp->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(cvp->base))
		return PTR_ERR(cvp->base);

	INIT_LIST_HEAD(&cvp->maps);
	init_waitqueue_head(&cvp->waitqueue);

	spin_lock_init(&cvp->lock);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;

	ret = devm_request_irq(dev, irq, cvp_irq, IRQF_TRIGGER_HIGH, "cvp", cvp);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, cvp);

	cvp->iommu = iommu_paging_domain_alloc(dev);
	if (IS_ERR(cvp->iommu))
		return PTR_ERR(cvp->iommu);

	ret = iommu_attach_device(cvp->iommu, dev);
	if (ret)
		return ret;

	/* Get power domains and corresponding power levels */
	cvp->pds[PM_DOMAIN_MXC] = dev_pm_domain_attach_by_name(dev, "mxc");
	if (IS_ERR_OR_NULL(cvp->pds[PM_DOMAIN_MXC]))
		return PTR_ERR(cvp->pds[PM_DOMAIN_MXC]) ? : -ENODATA;
	cvp->pds[PM_DOMAIN_MMCX] = dev_pm_domain_attach_by_name(dev, "mmcx");
	if (IS_ERR_OR_NULL(cvp->pds[PM_DOMAIN_MMCX]))
		return PTR_ERR(cvp->pds[PM_DOMAIN_MMCX]) ? : -ENODATA;
	cvp->pds[PM_DOMAIN_CTRL] = dev_pm_domain_attach_by_name(dev, "ctrl");
	if (IS_ERR_OR_NULL(cvp->pds[PM_DOMAIN_CTRL]))
		return PTR_ERR(cvp->pds[PM_DOMAIN_CTRL]) ? : -ENODATA;
	cvp->pds[PM_DOMAIN_HW] = dev_pm_domain_attach_by_name(dev, "hw");
	if (IS_ERR_OR_NULL(cvp->pds[PM_DOMAIN_HW]))
		return PTR_ERR(cvp->pds[PM_DOMAIN_HW]) ? : -ENODATA;

	cvp->perf[PM_DOMAIN_MXC] = of_get_required_opp_performance_state(dev->of_node, 0);
	if (cvp->perf[PM_DOMAIN_MXC] < 0)
		return cvp->perf[PM_DOMAIN_MXC];
	cvp->perf[PM_DOMAIN_MMCX] = of_get_required_opp_performance_state(dev->of_node, 1);
	if (cvp->perf[PM_DOMAIN_MMCX] < 0)
		return cvp->perf[PM_DOMAIN_MMCX];
	cvp->perf[PM_DOMAIN_CTRL] = of_get_required_opp_performance_state(dev->of_node, 2);
	if (cvp->perf[PM_DOMAIN_CTRL] < 0)
		return cvp->perf[PM_DOMAIN_CTRL];
	cvp->perf[PM_DOMAIN_HW] = of_get_required_opp_performance_state(dev->of_node, 3);
	if (cvp->perf[PM_DOMAIN_HW] < 0)
		return cvp->perf[PM_DOMAIN_HW];

	cvp->resets[RESET_BUS] = devm_reset_control_get(dev, "bus");
	if (IS_ERR(cvp->resets[RESET_BUS]))
		return PTR_ERR(cvp->resets[RESET_BUS]);
#if 0
	// RESET_XO is shared with iris, but this causes the iris fw to shutdown
	// keep this commented until we figure out how to use this
	cvp->resets[RESET_XO] = devm_reset_control_get_exclusive_released(dev, "xo");
	if (IS_ERR(cvp->resets[RESET_XO]))
		return PTR_ERR(cvp->resets[RESET_XO]);
#endif
	cvp->resets[RESET_CORE] = devm_reset_control_get(dev, "core");
	if (IS_ERR(cvp->resets[RESET_CORE]))
		return PTR_ERR(cvp->resets[RESET_CORE]);

	cvp->clks[CLK_IFACE] = devm_clk_get_prepared(dev, "iface");
	if (IS_ERR(cvp->clks[CLK_IFACE]))
		return PTR_ERR(cvp->clks[CLK_IFACE]);
	cvp->clks[CLK_CORE] = devm_clk_get_prepared(dev, "core");
	if (IS_ERR(cvp->clks[CLK_CORE]))
		return PTR_ERR(cvp->clks[CLK_CORE]);
	cvp->clks[CLK_HW] = devm_clk_get_prepared(dev, "hw");
	if (IS_ERR(cvp->clks[CLK_HW]))
		return PTR_ERR(cvp->clks[CLK_HW]);

	// Uses VIDEO_CC_MVS1C_CLK which looks like it's video_cc_mvs1c_div2_div_clk_src
	// which scales the input rate by 2 to match what is in the freq_tbl??
	// So try to map those allowed rates from the ftbl_video_cc_mvs1_clk_src_sm8650 ->
	// 420000000 -> 840000000
	// 555000000 -> 1110000000
	// 675000000 -> 1350000000
	// 750000000 -> 1500000000
	// 825000000 -> 1650000000
	u32 rate_to_set = 825000000;
	clk_set_rate(cvp->clks[CLK_CORE], rate_to_set);
	u32 applied_rate = clk_get_rate(cvp->clks[CLK_CORE]);
	printk("cvp clock rate: %d (called with set_rate: %d)", applied_rate, rate_to_set);

	cvp->miscdev.minor = MISC_DYNAMIC_MINOR;
	cvp->miscdev.name = dev->of_node->name;
	cvp->miscdev.fops = &cvp_fops;
	cvp->miscdev.parent = dev;

	return misc_register(&cvp->miscdev);
}

static void cvp_remove(struct platform_device *pdev)
{
	struct cvp *cvp = platform_get_drvdata(pdev);

	misc_deregister(&cvp->miscdev);
	return;
}

static const struct of_device_id cvp_match_table[] = {
	{ .compatible = "qcom,sm8650-xtensa" },
	{ }
};
MODULE_DEVICE_TABLE(of, cvp_match_table);

static struct platform_driver cvp_driver = {
	.probe = cvp_probe,
	.remove = cvp_remove,
	.driver = {
		.name = "cvp",
		.of_match_table = cvp_match_table,
	},
};
module_platform_driver(cvp_driver);
