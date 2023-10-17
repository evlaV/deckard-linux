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
#include <linux/interconnect.h>
#include <dt-bindings/mailbox/qcom-ipcc.h>
#include <linux/iommu.h>
#include <linux/mailbox_client.h>

#define QDSP6SS_VERSION 0x00

#define QDSP6SS_RST_EVB 0x10

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

#define QDSP6SS_QDSP6V67SS_PLL 0x40000

struct dsp {
	struct miscdevice miscdev;
	wait_queue_head_t waitqueue;
	atomic_t pending;
	spinlock_t lock;
	struct list_head maps;
	atomic_t open, booted;
	struct iommu_domain *iommu;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan[4];
	void __iomem *base, *evb_sel;
	struct clk_bulk_data *clks;
	unsigned num_clks;
	bool is_xtensa;
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

#define DSP_IOCTL_IRQ_STATUS 	_IO('C', 0) /* returns mask of pending ipcc signals and clears */
/* note: dsp fd will have POLLIN event if there are any pending ipcc signals */
#define DSP_IOCTL_IRQ_SEND  	_IO('C', 1) /* send an ipcc signal, argument is integer in 0-3 range */
#define DSP_IOCTL_BOOT 		_IO('C', 2) /* boot the DSP. argument is address to boot at */
/* note: for xtensa boot address is ignored (always 0) */
#define DSP_IOCTL_SHUTDOWN 	_IO('C', 3) /* shutdown DSP. no argument */
#define DSP_IOCTL_MAP_DMABUF 	_IO('C', 4) /* 64-bit argument: low 32-bits dmabuf fd, high 32-bits VA */
#define DSP_IOCTL_MAP_DIRECT 	_IOW('C', 5, struct map_direct)
#define DSP_IOCTL_UNMAP 	_IO('C', 6) /* 32 bit va argument */

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
   u16 alpha = (freq % 19200000) * 32 / 9375;

   write32(pll_reg + PLL_MODE, 0x00);

   write32(pll_reg + PLL_L_VAL, 0x44 << 16 | l);
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

static int dsp_boot(struct dsp *dsp, u32 boot_addr)
{
	if (dsp->evb_sel)
		writel(0, dsp->evb_sel);

#if 0
	// set src to QDSP6SSPLL
	writel(0x200, dsp->base + QDSP6SS_CORE_CFG_RCGR);
	writel(1, dsp->base + QDSP6SS_CORE_CMD_RCGR);

	writel(1, dsp->base + QDSP6SS_XO_CBCR);
	writel(1, dsp->base + QDSP6SS_SLEEP_CBCR);
	writel(1, dsp->base + QDSP6SS_CORE_CBCR);

	usleep_range(1000, 2000); // XXX
#endif

	writel(boot_addr >> 4, dsp->base + QDSP6SS_RST_EVB);
	writel(1, dsp->base + QDSP6SS_BOOT_CORE_START);
	usleep_range(1000, 2000); // XXX
	writel(1, dsp->base + QDSP6SS_BOOT_CMD);

	usleep_range(1000, 2000);

	return readl(dsp->base + QDSP6SS_BOOT_STATUS);
}

static int dsp_shutdown(struct dsp *dsp)
{
	// XXX possible race condition if dsp_shutdown called twice simultaneously..
	if (atomic_read(&dsp->booted) == 0)
		return -EBUSY;

	if (dsp->is_xtensa) {
		writel(1, dsp->base + 0xc1000); // VCODEC_VPU_WRAPPER_TZ_XTSS_SW_RESET

		// disable CPU watchdog while off
		writel(0, dsp->base + 0x90008); // CPU_WD_WDOGCONTROL
	} else {
		writel(~0u, dsp->base +  QDSP6SS_RESET_CFG);
		writel(1, dsp->base +  QDSP6SS_RESET_CMD);
		usleep_range(10000, 20000); // XXX
		writel(0, dsp->base +  QDSP6SS_RESET_CFG);
		//writel(1, dsp->base +  QDSP6SS_RESET_CMD);

#if 0
		writel(0, dsp->base +  QDSP6SS_XO_CBCR);
		writel(0, dsp->base +  QDSP6SS_SLEEP_CBCR);
		writel(0, dsp->base +  QDSP6SS_CORE_CBCR);
#endif
	}

	atomic_set(&dsp->booted, 0);
	return 0;
}

static int dsp_open(struct inode *inode, struct file *file)
{
	struct dsp *dsp = file->private_data;

	if (atomic_xchg(&dsp->open, 1)) /* only open one at a time */
		return -EPERM;
	return 0;
}

static void dsp_unmap(struct dsp *dsp, struct dma_map *map);

static int dsp_release(struct inode *inode, struct file *file)
{
	struct dsp *dsp = file->private_data;
	struct dma_map *map, *n;

	dsp_shutdown(dsp);

	/* no lock - ioctl() should not be ongoing when release() is called */
	list_for_each_entry_safe(map, n, &dsp->maps, node)
		dsp_unmap(dsp, map);
	INIT_LIST_HEAD(&dsp->maps);

	atomic_set(&dsp->open, 0);
	return 0;
}

static __poll_t dsp_poll(struct file *file, struct poll_table_struct *wait)
{
	struct dsp *dsp = file->private_data;

	poll_wait(file, &dsp->waitqueue, wait);

	if (atomic_read(&dsp->pending))
		return POLLIN | POLLRDNORM;
	return 0;
}

static long dsp_map_dmabuf(struct dsp *dsp, int fd, u32 iova)
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

	map->attach = dma_buf_attach(map->buf, dsp->miscdev.parent);
	if (IS_ERR(map->attach)) {
		ret = PTR_ERR(map->attach);
		goto attach_err;
	}

	map->table = dma_buf_map_attachment(map->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(map->table)) {
		ret = PTR_ERR(map->table);
		goto map_err;
	}

	// (CVP firmware needs IOMMU_PRIV, use 0 address to detect?)
	ret = iommu_map_sgtable(dsp->iommu, iova, map->table, IOMMU_READ | IOMMU_WRITE | (iova ? 0 : IOMMU_PRIV));
	WARN_ON(!ret);
	if (ret < 0)
		goto iommu_err;

	map->iova = iova;
	map->size = ret;

	spin_lock(&dsp->lock);
	list_add_tail(&map->node, &dsp->maps);
	spin_unlock(&dsp->lock);

	return ret;

iommu_err:
	dma_buf_unmap_attachment(map->attach, map->table, DMA_BIDIRECTIONAL);
map_err:
	dma_buf_detach(map->buf, map->attach);
attach_err:
	dma_buf_put(map->buf);
get_err:
	kfree(map);

	return ret;
}

static int dsp_map_direct(struct dsp *dsp, u32 iova, u32 phys, u32 size)
{
	struct dma_map *map;
	int ret;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	ret = iommu_map(dsp->iommu, iova, phys, size, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		kfree(map);
		return ret;
	}

	map->buf = NULL;
	map->attach = NULL;
	map->table = NULL;
	map->iova = iova;
	map->size = size;

	spin_lock(&dsp->lock);
	list_add_tail(&map->node, &dsp->maps);
	spin_unlock(&dsp->lock);
	return 0;
}

static void dsp_unmap(struct dsp *dsp, struct dma_map *map)
{
	iommu_unmap(dsp->iommu, map->iova, map->size);
	if (map->buf) {
		dma_buf_unmap_attachment(map->attach, map->table, DMA_BIDIRECTIONAL);
		dma_buf_detach(map->buf, map->attach);
		dma_buf_put(map->buf);
	}
	kfree(map);
}

static long dsp_unmap_by_va(struct dsp *dsp, u32 va)
{
	struct dma_map *map, *n;

	spin_lock(&dsp->lock);
	list_for_each_entry_safe(map, n, &dsp->maps, node) {
		if (map->iova != va)
			continue;

		list_del(&map->node);
		spin_unlock(&dsp->lock);
		dsp_unmap(dsp, map);
		return 0;
	}
	spin_unlock(&dsp->lock);
	return -EINVAL;
}

#define CPU_CS_X2HSOFTINTCLR	0xa001c
#define CPU_CS_H2XSOFTINT	0xa0150
#define WRAPPER_INTR_STATUS	0xb000c

static long dsp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dsp *dsp = file->private_data;
	char __user *argp = (char __user *)arg;

	switch (cmd) {
	case DSP_IOCTL_IRQ_STATUS:
		return atomic_xchg(&dsp->pending, 0);
	case DSP_IOCTL_IRQ_SEND:
		if (dsp->is_xtensa) {
			writel_relaxed(1, dsp->base + CPU_CS_H2XSOFTINT);
			return 0;
		}

		if (arg >= 4)
			return -EINVAL;
		mbox_send_message(dsp->mbox_chan[arg], NULL);
		return 0;
	case DSP_IOCTL_BOOT:
		if (atomic_xchg(&dsp->booted, 1))
			return -EBUSY;

		if (dsp->is_xtensa) {
			/* Bring out of reset */
			writel(0, dsp->base + 0xc1000);
			return 0;
		}
		return dsp_boot(dsp, arg);
	case DSP_IOCTL_SHUTDOWN:
		return dsp_shutdown(dsp);
	case DSP_IOCTL_MAP_DMABUF:
		return dsp_map_dmabuf(dsp, arg & 0xffffffffu, arg >> 32);
	case DSP_IOCTL_MAP_DIRECT: {
		struct map_direct map;
		if (copy_from_user(&map, argp, sizeof(map)))
			return -EFAULT;
		return dsp_map_direct(dsp, map.va, map.pa, map.size);
	}
	case DSP_IOCTL_UNMAP:
		return dsp_unmap_by_va(dsp, arg);
	default:
		return -ENOTTY;
	}
}

static const struct file_operations dsp_fops = {
	.open = dsp_open,
	.release = dsp_release,
	.poll = dsp_poll,
	.unlocked_ioctl = dsp_ioctl,
	.compat_ioctl = dsp_ioctl,
};

static irqreturn_t ipcc_irq(int irq, void *data)
{
	struct dsp *dsp = (void*) (((size_t) data) & ~3ul);
	atomic_or(BIT( (size_t) data & 3), &dsp->pending);
	wake_up_interruptible(&dsp->waitqueue);
	return IRQ_HANDLED;
}

static irqreturn_t xtensa_irq(int irq, void *data)
{
	struct dsp *dsp = data;
	u32 status;

	status = readl_relaxed(dsp->base + WRAPPER_INTR_STATUS);
	writel_relaxed(1, dsp->base + CPU_CS_X2HSOFTINTCLR);

	atomic_or(status, &dsp->pending);
	wake_up_interruptible(&dsp->waitqueue);
	return IRQ_HANDLED;
}

static int dsp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dsp *dsp;
	int i, irq, ret;
	//u32 max_rate;
	static int x;

	// missing: driving voltage rails. defer probe a bit for always-on to activate
	if (x++ < 3)
		return -EPROBE_DEFER;

	dsp = devm_kzalloc(dev, sizeof(*dsp), GFP_KERNEL);
	if (!dsp)
		return -ENOMEM;

	dsp->is_xtensa = of_device_is_compatible(dev->of_node, "qcom,sm8650-xtensa");

	/* 36-bit dma address mask */
	if (!dsp->is_xtensa) {
		ret = dma_set_mask_and_coherent(dev, 0xffffffffful);
		if (ret)
			return ret;
	}

	dsp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsp->base))
		return PTR_ERR(dsp->base);

	if (platform_get_resource_byname(pdev, IORESOURCE_MEM, "evb_sel")) {
		dsp->evb_sel = devm_platform_ioremap_resource_byname(pdev, "evb_sel");
		if (IS_ERR(dsp->evb_sel))
			return PTR_ERR(dsp->evb_sel);
	}

	INIT_LIST_HEAD(&dsp->maps);
	init_waitqueue_head(&dsp->waitqueue);

	ret = devm_clk_bulk_get_all(dev, &dsp->clks);
	if (ret < 0)
		return ret;
	dsp->num_clks = ret;

	if (dsp->is_xtensa) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return -ENODEV;

		ret = devm_request_irq(dev, irq, xtensa_irq, IRQF_TRIGGER_HIGH, "cvp", dsp);
		if (ret)
			return ret;
	} else {
		for (i = 0; i < 4; i++) {
			irq = platform_get_irq(pdev, i);
			if (irq < 0)
				return -ENODEV;

			ret = devm_request_irq(dev, irq, ipcc_irq, IRQF_TRIGGER_HIGH, "ipcc", (void*) ((size_t) dsp | i));
			if (ret)
				return ret;
		}

		dsp->mbox_client.dev = dev;
		dsp->mbox_client.knows_txdone = true;
		for (i = 0; i < 4; i++) {
			dsp->mbox_chan[i] = mbox_request_channel(&dsp->mbox_client, i);
			WARN_ON(IS_ERR(dsp->mbox_chan[i]));
		}
	}

	platform_set_drvdata(pdev, dsp);

	dsp->iommu = iommu_domain_alloc(&platform_bus_type);
	ret = iommu_attach_device(dsp->iommu, dev);
	WARN_ON(ret);

	dsp->miscdev.minor = MISC_DYNAMIC_MINOR;
	dsp->miscdev.name = dev->of_node->name;
	dsp->miscdev.fops = &dsp_fops;
	dsp->miscdev.parent = dev;
	ret = misc_register(&dsp->miscdev);
	if (ret)
		return ret;

	if (dsp->is_xtensa)
		clk_set_rate(dsp->clks[1].clk, 355200000); //max_rate);

	// XXX
	ret = clk_bulk_prepare_enable(dsp->num_clks, dsp->clks);
	WARN_ON(ret);

	// XXX
	if (!strcmp(dev->of_node->name, "cdsp"))
	{
		struct icc_path *dma_path;
		void __iomem *turingcc = devm_ioremap(dev, 0x32008000, 0x20000);
		WARN_ON(IS_ERR_OR_NULL(turingcc));

		// 0x32008404 ?
		writel_relaxed(1, turingcc + 0x404); // TURING_Q6SS_Q6_AXIM_CBCR
		//writel_relaxed(1, turingcc + 0x220); // TURING_CENG_CBCR
		writel_relaxed(1, turingcc + 0x22c); // TURING_NSPNOC_CBCR
		writel_relaxed(1, turingcc + 0x414); // TURING_Q6SS_AHBS_AON_CBCR


		dma_path = devm_of_icc_get(dev, "mem");
		WARN_ON(IS_ERR(dma_path));

		icc_set_bw(dma_path, 0, kBps_to_icc(1000*1000*50));

		// TURING_QDSP6SS_PLL_PLL_CM_PLL_LUCID_OLE
		clk_pll_configure_lucid_ole(dsp->base + 0x40000, 460800000);
	}

	return 0;
}

static int dsp_remove(struct platform_device *pdev)
{
	struct dsp *dsp = platform_get_drvdata(pdev);

	misc_deregister(&dsp->miscdev);
	return 0;
}

static const struct of_device_id dsp_match_table[] = {
	{ .compatible = "qcom,sm8650-hexagon" },
	{ .compatible = "qcom,sm8650-xtensa" },
	{ }
};
MODULE_DEVICE_TABLE(of, dsp_match_table);

static struct platform_driver dsp_driver = {
	.probe = dsp_probe,
	.remove = dsp_remove,
	.driver = {
		.name = "dsp-sm8650",
		.of_match_table = dsp_match_table,
	},
};
module_platform_driver(dsp_driver);
