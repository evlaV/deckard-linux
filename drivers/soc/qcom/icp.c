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
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/hwspinlock.h>

// Custom ioctls
#define ICP_IOCTL_BOOT _IOW('C', 0, struct icp_boot_config)
#define ICP_IOCTL_SHUTDOWN _IO('C', 1)
#define ICP_IOCTL_MAP_DMABUF _IOW('C', 2, struct icp_map_dmabuf_config)
#define ICP_IOCTL_MAP_DIRECT _IOW('C', 3, struct icp_map_direct_config)
#define ICP_IOCTL_UNMAP_DMABUF _IO('C', 4)
#define ICP_IOCTL_SEND_IRQ _IO('C', 5) // used for ICP wakeup after sending data over HFI
#define ICP_IOCTL_SEND_IPCC _IO('C', 6) // used to signal the ICP after sychronization data changes
#define ICP_IOCTL_SYNX_TRYLOCK _IO('C', 7)
#define ICP_IOCTL_SYNX_UNLOCK _IO('C', 8)

struct icp_boot_config
{
	// ICP memory regions info
	__u32 qtbl_iova, qtbl_len;
	__u32 sfr_iova, sfr_len;
	__u32 sec_heap_iova, sec_heap_len;
	__u32 shared_mem_iova, shared_mem_len;
	__u32 qdss_mem_iova, qdss_mem_len;
	__u32 io_mem_iova, io_mem_len;
	__u32 fwuncached_mem_iova, fwuncached_mem_len;
	__u32 device_mem_iova, device_mem_len;
};

struct icp_map_dmabuf_config
{
	__s32 fd;
	__u64 iova; // device address up to 36bits
	// Mapping properties
	__u32 priv; // SMMU mapping done with IOMMU_PRIVATE when set
	__u32 coherent; // SMMU mapping done with IOMMU_CACHE when set to enable cache coherency
};

struct icp_map_direct_config
{
	__u32 iova;
	__u32 phys;
	__u32 size;
};

#define ICP_SYS_RESET		0x0
#define 	ICP_SYS_RESET_FUNC_RESET BIT(4)
#define 	ICP_SYS_RESET_DBG_RESET BIT(3)
#define 	ICP_SYS_RESET_CPU_RESET BIT(2)
#define ICP_SYS_CTRL 		0x4
#define 	ICP_SYS_CTRL_CPU_EN BIT(9)
#define ICP_SYS_STATUS 		0xC
#define 	ICP_SYS_STATUS_STANDBY	BIT(7)
#define ICP_SYS_RESET_VEC 	0x80

#define ICP_GP_REG(i) 	(0x1020 + (i*4)) // [0, 23]
#define ICP_OB_MASK 	0x1200
#define ICP_OB_CLEAR 	0x1204
#define ICP_OB_SET 		0x1208
#define ICP_OB_STATUS 	0x120C
#define ICP_OB_CMD 		0x1210
#define ICP_HOST2ICPINT 0x1300
#define ICP_PFAULT_INFO 0x1400

// For ICP_OB_MASK/CLEAR/SET/STATUS
#define ICP_OB_WDT_BITE_WS1 	BIT(6)
#define ICP_OB_WDT_BARK_WS1 	BIT(5)
#define ICP_OB_WDT_BITE_WS0 	BIT(4)
#define ICP_OB_WDT_BARK_WS0 	BIT(3)
#define ICP_OB_ICP2HOSTINT 		BIT(2)
#define ICP_OB_PFATALERROR 		BIT(1)
#define ICP_OB_DBLEXCEPTION 	BIT(0)
// For ICP_OB_CMD
#define ICP_OB_CMD_SET 		BIT(0)
#define ICP_OB_CMD_CLEAR 	BIT(1)

#define ICP_DOM0_CFG	0x6000
#define ICP_DOM1_CFG	0x6020

#define ICP_WD0_CTRL 0x4008

struct icp_data {
	struct miscdevice miscdev;
	struct device *dev;
	// HWIO
	void __iomem *icp_base;
	void __iomem *ipe_base;
	void __iomem *bps_base;
	// IOMMU
	struct iommu_domain *iommu;
	// Interrupts
	int icp_irq;
	int ipcc_irq;
	// Power domain
	int genpd_num;
	struct device **genpd;
	struct device_link **genpd_link;
	// Clocks
	int clk_cnt;
	struct clk_bulk_data *clks;
	// Interconnect
	struct icc_path *sf_dma_path; // memory accesses for IPE (and other non real time cam modules)
	struct icc_path *icp_dma_path; // memory accesses for ICP
	// Mailbox
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan;
	// Hardware mutex
	struct hwspinlock* hwlock_synx;

	atomic_t open, booted, data_available;
	struct list_head maps; // stores the iommu mappings
	wait_queue_head_t waitqueue; // to wake up polling processes on ICP2HOST or IPCC signal interrupts
};

struct dma_map {
	struct list_head node;
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *table;
	u64 iova; // need u64 for 36-bit addressing support
	u32 size;
};

static long icp_map_dmabuf(struct icp_data *icp, struct icp_map_dmabuf_config __user *cfg_uptr)
{
	struct dma_map *map;
	int ret;

	struct icp_map_dmabuf_config config;
	if(copy_from_user(&config, cfg_uptr, sizeof(struct icp_map_dmabuf_config))) {
		dev_err(icp->dev, "Failed to retrieve the ICP DMABUF mapping configuration from ioctl argument\n");
		return -EINVAL;
	}

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	map->buf = dma_buf_get(config.fd);
	if (IS_ERR(map->buf)) {
		ret = PTR_ERR(map->buf);
		goto get_err;
	}

	map->attach = dma_buf_attach(map->buf, icp->miscdev.parent);
	if (IS_ERR(map->attach)) {
		ret = PTR_ERR(map->attach);
		goto attach_err;
	}

	map->table = dma_buf_map_attachment(map->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(map->table)) {
		ret = PTR_ERR(map->table);
		goto map_err;
	}

	// ICP firmware needs IOMMU_PRIV
	int map_protection_bits = IOMMU_READ | IOMMU_WRITE;
	if(config.priv) {
		map_protection_bits |= IOMMU_PRIV;
	}
	if(config.coherent) {
		map_protection_bits |= IOMMU_CACHE;
	}
	ret = iommu_map_sgtable(icp->iommu, config.iova, map->table, map_protection_bits);
	WARN_ON(!ret);
	if (ret < 0)
		goto iommu_err;

	map->iova = config.iova;
	map->size = ret;

	list_add_tail(&map->node, &icp->maps);

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

static long icp_map_direct(struct icp_data *icp, struct icp_map_direct_config __user *cfg_uptr)
{
	struct dma_map *map;
	int ret;

	struct icp_map_direct_config config;
	if(copy_from_user(&config, cfg_uptr, sizeof(struct icp_map_direct_config))) {
		dev_err(icp->dev, "Failed to retrieve the ICP DMABUF mapping configuration from ioctl argument\n");
		return -EINVAL;
	}

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	ret = iommu_map(icp->iommu, config.iova, config.phys, config.size, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		kfree(map);
		return ret;
	}

	map->buf = NULL;
	map->attach = NULL;
	map->table = NULL;
	map->iova = config.iova;
	map->size = config.size;
	list_add_tail(&map->node, &icp->maps);

	return 0;
}

static void icp_unmap(struct icp_data *icp, struct dma_map *map)
{
	iommu_unmap(icp->iommu, map->iova, map->size);
	if (map->buf) {
		dma_buf_unmap_attachment(map->attach, map->table, DMA_BIDIRECTIONAL);
		dma_buf_detach(map->buf, map->attach);
		dma_buf_put(map->buf);
	}
	kfree(map);
}

static long icp_unmap_dmabuf_by_va(struct icp_data *icp, u64 va)
{
	struct dma_map *map, *n;
	list_for_each_entry_safe(map, n, &icp->maps, node) {
		if (map->iova != va)
			continue;

		list_del(&map->node);
		icp_unmap(icp, map);
		return 0;
	}
	return -EINVAL;
}

static int reset_ipe_bps_cores(struct icp_data *icp)
{
	int retry_cnt = 0;
	int ipe_bps_reset_bits = 0;
	
#define IPE_BPS_RESET_MAX_NUM_RETRY 50

#define BPS_CDM_RESET_CMD 0x10
#define BPS_CDM_IRQ_STATUS 0x44
#define BPS_CDM_IRQ_CLEAR 0x34
#define BPS_CDM_IRQ_CLEAR_CMD 0x38 // single bit
#define BPS_TOP_RESET_CMD 0x1008
#define BPS_TOP_IRQ_STATUS 0x100c
#define BPS_TOP_IRQ_CLEAR 0x1014
#define BPS_TOP_IRQ_CMD 0x101c // clear on bit 0

#define IPE_CDM_RESET_CMD 0x10
#define IPE_CDM_IRQ_STATUS 0x44
#define IPE_CDM_IRQ_CLEAR 0x34
#define IPE_CDM_IRQ_CLEAR_CMD 0x38 // single bit
#define IPE_TOP_RESET_CMD 0x1008
#define IPE_TOP_IRQ_STATUS 0x100c
#define IPE_TOP_IRQ_CLEAR 0x1014
#define IPE_TOP_IRQ_CMD 0x101c // clear on bit 0

	// BPS CMD core reset and BPS reset
	writel_relaxed(0x7f, icp->bps_base + BPS_CDM_RESET_CMD); 	// all reset bits
	writel_relaxed(0x3f, icp->bps_base + BPS_TOP_RESET_CMD); 	// all reset bits
	// IPE CMD core reset and IPE reset
	writel_relaxed(0x7f, icp->ipe_base + IPE_CDM_RESET_CMD); 	// all reset bits
	writel_relaxed(0x3f, icp->ipe_base + IPE_TOP_RESET_CMD); 	// all reset bits

	retry_cnt = 0;
	ipe_bps_reset_bits = 0; // 
	while(retry_cnt < IPE_BPS_RESET_MAX_NUM_RETRY) {
		u32 status = 0;
		usleep_range(1000, 2000);

		status = readl_relaxed(icp->bps_base + BPS_CDM_IRQ_STATUS);
		if(status & 0x1) {
			ipe_bps_reset_bits |= 0x1;
			writel_relaxed(0x1, icp->bps_base + BPS_CDM_IRQ_CLEAR);
			writel_relaxed(0x1, icp->bps_base + BPS_CDM_IRQ_CLEAR_CMD);
		}

		status = readl_relaxed(icp->bps_base + BPS_TOP_IRQ_STATUS);
		if(status & 0x1) {
			ipe_bps_reset_bits |= 0x2;
			writel_relaxed(0x1, icp->bps_base + BPS_TOP_IRQ_CLEAR);
			writel_relaxed(0x1, icp->bps_base + BPS_TOP_IRQ_CMD);
		}

		status = readl_relaxed(icp->ipe_base + IPE_CDM_IRQ_STATUS);
		if(status & 0x1) {
			ipe_bps_reset_bits |= 0x4;
			writel_relaxed(0x1, icp->ipe_base + IPE_CDM_IRQ_CLEAR);
			writel_relaxed(0x1, icp->ipe_base + IPE_CDM_IRQ_CLEAR_CMD);
		}

		status = readl_relaxed(icp->ipe_base + IPE_TOP_IRQ_STATUS);
		if(status & 0x1) {
			ipe_bps_reset_bits |= 0x8;
			writel_relaxed(0x1, icp->ipe_base + IPE_TOP_IRQ_CLEAR);
			writel_relaxed(0x1, icp->ipe_base + IPE_TOP_IRQ_CMD);
		}

		// Stop if all cores have been resetted
		if(ipe_bps_reset_bits == 0xf)
			break;

		retry_cnt++;
	}
	if(retry_cnt == IPE_BPS_RESET_MAX_NUM_RETRY) {
		dev_err(icp->dev, "Failed to reset the IPE/BPS cores (reset bitfield=0x%x)\n", ipe_bps_reset_bits);
		return -ENODEV;
	}

	return 0;
}

static int icp_boot(struct icp_data *icp, struct icp_boot_config __user *cfg_uptr)
{
	struct icp_boot_config config;
	if(copy_from_user(&config, cfg_uptr, sizeof(struct icp_boot_config))) {
		dev_err(icp->dev, "Failed to retrieve boot configuration from ioctl argument\n");
		return -EINVAL;
	}

	// Return an error if the ICP is already booted
	if(atomic_xchg(&icp->booted, 1))
		return -EBUSY;


	writel_relaxed(0x3, icp->icp_base + ICP_WD0_CTRL); // enable watchdog
	// ICP core reset
	writel_relaxed(ICP_SYS_RESET_FUNC_RESET | ICP_SYS_RESET_CPU_RESET, icp->icp_base + ICP_SYS_RESET);
	// IPE/BPS cores reset is not necessary as it's done by the ICP after booting but might be useful to debug issues or do hardware recovery
	reset_ipe_bps_cores(icp);

    // We need to provide the domain config before starting the core otherwise the firmware will complained about an invalid programming domain ID
	writel_relaxed(0x00FF00FF, icp->icp_base + ICP_DOM0_CFG);
	writel_relaxed(0xFF00FF00, icp->icp_base + ICP_DOM1_CFG);
    
	// Set reset vector address and boot ICP core
	writel_relaxed(0x0, icp->icp_base + ICP_SYS_RESET_VEC);
	writel_relaxed(ICP_SYS_CTRL_CPU_EN, icp->icp_base + ICP_SYS_CTRL);

	writel_relaxed(0x7F, icp->icp_base + ICP_OB_MASK); // enable all IRQ bits

	// Setup GP registers to give informations about the ICP memory regions
	writel_relaxed(config.qtbl_iova, icp->icp_base + ICP_GP_REG(6)); // qtbl iova
	writel_relaxed(config.sfr_iova, icp->icp_base + ICP_GP_REG(10)); // sfr iova
	writel_relaxed(config.shared_mem_iova, icp->icp_base + ICP_GP_REG(4)); // shared mem iova
	writel_relaxed(config.shared_mem_len, icp->icp_base + ICP_GP_REG(5)); // shared mem size
	writel_relaxed(config.sec_heap_iova, icp->icp_base + ICP_GP_REG(7)); // secondary heap iova
	writel_relaxed(config.sec_heap_len, icp->icp_base + ICP_GP_REG(8)); // secondary heap len
	writel_relaxed(config.qdss_mem_iova, icp->icp_base + ICP_GP_REG(11)); // qdss iova
	writel_relaxed(config.qdss_mem_len, icp->icp_base + ICP_GP_REG(12)); // qdss len
	writel_relaxed(config.io_mem_iova, icp->icp_base + ICP_GP_REG(13)); // io region 1 iova
	writel_relaxed(config.io_mem_len, icp->icp_base + ICP_GP_REG(14)); // io region 1 len
	writel_relaxed(0x0, icp->icp_base + ICP_GP_REG(15)); // io region 2 iova (not used)
	writel_relaxed(0x0, icp->icp_base + ICP_GP_REG(16)); // io region 2 len (not used)
	writel_relaxed(config.fwuncached_mem_iova, icp->icp_base + ICP_GP_REG(17)); // fwuncached region iova
	writel_relaxed(config.fwuncached_mem_len, icp->icp_base + ICP_GP_REG(18)); // fwuncached region len
	writel_relaxed(config.device_mem_iova, icp->icp_base + ICP_GP_REG(19)); // device region iova
	writel_relaxed(config.device_mem_len, icp->icp_base + ICP_GP_REG(20)); // device region len

	// Request ICP initialization and wait until we receive a successful init response
	writel_relaxed(0x1, icp->icp_base + ICP_GP_REG(2)); // host->icp init request
	int cnt = 10;
	while(cnt) {
		usleep_range(1000, 2000);
		if(readl_relaxed(icp->icp_base + ICP_GP_REG(3)) & 0x1) // icp->host init response
			break;
		cnt--;
	}
	if(!cnt) {
		dev_err(icp->dev, "ICP boot failure (init=0x%x) \n", readl_relaxed(icp->icp_base + ICP_GP_REG(3)));
		return -EIO;
	}

	u32 fw_version = readl_relaxed(icp->icp_base + ICP_GP_REG(1)); // icp fw version
	dev_warn(icp->dev, "ICP boot succeeded (firmware version: %u.%u.%u)\n", (fw_version&0xff000000)>>24, (fw_version&0x00ff0000)>>16, fw_version&0xffff); 

	return 0;
}

static void icp_shutdown(struct icp_data *icp)
{
	// Wait until ICP is not doing anything
	int cnt = 10;
	while(cnt) {
		usleep_range(10000, 20000);
		if(readl_relaxed(icp->icp_base + ICP_SYS_STATUS) & ICP_SYS_STATUS_STANDBY) // icp->host init response
			break;
		cnt--;
	}
	if(!cnt) {
		dev_err(icp->dev, "ICP core did not enter standby state during shutdown \n");
	}

	// Disable ICP
	writel_relaxed(0x0, icp->icp_base + ICP_SYS_CTRL);

	// Reset all ICP general purpose registers and ICP core
	for(int i=0; i<24; i++)
		writel_relaxed(0x0, icp->icp_base + ICP_GP_REG(i));
	writel_relaxed(ICP_SYS_RESET_FUNC_RESET | ICP_SYS_RESET_CPU_RESET, icp->icp_base + ICP_SYS_RESET);
	
	// Disable WD0 while ICP is shut down otherwise watchdog 0 will complain about a page fault
	// and we'll see the WD0_BITE in the OB_STATUS in our next attempt to boot
	writel_relaxed(0x0, icp->icp_base + ICP_WD0_CTRL);

	atomic_set(&icp->booted, 0);
}

static int icp_send_irq(struct icp_data *icp)
{
	writel_relaxed(0x1, icp->icp_base + ICP_HOST2ICPINT); // seems like any non-zero value sent to the HOST2ICP interupt is fine
	return 0;
}

static irqreturn_t icp_irq(int irq, void *data)
{
	struct icp_data *icp = data;

	u32 ob_status = readl_relaxed(icp->icp_base + ICP_OB_STATUS);
	writel_relaxed(ob_status, icp->icp_base + ICP_OB_CLEAR);
	writel_relaxed(ICP_OB_CMD_CLEAR, icp->icp_base + ICP_OB_CMD);

	// Wake up polling processes when getting the ICP->HOST interrupt (usually raised when something new is on the HFI queues)
	if (ob_status & ICP_OB_ICP2HOSTINT) {
		atomic_xchg(&icp->data_available, 1);
		wake_up_interruptible(&icp->waitqueue);
	}

	// We don't expect anything else than ICP2HOST and watchdog bark interupts
	if ((ob_status & ~(ICP_OB_ICP2HOSTINT | ICP_OB_WDT_BARK_WS0)) != 0x0) {
		dev_warn(icp->dev, "Unexpected ICP IRQ (irq=%d|%d) | ICP_OB_STATUS=0x%x | ICP_SYS_STATUS=0x%x | ICP_PFAULT_INFO=0x%x \n",
			irq, icp->icp_irq, ob_status, readl_relaxed(icp->icp_base + ICP_SYS_STATUS), readl_relaxed(icp->icp_base + ICP_PFAULT_INFO));
	}
		
	// Note: watchdog 0 bites can be cleared either with a custom clear sequence or by fully resetting the ICP

	return IRQ_HANDLED;
}

static irqreturn_t ipcc_irq(int irq, void *data)
{
	struct icp_data *icp = data;
	atomic_xchg(&icp->data_available, 1);
	wake_up_interruptible(&icp->waitqueue);
	return IRQ_HANDLED;
}

static int icp_open(struct inode *inode, struct file *file)
{
	struct icp_data *icp = container_of(file->private_data, struct icp_data, miscdev);
	int ret;

	// Make sure the file is opened by one user only
	if(atomic_xchg(&icp->open, 1))
		return -EBUSY;

	// Power is kept on while device is opened
	ret = pm_runtime_resume_and_get(icp->dev);
	if (ret < 0)
		return ret;

	return 0;
}

static int icp_release(struct inode *inode, struct file *file)
{
	struct icp_data *icp = container_of(file->private_data, struct icp_data, miscdev);
	struct dma_map *map, *n;

	// shutdown ICP if not already done before closing the file
	if(atomic_read(&icp->booted)) {
		icp_shutdown(icp);
	}

	// Clear iommu mappings
	list_for_each_entry_safe(map, n, &icp->maps, node)
		icp_unmap(icp, map);
	INIT_LIST_HEAD(&icp->maps);

	// Release power
	pm_runtime_put_sync(icp->dev);

	atomic_set(&icp->open, 0);
	return 0;
}

static long icp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct icp_data *icp = container_of(file->private_data, struct icp_data, miscdev);
	int ret = 0;
switch (cmd) {
	case ICP_IOCTL_BOOT:
		ret = icp_boot(icp, (void*)arg);
		break;
	case ICP_IOCTL_SHUTDOWN:
		icp_shutdown(icp);
		break;
	case ICP_IOCTL_MAP_DMABUF:
		ret = icp_map_dmabuf(icp, (void*)arg);
		break;
	case ICP_IOCTL_MAP_DIRECT:
		ret = icp_map_direct(icp, (void*)arg);
		break;
	case ICP_IOCTL_UNMAP_DMABUF:
		ret = icp_unmap_dmabuf_by_va(icp, (u64)arg);
		break;
	case ICP_IOCTL_SEND_IRQ:
		ret = icp_send_irq(icp);
		break;
	case ICP_IOCTL_SEND_IPCC:
		mbox_send_message(icp->mbox_chan, NULL);
		break;
	case ICP_IOCTL_SYNX_TRYLOCK:
		ret = hwspin_trylock_raw(icp->hwlock_synx);
		break;
	case ICP_IOCTL_SYNX_UNLOCK:
		hwspin_unlock_raw(icp->hwlock_synx);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}

static __poll_t icp_poll(struct file *file, struct poll_table_struct *wait)
{
	struct icp_data *icp = container_of(file->private_data, struct icp_data, miscdev);

	poll_wait(file, &icp->waitqueue, wait);
	if (atomic_xchg(&icp->data_available, 0))
		return POLLIN | POLLRDNORM;

	return 0;
}

struct file_operations icp_fops = {
	.open = icp_open,
	.release = icp_release,
	.unlocked_ioctl = icp_ioctl,
	.poll = icp_poll,
};

static int set_power(struct icp_data* icp)
{
	struct device* dev = icp->dev;
	int i, ret;

	icp->genpd_num = of_count_phandle_with_args(dev->of_node, "power-domains", "#power-domain-cells");
	if (icp->genpd_num < 0) {
		dev_err(dev, "Power domains are not defined for icp\n");
		return icp->genpd_num;
	}

	icp->genpd = devm_kmalloc_array(dev, icp->genpd_num, sizeof(*icp->genpd), GFP_KERNEL);
	if (!icp->genpd)
		return -ENOMEM;

	icp->genpd_link = devm_kmalloc_array(dev, icp->genpd_num, sizeof(*icp->genpd_link), GFP_KERNEL);
	if (!icp->genpd_link)
		return -ENOMEM;

	for (i=0; i < icp->genpd_num; i++) {
		icp->genpd[i] = NULL;
		icp->genpd_link[i] = NULL;
	}

	for (i=0; i < icp->genpd_num; i++) {
		icp->genpd[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(icp->genpd[i])) {
			ret = PTR_ERR(icp->genpd[i]);
			icp->genpd[i] = NULL;
			goto attach_error;
		}
	}

	for (i=0; i < icp->genpd_num; i++) {
		icp->genpd_link[i] = device_link_add(dev, icp->genpd[i],
											   DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME |
											   DL_FLAG_RPM_ACTIVE);
		if (icp->genpd_link[i] == NULL) {
			ret = -EINVAL;
			goto link_add_error;
		}
	}

	pm_runtime_enable(dev);

	return 0;

link_add_error:
	for (i=0; i < icp->genpd_num; i++) {
		if (icp->genpd_link[i] != NULL) {
			device_link_del(icp->genpd_link[i]);
		}
	}

attach_error:
	// Detach successfully attached power domains
	for (i=0; i < icp->genpd_num; i++) {
		if (icp->genpd[i] != NULL) {
			dev_pm_domain_detach(icp->genpd[i], true);
		}
	}

	return ret;
}

static void put_power(struct icp_data* icp)
{
	int i;

	pm_runtime_disable(icp->dev);

	for (i=0; i<icp->genpd_num; i++)
		device_link_del(icp->genpd_link[i]);

	for (i=0; i<icp->genpd_num; i++)
		dev_pm_domain_detach(icp->genpd[i], true);
}

static void set_clk_rate(struct icp_data *icp, const char *name, unsigned long rate)
{
	for(int i=0; i<icp->clk_cnt; i++) {
		if(strcmp(icp->clks[i].id, name) == 0) {
			if(clk_set_rate(icp->clks[i].clk, rate))
			{
				dev_err(icp->dev, "Failed to set clock rate for clock %s and rate %lu\n", icp->clks[i].id, rate);
			}
			return;
		}
	}
	dev_err(icp->dev, "Failed to find valid clock for input clock name %s\n", name);
}

static int icp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	struct icp_data *icp = devm_kzalloc(dev, sizeof(struct icp_data), GFP_KERNEL);
	if (!icp)
		return -ENOMEM;

	platform_set_drvdata(pdev, icp);
	icp->dev = dev;

	INIT_LIST_HEAD(&icp->maps);
	init_waitqueue_head(&icp->waitqueue);

	// IO memory mapping
	icp->icp_base = devm_platform_ioremap_resource_byname(pdev, "icp");
	if (IS_ERR(icp->icp_base))
		return PTR_ERR(icp->icp_base);

	icp->bps_base = devm_platform_ioremap_resource_byname(pdev, "bps");
	if (IS_ERR(icp->bps_base))
		return PTR_ERR(icp->bps_base);

	icp->ipe_base = devm_platform_ioremap_resource_byname(pdev, "ipe");
	if (IS_ERR(icp->ipe_base))
		return PTR_ERR(icp->ipe_base);

	// Do we need that?
	ret = dma_set_mask_and_coherent(dev, 0xffffffff);
	if (ret)
		return ret;

	// ICP bandwidth setup
	icp->sf_dma_path = devm_of_icc_get(dev, "cam_sf_0_mnoc");
	if(IS_ERR(icp->sf_dma_path)) {
		return PTR_ERR(icp->sf_dma_path);
	}
	icp->icp_dma_path = devm_of_icc_get(dev, "cam_sf_icp_mnoc");
	if(IS_ERR(icp->icp_dma_path)) {
		return PTR_ERR(icp->icp_dma_path);
	}
	icc_set_bw(icp->sf_dma_path, kBps_to_icc(0), kBps_to_icc(50*1000*1000));
	icc_set_bw(icp->icp_dma_path, kBps_to_icc(0), kBps_to_icc(50*1000*1000));

	// Get the hardware mutex used for synx (we could use another lock here since we manually create the DMA
	// mapping for the ICP, we just need to give the right TCSR mutex register range)
#define SYNX_HW_SPINLOCK_ID 0xa
	icp->hwlock_synx = devm_hwspin_lock_request_specific(dev, SYNX_HW_SPINLOCK_ID); // synx lock

	// IRQs
	ret = platform_get_irq_byname(pdev, "icp");
	if (ret < 0)
		return -ENODEV;
	icp->icp_irq = ret;
	ret = devm_request_irq(dev, icp->icp_irq, icp_irq, IRQF_TRIGGER_RISING, "icp", icp);
	if (ret)
		return -ENODEV;

	ret = platform_get_irq_byname(pdev, "ipcc");
	if (ret < 0)
		return -ENODEV;
	icp->ipcc_irq = ret;
	ret = devm_request_irq(dev, icp->ipcc_irq, ipcc_irq, IRQF_TRIGGER_RISING, "ipcc", icp);
	if (ret)
		return -ENODEV;

	// ERROR MANAGEMENT START HERE
	// Power domain
	ret = set_power(icp);
	if (ret)
		return ret;

	// Clocks
	ret = devm_clk_bulk_get_all(dev, &icp->clks);
	if (ret < 0)
		goto clk_setup_error;
	icp->clk_cnt = ret;

	// Set custom clock frequencies
	set_clk_rate(icp, "cpas_fast_ahb", 400000000);
	// Note that IPE/BPS use CAM_CC_CAMNOC_AXI_NRT_CLK but it uses the same frequency as CAM_CC_CAMNOC_AXI_RT_CLK
	set_clk_rate(icp, "camnoc_axi_rt", 400000000);
	set_clk_rate(icp, "icp_src", 600000000);
	set_clk_rate(icp, "bps_src", 785000000);
	set_clk_rate(icp, "ipe_nps_src", 825000000);

	ret = clk_bulk_prepare_enable(icp->clk_cnt, icp->clks);
	if (ret)
		goto clk_setup_error;

	// IOMMU
	icp->iommu = iommu_paging_domain_alloc(dev);
	ret = iommu_attach_device(icp->iommu, dev);
	if (ret < 0)
		goto iommu_error;

	// Setup misc device
	icp->miscdev.minor = MISC_DYNAMIC_MINOR;
	icp->miscdev.name = dev->of_node->name;
	icp->miscdev.fops = &icp_fops;
	icp->miscdev.parent = dev;
	ret = misc_register(&icp->miscdev);
	if (ret)
		goto misc_register_error;

	// Setup IPCC mailboxes
	icp->mbox_client.dev = dev;
	icp->mbox_client.knows_txdone = true;
	icp->mbox_chan = mbox_request_channel(&icp->mbox_client, 0);
	WARN_ON(IS_ERR(icp->mbox_chan));

	return 0;

misc_register_error:
	iommu_detach_device(icp->iommu, dev);
	iommu_domain_free(icp->iommu);
iommu_error:
	clk_bulk_disable_unprepare(icp->clk_cnt, icp->clks);
clk_setup_error:
	put_power(icp);

	return ret;
}

static void icp_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct icp_data *icp = platform_get_drvdata(pdev);

	misc_deregister(&icp->miscdev);

	mbox_free_channel(icp->mbox_chan);

	// Reset ICP bandwidth
	icc_set_bw(icp->sf_dma_path, kBps_to_icc(0), kBps_to_icc(0));
	icc_set_bw(icp->icp_dma_path, kBps_to_icc(0), kBps_to_icc(0));

	// Release IOMMU
	iommu_detach_device(icp->iommu, dev);
	iommu_domain_free(icp->iommu);

	// Free IRQ
	devm_free_irq(dev, icp->icp_irq, icp);
	devm_free_irq(dev, icp->ipcc_irq, icp);

	// Disable clocks
	clk_bulk_disable_unprepare(icp->clk_cnt, icp->clks);

	put_power(icp);
}

static const struct of_device_id icp_match_table[] = {
	{ .compatible = "qcom,sm8650-icp" },
	{ }
};
MODULE_DEVICE_TABLE(of, icp_match_table);

static struct platform_driver icp_driver = {
	.probe = icp_probe,
	.remove = icp_remove,
	.driver = {
		.name = "icp-sm8650",
		.of_match_table = icp_match_table,
	},
};
module_platform_driver(icp_driver);

 
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS("DMA_BUF");
