#include <linux/clk.h>
#include <linux/iommu.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#define BITRANGE(x, lo, hi) (((x) << lo) & GENMASK(hi, lo))

#define REG_IRQ_EN	0x9000
#define REG_IRQ_STAT	0x9004
#define REG_IRQ_CLEAR	0x900c

#define IRQ_PERIOD(a)	BIT((a) * 3 + 0)
#define IRQ_XRUN(a)	BIT((a) * 3 + 1) /* or overflow for WRDMA */
#define IRQ_ERR(a)	BIT((a) * 3 + 2)

/* values chosen to simplify lpaif_irq(), do not change without updating lpaif_irq()
 * (some LPAIF have more channels than this, but this should always be enough)
 */
#define RDDMA_MAX 5
#define WRDMA_MAX 4

#define I2S_BASE(i) (0x1000 + 0x1000 * (i))
#define RDDMA_BASE(i) (0xe000 + 0x1000 * (i))
#define WRDMA_BASE(i) (0x1a000 + 0x1000 * (i))

#define REG_DMA_CTL	0x00
#define REG_DMA_BASE	0x04
#define REG_DMA_LEN	0x08
#define REG_DMA_CURR_ADDR	0x0c
#define REG_DMA_PER_LEN		0x10
#define REG_DMA_CODEC_INTF	0x50

#define DMA_CTL_ENABLE BIT(0)
#define DMA_CTL_FIFO_WATERMARK(x) BITRANGE(x, 1, 5)
#define DMA_CTL_AUDIO_INTF(x) BITRANGE(x, 12, 15)
#define DMA_CTL_RESET BIT(31)

#define RDDMA_CTL_WPSCNT(x) BITRANGE(x, 16, 19)
#define RDDMA_CTL_BURST_EN BIT(20)
#define RDDMA_CTL_DYNAMIC_CLOCK BIT(21)
#define RDDMA_CTL_PADDING_EN BIT(22)
#define RDDMA_CTL_PADDING_NUM(x) BITRANGE(x, 23, 27)
#define RDDMA_CTL_BURST8 BIT(28)
#define RDDMA_CTL_BURST16 BIT(29)
#define RDDMA_CTL_DYNBURST BIT(30)

#define WRDMA_CTL_WPSCNT(x) BITRANGE(x, 17, 20)
#define WRDMA_CTL_BURST_EN BIT(21)
#define WRDMA_CTL_DYNAMIC_CLOCK BIT(22)
#define WRDMA_CTL_BURST8 BIT(23)
#define WRDMA_CTL_BURST16 BIT(24)

#define DMA_CODEC_INTF_NUM_ACTIVE_CHANNEL(x) BITRANGE(x, 0, 7)
#define DMA_CODEC_INTF_CODEC_INTF(x) BITRANGE(x, 16, 19)
#define DMA_CODEC_INTF_CODEC_FS_DELAY(x) BITRANGE(x, 21, 24)
#define DMA_CODEC_INTF_CODEC_FS_SEL(x) BITRANGE(x, 25, 27)
#define DMA_CODEC_INTF_CODEC_ENABLE_16BIT_PACKING BIT(29)
#define DMA_CODEC_INTF_CODEC_ENABLE BIT(30)
#define DMA_CODEC_INTF_CODEC_RESET BIT(31)

//..

#define DMA_CTL_DEFAULT (DMA_CTL_ENABLE | DMA_CTL_FIFO_WATERMARK(0x7) | \
			 DMA_CTL_AUDIO_INTF(0) | RDDMA_CTL_WPSCNT(0) |  \
			 RDDMA_CTL_BURST_EN | RDDMA_CTL_DYNAMIC_CLOCK)

#define DMA_CTL_DEFAULT_CAPTURE (DMA_CTL_ENABLE | DMA_CTL_FIFO_WATERMARK(0x7) | \
				 DMA_CTL_AUDIO_INTF(0) | WRDMA_CTL_WPSCNT(0) | \
				 WRDMA_CTL_BURST_EN |  WRDMA_CTL_DYNAMIC_CLOCK)


#define REG_I2S_CTL 0x00
#define I2S_CTL_BIT_WIDTH(x) BITRANGE(x, 0, 1)
#define I2S_CTL_WS_SRC BIT(2)
#define I2S_CTL_MIC_MONO BIT(3)
#define I2S_CTL_MIC_MODE(x) BITRANGE(x, 4, 8)
#define I2S_CTL_MIC_EN BIT(9)
#define I2S_CTL_SPKR_MONO BIT(10)
#define I2S_CTL_SPKR_MODE(x) BITRANGE(x, 11, 15)
#define I2S_CTL_SPKR_EN BIT(16)
#define I2S_CTL_LOOPBACK BIT(17)
#define I2S_CTL_LONG_RATE(x) BITRANGE(x, 18, 23)
#define I2S_CTL_EN_LONG_RATE BIT(24)
#define I2S_CTL_RESET BIT(31)

/* device tree config */
#define LPASS_INTF_CODEC BIT(15)
#define LPASS_INTF_CODEC_MASK(x) ((x)>>4&0xff)

struct lpaif {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data *clks;
	int num_clks;
	u32 rddma_count;
	u32 wrdma_count;
	bool no_lpm;

	struct lpaif_channel {
		void __iomem *base;
		struct snd_pcm_substream *substream;
		u32 intf;

		struct lpaif_channel *link_chan;
		struct lpaif_channel *slave_chan;
		struct snd_dma_buffer slave_buf;
		u32 slave_intf;
	} dma[RDDMA_MAX + WRDMA_MAX];
};

static struct lpaif_channel*
alloc_dma_channel(struct lpaif *lpaif, struct snd_pcm_substream *substream, int dir)
{
	struct lpaif_channel *ch = NULL;
	u32 i;
	/* XXX: add a spinlock */
	if (dir == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < lpaif->rddma_count; i++) {
			if (!lpaif->dma[i].substream) {
				ch = &lpaif->dma[i];
				ch->substream = substream;
				break;
			}
		}
	} else {
		for (i = 0; i < lpaif->wrdma_count; i++) {
			if (!lpaif->dma[RDDMA_MAX + i].substream) {
				ch = &lpaif->dma[RDDMA_MAX + i];
				ch->substream = substream;
				break;
			}
		}
	}
	return ch;
}

static inline bool
is_slave(struct lpaif *lpaif, struct lpaif_channel *ch)
{
	return (unsigned long) (ch - lpaif->dma) >= ARRAY_SIZE(lpaif->dma);
}

static int
lpaif_open(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct lpaif *lpaif = snd_soc_component_get_drvdata(component);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_runtime = snd_soc_substream_to_rtd(substream);
	struct lpaif_channel *ch;
	u32 intf, max_channels = 2; /* FIXME: only 1/2 channel I2S supported */
	int ret, type;

	/* XXX: make sure everything is cleaned up correctly if open() fails */

	clk_bulk_prepare_enable(lpaif->num_clks, lpaif->clks); /* TODO: clocks turn off when paused */

	if (runtime->private_data) { /* slave */
		ch = runtime->private_data;
		intf = (u32) soc_runtime->dai_link->id >> 16;

		if (intf & LPASS_INTF_CODEC)
			max_channels = hweight8(LPASS_INTF_CODEC_MASK(intf));
		snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_CHANNELS, 1, max_channels);

		/* allocate one page, but only 64 bytes are used.. */
		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV_WC, lpaif->dev, 0x1000, &ch->slave_buf);
		if (ret)
			return ret;

		ch->slave_intf = intf;
		ch->slave_chan = alloc_dma_channel(lpaif, substream, substream->stream);
		WARN_ON(!ch->slave_chan);

		/* XXX: pretty hacky, get the master lpaif to allocate channel */
		component = soc_runtime->components[soc_runtime->num_components - 2];
		lpaif = snd_soc_component_get_drvdata(component);

		ch->link_chan = alloc_dma_channel(lpaif, substream, !substream->stream);

		return 0;
	}

	intf = soc_runtime->dai_link->id & 0xffff;

	ch = alloc_dma_channel(lpaif, substream, substream->stream);
	if (!ch)
		return -ENOMEM;

	ch->intf = intf;

	runtime->private_data = ch;

	runtime->hw.info = SNDRV_PCM_INFO_MMAP |
			   SNDRV_PCM_INFO_MMAP_VALID |
			   SNDRV_PCM_INFO_INTERLEAVED |
			   SNDRV_PCM_INFO_PAUSE |
			   SNDRV_PCM_INFO_RESUME;
	runtime->hw.formats = SNDRV_PCM_FMTBIT_S16;
	runtime->hw.period_bytes_min = 32; /* lowest CURR_ADDR increment? */
	runtime->hw.period_bytes_max = 0x2000;
	runtime->hw.buffer_bytes_max = 0x2000;
	/* no limits on # of periods */
	runtime->hw.periods_min = 2;
	runtime->hw.periods_max = 16;

	if (intf & LPASS_INTF_CODEC)
		max_channels = hweight8(LPASS_INTF_CODEC_MASK(intf));
	snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_CHANNELS, 1, max_channels);

	if (lpaif->no_lpm)
		type = SNDRV_DMA_TYPE_NONCOHERENT;
	else
		type = SNDRV_DMA_TYPE_DEV_WC; /* NONCOHERENT doesn't work with shared-dma-pool */

	ret = snd_dma_alloc_pages(type, lpaif->dev, 0x2000, &substream->dma_buffer);
	if (ret)
		return -ENOMEM; // XXX free channel

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	if (type == SNDRV_DMA_TYPE_NONCOHERENT) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			substream->dma_buffer.dev.dir = DMA_TO_DEVICE;
		else
			substream->dma_buffer.dev.dir = DMA_FROM_DEVICE;
		substream->runtime->hw.info |= SNDRV_PCM_INFO_EXPLICIT_SYNC;
	}
	return 0;
}

static int
lpaif_close(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct lpaif *lpaif = snd_soc_component_get_drvdata(component);
	struct lpaif_channel *ch = substream->runtime->private_data;

	clk_bulk_disable_unprepare(lpaif->num_clks, lpaif->clks);

	if (is_slave(lpaif, ch))
		return 0;

	if (ch->slave_chan) {
		snd_dma_free_pages(&ch->slave_buf);
		ch->slave_chan->substream = NULL;
		ch->slave_chan = NULL;

		ch->link_chan->substream = NULL;
	}

	ch->substream = NULL;

	snd_dma_free_pages(&substream->dma_buffer);
	return 0;
}

static int
lpaif_prepare(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct lpaif *lpaif = snd_soc_component_get_drvdata(component);
	struct lpaif_channel *ch = substream->runtime->private_data;

	if (is_slave(lpaif, ch))
		return 0;

	if (ch->slave_chan) {
		/* - both sides are configured in "burst4" mode (16 byte read/writes)
		 * - the fifo watermark is set to 8 (32 bytes)
		 * both are started at the same time, the reading side will immediately
		 * read 32 bytes and be 32 bytes "ahead" of the writing side,
		 * which is equivalent to being (LEN-32) bytes "behind"
		 * the buffer size ("LEN") is what determines the added latency
		 *
		 * use LEN = 64 bytes which should be enough for this situation
		 * this is 0.333 ms of latency for 2ch 16-bit
		 * (can probably go lower by changing fifo watermark..)
		 */
		void __iomem *base = ch->slave_chan->base;
		writel_relaxed(ch->slave_buf.addr, base + REG_DMA_BASE);
		writel_relaxed(0xf, base + REG_DMA_LEN);
		writel_relaxed(~0u, base + REG_DMA_PER_LEN); /* no period interrupts */

		base = ch->link_chan->base;
		writel_relaxed(ch->slave_buf.addr, base + REG_DMA_BASE);
		writel_relaxed(0xf, base + REG_DMA_LEN);
		writel_relaxed(~0u, base + REG_DMA_PER_LEN); /* no period interrupts */
	}

	writel_relaxed(substream->runtime->dma_addr, ch->base + REG_DMA_BASE);
	writel_relaxed((snd_pcm_lib_buffer_bytes(substream) >> 2) - 1, ch->base + REG_DMA_LEN);
	writel_relaxed((snd_pcm_lib_period_bytes(substream) >> 2) - 1, ch->base + REG_DMA_PER_LEN);

	return 0;
}

static int
lpaif_trigger(struct snd_soc_component *component,
	      struct snd_pcm_substream *substream,
	      int cmd)
{
	struct lpaif *lpaif = snd_soc_component_get_drvdata(component);
	struct lpaif_channel *ch = substream->runtime->private_data;
	void __iomem *base = ch->base;
	void __iomem *i2s_base;
	u32 intf = ch->intf;
	u32 dma_ctl = DMA_CTL_DEFAULT;
	u32 dma_ctl2 = DMA_CTL_DEFAULT_CAPTURE;
	u32 codec_intf = 0;
	u32 i2s_ctl = 0;
	int i, j;
	bool is_master = !is_slave(lpaif, ch) && ch->slave_chan;

	if (is_slave(lpaif, ch)) {
		base = ch->slave_chan->base;
		intf = ch->slave_intf;
	}

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		dma_ctl = DMA_CTL_DEFAULT_CAPTURE;
		dma_ctl2 = DMA_CTL_DEFAULT;
	}

	if (intf & LPASS_INTF_CODEC) {
		u32 ch_mask = LPASS_INTF_CODEC_MASK(intf);
		/* keep only first "substream->runtime->channels" bits of ch_mask */
		for (i = 0, j = 0; j < substream->runtime->channels; i++) {
			if (ch_mask & BIT(i))
				j++;
		}
		ch_mask &= BIT(i) - 1;

		codec_intf =
			DMA_CODEC_INTF_NUM_ACTIVE_CHANNEL(ch_mask) |
			DMA_CODEC_INTF_CODEC_INTF(intf & 0xf) |
			DMA_CODEC_INTF_CODEC_FS_SEL(ffs(ch_mask)-1) | /* ctz(ch_mask) */
			DMA_CODEC_INTF_CODEC_ENABLE_16BIT_PACKING |
			DMA_CODEC_INTF_CODEC_ENABLE;
	} else {
		dma_ctl |= DMA_CTL_AUDIO_INTF(intf & 0xf);
		dma_ctl2 |= DMA_CTL_AUDIO_INTF(intf & 0xf);
		i2s_base = lpaif->base + I2S_BASE((intf & 0xf) - 1);
		/* assume 16bit 2ch playback */
		i2s_ctl = I2S_CTL_BIT_WIDTH(0) |
			  I2S_CTL_SPKR_MODE(1) |
			  I2S_CTL_SPKR_EN |
			  I2S_CTL_LONG_RATE(0xf);

		if (is_master)
			i2s_ctl |= I2S_CTL_LOOPBACK | I2S_CTL_MIC_EN | I2S_CTL_MIC_MODE(1);
	};

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		writel_relaxed(dma_ctl, base + REG_DMA_CTL);
		if (is_master)
			writel_relaxed(dma_ctl2, ch->link_chan->base + REG_DMA_CTL);
		if (codec_intf)
			writel_relaxed(codec_intf, base + REG_DMA_CODEC_INTF);
		if (i2s_ctl)
			writel_relaxed(i2s_ctl, i2s_base + REG_I2S_CTL);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		writel_relaxed(0, base + REG_DMA_CTL);
		if (is_master)
			writel_relaxed(0, ch->link_chan->base + REG_DMA_CTL);
		if (codec_intf)
			writel_relaxed(DMA_CODEC_INTF_CODEC_RESET, base + REG_DMA_CODEC_INTF);
		if (i2s_ctl)
			writel_relaxed(I2S_CTL_RESET, i2s_base + REG_I2S_CTL);
		break;
	}
	return 0;
}

static snd_pcm_uframes_t
lpaif_pointer(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct lpaif_channel *ch = substream->runtime->private_data;
	u32 curr_addr = readl_relaxed(ch->base + REG_DMA_CURR_ADDR);
	return bytes_to_frames(substream->runtime, curr_addr - substream->runtime->dma_addr);
}

static const struct snd_soc_component_driver lpaif_component_driver = {
	.name = "lpaif",
	.open = lpaif_open,
	.close = lpaif_close,
	.prepare = lpaif_prepare,
	.trigger = lpaif_trigger,
	.pointer = lpaif_pointer,
};

static irqreturn_t lpaif_irq(int irq, void *data)
{
	struct lpaif *lpaif = data;
	u32 status = readl_relaxed(lpaif->base + REG_IRQ_STAT);
	u32 status_ok = 0;
	int i;

	for (i = 0; i < 9; i++) {
		if ((status & IRQ_PERIOD(i)) && lpaif->dma[i].substream)
			snd_pcm_period_elapsed(lpaif->dma[i].substream);

		status_ok |= IRQ_PERIOD(i);
	}

	if (status & ~status_ok)
		dev_warn(lpaif->dev, "err/overflow/underflow irq (0x%.8x)\n", status);

	writel_relaxed(status, lpaif->base + REG_IRQ_CLEAR);
	return IRQ_HANDLED;
}

static int lpaif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lpaif *lpaif;
	int irq, ret, i;

	ret = of_reserved_mem_device_init(dev);
	if (ret && ret != -ENODEV)
		return ret;

	if (ret == -ENODEV) { // XXX
		struct iommu_domain *domain = iommu_get_dma_domain(dev);
		iommu_map(domain, 0x06000000, 0x06000000, SZ_16M, IOMMU_READ|IOMMU_WRITE, GFP_KERNEL);
	}

	lpaif = devm_kzalloc(dev, sizeof(*lpaif), GFP_KERNEL);
	if (!lpaif)
		return -ENOMEM;

	if (ret == -ENODEV)
		lpaif->no_lpm = true;

	lpaif->dev = dev;

	lpaif->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lpaif->base))
		return PTR_ERR(lpaif->base);

	ret = devm_clk_bulk_get_all(dev, &lpaif->clks);
	if (ret < 0)
		return ret;
	lpaif->num_clks = ret;

	/* a HW register tells how many DMA channels, but every LPAIF always has enough */
	lpaif->rddma_count = RDDMA_MAX;
	lpaif->wrdma_count = WRDMA_MAX;
	for (i = 0; i < lpaif->rddma_count; i++)
		lpaif->dma[i].base = lpaif->base + RDDMA_BASE(i);
	for (i = 0; i < lpaif->wrdma_count; i++)
		lpaif->dma[RDDMA_MAX + i].base = lpaif->base + WRDMA_BASE(i);

	dev_set_drvdata(dev, lpaif);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, lpaif_irq, IRQF_TRIGGER_HIGH, "lpaif", lpaif);
	if (ret < 0)
		return ret;

	/* enable all interrupts */
	writel_relaxed(~0u, lpaif->base + REG_IRQ_EN);

	return devm_snd_soc_register_component(dev, &lpaif_component_driver, NULL, 0);
}

static const struct of_device_id lpaif_dt_match[] = {
	{.compatible = "qcom,lpaif-v5"},
	{}
};

MODULE_DEVICE_TABLE(of, lpaif_dt_match);

static struct platform_driver lpaif_driver = {
	.probe  = lpaif_probe,
	.driver = {
		.name = "snd-lpaif",
		.of_match_table = lpaif_dt_match,
	},
};
module_platform_driver(lpaif_driver);


