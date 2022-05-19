#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <linux/soundwire/sdw.h>
#include <trace/gpuvis.h>

#define BITRANGE(x, lo, hi) (((x) << lo) & GENMASK(hi, lo))

#define REG_I2S_CTL(a) (0x1000 + 0x1000 * (a))
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

#define REG_PCM_I2S_SEL(a) (0x1200 + 0x1000 * (a))

#define REG_PCM_CTL(a) (0x1500 + 0x1000 * (a))
#define PCM_CTL_TPCM_WIDTH_16_BIT	BIT(10)
#define PCM_CTL_RPCM_WIDTH_16_BIT	BIT(11)
#define PCM_CTL_AUX_MODE_AUX		BIT(12)
#define PCM_CTL_SYNC_SRC_INTERNAL 	BIT(13)
#define PCM_CTL_LOOPBACK		BIT(14)
#define PCM_CTL_RATE(x)			BITRANGE(x, 15, 17)
#define PCM_CTL_CTRL_DATA_OE_ENABLE	BIT(18)
#define PCM_CTL_ONE_SLOT_SYNC_EN 	BIT(19)
#define PCM_CTL_ENABLE			BIT(24)
#define PCM_CTL_ENABLE_TX 		BIT(25)
#define PCM_CTL_ENABLE_RX 		BIT(26)
#define PCM_CTL_RESET_TX 		BIT(27)
#define PCM_CTL_RESET_RX 		BIT(28)
#define PCM_CTL_RESET 			BIT(31)

#define REG_PCM_TDM_CTL(a) (0x1518 + 0x1000 * (a))
#define PCM_TDM_CTL_RATE(x)			BITRANGE(x, 0, 8)
#define PCM_TDM_CTL_RPCM_WIDTH(x)		BITRANGE(x, 9, 13)
#define PCM_TDM_CTL_TPCM_WIDTH(x)		BITRANGE(x, 14, 18)
#define PCM_TDM_CTL_SYNC_DELAY(x)		BITRANGE(x, 25, 26)
#define PCM_TDM_CTL_INV_RPCM_SYNC		BIT(27)
#define PCM_TDM_CTL_INV_TPCM_SYNC		BIT(28)
#define PCM_TDM_CTL_ENABLE_DIFF_SAMPLE_WIDTH	BIT(29)
#define PCM_TDM_CTL_ENABLE			BIT(30)

// SYNC_DELAY values
#define DELAY_2_CYCLE 0
#define DELAY_1_CYCLE 1
#define DELAY_0_CYCLE 2


#define REG_PCM_TDM_SAMPLE_WIDTH(a) (0x151c + 0x1000 * (a))
// only used if ENABLE_DIFF_SAMPLE_WIDTH

#define REG_PCM_RPCM_SLOT_NUM(a) (0x1520 + 0x1000 * (a))
#define REG_PCM_TPCM_SLOT_NUM(a) (0x1524 + 0x1000 * (a))
// mask of enabled slots

#define REG_PCM_LANE_CONFIG(a) (0x1528 + 0x1000 * (a))
// lane directions and enable


/* HDMI only */
#define REG_PARITY_CALC_EN 	0x34
#define REG_DMA0_CTRL		0x38
#define REG_CHANNEL_STATUS(x)	(0x48 + (x) * 8)
#define REG_VBIT_CTRL		0xc0


#define REG_IRQ_EN(x) ((x)->base + ((x)->id == AIF_HDMI ? 0x2004 : 0x9000))
#define REG_IRQ_STAT(x) ((x)->base + ((x)->id == AIF_HDMI ? 0x2008 : 0x9004))
#define REG_IRQ_CLEAR(x) ((x)->base + ((x)->id == AIF_HDMI ? 0x200c : 0x900c))
#define IRQ_RD_PER(a) BIT((a) * 3 + 0)
#define IRQ_RD_XRUN(a) BIT((a) * 3 + 1)
#define IRQ_RD_ERR(a) BIT((a) * 3 + 2)
#define IRQ_WR_PER(a) BIT((a) * 3 + 15)
#define IRQ_WR_XRUN(a) BIT((a) * 3 + 16)
#define IRQ_WR_ERR(a) BIT((a) * 3 + 17)

#define REG_RDDMA 0xe000
#define REG_WRDMA 0x1a000
#define REG_HDMIDMA 0x3000

#define REG_DMA_CTL(x, a) 	(reg_dma_base(x) + 0x00 + 0x1000 * (a))
#define REG_DMA_BASE(x, a) 	(reg_dma_base(x) + 0x04 + 0x1000 * (a))
#define REG_DMA_LEN(x, a) 	(reg_dma_base(x) + 0x08 + 0x1000 * (a))
#define REG_DMA_CURR_ADDR(x, a) (reg_dma_base(x) + 0x0c + 0x1000 * (a))
#define REG_DMA_PER_LEN(x, a) 	(reg_dma_base(x) + 0x10 + 0x1000 * (a))
#define REG_DMA_CODEC_INTF(x, a) (reg_dma_base(x) + 0x50 + 0x1000 * (a))

#define DMA_CTL_ENABLE BIT(0)
#define DMA_CTL_FIFO_WATERMARK(x) BITRANGE(x, 1, 5)
#define DMA_CTL_AUDIO_INTF(x) BITRANGE(x, 12, 15) /* note: not for HDMI */
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

#define HDMIDMA_CTL_WPSCNT(x) BITRANGE(x, 10, 12)
#define HDMIDMA_CTL_BURST_EN BIT(13)
#define HDMIDMA_CTL_DYNAMIC_CLOCK BIT(14)
#define HDMIDMA_CTL_BURST8 BIT(15)
#define HDMIDMA_CTL_BURST16 BIT(16)
#define HDMIDMA_CTL_GATHER_MODE BIT(17)
#define HDMIDMA_CTL_DYNBURST BIT(18)

#define DMA_CODEC_INTF_NUM_ACTIVE_CHANNEL(x) BITRANGE(x, 0, 7)
#define DMA_CODEC_INTF_CODEC_INTF(x) BITRANGE(x, 16, 19)
#define DMA_CODEC_INTF_CODEC_FS_DELAY(x) BITRANGE(x, 21, 24)
#define DMA_CODEC_INTF_CODEC_FS_SEL(x) BITRANGE(x, 25, 27)
#define DMA_CODEC_INTF_CODEC_ENABLE_16BIT_PACKING BIT(29)
#define DMA_CODEC_INTF_CODEC_ENABLE BIT(30)
#define DMA_CODEC_INTF_CODEC_RESET BIT(31)

//#define SND_CACHED_MAPPINGS_WITH_CACHE_SYNC

enum {
	AIF_WSA,
	AIF_VA,
	AIF_HDMI,
	AIF_MI2S,
	AIF_WSA2,
	AIF_SPKR_I2S,
	AIF_COUNT,
};

struct lpaif {
	void __iomem *base, *lpm_base;
	struct snd_pcm_substream *substream;
	u32 lpm_phys, lpm_size;
	int id;
};

static inline void __iomem* reg_dma_base(struct lpaif *aif)
{
	if (aif->id == AIF_WSA2)
		return aif->base + REG_RDDMA + 0x1000;
	if (aif->id == AIF_WSA || aif->id == AIF_SPKR_I2S)
		return aif->base + REG_RDDMA;
	else if (aif->id == AIF_VA || aif->id == AIF_MI2S)
		return aif->base + REG_WRDMA;
	else
		return aif->base + REG_HDMIDMA;
}

struct lpass {
	struct snd_soc_card card;
	struct snd_soc_dai_link links[5];
	struct snd_soc_dai_link_component codecs[6];
	struct snd_soc_dai_link_component platform;

	struct clk_bulk_data clks[6];

	struct sdw_stream_runtime *sruntime; /* for WSA */
	bool stream_prepared;
	void __iomem *hdmi_ctl;

	struct lpaif aif[AIF_COUNT];
};

static int sm8250_snd_startup(struct snd_pcm_substream *substream)
{
	TRACE_BEGIN_CTX();
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct lpass *lp = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *codec_dai;
	int ret, i;

	lp->sruntime = sdw_alloc_stream("WSA");
	if (!lp->sruntime)
		return -ENOMEM;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_stream(codec_dai, lp->sruntime,
					     substream->stream);
		if (ret < 0 && ret != -ENOTSUPP) {
			dev_err(rtd->dev, "Failed to set sdw stream on %s\n",
				codec_dai->name);
			sdw_release_stream(lp->sruntime);
			return ret;
		}
	}

	return 0;
}

static void sm8250_snd_shutdown(struct snd_pcm_substream *substream)
{
	TRACE_BEGIN_CTX();
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct lpass *lp = snd_soc_card_get_drvdata(rtd->card);

	sdw_release_stream(lp->sruntime);
}

static int sm8250_snd_prepare(struct snd_pcm_substream *substream)
{
	TRACE_BEGIN_CTX();
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct lpass *lp = snd_soc_card_get_drvdata(rtd->card);
	int ret;

	if (lp->stream_prepared)
		return 0;

	ret = sdw_prepare_stream(lp->sruntime);
	if (ret)
		return ret;

	/**
	 * NOTE: there is a strict hw requirement about the ordering of port
	 * enables and actual WSA881x PA enable. PA enable should only happen
	 * after soundwire ports are enabled if not DC on the line is
	 * accumulated resulting in Click/Pop Noise
	 * PA enable/mute are handled as part of codec DAPM and digital mute.
	 */

	ret = sdw_enable_stream(lp->sruntime);
	if (ret) {
		sdw_deprepare_stream(lp->sruntime);
		return ret;
	}
	lp->stream_prepared = true;

	return 0;
}

static int sm8250_snd_hw_free(struct snd_pcm_substream *substream)
{
	TRACE_BEGIN_CTX();
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct lpass *lp = snd_soc_card_get_drvdata(rtd->card);

	if (lp->stream_prepared) {
		sdw_disable_stream(lp->sruntime);
		sdw_deprepare_stream(lp->sruntime);
		lp->stream_prepared = false;
	}
	return 0;
}

static const struct snd_soc_ops sm8250_wsa_be_ops = {
	.startup = sm8250_snd_startup,
	.shutdown = sm8250_snd_shutdown,
	.hw_free = sm8250_snd_hw_free,
	.prepare = sm8250_snd_prepare,
};

static int sm8250_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	TRACE_BEGIN_CTX();
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	return 0;
}

static void aif_write(void __iomem *ptr, u32 val)
{
	TRACE_BEGIN_CTX();
	writel(val, ptr);
}

static struct lpaif* find_aif(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream)
{
	TRACE_BEGIN_CTX();
	struct lpass *lpass = snd_soc_component_get_drvdata(component);
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	return &lpass->aif[rtd->dai_link->id];
}

static int lpass_platform_pcmops_open(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream)
{
	TRACE_BEGIN_CTX();
	struct lpaif *aif = find_aif(component, substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	WARN_ON(aif->substream);
	aif->substream = substream;

	runtime->hw.info = SNDRV_PCM_INFO_MMAP |
			   SNDRV_PCM_INFO_MMAP_VALID |
			   SNDRV_PCM_INFO_INTERLEAVED |
			   SNDRV_PCM_INFO_PAUSE |
			   SNDRV_PCM_INFO_RESUME;
	runtime->hw.formats = SNDRV_PCM_FMTBIT_S16;
	if (aif->id == AIF_HDMI)
		runtime->hw.formats = SNDRV_PCM_FMTBIT_S24;
	runtime->hw.period_bytes_min = 32; /* lowest CURR_ADDR increment? */
	runtime->hw.period_bytes_max = aif->lpm_size;
	runtime->hw.periods_min = 1;
	runtime->hw.periods_max = 16; /* no real limit */
	runtime->hw.buffer_bytes_max = aif->lpm_size;
	runtime->hw.fifo_size = 0; /* TODO */

	substream->dma_buffer.area = aif->lpm_base;
	substream->dma_buffer.addr = aif->lpm_phys;
	substream->dma_buffer.bytes = aif->lpm_size;
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

#ifdef SND_CACHED_MAPPINGS_WITH_CACHE_SYNC
	// hack for snd_dma_lpm_sync to be used
	substream->dma_buffer.dev.type = SNDRV_DMA_TYPE_UNKNOWN;
	substream->dma_buffer.dev.dir = DMA_TO_DEVICE;
	if (aif->id == AIF_VA)
		substream->dma_buffer.dev.dir = DMA_FROM_DEVICE;
	substream->dma_buffer.dev.need_sync = 1;
	substream->runtime->hw.info |= SNDRV_PCM_INFO_EXPLICIT_SYNC;
#endif

	return 0;
}

static int lpass_platform_pcmops_close(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream)
{
	TRACE_BEGIN_CTX();
	struct lpaif *aif = find_aif(component, substream);

	WARN_ON(!aif->substream);
	aif->substream = NULL;
	return 0;
}

static int lpass_platform_pcmops_prepare(struct snd_soc_component *component,
					 struct snd_pcm_substream *substream)
{
	TRACE_BEGIN_CTX();
	struct lpass *lpass = snd_soc_component_get_drvdata(component);
	struct lpaif *aif = find_aif(component, substream);

	if (aif->id == AIF_HDMI) {
		/* reset and disable legacy mode */
		writel(BIT(31), lpass->hdmi_ctl);
		writel(0, lpass->hdmi_ctl);
		writel(0, lpass->hdmi_ctl + 0x08);

		/* HW insert P (parity), V (valid), C(channel status), U(user) */
		writel(1, aif->base + REG_PARITY_CALC_EN);
		writel(1, aif->base + REG_VBIT_CTRL);
		writel(3, aif->base + REG_DMA0_CTRL);

		/* channel status bits
		 * 48khz 16bit (even though only 24-bit is supported. required for Index audio)
		 */
		writel(0x02000000, aif->base + REG_CHANNEL_STATUS(0));
		writel(0x00000002, aif->base + REG_CHANNEL_STATUS(0) + 4);
	}

	aif_write(REG_DMA_BASE(aif, 0), aif->lpm_phys);
	aif_write(REG_DMA_LEN(aif, 0), (snd_pcm_lib_buffer_bytes(substream) >> 2) - 1);
	aif_write(REG_DMA_PER_LEN(aif, 0), (snd_pcm_lib_period_bytes(substream) >> 2) - 1);

	return 0;
}

static int lpass_platform_pcmops_trigger(struct snd_soc_component *component,
					 struct snd_pcm_substream *substream,
					 int cmd)
{
	TRACE_BEGIN_CTX();
	struct lpaif *aif = find_aif(component, substream);
	u32 codec_intf = DMA_CODEC_INTF_NUM_ACTIVE_CHANNEL(3) |
		DMA_CODEC_INTF_CODEC_INTF((aif->id == AIF_WSA2 ? 2 : 1)) |
		DMA_CODEC_INTF_CODEC_ENABLE |
		DMA_CODEC_INTF_CODEC_ENABLE_16BIT_PACKING;


	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		switch (aif->id) {
		case AIF_WSA:
			aif_write(REG_DMA_CTL(aif, 0), DMA_CTL_ENABLE | DMA_CTL_FIFO_WATERMARK(0x7) |
				DMA_CTL_AUDIO_INTF(0) | RDDMA_CTL_WPSCNT(0) | RDDMA_CTL_BURST_EN |
				RDDMA_CTL_DYNAMIC_CLOCK);
			aif_write(REG_DMA_CODEC_INTF(aif, 0), codec_intf);
			break;
		case AIF_SPKR_I2S:
			aif_write(aif->base + REG_I2S_CTL(0), I2S_CTL_RESET);
			aif_write(aif->base + REG_I2S_CTL(0),
				I2S_CTL_BIT_WIDTH(0) |
				// I2S_CTL_WS_SRC |
				//I2S_CTL_SPKR_MONO |
				I2S_CTL_SPKR_MODE(1) | // note: 1 is "SD0 only - stereo or mono"
				I2S_CTL_SPKR_EN |
				I2S_CTL_LONG_RATE(0xf));
			aif_write(REG_DMA_CTL(aif, 0), DMA_CTL_ENABLE | DMA_CTL_FIFO_WATERMARK(0x7) |
				DMA_CTL_AUDIO_INTF(1) | RDDMA_CTL_WPSCNT(0) | RDDMA_CTL_BURST_EN |
				RDDMA_CTL_DYNAMIC_CLOCK);
			break;
		case AIF_WSA2:
			aif_write(REG_DMA_CTL(aif, 0), DMA_CTL_ENABLE | DMA_CTL_FIFO_WATERMARK(0x7) |
				DMA_CTL_AUDIO_INTF(0) | RDDMA_CTL_WPSCNT(0) | RDDMA_CTL_BURST_EN |
				RDDMA_CTL_DYNAMIC_CLOCK);
			aif_write(REG_DMA_CODEC_INTF(aif, 0), codec_intf);
			break;
		case AIF_VA:
			aif_write(REG_IRQ_EN(aif), IRQ_WR_PER(0) | IRQ_WR_XRUN(0) | IRQ_WR_ERR(0));
			aif_write(REG_DMA_CTL(aif, 0), DMA_CTL_ENABLE | DMA_CTL_FIFO_WATERMARK(0x7) |
				DMA_CTL_AUDIO_INTF(0) | WRDMA_CTL_WPSCNT(0) | WRDMA_CTL_BURST_EN |
				WRDMA_CTL_DYNAMIC_CLOCK);
			aif_write(REG_DMA_CODEC_INTF(aif, 0), codec_intf);
			break;
		case AIF_MI2S:
#if 1
			// reset
			aif_write(aif->base + REG_PCM_I2S_SEL(2), 1); //PCM/TDM mode

			aif_write(aif->base + REG_PCM_CTL(2),
				PCM_CTL_RESET_RX |
				PCM_CTL_RESET);
			// master mode, slot sync, oe disable
			aif_write(aif->base + REG_PCM_CTL(2),
				PCM_CTL_SYNC_SRC_INTERNAL |
				PCM_CTL_ONE_SLOT_SYNC_EN);
			aif_write(aif->base + REG_PCM_TDM_CTL(2),
				PCM_TDM_CTL_RATE(127) | // bits per frame minus 1
				PCM_TDM_CTL_RPCM_WIDTH(31) | // bits per sample minus 1
				PCM_TDM_CTL_SYNC_DELAY(DELAY_1_CYCLE) |
				PCM_TDM_CTL_ENABLE_DIFF_SAMPLE_WIDTH |
				PCM_TDM_CTL_ENABLE);
			// this gives 32-bit long wordselect for slot sync
			// use 0x1EF for 16-bit long wordselect
			aif_write(aif->base + REG_PCM_TDM_SAMPLE_WIDTH(2), 0x3ff);
			// slot0-slot7 enabled
			aif_write(aif->base + REG_PCM_RPCM_SLOT_NUM(2), 0xff);
			// lane0 enabled and lane0 in mic mode
			// qcom driver doesn't write this (lane_cfg_is_supported==false)
			//aif_write(aif->base + REG_PCM_LANE_CONFIG(2), 0x10001);
			// enable rx
			aif_write(aif->base + REG_PCM_CTL(2),
				PCM_CTL_SYNC_SRC_INTERNAL |
				PCM_CTL_ONE_SLOT_SYNC_EN |
				PCM_CTL_ENABLE |
				PCM_CTL_ENABLE_RX);
#else
			aif_write(aif->base + REG_I2S_CTL(2), I2S_CTL_RESET);
			aif_write(aif->base + REG_I2S_CTL(2),
				I2S_CTL_BIT_WIDTH(0) |
				// I2S_CTL_WS_SRC |
				//I2S_CTL_SPKR_MONO |
				I2S_CTL_SPKR_MODE(1) | // note: 1 is "SD0 only - stereo or mono"
				I2S_CTL_SPKR_EN |
				I2S_CTL_LONG_RATE(0xf));
#endif
			aif_write(REG_IRQ_EN(aif), IRQ_RD_PER(0) | IRQ_RD_XRUN(0) | IRQ_RD_ERR(0));
			aif_write(REG_DMA_CTL(aif, 0), DMA_CTL_ENABLE | DMA_CTL_FIFO_WATERMARK(0x7) |
				DMA_CTL_AUDIO_INTF(3) | RDDMA_CTL_WPSCNT(0) | RDDMA_CTL_BURST_EN |
				RDDMA_CTL_DYNAMIC_CLOCK);
			break;
		case AIF_HDMI:
			writel(0x0005a00f, aif->base + 0x3000);
			writel(0x00003de9, aif->base + 0x1c);
			writel(0x00020002, aif->base + 0xc8);
			writel(0x0005e00f, aif->base + 0x3000);
			writel(0x01804007, REG_IRQ_EN(aif));
			writel(0x01804007, REG_IRQ_CLEAR(aif));
			break;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (aif->id == AIF_HDMI) {
			writel(0, REG_IRQ_EN(aif));
			writel(0x0005a00f, aif->base + 0x3000);
			writel(0x00003de8, aif->base + 0x1c);
			writel(0x00020003, aif->base + 0xc8);
			writel(0x0005a00e, aif->base + 0x3000);
			break;
		}

		writel(0, REG_DMA_CTL(aif, 0));
		if (aif->id != AIF_MI2S && aif->id != AIF_SPKR_I2S) {
			writel(codec_intf | DMA_CODEC_INTF_CODEC_RESET, REG_DMA_CODEC_INTF(aif, 0));
			// also write again with DMA_CODEC_INTF_CODEC_INTF cleared?
		}
		if (aif->id == AIF_VA)
			writel(0, REG_IRQ_EN(aif));
		break;
	}
	return 0;
}

static snd_pcm_uframes_t lpass_platform_pcmops_pointer(
		struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	TRACE_BEGIN_CTX();
	struct lpaif *aif = find_aif(component, substream);
	u32 curr_addr = readl(REG_DMA_CURR_ADDR(aif, 0));

	return bytes_to_frames(substream->runtime, curr_addr - aif->lpm_phys);
}

static int lpass_platform_pcmops_mmap(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream,
				      struct vm_area_struct *vma)
{
	TRACE_BEGIN_CTX();
	struct lpaif *ai = find_aif(component, substream);
	unsigned long user_count = vma_pages(vma);
	unsigned long count = ai->lpm_size >> PAGE_SHIFT;
	unsigned long pfn = PHYS_PFN(ai->lpm_phys);

#ifdef SND_CACHED_MAPPINGS_WITH_CACHE_SYNC
	// use cached (non-coherent) with explicit sync instead
#else
	vma->vm_page_prot = pgprot_dmacoherent(vma->vm_page_prot);
#endif

	if (vma->vm_pgoff >= count || user_count > count - vma->vm_pgoff)
		return -ENXIO;
	printk(KERN_INFO "lpass_platform_pcmops_mmap(vma, 0x%lx, %ld (0x%lx), %ld, prot)", vma->vm_start, pfn + vma->vm_pgoff, (pfn + vma->vm_pgoff) << PAGE_SHIFT, user_count << PAGE_SHIFT);
	return remap_pfn_range(vma, vma->vm_start, pfn + vma->vm_pgoff,
			user_count << PAGE_SHIFT, vma->vm_page_prot);
}

static const struct snd_soc_component_driver lpass_component_driver = {
	.name		= "lpass-platform",
	.open		= lpass_platform_pcmops_open,
	.close		= lpass_platform_pcmops_close,
	.prepare	= lpass_platform_pcmops_prepare,
	.trigger	= lpass_platform_pcmops_trigger,
	.pointer	= lpass_platform_pcmops_pointer,
	.mmap		= lpass_platform_pcmops_mmap,
};

static irqreturn_t lpass_wsa_irq(int irq, void *data)
{
	TRACE_BEGIN_CTX();
	struct lpass *lp = data;
	u32 irqs = readl(REG_IRQ_STAT(&lp->aif[AIF_WSA]));

	if ((irqs & IRQ_RD_PER(0)) && lp->aif[AIF_WSA].substream)
		snd_pcm_period_elapsed(lp->aif[AIF_WSA].substream);

	if ((irqs & IRQ_RD_PER(0)) && lp->aif[AIF_SPKR_I2S].substream)
		snd_pcm_period_elapsed(lp->aif[AIF_SPKR_I2S].substream);

	if ((irqs & IRQ_RD_PER(1)) && lp->aif[AIF_WSA2].substream)
		snd_pcm_period_elapsed(lp->aif[AIF_WSA2].substream);

	if (irqs & ~(IRQ_RD_PER(0) | IRQ_RD_PER(1)))
		printk("WSA AIF unexpected irqs=%.8x\n", irqs);

	writel(irqs, REG_IRQ_CLEAR(&lp->aif[AIF_WSA]));
	return IRQ_HANDLED;
}

static irqreturn_t lpass_va_irq(int irq, void *data)
{
	TRACE_BEGIN_CTX();
	struct lpass *lp = data;
	u32 irqs = readl(REG_IRQ_STAT(&lp->aif[AIF_VA]));

	if ((irqs & IRQ_WR_PER(0)) && lp->aif[AIF_VA].substream)
		snd_pcm_period_elapsed(lp->aif[AIF_VA].substream);

	if (irqs & ~IRQ_WR_PER(0))
		printk("VA AIF unexpected irqs=%.8x\n", irqs);

	writel(irqs, REG_IRQ_CLEAR(&lp->aif[AIF_VA]));
	return IRQ_HANDLED;
}

static irqreturn_t lpass_hdmi_irq(int irq, void *data)
{
	TRACE_BEGIN_CTX();
	struct lpass *lp = data;
	u32 irqs = readl(REG_IRQ_STAT(&lp->aif[AIF_HDMI]));

	if ((irqs & IRQ_RD_PER(0)) && lp->aif[AIF_HDMI].substream)
		snd_pcm_period_elapsed(lp->aif[AIF_HDMI].substream);

	if (irqs & ~(IRQ_RD_PER(0) | 0x01804000))
		printk("WSA AIF unexpected irqs=%.8x\n", irqs);

	//SDEDP_AUD_DIS
	//METADATA_DONE_ACK
	//REQON_PRELOAD_DMA0_ACK_
	writel(irqs | 0x01804000, REG_IRQ_CLEAR(&lp->aif[AIF_HDMI]));
	return IRQ_HANDLED;
}

static irqreturn_t lpass_mi2s_irq(int irq, void *data)
{
	TRACE_BEGIN_CTX();
	struct lpass *lp = data;
	u32 irqs = readl(REG_IRQ_STAT(&lp->aif[AIF_MI2S]));

	if ((irqs & IRQ_RD_PER(0)) && lp->aif[AIF_MI2S].substream)
		snd_pcm_period_elapsed(lp->aif[AIF_MI2S].substream);

	if (irqs & ~IRQ_RD_PER(0))
		printk("MI2S AIF unexpected irqs=%.8x\n", irqs);

	writel(irqs, REG_IRQ_CLEAR(&lp->aif[AIF_MI2S]));
	return IRQ_HANDLED;
}

static int sm8250_platform_probe(struct platform_device *pdev)
{
	TRACE_BEGIN_CTX();
	static const char *name_lpaif[] = { "wsa-lpaif", "va-lpaif", "hdmi-lpaif", "mi2s-lpaif" };
	static const char *name_lpm[] = { "wsa-lpm", "va-lpm", "hdmi-mi2s-lpm" };
	static irq_handler_t irq_fn[] = { lpass_wsa_irq, lpass_va_irq, lpass_hdmi_irq, lpass_mi2s_irq };
	struct resource *res;
	struct of_phandle_args args;
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card;
	struct lpass *lpass;
	struct lpaif *aif;
	int ret, i, irq;
	bool enable_wsa, enable_i2s, enable_hdmi;
	int codec_count;

	lpass = devm_kzalloc(dev, sizeof(*lpass), GFP_KERNEL);
	if (!lpass)
		return -ENOMEM;

	lpass->clks[0].id = "pcnoc-sway-clk";
	lpass->clks[1].id = "pcnoc-mport-clk";
	lpass->clks[2].id = "wsa-mclk";
	lpass->clks[3].id = "va-mclk";
	lpass->clks[4].id = "mi2s-ter-ibit";
	lpass->clks[5].id = "mi2s-sen-ibit";

	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(lpass->clks), lpass->clks);
	if (ret) {
		dev_err(dev, "Failed to get clocks %d\n", ret);
		return ret;
	}

	clk_set_rate(lpass->clks[4].clk, 24576000);
	clk_set_rate(lpass->clks[5].clk, 48000*32);

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(lpass->clks), lpass->clks);
	if (ret) {
		dev_err(dev, "Failed to enable clocks %d\n", ret);
		return ret;
	}
	/* XXX: unprepare clocks on cleanup */

	for (i = 0; i < 4; i++) {
		aif = &lpass->aif[i];
		aif->id = i;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name_lpaif[i]);
		aif->base = devm_ioremap_resource(dev, res);
		if (IS_ERR(aif->base))
			return PTR_ERR(aif->base);

		/* HDMI / MI2S share LPM. with this, don't use both at the same time! */
		if (i == 3) {
			aif->lpm_base = lpass->aif[i - 1].lpm_base;
			aif->lpm_phys = lpass->aif[i - 1].lpm_phys;
			aif->lpm_size = lpass->aif[i - 1].lpm_size;
		} else {

			res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name_lpm[i]);

#ifdef SND_CACHED_MAPPINGS_WITH_CACHE_SYNC
			// "devm_ioremap_cached"
			{
				void __iomem **ptr;
				ptr = devres_alloc(devm_ioremap_release, sizeof(*ptr), GFP_KERNEL);
				aif->lpm_base = ioremap_prot(res->start, resource_size(res), PROT_NORMAL);
				*ptr = aif->lpm_base;
				devres_add(dev, ptr);
			}
#else
			aif->lpm_base = devm_ioremap_resource_wc(dev, res);
#endif
			
			if (IS_ERR(aif->lpm_base))
				return PTR_ERR(aif->lpm_base);

			aif->lpm_phys = res->start;
			aif->lpm_size = resource_size(res);

		}

		irq = platform_get_irq_byname(pdev, name_lpaif[i]);
		if (irq < 0)
			return -ENODEV;

		ret = devm_request_irq(dev, irq, irq_fn[i],
				IRQF_TRIGGER_HIGH, name_lpaif[i], lpass);
		if (ret) {
			dev_err(&pdev->dev, "irq request failed: %d\n", ret);
			return ret;
		}
	}
	lpass->aif[AIF_WSA].lpm_size /= 2;
	aif = &lpass->aif[AIF_WSA2];
	aif->base = lpass->aif[AIF_WSA].base;
	aif->lpm_base = lpass->aif[AIF_WSA].lpm_base + lpass->aif[AIF_WSA].lpm_size;
	aif->lpm_phys = lpass->aif[AIF_WSA].lpm_phys + lpass->aif[AIF_WSA].lpm_size;
	aif->lpm_size = lpass->aif[AIF_WSA].lpm_size;
	aif->id = AIF_WSA2;

	/* SPKR I2S and WSA SWR share the same resources, do not use at the same time */
	aif = &lpass->aif[AIF_SPKR_I2S];
	aif->base = lpass->aif[AIF_WSA].base;
	aif->lpm_base = lpass->aif[AIF_WSA].lpm_base;
	aif->lpm_phys = lpass->aif[AIF_WSA].lpm_phys;
	aif->lpm_size = lpass->aif[AIF_WSA].lpm_size;
	aif->id = AIF_SPKR_I2S;

	writel(~0u, REG_IRQ_EN(&lpass->aif[AIF_WSA]));
	//writel(~0u, REG_IRQ_EN(&lpass->aif[AIF_VA]));

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hdmi-ctl");
	lpass->hdmi_ctl = devm_ioremap_resource(dev, res);
	if (IS_ERR(lpass->hdmi_ctl))
		return PTR_ERR(lpass->hdmi_ctl);

	card = &lpass->card;

	card->name = card->driver_name = "SM8250 LPASS";
	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, lpass);

	lpass->platform.of_node = dev->of_node;

	enable_wsa = of_property_read_bool(dev->of_node, "enable-wsa");
	enable_i2s = of_property_read_bool(dev->of_node, "enable-i2s");
	enable_hdmi = of_property_read_bool(dev->of_node, "enable-hdmi");
	if (!enable_wsa && !enable_i2s)
		enable_wsa = true;
	if (enable_wsa && enable_i2s)
		enable_i2s = false;
	if (enable_wsa)
		codec_count = ARRAY_SIZE(lpass->codecs);
	else
		codec_count = 3; // for spkr_i2s

	for (i = 0; i < codec_count; i++) {
		ret = of_parse_phandle_with_args(dev->of_node, "codecs",
				"#sound-dai-cells", i, &args);
		if (ret)
			return ret;

		lpass->codecs[i].of_node = args.np;
		ret = snd_soc_get_dai_name(&args, &lpass->codecs[i].dai_name);
		if (ret) {
			printk("snd_soc_get_dai_name %d %d\n", i, ret);
			return ret;
		} else {
        		printk("snd_soc_get_dai_name success %s\n", lpass->codecs[i].dai_name);
        	}
	}

	if (enable_wsa) {
		lpass->links[0] = (struct snd_soc_dai_link) {
			.name = "WSA Playback",
			.stream_name = "WSA Playback",
			.playback_only = true,
			.cpus = (void*) 1, // XXX see snd_soc_fill_dummy_dai
			.platforms = &lpass->platform,
			.num_platforms = 1,
			.codecs = &lpass->codecs[0],
			.num_codecs = 4,
			.be_hw_params_fixup = sm8250_be_hw_params_fixup,
			.ops = &sm8250_wsa_be_ops,
			.id = AIF_WSA,
		};
		lpass->links[1] = (struct snd_soc_dai_link) {
			.name = "VA Capture",
			.stream_name = "VA Capture",
			.capture_only = true,
			.cpus = (void*) 1, // XXX
			.platforms = &lpass->platform,
			.num_platforms = 1,
			.codecs = &lpass->codecs[5],
			.num_codecs = 1,
			.be_hw_params_fixup = sm8250_be_hw_params_fixup,
			.id = AIF_VA,
		};
		lpass->links[2] = (struct snd_soc_dai_link) {
			.name = "HDMI Playback",
			.stream_name = "HDMI Playback",
			.playback_only = true,
			.cpus = (void*) 1, // XXX
			.platforms = &lpass->platform,
			.num_platforms = 1,
			.codecs = (void*) 1, // XXX
			.be_hw_params_fixup = sm8250_be_hw_params_fixup,
			.id = AIF_HDMI,
		};
		lpass->links[3] = (struct snd_soc_dai_link) {
			.name = "WSA Playback2",
			.stream_name = "WSA Playback2",
			.playback_only = true,
			.cpus = (void*) 1, // XXX
			.platforms = &lpass->platform,
			.num_platforms = 1,
			.codecs = &lpass->codecs[4],
			.num_codecs = 1,
			.be_hw_params_fixup = sm8250_be_hw_params_fixup,
			.id = AIF_WSA2,
		};
		lpass->links[4] = (struct snd_soc_dai_link) {
			.name = "MI2S2 Capture",
			.stream_name = "MI2S2 Capture",
			.capture_only = true,
			.cpus = (void*) 1, // XXX
			.platforms = &lpass->platform,
			.num_platforms = 1,
			.codecs = (void*) 1, // XXX
			.be_hw_params_fixup = sm8250_be_hw_params_fixup,
			.id = AIF_MI2S,
		};

		card->num_links = ARRAY_SIZE(lpass->links);
	} else { // spkr_i2s
		lpass->links[0] = (struct snd_soc_dai_link) {
			.name = "SPKR Playback",
			.stream_name = "SPKR Playback",
			.playback_only = true,
			.cpus = (void*) 1, // XXX see snd_soc_fill_dummy_dai
			.platforms = &lpass->platform,
			.num_platforms = 1,
			.codecs = &lpass->codecs[0],
			.num_codecs = 2,
			.be_hw_params_fixup = sm8250_be_hw_params_fixup,
			.id = AIF_SPKR_I2S,
		};
		lpass->links[1] = (struct snd_soc_dai_link) {
			.name = "VA Capture",
			.stream_name = "VA Capture",
			.capture_only = true,
			.cpus = (void*) 1, // XXX
			.platforms = &lpass->platform,
			.num_platforms = 1,
			.codecs = &lpass->codecs[2],
			.num_codecs = 1,
			.be_hw_params_fixup = sm8250_be_hw_params_fixup,
			.id = AIF_VA,
		};
		card->num_links = 2;
		if (enable_hdmi) {
			lpass->links[2] = (struct snd_soc_dai_link) {
				.name = "HDMI Playback",
				.stream_name = "HDMI Playback",
				.playback_only = true,
				.cpus = (void*) 1, // XXX
				.platforms = &lpass->platform,
				.num_platforms = 1,
				.codecs = (void*) 1, // XXX
				.be_hw_params_fixup = sm8250_be_hw_params_fixup,
				.id = AIF_HDMI,
			};
			++card->num_links;
		}
	}

	card->dai_link = lpass->links;

	ret = devm_snd_soc_register_component(dev, &lpass_component_driver, NULL, 0);
	if (ret) {
		dev_err(dev, "error registering platform driver: %d\n", ret);
		return ret;
	}

	printk("sm8250_platform_probe a\n");

	return devm_snd_soc_register_card(dev, &lpass->card);
}

static const struct of_device_id snd_sm8250_dt_match[] = {
	{.compatible = "qcom,sm8650-lpass"},
	{}
};

MODULE_DEVICE_TABLE(of, snd_sm8250_dt_match);

static struct platform_driver snd_sm8250_driver = {
	.probe  = sm8250_platform_probe,
	.driver = {
		.name = "snd-sm8250",
		.of_match_table = snd_sm8250_dt_match,
	},
};
module_platform_driver(snd_sm8250_driver);
