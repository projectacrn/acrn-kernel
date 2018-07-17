/*
 *	kmb_platform.c - ASoC CPU DAI driver for KMB
 *
 *  Copyright (C) 2018 Intel Corp
 *  Authors:	Sia Jee Heng <jee.heng.sia@intel.com>
 *  Authors:	Sit, Michael Wei Hong <michael.wei.hong.sit@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *
 */

#include <linux/clk.h>
#include <linux/module.h>
#include "kmb_platform.h"
#include <linux/delay.h>
#include <linux/io.h>

#include <linux/interrupt.h>
#include <linux/rcupdate.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#define FORMAT(fmt) "%s: " fmt, __func__
#define pr_fmt(fmt) KBUILD_MODNAME ": " FORMAT(fmt)

#define BUFFER_BYTES_MAX	(3 * 2 * 8 * PERIOD_BYTES_MIN)
#define PERIOD_BYTES_MIN	4096
#define PERIODS_MIN		2

#define kmb_pcm_tx_fn(sample_bits) \
static unsigned int dw_pcm_tx_##sample_bits(struct kmb_i2s_info *dev, \
		struct snd_pcm_runtime *runtime, unsigned int tx_ptr, \
		bool *period_elapsed) \
{ \
	const u##sample_bits(*p)[2] = (void *)runtime->dma_area; \
	unsigned int period_pos = tx_ptr % runtime->period_size; \
	int i; \
\
	for (i = 0; i < dev->fifo_th; i++) { \
		iowrite32(p[tx_ptr][0], dev->i2s_base + LRBR_LTHR(0)); \
		iowrite32(p[tx_ptr][1], dev->i2s_base + RRBR_RTHR(0)); \
		period_pos++; \
		if (++tx_ptr >= runtime->buffer_size) \
			tx_ptr = 0; \
	} \
	*period_elapsed = period_pos >= runtime->period_size; \
	return tx_ptr; \
}


#define kmb_pcm_rx_fn(sample_bits) \
static unsigned int dw_pcm_rx_##sample_bits(struct kmb_i2s_info *dev, \
		struct snd_pcm_runtime *runtime, unsigned int rx_ptr, \
		bool *period_elapsed) \
{ \
	u##sample_bits(*p)[2] = (void *)runtime->dma_area; \
	unsigned int period_pos = rx_ptr % runtime->period_size; \
	int i; \
\
	for (i = 0; i < dev->fifo_th; i++) { \
		p[rx_ptr][0] = ioread32(dev->i2s_base + LRBR_LTHR(0)); \
		p[rx_ptr][1] = ioread32(dev->i2s_base + RRBR_RTHR(0)); \
		period_pos++; \
		if (++rx_ptr >= runtime->buffer_size) \
			rx_ptr = 0; \
	} \
	*period_elapsed = period_pos >= runtime->period_size; \
	return rx_ptr; \
}

kmb_pcm_tx_fn(16);
kmb_pcm_tx_fn(32);
kmb_pcm_rx_fn(16);
kmb_pcm_rx_fn(32);


#define CPR_PHY_ADDRESS 0x60070000
void __iomem *cpr_base;

/* Maximum bit resolution of a channel - not uniformly spaced */
static const u32 fifo_width[COMP_MAX_WORDSIZE] = {
	12, 16, 20, 24, 32, 0, 0, 0
};

/* Width of (DMA) bus */
static const u32 bus_widths[COMP_MAX_DATA_WIDTH] = {
	DMA_SLAVE_BUSWIDTH_1_BYTE,
	DMA_SLAVE_BUSWIDTH_2_BYTES,
	DMA_SLAVE_BUSWIDTH_4_BYTES,
	DMA_SLAVE_BUSWIDTH_UNDEFINED
};

/* PCM format to support channel resolution */
static const u32 formats[COMP_MAX_WORDSIZE] = {
	SNDRV_PCM_FMTBIT_S16_LE,
	SNDRV_PCM_FMTBIT_S16_LE,
	SNDRV_PCM_FMTBIT_S24_LE,
	SNDRV_PCM_FMTBIT_S24_LE,
	SNDRV_PCM_FMTBIT_S32_LE,
	0,
	0,
	0
};

static const struct snd_pcm_hardware kmb_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.rates = SNDRV_PCM_RATE_16000 |
		 SNDRV_PCM_RATE_48000,
	.rate_min = 16000,
	.rate_max = 48000,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
		   SNDRV_PCM_FMTBIT_S24_LE |
		   SNDRV_PCM_FMTBIT_S32_LE,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = BUFFER_BYTES_MAX,
	.period_bytes_min = PERIOD_BYTES_MIN,
	.period_bytes_max = BUFFER_BYTES_MAX / PERIODS_MIN,
	.periods_min = PERIODS_MIN,
	.periods_max = BUFFER_BYTES_MAX / PERIOD_BYTES_MIN,
	.fifo_size = 16,
};

static inline void write_cpr_reg(void __iomem *io_base, int reg, u32 val)
{
	writel(val, io_base + reg);
}

static inline u32 read_cpr_reg(void __iomem *io_base, int reg)
{
	return readl(io_base + reg);
}

static inline void i2s_write_reg(void __iomem *io_base, int reg, u32 val)
{
	writel(val, io_base + reg);
}

static inline u32 i2s_read_reg(void __iomem *io_base, int reg)
{
	return readl(io_base + reg);
}

static inline void i2s_disable_channels(struct kmb_i2s_info *dev, u32 stream)
{
	u32 i = 0;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
//	for (i = 0; i < 4; i++) // Commented out for single I2S port on PSS
		i2s_write_reg(dev->i2s_base, TER(i), 0);
	} else {
//	for (i = 0; i < 4; i++) // Commented out for single I2S port on PSS
		i2s_write_reg(dev->i2s_base, RER(i), 0);
	}
}

static inline void i2s_clear_irqs(struct kmb_i2s_info *dev, u32 stream)
{
	u32 i = 0;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
//	for (i = 0; i < 4; i++) // Commented out for single I2S port on PSS
		i2s_read_reg(dev->i2s_base, TOR(i));
	} else {
//	for (i = 0; i < 4; i++) // Commented out for single I2S port on PSS
		i2s_read_reg(dev->i2s_base, ROR(i));
	}
}

static inline void i2s_disable_irqs(struct kmb_i2s_info *dev, u32 stream,
				    int chan_nr)
{
	u32 i = 0, irq;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < (chan_nr / 2); i++) {
			irq = i2s_read_reg(dev->i2s_base, IMR(i));
			i2s_write_reg(dev->i2s_base, IMR(i), irq | 0x30);
		}
	} else {
		for (i = 0; i < (chan_nr / 2); i++) {
			irq = i2s_read_reg(dev->i2s_base, IMR(i));
			i2s_write_reg(dev->i2s_base, IMR(i), irq | 0x03);
		}
	}
}

static inline void i2s_enable_irqs(struct kmb_i2s_info *dev, u32 stream,
				   int chan_nr)
{
	u32 i = 0, irq;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < (chan_nr / 2); i++) {
			irq = i2s_read_reg(dev->i2s_base, IMR(i));
			i2s_write_reg(dev->i2s_base, IMR(i), irq & ~0x30);
		}
	} else {
		for (i = 0; i < (chan_nr / 2); i++) {
			irq = i2s_read_reg(dev->i2s_base, IMR(i));
			i2s_write_reg(dev->i2s_base, IMR(i), irq & ~0x03);
		}
	}
}

static void kmb_pcm_transfer(struct kmb_i2s_info *dev, bool push)
{
	struct snd_pcm_substream *substream;
	bool active, period_elapsed;

	rcu_read_lock();
	if (push)
		substream = rcu_dereference(dev->tx_substream);
	else
		substream = rcu_dereference(dev->rx_substream);
	active = substream && snd_pcm_running(substream);
	if (active) {
		unsigned int ptr;
		unsigned int new_ptr;

		if (push) {
			ptr = READ_ONCE(dev->tx_ptr);
			new_ptr = dev->tx_fn(dev, substream->runtime, ptr,
					&period_elapsed);
			cmpxchg(&dev->tx_ptr, ptr, new_ptr);
		} else {
			ptr = READ_ONCE(dev->rx_ptr);
			new_ptr = dev->rx_fn(dev, substream->runtime, ptr,
					&period_elapsed);
			cmpxchg(&dev->rx_ptr, ptr, new_ptr);
		}

		if (period_elapsed)
			snd_pcm_period_elapsed(substream);
	}
	rcu_read_unlock();
}

void kmb_pcm_push_tx(struct kmb_i2s_info *dev)
{
	int i;

	kmb_pcm_transfer(dev, true);
}

void kmb_pcm_pop_rx(struct kmb_i2s_info *dev)
{
	kmb_pcm_transfer(dev, false);
}

static int kmb_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct kmb_i2s_info *dev = snd_soc_dai_get_drvdata(rtd->cpu_dai);

	snd_soc_set_runtime_hwparams(substream, &kmb_pcm_hardware);
	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	runtime->private_data = dev;

	return 0;
}

static int kmb_pcm_close(struct snd_pcm_substream *substream)
{
	synchronize_rcu();
	return 0;
}

static int kmb_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int kmb_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct kmb_i2s_info *dev = runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			WRITE_ONCE(dev->tx_ptr, 0);
			rcu_assign_pointer(dev->tx_substream, substream);
		} else {
			WRITE_ONCE(dev->rx_ptr, 0);
			rcu_assign_pointer(dev->rx_substream, substream);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			rcu_assign_pointer(dev->tx_substream, NULL);
		else
			rcu_assign_pointer(dev->rx_substream, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static irqreturn_t i2s_irq_handler(int irq, void *dev_id)
{
	struct kmb_i2s_info *dev = dev_id;
	bool irq_valid = false;
	u32 isr[4], regval;
	int i = 0;

//	for (i = 0; i < 4; i++){// Commented out for single I2S port on PSS
		isr[i] = i2s_read_reg(dev->i2s_base, ISR(i));
//	}

	i2s_clear_irqs(dev, SNDRV_PCM_STREAM_PLAYBACK);
	i2s_clear_irqs(dev, SNDRV_PCM_STREAM_CAPTURE);

//	for (i = 0; i < 4; i++) { // Commented out for single I2S port on PSS
		/*
		 * Check if TX fifo is empty. If empty fill FIFO with samples
		 * NOTE: Only two channels supported
		 */
		if ((isr[i] & ISR_TXFE) && (i == 0)) {
			kmb_pcm_push_tx(dev);
			irq_valid = true;
		}

		/*
		 * Data available. Retrieve samples from FIFO
		 * NOTE: Only two channels supported
		 */
		if ((isr[i] & ISR_RXDA) && (i == 0)) {
			kmb_pcm_pop_rx(dev);
			irq_valid = true;
		}

		/* Error Handling: TX */
		if (isr[i] & ISR_TXFO) {
			dev_err(dev->dev, "TX overrun (ch_id=%d)\n", i);
			irq_valid = true;
		}

		/* Error Handling: RX */
		if (isr[i] & ISR_RXFO) {

			dev_err(dev->dev, "RX overrun (ch_id=%d)\n", i);
			irq_valid = true;
		}
//	}

	if (irq_valid)
		return IRQ_HANDLED;
	else
		return IRQ_NONE;
}

/*
 * KMB PLATFORM
 */

/*
 * KMB Platform functions
 */
static int kmb_platform_pcm_new(struct snd_soc_pcm_runtime *soc_runtime)
{
	size_t size = kmb_pcm_hardware.buffer_bytes_max;

	return snd_pcm_lib_preallocate_pages_for_all(soc_runtime->pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL), size, size);
} /* kmb_platform_pcm_new */

static void kmb_platform_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
} /* kmb_platform_pcm_free */

/**
 * kmb_pcm_hw_params - Allocate memory for Ring Buffer according
 * to hw_params.
 * It's called in a non-atomic context
 *
 * @param substream Substream for which the stream function is called
 * @param hw_params Stream command thats requested from upper layer
 * return status 0 ==> OK
 *
 */
static int kmb_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct kmb_i2s_info *dev = runtime->private_data;
	int ret_val;

	switch (params_channels(hw_params)) {
	case 2:
		break;
	default:
		dev_err(dev->dev, "invalid channels number\n");
		return -EINVAL;
	}

	switch (params_format(hw_params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dev->tx_fn = dw_pcm_tx_16;
		dev->rx_fn = dw_pcm_rx_16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		dev->tx_fn = dw_pcm_tx_32;
		dev->rx_fn = dw_pcm_rx_32;
		break;
	default:
		dev_err(dev->dev, "invalid format\n");
		return -EINVAL;
	}

	ret_val = snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(hw_params));

	if (ret_val < 0)
		return ret_val;
	else
		return 0;
} /* kmb_pcm_hw_params */

static snd_pcm_uframes_t kmb_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct kmb_i2s_info *dev = runtime->private_data;
	snd_pcm_uframes_t pos;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pos = READ_ONCE(dev->tx_ptr);
	else
		pos = READ_ONCE(dev->rx_ptr);

	return pos < runtime->buffer_size ? pos : 0;
}

static struct snd_pcm_ops kmb_platform_ops = {
	.open = kmb_pcm_open,
	.close = kmb_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = kmb_pcm_hw_params,
	.hw_free = kmb_pcm_hw_free,
	.prepare = NULL,
	.trigger = kmb_pcm_trigger,
	.pointer = kmb_pcm_pointer,
};

static const struct snd_soc_component_driver kmb_component = {
	.name           = "kmb",
	.ops		= &kmb_platform_ops,
	.probe		= NULL,
	.pcm_new	= kmb_platform_pcm_new,
	.pcm_free	= kmb_platform_pcm_free,
};

/*
 * SND SOC DAI OPs
 */
static int kmb_probe(struct snd_soc_dai *cpu_dai)
{
	return 0;
} /* kmb_probe */

static int kmb_remove(struct snd_soc_dai *cpu_dai)
{
	return 0;
} /* kmb_remove */

static void i2s_start(struct kmb_i2s_info *dev,
		      struct snd_pcm_substream *substream)
{
	struct i2s_clk_config_data *config = &dev->config;

	i2s_write_reg(dev->i2s_base, IER, 1);
	i2s_enable_irqs(dev, substream->stream, config->chan_nr);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		i2s_write_reg(dev->i2s_base, ITER, 1);
	else
		i2s_write_reg(dev->i2s_base, IRER, 1);

	i2s_write_reg(dev->i2s_base, CER, 1);
}

static void i2s_stop(struct kmb_i2s_info *dev,
		struct snd_pcm_substream *substream)
{
	i2s_clear_irqs(dev, substream->stream);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		i2s_write_reg(dev->i2s_base, ITER, 0);
	else
		i2s_write_reg(dev->i2s_base, IRER, 0);

	i2s_disable_irqs(dev, substream->stream, 8);

	if (!dev->active) {
		i2s_write_reg(dev->i2s_base, CER, 0);
		i2s_write_reg(dev->i2s_base, IER, 0);
	}
}

static int kmb_dai_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	struct kmb_i2s_info *dev = snd_soc_dai_get_drvdata(cpu_dai);
	union kmb_i2s_snd_dma_data *dma_data = NULL;

	if (!(dev->capability & DWC_I2S_RECORD) &&
			(substream->stream == SNDRV_PCM_STREAM_CAPTURE))
		return -EINVAL;

	if (!(dev->capability & DWC_I2S_PLAY) &&
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK))
		return -EINVAL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &dev->play_dma_data;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		dma_data = &dev->capture_dma_data;

	snd_soc_dai_set_dma_data(cpu_dai, substream, (void *)dma_data);

	return 0;
} /* kmb_dai_startup */

static void kmb_dai_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *cpu_dai)
{
	snd_soc_dai_set_dma_data(cpu_dai, substream, NULL);
} /* kmb_dai_shutdown */

static int kmb_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	struct kmb_i2s_info *dev_info = snd_soc_dai_get_drvdata(cpu_dai);
	int ret = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		if (dev_info->capability & DW_I2S_SLAVE)
			ret = 0;
		else
			ret = -EINVAL;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		if (dev_info->capability & DW_I2S_MASTER)
			ret = 0;
		else
			ret = -EINVAL;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
		ret = -EINVAL;
		break;
	default:
		dev_dbg(dev_info->dev, "kmb : Invalid master/slave format\n");
		ret = -EINVAL;
		break;
	}

	return 0;
} /* kmb_set_dai_fmt */

/**
 * kmb_dai_trigger- stream activities are handled here
 * This function is called whenever a stream activity is invoked
 * The Trigger function is called in an atomic context
 *
 * @param substream Substream for which the stream function is called
 * @param cmd The stream command thats requested from upper layer
 * return status 0 ==> OK
 *
 */
static int kmb_dai_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *cpu_dai)
{
	struct kmb_i2s_info *dev_info  = snd_soc_dai_get_drvdata(cpu_dai);
	int ret_val = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dev_info->active++;
		i2s_start(dev_info, substream);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dev_info->active--;
		i2s_stop(dev_info, substream);
		break;
	default:
		ret_val = -EINVAL;
		break;
	}
	return ret_val;
} /* kmb_dai_trigger */

static void i2s_config(struct kmb_i2s_info *dev, int stream)
{
	u32 ch_reg, regval;
	struct i2s_clk_config_data *config = &dev->config;

	i2s_disable_channels(dev, stream);

	for (ch_reg = 0; ch_reg < (config->chan_nr / 2); ch_reg++) {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK) {

			i2s_write_reg(dev->i2s_base, TCR(ch_reg),
				      dev->xfer_resolution);
			i2s_write_reg(dev->i2s_base, TFCR(ch_reg),
				      dev->fifo_th - 1);
			i2s_write_reg(dev->i2s_base, TER(ch_reg), 1);
		} else {
			i2s_write_reg(dev->i2s_base, RCR(ch_reg),
				      dev->xfer_resolution);
			i2s_write_reg(dev->i2s_base, RFCR(ch_reg),
				      dev->fifo_th - 1);
			i2s_write_reg(dev->i2s_base, RER(ch_reg), 1);
		}
	}
}

/**
 * kmb_dai_hw_params - Allocate memory for Ring Buffer according
 * to hw_params.
 * It's called in a non-atomic context
 *
 * @param substream Substream for which the stream function is called
 * @param hw_params Stream command thats requested from upper layer
 * @param cpu_dai Pointer to the CPU DAI that is used
 * return status 0 ==> OK
 *
 */
static int kmb_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params,
		struct snd_soc_dai *cpu_dai)
{
	struct kmb_i2s_info *dev_info = snd_soc_dai_get_drvdata(cpu_dai);
	struct i2s_clk_config_data *config = &dev_info->config;
	int ret;

	switch (params_format(hw_params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		config->data_width = 16;
		dev_info->ccr = 0x00;
		dev_info->xfer_resolution = 0x02;
		break;

	case SNDRV_PCM_FORMAT_S24_LE:
		config->data_width = 24;
		dev_info->ccr = 0x08;
		dev_info->xfer_resolution = 0x04;
		break;

	case SNDRV_PCM_FORMAT_S32_LE:
		config->data_width = 32;
		dev_info->ccr = 0x10;
		dev_info->xfer_resolution = 0x05;
		break;

	default:
		dev_err(dev_info->dev, "kmb: unsupported PCM fmt");
		return -EINVAL;
	}

	config->chan_nr = params_channels(hw_params);

	switch (config->chan_nr) {
	case EIGHT_CHANNEL_SUPPORT:
	case SIX_CHANNEL_SUPPORT:
	case FOUR_CHANNEL_SUPPORT:
	case TWO_CHANNEL_SUPPORT:
		break;
	default:
		dev_err(dev_info->dev, "channel not supported\n");
		return -EINVAL;
	}

	i2s_config(dev_info, substream->stream);

	i2s_write_reg(dev_info->i2s_base, CCR, dev_info->ccr);

	config->sample_rate = params_rate(hw_params);

	if (dev_info->capability & DW_I2S_MASTER) {
		if (dev_info->i2s_clk_cfg) {
			ret = dev_info->i2s_clk_cfg(config);
			if (ret < 0) {
				dev_err(dev_info->dev, "runtime audio clk config fail\n");
				return ret;
			}
		} else {
			u32 bitclk = config->sample_rate *
					config->data_width * 2;

			ret = clk_set_rate(dev_info->clk_i2s, bitclk);
			if (ret) {
				dev_err(dev_info->dev, "Can't set I2S clock rate: %d\n",
					ret);
				return ret;
			}
		}
	}
	return 0;
}

static int kmb_dai_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	struct kmb_i2s_info *dev_info = snd_soc_dai_get_drvdata(cpu_dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		i2s_write_reg(dev_info->i2s_base, TXFFR, 1);
	else
		i2s_write_reg(dev_info->i2s_base, RXFFR, 1);

	return 0;
}

static struct snd_soc_dai_ops kmb_dai_ops = {
	.startup	= kmb_dai_startup,
	.shutdown	= kmb_dai_shutdown,
	.trigger	= kmb_dai_trigger,
	.hw_params	= kmb_dai_hw_params,
	.prepare    = kmb_dai_prepare,
	.set_pll    = NULL,
	.set_fmt	= kmb_set_dai_fmt,
};

#define I2S_SUPPORTED_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

#define I2S_SAMPLE_RATES (SNDRV_PCM_RATE_8000_192000 | \
			SNDRV_PCM_RATE_CONTINUOUS)

#define I2S_SUPPORTED_FORMATS (SNDRV_PCM_FMTBIT_S24_LE | \
			       SNDRV_PCM_FMTBIT_S24_3LE | \
			       SNDRV_PCM_FMTBIT_S16_LE | \
			       SNDRV_PCM_FMTBIT_U16_LE | \
			       SNDRV_PCM_FMTBIT_S8 | \
			       SNDRV_PCM_FMTBIT_U8)


static struct snd_soc_dai_driver intel_kmb_platform_dai[] = {
	{
		.name = "kmb-plat-dai",
		.playback = {
			.channels_min = I2S_STEREO_CHANNEL,
			.channels_max = I2S_TDM8_CHANNEL,
			.rates = I2S_SAMPLE_RATES,
			.rate_min = I2S_MIN_RATE,
			.rate_max = I2S_MAX_RATE,
			/*FIXME: Used constraint list for i2s vs tdm8*/
			.formats = (SNDRV_PCM_FMTBIT_S32_LE |
				    SNDRV_PCM_FMTBIT_S24_LE |
				    SNDRV_PCM_FMTBIT_S16_LE),
		},
		.capture = {
			.channels_min = I2S_STEREO_CHANNEL,
			.channels_max = I2S_TDM8_CHANNEL,
			.rates = I2S_SAMPLE_RATES,
			.rate_min = I2S_MIN_RATE,
			.rate_max = I2S_MAX_RATE,
			/*FIXME: Used constraint list for i2s vs tdm8*/
			.formats = (SNDRV_PCM_FMTBIT_S32_LE |
				    SNDRV_PCM_FMTBIT_S24_LE |
				    SNDRV_PCM_FMTBIT_S16_LE),
		},
		.ops = &kmb_dai_ops,
		.probe = kmb_probe,
		.remove = kmb_remove,
	},

};

static int kmb_configure_dai(struct kmb_i2s_info *dev,
				   struct snd_soc_dai_driver *kmb_i2s_dai,
				   unsigned int rates)
{
	/*
	 * Read component parameter registers to extract
	 * the I2S block's configuration.
	 */
	u32 comp1 = i2s_read_reg(dev->i2s_base, dev->i2s_reg_comp1);
	u32 comp2 = i2s_read_reg(dev->i2s_base, dev->i2s_reg_comp2);
	u32 fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));
	u32 idx;

	if (dev->capability & DWC_I2S_RECORD &&
			dev->quirks & DW_I2S_QUIRK_COMP_PARAM1)
		comp1 = comp1 & ~BIT(5);

	if (COMP1_TX_ENABLED(comp1)) {
		dev_dbg(dev->dev, "kmb: play supported\n");
		idx = COMP1_TX_WORDSIZE_0(comp1);
		if (WARN_ON(idx >= ARRAY_SIZE(formats)))
			return -EINVAL;
		if (dev->quirks & DW_I2S_QUIRK_16BIT_IDX_OVERRIDE)
			idx = 1;
		kmb_i2s_dai->playback.channels_min = MIN_CHANNEL_NUM;
		kmb_i2s_dai->playback.channels_max =
				1 << (COMP1_TX_CHANNELS(comp1) + 1);
		kmb_i2s_dai->playback.formats = formats[idx];
		kmb_i2s_dai->playback.rates = rates;
	}

	if (COMP1_RX_ENABLED(comp1)) {
		dev_dbg(dev->dev, "kmb: record supported\n");
		idx = COMP2_RX_WORDSIZE_0(comp2);
		if (WARN_ON(idx >= ARRAY_SIZE(formats)))
			return -EINVAL;
		if (dev->quirks & DW_I2S_QUIRK_16BIT_IDX_OVERRIDE)
			idx = 1;
		kmb_i2s_dai->capture.channels_min = MIN_CHANNEL_NUM;
		kmb_i2s_dai->capture.channels_max =
				1 << (COMP1_RX_CHANNELS(comp1) + 1);
		kmb_i2s_dai->capture.formats = formats[idx];
		kmb_i2s_dai->capture.rates = rates;
	}

	if (COMP1_MODE_EN(comp1)) {
		dev_dbg(dev->dev, "kmb: i2s master mode supported\n");
		dev->capability |= DW_I2S_MASTER;
	} else {
		dev_dbg(dev->dev, "kmb: i2s slave mode supported\n");
		dev->capability |= DW_I2S_SLAVE;
	}

	dev->fifo_th = fifo_depth / 2;
	return 0;
}

static int kmb_configure_dai_by_dt(struct kmb_i2s_info *dev,
				   struct snd_soc_dai_driver *kmb_i2s_dai,
				   struct resource *res)
{
	u32 comp1 = i2s_read_reg(dev->i2s_base, I2S_COMP_PARAM_1);
	u32 comp2 = i2s_read_reg(dev->i2s_base, I2S_COMP_PARAM_2);
	u32 fifo_depth = 1 << (1 + COMP1_FIFO_DEPTH_GLOBAL(comp1));
	u32 idx = COMP1_APB_DATA_WIDTH(comp1);
	u32 idx2;
	int ret;

	if (WARN_ON(idx >= ARRAY_SIZE(bus_widths)))
		return -EINVAL;

	ret = kmb_configure_dai(dev, kmb_i2s_dai, SNDRV_PCM_RATE_8000_192000);
	if (ret < 0)
		return ret;

	if (COMP1_TX_ENABLED(comp1)) {
		idx2 = COMP1_TX_WORDSIZE_0(comp1);
		dev->capability |= DWC_I2S_PLAY;
		dev->play_dma_data.dt.addr = res->start + I2S_TXDMA;
		dev->play_dma_data.dt.addr_width = bus_widths[idx];
		dev->play_dma_data.dt.fifo_size = fifo_depth *
			(fifo_width[idx2]) >> 8;
		dev->play_dma_data.dt.maxburst = 16;
	}
	if (COMP1_RX_ENABLED(comp1)) {
		idx2 = COMP2_RX_WORDSIZE_0(comp2);

		dev->capability |= DWC_I2S_RECORD;
		dev->capture_dma_data.dt.addr = res->start + I2S_RXDMA;
		dev->capture_dma_data.dt.addr_width = bus_widths[idx];
		dev->capture_dma_data.dt.fifo_size = fifo_depth *
			(fifo_width[idx2] >> 8);
		dev->capture_dma_data.dt.maxburst = 16;
	}
	return 0;

}

static int kmb_plat_dai_probe(struct platform_device *pdev)
{
	int ret, irq;
	int index = pdev->id;
	struct kmb_i2s_info *i2s_info;
	struct snd_soc_dai_driver *kmb_i2s_dai;
	struct resource *res;
	u32 regval;
	const char *clk_id;

	cpr_base = ioremap(CPR_PHY_ADDRESS, 0x200);

	write_cpr_reg(cpr_base, 0x010, 0xFFFFFFFF);
	write_cpr_reg(cpr_base, 0x014, 0xFFFFFFF8);
	write_cpr_reg(cpr_base, 0x0, 0xFFFFFFFF);
	write_cpr_reg(cpr_base, 0x0100, 0xFFFFFFFF);
	write_cpr_reg(cpr_base, 0x0, 0xE3FFFFFF);
	write_cpr_reg(cpr_base, 0x070, 0x000A001E);
	write_cpr_reg(cpr_base, 0x0, 0xFFFFFFFF);
	write_cpr_reg(cpr_base, 0x0, 0xE3FFFFFF);
	write_cpr_reg(cpr_base, 0x11C, 0x1FF8000F);

	i2s_info = devm_kzalloc(&pdev->dev, sizeof(struct kmb_i2s_info),
				GFP_KERNEL);
	if (!i2s_info)
		return -ENOMEM;

	kmb_i2s_dai = devm_kzalloc(&pdev->dev, sizeof(*kmb_i2s_dai),
				GFP_KERNEL);
	if (!kmb_i2s_dai)
		return -ENOMEM;

	kmb_i2s_dai->ops = &kmb_dai_ops;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2s_info->i2s_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2s_info->i2s_base))
		return PTR_ERR(i2s_info->i2s_base);

	i2s_info->dev = &pdev->dev;

	irq = platform_get_irq(pdev, 0);
	if (irq >= 0) {
		ret = devm_request_irq(&pdev->dev, irq, i2s_irq_handler, IRQF_SHARED,
				pdev->name, i2s_info);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request irq\n");
			return ret;
		}
	}

	i2s_info->i2s_reg_comp1 = I2S_COMP_PARAM_1;
	i2s_info->i2s_reg_comp2 = I2S_COMP_PARAM_2;
		clk_id = "osc";
		ret = kmb_configure_dai_by_dt(i2s_info, kmb_i2s_dai, res);
	if (ret < 0)
		return ret;

	if (i2s_info->capability & DW_I2S_MASTER) {
		i2s_info->clk_i2s = devm_clk_get(&pdev->dev, clk_id);
		if (IS_ERR(i2s_info->clk_i2s)) {
			dev_err(&pdev->dev, "no clock configure method\n");
			return PTR_ERR(i2s_info->clk_i2s);
		}
		ret = clk_prepare_enable(i2s_info->clk_i2s);
		if (ret < 0)
			return ret;
	}

	ret = devm_snd_soc_register_component(&pdev->dev, &kmb_component,
				intel_kmb_platform_dai,
				ARRAY_SIZE(intel_kmb_platform_dai));

	if (ret) {
		dev_err(&pdev->dev, "not able to register dai\n");
		snd_soc_unregister_component(&pdev->dev);
		return -EBUSY;
	}

	dev_set_drvdata(&pdev->dev, i2s_info);
	return ret;
}

static int kmb_plat_dai_remove(struct platform_device *pdev)
{
	int i;
	int index = pdev->id;

	struct kmb_i2s_info *i2s_info = dev_get_drvdata(&pdev->dev);

	platform_set_drvdata(pdev, NULL);
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static const struct of_device_id kmb_plat_of_match[] = {
	{ .compatible = "snps,designware-i2s", },
	{},
};

static struct platform_driver kmb_plat_dai_driver = {
	.driver		= {
		.name		= "kmb-plat-dai",
		.of_match_table = kmb_plat_of_match,
		.owner		= THIS_MODULE,
	},
	.probe		= kmb_plat_dai_probe,
	.remove		= kmb_plat_dai_remove,
};


static int __init kmb_plat_dai_init(void)
{
	return  platform_driver_register(&kmb_plat_dai_driver);
}
module_init(kmb_plat_dai_init);

static void __exit kmb_plat_dai_exit(void)
{
	platform_driver_unregister(&kmb_plat_dai_driver);
}
module_exit(kmb_plat_dai_exit);

MODULE_DESCRIPTION("ASoC Intel(R) KeemBay Platform driver");
MODULE_AUTHOR("Sia Jee Heng <jee.heng.sia@intel.com>");
MODULE_AUTHOR("Sit, Michael Wei Hong <michael.wei.hong.sit@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:kmb_platform");
