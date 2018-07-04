/*
 *  kmb-mach.c - ASOC Machine driver for KMB
 *
 *  Copyright (C) 2018 Intel Corp
 *  Author: Sia Jee Heng <jee.heng.sia@intel.com>
 *  Author: Sit, Michael Wei Hong <michael.wei.hong.sit@intel.com>
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>

/* I2S support */
#define I2S_MONO_CHANNEL	1
#define I2S_STEREO_CHANNEL	2

#define I2S_MIN_RATE		8000
#define I2S_MAX_RATE		48000
#define I2S_MIN_PERIODS		10
#define I2S_MAX_PERIODS		50
#define I2S_FIFO_SIZE		8

#define I2S_MAX_BUFFER		96000
#define I2S_MIN_BUFFER		96000
#define I2S_MIN_PERIOD_BYTES	640
#define I2S_MAX_PERIOD_BYTES	48000

static struct snd_pcm_hardware kmb_pcm_hw_stereo = {
	.info =	(SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_DOUBLE |
		 SNDRV_PCM_INFO_PAUSE |
		 SNDRV_PCM_INFO_RESUME |
		 SNDRV_PCM_INFO_MMAP|
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_SYNC_START),
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
		   SNDRV_PCM_FMTBIT_S24_LE |
		   SNDRV_PCM_FMTBIT_S32_LE,
	.rates = (SNDRV_PCM_RATE_CONTINUOUS),
	.rate_min = I2S_MIN_RATE,
	.rate_max = I2S_MAX_RATE,
	.channels_min = I2S_MONO_CHANNEL,
	.channels_max = I2S_STEREO_CHANNEL,
	.buffer_bytes_max = I2S_MAX_BUFFER,
	.period_bytes_min = I2S_MIN_PERIOD_BYTES,
	.period_bytes_max = I2S_MAX_PERIOD_BYTES,
	.periods_min = I2S_MIN_PERIODS,
	.periods_max = I2S_MAX_PERIODS,
	.fifo_size = I2S_FIFO_SIZE,
};

static unsigned int channels_2[] = {
	2,
};

static struct snd_pcm_hw_constraint_list constraints_2ch = {
	.count	= ARRAY_SIZE(channels_2),
	.list	= channels_2,
};

static unsigned int rates[] = {
	16000,
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_rates = {
	.count	= ARRAY_SIZE(rates),
	.list	= rates,
};

static int kmb_mach_dai_link_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;

	return 0;
} /*kmb_mach_dai_link_prepare*/

static int kmb_mach_dai_link_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	int ret = 0;
	unsigned int fmt;

	fmt =   SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS; //Codec Slave, SSP Master
	return 0;
} /* kmb_mach_dai_link_hw_params*/

static int kmb_mach_dai_link_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *str_runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;

	str_runtime = substream->runtime;
	str_runtime->hw = kmb_pcm_hw_stereo;
	ret = snd_pcm_hw_constraint_list(str_runtime, 0,
						SNDRV_PCM_HW_PARAM_CHANNELS,
						&constraints_2ch);

	ret |= snd_pcm_hw_constraint_mask64(str_runtime,
						SNDRV_PCM_HW_PARAM_FORMAT,
						SNDRV_PCM_FMTBIT_S16_LE |
						SNDRV_PCM_FMTBIT_S24_LE |
						SNDRV_PCM_FMTBIT_S32_LE);

	if (ret) {
		pr_debug("%s : Fail to set channel constraint for stereo\n",
				__func__);
		return ret;
	}

	return snd_pcm_hw_constraint_list(str_runtime, 0,
					   SNDRV_PCM_HW_PARAM_RATE,
					   &constraints_rates);
} /*kmb_mach_dai_link_startup*/

static struct snd_soc_ops kmb_mach_dai_link_ops = {
	.startup = kmb_mach_dai_link_startup,
	.hw_params = kmb_mach_dai_link_hw_params,
	.prepare = kmb_mach_dai_link_prepare,
};

/* kmb digital audio interface glue */
static struct snd_soc_dai_link kmb_mach_dais[] = {
	{
		.name = "kmb_audio_card",
		.stream_name = "kmb audio",
		.cpu_dai_name = "kmb-plat-dai",
		.platform_name = "60080000.pcm_audio",
		.nonatomic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ops = &kmb_mach_dai_link_ops,
	},
};

/* kmb audio machine driver */
static struct snd_soc_card kmb_mach = {
	.name = "kmb_audio_card",
	.dai_link = kmb_mach_dais,
	.num_links = ARRAY_SIZE(kmb_mach_dais),
};

static int kmb_mach_audio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &kmb_mach;
	int ret;

	card->dev = &pdev->dev;
	kmb_mach.dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
	}

	return ret;
}

static int kmb_mach_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id kmb_mach_of_match[] = {
	{ .compatible = "intel,kmb-snd-asoc", },
	{},
};

static struct platform_driver kmb_mach_audio = {
	.probe = kmb_mach_audio_probe,
	.remove = kmb_mach_audio_remove,
	.driver = {
		.name = "kmb_mach",
		.of_match_table = kmb_mach_of_match,
	},
};

module_platform_driver(kmb_mach_audio)

/* Module information */
MODULE_DESCRIPTION("Intel Audio machine driver for KeemBay");
MODULE_AUTHOR("Sia Jee Heng <jee.heng.sia@intel.com>");
MODULE_AUTHOR("Sit, Michael Wei Hong <michael.wei.hong.sit@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:kmb_mach");