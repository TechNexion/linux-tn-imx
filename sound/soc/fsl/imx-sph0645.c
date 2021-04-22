/*
 * ASoC driver for BBB + NXP TFA98xx family of devices
 *
 * Author:      Sebastien Jan <sjan@baylibre.com>
 * Copyright (C) 2015 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG
#define pr_fmt(fmt) "%s(): " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include <asm/dma.h>
#include <asm-generic/types.h>

#define DAI_NAME_SIZE	32

static u32 imx_sph0645_rates[] = { 32000, 48000 };
static struct snd_pcm_hw_constraint_list imx_sph0645_rate_constraints = {
	.count = ARRAY_SIZE(imx_sph0645_rates),
	.list = imx_sph0645_rates,
};

struct snd_soc_card_drvdata_imx_tfa {
	struct clk *mclk;
	struct snd_soc_dai_link *dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	unsigned int clk_frequency;
	int pstreams;
	int cstreams;
};

static int imx_sph0645_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_imx_tfa *drvdata =
		snd_soc_card_get_drvdata(soc_card);
	int ret;

	pr_info("\n");

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &imx_sph0645_rate_constraints);
	if (ret)
		return ret;

	if (drvdata->mclk)
		return clk_prepare_enable(drvdata->mclk);

	return 0;
}

static void imx_sph0645_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_imx_tfa *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	pr_info("\n");
	if (drvdata->mclk)
		clk_disable_unprepare(drvdata->mclk);
}

static int imx_sph0645_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	int ret = 0;

	/* set cpu DAI configuration SND_SOC_DAIFMT_I2S*/
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
			| SND_SOC_DAIFMT_IB_IF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret) {
		dev_err(cpu_dai->dev, "failed to set cpu dai fmt\n");
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_OUT);
	if (ret) {
		dev_err(cpu_dai->dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}
	return ret;
}

static int imx_sph0645_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_imx_tfa *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->pstreams != 0 || drvdata->cstreams != 0)
		return 0;

	return 0;
}

static int imx_sph0645_trigger(struct snd_pcm_substream *stream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_card_drvdata_imx_tfa *drvdata =
		snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

	pr_info("\n");
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (stream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			drvdata->pstreams++;
		else
			drvdata->cstreams++;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (stream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (drvdata->pstreams > 0)
				drvdata->pstreams--;
			else
				pr_err("Error in playback streams count\n");
		} else {
			if (drvdata->cstreams > 0)
				drvdata->cstreams--;
			else
				pr_err("Error in capture streams count\n");
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return 0;
}

static struct snd_soc_ops imx_sph0645_ops = {
	.startup = imx_sph0645_startup,
	.shutdown = imx_sph0645_shutdown,
	.hw_params = imx_sph0645_hw_params,
	.hw_free = imx_sph0645_hw_free,
	.trigger = imx_sph0645_trigger,
};

static int imx_sph0645_init(struct snd_soc_pcm_runtime *rtd)
{
	pr_info("\n");
	dev_dbg(rtd->card->dev, "%s,%d: dai_init\n", __FUNCTION__, __LINE__);

	return 0;
}

static void *tfa_devm_kstrdup(struct device *dev, char *buf)
{
	char *str = devm_kzalloc(dev, strlen(buf) + 1, GFP_KERNEL);

	pr_info("\n");
	if (!str)
		return str;
	memcpy(str, buf, strlen(buf));
	return str;
}

#if defined(CONFIG_OF)
/*
 * The structs are used as place holders. They will be completely
 * filled with data from dt node.
 */
	
static struct snd_soc_dai_link imx_dai_sph0645[] = {
	{
		.name		= "HiFi",
		.stream_name	= "HiFi",
		.ops            = &imx_sph0645_ops,
		.init           = imx_sph0645_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS |
			SND_SOC_DAIFMT_NB_NF,
	},
};

static const struct of_device_id imx_tfa98_dt_ids[] = {
	{
		.compatible = "nxp,imx-audio-sph0645",
		.data = &imx_dai_sph0645,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_tfa98_dt_ids);

static const struct snd_soc_dapm_widget imx_sph0645_widgets[] = {
//	SND_SOC_DAPM_SPK("Speaker", NULL),
//	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_kcontrol_new imx_sph0645_controls[] = {
};

static struct snd_soc_card imx_sph0645_soc_card = {
	.owner = THIS_MODULE,
	.name = "SPH0645",	/* default name if none defined in DT */
	.dai_link = imx_dai_sph0645,
	.num_links = ARRAY_SIZE(imx_dai_sph0645),
	.controls = imx_sph0645_controls,
	.num_controls = ARRAY_SIZE(imx_sph0645_controls),
	.dapm_widgets = imx_sph0645_widgets,
	.num_dapm_widgets = ARRAY_SIZE(imx_sph0645_widgets),
};

static int imx_sph0645_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *np = pdev->dev.of_node;
	struct platform_device *cpu_pdev;
	struct snd_soc_dai_link_component *dlc;
	struct snd_soc_dai_link_component *codecs;
	struct snd_soc_dai_link *dai;
	struct snd_soc_card_drvdata_imx_tfa *drvdata = NULL;
	struct clk *mclk;
	int ret = 0;
	int i, num_codecs;

	pr_info("imx_sph0645_probe\n");

	imx_sph0645_soc_card.dev = &pdev->dev;

	dlc = devm_kzalloc(&pdev->dev, 2 * sizeof(*dlc), GFP_KERNEL);
	if (!dlc)
		return -ENOMEM;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}
	ret = snd_soc_of_parse_card_name(&imx_sph0645_soc_card, "nxp,model");
	if (ret)
		goto fail;;
	num_codecs = of_count_phandle_with_args(np, "nxp,audio-codec", NULL);
	if (num_codecs < 1) {
		ret = -EINVAL;
		goto fail;
	}
	pr_info("Found %d codec(s)\n", num_codecs);

	codecs = devm_kzalloc(&pdev->dev,
			sizeof(struct snd_soc_dai_link_component) * num_codecs,
			GFP_KERNEL);

	if (!codecs) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < num_codecs; i++) {
		char name[18];

		codecs[i].of_node = of_parse_phandle(np, "nxp,audio-codec", i);
		snprintf(name, sizeof(name), "sph0645");
		codecs[i].dai_name = tfa_devm_kstrdup(&pdev->dev, name);
	}
	dai = &imx_dai_sph0645[0];
	dai->cpus = &dlc[0];
	dai->num_cpus = 1;
	dai->platforms = &dlc[1];
	dai->num_platforms = 1;
	dai->codecs = codecs;
	dai->num_codecs = num_codecs;

	dai->platforms->of_node = cpu_np;
	dai->cpus->dai_name = dev_name(&cpu_pdev->dev);
	dai->cpus->of_node = of_parse_phandle(np, "ssi-controller", 0);
	if (!dai->cpus->of_node) {
		ret = -EINVAL;
		goto fail;
	}

	/* Only set the platforms->of_node if the platforms->name is not set */
	if (!dai->platforms->name)
		dai->platforms->of_node = dai->cpus->of_node;

	mclk = devm_clk_get(&pdev->dev, NULL);
	if (PTR_ERR(mclk) == -EPROBE_DEFER) {
		pr_info("getting clk defered\n");
		return -EPROBE_DEFER;
	} else if (IS_ERR(mclk)) {
		dev_dbg(&pdev->dev, "clock not found.\n");
		mclk = NULL;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata) {
		ret = -ENOMEM;
		goto fail;
	}

	if (mclk) {
		pr_info("Reference clock found: %s @ %ld\n",
			__clk_get_name(mclk),clk_get_rate(mclk));
		clk_prepare(mclk);
		clk_enable(mclk);
		drvdata->mclk = mclk;
	}
	drvdata->clk_frequency = clk_get_rate(drvdata->mclk);
	drvdata->dai = dai;

	platform_set_drvdata(pdev, &drvdata->card);
	snd_soc_card_set_drvdata(&imx_sph0645_soc_card, drvdata);

	ret = devm_snd_soc_register_card(&pdev->dev, &imx_sph0645_soc_card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);

	pr_info("done\n");
	return ret;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
#if 0
	if (codecs)
		of_node_put(codecs);
#endif

	return ret;
}

static int imx_sph0645_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct snd_soc_card_drvdata_imx_tfa *drvdata =
				snd_soc_card_get_drvdata(card);

	pr_info("\n");

	if (drvdata->mclk)
		clk_disable(drvdata->mclk);

	return 0;
}

static struct platform_driver imx_sph0645_driver = {
	.probe		= imx_sph0645_probe,
	.remove		= imx_sph0645_remove,
	.driver		= {
		.name	= "imx-sph0645",
		.owner	= THIS_MODULE,
		.pm	= &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(imx_tfa98_dt_ids),
	},
};
#endif

static int __init _sph0645_init(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt())
		return platform_driver_register(&imx_sph0645_driver);
#endif
	return 0;
}

static void __exit _sph0645_exit(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt()) {
		platform_driver_unregister(&imx_sph0645_driver);
		return;
	}
#endif
}

module_init(_sph0645_init);
module_exit(_sph0645_exit);

MODULE_AUTHOR("Alejandro Lozano");
MODULE_DESCRIPTION("i.MX7d SPH0645 I2S MEMS driver");
MODULE_LICENSE("GPL");
