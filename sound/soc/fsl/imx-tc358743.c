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

static u32 imx_tc358743_rates[] = { 32000, 48000 };
static struct snd_pcm_hw_constraint_list imx_tc358743_rate_constraints = {
	.count = ARRAY_SIZE(imx_tc358743_rates),
	.list = imx_tc358743_rates,
};

struct snd_soc_card_drvdata_imx_tc358743 {
	struct clk *mclk;
	struct snd_soc_dai_link *dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	unsigned int clk_frequency;
	int pstreams;
	int cstreams;
};

static int imx_tc358743_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_imx_tc358743 *drvdata =
		snd_soc_card_get_drvdata(soc_card);
	int ret;

	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &imx_tc358743_rate_constraints);
	if (ret)
		return ret;

	if (drvdata->mclk)
		return clk_prepare_enable(drvdata->mclk);

	return 0;
}

static void imx_tc358743_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_imx_tc358743 *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->mclk)
		clk_disable_unprepare(drvdata->mclk);
}

static int imx_tc358743_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	int ret = 0;

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, 0, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(cpu_dai->dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}
	return ret;
}

static int imx_tc358743_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_imx_tc358743 *drvdata =
		snd_soc_card_get_drvdata(soc_card);

	if (drvdata->pstreams != 0 || drvdata->cstreams != 0)
		return 0;

	return 0;
}

static int imx_tc358743_trigger(struct snd_pcm_substream *stream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_card_drvdata_imx_tc358743 *drvdata =
		snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

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

static struct snd_soc_ops imx_tc358743_ops = {
	.startup = imx_tc358743_startup,
	.shutdown = imx_tc358743_shutdown,
	.hw_params = imx_tc358743_hw_params,
	.hw_free = imx_tc358743_hw_free,
	.trigger = imx_tc358743_trigger,
};

#if defined(CONFIG_OF)
/*
 * The structs are used as place holders. They will be completely
 * filled with data from dt node.
 */

static struct snd_soc_dai_link imx_tc358743_dai_link = {
	.name		= "hdmi-in",
	.stream_name	= "hdmi-in",
	.ops            = &imx_tc358743_ops,
	.dai_fmt = SND_SOC_DAIFMT_I2S		/* I2S format */
		| SND_SOC_DAIFMT_CBM_CFM	/* Codec is master , imx is slave */
		| SND_SOC_DAIFMT_NB_NF,		/* BCLK and FS is normal , invert needless*/
};

static struct snd_soc_card imx_tc358743_soc_card = {
	.name = "card-hdmi-in",
	.owner = THIS_MODULE,
	.dai_link = &imx_tc358743_dai_link,
	.num_links = 1,
};

static int imx_tc358743_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np;
	struct device_node *codec_np;
	struct snd_soc_card_drvdata_imx_tc358743 *drvdata = NULL;
	struct snd_soc_dai_link_component *comp;
	struct clk *mclk;
	int ret = 0;

	imx_tc358743_soc_card.dev = &pdev->dev;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	codec_np = of_parse_phandle(pdev->dev.of_node, "nxp,audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle of codec missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	comp = devm_kzalloc(&pdev->dev, 3 * sizeof(*comp), GFP_KERNEL);
	if (!comp) {
	        ret = -ENOMEM;
	        goto fail;
	}

	imx_tc358743_dai_link.cpus	= &comp[0];
	imx_tc358743_dai_link.codecs    = &comp[1];
	imx_tc358743_dai_link.platforms = &comp[2];
	imx_tc358743_dai_link.num_cpus  = 1;
	imx_tc358743_dai_link.num_codecs        = 1;
	imx_tc358743_dai_link.num_platforms     = 1;
	imx_tc358743_dai_link.codecs->of_node = codec_np;
	imx_tc358743_dai_link.codecs->dai_name = "tc358743"; /*please reference the 'name' attribute of snd_soc_dai_driver object of codec driver*/
	imx_tc358743_dai_link.platforms->of_node = cpu_np;
	imx_tc358743_dai_link.cpus->of_node = cpu_np;

	mclk = devm_clk_get(&pdev->dev, NULL);
	if (PTR_ERR(mclk) == -EPROBE_DEFER) {
		dev_err(&pdev->dev, "getting clk defered\n");
		return -EPROBE_DEFER;
	} else if (IS_ERR(mclk)) {
		dev_err(&pdev->dev, "clock not found.\n");
		mclk = NULL;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata) {
		ret = -ENOMEM;
		goto fail;
	}

	if (mclk) {
		dev_info(&pdev->dev, "Reference clock found: %s @ %ld\n",
				__clk_get_name(mclk),clk_get_rate(mclk));
		clk_prepare(mclk);
		clk_enable(mclk);
		drvdata->mclk = mclk;
	}
	drvdata->clk_frequency = clk_get_rate(drvdata->mclk);
	drvdata->dai = &imx_tc358743_dai_link;

	platform_set_drvdata(pdev, &drvdata->card);
	snd_soc_card_set_drvdata(&imx_tc358743_soc_card, drvdata);

	ret = devm_snd_soc_register_card(&pdev->dev, &imx_tc358743_soc_card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);

	return ret;
fail:
	if (cpu_np)
		of_node_put(cpu_np);

	return ret;
}

static int imx_tc358743_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct snd_soc_card_drvdata_imx_tc358743 *drvdata =
		snd_soc_card_get_drvdata(card);

	if (drvdata->mclk)
		clk_disable(drvdata->mclk);

	return 0;
}

static const struct of_device_id imx_tc358743_dt_ids[] = {
	{
		.compatible = "nxp,imx-audio-tc358743",
		.data = &imx_tc358743_dai_link,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_tc358743_dt_ids);

static struct platform_driver imx_tc358743_driver = {
	.probe		= imx_tc358743_probe,
	.remove		= imx_tc358743_remove,
	.driver		= {
		.name	= "imx-tc358743",
		.owner	= THIS_MODULE,
		.pm	= &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(imx_tc358743_dt_ids),
	},
};
#endif

static int __init _tc358743_init(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt())
		return platform_driver_register(&imx_tc358743_driver);
#endif
	return 0;
}

static void __exit _tc358743_exit(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt()) {
		platform_driver_unregister(&imx_tc358743_driver);
		return;
	}
#endif
}

module_init(_tc358743_init);
module_exit(_tc358743_exit);

MODULE_DESCRIPTION("i.MX with toshiba tc358743 hdmi in driver");
