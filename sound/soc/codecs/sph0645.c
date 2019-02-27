/*
 * I2S MEMS microphone driver for SPH0645
 *
 * - Non configurable.
 * - I2S interface 24 bit data
 *
 * Copyright (c) 2015 Axis Communications AB
 *
 * Licensed under GPL v2.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#define SPH0645_RATE_MIN 32000 /* 7190  Hz, from data sheet */
#define SPH0645_RATE_MAX 64000 /* 52800 Hz, from data sheet */

#define SPH0645_FORMATS (SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32 ) //SNDRV_PCM_FMTBIT_S32

static struct snd_soc_dai_driver sph0645_dai = {
	.name = "sph0645",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rate_min = SPH0645_RATE_MIN,
		.rate_max = SPH0645_RATE_MAX,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.formats = SPH0645_FORMATS,
	},
};

static struct snd_soc_codec_driver sph0645_codec_driver = {
};

static int sph0645_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &sph0645_codec_driver,
			&sph0645_dai, 1);
}

static int sph0645_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sph0645_ids[] = {
	{ .compatible = "knowles,sph0645", },
	{ }
};
MODULE_DEVICE_TABLE(of, sph0645_ids);
#endif

static struct platform_driver sph0645_driver = {
	.driver = {
		.name = "sph0645",
		.of_match_table = of_match_ptr(sph0645_ids),
	},
	.probe = sph0645_probe,
	.remove = sph0645_remove,
};

module_platform_driver(sph0645_driver);

MODULE_DESCRIPTION("ASoC SPH0645 driver");
MODULE_AUTHOR("Alejandro Lozano <alejandro.lozano@nxp.com>");
MODULE_LICENSE("GPL v2");
