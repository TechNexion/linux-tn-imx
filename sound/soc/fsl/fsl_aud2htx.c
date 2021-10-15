// SPDX-License-Identifier: GPL-2.0+
//
// Freescale ALSA SoC Digital Audio Interface (SAI) driver.
//
// Copyright 2012-2016 Freescale Semiconductor, Inc.

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <linux/dma-mapping.h>
#include "fsl_aud2htx.h"
#include "imx-pcm.h"

static int fsl_aud2htx_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *cpu_dai)
{
	struct fsl_aud2htx *aud2htx = snd_soc_dai_get_drvdata(cpu_dai);

	regmap_update_bits(aud2htx->regmap, AUD2HTX_CTRL_EXT,
			   AUD2HTX_CTRE_DT_MASK, 0);

	regmap_update_bits(aud2htx->regmap, AUD2HTX_IRQ_MASK,
			   AUD2HTX_WM_HIGH_IRQ_MASK |
			   AUD2HTX_WM_LOW_IRQ_MASK |
			   AUD2HTX_OVF_MASK,
			   AUD2HTX_WM_HIGH_IRQ_MASK |
			   AUD2HTX_WM_LOW_IRQ_MASK |
			   AUD2HTX_OVF_MASK);

	regmap_update_bits(aud2htx->regmap, AUD2HTX_CTRL_EXT,
			   AUD2HTX_CTRE_WL_MASK,
			   0x10 << AUD2HTX_CTRE_WL_SHIFT);
	regmap_update_bits(aud2htx->regmap, AUD2HTX_CTRL_EXT,
			   AUD2HTX_CTRE_WH_MASK,
			   0x10 << AUD2HTX_CTRE_WH_SHIFT);
	return 0;
}

static int fsl_aud2htx_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct fsl_aud2htx *aud2htx = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		regmap_update_bits(aud2htx->regmap, AUD2HTX_CTRL,
				   AUD2HTX_CTRL_EN, AUD2HTX_CTRL_EN);
		regmap_update_bits(aud2htx->regmap, AUD2HTX_CTRL_EXT,
				   AUD2HTX_CTRE_DE, AUD2HTX_CTRE_DE);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		regmap_update_bits(aud2htx->regmap, AUD2HTX_CTRL_EXT,
				   AUD2HTX_CTRE_DE, 0);
		regmap_update_bits(aud2htx->regmap, AUD2HTX_CTRL,
				   AUD2HTX_CTRL_EN, 0);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int fsl_aud2htx_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				      int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int fsl_aud2htx_set_dai_fmt(struct snd_soc_dai *cpu_dai,
				   unsigned int fmt)
{
	return 0;
}

static int fsl_aud2htx_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
					u32 tx_mask,
					u32 rx_mask, int slots, int slot_width)
{
	return 0;
}

static const struct snd_soc_dai_ops fsl_aud2htx_dai_ops = {
	.hw_params	= fsl_aud2htx_hw_params,
	.trigger	= fsl_aud2htx_trigger,
	.set_sysclk	= fsl_aud2htx_set_dai_sysclk,
	.set_fmt	= fsl_aud2htx_set_dai_fmt,
	.set_tdm_slot	= fsl_aud2htx_set_dai_tdm_slot,
};

static int fsl_aud2htx_dai_probe(struct snd_soc_dai *cpu_dai)
{
	struct fsl_aud2htx *aud2htx = dev_get_drvdata(cpu_dai->dev);

	snd_soc_dai_init_dma_data(cpu_dai, &aud2htx->dma_params_tx,
				  &aud2htx->dma_params_rx);

	return 0;
}

static struct snd_soc_dai_driver fsl_aud2htx_dai = {
	.probe = fsl_aud2htx_dai_probe,
	.playback = {
		.stream_name = "CPU-Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_32000 |
			 SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 |
			 SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 |
			 SNDRV_PCM_RATE_176400 |
			 SNDRV_PCM_RATE_192000,
		.formats = FSL_AUD2HTX_FORMATS,
	},
	.ops = &fsl_aud2htx_dai_ops,
};

static const struct snd_soc_component_driver fsl_aud2htx_component = {
	.name	= "fsl-aud2htx",
};

static const struct reg_default fsl_aud2htx_reg_defaults[] = {
	{AUD2HTX_CTRL,		0x00000000},
	{AUD2HTX_CTRL_EXT,	0x00000000},
	{AUD2HTX_WR,		0x00000000},
	{AUD2HTX_STATUS,	0x00000000},
	{AUD2HTX_IRQ_NOMASK,	0x00000000},
	{AUD2HTX_IRQ_MASKED,	0x00000000},
	{AUD2HTX_IRQ_MASK,	0x00000000},
};

static bool fsl_aud2htx_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AUD2HTX_CTRL:
	case AUD2HTX_CTRL_EXT:
	case AUD2HTX_STATUS:
	case AUD2HTX_IRQ_NOMASK:
	case AUD2HTX_IRQ_MASKED:
	case AUD2HTX_IRQ_MASK:
		return true;
	default:
		return false;
	}
}

static bool fsl_aud2htx_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AUD2HTX_CTRL:
	case AUD2HTX_CTRL_EXT:
	case AUD2HTX_WR:
	case AUD2HTX_IRQ_NOMASK:
	case AUD2HTX_IRQ_MASKED:
	case AUD2HTX_IRQ_MASK:
		return true;
	default:
		return false;
	}
}

static bool fsl_aud2htx_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AUD2HTX_STATUS:
	case AUD2HTX_IRQ_NOMASK:
	case AUD2HTX_IRQ_MASKED:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config fsl_aud2htx_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.max_register = AUD2HTX_IRQ_MASK,
	.reg_defaults = fsl_aud2htx_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(fsl_aud2htx_reg_defaults),
	.readable_reg = fsl_aud2htx_readable_reg,
	.volatile_reg = fsl_aud2htx_volatile_reg,
	.writeable_reg = fsl_aud2htx_writeable_reg,
	.cache_type = REGCACHE_RBTREE,
};

static const struct of_device_id fsl_aud2htx_dt_ids[] = {
	{ .compatible = "fsl,imx8mp-aud2htx",},
	{}
};

MODULE_DEVICE_TABLE(of, fsl_aud2htx_dt_ids);

static irqreturn_t fsl_aud2htx_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int fsl_aud2htx_probe(struct platform_device *pdev)
{
	struct fsl_aud2htx *aud2htx;
	struct resource *res;
	struct device_node *np;
	void __iomem *regs;
	int ret, irq;

	aud2htx = devm_kzalloc(&pdev->dev, sizeof(*aud2htx), GFP_KERNEL);
	if (!aud2htx)
		return -ENOMEM;

	aud2htx->pdev = pdev;
	np = pdev->dev.of_node;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs)) {
		dev_err(&pdev->dev, "failed ioremap\n");
		return PTR_ERR(regs);
	}

	aud2htx->regmap = devm_regmap_init_mmio_clk(&pdev->dev, NULL, regs,
						    &fsl_aud2htx_regmap_config);
	if (IS_ERR(aud2htx->regmap)) {
		dev_err(&pdev->dev, "failed to init regmap");
		return PTR_ERR(aud2htx->regmap);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq for node %s\n",
			dev_name(&pdev->dev));
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, fsl_aud2htx_isr, 0,
			       dev_name(&pdev->dev), aud2htx);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim irq %u: %d\n", irq, ret);
		return ret;
	}

	aud2htx->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (IS_ERR(aud2htx->bus_clk)) {
		dev_err(&pdev->dev, "failed to get mem clock\n");
		return PTR_ERR(aud2htx->bus_clk);
	}

	aud2htx->dma_params_tx.chan_name = "tx";
	aud2htx->dma_params_tx.maxburst = 16;
	aud2htx->dma_params_tx.addr = res->start + AUD2HTX_WR;

	platform_set_drvdata(pdev, aud2htx);
	pm_runtime_enable(&pdev->dev);

	regcache_cache_only(aud2htx->regmap, true);

	ret = devm_snd_soc_register_component(&pdev->dev,
					      &fsl_aud2htx_component,
					      &fsl_aud2htx_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ASoC DAI\n");
		goto err_pm_disable;
	}

	ret = imx_pcm_dma_init(pdev, IMX_DEFAULT_DMABUF_SIZE);
	if (ret) {
		dev_err(&pdev->dev, "failed to init imx pcm dma: %d\n", ret);
		goto err_snd_soc_unregister;
	}

	return ret;

err_snd_soc_unregister:
	snd_soc_unregister_component(&pdev->dev);

err_pm_disable:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int fsl_aud2htx_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int fsl_aud2htx_runtime_suspend(struct device *dev)
{
	struct fsl_aud2htx *aud2htx = dev_get_drvdata(dev);

	regcache_cache_only(aud2htx->regmap, true);
	clk_disable_unprepare(aud2htx->bus_clk);

	return 0;
}

static int fsl_aud2htx_runtime_resume(struct device *dev)
{
	struct fsl_aud2htx *aud2htx = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(aud2htx->bus_clk);
	if (ret)
		return ret;

	regcache_cache_only(aud2htx->regmap, false);
	regcache_mark_dirty(aud2htx->regmap);
	regcache_sync(aud2htx->regmap);

	return 0;
}
#endif /*CONFIG_PM*/

static const struct dev_pm_ops fsl_aud2htx_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_aud2htx_runtime_suspend,
			   fsl_aud2htx_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static struct platform_driver fsl_aud2htx_driver = {
	.probe = fsl_aud2htx_probe,
	.remove = fsl_aud2htx_remove,
	.driver = {
		.name = "fsl-aud2htx",
		.pm = &fsl_aud2htx_pm_ops,
		.of_match_table = fsl_aud2htx_dt_ids,
	},
};
module_platform_driver(fsl_aud2htx_driver);

MODULE_DESCRIPTION("NXP AUD2HTX driver");
MODULE_LICENSE("GPL v2");
