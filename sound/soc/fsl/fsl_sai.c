// SPDX-License-Identifier: GPL-2.0+
//
// Freescale ALSA SoC Digital Audio Interface (SAI) driver.
//
// Copyright 2012-2015 Freescale Semiconductor, Inc.

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pm_qos.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/pm_qos.h>
#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>
#include <linux/busfreq-imx.h>

#include "fsl_dsd.h"
#include "fsl_sai.h"
#include "imx-pcm.h"

#define FSL_SAI_FLAGS (FSL_SAI_CSR_SEIE |\
		       FSL_SAI_CSR_FEIE)

static const unsigned int fsl_sai_rates[] = {
	8000, 11025, 12000, 16000, 22050,
	24000, 32000, 44100, 48000, 64000,
	88200, 96000, 176400, 192000, 352800,
	384000, 705600, 768000, 1411200, 2822400,
};

static const struct snd_pcm_hw_constraint_list fsl_sai_rate_constraints = {
	.count = ARRAY_SIZE(fsl_sai_rates),
	.list = fsl_sai_rates,
};

/**
 * fsl_sai_dir_is_synced - Check if stream is synced by the opposite stream
 *
 * SAI supports synchronous mode using bit/frame clocks of either Transmitter's
 * or Receiver's for both streams. This function is used to check if clocks of
 * the stream's are synced by the opposite stream.
 *
 * @sai: SAI context
 * @dir: stream direction
 */
static inline bool fsl_sai_dir_is_synced(struct fsl_sai *sai, int dir)
{
	int adir = (dir == TX) ? RX : TX;

	/* current dir in async mode while opposite dir in sync mode */
	return !sai->synchronous[dir] && sai->synchronous[adir];
}

static irqreturn_t fsl_sai_isr(int irq, void *devid)
{
	struct fsl_sai *sai = (struct fsl_sai *)devid;
	unsigned int ofs = sai->soc_data->reg_offset;
	struct device *dev = &sai->pdev->dev;
	u32 flags, xcsr, mask;
	bool irq_none = true;

	/*
	 * Both IRQ status bits and IRQ mask bits are in the xCSR but
	 * different shifts. And we here create a mask only for those
	 * IRQs that we activated.
	 */
	mask = (FSL_SAI_FLAGS >> FSL_SAI_CSR_xIE_SHIFT) << FSL_SAI_CSR_xF_SHIFT;

	/* Tx IRQ */
	regmap_read(sai->regmap, FSL_SAI_TCSR(ofs), &xcsr);
	flags = xcsr & mask;

	if (flags)
		irq_none = false;
	else
		goto irq_rx;

	if (flags & FSL_SAI_CSR_WSF)
		dev_dbg(dev, "isr: Start of Tx word detected\n");

	if (flags & FSL_SAI_CSR_SEF)
		dev_dbg(dev, "isr: Tx Frame sync error detected\n");

	if (flags & FSL_SAI_CSR_FEF)
		dev_dbg(dev, "isr: Transmit underrun detected\n");

	if (flags & FSL_SAI_CSR_FWF)
		dev_dbg(dev, "isr: Enabled transmit FIFO is empty\n");

	if (flags & FSL_SAI_CSR_FRF)
		dev_dbg(dev, "isr: Transmit FIFO watermark has been reached\n");

	flags &= FSL_SAI_CSR_xF_W_MASK;
	xcsr &= ~FSL_SAI_CSR_xF_MASK;

	if (flags)
		regmap_write(sai->regmap, FSL_SAI_TCSR(ofs), flags | xcsr);

irq_rx:
	/* Rx IRQ */
	regmap_read(sai->regmap, FSL_SAI_RCSR(ofs), &xcsr);
	flags = xcsr & mask;

	if (flags)
		irq_none = false;
	else
		goto out;

	if (flags & FSL_SAI_CSR_WSF)
		dev_dbg(dev, "isr: Start of Rx word detected\n");

	if (flags & FSL_SAI_CSR_SEF)
		dev_dbg(dev, "isr: Rx Frame sync error detected\n");

	if (flags & FSL_SAI_CSR_FEF)
		dev_dbg(dev, "isr: Receive overflow detected\n");

	if (flags & FSL_SAI_CSR_FWF)
		dev_dbg(dev, "isr: Enabled receive FIFO is full\n");

	if (flags & FSL_SAI_CSR_FRF)
		dev_dbg(dev, "isr: Receive FIFO watermark has been reached\n");

	flags &= FSL_SAI_CSR_xF_W_MASK;
	xcsr &= ~FSL_SAI_CSR_xF_MASK;

	if (flags)
		regmap_write(sai->regmap, FSL_SAI_RCSR(ofs), flags | xcsr);

out:
	if (irq_none)
		return IRQ_NONE;
	else
		return IRQ_HANDLED;
}

static int fsl_sai_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai, u32 tx_mask,
				u32 rx_mask, int slots, int slot_width)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(cpu_dai);

	sai->slots = slots;
	sai->slot_width = slot_width;

	return 0;
}

static int fsl_sai_set_dai_bclk_ratio(struct snd_soc_dai *dai,
				      unsigned int ratio)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(dai);

	sai->bclk_ratio = ratio;

	return 0;
}

static int fsl_sai_set_dai_sysclk_tr(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int fsl_dir)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int ofs = sai->soc_data->reg_offset;

	bool tx = fsl_dir == FSL_FMT_TRANSMITTER;
	u32 val_cr2 = 0;

	switch (clk_id) {
	case FSL_SAI_CLK_BUS:
		val_cr2 |= FSL_SAI_CR2_MSEL_BUS;
		break;
	case FSL_SAI_CLK_MAST1:
		val_cr2 |= FSL_SAI_CR2_MSEL_MCLK1;
		break;
	case FSL_SAI_CLK_MAST2:
		val_cr2 |= FSL_SAI_CR2_MSEL_MCLK2;
		break;
	case FSL_SAI_CLK_MAST3:
		val_cr2 |= FSL_SAI_CR2_MSEL_MCLK3;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(sai->regmap, FSL_SAI_xCR2(tx, ofs),
			   FSL_SAI_CR2_MSEL_MASK, val_cr2);

	return 0;
}

static int fsl_sai_set_mclk_rate(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(dai);
	struct clk *p = sai->mclk_clk[clk_id], *pll = 0, *npll = 0;
	u64 ratio = freq;
	int ret;

	while (p && sai->pll8k_clk && sai->pll11k_clk) {
		struct clk *pp = clk_get_parent(p);

		if (clk_is_match(pp, sai->pll8k_clk) ||
		    clk_is_match(pp, sai->pll11k_clk)) {
			pll = pp;
			break;
		}
		p = pp;
	}

	if (pll) {
		npll = (do_div(ratio, 8000) ? sai->pll11k_clk : sai->pll8k_clk);
		if (!clk_is_match(pll, npll)) {
			ret = clk_set_parent(p, npll);
			if (ret < 0)
				dev_warn(dai->dev,
					 "failed to set parent %s: %d\n",
					 __clk_get_name(npll), ret);
		}
	}

	ret = clk_set_rate(sai->mclk_clk[clk_id], freq);
	if (ret < 0)
		dev_err(dai->dev, "failed to set clock rate (%u): %d\n",
			freq, ret);

	return ret;
}

static int fsl_sai_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(cpu_dai);
	int ret;

	if (dir == SND_SOC_CLOCK_IN)
		return 0;

	if (freq > 0 && clk_id != FSL_SAI_CLK_BUS) {
		if (clk_id < 0 || clk_id >= FSL_SAI_MCLK_MAX) {
			dev_err(cpu_dai->dev, "Unknown clock id: %d\n", clk_id);
			return -EINVAL;
		}

		if (IS_ERR_OR_NULL(sai->mclk_clk[clk_id])) {
			dev_err(cpu_dai->dev, "Unassigned clock: %d\n", clk_id);
			return -EINVAL;
		}

		if (sai->mclk_streams == 0) {
			ret = fsl_sai_set_mclk_rate(cpu_dai, clk_id, freq);
			if (ret < 0)
				return ret;
		}
	}

	ret = fsl_sai_set_dai_sysclk_tr(cpu_dai, clk_id, freq,
					FSL_FMT_TRANSMITTER);
	if (ret) {
		dev_err(cpu_dai->dev, "Cannot set tx sysclk: %d\n", ret);
		return ret;
	}

	ret = fsl_sai_set_dai_sysclk_tr(cpu_dai, clk_id, freq,
					FSL_FMT_RECEIVER);
	if (ret)
		dev_err(cpu_dai->dev, "Cannot set rx sysclk: %d\n", ret);

	return ret;
}

static int fsl_sai_set_dai_fmt_tr(struct snd_soc_dai *cpu_dai,
				unsigned int fmt, int fsl_dir)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int ofs = sai->soc_data->reg_offset;
	bool tx = fsl_dir == FSL_FMT_TRANSMITTER;
	u32 val_cr2 = 0, val_cr4 = 0;

	if (!sai->is_lsb_first)
		val_cr4 |= FSL_SAI_CR4_MF;

	sai->is_dsp_mode = false;
	/* DAI mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/*
		 * Frame low, 1clk before data, one word length for frame sync,
		 * frame sync starts one serial clock cycle earlier,
		 * that is, together with the last bit of the previous
		 * data word.
		 */
		val_cr2 |= FSL_SAI_CR2_BCP;
		val_cr4 |= FSL_SAI_CR4_FSE | FSL_SAI_CR4_FSP;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/*
		 * Frame high, one word length for frame sync,
		 * frame sync asserts with the first bit of the frame.
		 */
		val_cr2 |= FSL_SAI_CR2_BCP;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/*
		 * Frame high, 1clk before data, one bit for frame sync,
		 * frame sync starts one serial clock cycle earlier,
		 * that is, together with the last bit of the previous
		 * data word.
		 */
		val_cr2 |= FSL_SAI_CR2_BCP;
		val_cr4 |= FSL_SAI_CR4_FSE;
		sai->is_dsp_mode = true;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/*
		 * Frame high, one bit for frame sync,
		 * frame sync asserts with the first bit of the frame.
		 */
		val_cr2 |= FSL_SAI_CR2_BCP;
		sai->is_dsp_mode = true;
		break;
	case SND_SOC_DAIFMT_PDM:
		val_cr2 |= FSL_SAI_CR2_BCP;
		val_cr4 &= ~FSL_SAI_CR4_MF;
		sai->is_dsp_mode = true;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		/* To be done */
	default:
		return -EINVAL;
	}

	/* DAI clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		/* Invert both clocks */
		val_cr2 ^= FSL_SAI_CR2_BCP;
		val_cr4 ^= FSL_SAI_CR4_FSP;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		/* Invert bit clock */
		val_cr2 ^= FSL_SAI_CR2_BCP;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		/* Invert frame clock */
		val_cr4 ^= FSL_SAI_CR4_FSP;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		/* Nothing to do for both normal cases */
		break;
	default:
		return -EINVAL;
	}

	sai->slave_mode[tx] = false;

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		val_cr2 |= FSL_SAI_CR2_BCD_MSTR;
		val_cr4 |= FSL_SAI_CR4_FSD_MSTR;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		sai->slave_mode[tx] = true;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		val_cr2 |= FSL_SAI_CR2_BCD_MSTR;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		val_cr4 |= FSL_SAI_CR4_FSD_MSTR;
		sai->slave_mode[tx] = true;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(sai->regmap, FSL_SAI_xCR2(tx, ofs),
			   FSL_SAI_CR2_BCP | FSL_SAI_CR2_BCD_MSTR, val_cr2);
	regmap_update_bits(sai->regmap, FSL_SAI_xCR4(tx, ofs),
			   FSL_SAI_CR4_MF | FSL_SAI_CR4_FSE |
			   FSL_SAI_CR4_FSP | FSL_SAI_CR4_FSD_MSTR, val_cr4);

	return 0;
}

static int fsl_sai_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(cpu_dai);
	int ret;

	if (sai->masterflag[FSL_FMT_TRANSMITTER])
		fmt = (fmt & (~SND_SOC_DAIFMT_MASTER_MASK)) |
				sai->masterflag[FSL_FMT_TRANSMITTER];

	ret = fsl_sai_set_dai_fmt_tr(cpu_dai, fmt, FSL_FMT_TRANSMITTER);
	if (ret) {
		dev_err(cpu_dai->dev, "Cannot set tx format: %d\n", ret);
		return ret;
	}

	if (sai->masterflag[FSL_FMT_RECEIVER])
		fmt = (fmt & (~SND_SOC_DAIFMT_MASTER_MASK)) |
				sai->masterflag[FSL_FMT_RECEIVER];

	ret = fsl_sai_set_dai_fmt_tr(cpu_dai, fmt, FSL_FMT_RECEIVER);
	if (ret)
		dev_err(cpu_dai->dev, "Cannot set rx format: %d\n", ret);

	return ret;
}

static int fsl_sai_set_bclk(struct snd_soc_dai *dai, bool tx, u32 freq)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(dai);
	unsigned int ofs = sai->soc_data->reg_offset;
	unsigned long clk_rate;
	int adir = tx ? RX : TX;
	int dir = tx ? TX : RX;
	unsigned int reg = 0;
	u32 ratio, savesub = freq, saveratio = 0, savediv = 0;
	u32 id;
	int ret = 0;

	/* Don't apply to slave mode */
	if (sai->slave_mode[tx])
		return 0;

	/*
	 * There is no point in polling MCLK0 if it is identical to MCLK1.
	 * And given that MQS use case has to use MCLK1 though two clocks
	 * are the same, we simply skip MCLK0 and start to find from MCLK1.
	 */
	id = sai->soc_data->mclk0_is_mclk1 ? 1 : 0;

	for (; id < FSL_SAI_MCLK_MAX; id++) {
		clk_rate = clk_get_rate(sai->mclk_clk[id]);
		if (!clk_rate)
			continue;

		ratio = clk_rate / freq;

		ret = clk_rate - ratio * freq;

		/*
		 * Drop the source that can not be
		 * divided into the required rate.
		 */
		if (ret != 0 && clk_rate / ret < 1000)
			continue;

		dev_dbg(dai->dev,
			"ratio %d for freq %dHz based on clock %ldHz\n",
			ratio, freq, clk_rate);

		if ((ratio % 2 == 0 && ratio >= 2 && ratio <= 512) ||
		    (ratio == 1 && sai->verid.major >= 3 && sai->verid.minor >= 1)) {

			if (ret < savesub) {
				saveratio = ratio;
				sai->mclk_id[tx] = id;
				savesub = ret;
			}

			if (ret == 0)
				break;
		}
	}

	if (saveratio == 0) {
		dev_err(dai->dev, "failed to derive required %cx rate: %d\n",
				tx ? 'T' : 'R', freq);
		return -EINVAL;
	}

	/*
	 * 1) For Asynchronous mode, we must set RCR2 register for capture, and
	 *    set TCR2 register for playback.
	 * 2) For Tx sync with Rx clock, we must set RCR2 register for playback
	 *    and capture.
	 * 3) For Rx sync with Tx clock, we must set TCR2 register for playback
	 *    and capture.
	 * 4) For Tx and Rx are both Synchronous with another SAI, we just
	 *    ignore it.
	 */
	if (fsl_sai_dir_is_synced(sai, adir))
		reg = FSL_SAI_xCR2(!tx, ofs);
	else if (!sai->synchronous[dir])
		reg = FSL_SAI_xCR2(tx, ofs);

	if (reg) {
		regmap_update_bits(sai->regmap, reg, FSL_SAI_CR2_MSEL_MASK,
			   FSL_SAI_CR2_MSEL(sai->mclk_id[tx]));

		savediv = (saveratio == 1 ? 0 : (saveratio >> 1) - 1);
		regmap_update_bits(sai->regmap, reg, FSL_SAI_CR2_DIV_MASK, savediv);

		if (sai->verid.major >= 3 && sai->verid.minor >= 1) {
			regmap_update_bits(sai->regmap, reg, FSL_SAI_CR2_BYP,
				   (saveratio == 1 ? FSL_SAI_CR2_BYP : 0));
		}
	}

	if (sai->soc_data->max_register >= FSL_SAI_MCTL) {
		/* SAI is in master mode at this point, so enable MCLK */
		regmap_update_bits(sai->regmap, FSL_SAI_MCTL,
				FSL_SAI_MCTL_MCLK_EN, FSL_SAI_MCTL_MCLK_EN);
	}

	dev_dbg(dai->dev, "best fit: clock id=%d, ratio=%d, deviation=%d\n",
			sai->mclk_id[tx], saveratio, savesub);

	return 0;
}

static int fsl_sai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *cpu_dai)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int ofs = sai->soc_data->reg_offset;
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int channels = params_channels(params);
	u32 word_width = params_width(params);
	u32 rate = params_rate(params);
	u32 val_cr4 = 0, val_cr5 = 0;
	u32 slots = (channels == 1) ? 2 : channels;
	u32 slot_width = word_width;
	int adir = tx ? RX : TX;
	u32 pins, bclk;
	int ret, i, trce_mask = 0, dl_cfg_cnt, dl_cfg_idx = 0;
	struct fsl_sai_dl_cfg *dl_cfg;

	if (sai->slot_width)
		slot_width = sai->slot_width;

	if (sai->slots)
		slots = sai->slots;
	else if (sai->bclk_ratio)
		slots = sai->bclk_ratio / slot_width;

	pins = DIV_ROUND_UP(channels, slots);
	sai->is_dsd = fsl_is_dsd(params);
	if (sai->is_dsd) {
		pins = channels;
		dl_cfg = sai->dsd_dl_cfg;
		dl_cfg_cnt = sai->dsd_dl_cfg_cnt;
	} else {
		dl_cfg = sai->pcm_dl_cfg;
		dl_cfg_cnt = sai->pcm_dl_cfg_cnt;
	}

	for (i = 0; i < dl_cfg_cnt; i++) {
		if (dl_cfg[i].pins == pins) {
			dl_cfg_idx = i;
			break;
		}
	}

	if (dl_cfg_idx >= dl_cfg_cnt) {
		dev_err(cpu_dai->dev, "fsl,dataline%s invalid or not provided.\n",
			sai->is_dsd ? ",dsd" : "");
		return -EINVAL;
	}

	bclk = rate*(sai->bclk_ratio ? sai->bclk_ratio : slots * slot_width);

	if (!IS_ERR_OR_NULL(sai->pinctrl)) {
		sai->pins_state = fsl_get_pins_state(sai->pinctrl, params, bclk);

		if (!IS_ERR_OR_NULL(sai->pins_state)) {
			ret = pinctrl_select_state(sai->pinctrl, sai->pins_state);
			if (ret) {
				dev_err(cpu_dai->dev,
					"failed to set proper pins state: %d\n", ret);
				return ret;
			}
		}
	}

	if (!sai->slave_mode[tx]) {
		ret = fsl_sai_set_bclk(cpu_dai, tx, bclk);
		if (ret)
			return ret;

		/* Do not enable the clock if it is already enabled */
		if (!(sai->mclk_streams & BIT(substream->stream))) {
			ret = clk_prepare_enable(sai->mclk_clk[sai->mclk_id[tx]]);
			if (ret)
				return ret;

			sai->mclk_streams |= BIT(substream->stream);
		}
	}

	if (!sai->is_dsp_mode)
		val_cr4 |= FSL_SAI_CR4_SYWD(slot_width);

	val_cr5 |= FSL_SAI_CR5_WNW(slot_width);
	val_cr5 |= FSL_SAI_CR5_W0W(slot_width);

	if (sai->is_lsb_first || sai->is_dsd)
		val_cr5 |= FSL_SAI_CR5_FBT(0);
	else
		val_cr5 |= FSL_SAI_CR5_FBT(word_width - 1);

	val_cr4 |= FSL_SAI_CR4_FRSZ(slots);

	/* Set to output mode to avoid tri-stated data pins */
	if (tx)
		val_cr4 |= FSL_SAI_CR4_CHMOD;

	/*
	 * For SAI master mode, when Tx(Rx) sync with Rx(Tx) clock, Rx(Tx) will
	 * generate bclk and frame clock for Tx(Rx), we should set RCR4(TCR4),
	 * RCR5(TCR5) for playback(capture), or there will be sync error.
	 */

	if (!sai->slave_mode[tx] && fsl_sai_dir_is_synced(sai, adir)) {
		regmap_update_bits(sai->regmap, FSL_SAI_xCR4(!tx, ofs),
				   FSL_SAI_CR4_SYWD_MASK | FSL_SAI_CR4_FRSZ_MASK |
				   FSL_SAI_CR4_CHMOD_MASK,
				   val_cr4);
		regmap_update_bits(sai->regmap, FSL_SAI_xCR5(!tx, ofs),
				   FSL_SAI_CR5_WNW_MASK | FSL_SAI_CR5_W0W_MASK |
				   FSL_SAI_CR5_FBT_MASK, val_cr5);
	}

	if (__sw_hweight8(dl_cfg[dl_cfg_idx].mask[tx]) <= 1 || sai->is_multi_lane)
		regmap_update_bits(sai->regmap, FSL_SAI_xCR4(tx, ofs),
				   FSL_SAI_CR4_FCOMB_MASK, 0);
	else
		regmap_update_bits(sai->regmap, FSL_SAI_xCR4(tx, ofs),
				   FSL_SAI_CR4_FCOMB_MASK, FSL_SAI_CR4_FCOMB_SOFT);

	if (tx)
		sai->dma_params_tx.addr = sai->res->start + FSL_SAI_TDR0 +
					  dl_cfg[dl_cfg_idx].start_off[tx] * 0x4;
	else
		sai->dma_params_rx.addr = sai->res->start + FSL_SAI_RDR0 +
					  dl_cfg[dl_cfg_idx].start_off[tx] * 0x4;

	if (sai->is_multi_lane) {
		if (tx) {
			sai->audio_config[tx].words_per_fifo = min(slots, channels);
			sai->audio_config[tx].dst_fifo_num = pins;
			sai->audio_config[tx].dst_fifo_off = dl_cfg[dl_cfg_idx].next_off[tx];
			sai->dma_params_tx.maxburst = sai->audio_config[tx].words_per_fifo * pins;
			sai->dma_params_tx.peripheral_config = &sai->audio_config[tx];
			sai->dma_params_tx.peripheral_size = sizeof(sai->audio_config[tx]);

			regmap_update_bits(sai->regmap, FSL_SAI_TCR1(ofs),
					   FSL_SAI_CR1_RFW_MASK(sai->soc_data->fifo_depth),
					   sai->soc_data->fifo_depth - sai->dma_params_tx.maxburst);
		} else {
			sai->audio_config[tx].words_per_fifo = min(slots, channels);
			sai->audio_config[tx].src_fifo_num = pins;
			sai->audio_config[tx].src_fifo_off = dl_cfg[dl_cfg_idx].next_off[tx];
			sai->dma_params_rx.maxburst = sai->audio_config[tx].words_per_fifo * pins;
			sai->dma_params_rx.peripheral_config = &sai->audio_config[tx];
			sai->dma_params_rx.peripheral_size = sizeof(sai->audio_config[tx]);

			regmap_update_bits(sai->regmap, FSL_SAI_RCR1(ofs),
					   FSL_SAI_CR1_RFW_MASK(sai->soc_data->fifo_depth),
					   sai->dma_params_rx.maxburst - 1);
		}
	}

	snd_soc_dai_init_dma_data(cpu_dai, &sai->dma_params_tx, &sai->dma_params_rx);

	if (__sw_hweight8(dl_cfg[dl_cfg_idx].mask[tx] & 0xFF) < pins) {
		dev_err(cpu_dai->dev, "channel not supported\n");
		return -EINVAL;
	}

	/*find a proper tcre setting*/
	for (i = 0; i < 8; i++) {
		trce_mask = (1 << (i + 1)) - 1;
		if (__sw_hweight8(dl_cfg[dl_cfg_idx].mask[tx] & trce_mask) == pins)
			break;
	}

	regmap_update_bits(sai->regmap, FSL_SAI_xCR3(tx, ofs),
		   FSL_SAI_CR3_TRCE_MASK,
		   FSL_SAI_CR3_TRCE((dl_cfg[dl_cfg_idx].mask[tx] & trce_mask)));

	regmap_update_bits(sai->regmap, FSL_SAI_xCR4(tx, ofs),
			   FSL_SAI_CR4_SYWD_MASK | FSL_SAI_CR4_FRSZ_MASK |
			   FSL_SAI_CR4_CHMOD_MASK,
			   val_cr4);
	regmap_update_bits(sai->regmap, FSL_SAI_xCR5(tx, ofs),
			   FSL_SAI_CR5_WNW_MASK | FSL_SAI_CR5_W0W_MASK |
			   FSL_SAI_CR5_FBT_MASK, val_cr5);
	regmap_write(sai->regmap, FSL_SAI_xMR(tx),
		     ~0UL - ((1 << min(channels, slots)) - 1));

	return 0;
}

static int fsl_sai_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(cpu_dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int ofs = sai->soc_data->reg_offset;

	regmap_update_bits(sai->regmap, FSL_SAI_xCR3(tx, ofs),
			   FSL_SAI_CR3_TRCE_MASK, 0);

	if (!sai->slave_mode[tx] &&
			sai->mclk_streams & BIT(substream->stream)) {
		clk_disable_unprepare(sai->mclk_clk[sai->mclk_id[tx]]);
		sai->mclk_streams &= ~BIT(substream->stream);
	}

	return 0;
}

static void fsl_sai_config_disable(struct fsl_sai *sai, int dir)
{
	unsigned int ofs = sai->soc_data->reg_offset;
	bool tx = dir == TX;
	u32 xcsr, count = 100;

	regmap_update_bits(sai->regmap, FSL_SAI_xCSR(tx, ofs),
			   FSL_SAI_CSR_TERE, 0);

	/* TERE will remain set till the end of current frame */
	do {
		udelay(10);
		regmap_read(sai->regmap, FSL_SAI_xCSR(tx, ofs), &xcsr);
	} while (--count && xcsr & FSL_SAI_CSR_TERE);

	regmap_update_bits(sai->regmap, FSL_SAI_xCSR(tx, ofs),
			   FSL_SAI_CSR_FR, FSL_SAI_CSR_FR);

	/*
	 * For sai master mode, after several open/close sai,
	 * there will be no frame clock, and can't recover
	 * anymore. Add software reset to fix this issue.
	 * This is a hardware bug, and will be fix in the
	 * next sai version.
	 */
	if (!sai->slave_mode[tx]) {
		/* Software Reset */
		regmap_write(sai->regmap, FSL_SAI_xCSR(tx, ofs), FSL_SAI_CSR_SR);
		/* Clear SR bit to finish the reset */
		regmap_write(sai->regmap, FSL_SAI_xCSR(tx, ofs), 0);
	}
}

static int fsl_sai_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *cpu_dai)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(cpu_dai);
	unsigned int ofs = sai->soc_data->reg_offset;
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	int adir = tx ? RX : TX;
	int dir = tx ? TX : RX;
	u32 xcsr;

	/*
	 * Asynchronous mode: Clear SYNC for both Tx and Rx.
	 * Rx sync with Tx clocks: Clear SYNC for Tx, set it for Rx.
	 * Tx sync with Rx clocks: Clear SYNC for Rx, set it for Tx.
	 */
	regmap_update_bits(sai->regmap, FSL_SAI_TCR2(ofs), FSL_SAI_CR2_SYNC,
			   sai->synchronous[TX] ? FSL_SAI_CR2_SYNC : 0);
	regmap_update_bits(sai->regmap, FSL_SAI_RCR2(ofs), FSL_SAI_CR2_SYNC,
			   sai->synchronous[RX] ? FSL_SAI_CR2_SYNC : 0);

	/*
	 * It is recommended that the transmitter is the last enabled
	 * and the first disabled.
	 */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		regmap_update_bits(sai->regmap, FSL_SAI_xCSR(tx, ofs),
				   FSL_SAI_CSR_FRDE, FSL_SAI_CSR_FRDE);
		regmap_update_bits(sai->regmap, FSL_SAI_xCSR(tx, ofs),
				   FSL_SAI_CSR_TERE, FSL_SAI_CSR_TERE);
		regmap_update_bits(sai->regmap, FSL_SAI_xCSR(tx, ofs),
				   FSL_SAI_CSR_SE, FSL_SAI_CSR_SE);
		/*
		 * Enable the opposite direction for synchronous mode
		 * 1. Tx sync with Rx: only set RE for Rx; set TE & RE for Tx
		 * 2. Rx sync with Tx: only set TE for Tx; set RE & TE for Rx
		 *
		 * RM recommends to enable RE after TE for case 1 and to enable
		 * TE after RE for case 2, but we here may not always guarantee
		 * that happens: "arecord 1.wav; aplay 2.wav" in case 1 enables
		 * TE after RE, which is against what RM recommends but should
		 * be safe to do, judging by years of testing results.
		 */
		if (fsl_sai_dir_is_synced(sai, adir))
			regmap_update_bits(sai->regmap, FSL_SAI_xCSR((!tx), ofs),
					   FSL_SAI_CSR_TERE, FSL_SAI_CSR_TERE);

		regmap_update_bits(sai->regmap, FSL_SAI_xCSR(tx, ofs),
				   FSL_SAI_CSR_xIE_MASK, FSL_SAI_FLAGS);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		regmap_update_bits(sai->regmap, FSL_SAI_xCSR(tx, ofs),
				   FSL_SAI_CSR_FRDE, 0);
		regmap_update_bits(sai->regmap, FSL_SAI_xCSR(tx, ofs),
				   FSL_SAI_CSR_xIE_MASK, 0);

		/* Check if the opposite FRDE is also disabled */
		regmap_read(sai->regmap, FSL_SAI_xCSR(!tx, ofs), &xcsr);

		/*
		 * If opposite stream provides clocks for synchronous mode and
		 * it is inactive, disable it before disabling the current one
		 */
		if (fsl_sai_dir_is_synced(sai, adir) && !(xcsr & FSL_SAI_CSR_FRDE))
			fsl_sai_config_disable(sai, adir);

		/*
		 * Disable current stream if either of:
		 * 1. current stream doesn't provide clocks for synchronous mode
		 * 2. current stream provides clocks for synchronous mode but no
		 *    more stream is active.
		 */
		if (!fsl_sai_dir_is_synced(sai, dir) || !(xcsr & FSL_SAI_CSR_FRDE))
			fsl_sai_config_disable(sai, dir);

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fsl_sai_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	struct fsl_sai *sai = snd_soc_dai_get_drvdata(cpu_dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	int ret, i, j, k = 0;
	u64 clk_rate[2];

	/*
	 * EDMA controller needs period size to be a multiple of
	 * tx/rx maxburst
	 */
	if (sai->soc_data->use_edma)
		snd_pcm_hw_constraint_step(substream->runtime, 0,
					   SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					   tx ? sai->dma_params_tx.maxburst :
					   sai->dma_params_rx.maxburst);

	sai->constraint_rates = fsl_sai_rate_constraints;
	if (sai->pll8k_clk || sai->pll11k_clk) {
		sai->constraint_rates.list = sai->constraint_rates_list;
		sai->constraint_rates.count = 0;
		for (i = 0; i < FAL_SAI_NUM_RATES; i++) {
			clk_rate[0] = clk_get_rate(sai->pll8k_clk);
			clk_rate[1] = clk_get_rate(sai->pll11k_clk);
			for (j = 0; j < 2; j++) {
				if (clk_rate[j] != 0 &&
				    do_div(clk_rate[j], fsl_sai_rates[i]) == 0) {
					sai->constraint_rates_list[k++] = fsl_sai_rates[i];
					sai->constraint_rates.count++;
				}
			}
		}

		/* protection for if there is no proper rate found*/
		if (!sai->constraint_rates.count)
			sai->constraint_rates = fsl_sai_rate_constraints;
	}
	ret = snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &sai->constraint_rates);

	return ret;
}

static const struct snd_soc_dai_ops fsl_sai_pcm_dai_ops = {
	.set_bclk_ratio	= fsl_sai_set_dai_bclk_ratio,
	.set_sysclk	= fsl_sai_set_dai_sysclk,
	.set_fmt	= fsl_sai_set_dai_fmt,
	.set_tdm_slot	= fsl_sai_set_dai_tdm_slot,
	.hw_params	= fsl_sai_hw_params,
	.hw_free	= fsl_sai_hw_free,
	.trigger	= fsl_sai_trigger,
	.startup	= fsl_sai_startup,
};

static int fsl_sai_dai_probe(struct snd_soc_dai *cpu_dai)
{
	struct fsl_sai *sai = dev_get_drvdata(cpu_dai->dev);
	unsigned int ofs = sai->soc_data->reg_offset;

	regmap_update_bits(sai->regmap, FSL_SAI_TCR1(ofs),
			   FSL_SAI_CR1_RFW_MASK(sai->soc_data->fifo_depth),
			   sai->soc_data->fifo_depth - sai->dma_params_tx.maxburst);
	regmap_update_bits(sai->regmap, FSL_SAI_RCR1(ofs),
			   FSL_SAI_CR1_RFW_MASK(sai->soc_data->fifo_depth),
			   sai->dma_params_rx.maxburst - 1);

	snd_soc_dai_init_dma_data(cpu_dai, &sai->dma_params_tx,
				&sai->dma_params_rx);

	return 0;
}

static int fsl_sai_dai_resume(struct snd_soc_component *component)
{
	struct fsl_sai *sai = snd_soc_component_get_drvdata(component);
	int ret;

	if (!IS_ERR_OR_NULL(sai->pinctrl) && !IS_ERR_OR_NULL(sai->pins_state)) {
		ret = pinctrl_select_state(sai->pinctrl, sai->pins_state);
		if (ret) {
			dev_err(&sai->pdev->dev,
				"failed to set proper pins state: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static struct snd_soc_dai_driver fsl_sai_dai_template = {
	.probe = fsl_sai_dai_probe,
	.playback = {
		.stream_name = "CPU-Playback",
		.channels_min = 1,
		.channels_max = 32,
		.rate_min = 8000,
		.rate_max = 2822400,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = FSL_SAI_FORMATS,
	},
	.capture = {
		.stream_name = "CPU-Capture",
		.channels_min = 1,
		.channels_max = 32,
		.rate_min = 8000,
		.rate_max = 2822400,
		.rates = SNDRV_PCM_RATE_KNOT,
		.formats = FSL_SAI_FORMATS,
	},
	.ops = &fsl_sai_pcm_dai_ops,
};

static const struct snd_soc_component_driver fsl_component = {
	.name           = "fsl-sai",
	.resume  = fsl_sai_dai_resume,
};

static struct reg_default fsl_sai_reg_defaults_ofs0[] = {
	{FSL_SAI_TCR1(0), 0},
	{FSL_SAI_TCR2(0), 0},
	{FSL_SAI_TCR3(0), 0},
	{FSL_SAI_TCR4(0), 0},
	{FSL_SAI_TCR5(0), 0},
	{FSL_SAI_TDR0, 0},
	{FSL_SAI_TDR1, 0},
	{FSL_SAI_TDR2, 0},
	{FSL_SAI_TDR3, 0},
	{FSL_SAI_TDR4, 0},
	{FSL_SAI_TDR5, 0},
	{FSL_SAI_TDR6, 0},
	{FSL_SAI_TDR7, 0},
	{FSL_SAI_TMR, 0},
	{FSL_SAI_RCR1(0), 0},
	{FSL_SAI_RCR2(0), 0},
	{FSL_SAI_RCR3(0), 0},
	{FSL_SAI_RCR4(0), 0},
	{FSL_SAI_RCR5(0), 0},
	{FSL_SAI_RMR, 0},
};

static struct reg_default fsl_sai_reg_defaults_ofs8[] = {
	{FSL_SAI_TCR1(8), 0},
	{FSL_SAI_TCR2(8), 0},
	{FSL_SAI_TCR3(8), 0},
	{FSL_SAI_TCR4(8), 0},
	{FSL_SAI_TCR5(8), 0},
	{FSL_SAI_TDR0, 0},
	{FSL_SAI_TDR1, 0},
	{FSL_SAI_TDR2, 0},
	{FSL_SAI_TDR3, 0},
	{FSL_SAI_TDR4, 0},
	{FSL_SAI_TDR5, 0},
	{FSL_SAI_TDR6, 0},
	{FSL_SAI_TDR7, 0},
	{FSL_SAI_TMR, 0},
	{FSL_SAI_RCR1(8), 0},
	{FSL_SAI_RCR2(8), 0},
	{FSL_SAI_RCR3(8), 0},
	{FSL_SAI_RCR4(8), 0},
	{FSL_SAI_RCR5(8), 0},
	{FSL_SAI_RMR, 0},
	{FSL_SAI_MCTL, 0},
	{FSL_SAI_MDIV, 0},
};

static bool fsl_sai_readable_reg(struct device *dev, unsigned int reg)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int ofs = sai->soc_data->reg_offset;

	if (reg >= FSL_SAI_TCSR(ofs) && reg <= FSL_SAI_TCR5(ofs))
		return true;

	if (reg >= FSL_SAI_RCSR(ofs) && reg <= FSL_SAI_RCR5(ofs))
		return true;

	switch (reg) {
	case FSL_SAI_TFR0:
	case FSL_SAI_TFR1:
	case FSL_SAI_TFR2:
	case FSL_SAI_TFR3:
	case FSL_SAI_TFR4:
	case FSL_SAI_TFR5:
	case FSL_SAI_TFR6:
	case FSL_SAI_TFR7:
	case FSL_SAI_TMR:
	case FSL_SAI_RDR0:
	case FSL_SAI_RDR1:
	case FSL_SAI_RDR2:
	case FSL_SAI_RDR3:
	case FSL_SAI_RDR4:
	case FSL_SAI_RDR5:
	case FSL_SAI_RDR6:
	case FSL_SAI_RDR7:
	case FSL_SAI_RFR0:
	case FSL_SAI_RFR1:
	case FSL_SAI_RFR2:
	case FSL_SAI_RFR3:
	case FSL_SAI_RFR4:
	case FSL_SAI_RFR5:
	case FSL_SAI_RFR6:
	case FSL_SAI_RFR7:
	case FSL_SAI_RMR:
	case FSL_SAI_MCTL:
	case FSL_SAI_MDIV:
	case FSL_SAI_VERID:
	case FSL_SAI_PARAM:
	case FSL_SAI_TTCTN:
	case FSL_SAI_RTCTN:
	case FSL_SAI_TTCTL:
	case FSL_SAI_TBCTN:
	case FSL_SAI_TTCAP:
	case FSL_SAI_RTCTL:
	case FSL_SAI_RBCTN:
	case FSL_SAI_RTCAP:
		return true;
	default:
		return false;
	}
}

static bool fsl_sai_volatile_reg(struct device *dev, unsigned int reg)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int ofs = sai->soc_data->reg_offset;

	if (reg == FSL_SAI_TCSR(ofs) || reg == FSL_SAI_RCSR(ofs))
		return true;

	/* Set VERID and PARAM be volatile for reading value in probe */
	if (ofs == 8 && (reg == FSL_SAI_VERID || reg == FSL_SAI_PARAM))
		return true;

	switch (reg) {
	case FSL_SAI_TFR0:
	case FSL_SAI_TFR1:
	case FSL_SAI_TFR2:
	case FSL_SAI_TFR3:
	case FSL_SAI_TFR4:
	case FSL_SAI_TFR5:
	case FSL_SAI_TFR6:
	case FSL_SAI_TFR7:
	case FSL_SAI_RFR0:
	case FSL_SAI_RFR1:
	case FSL_SAI_RFR2:
	case FSL_SAI_RFR3:
	case FSL_SAI_RFR4:
	case FSL_SAI_RFR5:
	case FSL_SAI_RFR6:
	case FSL_SAI_RFR7:
	case FSL_SAI_RDR0:
	case FSL_SAI_RDR1:
	case FSL_SAI_RDR2:
	case FSL_SAI_RDR3:
	case FSL_SAI_RDR4:
	case FSL_SAI_RDR5:
	case FSL_SAI_RDR6:
	case FSL_SAI_RDR7:
	case FSL_SAI_TTCTN:
	case FSL_SAI_TTCTL:
	case FSL_SAI_TBCTN:
	case FSL_SAI_TTCAP:
	case FSL_SAI_RTCTN:
	case FSL_SAI_RTCTL:
	case FSL_SAI_RBCTN:
	case FSL_SAI_RTCAP:
		return true;
	default:
		return false;
	}
}

static bool fsl_sai_writeable_reg(struct device *dev, unsigned int reg)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int ofs = sai->soc_data->reg_offset;

	if (reg >= FSL_SAI_TCSR(ofs) && reg <= FSL_SAI_TCR5(ofs))
		return true;

	if (reg >= FSL_SAI_RCSR(ofs) && reg <= FSL_SAI_RCR5(ofs))
		return true;

	switch (reg) {
	case FSL_SAI_TDR0:
	case FSL_SAI_TDR1:
	case FSL_SAI_TDR2:
	case FSL_SAI_TDR3:
	case FSL_SAI_TDR4:
	case FSL_SAI_TDR5:
	case FSL_SAI_TDR6:
	case FSL_SAI_TDR7:
	case FSL_SAI_TMR:
	case FSL_SAI_RMR:
	case FSL_SAI_MCTL:
	case FSL_SAI_MDIV:
	case FSL_SAI_TTCTL:
	case FSL_SAI_RTCTL:
		return true;
	default:
		return false;
	}
}

static struct regmap_config fsl_sai_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.fast_io = true,

	.max_register = FSL_SAI_RMR,
	.reg_defaults = fsl_sai_reg_defaults_ofs0,
	.num_reg_defaults = ARRAY_SIZE(fsl_sai_reg_defaults_ofs0),
	.readable_reg = fsl_sai_readable_reg,
	.volatile_reg = fsl_sai_volatile_reg,
	.writeable_reg = fsl_sai_writeable_reg,
	.cache_type = REGCACHE_FLAT,
};

static int fsl_sai_check_version(struct device *dev)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned char ofs = sai->soc_data->reg_offset;
	unsigned int val;
	int ret;

	if (FSL_SAI_TCSR(ofs) == FSL_SAI_VERID)
		return 0;

	ret = regmap_read(sai->regmap, FSL_SAI_VERID, &val);
	if (ret < 0)
		return ret;

	dev_dbg(dev, "VERID: 0x%016X\n", val);

	sai->verid.major = (val & FSL_SAI_VERID_MAJOR_MASK) >>
			   FSL_SAI_VERID_MAJOR_SHIFT;
	sai->verid.minor = (val & FSL_SAI_VERID_MINOR_MASK) >>
			   FSL_SAI_VERID_MINOR_SHIFT;
	sai->verid.feature = val & FSL_SAI_VERID_FEATURE_MASK;

	ret = regmap_read(sai->regmap, FSL_SAI_PARAM, &val);
	if (ret < 0)
		return ret;

	dev_dbg(dev, "PARAM: 0x%016X\n", val);

	/* Max slots per frame, power of 2 */
	sai->param.slot_num = 1 <<
		((val & FSL_SAI_PARAM_SPF_MASK) >> FSL_SAI_PARAM_SPF_SHIFT);

	/* Words per fifo, power of 2 */
	sai->param.fifo_depth = 1 <<
		((val & FSL_SAI_PARAM_WPF_MASK) >> FSL_SAI_PARAM_WPF_SHIFT);

	/* Number of datalines implemented */
	sai->param.dataline = val & FSL_SAI_PARAM_DLN_MASK;

	return 0;
}

static int fsl_sai_runtime_suspend(struct device *dev);
static int fsl_sai_runtime_resume(struct device *dev);

static unsigned int fsl_sai_calc_dl_off(unsigned long dl_mask)
{
	int fbidx, nbidx, offset;

	fbidx = find_first_bit(&dl_mask, 8);
	nbidx = find_next_bit(&dl_mask, 8, fbidx + 1);
	offset = nbidx - fbidx - 1;

	return (offset < 0 || offset >= 7 ? 0 : offset);
}

static int fsl_sai_read_dlcfg(struct platform_device *pdev, char *pn,
	struct fsl_sai_dl_cfg **rcfg, unsigned int soc_dl)
{
	int ret, elems, i, index, num_cfg;
	struct device_node *np = pdev->dev.of_node;
	struct fsl_sai_dl_cfg *cfg;
	unsigned long dl_mask;
	u32 rx, tx, pins;

	*rcfg = NULL;

	elems = of_property_count_u32_elems(np, pn);

	/* consider default value "0 0x1 0x1" if property is missing */
	if (elems <= 0)
		elems = 3;

	if (elems % 3) {
		dev_err(&pdev->dev,
			"Number of elements in %s must be divisible to 3.\n", pn);
		return -EINVAL;
	}

	num_cfg = elems / 3;
	cfg = devm_kzalloc(&pdev->dev, num_cfg * sizeof(*cfg), GFP_KERNEL);
	if (cfg == NULL) {
		dev_err(&pdev->dev, "Cannot allocate memory for %s.\n", pn);
		return -ENOMEM;
	}

	for (i = 0, index = 0; i < num_cfg; i++) {
		ret = of_property_read_u32_index(np, pn, index++, &pins);
		if (ret)
			pins = 0;

		ret = of_property_read_u32_index(np, pn, index++, &rx);
		if (ret)
			rx = 1;

		ret = of_property_read_u32_index(np, pn, index++, &tx);
		if (ret)
			tx = 1;

		if ((rx & ~soc_dl) || (tx & ~soc_dl)) {
			dev_err(&pdev->dev,
				"%s: dataline cfg[%d] setting error, mask is 0x%x\n",
				 pn, i, soc_dl);
			return -EINVAL;
		}

		cfg[i].pins = pins;
		cfg[i].mask[0] = rx;
		dl_mask = rx;
		cfg[i].start_off[0] = find_first_bit(&dl_mask, 8);
		cfg[i].next_off[0] = fsl_sai_calc_dl_off(rx);
		cfg[i].mask[1] = tx;
		dl_mask = tx;
		cfg[i].start_off[1] = find_first_bit(&dl_mask, 8);
		cfg[i].next_off[1] = fsl_sai_calc_dl_off(tx);
	}

	*rcfg = cfg;
	return num_cfg;
}

static int fsl_sai_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_sai *sai;
	struct regmap *gpr;
	void __iomem *base;
	char tmp[8];
	int irq, ret, i;
	int index;

	sai = devm_kzalloc(&pdev->dev, sizeof(*sai), GFP_KERNEL);
	if (!sai)
		return -ENOMEM;

	sai->pdev = pdev;
	sai->soc_data = of_device_get_match_data(&pdev->dev);

	sai->is_lsb_first = of_property_read_bool(np, "lsb-first");

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &sai->res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	if (sai->soc_data->reg_offset == 8) {
		fsl_sai_regmap_config.reg_defaults = fsl_sai_reg_defaults_ofs8;
		fsl_sai_regmap_config.max_register = FSL_SAI_MDIV;
		fsl_sai_regmap_config.num_reg_defaults =
			ARRAY_SIZE(fsl_sai_reg_defaults_ofs8);
	}

	sai->regmap = devm_regmap_init_mmio(&pdev->dev, base, &fsl_sai_regmap_config);
	if (IS_ERR(sai->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		return PTR_ERR(sai->regmap);
	}

	sai->bus_clk = devm_clk_get(&pdev->dev, "bus");
	/* Compatible with old DTB cases */
	if (IS_ERR(sai->bus_clk) && PTR_ERR(sai->bus_clk) != -EPROBE_DEFER)
		sai->bus_clk = devm_clk_get(&pdev->dev, "sai");
	if (IS_ERR(sai->bus_clk)) {
		dev_err(&pdev->dev, "failed to get bus clock: %ld\n",
				PTR_ERR(sai->bus_clk));
		/* -EPROBE_DEFER */
		return PTR_ERR(sai->bus_clk);
	}

	for (i = 1; i < FSL_SAI_MCLK_MAX; i++) {
		sprintf(tmp, "mclk%d", i);
		sai->mclk_clk[i] = devm_clk_get(&pdev->dev, tmp);
		if (IS_ERR(sai->mclk_clk[i])) {
			dev_err(&pdev->dev, "failed to get mclk%d clock: %ld\n",
					i, PTR_ERR(sai->mclk_clk[i]));
			sai->mclk_clk[i] = NULL;
		}
	}

	if (sai->soc_data->mclk0_is_mclk1)
		sai->mclk_clk[0] = sai->mclk_clk[1];
	else
		sai->mclk_clk[0] = sai->bus_clk;

	sai->pll8k_clk = devm_clk_get(&pdev->dev, "pll8k");
	if (IS_ERR(sai->pll8k_clk))
		sai->pll8k_clk = NULL;

	sai->pll11k_clk = devm_clk_get(&pdev->dev, "pll11k");
	if (IS_ERR(sai->pll11k_clk))
		sai->pll11k_clk = NULL;

	if (of_find_property(np, "fsl,sai-multi-lane", NULL))
		sai->is_multi_lane = true;

	/*dataline mask for rx and tx*/
	ret = fsl_sai_read_dlcfg(pdev, "fsl,dataline", &sai->pcm_dl_cfg,
					sai->soc_data->dataline);
	if (ret < 0)
		return ret;

	sai->pcm_dl_cfg_cnt = ret;

	ret = fsl_sai_read_dlcfg(pdev, "fsl,dataline,dsd", &sai->dsd_dl_cfg,
					sai->soc_data->dataline);
	if (ret < 0)
		return ret;

	sai->dsd_dl_cfg_cnt = ret;

	if ((of_find_property(np, "fsl,i2s-xtor", NULL) != NULL) ||
	    (of_find_property(np, "fsl,txm-rxs", NULL) != NULL))
	{
		sai->masterflag[FSL_FMT_TRANSMITTER] = SND_SOC_DAIFMT_CBS_CFS;
		sai->masterflag[FSL_FMT_RECEIVER] = SND_SOC_DAIFMT_CBM_CFM;
	} else {
		if (!of_property_read_u32(np, "fsl,txmasterflag",
			&sai->masterflag[FSL_FMT_TRANSMITTER]))
			sai->masterflag[FSL_FMT_TRANSMITTER] <<= 12;
		if (!of_property_read_u32(np, "fsl,rxmasterflag",
			&sai->masterflag[FSL_FMT_RECEIVER]))
			sai->masterflag[FSL_FMT_RECEIVER] <<= 12;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(&pdev->dev, irq, fsl_sai_isr, IRQF_SHARED,
			       np->name, sai);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim irq %u\n", irq);
		return ret;
	}

	memcpy(&sai->cpu_dai_drv, &fsl_sai_dai_template,
	       sizeof(fsl_sai_dai_template));

	/* Sync Tx with Rx as default by following old DT binding */
	sai->synchronous[RX] = true;
	sai->synchronous[TX] = false;
	sai->cpu_dai_drv.symmetric_rate = 1;
	sai->cpu_dai_drv.symmetric_channels = 1;
	sai->cpu_dai_drv.symmetric_sample_bits = 1;

	if (of_find_property(np, "fsl,sai-synchronous-rx", NULL) &&
	    of_find_property(np, "fsl,sai-asynchronous", NULL)) {
		/* error out if both synchronous and asynchronous are present */
		dev_err(&pdev->dev, "invalid binding for synchronous mode\n");
		return -EINVAL;
	}

	if (of_find_property(np, "fsl,sai-synchronous-rx", NULL)) {
		/* Sync Rx with Tx */
		sai->synchronous[RX] = false;
		sai->synchronous[TX] = true;
	} else if (of_find_property(np, "fsl,sai-asynchronous", NULL)) {
		/* Discard all settings for asynchronous mode */
		sai->synchronous[RX] = false;
		sai->synchronous[TX] = false;
		sai->cpu_dai_drv.symmetric_rate = 0;
		sai->cpu_dai_drv.symmetric_channels = 0;
		sai->cpu_dai_drv.symmetric_sample_bits = 0;
	}

	if (of_find_property(np, "fsl,sai-mclk-direction-output", NULL) &&
	    of_device_is_compatible(np, "fsl,imx6ul-sai")) {
		gpr = syscon_regmap_lookup_by_compatible("fsl,imx6ul-iomuxc-gpr");
		if (IS_ERR(gpr)) {
			dev_err(&pdev->dev, "cannot find iomuxc registers\n");
			return PTR_ERR(gpr);
		}

		index = of_alias_get_id(np, "sai");
		if (index < 0)
			return index;

		regmap_update_bits(gpr, IOMUXC_GPR1, MCLK_DIR(index),
				   MCLK_DIR(index));
	}

	sai->dma_params_rx.addr = sai->res->start + FSL_SAI_RDR0;
	sai->dma_params_tx.addr = sai->res->start + FSL_SAI_TDR0;
	sai->dma_params_rx.maxburst =
		sai->soc_data->max_burst[RX] ? sai->soc_data->max_burst[RX] : FSL_SAI_MAXBURST_RX;
	sai->dma_params_tx.maxburst =
		sai->soc_data->max_burst[TX] ? sai->soc_data->max_burst[TX] : FSL_SAI_MAXBURST_TX;

	sai->pinctrl = devm_pinctrl_get(&pdev->dev);

	platform_set_drvdata(pdev, sai);
	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = fsl_sai_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&pdev->dev);
		goto err_pm_get_sync;
	}

	/* Get sai version */
	ret = fsl_sai_check_version(&pdev->dev);
	if (ret < 0)
		dev_warn(&pdev->dev, "Error reading SAI version: %d\n", ret);

	/* Select MCLK direction */
	if (of_find_property(np, "fsl,sai-mclk-direction-output", NULL) &&
	    sai->soc_data->max_register >= FSL_SAI_MCTL) {
		regmap_update_bits(sai->regmap, FSL_SAI_MCTL,
				   FSL_SAI_MCTL_MCLK_EN, FSL_SAI_MCTL_MCLK_EN);
	}

	ret = pm_runtime_put_sync(&pdev->dev);
	if (ret < 0)
		goto err_pm_get_sync;

	if (sai->verid.feature & FSL_SAI_VERID_TSTMP_EN) {
		if (of_find_property(np, "fsl,sai-monitor-spdif", NULL) &&
		    of_device_is_compatible(np, "fsl,imx8mm-sai")) {
			sai->regmap_gpr = syscon_regmap_lookup_by_compatible("fsl,imx8mm-iomuxc-gpr");
			if (IS_ERR(sai->regmap_gpr))
				dev_warn(&pdev->dev, "cannot find iomuxc registers\n");

			sai->gpr_idx = of_alias_get_id(np, "sai");
			if (sai->gpr_idx < 0)
				dev_warn(&pdev->dev, "cannot find sai alias id\n");

			if (sai->gpr_idx > 0 && !IS_ERR(sai->regmap_gpr))
				sai->monitor_spdif = true;
		}

		if(sai->monitor_spdif) {
			ret = sysfs_create_group(&pdev->dev.kobj,
					 fsl_sai_get_dev_attribute_group(sai->monitor_spdif));
			if (ret) {
				dev_err(&pdev->dev, "fail to create sys group\n");
				goto err_pm_disable;
			}
		}
	}

	/*
	 * Register platform component before registering cpu dai for there
	 * is not defer probe for platform component in snd_soc_add_pcm_runtime().
	 */
	if (sai->soc_data->use_imx_pcm) {
		ret = imx_pcm_dma_init(pdev, IMX_SAI_DMABUF_SIZE);
		if (ret)
			goto err_component_register;
	} else {
		ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
		if (ret)
			goto err_component_register;
	}

	ret = devm_snd_soc_register_component(&pdev->dev, &fsl_component,
					      &sai->cpu_dai_drv, 1);
	if (ret)
		goto err_component_register;

	return ret;

err_component_register:
	if (sai->verid.feature & FSL_SAI_VERID_TSTMP_EN)
		sysfs_remove_group(&pdev->dev.kobj,
				   fsl_sai_get_dev_attribute_group(sai->monitor_spdif));
err_pm_get_sync:
	if (!pm_runtime_status_suspended(&pdev->dev))
		fsl_sai_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int fsl_sai_remove(struct platform_device *pdev)
{
	struct fsl_sai *sai = dev_get_drvdata(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		fsl_sai_runtime_suspend(&pdev->dev);

	if (sai->verid.feature & FSL_SAI_VERID_TSTMP_EN)
		sysfs_remove_group(&pdev->dev.kobj,  fsl_sai_get_dev_attribute_group(sai->monitor_spdif));

	return 0;
}

static const struct fsl_sai_soc_data fsl_sai_vf610_data = {
	.use_imx_pcm = false,
	.use_edma = false,
	.fifo_depth = 32,
	.reg_offset = 0,
	.mclk0_is_mclk1 = false,
	.dataline = 0x1,
	.fifos = 1,
	.flags = 0,
	.max_register = FSL_SAI_RMR,
};

static const struct fsl_sai_soc_data fsl_sai_imx6sx_data = {
	.use_imx_pcm = true,
	.use_edma = false,
	.fifo_depth = 32,
	.reg_offset = 0,
	.mclk0_is_mclk1 = true,
	.dataline = 0x1,
	.fifos = 1,
	.flags = 0,
	.max_register = FSL_SAI_RMR,
};

static const struct fsl_sai_soc_data fsl_sai_imx7ulp_data = {
	.use_imx_pcm = true,
	.use_edma = false,
	.fifo_depth = 16,
	.reg_offset = 8,
	.mclk0_is_mclk1 = false,
	.flags = PMQOS_CPU_LATENCY,
	.dataline = 0x3,
	.fifos = 2,
	.max_register = FSL_SAI_RMR,
};

static const struct fsl_sai_soc_data fsl_sai_imx8mq_data = {
	.use_imx_pcm = true,
	.use_edma = false,
	.fifo_depth = 128,
	.reg_offset = 8,
	.mclk0_is_mclk1 = false,
	.dataline = 0xff,
	.fifos = 8,
	.flags = 0,
	.max_register = FSL_SAI_RMR,
};

static const struct fsl_sai_soc_data fsl_sai_imx8qm_data = {
	.use_imx_pcm = true,
	.use_edma = true,
	.fifo_depth = 64,
	.reg_offset = 0,
	.mclk0_is_mclk1 = false,
	.dataline = 0xf,
	.fifos = 1,
	.flags = 0,
	.max_register = FSL_SAI_RMR,
};

static const struct fsl_sai_soc_data fsl_sai_imx8mm_data = {
	.use_imx_pcm = true,
	.use_edma = false,
	.fifo_depth = 128,
	.reg_offset = 8,
	.mclk0_is_mclk1 = false,
	.dataline = 0xff,
	.fifos = 8,
	.flags = 0,
	.max_register = FSL_SAI_MCTL,
};

static const struct fsl_sai_soc_data fsl_sai_imx8mp_data = {
	.use_imx_pcm = true,
	.use_edma = false,
	.fifo_depth = 128,
	.reg_offset = 8,
	.mclk0_is_mclk1 = false,
	.dataline = 0xff,
	.fifos = 8,
	.flags = 0,
	.max_register = FSL_SAI_MDIV,
};

static const struct fsl_sai_soc_data fsl_sai_imx8ulp_data = {
	.use_imx_pcm = true,
	.use_edma = true,
	.fifo_depth = 16,
	.reg_offset = 8,
	.mclk0_is_mclk1 = false,
	.dataline = 0xf,
	.fifos = 4,
	.flags = PMQOS_CPU_LATENCY,
	.max_register = FSL_SAI_RTCAP,
};

static const struct fsl_sai_soc_data fsl_sai_imx93_data = {
	.use_imx_pcm = true,
	.use_edma = true,
	.fifo_depth = 128,
	.reg_offset = 8,
	.mclk0_is_mclk1 = false,
	.dataline = 0xf,
	.fifos = 4,
	.flags = 0,
	.max_register = FSL_SAI_MCTL,
	.max_burst = {8, 8},
};

static const struct of_device_id fsl_sai_ids[] = {
	{ .compatible = "fsl,vf610-sai", .data = &fsl_sai_vf610_data },
	{ .compatible = "fsl,imx6sx-sai", .data = &fsl_sai_imx6sx_data },
	{ .compatible = "fsl,imx6ul-sai", .data = &fsl_sai_imx6sx_data },
	{ .compatible = "fsl,imx7ulp-sai", .data = &fsl_sai_imx7ulp_data },
	{ .compatible = "fsl,imx8mq-sai", .data = &fsl_sai_imx8mq_data },
	{ .compatible = "fsl,imx8mm-sai", .data = &fsl_sai_imx8mm_data },
	{ .compatible = "fsl,imx8mp-sai", .data = &fsl_sai_imx8mp_data },
	{ .compatible = "fsl,imx8qm-sai", .data = &fsl_sai_imx8qm_data },
	{ .compatible = "fsl,imx8ulp-sai", .data = &fsl_sai_imx8ulp_data },
	{ .compatible = "fsl,imx8mn-sai", .data = &fsl_sai_imx8mp_data },
	{ .compatible = "fsl,imx93-sai", .data = &fsl_sai_imx93_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_sai_ids);

static int fsl_sai_runtime_suspend(struct device *dev)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);

	release_bus_freq(BUS_FREQ_AUDIO);

	if (sai->mclk_streams & BIT(SNDRV_PCM_STREAM_CAPTURE))
		clk_disable_unprepare(sai->mclk_clk[sai->mclk_id[0]]);

	if (sai->mclk_streams & BIT(SNDRV_PCM_STREAM_PLAYBACK))
		clk_disable_unprepare(sai->mclk_clk[sai->mclk_id[1]]);

	clk_disable_unprepare(sai->bus_clk);

	if (sai->soc_data->flags & PMQOS_CPU_LATENCY)
		cpu_latency_qos_remove_request(&sai->pm_qos_req);

	regcache_cache_only(sai->regmap, true);

	return 0;
}

static int fsl_sai_runtime_resume(struct device *dev)
{
	struct fsl_sai *sai = dev_get_drvdata(dev);
	unsigned int ofs = sai->soc_data->reg_offset;
	int ret;

	ret = clk_prepare_enable(sai->bus_clk);
	if (ret) {
		dev_err(dev, "failed to enable bus clock: %d\n", ret);
		return ret;
	}

	if (sai->mclk_streams & BIT(SNDRV_PCM_STREAM_PLAYBACK)) {
		ret = clk_prepare_enable(sai->mclk_clk[sai->mclk_id[1]]);
		if (ret)
			goto disable_bus_clk;
	}

	if (sai->mclk_streams & BIT(SNDRV_PCM_STREAM_CAPTURE)) {
		ret = clk_prepare_enable(sai->mclk_clk[sai->mclk_id[0]]);
		if (ret)
			goto disable_tx_clk;
	}

	if (sai->soc_data->flags & PMQOS_CPU_LATENCY)
		cpu_latency_qos_add_request(&sai->pm_qos_req, 0);

	request_bus_freq(BUS_FREQ_AUDIO);

	regcache_cache_only(sai->regmap, false);
	regcache_mark_dirty(sai->regmap);
	regmap_write(sai->regmap, FSL_SAI_TCSR(ofs), FSL_SAI_CSR_SR);
	regmap_write(sai->regmap, FSL_SAI_RCSR(ofs), FSL_SAI_CSR_SR);
	usleep_range(1000, 2000);
	regmap_write(sai->regmap, FSL_SAI_TCSR(ofs), 0);
	regmap_write(sai->regmap, FSL_SAI_RCSR(ofs), 0);

	ret = regcache_sync(sai->regmap);
	if (ret)
		goto disable_rx_clk;

	return 0;

disable_rx_clk:
	if (sai->mclk_streams & BIT(SNDRV_PCM_STREAM_CAPTURE))
		clk_disable_unprepare(sai->mclk_clk[sai->mclk_id[0]]);
disable_tx_clk:
	if (sai->mclk_streams & BIT(SNDRV_PCM_STREAM_PLAYBACK))
		clk_disable_unprepare(sai->mclk_clk[sai->mclk_id[1]]);
disable_bus_clk:
	clk_disable_unprepare(sai->bus_clk);

	return ret;
}

static const struct dev_pm_ops fsl_sai_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_sai_runtime_suspend,
			   fsl_sai_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static struct platform_driver fsl_sai_driver = {
	.probe = fsl_sai_probe,
	.remove = fsl_sai_remove,
	.driver = {
		.name = "fsl-sai",
		.pm = &fsl_sai_pm_ops,
		.of_match_table = fsl_sai_ids,
	},
};
module_platform_driver(fsl_sai_driver);

MODULE_DESCRIPTION("Freescale Soc SAI Interface");
MODULE_AUTHOR("Xiubo Li, <Li.Xiubo@freescale.com>");
MODULE_ALIAS("platform:fsl-sai");
MODULE_LICENSE("GPL");
