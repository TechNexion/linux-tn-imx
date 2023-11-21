// SPDX-License-Identifier: GPL-2.0
// Copyright 2018 NXP

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/core.h>

#include "fsl_micfil.h"
#include "imx-pcm.h"

#define MICFIL_NUM_RATES  7
#define FSL_MICFIL_RATES		SNDRV_PCM_RATE_8000_48000
#define FSL_MICFIL_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE)

struct fsl_micfil {
	struct platform_device *pdev;
	struct regmap *regmap;
	const struct fsl_micfil_soc_data *soc;
	struct clk *busclk;
	struct clk *mclk;
	struct clk *clk_src[MICFIL_CLK_SRC_NUM];
	struct snd_dmaengine_dai_dma_data dma_params_rx;
	struct kobject *hwvad_kobject;
	struct sdma_audio_config audio_config;
	struct snd_pcm_hw_constraint_list constraint_rates;
	unsigned int constraint_rates_list[MICFIL_NUM_RATES];
	unsigned int vad_channel;
	unsigned int dataline;
	char name[32];
	int irq[MICFIL_IRQ_LINES];
	unsigned int mclk_streams;
	int quality;	/*QUALITY 2-0 bits */
	bool slave_mode;
	int channel_gain[8];
	int clk_src_id;
	int dc_remover;
	int vad_sound_gain;
	int vad_noise_gain;
	int vad_input_gain;
	int vad_frame_time;
	int vad_init_time;
	int vad_init_mode;
	int vad_nfil_adjust;
	int vad_hpf;
	int vad_zcd_th;
	int vad_zcd_auto;
	int vad_zcd_en;
	int vad_zcd_adj;
	int vad_rate_index;
	atomic_t recording_state;
	atomic_t hwvad_state;
	/* spinlock to control HWVAD enable/disable */
	spinlock_t hwvad_lock;
};

struct fsl_micfil_soc_data {
	unsigned int fifos;
	unsigned int fifo_depth;
	unsigned int dataline;
	bool imx;
	bool use_edma;
	u64  formats;
};

static char *envp[] = {
	"EVENT=PDM_VOICE_DETECT",
	NULL,
};

static struct fsl_micfil_soc_data fsl_micfil_imx8mm = {
	.imx = true,
	.fifos = 8,
	.fifo_depth = 8,
	.dataline =  0xf,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
};

static struct fsl_micfil_soc_data fsl_micfil_imx8mp = {
	.imx = true,
	.fifos = 8,
	.fifo_depth = 32,
	.dataline =  0xf,
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
};

static struct fsl_micfil_soc_data fsl_micfil_imx93 = {
	.imx = true,
	.use_edma = true,
	.fifos = 8,
	.fifo_depth = 32,
	.dataline =  0xf,
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
};

static const struct of_device_id fsl_micfil_dt_ids[] = {
	{ .compatible = "fsl,imx8mm-micfil", .data = &fsl_micfil_imx8mm },
	{ .compatible = "fsl,imx8mp-micfil", .data = &fsl_micfil_imx8mp },
	{ .compatible = "fsl,imx93-micfil", .data = &fsl_micfil_imx93 },
	{}
};
MODULE_DEVICE_TABLE(of, fsl_micfil_dt_ids);

/* Table 5. Quality Modes
 * Medium	0 0 0
 * High		0 0 1
 * Very Low 2	1 0 0
 * Very Low 1	1 0 1
 * Very Low 0	1 1 0
 * Low		1 1 1
 */
static const char * const micfil_quality_select_texts[] = {
	"Medium", "High",
	"N/A", "N/A",
	"VLow2", "VLow1",
	"VLow0", "Low",
};

static const char * const micfil_hwvad_init_mode[] = {
	"Envelope mode", "Energy mode",
};

static const char * const micfil_hwvad_hpf_texts[] = {
	"Filter bypass",
	"Cut-off @1750Hz",
	"Cut-off @215Hz",
	"Cut-off @102Hz",
};

static const char * const micfil_hwvad_zcd_enable[] = {
	"OFF", "ON",
};

static const char * const micfil_hwvad_zcdauto_enable[] = {
	"OFF", "ON",
};

static const char * const micfil_hwvad_noise_decimation[] = {
	"Disabled", "Enabled",
};

/* when adding new rate text, also add it to the
 * micfil_hwvad_rate_ints
 */
static const char * const micfil_hwvad_rate[] = {
	"48KHz", "44.1KHz",
};

static const int micfil_hwvad_rate_ints[] = {
	48000, 44100,
};

static const char * const micfil_clk_src_texts[] = {
	"Auto", "AudioPLL1", "AudioPLL2", "ExtClk3",
};

/* DC Remover Control
 * Filter Bypassed	1 1
 * Cut-off @21Hz	0 0
 * Cut-off @83Hz	0 1
 * Cut-off @152HZ	1 0
 */
static const char * const micfil_dc_remover_texts[] = {
	"Cut-off @21Hz", "Cut-off @83Hz",
	"Cut-off @152Hz", "Bypass",
};

static const struct soc_enum fsl_micfil_quality_enum =
	SOC_ENUM_SINGLE(REG_MICFIL_CTRL2,
			MICFIL_CTRL2_QSEL_SHIFT,
			ARRAY_SIZE(micfil_quality_select_texts),
			micfil_quality_select_texts);
static const struct soc_enum fsl_micfil_hwvad_init_mode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(micfil_hwvad_init_mode),
			    micfil_hwvad_init_mode);
static const struct soc_enum fsl_micfil_hwvad_hpf_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(micfil_hwvad_hpf_texts),
			    micfil_hwvad_hpf_texts);
static const struct soc_enum fsl_micfil_hwvad_zcd_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(micfil_hwvad_zcd_enable),
			    micfil_hwvad_zcd_enable);
static const struct soc_enum fsl_micfil_hwvad_zcdauto_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(micfil_hwvad_zcdauto_enable),
			    micfil_hwvad_zcdauto_enable);
static const struct soc_enum fsl_micfil_hwvad_ndec_enum =
	SOC_ENUM_SINGLE(REG_MICFIL_VAD0_NCONFIG,
			MICFIL_VAD0_NCONFIG_NOREN_SHIFT,
			ARRAY_SIZE(micfil_hwvad_noise_decimation),
			micfil_hwvad_noise_decimation);
static const struct soc_enum fsl_micfil_hwvad_rate_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(micfil_hwvad_rate),
			    micfil_hwvad_rate);
static const struct soc_enum fsl_micfil_clk_src_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(micfil_clk_src_texts),
			    micfil_clk_src_texts);
static const struct soc_enum fsl_micfil_dc_remover_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(micfil_dc_remover_texts),
			    micfil_dc_remover_texts);

static int micfil_put_clk_src(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);
	int val = snd_soc_enum_item_to_val(e, item[0]);

	micfil->clk_src_id = val;

	return 0;
}

static int micfil_get_clk_src(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->clk_src_id;

	return 0;
}

static int micfil_put_dc_remover_state(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);
	unsigned int *item = ucontrol->value.enumerated.item;
	int val = snd_soc_enum_item_to_val(e, item[0]);
	int i = 0, ret = 0;
	u32 reg_val = 0;

	if (val < 0 || val > 3)
		return -EINVAL;

	micfil->dc_remover = val;

	/* Calculate total value for all channels */
	for (i = 0; i < 8; i++)
		reg_val |= MICFIL_DC_MODE(val, i);

	/* Update DC Remover mode for all channels */
	ret = snd_soc_component_update_bits(comp, REG_MICFIL_DC_CTRL,
					    MICFIL_DC_CTRL_MASK, reg_val);
	if (ret < 0)
		return ret;

	return 0;
}

static int micfil_get_dc_remover_state(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->dc_remover;

	return 0;
}

static int hwvad_put_init_mode(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);
	int val = snd_soc_enum_item_to_val(e, item[0]);

	/* 0 - Envelope-based Mode
	 * 1 - Energy-based Mode
	 */
	micfil->vad_init_mode = val;
	return 0;
}

static int hwvad_get_init_mode(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_init_mode;

	return 0;
}

static int hwvad_put_hpf(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);
	int val = snd_soc_enum_item_to_val(e, item[0]);

	/* 00 - HPF Bypass
	 * 01 - Cut-off frequency 1750Hz
	 * 10 - Cut-off frequency 215Hz
	 * 11 - Cut-off frequency 102Hz
	 */
	micfil->vad_hpf = val;

	return 0;
}

static int hwvad_get_hpf(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_hpf;

	return 0;
}

static int hwvad_put_zcd_en(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);
	int val = snd_soc_enum_item_to_val(e, item[0]);

	micfil->vad_zcd_en = val;

	return 0;
}

static int hwvad_get_zcd_en(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_zcd_en;

	return 0;
}

static int hwvad_put_rate(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);
	int val = snd_soc_enum_item_to_val(e, item[0]);

	micfil->vad_rate_index = val;

	return 0;
}

static int hwvad_get_rate(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_rate_index;

	return 0;
}

static int hwvad_put_zcd_auto(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);
	int val = snd_soc_enum_item_to_val(e, item[0]);

	micfil->vad_zcd_auto = val;

	return 0;
}

static int hwvad_get_zcd_auto(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_zcd_auto;

	return 0;
}

static int gain_info(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xf;

	return 0;
}

static int hwvad_put_input_gain(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	micfil->vad_input_gain = ucontrol->value.integer.value[0];

	return 0;
}

static int hwvad_get_input_gain(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_input_gain;

	return 0;
}

static int hwvad_put_sound_gain(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	micfil->vad_sound_gain = ucontrol->value.integer.value[0];

	return 0;
}

static int hwvad_get_sound_gain(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_sound_gain;

	return 0;
}

static int hwvad_put_noise_gain(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	micfil->vad_noise_gain = ucontrol->value.integer.value[0];

	return 0;
}

static int hwvad_get_noise_gain(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_noise_gain;

	return 0;
}

static int hwvad_framet_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 1;
	uinfo->value.integer.max = 64;

	return 0;
}

static int hwvad_put_frame_time(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	micfil->vad_frame_time = ucontrol->value.integer.value[0];

	return 0;
}

static int hwvad_get_frame_time(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_frame_time;

	return 0;
}

static int hwvad_initt_info(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 1;
	uinfo->value.integer.max = 32;

	return 0;
}

static int hwvad_put_init_time(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	micfil->vad_init_time = ucontrol->value.integer.value[0];

	return 0;
}

static int hwvad_get_init_time(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_init_time;

	return 0;
}

static int hwvad_nfiladj_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 1;
	uinfo->value.integer.max = 32;

	return 0;
}

static int hwvad_put_nfil_adjust(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	micfil->vad_nfil_adjust = ucontrol->value.integer.value[0];

	return 0;
}

static int hwvad_get_nfil_adjust(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_nfil_adjust;

	return 0;
}

static int hwvad_zcdth_info(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 1;
	uinfo->value.integer.max = 1024;

	return 0;
}

static int hwvad_put_zcd_th(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	micfil->vad_zcd_th = ucontrol->value.integer.value[0];

	return 0;
}

static int hwvad_get_zcd_th(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_zcd_th;

	return 0;
}

static int hwvad_zcdadj_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 1;
	uinfo->value.integer.max = 16;

	return 0;
}

static int hwvad_put_zcd_adj(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	micfil->vad_zcd_adj = ucontrol->value.integer.value[0];

	return 0;
}

static int hwvad_get_zcd_adj(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_micfil *micfil = snd_soc_component_get_drvdata(comp);

	ucontrol->value.enumerated.item[0] = micfil->vad_zcd_adj;

	return 0;
}

static DECLARE_TLV_DB_SCALE(gain_tlv, 0, 100, 0);

static const struct snd_kcontrol_new fsl_micfil_snd_controls[] = {
	SOC_SINGLE_SX_TLV("CH0 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(0), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH1 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(1), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH2 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(2), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH3 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(3), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH4 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(4), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH5 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(5), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH6 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(6), 0x8, 0xF, gain_tlv),
	SOC_SINGLE_SX_TLV("CH7 Volume", REG_MICFIL_OUT_CTRL,
			  MICFIL_OUTGAIN_CHX_SHIFT(7), 0x8, 0xF, gain_tlv),
	SOC_ENUM_EXT("MICFIL Quality Select",
		     fsl_micfil_quality_enum,
		     snd_soc_get_enum_double, snd_soc_put_enum_double),
	SOC_ENUM_EXT("HWVAD Initialization Mode",
		     fsl_micfil_hwvad_init_mode_enum,
		     hwvad_get_init_mode, hwvad_put_init_mode),
	SOC_ENUM_EXT("HWVAD High-Pass Filter",
		     fsl_micfil_hwvad_hpf_enum,
		     hwvad_get_hpf, hwvad_put_hpf),
	SOC_ENUM_EXT("HWVAD Zero-Crossing Detector Enable",
		     fsl_micfil_hwvad_zcd_enum,
		     hwvad_get_zcd_en, hwvad_put_zcd_en),
	SOC_ENUM_EXT("HWVAD Zero-Crossing Detector Auto Threshold",
		     fsl_micfil_hwvad_zcdauto_enum,
		     hwvad_get_zcd_auto, hwvad_put_zcd_auto),
	SOC_ENUM_EXT("HWVAD Noise OR Enable",
		     fsl_micfil_hwvad_ndec_enum,
		     snd_soc_get_enum_double, snd_soc_put_enum_double),
	SOC_ENUM_EXT("HWVAD Sampling Rate",
		     fsl_micfil_hwvad_rate_enum,
		     hwvad_get_rate, hwvad_put_rate),
	SOC_ENUM_EXT("Clock Source",
		     fsl_micfil_clk_src_enum,
		     micfil_get_clk_src, micfil_put_clk_src),
	SOC_ENUM_EXT("MICFIL DC Remover Control", fsl_micfil_dc_remover_enum,
		     micfil_get_dc_remover_state, micfil_put_dc_remover_state),
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HWVAD Input Gain",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_WRITE,
		.info = gain_info,
		.get = hwvad_get_input_gain,
		.put = hwvad_put_input_gain,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HWVAD Sound Gain",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_WRITE,
		.info = gain_info,
		.get = hwvad_get_sound_gain,
		.put = hwvad_put_sound_gain,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HWVAD Noise Gain",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_WRITE,
		.info = gain_info,
		.get = hwvad_get_noise_gain,
		.put = hwvad_put_noise_gain,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HWVAD Detector Frame Time",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_WRITE,
		.info = hwvad_framet_info,
		.get = hwvad_get_frame_time,
		.put = hwvad_put_frame_time,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HWVAD Detector Initialization Time",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_WRITE,
		.info = hwvad_initt_info,
		.get = hwvad_get_init_time,
		.put = hwvad_put_init_time,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HWVAD Noise Filter Adjustment",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_WRITE,
		.info = hwvad_nfiladj_info,
		.get = hwvad_get_nfil_adjust,
		.put = hwvad_put_nfil_adjust,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HWVAD Zero-Crossing Detector Threshold",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_WRITE,
		.info = hwvad_zcdth_info,
		.get = hwvad_get_zcd_th,
		.put = hwvad_put_zcd_th,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HWVAD Zero-Crossing Detector Adjustment",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_WRITE,
		.info = hwvad_zcdadj_info,
		.get = hwvad_get_zcd_adj,
		.put = hwvad_put_zcd_adj,
	},

};

static int disable_hwvad(struct device *dev, bool sync);


static inline int get_pdm_clk(struct fsl_micfil *micfil,
			      unsigned int rate)
{
	u32 ctrl2_reg;
	int qsel, osr;
	int bclk;

	regmap_read(micfil->regmap, REG_MICFIL_CTRL2, &ctrl2_reg);
	osr = 16 - ((ctrl2_reg & MICFIL_CTRL2_CICOSR_MASK)
		    >> MICFIL_CTRL2_CICOSR_SHIFT);

	regmap_read(micfil->regmap, REG_MICFIL_CTRL2, &ctrl2_reg);
	qsel = ctrl2_reg & MICFIL_CTRL2_QSEL_MASK;

	switch (qsel) {
	case MICFIL_HIGH_QUALITY:
		bclk = rate * 8 * osr / 2; /* kfactor = 0.5 */
		break;
	case MICFIL_MEDIUM_QUALITY:
	case MICFIL_VLOW0_QUALITY:
		bclk = rate * 4 * osr * 1; /* kfactor = 1 */
		break;
	case MICFIL_LOW_QUALITY:
	case MICFIL_VLOW1_QUALITY:
		bclk = rate * 2 * osr * 2; /* kfactor = 2 */
		break;
	case MICFIL_VLOW2_QUALITY:
		bclk = rate * osr * 4; /* kfactor = 4 */
		break;
	default:
		dev_err(&micfil->pdev->dev,
			"Please make sure you select a valid quality.\n");
		bclk = -1;
		break;
	}

	return bclk;
}

static inline int get_clk_div(struct fsl_micfil *micfil,
			      unsigned int rate)
{
	u32 ctrl2_reg;
	long mclk_rate;
	int clk_div;

	regmap_read(micfil->regmap, REG_MICFIL_CTRL2, &ctrl2_reg);

	mclk_rate = clk_get_rate(micfil->mclk);

	clk_div = mclk_rate / (get_pdm_clk(micfil, rate) * 2);

	return clk_div;
}

/* The SRES is a self-negated bit which provides the CPU with the
 * capability to initialize the PDM Interface module through the
 * slave-bus interface. This bit always reads as zero, and this
 * bit is only effective when MDIS is cleared
 */
static int fsl_micfil_reset(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;

	ret = regmap_update_bits(micfil->regmap,
				 REG_MICFIL_CTRL1,
				 MICFIL_CTRL1_MDIS_MASK,
				 0);
	if (ret) {
		dev_err(dev, "failed to clear MDIS bit %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(micfil->regmap,
				 REG_MICFIL_CTRL1,
				 MICFIL_CTRL1_SRES_MASK,
				 MICFIL_CTRL1_SRES);
	if (ret) {
		dev_err(dev, "failed to reset MICFIL: %d\n", ret);
		return ret;
	}

	/* w1c */
	regmap_write_bits(micfil->regmap, REG_MICFIL_STAT, 0xFF, 0xFF);

	return 0;
}

/* enable/disable hwvad interrupts */
static int configure_hwvad_interrupts(struct device *dev,
				      int enable)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	u32 vadie_reg = enable ? MICFIL_VAD0_CTRL1_IE : 0;
	u32 vaderie_reg = enable ? MICFIL_VAD0_CTRL1_ERIE : 0;

	/* Voice Activity Detector Error Interruption Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_ERIE_MASK,
				 vaderie_reg);
	if (ret) {
		dev_err(dev,
			"Failed to set/clear VADERIE in CTRL1_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Voice Activity Detector Interruption Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_IE_MASK,
				 vadie_reg);
	if (ret) {
		dev_err(dev,
			"Failed to set/clear VADIE in CTRL1_VAD0 [%d]\n",
			ret);
		return ret;
	}

	return 0;
}

static int init_hwvad_internal_filters(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;

	/* Voice Activity Detector Internal Filters Initialization*/
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_ST10_MASK,
				 MICFIL_VAD0_CTRL1_ST10);
	if (ret) {
		dev_err(dev,
			"Failed to set VADST10 in CTRL1_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* sleep for 100ms - it should be enough for bit to stay
	 * pulsed for more than 2 cycles
	 */
	mdelay(MICFIL_SLEEP);

	/* Voice Activity Detector Enabled */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_ST10_MASK,
				 0);
	if (ret) {
		dev_err(dev,
			"Failed to clear VADST10 in CTRL1_VAD0 [%d]\n",
			ret);
		return ret;
	}
	return 0;
}

/* Zero-Crossing Detector Initialization
 * Optionally a Zero-Crossing Detection block (ZCD) could
 * be enabled to avoid low energy voiced speech be missed,
 * improving the voice detection performance.
 * See Section 8.4.3
 */
static int __maybe_unused init_zcd(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;

	/* exit if zcd is not enabled from userspace */
	if (!micfil->vad_zcd_en)
		return 0;

	if (micfil->vad_zcd_auto) {
		/* Zero-Crossing Detector Adjustment */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_ZCD,
					 MICFIL_VAD0_ZCD_ZCDADJ_MASK,
					 micfil->vad_zcd_adj);
		if (ret) {
			dev_err(dev,
				"Failed to set ZCDADJ in ZCD_VAD0 [%d]\n",
				ret);
			return ret;
		}
	}

	/* Zero-Crossing Detector Threshold */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_ZCD,
				 MICFIL_VAD0_ZCD_ZCDTH_MASK,
				 MICFIL_VAD0_ZCD_ZCDTH(micfil->vad_zcd_th));
	if (ret) {
		dev_err(dev, "Failed to set ZCDTH in ZCD_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Zero-Crossing Detector AND Behavior */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_ZCD,
				 MICFIL_VAD0_ZCD_ZCDAND_MASK,
				 MICFIL_HWVAD_ZCDAND);
	if (ret) {
		dev_err(dev, "Failed to set ZCDAND in ZCD_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Zero-Crossing Detector Automatic Threshold */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_ZCD,
				 MICFIL_VAD0_ZCD_ZCDAUT_MASK,
				 micfil->vad_zcd_auto);
	if (ret) {
		dev_err(dev,
			"Failed to set/clear ZCDAUT in ZCD_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Zero-Crossing Detector Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_ZCD,
				 MICFIL_VAD0_ZCD_ZCDEN_MASK,
				 MICFIL_VAD0_ZCD_ZCDEN);
	if (ret) {
		dev_err(dev, "Failed to set ZCDEN in ZCD_VAD0 [%d]\n", ret);
		return ret;
	}

	return 0;
}

/* Configuration done only in energy-based initialization mode */
static int init_hwvad_energy_mode(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret, i;
	u32 stat;
	u32 flag;

	dev_info(dev, "Energy-based mode initialization\n");

	/* Voice Activity Detector Reset */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_RST_SHIFT,
				 MICFIL_VAD0_CTRL1_RST);
	if (ret) {
		dev_err(dev, "Failed to set VADRST in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Voice Activity Detector Enabled */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_EN_MASK,
				 MICFIL_VAD0_CTRL1_EN);
	if (ret) {
		dev_err(dev, "Failed to set VADEN in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* it would be a good idea to wait some time before VADEN
	 * is set
	 */
	mdelay(5 * MICFIL_SLEEP);

	/* Enable Interrupts */
	ret = configure_hwvad_interrupts(dev, 1);

	/* Initialize Zero Crossing Detector */
	ret = init_zcd(dev);
	if (ret)
		return ret;

	/* Enable MICFIL module */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
				 MICFIL_CTRL1_PDMIEN_MASK,
				 MICFIL_CTRL1_PDMIEN);
	if (ret) {
		dev_err(dev, "failed to enable the module\n");
		return ret;
	}

	/* Wait for INITF to be asserted */
	for (i = 0; i < MICFIL_MAX_RETRY; i++) {
		ret = regmap_read(micfil->regmap, REG_MICFIL_VAD0_STAT, &stat);
		if (ret) {
			dev_err(dev, "failed to read register %d\n",
				REG_MICFIL_VAD0_STAT);
			return ret;
		}

		flag = (stat & MICFIL_VAD0_STAT_INITF_MASK);
		if (flag == 0)
			break;

		mdelay(MICFIL_SLEEP);
	}

	if (i == MICFIL_MAX_RETRY) {
		dev_err(dev, "initf not asserted. Failed to init hwvad\n");
		return -EBUSY;
	}

	/* Initialize Internal Filters */
	ret = init_hwvad_internal_filters(dev);
	if (ret)
		return ret;

	return ret;
}

/* Configuration done only in envelope-based initialization mode */
static int init_hwvad_envelope_mode(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret, i;
	u32 stat;
	u32 flag;

	/* Frame energy disable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL2,
				 MICFIL_VAD0_CTRL2_FRENDIS_MASK,
				 MICFIL_VAD0_CTRL2_FRENDIS);
	if (ret) {
		dev_err(dev, "Failed to set FRENDIS in CTRL2_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Enable pre-filter Noise & Signal */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL2,
				 MICFIL_VAD0_CTRL2_PREFEN_MASK,
				 MICFIL_VAD0_CTRL2_PREFEN);
	if (ret) {
		dev_err(dev, "Failed to set PREFEN in CTRL2_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Enable Signal Filter */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_SCONFIG,
				 MICFIL_VAD0_SCONFIG_SFILEN_MASK,
				 MICFIL_VAD0_SCONFIG_SFILEN);
	if (ret) {
		dev_err(dev,
			"Failed to set SFILEN in SCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Signal Maximum Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_SCONFIG,
				 MICFIL_VAD0_SCONFIG_SMAXEN_MASK,
				 MICFIL_VAD0_SCONFIG_SMAXEN);
	if (ret) {
		dev_err(dev,
			"Failed to set SMAXEN in SCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Allways enable noise filter, not based on voice activity
	 * information
	 */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_NCONFIG,
				 MICFIL_VAD0_NCONFIG_NFILAUT_MASK,
				 0);
	if (ret) {
		dev_err(dev,
			"Failed to set NFILAUT in NCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Noise Minimum Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_NCONFIG,
				 MICFIL_VAD0_NCONFIG_NMINEN_MASK,
				 MICFIL_VAD0_NCONFIG_NMINEN);
	if (ret) {
		dev_err(dev,
			"Failed to set NMINEN in NCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Noise Decimation Enable */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_NCONFIG,
				 MICFIL_VAD0_NCONFIG_NDECEN_MASK,
				 MICFIL_VAD0_NCONFIG_NDECEN);
	if (ret) {
		dev_err(dev,
			"Failed to set NDECEN in NCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* Voice Activity Detector Reset */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_RST_MASK,
				 MICFIL_VAD0_CTRL1_RST);
	if (ret) {
		dev_err(dev, "Failed to set VADRST in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Initialize Zero Crossing Detector */
	ret = init_zcd(dev);
	if (ret)
		return ret;

	/* Voice Activity Detector Enabled */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_EN_MASK,
				 MICFIL_VAD0_CTRL1_EN);
	if (ret) {
		dev_err(dev, "Failed to set VADEN in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* Enable MICFIL module */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
				 MICFIL_CTRL1_PDMIEN_MASK,
				 MICFIL_CTRL1_PDMIEN);
	if (ret) {
		dev_err(dev, "failed to enable the module\n");
		return ret;
	}

	/* it would be a good idea to wait some time before VADEN
	 * is set
	 */
	mdelay(3 * MICFIL_SLEEP);

	/* Wait for INITF to be asserted */
	for (i = 0; i < MICFIL_MAX_RETRY; i++) {
		ret = regmap_read(micfil->regmap, REG_MICFIL_VAD0_STAT, &stat);
		if (ret) {
			dev_err(dev, "failed to read register %d\n",
				REG_MICFIL_VAD0_STAT);
			return ret;
		}

		flag = (stat & MICFIL_VAD0_STAT_INITF_MASK);
		if (flag == 0)
			break;

		mdelay(MICFIL_SLEEP);
	}

	if (i == MICFIL_MAX_RETRY) {
		dev_err(dev, "initf not asserted. Failed to init hwvad\n");
		return -EBUSY;
	}

	/* Initialize Internal Filters */
	ret = init_hwvad_internal_filters(dev);
	if (ret)
		return ret;

	/* Enable interrupts */
	ret = configure_hwvad_interrupts(dev, 1);
	if (ret)
		return ret;

	return ret;
}

/* Hardware Voice Active Detection: The HWVAD takes data from the input
 * of a selected PDM microphone to detect if there is any
 * voice activity. When a voice activity is detected, an interrupt could
 * be delivered to the system. Initialization in section 8.4:
 * Can work in two modes:
 *  -> Eneveope-based mode (section 8.4.1)
 *  -> Energy-based mode (section 8.4.2)
 *
 * It is important to remark that the HWVAD detector could be enabled
 * or reset only when the MICFIL isn't running i.e. when the BSY_FIL
 * bit in STAT register is cleared
 */
static int __maybe_unused init_hwvad(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	u32 reg_val;

	/* configure CIC OSR in VADCICOSR */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_CICOSR_MASK,
				 MICFIL_CTRL2_OSR_DEFAULT);
	if (ret) {
		dev_err(dev, "Failed to set CICOSR in CTRL1_VAD0i [%d]\n", ret);
		return ret;
	}

	/* configure source channel in VADCHSEL */
	reg_val = MICFIL_VAD0_CTRL1_CHSEL(micfil->vad_channel);
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_CHSEL_MASK,
				 reg_val);
	if (ret) {
		dev_err(dev, "Failed to set CHSEL in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure detector frame time VADFRAMET */
	reg_val = MICFIL_VAD0_CTRL2_FRAMET(micfil->vad_frame_time);
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL2,
				 MICFIL_VAD0_CTRL2_FRAMET_MASK,
				 reg_val);
	if (ret) {
		dev_err(dev, "Failed to set FRAMET in CTRL2_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure initialization time in VADINITT */
	reg_val = MICFIL_VAD0_CTRL1_INITT(micfil->vad_init_time);
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL1,
				 MICFIL_VAD0_CTRL1_INITT_MASK,
				 reg_val);
	if (ret) {
		dev_err(dev, "Failed to set INITT in CTRL1_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure input gain in VADINPGAIN */
	reg_val = MICFIL_VAD0_CTRL2_INPGAIN(micfil->vad_input_gain);
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL2,
				 MICFIL_VAD0_CTRL2_INPGAIN_MASK,
				 reg_val);
	if (ret) {
		dev_err(dev, "Failed to set INPGAIN in CTRL2_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure sound gain in SGAIN */
	reg_val = MICFIL_VAD0_SCONFIG_SGAIN(micfil->vad_sound_gain);
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_SCONFIG,
				 MICFIL_VAD0_SCONFIG_SGAIN_MASK,
				 reg_val);
	if (ret) {
		dev_err(dev, "Failed to set SGAIN in SCONFIG_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure noise gain in NGAIN */
	reg_val = MICFIL_VAD0_NCONFIG_NGAIN(micfil->vad_noise_gain);
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_NCONFIG,
				 MICFIL_VAD0_NCONFIG_NGAIN_MASK,
				 reg_val);
	if (ret) {
		dev_err(dev, "Failed to set NGAIN in NCONFIG_VAD0 [%d]\n", ret);
		return ret;
	}

	/* configure or clear the VADNFILADJ based on mode */
	reg_val = MICFIL_VAD0_NCONFIG_NFILADJ(micfil->vad_nfil_adjust);
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_NCONFIG,
				 MICFIL_VAD0_NCONFIG_NFILADJ_MASK,
				 reg_val);
	if (ret) {
		dev_err(dev,
			"Failed to set VADNFILADJ in NCONFIG_VAD0 [%d]\n",
			ret);
		return ret;
	}

	/* enable the high-pass filter in VADHPF */
	reg_val = MICFIL_VAD0_CTRL2_HPF(micfil->vad_hpf);
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_VAD0_CTRL2,
				 MICFIL_VAD0_CTRL2_HPF_MASK,
				 reg_val);
	if (ret) {
		dev_err(dev, "Failed to set HPF in CTRL2_VAD0 [%d]\n", ret);
		return ret;
	}

	/* envelope-based specific initialization */
	if (micfil->vad_init_mode == MICFIL_HWVAD_ENVELOPE_MODE) {
		ret = init_hwvad_envelope_mode(dev);
		if (ret)
			return ret;
	} else {
		ret = init_hwvad_energy_mode(dev);
		if (ret)
			return ret;
	}

	return 0;
}

static inline bool clk_in_list(struct clk *p, struct clk *clk_src[])
{
	int i;

	for (i = 0; i < MICFIL_CLK_SRC_NUM; i++)
		if (clk_is_match(p, clk_src[i]))
			return true;

	return false;
}

#define CLK_8K_FREQ    24576000
#define CLK_11K_FREQ   22579200

static int fsl_micfil_set_mclk_rate(struct fsl_micfil *micfil, int clk_id,
				    unsigned int freq)
{
	struct clk *p = micfil->mclk, *pll = 0, *npll = 0;
	struct device *dev = &micfil->pdev->dev;
	u64 ratio = freq;
	u64 clk_rate;
	int ret;
	int i;

	/* Do not touch the clock if hwvad is already enabled
	 * since you can record only at hwvad rate and clock
	 * has already been set to the required frequency
	 */
	if (atomic_read(&micfil->hwvad_state) == MICFIL_HWVAD_ON)
		return 0;

	while (p) {
		struct clk *pp = clk_get_parent(p);

		if (clk_in_list(pp, micfil->clk_src)) {
			pll = pp;
			break;
		}
		p = pp;
	}

	if (!pll) {
		dev_err(dev, "reached a null clock\n");
		return -EINVAL;
	}

	if (micfil->clk_src_id == MICFIL_CLK_AUTO) {
		for (i = 0; i < MICFIL_CLK_SRC_NUM; i++) {
			clk_rate = clk_get_rate(micfil->clk_src[i]);
			/* This is an workaround since audio_pll2 clock
			 * has 722534399 rate and this will never divide
			 * to any known frequency ???
			 */
			clk_rate = round_up(clk_rate, 10);
			if (clk_rate != 0 && do_div(clk_rate, ratio) == 0) {
				npll = micfil->clk_src[i];
				break;
			}
		}
	} else {
		/* clock id is offseted by 1 since ID=0 means
		 * auto clock selection
		 */
		npll = micfil->clk_src[micfil->clk_src_id - 1];
	}

	if (!npll) {
		dev_err(dev,
			"failed to find a suitable clock source\n");
		return -EINVAL;
	}

	clk_disable_unprepare(micfil->mclk);
	if (!clk_is_match(pll, npll)) {
		ret = clk_set_parent(p, npll);
		if (ret < 0)
			dev_warn(dev,
				 "failed to set parrent %d\n", ret);
	}

	clk_rate = freq % 8000 == 0 ? CLK_8K_FREQ : CLK_11K_FREQ;
	ret = clk_set_rate(micfil->mclk, clk_rate);
	if (ret)
		dev_warn(dev, "failed to set rate (%llu): %d\n", clk_rate, ret);
	clk_prepare_enable(micfil->mclk);

	return ret;
}

static int fsl_micfil_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	unsigned int rates[MICFIL_NUM_RATES] = {8000, 11025, 16000, 22050, 32000, 44100, 48000};
	int i, j, k = 0;
	u64 clk_rate;

	if (!micfil) {
		dev_err(dai->dev, "micfil dai priv_data not set\n");
		return -EINVAL;
	}

	micfil->constraint_rates.list = micfil->constraint_rates_list;
	micfil->constraint_rates.count = 0;

	for (j = 0; j < MICFIL_NUM_RATES; j++) {
		for (i = 0; i < MICFIL_CLK_SRC_NUM; i++) {
			clk_rate = clk_get_rate(micfil->clk_src[i]);
			if (clk_rate != 0 && do_div(clk_rate, rates[j]) == 0) {
				micfil->constraint_rates_list[k++] = rates[j];
				micfil->constraint_rates.count++;
				break;
			}
		}
	}

	snd_pcm_hw_constraint_list(substream->runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
				   &micfil->constraint_rates);

	return 0;
}

static int fsl_micfil_trigger(struct snd_pcm_substream *substream, int cmd,
			      struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &micfil->pdev->dev;
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = fsl_micfil_reset(dev);
		if (ret) {
			dev_err(dev, "failed to soft reset\n");
			return ret;
		}

		/* DMA Interrupt Selection - DISEL bits
		 * 00 - DMA and IRQ disabled
		 * 01 - DMA req enabled
		 * 10 - IRQ enabled
		 * 11 - reserved
		 */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
					 MICFIL_CTRL1_DISEL_MASK,
					 (1 << MICFIL_CTRL1_DISEL_SHIFT));
		if (ret) {
			dev_err(dev, "failed to update DISEL bits\n");
			return ret;
		}

		/* Enable the module */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
					 MICFIL_CTRL1_PDMIEN_MASK,
					 MICFIL_CTRL1_PDMIEN);
		if (ret) {
			dev_err(dev, "failed to enable the module\n");
			return ret;
		}

		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* Disable the module */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
					 MICFIL_CTRL1_PDMIEN_MASK,
					 0);
		if (ret) {
			dev_err(dev, "failed to enable the module\n");
			return ret;
		}

		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
					 MICFIL_CTRL1_DISEL_MASK,
					 (0 << MICFIL_CTRL1_DISEL_SHIFT));
		if (ret) {
			dev_err(dev, "failed to update DISEL bits\n");
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int fsl_set_clock_params(struct device *dev, unsigned int rate)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int clk_div;
	int ret;

	ret = fsl_micfil_set_mclk_rate(micfil, 0, rate);
	if (ret < 0)
		dev_err(dev, "failed to set mclk[%lu] to rate %u\n",
			clk_get_rate(micfil->mclk), rate);

	/* set CICOSR */
	ret |= regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL2,
				 MICFIL_CTRL2_CICOSR_MASK,
				 MICFIL_CTRL2_OSR_DEFAULT);
	if (ret)
		dev_err(dev, "failed to set CICOSR in reg 0x%X\n",
			REG_MICFIL_CTRL2);

	/* set CLK_DIV */
	clk_div = get_clk_div(micfil, rate);
	if (clk_div < 0)
		ret = -EINVAL;

	ret |= regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL2,
				 MICFIL_CTRL2_CLKDIV_MASK, clk_div);
	if (ret)
		dev_err(dev, "failed to set CLKDIV in reg 0x%X\n",
			REG_MICFIL_CTRL2);

	return ret;
}

static int fsl_micfil_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	struct device *dev = &micfil->pdev->dev;
	unsigned int hwvad_rate;
	int ret;
	u32 hwvad_state;

	hwvad_rate = micfil_hwvad_rate_ints[micfil->vad_rate_index];
	hwvad_state = atomic_read(&micfil->hwvad_state);

	/* if hwvad is enabled, make sure you are recording at
	 * the same rate the hwvad is on or reject it to avoid
	 * changing the clock rate.
	 */
	if (hwvad_state == MICFIL_HWVAD_ON && rate != hwvad_rate) {
		dev_err(dev, "Record at hwvad rate %u\n", hwvad_rate);
		return -EINVAL;
	}

	atomic_set(&micfil->recording_state, MICFIL_RECORDING_ON);

	if (hwvad_state == MICFIL_HWVAD_OFF) {
		/* 1. Disable the module */
		ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
					 MICFIL_CTRL1_PDMIEN_MASK, 0);
		if (ret) {
			dev_err(dev, "failed to disable the module\n");
			return ret;
		}
	}

	/* enable channels */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL1,
				 0xFF, ((1 << channels) - 1));
	if (ret) {
		dev_err(dev, "failed to enable channels %d, reg 0x%X\n", ret,
			REG_MICFIL_CTRL1);
		return ret;
	}

	ret = fsl_set_clock_params(dev, rate);
	if (ret < 0) {
		dev_err(dev, "Failed to set clock parameters [%d]\n", ret);
		return ret;
	}

	micfil->audio_config.src_fifo_num = channels;
	micfil->audio_config.sw_done_sel = BIT(31);
	micfil->dma_params_rx.peripheral_config  = &micfil->audio_config;
	micfil->dma_params_rx.peripheral_size    = sizeof(micfil->audio_config);
	if (micfil->soc->use_edma)
		micfil->dma_params_rx.maxburst = channels;
	else
		micfil->dma_params_rx.maxburst = channels * MICFIL_DMA_MAXBURST_RX;

	return 0;
}

static int fsl_micfil_hw_free(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);

	atomic_set(&micfil->recording_state, MICFIL_RECORDING_OFF);

	return 0;
}

static int fsl_micfil_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
				     unsigned int freq, int dir)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);
	struct device *dev = &micfil->pdev->dev;

	int ret;

	if (!freq)
		return 0;

	ret = fsl_micfil_set_mclk_rate(micfil, clk_id, freq);
	if (ret < 0)
		dev_err(dev, "failed to set mclk[%lu] to rate %u\n",
			clk_get_rate(micfil->mclk), freq);

	return ret;
}

static int fsl_micfil_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct fsl_micfil *micfil = snd_soc_dai_get_drvdata(dai);

	/* DAI MODE */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	/* DAI CLK INVERSION */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	micfil->slave_mode = false;

	return 0;
}

static const struct snd_soc_dai_ops fsl_micfil_dai_ops = {
	.startup = fsl_micfil_startup,
	.trigger = fsl_micfil_trigger,
	.hw_params = fsl_micfil_hw_params,
	.hw_free = fsl_micfil_hw_free,
	.set_sysclk = fsl_micfil_set_dai_sysclk,
	.set_fmt = fsl_micfil_set_dai_fmt,
};

static int fsl_micfil_dai_probe(struct snd_soc_dai *cpu_dai)
{
	struct fsl_micfil *micfil = dev_get_drvdata(cpu_dai->dev);
	struct device *dev = cpu_dai->dev;
	unsigned int val;
	int ret;
	int i;

	/* set qsel to medium */
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_CTRL2,
				 MICFIL_CTRL2_QSEL_MASK, MICFIL_VLOW0_QUALITY);
	if (ret) {
		dev_err(dev, "failed to set quality mode bits, reg 0x%X\n",
			REG_MICFIL_CTRL2);
		return ret;
	}

	/* set default gain to 2 */
	regmap_write(micfil->regmap, REG_MICFIL_OUT_CTRL, 0x22222222);
	for (i = 0; i < MICFIL_OUTPUT_CHANNELS; i++)
		micfil->channel_gain[i] = 0xA;

	/* set DC Remover in bypass mode*/
	val = 0;
	for (i = 0; i < MICFIL_OUTPUT_CHANNELS; i++)
		val |= MICFIL_DC_MODE(MICFIL_DC_BYPASS, i);
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_DC_CTRL,
				MICFIL_DC_CTRL_MASK, val);
	if (ret) {
		dev_err(dev, "failed to set DC Remover mode bits, reg 0x%X\n",
			REG_MICFIL_DC_CTRL);
		return ret;
	}
	micfil->dc_remover = MICFIL_DC_BYPASS;

	snd_soc_dai_init_dma_data(cpu_dai, NULL,
				  &micfil->dma_params_rx);

	/* FIFO Watermark Control - FIFOWMK*/
	val = MICFIL_FIFO_CTRL_FIFOWMK(micfil->soc->fifo_depth) - 1;
	ret = regmap_update_bits(micfil->regmap, REG_MICFIL_FIFO_CTRL,
				 MICFIL_FIFO_CTRL_FIFOWMK_MASK,
				 val);
	if (ret) {
		dev_err(dev, "failed to set FIFOWMK\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_dai_driver fsl_micfil_dai = {
	.probe = fsl_micfil_dai_probe,
	.capture = {
		.stream_name = "CPU-Capture",
		.channels_min = 1,
		.channels_max = 8,
		.rates = FSL_MICFIL_RATES,
		.formats = FSL_MICFIL_FORMATS,
	},
	.ops = &fsl_micfil_dai_ops,
};

static const struct snd_soc_component_driver fsl_micfil_component = {
	.name		= "fsl-micfil-dai",
	.controls       = fsl_micfil_snd_controls,
	.num_controls   = ARRAY_SIZE(fsl_micfil_snd_controls),

};

/* REGMAP */
static const struct reg_default fsl_micfil_reg_defaults[] = {
	{REG_MICFIL_CTRL1,		0x00000000},
	{REG_MICFIL_CTRL2,		0x00000000},
	{REG_MICFIL_STAT,		0x00000000},
	{REG_MICFIL_FIFO_CTRL,		0x00000007},
	{REG_MICFIL_FIFO_STAT,		0x00000000},
	{REG_MICFIL_DATACH0,		0x00000000},
	{REG_MICFIL_DATACH1,		0x00000000},
	{REG_MICFIL_DATACH2,		0x00000000},
	{REG_MICFIL_DATACH3,		0x00000000},
	{REG_MICFIL_DATACH4,		0x00000000},
	{REG_MICFIL_DATACH5,		0x00000000},
	{REG_MICFIL_DATACH6,		0x00000000},
	{REG_MICFIL_DATACH7,		0x00000000},
	{REG_MICFIL_DC_CTRL,		0x00000000},
	{REG_MICFIL_OUT_CTRL,		0x00000000},
	{REG_MICFIL_OUT_STAT,		0x00000000},
	{REG_MICFIL_VAD0_CTRL1,		0x00000000},
	{REG_MICFIL_VAD0_CTRL2,		0x000A0000},
	{REG_MICFIL_VAD0_STAT,		0x00000000},
	{REG_MICFIL_VAD0_SCONFIG,	0x00000000},
	{REG_MICFIL_VAD0_NCONFIG,	0x80000000},
	{REG_MICFIL_VAD0_NDATA,		0x00000000},
	{REG_MICFIL_VAD0_ZCD,		0x00000004},
};

static bool fsl_micfil_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_MICFIL_CTRL1:
	case REG_MICFIL_CTRL2:
	case REG_MICFIL_STAT:
	case REG_MICFIL_FIFO_CTRL:
	case REG_MICFIL_FIFO_STAT:
	case REG_MICFIL_DATACH0:
	case REG_MICFIL_DATACH1:
	case REG_MICFIL_DATACH2:
	case REG_MICFIL_DATACH3:
	case REG_MICFIL_DATACH4:
	case REG_MICFIL_DATACH5:
	case REG_MICFIL_DATACH6:
	case REG_MICFIL_DATACH7:
	case REG_MICFIL_DC_CTRL:
	case REG_MICFIL_OUT_CTRL:
	case REG_MICFIL_OUT_STAT:
	case REG_MICFIL_VAD0_CTRL1:
	case REG_MICFIL_VAD0_CTRL2:
	case REG_MICFIL_VAD0_STAT:
	case REG_MICFIL_VAD0_SCONFIG:
	case REG_MICFIL_VAD0_NCONFIG:
	case REG_MICFIL_VAD0_NDATA:
	case REG_MICFIL_VAD0_ZCD:
		return true;
	default:
		return false;
	}
}

static bool fsl_micfil_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_MICFIL_CTRL1:
	case REG_MICFIL_CTRL2:
	case REG_MICFIL_STAT:		/* Write 1 to Clear */
	case REG_MICFIL_FIFO_CTRL:
	case REG_MICFIL_FIFO_STAT:	/* Write 1 to Clear */
	case REG_MICFIL_DC_CTRL:
	case REG_MICFIL_OUT_CTRL:
	case REG_MICFIL_OUT_STAT:	/* Write 1 to Clear */
	case REG_MICFIL_VAD0_CTRL1:
	case REG_MICFIL_VAD0_CTRL2:
	case REG_MICFIL_VAD0_STAT:	/* Write 1 to Clear */
	case REG_MICFIL_VAD0_SCONFIG:
	case REG_MICFIL_VAD0_NCONFIG:
	case REG_MICFIL_VAD0_ZCD:
		return true;
	default:
		return false;
	}
}

static bool fsl_micfil_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_MICFIL_STAT:
	case REG_MICFIL_DATACH0:
	case REG_MICFIL_DATACH1:
	case REG_MICFIL_DATACH2:
	case REG_MICFIL_DATACH3:
	case REG_MICFIL_DATACH4:
	case REG_MICFIL_DATACH5:
	case REG_MICFIL_DATACH6:
	case REG_MICFIL_DATACH7:
	case REG_MICFIL_VAD0_STAT:
	case REG_MICFIL_VAD0_NDATA:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config fsl_micfil_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.max_register = REG_MICFIL_VAD0_ZCD,
	.reg_defaults = fsl_micfil_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(fsl_micfil_reg_defaults),
	.readable_reg = fsl_micfil_readable_reg,
	.volatile_reg = fsl_micfil_volatile_reg,
	.writeable_reg = fsl_micfil_writeable_reg,
	.cache_type = REGCACHE_RBTREE,
};

/* END OF REGMAP */

static irqreturn_t voice_detected_fn(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct device *dev = &micfil->pdev->dev;
	int ret;

	/* disable hwvad */
	spin_lock(&micfil->hwvad_lock);
	ret = disable_hwvad(dev, true);
	spin_unlock(&micfil->hwvad_lock);

	if (ret)
		dev_err(dev, "Failed to disable HWVAD module: %d\n", ret);

	/* notify userspace that voice was detected */
	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);

	return IRQ_HANDLED;
}

static irqreturn_t hwvad_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct device *dev = &micfil->pdev->dev;
	int ret;
	u32 vad0_reg;

	regmap_read(micfil->regmap, REG_MICFIL_VAD0_STAT, &vad0_reg);

	/* The only difference between MICFIL_VAD0_STAT_EF and
	 * MICFIL_VAD0_STAT_IF is that the former requires Write
	 * 1 to Clear. Since both flags are set, it is enough
	 * to only read one of them
	 */
	if (vad0_reg & MICFIL_VAD0_STAT_IF_MASK) {
		/* Write 1 to clear */
		regmap_write_bits(micfil->regmap, REG_MICFIL_VAD0_STAT,
				  MICFIL_VAD0_STAT_IF_MASK,
				  MICFIL_VAD0_STAT_IF);

		/* disable hwvad interrupts */
		ret = configure_hwvad_interrupts(dev, 0);
		if (ret)
			dev_err(dev, "Failed to disable interrupts\n");
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t hwvad_err_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct device *dev = &micfil->pdev->dev;
	u32 vad0_reg;

	regmap_read(micfil->regmap, REG_MICFIL_VAD0_STAT, &vad0_reg);

	if (vad0_reg & MICFIL_VAD0_STAT_INSATF_MASK)
		dev_dbg(dev, "voice activity input overflow/underflow detected\n");

	if (vad0_reg & MICFIL_VAD0_STAT_INITF_MASK)
		dev_dbg(dev, "voice activity dectector is initializing\n");

	return IRQ_HANDLED;
}

static irqreturn_t micfil_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct platform_device *pdev = micfil->pdev;
	u32 stat_reg;
	u32 fifo_stat_reg;
	u32 ctrl1_reg;
	bool dma_enabled;
	int i;

	regmap_read(micfil->regmap, REG_MICFIL_STAT, &stat_reg);
	regmap_read(micfil->regmap, REG_MICFIL_CTRL1, &ctrl1_reg);
	regmap_read(micfil->regmap, REG_MICFIL_FIFO_STAT, &fifo_stat_reg);

	dma_enabled = MICFIL_DMA_ENABLED(ctrl1_reg);

	/* Channel 0-7 Output Data Flags */
	for (i = 0; i < MICFIL_OUTPUT_CHANNELS; i++) {
		if (stat_reg & MICFIL_STAT_CHXF_MASK(i))
			dev_dbg(&pdev->dev,
				"Data available in Data Channel %d\n", i);
		/* if DMA is not enabled, field must be written with 1
		 * to clear
		 */
		if (!dma_enabled)
			regmap_write_bits(micfil->regmap,
					  REG_MICFIL_STAT,
					  MICFIL_STAT_CHXF_MASK(i),
					  1);
	}

	for (i = 0; i < MICFIL_FIFO_NUM; i++) {
		if (fifo_stat_reg & MICFIL_FIFO_STAT_FIFOX_OVER_MASK(i))
			dev_dbg(&pdev->dev,
				"FIFO Overflow Exception flag for channel %d\n",
				i);

		if (fifo_stat_reg & MICFIL_FIFO_STAT_FIFOX_UNDER_MASK(i))
			dev_dbg(&pdev->dev,
				"FIFO Underflow Exception flag for channel %d\n",
				i);
	}

	return IRQ_HANDLED;
}

static irqreturn_t micfil_err_isr(int irq, void *devid)
{
	struct fsl_micfil *micfil = (struct fsl_micfil *)devid;
	struct platform_device *pdev = micfil->pdev;
	u32 stat_reg;

	regmap_read(micfil->regmap, REG_MICFIL_STAT, &stat_reg);

	if (stat_reg & MICFIL_STAT_BSY_FIL_MASK)
		dev_dbg(&pdev->dev, "isr: Decimation Filter is running\n");

	if (stat_reg & MICFIL_STAT_FIR_RDY_MASK)
		dev_dbg(&pdev->dev, "isr: FIR Filter Data ready\n");

	if (stat_reg & MICFIL_STAT_LOWFREQF_MASK) {
		dev_dbg(&pdev->dev, "isr: ipg_clk_app is too low\n");
		regmap_write_bits(micfil->regmap, REG_MICFIL_STAT,
				  MICFIL_STAT_LOWFREQF_MASK, 1);
	}

	return IRQ_HANDLED;
}

static int fsl_set_clock_params(struct device *, unsigned int);

static int enable_hwvad(struct device *dev, bool sync)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	int rate;
	u32 state;

	if (sync)
		pm_runtime_get_sync(dev);

	state = atomic_cmpxchg(&micfil->hwvad_state,
			       MICFIL_HWVAD_OFF,
			       MICFIL_HWVAD_ON);

	/* we should not reenable when sync = true because
	 * this means enable was called for second time by
	 * user. However state = ON and sync = false can only
	 * occur when enable is called from system_resume. In
	 * this case we should enable the hwvad
	 */
	if (sync && state == MICFIL_HWVAD_ON) {
		dev_err(dev, "hwvad already on\n");
		ret = -EBUSY;
		goto enable_error;
	}

	if (micfil->vad_rate_index >= ARRAY_SIZE(micfil_hwvad_rate_ints)) {
		dev_err(dev, "There are more select texts than rates\n");
		ret = -EINVAL;
		goto enable_error;
	}

	rate = micfil_hwvad_rate_ints[micfil->vad_rate_index];

	/* This is required because if an arecord was done,
	 * suspend function will mark regmap as cache only
	 * and reads/writes in volatile regs will fail
	 */
	regcache_cache_only(micfil->regmap, false);
	regcache_mark_dirty(micfil->regmap);
	regcache_sync(micfil->regmap);

	ret = fsl_set_clock_params(dev, rate);
	if (ret)
		goto enable_error;

	ret = fsl_micfil_reset(dev);
	if (ret)
		goto enable_error;

	/* Initialize Hardware Voice Activity */
	ret = init_hwvad(dev);
	if (ret == 0)
		return 0;

enable_error:
	if (state == MICFIL_HWVAD_OFF)
		atomic_cmpxchg(&micfil->hwvad_state,
			       MICFIL_HWVAD_ON, MICFIL_HWVAD_OFF);
	if (sync)
		pm_runtime_put_sync(dev);
	return ret;
}

static int disable_hwvad(struct device *dev, bool sync)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret = 0;
	u32 state;

	/* disable is called with sync = false only from
	 * system suspend and in this case, you should not
	 * change the hwvad_state so we know at system_resume
	 * to reenable hwvad
	 */
	if (sync)
		state = atomic_cmpxchg(&micfil->hwvad_state,
				       MICFIL_HWVAD_ON,
				       MICFIL_HWVAD_OFF);
	else
		state = atomic_read(&micfil->hwvad_state);

	if (state == MICFIL_HWVAD_ON) {
		/* This is required because if an arecord was done,
		 * suspend function will mark regmap as cache only
		 * and reads/writes in volatile regs will fail
		 */
		regcache_cache_only(micfil->regmap, false);
		regcache_mark_dirty(micfil->regmap);
		regcache_sync(micfil->regmap);

		/* Voice Activity Detector Reset */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_CTRL1,
					  MICFIL_VAD0_CTRL1_RST_SHIFT,
					  MICFIL_VAD0_CTRL1_RST);

		/* Disable HWVAD */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_CTRL1,
					  MICFIL_VAD0_CTRL1_EN_MASK,
					  0);

		/* Disable Signal Filter */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_SCONFIG,
					  MICFIL_VAD0_SCONFIG_SFILEN_MASK,
					  0);

		/* Signal Maximum Enable */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_SCONFIG,
					  MICFIL_VAD0_SCONFIG_SMAXEN_MASK,
					  0);

		/* Enable pre-filter Noise & Signal */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_CTRL2,
					  MICFIL_VAD0_CTRL2_PREFEN_MASK,
					  0);

		/* Noise Decimation Enable */
		ret |= regmap_update_bits(micfil->regmap,
					  REG_MICFIL_VAD0_NCONFIG,
					  MICFIL_VAD0_NCONFIG_NDECEN_MASK,
					  0);

		/* disable the module and clock only if recording
		 * is not done in parallel
		 */
		state = atomic_read(&micfil->recording_state);
		if (state == MICFIL_RECORDING_OFF) {
		/* Disable MICFIL module */
			ret |= regmap_update_bits(micfil->regmap,
						  REG_MICFIL_CTRL1,
						  MICFIL_CTRL1_PDMIEN_MASK,
						  0);
		}

		if (sync)
			pm_runtime_put_sync(dev);
	} else {
		ret = -EPERM;
		dev_err(dev, "HWVAD is not enabled %d\n", ret);
	}

	return ret;
}

static ssize_t micfil_hwvad_handler(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct kobject *nand_kobj = kobj->parent;
	struct device *dev = container_of(nand_kobj, struct device, kobj);
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	unsigned long vad_channel;
	int ret;

	ret = kstrtoul(buf, 16, &vad_channel);
	if (ret < 0)
		return -EINVAL;

	spin_lock(&micfil->hwvad_lock);
	if (vad_channel <= 7) {
		micfil->vad_channel = vad_channel;
		ret = enable_hwvad(dev, true);
	} else {
		micfil->vad_channel = -1;
		ret = disable_hwvad(dev, true);
	}
	spin_unlock(&micfil->hwvad_lock);

	if (ret) {
		dev_err(dev, "Failed to %s hwvad: %d\n",
			vad_channel <= 7 ? "enable" : "disable", ret);
		return ret;
	}

	return count;
}

static struct kobj_attribute hwvad_en_attr = __ATTR(enable,
						   0660,
						   NULL,
						   micfil_hwvad_handler);

static int fsl_micfil_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_micfil *micfil;
	struct resource *res;
	void __iomem *regs;
	int ret, i;
	unsigned long irqflag = 0;

	micfil = devm_kzalloc(&pdev->dev, sizeof(*micfil), GFP_KERNEL);
	if (!micfil)
		return -ENOMEM;

	micfil->pdev = pdev;
	strncpy(micfil->name, np->name, sizeof(micfil->name) - 1);

	micfil->soc = of_device_get_match_data(&pdev->dev);

	/* ipg_clk is used to control the registers
	 * ipg_clk_app is used to operate the filter
	 */
	micfil->mclk = devm_clk_get(&pdev->dev, "ipg_clk_app");
	if (IS_ERR(micfil->mclk)) {
		dev_err(&pdev->dev, "failed to get core clock: %ld\n",
			PTR_ERR(micfil->mclk));
		return PTR_ERR(micfil->mclk);
	}

	micfil->busclk = devm_clk_get(&pdev->dev, "ipg_clk");
	if (IS_ERR(micfil->busclk)) {
		dev_err(&pdev->dev, "failed to get ipg clock: %ld\n",
			PTR_ERR(micfil->busclk));
		return PTR_ERR(micfil->busclk);
	}

	/* get audio pll1 and pll2 */
	micfil->clk_src[MICFIL_AUDIO_PLL1] = devm_clk_get(&pdev->dev, "pll8k");
	if (IS_ERR(micfil->clk_src[MICFIL_AUDIO_PLL1]))
		micfil->clk_src[MICFIL_AUDIO_PLL1] = NULL;

	micfil->clk_src[MICFIL_AUDIO_PLL2] = devm_clk_get(&pdev->dev, "pll11k");
	if (IS_ERR(micfil->clk_src[MICFIL_AUDIO_PLL2]))
		micfil->clk_src[MICFIL_AUDIO_PLL2] = NULL;

	micfil->clk_src[MICFIL_CLK_EXT3] = devm_clk_get(&pdev->dev, "clkext3");
	if (IS_ERR(micfil->clk_src[MICFIL_CLK_EXT3]))
		micfil->clk_src[MICFIL_CLK_EXT3] = NULL;

	/* init regmap */
	regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	micfil->regmap = devm_regmap_init_mmio(&pdev->dev,
					       regs,
					       &fsl_micfil_regmap_config);
	if (IS_ERR(micfil->regmap)) {
		dev_err(&pdev->dev, "failed to init MICFIL regmap: %ld\n",
			PTR_ERR(micfil->regmap));
		return PTR_ERR(micfil->regmap);
	}

	/* dataline mask for RX */
	ret = of_property_read_u32_index(np,
					 "fsl,dataline",
					 0,
					 &micfil->dataline);
	if (ret)
		micfil->dataline = 1;

	if (micfil->dataline & ~micfil->soc->dataline) {
		dev_err(&pdev->dev, "dataline setting error, Mask is 0x%X\n",
			micfil->soc->dataline);
		return -EINVAL;
	}

	/* get IRQs */
	for (i = 0; i < MICFIL_IRQ_LINES; i++) {
		micfil->irq[i] = platform_get_irq(pdev, i);
		dev_err(&pdev->dev, "GET IRQ: %d\n", micfil->irq[i]);
		if (micfil->irq[i] < 0)
			return micfil->irq[i];
	}

	if (of_property_read_bool(np, "fsl,shared-interrupt"))
		irqflag = IRQF_SHARED;

	/* Digital Microphone interface voice activity detector event
	 * interrupt - IRQ 44
	 */
	ret = devm_request_threaded_irq(&pdev->dev, micfil->irq[2],
					hwvad_isr, voice_detected_fn,
					irqflag, micfil->name, micfil);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim hwvad event irq %u\n",
			micfil->irq[0]);
		return ret;
	}

	/* Digital Microphone interface voice activity detector error
	 * interrupt - IRQ 45
	 */
	ret = devm_request_irq(&pdev->dev, micfil->irq[3],
			       hwvad_err_isr, irqflag,
			       micfil->name, micfil);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim hwvad error irq %u\n",
			micfil->irq[1]);
		return ret;
	}

	/* Digital Microphone interface interrupt - IRQ 109 */
	ret = devm_request_irq(&pdev->dev, micfil->irq[0],
			       micfil_isr, irqflag,
			       micfil->name, micfil);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim mic interface irq %u\n",
			micfil->irq[0]);
		return ret;
	}

	/* Digital Microphone interface error interrupt */
	ret = devm_request_irq(&pdev->dev, micfil->irq[1],
			       micfil_err_isr, irqflag,
			       micfil->name, micfil);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim mic interface error irq %u\n",
			micfil->irq[1]);
		return ret;
	}

	micfil->slave_mode = false;

	micfil->dma_params_rx.chan_name = "rx";
	micfil->dma_params_rx.addr = res->start + REG_MICFIL_DATACH0;
	micfil->dma_params_rx.maxburst = MICFIL_DMA_MAXBURST_RX;

	/* set default rate to first value in available vad rates */
	micfil->vad_rate_index = 0;
	/* init HWVAD enable/disable spinlock */
	spin_lock_init(&micfil->hwvad_lock);

	platform_set_drvdata(pdev, micfil);

	pm_runtime_enable(&pdev->dev);
	regcache_cache_only(micfil->regmap, true);

	/*
	 * Register platform component before registering cpu dai for there
	 * is not defer probe for platform component in snd_soc_add_pcm_runtime().
	 */
	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "failed to pcm register\n");
		return ret;
	}

	fsl_micfil_dai.capture.formats = micfil->soc->formats;

	ret = devm_snd_soc_register_component(&pdev->dev, &fsl_micfil_component,
					      &fsl_micfil_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "failed to register component %s\n",
			fsl_micfil_component.name);
		return ret;
	}

	/* create sysfs entry used to enable hwvad from userspace */
	micfil->hwvad_kobject = kobject_create_and_add("hwvad",
						       &pdev->dev.kobj);
	if (!micfil->hwvad_kobject)
		return -ENOMEM;

	ret = sysfs_create_file(micfil->hwvad_kobject,
				&hwvad_en_attr.attr);
	if (ret) {
		dev_err(&pdev->dev, "failed to create file for hwvad_enable\n");
		kobject_put(micfil->hwvad_kobject);
		return -ENOMEM;
	}

	return 0;
}

static int __maybe_unused fsl_micfil_runtime_suspend(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	u32 state;

	state = atomic_read(&micfil->hwvad_state);
	if (state == MICFIL_HWVAD_ON)
		return 0;

	regcache_cache_only(micfil->regmap, true);

	clk_disable_unprepare(micfil->mclk);
	clk_disable_unprepare(micfil->busclk);

	return 0;
}

static int __maybe_unused fsl_micfil_runtime_resume(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	u32 state;

	state = atomic_read(&micfil->hwvad_state);

	/* enable mclk only if the hwvad is not enabled
	 * When hwvad is enabled, clock won't be disabled
	 * in suspend since hwvad and recording share the
	 * same clock
	 */
	if (state == MICFIL_HWVAD_ON)
		return 0;

	ret = clk_prepare_enable(micfil->busclk);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(micfil->mclk);
	if (ret < 0) {
		clk_disable_unprepare(micfil->busclk);
		return ret;
	}

	regcache_cache_only(micfil->regmap, false);
	regcache_mark_dirty(micfil->regmap);
	regcache_sync(micfil->regmap);

	return 0;
}

static int __maybe_unused fsl_micfil_suspend(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	u32 state;

	state = atomic_read(&micfil->hwvad_state);

	if (state == MICFIL_HWVAD_ON) {
		dev_err(dev, "Disabling hwvad on suspend");
		ret = disable_hwvad(dev, false);
		if (ret)
			dev_warn(dev, "Failed to disable hwvad");
	}

	pm_runtime_force_suspend(dev);

	return 0;
}

static int __maybe_unused fsl_micfil_resume(struct device *dev)
{
	struct fsl_micfil *micfil = dev_get_drvdata(dev);
	int ret;
	u32 state;

	pm_runtime_force_resume(dev);

	state = atomic_read(&micfil->hwvad_state);
	if (state == MICFIL_HWVAD_ON) {
		dev_err(dev, "Enabling hwvad on resume");
		ret = enable_hwvad(dev, false);
		if (ret)
			dev_warn(dev, "Failed to re-enable hwvad");
	}

	return 0;
}

static const struct dev_pm_ops fsl_micfil_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_micfil_runtime_suspend,
			   fsl_micfil_runtime_resume,
			   NULL)
	SET_SYSTEM_SLEEP_PM_OPS(fsl_micfil_suspend,
				fsl_micfil_resume)
};

static struct platform_driver fsl_micfil_driver = {
	.probe = fsl_micfil_probe,
	.driver = {
		.name = "fsl-micfil-dai",
		.pm = &fsl_micfil_pm_ops,
		.of_match_table = fsl_micfil_dt_ids,
	},
};
module_platform_driver(fsl_micfil_driver);

MODULE_AUTHOR("Cosmin-Gabriel Samoila <cosmin.samoila@nxp.com>");
MODULE_DESCRIPTION("NXP PDM Microphone Interface (MICFIL) driver");
MODULE_LICENSE("GPL v2");
