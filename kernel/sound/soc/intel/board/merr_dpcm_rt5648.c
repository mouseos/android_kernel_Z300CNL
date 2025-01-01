/*
 *  merr_saltbay_rt5648.c - ASoc Machine driver for Intel Merrifield MID platform
 *  for the rt5648 codec
 *
 *  Copyright (C) 2012 Intel Corp
 *  Author: Vinod Koul <vinod.koul@linux.intel.com>
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
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/rpmsg.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <linux/platform_data/intel_mid_remoteproc.h>
#include <asm/platform_mrfld_audio.h>
#include <asm/intel_sst_mrfld.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/input.h>
#include "../../codecs/rt5648.h"
#include "../platform-libs/controls_v2_dpcm.h"


#define DEFAULT_MCLK       (19200000)
static unsigned int codec_clk_rate = (48000 * 512);

/* Register address for OSC Clock */
#define MERR_OSC_CLKOUT_CTRL0_REG_ADDR  0xFF00BC04
/* Size of osc clock register */
#define MERR_OSC_CLKOUT_CTRL0_REG_SIZE  4

struct mrfld_mc_private {
	struct snd_soc_jack jack;
	int jack_retry;
	void __iomem    *osc_clk0_reg;
};

struct mrfld_slot_info {
	unsigned int tx_mask;
	unsigned int rx_mask;
	int slots;
	int slot_width;
};

static const struct snd_soc_pcm_stream mrfld_rt5648_dai_params_ssp1_fm = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_PCM_RATE_48000,
	.rate_max = SNDRV_PCM_RATE_48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream mrfld_rt5648_ssp1_bt_nb = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_PCM_RATE_8000,
	.rate_max = SNDRV_PCM_RATE_8000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream mrfld_rt5648_ssp1_bt_wb = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_PCM_RATE_16000,
	.rate_max = SNDRV_PCM_RATE_16000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream mrfld_rt5648_ssp1_bt_a2dp = {
   .formats = SNDRV_PCM_FMTBIT_S24_LE,
   .rate_min = SNDRV_PCM_RATE_48000,
   .rate_max = SNDRV_PCM_RATE_48000,
   .channels_min = 2,
   .channels_max = 2,
};

/* set_osc_clk0-	enable/disables the osc clock0
 * addr:		address of the register to write to
 * enable:		bool to enable or disable the clock
 */
static inline void set_soc_osc_clk0(void __iomem *addr, bool enable)
{
	u32 osc_clk_ctrl;

	osc_clk_ctrl = readl(addr);
	if (enable)
		osc_clk_ctrl |= BIT(31);
	else
		osc_clk_ctrl &= ~(BIT(31));

	pr_debug("%s: enable:%d val 0x%x\n", __func__, enable, osc_clk_ctrl);

	writel(osc_clk_ctrl, addr);
}

/* TODO: find better way of doing this */
static struct snd_soc_dai *find_codec_dai(struct snd_soc_card *card, const char *dai_name)
{
	int i;
	for (i = 0; i < card->num_rtd; i++) {
		if (!strcmp(card->rtd[i].codec_dai->name, dai_name))
			return card->rtd[i].codec_dai;
	}
	pr_err("%s: unable to find codec dai\n", __func__);
	/* this should never occur */
	WARN_ON(1);
	return NULL;
}

static int mrfld_set_clk_fmt(struct snd_soc_dai *codec_dai)
{
	unsigned int fmt;
	int ret;
	struct snd_soc_card *card = codec_dai->card;
	struct mrfld_mc_private *ctx = snd_soc_card_get_drvdata(card);

	pr_debug("Enter in %s\n", __func__);

	/* Enable the osc clock at start so that it gets settling time */
	set_soc_osc_clk0(ctx->osc_clk0_reg, true);

	/* ALC5648 Slave Mode */
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);

	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5648_PLL1_S_MCLK,
				  DEFAULT_MCLK, codec_clk_rate);
	if (ret < 0) {
		pr_err("can't set codec pll: %d\n", ret);
		return ret;
	}
	ret = snd_soc_dai_set_sysclk(codec_dai, RT5648_SCLK_S_PLL1,
					codec_clk_rate, SND_SOC_CLOCK_IN);

	if (ret < 0) {
		pr_err("can't set codec clock %d\n", ret);
		return ret;
	}
	return 0;
}

static int mrfld_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	if (!strcmp(codec_dai->name, "rt5648-aif1"))
		return mrfld_set_clk_fmt(codec_dai);
	return 0;
}

static int mrfld_compr_set_params(struct snd_compr_stream *cstream)
{
	return 0;
}

static const struct snd_soc_pcm_stream mrfld_rt5648_dai_params_ssp0 = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_PCM_RATE_48000,
	.rate_max = SNDRV_PCM_RATE_48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream mrfld_rt5648_dai_params_ssp2 = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = SNDRV_PCM_RATE_48000,
	.rate_max = SNDRV_PCM_RATE_48000,
	.channels_min = 2,
	.channels_max = 2,
};

#define MRFLD_CONFIG_SLOT(slot_tx_mask, slot_rx_mask, num_slot, width)\
	(struct mrfld_slot_info){ .tx_mask = slot_tx_mask,			\
				  .rx_mask = slot_rx_mask,			\
				  .slots = num_slot,				\
				  .slot_width = width, }

static int merr_set_slot_and_format(struct snd_soc_dai *dai,
			struct mrfld_slot_info *slot_info, unsigned int fmt)
{
	int ret;

	ret = snd_soc_dai_set_tdm_slot(dai, slot_info->tx_mask,
		slot_info->rx_mask, slot_info->slots, slot_info->slot_width);
	if (ret < 0) {
		pr_err("can't set codec pcm format %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}
	return ret;
}

static int merr_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	int ret = 0;
	unsigned int fmt;
	struct mrfld_slot_info *info;
	struct snd_interval *rate =  hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);

	/* RT5648 slave Mode */
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
				| SND_SOC_DAIFMT_CBS_CFS;
	info = &MRFLD_CONFIG_SLOT(0x3, 0x3, 2, SNDRV_PCM_FORMAT_S24_LE);
	ret = merr_set_slot_and_format(rtd->cpu_dai, info, fmt);

	pr_debug("Invoked %s for dailink %s\n", __func__, rtd->dai_link->name);

	rate->min = rate->max = SNDRV_PCM_RATE_48000;
	channels->min = channels->max = 2;

	/* set SSP2 to 24-bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
					SNDRV_PCM_HW_PARAM_FIRST_MASK],
					SNDRV_PCM_FORMAT_S24_LE);
	return 0;
}

#define BT_DOMAIN_NB 0
#define BT_DOMAIN_WB 1
#define BT_DOMAIN_A2DP 2

static int mrfld_bt_fm_fixup(struct snd_soc_dai_link *dai_link, struct snd_soc_dai *dai)
{
	unsigned int fmt;
	unsigned int mask, reg_val;
	bool is_bt;
	u16 is_bt_wb;
	int ret;
	struct mrfld_slot_info *info;

	pr_debug("In %s\n", __func__);
	mask = (1 << fls(1)) - 1;
	reg_val = snd_soc_platform_read(dai->platform, SST_MUX_REG);
	mask = (1 << fls(1)) - 1;
	is_bt = (reg_val >> SST_BT_FM_MUX_SHIFT) & mask;
	is_bt_wb = (reg_val >> SST_BT_MODE_SHIFT) & mask;
	pr_debug("%s is_bt:%d , is_bt_wb:0x%x\n", __func__, is_bt, is_bt_wb);

	if (is_bt) {
		switch (is_bt_wb) {
		case BT_DOMAIN_WB:
			dai_link->params = &mrfld_rt5648_ssp1_bt_wb;
			info = &MRFLD_CONFIG_SLOT(0x01, 0x01, 1, SNDRV_PCM_FORMAT_S16_LE);
			break;
		case BT_DOMAIN_NB:
			dai_link->params = &mrfld_rt5648_ssp1_bt_nb;
			info = &MRFLD_CONFIG_SLOT(0x01, 0x01, 1, SNDRV_PCM_FORMAT_S16_LE);
			break;
		case BT_DOMAIN_A2DP:
			dai_link->params = &mrfld_rt5648_ssp1_bt_a2dp;
			info = &MRFLD_CONFIG_SLOT(0x03, 0x00, 2, SNDRV_PCM_FORMAT_S16_LE);
			break;
		default:
			return -EINVAL;
		}

		fmt = SND_SOC_DAIFMT_IB_NF | SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBS_CFS;
	} else {
		fmt = SND_SOC_DAIFMT_IB_NF | SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_CBS_CFS;
		dai_link->params = &mrfld_rt5648_dai_params_ssp1_fm;
		info = &MRFLD_CONFIG_SLOT(0x00, 0x03, 2, SNDRV_PCM_FORMAT_S16_LE);
	}
	ret = merr_set_slot_and_format(dai, info, fmt);

	return ret;
}

static int mrfld_modem_fixup(struct snd_soc_dai_link *dai_link, struct snd_soc_dai *dai)
{
	int ret;
	unsigned int fmt;
	struct mrfld_slot_info *info;

	info = &MRFLD_CONFIG_SLOT(0x01, 0x01, 1, SNDRV_PCM_FORMAT_S16_LE);
	fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF
						| SND_SOC_DAIFMT_CBS_CFS;
	ret = merr_set_slot_and_format(dai, info, fmt);
	return ret;
}

static int mrfld_codec_loop_fixup(struct snd_soc_dai_link *dai_link, struct snd_soc_dai *dai)
{
	int ret;
	unsigned int fmt;
	struct mrfld_slot_info *info;

	info = &MRFLD_CONFIG_SLOT(0x3, 0x3, 2, SNDRV_PCM_FORMAT_S24_LE);
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	ret = merr_set_slot_and_format(dai, info, fmt);
	return ret;
}

enum gpios {
#ifdef PROJECT_Z300CL
	MRFLD_HSDET,
	MRFLD_HOOKDET,
	NUM_HS_GPIOS,
#else
	MRFLD_HSDET,
	NUM_HS_GPIOS,
#endif
};

#if 1
extern struct snd_soc_jack_gpio hs_gpio[];
#else
static int mrfld_jack_gpio_detect(void);
static int mrfld_jack_gpio_detect_bp(void);

static struct snd_soc_jack_gpio hs_gpio[] = {
	[MRFLD_HSDET] = {
		.name			= "JACK_DET_FILTR",
		.report			= SND_JACK_HEADSET,
		.debounce_time		= 50,
		.jack_status_check	= mrfld_jack_gpio_detect,
		.irq_flags		= IRQF_TRIGGER_FALLING |
					  IRQF_TRIGGER_RISING,
		.invert			= 1,
	},
	[MRFLD_HOOKDET] = {
		.name			= "HOOK_DET",
		.report			= SND_JACK_HEADSET | SND_JACK_BTN_0,
		.debounce_time		= 100,
		.jack_status_check	= mrfld_jack_gpio_detect_bp,
		.irq_flags		= IRQF_TRIGGER_FALLING |
					  IRQF_TRIGGER_RISING,
	},
};

static int mrfld_jack_gpio_detect_bp(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[MRFLD_HOOKDET];
	struct snd_soc_jack *jack = gpio->jack;
	/* struct snd_soc_codec *codec = jack->codec; */

	int enable, hs_status, status = 0;

	pr_debug("enter %s\n", __func__);
	status = jack->status;
	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;

	/* Check for headset status before processing interrupt */
	gpio = &hs_gpio[MRFLD_HSDET];
	hs_status = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		hs_status = !hs_status;
	pr_debug("in %s hook detect = 0x%x, headset detect = 0x%x\n",
			__func__, enable, hs_status);
	pr_debug("in %s jack status = 0x%x\n", __func__, jack->status);
	if (((jack->status & SND_JACK_HEADSET) == SND_JACK_HEADSET)
						&& (hs_status)) {
		if (enable)
			status = SND_JACK_HEADSET | SND_JACK_BTN_0;
		else
			status = SND_JACK_HEADSET;
	} else {
		pr_debug("%s:Spurious BP interrupt : jack_status 0x%x, HS_status 0x%x\n",
				__func__, jack->status, hs_status);
	}
	pr_debug("leave %s: status 0x%x\n", __func__, status);

	return status;
}

static int mrfld_jack_gpio_detect(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[MRFLD_HSDET];
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;

	int enable, status = 0;

	enable = gpio_get_value(gpio->gpio);
	if (gpio->invert)
		enable = !enable;
	pr_debug("%s:gpio->%d=0x%d\n", __func__, gpio->gpio, enable);

	if (enable) {
		status = rt5648_headset_detect(codec, 1);
		pr_debug("%s:headset insert, status %d\n", __func__, status);
	} else {
		status = rt5648_headset_detect(codec, 0);
		pr_debug("%s:headset removal, status %d\n", __func__, status);
	}

	return status;
}
#endif

static inline struct snd_soc_codec *mrfld_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "rt5648.1-001a")) {
			pr_debug("codec was %s", codec->name);
			continue;
		} else {
			pr_debug("found codec %s\n", codec->name);
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec", __func__);
		return NULL;
	}
	return codec;
}

static int mrfld_set_bias_level(struct snd_soc_card *card,
	struct snd_soc_dapm_context *dapm,
	enum snd_soc_bias_level level)
{
	struct snd_soc_dai *aif1_dai = find_codec_dai(card, "rt5648-aif1");
	int ret = 0;

	if (!aif1_dai)
		return -ENODEV;

	if (dapm->dev != aif1_dai->dev)
		return 0;
	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (card->dapm.bias_level == SND_SOC_BIAS_STANDBY)
			ret = mrfld_set_clk_fmt(aif1_dai);
		break;
	default:
		break;
	}
	pr_debug("%s card(%s)->bias_level %u\n", __func__, card->name,
		card->dapm.bias_level);
	return ret;
}

static int mrfld_set_bias_level_post(struct snd_soc_card *card,
				     struct snd_soc_dapm_context *dapm,
				     enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;
	struct mrfld_mc_private *ctx = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *aif1_dai = find_codec_dai(card, "rt5648-aif1");

	if (!aif1_dai)
		return -ENODEV;

	if (dapm->dev != aif1_dai->dev)
		return 0;

	codec = mrfld_get_codec(card);
	if (!codec)
		return -EIO;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		card->dapm.bias_level = level;
		break;
	case SND_SOC_BIAS_STANDBY:
		/* Processing already done during set_bias_level()
		 * callback. No action required here.
		 */
		/* Turn off 19.2MHz soc osc clock */
		if (card->dapm.bias_level == SND_SOC_BIAS_PREPARE)
			set_soc_osc_clk0(ctx->osc_clk0_reg, false);
		card->dapm.bias_level = level;
		break;
	case SND_SOC_BIAS_OFF:
		if (codec->dapm.bias_level != SND_SOC_BIAS_OFF)
			break;
		card->dapm.bias_level = level;
		break;
	default:
		return -EINVAL;
	}
	pr_debug("%s:card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return 0;
}

/* ALC5648 widgets */
static const struct snd_soc_dapm_widget widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

/* ALC5648 Audio Map */
static const struct snd_soc_dapm_route map[] = {
	/* {"micbias1", NULL, "Headset Mic"},
	{"IN1P", NULL, "micbias1"}, */
	{"IN1P", NULL, "Headset Mic"},
	{"DMIC1", NULL, "Int Mic"},
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},
	{"Ext Spk", NULL, "SPOL"},
	{"Ext Spk", NULL, "SPOR"},

	/* SWM map link the SWM outs to codec AIF */
	{ "AIF1 Playback", NULL, "ssp2 Tx"},
	{ "ssp2 Tx", NULL, "codec_out0"},
	{ "ssp2 Tx", NULL, "codec_out1"},
	{ "codec_in0", NULL, "ssp2 Rx" },
	{ "codec_in1", NULL, "ssp2 Rx" },
	{ "ssp2 Rx", NULL, "AIF1 Capture"},
	{ "pcm1_out", NULL, "Dummy Capture"},
	{ "pcm2_out", NULL, "Dummy Capture"},

	{ "ssp0 Tx", NULL, "modem_out"},
	{ "modem_in", NULL, "ssp0 Rx" },

	{ "ssp1 Tx", NULL, "bt_fm_out"},
	{ "bt_fm_in", NULL, "ssp1 Rx" },
};

static const struct snd_kcontrol_new ssp_comms_controls[] = {
		SOC_DAPM_PIN_SWITCH("Headphone"),
		SOC_DAPM_PIN_SWITCH("Headset Mic"),
		SOC_DAPM_PIN_SWITCH("Ext Spk"),
		SOC_DAPM_PIN_SWITCH("Int Mic"),
};

static int mrfld_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	unsigned int fmt;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_codec *codec = mrfld_get_codec(card);
	struct snd_soc_dai *aif1_dai = find_codec_dai(card, "rt5648-aif1");
	struct snd_soc_dapm_context *dapm =  &(codec->dapm);
	struct mrfld_mc_private *ctx = snd_soc_card_get_drvdata(card);

	if (!aif1_dai)
		return -ENODEV;

	pr_debug("Entry %s\n", __func__);

	/* RT5648 slave Mode */
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
				| SND_SOC_DAIFMT_CBS_CFS;
	ret = snd_soc_dai_set_fmt(aif1_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	mrfld_set_bias_level_post(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	/* FIXME
	 * set all the nc_pins, set all the init control
	 * and add any machine controls here
	 */

	pr_info("%s: card name: %s, snd_card: %s, codec: %p\n",
			__func__, codec->card->name, codec->card->snd_card->longname, codec);

	ctx->jack_retry = 0;
	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			       SND_JACK_HEADSET | SND_JACK_HEADPHONE |
			       SND_JACK_BTN_0,
			       &ctx->jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}

	pr_info("%s: hs_gpio array size %d\n", __func__, NUM_HS_GPIOS);
	ret = snd_soc_jack_add_gpios(&ctx->jack, NUM_HS_GPIOS, hs_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}
	snd_jack_set_key(ctx->jack.jack, SND_JACK_BTN_0, KEY_MEDIA);

	ret = snd_soc_add_card_controls(card, ssp_comms_controls,
				ARRAY_SIZE(ssp_comms_controls));
	if (ret) {
		pr_err("Add Comms Controls failed %d",
				ret);
		return ret;
	}

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "HPOL");
	snd_soc_dapm_ignore_suspend(dapm, "HPOR");

	snd_soc_dapm_ignore_suspend(dapm, "SPOL");
	snd_soc_dapm_ignore_suspend(dapm, "SPOR");

	snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(dapm, "Headphone");
	snd_soc_dapm_enable_pin(dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(dapm, "Int Mic");

	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);

	return 0;
}

static unsigned int rates_8000_16000[] = {
	8000,
	16000,
};

static struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list  = rates_8000_16000,
};

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int mrfld_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops mrfld_ops = {
	.startup = mrfld_startup,
	.hw_params = mrfld_hw_params,
};

static int mrfld_8k_16k_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_8000_16000);
}

static struct snd_soc_ops mrfld_8k_16k_ops = {
	.startup = mrfld_8k_16k_startup,
	.hw_params = mrfld_hw_params,
};

static struct snd_soc_ops mrfld_be_ssp2_ops = {
	.hw_params = mrfld_hw_params,
};
static struct snd_soc_compr_ops mrfld_compr_ops = {
	.set_params = mrfld_compr_set_params,
};

struct snd_soc_dai_link mrfld_rt5648_msic_dailink[] = {
	[MERR_DPCM_AUDIO] = {
		.name = "Merrifield Audio Port",
		.stream_name = "Saltbay Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.init = mrfld_init,
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &mrfld_ops,
	},
	[MERR_DPCM_DB] = {
		.name = "Merrifield DB Audio Port",
		.stream_name = "Deep Buffer Audio",
		.cpu_dai_name = "Deepbuffer-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &mrfld_ops,
	},
	[MERR_DPCM_LL] = {
		.name = "Merrifield LL Audio Port",
		.stream_name = "Low Latency Audio",
		.cpu_dai_name = "Lowlatency-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &mrfld_ops,
	},
	[MERR_DPCM_COMPR] = {
		.name = "Merrifield Compress Port",
		.stream_name = "Saltbay Compress",
		.platform_name = "sst-platform",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.compr_ops = &mrfld_compr_ops,
	},
	[MERR_DPCM_VOIP] = {
		.name = "Merrifield VOIP Port",
		.stream_name = "Saltbay Voip",
		.cpu_dai_name = "Voip-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &mrfld_8k_16k_ops,
		.dynamic = 1,
	},
	[MERR_DPCM_PROBE] = {
		.name = "Merrifield Probe Port",
		.stream_name = "Saltbay Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.playback_count = 8,
		.capture_count = 8,
	},
	/* CODEC<->CODEC link */
	{
		.name = "Merrifield Codec-Loop Port",
		.stream_name = "Saltbay Codec-Loop",
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "rt5648-aif1",
		.codec_name = "rt5648.1-001a",
		.params = &mrfld_rt5648_dai_params_ssp2,
		.be_fixup = mrfld_codec_loop_fixup,
		.dsp_loopback = true,
	},
	{
		.name = "Merrifield Modem-Loop Port",
		.stream_name = "Saltbay Modem-Loop",
		.cpu_dai_name = "ssp0-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &mrfld_rt5648_dai_params_ssp0,
		.be_fixup = mrfld_modem_fixup,
		.dsp_loopback = true,
	},
	{
		.name = "Merrifield BTFM-Loop Port",
		.stream_name = "Saltbay BTFM-Loop",
		.cpu_dai_name = "ssp1-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &mrfld_rt5648_ssp1_bt_nb,
		.be_fixup = mrfld_bt_fm_fixup,
		.dsp_loopback = true,
	},

	/* back ends */
	{
		.name = "SSP2-Codec",
		.be_id = 1,
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.no_pcm = 1,
		.codec_dai_name = "rt5648-aif1",
		.codec_name = "rt5648.1-001a",
		.be_hw_params_fixup = merr_codec_fixup,
		.ignore_suspend = 1,
		.ops = &mrfld_be_ssp2_ops,
	},
	{
		.name = "SSP1-BTFM",
		.be_id = 2,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
	{
		.name = "SSP0-Modem",
		.be_id = 3,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_mrfld_prepare(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}

static void snd_mrfld_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
}

static int snd_mrfld_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}
#else
#define snd_mrfld_prepare NULL
#define snd_mrfld_complete NULL
#define snd_mrfld_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_mrfld = {
	.name = "rt5648-audio",
	.dai_link = mrfld_rt5648_msic_dailink,
	.num_links = ARRAY_SIZE(mrfld_rt5648_msic_dailink),
	.set_bias_level = mrfld_set_bias_level,
	.set_bias_level_post = mrfld_set_bias_level_post,
	.dapm_widgets = widgets,
	.num_dapm_widgets = ARRAY_SIZE(widgets),
	.dapm_routes = map,
	.num_dapm_routes = ARRAY_SIZE(map),
};

static int snd_mrfld_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct mrfld_mc_private *drv;
	struct mrfld_audio_platform_data *pdata;
#ifndef UART_DEBUG
	/* static int hp_enable; */
#endif

	pr_debug("Entry %s\n", __func__);

	drv = kzalloc(sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
	pdata = pdev->dev.platform_data;
	/*TODO: remove hardcode before IFWI ready */
#if 0 /* Set headset gpio by asustek_headset driver */
#if 0 /* Mofd VV board */
	hs_gpio[MRFLD_HSDET].gpio = 48;
	hs_gpio[MRFLD_HOOKDET].gpio = 49;
#else /* EVB */
	hs_gpio[MRFLD_HSDET].gpio = 58;
	hs_gpio[MRFLD_HOOKDET].gpio = 57;
#endif
#endif
	/* ioremap the register */
	drv->osc_clk0_reg = devm_ioremap_nocache(&pdev->dev,
					MERR_OSC_CLKOUT_CTRL0_REG_ADDR,
					MERR_OSC_CLKOUT_CTRL0_REG_SIZE);
	if (!drv->osc_clk0_reg) {
		pr_err("osc clk0 ctrl ioremap failed\n");
		ret_val = -1;
		goto unalloc;
	}

	/* register the soc card */
	snd_soc_card_mrfld.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_mrfld, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_mrfld);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, &snd_soc_card_mrfld);
	return ret_val;

unalloc:
	kfree(drv);
	return ret_val;
}

static int snd_mrfld_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mrfld_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	snd_soc_jack_free_gpios(&drv->jack, 1, hs_gpio);
	kfree(drv);
	pr_debug("In %s\n", __func__);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void snd_mrfld_mc_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mrfld_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	snd_soc_jack_free_gpios(&drv->jack, 1, hs_gpio);
	pr_debug("In %s\n", __func__);
}

const struct dev_pm_ops snd_mrfld_rt5648_mc_pm_ops = {
	.prepare = snd_mrfld_prepare,
	.complete = snd_mrfld_complete,
	.poweroff = snd_mrfld_poweroff,
};

static struct platform_driver snd_mrfld_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mrfld_rt5648",
		.pm = &snd_mrfld_rt5648_mc_pm_ops,
	},
	.probe = snd_mrfld_mc_probe,
	.remove = snd_mrfld_mc_remove,
	.shutdown = snd_mrfld_mc_shutdown,
};

static int snd_mrfld_driver_init(void)
{
	pr_info("Merrifield Machine Driver mrfld_rt5648 registerd\n");
	return platform_driver_register(&snd_mrfld_mc_driver);
}

static void snd_mrfld_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_mrfld_mc_driver);
}

static int snd_mrfld_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	pr_info("In %s\n", __func__);

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed snd_mrfld rpmsg device\n");

	ret = snd_mrfld_driver_init();

out:
	return ret;
}

static void snd_mrfld_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	snd_mrfld_driver_exit();
	dev_info(&rpdev->dev, "Removed snd_mrfld rpmsg device\n");
}

static void snd_mrfld_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
				int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);
}

static struct rpmsg_device_id snd_mrfld_rpmsg_id_table[] = {
	{ .name = "rpmsg_mrfld_rt5648_audio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, snd_mrfld_rpmsg_id_table);

static struct rpmsg_driver snd_mrfld_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= snd_mrfld_rpmsg_id_table,
	.probe		= snd_mrfld_rpmsg_probe,
	.callback	= snd_mrfld_rpmsg_cb,
	.remove		= snd_mrfld_rpmsg_remove,
};

static int __init snd_mrfld_rpmsg_init(void)
{
	return register_rpmsg_driver(&snd_mrfld_rpmsg);
}
late_initcall(snd_mrfld_rpmsg_init);

static void __exit snd_mrfld_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&snd_mrfld_rpmsg);
}
module_exit(snd_mrfld_rpmsg_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Merrifield MID Machine driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mrfld-audio");
