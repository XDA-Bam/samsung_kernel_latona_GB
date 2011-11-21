/*
 * sdp4430.c  --  SoC audio for TI OMAP4430 SDP
 *
 * Author: Misael Lopez Cruz <x0052729@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mfd/twl6040-codec.h>
#include <linux/i2c/twl.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>

#include <asm/mach-types.h>
#include <plat/hardware.h>
#include <plat/mux.h>

#include "omap-mcpdm.h"
#include "omap-abe.h"
#include "omap-pcm.h"
#include "omap-mcbsp.h"
#include "omap-dmic.h"
#include "../codecs/twl6040.h"

#ifdef CONFIG_SND_OMAP_SOC_HDMI
#include "omap-hdmi.h"
#endif

#define TPS6130X_I2C_ADAPTER	1

static int twl6040_power_mode;
static int mcbsp_cfg;

static struct i2c_client *tps6130x_client;
static struct i2c_board_info tps6130x_hwmon_info = {
	I2C_BOARD_INFO("tps6130x", 0x33),
};

/* configure the TPS6130x Handsfree Boost Converter */
static int sdp4430_tps6130x_configure(void)
{
	u8 data[2];

	data[0] = 0x01;
	data[1] = 0x60;
	if (i2c_master_send(tps6130x_client, data, 2) != 2)
		printk(KERN_ERR "I2C write to TPS6130x failed\n");

	data[0] = 0x02;
	if (i2c_master_send(tps6130x_client, data, 2) != 2)
		printk(KERN_ERR "I2C write to TPS6130x failed\n");

	return 0;
}

static int sdp4430_modem_mcbsp_configure(struct snd_pcm_substream *substream,
	int flag)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_substream *modem_substream[2];
	struct snd_soc_pcm_runtime *modem_rtd;

	if (flag) {
		if (!mcbsp_cfg) {
			modem_substream[substream->stream] =
				snd_soc_get_dai_substream(rtd->card,
							OMAP_ABE_BE_MM_EXT1,
							substream->stream);
			if (unlikely(modem_substream[substream->stream] == NULL))
				return -ENODEV;

			modem_rtd =
				modem_substream[substream->stream]->private_data;

			/* Set cpu DAI configuration */
			ret = snd_soc_dai_set_fmt(modem_rtd->cpu_dai,
					  SND_SOC_DAIFMT_I2S |
					  SND_SOC_DAIFMT_NB_NF |
					  SND_SOC_DAIFMT_CBM_CFM);

			if (unlikely(ret < 0)) {
				printk(KERN_ERR "can't set Modem cpu DAI configuration\n");
				goto exit;
			} else {
				mcbsp_cfg = 1;
			}
		}
	} else {
		mcbsp_cfg = 0;
	}

exit:
	return ret;
}

static int sdp4430_mcpdm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int clk_id, freq;
	int ret = 0;

	if (twl6040_power_mode) {
		clk_id = TWL6040_SYSCLK_SEL_HPPLL;
		freq = 38400000;
	} else {
		clk_id = TWL6040_SYSCLK_SEL_LPPLL;
		freq = 32768;
	}

	/* set the codec mclk */
	ret = snd_soc_dai_set_sysclk(codec_dai, clk_id, freq,
				SND_SOC_CLOCK_IN);
	if (ret) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	if (rtd->current_fe == ABE_FRONTEND_DAI_MODEM) {
		/* set Modem McBSP configuration  */
		ret = sdp4430_modem_mcbsp_configure(substream, 1);
	}

	return ret;
}

static int sdp4430_mcpdm_hw_free(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	if (rtd->current_fe == ABE_FRONTEND_DAI_MODEM) {
		/* freed Modem McBSP configuration */
		ret = sdp4430_modem_mcbsp_configure(substream, 0);
	}

	return ret;
}

static struct snd_soc_ops sdp4430_mcpdm_ops = {
	.hw_params = sdp4430_mcpdm_hw_params,
	.hw_free = sdp4430_mcpdm_hw_free,
};

static int sdp4430_mcbsp_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned int be_id;

        be_id = rtd->dai_link->be_id;

	if (be_id == OMAP_ABE_DAI_MM_FM) {
		/* Set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	} else if (be_id == OMAP_ABE_DAI_BT_VX) {
	        ret = snd_soc_dai_set_fmt(cpu_dai,
                                  SND_SOC_DAIFMT_DSP_B |
                                  SND_SOC_DAIFMT_NB_IF |
                                  SND_SOC_DAIFMT_CBM_CFM);
	}

	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/*
	 * TODO: where does this clock come from (external source??) -
	 * do we need to enable it.
	 */
	/* Set McBSP clock to external */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK,
				     64 * params_rate(params),
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu system clock\n");
		return ret;
	}
	return 0;
}

static struct snd_soc_ops sdp4430_mcbsp_ops = {
	.hw_params = sdp4430_mcbsp_hw_params,
};

static int sdp4430_dmic_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;

	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_DMIC_SYSCLK_PAD_CLKS,
				     19200000, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set DMIC cpu system clock\n");
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_DMIC_CLKDIV, 8);
	if (ret < 0) {
		printk(KERN_ERR "can't set DMIC cpu clock divider\n");
		return ret;
	}

	if (rtd->current_fe == ABE_FRONTEND_DAI_MODEM) {
		/* set Modem McBSP configuration  */
		ret = sdp4430_modem_mcbsp_configure(substream, 1);
	}

	return ret;
}

static int sdp4430_dmic_hw_free(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	if (rtd->current_fe == ABE_FRONTEND_DAI_MODEM) {
		/* freed Modem McBSP configuration */
		ret = sdp4430_modem_mcbsp_configure(substream, 0);
	}

	return ret;
}

static struct snd_soc_ops sdp4430_dmic_ops = {
	.hw_params = sdp4430_dmic_hw_params,
	.hw_free = sdp4430_dmic_hw_free,
};

static int mcbsp_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *channels = hw_param_interval(params,
                                       SNDRV_PCM_HW_PARAM_CHANNELS);
	unsigned int be_id;
	unsigned int val;

        be_id = rtd->dai_link->be_id;

	switch (be_id) {
	case OMAP_ABE_DAI_MM_FM:
		channels->min = 2;
		val = SNDRV_PCM_FORMAT_S32_LE;
		break;
	case OMAP_ABE_DAI_BT_VX:
		channels->min = 1;
		val = SNDRV_PCM_FORMAT_S16_LE;
		break;
	default:
		val = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}

	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
		     val);
	return 0;
}

static int dmic_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
	                            SNDRV_PCM_HW_PARAM_FIRST_MASK],
	                            SNDRV_PCM_FORMAT_S32_LE);
	return 0;
}

/* Headset jack */
static struct snd_soc_jack hs_jack;

/*Headset jack detection DAPM pins */
static struct snd_soc_jack_pin hs_jack_pins[] = {
	{
		.pin = "Headset Mic",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headset Stereophone",
		.mask = SND_JACK_HEADPHONE,
	},
};

static int sdp4430_get_power_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = twl6040_power_mode;
	return 0;
}

static int sdp4430_set_power_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	if (twl6040_power_mode == ucontrol->value.integer.value[0])
		return 0;

	twl6040_power_mode = ucontrol->value.integer.value[0];

	return 1;
}

static const char *power_texts[] = {"Low-Power", "High-Performance"};

static const struct soc_enum sdp4430_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, power_texts),
};

static const struct snd_kcontrol_new sdp4430_controls[] = {
	SOC_ENUM_EXT("TWL6040 Power Mode", sdp4430_enum[0],
		sdp4430_get_power_mode, sdp4430_set_power_mode),
};

/* SDP4430 machine DAPM */
static const struct snd_soc_dapm_widget sdp4430_twl6040_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_HP("Headset Stereophone", NULL),
	SND_SOC_DAPM_SPK("Earphone Spk", NULL),
	SND_SOC_DAPM_INPUT("Aux/FM Stereo In"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* External Mics: MAINMIC, SUBMIC with bias*/
	{"MAINMIC", NULL, "Main Mic Bias"},
	{"SUBMIC", NULL, "Main Mic Bias"},
	{"Main Mic Bias", NULL, "Ext Mic"},

	/* External Speakers: HFL, HFR */
	{"Ext Spk", NULL, "HFL"},
	{"Ext Spk", NULL, "HFR"},

	/* Headset Mic: HSMIC with bias */
	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Mic"},

	/* Headset Stereophone (Headphone): HSOL, HSOR */
	{"Headset Stereophone", NULL, "HSOL"},
	{"Headset Stereophone", NULL, "HSOR"},

	/* Earphone speaker */
	{"Earphone Spk", NULL, "EP"},

	/* Aux/FM Stereo In: AFML, AFMR */
	{"AFML", NULL, "Aux/FM Stereo In"},
	{"AFMR", NULL, "Aux/FM Stereo In"},
};

static int sdp4430_twl6040_init_hs(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret;

	/* Add SDP4430 specific controls */
	ret = snd_soc_add_controls(codec, sdp4430_controls,
				ARRAY_SIZE(sdp4430_controls));
	if (ret)
		return ret;

	/* Add SDP4430 specific widgets */
	ret = snd_soc_dapm_new_controls(codec->dapm, sdp4430_twl6040_dapm_widgets,
				ARRAY_SIZE(sdp4430_twl6040_dapm_widgets));
	if (ret)
		return ret;

	/* Set up SDP4430 specific audio path audio_map */
	snd_soc_dapm_add_routes(codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	/* SDP4430 connected pins */
	snd_soc_dapm_enable_pin(codec->dapm, "Ext Mic");
	snd_soc_dapm_enable_pin(codec->dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(codec->dapm, "AFML");
	snd_soc_dapm_enable_pin(codec->dapm, "AFMR");
	snd_soc_dapm_disable_pin(codec->dapm, "Headset Mic");
	snd_soc_dapm_disable_pin(codec->dapm, "Headset Stereophone");

	/* allow audio paths from the audio modem to run during suspend */
	snd_soc_dapm_ignore_suspend(codec, "Ext Mic");
	snd_soc_dapm_ignore_suspend(codec, "Ext Spk");
	snd_soc_dapm_ignore_suspend(codec, "AFML");
	snd_soc_dapm_ignore_suspend(codec, "AFMR");
	snd_soc_dapm_ignore_suspend(codec, "Headset Mic");
	snd_soc_dapm_ignore_suspend(codec, "Headset Stereophone");

	ret = snd_soc_dapm_sync(codec->dapm);
	if (ret)
		return ret;

	/*Headset jack detection */
	ret = snd_soc_jack_new(codec, "Headset Jack",
				SND_JACK_HEADSET, &hs_jack);
	if (ret)
		return ret;

	ret = snd_soc_jack_add_pins(&hs_jack, ARRAY_SIZE(hs_jack_pins),
				hs_jack_pins);

	if (machine_is_omap_4430sdp())
		twl6040_hs_jack_detect(codec, &hs_jack, SND_JACK_HEADSET);
	else
		snd_soc_jack_report(&hs_jack, SND_JACK_HEADSET, SND_JACK_HEADSET);

	/* wait 500 ms before switching of HS power */
	rtd->pmdown_time = 500;

	return ret;
}

static int sdp4430_twl6040_init_hf(struct snd_soc_pcm_runtime *rtd)
{
	/* wait 500 ms before switching of HF power */
	rtd->pmdown_time = 500;
	return 0;
}

/* TODO: make this a separate BT CODEC driver or DUMMY */
static struct snd_soc_dai_driver dai[] = {
{
	.name = "Bluetooth",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
					SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
/* TODO: make this a separate FM CODEC driver or DUMMY */
{
	.name = "FM Digital",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "HDMI",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
},
};

static const char *mm1_be[] = {
		OMAP_ABE_BE_PDM_DL1,
		OMAP_ABE_BE_PDM_UL1,
		OMAP_ABE_BE_PDM_DL2,
		OMAP_ABE_BE_BT_VX,
		OMAP_ABE_BE_MM_EXT0,
		OMAP_ABE_BE_DMIC0,
		OMAP_ABE_BE_DMIC1,
		OMAP_ABE_BE_DMIC2,
};

static const char *mm2_be[] = {
		OMAP_ABE_BE_PDM_UL1,
		OMAP_ABE_BE_BT_VX,
		OMAP_ABE_BE_MM_EXT0,
		OMAP_ABE_BE_DMIC0,
		OMAP_ABE_BE_DMIC1,
		OMAP_ABE_BE_DMIC2,
};

static const char *tones_be[] = {
		OMAP_ABE_BE_PDM_DL1,
		OMAP_ABE_BE_PDM_DL2,
		OMAP_ABE_BE_BT_VX,
		OMAP_ABE_BE_MM_EXT0,
};

static const char *vib_be[] = {
		OMAP_ABE_BE_PDM_VIB,
};

static const char *modem_be[] = {
		OMAP_ABE_BE_PDM_DL1,
		OMAP_ABE_BE_PDM_UL1,
		OMAP_ABE_BE_PDM_DL2,
		OMAP_ABE_BE_BT_VX,
		OMAP_ABE_BE_DMIC0,
		OMAP_ABE_BE_DMIC1,
		OMAP_ABE_BE_DMIC2,
};

static const char *mm_lp_be[] = {
		OMAP_ABE_BE_PDM_DL1,
		OMAP_ABE_BE_PDM_DL2,
		OMAP_ABE_BE_BT_VX,
		OMAP_ABE_BE_MM_EXT0,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link sdp4430_dai[] = {

/*
 * Frontend DAIs - i.e. userspace visible interfaces (ALSA PCMs)
 */

	{
		.name = "SDP4430 Media",
		.stream_name = "Multimedia",

		/* ABE components - MM-UL & MM_DL */
		.cpu_dai_name = "MultiMedia1",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = mm1_be,
		.num_be = ARRAY_SIZE(mm1_be),
		.fe_playback_channels = 2,
		.fe_capture_channels = 8,
		.no_host_mode = SND_SOC_DAI_LINK_OPT_HOST,
	},
	{
		.name = "SDP4430 Media Capture",
		.stream_name = "Multimedia Capture",

		/* ABE components - MM-UL2 */
		.cpu_dai_name = "MultiMedia2",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = mm2_be,
		.num_be = ARRAY_SIZE(mm2_be),
		.fe_capture_channels = 2,
	},
	{
		.name = "SDP4430 Voice",
		.stream_name = "Voice",

		/* ABE components - VX-UL & VX-DL */
		.cpu_dai_name = "Voice",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = mm1_be,
		.num_be = ARRAY_SIZE(mm1_be),
		.fe_playback_channels = 2,
		.fe_capture_channels = 2,
		.no_host_mode = SND_SOC_DAI_LINK_OPT_HOST,
	},
	{
		.name = "SDP4430 Tones Playback",
		.stream_name = "Tone Playback",

		/* ABE components - TONES_DL */
		.cpu_dai_name = "Tones",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = tones_be,
		.num_be = ARRAY_SIZE(tones_be),
		.fe_playback_channels = 2,
	},
	{
		.name = "SDP4430 Vibra Playback",
		.stream_name = "VIB-DL",

		.cpu_dai_name = "Vibra",
		.platform_name = "omap-pcm-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = vib_be,
		.num_be = ARRAY_SIZE(vib_be),
		.fe_playback_channels = 2,
	},
	{
		.name = "SDP4430 MODEM",
		.stream_name = "MODEM",

		/* ABE components - MODEM <-> McBSP2 */
		.cpu_dai_name = "MODEM",
		.platform_name = "omap-aess-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = modem_be,
		.num_be = ARRAY_SIZE(modem_be),
		.fe_playback_channels = 2,
		.fe_capture_channels = 2,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
	},
	{
		.name = "SDP4430 Media LP",
		.stream_name = "Multimedia",

		/* ABE components - MM-DL (mmap) */
		.cpu_dai_name = "MultiMedia1 LP",
		.platform_name = "omap-aess-audio",

		.dynamic = 1, /* BE is dynamic */
		.supported_be = mm_lp_be,
		.num_be = ARRAY_SIZE(mm_lp_be),
		.fe_playback_channels = 2,
		.no_host_mode = SND_SOC_DAI_LINK_OPT_HOST,
	},
#ifdef CONFIG_SND_OMAP_SOC_HDMI
	{
		.name = "hdmi",
		.stream_name = "HDMI",

		.cpu_dai_name = "hdmi-dai",
		.platform_name = "omap-pcm-audio",

		/* HDMI*/
		.codec_dai_name = "HDMI",

		.no_codec = 1,
	},
#endif
	{
		.name = "Legacy McBSP",
		.stream_name = "Multimedia",

		/* ABE components - MCBSP2 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.platform_name = "omap-pcm-audio",

		/* FM */
		.codec_dai_name = "FM Digital",

		.no_codec = 1, /* TODO: have a dummy CODEC */
		.ops = &sdp4430_mcbsp_ops,
	},
	{
		.name = "Legacy McPDM",
		.stream_name = "Headset Playback",

		/* ABE components - DL1 */
		.cpu_dai_name = "mcpdm-dl",
		.platform_name = "omap-pcm-audio",

		/* Phoenix - DL1 DAC */
		.codec_dai_name =  "twl6040-dl1",
		.codec_name = "twl6040-codec",

		.ops = &sdp4430_mcpdm_ops,
		.ignore_suspend = 1,
	},
	{
		.name = "Legacy DMIC",
		.stream_name = "DMIC Capture",

		/* ABE components - DMIC0 */
		.cpu_dai_name = "omap-dmic-dai-0",
		.platform_name = "omap-pcm-audio",

		/* DMIC codec */
		.codec_dai_name = "dmic-hifi",
		.codec_name = "dmic-codec.0",

		.ops = &sdp4430_dmic_ops,
		.ignore_suspend = 1,
	},

/*
 * Backend DAIs - i.e. dynamically matched interfaces, invisible to userspace.
 * Matched to above interfaces at runtime, based upon use case.
 */

	{
		.name = OMAP_ABE_BE_PDM_DL1,
		.stream_name = "HS Playback",

		/* ABE components - DL1 */
		.cpu_dai_name = "mcpdm-dl1",
		.platform_name = "omap-aess-audio",

		/* Phoenix - DL1 DAC */
		.codec_dai_name =  "twl6040-dl1",
		.codec_name = "twl6040-codec",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.init = sdp4430_twl6040_init_hs,
		.ops = &sdp4430_mcpdm_ops,
		.be_id = OMAP_ABE_DAI_PDM_DL1,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_PDM_UL1,
		.stream_name = "Analog Capture",

		/* ABE components - UL1 */
		.cpu_dai_name = "mcpdm-ul1",
		.platform_name = "omap-aess-audio",

		/* Phoenix - UL ADC */
		.codec_dai_name =  "twl6040-ul",
		.codec_name = "twl6040-codec",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.ops = &sdp4430_mcpdm_ops,
		.be_id = OMAP_ABE_DAI_PDM_UL,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_PDM_DL2,
		.stream_name = "HF Playback",

		/* ABE components - DL2 */
		.cpu_dai_name = "mcpdm-dl2",
		.platform_name = "omap-aess-audio",

		/* Phoenix - DL2 DAC */
		.codec_dai_name =  "twl6040-dl2",
		.codec_name = "twl6040-codec",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.init = sdp4430_twl6040_init_hf,
		.ops = &sdp4430_mcpdm_ops,
		.be_id = OMAP_ABE_DAI_PDM_DL2,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_PDM_VIB,
		.stream_name = "Vibra",

		/* ABE components - VIB1 DL */
		.cpu_dai_name = "mcpdm-vib",
		.platform_name = "omap-aess-audio",

		/* Phoenix - PDM to PWM */
		.codec_dai_name =  "twl6040-vib",
		.codec_name = "twl6040-codec",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.ops = &sdp4430_mcpdm_ops,
		.be_id = OMAP_ABE_DAI_PDM_VIB,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_BT_VX,
		.stream_name = "BT",

		/* ABE components - MCBSP1 - BT-VX */
		.cpu_dai_name = "omap-mcbsp-dai.0",
		.platform_name = "omap-aess-audio",

		/* Bluetooth */
		.codec_dai_name = "Bluetooth",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.no_codec = 1, /* TODO: have a dummy CODEC */
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &sdp4430_mcbsp_ops,
		.be_id = OMAP_ABE_DAI_BT_VX,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_MM_EXT0,
		.stream_name = "FM",

		/* ABE components - MCBSP2 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.platform_name = "omap-aess-audio",

		/* FM */
		.codec_dai_name = "FM Digital",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.no_codec = 1, /* TODO: have a dummy CODEC */
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &sdp4430_mcbsp_ops,
		.be_id = OMAP_ABE_DAI_MM_FM,
	},
	{
		.name = OMAP_ABE_BE_MM_EXT1,
		.stream_name = "MODEM",

		/* ABE components - MCBSP2 - MM-EXT */
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.platform_name = "omap-aess-audio",

		/* MODEM */
		.codec_dai_name = "MODEM",

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.no_codec = 1, /* TODO: have a dummy CODEC */
		.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
		.ops = &sdp4430_mcbsp_ops,
		.be_id = OMAP_ABE_DAI_MODEM,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_DMIC0,
		.stream_name = "DMIC0",

		/* ABE components - DMIC UL 1 */
		.cpu_dai_name = "omap-dmic-abe-dai-0",
		.platform_name = "omap-aess-audio",

		/* DMIC 0 */
		.codec_dai_name = "dmic-hifi",
		.codec_name = "dmic-codec.0",
		.ops = &sdp4430_dmic_ops,

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = dmic_be_hw_params_fixup,
		.be_id = OMAP_ABE_DAI_DMIC0,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_DMIC1,
		.stream_name = "DMIC1",

		/* ABE components - DMIC UL 1 */
		.cpu_dai_name = "omap-dmic-abe-dai-1",
		.platform_name = "omap-aess-audio",

		/* DMIC 1 */
		.codec_dai_name = "dmic-hifi",
		.codec_name = "dmic-codec.1",
		.ops = &sdp4430_dmic_ops,

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = dmic_be_hw_params_fixup,
		.be_id = OMAP_ABE_DAI_DMIC1,
		.ignore_suspend = 1,
	},
	{
		.name = OMAP_ABE_BE_DMIC2,
		.stream_name = "DMIC2",

		/* ABE components - DMIC UL 2 */
		.cpu_dai_name = "omap-dmic-abe-dai-2",
		.platform_name = "omap-aess-audio",

		/* DMIC 2 */
		.codec_dai_name = "dmic-hifi",
		.codec_name = "dmic-codec.2",
		.ops = &sdp4430_dmic_ops,

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = dmic_be_hw_params_fixup,
		.be_id = OMAP_ABE_DAI_DMIC2,
		.ignore_suspend = 1,
	},
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_sdp4430 = {
	.name = "SDP4430",
	.long_name = "TI OMAP4 SDP4430 Board",
	.dai_link = sdp4430_dai,
	.num_links = ARRAY_SIZE(sdp4430_dai),
};

static struct platform_device *sdp4430_snd_device;

static int __init sdp4430_soc_init(void)
{
	struct i2c_adapter *adapter;
	u8 gpoctl = 0;
	int ret = 0;

	if (!machine_is_omap_4430sdp() && !machine_is_omap4_panda()) {
		pr_debug("Not SDP4430 or PandaBoard!\n");
		return -ENODEV;
	}
	printk(KERN_INFO "SDP4430 SoC init\n");

	sdp4430_snd_device = platform_device_alloc("soc-audio", -1);
	if (!sdp4430_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	snd_soc_register_dais(&sdp4430_snd_device->dev, dai, ARRAY_SIZE(dai));

	platform_set_drvdata(sdp4430_snd_device, &snd_soc_sdp4430);

	ret = platform_device_add(sdp4430_snd_device);
	if (ret)
		goto err;

	/* enable tps6130x */
	ret = twl_i2c_read_u8(TWL_MODULE_AUDIO_VOICE, &gpoctl,
			      TWL6040_REG_GPOCTL);
	if (ret)
		goto err;

	ret = twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, gpoctl | TWL6040_GPO2,
			       TWL6040_REG_GPOCTL);
	if (ret)
		goto err;

	adapter = i2c_get_adapter(TPS6130X_I2C_ADAPTER);
	if (!adapter) {
		printk(KERN_ERR "can't get i2c adapter\n");
		ret = -ENODEV;
		goto adp_err;
	}

	tps6130x_client = i2c_new_device(adapter, &tps6130x_hwmon_info);
	if (!tps6130x_client) {
		printk(KERN_ERR "can't add i2c device\n");
		ret = -ENODEV;
		goto tps_err;
	}

	/* Only configure the TPS6130x on SDP4430 */
	if (machine_is_omap_4430sdp())
		sdp4430_tps6130x_configure();

	return ret;

tps_err:
	i2c_put_adapter(adapter);
adp_err:
	twl_i2c_write_u8(TWL_MODULE_AUDIO_VOICE, gpoctl, TWL6040_REG_GPOCTL);
err:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(sdp4430_snd_device);
	return ret;
}
module_init(sdp4430_soc_init);

static void __exit sdp4430_soc_exit(void)
{
	platform_device_unregister(sdp4430_snd_device);
	i2c_unregister_device(tps6130x_client);
}
module_exit(sdp4430_soc_exit);

MODULE_AUTHOR("Misael Lopez Cruz <x0052729@ti.com>");
MODULE_DESCRIPTION("ALSA SoC SDP4430");
MODULE_LICENSE("GPL");

