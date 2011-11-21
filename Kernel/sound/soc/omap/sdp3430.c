/*
 * sdp3430.c  --  SoC audio for TI OMAP3430 SDP
 *
 * Author: Misael Lopez Cruz <x0052729@ti.com>
 *
 * Based on:
 * Author: Steve Sakoman <steve@sakoman.com>
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
#include <linux/i2c/twl.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mcbsp.h>
#include <plat/mux.h>

/* Register descriptions for twl4030 codec part */
#include <linux/mfd/twl4030-codec.h>

/* C2 State Patch */
#include <plat/omap-pm.h>  

#include "omap-mcbsp.h"
#include "omap-pcm.h"

/* TWL4030 PMBR1 Register */
#ifdef CONFIG_SND_SOC_MAX97000
#include "../codecs/max97000.h"
#elif CONFIG_SND_SOC_MAX9877
#include "../codecs/max9877.h"
#elif CONFIG_SND_SOC_YDA165
#include "../codecs/yda165.h"
#endif

#define ZEUS_PCM_SELECT_GPIO	OMAP_GPIO_PCM_SEL

#define TWL4030_INTBR_PMBR1		0x0D

/* TWL4030 PMBR1 Register GPIO6 mux bit */
#define TWL4030_GPIO6_PWM0_MUTE(value)	(value << 2)

//static struct snd_soc_card snd_soc_sdp3430;

static int sdp3430_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

int sdp3430_hw_free(struct snd_pcm_substream *substream) 
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;     
	int ret;        

	/* Use function clock for mcBSP2 */     

	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK, 
					0, SND_SOC_CLOCK_OUT);  

	return 0; 
}

static int snd_hw_latency;
extern void omap_dpll3_errat_wa(int disable);

int sdp3430_i2s_startup(struct snd_pcm_substream *substream)
{      
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	/*        
	  * Hold C2 as min latency constraint. Deeper states   
	  * MPU RET/OFF is overhead and consume more power than    
	  * savings.    
	  * snd_hw_latency check takes care of playback and capture      
	  * usecase.      
	  */     

	  if (!snd_hw_latency++) { 
	  	omap_pm_set_max_mpu_wakeup_lat(&substream->latency_pm_qos_req, 18);        

	  	/*           
	  	  * As of now for MP3 playback case need to enable dpll3 
	  	  * autoidle part of dpll3 lock errata.          
	  	  * REVISIT: Remove this, Once the dpll3 lock errata is     
	  	  * updated with with a new workaround without impacting mp3 usecase.          
	  	  */              

              printk("sdp3430_i2s_startup  \n");
	  	
	  	omap_dpll3_errat_wa(0);  
	  }      

	  return 0;
}

int sdp3430_i2s_shutdown(struct snd_pcm_substream *substream)
{    
	struct snd_soc_pcm_runtime *rtd = substream->private_data;	

	/* remove latency constraint */       
	snd_hw_latency--;     

	if (!snd_hw_latency) {   
		omap_pm_set_max_mpu_wakeup_lat(&substream->latency_pm_qos_req, -1);     
		printk("sdp3430_i2s_shutdown \n");
		omap_dpll3_errat_wa(1);    
	}   

	return 0;
}	
static struct snd_soc_ops sdp3430_ops = {
	.startup = sdp3430_i2s_startup,	
	.hw_params = sdp3430_hw_params,	
	.hw_free = sdp3430_hw_free,	
	.shutdown = sdp3430_i2s_shutdown,
};

static int sdp3430_hw_voice_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				SND_SOC_DAIFMT_DSP_A |
				SND_SOC_DAIFMT_IB_NF |
				SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_DSP_A |
				SND_SOC_DAIFMT_IB_NF |
				SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
					    SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops sdp3430_voice_ops = {
	.hw_params = sdp3430_hw_voice_params,
};

/* SDP3430 machine DAPM */
static const struct snd_soc_dapm_widget sdp3430_twl4030_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_HP("Headset Stereophone", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* External Mics: MAINMIC, SUBMIC with bias*/
	{"MAINMIC", NULL, "Mic Bias 1"},
	{"SUBMIC", NULL, "Mic Bias 2"},
	{"Mic Bias 1", NULL, "Ext Mic"},
	{"Mic Bias 2", NULL, "Ext Mic"},

	/* External Speakers: HFL, HFR */
	{"Ext Spk", NULL, "HFL"},
	{"Ext Spk", NULL, "HFR"},

	/* Headset Mic: HSMIC with bias */
	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Mic"},

	/* Headset Stereophone (Headphone): HSOL, HSOR */
	{"Headset Stereophone", NULL, "HSOL"},
	{"Headset Stereophone", NULL, "HSOR"},
};

static int sdp3430_twl4030_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret;

	/* Add SDP3430 specific widgets */
	ret = snd_soc_dapm_new_controls(codec->dapm, sdp3430_twl4030_dapm_widgets,
				ARRAY_SIZE(sdp3430_twl4030_dapm_widgets));
	if (ret)
		return ret;

#ifdef CONFIG_SND_SOC_MAX97000
       /* add MAX97000 specific controls */
	ret = max97000_add_controls(codec);
#elif CONFIG_SND_SOC_MAX9877
	printk(KERN_NOTICE "MAX 9877 AMPLIFIER\n");
	ret = max9877_add_controls(codec);
#elif CONFIG_SND_SOC_YDA165
	printk(KERN_NOTICE "YDA165 AMPLIFIER\n");
	ret = yda165_add_controls(codec);
#endif

	if (ret)
	{
		printk( "********** max audio amp add controls , %d\n", ret);
	}

	/* Set up SDP3430 specific audio path audio_map */
	snd_soc_dapm_add_routes(codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	/* SDP3430 connected pins */
	snd_soc_dapm_enable_pin(codec->dapm, "Ext Mic");
	snd_soc_dapm_enable_pin(codec->dapm, "Ext Spk");
	snd_soc_dapm_disable_pin(codec->dapm, "Headset Mic");
	snd_soc_dapm_disable_pin(codec->dapm, "Headset Stereophone");
	snd_soc_dapm_enable_pin(codec->dapm, "EARPIECE"); // reciever
	snd_soc_dapm_enable_pin(codec->dapm, "PREDRIVEL");
	snd_soc_dapm_enable_pin(codec->dapm, "PREDRIVER");

	/* TWL4030 not connected pins */
	snd_soc_dapm_nc_pin(codec->dapm, "AUXL");
	snd_soc_dapm_nc_pin(codec->dapm, "AUXR");
	snd_soc_dapm_nc_pin(codec->dapm, "CARKITMIC");
	snd_soc_dapm_nc_pin(codec->dapm, "DIGIMIC0");
	snd_soc_dapm_nc_pin(codec->dapm, "DIGIMIC1");

	//snd_soc_dapm_nc_pin(codec->dapm, "OUTL");
	//snd_soc_dapm_nc_pin(codec->dapm, "OUTR");
	snd_soc_dapm_nc_pin(codec->dapm, "EARPIECE");
	snd_soc_dapm_nc_pin(codec->dapm, "PREDRIVEL");
	snd_soc_dapm_nc_pin(codec->dapm, "PREDRIVER");
	snd_soc_dapm_nc_pin(codec->dapm, "CARKITL");
	snd_soc_dapm_nc_pin(codec->dapm, "CARKITR");
	snd_soc_dapm_nc_pin(codec->dapm, "Ext Spk");

	ret = snd_soc_dapm_sync(codec->dapm);
	if (ret)
		return ret;

	if (gpio_request(ZEUS_PCM_SELECT_GPIO, "PCM_SEL") == 0)
	{
		gpio_direction_output(ZEUS_PCM_SELECT_GPIO, 0);
	}

	return ret;
}

static int sdp3430_twl4030_voice_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	unsigned short reg;

	/* Enable voice interface */
	reg = codec->driver->read(codec, TWL4030_REG_VOICE_IF);
	reg |= TWL4030_VIF_DIN_EN | TWL4030_VIF_DOUT_EN | TWL4030_VIF_EN;
	codec->driver->write(codec, TWL4030_REG_VOICE_IF, reg);

	return 0;
}


/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link sdp3430_dai[] = {
	{
		.name = "TWL4030 I2S",
		.stream_name = "TWL4030 Audio",
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.codec_dai_name = "twl4030-hifi",
		.platform_name = "omap-pcm-audio",
		.codec_name = "twl4030-codec",
		.init = sdp3430_twl4030_init,
		.ops = &sdp3430_ops,
	},
	{
		.name = "TWL4030 PCM",
		.stream_name = "TWL4030 Voice",
		.cpu_dai_name = "omap-mcbsp-dai.2",
		.codec_dai_name = "twl4030-voice",
		.platform_name = "omap-pcm-audio",
		.codec_name = "twl4030-codec",
		.init = sdp3430_twl4030_voice_init,
		.ops = &sdp3430_voice_ops,
	},
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_sdp3430 = {
	.name = "SDP3430",
	.long_name = "sdp3430 (twl4030)",
	.dai_link = sdp3430_dai,
	.num_links = ARRAY_SIZE(sdp3430_dai),
};

static struct platform_device *sdp3430_snd_device;

static int __init sdp3430_soc_init(void)
{
	int ret;
	u8 pin_mux;

    printk(KERN_EMERG " %s : %s : %i \n", __FILE__, __FUNCTION__, __LINE__);

	#if 0
	if (!machine_is_omap_3430sdp()) {
		pr_debug("Not SDP3430!\n");
		return -ENODEV;
	}
	#endif
	printk(KERN_INFO "SDP3430 SoC init\n");

	sdp3430_snd_device = platform_device_alloc("soc-audio", -1);
	if (!sdp3430_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(sdp3430_snd_device, &snd_soc_sdp3430);


	ret = platform_device_add(sdp3430_snd_device);
	if (ret)
		goto err1;
	else
		printk(KERN_EMERG "platform_device_add success\n");

	return 0;

err1:

	printk(KERN_EMERG " %s : %s : %i UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU\n", __FILE__, __FUNCTION__, __LINE__);
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(sdp3430_snd_device);

	return ret;
}
module_init(sdp3430_soc_init);

static void __exit sdp3430_soc_exit(void)
{
	gpio_free(ZEUS_PCM_SELECT_GPIO);
	platform_device_unregister(sdp3430_snd_device);
}
module_exit(sdp3430_soc_exit);

MODULE_AUTHOR("Misael Lopez Cruz <x0052729@ti.com>");
MODULE_DESCRIPTION("ALSA SoC SDP3430");
MODULE_LICENSE("GPL");
