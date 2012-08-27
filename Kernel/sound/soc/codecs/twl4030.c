/*
 * ALSA SoC TWL4030 codec driver
 *
 * Author:      Steve Sakoman, <steve@sakoman.com>
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <mach/gpio.h>
#include <plat/mux.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

 /* Register descriptions are here */
#include <linux/mfd/twl4030-codec.h>

#define SAMSUNG_CUSTOMISATION

#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
#define ENABLE_LINEOUT
#define SPK_LINE_OUT_SEL OMAP_GPIO_SPK_LINE_OUT

#endif

#ifdef SAMSUNG_CUSTOMISATION
#ifdef CONFIG_SND_SOC_MAX97000
#include "./max97000.h"
#elif CONFIG_SND_SOC_MAX9877
#include "./max9877.h"
#elif CONFIG_SND_SOC_YDA165
#include "./yda165.h"
#endif

#if SEC_AUDIO_DEBUG
#define P(format,...)\
 printk("[audio:%s]" format "\n", __func__, ## __VA_ARGS__);
#else
#define P(format,...)
#endif

#define USE_GPIO_MAIN_MIC_BIAS
#define VOICE_RECOGNITION
#define VOICE_IF_AP_MASTER
#include "sec_gain.h"


struct wake_lock T2_wakelock;
extern u32 hw_revision;//for detecting hw rev

struct delayed_work codec_control_work;
struct delayed_work codec_down_work;

/* Shadow register used by the audio driver */
#define TWL4030_REG_SW_SHADOW		0x4A
#define TWL4030_CACHEREGNUM	(TWL4030_REG_SW_SHADOW + 1)

/* TWL4030_REG_SW_SHADOW (0x4A) Fields */
#define TWL4030_HFL_EN			0x01
#define TWL4030_HFR_EN			0x02

#endif //SAMSUNG_CUSTOMISATION

/*
 * twl4030 register cache & default register settings
 */
// swin.kim modify register default value
#if 1
static const u8 twl4030_reg_new[TWL4030_CACHEREGNUM] = {
  0x00, /* this register not used		*/
  0x92, /* REG_CODEC_MODE		(0x1)	*/
  0xFF, /* REG_OPTION	        (0x2)   */
  0x00, /* REG_UNKNOWN		    (0x3)	*/
  0x00, /* REG_MICBIAS_CTL	    (0x4)	*/
  0x00, /* REG_ANAMICL		    (0x5)	*/
  0x00, /* REG_ANAMICR		    (0x6)	*/
  0x00, /* REG_AVADC_CTL		(0x7)	*/
  0x00, /* REG_ADCMICSEL		(0x8)	*/
  0x00, /* REG_DIGMIXING		(0x9)	*/
  0x0c, /* REG_ATXL1PGA		    (0xA)	*/
  0x0c, /* REG_ATXR1PGA		    (0xB)	*/
  0x08, /* REG_AVTXL2PGA		(0xC)	*/
  0x08, /* REG_AVTXR2PGA		(0xD)	*/
  0x01, /* REG_AUDIO_IF		    (0xE)	*/
  0xe1, /* REG_VOICE_IF		    (0xF)	*/
  0x3f, /* REG_ARXR1PGA		    (0x10)	*/
  0x3f, /* REG_ARXL1PGA		    (0x11)	*/
  0x3f, /* REG_ARXR2PGA		    (0x12)	*/
  0x3f, /* REG_ARXL2PGA		    (0x13)	*/
  0x2e, /* REG_VRXPGA           (0x14)  */
  0x00, /* REG_VSTPGA           (0x15)  */
  0x12, /* REG_VRX2ARXPGA		(0x16)	*/
  0x00, /* REG_AVDAC_CTL		(0x17)	*/
  0x00, /* REG_ARX2VTXPGA		(0x18)	*/
  0x32, /* REG_ARXL1_APGA_CTL	(0x19)	*/
  0x32, /* REG_ARXR1_APGA_CTL	(0x1A)	*/
  0x00, /* REG_ARXL2_APGA_CTL	(0x1B)	*/
  0x00, /* REG_ARXR2_APGA_CTL	(0x1C)	*/
  0x00, /* REG_ATX2ARXPGA		(0x1D)	*/
  0x00, /* REG_BT_IF            (0x1E)	*/
  0x55, /* REG_BTPGA            (0x1F)	*/
  0x00, /* REG_BTSTPGA		    (0x20)	*/
  0x00, /* REG_EAR_CTL		    (0x21)	*/
  0x00, /* REG_HS_SEL           (0x22)	*/
  0x00, /* REG_HS_GAIN_SET	    (0x23)	*/
  0x00, /* REG_HS_POPN_SET	    (0x24)	*/
  0x00, /* REG_PREDL_CTL		(0x25)	*/
  0x00, /* REG_PREDR_CTL		(0x26)	*/
  0x00, /* REG_PRECKL_CTL		(0x27)	*/
  0x00, /* REG_PRECKR_CTL		(0x28)	*/
  0x00, /* REG_HFL_CTL		    (0x29)	*/
  0x00, /* REG_HFR_CTL		    (0x2A)	*/
  0x05, /* REG_ALC_CTL		    (0x2B)	*/
  0x00, /* REG_ALC_SET1		    (0x2C)	*/
  0x00, /* REG_ALC_SET2		    (0x2D)	*/
  0x00, /* REG_BOOST_CTL		(0x2E)	*/
  0x00, /* REG_SOFTVOL_CTL	    (0x2F)	*/
  0x13, /* REG_DTMF_FREQSEL	    (0x30)	*/
  0x00, /* REG_DTMF_TONEXT1H	(0x31)	*/
  0x00, /* REG_DTMF_TONEXT1L	(0x32)	*/
  0x00, /* REG_DTMF_TONEXT2H	(0x33)	*/
  0x00, /* REG_DTMF_TONEXT2L	(0x34)	*/
  0x79, /* REG_DTMF_TONOFF	    (0x35)	*/
  0x11, /* REG_DTMF_WANONOFF	(0x36)	*/
  0x00, /* REG_I2S_RX_SCRAMBLE_H	(0x37)	*/
  0x00, /* REG_I2S_RX_SCRAMBLE_M	(0x38)	*/
  0x00, /* REG_I2S_RX_SCRAMBLE_L	(0x39)	*/
  0x16, /* REG_APLL_CTL		    (0x3A)	*/
  0x00, /* REG_DTMF_CTL		    (0x3B)	*/
  0x44, /* REG_DTMF_PGA_CTL2	(0x3C)	*/
  0x69, /* REG_DTMF_PGA_CTL1	(0x3D)	*/
  0x02, /* REG_MISC_SET_1		(0x3E)	*/
  0x00, /* REG_PCMBTMUX		    (0x3F)	*/
  0x00, /* not used             (0x40)	*/
  0x00, /* not used             (0x41)	*/
  0x00, /* not used             (0x42)	*/
  0x20, /* REG_RX_PATH_SEL	    (0x43)	*/
  0x00, /* REG_VDL_APGA_CTL	    (0x44)	*/
  0x00, /* REG_VIBRA_CTL		(0x45)	*/
  0x00, /* REG_VIBRA_SET		(0x46)	*/
  0x00, /* REG_VIBRA_PWM_SET	(0x47)	*/
  0x00, /* REG_ANAMIC_GAIN	    (0x48)	*/
  0x00, /* REG_MISC_SET_2		(0x49)	*/
  0x00, /* REG_SW_SHADOW		(0x4A)	- Shadow, non HW register */
};
#endif

/*
 * twl4030 register cache & default register settings
 */
static const u8 twl4030_reg[TWL4030_CACHEREGNUM] = {
	0x00, /* this register not used		*/
	0x91, /* REG_CODEC_MODE		(0x1)	*/
	0xc3, /* REG_OPTION		(0x2)	*/
	0x00, /* REG_UNKNOWN		(0x3)	*/
	0x00, /* REG_MICBIAS_CTL	(0x4)	*/
	0x20, /* REG_ANAMICL		(0x5)	*/
	0x00, /* REG_ANAMICR		(0x6)	*/
	0x00, /* REG_AVADC_CTL		(0x7)	*/
	0x00, /* REG_ADCMICSEL		(0x8)	*/
	0x00, /* REG_DIGMIXING		(0x9)	*/
	0x0c, /* REG_ATXL1PGA		(0xA)	*/
	0x0c, /* REG_ATXR1PGA		(0xB)	*/
	0x00, /* REG_AVTXL2PGA		(0xC)	*/
	0x00, /* REG_AVTXR2PGA		(0xD)	*/
	0x01, /* REG_AUDIO_IF		(0xE)	*/
	0x04, /* REG_VOICE_IF		(0xF)	*/
	0x00, /* REG_ARXR1PGA		(0x10)	*/
	0x00, /* REG_ARXL1PGA		(0x11)	*/
	0x6c, /* REG_ARXR2PGA		(0x12)	*/
	0x6c, /* REG_ARXL2PGA		(0x13)	*/
	0x00, /* REG_VRXPGA		(0x14)	*/
	0x00, /* REG_VSTPGA		(0x15)	*/
	0x00, /* REG_VRX2ARXPGA		(0x16)	*/
	0x0c, /* REG_AVDAC_CTL		(0x17)	*/
	0x00, /* REG_ARX2VTXPGA		(0x18)	*/
	0x00, /* REG_ARXL1_APGA_CTL	(0x19)	*/
	0x00, /* REG_ARXR1_APGA_CTL	(0x1A)	*/
	0x4b, /* REG_ARXL2_APGA_CTL	(0x1B)	*/
	0x4b, /* REG_ARXR2_APGA_CTL	(0x1C)	*/
	0x00, /* REG_ATX2ARXPGA		(0x1D)	*/
	0x00, /* REG_BT_IF		(0x1E)	*/
	0x00, /* REG_BTPGA		(0x1F)	*/
	0x00, /* REG_BTSTPGA		(0x20)	*/
	0x00, /* REG_EAR_CTL		(0x21)	*/
	0x24, /* REG_HS_SEL		(0x22)	*/
	0x0a, /* REG_HS_GAIN_SET	(0x23)	*/
	0x00, /* REG_HS_POPN_SET	(0x24)	*/
	0x00, /* REG_PREDL_CTL		(0x25)	*/
	0x00, /* REG_PREDR_CTL		(0x26)	*/
	0x00, /* REG_PRECKL_CTL		(0x27)	*/
	0x00, /* REG_PRECKR_CTL		(0x28)	*/
	0x00, /* REG_HFL_CTL		(0x29)	*/
	0x00, /* REG_HFR_CTL		(0x2A)	*/
	0x00, /* REG_ALC_CTL		(0x2B)	*/
	0x00, /* REG_ALC_SET1		(0x2C)	*/
	0x00, /* REG_ALC_SET2		(0x2D)	*/
	0x00, /* REG_BOOST_CTL		(0x2E)	*/
	0x00, /* REG_SOFTVOL_CTL	(0x2F)	*/
	0x00, /* REG_DTMF_FREQSEL	(0x30)	*/
	0x00, /* REG_DTMF_TONEXT1H	(0x31)	*/
	0x00, /* REG_DTMF_TONEXT1L	(0x32)	*/
	0x00, /* REG_DTMF_TONEXT2H	(0x33)	*/
	0x00, /* REG_DTMF_TONEXT2L	(0x34)	*/
	0x00, /* REG_DTMF_TONOFF	(0x35)	*/
	0x00, /* REG_DTMF_WANONOFF	(0x36)	*/
	0x00, /* REG_I2S_RX_SCRAMBLE_H	(0x37)	*/
	0x00, /* REG_I2S_RX_SCRAMBLE_M	(0x38)	*/
	0x00, /* REG_I2S_RX_SCRAMBLE_L	(0x39)	*/
	0x16, /* REG_APLL_CTL		(0x3A)	*/
	0x00, /* REG_DTMF_CTL		(0x3B)	*/
	0x00, /* REG_DTMF_PGA_CTL2	(0x3C)	*/
	0x00, /* REG_DTMF_PGA_CTL1	(0x3D)	*/
	0x00, /* REG_MISC_SET_1		(0x3E)	*/
	0x00, /* REG_PCMBTMUX		(0x3F)	*/
	0x00, /* not used		(0x40)	*/
	0x00, /* not used		(0x41)	*/
	0x00, /* not used		(0x42)	*/
	0x00, /* REG_RX_PATH_SEL	(0x43)	*/
	0x00, /* REG_VDL_APGA_CTL	(0x44)	*/
	0x00, /* REG_VIBRA_CTL		(0x45)	*/
	0x00, /* REG_VIBRA_SET		(0x46)	*/
	0x00, /* REG_VIBRA_PWM_SET	(0x47)	*/
	0x00, /* REG_ANAMIC_GAIN	(0x48)	*/
	0x00, /* REG_MISC_SET_2		(0x49)	*/
	0x00, /* REG_SW_SHADOW		(0x4A)	- Shadow, non HW register */
};

#ifndef SAMSUNG_CUSTOMISATION
int twl4030_mode = 0;
int twl4030_vr_mode = false;
#endif

#ifdef SAMSUNG_CUSTOMISATION
int twl4030_mode = 0;
int twl4030_old_mode = 0;
int twl4030_remap = 0;
int twl4030_call_device = 0;
int twl4030_playback_device = 0;
bool twl4030_codec_suspended = false;
int twl4030_recording_device = 0;
int twl4030_voip_device = 0;
int twl4030_fm_device = 0;
int twl4030_mic_mute_enable = 0;
int twl4030_rec_8k_enable = 0;
int twl4030_fm_radio_mute_enable = 0;
int twl4030_vr_mode = false;
int is_read_gain = false;
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
int voip_main_mode = VOIP_MAIN_OFF;
#endif

static const char *audio_path[] = 		{"Playback Path", "Voice Call Path", "Memo Path", "VT Call Path", "VOIP Call Path",  "FM Radio Path",
													"Idle Mode","Mic Mute", "Loopback Path", "VR Mode"};
static const char *playback_path[]  = {"off","RCV","SPK","HP","SPK_HP", "EXTRA_DOCK_SPEAKER"};
static const char *voicecall_path[]=     	{"off","RCV","SPK","3HP","4HP", "BT"};
static const char *voicememo_path[] =	{"off","MAIN","SUB", "HP", "BT"};
static const char *vtcall_path[]=     	{"off","RCV","SPK","3HP","4HP", "BT"};
static const char *voip_path[]=             {"off","RCV","SPK","3HP","4HP", "BT"};
static const char *fmradio_path[]=        	{"off","SPK","HP"};
static const char *loopback_path[]=      {"off","RCV","SPK", "HP"};
static const char *idle_mode[]    = {"on","off"};
static const char *mic_mute[]    = {"off", "on"};   // hskwon-ss-db05, to support mic mute/unmute for CTS test
static const char *vr_mode[]    = {"off", "on"};
static const char *bt_dtmf_enum[]=      {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16"};

twl4030_codec_setting playback_off[]={
  {0x00, 0x00}
};
twl4030_codec_setting playback_rcv[]={
  {0x00, 0x00}
};
twl4030_codec_setting playback_spk[]={
  //  {0x0e, 0x01}, //TWL4030_REG_AUDIO_IF
  //  {0x3a, 0x16}, //TWL4030_REG_APLL_CTL
  {0x01, 0x93}, //TWL4030_REG_CODEC_MODE
  {0x13, 0x3f}, //TWL4030_REG_ARXL2PGA
  {0x12, 0x3f}, //TWL4030_REG_ARXR2PGA
  {0x17, 0x0c}, //TWL4030_REG_AVDAC_CTL
  {0x1b, 0x23}, //TWL4030_REG_ARXL2_APGA_CTL
  {0x1c, 0x23}, //TWL4030_REG_ARXR2_APGA_CTL
  {0x21, 0x00}, //TWL4030_REG_EAR_CTL
  {0x22, 0x00}, //TWL4030_REG_HS_SEL
  {0x26, 0x28}, //TWL4030_REG_PREDR_CTL
  {0x25, 0x28},  //TWL4030_REG_PREDL_CTL

  {0x43, 0x00}
};
twl4030_codec_setting playback_hp[]={
  //  {0x0e, 0x01}, //TWL4030_REG_AUDIO_IF
  //  {0x3a, 0x16}, //TWL4030_REG_APLL_CTL
  {0x01, 0x93}, //TWL4030_REG_CODEC_MODE
  {0x13, 0x3f}, //TWL4030_REG_ARXL2PGA
  {0x12, 0x3f}, //TWL4030_REG_ARXR2PGA
  {0x17, 0x0c}, //TWL4030_REG_AVDAC_CTL
  {0x1b, 0x23}, //TWL4030_REG_ARXL2_APGA_CTL
  {0x1c, 0x23}, //TWL4030_REG_ARXR2_APGA_CTL
  {0x21, 0x00}, //TWL4030_REG_EAR_CTL
  {0x23, 0x0A}, //TWL4030_REG_HS_GAIN_SET
  {0x24, 0x41}, //TWL4030_REG_HS_POPN_SET
  {0x24, 0x42}, //TWL4030_REG_HS_POPN_SET
  {0x22, 0x24},  //TWL4030_REG_HS_SEL
  {0x26, 0x00}, //TWL4030_REG_PREDR_CTL
  {0x25, 0x00},  //TWL4030_REG_PREDL_CTL

  {0x43, 0x00}
};
twl4030_codec_setting playback_spk_hp[]={
  //  {0x0e, 0x01}, //TWL4030_REG_AUDIO_IF
  //  {0x3a, 0x16}, //TWL4030_REG_APLL_CTL
  {0x01, 0x93}, //TWL4030_REG_CODEC_MODE
  {0x13, 0x3f}, //TWL4030_REG_ARXL2PGA
  {0x12, 0x3f}, //TWL4030_REG_ARXR2PGA
  {0x17, 0x0c}, //TWL4030_REG_AVDAC_CTL
  {0x1b, 0x23}, //TWL4030_REG_ARXL2_APGA_CTL
  {0x1c, 0x23}, //TWL4030_REG_ARXR2_APGA_CTL
  {0x21, 0x00}, //TWL4030_REG_EAR_CTL
  {0x22, 0x24}, //TWL4030_REG_HS_SEL
  {0x23, 0x0A}, //TWL4030_REG_HS_GAIN_SET
  {0x24, 0x41}, //TWL4030_REG_HS_POPN_SET
  {0x24, 0x42}, //TWL4030_REG_HS_POPN_SET
  {0x25, 0x28}, //TWL4030_REG_PREDL_CTL
  {0x26, 0x28}, //TWL4030_REG_PREDR_CTL

  {0x43, 0x00}
};


//voicecall_path {"off","RCV","SPK","HP","BT"};
twl4030_codec_setting voicecall_off[]={
  {0x1e, 0x00},  //TWL4030_REG_BT_IF
  {0x20, 0x00},  //TWL4030_REG_BTSTPGA
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x43, 0x20},  //TWL4030_REG_RX_PATH_SEL
  {0x17, 0x0c}   //TWL4030_REG_AVDAC_CTL
};

twl4030_codec_setting voicecall_rcv[]={
  {0x01, 0x92},  //TWL4030_REG_ANAMICR
  //{0x21, 0x34},  //TWL4030_REG_EAR_CTL
  {0x3a, 0x16},  //TWL4030_REG_APLL_CTL
  {0x09, 0x30},  //TWL4030_REG_DIGMIXING
  {0x25, 0x20},  //TWL4030_REG_PREDL_CTL
  {0x26, 0x20},  //TWL4030_REG_PREDR_CTL
  {0x0e, 0x01},  //TWL4030_REG_AUDIO_IF
  {0x0f, 0xe1},  //TWL4030_REG_VOICE_IF
  {0x14, 0x20},  //TWL4030_REG_VRXPGA
  {0x16, 0x19},  //TWL4030_REG_VRX2ARXPGA
  {0x17, 0x08},  //TWL4030_REG_AVDAC_CTL
  {0x1c, 0x1b},  //TWL4030_REG_ARXR2_APGA_CTL
  {0x1b, 0x1b},  //TWL4030_REG_ARXL2_APGA_CTL

  {0x13, 0x6f},  //TWL4030_REG_ARXL2PGA
  {0x12, 0x6f},  //TWL4030_REG_ARXR2PGA
  {0x48, 0x22},  //TWL4030_REG_ANAMIC_GAIN

  //reset other path
  {0x22, 0x00},  //TWL4030_REG_HS_SEL
  {0x23, 0x00},  //TWL4030_REG_HS_GAIN_SET
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x1e, 0x00},  //TWL4030_REG_BT_IF

  // main mic
	#ifdef USE_GPIO_MIC_SEL
  {0x04, 0x01},  //TWL4030_REG_MICBIAS_CTL
#else
  {0x04, 0x02},  //TWL4030_REG_MICBIAS_CTL
#endif


  {0x05, 0x11},  //TWL4030_REG_ANAMICL
  {0x02, 0xf5},  //TWL4030_REG_ANAMICR
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL

  {0x0c, 0x11},
};


twl4030_codec_setting voicecall_spk[] ={
  {0x01, 0x92},  //TWL4030_REG_CODEC_MODE
  {0x3a, 0x16},  //TWL4030_REG_APLL_CTL
  //{0x26, 0x28},  //TWL4030_REG_PREDR_CTL
  //{0x25, 0x24},  //TWL4030_REG_PREDL_CTL
  {0x09, 0x30},  //TWL4030_REG_DIGMIXING
  {0x13, 0x7f},  //TWL4030_REG_ARXL2PGA
  {0x12, 0x7f},  //TWL4030_REG_ARXR2PGA
  {0x17, 0x08},  //TWL4030_REG_AVDAC_CTL

  //reset other path
  {0x21, 0x00},  //TWL4030_REG_EAR_CTL
  {0x22, 0x00},  //TWL4030_REG_HS_SEL
  {0x23, 0x00},  //TWL4030_REG_HS_GAIN_SET
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x1e, 0x00},  //TWL4030_REG_BT_IF

  // main mic
	#ifdef USE_GPIO_MIC_SEL
  {0x04, 0x01},  //TWL4030_REG_MICBIAS_CTL
#else
  {0x04, 0x02},  //TWL4030_REG_MICBIAS_CTL
#endif

  {0x05, 0x11},  //TWL4030_REG_ANAMICL
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL
  {0x48, 0x24},  //TWL4030_REG_ANAMIC_GAIN

  {0x16, 0x17},
  {0x1b, 0x1b},
  {0x1c, 0x1b},
  {0x14, 0x29},
  {0x0c, 0x08},
};

twl4030_codec_setting voicecall_hp3p[]= {
  {0x01, 0x92},  //TWL4030_REG_CODEC_MODE
  {0x02, 0xf5},  //TWL4030_REG_OPTION
  {0x09, 0x30},  //TWL4030_REG_DIGMIXING
  {0x0e, 0x01},  //TWL4030_REG_AUDIO_IF
  {0x12, 0x00},  //TWL4030_REG_ARXR2PGA
  {0x13, 0x00},  //TWL4030_REG_ARXL2PGA
  {0x17, 0x0c},  //TWL4030_REG_AVDAC_CTL
  {0x1b, 0x1b}, //TWL4030_REG_ARXL2_APGA_CTL
  {0x1c, 0x1b},  //TWL4030_REG_ARXR2_APGA_CTL
  {0x22, 0x24},  //TWL4030_REG_HS_SEL
  {0x23, 0x0a}, //TWL4030_REG_HS_GAIN_SET
  {0x24, 0x41},  //TWL4030_REG_HS_POPN_SET
  {0x24, 0x42}, //TWL4030_REG_HS_POPN_SET
  {0x3a, 0x16},  //TWL4030_REG_APLL_CTL

  //reset other path
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x21, 0x00}, //TWL4030_REG_EAR_CTL
  {0x25, 0x00},  //TWL4030_REG_PREDL_CTL
  {0x26, 0x00},  //TWL4030_REG_PREDR_CTL
  {0x1e, 0x00},  //TWL4030_REG_BT_IF

  // main mic
  {0x48, 0x04},  //TWL4030_REG_ANAMIC_GAIN
	#ifdef USE_GPIO_MIC_SEL
  {0x04, 0x01},  //TWL4030_REG_MICBIAS_CTL
#else
  {0x04, 0x02},  //TWL4030_REG_MICBIAS_CTL
#endif

  {0x05, 0x11},  //TWL4030_REG_ANAMICL
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL

  {0x16, 0x17},
  {0x14, 0x2b},
  {0x0c, 0x08},
};


twl4030_codec_setting voicecall_hp4p[]= {
  {0x01, 0x92},  //TWL4030_REG_CODEC_MODE
  {0x02, 0xf5},  //TWL4030_REG_OPTION
  {0x09, 0x30},  //TWL4030_REG_DIGMIXING
  {0x0e, 0x01},  //TWL4030_REG_AUDIO_IF
  {0x12, 0x00},  //TWL4030_REG_ARXR2PGA
  {0x13, 0x00},  //TWL4030_REG_ARXL2PGA
  {0x17, 0x0c},  //TWL4030_REG_AVDAC_CTL
  {0x1b, 0x1b}, //TWL4030_REG_ARXL2_APGA_CTL
  {0x1c, 0x1b},  //TWL4030_REG_ARXR2_APGA_CTL
  {0x22, 0x24},  //TWL4030_REG_HS_SEL
  {0x23, 0x0a}, //TWL4030_REG_HS_GAIN_SET
  {0x24, 0x41},  //TWL4030_REG_HS_POPN_SET
  {0x24, 0x42}, //TWL4030_REG_HS_POPN_SET
  {0x3a, 0x16},  //TWL4030_REG_APLL_CTL

  //reset other path
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x25, 0x00},  //TWL4030_REG_PREDL_CTL
  {0x26, 0x00},  //TWL4030_REG_PREDR_CTL
  {0x21, 0x00}, //TWL4030_REG_EAR_CTL
  {0x1e, 0x00},  //TWL4030_REG_BT_IF

  //mic control
  {0x48, 0x04},  //TWL4030_REG_ANAMIC_GAIN
  {0x04, 0x04},  //TWL4030_REG_MICBIAS_CTL
  {0x05, 0x12},  //TWL4030_REG_ANAMICL
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL

  {0x16, 0x17},
  {0x14, 0x2b},
  {0x0c, 0x08},
};


twl4030_codec_setting voicecall_bt[]={
  //RX volume
  {0x1f, 0x5b},  //TWL4030_REG_BTPGA
  //output
  {0x17, 0x00},  //TWL4030_REG_AVDAC_CTL
  {0x43, 0x10},  //TWL4030_REG_RX_PATH_SEL
  {0x3f, 0xa0},  //TWL4030_REG_PCMBTMUX
  {0x1e, 0x61},  //TWL4030_REG_BT_IF
  //reset other path
  {0x22, 0x00},  //TWL4030_REG_HS_SEL
  {0x23, 0x00},  //TWL4030_REG_HS_GAIN_SET
  {0x25, 0x00},  //TWL4030_REG_PREDL_CTL
  {0x26, 0x00},  //TWL4030_REG_PREDR_CTL
  {0x21, 0x00}, //TWL4030_REG_EAR_CTL
  //input
  {0x04, 0x00},  //TWL4030_REG_MICBIAS_CTL
  {0x05, 0x00},  //TWL4030_REG_ANAMICL
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL
};


// hskwon-ss-cl31, added for FMC(VoIP) call path
//voipcall_path {"off","RCV","SPK","HP"};
twl4030_codec_setting voipcall_off[]={
  {0x1e, 0x00},  //TWL4030_REG_BT_IF
  {0x20, 0x00},  //TWL4030_REG_BTSTPGA
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x43, 0x20},  //TWL4030_REG_RX_PATH_SEL
  {0x17, 0x0c}   //TWL4030_REG_AVDAC_CTL
};

twl4030_codec_setting voipcall_rcv[]={
  {0x01, 0x93},  //TWL4030_REG_CODEC_MODE
  //{0x21, 0x34},  //TWL4030_REG_EAR_CTL
  {0x3a, 0x16},  //TWL4030_REG_APLL_CTL
  {0x25, 0x20},  //TWL4030_REG_PREDL_CTL
  {0x26, 0x20},  //TWL4030_REG_PREDR_CTL
  {0x0e, 0x01},  //TWL4030_REG_AUDIO_IF
  {0x17, 0x08},  //TWL4030_REG_AVDAC_CTL
  {0x1c, 0x13},  //TWL4030_REG_ARXR2_APGA_CTL
  {0x1b, 0x13},  //TWL4030_REG_ARXL2_APGA_CTL

  {0x13, 0xb9},  //TWL4030_REG_ARXL2PGA
  {0x12, 0xb9},  //TWL4030_REG_ARXR2PGA
  {0x48, 0x2D},  //TWL4030_REG_ANAMIC_GAIN

  //reset other path
  {0x22, 0x00},  //TWL4030_REG_HS_SEL
  {0x23, 0x00},  //TWL4030_REG_HS_GAIN_SET
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x1e, 0x00},  //TWL4030_REG_BT_IF

  // main mic
	#ifdef USE_GPIO_MIC_SEL
  {0x04, 0x01},  //TWL4030_REG_MICBIAS_CTL
#else
  {0x04, 0x02},  //TWL4030_REG_MICBIAS_CTL
#endif

  {0x05, 0x11},  //TWL4030_REG_ANAMICL
  {0x02, 0xf5},  //TWL4030_REG_OPTION
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL

  {0x0c, 0x06},  //TWL4030_REG_AVTXL2PGA
  {0x21, 0x34}   //TWL4030_REG_EAR_CTL
};

twl4030_codec_setting voipcall_spk[] ={
  {0x01, 0x93},  //TWL4030_REG_CODEC_MODE
  {0x3a, 0x16},  //TWL4030_REG_APLL_CTL
  {0x13, 0xbf},  //TWL4030_REG_ARXL2PGA
  {0x12, 0xbf},  //TWL4030_REG_ARXR2PGA
  {0x17, 0x08},  //TWL4030_REG_AVDAC_CTL

  //reset other path
  {0x21, 0x00},  //TWL4030_REG_EAR_CTL
  {0x22, 0x00},  //TWL4030_REG_HS_SEL
  {0x23, 0x00},  //TWL4030_REG_HS_GAIN_SET
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x1e, 0x00},  //TWL4030_REG_BT_IF

  // main mic
	#ifdef USE_GPIO_MIC_SEL
  {0x04, 0x01},  //TWL4030_REG_MICBIAS_CTL
#else
  {0x04, 0x02},  //TWL4030_REG_MICBIAS_CTL
#endif

  {0x05, 0x11},  //TWL4030_REG_ANAMICL
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL
  {0x48, 0x1B},  //TWL4030_REG_ANAMIC_GAIN

  {0x1b, 0x0B},
  {0x1c, 0x0B},
  {0x0c, 0x08},

  {0x26, 0x28},  //TWL4030_PREDR_CTL
  {0x25, 0x28}   //TWL4030_PREDL_CTL
};


twl4030_codec_setting voipcall_hp3p[]= {
  {0x01, 0x93},  //TWL4030_REG_CODEC_MODE
  {0x02, 0xf5},  //TWL4030_REG_OPTION
  {0x0e, 0x01},  //TWL4030_REG_AUDIO_IF
  {0x12, 0x3f},  //TWL4030_REG_ARXR2PGA
  {0x13, 0x3f},  //TWL4030_REG_ARXL2PGA
  {0x17, 0x0c},  //TWL4030_REG_AVDAC_CTL
  {0x1b, 0x13},  //TWL4030_REG_ARXL2_APGA_CTL
  {0x1c, 0x13},  //TWL4030_REG_ARXR2_APGA_CTL
  {0x22, 0x24},  //TWL4030_REG_HS_SEL
  {0x23, 0x0a},  //TWL4030_REG_HS_GAIN_SET
  {0x24, 0x41},  //TWL4030_REG_HS_POPN_SET
  {0x24, 0x42},  //TWL4030_REG_HS_POPN_SET
  {0x3a, 0x16},  //TWL4030_REG_APLL_CTL

  //reset other path
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x21, 0x00},  //TWL4030_REG_EAR_CTL
  {0x25, 0x00},  //TWL4030_REG_PREDL_CTL
  {0x26, 0x00},  //TWL4030_REG_PREDR_CTL
  {0x1e, 0x00},  //TWL4030_REG_BT_IF

  //mic control
	#ifdef USE_GPIO_MIC_SEL
  {0x04, 0x01},  //TWL4030_REG_MICBIAS_CTL
#else
  {0x04, 0x02},  //TWL4030_REG_MICBIAS_CTL
#endif

  {0x48, 0x24},  //TWL4030_REG_ANAMIC_GAIN
  {0x05, 0x11},  //TWL4030_REG_ANAMICL
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL

  {0x0c, 0x08},
};


twl4030_codec_setting voipcall_hp4p[]= {
  {0x01, 0x93},  //TWL4030_REG_CODEC_MODE
  {0x02, 0xf5},  //TWL4030_REG_OPTION
  {0x0e, 0x01},  //TWL4030_REG_AUDIO_IF
  {0x12, 0x3f},  //TWL4030_REG_ARXR2PGA
  {0x13, 0x3f},  //TWL4030_REG_ARXL2PGA
  {0x17, 0x0c},  //TWL4030_REG_AVDAC_CTL
  {0x1b, 0x13},  //TWL4030_REG_ARXL2_APGA_CTL
  {0x1c, 0x13},  //TWL4030_REG_ARXR2_APGA_CTL
  {0x22, 0x24},  //TWL4030_REG_HS_SEL
  {0x23, 0x0a},  //TWL4030_REG_HS_GAIN_SET
  {0x24, 0x41},  //TWL4030_REG_HS_POPN_SET
  {0x24, 0x42},  //TWL4030_REG_HS_POPN_SET
  {0x3a, 0x16},  //TWL4030_REG_APLL_CTL

  //reset other path
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x25, 0x00},  //TWL4030_REG_PREDL_CTL
  {0x26, 0x00},  //TWL4030_REG_PREDR_CTL
  {0x21, 0x00},  //TWL4030_REG_EAR_CTL
  {0x1e, 0x00},  //TWL4030_REG_BT_IF

  //mic control
  {0x48, 0x2D},  //TWL4030_REG_ANAMIC_GAIN
  {0x04, 0x04},  //TWL4030_REG_MICBIAS_CTL
  {0x05, 0x12},  //TWL4030_REG_ANAMICL
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL

  {0x0c, 0x06},
};
// hskwon-ss-cl31, added for FMC(VoIP) call path
twl4030_codec_setting fmradio_spk[] ={
	{0x01, 0x92},  //TWL4030_REG_CODEC_MODE
	{0x05,0x14},	//KHoAudioANAMICL mic ampL enable, AUXL enable
	{0x06,0x14},		//KHoAudioANAMICR mic ampR enable, AUXR enable
	{0x07,0x00},		//KHoAudioAVADC_CTL
	{0x08, 0x00},  //TWL4030_REG_ADCMICSEL
	{0x3e,0x20},		//KHoAudioMISC_SET_1
	{0x13, 0x7f},  //TWL4030_REG_ARXL2PGA
	{0x12, 0x7f},  //TWL4030_REG_ARXR2PGA
	{0x21, 0x00},  //TWL4030_REG_EAR_CTL
	{0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
	{0x1c, 0x07},  //TWL4030_REG_ARXR2_APGA_CTL
	{0x1b, 0x07},  //TWL4030_REG_ARXL2_APGA_CTL
	{0x26, 0X28},  //TWL4030_PREDR_CTL
      {0x25, 0x24}   //TWL4030_PREDL_CTL
};
twl4030_codec_setting fmradio_hp[]= {
	{0x01, 0x92},  //TWL4030_REG_CODEC_MODE
	{0x05,0x14},	//KHoAudioANAMICL mic ampL enable, AUXL enable
	{0x06,0x14},		//KHoAudioANAMICR mic ampR enable, AUXR enable
	{0x07,0x00},		//KHoAudioAVADC_CTL
	{0x08, 0x00},  //TWL4030_REG_ADCMICSEL
	{0x3e,0x20},		//KHoAudioMISC_SET_1
	{0x13, 0x7f},  //TWL4030_REG_ARXL2PGA
	{0x12, 0x7f},  //TWL4030_REG_ARXR2PGA
	{0x21, 0x00},  //TWL4030_REG_EAR_CTL
	{0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
	{0x1c, 0x07},  //TWL4030_REG_ARXR2_APGA_CTL
	{0x1b, 0x07},  //TWL4030_REG_ARXL2_APGA_CTL
	{0x22, 0x24},  //TWL4030_REG_HS_SEL
	{0x24, 0x41},  //TWL4030_REG_HS_POPN_SET
	{0x24, 0x42}, //TWL4030_REG_HS_POPN_SET
};

twl4030_codec_setting volume_off[]={
  {0x44, 0x00},  //TWL4030_REG_VDL_APGA_CTL
  {0x1b, 0x00},  //TWL4030_REG_ARXL2_APGA_CTL
  {0x1c, 0x00},  //TWL4030_REG_ARXR2_APGA_CTL
  {0x22, 0x00},  //TWL4030_REG_HS_SEL
  {0x25, 0x00},  //TWL4030_REG_PREDL_CTL
  {0x26, 0x00},  //TWL4030_REG_PREDR_CTL
	{0x21, 0x00}, //TWL4030_REG_EAR_CTL
  {0x09, 0x00},
  {0x04, 0x00},
  {0x07, 0x00},
  {0x05, 0x00},
  {0x06, 0x00},
  {0x17, 0x00},
  #ifdef VOICE_IF_AP_MASTER
  {0x0f, 0x00},
  #endif
  {0xe, 0x0},
  {0x3a, 0x6},
};

#if 0
twl4030_codec_setting Loopback_rcv[]={
	{0x01, 0x92},  //TWL4030_REG_ANAMICR
	{0x3a, 0x16},  //TWL4030_REG_APLL_CTL
	{0x09, 0x30},  //TWL4030_REG_DIGMIXING
	{0x25, 0x20},  //TWL4030_REG_PREDL_CTL
	{0x26, 0x20},  //TWL4030_REG_PREDR_CTL
	{0x0e, 0x01},  //TWL4030_REG_AUDIO_IF
	{0x0f, 0xe1},  //TWL4030_REG_VOICE_IF
	{0x14, 0x20},  //TWL4030_REG_VRXPGA
	{0x16, 0x19},  //TWL4030_REG_VRX2ARXPGA
	{0x17, 0x08},  //TWL4030_REG_AVDAC_CTL
	{0x3e, 0x20},	  //KHoAudioMISC_SET_1
	{0x1c, 0x25},  //TWL4030_REG_ARXR2_APGA_CTL 4dB
	{0x1b, 0x25},  //TWL4030_REG_ARXL2_APGA_CTL 4dB
	{0x13, 0x6f},  //TWL4030_REG_ARXL2PGA
	{0x12, 0x6f},  //TWL4030_REG_ARXR2PGA
	{0x48, 0x24},  //TWL4030_REG_ANAMIC_GAIN
	{0x22, 0x00},  //TWL4030_REG_HS_SEL
	{0x23, 0x00},  //TWL4030_REG_HS_GAIN_SET
	{0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
	{0x1e, 0x00},  //TWL4030_REG_BT_IF
	{0x04, 0x03},  //TWL4030_REG_MICBIAS_CTL
	{0x05, 0x11},  //TWL4030_REG_ANAMICL
	{0x02, 0xf5},  //TWL4030_REG_ANAMICR
	{0x06, 0x11},  //TWL4030_REG_ANAMICR
	{0x08, 0x00},  //TWL4030_REG_ADCMICSEL
	{0x21, 0x15}  //TWL4030_REG_EAR_CTL
};
#else
twl4030_codec_setting Loopback_rcv[]={
  {0x01, 0x93},  //TWL4030_REG_ANAMICR
   {0x3a, 0x16},  //TWL4030_REG_APLL_CTL
  {0x09, 0x10},  //TWL4030_REG_DIGMIXING
  {0x25, 0x20},  //TWL4030_REG_PREDL_CTL
  {0x26, 0x20},  //TWL4030_REG_PREDR_CTL
  {0x0e, 0x01},  //TWL4030_REG_AUDIO_IF
  {0x0f, 0xe1},  //TWL4030_REG_VOICE_IF
  {0x14, 0x2e},  //TWL4030_REG_VRXPGA
  {0x16, 0x12},  //TWL4030_REG_VRX2ARXPGA
  {0x17, 0x08},  //TWL4030_REG_AVDAC_CTL
  {0x1c, 0x1b},  //TWL4030_REG_ARXR2_APGA_CTL
  {0x1b, 0x1b},  //TWL4030_REG_ARXL2_APGA_CTL
  {0x13, 0x6f},  //TWL4030_REG_ARXL2PGA
  {0x12, 0x6f},  //TWL4030_REG_ARXR2PGA
  {0x48, 0x24},  //TWL4030_REG_ANAMIC_GAIN
  {0x22, 0x00},  //TWL4030_REG_HS_SEL
  {0x23, 0x00},  //TWL4030_REG_HS_GAIN_SET
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x1e, 0x00},  //TWL4030_REG_BT_IF
  {0x04, 0x03},  //TWL4030_REG_MICBIAS_CTL
  {0x05, 0x11},  //TWL4030_REG_ANAMICL
  {0x02, 0xf5},  //TWL4030_REG_ANAMICR
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL
  {0x0a, 0x0c},
  {0x21, 0x34}   //TWL4030_REG_EAR_CTL
};
#endif

#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
twl4030_codec_setting Loopback_spk[]={
	{0x01, 0x93},  //TWL4030_REG_CODEC_MODE    
	{0x02, 0xf5},  //TWL4030_REG_OPTION
	{0x09, 0x00},  //TWL4030_REG_DIGMIXING
	{0x0e, 0x01},  //TWL4030_REG_AUDIO_IF    
	{0x12, 0xF1},  //TWL4030_REG_ARXR2PGA       
	{0x13, 0xF1},  //TWL4030_REG_ARXL2PGA
	{0x17, 0x00},  //TWL4030_REG_AVDAC_CTL
	{0x3e, 0x20},  //KHoAudioMISC_SET_1
	{0x1c, 0x35},  //TWL4030_REG_ARXR2_APGA_CTL 4dB
	{0x1b, 0x35},  //TWL4030_REG_ARXL2_APGA_CTL 4dB
	{0x44, 0x25},  //TWL4030_REG_ARXL2_APGA_CTL 4dB  --->>>
	{0x22, 0x00},  //TWL4030_REG_HS_SEL
	{0x23, 0x00},  //TWL4030_REG_HS_GAIN_SET       
	{0x24, 0x00},  //TWL4030_REG_HS_POPN_SET     
	{0x24, 0x00},  //TWL4030_REG_HS_POPN_SET
	{0x3a, 0x16},  //TWL4030_REG_APLL_CTL
	{0x3f, 0x00},  //TWL4030_REG_PCMBTMUX    
	{0x25, 0x11},  //TWL4030_REG_PREDL_CTL
	{0x26, 0x11},  //TWL4030_REG_PREDR_CTL	
	{0x21, 0x00},  //TWL4030_REG_EAR_CTL  
	{0x1e, 0x00},  //TWL4030_REG_BT_IF
	{0x48, 0x00},  //TWL4030_REG_ANAMIC_GAIN  --->>>
	{0x04, 0x04},  //TWL4030_REG_MICBIAS_CTL 
	{0x05, 0x12},  //TWL4030_REG_ANAMICL
	{0x06, 0x00},  //TWL4030_REG_ANAMICR
	{0x07, 0x00},  //TWL4030_REG_AVADC_CTL	
	{0x08, 0x00},  //TWL4030_REG_ADCMICSEL  
};

twl4030_codec_setting Loopback_headset[]={
	{0x01, 0x93},  //TWL4030_REG_CODEC_MODE    
	{0x02, 0xf5},  //TWL4030_REG_OPTION
	{0x09, 0x00},  //TWL4030_REG_DIGMIXING
	{0x0e, 0x01},  //TWL4030_REG_AUDIO_IF    
	{0x12, 0x3f},  //TWL4030_REG_ARXR2PGA       
	{0x13, 0x3f},  //TWL4030_REG_ARXL2PGA
	{0x17, 0x00},  //TWL4030_REG_AVDAC_CTL
	{0x3e, 0x20},  //KHoAudioMISC_SET_1
	{0x1c, 0x53},  //TWL4030_REG_ARXR2_APGA_CTL 4dB
	{0x1b, 0x53},  //TWL4030_REG_ARXL2_APGA_CTL 4dB
	{0x44, 0x0d},  //TWL4030_REG_ARXL2_APGA_CTL 4dB  --->>>
	{0x22, 0x09},  //TWL4030_REG_HS_SEL
	{0x23, 0x05},  //TWL4030_REG_HS_GAIN_SET       
	{0x24, 0x41},  //TWL4030_REG_HS_POPN_SET     
	{0x24, 0x42},  //TWL4030_REG_HS_POPN_SET
	{0x3a, 0x16},  //TWL4030_REG_APLL_CTL
	{0x3f, 0x00},  //TWL4030_REG_PCMBTMUX    
	{0x25, 0x00},  //TWL4030_REG_PREDL_CTL
	{0x26, 0x00},  //TWL4030_REG_PREDR_CTL	
	{0x21, 0x00},  //TWL4030_REG_EAR_CTL  
	{0x1e, 0x00},  //TWL4030_REG_BT_IF
	{0x48, 0x02},  //TWL4030_REG_ANAMIC_GAIN  --->>>
	{0x04, 0x04},  //TWL4030_REG_MICBIAS_CTL 
	{0x05, 0x12},  //TWL4030_REG_ANAMICL
	{0x06, 0x00},  //TWL4030_REG_ANAMICR
	{0x07, 0x00},  //TWL4030_REG_AVADC_CTL	
	{0x08, 0x00},  //TWL4030_REG_ADCMICSEL   	
};
#else
twl4030_codec_setting Loopback_spk[]={
  {0x01, 0x92},  //TWL4030_REG_CODEC_MODE
  {0x3a, 0x16},  //TWL4030_REG_APLL_CTL
  {0x09, 0x30},  //TWL4030_REG_DIGMIXING
  {0x13, 0x7f},  //TWL4030_REG_ARXL2PGA
  {0x12, 0x7f},  //TWL4030_REG_ARXR2PGA
  {0x17, 0x08},  //TWL4030_REG_AVDAC_CTL
  {0x21, 0x00},  //TWL4030_REG_EAR_CTL
  {0x22, 0x00},  //TWL4030_REG_HS_SEL
  {0x23, 0x00},  //TWL4030_REG_HS_GAIN_SET
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x1e, 0x00},  //TWL4030_REG_BT_IF
  {0x05, 0x11},  //TWL4030_REG_ANAMICL
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL
  {0x48, 0x12},  //TWL4030_REG_ANAMIC_GAIN
  {0x16, 0x17},
  {0x3e, 0x20},	  //KHoAudioMISC_SET_1
  {0x1c, 0x25},  //TWL4030_REG_ARXR2_APGA_CTL 4dB
  {0x1b, 0x25},  //TWL4030_REG_ARXL2_APGA_CTL 4dB
  {0x14, 0x29},
  {0x0c, 0x08},
  {0x26, 0x24},  //TWL4030_PREDR_CTL
  {0x25, 0x24}   //TWL4030_PREDL_CTL
};
twl4030_codec_setting Loopback_headset[]={
  {0x01, 0x93},  //TWL4030_REG_CODEC_MODE
  {0x02, 0xf5},  //TWL4030_REG_OPTION
  {0x09, 0x00},  //TWL4030_REG_DIGMIXING
  {0x0e, 0x01},  //TWL4030_REG_AUDIO_IF
  {0x12, 0x6f},  //TWL4030_REG_ARXR2PGA
  {0x13, 0x6f},  //TWL4030_REG_ARXL2PGA
  {0x17, 0x0c},  //TWL4030_REG_AVDAC_CTL
  {0x3e, 0x20},	  //KHoAudioMISC_SET_1
  {0x1c, 0x25},  //TWL4030_REG_ARXR2_APGA_CTL 4dB
  {0x1b, 0x25},  //TWL4030_REG_ARXL2_APGA_CTL 4dB
  {0x44, 0x25},  //TWL4030_REG_ARXL2_APGA_CTL 4dB
  {0x22, 0x2c},  //TWL4030_REG_HS_SEL
  {0x23, 0x05},  //TWL4030_REG_HS_GAIN_SET
  {0x24, 0x41},  //TWL4030_REG_HS_POPN_SET
  {0x24, 0x42},  //TWL4030_REG_HS_POPN_SET
  {0x3a, 0x16},  //TWL4030_REG_APLL_CTL
  {0x3f, 0x00},  //TWL4030_REG_PCMBTMUX
  {0x25, 0x00},  //TWL4030_REG_PREDL_CTL
  {0x26, 0x00},  //TWL4030_REG_PREDR_CTL
  {0x21, 0x00},  //TWL4030_REG_EAR_CTL
  {0x1e, 0x00},  //TWL4030_REG_BT_IF
  {0x48, 0x04},  //TWL4030_REG_ANAMIC_GAIN
  {0x04, 0x04},  //TWL4030_REG_MICBIAS_CTL
  {0x05, 0x12},  //TWL4030_REG_ANAMICL
  {0x06, 0x00},  //TWL4030_REG_ANAMICR
  {0x07, 0x0a},  //TWL4030_REG_AVADC_CTL
  {0x08, 0x00},  //TWL4030_REG_ADCMICSEL
};
#endif
#endif //SAMSUNG_CUSTOMISATION

/* codec private data */
struct twl4030_priv {
	struct snd_soc_codec codec;

	unsigned int codec_powered;

	/* reference counts of AIF/APLL users */
	unsigned int apll_enabled;

	struct snd_pcm_substream *master_substream;
	struct snd_pcm_substream *slave_substream;

	unsigned int configured;
	unsigned int rate;
	unsigned int sample_bits;
	unsigned int channels;

	unsigned int sysclk;

	/* Output (with associated amp) states */
	u8 hsl_enabled, hsr_enabled;
	u8 earpiece_enabled;
	u8 predrivel_enabled, predriver_enabled;
	u8 carkitl_enabled, carkitr_enabled;

	/* Delay needed after enabling the digimic interface */
	unsigned int digimic_delay;
};
  
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
static void twl4030_main_mic_bias_control(int onoff)
{
#ifdef USE_GPIO_MAIN_MIC_BIAS
	if(onoff) {		
		gpio_set_value(MAIN_MIC_BIAS_GPIO, 1);
		printk("[TWL4030] MAIN MIC BIAS ON !!\n");
	} else {
		gpio_set_value(MAIN_MIC_BIAS_GPIO, 0);
		printk("[TWL4030] MAIN MIC BIAS OFF !!\n");
	}
#endif
}

void twl4030_spk_line_out_sel(int mode)
{
#ifdef ENABLE_LINEOUT
	if(mode) {		
		gpio_set_value(SPK_LINE_OUT_SEL, 1);		
		printk("[TWL4030] SPK LINEOUT SEL = SPK(ON)\n");
	} else {
		gpio_set_value(SPK_LINE_OUT_SEL, 0);
		printk("[TWL4030] SPK LINEOUT SEL = LINE OUT(OFF)\n");
	}
#endif
}
#endif
int twl4030_get_voicecall_state(void)
{
	if(twl4030_remap== 1)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL_GPL(twl4030_get_voicecall_state);

#ifdef SAMSUNG_CUSTOMISATION
static void twl4030_set_remap(void)
{
	u8 data = 0x00;
	printk("IDLE mode - Power resources has to be set \n");

	twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_ACTIVE , 0x41 ); // VINTANA1
	twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &data, 0x41 );
	//P("after setting voicecall path : VINTANA1 = %x\n", data);

	twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_ACTIVE , 0x45 ); // VINTANA2
	twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &data, 0x45 );
	//P("after setting voicecall path : VINTANA2 = %x\n", data);

	twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_ACTIVE , 0x49 ); // EVINTDIG
	twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &data, 0x49 );
	//P("after setting voicecall path : VINTDIG = %x\n", data);

	twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_ACTIVE , 0x87 ); // EVCLKEN
	twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &data, 0x87 );
	//P("after setting voicecall path : CLKEN = %x\n", data);

	twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_ACTIVE , 0x8D ); // EVHFCLKOUT
	twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &data, 0x8D );
	//P("after setting voicecall path : HFCLKEN = %x\n", data);

	twl4030_remap = 1;
}
static void twl4030_unset_remap(void)
{
	u8 data = 0x00;
	int ret = 0;
	int retn = 0;

	if(twl4030_remap)
	{
	    printk("IDLE mode - Power resources has to be unset \n");

    	twl4030_remap = 0;
		/*
		   twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_OFF , 0x41 ); // VINTANA1
		   twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER,&data, 0x41 );
		   printk("after setting idle path : VINTANA1 = %x\n", data);
		*/

		ret = twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_OFF , 0x45 ); // VINTANA2
		retn = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &data, 0x45 );
		//P("after setting idle path : VINTANA2 = %x, ret = %d, retn = %d\n", data, ret, retn);

		ret = twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_SLEEP , 0x49 ); // EVINTDIG
		retn = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &data, 0x49 );
		//P("after setting idle path : VINTDIG = %x, ret = %d, retn = %d\n", data, ret, retn);

		ret = twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_OFF , 0x87 ); // EVCLKEN
		retn = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &data, 0x87 );
		//P("after setting idle path : CLKEN = %x, ret = %d, retn = %d\n", data, ret, retn);

		ret = twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_OFF , 0x8D ); // EVHFCLKOUT
		retn = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &data, 0x8D );
		//P("after setting idle path : HFCLKEN = %x, ret = %d, retn = %d\n", data, ret, retn);

		twl4030_remap = 0;
	}
	else
	{
    	printk("IDLE mode - Power resources are not set\n");
	}
}

#endif

/*
 * read twl4030 register cache
 */
static inline unsigned int twl4030_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL4030_CACHEREGNUM)
		return -EIO;

	return cache[reg];
}

/*
 * write twl4030 register cache
 */
static inline void twl4030_write_reg_cache(struct snd_soc_codec *codec,
						u8 reg, u8 value)
{
	u8 *cache = codec->reg_cache;

	if (reg >= TWL4030_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the twl4030 register space
 */
/* static */ int twl4030_write(struct snd_soc_codec *codec,
			unsigned int reg, unsigned int value)
{
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	int write_to_reg = 0;

	twl4030_write_reg_cache(codec, reg, value);
	
#if 0 // Disabling the path which decide if the given register can be written 	
	if (likely(reg < TWL4030_REG_SW_SHADOW)) {
		/* Decide if the given register can be written */
		switch (reg) {
		case TWL4030_REG_EAR_CTL:
			if (twl4030->earpiece_enabled)
				write_to_reg = 1;
			break;
		case TWL4030_REG_PREDL_CTL:
			if (twl4030->predrivel_enabled)
				write_to_reg = 1;
			break;
		case TWL4030_REG_PREDR_CTL:
			if (twl4030->predriver_enabled)
				write_to_reg = 1;
			break;
		case TWL4030_REG_PRECKL_CTL:
			if (twl4030->carkitl_enabled)
				write_to_reg = 1;
			break;
		case TWL4030_REG_PRECKR_CTL:
			if (twl4030->carkitr_enabled)
				write_to_reg = 1;
			break;
		case TWL4030_REG_HS_GAIN_SET:
			if (twl4030->hsl_enabled || twl4030->hsr_enabled)
				write_to_reg = 1;
			break;
		default:
			/* All other register can be written */
			write_to_reg = 1;
			break;
		}
		if (write_to_reg)
			return twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,
						    value, reg);
	}
#endif // Disabling the path which decide if the given register can be written 	
	
	twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,   value, reg);
		
	return 0;
}

/*
 * modified to the twl4030 register space
 */
int twl4030_modify(struct snd_soc_codec *codec,
  unsigned int reg, unsigned int value, unsigned int mask)
{
	u8 reg_val = 0;

	twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &reg_val, reg);

	value = ((reg_val &~mask)|(value & mask));
//	printkd("ID : 0x%x, Value : 0x%x preValue : 0x%x",reg,value, reg_val );

	return twl4030_write(codec, reg, value);
}

#ifdef SAMSUNG_CUSTOMISATION
/*
 * modified to the twl4030 register space
 */
int twl4030_modify_direct(unsigned int reg, unsigned int value, unsigned int mask)
{
	u8 reg_val = 0;

	if (reg >= TWL4030_CACHEREGNUM)
		return -EIO;

	twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &reg_val, reg);

	value = ((reg_val &~mask)|(value & mask));
//	printk("ID : 0x%x, Value : 0x%x preValue : 0x%x\n",reg,value, reg_val );

	return twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE, value, reg);
}

static void codec_control_work_handler(struct work_struct *work )
{
	printk(" twl4030_audio work_handler %d\n", twl4030_call_device);
	switch(twl4030_call_device)
	{
		case OFF:
		break;

		#ifndef VOICECALL_TUNE
		case RCV:
			twl4030_modify_direct(0x21, 0x04, ~EAR_CTL_GAIN_MASK);
		break;

		case SPK:
			twl4030_modify_direct(0x26, 0x08, ~PREDL_CTL_GAIN_MASK);  //TWL4030_PREDR_CTL
	      	twl4030_modify_direct(0x25, 0x04, ~PREDL_CTL_GAIN_MASK);   //TWL4030_PREDL_CTL
		break;

		case HP3P:
		case HP4P:
			twl4030_modify_direct(0x09, 0x30, 0xff);   //
			twl4030_modify_direct(0x1b, 0x03, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXL2_APGA_CTL
			twl4030_modify_direct(0x1c, 0x03, ~ARX_APGA_GAIN_MASK);  //TWL4030_REG_ARXR2_APGA_CTL
		break;

		case BT:
		//PMBR1
			twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x40, 0x0d);
		break;
		#endif

		default:
			printk("!!!!codec_control_work_handle setting failr!!!\n");

		break;

	}
	twl4030_set_remap();
	wake_unlock( &T2_wakelock);
}

static void codec_down_work_handler(struct work_struct *work )
{
	u8 mode = 0;

	//printk("");

	mode &= ~TWL4030_CODECPDZ;

	twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE, mode, TWL4030_REG_CODEC_MODE);
	printk("!!!! codec bias off !!!\n");
}

/*
 * reset output register
 */
static int twl4030_reset_all_output(struct snd_soc_codec *codec)
{
//	P("");

	twl4030_write(codec, 0x3f, 0x00);  //TWL4030_REG_PCMBTMUX
	twl4030_write(codec, 0x21, 0x00);   //TWL4030_REG_EAR_CTL
	twl4030_write(codec, 0x26, 0x00);  //TWL4030_PREDR_CTL
	twl4030_write(codec, 0x25, 0x00);   //TWL4030_PREDL_CTL
	twl4030_write(codec, 0x22, 0x00);  //TWL4030_REG_HS_SEL
	twl4030_write(codec, 0x23, 0x00);  //TWL4030_REG_HS_GAIN_SET

	return 0;
}

static void twl4030_set_pcm_sel(int mode)
{
	printk("twl4030_set_pcm_sel mode %d\n",  mode);
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
	if(mode == BT_SEL_PCM_MODE){
			gpio_set_value(PCM_SEL,0);
	}else if(mode == BT_SEL_I2S_MODE){
			gpio_set_value(PCM_SEL,1);
	}else if(mode == BT_SEL_LOW_MODE){
			gpio_set_value(PCM_SEL,0);
	}else
		printk("ERR!! twl4030_set_pcm_sel() %d\n", mode);
#else		
	if(mode == BT_SEL_PCM_MODE){
			gpio_set_value(PCM_SEL,1);
	}else if(mode == BT_SEL_I2S_MODE){
			gpio_set_value(PCM_SEL,0);
	}else if(mode == BT_SEL_LOW_MODE){
		gpio_set_value(PCM_SEL,0);
	}else
		printk("ERR!! twl4030_set_pcm_sel() %d\n", mode);
#endif		
}

#endif // SAMSUNG_CUSTOMISATION

static void twl4030_codec_enable(struct snd_soc_codec *codec, int enable)
{
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	int mode, i =0;

#ifndef SAMSUNG_CUSTOMISATION
	if (enable == twl4030->codec_powered)
		return;
#endif
	if (enable)
		mode = twl4030_codec_enable_resource(TWL4030_CODEC_RES_POWER);
	else
	{
		mode = twl4030_codec_disable_resource(TWL4030_CODEC_RES_POWER);
#ifdef SAMSUNG_CUSTOMISATION
#if defined(CONFIG_SND_SOC_MAX97000)
		//if(!twl4030_codec_suspended)
		max97000_power_down_mode(); //for powerdown noise
#endif
#if defined(CONFIG_SND_SOC_MAX9877)
		//if(!twl4030_codec_suspended)
		max9877_power_down_mode(); //for powerdown noise
#endif
		for(i=0;i<ARRAY_SIZE(volume_off);i++){
			twl4030_write(codec, volume_off[i].reg,volume_off[i].value);                        
		}
		mdelay(20);
		mode = 0x91;
#endif
	}

#ifndef SAMSUNG_CUSTOMISATION
	if (mode >= 0) {
		twl4030_write_reg_cache(codec, TWL4030_REG_CODEC_MODE, mode);
		twl4030->codec_powered = enable;
	}
#else
		twl4030_write(codec, TWL4030_REG_CODEC_MODE, mode);
		twl4030->codec_powered = enable;
#endif

	/* REVISIT: this delay is present in TI sample drivers */
	/* but there seems to be no TRM requirement for it     */
	udelay(10);
}

#ifdef SAMSUNG_CUSTOMISATION
//enable pcmic
static void twl4030_vintana1_power_enable(int enable)
{
	if(enable)
	{
		printk("[light] power up VINTIANA2....\n");
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x41, TWL4030_VINTANA2_DEDICATED);
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, DEV_GRP_BELONG_P1, TWL4030_VINTANA2_DEV_GRP) ;
	}
	else
	{
		printk("[light] power down VINTIANA2....\n");
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x41, TWL4030_VINTANA2_DEDICATED);
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, DEV_GRP_BELONG_NONE, TWL4030_VINTANA2_DEV_GRP) ;
	}
}
#endif

static inline void twl4030_check_defaults(struct snd_soc_codec *codec)
{
	int i, difference = 0;
	u8 val;

	dev_dbg(codec->dev, "Checking TWL audio default configuration\n");
	for (i = 1; i <= TWL4030_REG_MISC_SET_2; i++) {
		twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &val, i);
		if (val != twl4030_reg[i]) {
			difference++;
			dev_dbg(codec->dev,
				 "Reg 0x%02x: chip: 0x%02x driver: 0x%02x\n",
				 i, val, twl4030_reg[i]);
		}
	}
	dev_dbg(codec->dev, "Found %d non maching registers. %s\n",
		 difference, difference ? "Not OK" : "OK");
}

static inline void twl4030_reset_registers(struct snd_soc_codec *codec)
{
	int i;

	/* set all audio section registers to reasonable defaults */
	for (i = TWL4030_REG_OPTION; i <= TWL4030_REG_MISC_SET_2; i++)
		if (i != TWL4030_REG_APLL_CTL)
			twl4030_write(codec, i, twl4030_reg[i]);

}

static void twl4030_init_chip(struct snd_soc_codec *codec)
{
	struct twl4030_codec_audio_data *pdata = dev_get_platdata(codec->dev);
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 reg, byte;
	int i = 0;

    printk(KERN_INFO "TWL4030 Audio Codec init \n");

	/* Check defaults, if instructed before anything else */
	if (pdata && pdata->check_defaults)
		twl4030_check_defaults(codec);

	/* Reset registers, if no setup data or if instructed to do so */
	if (!pdata || (pdata && pdata->reset_registers))
		twl4030_reset_registers(codec);

	/* Refresh APLL_CTL register from HW */
	twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &byte,
			    TWL4030_REG_APLL_CTL);
	twl4030_write_reg_cache(codec, TWL4030_REG_APLL_CTL, byte);

	/* anti-pop when changing analog gain */
	reg = twl4030_read_reg_cache(codec, TWL4030_REG_MISC_SET_1);
	twl4030_write(codec, TWL4030_REG_MISC_SET_1,
		reg | TWL4030_SMOOTH_ANAVOL_EN);

	twl4030_write(codec, TWL4030_REG_OPTION,
		TWL4030_ATXL1_EN | TWL4030_ATXR1_EN |
		TWL4030_ARXL2_EN | TWL4030_ARXR2_EN);

	/* REG_ARXR2_APGA_CTL reset according to the TRM: 0dB, DA_EN */
	twl4030_write(codec, TWL4030_REG_ARXR2_APGA_CTL, 0x32);

	/* Machine dependent setup */
	if (!pdata)
		return;

	twl4030->digimic_delay = pdata->digimic_delay;

	reg = twl4030_read_reg_cache(codec, TWL4030_REG_HS_POPN_SET);
	reg &= ~TWL4030_RAMP_DELAY;
	reg |= (pdata->ramp_delay_value << 2);
	twl4030_write_reg_cache(codec, TWL4030_REG_HS_POPN_SET, reg);

	/* initiate offset cancellation */
	twl4030_codec_enable(codec, 1);

	reg = twl4030_read_reg_cache(codec, TWL4030_REG_ANAMICL);
	reg &= ~TWL4030_OFFSET_CNCL_SEL;
	reg |= pdata->offset_cncl_path;
	twl4030_write(codec, TWL4030_REG_ANAMICL,
		reg | TWL4030_CNCL_OFFSET_START);

	/* wait for offset cancellation to complete */
	do {
		/* this takes a little while, so don't slam i2c */
		udelay(2000);
		twl_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &byte,
				    TWL4030_REG_ANAMICL);
	} while ((i++ < 100) &&
		 ((byte & TWL4030_CNCL_OFFSET_START) ==
		  TWL4030_CNCL_OFFSET_START));

	/* Make sure that the reg_cache has the same value as the HW */
	twl4030_write_reg_cache(codec, TWL4030_REG_ANAMICL, byte);

	twl4030_codec_enable(codec, 0);

	/* set all audio section registers to reasonable defaults */
	for (i = TWL4030_REG_OPTION; i <= TWL4030_REG_MISC_SET_2; i++)
	//	twl4030_write(codec, i,	cache[i]);
	twl4030_write(codec, i, twl4030_reg_new[i]);
}

#ifdef SAMSUNG_CUSTOMISATION
static void twl4030_codec_mute(struct snd_soc_codec *codec, int mute)
{
	 //struct twl4030_priv *twl4030 = codec->private_data;
     u8 reg_val = 0x00;

//     P("mute : %d", mute);

	 //if (mute == twl4030->codec_muted)
	     //return;

     if (0 /* mute */)
     {
          /* Bypass the reg_cache and mute the volumes
          * Headset mute is done in it's own event handler
          * Things to mute:  Earpiece, PreDrivL/R, CarkitL/R
          */
          reg_val = twl4030_read_reg_cache(codec, TWL4030_REG_EAR_CTL);
          twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,reg_val & (~TWL4030_EAR_GAIN),TWL4030_REG_EAR_CTL);

          reg_val = twl4030_read_reg_cache(codec, TWL4030_REG_PREDL_CTL);
          twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,reg_val & (~TWL4030_PREDL_GAIN),TWL4030_REG_PREDL_CTL);

          reg_val = twl4030_read_reg_cache(codec, TWL4030_REG_PREDR_CTL);
          twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,reg_val & (~TWL4030_PREDR_GAIN),TWL4030_REG_PREDL_CTL);

          // add swin.kim for Headset output ctr
          reg_val = twl4030_read_reg_cache(codec, TWL4030_REG_HS_GAIN_SET);
          twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,reg_val & (~TWL4030_REG_HS_GAIN_SET),TWL4030_REG_HS_GAIN_SET);

          /* Disable PLL */
          reg_val = twl4030_read_reg_cache(codec, TWL4030_REG_APLL_CTL);
          reg_val &= ~TWL4030_APLL_EN;
          twl4030_write(codec, TWL4030_REG_APLL_CTL, reg_val);
     }
     else
     {
          /* Restore the volumes
          * Headset mute is done in it's own event handler
          * Things to restore:  Earpiece, PreDrivL/R, CarkitL/R
          */
          twl4030_write(codec, TWL4030_REG_EAR_CTL,twl4030_read_reg_cache(codec, TWL4030_REG_EAR_CTL));
          twl4030_write(codec, TWL4030_REG_PREDL_CTL,twl4030_read_reg_cache(codec, TWL4030_REG_PREDL_CTL));
          twl4030_write(codec, TWL4030_REG_PREDR_CTL,twl4030_read_reg_cache(codec, TWL4030_REG_PREDR_CTL));
          // add swin.kim for Headset output ctr
          twl4030_write(codec, TWL4030_REG_HS_GAIN_SET,twl4030_read_reg_cache(codec, TWL4030_REG_HS_GAIN_SET));

          /* Enable PLL */
          reg_val = twl4030_read_reg_cache(codec, TWL4030_REG_APLL_CTL);
          reg_val |= TWL4030_APLL_EN;
          twl4030_write(codec, TWL4030_REG_APLL_CTL, reg_val);
	}

	//twl4030->codec_muted = mute;

}
#endif

/*
 * Unconditional power down
 */
static void twl4030_power_down(struct snd_soc_codec *codec)
{
//    P("");
	/* power down */
    //printk("twl4030_write twl4030_power_down \n");
	twl4030_codec_enable(codec, 0);
}


static void twl4030_apll_enable(struct snd_soc_codec *codec, int enable)
{
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	int status = -1;

	printk("twl4030_apll_enable %d\n", enable);
	if (enable) {
		twl4030->apll_enabled++;
		if (twl4030->apll_enabled == 1)
			status = twl4030_codec_enable_resource(
							TWL4030_CODEC_RES_APLL);
	} else {
		twl4030->apll_enabled--;
		if (!twl4030->apll_enabled)
			status = twl4030_codec_disable_resource(
							TWL4030_CODEC_RES_APLL);
	}

	if (status >= 0)
		twl4030_write_reg_cache(codec, TWL4030_REG_APLL_CTL, status);
}

/* Earpiece */
static const struct snd_kcontrol_new twl4030_dapm_earpiece_controls[] = {
	SOC_DAPM_SINGLE("Voice", TWL4030_REG_EAR_CTL, 0, 1, 0),
	SOC_DAPM_SINGLE("AudioL1", TWL4030_REG_EAR_CTL, 1, 1, 0),
	SOC_DAPM_SINGLE("AudioL2", TWL4030_REG_EAR_CTL, 2, 1, 0),
	SOC_DAPM_SINGLE("AudioR1", TWL4030_REG_EAR_CTL, 3, 1, 0),
};

/* PreDrive Left */
static const struct snd_kcontrol_new twl4030_dapm_predrivel_controls[] = {
	SOC_DAPM_SINGLE("Voice", TWL4030_REG_PREDL_CTL, 0, 1, 0),
	SOC_DAPM_SINGLE("AudioL1", TWL4030_REG_PREDL_CTL, 1, 1, 0),
	SOC_DAPM_SINGLE("AudioL2", TWL4030_REG_PREDL_CTL, 2, 1, 0),
	SOC_DAPM_SINGLE("AudioR2", TWL4030_REG_PREDL_CTL, 3, 1, 0),
};

/* PreDrive Right */
static const struct snd_kcontrol_new twl4030_dapm_predriver_controls[] = {
	SOC_DAPM_SINGLE("Voice", TWL4030_REG_PREDR_CTL, 0, 1, 0),
	SOC_DAPM_SINGLE("AudioR1", TWL4030_REG_PREDR_CTL, 1, 1, 0),
	SOC_DAPM_SINGLE("AudioR2", TWL4030_REG_PREDR_CTL, 2, 1, 0),
	SOC_DAPM_SINGLE("AudioL2", TWL4030_REG_PREDR_CTL, 3, 1, 0),
};

/* Headset Left */
static const struct snd_kcontrol_new twl4030_dapm_hsol_controls[] = {
	SOC_DAPM_SINGLE("Voice", TWL4030_REG_HS_SEL, 0, 1, 0),
	SOC_DAPM_SINGLE("AudioL1", TWL4030_REG_HS_SEL, 1, 1, 0),
	SOC_DAPM_SINGLE("AudioL2", TWL4030_REG_HS_SEL, 2, 1, 0),
};

/* Headset Right */
static const struct snd_kcontrol_new twl4030_dapm_hsor_controls[] = {
	SOC_DAPM_SINGLE("Voice", TWL4030_REG_HS_SEL, 3, 1, 0),
	SOC_DAPM_SINGLE("AudioR1", TWL4030_REG_HS_SEL, 4, 1, 0),
	SOC_DAPM_SINGLE("AudioR2", TWL4030_REG_HS_SEL, 5, 1, 0),
};

/* Carkit Left */
static const struct snd_kcontrol_new twl4030_dapm_carkitl_controls[] = {
	SOC_DAPM_SINGLE("Voice", TWL4030_REG_PRECKL_CTL, 0, 1, 0),
	SOC_DAPM_SINGLE("AudioL1", TWL4030_REG_PRECKL_CTL, 1, 1, 0),
	SOC_DAPM_SINGLE("AudioL2", TWL4030_REG_PRECKL_CTL, 2, 1, 0),
};

/* Carkit Right */
static const struct snd_kcontrol_new twl4030_dapm_carkitr_controls[] = {
	SOC_DAPM_SINGLE("Voice", TWL4030_REG_PRECKR_CTL, 0, 1, 0),
	SOC_DAPM_SINGLE("AudioR1", TWL4030_REG_PRECKR_CTL, 1, 1, 0),
	SOC_DAPM_SINGLE("AudioR2", TWL4030_REG_PRECKR_CTL, 2, 1, 0),
};

/* Handsfree Left */
static const char *twl4030_handsfreel_texts[] =
		{"Voice", "AudioL1", "AudioL2", "AudioR2"};

static const struct soc_enum twl4030_handsfreel_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_HFL_CTL, 0,
			ARRAY_SIZE(twl4030_handsfreel_texts),
			twl4030_handsfreel_texts);

static const struct snd_kcontrol_new twl4030_dapm_handsfreel_control =
SOC_DAPM_ENUM("Route", twl4030_handsfreel_enum);

/* Handsfree Left virtual mute */
static const struct snd_kcontrol_new twl4030_dapm_handsfreelmute_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_SW_SHADOW, 0, 1, 0);

/* Handsfree Right */
static const char *twl4030_handsfreer_texts[] =
		{"Voice", "AudioR1", "AudioR2", "AudioL2"};

static const struct soc_enum twl4030_handsfreer_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_HFR_CTL, 0,
			ARRAY_SIZE(twl4030_handsfreer_texts),
			twl4030_handsfreer_texts);

static const struct snd_kcontrol_new twl4030_dapm_handsfreer_control =
SOC_DAPM_ENUM("Route", twl4030_handsfreer_enum);

/* Handsfree Right virtual mute */
static const struct snd_kcontrol_new twl4030_dapm_handsfreermute_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_SW_SHADOW, 1, 1, 0);

/* Vibra */
/* Vibra audio path selection */
static const char *twl4030_vibra_texts[] =
		{"AudioL1", "AudioR1", "AudioL2", "AudioR2"};

static const struct soc_enum twl4030_vibra_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_VIBRA_CTL, 2,
			ARRAY_SIZE(twl4030_vibra_texts),
			twl4030_vibra_texts);

static const struct snd_kcontrol_new twl4030_dapm_vibra_control =
SOC_DAPM_ENUM("Route", twl4030_vibra_enum);

/* Vibra path selection: local vibrator (PWM) or audio driven */
static const char *twl4030_vibrapath_texts[] =
		{"Local vibrator", "Audio"};

static const struct soc_enum twl4030_vibrapath_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_VIBRA_CTL, 4,
			ARRAY_SIZE(twl4030_vibrapath_texts),
			twl4030_vibrapath_texts);

static const struct snd_kcontrol_new twl4030_dapm_vibrapath_control =
SOC_DAPM_ENUM("Route", twl4030_vibrapath_enum);

/* Left analog microphone selection */
static const struct snd_kcontrol_new twl4030_dapm_analoglmic_controls[] = {
	SOC_DAPM_SINGLE("Main Mic Capture Switch",
			TWL4030_REG_ANAMICL, 0, 1, 0),
	SOC_DAPM_SINGLE("Headset Mic Capture Switch",
			TWL4030_REG_ANAMICL, 1, 1, 0),
	SOC_DAPM_SINGLE("AUXL Capture Switch",
			TWL4030_REG_ANAMICL, 2, 1, 0),
	SOC_DAPM_SINGLE("Carkit Mic Capture Switch",
			TWL4030_REG_ANAMICL, 3, 1, 0),
};

/* Right analog microphone selection */
static const struct snd_kcontrol_new twl4030_dapm_analogrmic_controls[] = {
	SOC_DAPM_SINGLE("Sub Mic Capture Switch", TWL4030_REG_ANAMICR, 0, 1, 0),
	SOC_DAPM_SINGLE("AUXR Capture Switch", TWL4030_REG_ANAMICR, 2, 1, 0),
};

/* TX1 L/R Analog/Digital microphone selection */
static const char *twl4030_micpathtx1_texts[] =
		{"Analog", "Digimic0"};

static const struct soc_enum twl4030_micpathtx1_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_ADCMICSEL, 0,
			ARRAY_SIZE(twl4030_micpathtx1_texts),
			twl4030_micpathtx1_texts);

static const struct snd_kcontrol_new twl4030_dapm_micpathtx1_control =
SOC_DAPM_ENUM("Route", twl4030_micpathtx1_enum);

/* TX2 L/R Analog/Digital microphone selection */
static const char *twl4030_micpathtx2_texts[] =
		{"Analog", "Digimic1"};

static const struct soc_enum twl4030_micpathtx2_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_ADCMICSEL, 2,
			ARRAY_SIZE(twl4030_micpathtx2_texts),
			twl4030_micpathtx2_texts);

static const struct snd_kcontrol_new twl4030_dapm_micpathtx2_control =
SOC_DAPM_ENUM("Route", twl4030_micpathtx2_enum);

/* Analog bypass for AudioR1 */
static const struct snd_kcontrol_new twl4030_dapm_abypassr1_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_ARXR1_APGA_CTL, 2, 1, 0);

/* Analog bypass for AudioL1 */
static const struct snd_kcontrol_new twl4030_dapm_abypassl1_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_ARXL1_APGA_CTL, 2, 1, 0);

/* Analog bypass for AudioR2 */
static const struct snd_kcontrol_new twl4030_dapm_abypassr2_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_ARXR2_APGA_CTL, 2, 1, 0);

/* Analog bypass for AudioL2 */
static const struct snd_kcontrol_new twl4030_dapm_abypassl2_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_ARXL2_APGA_CTL, 2, 1, 0);

/* Analog bypass for Voice */
static const struct snd_kcontrol_new twl4030_dapm_abypassv_control =
	SOC_DAPM_SINGLE("Switch", TWL4030_REG_VDL_APGA_CTL, 2, 1, 0);

/* Digital bypass gain, mute instead of -30dB */
static const unsigned int twl4030_dapm_dbypass_tlv[] = {
	TLV_DB_RANGE_HEAD(3),
	0, 1, TLV_DB_SCALE_ITEM(-3000, 600, 1),
	2, 3, TLV_DB_SCALE_ITEM(-2400, 0, 0),
	4, 7, TLV_DB_SCALE_ITEM(-1800, 600, 0),
};

/* Digital bypass left (TX1L -> RX2L) */
static const struct snd_kcontrol_new twl4030_dapm_dbypassl_control =
	SOC_DAPM_SINGLE_TLV("Volume",
			TWL4030_REG_ATX2ARXPGA, 3, 7, 0,
			twl4030_dapm_dbypass_tlv);

/* Digital bypass right (TX1R -> RX2R) */
static const struct snd_kcontrol_new twl4030_dapm_dbypassr_control =
	SOC_DAPM_SINGLE_TLV("Volume",
			TWL4030_REG_ATX2ARXPGA, 0, 7, 0,
			twl4030_dapm_dbypass_tlv);

/*
 * Voice Sidetone GAIN volume control:
 * from -51 to -10 dB in 1 dB steps (mute instead of -51 dB)
 */
static DECLARE_TLV_DB_SCALE(twl4030_dapm_dbypassv_tlv, -5100, 100, 1);

/* Digital bypass voice: sidetone (VUL -> VDL)*/
static const struct snd_kcontrol_new twl4030_dapm_dbypassv_control =
	SOC_DAPM_SINGLE_TLV("Volume",
			TWL4030_REG_VSTPGA, 0, 0x29, 0,
			twl4030_dapm_dbypassv_tlv);

/*
 * Output PGA builder:
 * Handle the muting and unmuting of the given output (turning off the
 * amplifier associated with the output pin)
 * On mute bypass the reg_cache and write 0 to the register
 * On unmute: restore the register content from the reg_cache
 * Outputs handled in this way:  Earpiece, PreDrivL/R, CarkitL/R
 */
#define TWL4030_OUTPUT_PGA(pin_name, reg, mask)				\
static int pin_name##pga_event(struct snd_soc_dapm_widget *w,		\
		struct snd_kcontrol *kcontrol, int event)		\
{									\
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(w->codec); \
									\
	switch (event) {						\
	case SND_SOC_DAPM_POST_PMU:					\
		twl4030->pin_name##_enabled = 1;			\
		twl4030_write(w->codec, reg,				\
			twl4030_read_reg_cache(w->codec, reg));		\
		break;							\
	case SND_SOC_DAPM_POST_PMD:					\
		twl4030->pin_name##_enabled = 0;			\
		twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,		\
					0, reg);			\
		break;							\
	}								\
	return 0;							\
}

TWL4030_OUTPUT_PGA(earpiece, TWL4030_REG_EAR_CTL, TWL4030_EAR_GAIN);
TWL4030_OUTPUT_PGA(predrivel, TWL4030_REG_PREDL_CTL, TWL4030_PREDL_GAIN);
TWL4030_OUTPUT_PGA(predriver, TWL4030_REG_PREDR_CTL, TWL4030_PREDR_GAIN);
TWL4030_OUTPUT_PGA(carkitl, TWL4030_REG_PRECKL_CTL, TWL4030_PRECKL_GAIN);
TWL4030_OUTPUT_PGA(carkitr, TWL4030_REG_PRECKR_CTL, TWL4030_PRECKR_GAIN);

static void handsfree_ramp(struct snd_soc_codec *codec, int reg, int ramp)
{
	unsigned char hs_ctl;

	hs_ctl = twl4030_read_reg_cache(codec, reg);

	if (ramp) {
		/* HF ramp-up */
		hs_ctl |= TWL4030_HF_CTL_REF_EN;
		twl4030_write(codec, reg, hs_ctl);
		udelay(10);
		hs_ctl |= TWL4030_HF_CTL_RAMP_EN;
		twl4030_write(codec, reg, hs_ctl);
		udelay(40);
		hs_ctl |= TWL4030_HF_CTL_LOOP_EN;
		hs_ctl |= TWL4030_HF_CTL_HB_EN;
		twl4030_write(codec, reg, hs_ctl);
	} else {
		/* HF ramp-down */
		hs_ctl &= ~TWL4030_HF_CTL_LOOP_EN;
		hs_ctl &= ~TWL4030_HF_CTL_HB_EN;
		twl4030_write(codec, reg, hs_ctl);
		hs_ctl &= ~TWL4030_HF_CTL_RAMP_EN;
		twl4030_write(codec, reg, hs_ctl);
		udelay(40);
		hs_ctl &= ~TWL4030_HF_CTL_REF_EN;
		twl4030_write(codec, reg, hs_ctl);
	}
}

static int handsfreelpga_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		handsfree_ramp(w->codec, TWL4030_REG_HFL_CTL, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		handsfree_ramp(w->codec, TWL4030_REG_HFL_CTL, 0);
		break;
	}
	return 0;
}

static int handsfreerpga_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		handsfree_ramp(w->codec, TWL4030_REG_HFR_CTL, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		handsfree_ramp(w->codec, TWL4030_REG_HFR_CTL, 0);
		break;
	}
	return 0;
}

static int vibramux_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	twl4030_write(w->codec, TWL4030_REG_VIBRA_SET, 0xff);
	return 0;
}

static int apll_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
#ifndef SAMSUNG_CUSTOMISATION
	printk("apll_event %d\n", event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		twl4030_apll_enable(w->codec, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		twl4030_apll_enable(w->codec, 0);
		break;
	}
#endif
	return 0;
}

static int aif_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
#ifndef SAMSUNG_CUSTOMISATION
	u8 audio_if;
	printk("aif_event %d\n", event);

	audio_if = twl4030_read_reg_cache(w->codec, TWL4030_REG_AUDIO_IF);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Enable AIF */
		/* enable the PLL before we use it to clock the DAI */
		twl4030_apll_enable(w->codec, 1);

		twl4030_write(w->codec, TWL4030_REG_AUDIO_IF,
						audio_if | TWL4030_AIF_EN);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* disable the DAI before we stop it's source PLL */
		twl4030_write(w->codec, TWL4030_REG_AUDIO_IF,
						audio_if &  ~TWL4030_AIF_EN);
		twl4030_apll_enable(w->codec, 0);
		break;
	}
#endif
	return 0;
}

static void headset_ramp(struct snd_soc_codec *codec, int ramp)
{
	struct twl4030_codec_audio_data *pdata = codec->dev->platform_data;
	unsigned char hs_gain, hs_pop;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	/* Base values for ramp delay calculation: 2^19 - 2^26 */
	unsigned int ramp_base[] = {524288, 1048576, 2097152, 4194304,
				    8388608, 16777216, 33554432, 67108864};

	hs_gain = twl4030_read_reg_cache(codec, TWL4030_REG_HS_GAIN_SET);
	hs_pop = twl4030_read_reg_cache(codec, TWL4030_REG_HS_POPN_SET);

	/* Enable external mute control, this dramatically reduces
	 * the pop-noise */
	if (pdata && pdata->hs_extmute) {
		if (pdata->set_hs_extmute) {
			pdata->set_hs_extmute(1);
		} else {
			hs_pop |= TWL4030_EXTMUTE;
			twl4030_write(codec, TWL4030_REG_HS_POPN_SET, hs_pop);
		}
	}

	if (ramp) {
		/* Headset ramp-up according to the TRM */
		hs_pop |= TWL4030_VMID_EN;
		twl4030_write(codec, TWL4030_REG_HS_POPN_SET, hs_pop);
		/* Actually write to the register */
		twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,
					hs_gain,
					TWL4030_REG_HS_GAIN_SET);
		hs_pop |= TWL4030_RAMP_EN;
		twl4030_write(codec, TWL4030_REG_HS_POPN_SET, hs_pop);
		/* Wait ramp delay time + 1, so the VMID can settle */
		mdelay((ramp_base[(hs_pop & TWL4030_RAMP_DELAY) >> 2] /
			twl4030->sysclk) + 1);
	} else {
		/* Headset ramp-down _not_ according to
		 * the TRM, but in a way that it is working */
		hs_pop &= ~TWL4030_RAMP_EN;
		twl4030_write(codec, TWL4030_REG_HS_POPN_SET, hs_pop);
		/* Wait ramp delay time + 1, so the VMID can settle */
		mdelay((ramp_base[(hs_pop & TWL4030_RAMP_DELAY) >> 2] /
			twl4030->sysclk) + 1);
		/* Bypass the reg_cache to mute the headset */
		twl_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE,
					hs_gain & (~0x0f),
					TWL4030_REG_HS_GAIN_SET);

		hs_pop &= ~TWL4030_VMID_EN;
		twl4030_write(codec, TWL4030_REG_HS_POPN_SET, hs_pop);
	}

	/* Disable external mute */
	if (pdata && pdata->hs_extmute) {
		if (pdata->set_hs_extmute) {
			pdata->set_hs_extmute(0);
		} else {
			hs_pop &= ~TWL4030_EXTMUTE;
			twl4030_write(codec, TWL4030_REG_HS_POPN_SET, hs_pop);
		}
	}
}

static int headsetlpga_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(w->codec);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Do the ramp-up only once */
		if (!twl4030->hsr_enabled)
			headset_ramp(w->codec, 1);

		twl4030->hsl_enabled = 1;
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Do the ramp-down only if both headsetL/R is disabled */
		if (!twl4030->hsr_enabled)
			headset_ramp(w->codec, 0);

		twl4030->hsl_enabled = 0;
		break;
	}
	return 0;
}

static int headsetrpga_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(w->codec);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Do the ramp-up only once */
		if (!twl4030->hsl_enabled)
			headset_ramp(w->codec, 1);

		twl4030->hsr_enabled = 1;
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Do the ramp-down only if both headsetL/R is disabled */
		if (!twl4030->hsl_enabled)
			headset_ramp(w->codec, 0);

		twl4030->hsr_enabled = 0;
		break;
	}
	return 0;
}

static int digimic_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(w->codec);

	if (twl4030->digimic_delay)
		mdelay(twl4030->digimic_delay);
	return 0;
}

/*
 * Some of the gain controls in TWL (mostly those which are associated with
 * the outputs) are implemented in an interesting way:
 * 0x0 : Power down (mute)
 * 0x1 : 6dB
 * 0x2 : 0 dB
 * 0x3 : -6 dB
 * Inverting not going to help with these.
 * Custom volsw and volsw_2r get/put functions to handle these gain bits.
 */
#define SOC_DOUBLE_TLV_TWL4030(xname, xreg, shift_left, shift_right, xmax,\
			       xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_get_volsw_twl4030, \
	.put = snd_soc_put_volsw_twl4030, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .shift = shift_left, .rshift = shift_right,\
		 .max = xmax, .invert = xinvert} }
#define SOC_DOUBLE_R_TLV_TWL4030(xname, reg_left, reg_right, xshift, xmax,\
				 xinvert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_2r, \
	.get = snd_soc_get_volsw_r2_twl4030,\
	.put = snd_soc_put_volsw_r2_twl4030, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = reg_left, .rreg = reg_right, .shift = xshift, \
		 .rshift = xshift, .max = xmax, .invert = xinvert} }
#define SOC_SINGLE_TLV_TWL4030(xname, xreg, xshift, xmax, xinvert, tlv_array) \
	SOC_DOUBLE_TLV_TWL4030(xname, xreg, xshift, xshift, xmax, \
			       xinvert, tlv_array)

static int snd_soc_get_volsw_twl4030(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;

	ucontrol->value.integer.value[0] =
		(snd_soc_read(codec, reg) >> shift) & mask;
	if (ucontrol->value.integer.value[0])
		ucontrol->value.integer.value[0] =
			max + 1 - ucontrol->value.integer.value[0];

	if (shift != rshift) {
		ucontrol->value.integer.value[1] =
			(snd_soc_read(codec, reg) >> rshift) & mask;
		if (ucontrol->value.integer.value[1])
			ucontrol->value.integer.value[1] =
				max + 1 - ucontrol->value.integer.value[1];
	}

	return 0;
}

static int snd_soc_put_volsw_twl4030(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;
	unsigned short val, val2, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);

	val_mask = mask << shift;
	if (val)
		val = max + 1 - val;
	val = val << shift;
	if (shift != rshift) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		val_mask |= mask << rshift;
		if (val2)
			val2 = max + 1 - val2;
		val |= val2 << rshift;
	}
	return snd_soc_update_bits(codec, reg, val_mask, val);
}

static int snd_soc_get_volsw_r2_twl4030(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	int mask = (1<<fls(max))-1;

	ucontrol->value.integer.value[0] =
		(snd_soc_read(codec, reg) >> shift) & mask;
	ucontrol->value.integer.value[1] =
		(snd_soc_read(codec, reg2) >> shift) & mask;

	if (ucontrol->value.integer.value[0])
		ucontrol->value.integer.value[0] =
			max + 1 - ucontrol->value.integer.value[0];
	if (ucontrol->value.integer.value[1])
		ucontrol->value.integer.value[1] =
			max + 1 - ucontrol->value.integer.value[1];

	return 0;
}

static int snd_soc_put_volsw_r2_twl4030(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	int max = mc->max;
	int mask = (1 << fls(max)) - 1;
	int err;
	unsigned short val, val2, val_mask;

	val_mask = mask << shift;
	val = (ucontrol->value.integer.value[0] & mask);
	val2 = (ucontrol->value.integer.value[1] & mask);

	if (val)
		val = max + 1 - val;
	if (val2)
		val2 = max + 1 - val2;

	val = val << shift;
	val2 = val2 << shift;

	err = snd_soc_update_bits(codec, reg, val_mask, val);
	if (err < 0)
		return err;

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
	return err;
}

/* Codec operation modes */
static const char *twl4030_op_modes_texts[] = {
	"Option 2 (voice/audio)", "Option 1 (audio)"
};

static const struct soc_enum twl4030_op_modes_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_CODEC_MODE, 0,
			ARRAY_SIZE(twl4030_op_modes_texts),
			twl4030_op_modes_texts);

static int snd_soc_put_twl4030_opmode_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned short val;
	unsigned short mask, bitmask;

	if (twl4030->configured) {
		printk(KERN_ERR "twl4030 operation mode cannot be "
			"changed on-the-fly\n");
		return -EBUSY;
	}

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	val = ucontrol->value.enumerated.item[0] << e->shift_l;
	mask = (bitmask - 1) << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->max - 1)
			return -EINVAL;
		val |= ucontrol->value.enumerated.item[1] << e->shift_r;
		mask |= (bitmask - 1) << e->shift_r;
	}

	return snd_soc_update_bits(codec, e->reg, mask, val);
}

/*
 * FGAIN volume control:
 * from -62 to 0 dB in 1 dB steps (mute instead of -63 dB)
 */
static DECLARE_TLV_DB_SCALE(digital_fine_tlv, -6300, 100, 1);

/*
 * CGAIN volume control:
 * 0 dB to 12 dB in 6 dB steps
 * value 2 and 3 means 12 dB
 */
static DECLARE_TLV_DB_SCALE(digital_coarse_tlv, 0, 600, 0);

/*
 * Voice Downlink GAIN volume control:
 * from -37 to 12 dB in 1 dB steps (mute instead of -37 dB)
 */
static DECLARE_TLV_DB_SCALE(digital_voice_downlink_tlv, -3700, 100, 1);

/*
 * Analog playback gain
 * -24 dB to 12 dB in 2 dB steps
 */
static DECLARE_TLV_DB_SCALE(analog_tlv, -2400, 200, 0);

/*
 * Gain controls tied to outputs
 * -6 dB to 6 dB in 6 dB steps (mute instead of -12)
 */
static DECLARE_TLV_DB_SCALE(output_tvl, -1200, 600, 1);

/*
 * Gain control for earpiece amplifier
 * 0 dB to 12 dB in 6 dB steps (mute instead of -6)
 */
static DECLARE_TLV_DB_SCALE(output_ear_tvl, -600, 600, 1);

/*
 * Capture gain after the ADCs
 * from 0 dB to 31 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(digital_capture_tlv, 0, 100, 0);

/*
 * Gain control for input amplifiers
 * 0 dB to 30 dB in 6 dB steps
 */
static DECLARE_TLV_DB_SCALE(input_gain_tlv, 0, 600, 0);

/* AVADC clock priority */
static const char *twl4030_avadc_clk_priority_texts[] = {
	"Voice high priority", "HiFi high priority"
};

static const struct soc_enum twl4030_avadc_clk_priority_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_AVADC_CTL, 2,
			ARRAY_SIZE(twl4030_avadc_clk_priority_texts),
			twl4030_avadc_clk_priority_texts);

static const char *twl4030_rampdelay_texts[] = {
	"27/20/14 ms", "55/40/27 ms", "109/81/55 ms", "218/161/109 ms",
	"437/323/218 ms", "874/645/437 ms", "1748/1291/874 ms",
	"3495/2581/1748 ms"
};

static const struct soc_enum twl4030_rampdelay_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_HS_POPN_SET, 2,
			ARRAY_SIZE(twl4030_rampdelay_texts),
			twl4030_rampdelay_texts);

/* Vibra H-bridge direction mode */
static const char *twl4030_vibradirmode_texts[] = {
	"Vibra H-bridge direction", "Audio data MSB",
};

static const struct soc_enum twl4030_vibradirmode_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_VIBRA_CTL, 5,
			ARRAY_SIZE(twl4030_vibradirmode_texts),
			twl4030_vibradirmode_texts);

/* Vibra H-bridge direction */
static const char *twl4030_vibradir_texts[] = {
	"Positive polarity", "Negative polarity",
};

static const struct soc_enum twl4030_vibradir_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_VIBRA_CTL, 1,
			ARRAY_SIZE(twl4030_vibradir_texts),
			twl4030_vibradir_texts);

#ifdef SAMSUNG_CUSTOMISATION
static int mic_enable(struct snd_soc_codec *codec, int mode, int enable)
{
	printk("mic_enable mode : %d, enable : %d\n", mode, enable);

	if(twl4030_mic_mute_enable)
	{
		printk("twl4030.c mic_enable() : mic muted, do not power on");
		return 0;
	}

	if(enable)
	{
		switch(mode)
		{
			case MAIN_MIC:
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )			
				twl4030_main_mic_bias_control(true);
#endif				
				twl4030_write(codec, 0x04, 0x03);  //TWL4030_REG_MICBIAS_CTL
				break;

			case HP_MIC:
				twl4030_write(codec, 0x05, 0x12);
				break;

			default:
			break;
		}
	}
	else
	{
		switch(mode)
		{
			case HP_MIC:
			twl4030_write(codec, 0x05, 0x00);

			default:
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )			
			twl4030_main_mic_bias_control(false);
#endif			
			twl4030_write(codec, 0x04, 0x00);  //TWL4030_REG_MICBIAS_CTL
			break;
		}
	}

	return 0;
}
#endif // #ifdef SAMSUNG_CUSTOMISATION

#if 1
#ifdef VOICE_RECOGNITION
int twl4030_is_vr_mode(void)
{
	return twl4030_vr_mode;
}
EXPORT_SYMBOL(twl4030_is_vr_mode);
#endif // VOICE_RECOGNITION

#endif // if 1

#ifdef SAMSUNG_CUSTOMISATION
static int twl4030_set_playback_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol,unsigned int codec_mode)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	printk("twl4030_set_playback_path value = %ld device= %d mode = %d codec_mode = %d \n",ucontrol->value.integer.value[0], twl4030_playback_device, twl4030_mode, codec_mode);


	if((twl4030_playback_device == ucontrol->value.integer.value[0])) //DB24 sec_lilkan
	{
		printk("twl4030_set_playback_device same device d=%d  \n",twl4030_playback_device);
		return TWL4030_SAME_DEVICE;
	}

	twl4030_playback_device = ucontrol->value.integer.value[0];
	twl4030_call_device = 0;
	twl4030_fm_device = 0;

	twl4030_write(codec, 0x0e, 0x01); 
	twl4030_write(codec, 0x3a, 0x16); 

	switch(ucontrol->value.integer.value[0])
	{
		case OFF:
		#ifdef PATH_SET_FROM_ARRAY

		for(i=0;i<ARRAY_SIZE(playback_off);i++)
		{
		    twl4030_write(codec, playback_off[i].reg,playback_off[i].value);
		}

		#else
		twl4030_write(codec, 0x00, 0x00);
		#endif
		break;

		case RCV:
		#ifdef PATH_SET_FROM_ARRAY

		for(i=0;i<ARRAY_SIZE(playback_rcv);i++)
		{
		    twl4030_write(codec, playback_rcv[i].reg,playback_rcv[i].value);
		}

		#else
		printk("set rcv playback path\n");
		twl4030_write(codec, 0x01, 0x93); //TWL4030_REG_CODEC_MODE
		twl4030_write(codec, 0x17, 0x0c); //TWL4030_REG_AVDAC_CTL
		twl4030_modify(codec, 0x1b, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXL2_APGA_CTL
		twl4030_modify(codec, 0x1c, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXR2_APGA_CTL

		twl4030_write(codec, 0x22, 0x00); //TWL4030_REG_HS_SEL
		twl4030_write(codec, 0x26, 0x00); //TWL4030_REG_PREDR_CTL
		twl4030_write(codec, 0x25, 0x00);  //TWL4030_REG_PREDL_CTL

		twl4030_modify(codec, 0x21, 0x35, ~EAR_CTL_GAIN_MASK);
		twl4030_write(codec, 0x43, 0x00);
		#endif
		break;

		case SPK:
		case EXTRA_SPEAKER:
		#ifdef PATH_SET_FROM_ARRAY

		for(i=0;i<ARRAY_SIZE(playback_spk);i++)
		{
		    twl4030_write(codec, playback_spk[i].reg,playback_spk[i].value);
		}

		#else
		//{0x0e, 0x01}, //TWL4030_REG_AUDIO_IF
		//{0x3a, 0x16}, //TWL4030_REG_APLL_CTL
		twl4030_write(codec, 0x01, 0x93); //TWL4030_REG_CODEC_MODE
		twl4030_write(codec, 0x17, 0x0c); //TWL4030_REG_AVDAC_CTL
		twl4030_modify(codec, 0x1b, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXL2_APGA_CTL
		twl4030_modify(codec, 0x1c, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXR2_APGA_CTL
		twl4030_write(codec, 0x21, 0x00); //TWL4030_REG_EAR_CTL
		twl4030_write(codec, 0x22, 0x00); //TWL4030_REG_HS_SEL
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )	
		twl4030_modify(codec, 0x26, 0x24, ~PREDL_CTL_GAIN_MASK); //TWL4030_REG_PREDR_CTL
		twl4030_modify(codec, 0x25, 0x24, ~PREDL_CTL_GAIN_MASK);  //TWL4030_REG_PREDL_CTL	    
#else				
		twl4030_modify(codec, 0x26, 0x28, ~PREDL_CTL_GAIN_MASK); //TWL4030_REG_PREDR_CTL
		twl4030_modify(codec, 0x25, 0x28, ~PREDL_CTL_GAIN_MASK);  //TWL4030_REG_PREDL_CTL
#endif
		twl4030_write(codec, 0x43, 0x00);

#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
		if(ucontrol->value.integer.value[0] == EXTRA_SPEAKER)
			twl4030_spk_line_out_sel(0);
#endif			
		#endif
		break;

		case HP3P:
		case HP4P:
		#ifdef PATH_SET_FROM_ARRAY

		for(i=0;i<ARRAY_SIZE(playback_hp);i++)
		{
		    twl4030_write(codec, playback_hp[i].reg,playback_hp[i].value);
		}

		#else
		    //{0x0e, 0x01);, //TWL4030_REG_AUDIO_IF
		    //{0x3a, 0x16}, //TWL4030_REG_APLL_CTL
	    	twl4030_write(codec, 0x01, 0x93); //TWL4030_REG_CODEC_MODE
	    	//twl4030_write(codec, 0x13, 0x3b); //TWL4030_REG_ARXL2PGA
	    	//twl4030_write(codec, 0x12, 0x3b); //TWL4030_REG_ARXR2PGA
	    	twl4030_write(codec, 0x17, 0x0c); //TWL4030_REG_AVDAC_CTL
	    	twl4030_modify(codec, 0x1b, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXL2_APGA_CTL
	    	twl4030_modify(codec, 0x1c, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXR2_APGA_CTL
	    	twl4030_write(codec, 0x21, 0x00); //TWL4030_REG_EAR_CTL
	    	//twl4030_write(codec, 0x23, 0x05); //TWL4030_REG_HS_GAIN_SET
	    	twl4030_write(codec, 0x24, 0x41); //TWL4030_REG_HS_POPN_SET
	    	twl4030_write(codec, 0x24, 0x42); //TWL4030_REG_HS_POPN_SET
            twl4030_write(codec, 0x22, 0x24);  //TWL4030_REG_HS_SEL
	    	twl4030_write(codec, 0x26, 0x00); //TWL4030_REG_PREDR_CTL
	    	twl4030_write(codec, 0x25, 0x00);  //TWL4030_REG_PREDL_CTL

	    	twl4030_write(codec, 0x43, 0x00);
		#endif
		break;

		case SPK_HP:
		#ifdef PATH_SET_FROM_ARRAY
		for(i=0;i<ARRAY_SIZE(playback_spk_hp);i++)
		{
		    twl4030_write(codec, playback_spk_hp[i].reg,playback_spk_hp[i].value);
		}
		#else
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
		    //{0x0e, 0x01);, //TWL4030_REG_AUDIO_IF
		    //{0x3a, 0x16);, //TWL4030_REG_APLL_CTL
		    twl4030_write(codec, 0x17, 0x0f); //TWL4030_REG_AVDAC_CTL enable DACL1,R1, DACL2, R2
	    	twl4030_write(codec, 0x43, 0x0a); //TWL4030_REG_AVDAC_CTL  
	    	twl4030_write(codec, 0x01, 0x93); //TWL4030_REG_CODEC_MODE
	    	//twl4030_write(codec, 0x13, 0x3d); //TWL4030_REG_ARXL2PGA
	    	//twl4030_write(codec, 0x12, 0x3d); //TWL4030_REG_ARXR2PGA 
			twl4030_modify(codec, 0x19, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXL1_APGA_CTL 
	    	twl4030_modify(codec, 0x1a, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXR1_APGA_CTL 
	    	twl4030_write(codec, 0x22, 0x12); //TWL4030_REG_HS_SEL   select AUDIO L1, R1
	    	
	    	twl4030_modify(codec, 0x1b, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXL2_APGA_CTL 
	    	twl4030_modify(codec, 0x1c, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXR2_APGA_CTL 	    	
	    	twl4030_write(codec, 0x21, 0x00); //TWL4030_REG_EAR_CTL  

	    	//twl4030_write(codec, 0x23, 0x0f); //TWL4030_REG_HS_GAIN_SET
	    	twl4030_write(codec, 0x24, 0x41); //TWL4030_REG_HS_POPN_SET       
	    	twl4030_write(codec, 0x24, 0x42); //TWL4030_REG_HS_POPN_SET   
	    	twl4030_modify(codec, 0x25, 0x24, ~PREDL_CTL_GAIN_MASK); //TWL4030_REG_PREDL_CTL   
	    	twl4030_modify(codec, 0x26, 0x24, ~PREDL_CTL_GAIN_MASK); //TWL4030_REG_PREDR_CTL   
	    
	    	//twl4030_write(codec, 0x43, 0x00);
#else					
		    //{0x0e, 0x01);, //TWL4030_REG_AUDIO_IF
		    //{0x3a, 0x16);, //TWL4030_REG_APLL_CTL
	    	twl4030_write(codec, 0x01, 0x93); //TWL4030_REG_CODEC_MODE
	    	//twl4030_write(codec, 0x13, 0x3d); //TWL4030_REG_ARXL2PGA
	    	//twl4030_write(codec, 0x12, 0x3d); //TWL4030_REG_ARXR2PGA
	    	twl4030_write(codec, 0x17, 0x0c); //TWL4030_REG_AVDAC_CTL
	    	twl4030_modify(codec, 0x1b, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXL2_APGA_CTL
	    	twl4030_modify(codec, 0x1c, 0x33, ~ARX_APGA_GAIN_MASK); //TWL4030_REG_ARXR2_APGA_CTL
	    	twl4030_write(codec, 0x21, 0x00); //TWL4030_REG_EAR_CTL
	    	twl4030_write(codec, 0x22, 0x24); //TWL4030_REG_HS_SEL
	    	//twl4030_write(codec, 0x23, 0x0f); //TWL4030_REG_HS_GAIN_SET
	    	twl4030_write(codec, 0x24, 0x41); //TWL4030_REG_HS_POPN_SET
	    	twl4030_write(codec, 0x24, 0x42); //TWL4030_REG_HS_POPN_SET
	    	twl4030_modify(codec, 0x25, 0x28, ~PREDL_CTL_GAIN_MASK); //TWL4030_REG_PREDL_CTL
	    	twl4030_modify(codec, 0x26, 0x28, ~PREDL_CTL_GAIN_MASK); //TWL4030_REG_PREDR_CTL

	    	twl4030_write(codec, 0x43, 0x00);
#endif			
		#endif
		break;

		default:
		    printk("!!!!playback path setting failed!!!\n");
		break;
	}

	return 0;
}


static int twl4030_set_voicecall_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol,unsigned int codec_mode)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int i=0;

	printk("twl4030_set_voicecall_path value = %ld call_device = %d, mode = %d \n",ucontrol->value.integer.value[0], twl4030_call_device, codec_mode);

	if((twl4030_call_device == ucontrol->value.integer.value[0])) //DB24 sec_lilkan
	{
		twl4030_set_remap();
		printk("twl4030_set_voicecall_path same device d=%d  \n",twl4030_call_device);
		return TWL4030_SAME_DEVICE;
	}
	cancel_delayed_work(&codec_control_work);

	twl4030_call_device = ucontrol->value.integer.value[0];
	//W/A for callmute : it should be removed after fix -taesung kim
	wake_lock( &T2_wakelock);
	twl4030_playback_device = 0;
    twl4030_fm_device = 0;

	twl4030_set_pcm_sel(BT_SEL_LOW_MODE);

	twl4030_write(codec, 0x0e, 0x01); 
	twl4030_write(codec, 0x3a, 0x16); 
	switch(ucontrol->value.integer.value[0])
	{
		case OFF:
		for(i=0;i<ARRAY_SIZE(voicecall_off);i++)
		{
			twl4030_write(codec, voicecall_off[i].reg,voicecall_off[i].value);
		}

		break;

		case RCV:

		#ifdef PATH_SET_FROM_ARRAY

		for(i=0;i<ARRAY_SIZE(voicecall_rcv);i++)
		{
			twl4030_write(codec, voicecall_rcv[i].reg,voicecall_rcv[i].value);
		}

		#else
		#ifdef VOICE_IF_AP_MASTER
		twl4030_write(codec, 0x0f, 0x61);  //TWL4030_REG_VOICE_IF
		printk("enable ap master mode\n");
		#else
		twl4030_write(codec, 0x0f, 0xe1);  //TWL4030_REG_VOICE_IF
		#endif
		twl4030_write(codec, 0x01, 0x92);  //TWL4030_REG_ANAMICR
		twl4030_write(codec, 0x3a, 0x16);  //TWL4030_REG_APLL_CTL
		twl4030_write(codec, 0x09, 0x30);  //TWL4030_REG_DIGMIXING
		twl4030_write(codec, 0x25, 0x20);  //TWL4030_REG_PREDL_CTL
		twl4030_write(codec, 0x26, 0x20);  //TWL4030_REG_PREDR_CTL
		twl4030_write(codec, 0x0e, 0x01);  //TWL4030_REG_AUDIO_IF
		//twl4030_write(codec, 0x0f, 0xe1);  //TWL4030_REG_VOICE_IF
		twl4030_write(codec, 0x17, 0x08);  //TWL4030_REG_AVDAC_CTL
		twl4030_modify(codec, 0x1c, 0x1b, ~ARX_APGA_GAIN_MASK);  //TWL4030_REG_ARXR2_APGA_CTL
		twl4030_modify(codec, 0x1b, 0x1b, ~ARX_APGA_GAIN_MASK);  //TWL4030_REG_ARXL2_APGA_CTL

		//reset other path
		twl4030_write(codec, 0x22, 0x00);  //TWL4030_REG_HS_SEL
		twl4030_write(codec, 0x3f, 0x00);  //TWL4030_REG_PCMBTMUX
		twl4030_write(codec, 0x1e, 0x00); //TWL4030_REG_BT_IF

		mic_enable(codec, MAIN_MIC, 1);

		twl4030_write(codec, 0x05, 0x11);  //TWL4030_REG_ANAMICL
		twl4030_write(codec, 0x02, 0xf5);  //TWL4030_REG_ANAMICR
		twl4030_write(codec, 0x06, 0x00);  //TWL4030_REG_ANAMICR
		twl4030_write(codec, 0x07, 0x0a);  //TWL4030_REG_AVADC_CTL
		twl4030_write(codec, 0x08, 0x00);  //TWL4030_REG_ADCMICSEL

       	//twl4030_modify(codec, 0x21, 0x35, ~EAR_CTL_GAIN_MASK); //set in work queue
		#endif

		break;

		case SPK:
		#ifdef PATH_SET_FROM_ARRAY
		for(i=0;i<ARRAY_SIZE(voicecall_spk);i++)
		{
			  twl4030_write(codec, voicecall_spk[i].reg,voicecall_spk[i].value);
		}
		#else
		#ifdef VOICE_IF_AP_MASTER
		twl4030_write(codec, 0x0f, 0x61);  //TWL4030_REG_VOICE_IF
		#else
		twl4030_write(codec, 0x0f, 0xe1);  //TWL4030_REG_VOICE_IF
		#endif
		twl4030_write(codec, 0x01, 0x92);  //TWL4030_REG_CODEC_MODE
		twl4030_write(codec, 0x3a, 0x16);  //TWL4030_REG_APLL_CTL
		twl4030_write(codec, 0x09, 0x30);  //TWL4030_REG_DIGMIXING
		twl4030_write(codec, 0x17, 0x08);  //TWL4030_REG_AVDAC_CTL
		//reset other path
		twl4030_modify(codec, 0x21, 0x00, ~EAR_CTL_GAIN_MASK);   //TWL4030_REG_EAR_CTL
		twl4030_write(codec, 0x3f, 0x00);  //TWL4030_REG_PCMBTMUX
		twl4030_write(codec, 0x1e, 0x00); //TWL4030_REG_BT_IF
		twl4030_write(codec, 0x22, 0x00);  //TWL4030_REG_HS_GAIN_SET
		twl4030_write(codec, 0x23, 0x00); //TWL4030_REG_HS_SEL

		mic_enable(codec, MAIN_MIC, 1);

		twl4030_write(codec, 0x05, 0x11);  //TWL4030_REG_ANAMICL
		twl4030_write(codec, 0x06, 0x00);  //TWL4030_REG_ANAMICR
		twl4030_write(codec, 0x07, 0x0a);  //TWL4030_REG_AVADC_CTL
		twl4030_write(codec, 0x08, 0x00);  //TWL4030_REG_ADCMICSEL

      		//twl4030_modify(codec, 0x26, 0x28, ~PREDL_CTL_GAIN_MASK);  //set in work queue
	      //twl4030_modify(codec, 0x25, 0x24, ~PREDL_CTL_GAIN_MASK);   //set in work queue
		#endif

        break;

		case HP3P:
		#ifdef PATH_SET_FROM_ARRAY

		for(i=0;i<ARRAY_SIZE(voicecall_hp3p);i++)
		{
			 twl4030_write(codec, voicecall_hp3p[i].reg,voicecall_hp3p[i].value);
		}
		#else
		#ifdef VOICE_IF_AP_MASTER
		twl4030_write(codec, 0x0f, 0x61);  //TWL4030_REG_VOICE_IF
		#else
		twl4030_write(codec, 0x0f, 0xe1);  //TWL4030_REG_VOICE_IF
		#endif
		twl4030_write(codec, 0x01, 0x92);  //TWL4030_REG_CODEC_MODE
		twl4030_write(codec, 0x02, 0xf5);  //TWL4030_REG_OPTION
		twl4030_write(codec, 0x09, 0x30);  //TWL4030_REG_DIGMIXING
		twl4030_write(codec, 0x0e, 0x01);  //TWL4030_REG_AUDIO_IF
		twl4030_write(codec, 0x17, 0x0c);  //TWL4030_REG_AVDAC_CTL
		twl4030_write(codec, 0x22, 0x24);  //TWL4030_REG_HS_SEL
		twl4030_write(codec, 0x24, 0x41);  //TWL4030_REG_HS_POPN_SET
		twl4030_write(codec, 0x24, 0x42); //TWL4030_REG_HS_POPN_SET
		twl4030_write(codec, 0x3a, 0x16);  //TWL4030_REG_APLL_CTL

		//reset other path
		twl4030_write(codec, 0x3f, 0x00);  //TWL4030_REG_PCMBTMUX
		twl4030_modify(codec, 0x21, 0x00, ~EAR_CTL_GAIN_MASK);   //TWL4030_REG_EAR_CTL
		twl4030_modify(codec, 0x26, 0x00, ~PREDL_CTL_GAIN_MASK);  //TWL4030_PREDR_CTL
	    twl4030_modify(codec, 0x25, 0x00, ~PREDL_CTL_GAIN_MASK);   //TWL4030_PREDL_CTL
	    twl4030_write(codec, 0x1e, 0x00); //TWL4030_REG_BT_IF

		//mic control
		mic_enable(codec, MAIN_MIC, 1);
		twl4030_write(codec, 0x05, 0x11);  //TWL4030_REG_ANAMICL
		twl4030_write(codec, 0x06, 0x00);  //TWL4030_REG_ANAMICR
		twl4030_write(codec, 0x07, 0x0a);  //TWL4030_REG_AVADC_CTL
		twl4030_write(codec, 0x08, 0x00);  //TWL4030_REG_ADCMICSEL

		twl4030_write(codec, 0x09, 0x00);  //TWL4030_REG_DIGMIXING
		//twl4030_modify(codec, 0x1b, 0x3b, ~ARX_APGA_GAIN_MASK); //set in work queue
		//twl4030_modify(codec, 0x1c, 0x3b, ~ARX_APGA_GAIN_MASK); //set in work queue
		#endif
		break;

		case HP4P:
		#ifdef PATH_SET_FROM_ARRAY

		for(i=0;i<ARRAY_SIZE(voicecall_hp4p);i++)
		{
			 twl4030_write(codec, voicecall_hp4p[i].reg,voicecall_hp4p[i].value);
		}

		#else
		#ifdef VOICE_IF_AP_MASTER
		twl4030_write(codec, 0x0f, 0x61);  //TWL4030_REG_VOICE_IF
		#else
		twl4030_write(codec, 0x0f, 0xe1);  //TWL4030_REG_VOICE_IF
		#endif
		twl4030_write(codec, 0x01, 0x92);  //TWL4030_REG_CODEC_MODE
		twl4030_write(codec, 0x02, 0xf5);  //TWL4030_REG_OPTION
		twl4030_write(codec, 0x09, 0x30);  //TWL4030_REG_DIGMIXING
		twl4030_write(codec, 0x0e, 0x01);  //TWL4030_REG_AUDIO_IF
		twl4030_write(codec, 0x17, 0x0c);  //TWL4030_REG_AVDAC_CTL
		twl4030_write(codec, 0x22, 0x24);  //TWL4030_REG_HS_SEL
		twl4030_write(codec, 0x24, 0x41);  //TWL4030_REG_HS_POPN_SET
		twl4030_write(codec, 0x24, 0x42); //TWL4030_REG_HS_POPN_SET
		twl4030_write(codec, 0x3a, 0x16);  //TWL4030_REG_APLL_CTL

		//reset other path
		twl4030_write(codec, 0x3f, 0x00);  //TWL4030_REG_PCMBTMUX
		twl4030_modify(codec, 0x21, 0x00, ~EAR_CTL_GAIN_MASK);   //TWL4030_REG_EAR_CTL
		twl4030_modify(codec, 0x26, 0x00, ~PREDL_CTL_GAIN_MASK);  //TWL4030_PREDR_CTL
	    twl4030_modify(codec, 0x25, 0x00, ~PREDL_CTL_GAIN_MASK);   //TWL4030_PREDL_CTL
	    twl4030_write(codec, 0x1e, 0x00); //TWL4030_REG_BT_IF

		//mic control
		twl4030_write(codec, 0x04, 0x04);  //TWL4030_REG_MICBIAS_CTL
		mic_enable(codec, HP_MIC, 1);
		//twl4030_write(codec, 0x05, 0x12);  //TWL4030_REG_ANAMICL
		twl4030_write(codec, 0x06, 0x00);  //TWL4030_REG_ANAMICR
		twl4030_write(codec, 0x07, 0x0a);  //TWL4030_REG_AVADC_CTL
		twl4030_write(codec, 0x08, 0x00);  //TWL4030_REG_ADCMICSEL

		twl4030_write(codec, 0x09, 0x00);  //set in work queue
		//twl4030_modify(codec, 0x1b, 0x3b, ~ARX_APGA_GAIN_MASK); //set in work queue
		//twl4030_modify(codec, 0x1c, 0x3b, ~ARX_APGA_GAIN_MASK); //set in work queue
		#endif
		break;

		case BT:
		#ifdef PATH_SET_FROM_ARRAY

		for(i=0;i<ARRAY_SIZE(voicecall_bt);i++)
		{
			 twl4030_write(codec, voicecall_bt[i].reg,voicecall_bt[i].value);
		}
		#else
		twl4030_set_pcm_sel(BT_SEL_PCM_MODE);
		#ifdef VOICE_IF_AP_MASTER
		twl4030_write(codec, 0x0f, 0x61);  //TWL4030_REG_VOICE_IF
		#else
		twl4030_write(codec, 0x0f, 0xe1);  //TWL4030_REG_VOICE_IF
		#endif
		//output
		twl4030_write(codec, 0x01, 0x92);
		twl4030_write(codec, 0x3a, 0x16);

		twl4030_write(codec, 0x17, 0x00);  //TWL4030_REG_AVDAC_CTL
		twl4030_write(codec, 0x43, 0x10);  //TWL4030_REG_RX_PATH_SEL
		twl4030_write(codec, 0x3f, 0xa0);  //TWL4030_REG_PCMBTMUX
		twl4030_write(codec, 0x1e, 0x61);  //TWL4030_REG_BT_IF{0x22, 0x00},  //TWL4030_REG_HS_SEL

		twl4030_write(codec, 0x22, 0x00);  //TWL4030_REG_HS_SEL  fix bt call rx path problem
		twl4030_write(codec, 0x23, 0x00);  //TWL4030_REG_HS_GAIN_SET
		twl4030_modify(codec, 0x21, 0x00, ~EAR_CTL_GAIN_MASK);   //TWL4030_REG_EAR_CTL
		twl4030_modify(codec, 0x26, 0x00, ~PREDL_CTL_GAIN_MASK);  //TWL4030_PREDR_CTL
	   	twl4030_modify(codec, 0x25, 0x00, ~PREDL_CTL_GAIN_MASK);   //TWL4030_PREDL_CTL

		//input
		twl4030_write(codec, 0x04, 0x00);  //TWL4030_REG_MICBIAS_CTL
		twl4030_write(codec, 0x05, 0x00);  //TWL4030_REG_ANAMICL
		twl4030_write(codec, 0x06, 0x00);  //TWL4030_REG_ANAMICR
		twl4030_write(codec, 0x08, 0x00);  //TWL4030_REG_ADCMICSEL
		//twl4030_i2c_write_u8(TWL4030_MODULE_INTBR, 0x40, 0x0d); //set in work queue
		#endif
		break;

		default:
			printk("!!!!voicecall path setting failed!!!\n");

		break;
	}

	return 0;
}

#if 0
#ifdef VOICE_RECOGNITION
int twl4030_is_vr_mode(void)
{
	return twl4030_vr_mode;
}
EXPORT_SYMBOL(twl4030_is_vr_mode);
#endif
#endif

static int twl4030_set_voicememo_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol,unsigned int codec_mode)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	twl4030_call_device = 0;
	twl4030_fm_device = 0;

	if(twl4030_recording_device == ucontrol->value.integer.value[0]){		
		printk("voice memo device is same force return\n");
		return 0;
	}
	twl4030_recording_device =ucontrol->value.integer.value[0] ;
	printk("twl4030_set_voicememo_path value = %ld \n",ucontrol->value.integer.value[0]);

	twl4030_write(codec, 0x0e, 0x01); 
	twl4030_write(codec, 0x3a, 0x16); 

	twl4030_write(codec, 0x07, 0x00);
	twl4030_write(codec, 0x08, 0x00);	
	twl4030_write(codec, 0x01, 0x93);
	twl4030_write(codec, 0x44, 0x32);	

	// Tx gain control

	switch(ucontrol->value.integer.value[0])
	{
		case OFF:
			break;

		case MAIN_MIC:
			mic_enable(codec, HP_MIC, 0); 
			mic_enable(codec, MAIN_MIC, 1);           

			twl4030_write(codec, 0x05, 0x11);
			twl4030_write(codec, 0x06, 0x00);
			twl4030_write(codec, 0x07, 0x0e);
			mdelay(10);

			if(twl4030_voip_device == RCV)
	        {
	            twl4030_write(codec, 0x0c, 0x06);
	            twl4030_write(codec, 0x48, 0x2D);
	            twl4030_write(codec, 0x1b, 0x13);
	            twl4030_write(codec, 0x1c, 0x13);
	            twl4030_write(codec, 0x25, 0x20);
	            twl4030_write(codec, 0x26, 0x20);
	        }
	        else if(twl4030_voip_device == SPK)
	        {
	            twl4030_write(codec, 0x0c, 0x08);
	            twl4030_write(codec, 0x48, 0x1B);
	            twl4030_write(codec, 0x1b, 0x0b);
	            twl4030_write(codec, 0x1c, 0x0b);
	        }
	        else if(twl4030_voip_device == HP3P)
	        {
	            twl4030_write(codec, 0x0c, 0x08);
	            twl4030_write(codec, 0x48, 0x24);
	            twl4030_write(codec, 0x1b, 0x13);
	            twl4030_write(codec, 0x1c, 0x13);
	            twl4030_write(codec, 0x25, 0x00);
	            twl4030_write(codec, 0x26, 0x00);
	        }
	        else if(twl4030_voip_device == HP4P)
	        {
	            twl4030_write(codec, 0x0c, 0x06);
	            twl4030_write(codec, 0x48, 0x2d);
	            twl4030_write(codec, 0x1b, 0x13);
	            twl4030_write(codec, 0x1c, 0x13);
	            twl4030_write(codec, 0x25, 0x00);
	            twl4030_write(codec, 0x26, 0x00);
	        }
			break;

		case HP_MIC:
			mic_enable(codec, MAIN_MIC, 0);
			twl4030_write(codec, 0x04, 0x04);
			twl4030_write(codec, 0x05, 0x12);
			twl4030_write(codec, 0x07, 0x0a);

			if(twl4030_voip_device == RCV)
	        {
	          	twl4030_write(codec, 0x0c, 0x06);
	            twl4030_write(codec, 0x48, 0x2D);
	            twl4030_write(codec, 0x1b, 0x13);
	            twl4030_write(codec, 0x1c, 0x13);
	            twl4030_write(codec, 0x25, 0x20);
	            twl4030_write(codec, 0x26, 0x20);
	        }
	        else if(twl4030_voip_device == SPK)
	        {
	            twl4030_write(codec, 0x0c, 0x08);
	            twl4030_write(codec, 0x48, 0x1B);
	            twl4030_write(codec, 0x1b, 0x0b);
	            twl4030_write(codec, 0x1c, 0x0b);
	        }
	        else if(twl4030_voip_device == HP3P)
	        {
	            twl4030_write(codec, 0x0c, 0x08);
	            twl4030_write(codec, 0x48, 0x24);
	        	twl4030_write(codec, 0x1b, 0x13);
	            twl4030_write(codec, 0x1c, 0x13);
	            twl4030_write(codec, 0x25, 0x00);
	            twl4030_write(codec, 0x26, 0x00);
	         }
	         else if(twl4030_voip_device == HP4P)
	         {
	            twl4030_write(codec, 0x0c, 0x06);
	            twl4030_write(codec, 0x48, 0x2d);
	            twl4030_write(codec, 0x1b, 0x13);
	            twl4030_write(codec, 0x1c, 0x13);
	            twl4030_write(codec, 0x25, 0x00);
	            twl4030_write(codec, 0x26, 0x00);
	          }
			break;

			case BT_MIC:
				twl4030_write(codec, 0x0e, 0x02); //for disconnecting codec i2s line, must be set AUDIO_IF enable 2010.11.05 changoh.heo
				twl4030_set_pcm_sel(BT_SEL_I2S_MODE);
			break;
	}

	if(twl4030_vr_mode)
		mdelay(360);
	
	return 0;
}

// hskwon-ss-cl31, added for FMC(VoIP) call path
static int twl4030_set_voipcall_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol,unsigned int codec_mode)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int i=0;

    printk("twl4030_set_voipcall_path value = %ld call_device = %d \n",ucontrol->value.integer.value[0], twl4030_voip_device);

	twl4030_voip_device = ucontrol->value.integer.value[0];
	twl4030_call_device = 0;
	twl4030_recording_device = 0;

	twl4030_write(codec, 0x0e, 0x01); 
	twl4030_write(codec, 0x3a, 0x16); 

	switch(ucontrol->value.integer.value[0])
	{
		case OFF:
			for(i=0;i<ARRAY_SIZE(voipcall_off);i++)
			{
				twl4030_write(codec, voipcall_off[i].reg,voipcall_off[i].value);
			}

			break;

		case RCV:
			for(i=0;i<ARRAY_SIZE(voipcall_rcv);i++)
			{
				twl4030_write(codec, voipcall_rcv[i].reg,voipcall_rcv[i].value);
			}
			mic_enable(codec, MAIN_MIC, 1);
			break;

		case SPK:
			for(i=0;i<ARRAY_SIZE(voipcall_spk);i++)
			{
				twl4030_write(codec, voipcall_spk[i].reg,voipcall_spk[i].value);
			}
			mic_enable(codec, MAIN_MIC, 1);
			break;

		case HP3P:
			for(i=0;i<ARRAY_SIZE(voipcall_hp3p);i++)
			{
				twl4030_write(codec, voipcall_hp3p[i].reg,voipcall_hp3p[i].value);
			}
			break;

		case HP4P:
			for(i=0;i<ARRAY_SIZE(voipcall_hp4p);i++)
			{
				twl4030_write(codec, voipcall_hp4p[i].reg,voipcall_hp4p[i].value);
			}
			break;

		case BT:
			printk("bt voip not support in twl4030\n");
			break;

		default:
			printk("!!!!voipcall path setting failed!!!\n");

			break;
	}
    return 0;
}
static int twl4030_set_fmradio_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol,unsigned int codec_mode)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);


	if((twl4030_fm_device == ucontrol->value.integer.value[0])) //DB24 sec_lilkan
	{
		printk("twl4030_set_fmradio_path same device d=%d mute = %d \n",twl4030_fm_device, twl4030_fm_radio_mute_enable);
		return TWL4030_SAME_DEVICE;
	}

    printk("twl4030_set_fmradio_path value = %ld output_device = %d, mute = %d \n",
		ucontrol->value.integer.value[0], twl4030_fm_device, twl4030_fm_radio_mute_enable);

    twl4030_fm_device = ucontrol->value.integer.value[0];
	twl4030_playback_device = 0;
	twl4030_call_device = 0;
	twl4030_recording_device = 0;
	
	twl4030_write(codec, 0x0e, 0x01); 
	twl4030_write(codec, 0x3a, 0x16); 

	twl4030_write(codec, 0x01,0x93);                
	twl4030_write(codec, 0x05,0x14);
	twl4030_write(codec, 0x06,0x14);
	twl4030_write(codec, 0x07,0x00);
	twl4030_write(codec, 0x08,0x00);
	if(!twl4030_fm_radio_mute_enable)
		twl4030_write(codec, 0x3e,0x20);
	twl4030_write(codec, 0x13,0x7f);
	twl4030_write(codec, 0x12,0x7f);
	twl4030_write(codec, 0x17, 0x0c);
	twl4030_write(codec, 0x21,0x00);
	twl4030_write(codec, 0x3f,0x00);
	twl4030_write(codec, 0x1c,0x07);
	twl4030_write(codec, 0x1b,0x07);

	switch(ucontrol->value.integer.value[0])
	{
		case OFF:
		break;

		case SPK:
			twl4030_write(codec, 0x26,0x28);   //TWL4030_PREDR_CTL
			twl4030_write(codec, 0x25,0x24);   //TWL4030_PREDL_CTL
		break;

		case HP3P:
		case HP4P:
			twl4030_write(codec, 0x22,0x24);   //TWL4030_PREDR_CTL
			twl4030_write(codec, 0x24,0x41);   //TWL4030_PREDL_CTL
			twl4030_write(codec, 0x24,0x42);   //TWL4030_PREDL_CTL
		break;

		default:
			printk("!!!!fmradio path setting failed!!!\n");

		break;
	}

    return 0;
}

static int twl4030_set_loopback_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int i=0;

	printk("twl4030_set_loopback_path value = %ld output_device = %d \n",ucontrol->value.integer.value[0], twl4030_fm_device);

	twl4030_fm_device = ucontrol->value.integer.value[0];

	twl4030_write(codec, 0x0e, 0x01); 
	twl4030_write(codec, 0x3a, 0x16); 


	switch(ucontrol->value.integer.value[0])
	{
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )	
		case OFF:
			mic_enable(codec, MAIN_MIC, 0);
			mic_enable(codec, HP_MIC, 0);
			max9877_set_force_out_mode(LOOP_BACK,ucontrol->value.integer.value[0]);
			break;
#endif			
		case RCV:
		mic_enable(codec, MAIN_MIC, 1);
		for(i=0;i<ARRAY_SIZE(Loopback_rcv);i++)
		{
			twl4030_write(codec, Loopback_rcv[i].reg, Loopback_rcv[i].value);
		}

		break;

		case SPK:
		mic_enable(codec, MAIN_MIC, 1);
		for(i=0;i<ARRAY_SIZE(Loopback_spk);i++)
		{
			  twl4030_write(codec, Loopback_spk[i].reg, Loopback_spk[i].value);
		}
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
		mic_enable(codec, HP_MIC, 0);				
		mic_enable(codec, MAIN_MIC, 1);
		twl4030_write(codec, 0x05, 0x11);
		max9877_set_force_out_mode(LOOP_BACK,SPK);
		twl4030_spk_line_out_sel(1);
#endif		
		break;

		case HP3P:
		case HP4P:
		mic_enable(codec, HP_MIC, 1);
		for(i=0;i<ARRAY_SIZE(Loopback_headset);i++)
		{
			 twl4030_write(codec, Loopback_headset[i].reg, Loopback_headset[i].value);
		}
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )		
		max9877_set_force_out_mode(LOOP_BACK,ucontrol->value.integer.value[0]);
#endif		
		break;

		default:
			printk("!!!!loopback path setting failed!!!\n");

		break;
	}

    return 0;
}
static int twl4030_set_idle_mode(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol,unsigned int codec_mode)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	printk("twl4030_set_idle_mode value = %ld \n",ucontrol->value.integer.value[0]);

	cancel_delayed_work(&codec_control_work);
	twl4030_remap = 1;
	twl4030_unset_remap();
	wake_unlock( &T2_wakelock);
	if(ucontrol->value.integer.value[0])  //off
	{
		twl4030_mic_mute_enable = 0;
		twl4030_playback_device = 0;
		twl4030_call_device = 0;
		twl4030_voip_device = 0;
		twl4030_recording_device = 0;
		twl4030_fm_device = 0;
		twl4030_rec_8k_enable = 0;
#ifdef VOICE_RECOGNITION
		twl4030_vr_mode = false;
#endif
		#ifdef USE_GPIO_MIC_SEL
		gpio_set_value(OMAP_GPIO_MIC_SEL, 0);
		#endif
		#ifdef USE_MAIN_MIC_LDO
		gpio_set_value(MAIN_MIC_BIAS_EN, 0);
		#endif
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )		
#ifdef USE_GPIO_MAIN_MIC_BIAS
		twl4030_main_mic_bias_control(false);
#endif 
#endif		

		twl4030_set_pcm_sel(BT_SEL_LOW_MODE);

#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
		twl4030_spk_line_out_sel(0);
#endif		
		twl4030_power_down(codec);
	}

	return 0;
}
// hskwon-ss-db05, to support mic mute/unmute for CTS test
static int twl4030_set_mic_mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol,unsigned int codec_mode)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	printk("twl4030_set_mic_mute value = %ld \n",ucontrol->value.integer.value[0]);

	if(ucontrol->value.integer.value[0])    // on, mute
	{
		if(twl4030_call_device == BT)
			twl4030_write(codec, 0x1e, 0x21); //tx power down
		else if(twl4030_call_device == HP4P)
			mic_enable(codec, HP_MIC, 0);
		else
			mic_enable(codec, MAIN_MIC, 0);

		twl4030_mic_mute_enable = 1;
	}
    else
    {
    	twl4030_mic_mute_enable = 0;

		if(twl4030_call_device == BT)
			twl4030_write(codec, 0x1e, 0x61); //tx power on
		else if(twl4030_call_device == HP4P)
			mic_enable(codec, HP_MIC, 1);
		else
			mic_enable(codec, MAIN_MIC, 1);
    }

	return 0;
}
#ifdef VOICE_RECOGNITION
static int twl4030_set_vr_mode(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol,unsigned int codec_mode)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	printk("twl4030_set_vr_mode value = %ld \n",ucontrol->value.integer.value[0]);

	if(ucontrol->value.integer.value[0])    // on, mute
	{
		twl4030_vr_mode = true;
	}
	else    // off, unmute
	{
		twl4030_vr_mode = false;
	}

	return 0;
}
#endif

static int twl4030_get_mic_mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol,unsigned int codec_mode)
{
	//printk("twl4030_get_mic_mute return is %d\n", twl4030_mic_mute_enable);
	return twl4030_mic_mute_enable;
}

#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
static int twl4030_get_voip_main_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	printk("twl4030_get_voip_main_path = %d !!!\n", voip_main_mode);
	return voip_main_mode;
}

static int twl4030_set_voip_main_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	voip_main_mode = ucontrol->value.integer.value[0];
	
	printk("twl4030_set_voip_main_path = %d !!!\n", voip_main_mode);
	
	switch(voip_main_mode)
	{
		case VOIP_MAIN_ON:
			//twl4030_set_voipcall_path(kcontrol, ucontrol, VOIP_CALL);
		    set_codec_gain(codec, VOIP_CALL, twl4030_playback_device);
			max9877_set_force_out_mode(VOIP_CALL,twl4030_playback_device);
			twl4030_voip_device = twl4030_playback_device;
			//twl4030_playback_device = 0;
			break;
		case VOIP_MAIN_OFF:
		default:
			set_codec_gain(codec, PLAY_BACK, twl4030_playback_device);
			max9877_set_force_out_mode(PLAY_BACK,twl4030_playback_device);
			//twl4030_set_idle_mode(kcontrol,ucontrol,IDLE_MODE);
			//max9877_set_force_out_mode(VOIP_CALL,OFF);
			break;
	}
	return 0;	
}
#endif

static int twl4030_get_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int i =0;

	while(audio_path[i] !=NULL)
	{
		if(!strncmp(audio_path[i], kcontrol->id.name,sizeof(audio_path[i])))
		{
			switch(i)
			{
				case PLAY_BACK:
				case VOICE_CALL:
				case VOICE_MEMO:
				case VOIP_CALL:
				case FM_RADIO:
				case IDLE_MODE:
				case LOOP_BACK:
				case VT_CALL:
					return 0;
				case MIC_MUTE:
	                return twl4030_get_mic_mute(kcontrol, ucontrol, i);
				default:
					return 0;
			}
		}
		i++;
	}
	return 0;
}

int twl4030_get_codec_mode(void)
{	
    P("twl4030_get_codec_mode = %d !!!\n", twl4030_mode);
	
    return twl4030_mode;
}
EXPORT_SYMBOL(twl4030_get_codec_mode);

static int twl4030_get_status(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    //P("twl4030_get_status codec_mode = %d !!!\n", twl4030_get_codec_mode());
    
    ucontrol->value.integer.value[0] = twl4030_get_codec_mode()+1;
    return 0;
}

static int twl4030_set_path(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	int i =0;
	int state = 0;

	printk("twl4030_set name=%s \n", kcontrol->id.name);


	#if defined(APPLY_AUDIOTEST_APP) && defined(APPLY_GAIN_INIT_FROM_INI)
	if(!is_read_gain){
		set_codec_gain_init(codec);
		is_read_gain = true;
		set_amp_gain_init();
	}
	#endif

	cancel_delayed_work( &codec_down_work );
	wake_unlock( &T2_wakelock);
	#if 0
	cancel_delayed_work_sync(&twl4030_register_dump_work_queue);
	#endif

#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
	twl4030_spk_line_out_sel(1);
#endif	
	twl4030_write(codec, 0x0e, 0x01);
	while(audio_path[i] !=NULL)
	{
		if(!strncmp(audio_path[i], kcontrol->id.name,sizeof(audio_path[i])))
		{
			if(i != MIC_MUTE)
			{
				twl4030_old_mode = twl4030_mode;
				twl4030_mode = i;
			}
			switch(i)
			{
				case PLAY_BACK:
					#ifndef VOICECALL_TUNE
					twl4030_unset_remap();
					state =twl4030_set_playback_path(kcontrol, ucontrol,i);
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
					if(state != TWL4030_SAME_DEVICE) {
						if(voip_main_mode == VOIP_MAIN_ON) {
							set_codec_gain(codec, VOIP_CALL, ucontrol->value.integer.value[0]);
							max9877_set_force_out_mode(VOIP_CALL,ucontrol->value.integer.value[0]);
						} else 
							set_codec_gain(codec, i, ucontrol->value.integer.value[0]);
					}
#else									
					if(state != TWL4030_SAME_DEVICE)
						set_codec_gain(codec, i, ucontrol->value.integer.value[0]);
#endif
					//schedule_delayed_work(&twl4030_register_dump_work_queue, 500);
					#endif
					return 0;

				case VOICE_CALL:
					twl4030_remap = 0; //for 0x1b gain setting.
					state = twl4030_set_voicecall_path(kcontrol, ucontrol,i);
					#ifndef VOICECALL_TUNE
					if(state != TWL4030_SAME_DEVICE)
					{
						set_codec_gain(codec, i, ucontrol->value.integer.value[0]);
						if(ucontrol->value.integer.value[0] == BT)
							schedule_delayed_work( &codec_control_work, 100 );
						else
							schedule_delayed_work( &codec_control_work, 0);
					}
					#endif
					return 0;

				case VOICE_MEMO:
					twl4030_set_voicememo_path(kcontrol, ucontrol, i);
					#ifndef REC_TUNE
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )					
					if(voip_main_mode == VOIP_MAIN_ON) {
						printk("[twl4030] current voip mode, don't need voicememo gain setting \n");
						return 0;
					}
#endif					
					set_codec_gain(codec, i, ucontrol->value.integer.value[0]);
					#endif
					return 0;

				case VT_CALL:
					twl4030_remap = 0; //for 0x1b gain setting.
					state = twl4030_set_voicecall_path(kcontrol, ucontrol,i);
					#ifndef VOICECALL_TUNE
					if(state != TWL4030_SAME_DEVICE)
					{
						set_codec_gain(codec, i, ucontrol->value.integer.value[0]);
						if(ucontrol->value.integer.value[0] == BT)
							schedule_delayed_work( &codec_control_work, 100 );
						else
							schedule_delayed_work( &codec_control_work, 0);
					}
					#endif
					return 0;

				case VOIP_CALL:
					twl4030_set_voipcall_path(kcontrol, ucontrol, i);
		           	set_codec_gain(codec, i, ucontrol->value.integer.value[0]);
					return 0;

				case FM_RADIO:
					twl4030_unset_remap();
					state = twl4030_set_fmradio_path(kcontrol, ucontrol, i);
					if(state != TWL4030_SAME_DEVICE)
						set_codec_gain(codec, i, ucontrol->value.integer.value[0]);
					twl4030_set_remap();
					return 0;

				case IDLE_MODE:
					#if !defined(REC_TUNE) && !defined(VOICECALL_TUNE)
					twl4030_set_idle_mode(kcontrol, ucontrol,i);
					#endif
					return 0;

				case MIC_MUTE:
               			twl4030_set_mic_mute(kcontrol, ucontrol, i);
                  			return 0;

#ifdef VOICE_RECOGNITION
				case VR_MODE:
	               		twl4030_set_vr_mode(kcontrol, ucontrol, i);
                  		return 0;
#endif

				default:
					return 0;
			}
		}
		i++;
	}
	return 0;
}

static int twl4030_get_dtmf_volume(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	//printk("twl4030_get_dtmf_volume\n");
	return 0;
}



static int twl4030_set_dtmf_volume(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	printk("twl4030_set_dtmf_volume %ld ", ucontrol->value.integer.value[0]);

	if(!twl4030_remap)
	{
		printk(" remap is not set force return!!\n");
		return 0;
	}
	else
		printk("\n");

    switch((int)ucontrol->value.integer.value[0])
    {

        case 0x1:
            twl4030_write(codec,TWL4030_REG_DTMF_PGA_CTL2,0x01);
            break;
        case 0x2:
            twl4030_write(codec,TWL4030_REG_DTMF_PGA_CTL2,0x02);
            break;
        case 0x3:
            twl4030_write(codec,TWL4030_REG_DTMF_PGA_CTL2,0x03);
            break;
        case 0x4:
            twl4030_write(codec,TWL4030_REG_DTMF_PGA_CTL2,0x04);
            break;
        case 0x5:
            twl4030_write(codec,TWL4030_REG_DTMF_PGA_CTL2,0x05);
            break;
        default:
            break;

    }
    return 0;
}

static int twl4030_get_dtmf_generator(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int twl4030_set_dtmf_generator(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	printk("twl4030_set_dtmf_generator %ld ", ucontrol->value.integer.value[0]);

	if(!twl4030_remap)
	{
		printk(" remap is not set force return!!\n");
		return 0;
	}
	else
	printk("\n");


	twl4030_write(codec,TWL4030_REG_DTMF_CTL,0x00);        //tone generator stop
	twl4030_write(codec,TWL4030_REG_PCMBTMUX, 0xa4);
	//twl4030_write(codec,TWL4030_REG_DTMF_TONOFF,0x14);
	twl4030_write(codec,TWL4030_REG_DTMF_TONOFF,0xab);
	// twl4030_write(codec,TWL4030_REG_DTMF_WANONOFF,0x33);
	twl4030_write(codec,TWL4030_REG_DTMF_WANONOFF,0x5b);

	if ((ucontrol->value.integer.value[0] <= 0x13) && (ucontrol->value.integer.value[0] != 0xf))
	{
		twl4030_write(codec,TWL4030_REG_DTMF_FREQSEL,ucontrol->value.integer.value[0]);
	}
	if ((ucontrol->value.integer.value[0] == 0x14) || (ucontrol->value.integer.value[0] == 0xf))
		twl4030_write(codec,TWL4030_REG_DTMF_CTL,0x00); //tone generator stop
	else
 		twl4030_write(codec,TWL4030_REG_DTMF_CTL,0x05); //dual tone,wobble,tone generator start

	return 0;
}


int twl4030_is_rec_8k_enable(void)
{
	printk("twl4030_get_8k_enable %d\n", twl4030_rec_8k_enable);
	return twl4030_rec_8k_enable;
}
EXPORT_SYMBOL(twl4030_is_rec_8k_enable);
static int twl4030_get_rec_8k_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return twl4030_rec_8k_enable;
}
static int twl4030_set_rec_8k_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	printk("twl4030_set_8k_enable %ld\n", ucontrol->value.integer.value[0]);
	twl4030_rec_8k_enable = ucontrol->value.integer.value[0];

	return 0;
}

static int twl4030_get_fm_radio_mute_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return twl4030_fm_radio_mute_enable;
}
static int twl4030_set_fm_radio_mute_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	printk("twl4030_set_fm_radio_mute_enable %ld\n", ucontrol->value.integer.value[0]);

	if(twl4030_fm_radio_mute_enable == ucontrol->value.integer.value[0])
	{
		P("twl4030_fm_radio_mute_enable is same\n");
		return 0;
	}
	
	twl4030_fm_radio_mute_enable = ucontrol->value.integer.value[0];
	
	if(ucontrol->value.integer.value[0]){
#if ( defined( CONFIG_MACH_SAMSUNG_LATONA ) ) // jypark72, to avoid build error
	 	max97000_power_down_mode(); 
#endif
	 	twl4030_write(codec, 0x3e,0x00);   	   	
#if ( defined( CONFIG_MACH_SAMSUNG_LATONA ) ) // jypark72, to avoid build error
		max97000_set_force_out_mode(FM_RADIO, twl4030_fm_device);		
#endif
   	}else{
#if ( defined( CONFIG_MACH_SAMSUNG_LATONA ) ) // jypark72, to avoid build error
   		max97000_power_down_mode(); 
#endif
   		twl4030_write(codec, 0x3e,0x20);
#if ( defined( CONFIG_MACH_SAMSUNG_LATONA ) ) // jypark72, to avoid build error
		max97000_set_force_out_mode(FM_RADIO, twl4030_fm_device);		
#endif
    }

	return 0;
}

static const struct soc_enum path_control_enum[]=
{
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(playback_path),playback_path),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voicecall_path),voicecall_path),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voicememo_path),voicememo_path),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(voip_path),voip_path),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fmradio_path),fmradio_path),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(idle_mode),idle_mode),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mic_mute),mic_mute), // hskwon-ss-db05, to support mic mute/unmute for CTS test
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(loopback_path),loopback_path),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(bt_dtmf_enum),bt_dtmf_enum),
#ifdef VOICE_RECOGNITION
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(vr_mode),vr_mode),
#endif
};

#endif //SAMSUNG_CUSTOMISATION
/* Digimic Left and right swapping */
static const char *twl4030_digimicswap_texts[] = {
	"Not swapped", "Swapped",
};

static const struct soc_enum twl4030_digimicswap_enum =
	SOC_ENUM_SINGLE(TWL4030_REG_MISC_SET_1, 0,
			ARRAY_SIZE(twl4030_digimicswap_texts),
			twl4030_digimicswap_texts);

static const struct snd_kcontrol_new twl4030_snd_controls[] = {
	/* Codec operation mode control */
	SOC_ENUM_EXT("Codec Operation Mode", twl4030_op_modes_enum,
		snd_soc_get_enum_double,
		snd_soc_put_twl4030_opmode_enum_double),

	/* Common playback gain controls */
	SOC_DOUBLE_R_TLV("DAC1 Digital Fine Playback Volume",
		TWL4030_REG_ARXL1PGA, TWL4030_REG_ARXR1PGA,
		0, 0x3f, 0, digital_fine_tlv),
	SOC_DOUBLE_R_TLV("DAC2 Digital Fine Playback Volume",
		TWL4030_REG_ARXL2PGA, TWL4030_REG_ARXR2PGA,
		0, 0x3f, 0, digital_fine_tlv),

	SOC_DOUBLE_R_TLV("DAC1 Digital Coarse Playback Volume",
		TWL4030_REG_ARXL1PGA, TWL4030_REG_ARXR1PGA,
		6, 0x2, 0, digital_coarse_tlv),
	SOC_DOUBLE_R_TLV("DAC2 Digital Coarse Playback Volume",
		TWL4030_REG_ARXL2PGA, TWL4030_REG_ARXR2PGA,
		6, 0x2, 0, digital_coarse_tlv),

	SOC_DOUBLE_R_TLV("DAC1 Analog Playback Volume",
		TWL4030_REG_ARXL1_APGA_CTL, TWL4030_REG_ARXR1_APGA_CTL,
		3, 0x12, 1, analog_tlv),
	SOC_DOUBLE_R_TLV("DAC2 Analog Playback Volume",
		TWL4030_REG_ARXL2_APGA_CTL, TWL4030_REG_ARXR2_APGA_CTL,
		3, 0x12, 1, analog_tlv),
	SOC_DOUBLE_R("DAC1 Analog Playback Switch",
		TWL4030_REG_ARXL1_APGA_CTL, TWL4030_REG_ARXR1_APGA_CTL,
		1, 1, 0),
	SOC_DOUBLE_R("DAC2 Analog Playback Switch",
		TWL4030_REG_ARXL2_APGA_CTL, TWL4030_REG_ARXR2_APGA_CTL,
		1, 1, 0),

	/* Common voice downlink gain controls */
	SOC_SINGLE_TLV("DAC Voice Digital Downlink Volume",
		TWL4030_REG_VRXPGA, 0, 0x31, 0, digital_voice_downlink_tlv),

	SOC_SINGLE_TLV("DAC Voice Analog Downlink Volume",
		TWL4030_REG_VDL_APGA_CTL, 3, 0x12, 1, analog_tlv),

	SOC_SINGLE("DAC Voice Analog Downlink Switch",
		TWL4030_REG_VDL_APGA_CTL, 1, 1, 0),

	/* Separate output gain controls */
	SOC_DOUBLE_R_TLV_TWL4030("PreDriv Playback Volume",
		TWL4030_REG_PREDL_CTL, TWL4030_REG_PREDR_CTL,
		4, 3, 0, output_tvl),

	SOC_DOUBLE_TLV_TWL4030("Headset Playback Volume",
		TWL4030_REG_HS_GAIN_SET, 0, 2, 3, 0, output_tvl),

	SOC_DOUBLE_R_TLV_TWL4030("Carkit Playback Volume",
		TWL4030_REG_PRECKL_CTL, TWL4030_REG_PRECKR_CTL,
		4, 3, 0, output_tvl),

	SOC_SINGLE_TLV_TWL4030("Earpiece Playback Volume",
		TWL4030_REG_EAR_CTL, 4, 3, 0, output_ear_tvl),

	/* Common capture gain controls */
	SOC_DOUBLE_R_TLV("TX1 Digital Capture Volume",
		TWL4030_REG_ATXL1PGA, TWL4030_REG_ATXR1PGA,
		0, 0x1f, 0, digital_capture_tlv),
	SOC_DOUBLE_R_TLV("TX2 Digital Capture Volume",
		TWL4030_REG_AVTXL2PGA, TWL4030_REG_AVTXR2PGA,
		0, 0x1f, 0, digital_capture_tlv),

	SOC_DOUBLE_TLV("Analog Capture Volume", TWL4030_REG_ANAMIC_GAIN,
		0, 3, 5, 0, input_gain_tlv),

	SOC_ENUM("AVADC Clock Priority", twl4030_avadc_clk_priority_enum),

	SOC_ENUM("HS ramp delay", twl4030_rampdelay_enum),

	SOC_ENUM("Vibra H-bridge mode", twl4030_vibradirmode_enum),
	SOC_ENUM("Vibra H-bridge direction", twl4030_vibradir_enum),

	SOC_ENUM("Digimic LR Swap", twl4030_digimicswap_enum),

#ifdef SAMSUNG_CUSTOMISATION
	/* normal play setting */
    SOC_ENUM_EXT("Playback Path", path_control_enum[0],
                twl4030_get_path,twl4030_set_path),
	/* voice call setting */
	SOC_ENUM_EXT("Voice Call Path", path_control_enum[1],
			twl4030_get_path,twl4030_set_path),

	/* voice memol setting */
	SOC_ENUM_EXT("Memo Path", path_control_enum[2],
			twl4030_get_path,twl4030_set_path),

	/* VOIP call setting */
	SOC_ENUM_EXT("VT Call Path", path_control_enum[1],
					twl4030_get_path, twl4030_set_path),
	SOC_ENUM_EXT("VOIP Call Path", path_control_enum[3],
			twl4030_get_path,twl4030_set_path),

	/* FM Radio Path setting */
	SOC_ENUM_EXT("FM Radio Path", path_control_enum[4],
					twl4030_get_path,twl4030_set_path),
	/* Idle Mode setting */
	SOC_ENUM_EXT("Idle Mode", path_control_enum[5],
			twl4030_get_status,twl4030_set_path),

	/* Mic Mute setting, hskwon-ss-db05, to support mic mute/unmute for CTS test */
	SOC_ENUM_EXT("Mic Mute", path_control_enum[6],
			twl4030_get_path,twl4030_set_path),

#ifdef VOICE_RECOGNITION
	SOC_ENUM_EXT("VR Mode", path_control_enum[6],
					twl4030_get_path,twl4030_set_path),
#endif

	SOC_ENUM_EXT("BT DTMF Volume", path_control_enum[8],
					twl4030_get_dtmf_volume, twl4030_set_dtmf_volume),

	SOC_ENUM_EXT("BT DTMF Generator", path_control_enum[8],
					twl4030_get_dtmf_generator, twl4030_set_dtmf_generator),

	SOC_ENUM_EXT("Loopback Path", path_control_enum[7],
					twl4030_get_path, twl4030_set_loopback_path),

	SOC_ENUM_EXT("Rec 8K Enable", path_control_enum[6],
					twl4030_get_rec_8k_enable, twl4030_set_rec_8k_enable),

	SOC_ENUM_EXT("FM Radio Mute Enable", path_control_enum[6],
					twl4030_get_fm_radio_mute_enable, twl4030_set_fm_radio_mute_enable),

#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )					
	/* voip call setting */
	SOC_ENUM_EXT("Codec Status", path_control_enum[8],
			twl4030_get_voip_main_path,twl4030_set_voip_main_path),						
#endif			
#endif // SAMSUNG_CUSTOMISATION
};

/* add non dapm controls */
static int twl4030_add_controls(struct snd_soc_codec *codec)
{
  int err, i;

  for (i = 0; i < ARRAY_SIZE(twl4030_snd_controls); i++) {
    err = snd_ctl_add(codec->card,
    snd_soc_cnew(&twl4030_snd_controls[i],
    codec, NULL));
    if (err < 0)
      return err;
  }

  return 0;
}

static const struct snd_soc_dapm_widget twl4030_dapm_widgets[] = {
	/* Left channel inputs */
	SND_SOC_DAPM_INPUT("MAINMIC"),
	SND_SOC_DAPM_INPUT("HSMIC"),
	SND_SOC_DAPM_INPUT("AUXL"),
	SND_SOC_DAPM_INPUT("CARKITMIC"),
	/* Right channel inputs */
	SND_SOC_DAPM_INPUT("SUBMIC"),
	SND_SOC_DAPM_INPUT("AUXR"),
	/* Digital microphones (Stereo) */
	SND_SOC_DAPM_INPUT("DIGIMIC0"),
	SND_SOC_DAPM_INPUT("DIGIMIC1"),

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("EARPIECE"),
	SND_SOC_DAPM_OUTPUT("PREDRIVEL"),
	SND_SOC_DAPM_OUTPUT("PREDRIVER"),
	SND_SOC_DAPM_OUTPUT("HSOL"),
	SND_SOC_DAPM_OUTPUT("HSOR"),
	SND_SOC_DAPM_OUTPUT("CARKITL"),
	SND_SOC_DAPM_OUTPUT("CARKITR"),
	SND_SOC_DAPM_OUTPUT("HFL"),
	SND_SOC_DAPM_OUTPUT("HFR"),
	SND_SOC_DAPM_OUTPUT("VIBRA"),

	/* AIF and APLL clocks for running DAIs (including loopback) */
	SND_SOC_DAPM_OUTPUT("Virtual HiFi OUT"),
	SND_SOC_DAPM_INPUT("Virtual HiFi IN"),
	SND_SOC_DAPM_OUTPUT("Virtual Voice OUT"),

	/* DACs */
	SND_SOC_DAPM_DAC("DAC Right1", "Right Front HiFi Playback",
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC Left1", "Left Front HiFi Playback",
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC Right2", "Right Rear HiFi Playback",
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC Left2", "Left Rear HiFi Playback",
			SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC Voice", "Voice Playback",
			SND_SOC_NOPM, 0, 0),

	/* Analog bypasses */
	SND_SOC_DAPM_SWITCH("Right1 Analog Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_abypassr1_control),
	SND_SOC_DAPM_SWITCH("Left1 Analog Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_abypassl1_control),
	SND_SOC_DAPM_SWITCH("Right2 Analog Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_abypassr2_control),
	SND_SOC_DAPM_SWITCH("Left2 Analog Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_abypassl2_control),
	SND_SOC_DAPM_SWITCH("Voice Analog Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_abypassv_control),

	/* Master analog loopback switch */
	SND_SOC_DAPM_SUPPLY("FM Loop Enable", TWL4030_REG_MISC_SET_1, 5, 0,
			    NULL, 0),

	/* Digital bypasses */
	SND_SOC_DAPM_SWITCH("Left Digital Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_dbypassl_control),
	SND_SOC_DAPM_SWITCH("Right Digital Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_dbypassr_control),
	SND_SOC_DAPM_SWITCH("Voice Digital Loopback", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_dbypassv_control),

	/* Digital mixers, power control for the physical DACs */
	SND_SOC_DAPM_MIXER("Digital R1 Playback Mixer",
			TWL4030_REG_AVDAC_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Digital L1 Playback Mixer",
			TWL4030_REG_AVDAC_CTL, 1, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Digital R2 Playback Mixer",
			TWL4030_REG_AVDAC_CTL, 2, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Digital L2 Playback Mixer",
			TWL4030_REG_AVDAC_CTL, 3, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Digital Voice Playback Mixer",
			TWL4030_REG_AVDAC_CTL, 4, 0, NULL, 0),

	/* Analog mixers, power control for the physical PGAs */
	SND_SOC_DAPM_MIXER("Analog R1 Playback Mixer",
			TWL4030_REG_ARXR1_APGA_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Analog L1 Playback Mixer",
			TWL4030_REG_ARXL1_APGA_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Analog R2 Playback Mixer",
			TWL4030_REG_ARXR2_APGA_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Analog L2 Playback Mixer",
			TWL4030_REG_ARXL2_APGA_CTL, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Analog Voice Playback Mixer",
			TWL4030_REG_VDL_APGA_CTL, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("APLL Enable", SND_SOC_NOPM, 0, 0, apll_event,
			    SND_SOC_DAPM_PRE_PMU|SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("AIF Enable", SND_SOC_NOPM, 0, 0, aif_event,
			    SND_SOC_DAPM_PRE_PMU|SND_SOC_DAPM_POST_PMD),

	/* Output MIXER controls */
	/* Earpiece */
	SND_SOC_DAPM_MIXER("Earpiece Mixer", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_earpiece_controls[0],
			ARRAY_SIZE(twl4030_dapm_earpiece_controls)),
	SND_SOC_DAPM_PGA_E("Earpiece PGA", SND_SOC_NOPM,
			0, 0, NULL, 0, earpiecepga_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	/* PreDrivL/R */
	SND_SOC_DAPM_MIXER("PredriveL Mixer", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_predrivel_controls[0],
			ARRAY_SIZE(twl4030_dapm_predrivel_controls)),
	SND_SOC_DAPM_PGA_E("PredriveL PGA", SND_SOC_NOPM,
			0, 0, NULL, 0, predrivelpga_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("PredriveR Mixer", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_predriver_controls[0],
			ARRAY_SIZE(twl4030_dapm_predriver_controls)),
	SND_SOC_DAPM_PGA_E("PredriveR PGA", SND_SOC_NOPM,
			0, 0, NULL, 0, predriverpga_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	/* HeadsetL/R */
	SND_SOC_DAPM_MIXER("HeadsetL Mixer", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_hsol_controls[0],
			ARRAY_SIZE(twl4030_dapm_hsol_controls)),
	SND_SOC_DAPM_PGA_E("HeadsetL PGA", SND_SOC_NOPM,
			0, 0, NULL, 0, headsetlpga_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("HeadsetR Mixer", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_hsor_controls[0],
			ARRAY_SIZE(twl4030_dapm_hsor_controls)),
	SND_SOC_DAPM_PGA_E("HeadsetR PGA", SND_SOC_NOPM,
			0, 0, NULL, 0, headsetrpga_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	/* CarkitL/R */
	SND_SOC_DAPM_MIXER("CarkitL Mixer", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_carkitl_controls[0],
			ARRAY_SIZE(twl4030_dapm_carkitl_controls)),
	SND_SOC_DAPM_PGA_E("CarkitL PGA", SND_SOC_NOPM,
			0, 0, NULL, 0, carkitlpga_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("CarkitR Mixer", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_carkitr_controls[0],
			ARRAY_SIZE(twl4030_dapm_carkitr_controls)),
	SND_SOC_DAPM_PGA_E("CarkitR PGA", SND_SOC_NOPM,
			0, 0, NULL, 0, carkitrpga_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),

	/* Output MUX controls */
	/* HandsfreeL/R */
	SND_SOC_DAPM_MUX("HandsfreeL Mux", SND_SOC_NOPM, 0, 0,
		&twl4030_dapm_handsfreel_control),
	SND_SOC_DAPM_SWITCH("HandsfreeL", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_handsfreelmute_control),
	SND_SOC_DAPM_PGA_E("HandsfreeL PGA", SND_SOC_NOPM,
			0, 0, NULL, 0, handsfreelpga_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MUX("HandsfreeR Mux", SND_SOC_NOPM, 5, 0,
		&twl4030_dapm_handsfreer_control),
	SND_SOC_DAPM_SWITCH("HandsfreeR", SND_SOC_NOPM, 0, 0,
			&twl4030_dapm_handsfreermute_control),
	SND_SOC_DAPM_PGA_E("HandsfreeR PGA", SND_SOC_NOPM,
			0, 0, NULL, 0, handsfreerpga_event,
			SND_SOC_DAPM_POST_PMU|SND_SOC_DAPM_POST_PMD),
	/* Vibra */
	SND_SOC_DAPM_MUX_E("Vibra Mux", TWL4030_REG_VIBRA_CTL, 0, 0,
			   &twl4030_dapm_vibra_control, vibramux_event,
			   SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_MUX("Vibra Route", SND_SOC_NOPM, 0, 0,
		&twl4030_dapm_vibrapath_control),

	/* Introducing four virtual ADC, since TWL4030 have four channel for
	   capture */
	SND_SOC_DAPM_ADC("ADC Virtual Left1", "Left Front Capture",
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC Virtual Right1", "Right Front Capture",
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC Virtual Left2", "Left Rear Capture",
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC Virtual Right2", "Right Rear Capture",
		SND_SOC_NOPM, 0, 0),

	/* Analog/Digital mic path selection.
	   TX1 Left/Right: either analog Left/Right or Digimic0
	   TX2 Left/Right: either analog Left/Right or Digimic1 */
	SND_SOC_DAPM_MUX("TX1 Capture Route", SND_SOC_NOPM, 0, 0,
		&twl4030_dapm_micpathtx1_control),
	SND_SOC_DAPM_MUX("TX2 Capture Route", SND_SOC_NOPM, 0, 0,
		&twl4030_dapm_micpathtx2_control),

	/* Analog input mixers for the capture amplifiers */
	SND_SOC_DAPM_MIXER("Analog Left",
		TWL4030_REG_ANAMICL, 4, 0,
		&twl4030_dapm_analoglmic_controls[0],
		ARRAY_SIZE(twl4030_dapm_analoglmic_controls)),
	SND_SOC_DAPM_MIXER("Analog Right",
		TWL4030_REG_ANAMICR, 4, 0,
		&twl4030_dapm_analogrmic_controls[0],
		ARRAY_SIZE(twl4030_dapm_analogrmic_controls)),

	SND_SOC_DAPM_PGA("ADC Physical Left",
		TWL4030_REG_AVADC_CTL, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ADC Physical Right",
		TWL4030_REG_AVADC_CTL, 1, 0, NULL, 0),

	SND_SOC_DAPM_PGA_E("Digimic0 Enable",
		TWL4030_REG_ADCMICSEL, 1, 0, NULL, 0,
		digimic_event, SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_E("Digimic1 Enable",
		TWL4030_REG_ADCMICSEL, 3, 0, NULL, 0,
		digimic_event, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_SUPPLY("micbias1 select", TWL4030_REG_MICBIAS_CTL, 5, 0,
			    NULL, 0),
	SND_SOC_DAPM_SUPPLY("micbias2 select", TWL4030_REG_MICBIAS_CTL, 6, 0,
			    NULL, 0),

	SND_SOC_DAPM_MICBIAS("Mic Bias 1", TWL4030_REG_MICBIAS_CTL, 0, 0),
	SND_SOC_DAPM_MICBIAS("Mic Bias 2", TWL4030_REG_MICBIAS_CTL, 1, 0),
	SND_SOC_DAPM_MICBIAS("Headset Mic Bias", TWL4030_REG_MICBIAS_CTL, 2, 0),

};

static const struct snd_soc_dapm_route intercon[] = {
	{"Digital L1 Playback Mixer", NULL, "DAC Left1"},
	{"Digital R1 Playback Mixer", NULL, "DAC Right1"},
	{"Digital L2 Playback Mixer", NULL, "DAC Left2"},
	{"Digital R2 Playback Mixer", NULL, "DAC Right2"},
	{"Digital Voice Playback Mixer", NULL, "DAC Voice"},

	/* Supply for the digital part (APLL) */
	{"Digital Voice Playback Mixer", NULL, "APLL Enable"},

	{"DAC Left1", NULL, "AIF Enable"},
	{"DAC Right1", NULL, "AIF Enable"},
	{"DAC Left2", NULL, "AIF Enable"},
	{"DAC Right1", NULL, "AIF Enable"},

	{"Digital R2 Playback Mixer", NULL, "AIF Enable"},
	{"Digital L2 Playback Mixer", NULL, "AIF Enable"},

	{"Analog L1 Playback Mixer", NULL, "Digital L1 Playback Mixer"},
	{"Analog R1 Playback Mixer", NULL, "Digital R1 Playback Mixer"},
	{"Analog L2 Playback Mixer", NULL, "Digital L2 Playback Mixer"},
	{"Analog R2 Playback Mixer", NULL, "Digital R2 Playback Mixer"},
	{"Analog Voice Playback Mixer", NULL, "Digital Voice Playback Mixer"},

	/* Internal playback routings */
	/* Earpiece */
	{"Earpiece Mixer", "Voice", "Analog Voice Playback Mixer"},
	{"Earpiece Mixer", "AudioL1", "Analog L1 Playback Mixer"},
	{"Earpiece Mixer", "AudioL2", "Analog L2 Playback Mixer"},
	{"Earpiece Mixer", "AudioR1", "Analog R1 Playback Mixer"},
	{"Earpiece PGA", NULL, "Earpiece Mixer"},
	/* PreDrivL */
	{"PredriveL Mixer", "Voice", "Analog Voice Playback Mixer"},
	{"PredriveL Mixer", "AudioL1", "Analog L1 Playback Mixer"},
	{"PredriveL Mixer", "AudioL2", "Analog L2 Playback Mixer"},
	{"PredriveL Mixer", "AudioR2", "Analog R2 Playback Mixer"},
	{"PredriveL PGA", NULL, "PredriveL Mixer"},
	/* PreDrivR */
	{"PredriveR Mixer", "Voice", "Analog Voice Playback Mixer"},
	{"PredriveR Mixer", "AudioR1", "Analog R1 Playback Mixer"},
	{"PredriveR Mixer", "AudioR2", "Analog R2 Playback Mixer"},
	{"PredriveR Mixer", "AudioL2", "Analog L2 Playback Mixer"},
	{"PredriveR PGA", NULL, "PredriveR Mixer"},
	/* HeadsetL */
	{"HeadsetL Mixer", "Voice", "Analog Voice Playback Mixer"},
	{"HeadsetL Mixer", "AudioL1", "Analog L1 Playback Mixer"},
	{"HeadsetL Mixer", "AudioL2", "Analog L2 Playback Mixer"},
	{"HeadsetL PGA", NULL, "HeadsetL Mixer"},
	/* HeadsetR */
	{"HeadsetR Mixer", "Voice", "Analog Voice Playback Mixer"},
	{"HeadsetR Mixer", "AudioR1", "Analog R1 Playback Mixer"},
	{"HeadsetR Mixer", "AudioR2", "Analog R2 Playback Mixer"},
	{"HeadsetR PGA", NULL, "HeadsetR Mixer"},
	/* CarkitL */
	{"CarkitL Mixer", "Voice", "Analog Voice Playback Mixer"},
	{"CarkitL Mixer", "AudioL1", "Analog L1 Playback Mixer"},
	{"CarkitL Mixer", "AudioL2", "Analog L2 Playback Mixer"},
	{"CarkitL PGA", NULL, "CarkitL Mixer"},
	/* CarkitR */
	{"CarkitR Mixer", "Voice", "Analog Voice Playback Mixer"},
	{"CarkitR Mixer", "AudioR1", "Analog R1 Playback Mixer"},
	{"CarkitR Mixer", "AudioR2", "Analog R2 Playback Mixer"},
	{"CarkitR PGA", NULL, "CarkitR Mixer"},
	/* HandsfreeL */
	{"HandsfreeL Mux", "Voice", "Analog Voice Playback Mixer"},
	{"HandsfreeL Mux", "AudioL1", "Analog L1 Playback Mixer"},
	{"HandsfreeL Mux", "AudioL2", "Analog L2 Playback Mixer"},
	{"HandsfreeL Mux", "AudioR2", "Analog R2 Playback Mixer"},
	{"HandsfreeL", "Switch", "HandsfreeL Mux"},
	{"HandsfreeL PGA", NULL, "HandsfreeL"},
	/* HandsfreeR */
	{"HandsfreeR Mux", "Voice", "Analog Voice Playback Mixer"},
	{"HandsfreeR Mux", "AudioR1", "Analog R1 Playback Mixer"},
	{"HandsfreeR Mux", "AudioR2", "Analog R2 Playback Mixer"},
	{"HandsfreeR Mux", "AudioL2", "Analog L2 Playback Mixer"},
	{"HandsfreeR", "Switch", "HandsfreeR Mux"},
	{"HandsfreeR PGA", NULL, "HandsfreeR"},
	/* Vibra */
	{"Vibra Mux", "AudioL1", "DAC Left1"},
	{"Vibra Mux", "AudioR1", "DAC Right1"},
	{"Vibra Mux", "AudioL2", "DAC Left2"},
	{"Vibra Mux", "AudioR2", "DAC Right2"},

	/* outputs */
	/* Must be always connected (for AIF and APLL) */
	{"Virtual HiFi OUT", NULL, "DAC Left1"},
	{"Virtual HiFi OUT", NULL, "DAC Right1"},
	{"Virtual HiFi OUT", NULL, "DAC Left2"},
	{"Virtual HiFi OUT", NULL, "DAC Right2"},
	/* Must be always connected (for APLL) */
	{"Virtual Voice OUT", NULL, "Digital Voice Playback Mixer"},
	/* Physical outputs */
	{"EARPIECE", NULL, "Earpiece PGA"},
	{"PREDRIVEL", NULL, "PredriveL PGA"},
	{"PREDRIVER", NULL, "PredriveR PGA"},
	{"HSOL", NULL, "HeadsetL PGA"},
	{"HSOR", NULL, "HeadsetR PGA"},
	{"CARKITL", NULL, "CarkitL PGA"},
	{"CARKITR", NULL, "CarkitR PGA"},
	{"HFL", NULL, "HandsfreeL PGA"},
	{"HFR", NULL, "HandsfreeR PGA"},
	{"Vibra Route", "Audio", "Vibra Mux"},
	{"VIBRA", NULL, "Vibra Route"},

	/* Capture path */
	/* Must be always connected (for AIF and APLL) */
	{"ADC Virtual Left1", NULL, "Virtual HiFi IN"},
	{"ADC Virtual Right1", NULL, "Virtual HiFi IN"},
	{"ADC Virtual Left2", NULL, "Virtual HiFi IN"},
	{"ADC Virtual Right2", NULL, "Virtual HiFi IN"},
	/* Physical inputs */
	{"Analog Left", "Main Mic Capture Switch", "MAINMIC"},
	{"Analog Left", "Headset Mic Capture Switch", "HSMIC"},
	{"Analog Left", "AUXL Capture Switch", "AUXL"},
	{"Analog Left", "Carkit Mic Capture Switch", "CARKITMIC"},

	{"Analog Right", "Sub Mic Capture Switch", "SUBMIC"},
	{"Analog Right", "AUXR Capture Switch", "AUXR"},

	{"ADC Physical Left", NULL, "Analog Left"},
	{"ADC Physical Right", NULL, "Analog Right"},

	{"Digimic0 Enable", NULL, "DIGIMIC0"},
	{"Digimic1 Enable", NULL, "DIGIMIC1"},

	{"DIGIMIC0", NULL, "micbias1 select"},
	{"DIGIMIC1", NULL, "micbias2 select"},

	/* TX1 Left capture path */
	{"TX1 Capture Route", "Analog", "ADC Physical Left"},
	{"TX1 Capture Route", "Digimic0", "Digimic0 Enable"},
	/* TX1 Right capture path */
	{"TX1 Capture Route", "Analog", "ADC Physical Right"},
	{"TX1 Capture Route", "Digimic0", "Digimic0 Enable"},
	/* TX2 Left capture path */
	{"TX2 Capture Route", "Analog", "ADC Physical Left"},
	{"TX2 Capture Route", "Digimic1", "Digimic1 Enable"},
	/* TX2 Right capture path */
	{"TX2 Capture Route", "Analog", "ADC Physical Right"},
	{"TX2 Capture Route", "Digimic1", "Digimic1 Enable"},

	{"ADC Virtual Left1", NULL, "TX1 Capture Route"},
	{"ADC Virtual Right1", NULL, "TX1 Capture Route"},
	{"ADC Virtual Left2", NULL, "TX2 Capture Route"},
	{"ADC Virtual Right2", NULL, "TX2 Capture Route"},

	{"ADC Virtual Left1", NULL, "AIF Enable"},
	{"ADC Virtual Right1", NULL, "AIF Enable"},
	{"ADC Virtual Left2", NULL, "AIF Enable"},
	{"ADC Virtual Right2", NULL, "AIF Enable"},

	/* Analog bypass routes */
	{"Right1 Analog Loopback", "Switch", "Analog Right"},
	{"Left1 Analog Loopback", "Switch", "Analog Left"},
	{"Right2 Analog Loopback", "Switch", "Analog Right"},
	{"Left2 Analog Loopback", "Switch", "Analog Left"},
	{"Voice Analog Loopback", "Switch", "Analog Left"},

	/* Supply for the Analog loopbacks */
	{"Right1 Analog Loopback", NULL, "FM Loop Enable"},
	{"Left1 Analog Loopback", NULL, "FM Loop Enable"},
	{"Right2 Analog Loopback", NULL, "FM Loop Enable"},
	{"Left2 Analog Loopback", NULL, "FM Loop Enable"},
	{"Voice Analog Loopback", NULL, "FM Loop Enable"},

	{"Analog R1 Playback Mixer", NULL, "Right1 Analog Loopback"},
	{"Analog L1 Playback Mixer", NULL, "Left1 Analog Loopback"},
	{"Analog R2 Playback Mixer", NULL, "Right2 Analog Loopback"},
	{"Analog L2 Playback Mixer", NULL, "Left2 Analog Loopback"},
	{"Analog Voice Playback Mixer", NULL, "Voice Analog Loopback"},

	/* Digital bypass routes */
	{"Right Digital Loopback", "Volume", "TX1 Capture Route"},
	{"Left Digital Loopback", "Volume", "TX1 Capture Route"},
	{"Voice Digital Loopback", "Volume", "TX2 Capture Route"},

	{"Digital R2 Playback Mixer", NULL, "Right Digital Loopback"},
	{"Digital L2 Playback Mixer", NULL, "Left Digital Loopback"},
	{"Digital Voice Playback Mixer", NULL, "Voice Digital Loopback"},

};

static int twl4030_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec->dapm, twl4030_dapm_widgets,
				 ARRAY_SIZE(twl4030_dapm_widgets));

	snd_soc_dapm_add_routes(codec->dapm, intercon, ARRAY_SIZE(intercon));

	return 0;
}

static int twl4030_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{

	printk("twl4030_set_bias_level %d\n", level);
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:	
#ifndef SAMSUNG_CUSTOMISATION		
		if (codec->dapm->bias_level == SND_SOC_BIAS_OFF)
			twl4030_codec_enable(codec, 1);
#endif
		break;
	case SND_SOC_BIAS_OFF:
		twl4030_codec_enable(codec, 0);
		break;
	}
	codec->dapm->bias_level = level;

	return 0;
}

static void twl4030_constraints(struct twl4030_priv *twl4030,
				struct snd_pcm_substream *mst_substream)
{
	struct snd_pcm_substream *slv_substream;

	/* Pick the stream, which need to be constrained */
	if (mst_substream == twl4030->master_substream)
		slv_substream = twl4030->slave_substream;
	else if (mst_substream == twl4030->slave_substream)
		slv_substream = twl4030->master_substream;
	else /* This should not happen.. */
		return;

	/* Set the constraints according to the already configured stream */
	snd_pcm_hw_constraint_minmax(slv_substream->runtime,
				SNDRV_PCM_HW_PARAM_RATE,
				twl4030->rate,
				twl4030->rate);

	snd_pcm_hw_constraint_minmax(slv_substream->runtime,
				SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
				twl4030->sample_bits,
				twl4030->sample_bits);

	snd_pcm_hw_constraint_minmax(slv_substream->runtime,
				SNDRV_PCM_HW_PARAM_CHANNELS,
				twl4030->channels,
				twl4030->channels);
}

/* In case of 4 channel mode, the RX1 L/R for playback and the TX2 L/R for
 * capture has to be enabled/disabled. */
static void twl4030_tdm_enable(struct snd_soc_codec *codec, int direction,
				int enable)
{
	u8 reg, mask;

	reg = twl4030_read_reg_cache(codec, TWL4030_REG_OPTION);

	if (direction == SNDRV_PCM_STREAM_PLAYBACK)
		mask = TWL4030_ARXL1_VRX_EN | TWL4030_ARXR1_EN;
	else
		mask = TWL4030_ATXL2_VTXL_EN | TWL4030_ATXR2_VTXR_EN;

	if (enable)
		reg |= mask;
	else
		reg &= ~mask;

	twl4030_write(codec, TWL4030_REG_OPTION, reg);
}

static int twl4030_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

	if (twl4030->master_substream) {
		twl4030->slave_substream = substream;
		/* The DAI has one configuration for playback and capture, so
		 * if the DAI has been already configured then constrain this
		 * substream to match it. */
		if (twl4030->configured)
			twl4030_constraints(twl4030, twl4030->master_substream);
	} else {
		if (!(twl4030_read_reg_cache(codec, TWL4030_REG_CODEC_MODE) &
			TWL4030_OPTION_1)) {
			/* In option2 4 channel is not supported, set the
			 * constraint for the first stream for channels, the
			 * second stream will 'inherit' this cosntraint */
			snd_pcm_hw_constraint_minmax(substream->runtime,
						SNDRV_PCM_HW_PARAM_CHANNELS,
						2, 2);
		}
		twl4030->master_substream = substream;
	}

	return 0;
}

static void twl4030_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

	if (twl4030->master_substream == substream)
		twl4030->master_substream = twl4030->slave_substream;

	twl4030->slave_substream = NULL;

	/* If all streams are closed, or the remaining stream has not yet
	 * been configured than set the DAI as not configured. */
	if (!twl4030->master_substream)
		twl4030->configured = 0;
	 else if (!twl4030->master_substream->runtime->channels)
		twl4030->configured = 0;

	 /* If the closing substream had 4 channel, do the necessary cleanup */
	if (substream->runtime->channels == 4)
		twl4030_tdm_enable(codec, substream->stream, 0);
}

static int twl4030_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 mode, old_mode, format, old_format;

	 /* If the substream has 4 channel, do the necessary setup */
	if (params_channels(params) == 4) {
		format = twl4030_read_reg_cache(codec, TWL4030_REG_AUDIO_IF);
		mode = twl4030_read_reg_cache(codec, TWL4030_REG_CODEC_MODE);

		/* Safety check: are we in the correct operating mode and
		 * the interface is in TDM mode? */
		if ((mode & TWL4030_OPTION_1) &&
		    ((format & TWL4030_AIF_FORMAT) == TWL4030_AIF_FORMAT_TDM))
			twl4030_tdm_enable(codec, substream->stream, 1);
		else
			return -EINVAL;
	}

	if (twl4030->configured)
		/* Ignoring hw_params for already configured DAI */
		return 0;

	/* bit rate */
	old_mode = twl4030_read_reg_cache(codec,
			TWL4030_REG_CODEC_MODE) & ~TWL4030_CODECPDZ;
	mode = old_mode & ~TWL4030_APLL_RATE;

	switch (params_rate(params)) {
	case 8000:
		mode |= TWL4030_APLL_RATE_8000;
		break;
	case 11025:
		mode |= TWL4030_APLL_RATE_11025;
		break;
	case 12000:
		mode |= TWL4030_APLL_RATE_12000;
		break;
	case 16000:
		mode |= TWL4030_APLL_RATE_16000;
		break;
	case 22050:
		mode |= TWL4030_APLL_RATE_22050;
		break;
	case 24000:
		mode |= TWL4030_APLL_RATE_24000;
		break;
	case 32000:
		mode |= TWL4030_APLL_RATE_32000;
		break;
	case 44100:
		mode |= TWL4030_APLL_RATE_44100;
		break;
	case 48000:
		mode |= TWL4030_APLL_RATE_48000;
		break;
	case 96000:
		mode |= TWL4030_APLL_RATE_96000;
		break;
	default:
		printk(KERN_ERR "TWL4030 hw params: unknown rate %d\n",
			params_rate(params));
		return -EINVAL;
	}

	/* sample size */
	old_format = twl4030_read_reg_cache(codec, TWL4030_REG_AUDIO_IF);
	format = old_format;
	format &= ~TWL4030_DATA_WIDTH;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		format |= TWL4030_DATA_WIDTH_16S_16W;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		format |= TWL4030_DATA_WIDTH_32S_24W;
		break;
	default:
		printk(KERN_ERR "TWL4030 hw params: unknown format %d\n",
			params_format(params));
		return -EINVAL;
	}

	if (format != old_format || mode != old_mode) {
		if (twl4030->codec_powered) {
			/*
			 * If the codec is powered, than we need to toggle the
			 * codec power.
			 */
			twl4030_codec_enable(codec, 0);
			twl4030_write(codec, TWL4030_REG_CODEC_MODE, mode);
			twl4030_write(codec, TWL4030_REG_AUDIO_IF, format);
			twl4030_codec_enable(codec, 1);
		} else {
			twl4030_write(codec, TWL4030_REG_CODEC_MODE, mode);
			twl4030_write(codec, TWL4030_REG_AUDIO_IF, format);
		}
	}

	/* Store the important parameters for the DAI configuration and set
	 * the DAI as configured */
	twl4030->configured = 1;
	twl4030->rate = params_rate(params);
	twl4030->sample_bits = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_SAMPLE_BITS)->min;
	twl4030->channels = params_channels(params);

	/* If both playback and capture streams are open, and one of them
	 * is setting the hw parameters right now (since we are here), set
	 * constraints to the other stream to match the current one. */
	if (twl4030->slave_substream)
		twl4030_constraints(twl4030, substream);

	return 0;
}

static int twl4030_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

	switch (freq) {
	case 19200000:
	case 26000000:
	case 38400000:
		break;
	default:
		dev_err(codec->dev, "Unsupported APLL mclk: %u\n", freq);
		return -EINVAL;
	}

	if ((freq / 1000) != twl4030->sysclk) {
		dev_err(codec->dev,
			"Mismatch in APLL mclk: %u (configured: %u)\n",
			freq, twl4030->sysclk * 1000);
		return -EINVAL;
	}

	return 0;
}

static int twl4030_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 old_format, format;

	/* get format */
	old_format = twl4030_read_reg_cache(codec, TWL4030_REG_AUDIO_IF);
	format = old_format;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		format &= ~(TWL4030_AIF_SLAVE_EN);
		format &= ~(TWL4030_CLK256FS_EN);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		format |= TWL4030_AIF_SLAVE_EN;
		format |= TWL4030_CLK256FS_EN;
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	format &= ~TWL4030_AIF_FORMAT;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format |= TWL4030_AIF_FORMAT_CODEC;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		format |= TWL4030_AIF_FORMAT_TDM;
		break;
	default:
		return -EINVAL;
	}

	if (format != old_format) {
    	/* clear CODECPDZ before changing format (codec requirement) */
        //twl4030_codec_enable(codec, 0);

        /* change format */
        twl4030_write(codec, TWL4030_REG_AUDIO_IF, format);

        /* set CODECPDZ afterwards */
        //twl4030_codec_enable(codec, 1);
	}

	return 0;
}

static int twl4030_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 reg = twl4030_read_reg_cache(codec, TWL4030_REG_AUDIO_IF);

	if (tristate)
		reg |= TWL4030_AIF_TRI_EN;
	else
		reg &= ~TWL4030_AIF_TRI_EN;

	return twl4030_write(codec, TWL4030_REG_AUDIO_IF, reg);
}

/* In case of voice mode, the RX1 L(VRX) for downlink and the TX2 L/R
 * (VTXL, VTXR) for uplink has to be enabled/disabled. */
static void twl4030_voice_enable(struct snd_soc_codec *codec, int direction,
				int enable)
{
	u8 reg, mask;

	reg = twl4030_read_reg_cache(codec, TWL4030_REG_OPTION);

	if (direction == SNDRV_PCM_STREAM_PLAYBACK)
		mask = TWL4030_ARXL1_VRX_EN;
	else
		mask = TWL4030_ATXL2_VTXL_EN | TWL4030_ATXR2_VTXR_EN;

	if (enable)
		reg |= mask;
	else
		reg &= ~mask;

	twl4030_write(codec, TWL4030_REG_OPTION, reg);
}

static int twl4030_voice_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 mode;

	/* If the system master clock is not 26MHz, the voice PCM interface is
	 * not avilable.
	 */
	if (twl4030->sysclk != 26000) {
		dev_err(codec->dev, "The board is configured for %u Hz, while"
			"the Voice interface needs 26MHz APLL mclk\n",
			twl4030->sysclk * 1000);
		return -EINVAL;
	}

	/* If the codec mode is not option2, the voice PCM interface is not
	 * avilable.
	 */
	mode = twl4030_read_reg_cache(codec, TWL4030_REG_CODEC_MODE)
		& TWL4030_OPT_MODE;

	if (mode != TWL4030_OPTION_2) {
		printk(KERN_ERR "TWL4030 voice startup: "
			"the codec mode is not option2\n");
		return -EINVAL;
	}

	return 0;
}

static void twl4030_voice_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	/* Enable voice digital filters */
	twl4030_voice_enable(codec, substream->stream, 0);
}

static int twl4030_voice_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 old_mode, mode;

	/* Enable voice digital filters */
	twl4030_voice_enable(codec, substream->stream, 1);

	/* bit rate */
	old_mode = twl4030_read_reg_cache(codec, TWL4030_REG_CODEC_MODE)
		& ~(TWL4030_CODECPDZ);
	mode = old_mode;

	switch (params_rate(params)) {
	case 8000:
		mode &= ~(TWL4030_SEL_16K);
		break;
	case 16000:
		mode |= TWL4030_SEL_16K;
		break;
	default:
		printk(KERN_ERR "TWL4030 voice hw params: unknown rate %d\n",
			params_rate(params));
		return -EINVAL;
	}

	if (mode != old_mode) {
		if (twl4030->codec_powered) {
			/*
			 * If the codec is powered, than we need to toggle the
			 * codec power.
			 */
			twl4030_codec_enable(codec, 0);
			twl4030_write(codec, TWL4030_REG_CODEC_MODE, mode);
			twl4030_codec_enable(codec, 1);
		} else {
			twl4030_write(codec, TWL4030_REG_CODEC_MODE, mode);
		}
	}

	return 0;
}

static int twl4030_voice_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);

	if (freq != 26000000) {
		dev_err(codec->dev, "Unsupported APLL mclk: %u, the Voice"
			"interface needs 26MHz APLL mclk\n", freq);
		return -EINVAL;
	}
	if ((freq / 1000) != twl4030->sysclk) {
		dev_err(codec->dev,
			"Mismatch in APLL mclk: %u (configured: %u)\n",
			freq, twl4030->sysclk * 1000);
		return -EINVAL;
	}
	return 0;
}

static int twl4030_voice_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct twl4030_priv *twl4030 = snd_soc_codec_get_drvdata(codec);
	u8 old_format, format;

	/* get format */
	old_format = twl4030_read_reg_cache(codec, TWL4030_REG_VOICE_IF);
	format = old_format;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		format &= ~(TWL4030_VIF_SLAVE_EN);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		format |= TWL4030_VIF_SLAVE_EN;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_NF:
		format &= ~(TWL4030_VIF_FORMAT);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		format |= TWL4030_VIF_FORMAT;
		break;
	default:
		return -EINVAL;
	}

	if (format != old_format) {
		if (twl4030->codec_powered) {
			/*
			 * If the codec is powered, than we need to toggle the
			 * codec power.
			 */
			twl4030_codec_enable(codec, 0);
			twl4030_write(codec, TWL4030_REG_VOICE_IF, format);
			twl4030_codec_enable(codec, 1);
		} else {
			twl4030_write(codec, TWL4030_REG_VOICE_IF, format);
		}
	}

	return 0;
}

static int twl4030_voice_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 reg = twl4030_read_reg_cache(codec, TWL4030_REG_VOICE_IF);

	if (tristate)
		reg |= TWL4030_VIF_TRI_EN;
	else
		reg &= ~TWL4030_VIF_TRI_EN;

	return twl4030_write(codec, TWL4030_REG_VOICE_IF, reg);
}

#define TWL4030_RATES	 (SNDRV_PCM_RATE_8000_48000)
#define TWL4030_FORMATS	 (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FORMAT_S24_LE)

static struct snd_soc_dai_ops twl4030_dai_hifi_ops = {
	.startup	= twl4030_startup,
	.shutdown	= twl4030_shutdown,
	.hw_params	= twl4030_hw_params,
	.set_sysclk	= twl4030_set_dai_sysclk,
	.set_fmt	= twl4030_set_dai_fmt,
	.set_tristate	= twl4030_set_tristate,
};

static struct snd_soc_dai_ops twl4030_dai_voice_ops = {
	.startup	= twl4030_voice_startup,
	.shutdown	= twl4030_voice_shutdown,
	.hw_params	= twl4030_voice_hw_params,
	.set_sysclk	= twl4030_voice_set_dai_sysclk,
	.set_fmt	= twl4030_voice_set_dai_fmt,
	.set_tristate	= twl4030_voice_set_tristate,
};

/* static */ struct snd_soc_dai_driver twl4030_dai[] = {
{
	.name = "twl4030-hifi",
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 2,
		.channels_max = 4,
		/* .rates = TWL4030_RATES | SNDRV_PCM_RATE_96000, */
		.rates = SNDRV_PCM_RATE_44100,
		.formats = TWL4030_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 4,
		/* .rates = TWL4030_RATES, */
		.rates = SNDRV_PCM_RATE_44100,
		.formats = TWL4030_FORMATS,},
	.ops = &twl4030_dai_hifi_ops,
},
{
	.name = "twl4030-voice",
	.playback = {
		.stream_name = "Voice Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &twl4030_dai_voice_ops,
},
};

static int twl4030_soc_suspend(struct snd_soc_codec *codec, pm_message_t state)
{

#ifdef SAMSUNG_CUSTOMISATION
	if((twl4030_mode !=VOICE_CALL)&&(twl4030_mode !=VOICE_MEMO)
		&&(twl4030_mode !=VOIP_CALL)&&(twl4030_mode !=FM_RADIO)&&(twl4030_mode !=VT_CALL))
	{
		printk("twl4030_suspend testmode is %d\n", twl4030_mode);
	        twl4030_playback_device = 0; //device off
	        twl4030_call_device = 0;
	        twl4030_voip_device = 0;	// hskwon-ss-cl31, added for FMC(VoIP) call path
	        twl4030_codec_suspended = true;
		twl4030_fm_device = 0;
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )		
		twl4030_spk_line_out_sel(0);
#endif		
#endif		
		twl4030_set_bias_level(codec, SND_SOC_BIAS_OFF);

#ifdef SAMSUNG_CUSTOMISATION
	}
#endif
	return 0;
}

static int twl4030_soc_resume(struct snd_soc_codec *codec)
{
#ifdef SAMSUNG_CUSTOMISATION
	P("");
	twl4030_codec_suspended = false;
	if((twl4030_mode !=VOICE_CALL)&&(twl4030_mode !=VOICE_MEMO)
  		&&(twl4030_mode !=VOIP_CALL)&&(twl4030_mode !=FM_RADIO)&&(twl4030_mode !=VT_CALL))
	{
#endif
		twl4030_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
#ifdef SAMSUNG_CUSTOMISATION
		twl4030_vintana1_power_enable(1);//twl power up
	}
#endif
	return 0;
}

static int twl4030_soc_probe(struct snd_soc_codec *codec)
{
	struct twl4030_priv *twl4030;

	twl4030 = kzalloc(sizeof(struct twl4030_priv), GFP_KERNEL);
	if (twl4030 == NULL) {
		printk("Can not allocate memroy\n");
		return -ENOMEM;
	}
	snd_soc_codec_set_drvdata(codec, twl4030);
	/* Set the defaults, and power up the codec */
	twl4030->sysclk = twl4030_codec_get_mclk() / 1000;
	codec->dapm->bias_level = SND_SOC_BIAS_OFF;
	codec->idle_bias_off = 1;

#ifdef SAMSUNG_CUSTOMISATION
       INIT_DELAYED_WORK( &codec_control_work, codec_control_work_handler ); //sec_lilkan
       INIT_DELAYED_WORK( &codec_down_work, codec_down_work_handler ); //sec_lilkan

       wake_lock_init( &T2_wakelock, WAKE_LOCK_SUSPEND, "twl4030_codec" );
#endif

	twl4030_init_chip(codec);

	snd_soc_add_controls(codec, twl4030_snd_controls,
				ARRAY_SIZE(twl4030_snd_controls));
	twl4030_add_widgets(codec);

#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )		
#ifdef ENABLE_LINEOUT
	if (gpio_request(SPK_LINE_OUT_SEL , "SPK_LINE_OUT_SEL") == 0)
			gpio_direction_output(SPK_LINE_OUT_SEL , 1);
		else
			printk(KERN_ERR "[twl4030] fail to gpio request SPK_LINE_OUT_SEL\n");	
#endif
	if (gpio_request(MAIN_MIC_BIAS_GPIO , "MAIN_MIC_BIAS") == 0)
			gpio_direction_output(MAIN_MIC_BIAS_GPIO , 1);
		else
			printk("[twl4030] fail to gpio request MAIN_MIC_BIAS\n");	
	twl4030_vintana1_power_enable(1);//twl power up
#endif	
	return 0;
}

static int twl4030_soc_remove(struct snd_soc_codec *codec)
{
	twl4030_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_twl4030 = {
	.probe = twl4030_soc_probe,
	.remove = twl4030_soc_remove,
	.suspend = twl4030_soc_suspend,
	.resume = twl4030_soc_resume,
	.read = twl4030_read_reg_cache,
	.write = twl4030_write,
#ifndef SAMSUNG_CUSTOMISATION
	.set_bias_level = twl4030_set_bias_level,
#endif
	.reg_cache_size = sizeof(twl4030_reg),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = twl4030_reg,
};

static int __devinit twl4030_codec_probe(struct platform_device *pdev)
{
	struct twl4030_codec_audio_data *pdata = pdev->dev.platform_data;


	if (!pdata) {
		dev_err(&pdev->dev, "platform_data is missing\n");
		return -EINVAL;
	}

	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_twl4030,
			twl4030_dai, ARRAY_SIZE(twl4030_dai));
}

static int __devexit twl4030_codec_remove(struct platform_device *pdev)
{
	struct twl4030_priv *twl4030 = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_codec(&pdev->dev);
	kfree(twl4030);
	return 0;
}

MODULE_ALIAS("platform:twl4030-codec");

static struct platform_driver twl4030_codec_driver = {
	.probe		= twl4030_codec_probe,
	.remove		= __devexit_p(twl4030_codec_remove),
	.driver		= {
		.name	= "twl4030-codec",
		.owner	= THIS_MODULE,
	},
};

static int __init twl4030_modinit(void)
{
	return platform_driver_register(&twl4030_codec_driver);
}
module_init(twl4030_modinit);

static void __exit twl4030_exit(void)
{
	platform_driver_unregister(&twl4030_codec_driver);
}
module_exit(twl4030_exit);

MODULE_DESCRIPTION("ASoC TWL4030 codec driver");
MODULE_AUTHOR("Steve Sakoman");
MODULE_LICENSE("GPL");
