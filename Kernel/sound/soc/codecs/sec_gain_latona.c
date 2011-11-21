
#include <sound/soc.h>
#include <linux/slab.h>

 /* Register descriptions are here */
#include <linux/mfd/twl4030-codec.h>

#include "./sec_gain.h"

#define SEC_AUDIO_DEBUG 1

#if SEC_AUDIO_DEBUG
#define P(format,...)\
		printk("audio:%s() " format "\n", __func__, ## __VA_ARGS__);
#else
#define P(format,...)
#endif

//default gain value from oscar
static unsigned int music_ear_gain[5] = {(0x0 << 6) | 0x3f, (0x0 << 6) | 0x3f, 
											 (0x3 << 3),  (0x3 << 3), (0x1 << 2)|0x1};
static unsigned int music_spk_gain[6] = {(0x0 << 6) | 0x3f, (0x0 << 6) | 0x3f, 
												(0x5<<3), (0x5<<3), (0x2<<4), (0x2<<4)};
#ifdef APPLY_AUDIOTEST_APP
typedef struct applyReg
{
	unsigned int reg;
	unsigned int val;
} applyFileReg;

#define MAX_FILE_SIZE		200
#define MAX_REG_NUM 		15

applyFileReg regFromFile[MAX_REG_NUM];

#include <linux/fs.h>
#include <linux/vmalloc.h>

static int twl4030_set_reg_from_file(struct snd_soc_codec *codec, char* filename, int mode);
static char StringToHexFor16Bit( char *ss );
void set_codec_gain_init(struct snd_soc_codec *codec);

extern int twl4030_get_voicecall_state();
#ifdef VOICE_RECOGNITION
extern int twl4030_is_vr_mode(void);
#endif
#endif

void set_codec_gain(struct snd_soc_codec *codec, int mode, int device)
{
	P("mode : %d, device : %d", mode, device);	
	if(mode == PLAY_BACK)
	{
		switch(device)
		{
			case HP3P: 
			case HP4P: 
				if(!get_sec_gain_test_mode()){
					//Comon
					twl4030_write(codec, 0x12, music_ear_gain[0]); // ARXR2PGA (fine[0:5], Corse[6:7])
					twl4030_write(codec, 0x13, music_ear_gain[1]); // ARXL2PGA (fine[0:5], Corse[6:7])
					twl4030_modify(codec, 0x1b, music_ear_gain[2], ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
					twl4030_modify(codec, 0x1C, music_ear_gain[3], ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

					twl4030_write(codec, 0x23, music_ear_gain[4]); //HS_GAIN(L[0:1], R[2:3])
					return ;
				}
#if defined(APPLY_AUDIOTEST_APP) && defined(APPLY_GAIN_INIT_FROM_INI)
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/MusicEar.ini", 0);
#elif defined(APPLY_AUDIOTEST_APP)
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/MusicEar.ini", 0);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/MusicEar.ini", 0);
#endif
				break;
			case SPK: 
			case SPK_HP: 
			case EXTRA_SPEAKER:
				if(!get_sec_gain_test_mode()){
					//Comon
					twl4030_write(codec, 0x12, music_spk_gain[0]); // ARXR2PGA (fine[0:5], Corse[6:7])
					twl4030_write(codec, 0x13, music_spk_gain[1]); // ARXL2PGA (fine[0:5], Corse[6:7])
					twl4030_modify(codec, 0x1b, music_spk_gain[2], ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
					twl4030_modify(codec, 0x1C, music_spk_gain[3], ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

					twl4030_modify(codec, 0x25, music_spk_gain[4], PREDL_CTL_GAIN_MASK); //PREDL_CTL [4:5]
					twl4030_modify(codec, 0x26, music_spk_gain[5], PREDL_CTL_GAIN_MASK); //PREDR_CTL[4:5]
					if(device == SPK_HP)
						twl4030_write(codec, 0x23, (0x3 << 2)|0x3); //HS_GAIN(L[0:1], R[2:3])

					return ;
				}
#if defined(APPLY_AUDIOTEST_APP) && defined(APPLY_GAIN_INIT_FROM_INI)
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/MusicSpk.ini", 0);
#elif defined(APPLY_AUDIOTEST_APP)
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/MusicSpk.ini", 0);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/MusicSpk.ini", 0);
#endif
				break;
			case OFF:
				break;
			case RCV: 
				//Comon
				twl4030_write(codec, 0x12, (0x1 << 6) | 0x2F); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x1 << 6) | 0x2F); // ARXL2PGA (fine[0:5], Corse[6:7])
				twl4030_modify(codec, 0x1b, (0x3 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
				twl4030_modify(codec, 0x1C, (0x3 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				twl4030_modify(codec, 0x21, (0x2 << 4), EAR_CTL_GAIN_MASK); //EAR_CTL [4:5]		
				return ;
				break;

	
			case BT: 
				//Comon
				printk("TWL4030 doese NOT Support BT Play");
				break;
			default:
				break;
		}
	}
	else if (mode == VOICE_MEMO)
	{
		switch(device)
		{
			case OFF:
				break;
			case MAIN_MIC: 
#ifndef APPLY_AUDIOTEST_APP
				if(twl4030_is_rec_8k_enable())
				{
					printk("set voicememo 8k gain\n");
					twl4030_write(codec, 0x48, 0x12); // ANAMIC_GAIN (L[0:2], R[3:5])
					twl4030_write(codec, 0x0A, 0x1d); //AVTXL1PGA [0:7]
					twl4030_write(codec, 0x0B, 0x1d); //AVTXR1PGA [0:7]
				}
				else
				{
					printk("set voicememo 16k gain\n");
					twl4030_write(codec, 0x48, 0x1b); // ANAMIC_GAIN (L[0:2], R[3:5])
					twl4030_write(codec, 0x0A, 0x1b); //AVTXL1PGA [0:7]
					twl4030_write(codec, 0x0B, 0x1b); //AVTXR1PGA [0:7]
				}
#else
					if(twl4030_is_vr_mode())
					{
						if(!get_sec_gain_test_mode())
							twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceRecMainMic.ini", 0);
						else
							twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VoiceRecMainMic.ini", 0);
					}else{
						if(!get_sec_gain_test_mode())
							twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/RecMainMic.ini", 0);
						else
							twl4030_set_reg_from_file(codec, "/sdcard/external_sd/RecMainMic.ini", 0);
					}
#endif
				break;
			case SUB_MIC: 
#ifndef APPLY_AUDIOTEST_APP
				if(twl4030_is_rec_8k_enable())
				{
					printk("set voicememo 8k gain\n");
					twl4030_write(codec, 0x48, 0x12); // ANAMIC_GAIN (L[0:2], R[3:5])
					twl4030_write(codec, 0x0A, 0x1d); //AVTXL1PGA [0:7]
					twl4030_write(codec, 0x0B, 0x1d); //AVTXR1PGA [0:7]
				}
				else
				{
					printk("set voicememo 16k gain\n");
					twl4030_write(codec, 0x48, 0x1b); // ANAMIC_GAIN (L[0:2], R[3:5])
					twl4030_write(codec, 0x0A, 0x1b); //AVTXL1PGA [0:7]
					twl4030_write(codec, 0x0B, 0x1b); //AVTXR1PGA [0:7]
				}
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/RecMainMic.ini", 0);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/RecMainMic.ini", 0);
				
#endif
				break;
			case HP_MIC: 
#ifndef APPLY_AUDIOTEST_APP
				if(twl4030_is_rec_8k_enable())
				{
					printk("set voicememo 8k gain\n");
					twl4030_write(codec, 0x48, 0x12); // ANAMIC_GAIN (L[0:2], R[3:5])
					twl4030_write(codec, 0x0A, 0x1b); //AVTXL1PGA [0:7]
					twl4030_write(codec, 0x0B, 0x1b); //AVTXR1PGA [0:7]
				}
				else
				{
					printk("set voicememo 16k gain\n");
					twl4030_write(codec, 0x48, 0x1b); // ANAMIC_GAIN (L[0:2], R[3:5])
					twl4030_write(codec, 0x0A, 0x1b); //AVTXL1PGA [0:7]
					twl4030_write(codec, 0x0B, 0x1b); //AVTXR1PGA [0:7]
				}
#else
				if(twl4030_is_vr_mode())
				{
					if(!get_sec_gain_test_mode())
						twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceRecHeadSetMic.ini", 0);
					else
						twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VoiceRecHeadSetMic.ini", 0);
				}else{
					if(!get_sec_gain_test_mode())
						twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/RecHeadSetMic.ini", 0);
					else
						twl4030_set_reg_from_file(codec, "/sdcard/external_sd/RecHeadSetMic.ini", 0);
				}
#endif
				break;
			case BT_MIC: 
				printk("TWL4030 doese NOT support BT Mic Recording");
				break;
			default:
				break;
		}
	}
	else if (mode == VOICE_CALL)
	{
		switch(device)
		{
			case OFF:
				break;
			case RCV: 
				//play when voice
				twl4030_write(codec, 0x12, (0x2 << 6) | 0x39); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x2 << 6) | 0x39); // ARXL2PGA (fine[0:5], Corse[6:7])
				
#ifndef APPLY_AUDIOTEST_APP				
				//TX Common
				twl4030_write(codec, 0x48, (0x0 << 3) | 0x03); // ANAMIC_GAIN (L[0:2], R[3:5]) 18dB
				twl4030_write(codec, 0x0C, 0x0a); //AVTXL2PGA [0:7] 0dB
				twl4030_write(codec, 0x0D, 0x15); //AVTXR2PGA [0:7] 0dB

				//RX Common
				twl4030_write(codec, 0x14, 0x25); //VRXPGA [0:7] 0dB
				twl4030_write(codec, 0x16, 0x19); //VRX2ARXPGA [0:7]  -3dB
				twl4030_write(codec, 0x1b, 0x23);//ARXL2_APGA_CTL [3:7] 0dB
				twl4030_write(codec, 0x1c, 0x23);//ARXR2_APGA_CTL [3:7] 0dB
//				twl4030_modify(codec, 0x1b, (0x4 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7] 0dB
//				twl4030_modify(codec, 0x1C, (0x4 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7] 0dB

				//rx depend output
				//twl4030_modify(codec, 0x21, (0x3 << 4), EAR_CTL_GAIN_MASK); //EAR_CTL [4:5] 0dB
				twl4030_write(codec, 0x21, 0x34);		//EAR_CTL [4:5] 0dB	
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceCallRcv.ini", 1);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VoiceCallRcv.ini", 1);
#endif				
				break;
			case SPK: 
				//play when voice
				twl4030_write(codec, 0x12, (0x2 << 6) | 0x2f); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x2 << 6) | 0x2f); // ARXL2PGA (fine[0:5], Corse[6:7])
				
#ifndef APPLY_AUDIOTEST_APP				
				//TX Common
				twl4030_write(codec, 0x48, (0x0 << 3) | 0x4); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x0C, 0x09); //AVTXL2PGA [0:7] 3db
				twl4030_write(codec, 0x0D, 0x03); //AVTXR2PGA [0:7] 3db

				//RX Common
				twl4030_write(codec, 0x14, 0x25); //VRXPGA [0:7] 3db
				twl4030_write(codec, 0x16, 0x19); //VRX2ARXPGA [0:7] -3db
				twl4030_write(codec, 0x1b, 0x23);//ARXL2_APGA_CTL [3:7] 0dB
				twl4030_write(codec, 0x1c, 0x23);//ARXR2_APGA_CTL [3:7] 0dB
//				twl4030_modify(codec, 0x1b, (0x0 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
//				twl4030_modify(codec, 0x1C, (0x0 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend output
				twl4030_modify(codec, 0x25, (0x2 << 4), PREDL_CTL_GAIN_MASK); //PREDL_CTL [4:5]
				twl4030_modify(codec, 0x26, (0x2 << 4), PREDL_CTL_GAIN_MASK); //PREDR_CTL[4:5]
			//	twl4030_write(codec, 0x25, 0x2c);
			//	twl4030_write(codec, 0x26, 0x2c);
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceCallSpk.ini",2);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VoiceCallSpk.ini", 2);
#endif				
				break;
			case HP3P: 
				//play when voice
				twl4030_write(codec, 0x12, (0x1 << 6) | 0x2f); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x1 << 6) | 0x2f); // ARXL2PGA (fine[0:5], Corse[6:7])
				
#ifndef APPLY_AUDIOTEST_APP
				//TX Common must be match with rev tx gain
				//twl4030_write(codec, 0x48, 0x24); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x48, (0x0 << 3) | 0x3); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x0C, 0x00); //AVTXL2PGA [0:7]
				twl4030_write(codec, 0x0D, 0x00); //AVTXR2PGA [0:7]				

				//RX Common must be match with hp4p rx gain
				twl4030_write(codec, 0x14, 0x26); //VRXPGA [0:7]
				twl4030_write(codec, 0x16, 0x12); //VRX2ARXPGA [0:7]
				twl4030_modify(codec, 0x1b, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
				twl4030_modify(codec, 0x1C, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend output
				twl4030_write(codec, 0x23, (0x1 << 2) | 0x1); //HS_GAIN(L[0:1], R[2:3])
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceCall3pEar.ini",3);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VoiceCall3pEar.ini", 3);
#endif				
				break;
			case HP4P: 
				//play when voice
				twl4030_write(codec, 0x12, (0x1 << 6) | 0x2f); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x1 << 6) | 0x2f); // ARXL2PGA (fine[0:5], Corse[6:7])
				
#ifndef APPLY_AUDIOTEST_APP
				//TX Common
				//twl4030_write(codec, 0x48, 0x24); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x48, (0x0 << 3) | 0x1); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x0C, 0x16); //AVTXL2PGA [0:7]
				twl4030_write(codec, 0x0D, 0x16); //AVTXR2PGA [0:7]				

				//RX Common
				twl4030_write(codec, 0x14, 0x26); //VRXPGA [0:7]
				twl4030_write(codec, 0x16, 0x12); //VRX2ARXPGA [0:7]
				twl4030_modify(codec, 0x1b, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
				twl4030_modify(codec, 0x1C, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend output
				twl4030_write(codec, 0x23, (0x1 << 2) | 0x1); //HS_GAIN(L[0:1], R[2:3])
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceCall4pEar.ini",3);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VoiceCall4pEar.ini", 3);
#endif	

				break;
			case BT: 
#ifndef APPLY_AUDIOTEST_APP
				twl4030_write(codec, 0x1F, (0x5 << 4)| 0x5); //BTPGA(RX[0:3], Tx[4:7])
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceCallBT.ini",0);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VoiceCallBT.ini", 0);
#endif
				break;
			default:
				break;
		}		
	}
	else if (mode == VT_CALL)
	{
		switch(device)
		{
			case OFF:
				break;
			case RCV: 
				//play when voice
				twl4030_write(codec, 0x12, (0x2 << 6) | 0x39); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x2 << 6) | 0x39); // ARXL2PGA (fine[0:5], Corse[6:7])
				
#ifndef APPLY_AUDIOTEST_APP
				//TX Common
				twl4030_write(codec, 0x48, (0x0 << 3) | 0x03); // ANAMIC_GAIN (L[0:2], R[3:5]) 18dB
				twl4030_write(codec, 0x0C, 0x0a); //AVTXL2PGA [0:7] 0dB
				twl4030_write(codec, 0x0D, 0x15); //AVTXR2PGA [0:7] 0dB

				//RX Common
				twl4030_write(codec, 0x14, 0x25); //VRXPGA [0:7] 0dB
				twl4030_write(codec, 0x16, 0x19); //VRX2ARXPGA [0:7]  -3dB
				twl4030_write(codec, 0x1b, 0x23);//ARXL2_APGA_CTL [3:7] 0dB
				twl4030_write(codec, 0x1c, 0x23);//ARXR2_APGA_CTL [3:7] 0dB
//				twl4030_modify(codec, 0x1b, (0x4 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7] 0dB
//				twl4030_modify(codec, 0x1C, (0x4 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7] 0dB

				//rx depend output
				//twl4030_modify(codec, 0x21, (0x3 << 4), EAR_CTL_GAIN_MASK); //EAR_CTL [4:5] 0dB
				twl4030_write(codec, 0x21, 0x34);		//EAR_CTL [4:5] 0dB	
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceCallRcv.ini", 1);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VtCallRcv.ini", 1);
#endif				
				break;
			case SPK: 
				//play when voice
				twl4030_write(codec, 0x12, (0x1 << 6) | 0x2f); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x1 << 6) | 0x2f); // ARXL2PGA (fine[0:5], Corse[6:7])
				
#ifndef APPLY_AUDIOTEST_APP
				//TX Common
				twl4030_write(codec, 0x48, (0x0 << 3) | 0x4); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x0C, 0x09); //AVTXL2PGA [0:7] 3db
				twl4030_write(codec, 0x0D, 0x03); //AVTXR2PGA [0:7] 3db

				//RX Common
				twl4030_write(codec, 0x14, 0x25); //VRXPGA [0:7] 3db
				twl4030_write(codec, 0x16, 0x19); //VRX2ARXPGA [0:7] -3db
				twl4030_write(codec, 0x1b, 0x23);//ARXL2_APGA_CTL [3:7] 0dB
				twl4030_write(codec, 0x1c, 0x23);//ARXR2_APGA_CTL [3:7] 0dB
//				twl4030_modify(codec, 0x1b, (0x0 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
//				twl4030_modify(codec, 0x1C, (0x0 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend output
				twl4030_modify(codec, 0x25, (0x2 << 4), PREDL_CTL_GAIN_MASK); //PREDL_CTL [4:5]
				twl4030_modify(codec, 0x26, (0x2 << 4), PREDL_CTL_GAIN_MASK); //PREDR_CTL[4:5]
			//	twl4030_write(codec, 0x25, 0x2c);
			//	twl4030_write(codec, 0x26, 0x2c);
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceCallSpk.ini", 2);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VtCallSpk.ini", 2);
#endif				
				break;
			case HP3P: 
				//play when voice
				twl4030_write(codec, 0x12, (0x1 << 6) | 0x2f); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x1 << 6) | 0x2f); // ARXL2PGA (fine[0:5], Corse[6:7])
				
#ifndef APPLY_AUDIOTEST_APP
				//TX Common must be match with rev tx gain
				//twl4030_write(codec, 0x48, 0x24); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x48, (0x0 << 3) | 0x3); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x0C, 0x00); //AVTXL2PGA [0:7]
				twl4030_write(codec, 0x0D, 0x00); //AVTXR2PGA [0:7]				

				//RX Common must be match with hp4p rx gain
				twl4030_write(codec, 0x14, 0x26); //VRXPGA [0:7]
				twl4030_write(codec, 0x16, 0x12); //VRX2ARXPGA [0:7]
				twl4030_modify(codec, 0x1b, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
				twl4030_modify(codec, 0x1C, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend output
				twl4030_write(codec, 0x23, (0x1 << 2) | 0x1); //HS_GAIN(L[0:1], R[2:3])
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceCall3pEar.ini", 3);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VtCall3pEar.ini", 3);
#endif				
				break;
			case HP4P: 
				//play when voice
				twl4030_write(codec, 0x12, (0x1 << 6) | 0x2f); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x1 << 6) | 0x2f); // ARXL2PGA (fine[0:5], Corse[6:7])
				
#ifndef APPLY_AUDIOTEST_APP				
				//TX Common
				//twl4030_write(codec, 0x48, 0x24); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x48, (0x0 << 3) | 0x1); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x0C, 0x16); //AVTXL2PGA [0:7]
				twl4030_write(codec, 0x0D, 0x16); //AVTXR2PGA [0:7]				

				//RX Common
				twl4030_write(codec, 0x14, 0x26); //VRXPGA [0:7]
				twl4030_write(codec, 0x16, 0x12); //VRX2ARXPGA [0:7]
				twl4030_modify(codec, 0x1b, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
				twl4030_modify(codec, 0x1C, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend output
				twl4030_write(codec, 0x23, (0x1 << 2) | 0x1); //HS_GAIN(L[0:1], R[2:3])
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceCall4pEar.ini", 3);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VtCall4pEar.ini", 3);
#endif	
				break;
			case BT: 
#ifndef APPLY_AUDIOTEST_APP
				twl4030_write(codec, 0x1F, (0x5 << 4)| 0x5); //BTPGA(RX[0:3], Tx[4:7])
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/VoiceCallBT.ini", 0);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/VtCallBT.ini", 0);
#endif
				break;
			default:
				break;
		}		
	}
	else if (mode == VOIP_CALL)
	{
		switch(device)
		{
			case OFF:
				break;
			case RCV: 
				//TX Common
				twl4030_write(codec, 0x48, (0x4 << 3) | 0x4); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x0C, 0x08); //AVTXL2PGA [0:7]
				twl4030_write(codec, 0x0D, 0x08); //AVTXR2PGA [0:7]

				//RX Common
				twl4030_write(codec, 0x12, (0x1 << 6) | 0x2F); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x1 << 6) | 0x2F); // ARXL2PGA (fine[0:5], Corse[6:7])
				twl4030_modify(codec, 0x1b, (0xd << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
				twl4030_modify(codec, 0x1C, (0xd << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend
				twl4030_modify(codec, 0x21, (0x3 << 4), EAR_CTL_GAIN_MASK); //EAR_CTL [4:5]				

				break;
			case SPK: 
				//TX Common
				twl4030_write(codec, 0x48, (0x4 << 3) | 0x4); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x0C, 0x08); //AVTXL2PGA [0:7]
				twl4030_write(codec, 0x0D, 0x08); //AVTXR2PGA [0:7]

				//RX Common
				twl4030_write(codec, 0x12, (0x1 << 6) | 0x2F); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x1 << 6) | 0x2F); // ARXL2PGA (fine[0:5], Corse[6:7])
				twl4030_modify(codec, 0x1b, (0x3 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
				twl4030_modify(codec, 0x1C, (0x3 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend output
				twl4030_modify(codec, 0x25, (0x2 << 4), PREDL_CTL_GAIN_MASK); //PREDL_CTL [4:5]
				twl4030_modify(codec, 0x26, (0x2 << 4), PREDL_CTL_GAIN_MASK); //PREDR_CTL[4:5]

				break;
			case HP3P: 
				//TX Common
				twl4030_write(codec, 0x48, 0x02); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x0C, 0xD); //AVTXL2PGA [0:7]
				twl4030_write(codec, 0x0D, 0xD); //AVTXR2PGA [0:7]	

				//RX Common
				twl4030_write(codec, 0x12, (0x1 << 6) | 0x2F); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x1 << 6) | 0x2F); // ARXL2PGA (fine[0:5], Corse[6:7])
				twl4030_modify(codec, 0x1b, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
				twl4030_modify(codec, 0x1C, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend output
				twl4030_write(codec, 0x23, (0x1 << 2)|0x1); //HS_GAIN(L[0:1], R[2:3])
			case HP4P: 
				//TX Common
				twl4030_write(codec, 0x48, 0x02); // ANAMIC_GAIN (L[0:2], R[3:5])
				twl4030_write(codec, 0x0C, 0xD); //AVTXL2PGA [0:7]
				twl4030_write(codec, 0x0D, 0xD); //AVTXR2PGA [0:7]	

				//RX Common
				twl4030_write(codec, 0x12, (0x1 << 6) | 0x2F); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x1 << 6) | 0x2F); // ARXL2PGA (fine[0:5], Corse[6:7])
				twl4030_modify(codec, 0x1b, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
				twl4030_modify(codec, 0x1C, (0x7 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend output
				twl4030_write(codec, 0x23, (0x1 << 2)|0x1); //HS_GAIN(L[0:1], R[2:3])
				break;
			case BT: 
				printk("voip bt not supported\n");
				break;
			default:
				break;
		}		
	}
	else if (mode == FM_RADIO)
	{
		switch(device)
		{
			case OFF:
				break;
			case SPK: 
				//play
				twl4030_write(codec, 0x12, (0x2 << 6) | 0x3f); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x2 << 6) | 0x3f); // ARXL2PGA (fine[0:5], Corse[6:7])

#ifndef APPLY_AUDIOTEST_APP				
				//tx common
				twl4030_write(codec, 0x48, (0x4 << 3) | 0x4); // ANAMIC_GAIN (L[0:2], R[3:5])				

				//RX common
				twl4030_modify(codec, 0x1b, (0x9 << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
				twl4030_modify(codec, 0x1C, (0x9 << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]

				//RX depend output
				twl4030_modify(codec, 0x25, (0x3 << 4), PREDL_CTL_GAIN_MASK); //PREDL_CTL [4:5]
				twl4030_modify(codec, 0x26, (0x3 << 4), PREDL_CTL_GAIN_MASK); //PREDR_CTL[4:5]
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/FMRadioSpk.ini", 0);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/FMRadioSpk.ini", 0);
#endif
				break;
			case HP3P: 
			case HP4P: 
				//play
				twl4030_write(codec, 0x12, (0x2 << 6) | 0x30); // ARXR2PGA (fine[0:5], Corse[6:7])
				twl4030_write(codec, 0x13, (0x2 << 6) | 0x30); // ARXL2PGA (fine[0:5], Corse[6:7])
				
#ifndef APPLY_AUDIOTEST_APP	
				//tx common
				//twl4030_write(codec, 0x48, (0x4 << 3) | 0x4); // ANAMIC_GAIN (L[0:2], R[3:5])				
				twl4030_write(codec, 0x48, 0x12); // ANAMIC_GAIN (L[0:2], R[3:5])				

				//RX common
//				twl4030_modify(codec, 0x1b, (0xd << 3), ARX_APGA_GAIN_MASK); //ARXL2_APGA_CTL [3:7]
//				twl4030_modify(codec, 0x1C, (0xd << 3), ARX_APGA_GAIN_MASK); //ARXR2_APGA_CTL [3:7]
				twl4030_write(codec, 0x1b, 0x57); //ARXL2_APGA_CTL [3:7]	
				twl4030_write(codec, 0x1C, 0x57); //ARXR2_APGA_CTL [3:7]
				
				//RX depend output
				//twl4030_write(codec, 0x23, (0x1 << 2)|0x1); //HS_GAIN(L[0:1], R[2:3])
				twl4030_write(codec, 0x23, 0x5); //HS_GAIN(L[0:1], R[2:3])
#else
				if(!get_sec_gain_test_mode())
					twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/FMRadioEar.ini", 0);
				else
					twl4030_set_reg_from_file(codec, "/sdcard/external_sd/FMRadioEar.ini", 0);
#endif
				break;
			default:
				break;
		}		
	}
	

}

static char StringToHexFor16Bit( char *ss )
{
	int i = 0 ;
	u8 val = 0;

	//printk( "[audio gain] 0x%x, 0x%x!!\n",ss[0], ss[1]);

	for ( i = 0 ; i < 2 ; i++ )
	{	 
		if( ss[i] >='0' && ss[i] <='9' )
			val = val*16 + ( ss[i] - '0' );
		else if(ss[i] >='a' && ss[i] <='f' ) 
			val = val*16 + ( ss[i] - 'a'+10 );
		else if(ss[i] >='A' && ss[i] <='F' ) 
			val = val*16 + ( ss[i] - 'A'+10 );
		else
			break;
	}

	//printk( "[audio gain] val = 0x%x!!\n",val);
		
	return val;
}

#ifdef APPLY_AUDIOTEST_APP
static int twl4030_set_reg_from_file(struct snd_soc_codec *codec, char* filename, int mode)
{
	struct file *fp = NULL;
	mm_segment_t oldfs;
	int nFileSize = 0, i;
	char* pBuf;
	int nSize = -1, iPos = 0, nIndex = 0;
	char* arTemp;
	bool	bRunning	= true;

	printk("twl4030 gain setting %s\n", filename);
	
	fp = filp_open( filename, O_RDONLY, 0 ) ;
	if ( fp && ( fp!= 0xfffffffe ) && ( fp != 0xfffffff3 ) )
	{
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		nFileSize = fp->f_op->llseek(fp, 0, SEEK_END);
		fp->f_op->llseek(fp, 0, SEEK_SET);
		pBuf = (char*)kmalloc(nFileSize+1, GFP_KERNEL);
		fp->f_op->read(fp, pBuf, nFileSize, &fp->f_pos);
		pBuf[nFileSize] = '\0';
		filp_close(fp, current->files);
		set_fs(oldfs);

//		printk("%s\n", pBuf);

//		printk( "twl4030_OpenINIFile : File Size = %d \n", nFileSize );
	}
	else
	{
		  WARN( "twl4030_OpenINIFile : Do not find %s file !! \n", filename );
		  return -1;
	}

	nSize = nFileSize;

	while( bRunning )
	{
		if( ( nSize - iPos ) <= 0 ) 
		{
			bRunning = false;
			break;
		}
		arTemp = pBuf + iPos;

		if( ((arTemp[0] == 0x30) && ((arTemp[1] == 'x') || (arTemp[1] == 'X')))
			&& ((arTemp[5] == 0x30) && ((arTemp[6] == 'x') || (arTemp[6] == 'X')))
			&& (arTemp[4] == '=') ){
			regFromFile[nIndex].reg = StringToHexFor16Bit(arTemp+2);
			regFromFile[nIndex].val = StringToHexFor16Bit(arTemp+7);

			//printk( "[SKLee] Get data [%d] = 0x%x, 0x%x!!\n",nIndex, regFromFile[nIndex].reg,regFromFile[nIndex].val );
			nIndex++;
		}
		else{
			printk( "[SKLee] Abnormal Case!!\n");
			break;
		}
		//printk( "[SKLee] iPos1 = %d\n", iPos);
		for(i=iPos+9 ; i< nSize ; i++){
			if( *(pBuf+i) == 0x30 ){
				iPos = i;
				break;
			}
		}
		//printk( "[SKLee] iPos2 = %d\n", iPos);
		if(i >= nSize)
			break;
	}

	if(mode >= 0)
	for(i=0 ; i<nIndex ; i++){
//		printk( "[SKLee] Get data [%d] = 0x%x, 0x%x!!\n",i, regFromFile[i].reg,regFromFile[i].val );
		if((twl4030_get_codec_mode() != VOICE_CALL) && (twl4030_get_codec_mode() != VT_CALL)){
			if((mode == 1) && (regFromFile[i].reg == 0x21))
				regFromFile[i].val = (regFromFile[i].val & 0xf0);
			else if((mode == 2) && (regFromFile[i].reg == 0x26))
				regFromFile[i].val = (regFromFile[i].val & 0xf0) | 0x08;
			else if((mode == 2) && (regFromFile[i].reg == 0x25))
				regFromFile[i].val = (regFromFile[i].val & 0xf0) | 0x04;
			else if((mode == 3) && (regFromFile[i].reg == 0x1b))
				regFromFile[i].val = (regFromFile[i].val & 0xf8);
			else if((mode == 3) && (regFromFile[i].reg == 0x1c))
				regFromFile[i].val = (regFromFile[i].val & 0xf8);
		}
		else{
			if((mode == 1) && (regFromFile[i].reg == 0x21))
				regFromFile[i].val = (regFromFile[i].val & 0xf0) | 0x04;
			else if((mode == 2) && (regFromFile[i].reg == 0x26))
				regFromFile[i].val = (regFromFile[i].val & 0xf0) | 0x08;
			else if((mode == 2) && (regFromFile[i].reg == 0x25))
				regFromFile[i].val = (regFromFile[i].val & 0xf0) | 0x04;
			else if((mode == 3) && (regFromFile[i].reg == 0x1b))
				regFromFile[i].val = (regFromFile[i].val & 0xf8) | 0x03;
			else if((mode == 3) && (regFromFile[i].reg == 0x1c))
				regFromFile[i].val = (regFromFile[i].val & 0xf8) | 0x03;
		}
		twl4030_write(codec, regFromFile[i].reg, regFromFile[i].val);
		
	}
	#if defined(APPLY_GAIN_INIT_FROM_INI)
	else
		for(i=0 ; i<nIndex ; i++){
			if(mode == GAIN_INIT_MUSIC_SPK){
				music_spk_gain[i] = regFromFile[i].val;
				printk("music_spk_gain : index %d, value 0x%x\n", i,  music_spk_gain[i]);
			}else if(mode == GAIN_INIT_MUSIC_EAR){
				music_ear_gain[i] = regFromFile[i].val;
				printk("music_ear_gain : index %d, value 0x%x\n", i,  music_ear_gain[i]);
			}
		}
	#endif
	kfree( pBuf );
	
	return nFileSize;
}
#endif

#if defined(APPLY_AUDIOTEST_APP) && defined(APPLY_GAIN_INIT_FROM_INI)
void set_codec_gain_init(struct snd_soc_codec *codec)
{
	P("");	
	twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/MusicSpk.ini", GAIN_INIT_MUSIC_SPK);
	twl4030_set_reg_from_file(codec, "/system/etc/audio/codec/MusicEar.ini", GAIN_INIT_MUSIC_EAR);
}
#endif