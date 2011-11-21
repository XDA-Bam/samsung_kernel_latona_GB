/*
 * max9877.c  --  amp driver for max9877
 *
 * Copyright (C) 2009 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#define SEC_MAX9877_DEBUG 1

#if SEC_MAX9877_DEBUG
#define P(format,...)\
		printk("[audio:%s]" format "\n", __func__, ## __VA_ARGS__);
#else
#define P(format,...)
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <plat/gpio.h>
#include <plat/mux.h>
#include <sound/soc.h>
#include <linux/slab.h>

#include <linux/mfd/twl4030-codec.h>
#include "max9877.h"

#define MAX9877_NAME "max9877"

//#define MAX9877_TUNE	1

#define APPLY_AUDIOTEST_APP //for gain setting from ini file
#define APPLY_GAIN_INIT_FROM_INI //for gain setting from ini file when boot up only
//#define EAR_AMP_SHDN_ALWAYS_ON
#define EAR_AMP_DIVIDED

#define MAX9877_GAIN_MAX	5
static unsigned int music_ear_amp_gain[MAX9877_GAIN_MAX];
static unsigned int music_spk_amp_gain[MAX9877_GAIN_MAX];

static struct i2c_client *i2c;
struct delayed_work amp_control_work;
static struct wake_lock max9877_wakelock;
#ifdef EAR_AMP_DIVIDED
	int ear_amp_set = false;
#endif

//static u8 max9877_regs[5] = { 0x40, 0x00, 0x00, 0x00, 0x49 };
static u8 max9877_regs[5] = { 0x40, 0x1e, 0x1e, 0x1e, 0x81};
static u8 max9877_regs_backup[10];

#define DEFAULT_INPUT 0x40
#define SPK_VOLUME 0x1c
#define SPK_HP_VOLUME 0x1c
#define HPL_VOLUME 0x1a
#define HPR_VOLUME 0x1a
#define SPK_CALL_VOLUME 0x1b
#define HPL_CALL_VOLUME 0x17
#define HPR_CALL_VOLUME 0x17

static u8 old_output_mode = 0x00;
static u8 prev_output_mode = 0x00;
static u8 curr_output_mode = 0x00;

#ifdef EAR_AMP_DIVIDED
void max9877_ear_amp_control(int onoff)
{
	if(onoff) {
		mdelay(350);
		if (gpio_request(OMAP_GPIO_EAR_AMP_SHDN , "EAR_AMP_SHDN") == 0)
			gpio_direction_output(OMAP_GPIO_EAR_AMP_SHDN , 1);
		else
			printk("[MAX9877] fail to gpio request EAR_AMP_SHDN\n");	
		gpio_set_value(OMAP_GPIO_EAR_AMP_SHDN, 1);
		ear_amp_set = true;
		printk("[MAX9877] == EAR AMP ON !!\n");
	} else {
		gpio_set_value(OMAP_GPIO_EAR_AMP_SHDN, 0);
		gpio_free(OMAP_GPIO_EAR_AMP_SHDN);
		ear_amp_set = false;
		printk("[MAX9877] == EAR AMP OFF !!\n");
	}
}
EXPORT_SYMBOL_GPL(max9877_ear_amp_control);
#endif // EAR_AMP_DIVIDED

#ifdef USE_SPK_LINE_OUT_SEL
void max9877_spk_lineout_control(int onoff)
{
	if(onoff) {
		if (gpio_request(SPK_LINE_OUT_SEL , "SPK_LINE_OUT_SEL") == 0)
			gpio_direction_output(SPK_LINE_OUT_SEL , 1);
		else
			printk("[MAX9877] fail to gpio request SPK_LINE_OUT_SEL\n");	
		gpio_set_value(SPK_LINE_OUT_SEL, 1);
		printk("[MAX9877] == SPK ON !!\n");
	} else {
		gpio_set_value(SPK_LINE_OUT_SEL, 0);
		gpio_free(SPK_LINE_OUT_SEL);
		printk("[MAX9877] == SPK OFF !!\n");
	}
}
EXPORT_SYMBOL_GPL(max9877_spk_lineout_control);
#endif // USE_SPK_LINE_OUT_SEL


#ifdef APPLY_AUDIOTEST_APP
typedef struct applyReg
{
	unsigned int reg;
	unsigned int val;
} applyFileReg;

#define MAX_FILE_SIZE	200
#define MAX_REG_NUM 	15

applyFileReg regFromFileAmp[MAX_REG_NUM];

#include <linux/fs.h>
#include <linux/vmalloc.h>

static char StringToHexFor16Bit( char *ss )
{
	int i = 0 ;
	u8 val = 0;

	//printk( "[SKLee] 0x%x, 0x%x!!\n",ss[0], ss[1]);

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

	//printk( "[SKLee] val = 0x%x!!\n",val);
	return val;
}

static int max9877_set_reg_from_file(char* filename, int mode)
{
	struct file *fp = NULL;
	mm_segment_t oldfs;
	int nFileSize = 0, i;
	char* pBuf;
	int nSize = -1, iPos = 0, nIndex = 0;
	char* arTemp;
	bool	bRunning	= true;

	printk("max9877 gain setting %s\n", filename);
	
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
//		printk( "max9877_OpenINIFile(%s) : File Size = %d \n", filename, nFileSize );
	}
	else
	{
		  printk( "max9877_OpenINIFile : Do not find %s file !! \n", filename );
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
			regFromFileAmp[nIndex].reg = StringToHexFor16Bit(arTemp+2);
			regFromFileAmp[nIndex].val = StringToHexFor16Bit(arTemp+7);

			//printk( "[SKLee] Get data [%d] = 0x%x, 0x%x!!\n",nIndex, regFromFileAmp[nIndex].reg,regFromFileAmp[nIndex].val );
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

	if(mode >= 0) {
		for(i=0 ; i<nIndex ; i++) {
	//		printk( "[SKLee] Get data [%d] = 0x%x, 0x%x!!\n",i, regFromFileAmp[i].reg,regFromFileAmp[i].val );
			max9877_regs[regFromFileAmp[i].reg] = regFromFileAmp[i].val;
	//		twl4030_write(codec, regFromFileAmp[i].reg, regFromFileAmp[i].val);
		}
	}
#if defined(APPLY_GAIN_INIT_FROM_INI)
	else {
		for(i=0 ; i<nIndex ; i++) {
			//printk( "[SKLee] Get data [%d] = 0x%x, 0x%x!!\n",i, regFromFileAmp[i].reg,regFromFileAmp[i].val );
			if(mode == GAIN_INIT_MUSIC_SPK)
			{
				music_spk_amp_gain[i] = regFromFileAmp[i].val;
				printk("music_spk_amp_gain %d, 0x%x\n",i, music_spk_amp_gain[i]);
			}else if(mode == GAIN_INIT_MUSIC_EAR){
				music_ear_amp_gain[i]= regFromFileAmp[i].val;
				printk("music_ear_amp_gain %d, 0x%x\n",i, music_ear_amp_gain[i]);
			}
			
		//	twl4030_write(codec, regFromFileAmp[i].reg, regFromFileAmp[i].val);
		}
	}
#endif
	kfree( pBuf );
	
	return nFileSize;
}
#endif

#if defined(APPLY_AUDIOTEST_APP) && defined(APPLY_GAIN_INIT_FROM_INI)
void set_amp_gain_init()
{
	P("");	
	max9877_set_reg_from_file("/system/etc/audio/codec/MusicSpkAmp.ini", GAIN_INIT_MUSIC_SPK);
	max9877_set_reg_from_file("/system/etc/audio/codec/MusicEarAmp.ini", GAIN_INIT_MUSIC_EAR);
}
#endif

#ifdef APPLY_AUDIOTEST_APP
void max9877_apply_case(int spkORear, int musicORvoice)
{
	int i = 0;
	if(spkORear == INA_SPK || spkORear == INA_SPK_HP) {
		if(musicORvoice == VOICE_CALL) {
			if(!get_sec_gain_test_mode())		
				max9877_set_reg_from_file("/system/etc/audio/codec/VoiceCallSpkAmp.ini", 0);
			else
				max9877_set_reg_from_file("/sdcard/external_sd/VoiceCallSpkAmp.ini", 0);
		} else if(musicORvoice == VT_CALL) {
			if(!get_sec_gain_test_mode())		
				max9877_set_reg_from_file("/system/etc/audio/codec/VtCallSpkAmp.ini", 0);
			else
				max9877_set_reg_from_file("/sdcard/external_sd/VtCallSpkAmp.ini", 0);
		} else if(musicORvoice == VOIP_CALL) {
			if(!get_sec_gain_test_mode())		
				max9877_set_reg_from_file("/system/etc/audio/codec/VoipCallSpkAmp.ini", 0);
			else
				max9877_set_reg_from_file("/sdcard/external_sd/VoipCallSpkAmp.ini", 0);
		} else {
			if(!get_sec_gain_test_mode()) {
#if defined(APPLY_GAIN_INIT_FROM_INI)
				for(i = 0 ; i < MAX9877_GAIN_MAX ; i++)
					max9877_regs[i] = music_spk_amp_gain[i];
#else
				max9877_set_reg_from_file("/system/etc/audio/codec/MusicSpkAmp.ini", 0);
#endif
			} else
				max9877_set_reg_from_file("/sdcard/external_sd/MusicSpkAmp.ini", 0);		
		}
	} else if(spkORear == INB_HP || spkORear == INA_HP) {
		if(musicORvoice == VOICE_CALL) {
			if(!get_sec_gain_test_mode())
				max9877_set_reg_from_file("/system/etc/audio/codec/VoiceCallEarAmp.ini", 0);
			else
				max9877_set_reg_from_file("/sdcard/external_sd/VoiceCallEarAmp.ini", 0);
		} else if(musicORvoice == VT_CALL) {
			if(!get_sec_gain_test_mode())
				max9877_set_reg_from_file("/system/etc/audio/codec/VtCallEarAmp.ini", 0);
			else
				max9877_set_reg_from_file("/sdcard/external_sd/VtCallEarAmp.ini", 0);
		} else if(musicORvoice == VOIP_CALL) {
			if(!get_sec_gain_test_mode())		
				max9877_set_reg_from_file("/system/etc/audio/codec/VoipCallEarAmp.ini", 0);
			else
				max9877_set_reg_from_file("/sdcard/external_sd/VoipCallEarAmp.ini", 0);
		} else {
			if(!get_sec_gain_test_mode()){
#if defined(APPLY_GAIN_INIT_FROM_INI)
				for(i=0 ; i<MAX9877_GAIN_MAX ; i++)
					max9877_regs[i] = music_ear_amp_gain[i];
#else
				max9877_set_reg_from_file("/system/etc/audio/codec/MusicEarAmp.ini", 0);
#endif
			} else
				max9877_set_reg_from_file("/sdcard/external_sd/MusicEarAmp.ini", 0);	
		}
	}
	else
		printk( "[MAX9877] Abnormal Case!!\n");
}
#endif

static void max9877_write_regs(void)
{
	//printk("****************** max9877_write_regs() in**\n");
	unsigned int i;
	u8 data[6];

	data[0] = MAX9877_INPUT_MODE;
	
	for (i = 0; i < ARRAY_SIZE(max9877_regs); i++)
	{
		 data[i + 1] = max9877_regs[i];
		//printk("*** Max9877 Register [ 0x%x : 0x%x] \n", i+1, max9877_regs[i]);
	}
	
	if (i2c_master_send(i2c, data, 6) != 6)
		dev_err(&i2c->dev, "i2c write failed\n");

}

static int max9877_get_reg(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		   (struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int mask = mc->max;
	unsigned int invert = mc->invert;

	ucontrol->value.integer.value[0] = (max9877_regs[reg] >> shift) & mask;

	if (invert)
		ucontrol->value.integer.value[0] =
				mask - ucontrol->value.integer.value[0];

	return 0;
}

static int max9877_set_reg(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
			 (struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int mask = mc->max;
	unsigned int invert = mc->invert;
	unsigned int val = (ucontrol->value.integer.value[0] & mask);

	P("");

	if (invert)
		val = mask - val;

	if (((max9877_regs[reg] >> shift) & mask) == val)
		return 0;

	max9877_regs[reg] &= ~(mask << shift);
	max9877_regs[reg] |= val << shift;
	max9877_write_regs();

	return 1;
}

static int max9877_get_2reg(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	unsigned int mask = mc->max;

	ucontrol->value.integer.value[0] = (max9877_regs[reg] >> shift) & mask;
	ucontrol->value.integer.value[1] = (max9877_regs[reg2] >> shift) & mask;

	return 0;
}

static int max9877_set_2reg(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	unsigned int mask = mc->max;
	unsigned int val = (ucontrol->value.integer.value[0] & mask);
	unsigned int val2 = (ucontrol->value.integer.value[1] & mask);
	unsigned int change = 1;

	P("");

	if (((max9877_regs[reg] >> shift) & mask) == val)
		change = 0;

	if (((max9877_regs[reg2] >> shift) & mask) == val2)
		change = 0;

	if (change) {
		max9877_regs[reg] &= ~(mask << shift);
		max9877_regs[reg] |= val << shift;
		max9877_regs[reg2] &= ~(mask << shift);
		max9877_regs[reg2] |= val2 << shift;
		max9877_write_regs();
	}

	return change;
}

void max9877_power_down_mode(void)
{
	max9877_regs[0] = 0x0;
	max9877_regs[1] = 0x0;
	max9877_regs[2] = 0x0;
	max9877_regs[3] = 0x0;
	max9877_regs[4] = 0x40;
	prev_output_mode = max9877_regs[4];
	max9877_write_regs();
}
EXPORT_SYMBOL_GPL(max9877_power_down_mode);

static int max9877_get_out_mode(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	u8 value = max9877_regs[MAX9877_OUTPUT_MODE] & MAX9877_OUTMODE_MASK;

	if (value)
	value -= 1;

	//P(" value : ox%x", value);

	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int max9877_set_output_volume(void)
{
	if(twl4030_get_codec_mode() == VOICE_CALL)
	{
		max9877_regs[MAX9877_SPK_VOLUME] = SPK_CALL_VOLUME;
		max9877_regs[MAX9877_HPL_VOLUME] = HPL_CALL_VOLUME;
		max9877_regs[MAX9877_HPR_VOLUME] = HPR_CALL_VOLUME;
	}else{
		max9877_regs[MAX9877_SPK_VOLUME] = SPK_VOLUME;
		max9877_regs[MAX9877_HPL_VOLUME] = HPL_VOLUME;
		max9877_regs[MAX9877_HPR_VOLUME] = HPR_VOLUME;
	}

	return 0;
}

static void amp_control_work_handler(struct work_struct *work )
{
#if 0
	if(curr_output_mode==3 || curr_output_mode==0xf)
	{
		max9877_regs[MAX9877_SPK_VOLUME] = SPK_HP_VOLUME;
		max9877_regs[MAX9877_HPL_VOLUME] = 0x0f;
		max9877_regs[MAX9877_HPR_VOLUME] = 0x0f;
	}
	else
	{
		max9877_set_output_volume();
	}
#endif
	max9877_write_regs();
	wake_unlock( &max9877_wakelock);
}

void max9877_set_force_out_mode(int mode, int output)
{
	printk("max9877_set_force_out_mode %d, output %d\n", mode, output);
	
	switch(mode)
	{
		case VOIP_CALL:
			if(output == SPK)
			{
				max9877_apply_case(INA_SPK, VOIP_CALL);
				max9877_write_regs();
			}
			else if(output == HP4P || output == HP3P)
			{
				if(!ear_amp_set)
					max9877_ear_amp_control(true);
			}
			else
			{
				if(ear_amp_set)
					max9877_ear_amp_control(false);
			}
			break;
		case LOOP_BACK:
			if(output == SPK)
			{
				max9877_apply_case(INA_SPK, PLAY_BACK);
				max9877_write_regs();
			}
			else if(output == HP4P || output == HP3P)
			{
				if(!ear_amp_set)
					max9877_ear_amp_control(true);
			}
			else
			{
				if(ear_amp_set)
					max9877_ear_amp_control(false);
			}
			break;
		default:
			if(output == SPK)
			{
				max9877_apply_case(INA_SPK, mode);
				max9877_write_regs();
			}
			else if(output == HP4P || output == HP3P)
			{
				if(!ear_amp_set)
					max9877_ear_amp_control(true);
			}
			else
			{
				if(ear_amp_set)
					max9877_ear_amp_control(false);
			}
			break;
	}
}
EXPORT_SYMBOL_GPL(max9877_set_force_out_mode);

static int max9877_set_out_mode(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{	
	//TI patch start
	static u32 testID = 0;
	//TI Patch end
    curr_output_mode = ucontrol->value.integer.value[0];
	
	P(" mode : %d", curr_output_mode);
	//TI patch start : To prevent an initial noise case --> should be applied for this project only till find a real solution 
	if(testID < 2 && curr_output_mode == 0)
	{
		testID++;
		printk("[max9877] max9877_set_out_mode++ return once \n");

		return;
	}
	//TI Patch end
	
#ifdef APPLY_AUDIOTEST_APP
	max9877_apply_case(curr_output_mode, twl4030_get_codec_mode());
#endif	
	
#ifdef CONFIG_SND_SOC_MAX9879
	
	switch(curr_output_mode)
	{ 
		case INA_SPK:
			curr_output_mode = MAX9879_ENA | MAX9879_LSPKEN | MAX9879_RSPKEN;
#ifdef EAR_AMP_DIVIDED			
			if(ear_amp_set)
				max9877_ear_amp_control(false);
#endif				
			break;
		case INA_HP:
			curr_output_mode = MAX9879_ENA |MAX9879_HPEN;
#ifdef EAR_AMP_DIVIDED
			if(!ear_amp_set)
				max9877_ear_amp_control(true);
			return 1;
#endif			
			break;
		case INA_SPK_HP:
			curr_output_mode = MAX9879_ENA | MAX9879_LSPKEN | MAX9879_RSPKEN | MAX9879_HPEN;
#ifdef EAR_AMP_DIVIDED		
			if(!ear_amp_set)
				max9877_ear_amp_control(true);
#endif			
			break;
		case INB_SPK:
			curr_output_mode = MAX9879_ENB | MAX9879_LSPKEN | MAX9879_RSPKEN;
#ifdef EAR_AMP_DIVIDED			
			if(ear_amp_set)
				max9877_ear_amp_control(false);
#endif				
			break;
		case INB_HP:
			curr_output_mode = MAX9879_ENB |MAX9879_HPEN;
#ifdef EAR_AMP_DIVIDED
			if(!ear_amp_set)
				max9877_ear_amp_control(true);
			return 1;
#endif	
			break;
		case INB_SPK_HP:
			curr_output_mode = MAX9879_ENB | MAX9879_LSPKEN | MAX9879_RSPKEN | MAX9879_HPEN;
#ifdef EAR_AMP_DIVIDED
			if(!ear_amp_set)
				max9877_ear_amp_control(true);
#endif	
			break;
		case INA_INB_SPK:
			curr_output_mode = MAX9879_ENA | MAX9879_ENB |MAX9879_LSPKEN | MAX9879_RSPKEN;
#ifdef EAR_AMP_DIVIDED			
			if(ear_amp_set)
				max9877_ear_amp_control(false);
#endif				
			break;
		case INA_INB_HP:
			curr_output_mode = MAX9879_ENA | MAX9879_ENB |MAX9879_HPEN;
#ifdef EAR_AMP_DIVIDED
			if(!ear_amp_set)
				max9877_ear_amp_control(true);
			return 1;
#endif	
			break;
		case INA_INB_SPK_HP:
			curr_output_mode = MAX9879_ENA | MAX9879_ENB |MAX9879_LSPKEN | MAX9879_RSPKEN |MAX9879_HPEN ;
#ifdef EAR_AMP_DIVIDED
			if(!ear_amp_set)
				max9877_ear_amp_control(true);
#endif	
			break;
		case INA_L_SPK:
			curr_output_mode = MAX9879_ENA | MAX9879_LSPKEN;
#ifdef EAR_AMP_DIVIDED			
			if(ear_amp_set)
				max9877_ear_amp_control(false);
#endif				
			break;
		case INA_R_SPK:
			curr_output_mode = MAX9879_ENA | MAX9879_RSPKEN;
#ifdef EAR_AMP_DIVIDED			
			if(ear_amp_set)
				max9877_ear_amp_control(false);
#endif				
			break;
		case INB_L_SPK:
			curr_output_mode = MAX9879_ENB | MAX9879_LSPKEN;
#ifdef EAR_AMP_DIVIDED			
			if(ear_amp_set)
				max9877_ear_amp_control(false);
#endif				
			break;
		case INB_R_SPK:
			curr_output_mode = MAX9879_ENB | MAX9879_RSPKEN;
#ifdef EAR_AMP_DIVIDED			
			if(ear_amp_set)
				max9877_ear_amp_control(false);
#endif				
			break;
		case INA_INB_L_SPK:
		case INA_INB_R_SPK:
		default:
#ifdef EAR_AMP_DIVIDED			
			if(ear_amp_set)
				max9877_ear_amp_control(false);
#endif			
			printk("MAX9877.c : MAX9879 Not Supprot Mode %d\n", curr_output_mode);
			return 1;
			break;

	}
#else		
    curr_output_mode += 1;
#endif		

#if 0
	max9877_regs[MAX9877_OUTPUT_MODE] &= ~MAX9877_BYPASS;
	max9877_regs[MAX9877_OUTPUT_MODE] &= ~MAX9877_OUTMODE_MASK;
	max9877_regs[MAX9877_OUTPUT_MODE] |= curr_output_mode;
	max9877_regs[MAX9877_OUTPUT_MODE] |= MAX9877_SHDN;
#endif
	if(prev_output_mode==max9877_regs[MAX9877_OUTPUT_MODE])
	{		
		printk("MAX9877.c : same output mode!!! mode = %d",prev_output_mode );
		return 1;
	}

	old_output_mode = max9877_regs[MAX9877_OUTPUT_MODE] ;
	prev_output_mode = max9877_regs[MAX9877_OUTPUT_MODE] ;

	wake_lock( &max9877_wakelock);
	schedule_delayed_work( &amp_control_work, 3 );

	return 1;
}

static int max9877_get_osc_mode(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	u8 value = (max9877_regs[MAX9877_OUTPUT_MODE] & MAX9877_OSC_MASK);

	value = value >> MAX9877_OSC_OFFSET;

	P(" value : ox%x", value);

	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int max9877_set_osc_mode(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	u8 value = ucontrol->value.integer.value[0];

	value = value << MAX9877_OSC_OFFSET;

	P("");

	if ((max9877_regs[MAX9877_OUTPUT_MODE] & MAX9877_OSC_MASK) == value)
			return 0;

	P("value : 0x%x", value);

	max9877_regs[MAX9877_OUTPUT_MODE] &= ~MAX9877_OSC_MASK;
	max9877_regs[MAX9877_OUTPUT_MODE] |= value;
	max9877_write_regs();

	return 1;
}

static int max9877_set_power_control(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	
#ifndef MAX9877_TUNE
	if(ucontrol->value.integer.value[0])  //off case
	{
#ifdef EAR_AMP_DIVIDED				
		if(ear_amp_set)
			max9877_ear_amp_control(false);
#endif		
		 max9877_regs[0] = 0x0;
		 max9877_regs[1] = 0x0;
		 max9877_regs[2] = 0x0;
		 max9877_regs[3] = 0x0;
		 max9877_regs[4] = 0x40;
		 prev_output_mode = max9877_regs[4];
		 printk("[MAX9877] max9877_power_control mode = OFF\n");
	}
	else
	{
		 max9877_regs[0] = DEFAULT_INPUT;
		 max9877_regs[1] = SPK_VOLUME;
		 max9877_regs[2] = HPL_VOLUME;
		 max9877_regs[3] = HPR_VOLUME;
		 max9877_regs[4] = old_output_mode;
		 printk("[MAX9877] max9877_power_control mode = ON\n");
	}
	
	max9877_write_regs();
#endif

	return 0;
}

static int max9877_get_power_control(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	int power_on = 0;
	
	if(max9877_regs[MAX9877_OUTPUT_MODE] &= MAX9877_SHDN)
	{
		power_on = 1;
	}
	else 
		power_on = 0;
	
//	printk("[max9877] get power %d\n",power_on);
	ucontrol->value.integer.value[0] = power_on;

	return 0;
}
static const unsigned int max9877_pgain_tlv[] = {
        TLV_DB_RANGE_HEAD(2),
        0, 1, TLV_DB_SCALE_ITEM(0, 900, 0),
        2, 2, TLV_DB_SCALE_ITEM(2000, 0, 0),
};

static const unsigned int max9877_output_tlv[] = {
        TLV_DB_RANGE_HEAD(4),
        0, 7, TLV_DB_SCALE_ITEM(-7900, 400, 1),
        8, 15, TLV_DB_SCALE_ITEM(-4700, 300, 0),
        16, 23, TLV_DB_SCALE_ITEM(-2300, 200, 0),
        24, 31, TLV_DB_SCALE_ITEM(-700, 100, 0),
};

static const char *max9877_out_mode[] = {
       "INA -> SPK",
       "INA -> HP",
       "INA -> SPK and HP",
       "INB -> SPK",
       "INB -> HP",
       "INB -> SPK and HP",
       "INA + INB -> SPK",
       "INA + INB -> HP",
       "INA + INB -> SPK and HP",
#ifdef CONFIG_SND_SOC_MAX9879
	   "INA -> L_SPK",
	   "INA -> R_SPK",
	   "INB -> L_SPK",
	   "INB -> R_SPK",
	   "INA + INB -> L_SPK",
 	   "INA + INB -> R_SPK",		
#endif
};

static const char *max9877_osc_mode[] = {
   "1176KHz",
   "1100KHz",
   "700KHz",
};

static const char *max9877_power_onoff[]={
   "ON",
   "OFF"
};

static const struct soc_enum max9877_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max9877_out_mode), max9877_out_mode),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max9877_osc_mode), max9877_osc_mode),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max9877_power_onoff), max9877_power_onoff),
};

static const struct snd_kcontrol_new max9877_controls[] = {
	SOC_SINGLE_EXT_TLV("MAX9877 PGAINB Playback Volume",
					MAX9877_INPUT_MODE, 0, 2, 0,
					max9877_get_reg, max9877_set_reg, max9877_pgain_tlv),
	SOC_SINGLE_EXT_TLV("MAX9877 PGAINA Playback Volume",
					MAX9877_INPUT_MODE, 2, 2, 0,
					max9877_get_reg, max9877_set_reg, max9877_pgain_tlv),
	SOC_SINGLE_EXT_TLV("MAX9877 Amp Speaker Playback Volume",
					MAX9877_SPK_VOLUME, 0, 31, 0,
					max9877_get_reg, max9877_set_reg, max9877_output_tlv),
	SOC_DOUBLE_R_EXT_TLV("MAX9877 Amp HP Playback Volume",
					MAX9877_HPL_VOLUME, MAX9877_HPR_VOLUME, 0, 31, 0,
					max9877_get_2reg, max9877_set_2reg, max9877_output_tlv),
	SOC_SINGLE_EXT("MAX9877 INB Stereo Switch",
					MAX9877_INPUT_MODE, 4, 1, 1,
					max9877_get_reg, max9877_set_reg),
	SOC_SINGLE_EXT("MAX9877 INA Stereo Switch",
					MAX9877_INPUT_MODE, 5, 1, 1,
					max9877_get_reg, max9877_set_reg),
	SOC_SINGLE_EXT("MAX9877 Zero-crossing detection Switch",
					MAX9877_INPUT_MODE, 6, 1, 0,
					max9877_get_reg, max9877_set_reg),
	SOC_SINGLE_EXT("MAX9877 Bypass Mode Switch",
					MAX9877_OUTPUT_MODE, 6, 1, 0,
					max9877_get_reg, max9877_set_reg),
	SOC_SINGLE_EXT("MAX9877 Shutdown Mode Switch",
					MAX9877_OUTPUT_MODE, 7, 1, 1,
					max9877_get_reg, max9877_set_reg),
	SOC_ENUM_EXT("MAX9877 Output Mode", max9877_enum[0],
					max9877_get_out_mode, max9877_set_out_mode),
	SOC_ENUM_EXT("MAX9877 Oscillator Mode", max9877_enum[1],
					max9877_get_osc_mode, max9877_set_osc_mode),
	SOC_ENUM_EXT("Amp Enable", max9877_enum[2],
		   max9877_get_power_control, max9877_set_power_control),                        
};

#if 1
/* This function is called from ASoC machine driver */
int max9877_add_controls(struct snd_soc_codec *codec)
{
	return snd_soc_add_controls(codec, max9877_controls,
			ARRAY_SIZE(max9877_controls));
}
EXPORT_SYMBOL_GPL(max9877_add_controls);
#else 
  /* This function is called from ASoC machine driver */
int max9877_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(max9877_controls); i++) {
		err = snd_ctl_add(codec->card, snd_soc_cnew(&max9877_controls[i], codec, NULL));
		if (err < 0)
				return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(max9877_add_controls);
#endif

#if ( defined( CONFIG_MACH_SAMSUNG_P1LITE ) && ( CONFIG_SAMSUNG_REL_HW_REV <= 2 ) )	
extern u32 hw_revision;
#elif ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) && ( CONFIG_SAMSUNG_REL_HW_REV <= 2 ) )	
extern u32 hw_revision;
#endif

static int __devinit max9877_i2c_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
	i2c = client;
	
	printk("max9877_init\n");
	if (gpio_request(OMAP_GPIO_AMP_SHDN , "AMP_SHDN") == 0) 
		gpio_direction_output(OMAP_GPIO_AMP_SHDN , 1);
	else
		printk("[MAX9877] fail to gpio request AMP_SHDN\n");
#if ( defined( CONFIG_MACH_SAMSUNG_P1LITE ) && ( CONFIG_SAMSUNG_REL_HW_REV > 2 ) )			
	gpio_set_value(OMAP_GPIO_AMP_SHDN, 1);
#elif ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) && ( CONFIG_SAMSUNG_REL_HW_REV > 2 ) )			
	gpio_set_value(OMAP_GPIO_AMP_SHDN, 1);
#else
	if(hw_revision == 7) {
		printk("[MAX9877] 0.25 HW detect amp ON setting\n");
		i2c->addr = 0x4D;
		gpio_set_value(OMAP_GPIO_AMP_SHDN, 1);
	} else
		gpio_set_value(OMAP_GPIO_AMP_SHDN, 0);
#endif

#ifdef USE_SPK_LINE_OUT_SEL
	max9877_spk_lineout_control(true);
#endif // USE_SPK_LINE_OUT_SEL

#ifdef EAR_AMP_SHDN_ALWAYS_ON
	if (gpio_request(OMAP_GPIO_EAR_AMP_SHDN , "EAR_AMP_SHDN") == 0)
		gpio_direction_output(OMAP_GPIO_EAR_AMP_SHDN , 1);	
	else
		printk("[twl4030] fail to gpio request EAR_AMP_SHDN\n");	
	gpio_set_value(OMAP_GPIO_EAR_AMP_SHDN, 1);
#endif // EAR_AMP_SHDN_ALWAYS_ON
	wake_lock_init( &max9877_wakelock, WAKE_LOCK_SUSPEND, "max9877_audio");
	//max9877_write_regs();

	return 0;
}

static __devexit int max9877_i2c_remove(struct i2c_client *client)
{
	i2c = NULL;

	return 0;
}

static int max9877_suspend(struct i2c_client *client, pm_message_t mesg)
{
	P("");

#if 0
	max9877_regs[0] = 0x0;
	max9877_regs[1] = 0x0;
	max9877_regs[2] = 0x0;
	max9877_regs[3] = 0x0;
	max9877_regs[4] = 0x40;

	max9877_write_regs();
#endif

	return 0;
}

static void max9877_shutdown(struct i2c_client *client)
{

	printk("shutdown !!\n");

	max9877_regs[0] = 0x0;
	max9877_regs[1] = 0x0;
	max9877_regs[2] = 0x0;
	max9877_regs[3] = 0x0;
	max9877_regs[4] = 0x40;

	max9877_write_regs();

}

static int max9877_resume(struct i2c_client *client)
{
	#if 0
	max9877_regs[0] = 0x40;
	max9877_regs[1] = 0x1e;
	max9877_regs[2] = 0x1e;
	max9877_regs[3] = 0x1e;
	max9877_regs[4] = 0x81;

	max9877_write_regs();
	#endif
	return 0;
}

static const struct i2c_device_id max9877_i2c_id[] = {
	{ "max9877", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9877_i2c_id);

static struct i2c_driver max9877_i2c_driver = {
	.driver = {
		.name = "max9877",
		.owner = THIS_MODULE,
	},
	.probe = max9877_i2c_probe,
	.remove = __devexit_p(max9877_i2c_remove),
	.shutdown = max9877_shutdown,
	.suspend = max9877_suspend,
	.resume = max9877_resume,	
	.id_table = max9877_i2c_id,
};

static int __init max9877_init(void)
{
	INIT_DELAYED_WORK( &amp_control_work, amp_control_work_handler ); //sec_lilkan	
		return i2c_add_driver(&max9877_i2c_driver);
}
module_init(max9877_init);

static void __exit max9877_exit(void)
{
	i2c_del_driver(&max9877_i2c_driver);
}
module_exit(max9877_exit);

MODULE_DESCRIPTION("ASoC MAX9877 amp driver");
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_LICENSE("GPL");

