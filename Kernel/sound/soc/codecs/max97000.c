/*
 * max97000.c  --  amp driver for max97000
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <plat/gpio.h>
#include <plat/mux.h>

/* Register descriptions are here */
#include <linux/mfd/twl4030-codec.h>
#include "max97000.h"

#define MAX97000_NAME "max97000"
//#define SEC_MAX97000_DEBUG 1

#if SEC_MAX97000_DEBUG
#define P(format,...)\
		printk("[audio:%s]" format "\n", __func__, ## __VA_ARGS__);
#else
#define P(format,...)
#endif

//#define APPLY_AUDIOTEST_APP //for gain setting from ini file
//#define APPLY_GAIN_INIT_FROM_INI //for gain setting from ini file when boot up only

#define MAX97000_GAIN_MAX	7
static unsigned int music_ear_amp_gain[MAX97000_GAIN_MAX];
static unsigned int music_spk_amp_gain[MAX97000_GAIN_MAX];


#if defined(MAX97000_USE_GPIO_I2C)
	#include <plat/i2c-omap-gpio.h>
	static OMAP_GPIO_I2C_CLIENT * max97000_i2c_client;
	static struct i2c_client *max97000_dummy_i2c_client;
#else
	static struct i2c_client *max97000_i2c_client;
#endif

struct delayed_work amp_control_work;
static struct wake_lock max97000_wakelock;

static u8 max97000_regs[10] = { 0x36, 0xCC, 0x03, 0x1f, 0x5f, 0x3f, 0x00, 0x00, 0x96, 0x00};
static u8 max97000_regs_backup[10];

#define DEFAULT_INPUT 0x36

#define HPL_VOLUME 0x10
#define HPR_VOLUME 0x10
#define SPK_VOLUME 0x20
#define INA_GAIN 0x24
#define INB_GAIN 0x02

static u8 old_output_mode = 0x00;
//static u8 prev_output_mode = 0x00;
static u8 curr_output_mode = OUTPUT_OFF;


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


static int max97000_set_reg_from_file(char* filename, int mode)
{
	struct file *fp = NULL;
	mm_segment_t oldfs;
	int nFileSize = 0, i;
	char* pBuf;
	int nSize = -1, iPos = 0, nIndex = 0;
	char* arTemp;
	bool	bRunning	= true;

	printk("max97000 gain setting %s\n", filename);

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

//		printk( "max97000_OpenINIFile(%s) : File Size = %d \n", filename, nFileSize );
	}
	else
	{
		  printk( "max97000_OpenINIFile : Do not find %s file !! \n", filename );
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

	if(mode >= 0)
	for(i=0 ; i<nIndex ; i++){
//		printk( "[SKLee] Get data [%d] = 0x%x, 0x%x!!\n",i, regFromFileAmp[i].reg,regFromFileAmp[i].val );
		max97000_regs[regFromFileAmp[i].reg] = regFromFileAmp[i].val;
//		twl4030_write(codec, regFromFileAmp[i].reg, regFromFileAmp[i].val);
	}
	#if defined(APPLY_GAIN_INIT_FROM_INI)
	else
		for(i=0 ; i<nIndex ; i++){
		//printk( "[SKLee] Get data [%d] = 0x%x, 0x%x!!\n",i, regFromFileAmp[i].reg,regFromFileAmp[i].val );
			if(mode == GAIN_INIT_MUSIC_SPK)
			{
				music_spk_amp_gain[i] = regFromFileAmp[i].val;
				printk("music_spk_amp_gain %d, 0x%x\n",i, music_spk_amp_gain[i]);
			}else if(mode == GAIN_INIT_MUSIC_EAR){
				music_ear_amp_gain[i]= regFromFileAmp[i].val;
				printk("music_ear_amp_gain %d, 0x%x\n",i, music_ear_amp_gain[i]);
			}
		//		twl4030_write(codec, regFromFileAmp[i].reg, regFromFileAmp[i].val);
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
	max97000_set_reg_from_file("/system/etc/audio/codec/MusicSpkAmp.ini", GAIN_INIT_MUSIC_SPK);
	max97000_set_reg_from_file("/system/etc/audio/codec/MusicEarAmp.ini", GAIN_INIT_MUSIC_EAR);
}
#endif
#ifdef APPLY_AUDIOTEST_APP
void max97000_apply_case(int spkORear, int musicORvoice)
{
	int i = 0;
	P("max97000_apply_case %d, %d\n",spkORear,musicORvoice);
	if(spkORear == INA_SPK || spkORear == INA_SPK_HP){
		if(musicORvoice == VOICE_CALL){
			if(!get_sec_gain_test_mode())
				max97000_set_reg_from_file("/system/etc/audio/codec/VoiceCallSpkAmp.ini", 0);
			else
				max97000_set_reg_from_file("/sdcard/external_sd/VoiceCallSpkAmp.ini", 0);
		}
		else if(musicORvoice == VT_CALL){
			if(!get_sec_gain_test_mode())
				max97000_set_reg_from_file("/system/etc/audio/codec/VoiceCallSpkAmp.ini", 0);
			else
				max97000_set_reg_from_file("/sdcard/external_sd/VtCallSpkAmp.ini", 0);
		}
		else
		{
			if(!get_sec_gain_test_mode()){
				#if defined(APPLY_GAIN_INIT_FROM_INI)
				for(i=0 ; i<MAX97000_GAIN_MAX ; i++){
					if(i == 6)
						max97000_regs[8] = music_spk_amp_gain[i];
					else
						max97000_regs[i] = music_spk_amp_gain[i];
				}
				#else
				max97000_set_reg_from_file("/system/etc/audio/codec/MusicSpkAmp.ini", 0);
				#endif
			}
			else
				max97000_set_reg_from_file("/sdcard/external_sd/MusicSpkAmp.ini", 0);

		}
	}
	else if(spkORear == INB_HP){
		if(musicORvoice == VOICE_CALL){
			if(!get_sec_gain_test_mode())
				max97000_set_reg_from_file("/system/etc/audio/codec/VoiceCallEarAmp.ini", 0);
			else
				max97000_set_reg_from_file("/sdcard/external_sd/VoiceCallEarAmp.ini", 0);
		}
		else if(musicORvoice == VT_CALL){
			if(!get_sec_gain_test_mode())
				max97000_set_reg_from_file("/system/etc/audio/codec/VoiceCallEarAmp.ini", 0);
			else
				max97000_set_reg_from_file("/sdcard/external_sd/VtCallEarAmp.ini", 0);
		}
		else if(musicORvoice == FM_RADIO){
				max97000_regs[0] = 0x02;
				max97000_regs[1] = 0x84;
				max97000_regs[2] = 0x00;
				max97000_regs[3] = 0x98;
				max97000_regs[4] = 0x18;
				max97000_regs[5] = 0x1e;
				max97000_regs[8] = 0x86;
				printk("[max97000]force fm radio gain setting\n");
		}
		else
		{
			if(!get_sec_gain_test_mode()){
				#if defined(APPLY_GAIN_INIT_FROM_INI)
				for(i=0 ; i<MAX97000_GAIN_MAX ; i++){
					if(i == 6)
						max97000_regs[8] = music_ear_amp_gain[i];
					else
						max97000_regs[i] = music_ear_amp_gain[i];
				}
				#else
				max97000_set_reg_from_file("/system/etc/audio/codec/MusicEarAmp.ini", 0);
				#endif
			}
			else
				max97000_set_reg_from_file("/sdcard/external_sd/MusicEarAmp.ini", 0);
		}
	}
//	else
//		printk( "[SKLee : 97000] Abnormal Case!!\n");
}
#endif

#if defined(MAX97000_USE_GPIO_I2C)
static int i2c_read( unsigned char reg_addr )
{
	int ret = 0;
	unsigned char buf[2];
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;

	i2c_rd_param.reg_len = 1;
	i2c_rd_param.reg_addr = &reg_addr;
	i2c_rd_param.rdata_len = 2;
	i2c_rd_param.rdata = buf;
	omap_gpio_i2c_read(max97000_i2c_client, &i2c_rd_param);

	ret = buf[0] << 8 | buf[1];

	return ret;
}

static int i2c_write( unsigned char *buf, u8 len )
{
	int ret = 0;
	OMAP_GPIO_I2C_WR_DATA i2c_wr_param;

	i2c_wr_param.reg_len = 0;
	i2c_wr_param.reg_addr = NULL;
	i2c_wr_param.wdata_len = len;
	i2c_wr_param.wdata = buf;
	omap_gpio_i2c_write(max97000_i2c_client, &i2c_wr_param);

	return ret;
}
#endif

void max97000_write_regs(void)
{
	unsigned int i;
	u8 data[11];

	data[0] = MAX97000_INPUT_GAIN;
	for (i = 0; i < ARRAY_SIZE(max97000_regs); i++)
	{
		data[i + 1] = max97000_regs[i];
		#ifdef SEC_MAX97000_DEBUG
		if(max97000_regs_backup[i] != max97000_regs[i])
		{
			printk("Max97000 Register [ 0x%x : 0x%x] \n", i, max97000_regs[i]);
			max97000_regs_backup[i] = max97000_regs[i];
		}
		#endif
   	}

#if defined(MAX97000_USE_GPIO_I2C)
        if (i2c_write(data, 11) != 0)
#else
        if (i2c_master_send(max97000_i2c_client, data, 11) != 11)
#endif
        {
			//dev_err(&max97000_i2c_client-dev, "max97000_i2c_client write failed\n");
			printk("max97000 max97000_i2c_client error !!!!\n");
        }
}

void max97000_write_single(const unsigned int reg, const unsigned int value)
{
	u8 data[3];

	data[0] = reg;
	data[1] = value;

#if defined(MAX97000_USE_GPIO_I2C)
        if (i2c_write(data, 2) != 0)
#else
        if (i2c_master_send(max97000_i2c_client, data, 2) != 2)
#endif
	{
		//dev_err(&max97000_i2c_client->dev, "max97000_i2c_client write failed\n");
		printk("max97000 max97000_i2c_client error !!!!\n");
	}

	printk("max97000 write 0x%x, 0x%x", reg, value);
}
static int max97000_get_reg(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
         struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
         unsigned int reg = mc->reg;
         unsigned int shift = mc->shift;
         unsigned int mask = mc->max;
         unsigned int invert = mc->invert;

        ucontrol->value.integer.value[0] = (max97000_regs[reg] >> shift) & mask;

        if (invert)
                ucontrol->value.integer.value[0] =
                        mask - ucontrol->value.integer.value[0];

        return 0;
}

static int max97000_set_reg(struct snd_kcontrol *kcontrol,
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

        if (((max97000_regs[reg] >> shift) & mask) == val)
                return 0;

        max97000_regs[reg] &= ~(mask << shift);
        max97000_regs[reg] |= val << shift;
        max97000_write_regs();
        return 1;
}

static int max97000_get_2reg(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        unsigned int reg = mc->reg;
        unsigned int reg2 = mc->rreg;
        unsigned int shift = mc->shift;
        unsigned int mask = mc->max;

        ucontrol->value.integer.value[0] = (max97000_regs[reg] >> shift) & mask;
        ucontrol->value.integer.value[1] = (max97000_regs[reg2] >> shift) & mask;

        return 0;
}

static int max97000_set_2reg(struct snd_kcontrol *kcontrol,
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

        if (((max97000_regs[reg] >> shift) & mask) == val)
                change = 0;

        if (((max97000_regs[reg2] >> shift) & mask) == val2)
                change = 0;

        if (change) {
                max97000_regs[reg] &= ~(mask << shift);
                max97000_regs[reg] |= val << shift;
                max97000_regs[reg2] &= ~(mask << shift);
                max97000_regs[reg2] |= val2 << shift;
                max97000_write_regs();
        }

        return change;
}

static void max97000_power_down_work_handler(struct work_struct *work)
{
	//printk("max97000_power_down_work_handler \n");
	curr_output_mode = OUTPUT_OFF;
	max97000_regs[2] = 0x0;
	max97000_regs[5] = 0x0;
	max97000_regs[6] = 0x0;
	max97000_regs[7] = 0x0;
	max97000_regs[8] = 0x0;
	//prev_output_mode = 0x01;
	max97000_write_regs();
	wake_unlock( &max97000_wakelock);
}
static DECLARE_DELAYED_WORK(max97000_power_down_work_queue, max97000_power_down_work_handler);

void max97000_power_down_mode(void)
{
		curr_output_mode = OUTPUT_OFF;
//		cancel_delayed_work_sync(&max97000_power_down_work_queue);
		max97000_regs[9] = 0x1;
		max97000_regs[8] = max97000_regs[8] & ~(MAX97000_SPKEN |MAX97000_HPLEN |MAX97000_HPREN);
		max97000_write_regs();
		mdelay(20);
//		mdelay(50);
		max97000_regs[0] = 0x0;
		max97000_regs[1] = 0x0;
		max97000_regs[3] = 0x0;
		max97000_regs[4] = 0x0;
		max97000_regs[2] = 0x0;
		max97000_regs[5] = 0x0;
		max97000_regs[6] = 0x0;
		max97000_regs[7] = 0x0;
		max97000_regs[8] = 0x0;
		//prev_output_mode = 0x01;
		max97000_write_regs();
		mdelay(20);
//		mdelay(10);
//		wake_lock( &max97000_wakelock);
//		schedule_delayed_work(&max97000_power_down_work_queue, 200);
}

EXPORT_SYMBOL_GPL(max97000_power_down_mode);

static int max97000_set_power_control(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	printk("[max97000] max97000_power_control value = %d \n", (int)ucontrol->value.integer.value[0]);

	curr_output_mode = OUTPUT_OFF;
	if(ucontrol->value.integer.value[0])  //off case
	{
		max97000_power_down_mode();
	}
	else
	{
		max97000_regs[0] = DEFAULT_INPUT;
		max97000_regs[3] = HPL_VOLUME;
		max97000_regs[4] = HPR_VOLUME;
		max97000_regs[5] = SPK_VOLUME;
		max97000_regs[8] = old_output_mode;
		max97000_write_regs();
	}
	return 0;
}

static int max97000_get_out_mode(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{

  u8 value = max97000_regs[MAX97000_POWER] & MAX97000_SHDN;
  if (value)
    value -= 1;

  //P(" value : ox%x", value);

  ucontrol->value.integer.value[0] = value;
  return 0;
}

static int max97000_set_output_volume(void)
{
	#if 0
	if(twl4030_get_codec_mode() == VOICE_CALL)
	{
		max97000_regs[MAX97000_SPK_GAIN] = SPK_CALL_VOLUME;
		max97000_regs[MAX97000_HPL_GAIN] = HPL_CALL_VOLUME;
		max97000_regs[MAX97000_HPR_GAIN] = HPR_CALL_VOLUME;
	}else{
	#endif
		max97000_regs[MAX97000_SPK_GAIN] = SPK_VOLUME;
		max97000_regs[MAX97000_HPL_GAIN] = HPL_VOLUME;
		max97000_regs[MAX97000_HPR_GAIN] = HPR_VOLUME;
	//}

	return 0;
}

static void amp_control_work_handler(struct work_struct *work )
{
	    printk("amp_control_work_handler() in\n");

	    if(curr_output_mode==3)
	    {
	    	    max97000_regs[MAX97000_SPK_GAIN] = SPK_VOLUME;
	  	    max97000_regs[MAX97000_HPL_GAIN] = 0x11;
	 	    max97000_regs[MAX97000_HPR_GAIN] = 0x11;
	    }
	    else
	    {
    		max97000_set_output_volume();
	    }

	    max97000_write_regs();
	    wake_unlock( &max97000_wakelock);
}
static int max97000_set_out_mode(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	u8 value = ucontrol->value.integer.value[0];
	u8 inputMode = 0x00;
	u8 hpMixer = 0x00;
	u8 spkMixer = 0x00;
	u8 powerManage = 0x00;

	if(curr_output_mode == ucontrol->value.integer.value[0]){
		printk(" max97000_set_out_mode() output is same!!!! %ld\n", ucontrol->value.integer.value[0]);
		return 0;
	}

	curr_output_mode = ucontrol->value.integer.value[0];

	printk(" max97000_set_out_mode() in %ld**\n", ucontrol->value.integer.value[0]);

	wake_unlock( &max97000_wakelock);
//	cancel_delayed_work_sync(&max97000_power_down_work_queue);

	P(" mode : %d", value);
	switch(value)
	{
		case INA_SPK:
			inputMode = INA_GAIN;
			spkMixer = MAX97000_SPK_INA1 | MAX97000_SPK_INA2;
			hpMixer = 0x00;
			powerManage = MAX97000_SPKEN; /*| MAX97000_SWEN;*/
		break;
		case INA_HP:
			inputMode = INA_GAIN;
			spkMixer = 0x00;
			hpMixer = MAX97000_HPR_INA1 | MAX97000_HPL_INA1 |MAX97000_HPR_INA2 | MAX97000_HPL_INA2;
			powerManage = MAX97000_HPLEN | MAX97000_HPREN | MAX97000_SWEN;
		break;
		case INA_SPK_HP:
			inputMode = INA_GAIN;
			spkMixer = MAX97000_SPK_INA1 | MAX97000_SPK_INA2;
			hpMixer = MAX97000_HPR_INA1 | MAX97000_HPL_INA1 |MAX97000_HPR_INA2 | MAX97000_HPL_INA2;
			powerManage = MAX97000_SPKEN | MAX97000_HPLEN | MAX97000_HPREN;
		break;
		case INB_SPK:
			inputMode = INB_GAIN;
			spkMixer = MAX97000_SPK_INB1 | MAX97000_SPK_INB2;
			hpMixer = 0x00;
			powerManage = MAX97000_SPKEN;
		break;
		case INB_HP:
			inputMode = INB_GAIN;
			spkMixer = 0x00;
			hpMixer = MAX97000_HPR_INB1 | MAX97000_HPL_INB1 | MAX97000_HPR_INB2 | MAX97000_HPL_INB2;
			powerManage = MAX97000_HPLEN | MAX97000_HPREN; /*| MAX97000_LPMODE_INB | MAX97000_SWEN;*/
		break;
		case INB_SPK_HP:
			inputMode = INB_GAIN;
			spkMixer = MAX97000_SPK_INB1 | MAX97000_SPK_INB2;
			hpMixer = MAX97000_HPR_INB1 | MAX97000_HPL_INB1 | MAX97000_HPR_INB2 | MAX97000_HPL_INB2;
			powerManage = MAX97000_SPKEN | MAX97000_HPLEN | MAX97000_HPREN;
		break;
		case INA_INB_SPK:
			inputMode = INA_GAIN | INB_GAIN;
			spkMixer = MAX97000_SPK_INA1 | MAX97000_SPK_INA2 | MAX97000_SPK_INB1 | MAX97000_SPK_INB2;
			hpMixer = 0x00;
			powerManage = MAX97000_SPKEN;
		break;
		case INA_INB_HP:
			inputMode = INA_GAIN | INB_GAIN;
			spkMixer = 0x00;
			hpMixer = MAX97000_HPR_INA1 | MAX97000_HPL_INA1 |MAX97000_HPR_INA2 | MAX97000_HPL_INA2
				| MAX97000_HPR_INB1 | MAX97000_HPL_INB1 | MAX97000_HPR_INB2 | MAX97000_HPL_INB2;
			powerManage = MAX97000_LPMODE_INA | MAX97000_LPMODE_INB | MAX97000_HPLEN | MAX97000_HPREN;
		break;
		case INA_INB_SPK_HP:
			inputMode = INA_GAIN | INB_GAIN;
			spkMixer = MAX97000_SPK_INA1 | MAX97000_SPK_INA2 | MAX97000_SPK_INB1 | MAX97000_SPK_INB2;
			hpMixer = MAX97000_HPR_INA1 | MAX97000_HPL_INA1 |MAX97000_HPR_INA2 | MAX97000_HPL_INA2
				| MAX97000_HPR_INB1 | MAX97000_HPL_INB1 | MAX97000_HPR_INB2 | MAX97000_HPL_INB2;
			powerManage = MAX97000_SPKEN | MAX97000_HPLEN | MAX97000_HPREN;
		break;

		default:
			printk("MAX97000.c : Not Supprot Mode %d", value);
		break;

	}


	 P(" value : ox%x", value);

	max97000_set_output_volume();

	max97000_regs[MAX97000_INPUT_GAIN] = inputMode;
	max97000_regs[MAX97000_SPK_MIXER] = spkMixer;
	max97000_regs[MAX97000_HP_MIXER] = hpMixer;
	max97000_regs[MAX97000_POWER] = powerManage;
	max97000_regs[MAX97000_POWER] |= MAX97000_SHDN;
	max97000_regs[9] = 0x0;

#ifdef APPLY_AUDIOTEST_APP
	max97000_apply_case(value, twl4030_get_codec_mode());
#endif


	if(value == INA_SPK_HP)
	{
		max97000_regs[MAX97000_SPK_MIXER] = spkMixer;
		max97000_regs[MAX97000_HP_MIXER] = hpMixer;
		max97000_regs[MAX97000_POWER] = powerManage;
		max97000_regs[MAX97000_POWER] |= MAX97000_SHDN;
		max97000_regs[9] = 0x0;
	}

	mdelay(30);	
	max97000_write_regs();

	old_output_mode = max97000_regs[MAX97000_POWER] ;
	//prev_output_mode = max97000_regs[MAX97000_POWER] ;

#if 0
	if(value == INA_SPK_HP)
	{
		max97000_set_output_volume();
		max97000_write_regs();
	}
#endif

	return 1;
}
void max97000_set_force_out_mode(int mode, int output)
{
	printk("max97000_set_force_out_mode %d, output %d\n", mode, output);
	switch(mode)
	{
		case FM_RADIO:
			if(output  == SPK)
			{
				#ifdef APPLY_AUDIOTEST_APP
				max97000_apply_case(INA_SPK, FM_RADIO);
				#endif
				max97000_write_regs();
			}
			else if(output == HP4P || output == HP3P)
			{
				#ifdef APPLY_AUDIOTEST_APP
				max97000_apply_case(INB_HP, FM_RADIO);		
				#endif
				max97000_write_regs();
			}
			else
				printk("doesn`t supoort other device\n");
		break;
		default:
			printk("mdoe %d, doesn`t supoort \n", mode);
	}
}
EXPORT_SYMBOL_GPL(max97000_set_force_out_mode);

static int max97000_get_power_control(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	int power_on = 0;
	if(max97000_regs[MAX97000_POWER] &= MAX97000_SHDN)
	{
		power_on = 1;
	}
	else
		power_on = 0;

	P("[max97000] get power %d",power_on);
	ucontrol->value.integer.value[0] = power_on;

	return 0;
}
static const unsigned int max97000_pgain_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 1, TLV_DB_SCALE_ITEM(0, 900, 0),
	2, 2, TLV_DB_SCALE_ITEM(2000, 0, 0),
};

static const unsigned int max97000_output_tlv[] = {
	TLV_DB_RANGE_HEAD(4),
	0, 7, TLV_DB_SCALE_ITEM(-7900, 400, 1),
	8, 15, TLV_DB_SCALE_ITEM(-4700, 300, 0),
	16, 23, TLV_DB_SCALE_ITEM(-2300, 200, 0),
	24, 31, TLV_DB_SCALE_ITEM(-700, 100, 0),
};

static const char *max97000_out_mode[] = {
	"INA -> SPK",
	"INA -> HP",
	"INA -> SPK and HP",
	"INB -> SPK",
	"INB -> HP",
	"INB -> SPK and HP",
	"INA + INB -> SPK",
	"INA + INB -> HP",
	"INA + INB -> SPK and HP",
};

static const char *max97000_osc_mode[] = {
	"1176KHz",
	"1100KHz",
	"700KHz",
};

static const char *max97000_power_onoff[]={
	"ON",
	"OFF"
};

static const struct soc_enum max97000_enum[] = {
        SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max97000_out_mode), max97000_out_mode),
        SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max97000_osc_mode), max97000_osc_mode),
        SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(max97000_power_onoff), max97000_power_onoff),
};

static const struct snd_kcontrol_new max97000_controls[] = {
        SOC_SINGLE_EXT_TLV("MAX97000 PGAINB Playback Volume",
                        MAX97000_INPUT_GAIN, 0, 2, 0,
                        max97000_get_reg, max97000_set_reg, max97000_pgain_tlv),
        SOC_SINGLE_EXT_TLV("MAX97000 PGAINA Playback Volume",
                        MAX97000_INPUT_GAIN, 2, 2, 0,
                        max97000_get_reg, max97000_set_reg, max97000_pgain_tlv),
        SOC_SINGLE_EXT_TLV("MAX97000 Amp Speaker Playback Volume",
                        MAX97000_SPK_GAIN, 0, 31, 0,
                        max97000_get_reg, max97000_set_reg, max97000_output_tlv),
        SOC_DOUBLE_R_EXT_TLV("MAX97000 Amp HP Playback Volume",
                        MAX97000_HPL_GAIN, MAX97000_HPR_GAIN, 0, 31, 0,
                        max97000_get_2reg, max97000_set_2reg, max97000_output_tlv),
        SOC_SINGLE_EXT("MAX97000 INB Stereo Switch",
                        MAX97000_INPUT_GAIN, 4, 1, 1,
                        max97000_get_reg, max97000_set_reg),
        SOC_SINGLE_EXT("MAX97000 INA Stereo Switch",
                        MAX97000_INPUT_GAIN, 5, 1, 1,
                        max97000_get_reg, max97000_set_reg),
        SOC_SINGLE_EXT("MAX97000 Zero-crossing detection Switch",
                        MAX97000_INPUT_GAIN, 6, 1, 0,
                        max97000_get_reg, max97000_set_reg),
        SOC_SINGLE_EXT("MAX97000 Bypass Mode Switch",
                        MAX97000_POWER, 6, 1, 0,
                        max97000_get_reg, max97000_set_reg),
        SOC_SINGLE_EXT("MAX97000 Shutdown Mode Switch",
                        MAX97000_POWER, 7, 1, 1,
                        max97000_get_reg, max97000_set_reg),
        SOC_ENUM_EXT("MAX97000 Output Mode", max97000_enum[0],
                        max97000_get_out_mode, max97000_set_out_mode),
  //      SOC_ENUM_EXT("MAX97000 Oscillator Mode", max97000_enum[1],
   //                     max97000_get_osc_mode, max97000_set_osc_mode),
		SOC_ENUM_EXT("Amp Enable", max97000_enum[2],
	  		   max97000_get_power_control, max97000_set_power_control),
};

#if 0
  /* This function is called from ASoC machine driver */
int max97000_add_controls(struct snd_soc_codec *codec)
{
        int err, i;

        for (i = 0; i < ARRAY_SIZE(max97000_controls); i++) {
                err = snd_ctl_add(codec->card,
                                snd_soc_cnew(&max97000_controls[i],
                                        codec, NULL));
                if (err < 0)
                        return err;
        }

        return 0;
}
EXPORT_SYMBOL_GPL(max97000_add_controls);
#else
/* This function is called from ASoC machine driver */
int max97000_add_controls(struct snd_soc_codec *codec)
{
	return snd_soc_add_controls(codec, max97000_controls,
			ARRAY_SIZE(max97000_controls));
}
EXPORT_SYMBOL_GPL(max97000_add_controls);
#endif

static int __devinit max97000_i2c_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
	P("");

#if defined(MAX97000_USE_GPIO_I2C)
	max97000_dummy_i2c_client = client;
#else
	max97000_i2c_client = client;
#endif

#if 0 //changoh.heo 2011.02.08 OMAP_GPIO_AMP_SHDN was requested in mux.c. Therefore delete it.
	if (gpio_request(OMAP_GPIO_AMP_SHDN , "OMAP_GPIO_AMP_SHDN") == 0)
	{
		gpio_direction_output(OMAP_GPIO_AMP_SHDN , 1);
	}else
		printk("[MAX97000] fail to gpio request OMAP_GPIO_AMP_SHDN\n");
#endif

	gpio_set_value(OMAP_GPIO_AMP_SHDN, 1);
	wake_lock_init( &max97000_wakelock, WAKE_LOCK_SUSPEND, "max97000_audio");

	return 0;
}

static __devexit int max97000_i2c_remove(struct i2c_client *client)
{
        max97000_i2c_client = NULL;

        return 0;
}

static int max97000_suspend(struct i2c_client *client, pm_message_t mesg)
{
	P("");
	curr_output_mode = OUTPUT_OFF;
	return 0;
}

static void max97000_shutdown(struct i2c_client *client)
{
	printk("shutdown !!");
	curr_output_mode = OUTPUT_OFF;
	max97000_regs[0] = 0x0;
	max97000_regs[1] = 0x0;
	max97000_regs[2] = 0x0;
	max97000_regs[3] = 0x0;
	max97000_regs[4] = 0x40;
	max97000_write_regs();
}

static int max97000_resume(struct i2c_client *client)
{
	curr_output_mode = OUTPUT_OFF;
	return 0;
}

static const struct i2c_device_id max97000_i2c_id[] = {
        { "max97000", 0 },
        { }
};
MODULE_DEVICE_TABLE(max97000_i2c_client, max97000_i2c_id);

static struct i2c_driver max97000_i2c_driver = {
        .driver = {
                .name = "max97000",
                .owner = THIS_MODULE,
        },
	.probe = max97000_i2c_probe,
	.remove = __devexit_p(max97000_i2c_remove),
	.shutdown = max97000_shutdown,
	.suspend = max97000_suspend,
	.resume = max97000_resume,
	.id_table = max97000_i2c_id,
};

static int __init max97000_init(void)
{
	int ret = 0;
	printk("max97000_init\n");
	INIT_DELAYED_WORK( &amp_control_work, amp_control_work_handler ); //sec_lilkan
#if defined(MAX97000_USE_GPIO_I2C)
	max97000_i2c_client = omap_gpio_i2c_init(OMAP_GPIO_AP_I2C_SDA,
						  OMAP_GPIO_AP_I2C_SCL,
						  //0x36,
						  //0x9A,
						  0x4D,
						  200);
	if(max97000_i2c_client == NULL)
	{
		printk(KERN_ERR "[FG] omap_gpio_i2c_init failed!\n");
	}
#endif
	ret = i2c_add_driver(&max97000_i2c_driver);
	if(ret)
		printk( KERN_ERR "[MAX97000] i2c_add_driver failed");

	curr_output_mode = OUTPUT_OFF;
	return ret;
}
module_init(max97000_init);

static void __exit max97000_exit(void)
{
#if defined(MAX97000_USE_GPIO_I2C)
	omap_gpio_i2c_deinit(max97000_i2c_client);
#else
	i2c_del_driver(&max97000_i2c_driver);
#endif

}
module_exit(max97000_exit);

MODULE_DESCRIPTION("ASoC MAX97000 amp driver");
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_LICENSE("GPL");

