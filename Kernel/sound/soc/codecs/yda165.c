/*
 * yda165.c  --  amp driver for yda165
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
#include <linux/fs.h>
#include <linux/vmalloc.h>

/* Register descriptions are here */
#include <linux/mfd/twl4030-codec.h>
#include "yda165.h"

#define YDA165_NAME "yda165"
#define SEC_YDA165_DEBUG 1

#if SEC_YDA165_DEBUG
#define P(format,...)\
		printk("[audio:%s]" format "\n", __func__, ## __VA_ARGS__);
#else
#define P(format,...)
#endif

#define YDA165_GAIN_MAX	7
static unsigned int music_ear_amp_gain[YDA165_GAIN_MAX];
static unsigned int music_spk_amp_gain[YDA165_GAIN_MAX];


#if defined(YDA165_USE_GPIO_I2C)
	#include <plat/i2c-omap-gpio.h>
	static OMAP_GPIO_I2C_CLIENT * yda165_i2c_client;
	static struct i2c_client *yda165_dummy_i2c_client;
#else
	static struct i2c_client *yda165_i2c_client;
#endif

static struct wake_lock yda165_wakelock;

static u8 yda165_regs[9] = { 0x80, 0x08, 0x0d, 0x80, 0x22, 0x40, 0x40, 0x00, 0x00};
static u8 yda165_regs_backup[9];

#define HP_VOLUME 0x10
#define SP_VOLUME 0x20
#define INA_GAIN 0x24
#define INB_GAIN 0x02

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
static char StringToHexFor16Bit( char *ss )
{
	int i = 0 ;
	u8 val = 0;

	//printk( "[SKLee] 0x%x, 0x%x!!\n",ss[0], ss[1]);
	for ( i = 0 ; i < 2 ; i++ ){
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


static int yda165_set_reg_from_file(char* filename, int mode)
{
	struct file *fp = NULL;
	mm_segment_t oldfs;
	int nFileSize = 0, i;
	char* pBuf;
	int nSize = -1, iPos = 0, nIndex = 0;
	char* arTemp;
	bool	bRunning	= true;

	printk("yda165 gain setting %s\n", filename);

	fp = filp_open( filename, O_RDONLY, 0 ) ;
	if ( fp && ( fp!= 0xfffffffe ) && ( fp != 0xfffffff3 ) ){
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
//		printk( "yda165_OpenINIFile(%s) : File Size = %d \n", filename, nFileSize );
	}else{
		printk( "yda165_OpenINIFile : Do not find %s file !! \n", filename );
		return -1;
	}

	nSize = nFileSize;

	while( bRunning ){
		if( ( nSize - iPos ) <= 0 ){
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
		}else{
			printk( "[SKLee] Abnormal Case!!\n");
			break;
		}

		for(i=iPos+9 ; i< nSize ; i++){
			if( *(pBuf+i) == 0x30 ){
				iPos = i;
				break;
			}
		}
		if(i >= nSize)
			break;
	}

	if(mode >= 0)
        	for(i=0 ; i<nIndex ; i++){
        //		printk( "[SKLee] Get data [%d] = 0x%x, 0x%x!!\n",i, regFromFileAmp[i].reg,regFromFileAmp[i].val );
                        if(regFromFileAmp[i].reg == 0x8)
                		yda165_regs[7] = regFromFileAmp[i].val;                                
                        else
                		yda165_regs[regFromFileAmp[i].reg+1] = regFromFileAmp[i].val;
        //		twl4030_write(codec, regFromFileAmp[i].reg, regFromFileAmp[i].val);
        	}
	#if defined(APPLY_GAIN_INIT_FROM_INI)
	else
		for(i=0 ; i<nIndex ; i++)
		        //printk( "[SKLee] Get data [%d] = 0x%x, 0x%x!!\n",i, regFromFileAmp[i].reg,regFromFileAmp[i].val );
			if(mode == GAIN_INIT_MUSIC_SPK){
				music_spk_amp_gain[i] = regFromFileAmp[i].val;
				printk("music_spk_amp_gain %d, 0x%x\n",i, music_spk_amp_gain[i]);
			}else if(mode == GAIN_INIT_MUSIC_EAR){
				music_ear_amp_gain[i]= regFromFileAmp[i].val;
				printk("music_ear_amp_gain %d, 0x%x\n",i, music_ear_amp_gain[i]);
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
	yda165_set_reg_from_file("/system/etc/audio/codec/MusicSpkAmp.ini", GAIN_INIT_MUSIC_SPK);
	yda165_set_reg_from_file("/system/etc/audio/codec/MusicEarAmp.ini", GAIN_INIT_MUSIC_EAR);
}
#endif
#ifdef APPLY_AUDIOTEST_APP
void yda165_apply_case(int spkORear, int musicORvoice)
{
	int i = 0;
	P("yda165_apply_case %d, %d\n",spkORear,musicORvoice);
	if(spkORear == INA_SPK || spkORear == INA_SPK_HP){
		if(musicORvoice == VOICE_CALL){
			if(!get_sec_gain_test_mode())
				yda165_set_reg_from_file("/system/etc/audio/codec/VoiceCallSpkAmp.ini", 0);
			else
				yda165_set_reg_from_file("/sdcard/external_sd/VoiceCallSpkAmp.ini", 0);
		}
		else if(musicORvoice == VT_CALL){
			if(!get_sec_gain_test_mode())
				yda165_set_reg_from_file("/system/etc/audio/codec/VoiceCallSpkAmp.ini", 0);
			else
				yda165_set_reg_from_file("/sdcard/external_sd/VtCallSpkAmp.ini", 0);
		}
		else
		{
			if(!get_sec_gain_test_mode()){
				#if defined(APPLY_GAIN_INIT_FROM_INI)
				for(i=0 ; i<YDA165_GAIN_MAX ; i++){
					if(i == 6)
						yda165_regs[8] = music_spk_amp_gain[i];
					else
						yda165_regs[i] = music_spk_amp_gain[i];
				}
				#else
				yda165_set_reg_from_file("/system/etc/audio/codec/MusicSpkAmp.ini", 0);
				#endif
			}
			else
				yda165_set_reg_from_file("/sdcard/external_sd/MusicSpkAmp.ini", 0);

		}
	}
	else if(spkORear == INB_HP){
		if(musicORvoice == VOICE_CALL){
			if(!get_sec_gain_test_mode())
				yda165_set_reg_from_file("/system/etc/audio/codec/VoiceCallEarAmp.ini", 0);
			else
				yda165_set_reg_from_file("/sdcard/external_sd/VoiceCallEarAmp.ini", 0);
		}
		else if(musicORvoice == VT_CALL){
			if(!get_sec_gain_test_mode())
				yda165_set_reg_from_file("/system/etc/audio/codec/VoiceCallEarAmp.ini", 0);
			else
				yda165_set_reg_from_file("/sdcard/external_sd/VtCallEarAmp.ini", 0);
		}
		else
		{
			if(!get_sec_gain_test_mode()){
				#if defined(APPLY_GAIN_INIT_FROM_INI)
				for(i=0 ; i<YDA165_GAIN_MAX ; i++){
					if(i == 6)
						yda165_regs[8] = music_ear_amp_gain[i];
					else
						yda165_regs[i] = music_ear_amp_gain[i];
				}
				#else
				yda165_set_reg_from_file("/system/etc/audio/codec/MusicEarAmp.ini", 0);
				#endif
			}
			else
				yda165_set_reg_from_file("/sdcard/external_sd/MusicEarAmp.ini", 0);
		}
	}
//	else
//		printk( "[SKLee : 97000] Abnormal Case!!\n");
}
#endif

#if defined(YDA165_USE_GPIO_I2C)
static int i2c_read( unsigned char reg_addr )
{
	int ret = 0;
	unsigned char buf[2];
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;

	i2c_rd_param.reg_len = 1;
	i2c_rd_param.reg_addr = &reg_addr;
	i2c_rd_param.rdata_len = 2;
	i2c_rd_param.rdata = buf;
	omap_gpio_i2c_read(yda165_i2c_client, &i2c_rd_param);

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
	omap_gpio_i2c_write(yda165_i2c_client, &i2c_wr_param);

	return ret;
}
#endif

void yda165_write_regs(void)
{
	unsigned int i;
	u8 data[11];

	data[0] = YDA165_RESET_BADR;
	for (i = 0; i < ARRAY_SIZE(yda165_regs); i++){
		data[i + 1] = yda165_regs[i];
		#ifdef SEC_YDA165_DEBUG
		if(yda165_regs_backup[i] != yda165_regs[i]){
			printk("Yda165 Register [ 0x%x : 0x%x] \n", i, yda165_regs[i]);
			yda165_regs_backup[i] = yda165_regs[i];
		}
		#endif
   	}

#if defined(YDA165_USE_GPIO_I2C)
        if (i2c_write(data, 11) != 0)
#else
        if (i2c_master_send(yda165_i2c_client, data, 11) != 11)
#endif
        {
			//dev_err(&yda165_i2c_client-dev, "yda165_i2c_client write failed\n");
			printk("yda165 yda165_i2c_client error !!!!\n");
        }
}

void yda165_write_single(const unsigned int reg, const unsigned int value)
{
	u8 data[3];

	data[0] = reg;
	data[1] = value;

#if defined(YDA165_USE_GPIO_I2C)
        if (i2c_write(data, 2) != 0)
#else
        if (i2c_master_send(yda165_i2c_client, data, 2) != 2)
#endif
	{
		//dev_err(&yda165_i2c_client->dev, "yda165_i2c_client write failed\n");
		printk("yda165 yda165_i2c_client error !!!!\n");
	}

	printk("yda165 write 0x%x, 0x%x", reg, value);
}
void yda165_power_down_mode(void)
{
        /* 1. HP & SP Mute */
        yda165_regs[YDA165_SPATT] &= 0xe0;
        yda165_regs[YDA165_HPATT] &= 0xe0;
        yda165_write_regs();
        /* 2. HP & SP power down */
        yda165_regs[YDA165_HS_GAIN] &= ~YDA165_HIZ_SP;
        yda165_regs[YDA165_HS_GAIN] |= YDA165_HIZ_HP;
        yda165_write_regs();
        /* 3. reset all resisters */        
        yda165_regs[YDA165_RESET] = 0x80;
        yda165_write_regs();
        curr_output_mode = OUTPUT_OFF;
}

EXPORT_SYMBOL_GPL(yda165_power_down_mode);

static int yda165_set_power_control(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	printk("[yda165] yda165_power_control value = %d \n", (int)ucontrol->value.integer.value[0]);

	curr_output_mode = OUTPUT_OFF;
	if(ucontrol->value.integer.value[0]){
		yda165_power_down_mode();
	} else
                printk(KERN_ERR "[YDA165] Not support AMP On Mode\n");
        
	return 0;
}

static int yda165_get_out_mode(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
        u8 value = curr_output_mode;
        ucontrol->value.integer.value[0] = value;
        
        return 0;
}

static int yda165_set_output_volume(void)
{
        yda165_regs[YDA165_SP_GAIN] |= SP_VOLUME;
	yda165_regs[YDA165_HS_GAIN] |= HP_VOLUME;
	yda165_regs[YDA165_INPUT_GAIN] = INA_GAIN | INB_GAIN;
        yda165_regs[YDA165_HPATT] |= 0x1f;
	yda165_regs[YDA165_SPATT] |= 0x1f;

	return 0;
}

static int yda165_set_out_mode(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	u8 value = ucontrol->value.integer.value[0];
	u8 mixer = 0x00;
	u8 output_port = 0x08;

	if(curr_output_mode == ucontrol->value.integer.value[0]){
		printk(" yda165_set_out_mode() output is same!!!! %ld\n", ucontrol->value.integer.value[0]);
		return 0;
	}
        P(" mode : %d", value);

	curr_output_mode = ucontrol->value.integer.value[0];
	printk(" yda165_set_out_mode() in %ld**\n", ucontrol->value.integer.value[0]);
	wake_unlock( &yda165_wakelock);

	switch(value)
	{
		case INA_SPK:
			mixer |= YDA165_SP_BMIX;
			output_port |= YDA165_HIZ_SP;
		break;
		case INA_SPK_HP:
			mixer = YDA165_SP_BMIX | YDA165_HP_BMIX;
			output_port |= YDA165_HIZ_SP;
                        output_port &= ~(YDA165_HIZ_HP);
		break;
        	case INB_HP:
			mixer = YDA165_HP_AMIX;
                        output_port &= ~(YDA165_HIZ_HP);
		break;
		case INA_INB_SPK_HP:
			mixer = YDA165_SP_BMIX | YDA165_HP_AMIX;
			output_port |= YDA165_HIZ_SP;
                        output_port &= ~(YDA165_HIZ_HP);
		break;

		default:
			printk(KERN_ERR "[YDA165]Not Supprot Mode %d", value);
		break;

	}
	yda165_set_output_volume();

	yda165_regs[YDA165_INPUT_GAIN] = mixer;
	yda165_regs[YDA165_HS_GAIN] |= output_port;
	yda165_regs[YDA165_RESET] |= ~YDA165_SRST;

#ifdef APPLY_AUDIOTEST_APP
	yda165_apply_case(value, twl4030_get_codec_mode());
#endif
	mdelay(30);	
	yda165_write_regs();        
	return 1;
}
void yda165_set_force_out_mode(int mode, int output)
{
	printk("yda165_set_force_out_mode %d, output %d\n", mode, output);
	switch(mode)
	{
		case FM_RADIO:
			if(output  == SPK)
			{
				#ifdef APPLY_AUDIOTEST_APP
				yda165_apply_case(INA_SPK, FM_RADIO);
				#endif
				yda165_write_regs();
			}
			else if(output == HP4P || output == HP3P)
			{
				#ifdef APPLY_AUDIOTEST_APP
				yda165_apply_case(INB_HP, FM_RADIO);		
				#endif
				yda165_write_regs();
			}
			else
				printk("doesn`t supoort other device\n");
		break;
		default:
			printk("mdoe %d, doesn`t supoort \n", mode);
	}
}
EXPORT_SYMBOL_GPL(yda165_set_force_out_mode);

static int yda165_get_power_control(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	int power_on = 0;
	if(yda165_regs[YDA165_RESET] &= YDA165_SRST){ /* power off*/
		power_on = 0;
	}
	else /* power on*/
		power_on = 0;

	P("[yda165] get power %d",power_on);
	ucontrol->value.integer.value[0] = power_on;

	return 0;
}
static const unsigned int yda165_pgain_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 1, TLV_DB_SCALE_ITEM(0, 900, 0),
	2, 2, TLV_DB_SCALE_ITEM(2000, 0, 0),
};

static const unsigned int yda165_output_tlv[] = {
	TLV_DB_RANGE_HEAD(4),
	0, 7, TLV_DB_SCALE_ITEM(-7900, 400, 1),
	8, 15, TLV_DB_SCALE_ITEM(-4700, 300, 0),
	16, 23, TLV_DB_SCALE_ITEM(-2300, 200, 0),
	24, 31, TLV_DB_SCALE_ITEM(-700, 100, 0),
};

static const char *yda165_out_mode[] = {
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

static const char *yda165_osc_mode[] = {
	"1176KHz",
	"1100KHz",
	"700KHz",
};

static const char *yda165_power_onoff[]={
	"ON",
	"OFF"
};

static const struct soc_enum yda165_enum[] = {
        SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(yda165_out_mode), yda165_out_mode),
        SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(yda165_osc_mode), yda165_osc_mode),
        SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(yda165_power_onoff), yda165_power_onoff),
};

static const struct snd_kcontrol_new yda165_controls[] = {
        SOC_ENUM_EXT("MAX97000 Output Mode", yda165_enum[0],
                        yda165_get_out_mode, yda165_set_out_mode),
	SOC_ENUM_EXT("Amp Enable", yda165_enum[2],
	  		   yda165_get_power_control, yda165_set_power_control),
};

/* This function is called from ASoC machine driver */
int yda165_add_controls(struct snd_soc_codec *codec)
{
	return snd_soc_add_controls(codec, yda165_controls,
			ARRAY_SIZE(yda165_controls));
}
EXPORT_SYMBOL_GPL(yda165_add_controls);

static int __devinit yda165_i2c_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
	P("");

#if defined(YDA165_USE_GPIO_I2C)
	yda165_dummy_i2c_client = client;
#else
	yda165_i2c_client = client;
#endif

	if (gpio_request(OMAP_GPIO_AMP_SHDN , "OMAP_GPIO_AMP_SHDN") == 0)
	{
		gpio_direction_output(OMAP_GPIO_AMP_SHDN , 1);
	}else
		printk("[YDA165] fail to gpio request OMAP_GPIO_AMP_SHDN\n");

	gpio_set_value(OMAP_GPIO_AMP_SHDN, 1);
	wake_lock_init( &yda165_wakelock, WAKE_LOCK_SUSPEND, "yda165_audio");

	return 0;
}

static __devexit int yda165_i2c_remove(struct i2c_client *client)
{
        yda165_i2c_client = NULL;

        return 0;
}

static int yda165_suspend(struct i2c_client *client, pm_message_t mesg)
{
	P("");
	curr_output_mode = OUTPUT_OFF;
	return 0;
}

static void yda165_shutdown(struct i2c_client *client)
{
	printk("shutdown !!");
	curr_output_mode = OUTPUT_OFF;
	yda165_regs[0] = 0x0;
	yda165_regs[1] = 0x0;
	yda165_regs[2] = 0x0;
	yda165_regs[3] = 0x0;
	yda165_regs[4] = 0x40;
	yda165_write_regs();
}

static int yda165_resume(struct i2c_client *client)
{
	curr_output_mode = OUTPUT_OFF;
	return 0;
}

static const struct i2c_device_id yda165_i2c_id[] = {
        { "yda165", 0 },
        { }
};
MODULE_DEVICE_TABLE(yda165_i2c_client, yda165_i2c_id);

static struct i2c_driver yda165_i2c_driver = {
        .driver = {
                .name = "yda165",
                .owner = THIS_MODULE,
        },
	.probe = yda165_i2c_probe,
	.remove = __devexit_p(yda165_i2c_remove),
	.shutdown = yda165_shutdown,
	.suspend = yda165_suspend,
	.resume = yda165_resume,
	.id_table = yda165_i2c_id,
};

static int __init yda165_init(void)
{
	int ret = 0;
	printk("yda165_init\n");
#if defined(YDA165_USE_GPIO_I2C)
	yda165_i2c_client = omap_gpio_i2c_init(OMAP_GPIO_AP_I2C_SDA,
						  OMAP_GPIO_AP_I2C_SCL,
						  0x4D,
						  200);
	if(yda165_i2c_client == NULL)
	{
		printk(KERN_ERR "[FG] omap_gpio_i2c_init failed!\n");
	}
#endif
	ret = i2c_add_driver(&yda165_i2c_driver);
	if(ret)
		printk( KERN_ERR "[YDA165] i2c_add_driver failed");

	curr_output_mode = OUTPUT_OFF;
	return ret;
}
module_init(yda165_init);

static void __exit yda165_exit(void)
{
#if defined(YDA165_USE_GPIO_I2C)
	omap_gpio_i2c_deinit(yda165_i2c_client);
#else
	i2c_del_driver(&yda165_i2c_driver);
#endif

}
module_exit(yda165_exit);

MODULE_DESCRIPTION("ASoC YDA165 amp driver");
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_LICENSE("GPL");

