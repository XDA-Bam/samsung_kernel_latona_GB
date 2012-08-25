/*
 * module/samsung_battery/fuelgauge_max17040.c
 *
 * SAMSUNG battery driver for Linux
 *
 * Copyright (C) 2009 SAMSUNG ELECTRONICS.
 * Author: EUNGON KIM (egstyle.kim@samsung.com)
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <mach/gpio.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/rtc.h>

#include "common.h"

#if defined(CONFIG_USE_GPIO_I2C)
    #include <plat/i2c-omap-gpio.h>
#endif    

#define DRIVER_NAME "secFuelgaugeDev"

#define MAX17042
#define TEMPERATURE_FROM_MAX17042
//#define FUEL_DEBUG_MODE

#ifdef FUEL_DEBUG_MODE
#include <linux/wakelock.h>
#endif
/* Slave address */
#define MAX17042_SLAVE_ADDR	0x36
/* Register address */
#define STATUS_REG			0x00
#define VALRT_THRESHOLD_REG	0x01
#define TALRT_THRESHOLD_REG	0x02
#define SALRT_THRESHOLD_REG	0x03
#define REMCAP_REP_REG		0x05
#define SOCREP_REG			0x06
#define TEMPERATURE_REG		0x08
#define VCELL_REG			0x09
#define CURRENT_REG			0x0A
#define AVG_CURRENT_REG		0x0B
#define SOCMIX_REG			0x0D
#define SOCAV_REG			0x0E
#define REMCAP_MIX_REG		0x0F
#define FULLCAP_REG			0x10
#define RFAST_REG			0x15
#define CYCLES_REG			0x17
#define DESIGNCAP_REG		0x18
#define AVR_VCELL_REG		0x19
#define CONFIG_REG			0x1D
#define REMCAP_AV_REG		0x1F
#define VERSION_REG			0x21
#define FullCAP_NOM_REG		0x23
#define MISCCFG_REG			0x2B
#define RCOMP_REG			0x38
#define FSTAT_REG			0x3D
#define dQacc_REG			0x45
#define dPacc_REG			0x46
#define OCV_REG				0xEE
#define VFOCV_REG			0xFB
#define VFSOC_REG			0xFF

#define LOW_BATT_COMP_RANGE_NUM	5
#define MAX_LOW_BATT_CHECK_CNT	10  /* 20 seconds */

#if defined(CONFIG_USE_GPIO_I2C)
    static OMAP_GPIO_I2C_CLIENT * fuelgauge_i2c_client;
	static struct i2c_client * fuelgauge_i2c_dummy_client;
#else
    static struct i2c_client * fuelgauge_i2c_client;
#endif

struct delayed_work fuelgauge_work;

static SEC_battery_charger_info *sec_bci;

// Prototype
       int get_fuelgauge_adc_value(      bool );
       int get_fuelgauge_ptg_value( bool );
	   int get_fuelgauge_current( bool is_sleep, bool avg );
	   int fuelgauge_reset_soc(void);
	   int fuelgauge_reset_capacity(int sel);
	   void fuelgauge_fullcharged_compensation(u32 is_recharging, u32 pre_update);
	   int get_battery_type(void);
	   int do_low_batt_compensation(int fg_soc,int fg_vcell, int fg_current);
	   void fuelgauge_check_vf_fullcap_range(void);
	   int fuelgauge_check_cap_corruption(void);
static int fuelgauge_check_battery_present(void);
static u16 i2c_read( unsigned char );
static int i2c_write( unsigned char *, u8 );
static int fuelgauge_i2c_write(u8 addr, u16 w_data);
#if !defined(CONFIG_USE_GPIO_I2C)
static u16 i2c_read_dur_sleep( unsigned char );
#endif
static irqreturn_t low_battery_isr( int, void * );
static void fuelgauge_work_handler( struct work_struct * );
static int fuelgauge_probe( struct i2c_client *, const struct i2c_device_id * );
static int fuelgauge_remove( struct i2c_client * );
static void fuelgauge_shutdown( struct i2c_client * );
static int fuelgauge_suspend( struct i2c_client * , pm_message_t );
static int fuelgauge_resume( struct i2c_client * );
       int fuelgauge_init( void );
       void fuelgauge_exit( void );
static void fuelgauge_low_batt_compensation(u32 level);

s32 fuelgauge_read_batt_temp(void);

extern SEC_battery_charger_info *get_sec_bci( void );
extern int _low_battery_alarm_( void );
extern s32 normal_i2c_read_word( u8 , u8 , u8 * );
extern s32 t2_read_word(u8 , u8 , u8 *);
extern int get_jig_status();

#ifdef FUEL_DEBUG_MODE
struct delayed_work fg_monitor_work;
struct delayed_work fuelgauge_work;
static struct wake_lock sec_fg_wakelock;
#endif
struct delayed_work low_batt_compensation_work;


static const struct i2c_device_id fuelgauge_i2c_id[] = {
    { DRIVER_NAME, 0 },
    { },
};

const int atl_temp_table[][2] = {
	{-51311,		-200},
	{-50431,		-190},
	{-49551,		-180},
	{-47791,		-170},
	{-46911,		-160},
	{-46031,		-150},
	{-45151,		-140},
	{-43391,		-130},
	{-42511,		-120},
	{-41631,		-110},
	{-40751,		-100},
	{-39871,		-90},
	{-38111,		-80},
	{-35471,		-70},
	{-34591,		-60},
	{-33711,		-50},
	{-31951,		-40},
	{-31071,		-30},
	{-29311,		-20},
	{-28451,		-10},
	{-27551,		0},
	{-26651,		10},
	{463,		20},
	{1838,		30},
	{3250,		40},
	{4702,		50},
	{6152,		60},
	{7468,		70},
	{8822,		80},
	{10140,		90},
	{11546,		100},
	{12826,		110},
	{14058,		120},
	{15370,		130},
	{16405,		140},
	{17717,		150},
	{18780,		160},
	{20000,		170},
	{21031,		180},
	{22136,		190},
	{23156,		200},
	{24198,		210},
	{25027,		220},
	{26124,		230},
	{26920,		240},
	{27608,		250},
	{28421,		260},
	{29218,		270},
	{30074,		280},
	{30838,		290},
	{31343,		300},
	{32074,		310},
	{32604,		320},
	{33370,		330},
	{33889,		340},
	{34713,		350},
	{35276,		360},
	{35842,		370},
	{36354,		380},
	{36873,		390},
	{37417,		400},
	{37920,		410},
	{38405,		420},
	{38826,		430},
	{39276,		440},
	{39729,		450},
	{40172,		460},
	{40510,		470},
	{40916,		480},
	{41312,		490},
	{41670,		500},
	{42042,		510},
	{42354,		520},
	{42655,		530},
	{42931,		540},
	{43270,		550},
	{43562,		560},
	{43854,		570},
	{44074,		580},
	{44405,		590},
	{44577,		600},
	{44811,		610},
	{45183,		620},
	{45370,		630},
	{45542,		640},
	{45635,		650},
	{45947,		660},
	{46136,		670},
	{46308,		680},
	{46514,		690},
	{46655,		700},
};

/* ======= For low battery compensation. ======= */

// SDI type Offset
#define SDI_Range4_1_Offset		3320
#define SDI_Range4_3_Offset		3410
#define SDI_Range3_1_Offset		3451
#define SDI_Range3_3_Offset		3454
#define SDI_Range2_1_Offset		3461
#define SDI_Range2_3_Offset		3544
#define SDI_Range1_1_Offset		3456
#define SDI_Range1_3_Offset		3536

#define SDI_Range4_1_Slope		0
#define SDI_Range4_3_Slope		0
#define SDI_Range3_1_Slope		97
#define SDI_Range3_3_Slope		27
#define SDI_Range2_1_Slope		96
#define SDI_Range2_3_Slope		134
#define SDI_Range1_1_Slope		0
#define SDI_Range1_3_Slope		0

// ATL type threshold
#define ATL_Range5_1_Offset		3277
#define ATL_Range5_3_Offset		3293
#define ATL_Range4_1_Offset		3312
#define ATL_Range4_3_Offset		3305
#define ATL_Range3_1_Offset		3310
#define ATL_Range3_3_Offset		3333
#define ATL_Range2_1_Offset		3335
#define ATL_Range2_3_Offset		3356
#define ATL_Range1_1_Offset		3325
#define ATL_Range1_3_Offset		3342

#define ATL_Range5_1_Slope		0
#define ATL_Range5_3_Slope		0
#define ATL_Range4_1_Slope		30  // 0.03
#define ATL_Range4_3_Slope		667  // 0.00667
#define ATL_Range3_1_Slope		20
#define ATL_Range3_3_Slope		40
#define ATL_Range2_1_Slope		60
#define ATL_Range2_3_Slope		76
#define ATL_Range1_1_Slope		0
#define ATL_Range1_3_Slope		0


// SDI Battery Data
#define SDI_Capacity 0x1F40  // 4000mAh
#define SDI_VFCapacity 0x29AC  // 5333mAh
// ATL Battery Data
#define ATL_Capacity 0x1FBE  // 4063mAh
#define ATL_VFCapacity 0x2A54  // 5418mAh

typedef enum {	
	POSITIVE = 0,	
	NEGATIVE = 1
} sign_type_t;

typedef enum {
	UNKNOWN_TYPE = 0,
	SDI_BATTERY_TYPE,
	ATL_BATTERY_TYPE
} battery_type_t;

static u16 Capacity = SDI_Capacity;
static u16 VFCapacity = 0;
static u32 battery_type = UNKNOWN_TYPE;


static u32 PrevFullCap = 0;
static u32 PrevVfFullCap = 0;
static u32 FirstFullChargedCAP = 0;


static u32 prevVfSOC = 0;
static u32 prevRepSOC = 0;
static u32 prevRemCap = 0;
static u32 prevMixCap = 0;
static u32 prevFullCapacity= 0;
static u32 prevVFCapacity= 0;
static u32 prevVfOCV = 0;

static int low_batt_comp_cnt[LOW_BATT_COMP_RANGE_NUM][2] = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} };
static int check_start_vol = 0;
int lowbatt_soc, lowbatt_vcell, lowbatt_current, lowbatt_count;

spinlock_t fg_lock;


#if 1 //!defined(CONFIG_USE_GPIO_I2C)
static struct i2c_driver fuelgauge_i2c_driver =
{
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
    },       

    .probe      = fuelgauge_probe,
    .remove     = __devexit_p( fuelgauge_remove ),
    .shutdown   = fuelgauge_shutdown,
    .suspend    = fuelgauge_suspend,
    .resume     = fuelgauge_resume,
    .id_table   = fuelgauge_i2c_id,
};
#endif

static void fuelgauge_test_print(void)
{
	u8 data[2];
	u32 average_vcell = 0;
	u16 w_data;
	u32 temp;
	u32 temp2;
	u16 reg_data;
	u16 read_data;

	read_data = i2c_read(AVR_VCELL_REG);
	//data[1] = (read_data >> 8);
	//data[0] = read_data & 0xFF;


	w_data = read_data;

	temp = (w_data & 0xFFF) * 78125;
	average_vcell = temp / 1000000;

	temp = ((w_data & 0xF000) >> 4) * 78125;
	temp2 = temp / 1000000;
	average_vcell += (temp2 << 4);

	printk("%s : AVG_VCELL(%d), data(0x%04x)\n", __func__, average_vcell, read_data);

	reg_data = i2c_read(FULLCAP_REG);
	printk("%s : FULLCAP(%d), data(0x%04x)\n", __func__, reg_data/2, reg_data);

	reg_data = i2c_read(REMCAP_MIX_REG);
	printk("%s : REMCAP_MIX(%d), data(0x%04x)\n", __func__, reg_data/2, reg_data);

	reg_data = i2c_read(REMCAP_AV_REG);
	printk("%s : REMCAP_AV(%d), data(0x%04x)\n", __func__, reg_data/2, reg_data);

}



int get_fuelgauge_adc_value(      bool is_sleep )
{
    u16 result;
    u32 temp;
	u32 temp2;
	u32 vcell = 0;
	
#if !defined(CONFIG_USE_GPIO_I2C)
    if( is_sleep )
        result = i2c_read_dur_sleep( VCELL_REG );
    else
        result = i2c_read( VCELL_REG );
#else
    result = i2c_read( VCELL_REG );
#endif
	//result = ((result & 0xFF) << 8) | ((result & 0xFF00) >> 8);

	temp = (result & 0xFFF) * 78125;
 	vcell = temp / 1000000;

	temp = ((result & 0xF000) >> 4) * 78125;	
	temp2 = temp / 1000000;	
	vcell += (temp2 << 4);

#ifdef FUEL_DEBUG_MODE
	printk("%s : VCELL(%d), data(0x%04x)\n", __func__, vcell, result);
#endif
	//printk("%s, vcell:%d\n", __func__, vcell);
    return vcell;
}

int get_fuelgauge_ptg_value( bool is_sleep )
{
    u16 val;
    u16 val2;
	int soc = 0;	
	u32 temp = 0;
	u32 remcap = 0;
	u32 fullcap = 0;
	
#if !defined(CONFIG_USE_GPIO_I2C)
    if( is_sleep )
	{
        val = i2c_read_dur_sleep( SOCREP_REG ); 
		val2 = i2c_read_dur_sleep( REMCAP_REP_REG ); 
   	}
    else
   	{
        val = i2c_read( SOCREP_REG );
		val2 = i2c_read( REMCAP_REP_REG );
   	}
#else
    val = i2c_read( SOCREP_REG );
	val2 = i2c_read( REMCAP_REP_REG );
#endif

	//temp = ((val & 0xFF00) >> 8) * 39 / 1000;
	//soc = (val & 0xFF);
	temp = (val & 0xFF) * 39 / 1000;
	soc = val >> 8;

	remcap = val2 / 2;
	fullcap = i2c_read(FULLCAP_REG);

#ifdef FUEL_DEBUG_MODE
	printk("%s : SOC(%d), data(0x%04x)\n", __func__, soc, val);
	printk("%s : FullCAP(%d), data(0x%04x)\n", __func__, (fullcap/2), (u16)fullcap);
	printk("%s : RemCAP(%d), data(0x%04x)\n", __func__, remcap, val2);
#endif
	//printk("%s, soc:%d\n", __func__, soc);
    return ( soc > 100 ) ? 100 : soc;
}

int fuelgauge_read_vfsoc(void)
{
	u8 data[2];
	u32 vfsoc = 0;
	u32 temp = 0;
	u16 read_data;

	read_data = i2c_read(VFSOC_REG);
	data[1] = (read_data >> 8);
	data[0] = read_data & 0xFF;

	temp = data[0] * 39 / 1000;

	vfsoc = data[1];

	return vfsoc;
}

int get_fuelgauge_current( bool is_sleep, bool avg )
{
    u32 val;
    u32 val2;
	u32 temp, sign;
	s32 i_current = 0;
	s32 avg_current = 0;
	
#if !defined(CONFIG_USE_GPIO_I2C)
    if( is_sleep )
	{
        val = i2c_read_dur_sleep( CURRENT_REG ); 
		val2 = i2c_read_dur_sleep( AVG_CURRENT_REG ); 
   	}
    else
   	{
        val = i2c_read( CURRENT_REG );
		val2 = i2c_read( AVG_CURRENT_REG );
   	}
#else
    val = i2c_read( CURRENT_REG );
	val2 = i2c_read( AVG_CURRENT_REG );
#endif
	
	temp = val & 0xFFFF;	
	if(temp & (0x1 << 15))
	{
		sign = NEGATIVE;
		temp = (~(temp) & 0xFFFF) + 1;
	}
	else
		sign = POSITIVE;

	temp = temp * 15625;	
	i_current = temp / 100000;	
	if(sign)		
		i_current *= -1;

	temp = val2 & 0xFFFF;
	if(temp & (0x1 << 15))
	{
		sign = NEGATIVE;
		temp = (~(temp) & 0xFFFF) + 1;
	}
	else
		sign = POSITIVE;

	temp = temp * 15625;	
	avg_current = temp / 100000;
	if(sign)
		avg_current *= -1;
	if(avg)
		return avg_current;
	else
		return i_current;
}

int get_fuelgauge_avg_current(void)
{
	u8  data2[2];
	u32 temp, sign;
	s32 avg_current = 0;
	u16 read_data;

	read_data = i2c_read(AVG_CURRENT_REG);

	data2[1] = (read_data >> 8);
	data2[0] = read_data & 0xFF;

	temp = ((data2[1]<<8) | data2[0]) & 0xFFFF;
	if(temp & (0x1 << 15))
	{
		sign = NEGATIVE;
		temp = (~(temp) & 0xFFFF) + 1;
	}
	else
		sign = POSITIVE;

	temp = temp * 15625;
	avg_current = temp / 100000;
	
	if(sign)
		avg_current *= -1;

	return avg_current;
}

int fuelgauge_reset_soc(void)
{
	u8 data[2];
	s32 ret = 0;
	u16 read_data;
	//printk("%s : Before quick-start - VfOCV(%d), VfSOC(%d), RepSOC(%d)\n",
	//				__func__, fg_read_vfocv(), fg_read_vfsoc(), fg_read_soc());

	if(sec_bci->charger.is_charging) {
		printk("%s : Return by DCIN input (TA or USB)\n", __func__);
		return 0;
	}

	if(!get_jig_status()) {
		printk("%s : Return by No JIG_ON signal\n", __func__);
		return 0;
	}

	// cycle 0
	fuelgauge_i2c_write(CYCLES_REG, (u16)(0x0));

	read_data = i2c_read(MISCCFG_REG);
	read_data |= (0x1 << 10); // Set bit10 makes quick start

	fuelgauge_i2c_write(MISCCFG_REG, (u16)(read_data));

	msleep(250);
	fuelgauge_i2c_write(FULLCAP_REG, Capacity);  // FullCAP
	msleep(500);

	//printk("%s : After quick-start - VfOCV(%d), VfSOC(%d), RepSOC(%d)\n",
	//				__func__, fg_read_vfocv(), fg_read_vfsoc(), fg_read_soc());

	// cycle 160
	fuelgauge_i2c_write(CYCLES_REG, (u16)(0x00a0));

	return ret;
}


int fuelgauge_reset_capacity(int sel)
{
	s32 ret = 0;
	
	ret = fuelgauge_i2c_write(DESIGNCAP_REG, VFCapacity-1);  // DesignCAP
	return ret;
}

void set_fuelgauge_battery_type()
{
	u16 data = 0;
	u8 type_str[10];
	
	data = i2c_read( DESIGNCAP_REG );

	if((data == SDI_VFCapacity) || (data == SDI_VFCapacity-1))
		battery_type = SDI_BATTERY_TYPE;
	else if((data == ATL_VFCapacity) || (data == ATL_VFCapacity-1))
		battery_type = ATL_BATTERY_TYPE;

	if(battery_type == SDI_BATTERY_TYPE)
		sprintf(type_str, "SDI");
	else if(battery_type == ATL_BATTERY_TYPE)
		sprintf(type_str, "ATL");
	else
		sprintf(type_str, "Unknown");

	printk("%s : DesignCAP(0x%04x), Battery type(%s)\n", __func__, data, type_str);

	switch(battery_type) {
		case ATL_BATTERY_TYPE:
			Capacity = ATL_Capacity;
			VFCapacity = ATL_VFCapacity;
			break;
		case SDI_BATTERY_TYPE:
		default:
			Capacity = SDI_Capacity;
			VFCapacity = SDI_VFCapacity;
			break;
	}
}

int fuelgauge_read_vfocv(void)
{
	u8 data[2];
	u32 vfocv = 0;
	u16 w_data;
	u32 temp;
	u32 temp2;
	u16 read_data;

	read_data = i2c_read(VFOCV_REG);
	data[1] = (read_data >> 8);
	data[0] = read_data & 0xFF;
	
	w_data = (data[1]<<8) | data[0];

	temp = (w_data & 0xFFF) * 78125;
	vfocv = temp / 1000000;

	temp = ((w_data & 0xF000) >> 4) * 78125;
	temp2 = temp / 1000000;
	vfocv += (temp2 << 4);

	return vfocv;
}

static s32 fuelgauge_read_temp(void)
{
	u8 data[2];
	s32 temper = 0;
	s32 trim1 = 15385;
	s32 trim2 = 22308;
	u16 read_data;

	if (read_data = i2c_read(TEMPERATURE_REG) < 0) {
		pr_err("%s: Failed to read SOCREP\n", __func__);
		return -1;
	}
	data[1] = (read_data >> 8);
	data[0] = read_data & 0xFF;

	if(data[1]&(0x1 << 7)) //Negative
	{
		temper = ((~(data[1]))&0xFF)+1;
		temper *= (-1000);
	}
	else
	{
		temper = data[1] & 0x7f;
		temper *= 1000;
		temper += data[0] * 39 / 10;
		if(temper > 47000)
			temper = temper * trim1/10000 - trim2;
	}
#ifdef FUEL_DEBUG_MODE
	printk("%s, TEMPERATURE(%d), data(%x)\n", __func__, temper, (data[1] << 8) | data[0]);
#endif
	return temper;
}


static s32 fuelgauge_read_temp2(void)
{
	u8 data[2];
	s32 temper = 0;
	s32 trim1_1 = 122;
	s32 trim1_2 = 8950;
	s32 trim2_1 = 200;
	s32 trim2_2 = 51000;
	u16 read_data;

	read_data = i2c_read(TEMPERATURE_REG);

	data[1] = (read_data >> 8);
	data[0] = read_data & 0xFF;
	//printk("fuelgauge_read_temp2 TEMPERATURE_REG : %x, 1:%x, 0:%x\n", read_data, data[1], data[0]);

	if(data[1]&(0x1 << 7)) //Negative
	{
		temper = ((~(data[1]))&0xFF)+1;
		temper *= (-1000);
	}
	else
	{
		temper = data[1] & 0x7f;
		temper *= 1000;
		temper += data[0] * 39 / 10;
		if(temper >= 47000 && temper <60000)
			temper = temper * trim1_1/100 - trim1_2;
		else if(temper >=60000)
			temper = temper * trim2_1/100 - trim2_2;
	}

	return temper;
}
static s32 fuelgauge_read_temp3(void)
{
	u8 data[2];
	s32 temper = 0;
	s32 trim1_1 = 257;
	s32 trim1_2 = 126785;
	s32 trim2_1 = 88;
	s32 trim2_2 = 24911;
	int array_size = 0;
	int table_temp = 0;
	int i = 0;
	u16 read_data;

/*	if (read_data = i2c_read(TEMPERATURE_REG) < 0) {
		pr_err("%s: Failed to read SOCREP\n", __func__);
		return -1;
	}*/

	read_data = i2c_read(TEMPERATURE_REG);
	data[1] = (read_data >> 8);
	data[0] = read_data & 0xFF;

	//printk("fuelgauge_read_temp3 read_data : %x, data[1] : %x, data[0] : %x\n", read_data, data[1], data[0]);
	if(data[1]&(0x1 << 7)) //Negative
	{
		temper = ((~(data[1]))&0xFF)+1;
		temper *= (-1000);
		temper = temper * trim2_1/100 - trim2_2;
	}
	else
	{
		temper = data[1] & 0x7f;
		temper *= 1000;
		temper += data[0] * 39 / 10;
	}

	array_size = ARRAY_SIZE(atl_temp_table);
	for (i = 0; i < (array_size - 1); i++) {
		if (i == 0) {
			if (temper <= atl_temp_table[0][0]) {
				table_temp = atl_temp_table[0][1];
				break;
			} else if (temper >= atl_temp_table[array_size-1][0]) {
				table_temp = atl_temp_table[array_size-1][1];
				break;
			}
		}

		if (atl_temp_table[i][0] < temper &&
				atl_temp_table[i+1][0] >= temper) {
			table_temp = atl_temp_table[i+1][1];
		}
	}
	//printk("ATL temp : fuel_temp(%d), table_temp(%d)\n", temper, table_temp);

	return (table_temp * 100);
}

s32 fuelgauge_read_batt_temp(void)
{
	if(fuelgauge_check_battery_present())
	{
		//printk("fg_read_batt_temp battery_type = %d\n", battery_type);
		if(battery_type == SDI_BATTERY_TYPE)
			return fuelgauge_read_temp2();
		else if(battery_type == ATL_BATTERY_TYPE)
			return fuelgauge_read_temp3();
		else
			return (s32)30000;
	}			
	else
	{
		return (s32)20000;
	}
}

static int i2c_read_mbyte( unsigned char reg_addr,  u16* r_data, int len )
{
#if defined(CONFIG_USE_GPIO_I2C)
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;

	i2c_rd_param.reg_len = 1;
	i2c_rd_param.reg_addr = &reg_addr;
	i2c_rd_param.rdata_len = len;
	i2c_rd_param.rdata = r_data;
	omap_gpio_i2c_read(fuelgauge_i2c_client, &i2c_rd_param);
#else
	printk("%s, Not supported this function\n", __func__);
	return -1;
#endif
}


static u16 i2c_read( unsigned char reg_addr )
{
    u16 ret = 0;
    unsigned char buf[2];

#if defined(CONFIG_USE_GPIO_I2C)
    OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
#else
    struct i2c_msg msg1[1],msg2[1];
#endif

#if defined(CONFIG_USE_GPIO_I2C)
    i2c_rd_param.reg_len = 1;
    i2c_rd_param.reg_addr = &reg_addr;
    i2c_rd_param.rdata_len = 2;
    i2c_rd_param.rdata = buf;
    omap_gpio_i2c_read(fuelgauge_i2c_client, &i2c_rd_param);
#else
    msg1->addr = fuelgauge_i2c_client->addr;
    msg1->flags = I2C_M_WR;
    msg1->len = 1;
    msg1->buf = &reg_addr;

    ret = i2c_transfer(fuelgauge_i2c_client->adapter, msg1, 1);
    if( ret < 0 )
    {
        printk( KERN_ERR "[FG] fail to read max17040." );
        return -1;
    }
    else
    {
        msg2->addr = fuelgauge_i2c_client->addr;
        msg2->flags = I2C_M_RD;
        msg2->len = 2;
        msg2->buf = buf;

        ret = i2c_transfer( fuelgauge_i2c_client->adapter, msg2, 1 );

        if( ret < 0 )
        {
            printk( KERN_ERR "[FG] fail to read max17040." );
            return -1;
        }
    }
#endif

    //ret = buf[0] << 8 | buf[1];  // original
    ret = buf[1] << 8 | buf[0];

    return ret;
}
static int fuelgauge_i2c_write(u8 addr, u16 w_data)
{
	unsigned char buf[3];
	buf[0] = addr;
	buf[1] = w_data& 0xFF;
	buf[2] = w_data >> 8;
	i2c_write( buf, 2 );
	return 0;
}

static int fuelgauge_i2c_write_and_verify(u8 addr, u16 w_data)
{
	unsigned char buf[3];
	u8 retry_cnt = 2;
	u16 read_data;

retry_write:
	buf[0] = addr;
	buf[1] = w_data& 0xFF;
	buf[2] = w_data >> 8;
	i2c_write( buf, 3 );

	read_data = i2c_read(addr);

	if(read_data != w_data)
	{
		printk("%s: verification failed (addr : 0x%x, w_data : 0x%x, r_data : 0x%x)\n", __func__, addr, w_data, read_data);
		if(retry_cnt--)
			goto retry_write;
	}
	
	return 0;
}


static int i2c_write( unsigned char *buf, u8 len )
{
    int ret = 0;

#if defined(CONFIG_USE_GPIO_I2C)
    OMAP_GPIO_I2C_WR_DATA i2c_wr_param;
#else
    struct i2c_msg msg;
#endif

#if !defined(CONFIG_USE_GPIO_I2C)
    msg.addr    = fuelgauge_i2c_client->addr;
    msg.flags = I2C_M_WR;
    msg.len = len;
    msg.buf = buf;

    ret = i2c_transfer( fuelgauge_i2c_client->adapter, &msg, 1 );

    if( ret < 0 )
    {
        printk( KERN_ERR "[FG] fail to write max17040." );
        return -1;
    }
#else
    i2c_wr_param.reg_len = 1;
    i2c_wr_param.reg_addr = &(buf[0]);
    i2c_wr_param.wdata_len = len;
    i2c_wr_param.wdata = &(buf[1]);
    omap_gpio_i2c_write(fuelgauge_i2c_client, &i2c_wr_param);
#endif

    return ret;
}

#if !defined(CONFIG_USE_GPIO_I2C)
static u16 i2c_read_dur_sleep( unsigned char reg_addr )
{
    unsigned char buf[2];
    unsigned int ret;

    ret = t2_read_word(0x36, reg_addr, buf);
    if(ret < 0)
    {
        printk(KERN_ERR"[%s] Fail to Read max17040\n", __FUNCTION__);
        return -1;
    } 

    ret = buf[0] << 8 | buf[1];

    return ret;
}
#endif

 
static irqreturn_t low_battery_isr( int irq, void *_di )
{
    if ( sec_bci->ready )
    {
        cancel_delayed_work( &fuelgauge_work );
        schedule_delayed_work( &fuelgauge_work, 0 );
    }
    
    return IRQ_HANDLED; 
}

static void reset_low_batt_comp_cnt(void)
{
	memset(low_batt_comp_cnt, 0x0, sizeof(low_batt_comp_cnt));
//	printk("%s : Reset check array count.\n", __func__);
}

static void display_low_batt_comp_cnt(void)
{
	u8 type_str[10];

	if(battery_type == SDI_BATTERY_TYPE)
		sprintf(type_str, "SDI");
	else if(battery_type == ATL_BATTERY_TYPE)
		sprintf(type_str, "ATL");
	else
		sprintf(type_str, "Unknown");

	printk("Check Array(%s) : [%d, %d], [%d, %d], [%d, %d], [%d, %d], [%d, %d]\n", type_str,
			low_batt_comp_cnt[0][0], low_batt_comp_cnt[0][1], low_batt_comp_cnt[1][0], low_batt_comp_cnt[1][1],
			low_batt_comp_cnt[2][0], low_batt_comp_cnt[2][1], low_batt_comp_cnt[3][0], low_batt_comp_cnt[3][1],
			low_batt_comp_cnt[4][0], low_batt_comp_cnt[4][1]);
}

static int check_low_batt_comp_condtion(int* nLevel)
{
	int i = 0;
	int j = 0;
	int ret = 0;

	for(i = 0; i < LOW_BATT_COMP_RANGE_NUM; i++)
	{
		for(j = 0; j < 2; j++)
		{
			if(low_batt_comp_cnt[i][j] >= MAX_LOW_BATT_CHECK_CNT)
			{
				display_low_batt_comp_cnt();

				ret = 1;
				*nLevel = j*2 + 1;  // 0->1%, 1->3%
				break;
			}
		}
	}

	return ret;
}

static void add_low_batt_comp_cnt(int range, int level)
{
	int i = 0;
	int j = 0;

	// Increase the requested count value, and reset others.
	low_batt_comp_cnt[range-1][level/2] ++;

	for(i = 0; i < LOW_BATT_COMP_RANGE_NUM; i++)
	{
		for(j = 0; j < 2; j++)
		{
			if(i == range-1 && j == level/2)
				continue;  // keep the count value.
			else
				low_batt_comp_cnt[i][j] = 0;  // reset
		}
	}
}

static int get_low_batt_threshold(int range, int level, int nCurrent)
{
	int ret = 0;

	if(battery_type == SDI_BATTERY_TYPE)
	{
		switch(range) {
		case 4:
			if(level == 1)
				ret = SDI_Range4_1_Offset + ((nCurrent * SDI_Range4_1_Slope) / 1000);
			else if(level == 3)
				ret = SDI_Range4_3_Offset + ((nCurrent * SDI_Range4_3_Slope) / 1000);
			break;
		
		case 3:
			if(level == 1)
				ret = SDI_Range3_1_Offset + ((nCurrent * SDI_Range3_1_Slope) / 1000);
			else if(level == 3)
				ret = SDI_Range3_3_Offset + ((nCurrent * SDI_Range3_3_Slope) / 1000);
			break;
		
		case 2:
			if(level == 1)
				ret = SDI_Range2_1_Offset + ((nCurrent * SDI_Range2_1_Slope) / 1000);
			else if(level == 3)
				ret = SDI_Range2_3_Offset + ((nCurrent * SDI_Range2_3_Slope) / 1000);
			break;
		
		case 1:
			if(level == 1)
				ret = SDI_Range1_1_Offset + ((nCurrent * SDI_Range1_1_Slope) / 1000);
			else if(level == 3)
				ret = SDI_Range1_3_Offset + ((nCurrent * SDI_Range1_3_Slope) / 1000);
			break;
		
		default:
			break;
		}
	}
	else if(battery_type == ATL_BATTERY_TYPE)
	{
		switch(range) {
		case 5:
			if(level == 1)
				ret = ATL_Range5_1_Offset + ((nCurrent * ATL_Range5_1_Slope) / 1000);
			else if(level == 3)
				ret = ATL_Range5_3_Offset + ((nCurrent * ATL_Range5_3_Slope) / 1000);
			break;

		case 4:
			if(level == 1)
				ret = ATL_Range4_1_Offset + ((nCurrent * ATL_Range4_1_Slope) / 1000);
			else if(level == 3)
				ret = ATL_Range4_3_Offset + ((nCurrent * ATL_Range4_3_Slope) / 100000);  // Slope value range is different
			break;
		
		case 3:
			if(level == 1)
				ret = ATL_Range3_1_Offset + ((nCurrent * ATL_Range3_1_Slope) / 1000);
			else if(level == 3)
				ret = ATL_Range3_3_Offset + ((nCurrent * ATL_Range3_3_Slope) / 1000);
			break;
		
		case 2:
			if(level == 1)
				ret = ATL_Range2_1_Offset + ((nCurrent * ATL_Range2_1_Slope) / 1000);
			else if(level == 3)
				ret = ATL_Range2_3_Offset + ((nCurrent * ATL_Range2_3_Slope) / 1000);
			break;
		
		case 1:
			if(level == 1)
				ret = ATL_Range1_1_Offset + ((nCurrent * ATL_Range1_1_Slope) / 1000);
			else if(level == 3)
				ret = ATL_Range1_3_Offset + ((nCurrent * ATL_Range1_3_Slope) / 1000);
			break;
		
		default:
			break;
		}
	}
//	printk("%s : Range%d, Level%d, Avg_Current(%d) -> Threshold(%d)\n", __func__, range, level, avg_current, ret);

	return ret;
}

int get_battery_type()
{
	return battery_type;
}

static int do_real_low_batt_compensation(int fg_soc, int fg_vcell, int fg_current)
{
	int fg_avg_current=0;
	int fg_min_current=0;
	int bCntReset=0;
	int new_level = 0;
	int ret = 0;

	if(!sec_bci->charger.is_charging && !sec_bci->battery.low_batt_comp_flag
		&& (fg_vcell <= check_start_vol))
	{
		printk("do_low_batt_compensation : start_vol = %d, fg_soc = %d, fg_vcell = %d, fg_current = %d\n", 
			check_start_vol, fg_soc, fg_vcell, fg_current);

		fg_avg_current = get_fuelgauge_avg_current();
		fg_min_current = min(fg_avg_current, fg_current);
	
		if(battery_type == SDI_BATTERY_TYPE)
		{
			if(fg_min_current < -1250)  // I > 1.25A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(4, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(4, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(4, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(4, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -1250 && fg_min_current < -750)  // 0.75A < I <= 1.25A
			{		
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(3, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(3, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(3, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(3, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -750 && fg_min_current < -100)  // 0.1A < I <= 0.75A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(2, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(2, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(2, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(2, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -100 && fg_min_current < 0)  // I <= 0.1A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(1, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(1, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(1, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(1, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
		}
		else if(battery_type == ATL_BATTERY_TYPE)
		{
			if(fg_min_current < -1000)  // I > 1A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(5, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(5, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(5, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(5, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -1000 && fg_min_current < -700)  // 0.7A < I <= 1A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(4, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(4, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(4, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(4, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -700 && fg_min_current < -500)  // 0.5A < I <= 0.7A
			{		
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(3, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(3, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(3, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(3, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -500 && fg_min_current < -250)  // 0.25A < I <= 0.5A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(2, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(2, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(2, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(2, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -250 && fg_min_current < 0)  // I <= 0.25A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(1, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(1, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(1, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(1, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
		}

		printk("check_low_batt_comp_condtion\n");
		if(check_low_batt_comp_condtion(&new_level))
		{
			if(get_jig_status() && new_level == 1)
				new_level = 0;
			printk("%s, new_level = %d\n", __func__, new_level);
			fuelgauge_low_batt_compensation(new_level);
			reset_low_batt_comp_cnt();
		}

		if (bCntReset)
			reset_low_batt_comp_cnt();
		
		// if compensation finished, then read SOC again!!
		if(sec_bci->battery.low_batt_comp_flag)
		{
			printk("%s : MIN_CURRENT(%d), AVG_CURRENT(%d), CURRENT(%d), SOC(%d), VCELL(%d)\n",
				__func__, fg_min_current, fg_avg_current, fg_current, fg_soc, fg_vcell);
			fg_soc = get_fuelgauge_ptg_value(false); //fg_read_soc();
			printk("%s : SOC is set to %d\n", __func__, fg_soc);
			ret = 1;
		}
	}
	return ret;
}

static void fg_low_batt_compensation_work_handler( struct work_struct *work )
{

	lowbatt_soc = get_fuelgauge_ptg_value(false);
	lowbatt_vcell = get_fuelgauge_adc_value(false);
	lowbatt_current = get_fuelgauge_current(false, false);

	do_real_low_batt_compensation(lowbatt_soc, lowbatt_vcell, lowbatt_current);
	lowbatt_count--;
	if(lowbatt_count > 0)
	{
		if(get_jig_status())
			schedule_delayed_work( &low_batt_compensation_work, HZ/10 );
		else
			schedule_delayed_work( &low_batt_compensation_work, 2*HZ );
	}
}	

int do_low_batt_compensation(int fg_soc, int fg_vcell, int fg_current)
{
	int nRet=0;
	int fg_avg_current=0;
	int fg_min_current=0;
	int new_level = 0;
	int bCntReset=0;

	if(!check_start_vol)  // If not initialized yet, then init threshold values.
	{
		if(battery_type == SDI_BATTERY_TYPE)
			check_start_vol = 3550;  // Under 3.55V
		else if(battery_type == ATL_BATTERY_TYPE)
			check_start_vol = 3450;  // Under 3.45V
	}
	if(!sec_bci->charger.is_charging && !sec_bci->battery.low_batt_comp_flag
		&& (fg_vcell <= check_start_vol))
	{
		lowbatt_soc = fg_soc;
		lowbatt_vcell = fg_vcell;
		lowbatt_current = fg_current;
		lowbatt_count = 10;
		schedule_delayed_work( &low_batt_compensation_work, HZ/10 );
	}

	/////////////////////////////////////////////////////////////////

	nRet = fg_soc;
	
	return nRet;	
}

static void fuelgauge_work_handler( struct work_struct *work )
{
    printk( "[FG] ext.low_battery!\n" );
    _low_battery_alarm_();
}

void fuelgauge_periodic_read(void)
{
	u8 data[2], reg;
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	printk("[MAX17042] %d/%d/%d %02d:%02d,",
		tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 2000, tm.tm_hour, tm.tm_min);

	for(reg = 0; reg < 0x50; reg++)
		printk("%04xh,", i2c_read(reg));

	for(reg = 0xe0; reg < 0x100; reg++)
	{
		if(reg==0xff) {
			printk("%04xh\n", i2c_read(reg));
			break;
		}
		else
			printk("%04xh,", i2c_read(reg));
	}

}

static void fuelgauge_read_model_data(void)
{
	u16 data0[16], data1[16], data2[16];
	int i = 0;

	printk("[FG_Model] ");

	// Unlock model access
	fuelgauge_i2c_write(0x62, 0x0059);  // Unlock Model Access
	fuelgauge_i2c_write(0x63, 0x00C4);

	// Read model data
	i2c_read_mbyte(0x80, data0, 32);
	i2c_read_mbyte(0x90, data1, 32);
	i2c_read_mbyte(0xa0, data2, 32);

	// Print model data
	for(i = 0; i < 16; i++)
		printk("0x%04x, ", data0[i]);

	for(i = 0; i < 16; i++)
		printk("0x%04x, ", data1[i]);

	for(i = 0; i < 16; i++) {
		if(i==15)
			printk("0x%04x", data2[i]);
		else
			printk("0x%04x, ", data2[i]);
	}
	printk("\n");

relock:
	// Lock model access
	fuelgauge_i2c_write(0x62, 0x0000);  // Lock Model Access
	fuelgauge_i2c_write(0x63, 0x0000);

	// Read model data again
	i2c_read_mbyte(0x80, data0, 32);
	i2c_read_mbyte(0x90, data1, 32);
	i2c_read_mbyte(0xa0, data2, 32);

	for(i=0; i<16; i++) {
		if( data0[i] || data1[i] || data2[i]) {
			printk("%s : data is non-zero, lock again!!\n", __func__);
			goto relock;
		}
	}
	
}

int fuelgauge_check_chip_state(void)
{
	u32 vcell, soc;

	vcell = get_fuelgauge_adc_value(false);
	soc = get_fuelgauge_ptg_value(false);

	printk("%s : vcell(%d), soc(%d)\n", __func__, vcell, soc);
	
	// if read operation fails, then it's not alive status
	if( (vcell < 0) || (soc < 0) )
		return 0;
	else
		return 1;
}

int soc_restart_flag;
int fuelgauge_adjust_capacity(void)
{
	u8 data[2];
	s32 ret = 0;

	data[0] = 0;
	data[1] = 0;

	// 1. Write RemCapREP(05h)=0;
	fuelgauge_i2c_write(REMCAP_REP_REG, data);
	
#if 0  // Not used (Recommendation from MAXIM)
	// 2. Write RemCapMIX(0Fh)=0;
	if (fg_i2c_write(client, REMCAP_MIX_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write RemCap_MIX\n", __func__);
		return -1;
	}

	// 3. Write RemCapAV(1Fh)=0;
	if (fg_i2c_write(client, REMCAP_AV_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write RemCap_AV\n", __func__);
		return -1;
	}

	//4. Write RepSOC(06h)=0;
	if (fg_i2c_write(client, SOCREP_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write SOC_REP\n", __func__);
		return -1;
	}

	//5. Write MixSOC(0Dh)=0;
	if (fg_i2c_write(client, SOCMIX_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write SOC_MIX\n", __func__);
		return -1;
	}

	//6. Write SOCAV(0Eh)=Table_SOC;
	if (fg_i2c_write(client, SOCAV_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write SOC_AV\n", __func__);
		return -1;
	}
#endif
	msleep(200);

	printk("%s : After adjust - RepSOC(%d)\n", __func__, get_fuelgauge_ptg_value(false));

	soc_restart_flag = 1;  // Set flag

	return ret;
}

static void fuelgauge_low_batt_compensation(u32 level)
{
	u16 read_val = 0;
	u32 temp = 0;
	u16 tempVal=0;

	printk("%s : Adjust SOCrep to %d!!\n", __func__, level);

	//1) RemCapREP (05h) = FullCap(10h) x 0.034 (or 0.014)
	read_val = i2c_read(0x10);
	temp = read_val * (level*10 + 4) / 1000;
	fuelgauge_i2c_write(0x05, (u16)temp);

	//2) RemCapMix(0Fh) = RemCapREP
//	fuelgauge_i2c_write(0x0f, (u16)temp);

	//3) RemCapAV(1Fh) = RemCapREP; 
//	fuelgauge_i2c_write(0x1f, (u16)temp);

	//4) RepSOC (06h) = 3.4% or 1.4%
	tempVal=(u16)((level << 8) | 0x67);  // 103(0x67) * 0.0039 = 0.4%
	fuelgauge_i2c_write(0x06, tempVal);

	//5) MixSOC (0Dh) = RepSOC
	fuelgauge_i2c_write(0x0D, tempVal);

	//6) AVSOC (0Eh) = RepSOC; 
	fuelgauge_i2c_write(0x0E, tempVal);	

	sec_bci->battery.low_batt_comp_flag = 1;  // Set flag
	
}


static int fuelgauge_alert_init(void)
{
	u16 read_data = 0;

	/* ======= Use RepSOC ======= */ 
	read_data = i2c_read(MISCCFG_REG);
	printk("%s, MISCCFG : %x\n", __func__, read_data);

	read_data = read_data & ~(0x03);
	fuelgauge_i2c_write(MISCCFG_REG, read_data);

	/* ======= SALRT Threshold setting ======= */ 
	fuelgauge_i2c_write(SALRT_THRESHOLD_REG, 0xFF01);
	read_data = i2c_read(SALRT_THRESHOLD_REG);
	if(read_data != 0xFF01){
		printk("%s, SALRT_THRESHOLD_REG is not valid (0x%x)\n", __func__, read_data);
		return -1;
	}

	/* ======= Reset VALRT Threshold setting (disable) ======= */
	fuelgauge_i2c_write(VALRT_THRESHOLD_REG, 0xFF00);
	read_data = i2c_read(VALRT_THRESHOLD_REG);
	if(read_data != 0xFF00){
		printk(KERN_ERR "%s : VALRT_THRESHOLD_REG is not valid (0x%x)\n", __func__, read_data);
	}

	/* ======= Reset TALRT Threshold setting (disable) ======= */
	fuelgauge_i2c_write(TALRT_THRESHOLD_REG, 0x7F80);
	read_data = i2c_read(TALRT_THRESHOLD_REG);
	if(read_data != 0x7F80){
		printk(KERN_ERR "%s : TALRT_THRESHOLD_REG is not valid (0x%x)\n", __func__, read_data);
	}
	mdelay(100);
	
	/* ======= Enable SOC alerts ======= */
	read_data = i2c_read(CONFIG_REG);
	read_data = read_data | (0x1 << 2);
	fuelgauge_i2c_write(CONFIG_REG, read_data);

	read_data = i2c_read(CONFIG_REG);
	printk("%s, CONFIG_REG (after) : %x\n", __func__, read_data); // 2214
		
	return 1;
}

int fuelgauge_check_status_reg(void)
{
	u8 status_data[2];
	int ret = 0;
	u16 read_data;

	// 1. Check Smn was generatedread
	read_data = i2c_read(STATUS_REG);
	status_data[1] = (read_data >> 8);
	status_data[0] = read_data & 0xFF;
	printk("%s - addr(0x00), data(0x%04x)\n", __func__, (status_data[1]<<8) | status_data[0]);

	if(status_data[1] & (0x1 << 2))
		ret = 1;

	// 2. clear Status reg
	fuelgauge_i2c_write(STATUS_REG, (read_data & 0x00FF)); // status_data[1] = 0;

	return ret;
}

static int fuelgauge_check_battery_present(void)
{
	u8 status_data[2];
	int ret = 1;
	u16 read_data;
	
	// 1. Check Bst bit
	read_data = i2c_read(STATUS_REG);
	status_data[1] = (read_data >> 8);
	status_data[0] = read_data & 0xFF;
	
	if(status_data[0] & (0x1 << 3))
	{
		printk("%s - addr(0x01), data(0x%04x)\n", __func__, (status_data[1]<<8) | status_data[0]);
		printk("%s : battery is absent!!\n", __func__);
		ret = 0;
	}

	return ret;
}

#ifdef FUEL_DEBUG_MODE
static void fg_monitor_work_handler( struct work_struct *work )
{
	printk("fg_monitor_work_handler\n");

	printk("TEMPERATURE : %d\n", fuelgauge_read_batt_temp());
	fuelgauge_check_vf_fullcap_range();
	get_fuelgauge_ptg_value(false);
	fuelgauge_test_print();
	get_fuelgauge_adc_value(false);
	printk("read current : CURRENT(%dmA), AVG_CURRENT(%dmA)\n", 
		get_fuelgauge_current(false, false), get_fuelgauge_current(false, true));
	
	schedule_delayed_work( &fg_monitor_work, 5*HZ );
}
#endif

static int fuelgauge_probe( struct i2c_client *client, 
                            const struct i2c_device_id *id )
{
    int ret = 0;
    unsigned char buf[3];
	unsigned long flag = 0;

    printk( "[FG] Fuelgauge Probe.\n" );

    sec_bci = get_sec_bci();

    if( strcmp( client->name, DRIVER_NAME) != 0 )
    {
        ret = -1;
        printk( "[FG] device not supported.\n" );
    }

    #if defined(CONFIG_USE_GPIO_I2C)
    fuelgauge_i2c_dummy_client = client;
    #else
    fuelgauge_i2c_client = client;
    #endif

	if(fuelgauge_i2c_client == NULL)
	{
		printk("fuelgauge_probe fail : fuelgauge_i2c_client is null\n");
		return 0;
	}


	set_fuelgauge_battery_type();

	// Init parameters to prevent wrong compensation.
	{
		u16 version_info;
		version_info = i2c_read(VERSION_REG); // Version register
		printk("fuelgauge_probe version : %x\n", version_info);
	}
	PrevFullCap = i2c_read(FULLCAP_REG);
	PrevVfFullCap = i2c_read(FullCAP_NOM_REG);
	FirstFullChargedCAP = PrevFullCap;  // Init FullCAP of first full charging.

	prevVfSOC = fuelgauge_read_vfsoc();
	prevRepSOC = get_fuelgauge_ptg_value(false); //fg_read_soc();
	prevRemCap = i2c_read(REMCAP_REP_REG);
	prevMixCap = i2c_read(REMCAP_MIX_REG);
	prevVfOCV = i2c_read(VFOCV_REG);
	prevFullCapacity = PrevFullCap;
	prevVFCapacity = PrevVfFullCap;

	spin_lock_init(&fg_lock);

	spin_lock_irqsave(&fg_lock, flag);
	fuelgauge_read_model_data();
	spin_unlock_irqrestore(&fg_lock, flag);

    if ( client->irq )
    {
        INIT_DELAYED_WORK( &fuelgauge_work, fuelgauge_work_handler );

		if(fuelgauge_alert_init() <= 0)
			printk("%s, fuelgauge init failed \n", __func__);

        ret = irq_to_gpio( client->irq );
        printk( "[FG] FUEL_INT_GPIO : %d \n", ret );

        set_irq_type( client->irq, IRQ_TYPE_EDGE_FALLING );
        ret = request_irq( client->irq, low_battery_isr, IRQF_DISABLED, client->name, NULL );
        if ( ret )
        {
            printk( "[FG] could not request irq %d, status %d\n", client->irq, ret );
        }
    }
#ifdef FUEL_DEBUG_MODE
	INIT_DELAYED_WORK( &fg_monitor_work, fg_monitor_work_handler );
	schedule_delayed_work( &fg_monitor_work, 5*HZ );
	wake_lock_init( &sec_fg_wakelock, WAKE_LOCK_SUSPEND, "samsung-fuelgauge" );
	wake_lock( &sec_fg_wakelock );
#endif	
	INIT_DELAYED_WORK( &low_batt_compensation_work, fg_low_batt_compensation_work_handler );

	sec_bci->charger.fuelgauge_full_soc = 95;  // for adjust fuelgauge

    return ret;
}

void fuelgauge_fullcharged_compensation(u32 is_recharging, u32 pre_update)
{
	static u16 NewFullCap_data = 0;
	
	printk("%s : is_recharging(%d), pre_update(%d)\n", __func__, is_recharging, pre_update);

	NewFullCap_data = i2c_read(FULLCAP_REG);

	if(NewFullCap_data > (Capacity * 110 / 100))
	{
		printk("%s : [Case 1] PrevFullCap = 0x%04x, NewFullCap = 0x%04x\n",
			__func__, PrevFullCap, NewFullCap_data);

		NewFullCap_data = (Capacity * 110) / 100;

//		fg_write_register(REMCAP_MIX_REG, (u16)(NewFullCap_data));
		fuelgauge_i2c_write(REMCAP_REP_REG, (u16)(NewFullCap_data));
		fuelgauge_i2c_write(FULLCAP_REG, (u16)(NewFullCap_data));
	}
	else if(NewFullCap_data < (Capacity * 70 / 100))
	{
		printk("%s : [Case 5] PrevFullCap = 0x%04x, NewFullCap = 0x%04x\n",
			__func__, PrevFullCap, NewFullCap_data);

		NewFullCap_data = (Capacity * 70) / 100;

//		fg_write_register(REMCAP_MIX_REG, (u16)(NewFullCap_data));
		fuelgauge_i2c_write(REMCAP_REP_REG, (u16)(NewFullCap_data));
		fuelgauge_i2c_write(FULLCAP_REG, (u16)(NewFullCap_data));
	}
	else
	{
		if(NewFullCap_data > (PrevFullCap * 105 / 100))
		{
			printk("%s : [Case 2] PrevFullCap = 0x%04x, NewFullCap = 0x%04x\n",
				__func__, PrevFullCap, NewFullCap_data);

			NewFullCap_data = (PrevFullCap * 105) / 100;

//			fg_write_register(REMCAP_MIX_REG, (u16)(NewFullCap_data));
			fuelgauge_i2c_write(REMCAP_REP_REG, (u16)(NewFullCap_data));
			fuelgauge_i2c_write(FULLCAP_REG, (u16)(NewFullCap_data));
		}
		else if(NewFullCap_data < (PrevFullCap * 90 / 100))
		{
			printk("%s : [Case 3] PrevFullCap = 0x%04x, NewFullCap = 0x%04x\n",
				__func__, PrevFullCap, NewFullCap_data);
		
			NewFullCap_data = (PrevFullCap * 90) / 100;
		
//			fg_write_register(REMCAP_MIX_REG, (u16)(NewFullCap_data));
			fuelgauge_i2c_write(REMCAP_REP_REG, (u16)(NewFullCap_data));
			fuelgauge_i2c_write(FULLCAP_REG, (u16)(NewFullCap_data));
		}
		else
		{
			printk("%s : [Case 4] PrevFullCap = 0x%04x, NewFullCap = 0x%04x\n",
				__func__, PrevFullCap, NewFullCap_data);

			// Do nothing...
		}
	}

	// In case of recharging, re-write FirstFullChargedCAP to FullCAP, RemCAP_REP.
	if(!is_recharging)
		FirstFullChargedCAP = NewFullCap_data;
	else {
		printk("%s : [Case 6] FirstFullCap = 0x%04x, NewFullCap = 0x%04x\n",
			__func__, FirstFullChargedCAP, NewFullCap_data);

		fuelgauge_i2c_write(REMCAP_REP_REG, (u16)(FirstFullChargedCAP));
		fuelgauge_i2c_write(FULLCAP_REG, (u16)(FirstFullChargedCAP));
	}

	//4. Write RepSOC(06h)=100%;
	fuelgauge_i2c_write(SOCREP_REG, (u16)(0x64 << 8));

	//5. Write MixSOC(0Dh)=100%;
	fuelgauge_i2c_write(SOCMIX_REG, (u16)(0x64 << 8));

	//6. Write AVSOC(0Eh)=100%;
	fuelgauge_i2c_write(SOCAV_REG, (u16)(0x64 << 8));

	if(!pre_update)  // if pre_update case, skip updating PrevFullCAP value.
		PrevFullCap = i2c_read(FULLCAP_REG);

	printk("%s : (A) FullCap = 0x%04x, RemCap = 0x%04x\n", __func__,
		i2c_read(FULLCAP_REG), i2c_read(REMCAP_REP_REG));

	fuelgauge_periodic_read();

}


void fuelgauge_check_vf_fullcap_range(void)
{
	static u16 NewVfFullCap = 0;
	u16 print_flag = 1;
	
	NewVfFullCap = i2c_read(FullCAP_NOM_REG);

	if(NewVfFullCap > (VFCapacity * 110 / 100))
	{
		printk("%s : [Case 1] PrevVfFullCap = 0x%04x, NewVfFullCap = 0x%04x\n",
			__func__, PrevVfFullCap, NewVfFullCap);

		NewVfFullCap = (VFCapacity * 110) / 100;

		fuelgauge_i2c_write(dQacc_REG, (u16)(NewVfFullCap / 4));
		fuelgauge_i2c_write(dPacc_REG, (u16)0x3200);
	}
	else if(NewVfFullCap < (VFCapacity * 70 / 100))
	{
		printk("%s : [Case 5] PrevVfFullCap = 0x%04x, NewVfFullCap = 0x%04x\n",
			__func__, PrevVfFullCap, NewVfFullCap);

		NewVfFullCap = (VFCapacity * 70) / 100;

		fuelgauge_i2c_write(dQacc_REG, (u16)(NewVfFullCap / 4));
		fuelgauge_i2c_write(dPacc_REG, (u16)0x3200);
	}
	else
	{
		if(NewVfFullCap > (PrevVfFullCap * 105 / 100))
		{
			printk("%s : [Case 2] PrevVfFullCap = 0x%04x, NewVfFullCap = 0x%04x\n",
				__func__, PrevVfFullCap, NewVfFullCap);

			NewVfFullCap = (PrevVfFullCap * 105) / 100;

			fuelgauge_i2c_write(dQacc_REG, (u16)(NewVfFullCap / 4));
			fuelgauge_i2c_write(dPacc_REG, (u16)0x3200);
		}
		else if(NewVfFullCap < (PrevVfFullCap * 90 / 100))
		{
			printk("%s : [Case 3] PrevVfFullCap = 0x%04x, NewVfFullCap = 0x%04x\n",
				__func__, PrevVfFullCap, NewVfFullCap);
		
			NewVfFullCap = (PrevVfFullCap * 90) / 100;
		
			fuelgauge_i2c_write(dQacc_REG, (u16)(NewVfFullCap / 4));
			fuelgauge_i2c_write(dPacc_REG, (u16)0x3200);
		}
		else
		{
			printk("%s : [Case 4] PrevVfFullCap = 0x%04x, NewVfFullCap = 0x%04x\n",
				__func__, PrevVfFullCap, NewVfFullCap);

			// Do nothing...
			print_flag = 0;
		}
	}

	PrevVfFullCap = i2c_read(FullCAP_NOM_REG);

	if(print_flag)
		printk("%s : VfFullCap(0x%04x), dQacc(0x%04x), dPacc(0x%04x)\n", __func__,
			i2c_read(FullCAP_NOM_REG), i2c_read(dQacc_REG), i2c_read(dPacc_REG));

}

int fuelgauge_check_cap_corruption(void)
{
	u32 VfSOC = fuelgauge_read_vfsoc();
	u32 RepSOC = get_fuelgauge_ptg_value(false);
	u32 MixCap = i2c_read(REMCAP_MIX_REG);
	u32 VfOCV = i2c_read(VFOCV_REG);
	u32 RemCap = i2c_read(REMCAP_REP_REG);
	u32 FullCapacity= i2c_read(FULLCAP_REG);
	u32 VfFullCapacity = i2c_read(FullCAP_NOM_REG);
	u32 temp, temp2, newVfOCV, pr_vfocv;
	unsigned long flag = 0;
	int ret = 0;

	// If usgin Jig, then skip checking.
	if(get_jig_status()) {
		printk("%s : Return by Using Jig(%d)\n", __func__, get_jig_status());
		return 0;  // it's ok
	}

	// Check full charge learning case.
	if( ((VfSOC >= 70) && ((RemCap >= (FullCapacity * 995 / 1000)) && (RemCap <= (FullCapacity * 1005 / 1000))))
		|| sec_bci->battery.low_batt_comp_flag || soc_restart_flag)
	{
		printk("%s : RemCap(%d), FullCap(%d), SOC(%d), low_batt_comp_flag(%d), soc_restart_flag(%d)\n",
			__func__, (RemCap/2), (FullCapacity/2), RepSOC, sec_bci->battery.low_batt_comp_flag, soc_restart_flag);
		prevRepSOC = RepSOC;
		prevRemCap = RemCap;
		prevFullCapacity= FullCapacity;
		if(soc_restart_flag)  // reset flag
			soc_restart_flag = 0;

		ret = 1;   // recover case
	}

	// ocv calculation for print
	temp = (VfOCV & 0xFFF) * 78125;
	pr_vfocv = temp / 1000000;
	
	temp = ((VfOCV & 0xF000) >> 4) * 78125;
	temp2 = temp / 1000000;
	pr_vfocv += (temp2 << 4);

	printk("%s : VfSOC(%d), RepSOC(%d), MixCap(%d), VfOCV(0x%04x, %d)\n",
		__func__, VfSOC, RepSOC, (MixCap/2), VfOCV, pr_vfocv);

	if( ( ((VfSOC+5) < prevVfSOC) || (VfSOC > (prevVfSOC+5)) )
		|| ( ((RepSOC+5) < prevRepSOC) || (RepSOC > (prevRepSOC+5)) )
		|| ( ((MixCap+530) < prevMixCap) || (MixCap > (prevMixCap+530)) ) )  // MixCap differ is greater than 265mAh
	{
		fuelgauge_periodic_read();

		printk("[FG_Recovery] (B) VfSOC(%d), prevVfSOC(%d), RepSOC(%d), prevRepSOC(%d), MixCap(%d), prevMixCap(%d)\n",
			VfSOC, prevVfSOC, RepSOC, prevRepSOC, (MixCap/2), (prevMixCap/2));

		spin_lock_irqsave(&fg_lock, flag);

		fuelgauge_i2c_write_and_verify(REMCAP_MIX_REG , prevMixCap);  // MixCap
		fuelgauge_i2c_write(VFOCV_REG, prevVfOCV);
		mdelay(200);

		fuelgauge_i2c_write_and_verify(REMCAP_REP_REG, prevRemCap);  // RemCAP
		VfSOC = i2c_read(VFSOC_REG);  // read VFSOC
		fuelgauge_i2c_write(0x60, 0x0080);	 // Enable Write Access to VFSOC0
		fuelgauge_i2c_write_and_verify(0x48, VfSOC);  // VFSOC0
		fuelgauge_i2c_write(0x60, 0x0000); 	 // Disable Write Access to VFSOC0

		fuelgauge_i2c_write_and_verify(0x45, (prevVFCapacity / 4));  // dQ_acc
		fuelgauge_i2c_write_and_verify(0x46, 0x3200);  // dP_acc
		fuelgauge_i2c_write_and_verify(FULLCAP_REG, prevFullCapacity);
		fuelgauge_i2c_write_and_verify(FullCAP_NOM_REG, prevVFCapacity);  // FullCAPNom

		spin_unlock_irqrestore(&fg_lock, flag);

		msleep(200);

//		prevVfSOC = fg_read_vfsoc();
//		prevRepSOC = fg_read_soc();
//		prevRemCap = fg_read_register(REMCAP_REP_REG);
//		prevMixCap = fg_read_register(REMCAP_MIX_REG);
//		prevFullCapacity= fg_read_register(FULLCAP_REG);
//		prevVFCapacity = fg_read_register(FullCAP_NOM_REG);
//		prevVfOCV = fg_read_register(VFOCV_REG);

		// ocv calculation for print
		newVfOCV = i2c_read(VFOCV_REG);
		temp = (newVfOCV & 0xFFF) * 78125;
		pr_vfocv = temp / 1000000;

		temp = ((newVfOCV & 0xF000) >> 4) * 78125;
		temp2 = temp / 1000000;
		pr_vfocv += (temp2 << 4);

		printk("[FG_Recovery] (A) newVfSOC(%d), newRepSOC(%d), newMixCap(%d), newVfOCV(0x%04x, %d)\n",
			fuelgauge_read_vfsoc(), get_fuelgauge_ptg_value(false), (i2c_read(REMCAP_MIX_REG)/2), newVfOCV, pr_vfocv);
		
		fuelgauge_periodic_read();

		ret = 1;
	}
	else {
		prevVfSOC = VfSOC;
		prevRepSOC = RepSOC;
		prevRemCap = RemCap;
		prevMixCap = MixCap;
		prevFullCapacity= FullCapacity;
		prevVFCapacity = VfFullCapacity;
		prevVfOCV = VfOCV;
	}

	return ret;
}

static int fuelgauge_remove( struct i2c_client * client )
{
    return 0;
}

static void fuelgauge_shutdown( struct i2c_client * client )
{
    return;
}

static int fuelgauge_suspend( struct i2c_client * client , pm_message_t mesg )
{

#if 0

    int batt_ptg = 0;
    int ret = 0;

    unsigned char alert_th;
    unsigned char buf[3];

    if(!sec_bci->charger.is_charging)
    {
        // When chargine battery, set alert threshold by using remained battery percentage.
        batt_ptg = get_fuelgauge_ptg_value(false);

        if(batt_ptg >= 15)
            alert_th = 0x11; // 15%
        else if(batt_ptg >= 4)
            alert_th = 0x1C; // 4%
        else
            alert_th = 0x1F; // 1%
    }

    ret = i2c_read( REG_RCOMP );
    buf[0] = REG_RCOMP;
    buf[1] = ret >> 8;
    buf[2] = alert_th;
    i2c_write( buf, 3 );
    printk("%s, set alert threshold 0x%2x\n", __func__, alert_th);
#endif
    return 0;
}

static int fuelgauge_resume( struct i2c_client * client )
{
#if 0

    int ret = 0;
    unsigned char buf[3];

    ret = i2c_read( REG_RCOMP );
    buf[0] = REG_RCOMP;
    buf[1] = ret >> 8;
    buf[2] = 0x1F;
    i2c_write( buf, 3 );
    printk("%s, set alert threshold 1%\n", __func__);
#endif
    return 0;
}

int fuelgauge_init( void )
{
    int ret;

#if defined(CONFIG_USE_GPIO_I2C)
    fuelgauge_i2c_client = omap_gpio_i2c_init(OMAP_GPIO_FUEL_SDA,OMAP_GPIO_FUEL_SCL, MAX17042_SLAVE_ADDR, 100);

	printk("[FG] Fuelgauge Init. add i2c driver!\n");
    if(fuelgauge_i2c_client == NULL)
    {
        printk(KERN_ERR "[FG] omap_gpio_i2c_init failed!\n");
    }
	
	printk("[FG] Fuelgauge Init. add dummy i2c driver!\n");
    if( ( ret = i2c_add_driver( &fuelgauge_i2c_driver ) < 0 ) )
    {
        printk( KERN_ERR "[FG] i2c_add_driver failed.\n" );
    }
#else
    printk("[FG] Fuelgauge Init. add i2c driver!\n");
    if( ( ret = i2c_add_driver( &fuelgauge_i2c_driver ) < 0 ) )
    {
        printk( KERN_ERR "[FG] i2c_add_driver failed.\n" );
    }
#endif
    return ret;    
}

void fuelgauge_exit( void )
{
    printk("[FG] Fuelgauge Exit.\n");

#if defined(CONFIG_USE_GPIO_I2C)
    omap_gpio_i2c_deinit(fuelgauge_i2c_client);
#endif
    i2c_del_driver( &fuelgauge_i2c_driver );
}
