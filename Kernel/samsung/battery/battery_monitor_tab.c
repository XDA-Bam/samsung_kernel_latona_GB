/*
 * module/samsung_battery/battery_monitor.c
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c/twl4030-madc.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <plat/omap3-gptimer12.h>
#include "common.h"

// To access sysfs file system [+]
#include <linux/uaccess.h>
#include <linux/fs.h>
// To access sysfs file system [-]

#include <linux/gpio.h>
#include <plat/mux.h>

#define DRIVER_NAME "secBattMonitor"

#define TEMP_DEG    0
#define TEMP_ADC    1

/* In module TWL4030_MODULE_PM_RECEIVER */
#define VSEL_VINTANA2_2V75  0x01
#define CARKIT_ANA_CTRL     0xBB
#define SEL_MADC_MCPC       0x08
//#define _OMS_FEATURES_ // CHINA BORQS CONCEPTS

static DEFINE_MUTEX( battery_lock );

static int NCP15WB473_batt_table[] = 
{
    /* -15C ~ 85C */

    /*0[-15] ~  14[-1]*/
    360850,342567,322720,304162,287000,
    271697,255331,241075,227712,215182,
    206463,192394,182034,172302,163156, 
    /*15[0] ~ 34[19]*/
    158214,146469,138859,131694,124947,
    122259,118590,112599,106949,101621,
    95227, 91845, 87363, 83128, 79126,
    74730, 71764, 68379, 65175, 62141,
    /*35[20] ~ 74[59]*/    
    59065, 56546, 53966, 51520, 49201,
    47000, 44912, 42929, 41046, 39257,
    37643, 35942, 34406, 32945, 31555,
    30334, 28972, 27773, 26630, 25542,
    24591, 23515, 22571, 21672, 20813,
    20048, 19211, 18463, 17750, 17068,
    16433, 15793, 15197, 14627, 14082,
    13539, 13060, 12582, 12124, 11685,
    /*75[60] ~ 100[85]*/
    11209, 10862, 10476, 10106,  9751,
    9328,  9083,  8770,  8469,  8180,
    7798,  7635,  7379,  7133,  6896,
    6544,  6450,  6240,  6038,  5843,
    5518,  5475,  5302,  5134,  4973,
    4674
};

struct battery_device_config
// THIS CONFIG IS SET IN BOARD_FILE.(platform_data)
{
    /* SUPPORT MONITORING CHARGE CURRENT FOR CHECKING FULL */
    int MONITORING_CHG_CURRENT;
    int CHG_CURRENT_ADC_PORT;

    /* SUPPORT MONITORING TEMPERATURE OF THE SYSTEM FOR BLOCKING CHARGE */
    int MONITORING_SYSTEM_TEMP;
    int TEMP_ADC_PORT;
};

struct battery_device_info 
{
    struct device *dev;
    struct delayed_work battery_monitor_work;

    // LDO USB1V5, USB1V9 have a same unique operating mode.
    struct regulator *usb1v5;
    struct regulator *usb1v8;
    // LDO USB3V1 have a unique operating mode.
    struct regulator *usb3v1;

    struct power_supply sec_battery;
    struct power_supply sec_ac;
    struct power_supply sec_usb;    
};

static struct device *this_dev;
static struct wake_lock sec_bc_wakelock;

static SEC_battery_charger_info sec_bci;
static struct battery_device_config *device_config;

static struct gptimer12_timer batt_gptimer_12;

static char *samsung_bci_supplied_to[] = {
    "battery",
};


SEC_battery_charger_info *get_sec_bci( void )
{
    return &sec_bci;
}

// Prototype
       int _charger_state_change_( int , int, bool );
       int _low_battery_alarm_( void );
       int _get_average_value_( int *, int );
       int _get_t2adc_data_( int );
	   int get_jig_status();
static int turn_resources_off_for_adc( void );
static int turn_resources_on_for_adc( void );
static int get_elapsed_time_secs( unsigned long long * );
static int t2adc_to_temperature( int , int );
static int do_fuelgauge_reset_soc( void );
static int do_fuelgauge_reset_cap( int sel );

static int get_battery_level_adc( void );
static int get_battery_level_ptg( void );
static int get_system_temperature( bool );
static int get_charging_current_adc_val( void );
static int check_full_charge_using_chg_current( int );
static void get_system_status_in_sleep( int *, int *, int *, int * );
static int battery_monitor_core( bool );
static void battery_monitor_work_handler( struct work_struct * );
static int battery_monitor_fleeting_wakeup_handler( unsigned long ); 
static int samsung_battery_get_property( struct power_supply *, enum power_supply_property , union power_supply_propval * );
static int samsung_ac_get_property( struct power_supply *, enum power_supply_property, union power_supply_propval * );
static int samsung_usb_get_property( struct power_supply *, enum power_supply_property, union power_supply_propval * );
static void samsung_pwr_external_power_changed( struct power_supply * );
static ssize_t store_event(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t size);
static ssize_t store_batt_sysfs(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t size);


static int __devinit battery_probe( struct platform_device * );
static int __devexit battery_remove( struct platform_device * );
static int battery_suspend( struct platform_device *, pm_message_t );
static int battery_resume( struct platform_device * );
static int __init battery_init( void );
static void __exit battery_exit( void );

// Charger
extern int charger_init( void );
extern void charger_exit( void );
extern int _battery_state_change_( int category, int value, bool is_sleep );
extern int _check_full_charge_dur_sleep_( void );
extern int _cable_status_now_( void );

// Fuel Guage
extern int fuelgauge_init( void );
extern void fuelgauge_exit( void );
extern int fuelgauge_reset_soc(void);
extern int fuelgauge_reset_capacity(int sel);

extern int get_fuelgauge_adc_value( bool is_sleep );
extern int get_fuelgauge_ptg_value( bool is_sleep );
extern s32 fuelgauge_read_batt_temp(void);
extern int get_battery_type(void);
extern int get_fuelgauge_current( bool is_sleep, bool avg );
extern int do_low_batt_compensation(int fg_soc,int fg_vcell, int fg_current);
extern int fuelgauge_check_cap_corruption(void);
extern void fuelgauge_check_vf_fullcap_range(void);

#if 0
extern int update_rcomp_by_temperature(int temp);
#endif

// Sleep i2c, ADC
extern void twl4030_i2c_init( void );
extern void twl4030_i2c_disinit( void );
extern void normal_i2c_init( void );
extern void normal_i2c_disinit( void );
extern s32 t2_adc_data( u8 channel );

extern unsigned long long sched_clock( void );

static bool boot_complete = false;
static int boot_monitor_count = 0;
static bool battery_suspend_state = false;

int stop_temperature_overheat = CHARGE_STOP_TEMPERATURE_MAX;
int recover_temperature_overheat = CHARGE_RECOVER_TEMPERATURE_MAX;

#ifdef CONFIG_SEC_BATTERY_USE_RECOVERY_MODE
static int recovery_mode = 0;
module_param(recovery_mode, bool, 0);
#endif  /* CONFIG_SEC_BATTER_USE_RECOVERY_MODE */

// ------------------------------------------------------------------------- // 
//                           sysfs interface                                 //
// ------------------------------------------------------------------------- // 
#define  __ATTR_SHOW_CALLBACK( _name, _ret_val ) \
static ssize_t _name( struct kobject *kobj, \
              struct kobj_attribute *attr, \
              char *buf ) \
{ \
    return sprintf ( buf, "%d\n", _ret_val ); \
} 

static int get_batt_monitor_temp( void )
{
    return sec_bci.battery.support_monitor_temp;
}

static ssize_t store_batt_monitor_temp(struct kobject *kobj,
                    struct kobj_attribute *attr,
                    const char *buf, size_t size)
{
    int flag;
    
    sscanf( buf, "%d", &flag );

    printk("[BM] change value %d\n",flag);

    sec_bci.battery.support_monitor_temp = flag;
    sec_bci.battery.support_monitor_timeout = flag;
    sec_bci.battery.support_monitor_full = flag;

    return size;
}

static ssize_t store_batt_boot_complete(struct kobject *kobj,
                    struct kobj_attribute *attr,
                    const char *buf, size_t size)
{
    int flag;

    sscanf( buf, "%d", &flag );
    printk("[BM] boot complete flag:%d, buf:%s, size:%d\n",flag, buf, size);

    boot_complete = true;

    return size;
}

static ssize_t show_batt_type( struct kobject *kobj, 
              struct kobj_attribute *attr, 
              char *buf ) 
{ 
	if(get_battery_type() == 1)
	    return sprintf( buf, "SDI_SDI");
	else if(get_battery_type() == 2)
		return sprintf( buf, "ATL_ATL");
	else
		return sprintf( buf, "Unknown");
} 

__ATTR_SHOW_CALLBACK( show_batt_vol, get_battery_level_adc() )
__ATTR_SHOW_CALLBACK( show_batt_vol_aver, get_battery_level_adc() )	
__ATTR_SHOW_CALLBACK( show_batt_vol_adc, 0 )
__ATTR_SHOW_CALLBACK( show_batt_vol_adc_cal, 0 )
__ATTR_SHOW_CALLBACK( show_batt_vol_adc_aver, 0 )
__ATTR_SHOW_CALLBACK( show_batt_temp, fuelgauge_read_batt_temp()/100 )
__ATTR_SHOW_CALLBACK( show_batt_temp_adc, 0 )  //get_system_temperature( TEMP_ADC )
__ATTR_SHOW_CALLBACK( show_batt_temp_adc_aver, 0 )
__ATTR_SHOW_CALLBACK( show_batt_temp_adc_cal, 0 )
__ATTR_SHOW_CALLBACK( show_batt_v_f_adc, 0 )
__ATTR_SHOW_CALLBACK( show_batt_test_mode, 0 )
__ATTR_SHOW_CALLBACK( show_batt_capacity, get_battery_level_ptg() )
__ATTR_SHOW_CALLBACK( show_batt_fuelgauge_reset_soc, 0 )
__ATTR_SHOW_CALLBACK( show_batt_monitor_temp, get_batt_monitor_temp() )
__ATTR_SHOW_CALLBACK( show_batt_temp_check, sec_bci.battery.battery_health)
__ATTR_SHOW_CALLBACK( show_batt_full_check, (sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_FULL)?1:0)
__ATTR_SHOW_CALLBACK( show_charging_source, sec_bci.charger.cable_status )

#ifdef _OMS_FEATURES_
__ATTR_SHOW_CALLBACK( show_batt_vol_toolow, sec_bci.battery.battery_vol_toolow )
static struct kobj_attribute batt_vol_toolow =
    __ATTR( batt_vol_toolow, 0644, show_batt_vol_toolow, NULL );
#endif

static struct kobj_attribute batt_sysfs_testmode[] = {
	/* Event logging - Put these attributes at first position of this array 
       For using the call back function 'store_event'
	*/
    __ATTR( mp3, 0666, NULL, store_event ), 				// offset = 0
    __ATTR( talk_wcdma, 0666, NULL, store_event ), 			// offset = 1
    __ATTR( talk_gsm, 0666, NULL, store_event ), 			// offset = 2
    __ATTR( data_call, 0666, NULL, store_event ), 			// offset = 3
    __ATTR( vt_call, 0666, NULL, store_event ), 			// offset = 4
    __ATTR( camera_preview, 0666, NULL, store_event ), 		// offset = 5	
    __ATTR( camera_recording, 0666, NULL, store_event ),	// offset = 6
    __ATTR( video, 0666, NULL, store_event ), 				// offset = 7
    __ATTR( g_map, 0666, NULL, store_event ), 				// offset = 8
    __ATTR( e_book, 0666, NULL, store_event ), 				// offset = 9
    __ATTR( bt_call, 0666, NULL, store_event ), 			// offset = 10
    __ATTR( wap_browsing, 0666, NULL, store_event ), 		// offset = 11
    __ATTR( wifi_browsing, 0666, NULL, store_event ), 		// offset = 12
    __ATTR( game, 0666, NULL, store_event ), 				// offset = 13
    /* END of Event logging */

    __ATTR( batt_vol, 0666, show_batt_vol, NULL ),							// offset = 14
    __ATTR( batt_vol_adc, 0666, show_batt_vol_adc, NULL ),					// offset = 15
	__ATTR( batt_vol_aver, 0666, show_batt_vol_aver, NULL ),				// offset = 16
	__ATTR( batt_vol_adc_cal, 0666, show_batt_vol_adc_cal, NULL ),			// offset = 17
	__ATTR( batt_vol_adc_aver, 0666, show_batt_vol_adc_aver, NULL ),		// offset = 18
    __ATTR( batt_temp, 0666, show_batt_temp, NULL ),						// offset = 19
	__ATTR( batt_temp_aver, 0666, show_batt_temp, NULL ),					// offset = 20
    __ATTR( batt_temp_adc, 0666, show_batt_temp_adc, NULL ),				// offset = 21
	__ATTR( batt_temp_adc_aver, 0666, show_batt_temp_adc_aver, NULL ),		// offset = 22
	__ATTR( batt_temp_adc_cal, 0666, show_batt_temp_adc_cal, NULL ),		// offset = 23
    __ATTR( batt_v_f_adc, 0666, show_batt_v_f_adc, NULL ),					// offset = 24
    __ATTR( batt_capacity, 0666, show_batt_capacity, NULL ),				// offset = 25
	__ATTR( reset_soc, 0666, show_batt_fuelgauge_reset_soc, store_batt_sysfs ), 	// offset = 26
	__ATTR( reset_cap, 0666, NULL, store_batt_sysfs ),								// offset = 27
    __ATTR( batt_monitor_temp, 0666, show_batt_monitor_temp, store_batt_monitor_temp ), // offset = 28
    __ATTR( batt_boot_complete, 0666, NULL, store_batt_boot_complete ),		// offset = 29
    __ATTR( fg_soc, 0666, show_batt_capacity, NULL ),						// offset = 30
    __ATTR( batt_temp_check, 0666, show_batt_temp_check, NULL ),			// offset = 31
    __ATTR( batt_full_check, 0666, show_batt_full_check, NULL ),    		// offset = 32
    __ATTR( charging_source, 0666, show_charging_source, NULL ), 			// offset = 33
	__ATTR( batt_type, 0666, show_batt_type, NULL ), 						// offset = 34
	__ATTR( batt_test_mode, 0666, show_batt_test_mode, store_batt_sysfs ),				// offset = 35
};

/* Event logging */
u32 event_logging = 0;

enum{
	MP3 = 0, TALK_WCDMA, TALK_GSM, DATA_CALL, VT_CALL, CAMERA_PREVIEW, CAMERA_RECORDING, 
	VIDEO, G_MAP, E_BOOK, BT_CALL, WAP_BROWSING, WIFI_BROWSING, GAME,
	RESET_SOC = 26, RESET_CAP = 27, BATT_TEST_MODE = 35, 
};
static ssize_t store_event(struct kobject *kobj,
                    struct kobj_attribute *attr,
                    const char *buf, size_t size)
{
    int flag;

	const ptrdiff_t off = attr - batt_sysfs_testmode;

    sscanf( buf, "%d", &flag );

	if(flag == 1)
		event_logging |= (0x1 << off);
	else if(flag == 0)
		event_logging &= ~(0x1 << off);
	
    printk("[BM] store_event offset=%d, value=%d\n", off, flag);
    return size;
}

static ssize_t store_batt_sysfs(struct kobject *kobj,
                    struct kobj_attribute *attr,
                    const char *buf, size_t size)
{
	const ptrdiff_t off = attr - batt_sysfs_testmode;
	int x;

	switch(off)
	{
		case BATT_TEST_MODE:
			if (sscanf(buf, "%d\n", &x) == 1)
			{
				printk("BATT_TEST_MODE 1\n");
			}else
			{
				printk("BATT_TEST_MODE 0\n");			
			}
			break;
		case RESET_SOC:
			printk("RESET_SOC\n");
			if (sscanf(buf, "%d\n", &x) == 1)
			{
				do_fuelgauge_reset_soc();
			}
			break;
		case RESET_CAP:
			printk("RESET_CAP\n");
			if (sscanf(buf, "%d\n", &x) == 1 || x==2 || x==3 || x==4)
			{
				if (x==1 || x== 2 || x==3 || x==4)
					do_fuelgauge_reset_cap(x);
			}
			break;
		default:
			break;
	}
	return size;
}


/* END of Event logging */

int _charger_state_change_( int category, int value, bool is_sleep )
{   
    printk( "[BM] cate: %d, value: %d\n", category, value );

    if( category == STATUS_CATEGORY_CABLE )
    {
        switch( value )
        {
            case POWER_SUPPLY_TYPE_BATTERY :
                /*Stop monitoring the batt. level for Re-charging*/
                sec_bci.battery.monitor_field_rechg_vol = false;

                /*Stop monitoring the temperature*/
                sec_bci.battery.monitor_field_temp = false;

                sec_bci.battery.confirm_full_by_current = 0;
                sec_bci.battery.confirm_recharge = 0;

                sec_bci.charger.charging_timeout = DEFAULT_CHARGING_TIMEOUT;

                sec_bci.charger.full_charge_dur_sleep = 0x0;
                break;

            case POWER_SUPPLY_TYPE_MAINS :
                sec_bci.charger.charging_timeout = DEFAULT_CHARGING_TIMEOUT;
                wake_lock_timeout( &sec_bc_wakelock , HZ );         
                break;

            case POWER_SUPPLY_TYPE_USB :            
                break;

            default :
                break;
        }

        goto Out_Charger_State_Change;
    }
    else if( category == STATUS_CATEGORY_CHARGING )
    {
        switch( value )
        {
            case POWER_SUPPLY_STATUS_UNKNOWN :
            case POWER_SUPPLY_STATUS_NOT_CHARGING :
                //sec_bci.charger.full_charge = false;
                
                /*Stop monitoring the batt. level for Re-charging*/
                sec_bci.battery.monitor_field_rechg_vol = false;

                if( sec_bci.battery.battery_health != POWER_SUPPLY_HEALTH_OVERHEAT 
                    && sec_bci.battery.battery_health != POWER_SUPPLY_HEALTH_COLD )
                {
                    /*Stop monitoring the temperature*/
                    sec_bci.battery.monitor_field_temp = false;
                }

                break;

            case POWER_SUPPLY_STATUS_DISCHARGING :
                break;

            case POWER_SUPPLY_STATUS_FULL :
                /*Start monitoring the batt. level for Re-charging*/
                sec_bci.battery.monitor_field_rechg_vol = true;

                /*Stop monitoring the temperature*/
                sec_bci.battery.monitor_field_temp = false;

                wake_lock_timeout( &sec_bc_wakelock , HZ );
                break;

            case POWER_SUPPLY_STATUS_CHARGING :
                /*Start monitoring the temperature*/
                sec_bci.battery.monitor_field_temp = true;

                /*Stop monitoring the batt. level for Re-charging*/
                sec_bci.battery.monitor_field_rechg_vol = false;

                break;

            case POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL :
                /*Start monitoring the temperature*/
                sec_bci.battery.monitor_field_temp = true;

                /*Stop monitoring the batt. level for Re-charging*/
                sec_bci.battery.monitor_field_rechg_vol = false;

				/*Not change the battery bar - keep battery full screen*/
				//goto Out_Charger_State_Change;
                break;

            case POWER_SUPPLY_STATUS_RECHARGING_FOR_TEMP :
                /*Start monitoring the temperature*/
                sec_bci.battery.monitor_field_temp = true;

                /*Stop monitoring the batt. level for Re-charging*/
                sec_bci.battery.monitor_field_rechg_vol = false;

                break;

            default :
                break;
        }

    }

    if( !is_sleep )
    {
        struct battery_device_info *di;
        struct platform_device *pdev;

        pdev = to_platform_device( this_dev );
        di = platform_get_drvdata( pdev );
        cancel_delayed_work( &di->battery_monitor_work );
        queue_delayed_work( sec_bci.sec_battery_workq, &di->battery_monitor_work, 5 * HZ ); 

        power_supply_changed( &di->sec_battery );
        power_supply_changed( &di->sec_ac );
        power_supply_changed( &di->sec_usb );
    }
    else
    {
        release_gptimer12( &batt_gptimer_12 );
        request_gptimer12( &batt_gptimer_12 );
    }

Out_Charger_State_Change :
    return 0;
}


int get_jig_status()
{
	if(gpio_get_value(OMAP_GPIO_JIG_ON18))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int _low_battery_alarm_()
{
    struct battery_device_info *di;
    struct platform_device *pdev;
    int level;

    pdev = to_platform_device( this_dev );
    di = platform_get_drvdata( pdev );

    level = get_battery_level_ptg();
    if ( level == 1 )
        sec_bci.battery.battery_level_ptg = 0;
    else 
        sec_bci.battery.battery_level_ptg = level;

    wake_lock_timeout( &sec_bc_wakelock , HZ );
    power_supply_changed( &di->sec_battery );

    return 0;
}

int _get_average_value_( int *data, int count )
{
    int average;
    int min = 0;
    int max = 0;
    int sum = 0;
    int i;

    if ( count >= 5 )
    {
        min = max = data[0];
        for( i = 0; i < count; i++ )
        {
            if( data[i] < min )
                min = data[i];

            if( data[i] > max )
                max = data[i];

            sum += data[i];
        }
        average = ( sum - max - min ) / ( count - 2 );
    }
    else
    {
        for( i = 0; i < count; i++ )
            sum += data[i];

        average = sum / count;
    }

    return average; 
}

int _get_t2adc_data_( int ch )
{
    int ret = 0;
    int val[5];
    int i;
    struct twl4030_madc_request req;

	if ( ch >= 1 && ch <= 7 ){
	    turn_resources_on_for_adc();
	    twl_i2c_write_u8( TWL4030_MODULE_USB, SEL_MADC_MCPC, CARKIT_ANA_CTRL );

	    msleep(100);
	}

    req.channels = ( 1 << ch );
    req.do_avg = 0;
    req.method = TWL4030_MADC_SW1;
    req.active = 0;
    req.func_cb = NULL;

    #if 0
    twl4030_madc_conversion( &req );
    ret = req.rbuf[ch];
    #else
    for ( i = 0; i < 5 ; i++ )
    {
        twl4030_madc_conversion( &req );
        val[i] = req.rbuf[ch];
    }

    ret = _get_average_value_( val, 5 );
    #endif

	if ( ch >= 1 && ch <= 7 ){
	    turn_resources_off_for_adc();
	}

    return ret;
}

int turn_resources_on_for_adc()
{
    int ret;
    u8 val = 0; 
    
    struct battery_device_info *di;
    struct platform_device *pdev;

    pdev = to_platform_device( this_dev );
    di = platform_get_drvdata( pdev );

    ret = twl_i2c_read_u8( TWL4030_MODULE_MADC, &val, TWL4030_MADC_CTRL1 );
    val &= ~TWL4030_MADC_MADCON;
    ret = twl_i2c_write_u8( TWL4030_MODULE_MADC, val, TWL4030_MADC_CTRL1 );
    msleep( 10 );

    ret = twl_i2c_read_u8( TWL4030_MODULE_MADC, &val, TWL4030_MADC_CTRL1 );
    val |= TWL4030_MADC_MADCON;
    ret = twl_i2c_write_u8( TWL4030_MODULE_MADC, val, TWL4030_MADC_CTRL1 );
    
    if(device_config->TEMP_ADC_PORT != 0)
    {
        ret = regulator_enable( di->usb3v1 );
        if ( ret )
            printk("[BM] Regulator 3v1 error!!\n");

        ret = regulator_enable( di->usb1v8 );
        if ( ret )
            printk("[BM] Regulator 1v8 error!!\n");

        ret = regulator_enable( di->usb1v5 );
        if ( ret )
            printk("[BM] Regulator 1v5 error!!\n");
    }    

    twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, 0x14, 0x7D/*VUSB_DEDICATED1*/ );
    twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, 0x0, 0x7E/*VUSB_DEDICATED2*/ );
    twl_i2c_read_u8( TWL4030_MODULE_USB, &val, 0xFE/*PHY_CLK_CTRL*/ );
    val |= 0x1;
    twl_i2c_write_u8( TWL4030_MODULE_USB, val, 0xFE/*PHY_CLK_CTRL*/ );

    twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, VSEL_VINTANA2_2V75, TWL4030_VINTANA2_DEDICATED );    
    twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, 0x20, TWL4030_VINTANA2_DEV_GRP );

    return 0;
}

int turn_resources_off_for_adc()
{
    u8 val = 0; 
    struct battery_device_info *di;
    struct platform_device *pdev;

    pdev = to_platform_device( this_dev );
    di = platform_get_drvdata( pdev );

    if ( sec_bci.charger.cable_status == POWER_SUPPLY_TYPE_BATTERY )
    {
        twl_i2c_read_u8( TWL4030_MODULE_USB, &val, 0xFE/*PHY_CLK_CTRL*/ );
        val &= 0xFE;
        twl_i2c_write_u8( TWL4030_MODULE_USB, val, 0xFE/*PHY_CLK_CTRL*/ );
    }

    
    if(device_config->TEMP_ADC_PORT != 0)
    {
        regulator_disable( di->usb1v5 );
        regulator_disable( di->usb1v8 );
        regulator_disable( di->usb3v1 );
    }
    return 0;
}

static int get_elapsed_time_secs( unsigned long long *start )
{
    unsigned long long now;
    unsigned long long diff;
    unsigned long long max = 0xFFFFFFFF;

    max = ( max << 32 ) | 0xFFFFFFFF;
    
    now = sched_clock();

    if ( now >= *start )
    {
        diff = now - *start;
    }
    else 
    {
   		sec_bci.charger.charge_start_time = now;
		diff = 0;
        //diff = max - *start + now;
    }

    do_div(diff, 1000000000L);
	/*
    printk( KERN_DEBUG "[BM] now: %llu, start: %llu, diff:%d\n",
        now, *start, (int)diff );
	*/
    return (int)diff;       
}

static int t2adc_to_temperature( int value, int channel )
{
    int mvolt, r;
    int temp;

    /*Caluclating voltage(adc) and resistance of thermistor */
    if( channel == 5 )
    {
        // vol = conv_result * step-size / R (TWLTRM Table 9-4)
        mvolt = value * 2500 / 1023; // mV
    }
    else if (channel == 0)
    {
        mvolt = value * 1500 / 1023;
    }
    else
    {
        mvolt = 0;
    }

	if(mvolt == 1500) // Prevent error - Division by zero in kernel.
		mvolt = 1499;

    //printk("[BM] TEMP. adc: %d, mVolt: %dmA\n", value , mvolt );
    // for ZEUS - VDD: 3000mV, Rt = ( 100K * Vadc  ) / ( VDD - 2 * Vadc )
    r = ( 100000 * mvolt ) / ( 3000 - 2 * mvolt );

    /*calculating temperature*/
    for( temp = 100; temp >= 0; temp-- )
    {
        if( ( NCP15WB473_batt_table[temp] - r ) >= 0 )
            break;
    }
    temp -= 15;
    temp=temp-2;
    
    return temp;
}

static int do_fuelgauge_reset_soc( void )
{
	int fg_current;
	fuelgauge_reset_soc();


    sec_bci.battery.battery_level_ptg = get_battery_level_ptg();
    msleep(10);
    sec_bci.battery.battery_level_vol= get_battery_level_adc();
	fg_current = get_fuelgauge_current(false, false);

	sec_bci.battery.battery_level_ptg = 
		do_low_batt_compensation(sec_bci.battery.battery_level_ptg, sec_bci.battery.battery_level_vol, fg_current);

    return 1;
}

static int do_fuelgauge_reset_cap( int sel)
{
	fuelgauge_reset_capacity(sel);
    return 1;
}

static int get_battery_level_adc( void )
{
    int value;
    value = get_fuelgauge_adc_value( CHARGE_DUR_ACTIVE );
    if(value < 0)
        value = sec_bci.battery.battery_level_vol;

#ifdef CONFIG_SAMSUNG_BATTERY_TESTMODE
    return 4100;
#else
    return value;
#endif

}

static int get_adjusted_battery_ptg(int value)
{
	if(value > 80)
		value = ((value*100-7600)*8/10+50)/70+75;

    if ( sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_FULL )
		value = 100;

	if(value > 100)
		value = 100;
	return value;
}


static int get_battery_level_ptg( void )
{
    int value;

    value = get_fuelgauge_ptg_value( CHARGE_DUR_ACTIVE );

	value = get_adjusted_battery_ptg(value);
/*
    if ( sec_bci.charger.is_charging && value >= 100)
        value = 99;
*/
    if(!boot_complete && value <= 0)
        value = 1;

#ifdef CONFIG_SAMSUNG_BATTERY_TESTMODE
    return 60;
#else
    return value;
#endif
}

static int get_system_temperature( bool flag )
{
    int adc;
    int temp;
    
    adc = _get_t2adc_data_( device_config->TEMP_ADC_PORT );

    if( flag )
        return adc;

    temp = t2adc_to_temperature( adc, device_config->TEMP_ADC_PORT );

    return temp;
}

static int get_charging_current_adc_val( void )
{
    int adc;
    
    adc = _get_t2adc_data_( device_config->CHG_CURRENT_ADC_PORT );

    return adc;
}

static int check_full_charge_using_chg_current( int charge_current_adc )
{

    if ( sec_bci.battery.battery_level_vol < 4000 )
    {
        sec_bci.battery.confirm_full_by_current = 0;
        return 0;
    }

    if (sec_bci.battery.support_monitor_full)
    {
        if ( charge_current_adc <= CHARGE_FULL_CURRENT_ADC )
        {
            sec_bci.battery.confirm_full_by_current++;

            // changing freq. of monitoring adc to Burst.
            batt_gptimer_12.expire_time = 5;
            sec_bci.battery.monitor_duration = 5;
        }
        else
        {
            sec_bci.battery.confirm_full_by_current = 0;
            // changing freq. of monitoring adc to Default.
            batt_gptimer_12.expire_time = MONITOR_DURATION_DUR_SLEEP;
            sec_bci.battery.monitor_duration = MONITOR_DEFAULT_DURATION;
        }

        if ( sec_bci.battery.confirm_full_by_current >= 4 )
        {
            batt_gptimer_12.expire_time = MONITOR_DURATION_DUR_SLEEP;
            sec_bci.battery.monitor_duration = MONITOR_DEFAULT_DURATION;
            sec_bci.battery.confirm_full_by_current = 0;

            return 1;
        }   
    }
    return 0; 
}

static void get_system_status_in_sleep( int *battery_level_ptg, 
                    int *battery_level_vol, 
                    int *battery_temp, 
                    int *charge_current_adc )
{
    int temp_adc;
    int ptg_val;

    twl4030_i2c_init();
    normal_i2c_init();

    ptg_val = get_fuelgauge_ptg_value( CHARGE_DUR_SLEEP );
	ptg_val = get_adjusted_battery_ptg(ptg_val);

    if ( ptg_val >= 0 )
    {
        *battery_level_ptg = ptg_val;
    }

    *battery_level_vol = get_fuelgauge_adc_value( CHARGE_DUR_SLEEP );

    temp_adc = t2_adc_data( device_config->TEMP_ADC_PORT );
    //temp_adc = _get_t2adc_data_( device_config->TEMP_ADC_PORT );

    if ( device_config->MONITORING_CHG_CURRENT )
        //*charge_current_adc = _get_t2adc_data_ ( device_config->CHG_CURRENT_ADC_PORT );
        *charge_current_adc = t2_adc_data ( device_config->CHG_CURRENT_ADC_PORT );

    normal_i2c_disinit();
    twl4030_i2c_disinit();

    *battery_temp = t2adc_to_temperature( temp_adc, device_config->TEMP_ADC_PORT ); 
}

static int battery_monitor_core( bool is_sleep )
{

    int charging_time;

	if(event_logging)
	{
		stop_temperature_overheat = CHARGE_STOP_TEMPERATURE_EVENT;
		recover_temperature_overheat = CHARGE_RECOVER_TEMPERATURE_EVENT;
	}
	else
	{
		stop_temperature_overheat = CHARGE_STOP_TEMPERATURE_MAX;
		recover_temperature_overheat = CHARGE_RECOVER_TEMPERATURE_MAX;	
	}
	
    /*Monitoring the system temperature*/
    if ( sec_bci.battery.monitor_field_temp )
    {
        if ( sec_bci.battery.support_monitor_timeout )
        {
            /*Check charging time*/
            charging_time = get_elapsed_time_secs( &sec_bci.charger.charge_start_time );
            if ( charging_time >= sec_bci.charger.charging_timeout )
            {
                if ( is_sleep ) 
                    sec_bci.charger.full_charge_dur_sleep = 0x2;
                else
                    _battery_state_change_( STATUS_CATEGORY_CHARGING, 
                                POWER_SUPPLY_STATUS_CHARGING_OVERTIME, 
                                is_sleep );

                return -1;
            }
        }

        if ( sec_bci.battery.support_monitor_temp )
        {
            if ( sec_bci.battery.battery_health == POWER_SUPPLY_HEALTH_OVERHEAT 
                || sec_bci.battery.battery_health == POWER_SUPPLY_HEALTH_COLD )
            {
                if ( sec_bci.battery.battery_temp <= recover_temperature_overheat //CHARGE_RECOVER_TEMPERATURE_MAX 
                    && sec_bci.battery.battery_temp >= CHARGE_RECOVER_TEMPERATURE_MIN )
                {
                    sec_bci.battery.battery_health = POWER_SUPPLY_HEALTH_GOOD;
                    _battery_state_change_( STATUS_CATEGORY_TEMP, 
                                BATTERY_TEMPERATURE_NORMAL, 
                                is_sleep );
                }

            }
            else
            {
                if ( sec_bci.battery.monitor_duration > MONITOR_TEMP_DURATION )
                    sec_bci.battery.monitor_duration = MONITOR_TEMP_DURATION;

                if ( sec_bci.battery.battery_temp >= stop_temperature_overheat) //CHARGE_STOP_TEMPERATURE_MAX )
                {
                	printk("[TA] Temperature is high (%d*)\n", sec_bci.battery.battery_temp);
                    if ( sec_bci.battery.battery_health != POWER_SUPPLY_HEALTH_OVERHEAT )
                    {
                        sec_bci.battery.battery_health = POWER_SUPPLY_HEALTH_OVERHEAT;

                        _battery_state_change_( STATUS_CATEGORY_TEMP, 
                                    BATTERY_TEMPERATURE_HIGH, 
                                    is_sleep );
                    }
                }
                else if ( sec_bci.battery.battery_temp <= CHARGE_STOP_TEMPERATURE_MIN )
                {
					printk("[TA] Temperature is low (%d*)\n", sec_bci.battery.battery_temp);
                    if ( sec_bci.battery.battery_health != POWER_SUPPLY_HEALTH_COLD )
                    {
                        sec_bci.battery.battery_health = POWER_SUPPLY_HEALTH_COLD;

                        _battery_state_change_( STATUS_CATEGORY_TEMP, 
                                    BATTERY_TEMPERATURE_LOW, 
                                    is_sleep );
                    }
                }
                else
                {
                    if ( sec_bci.battery.battery_health != POWER_SUPPLY_HEALTH_GOOD )
                    {
                        sec_bci.battery.battery_health = POWER_SUPPLY_HEALTH_GOOD;
                        _battery_state_change_( STATUS_CATEGORY_TEMP, 
                                    BATTERY_TEMPERATURE_NORMAL, 
                                    is_sleep );
                    }
                }
            }
        }
    }

    /*Monitoring the battery level for Re-charging*/
    if ( sec_bci.battery.monitor_field_rechg_vol && (sec_bci.charger.rechg_count <= 0 || is_sleep))
    {
        if ( sec_bci.battery.monitor_duration > MONITOR_RECHG_VOL_DURATION )
            sec_bci.battery.monitor_duration = MONITOR_RECHG_VOL_DURATION;

        if ( sec_bci.battery.battery_level_vol <= CHARGE_RECHG_VOLTAGE )
        {
            sec_bci.battery.confirm_recharge++;
            if ( sec_bci.battery.confirm_recharge >= 2 )
            {
                printk( "[BM] RE-charging vol\n" );
                sec_bci.battery.confirm_recharge = 0;   
                _battery_state_change_( STATUS_CATEGORY_CHARGING, 
                            POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL, 
                            is_sleep );
            }
        }
        else
        {
            sec_bci.battery.confirm_recharge = 0;
        }
    }

    return 0;   
}

static void battery_monitor_work_handler( struct work_struct *work )
{
    int is_full = 0;
    int charge_current_adc;
	int fg_current = 0;
	int recover_flag = 0;
    struct battery_device_info *di = container_of( work,
                            struct battery_device_info,
                            battery_monitor_work.work );

	if(battery_suspend_state)
	{
		printk("[BM] battery is in suspend state. return.\n");
		return;
	}
	

    boot_monitor_count++;
    if(!boot_complete && boot_monitor_count >= 3)
    {
        printk("[BM] boot complete \n");
        boot_complete = true;
    }
    
    if(!(boot_monitor_count % 10)) // every 5 minutes
    	fuelgauge_check_vf_fullcap_range();

    //recover_flag = fuelgauge_check_cap_corruption();

	if(sec_bci.charger.rechg_count > 0)
		sec_bci.charger.rechg_count--;

    if ( device_config->MONITORING_SYSTEM_TEMP )
        sec_bci.battery.battery_temp = fuelgauge_read_batt_temp(); //get_system_temperature( TEMP_DEG );
    else
        sec_bci.battery.battery_temp = 0;

    #if 0
	update_rcomp_by_temperature(sec_bci.battery.battery_temp);
	#endif
	
    /* Monitoring the battery info. */
    sec_bci.battery.battery_level_ptg = get_battery_level_ptg();
    msleep(10);
    sec_bci.battery.battery_level_vol= get_battery_level_adc();
	fg_current = get_fuelgauge_current(false, false);

	if(!get_jig_status()) // If jig cable is connected, then skip low batt compensation check.
		sec_bci.battery.battery_level_ptg = 
			do_low_batt_compensation(sec_bci.battery.battery_level_ptg, sec_bci.battery.battery_level_vol, fg_current);

    if( !( sec_bci.battery.monitor_field_temp ) && !( sec_bci.battery.monitor_field_rechg_vol ) )
    {
        sec_bci.battery.monitor_duration = MONITOR_DEFAULT_DURATION;
    }
    else
    {
        // Workaround : check status of cabel at this point.
        if ( !_cable_status_now_() )
        {
            _battery_state_change_( STATUS_CATEGORY_ETC, 
                        ETC_CABLE_IS_DISCONNECTED, 
                        CHARGE_DUR_ACTIVE );
        }

        if ( sec_bci.charger.is_charging && device_config->MONITORING_CHG_CURRENT )
        {
            // in charging && enable monitor_chg_current
            charge_current_adc = get_charging_current_adc_val();
            is_full = check_full_charge_using_chg_current( charge_current_adc );

            if ( is_full )
            {
                _battery_state_change_( STATUS_CATEGORY_CHARGING, 
                            POWER_SUPPLY_STATUS_FULL, 
                            CHARGE_DUR_ACTIVE );
            }
            else
            {
                battery_monitor_core( CHARGE_DUR_ACTIVE );
            }   
        }
        else
        {
            battery_monitor_core( CHARGE_DUR_ACTIVE );
        }
    }

    #if 1 
    printk( "[BM] monitor BATT.(%d%%, %dmV, %d* (ADC %d), Fuelgauge %d count=%d, charging=%d)\n", 
            sec_bci.battery.battery_level_ptg,
            sec_bci.battery.battery_level_vol,
            get_system_temperature( TEMP_DEG ),
            get_system_temperature( TEMP_ADC ),
            fuelgauge_read_batt_temp(),
            boot_monitor_count,
            sec_bci.charger.is_charging
            );
    #endif

	//printk("[BM] adc 167 -> %d^, adc 198 -> %d^\n", t2adc_to_temperature(927, 0), t2adc_to_temperature(884, 0));

    power_supply_changed( &di->sec_battery );
    power_supply_changed( &di->sec_ac );
    power_supply_changed( &di->sec_usb );

    queue_delayed_work( sec_bci.sec_battery_workq, 
                &di->battery_monitor_work, 
                sec_bci.battery.monitor_duration * HZ);
}

static int battery_monitor_fleeting_wakeup_handler( unsigned long arg ) 
{
    int ret = 0;
    int is_full = 0;
    int charge_current_adc = 0;

    get_system_status_in_sleep( &sec_bci.battery.battery_level_ptg,
                &sec_bci.battery.battery_level_vol,
                &sec_bci.battery.battery_temp, 
                &charge_current_adc );

	if( sec_bci.charger.is_charging && (sec_bci.battery.battery_level_ptg >= 90 || sec_bci.battery.battery_level_vol >= 4050) )
	{
	    if ( device_config->MONITORING_CHG_CURRENT )
	        is_full = check_full_charge_using_chg_current( charge_current_adc );
	    else
	        is_full = _check_full_charge_dur_sleep_();
	}

    if ( is_full )
    {
        sec_bci.charger.full_charge_dur_sleep = 0x1;
        ret = -1;
    }
    else
    {
        sec_bci.charger.full_charge_dur_sleep = 0x0;
        ret = battery_monitor_core( CHARGE_DUR_SLEEP );
    }

    if ( ret >= 0 )
        request_gptimer12( &batt_gptimer_12 );

    return ret;
}

// ------------------------------------------------------------------------- // 
//                            Power supply monitor                           //
// ------------------------------------------------------------------------- // 

static enum power_supply_property samsung_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
	#ifdef _OMS_FEATURES_
    POWER_SUPPLY_PROP_TEMP,
	#endif
    POWER_SUPPLY_PROP_CAPACITY, // in percents
};

static enum power_supply_property samsung_ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property samsung_usb_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int samsung_battery_get_property( struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val )
{
    switch ( psp ) 
    {
        case POWER_SUPPLY_PROP_STATUS:
			if( !sec_bci.charger.samsung_charger)
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else
			{
	            if( sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL 
	                || sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_RECHARGING_FOR_TEMP )
	                val->intval = POWER_SUPPLY_STATUS_CHARGING;
	            else
	                val->intval = sec_bci.charger.charge_status;
			}
            break;

        case POWER_SUPPLY_PROP_HEALTH:
            val->intval = sec_bci.battery.battery_health;
            break;

        case POWER_SUPPLY_PROP_PRESENT:
            val->intval = sec_bci.battery.battery_level_vol * 1000;
            val->intval = val->intval <= 0 ? 0 : 1;
            break;

        case POWER_SUPPLY_PROP_ONLINE :
            val->intval = sec_bci.charger.cable_status;
            break;

        case POWER_SUPPLY_PROP_TECHNOLOGY :
            val->intval = sec_bci.battery.battery_technology;
            break;

        case POWER_SUPPLY_PROP_VOLTAGE_NOW :
            val->intval = sec_bci.battery.battery_level_vol * 1000;
            break;
#ifdef _OMS_FEATURES_
        case POWER_SUPPLY_PROP_TEMP :
            val->intval = sec_bci.battery.battery_temp * 10;
            break;
#endif
        case POWER_SUPPLY_PROP_CAPACITY : /* in percents! */
            val->intval = sec_bci.battery.battery_level_ptg;
            break;


        default :
            return -EINVAL;
    }

    //printk("[BM] GET %d, %d  !!! \n", psp, val->intval );
    return 0;
}

static int samsung_ac_get_property( struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val )
{

    switch ( psp ) 
    {        
        case POWER_SUPPLY_PROP_ONLINE :
            if ( sec_bci.charger.cable_status == POWER_SUPPLY_TYPE_MAINS )
                val->intval = 1;
            else 
                val->intval = 0;
            break;
			
        case POWER_SUPPLY_PROP_VOLTAGE_NOW :
            val->intval = sec_bci.battery.battery_level_vol * 1000;
            break;

        default :
            return -EINVAL;
    }

    return 0;
}

static int samsung_usb_get_property( struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val )
{

    switch ( psp ) 
    {        
        case POWER_SUPPLY_PROP_ONLINE :
            if ( sec_bci.charger.cable_status == POWER_SUPPLY_TYPE_USB )
                val->intval = 1;
            else 
                val->intval = 0;

            break;
			
        case POWER_SUPPLY_PROP_VOLTAGE_NOW :
            val->intval = sec_bci.battery.battery_level_vol * 1000;
            break;

        default :
            return -EINVAL;
    }

    return 0;
}

static void samsung_pwr_external_power_changed( struct power_supply *psy ) 
{
    //cancel_delayed_work(&di->twl4030_bci_monitor_work);
    //schedule_delayed_work(&di->twl4030_bci_monitor_work, 0);
}


// ------------------------------------------------------------------------- // 
//                           Driver interface                                //
// ------------------------------------------------------------------------- // 

static int __devinit battery_probe( struct platform_device *pdev )
{
    int ret = 0;
    int i = 0;

    struct battery_device_info *di;
    
    printk( "[BM] Battery Probe...\n\n" );

    this_dev = &pdev->dev; 

    di = kzalloc( sizeof(*di), GFP_KERNEL );
    if(!di)
        return -ENOMEM;

    platform_set_drvdata( pdev, di );
    
    di->dev = &pdev->dev;
    device_config = pdev->dev.platform_data;

    INIT_DELAYED_WORK( &di->battery_monitor_work, battery_monitor_work_handler );

    /*Create power supplies*/
    di->sec_battery.name = "battery";
    di->sec_battery.type = POWER_SUPPLY_TYPE_BATTERY;
    di->sec_battery.properties = samsung_battery_props;
    di->sec_battery.num_properties = ARRAY_SIZE( samsung_battery_props );
    di->sec_battery.get_property = samsung_battery_get_property;
    di->sec_battery.external_power_changed = samsung_pwr_external_power_changed;

    di->sec_ac.name = "ac";
    di->sec_ac.type = POWER_SUPPLY_TYPE_MAINS;
    di->sec_ac.supplied_to = samsung_bci_supplied_to;
    di->sec_ac.num_supplicants = ARRAY_SIZE( samsung_bci_supplied_to );
    di->sec_ac.properties = samsung_ac_props;
    di->sec_ac.num_properties = ARRAY_SIZE( samsung_ac_props );
    di->sec_ac.get_property = samsung_ac_get_property;
    di->sec_ac.external_power_changed = samsung_pwr_external_power_changed;

    di->sec_usb.name = "usb";
    di->sec_usb.type = POWER_SUPPLY_TYPE_USB;
    di->sec_usb.supplied_to = samsung_bci_supplied_to;
    di->sec_usb.num_supplicants = ARRAY_SIZE( samsung_bci_supplied_to );
    di->sec_usb.properties = samsung_usb_props;
    di->sec_usb.num_properties = ARRAY_SIZE( samsung_usb_props );
    di->sec_usb.get_property = samsung_usb_get_property;
    di->sec_usb.external_power_changed = samsung_pwr_external_power_changed;

    // USE_REGULATOR [+]
    di->usb3v1 = regulator_get( &pdev->dev, "usb3v1" );
    if( IS_ERR( di->usb3v1 ) )
        goto fail_regulator1;

    di->usb1v8 = regulator_get( &pdev->dev, "usb1v8" );
    if( IS_ERR( di->usb1v8 ) )
        goto fail_regulator2;

    di->usb1v5 = regulator_get( &pdev->dev, "usb1v5" );
    if( IS_ERR( di->usb1v5 ) )
        goto fail_regulator3;
    // USE_REGULATOR [-]

    ret = power_supply_register( &pdev->dev, &di->sec_battery );
    if( ret )
    {
        printk( "[BM] Failed to register main battery, charger\n" );
        goto batt_regi_fail1;
    }

    ret = power_supply_register( &pdev->dev, &di->sec_ac );
    if( ret )
    {
        printk( "[BM] Failed to register ac\n" );
        goto batt_regi_fail2;
    }

    ret = power_supply_register( &pdev->dev, &di->sec_usb );
    if( ret )
    {
        printk( "[BM] Failed to register usb\n" );
        goto batt_regi_fail3;
    }


#ifdef _OMS_FEATURES_
    // Create battery sysfs files for sharing battery information with platform.
    ret = sysfs_create_file( &di->sec_battery.dev->kobj, &batt_vol_toolow.attr );
    if ( ret )
    {
        printk( "[BM] sysfs create fail - %s\n", batt_vol_toolow.attr.name );
    }
#endif

    for( i = 0; i < ARRAY_SIZE( batt_sysfs_testmode ); i++ )
    {
        ret = sysfs_create_file( &di->sec_battery.dev->kobj, 
                     &batt_sysfs_testmode[i].attr );
        if ( ret )
        {
            printk( "[BM] sysfs create fail - %s\n", batt_sysfs_testmode[i].attr.name );
        }
    }

    // Init. ADC
    turn_resources_on_for_adc();
    twl_i2c_write_u8( TWL4030_MODULE_USB, SEL_MADC_MCPC, CARKIT_ANA_CTRL );
    turn_resources_off_for_adc();
	battery_suspend_state = false;
    // Set gptimer12 for checking battery status in sleep mode.
    batt_gptimer_12.name = "samsung_battery_timer";
    batt_gptimer_12.expire_time =(unsigned int) MONITOR_DURATION_DUR_SLEEP;
    batt_gptimer_12.expire_callback = &battery_monitor_fleeting_wakeup_handler;
    batt_gptimer_12.data = (unsigned long) di;

#ifdef CONFIG_SEC_BATTERY_USE_RECOVERY_MODE
    if (likely(recovery_mode == 0))
        queue_delayed_work( sec_bci.sec_battery_workq, &di->battery_monitor_work, HZ/2 );
    else
        queue_delayed_work( sec_bci.sec_battery_workq, &di->battery_monitor_work, 0 );
#else
    queue_delayed_work( sec_bci.sec_battery_workq, &di->battery_monitor_work, HZ/2 );
#endif

    sec_bci.ready = true;

    return 0;

batt_regi_fail3:
    power_supply_unregister( &di->sec_ac );

batt_regi_fail2:
    power_supply_unregister( &di->sec_battery );

batt_regi_fail1:
// USE_REGULATOR [+]
    regulator_put( di->usb1v5 );
    di->usb1v5 = NULL;

fail_regulator3:
    regulator_put( di->usb1v8 );
    di->usb1v8 = NULL;

fail_regulator2:
    regulator_put( di->usb3v1 );
    di->usb3v1 = NULL;

fail_regulator1:
// USE_REGULATOR [-]
    kfree(di);

    return ret;
}

static int __devexit battery_remove( struct platform_device *pdev )
{
    struct battery_device_info *di = platform_get_drvdata( pdev );

    flush_scheduled_work();
    cancel_delayed_work( &di->battery_monitor_work );

    power_supply_unregister( &di->sec_ac );
    power_supply_unregister( &di->sec_battery );

    // USE_REGULATOR [+]
    regulator_put( di->usb1v5 );
    regulator_put( di->usb1v8 );
    regulator_put( di->usb3v1 );
    // USE_REGULATOR [-]

    platform_set_drvdata( pdev, NULL );
    kfree( di );

    return 0;
}

static int battery_suspend( struct platform_device *pdev,
                            pm_message_t state )
{
    struct battery_device_info *di = platform_get_drvdata( pdev );

	battery_suspend_state = true;
    cancel_delayed_work( &di->battery_monitor_work );

	sec_bci.charger.rechg_count = 0;
    if( sec_bci.charger.cable_status == POWER_SUPPLY_TYPE_MAINS )
    {
   		#if 0
        struct file *filp;
        char buf;
        int count;
        int retval = 0;
        mm_segment_t fs;

        fs = get_fs();
        set_fs(KERNEL_DS);
        filp = filp_open("/sys/power/vdd1_lock", 
                00000001/*O_WRONLY*/|00010000/*O_SYNC*/, 
                0x0);
        buf='1';
        count=filp->f_op->write(filp, &buf, 1, &filp->f_pos);
        retval = filp_close(filp, NULL);
        set_fs(fs);
		#endif
        request_gptimer12( &batt_gptimer_12 );
    }

    return 0;
}

static int battery_resume( struct platform_device *pdev )
{
    struct battery_device_info *di = platform_get_drvdata( pdev );

	printk("battery_resume\n");
	battery_suspend_state = false;
    if ( batt_gptimer_12.active )
    {
    	#if 0
        struct file *filp;
        char buf;
        int count;
        int retval = 0;
        mm_segment_t fs;

        fs = get_fs();
        set_fs(KERNEL_DS);
        filp = filp_open("/sys/power/vdd1_lock", 
                00000001/*O_WRONLY*/|00010000/*O_SYNC*/, 
                0x0);
        buf='0';
        count=filp->f_op->write(filp, &buf, 1, &filp->f_pos);
        retval = filp_close(filp, NULL);
        set_fs(fs);
		#endif
        release_gptimer12( &batt_gptimer_12 );
    }

    switch ( sec_bci.charger.full_charge_dur_sleep )
    {
        case 0x1 : 
            _battery_state_change_( STATUS_CATEGORY_CHARGING, 
                        POWER_SUPPLY_STATUS_FULL_DUR_SLEEP, 
                        CHARGE_DUR_ACTIVE );
            break;

        case 0x2 : 
            _battery_state_change_( STATUS_CATEGORY_CHARGING, 
                        POWER_SUPPLY_STATUS_CHARGING_OVERTIME, 
                        CHARGE_DUR_ACTIVE );
            break;

        default : 
            break;
    }

	if(sec_bci.charger.full_charge_dur_sleep)
		wake_lock_timeout( &sec_bc_wakelock , HZ * 5 );
	
    power_supply_changed( &di->sec_battery );
    power_supply_changed( &di->sec_ac );
    power_supply_changed( &di->sec_usb );

    sec_bci.charger.full_charge_dur_sleep = 0x0;
    
    queue_delayed_work( sec_bci.sec_battery_workq, &di->battery_monitor_work, HZ );

    return 0;
}

struct platform_driver battery_platform_driver = {
    .probe      = battery_probe,
    .remove     = __devexit_p( battery_remove ),
    .suspend    = &battery_suspend,
    .resume     = &battery_resume,
    .driver     = {
        .name = DRIVER_NAME,
    },
};


static int __init battery_init( void )
{
    int ret;

    printk( "\n[BM] Battery Init.\n" );

    sec_bci.ready = false;

    sec_bci.battery.battery_health = POWER_SUPPLY_HEALTH_GOOD;
    sec_bci.battery.battery_technology = POWER_SUPPLY_TECHNOLOGY_LION;
    sec_bci.battery.battery_level_ptg = 0;
    sec_bci.battery.battery_level_vol = 0;
    sec_bci.battery.monitor_duration = MONITOR_DEFAULT_DURATION;
    sec_bci.battery.monitor_field_temp = false;
    sec_bci.battery.monitor_field_rechg_vol = false;
    sec_bci.battery.confirm_full_by_current = 0;
    sec_bci.battery.support_monitor_temp = 1;
    sec_bci.battery.support_monitor_timeout = 1;
    sec_bci.battery.support_monitor_full = 1;
    sec_bci.battery.confirm_recharge = 0;
	sec_bci.battery.low_batt_comp_flag = 0;

    sec_bci.charger.prev_cable_status = -1;
    sec_bci.charger.cable_status = -1;
    sec_bci.charger.prev_charge_status = 0;
    sec_bci.charger.charge_status = 0;
    sec_bci.charger.full_charge_dur_sleep = 0x0;
    sec_bci.charger.is_charging = false;
    sec_bci.charger.charge_start_time = 0;
    sec_bci.charger.charged_time = 0;
    sec_bci.charger.charging_timeout = DEFAULT_CHARGING_TIMEOUT;
    sec_bci.charger.use_ta_nconnected_irq = false;
	sec_bci.charger.rechg_count = 0;

    sec_bci.sec_battery_workq = create_singlethread_workqueue("sec_battery_workq");

    init_gptimer12();
    printk( "[BM] Init Gptimer called \n" );

    /* Get the charger driver */
    if( ( ret = charger_init() < 0 ) )
    {
        printk( "[BM] Fail to get charger driver.\n" );
        return ret;
    }

    /* Get the fuelgauge driver */
    if( ( ret = fuelgauge_init() < 0 ) )
    {
        printk( "[BM] Fail to get fuelgauge driver.\n" );        
        return ret;
    }

    wake_lock_init( &sec_bc_wakelock, WAKE_LOCK_SUSPEND, "samsung-battery" );

    ret = platform_driver_register( &battery_platform_driver );

    return ret;
}
module_init( battery_init );

static void __exit battery_exit( void )
{
    /*Remove the charger driver*/
    charger_exit();
    /*Remove the fuelgauge driver*/
    fuelgauge_exit();

    finish_gptimer12();
    
    platform_driver_unregister( &battery_platform_driver );
    printk( KERN_ALERT "[BM] Battery Driver Exit.\n" );
}

module_exit( battery_exit );

MODULE_AUTHOR( "EUNGON KIM <egstyle.kim@samsung.com>" );
MODULE_DESCRIPTION( "Samsung Battery monitor driver" );
MODULE_LICENSE( "GPL" );
