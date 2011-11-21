/*
 * module/samsung_battery/charger_max8845.c
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
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <mach/gpio.h>
#include <plat/mux.h>
#include <plat/microusbic.h>
#include <plat/omap3-gptimer12.h>
#include "common.h"

#define DRIVER_NAME             "secChargerDev"


// THIS CONFIG IS SET IN BOARD_FILE.(platform_data)
struct charger_device_config 
{
    /* CHECK BATTERY VF USING ADC */
    int VF_CHECK_USING_ADC; // true or false
    int VF_ADC_PORT;
    
    /* SUPPORT CHG_ING IRQ FOR CHECKING FULL */
    int SUPPORT_CHG_ING_IRQ;
};

static struct charger_device_config *device_config;

struct charger_device_info 
{
    struct device *dev;
    struct delayed_work cable_detection_work;
    struct delayed_work full_charge_work;

    struct regulator *usb1v5;
    struct regulator *usb1v8;
    struct regulator *usb3v1;
    bool usb3v1_is_enabled;
};

static struct device *this_dev;

static int KCHG_EN_GPIO;
static int KCHG_ING_GPIO; // KCHG_ING_GPIO (LOW: Not Full, HIGH: Full)
static int KCHG_ING_IRQ; 
static bool KCHG_ING_IRQ_ENABLE;
static int KTA_NCONN_GPIO;
static int KTA_NCONN_IRQ;
static int KUSB_CONN_IRQ;
static int KCHARGING_BOOT;

static SEC_battery_charger_info *sec_bci;

// Prototype
       int _check_full_charge_dur_sleep_( void );
       int _battery_state_change_( int, int, bool );
       int _cable_status_now_( void );
static void clear_charge_start_time( void );
static void set_charge_start_time( void );
static void change_cable_status( int, struct charger_device_info *, bool );
static void change_charge_status( int, bool );
static void enable_charging( bool );
static void disable_charging( bool );
static bool check_battery_vf( void );
static irqreturn_t cable_detection_isr( int, void * );
static void cable_detection_work_handler( struct work_struct * );
static irqreturn_t full_charge_isr( int, void * );
static void full_charge_work_handler( struct work_struct * );
static int __devinit charger_probe( struct platform_device * );
static int __devexit charger_remove( struct platform_device * );
static int charger_suspend( struct platform_device *, pm_message_t );
static int charger_resume( struct platform_device * );
       int charger_init( void );
       void charger_exit( void );

extern SEC_battery_charger_info *get_sec_bci( void );
extern int _charger_state_change_( int category, int value, bool is_sleep );
extern int _get_t2adc_data_( int ch );
extern int _get_average_value_( int *data, int count );
extern int omap34xx_pad_get_wakeup_status( int gpio );
extern int omap34xx_pad_set_padoff( int gpio, int wakeup_en );
extern unsigned long long sched_clock( void );
extern int get_real_usbic_state(void);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
extern int set_tsp_for_ta_detect(int state);
#endif

#undef USE_DISABLE_CONN_IRQ

#ifdef USE_DISABLE_CONN_IRQ
#define MANAGE_CONN_IRQ
#ifdef MANAGE_CONN_IRQ
static int enable_irq_conn( bool );

static spinlock_t irqctl_lock = SPIN_LOCK_UNLOCKED;
static bool is_enabling_conn_irq;

static int enable_irq_conn( bool en )
{
    unsigned long flags;

    spin_lock_irqsave( &irqctl_lock, flags );
    if ( en )
    {
        if ( !is_enabling_conn_irq )
        {
            if ( !sec_bci->charger.use_ta_nconnected_irq )
                enable_irq( KUSB_CONN_IRQ );
            
            enable_irq( KTA_NCONN_IRQ );
            is_enabling_conn_irq = true;
        }
        else
            return -1;
        
    }
    else
    {
        if ( is_enabling_conn_irq )
        {
            if ( !sec_bci->charger.use_ta_nconnected_irq )
                disable_irq( KUSB_CONN_IRQ );
            
            disable_irq( KTA_NCONN_IRQ );
            is_enabling_conn_irq = false;
        }
        else
            return -1;
        
    }
    spin_unlock_irqrestore( &irqctl_lock, flags );

    return 0;
}
#endif

#endif


int _check_full_charge_dur_sleep_( void )
{
    int ret = 0;
    int chg_ing_level = 0;
    int i;
	int j;
    unsigned char confirm_full = 0x0;

    // Check 
    for ( i = 0; i < 8; i++ )
    {
        chg_ing_level = gpio_get_value( KCHG_ING_GPIO );
        confirm_full |= chg_ing_level << i;
        mdelay( 3 );
    }

	if ( confirm_full != 0 )
    {
        ret = 1;
    }
	else
	{
		ret = 0;
	}
    return ret;

}

int _battery_state_change_( int category, int value, bool is_sleep )
{
    struct charger_device_info *di;
    struct platform_device *pdev;


    pdev = to_platform_device( this_dev );
    di = platform_get_drvdata( pdev );

    //printk( "[TA] cate: %d, value: %d, %s\n", category, value, di->dev->kobj.name );
    switch( category )
    {
        case STATUS_CATEGORY_TEMP:
            switch ( value )
            {
                case BATTERY_TEMPERATURE_NORMAL :
                    printk( "[TA] Charging re start normal TEMP!!\n" );
                    change_charge_status( POWER_SUPPLY_STATUS_RECHARGING_FOR_TEMP, is_sleep );
                    break;
            
                case BATTERY_TEMPERATURE_LOW :
                    printk( "[TA] Charging stop LOW TEMP!!\n" );
                    change_charge_status( POWER_SUPPLY_STATUS_NOT_CHARGING, is_sleep );
                    break;
            
                case BATTERY_TEMPERATURE_HIGH :
                    printk( "[TA] Charging stop HI TEMP!!\n" );
                    change_charge_status( POWER_SUPPLY_STATUS_NOT_CHARGING, is_sleep );
                    break;
            
                default :
                    break;
            }

            break;

        case STATUS_CATEGORY_CHARGING:
            switch ( value )
            {
                case POWER_SUPPLY_STATUS_FULL :
                    printk( "[TA] Charge FULL! (monitoring charge current)\n" );
                    change_charge_status( POWER_SUPPLY_STATUS_FULL, is_sleep );
                    break;
            
                case POWER_SUPPLY_STATUS_CHARGING_OVERTIME :
                    printk( "[TA] CHARGING TAKE OVER 5 hours !!\n" );
                    change_charge_status( POWER_SUPPLY_STATUS_FULL, is_sleep );
                    break;
            
                case POWER_SUPPLY_STATUS_FULL_DUR_SLEEP :
                    printk( "[TA] Charge FULL!\n" );
                    change_charge_status( POWER_SUPPLY_STATUS_FULL, is_sleep );
                    break;
            
                case POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL :
                    printk( "[TA] Re-Charging Start!!\n" );
                    change_charge_status( POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL, is_sleep );
                    break;
            
                default :
                    break;
            }
            
            break;

        case STATUS_CATEGORY_ETC:
            switch ( value )
            {
                case ETC_CABLE_IS_DISCONNECTED :
                    printk( "[TA] CABLE IS NOT CONNECTED.... Charge Stop!!\n" );
                    change_cable_status( POWER_SUPPLY_TYPE_BATTERY, di, is_sleep );
                    break;

                default : 
                    break;
            }

            break;

        default:
            printk( "[TA] Invalid category!!!!!\n" );
            break;
    }

    return 0;
}

int _cable_status_now_( void )
{
    int ret = 0;
    
    if ( sec_bci->charger.use_ta_nconnected_irq )
    {
        ret = gpio_get_value( KTA_NCONN_GPIO ) ? 0 : 1;
    }
    else
    {
        // temporary remove
        ret = get_real_usbic_state();
    }

    return ret;
}

static void clear_charge_start_time( void )
{
    sec_bci->charger.charge_start_time = sched_clock();
}

static void set_charge_start_time( void )
{
    sec_bci->charger.charge_start_time = sched_clock();
}

static void change_cable_status( int status, 
                struct charger_device_info *di, 
                bool is_sleep )
{
    sec_bci->charger.prev_cable_status = sec_bci->charger.cable_status;
    sec_bci->charger.cable_status = status;

    _charger_state_change_( STATUS_CATEGORY_CABLE, status, is_sleep );

    switch ( status )
    {
        case POWER_SUPPLY_TYPE_BATTERY :
            /*Diable charging*/
            change_charge_status( POWER_SUPPLY_STATUS_DISCHARGING, is_sleep );

            break;

        case POWER_SUPPLY_TYPE_MAINS :        
        case POWER_SUPPLY_TYPE_USB :
            /*Enable charging*/
            change_charge_status( POWER_SUPPLY_STATUS_CHARGING, is_sleep );

            break;

        default :
            break;
    }

}

static void change_charge_status( int status, bool is_sleep )
{

    switch ( status )
    {
    case POWER_SUPPLY_STATUS_UNKNOWN :
    case POWER_SUPPLY_STATUS_DISCHARGING :
    case POWER_SUPPLY_STATUS_NOT_CHARGING :
        if(sec_bci->battery.battery_health != POWER_SUPPLY_HEALTH_DEAD)
            disable_charging( is_sleep );
        break;

    case POWER_SUPPLY_STATUS_FULL :
		sec_bci->charger.rechg_count = 4;
        disable_charging( is_sleep );
        /*Cancel timer*/
        clear_charge_start_time();

        break;

    case POWER_SUPPLY_STATUS_CHARGING :

        if ( sec_bci->battery.battery_vf_ok )
        {
            sec_bci->battery.battery_health = POWER_SUPPLY_HEALTH_GOOD;

            /*Start monitoring charging time*/
            set_charge_start_time();

        }
        else
        {
            sec_bci->battery.battery_health = POWER_SUPPLY_HEALTH_DEAD;

            status = POWER_SUPPLY_STATUS_DISCHARGING;

            printk( "[TA] INVALID BATTERY, %d !! \n", status );
        }
        enable_charging( is_sleep );
        break;

    case POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL :
        /*Start monitoring charging time*/
        sec_bci->charger.charging_timeout = DEFAULT_RECHARGING_TIMEOUT;
        set_charge_start_time();

        enable_charging( is_sleep );

        break;
        
    case POWER_SUPPLY_STATUS_RECHARGING_FOR_TEMP :
        enable_charging( is_sleep );
        break;

    default :
        ;
    }

    sec_bci->charger.prev_charge_status = sec_bci->charger.charge_status;
    sec_bci->charger.charge_status = status;

    _charger_state_change_( STATUS_CATEGORY_CHARGING, status, is_sleep );

}

static void enable_charging( bool is_sleep )
{
    // CHG_EN is low active
    if ( is_sleep )
        omap_set_gpio_dataout_in_sleep( KCHG_EN_GPIO, 0 );
    else
        gpio_set_value( KCHG_EN_GPIO, 0 ); 

    sec_bci->charger.is_charging = true;
}

static void disable_charging( bool is_sleep )
{
    // CHG_EN is low active
    if ( is_sleep )
        omap_set_gpio_dataout_in_sleep( KCHG_EN_GPIO, 1 ); 
    else
        gpio_set_value( KCHG_EN_GPIO, 1 ); 

    sec_bci->charger.is_charging = false;
}

static bool check_battery_vf( void )
{
    int count = 0;
    int val;
    bool ret = false;
    
    count = 0;
    disable_charging( CHARGE_DUR_ACTIVE );
    msleep( 100 );
    enable_charging( CHARGE_DUR_ACTIVE );
    val = gpio_get_value( KCHG_ING_GPIO );
    if ( !val )
        return true;
    
    while ( count < 10 )
    {
        if ( !gpio_get_value( KCHG_ING_GPIO ) )
            return true;

        count++;
        msleep( 1 );
    }

    if ( !ret && device_config->VF_CHECK_USING_ADC )
    {
        //msleep( 500 );
        count = _get_t2adc_data_( device_config->VF_ADC_PORT );
        printk("[TA] vf: %d\n", count);
        if ( count < 100 )
            return true;
    }

    return ret;
}

static irqreturn_t cable_detection_isr( int irq, void *_di )
{
    struct charger_device_info *di = _di;

    if ( sec_bci->ready )
    {
#ifdef USE_DISABLE_CONN_IRQ
#ifdef MANAGE_CONN_IRQ
        enable_irq_conn( false );
#else
        if ( !sec_bci->charger.use_ta_nconnected_irq )
            disable_irq( KUSB_CONN_IRQ );

        disable_irq( KTA_NCONN_IRQ );
#endif
#endif
        cancel_delayed_work( &di->cable_detection_work );
        queue_delayed_work( sec_bci->sec_battery_workq, &di->cable_detection_work, 0 );
    }

    return IRQ_HANDLED;
}

static void cable_detection_work_handler( struct work_struct * work )
{
    struct charger_device_info *di = container_of( work,
                                        struct charger_device_info,
                                        cable_detection_work.work );
    int n_usbic_state;
    int count;
    int ret;

    printk("[TA] cable_detection_work_handler start!!!!\n");

    #if 0
    if(gpio_get_value(KTA_NCONN_GPIO))
    {
        printk("[TA] TA_NCONNECTED is HIGH\n");
        n_usbic_state = 0;
    }
    else
    {
        printk("[TA] TA_NCONNECTED is LOW\n");
        n_usbic_state = MICROUSBIC_USB_CABLE;
    }
    #endif
    
    clear_charge_start_time();

    n_usbic_state = get_real_usbic_state();
    printk( "[TA] cable_detection_isr handler. usbic_state: %d\n", n_usbic_state );

    if ( sec_bci->charger.use_ta_nconnected_irq )
    {
        if ( gpio_get_value( KTA_NCONN_GPIO ) )
        {
            count = 0;
            while ( count < 10 )
            {
                if ( gpio_get_value( KTA_NCONN_GPIO ) )
                {
                    count++;
                    msleep( 1 );
                    if ( count == 10 )
                    {
                        n_usbic_state = -10;
                        printk( "[TA] CABLE UNPLUGGED.\n" );
                    }
                }
                else
                {
                    break;
                }    
            }
        }
    }

    // Workaround for Archer [+]
    if ( !n_usbic_state && sec_bci->charger.use_ta_nconnected_irq )
    {
        if ( !gpio_get_value( KTA_NCONN_GPIO ) )
            n_usbic_state = MICROUSBIC_5W_CHARGER;
    }
    // Workaround for Archer [-}

    switch ( n_usbic_state )
    {
        case MICROUSBIC_5W_CHARGER :
        case MICROUSBIC_TA_CHARGER :
        case MICROUSBIC_USB_CHARGER :
        case MICROUSBIC_PHONE_USB : 
        case MICROUSBIC_USB_CABLE :
            if ( sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_USB
                || sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_MAINS )
            {
                //printk( "[TA] already Plugged.\n" );
                goto Out_IRQ_Cable_Det;
            }

            #ifdef WR_ADC
            /* Workaround to get proper adc value */
            if ( !di->usb3v1_is_enabled )
            {
                ret = regulator_enable( di->usb3v1 );
                if ( ret )
                    printk("[TA] Regulator 3v1 error!!\n");
                else
                    di->usb3v1_is_enabled = true;
            }

            twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_ACTIVE, TWL4030_VUSB3V1_REMAP );
            twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_ACTIVE, TWL4030_VINTANA2_REMAP );

            msleep( 100 );
            #endif

            /*Check VF*/
            sec_bci->battery.battery_vf_ok = check_battery_vf();

            /*TA or USB is inserted*/
            if ( n_usbic_state == MICROUSBIC_USB_CABLE )
            {
                //current : 482mA
                printk( "[TA] USB CABLE PLUGGED\n" );
                change_cable_status( POWER_SUPPLY_TYPE_USB, di, CHARGE_DUR_ACTIVE );
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
                set_tsp_for_ta_detect(1);
#endif
            }
            else
            {
                //current : 636mA
                printk( "[TA] CHARGER CABLE PLUGGED\n" );
                change_cable_status( POWER_SUPPLY_TYPE_MAINS, di, CHARGE_DUR_ACTIVE );
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
                set_tsp_for_ta_detect(1);
#endif
            }

            if ( device_config->SUPPORT_CHG_ING_IRQ && KCHG_ING_IRQ_ENABLE == false)
            {
            	printk("%s, enable chg_ing irq\n", __func__);
                enable_irq( KCHG_ING_IRQ );
                KCHG_ING_IRQ_ENABLE = true;
            }

            break;

        default:
            if ( sec_bci->charger.prev_cable_status != POWER_SUPPLY_TYPE_BATTERY
                && sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_BATTERY )
            {
                //printk( "[TA] already Unplugged.\n" );
                goto Out_IRQ_Cable_Det;
            }
            else if ( sec_bci->charger.prev_cable_status == -1
                && sec_bci->charger.cable_status == -1 )
            {
                printk( "[TA] Fisrt time after bootig.\n" );
                goto FirstTime_Boot;
            }

            if(device_config->SUPPORT_CHG_ING_IRQ && KCHG_ING_IRQ_ENABLE)
            {
            	printk("%s, disable chg_ing irq\n", __func__);
                disable_irq( KCHG_ING_IRQ );
                KCHG_ING_IRQ_ENABLE = false;
            }

            #ifdef WR_ADC
            /* Workaround to get proper adc value */
            if ( di->usb3v1_is_enabled )
                regulator_disable( di->usb3v1 );

            di->usb3v1_is_enabled = false;

            twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_OFF, TWL4030_VUSB3V1_REMAP );
            twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_OFF, TWL4030_VINTANA2_REMAP );
            #endif

FirstTime_Boot:
            /*TA or USB is ejected*/
            printk( "[TA] CABLE UNPLUGGED\n" );
            change_cable_status( POWER_SUPPLY_TYPE_BATTERY, di, CHARGE_DUR_ACTIVE );
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
            set_tsp_for_ta_detect(0);
#endif

            break;
    }

Out_IRQ_Cable_Det:
    ;
#ifdef USE_DISABLE_CONN_IRQ
#ifdef MANAGE_CONN_IRQ
    enable_irq_conn( true );
#else
    if ( !sec_bci->charger.use_ta_nconnected_irq )
        enable_irq( KUSB_CONN_IRQ );

    enable_irq( KTA_NCONN_IRQ );
#endif
#endif
}

static irqreturn_t full_charge_isr( int irq, void *_di )
{
    struct charger_device_info *di = _di;

    if ( sec_bci->ready)
    {
        #if 0
        if(KCHG_ING_IRQ_ENABLE && sec_bci->charger.cable_status != POWER_SUPPLY_TYPE_BATTERY)
        {
            printk("[TA] disable_irq - chg_ing\n");
            disable_irq( KCHG_ING_IRQ );
            KCHG_ING_IRQ_ENABLE = false;    
        }
        #endif

		if(sec_bci->charger.is_charging && !KCHARGING_BOOT)
		{
	        cancel_delayed_work( &di->full_charge_work );
    	    queue_delayed_work( sec_bci->sec_battery_workq, &di->full_charge_work, HZ/10 );
	   	}

		if(KCHARGING_BOOT > 0)
			KCHARGING_BOOT--;
    }

    return IRQ_HANDLED;
}

static void full_charge_work_handler( struct work_struct *work )
{
    struct charger_device_info *di;
    int count;
    int n_usbic_state;

    if ( !sec_bci->charger.is_charging )
        goto Enable_IRQ_Full_Det;

   printk( "[TA] Full_charge_work_handler!\n" );

    #if 0
    if(gpio_get_value(KTA_NCONN_GPIO))
    {
        printk("[TA] TA_NCONNECTED is HIGH\n");
        n_usbic_state = 0;
    }
    else
    {
        printk("[TA] TA_NCONNECTED is LOW\n");
        n_usbic_state = MICROUSBIC_USB_CABLE;
    }
    #endif

    
    // Temporary remove
    n_usbic_state = get_real_usbic_state();
	//printk("full_charge_work_handler, n_usbic_state = %d\n", n_usbic_state);
    switch ( n_usbic_state )
    {
        case MICROUSBIC_5W_CHARGER :
        case MICROUSBIC_TA_CHARGER :
        case MICROUSBIC_USB_CHARGER :
        case MICROUSBIC_USB_CABLE :
        case MICROUSBIC_PHONE_USB : 
            break;

        // Not connected
        default :
            goto Enable_IRQ_Full_Det;       
    }

    count = 0;
    while ( count < 10 )
    {
    	if(sec_bci->battery.battery_level_ptg >= 90 || sec_bci->battery.battery_level_vol >= 4050)
			break;
        if ( !gpio_get_value( KCHG_ING_GPIO ) )
        {
            if(sec_bci->battery.battery_health == POWER_SUPPLY_HEALTH_DEAD)
            {
                printk("[TA] %s, CHG_ING is low, Battery health is dead. - recharging\n", __func__);
                sec_bci->battery.battery_vf_ok = check_battery_vf();
                change_charge_status( POWER_SUPPLY_STATUS_CHARGING, CHARGE_DUR_ACTIVE );
            }
            printk("[TA] %s, CHG_ING is low, count:%d\n", __func__, count);
            goto Enable_IRQ_Full_Det;
            break;
        }
        msleep( 10 );
        count++;
    }

    di = container_of( work, struct charger_device_info, full_charge_work.work );

    if ( device_config->SUPPORT_CHG_ING_IRQ && (sec_bci->battery.battery_level_ptg >= 90 || sec_bci->battery.battery_level_vol >= 4050))
    {
        if ( sec_bci->charger.is_charging )
        {
            printk( "[TA] Charge FULL!\n" );
            change_charge_status( POWER_SUPPLY_STATUS_FULL, CHARGE_DUR_ACTIVE );
        }
    }
    
Enable_IRQ_Full_Det :
	#if 0
	if(!KCHG_ING_IRQ_ENABLE && sec_bci->charger.cable_status != POWER_SUPPLY_TYPE_BATTERY)
	{
		printk("[TA] enable_irq - chg_ing\n");
		enable_irq( KCHG_ING_IRQ );
		KCHG_ING_IRQ_ENABLE = true;
	}
	#endif
    return;
}

static int __devinit charger_probe( struct platform_device *pdev )
{

    int ret = 0;
    int irq = 0;
    struct charger_device_info *di;
    
    printk( "[TA] Charger probe...\n" );

    sec_bci = get_sec_bci();

    di = kzalloc( sizeof(*di), GFP_KERNEL );
    if (!di)
        return -ENOMEM;

    platform_set_drvdata( pdev, di );
    di->dev = &pdev->dev;
    device_config = pdev->dev.platform_data;

    #if 0
    printk( "[TA] %d, %d, %d\n ",
            device_config->VF_CHECK_USING_ADC,
            device_config->VF_ADC_PORT,
            device_config->SUPPORT_CHG_ING_IRQ);
    #endif
    
    this_dev = &pdev->dev; 

    /*Init Work*/
    INIT_DELAYED_WORK( &di->cable_detection_work, cable_detection_work_handler );
    INIT_DELAYED_WORK( &di->full_charge_work, full_charge_work_handler );

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

    /*Request charger interface interruption*/
#ifdef USE_DISABLE_CONN_IRQ
#ifdef MANAGE_CONN_IRQ
    is_enabling_conn_irq = false;
#endif
#endif

    // Refer board_init_battery for checking resources
    KUSB_CONN_IRQ = platform_get_irq( pdev, 0 ); //-> Latona does not used KUSB_CONN_IRQ.
    if ( KUSB_CONN_IRQ ) 
    {
        ret = request_irq( KUSB_CONN_IRQ, cable_detection_isr, 
                IRQF_DISABLED | IRQF_SHARED, pdev->name, di );
        if ( ret )
        {
            printk( "[TA] 1. could not request irq %d, status %d\n", KUSB_CONN_IRQ, ret );
            goto usb_irq_fail;
        }
        set_irq_type( KUSB_CONN_IRQ, IRQ_TYPE_EDGE_BOTH );
#ifdef USE_DISABLE_CONN_IRQ
        disable_irq( KUSB_CONN_IRQ );
#endif
    }
    else
    {
        sec_bci->charger.use_ta_nconnected_irq = true;
    }
    
    KTA_NCONN_IRQ = platform_get_irq( pdev, 1 );
    ret = request_irq( KTA_NCONN_IRQ, cable_detection_isr, IRQF_DISABLED, pdev->name, di );
    if ( ret )
    {
        printk( "[TA] 2. could not request irq %d, status %d\n", KTA_NCONN_IRQ, ret );
        goto ta_irq_fail;
    }
    set_irq_type( KTA_NCONN_IRQ, IRQ_TYPE_EDGE_BOTH );

#ifdef USE_DISABLE_CONN_IRQ
    disable_irq( KTA_NCONN_IRQ );
#endif
    if ( sec_bci->charger.use_ta_nconnected_irq )
        KTA_NCONN_GPIO = irq_to_gpio( KTA_NCONN_IRQ );  

    KCHG_ING_IRQ = platform_get_irq( pdev, 2 );
    KCHG_ING_GPIO = irq_to_gpio( KCHG_ING_IRQ );
    printk( "[TA] CHG_ING IRQ : %d \n", KCHG_ING_IRQ );
    printk( "[TA] CHG_ING GPIO : %d \n", KCHG_ING_GPIO );

    if ( device_config->SUPPORT_CHG_ING_IRQ )
    {
        ret = request_irq( KCHG_ING_IRQ, full_charge_isr, IRQF_DISABLED, pdev->name, di ); 
        set_irq_type( KCHG_ING_IRQ, IRQ_TYPE_EDGE_RISING);
        if ( ret )
        {
            printk( "[TA] 3. could not request irq2 status %d\n", ret );
            goto chg_full_irq_fail;
        }
        disable_irq( KCHG_ING_IRQ );
        KCHG_ING_IRQ_ENABLE = false;
    }

    KCHG_EN_GPIO = irq_to_gpio( platform_get_irq( pdev, 3 ) );
    printk( "[TA] CHG_EN GPIO : %d \n", KCHG_EN_GPIO );

	if(!gpio_get_value(KTA_NCONN_GPIO))
		KCHARGING_BOOT = 1;
	else 
		KCHARGING_BOOT = 0;
    queue_delayed_work( sec_bci->sec_battery_workq, &di->cable_detection_work, HZ );

    return 0;

chg_full_irq_fail:
    irq = platform_get_irq( pdev, 1 );
    free_irq( irq, di );

ta_irq_fail:
    irq = platform_get_irq( pdev, 0 );
    free_irq( irq, di );

usb_irq_fail:
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

static int __devexit charger_remove( struct platform_device *pdev )
{
    struct charger_device_info *di = platform_get_drvdata( pdev );
    int irq;

    irq = platform_get_irq( pdev, 0 );
    free_irq( irq, di );

    irq = platform_get_irq( pdev, 1 );
    free_irq( irq, di );

    flush_scheduled_work();

    // USE_REGULATOR [+]
    regulator_put( di->usb1v5 );
    regulator_put( di->usb1v8 );
    regulator_put( di->usb3v1 );
    // USE_REGULATOR [-]

    platform_set_drvdata( pdev, NULL );
    kfree( di );

    return 0;
}

static int charger_suspend( struct platform_device *pdev,
                                  pm_message_t state )
{
    //disable_irq_wake( KCHG_ING_IRQ );
    //omap34xx_pad_set_padoff( KCHG_ING_GPIO, 0 );
    
    return 0;
}

static int charger_resume( struct platform_device *pdev )
{
    //omap34xx_pad_set_padoff( KCHG_ING_GPIO, 1 );
    //enable_irq_wake( KCHG_ING_IRQ );
    
    return 0;
}

struct platform_driver charger_platform_driver = {
    .probe      = &charger_probe,
    .remove     = __devexit_p( charger_remove ),
    .suspend    = &charger_suspend,
    .resume     = &charger_resume,
    .driver     = {
        .name = DRIVER_NAME,
    },
};

int charger_init( void )
{
    int ret;

    printk( KERN_ALERT "[TA] Charger Init.\n" );
    ret = platform_driver_register( &charger_platform_driver );

    return ret;
}

void charger_exit( void )
{
    platform_driver_unregister( &charger_platform_driver );
    printk( KERN_ALERT "[TA] Charger IC Exit.\n" );
}
