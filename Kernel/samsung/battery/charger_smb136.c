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
#include <linux/i2c.h>
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
#include <plat/board.h>
#include <plat/omap3-gptimer12.h>
#include <linux/i2c/twl4030-madc.h>
#include <linux/i2c/twl.h>
#include "common.h"

#define DRIVER_NAME             "secChargerDev"

//#define SMB136_SLAVE_ADDR		9A
/* SMB136 Registers. */
#define SMB_ChargeCurrent		0x00
#define SMB_InputCurrentLimit	0x01
#define SMB_FloatVoltage		0x02
#define SMB_ControlA			0x03
#define SMB_ControlB			0x04
#define SMB_PinControl			0x05
#define SMB_OTGControl			0x06
#define SMB_Fault				0x07
#define SMB_Temperature			0x08
#define SMB_SafetyTimer			0x09
#define SMB_VSYS				0x0A
#define SMB_I2CAddr				0x0B

#define SMB_IRQreset			0x30
#define SMB_CommandA			0x31
#define SMB_StatusA				0x32
#define SMB_StatusB				0x33
#define SMB_StatusC				0x34
#define SMB_StatusD				0x35
#define SMB_StatusE				0x36
#define SMB_StatusF				0x37
#define SMB_StatusG				0x38
#define SMB_StatusH				0x39
#define SMB_DeviceID			0x3B
#define SMB_CommandB			0x3C

/* SMB_StatusC register bit. */
#define SMB_USB					1
#define SMB_CHARGER				0
#define Compelete				1
#define Busy					0
#define InputCurrent275			0xE
#define InputCurrent500			0xF
#define InputCurrent700			0x0
#define InputCurrent800			0x1
#define InputCurrent900			0x2
#define InputCurrent1000		0x3
#define InputCurrent1100		0x4
#define InputCurrent1200		0x5
#define InputCurrent1300		0x6
#define InputCurrent1400		0x7

#define AP_ADC_CHECK_1			6

#if ( (defined( CONFIG_MACH_SAMSUNG_P1LITE ) || defined( CONFIG_MACH_SAMSUNG_P1WIFI )) && ( CONFIG_SAMSUNG_REL_HW_REV >= 3 ) )
	#define USE_GPIO_I2C_CHARGER
	#define CHARGER_SLAVE_ADDR 0x4D
	
    #include <plat/i2c-omap-gpio.h>
    static OMAP_GPIO_I2C_CLIENT * charger_gpio_i2c_client;
#else
	#define USE_EXCLUSIVE_I2C_CHARGER
	static struct i2c_client * charger_i2c_client;
#endif

extern void qt602240_inform_charger_connection(int mode);
//extern int get_usbmenupath_value(void);

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
    struct delayed_work full_comp_work;

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

static struct wake_lock sec_charger_wakelock;
static SEC_battery_charger_info *sec_bci;

// Prototype
       int _check_full_charge_dur_sleep_( void );
       int _battery_state_change_( int, int, bool );
       int _cable_status_now_( void );
	   int is_fullcharged(void);
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
static int __devinit charger_probe( struct platform_device *pdev );
static int __devexit charger_remove( struct platform_device * );
static int charger_suspend( struct platform_device *, pm_message_t );
static int charger_resume( struct platform_device * );
       int charger_init( void );
       void charger_exit( void );
s32 t2_write(u8, u8, u8);
static void set_charging_current( bool );

extern SEC_battery_charger_info *get_sec_bci( void );
extern int _charger_state_change_( int category, int value, bool is_sleep );
extern int _get_t2adc_data_( int ch );
extern int _get_average_value_( int *data, int count );
extern int omap34xx_pad_get_wakeup_status( int gpio );
extern int omap34xx_pad_set_padoff( int gpio, int wakeup_en );
extern unsigned long long sched_clock( void );
#ifdef CONFIG_FSA9480_MICROUSB
extern int get_real_usbic_state(void);
#else
static int get_charger_type(bool, bool);
#endif
extern s32 t2_read_word(u8 , u8 , u8 *);

extern int get_fuelgauge_current( bool is_sleep, bool avg );
extern void fuelgauge_fullcharged_compensation(u32 is_recharging, u32 pre_update);





static u8 charger_i2c_read( unsigned char reg_addr)
{
    int ret = 0;
    unsigned char buf;

#if defined(USE_GPIO_I2C_CHARGER)
    OMAP_GPIO_I2C_RD_DATA i2c_rd_param;

	if(charger_gpio_i2c_client == NULL)
	{
		printk("charger_gpio_i2c_client i2c is not ready!!");
		return -1;
	}

    i2c_rd_param.reg_len = 1;
    i2c_rd_param.reg_addr = &reg_addr;
    i2c_rd_param.rdata_len = 1;
    i2c_rd_param.rdata = &buf;
    omap_gpio_i2c_read(charger_gpio_i2c_client, &i2c_rd_param);

#elif defined(USE_EXCLUSIVE_I2C_CHARGER)
    struct i2c_msg msg1[1],msg2[1];

    msg1->addr = charger_i2c_client->addr;
    msg1->flags = 0; // I2C_M_WR
    msg1->len = 1;
    msg1->buf = &reg_addr;

    ret = i2c_transfer(charger_i2c_client->adapter, msg1, 1);
    if( ret < 0 )
    {
        printk( KERN_ERR "[FG] fail to read smb136." );
        return -1;
    }
    else
    {
        msg2->addr = charger_i2c_client->addr;
        msg2->flags = I2C_M_RD;
        msg2->len = 1;
        msg2->buf = &buf;

        ret = i2c_transfer( charger_i2c_client->adapter, msg2, 1 );

        if( ret < 0 )
        {
            printk( KERN_ERR "[FG] fail to read smb136." );
            return -1;
        }
    }
#endif

   // ret = buf[0] << 8 | buf[1];

    return buf;

}

static int charger_i2c_write( u8 reg, u8 data)
{
    int ret = 0;
	u8 buf[2];
	
#if defined(USE_GPIO_I2C_CHARGER)
	OMAP_GPIO_I2C_WR_DATA i2c_wr_param;

	if(charger_gpio_i2c_client == NULL)
	{
		printk("charger_gpio_i2c_client i2c is not ready!!");
		return -1;
	}

    i2c_wr_param.reg_addr = &reg;
    i2c_wr_param.reg_len = 1;
    i2c_wr_param.wdata_len = 1;
    i2c_wr_param.wdata = &data;
    omap_gpio_i2c_write(charger_gpio_i2c_client, &i2c_wr_param);
	
#elif defined(USE_EXCLUSIVE_I2C_CHARGER)
	
   	struct i2c_msg msg;
	
    msg.addr = charger_i2c_client->addr;
    msg.flags = 0; //I2C_M_WR
    msg.len = 2;
    
    buf[0] = reg;
    buf[1] = data;
    msg.buf = buf;

    ret = i2c_transfer( charger_i2c_client->adapter, &msg, 1 );

    if( ret < 0 )
    {
        printk( KERN_ERR "[FG] fail to write smb136." );
        return -1;
    }
#endif

    return ret;
}

static u8 charger_i2c_read_sleep( unsigned char reg_addr )
{

#if defined(USE_EXCLUSIVE_I2C_CHARGER)
    unsigned char buf;
    unsigned int ret;

    ret = t2_read_word(charger_i2c_client->addr, reg_addr, &buf);
    if(ret < 0)
    {
        printk(KERN_ERR"[%s] Fail to Read smb136\n", __FUNCTION__);
        return -1;
    } 

    //ret = buf[0] << 8 | buf[1];

    return buf;
	
#elif defined(USE_GPIO_I2C_CHARGER)
	return charger_i2c_read(reg_addr);
#endif

}

static int charger_i2c_write_sleep( u8 reg, u8 data )
{

#if defined(USE_EXCLUSIVE_I2C_CHARGER)
    unsigned int ret;

    ret = t2_write(charger_i2c_client->addr, data, reg);
    if(ret < 0)
    {
        printk(KERN_ERR"[%s] Fail to Write smb136\n", __FUNCTION__);
        return -1;
    } 

    return ret;

#elif defined(USE_GPIO_I2C_CHARGER)
	return charger_i2c_write(reg, data);
#endif
}

int _check_full_charge_dur_sleep_( void )
{
#if 0
    int ret = 0;
    int chg_ing_level = 0;
    int i;
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
        printk( "[TA] Charged!\n" );
        ret = 1;
    }
	else
	{
		ret = 0;
	}
    return ret;
#else
	return is_fullcharged();
#endif
}


static void test_read(void)
{
	u8 data;
	u32 addr;

	for(addr=0;addr<0x0c;addr++)
	{
		data = charger_i2c_read(addr);		
		printk("SMB136 addr : 0x%02x data : 0x%02x\n", addr,data);
	}

	for(addr=0x31;addr<0x3D;addr++)
	{
		data = charger_i2c_read(addr);		
		printk("SMB136 addr : 0x%02x data : 0x%02x\n", addr,data);
	}

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
#ifdef CONFIG_FSA9480_MICROUSB
	return get_real_usbic_state();
#else
	int cable_type = 0;

	cable_type = get_charger_type(false, CHARGE_DUR_ACTIVE);
/*	if(cable_type == MICROUSBIC_USB_CABLE)
		wake_lock(&sec_charger_wakelock);
	else
		wake_lock_timeout(&sec_charger_wakelock, HZ);

//	sec_bci->charger.cable_status = cable_type;*/
    return cable_type;
#endif
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

            status = POWER_SUPPLY_STATUS_NOT_CHARGING;

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
	printk("%s\n", __func__);
	
    if ( is_sleep )
        omap_set_gpio_dataout_in_sleep( KCHG_EN_GPIO, 0 ); 
    else
   	{
		set_charging_current(is_sleep);
        gpio_set_value( KCHG_EN_GPIO, 0 ); 
   	}
    sec_bci->charger.is_charging = true;
	return;
}

static void disable_charging( bool is_sleep )
{
	printk("%s\n", __func__);
    if ( is_sleep )
        omap_set_gpio_dataout_in_sleep( KCHG_EN_GPIO, 1 ); 
    else
        gpio_set_value( KCHG_EN_GPIO, 1 ); 

    sec_bci->charger.is_charging = false;
}

static void set_charging_current( bool is_sleep)
{
	u8 data;
#ifdef CONFIG_FSA9480_MICROUSB
	int n_usbic_state = get_real_usbic_state();
#else
    int n_usbic_state = get_charger_type(false, is_sleep);
#endif

	/* if cable is not samsungcharger, set charging current to 500mA */
	if(!sec_bci->charger.samsung_charger) 
		n_usbic_state = MICROUSBIC_USB_CABLE;

    switch ( n_usbic_state )
    {
        case MICROUSBIC_TA_CHARGER :
			printk("set_charging_current - TA 1500mA\n");
			//1. HC mode
			data= 0x8c;	
			(!is_sleep)?charger_i2c_write(SMB_CommandA, data):charger_i2c_write_sleep(SMB_CommandA, data);
			udelay(10);

			// 2. Change USB5/1/HC Control from Pin to I2C
			(!is_sleep)?charger_i2c_write(SMB_PinControl, 0x8):charger_i2c_write_sleep(SMB_PinControl, 0x8);
			udelay(10);

			(!is_sleep)?charger_i2c_write(SMB_CommandA, 0x8c):charger_i2c_write_sleep(SMB_CommandA, 0x8c);
			udelay(10);

			//3. Set charge current to 1500mA
			data = 0xf4;
			
			(!is_sleep)?charger_i2c_write(SMB_ChargeCurrent, data):charger_i2c_write_sleep(SMB_ChargeCurrent, data);
			udelay(10);
			break;
        case MICROUSBIC_5W_CHARGER :
        case MICROUSBIC_USB_CHARGER :
        case MICROUSBIC_PHONE_USB : 
        case MICROUSBIC_USB_CABLE :
			printk("set_charging_current - USB 500mA\n");
			// 1. USBIN 500mA mode 
			data= 0x88;	

			(!is_sleep)?charger_i2c_write(SMB_CommandA, data):charger_i2c_write_sleep(SMB_CommandA, data);
			udelay(10);

			// 2. Change USB5/1/HC Control from Pin to I2C
			(!is_sleep)?charger_i2c_write(SMB_PinControl, 0x8):charger_i2c_write_sleep(SMB_PinControl, 0x8);
			udelay(10);

			(!is_sleep)?charger_i2c_write(SMB_CommandA, 0x88):charger_i2c_write_sleep(SMB_CommandA, 0x88);
			udelay(10);

			// 3. Set charge current to 500mA
			data = 0x14;
			
			(!is_sleep)?charger_i2c_write(SMB_ChargeCurrent, data):charger_i2c_write_sleep(SMB_ChargeCurrent, data);
			udelay(10);
			break;
		default:
			printk("%s, cable is not connected. charging is not enabled. \n");
			return;
	}

	// 3. Disable Automatic Input Current Limit
	data = 0xe6;
	(!is_sleep)?charger_i2c_write(SMB_InputCurrentLimit, data):charger_i2c_write_sleep(SMB_InputCurrentLimit, data);
	udelay(10);

	//4. Automatic Recharge Disabed 
	data = 0x8c;
	(!is_sleep)?charger_i2c_write(SMB_ControlA, data):charger_i2c_write_sleep(SMB_ControlA, data);
	udelay(10);

	//5. Safty timer Disabled
	data = 0x28;
	(!is_sleep)?charger_i2c_write(SMB_ControlB, data):charger_i2c_write_sleep(SMB_ControlB, data);
	udelay(10);

	//6. Disable USB D+/D- Detection
	data = 0x28;
	(!is_sleep)?charger_i2c_write(SMB_OTGControl, data):charger_i2c_write_sleep(SMB_OTGControl, data);
	udelay(10);

	//7. Set Output Polarity for STAT
	data = 0xCA;
	(!is_sleep)?charger_i2c_write(SMB_FloatVoltage, data):charger_i2c_write_sleep(SMB_FloatVoltage, data);
	udelay(10);

	//8. Re-load Enable
	data = 0x4b;
	(!is_sleep)?charger_i2c_write(SMB_SafetyTimer, data):charger_i2c_write_sleep(SMB_SafetyTimer, data);
	udelay(10);

	//9. Set Fault interrupt register (STAT output)
	#if 0
	data = 0x01;
	(!is_sleep)?charger_i2c_write(SMB_Fault, data):charger_i2c_write_sleep(SMB_Fault, data);
	udelay(10);
	#endif
}

void set_otg_mode(int enable)
{
	u8 data;
	int r_data;

	printk("[SMB136] %s (enable : %d)\n", __func__, enable);

	if(enable)  // Enable OTG Mode
	{
		// 1. Set OTG Mode (Clear Bit5 of Pin Control)
		r_data = charger_i2c_read(SMB_PinControl);
		data = r_data & ~(0x1 << 5);
		charger_i2c_write(SMB_PinControl, data);
		udelay(10);

		// 2. Enable OTG Mode (Set Bit1 of Command Register A)
		r_data = charger_i2c_read(SMB_CommandA);
		data = r_data | (0x1 << 1);
		charger_i2c_write(SMB_CommandA, data);
		udelay(10);
	}
	else  // Re-init charger IC
	{
		// 1. Allow volatile writes to 00~09h, USB 500mA Mode, USB5/1 Mode
		data = 0x88;
		charger_i2c_write(SMB_CommandA, data);
		udelay(10);

		// 2. Change USB5/1/HC Control from Pin to I2C
		data = 0x08;
		charger_i2c_write(SMB_PinControl, data);
		udelay(10);

		// 3. Allow volatile writes to 00~09h, USB 500mA Mode, USB5/1 Mode
		data = 0x88;
		charger_i2c_write(SMB_CommandA, data);
		udelay(10);

		// 4. Disable Automatic Input Current Limit
		data = 0xe6;
		charger_i2c_write(SMB_InputCurrentLimit, data);
		udelay(10);

		//5. Fast Charge Current set 500mA
		data = 0xf4;
		charger_i2c_write(SMB_ChargeCurrent, data);
		udelay(10);

		//6. Automatic Recharge Disabed
		data = 0x8c;
		charger_i2c_write(SMB_ControlA, data);
		udelay(10);

		//7. Safty timer Disabled
		data = 0x28;
		charger_i2c_write(SMB_ControlB, data);
		udelay(10);

		//8. Disable USB D+/D- Detection
		data = 0x28;
		charger_i2c_write(SMB_OTGControl, data);
		udelay(10);

		//9. Set Output Polarity for STAT
		data = 0xca;
		charger_i2c_write(SMB_FloatVoltage, data);
		udelay(10);

		//10. Re-load Enable
		data = 0x4b;
		charger_i2c_write(SMB_SafetyTimer, data);
		udelay(10);
	}
}

int is_already_fullcharged(void)
{
	u8 data=0;
	data = (u8)charger_i2c_read(SMB_StatusE);
	printk("SMB136 - SMB_StatusE data : 0x%02x\n",data);

	if ((data & 0x08) == 0x08) // if error bit check, ignore the status of charger-ic
  		return 0;

	if((data & 0xc0) == 0xc0)	// At least one charge cycle terminated, Charge current < Termination Current
		return 1;
	else
		return 0;
}

int is_fullcharged(void)
{
	u8 data=0;
	data = (u8)charger_i2c_read(SMB_StatusE);
	printk("SMB136 - SMB_StatusE data : 0x%02x\n",data);

	if ((data & 0x08) == 0x08) // if error bit check, ignore the status of charger-ic
  		return 0;

	if(data & 0x40)	// Charge current < Termination Current
		return 1;
	else
		return 0;
}

int is_charging_active(void)
{
	u8 data=0;
	data = (u8)charger_i2c_read(SMB_StatusH);		
	printk("SMB136 addr : 0x39 data : 0x%02x\n",data);

	if(data & 0x01)
		return 1;
	else
		return 0;
}

static bool check_battery_vf( void )
{
//    int count = 0;
//    int val;
    bool ret = false;

	return true;

	#if 0
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
	#endif
	
    return ret;
}

static irqreturn_t cable_detection_isr( int irq, void *_di )
{
    struct charger_device_info *di = _di;

    if ( sec_bci->ready )
    {
		cancel_delayed_work( &di->cable_detection_work );
		if(sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_BATTERY)
			queue_delayed_work( sec_bci->sec_battery_workq, &di->cable_detection_work, HZ/2 );
		else
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

    printk("[TA] cable_detection_work_handler start!!!!\n");

    clear_charge_start_time();

#ifdef CONFIG_FSA9480_MICROUSB
    n_usbic_state = get_real_usbic_state();
#else
    n_usbic_state = get_charger_type(true, CHARGE_DUR_ACTIVE);
#endif

    printk( "[TA] cable_detection_isr handler. usbic_state: %d\n", n_usbic_state );

    switch ( n_usbic_state )
    {
    	case MICROUSBIC_USB_CABLE :
		    wake_lock( &sec_charger_wakelock);
        case MICROUSBIC_5W_CHARGER :
        case MICROUSBIC_TA_CHARGER :
        case MICROUSBIC_USB_CHARGER :
        case MICROUSBIC_PHONE_USB : 
            if ( sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_USB
                || sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_MAINS )
            {
                //printk( "[TA] already Plugged.\n" );
                goto Out_IRQ_Cable_Det;
            }

            /*Check VF*/
            sec_bci->battery.battery_vf_ok = check_battery_vf();

            /*TA or USB is inserted*/
            if ( n_usbic_state == MICROUSBIC_USB_CABLE )
            {
                //current : 482mA
                printk( "[TA] USB CABLE PLUGGED\n" );
                change_cable_status( POWER_SUPPLY_TYPE_USB, di, CHARGE_DUR_ACTIVE );
            }
            else
            {
                //current : 636mA
                printk( "[TA] CHARGER CABLE PLUGGED\n" );
                change_cable_status( POWER_SUPPLY_TYPE_MAINS, di, CHARGE_DUR_ACTIVE );
            }

            qt602240_inform_charger_connection(1);
            
            if ( device_config->SUPPORT_CHG_ING_IRQ && KCHG_ING_IRQ_ENABLE == false)
            {	
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

            if(KCHG_ING_IRQ_ENABLE)
            {
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
			wake_lock_timeout( &sec_charger_wakelock , HZ * 5 );
            qt602240_inform_charger_connection(0);
            break;
    }

Out_IRQ_Cable_Det:
	return;
}

static irqreturn_t full_charge_isr( int irq, void *_di )
{
    struct charger_device_info *di = _di;

    if ( sec_bci->ready)
    {
		if(KCHG_ING_IRQ_ENABLE)
		{
			disable_irq_nosync( KCHG_ING_IRQ );
			KCHG_ING_IRQ_ENABLE = false;
		}
        cancel_delayed_work( &di->full_charge_work );
        queue_delayed_work( sec_bci->sec_battery_workq, &di->full_charge_work, 2*HZ );
    }

    return IRQ_HANDLED;
}

static void full_charge_work_handler( struct work_struct *work )
{
    struct charger_device_info *di;
    int count;
    int n_usbic_state;

	printk( "[TA] Full_charge_work_handler!\n" );

    if ( !sec_bci->charger.is_charging )
        goto Enable_IRQ_Full_Det;

    // Temporary remove
 #ifdef CONFIG_FSA9480_MICROUSB
    n_usbic_state = get_real_usbic_state();
 #else
	n_usbic_state = get_charger_type(false, CHARGE_DUR_ACTIVE);
 #endif
    
 
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

    if ( device_config->SUPPORT_CHG_ING_IRQ )
    {
    	// is_fullcharged() check
        if (is_fullcharged() && sec_bci->charger.is_charging && 
			(sec_bci->battery.battery_level_ptg >= 90 || sec_bci->battery.battery_level_vol >= 4050))
        {
        	int is_recharging;
			if(sec_bci->charger.charge_status == POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL)
				is_recharging = 1;
			else
				is_recharging = 0;
			
            printk( "[TA] Charge FULL!\n" );
            change_charge_status( POWER_SUPPLY_STATUS_FULL, CHARGE_DUR_ACTIVE );
			fuelgauge_fullcharged_compensation(is_recharging, 1);
			cancel_delayed_work( &di->full_comp_work );
			queue_delayed_work( sec_bci->sec_battery_workq, &di->full_comp_work, HZ );			
        }
    }
    
Enable_IRQ_Full_Det :
	if ( KCHG_ING_IRQ_ENABLE == false)
	{	
		enable_irq( KCHG_ING_IRQ );
		KCHG_ING_IRQ_ENABLE = true;
	}

    return;
}

static void full_comp_work_handler( struct work_struct *work )
{
	int avg_current = get_fuelgauge_current(false, true);  //bool is_sleep, bool avg
	int is_recharging;
	struct charger_device_info *di;

	di = container_of( work, struct charger_device_info, full_comp_work.work );

	if(sec_bci->charger.charge_status == POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL)
		is_recharging = 1;
	else
		is_recharging = 0;

	if(avg_current >= 25) // // Real threshold is 25.625mA
	{
		printk("%s : full charge compensation wait(avg_current : %d)\n", __func__, avg_current);
		cancel_delayed_work( &di->full_comp_work );
		queue_delayed_work( sec_bci->sec_battery_workq, &di->full_comp_work, HZ ); // after 1 second.
	}
	else
	{
		printk("%s : full charge compensation start (avg_current : %d)\n", __func__, avg_current);
		fuelgauge_fullcharged_compensation(is_recharging, 0);
	}
	
	return;
}
	


#define STS_VBUS		0x80
#define STS_USB			0x04
#define STS_CHG			0x02
#define NO_PW_CONN		0
#define AC_PW_CONN		0x01
#define USB_PW_CONN		0x02

extern int get_adc_conversion(int ch);
extern int get_id_ver(void);
extern int usb_pres;



#ifndef CONFIG_FSA9480_MICROUSB
static int get_charger_type(bool is_first_detection, bool is_sleep)
{
	u8 ret = 0;
	u8 hwsts = 0;
	u8 val = 0;

	ret = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &hwsts, 0x0F /*REG_STS_HW_CONDITIONS*/);
	if (ret) {
		pr_err("[TA] get_charger_type: error reading STS_HW_CONDITIONS\n");
		return ret;
	}

	if(hwsts & STS_VBUS)
	{
		if(usb_pres)
		{
			ret = USB_PW_CONN;
		}
		else
		{
			int adc_mvol;
			int usbmenu;

			//usbmenu = get_usbmenupath_value();
			ret = AC_PW_CONN;

			gpio_set_value(OMAP_GPIO_USB_SEL_1, 0); // UART_SEL1
			gpio_set_value(OMAP_GPIO_USB_SEL_2, 0); // UART_SEL2

			if(is_sleep)
				mdelay(50);
			else
				msleep(50);

			adc_mvol = get_adc_conversion(AP_ADC_CHECK_1);
			printk("[TA] get_charger_type : adc mvol = %d\n", adc_mvol);
			
			if(adc_mvol >= 950 && adc_mvol < 1600)
			{
				sec_bci->charger.samsung_charger = true;
			}
			else
			{
				sec_bci->charger.samsung_charger = false;
				//if(is_first_detection)
				//	ret = USB_PW_CONN;
			}
			
			gpio_set_value(OMAP_GPIO_USB_SEL_1, 1); // UART_SEL1
			gpio_set_value(OMAP_GPIO_USB_SEL_2, 0); // UART_SEL2
			/*
			if(usbmenu & 0x1) //AP
			{
				gpio_set_value(OMAP_GPIO_USB_SEL_1, 1); // UART_SEL1
				gpio_set_value(OMAP_GPIO_USB_SEL_2, 0); // UART_SEL2
			}
			else // CP
			{
				gpio_set_value(OMAP_GPIO_USB_SEL_1, 0); // UART_SEL1
				gpio_set_value(OMAP_GPIO_USB_SEL_2, 1); // UART_SEL2
			}
			*/
		}
	}
	else
	{
		usb_pres = 0;
		sec_bci->charger.samsung_charger = false;
	}

	if(ret == AC_PW_CONN)
		return MICROUSBIC_TA_CHARGER;
	else if(ret == USB_PW_CONN)
		return MICROUSBIC_USB_CABLE;
	else 
		return MICROUSBIC_NO_DEVICE;
}
#endif



#ifdef USE_EXCLUSIVE_I2C_CHARGER

//static int __devinit charger_i2c_probe( struct i2c_client *client, const struct i2c_device_id *id )
static int charger_i2c_probe( struct i2c_client *client, const struct i2c_device_id *id )
{
    int ret = 0;

	printk("charger_i2c_probe\n");
	charger_i2c_client = client;
	if(charger_i2c_client == NULL)
	{
		printk("charger_probe fail : charger_i2c_client is null\n");
		return 0;
	}
#if (defined( CONFIG_MACH_SAMSUNG_P1LITE ) && ( CONFIG_SAMSUNG_REL_HW_REV == 2 ) )
	{
		extern u32 hw_revision;
		if(hw_revision == 0x7){
			charger_i2c_client->addr = 0x5d;
			printk("%s, change charger i2c address -> 0x5d, hw_revision=%x\n", __func__, hw_revision);
		}
	}
#endif	
	test_read();
	return ret;
}
static int charger_i2c_remove( struct i2c_client * client ){    return 0;}

static const struct i2c_device_id charger_i2c_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};

static struct i2c_driver charger_i2c_driver =
{
    .probe = charger_i2c_probe,
	.remove     = __devexit_p( charger_i2c_remove ),
//	.suspend    = charger_i2c_suspend,
//	.resume     = charger_i2c_resume,
    .driver = {
        .name = DRIVER_NAME,
    },       
    .id_table = charger_i2c_id,
};
#endif

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

	this_dev = &pdev->dev; 
	 
    /*Init Work*/
    INIT_DELAYED_WORK( &di->cable_detection_work, cable_detection_work_handler );
    INIT_DELAYED_WORK( &di->full_charge_work, full_charge_work_handler );
    INIT_DELAYED_WORK( &di->full_comp_work, full_comp_work_handler );

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

	{
	    u8 val = 0; 

	    twl_i2c_read_u8( TWL4030_MODULE_INT, &val, 0x05/*REG_PWR_EDR1*/ );
		twl_i2c_write_u8( TWL4030_MODULE_INT, 0xFF , 0x05 /*REG_PWR_EDR1*/ );
	    twl_i2c_read_u8( TWL4030_MODULE_INT, &val, 0x05/*REG_PWR_EDR1*/ );
	}

   	KUSB_CONN_IRQ = platform_get_irq( pdev, 0 ); 
	if(KUSB_CONN_IRQ)
	{
		ret = request_irq( KUSB_CONN_IRQ, cable_detection_isr, IRQF_SHARED, pdev->name, di );
		if(ret)
		{
			printk( "[TA] 1. could not request irq %d, status %d\n", KUSB_CONN_IRQ, ret );
			goto usb_irq_fail;
		}
		set_irq_type( KUSB_CONN_IRQ, IRQ_TYPE_EDGE_BOTH );
	}
#if 0
	KTA_NCONN_IRQ = platform_get_irq( pdev, 1 );
    ret = request_irq( KTA_NCONN_IRQ, cable_detection_isr, IRQF_SHARED, pdev->name, di );
    if(ret)
    {
        printk( "[TA] 2. could not request irq %d, status %d\n", KTA_NCONN_IRQ, ret );
        goto ta_irq_fail;
    }

	set_irq_type( KTA_NCONN_IRQ, IRQ_TYPE_EDGE_BOTH );
#endif
    if ( device_config->SUPPORT_CHG_ING_IRQ )
	{
		KCHG_ING_IRQ = platform_get_irq( pdev, 2 );
		KCHG_ING_GPIO = irq_to_gpio( KCHG_ING_IRQ );
		printk( "[TA] CHG_ING IRQ : %d \n", KCHG_ING_IRQ );
		printk( "[TA] CHG_ING GPIO : %d \n", KCHG_ING_GPIO );

		ret = request_irq( KCHG_ING_IRQ, full_charge_isr, IRQF_DISABLED, pdev->name, di );
		set_irq_type( KCHG_ING_IRQ, IRQ_TYPE_EDGE_RISING );

		if(ret)
		{
			printk( "[TA] 3. could not request irq2 status %d\n", ret );
			goto chg_full_irq_fail;
		}
		disable_irq( KCHG_ING_IRQ );
		KCHG_ING_IRQ_ENABLE = false;
	}
	KCHG_EN_GPIO = irq_to_gpio( platform_get_irq( pdev, 3 ) );
    queue_delayed_work( sec_bci->sec_battery_workq, &di->cable_detection_work, HZ );

	if (gpio_request(OMAP_GPIO_USB_SEL_1, "USB_SEL_1"))
	{
		printk(KERN_ERR "Filed to request OMAP_GPIO_USB_SEL_1!\n");
	}
	gpio_direction_output(OMAP_GPIO_USB_SEL_1, 1);

	if (gpio_request(OMAP_GPIO_USB_SEL_2, "USB_SEL_2"))
	{
		printk(KERN_ERR "Filed to request OMAP_GPIO_USB_SEL_2!\n");
	}
	gpio_direction_output(OMAP_GPIO_USB_SEL_2, 1);
	printk("charger_probe set usb_sel\n");

//	gpio_direction_output(OMAP_GPIO_USB_SEL_1, 1);
//	gpio_direction_output(OMAP_GPIO_USB_SEL_2, 0);

#if defined(USE_GPIO_I2C_CHARGER)
	test_read();
#endif

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

static int charger_remove( struct platform_device *pdev )
{

    struct charger_device_info *di = platform_get_drvdata( pdev );

    free_irq( KUSB_CONN_IRQ, di );
    free_irq( KTA_NCONN_IRQ, di );

    flush_scheduled_work();

    // USE_REGULATOR [+]
    regulator_put( di->usb1v5 );
    regulator_put( di->usb1v8 );
    regulator_put( di->usb3v1 );
    // USE_REGULATOR [-]

    kfree( di );

    return 0;
}

static int charger_suspend( struct platform_device *pdev, pm_message_t state )
{
    return 0;
}

static int charger_resume( struct platform_device *pdev )
{
    return 0;
}

static int charger_shutdown( struct platform_device *pdev )
{
	disable_charging(false);
    return 0;
}

struct platform_driver charger_platform_driver = {    
	.probe      = &charger_probe,
	.remove     = __devexit_p( charger_remove ),
	.suspend    = &charger_suspend,
	.resume     = &charger_resume,
	.shutdown	= &charger_shutdown,
	.driver     = {
		.name = DRIVER_NAME,
	},
};

int charger_init( void )
{
    int ret;

#if defined(USE_EXCLUSIVE_I2C_CHARGER)
	printk("%s, i2c driver register!\n", __func__);
    if( ( ret = i2c_add_driver( &charger_i2c_driver ) < 0 ) )
    {
        printk( KERN_ERR "[CHARGER] i2c_add_driver failed.\n" );
    }
	printk("[CHARGER] i2c_add_driver result = %d\n", ret);
	
#elif defined(USE_GPIO_I2C_CHARGER)
	charger_gpio_i2c_client = omap_gpio_i2c_init(
		OMAP_GPIO_SENSOR_I2C_SDA, OMAP_GPIO_SENSOR_I2C_SCL, CHARGER_SLAVE_ADDR, 100);
#endif

	printk("%s, platform driver register!\n", __func__);
	ret = platform_driver_register( &charger_platform_driver );

	wake_lock_init( &sec_charger_wakelock, WAKE_LOCK_SUSPEND, "samsung-charger" );

    return ret;
}

void charger_exit( void )
{
#if defined(USE_EXCLUSIVE_I2C_CHARGER)
    i2c_del_driver( &charger_i2c_driver );

#elif defined(USE_GPIO_I2C_CHARGER)
	omap_gpio_i2c_deinit(charger_gpio_i2c_client);
#endif
}
