/*
 * qt602240_ts.c - AT42QT602240 Touchscreen driver
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

/*
 * TODO:
 * - Multi touch support
 * - Gesture support
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/qt602240_ts.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <asm/gpio.h>
#include <plat/gpio.h>
#include <linux/leds.h>
#include "qt602240.h"
//#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <plat/omap-pm.h>	// ryun 20200107 for touch boost
#include <linux/mutex.h>

/******************************************************************************
*
*
*       QT602240 Object table init
*
*
* *****************************************************************************/
//General Object
gen_powerconfig_t7_config_t power_config = {0};                 //Power config settings.
gen_acquisitionconfig_t8_config_t acquisition_config = {0};     // Acquisition config.

//Touch Object
touch_multitouchscreen_t9_config_t touchscreen_config = {0};    //Multitouch screen config.
touch_keyarray_t15_config_t keyarray_config = {0};              //Key array config.
touch_proximity_t23_config_t proximity_config = {0};        //Proximity config.

//Signal Processing Objects
proci_gripfacesuppression_t20_config_t gripfacesuppression_config = {0};    //Grip / face suppression config.
procg_noisesuppression_t22_config_t noise_suppression_config = {0};         //Noise suppression config.
proci_onetouchgestureprocessor_t24_config_t onetouch_gesture_config = {0};  //One-touch gesture config.
proci_twotouchgestureprocessor_t27_config_t twotouch_gesture_config = {0};  //Two-touch gesture config.

//Support Objects
spt_gpiopwm_t19_config_t  gpiopwm_config = {0};             //GPIO/PWM config
spt_selftest_t25_config_t selftest_config = {0};            //Selftest config.
spt_cteconfig_t28_config_t cte_config = {0};                //Capacitive touch engine config.

spt_comcconfig_t18_config_t   comc_config = {0};            //Communication config settings.
gen_commandprocessor_t6_config_t    command_config = {0};

static report_finger_info_t fingerInfo[MAX_USING_FINGER_NUM];

static unsigned int set_mode_for_ta = 0;        // 2: boot mode 1: TA or USB, 0: normal
static int set_mode_for_amoled = 0;        //0: TFt-LCD, 1: AMOLED
static int gFirmware_Update_State = FW_UPDATE_READY;
#if defined (CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
static bool gbfilter =false;
#endif

//extern symbol from mach-p1.c
//#define CMC623_TUNING
extern struct class *sec_class;

extern u32 hw_revision;

extern int max8998_ldo_enable_direct(int ldo);
extern int max8998_ldo_disable_direct(int ldo);


struct qt602240_data * p_qt602240_data;
#ifdef CONFIG_HAS_EARLYSUSPEND
static void qt602240_early_suspend(struct early_suspend *);
static void qt602240_late_resume(struct early_suspend *);
#endif    /* CONFIG_HAS_EARLYSUSPEND */

static bool cal_check_flag = false;
static unsigned int qt_time_point=0;
static unsigned int qt_time_diff=0;
static unsigned int qt_timer_state =0;
static unsigned int is_suspend_state = 0; // to check suspend mode
static unsigned int is_boot_state = 1; // to check boot mode
static unsigned int qt60224_notfound_flag=1;

static int key_led_en_gpio;

static DECLARE_MUTEX(g_tsp_mutex);
static struct workqueue_struct *tsp_wq;

#ifdef TOUCH_BOOST
// OMAP3630 OPP Clock Frequency Table
#define VDD1_OPP4_FREQ         S1000M
#define VDD1_OPP3_FREQ         S800M
#define VDD1_OPP2_FREQ         S600M
#define VDD1_OPP1_FREQ         S300M

unsigned short enable_touch_boost;
unsigned short touch_boost_state;

struct timer_list opp_set_timer;	
struct work_struct constraint_wq;
static ssize_t touch_boost_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t touch_boost_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);

static DEVICE_ATTR(touch_boost, S_IRUGO | S_IWUSR | S_IWGRP, touch_boost_show, touch_boost_store);

static ssize_t touch_boost_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (attr == &dev_attr_touch_boost.attr)
		return sprintf(buf, "%hu\n", enable_touch_boost);
	
	else
		return -EINVAL;

}
static ssize_t touch_boost_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1) {
		printk(KERN_ERR "ts_store: Invalid value\n");
		return -EINVAL;
	}

	if (attr == &dev_attr_touch_boost.attr) {
		enable_touch_boost = value;
	} 

	 else {
		return -EINVAL;
	}

	return size;
}

static void tsc_timer_out (unsigned long v)
{
    schedule_work(&constraint_wq);
    return;
}

void tsc_remove_constraint_handler(struct work_struct *work)
{
    omap_pm_set_min_mpu_freq((struct device *)p_qt602240_data->input_dev, VDD1_OPP1_FREQ);
    touch_boost_state = 0;
}
#endif

static ssize_t bootcomplete_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static int qt602240_write_object(struct qt602240_data *data, u8 type, u8 offset, u8 val);
void qt602240_inform_charger_connection(int mode);

void set_autocal(uint8_t value)
{
	static int g_value = 0;
	int error;
	if (down_interruptible(&g_tsp_mutex))
		return;

	if(g_value != value) {
		g_value = value;
        acquisition_config.tchautocal = value;
		error = qt602240_write_object(p_qt602240_data, QT602240_GEN_ACQUIRE,
			QT602240_ACQUIRE_TCHAUTOCAL, value);
		if (error < 0)
		{
			printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
		}
		else {
		    if(value)
			    printk(KERN_DEBUG "[TSP] autocalibration enabled : %d\n", value);
		    else
			    printk(KERN_DEBUG "[TSP] autocalibration disabled\n");			
		}
	}
	up(&g_tsp_mutex);
}

void force_disable_tsp_autocal(void)
{
	int error;

	if((!is_boot_state && !is_suspend_state) && !qt60224_notfound_flag)
	{
		if (down_interruptible(&g_tsp_mutex))
			return;

		error = qt602240_write_object(p_qt602240_data, QT602240_GEN_ACQUIRE,
			QT602240_ACQUIRE_TCHAUTOCAL, 0);
		if (error < 0)
		{
			printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
		}

		printk(KERN_DEBUG "[TSP] autocalibration force disabled\n");

		up(&g_tsp_mutex);
	}
}
EXPORT_SYMBOL(force_disable_tsp_autocal);

void calibrate_chip(struct qt602240_data *data);
void bootcomplete(void)
{
	if(qt60224_notfound_flag == 1) return;
	is_boot_state = 0;
	qt602240_inform_charger_connection(set_mode_for_ta);
	set_autocal(0);
	calibrate_chip(p_qt602240_data); // workaround for lock screen TSP lockup
	printk(KERN_DEBUG "[TSP] set boot threshold = %d\n", touchscreen_config.tchthr);
}

static DEVICE_ATTR(bootcomplete, 0220, NULL, bootcomplete_store);
static ssize_t bootcomplete_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1) {
		printk(KERN_ERR "bootcomplete_store: Invalid value\n");
		return -EINVAL;
	}

	if (attr == &dev_attr_bootcomplete) {
		bootcomplete();
	} 
	else {
		return -EINVAL;
	}

	return size;
}

struct work_struct autocal_work;
struct timer_list autocal_timer;

void disable_autocal(struct work_struct *work)
{
	if((!is_boot_state && !is_suspend_state) && !qt60224_notfound_flag)
	{
		set_autocal(0);
	}
}

static void autocal_timer_out (unsigned long v)
{
	schedule_work(&autocal_work);
	return;
}

void enable_autocal_timer(unsigned int value)
{
	if((!is_boot_state && !is_suspend_state) && !qt60224_notfound_flag)
	{
		set_autocal(value);
		mod_timer(&autocal_timer, jiffies + (3 * HZ));
	}
}

#if defined (KEY_LED_CONTROL)
void led_control(int data)
{
    int i;

    for(i = 0; i < data; i++)
    {
        gpio_set_value(key_led_en_gpio, 1);
        udelay(10);
        gpio_set_value(key_led_en_gpio, 0);
        udelay(10);
    }
    gpio_set_value(key_led_en_gpio, 1);

    //wait for the mode change
    udelay(700);
}

void led_control_INTLOCK(int data)
{
    int i;

    data = data -1;

    local_irq_disable();
    gpio_set_value(key_led_en_gpio, 0);
    udelay(2);

    for(i = 0; i < data; i++)
    {
        gpio_set_value(key_led_en_gpio, 1);
        udelay(2);
        gpio_set_value(key_led_en_gpio, 0);
        udelay(2);
    }
    gpio_set_value(key_led_en_gpio, 1);
    local_irq_enable();

    //wait for the mode change
    mdelay(1);
}

void init_led(void)
{
#if ( (defined( CONFIG_MACH_SAMSUNG_P1LITE ) || defined( CONFIG_MACH_SAMSUNG_P1WIFI )) && ( CONFIG_SAMSUNG_REL_HW_REV >= 3 ) )
    if(hw_revision < 4)
        key_led_en_gpio = OMAP_GPIO_ACCESSORY_EN;
    else
#endif
        key_led_en_gpio = KEYLED_EN;
    if(gpio_is_valid(key_led_en_gpio))
    {
        gpio_request(key_led_en_gpio, "KEYLED_EN");
        gpio_direction_output(key_led_en_gpio, 0);
    }
}

void touch_led_on(int val)
{
#ifndef CONFIG_TARGET_LOCALE_KOR
    static int preset = 0;
    int set = 2;
    printk(KERN_DEBUG "[TSP] keyled : %d \n", val );

    if(val < 42)
        set = 1;

    if(val > 0)
    {
        if(set !=preset)
        {
            //KEYLED is only working in low current mode.
            led_control_INTLOCK(16);
            led_control_INTLOCK(KEYLED_ADDRESS_MAX);        // [Address ] Max current setting
            led_control_INTLOCK(KEYLED_DATA_LOW);           // [Data] Low current mode
            led_control_INTLOCK(KEYLED_ADDRESS_LOW);        // [Address] Low current mode
            if(set == 1)
            {
                led_control_INTLOCK(2);                     // [Data] 0.5mA
                preset = 1;
                printk(KERN_DEBUG "[TSP] keyled : 0.5mA\n");
            }
            else
            {
                led_control_INTLOCK(4);                     // [Data] 2mA
                preset = 2;
                printk(KERN_DEBUG "[TSP] keyled : 2mA\n");
            }
        }
    }
    else
    {
        gpio_set_value(key_led_en_gpio, 0);
        preset = 0;
    }
#else
    static bool bLedOn = false;
    if(val > 0)
    {
        if(!bLedOn)
        {
            led_control_INTLOCK(16);
            led_control_INTLOCK(KEYLED_ADDRESS_CURRENT);      // [Address] current level setting
            led_control_INTLOCK(15);                          // [Data] 15th level

            led_control_INTLOCK(KEYLED_ADDRESS_MAX);          // [Address ] Max current setting
            led_control_INTLOCK(3);                           // [Data] 15ma MAX mode
            bLedOn = true;
        }
    }
    else
    {
        gpio_set_value(key_led_en_gpio, 0);
        bLedOn = false;
    }
#endif
}


static ssize_t key_led_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size)
{
    int i = 0;
    if(sscanf(buf,"%d",&i) !=1 )
    {
        printk(KERN_ERR"[TSP] keyled write error\n");
    }

    touch_led_on(i);

    return size;
}
static DEVICE_ATTR(brightness, S_IWUSR | S_IWGRP, NULL, key_led_store);

#endif      //KEY_LED_CONTROL


#if defined(DRIVER_FILTER)
#if defined (CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
static void equalize_coordinate(bool detect, u8 id, u16 *px, u16 *py)
{
    static int tcount[MAX_USING_FINGER_NUM] = { 0, };
    static u16 pre_x[MAX_USING_FINGER_NUM][4] = {{0}, };
    static u16 pre_y[MAX_USING_FINGER_NUM][4] = {{0}, };
    int coff[4] = {0,};
    int distance = 0;

    if(detect)
    {
        tcount[id] = 0;
    }

    pre_x[id][tcount[id]%4] = *px;
    pre_y[id][tcount[id]%4] = *py;

    if(gbfilter)
    {
         if(tcount[id] >3)
        {
            *px = (u16)((*px + pre_x[id][(tcount[id]-1)%4] + pre_x[id][(tcount[id]-2)%4] + pre_x[id][(tcount[id]-3)%4] )/4);
            *py = (u16)((*py+ pre_y[id][(tcount[id]-1)%4] + pre_y[id][(tcount[id]-2)%4]+ pre_y[id][(tcount[id]-3)%4])/4);
        }
        else switch(tcount[id])
        {
            case 2:
            {
                *px = (u16)((*px + pre_x[id][(tcount[id]-1)%4])>>1);
                *py = (u16)((*py + pre_y[id][(tcount[id]-1)%4])>>1);
                break;
            }

            case 3:
            {
                *px = (u16)((*px + pre_x[id][(tcount[id]-1)%4] + pre_x[id][(tcount[id]-2)%4])/3);
                *py = (u16)((*py + pre_y[id][(tcount[id]-1)%4] + pre_y[id][(tcount[id]-2)%4])/3);
                break;
            }

            default:
                break;
        }

    }
    else if(tcount[id] >3)
    {
        {
            distance = abs(pre_x[id][(tcount[id]-1)%4] - *px) + abs(pre_y[id][(tcount[id]-1)%4] - *py);

            coff[0] = (u8)(2 + distance/5);
            if(coff[0] < 8)
            {
                coff[0] = max(2, coff[0]);
                coff[1] = min((8 - coff[0]), (coff[0]>>1)+1);
                coff[2] = min((8 - coff[0] - coff[1]), (coff[1]>>1)+1);
                coff[3] = 8 - coff[0] - coff[1] - coff[2];

                *px = (u16)((*px*(coff[0]) + pre_x[id][(tcount[id]-1)%4]*(coff[1])
                    + pre_x[id][(tcount[id]-2)%4]*(coff[2]) + pre_x[id][(tcount[id]-3)%4]*(coff[3]))/8);
                *py = (u16)((*py*(coff[0]) + pre_y[id][(tcount[id]-1)%4]*(coff[1])
                    + pre_y[id][(tcount[id]-2)%4]*(coff[2]) + pre_y[id][(tcount[id]-3)%4]*(coff[3]))/8);
            }
            else
            {
                *px = (u16)((*px*4 + pre_x[id][(tcount[id]-1)%4])/5);
                *py = (u16)((*py*4 + pre_y[id][(tcount[id]-1)%4])/5);
            }
        }
     }
    tcount[id]++;
}

#else   //CONFIG_TARGET_LOCALE_KOR
static void equalize_coordinate(bool detect, u8 id, u16 *px, u16 *py)
{
    static int tcount[MAX_USING_FINGER_NUM] = { 0, };
    static u16 pre_x[MAX_USING_FINGER_NUM][4] = {{0}, };
    static u16 pre_y[MAX_USING_FINGER_NUM][4] = {{0}, };
    int coff[4] = {0,};
    int distance = 0;

    if(detect)
    {
        tcount[id] = 0;
    }

    pre_x[id][tcount[id]%4] = *px;
    pre_y[id][tcount[id]%4] = *py;

    if(tcount[id] >3)
    {
        distance = abs(pre_x[id][(tcount[id]-1)%4] - *px) + abs(pre_y[id][(tcount[id]-1)%4] - *py);

        coff[0] = (u8)(4 + distance/5);
        if(coff[0] < 8)
        {
            coff[0] = max(4, coff[0]);
            coff[1] = min((10 - coff[0]), (coff[0]>>1)+1);
            coff[2] = min((10 - coff[0] - coff[1]), (coff[1]>>1)+1);
            coff[3] = 10 - coff[0] - coff[1] - coff[2];

            *px = (u16)((*px*(coff[0]) + pre_x[id][(tcount[id]-1)%4]*(coff[1])
                + pre_x[id][(tcount[id]-2)%4]*(coff[2]) + pre_x[id][(tcount[id]-3)%4]*(coff[3]))/10);
            *py = (u16)((*py*(coff[0]) + pre_y[id][(tcount[id]-1)%4]*(coff[1])
                + pre_y[id][(tcount[id]-2)%4]*(coff[2]) + pre_y[id][(tcount[id]-3)%4]*(coff[3]))/10);
        }
        else
        {
            *px = (u16)((*px*4 + pre_x[id][(tcount[id]-1)%4])/5);
            *py = (u16)((*py*4 + pre_y[id][(tcount[id]-1)%4])/5);
        }
    }

    tcount[id]++;
}
#endif
#endif  //DRIVER_FILTER

static void release_all_fingers(struct input_dev *input_dev)
{
    int i;
    for ( i= 0; i<MAX_USING_FINGER_NUM; ++i )
    {
        if ( fingerInfo[i].pressure == -1 )
            continue;

        fingerInfo[i].pressure = 0;

        input_report_abs(input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
        input_report_abs(input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
        input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].pressure);
        input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].width);
        input_report_abs(input_dev, ABS_MT_TRACKING_ID, i);
        input_mt_sync(input_dev);

        if ( fingerInfo[i].pressure == 0 )
            fingerInfo[i].pressure= -1;
    }

    input_sync(input_dev);
}

static struct qt602240_object * qt602240_get_object(struct qt602240_data *data, u8 type)
{
    struct qt602240_object *object;
    int i;

    for (i = 0; i < data->info->object_num; i++)
    {
        object = data->object_table + i;
        if (object->type == type)
        {
            return object;
        }
    }

    dev_err(&data->client->dev, "invalid type\n");
    return NULL;
}

int qt602240_reg_write(struct qt602240_data *qt602240data,  u8 type, u8 *values)
{
    int i;
    unsigned char data[I2C_MAX_SEND_LENGTH];
    struct i2c_msg wmsg;
    struct qt602240_object *object;

    object = qt602240_get_object(qt602240data, type);
    if (!object)
        printk(KERN_ERR"[TSP] error : 0x%x\n", object->type);

    if(object->size > ( I2C_MAX_SEND_LENGTH - 2 ))
        printk(KERN_ERR"[TSP][ERROR] %s() data length error\n", __FUNCTION__);

    wmsg.addr = qt602240data->client->addr;
    wmsg.flags = I2C_M_WR;
    wmsg.len = (object->size + 3);
    wmsg.buf = data;

    data[0] = object->start_address & 0x00ff;
    data[1] = object->start_address >> 8;

    for (i = 0; i < object->size + 1; i++)
    {
        data[i+2] = *(values+i);
    }

    return (i2c_transfer(qt602240data->client->adapter, &wmsg, 1));
}

int QT602240_Command_Config_Init(struct qt602240_data *data)
{
    command_config.reset = 0x0;
    command_config.backupnv = 0x0;
    command_config.calibrate = 0x0;
    command_config.reportall = 0x0;
    command_config.reserve= 0x0;
    command_config.diagnostic = 0x0;

    return (qt602240_reg_write(data, GEN_COMMANDPROCESSOR_T6, (void *) &command_config));
}

int QT602240_Powr_Config_Init(struct qt602240_data *data)
{
    power_config.idleacqint = 32;    // 32ms in idle status
    power_config.actvacqint = 0xff;  // free run in active status
    power_config.actv2idleto = 0x32; // 10s
    return (qt602240_reg_write(data, GEN_POWERCONFIG_T7, (void *) &power_config));
}

int QT602240_Acquisition_Config_Init(struct qt602240_data *data)
{
    acquisition_config.chrgtime = 8;               //2us       Charge-transfer dwell time
    acquisition_config.reserved = 0x00;
    acquisition_config.tchdrift = 1;               // 1s              Touch drift time
    acquisition_config.driftst = 1;                // 1 cycle        Drift suspend time
    acquisition_config.tchautocal = 0x00;          // infinite        Touch Automatic Calibration
    acquisition_config.sync = 0x00;                // disabled
    acquisition_config.atchcalst = 0x04;           // 1800ms      Anti-touch calibration time
    acquisition_config.atchcalsthr = 20;         // Anti-touch Calibration suspend threshold

    return (qt602240_reg_write(data, GEN_ACQUISITIONCONFIG_T8, (void *) &acquisition_config));
}

int QT602240_Multitouch_Config_Init(struct qt602240_data *data)
{
    //0x80 :Scan en
    //0x8 : Disable vector change, 0x2: Enable reporting, 0x1 : Enable the multi-touch
#if defined(_SUPPORT_TOUCH_AMPLITUDE_)
    touchscreen_config.ctrl = 0x8b;         //Enable amplitude change : 0x0 << 3
#else
    touchscreen_config.ctrl = 0x8f;         //Disable amplitude change : 0x1 << 3
#endif
    touchscreen_config.xorigin = 0x00;
    touchscreen_config.yorigin = 0x00;

    touchscreen_config.xsize = 0x12;
    touchscreen_config.ysize = 0x0b;
    touchscreen_config.akscfg = 1;
    touchscreen_config.blen = 0x00;         // Gain of the analog circuits in front of the ADC [7:4]
    touchscreen_config.tchthr = 28;         // touch Threshold value
    touchscreen_config.orient = 0x04;       // 0x4 : Invert Y, 0x2 : Invert X, 0x1 : Switch

    touchscreen_config.mrgtimeout = 0x00;
    touchscreen_config.movhysti = 16;       // Move hysteresis, initial
    touchscreen_config.movhystn = 10;       // Move hysteresis, next
#if defined(CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
    touchscreen_config.movfilter = 0x00;    // Filter Limit[6:4] , Adapt threshold [3:0]
#else
    touchscreen_config.movfilter = 0x0b;    // Filter Limit[6:4] , Adapt threshold [3:0]
#endif    
    touchscreen_config.numtouch= 0x05;
    touchscreen_config.tchdi = 0x02;
    touchscreen_config.mrghyst = 0x5;       // Merge hysteresis
    touchscreen_config.mrgthr = 0x5;        // Merge threshold
    touchscreen_config.amphyst = 0x0a;      // Amplitude hysteresis
    touchscreen_config.xrange1= 0xff;   
    touchscreen_config.xrange2= 0x03;

    touchscreen_config.yrange1 = 0x57;  
    touchscreen_config.yrange2 = 0x02;
    touchscreen_config.xloclip = 0x00;
    touchscreen_config.xhiclip = 0x00;
    touchscreen_config.yloclip = 0x00;
    touchscreen_config.yhiclip = 0x00;
    touchscreen_config.xedgectrl = 0x00;
    touchscreen_config.xedgedist = 0x00;
    touchscreen_config.yedgectrl = 0x00;
    touchscreen_config.yedgedist = 0x00;
#ifdef CONFIG_TARGET_LOCALE_USAGSM    
    touchscreen_config.jumplimit = 18; 
#else
    touchscreen_config.jumplimit = 10;            // ??*8
#endif

    return (qt602240_reg_write(data, TOUCH_MULTITOUCHSCREEN_T9, (void *) &touchscreen_config));
}

int QT602240_KeyArrary_Config_Init(struct qt602240_data *data)
{

    keyarray_config.ctrl = 0x3;
    keyarray_config.xorigin = 0x00;
    keyarray_config.yorigin = 0x0b;
    keyarray_config.xsize = 0x04;
    keyarray_config.ysize = 0x01;
    keyarray_config.akscfg = 1;
    keyarray_config.blen = 0x00;
#ifdef CONFIG_TARGET_LOCALE_USAGSM
    keyarray_config.tchthr = 15;        //25
#else
    keyarray_config.tchthr = 20;        //25
#endif
    keyarray_config.tchdi = 3;      

    keyarray_config.reserved[0] = 0;
    keyarray_config.reserved[1] = 0;

    return (qt602240_reg_write(data, TOUCH_KEYARRAY_T15, (void *) &keyarray_config));
}

int QT602240_GPIOPWM_Config_Init(struct qt602240_data *data)
{
    gpiopwm_config.ctrl = 0;
    gpiopwm_config.reportmask = 0;
    gpiopwm_config.dir = 0;
    gpiopwm_config.intpullup = 0;
    gpiopwm_config.out = 0;
    gpiopwm_config.wake = 0;
    gpiopwm_config.pwm = 0;
    gpiopwm_config.period = 0;
    gpiopwm_config.duty[0] = 0;
    gpiopwm_config.duty[1] = 0;
    gpiopwm_config.duty[2] = 0;
    gpiopwm_config.duty[3] = 0;

    return (qt602240_reg_write(data, SPT_GPIOPWM_T19, (void *) &gpiopwm_config));
}

int QT602240_Grip_Face_Suppression_Config_Init(struct qt602240_data *data)
{
    gripfacesuppression_config.ctrl = 0x13;
    gripfacesuppression_config.xlogrip = 5;
    gripfacesuppression_config.xhigrip = 5;
    gripfacesuppression_config.ylogrip = 5;
    gripfacesuppression_config.yhigrip = 5;
    gripfacesuppression_config.maxtchs = 0x00;
    gripfacesuppression_config.reserved = 0x00;
    gripfacesuppression_config.szthr1 = 0x19;
    gripfacesuppression_config.szthr2 = 0x0f;
    gripfacesuppression_config.shpthr1 = 0x02;
    gripfacesuppression_config.shpthr2 = 0x0a;
    gripfacesuppression_config.supextto = 0x32;

    /* Write grip suppression config to chip. */
    return (qt602240_reg_write(data, PROCI_GRIPFACESUPPRESSION_T20, (void *) &gripfacesuppression_config));
}

int QT602240_Noise_Config_Init(struct qt602240_data *data)
{

    //0x8 : Enable Median filter, 0x4 : Enable Frequency hopping, 0x1 : Enable
    noise_suppression_config.ctrl = 0x0d;        //Median filter off, report enable
    noise_suppression_config.reserved = 0;
    noise_suppression_config.reserved1 = 0;
    noise_suppression_config.gcaful1 = 0;        // Upper limit for the GCAF
    noise_suppression_config.gcaful2 = 0;
    noise_suppression_config.gcafll1 = 0;        // Lower limit for the GCAF
    noise_suppression_config.gcafll2 = 0;
    noise_suppression_config.actvgcafvalid = 3;  //Minium number of samples in active mode
    noise_suppression_config.noisethr = 20;      // Threshold for the noise signal
    noise_suppression_config.freqhopscale = 0x00;
#if defined(CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
    noise_suppression_config.freq[0] = 11;
    noise_suppression_config.freq[1] = 15;
    noise_suppression_config.freq[2] = 36;
    noise_suppression_config.freq[3] = 45;
    noise_suppression_config.freq[4] = 55;
#else
    noise_suppression_config.freq[0] = 10;
    noise_suppression_config.freq[1] = 15;
    noise_suppression_config.freq[2] = 20;
    noise_suppression_config.freq[3] = 25;
    noise_suppression_config.freq[4] = 30;
#endif
    noise_suppression_config.idlegcafvalid = 3;

    /* Write Noise suppression config to chip. */
    return (qt602240_reg_write(data, PROCG_NOISESUPPRESSION_T22, (void *) &noise_suppression_config));
}

int QT602240_One_Touch_Gesture_Config_Init(struct qt602240_data *data)
{
    /* Disable one touch gestures. */
    onetouch_gesture_config.ctrl = 0;
    onetouch_gesture_config.numgest = 0;
    onetouch_gesture_config.gesten1 = 0;    //2byte
    onetouch_gesture_config.gesten2 = 0;
    onetouch_gesture_config.pressproc = 0;
    onetouch_gesture_config.tapto = 0;
    onetouch_gesture_config.flickto = 0;
    onetouch_gesture_config.dragto = 0;
    onetouch_gesture_config.spressto = 0;
    onetouch_gesture_config.lpressto = 0;
    onetouch_gesture_config.reppressto = 0;
    onetouch_gesture_config.flickthr1 = 0;       //2byte
    onetouch_gesture_config.flickthr2 = 0;
    onetouch_gesture_config.dragthr1 = 0;       //2byte
    onetouch_gesture_config.dragthr2 = 0;
    onetouch_gesture_config.tapthr1 = 0;        //2byte
    onetouch_gesture_config.tapthr2 = 0;
    onetouch_gesture_config.throwthr1 = 0;     //2byte
    onetouch_gesture_config.throwthr2 = 0;

    return (qt602240_reg_write(data, PROCI_ONETOUCHGESTUREPROCESSOR_T24, (void *) &onetouch_gesture_config));
}

int QT602240_Proximity_Config_Init(struct qt602240_data *data)
{
    /* Disable Proximity. */
    proximity_config.ctrl = 0;
    proximity_config.xorigin = 0;
    proximity_config.xsize = 0;
    proximity_config.ysize = 0;
    proximity_config.reserved_for_future_aks_usage = 0;
    proximity_config.blen = 0;
    proximity_config.tchthr1 = 0;
    proximity_config.tchthr2 = 0;
    proximity_config.tchdi = 0;
    proximity_config.average = 0;
    proximity_config.rate1 = 0;
    proximity_config.rate2 = 0;
    proximity_config.mvdthr1 = 0;
    proximity_config.mvdthr2 = 0;

    return (qt602240_reg_write(data, TOUCH_PROXIMITY_T23, (void *) &proximity_config));
}

int QT602240_Selftest_Config_Init(struct qt602240_data *data)
{
    selftest_config.ctrl = 0;
    selftest_config.cmd = 0;

    return (qt602240_reg_write(data, SPT_SELFTEST_T25, (void *) &selftest_config));
}

int QT602240_Two_touch_Gesture_Config_Init(struct qt602240_data *data)
{
    /* Disable two touch gestures. */
    twotouch_gesture_config.ctrl = 0;
    twotouch_gesture_config.numgest = 0;
    twotouch_gesture_config.reserved2 = 0;
    twotouch_gesture_config.gesten = 0;
    twotouch_gesture_config.rotatethr = 0;
    twotouch_gesture_config.zoomthr1 = 0;       //2byte
    twotouch_gesture_config.zoomthr2 = 0;

    return (qt602240_reg_write(data, PROCI_TWOTOUCHGESTUREPROCESSOR_T27, (void *) &twotouch_gesture_config));
}

int QT602240_CTE_Config_Init(struct qt602240_data *data)
{
     /* Set CTE config */
    cte_config.ctrl = 0x00;     //reserved
    cte_config.cmd = 0x00;

    cte_config.mode = 0x02;

    cte_config.idlegcafdepth = 0x8;      //Size of sampling window in idle acquisition mode
    cte_config.actvgcafdepth = 0x20;     //Size of sampling window in active acquisition mode
    cte_config.voltage = 0x0a;           // 0.01 * 10 + 2.7

     /* Write CTE config to chip. */
    return (qt602240_reg_write(data, SPT_CTECONFIG_T28, (void *) &cte_config));
}

static int qt602240_object_readable(unsigned int type)
{
    switch (type) {
    case QT602240_GEN_MESSAGE:
    case QT602240_GEN_COMMAND:
    case QT602240_GEN_POWER:
    case QT602240_GEN_ACQUIRE:
    case QT602240_TOUCH_MULTI:
    case QT602240_TOUCH_KEYARRAY:
    case QT602240_TOUCH_PROXIMITY:
    case QT602240_PROCI_GRIPFACE:
    case QT602240_PROCG_NOISE:
    case QT602240_PROCI_ONETOUCH:
    case QT602240_PROCI_TWOTOUCH:
    case QT602240_SPT_GPIOPWM:
    case QT602240_SPT_SELFTEST:
    case QT602240_SPT_CTECONFIG:
        return 1;
    default:
        return 0;
    }
}

static int qt602240_check_bootloader(struct i2c_client *client,
        unsigned int state)
{
    u8 val;
    static int error_count = 0;

recheck:
    if (i2c_master_recv(client, &val, 1) != 1)
    {
        dev_err(&client->dev, "i2c recv failed\n");
        return -EIO;
    }

    switch (state) {
    case QT602240_WAITING_BOOTLOAD_CMD:
    case QT602240_WAITING_FRAME_DATA:
        val &= ~QT602240_BOOT_STATUS_MASK;
        break;
    case QT602240_FRAME_CRC_PASS:
        if (val == QT602240_FRAME_CRC_CHECK)
            goto recheck;
        break;
    default:
        return -EINVAL;
    }

    if (val != state)
    {
        if(error_count>=10)
        {
        dev_err(&client->dev, "Unvalid bootloader mode state\n");
        return -EINVAL;
           }
           else
           {
        error_count++;
        printk(KERN_ERR"[TSP] state : 0x%x, return val : 0x%d , error_count : %d\n", state, val, error_count );
        return 1;
         }
    }

    return 0;
}

static int qt602240_unlock_bootloader(struct i2c_client *client)
{
    u8 buf[2];

    buf[0] = QT602240_UNLOCK_CMD_LSB;
    buf[1] = QT602240_UNLOCK_CMD_MSB;

    if (i2c_master_send(client, buf, 2) != 2) {
        dev_err(&client->dev, "i2c send failed\n");
        return -EIO;
    }

    return 0;
}

static int qt602240_fw_write(struct i2c_client *client, const u8 *data,
        unsigned int frame_size)
{
    if (i2c_master_send(client, data, frame_size) != frame_size) {
        dev_err(&client->dev, "i2c send failed\n");
        return -EIO;
    }

    return 0;
}

static int qt602240_read_reg(struct i2c_client *client, u16 reg)
{
    u8 buf[2];
    u8 val;

    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;

    if (i2c_master_send(client, buf, 2) != 2) {
        dev_err(&client->dev, "i2c send failed\n");
        return -EIO;
    }

    if (i2c_master_recv(client, &val, 1) != 1) {
        dev_err(&client->dev, "i2c recv failed\n");
        return -EIO;
    }

    return val;
}

static int qt602240_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
    u8 buf[3];

    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;
    buf[2] = val;

    if (i2c_master_send(client, buf, 3) != 3) {
        dev_err(&client->dev, "i2c send failed\n");
        return -EIO;
    }

    return 0;
}

static int qt602240_read_object_table(struct i2c_client *client, u16 reg,
        u8 *object_buf)
{
    u8 buf[2];

    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;

    if (i2c_master_send(client, buf, 2) != 2) {
        dev_err(&client->dev, "i2c send failed\n");
        return -EIO;
    }

    if (i2c_master_recv(client, object_buf, 6) != 6) {
        dev_err(&client->dev, "i2c recv failed\n");
        return -EIO;
    }

    return 0;
}

static int qt602240_read_message(struct qt602240_data *data)
{
    struct i2c_client *client = data->client;
    struct qt602240_object *object;
    u16 reg;
    u8 buf[2];

    object = qt602240_get_object(data, QT602240_GEN_MESSAGE);
    if (!object)
        return -EINVAL;

    reg = object->start_address;

    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;

    if (i2c_master_send(client, buf, 2) != 2) {
        dev_err(&client->dev, "i2c send failed\n");
        return -EIO;
    }

    if (i2c_master_recv(client, (u8 *)data->object_message, 9) != 9) {
        dev_err(&client->dev, "i2c recv failed\n");
        return -EIO;
    }

    return 0;
}

static int qt602240_read_diagnostic(u16 read_addr, u8 *buffer, u8 size)
{
    struct i2c_client *client = p_qt602240_data->client;
    struct qt602240_object *object;
    u16 reg;
    u8 buf[2];

    object = qt602240_get_object(p_qt602240_data, QT602240_DEBUG_DIAGNOSTIC);
    if (!object)
        return -EINVAL;

    reg = object->start_address + read_addr;

    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;

    if (i2c_master_send(client, buf, 2) != 2) {
        dev_err(&client->dev, "i2c send failed\n");
        return -EIO;
    }

    if (i2c_master_recv(client, buffer, size) != size) {
        dev_err(&client->dev, "i2c recv failed\n");
        return -EIO;
    }

    return 0;
}


static int qt602240_read_object(struct qt602240_data *data, u8 type, u8 offset)
{
    struct qt602240_object *object;
    u16 reg;

    object = qt602240_get_object(data, type);
    if (!object)
        return -EINVAL;

    reg = object->start_address;

    return qt602240_read_reg(data->client, reg + offset);
}

static int qt602240_write_object(struct qt602240_data *data, u8 type,
        u8 offset, u8 val)
{
    struct qt602240_object *object;
    u16 reg;

    object = qt602240_get_object(data, type);
    if (!object)
    {
        printk(KERN_ERR"[TSP] error : 0x%x\n", object->type);
        return -EINVAL;
    }

    reg = object->start_address;
    return qt602240_write_reg(data->client, reg + offset, val);
}

void calibrate_chip(struct qt602240_data *data)
{
    uint8_t atchcalst, atchcalsthr;
    int error;

    if(!cal_check_flag)
    {
        printk(KERN_DEBUG "[TSP] Calibrating...\n");
        /* change calibration suspend settings to zero until calibration confirmed good */
        /* store normal settings */
        atchcalst = acquisition_config.atchcalst;
        atchcalsthr = acquisition_config.atchcalsthr;

        /* resume calibration must be performed with zero settings */
        acquisition_config.atchcalst = 0;
        acquisition_config.atchcalsthr = 0;

        error = qt602240_write_object(p_qt602240_data, QT602240_GEN_ACQUIRE,
            QT602240_ACQUIRE_ATCHCALST, acquisition_config.atchcalst);
        error |= qt602240_write_object(p_qt602240_data, QT602240_GEN_ACQUIRE,
            QT602240_ACQUIRE_ATCHCALSTHR, acquisition_config.atchcalsthr);

        if(error<0)
        {
            printk(KERN_ERR "[TSP] fail to write the Acqusition config\n");
        }

        /* restore settings to the local structure so that when we confirm the
        * cal is good we can correct them in the chip */
        /* this must be done before returning */
        acquisition_config.atchcalst = atchcalst;
        acquisition_config.atchcalsthr = atchcalsthr;
    }

    /* send calibration command to the chip */
    error = qt602240_write_object(data, QT602240_GEN_COMMAND,
                QT602240_COMMAND_CALIBRATE, 1);
    if (error < 0)
    {
        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
    }
    else
    {
        /* set flag to show we must still confirm if calibration was good or bad */
        cal_check_flag = true;
        release_all_fingers(data->input_dev);
    }
}

static int check_abs_time(void)
{
    qt_time_diff = jiffies_to_msecs(jiffies) - qt_time_point;
    if(qt_time_diff >0)
        return 1;
    else
        return 0;
}

void check_chip_calibration(struct qt602240_data *data)
{
    u8 data_buffer[100] = { 0 };
    u16 check_tsp = 0;
    int try_ctr = 0;
    int tch_ch = 0, atch_ch = 0;
    int i, error, x_line_limit;

    if (down_interruptible(&g_tsp_mutex))
        return;
    /* we have had the first touchscreen or face suppression message
    * after a calibration - check the sensor state and try to confirm if
    * cal was good or bad */

    /* get touch flags from the chip using the diagnostic object */
    /* write command to command processor to get touch flags - 0xF3 Command required to do this */
    error = qt602240_write_object(data, QT602240_GEN_COMMAND,
            QT602240_COMMAND_DIAGNOSTIC, 0xF3);
    if (error < 0)
    {
        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
    }

    qt602240_read_diagnostic(0, data_buffer, 2 );

    while(!((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00)))
    {
        /* wait for data to be valid  */
        if(try_ctr > 10) //0318 hugh 100-> 10
        {
            /* Failed! */
            printk(KERN_ERR "[TSP] Diagnostic Data did not update!!\n");
            qt_timer_state = 0;//0430 hugh

            /* soft reset */
            qt602240_write_object(data, QT602240_GEN_COMMAND,
                    QT602240_COMMAND_RESET, 1);

            /* wait for soft reset */
            msleep(100);
            calibrate_chip(data);
            break;
        }
        msleep(2); //0318 hugh  3-> 2
        try_ctr++; /* timeout counter */

        qt602240_read_diagnostic(0, data_buffer, 2 );
    }

    /* data is ready - read the detection flags */
    qt602240_read_diagnostic(0, data_buffer, 82);

    /* data array is 20 x 16 bits for each set of flags, 2 byte header, 40 bytes for touch flags 40 bytes for antitouch flags*/

    /* count up the channels/bits if we recived the data properly */
    if((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00))
    {
        x_line_limit = touchscreen_config.xsize;

        if(x_line_limit > 20)
        {
            /* hard limit at 20 so we don't over-index the array */
            x_line_limit = 20;
        }

        /* double the limit as the array is in bytes not words */
        x_line_limit = x_line_limit << 1;

        for(i = 0; i < x_line_limit; i++)
        {
            check_tsp = data_buffer[2+i];
            while(check_tsp)
            {
                if(check_tsp & 0x1)
                {
                    tch_ch++;
                }
                check_tsp = check_tsp >>1;
            }

            check_tsp = data_buffer[42+i];
            while(check_tsp)
            {
                if(check_tsp & 0x1)
                {
                    atch_ch++;
                }
                check_tsp = check_tsp >>1;
            }
        }

        /* print how many channels we counted */
        if(atch_ch>0)
        {
            printk(KERN_DEBUG "[TSP] Flags Counted channels: t:%d a:%d \n", tch_ch, atch_ch);
        }

        /* send page up command so we can detect when data updates next time,
             * page byte will sit at 1 until we next send F3 command */
        error = qt602240_write_object(data, QT602240_GEN_COMMAND,
            QT602240_COMMAND_DIAGNOSTIC, 0x01);
        if (error < 0)
        {
            printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
        }

        /* process counters and decide if we must re-calibrate or if cal was good */
        if((tch_ch>0) && (atch_ch == 0))  //jwlee change.
        {
            /* cal was good - don't need to check any more */
            //hugh 0312
            if(!check_abs_time())
                qt_time_diff=301;

            if(qt_timer_state == 1)
            {
                if(qt_time_diff > 300)
                {
                    printk(KERN_DEBUG "[TSP] calibration was good\n");
                    qt_timer_state =0;
                    cal_check_flag = false;

                    /* Write normal acquisition config back to the chip. */
                    error = qt602240_write_object(p_qt602240_data, QT602240_GEN_ACQUIRE,
                        QT602240_ACQUIRE_ATCHCALST, acquisition_config.atchcalst);
                    error |= qt602240_write_object(p_qt602240_data, QT602240_GEN_ACQUIRE,
                        QT602240_ACQUIRE_ATCHCALSTHR, acquisition_config.atchcalsthr);
                    if(error<0)
                    {
                        printk(KERN_ERR "[TSP] fail to write the Acqusition config\n");
                    }
                }

            }
            else
            {
                qt_timer_state=1;
                qt_time_point = jiffies_to_msecs(jiffies);
                cal_check_flag=true;
            }

        }
        else if(atch_ch >= 3)        //jwlee add 0325
        {
            printk(KERN_DEBUG "[TSP] calibration was bad\n");

            /* cal was bad - must recalibrate and check afterwards */
            calibrate_chip(data);
            qt_timer_state=0;
        }
        else
        {
            /* we cannot confirm if good or bad - we must wait for next touch  message to confirm */
            /* Reset the 100ms timer */
            qt_timer_state=0;//0430 hugh 1 --> 0
            qt_time_point = jiffies_to_msecs(jiffies);
        }
    }
	up(&g_tsp_mutex);
}


static void qt602240_input_read(struct qt602240_data *data)
{
    struct qt602240_message *message = data->object_message;
    struct qt602240_object *object;
    struct input_dev *input_dev = data->input_dev;
    bool touch_message_flag;
    u8 reportid = 0xff;
    u8 touch_status = 0;
    u8 id, size;
    int i;
    int bChangeUpDn= 0;
    int x = 0, y = 0;
    static int nPrevID= -1;
    int touch_count = 0;

#ifdef TOUCH_BOOST
    if(enable_touch_boost) //added for touchscreen boost,samsung customisation,enabled in init.rc
    {
        if (timer_pending(&opp_set_timer))
            del_timer(&opp_set_timer);
        if(!touch_boost_state)
        {
            omap_pm_set_min_mpu_freq((struct device *)p_qt602240_data->input_dev, VDD1_OPP4_FREQ);
            touch_boost_state = 1;
        }
        mod_timer(&opp_set_timer, jiffies + (100 * HZ) / 1000);
	}
#endif

    touch_message_flag = false;

    if (down_interruptible(&g_tsp_mutex))
        return;
    if (qt602240_read_message(data))
    {
        printk(KERN_ERR "[TSP] Couldn't read message\n");

        /* soft reset and try to get message again*/
        qt602240_write_object(data, QT602240_GEN_COMMAND,
                QT602240_COMMAND_RESET, 1);
        msleep(100);
    }
    up(&g_tsp_mutex);

    reportid = message->reportid;

    if(( reportid >= REPORTID_TSP_MIN )&& ( reportid <= REPORTID_TSP_MAX ))
    {
        id = message->reportid-2;

        /* whether reportid is thing of QT602240_TOUCH_MULTI */
        object = qt602240_get_object(data, QT602240_TOUCH_MULTI);
        if (!object)
        {
            printk(KERN_ERR "[TSP] Couldn't get the object\n");
        }

        touch_status = message->message[0];          //Message[0] : Touch status

        x = (message->message[1] << 2) |             //Message[1] : x position MSByte
              ((message->message[3] & ~0x3f) >> 6);  //Message[3] : x position LSBits , bit4 ~ 7

        y = (message->message[2] << 2) |             //Message[2] : y position MSByte
             ((message->message[3] & ~0xf3) >> 2);   //Message[3] : y position LSBits , bit0~3

        size = message->message[4];

        if (touch_status & 0x20)                   // Release : 0x20
        {
            fingerInfo[id].pressure= 0;
            bChangeUpDn= 1;
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
            printk(KERN_DEBUG "[TSP] Finger[%d] Up    (%d,%d) size : %d\n", id, fingerInfo[id].x, fingerInfo[id].y, size);
#endif
        }
        else if ((touch_status & 0xc0) == 0xc0)      // Detect & Press  : 0x80 | 0x40
        {
            touch_message_flag = true;
#if defined(_SUPPORT_TOUCH_AMPLITUDE_)
            fingerInfo[id].pressure= message->message[5];
#else
            fingerInfo[id].pressure= 40;
#endif
            fingerInfo[id].x= (int16_t)x;
            fingerInfo[id].y= (int16_t)y;
            bChangeUpDn= 1;
#if defined(DRIVER_FILTER)
            equalize_coordinate(1, id, &fingerInfo[id].x, &fingerInfo[id].y);
#endif
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
            printk(KERN_DEBUG "[TSP] Finger[%d] Down  (%d,%d) size : %d \n", id, fingerInfo[id].x, fingerInfo[id].y, size);
#endif
        }
        else if ((touch_status & 0x90) == 0x90)          // Detect & Move : 0x80 | 0x10
        {
            touch_message_flag = true;
#if defined(_SUPPORT_TOUCH_AMPLITUDE_)
            fingerInfo[id].pressure= message->message[5];
#endif
            fingerInfo[id].x= (int16_t)x;
            fingerInfo[id].y= (int16_t)y;
#if defined(DRIVER_FILTER)
            equalize_coordinate(0, id, &fingerInfo[id].x, &fingerInfo[id].y);
#endif
        }
#if defined(_SUPPORT_TOUCH_AMPLITUDE_)
        else if(touch_status == 0x84)                      // amplitude chage
        {
            fingerInfo[id].pressure= message->message[5];
        }
#endif
        else
        {
            printk(KERN_DEBUG "[TSP] Unknown state(%x)! \n", touch_status);
        }

        fingerInfo[id].width = size;

        if( nPrevID >= id || bChangeUpDn )
        {
            for( i= 0; i<MAX_USING_FINGER_NUM; ++i )
            {
                if(fingerInfo[i].pressure == -1 )
                {
                    continue;
                }

                input_report_abs(input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
                input_report_abs(input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
                input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].pressure);
                input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].width);
                input_report_abs(input_dev, ABS_MT_TRACKING_ID, i);
                input_mt_sync(input_dev);

                if(fingerInfo[i].pressure == 0 )
                {
                    fingerInfo[i].pressure= -1;
                }
                else touch_count++;
            }

            input_sync(input_dev);
        }

        nPrevID= id;

    }
    else if(( reportid >= REPORTID_TSPKEY_MIN )&& ( reportid <= REPORTID_TSPKEY_MAX ))
    {

        /* whether reportid is thing of QT602240_TOUCH_KEYARRAY */
        object = qt602240_get_object(data, QT602240_TOUCH_KEYARRAY);
        if (!object)
        {
            printk(KERN_ERR "[TSP] Couldn't get the object\n");
        }

        for(i = 0; i <NUMOFKEYS; i++ )
        {
            if(tsp_keystatus[i])
            {
                input_report_key(input_dev, tsp_keycodes[i], 0);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
                printk(KERN_DEBUG "[TSP] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
#endif
                tsp_keystatus[i] = KEY_RELEASE;
            }
            else if(message->message[1] & (0x1<<i) )
            {
                if(message->message[0] & 0x80)                                  // detect
                {
                    input_report_key(input_dev, tsp_keycodes[i], 1);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
                    printk(KERN_DEBUG "[TSP] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
#endif
                    tsp_keystatus[i] = KEY_PRESS;
                }
            }
        }
    }
    else if( reportid == REPORTID_PALM )          //20102017 julia
    {
        /* whether reportid is thing of QT602240_PROCI_GRIPFACE */
        object = qt602240_get_object(data, QT602240_PROCI_GRIPFACE);
        if (!object)
        {
            printk(KERN_ERR "[TSP] Couldn't get the object : palm touch.\n");
        }

        if(((message->message[0])&0x01) == 0x00)
        {
            printk(KERN_DEBUG "[TSP] Palm Touch!released.\n");
            release_all_fingers(input_dev);
            calibrate_chip(data);
        }
        else
        {
            printk(KERN_DEBUG "[TSP] Palm Touch!suppressed\n");
            touch_message_flag = true;
            enable_autocal_timer(5);
        }
    }
    else if (reportid == REPORTID_COMMANDPROECESSOR)
    {
        /* Check the calibration and overflow */
        object = qt602240_get_object(data, QT602240_GEN_COMMAND);
        if (!object)
        {
            printk(KERN_ERR "[TSP] Couldn't get the object : QT602240_GEN_COMMAND \n");
        }

        if(((message->message[0])&0x10) == 0x10)
        {
            printk(KERN_DEBUG "[TSP] The device is calibrating...\n");
            cal_check_flag = true;
            qt_timer_state = 0;
            qt_time_point = 0;
        }

        if(((message->message[0])&0x40) == 0x40)
        {
            printk(KERN_ERR "[TSP] Overflow!!0: 0x%x, 1: 0x%x, 2: 0x%x, 3: 0x%x, 4: 0x%x, 5: 0x%x, 6: 0x%x, checksum: 0x%x\n",
                message->message[0], message->message[1], message->message[2],
                message->message[3], message->message[4], message->message[5],
                message->message[6], message->checksum);

            release_all_fingers(input_dev);

            /* try to soft reset */
            if (down_interruptible(&g_tsp_mutex))
                return;
            qt602240_write_object(data, QT602240_GEN_COMMAND,
                    QT602240_COMMAND_RESET, 1);

            /* wait for soft reset */
            msleep(100);

            /* calibrate again */
            calibrate_chip(data);
            up(&g_tsp_mutex);
        }

    }

    if(cal_check_flag && touch_message_flag)
    {
        check_chip_calibration(data);
        if(touch_count >= 2)
        {
            enable_autocal_timer(5);
        }
    }
}

static irqreturn_t qt602240_interrupt(int irq, void *dev_id)
{
    struct qt602240_data *data = dev_id;
    disable_irq_nosync(data->irq);
    queue_work(tsp_wq, &data->tsp_work);
    return IRQ_HANDLED;
}

void qt602240_reg_init(struct qt602240_data *data)
{
    int ret;

    ret = QT602240_Command_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Command config\n");

    ret = QT602240_Powr_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Power config\n");

    ret = QT602240_Acquisition_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Acqusition config\n");

    ret = QT602240_Multitouch_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Multi-touch config\n");

    ret = QT602240_KeyArrary_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the KeyArrary config\n");

    ret = QT602240_Grip_Face_Suppression_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the suppression config\n");

    ret = QT602240_Noise_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Noise config\n");

    ret = QT602240_One_Touch_Gesture_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the 1 Gesture config\n");

    ret = QT602240_GPIOPWM_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the GPIO/PWM config\n");

    ret = QT602240_Proximity_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the Proximity config\n");

    ret = QT602240_Selftest_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the selftest config\n");

    ret = QT602240_Two_touch_Gesture_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the 2 gesture config\n");

    ret = QT602240_CTE_Config_Init(data);
    if(ret<0)
        printk(KERN_ERR "[TSP] fail to initialize the CTE config\n");

}

void qt602240_check_matrix_size(struct qt602240_data *data)
{
    u8 mode;
    mode = touchscreen_config.xsize - 16;
    if( mode != cte_config.mode)
        qt602240_write_object(data, QT602240_SPT_CTECONFIG,
                QT602240_CTE_MODE, mode);
}

static int qt602240_info_get(struct qt602240_data *data)
{
    struct i2c_client *client = data->client;
    struct qt602240_info *info = data->info;
    int val;

    val = qt602240_read_reg(client, QT602240_FAMILY_ID);
    if (val < 0)
        return -EIO;
    info->family_id = val;

    val = qt602240_read_reg(client, QT602240_VARIANT_ID);
    if (val < 0)
        return -EIO;
    info->variant_id = val;

    val = qt602240_read_reg(client, QT602240_VERSION);
    if (val < 0)
        return -EIO;
    info->version = val;

    val = qt602240_read_reg(client, QT602240_BUILD);
    if (val < 0)
        return -EIO;
    info->build = val;

    val = qt602240_read_reg(client, QT602240_MATRIX_X_SIZE);
    if (val < 0)
        return -EIO;
    info->matrix_xsize = val;

    val = qt602240_read_reg(client, QT602240_MATRIX_Y_SIZE);
    if (val < 0)
        return -EIO;
    info->matrix_ysize = val;

    val = qt602240_read_reg(client, QT602240_OBJECT_NUM);
    if (val < 0)
        return -EIO;
    info->object_num = val;

    return 0;
}

static int qt602240_initialize(struct qt602240_data *data)
{
    struct i2c_client *client = data->client;
    struct qt602240_info *info;
    int i;
    int ret;
    u16 reg;
    u8 buf[QT602240_OBJECT_SIZE];
    u8 reportid = 0;

    info = data->info = kzalloc(sizeof(struct qt602240_info), GFP_KERNEL);

    if (!data->info) {
        dev_err(&data->client->dev, "Failed to allocate memory\n");
        return -ENOMEM;
    }

    ret = qt602240_info_get(data);
    if (ret < 0)
    {
        dev_err( &data->client->dev, "[TSP] Failed to get TSP info. \n");
        return ret;
    }
    else
    {
            printk(KERN_DEBUG "[TSP] F/W version    : %d\n", data->info->version );
            printk(KERN_DEBUG "[TSP] Family  ID     : %d\n", data->info->family_id);
            printk(KERN_DEBUG "[TSP] Variant ID     : %d\n", data->info->variant_id);
            printk(KERN_DEBUG "[TSP] Build number   : %d\n", data->info->build);
            printk(KERN_DEBUG "[TSP] X size         : %d\n", data->info->matrix_xsize);
            printk(KERN_DEBUG "[TSP] Y size         : %d\n", data->info->matrix_ysize);
        }

    data->object_table =
        kzalloc(sizeof(struct qt602240_object) * data->info->object_num,
                GFP_KERNEL);
    data->object_message =
        kzalloc(sizeof(struct qt602240_message), GFP_KERNEL);
    if (!data->object_table || !data->object_message) {
        dev_err(&data->client->dev, "Failed to allocate memory\n");
        return -ENOMEM;
    }

    /* get object table information */
    for (i = 0; i < data->info->object_num; i++) {
        struct qt602240_object *object = data->object_table + i;

        reg = QT602240_OBJECT_START + QT602240_OBJECT_SIZE * i;
        ret = qt602240_read_object_table(client, reg, buf);
        if (ret)
            return ret;

        object->type = buf[0];
        object->start_address = (buf[2] << 8) | buf[1];
        object->size = buf[3];
        object->instances = buf[4];
        object->num_report_ids = buf[5];

        if (object->num_report_ids) {
            reportid += object->num_report_ids *
                (object->instances + 1);
            object->max_reportid = reportid;
        }
    }

    /* Init qt602240 reg*/
    qt602240_reg_init(data);

    /* check X/Y matrix size */
    qt602240_check_matrix_size(data);

    /* read dummy message to make high CHG pin */
    do {
        ret = qt602240_read_object(data, QT602240_GEN_MESSAGE, 0);
        if (ret < 0)
            return ret;
    } while (ret != 0xff);


    /* backup to memory */
    qt602240_write_object(data, QT602240_GEN_COMMAND,
            QT602240_COMMAND_BACKUPNV, QT602240_BACKUP_VALUE);
    /* soft reset */
    qt602240_write_object(data, QT602240_GEN_COMMAND,
            QT602240_COMMAND_RESET, 1);

        msleep(100);

    printk(KERN_DEBUG "[TSP] Family ID: %d Variant ID: %d Version: %d Build: %d\n",
            info->family_id, info->variant_id, info->version, info->build);

    printk(KERN_DEBUG "[TSP] Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
            info->matrix_xsize, info->matrix_ysize, info->object_num);

    calibrate_chip(data);

    return 0;
}

static ssize_t qt602240_info_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct qt602240_data *data = dev_get_drvdata(dev);
    struct qt602240_info *info = data->info;
    int count = 0;

    count += sprintf(buf + count, "Family ID:\t0x%x (%d)\n",
            info->family_id, info->family_id);
    count += sprintf(buf + count, "Variant ID:\t0x%x (%d)\n",
            info->variant_id, info->variant_id);
    count += sprintf(buf + count, "Version:\t0x%x (%d)\n",
            info->version, info->version);
    count += sprintf(buf + count, "Build:\t\t0x%x (%d)\n",
            info->build, info->build);
    count += sprintf(buf + count, "Matrix X Size:\t0x%x (%d)\n",
            info->matrix_xsize, info->matrix_xsize);
    count += sprintf(buf + count, "Matrix Y Size:\t0x%x (%d)\n",
            info->matrix_ysize, info->matrix_ysize);
    count += sprintf(buf + count, "Object Num:\t0x%x (%d)\n",
            info->object_num, info->object_num);

    return count;
}

static ssize_t qt602240_object_table_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct qt602240_data *data = dev_get_drvdata(dev);
    struct qt602240_object *object;
    int count = 0;
    int i;

    for (i = 0; i < data->info->object_num; i++) {
        object = data->object_table + i;

        count += sprintf(buf + count,
                "Object Table Element %d\n", i + 1);
        count += sprintf(buf + count, "  type:\t\t\t0x%x (%d)\n",
                object->type, object->type);
        count += sprintf(buf + count, "  start_address:\t0x%x (%d)\n",
                object->start_address, object->start_address);
        count += sprintf(buf + count, "  size:\t\t\t0x%x (%d)\n",
                object->size, object->size);
        count += sprintf(buf + count, "  instances:\t\t0x%x (%d)\n",
                object->instances, object->instances);
        count += sprintf(buf + count, "  num_report_ids:\t0x%x (%d)\n",
                object->num_report_ids, object->num_report_ids);
        count += sprintf(buf + count, "  max_reportid:\t\t0x%x (%d)\n",
                object->max_reportid, object->max_reportid);
        count += sprintf(buf + count, "\n");
    }

    return count;
}

static ssize_t qt602240_object_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct qt602240_data *data = dev_get_drvdata(dev);
    struct qt602240_object *object;
    int val;
    int count = 0;
    int i, j;

    for (i = 0; i < data->info->object_num; i++) {
        object = data->object_table + i;

        count += sprintf(buf + count,
                "Object Table Element %d(Type %d)\n",
                i + 1, object->type);

        if (!qt602240_object_readable(object->type)) {
            count += sprintf(buf + count, "\n");
            continue;
        }

        for (j = 0; j < object->size + 1; j++) {
            val = qt602240_read_object(data, object->type, j);
            if (val < 0)
                return count;

            count += sprintf(buf + count,
                    "  Byte %d: 0x%x (%d)\n", j, val, val);
        }

        count += sprintf(buf + count, "\n");
    }

    return count;
}

static ssize_t qt602240_object_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct qt602240_data *data = dev_get_drvdata(dev);
    struct qt602240_object *object;
    unsigned int type;
    unsigned int offset;
    unsigned int val;
    int ret;

    if (sscanf(buf, "%u %u %u", &type, &offset, &val) != 3) {
        dev_err(dev, "Invalid values\n");
        return -EINVAL;
    }

    object = qt602240_get_object(data, type);
    if (!object || (offset > object->size)) {
        dev_err(dev, "Invalid values\n");
        return -EINVAL;
    }
    if (down_interruptible(&g_tsp_mutex))
        return -ERESTARTSYS;

    ret = qt602240_write_object(data, type, offset, val);
    if (ret < 0) {
        up(&g_tsp_mutex);
        return ret;
    }
    up(&g_tsp_mutex);
    return count;
}

static int qt602240_update_fw(struct device *dev)
{
    struct qt602240_data *data = dev_get_drvdata(dev);
    unsigned int frame_size;
    unsigned int pos = 0;
    int ret;

    if (down_interruptible(&g_tsp_mutex))
        return -ERESTARTSYS;

    /* change to the bootloader mode */
    qt602240_write_object(data, QT602240_GEN_COMMAND,
            QT602240_COMMAND_RESET, QT602240_BOOT_VALUE);
    msleep(100);

    /* change to slave address of bootloader */
    if (data->client->addr == QT602240_APP_LOW)
        data->client->addr = QT602240_BOOT_LOW;
    else
        data->client->addr = QT602240_BOOT_HIGH;

    ret = qt602240_check_bootloader(data->client,
            QT602240_WAITING_BOOTLOAD_CMD);
    if (ret < 0)
        goto err_fw;

    /* unlock bootloader */
    qt602240_unlock_bootloader(data->client);
    msleep(100);

    while (pos < sizeof(firmware_602240) )
    {
        ret = qt602240_check_bootloader(data->client,
                QT602240_WAITING_FRAME_DATA);
        if (ret < 0)
            goto err_fw;

        frame_size = ((*(firmware_602240+ pos) << 8) | *(firmware_602240 + pos + 1));

        /* We should add 2 at frame size as the firmware data is not
         * included the CRC bytes.
         */
        frame_size += 2;

        printk(KERN_DEBUG "[TSP] frame_size : 0x%x\n", frame_size);

        /* write one frame to device */
        qt602240_fw_write(data->client, firmware_602240 + pos, frame_size);

        ret = qt602240_check_bootloader(data->client,
                QT602240_FRAME_CRC_PASS);
        if (ret < 0)
            goto err_fw;
        else if( ret == 0)
            pos += frame_size;

        dev_info(dev, "Updated %zd bytes / %zd bytes\n", pos, sizeof(firmware_602240));
    }

    err_fw:
    /* change to slave address of application */
    if (data->client->addr == QT602240_BOOT_LOW)
        data->client->addr = QT602240_APP_LOW;
    else
        data->client->addr = QT602240_APP_HIGH;

    up(&g_tsp_mutex);

    return ret;
}

static ssize_t qt602240_update_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", gFirmware_Update_State);
}

static ssize_t qt602240_update_fw_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int num;
    int ret = 0;

    if (sscanf(buf, "%u", &num) != 1) {
        dev_err(dev, "Invalid values\n");
        return -EINVAL;
    }

    if (num != 1) {
        dev_err(dev, "Invalid values\n");
        return -EINVAL;
    }

    printk(KERN_DEBUG "[TSP] F/W  downloading...\n");
    gFirmware_Update_State = FW_UPDATE_DOWNLOADING;

    if(p_qt602240_data->info->version < BUILTIN_FIRMWARE_VERSION)
    {
        ret = qt602240_update_fw(dev);
    }

    if (ret < 0)
    {
        gFirmware_Update_State = FW_UPDATE_FAIL;
        dev_err(dev, "The firmware update fail\n");
    }
    else
    {
        gFirmware_Update_State = FW_UPDATE_DONE;
        dev_info(dev, "The firmware update success\n");
    }

    return count;
}

void qt602240_set_amoled_display(int mode)
{
    set_mode_for_amoled = mode;
}
EXPORT_SYMBOL(qt602240_set_amoled_display);

// mode 1 = Charger connected
// mode 0 = Charger disconnected
void qt602240_inform_charger_connection(int mode)
{
    if(qt60224_notfound_flag == 1)
        return;

    set_mode_for_ta = mode;

    if(is_boot_state == 2)
    {
        printk(KERN_DEBUG "[TSP] Postpone TA detection setting during boot\n");
        return;
    }

    if(is_boot_state == 1)
    {
        is_boot_state = 2;
    }

    if(is_suspend_state == 1)
    {
        printk(KERN_DEBUG "[TSP] Postpone processing wakeup signal from TA\n");
        is_suspend_state = 2;
        return;
    }

    if(p_qt602240_data != NULL)
    {
        if (!work_pending(&p_qt602240_data->ta_work))
        {
            schedule_work(&p_qt602240_data->ta_work);
        }
    }
}
EXPORT_SYMBOL(qt602240_inform_charger_connection);

static void qt602240_tsp_worker(struct work_struct *work)
{
    struct qt602240_data *data = container_of(work,
        struct qt602240_data, tsp_work);
    qt602240_input_read(data);
    enable_irq(data->irq);
}


static void qt602240_ta_worker(struct work_struct *work)
{
    struct qt602240_data *data = container_of(work,
        struct qt602240_data, ta_work);
    int error;

    if (down_interruptible(&g_tsp_mutex))
        return;

    if(set_mode_for_ta == 2)
    {
        printk(KERN_DEBUG "[TSP] high noise mode on boot time (threshold40)\n");
        touchscreen_config.tchthr = 40;
    }
    else if(set_mode_for_ta)
    {
        printk(KERN_DEBUG "[TSP] TA is connected.\n");
        touchscreen_config.tchthr = 28;
    }
    else
    {
        printk(KERN_DEBUG "[TSP] TA is disconnected.\n");
        touchscreen_config.tchthr = 25;
    }

    error = qt602240_write_object(data, QT602240_TOUCH_MULTI,
                QT602240_TOUCH_TCHTHR, touchscreen_config.tchthr);
    if (error < 0)
    {
        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
    }

    up(&g_tsp_mutex);
}

static DEVICE_ATTR(info, 0444, qt602240_info_show, NULL);
static DEVICE_ATTR(object_table, 0444, qt602240_object_table_show, NULL);
static DEVICE_ATTR(object, 0664, qt602240_object_show, qt602240_object_store);
static DEVICE_ATTR(update_fw, 0220, NULL, qt602240_update_fw_store);
static DEVICE_ATTR(update_status, 0444, qt602240_update_status_show, NULL);

static struct attribute *qt602240_attrs[] = {
    &dev_attr_info.attr,
    &dev_attr_object_table.attr,
    &dev_attr_object.attr,
    &dev_attr_update_fw.attr,
    &dev_attr_update_status.attr,
    NULL
};

static const struct attribute_group qt602240_attr_group = {
    .attrs = qt602240_attrs,
};

#ifdef ENABLE_NOISE_TEST_MODE
struct device *qt602240_noise_test;
//botton_right, botton_left, center, top_right, top_left
unsigned char test_node[5] = {201, 193, 113, 21, 13};

void diagnostic_chip(u8 mode)
{
    int error;
    error = qt602240_write_object(p_qt602240_data, QT602240_GEN_COMMAND,
            QT602240_COMMAND_DIAGNOSTIC, mode);
    if (error < 0)
    {
        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
    }
}

void read_dbg_data(uint8_t dbg_mode , uint8_t node, uint16_t * dbg_data)
{
    u8 read_page, read_point;
    u8 data_buffer[2] = { 0 };
    int i;

    if (down_interruptible(&g_tsp_mutex))
        return;

    read_page = node / 64;
    node %= 64;
    read_point = (node *2) + 2;

    //Page Num Clear
    diagnostic_chip(QT_CTE_MODE);
    msleep(20);

    diagnostic_chip(dbg_mode);
    msleep(20);

    for(i = 0; i < 5; i++)
    {
        qt602240_read_diagnostic(0, data_buffer, 1);
        if(data_buffer[0] == dbg_mode)
        {
            break;
        }
        msleep(20);
    }
    printk(KERN_DEBUG "[TSP] page clear \n");

    for(i = 1; i <= read_page; i++)
    {
        diagnostic_chip(QT_PAGE_UP);
        msleep(20);
        qt602240_read_diagnostic(1, data_buffer, 1);
        printk(KERN_DEBUG "[TSP] page buffer : %d, i : %d \n", data_buffer[0], i);
        if(data_buffer[0] != i)
        {
            if(data_buffer[0] >= 0x4)
                break;
            i--;
        }
    }

    qt602240_read_diagnostic(read_point, data_buffer, 2);
    *dbg_data= ((uint16_t)data_buffer[1]<<8)+ (uint16_t)data_buffer[0];

	up(&g_tsp_mutex);
}

static ssize_t set_refer0_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_refrence=0;
    read_dbg_data(QT_REFERENCE_MODE, test_node[0],&qt_refrence);
    return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_refer1_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_refrence=0;
    read_dbg_data(QT_REFERENCE_MODE, test_node[1],&qt_refrence);
    return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_refer2_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_refrence=0;
    read_dbg_data(QT_REFERENCE_MODE, test_node[2],&qt_refrence);
    return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_refer3_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_refrence=0;
    read_dbg_data(QT_REFERENCE_MODE, test_node[3],&qt_refrence);
    return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_refer4_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_refrence=0;
    read_dbg_data(QT_REFERENCE_MODE, test_node[4],&qt_refrence);
    return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_delta0_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_delta=0;
    read_dbg_data(QT_DELTA_MODE, test_node[0],&qt_delta);
    if(qt_delta < 32767)
        return sprintf(buf, "%u\n", qt_delta);
    else
        qt_delta = 65535 - qt_delta;

    return sprintf(buf, "-%u\n", qt_delta);
}

static ssize_t set_delta1_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_delta=0;
    read_dbg_data(QT_DELTA_MODE, test_node[1],&qt_delta);
    if(qt_delta < 32767)
        return sprintf(buf, "%u\n", qt_delta);
    else
        qt_delta = 65535 - qt_delta;

    return sprintf(buf, "-%u\n", qt_delta);
}

static ssize_t set_delta2_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_delta=0;
    read_dbg_data(QT_DELTA_MODE, test_node[2],&qt_delta);
    if(qt_delta < 32767)
        return sprintf(buf, "%u\n", qt_delta);
    else
        qt_delta = 65535 - qt_delta;

    return sprintf(buf, "-%u\n", qt_delta);
}

static ssize_t set_delta3_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_delta=0;
    read_dbg_data(QT_DELTA_MODE, test_node[3],&qt_delta);
    if(qt_delta < 32767)
        return sprintf(buf, "%u\n", qt_delta);
    else
        qt_delta = 65535 - qt_delta;

    return sprintf(buf, "-%u\n", qt_delta);
}

static ssize_t set_delta4_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint16_t qt_delta=0;
    read_dbg_data(QT_DELTA_MODE, test_node[4],&qt_delta);
    if(qt_delta < 32767)
        return sprintf(buf, "%u\n", qt_delta);
    else
        qt_delta = 65535 - qt_delta;

    return sprintf(buf, "-%u\n", qt_delta);
}

static ssize_t set_threshold_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", touchscreen_config.tchthr);
}

static DEVICE_ATTR(set_refer0, S_IRUGO, set_refer0_mode_show, NULL);
static DEVICE_ATTR(set_delta0, S_IRUGO, set_delta0_mode_show, NULL);
static DEVICE_ATTR(set_refer1, S_IRUGO, set_refer1_mode_show, NULL);
static DEVICE_ATTR(set_delta1, S_IRUGO, set_delta1_mode_show, NULL);
static DEVICE_ATTR(set_refer2, S_IRUGO, set_refer2_mode_show, NULL);
static DEVICE_ATTR(set_delta2, S_IRUGO, set_delta2_mode_show, NULL);
static DEVICE_ATTR(set_refer3, S_IRUGO, set_refer3_mode_show, NULL);
static DEVICE_ATTR(set_delta3, S_IRUGO, set_delta3_mode_show, NULL);
static DEVICE_ATTR(set_refer4, S_IRUGO, set_refer4_mode_show, NULL);
static DEVICE_ATTR(set_delta4, S_IRUGO, set_delta4_mode_show, NULL);
static DEVICE_ATTR(set_threshould, S_IRUGO, set_threshold_mode_show, NULL);
#endif /* ENABLE_NOISE_TEST_MODE */

extern int CMC623_tuning_load_from_file(void);

static ssize_t firmware1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if !defined(CMC623_TUNING)
    u8 version = p_qt602240_data->info->version;
    u8 build = p_qt602240_data->info->build;
    printk(KERN_DEBUG "[TSP] Firmware %x %x\n", version, build);

    return sprintf(buf, "%x.%x\n", version>>4, version&0xf);
#else
    int ret;

	printk(KERN_DEBUG "[TSP] CMC623 tunning start ret = %d\n", ret);
    ret = CMC623_tuning_load_from_file();
    printk(KERN_DEBUG "[TSP] CMC623 tunning start ret = %d\n", ret);

    if(ret<0)
        return sprintf(buf, "CMC623 Tunning FAIL (%d)!!\n", ret);
    else
        return sprintf(buf, "CMC623 Tunning OK (%d)!!\n" ,ret);
#endif
}

static ssize_t firmware2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%x.%x\n", BUILTIN_FIRMWARE_VERSION>>4, BUILTIN_FIRMWARE_VERSION&0xf);
}

static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", touchscreen_config.tchthr);
}

static ssize_t key_threshold_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size)
{
    int i, ret =0;
    if(sscanf(buf,"%d",&i)==1)
    {
        touchscreen_config.tchthr = i;
        printk(KERN_DEBUG "[TSP] threshold is changed to %d\n",i);
    }
    else
        printk(KERN_DEBUG "[TSP] threshold write error\n");

    ret = qt602240_write_object(p_qt602240_data, QT602240_TOUCH_MULTI,
                QT602240_TOUCH_TCHTHR, i);
    if (ret < 0)
    {
        printk(KERN_DEBUG "[TSP] error %s: write_object\n", __func__);
    }

    return size;
}

static DEVICE_ATTR(firmware1, S_IRUGO, firmware1_show, NULL);
static DEVICE_ATTR(firmware2, S_IRUGO, firmware2_show, NULL);
static DEVICE_ATTR(key_threshold, S_IRUGO | S_IWUSR | S_IWGRP, key_threshold_show, key_threshold_store);

#if defined (CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
static ssize_t tsp_filter_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t size)
{
    int i, ret =0;
    if(sscanf(buf,"%d",&i)==1)
    {
        if( i == 0x1)
        {
            gbfilter = true;
        }
        else
        {
            gbfilter = false;
        }
    }
    else
        printk(KERN_DEBUG "[TSP] threshold write error\n");

    return size;
}
static DEVICE_ATTR(tsp_filter, S_IWUSR | S_IWGRP, NULL, tsp_filter_store);
#endif

static int __devinit qt602240_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct qt602240_data *data;
    struct input_dev *input_dev;
    struct device *ts_dev;
#if defined (KEY_LED_CONTROL)
    struct class *leds_class;
    struct device *led_dev;
#endif      //KEY_LED_CONTROL

    int ret;
    int i;

    data = kzalloc(sizeof(struct qt602240_data), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!data || !input_dev) {
        dev_err(&client->dev, "Failed to allocate memory\n");
        ret = -ENOMEM;
        goto err_free_mem;
    }

    client->irq = IRQ_TOUCH_INT;

    tsp_wq = create_singlethread_workqueue("tsp_wq");

    INIT_WORK(&data->ta_work, qt602240_ta_worker);
    INIT_WORK(&data->tsp_work, qt602240_tsp_worker);

    data->client = client;
    data->input_dev = input_dev;
    data->irq = client->irq;

    data->gpio_en = OMAP_GPIO_TOUCH_EN;

    if(gpio_is_valid(data->gpio_en))
    {
        if(gpio_request(data->gpio_en, NULL))
        {
            printk("[TSP] Can't request ENABLE gpio\n");
        }
        gpio_direction_output(data->gpio_en, 1);
    }

    mdelay(50);

    input_dev->name = "touchscreen";
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &client->dev;

    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(BTN_TOUCH, input_dev->keybit);
    set_bit(EV_ABS, input_dev->evbit);

    input_set_abs_params(input_dev, ABS_MT_POSITION_X,
            0, QT602240_MAX_XC-1, 0, 0);

    input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
            0, QT602240_MAX_YC-1, 0, 0);

    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
            0, QT602240_MAX_PRESSURE, 0, 0);

    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
            0, QT602240_MAX_SIZE, 0, 0);

    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
            0, MAX_USING_FINGER_NUM - 1, 0, 0);

        input_dev->keycode = tsp_keycodes;

        for(i = 0; i < NUMOFKEYS; i++)
        {
            input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
            tsp_keystatus[i] = KEY_RELEASE;
        }

    input_set_drvdata(input_dev, data);

        p_qt602240_data = data;

    /* for multi-touch */
    for (i=0; i<MAX_USING_FINGER_NUM ; i++)        // touchscreen_config.numtouch is 5
        fingerInfo[i].pressure = -1;

    ret = qt602240_initialize(data);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to initialize TSP IC\n");
        goto err_free_object;
    }

    for(i = 0; i < 3; i++)
            qt602240_input_read(data);

        ret = request_irq(data->irq, qt602240_interrupt, IRQF_TRIGGER_LOW,
            client->dev.driver->name, data);

    if (ret < 0) {
        dev_err(&client->dev, "Failed to register interrupt\n");
        goto err_free_object;
    }

    ret = input_register_device(input_dev);
    if (ret < 0)
    {
        dev_err(&client->dev, "Failed to register as input device\n");
        goto err_free_irq;
    }

    ret = sysfs_create_group(&client->dev.kobj, &qt602240_attr_group);
    if (ret)
    {
        dev_err(&client->dev, "Failed to create sysfs\n");
        goto err_free_irq;
    }

    i2c_set_clientdata(client, data);

    ts_dev = device_create(sec_class, NULL, 0, NULL, "ts");
    if (IS_ERR(ts_dev))
        pr_err("Failed to create device(ts)!\n");

#ifdef TOUCH_BOOST
	if (device_create_file(ts_dev, &dev_attr_touch_boost) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_touch_boost.attr.name);
#endif

    if (device_create_file(ts_dev, &dev_attr_firmware1) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_firmware1.attr.name);

    if (device_create_file(ts_dev, &dev_attr_firmware2) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_firmware2.attr.name);

    if (device_create_file(ts_dev, &dev_attr_key_threshold) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_key_threshold.attr.name);

#if defined (CONFIG_TARGET_LOCALE_KOR) || defined (CONFIG_TARGET_LOCALE_USAGSM)
    if (device_create_file(ts_dev, &dev_attr_tsp_filter) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_tsp_filter.attr.name);
#endif

#ifdef ENABLE_NOISE_TEST_MODE
    qt602240_noise_test = device_create(sec_class, NULL, 0, NULL, "qt602240_noise_test");

    if (IS_ERR(qt602240_noise_test))
        printk(KERN_ERR "Failed to create device(qt602240_noise_test)!\n");

    if (device_create_file(qt602240_noise_test, &dev_attr_set_refer0)< 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer0.attr.name);
    if (device_create_file(qt602240_noise_test, &dev_attr_set_delta0) < 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta0.attr.name);
    if (device_create_file(qt602240_noise_test, &dev_attr_set_refer1)< 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer1.attr.name);
    if (device_create_file(qt602240_noise_test, &dev_attr_set_delta1) < 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta1.attr.name);
    if (device_create_file(qt602240_noise_test, &dev_attr_set_refer2)< 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer2.attr.name);
    if (device_create_file(qt602240_noise_test, &dev_attr_set_delta2) < 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta2.attr.name);
    if (device_create_file(qt602240_noise_test, &dev_attr_set_refer3)< 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer3.attr.name);
    if (device_create_file(qt602240_noise_test, &dev_attr_set_delta3) < 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta3.attr.name);
    if (device_create_file(qt602240_noise_test, &dev_attr_set_refer4)< 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer4.attr.name);
    if (device_create_file(qt602240_noise_test, &dev_attr_set_delta4) < 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta4.attr.name);
    if (device_create_file(qt602240_noise_test, &dev_attr_set_threshould) < 0)
        printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_threshould.attr.name);

#endif

	if (device_create_file(ts_dev, &dev_attr_bootcomplete) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_bootcomplete.attr.name);

#ifdef TOUCH_BOOST
  	init_timer(&opp_set_timer);
  	opp_set_timer.data = 0; 
  	opp_set_timer.function = tsc_timer_out;
	INIT_WORK(&constraint_wq, tsc_remove_constraint_handler);
#endif

#if defined (KEY_LED_CONTROL)
        init_led();

        leds_class = class_create(THIS_MODULE, "leds");
        if (IS_ERR(leds_class))
            return PTR_ERR(leds_class);

        led_dev = device_create(leds_class, NULL, 0, NULL, "button-backlight");

        if (device_create_file(led_dev, &dev_attr_brightness) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
#endif        //KEY_LED_CONTROL

#ifdef CONFIG_HAS_EARLYSUSPEND
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    data->early_suspend.suspend = qt602240_early_suspend;
    data->early_suspend.resume = qt602240_late_resume;
    register_early_suspend(&data->early_suspend);
#endif    /* CONFIG_HAS_EARLYSUSPEND */

    qt60224_notfound_flag = 0;

    if(is_boot_state) {
        qt602240_inform_charger_connection(2);
        set_autocal(10); // holding 2 seconds on touch point makes calibration automatically
        INIT_WORK(&autocal_work, disable_autocal);
        init_timer(&autocal_timer);
        autocal_timer.data = 0;
        autocal_timer.function = autocal_timer_out;
    }

    return 0;

err_free_irq:
    free_irq(client->irq, data);
err_free_object:
    kfree(data->object_message);
    kfree(data->object_table);
    kfree(data->info);
err_free_mem:
    input_free_device(input_dev);
    kfree(data);
    return ret;
}

static int __devexit qt602240_remove(struct i2c_client *client)
{
    struct qt602240_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&data->early_suspend);
#endif    /* CONFIG_HAS_EARLYSUSPEND */

    free_irq(data->irq, data);
    input_unregister_device(data->input_dev);
    kfree(data->object_message);
    kfree(data->object_table);
    kfree(data->info);
    kfree(data);

    sysfs_remove_group(&client->dev.kobj, &qt602240_attr_group);
    i2c_set_clientdata(client, NULL);

    return 0;
}

static void qt602240_shutdown(struct i2c_client *client)
{
    struct qt602240_data *data = i2c_get_clientdata(client);

    free_irq(data->irq, data);
    gpio_set_value(data->gpio_en, 0);
#if defined (KEY_LED_CONTROL)
    touch_led_on(0);
#endif      //KEY_LED_CONTROL
}

static void qt602240_suspend(struct i2c_client *client,  pm_message_t state)
{
    struct qt602240_data *data = i2c_get_clientdata(client);

	if (down_interruptible(&g_tsp_mutex))
		return;
	is_suspend_state = 1;
	
    /* touch disable */
    qt602240_write_object(data, QT602240_TOUCH_MULTI,
            QT602240_TOUCH_CTRL, 0);

    qt602240_write_object(data, QT602240_GEN_POWER,
            QT602240_POWER_IDLEACQINT, 0);

    qt602240_write_object(data, QT602240_GEN_POWER,
            QT602240_POWER_ACTVACQINT, 0);
    up(&g_tsp_mutex);

	msleep(100);

    disable_irq(data->irq);
    flush_workqueue(tsp_wq);

    /* for multi-touch */
    release_all_fingers(data->input_dev);


#if defined (KEY_LED_CONTROL)
    touch_led_on(0);
#endif      //KEY_LED_CONTROL

    qt_timer_state = 0;

}

static void qt602240_resume(struct i2c_client *client)
{
    int ret = 0;
    struct qt602240_data *data = i2c_get_clientdata(client);

    while(1)
    {
        /* soft reset */
        qt602240_write_object(data, QT602240_GEN_COMMAND,
                QT602240_COMMAND_RESET, 1);

        /* wait for soft reset */
        msleep(100);
        if (ret < 0)
        {
            printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
        }
        else
            break;
    }

    calibrate_chip(data);

    if(is_suspend_state==2) {
        qt602240_inform_charger_connection(set_mode_for_ta);
    }
    is_suspend_state = 0;
    enable_autocal_timer(10);
// !! Should not turn off
//#if defined (KEY_LED_CONTROL)
//    init_led();
//#endif      //KEY_LED_CONTROL
    enable_irq(data->irq);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qt602240_early_suspend(struct early_suspend *early_sus)
{
    struct qt602240_data *data = container_of(early_sus,
            struct qt602240_data, early_suspend);
   qt602240_suspend(data->client, PMSG_SUSPEND);
}

static void qt602240_late_resume(struct early_suspend *early_sus)
{
    struct qt602240_data *data = container_of(early_sus,
            struct qt602240_data, early_suspend);
    qt602240_resume(data->client);
}
#endif    // End of CONFIG_HAS_EARLYSUSPEND

static const struct i2c_device_id qt602240_id[] = {
    { "qt602240_ts", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, qt602240_id);

static struct i2c_driver qt602240_driver = {
    .driver = {
        .name = "qt602240_ts",
    },
    .probe        = qt602240_probe,
    .remove        = __devexit_p(qt602240_remove),
    .shutdown        = qt602240_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = qt602240_suspend,
    .resume        = qt602240_resume,
#endif
    .id_table    = qt602240_id,
};

static int __init qt602240_init(void)
{
    return i2c_add_driver(&qt602240_driver);
}

static void __exit qt602240_exit(void)
{
    i2c_del_driver(&qt602240_driver);
}

module_init(qt602240_init);
module_exit(qt602240_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("AT42QT602240 Touchscreen driver");
MODULE_LICENSE("GPL");
