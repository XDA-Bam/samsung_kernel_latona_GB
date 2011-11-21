/*
 *  Copyright (C) 2010, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#define RESERVED_T0                               0u
#define RESERVED_T1                               1u
#define DEBUG_DELTAS_T2                           2u
#define DEBUG_REFERENCES_T3                       3u
#define DEBUG_SIGNALS_T4                          4u
#define GEN_MESSAGEPROCESSOR_T5                   5u
#define GEN_COMMANDPROCESSOR_T6                   6u
#define GEN_POWERCONFIG_T7                        7u
#define GEN_ACQUISITIONCONFIG_T8                  8u
#define TOUCH_MULTITOUCHSCREEN_T9                 9u
#define TOUCH_SINGLETOUCHSCREEN_T10               10u
#define TOUCH_XSLIDER_T11                         11u
#define TOUCH_YSLIDER_T12                         12u
#define TOUCH_XWHEEL_T13                          13u
#define TOUCH_YWHEEL_T14                          14u
#define TOUCH_KEYARRAY_T15                        15u
#define PROCG_SIGNALFILTER_T16                    16u
#define PROCI_LINEARIZATIONTABLE_T17              17u
#define SPT_COMCONFIG_T18                         18u
#define SPT_GPIOPWM_T19                           19u
#define PROCI_GRIPFACESUPPRESSION_T20             20u
#define RESERVED_T21                              21u
#define PROCG_NOISESUPPRESSION_T22                22u
#define TOUCH_PROXIMITY_T23	                      23u
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24        24u
#define SPT_SELFTEST_T25                          25u
#define DEBUG_CTERANGE_T26                        26u
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27        27u
#define SPT_CTECONFIG_T28                         28u
#define SPT_GPI_T29                               29u
#define SPT_GATE_T30                              30u
#define TOUCH_KEYSET_T31                          31u
#define TOUCH_XSLIDERSET_T32                      32u


#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
// #include <asm/semaphore.h>
#include <linux/semaphore.h>	// ryun
#include <asm/mach-types.h>

//#include <asm/arch/gpio.h>
//#include <asm/arch/mux.h>
#include <plat/gpio.h>	//ryun
#include <plat/mux.h>	//ryun 
#include <linux/delay.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <linux/firmware.h>
//#include "dprintk.h"
//#include "message.h"
#include <linux/time.h>

#include <linux/i2c/twl.h>	// ryun 20091125 
#include <linux/earlysuspend.h>	// ryun 20200107 for early suspend
#include <plat/omap-pm.h>	// ryun 20200107 for touch boost

#ifdef TOUCH_PROC
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#endif

#include <linux/wakelock.h>
#include "atmel_touch.h"

#ifdef CONFIG_TOUCHKEY_LOCK
extern unsigned int touchkey_lock_flag;
#endif

extern unsigned char g_version;
extern Atmel_model_type g_model;
extern uint8_t cal_check_flag;//20100208
extern unsigned int qt_time_point;
extern unsigned int qt_timer_state ;
extern int ta_state;
//#define TEST_TOUCH_KEY_IN_ATMEL

int fh_err_count;
extern void set_frequency_hopping_table(int mode);

extern int is_suspend_state; //to check suspend mode

// for reseting on I2C fail
extern uint8_t touch_state;
extern uint8_t get_message(void);

// OMAP3630 OPP Clock Frequency Table
#define VDD1_OPP4_FREQ         S1000M
#define VDD1_OPP3_FREQ         S800M
#define VDD1_OPP2_FREQ         S600M
#define VDD1_OPP1_FREQ         S300M

//modified for samsung customisation -touchscreen 
unsigned short enable_touch_boost;
unsigned int g_firmware_ret = 2;
static ssize_t ts_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t ts_store(struct kobject *k, struct kobj_attribute *,
			  const char *buf, size_t n);
static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_update_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size);
static ssize_t firmware_version_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_version_read_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf);
//modified for samsung customisation

const unsigned char fw_bin_version = 0x16;
const unsigned char fw_bin_build = 0xAB;

#define __CONFIG_ATMEL__

#define TOUCHSCREEN_NAME			"touchscreen"
#define DEFAULT_PRESSURE_UP		0
#define DEFAULT_PRESSURE_DOWN		256

static struct touchscreen_t tsp;
//static struct work_struct tsp_work;	//	 ryun
static struct workqueue_struct *tsp_wq;
//static int g_touch_onoff_status = 0;
static int g_enable_touchscreen_handler = 0;	// fixed for i2c timeout error.
//static unsigned int g_version_read_addr = 0;
//static unsigned short g_position_read_addr = 0;

#define IS_ATMEL	1	// ryun

#define DRIVER_FILTER

#define TOUCH_MENU	  KEY_MENU
#define TOUCH_SEARCH  KEY_SEARCH 
#define TOUCH_HOME  KEY_HOME
#define TOUCH_BACK	  KEY_BACK

#if defined(CONFIG_MACH_SAMSUNG_LATONA) || defined(CONFIG_MACH_SAMSUNG_P1WIFI)
#define MAX_TOUCH_X_RESOLUTION	480
#define MAX_TOUCH_Y_RESOLUTION	800
int atmel_ts_tk_keycode[] =
{ TOUCH_MENU, TOUCH_BACK };
#endif

struct touchscreen_t;

struct touchscreen_t {
	struct input_dev * inputdevice;
	int touched;
	int irq;
	int irq_type;
	int irq_enabled;
	struct ts_device *dev;
	struct early_suspend	early_suspend;// ryun 20200107 for early suspend
	struct work_struct  tsp_work;	// ryun 20100107 
	struct timer_list opp_set_timer;	// ryun 20100107 for touch boost
	struct work_struct constraint_wq;
	int opp_high;	// ryun 20100107 for touch boost	
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void atmel_ts_early_suspend(struct early_suspend *h);
void atmel_ts_late_resume(struct early_suspend *h);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef FEATURE_MOVE_LIMIT
int pre_x, pre_y, pre_size;
#endif

#ifdef TOUCH_PROC
int touch_proc_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
int touch_proc_read(char *page, char **start, off_t off,int count, int *eof, void *data);
int touch_proc_write(struct file *file, const char __user *buffer, unsigned long count, void *data);

#define IOCTL_TOUCH_PROC 'T'
enum 
{
        TOUCH_GET_VERSION = _IOR(IOCTL_TOUCH_PROC, 0, char*),
        TOUCH_GET_T_KEY_STATE = _IOR(IOCTL_TOUCH_PROC, 1, int),
        TOUCH_GET_SW_VERSION = _IOR(IOCTL_TOUCH_PROC, 2, char*),
};

const char fw_version[10]="0X16";

struct proc_dir_entry *touch_proc;
struct file_operations touch_proc_fops = 
{
        .ioctl=touch_proc_ioctl,
};
#endif

struct wake_lock tsp_firmware_wake_lock;

// [[ ryun 20100113 
typedef struct
{
	int x;
	int y;
	int press;
	int size;
} dec_input;

static dec_input touch_info[MAX_TOUCH_NUM] = {0};
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
static int prev_touch_count = 0;
#endif

#define REPORT( touch, width, x, y, id) \
{	input_report_abs(tsp.inputdevice, ABS_MT_TOUCH_MAJOR, touch ); \
	input_report_abs(tsp.inputdevice, ABS_MT_WIDTH_MAJOR, width ); \
	input_report_abs(tsp.inputdevice, ABS_MT_POSITION_X, x); \
	input_report_abs(tsp.inputdevice, ABS_MT_POSITION_Y, y); \
	input_report_abs(tsp.inputdevice, ABS_MT_TRACKING_ID, id); \
	input_mt_sync(tsp.inputdevice); }

// ]] ryun 20100113 

#ifdef ENABLE_NOISE_TEST_MODE
extern ssize_t set_refer0_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_refer1_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_refer2_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_refer3_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_refer4_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_refer5_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_refer6_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_delta0_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_delta1_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_delta2_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_delta3_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_delta4_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_delta5_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_delta6_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t threshold_show(struct device *dev, struct device_attribute *attr, char *buf);

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
static DEVICE_ATTR(set_refer5, S_IRUGO, set_refer5_mode_show, NULL);
static DEVICE_ATTR(set_delta5, S_IRUGO, set_delta5_mode_show, NULL);
static DEVICE_ATTR(set_refer6, S_IRUGO, set_refer6_mode_show, NULL);
static DEVICE_ATTR(set_delta6, S_IRUGO, set_delta6_mode_show, NULL);
static DEVICE_ATTR(set_threshold, S_IRUGO, threshold_show, NULL);
#endif /* ENABLE_NOISE_TEST_MODE */

void read_func_for_only_single_touch(struct work_struct *work);
void read_func_for_multi_touch(struct work_struct *work);
void keyarray_handler(uint8_t * atmel_msg);
void handle_multi_touch(uint8_t *atmel_msg);
void handle_keyarray(uint8_t * atmel_msg);
void initialize_multi_touch(void);



void (*atmel_handler_functions[MODEL_TYPE_MAX])(struct work_struct *work) =
{
    read_func_for_only_single_touch, // default handler
    read_func_for_multi_touch, // LATONA
};

static irqreturn_t touchscreen_handler(int irq, void *dev_id);
void set_touch_irq_gpio_init(void);
void set_touch_irq_gpio_disable(void);	// ryun 20091203
void clear_touch_history(void);

//samsung customisation
static struct kobj_attribute touch_boost_attr =     __ATTR(touch_boost, 0644, ts_show, ts_store);
static struct kobj_attribute firmware_attr =        __ATTR(set_qt_firm_update, 0220, NULL, firmware_update_store);
static struct kobj_attribute firmware_binary_attr = __ATTR(set_qt_firm_version, 0444, firmware_version_show, NULL);
static struct kobj_attribute firmware_binary_read_attr = __ATTR(set_qt_firm_version_read, 0444, firmware_version_read_show, NULL);
static struct kobj_attribute firmware_ret_attr =    __ATTR(set_qt_firm_status, 0444, firmware_ret_show, NULL);

/*------------------------------ for tunning ATmel - start ----------------------------*/
extern  ssize_t set_power_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_acquisition_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_acquisition_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_touchscreen_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_touchscreen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_keyarray_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_keyarray_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_grip_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_grip_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_noise_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_noise_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_total_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_total_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_write_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_write_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);

static DEVICE_ATTR(set_power, S_IRUGO | S_IWUSR, set_power_show, set_power_store);
static DEVICE_ATTR(set_acquisition, S_IRUGO | S_IWUSR, set_acquisition_show, set_acquisition_store);
static DEVICE_ATTR(set_touchscreen, S_IRUGO | S_IWUSR, set_touchscreen_show, set_touchscreen_store);
static DEVICE_ATTR(set_keyarray, S_IRUGO | S_IWUSR, set_keyarray_show, set_keyarray_store);
static DEVICE_ATTR(set_grip , S_IRUGO | S_IWUSR, set_grip_show, set_grip_store);
static DEVICE_ATTR(set_noise, S_IRUGO | S_IWUSR, set_noise_show, set_noise_store);
static DEVICE_ATTR(set_total, S_IRUGO | S_IWUSR, set_total_show, set_total_store);
static DEVICE_ATTR(set_write, S_IRUGO | S_IWUSR, set_write_show, set_write_store);

static ssize_t bootcomplete_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n);
static struct kobj_attribute bootcomplete_attr =        __ATTR(bootcomplete, 0220, NULL, bootcomplete_store);

extern void bootcomplete(void);
extern void enable_autocal_timer(unsigned int value);
static ssize_t bootcomplete_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1) {
		printk(KERN_ERR "bootcomplete_store: Invalid value\n");
		return -EINVAL;
	}

	if (attr == &bootcomplete_attr) {
		bootcomplete();
	} 
	else {
		return -EINVAL;
	}

	return n;
}

/*------------------------------ for tunning ATmel - end ----------------------------*/

extern void restore_acquisition_config(void);
extern void restore_power_config(void);
extern uint8_t calibrate_chip(void);

static unsigned char menu_button = 0;
static unsigned char back_button = 0;

extern uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance);

void clear_touch_history(void)
{
	int i;
	// Clear touch history
	for(i=0;i<MAX_TOUCH_NUM;i++)
	{
		if(touch_info[i].press == -1) continue;
		if(touch_info[i].press > 0)
		{
			touch_info[i].press = 0;
			REPORT( touch_info[i].press, touch_info[i].size, touch_info[i].x, touch_info[i].y, i);
		}
		if(touch_info[i].press == 0) touch_info[i].press = -1;
	}
	input_sync(tsp.inputdevice);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
	prev_touch_count = 0;
#endif
}

static ssize_t ts_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	if (attr == &touch_boost_attr)
		return sprintf(buf, "%hu\n", enable_touch_boost);
	
	else
		return -EINVAL;

}
static ssize_t ts_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1) {
		printk(KERN_ERR "ts_store: Invalid value\n");
		return -EINVAL;
	}

	if (attr == &touch_boost_attr) {
		enable_touch_boost = value;
	} 

	 else {
		return -EINVAL;
	}

	return n;
}
//samsung customisation

//[[ ryun 20100107 for touch boost
static void tsc_timer_out (unsigned long v)
	{
		schedule_work(&(tsp.constraint_wq));
		return;
	}

void tsc_remove_constraint_handler(struct work_struct *work)
{
		//omap_pm_set_min_mpu_freq(&(tsp.dev), VDD1_OPP1_FREQ);     //Jyothi
                tsp.opp_high = 0;
}

//]] ryun 20200107 for touch boost

/* ryun 20091125 
struct omap3430_pin_config touch_i2c_gpio_init[] = {
	
	OMAP3430_PAD_CFG("TOUCH I2C SCL", OMAP3430_PAD_I2C_TOUCH_SCL, PAD_MODE0, PAD_INPUT_PU, PAD_OFF_PU, PAD_WAKEUP_NA),
	OMAP3430_PAD_CFG("TOUCH I2C SDA", OMAP3430_PAD_I2C_TOUCH_SDA, PAD_MODE0, PAD_INPUT_PU, PAD_OFF_PU, PAD_WAKEUP_NA),
};

struct omap3430_pin_config touch_irq_gpio_init[] = {
	OMAP3430_PAD_CFG("TOUCH INT",		OMAP3430_PAD_TOUCH_INT, PAD_MODE4, PAD_INPUT_PU, PAD_OFF_PU, PAD_WAKEUP_NA),
};
*/
void set_touch_i2c_gpio_init(void)
{
	printk(KERN_DEBUG "[TSP] %s() \n", __FUNCTION__);
}

void set_touch_irq_gpio_init(void)
{
	printk(KERN_DEBUG "[TSP] %s() \n", __FUNCTION__);
	gpio_direction_input(OMAP_GPIO_TOUCH_INT);
}

// [[ ryun 20091203
void set_touch_irq_gpio_disable(void)
{
	printk(KERN_DEBUG "[TSP] %s() \n", __FUNCTION__);
	if(g_enable_touchscreen_handler == 1)
	{
		free_irq(tsp.irq, &tsp);
	gpio_free(OMAP_GPIO_TOUCH_INT);
		g_enable_touchscreen_handler = 0;
	}	
}
// ]] ryun 20091203

unsigned char get_touch_irq_gpio_value(void)			
{
	//return gpio_get_value(GPIO_TOUCH_IRQ);
	return gpio_get_value(OMAP_GPIO_TOUCH_INT);
// ***********************************************************************
// NOTE: HAL function: User adds/ writes to the body of this function
// ***********************************************************************
}

#define U8	__u8
#define  U16 	unsigned short int
#define READ_MEM_OK                 1u


extern unsigned int g_i2c_debugging_enable;
extern U8 read_mem(U16 start, U8 size, U8 *mem);
extern uint16_t message_processor_address;
extern uint8_t max_message_length;
extern uint8_t *atmel_msg;
extern uint8_t QT_Boot(uint8_t qt_force_update);
extern unsigned long simple_strtoul(const char *,char **,unsigned int);
extern unsigned char g_version, g_build, qt60224_notfound_flag;


/* firmware 2009.09.24 CHJ - end 1/2 */


void disable_tsp_irq(void)
{
	printk(KERN_DEBUG "[TSP] disabling tsp irq and flushing workqueue\n");
	disable_irq(tsp.irq);
	flush_workqueue(tsp_wq);
}

void enable_tsp_irq(void)
{
	printk(KERN_DEBUG "[TSP] enabling tsp irq\n");
	enable_irq(tsp.irq);	
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{	// v1.2 = 18 , v1.4 = 20 , v1.5 = 21
	printk(KERN_DEBUG "[TSP] QT602240 Firmware Ver.\n");
	printk(KERN_DEBUG "[TSP] version = %x\n", g_version);
	printk(KERN_DEBUG "[TSP] Build = %x\n", g_build);
	//	printk("[TSP] version = %d\n", info_block->info_id.version);
	//	sprintf(buf, "QT602240 Firmware Ver. %x\n", info_block->info_id.version);
	sprintf(buf, "QT602240 Firmware Ver. %x \nQT602240 Firmware Build. %x\n", g_version, g_build );

	return sprintf(buf, "%s", buf );
}

static ssize_t firmware_update_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *after;
	g_firmware_ret = 2;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s\n", __FUNCTION__);

	if ( value == 1 )	// auto update.
	{
		printk(KERN_DEBUG "[TSP] Firmware update start!!\n" );
		printk(KERN_DEBUG "[TSP] version = 0x%x\n", g_version );

//		if( g_version <= 22 ) 
		if(((g_version != 0x14)&&(g_version <0x16))||((g_version==0x16)&&(g_build==0xaa)))
		{			
			wake_lock(&tsp_firmware_wake_lock);
#ifdef CONFIG_HAS_EARLYSUSPEND
			unregister_early_suspend(&tsp.early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
			g_firmware_ret = QT_Boot(qt60224_notfound_flag);
#ifdef CONFIG_HAS_EARLYSUSPEND
			register_early_suspend(&tsp.early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
			qt60224_notfound_flag = 0;
			wake_unlock(&tsp_firmware_wake_lock);			
		}	
		else
		{
			g_firmware_ret = 1; 
		}
		printk(KERN_DEBUG "[TSP] Firmware result = %d\n", g_firmware_ret );

		return size;
	}
	else
	{
		g_firmware_ret = 0;
		return 0;
	}
}

static ssize_t firmware_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "[TSP] QT602240 Firmware Image Ver.\n");
	return sprintf(buf, "%d\n", fw_bin_version);
}

static ssize_t firmware_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_version);
}

static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(g_firmware_ret == 2)
	{
		return sprintf(buf, "PASS\n");
	}
	
	return sprintf(buf, "FAIL\n");
}

void initialize_multi_touch(void)
{
	int i;
	for(i = 0;i < MAX_TOUCH_NUM;i++)
	{
		touch_info[i].x = 0;
		touch_info[i].y = 0;
		touch_info[i].press = -1;
		touch_info[i].size = 0;
	}
}

void keyarray_handler(uint8_t * atmel_msg)
{
	if( (atmel_msg[2] & 0x1) && (menu_button==0) ) // menu press
	{
		menu_button = 1;
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
		printk(KERN_DEBUG "[TSP] menu_button is pressed\n");
#endif
		input_report_key(tsp.inputdevice, 139, DEFAULT_PRESSURE_DOWN);
        	input_sync(tsp.inputdevice);    		
	}
	else if( (atmel_msg[2] & 0x2) && (back_button==0) ) // back press
	{
		back_button = 1;
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
		printk(KERN_DEBUG "[TSP] back_button is pressed\n");                
#endif
		input_report_key(tsp.inputdevice, 158, DEFAULT_PRESSURE_DOWN);                
        	input_sync(tsp.inputdevice);    				
	}
	else if( (~atmel_msg[2] & (0x1)) && menu_button==1 ) // menu_release
	{
		menu_button = 0;
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
		printk(KERN_DEBUG "[TSP] menu_button is released\n");                                
#endif
		input_report_key(tsp.inputdevice, 139, DEFAULT_PRESSURE_UP);     
        	input_sync(tsp.inputdevice);    				
	}
	else if( (~atmel_msg[2] & (0x2)) && back_button==1 ) // menu_release
	{
		back_button = 0;
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
		printk(KERN_DEBUG "[TSP] back_button is released\n");
#endif
		input_report_key(tsp.inputdevice, 158, DEFAULT_PRESSURE_UP); 
        	input_sync(tsp.inputdevice);    				
	}
	else
	{
		menu_button=0; 
		back_button=0;
		printk("Unknow state of touch key\n");
	}

}

void handle_keyarray(uint8_t * atmel_msg)
{

    switch(g_model)
    {
        case LATONA:
        {
            keyarray_handler(atmel_msg);
        }
        break;
        default:
        {
            printk("[TSP][ERROR] atmel message of key_array was not handled normally\n");
        }
    }
}

#if defined(DRIVER_FILTER)
static void equalize_coordinate(bool detect, u8 id, u16 *px, u16 *py)
{
    static int tcount[MAX_TOUCH_NUM] = { 0, };
    static u16 pre_x[MAX_TOUCH_NUM][4] = {{0}, };
    static u16 pre_y[MAX_TOUCH_NUM][4] = {{0}, };
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

//            printk(KERN_DEBUG "[TSP] %d, %d, %d, %d \n", coff[0], coff[1], coff[2], coff[3]);

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
#endif  //DRIVER_FILTER

#ifdef FEATURE_MOVE_LIMIT
#define MOVE_LIMIT_SQUARE (150*150) // 100*100
#define DISTANCE_SQUARE(X1, Y1, X2, Y2)    (((X2-X1)*(X2-X1))+((Y2-Y1)*(Y2-Y1)))

int pre_x, pre_y, pre_size;

#endif

////ryun 20100208 add
extern void check_chip_calibration(unsigned char one_touch_input_flag);
void handle_multi_touch(uint8_t *atmel_msg)
{
	u16 x=0, y=0;
	unsigned int size ;	// ryun 20100113 
	static int check_flag=0; // ryun 20100113 	
	uint8_t touch_message_flag = 0;// ryun 20100208
	unsigned char one_touch_input_flag=0;
	unsigned char cal_release_number_of_check=0;
	int id;
	int i, touch_count;

	x = atmel_msg[2];
	x = x << 2;
	x = x | (atmel_msg[4] >> 6);

	y = atmel_msg[3];
	y = y << 2;
	y = y | ((atmel_msg[4] & 0x6)  >> 2);

	size = atmel_msg[5];

	/* For valid inputs. */
	if ((atmel_msg[0] > 1) && (atmel_msg[0] < MAX_TOUCH_NUM+2))
	{
		if((x > MAX_TOUCH_X_RESOLUTION) || (y > MAX_TOUCH_Y_RESOLUTION))
		{
			int repeat_count=0;
			unsigned char do_reset=1;
			unsigned char resume_success=0;

			do {
				if( do_reset )
				{
					printk(KERN_DEBUG "[TSP] Reset TSP IC on wrong coordination\n");
					gpio_direction_output(OMAP_GPIO_TOUCH_INT, 0);
					gpio_set_value(OMAP_GPIO_TOUCH_EN, 0);
					msleep(200);
					gpio_direction_output(OMAP_GPIO_TOUCH_INT, 1);
					gpio_direction_input(OMAP_GPIO_TOUCH_INT);
					gpio_set_value(OMAP_GPIO_TOUCH_EN, 1);
					msleep(80); // recommended value
					calibrate_chip();
				}

				touch_state = 0;
				get_message();

				if( (touch_state & 0x10) == 0x10 )
				{
					printk(KERN_DEBUG "[TSP] reset and calibration success\n");
					resume_success = 1;
				}
				else
				{
					printk("[TSP] retry to reset\n");
					resume_success = 0;
					do_reset = 1;
				}
			} while (resume_success != 1 && repeat_count++ < RETRY_COUNT);
			clear_touch_history();
			return;
		}

		id = atmel_msg[0] - 2;
		/* case.1 - 11000000 -> DETECT & PRESS */
		if( ( atmel_msg[1] & 0xC0 ) == 0xC0  ) 
		{
			touch_message_flag = 1;
#ifdef CONFIG_TOUCHKEY_LOCK
			touchkey_lock_flag = 1;
#endif
			touch_info[id].press = 40;
			touch_info[id].size = size;
			touch_info[id].x = x;
			touch_info[id].y = y;
#if defined(DRIVER_FILTER)
			equalize_coordinate(1, id, &touch_info[id].x, &touch_info[id].y);
#endif
		}
		/* case.2 - case 10010000 -> DETECT & MOVE */
		else if( ( atmel_msg[1] & 0x90 ) == 0x90 )
		{
			touch_message_flag = 1;
			touch_info[id].press = 40;
			touch_info[id].size = size;
			touch_info[id].x = x;
			touch_info[id].y = y;
#if defined(DRIVER_FILTER)
			equalize_coordinate(0, id, &touch_info[id].x, &touch_info[id].y);
#endif
		}
		/* case.3 - case 00100000 -> RELEASE */
		else if( ((atmel_msg[1] & 0x20 ) == 0x20))   
		{
			touch_info[id].press = 0;
			touch_info[id].size = size;
		}
		else
		{
			printk("[TSP] exception case id[%d],x=%d,y=%d\n", id, x, y);
			return;
		}

		for(i = 0, touch_count = 0;i < MAX_TOUCH_NUM;i++)
		{
			if(touch_info[i].press == -1) continue;
			REPORT( touch_info[i].press, touch_info[i].size, touch_info[i].x, touch_info[i].y, i);
			if(touch_info[i].press == 0) touch_info[i].press = -1;
			else touch_count++;
		}
		input_sync(tsp.inputdevice);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
		if(prev_touch_count != touch_count) {
			printk(KERN_DEBUG "[TSP] id[%d],x=%d,y=%d,%dpoint(s)\n", id, x, y, touch_count);
			prev_touch_count = touch_count;
		}
#endif
#ifdef CONFIG_TOUCHKEY_LOCK
		if(touch_count == 0) touchkey_lock_flag = 0;
#endif
	}
	/* case.4 - Palm Touch & Unknow sate */
	else if ( atmel_msg[0] == 14 )
	{
		if((atmel_msg[1]&0x01) == 0x00)   
		{
#ifdef CONFIG_TOUCHKEY_LOCK
			touchkey_lock_flag = 0;
#endif
			printk("[TSP] Palm Touch! - %d <released from palm touch>\n", atmel_msg[1]);
			clear_touch_history();
		}
		else
		{
#ifdef CONFIG_TOUCHKEY_LOCK
			touchkey_lock_flag = 1;
#endif
			printk("[TSP] test Palm Touch! - %d <face suppresstion status bit>\n", atmel_msg[1] );	
			touch_message_flag = 1;// ryun 20100208
			enable_autocal_timer(10);
		}
	}	
	else if ( atmel_msg[0] == 0 )
	{
	printk("[TSP] Error : %d - What happen to TSP chip?\n", __LINE__ );

//		touch_hw_rst( );  // TOUCH HW RESET
//		TSP_set_for_ta_charging( config_set_mode );   // 0 for battery, 1 for TA, 2 for USB
	}

	if ( atmel_msg[0] == 1 )
	{
		if((atmel_msg[1]&0x10) == 0x10)
		{
			printk(KERN_DEBUG "[TSP] The device is calibrating...\n");
			cal_check_flag = 1;
			qt_timer_state = 0;
			qt_time_point = 0;
		}
	}
	else if(touch_message_flag && (cal_check_flag))
	{
		check_chip_calibration(one_touch_input_flag);
		if(touch_count >= 2)
		{
			enable_autocal_timer(10);
		}
	}
}    

void read_func_for_only_single_touch(struct work_struct *work)
{
//	uint8_t ret_val = MESSAGE_READ_FAILED;
	u16 x=0, y=0;
	u16 x480, y800, press;
	int status;
	u8 family_id;
//	PRINT_FUNCTION_ENTER;
	struct touchscreen_t *ts = container_of(work,
					struct touchscreen_t, tsp_work);
	if(enable_touch_boost) //added for touchscreen boost,samsung customisation,enabled in init.rc
	{	
		if (timer_pending(&ts->opp_set_timer))
			del_timer(&ts->opp_set_timer);
		//omap_pm_set_min_mpu_freq(&(ts->dev), VDD1_OPP4_FREQ);
		mod_timer(&ts->opp_set_timer, jiffies + (100 * HZ) / 1000);
	}

//	g_i2c_debugging_enable = 0;
	if(read_mem(message_processor_address, max_message_length, atmel_msg) == READ_MEM_OK)
	{
//		g_i2c_debugging_enable = 0;
		
		if(atmel_msg[0]<2 || atmel_msg[0]>11)
		{
			g_i2c_debugging_enable = 0;
			printk("[TSP][ERROR] %s() - read fail \n", __FUNCTION__);
			enable_irq(tsp.irq);
			return ; 
		}

		//printk(DCM_INP, "[TSP][REAL]x: 0x%02x, 0x%02x \n", atmel_msg[2], atmel_msg[3]);
		x = atmel_msg[2];
		x = x << 2;
		x = x | (atmel_msg[4] >> 6);

		y = atmel_msg[3];
		y = y << 2;
		y = y | ((atmel_msg[4] & 0x6)  >> 2);
		x480 = x;
		y800 = y;
		if( ((atmel_msg[1] & 0x40) == 0x40  || (atmel_msg[1] & 0x10) == 0x10) && (atmel_msg[1] & 0x20) == 0)
			press = 1;
		else if( (atmel_msg[1] & 0x40) == 0 && (atmel_msg[1] & 0x20) == 0x20)
			press = 0;
		else
		{
			//press = 3;
			//printk("[TSP][WAR] unknow state : 0x%x\n", msg[1]);
			enable_irq(tsp.irq);
			return;
		}

		if(press == 1)
		{
			input_report_abs(tsp.inputdevice, ABS_X, x480);
			input_report_abs(tsp.inputdevice, ABS_Y, y800);
			input_report_key(tsp.inputdevice, BTN_TOUCH, DEFAULT_PRESSURE_DOWN);
			input_report_abs(tsp.inputdevice, ABS_PRESSURE, DEFAULT_PRESSURE_DOWN);
			input_sync(tsp.inputdevice);
			if(en_touch_log)
			{
				printk("[TSP][DOWN] id=%d, x=%d, y=%d, press=%d \n",(int)atmel_msg[0], x480, y800, press);
				en_touch_log = 0;
			}
		}else if(press == 0)
		{
			input_report_key(tsp.inputdevice, BTN_TOUCH, DEFAULT_PRESSURE_UP	);
			input_report_abs(tsp.inputdevice, ABS_PRESSURE, DEFAULT_PRESSURE_UP);
			input_sync(tsp.inputdevice);
			printk("[TSP][UP] id=%d, x=%d, y=%d, press=%d \n",(int)atmel_msg[0], x480, y800, press);
			en_touch_log = 1;
		}
//		ret_val = MESSAGE_READ_OK;
	}else
	{
//		g_i2c_debugging_enable = 0;
		printk("[TSP][ERROR] %s() - read fail \n", __FUNCTION__);
	}
//	PRINT_FUNCTION_EXIT;
	enable_irq(tsp.irq);
	return;
}

void check_frequency_hopping_error(uint8_t *atmel_msg)
{
	if(ta_state) {
		if(atmel_msg[1] & 0x8) {
			if(++fh_err_count == 12) fh_err_count = 0;
			if(!(fh_err_count % 3)) {
				set_frequency_hopping_table(fh_err_count/3);
			}
		}
	}
}

void read_func_for_multi_touch(struct work_struct *work)
{
	uint8_t object_type, instance;

//	PRINT_FUNCTION_ENTER;
	struct touchscreen_t *ts = container_of(work,
					struct touchscreen_t, tsp_work);

	if(enable_touch_boost) //added for touchscreen boost,samsung customisation,enabled in init.rc
	{
		if (timer_pending(&ts->opp_set_timer))
			del_timer(&ts->opp_set_timer);
		//omap_pm_set_min_mpu_freq(&(ts->dev), VDD1_OPP4_FREQ);
		mod_timer(&ts->opp_set_timer, jiffies + (500 * HZ) / 1000);
	}


//	g_i2c_debugging_enable = 0;
	if(read_mem(message_processor_address, max_message_length, atmel_msg) != READ_MEM_OK)
	{
//		g_i2c_debugging_enable = 0;
		printk("[TSP][ERROR] %s() - read fail \n", __FUNCTION__);
		enable_irq(tsp.irq);
		return ;
	}

	object_type = report_id_to_type(atmel_msg[0], &instance);


	switch(object_type)
	{
		case GEN_COMMANDPROCESSOR_T6:
		case PROCI_GRIPFACESUPPRESSION_T20:
		case TOUCH_MULTITOUCHSCREEN_T9:
			handle_multi_touch(atmel_msg);
			break;
		case TOUCH_KEYARRAY_T15:
        handle_keyarray(atmel_msg);
			break;
		case PROCG_NOISESUPPRESSION_T22:
			check_frequency_hopping_error(atmel_msg);
		case SPT_GPIOPWM_T19:
		case PROCI_ONETOUCHGESTUREPROCESSOR_T24:
		case PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
		case SPT_SELFTEST_T25:
		case SPT_CTECONFIG_T28:
		default:
			printk(KERN_DEBUG "[TSP] 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", atmel_msg[0], atmel_msg[1],atmel_msg[2], 
				    atmel_msg[3], atmel_msg[4],atmel_msg[5], atmel_msg[6], atmel_msg[7], atmel_msg[8]);
			break;
	};

	//if( atmel_msg[0] == 12 || atmel_msg[0] == 13)
	//{
       // handle_keyarray(atmel_msg);
	//}

//	g_i2c_debugging_enable = 0;

      //if( atmel_msg[0] >=2 && atmel_msg[0] <=11 )
      //{
      //    handle_multi_touch(atmel_msg);
      //}    

	enable_irq(tsp.irq);

	return;
}

void atmel_touchscreen_read(struct work_struct *work)
{
    atmel_handler_functions[g_model](work);
}

static irqreturn_t touchscreen_handler(int irq, void *dev_id)
{
	disable_irq_nosync(irq);
	queue_work(tsp_wq, &tsp.tsp_work);
	return IRQ_HANDLED;
}

extern void atmel_touch_probe(void);
//extern void atmel_touchscreen_read(struct work_struct *work); 

int  enable_irq_handler(void)
{
	if (tsp.irq != -1)
	{
		tsp.irq = OMAP_GPIO_IRQ(OMAP_GPIO_TOUCH_INT);	// ryun .. move to board-xxx.c
		tsp.irq_type = IRQF_TRIGGER_LOW; 

		if (request_irq(tsp.irq, touchscreen_handler, tsp.irq_type, TOUCHSCREEN_NAME, &tsp))	
		{
			printk("[TSP][ERR] Could not allocate touchscreen IRQ!\n");
			tsp.irq = -1;
			input_free_device(tsp.inputdevice);
			return -EINVAL;
		}
		else
		{
			printk(KERN_DEBUG "[TSP] register touchscreen IRQ!\n");
		}

		if(g_enable_touchscreen_handler == 0)
			g_enable_touchscreen_handler = 1;

		tsp.irq_enabled = 1;
	}
	return 0;
}

#ifdef ENABLE_NOISE_TEST_MODE
extern struct class *sec_class;
#endif

static int __init touchscreen_probe(struct platform_device *pdev)
{
	int ret;
	int error = -1;
//	u8 data[2] = {0,};

	printk(KERN_DEBUG "[TSP] touchscreen_probe !! \n");
	set_touch_irq_gpio_disable();	// ryun 20091203


	if(IS_ATMEL)
		printk(KERN_DEBUG "[TSP] atmel touch driver!! \n");
	else
		printk("[TSP][ERROR] unknown touch driver!! \n");
	
	memset(&tsp, 0, sizeof(tsp));
	
	tsp.inputdevice = input_allocate_device();

	if (!tsp.inputdevice)
	{
		printk("[TSP][ERR] input_allocate_device fail \n");
		return -ENOMEM;
	}

  	init_timer(&tsp.opp_set_timer);
  	tsp.opp_set_timer.data = (unsigned long)&tsp; 
  	tsp.opp_set_timer.function = tsc_timer_out;

	INIT_WORK(&(tsp.constraint_wq), tsc_remove_constraint_handler);

	/* request irq */
	if (tsp.irq != -1)
	{
		tsp.irq = OMAP_GPIO_IRQ(OMAP_GPIO_TOUCH_INT);
		tsp.irq_type = IRQF_TRIGGER_LOW; 
		tsp.irq_enabled = 1;
	}

    // default and common settings
	tsp.inputdevice->name = "sec_touchscreen";
	tsp.inputdevice->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_SYN);	// ryun
	tsp.inputdevice->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);		// ryun 20091127 
  	tsp.inputdevice->id.bustype = BUS_I2C;
  	tsp.inputdevice->id.vendor  = 0;
  	tsp.inputdevice->id.product =0;
 	tsp.inputdevice->id.version =0;
	
    // model specific settings
    switch(g_model) 
    {
        case LATONA:
        {
            tsp.inputdevice->keybit[BIT_WORD(TOUCH_MENU)] |= BIT_MASK(TOUCH_MENU);
            tsp.inputdevice->keybit[BIT_WORD(TOUCH_BACK)] |= BIT_MASK(TOUCH_BACK);
            tsp.inputdevice->keycode = atmel_ts_tk_keycode;
        	input_set_abs_params(tsp.inputdevice, ABS_MT_POSITION_X, 0, MAX_TOUCH_X_RESOLUTION, 0, 0);
        	input_set_abs_params(tsp.inputdevice, ABS_MT_POSITION_Y, 0, MAX_TOUCH_Y_RESOLUTION, 0, 0);
        	input_set_abs_params(tsp.inputdevice, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
        	input_set_abs_params(tsp.inputdevice, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
        	input_set_abs_params(tsp.inputdevice, ABS_MT_TRACKING_ID, 0, MAX_TOUCH_NUM - 1, 0, 0);
        }
        break;
    }        

	ret = input_register_device(tsp.inputdevice);
	if (ret) {
	printk(KERN_ERR "atmel_ts_probe: Unable to register %s \
			input device\n", tsp.inputdevice->name);
	}

	tsp_wq = create_singlethread_workqueue("tsp_wq");
#ifdef __CONFIG_ATMEL__
	INIT_WORK(&tsp.tsp_work, atmel_touchscreen_read);
#endif

#ifdef __CONFIG_ATMEL__	
	atmel_touch_probe();		// ryun !!!???
#endif
	if(qt60224_notfound_flag)
		return -EINVAL;

#ifdef CONFIG_HAS_EARLYSUSPEND
	tsp.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tsp.early_suspend.suspend = atmel_ts_early_suspend;
	tsp.early_suspend.resume = atmel_ts_late_resume;
	register_early_suspend(&tsp.early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

// [[ This will create the touchscreen sysfs entry under the /sys directory
struct kobject *ts_kobj;
ts_kobj = kobject_create_and_add("touchscreen", NULL);
	if (!ts_kobj)
		return -ENOMEM;


	error = sysfs_create_file(ts_kobj,
				  &touch_boost_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj,
				  &firmware_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
		}

	error = sysfs_create_file(ts_kobj,
				  &firmware_binary_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
		}

	error = sysfs_create_file(ts_kobj,
				  &firmware_binary_read_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
		}

	error = sysfs_create_file(ts_kobj,
				  &firmware_ret_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
		}

	/*------------------------------ for tunning ATmel - start ----------------------------*/
	error = sysfs_create_file(ts_kobj, &dev_attr_set_power.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(ts_kobj, &dev_attr_set_acquisition.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(ts_kobj, &dev_attr_set_touchscreen.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(ts_kobj, &dev_attr_set_keyarray.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(ts_kobj, &dev_attr_set_grip.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(ts_kobj, &dev_attr_set_noise.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(ts_kobj, &dev_attr_set_total.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(ts_kobj, &dev_attr_set_write.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	/*------------------------------ for tunning ATmel - end ----------------------------*/

#ifdef ENABLE_NOISE_TEST_MODE
//	struct kobject *qt602240_noise_test;
//	qt602240_noise_test = kobject_create_and_add("qt602240_noise_test", NULL);
//	if (!qt602240_noise_test) {
//		printk("Failed to create sysfs(qt602240_noise_test)!\n");
//		return -ENOMEM;
//	}
	struct device *qt602240_noise_test = device_create(sec_class, NULL, 0, NULL, "qt602240_noise_test");
 
	if (device_create_file(qt602240_noise_test, &dev_attr_set_refer0.attr)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer0.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_delta0.attr) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta0.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_refer1.attr)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer1.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_delta1.attr) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta1.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_refer2.attr)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer2.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_delta2.attr) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta2.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_refer3.attr)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer3.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_delta3.attr) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta3.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_refer4.attr)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer4.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_delta4.attr) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta4.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_refer5.attr)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer5.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_delta5.attr) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta5.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_refer6.attr)< 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_refer6.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_delta6.attr) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_delta6.attr.name);
	if (device_create_file(qt602240_noise_test, &dev_attr_set_threshold.attr) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_set_threshold.attr.name);
#endif

	error = sysfs_create_file(ts_kobj,
				  &bootcomplete_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}

// ]] This will create the touchscreen sysfs entry under the /sys directory

#ifdef TOUCH_PROC
	touch_proc = create_proc_entry("touch_proc", S_IFREG | S_IRUGO | S_IWUGO, 0);
	if (touch_proc)
	{
		touch_proc->proc_fops = &touch_proc_fops;
		printk(" succeeded in initializing touch proc file\n");
	}
	else
	{
	        printk(" error occured in initializing touch proc file\n");
	}
#endif
	printk(KERN_DEBUG "[TSP] success probe() !\n");

	return 0;
}

#ifdef TOUCH_PROC
int touch_proc_ioctl(struct inode *p_node, struct file *p_file, unsigned int cmd, unsigned long arg)
{
        switch(cmd)
        {
                case TOUCH_GET_VERSION :
                {
                        char fv[10];

                        sprintf(fv,"0X%x", g_version);
                        if(copy_to_user((void*)arg, (const void*)fv, sizeof(fv)))
                          return -EFAULT;
                }
                break;

                case TOUCH_GET_T_KEY_STATE :
                {
                        int key_short = 0;

                        key_short = menu_button || back_button;
                        if(copy_to_user((void*)arg, (const void*)&key_short, sizeof(int)))
                          return -EFAULT;
                
                }
                break;
                case TOUCH_GET_SW_VERSION :
                {
                        if(copy_to_user((void*)arg, (const void*)fw_version, sizeof(fw_version)))
                          return -EFAULT;
                }
                break;                
        }
        return 0;
}
#endif
static int touchscreen_remove(struct platform_device *pdev)
{

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&tsp.early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	input_unregister_device(tsp.inputdevice);

	if (tsp.irq != -1)
	{
//		down_interruptible(&sem_touch_handler);
		if(g_enable_touchscreen_handler == 1)
		{
			free_irq(tsp.irq, &tsp);
			g_enable_touchscreen_handler = 0;
		}
//		up(&sem_touch_handler);
	}

	gpio_set_value(OMAP_GPIO_TOUCH_EN, 0);
	return 0;
}

extern int atmel_suspend(void);
extern int atmel_resume(void);

static int touchscreen_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk(KERN_DEBUG "[TSP] touchscreen_suspend : touch power off\n");
	atmel_suspend();
	if (menu_button == 1)
	{
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
		printk(KERN_DEBUG "[TSP] menu_button force released\n");                                
#endif
		input_report_key(tsp.inputdevice, 139, DEFAULT_PRESSURE_UP);     
        	input_sync(tsp.inputdevice);    				
	}
	if (back_button == 1)
	{
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
		printk(KERN_DEBUG "[TSP] back_button force released\n");
#endif
		input_report_key(tsp.inputdevice, 158, DEFAULT_PRESSURE_UP); 
        	input_sync(tsp.inputdevice);    				
	}

// Workaround for losing state when suspend mode(power off)
#ifdef CONFIG_TOUCHKEY_LOCK
	touchkey_lock_flag = 0;
#endif
	return 0;
}

static int touchscreen_resume(struct platform_device *pdev)
{
	printk(KERN_DEBUG "[TSP] touchscreen_resume : touch power on\n");

	atmel_resume();
//	initialize_multi_touch(); 
	enable_irq(tsp.irq);
	return 0;
}

static int touchscreen_shutdown(struct platform_device *pdev)
{
	qt60224_notfound_flag = 1; // to prevent misorder

	disable_irq(tsp.irq);
	flush_workqueue(tsp_wq);

	if (tsp.irq != -1)
	{
		if(g_enable_touchscreen_handler == 1)
		{
			free_irq(tsp.irq, &tsp);
			g_enable_touchscreen_handler = 0;
		}
	}
	gpio_set_value(OMAP_GPIO_TOUCH_EN, 0);

	printk("[TSP] %s   !!!\n", __func__);

	return 0;
}

static void touchscreen_device_release(struct device *dev)
{
	/* Nothing */
}

static struct platform_driver touchscreen_driver = {
	.probe 		= touchscreen_probe,
	.remove 	= touchscreen_remove,
	.shutdown = touchscreen_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND		// 20100113 ryun 
	.suspend 	= &touchscreen_suspend,
	.resume 	= &touchscreen_resume,
#endif	
	.driver = {
		.name	= TOUCHSCREEN_NAME,
	},
};

static struct platform_device touchscreen_device = {
	.name 		= TOUCHSCREEN_NAME,
	.id 		= -1,
	.dev = {
		.release 	= touchscreen_device_release,
	},
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void atmel_ts_early_suspend(struct early_suspend *h)
{
//	melfas_ts_suspend(PMSG_SUSPEND);
	touchscreen_suspend(&touchscreen_device, PMSG_SUSPEND);
}

void atmel_ts_late_resume(struct early_suspend *h)
{
//	melfas_ts_resume();
	touchscreen_resume(&touchscreen_device);
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */

static int __init touchscreen_init(void)
{
	int ret;

#if defined(CONFIG_MACH_SAMSUNG_LATONA) || defined(CONFIG_MACH_SAMSUNG_P1WIFI)
    g_model = LATONA;
#else
    g_model = DEFAULT_MODEL;
#endif

        gpio_set_value(OMAP_GPIO_TOUCH_EN, 1);  // TOUCH EN
        msleep(80);

	wake_lock_init(&tsp_firmware_wake_lock, WAKE_LOCK_SUSPEND, "TSP");

	ret = platform_device_register(&touchscreen_device);
	if (ret != 0)
		return -ENODEV;

	ret = platform_driver_register(&touchscreen_driver);
	if (ret != 0) {
		platform_device_unregister(&touchscreen_device);
		return -ENODEV;
	}

	return 0;
}

static void __exit touchscreen_exit(void)
{
	wake_lock_destroy(&tsp_firmware_wake_lock);
	platform_driver_unregister(&touchscreen_driver);
	platform_device_unregister(&touchscreen_device);
}

int touchscreen_get_tsp_int_num(void)
{
        return tsp.irq;
}
module_init(touchscreen_init);
module_exit(touchscreen_exit);

MODULE_LICENSE("GPL");
