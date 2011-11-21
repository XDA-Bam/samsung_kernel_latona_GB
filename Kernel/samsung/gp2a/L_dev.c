/*
 * L_dev.c 
 *
 * Description: This file implements the Light sensor module
 *
 * Author: Varun Mahajan <m.varun@samsung.com>
 */

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/twl4030-madc.h>

#include <linux/delay.h>
#include <linux/earlysuspend.h>

#include "main.h"
#include "L_dev.h"
#include "common.h"

/*Light sensor device state, LDO state*/
#define  NOT_OPERATIONAL    0
#define  OPERATIONAL        1

#define TWL4030_MADC_CHANNEL_LIGHT        4

#define DEFAULT_POLLING_INTERVAL    500
#define TESTMODE_POLLING_INTERVAL   500

#define L_SYSFS_POLLING_ON            0
#define L_SYSFS_POLLING_OFF            1

#define L_SYSFS_START_POLLING        2
#define L_SYSFS_STOP_POLLING        3

#define L_SYSFS_ERROR              -1

// To turn on USB block in PMIC
#define VSEL_VINTANA2_2V75  0x01
#define CARKIT_ANA_CTRL     0xBB
#define SEL_MADC_MCPC       0x08

/*
 * lock: mutex to serialize the access to this data structure 
 * t2_vintana2_ldo: res_handle structure for VINTANA 2 LDO in TWL4030
 * device_state: operational or not operational
 * ldo_state: operational or not operational
 * ch_request: madc_chrequest structure to get the converted value for a 
 * particular channel from MADC in TWL4030
 * table: pointer to light sensor's illuminance level table
 */
typedef struct 
{
    struct mutex lock;
    struct res_handle *t2_vintana2_ldo;
    u16 device_state;
    u16 ldo_state;
    struct twl4030_madc_request ch_request;
    u32 *table;
    int last_adc_val;
    int last_lux_val;
    int last_brightness_step;
    struct input_dev *inputdevice;
    struct timer_list *light_polling_timer;
    unsigned long polling_time;
    unsigned long actual_polling_time;
    int cur_polling_state;
    int saved_polling_state;
    u16 lcd_brightness;
    int testmode;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
    u16 op_state;
} L_device_t;

/**************************************************************/
/*extern functions*/
/**************************************************************/
extern int pl_sensor_power_on(void);
extern int pl_sensor_power_off(void);
extern int pl_usb_power_on(void);
extern int pl_usb_power_off(void);
extern int pl_madc_power_on(void);
extern int pl_madc_power_off(void);

void L_dev_sync_mech_init(void); 
int L_dev_init(void);
int L_dev_exit(void);
int L_dev_suspend(void);
int L_dev_resume(void);
int L_dev_get_adc_val(u32 *);
int L_dev_get_illum_lvl(u16 *);
int get_average_adc_value(unsigned int * data, int count);
int L_dev_polling_start( void );
int L_dev_polling_stop( void );
int L_dev_set_timer(u16);

/**************************************************************/
/*static functions*/
/**************************************************************/
static int get_illum_lvl(u32, u16 *);


int L_sysfs_init(struct input_dev *);
void L_sysfs_exit(struct input_dev *);


static struct workqueue_struct *light_wq;
static int force_report = 1;

/*Light sensor device structure*/
static L_device_t L_dev =
{
    .t2_vintana2_ldo = NULL,
    .device_state = NOT_OPERATIONAL,
    .ldo_state = NOT_OPERATIONAL,
    .ch_request = 
    {
        .channels = TWL4030_MADC_ADCIN4,
        .do_avg = 0,
        .method = TWL4030_MADC_SW1,
        .active=0,
        .func_cb=NULL,
        .rbuf[TWL4030_MADC_CHANNEL_LIGHT] = 0,
    },
    .table = L_table,
    .last_adc_val = 0,
    .last_lux_val = 0,
    .last_brightness_step = -1,
    .inputdevice = NULL,
    .polling_time = DEFAULT_POLLING_INTERVAL,
    .cur_polling_state = L_SYSFS_POLLING_OFF,
    .saved_polling_state = L_SYSFS_POLLING_OFF,
    .lcd_brightness = 5,
    .testmode = 0,
    .op_state = 0,
};    

static int adc_vs_lux_table[][3] = {
    { 2, 8 },
    { 5, 10 },
    { 8, 45 },
    { 10, 69 },
    { 12, 79 },
    { 22, 102 },
    { 30, 159 },
    { 50, 196 },
    { 75, 200 },
    { 110, 245 },
    { 150, 295 },
    { 170, 307 },
    { 300, 326 },
    { 460, 378 },
    { 1000, 463 },
    { 1500, 500 },
    { 1700, 515 },
    { 2000, 526 },
    { 2850, 561 },
    { 3450, 570 },
    { 6000, 603 },
    { 11000, 648 },    /* lux, adc value when usb connected, adc value */
    { 0, 0, 0}
};

static int brightness_step_table[] = { 0, 15, 150, 1500, 10000000 };

/**************************************************************/
/*for synchronization mechanism*/
/**************************************************************/
#define LOCK()    mutex_lock(&(L_dev.lock))
#define UNLOCK()  mutex_unlock(&(L_dev.lock))

/*work struct*/
static void L_dev_work_func (struct work_struct *);
static DECLARE_DELAYED_WORK(L_ws, L_dev_work_func);

#ifdef CONFIG_HAS_EARLYSUSPEND
static int L_dev_early_suspend(struct early_suspend* handler);
static int L_dev_early_resume(struct early_suspend* handler);
#endif

#ifdef CONFIG_FSA9480_NOTIFY_USB_CONNECTION_STATE
static int usb_connection_state;
typedef void (*notification_handler)(const int);
extern int fsa9480_add_connection_state_monitor(notification_handler);
extern void fsa9480_remove_connection_state_monitor(notification_handler);

static void L_dev_usb_connection_state_change_handler(const int onoff)
{
    printk("[PLSENSOR][%s] PARAM = [%d]\n", __func__, onoff);
    usb_connection_state = onoff;
#if 0
    if(L_dev.saved_polling_state == L_SYSFS_POLLING_ON)
    {
        cancel_delayed_work_sync(&L_ws);
        if(onoff)
            queue_delayed_work(light_wq, &L_ws, msecs_to_jiffies(2000));
        else
            queue_delayed_work(light_wq, &L_ws, msecs_to_jiffies(7000));
    }
#endif
}
#endif

void L_dev_sync_mech_init(void)
{
    mutex_init(&(L_dev.lock));
}

static int turn_resources_on_for_adc()
{
    int ret;
    u8 val = 0; 
    
    ret = twl_i2c_read_u8( TWL4030_MODULE_MADC, &val, TWL4030_MADC_CTRL1 );
    val &= ~TWL4030_MADC_MADCON;
    ret = twl_i2c_write_u8( TWL4030_MODULE_MADC, val, TWL4030_MADC_CTRL1 );
    msleep( 10 );

    ret = twl_i2c_read_u8( TWL4030_MODULE_MADC, &val, TWL4030_MADC_CTRL1 );
    val |= TWL4030_MADC_MADCON;
    ret = twl_i2c_write_u8( TWL4030_MODULE_MADC, val, TWL4030_MADC_CTRL1 );
    
    pl_usb_power_on();
    twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, 0x14, 0x7D );
    twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, 0x0, 0x7E );
    twl_i2c_read_u8( TWL4030_MODULE_USB, &val, 0xFE );
    val |= 0x1;
    twl_i2c_write_u8( TWL4030_MODULE_USB, val, 0xFE );

    twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, VSEL_VINTANA2_2V75, TWL4030_VINTANA2_DEDICATED );    
    twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, 0x20, TWL4030_VINTANA2_DEV_GRP );
    twl_i2c_write_u8( TWL4030_MODULE_USB, SEL_MADC_MCPC, CARKIT_ANA_CTRL );
    
    return pl_usb_power_off();
}


/**************************************************************/
/* extern functions */
/**************************************************************/
int L_dev_init(void)
{
    int ret = 0;

    trace_in();

    LOCK();   

    debug("[light] power up VINTANA2....\n");
    if (ret != twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,0x01, 0x46))
        return -EIO;
    if (ret != twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,0x20, 0x43))
        return -EIO;
        
    L_dev.device_state = OPERATIONAL;
    L_dev.ldo_state = OPERATIONAL;

    UNLOCK();     

    if( !(L_dev.inputdevice = input_allocate_device()) )
    {
        ret = -ENOMEM;
        input_free_device(L_dev.inputdevice);
        failed(1);
    }

    set_bit(EV_ABS, L_dev.inputdevice->evbit);
    input_set_capability(L_dev.inputdevice, EV_ABS, ABS_MISC);
    input_set_capability(L_dev.inputdevice, EV_ABS, ABS_BRAKE);     /* status */
    input_set_capability(L_dev.inputdevice, EV_ABS, ABS_X);         /* wake */
    input_set_capability(L_dev.inputdevice, EV_ABS, ABS_THROTTLE);     /* enabled/delay */
    L_dev.inputdevice->name = "light_sensor"; 

    if((ret = input_register_device(L_dev.inputdevice))<0) 
    {
        failed(2);
    }
    else
    {
        L_sysfs_init(L_dev.inputdevice);
    }
    
    INIT_DELAYED_WORK(&L_ws, (void (*) (struct work_struct *))L_dev_work_func);
    light_wq = create_singlethread_workqueue("light_wq");
    if (!light_wq)
        return -ENOMEM;

#ifdef CONFIG_HAS_EARLYSUSPEND
    L_dev.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1 ;
    L_dev.early_suspend.suspend = (void *)L_dev_early_suspend;
    L_dev.early_suspend.resume = (void *)L_dev_early_resume;
    register_early_suspend(&L_dev.early_suspend);
#endif

#ifdef CONFIG_FSA9480_NOTIFY_USB_CONNECTION_STATE
    if(fsa9480_add_connection_state_monitor(L_dev_usb_connection_state_change_handler))
    {
        printk(KERN_ERR "[PLSENSOR][%s] : fsa9480_add_connection_state_monitor failed!\n", __func__);
    }
#endif

    L_dev.actual_polling_time = DEFAULT_POLLING_INTERVAL;

//-------------for test----------------
#if 0

        if( L_dev_polling_start() != 0 )
        {
            printk(KERN_ERR "L_dev_polling_start() : fail!! \n");
            ret = 0;
        }
        turn_resources_on_for_adc();

        pl_sensor_power_on();
#endif	
//-------------for test----------------

    trace_out();

    return ret;
}

int L_dev_exit(void)
{
    int ret = 0;

    trace_in();

    LOCK();

#ifdef CONFIG_FSA9480_NOTIFY_USB_CONNECTION_STATE
    fsa9480_remove_connection_state_monitor(L_dev_usb_connection_state_change_handler);
#endif

    L_dev.device_state = NOT_OPERATIONAL;
    L_dev.ldo_state = NOT_OPERATIONAL;

    L_sysfs_exit(L_dev.inputdevice);
    input_unregister_device(L_dev.inputdevice);    

    if (light_wq)
        destroy_workqueue(light_wq);

    UNLOCK();

    trace_out();

    return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int L_dev_early_suspend(struct early_suspend* handler)
{
    int ret = 0;
    trace_in() ;

    if(L_dev.saved_polling_state == L_SYSFS_POLLING_ON)
    {
        debug("[light] L_dev_suspend(void)\n");
        flush_work(&L_ws);
        cancel_delayed_work_sync(&L_ws);
        L_dev_polling_stop();
        L_dev.saved_polling_state = L_SYSFS_POLLING_ON;
        L_dev.cur_polling_state = L_SYSFS_POLLING_OFF;
    }

    printk( "%s: Early suspend sleep success!!!\n", __FUNCTION__ ) ;
    trace_out() ; 
    return ret;
}

static int L_dev_early_resume(struct early_suspend* handler)
{
    int ret = 0;
    trace_in();

    if(L_dev.cur_polling_state == L_SYSFS_POLLING_OFF && 
       L_dev.saved_polling_state == L_SYSFS_POLLING_ON)
    {
        debug("[light] L_dev_resume() : L_dev_polling_start() L_dev.polling_time = %d ms \n", L_dev.polling_time);
        L_dev.saved_polling_state = L_SYSFS_POLLING_OFF;
        L_dev_polling_start();
        L_dev.cur_polling_state = L_SYSFS_POLLING_ON;
    }

    printk( "%s: Early suspend wakeup success!!!\n", __FUNCTION__ ) ;
    trace_out();
    return ret;
}
#endif

int L_dev_suspend(void)
{
    int ret = 0;

    trace_in();

    LOCK();   

    if( L_dev.device_state == NOT_OPERATIONAL )
    {
        failed(1);
        ret = -1;
    }
    else if( L_dev.ldo_state == OPERATIONAL )
    {
#ifndef CONFIG_HAS_EARLYSUSPEND
        if(L_dev.saved_polling_state == L_SYSFS_POLLING_ON)
        {
            debug("[light] L_dev_suspend(void)\n");
            flush_work(&L_ws);
            cancel_delayed_work_sync(&L_ws);
            L_dev_polling_stop();    
            L_dev.saved_polling_state = L_SYSFS_POLLING_ON;
            L_dev.cur_polling_state = L_SYSFS_POLLING_OFF;
        }
#endif
        L_dev.ldo_state = NOT_OPERATIONAL;
    }

    UNLOCK(); 

    trace_out();

    return ret;
}

int L_dev_resume(void)
{
    int ret = 0;

    trace_in();
    LOCK();   

    if( L_dev.device_state == NOT_OPERATIONAL )
    {
        failed(1);
        ret = -1;
    }
    else if( L_dev.ldo_state == NOT_OPERATIONAL )
    {
        // add power up code.
        debug("[light] power up VINTIANA2....\n");
        if (ret != twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,0x41, 0x46))
            return -EIO;
        if (ret != twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,0x20, 0x43))
        
            return -EIO;
        L_dev.ldo_state = OPERATIONAL;

#ifndef CONFIG_HAS_EARLYSUSPEND
        debug("[light] resume!! L_dev.cur_polling_state=%d, L_dev.saved_polling_state=%d \n", L_dev.cur_polling_state, L_dev.saved_polling_state);
        if(L_dev.cur_polling_state == L_SYSFS_POLLING_OFF && 
           L_dev.saved_polling_state == L_SYSFS_POLLING_ON)
        {
            debug("[light] L_dev_resume() : L_dev_polling_start() L_dev.polling_time = %d ms \n", L_dev.polling_time);
            L_dev.saved_polling_state = L_SYSFS_POLLING_OFF;
            L_dev_polling_start();
            L_dev.cur_polling_state = L_SYSFS_POLLING_ON;
        }
#endif
    }

    UNLOCK(); 
    trace_out();

    return ret;
}

int L_dev_get_adc_val(u32 *adc_val)
{
    int ret = 0;

    LOCK();   

    if( L_dev.device_state == NOT_OPERATIONAL )
    {
        failed(1);
        ret = -1;
    }
    else if ( L_dev.ldo_state == NOT_OPERATIONAL )
    {
        failed(2);
        ret = -1;
    }
    else if((ret = twl4030_madc_conversion(&(L_dev.ch_request))) < 0 ) // ryun
    {
        failed(3);
    }    
    else
    {
        *adc_val = L_dev.ch_request.rbuf[TWL4030_MADC_CHANNEL_LIGHT];
        debug("ADC data: %u", L_dev.ch_request.rbuf[TWL4030_MADC_CHANNEL_LIGHT]);
    }

    UNLOCK(); 

    return ret;
}

int L_dev_get_illum_lvl(u16 *illum_lvl)
{
    int ret = 0;
    u32 mV;

    trace_in();

    LOCK();   

    if( L_dev.device_state == NOT_OPERATIONAL )
    {
        failed(1);
        ret = -1;
    }
    else if ( L_dev.ldo_state == NOT_OPERATIONAL )
    {
        failed(2);
        ret = -1;
    }
    else if((ret = twl4030_madc_conversion(&(L_dev.ch_request))) < 0 )
    {
        failed(3);
    } 
    else
    {
        mV = L_dev.ch_request.rbuf[TWL4030_MADC_CHANNEL_LIGHT];
        debug("   ADC data: %u", L_dev.ch_request.rbuf[TWL4030_MADC_CHANNEL_LIGHT]);
        if( get_illum_lvl(mV, illum_lvl) < 0 )
        {
            failed(4);
            ret = -1;
        }
    }

    UNLOCK(); 
    trace_out();
    return ret;
}    

static int get_illum_lvl(u32 mV, u16 *illum_lvl)
{
    int ret = -1;
    int i = L_LVL1_mV_IDX, j;

    if( L_dev.table[L_NO_OF_LVLS_IDX] > L_MAX_LVLS )
    {
        failed(1);
        return -1;
    }

    for( i = L_LVL1_mV_IDX, j = 1; j <= L_dev.table[L_NO_OF_LVLS_IDX]; i += L_LVL_mV_INCR, j++ )
    {
        if( (mV >= L_dev.table[i]) && (mV <= L_dev.table[i+1]) )
        {
            *illum_lvl = L_dev.table[i-3];
            ret = 0;
            break;
        }
    }
    return ret;
}

int get_average_adc_value(unsigned int * data, int count)
{
    int i=0, average, min=0xFFFFFFFF, max=0, total=0;
    for(i=0 ; i<count ; i++)
    {
        if(data[i] < min)
            min=data[i];
        if(data[i] > max)
            max=data[i];

        total+=data[i];
    }
    average = (total - min -max)/(count -2);
    return average;
}

int L_dev_get_op_state(void)
{
    return (int)(L_dev.op_state);
}

void L_dev_set_op_state(u16 op_state)
{
    switch(op_state)
    {
        case 0: // OFF
            if(L_dev.op_state == 1) // if ON state
            {
                L_dev.op_state = 0;
            }
            break;
        case 1: // ON
            L_dev.op_state = 1;
            break;
        case 2: // ON LOCK for factory test
            L_dev.op_state = 2;
            break;
    }
}

int L_dev_polling_start(void)
{    
    printk("[light] L_dev_polling_start() L_dev.polling_time = %d ms\n", L_dev.polling_time);
    if(L_dev.saved_polling_state == L_SYSFS_POLLING_ON)
    {
        debug("[light] L_dev_polling_start() ... already POLLING ON!! \n");
        return L_dev.saved_polling_state;
    }

    cancel_delayed_work_sync(&L_ws);
    queue_delayed_work(light_wq, &L_ws, msecs_to_jiffies(L_dev.actual_polling_time));

    L_dev.saved_polling_state = L_SYSFS_POLLING_ON;

    return L_dev.saved_polling_state;
}

int L_dev_polling_stop(void)
{
    printk("[light] L_dev_polling_stop() \n");

    if(L_dev.saved_polling_state == L_SYSFS_POLLING_OFF)
    {
        debug("[light] L_dev_polling_stop() ... already POLLING OFF!! \n");
        return L_dev.saved_polling_state;
    }

    flush_work(&L_ws);
    cancel_delayed_work_sync(&L_ws);

    L_dev.saved_polling_state = L_SYSFS_POLLING_OFF;
    return L_dev.saved_polling_state;
}

int L_dev_get_polling_state(void)
{
    if( L_dev.device_state == NOT_OPERATIONAL )
    {
        return L_SYSFS_POLLING_OFF;
    }
    else if ( L_dev.ldo_state == NOT_OPERATIONAL )
    {
        return L_SYSFS_POLLING_OFF;
    }
    
    return L_dev.saved_polling_state;
}

unsigned long L_dev_get_polling_interval( void )
{
    return L_dev.polling_time;
}

void L_dev_set_polling_interval(unsigned long interval)
{
    L_dev.polling_time = interval;
}

static void L_dev_work_func (struct work_struct *unused)
{
    u32 adc_val;

    trace_in() ;

    debug("[light] L_dev_work_func(), L_dev.saved_polling_state= %d\n", L_dev.saved_polling_state);
    if( !(L_dev.saved_polling_state) )
    {
        int adc_level = 0;
#ifdef CONFIG_FSA9480_NOTIFY_USB_CONNECTION_STATE        
        int adc_index = usb_connection_state ? 1 : 2;   /* To choose adc value from the table according the state of USB connection */
#else
        int adc_index = 2;
#endif

        if( L_dev_get_adc_val(&adc_val) < 0 )
        {
            failed(1);
            debug( "[light] Failed!!!\n" );
        }
        
        L_dev.last_adc_val = adc_val;
	//for test		
	//printk(KERN_DEBUG "LSENSOR: %s: adc_val=%d\n", __func__,adc_val);
        
        {
            int lux = 11000;
            int i = 0;
            
            for(; adc_vs_lux_table[i+1][0] > 0; i++)
            {
                    // in case the adc value is smaller than 30 lux
                    if(adc_val < adc_vs_lux_table[i][1])
                    {
                            lux = adc_vs_lux_table[i][0] * adc_val / adc_vs_lux_table[i][1];
                            break;
                    }

                    if(adc_val >= adc_vs_lux_table[i][1] && adc_val < adc_vs_lux_table[i+1][1])
                    {
                            lux = (adc_vs_lux_table[i+1][0] - adc_vs_lux_table[i][0]) /
                                  (adc_vs_lux_table[i+1][1] - adc_vs_lux_table[i][1]) * 
                                  (adc_val - adc_vs_lux_table[i][1]) +
                                  adc_vs_lux_table[i][0];
                            break;
                    }
            }

	     L_dev.last_lux_val = lux;
		 
            if(L_dev.testmode == 1)
            {
                input_report_abs(L_dev.inputdevice, ABS_MISC, lux);
                input_sync(L_dev.inputdevice);
		  //L_dev.last_lux_val = lux;
            }
            else if(L_dev.testmode == 2)
            {
                input_report_abs(L_dev.inputdevice, ABS_MISC, lux+200);
                input_sync(L_dev.inputdevice);
		 //L_dev.last_lux_val = lux;		
            }
            else
            {
                int i = 0;
                int step = 0;
                int final = sizeof(brightness_step_table)/sizeof(int);
                
                for(; i < final - 1; i++)
                {
                    if((lux >= brightness_step_table[i]) && (lux < brightness_step_table[i+1]))
                    {
                        step = i;
                        break;
                    }
                }
                
                if(L_dev.last_brightness_step != step)
                {
                    input_report_abs(L_dev.inputdevice, ABS_MISC, lux);
                    input_sync(L_dev.inputdevice);
                    //L_dev.last_lux_val = lux;
                    L_dev.last_brightness_step = step;
		     printk(KERN_DEBUG "LSENSOR: %s: adc_val=%d lux = %d \n", __func__, adc_val, lux);			
                }
            }
        }


// If you want see adc value in test mode, please uncomment the below two lines
//       input_report_abs(L_dev.inputdevice, ABS_X, adc_val);
//       input_sync(L_dev.inputdevice);
    }

    queue_delayed_work(light_wq, &L_ws, msecs_to_jiffies(L_dev.actual_polling_time));

    trace_out() ;
}


static ssize_t L_delay_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t L_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t L_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t L_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t L_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t L_data_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t L_status_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t L_adc_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t L_lux_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t L_testmode_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t L_testmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP, L_delay_show, L_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, L_enable_show, L_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, L_wake_store);
static DEVICE_ATTR(data, S_IRUGO, L_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, L_status_show, NULL);
static DEVICE_ATTR(adc, S_IRUGO, L_adc_show, NULL);
static DEVICE_ATTR(lux, S_IRUGO, L_lux_show, NULL);
static DEVICE_ATTR(testmode, 0664, L_testmode_show, L_testmode_store);

static struct attribute *light_attributes[] = {
    &dev_attr_adc.attr,
    &dev_attr_lux.attr,
    NULL
};

static struct attribute *L_attributes[] = {
    &dev_attr_poll_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,
    &dev_attr_status.attr,
    &dev_attr_adc.attr,
    &dev_attr_lux.attr,
    &dev_attr_testmode.attr,
    NULL
};

static struct attribute_group L_attribute_group = {
    .attrs = L_attributes
};

extern struct class *sensors_class;
extern int sensors_register(struct device *dev, void * drvdata, struct device_attribute *attributes[], char *name);
static struct device *light_sensor_device;

int L_sysfs_init(struct input_dev * input_data)
{
    int ret = 0;

    trace_in();

    ret = sysfs_create_group(&input_data->dev.kobj, &L_attribute_group);
    if (ret) {
        printk(KERN_ERR "L_sysfs_init: sysfs_create_group failed[%s]\n", input_data->name);
    }
    ret = sensors_register(light_sensor_device, NULL, light_attributes, "light_sensor");
    if(ret) {
    	printk(KERN_ERR "%s: cound not register accelerometer sensor device(%d).\n", __func__, ret);
    }

    trace_out();

    return ret;
}    

void L_sysfs_exit(struct input_dev * input_data)
{
    trace_in();

    sysfs_remove_group(&input_data->dev.kobj, &L_attribute_group);

    trace_out();
}    

static ssize_t L_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", L_dev.last_adc_val);
}

static ssize_t L_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", L_dev.last_lux_val);
}

static ssize_t L_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
    int x;

    spin_lock_irqsave(&input_data->event_lock, flags);

    x = input_data->abs[ABS_MISC];

    spin_unlock_irqrestore(&input_data->event_lock, flags);

    return sprintf(buf, "%d\n", x);
}

static ssize_t L_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t ret;
    int enabled = 0;
    int power_state;
   
    trace_in();
    
    power_state = L_dev_get_polling_state();
    
    if( power_state == L_SYSFS_POLLING_ON)
    {
        enabled = 1;
    }
    else
    {
        enabled = 0;
    }        
        
    debug("   enabled: %d", enabled);

    ret = sprintf(buf, "%d\n",enabled);

    trace_out();
    return ret;
}

static ssize_t L_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    ssize_t ret = strlen(buf);
    int enabled = 0;

    trace_in();
    
    if (strncmp(buf, "0", 1) == 0 ) 
    {
        if( L_dev_polling_stop() != 1 )
        {
            printk(KERN_ERR "L_dev_polling_stop() : fail!! \n");
            ret = 0;
        }
        pl_madc_power_off();
        pl_sensor_power_off();
    }
    else if(strncmp(buf, "1", 1) == 0 ) 
    {
        enabled = 1;
        if( L_dev_polling_start() != 0 )
        {
            printk(KERN_ERR "L_dev_polling_start() : fail!! \n");
            ret = 0;
        }
        turn_resources_on_for_adc();

        pl_sensor_power_on();
        force_report = 1;
    }
        
    input_report_abs(L_dev.inputdevice, ABS_THROTTLE, (enabled<<16) | L_dev_get_polling_interval());
    
    trace_out();
    return ret;
}

static ssize_t L_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    size_t ret;
    unsigned long interval = L_dev_get_polling_interval();

    trace_in();

    debug("   sensor L_interval_show() : %lu", interval );
    ret = sprintf(buf, "%lu\n", interval);

    trace_out();
    return ret;
}

static ssize_t L_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    ssize_t ret = strlen(buf);
    unsigned long delay;
    int enabled = 0;
    
    trace_in();

    sscanf(buf, "%lu", &delay);
    L_dev_set_polling_interval(delay);

    if( L_dev_get_polling_state() == L_SYSFS_POLLING_ON)
    {
        enabled = 1;
    }
    else
    {
        enabled = 0;
    }        
    
    debug("   sensor L_interval_store() : %lu sec", delay );

    input_report_abs(L_dev.inputdevice, ABS_THROTTLE, (enabled<<16) | delay);

    trace_out();

    return ret;

}

static ssize_t L_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    static int cnt = 1;

    input_report_abs(L_dev.inputdevice, ABS_X, cnt++);

    return count;
}

static ssize_t L_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned long flags;
    int status;

    spin_lock_irqsave(&L_dev.inputdevice->event_lock, flags);

    status = L_dev.inputdevice->abs[ABS_BRAKE];

    spin_unlock_irqrestore(&L_dev.inputdevice->event_lock, flags);

    return sprintf(buf, "%d\n", status);
}

static ssize_t L_testmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int value = simple_strtoul(buf, NULL,10);
    L_dev.testmode = value;
    if(value)
    {
        L_dev.actual_polling_time = TESTMODE_POLLING_INTERVAL;
    }
    else
    {
        L_dev.actual_polling_time = DEFAULT_POLLING_INTERVAL;
    }
    
    return count;
}

static ssize_t L_testmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", L_dev.testmode);
}

