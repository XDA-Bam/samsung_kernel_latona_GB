/*
 * main.c 
 *
 * Description: This file implements the Main module of the GP2AP002A00F driver 
 *
 * Author: Varun Mahajan <m.varun@samsung.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c/twl.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <plat/io.h>
#include <plat/mux.h>

#define I2C_M_WR 0
#define PHY_TO_OFF_PM_RECIEVER(p)	(p - 0x5b)

#include <linux/workqueue.h>
#include <linux/stat.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/wakelock.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/irq.h>

#include "ioctls.h"
#include "P_dev.h"
#include "L_dev.h"
#include "common.h"
#include "main.h"

#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
#include "i2c_drv.c"
#else
#include <plat/i2c-omap-gpio.h>
OMAP_GPIO_I2C_CLIENT * this_client;
#endif


/*****************************************************************************/
/****************           PROXIMITY SENSOR SPECIFIC          ***************/
/*****************************************************************************/
/*flag states*/
#define SET   1
#define RESET 0

/*Wait queue structure*/
typedef struct
{
	struct list_head list;
	struct mutex list_lock;
	wait_queue_head_t waitq;
} P_queue_t;

/*for user count*/
typedef struct
{
	struct mutex lock;
	int count;
} user_count_t;

/*file structure private data*/
typedef struct
{
	struct list_head node;
	atomic_t int_flag;
	wait_queue_t waitq_entry;
} P_fpriv_data_t;

/*P IRQ*/
#define P_IRQ OMAP_GPIO_IRQ(OMAP_GPIO_PS_VOUT)

/*extern functions*/
int P_waitq_wkup_proc(void);

void P_enable_int(void);
void P_disable_int(void);

int pl_power_init( void );
void pl_power_exit( void );

/**********************************************************/
/*static functions*/
/**********************************************************/
/*ISR*/
static irqreturn_t P_isr( int irq, void *unused );

/*waitq related functions*/
static void P_waitq_init(void);
static void P_waitq_list_insert_proc(P_fpriv_data_t *);
static void P_waitq_prepare_to_wait(P_fpriv_data_t *);
static void P_waitq_list_remove_proc(P_fpriv_data_t *);
static void P_waitq_finish_wait(P_fpriv_data_t *);

/*module param*/
static u16 an_sleep_func = P_AN_SLEEP_OFF;
static u16 detec_cyc = P_CYC_8ms;

/*work struct*/
static struct work_struct P_ws;

static user_count_t P_user;

/*Wait queue*/
static P_queue_t P_waitq;

static struct wake_lock P_sensor_wake_lock;


/*****************************************************************************/
/******************           LIGHT SENSOR SPECIFIC          *****************/
/*****************************************************************************/


static u32 g_illum_lvl;

/*module param: light sensor's illuminance level table*/
u32 L_table [L_MAX_LVLS*5 + 1] = 
{
	/*total no of levels*/
	9,

	/*level no*/  /*lux (min)*/  /*lux(max)*/  /*mV(min)*/  /*mV(max)*/
	1,               1,           164,            0,          100,
	2,             165,           287,          101,         200,
	3,             288,           496,         201,         300,
	4,             497,           868,         301,         400,
	5,             869,          1531,         401,         500,
	6,            1532,          2691,         501,         600,
	7,            2692,          4691,         601,         700,
	8,            4692,          8279,         701,         800,
	9,            8280,        100000,         801,         2500,    
};


static irqreturn_t P_isr( int irq, void *unused )
{
	int ret = 0;

	wake_lock_timeout(&P_sensor_wake_lock, 3*HZ);

	debug("[ryun] Proximity interrupt!! \n");
	trace_in(); 

	ret = schedule_work(&P_ws);
	debug("[ryun] schedule_work(&P_ws) ret(1:succes, 0:fail) = %d \n", ret );

	trace_out();
	return IRQ_HANDLED;
}

/**********************************************************/
static void P_waitq_init(void)
{
	trace_in();

	mutex_init( &(P_waitq.list_lock) );
	init_waitqueue_head( &(P_waitq.waitq) );
	INIT_LIST_HEAD( &(P_waitq.list) );

	trace_out();
}

static void P_waitq_list_insert_proc( P_fpriv_data_t *fpriv )
{
	trace_in();
	mutex_lock( &(P_waitq.list_lock) );
	list_add_tail(&(fpriv->node), &(P_waitq.list));
	mutex_unlock( &(P_waitq.list_lock) );
	trace_out();
}

static void P_waitq_prepare_to_wait( P_fpriv_data_t *fpriv )
{
	trace_in();
	prepare_to_wait(&(P_waitq.waitq), &(fpriv->waitq_entry), TASK_INTERRUPTIBLE);
	trace_out();
}

static void P_waitq_list_remove_proc( P_fpriv_data_t *fpriv )
{
	trace_in();
	mutex_lock( &(P_waitq.list_lock) );
	list_del_init( &(fpriv->node) );
	mutex_unlock( &(P_waitq.list_lock) );
	trace_out();
}

static void P_waitq_finish_wait( P_fpriv_data_t *fpriv )
{
	trace_in();
	finish_wait(&(P_waitq.waitq), &(fpriv->waitq_entry));
	trace_out();
}

int P_waitq_wkup_proc(void)
{
	int no_of_processes = 0;
	struct list_head *node;
	P_fpriv_data_t *fpriv;

	trace_in();

	mutex_lock( &(P_waitq.list_lock) );

	list_for_each( node, &(P_waitq.list) )
	{
		fpriv = list_entry(node, P_fpriv_data_t, node);
		atomic_set( &(fpriv->int_flag), SET );
		no_of_processes++;
	}

	mutex_unlock( &(P_waitq.list_lock) );

	if( no_of_processes == 0 )
	{
		debug("   no process is waiting");
	}
	else
	{
		wake_up_interruptible( &(P_waitq.waitq) );
		debug("   no of processes woken up: %d", no_of_processes);
	}

	trace_out();

	return no_of_processes;
}

void P_enable_int(void)
{
	u32 reg_val = 0;

	debug("   INTERRUPT ENABLED");
	enable_irq_wake(P_IRQ);

	// enable irq
	reg_val = omap_readl(0x4905001C);
	reg_val |= 0x00200000; 
	omap_writel(reg_val, 0x4905001C);
//	reg_val = omap_readl(0x4905001C);
//	printk("P_enable_int 1C : %x \n", reg_val);
	
	// enable wakeup
	reg_val = omap_readl(0x49050020);
	reg_val |= 0x00200000;
	omap_writel(reg_val, 0x49050020);
//	reg_val = omap_readl(0x49050020);
//	printk("P_enable_int 20: %x \n", reg_val);

	
	// enable wakeup
	reg_val = omap_readl(0x480020b0);
	reg_val |= (1<<30);
	omap_writel(reg_val, 0x480020b0);
//	reg_val = omap_readl(0x480020b0);
//	printk("P_enable_int pad: %x \n", reg_val);

}

void P_disable_int(void)
{
	u32 reg_val = 0;

	debug("   INTERRUPT DISABLED");
	disable_irq_wake(P_IRQ);

	// disable irq
	reg_val = omap_readl(0x4905001C);
	reg_val &= 0xFFDFFFFF; 
	omap_writel(reg_val, 0x4905001C);
//	reg_val = omap_readl(0x4905001C);
//	printk("P_disable_int 1C : %x \n", reg_val);

	// disable wakeup
	reg_val = omap_readl(0x49050020);
	reg_val &= 0xFFDFFFFF;
	omap_writel(reg_val, 0x49050020);
//	reg_val = omap_readl(0x49050020);
//	printk("P_disable_int 20: %x \n", reg_val);	

	// disable wakeup
	reg_val = omap_readl(0x480020b0);
	reg_val &= ~(1<<30);
	omap_writel(reg_val, 0x480020b0);
//	reg_val = omap_readl(0x480020b0);
//	printk("P_enable_int pad: %x \n", reg_val);

}

/*****************************************************************************/
/******************           LIGHT SENSOR SPECIFIC          *****************/
/*****************************************************************************/
// GPIO Setting
static void GPIO_setting( void )
{
	u32 reg_val;
	void __iomem*   gpio53_pad_config_reg;

	gpio53_pad_config_reg = ( void __iomem * )OMAP2_L3_IO_ADDRESS( 0x480020B0 );  // 0x480020B0 : gpio_53
	reg_val = __raw_readl( gpio53_pad_config_reg );

	reg_val &= 0xFEE0FFFF; // save origin val
	reg_val |= 0x011C0000; // input En, PullType UP, Pull En, Muxmod 4 : GPIO

	__raw_writel( reg_val, gpio53_pad_config_reg );

#if defined(CONFIG_MACH_SAMSUNG_LATONA) && CONFIG_SAMSUNG_REL_HW_REV >= 1
	gpio_free(OMAP_GPIO_ALS_EN);
	if(gpio_is_valid(OMAP_GPIO_ALS_EN))
	{
		if(gpio_request(OMAP_GPIO_ALS_EN, NULL))
		{
			printk(KERN_ERR "Failed to request OMAP_GPIO_ALS_EN!\n");
		}
		gpio_direction_output(OMAP_GPIO_ALS_EN, 0);
	}
#endif

	trace_out(); 
}

int __init PL_driver_init(void)
{
	int ret = 0;

	printk("[PL_SENSOR] INIT\n");
	
	trace_in(); 

	/*Proximity sensor: initialize user count*/
	mutex_init(&(P_user.lock));
	P_user.count = 0;

	/*Proximity sensor: Initilize the P dev synchronization mechanism*/
	P_dev_sync_mech_init();

	debug("module param an_sleep_func = %d, detec_cyc = %d", an_sleep_func, detec_cyc);

	/*Proximity sensor: initialize analog sleep function state and detection cycle*/
	P_dev_an_func_state_init(an_sleep_func);
	P_dev_detec_cyc_init(detec_cyc);

	/*Light sensor: Initilize the L dev synchronization mechanism*/
	L_dev_sync_mech_init();    

	/*Proximity sensor: Initialize the waitq*/
	P_waitq_init();

	if( (ret = pl_power_init()) < 0 )
	{
		error(" pl_power_init failed");
		failed(0);
		return ret; 	  	
	}

	/*Proximity sensor: ISR's bottom half*/
	INIT_WORK(&P_ws, (void (*) (struct work_struct *))P_dev_work_func);

	GPIO_setting();

	/*Proximity sensor: IRQ*/
	set_irq_type(P_IRQ, IRQF_TRIGGER_FALLING);

	if( (ret = request_irq(P_IRQ,P_isr,0 ,DEVICE_NAME, (void *)NULL)) < 0 )
	{
		error("request_irq failed %d", P_IRQ);
		failed(3);
		return -EIO;
	}

	enable_irq_wake(P_IRQ); // keep wakeup attr in sleep state

	wake_lock_init(&P_sensor_wake_lock, WAKE_LOCK_SUSPEND, "P_sensor");

#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
	/*Add the i2c driver*/
	if ( (ret = PL_i2c_drv_init() < 0) ) 
	{
		printk(KERN_ERR "[PSENSOR] PL_i2c_drv_init returns[%d]!\n", ret);
		free_irq(P_IRQ, (void *)NULL);
		pl_power_exit();
		return ret;
	}
#else
	this_client = omap_gpio_i2c_init(OMAP_GPIO_ALS_SDA, OMAP_GPIO_ALS_SCL, 0x44, 200);
	
	if((ret = P_dev_init()) < 0)
	{
		printk(KERN_ERR "[PSENSOR] P_dev_init returns[%d]!\n", ret);
		free_irq(P_IRQ, (void *)NULL);
		pl_power_exit();
		return ret;
	}
	
	if((ret = L_dev_init()) < 0)
	{
		printk(KERN_ERR "[PSENSOR] L_dev_init returs[%d]!\n", ret);
		free_irq(P_IRQ, (void *)NULL);
		P_dev_exit();
		pl_power_exit();
		return ret;
	}
#endif	// CONFIG_INPUT_GP2A_USE_GPIO_I2C
	
	trace_out();
	return ret;
}

void __exit PL_driver_exit(void)
{
	trace_in(); 

#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
	/*Delete the i2c driver*/
	PL_i2c_drv_exit();
#endif

	gpio_free(OMAP_GPIO_PS_VOUT);

#if defined(CONFIG_MACH_SAMSUNG_LATONA) && CONFIG_SAMSUNG_REL_HW_REV >= 1
	gpio_set_value(OMAP_GPIO_ALS_EN, 0);
	gpio_free(OMAP_GPIO_ALS_EN);
#endif

	/*Proximity sensor: IRQ*/
	free_irq(P_IRQ, (void *)NULL);

	pl_power_exit();
	trace_out();
}

module_init(PL_driver_init);
module_exit(PL_driver_exit);

MODULE_AUTHOR("Varun Mahajan <m.varun@samsung.com>");
MODULE_DESCRIPTION("GP2AP002A00F light/proximity sensor driver");
MODULE_LICENSE("GPL");
