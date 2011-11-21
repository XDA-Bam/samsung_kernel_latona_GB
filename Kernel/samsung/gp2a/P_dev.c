/*
 * P_dev.c 
 *
 * Description: This file implements the Proximity sensor module
 *
 * Author: Varun Mahajan <m.varun@samsung.com>
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/i2c/twl.h>
#include <linux/input.h>

#define I2C_M_WR 0
#define PHY_TO_OFF_PM_RECIEVER(p)	(p - 0x5b)

#include "P_regs.h"
#include "main.h"
#include "P_dev.h"
#include "common.h"

#if defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
#include <plat/i2c-omap-gpio.h>
extern OMAP_GPIO_I2C_CLIENT *this_client;
#endif

/**************************************************************/
#define  NOT_DETECTED  1
#define  DETECTED      0

/*power state*/
#define  SHUTDOWN      P_SHUTDOWN
#define  OPERATIONAL   P_OPERATIONAL

/*detection distance*/
#define  D_40_mm       40
#define  D_50_mm       50

/*detection hysteresis*/
#define  HYST_0p       0  /*0% (polling mode)*/
#define  HYST_10p      10 /*10%*/
#define  HYST_20p      20 /*20%*/

/*interrupt state*/
#define  DISABLED      0
#define  ENABLED       1

#define P_SYSFS_ERROR              -1
#define P_SYSFS_DEV_SHUTDOWN        0
#define P_SYSFS_MODE_A_OPERATIONAL  1
#define P_SYSFS_MODE_B_OPERATIONAL  2

#define P_SYSFS_PWRUP_MODE_A        3
#define P_SYSFS_PWRUP_MODE_B        4
#define P_SYSFS_SHUTDOWN            5
#define P_SYSFS_RETURN_TO_OP        6
#define P_SYSFS_RESET               7

#define P_SYSFS_OBJ_NOT_DETECTED    0
#define P_SYSFS_OBJ_DETECTED        1

/**************************************************************/
/*Data Structures*/
/**************************************************************/
/*
 * operation_mode: mode of operation
 * detection_dist: detection distance
 * detection_dist_hyst: detection distance hysteresis
 * detection_cyc: detection cycle
 * sleep_func_state: analog sleep function state
 * int_state: interrupt state
 */
typedef struct
{
	u16 operation_mode;
	u16 detection_dist;
	u16 detection_dist_hyst;
	u16 detection_cyc;
	u16 sleep_func_state;
	u16 int_status;
} P_dev_settings_t;

/*
 * lock: mutex to serialize the access to this data structure 
 * client: pointer to the i2c client struture for this device
 * power_state: operational or shutdown
 * device_state: specifies whether the sensor is present in the system or not 
 * regs: sensor's registers
 */
typedef struct 
{
	struct mutex lock;
#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
    struct i2c_client const *client;
#endif
	u16 power_state;
	P_dev_settings_t settings;
	u16 device_state;
	u8 regs[NUM_OF_REGS];
	struct input_dev *inputdevice;
	int delay;
} P_device_t;

/* extern functions */
void P_dev_sync_mech_init(void); 
void P_dev_an_func_state_init(u16);
void P_dev_detec_cyc_init(u16);

int P_dev_init_detec_cyc(u16 *);

#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
int P_dev_init(struct i2c_client *);
#else
int P_dev_init(void);
#endif
int P_dev_exit(void);

int P_dev_shutdown(void);
int P_dev_return_to_op(void);

int P_dev_reset(void);

void P_dev_check_wakeup_src(void);

int P_dev_get_mode(u16 *);
int P_dev_get_pwrstate_mode(u16 *, u16 *);

void P_dev_work_func(struct work_struct *);

int P_dev_powerup_set_op_mode(u16);

int P_dev_get_prox_output(u16 *);

extern int pl_sensor_power_on();
extern int pl_sensor_power_off();

/*static functions*/
static void enable_int(void);
static void disable_int(void);

static int vaux1_ldo_up(void);
static int vaux1_ldo_down(void);
static void vaux1_ldo_map_active(void);
static void vaux1_ldo_map_default(void);

static int make_operational(void);
static int shutdown(void);

static int set_mode(u16, u16, u16);

static int get_prox_output(u16 *);

static int i2c_read(u8);
static int i2c_write(u8);

int P_sysfs_init(struct input_dev *);
void P_sysfs_exit(struct input_dev *);

/*Proximity sensor device structure*/
static P_device_t P_dev =
{
#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
    .client = NULL,
#endif
	.power_state = SHUTDOWN,
	.settings = 
	{
		.operation_mode = P_NOT_INITIALIZED,
		.detection_cyc = P_CYC_8ms, 
		.sleep_func_state = P_AN_SLEEP_OFF, 
		.int_status = ENABLED, 
	},
	.device_state = NOT_DETECTED,
	.inputdevice = NULL,
};

/* for synchronization mechanism */
#define LOCK()    mutex_lock(&(P_dev.lock))
#define UNLOCK()  mutex_unlock(&(P_dev.lock))

void P_dev_sync_mech_init(void)
{
	mutex_init(&(P_dev.lock));
}

/* extern functions */
void P_dev_an_func_state_init(u16 state)
{
	trace_in();

	LOCK();

	if( (state != (u16)P_AN_SLEEP_OFF) && (state != (u16)P_AN_SLEEP_ON) )
	{
		failed(1);
	}
	else
	{
		P_dev.settings.sleep_func_state = state;
	}

	UNLOCK(); 

	trace_out();
}

void P_dev_detec_cyc_init(u16 detection_cyc)
{
	u16 cyc_range = P_CYC_1024ms;

	trace_in();

	LOCK();

	if( detection_cyc > cyc_range )
	{
		failed(1);
	}
	else
	{
		P_dev.settings.sleep_func_state = detection_cyc;
	}

	UNLOCK(); 

	trace_out();
}

#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
int P_dev_init(struct i2c_client *client)
#else
int P_dev_init()
#endif
{
	int ret = 0;

	trace_in();

	LOCK();   

	/*delay after Vcc is enabled*/
	mdelay(1);
	
	if( (ret = vaux1_ldo_up()) < 0 )
	{
		failed(1);
	}    
	else if( (ret = vaux1_ldo_down()) < 0 )
	{
		failed(2);
	}
	else if( !(P_dev.inputdevice = input_allocate_device()) )
	{
		ret = -ENOMEM;
		input_free_device(P_dev.inputdevice);
		failed(3);
	}
	
    set_bit(EV_ABS, P_dev.inputdevice->evbit);
    input_set_capability(P_dev.inputdevice, EV_ABS, ABS_DISTANCE);
    input_set_capability(P_dev.inputdevice, EV_ABS, ABS_BRAKE); 	/* status */
    input_set_capability(P_dev.inputdevice, EV_ABS, ABS_MISC); 		/* wake */
    input_set_capability(P_dev.inputdevice, EV_ABS, ABS_THROTTLE); 	/* enabled/delay */
	P_dev.inputdevice->name = "proximity_sensor";

	if((ret = input_register_device(P_dev.inputdevice))<0) 
	{
		failed(4);
	}
	else
	{
		vaux1_ldo_map_default();
#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
        P_dev.client = client;	
#endif
		P_dev.device_state = DETECTED;
		P_sysfs_init(P_dev.inputdevice);
	}

	UNLOCK();   
	
//-------------for test----------------
#if 0
		printk("proximity on\n");
		P_dev_powerup_set_op_mode(P_MODE_B);
#endif	
//-------------for test----------------

	trace_out();

	return ret;
}

int P_dev_exit(void)
{
	int ret = 0;

	trace_in();

	LOCK();

#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
    P_dev.client = NULL;	
#endif
	P_dev.device_state = NOT_DETECTED;

    P_sysfs_exit(P_dev.inputdevice);
	input_unregister_device(P_dev.inputdevice);	

	UNLOCK(); 

	trace_out();

	return ret;
}

int P_dev_shutdown(void)
{
	int ret = 0;

	trace_in();

	LOCK();   

	if( P_dev.device_state == NOT_DETECTED )
	{
		failed(1);
		ret = -ENODEV;   
	}
	else if ( P_dev.power_state == SHUTDOWN )
	{
		debug("    already shutdown");
		ret = 0;
	}
	else if( (ret = shutdown()) < 0 )
	{
		failed(2);
	}
	else
	{
		/*wake up the waiting processes*/
		P_waitq_wkup_proc();
	}

	UNLOCK(); 

	trace_out();

	return ret;
}	

int P_dev_return_to_op(void)
{
	int ret = 0;

	trace_in();

	LOCK();   

	if( P_dev.device_state == NOT_DETECTED )
	{
		failed(1);
		ret = -ENODEV;   
	}
	else if ( P_dev.power_state == OPERATIONAL )
	{
		debug("    already operational");
		ret = 0;
	}
	else if( (ret = make_operational()) < 0 )
	{
		failed(4);
	}

	UNLOCK(); 

	trace_out();

	return ret;
} 

int P_dev_reset(void)
{
	int ret = 0;

	trace_in();

	LOCK();   

	if( P_dev.device_state == NOT_DETECTED )
	{
		failed(1);
		ret = -ENODEV;   
	}
	else if ( P_dev.power_state == SHUTDOWN )
	{
		failed(2);
		ret = -EPERM;
	}
	else if( (ret = shutdown()) < 0 )
	{
		failed(3);
	}
	else if( (ret = make_operational()) < 0 )
	{
		failed(4);
	}     

	UNLOCK(); 

	trace_out();

	return ret;
}

void P_dev_check_wakeup_src(void)
{
	trace_in();
	trace_out();
}

int P_dev_get_mode(u16 *mode)
{
	int ret = 0;

	trace_in();

	LOCK();   
	debug("[ryun] P_dev_get_mode() P_dev.device_state = %d \n", P_dev.device_state);

	if( P_dev.device_state == NOT_DETECTED )
	{
		failed(1);
		ret = -ENODEV; 
	}
	else if( P_dev.power_state == SHUTDOWN )
	{
		failed(2);
		ret = -EPERM;
	}      
	else 
	{
		*mode = P_dev.settings.operation_mode;
	} 

	UNLOCK(); 

	trace_out();

	return ret;
}

int P_dev_get_pwrstate_mode(u16 *power_state, u16 *mode)
{
	int ret = 0;

	trace_in();

	LOCK();   

	if( P_dev.device_state == NOT_DETECTED )
	{
		failed(1);
		ret = -ENODEV; 
	}
	else 
	{
		*power_state = P_dev.power_state;
		*mode = P_dev.settings.operation_mode;
	} 

	UNLOCK(); 

	trace_out();

	return ret;
}

void P_dev_work_func(struct work_struct *work)
{
	u16 prox_op=P_OBJ_NOT_DETECTED;
	trace_in();

	LOCK(); 

	if( P_dev.device_state == NOT_DETECTED )
	{
		failed(1);
	}
	else if( P_dev.power_state == SHUTDOWN )
	{
		failed(2);
	}    
	else
	{
		disable_int();

		/*clear the interrupt and wake up the processes*/
		if( get_prox_output(&prox_op) < 0 )
		{
			failed(3);
		}
		else
		{
			printk("   prox output: 0x%x\n", (0x01 & P_dev.regs[PROX])); 
		}

		input_report_abs(P_dev.inputdevice, ABS_DISTANCE,  (0x01 & P_dev.regs[PROX])? DETECTED : NOT_DETECTED);
		input_sync(P_dev.inputdevice);
		P_waitq_wkup_proc();
	} 

	UNLOCK(); 

	trace_out();
}

int P_dev_get_prox_output(u16 *prox_op)
{
	int ret = 0;

	trace_in();

	LOCK();   

	if( P_dev.device_state == NOT_DETECTED )
	{
		failed(1);
		ret = -ENODEV; 
	}
	else if( P_dev.power_state == SHUTDOWN )
	{
		failed(2);
		ret = -1;
	}
	else if( (ret = get_prox_output(prox_op)) < 0 )
	{
		failed(3);
	}

	UNLOCK(); 

	trace_out();

	return ret;
}

int P_dev_powerup_set_op_mode(u16 mode)
{
	int ret = 0;

	trace_in();

	LOCK();

	if( P_dev.device_state == NOT_DETECTED )
	{
		failed(1);
		ret = -ENODEV; 
	}
	else if( (ret = set_mode(mode, P_dev.settings.detection_cyc, 
			  P_dev.settings.sleep_func_state)) < 0 )
	{
		failed(2);
	}

	UNLOCK(); 

	trace_out();

	return ret;
}

/**************************************************************/
/*static functons*/
/**************************************************************/
static void enable_int(void)
{
	trace_in();

	if( P_dev.settings.int_status == DISABLED )
	{
		P_enable_int();
		P_dev.settings.int_status = ENABLED;
	}
	else
	{
		debug("   IMBALANCE (this is not an error)");
	}

	trace_out();
}

static void disable_int(void)
{
	trace_in();

	if( P_dev.settings.int_status == ENABLED )
	{
		P_disable_int();
		P_dev.settings.int_status = DISABLED;
	}  
	else
	{
		debug("   IMBALANCE (this is not an error)");
	}    

	trace_out();
}

static int vaux1_ldo_up(void)
{
	int ret = 0;

	trace_in();

	debug("[ryun] power up Proximity/Light chip....\n");
	pl_sensor_power_on();

	trace_out();

	return ret;
}

static int vaux1_ldo_down(void)
{
	int ret = 0;

	trace_in();

	debug("[ryun] power down Proximity/Light chip....\n");

	pl_sensor_power_off();

	trace_out();

	return ret;
}

static void vaux1_ldo_map_active(void)
{
	trace_in();
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x99, PHY_TO_OFF_PM_RECIEVER(0x74));
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x99, PHY_TO_OFF_PM_RECIEVER(0x78));
	trace_out();
}

static void vaux1_ldo_map_default(void)
{
	trace_in();
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00, PHY_TO_OFF_PM_RECIEVER(0x74));
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00, PHY_TO_OFF_PM_RECIEVER(0x78));
	trace_out();
}

static int set_mode(u16 mode, u16 detection_cyc, u16 sleep_func_state)
{
	int ret = 0;
	int i;
	u16 cyc_range = P_CYC_1024ms;

	trace_in();

	if( P_dev.power_state == OPERATIONAL )
	{
		shutdown();
	}

	if( (ret = vaux1_ldo_up()) < 0 )
	{
		failed(12);
		goto OUT;
	}

	/*initialize all the registers to 0*/
	for( i = 0; i < NUM_OF_REGS; i++ )
	{
		P_dev.regs[i] = 0;
	}

	if( detection_cyc > cyc_range ) detection_cyc = P_CYC_8ms; /*default*/

	if( sleep_func_state != P_AN_SLEEP_ON ) 
		sleep_func_state = P_AN_SLEEP_OFF; /*default*/

	if( mode == P_MODE_A )
	{
		/*disable the host interrupt*/
		disable_int();

		/*The previous mode might be B and some processes 
		might be sleeping, so before changing the mode to A 
		we need to wake up those processes*/
		P_waitq_wkup_proc();

		P_dev.regs[GAIN] |= GAIN_LED0;

		P_dev.regs[HYS] |= (HYS_HYSC1|HYS_HYSF1|HYS_HYSD);

		P_dev.regs[CYCLE] |= ((detection_cyc << 3) & CYCLE_CYCL_MASK);
		P_dev.regs[CYCLE] |= CYCLE_OSC;

		/*disable the device VOUT*/
		P_dev.regs[CON] |= (CON_OCON1|CON_OCON0);

		if( sleep_func_state == P_AN_SLEEP_ON )
		{
			P_dev.regs[OPMOD] |= OPMOD_ASD;
		}
		
		P_dev.regs[OPMOD] |= OPMOD_SSD;

		if( (ret = i2c_write(GAIN)) < 0 )
		{
			failed(1);
		}
		else if( (ret = i2c_write(HYS)) < 0 )
		{
			failed(2);
		}        
		else if( (ret = i2c_write(CYCLE)) < 0 )
		{
			failed(3);
		}
		else if( (ret = i2c_write(CON)) < 0 )
		{
			failed(4);
		}          
		else if( (ret = i2c_write(OPMOD)) < 0 )
		{
			failed(5);
		}
		else
		{
			P_dev.settings.operation_mode = P_MODE_A;
			P_dev.settings.sleep_func_state = sleep_func_state;
			P_dev.settings.detection_cyc = detection_cyc;
			P_dev.settings.detection_dist = D_40_mm;
			P_dev.settings.detection_dist_hyst = HYST_0p;

			P_dev.power_state = OPERATIONAL;

			/*delay for the device to get stabilized*/
			mdelay(1);
		}
	}
	else if( mode == P_MODE_B )
	{
		P_dev.regs[GAIN] |= GAIN_LED0;

		P_dev.regs[HYS] = HYS_HYSC0|HYS_HYSF3|HYS_HYSF2|HYS_HYSF1|HYS_HYSF0;

		P_dev.regs[CYCLE] |= ((detection_cyc << 3) & CYCLE_CYCL_MASK);
		P_dev.regs[CYCLE] |= CYCLE_OSC;

		P_dev.regs[CON] &= ~(CON_OCON1|CON_OCON0);

		if( sleep_func_state == P_AN_SLEEP_ON )
		{
			P_dev.regs[OPMOD] |= OPMOD_ASD;
		}
		
		P_dev.regs[OPMOD] |= (OPMOD_SSD|OPMOD_VCON);

		if( (ret = i2c_write(GAIN)) < 0 )
		{
			failed(6);
		}
		if( (ret = i2c_write(CON)) < 0 )
		{
			failed(7);
		}        
		else if( (ret = i2c_write(HYS)) < 0 )
		{
			failed(8);
		}        
		else if( (ret = i2c_write(CYCLE)) < 0 )
		{
			failed(9);
		}
		else if( (ret = i2c_write(OPMOD)) < 0 )
		{
			failed(10);
		}
		else
		{
			vaux1_ldo_map_active();

			P_dev.settings.operation_mode = P_MODE_B;
			P_dev.settings.sleep_func_state = sleep_func_state;
			P_dev.settings.detection_cyc = detection_cyc;
			P_dev.settings.detection_dist = D_50_mm;

			/*If hyst value needs to be changed, change it here*/
			P_dev.settings.detection_dist_hyst = HYST_10p;

			P_dev.power_state = OPERATIONAL;

			/*delay for the device to get stabilized: this is done before 
			enabling the interrupt because it has been observed 
			that the device sends an interrupt once it is powered
			up in mode B, the first interrupt will be ignored*/ 
			mdelay(1);

			/* Ryunkyun.Park 20091109 for init value for proximity sensor. */
			debug("[PROXIMITY] input_report_abs(P_dev.inputdevice, ABS_DISTANCE,  NOT_DETECTED);\n");
			input_report_abs(P_dev.inputdevice, ABS_DISTANCE,  NOT_DETECTED);	// ryun 20091109 dummy value.
			input_sync(P_dev.inputdevice);

			enable_int();            
		}
	}
	else 
	{
		failed(11);
		ret = -EINVAL;
	}

OUT:
	trace_out();

	return ret;
}

static int get_prox_output(u16 *prox_op)
{
	int ret = 0;

	trace_in();

	if( (ret = i2c_read(PROX)) < 0 )
	{
		failed(1);
	}
	else
	{
		*prox_op = (P_dev.regs[PROX] & PROX_VO) ?
		P_OBJ_DETECTED : P_OBJ_NOT_DETECTED;
	}

	/*The mode is B and this read is the first read after the status change interrupt*/
	if( (P_dev.settings.operation_mode == P_MODE_B) &&
		(P_dev.settings.int_status == DISABLED) )
	{
		P_dev.regs[HYS] = 0;

		if( *prox_op == P_OBJ_NOT_DETECTED )
		{           
			P_dev.regs[HYS] = HYS_HYSC0|HYS_HYSF3|HYS_HYSF2|HYS_HYSF1|HYS_HYSF0;
		}
		else 
		{
			P_dev.regs[HYS] = HYS_HYSF3|HYS_HYSF2|HYS_HYSF1|HYS_HYSF0;
		}

		if( (ret = i2c_write(HYS)) < 0 )
		{
			failed(2);
			goto OUT;
		}

		P_dev.regs[CON] = 0;
		P_dev.regs[CON] |= (CON_OCON1|CON_OCON0);

		if( (ret = i2c_write(CON)) < 0 )
		{
			failed(3);
			goto OUT;
		}       

		enable_int();

		P_dev.regs[CON] = 0;
		P_dev.regs[CON] &= ~(CON_OCON1|CON_OCON0);

		if( (ret = i2c_write(INT_CLR(CON))) < 0 )
		{
			failed(4);
			goto OUT;
		} 
	}

	OUT:
	trace_out();

	return ret;
}

static int shutdown(void)
{
	int ret = 0;

	trace_in();

	if( P_dev.settings.operation_mode == P_MODE_B )
	{
		disable_int();
		P_dev.regs[OPMOD] |= OPMOD_VCON;
	}
	else if( P_dev.settings.operation_mode == P_MODE_A )
	{
		P_dev.regs[OPMOD] &= ~OPMOD_VCON;
	}

	P_dev.regs[OPMOD] &= ~OPMOD_SSD;

	if( (ret = i2c_write(OPMOD)) < 0 )
	{
		failed(1);
	}
	else
	{
		if( P_dev.settings.operation_mode == P_MODE_B )
		{
			vaux1_ldo_map_default();
		}
		
		mdelay(1);
		if( (ret = vaux1_ldo_down()) < 0 )
		{
			failed(2);
		}
		
		P_dev.power_state = SHUTDOWN;
	}

	trace_out();

	return ret;
}

static int make_operational(void)
{
	int ret = 0;

	trace_in();

	if( (ret = vaux1_ldo_up()) < 0 )
	{
		failed(6);
	}
	else if( P_dev.settings.operation_mode == P_MODE_A )
	{
		P_dev.regs[OPMOD] |= OPMOD_SSD;
		P_dev.regs[OPMOD] &= ~OPMOD_VCON;

		if( (ret = i2c_write(OPMOD)) < 0 )
		{
			failed(1);
		}
		else
		{
			P_dev.power_state = OPERATIONAL;
		}         
	}
	else if( P_dev.settings.operation_mode == P_MODE_B )
	{
		P_dev.regs[CON] = 0;
		P_dev.regs[CON] |= (CON_OCON1|CON_OCON0);

		if( (ret = i2c_write(CON)) < 0 )
		{
			failed(2);
			goto OUT;
		}

		P_dev.regs[HYS] = HYS_HYSC0|HYS_HYSF3|HYS_HYSF2|HYS_HYSF1|HYS_HYSF0;

		if( (ret = i2c_write(HYS)) < 0 )
		{
			failed(3);
			goto OUT;
		}

		P_dev.regs[OPMOD] |= (OPMOD_SSD|OPMOD_VCON);

		if( (ret = i2c_write(OPMOD)) < 0 )
		{
			failed(4);
			goto OUT;
		}

		enable_int();

		P_dev.regs[CON] = 0;
		P_dev.regs[CON] &= ~(CON_OCON1|CON_OCON0);

		if( (ret = i2c_write(CON)) < 0 )
		{
			failed(5);
			goto OUT;
		}

		vaux1_ldo_map_active();
		P_dev.power_state = OPERATIONAL;
	}

	mdelay(1);

OUT:
	trace_out();

	return ret;
}

static int i2c_read(u8 reg)
{
#if defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
	u8 aux[2] = {0.0};

	if( reg != PROX )
	{
		return -1;
	}

	i2c_rd_param.reg_len = 1;
	i2c_rd_param.reg_addr = &reg;
	i2c_rd_param.rdata_len = 2;
	i2c_rd_param.rdata = aux;
	omap_gpio_i2c_read(this_client, &i2c_rd_param);

	P_dev.regs[reg] = aux[1];
	debug("   read 0x%x: 0x%x", reg, P_dev.regs[reg]);

	return 0;
#else
    int ret = 0;
    struct i2c_msg msg[1];
    
    u8 aux[2] = {0.0};

    if( reg != PROX )
    {
        return -1;
    }

    msg[0].addr	= P_dev.client->addr;
    msg[0].flags = I2C_M_RD;
    msg[0].len   = 2;
    msg[0].buf = aux;

    ret = i2c_transfer(P_dev.client->adapter, msg, 1);

    if( ret < 0 )
    {
        failed(1);
        error("i2c_transfer failed %d", ret);
    }
    else if( ret >= 0 )
    {
        P_dev.regs[reg] = aux[1];
        debug("   read 0x%x: 0x%x", reg, P_dev.regs[reg]);
        ret = 0;
    }

    return ret;
#endif
}

static int i2c_write( u8 reg )
{
#if defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
	u8 buf[2];
    OMAP_GPIO_I2C_WR_DATA i2c_wr_param;

	if( ! ((NOT_INT_CLR(reg) > PROX) && (NOT_INT_CLR(reg) <= CON)) )
	{
		return -1;
	}

	debug("   write 0x%x: 0x%x", NOT_INT_CLR(reg), P_dev.regs[NOT_INT_CLR(reg)]);

	i2c_wr_param.reg_len = 1;
	i2c_wr_param.reg_addr = &reg;
	i2c_wr_param.wdata_len = 1;
	buf[0] = P_dev.regs[NOT_INT_CLR(reg)];
	i2c_wr_param.wdata = buf;
	omap_gpio_i2c_write(this_client, &i2c_wr_param);

	return 0;
#else
    int ret = 0;
    struct i2c_msg msg[1];
    u8 buf[2];

    if( ! ((NOT_INT_CLR(reg) > PROX) && (NOT_INT_CLR(reg) <= CON)) )
    {
        return -1;
    }
    
    debug("   write 0x%x: 0x%x", NOT_INT_CLR(reg), 
    	      P_dev.regs[NOT_INT_CLR(reg)]);

    msg[0].addr	= P_dev.client->addr;
    msg[0].flags = I2C_M_WR;
    msg[0].len = 2;

    buf[0] = reg;
    buf[1] = P_dev.regs[NOT_INT_CLR(reg)];
    msg[0].buf = buf;

    ret = i2c_transfer(P_dev.client->adapter, msg, 1);

    if( ret < 0 )
    {
        failed(1);
        error("i2c_transfer failed %d", ret);
    }

    if( ret >= 0 )
    {
        ret = 0;
    }

    return ret;
#endif
}

/*static functions for OSCAR*/
static ssize_t P_delay_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t P_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t P_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t P_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t P_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t P_data_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t P_status_show(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP, P_delay_show, P_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, P_enable_show, P_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, P_wake_store);
static DEVICE_ATTR(adc, S_IRUGO, P_data_show, NULL);
static DEVICE_ATTR(state, S_IRUGO, P_status_show, NULL);

static struct attribute *proximity_attributes[] = {
    &dev_attr_adc.attr,
    &dev_attr_state.attr,
    NULL
};

static struct attribute *P_attributes[] = {
    &dev_attr_poll_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
    NULL
};

static struct attribute_group P_attribute_group = {
    .attrs = P_attributes
};

extern struct class *sensors_class;
extern int sensors_register(struct device *dev, void * drvdata, struct device_attribute *attributes[], char *name);
static struct device *proximity_sensor_device;

int P_sysfs_init(struct input_dev * input_data)
{
    int ret = 0;

    trace_in();

    ret = sysfs_create_group(&input_data->dev.kobj, &P_attribute_group);
    if (ret) {
        printk(KERN_ERR "P_sysfs_init: sysfs_create_group failed[%s]\n", input_data->name);
    }
    ret = sensors_register(proximity_sensor_device, NULL, proximity_attributes, "proximity_sensor");
    if(ret) {
    	printk(KERN_ERR "%s: cound not register accelerometer sensor device(%d).\n", __func__, ret);
    }

    trace_out();

    return ret;
}

void P_sysfs_exit(struct input_dev * input_data)
{
    trace_in();

    sysfs_remove_group(&input_data->dev.kobj, &P_attribute_group);
	
    trace_out();
}


static ssize_t P_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned long flags;
    int x;

    spin_lock_irqsave(&P_dev.inputdevice->event_lock, flags);

    x = P_dev.inputdevice->abs[ABS_DISTANCE];

    spin_unlock_irqrestore(&P_dev.inputdevice->event_lock, flags);

    return sprintf(buf, "%d\n", x);
}


static ssize_t P_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t ret;
    int enabled = 0;
    u16 power_state, mode;
   
    trace_in();

    if( P_dev_get_pwrstate_mode(&power_state, &mode) < 0 )
    {
        enabled = 0;
        failed(1);
    }
    else
    {
        if( power_state == P_SHUTDOWN )
        {
            enabled = 0;
        }
        else if ( (mode == P_MODE_B) && (power_state == P_OPERATIONAL) )
        {
            enabled = 1;
        }        
    }
    
    debug("   enabled: %d", enabled);

    ret = sprintf(buf, "%d\n",enabled);

    trace_out();

    return ret;
}

static ssize_t P_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = 0;
	int enabled = 0;    
	trace_in();
	
	if (strncmp(buf, "0", 1) == 0 ) 
		{
			ret = 1;
			enabled = 0;
			printk("proximity off\n");
			if( P_dev_shutdown() < 0 )
			{
				printk("P_dev_shutdown() : fail!! \n");
				ret = -1;
			}
		}
	else if(strncmp(buf, "1", 1) == 0 ) 
		{
			ret = 1;
			enabled = 1;
			printk("proximity on\n");
			if( P_dev_powerup_set_op_mode(P_MODE_B) < 0 )
			{
				printk("P_dev_powerup_set_op_mode() : fail!! \n");
				ret = -1;
			}
		}

	input_report_abs(P_dev.inputdevice, ABS_THROTTLE, (enabled<<16) | P_dev.delay);
	
	trace_out();
	return ret;
}


static ssize_t P_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t ret;
	trace_in();

	debug("   P_interval_show()");
	ret = sprintf(buf, "%d\n", P_dev.delay);

	trace_out();
	return ret;
}

static ssize_t P_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strlen(buf);
	trace_in();

    int enabled = 0;
    u16 power_state, mode;
   
    trace_in();

    if( P_dev_get_pwrstate_mode(&power_state, &mode) < 0 )
    {
        enabled = 0;
    }
    else
    {
        if( power_state == P_SHUTDOWN )
        {
            enabled = 0;
        }
        else if ( (mode == P_MODE_B) && (power_state == P_OPERATIONAL) )
        {
            enabled = 1;
        }        
    }

    sscanf(buf, "%d", &P_dev.delay);

    input_report_abs(P_dev.inputdevice, ABS_THROTTLE, (enabled<<16) | P_dev.delay);

	return ret;
}

static ssize_t P_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    static int cnt = 1;

    input_report_abs(P_dev.inputdevice, ABS_MISC, cnt++);

    return count;
}

static ssize_t P_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned long flags;
    int status;

    spin_lock_irqsave(&P_dev.inputdevice->event_lock, flags);

    status = P_dev.inputdevice->abs[ABS_BRAKE];

    spin_unlock_irqrestore(&P_dev.inputdevice->event_lock, flags);

    return sprintf(buf, "%d\n", status);
}


