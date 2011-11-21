/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2009 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/
#include <linux/pwm.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/irq.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/leds.h>
#include <asm/mach/irq.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/timed_output.h>

#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <asm/delay.h>
#include <plat/board.h>
#include <plat/dmtimer.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <asm/mach/time.h>
#include <mach/io.h>
#include <asm/mach-types.h>
#include <plat/mux.h>


#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** This SPI supports only one actuator.
*/
/*For 100%Duty Cycle*/
#define PWM_DUTY_MAX	CONFIG_SAMSUNG_VIBETONZ_PWM_DUTY_MAX

#define VIBE_GPTIMER_NUM	CONFIG_SAMSUNG_VIBETONZ_GPTIMER
#define _VIBE_GPTIMER_NAME(num)	"gptimer"#num
#define VIBE_GPTIMER_NAME(num)	_VIBE_GPTIMER_NAME(num)

	/* Error and Return value codes */
#define VIBE_S_SUCCESS	0	/*!< Success */
#define VIBE_E_FAIL		-4	/*!< Generic error */
	
#define GPIO_LEVEL_LOW				0
#define GPIO_LEVEL_HIGH				1
	
#define TIMER_DISABLED				0
#define TIMER_ENABLED				1

#define NUM_ACTUATORS 1


static bool g_bAmpEnabled = false;


static int g_pwmvalue=PWM_DUTY_MAX;
static int g_dutycycle;

spinlock_t vib_lock;


static struct omap_dm_timer *gptimer;	/*For OMAP3430 "gptimer9"*/

extern int omap_setup_irq(unsigned int irq, struct irqaction *act);

static ssize_t pwmvalue_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	
	count = sprintf(buf,"%d\n", g_pwmvalue);
	
	return count;
}

ssize_t pwmvalue_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	g_pwmvalue = simple_strtoul(buf, NULL, 10);	

	return size;
}

static DEVICE_ATTR(pwmvalue, S_IRUGO | S_IWUSR, pwmvalue_show, pwmvalue_store);

/*
static irqreturn_t vibtonz_omap2_gp_timer_interrupt(int irq,void *dev_id)
{
	unsigned long flags;
	struct omap_dm_timer *gpt = NULL;
	
	spin_lock_irqsave(&(vib_lock), flags);
    
	gpt = (struct omap_dm_timer *)dev_id;
	if(omap_dm_timer_get_irq(gpt)) {
		omap_dm_timer_write_status(gpt,OMAP_TIMER_INT_OVERFLOW);
	}

	spin_unlock_irqrestore(&(vib_lock), flags);

	spin_lock_irqsave(&(vib_lock), flags);

	if(omap_dm_timer_get_irq(gpt)) {
		omap_dm_timer_write_status(gpt,OMAP_TIMER_INT_MATCH);
	}

	spin_unlock_irqrestore(&(vib_lock), flags);

	return IRQ_HANDLED;
}

static struct irqaction omap2_gp_timer_irq={
	.name=VIBE_GPTIMER_NAME(VIBE_GPTIMER_NUM),
	.flags=IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler=vibtonz_omap2_gp_timer_interrupt,
};
*/

 /*
 *  For requesting the "GPtimer#" also for setting the default value in registers
 */
static int vibtonz_ReqGPTimer(void)
{
	int ret;

//	printk("[VIBRATOR] %s \n",__func__);
	
	gptimer=omap_dm_timer_request_specific(VIBE_GPTIMER_NUM);

	if (gptimer == NULL) {
//		printk("failed to request pwm timer\n");
		ret = -ENODEV;
	}

//	omap_dm_timer_enable(gptimer);
	omap_dm_timer_set_source(gptimer, OMAP_TIMER_SRC_SYS_CLK);
	omap_dm_timer_disable(gptimer);
//	omap_dm_timer_set_load(gptimer, 1, 0xffffff00);

	/*
	   Change for Interrupt
	 */
//	omap2_gp_timer_irq.dev_id=(void *)gptimer;
//	setup_irq(omap_dm_timer_get_irq(gptimer),&omap2_gp_timer_irq);		/* Request for interrupt Number */
//	omap_dm_timer_set_int_enable(gptimer,OMAP_TIMER_INT_OVERFLOW);
//	omap_dm_timer_set_int_enable(gptimer,OMAP_TIMER_INT_MATCH);
//	omap_dm_timer_stop(gptimer);
//	omap_dm_timer_disable(gptimer);
	return 0;
}

void vibtonzGPtimer_enable()
{
	omap_dm_timer_enable(gptimer);
}

/*
 * For setting the values in registers for varying the duty cycle and time period
 */
static void vibtonz_GPTimerSetValue(unsigned long load,unsigned long cmp)
{
	unsigned long load_reg, cmp_reg;
//	printk("[VIBRATOR] %s \n",__func__);

	load_reg =  load;	/* For setting the frequency=22.2Khz */
	cmp_reg =  cmp;		/* For varying the duty cycle */

	omap_dm_timer_enable(gptimer);
	omap_dm_timer_set_load(gptimer, 1, -load_reg);
	omap_dm_timer_set_match(gptimer, 1, -cmp_reg);
	omap_dm_timer_set_pwm(gptimer, 0, 1,OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_write_counter(gptimer, -2);
	omap_dm_timer_save_context(gptimer);
}


/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
#if 0
#error Please review the code between the #if and #endif

    if (g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable.\n"));

        g_bAmpEnabled = false;

#if 0
        mhn_gpio_set_level(GPIO_EN, GPIO_LEVEL_LOW);
	    mz_ops.bstat &= ~HN_BATTERY_MOTOR;
#endif
    }
#endif
//unsigned long flags;

//		spin_lock_irqsave(&(vib_lock), flags);
		if (g_bAmpEnabled) {
	//		printk(KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable.\n");
			g_bAmpEnabled = false;
			gpio_set_value(OMAP_GPIO_VIBTONE_EN, GPIO_LEVEL_LOW);
//			omap_dm_timer_stop(gptimer);	
		}
//		spin_unlock_irqrestore(&(vib_lock), flags);	
		return VIBE_S_SUCCESS;

}

/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
#if 0
#error Please review the code between the #if and #endif

    if (!g_bAmpEnabled)
    {
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpEnable.\n"));

        g_bAmpEnabled = true;

#if 0
        /* 
        ** Ensure the PWM frequency is at the expected value. These 2 lines of code
        ** can be removed if no other application alters the PWM frequency.
        */
        PWM_CTRL  = 0;                  /* 13Mhz / (0 + 1) = 13MHz */
        PWM_PERIOD = PWM_DUTY_MAX;      /* 13Mhz / (PWM_DUTY_MAX + 1) = 22.4kHz */

        /* Set duty cycle to 50% */
        PWM_DUTY = (PWM_DUTY_MAX+1)>>1; /* Duty cycle range = [0, PWM_DUTY_MAX] */

        /* Enable amp */
        mhn_gpio_set_level(GPIO_EN, GPIO_LEVEL_HIGH);
        mz_ops.bstat |= HN_BATTERY_MOTOR;
#endif
    }
#endif
//unsigned long flags;
//		spin_lock_irqsave(&(vib_lock), flags);
		if (!g_bAmpEnabled) {
	//		printk(KERN_DEBUG "ImmVibeSPI_ForceOut_AmpEnable.\n");
			g_bAmpEnabled = true;
			gpio_set_value(OMAP_GPIO_VIBTONE_EN, GPIO_LEVEL_HIGH);
//			vibtonz_GPTimerSetValue(PWM_DUTY_MAX, PWM_DUTY_VALUE); /* set the duty according to the modify value later */
//			omap_dm_timer_start(gptimer);  /* start the GPtimer9 */ 		
		}
//		spin_unlock_irqrestore(&(vib_lock), flags);	
		return VIBE_S_SUCCESS;

}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
	{
	//	printk("[VIBRATOR] %s \n",__func__);
	
		/* Disable amp */
	    spin_lock_init(&(vib_lock));
		g_bAmpEnabled = true;	/* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */
		ImmVibeSPI_ForceOut_AmpDisable(0);
		vibtonz_ReqGPTimer();
		return VIBE_S_SUCCESS;
	}


/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
//#error Please review the code between the #if and #endif

 //   DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Terminate.\n"));

    /* 
    ** Disable amp.
    ** If multiple actuators are supported, please make sure to call ImmVibeSPI_ForceOut_AmpDisable
    ** for each actuator (provide the actuator index as input argument).
    */
    ImmVibeSPI_ForceOut_AmpDisable(0);

#if 0
    /* Set PWM frequency */
    PWM_CTRL  = 0;                  /* 13Mhz / (0 + 1) = 13MHz */
    PWM_PERIOD = PWM_DUTY_MAX;      /* 13Mhz / (PWM_DUTY_MAX + 1) = 22.4kHz */

    /* Set duty cycle to 50% */
    PWM_DUTY = (PWM_DUTY_MAX+1)>>1; /* Duty cycle range = [0, PWM_DUTY_MAX] */
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Set(VibeUInt8 nActuatorIndex, VibeInt8 nForce)
	{
	//	printk("[VIBRATOR] %s \n",__func__);
		unsigned long flags;
		spin_lock_irqsave(&(vib_lock), flags);

		if (nForce == 0) {	
			ImmVibeSPI_ForceOut_AmpDisable(0);
			omap_dm_timer_enable(gptimer);
			omap_dm_timer_stop(gptimer);
		}
		else {
			ImmVibeSPI_ForceOut_AmpEnable(0);
			/*
			 * Formula for matching the user space force (-127 to +127) to Duty cycle.
			 * Duty cycle will vary from 0 to 45('0' means 0% duty cycle,'45' means
			 * 100% duty cycle.Also if user space force equals to -127 then duty cycle will be 0
			 * (0%),if force equals to 0 duty cycle will be 22.5(50%),if +127 then duty cycle will
			 * be 45(100%)
			 */
			 g_dutycycle = ((nForce + 128) * (g_pwmvalue>>1)/128);
            
			if (g_pwmvalue>0)
			{
				vibtonz_GPTimerSetValue(g_pwmvalue, g_dutycycle); /* set the duty according to the modify value later */
			}
			omap_dm_timer_start(gptimer);  /* start the GPtimer9 */ 

		}
		spin_unlock_irqrestore(&(vib_lock), flags);

		return VIBE_S_SUCCESS;
	}


/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    /* This function is not called for LRA device */
    return VIBE_S_SUCCESS;
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
//#error Please review the code between the #if and #endif

#if 0 
    #error  "The OEM must handle different frequency parameters here"
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
//#error Please review the code between the #if and #endif

#if 0   /* The following code is provided as a sample. Please modify as required. */
    if ((!szDevName) || (nSize < 1)) return VIBE_E_FAIL;

    DbgOut((KERN_DEBUG "ImmVibeSPI_Device_GetName.\n"));

    strncpy(szDevName, "Generic Linux Device", nSize-1);
    szDevName[nSize - 1] = '\0';    /* make sure the string is NULL terminated */
#endif

    return VIBE_S_SUCCESS;
}
