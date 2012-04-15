/*
 * linux/drivers/i2c/chips/twl4030_poweroff.c
 *
 * Power off device
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Written by Peter De Schrijver <peter.de-schrijver@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/pm.h>
#include <linux/i2c/twl.h>

#include <mach/gpio.h>
#include <plat/mux.h>
#include <plat/microusbic.h>
#if defined(CONFIG_USB_ANDROID)
#include <linux/usb/android_composite.h>
#endif
#include <linux/reboot.h>
#include <linux/delay.h>

#define PWR_P1_SW_EVENTS	0x10
#define PWR_DEVOFF	(1<<0)

#define REG_RTC_INTERRUPT_REG       0X0F

/* FIXME: GPIO configurations should be moved to the other header file */

//#define GPIO_MSM_RST	178
//#define GPIO_FONE_ACTIVE 140	

static void twl4030_poweroff(void)
{
	u8 val;
	int err;

    // Clear IT_ALARM bit in RTC_INTERRUPTS_REG [+]
	err = twl_i2c_read_u8(TWL4030_MODULE_RTC, &val,
                    REG_RTC_INTERRUPT_REG);
	if (err) {
		printk(KERN_WARNING "I2C error %d while reading TWL4030"
					"REG_RTC_INTERRUPT_REG\n", err);
		return ;
	}

	val &= ~(0x01<<3);
	err = twl_i2c_write_u8(TWL4030_MODULE_RTC, val,
                    REG_RTC_INTERRUPT_REG);
	if (err) {
		printk(KERN_WARNING "I2C error %d while writing TWL4030"
					"REG_RTC_INTERRUPT_REG\n", err);
		return ;
	}

#if 0
	err = twl_i2c_read_u8(TWL4030_MODULE_RTC, &val,
                    REG_RTC_INTERRUPT_REG);
	if (err) {
		printk(KERN_WARNING "I2C error %d while reading TWL4030"
					"REG_RTC_INTERRUPT_REG\n", err);
		return ;
	}
	printk("Clear IT ALARM bit in RTC INTERRUPT REG( = 0x%02x) Done!\n", val);
#endif
    // Clear IT_ALARM bit in RTC_INTERRUPTS_REG [-]

	err = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &val,
				  PWR_P1_SW_EVENTS);
	if (err) {
		printk(KERN_WARNING "I2C error %d while reading TWL4030"
					"PM_MASTER P1_SW_EVENTS\n", err);
		return ;
	}

	val |= PWR_DEVOFF;

#if 0
	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, val,
				   PWR_P1_SW_EVENTS);

	if (err) {
		printk(KERN_WARNING "I2C error %d while writing TWL4030"
					"PM_MASTER P1_SW_EVENTS\n", err);
		return ;
	}
#else
    while(1) {
    	err = twl_i2c_write_u8(TWL4030_MODULE_PM_MASTER, val,
    				   PWR_P1_SW_EVENTS);
        mdelay(500);
    };
#endif

	return;
}

extern void omap_watchdog_reset(void);
static void board_poweroff(void)
{
	/* int n_usbic_state; */

	printk("\nPower off routine - Board Shutdown!! \n");

/*
#if defined(CONFIG_USB_ANDROID)
	android_usb_set_connected(0);
#endif
*/

	/* get_real_usbic_state(); */
	//gpio_direction_output(GPIO_MSM_RST,0);
	//gpio_direction_output(GPIO_FONE_ACTIVE, 0);
	// if (GPIO_TA_CONNECTED_N is LOW)
#if ( defined( CONFIG_MACH_SAMSUNG_LATONA ) ) // jypark72, to avoid build error
	if ( !gpio_get_value( OMAP_GPIO_TA_NCONNECTED ) || gpio_get_value( OMAP_GPIO_JIG_ON18 ) )
#elif ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
	if (gpio_get_value( OMAP_GPIO_JIG_ON18 ) )
#endif
	{
		printk("Warmreset by TA or USB or Jtag - check 2 pins : JIG_ON18, TA_nConnected \n\n");

		preempt_disable();
		local_irq_disable();
		local_fiq_disable();

#ifdef CONFIG_SAMSUNG_KERNEL_DEBUG
		omap_writel(0x54455352, OMAP343X_CTRL_BASE + 0x9C4); // set to normal reset
#endif

#if 1
		/* using watchdog reset */
		omap_watchdog_reset();
		//machine_restart("ta_inserted");
#else
		/* using core_dpll_warmreset with global reset */
		//omap3_configure_core_dpll_warmreset();
		machine_restart("ta_inserted");
#endif

		while(1);
	}
	else
	{
		while(1)
		{
			if (gpio_get_value(OMAP_GPIO_KEY_PWRON)) {
				printk("Power button is pressed\n\n");
			}
			else {
				printk("Power Off !\n\n");
				gpio_set_value( OMAP_GPIO_PS_HOLD_PU, 0 ); 
				twl4030_poweroff();
				//for(;;);
			}
		}
	}
	return;
}



static int __init twl4030_poweroff_init(void)
{
	pm_power_off = board_poweroff;

	return 0;
}

static void __exit twl4030_poweroff_exit(void)
{
	pm_power_off = NULL;
}

module_init(twl4030_poweroff_init);
module_exit(twl4030_poweroff_exit);

MODULE_ALIAS("i2c:twl4030-poweroff");
MODULE_DESCRIPTION("Triton2 device power off");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter De Schrijver");
