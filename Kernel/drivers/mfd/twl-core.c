/*
 * twl_core.c - driver for TWL4030/TWL5030/TWL60X0/TPS659x0 PM
 * and audio CODEC devices
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * Modifications to defer interrupt handling to a kernel thread:
 * Copyright (C) 2006 MontaVista Software, Inc.
 *
 * Based on tlv320aic23.c:
 * Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
 *
 * Code cleanup and modifications to IRQ handler.
 * by syed khasim <x0khasim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <linux/regulator/machine.h>

#include <linux/i2c.h>
#include <linux/i2c/twl.h>

#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)
#include <plat/cpu.h>
#endif

/*
 * The TWL4030 "Triton 2" is one of a family of a multi-function "Power
 * Management and System Companion Device" chips originally designed for
 * use in OMAP2 and OMAP 3 based systems.  Its control interfaces use I2C,
 * often at around 3 Mbit/sec, including for interrupt handling.
 *
 * This driver core provides genirq support for the interrupts emitted,
 * by the various modules, and exports register access primitives.
 *
 * FIXME this driver currently requires use of the first interrupt line
 * (and associated registers).
 */

#define DRIVER_NAME			"twl"

#if defined(CONFIG_TWL4030_BCI_BATTERY) || \
	defined(CONFIG_TWL4030_BCI_BATTERY_MODULE) || \
	defined(CONFIG_TWL6030_BCI_BATTERY) || \
	defined(CONFIG_TWL6030_BCI_BATTERY_MODULE)
#define twl_has_bci()		true
#else
#define twl_has_bci()		false
#endif

#if defined(CONFIG_KEYBOARD_TWL4030) || defined(CONFIG_KEYBOARD_TWL4030_MODULE)
#define twl_has_keypad()	true
#else
#define twl_has_keypad()	false
#endif

#if defined(CONFIG_GPIO_TWL4030) || defined(CONFIG_GPIO_TWL4030_MODULE)
#define twl_has_gpio()	true
#else
#define twl_has_gpio()	false
#endif

#if defined(CONFIG_REGULATOR_TWL4030) \
	|| defined(CONFIG_REGULATOR_TWL4030_MODULE)
#define twl_has_regulator()	true
#else
#define twl_has_regulator()	false
#endif

#if defined(CONFIG_TWL4030_MADC) || defined(CONFIG_TWL4030_MADC_MODULE) ||\
    defined(CONFIG_TWL6030_GPADC) || defined(CONFIG_TWL6030_GPADC_MODULE)
#define twl_has_madc()	true
#else
#define twl_has_madc()	false
#endif

#ifdef CONFIG_TWL4030_POWER
#define twl_has_power()        true
#else
#define twl_has_power()        false
#endif

#if defined(CONFIG_RTC_DRV_TWL4030) || defined(CONFIG_RTC_DRV_TWL4030_MODULE)
#define twl_has_rtc()	true
#else
#define twl_has_rtc()	false
#endif

#if defined(CONFIG_TWL4030_USB) || defined(CONFIG_TWL4030_USB_MODULE) ||\
    defined(CONFIG_TWL6030_USB) || defined(CONFIG_TWL6030_USB_MODULE)
#define twl_has_usb()	true
#else
#define twl_has_usb()	false
#endif

#if defined(CONFIG_TWL4030_WATCHDOG) || \
	defined(CONFIG_TWL4030_WATCHDOG_MODULE)
#define twl_has_watchdog()        true
#else
#define twl_has_watchdog()        false
#endif

#if defined(CONFIG_TWL4030_CODEC) || defined(CONFIG_TWL4030_CODEC_MODULE) ||\
	defined(CONFIG_TWL6040_CODEC) || defined(CONFIG_TWL6040_CODEC_MODULE)
#define twl_has_codec()	true
#else
#define twl_has_codec()	false
#endif

/* Triton Core internal information (BEGIN) */

/* Last - for index max*/
#define TWL4030_MODULE_LAST		TWL4030_MODULE_SECURED_REG

#define TWL_NUM_SLAVES		4

#if defined(CONFIG_INPUT_TWL4030_PWRBUTTON) \
	|| defined(CONFIG_INPUT_TWL4030_PWRBUTTON_MODULE)
#define twl_has_pwrbutton()	true
#else
#define twl_has_pwrbutton()	false
#endif

#define SUB_CHIP_ID0 0
#define SUB_CHIP_ID1 1
#define SUB_CHIP_ID2 2
#define SUB_CHIP_ID3 3

#define TWL_MODULE_LAST TWL4030_MODULE_LAST

/* Base Address defns for twl4030_map[] */

/* subchip/slave 0 - USB ID */
#define TWL4030_BASEADD_USB		0x0000

/* subchip/slave 1 - AUD ID */
#define TWL4030_BASEADD_AUDIO_VOICE	0x0000
#define TWL4030_BASEADD_GPIO		0x0098
#define TWL4030_BASEADD_INTBR		0x0085
#define TWL4030_BASEADD_PIH		0x0080
#define TWL4030_BASEADD_TEST		0x004C

/* subchip/slave 2 - AUX ID */
#define TWL4030_BASEADD_INTERRUPTS	0x00B9
#define TWL4030_BASEADD_LED		0x00EE
#define TWL4030_BASEADD_MADC		0x0000
#define TWL4030_BASEADD_MAIN_CHARGE	0x0074
#define TWL4030_BASEADD_PRECHARGE	0x00AA
#define TWL4030_BASEADD_PWM0		0x00F8
#define TWL4030_BASEADD_PWM1		0x00FB
#define TWL4030_BASEADD_PWMA		0x00EF
#define TWL4030_BASEADD_PWMB		0x00F1
#define TWL4030_BASEADD_KEYPAD		0x00D2

#define TWL5031_BASEADD_ACCESSORY	0x0074 /* Replaces Main Charge */
#define TWL5031_BASEADD_INTERRUPTS	0x00B9 /* Different than TWL4030's
						  one */

/* subchip/slave 3 - POWER ID */
#define TWL4030_BASEADD_BACKUP		0x0014
#define TWL4030_BASEADD_INT		0x002E
#define TWL4030_BASEADD_PM_MASTER	0x0036
#define TWL4030_BASEADD_PM_RECEIVER	0x005B
#define TWL4030_BASEADD_RTC		0x001C
#define TWL4030_BASEADD_SECURED_REG	0x0000

/* Triton Core internal information (END) */


/* subchip/slave 0 0x48 - POWER */
#define TWL6030_BASEADD_RTC		0x0000
#define TWL6030_BASEADD_MEM		0x0017
#define TWL6030_BASEADD_PM_MASTER	0x001F
#define TWL6030_BASEADD_PM_SLAVE_MISC	0x0030 /* PM_RECEIVER */
#define TWL6030_BASEADD_PM_MISC		0x00E2
#define TWL6030_BASEADD_PM_PUPD		0x00F0

/* subchip/slave 1 0x49 - FEATURE */
#define TWL6030_BASEADD_USB		0x0000
#define TWL6030_BASEADD_GPADC_CTRL	0x002E
#define TWL6030_BASEADD_AUX		0x0090
#define TWL6030_BASEADD_PWM		0x00BA
#define TWL6030_BASEADD_GASGAUGE	0x00C0
#define TWL6030_BASEADD_PIH		0x00D0
#define TWL6030_BASEADD_CHARGER		0x00E0

/* subchip/slave 2 0x4A - DFT */
#define TWL6030_BASEADD_DIEID		0x00C0

/* subchip/slave 3 0x4B - AUDIO */
#define TWL6030_BASEADD_AUDIO		0x0000
#define TWL6030_BASEADD_RSV		0x0000
#define TWL6030_BASEADD_ZERO		0x0000

/* Few power values */
#define R_CFG_BOOT			0x05
#define R_PROTECT_KEY			0x0E

/* access control values for R_PROTECT_KEY */
#define KEY_UNLOCK1			0xce
#define KEY_UNLOCK2			0xec
#define KEY_LOCK			0x00

/* some fields in R_CFG_BOOT */
#define HFCLK_FREQ_19p2_MHZ		(1 << 0)
#define HFCLK_FREQ_26_MHZ		(2 << 0)
#define HFCLK_FREQ_38p4_MHZ		(3 << 0)
#define HIGH_PERF_SQ			(1 << 3)
#define CK32K_LOWPWR_EN			(1 << 7)

#define CLK32KG_CFG_STATE	0xBE

#define CLK32KG_CFG_STATE	0xBE


/* chip-specific feature flags, for i2c_device_id.driver_data */
#define TWL4030_VAUX2		BIT(0)	/* pre-5030 voltage ranges */
#define TPS_SUBSET		BIT(1)	/* tps659[23]0 have fewer LDOs */
#define TWL5031			BIT(2)  /* twl5031 has different registers */
#define TWL6030_CLASS		BIT(3)	/* TWL6030 class */

/*----------------------------------------------------------------------*/

/* is driver active, bound to a chip? */
static bool inuse;

static unsigned int twl_id;
unsigned int twl_rev(void)
{
	return twl_id;
}
EXPORT_SYMBOL(twl_rev);

/* Structure for each TWL4030/TWL6030 Slave */
struct twl_client {
	struct i2c_client *client;
	u8 address;

	/* max numb of i2c_msg required is for read =2 */
	struct i2c_msg xfer_msg[2];

	/* To lock access to xfer_msg */
	struct mutex xfer_lock;
};

static struct twl_client twl_modules[TWL_NUM_SLAVES];


/* mapping the module id to slave id and base address */
struct twl_mapping {
	unsigned char sid;	/* Slave ID */
	unsigned char base;	/* base address */
};
static struct twl_mapping *twl_map;

static struct twl_mapping twl4030_map[TWL4030_MODULE_LAST + 1] = {
	/*
	 * NOTE:  don't change this table without updating the
	 * <linux/i2c/twl.h> defines for TWL4030_MODULE_*
	 * so they continue to match the order in this table.
	 */

	{ 0, TWL4030_BASEADD_USB },

	{ 1, TWL4030_BASEADD_AUDIO_VOICE },
	{ 1, TWL4030_BASEADD_GPIO },
	{ 1, TWL4030_BASEADD_INTBR },
	{ 1, TWL4030_BASEADD_PIH },
	{ 1, TWL4030_BASEADD_TEST },

	{ 2, TWL4030_BASEADD_KEYPAD },
	{ 2, TWL4030_BASEADD_MADC },
	{ 2, TWL4030_BASEADD_INTERRUPTS },
	{ 2, TWL4030_BASEADD_LED },
	{ 2, TWL4030_BASEADD_MAIN_CHARGE },
	{ 2, TWL4030_BASEADD_PRECHARGE },
	{ 2, TWL4030_BASEADD_PWM0 },
	{ 2, TWL4030_BASEADD_PWM1 },
	{ 2, TWL4030_BASEADD_PWMA },
	{ 2, TWL4030_BASEADD_PWMB },
	{ 2, TWL5031_BASEADD_ACCESSORY },
	{ 2, TWL5031_BASEADD_INTERRUPTS },

	{ 3, TWL4030_BASEADD_BACKUP },
	{ 3, TWL4030_BASEADD_INT },
	{ 3, TWL4030_BASEADD_PM_MASTER },
	{ 3, TWL4030_BASEADD_PM_RECEIVER },
	{ 3, TWL4030_BASEADD_RTC },
	{ 3, TWL4030_BASEADD_SECURED_REG },
};

static struct twl_mapping twl6030_map[] = {
	/*
	 * NOTE:  don't change this table without updating the
	 * <linux/i2c/twl.h> defines for TWL4030_MODULE_*
	 * so they continue to match the order in this table.
	 */
	{ SUB_CHIP_ID1, TWL6030_BASEADD_USB },
	{ SUB_CHIP_ID3, TWL6030_BASEADD_AUDIO },
	{ SUB_CHIP_ID2, TWL6030_BASEADD_DIEID },
	{ SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	{ SUB_CHIP_ID1, TWL6030_BASEADD_PIH },

	{ SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	{ SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	{ SUB_CHIP_ID1, TWL6030_BASEADD_GPADC_CTRL },
	{ SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	{ SUB_CHIP_ID2, TWL6030_BASEADD_RSV },

	{ SUB_CHIP_ID1, TWL6030_BASEADD_CHARGER },
	{ SUB_CHIP_ID1, TWL6030_BASEADD_GASGAUGE },
	{ SUB_CHIP_ID1, TWL6030_BASEADD_PWM },
	{ SUB_CHIP_ID0, TWL6030_BASEADD_ZERO },
	{ SUB_CHIP_ID1, TWL6030_BASEADD_ZERO },

	{ SUB_CHIP_ID2, TWL6030_BASEADD_ZERO },
	{ SUB_CHIP_ID2, TWL6030_BASEADD_ZERO },
	{ SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	{ SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	{ SUB_CHIP_ID2, TWL6030_BASEADD_RSV },
	{ SUB_CHIP_ID0, TWL6030_BASEADD_PM_MASTER },
	{ SUB_CHIP_ID0, TWL6030_BASEADD_PM_SLAVE_MISC },

	{ SUB_CHIP_ID0, TWL6030_BASEADD_RTC },
	{ SUB_CHIP_ID0, TWL6030_BASEADD_MEM },
};

/*----------------------------------------------------------------------*/

/* Exported Functions */

/**
 * twl_i2c_write - Writes a n bit register in TWL4030/TWL5030/TWL60X0
 * @mod_no: module number
 * @value: an array of num_bytes+1 containing data to write
 * @reg: register address (just offset will do)
 * @num_bytes: number of bytes to transfer
 *
 * IMPORTANT: for 'value' parameter: Allocate value num_bytes+1 and
 * valid data starts at Offset 1.
 *
 * Returns the result of operation - 0 is success
 */

extern int twl4030_get_voicecall_state(void);

static u8 codec_pdz_val=0;

int twl_i2c_write(u8 mod_no, u8 *value, u8 reg, unsigned num_bytes)
{
	int ret;
	int sid;
	struct twl_client *twl;
	struct i2c_msg *msg;

	if(mod_no == TWL4030_MODULE_AUDIO_VOICE)
	{

		if(reg == 0x25 && value[1] == 0)
	      {
	      		if(twl4030_get_voicecall_state())
	      		{
	      		     printk("prevent 0x%x -  %x regiter configured to %x\n", mod_no, reg, value[1]);
	      		     return 0;
	      		}
	      }

	      if(reg == 0x26&& value[1] == 0)
	      {
	      		if(twl4030_get_voicecall_state())
	      		{
	      		     printk(" prevent 0x%x -  %x regiter configured to %x\n", mod_no, reg, value[1]);
	      		     return 0;
	      		}
	      }

	      if(reg == 0x01 && value[1] == 0)
	      {
	      		if(twl4030_get_voicecall_state())
	      		{
	      		     printk("prevent 0x%x -  %x regiter configured to %x\n", mod_no, reg, value[1]);
	      		     return 0;
	      		}
	      }

	  	if(reg == 0x1b)
		{
	      		if(twl4030_get_voicecall_state())
	      		{
	      		     printk(" prevent 0x%x -  %x regiter configured to %x\n", mod_no, reg, value[1]);
	      		     return 0;
	      		}
		
	  	}

	 	if(reg == 0x17 && value[1] == 0)
		{
	      		if(twl4030_get_voicecall_state())
	      		{
	      		     printk("prevent 0x%x -  %x regiter configured to %x\n", mod_no, reg, value[1]);
	      		     return 0;
	      		}
		}
	 	if( reg == 0x1c && value[1] == 0)
		{
	      		if(twl4030_get_voicecall_state())
	      		{
	      		     printk("prevent 0x%x -  %x regiter configured to %x\n", mod_no, reg, value[1]);
	      		     return 0;
	      		}
		}
		//printk(" prevent %d -  %x regiter configured to %x\n", mod_no, reg, value[1]);
	}
	else if(mod_no == TWL4030_MODULE_PM_RECEIVER)
	{
		#if 1
		if(reg == 0x45 && value[1] == 0)
	      {
	      		if(twl4030_get_voicecall_state())
	      		{
	      		     printk(" prevent %d -  %x regiter configured to %x\n", mod_no, reg, value[1]);
	      		     return 0;
	      		}
	      }
		#else
		if(twl4030_get_voicecall_state())
		{
			printk("prevent 0x%x -  %x regiter configured to %x\n", mod_no, reg, value[1]);
			return 0;
		}
		#endif
	}
	
	if (unlikely(mod_no > TWL_MODULE_LAST)) {
		pr_err("%s: invalid module number %d\n", DRIVER_NAME, mod_no);
		return -EPERM;
	}
	sid = twl_map[mod_no].sid;
	twl = &twl_modules[sid];

	if (unlikely(!inuse)) {
		pr_err("%s: client %d is not initialized\n", DRIVER_NAME, sid);
		return -EPERM;
	}
	mutex_lock(&twl->xfer_lock);
	/*
	 * [MSG1]: fill the register address data
	 * fill the data Tx buffer
	 */
	msg = &twl->xfer_msg[0];
	msg->addr = twl->address;
	msg->len = num_bytes + 1;
	msg->flags = 0;
	msg->buf = value;
	/* over write the first byte of buffer with the register address */
	*value = twl_map[mod_no].base + reg;
	ret = i2c_transfer(twl->client->adapter, twl->xfer_msg, 1);
	mutex_unlock(&twl->xfer_lock);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 1) {
		pr_err("%s: i2c_write failed to transfer all messages\n",
			DRIVER_NAME);
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}
EXPORT_SYMBOL(twl_i2c_write);
#ifdef T2_REG_DUMP //TI HS.Yoon 20101124 for TWL5025/5030 register full dump

//USB
#define USBVENDOR_ID_LO           0x00
#define USBVENDOR_ID_HI           0x01
#define USBPRODUCT_ID_LO          0x02
#define USBPRODUCT_ID_HI          0x03
#define USBFUNC_CTRL              0x04
#define USBFUNC_CTRL_SET          0x05
#define USBFUNC_CTRL_CLR          0x06
#define USBIFC_CTRL               0x07
#define USBIFC_CTRL_SET           0x08
#define USBIFC_CTRL_CLR           0x09
#define USBOTG_CTRL               0x0A
#define USBOTG_CTRL_SET           0x0B
#define USBOTG_CTRL_CLR           0x0C
#define USB_INT_EN_RISE           0x0D
#define USB_INT_EN_RISE_SET       0x0E
#define USB_INT_EN_RISE_CLR       0x0F
#define USB_INT_EN_FALL           0x10
#define USB_INT_EN_FALL_SET       0x11
#define USB_INT_EN_FALL_CLR       0x12
#define USB_INT_STS               0x13
#define USB_INT_LATCH             0x14
#define USBDEBUG                  0x15
#define USBSCRATCH_REG            0x16
#define USBSCRATCH_REG_SET        0x17
#define USBSCRATCH_REG_CLR        0x18
#define USBCARKIT_CTRL_SET        0x1A
#define USBCARKIT_CTRL            0x19
#define USBCARKIT_CTRL_CLR        0x1B
#define USBCARKIT_INT_DELAY       0x1C
#define USBCARKIT_INT_EN          0x1D
#define USBCARKIT_INT_EN_SET      0x1E
#define USBCARKIT_INT_EN_CLR      0x1F
#define USBCARKIT_INT_STS         0x20
#define USBCARKIT_INT_LATCH       0x21
#define USBCARKIT_PLS_CTRL        0x22
#define USBCARKIT_PLS_CTRL_SET    0x23
#define USBCARKIT_PLS_CTRL_CLR    0x24
#define USBTRANS_POS_WIDTH        0x25
#define USBTRANS_NEG_WIDTH        0x26
#define USBRCV_PLTY_RECOVERY      0x27
#define USBMCPC_CTRL              0x30
#define USBMCPC_CTRL_SET          0x31
#define USBMCPC_CTRL_CLR          0x32
#define USBMCPC_IO_CTRL           0x33
#define USBMCPC_IO_CTRL_SET       0x34
#define USBMCPC_IO_CTRL_CLR       0x35
#define USBMCPC_CTRL2             0x36
#define USBMCPC_CTRL2_SET         0x37
#define USBMCPC_CTRL2_CLR         0x38
#define USBOTHER_FUNC_CTRL        0x80
#define USBOTHER_FUNC_CTRL_SET    0x81
#define USBOTHER_FUNC_CTRL_CLR    0x82
#define USBOTHER_IFC_CTRL         0x83
#define USBOTHER_IFC_CTRL_SET     0x84
#define USBOTHER_IFC_CTRL_CLR     0x85
#define USBOTHER_INT_EN_RISE      0x86
#define USBOTHER_INT_EN_RISE_SET  0x87
#define USBOTHER_INT_EN_RISE_CLR  0x88
#define USBOTHER_INT_EN_FALL      0x89
#define USBOTHER_INT_EN_FALL_SET  0x8A
#define USBOTHER_INT_EN_FALL_CLR  0x8B
#define USBOTHER_INT_STS          0x8C
#define USBOTHER_INT_LATCH        0x8D
#define USBID_INT_EN_RISE         0x8E
#define USBID_INT_EN_RISE_SET     0x8F
#define USBID_INT_EN_RISE_CLR     0x90
#define USBID_INT_EN_FALL         0x91
#define USBID_INT_EN_FALL_SET     0x92
#define USBID_INT_EN_FALL_CLR     0x93
#define USBID_INT_STS             0x94
#define USBID_INT_LATCH           0x95
#define USBID_STATUS              0x96
#define USBCARKIT_SM_1_INT_EN     0x97
#define USBCARKIT_SM_1_INT_EN_SET 0x98
#define USBCARKIT_SM_1_INT_EN_CLR 0x99
#define USBCARKIT_SM_1_INT_STS    0x9A
#define USBCARKIT_SM_1_INT_LATCH  0x9B
#define USBCARKIT_SM_2_INT_EN     0x9C
#define USBCARKIT_SM_2_INT_EN_SET 0x9D
#define USBCARKIT_SM_2_INT_EN_CLR 0x9E
#define USBCARKIT_SM_2_INT_STS    0x9F
#define USBCARKIT_SM_2_INT_LATCH  0xA0
#define USBCARKIT_SM_CTRL         0xA1
#define USBCARKIT_SM_CTRL_SET     0xA2
#define USBCARKIT_SM_CTRL_CLR     0xA3
#define USBCARKIT_SM_CMD          0xA4
#define USBCARKIT_SM_CMD_SET      0xA5
#define USBCARKIT_SM_CMD_CLR      0xA6
#define USBCARKIT_SM_CMD_STS      0xA7
#define USBCARKIT_SM_STATUS       0xA8
#define USBCARKIT_SM_NEXT_STATUS  0xA9
#define USBCARKIT_SM_ERR_STATUS   0xAA
#define USBCARKIT_SM_CTRL_STATE   0xAB
#define USBPOWER_CTRL             0xAC
#define USBPOWER_CTRL_SET         0xAD
#define USBPOWER_CTRL_CLR         0xAE
#define USBOTHER_IFC_CTRL2        0xAF
#define USBOTHER_IFC_CTRL2_SET    0xB0
#define USBOTHER_IFC_CTRL2_CLR    0xB1
#define USBREG_CTRL_EN            0xB2
#define USBREG_CTRL_EN_SET        0xB3
#define USBREG_CTRL_EN_CLR        0xB4
#define USBREG_CTRL_ERROR         0xB5
#define USBOTHER_FUNC_CTRL2       0xB8
#define USBOTHER_FUNC_CTRL2_SET   0xB9
#define USBOTHER_FUNC_CTRL2_CLR   0xBA
#define USBCARKIT_ANA_CTRL        0xBB
#define USBCARKIT_ANA_CTRL_SET    0xBC
#define USBCARKIT_ANA_CTRL_CLR    0xBD
#define USBVBUS_DEBOUNCE          0xC0
#define USBID_DEBOUNCE            0xC1
#define USBTPH_DP_CON_MIN         0xC2
#define USBTPH_DP_CON_MAX         0xC3
#define USBTCR_DP_CON_MIN         0xC4
#define USBTCR_DP_CON_MAX         0xC5
#define USBTPH_DP_PD_SHORT        0xC6
#define USBTPH_CMD_DLY            0xC7
#define USBTPH_DET_RST            0xC8
#define USBTPH_AUD_BIAS           0xC9
#define USBTCR_UART_DET_MIN       0xCA
#define USBTCR_UART_DET_MAX       0xCB
#define USBTPH_ID_INT_PW          0xCD
#define USBTACC_ID_INT_WAIT       0xCE
#define USBTACC_ID_INT_PW         0xCF
#define USBTPH_CMD_WAIT           0xD0
#define USBTPH_ACK_WAIT           0xD1
#define USBTPH_DP_DISC_DET        0xD2
#define USBVBAT_TIMER             0xD3
#define USBCARKIT_4W_DEBUG        0xE0
#define USBCARKIT_5W_DEBUG        0xE1
#define USBTEST_CTRL_SET          0xEA
#define USBTEST_CTRL_CLR          0xEB
#define USBTEST_CARKIT_SET        0xEC
#define USBTEST_CARKIT_CLEAR      0xED
#define USBTEST_POWER_SET         0xEE
#define USBTEST_POWER_CLR         0xEF
#define USBTEST_ULPI              0xF0
#define USBTXVR_EN_TEST_SET       0xF2
#define USBTXVR_EN_TEST_CLR       0xF3
#define USBVBUS_EN_TEST           0xF4
#define USBID_EN_TEST             0xF5
#define USBPSM_EN_TEST_SET        0xF6
#define USBPSM_EN_TEST_CLR        0xF7
#define USBPHY_TRIM_CTRL          0xFC
#define USBPHY_PWR_CTRL           0xFD
#define USBPHY_CLK_CTRL           0xFE
#define USBPHY_CLK_CTRL_STS       0xFF

#define AudioCODEC_MODE           0x01
#define AudioOPTION               0x02
#define AudioMICBIAS_CTL          0x04
#define AudioANAMICL              0x05
#define AudioANAMICR              0x06
#define AudioAVADC_CTL            0x07
#define AudioADCMICSEL            0x08
#define AudioDIGMIXING            0x09
#define AudioATXL1PGA             0x0A
#define AudioATXR1PGA             0x0B
#define AudioAVTXL2PGA            0x0C
#define AudioAVTXR2PGA            0x0D
#define AudioAUDIO_IF             0x0E
#define AudioVOICE_IF             0x0F
#define AudioARXR1PGA             0x10
#define AudioARXL1PGA             0x11
#define AudioARXR2PGA             0x12
#define AudioARXL2PGA             0x13
#define AudioVRXPGA               0x14
#define AudioVSTPGA               0x15
#define AudioVRX2ARXPGA           0x16
#define AudioAVDAC_CTL            0x17
#define AudioARX2VTXPGA           0x18
#define AudioARXL1_APGA_CTL       0x19
#define AudioARXR1_APGA_CTL       0x1A
#define AudioARXL2_APGA_CTL       0x1B
#define AudioARXR2_APGA_CTL       0x1C
#define AudioATX2ARXPGA           0x1D
#define AudioBT_IF                0x1E
#define AudioBTPGA                0x1F
#define AudioBTSTPGA              0x20
#define AudioEAR_CTL              0x21
#define AudioHS_SEL               0x22
#define AudioHS_GAIN_SET          0x23
#define AudioHS_POPN_SET          0x24
#define AudioPREDL_CTL            0x25
#define AudioPREDR_CTL            0x26
#define AudioPRECKL_CTL           0x27
#define AudioPRECKR_CTL           0x28
#define AudioHFL_CTL              0x29
#define AudioHFR_CTL              0x2A
#define AudioALC_CTL              0x2B
#define AudioALC_SET1             0x2C
#define AudioALC_SET2             0x2D
#define AudioBOOST_CTL            0x2E
#define AudioSOFTVOL_CTL          0x2F
#define AudioDTMF_FREQSEL         0x30
#define AudioDTMF_TONEXT1H        0x31
#define AudioDTMF_TONEXT1L        0x32
#define AudioDTMF_TONEXT2H        0x33
#define AudioDTMF_TONEXT2L        0x34
#define AudioDTMF_TONOFF          0x35
#define AudioDTMF_WANONOFF        0x36
#define AudioI2S_RX_SCRAMBLE_H    0x37
#define AudioI2S_RX_SCRAMBLE_M    0x38
#define AudioI2S_RX_SCRAMBLE_L    0x39
#define AudioAPLL_CTL             0x3A
#define AudioDTMF_CTL             0x3B
#define AudioDTMF_PGA_CTL2        0x3C
#define AudioDTMF_PGA_CTL1        0x3D
#define AudioMISC_SET_1           0x3E
#define AudioPCMBTMUX             0x3F
#define AudioRX_PATH_SEL          0x43
#define AudioVDL_APGA_CTL         0x44
#define AudioVIBRA_CTL            0x45
#define AudioVIBRA_SET            0x46
#define AudioVIBRA_PWM_SET        0x47
#define AudioANAMIC_GAIN          0x48
#define AudioMISC_SET_2           0x49

//TEST
#define TESTAUDIO_TEST_CTL        0x4C
#define TESTINT_TEST_CTL          0x4D
#define TESTDAC_ADC_TEST_CTL      0x4E
#define TESTRXTX_TRIM_IB          0x4F
#define TESTCLD_CONTROL           0x50
#define TESTCLD_MODE_TIMING       0x51
#define TESTCLD_TRIM_RAMP         0x52
#define TESTCLD_TESTV_CTL         0x53
#define TESTAPLL_TEST_CTL         0x54
#define TESTAPLL_TEST_DIV         0x55
#define TESTAPLL_TEST_CTL2        0x56
#define TESTAPLL_TEST_CUR         0x57
#define TESTDIGMIC_BIAS1_CTL      0x58
#define TESTDIGMIC_BIAS2_CTL      0x59
#define TESTRX_OFFSET_VOICE       0x5A
#define TESTRX_OFFSET_AL1         0x5B
#define TESTRX_OFFSET_AR1         0x5C
#define TESTRX_OFFSET_AL2         0x5D
#define TESTRX_OFFSET_AR2         0x5E
#define TESTOFFSET1               0x5F
#define TESTOFFSET2               0x60

//Primary Interrupt Handler Registers
#define PIH_ISR1                  0x81
#define PIH_ISR2                  0x82
#define PIH_SIR                   0x83

//INTBR
#define INTBRIDCODE_7_0           0x85
#define INTBRIDCODE_15_8          0x86
#define INTBRIDCODE_23_16         0x87
#define INTBRIDCODE_31_24         0x88
#define INTBRDIEID_7_0            0x89
#define INTBRDIEID_15_8           0x8A
#define INTBRDIEID_23_16          0x8B
#define INTBRDIEID_31_24          0x8C
#define INTBRDIEID_39_32          0x8D
#define INTBRDIEID_47_40          0x8E
#define INTBRDIEID_55_48          0x8F
#define INTBRDIEID_63_56          0x90
#define INTBRGPBR1                0x91
#define INTBRPMBR1                0x92
#define INTBRPMBR2                0x93
#define INTBRGPPUPDCTR1           0x94
#define INTBRGPPUPDCTR2           0x95
#define INTBRGPPUPDCTR3           0x96
#define INTBRUNLOCK_TEST_REG      0x97

//GPIO
#define GPIODATAIN1               0x98
#define GPIODATAIN2               0x99
#define GPIODATAIN3               0x9A
#define GPIODATADIR1              0x9B
#define GPIODATADIR2              0x9C
#define GPIODATADIR3              0x9D
#define GPIODATAOUT1              0x9E
#define GPIODATAOUT2              0x9F
#define GPIODATAOUT3              0xA0
#define GPIOCLEARDATAOUT1         0xA1
#define GPIOCLEARDATAOUT2         0xA2
#define GPIOCLEARDATAOUT3         0xA3
#define GPIOSETDATAOUT1           0xA4
#define GPIOSETDATAOUT2           0xA5
#define GPIOSETDATAOUT3           0xA6
#define GPIO_DEBEN1               0xA7
#define GPIO_DEBEN2               0xA8
#define GPIO_DEBEN3               0xA9
#define GPIO_CTRL                 0xAA
#define GPIOPUPDCTR1              0xAB
#define GPIOPUPDCTR2              0xAC
#define GPIOPUPDCTR3              0xAD
#define GPIOPUPDCTR4              0xAE
#define GPIOPUPDCTR5              0xAF
#define GPIO_TEST                 0xB0
#define GPIO_ISR1A                0xB1
#define GPIO_ISR2A                0xB2
#define GPIO_IMR1A                0xB4
#define GPIO_IMR2A                0xB5
#define GPIO_IMR3A                0xB6
#define GPIO_ISR1B                0xB7
#define GPIO_ISR2B                0xB8
#define GPIO_ISR3B                0xB9
#define GPIO_IMR1B                0xBA
#define GPIO_IMR2B                0xBB
#define GPIO_IMR3B                0xBC
#define GPIO_SIR1                 0xBD
#define GPIO_SIR2                 0xBE
#define GPIO_SIR3                 0xBF
#define GPIO_EDR1                 0xC0
#define GPIO_EDR2                 0xC1
#define GPIO_EDR3                 0xC2
#define GPIO_EDR4                 0xC3
#define GPIO_EDR5                 0xC4
#define GPIO_SIH_CTRL             0xC5

//MADC
#define MADCCTRL1                 0x00
#define MADCCTRL2                 0x01
#define MADCRTSELECT_LSB          0x02
#define MADCRTSELECT_MSB          0x03
#define MADCRTAVERAGE_LSB         0x04
#define MADCRTAVERAGE_MSB         0x05
#define MADCSW1SELECT_LSB         0x06
#define MADCSW1SELECT_MSB         0x07
#define MADCSW1AVERAGE_LSB        0x08
#define MADCSW1AVERAGE_MSB        0x09
#define MADCSW2SELECT_LSB         0x0A
#define MADCSW2SELECT_MSB         0x0B
#define MADCSW2AVERAGE_LSB        0x0C
#define MADCSW2AVERAGE_MSB        0x0D
#define MADCBCI_USBAVERAGE        0x0E
#define MADCACQUISITION           0x0F
#define MADCUSBREF_LSB            0x10
#define MADCUSBREF_MSB            0x11
#define MADCCTRL_SW1              0x12
#define MADCCTRL_SW2              0x13
#define MADCMADC_TEST             0x14
#define MADCGP_MADC_TEST1         0x15
#define MADCGP_MADC_TEST2         0x16
#define MADCRTCH0_LSB             0x17
#define MADCRTCH0_MSB             0x18
#define MADCRTCH1_LSB             0x19
#define MADCRTCH1_MSB             0x1A
#define MADCRTCH2_LSB             0x1B
#define MADCRTCH2_MSB             0x1C
#define MADCRTCH3_LSB             0x1D
#define MADCRTCH3_MSB             0x1E
#define MADCRTCH4_LSB             0x1F
#define MADCRTCH4_MSB             0x20
#define MADCRTCH5_LSB             0x21
#define MADCRTCH5_MSB             0x22
#define MADCRTCH6_LSB             0x23
#define MADCRTCH6_MSB             0x24
#define MADCRTCH7_LSB             0x25
#define MADCRTCH7_MSB             0x26
#define MADCRTCH8_LSB             0x27
#define MADCRTCH8_MSB             0x28
#define MADCRTCH9_LSB             0x29
#define MADCRTCH9_MSB             0x2A
#define MADCRTCH10_LSB            0x2B
#define MADCRTCH10_MSB            0x2C
#define MADCRTCH11_LSB            0x2D
#define MADCRTCH11_MSB            0x2E
#define MADCRTCH12_LSB            0x2F
#define MADCRTCH12_MSB            0x30
#define MADCRTCH13_LSB            0x31
#define MADCRTCH13_MSB            0x32
#define MADCRTCH14_LSB            0x33
#define MADCRTCH14_MSB            0x34
#define MADCRTCH15_LSB            0x35
#define MADCRTCH15_MSB            0x36
#define MADCGPCH0_LSB             0x37
#define MADCGPCH0_MSB             0x38
#define MADCGPCH1_LSB             0x39
#define MADCGPCH1_MSB             0x3A
#define MADCGPCH2_LSB             0x3B
#define MADCGPCH2_MSB             0x3C
#define MADCGPCH3_LSB             0x3D
#define MADCGPCH3_MSB             0x3E
#define MADCGPCH4_LSB             0x3F
#define MADCGPCH4_MSB             0x40
#define MADCGPCH5_LSB             0x41
#define MADCGPCH5_MSB             0x42
#define MADCGPCH6_LSB             0x43
#define MADCGPCH6_MSB             0x44
#define MADCGPCH7_LSB             0x45
#define MADCGPCH7_MSB             0x46
#define MADCGPCH8_LSB             0x47
#define MADCGPCH8_MSB             0x48
#define MADCGPCH9_LSB             0x49
#define MADCGPCH9_MSB             0x4A
#define MADCGPCH10_LSB            0x4B
#define MADCGPCH10_MSB            0x4C
#define MADCGPCH11_LSB            0x4D
#define MADCGPCH11_MSB            0x4E
#define MADCGPCH12_LSB            0x4F
#define MADCGPCH12_MSB            0x50
#define MADCGPCH13_LSB            0x51
#define MADCGPCH13_MSB            0x52
#define MADCGPCH14_LSB            0x53
#define MADCGPCH14_MSB            0x54
#define MADCGPCH15_LSB            0x55
#define MADCGPCH15_MSB            0x56
#define MADCBCICH0_LSB            0x57
#define MADCBCICH0_MSB            0x58
#define MADCBCICH1_LSB            0x59
#define MADCBCICH1_MSB            0x5A
#define MADCBCICH2_LSB            0x5B
#define MADCBCICH2_MSB            0x5C
#define MADCBCICH3_LSB            0x5D
#define MADCBCICH3_MSB            0x5E
#define MADCBCICH4_LSB            0x5F
#define MADCBCICH4_MSB            0x60
#define MADC_ISR1                 0x61
#define MADC_IMR1                 0x62
#define MADC_ISR2                 0x63
#define MADC_IMR2                 0x64
#define MADC_SIR                  0x65
#define MADC_EDR                  0x66
#define MADC_SIH_CTRL             0x67

// BCI Interrupt Registers

#define BciISR1A                  0xB9
#define BciISR2A                  0xBA
#define BciIMR1A                  0xBB
#define BciIMR2A                  0xBC
#define BciISR1B                  0xBD
#define BciISR2B                  0xBE
#define BciIMR1B                  0xBF
#define BciIMR2B                  0xC0
#define BciSIR1                   0xC1
#define BciSIR2                   0xC2
#define BciEDR1                   0xC3
#define BciEDR2                   0xC4
#define BciEDR3                   0xC5
#define BciSIHCtrl                0xC6

// Main Charge
#define BCIMDEN                   0x74
#define BCIMDKEY                  0x75
#define BCIMSTATEC                0x76
#define BCIMSTATEP                0x77
#define BCIVBAT1                  0x78
#define BCIVBAT2                  0x79
#define BCITBAT1                  0x7A
#define BCITBAT2                  0x7B
#define BCIICHG1                  0x7C
#define BCIICHG2                  0x7D
#define BCIVAC1                   0x7E
#define BCIVAC2                   0x7F
#define BCIVBUS1                  0x80
#define BCIVBUS2                  0x81
#define BCIMFSTS2                 0x82
#define BCIMFSTS3                 0x83
#define BCIMFSTS4                 0x84
#define BCIMFKEY                  0x85
#define BCIMFEN1                  0x86
#define BCIMFEN2                  0x87
#define BCIMFEN3                  0x88
#define BCIMFEN4                  0x89
#define BCIMFTH1                  0x8A
#define BCIMFTH2                  0x8B
#define BCIMFTH3                  0x8C
#define BCIMFTH4                  0x8D
#define BCIMFTH5                  0x8E
#define BCIMFTH6                  0x8F
#define BCIMFTH7                  0x90
#define BCIMFTH8                  0x91
#define BCIMFTH9                  0x92
#define BCITIMER1                 0x93
#define BCITIMER2                 0x94
#define BCIWDKEY                  0x95
#define BCIWD                     0x96
#define BCICTL1                   0x97

#define BCICTL2                   0x98
#define BCIVREF1                  0x99
#define BCIVREF2                  0x9A
#define BCIIREF1                  0x9B
#define BCIIREF2                  0x9C
#define BCIPWM2                   0x9D
#define BCIPWM1                   0x9E
#define BCITRIM1                  0x9F
#define BCITRIM2                  0xA0
#define BCITRIM3                  0xA1
#define BCITRIM4                  0xA2
#define BCIVREFCOMB1              0xA3
#define BCIVREFCOMB2              0xA4
#define BCIIREFCOMB1              0xA5
#define BCIIREFCOMB2              0xA6
#define BCIMNTEST1                0xA7
#define BCIMNTEST2                0xA8
#define BCIMNTEST3                0xA9

//Precharge
#define BCIPSTATE                 0xAA
#define BCIMFSTS1                 0xAB
#define BCITRIM5                  0xAC

//Keypad Registers
#define KeyPCtrl                  0xD2
#define KeyPDebounce              0xD3
#define KeyPLongKey               0xD4
#define KeyPLkPtv                 0xD5
#define KeyPTimeout1              0xD6
#define KeyPTimeout2              0xD7
#define KeyPKBC                   0xD8
#define KeyPKBR                   0xD9
#define KeyPSMS                   0xDA
#define KeyP_7_0                  0xDB
#define KeyP_15_8                 0xDC
#define KeyP_23_16                0xDD
#define KeyP_31_24                0xDE
#define KeyP_39_32                0xDF
#define KeyP_47_40                0xE0
#define KeyP_55_48                0xE1
#define KeyP_63_56                0xE2
#define KeyPISR1                  0xE3
#define KeyPIMR1                  0xE4
#define KeyPISR2                  0xE5
#define KeyPIMR2                  0xE6
#define KeyPSIR                   0xE7
#define KeyPEDR                   0xE8
#define KeyPSIHCtrl               0xE9

//LED ENABLE
#define LEDEN                     0xEE
//PWMA
#define PWMAON                    0xEF
#define PWMAOFF                   0xF0
//PWMB
#define PWMBON                    0xF1
#define PWMBOFF                   0xF2
//PWM0
#define PWM0ON                    0xF8
#define PWM0OFF                   0xF9
//PWM1
#define PWM1ON                    0xFB
#define PWM1OFF                   0xFC

//Secure register
#define SECURED_REG_A             0x00
#define SECURED_REG_B             0x01
#define SECURED_REG_C             0x02
#define SECURED_REG_D             0x03
#define SECURED_REG_E             0x04
#define SECURED_REG_F             0x05
#define SECURED_REG_G             0x06
#define SECURED_REG_H             0x07
#define SECURED_REG_I             0x08
#define SECURED_REG_J             0x09
#define SECURED_REG_K             0x0A
#define SECURED_REG_L             0x0B
#define SECURED_REG_M             0x0C
#define SECURED_REG_N             0x0D
#define SECURED_REG_O             0x0E
#define SECURED_REG_P             0x0F
#define SECURED_REG_Q             0x10
#define SECURED_REG_R             0x11
#define SECURED_REG_S             0x12
#define SECURED_REG_U             0x13

//Backup reg
#define BACKUP_REG_A              0x14
#define BACKUP_REG_B              0x15
#define BACKUP_REG_C              0x16
#define BACKUP_REG_D              0x17
#define BACKUP_REG_E              0x18
#define BACKUP_REG_F              0x19
#define BACKUP_REG_G              0x1A
#define BACKUP_REG_H              0x1B

//RTC
#define RTCSECONDS_REG            0x1C
#define RTCMINUTES_REG            0x1D
#define RTCHOURS_REG              0x1E
#define RTCDAYS_REG               0x1F
#define RTCMONTHS_REG             0x20
#define RTCYEARS_REG              0x21
#define RTCWEEKS_REG              0x22
#define RTCALARM_SECONDS_REG      0x23
#define RTCALARM_MINUTES_REG      0x24
#define RTCALARM_HOURS_REG        0x25
#define RTCALARM_DAYS_REG         0x26
#define RTCALARM_MONTHS_REG       0x27
#define RTCALARM_YEARS_REG        0x28
#define RTC_CTRL_REG              0x29
#define RTC_STATUS_REG            0x2A
#define RTC_INTERRUPTS_REG        0x2B
#define RTC_COMP_LSB_REG          0x2C
#define RTC_COMP_MSB_REG          0x2D

//PM Master : Int Power
#define PWR_ISR1                  0x2E
#define PWR_IMR1                  0x2F
#define PWR_ISR2                  0x30
#define PWR_IMR2                  0x31
#define PWR_SIR                   0x32
#define PWR_EDR1                  0x33
#define PWR_EDR2                  0x34
#define PWR_SIH_CTRL              0x35

//PM Master :
#define PWRCFG_P1_TRANSITION      0x36
#define PWRCFG_P2_TRANSITION      0x37
#define PWRCFG_P3_TRANSITION      0x38
#define PWRCFG_P123_TRANSITION    0x39
#define PWRSTS_BOOT               0x3A
#define PWRCFG_BOOT               0x3B
#define PWRSHUNDAN                0x3C
#define PWRBOOT_BCI               0x3D
#define PWRCFG_PWRANA1            0x3E
#define PWRCFG_PWRANA2            0x3F
#define PWRBGAP_TRIM              0x40
#define PWRBACKUP_MISC_STS        0x41
#define PWRBACKUP_MISC_CFG        0x42
#define PWRBACKUP_MISC_TST        0x43
#define PWRPROTECT_KEY            0x44
#define PWRSTS_HW_CONDITIONS      0x45
#define PWRP1_SW_EVENTS           0x46
#define PWRP2_SW_EVENTS           0x47
#define PWRP3_SW_EVENTS           0x48
#define PWRSTS_P123_STATE         0x49
#define PWRPB_CFG                 0x4A
#define PWRPB_WORD_MSB            0x4B
#define PWRPB_WORD_LSB            0x4C
#define PWRRESERVED_A             0x4D
#define PWRRESERVED_B             0x4E
#define PWRRESERVED_C             0x4F
#define PWRRESERVED_D             0x50
#define PWRRESERVED_E             0x51
#define PWRSEQ_ADD_W2P            0x52
#define PWRSEQ_ADD_P2A            0x53
#define PWRSEQ_ADD_A2W            0x54
#define PWRSEQ_ADD_A2S            0x55
#define PWRSEQ_ADD_S2A12          0x56
#define PWRSEQ_ADD_S2A3           0x57
#define PWRSEQ_ADD_WARM           0x58
#define PWRMEMORY_ADDRESS         0x59
#define PWRMEMORY_DATA            0x5A

//PM Reciever : Secure mode
#define PWRSC_CONFIG              0x5B
#define PWRSC_DETECT1             0x5C
#define PWRSC_DETECT2             0x5D
#define PWRWATCHDOG_CFG           0x5E
#define PWRIT_CHECK_CFG           0x5F
#define PWRVIBRATOR_CFG           0x60
#define PWRDCDC_GLOBAL_CFG        0x61

//TRIM
#define PWRVDD1_TRIM1             0x62
#define PWRVDD1_TRIM2             0x63
#define PWRVDD2_TRIM1             0x64
#define PWRVDD2_TRIM2             0x65
#define PWRVIO_TRIM1              0x66
#define PWRVIO_TRIM2              0x67

//MISC
#define PWRMISC_CFG               0x68

//TEST
#define PWRLS_TST_A               0x69
#define PWRLS_TST_B               0x6A
#define PWRLS_TST_C               0x6B
#define PWRLS_TST_D               0x6C
#define PWRBB_CFG                 0x6D
#define PWRMISC_TST               0x6E
#define PWRTRIM1                  0x6F
#define PWRTRIM2                  0x70
#define PWRDCDC_TIMEOUT           0x71

//AUX1
#define PWRVAUX1_DEV_GRP          0x72
#define PWRVAUX1_TYPE             0x73
#define PWRVAUX1_REMAP            0x74
#define PWRVAUX1_DEDICATED        0x75
//AUX2
#define PWRVAUX2_DEV_GRP          0x76
#define PWRVAUX2_TYPE             0x77
#define PWRVAUX2_REMAP            0x78
#define PWRVAUX2_DEDICATED        0x79
//AUX3
#define PWRVAUX3_DEV_GRP          0x7A
#define PWRVAUX3_TYPE             0x7B
#define PWRVAUX3_REMAP            0x7C
#define PWRVAUX3_DEDICATED        0x7D
//AUX4
#define PWRVAUX4_DEV_GRP          0x7E
#define PWRVAUX4_TYPE             0x7F
#define PWRVAUX4_REMAP            0x80
#define PWRVAUX4_DEDICATED        0x81
//MMC1
#define PWRVMMC1_DEV_GRP          0x82
#define PWRVMMC1_TYPE             0x83
#define PWRVMMC1_REMAP            0x84
#define PWRVMMC1_DEDICATED        0x85
//MMC2
#define PWRVMMC2_DEV_GRP          0x86
#define PWRVMMC2_TYPE             0x87
#define PWRVMMC2_REMAP            0x88
#define PWRVMMC2_DEDICATED        0x89
//PLL1
#define PWRVPLL1_DEV_GRP          0x8A
#define PWRVPLL1_TYPE             0x8B
#define PWRVPLL1_REMAP            0x8C
#define PWRVPLL1_DEDICATED        0x8D
//PLL2
#define PWRVPLL2_DEV_GRP          0x8E
#define PWRVPLL2_TYPE             0x8F
#define PWRVPLL2_REMAP            0x90
#define PWRVPLL2_DEDICATED        0x91
//SIM
#define PWRVSIM_DEV_GRP           0x92
#define PWRVSIM_TYPE              0x93
#define PWRVSIM_REMAP             0x94
#define PWRVSIM_DEDICATED         0x95
//DAC
#define PWRVDAC_DEV_GRP           0x96
#define PWRVDAC_TYPE              0x97
#define PWRVDAC_REMAP             0x98
#define PWRVDAC_DEDICATED         0x99
//INTANA1
#define PWRVINTANA1_DEV_GRP       0x9A
#define PWRVINTANA1_TYPE          0x9B
#define PWRVINTANA1_REMAP         0x9C
#define PWRVINTANA1_DEDICATED     0x9D
//INTANA2
#define PWRVINTANA2_DEV_GRP       0x9E
#define PWRVINTANA2_TYPE          0x9F
#define PWRVINTANA2_REMAP         0xA0
#define PWRVINTANA2_DEDICATED     0xA1
//INTDIG
#define PWRVINTDIG_DEV_GRP        0xA2
#define PWRVINTDIG_TYPE           0xA3
#define PWRVINTDIG_REMAP          0xA4
#define PWRVINTDIG_DEDICATED      0xA5
//VIO
#define PWRVIO_DEV_GRP            0xA6
#define PWRVIO_TYPE               0xA7
#define PWRVIO_REMAP              0xA8
#define PWRVIO_CFG                0xA9
#define PWRVIO_MISC_CFG           0xAA
#define PWRVIO_TEST1              0xAB
#define PWRVIO_TEST2              0xAC
#define PWRVIO_OSC                0xAD
#define PWRVIO_RESERVED           0xAE
#define PWRVIO_VSEL               0xAF
//VDD1 CFG
#define PWRVDD1_DEV_GRP           0xB0
#define PWRVDD1_TYPE              0xB1
#define PWRVDD1_REMAP             0xB2
#define PWRVDD1_CFG               0xB3
#define PWRVDD1_MISC_CFG          0xB4
#define PWRVDD1_TEST1             0xB5
#define PWRVDD1_TEST2             0xB6
#define PWRVDD1_OSC               0xB7
#define PWRVDD1_RESERVED          0xB8
#define PWRVDD1_VSEL              0xB9
#define PWRVDD1_VMODE_CFG         0xBA
#define PWRVDD1_VFLOOR            0xBB
#define PWRVDD1_VROOF             0xBC
#define PWRVDD1_STEP              0xBD
//VDD2 CFG
#define PWRVDD2_DEV_GRP           0xBE
#define PWRVDD2_TYPE              0xBF
#define PWRVDD2_REMAP             0xC0
#define PWRVDD2_CFG               0xC1
#define PWRVDD2_MISC_CFG          0xC2
#define PWRVDD2_TEST1             0xC3
#define PWRVDD2_TEST2             0xC4
#define PWRVDD2_OSC               0xC5
#define PWRVDD2_RESERVED          0xC6
#define PWRVDD2_VSEL              0xC7
#define PWRVDD2_VMODE_CFG         0xC8
#define PWRVDD2_VFLOOR            0xC9
#define PWRVDD2_VROOF             0xCA
#define PWRVDD2_STEP              0xCB
//VUSB1.5
#define PWRVUSB1V5_DEV_GRP        0xCC
#define PWRVUSB1V5_TYPE           0xCD
#define PWRVUSB1V5_REMAP          0xCE
//VUSB1.8
#define PWRVUSB1V8_DEV_GRP        0xCF
#define PWRVUSB1V8_TYPE           0xD0
#define PWRVUSB1V8_REMAP          0xD1
//VUSB3.0
#define PWRVUSB3V1_DEV_GRP        0xD2
#define PWRVUSB3V1_TYPE           0xD3
#define PWRVUSB3V1_REMAP          0xD4
//VUSBCP
#define PWRVUSBCP_DEV_GRP         0xD5
#define PWRVUSBCP_TYPE            0xD6
#define PWRVUSBCP_REMAP           0xD7
#define PWRVUSB_DEDICATED1        0xD8
#define PWRVUSB_DEDICATED2        0xD9
//VREGEN
#define PWRREGEN_DEV_GRP          0xDA
#define PWRREGEN_TYPE             0xDB
#define PWRREGEN_REMAP            0xDC
//VRESPRON
#define PWRNRESPWRON_DEV_GRP      0xDD
#define PWRNRESPWRON_TYPE         0xDE
#define PWRNRESPWRON_REMAP        0xDF
//VCLKEN
#define PWRCLKEN_DEV_GRP          0xE0
#define PWRCLKEN_TYPE             0xE1
#define PWRCLKEN_REMAP            0xE2
//VSYSEN
#define PWRSYSEN_DEV_GRP          0xE3
#define PWRSYSEN_TYPE             0xE4
#define PWRSYSEN_REMAP            0xE5
//VHFCLKOUT
#define PWRHFCLKOUT_DEV_GRP       0xE6
#define PWRHFCLKOUT_TYPE          0xE7
#define PWRHFCLKOUT_REMAP         0xE8
//V32KCLKOUT
#define PWR32KCLKOUT_DEV_GRP      0xE9
#define PWR32KCLKOUT_TYPE         0xEA
#define PWR32KCLKOUT_REMAP        0xEB
//VT2RESET
#define PWRTRITON_RESET_DEV_GRP   0xEC
#define PWRTRITON_RESET_TYPE      0xED
#define PWRTRITON_RESET_REMAP     0xEE
//VT2MAINREF
#define PWRMAINREF_DEV_GRP        0xEF
#define PWRMAINREF_TYPE           0xF0
#define PWRMAINREF_REMAP          0xF1

//SMART REFLEX
#define SMRVDD1_SR_CONTROL        0x00
#define SMRVDD2_SR_CONTROL        0x01

struct __t2reg{
	u8 dev_id;
	u8 reg;
	char regn[35];
	};

struct __t2reg t2_reg_full_dump_list[758]= {
	{ 0x48, USBVENDOR_ID_LO,"USBVENDOR_ID_LO"},
	{ 0x48, USBVENDOR_ID_HI,"USBVENDOR_ID_HI"},
	{ 0x48, USBPRODUCT_ID_LO,"USBPRODUCT_ID_LO"},
	{ 0x48, USBPRODUCT_ID_HI,"USBPRODUCT_ID_HI"},
	{ 0x48, USBFUNC_CTRL,"USBFUNC_CTRL"},
	{ 0x48, USBFUNC_CTRL_SET,"USBFUNC_CTRL_SET"},
	{ 0x48, USBFUNC_CTRL_CLR,"USBFUNC_CTRL_CLR"},
	{ 0x48, USBIFC_CTRL,"USBIFC_CTRL"},
	{ 0x48, USBIFC_CTRL_SET,"USBIFC_CTRL_SET"},
	{ 0x48, USBIFC_CTRL_CLR,"USBIFC_CTRL_CLR"},
	{ 0x48, USBOTG_CTRL,"USBOTG_CTRL"},
	{ 0x48, USBOTG_CTRL_SET,"USBOTG_CTRL_SET"},
	{ 0x48, USBOTG_CTRL_CLR,"USBOTG_CTRL_CLR"},
	{ 0x48, USB_INT_EN_RISE,"USB_INT_EN_RISE"},
	{ 0x48, USB_INT_EN_RISE_SET,"USB_INT_EN_RISE_SET"},
	{ 0x48, USB_INT_EN_RISE_CLR,"USB_INT_EN_RISE_CLR"},
	{ 0x48, USB_INT_EN_FALL,"USB_INT_EN_FALL"},
	{ 0x48, USB_INT_EN_FALL_SET,"USB_INT_EN_FALL_SET"},
	{ 0x48, USB_INT_EN_FALL_CLR,"USB_INT_EN_FALL_CLR"},
	{ 0x48, USB_INT_STS,"USB_INT_STS"},
	{ 0x48, USB_INT_LATCH,"USB_INT_LATCH"},
	{ 0x48, USBDEBUG,"USBDEBUG"},
	{ 0x48, USBSCRATCH_REG,"USBSCRATCH_REG"},
	{ 0x48, USBSCRATCH_REG_SET,"USBSCRATCH_REG_SET"},
	{ 0x48, USBSCRATCH_REG_CLR,"USBSCRATCH_REG_CLR"},
	{ 0x48, USBCARKIT_CTRL_SET,"USBCARKIT_CTRL_SET"},
	{ 0x48, USBCARKIT_CTRL,"USBCARKIT_CTRL"},
	{ 0x48, USBCARKIT_CTRL_CLR,"USBCARKIT_CTRL_CLR"},
	{ 0x48, USBCARKIT_INT_DELAY,"USBCARKIT_INT_DELAY"},
	{ 0x48, USBCARKIT_INT_EN,"USBCARKIT_INT_EN"},
	{ 0x48, USBCARKIT_INT_EN_SET,"USBCARKIT_INT_EN_SET"},
	{ 0x48, USBCARKIT_INT_EN_CLR,"USBCARKIT_INT_EN_CLR"},
	{ 0x48, USBCARKIT_INT_STS,"USBCARKIT_INT_STS"},
	{ 0x48, USBCARKIT_INT_LATCH,"USBCARKIT_INT_LATCH"},
	{ 0x48, USBCARKIT_PLS_CTRL,"USBCARKIT_PLS_CTRL"},
	{ 0x48, USBCARKIT_PLS_CTRL_SET,"USBCARKIT_PLS_CTRL_SET"},
	{ 0x48, USBCARKIT_PLS_CTRL_CLR,"USBCARKIT_PLS_CTRL_CLR"},
	{ 0x48, USBTRANS_POS_WIDTH,"USBTRANS_POS_WIDTH"},
	{ 0x48, USBTRANS_NEG_WIDTH,"USBTRANS_NEG_WIDTH"},
	{ 0x48, USBRCV_PLTY_RECOVERY,"USBRCV_PLTY_RECOVERY"},
	{ 0x48, USBMCPC_CTRL,"USBMCPC_CTRL"},
	{ 0x48, USBMCPC_CTRL_SET,"USBMCPC_CTRL_SET"},
	{ 0x48, USBMCPC_CTRL_CLR,"USBMCPC_CTRL_CLR"},
	{ 0x48, USBMCPC_IO_CTRL,"USBMCPC_IO_CTRL"},
	{ 0x48, USBMCPC_IO_CTRL_SET,"USBMCPC_IO_CTRL_SET"},
	{ 0x48, USBMCPC_IO_CTRL_CLR,"USBMCPC_IO_CTRL_CLR"},
	{ 0x48, USBMCPC_CTRL2,"USBMCPC_CTRL2"},
	{ 0x48, USBMCPC_CTRL2_SET,"USBMCPC_CTRL2_SET"},
	{ 0x48, USBMCPC_CTRL2_CLR,"USBMCPC_CTRL2_CLR"},
	{ 0x48, USBOTHER_FUNC_CTRL,"USBOTHER_FUNC_CTRL"},
	{ 0x48, USBOTHER_FUNC_CTRL_SET,"USBOTHER_FUNC_CTRL_SET"},
	{ 0x48, USBOTHER_FUNC_CTRL_CLR,"USBOTHER_FUNC_CTRL_CLR"},
	{ 0x48, USBOTHER_IFC_CTRL,"USBOTHER_IFC_CTRL"},
	{ 0x48, USBOTHER_IFC_CTRL_SET,"USBOTHER_IFC_CTRL_SET"},
	{ 0x48, USBOTHER_IFC_CTRL_CLR,"USBOTHER_IFC_CTRL_CLR"},
	{ 0x48, USBOTHER_INT_EN_RISE,"USBOTHER_INT_EN_RISE"},
	{ 0x48, USBOTHER_INT_EN_RISE_SET,"USBOTHER_INT_EN_RISE_SET"},
	{ 0x48, USBOTHER_INT_EN_RISE_CLR,"USBOTHER_INT_EN_RISE_CLR"},
	{ 0x48, USBOTHER_INT_EN_FALL,"USBOTHER_INT_EN_FALL"},
	{ 0x48, USBOTHER_INT_EN_FALL_SET,"USBOTHER_INT_EN_FALL_SET"},
	{ 0x48, USBOTHER_INT_EN_FALL_CLR,"USBOTHER_INT_EN_FALL_CLR"},
	{ 0x48, USBOTHER_INT_STS,"USBOTHER_INT_STS"},
	{ 0x48, USBOTHER_INT_LATCH,"USBOTHER_INT_LATCH"},
	{ 0x48, USBID_INT_EN_RISE,"USBID_INT_EN_RISE"},
	{ 0x48, USBID_INT_EN_RISE_SET,"USBID_INT_EN_RISE_SET"},
	{ 0x48, USBID_INT_EN_RISE_CLR,"USBID_INT_EN_RISE_CLR"},
	{ 0x48, USBID_INT_EN_FALL,"USBID_INT_EN_FALL"},
	{ 0x48, USBID_INT_EN_FALL_SET,"USBID_INT_EN_FALL_SET"},
	{ 0x48, USBID_INT_EN_FALL_CLR,"USBID_INT_EN_FALL_CLR"},
	{ 0x48, USBID_INT_STS,"USBID_INT_STS"},
	{ 0x48, USBID_INT_LATCH,"USBID_INT_LATCH"},
	{ 0x48, USBID_STATUS,"USBID_STATUS"},
	{ 0x48, USBCARKIT_SM_1_INT_EN,"USBCARKIT_SM_1_INT_EN"},
	{ 0x48, USBCARKIT_SM_1_INT_EN_SET,"USBCARKIT_SM_1_INT_EN_SET"},
	{ 0x48, USBCARKIT_SM_1_INT_EN_CLR,"USBCARKIT_SM_1_INT_EN_CLR"},
	{ 0x48, USBCARKIT_SM_1_INT_STS,"USBCARKIT_SM_1_INT_STS"},
	{ 0x48, USBCARKIT_SM_1_INT_LATCH,"USBCARKIT_SM_1_INT_LATCH"},
	{ 0x48, USBCARKIT_SM_2_INT_EN,"USBCARKIT_SM_2_INT_EN"},
	{ 0x48, USBCARKIT_SM_2_INT_EN_SET,"USBCARKIT_SM_2_INT_EN_SET"},
	{ 0x48, USBCARKIT_SM_2_INT_EN_CLR,"USBCARKIT_SM_2_INT_EN_CLR"},
	{ 0x48, USBCARKIT_SM_2_INT_STS,"USBCARKIT_SM_2_INT_STS"},
	{ 0x48, USBCARKIT_SM_2_INT_LATCH,"USBCARKIT_SM_2_INT_LATCH"},
	{ 0x48, USBCARKIT_SM_CTRL,"USBCARKIT_SM_CTRL"},
	{ 0x48, USBCARKIT_SM_CTRL_SET,"USBCARKIT_SM_CTRL_SET"},
	{ 0x48, USBCARKIT_SM_CTRL_CLR,"USBCARKIT_SM_CTRL_CLR"},
	{ 0x48, USBCARKIT_SM_CMD,"USBCARKIT_SM_CMD"},
	{ 0x48, USBCARKIT_SM_CMD_SET,"USBCARKIT_SM_CMD_SET"},
	{ 0x48, USBCARKIT_SM_CMD_CLR,"USBCARKIT_SM_CMD_CLR"},
	{ 0x48, USBCARKIT_SM_CMD_STS,"USBCARKIT_SM_CMD_STS"},
	{ 0x48, USBCARKIT_SM_STATUS,"USBCARKIT_SM_STATUS"},
	{ 0x48, USBCARKIT_SM_NEXT_STATUS,"USBCARKIT_SM_NEXT_STATUS"},
	{ 0x48, USBCARKIT_SM_ERR_STATUS,"USBCARKIT_SM_ERR_STATUS"},
	{ 0x48, USBCARKIT_SM_CTRL_STATE,"USBCARKIT_SM_CTRL_STATE"},
	{ 0x48, USBPOWER_CTRL,"USBPOWER_CTRL"},
	{ 0x48, USBPOWER_CTRL_SET,"USBPOWER_CTRL_SET"},
	{ 0x48, USBPOWER_CTRL_CLR,"USBPOWER_CTRL_CLR"},
	{ 0x48, USBOTHER_IFC_CTRL2,"USBOTHER_IFC_CTRL2"},
	{ 0x48, USBOTHER_IFC_CTRL2_SET,"USBOTHER_IFC_CTRL2_SET"},
	{ 0x48, USBOTHER_IFC_CTRL2_CLR,"USBOTHER_IFC_CTRL2_CLR"},
	{ 0x48, USBREG_CTRL_EN,"USBREG_CTRL_EN"},
	{ 0x48, USBREG_CTRL_EN_SET,"USBREG_CTRL_EN_SET"},
	{ 0x48, USBREG_CTRL_EN_CLR,"USBREG_CTRL_EN_CLR"},
	{ 0x48, USBREG_CTRL_ERROR,"USBREG_CTRL_ERROR"},
	{ 0x48, USBOTHER_FUNC_CTRL2,"USBOTHER_FUNC_CTRL2"},
	{ 0x48, USBOTHER_FUNC_CTRL2_SET,"USBOTHER_FUNC_CTRL2_SET"},
	{ 0x48, USBOTHER_FUNC_CTRL2_CLR,"USBOTHER_FUNC_CTRL2_CLR"},
	{ 0x48, USBCARKIT_ANA_CTRL,"USBCARKIT_ANA_CTRL"},
	{ 0x48, USBCARKIT_ANA_CTRL_SET,"USBCARKIT_ANA_CTRL_SET"},
	{ 0x48, USBCARKIT_ANA_CTRL_CLR,"USBCARKIT_ANA_CTRL_CLR"},
	{ 0x48, USBVBUS_DEBOUNCE,"USBVBUS_DEBOUNCE"},
	{ 0x48, USBID_DEBOUNCE,"USBID_DEBOUNCE"},
	{ 0x48, USBTPH_DP_CON_MIN,"USBTPH_DP_CON_MIN"},
	{ 0x48, USBTPH_DP_CON_MAX,"USBTPH_DP_CON_MAX"},
	{ 0x48, USBTCR_DP_CON_MIN,"USBTCR_DP_CON_MIN"},
	{ 0x48, USBTCR_DP_CON_MAX,"USBTCR_DP_CON_MAX"},
	{ 0x48, USBTPH_DP_PD_SHORT,"USBTPH_DP_PD_SHORT"},
	{ 0x48, USBTPH_CMD_DLY,"USBTPH_CMD_DLY"},
	{ 0x48, USBTPH_DET_RST,"USBTPH_DET_RST"},
	{ 0x48, USBTPH_AUD_BIAS,"USBTPH_AUD_BIAS"},
	{ 0x48, USBTCR_UART_DET_MIN,"USBTCR_UART_DET_MIN"},
	{ 0x48, USBTCR_UART_DET_MAX,"USBTCR_UART_DET_MAX"},
	{ 0x48, USBTPH_ID_INT_PW,"USBTPH_ID_INT_PW"},
	{ 0x48, USBTACC_ID_INT_WAIT,"USBTACC_ID_INT_WAIT"},
	{ 0x48, USBTACC_ID_INT_PW,"USBTACC_ID_INT_PW"},
	{ 0x48, USBTPH_CMD_WAIT,"USBTPH_CMD_WAIT"},
	{ 0x48, USBTPH_ACK_WAIT,"USBTPH_ACK_WAIT"},
	{ 0x48, USBTPH_DP_DISC_DET,"USBTPH_DP_DISC_DET"},
	{ 0x48, USBVBAT_TIMER,"USBVBAT_TIMER"},
	{ 0x48, USBCARKIT_4W_DEBUG,"USBCARKIT_4W_DEBUG"},
	{ 0x48, USBCARKIT_5W_DEBUG,"USBCARKIT_5W_DEBUG"},
	{ 0x48, USBTEST_CTRL_SET,"USBTEST_CTRL_SET"},
	{ 0x48, USBTEST_CTRL_CLR,"USBTEST_CTRL_CLR"},
	{ 0x48, USBTEST_CARKIT_SET,"USBTEST_CARKIT_SET"},
	{ 0x48, USBTEST_CARKIT_CLEAR,"USBTEST_CARKIT_CLEAR"},
	{ 0x48, USBTEST_POWER_SET,"USBTEST_POWER_SET"},
	{ 0x48, USBTEST_POWER_CLR,"USBTEST_POWER_CLR"},
	{ 0x48, USBTEST_ULPI,"USBTEST_ULPI"},
	{ 0x48, USBTXVR_EN_TEST_SET,"USBTXVR_EN_TEST_SET"},
	{ 0x48, USBTXVR_EN_TEST_CLR,"USBTXVR_EN_TEST_CLR"},
	{ 0x48, USBVBUS_EN_TEST,"USBVBUS_EN_TEST"},
	{ 0x48, USBID_EN_TEST,"USBID_EN_TEST"},
	{ 0x48, USBPSM_EN_TEST_SET,"USBPSM_EN_TEST_SET"},
	{ 0x48, USBPSM_EN_TEST_CLR,"USBPSM_EN_TEST_CLR"},
	{ 0x48, USBPHY_TRIM_CTRL,"USBPHY_TRIM_CTRL"},
	{ 0x48, USBPHY_PWR_CTRL,"USBPHY_PWR_CTRL"},
	{ 0x48, USBPHY_CLK_CTRL,"USBPHY_CLK_CTRL"},
	{ 0x48, USBPHY_CLK_CTRL_STS,"USBPHY_CLK_CTRL_STS"},
	
	{ 0x49, AudioCODEC_MODE,"AudioCODEC_MODE"},
	{ 0x49, AudioOPTION,"AudioOPTION"},
	{ 0x49, AudioMICBIAS_CTL,"AudioMICBIAS_CTL"},
	{ 0x49, AudioANAMICL,"AudioANAMICL"},
	{ 0x49, AudioANAMICR,"AudioANAMICR"},
	{ 0x49, AudioAVADC_CTL,"AudioAVADC_CTL"},
	{ 0x49, AudioADCMICSEL,"AudioADCMICSEL"},
	{ 0x49, AudioDIGMIXING,"AudioDIGMIXING"},
	{ 0x49, AudioATXL1PGA,"AudioATXL1PGA"},
	{ 0x49, AudioATXR1PGA,"AudioATXR1PGA"},
	{ 0x49, AudioAVTXL2PGA,"AudioAVTXL2PGA"},
	{ 0x49, AudioAVTXR2PGA,"AudioAVTXR2PGA"},
	{ 0x49, AudioAUDIO_IF,"AudioAUDIO_IF"},
	{ 0x49, AudioVOICE_IF,"AudioVOICE_IF"},
	{ 0x49, AudioARXR1PGA,"AudioARXR1PGA"},
	{ 0x49, AudioARXL1PGA,"AudioARXL1PGA"},
	{ 0x49, AudioARXR2PGA,"AudioARXR2PGA"},
	{ 0x49, AudioARXL2PGA,"AudioARXL2PGA"},
	{ 0x49, AudioVRXPGA,"AudioVRXPGA"},
	{ 0x49, AudioVSTPGA,"AudioVSTPGA"},
	{ 0x49, AudioVRX2ARXPGA,"AudioVRX2ARXPGA"},
	{ 0x49, AudioAVDAC_CTL,"AudioAVDAC_CTL"},
	{ 0x49, AudioARX2VTXPGA,"AudioARX2VTXPGA"},
	{ 0x49, AudioARXL1_APGA_CTL,"AudioARXL1_APGA_CTL"},
	{ 0x49, AudioARXR1_APGA_CTL,"AudioARXR1_APGA_CTL"},
	{ 0x49, AudioARXL2_APGA_CTL,"AudioARXL2_APGA_CTL"},
	{ 0x49, AudioARXR2_APGA_CTL,"AudioARXR2_APGA_CTL"},
	{ 0x49, AudioATX2ARXPGA,"AudioATX2ARXPGA"},
	{ 0x49, AudioBT_IF,"AudioBT_IF"},
	{ 0x49, AudioBTPGA,"AudioBTPGA"},
	{ 0x49, AudioBTSTPGA,"AudioBTSTPGA"},
	{ 0x49, AudioEAR_CTL,"AudioEAR_CTL"},
	{ 0x49, AudioHS_SEL,"AudioHS_SEL"},
	{ 0x49, AudioHS_GAIN_SET,"AudioHS_GAIN_SET"},
	{ 0x49, AudioHS_POPN_SET,"AudioHS_POPN_SET"},
	{ 0x49, AudioPREDL_CTL,"AudioPREDL_CTL"},
	{ 0x49, AudioPREDR_CTL,"AudioPREDR_CTL"},
	{ 0x49, AudioPRECKL_CTL,"AudioPRECKL_CTL"},
	{ 0x49, AudioPRECKR_CTL,"AudioPRECKR_CTL"},
	{ 0x49, AudioHFL_CTL,"AudioHFL_CTL"},
	{ 0x49, AudioHFR_CTL,"AudioHFR_CTL"},
	{ 0x49, AudioALC_CTL,"AudioALC_CTL"},
	{ 0x49, AudioALC_SET1,"AudioALC_SET1"},
	{ 0x49, AudioALC_SET2,"AudioALC_SET2"},
	{ 0x49, AudioBOOST_CTL,"AudioBOOST_CTL"},
	{ 0x49, AudioSOFTVOL_CTL,"AudioSOFTVOL_CTL"},
	{ 0x49, AudioDTMF_FREQSEL,"AudioDTMF_FREQSEL"},
	{ 0x49, AudioDTMF_TONEXT1H,"AudioDTMF_TONEXT1H"},
	{ 0x49, AudioDTMF_TONEXT1L,"AudioDTMF_TONEXT1L"},
	{ 0x49, AudioDTMF_TONEXT2H,"AudioDTMF_TONEXT2H"},
	{ 0x49, AudioDTMF_TONEXT2L,"AudioDTMF_TONEXT2L"},
	{ 0x49, AudioDTMF_TONOFF,"AudioDTMF_TONOFF"},
	{ 0x49, AudioDTMF_WANONOFF,"AudioDTMF_WANONOFF"},
	{ 0x49, AudioI2S_RX_SCRAMBLE_H,"AudioI2S_RX_SCRAMBLE_H"},
	{ 0x49, AudioI2S_RX_SCRAMBLE_M,"AudioI2S_RX_SCRAMBLE_M"},
	{ 0x49, AudioI2S_RX_SCRAMBLE_L,"AudioI2S_RX_SCRAMBLE_L"},
	{ 0x49, AudioAPLL_CTL,"AudioAPLL_CTL"},
	{ 0x49, AudioDTMF_CTL,"AudioDTMF_CTL"},
	{ 0x49, AudioDTMF_PGA_CTL2,"AudioDTMF_PGA_CTL2"},
	{ 0x49, AudioDTMF_PGA_CTL1,"AudioDTMF_PGA_CTL1"},
	{ 0x49, AudioMISC_SET_1,"AudioMISC_SET_1"},
	{ 0x49, AudioPCMBTMUX,"AudioPCMBTMUX"},
	{ 0x49, AudioRX_PATH_SEL,"AudioRX_PATH_SEL"},
	{ 0x49, AudioVDL_APGA_CTL,"AudioVDL_APGA_CTL"},
	{ 0x49, AudioVIBRA_CTL,"AudioVIBRA_CTL"},
	{ 0x49, AudioVIBRA_SET,"AudioVIBRA_SET"},
	{ 0x49, AudioVIBRA_PWM_SET,"AudioVIBRA_PWM_SET"},
	{ 0x49, AudioANAMIC_GAIN,"AudioANAMIC_GAIN"},
	{ 0x49, AudioMISC_SET_2,"AudioMISC_SET_2"},
	{ 0x49, TESTAUDIO_TEST_CTL,"TESTAUDIO_TEST_CTL"},
	{ 0x49, TESTINT_TEST_CTL,"TESTINT_TEST_CTL"},
	{ 0x49, TESTDAC_ADC_TEST_CTL,"TESTDAC_ADC_TEST_CTL"},
	{ 0x49, TESTRXTX_TRIM_IB,"TESTRXTX_TRIM_IB"},
	{ 0x49, TESTCLD_CONTROL,"TESTCLD_CONTROL"},
	{ 0x49, TESTCLD_MODE_TIMING,"TESTCLD_MODE_TIMING"},
	{ 0x49, TESTCLD_TRIM_RAMP,"TESTCLD_TRIM_RAMP"},
	{ 0x49, TESTCLD_TESTV_CTL,"TESTCLD_TESTV_CTL"},
	{ 0x49, TESTAPLL_TEST_CTL,"TESTAPLL_TEST_CTL"},
	{ 0x49, TESTAPLL_TEST_DIV,"TESTAPLL_TEST_DIV"},
	{ 0x49, TESTAPLL_TEST_CTL2,"TESTAPLL_TEST_CTL2"},
	{ 0x49, TESTAPLL_TEST_CUR,"TESTAPLL_TEST_CUR"},
	{ 0x49, TESTDIGMIC_BIAS1_CTL,"TESTDIGMIC_BIAS1_CTL"},
	{ 0x49, TESTDIGMIC_BIAS2_CTL,"TESTDIGMIC_BIAS2_CTL"},
	{ 0x49, TESTRX_OFFSET_VOICE,"TESTRX_OFFSET_VOICE"},
	{ 0x49, TESTRX_OFFSET_AL1,"TESTRX_OFFSET_AL1"},
	{ 0x49, TESTRX_OFFSET_AR1,"TESTRX_OFFSET_AR1"},
	{ 0x49, TESTRX_OFFSET_AL2,"TESTRX_OFFSET_AL2"},
	{ 0x49, TESTRX_OFFSET_AR2,"TESTRX_OFFSET_AR2"},
	{ 0x49, TESTOFFSET1,"TESTOFFSET1"},
	{ 0x49, TESTOFFSET2,"TESTOFFSET2"},
	{ 0x49, PIH_ISR1,"PIH_ISR1"},
	{ 0x49, PIH_ISR2,"PIH_ISR2"},
	{ 0x49, PIH_SIR,"PIH_SIR"},
	{ 0x49, INTBRIDCODE_7_0,"INTBRIDCODE_7_0"},
	{ 0x49, INTBRIDCODE_15_8,"INTBRIDCODE_15_8"},
	{ 0x49, INTBRIDCODE_23_16,"INTBRIDCODE_23_16"},
	{ 0x49, INTBRIDCODE_31_24,"INTBRIDCODE_31_24"},
	{ 0x49, INTBRDIEID_7_0,"INTBRDIEID_7_0"},
	{ 0x49, INTBRDIEID_15_8,"INTBRDIEID_15_8"},
	{ 0x49, INTBRDIEID_23_16,"INTBRDIEID_23_16"},
	{ 0x49, INTBRDIEID_31_24,"INTBRDIEID_31_24"},
	{ 0x49, INTBRDIEID_39_32,"INTBRDIEID_39_32"},
	{ 0x49, INTBRDIEID_47_40,"INTBRDIEID_47_40"},
	{ 0x49, INTBRDIEID_55_48,"INTBRDIEID_55_48"},
	{ 0x49, INTBRDIEID_63_56,"INTBRDIEID_63_56"},
	{ 0x49, INTBRGPBR1,"INTBRGPBR1"},
	{ 0x49, INTBRPMBR1,"INTBRPMBR1"},
	{ 0x49, INTBRPMBR2,"INTBRPMBR2"},
	{ 0x49, INTBRGPPUPDCTR1,"INTBRGPPUPDCTR1"},
	{ 0x49, INTBRGPPUPDCTR2,"INTBRGPPUPDCTR2"},
	{ 0x49, INTBRGPPUPDCTR3,"INTBRGPPUPDCTR3"},
	{ 0x49, INTBRUNLOCK_TEST_REG,"INTBRUNLOCK_TEST_REG"},
	{ 0x49, GPIODATAIN1,"GPIODATAIN1"},
	{ 0x49, GPIODATAIN2,"GPIODATAIN2"},
	{ 0x49, GPIODATAIN3,"GPIODATAIN3"},
	{ 0x49, GPIODATADIR1,"GPIODATADIR1"},
	{ 0x49, GPIODATADIR2,"GPIODATADIR2"},
	{ 0x49, GPIODATADIR3,"GPIODATADIR3"},
	{ 0x49, GPIODATAOUT1,"GPIODATAOUT1"},
	{ 0x49, GPIODATAOUT2,"GPIODATAOUT2"},
	{ 0x49, GPIODATAOUT3,"GPIODATAOUT3"},
	{ 0x49, GPIOCLEARDATAOUT1,"GPIOCLEARDATAOUT1"},
	{ 0x49, GPIOCLEARDATAOUT2,"GPIOCLEARDATAOUT2"},
	{ 0x49, GPIOCLEARDATAOUT3,"GPIOCLEARDATAOUT3"},
	{ 0x49, GPIOSETDATAOUT1,"GPIOSETDATAOUT1"},
	{ 0x49, GPIOSETDATAOUT2,"GPIOSETDATAOUT2"},
	{ 0x49, GPIOSETDATAOUT3,"GPIOSETDATAOUT3"},
	{ 0x49, GPIO_DEBEN1,"GPIO_DEBEN1"},
	{ 0x49, GPIO_DEBEN2,"GPIO_DEBEN2"},
	{ 0x49, GPIO_DEBEN3,"GPIO_DEBEN3"},
	{ 0x49, GPIO_CTRL,"GPIO_CTRL"},
	{ 0x49, GPIOPUPDCTR1,"GPIOPUPDCTR1"},
	{ 0x49, GPIOPUPDCTR2,"GPIOPUPDCTR2"},
	{ 0x49, GPIOPUPDCTR3,"GPIOPUPDCTR3"},
	{ 0x49, GPIOPUPDCTR4,"GPIOPUPDCTR4"},
	{ 0x49, GPIOPUPDCTR5,"GPIOPUPDCTR5"},
	{ 0x49, GPIO_TEST,"GPIO_TEST"},
	{ 0x49, GPIO_ISR1A,"GPIO_ISR1A"},
	{ 0x49, GPIO_ISR2A,"GPIO_ISR2A"},
//	{ 0x49, GPIO_ISR3A,"GPIO_ISR3A"},
	{ 0x49, GPIO_IMR1A,"GPIO_IMR1A"},
	{ 0x49, GPIO_IMR2A,"GPIO_IMR2A"},
	{ 0x49, GPIO_IMR3A,"GPIO_IMR3A"},
	{ 0x49, GPIO_ISR1B,"GPIO_ISR1B"},
	{ 0x49, GPIO_ISR2B,"GPIO_ISR2B"},
	{ 0x49, GPIO_ISR3B,"GPIO_ISR3B"},
	{ 0x49, GPIO_IMR1B,"GPIO_IMR1B"},
	{ 0x49, GPIO_IMR2B,"GPIO_IMR2B"},
	{ 0x49, GPIO_IMR3B,"GPIO_IMR3B"},
	{ 0x49, GPIO_SIR1,"GPIO_SIR1"},
	{ 0x49, GPIO_SIR2,"GPIO_SIR2"},
	{ 0x49, GPIO_SIR3,"GPIO_SIR3"},
	{ 0x49, GPIO_EDR1,"GPIO_EDR1"},
	{ 0x49, GPIO_EDR2,"GPIO_EDR2"},
	{ 0x49, GPIO_EDR3,"GPIO_EDR3"},
	{ 0x49, GPIO_EDR4,"GPIO_EDR4"},
	{ 0x49, GPIO_EDR5,"GPIO_EDR5"},
	{ 0x49, GPIO_SIH_CTRL,"GPIO_SIH_CTRL"},
	
	{ 0x4a, MADCCTRL1,"MADCCTRL1"},
	{ 0x4a, MADCCTRL2,"MADCCTRL2"},
	{ 0x4a, MADCRTSELECT_LSB,"MADCRTSELECT_LSB"},
	{ 0x4a, MADCRTSELECT_MSB,"MADCRTSELECT_MSB"},
	{ 0x4a, MADCRTAVERAGE_LSB,"MADCRTAVERAGE_LSB"},
	{ 0x4a, MADCRTAVERAGE_MSB,"MADCRTAVERAGE_MSB"},
	{ 0x4a, MADCSW1SELECT_LSB,"MADCSW1SELECT_LSB"},
	{ 0x4a, MADCSW1SELECT_MSB,"MADCSW1SELECT_MSB"},
	{ 0x4a, MADCSW1AVERAGE_LSB,"MADCSW1AVERAGE_LSB"},
	{ 0x4a, MADCSW1AVERAGE_MSB,"MADCSW1AVERAGE_MSB"},
	{ 0x4a, MADCSW2SELECT_LSB,"MADCSW2SELECT_LSB"},
	{ 0x4a, MADCSW2SELECT_MSB,"MADCSW2SELECT_MSB"},
	{ 0x4a, MADCSW2AVERAGE_LSB,"MADCSW2AVERAGE_LSB"},
	{ 0x4a, MADCSW2AVERAGE_MSB,"MADCSW2AVERAGE_MSB"},
	{ 0x4a, MADCBCI_USBAVERAGE,"MADCBCI_USBAVERAGE"},
	{ 0x4a, MADCACQUISITION,"MADCACQUISITION"},
	{ 0x4a, MADCUSBREF_LSB,"MADCUSBREF_LSB"},
	{ 0x4a, MADCUSBREF_MSB,"MADCUSBREF_MSB"},
	{ 0x4a, MADCCTRL_SW1,"MADCCTRL_SW1"},
	{ 0x4a, MADCCTRL_SW2,"MADCCTRL_SW2"},
	{ 0x4a, MADCMADC_TEST,"MADCMADC_TEST"},
	{ 0x4a, MADCGP_MADC_TEST1,"MADCGP_MADC_TEST1"},
	{ 0x4a, MADCGP_MADC_TEST2,"MADCGP_MADC_TEST2"},
	{ 0x4a, MADCRTCH0_LSB,"MADCRTCH0_LSB"},
	{ 0x4a, MADCRTCH0_MSB,"MADCRTCH0_MSB"},
	{ 0x4a, MADCRTCH1_LSB,"MADCRTCH1_LSB"},
	{ 0x4a, MADCRTCH1_MSB,"MADCRTCH1_MSB"},
	{ 0x4a, MADCRTCH2_LSB,"MADCRTCH2_LSB"},
	{ 0x4a, MADCRTCH2_MSB,"MADCRTCH2_MSB"},
	{ 0x4a, MADCRTCH3_LSB,"MADCRTCH3_LSB"},
	{ 0x4a, MADCRTCH3_MSB,"MADCRTCH3_MSB"},
	{ 0x4a, MADCRTCH4_LSB,"MADCRTCH4_LSB"},
	{ 0x4a, MADCRTCH4_MSB,"MADCRTCH4_MSB"},
	{ 0x4a, MADCRTCH5_LSB,"MADCRTCH5_LSB"},
	{ 0x4a, MADCRTCH5_MSB,"MADCRTCH5_MSB"},
	{ 0x4a, MADCRTCH6_LSB,"MADCRTCH6_LSB"},
	{ 0x4a, MADCRTCH6_MSB,"MADCRTCH6_MSB"},
	{ 0x4a, MADCRTCH7_LSB,"MADCRTCH7_LSB"},
	{ 0x4a, MADCRTCH7_MSB,"MADCRTCH7_MSB"},
	{ 0x4a, MADCRTCH8_LSB,"MADCRTCH8_LSB"},
	{ 0x4a, MADCRTCH8_MSB,"MADCRTCH8_MSB"},
	{ 0x4a, MADCRTCH9_LSB,"MADCRTCH9_LSB"},
	{ 0x4a, MADCRTCH9_MSB,"MADCRTCH9_MSB"},
	{ 0x4a, MADCRTCH10_LSB,"MADCRTCH10_LSB"},
	{ 0x4a, MADCRTCH10_MSB,"MADCRTCH10_MSB"},
	{ 0x4a, MADCRTCH11_LSB,"MADCRTCH11_LSB"},
	{ 0x4a, MADCRTCH11_MSB,"MADCRTCH11_MSB"},
	{ 0x4a, MADCRTCH12_LSB,"MADCRTCH12_LSB"},
	{ 0x4a, MADCRTCH12_MSB,"MADCRTCH12_MSB"},
	{ 0x4a, MADCRTCH13_LSB,"MADCRTCH13_LSB"},
	{ 0x4a, MADCRTCH13_MSB,"MADCRTCH13_MSB"},
	{ 0x4a, MADCRTCH14_LSB,"MADCRTCH14_LSB"},
	{ 0x4a, MADCRTCH14_MSB,"MADCRTCH14_MSB"},
	{ 0x4a, MADCRTCH15_LSB,"MADCRTCH15_LSB"},
	{ 0x4a, MADCRTCH15_MSB,"MADCRTCH15_MSB"},
	{ 0x4a, MADCGPCH0_LSB,"MADCGPCH0_LSB"},
	{ 0x4a, MADCGPCH0_MSB,"MADCGPCH0_MSB"},
	{ 0x4a, MADCGPCH1_LSB,"MADCGPCH1_LSB"},
	{ 0x4a, MADCGPCH1_MSB,"MADCGPCH1_MSB"},
	{ 0x4a, MADCGPCH2_LSB,"MADCGPCH2_LSB"},
	{ 0x4a, MADCGPCH2_MSB,"MADCGPCH2_MSB"},
	{ 0x4a, MADCGPCH3_LSB,"MADCGPCH3_LSB"},
	{ 0x4a, MADCGPCH3_MSB,"MADCGPCH3_MSB"},
	{ 0x4a, MADCGPCH4_LSB,"MADCGPCH4_LSB"},
	{ 0x4a, MADCGPCH4_MSB,"MADCGPCH4_MSB"},
	{ 0x4a, MADCGPCH5_LSB,"MADCGPCH5_LSB"},
	{ 0x4a, MADCGPCH5_MSB,"MADCGPCH5_MSB"},
	{ 0x4a, MADCGPCH6_LSB,"MADCGPCH6_LSB"},
	{ 0x4a, MADCGPCH6_MSB,"MADCGPCH6_MSB"},
	{ 0x4a, MADCGPCH7_LSB,"MADCGPCH7_LSB"},
	{ 0x4a, MADCGPCH7_MSB,"MADCGPCH7_MSB"},
	{ 0x4a, MADCGPCH8_LSB,"MADCGPCH8_LSB"},
	{ 0x4a, MADCGPCH8_MSB,"MADCGPCH8_MSB"},
	{ 0x4a, MADCGPCH9_LSB,"MADCGPCH9_LSB"},
	{ 0x4a, MADCGPCH9_MSB,"MADCGPCH9_MSB"},
	{ 0x4a, MADCGPCH10_LSB,"MADCGPCH10_LSB"},
	{ 0x4a, MADCGPCH10_MSB,"MADCGPCH10_MSB"},
	{ 0x4a, MADCGPCH11_LSB,"MADCGPCH11_LSB"},
	{ 0x4a, MADCGPCH11_MSB,"MADCGPCH11_MSB"},
	{ 0x4a, MADCGPCH12_LSB,"MADCGPCH12_LSB"},
	{ 0x4a, MADCGPCH12_MSB,"MADCGPCH12_MSB"},
	{ 0x4a, MADCGPCH13_LSB,"MADCGPCH13_LSB"},
	{ 0x4a, MADCGPCH13_MSB,"MADCGPCH13_MSB"},
	{ 0x4a, MADCGPCH14_LSB,"MADCGPCH14_LSB"},
	{ 0x4a, MADCGPCH14_MSB,"MADCGPCH14_MSB"},
	{ 0x4a, MADCGPCH15_LSB,"MADCGPCH15_LSB"},
	{ 0x4a, MADCGPCH15_MSB,"MADCGPCH15_MSB"},
	{ 0x4a, MADCBCICH0_LSB,"MADCBCICH0_LSB"},
	{ 0x4a, MADCBCICH0_MSB,"MADCBCICH0_MSB"},
	{ 0x4a, MADCBCICH1_LSB,"MADCBCICH1_LSB"},
	{ 0x4a, MADCBCICH1_MSB,"MADCBCICH1_MSB"},
	{ 0x4a, MADCBCICH2_LSB,"MADCBCICH2_LSB"},
	{ 0x4a, MADCBCICH2_MSB,"MADCBCICH2_MSB"},
	{ 0x4a, MADCBCICH3_LSB,"MADCBCICH3_LSB"},
	{ 0x4a, MADCBCICH3_MSB,"MADCBCICH3_MSB"},
	{ 0x4a, MADCBCICH4_LSB,"MADCBCICH4_LSB"},
	{ 0x4a, MADCBCICH4_MSB,"MADCBCICH4_MSB"},
	{ 0x4a, MADC_ISR1,"MADC_ISR1"},
	{ 0x4a, MADC_IMR1,"MADC_IMR1"},
	{ 0x4a, MADC_ISR2,"MADC_ISR2"},
	{ 0x4a, MADC_IMR2,"MADC_IMR2"},
	{ 0x4a, MADC_SIR,"MADC_SIR"},
	{ 0x4a, MADC_EDR,"MADC_EDR"},
	{ 0x4a, MADC_SIH_CTRL,"MADC_SIH_CTRL"},
	{ 0x4a, BciISR1A,"BciISR1A"},
	{ 0x4a, BciISR2A,"BciISR2A"},
	{ 0x4a, BciIMR1A,"BciIMR1A"},
	{ 0x4a, BciIMR2A,"BciIMR2A"},
	{ 0x4a, BciISR1B,"BciISR1B"},
	{ 0x4a, BciISR2B,"BciISR2B"},
	{ 0x4a, BciIMR1B,"BciIMR1B"},
	{ 0x4a, BciIMR2B,"BciIMR2B"},
	{ 0x4a, BciSIR1,"BciSIR1"},
	{ 0x4a, BciSIR2,"BciSIR2"},
	{ 0x4a, BciEDR1,"BciEDR1"},
	{ 0x4a, BciEDR2,"BciEDR2"},
	{ 0x4a, BciEDR3,"BciEDR3"},
	{ 0x4a, BciSIHCtrl,"BciSIHCtrl"},
	{ 0x4a, BCIMDEN,"BCIMDEN"},
	{ 0x4a, BCIMDKEY,"BCIMDKEY"},
	{ 0x4a, BCIMSTATEC,"BCIMSTATEC"},
	{ 0x4a, BCIMSTATEP,"BCIMSTATEP"},
	{ 0x4a, BCIVBAT1,"BCIVBAT1"},
	{ 0x4a, BCIVBAT2,"BCIVBAT2"},
	{ 0x4a, BCITBAT1,"BCITBAT1"},
	{ 0x4a, BCITBAT2,"BCITBAT2"},
	{ 0x4a, BCIICHG1,"BCIICHG1"},
	{ 0x4a, BCIICHG2,"BCIICHG2"},
	{ 0x4a, BCIVAC1,"BCIVAC1"},
	{ 0x4a, BCIVAC2,"BCIVAC2"},
	{ 0x4a, BCIVBUS1,"BCIVBUS1"},
	{ 0x4a, BCIVBUS2,"BCIVBUS2"},
	{ 0x4a, BCIMFSTS2,"BCIMFSTS2"},
	{ 0x4a, BCIMFSTS3,"BCIMFSTS3"},
	{ 0x4a, BCIMFSTS4,"BCIMFSTS4"},
	{ 0x4a, BCIMFKEY,"BCIMFKEY"},
	{ 0x4a, BCIMFEN1,"BCIMFEN1"},
	{ 0x4a, BCIMFEN2,"BCIMFEN2"},
	{ 0x4a, BCIMFEN3,"BCIMFEN3"},
	{ 0x4a, BCIMFEN4,"BCIMFEN4"},
	{ 0x4a, BCIMFTH1,"BCIMFTH1"},
	{ 0x4a, BCIMFTH2,"BCIMFTH2"},
	{ 0x4a, BCIMFTH3,"BCIMFTH3"},
	{ 0x4a, BCIMFTH4,"BCIMFTH4"},
	{ 0x4a, BCIMFTH5,"BCIMFTH5"},
	{ 0x4a, BCIMFTH6,"BCIMFTH6"},
	{ 0x4a, BCIMFTH7,"BCIMFTH7"},
	{ 0x4a, BCIMFTH8,"BCIMFTH8"},
	{ 0x4a, BCIMFTH9,"BCIMFTH9"},
	{ 0x4a, BCITIMER1,"BCITIMER1"},
	{ 0x4a, BCITIMER2,"BCITIMER2"},
	{ 0x4a, BCIWDKEY,"BCIWDKEY"},
	{ 0x4a, BCIWD,"BCIWD"},
	{ 0x4a, BCICTL1,"BCICTL1"},
	{ 0x4a, BCICTL2,"BCICTL2"},
	{ 0x4a, BCIVREF1,"BCIVREF1"},
	{ 0x4a, BCIVREF2,"BCIVREF2"},
	{ 0x4a, BCIIREF1,"BCIIREF1"},
	{ 0x4a, BCIIREF2,"BCIIREF2"},
	{ 0x4a, BCIPWM2,"BCIPWM2"},
	{ 0x4a, BCIPWM1,"BCIPWM1"},
	{ 0x4a, BCITRIM1,"BCITRIM1"},
	{ 0x4a, BCITRIM2,"BCITRIM2"},
	{ 0x4a, BCITRIM3,"BCITRIM3"},
	{ 0x4a, BCITRIM4,"BCITRIM4"},
	{ 0x4a, BCIVREFCOMB1,"BCIVREFCOMB1"},
	{ 0x4a, BCIVREFCOMB2,"BCIVREFCOMB2"},
	{ 0x4a, BCIIREFCOMB1,"BCIIREFCOMB1"},
	{ 0x4a, BCIIREFCOMB2,"BCIIREFCOMB2"},
	{ 0x4a, BCIMNTEST1,"BCIMNTEST1"},
	{ 0x4a, BCIMNTEST2,"BCIMNTEST2"},
	{ 0x4a, BCIMNTEST3,"BCIMNTEST3"},
	{ 0x4a, BCIPSTATE,"BCIPSTATE"},
	{ 0x4a, BCIMFSTS1,"BCIMFSTS1"},
	{ 0x4a, BCITRIM5,"BCITRIM5"},
	{ 0x4a, KeyPCtrl,"KeyPCtrl"},
	{ 0x4a, KeyPDebounce,"KeyPDebounce"},
	{ 0x4a, KeyPLongKey,"KeyPLongKey"},
	{ 0x4a, KeyPLkPtv,"KeyPLkPtv"},
	{ 0x4a, KeyPTimeout1,"KeyPTimeout1"},
	{ 0x4a, KeyPTimeout2,"KeyPTimeout2"},
	{ 0x4a, KeyPKBC,"KeyPKBC"},
	{ 0x4a, KeyPKBR,"KeyPKBR"},
	{ 0x4a, KeyPSMS,"KeyPSMS"},
	{ 0x4a, KeyP_7_0,"KeyP_7_0"},
	{ 0x4a, KeyP_15_8,"KeyP_15_8"},
	{ 0x4a, KeyP_23_16,"KeyP_23_16"},
	{ 0x4a, KeyP_31_24,"KeyP_31_24"},
	{ 0x4a, KeyP_39_32,"KeyP_39_32"},
	{ 0x4a, KeyP_47_40,"KeyP_47_40"},
	{ 0x4a, KeyP_55_48,"KeyP_55_48"},
	{ 0x4a, KeyP_63_56,"KeyP_63_56"},
	{ 0x4a, KeyPISR1,"KeyPISR1"},
	{ 0x4a, KeyPIMR1,"KeyPIMR1"},
	{ 0x4a, KeyPISR2,"KeyPISR2"},
	{ 0x4a, KeyPIMR2,"KeyPIMR2"},
	{ 0x4a, KeyPSIR,"KeyPSIR"},
	{ 0x4a, KeyPEDR,"KeyPEDR"},
	{ 0x4a, KeyPSIHCtrl,"KeyPSIHCtrl"},
	{ 0x4a, LEDEN,"LEDEN"},
	{ 0x4a, PWMAON,"PWMAON"},
	{ 0x4a, PWMAOFF,"PWMAOFF"},
	{ 0x4a, PWMBON,"PWMBON"},
	{ 0x4a, PWMBOFF,"PWMBOFF"},
	{ 0x4a, PWM0ON,"PWM0ON"},
	{ 0x4a, PWM0OFF,"PWM0OFF"},
	{ 0x4a, PWM1ON,"PWM1ON"},
	{ 0x4a, PWM1OFF,"PWM1OFF"},
	
	{ 0x4b, SECURED_REG_A,"SECURED_REG_A"},
	{ 0x4b, SECURED_REG_B,"SECURED_REG_B"},
	{ 0x4b, SECURED_REG_C,"SECURED_REG_C"},
	{ 0x4b, SECURED_REG_D,"SECURED_REG_D"},
	{ 0x4b, SECURED_REG_E,"SECURED_REG_E"},
	{ 0x4b, SECURED_REG_F,"SECURED_REG_F"},
	{ 0x4b, SECURED_REG_G,"SECURED_REG_G"},
	{ 0x4b, SECURED_REG_H,"SECURED_REG_H"},
	{ 0x4b, SECURED_REG_I,"SECURED_REG_I"},
	{ 0x4b, SECURED_REG_J,"SECURED_REG_J"},
	{ 0x4b, SECURED_REG_K,"SECURED_REG_K"},
	{ 0x4b, SECURED_REG_L,"SECURED_REG_L"},
	{ 0x4b, SECURED_REG_M,"SECURED_REG_M"},
	{ 0x4b, SECURED_REG_N,"SECURED_REG_N"},
	{ 0x4b, SECURED_REG_O,"SECURED_REG_O"},
	{ 0x4b, SECURED_REG_P,"SECURED_REG_P"},
	{ 0x4b, SECURED_REG_Q,"SECURED_REG_Q"},
	{ 0x4b, SECURED_REG_R,"SECURED_REG_R"},
	{ 0x4b, SECURED_REG_S,"SECURED_REG_S"},
	{ 0x4b, SECURED_REG_U,"SECURED_REG_U"},
	{ 0x4b, BACKUP_REG_A,"BACKUP_REG_A"},
	{ 0x4b, BACKUP_REG_B,"BACKUP_REG_B"},
	{ 0x4b, BACKUP_REG_C,"BACKUP_REG_C"},
	{ 0x4b, BACKUP_REG_D,"BACKUP_REG_D"},
	{ 0x4b, BACKUP_REG_E,"BACKUP_REG_E"},
	{ 0x4b, BACKUP_REG_F,"BACKUP_REG_F"},
	{ 0x4b, BACKUP_REG_G,"BACKUP_REG_G"},
	{ 0x4b, BACKUP_REG_H,"BACKUP_REG_H"},
	{ 0x4b, RTCSECONDS_REG,"RTCSECONDS_REG"},
	{ 0x4b, RTCMINUTES_REG,"RTCMINUTES_REG"},
	{ 0x4b, RTCHOURS_REG,"RTCHOURS_REG"},
	{ 0x4b, RTCDAYS_REG,"RTCDAYS_REG"},
	{ 0x4b, RTCMONTHS_REG,"RTCMONTHS_REG"},
	{ 0x4b, RTCYEARS_REG,"RTCYEARS_REG"},
	{ 0x4b, RTCWEEKS_REG,"RTCWEEKS_REG"},
	{ 0x4b, RTCALARM_SECONDS_REG,"RTCALARM_SECONDS_REG"},
	{ 0x4b, RTCALARM_MINUTES_REG,"RTCALARM_MINUTES_REG"},
	{ 0x4b, RTCALARM_HOURS_REG,"RTCALARM_HOURS_REG"},
	{ 0x4b, RTCALARM_DAYS_REG,"RTCALARM_DAYS_REG"},
	{ 0x4b, RTCALARM_MONTHS_REG,"RTCALARM_MONTHS_REG"},
	{ 0x4b, RTCALARM_YEARS_REG,"RTCALARM_YEARS_REG"},
	{ 0x4b, RTC_CTRL_REG,"RTC_CTRL_REG"},
	{ 0x4b, RTC_STATUS_REG,"RTC_STATUS_REG"},
	{ 0x4b, RTC_INTERRUPTS_REG,"RTC_INTERRUPTS_REG"},
	{ 0x4b, RTC_COMP_LSB_REG,"RTC_COMP_LSB_REG"},
	{ 0x4b, RTC_COMP_MSB_REG,"RTC_COMP_MSB_REG"},
	{ 0x4b, PWR_ISR1,"PWR_ISR1"},
	{ 0x4b, PWR_IMR1,"PWR_IMR1"},
	{ 0x4b, PWR_ISR2,"PWR_ISR2"},
	{ 0x4b, PWR_IMR2,"PWR_IMR2"},
	{ 0x4b, PWR_SIR,"PWR_SIR"},
	{ 0x4b, PWR_EDR1,"PWR_EDR1"},
	{ 0x4b, PWR_EDR2,"PWR_EDR2"},
	{ 0x4b, PWR_SIH_CTRL,"PWR_SIH_CTRL"},
	{ 0x4b, PWRCFG_P1_TRANSITION,"PWRCFG_P1_TRANSITION"},
	{ 0x4b, PWRCFG_P2_TRANSITION,"PWRCFG_P2_TRANSITION"},
	{ 0x4b, PWRCFG_P3_TRANSITION,"PWRCFG_P3_TRANSITION"},
	{ 0x4b, PWRCFG_P123_TRANSITION,"PWRCFG_P123_TRANSITION"},
	{ 0x4b, PWRSTS_BOOT,"PWRSTS_BOOT"},
	{ 0x4b, PWRCFG_BOOT,"PWRCFG_BOOT"},
	{ 0x4b, PWRSHUNDAN,"PWRSHUNDAN"},
	{ 0x4b, PWRBOOT_BCI,"PWRBOOT_BCI"},
	{ 0x4b, PWRCFG_PWRANA1,"PWRCFG_PWRANA1"},
	{ 0x4b, PWRCFG_PWRANA2,"PWRCFG_PWRANA2"},
	{ 0x4b, PWRBGAP_TRIM,"PWRBGAP_TRIM"},
	{ 0x4b, PWRBACKUP_MISC_STS,"PWRBACKUP_MISC_STS"},
	{ 0x4b, PWRBACKUP_MISC_CFG,"PWRBACKUP_MISC_CFG"},
	{ 0x4b, PWRBACKUP_MISC_TST,"PWRBACKUP_MISC_TST"},
	{ 0x4b, PWRPROTECT_KEY,"PWRPROTECT_KEY"},
	{ 0x4b, PWRSTS_HW_CONDITIONS,"PWRSTS_HW_CONDITIONS"},
	{ 0x4b, PWRP1_SW_EVENTS,"PWRP1_SW_EVENTS"},
	{ 0x4b, PWRP2_SW_EVENTS,"PWRP2_SW_EVENTS"},
	{ 0x4b, PWRP3_SW_EVENTS,"PWRP3_SW_EVENTS"},
	{ 0x4b, PWRSTS_P123_STATE,"PWRSTS_P123_STATE"},
	{ 0x4b, PWRPB_CFG,"PWRPB_CFG"},
	{ 0x4b, PWRPB_WORD_MSB,"PWRPB_WORD_MSB"},
	{ 0x4b, PWRPB_WORD_LSB,"PWRPB_WORD_LSB"},
	{ 0x4b, PWRRESERVED_A,"PWRRESERVED_A"},
	{ 0x4b, PWRRESERVED_B,"PWRRESERVED_B"},
	{ 0x4b, PWRRESERVED_C,"PWRRESERVED_C"},
	{ 0x4b, PWRRESERVED_D,"PWRRESERVED_D"},
	{ 0x4b, PWRRESERVED_E,"PWRRESERVED_E"},
	{ 0x4b, PWRSEQ_ADD_W2P,"PWRSEQ_ADD_W2P"},
	{ 0x4b, PWRSEQ_ADD_P2A,"PWRSEQ_ADD_P2A"},
	{ 0x4b, PWRSEQ_ADD_A2W,"PWRSEQ_ADD_A2W"},
	{ 0x4b, PWRSEQ_ADD_A2S,"PWRSEQ_ADD_A2S"},
	{ 0x4b, PWRSEQ_ADD_S2A12,"PWRSEQ_ADD_S2A12"},
	{ 0x4b, PWRSEQ_ADD_S2A3,"PWRSEQ_ADD_S2A3"},
	{ 0x4b, PWRSEQ_ADD_WARM,"PWRSEQ_ADD_WARM"},
	{ 0x4b, PWRMEMORY_ADDRESS,"PWRMEMORY_ADDRESS"},
	{ 0x4b, PWRMEMORY_DATA,"PWRMEMORY_DATA"},
	{ 0x4b, PWRSC_CONFIG,"PWRSC_CONFIG"},
	{ 0x4b, PWRSC_DETECT1,"PWRSC_DETECT1"},
	{ 0x4b, PWRSC_DETECT2,"PWRSC_DETECT2"},
	{ 0x4b, PWRWATCHDOG_CFG,"PWRWATCHDOG_CFG"},
	{ 0x4b, PWRIT_CHECK_CFG,"PWRIT_CHECK_CFG"},
	{ 0x4b, PWRVIBRATOR_CFG,"PWRVIBRATOR_CFG"},
	{ 0x4b, PWRDCDC_GLOBAL_CFG,"PWRDCDC_GLOBAL_CFG"},
	{ 0x4b, PWRVDD1_TRIM1,"PWRVDD1_TRIM1"},
	{ 0x4b, PWRVDD1_TRIM2,"PWRVDD1_TRIM2"},
	{ 0x4b, PWRVDD2_TRIM1,"PWRVDD2_TRIM1"},
	{ 0x4b, PWRVDD2_TRIM2,"PWRVDD2_TRIM2"},
	{ 0x4b, PWRVIO_TRIM1,"PWRVIO_TRIM1"},
	{ 0x4b, PWRVIO_TRIM2,"PWRVIO_TRIM2"},
	{ 0x4b, PWRMISC_CFG,"PWRMISC_CFG"},
	{ 0x4b, PWRLS_TST_A,"PWRLS_TST_A"},
	{ 0x4b, PWRLS_TST_B,"PWRLS_TST_B"},
	{ 0x4b, PWRLS_TST_C,"PWRLS_TST_C"},
	{ 0x4b, PWRLS_TST_D,"PWRLS_TST_D"},
	{ 0x4b, PWRBB_CFG,"PWRBB_CFG"},
	{ 0x4b, PWRMISC_TST,"PWRMISC_TST"},
	{ 0x4b, PWRTRIM1,"PWRTRIM1"},
	{ 0x4b, PWRTRIM2,"PWRTRIM2"},
	{ 0x4b, PWRDCDC_TIMEOUT,"PWRDCDC_TIMEOUT"},
	{ 0x4b, PWRVAUX1_DEV_GRP,"PWRVAUX1_DEV_GRP"},
	{ 0x4b, PWRVAUX1_TYPE,"PWRVAUX1_TYPE"},
	{ 0x4b, PWRVAUX1_REMAP,"PWRVAUX1_REMAP"},
	{ 0x4b, PWRVAUX1_DEDICATED,"PWRVAUX1_DEDICATED"},
	{ 0x4b, PWRVAUX2_DEV_GRP,"PWRVAUX2_DEV_GRP"},
	{ 0x4b, PWRVAUX2_TYPE,"PWRVAUX2_TYPE"},
	{ 0x4b, PWRVAUX2_REMAP,"PWRVAUX2_REMAP"},
	{ 0x4b, PWRVAUX2_DEDICATED,"PWRVAUX2_DEDICATED"},
	{ 0x4b, PWRVAUX3_DEV_GRP,"PWRVAUX3_DEV_GRP"},
	{ 0x4b, PWRVAUX3_TYPE,"PWRVAUX3_TYPE"},
	{ 0x4b, PWRVAUX3_REMAP,"PWRVAUX3_REMAP"},
	{ 0x4b, PWRVAUX3_DEDICATED,"PWRVAUX3_DEDICATED"},
	{ 0x4b, PWRVAUX4_DEV_GRP,"PWRVAUX4_DEV_GRP"},
	{ 0x4b, PWRVAUX4_TYPE,"PWRVAUX4_TYPE"},
	{ 0x4b, PWRVAUX4_REMAP,"PWRVAUX4_REMAP"},
	{ 0x4b, PWRVAUX4_DEDICATED,"PWRVAUX4_DEDICATED"},
	{ 0x4b, PWRVMMC1_DEV_GRP,"PWRVMMC1_DEV_GRP"},
	{ 0x4b, PWRVMMC1_TYPE,"PWRVMMC1_TYPE"},
	{ 0x4b, PWRVMMC1_REMAP,"PWRVMMC1_REMAP"},
	{ 0x4b, PWRVMMC1_DEDICATED,"PWRVMMC1_DEDICATED"},
	{ 0x4b, PWRVMMC2_DEV_GRP,"PWRVMMC2_DEV_GRP"},
	{ 0x4b, PWRVMMC2_TYPE,"PWRVMMC2_TYPE"},
	{ 0x4b, PWRVMMC2_REMAP,"PWRVMMC2_REMAP"},
	{ 0x4b, PWRVMMC2_DEDICATED,"PWRVMMC2_DEDICATED"},
	{ 0x4b, PWRVPLL1_DEV_GRP,"PWRVPLL1_DEV_GRP"},
	{ 0x4b, PWRVPLL1_TYPE,"PWRVPLL1_TYPE"},
	{ 0x4b, PWRVPLL1_REMAP,"PWRVPLL1_REMAP"},
	{ 0x4b, PWRVPLL1_DEDICATED,"PWRVPLL1_DEDICATED"},
	{ 0x4b, PWRVPLL2_DEV_GRP,"PWRVPLL2_DEV_GRP"},
	{ 0x4b, PWRVPLL2_TYPE,"PWRVPLL2_TYPE"},
	{ 0x4b, PWRVPLL2_REMAP,"PWRVPLL2_REMAP"},
	{ 0x4b, PWRVPLL2_DEDICATED,"PWRVPLL2_DEDICATED"},
	{ 0x4b, PWRVSIM_DEV_GRP,"PWRVSIM_DEV_GRP"},
	{ 0x4b, PWRVSIM_TYPE,"PWRVSIM_TYPE"},
	{ 0x4b, PWRVSIM_REMAP,"PWRVSIM_REMAP"},
	{ 0x4b, PWRVSIM_DEDICATED,"PWRVSIM_DEDICATED"},
	{ 0x4b, PWRVDAC_DEV_GRP,"PWRVDAC_DEV_GRP"},
	{ 0x4b, PWRVDAC_TYPE,"PWRVDAC_TYPE"},
	{ 0x4b, PWRVDAC_REMAP,"PWRVDAC_REMAP"},
	{ 0x4b, PWRVDAC_DEDICATED,"PWRVDAC_DEDICATED"},
	{ 0x4b, PWRVINTANA1_DEV_GRP,"PWRVINTANA1_DEV_GRP"},
	{ 0x4b, PWRVINTANA1_TYPE,"PWRVINTANA1_TYPE"},
	{ 0x4b, PWRVINTANA1_REMAP,"PWRVINTANA1_REMAP"},
	{ 0x4b, PWRVINTANA1_DEDICATED,"PWRVINTANA1_DEDICATED"},
	{ 0x4b, PWRVINTANA2_DEV_GRP,"PWRVINTANA2_DEV_GRP"},
	{ 0x4b, PWRVINTANA2_TYPE,"PWRVINTANA2_TYPE"},
	{ 0x4b, PWRVINTANA2_REMAP,"PWRVINTANA2_REMAP"},
	{ 0x4b, PWRVINTANA2_DEDICATED,"PWRVINTANA2_DEDICATED"},
	{ 0x4b, PWRVINTDIG_DEV_GRP,"PWRVINTDIG_DEV_GRP"},
	{ 0x4b, PWRVINTDIG_TYPE,"PWRVINTDIG_TYPE"},
	{ 0x4b, PWRVINTDIG_REMAP,"PWRVINTDIG_REMAP"},
	{ 0x4b, PWRVINTDIG_DEDICATED,"PWRVINTDIG_DEDICATED"},
	{ 0x4b, PWRVIO_DEV_GRP,"PWRVIO_DEV_GRP"},
	{ 0x4b, PWRVIO_TYPE,"PWRVIO_TYPE"},
	{ 0x4b, PWRVIO_REMAP,"PWRVIO_REMAP"},
	{ 0x4b, PWRVIO_CFG,"PWRVIO_CFG"},
	{ 0x4b, PWRVIO_MISC_CFG,"PWRVIO_MISC_CFG"},
	{ 0x4b, PWRVIO_TEST1,"PWRVIO_TEST1"},
	{ 0x4b, PWRVIO_TEST2,"PWRVIO_TEST2"},
	{ 0x4b, PWRVIO_OSC,"PWRVIO_OSC"},
	{ 0x4b, PWRVIO_RESERVED,"PWRVIO_RESERVED"},
	{ 0x4b, PWRVIO_VSEL,"PWRVIO_VSEL"},
	{ 0x4b, PWRVDD1_DEV_GRP,"PWRVDD1_DEV_GRP"},
	{ 0x4b, PWRVDD1_TYPE,"PWRVDD1_TYPE"},
	{ 0x4b, PWRVDD1_REMAP,"PWRVDD1_REMAP"},
	{ 0x4b, PWRVDD1_CFG,"PWRVDD1_CFG"},
	{ 0x4b, PWRVDD1_MISC_CFG,"PWRVDD1_MISC_CFG"},
	{ 0x4b, PWRVDD1_TEST1,"PWRVDD1_TEST1"},
	{ 0x4b, PWRVDD1_TEST2,"PWRVDD1_TEST2"},
	{ 0x4b, PWRVDD1_OSC,"PWRVDD1_OSC"},
	{ 0x4b, PWRVDD1_RESERVED,"PWRVDD1_RESERVED"},
	{ 0x4b, PWRVDD1_VSEL,"PWRVDD1_VSEL"},
	{ 0x4b, PWRVDD1_VMODE_CFG,"PWRVDD1_VMODE_CFG"},
	{ 0x4b, PWRVDD1_VFLOOR,"PWRVDD1_VFLOOR"},
	{ 0x4b, PWRVDD1_VROOF,"PWRVDD1_VROOF"},
	{ 0x4b, PWRVDD1_STEP,"PWRVDD1_STEP"},
	{ 0x4b, PWRVDD2_DEV_GRP,"PWRVDD2_DEV_GRP"},
	{ 0x4b, PWRVDD2_TYPE,"PWRVDD2_TYPE"},
	{ 0x4b, PWRVDD2_REMAP,"PWRVDD2_REMAP"},
	{ 0x4b, PWRVDD2_CFG,"PWRVDD2_CFG"},
	{ 0x4b, PWRVDD2_MISC_CFG,"PWRVDD2_MISC_CFG"},
	{ 0x4b, PWRVDD2_TEST1,"PWRVDD2_TEST1"},
	{ 0x4b, PWRVDD2_TEST2,"PWRVDD2_TEST2"},
	{ 0x4b, PWRVDD2_OSC,"PWRVDD2_OSC"},
	{ 0x4b, PWRVDD2_RESERVED,"PWRVDD2_RESERVED"},
	{ 0x4b, PWRVDD2_VSEL,"PWRVDD2_VSEL"},
	{ 0x4b, PWRVDD2_VMODE_CFG,"PWRVDD2_VMODE_CFG"},
	{ 0x4b, PWRVDD2_VFLOOR,"PWRVDD2_VFLOOR"},
	{ 0x4b, PWRVDD2_VROOF,"PWRVDD2_VROOF"},
	{ 0x4b, PWRVDD2_STEP,"PWRVDD2_STEP"},
	{ 0x4b, PWRVUSB1V5_DEV_GRP,"PWRVUSB1V5_DEV_GRP"},
	{ 0x4b, PWRVUSB1V5_TYPE,"PWRVUSB1V5_TYPE"},
	{ 0x4b, PWRVUSB1V5_REMAP,"PWRVUSB1V5_REMAP"},
	{ 0x4b, PWRVUSB1V8_DEV_GRP,"PWRVUSB1V8_DEV_GRP"},
	{ 0x4b, PWRVUSB1V8_TYPE,"PWRVUSB1V8_TYPE"},
	{ 0x4b, PWRVUSB1V8_REMAP,"PWRVUSB1V8_REMAP"},
	{ 0x4b, PWRVUSB3V1_DEV_GRP,"PWRVUSB3V1_DEV_GRP"},
	{ 0x4b, PWRVUSB3V1_TYPE,"PWRVUSB3V1_TYPE"},
	{ 0x4b, PWRVUSB3V1_REMAP,"PWRVUSB3V1_REMAP"},
	{ 0x4b, PWRVUSBCP_DEV_GRP,"PWRVUSBCP_DEV_GRP"},
	{ 0x4b, PWRVUSBCP_TYPE,"PWRVUSBCP_TYPE"},
	{ 0x4b, PWRVUSBCP_REMAP,"PWRVUSBCP_REMAP"},
	{ 0x4b, PWRVUSB_DEDICATED1,"PWRVUSB_DEDICATED1"},
	{ 0x4b, PWRVUSB_DEDICATED2,"PWRVUSB_DEDICATED2"},
	{ 0x4b, PWRREGEN_DEV_GRP,"PWRREGEN_DEV_GRP"},
	{ 0x4b, PWRREGEN_TYPE,"PWRREGEN_TYPE"},
	{ 0x4b, PWRREGEN_REMAP,"PWRREGEN_REMAP"},
	{ 0x4b, PWRNRESPWRON_DEV_GRP,"PWRNRESPWRON_DEV_GRP"},
	{ 0x4b, PWRNRESPWRON_TYPE,"PWRNRESPWRON_TYPE"},
	{ 0x4b, PWRNRESPWRON_REMAP,"PWRNRESPWRON_REMAP"},
	{ 0x4b, PWRCLKEN_DEV_GRP,"PWRCLKEN_DEV_GRP"},
	{ 0x4b, PWRCLKEN_TYPE,"PWRCLKEN_TYPE"},
	{ 0x4b, PWRCLKEN_REMAP,"PWRCLKEN_REMAP"},
	{ 0x4b, PWRSYSEN_DEV_GRP,"PWRSYSEN_DEV_GRP"},
	{ 0x4b, PWRSYSEN_TYPE,"PWRSYSEN_TYPE"},
	{ 0x4b, PWRSYSEN_REMAP,"PWRSYSEN_REMAP"},
	{ 0x4b, PWRHFCLKOUT_DEV_GRP,"PWRHFCLKOUT_DEV_GRP"},
	{ 0x4b, PWRHFCLKOUT_TYPE,"PWRHFCLKOUT_TYPE"},
	{ 0x4b, PWRHFCLKOUT_REMAP,"PWRHFCLKOUT_REMAP"},
	{ 0x4b, PWR32KCLKOUT_DEV_GRP,"PWR32KCLKOUT_DEV_GRP"},
	{ 0x4b, PWR32KCLKOUT_TYPE,"PWR32KCLKOUT_TYPE"},
	{ 0x4b, PWR32KCLKOUT_REMAP,"PWR32KCLKOUT_REMAP"},
	{ 0x4b, PWRTRITON_RESET_DEV_GRP,"PWRTRITON_RESET_DEV_GRP"},
	{ 0x4b, PWRTRITON_RESET_TYPE,"PWRTRITON_RESET_TYPE"},
	{ 0x4b, PWRTRITON_RESET_REMAP,"PWRTRITON_RESET_REMAP"},
	{ 0x4b, PWRMAINREF_DEV_GRP,"PWRMAINREF_DEV_GRP"},
	{ 0x4b, PWRMAINREF_TYPE,"PWRMAINREF_TYPE"},
	{ 0x4b, PWRMAINREF_REMAP,"PWRMAINREF_REMAP"},
	
	{ 0x12, SMRVDD1_SR_CONTROL,"SMRVDD1_SR_CONTROL"},
	{ 0x12, SMRVDD2_SR_CONTROL,"SMRVDD2_SR_CONTROL"},
	{ 0xFF,0xFF,"33"}
};

static int twl_i2c_read_dump(u8 addr, u8 *value, u8 reg)
{
	int ret;
	u8 val;
	int sid;
	struct twl_client *twl;
	struct i2c_msg *msg;

	if( addr >= 0x48 )
		sid = addr-0x48;
	else
		sid = 4;
		
	twl = &twl_modules[sid];

	if (unlikely(!inuse)) {
		pr_err("%s: client %d is not initialized\n", DRIVER_NAME, sid);
		return -EPERM;
	}
	mutex_lock(&twl->xfer_lock);
	/* [MSG1] fill the register address data */
	msg = &twl->xfer_msg[0];
	msg->addr = twl->address;
	msg->len = 1;
	msg->flags = 0;	/* Read the register value */
	val = reg;
	msg->buf = &val;
	/* [MSG2] fill the data rx buffer */
	msg = &twl->xfer_msg[1];
	msg->addr = twl->address;
	msg->flags = I2C_M_RD;	/* Read the register value */
	msg->len = 1;	/* only n bytes */
	msg->buf = value;
	ret = i2c_transfer(twl->client->adapter, twl->xfer_msg, 2);
	mutex_unlock(&twl->xfer_lock);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 2) {
		pr_err("%s: i2c_read failed to transfer all messages\n",
			DRIVER_NAME);
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}

int twl_i2c_read_regdump()
{
	u16 i=0;
	u8 temp;
	
	printk("************************TWL5025/5030 Full Register Dump[Start]************************\n");
	for(;t2_reg_full_dump_list[i].dev_id != 0xff; i++)
		if( !twl_i2c_read_dump(t2_reg_full_dump_list[i].dev_id, &temp, t2_reg_full_dump_list[i].reg) )
			printk("%s = 0x%x\n", t2_reg_full_dump_list[i].regn, temp);
		else
			printk("Read Error!! while reading %s\n", t2_reg_full_dump_list[i].regn);
	printk("************************TWL5025/5030 Full Register Dump[End]************************\n");
}
EXPORT_SYMBOL(twl_i2c_read_regdump);
#endif

/**
 * twl_i2c_read - Reads a n bit register in TWL4030/TWL5030/TWL60X0
 * @mod_no: module number
 * @value: an array of num_bytes containing data to be read
 * @reg: register address (just offset will do)
 * @num_bytes: number of bytes to transfer
 *
 * Returns result of operation - num_bytes is success else failure.
 */
int twl_i2c_read(u8 mod_no, u8 *value, u8 reg, unsigned num_bytes)
{
	int ret;
	u8 val;
	int sid;
	struct twl_client *twl;
	struct i2c_msg *msg;

	if (unlikely(mod_no > TWL_MODULE_LAST)) {
		pr_err("%s: invalid module number %d\n", DRIVER_NAME, mod_no);
		return -EPERM;
	}
	sid = twl_map[mod_no].sid;
	twl = &twl_modules[sid];

	if (unlikely(!inuse)) {
		pr_err("%s: client %d is not initialized\n", DRIVER_NAME, sid);
		return -EPERM;
	}
	mutex_lock(&twl->xfer_lock);
	/* [MSG1] fill the register address data */
	msg = &twl->xfer_msg[0];
	msg->addr = twl->address;
	msg->len = 1;
	msg->flags = 0;	/* Read the register value */
	val = twl_map[mod_no].base + reg;
	msg->buf = &val;
	/* [MSG2] fill the data rx buffer */
	msg = &twl->xfer_msg[1];
	msg->addr = twl->address;
	msg->flags = I2C_M_RD;	/* Read the register value */
	msg->len = num_bytes;	/* only n bytes */
	msg->buf = value;
	ret = i2c_transfer(twl->client->adapter, twl->xfer_msg, 2);
	mutex_unlock(&twl->xfer_lock);

	/* i2c_transfer returns number of messages transferred */
	if (ret != 2) {
		pr_err("%s: i2c_read failed to transfer all messages\n",
			DRIVER_NAME);
		if (ret < 0)
			return ret;
		else
			return -EIO;
	} else {
		return 0;
	}
}
EXPORT_SYMBOL(twl_i2c_read);

/**
 * twl_i2c_write_u8 - Writes a 8 bit register in TWL4030/TWL5030/TWL60X0
 * @mod_no: module number
 * @value: the value to be written 8 bit
 * @reg: register address (just offset will do)
 *
 * Returns result of operation - 0 is success
 */
int twl_i2c_write_u8(u8 mod_no, u8 value, u8 reg)
{

	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	u8 temp_buffer[2] = { 0 };
	/* offset 1 contains the data */
	temp_buffer[1] = value;
	return twl_i2c_write(mod_no, temp_buffer, reg, 1);
}
EXPORT_SYMBOL(twl_i2c_write_u8);

/**
 * twl_i2c_read_u8 - Reads a 8 bit register from TWL4030/TWL5030/TWL60X0
 * @mod_no: module number
 * @value: the value read 8 bit
 * @reg: register address (just offset will do)
 *
 * Returns result of operation - 0 is success
 */
int twl_i2c_read_u8(u8 mod_no, u8 *value, u8 reg)
{
	return twl_i2c_read(mod_no, value, reg, 1);
}
EXPORT_SYMBOL(twl_i2c_read_u8);

/*----------------------------------------------------------------------*/

static struct device *
add_numbered_child(unsigned chip, const char *name, int num,
		void *pdata, unsigned pdata_len,
		bool can_wakeup, int irq0, int irq1)
{
	struct platform_device	*pdev;
	struct twl_client	*twl = &twl_modules[chip];
	int			status;

	pdev = platform_device_alloc(name, num);
	if (!pdev) {
		dev_dbg(&twl->client->dev, "can't alloc dev\n");
		status = -ENOMEM;
		goto err;
	}

	device_init_wakeup(&pdev->dev, can_wakeup);
	pdev->dev.parent = &twl->client->dev;

	if (pdata) {
		status = platform_device_add_data(pdev, pdata, pdata_len);
		if (status < 0) {
			dev_dbg(&pdev->dev, "can't add platform_data\n");
			goto err;
		}
	}

	if (irq0) {
		struct resource r[2] = {
			{ .start = irq0, .flags = IORESOURCE_IRQ, },
			{ .start = irq1, .flags = IORESOURCE_IRQ, },
		};

		status = platform_device_add_resources(pdev, r, irq1 ? 2 : 1);
		if (status < 0) {
			dev_dbg(&pdev->dev, "can't add irqs\n");
			goto err;
		}
	}

	status = platform_device_add(pdev);

err:
	if (status < 0) {
		platform_device_put(pdev);
		dev_err(&twl->client->dev, "can't add %s dev\n", name);
		return ERR_PTR(status);
	}
	return &pdev->dev;
}

static inline struct device *add_child(unsigned chip, const char *name,
		void *pdata, unsigned pdata_len,
		bool can_wakeup, int irq0, int irq1)
{
	return add_numbered_child(chip, name, -1, pdata, pdata_len,
		can_wakeup, irq0, irq1);
}

static struct device *
add_regulator_linked(int num, struct regulator_init_data *pdata,
		struct regulator_consumer_supply *consumers,
		unsigned num_consumers)
{
	unsigned sub_chip_id;
	/* regulator framework demands init_data ... */
	if (!pdata)
		return NULL;

	if (consumers) {
		pdata->consumer_supplies = consumers;
		pdata->num_consumer_supplies = num_consumers;
	}

	/* NOTE:  we currently ignore regulator IRQs, e.g. for short circuits */
	sub_chip_id = twl_map[TWL_MODULE_PM_MASTER].sid;
	return add_numbered_child(sub_chip_id, "twl_reg", num,
		pdata, sizeof(*pdata), false, 0, 0);
}

static struct device *
add_regulator(int num, struct regulator_init_data *pdata)
{
	return add_regulator_linked(num, pdata, NULL, 0);
}

/*
 * NOTE:  We know the first 8 IRQs after pdata->base_irq are
 * for the PIH, and the next are for the PWR_INT SIH, since
 * that's how twl_init_irq() sets things up.
 */

static int
add_children(struct twl4030_platform_data *pdata, unsigned long features)
{
	struct device	*child;
	unsigned sub_chip_id;

	if (twl_has_gpio() && pdata->gpio) {
		child = add_child(SUB_CHIP_ID1, "twl4030_gpio",
				pdata->gpio, sizeof(*pdata->gpio),
				false, pdata->irq_base + GPIO_INTR_OFFSET, 0);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	if (twl_has_keypad() && pdata->keypad) {
		child = add_child(SUB_CHIP_ID2, "twl4030_keypad",
				pdata->keypad, sizeof(*pdata->keypad),
				true, pdata->irq_base + KEYPAD_INTR_OFFSET, 0);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}
		if (twl_has_bci() && pdata->bci &&
			!(features & (TPS_SUBSET | TWL5031))) {
			child = add_child(3, "twl4030_bci",
			pdata->bci, sizeof(*pdata->bci),
			false,
			/* irq0 = CHG_PRES, irq1 = BCI */
			pdata->irq_base + BCI_PRES_INTR_OFFSET,
			pdata->irq_base + BCI_INTR_OFFSET);
			if (IS_ERR(child))
				return PTR_ERR(child);
		}
	if (twl_has_bci() && pdata->bci &&
	    (features & TWL6030_CLASS)) {
		child = add_child(1, "twl6030_bci",
				pdata->bci, sizeof(*pdata->bci),
				false,
				pdata->irq_base + CHARGER_INTR_OFFSET,
				pdata->irq_base + CHARGERFAULT_INTR_OFFSET);
	}


	if (twl_has_madc() && pdata->madc && twl_class_is_4030()) {
		child = add_child(2, "twl4030_madc",
				pdata->madc, sizeof(*pdata->madc),
				true, pdata->irq_base + MADC_INTR_OFFSET, 0);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	if (twl_has_madc() && pdata->madc && twl_class_is_6030()) {
		child = add_child(1, "twl6030_gpadc",
				pdata->madc, sizeof(*pdata->madc),
				true, pdata->irq_base + MADC_INTR_OFFSET,
				pdata->irq_base + GPADCSW_INTR_OFFSET);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	if (twl_has_rtc()) {
		/*
		 * REVISIT platform_data here currently might expose the
		 * "msecure" line ... but for now we just expect board
		 * setup to tell the chip "it's always ok to SET_TIME".
		 * Eventually, Linux might become more aware of such
		 * HW security concerns, and "least privilege".
		 */
		sub_chip_id = twl_map[TWL_MODULE_RTC].sid;
		child = add_child(sub_chip_id, "twl_rtc",
				NULL, 0,
				true, pdata->irq_base + RTC_INTR_OFFSET, 0);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	if (twl_has_usb() && pdata->usb && twl_class_is_4030()) {

		static struct regulator_consumer_supply usb1v5[] = {
                          {
                        .supply =       "usb1v5",
                     },
                     {
                        .supply =       "usb1v5",
                     },
		};
		static struct regulator_consumer_supply usb1v8[] = {
                   {
                                .supply =       "usb1v8",
                        },
                        {
                                .supply =       "usb1v8",
                        },
		};
		static struct regulator_consumer_supply usb3v1[] = {
                     {
                                .supply =       "usb3v1",
                        },
                        {
                                .supply =       "usb3v1",
                        },
		};

	/* First add the regulators so that they can be used by transceiver */
		if (twl_has_regulator()) {
			/* this is a template that gets copied */
			struct regulator_init_data usb_fixed = {
				.constraints.valid_modes_mask =
					REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
				.constraints.valid_ops_mask =
					REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			};

                        usb1v5[1].dev = pdata->usb->sensor_dev;
                        usb1v8[1].dev = pdata->usb->sensor_dev;
                        usb3v1[1].dev = pdata->usb->sensor_dev;
			child = add_regulator_linked(TWL4030_REG_VUSB1V5,
						      &usb_fixed, &usb1v5, 2);
			if (IS_ERR(child))
				return PTR_ERR(child);

			child = add_regulator_linked(TWL4030_REG_VUSB1V8,
						      &usb_fixed, &usb1v8, 2);
			if (IS_ERR(child))
				return PTR_ERR(child);

			child = add_regulator_linked(TWL4030_REG_VUSB3V1,
						      &usb_fixed, &usb3v1, 2);
			if (IS_ERR(child))
				return PTR_ERR(child);

		}

		child = add_child(0, "twl4030_usb",
				pdata->usb, sizeof(*pdata->usb),
				true,
				/* irq0 = USB_PRES, irq1 = USB */
				pdata->irq_base + USB_PRES_INTR_OFFSET,
				pdata->irq_base + USB_INTR_OFFSET);

		if (IS_ERR(child))
			return PTR_ERR(child);

		/* we need to connect regulators to this transceiver */
		if (twl_has_regulator() && child) {
			usb1v5[0].dev = child;
			usb1v8[0].dev = child;
			usb3v1[0].dev = child;
		}
	}
	if (twl_has_usb() && twl_class_is_6030()) {
		child = add_child(0, "twl6030_usb",
		pdata->usb, sizeof(*pdata->usb),
		true,
		/* irq0 = USB_PRES, irq1 = USB */
		pdata->irq_base + USBOTG_INTR_OFFSET,
		pdata->irq_base + USB_PRES_INTR_OFFSET);

	if (IS_ERR(child))
		return PTR_ERR(child);
	}

	if (twl_has_watchdog()) {
		child = add_child(0, "twl4030_wdt", NULL, 0, false, 0, 0);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	if (twl_has_pwrbutton()) {
		child = add_child(1, "twl4030_pwrbutton",
				NULL, 0, true, pdata->irq_base + 8 + 0, 0);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	if (twl_has_codec() && pdata->codec && twl_class_is_4030()) {
		sub_chip_id = twl_map[TWL_MODULE_AUDIO_VOICE].sid;
		child = add_child(sub_chip_id, "twl4030-audio",
				pdata->codec, sizeof(*pdata->codec),
				false, 0, 0);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	/* Phoenix codec driver is probed directly atm */
	if (twl_has_codec() && pdata->codec && twl_class_is_6030()) {
		sub_chip_id = twl_map[TWL_MODULE_AUDIO_VOICE].sid;
		child = add_child(sub_chip_id, "twl6040_audio",
				pdata->codec, sizeof(*pdata->codec),
				false, 0, 0);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	/* twl4030 regulators */
	if (twl_has_regulator() && twl_class_is_4030()) {
		child = add_regulator(TWL4030_REG_VPLL1, pdata->vpll1);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VIO, pdata->vio);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VDD1, pdata->vdd1);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VDD2, pdata->vdd2);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VMMC1, pdata->vmmc1);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VDAC, pdata->vdac);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator((features & TWL4030_VAUX2)
					? TWL4030_REG_VAUX2_4030
					: TWL4030_REG_VAUX2,
				pdata->vaux2);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VINTANA1, pdata->vintana1);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VINTANA2, pdata->vintana2);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VINTDIG, pdata->vintdig);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	/* maybe add LDOs that are omitted on cost-reduced parts */
	if (twl_has_regulator() && !(features & TPS_SUBSET)
	  && twl_class_is_4030()) {
		child = add_regulator(TWL4030_REG_VPLL2, pdata->vpll2);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VMMC2, pdata->vmmc2);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VSIM, pdata->vsim);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VAUX1, pdata->vaux1);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VAUX3, pdata->vaux3);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL4030_REG_VAUX4, pdata->vaux4);
		if (IS_ERR(child))
			return PTR_ERR(child);
			//TEST
		child = add_regulator(TWL6030_REG_CLK32KG, pdata->clk32kg);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	/* twl6030 regulators */
	if (twl_has_regulator() && twl_class_is_6030()) {
		child = add_regulator(TWL6030_REG_VMMC, pdata->vmmc);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL6030_REG_VPP, pdata->vpp);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL6030_REG_VUSIM, pdata->vusim);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL6030_REG_VANA, pdata->vana);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL6030_REG_VCXIO, pdata->vcxio);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL6030_REG_VDAC, pdata->vdac);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL6030_REG_VUSB, pdata->vusb);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL6030_REG_VAUX1_6030, pdata->vaux1);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL6030_REG_VAUX2_6030, pdata->vaux2);
		if (IS_ERR(child))
			return PTR_ERR(child);

		child = add_regulator(TWL6030_REG_VAUX3_6030, pdata->vaux3);
		if (IS_ERR(child))
			return PTR_ERR(child);
	}

	return 0;
}

/*----------------------------------------------------------------------*/

/*
 * These three functions initialize the on-chip clock framework,
 * letting it generate the right frequencies for USB, MADC, and
 * other purposes.
 */
static inline int __init protect_pm_master(void)
{
	int e = 0;

	e = twl_i2c_write_u8(TWL_MODULE_PM_MASTER, KEY_LOCK,
			R_PROTECT_KEY);
	return e;
}

static inline int __init unprotect_pm_master(void)
{
	int e = 0;

	e |= twl_i2c_write_u8(TWL_MODULE_PM_MASTER, KEY_UNLOCK1,
			R_PROTECT_KEY);
	e |= twl_i2c_write_u8(TWL_MODULE_PM_MASTER, KEY_UNLOCK2,
			R_PROTECT_KEY);
	return e;
}

static void clocks_init(struct device *dev,
			struct twl4030_clock_init_data *clock)
{
	int e = 0;
	struct clk *osc;
	u32 rate;
	u8 ctrl = HFCLK_FREQ_26_MHZ;

#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)
	if (cpu_is_omap2430())
		osc = clk_get(dev, "osc_ck");
	else
		osc = clk_get(dev, "osc_sys_ck");

	if (IS_ERR(osc)) {
		printk(KERN_WARNING "Skipping twl internal clock init and "
				"using bootloader value (unknown osc rate)\n");
		return;
	}

	rate = clk_get_rate(osc);
	clk_put(osc);

#else
	/* REVISIT for non-OMAP systems, pass the clock rate from
	 * board init code, using platform_data.
	 */
	osc = ERR_PTR(-EIO);

	printk(KERN_WARNING "Skipping twl internal clock init and "
	       "using bootloader value (unknown osc rate)\n");

	return;
#endif

	switch (rate) {
	case 19200000:
		ctrl = HFCLK_FREQ_19p2_MHZ;
		break;
	case 26000000:
		ctrl = HFCLK_FREQ_26_MHZ;
		break;
	case 38400000:
		ctrl = HFCLK_FREQ_38p4_MHZ;
		break;
	}

	ctrl |= HIGH_PERF_SQ;
	if (clock && clock->ck32k_lowpwr_enable)
		ctrl |= CK32K_LOWPWR_EN;

	e |= unprotect_pm_master();
	/* effect->MADC+USB ck en */
	e |= twl_i2c_write_u8(TWL_MODULE_PM_MASTER, ctrl, R_CFG_BOOT);
	e |= protect_pm_master();

	if (e < 0)
		pr_err("%s: clock init err [%d]\n", DRIVER_NAME, e);

	/* ENABLE TRITON ADC CLOCK */
	{
		u8 val = 0;

		// set GPBR1 register MADC_HFCLK_EN -> 1, DEFAULT_MADC_CLK_EN -> 1 
		twl_i2c_read_u8( TWL4030_MODULE_INTBR, &val, 0x0C);
		twl_i2c_write_u8( TWL4030_MODULE_INTBR, (val | 0x90), 0x0C );
	}
}

/*----------------------------------------------------------------------*/

static int __devexit twl_remove(struct i2c_client *client)
{
	unsigned i;
	int status;

	if (twl_class_is_4030())
		status = twl4030_exit_irq();
	else
		status = twl6030_exit_irq();

	if (status < 0)
		return status;

	for (i = 0; i < TWL_NUM_SLAVES; i++) {
		struct twl_client	*twl = &twl_modules[i];

		if (twl->client && twl->client != client)
			i2c_unregister_device(twl->client);
		twl_modules[i].client = NULL;
	}
	inuse = false;
	return 0;
}

static void _init_twl6030_settings(void)
{
	/* unmask PREQ transition */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0xE0, 0x02);

	/* USB_VBUS_CTRL_CLR */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0xFF, 0x05);
	/* USB_ID_CRTL_CLR */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0xFF, 0x07);

	/* GPADC_CTRL */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x00, 0x2E);
	/* TOGGLE1 */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x51, 0x90);
	/* MISC1 */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00, 0xE4);
	/* MISC2 */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00, 0xE5);

	/*
	 * BBSPOR_CFG - Disable BB charging. It should be
	 * taken care by proper driver
	 */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x62, 0xE6);
	/* CFG_INPUT_PUPD2 */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x65, 0xF1);
	/* CFG_INPUT_PUPD4 */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00, 0xF3);
	/* CFG_LDO_PD2 */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00, 0xF5);
	/* CHARGERUSB_CTRL3 */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x21, 0xEA);
}

/* NOTE:  this driver only handles a single twl4030/tps659x0 chip */
static int __devinit
twl_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int				status;
	unsigned			i;
	struct twl4030_platform_data	*pdata = client->dev.platform_data;
	u8 temp = 0;

	if (!pdata) {
		dev_dbg(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		dev_dbg(&client->dev, "can't talk I2C?\n");
		return -EIO;
	}

	if (inuse) {
		dev_dbg(&client->dev, "driver is already in use\n");
		return -EBUSY;
	}

	for (i = 0; i < TWL_NUM_SLAVES; i++) {
		struct twl_client	*twl = &twl_modules[i];

		twl->address = client->addr + i;

		if (i == 0)
			twl->client = client;
		else {
			twl->client = i2c_new_dummy(client->adapter,
					twl->address);
			if (!twl->client) {
				dev_err(&client->dev,
					"can't attach client %d\n", i);
				status = -ENOMEM;
				goto fail;
			}
		}
		mutex_init(&twl->xfer_lock);
	}
	inuse = true;
	if ((id->driver_data) & TWL6030_CLASS) {
		twl_id = TWL6030_CLASS_ID;
		twl_map = &twl6030_map[0];
	} else {
		twl_id = TWL4030_CLASS_ID;
		twl_map = &twl4030_map[0];
	}

	/* setup clock framework */
	clocks_init(&client->dev, pdata->clock);

	/* load power event scripts */
	if (twl_has_power()) {
		twl4030_power_sr_init();
		 if (pdata->power)
			twl4030_power_init(pdata->power);
	}

	/* Maybe init the T2 Interrupt subsystem */
	if (client->irq
			&& pdata->irq_base
			&& pdata->irq_end > pdata->irq_base) {
		if (twl_class_is_4030()) {
			twl4030_init_chip_irq(id->name);
			status = twl4030_init_irq(client->irq, pdata->irq_base,
			pdata->irq_end);
		} else {
			status = twl6030_init_irq(client->irq, pdata->irq_base,
			pdata->irq_end);
		}

		if (status < 0)
			goto fail;
	}

	/* Disable TWL4030/TWL5030 I2C Pull-up on I2C1 and I2C4(SR) interface.
	 * Program I2C_SCL_CTRL_PU(bit 0)=0, I2C_SDA_CTRL_PU (bit 2)=0,
	 * SR_I2C_SCL_CTRL_PU(bit 4)=0 and SR_I2C_SDA_CTRL_PU(bit 6)=0.
	 */

	if (twl_class_is_4030()) {
		twl_i2c_read_u8(TWL4030_MODULE_INTBR, &temp, REG_GPPUPDCTR1);
		temp &= ~(SR_I2C_SDA_CTRL_PU | SR_I2C_SCL_CTRL_PU | \
		I2C_SDA_CTRL_PU | I2C_SCL_CTRL_PU);
		twl_i2c_write_u8(TWL4030_MODULE_INTBR, temp, REG_GPPUPDCTR1);
	}

	if (twl_class_is_6030()) {
		twl_i2c_write_u8(TWL6030_MODULE_ID0, 0xE1, CLK32KG_CFG_STATE);
		/* Remove unwanted settings on twl chip as part of twl init. */
		_init_twl6030_settings();
	}

	status = add_children(pdata, id->driver_data);
fail:
	if (status < 0)
		twl_remove(client);
	return status;
}

static const struct i2c_device_id twl_ids[] = {
	{ "twl4030", TWL4030_VAUX2 },	/* "Triton 2" */
	{ "twl5030", 0 },		/* T2 updated */
	{ "twl5031", TWL5031 },		/* TWL5030 updated */
	{ "tps65950", 0 },		/* catalog version of twl5030 */
	{ "tps65930", TPS_SUBSET },	/* fewer LDOs and DACs; no charger */
	{ "tps65920", TPS_SUBSET },	/* fewer LDOs; no codec or charger */
	{ "twl6030", TWL6030_CLASS },	/* "Phoenix power chip" */
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(i2c, twl_ids);

/* One Client Driver , 4 Clients */
static struct i2c_driver twl_driver = {
	.driver.name	= DRIVER_NAME,
	.id_table	= twl_ids,
	.probe		= twl_probe,
	.remove		= __devexit_p(twl_remove),
};

static int __init twl_init(void)
{
	return i2c_add_driver(&twl_driver);
}
subsys_initcall(twl_init);

static void __exit twl_exit(void)
{
	i2c_del_driver(&twl_driver);
}
module_exit(twl_exit);

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("I2C Core interface for TWL");
MODULE_LICENSE("GPL");
