/*
 *  drivers/switch/switch_sio.c
 *
 * Copyright (C) 2009 Samsung Electronics Co. Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/kdev_t.h>
#include <plat/mux.h>
#include <mach/hardware.h>
#include <mach/sec_param.h>
#if defined(CONFIG_USB_ANDROID)
#include <linux/usb/android_composite.h>

#undef PARAM_CONSOLE_MODE
#endif


#ifdef CONFIG_FSA9480_MICROUSB
extern void mcirousb_usbpath_change(int usb_path);
#endif
#include <plat/microusbic.h>

extern int get_real_usbic_state(void);

#ifdef CONFIG_KEYBOARD_P1
extern bool keyboard_enable;
#endif

extern struct class *sec_class;
extern void (*sec_set_param_value)(int idx, void *value);
extern void (*sec_get_param_value)(int idx, void *value);


typedef enum
{
	USB_SW_AP,
	USB_SW_CP
} USB_SWITCH_MODE;

typedef enum
{
	UART_SW_AP,
	UART_SW_CP
} UART_SWITCH_MODE;

typedef enum
{
	AP_USB_MODE,
	AP_UART_MODE,
	CP_USB_MODE,
	CP_UART_MODE,
} USB_UART_SW_MODE_TYPE;

/* 1 : PDA, 2 : MODEM */
#define SWITCH_PDA			1
#define SWITCH_MODEM		2

#define USBSTATE_TA_CHARGER		3
#define USBSTATE_USB_CHARGER		2
#define USBSTATE_USB_CABLE		1
#define USBSTATE_NO_DEVICE		0
#define USBSTATE_PHONE_USB		-1


#ifndef GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW   0
#endif
#ifndef GPIO_LEVEL_HIGH
#define GPIO_LEVEL_HIGH  1
#endif

#define USB_SEL_MASK    (1 << 0)
#define UART_SEL_MASK   (1 << 1)

static int usb_path = SWITCH_PDA;
static int uart_current_owner = SWITCH_MODEM;

struct delayed_work sio_switch_init_work;

static void sio_switch_gpio_init(void)
{
    /*do not gpio init for prevent lockup when boot up.*/

	if (gpio_request(OMAP_GPIO_UART_SEL, "UART_SEL"))
	{
		printk(KERN_ERR "Filed to request OMAP_GPIO_UART_SEL!\n");
	}
	gpio_direction_output(OMAP_GPIO_UART_SEL, GPIO_LEVEL_LOW);

	if (gpio_request(OMAP_GPIO_CP_VBUS_EN, "USB_SEL"))
	{
		printk(KERN_ERR "Filed to request OMAP_GPIO_CP_VBUS_EN!\n");
	}
	gpio_direction_output(OMAP_GPIO_CP_VBUS_EN, GPIO_LEVEL_LOW);

}

static void usb_api_set_usb_switch(USB_SWITCH_MODE usb_switch)
{
	if(usb_switch == USB_SW_CP)
	{
		//USB_SEL GPIO Set High => CP USB enable
		gpio_set_value(OMAP_GPIO_CP_VBUS_EN, GPIO_LEVEL_HIGH);
#ifdef CONFIG_FSA9480_MICROUSB
		mcirousb_usbpath_change(1);
#endif
		usb_path = SWITCH_MODEM;
	}
	else
	{
		//USB_SEL GPIO Set Low => AP USB enable
		gpio_set_value(OMAP_GPIO_CP_VBUS_EN, GPIO_LEVEL_LOW);
#ifdef CONFIG_FSA9480_MICROUSB
		mcirousb_usbpath_change(0);
#endif
		usb_path = SWITCH_PDA;
	}
}

static void sio_switch_config(USB_UART_SW_MODE_TYPE sio_mode)
{
	switch (sio_mode)
	{
		case AP_USB_MODE:
			usb_api_set_usb_switch(USB_SW_AP);
			break;
		case CP_USB_MODE:
			usb_api_set_usb_switch(USB_SW_CP);
			break;
		case AP_UART_MODE:
			gpio_set_value(OMAP_GPIO_UART_SEL, GPIO_LEVEL_HIGH);
			break;
		case CP_UART_MODE:
#ifdef CONFIG_KEYBOARD_P1
			if(!keyboard_enable)
			{
			gpio_set_value(OMAP_GPIO_UART_SEL, GPIO_LEVEL_LOW);
			}
#else
			gpio_set_value(OMAP_GPIO_UART_SEL, GPIO_LEVEL_LOW);
#endif
			break;
		default:
			printk("sio_switch_config error");
			break;
	}
}

/* for sysfs control (/sys/class/sec/switch/usb_sel) */
static ssize_t usb_sel_show
(
	struct device *dev,
	struct device_attribute *attr,
	char *buf
)
{
	return snprintf(buf, PAGE_SIZE, "USB Switch : %s\n", (usb_path == SWITCH_PDA) ? "PDA" : "MODEM");
}

static ssize_t usb_sel_store
(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t size
)
{
	int switch_sel;
	int path_save = 1;

	if (sec_get_param_value)
	{
		sec_get_param_value(__SWITCH_SEL, &switch_sel);
	}

	if(strstr(buf, "PDA") || strstr(buf, "pda"))
	{
		if(usb_path != SWITCH_PDA)
		{
			sio_switch_config(AP_USB_MODE);
		}
		usb_path = SWITCH_PDA;
		switch_sel |= USB_SEL_MASK;
		printk("[USB Switch] Path : PDA\n");
	}
	else if(strstr(buf, "MODEM") || strstr(buf, "modem"))
	{
		if(usb_path != SWITCH_MODEM)
		{
			sio_switch_config(CP_USB_MODE);
		}
		usb_path = SWITCH_MODEM;
		switch_sel &= ~USB_SEL_MASK;
		printk("[USB Switch] Path : MODEM\n");
	}

	if(strstr(buf, "NOSAVE") || strstr(buf, "nosave"))
	{
		path_save = 0;
		printk("[USB Switch] path is not saved\n");
	}

	if(path_save)
	{
		if (sec_set_param_value)
		{
			sec_set_param_value(__SWITCH_SEL, &switch_sel);
		}
	}

	return size;
}

/* for sysfs control (/sys/class/sec/switch/uart_sel) */
static ssize_t uart_switch_show
(
	struct device *dev,
	struct device_attribute *attr,
	char *buf
)
{
	if ( uart_current_owner == SWITCH_PDA )
	{
		return snprintf(buf, PAGE_SIZE, "[UART Switch] Current UART owner = PDA\n");	
	}
	else if ( uart_current_owner == SWITCH_MODEM )
	{
		return snprintf(buf, PAGE_SIZE, "[UART Switch] Current UART owner = MODEM\n");
	}

	return 0;
}

static ssize_t uart_switch_store
(
	struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t size
)
{
	int switch_sel;
#ifdef PARAM_CONSOLE_MODE
	int console_mode;
#endif
	int path_save = 1;

	if (sec_get_param_value)
	{
		sec_get_param_value(__SWITCH_SEL, &switch_sel);
#ifdef PARAM_CONSOLE_MODE
		sec_get_param_value(__CONSOLE_MODE, &console_mode);
#endif
	}

	if (strstr(buf, "PDA") || strstr(buf, "pda"))
	{
		if(uart_current_owner != SWITCH_PDA)
		{
			sio_switch_config(AP_UART_MODE);
		}
		uart_current_owner = SWITCH_PDA;
		switch_sel |= UART_SEL_MASK;
#ifdef PARAM_CONSOLE_MODE
		console_mode = 1;
#endif
		printk("[UART Switch] Path : PDA\n");
	}
	else if (strstr(buf, "MODEM") || strstr(buf, "modem"))
	{
		if(uart_current_owner != SWITCH_MODEM)
		{
			sio_switch_config(CP_UART_MODE);
		}
		uart_current_owner = SWITCH_MODEM;
		switch_sel &= ~UART_SEL_MASK;
#ifdef PARAM_CONSOLE_MODE
		console_mode = 0;
#endif
		printk("[UART Switch] Path : MODEM\n");
	}

	if(strstr(buf, "NOSAVE") || strstr(buf, "nosave"))
	{
		path_save = 0;
		printk("[UART Switch] path is not saved\n");
	}

	if(path_save)
	{
		if (sec_set_param_value)
		{
			sec_set_param_value(__SWITCH_SEL, &switch_sel);
#ifdef PARAM_CONSOLE_MODE
			sec_set_param_value(__CONSOLE_MODE, &console_mode);
#endif
		}
	}

	return size;
}

static ssize_t usb_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_FSA9480_MICROUSB
	int cable_state = get_real_usbic_state();
	//  int cable_state = get_usbic_state();
#endif

	sprintf(buf, "%s\n", (cable_state == MICROUSBIC_USB_CABLE)?"USB_STATE_CONFIGURED":"USB_STATE_NOTCONFIGURED");

	return sprintf(buf, "%s\n", buf);
} 

static ssize_t usb_state_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	printk("[USBMENU]%s\n ", __func__);
	return 0;
}

/*sysfs for usb cable's state.*/
static DEVICE_ATTR(usb_state, 0664, usb_state_show, usb_state_store);
static DEVICE_ATTR(usb_sel, 0664, usb_sel_show, usb_sel_store);
static DEVICE_ATTR(uart_sel, 0664, uart_switch_show, uart_switch_store);

static struct attribute *switch_sio_attributes[] = {
	&dev_attr_usb_state.attr,
	&dev_attr_usb_sel.attr,
	&dev_attr_uart_sel.attr,
	NULL
};

static const struct attribute_group switch_sio_group = {
	.attrs = switch_sio_attributes,
};
static void sio_switch_init_worker(struct work_struct *ignored)
{
	int switch_sel;

	if (sec_get_param_value) {
		sec_get_param_value(__SWITCH_SEL, &switch_sel);
		cancel_delayed_work(&sio_switch_init_work);

	} else {
		schedule_delayed_work(&sio_switch_init_work, msecs_to_jiffies(100));		
		return;
	}

	if (switch_sel & USB_SEL_MASK)
		{
		usb_path = SWITCH_PDA;
		sio_switch_config(AP_USB_MODE);
		}
	else
		usb_path = SWITCH_MODEM;

	if (switch_sel & UART_SEL_MASK)
		uart_current_owner = SWITCH_PDA;
	else
		uart_current_owner = SWITCH_MODEM;

	if (uart_current_owner == SWITCH_PDA)
	{
		sio_switch_config(AP_UART_MODE);
	}
	else if (uart_current_owner == SWITCH_MODEM)
	{
		printk("----------------- Cutting off PDA UART ---------------------\n");
		sio_switch_config(CP_UART_MODE);
	}

	if (usb_path == SWITCH_MODEM)
	{
		sio_switch_config(CP_USB_MODE);
	}
}

static int sio_switch_probe(struct platform_device *pdev)
{
	int ret = 0;

	sio_switch_gpio_init();

	INIT_DELAYED_WORK(&sio_switch_init_work, sio_switch_init_worker);
	schedule_delayed_work(&sio_switch_init_work, msecs_to_jiffies(200));

	printk("[%s]: initialized\n", pdev->name);

	ret = sysfs_create_group(&pdev->dev.kobj, &switch_sio_group);
	if (ret) {
		dev_err(&pdev,
				"failed to create switch_sio attribute group\n");
		goto probe_err;
	}

	return 0;

probe_err:

	return ret;
}

static int __devexit sio_switch_remove(struct platform_device *pdev)
{
    sysfs_remove_group(&pdev->dev.kobj, &switch_sio_group);

	return 0;
}

static struct platform_driver sio_switch_driver = {
	.probe		= sio_switch_probe,
	.remove		= __devexit_p(sio_switch_remove),
	.driver		= {
		.name	= "switch-sio",
	},
};

static int __init sio_switch_init(void)
{
	return platform_driver_register(&sio_switch_driver);
}

static void __exit sio_switch_exit(void)
{
	platform_driver_unregister(&sio_switch_driver);
}

module_init(sio_switch_init);
module_exit(sio_switch_exit);

MODULE_AUTHOR("SAMSUNG ELECTRONICS CO., LTD");
MODULE_DESCRIPTION("SIO Switch driver");
MODULE_LICENSE("GPL");
