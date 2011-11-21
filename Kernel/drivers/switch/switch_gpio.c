/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/i2c/twl4030-madc.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <mach/hardware.h>
#include <plat/mux.h>
#include "./switch_omap_gpio.h"

//#define CONFIG_DEBUG_SEC_HEADSET
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
#define DISTINCTION_3POLE_4POLE_USE_SENDEND
#endif

#ifdef CONFIG_DEBUG_SEC_HEADSET
#define SEC_HEADSET_DBG(fmt, arg...) printk(KERN_INFO "[HEADSET] %s () " fmt "\r\n", __func__, ## arg)
#else
#define SEC_HEADSET_DBG(fmt, arg...) do {} while(0)
#endif

#define HEADSET_ATTACH_COUNT		3
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
#define HEADSET_DETACH_COUNT		5
#else
#define HEADSET_DETACH_COUNT		2
#endif
#define HEADSET_CHECK_TIME			get_jiffies_64() + (HZ/20)// 1000ms / 20 = 50ms
#define SEND_END_ENABLE_TIME 		get_jiffies_64() + (HZ*2)// 1000ms * 2 = 2sec
#define WAKE_LOCK_TIME 			(HZ*2)// 1000ms  = 2sec

struct gpio_switch_data	*data = NULL;
static unsigned short int headset_status;
static struct timer_list headset_detect_timer;
static unsigned int headset_detect_timer_token;
static struct wake_lock headset_wake_lock;

//for regulator control for getting adc value
#define VINTANA2_DEV_GRP   0x43
#define VINTANA2_DEDICATED 0x46
#define VSEL_VINTANA2_2V75 0x01

#define VUSB1V8_DEV_GRP     0x74
#define VUSB3V1_DEV_GRP     0x77
#define CARKIT_ANA_CTRL     0xBB
#define SEL_MADC_MCPC       0x08

extern int twl4030_get_voicecall_state(void);

struct switch_device_info
{
        struct device *dev;
};

static struct device *this_dev;

#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
extern void ear_key_disable_irq(void);
extern void ear_key_enable_irq(void);
extern void release_ear_key(void);
#endif

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
};

short int get_headset_status(void)
{
	SEC_HEADSET_DBG(" headset_status %d", headset_status);
	return headset_status;
}
EXPORT_SYMBOL(get_headset_status);

static void release_headset_event(struct work_struct *work)
{
	printk("[Headset]Headset attached\n");
	headset_status = 1;
	switch_set_state(&data->sdev, 1);
}
static DECLARE_DELAYED_WORK(release_headset_event_work, release_headset_event);

#ifdef DISTINCTION_3POLE_4POLE_USE_SENDEND
static int get_t2adc_data( int ch )
{
    int ret = 0;
    int sendend_state = 0;

#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
	sendend_state = gpio_get_value(OMAP_GPIO_EAR_ADC_3_5);
#else	
	sendend_state = gpio_get_value(EAR_KEY_GPIO);
#endif	
	
	printk("3pole headset detecing use sendend %d\n", sendend_state);
	
	if(sendend_state)
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )	
		return 171;
#else		
		return 101;
#endif		
	else
		return 0;
}
EXPORT_SYMBOL_GPL(get_t2adc_data);
#else
static int get_t2adc_data( int ch )
{
    int ret = 0;
    struct twl4030_madc_request req;
    
    req.channels = ( 1 << ch );
    req.do_avg = 0;
    req.method = TWL4030_MADC_SW1;
    req.active = 0;
    req.func_cb = NULL;
    twl4030_madc_conversion( &req );

    ret = req.rbuf[ch];
	printk("adc value is : %d\n", ret);

    return ret;
}
EXPORT_SYMBOL_GPL(get_t2adc_data);
#endif

static void ear_adc_caculrator(struct work_struct *work)
{
	int adc = 0;
	int state = 0;
	
	del_timer(&headset_detect_timer);
	
	state = gpio_get_value(EAR_DET_GPIO) ^ EAR_DETECT_INVERT_ENABLE;	

	wake_lock(&headset_wake_lock);
	
	#ifdef USE_ADC_SEL_GPIO
	gpio_direction_output(EAR_ADC_SEL_GPIO , 1);
	#endif
	gpio_direction_output(EAR_MIC_BIAS_GPIO, 1);
	msleep(50);
	
	if (state)
	{
		if(headset_status != HEADSET_DISCONNET){
			printk("[Headset] Headset is not disconnect before attach\n");
			switch_set_state(&data->sdev,HEADSET_DISCONNET);
			headset_status = HEADSET_DISCONNET;
		}
		adc = get_t2adc_data(2);

		#ifndef USE_ONLY_AS_4POLE
#if ( defined( CONFIG_MACH_SAMSUNG_P1WIFI ) )
		if(adc > 170)
#else		
		if(adc > 230)
#endif		
		#endif
		{	
			switch_set_state(&data->sdev,HEADSET_4POLE_WITH_MIC);
			headset_status = HEADSET_4POLE_WITH_MIC;
		
			printk("[Headset] 4pole ear-mic adc is %d\n", adc);
			#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
			ear_key_enable_irq();
			#endif
		}
		#ifndef USE_ONLY_AS_4POLE
		else 
		{
			switch_set_state(&data->sdev, HEADSET_3POLE);
			headset_status = HEADSET_3POLE;

			printk("[Headset] 3pole earphone adc is %d\n", adc);
			#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
			ear_key_disable_irq();
			#endif
			gpio_direction_output(EAR_MIC_BIAS_GPIO, 0);
			#ifdef USE_ADC_SEL_GPIO
			gpio_direction_output(EAR_ADC_SEL_GPIO , 0);
			#endif
		}	
		#endif
	}
	else
	{
		printk("[Headset] Headset detached\n");        	
		switch_set_state(&data->sdev, state);

		headset_status = HEADSET_DISCONNET;
		#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
		ear_key_disable_irq();
		#endif	

		//[for sleep current when insert Headset
		twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_OFF , TWL4030_VINTANA2_REMAP );
		twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_OFF , TWL4030_VUSB3V1_REMAP ); 
		//]
		
		gpio_direction_output(EAR_MIC_BIAS_GPIO, 0);
		#ifdef USE_ADC_SEL_GPIO
		gpio_direction_output(EAR_ADC_SEL_GPIO , 0);
		#endif
	}
	wake_unlock(&headset_wake_lock);
	wake_lock_timeout(&headset_wake_lock, WAKE_LOCK_TIME);
}
static DECLARE_DELAYED_WORK(ear_adc_cal_work, ear_adc_caculrator);

static void headset_detect_timer_handler(unsigned long arg)
{
	int state;
	int count = 0;
	del_timer(&headset_detect_timer);
	state = gpio_get_value(EAR_DET_GPIO) ^ EAR_DETECT_INVERT_ENABLE;

	if(state)
		count = HEADSET_ATTACH_COUNT;
	else if(!state)
		count = HEADSET_DETACH_COUNT;
		
	SEC_HEADSET_DBG("Check attach state - headset_detect_timer_token is %d", headset_detect_timer_token);
	
	if(headset_detect_timer_token < count)
	{
		headset_detect_timer.expires = HEADSET_CHECK_TIME;
		add_timer(&headset_detect_timer);
		headset_detect_timer_token++;
	}
	else if(headset_detect_timer_token == count)
	{
		cancel_delayed_work_sync(&ear_adc_cal_work);
		schedule_delayed_work(&ear_adc_cal_work, 20);
		SEC_HEADSET_DBG("add work queue - timer token is %d", count);
		headset_detect_timer_token = 0;
	}
	else
	{
		printk(KERN_ALERT "wrong headset_detect_timer_token count %d", headset_detect_timer_token);
		gpio_direction_output(EAR_MIC_BIAS_GPIO, 0);
	}
}

static void gpio_switch_work(struct work_struct *work)
{
	int state;
	SEC_HEADSET_DBG("");  
	//struct gpio_switch_data	*data =
	//container_of(work, struct gpio_switch_data, work);
	del_timer(&headset_detect_timer);
	cancel_delayed_work_sync(&ear_adc_cal_work);
	
	state = gpio_get_value(data->gpio) ^ EAR_DETECT_INVERT_ENABLE;

	if (state || !state)
	{
		//wake_lock(&headset_sendend_wake_lock);
		SEC_HEADSET_DBG("Headset attached timer start");
		headset_detect_timer_token = 0;
		headset_detect_timer.expires = HEADSET_CHECK_TIME;
		add_timer(&headset_detect_timer);
	}

	else
		SEC_HEADSET_DBG("Headset state does not valid. or send_end event");
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;

	#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
	int state;
	SEC_HEADSET_DBG("");  
	state = gpio_get_value(data->gpio) ^ EAR_DETECT_INVERT_ENABLE;
	 if (!state)
	{
		ear_key_disable_irq();	
	}
 	#endif
	wake_lock_timeout(&headset_wake_lock, WAKE_LOCK_TIME);
	gpio_direction_output(EAR_MIC_BIAS_GPIO, 0); //for detach noise
	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;
	SEC_HEADSET_DBG("");  
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int switch_gpio_suspend(struct platform_device *device, pm_message_t state)
{

	if(headset_status == HEADSET_4POLE_WITH_MIC)
	{
		printk("[HEADSET] 4POLE_HEADSET enable VINTANA2, USB3V1");
		twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_ACTIVE , TWL4030_VINTANA2_REMAP );
		twl_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, REMAP_ACTIVE , TWL4030_VUSB3V1_REMAP ); 
	}

	return 0;
}
static int swtich_gpio_resume(struct platform_device *device)
{
	return 0;
}

static int gpio_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int ret = 0;  
  
	SEC_HEADSET_DBG("");  
	this_dev = &pdev->dev;

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_gpio_print_state;

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = gpio_request(switch_data->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	INIT_WORK(&switch_data->work, gpio_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(switch_data->irq, gpio_irq_handler,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  pdev->name, 
			  switch_data); //Iyyappan_HS
			  
		
	if (ret < 0)
		goto err_request_irq;

	data = switch_data;	

	if (gpio_request(EAR_MIC_BIAS_GPIO , "EAR_MICBIAS_EN") == 0) 
	{
		gpio_direction_output(EAR_MIC_BIAS_GPIO , 0);
	}
	
	#ifdef USE_ADC_SEL_GPIO
	if (gpio_request(EAR_ADC_SEL_GPIO , "EAR_ADC_SEL") == 0) 
	{
		gpio_direction_output(EAR_ADC_SEL_GPIO , 0);
	}
	#endif
	
	
	init_timer(&headset_detect_timer);
	headset_detect_timer.function = headset_detect_timer_handler;

	wake_lock_init(&headset_wake_lock, WAKE_LOCK_SUSPEND, "headset");

	enable_irq_wake(switch_data->irq);

	/* Perform initial detection */
	gpio_switch_work(&switch_data->work);

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);
	SEC_HEADSET_DBG("");
	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= __devexit_p(gpio_switch_remove),
	.suspend	= switch_gpio_suspend,
	.resume		= swtich_gpio_resume,
	.driver		= {
		.name	= "switch-gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init gpio_switch_init(void)
{
	#ifdef USE_ADC_SEL_GPIO
	gpio_free(EAR_ADC_SEL_GPIO );
	#endif
	gpio_free(EAR_MIC_BIAS_GPIO);
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

#ifndef MODULE
late_initcall(gpio_switch_init);
#else
module_init(gpio_switch_init);
#endif
module_exit(gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
