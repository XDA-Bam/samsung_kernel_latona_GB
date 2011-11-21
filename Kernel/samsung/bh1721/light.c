/*
 *  bh1721fvc.c - Ambient Light Sensor IC
 *
 *  Copyright (C) 2009 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#define NUM_OF_BYTES_WRITE   1
#define NUM_OF_BYTES_READ    2

#define BH1721_ON   1
#define BH1721_OFF  0

#define MAX_LEVEL     5
#define MAX_LUX       65528
 
#define BH1721_DEFAULT_DELAY    500     // 500 ms

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

const unsigned char POWER_DOWN = 0x00;
const unsigned char POWER_ON = 0x01;
const unsigned char AUTO_RESOLUTION_1 = 0x10;
const unsigned char AUTO_RESOLUTION_2 = 0x20;
const unsigned char H_RESOLUTION_1 = 0x12;
const unsigned char H_RESOLUTION_2 = 0x22;
const unsigned char L_RESOLUTION_1 = 0x13;
const unsigned char L_RESOLUTION_2 = 0x23;
const unsigned char L_RESOLUTION_3 = 0x16;
const unsigned char L_RESOLUTION_4 = 0x26;

struct bh1721_data {
    unsigned char illuminance_data[2];
    struct input_dev *input;
    struct i2c_client *client;
    struct delayed_work work;
    atomic_t enable;                /* attribute value */
    atomic_t delay;                 /* attribute value */
    int last_brightness_step;
    int testmode;
};

static struct bh1721_data *bh1721;
static struct workqueue_struct* work_queue;


/* extern functions to turn on sensor via regulator */
extern int bh1721_sensor_power_on( void );
extern int bh1721_sensor_power_off( void );

extern int  bh1721_power_init( void );
extern void bh1721_power_exit( void );

static int brightness_step_table[] = { 0, 15, 50, 160, 510, 1600, 3800, 5400, 10000000 };

static int 
bh1721_write_command(struct i2c_client *client, const char *command)
{
    return i2c_master_send(client, command, NUM_OF_BYTES_WRITE);
}

static int 
bh1721_read_value(struct i2c_client *client, char *buf)
{
    return i2c_master_recv(client, buf, NUM_OF_BYTES_READ);
}

static int bh1721_reset(void)
{
    gpio_set_value(OMAP_GPIO_ALS_NRST, 0);
    msleep(10);
    gpio_set_value(OMAP_GPIO_ALS_NRST, 1);
    return 0;
}

static int bh1721_hw_init(void)
{
	  gpio_free(OMAP_GPIO_ALS_NRST);
    if(gpio_is_valid(OMAP_GPIO_ALS_NRST))
    {
        if(gpio_request(OMAP_GPIO_ALS_NRST, NULL))
        {
            printk(KERN_ERR "Failed to request OMAP_GPIO_ALS_NRST\n");
            return -1;
        }
        gpio_direction_output(OMAP_GPIO_ALS_NRST, 1);
    }
    return 0;
}

static void bh1721_power_up(struct bh1721_data *data)
{
    gpio_set_value(OMAP_GPIO_ALS_NRST, 0);
    msleep(1);
    
    bh1721_sensor_power_on();
    msleep(3);
    
    gpio_set_value(OMAP_GPIO_ALS_NRST, 1);
    
    /* POWER BH1721_ON and set H-resolution mode */
    if((bh1721_write_command(bh1721->client, &POWER_ON)) > 0)
        atomic_set(&bh1721->enable, BH1721_ON);
    else
        printk(KERN_ERR "[BH1721][%s] Failed to write POWER_ON command!\n", __func__);
        
}

static void bh1721_power_down(struct bh1721_data *data)
{
    if((bh1721_write_command(bh1721->client, &POWER_DOWN)) > 0)
        atomic_set(&bh1721->enable, BH1721_OFF);
    else
        printk(KERN_ERR "[BH1721][%s] Failed to write POWER_OFF command!\n", __func__);

    bh1721_sensor_power_off();
    msleep(1);
}

static unsigned int bh1721_measure(void)
{
    unsigned int result;

    bh1721_write_command(bh1721->client, &H_RESOLUTION_2);

    /* Maximum measurement time for H-resolution mode*/
    msleep(180);
    
    if(bh1721_read_value(bh1721->client, bh1721->illuminance_data) <= 0)
        printk(KERN_ERR "[BH1721][%s] Failed to read illuminance!\n", __func__);
    

    result = bh1721->illuminance_data[0] << 8 | bh1721->illuminance_data[1];
    result = (result*10)/12;
    
    //printk("[BH1721] RAW[0x%x][0x%x] - LUX[%d]\n", bh1721->illuminance_data[0], bh1721->illuminance_data[1], result);
    return result;
}

static void bh1721_set_enable(struct device *dev, int enable)
{
    int delay = atomic_read(&bh1721->delay);
    printk("\n bh1721_set_enable \n");
    if (enable) {                   /* enable if state will be changed */
		printk("\n bh1721_set_enable1 \n");
        if (!atomic_cmpxchg(&bh1721->enable, 0, 1)) {
		printk("\n bh1721_set_enable2 \n");
            bh1721_power_up(bh1721);
            queue_delayed_work(work_queue, &bh1721->work, delay_to_jiffies(500) + 1);
        }
    } else {                        /* disable if state will be changed */
    		printk("\n bh1721_set_enable3 \n");
        if (atomic_cmpxchg(&bh1721->enable, 1, 0)) {
		printk("\n bh1721_set_enable4 \n");
            cancel_delayed_work_sync(&bh1721->work);
            bh1721_power_down(bh1721);
        }
    }
    
    atomic_set(&bh1721->enable, enable);
}

static void bh1721_set_delay(struct device *dev, int delay)
{
    atomic_set(&bh1721->delay, delay);

    if (atomic_read(&bh1721->enable)) {
        cancel_delayed_work_sync(&bh1721->work);
        queue_delayed_work(work_queue, &bh1721->work, delay_to_jiffies(500) + 1);
    }
}

/*
 * I/F for sysfs
 */
static ssize_t bh1721_delay_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bh1721_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t bh1721_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bh1721_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t bh1721_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t bh1721_data_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bh1721_status_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bh1721_lux_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bh1721_testmode_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t bh1721_testmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP, bh1721_delay_show, bh1721_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, bh1721_enable_show, bh1721_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, bh1721_wake_store);
static DEVICE_ATTR(data, S_IRUGO, bh1721_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, bh1721_status_show, NULL);
//static DEVICE_ATTR(lux, S_IRUGO|S_IWUSR, bh1721_lux_show, NULL);
static DEVICE_ATTR(testmode, 0664, bh1721_testmode_show, bh1721_testmode_store);

static struct device_attribute dev_attr_light_sensor_lux = 
	__ATTR(lux, S_IRUSR | S_IRGRP, bh1721_lux_show, NULL);


static struct attribute *bh1721_attributes[] = {
    &dev_attr_poll_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,
    &dev_attr_status.attr,
    &dev_attr_testmode.attr,
    NULL
};

/* 
 * sysfs attributes for additional functions.
 * this interfaces will be located under /sys/class/sensors/xxx
 */
static struct device_attribute *additional_light_attrs[] = {
	&dev_attr_light_sensor_lux,
	NULL,
};

static struct attribute_group bh1721_attribute_group = {
    .attrs = bh1721_attributes
};

int bh1721_sysfs_init(struct input_dev * input_data)
{
    int ret = 0;

    ret = sysfs_create_group(&input_data->dev.kobj, &bh1721_attribute_group);
    if (ret) {
        printk(KERN_ERR "bh1721_sysfs_init: sysfs_create_group failed[%s]\n", input_data->name);
    }

    return ret;
}	

void bh1721_sysfs_exit(struct input_dev * input_data)
{
    sysfs_remove_group(&input_data->dev.kobj, &bh1721_attribute_group);
}	


static ssize_t bh1721_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{    
    return sprintf(buf, "%d\n", bh1721_measure());
}


static ssize_t bh1721_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
    int x;

    spin_lock_irqsave(&input_data->event_lock, flags);

    x = input_data->abs[ABS_MISC];

    spin_unlock_irqrestore(&input_data->event_lock, flags);

    return sprintf(buf, "%d\n", x);
}

static ssize_t bh1721_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t ret;
    ret = sprintf(buf, "%d\n", atomic_read(&bh1721->enable));
    return ret;
}

static ssize_t bh1721_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int mode = 0;
    struct input_dev *input_data = to_input_dev(dev);

    sscanf(buf,"%d",&mode);    
    
    bh1721_set_enable(dev, mode);
    
    input_report_abs(input_data, ABS_THROTTLE, (mode<<16) | atomic_read(&bh1721->delay));
	
    return count;
}

static ssize_t bh1721_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", atomic_read(&bh1721->delay));
}

static ssize_t bh1721_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    ssize_t ret = strlen(buf);
    unsigned long delay;
    struct input_dev *input_data = to_input_dev(dev);
    
    sscanf(buf, "%lu", &delay);
    atomic_set(&bh1721->delay, delay);

    input_report_abs(input_data, ABS_THROTTLE, (atomic_read(&bh1721->enable)<<16) | delay);

    return ret;

}

static ssize_t bh1721_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    static int cnt = 1;
    struct input_dev *input_data = to_input_dev(dev);

    input_report_abs(input_data, ABS_X, cnt++);

    return count;
}

static ssize_t bh1721_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned long flags;
    int status;
    struct input_dev *input_data = to_input_dev(dev);

    spin_lock_irqsave(&input_data->event_lock, flags);

    status = input_data->abs[ABS_BRAKE];

    spin_unlock_irqrestore(&input_data->event_lock, flags);

    return sprintf(buf, "%d\n", status);
}

static ssize_t bh1721_testmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int value = simple_strtoul(buf, NULL,10);
    bh1721->testmode = value;
    return count;
}

static ssize_t bh1721_testmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", bh1721->testmode);
}

static void bh1721_work_func(struct work_struct *work)
{
    struct bh1721_data *data = container_of((struct delayed_work *)work,
                                             struct bh1721_data, work);
                                             
    unsigned long delay = delay_to_jiffies(500);
    unsigned int lux = bh1721_measure();
    int i = 0;
    int step = 0;
    int final = sizeof(brightness_step_table)/sizeof(int);

    //printk("\n bh1721_work_func delay=%d \n",delay);
    
    for(; i < final - 1; i++)
    {
        if((lux >= brightness_step_table[i]) && (lux < brightness_step_table[i+1]))
        {
            step = i;
            break;
        }
    }
    
    if(bh1721->testmode == 1)
    {
        input_report_abs(bh1721->input, ABS_MISC, lux);
        input_sync(bh1721->input);
    }
    else
    {
        if(bh1721->last_brightness_step != step)
        {
            input_report_abs(bh1721->input, ABS_MISC, lux);
            input_sync(bh1721->input);
            bh1721->last_brightness_step = step;
	     printk("\n bh1721_work_func lux :%d \n",lux);		
        }
    }

    //schedule_delayed_work(&bh1721->work, delay);
    queue_delayed_work(work_queue, &bh1721->work, delay);
}

static struct device *light_sensor_device;
extern struct class *sensors_class;
extern int sensors_register(struct device *dev, void * drvdata, struct device_attribute *attributes[], char *name);

static int bh1721_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{    
    int ret = 0;
    
    printk("[BH1721] %s started......", __func__);
    
    /* initialize GPIO first */
    if(bh1721_hw_init())
    {
        printk(KERN_ERR "[BH1721][%s] Failed to initialize BH1721 light sensor!\n", __func__);
        return -ENXIO;
    }
    
    /* allocate memory to store the informations of the sensor */
    bh1721 = kzalloc(sizeof(struct bh1721_data), GFP_KERNEL);
    if (!bh1721)
    {
        printk(KERN_ERR "[BH1721][%s] Failed to allocate memory!\n", __func__);
        return -ENOMEM;
    }
    
    /* enable power from PMIC */
    if(bh1721_power_init())
    {
        printk(KERN_ERR "BH1721][%s] Failed to initialize power!\n", __func__);
        goto error_power_init;
    }
    
    bh1721->client = client;
    i2c_set_clientdata(client, bh1721);

	if( !(bh1721->input = input_allocate_device()) )
	{
		ret = -ENOMEM;
        goto error_input_alloc;
	}

    /* initialize input device */
    set_bit(EV_ABS, bh1721->input->evbit);
    input_set_capability(bh1721->input, EV_ABS, ABS_MISC);
    input_set_capability(bh1721->input, EV_ABS, ABS_BRAKE); 	/* status */
    input_set_capability(bh1721->input, EV_ABS, ABS_X); 		/* wake */
    input_set_capability(bh1721->input, EV_ABS, ABS_THROTTLE); 	/* enabled/delay */
	bh1721->input->name = "light_sensor"; 

	if((ret = input_register_device(bh1721->input)) < 0) 
    {
        printk(KERN_ERR "[BH1721][%s] Failed to register as input device!\n", __func__);
        goto error_input_init;
    }
    
    bh1721_set_delay(NULL, BH1721_DEFAULT_DELAY);

    /* setup driver interfaces */
    INIT_DELAYED_WORK(&bh1721->work, bh1721_work_func);
    work_queue = create_singlethread_workqueue( "bh1721_work_queue" ) ;
    if(work_queue == NULL)
    {
        printk(KERN_ERR "[BH1721][%s] Failed to create work queue\n", __func__);
        goto error_work_queue;
    }
    
    ret = sysfs_create_group(&bh1721->input->dev.kobj, &bh1721_attribute_group);
    if (ret < 0) {
        goto error_sysfs_init;
    }

    ret = sensors_register(light_sensor_device, bh1721, additional_light_attrs, "light_sensor");
    if(ret) {
		pr_err("%s: cound not register sensor device\n", __func__);
		goto error_sysfs_init;
    }

    printk(KERN_INFO "%s registered\n", id->name);


	////////////////////for test///////////////////////////
#if 0
    int delay = atomic_read(&bh1721->delay), i;
    unsigned int lux;
  
       bh1721_power_up(bh1721);
       queue_delayed_work(work_queue, &bh1721->work, delay_to_jiffies(delay) + 1);
	
//	for(i=0;i<10;i++) 
	{
		lux = bh1721_measure();
		printk("\n bh1721_test lux:%d \n", lux);
	}
#endif
	////////////////////for test///////////////////////////

    return 0;

error_sysfs_init:
    cancel_delayed_work_sync(&bh1721->work);
error_work_queue:
    input_unregister_device(bh1721->input);
error_input_init:
    input_free_device(bh1721->input);
error_input_alloc:
    i2c_set_clientdata(client, NULL);
    bh1721_power_exit();
error_power_init:
    kfree(bh1721);
    return ret;
}

static int __exit bh1721_remove(struct i2c_client *client)
{
    destroy_workqueue(work_queue);
    bh1721_set_enable(NULL, 0);
    bh1721_power_exit();
    i2c_set_clientdata(client, NULL);
    kfree(bh1721);

    printk("[BH1721][%s] Unload module...", __func__);
    return 0;
}

static int bh1721_suspend(struct i2c_client *client, pm_message_t mesg)
{    
    bh1721_write_command(bh1721->client, &POWER_DOWN);
    return 0;
}

static int bh1721_resume(struct i2c_client *client)
{    
    bh1721_reset();
    return 0;
}

static const struct i2c_device_id bh1721_id[] = {
    {"bh1721", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, bh1721_id);

static struct i2c_driver bh1721_i2c_driver = {
    .driver = {
           .name = "bh1721",
           },
    .probe = bh1721_probe,
    .remove = __exit_p(bh1721_remove),
    .suspend = bh1721_suspend,
    .resume = bh1721_resume,
    .id_table = bh1721_id,
};

static int __init bh1721_init(void)
{
    return i2c_add_driver(&bh1721_i2c_driver);
}

module_init(bh1721_init);

static void __exit bh1721_exit(void)
{    
    i2c_del_driver(&bh1721_i2c_driver);
}

module_exit(bh1721_exit);

MODULE_AUTHOR("Donggeun Kim <dg77.kim@samsung.com>");
MODULE_DESCRIPTION("Linux Device Driver for BH1721");
MODULE_LICENSE("GPL");
