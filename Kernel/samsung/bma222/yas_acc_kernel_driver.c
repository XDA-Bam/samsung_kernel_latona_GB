/*
 * Copyright (c) 2010 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <plat/i2c-omap-gpio.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif


#define __LINUX_KERNEL_DRIVER__
#include <linux/yas.h>
#include "yas_acc_driver.c"

#define YAS_ACC_KERNEL_VERSION                                                        "3.0.401"
#define YAS_ACC_KERNEL_NAME                                                   "accelerometer"

/* REL axes parameter range [um/s^2] (for input event) */
#define GRAVITY_EARTH                                                                 9806550
#define RELMAX_2G                                                         (GRAVITY_EARTH * 2)
#define RELMIN_2G                                                        (-GRAVITY_EARTH * 2)

#define delay_to_jiffies(d)                                       ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)                               (jiffies_to_msecs(delay_to_jiffies(d)))

/* ---------------------------------------------------------------------------------------- *
   Function prototype declaration
 * ---------------------------------------------------------------------------------------- */
static struct yas_acc_private_data *yas_acc_get_data(void);
static void yas_acc_set_data(struct yas_acc_private_data *);

static int yas_acc_ischg_enable(struct yas_acc_driver *, int);

static int yas_acc_lock(void);
static int yas_acc_unlock(void);
static int yas_acc_i2c_open(void);
static int yas_acc_i2c_close(void);
static int yas_acc_i2c_write(uint8_t, uint8_t, const uint8_t *, int);
static int yas_acc_i2c_read(uint8_t, uint8_t, uint8_t *, int);
static void yas_acc_msleep(int);

static int yas_acc_core_driver_init(struct yas_acc_private_data *);
static void yas_acc_core_driver_fini(struct yas_acc_private_data *);
static int yas_acc_get_enable(struct yas_acc_driver *);
static int yas_acc_set_enable(struct yas_acc_driver *, int);
static int yas_acc_get_delay(struct yas_acc_driver *);
static int yas_acc_set_delay(struct yas_acc_driver *, int);
static int yas_acc_get_position(struct yas_acc_driver *);
static int yas_acc_set_position(struct yas_acc_driver *, int);
static int yas_acc_get_threshold(struct yas_acc_driver *);
static int yas_acc_set_threshold(struct yas_acc_driver *, int);
static int yas_acc_get_filter_enable(struct yas_acc_driver *);
static int yas_acc_set_filter_enable(struct yas_acc_driver *, int);
static int yas_acc_measure(struct yas_acc_driver *, struct yas_acc_data *);
static int yas_acc_input_init(struct yas_acc_private_data *);
static void yas_acc_input_fini(struct yas_acc_private_data *);

static ssize_t yas_acc_enable_show(struct device *, struct device_attribute *, char *);
static ssize_t yas_acc_enable_store(struct device *,struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_delay_show(struct device *, struct device_attribute *, char *);
static ssize_t yas_acc_delay_store(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_position_show(struct device *, struct device_attribute *, char *);
static ssize_t yas_acc_position_store(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_threshold_show(struct device *, struct device_attribute *, char *);
static ssize_t yas_acc_threshold_store(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_filter_enable_show(struct device *, struct device_attribute *, char *);
static ssize_t yas_acc_filter_enable_store(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_wake_store(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_private_data_show(struct device *, struct device_attribute *, char *);
#if DEBUG
static ssize_t yas_acc_debug_reg_show(struct device *, struct device_attribute *, char *);
static int yas_acc_suspend(OMAP_GPIO_I2C_CLIENT *client, pm_message_t);
static int yas_acc_resume(OMAP_GPIO_I2C_CLIENT *client);
#endif

static void yas_acc_work_func(struct work_struct *);
//static int yas_acc_probe(struct i2c_client *, const struct i2c_device_id *);
//static int yas_acc_remove(struct i2c_client *);
static int yas_acc_suspend(OMAP_GPIO_I2C_CLIENT *client, pm_message_t);
static int yas_acc_resume(OMAP_GPIO_I2C_CLIENT *client);

/* ---------------------------------------------------------------------------------------- *
   Driver private data
 * ---------------------------------------------------------------------------------------- */
struct yas_acc_private_data {
    struct mutex driver_mutex;
    struct mutex data_mutex;
    //struct i2c_client *client;
    OMAP_GPIO_I2C_CLIENT * client;
    struct input_dev *input;
    struct yas_acc_driver *driver;
    struct delayed_work work;
    struct yas_acc_data last;
#if 1//DEBUG	
    int suspend;
    int suspend_enable;
#endif	
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
#endif	
};

static struct yas_acc_private_data *yas_acc_private_data = NULL;
static struct yas_acc_private_data *yas_acc_get_data(void) {return yas_acc_private_data;}
static void yas_acc_set_data(struct yas_acc_private_data *data) {yas_acc_private_data = data;}

/* ---------------------------------------------------------------------------------------- *
   Local function
 * ---------------------------------------------------------------------------------------- */
static int yas_acc_ischg_enable(struct yas_acc_driver *driver, int enable)
{
    if (driver->get_enable() == enable) {
        return 0;
    }

    return 1;
}

/* ---------------------------------------------------------------------------------------- *
   Accelerlomete core driver callback function
 * ---------------------------------------------------------------------------------------- */
static int yas_acc_lock(void)
{
    struct yas_acc_private_data *data = yas_acc_get_data();

    mutex_lock(&data->driver_mutex);

    return 0;
}

static int yas_acc_unlock(void)
{
    struct yas_acc_private_data *data = yas_acc_get_data();

    mutex_unlock(&data->driver_mutex);

    return 0;
}

static int yas_acc_i2c_open(void)
{
    return 0;
}

static int yas_acc_i2c_close(void)
{
    return 0;
}

static int yas_acc_i2c_write(uint8_t slave, uint8_t reg, const uint8_t *buf, int len)
{
    OMAP_GPIO_I2C_WR_DATA i2c_wr_param;
    struct yas_acc_private_data *data = yas_acc_get_data();
	
    i2c_wr_param.reg_addr = &reg;
    i2c_wr_param.reg_len = 1;
    i2c_wr_param.wdata_len = len;
    i2c_wr_param.wdata = buf;
    return omap_gpio_i2c_write(data->client, &i2c_wr_param);

}

static int yas_acc_i2c_read(uint8_t slave, uint8_t reg, uint8_t *buf, int len)
{
    OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
    struct yas_acc_private_data *data = yas_acc_get_data();
    
    i2c_rd_param.reg_addr = &reg;
    i2c_rd_param.reg_len = 1;
    i2c_rd_param.rdata_len = len;
    i2c_rd_param.rdata = buf;
    return omap_gpio_i2c_read(data->client, &i2c_rd_param);
}

static void yas_acc_msleep(int msec)
{
    msleep(msec);
}

/* ---------------------------------------------------------------------------------------- *
   Accelerometer core driver access function
 * ---------------------------------------------------------------------------------------- */
static int yas_acc_core_driver_init(struct yas_acc_private_data *data)
{
    struct yas_acc_driver_callback *cbk;
    struct yas_acc_driver *driver;
    int err;

    data->driver = driver = kzalloc(sizeof(struct yas_acc_driver), GFP_KERNEL);
    if (!driver) {
        err = -ENOMEM;
        return err;
    }

    cbk = &driver->callback;
    cbk->lock = yas_acc_lock;
    cbk->unlock = yas_acc_unlock;
    cbk->i2c_open = yas_acc_i2c_open;
    cbk->i2c_close = yas_acc_i2c_close;
    cbk->i2c_write = yas_acc_i2c_write;
    cbk->i2c_read = yas_acc_i2c_read;
    cbk->msleep = yas_acc_msleep;

    err = yas_acc_driver_init(driver);
    if (err != YAS_NO_ERROR) {
        kfree(driver);
        return err;
    }

    err = driver->init();
    if (err != YAS_NO_ERROR) {
        kfree(driver);
        return err;
    }

    err = driver->set_position(CONFIG_INPUT_BMA222_POSITION);
    if (err != YAS_NO_ERROR) {
        kfree(driver);
        return err;
    }

//-------------for test----------------	
#if 0

    struct yas_acc_data accel;
    int i;
    accel.xyz.v[0] = accel.xyz.v[1] = accel.xyz.v[2] = 0;
	printk(KERN_ERR "bma222_init_proc test 1\n");
	
    err = driver->set_enable(1);
    if (err != YAS_NO_ERROR) {
		printk(KERN_ERR "bma222_init_proc set_enable err \n");
        kfree(driver);
        return err;
    }

printk(KERN_ERR "bma222_init_proc test 2\n");

for(i=0;i<10;i++)
{
    err = driver->measure(&accel);
    if (err != YAS_NO_ERROR) {
		printk(KERN_ERR "bma222_init_proc measure err =%d\n",err);
        kfree(driver);
        return err;
    }

     printk(KERN_ERR "bma222_init_proc data(%10d %10d %10d) raw(%5d %5d %5d)\n",
     accel.xyz.v[0], accel.xyz.v[1], accel.xyz.v[2], accel.raw.v[0], accel.raw.v[1], accel.raw.v[2]);
}
#endif
printk(KERN_ERR "bma222_init_proc test 3 \n");

//-------------for test----------------

    return 0;
}

static void yas_acc_core_driver_fini(struct yas_acc_private_data *data)
{
    struct yas_acc_driver *driver = data->driver;

    driver->term();
    kfree(driver);
}

static int yas_acc_get_enable(struct yas_acc_driver *driver)
{
    return driver->get_enable();
}

static int yas_acc_set_enable(struct yas_acc_driver *driver, int enable)
{
    struct yas_acc_private_data *data = yas_acc_get_data();
    int delay = driver->get_delay();	
    if (yas_acc_ischg_enable(driver, enable)) {
        if (enable) {
            driver->set_enable(enable);
            schedule_delayed_work(&data->work, delay_to_jiffies(delay) + 1);
        } else {
            cancel_delayed_work_sync(&data->work);
            driver->set_enable(enable);
        }
    }

    return 0;
}

static int yas_acc_get_delay(struct yas_acc_driver *driver)
{
    return driver->get_delay();
}

static int yas_acc_set_delay(struct yas_acc_driver *driver, int delay)
{
    struct yas_acc_private_data *data = yas_acc_get_data();

    if (driver->get_enable()) {
        cancel_delayed_work_sync(&data->work);
        driver->set_delay(actual_delay(delay));
        schedule_delayed_work(&data->work, delay_to_jiffies(delay) + 1);
    } else {
        driver->set_delay(actual_delay(delay));
    }

    return 0;
}

static int yas_acc_get_offset(struct yas_acc_driver *driver, struct yas_vector *offset)
{
    return driver->get_offset(offset);
}

static int yas_acc_set_offset(struct yas_acc_driver *driver, struct yas_vector *offset)
{
    return driver->set_offset(offset);
}

static int yas_acc_get_position(struct yas_acc_driver *driver)
{
    return driver->get_position();
}

static int yas_acc_set_position(struct yas_acc_driver *driver, int position)
{
    return driver->set_position(position);
}

static int yas_acc_get_threshold(struct yas_acc_driver *driver)
{
    struct yas_acc_filter filter;

    driver->get_filter(&filter);

    return filter.threshold;
}

static int yas_acc_set_threshold(struct yas_acc_driver *driver, int threshold)
{
    struct yas_acc_filter filter;

    filter.threshold = threshold;

    return driver->set_filter(&filter);
}

static int yas_acc_get_filter_enable(struct yas_acc_driver *driver)
{
    return driver->get_filter_enable();
}

static int yas_acc_set_filter_enable(struct yas_acc_driver *driver, int enable)
{
    return driver->set_filter_enable(enable);
}

static int yas_acc_measure(struct yas_acc_driver *driver, struct yas_acc_data *accel)
{
    int err;

    err = driver->measure(accel);
    if (err != YAS_NO_ERROR) {
        return err;
    }

#if 0
    printk("data(%10d %10d %10d) raw(%5d %5d %5d)\n",
           accel->xyz.v[0], accel->xyz.v[1], accel->xyz.v[2], accel->raw.v[0], accel->raw.v[1], accel->raw.v[2]);
#endif

    return err;
}

/* ---------------------------------------------------------------------------------------- *
   Input device interface
 * ---------------------------------------------------------------------------------------- */
static int yas_acc_input_init(struct yas_acc_private_data *data)
{
    struct input_dev *dev;
    int err;

    dev = input_allocate_device();
    if (!dev) {
        return -ENOMEM;
    }
    dev->name = "accelerometer_sensor";
    dev->id.bustype = BUS_I2C;

    input_set_capability(dev, EV_REL, REL_RY);
	input_set_capability(dev, EV_REL, REL_X);
	input_set_capability(dev, EV_REL, REL_Y);
	input_set_capability(dev, EV_REL, REL_Z);
#if 0
    input_set_abs_params(dev, REL_X, RELMIN_2G, RELMAX_2G, 0, 0);
    input_set_abs_params(dev, REL_Y, RELMIN_2G, RELMAX_2G, 0, 0);
    input_set_abs_params(dev, REL_Z, RELMIN_2G, RELMAX_2G, 0, 0);
#endif
    input_set_drvdata(dev, data);

    err = input_register_device(dev);
    if (err < 0) {
        input_free_device(dev);
        return err;
    }
    data->input = dev;

    return 0;
}

static void yas_acc_input_fini(struct yas_acc_private_data *data)
{
    struct input_dev *dev = data->input;

    input_unregister_device(dev);
    input_free_device(dev);
}

static ssize_t yas_acc_enable_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", yas_acc_get_enable(data->driver));
}

static ssize_t yas_acc_enable_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long enable = simple_strtoul(buf, NULL, 10);
    yas_acc_set_enable(data->driver, enable);

    return count;
}

static ssize_t yas_acc_delay_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", yas_acc_get_delay(data->driver));
}

static ssize_t yas_acc_delay_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long delay = simple_strtoul(buf, NULL, 10);

    yas_acc_set_delay(data->driver, delay);

    return count;
}

static ssize_t yas_acc_offset_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct yas_vector offset;

    yas_acc_get_offset(data->driver, &offset);

    return sprintf(buf, "%d %d %d\n", offset.v[0], offset.v[1], offset.v[2]);
}

static ssize_t yas_acc_offset_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct yas_vector offset;

    sscanf(buf, "%d %d %d", &offset.v[0], &offset.v[1], &offset.v[2]);

    yas_acc_set_offset(data->driver, &offset);

    return count;
}

static ssize_t yas_acc_position_show(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", yas_acc_get_position(data->driver));
}

static ssize_t yas_acc_position_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf,
                                      size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long position = simple_strtoul(buf, NULL,10);

    yas_acc_set_position(data->driver, position);

    return count;
}

static ssize_t yas_acc_threshold_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", yas_acc_get_threshold(data->driver));
}

static ssize_t yas_acc_threshold_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long threshold = simple_strtoul(buf, NULL,10);

    yas_acc_set_threshold(data->driver, threshold);

    return count;
}

static ssize_t yas_acc_filter_enable_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", yas_acc_get_filter_enable(data->driver));
}

static ssize_t yas_acc_filter_enable_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long enable = simple_strtoul(buf, NULL,10);;

    yas_acc_set_filter_enable(data->driver, enable);

    return count;
}

static ssize_t yas_acc_wake_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    static atomic_t serial = ATOMIC_INIT(0);

    input_report_rel(input, REL_RY, atomic_inc_return(&serial));

    return count;
}

static ssize_t yas_acc_private_data_show(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct yas_acc_data accel;

    mutex_lock(&data->data_mutex);
    accel = data->last;
    mutex_unlock(&data->data_mutex);

    return sprintf(buf, "%d %d %d\n", accel.xyz.v[0], accel.xyz.v[1], accel.xyz.v[2]);
}

#if DEBUG
#if YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA150
#define ADR_MAX (0x16)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA222
#define ADR_MAX (0x40)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXSD9
#define ADR_MAX (0x0f)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTE9
#define ADR_MAX (0x5c)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTF9
#define ADR_MAX (0x60)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DL  || \
      YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLH || \
      YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLM
#define ADR_MAX (0x40)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS3DH
#define ADR_MAX (0x3e)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL345
#define ADR_MAX (0x3a)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL346
#define ADR_MAX (0x3d)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8452Q || \
      YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8453Q
#define ADR_MAX (0x32)
#else
#define ADR_MAX (0x16)
#endif
static uint8_t reg[ADR_MAX];

static ssize_t yas_acc_debug_reg_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct i2c_client *client = data->client;
    ssize_t count = 0;
    int ret;
    int i;

    memset(reg, -1, ADR_MAX);
    for (i = 0; i < ADR_MAX; i++) {
        ret = data->driver->get_register(i, &reg[i]);
        if(ret != 0) {
            dev_err(&client->dev, "get_register() erorr %d (%d)\n", ret, i);
        } else {
            count += sprintf(&buf[count], "%02x: %d\n", i, reg[i]);
        }
    }

    return count;
}

static ssize_t yas_acc_debug_suspend_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", data->suspend);
}

static ssize_t yas_acc_debug_suspend_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct i2c_client *client = data->client;
    unsigned long suspend = simple_strtoul(buf, NULL, 10);

    if (suspend) {
        pm_message_t msg;
        yas_acc_suspend(client, msg);
    } else {
        yas_acc_resume(client);
    }

    return count;
}
#endif /* DEBUG */

  
static int bma222_fast_calibration(char layout[])
{
    char tmp = 1;	// select x axis in cal_trigger by default
	int power_off_after_calibration = 0;
	struct yas_acc_private_data *data =yas_acc_get_data();
	
    if(!yas_bma222_get_enable())
    {
        yas_bma222_power_up();
        power_off_after_calibration = 1;
    }
	
    yas_bma222_update_bits(YAS_BMA222_COMP_TARGET_OFFSET_X, layout[0]);

    yas_bma222_update_bits(YAS_BMA222_EN_FAST_COMP, tmp);
    do
    {
        mdelay(2);
        tmp = yas_bma222_read_bits(YAS_BMA222_FAST_COMP_RDY_S);
    } while(tmp == 0);

    yas_bma222_update_bits(YAS_BMA222_COMP_TARGET_OFFSET_Y, layout[1]);

    tmp = 2;	//selet y axis in cal_trigger
    yas_bma222_update_bits(YAS_BMA222_EN_FAST_COMP, tmp);
    do
    {
        mdelay(2); 
        tmp = yas_bma222_read_bits( YAS_BMA222_FAST_COMP_RDY_S);
    } while(tmp == 0);

    yas_bma222_update_bits(YAS_BMA222_COMP_TARGET_OFFSET_Z, layout[2]);
    tmp = 3;	//selet z axis in cal_trigger
    yas_bma222_update_bits(YAS_BMA222_EN_FAST_COMP, tmp);
    do
    {
        mdelay(2); 
        tmp = yas_bma222_read_bits(YAS_BMA222_FAST_COMP_RDY_S);
    } while(tmp == 0);

    tmp = 1;	//unlock eeprom
    yas_bma222_update_bits(YAS_BMA222_UNLOCK_EE_WRITE_SETTING, tmp);
    yas_bma222_update_bits(YAS_BMA222_START_EE_WRITE_SETTING, 0x01);

    do
    {
        mdelay(2); 
        tmp = yas_bma222_read_bits(YAS_BMA222_EE_WRITE_SETTING_S);
    } while(tmp==0);

    tmp = 0; 	//lock eemprom	
    yas_bma222_update_bits(YAS_BMA222_UNLOCK_EE_WRITE_SETTING, tmp);

    if(power_off_after_calibration)
    {
        yas_bma222_power_down();
    }
	
    return 0;
}

static ssize_t bma222_calibration_show(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
}

static ssize_t bma222_calibration_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
    char layout[3] = {0, 0, 1};

    printk("### bma222_calibration start\n");
	
    bma222_fast_calibration(layout);
	
    printk("### bma222_calibration end\n");
	
    return count;
}

static DEVICE_ATTR(enable,
                   S_IRUGO|S_IWUSR|S_IWGRP,
                   yas_acc_enable_show,
                   yas_acc_enable_store
                   );
static DEVICE_ATTR(poll_delay,
                   S_IRUGO|S_IWUSR|S_IWGRP,
                   yas_acc_delay_show,
                   yas_acc_delay_store
                   );
static DEVICE_ATTR(offset,
                   S_IRUGO|S_IWUSR,
                   yas_acc_offset_show,
                   yas_acc_offset_store
                   );
static DEVICE_ATTR(position,
                   S_IRUGO|S_IWUSR,
                   yas_acc_position_show,
                   yas_acc_position_store
                   );
static DEVICE_ATTR(threshold,
                   S_IRUGO|S_IWUSR,
                   yas_acc_threshold_show,
                   yas_acc_threshold_store
                   );
static DEVICE_ATTR(filter_enable,
                   S_IRUGO|S_IWUSR,
                   yas_acc_filter_enable_show,
                   yas_acc_filter_enable_store
                   );
static DEVICE_ATTR(wake,
                   S_IWUSR|S_IWGRP,
                   NULL,
                   yas_acc_wake_store);
static DEVICE_ATTR(raw_data,
                   S_IRUGO,
                   yas_acc_private_data_show,
                   NULL);
static DEVICE_ATTR(calibration, S_IRUGO|S_IWUSR|S_IWGRP,
                   bma222_calibration_show, bma222_calibration_store);

#if DEBUG
static DEVICE_ATTR(debug_reg,
                   S_IRUGO,
                   yas_acc_debug_reg_show,
                   NULL
                   );
static DEVICE_ATTR(debug_suspend,
                   S_IRUGO|S_IWUSR,
                   yas_acc_debug_suspend_show,
                   yas_acc_debug_suspend_store
                   );
#endif /* DEBUG */

static struct attribute *accel_sensor_attrs[] = {
    &dev_attr_calibration.attr,
    &dev_attr_raw_data.attr,
    NULL
};

static struct attribute *yas_acc_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_poll_delay.attr,
    &dev_attr_offset.attr,
    &dev_attr_position.attr,
    &dev_attr_threshold.attr,
    &dev_attr_filter_enable.attr,
    &dev_attr_wake.attr,
#if DEBUG
    &dev_attr_debug_reg.attr,
    &dev_attr_debug_suspend.attr,
#endif /* DEBUG */
    NULL
};

static struct attribute_group yas_acc_attribute_group = {
    .attrs = yas_acc_attributes
};

static void yas_acc_work_func(struct work_struct *work)
{
    struct yas_acc_private_data *data = container_of((struct delayed_work *)work,
                                              struct yas_acc_private_data, work);
    struct yas_acc_data accel;
    unsigned long delay = delay_to_jiffies(yas_acc_get_delay(data->driver));

    accel.xyz.v[0] = accel.xyz.v[1] = accel.xyz.v[2] = 0;
    yas_acc_measure(data->driver, &accel);

    input_report_rel(data->input, REL_X, accel.xyz.v[0]);
    input_report_rel(data->input, REL_Y, accel.xyz.v[1]);
    input_report_rel(data->input, REL_Z, accel.xyz.v[2]);
    input_sync(data->input);
	
	//printk("\n yas_acc_work_func X:%d Y:%d Z:%d \n", accel.xyz.v[0], accel.xyz.v[1], accel.xyz.v[2]);
	
    mutex_lock(&data->data_mutex);
    data->last = accel;
    mutex_unlock(&data->data_mutex);

    schedule_delayed_work(&data->work, delay);
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static int bma222_early_suspend(struct early_suspend *handler)
{
/*
    struct yas_acc_private_data *data =yas_acc_get_data();
    struct yas_acc_driver *driver = data->driver;

    mutex_lock(&data->data_mutex);

    if (data->suspend == 0) {
        data->suspend_enable = yas_acc_get_enable(driver);
        if (data->suspend_enable) {
            cancel_delayed_work_sync(&data->work);
            yas_acc_set_enable(driver, 0);
        }
    }
    data->suspend = 1;

    mutex_unlock(&data->data_mutex);
*/
    return 0;
}
#endif

static int yas_acc_suspend(OMAP_GPIO_I2C_CLIENT *client, pm_message_t mesg)
{
/*    struct yas_acc_private_data *data =yas_acc_get_data();
    struct yas_acc_driver *driver = data->driver;

    mutex_lock(&data->data_mutex);

    if (data->suspend == 0) {
        data->suspend_enable = yas_acc_get_enable(driver);
        if (data->suspend_enable) {
            cancel_delayed_work_sync(&data->work);
            yas_acc_set_enable(driver, 0);
        }
    }
    data->suspend = 1;

    mutex_unlock(&data->data_mutex);
*/
    return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static int bma222_early_resume(struct early_suspend *handler)
{
/*    struct yas_acc_private_data *data =yas_acc_get_data();
    struct yas_acc_driver *driver = data->driver;
    int delay;

    mutex_lock(&data->data_mutex);

    if (data->suspend == 1) {
        if (data->suspend_enable) {
            delay = yas_acc_get_delay(driver);
            schedule_delayed_work(&data->work, delay_to_jiffies(delay) + 1);
            yas_acc_set_enable(driver, 1);
        }
    }
    data->suspend = 0;

    mutex_unlock(&data->data_mutex);
*/
    return 0;

}
#endif

static int yas_acc_resume(OMAP_GPIO_I2C_CLIENT *client)
{
 /*   struct yas_acc_private_data *data =yas_acc_get_data();
    struct yas_acc_driver *driver = data->driver;
    int delay;

    mutex_lock(&data->data_mutex);

    if (data->suspend == 1) {
        if (data->suspend_enable) {
            delay = yas_acc_get_delay(driver);
            schedule_delayed_work(&data->work, delay_to_jiffies(delay) + 1);
            yas_acc_set_enable(driver, 1);
        }
    }
    data->suspend = 0;

    mutex_unlock(&data->data_mutex);
*/
    return 0;
}

extern struct class *sensors_class;
extern int sensors_register(struct device *dev, void * drvdata, struct device_attribute *attributes[], char *name);
static struct device *accel_sensor_device;

//static int yas_acc_probe(struct i2c_client *client, const struct i2c_device_id *id)
static int bma222_init_proc()
{
    struct yas_acc_private_data *data;
    int err;
     printk(KERN_ERR "bma222_init_proc +\n");
    /* Setup private data */
    data = kzalloc(sizeof(struct yas_acc_private_data), GFP_KERNEL);
    if (!data) {
        err = -ENOMEM;
        goto ERR1;
    }
    yas_acc_set_data(data);

    mutex_init(&data->driver_mutex);
    mutex_init(&data->data_mutex);

    data->client = omap_gpio_i2c_init(OMAP_GPIO_SENSOR_SDA, OMAP_GPIO_SENSOR_SCL, 0x08, 200);
	
/*
    // Setup i2c client
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        goto ERR2;
    }
    i2c_set_clientdata(client, data);
    data->client = client;
*/
    /* Setup accelerometer core driver */
    err = yas_acc_core_driver_init(data);
    if (err < 0) {
        goto ERR2;
    }

    /* Setup driver interface */
    INIT_DELAYED_WORK(&data->work, yas_acc_work_func);

    /* Setup input device interface */
    err = yas_acc_input_init(data);
    if (err < 0) {
        goto ERR3;
    }

    /* Setup sysfs */
    err = sysfs_create_group(&data->input->dev.kobj, &yas_acc_attribute_group);
    if (err < 0) {
        goto ERR4;
    }

    err = sensors_register(accel_sensor_device, NULL, accel_sensor_attrs, "accelerometer_sensor");
    if(err) {
    	printk(KERN_ERR "%s: cound not register accelerometer sensor device(%d).\n", __func__, err);
    }
	
#if defined(CONFIG_HAS_EARLYSUSPEND)
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    data->early_suspend.suspend = (void *)bma222_early_suspend;
    data->early_suspend.resume  = (void *)bma222_early_resume;
    register_early_suspend(&data->early_suspend);
#endif	

	printk(KERN_ERR "bma222_init_proc -\n");


    return 0;

 ERR4:
    yas_acc_input_fini(data);
 ERR3:
    yas_acc_core_driver_fini(data);
 ERR2:
    kfree(data);
 ERR1:
    return err;
}

static int bma222_exit_proc()
{

  struct yas_acc_private_data *data = yas_acc_get_data();
  struct yas_acc_driver *driver = data->driver;
	
#if defined(CONFIG_HAS_EARLYSUSPEND)
		unregister_early_suspend(&data->early_suspend);
#endif

    yas_acc_set_enable(driver, 0);
    sysfs_remove_group(&data->input->dev.kobj, &yas_acc_attribute_group);
    yas_acc_input_fini(data);
    yas_acc_core_driver_fini(data);
    kfree(data);	

    return 0;
}

/*
static int yas_acc_remove(struct i2c_client *client)
{
    struct yas_acc_private_data *data = i2c_get_clientdata(client);
    struct yas_acc_driver *driver = data->driver;

    yas_acc_set_enable(driver, 0);
    sysfs_remove_group(&data->input->dev.kobj, &yas_acc_attribute_group);
    yas_acc_input_fini(data);
    yas_acc_core_driver_fini(data);
    kfree(data);

    return 0;
}

static const struct i2c_device_id yas_acc_id[] = {
    {YAS_ACC_KERNEL_NAME, 0},
    {},
};

MODULE_DEVICE_TABLE(i2c, yas_acc_id);

struct i2c_driver yas_acc_driver = {
    .driver = {
        .name = "accelerometer",
        .owner = THIS_MODULE,
    },
    .probe = yas_acc_probe,
    .remove = yas_acc_remove,
    .suspend = yas_acc_suspend,
    .resume = yas_acc_resume,
    .id_table = yas_acc_id,
};
*/

/* ---------------------------------------------------------------------------------------- *
   Module init and exit
 * ---------------------------------------------------------------------------------------- */
static int __init yas_acc_init(void)
{
    //return i2c_add_driver(&yas_acc_driver);
    return bma222_init_proc();
}
module_init(yas_acc_init);

static void __exit yas_acc_exit(void)
{
    //i2c_del_driver(&yas_acc_driver);
    bma222_exit_proc();
}
module_exit(yas_acc_exit);

MODULE_DESCRIPTION("accelerometer kernel driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(YAS_ACC_KERNEL_VERSION);
