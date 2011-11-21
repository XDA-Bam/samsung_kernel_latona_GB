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
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/i2c/k3g.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <plat/i2c-omap-gpio.h>

/* for debugging */
#define DEBUG 0
#define MUTEX_DEBUG 0

#define SENSOR_TYPE (4)

#if SENSOR_TYPE == 1
#define SENSOR_NAME "accelerometer"
#elif SENSOR_TYPE == 2
#define SENSOR_NAME "geomagnetic"
#elif SENSOR_TYPE == 3
#define SENSOR_NAME "orientation"
#elif SENSOR_TYPE == 4
#define SENSOR_NAME "gyro_sensor"
#elif SENSOR_TYPE == 5
#define SENSOR_NAME "light"
#elif SENSOR_TYPE == 6
#define SENSOR_NAME "pressure"
#elif SENSOR_TYPE == 7
#define SENSOR_NAME "temperature"
#elif SENSOR_TYPE == 8
#define SENSOR_NAME "proximity"
#endif

#define SENSOR_DEFAULT_DELAY            (200)   /* 200 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */

#define REL_STATUS			(REL_RX)
#define REL_WAKE			(REL_RY)
#define REL_CONTROL_REPORT			(REL_MISC)

/* k3g gyroscope registers */
#define WHO_AM_I        0x0F

#define CTRL_REG1       0x20    /* power control reg */
#define CTRL_REG2       0x21    /* power control reg */
#define CTRL_REG3       0x22    /* power control reg */
#define CTRL_REG4       0x23    /* interrupt control reg */
#define CTRL_REG5       0x24    /* interrupt control reg */
#define OUT_TEMP        0x26    /* Temperature data */
#define AXISDATA_REG    0x28

#define REG_INDEX(x)    (x) - CTRL_REG1

#define STATUS_REG      0x27    

#define PM_REG_MASK     0x08
#define PM_OFF          0x00
#define PM_NORMAL       0x08
#define ENABLE_ALL_AXES 0x07

#define ODR_BW_REG_MASK 0xF0
#define ODR100_BW12_5   0x00  /* ODR = 100Hz; BW = 12.5Hz */
#define ODR100_BW25     0x10  /* ODR = 100Hz; BW = 25Hz   */
#define ODR100_BW50     0x20  /* ODR = 100Hz; BW = 50Hz   */
#define ODR200_BW12_5   0x40  /* ODR = 200Hz; BW = 12.5Hz */
#define ODR200_BW25     0x50  /* ODR = 200Hz; BW = 25Hz   */
#define ODR200_BW50     0x60  /* ODR = 200Hz; BW = 50Hz   */
#define ODR200_BW70     0x70  /* ODR = 200Hz; BW = 70Hz   */
#define ODR400_BW25     0x90  /* ODR = 400Hz; BW = 25Hz   */
#define ODR400_BW50     0xA0  /* ODR = 400Hz; BW = 50Hz   */
#define ODR400_BW110    0xB0  /* ODR = 400Hz; BW = 110Hz  */
#define ODR800_BW50     0xE0  /* ODR = 800Hz; BW = 50Hz   */
#define ODR800_BW100    0xF0  /* ODR = 800Hz; BW = 100Hz  */

#define BDU_REG_MASK    0x80  /* Block Data Update. 0: Continuous update, 1: Output registers not updated until MSB and LB reading */
#define RANGE_REG_MASK  0x30  /* Full Scale selection. 00: 250 dps, 01: 500 dps, 10: 2000dps, 11: 2000dps */

#define RANGE_250       0x00
#define RANGE_500       0x10
#define RANGE_2000      0x20

#define MIN_ST            175
#define MAX_ST            875

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

#ifdef DEBUG
    #define dprintk(fmt,arg...)     printk("[K3G][%s] " fmt "\n",__func__, ## arg)
#else
    #define dprintk(x)
#endif

#if MUTEX_DEBUG
    #define MUTEX_LOCK(x) \
        printk("[K3G][%s] : LOCK\n", __func__); \
        mutex_lock(x)
        
    #define MUTEX_UNLOCK(x) \
        printk("[K3G][%s] : UNLOCK\n", __func__); \
        mutex_unlock(x)
#else
    #define MUTEX_LOCK(x)   mutex_lock(x)
    #define MUTEX_UNLOCK(x) mutex_unlock(x)
#endif

/*
 * k3g gyroscope data
 * brief structure containing gyroscope values for yaw, pitch and roll in
 * signed short
 */

struct k3g_t {
    short y,    /* yaw data. Range -2048 to 2047. */
          p,    /* pitch data. Range -2048 to 2047. */
          r;    /* roll data. Range -2048 to 2047. */
};

struct sensor_data {
    struct mutex mutex;
    OMAP_GPIO_I2C_CLIENT *client;
    struct k3g_platform_data *pdata;
    struct delayed_work work;
    struct k3g_t last;
    unsigned char reg[REG_INDEX(CTRL_REG5)+1];
    int enabled;
    int delay;
#if DEBUG
    int suspend;
#endif
};

static struct sensor_data *k3g_dev;

static int k3g_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);

static int k3g_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);

static int suspend(void);
static int resume(void);

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;

static struct workqueue_struct* work_queue;

/* set k3g digital gyroscope bandwidth */
static int k3g_set_bandwidth(char bw)
{
    int res = -EIO;
    
    dprintk("bandwidth = %d\n", bw);

    k3g_dev->reg[REG_INDEX(CTRL_REG1)] &= ~ODR_BW_REG_MASK;
    k3g_dev->reg[REG_INDEX(CTRL_REG1)] |= bw;
    
    res = k3g_i2c_write(CTRL_REG1, &k3g_dev->reg[REG_INDEX(CTRL_REG1)], 1);
    
    return res;
}

/* read selected bandwidth from k3g */
static int k3g_get_bandwidth(unsigned char *bw)
{
    int res = -EIO;
    unsigned char data = 0;
    
    
    if((res = k3g_i2c_read(CTRL_REG1, &data, 1)) >= 0)
    {
        *bw = (data & 0x30) >> 4;
        dprintk("bandwidth = %d\n", *bw);
        return 0;
    }
    
    return res;
}

static int k3g_set_mode(char mode)
{
    int res = -EIO;

    dprintk("mode = 0x%x\n", mode);
    
    k3g_dev->reg[REG_INDEX(CTRL_REG1)] &= ~PM_REG_MASK;
    k3g_dev->reg[REG_INDEX(CTRL_REG1)] |= mode;
    
    res = k3g_i2c_write(CTRL_REG1, &k3g_dev->reg[REG_INDEX(CTRL_REG1)], 1);
    
    return res;
}

static int k3g_set_range(char range)
{
    int res = -EIO;

    dprintk("range = 0x%x\n", range);
    
    k3g_dev->reg[REG_INDEX(CTRL_REG4)] &= ~RANGE_REG_MASK;
    k3g_dev->reg[REG_INDEX(CTRL_REG4)] |= range;
    
    res = k3g_i2c_write(CTRL_REG4, &k3g_dev->reg[REG_INDEX(CTRL_REG4)], 1);
    
    return res;
}

/* gyroscope data readout */
static int k3g_read_gyro_values(struct k3g_t *data)
{
    int res = 0;
    unsigned char gyro_data[6] = {0};
    unsigned char ready;
    int retry_count = 10;
    short hw_d[3] = { 0 };    /* x,y,z hardware data */

    // check ZYXDA ready bit
    do {
        k3g_i2c_read(STATUS_REG, &ready, 1);
        if(ready & 0x08)
            break;
        msleep(100);
    } while(retry_count-- > 0);
    
    if(retry_count <= 0)
    {
        printk(KERN_ERR "[K3G][%s] Can't get ZYXDA bit\n", __func__);
        return -EIO;
    }
        
    res = k3g_i2c_read(AXISDATA_REG, gyro_data, 6);
    if(!res)
    {
        hw_d[0] = (short) (((gyro_data[1]) << 8) | gyro_data[0]);
        hw_d[1] = (short) (((gyro_data[3]) << 8) | gyro_data[2]);
        hw_d[2] = (short) (((gyro_data[5]) << 8) | gyro_data[4]);

        k3g_dev->pdata->negate_x = 0;
        k3g_dev->pdata->negate_y = 0;
        k3g_dev->pdata->negate_z = 0;
        
        data->y = ((k3g_dev->pdata->negate_x) ? (-hw_d[2]) : (hw_d[2]));
        data->p = ((k3g_dev->pdata->negate_y) ? (-hw_d[1]) : (hw_d[1]));
        data->r = ((k3g_dev->pdata->negate_z) ? (-hw_d[0]) : (hw_d[0]));
    }
    
    printk("[k3g_read_gyro_values] %d, %d, %d \n", data->y, data->p, data->r);
    return res;
}


/* Device Initialization  */
static int device_init(void)
{
    return k3g_i2c_write(CTRL_REG1, k3g_dev->reg, 5);
}

static int k3g_power_on(void)
{
    int res;
    
    res = device_init();
    
    mdelay(800);
    
    printk("[%s] result of device init = %d\n", __func__, res);
    
    return res;
}

/*  i2c write routine for k3g digital gyroscope */
static int k3g_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    int res = 0;
    int i = 0;
    OMAP_GPIO_I2C_WR_DATA i2c_wr_param;
        
    if (k3g_dev->client == NULL)  /*  No global client pointer? */
        return -1;
        
    for(i = 0; i < len; i++, reg_addr++)
    {
        i2c_wr_param.reg_addr = &reg_addr;
        i2c_wr_param.reg_len  = 1;
        i2c_wr_param.wdata    = &data[i];
        i2c_wr_param.wdata_len= 1;
        
        if((res = omap_gpio_i2c_write(k3g_dev->client, &i2c_wr_param)))
        {
            printk(KERN_ERR "[K3G][%s] Failed to write via i2c!\n", __func__);
            break;
        }
    }

    return res;
}

/*  i2c read routine for k3g digital gyroscope */
static int k3g_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
    int res = 0;
    int i = 0;
    OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
    
    if (k3g_dev->client == NULL)  /*  No global client pointer? */
        return -1;
        
    for(i = 0; i < len; i++, reg_addr++)
    {
        i2c_rd_param.reg_addr = &reg_addr;
        i2c_rd_param.reg_len  = 1;
        i2c_rd_param.rdata    = &data[i];
        i2c_rd_param.rdata_len= 1;
        
        if((res = omap_gpio_i2c_read(k3g_dev->client, &i2c_rd_param)))
        {
            printk(KERN_ERR "[K3G][%s] Failed to read via i2c!\n", __func__);
            break;
        }
    }

    return res;
}

static ssize_t k3g_self_test(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    int res;    
    unsigned char temp[1] = {0};
    unsigned char gyro_data[6] = {0,0,0,0,0,0};
    short raw[3] = {0,0,0};
    int NOST[3], ST[3];
    int differ_x = 0, differ_y = 0, differ_z = 0;
    unsigned char bak_reg[5] = {0,0,0,0,0};
    unsigned char reg[5] = {0,0,0,0,0};
    unsigned char bZYXDA = 0;
    unsigned char pass = 0;
    int i = 0;
    int fail_count = 0;
    
    memset(NOST, 0, sizeof(int)*3);
    memset(ST, 0, sizeof(int)*3);
    
    // before starting self-test, backup register
    k3g_i2c_read(CTRL_REG1, bak_reg, 5);
    
    for(i = 0; i < 5; i++)
        printk("[gyro_self_test] backup reg[%d] = %2x\n", i, bak_reg[i]);
    
    // Initialize Sensor, turn on sensor, enable P/R/Y
    // Set BDU=1, Set ODR=200Hz, Cut-Off Frequency=50Hz, FS=2000dps
    reg[0] = 0x6f;
    reg[1] = 0x00;
    reg[2] = 0x00;
    reg[3] = 0xA0;
    reg[4] = 0x02;

    k3g_i2c_write(CTRL_REG1, reg, 5);
    
    // Power up, wait for 800ms for stable output
    mdelay(800);
        
    // Read 5 samples output before self-test on
    i = 0;
    fail_count = 0;
    while(i < 5)
    {
        // check ZYXDA ready bit
        k3g_i2c_read(STATUS_REG, temp, 1);
        bZYXDA = ((unsigned int)temp[0] & 0x08) >> 3;        
        
        if(!bZYXDA)
        {
            fail_count++;
            mdelay(100);
            if(fail_count < 10)
                continue;
            else
                goto exit;
        }
        
        res = k3g_i2c_read(AXISDATA_REG, &gyro_data[0], 6);

        raw[0] = (short) (((gyro_data[1]) << 8) | gyro_data[0]);
        raw[1] = (short) (((gyro_data[3]) << 8) | gyro_data[2]);
        raw[2] = (short) (((gyro_data[5]) << 8) | gyro_data[4]);
        
        NOST[0] += raw[0];
        NOST[1] += raw[1];
        NOST[2] += raw[2];
        
        printk("[gyro_self_test] raw[0] = %d\n", raw[0]);
        printk("[gyro_self_test] raw[1] = %d\n", raw[1]);
        printk("[gyro_self_test] raw[2] = %d\n", raw[2]);
        printk("\n");
        
        i++;
    }    
    
    for(i = 0; i < 3; i++)
        printk("[gyro_self_test] SUM of NOST[%d] = %d\n", i, NOST[i]);
    
    // calculate average of NOST and covert from ADC to DPS
    for(i = 0; i < 3; i++)
    {
        NOST[i] = (NOST[i] / 5) * 70 / 1000;
        printk("[gyro_self_test] AVG of NOST[%d] = %d\n", i, NOST[i]);
    }
    printk("\n");
    
    // Enable Self Test
    reg[0] = 0xA2;
    k3g_i2c_write(CTRL_REG4, &reg[0], 1);    
    
    mdelay(100);
    
    // Read 5 samples output after self-test on
    i = 0;
    fail_count = 0;
    while(i < 5)
    {
        // check ZYXDA ready bit
        k3g_i2c_read(STATUS_REG, &temp[0], 1);
        bZYXDA = ((unsigned int)temp[0] & 0x08) >> 3;        
        
        if(!bZYXDA)
        {
            fail_count++;
            mdelay(100);
            if(fail_count < 10)
                continue;
            else
                goto exit;
        }
        
        res = k3g_i2c_read(AXISDATA_REG, &gyro_data[0], 6);

        raw[0] = (short) (((gyro_data[1]) << 8) | gyro_data[0]);
        raw[1] = (short) (((gyro_data[3]) << 8) | gyro_data[2]);
        raw[2] = (short) (((gyro_data[5]) << 8) | gyro_data[4]);
        
        ST[0] += raw[0];
        ST[1] += raw[1];
        ST[2] += raw[2];
        
        printk("[gyro_self_test] raw[0] = %d\n", raw[0]);
        printk("[gyro_self_test] raw[1] = %d\n", raw[1]);
        printk("[gyro_self_test] raw[2] = %d\n", raw[2]);
        printk("\n");
        
        i++;
    }    
    
    for(i = 0; i < 3; i++)
        printk("[gyro_self_test] SUM of ST[%d] = %d\n", i, ST[i]);
    
    // calculate average of ST and convert from ADC to dps
    for(i = 0; i < 3; i++)
    {
        ST[i] = (ST[i] / 5) * 70 / 1000; // When FS=2000, 70 mdps/digit
        printk("[gyro_self_test] AVG of ST[%d] = %d\n", i, ST[i]);
    }    
        
    // check whether pass or not
    if( ST[0] >= NOST[0] )  // for x
        differ_x = ST[0] - NOST[0];
    else
        differ_x = NOST[0] - ST[0];
    
    if( ST[1] >= NOST[1] )  // for y
        differ_y = ST[1] - NOST[1];
    else
        differ_y = NOST[1] - ST[1];
        
    if( ST[2] >= NOST[2] )  // for z
        differ_z = ST[2] - NOST[2];
    else
        differ_z = NOST[2] - ST[2];
        
    printk("[gyro_self_test] differ x:%d, y:%d, z:%d\n", differ_x, differ_y, differ_z); 
        
    if( (MIN_ST <= differ_x && differ_x <= MAX_ST) && (MIN_ST <= differ_y && differ_y <= MAX_ST) &&
        (MIN_ST <= differ_z && differ_z <= MAX_ST) )
        pass = 1;    

exit:        
    // restore backup register
    k3g_i2c_write(CTRL_REG1, &bak_reg[0], 5);
    
    printk("[gyro_self_test] self-test result : %s\n", pass ? "pass" : "fail");
    count = sprintf(buf,"%d,%d,%d,%d,%d,%d,%d\n", NOST[0], NOST[1], NOST[2], ST[0], ST[1], ST[2], pass);

    return count;
}

static int k3g_validate_pdata(struct sensor_data *data)
{
    if (data->pdata->axis_map_x > 2 ||
        data->pdata->axis_map_y > 2 ||
        data->pdata->axis_map_z > 2) {
        printk(KERN_ERR "invalid axis_map value x:%u y:%u z%u\n",
            data->pdata->axis_map_x, data->pdata->axis_map_y,
            data->pdata->axis_map_z);
        return -EINVAL;
    }

    printk(KERN_INFO "%s : check map!!\n", __func__);

    /* Only allow 0 and 1 for negation boolean flag */
    if (data->pdata->negate_x > 1 ||
        data->pdata->negate_y > 1 ||
        data->pdata->negate_z > 1) {
        printk(KERN_ERR "invalid negate value x:%u y:%u z:%u\n",
            data->pdata->negate_x, data->pdata->negate_y,
            data->pdata->negate_z);
        return -EINVAL;
    }

    return 0;
}

static int k3g_detect_and_check_chip_id(void)
{
    OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
    unsigned char data;
    unsigned char reg = WHO_AM_I;
    int res = 0;
    
    i2c_rd_param.reg_len   = 0;
    i2c_rd_param.rdata_len = 1;
    i2c_rd_param.rdata     = &data;
    
    if((res = omap_gpio_i2c_read(k3g_dev->client, &i2c_rd_param)))
    {
        printk(KERN_ERR "[K3G][%s] : i2c_smbus_read_byte error!!.\n", __func__);
        return res;
    } 
    
    i2c_rd_param.reg_len  = 1;
    i2c_rd_param.reg_addr = &reg;
    
    if((res = omap_gpio_i2c_read(k3g_dev->client, &i2c_rd_param)))
    {
        printk(KERN_ERR "[K3G][%s] : i2c_smbus_read_byte error!!.\n", __func__);
        return res;
    } else
    {
        printk("[K3G][%s] CHIP ID = 0x%x\n", __func__, data);
    }
    
    return 0;
}

static void k3g_work_func(struct work_struct *work)
{
    struct sensor_data *data = container_of((struct delayed_work *)work,
                                             struct sensor_data, work);
    struct k3g_t value = {0,};
    unsigned long delay = delay_to_jiffies(k3g_dev->delay);

    //printk("k3g_work_func delay = %d, orig?_delay = %d \n",delay, k3g_dev->delay);
    k3g_read_gyro_values(&value);

     input_report_rel(this_data, REL_Z, value.y);
     input_report_rel(this_data, REL_Y, value.p);
     input_report_rel(this_data, REL_X, value.r);
    input_sync(this_data);

    MUTEX_LOCK(&k3g_dev->mutex);
    k3g_dev->last = value;
    MUTEX_UNLOCK(&k3g_dev->mutex);
   if (k3g_dev->delay>0)
   schedule_delayed_work(&k3g_dev->work, delay);
   //printk(" k3g_work_func : schedule_delayed_work \n");
}

/* Sysfs interface */
static ssize_t
sensor_delay_show(struct device *dev, 
                  struct device_attribute *attr, 
                  char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int delay;

    MUTEX_LOCK(&k3g_dev->mutex);

    delay = k3g_dev->delay;

    MUTEX_UNLOCK(&k3g_dev->mutex);

    return sprintf(buf, "%d\n", delay);
}

static ssize_t
sensor_delay_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf,
                   size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);

	//printk("sensor_delay_store : delay =%d \n",value);

    if (value < 0) {
        return count;
    }

    if (SENSOR_MAX_DELAY < value) {
        value = SENSOR_MAX_DELAY;
    }

    MUTEX_LOCK(&k3g_dev->mutex);

    if (data->enabled) {
        cancel_delayed_work(&k3g_dev->work);
	 printk(" sensor_delay_store : cancel_delayed_work \n");	
        schedule_delayed_work(&k3g_dev->work, delay_to_jiffies(value) + 1);
	 printk(" sensor_delay_store : schedule_delayed_work \n");	
    }
    
    k3g_dev->delay = value;

     input_report_rel(input_data, REL_CONTROL_REPORT, (k3g_dev->enabled<<16) | value);

    MUTEX_UNLOCK(&k3g_dev->mutex);

    return count;
}

static ssize_t
sensor_enable_show(struct device *dev,
                   struct device_attribute *attr,
                   char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int enabled;

    MUTEX_LOCK(&k3g_dev->mutex);

    enabled = k3g_dev->enabled;

    MUTEX_UNLOCK(&k3g_dev->mutex);

    return sprintf(buf, "%d\n", enabled);
}

static ssize_t
sensor_enable_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf,
                    size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtoul(buf, NULL, 10);

    if (value != 0 && value != 1) {
        return count;
    }

    printk("[K3G][%s] enabled(%d), value(%d)\n", __func__, k3g_dev->enabled, value);
    
    MUTEX_LOCK(&k3g_dev->mutex);

    if (k3g_dev->enabled && !value) {
	 printk("[K3G] gyro disabled !!!\n");	
        cancel_delayed_work(&k3g_dev->work);
	 k3g_dev->delay = -1;	
	 printk(" sensor_enable_store : cancel_delayed_work \n");
	 k3g_set_mode(PM_OFF);
    }
    if (!k3g_dev->enabled && value) {
	 printk("[K3G] gyro enabled !!!\n");
	 k3g_set_mode(PM_NORMAL);
        k3g_power_on();
        schedule_delayed_work(&k3g_dev->work, delay_to_jiffies(k3g_dev->delay) + 1);
	 printk(" sensor_enable_store : schedule_delayed_work \n");
    }
    k3g_dev->enabled = value;

     input_report_rel(input_data, REL_CONTROL_REPORT, (value<<16) | k3g_dev->delay);

    MUTEX_UNLOCK(&k3g_dev->mutex);

    return count;
}

static ssize_t
sensor_wake_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf,
                  size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    static int cnt = 1;

     input_report_rel(input_data, REL_WAKE, cnt++);

    return count;
}

static ssize_t
sensor_data_show(struct device *dev,
                 struct device_attribute *attr,
                 char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
#if SENSOR_TYPE <= 4
    int x, y, z;
#else
    int x;
#endif

    spin_lock_irqsave(&input_data->event_lock, flags);

    x = input_data->abs[REL_X];
#if SENSOR_TYPE <= 4
    y = input_data->abs[REL_Y];
    z = input_data->abs[REL_Z];
#endif

    spin_unlock_irqrestore(&input_data->event_lock, flags);

#if SENSOR_TYPE <= 4
    return sprintf(buf, "%d %d %d\n", x, y, z);
#else
    return sprintf(buf, "%d\n", x);
#endif
}

static ssize_t
sensor_status_show(struct device *dev,
                   struct device_attribute *attr,
                   char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
    int status;

    spin_lock_irqsave(&input_data->event_lock, flags);

    status = input_data->abs[REL_STATUS];

    spin_unlock_irqrestore(&input_data->event_lock, flags);

    return sprintf(buf, "%d\n", status);
}

static ssize_t
sensor_bandwidth_show(struct device *dev,
                      struct device_attribute *attr,
                      char *buf)
{
    unsigned char data = 0;
    
    return sprintf(buf, "%d\n", k3g_get_bandwidth(&data) ? 0 : data);
}

static ssize_t
sensor_bandwidth_store(struct device *dev,
                       struct device_attribute *attr,
                       const char *buf,
                       size_t count)
{
    unsigned char bw = (char)simple_strtoul(buf, NULL, 10);
    return k3g_set_bandwidth(bw) ? 0 : count;
}

static ssize_t
sensor_mode_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf,
                  size_t count)
{
    unsigned char mode = (char)simple_strtoul(buf, NULL, 10);
    return k3g_set_mode(mode) ? 0 : count;
}

static ssize_t
sensor_range_store(struct device *dev,
                   struct device_attribute *attr,
                   const char *buf,
                   size_t count)
{
    unsigned char range = (char)simple_strtoul(buf, NULL, 10);
    return k3g_set_range(range) ? 0 : count;
}

static ssize_t
sensor_value_show(struct device *dev,
                  struct device_attribute *attr,
                  char *buf)
{
    struct k3g_t data = {0,};
    
    if(!k3g_read_gyro_values(&data))
    {
        memcpy((void*)buf, (void*)&data, sizeof(struct k3g_t));
        return sizeof(struct k3g_t);
    }
    
    return 0;
}

static ssize_t
sensor_selftest_show(struct device *dev,
                     struct device_attribute *attr,
                     char *buf)
{
    // self test for factory test
    return k3g_self_test(dev, attr, buf);
}

#if DEBUG

static ssize_t sensor_debug_suspend_show(struct device *dev,
                                         struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);

    return sprintf(buf, "%d\n", k3g_dev->suspend);
}

static ssize_t sensor_debug_suspend_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
    unsigned long value = simple_strtoul(buf, NULL, 10);

    if (value) {
        suspend();
    } else {
        resume();
    }

    return count;
}
#endif /* DEBUG */

static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP, sensor_delay_show, sensor_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, sensor_enable_show, sensor_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, sensor_wake_store);
static DEVICE_ATTR(data, S_IRUGO, sensor_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, sensor_status_show, NULL);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP, sensor_bandwidth_show, sensor_bandwidth_store);
static DEVICE_ATTR(mode, S_IWUSR|S_IWGRP, NULL, sensor_mode_store);
static DEVICE_ATTR(range, S_IWUSR|S_IWGRP, NULL, sensor_range_store);
static DEVICE_ATTR(value, S_IRUGO, sensor_value_show, NULL);
static DEVICE_ATTR(selftest, S_IRUGO, sensor_selftest_show, NULL);

#if DEBUG
static DEVICE_ATTR(debug_suspend, S_IRUGO|S_IWUSR,
                   sensor_debug_suspend_show, sensor_debug_suspend_store);
#endif /* DEBUG */

static struct attribute *sensor_attributes[] = {
    &dev_attr_poll_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,
    &dev_attr_status.attr,
    &dev_attr_bandwidth.attr,
    &dev_attr_mode.attr,
    &dev_attr_range.attr,
    &dev_attr_value.attr,
    &dev_attr_selftest.attr,
#if DEBUG
    &dev_attr_debug_suspend.attr,
#endif /* DEBUG */
    NULL
};

static struct attribute_group sensor_attribute_group = {
    .attrs = sensor_attributes
};

static int
suspend(void)
{
    if (k3g_dev->enabled) {
        cancel_delayed_work(&k3g_dev->work);
	 printk(" suspend : cancel_delayed_work \n");
    }

	printk("[K3G] suspend \n");
	
    //k3g_set_mode(PM_OFF);

#if DEBUG
    {
        struct sensor_data *data = input_get_drvdata(this_data);
        data->suspend = 1;
    }
#endif /* DEBUG */
    return 0;
}

static int
resume(void)
{
    /* implement resume of the sensor */
    printk(KERN_DEBUG "%s: resume\n", SENSOR_NAME);

	printk("[K3G] resume \n");
	
	//k3g_power_on();

    if (k3g_dev->enabled) {
        k3g_power_on();
        schedule_delayed_work(&k3g_dev->work, delay_to_jiffies(k3g_dev->delay) + 1);
	 printk(" resume : schedule_delayed_work \n");
    }	
    
#if DEBUG
    {
        struct sensor_data *data = input_get_drvdata(this_data);
        k3g_dev->suspend = 0;
    }
#endif /* DEBUG */

    return 0;
}


static int
sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
/*
    struct sensor_data *data = input_get_drvdata(this_data);
    int rt = 0;

    MUTEX_LOCK(&k3g_dev->mutex);

    if (k3g_dev->enabled) {
//        rt = suspend();
    }

    MUTEX_UNLOCK(&k3g_dev->mutex);

    return rt;
    */
    return 0;
}

static int
sensor_resume(struct platform_device *pdev)
{
/*
    struct sensor_data *data = input_get_drvdata(this_data);
    int rt = 0;

    MUTEX_LOCK(&k3g_dev->mutex);

    if (k3g_dev->enabled) {
 //       rt = resume();
    }

    MUTEX_UNLOCK(&k3g_dev->mutex);

    return rt;
    */
    return 0;
}

static int
sensor_probe(struct platform_device *pdev)
{
    struct sensor_data *data = NULL;
    struct input_dev *input_data = NULL;
    int input_registered = 0, sysfs_created = 0;
    int rt;

    /*
     * OK. For now, we presume we have a valid client. We now create the
     * client structure, even though we cannot fill it completely yet.
     */

    printk("[K3G]sensor_probe\n");	
    data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
    if (data == NULL) {
        printk(KERN_ERR "[K3G][%s] Failed to allocate memory for module data\n", __func__);
        return -ENOMEM;
    }

    input_data = input_allocate_device();
    if (!input_data) {
        rt = -ENOMEM;
        printk(KERN_ERR
               "sensor_probe: Failed to allocate input_data device\n");
        goto err;
    }

    data->client = omap_gpio_i2c_init(OMAP_GPIO_SENSOR_I2C_SDA, OMAP_GPIO_SENSOR_I2C_SCL, 0x69, 200);
    if(data->client == NULL)
    {
        printk(KERN_ERR "[K3G][%s] Failed to initialize I2C client!\n", __func__);
        rt = -ENXIO;
        goto err;
    }
    
    data->pdata = kmalloc(sizeof(struct k3g_platform_data), GFP_KERNEL);
    if (data->pdata == NULL)
    {
        rt = -ENOMEM;
        goto err;
    }

    memset(data->pdata, 0, sizeof(struct k3g_platform_data));
    
    rt = k3g_validate_pdata(data);
    if (rt < 0) {
        printk(KERN_ERR "[K3G][%s] Failed to validate platform data\n", __func__);
        goto err;
    }
    
    data->delay = SENSOR_DEFAULT_DELAY;

    /* Initialize registers */
    data->reg[REG_INDEX(CTRL_REG1)] = ODR200_BW50 | 0x0F;
    data->reg[REG_INDEX(CTRL_REG2)] = 0;
    data->reg[REG_INDEX(CTRL_REG3)] = 0;
    data->reg[REG_INDEX(CTRL_REG4)] = BDU_REG_MASK | RANGE_500;
    data->reg[REG_INDEX(CTRL_REG5)] = 0;
    k3g_dev = data;

    if(k3g_detect_and_check_chip_id())
    {
        printk(KERN_ERR "[K3G][%s] Failed to detect and check chipset!\n", __func__);
        goto err;
    }

    /* setup driver interfaces */
    INIT_DELAYED_WORK(&k3g_dev->work, k3g_work_func);
/*	
    work_queue = create_singlethread_workqueue( "k3g_work_queue" ) ;
    if( work_queue < 0 )
    {
        printk("[K3G][%s] Can't create_singlethread_workqueue, ret= %x\n", __func__, (unsigned int)work_queue ) ;
        goto err ;
    }
*/    
    set_bit(EV_REL, input_data->evbit);
    input_set_capability(input_data, EV_REL, REL_X);
#if SENSOR_TYPE <= 4
    input_set_capability(input_data, EV_REL, REL_Y);
    input_set_capability(input_data, EV_REL, REL_Z);
#endif
    input_set_capability(input_data, EV_REL, REL_STATUS); /* status */
    input_set_capability(input_data, EV_REL, REL_WAKE); /* wake */
    input_set_capability(input_data, EV_REL, REL_CONTROL_REPORT); /* enabled/delay */
    input_data->name = SENSOR_NAME;

    rt = input_register_device(input_data);
    if (rt) {
        printk(KERN_ERR
               "sensor_probe: Unable to register input_data device: %s\n",
               input_data->name);
        goto err;
    }
    input_set_drvdata(input_data, data);
    input_registered = 1;

    rt = sysfs_create_group(&input_data->dev.kobj,
            &sensor_attribute_group);
    if (rt) {
        printk(KERN_ERR
               "sensor_probe: sysfs_create_group failed[%s]\n",
               input_data->name);
        goto err;
    }
    sysfs_created = 1;
    mutex_init(&data->mutex);
    this_data = input_data;


	////////////////////for test///////////////////////////
#if 0
	int j;
	struct k3g_t value = {0,};
	
	k3g_power_on();
	
	//for(j=0;j<10;j++) 
	{
		k3g_read_gyro_values(&value);
		

		printk("\n k3g_test X:%d Y:%d Z:%d \n", value.y, value.p,value.r);
	
	}
#endif
	////////////////////for test/////////////////////////// 

    return 0;

err:
    if (data != NULL) {
        if(data->client != NULL) {
            omap_gpio_i2c_deinit(data->client);
        }
        if(data->pdata != NULL) {
            kfree(data->pdata);
        }
        if (input_data != NULL) {
            if (sysfs_created) {
                sysfs_remove_group(&input_data->dev.kobj,
                        &sensor_attribute_group);
            }
            if (input_registered) {
                input_unregister_device(input_data);
            }
            else {
                input_free_device(input_data);
            }
            input_data = NULL;
        }
        kfree(data);
    }

    return rt;
}

static int
sensor_remove(struct platform_device *pdev)
{
    struct sensor_data *data;

    destroy_workqueue(work_queue);

    if (this_data != NULL) {        
        data = input_get_drvdata(this_data);
        if(data->client != NULL) {
            omap_gpio_i2c_deinit(data->client);
        }
        sysfs_remove_group(&this_data->dev.kobj,
                &sensor_attribute_group);
        input_unregister_device(this_data);
        if (data->pdata != NULL) {
            kfree(data->pdata);
        }
        if (data != NULL) {
            kfree(data);
        }
    }

    return 0;
}

/*
 * Module init and exit
 */
static struct platform_driver sensor_driver = {
    .probe      = sensor_probe,
    .remove     = sensor_remove,
    .suspend    = sensor_suspend,
    .resume     = sensor_resume,
    .driver = {
        .name   = SENSOR_NAME,
        .owner  = THIS_MODULE,
    },
};

static int __init sensor_init(void)
{
    sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
    if (IS_ERR(sensor_pdev)) {
        return -1;
    }
    return platform_driver_register(&sensor_driver);
}
module_init(sensor_init);

static void __exit sensor_exit(void)
{
    platform_driver_unregister(&sensor_driver);
    platform_device_unregister(sensor_pdev);
}
module_exit(sensor_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.2.0");
