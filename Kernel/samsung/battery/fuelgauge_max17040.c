/*
 * module/samsung_battery/fuelgauge_max17040.c
 *
 * SAMSUNG battery driver for Linux
 *
 * Copyright (C) 2009 SAMSUNG ELECTRONICS.
 * Author: EUNGON KIM (egstyle.kim@samsung.com)
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <mach/gpio.h>
#include <linux/power_supply.h>
#include "common.h"

#if defined(CONFIG_USE_GPIO_I2C)
    #include <plat/i2c-omap-gpio.h>
#endif    

#define DRIVER_NAME "secFuelgaugeDev"

#define I2C_M_WR    0x00

#define REG_VCELL   0x02
#define REG_SOC     0x04
#define REG_MODE    0x06
#define REG_VERSION 0x08
#define REG_RCOMP   0x0C
#define REG_CONFIG  0x0D
#define REG_COMMAND 0xFE


#if defined(CONFIG_USE_GPIO_I2C)
    static OMAP_GPIO_I2C_CLIENT * fuelgauge_i2c_client;
	static struct i2c_client * fuelgauge_i2c_dummy_client;
#else
    static struct i2c_client * fuelgauge_i2c_client;
#endif

struct delayed_work fuelgauge_work;

static SEC_battery_charger_info *sec_bci;

// Prototype
       int get_fuelgauge_adc_value( int, bool );
       int get_fuelgauge_ptg_value( bool );
       int fuelgauge_quickstart( void );
static int i2c_read( unsigned char );
static int i2c_write( unsigned char *, u8 );
#if !defined(CONFIG_USE_GPIO_I2C)
static int i2c_read_dur_sleep( unsigned char );
#endif
static irqreturn_t low_battery_isr( int, void * );
static void fuelgauge_work_handler( struct work_struct * );
static int fuelgauge_probe( struct i2c_client *, const struct i2c_device_id * );
static int fuelgauge_remove( struct i2c_client * );
static void fuelgauge_shutdown( struct i2c_client * );
static int fuelgauge_suspend( struct i2c_client * , pm_message_t );
static int fuelgauge_resume( struct i2c_client * );
       int fuelgauge_init( void );
       void fuelgauge_exit( void );

extern SEC_battery_charger_info *get_sec_bci( void );
extern int _low_battery_alarm_( void );
extern s32 normal_i2c_read_word( u8 , u8 , u8 * );
extern s32 t2_read_word(u8 , u8 , u8 *);

static const struct i2c_device_id fuelgauge_i2c_id[] = {
    { DRIVER_NAME, 0 },
    { },
};

static struct i2c_driver fuelgauge_i2c_driver =
{
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
    },       

    .probe      = fuelgauge_probe,
    .remove     = __devexit_p( fuelgauge_remove ),
    .shutdown   = fuelgauge_shutdown,
    .suspend    = fuelgauge_suspend,
    .resume     = fuelgauge_resume,
    .id_table   = fuelgauge_i2c_id,
};

int get_fuelgauge_adc_value( int count, bool is_sleep )
{
    int result;
    
#if !defined(CONFIG_USE_GPIO_I2C)
    if( is_sleep )
        result = i2c_read_dur_sleep( REG_VCELL );
    else
        result = i2c_read( REG_VCELL );
#else
    result = i2c_read( REG_VCELL );
#endif

    result = ( result >> 4 ) * 125 / 100;
    return result;
}

int get_fuelgauge_ptg_value( bool is_sleep )
{
    int val;

#if !defined(CONFIG_USE_GPIO_I2C)
    if( is_sleep )
        val = i2c_read_dur_sleep( REG_SOC ); 
    else
        val = i2c_read( REG_SOC );
#else
    val = i2c_read( REG_SOC );
#endif

    if ((val & 0x0F) >= 32)
        val = (val >> 8) + 1;
    else
        val = (val >> 8);

    //if val is lower than -1%, then soc is 0%
    if (val <= -1)
        val = 0;

    //printk("[FG] get_fuelgauge_ptg_value %d\n", val);
    return ( val > 100 ) ? 100 : val;
}

int fuelgauge_quickstart( void )
{
    unsigned char buf[3];

    buf[0] = REG_MODE;
    buf[1] = 0x40;
    buf[2] = 0x00;
    i2c_write( buf, 3 );

    return 0;
}

#if 0
int update_rcomp_by_temperature(int temp)
{
    static unsigned int appliedRcomp = 0;

    int tempCoHot = -75;
    int tempCoCold = -75;
    unsigned int newRcomp = 0;

    int ret = 0;
    unsigned char buf[3];

    if(temp > 20)
        newRcomp = startingRcomp + (((temp - 20) * tempCoHot)/100);
    else if(temp < 20)
        newRcomp = startingRcomp + (((temp - 20) * tempCoCold)/100);
    else
        newRcomp = startingRcomp;

    if(newRcomp != appliedRcomp)
    {
        ret = i2c_read( REG_RCOMP );
        buf[0] = REG_RCOMP;
        buf[1] = newRcomp;
        buf[2] = ret & 0xFF;
        i2c_write( buf, 3 );

        appliedRcomp = newRcomp;
        printk("%s, buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x\n", buf[0], buf[1], buf[2]);
        return 1;
    }

    return 0;
}
#endif

static int i2c_read( unsigned char reg_addr )
{
    int ret = 0;
    unsigned char buf[2];

#if defined(CONFIG_USE_GPIO_I2C)
    OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
#else
    struct i2c_msg msg1[1],msg2[1];
#endif

#if defined(CONFIG_USE_GPIO_I2C)
    i2c_rd_param.reg_len = 1;
    i2c_rd_param.reg_addr = &reg_addr;
    i2c_rd_param.rdata_len = 2;
    i2c_rd_param.rdata = buf;
    omap_gpio_i2c_read(fuelgauge_i2c_client, &i2c_rd_param);
#else
    msg1->addr = fuelgauge_i2c_client->addr;
    msg1->flags = I2C_M_WR;
    msg1->len = 1;
    msg1->buf = &reg_addr;

    ret = i2c_transfer(fuelgauge_i2c_client->adapter, msg1, 1);
    if( ret < 0 )
    {
        printk( KERN_ERR "[FG] fail to read max17040." );
        return -1;
    }
    else
    {
        msg2->addr = fuelgauge_i2c_client->addr;
        msg2->flags = I2C_M_RD;
        msg2->len = 2;
        msg2->buf = buf;

        ret = i2c_transfer( fuelgauge_i2c_client->adapter, msg2, 1 );

        if( ret < 0 )
        {
            printk( KERN_ERR "[FG] fail to read max17040." );
            return -1;
        }
    }
#endif

    ret = buf[0] << 8 | buf[1];

    return ret;
}

static int i2c_write( unsigned char *buf, u8 len )
{
    int ret = 0;

#if defined(CONFIG_USE_GPIO_I2C)
    OMAP_GPIO_I2C_WR_DATA i2c_wr_param;
#else
    struct i2c_msg msg;
#endif

#if !defined(CONFIG_USE_GPIO_I2C)
    msg.addr    = fuelgauge_i2c_client->addr;
    msg.flags = I2C_M_WR;
    msg.len = len;
    msg.buf = buf;

    ret = i2c_transfer( fuelgauge_i2c_client->adapter, &msg, 1 );

    if( ret < 0 )
    {
        printk( KERN_ERR "[FG] fail to write max17040." );
        return -1;
    }
#else
#if 0
    i2c_wr_param.reg_len = 0;
    i2c_wr_param.reg_addr = NULL;
    i2c_wr_param.wdata_len = len;
    i2c_wr_param.wdata = buf;
#else
    i2c_wr_param.reg_len = 1;
    i2c_wr_param.reg_addr = &(buf[0]);
    i2c_wr_param.wdata_len = len;
    i2c_wr_param.wdata = &(buf[1]);	
    omap_gpio_i2c_write(fuelgauge_i2c_client, &i2c_wr_param);
#endif
#endif

    return ret;
}

#if !defined(CONFIG_USE_GPIO_I2C)
static int i2c_read_dur_sleep( unsigned char reg_addr )
{
    unsigned char buf[2];
    unsigned int ret;

    ret = t2_read_word(0x36, reg_addr, buf);
    if(ret < 0)
    {
        printk(KERN_ERR"[%s] Fail to Read max17040\n", __FUNCTION__);
        return -1;
    } 

    ret = buf[0] << 8 | buf[1];

    return ret;
}
#endif

static irqreturn_t low_battery_isr( int irq, void *_di )
{
    if ( sec_bci->ready )
    {
        cancel_delayed_work( &fuelgauge_work );
        schedule_delayed_work( &fuelgauge_work, 0 );
    }
    
    return IRQ_HANDLED; 
}

static void fuelgauge_work_handler( struct work_struct *work )
{
    printk( "[FG] ext.low_battery!\n" );
    _low_battery_alarm_();
}

static int fuelgauge_probe( struct i2c_client *client, 
                            const struct i2c_device_id *id )
{
    int ret = 0;
    unsigned char buf[3];

    printk( "[FG] Fuelgauge Probe.\n" );

    sec_bci = get_sec_bci();

    if( strcmp( client->name, DRIVER_NAME) != 0 )
    {
        ret = -1;
        printk( "[FG] device not supported.\n" );
    }

    #if defined(CONFIG_USE_GPIO_I2C)
	fuelgauge_i2c_dummy_client = client;
    #else
    fuelgauge_i2c_client = client;
    #endif

    if ( client->irq )
    {
        INIT_DELAYED_WORK( &fuelgauge_work, fuelgauge_work_handler );

        // set alert threshold to 1%
        ret = i2c_read( REG_RCOMP );
        buf[0] = REG_RCOMP;
        buf[1] = ret >> 8;
        buf[2] = 0x1F; // 1% -> Refer Fuel guage datasheet.
        i2c_write( buf, 3 );

        ret = i2c_read( REG_RCOMP );
        printk( "[FG] val : %x \n", ret );

        ret = irq_to_gpio( client->irq );
        printk( "[FG] FUEL_INT_GPIO : %d \n", ret );

        set_irq_type( client->irq, IRQ_TYPE_EDGE_FALLING );
        ret = request_irq( client->irq, low_battery_isr, IRQF_DISABLED, client->name, NULL );
        if ( ret )
        {
            printk( "[FG] could not request irq %d, status %d\n", client->irq, ret );
        }
    }

    sec_bci->charger.fuelgauge_full_soc = 95;  // for adjust fuelgauge

    return ret;
}

static int fuelgauge_remove( struct i2c_client * client )
{
    return 0;
}

static void fuelgauge_shutdown( struct i2c_client * client )
{
    return;
}

static int fuelgauge_suspend( struct i2c_client * client , pm_message_t mesg )
{
    int batt_ptg = 0;
    int ret = 0;

    unsigned char alert_th;
    unsigned char buf[3];

    if(!sec_bci->charger.is_charging)
    {
        // When chargine battery, set alert threshold by using remained battery percentage.
        batt_ptg = get_fuelgauge_ptg_value(false);

        if(batt_ptg >= 15)
            alert_th = 0x11; // 15%
        else if(batt_ptg >= 4)
            alert_th = 0x1C; // 4%
        else
            alert_th = 0x1F; // 1%

	    ret = i2c_read( REG_RCOMP );
	    buf[0] = REG_RCOMP;
	    buf[1] = ret >> 8;
	    buf[2] = alert_th;
	    i2c_write( buf, 3 );
	    printk("%s, set alert threshold 0x%2x\n", __func__, alert_th);
    }

    return 0;
}

static int fuelgauge_resume( struct i2c_client * client )
{
    int ret = 0;
    unsigned char buf[3];

    ret = i2c_read( REG_RCOMP );
    buf[0] = REG_RCOMP;
    buf[1] = ret >> 8;
    buf[2] = 0x1F;
    i2c_write( buf, 3 );
    printk("%s, set alert threshold 1%\n", __func__);

    return 0;
}

int fuelgauge_init( void )
{
    int ret;

#if defined(CONFIG_USE_GPIO_I2C)
    fuelgauge_i2c_client = omap_gpio_i2c_init(OMAP_GPIO_FUEL_SDA,OMAP_GPIO_FUEL_SCL, 0x36, 100);

    if(fuelgauge_i2c_client == NULL)
    {
        printk(KERN_ERR "[FG] omap_gpio_i2c_init failed!\n");
    }

	printk("[FG] Fuelgauge Init. add dummy i2c driver!\n");
	if( ( ret = i2c_add_driver( &fuelgauge_i2c_driver ) < 0 ) )
	{
		printk( KERN_ERR "[FG] i2c_add_driver failed.\n" );    
	}
#else
    printk("[FG] Fuelgauge Init. add i2c driver!\n");
    if( ( ret = i2c_add_driver( &fuelgauge_i2c_driver ) < 0 ) )
    {
        printk( KERN_ERR "[FG] i2c_add_driver failed.\n" );
    }
#endif

    return ret;    
}

void fuelgauge_exit( void )
{
    printk("[FG] Fuelgauge Exit.\n");

#if defined(CONFIG_USE_GPIO_I2C)
    omap_gpio_i2c_deinit(fuelgauge_i2c_client);
#endif
    i2c_del_driver( &fuelgauge_i2c_driver );

}
