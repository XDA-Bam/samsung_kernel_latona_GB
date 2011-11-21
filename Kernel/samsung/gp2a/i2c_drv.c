/*
 * i2c_drv.c 
 *
 * Description: This file contains the implementation of the i2c driver 
 *
 * Author: Varun Mahajan <m.varun@samsung.com>
 */

#include <linux/kernel.h>
#include <linux/i2c.h>

#include "P_dev.h"
#include "L_dev.h"
#include "common.h"

/*extern functions*/
int PL_i2c_drv_init(void);
void PL_i2c_drv_exit(void);

/*static functions*/
static int PL_probe (struct i2c_client *,  const struct i2c_device_id *);
static int PL_remove(struct i2c_client *);
static int PL_suspend(struct i2c_client *, pm_message_t mesg);
static int PL_resume(struct i2c_client *);

static struct i2c_device_id PL_i2c_idtable[] = {
	{ "gp2a", 0 },
	{ }
};

static struct i2c_driver PL_i2c_driver =
{
    .driver = {
        .name = "gp2a",
	.owner = THIS_MODULE,
    },
    .id_table = PL_i2c_idtable,
    .probe = PL_probe,
    .remove = PL_remove,

    .suspend = PL_suspend,
    .resume = PL_resume,
};

static int PL_probe (struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;

    trace_in();

    if( strcmp(client->name, DEVICE_NAME) != 0 )
    {
        ret = -1;
        failed(1);
        error("device not supported");
    }
    else if( (ret = P_dev_init(client)) < 0 )
    {
        failed(2);
    }
    else if( (ret = L_dev_init()) < 0 )
    {
        failed(3);
    }

    trace_out();

    return ret;
}

static int PL_remove(struct i2c_client *client)
{
    int ret = 0;

    trace_in();

    if( strcmp(client->name, DEVICE_NAME) != 0 )
    {
        ret = -1;
        failed(1);
        error("device not supported");
    }
    else if( (ret = P_dev_exit()) < 0 )
    {
        failed(2);
    }
    else if( (ret = L_dev_exit()) < 0 )
    {
        failed(3);
    }

    trace_out();

    return ret;
}

static int PL_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret = 0;
	   
    trace_in();

    if( strcmp(client->name, DEVICE_NAME) != 0 )
    {
        ret = -1;
        failed(1);
        error("device not supported");
    }
    else if( (ret = L_dev_suspend()) < 0 )
    {
        failed(2);
    }

    trace_out();

    return 0;
}

static int PL_resume(struct i2c_client *client)
{
    int ret = 0;
	   
    trace_in();

    if( strcmp(client->name, DEVICE_NAME) != 0 )
    {
        ret = -1;
        failed(1);
        error("device not supported");
    }
    else 
    {
        P_dev_check_wakeup_src();
        
        if( (ret = L_dev_resume()) < 0 )
        {
            failed(3);
        }
    }

    trace_out();
 
    return ret;
}

int PL_i2c_drv_init(void)
{	
    int ret = 0;

    trace_in();

    if ( (ret = i2c_add_driver(&PL_i2c_driver) < 0) ) 
    {
        failed(1);
        error("i2c_add_driver failed");
    }
	debug("[ryun] PL_i2c_drv_init(void) : ret=%d \n", ret);
    trace_out();

    return ret;
}

void PL_i2c_drv_exit(void)
{
    trace_in();

    i2c_del_driver(&PL_i2c_driver);

    trace_out();
}

