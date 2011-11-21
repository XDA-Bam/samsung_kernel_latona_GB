
#include <linux/kernel.h>
#include <linux/i2c.h>

#include "Si4709_dev.h"
#include "common.h"


/*extern functions*/
int Si4709_i2c_drv_init(void);
void Si4709_i2c_drv_exit(void);

/*static functions*/
static int Si4709_probe (struct i2c_client *);
static int Si4709_remove(struct i2c_client *);
static int Si4709_suspend(struct i2c_client *, pm_message_t mesg);
static int Si4709_resume(struct i2c_client *);

static const struct i2c_device_id Si4709_i2c_id[] = {
 { "Si4709_driver", 0 },
 {},
};

static struct i2c_driver Si4709_i2c_driver =
{
    .driver = {
        .name = "Si4709_driver",
	.owner = THIS_MODULE,
    },
    .id_table   = Si4709_i2c_id,    
	
    .probe = Si4709_probe,
    .remove = Si4709_remove,

    .suspend = &Si4709_suspend,
    .resume = &Si4709_resume,
};


static int Si4709_probe (struct i2c_client *client)
{
    int ret = 0;

    debug("Si4709 i2c driver Si4709_probe called"); 

    if( strcmp(client->name, "Si4709_driver") != 0 )
    {
        ret = -1;
        debug("Si4709_probe: device not supported");
    }
    else if( (ret = Si4709_dev_init(client)) < 0 )
    {
        debug("Si4709_dev_init failed");
    }

    return ret;
}


static int Si4709_remove(struct i2c_client *client)
{
    int ret = 0;

    debug("Si4709 i2c driver Si4709_remove called"); 

    if( strcmp(client->name, "Si4709") != 0 )
    {
        ret = -1;
        debug("Si4709_remove: device not supported");
    }
    else if( (ret = Si4709_dev_exit()) < 0 )
    {
        debug("Si4709_dev_exit failed");
    }

    return ret;
}

static int Si4709_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret = 0;
	   
    debug("Si4709 i2c driver Si4709_suspend called"); 

    if( strcmp(client->name, "Si4709") != 0 )
    {
        ret = -1;
        debug("Si4709_suspend: device not supported");
    }
    else if( (ret = Si4709_dev_suspend()) < 0 )
    {
        debug("Si4709_dev_disable failed");
    }

    return 0;
}

static int Si4709_resume(struct i2c_client *client)
{
    int ret = 0;
	   
//    debug("Si4709 i2c driver Si4709_resume called"); 

    if( strcmp(client->name, "Si4709") != 0 )
    {
        ret = -1;
        debug("Si4709_resume: device not supported");
    }
    else if( (ret = Si4709_dev_resume()) < 0 )
    {
        debug("Si4709_dev_enable failed");
    }
 
    return 0;
}

int Si4709_i2c_drv_init(void)
{	
    int ret = 0;

    debug("Si4709 i2c driver Si4709_i2c_driver_init called"); 

    if ( (ret = i2c_add_driver(&Si4709_i2c_driver) < 0) ) 
    {
        error("Si4709 i2c_add_driver failed");
    }

    return ret;
}

void Si4709_i2c_drv_exit(void)
{
    debug("Si4709 i2c driver Si4709_i2c_driver_exit called"); 

    i2c_del_driver(&Si4709_i2c_driver);
}


