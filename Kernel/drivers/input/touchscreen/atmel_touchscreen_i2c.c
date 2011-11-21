#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>
//#include <linux/slab_def.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>
//#include <asm/arch/gpio.h>
//#include <asm/arch/mux.h>
#include <plat/gpio.h>	//ryun
#include <plat/mux.h>	//ryun 

//#include <asm/arch/hardware.h>
#include <mach/hardware.h>	// ryun
#include <linux/types.h>

#include "atmel_touch.h"

#define __CONFIG_ATMEL__

#define I2C_M_WR 0	// ryun

#ifdef __CONFIG_ATMEL__
#define u8	__u8
#endif

static int i2c_tsp_sensor_attach_adapter(struct i2c_adapter *adapter);
static int i2c_tsp_sensor_probe_client(struct i2c_adapter *adapter, int address, int kind);
static int i2c_tsp_sensor_detach_client(struct i2c_client *client);

//#define TSP_SENSOR_ADDRESS		0x4b	// onegun test

//#define CYPRESS_TSP_SENSOR_ADDRESS			0x20
//#define SYNAPTICS_TSP_SENSOR_ADDRESS			0x2C
#define ATMEL_TSP_SENSOR_ADDRESS			0x4A


extern u32 hw_revision;
#define IS_ATMEL	1


#define I2C_DF_NOTIFY				0x01


struct i2c_driver tsp_sensor_driver =
{
	//.name	= "tsp_sensor_driver",
	.driver	= {
		.name	= "tsp_driver",
		.owner	= THIS_MODULE,
	},
	//.flags	= I2C_DF_NOTIFY | I2C_M_IGNORE_NAK,
	.attach_adapter	= &i2c_tsp_sensor_attach_adapter,
	//.detach_client	= &i2c_tsp_sensor_detach_client,
	.remove	= &i2c_tsp_sensor_detach_client,
};


static struct i2c_client *g_client;

#ifdef __CONFIG_ATMEL__
#define U16	unsigned short int
#define U8	unsigned char

#endif

#if 1
extern unsigned int g_i2c_debugging_enable;

int i2c_tsp_sensor_read(u16 reg, u8 *read_val, unsigned int len )
{
	int id;
	int 	 err, i;
	struct 	 i2c_msg msg[1];
	unsigned char data[2];
	
	if( (g_client == NULL) || (!g_client->adapter) )
	{
		return -ENODEV;
	}
	
	msg->addr 	= g_client->addr;
	msg->flags 	= I2C_M_WR;
	msg->len 	= 2;
	msg->buf 	= data;
	data[0] = reg & 0x00ff;
	data[1] = reg >> 8;
	
	

	err = i2c_transfer(g_client->adapter, msg, 1);
#if 1//for debug
	if(g_i2c_debugging_enable == 1)
	{
		printk(KERN_DEBUG "[TSP][I2C] read addr[0] = 0x%x, addr[1] = 0x%x\n", data[0], data[1]);
	}
#endif

	if (err >= 0) 
	{
		msg->flags = I2C_M_RD;
		msg->len   = len;
		msg->buf   = read_val;
		err = i2c_transfer(g_client->adapter, msg, 1);

#if 1//for debug
		if(g_i2c_debugging_enable == 1)
		{
			printk(KERN_DEBUG "[TSP][I2C] read data = ");
			for(i=0 ; i<len ; i++)
			{
				printk(KERN_DEBUG "%d(0x%x), ", msg->buf[i],  msg->buf[i]);
			}
			printk(KERN_DEBUG "// len = %d, rtn=%d\n",msg->len, err);
		}
#endif
	}

	if (err >= 0) 
	{
		return 0;
	}

	id = i2c_adapter_id(g_client->adapter);
//	printk("[TSP] %s() - end\n", __FUNCTION__);
	return err;
}
#else
int i2c_tsp_sensor_read(u8 reg, u8 *val, unsigned int len )
{
	int id;
	int 	 err;
	struct 	 i2c_msg msg[1];
	unsigned char data[1];

	if( (g_client == NULL) || (!g_client->adapter) )
	{
		return -ENODEV;
	}
	
	msg->addr 	= g_client->addr;
	msg->flags 	= I2C_M_WR;
	msg->len 	= 1;
	msg->buf 	= data;
	*data       = reg;

	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) 
	{
		msg->flags = I2C_M_RD;
		msg->len   = len;
		msg->buf   = val;
		err = i2c_transfer(g_client->adapter, msg, 1);
	}

	if (err >= 0) 
	{
		return 0;
	}

	id = i2c_adapter_id(g_client->adapter);

	return err;
}

#endif
#define I2C_MAX_SEND_LENGTH	300
int i2c_tsp_sensor_write(u16 reg
	, u8 *read_val, unsigned int len)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[300];
	int i ;

	if( (g_client == NULL) || (!g_client->adapter) )
		return -ENODEV;

	if(len +2 > I2C_MAX_SEND_LENGTH)
	{
		printk("[TSP][ERROR] %s() data length error\n", __FUNCTION__);
		return -ENODEV;
	}
		
	msg->addr = g_client->addr;
	msg->flags = I2C_M_WR;
	msg->len = len + 2;
	msg->buf = data;
	data[0] = reg & 0x00ff;
	data[1] = reg >> 8;
	

	for (i = 0; i < len; i++)
	{
		data[i+2] = *(read_val+i);
	}

	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) return 0;

	return err;
}
void i2c_tsp_sensor_write_reg(u8 address, int data)
{
	u8 i2cdata[1];

	i2cdata[0] = data;
	i2c_tsp_sensor_write(address, i2cdata, 1);
}

static int i2c_tsp_sensor_attach_adapter(struct i2c_adapter *adapter)
{
	int addr = 0;
	int id = 0;

//	if(IS_ATMEL)
		addr = ATMEL_TSP_SENSOR_ADDRESS;
//	else
//		addr = CYPRESS_TSP_SENSOR_ADDRESS;
	//return i2c_probe(adapter, &addr_data, &i2c_tsp_sensor_probe_client);

	id = adapter->nr;
	/*
#if defined(CONFIG_MACH_NOWPLUS) || defined(CONFIG_MACH_NOWPLUS_MASS)
	if (id == 3)
#else
#if (CONFIG_ACME_REV >= CONFIG_ACME_REV04)
	if (id == 3)
#else
	if (id == 2)
#endif
#endif
*/
	if (id == 3)	// ryun 20091125 !!! ??? 
		return i2c_tsp_sensor_probe_client(adapter, addr, 0);
	return 0;
}

static int i2c_tsp_sensor_probe_client(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	int err = 0;
	printk(KERN_DEBUG "[TSP] %s() - start\n", __FUNCTION__);	// ryun 20091126

	if ( !i2c_check_functionality(adapter,I2C_FUNC_SMBUS_BYTE_DATA) ) {
		printk("byte op is not permited.\n");
		goto ERROR0;
	}

	new_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL );

	if ( !new_client )	{
		err = -ENOMEM;
		goto ERROR0;
	}

	new_client->addr = address; 
	printk(KERN_DEBUG "%s :: addr=%x\n", __FUNCTION__, address);
	new_client->adapter = adapter;
	new_client->driver = &tsp_sensor_driver;
	//new_client->flags = I2C_DF_NOTIFY | I2C_M_IGNORE_NAK;

	g_client = new_client;

	strlcpy(new_client->name, "tsp_driver", I2C_NAME_SIZE);

//	if ((err = i2c_attach_client(new_client)))	// ryun 20091126 move to board-xxx.c
//		goto ERROR1;
	printk(KERN_DEBUG "[TSP] %s() - end : success\n", __FUNCTION__);	// ryun 20091126
	return 0;

//	ERROR1:
//		printk("[TSP] %s() - end : fail !!!!!!!!!!!!!!!!!!err = %d  \n", __FUNCTION__, err);	// ryun 20091126
//		kfree(new_client);
	ERROR0:
		return err;
}

static int i2c_tsp_sensor_detach_client(struct i2c_client *client)
{
	
	printk(KERN_DEBUG "[TSP] %s() - start\n", __FUNCTION__);	// ryun 20091126
  	/* Try to detach the client from i2c space */
	//if ((err = i2c_detach_client(client))) {
        //return err;
	//}
	i2c_set_clientdata(client,NULL);

	kfree(client); /* Frees client data too, if allocated at the same time */
	g_client = NULL;
	return 0;
}


int i2c_tsp_sensor_init(void)
{
	int ret;

	if ( (ret = i2c_add_driver(&tsp_sensor_driver)) ) 
	{
		printk("TSP I2C Driver registration failed!\n");
		return ret;
	}

	return 0;
}
uint8_t read_boot_state(u8 *data)
{
	struct i2c_msg rmsg;
	int ret;

	rmsg.addr = QT602240_I2C_BOOT_ADDR ;
	rmsg.flags = I2C_M_RD;
	rmsg.len = 1;
	rmsg.buf = data;
	ret = i2c_transfer(g_client->adapter, &rmsg, 1);

	if ( ret < 0 )
	{
		printk("[TSP] %s,%d - Fail!!!! ret = %d\n", __func__, __LINE__, ret );
		return READ_MEM_FAILED; 
	}	
	else 
	{
		return READ_MEM_OK;
	}
}

uint8_t boot_unlock(void)
{
	struct i2c_msg wmsg;
	int ret;
	unsigned char data[2];

	printk(KERN_DEBUG "[TSP] %s - start, %d\n", __func__, __LINE__ );

	data[0] = 0xDC;
	data[1] = 0xAA;

	wmsg.addr = QT602240_I2C_BOOT_ADDR ;
	wmsg.flags = I2C_M_WR;
	wmsg.len = 2;
	wmsg.buf = data;

	ret = i2c_transfer(g_client->adapter, &wmsg, 1);

	if ( ret >= 0 )
		return WRITE_MEM_OK;
	else 
		return WRITE_MEM_FAILED; 
}

uint8_t boot_write_mem(uint16_t ByteCount, unsigned char * Data )
{
	struct i2c_msg wmsg;
//	unsigned char data[I2C_MAX_SEND_LENGTH];
	int ret;
//	int i;

	wmsg.addr = QT602240_I2C_BOOT_ADDR ;
	wmsg.flags = I2C_M_WR;
	wmsg.len = ByteCount;
	wmsg.buf = Data;

	ret = i2c_transfer(g_client->adapter, &wmsg, 1);
	if ( ret >= 0 )
	{
		return WRITE_MEM_OK;
	}	
	else 
	{
		printk("[TSP] %s,%d - Fail!!!!\n", __func__, __LINE__ );
		return WRITE_MEM_FAILED; 
	}
}

