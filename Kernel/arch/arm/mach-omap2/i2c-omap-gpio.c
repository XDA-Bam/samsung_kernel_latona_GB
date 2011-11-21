
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <plat/i2c-omap-gpio.h>
#include <linux/slab.h>

#define GPIO_LEVEL_HIGH 1
#define GPIO_LEVEL_LOW  0

#define I2C_M_WR	0

static DEFINE_MUTEX(omap_gpio_i2c_mutex);

//
// omap_gpio_i2c_init
// 
OMAP_GPIO_I2C_CLIENT * omap_gpio_i2c_init(const int sda, const int scl, const int addr, const int khz)
{
	// allocate memory space for internal structure
	OMAP_GPIO_I2C_CLIENT * client = (OMAP_GPIO_I2C_CLIENT *)kmalloc(sizeof(OMAP_GPIO_I2C_CLIENT), GFP_KERNEL);

	if(client == NULL)
	{
		printk(KERN_ERR "[%s] Failed to allocate memory for client!\n", __func__);
		return NULL;
	}
	
	mutex_lock(&omap_gpio_i2c_mutex);

	// verify/request SCL and set direction
	if(gpio_is_valid(scl)) 
	{
		if(gpio_request(scl, NULL))
		{
			printk(KERN_ERR "[%s] SCL line was requested by other module\n", __func__);
		}
		gpio_direction_output(scl, GPIO_LEVEL_HIGH);
	}

	// verify/request SDA and set direction
	if(gpio_is_valid(sda)) 
	{
		if(gpio_request(sda, NULL))
		{
			printk(KERN_ERR "[%s] SDA line was requested by other module!\n", __func__);
		}
		gpio_direction_output(sda, GPIO_LEVEL_HIGH);
	}
	
	// calculate delay between the wave in SCL
	client->delay   = 1000 / khz;
	client->scl 	= scl;
	client->sda	= sda;
	client->addr    = addr;

	mutex_unlock(&omap_gpio_i2c_mutex);

	return client;
}

EXPORT_SYMBOL(omap_gpio_i2c_init);

//
// omap_gpio_i2c_deinit
//
void omap_gpio_i2c_deinit(OMAP_GPIO_I2C_CLIENT * client)
{
	if(client == NULL) return;

	mutex_lock(&omap_gpio_i2c_mutex);
	gpio_free(client->scl);
	gpio_free(client->sda);
	mutex_unlock(&omap_gpio_i2c_mutex);

}

EXPORT_SYMBOL(omap_gpio_i2c_deinit);

/* i2c functions */

//
// omap_gpio_i2c_send_start
//
static void omap_gpio_i2c_send_start(OMAP_GPIO_I2C_CLIENT * client)
{
	gpio_set_value(client->sda, GPIO_LEVEL_LOW);
	udelay(client->delay);

	gpio_set_value(client->scl, GPIO_LEVEL_LOW);
	udelay(client->delay);
}

//
// omap_gpio_i2c_send_stop
//
static void omap_gpio_i2c_send_stop(OMAP_GPIO_I2C_CLIENT * client)
{
	gpio_set_value(client->sda, GPIO_LEVEL_LOW);
	udelay(client->delay);

	gpio_set_value(client->scl, GPIO_LEVEL_HIGH);
	udelay(client->delay);

	gpio_set_value(client->sda, GPIO_LEVEL_HIGH);
	udelay(client->delay);
}

//
// omap_gpio_i2c_send_byte
// 
static void omap_gpio_i2c_send_byte(OMAP_GPIO_I2C_CLIENT * client, unsigned char byte)
{
	int i = 0;

	for(i = 7; i >= 0; i--)
	{
		if((byte >> i) & 0x1)
			gpio_set_value(client->sda, GPIO_LEVEL_HIGH);
		else
			gpio_set_value(client->sda, GPIO_LEVEL_LOW);

		udelay(1);

		gpio_set_value(client->scl, GPIO_LEVEL_HIGH);
		udelay(client->delay);

		gpio_set_value(client->scl, GPIO_LEVEL_LOW);
		udelay(client->delay);
	}
}

//
// omap_gpio_i2c_read_byte
//
static unsigned char omap_gpio_i2c_read_byte(OMAP_GPIO_I2C_CLIENT * client)
{
	int i = 0;
	int byte = 0;

	gpio_direction_input(client->sda);

	for(i = 7; i >= 0; i--)
	{
		gpio_set_value(client->scl, GPIO_LEVEL_HIGH);
		udelay(client->delay);

		if(gpio_get_value(client->sda))
			byte |= (1 << i);

		gpio_set_value(client->scl, GPIO_LEVEL_LOW);
		udelay(client->delay);
	}

	gpio_direction_output(client->sda, GPIO_LEVEL_HIGH);
	udelay(client->delay);

	return byte;
}

//
// omap_gpio_i2c_send_ack
//
static void omap_gpio_i2c_send_ack(OMAP_GPIO_I2C_CLIENT * client)
{
	gpio_set_value(client->sda, GPIO_LEVEL_LOW);
	udelay(1);

	gpio_set_value(client->scl, GPIO_LEVEL_HIGH);
	udelay(client->delay);

	gpio_set_value(client->scl, GPIO_LEVEL_LOW);
	udelay(client->delay);

	gpio_set_value(client->sda, GPIO_LEVEL_HIGH);
	udelay(client->delay);
}

//
// omap_gpio_i2c_send_nack
//
static void omap_gpio_i2c_send_nack(OMAP_GPIO_I2C_CLIENT * client)
{
	gpio_set_value(client->sda, GPIO_LEVEL_HIGH);
	udelay(1);

	gpio_set_value(client->scl, GPIO_LEVEL_HIGH);
	udelay(client->delay);

	gpio_set_value(client->scl, GPIO_LEVEL_LOW);
	udelay(client->delay);
}

//
// omap_gpio_i2c_poll_ack
//
static unsigned char omap_gpio_i2c_poll_ack(OMAP_GPIO_I2C_CLIENT * client)
{
	int ack = 0;

	gpio_set_value(client->sda, GPIO_LEVEL_HIGH);

	gpio_direction_input(client->sda);
	udelay(client->delay);

	gpio_set_value(client->scl, GPIO_LEVEL_HIGH);
	udelay(client->delay);

	ack = gpio_get_value(client->sda) ? 0 : 1;

	gpio_set_value(client->scl, GPIO_LEVEL_LOW);
    
	gpio_direction_output(client->sda, GPIO_LEVEL_LOW);
	udelay(client->delay);

	return ack;
}

//
// omap_gpio_i2c_write
//
int omap_gpio_i2c_write(OMAP_GPIO_I2C_CLIENT * client, OMAP_GPIO_I2C_WR_DATA *i2c_param)
{
	int i = 0;

	mutex_lock(&omap_gpio_i2c_mutex);
	// send start condition
	omap_gpio_i2c_send_start(client);

	// send slave address
	omap_gpio_i2c_send_byte(client, (client->addr << 1) | I2C_M_WR);

	// receive ack/nack from slave
	if( !omap_gpio_i2c_poll_ack(client) )
	{
		omap_gpio_i2c_send_stop(client);
		printk("[omap_gpio_i2c_do_write] ack timeout while sending SA+W\n");
		mutex_unlock(&omap_gpio_i2c_mutex);
		return -EIO;
	}

	// send register address
	for(i=0; i < i2c_param->reg_len; i++)
	{
		omap_gpio_i2c_send_byte( client, i2c_param->reg_addr[(i2c_param->reg_len-1)-i] );
		if( !omap_gpio_i2c_poll_ack(client) )
		{
			omap_gpio_i2c_send_stop(client);
			printk("[omap_gpio_i2c_do_write] ack timeout while sending RA\n");
			mutex_unlock(&omap_gpio_i2c_mutex);
			return -EIO;
		}
	}

	// send data
	for(i=0; i < i2c_param->wdata_len; i++)
	{
		omap_gpio_i2c_send_byte( client, i2c_param->wdata[i] );
		if( !omap_gpio_i2c_poll_ack(client) )
		{
			omap_gpio_i2c_send_stop(client);
			printk("[omap_gpio_i2c_do_write] ack timeout while writing DATA\n");
			mutex_unlock(&omap_gpio_i2c_mutex);
			return -EIO;
		}
	}

	// send stop condition
	omap_gpio_i2c_send_stop(client);

	mutex_unlock(&omap_gpio_i2c_mutex);

	return 0;
}

EXPORT_SYMBOL(omap_gpio_i2c_write);

//
// omap_gpio_i2c_read
//
int omap_gpio_i2c_read(OMAP_GPIO_I2C_CLIENT * client, OMAP_GPIO_I2C_RD_DATA *i2c_param)
{
	int i = 0;
	
	// check argument
	if(client == NULL)
	{
		printk(KERN_ERR "[%s] client is null!\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&omap_gpio_i2c_mutex);

	// send start condition
	omap_gpio_i2c_send_start(client);
	
	// if the client wants to send/receive in one command using SR
	if(i2c_param->reg_len > 0)
	{
		// send slave address
		omap_gpio_i2c_send_byte(client, (client->addr << 1) | I2C_M_WR);

		// check ack/nack
		if( !omap_gpio_i2c_poll_ack(client) )
		{
			omap_gpio_i2c_send_stop(client);
			printk("[omap_gpio_i2c_do_read] ack timeout while sending SA+W\n");
			mutex_unlock(&omap_gpio_i2c_mutex);
			return -EIO;
		}

		// send data
		for(i = 0; i < i2c_param->reg_len; i++)
		{
			omap_gpio_i2c_send_byte( client, i2c_param->reg_addr[(i2c_param->reg_len-1)-i] );
			if( !omap_gpio_i2c_poll_ack(client) )
			{
				omap_gpio_i2c_send_stop(client);
				printk("[omap_gpio_i2c_do_read] ack timeout while sending RA\n");
		mutex_unlock(&omap_gpio_i2c_mutex);
				return -EIO;
			}
		}

		omap_gpio_i2c_send_stop(client);
		udelay(client->delay);

		omap_gpio_i2c_send_start(client);
	}

	// send slave address
	omap_gpio_i2c_send_byte(client, (client->addr << 1) | I2C_M_RD);

	if( !omap_gpio_i2c_poll_ack(client) )
	{
		omap_gpio_i2c_send_stop(client);
		printk("[omap_gpio_i2c_do_read] ack timeout while sending SA+R\n");
		mutex_unlock(&omap_gpio_i2c_mutex);
		return -EIO;
	}

	// receive data and send ack or nack
	for(i = 0; i < i2c_param->rdata_len; i++)
	{
		i2c_param->rdata[i] = omap_gpio_i2c_read_byte(client);

		if(i == (i2c_param->rdata_len-1))
			omap_gpio_i2c_send_nack(client);
		else
			omap_gpio_i2c_send_ack(client);
	}

	// terminate communication
	omap_gpio_i2c_send_stop(client);

	mutex_unlock(&omap_gpio_i2c_mutex);

	return 0;
}

EXPORT_SYMBOL(omap_gpio_i2c_read);

