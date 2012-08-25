
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include <plat/gpio.h>
#include <plat/mux.h>

#define DRIVER_NAME "secPLSensorPower"

static atomic_t reference_count;

static struct regulator *vaux2;

int bh1721_sensor_power_on( void );
int bh1721_sensor_power_off( void );

static int __devinit bh1721_power_probe( struct platform_device *pdev );
static int __devexit bh1721_power_remove( struct platform_device *pdev );

int  bh1721_power_init( void );
void bh1721_power_exit( void );

struct platform_driver bh1721_power_platform_driver = {
	.probe		= bh1721_power_probe,
	.remove		= __devexit_p( bh1721_power_remove ),
	.driver		= {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

int bh1721_sensor_power_on( void )
{
	int ret;

	if( !vaux2 )
	{
		printk("PSENSOR: %s: vaux2 is NULL!\n", __func__);
		return -1;
	}
	
	ret = regulator_enable( vaux2 );
	
	if( ret )
		printk("Regulator vaux2 error!!\n");

	atomic_inc(&reference_count);

	printk("[PSENSOR]: %s: REF COUNT:%d\n", __func__, atomic_read(&reference_count));

	return ret;
}

int bh1721_sensor_power_off( void )
{
	if( !vaux2 )
		return 0;
	
	if(atomic_read(&reference_count) >= 1)
	{
		atomic_dec(&reference_count);
	}
	else
	{
		return 0;
	}
	
	printk("[PSENSOR]: %s: REF COUNT:%d\n", __func__, atomic_read(&reference_count));

	if(atomic_read(&reference_count) == 0)
	{	
		return regulator_disable( vaux2 );
	}
	
	return 0;
}

static int __devinit bh1721_power_probe( struct platform_device *pdev )
{ 
	vaux2 = regulator_get( &pdev->dev, "vaux2" );
	if( IS_ERR( vaux2 ) )
	{
		printk("PSENSOR: %s: regulator_get failed to get vaux2!\n", __func__);
		return -1;
	}
	return 0;
}

static int __devexit bh1721_power_remove( struct platform_device *pdev )
{
	if( !vaux2 )
		return 0;

	regulator_put( vaux2 );

	return 0;
}

int bh1721_power_init( void )
{
	int ret;

	atomic_set(&reference_count, 0);

	ret = platform_driver_register( &bh1721_power_platform_driver );

	return ret;
}

void bh1721_power_exit( void )
{
	platform_driver_unregister( &bh1721_power_platform_driver );
}

