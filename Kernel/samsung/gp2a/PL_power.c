
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include <plat/gpio.h>
#include <plat/mux.h>
#include "common.h"

#define DRIVER_NAME "secPLSensorPower"


static atomic_t reference_count;

static struct regulator *vaux1;
static struct regulator *vaux2;
static struct regulator *vusb31;
static struct regulator *vusb18;
static struct regulator *vusb15;

int pl_sensor_power_on( void );
int pl_sensor_power_off( void );
int pl_usb_power_on( void );
int pl_usb_power_off( void );
int pl_madc_power_on( void );
int pl_madc_power_off( void );

static int __devinit pl_power_probe( struct platform_device *pdev );
static int __devexit pl_power_remove( struct platform_device *pdev );

int pl_power_init( void );
void pl_power_exit( void );

struct platform_driver pl_power_platform_driver = {
	.probe		= pl_power_probe,
	.remove		= __devexit_p( pl_power_remove ),
	.driver		= {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

int pl_sensor_power_on( void )
{
	int ret;

	trace_in();

#if defined(CONFIG_MACH_SAMSUNG_LATONA) && CONFIG_SAMSUNG_REL_HW_REV >= 1
	gpio_set_value(OMAP_GPIO_ALS_EN, 1);
#endif

	if( !vaux1 || !vaux2 )
	{
		printk("PSENSOR: %s: vaux1 or vaux2 is NULL!\n", __func__);
		return -1;
	}
	
	ret = regulator_enable( vaux1 );
	if( ret )
	{
		printk("Regulator vaux1 error!!\n");
		trace_out();
		return ret;
	}

	ret = regulator_enable( vaux2 );
	
	if( ret )
		printk("Regulator vaux2 error!!\n");

	atomic_inc(&reference_count);

	printk("[PSENSOR]: %s: REF COUNT:%d\n", __func__, atomic_read(&reference_count));

	trace_out();
	return ret;
}

int pl_sensor_power_off( void )
{
	int ret = 0;
	trace_in();

	if( !vaux1 || !vaux2 )
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
#if defined(CONFIG_MACH_SAMSUNG_LATONA) && CONFIG_SAMSUNG_REL_HW_REV >= 1
		gpio_set_value(OMAP_GPIO_ALS_EN, 0);
#endif
		ret = regulator_disable( vaux1 );
		return ret ? ret : regulator_disable( vaux2 );
	}
	
	return 0;
}
int pl_usb_power_on()
{
	int ret;

	trace_in();

	if( !vusb31 || !vusb18 || !vusb15 )
	{
		printk("PSENSOR: %s: vusb31[%x] or vusb18[%x] or vusb15[%x] is NULL!\n", __func__, vusb31, vusb18, vusb15);
		return -1;
	}
	
	ret = regulator_enable( vusb31 );
	if( ret )
		printk("Regulator vusb31 error!!\n");

	ret = regulator_enable( vusb18 );
	if( ret )
		printk("Regulator vusb18 error!!\n");

	ret = regulator_enable( vusb15 );
	if( ret )
		printk("Regulator vusb15 error!!\n");

	trace_out();
	return ret;
}

int pl_usb_power_off()
{
	int ret;

	trace_in();

	if( !vusb18 || !vusb15 )
	{
		printk("PSENSOR: %s: vusb18[%x] or vusb15[%x] is NULL!\n", __func__, vusb18, vusb15);
		return -1;
	}
	
	ret = regulator_disable( vusb18 );
	if( ret )
		printk("Regulator vusb18 error!!\n");

	ret = regulator_disable( vusb15 );
	if( ret )
		printk("Regulator vusb15 error!!\n");

	trace_out();
	return ret;
}

int pl_madc_power_on()
{
	int ret;

	trace_in();

	if( !vusb31 )
	{
		printk("PSENSOR: %s: vusb31[%x] is NULL!\n", __func__, vusb31);
		return -1;
	}
	
	ret = regulator_enable( vusb31 );
	if( ret )
		printk("Regulator vusb31 error!!\n");

    trace_out();
	return ret;
}

int pl_madc_power_off()
{
	int ret;

	trace_in();

	if( !vusb31 )
	{
		printk("PSENSOR: %s: vusb31[%x] is NULL!\n", __func__, vusb31);
		return -1;
	}
	
	ret = regulator_disable( vusb31 );
	if( ret )
		printk("Regulator vusb31 error!!\n");

	trace_out();
	return ret;
}

static int __devinit pl_power_probe( struct platform_device *pdev )
{ 
	trace_in();
	
	//printk("PSENSOR: %s: \n", __func__);

	vaux1 = regulator_get( &pdev->dev, "vaux1" );
	if( IS_ERR( vaux1 ) )
	{
		printk("PSENSOR: %s: regulator_get failed to get vaux1\n", __func__);
		trace_out();
		return -1;
	}

	vaux2 = regulator_get( &pdev->dev, "vaux2" );
	if( IS_ERR( vaux2 ) )
	{
		printk("PSENSOR: %s: regulator_get failed to get vaux2!\n", __func__);
		return -1;
	}

	vusb31 = regulator_get( &pdev->dev, "usb3v1" );
	if( IS_ERR( vusb31 ) )
	{
		printk("PSENSOR: %s: regulator_get failed to get vusb31!\n", __func__);
		return -1;
	}

	vusb18 = regulator_get( &pdev->dev, "usb1v8" );
	if( IS_ERR( vusb18 ) )
	{
		printk("PSENSOR: %s: regulator_get failed to get vusb18!\n", __func__);
		return -1;
	}

	vusb15 = regulator_get( &pdev->dev, "usb1v5" );
	if( IS_ERR( vusb15 ) )
	{
		printk("PSENSOR: %s: regulator_get failed to get vusb15!\n", __func__);
		return -1;
	}
	trace_out();
	return 0;
}

static int __devexit pl_power_remove( struct platform_device *pdev )
{
	trace_in();

	if(vaux1) regulator_put(vaux1);
	if(vaux2) regulator_put(vaux2);    
    if(vusb31) regulator_put(vusb31);
    if(vusb18) regulator_put(vusb18);
    if(vusb15) regulator_put(vusb15);
    
	trace_out();
	return 0;
}

int pl_power_init( void )
{
	int ret;

	trace_in();

	atomic_set(&reference_count, 0);

	ret = platform_driver_register( &pl_power_platform_driver );
	trace_out();
	return ret;
}

void pl_power_exit( void )
{
	trace_in();
	platform_driver_unregister( &pl_power_platform_driver );
	trace_out();
}

