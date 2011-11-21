
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <plat/gpio.h>
#include <plat/mux.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
static struct early_suspend early_suspend;
#endif

#define DEVICE_NAME "gps_driver"

/* for debugging */
#define DEBUG 0
#define DEBUG_DELAY 0
#define DEBUG_THRESHOLD 0
#define TRACE_FUNC() pr_debug(BMA222_NAME ": <trace> %s()\n", __FUNCTION__)

extern struct class *sec_class;
static struct device *gps_en_dev;
static struct device *gps_nrst_dev;
static struct device *gps_cntl_dev;

static ssize_t gps_en_value_show(struct device *dev, 
                                 struct device_attribute *attr, 
                                 char *buf)
{
	int value = gpio_get_value(OMAP_GPIO_GPS_EN);
	return sprintf(buf, "%d\n", value);
}

static ssize_t gps_en_value_store(struct device *dev,
                                  struct device_attribute *attr, 
                                  const char *buf, 
                                  size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	gpio_set_value(OMAP_GPIO_GPS_EN, enable);

	return count;
}

static ssize_t gps_en_name_show(struct device *dev,
                                struct device_attribute *attr, 
                                char *buf)
{
    return sprintf(buf, "%s\n", "GPS_PWR_EN");
}


static ssize_t gps_nrst_value_show(struct device *dev, 
                                   struct device_attribute *attr, 
                                   char *buf)
{
	int value = gpio_get_value(OMAP_GPIO_GPS_NRST);
	return sprintf(buf, "%d\n", value);
}

static ssize_t gps_nrst_value_store(struct device *dev,
                                    struct device_attribute *attr, 
                                    const char *buf, 
                                    size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	gpio_set_value(OMAP_GPIO_GPS_NRST, enable);

	return count;
}

static ssize_t gps_nrst_name_show(struct device *dev,
                                struct device_attribute *attr, 
                                char *buf)
{
    return sprintf(buf, "%s\n", "GPS_NRST");
}


static ssize_t gps_cntl_value_show(struct device *dev, 
                                   struct device_attribute *attr, 
                                   char *buf)
{
	int value = gpio_get_value(OMAP_GPIO_GPS_CNTL);
	return sprintf(buf, "%d\n", value);
}

static ssize_t gps_cntl_value_store(struct device *dev,
                                    struct device_attribute *attr, 
                                    const char *buf, 
                                    size_t count)
{
	unsigned long enable = simple_strtoul(buf, NULL, 10);

	gpio_set_value(OMAP_GPIO_GPS_CNTL, enable);

	return count;
}

static ssize_t gps_cntl_name_show(struct device *dev,
                                struct device_attribute *attr, 
                                char *buf)
{
    return sprintf(buf, "%s\n", "GPS_CNTL");
}

static struct device_attribute gps_en_attr[2] = {
	{
		.attr.name = "value",
		.attr.mode = 0664,
		.show = gps_en_value_show,
		.store = gps_en_value_store,
	},
	{
		.attr.name = "name",
		.attr.mode = 0444,
		.show = gps_en_name_show,
		.store = NULL,
	}
};


static struct device_attribute gps_nrst_attr[2] = {
	{
		.attr.name = "value",
		.attr.mode = 0664,
		.show = gps_nrst_value_show,
		.store = gps_nrst_value_store,
	},
	{
		.attr.name = "name",
		.attr.mode = 0444,
		.show = gps_nrst_name_show,
		.store = NULL,
	}
};


static struct device_attribute gps_cntl_attr[2] = {
	{
		.attr.name = "value",
		.attr.mode = 0664,
		.show = gps_cntl_value_show,
		.store = gps_cntl_value_store,
	},
	{
		.attr.name = "name",
		.attr.mode = 0444,
		.show = gps_cntl_name_show,
		.store = NULL,
	}
};

#if defined(CONFIG_HAS_EARLYSUSPEND)
static int gps_early_suspend(struct early_suspend *handler)
{
	return 0;
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
static int gps_early_resume(struct early_suspend *handler)
{
	return 0;
}
#endif

static int gps_init_proc()
{
	// initialize GPIO
	if(gpio_is_valid(OMAP_GPIO_GPS_EN))
	{
		if(gpio_request(OMAP_GPIO_GPS_EN, NULL))
		{
			printk(KERN_ERR "Failed to request OMAP_GPIO_GPS_EN\n");
		}
		gpio_direction_output(OMAP_GPIO_GPS_EN, 0);
	}

	if(gpio_is_valid(OMAP_GPIO_GPS_NRST))
	{
		if(gpio_request(OMAP_GPIO_GPS_NRST, NULL))
		{
			printk(KERN_ERR "Failed to request OMAP_GPIO_GPS_NRST\n");
		}
		gpio_direction_output(OMAP_GPIO_GPS_NRST, 1);
	}

	if(gpio_is_valid(OMAP_GPIO_GPS_CNTL))
	{
		if(gpio_request(OMAP_GPIO_GPS_CNTL, NULL))
		{
			printk(KERN_ERR "Failed to request OMAP_GPIO_GPS_CNTL\n");
		}
		gpio_direction_output(OMAP_GPIO_GPS_CNTL, 0);
	}

	// register into sysfs
	gps_en_dev = device_create(sec_class, NULL, 0, NULL, "GPS_PWR_EN");
	gps_nrst_dev = device_create(sec_class, NULL, 0, NULL, "GPS_NRST");
	gps_cntl_dev = device_create(sec_class, NULL, 0, NULL, "GPS_CNTL");

	if(device_create_file(gps_en_dev, &gps_en_attr[0]) < 0)
	{
		printk(KERN_ERR "Failed to create gps_en_attr[0]\n");
	}

	if(device_create_file(gps_en_dev, &gps_en_attr[1]) < 0)
	{
		printk(KERN_ERR "Failed to create gps_en_attr[1]\n");
	}

	if(device_create_file(gps_nrst_dev, &gps_nrst_attr[0]) < 0)
	{
		printk(KERN_ERR "Failed to create gps_nrst_attr[0]\n");
	}

	if(device_create_file(gps_nrst_dev, &gps_nrst_attr[1]) < 0)
	{
		printk(KERN_ERR "Failed to create gps_nrst_attr[0]\n");
	}

	if(device_create_file(gps_cntl_dev, &gps_cntl_attr[0]) < 0)
	{
		printk(KERN_ERR "Failed to create gps_cntl_attr[0]\n");
	}

	if(device_create_file(gps_cntl_dev, &gps_cntl_attr[1]) < 0)
	{
		printk(KERN_ERR "Failed to create gps_cntl_attr[0]\n");
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	early_suspend.suspend = (void *)gps_early_suspend;
	early_suspend.resume  = (void *)gps_early_resume;
	register_early_suspend(&early_suspend);
#endif

    return 0;

}

static int gps_exit_proc()
{
#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&early_suspend);
#endif

	// free gpio	
	gpio_free(OMAP_GPIO_GPS_EN);
	gpio_free(OMAP_GPIO_GPS_NRST);
	gpio_free(OMAP_GPIO_GPS_CNTL);

	// free sysfs
	device_remove_file(gps_en_dev, &gps_en_attr[0]);
	device_remove_file(gps_en_dev, &gps_en_attr[1]);
	device_remove_file(gps_nrst_dev, &gps_nrst_attr[0]);
	device_remove_file(gps_nrst_dev, &gps_nrst_attr[1]);
	device_remove_file(gps_cntl_dev, &gps_cntl_attr[0]);
	device_remove_file(gps_cntl_dev, &gps_cntl_attr[1]);


	return 0;
}


/*
 * Module init and exit
 */
static int __init gps_init(void)
{
    return gps_init_proc();
}
module_init(gps_init);

static void __exit gps_exit(void)
{
    gps_exit_proc();
}

module_exit(gps_exit);

MODULE_DESCRIPTION("GPS driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
