#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/wait.h>
#include <linux/stat.h>
#include <linux/ioctl.h>
#include <linux/gpio.h> 

static int sec_gain_open (struct inode *, struct file *);
static int sec_gain_release (struct inode *, struct file *);
static int sec_gain_test_mode_enalbe = 0;

int get_sec_gain_test_mode(void)
{
	return sec_gain_test_mode_enalbe;
}

EXPORT_SYMBOL_GPL(get_sec_gain_test_mode);

static struct file_operations sec_gain_fops =
{
    .owner = THIS_MODULE,
    .open = sec_gain_open,
    .release = sec_gain_release,
};

static struct miscdevice sec_gain_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "sec_gain",
    .fops = &sec_gain_fops,
};

static int sec_gain_open (struct inode *inode, struct file *filp)
{
	printk("sec_gain_open called\n");    

	sec_gain_test_mode_enalbe = 1;
	
	return 0;
}

static int sec_gain_release (struct inode *inode, struct file *filp)
{
	printk("sec_gain_release called\n");

	return 0;
}

 int __init sec_gain_driver_init(void)
{
    int ret = 0;		
    printk("sec_gain_driver_init called");  

    /*misc device registration*/
    if( (ret = misc_register(&sec_gain_misc_device)) < 0 )
    {
        printk("sec_gain_driver_init misc_register failed");
        return ret; 	  	
    }	
	   
    return ret;
}


void __exit sec_gain_driver_exit(void)
{
    printk("sec_gain_driver_exit called");  		  
    
    /*misc device deregistration*/
    misc_deregister(&sec_gain_misc_device);
}

module_init(sec_gain_driver_init);
module_exit(sec_gain_driver_exit);
MODULE_AUTHOR("changoh.heo <changoh.heo@samsung.com>");
MODULE_DESCRIPTION("sec_gain tuning driver");
MODULE_LICENSE("GPL");

