/* This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <plat/gpio.h>
#include <plat/hardware.h>
#include <plat/mux.h>

#include <linux/i2c/twl.h>

#if defined(CONFIG_INPUT_GPIO_VOLUME_KEY) && defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
#define POWER_KEY_FLAG (1<<0)
#define VOLDN_KEY_FLAG (1<<1)
#define VOLUP_KEY_FLAG (1<<2)

int key_flag = 0;
inline void check_force_crash(int key, int press)
{
	if(press) {
		key_flag |= key;
	}
	else {
		key_flag &= ~key;
	}
	if(key_flag == (POWER_KEY_FLAG | VOLDN_KEY_FLAG  | VOLUP_KEY_FLAG)) {
		printk(KERN_ERR "key_flag = %d\n", key_flag);
		printk(KERN_ERR "%s : Force Crash by keypad\n", __func__);
		panic("__forced_upload");
	}
}
#endif

static int __devinit power_key_driver_probe(struct platform_device *plat_dev);
static irqreturn_t powerkey_press_handler(int irq_num, void * dev);
#ifdef CONFIG_INPUT_HARD_RESET_KEY
static irqreturn_t homekey_press_handler(int irq_num, void * dev);
int home_key_press_status = 0;
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
static irqreturn_t volume_down_key_press_handler(int irq_num, void * dev);
static irqreturn_t volume_up_key_press_handler(int irq_num, void * dev);
#endif

struct work_struct  lcd_work;
static struct workqueue_struct *lcd_wq=NULL;
void lcd_work_func(struct work_struct *work)
{
//	printk("[LCD] lcd_work_func\n");
//    twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00, 0x1F);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x1, 0x22);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0xe0, 0x1F);

//	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00, 0x23);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x9, 0x26);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0xe0, 0x23);
}

ssize_t gpiokey_pressed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  unsigned int key_press_status=0;
  key_press_status = gpio_get_value(OMAP_GPIO_KEY_PWRON);
#ifdef CONFIG_INPUT_HARD_RESET_KEY
  key_press_status &= ~0x2;
  key_press_status |= ((!gpio_get_value(OMAP_GPIO_KEY_HOME)) << 1);
#endif

#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  key_press_status &= ~0xC;
  key_press_status |= ((!gpio_get_value(OMAP_GPIO_VOLUME_UP)) << 2);
  key_press_status |= ((!gpio_get_value(OMAP_GPIO_VOLUME_DOWN)) << 3);
#endif

  return sprintf(buf, "%u\n", key_press_status);
}

static DEVICE_ATTR(gpiokey_pressed, S_IRUGO, gpiokey_pressed_show, NULL);
  
static irqreturn_t powerkey_press_handler(int irq_num, void * dev)
{
  struct input_dev *ip_dev = (struct input_dev *)dev;
  int key_press_status=0; 

  if(!ip_dev){
    dev_err(ip_dev->dev.parent,"Input Device not allocated\n");
    return IRQ_HANDLED;
  }
  
  key_press_status = gpio_get_value(OMAP_GPIO_KEY_PWRON);

  if( key_press_status < 0 ){
    dev_err(ip_dev->dev.parent,"Failed to read GPIO value\n");
    return IRQ_HANDLED;
  }
  
  input_report_key(ip_dev,KEY_POWER,key_press_status);
  input_sync(ip_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
  dev_dbg(ip_dev->dev.parent,"Sent KEY_POWER event = %d\n",key_press_status);
  printk("[PWR-KEY] KEY_POWER event = %d\n",key_press_status);
#endif
#if defined(CONFIG_INPUT_GPIO_VOLUME_KEY) && defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
  check_force_crash(POWER_KEY_FLAG, key_press_status);
#endif

  if (lcd_wq && key_press_status)  
  	queue_work(lcd_wq, &lcd_work);
  
  return IRQ_HANDLED;
}

#ifdef CONFIG_INPUT_HARD_RESET_KEY
static irqreturn_t homekey_press_handler(int irq_num, void * dev)
{
  struct input_dev *ip_dev = (struct input_dev *)dev;
  
  if(!ip_dev){
    dev_err(ip_dev->dev.parent,"Input Device not allocated\n");
    return IRQ_HANDLED;
  }
  
  home_key_press_status = !gpio_get_value(OMAP_GPIO_KEY_HOME);
  
  if( home_key_press_status < 0 ){
    dev_err(ip_dev->dev.parent,"Failed to read GPIO value\n");
    return IRQ_HANDLED;
  }
  input_report_key(ip_dev,KEY_HOME,home_key_press_status);
  input_sync(ip_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
  dev_dbg(ip_dev->dev.parent,"Sent KEY_HOME event = %d\n",home_key_press_status);
  printk("Sent KEY_HOME event = %d\n",home_key_press_status);
#endif

  if (lcd_wq && home_key_press_status)
	queue_work(lcd_wq, &lcd_work);
  	
  return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
static irqreturn_t volume_down_key_press_handler(int irq_num, void * dev)
{
  struct input_dev *ip_dev = (struct input_dev *)dev;
  int key_press_status = 0;
  
  if(!ip_dev){
    dev_err(ip_dev->dev.parent,"Input Device not allocated\n");
    return IRQ_HANDLED;
  }
  
  key_press_status = !gpio_get_value(OMAP_GPIO_VOLUME_DOWN);
  
  if( key_press_status < 0 ){
    dev_err(ip_dev->dev.parent,"Failed to read GPIO value\n");
    return IRQ_HANDLED;
  }

  input_report_key(ip_dev, KEY_VOLUMEDOWN, key_press_status);
  input_sync(ip_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
  dev_dbg(ip_dev->dev.parent,"Sent KEY_VOLUMEDOWN event = %d\n", key_press_status);
  printk("Sent KEY_VOLUMEDOWN event = %d\n", key_press_status);
  check_force_crash(VOLDN_KEY_FLAG, key_press_status);
#endif
  return IRQ_HANDLED;
}

static irqreturn_t volume_up_key_press_handler(int irq_num, void * dev)
{
  struct input_dev *ip_dev = (struct input_dev *)dev;
  int key_press_status = 0;
  
  if(!ip_dev){
    dev_err(ip_dev->dev.parent,"Input Device not allocated\n");
    return IRQ_HANDLED;
  }
  
  key_press_status = !gpio_get_value(OMAP_GPIO_VOLUME_UP);
  
  if( key_press_status < 0 ){
    dev_err(ip_dev->dev.parent,"Failed to read GPIO value\n");
    return IRQ_HANDLED;
  }

  input_report_key(ip_dev, KEY_VOLUMEUP, key_press_status);
  input_sync(ip_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
  dev_dbg(ip_dev->dev.parent,"Sent KEY_VOLUMEUP event = %d\n", key_press_status);
  printk("Sent KEY_VOLUMEUP event = %d\n", key_press_status);
  check_force_crash(VOLUP_KEY_FLAG, key_press_status);
#endif
  return IRQ_HANDLED;
}
#endif

static int __devinit power_key_driver_probe(struct platform_device *plat_dev)
{
  struct input_dev *power_key=NULL;
  int pwr_key_irq=-1, err=0;

  struct kobject *gpiokey;
#ifdef CONFIG_INPUT_HARD_RESET_KEY
  int home_key_irq = -1;
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  int volume_up_key_irq = -1;
  int volume_down_key_irq = -1;
#endif

  gpiokey = kobject_create_and_add("gpiokey", NULL);
  if (!gpiokey) {
    printk("Failed to create sysfs(gpiokey)!\n");
    return -ENOMEM;
  }
  if (sysfs_create_file(gpiokey, &dev_attr_gpiokey_pressed.attr)< 0)
  printk("Failed to create device file(%s)!\n", dev_attr_gpiokey_pressed.attr.name);
   
  INIT_WORK(&lcd_work, lcd_work_func);
  lcd_wq = create_singlethread_workqueue("lcd_vaux_wq");

  pwr_key_irq = platform_get_irq(plat_dev, 0);
  if(pwr_key_irq <= 0 ){
    dev_err(&plat_dev->dev,"failed to map the power key to an IRQ %d\n", pwr_key_irq);
    err = -ENXIO;
    return err;
  }  
#ifdef CONFIG_INPUT_HARD_RESET_KEY
  home_key_irq = platform_get_irq(plat_dev, 1);
  if(home_key_irq <= 0){
    dev_err(&plat_dev->dev,"failed to map the power key to an IRQ %d\n", home_key_irq);
    err = -ENXIO;
    return err;
  }
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  volume_down_key_irq = platform_get_irq(plat_dev, 2);
  if(volume_down_key_irq <= 0) {
    dev_err(&plat_dev->dev, "failed to map the volume down key to an IRQ %d\n", volume_down_key_irq);
  }

  volume_up_key_irq = platform_get_irq(plat_dev, 3);
  if(volume_up_key_irq <= 0) {
    dev_err(&plat_dev->dev, "failed to map the volume up key to an IRQ %d\n", volume_up_key_irq);
  }
#endif
  power_key = input_allocate_device();
  if(!power_key)
  {
    dev_err(&plat_dev->dev,"failed to allocate an input devd %d \n",pwr_key_irq);
    err = -ENOMEM;
    return err;
  }
  err = request_irq(pwr_key_irq, &powerkey_press_handler ,IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
                    "power_key_driver",power_key);
#ifdef CONFIG_INPUT_HARD_RESET_KEY
  err |= request_irq(home_key_irq, &homekey_press_handler ,IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
                    "power_key_driver",power_key);
#endif
  if(err) {
#ifdef CONFIG_INPUT_HARD_RESET_KEY
    dev_err(&plat_dev->dev,"failed to request an IRQ handler for num %d & %d\n",pwr_key_irq, home_key_irq);
#else
    dev_err(&plat_dev->dev,"failed to request an IRQ handler for num %d\n",pwr_key_irq);
#endif
    goto free_input_dev;
  }

  dev_dbg(&plat_dev->dev,"\n Power Key Drive:Assigned IRQ num %d SUCCESS \n",pwr_key_irq);
#ifdef CONFIG_INPUT_HARD_RESET_KEY
  dev_dbg(&plat_dev->dev,"\n HOME Key Drive:Assigned IRQ num %d SUCCESS \n",home_key_irq);
#endif

#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  err = request_irq(volume_down_key_irq, &volume_down_key_press_handler, 
                    IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "power_key_driver", power_key);
  
  err |= request_irq(volume_up_key_irq, &volume_up_key_press_handler, 
                     IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "power_key_driver", power_key);

  if(err) {
    dev_err(&plat_dev->dev, "failed to request an IRQ handler for num %d & %d\n", 
            volume_down_key_irq, volume_up_key_irq);
  }

  dev_dbg(&plat_dev->dev, "\n Volume Down Key Driver:Assigned IRQ num %d SUCCESS\n", volume_down_key_irq);
  dev_dbg(&plat_dev->dev, "\n Volume Up Key Driver:Assigned IRQ num %d SUCCESS\n", volume_up_key_irq);
#endif

  /* register the input device now */
  input_set_capability(power_key, EV_KEY, KEY_POWER);
#ifdef CONFIG_INPUT_HARD_RESET_KEY
  input_set_capability(power_key, EV_KEY, KEY_HOME);
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  input_set_capability(power_key, EV_KEY, KEY_VOLUMEDOWN);
  input_set_capability(power_key, EV_KEY, KEY_VOLUMEUP);
#endif
  power_key->name = "sec_power_key";
  power_key->phys = "sec_power_key/input0";
  power_key->dev.parent = &plat_dev->dev;
  platform_set_drvdata(plat_dev, power_key);

  err = input_register_device(power_key);
  if (err) {
    dev_err(&plat_dev->dev, "power key couldn't be registered: %d\n", err);
    goto release_irq_num;
  }
 
  return 0;

release_irq_num:
  free_irq(pwr_key_irq,NULL); //pass devID as NULL as device registration failed 
#ifdef CONFIG_INPUT_HARD_RESET_KEY
  free_irq(home_key_irq, NULL);
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  free_irq(volume_down_key_irq, NULL);
  free_irq(volume_up_key_irq, NULL);
#endif

free_input_dev:
  input_free_device(power_key);

return err;

}

static int __devexit power_key_driver_remove(struct platform_device *plat_dev)
{
  struct input_dev *ip_dev= platform_get_drvdata(plat_dev);
  int pwr_key_irq=0;
#ifdef CONFIG_INPUT_HARD_RESET_KEY
  int home_key_irq=0;
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  int volume_down_key_irq = 0;
  int volume_up_key_irq = 0;
#endif

  pwr_key_irq = platform_get_irq(plat_dev,0);
#ifdef CONFIG_INPUT_HARD_RESET_KEY
  home_key_irq = platform_get_irq(plat_dev,1);
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  volume_down_key_irq = platform_get_irq(plat_dev, 2);
  volume_up_key_irq = platform_get_irq(plat_dev, 3);
#endif

  free_irq(pwr_key_irq,ip_dev);
#ifdef CONFIG_INPUT_HARD_RESET_KEY
  free_irq(home_key_irq,ip_dev);
#endif
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
  free_irq(volume_down_key_irq, ip_dev);
  free_irq(volume_up_key_irq, ip_dev);
#endif
  input_unregister_device(ip_dev);

  if (lcd_wq)
		destroy_workqueue(lcd_wq);

  return 0;
}
struct platform_driver power_key_driver_t = {
        .probe          = &power_key_driver_probe,
        .remove         = __devexit_p(power_key_driver_remove),
        .driver         = {
                .name   = "power_key_device", 
                .owner  = THIS_MODULE,
        },
};

static int __init power_key_driver_init(void)
{
        return platform_driver_register(&power_key_driver_t);
}
module_init(power_key_driver_init);

static void __exit power_key_driver_exit(void)
{
        platform_driver_unregister(&power_key_driver_t);
}
module_exit(power_key_driver_exit);

MODULE_ALIAS("platform:power key driver");
MODULE_DESCRIPTION("board zeus power key");
MODULE_LICENSE("GPL");






















