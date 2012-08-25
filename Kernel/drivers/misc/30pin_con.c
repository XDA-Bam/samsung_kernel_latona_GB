

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/device.h>


//#include <plat/gpio-cfg.h>
#include <asm/irq.h>
//#include <mach/regs-gpio.h>
//#include <mach/regs-clock.h>
//#include <mach/fsa9480_i2c.h>

//#include <mach/max8998_function.h>

#if defined(CONFIG_KEYBOARD_P1)
#include <linux/wakelock.h>
static struct wake_lock wake_lock_for_kbd;
#endif


#define SUBJECT "CONNECTOR_DRIVER"

#define ACC_CONDEV_DBG(format,...)\
	printk ("[ "SUBJECT " (%s,%d) ] " format "\n", __func__, __LINE__, ## __VA_ARGS__);

#undef SUPPORT_MHD_DEVICE 

#define ACCESSORY_ID 4
#define DETECTION_INTR_DELAY	 	get_jiffies_64() + (HZ*15)// 15s
#define IRQ_ACCESSORY_INT	OMAP_GPIO_IRQ(OMAP_GPIO_ACCESSORY_INT)
#define IRQ_DOCK_INT	OMAP_GPIO_IRQ(OMAP_GPIO_DOCK_INT)
#define IRQ_JIG_ON_INT	OMAP_GPIO_IRQ(OMAP_GPIO_JIG_ON18)

#ifdef SUPPORT_MHD_DEVICE 
#define IRQ_MHL_INT		IRQ_EINT10
#endif


#define ACC_TVOUT		1
#define ACC_LINEOUT		2
#define ACC_CARMOUNT	3 
#define ACC_DOWNLOADMODE 4
#define ACC_CAMERAKIT    5

#define DOCK_DESK		1
#define DOCK_KEYBD		2

int DOCK_STATE= 3;
int ACC_STATE=1;
int CONNECTED_ACC=0;
int CONNECTED_DOCK=0;

// chul2.park
#ifdef CONFIG_USB_S3C_OTG_HOST
// workaround for wrong interrupt at boot time :(
static unsigned int intr_count = 0;
#endif

#if ( CONFIG_SAMSUNG_REL_HW_REV > 2 )
	extern int get_adc_conversion(int channel);
#endif

extern void set_uart_suspend(int value);
#if defined(CONFIG_KEYBOARD_P1)
extern int check_keyboard_dock(void);
#endif

extern unsigned int HWREV;

#ifdef CONFIG_MHL_SII9234

extern void sii9234_tpi_init(void);
extern void MHD_HW_Reset(void);
extern void MHD_HW_Off(void);
extern int MHD_HW_IsOn(void);
extern int MHD_Read_deviceID(void);
extern byte MHD_Bridge_detect(void);
extern void MHD_INT_clear(void);
extern void MHD_OUT_EN(void);

//extern byte MHD_Bridge_detect();


 
//I2C driver add 20100614  kyungrok
extern struct i2c_driver SII9234_i2c_driver;
extern struct i2c_driver SII9234A_i2c_driver;
extern struct i2c_driver SII9234B_i2c_driver;
extern struct i2c_driver SII9234C_i2c_driver;

//extern void max8998_ldo3_8_control(int enable, unsigned int flag);
//extern void TVout_LDO_ctrl(int enable);


#endif 

static struct device *acc_dev;

static struct platform_driver acc_con_driver;
static struct timer_list connector_detect_timer;
static struct workqueue_struct *acc_con_workqueue;
static struct work_struct acc_con_work;

static struct workqueue_struct *jig_on_workqueue;
static struct work_struct jig_on_work;

static struct workqueue_struct *acc_ID_workqueue;
static struct work_struct acc_ID_work;

static struct workqueue_struct *acc_MHD_workqueue;
static struct work_struct acc_MHD_work;



void acc_con_interrupt_init(void);
static void acc_con_detect_timer_handler(unsigned long arg);
static int connector_detect_change(void);


#ifdef SUPPORT_MHD_DEVICE 
static ssize_t MHD_check_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res;
	TVout_LDO_ctrl(true);
	if(!MHD_HW_IsOn())
	{
		sii9234_tpi_init();
		res = MHD_Read_deviceID();
		MHD_HW_Off();		
	}
	else
	{
		sii9234_tpi_init();
		res = MHD_Read_deviceID();
	}
	
	count = sprintf(buf,"%d\n", res );
	TVout_LDO_ctrl(false);
	return count;
}

static ssize_t MHD_check_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("input data --> %s\n", buf);

	return size;
}

static DEVICE_ATTR(MHD_file, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, MHD_check_read, MHD_check_write);
#endif

static ssize_t acc_check_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int connected = 0;	
	if(0 == DOCK_STATE)
	{
		if(CONNECTED_DOCK == DOCK_DESK)
			connected |= (0x1<<0);
		else if(CONNECTED_DOCK == DOCK_KEYBD)
			connected |= (0x1<<1);
	}
	if(0 == ACC_STATE)
	{
		if(CONNECTED_ACC == ACC_CARMOUNT)
			connected |= (0x1<<2);
		else if(CONNECTED_ACC == ACC_TVOUT)
			connected |= (0x1<<3);
		else if(CONNECTED_ACC == ACC_LINEOUT)
			connected |= (0x1<<4);
	}
#if defined (CONFIG_TARGET_LOCALE_KOR)
//HW rev09 hpd is pull up by vcc1.8PDA   
	if(HWREV > 11 ) 
	{
		if(gpio_get_value(GPIO_HDMI_HPD))
			connected |= (0x1<<5);
	}
#else
//#if defined (CONFIG_TARGET_LOCALE_EUR)
//	if(HWREV > 0xC ) //rev0.7 is 0xD
//	{
#ifdef CONFIG_MHL_SII9234
		if(gpio_get_value(GPIO_HDMI_HPD)&& MHD_HW_IsOn())
			connected |= (0x1<<5);
#endif
//	}
//#endif	
#endif
	//connected = ((DOCK_STATE<<0)|(CONNECTED_DOCK<<4)|(ACC_STATE<<8)|(CONNECTED_ACC<<12));
	count = sprintf(buf,"%d\n", connected );
	ACC_CONDEV_DBG("%x",connected);
	return count;
}

static ssize_t acc_check_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("input data --> %s\n", buf);

	return size;
}

static DEVICE_ATTR(acc_file, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, acc_check_read, acc_check_write);


void acc_TA_check(int On)
{
	if(!gpio_get_value(OMAP_GPIO_ACCESSORY_INT))
	{
		if(On == 0)
			{
#ifdef CONFIG_MHL_SII9234
			MHD_HW_Off();
#endif
			}
		else if((On == 1) && (CONNECTED_DOCK == DOCK_DESK) )
			{
#ifdef CONFIG_MHL_SII9234
			sii9234_tpi_init();
#endif
			}
	}

}
EXPORT_SYMBOL(acc_TA_check);
static int connector_detect_change(void)
{
	int adc = 0,i,adc_sum=0;
	int adc_buff[5]={0};
	int adc_min=0;
	int adc_max=0;
	for(i = 0; i < 5; i++)
	{
#if ( CONFIG_SAMSUNG_REL_HW_REV > 2 )
		adc_buff[i] = get_adc_conversion(ACCESSORY_ID);
#else
		adc_buff[i] = 0;
#endif		
		adc_sum +=adc_buff[i];
		if(i == 0)
		{
			adc_min = adc_buff[0];
			adc_max = adc_buff[0];	
		}
		else
		{
			if(adc_max < adc_buff[i])
				adc_max = adc_buff[i];
			else if(adc_min > adc_buff[i])
				adc_min = adc_buff[i];
		}
		msleep(20);
	}
	adc = (adc_sum - adc_max - adc_min)/3;
	ACC_CONDEV_DBG("ACCESSORY_ID : ADC value = %d\n", adc);
	return adc;
}


static void acc_con_detect_timer_handler(unsigned long arg)
{
	ACC_CONDEV_DBG("");
	// [ dock detection in boot sequence
	acc_con_interrupt_init();
	if(!gpio_get_value(OMAP_GPIO_ACCESSORY_INT))
	{
		disable_irq(IRQ_ACCESSORY_INT);
		queue_work(acc_con_workqueue, &acc_con_work);
	}
	// ]
}


void acc_dock_check(int dock, int state)	
{
	char env_buf[60];
	char stat_buf[60];
    char *envp[3];
    int env_offset = 0;
	memset(env_buf, 0, sizeof(env_buf));
	memset(stat_buf, 0, sizeof(stat_buf));	

	//DOCK check
	if(dock == DOCK_KEYBD)
	{
		sprintf(env_buf, "DOCK=keyboard");
	}
	else if(dock == DOCK_DESK)
	{
		sprintf(env_buf, "DOCK=desk");
	}
	else 
	{
		sprintf(env_buf, "DOCK=unknown");
	}

	//state check
	if(state == 1)
	{
		sprintf(stat_buf, "STATE=offline");
	}
	else if(state == 0)
	{
		sprintf(stat_buf, "STATE=online");
	}
	else
	{
		sprintf(stat_buf, "STATE=unknown");
	}
	envp[env_offset++] = env_buf;
	envp[env_offset++] = stat_buf;
	envp[env_offset] = NULL;
	kobject_uevent_env(&acc_dev->kobj, KOBJ_CHANGE, envp);
	ACC_CONDEV_DBG("%s : %s",env_buf,stat_buf);
}

void acc_con_intr_handle(struct work_struct *_work)
{
	//ACC_CONDEV_DBG("");
	//check the flag MHL or keyboard
	int cur_state = gpio_get_value(OMAP_GPIO_ACCESSORY_INT);
#if defined(CONFIG_KEYBOARD_P1)
    static bool kbd_connected = false;
    if(kbd_connected)
    {
    wake_lock_timeout(&wake_lock_for_kbd, HZ);
        kbd_connected = false;
    }
#endif

	if(cur_state !=DOCK_STATE)
	{
		if(1==cur_state)
		{
			ACC_CONDEV_DBG("docking station detatched!!!");
			DOCK_STATE = cur_state;
#if defined(CONFIG_KEYBOARD_P1)
			check_keyboard_dock();
#endif			
#ifdef CONFIG_MHL_SII9234		
			//call MHL deinit
			MHD_HW_Off();
			//msleep(120);
			//max8998_ldo3_8_control(0,LDO_TV_OUT);  //ldo 3,8 off
			//printk("%s: LDO3_8 is disabled by TV \n", __func__);
			TVout_LDO_ctrl(false);
			
#endif	
			acc_dock_check(CONNECTED_DOCK , DOCK_STATE);
			CONNECTED_DOCK = 0;
		}
		else if(0==cur_state)
		{
			ACC_CONDEV_DBG("docking station attatched!!!");
			DOCK_STATE = cur_state;
#if defined(CONFIG_KEYBOARD_P1)
        	if(check_keyboard_dock())
        	{
				CONNECTED_DOCK = DOCK_KEYBD;
                        kbd_connected = true;
			}
			else
#endif				
        	{
				CONNECTED_DOCK = DOCK_DESK;
#ifdef CONFIG_MHL_SII9234
				//max8998_ldo3_8_control(1,LDO_TV_OUT); //ldo 3,8 on
				//printk("%s: LDO3_8 is enabled by TV \n", __func__);
				//msleep(120);
				TVout_LDO_ctrl(true);
				//call MHL init	
				sii9234_tpi_init();
#endif
         	}
			acc_dock_check(CONNECTED_DOCK , DOCK_STATE);
		}
	}
	else
	{
		ACC_CONDEV_DBG("Ignored");
	}
	enable_irq(IRQ_ACCESSORY_INT);
}

irqreturn_t jig_on_interrupt(int irq, void *ptr)
{
	ACC_CONDEV_DBG("");
	disable_irq_nosync(IRQ_JIG_ON_INT);

	queue_work(jig_on_workqueue, &jig_on_work);

	return IRQ_HANDLED; 
}


irqreturn_t acc_con_interrupt(int irq, void *ptr)
{
	ACC_CONDEV_DBG("");
	disable_irq_nosync(IRQ_ACCESSORY_INT);

	queue_work(acc_con_workqueue, &acc_con_work);

	return IRQ_HANDLED; 
}

void acc_con_interrupt_init(void)
{
	int ret;
	//s3c_gpio_cfgpin(GPIO_ACCESSORY_INT, S3C_GPIO_SFN(GPIO_ACCESSORY_INT_AF));
	//s3c_gpio_setpull(GPIO_ACCESSORY_INT, S3C_GPIO_PULL_UP);

// TEMP BLOCK	omap_set_gpio_debounce(OMAP_GPIO_ACCESSORY_INT, 1);
// TEMP BLOCK	omap_set_gpio_debounce_time(OMAP_GPIO_ACCESSORY_INT, 34);

	set_irq_type(IRQ_ACCESSORY_INT, IRQ_TYPE_EDGE_BOTH);
	
	ret = request_irq(IRQ_ACCESSORY_INT, acc_con_interrupt, IRQF_DISABLED, "Docking Detected", NULL);
	if (ret)
	 	ACC_CONDEV_DBG("Fail to register IRQ : GPIO_ACCESSORY_INT return : %d\n",ret);
}

irqreturn_t jig_on_int_handler(int irq, void *ptr)
{
	//ACC_CONDEV_DBG("");
	disable_irq_nosync(IRQ_JIG_ON_INT);
	queue_work(jig_on_workqueue, &jig_on_work);

	return IRQ_HANDLED; 
}

void jig_on_interrupt_init(void)
{
	int ret;
	set_irq_type(IRQ_JIG_ON_INT, IRQ_TYPE_EDGE_BOTH);
	
	ret = request_irq(IRQ_JIG_ON_INT, jig_on_int_handler, IRQF_DISABLED, "JIG_ON", NULL);
	if (ret)
	 	ACC_CONDEV_DBG("Fail to register IRQ : OMAP_GPIO_JIG_ON18 return : %d\n",ret);
}

void jig_on_work_handler(struct work_struct *_work)
{
	int value;
	value = gpio_get_value(OMAP_GPIO_JIG_ON18);
	printk(KERN_DEBUG "[JIG] JIG ON : %d\n", value);
// TEMP BLOCK	set_uart_suspend(!value);
	enable_irq(IRQ_JIG_ON_INT);
}

void acc_notified(int acc_adc)	
{
	char env_buf[60];
	char stat_buf[60];
    char *envp[3];
    int env_offset = 0;
	memset(env_buf, 0, sizeof(env_buf));
	memset(stat_buf, 0, sizeof(stat_buf));
	if(acc_adc != false)
	{
		if((2450<acc_adc) && (2650>acc_adc))
		{
			sprintf(env_buf, "ACCESSORY=downloadmode");
			CONNECTED_ACC = ACC_DOWNLOADMODE;
		}
		else if((1900<acc_adc) && (2100>acc_adc))
		{
			sprintf(env_buf, "ACCESSORY=camerakit");
			CONNECTED_ACC = ACC_CAMERAKIT;
		}
		else if((1540<acc_adc) && (1740>acc_adc))
		{
			sprintf(env_buf, "ACCESSORY=TV");
			CONNECTED_ACC = ACC_TVOUT;
		}
		else if((1160<acc_adc) && (1360>acc_adc))
		{
			sprintf(env_buf, "ACCESSORY=carmount");
			CONNECTED_ACC = ACC_CARMOUNT;
		}
		else if((800<acc_adc) && (1000>acc_adc))
		{
			sprintf(env_buf, "ACCESSORY=lineout");
			CONNECTED_ACC = ACC_LINEOUT;
		}
		else
		{
			sprintf(env_buf, "ACCESSORY=unknown");
			CONNECTED_ACC = 0;
		}
		sprintf(stat_buf, "STATE=online");
		envp[env_offset++] = env_buf;
		envp[env_offset++] = stat_buf;
		envp[env_offset] = NULL;
		if(CONNECTED_ACC == ACC_TVOUT)
		{
			//max8998_ldo3_8_control(1,LDO_TV_OUT); //ldo 3,8 on
			//printk("%s: LDO3_8 is enabled by TV \n", __func__);
			//msleep(100);
			//TVout_LDO_ctrl(true);
		}
		kobject_uevent_env(&acc_dev->kobj, KOBJ_CHANGE, envp);
		ACC_CONDEV_DBG("%s : %s",env_buf,stat_buf);
	}
	else
	{
		if(CONNECTED_ACC == ACC_DOWNLOADMODE)
			sprintf(env_buf, "ACCESSORY=downloadmode");
		else if(CONNECTED_ACC == ACC_CAMERAKIT)
			sprintf(env_buf, "ACCESSORY=camerakit");
		else if(CONNECTED_ACC == ACC_TVOUT)
			sprintf(env_buf, "ACCESSORY=TV");
		else if(CONNECTED_ACC == ACC_CARMOUNT)
			sprintf(env_buf, "ACCESSORY=carmount");
		else if(CONNECTED_ACC == ACC_LINEOUT)
			sprintf(env_buf, "ACCESSORY=lineout");
		else
			sprintf(env_buf, "ACCESSORY=unknown");
		
		sprintf(stat_buf, "STATE=offline");
		envp[env_offset++] = env_buf;
		envp[env_offset++] = stat_buf;
		envp[env_offset] = NULL;
		kobject_uevent_env(&acc_dev->kobj, KOBJ_CHANGE, envp);
		if(CONNECTED_ACC == ACC_TVOUT)
		{
			//msleep(200);
			//max8998_ldo3_8_control(0,LDO_TV_OUT);  //ldo 3,8 off
			//printk("%s: LDO3_8 is disabled by TV \n", __func__);
			//TVout_LDO_ctrl(false);
		}
		ACC_CONDEV_DBG("%s : %s",env_buf,stat_buf);
	}
	
}
void acc_ID_intr_handle(struct work_struct *_work)
{
	//ACC_CONDEV_DBG("");
	int acc_ID_val = 0, adc_val;
	acc_ID_val = gpio_get_value(OMAP_GPIO_DOCK_INT);
	ACC_CONDEV_DBG("GPIO_DOCK_INT is %d",acc_ID_val);

	if(acc_ID_val!=ACC_STATE)
	{
		if(1==acc_ID_val)
		{
			ACC_CONDEV_DBG("Accessory detatched");
			ACC_STATE = acc_ID_val;
			acc_notified(false);
			set_irq_type(IRQ_DOCK_INT, IRQ_TYPE_EDGE_FALLING);
#ifdef CONFIG_USB_S3C_OTG_HOST
		if(intr_count++)
			s3c_usb_cable(USB_OTGHOST_DETACHED);
#endif
		}
		else if(0==acc_ID_val)
		{
			msleep(420); //workaround for jack
			ACC_CONDEV_DBG("Accessory attached");
			ACC_STATE = acc_ID_val;
			adc_val = connector_detect_change();
			acc_notified(adc_val);
			set_irq_type(IRQ_DOCK_INT, IRQ_TYPE_EDGE_RISING);
#ifdef CONFIG_USB_S3C_OTG_HOST
		// check USB OTG Host ADC range...
		if(adc_val > 2700 && adc_val < 2799) {
			s3c_usb_cable(USB_OTGHOST_ATTACHED);
		}
#endif
		}
	}
	else
	{
		ACC_CONDEV_DBG("Ignored");
	}

	enable_irq(IRQ_DOCK_INT);
}

irqreturn_t acc_ID_interrupt(int irq, void *ptr)
{
	//ACC_CONDEV_DBG("");
	disable_irq_nosync(IRQ_DOCK_INT);
	queue_work(acc_ID_workqueue, &acc_ID_work);

	return IRQ_HANDLED; 
}

void acc_ID_interrupt_init(void)
{
	int ret;
	//s3c_gpio_cfgpin(GPIO_DOCK_INT, S3C_GPIO_SFN(GPIO_DOCK_INT_AF));
	//s3c_gpio_setpull(GPIO_DOCK_INT, S3C_GPIO_PULL_NONE);
	//gpio_direction_input(OMAP_GPIO_DOCK_INT);
	set_irq_type(IRQ_DOCK_INT, IRQ_TYPE_EDGE_BOTH);
	
	ret = request_irq(IRQ_DOCK_INT, acc_ID_interrupt, IRQF_DISABLED, "Accessory Detected", NULL);
	if (ret)
	 	ACC_CONDEV_DBG("Fail to register IRQ : GPIO_DOCK_INT return : %d\n",ret);
		 
}


#ifdef SUPPORT_MHD_DEVICE 
void acc_MHD_intr_handle(struct work_struct *_work)
{

	int val = gpio_get_value(GPIO_MHL_INT);
	ACC_CONDEV_DBG("++GPIO_MHL_INT =  %x",val);
	if(val && (!gpio_get_value(GPIO_ACCESSORY_INT)))
		MHD_OUT_EN();
	enable_irq(IRQ_MHL_INT);
}


irqreturn_t acc_MHD_interrupt(int irq, void *ptr)
{
	disable_irq_nosync(IRQ_MHL_INT);
	queue_work(acc_MHD_workqueue, &acc_MHD_work);

	return IRQ_HANDLED; 
}



void acc_MHD_interrupt_init(void)
{
	int ret;
	
	s3c_gpio_cfgpin(GPIO_MHL_INT, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_MHL_INT, S3C_GPIO_PULL_DOWN);
	set_irq_type(IRQ_MHL_INT, IRQ_TYPE_EDGE_RISING);
	
	ret = request_irq(IRQ_MHL_INT, acc_MHD_interrupt, IRQF_DISABLED, "MHL recovery", NULL);
	if (ret)
	 	ACC_CONDEV_DBG("Fail to register IRQ : GPIO_MHL_INT return : %d\n",ret);

}
#endif

static int acc_con_probe(struct platform_device *pdev)
{
	int 	retval;
	
	ACC_CONDEV_DBG("");	

	acc_dev = &pdev->dev;
#ifdef CONFIG_MHL_SII9234
	
		retval = i2c_add_driver(&SII9234A_i2c_driver);
		if (retval != 0)
			printk("[MHL SII9234A] can't add i2c driver\n");	
		else
			printk("[MHL SII9234A] add i2c driver\n");
	
		retval = i2c_add_driver(&SII9234B_i2c_driver);
		if (retval != 0)
			printk("[MHL SII9234B] can't add i2c driver\n");	
		else
			printk("[MHL SII9234B] add i2c driver\n");
	
		retval = i2c_add_driver(&SII9234C_i2c_driver);
		if (retval != 0)
			printk("[MHL SII9234C] can't add i2c driver\n");	
		else
			printk("[MHL SII9234C] add i2c driver\n");
		
		retval = i2c_add_driver(&SII9234_i2c_driver);
		if (retval != 0)
			printk("[MHL SII9234] can't add i2c driver\n");	
		else
			printk("[MHL SII9234] add i2c driver\n");


	
	//MHD_HW_Reset();  //9234 goes to D2
	MHD_HW_Off();
#endif		
	
	INIT_WORK(&acc_con_work, acc_con_intr_handle);
	acc_con_workqueue = create_singlethread_workqueue("acc_con_workqueue");

#if defined (CONFIG_TARGET_LOCALE_EUR) || defined (CONFIG_TARGET_LOCALE_HKTW) || defined (CONFIG_TARGET_LOCALE_HKTW_FET) || defined (CONFIG_TARGET_LOCALE_VZW) || defined (CONFIG_TARGET_LOCALE_USAGSM)
	if(HWREV >=0xB)
	{
	INIT_WORK(&acc_ID_work, acc_ID_intr_handle);
	acc_ID_workqueue = create_singlethread_workqueue("acc_ID_workqueue");
	acc_ID_interrupt_init();
	}
#else
#ifdef CONFIG_TARGET_LOCALE_KOR
	if(HWREV >=0xA)
	{
		INIT_WORK(&acc_ID_work, acc_ID_intr_handle);
		acc_ID_workqueue = create_singlethread_workqueue("acc_ID_workqueue");
		acc_ID_interrupt_init();
	}
#else
	INIT_WORK(&acc_ID_work, acc_ID_intr_handle);
	acc_ID_workqueue = create_singlethread_workqueue("acc_ID_workqueue");
	acc_ID_interrupt_init();
#endif
#endif

#ifdef SUPPORT_MHD_DEVICE 
	INIT_WORK(&acc_MHD_work, acc_MHD_intr_handle);
	acc_MHD_workqueue = create_singlethread_workqueue("acc_MHD_workqueue");
	acc_MHD_interrupt_init();
#endif

	INIT_WORK(&jig_on_work, jig_on_work_handler);
	jig_on_workqueue = create_singlethread_workqueue("jig_on_workqueue");
	jig_on_interrupt_init();
	if(gpio_get_value(OMAP_GPIO_JIG_ON18))
	{
		disable_irq(IRQ_JIG_ON_INT);
		queue_work(jig_on_workqueue, &jig_on_work);
	}

#if defined(CONFIG_KEYBOARD_P1)
    wake_lock_init(&wake_lock_for_kbd, WAKE_LOCK_SUSPEND, "kbd_wake_lock");
#endif

#ifdef SUPPORT_MHD_DEVICE 
	if (device_create_file(acc_dev, &dev_attr_MHD_file) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_MHD_file.attr.name);
#endif

	if (device_create_file(acc_dev, &dev_attr_acc_file) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_acc_file.attr.name);
	

	init_timer(&connector_detect_timer);
	connector_detect_timer.function = acc_con_detect_timer_handler;
	connector_detect_timer.expires = DETECTION_INTR_DELAY;
	add_timer(&connector_detect_timer);

	
	return 0;
}

static int acc_con_remove(struct platform_device *pdev)
{
	ACC_CONDEV_DBG("");
#ifdef CONFIG_MHL_SII9234
	i2c_del_driver(&SII9234A_i2c_driver);
	i2c_del_driver(&SII9234B_i2c_driver);
	i2c_del_driver(&SII9234C_i2c_driver);
	i2c_del_driver(&SII9234_i2c_driver);
#endif	
	return 0;
}

static int acc_con_suspend(struct platform_device *pdev, pm_message_t state)
{
	ACC_CONDEV_DBG("");
#ifdef CONFIG_MHL_SII9234	
	//call MHL deinit
	MHD_HW_Off();
	//msleep(120);
	//max8998_ldo3_8_control(0,LDO_TV_OUT);  //ldo 3,8 off
	//printk("%s: LDO3_8 is disabled by TV \n", __func__);
#endif	
	return 0;
}

static int acc_con_resume(struct platform_device *pdev)
{
	ACC_CONDEV_DBG("");
	if(0 == gpio_get_value(OMAP_GPIO_ACCESSORY_INT))
	{
#if 0
		if(CONNECTED_DOCK == DOCK_KEYBD)
		{
			check_keyboard_dock();
		}
		else
#endif
             if(CONNECTED_DOCK == DOCK_DESK)
		{
#ifdef CONFIG_MHL_SII9234		
			//max8998_ldo3_8_control(1,LDO_TV_OUT); //ldo 3,8 on
			//printk("%s: LDO3_8 is enabled by TV \n", __func__);
			//msleep(120);
			
			//call MHL init 
			sii9234_tpi_init();
#endif
		}

	}
	return 0;
}


static int __init acc_con_init(void)
{
	int ret;
#if defined (CONFIG_TARGET_LOCALE_EUR) || defined (CONFIG_TARGET_LOCALE_HKTW) || defined (CONFIG_TARGET_LOCALE_HKTW_FET) || defined (CONFIG_TARGET_LOCALE_VZW) || defined (CONFIG_TARGET_LOCALE_USAGSM)
	if(HWREV < 0x8)
		return -1;
#endif	
	ACC_CONDEV_DBG("");
	
	ret = platform_driver_register(&acc_con_driver);
	ACC_CONDEV_DBG("ret",ret);

	return ret;
}

static void __exit acc_con_exit(void)
{
	platform_driver_unregister(&acc_con_driver);
}

static struct platform_driver acc_con_driver = {
	.probe		= acc_con_probe,
	.remove		= acc_con_remove,
	.suspend	= acc_con_suspend,
	.resume		= acc_con_resume,
	.driver		= {
		.name		= "acc_con",
		.owner		= THIS_MODULE,
	},
};


late_initcall(acc_con_init);
module_exit(acc_con_exit);

MODULE_AUTHOR("Kyungrok Min <gyoungrok.min@samsung.com>");
MODULE_DESCRIPTION("acc connector driver");
MODULE_LICENSE("GPL");
