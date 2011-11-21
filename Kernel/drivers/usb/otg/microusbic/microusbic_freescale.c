
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
  
  
#include <plat/gpio.h> 
#include <plat/mux.h>
#include <plat/microusbic.h>               

#include "microusbic_dbg.h"

  
//#define REGISTER_MISC_DEVICE
//#define TIME_TEST


#ifdef REGISTER_MISC_DEVICE
#include <linux/miscdevice.h>
#endif
  

#include <mach/board-oscar.h> 
/////////////////////////////////////////////////////////////////////////////////
//
// REGISTER
//

#define MUX_CONTROL				0x02

#define MUX_INTERRUPT1			0x03
#define MUX_INTERRUPT2			0x04
#define MUX_INTERRUPT_MASK1		0x05
#define MUX_INTERRUPT_MASK2		0x06

#define MUX_DEVICE_TYPE1		0x0A
#define MUX_DEVICE_TYPE2		0x0B

#define MUX_MANUAL_SW1			0x13

// MicroUSB Switch / 091104 / ranjit / HQ
#define MUX_SWITCH_VALUE	0x49 // SPK
#define MUX_DEVICE_MODE	0x23

#define I2C_ADDRESS_TCA6416	0x25

/////////////////////////////////////////////////////////////////////////////////
//
// Definitions
//


static struct i2c_client *g_client;
static struct work_struct 	usb_switch_work;

static int usbic_state	= MICROUSBIC_NO_DEVICE;
static int usbic_switch_state =1; // default value is 1

// static functions
static int sync_usbic_state(void);

// Exported Common API
int usbic_switch_onoff(int enable);

extern int usbsel;
extern void (*usbsel_notify)(int);
extern int check_device_connected();
/////////////////////////////////////////////////////////////////////////////////
//
// I2C Driver
//

#if 1
int uusbic_read(u8 reg, u8 * value, u8 len)
{
	int ret;
	struct i2c_msg msg[2];

	if((g_client == NULL) || (!g_client->adapter))
	{
		printk("Error!!! i2c driver is null!\n");
		return -ENODEV;
	}

	msg[0].addr = g_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = g_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = value;

	ret = i2c_transfer(g_client->adapter, msg, 2);

	if(ret >= 0)
	{
		ret =0;
	}
	return ret;
}
#else
int uusbic_read(u8 reg, u8 * value, u8 len)
{
	int ret;
	struct i2c_msg msg[2];

	if((g_client == NULL) || (!g_client->adapter))
	{
		kmsg("Error!!! i2c driver is null!\n");
		return -ENODEV;
	}

	msg[0].addr = g_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	ret = i2c_transfer(g_client->adapter, msg, 1);

	if (ret >= 0) {
		msg[1].addr = g_client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = len;
		msg[1].buf = value;
		ret = i2c_transfer(g_client->adapter, &msg[1], 1);
	}

	kmsg("i2c_transfer : %d\n",ret);
	if(ret >= 0)
	{
		ret =0;
	}
	return ret;
}
#endif


#if 1
int uusbic_read_u8(u8 reg, u8 *value)
{
	return uusbic_read(reg,value,1);
}
#else
int uusbic_read_u8(u8 reg, u8 *val)
{
	int err;
	struct i2c_msg msg[1];
	struct i2c_msg msg2[1];
	unsigned char data[1];
	unsigned char data2[1];

	if( (g_client == NULL) || (!g_client->adapter) )
		return -ENODEV;

	msg->addr = g_client->addr;
	msg->flags |= I2C_M_WR;
	//msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
	data[0] = reg;
	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) {
		//mdelay(3);
		msg2->addr = g_client->addr;
		msg2->flags |= I2C_M_RD;
		msg2->len = 1;
		msg2->buf = data2;
		err = i2c_transfer(g_client->adapter, msg2, 1);
	}

	if (err >= 0) {
		*val = *data2;
		return 0;
	}

	return err;
}
#endif

int uusbic_write_u8(u8 reg, u8 value)
{
	int ret;
	struct i2c_msg msg[1];
	u8 data[2];


	if((g_client == NULL) || (!g_client->adapter))
	{
		return -ENODEV;
	}

	msg->addr = g_client->addr;
	msg->len = 2;
	msg->flags = 0;
	msg->buf = data;
	data[0] = reg;
	data[1] = value;

	ret = i2c_transfer(g_client->adapter, msg,1);
	if( ret >= 0 )
	{
		return 0;
	}
	return ret;
}



static int microusbic_i2c_remove(struct i2c_client * client)
{
	int ret=0;



	g_client = NULL;

	return ret;
}


static int microusbic_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{

	return 0;
}

static int microusbic_i2c_resume(struct i2c_client *client)
{
	printk("  client name : %s\n", client->name);
	return 0;
}




/////////////////////////////////////////////////////////////////////////////////
//
// SYS Debug Function
//

#ifdef REGISTER_MISC_DEVICE
static struct miscdevice microusbic_device = 
{
	.minor = 150,
	.name = "microusbic",
};
#endif

inline int show_column(char * buf)
{ 												
	char * s = buf;
	int i=0;

	s += sprintf(s,"    | ");

	for( i = 7; i >= 0; i--)
		s += sprintf(s,"%02x ",i);

	s += sprintf(s, "  value\n----+");


	for(i = 0; i < 9; i++)
		s += sprintf(s, " --");

	s += sprintf(s, "\n");

	return s - buf;
}

inline int show_reg(u8 reg, char * buf)
{
	char * s = buf;
	u8 v = 0;
	int j;

	uusbic_read_u8(reg, &v);

	s += sprintf(s, "0x%02x| ",reg);

	for(j=7; j >= 0; j--)
	{
		s += sprintf(s, " %d", (v & (1<<j))? 1:0);
		s += (j != 0)?	sprintf(s, " ") :
			 			sprintf(s, "   0x%02x\n", v);
	}
	return s - buf;
}


ssize_t show_uusbic1(struct device *dev,struct device_attribute * attr, char *buf)
{
	u8 value[20 + 1];
	int ret, i,j,v;
	char * s = buf;

	s += show_column(s);

	/* print 0x1 ~ 0x14 */
	ret = uusbic_read( 1, value, 20 );
	for(i = 1; i < 0x15; i++)
	{
		v = value[i-1];
		s += sprintf(s, "0x%02x| ",i);

		for(j=7; j >= 0; j--)
		{
			s += sprintf(s, " %d", (v & (1<<j))? 1:0);
			s += (j != 0) ?	sprintf(s, " ") :
							sprintf(s, "   0x%02x\n", v);
		}
	}

	/* clear buffer */
	for(i= 0;i<21;i++)
		value[i]=0;

	/* print 0x20 ~ 0x25 */
	ret = uusbic_read( 0x20, value, 6 );
	for(i = 0x20; i < 0x26; i++)
	{
		v = value[i-0x20];
		s += sprintf(s, "0x%02x| ",i);

		for(j=7; j >= 0; j--)
		{
			s += sprintf(s, " %d", (v & (1<<j))? 1:0);
			s += (j != 0) ?	sprintf(s, " ") :
							sprintf(s, "   0x%02x\n", v);
		}
	}

	s += 1;
	*s = 0;
	return s - buf +1;
}

ssize_t show_uusbic2(struct device *dev,struct device_attribute * attr, char *buf)
{
	int i;
	char * s = buf;

	s += show_column(s);

	/* print 0x1 ~ 0x14 */
	for(i = 0x1 ; i < 0x15; i++)
		s += show_reg(i,s);

	/* print 0x20 ~ 0x25 */
	for(i = 0x20 ; i < 0x26; i++)
		s += show_reg(i,s);

	s += 1;
	*s = 0;
	return s - buf +1;
}

ssize_t set_uusbic(struct device *dev,struct device_attribute * attr,
		const char * buf, size_t count)
{
	u8 reg, val;
	u16 data = (u16) simple_strtoul(buf, NULL, 16);

	reg = (data & 0xff00) >> 8;
	val = (data & 0x00ff);

	printk("Reg = 0x%02x, Val = 0x%02x\n", reg, val);

	uusbic_write_u8(reg,val);
	return count;
}

ssize_t show_intr(struct device *dev,struct device_attribute * attr, char *buf)
{
	char * s = buf;

	s += sprintf(s, "[ UIC ] MicroUSB_nINT gpio %d, level %d\n",
		OMAP3430_GPIO_MICRO_nINT, omap_get_gpio_datain(OMAP3430_GPIO_MICRO_nINT));

	s += 1;
	*s = 0;
	return s - buf +1;
}

ssize_t set_intr(struct device *dev,struct device_attribute * attr,
		const char * buf, size_t count)
{
	return count;
}


#define MAKE_REG(num) 								\
ssize_t show_reg_##num(struct device *dev, 			\
		struct device_attribute * attr, char *buf) 	\
{ 													\
	char * s = buf; 								\
	s += show_column(s);							\
	s += show_reg(0x##num,s); 						\
	s += 1; 										\
	*s = 0; 										\
	return s - buf +1; 								\
} 													\
ssize_t set_reg_##num(struct device *dev, 			\
		struct device_attribute * attr, 			\
		const char * buf, size_t count) 			\
{ 													\
	u8 val = (u8) simple_strtoul(buf, NULL, 16); 	\
	printk("0x%02x Val = 0x%02x\n", 0x##num, val); 	\
	uusbic_write_u8(0x##num,val); 					\
	return count; 									\
} 													\
static DEVICE_ATTR(r##num, S_IWUSR | S_IRUGO, show_reg_##num, set_reg_##num)

#define MAKE_FILE(num) \
do { \
	ret = device_create_file(dev, &dev_attr_r##num); \
} while(0)


ssize_t set_mux_switch(struct device *dev,struct device_attribute * attr,
		const char * buf, size_t count)
{
	int onoff = (int) simple_strtol(buf, NULL, 10);

	usbic_switch_onoff(onoff);
	//omap_set_gpio_dataout(OMAP3430_GPIO_MICRO_nINT, 0);

	return count;
}

ssize_t show_mux_switch(struct device *dev,struct device_attribute * attr, char *buf)
{
	char * s = buf;

	s += sprintf(s, "[ UIC ] mux switch : %s\n", usbic_switch_state? "ON" : "OFF");

	s += 1;
	*s = 0;
	return s - buf +1;
}

static DEVICE_ATTR(switch, S_IWUSR | S_IRUGO, show_mux_switch, set_mux_switch);
static DEVICE_ATTR(uusbic, S_IWUSR | S_IRUGO, show_uusbic1, set_uusbic);
static DEVICE_ATTR(ouusbic, S_IWUSR | S_IRUGO, show_uusbic2, set_uusbic);
static DEVICE_ATTR(intr, S_IWUSR | S_IRUGO, show_intr, set_intr);

MAKE_REG(2);
MAKE_REG(13);




/////////////////////////////////////////////////////////////////////////////////
//
// Time test
//

#ifdef TIME_TEST
u64 start_time;
u64 end_time;

#define START \
	do{ \
		start_time = get_jiffies_64(); \
		printk(KERN_ALERT"==> [s] : %s(%d)\n", __FUNCTION__,__LINE__); \
	} while(0)

#define END \
	do{ \
		end_time = get_jiffies_64(); \
		printk(KERN_ALERT"<== [e] : %s(%d) : %llu ms\n", __FUNCTION__,__LINE__,(end_time-start_time) * 10); \
	} while(0)

static int test_count =400;
ssize_t show_timetest(struct device *dev,struct device_attribute * attr, char *buf)
{
	int i;
	char * s = buf;

	START;
	for( i =0; i < test_count;i++)
	{
		uusbic_write_u8(0x24,1);
	}
	END;

	s += 1;
	*s = 0;
	return s - buf +1;
}
ssize_t set_timetest(struct device *dev,struct device_attribute * attr,
		const char * buf, size_t count)
{
	u32 data = (u32) simple_strtoul(buf, NULL, 16);

	printk("count %d\n",data);

	test_count = data;

	return count;
}

static DEVICE_ATTR(timetest, S_IWUSR | S_IRUGO, show_timetest, set_timetest);
#endif
/////////////////////////////////////////////////////////////////////////////////



static int microusbic_sysfs_create(void)
{
	struct device *dev;
	int ret=0;


#ifdef REGISTER_MISC_DEVICE
	dev = microusbic_device.this_device;
#else
	if(g_client == NULL)
	{
		return -ENODEV;
	}
	else
	{
		dev = &g_client->dev;
	}
#endif

	ret = device_create_file(dev, &dev_attr_switch);

	ret = device_create_file(dev, &dev_attr_uusbic);
	ret = device_create_file(dev, &dev_attr_ouusbic);
	ret = device_create_file(dev, &dev_attr_intr);

#ifdef TIME_TEST
	ret = device_create_file(dev, &dev_attr_timetest);
#endif

	MAKE_FILE(2);
	MAKE_FILE(13);

	return ret;
}

static void microusbic_sysfs_remove(void)
{
}


/////////////////////////////////////////////////////////////////////////////////
//
// print functions
//

static void print_device_type(u8 type1, u8 type2)
{
	printk("[ UIC ] Device Type : %s\n", 
			(type1==(1<<2))? "USB":
			(type1==(1<<3))? "UART":
			(type1==(1<<4))? "5W Charger":
			(type1==(1<<5))? "USB CHG":
			(type1==(1<<6))? "Dedicated CHG":
			(type1==(1<<7))? "USB OTG":
			(type2==(1<<0))? "JIG_USB_ON":
			(type2==(1<<1))? "JIG_USB_OFF":
			(type2==(1<<2))? "JIG_UART_ON":
			(type2==(1<<3))? "JIG_UART_OFF":
			(type2==(1<<5))? "TTY":
			"Unknown");
}



/////////////////////////////////////////////////////////////////////////////////
//
// static functions
//

inline static void check_usbic_intb(void)
{
	printk("  gpio %d, level %s\n", OMAP3430_GPIO_MICRO_nINT,omap_get_gpio_datain(OMAP3430_GPIO_MICRO_nINT) ? "HIGH":"LOW" );
}

inline static void usbic_intb_low(void)
{
	omap_set_gpio_dataout(OMAP3430_GPIO_MICRO_nINT, 0);
}

inline static int rescan_accessory(void)
{
	printk("[ UIC ] rescan_accessory\n");
	return uusbic_write_u8(MUX_DEVICE_MODE, 0);
}

static int sync_usbic_state(void)
{
	int ret = 0;
	u8 value;
	u8 value2;

	ret = uusbic_read_u8(MUX_DEVICE_TYPE1, &value);
	ret = uusbic_read_u8(MUX_DEVICE_TYPE2, &value2);

	print_device_type(value,value2);

	if(!usbsel)
	{
		printk ("  Phone USB detected\n");
		ret = MICROUSBIC_PHONE_USB;
/* FIXME: ARCHER -> OMAP_SAMSUNG */
#if defined CONFIG_MACH_OMAP_SAMSUNG
		gpio_set_value(OMAP_GPIO_CP_VBUS_EN, 1);
		printk("RANJ **** MODEM USB SELECT GPIO is set to HIGH \n");
#endif
		usbic_state = ret;
		return ret;
	}
	else if(value & (1<<6))
	{
		printk("  Dedicated Charger\n");
		ret = MICROUSBIC_TA_CHARGER;
	}
	else if(value & (1<<5))
	{
		printk(" USB Charge\n");
		ret = MICROUSBIC_USB_CHARGER;
	}
	else if(value & (1<<4))
	{
		printk(" 5W Charger\n");
		ret = MICROUSBIC_5W_CHARGER;
	}
	else if(value & (1<<2))
	{
		printk("  USB detected\n");
		ret = MICROUSBIC_USB_CABLE;
	}
	else if(value2 & (1<<2))
	{
		printk("  JIG_UART_ON detected\n");
		ret = MICROUSBIC_JIG_UART_ON;
	}
	else if(value2 & (1<<3))
	{
		printk("  JIG_UART_OFF detected\n");
		ret = MICROUSBIC_JIG_UART_OFF;
	}
	else
	{
		printk("  Not connected\n");
		ret = MICROUSBIC_NO_DEVICE;
	}


	usbic_state = ret;
	return ret;
}



/////////////////////////////////////////////////////////////////////////////////
//
// Interrupt API
//


int get_interrupt_event(struct microusbic_event * evt)
{
	u8 intr[2];
	int ret =0;

	ret = uusbic_read(MUX_INTERRUPT1, intr, 2);	// read interrupt regs 1,2



	// interrupt
	if((intr[0] == 0) && (intr[1] == 0))
	{
		evt->device = MICROUSBIC_NO_DEVICE;
		evt->event = 0;
		return 0;
	}
	else if(intr[0] & 1) // attach
	{
#if 0
		evt->device = sync_usbic_state();
#else
		 _get_real_usbic_state();
		evt->device = usbic_state;
#endif
		evt->event = MICROUSBIC_ATTACH;
	}
	else if(intr[0] & 2) // detach
	{
		evt->device = usbic_state;
		evt->event = MICROUSBIC_DETACH;
		usbic_state = MICROUSBIC_NO_DEVICE;
/* FIXME: ARCHER -> OMAP_SAMSUNG */
#if defined CONFIG_MACH_OMAP_SAMSUNG
		if(!usbsel){
			gpio_set_value(OMAP_GPIO_CP_VBUS_EN, 0);
			printk("RANJ ***** MODEM USB SELECT GPIO is set to LOW \n");
		}
#endif
	}
	else
	{
	
		evt->device = MICROUSBIC_NO_DEVICE;
		evt->event = -1;
		return 0;
	}

	return 1;
}



static int usbic_init_irq(int enable, u8 int1, u8 int2)
{
	u8 value;
	int ret =0;

	ret = uusbic_read_u8(MUX_CONTROL , &value);

	if(enable)
		value &= ~(0x01);
	else
		value |= 0x01;

	ret = uusbic_write_u8(MUX_CONTROL, value);
	


	ret = uusbic_write_u8(MUX_INTERRUPT_MASK1, int1);
	ret = uusbic_write_u8(MUX_INTERRUPT_MASK2, int2);
	value=0;
		ret = uusbic_read_u8(MUX_CONTROL , &value);

#if 0
	rescan_accessory();
#endif

	return ret;	
}



/////////////////////////////////////////////////////////////////////////////////
//
// Exported Common API
//

int get_usbic_state(void)
{
	printk(" state  %s (%d)\n", 
			(usbic_state==0) ? "No_Device" :
			(usbic_state==1) ? "USB_Cable" :
			(usbic_state==2) ? "USB Charger" :
			(usbic_state==3) ? "TA Charger" :
			(usbic_state==4) ? "JIG UART ON" :
			(usbic_state==5) ? "JIG UART OFF" :
			(usbic_state==-1) ? "Phone USB" :
			"Unknown_Device", usbic_state); 
	return usbic_state;
}
EXPORT_SYMBOL(get_usbic_state);

int get_real_usbic_state(void)
{
	return sync_usbic_state();
}
EXPORT_SYMBOL(get_real_usbic_state);

/*
 * switch connections selection
 *	sel : 	1 pda (auto mode)
 *			0 phone (mamual mode for phone)
 *	called by DCM
 */
void usbic_usb_switch(int sel)
{
	u8 i=0;

	uusbic_read_u8(MUX_CONTROL, &i);

	if(sel) /* PDA */
	{
		printk(" PDA\n");

		if(!(i & 0x04))
		{
			i |= 0x04;
			uusbic_write_u8(MUX_CONTROL,i);
			rescan_accessory();
		}
	}
	else /* Phone */
	{

		i &= ~(0x4);
		i |= (1<<4); // switch on
		uusbic_write_u8(MUX_CONTROL,i);
		uusbic_write_u8(MUX_MANUAL_SW1, MUX_SWITCH_VALUE);
	}
	//usbic_init_irq(1,0,0);
	sync_usbic_state();
}
EXPORT_SYMBOL(usbic_usb_switch);

// MicroUSB Switch / 091104 / ranjit / HQ
void usb_bootmode(void)
{
	u8 tmp=0;
	u8 i =0;

	uusbic_read_u8(MUX_MANUAL_SW1, &tmp);
	uusbic_read_u8(MUX_CONTROL, &i);
	printk("RANJ *****bootmode()**** MUX_MANUAL_SW1	 = %02x, MUX_CONTROL = %02x \n", tmp, i);

	if((tmp == 0x49) && (usbsel == 1))
		usbsel = 0;
}

/*
 * Switch connection selection
 * 	enable  0 : Open all switches.
 * 			1 : Automatic switching by accessory status
 */
int usbic_switch_onoff(int enable)
{
	u8 i=0;

	uusbic_read_u8(MUX_CONTROL, &i);

	if(enable) /* Automatic switching */
	{
		usbic_switch_state = 1;

		i |= (1<<4);
	}
	else
	{
		usbic_switch_state = 0;
	
		i &= ~(1<<4);
	}
	uusbic_write_u8(MUX_CONTROL,i);
	//usbic_init_irq(1,0,0);
	sync_usbic_state();

	if(!enable)
		usbic_intb_low();

	return 0;
}
EXPORT_SYMBOL(usbic_switch_onoff);


/////////////////////////////////////////////////////////////////////////////////
//
// Phone usb
//

static void usb_switch_handler(struct work_struct * work)
{
	printk("%s (%d) : USB Path %s\n", __FUNCTION__,__LINE__,usbsel?"PDA":"Phone");
	usbic_usb_switch(usbsel);
}

static void usb_switching(int flags)
{
	schedule_work(&usb_switch_work);
}



/////////////////////////////////////////////////////////////////////////////////
//
// Init & Exit
//
static int microusbic_i2c_probe(struct i2c_client * client,const struct i2c_device_id *id)
{
        int ret=0;
	u8 intr;

        if(client == NULL)
        {
                printk(" client is null\n");
        }
        g_client = client;
      
        return ret;
}

static const struct i2c_device_id microusbic_id[] = {
        { "microusbic", 0 },
        { }
};


struct i2c_driver microusbic_i2c_driver =
{
        .driver = {
                .name = "microusbic",
        },

        .probe = microusbic_i2c_probe,
        .remove = microusbic_i2c_remove,

        .suspend = microusbic_i2c_suspend,
        .resume = microusbic_i2c_resume,
        .id_table = microusbic_id
};

void microusbic_i2c_exit(void)
{
        i2c_del_driver(&microusbic_i2c_driver);
}


int  microusbic_i2c_init(void)
{
        int ret;

        if ( (ret = i2c_add_driver(&microusbic_i2c_driver)) ) {
                printk("Driver registration failed, module not inserted.\n");
                return ret;
        }

        return 0;
}


//static struct i2c_client *microusbic_client;
int microusbic_late_init(void)
{
	u8 intr;
	int cable_detect=0;
	
#ifdef REGISTER_MISC_DEVICE
	int result;
	result = misc_register(&microusbic_device);
	if (result < 0) return result;
#endif


	printk(" Microusbic  INIT \n");	
/* FIXME: ARCHER -> OMAP_SAMSUNG */
#if defined CONFIG_MACH_OMAP_SAMSUNG
	if(gpio_request(OMAP_GPIO_CP_VBUS_EN, "Phone_VBUS_enable")<0) {
		printk(KERN_ERR "Failed to get OMAP_GPIO_USB_SEL pin \n");
		return;
	}
	printk("RANJ ** MODEM USB SELECT PIN = %d configured as output \n", (int)OMAP_GPIO_CP_VBUS_EN);
	gpio_direction_output(OMAP_GPIO_CP_VBUS_EN, 0);
#endif
	microusbic_i2c_init();



	microusbic_sysfs_create();

	usbic_init_irq(1,0,0);

	check_usbic_intb();

	uusbic_read_u8(MUX_INTERRUPT1, &intr);	// clear interrupt

	check_usbic_intb();

	usbic_switch_onoff(1); // default is off

	// MicroUSB Switch / 091104 / ranjit / HQ
	usb_bootmode();
	
	//sync_usbic_state();
	
	INIT_WORK( &usb_switch_work, usb_switch_handler);
	
	printk("[ UIC ] set usbsel notifier.\n");
	usbsel_notify = usb_switching;
	if(usbsel_notify == NULL)
	{
		printk("[ UIC ] usbsel is not set!\n");
	}
	cable_detect=check_device_connected();
	if(cable_detect)
		printk(" USB Cable detected at boot time \n");

	return 0;
}

void microusbic_late_exit(void)
{
	printk("exit\n");
	microusbic_sysfs_remove();

	microusbic_i2c_exit();

#ifdef REGISTER_MISC_DEVICE
	misc_deregister(&microusbic_device);
#endif

}


fs_initcall(microusbic_late_init);


module_exit(microusbic_late_exit);

MODULE_AUTHOR("Hyuk Kang hyuk78.kang@samsung.com");
MODULE_DESCRIPTION("micro usb i2c");
MODULE_LICENSE("GPL");
