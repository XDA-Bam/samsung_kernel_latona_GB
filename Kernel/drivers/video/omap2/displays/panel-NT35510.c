/*
 * nt35510 LDI support
 *
 * Copyright (C) 2009 Samsung Corporation
 * Author: Samsung Electronics..
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c/twl.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
/*
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/mux.h>
#include <asm/mach-types.h>
#include <mach/control.h>

#include <mach/display.h>
*/
#include <plat/gpio.h>
#include <plat/hardware.h>
#include <plat/mux.h>
#include <asm/mach-types.h>
#include <plat/control.h>

#include <plat/display.h>

//#include <linux/leds.h>
#include <linux/backlight.h>
#include  <../../../../arch/arm/mach-omap2/mux.h>
#include <linux/i2c/twl.h>



#define LCD_XRES		480
#define LCD_YRES		800

static int current_panel = -1;	// 0:sony, 1:Hitachi(20mA) , 2:Hydis, 3:SMD, 4:Sony(a-Si), 5:Hitachi(17mA)
static int lcd_enabled = 0;
static int is_nt35510_spi_shutdown = 0;

// default setting : sony panel. 
static u16 LCD_HBP =	10;//20; 
static u16 LCD_HFP =	10; 
static u16 LCD_HSW =	10; 
static u16 LCD_VBP =	9;// 10;//8; 
static u16 LCD_VFP =	4;// 14;//6; 
static u16 LCD_VSW =	2; 

#define LCD_PIXCLOCK_MAX	        24000 // 26000


#define GPIO_LEVEL_LOW   0
#define GPIO_LEVEL_HIGH  1

#define POWER_OFF	0	// set in lcd_poweroff function.
#define POWER_ON	1	// set in lcd_poweron function

static struct spi_device *nt35510lcd_spi;
    

static atomic_t lcd_power_state = ATOMIC_INIT(POWER_ON);	// default is power on because bootloader already turn on LCD.
static atomic_t ldi_power_state = ATOMIC_INIT(POWER_ON);	// ldi power state

int g_lcdlevel = 0x6C;

// ------------------------------------------ // 
//          For Regulator Framework                            //
// ------------------------------------------ // 

struct regulator *vaux3;
struct regulator *vaux4;

#define MAX_NOTIFICATION_HANDLER	10

void nt35510_lcd_poweron(void);
void nt35510_lcd_poweroff(void);
void nt35510_lcd_LDO_on(void);
void nt35510_lcd_LDO_off(void);
void nt35510_ldi_poweron_sony(void);
void nt35510_ldi_poweroff_sony(void);
void nt35510_ldi_poweron_hitachi(void);
void nt35510_ldi_poweroff_hitachi(void);


// paramter : POWER_ON or POWER_OFF
typedef void (*notification_handler)(const int);
typedef struct
{
	int  state;
	spinlock_t vib_lock;
}timer_state_t;
timer_state_t timer_state;


notification_handler power_state_change_handler[MAX_NOTIFICATION_HANDLER];

int nt35510_add_power_state_monitor(notification_handler handler);
void nt35510_remove_power_state_monitor(notification_handler handler);

EXPORT_SYMBOL(nt35510_add_power_state_monitor);
EXPORT_SYMBOL(nt35510_remove_power_state_monitor);

static void nt35510_notify_power_state_changed(void);
static void aat1402_set_brightness(void);
//extern int omap34xx_pad_set_configs(struct pin_config *pin_configs, int n);
extern int omap34xx_pad_set_config_lcd(u16,u16);

#define PM_RECEIVER                     TWL4030_MODULE_PM_RECEIVER

#define ENABLE_VPLL2_DEDICATED          0x05
#define ENABLE_VPLL2_DEV_GRP            0x20
#define TWL4030_VPLL2_DEV_GRP           0x33
#define TWL4030_VPLL2_DEDICATED       	0x36


static struct pin_config  omap34xx_lcd_pins[] = {
/*
 *		Name, reg-offset,
 *		mux-mode | [active-mode | off-mode]
 */
 
	// MCSPI1 AND MCSPI2 PIN CONFIGURATION
	// FOR ALL SPI, SPI_CS0 IS I/O AND OTHER SPI CS ARE PURE OUT.
	// CLK AND SIMO/SOMI LINES ARE I/O.
/*
	// 206 (AB3, L, MCSPI1_CLK, DISPLAY_CLK, O)
	MUX_CFG_34XX("AB3_MCSPI1_CLK", 0x01C8,  OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT_PULLUP)
	// 207 (AB4, L, MCSPI1_SIMO, DISLPAY_SI, I)
	MUX_CFG_34XX("AB4_MCSPI1_SIMO", 0x01CA,  OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT_PULLDOWN)
	// 208 (AA4, L, MCSPI1_SOMI, DISLPAY_SO, O)
	MUX_CFG_34XX("AA4_MCSPI1_SOMI", 0x01CC,  OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT_PULLDOWN)
	// 209 (AC2, H, MCSPI1_CS0, DISPLAY_CS, O)
	MUX_CFG_34XX("AC2_MCSPI1_CS0", 0x01CE,  OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_INPUT_PULLUP)
	*/
	 //omap_mux_init_signal("gpio_163", OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	// omap_mux_init_signal("MCSPI1_SIMO", OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN);
	// omap_mux_init_signal("MCSPI1_SOMI", OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN);
	// 209 (AC2, H, MCSPI1_CS0, DISPLAY_CS, O)
	//omap_mux_init_signal("sys_nirq",OMAP_WAKEUP_EN | OMAP_PIN_INPUT_PULLUP);
	//omap_mux_init_signal("MCSPI1_CS0", OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	 
	
	
};

static struct pin_config  omap34xx_lcd_off_pins[] = {
/*
 *		Name, reg-offset,
 *		mux-mode | [active-mode | off-mode]
 */
 
	// MCSPI1 AND MCSPI2 PIN CONFIGURATION
	// FOR ALL SPI, SPI_CS0 IS I/O AND OTHER SPI CS ARE PURE OUT.
	// CLK AND SIMO/SOMI LINES ARE I/O.

	// 206 (AB3, L, MCSPI1_CLK, DISPLAY_CLK, O)
	//MUX_CFG_34XX("AB3_MCSPI1_CLK", 0x01C8,  OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN)
	// 207 (AB4, L, MCSPI1_SIMO, DISLPAY_SI, I)
	//MUX_CFG_34XX("AB4_MCSPI1_SIMO", 0x01CA,  OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN)
	// 208 (AA4, L, MCSPI1_SOMI, DISLPAY_SO, O)
	//MUX_CFG_34XX("AA4_MCSPI1_SOMI", 0x01CC,  OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN)
	// 209 (AC2, H, MCSPI1_CS0, DISPLAY_CS, O)
	//MUX_CFG_34XX("AC2_MCSPI1_CS0", 0x01CE,  OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN)
	
};

/*  Manual
 * defines HFB, HSW, HBP, VFP, VSW, VBP as shown below
 */

static struct omap_video_timings panel_timings = {0,};
#if 0 
= {
	/* 800 x 480 @ 60 Hz  Reduced blanking VESA CVT 0.31M3-R */
	.x_res          = LCD_XRES,
	.y_res          = LCD_YRES,
	.pixel_clock    = LCD_PIXCLOCK_MAX,
	.hfp            = LCD_HFP,
	.hsw            = LCD_HSW,
	.hbp            = LCD_HBP,
	.vfp            = LCD_VFP,
	.vsw            = LCD_VSW,
	.vbp            = LCD_VBP,
};
#endif
int nt35510_add_power_state_monitor(notification_handler handler)
{
	int index = 0;
	if(handler == NULL)
	{
		printk(KERN_ERR "[LCD][%s] param is null\n", __func__);
		return -EINVAL;
	}

	for(; index < MAX_NOTIFICATION_HANDLER; index++)
	{
		if(power_state_change_handler[index] == NULL)
		{
			power_state_change_handler[index] = handler;
			return 0;
		}
	}

	// there is no space this time
	printk(KERN_INFO "[LCD][%s] No spcae\n", __func__);

	return -ENOMEM;
}

void nt35510_remove_power_state_monitor(notification_handler handler)
{
	int index = 0;
	if(handler == NULL)
	{
		printk(KERN_ERR "[LCD][%s] param is null\n", __func__);
		return;
	}
	
	for(; index < MAX_NOTIFICATION_HANDLER; index++)
	{
		if(power_state_change_handler[index] == handler)
		{
			power_state_change_handler[index] = NULL;
		}
	}
}
	
static void nt35510_notify_power_state_changed(void)
{
	int index = 0;
	for(; index < MAX_NOTIFICATION_HANDLER; index++)
	{
		if(power_state_change_handler[index] != NULL)
		{
			power_state_change_handler[index](atomic_read(&lcd_power_state));
		}
	}

}

static __init int setup_current_panel(char *opt)
{
	current_panel = (u32)memparse(opt, &opt);
	return 0;
}
__setup("androidboot.current_panel=", setup_current_panel);

static int nt35510_panel_probe(struct omap_dss_device *dssdev)
{
	int lcd_id1, lcd_id2;
	printk(KERN_INFO " **** nt35510_panel_probe.\n");
		
	vaux4 = regulator_get( &dssdev->dev, "vaux4" );
	if( IS_ERR( vaux4 ) )
		printk( "Fail to register vaux4 using regulator framework!\n" );	

	vaux3 = regulator_get( &dssdev->dev, "vaux3" );
	if( IS_ERR( vaux3 ) )
		printk( "Fail to register vaux3 using regulator framework!\n" );	

	nt35510_lcd_LDO_on();

	//MLCD pin set to OUTPUT.
	if (gpio_request(OMAP_GPIO_MLCD_RST, "MLCD_RST") < 0) {
		printk(KERN_ERR "\n FAILED TO REQUEST GPIO %d \n", OMAP_GPIO_MLCD_RST);
		return;
	}
	gpio_direction_output(OMAP_GPIO_MLCD_RST, 1);
/*
	lcd_id1=gpio_get_value(OMAP_GPIO_LCD_ID1);
	lcd_id2=gpio_get_value(OMAP_GPIO_LCD_ID2);

	if(lcd_id1==0 && lcd_id2==1)
		current_panel=0;	// Sony
	else if(lcd_id1==1 && lcd_id2==0)
		current_panel=1;	// Hitachi
	else 
		current_panel=2;	// Hydis
*/
	printk("[LCD] %s() : current_panel=%d(0:sony, 1:Hitachi(20mA) , 2:Hydis, 3:SMD, 4:Sony(a-Si)), 5:Hitachi(17mA)\n",
		__func__, current_panel);

	if(current_panel == 1 || current_panel==5) // if hitachi 20mA, 17mA
	{
		LCD_HBP=	2; 
		LCD_HFP=	100; 
		LCD_HSW=	2;	
		LCD_VBP=	10; 	
		LCD_VFP=	8; 
		LCD_VSW=	2; 	
	}
	else if(current_panel == 3) // if SMD
	{
		LCD_HBP=	40;
		LCD_HFP =	10;
		LCD_HSW =	4;
		LCD_VBP =	7;
		LCD_VFP = 	6;
		LCD_VSW =	1;
	}
	else if(current_panel == 4) // if Sony(a-Si)
	{
		LCD_HBP=	14;
		LCD_HFP =	14;
		LCD_HSW =	10;
		LCD_VBP =	8;
		LCD_VFP = 	8;
		LCD_VSW =	2;
	}

	/* 800 x 480 @ 60 Hz  Reduced blanking VESA CVT 0.31M3-R */
	panel_timings.x_res          = LCD_XRES,
	panel_timings.y_res          = LCD_YRES,
	panel_timings.pixel_clock    = LCD_PIXCLOCK_MAX,
	panel_timings.hfp            = LCD_HFP,
	panel_timings.hsw            = LCD_HSW,
	panel_timings.hbp            = LCD_HBP,
	panel_timings.vfp            = LCD_VFP,
	panel_timings.vsw            = LCD_VSW,
	panel_timings.vbp            = LCD_VBP;

	if(current_panel==1 || current_panel==4 || current_panel==5) // Hitachi(20mA) || Sony(a-Si) || Hitachi(17mA)
	{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |  OMAP_DSS_LCD_IPC |
						OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_ONOFF ;
	}

	else // if(current_panel==0 || current_panel==2 || current_panel==3) // Sony || Hydis || SMD
	{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IPC |
						OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_IEO;
	}

	//dssdev->panel.recommended_bpp= 32;  /* 35 kernel  recommended_bpp field is removed */
	dssdev->panel.acb = 0;
	dssdev->panel.timings = panel_timings;
	
	return 0;
}

static void nt35510_panel_remove(struct omap_dss_device *dssdev)
{
	regulator_put( vaux4 );
	regulator_put( vaux3 );
}

static int nt35510_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;
	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* Delay recommended by panel DATASHEET */
	mdelay(4);
	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
        }
        dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		
	if(lcd_enabled ==1)
		lcd_enabled =0;
	else 
		nt35510_lcd_poweron();

	return r;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void nt35510_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;
	if(is_nt35510_spi_shutdown == 1)
	{
		printk("[%s] skip omapdss_dpi_display_disable..\n", __func__); 
		return;
	}
	nt35510_lcd_poweroff();
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	mdelay(4);

	omapdss_dpi_display_disable(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int nt35510_panel_suspend(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " **** nt35510_panel_suspend\n");
	spi_setup(nt35510lcd_spi);

	gpio_set_value(OMAP_GPIO_LCD_EN_SET, GPIO_LEVEL_LOW);
	mdelay(1);

    #if 0
	nt35510_lcd_poweroff();
    #else
//	nt35510_ldi_standby();
	nt35510_panel_disable(dssdev);
    #endif
   
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
    return 0;
}

static int nt35510_panel_resume(struct omap_dss_device *dssdev)
{
	printk(KERN_INFO " **** nt35510_panel_resume\n");

	spi_setup(nt35510lcd_spi);
    
//		msleep(150);
    
    #if 0
	nt35510_lcd_poweron();
    #else
//	nt35510_ldi_wakeup();
	nt35510_panel_enable(dssdev);
    #endif

	gpio_set_value(OMAP_GPIO_LCD_EN_SET, GPIO_LEVEL_LOW);
	mdelay(1);
	gpio_set_value(OMAP_GPIO_LCD_EN_SET, GPIO_LEVEL_HIGH);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return 0;
}

static struct omap_dss_driver nt35510_driver = {
	.probe          = nt35510_panel_probe,
	.remove         = nt35510_panel_remove,

	.enable         = nt35510_panel_enable,
	.disable        = nt35510_panel_disable,
	.suspend        = nt35510_panel_suspend,
	.resume         = nt35510_panel_resume,

	.driver		= {
		.name	= "nt35510_panel",
		.owner 	= THIS_MODULE,
	},
};

static void spi1writeindex(u8 index)
{
	volatile unsigned short cmd = 0;
	cmd= 0x0000|index;

	spi_write(nt35510lcd_spi,(unsigned char*)&cmd,2);

	udelay(100);
	udelay(100);
}

static void spi1writedata(u8 data)
{
	volatile unsigned short datas = 0;
	datas= 0x0100|data;
	spi_write(nt35510lcd_spi,(unsigned char*)&datas,2);

	udelay(100);
	udelay(100);
}


static void spi1write(u8 index, u8 data)
{
	volatile unsigned short cmd = 0;
	volatile unsigned short datas=0;

	cmd = 0x0000 | index;
	datas = 0x0100 | data;
	
	spi_write(nt35510lcd_spi,(unsigned char*)&cmd,2);
	udelay(100);
	spi_write(nt35510lcd_spi,(unsigned char *)&datas,2);
	udelay(100);
	udelay(100);
}

void nt35510_ldi_poweron_hitachi(void)
{
	printk("[LCD] %s() + \n", __func__);

	spi1writeindex(0x36);
	spi1writedata(0x00); 
	
	spi1writeindex(0x3A);
	spi1writedata(0x77);

	spi1writeindex(0x11);

	msleep(150);

	
	spi1writeindex(0xB0);
	spi1writedata(0x04);

	spi1writeindex(0xC1);
	spi1writedata(0x42);
	spi1writedata(0x31);
	spi1writedata(0x04);
	
	spi1writeindex(0xB0);
	spi1writedata(0x03);

	aat1402_set_brightness();

	spi1writeindex(0x29);	
	
	atomic_set(&ldi_power_state, POWER_ON);
	printk("[LCD] %s() -\n", __func__);
}


void nt35510_ldi_poweroff_hitachi(void)
{
	printk(" **** %s\n", __func__);
	
	// SLEEP IN
	spi1writeindex(0x10);	// SEQ_SLEEP IN_SET

	//Wait 120ms
	msleep(150);	

	// Display Off
//	spi1writeindex(0x28);	// SEQ_DISPOFF_SET
	spi1writeindex(0xB0);
	spi1writedata(0x04);
	spi1writeindex(0xB1);
	spi1writedata(0x01); 
	msleep(2);
	
	atomic_set(&ldi_power_state, POWER_OFF);
}

void nt35510_ldi_poweron_sony(void)
{
	printk("[LCD] %s() + \n", __func__);
	msleep(145);

	spi1writeindex(0x3A);
	spi1writedata(0x77);
	
	spi1writeindex(0x3B);
	spi1writedata(0x07);	
	spi1writedata(0x0A);
	spi1writedata(0x0E);
	spi1writedata(0x0A);
	spi1writedata(0x0A);

	spi1writeindex(0x2A);
	spi1writedata(0x00);
	spi1writedata(0x00);
	spi1writedata(0x01);
	spi1writedata(0xDF);
	
	spi1writeindex(0x2B);
	spi1writedata(0x00);
	spi1writedata(0x00);
	spi1writedata(0x03);
	spi1writedata(0x1F);

	spi1writeindex(0x36);
	spi1writedata(0xD4);

 	msleep(25);

	spi1writeindex(0x11);

	msleep(200);

	spi1writeindex(0x51);
	spi1writedata(0x0); // default brightness : 0
//	spi1writedata(0x6C); // default brightness : 108
	aat1402_set_brightness();

	spi1writeindex(0x55);
	spi1writedata(0x01); // CABC Off, 01:User interface mode, 02:still image mode, 03:moving image mode

	spi1writeindex(0x5E);
	spi1writedata(0x00); // example value. *3)

	spi1writeindex(0x53);
	spi1writedata(0x2C);

//	msleep(200);

	spi1writeindex(0x29);	

	atomic_set(&ldi_power_state, POWER_ON);
	printk("[LCD] %s() -\n", __func__);
}


void nt35510_ldi_poweroff_sony(void)
{
	printk(" **** %s\n", __func__);

	// Display Off
	spi1writeindex(0x28);	// SEQ_DISPOFF_SET

	msleep(25);	

	// SLEEP IN
	spi1writeindex(0x10);	// SEQ_SLEEP IN_SET

	//Wait 120ms
	msleep(150);	
	
	atomic_set(&ldi_power_state, POWER_OFF);
}

void nt35510_ldi_poweron_sony_a_si(void)
{
	printk("[LCD] %s() + \n", __func__);
	msleep(145);
	spi1writeindex(0x3A);
	spi1writedata(0x77);
	
	spi1writeindex(0x2A);
	spi1writedata(0x00);
	spi1writedata(0x00);
	spi1writedata(0x01);
	spi1writedata(0xDF);

	spi1writeindex(0x2B);
	spi1writedata(0x00);
	spi1writedata(0x00);
	spi1writedata(0x03);
	spi1writedata(0x1F);

	spi1writeindex(0x36); // SET_ADDRESS_MODE
	spi1writedata(0x00);

	spi1writeindex(0xF0); 
	spi1writedata(0x5A);
	spi1writedata(0x5A); 
	
	spi1writeindex(0xF1); 
	spi1writedata(0x5A);
	spi1writedata(0x5A);

	spi1writeindex(0xB2);
	spi1writedata(0x22);
	spi1writedata(0x31);
	
	spi1writeindex(0xBA);
	spi1writedata(0x07);
	spi1writedata(0xAA);
	spi1writedata(0x10);
	spi1writedata(0x10);
	spi1writedata(0x0A);
	spi1writedata(0x18);
	spi1writedata(0x1F);
	spi1writedata(0x1F);

	spi1writeindex(0xBB);
	spi1writedata(0x07);
		spi1writedata(0xAA);
	spi1writedata(0x10);
	spi1writedata(0x10);
	spi1writedata(0x0A);
	spi1writedata(0x1C);
	spi1writedata(0x1F);
	spi1writedata(0x1F);
	
	spi1writeindex(0xF2); // SET_DISPLAY_CONTROL
	spi1writedata(0x00);
	spi1writedata(0x08); // NVBP
	spi1writedata(0x08); // NVFB
	spi1writedata(0x08); // BVBP
	spi1writedata(0x08); // BVFP
	spi1writedata(0x14); // NHBP
	spi1writedata(0x14); // NHFP
	spi1writedata(0x14); // BHBP
	spi1writedata(0x14); // BHFP
	spi1writedata(0x08); // RGB_NVBP
	spi1writedata(0x08); // RGB_NVFP
	spi1writedata(0x08); // GRB_BVBP
	spi1writedata(0x08); // RGB_BVFP
	spi1writedata(0x31); // VPL[7]:Low, HPL[6]:Low, DPL[5]:falling, EPL[4]:High  // spi1writedata(0x11);
	spi1writedata(0x00);
	spi1writedata(0x00);
	
	spi1writeindex(0xF4);
	spi1writedata(0x01);
	spi1writedata(0x15);
	spi1writedata(0x01);
	spi1writedata(0x04);
	spi1writedata(0x01);
	spi1writedata(0x23);
	spi1writedata(0x66);
	spi1writedata(0x00);
	spi1writedata(0x12);
	spi1writedata(0x12);
	spi1writedata(0x0A);
	
	spi1writeindex(0xD0);
	spi1writedata(0x07);
	
	spi1writeindex(0xF6);
	spi1writedata(0x08);
	spi1writedata(0x00);
	spi1writedata(0x00);
	spi1writedata(0x05);
	spi1writedata(0x01);
	spi1writedata(0x14);
	spi1writedata(0x14);
	spi1writedata(0x04);
	spi1writedata(0x04);
	spi1writedata(0x06);
	spi1writedata(0x06);
	
	spi1writeindex(0xF7);
	spi1writedata(0x01);
	spi1writedata(0xD8);
	spi1writedata(0x50);
	spi1writedata(0xFA);
	spi1writedata(0x4E);
	spi1writedata(0x08);
	spi1writedata(0x08);
	spi1writedata(0x01);
		spi1writedata(0x01);
	spi1writedata(0x00);
		
		spi1writeindex(0xFA);	
	spi1writedata(0x02);
	spi1writedata(0x52);
	spi1writedata(0x52);
	spi1writedata(0x44);
	spi1writedata(0x38);
	spi1writedata(0x2D);
	spi1writedata(0x2D);
	spi1writedata(0x18);
	spi1writedata(0x18);
	spi1writedata(0x1E);
	spi1writedata(0x1E);
	spi1writedata(0x1B);
	spi1writedata(0x1D);
	spi1writedata(0x1B);
	spi1writedata(0x13);
	spi1writedata(0x04);
	spi1writedata(0x02);
	spi1writedata(0x52);
	spi1writedata(0x52);
	spi1writedata(0x43);
	spi1writedata(0x39);
	spi1writedata(0x2E);
	spi1writedata(0x2E);
	spi1writedata(0x18);
	spi1writedata(0x18);
	spi1writedata(0x1E);
	spi1writedata(0x1D);
	spi1writedata(0x1A);
	spi1writedata(0x1C);
	spi1writedata(0x1A);
	spi1writedata(0x12);
	spi1writedata(0x04);
	spi1writedata(0x02);
	spi1writedata(0x52);
	spi1writedata(0x52);
	spi1writedata(0x43);
	spi1writedata(0x38);
	spi1writedata(0x2C);
	spi1writedata(0x2D);
	spi1writedata(0x17);
	spi1writedata(0x17);
	spi1writedata(0x1D);
	spi1writedata(0x1C);
	spi1writedata(0x18);
	spi1writedata(0x1A);
	spi1writedata(0x18);
	spi1writedata(0x10);
	spi1writedata(0x04);
	
	spi1writeindex(0xFB);
	spi1writedata(0x02);
	spi1writedata(0x52);
	spi1writedata(0x52);
	spi1writedata(0x44);
	spi1writedata(0x38);
	spi1writedata(0x2D);
	spi1writedata(0x2D);
	spi1writedata(0x18);
	spi1writedata(0x18);
	spi1writedata(0x1E);
	spi1writedata(0x1E);
	spi1writedata(0x1B);
	spi1writedata(0x1D);
	spi1writedata(0x1B);
	spi1writedata(0x13);
	spi1writedata(0x04);
	spi1writedata(0x02);
	spi1writedata(0x52);
	spi1writedata(0x52);
	spi1writedata(0x43);
	spi1writedata(0x39);
	spi1writedata(0x2E);
	spi1writedata(0x2E);
	spi1writedata(0x18);
	spi1writedata(0x18);
	spi1writedata(0x1E);
	spi1writedata(0x1D);
	spi1writedata(0x1A);
	spi1writedata(0x1C);
	spi1writedata(0x1A);
	spi1writedata(0x12);
	spi1writedata(0x04);
	spi1writedata(0x02);
	spi1writedata(0x52);
	spi1writedata(0x52);
	spi1writedata(0x43);
	spi1writedata(0x38);
	spi1writedata(0x2C);
	spi1writedata(0x2D);
	spi1writedata(0x17);
	spi1writedata(0x17);
	spi1writedata(0x1D);
	spi1writedata(0x1C);
	spi1writedata(0x18);
	spi1writedata(0x1A);
	spi1writedata(0x18);
	spi1writedata(0x10);
	spi1writedata(0x04);

	msleep(25); // 25ms
	
	spi1writeindex(0x11);

	msleep(150); // 150ms

	spi1writeindex(0x51);
	spi1writedata(0x0); // default brightness : 0
//	spi1writedata(0x6C); // default brightness : 108
	aat1402_set_brightness();

	spi1writeindex(0x55);
	spi1writedata(0x01); // CABC Off, 01:User interface mode, 02:still image mode, 03:moving image mode

	spi1writeindex(0x5E);
	spi1writedata(0x00); // example value. *3)

	spi1writeindex(0x53);
	spi1writedata(0x2C);

	spi1writeindex(0xC0);	
	spi1writedata(0x80);
		spi1writedata(0x80);
	spi1writedata(0x30);
	
	spi1writeindex(0xC1);
	spi1writedata(0x11);

	spi1writeindex(0xC2);
	spi1writedata(0x08);
		spi1writedata(0x00);
		spi1writedata(0x00);		
	spi1writedata(0x03);
	spi1writedata(0x1F);
		spi1writedata(0x00);
		spi1writedata(0x00);
	spi1writedata(0x01);
	spi1writedata(0xDF);
	
	spi1writeindex(0xC3);
	spi1writedata(0x01);
		spi1writedata(0x00);
	spi1writedata(0x28);

	msleep(200); // 200ms
	
	spi1writeindex(0xF0);
	spi1writedata(0xA5);
	spi1writedata(0xA5);
	
	spi1writeindex(0xF1);
	spi1writedata(0xA5);
	spi1writedata(0xA5);
	
	spi1writeindex(0x29);

	atomic_set(&ldi_power_state, POWER_ON);
	printk("[LCD] %s() -\n", __func__);
}


void nt35510_ldi_poweroff_sony_a_si(void)
{
	printk(" **** %s\n", __func__);
		
	// Display Off
	spi1writeindex(0x28);	// SEQ_DISPOFF_SET

	msleep(50);	

	// SLEEP IN
	spi1writeindex(0x10);	// SEQ_SLEEP IN_SET

	//Wait 120ms
	msleep(150);	
	
	atomic_set(&ldi_power_state, POWER_OFF);
}

void lcd_hydis_gamma1(void)
{
	spi1writedata(0x00);
	spi1writedata(0x37);
	spi1writedata(0x00);
	spi1writedata(0x61);
	spi1writedata(0x00);
	spi1writedata(0x92);
	spi1writedata(0x00);
	spi1writedata(0xB4);
	spi1writedata(0x00);
	spi1writedata(0xCF);
	spi1writedata(0x01);
	spi1writedata(0x06);
	spi1writedata(0x01);
	spi1writedata(0x41);
	spi1writedata(0x01);
	spi1writedata(0x8A);
	spi1writedata(0x01);
	spi1writedata(0xA6);
	spi1writedata(0x01);
	spi1writedata(0xD1);
	spi1writedata(0x02);
	spi1writedata(0x01);
	spi1writedata(0x02);
	spi1writedata(0x3D);
	spi1writedata(0x02);
	spi1writedata(0x77);
	spi1writedata(0x02);
	spi1writedata(0x79);
	spi1writedata(0x02);
	spi1writedata(0xA5);
	spi1writedata(0x02);
	spi1writedata(0xD1);
	spi1writedata(0x02);
	spi1writedata(0xF9);
	spi1writedata(0x03);
	spi1writedata(0x25);
	spi1writedata(0x03);
	spi1writedata(0x43);
	spi1writedata(0x03);
	spi1writedata(0x6E);
	spi1writedata(0x03);
	spi1writedata(0x77);
	spi1writedata(0x03);
	spi1writedata(0x94);
	spi1writedata(0x03);
	spi1writedata(0x9F);
	spi1writedata(0x03);
	spi1writedata(0xAC);
	spi1writedata(0x03);
	spi1writedata(0xBA);
	spi1writedata(0x03);
	spi1writedata(0xF1);
}
void lcd_hydis_gamma2(void)
{
	spi1writedata(0x00);
	spi1writedata(0x37);
	spi1writedata(0x00);
	spi1writedata(0x50);
	spi1writedata(0x00);
	spi1writedata(0x89);
	spi1writedata(0x00);
	spi1writedata(0xA9);
	spi1writedata(0x00);
	spi1writedata(0xC0);
	spi1writedata(0x01);
	spi1writedata(0x06);
	spi1writedata(0x01);
	spi1writedata(0x26);
	spi1writedata(0x01);
	spi1writedata(0x54);
	spi1writedata(0x01);
	spi1writedata(0x79);
	spi1writedata(0x01);
	spi1writedata(0xB8);
	spi1writedata(0x01);
	spi1writedata(0xDF);
	spi1writedata(0x02);
	spi1writedata(0x2F);
	spi1writedata(0x02);
	spi1writedata(0x68);
	spi1writedata(0x02);
	spi1writedata(0x6A);
	spi1writedata(0x02);
	spi1writedata(0xA3);
	spi1writedata(0x02);
	spi1writedata(0xE0);
	spi1writedata(0x02);
	spi1writedata(0xF9);
	spi1writedata(0x03);
	spi1writedata(0x25);
	spi1writedata(0x03);
	spi1writedata(0x43);
	spi1writedata(0x03);
	spi1writedata(0x6E);
	spi1writedata(0x03);
	spi1writedata(0x77);
	spi1writedata(0x03);
	spi1writedata(0x94);
	spi1writedata(0x03);
	spi1writedata(0x9F);
	spi1writedata(0x03);
	spi1writedata(0xAC);
	spi1writedata(0x03);
	spi1writedata(0xBA);
	spi1writedata(0x03);
	spi1writedata(0xF1);

}

void nt35510_ldi_poweron_hydis(void)
{
	printk("[LCD] %s() + \n", __func__);
		
// [[ User Set
	/* Test Commands */
	spi1writeindex(0xFF);	
	spi1writedata(0xAA);
	spi1writedata(0x55);
	spi1writedata(0x25);
	spi1writedata(0x01);	

	/* Test Commands */
	spi1writeindex(0xF3);
	spi1writedata(0x00);
	spi1writedata(0x32);
	spi1writedata(0x00);
	spi1writedata(0x38);		
	spi1writedata(0x31);
	spi1writedata(0x08);
	spi1writedata(0x11);
	spi1writedata(0x00);

	/* Manufacture Command Set Selection */
	spi1writeindex(0xF0);
	spi1writedata(0x55);
	spi1writedata(0xAA);
	spi1writedata(0x52);
	spi1writedata(0x08);		
	spi1writedata(0x00);	

	/* SEC Setting */	
	spi1writeindex(0xB0);		
	spi1writedata(0x04); // 0x00  //DE Mode/Rising Edge for PCLK		 
	spi1writedata(0x0A); //VBP 10
	spi1writedata(0x0E); //VFP 14
	spi1writedata(0x09); //HBP 10
	spi1writedata(0x04); //HFP 10

	/*  */	
	spi1writeindex(0xB1);		
	spi1writedata(0xCC); // Backward for Gate
	spi1writedata(0x04); // "04 : Backward / 00 : Forward

	/* data */
	spi1writeindex(0x36);	
	spi1writedata(0x02); // 00 : Forward / 02 : Backward

	/* Display Clock in RGB Interface */
	spi1writeindex(0xB3);	
	spi1writedata(0x00); 

	/* Source Output Data Hold Time */
	spi1writeindex(0xB6);	
	spi1writedata(0x03); 

	/* GATE EQ */
	spi1writeindex(0xB7);	
	spi1writedata(0x70); 
	spi1writedata(0x70);

	/* SOURCE EQ */
	spi1writeindex(0xB8);
	spi1writedata(0x00);		
	spi1writedata(0x06);		
	spi1writedata(0x06);
	spi1writedata(0x06);

	/* Inversion Type */
	spi1writeindex(0xBC);
	spi1writedata(0x00); 
	spi1writedata(0x00);
	spi1writedata(0x00);

	/* Display Timing Control */
	spi1writeindex(0xBD);
	spi1writedata(0x01);			
	spi1writedata(0x84);		
	spi1writedata(0x06);
	spi1writedata(0x50);
	spi1writedata(0x00);

	/* aRD(Gateless) Setting */
	spi1writeindex(0xCC);
	spi1writedata(0x03);			
	spi1writedata(0x01);		
	spi1writedata(0x06);
// ]] User Set

// [[ Power Set
	spi1writeindex(0xF0);
	spi1writedata(0x55);
	spi1writedata(0xAA);
	spi1writedata(0x52);
	spi1writedata(0x08);
	spi1writedata(0x01);

	spi1writeindex(0xB0);
	spi1writedata(0x05);
	spi1writedata(0x05);
	spi1writedata(0x05);

	spi1writeindex(0xB1);
	spi1writedata(0x05);
	spi1writedata(0x05);
	spi1writedata(0x05);

	spi1writeindex(0xB2);
	spi1writedata(0x03);
	spi1writedata(0x03);
	spi1writedata(0x03);

	spi1writeindex(0xB8);
	spi1writedata(0x25);
	spi1writedata(0x25);
	spi1writedata(0x25);

	spi1writeindex(0xB3);
	spi1writedata(0x0B);
	spi1writedata(0x0B);
	spi1writedata(0x0B);

	spi1writeindex(0xB9);
	spi1writedata(0x34);
	spi1writedata(0x34);
	spi1writedata(0x34);

	spi1writeindex(0xBF);
	spi1writedata(0x01);

	spi1writeindex(0xB5);
	spi1writedata(0x08);
	spi1writedata(0x08);
	spi1writedata(0x08);

	spi1writeindex(0xBA);
	spi1writedata(0x14);
	spi1writedata(0x14);
	spi1writedata(0x14);

	spi1writeindex(0xB4);
	spi1writedata(0x2E);
	spi1writedata(0x2E);
	spi1writedata(0x2E);

	spi1writeindex(0xBC);
	spi1writedata(0x00);
	spi1writedata(0x50);
	spi1writedata(0x00);

	spi1writeindex(0xBD);
	spi1writedata(0x00);
	spi1writedata(0x60);
	spi1writedata(0x00);

	spi1writeindex(0xBE);
	spi1writedata(0x00);
	spi1writedata(0x40);

	spi1writeindex(0xCE);
	spi1writedata(0x00);
	spi1writedata(0x00);
	spi1writedata(0x00);
	spi1writedata(0x00);
	spi1writedata(0x00);
	spi1writedata(0x00);
	spi1writedata(0x00);

// ]] P S

// [[ Gamma Control
	spi1writeindex(0xD0);
	spi1writedata(0x09);
	spi1writedata(0x14);
	spi1writedata(0x07);
	spi1writedata(0x0D);

	spi1writeindex(0xD1);
	lcd_hydis_gamma1();
	spi1writeindex(0xD2);
	lcd_hydis_gamma1();
	spi1writeindex(0xD3);
	lcd_hydis_gamma1();
	spi1writeindex(0xD4);
	lcd_hydis_gamma2();	
	spi1writeindex(0xD5);
	lcd_hydis_gamma2();
	spi1writeindex(0xD6);
	lcd_hydis_gamma2();
// ]] Gamma Control
		
// [[
/* Manufacture Command Set Selection */
		spi1writeindex(0xF0);		
		spi1writedata(0x55);
		spi1writedata(0xAA);
		spi1writedata(0x52);
		spi1writedata(0x08);
		spi1writedata(0x00); 
		
// ]] 
	spi1writeindex(0x11);	// SLPOUT

	spi1writeindex(0x51);
	spi1writedata(0x0); // default brightness : 0
//	spi1writedata(0x6C); // default brightness : 108
	aat1402_set_brightness();

	spi1writeindex(0x53);
	spi1writedata(0x2C);

	msleep(120);	// 120ms
	spi1writeindex(0x29);	// DISPON

	atomic_set(&ldi_power_state, POWER_ON);	
	printk("[LCD] %s() -- \n", __func__);
}

void nt35510_ldi_poweroff_hydis(void)
{
	printk(" **** %s\n", __func__);
		
	// Display Off
	spi1writeindex(0x28);	// SEQ_DISPOFF_SET

//	if(current_panel==0) // sony
	{
		msleep(25);	
	}

	// SLEEP IN
	spi1writeindex(0x10);	// SEQ_SLEEP IN_SET

	//Wait 120ms
	msleep(150);	
	
	atomic_set(&ldi_power_state, POWER_OFF);
}

void nt35510_ldi_poweron_smd(void)
{
	printk("[LCD] %s() + \n", __func__);

	/* Initializing Sequence */
	spi1writeindex(0x36);
	spi1writedata(0x09); // 0A

	spi1writeindex(0xB0);
		spi1writedata(0x00); 
				
	spi1writeindex(0xC0);
	spi1writedata(0x28);	
	spi1writedata(0x08);
				
	spi1writeindex(0xC1);
		spi1writedata(0x01);		
	spi1writedata(0x30);
	spi1writedata(0x15);
	spi1writedata(0x05);
	spi1writedata(0x22);
		
	spi1writeindex(0xC4);
	spi1writedata(0x10);
		spi1writedata(0x01);			
	spi1writedata(0x00);
		
	spi1writeindex(0xC5);
	spi1writedata(0x06);
	spi1writedata(0x55);
	spi1writedata(0x03);
	spi1writedata(0x07);
	spi1writedata(0x0B);
	spi1writedata(0x33);
		spi1writedata(0x00); 
	spi1writedata(0x01);
		spi1writedata(0x03);

	spi1writeindex(0xC6); // RGB Sync option
	spi1writedata(0x00);

	spi1writeindex(0xC8); // gamma set RED
	spi1writedata(0x00); // R_Posi
	spi1writedata(0x00);
	spi1writedata(0x0F);
	spi1writedata(0x29);
	spi1writedata(0x43);
	spi1writedata(0x4E);
	spi1writedata(0x4F);
	spi1writedata(0x52);
	spi1writedata(0x50);
	spi1writedata(0x56);
	spi1writedata(0x5B);
	spi1writedata(0x5A);
	spi1writedata(0x57);
	spi1writedata(0x51);
	spi1writedata(0x55);
	spi1writedata(0x55);
	spi1writedata(0x5D);
	spi1writedata(0x5F);
	spi1writedata(0x1C);

	spi1writedata(0x00); // R_Nega
	spi1writedata(0x00);
	spi1writedata(0x0F);
	spi1writedata(0x29);
	spi1writedata(0x43);
	spi1writedata(0x4F);
	spi1writedata(0x4E);
	spi1writedata(0x52);
	spi1writedata(0x50);
	spi1writedata(0x56);
	spi1writedata(0x5B);
	spi1writedata(0x5A);
	spi1writedata(0x57);
	spi1writedata(0x51);
	spi1writedata(0x55);
	spi1writedata(0x55);
	spi1writedata(0x5D);
	spi1writedata(0x5F);
	spi1writedata(0x1C);
		
	spi1writeindex(0xC9); // gamma set GREEN
	spi1writedata(0x00); // G_Posi
	spi1writedata(0x00);
	spi1writedata(0x28);
	spi1writedata(0x36);
	spi1writedata(0x48);
	spi1writedata(0x50);
	spi1writedata(0x4E);
	spi1writedata(0x52);
	spi1writedata(0x4E);
	spi1writedata(0x53);
	spi1writedata(0x56);
	spi1writedata(0x55);
	spi1writedata(0x52);
	spi1writedata(0x4B);
	spi1writedata(0x4D);
	spi1writedata(0x4E);
	spi1writedata(0x4F);
	spi1writedata(0x4F);
	spi1writedata(0x0C);
	
	spi1writedata(0x00); // G_Nega
	spi1writedata(0x00);
	spi1writedata(0x28);
	spi1writedata(0x36);
	spi1writedata(0x48);
	spi1writedata(0x50);
	spi1writedata(0x4E);
	spi1writedata(0x52);
	spi1writedata(0x4E);
	spi1writedata(0x53);
	spi1writedata(0x56);
	spi1writedata(0x55);
	spi1writedata(0x52);
	spi1writedata(0x4B);
	spi1writedata(0x4D);
	spi1writedata(0x4E);
	spi1writedata(0x4F);
	spi1writedata(0x4F);
	spi1writedata(0x0C);

	spi1writeindex(0xCA); // gamma set BLUE
	spi1writedata(0x00); // B_Posi
	spi1writedata(0x00);
	spi1writedata(0x26);
	spi1writedata(0x35);
	spi1writedata(0x48);
	spi1writedata(0x52);
	spi1writedata(0x51);
	spi1writedata(0x57);
	spi1writedata(0x54);
	spi1writedata(0x5B);
	spi1writedata(0x60);
	spi1writedata(0x60);
	spi1writedata(0x5E);
	spi1writedata(0x59);
	spi1writedata(0x5E);
	spi1writedata(0x5A);
	spi1writedata(0x5D);
	spi1writedata(0x5C);
	spi1writedata(0x2A);
	
	spi1writedata(0x00); // B_Nega
	spi1writedata(0x00);
	spi1writedata(0x26);
	spi1writedata(0x35);
	spi1writedata(0x48);
	spi1writedata(0x52);
	spi1writedata(0x51);
	spi1writedata(0x57);
	spi1writedata(0x54);
	spi1writedata(0x5B);
	spi1writedata(0x60);
	spi1writedata(0x60);
	spi1writedata(0x5E);
	spi1writedata(0x59);
	spi1writedata(0x5E);
	spi1writedata(0x5A);
	spi1writedata(0x5D);
	spi1writedata(0x5C);
	spi1writedata(0x2A);

	spi1writeindex(0xD1);
	spi1writedata(0x33);
	spi1writedata(0x13);

	spi1writeindex(0xD2);
	spi1writedata(0x11);
		spi1writedata(0x00);
		spi1writedata(0x00);

	spi1writeindex(0xD3);
	spi1writedata(0x50);
	spi1writedata(0x50);

	spi1writeindex(0xD5);
	spi1writedata(0x2F);
	spi1writedata(0x11);
	spi1writedata(0x1E);
	spi1writedata(0x46);

	spi1writeindex(0xD6);
	spi1writedata(0x11);
	spi1writedata(0x0A);
		
////

	// Set PWM	
	spi1writeindex(0xB4);
	spi1writedata(0x0F);
	spi1writedata(0x00);
	spi1writedata(0x50);

	spi1writeindex(0xB5);
	spi1writedata(0x00); 
//		spi1writedata(0x6C); // default brightness : 108
		aat1402_set_brightness();

	spi1writeindex(0xB7);
	spi1writedata(0x24);

	spi1writeindex(0xB8);
	spi1writedata(0x01);
	
////	

	/* Sleep Out */
	spi1writeindex(0x11);
	msleep(20); // 20ms

	/* NVM Load Sequence */
//	spi1writeindex(0xD4);
//	spi1writedata(0x50);
//	spi1writedata(0x53);

	spi1writeindex(0xF8);
	spi1writedata(0x01);
	spi1writedata(0xF5);
	spi1writedata(0xF2);
	spi1writedata(0x71);
	spi1writedata(0x44);	

	spi1writeindex(0xFC);
	spi1writedata(0x00);
	spi1writedata(0x08);
	msleep(150); // 150ms

	/* Display On */	
		spi1writeindex(0x29);	

	atomic_set(&ldi_power_state, POWER_ON);
	printk("[LCD] %s() -- \n", __func__);
}

void nt35510_ldi_poweroff_smd(void)
{
	printk(" **** %s\n", __func__);
		
	//Display Off Command
	spi1writeindex(0x28);

	// Sleep In 
	spi1writeindex(0x10);
	msleep(120); // 120 ms
	
	atomic_set(&ldi_power_state, POWER_OFF);
}

void nt35510_lcd_LDO_on(void)
{
	int ret;
	printk("+++ %s\n", __func__);
//	twl_i2c_read_regdump();
#if 1	
	ret = regulator_enable( vaux3 ); //VAUX3 - 1.8V
	if ( ret )
		printk("Regulator vaux3 error!!\n");
#else
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00, 0x1F);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x1, 0x22);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0xe0, 0x1F);
#endif

	mdelay(1);
	ret = regulator_enable( vaux4 ); //VAUX4 - 2.8V
	if ( ret )
		printk("Regulator vaux4 error!!\n");
	
	mdelay(1);
	
	if(current_panel == 3) // if SMD
		msleep(50);
	
	printk("--- %s\n", __func__);
}

void nt35510_lcd_LDO_off(void)
{
	int ret;
	printk("+++ %s\n", __func__);

	// Reset Release (reset = L)
	gpio_set_value(OMAP_GPIO_MLCD_RST, GPIO_LEVEL_LOW); 
	mdelay(10);

	// VCI 2.8V OFF
	ret = regulator_disable( vaux4 );
	if ( ret )
		printk("Regulator vaux4 error!!\n");
	mdelay(1);

	// VDD3 1.8V OFF
#if 1
	ret = regulator_disable( vaux3 );
	if ( ret )
		printk("Regulator vaux3 error!!\n");
#else
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00, 0x1F);
#endif	

	printk("--- %s\n", __func__);
}

void nt35510_lcd_poweroff(void)
{
	if(current_panel==0) // Sony
		nt35510_ldi_poweroff_sony();
	else if(current_panel==1 || current_panel==5) // Hitachi 20mA, 17mA
		nt35510_ldi_poweroff_hitachi();
	else if(current_panel==2) // Hydis
		nt35510_ldi_poweroff_hydis();
	else if(current_panel==3) // SMD
		nt35510_ldi_poweroff_smd();
	else if(current_panel==4) // Sony(a-Si)
		nt35510_ldi_poweroff_sony_a_si();

	// turn OFF VCI (2.8V)
	// turn OFF VDD3 (1.8V)
	nt35510_lcd_LDO_off();

#if 1             
		omap_mux_init_signal("mcspi1_clk", OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mcspi1_simo",  OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mcspi1_somi", OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mcspi1_cs0", OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN);
#endif		
		
	return;	
}

void nt35510_lcd_poweron(void)
{

       u8 read=0;
#if 1	
	omap_mux_init_signal("mcspi1_clk",OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcspi1_simo",OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("mcspi1_somi", OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_signal("mcspi1_cs0", OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
#endif	

	nt35510_lcd_LDO_on();

	// Activate Reset
	gpio_set_value(OMAP_GPIO_MLCD_RST, GPIO_LEVEL_LOW);	

	mdelay(1);
	gpio_set_value(OMAP_GPIO_MLCD_RST, GPIO_LEVEL_HIGH);
	mdelay(5);
	
	//MLCD pin set to InputPulldown.
	omap_ctrl_writew(0x010C, 0x1c6);

	if(current_panel==0)	// Sony
		nt35510_ldi_poweron_sony();
	else if(current_panel==1 || current_panel==5) // Hitachi 20mA, 17mA
		nt35510_ldi_poweron_hitachi();
	else if(current_panel==2) // Hydis
		nt35510_ldi_poweron_hydis();
	else if(current_panel==3) // SMD
		nt35510_ldi_poweron_smd();
	else if(current_panel==4) // Sony(a-Si)
		nt35510_ldi_poweron_sony_a_si();

	aat1402_set_brightness();
       
	return;
}

static ssize_t nt35510_sysfs_store_lcd_power(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	int rc;
	int lcd_enable;

	dev_info(dev, "nt35510_sysfs_store_lcd_power\n");

	rc = strict_strtoul(buf, 0, (unsigned long *)&lcd_enable);
	if (rc < 0)
		return rc;

	if(lcd_enable) {
		nt35510_lcd_poweron();
	}
	else {
		nt35510_lcd_poweroff();
	}

	return len;
}

static DEVICE_ATTR(lcd_power, 0664,
		NULL, nt35510_sysfs_store_lcd_power);

// [[ backlight control 
static int current_intensity = 108;	// DEFAULT BRIGHTNESS
static DEFINE_SPINLOCK(aat1402_bl_lock);

static void aat1402_set_brightness(void)
{
	printk(KERN_DEBUG" *** aat1402_set_brightness : %d\n", current_intensity);
	//spin_lock_irqsave(&aat1402_bl_lock, flags);
	//spin_lock(&aat1402_bl_lock);

	if(current_panel==1 || current_panel==5)	// if Hitachi 20mA, 17mA
	{
		int orig_intensity = current_intensity;

		if(current_intensity>=108)
			current_intensity = ((current_intensity-108)*(216-91))/(255-108) + 91;
		else if(current_intensity>=34)
			current_intensity = ((current_intensity-34)*(91-29))/(108-34) + 29;
		else if(current_intensity>=20)
			current_intensity = ((current_intensity-20)*(29-17))/(34-20) + 17;

		printk(KERN_DEBUG" HITACHI PANEL(%d)! orig_intensity=%d, current_intensity=%d\n", current_panel, orig_intensity, current_intensity);
		
		spi1writeindex(0xB0);
		spi1writedata(0x02);

		spi1writeindex(0xB9);
		spi1writedata(0x00);
		spi1writedata(current_intensity);	// PWM control. default brightness : 108
		spi1writedata(0x02);
		spi1writedata(0x08);
		
		spi1writeindex(0xB0);
		spi1writedata(0x03);

		current_intensity= orig_intensity;		
	}
	else if(current_panel==3) // if SMD
	{
		spi1writeindex(0xB4);
		spi1writedata(0x0F);
		spi1writedata(0x00);
		spi1writedata(0x50);

		spi1writeindex(0xB5);
		spi1writedata(current_intensity); 

		spi1writeindex(0xB7);
		spi1writedata(0x24);

		spi1writeindex(0xB8);
		spi1writedata(0x01);
	}
	else // Sony, Hydis, Sony(a-Si)
	{
    	spi1writeindex(0x51);
	spi1writedata(current_intensity);
	}
	//spin_unlock_irqrestore(&aat1402_bl_lock, flags);
	//spin_unlock(&aat1402_bl_lock);

}

static int aat1402_bl_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}

static int aat1402_bl_set_intensity(struct backlight_device *bd)
{
	//unsigned long flags;
	int intensity = bd->props.brightness;
//	int retry_count=10;
	
	if( intensity < 0 || intensity > 255 )
		return;
/*
	while(atomic_read(&ldi_power_state)==POWER_OFF) 
	{
		if(--retry_count == 0)
			break;
		mdelay(5);
	}
*/	
	current_intensity = intensity;
	if(atomic_read(&ldi_power_state)==POWER_OFF) 
		return;
	aat1402_set_brightness();

	return 0;
}

static struct backlight_ops aat1402_bl_ops = {
	.get_brightness = aat1402_bl_get_intensity,
	.update_status  = aat1402_bl_set_intensity,
};
// ]] backlight control 

static int nt35510_spi_probe(struct spi_device *spi)
{
    struct backlight_properties props;
     int status =0;
	 int ret;
		
	printk(KERN_INFO " **** nt35510_spi_probe.\n");
	nt35510lcd_spi = spi;
	nt35510lcd_spi->mode = SPI_MODE_0;
	nt35510lcd_spi->bits_per_word = 9 ;

	printk(" nt35510lcd_spi->chip_select = %x\t, mode = %x\n", nt35510lcd_spi->chip_select,  nt35510lcd_spi->mode);
	printk("ax_speed_hz  = %x\t modalias = %s", nt35510lcd_spi->max_speed_hz, nt35510lcd_spi->modalias );
	
	status = spi_setup(nt35510lcd_spi);
	printk(" spi_setup ret = %x\n",status );
	
	omap_dss_register_driver(&nt35510_driver);
//	led_classdev_register(&spi->dev, &nt35510_backlight_led);
	struct backlight_device *bd;
	bd = backlight_device_register("omap_bl", &spi->dev, NULL, &aat1402_bl_ops, &props);
	bd->props.max_brightness = 255;
	bd->props.brightness = 125;

	ret = device_create_file(&(spi->dev), &dev_attr_lcd_power);
	if (ret < 0)
		dev_err(&(spi->dev), "failed to add sysfs entries\n");
		
#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
	lcd_enabled = 0;
	nt35510_lcd_poweron();
#else
	lcd_enabled =1;
#endif

	return 0;
}

static int nt35510_spi_remove(struct spi_device *spi)
{
//	led_classdev_unregister(&nt35510_backlight_led);
	omap_dss_unregister_driver(&nt35510_driver);

	return 0;
}
static void nt35510_spi_shutdown(struct spi_device *spi)
{
	printk("*** First power off LCD.\n");
	is_nt35510_spi_shutdown = 1;
	nt35510_lcd_poweroff();
	printk("*** power off - backlight.\n");
	gpio_set_value(OMAP_GPIO_LCD_EN_SET, GPIO_LEVEL_LOW);
}

static int nt35510_spi_suspend(struct spi_device *spi, pm_message_t mesg)
{
    //spi_send(spi, 2, 0x01);  /* R2 = 01h */
    //mdelay(40);

#if 0
	nt35510lcd_spi = spi;
	nt35510lcd_spi->mode = SPI_MODE_0;
	nt35510lcd_spi->bits_per_word = 16 ;
	spi_setup(nt35510lcd_spi);

	lcd_poweroff();
	zeus_panel_power_enable(0);
#endif

	return 0;
}

static int nt35510_spi_resume(struct spi_device *spi)
{
	/* reinitialize the panel */
#if 0
	zeus_panel_power_enable(1);
	nt35510lcd_spi = spi;
	nt35510lcd_spi->mode = SPI_MODE_0;
	nt35510lcd_spi->bits_per_word = 16 ;
	spi_setup(nt35510lcd_spi);

	lcd_poweron();
#endif
	return 0;
}

static struct spi_driver nt35510_spi_driver = {
	.probe    = nt35510_spi_probe,
	.remove   = nt35510_spi_remove,
	.shutdown = nt35510_spi_shutdown,
	.suspend  = nt35510_spi_suspend,
	.resume   = nt35510_spi_resume,
	.driver   = {
		.name   = "nt35510_disp_spi",
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
	},
};

static int __init nt35510_lcd_init(void)
{

   	return spi_register_driver(&nt35510_spi_driver);
}

static void __exit nt35510_lcd_exit(void)
{
	return spi_unregister_driver(&nt35510_spi_driver);
}

module_init(nt35510_lcd_init);
module_exit(nt35510_lcd_exit);
MODULE_LICENSE("GPL");
