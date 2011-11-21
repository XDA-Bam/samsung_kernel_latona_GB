/**
 * Copyright (C) 2010-2011, Samsung Electronics, Co., Ltd. All Rights Reserved.
 *  Written by System S/W Group, Open OS S/W R&D Team,
 *  Mobile Communication Division.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/**
 * Project Name : OMAP-Samsung Linux Kernel for Android
 *
 * Project Description :
 *
 * Comments : tabstop = 8, shiftwidth = 8, noexpandtab
 */

/**
 * File Name : board-latona-peripherals.c
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 26/Jan/2011
 * Version : Baby-Raccoon
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/regulator/machine.h>
#include <linux/mmc/host.h>
#include <linux/leds.h>
#include "twl4030.h"

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <plat/mcspi.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/control.h>
#include <linux/switch.h>
#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif

#include <linux/spi/spi.h>
#include <plat/mcspi.h>

#define ZEUS_CAM
#ifdef ZEUS_CAM
/* include files for cam pmic (power) and cam sensor */
#include "../../../drivers/media/video/cam_pmic.h"
#include "../../../drivers/media/video/ce147.h"
#include "../../../drivers/media/video/s5ka3dfx.h"
struct ce147_platform_data omap_board_ce147_platform_data;
struct s5ka3dfx_platform_data omap_board_s5ka3dfx_platform_data;
#endif

#if defined( CONFIG_SAMSUNG_PHONE_SVNET )
#include <linux/phone_svn/modemctl.h>
#if defined(CONFIG_PHONE_ONEDRAM)
#include <linux/phone_svn/onedram.h>
#elif defined (CONFIG_PHONE_IPC_SPI)
#include <linux/phone_svn/ipc_spi.h>
#endif
#include <linux/irq.h>
#endif // CONFIG_SAMSUNG_PHONE_SVNET

#ifdef CONFIG_WL127X_RFKILL
#include <linux/wl127x-rfkill.h>
#endif

#include "mux.h"
#include "hsmmc.h"

/* TODO: OMAP-Samsung Board-Porting Layer */
#include <mach/board-latona.h>
#include <mach/sec_log_buf.h>
#include <mach/sec_param.h>

#define OMAP_GPIO_TSP_INT 142

static struct gpio_switch_platform_data headset_switch_data = {
	.name = "h2w",
	.gpio = OMAP_GPIO_DET_3_5,	/* Omap3430 GPIO_27 For samsung zeus */
};

static struct platform_device headset_switch_device = {
	.name = "switch-gpio",
	.dev = {
		.platform_data = &headset_switch_data,
		}
};

#ifdef CONFIG_SWITCH_SIO
struct platform_device sec_sio_switch = {
	.name = "switch-sio",
	.id = -1,
};
#endif

#if defined( CONFIG_SAMSUNG_PHONE_SVNET )
#if defined( CONFIG_PHONE_ONEDRAM )
static void onedram_cfg_gpio( void )
{
	// Mux Setting -> mux_xxxx_rxx.c

	// Irq Setting - Onedram Mailbox Int
	set_irq_type( OMAP_GPIO_IRQ( 181 ), IRQ_TYPE_EDGE_RISING );
}

static struct onedram_platform_data onedram_data = {
	.cfg_gpio = onedram_cfg_gpio,
};

static struct resource onedram_res[] = {
	[ 0 ] = {
		.start = ( 0x80000000 + 0x05000000 ), // Physical memory address
		.end = ( 0x80000000 + 0x05000000 + 0x1000000 - 1 ),
		.flags = IORESOURCE_MEM,
	},
	[ 1 ] = {
		.start = OMAP_GPIO_IRQ( 181 ), // Irq - Onedram Mailbox Int
		.end = OMAP_GPIO_IRQ( 181 ),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device onedram = {
	.name = "onedram",
	.id = -1,
	.num_resources = ARRAY_SIZE( onedram_res ),
	.resource = onedram_res,
	.dev = {
		.platform_data = &onedram_data,
	},
};
#elif defined( CONFIG_PHONE_IPC_SPI )
static void ipc_spi_cfg_gpio( void );

static struct ipc_spi_platform_data ipc_spi_data = {
	.gpio_ipc_mrdy = OMAP_GPIO_IPC_MRDY,
	.gpio_ipc_srdy = OMAP_GPIO_IPC_SRDY,	

	.cfg_gpio = ipc_spi_cfg_gpio,
};

static struct resource ipc_spi_res[] = {
	[ 0 ] = {
		.start = OMAP_GPIO_IRQ( OMAP_GPIO_IPC_SRDY ),
		.end = OMAP_GPIO_IRQ( OMAP_GPIO_IPC_SRDY ),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device ipc_spi = {
	.name = "onedram",
	.id = -1,
	.num_resources = ARRAY_SIZE( ipc_spi_res ),
	.resource = ipc_spi_res,
	.dev = {
		.platform_data = &ipc_spi_data,
	},
};

static void ipc_spi_cfg_gpio( void )
{
	int err = 0;
	
	unsigned gpio_ipc_mrdy = ipc_spi_data.gpio_ipc_mrdy;
	unsigned gpio_ipc_srdy = ipc_spi_data.gpio_ipc_srdy;

	// Mux Setting -> mux_xxxx_rxx.c

	gpio_free( gpio_ipc_mrdy );
	err = gpio_request( gpio_ipc_mrdy, "IPC_MRDY" );
	if( err ) {
		printk( "ipc_spi_cfg_gpio - fail to request gpio %s : %d\n", "IPC_MRDY", err );
	}
	else {
		gpio_direction_output( gpio_ipc_mrdy, 0 );
	}

	gpio_free( gpio_ipc_srdy );
	err = gpio_request( gpio_ipc_srdy, "IPC_SRDY" );
	if( err ) {
		printk( "ipc_spi_cfg_gpio - fail to request gpio %s : %d\n", "IPC_SRDY", err );
	}
	else {
		gpio_direction_input( gpio_ipc_srdy );
	}
	
	// Irq Setting -
	set_irq_type( OMAP_GPIO_IRQ( OMAP_GPIO_IPC_SRDY ), IRQ_TYPE_LEVEL_HIGH );
}
#endif // CONFIG_PHONE_ONEDRAM, CONFIG_PHONE_IPC_SPI

static void modemctl_cfg_gpio( void );

static struct modemctl_platform_data mdmctl_data = {
	.name = "xmm",
	
	.gpio_phone_active = OMAP_GPIO_PHONE_ACTIVE,
	.gpio_pda_active = OMAP_GPIO_PDA_ACTIVE,
	.gpio_cp_reset = OMAP_GPIO_CP_RST, // cp_rst gpio - 43
	.gpio_reset_req_n = OMAP_GPIO_RESET_REQ_N,
	
#if ( defined( CONFIG_MACH_SAMSUNG_LATONA ) && ( CONFIG_SAMSUNG_REL_HW_REV >= 2 ) )
	.gpio_con_cp_sel = OMAP_GPIO_CON_CP_SEL,
#endif // HW Rev 02

	//.gpio_flm_sel = OMAP_GPIO_FLM_SEL,
	//.gpio_phone_on = GPIO_PHONE_ON,
	//.gpio_usim_boot = GPIO_USIM_BOOT,
	//.gpio_sim_ndetect = GPIO_SIM_nDETECT,
	
	.cfg_gpio = modemctl_cfg_gpio,
};

static struct resource mdmctl_res[] = {
	[ 0 ] = {
		.start = OMAP_GPIO_IRQ( OMAP_GPIO_PHONE_ACTIVE ), // phone active irq
		.end = OMAP_GPIO_IRQ( OMAP_GPIO_PHONE_ACTIVE ),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device modemctl = {
	.name = "modemctl",
	.id = -1,
	.num_resources = ARRAY_SIZE( mdmctl_res ),
	.resource = mdmctl_res,
	.dev = {
		.platform_data = &mdmctl_data,
	},
};

static void modemctl_cfg_gpio( void )
{
	int err = 0;
	
	unsigned gpio_cp_rst = mdmctl_data.gpio_cp_reset;
	unsigned gpio_pda_active = mdmctl_data.gpio_pda_active;
	unsigned gpio_phone_active = mdmctl_data.gpio_phone_active;
	unsigned gpio_reset_req_n = mdmctl_data.gpio_reset_req_n;
	
#if ( defined( CONFIG_MACH_SAMSUNG_LATONA ) && ( CONFIG_SAMSUNG_REL_HW_REV >= 2 ) )
	unsigned gpio_con_cp_sel = mdmctl_data.gpio_con_cp_sel;
#endif // HW Rev 02

	// Mux Setting -> mux_xxxx_rxx.c

	gpio_free( gpio_cp_rst );
	err = gpio_request( gpio_cp_rst, "CP_RST" );
	if( err ) {
		printk( "modemctl_cfg_gpio - fail to request gpio %s : %d\n", "CP_RST", err );
	}
	else {
		gpio_direction_output( gpio_cp_rst, 1 );
	}

	gpio_free( gpio_pda_active );
	err = gpio_request( gpio_pda_active, "PDA_ACTIVE" );
	if( err ) {
		printk( "modemctl_cfg_gpio - fail to request gpio %s : %d\n", "PDA_ACTIVE", err );
	}
	else {
		gpio_direction_output( gpio_pda_active, 0 );
	}

	gpio_free( gpio_phone_active );
	err = gpio_request( gpio_phone_active, "PHONE_ACTIVE" );
	if( err ) {
		printk( "modemctl_cfg_gpio - fail to request gpio %s : %d\n", "PHONE_ACTIVE", err );
	}
	else {
		gpio_direction_input( gpio_phone_active );
	}

	gpio_free( gpio_reset_req_n );
	err = gpio_request( gpio_reset_req_n, "RESET_REQ_N" );
	if( err ) {
		printk( "modemctl_cfg_gpio - fail to request gpio %s : %d\n", "RESET_REQ_N", err );
	}
	else {
		gpio_direction_output( gpio_reset_req_n, 0 );
	}

#if ( defined( CONFIG_MACH_SAMSUNG_LATONA ) && ( CONFIG_SAMSUNG_REL_HW_REV >= 2 ) )
	gpio_free( gpio_con_cp_sel );
	err = gpio_request( gpio_con_cp_sel, "CON_CP_SEL" );
	if( err ) {
		printk( "modemctl_cfg_gpio - fail to request gpio %s : %d\n", "CON_CP_SEL", err );
	}
	else {
		gpio_direction_output( gpio_con_cp_sel, 0 );
	}
#endif // HW Rev 02
	
	set_irq_type( OMAP_GPIO_IRQ( OMAP_GPIO_PHONE_ACTIVE ), IRQ_TYPE_EDGE_BOTH );

	//set_irq_type( gpio_sim_ndetect, IRQ_TYPE_EDGE_BOTH );
}
#endif // CONFIG_SAMSUNG_PHONE_SVNET

#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
static struct resource board_ear_key_resource = {
	.start = 0,
	.end = 0,
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
};
static struct platform_device board_ear_key_device = {
	.name = "sec_jack",
	.id = -1,
	.num_resources = 1,
	.resource = &board_ear_key_resource,
};
#endif
static struct regulator_consumer_supply omap_board_vdda_dac_supply = {
	.supply = "vdda_dac",
	
};
static struct regulator_consumer_supply omap_board_vsim_supply = {
	.supply		= "vmmc_aux",
};

static struct regulator_consumer_supply omap_board_vmmc1_supply = {
	.supply = "vmmc",
};

static struct regulator_consumer_supply omap_board_vmmc2_supply = {
	.supply = "vmmc",
};
static struct regulator_consumer_supply omap_board_vaux1_supply = {
	.supply = "vaux1",
};

static struct regulator_consumer_supply omap_board_vaux2_supply = {
	.supply = "vaux2",
};

static struct regulator_consumer_supply omap_board_vaux3_supply = {
	.supply = "vaux3",
};

static struct regulator_consumer_supply omap_board_vaux4_supply = {
	.supply = "vaux4",
};

static struct regulator_consumer_supply omap_board_vpll2_supply = {
	.supply = "vpll2",
};
struct regulator_init_data omap_board_vdac = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap_board_vdda_dac_supply,
};
/* VMMC1 for OMAP VDD_MMC1 (i/o) and MMC1 card */
static struct regulator_init_data omap_board_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &omap_board_vmmc1_supply,
};

/* VMMC2 for MMC2 card */
static struct regulator_init_data omap_board_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 1850000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &omap_board_vmmc2_supply,
};


/* VAUX1 for PL_SENSOR */
static struct regulator_init_data omap_board_aux1 = {
	.constraints = {
			.min_uV = 3000000,
			.max_uV = 3000000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap_board_vaux1_supply,
};

/* VAUX2 for PL_SENSOR */
static struct regulator_init_data omap_board_aux2 = {
	.constraints = {
			.min_uV = 2800000,
			.max_uV = 2800000,
			.apply_uV = true,
    #if 1  //TI patch to avoid leakage current in U404 chip(AP_SCL voltage correction for sensor )
			.always_on = true,
    #endif 
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap_board_vaux2_supply,
};


/* VSIM for OMAP VDD_MMC1A (i/o for DAT4..DAT7) */
static struct regulator_init_data omap_board_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &omap_board_vsim_supply,
};

static struct omap2_hsmmc_info mmc[] __initdata = {
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
	},
	{
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.power_saving	= true,
	},
	{
		.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}      /* Terminator */
};


static struct regulator_init_data omap_board_vpll2 = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &omap_board_vpll2_supply,
};


/* VAUX3 for LCD */
static struct regulator_init_data omap_board_aux3 = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.boot_on = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap_board_vaux3_supply,
};

/* VAUX4 for LCD */
static struct regulator_init_data omap_board_aux4 = {
	.constraints = {
			.min_uV = 2800000,
			.max_uV = 2800000,
			.boot_on = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap_board_vaux4_supply,
};

/* VPLL2 for LCD */
static struct regulator_init_data board_vpll2 = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.boot_on = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &omap_board_vpll2_supply,
};

static struct __initdata twl4030_power_data latona_t2scripts_data;



static int omap_board_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	//mmc[0].gpio_cd = gpio + 0;

	 mmc[1].gpio_cd = 23;
#ifdef CONFIG_MMC_EMBEDDED_SDIO
	/* The controller that is connected to the 128x device
	 * should have the card detect gpio disabled. This is
	 * achieved by initializing it with a negative value
	 */
	mmc[CONFIG_TIWLAN_MMC_CONTROLLER - 1].gpio_cd = -EINVAL;
#endif

	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	/*
	omap_board_vmmc1_supply.dev = mmc[0].dev;
	omap_board_vsim_supply.dev = mmc[0].dev;
	omap_board_vmmc2_supply.dev = mmc[1].dev;
	*/
	omap_board_vmmc1_supply.dev = mmc[1].dev;
	omap_board_vsim_supply.dev = mmc[1].dev;
	omap_board_vmmc2_supply.dev = mmc[0].dev;
	
	return 0;
}


//ZEUS_LCD
static struct omap_lcd_config board_lcd_config __initdata = {
    .ctrl_name = "internal",
};

//ZEUS_LCD
static struct omap_uart_config board_uart_config __initdata = {
#ifdef CONFIG_SERIAL_OMAP_CONSOLE
    .enabled_uarts = ((1 << 0) | (1 << 1) | (1 << 2)),
#else
    .enabled_uarts = ((1 << 0) | (1 << 1)),
#endif
};

static struct omap_board_config_kernel board_config[] __initdata = {
    {OMAP_TAG_UART, &board_uart_config},
    {OMAP_TAG_LCD, &board_lcd_config},  //ZEUS_LCD
};


#ifdef CONFIG_WL127X_RFKILL
static struct wl127x_rfkill_platform_data wl127x_plat_data = {
	.bt_nshutdown_gpio = OMAP_GPIO_BT_NRST,	/* Bluetooth Enable GPIO */
	.fm_enable_gpio = -1,	/* FM Enable GPIO */
};

static struct platform_device zoom2_wl127x_device = {
	.name = "wl127x-rfkill",
	.id = -1,
	.dev.platform_data = &wl127x_plat_data,
};
#endif

#ifdef CONFIG_SAMSUNG_HW_EMU_BOARD
static int omap_board_twl4030_keymap[] = {
	KEY(0, 1, KEY_MENU),
	KEY(0, 2, KEY_BACK),
	KEY(1, 1, KEY_CAMERA_FOCUS),
	KEY(1, 2, KEY_VOLUMEUP),
	KEY(2, 1, KEY_CAMERA),
	KEY(2, 2, KEY_VOLUMEDOWN),
	0
};
#else
static int omap_board_twl4030_keymap[] = {
	KEY(2, 1, KEY_VOLUMEUP),
	KEY(1, 1, KEY_VOLUMEDOWN),
	0
};
#endif
static struct matrix_keymap_data board_map_data = {
	.keymap = omap_board_twl4030_keymap,
	.keymap_size = ARRAY_SIZE(omap_board_twl4030_keymap),
};
static struct twl4030_keypad_data board_kp_data = {
	.keymap_data = &board_map_data,
	.rows = 5,
	.cols = 6,
	.rep = 0,
};

static struct resource board_power_key_resources[] = {
	[0] = {
	       // PWRON KEY
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	       },
	[1] = {
	       // HOME KEY
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	       },

};

static struct platform_device board_power_key_device = {
	.name = "power_key_device",
	.id = -1,
	.num_resources = ARRAY_SIZE(board_power_key_resources),
	.resource = board_power_key_resources,
};

static struct resource samsung_charger_resources[] = {
	[0] = {
	       // USB IRQ
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
	       },
	[1] = {
	       // TA IRQ
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	       },
	[2] = {
	       // CHG_ING_N
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	       },
	[3] = {
	       // CHG_EN
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ,
	       },
};

static int samsung_charger_config_data[] = {
	// [ CHECK VF USING ADC ]
	/*   1. ENABLE  (true, flase) */
	false,

	/*   2. ADCPORT (ADCPORT NUM) */
	1,

	// [ SUPPORT CHG_ING IRQ FOR CHECKING FULL ]
	/*   1. ENABLE  (true, flase) */
	true,
};

static int samsung_battery_config_data[] = {
	// [ SUPPORT MONITORING CHARGE CURRENT FOR CHECKING FULL ]
	/*   1. ENABLE  (true, flase) */
	false,
	/*   2. ADCPORT (ADCPORT NUM) */
	4,

	// [ SUPPORT MONITORING TEMPERATURE OF THE SYSTEM FOR BLOCKING CHARGE ]
	/*   1. ENABLE  (true, flase) */
	true,

	/*   2. ADCPORT (ADCPORT NUM) */
	0,
};

static struct platform_device samsung_charger_device = {
	.name = "secChargerDev",
	.id = -1,
	.num_resources = ARRAY_SIZE(samsung_charger_resources),
	.resource = samsung_charger_resources,

	.dev = {
		.platform_data = &samsung_charger_config_data,
		}
};

static struct platform_device samsung_battery_device = {
	.name = "secBattMonitor",
	.id = -1,
	.num_resources = 0,
	.dev = {
		.platform_data = &samsung_battery_config_data,
		}
};

static struct led_info sec_keyled_list[] = {
	{
	 .name = "button-backlight",
	 },
};

static struct led_platform_data sec_keyled_data = {
	.num_leds = ARRAY_SIZE(sec_keyled_list),
	.leds = sec_keyled_list,
};

static struct platform_device samsung_led_device = {
	.name = "secLedDriver",
	.id = -1,
	.num_resources = 0,
	.dev = {
		.platform_data = &sec_keyled_data,
		},
};

static struct platform_device samsung_vibrator_device = {
	.name = "secVibrator",
	.id = -1,
	.num_resources = 0,
};

static struct platform_device samsung_pl_sensor_power_device = {
	.name = "secPLSensorPower",
	.id = -1,
	.num_resources = 0,
};

#if defined( CONFIG_PHONE_IPC_SPI )
static struct omap2_mcspi_device_config board_ipc_spi_mcspi_config = {
	.turbo_mode     =   0,
	.single_channel =   1,
};

static struct spi_board_info board_spi_board_info[] __initdata = {
	[ 0 ] = {
		.modalias = "ipc_spi",
		.bus_num = 2,
		.chip_select = 0,
		.max_speed_hz = 24000000,
		.controller_data = &board_ipc_spi_mcspi_config,
	},

};
#endif // CONFIG_PHONE_IPC_SPI

static struct platform_device *board_devices[] __initdata = {

	&headset_switch_device,
#ifdef CONFIG_WL127X_RFKILL
	&zoom2_wl127x_device,
#endif
#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
	&board_ear_key_device,
#endif
	&board_power_key_device,
	&samsung_battery_device,
	&samsung_charger_device,
	&samsung_vibrator_device,
	&samsung_pl_sensor_power_device,
	&samsung_led_device,

#if defined( CONFIG_SAMSUNG_PHONE_SVNET )
#if defined( CONFIG_PHONE_ONEDRAM )
	&onedram,
#elif defined( CONFIG_PHONE_IPC_SPI )
	&ipc_spi,
#endif	
	&modemctl,
#endif // CONFIG_SAMSUNG_PHONE_SVNET
#ifdef CONFIG_SWITCH_SIO
    &sec_sio_switch,
#endif
};

static int omap_board_batt_table[] = {
/* 0 C*/
30800, 29500, 28300, 27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,  9280,  8950,  8620,  8310,
8020,  7730,  7460,  7200,  6950,  6710,  6470,  6250,  6040,  5830,
5640,  5450,  5260,  5090,  4920,  4760,  4600,  4450,  4310,  4170,
4040,  3910,  3790,  3670,  3550
};

static struct twl4030_bci_platform_data omap_board_bci_data = {
	.battery_tmp_tbl	= omap_board_batt_table,
	.tblsize		= ARRAY_SIZE(omap_board_batt_table),
};

static struct twl4030_usb_data omap_board_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_gpio_platform_data omap_board_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.setup		= omap_board_twl_gpio_setup,
	.debounce       = 0x04,
};
static struct twl4030_usb_data board_usb_data = {
	.usb_mode = T2_USB_MODE_ULPI,
};
static struct twl4030_madc_platform_data omap_board_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data omap_board_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data omap_board_codec_data = {
	.audio_mclk = 26000000,
	.audio = &omap_board_audio_data,
};

static struct twl4030_platform_data omap_board_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &omap_board_bci_data,
	.madc		= &omap_board_madc_data,
	.usb		= &omap_board_usb_data,
	.gpio		= &omap_board_gpio_data,
	.keypad     = &board_kp_data,
	.codec		= &omap_board_codec_data,
	.power 		= &latona_t2scripts_data,
	.vmmc1      = &omap_board_vmmc1,
	.vmmc2      = &omap_board_vmmc2,
	//.vsim       = &omap_board_vsim,
    .vaux1      = &omap_board_aux1,
	.vaux2      = &omap_board_aux2,
	.vaux3      = &omap_board_aux3,
	.vaux4      = &omap_board_aux4,
	.vpll2		= &omap_board_vpll2,
	.vdac		= &omap_board_vdac,

};

static struct i2c_board_info __initdata omap_board_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl5030", 0x48),
		.flags		= I2C_CLIENT_WAKE,
		.irq		= INT_34XX_SYS_NIRQ,
		.platform_data	= &omap_board_twldata,
	},
};


static struct i2c_board_info __initdata board_i2c_boardinfo1[] = {

#if 1
#if defined(CONFIG_FSA9480_MICROUSB)
	{
		I2C_BOARD_INFO("fsa9480", 0x25),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP_GPIO_IRQ(OMAP_GPIO_JACK_NINT),
	},
#elif defined(CONFIG_MICROUSBIC_INTR)
	{
		I2C_BOARD_INFO("microusbic", 0x25),
	},
#endif

#if defined(CONFIG_SND_SOC_MAX9877)
	{
		I2C_BOARD_INFO("max9877", 0x4d),
	},
#elif defined(CONFIG_SND_SOC_MAX97000)
	{
		I2C_BOARD_INFO("max97000", 0x4d),
	},
#endif
	{
		I2C_BOARD_INFO(CE147_DRIVER_NAME, CE147_I2C_ADDR),
		.platform_data = &omap_board_ce147_platform_data,
	},
	{
		I2C_BOARD_INFO(S5KA3DFX_DRIVER_NAME, S5KA3DFX_I2C_ADDR),
		.platform_data = &omap_board_s5ka3dfx_platform_data,
	},
	{
		I2C_BOARD_INFO("cam_pmic", CAM_PMIC_I2C_ADDR),
	},
	{
		I2C_BOARD_INFO("secFuelgaugeDev", 0x36),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP_GPIO_IRQ(OMAP_GPIO_FUEL_INT_N),
	},
#endif	
#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
	{
		I2C_BOARD_INFO("gp2a", 0x44),
	},
#endif
#if !defined(CONFIG_INPUT_YAS529_USE_GPIO_I2C)
	{
		I2C_BOARD_INFO("geomagnetic", 0x2E),
	},
#endif

	{
		I2C_BOARD_INFO("Si4709_driver", 0x10),			
	},
};
//Added for I2C3 register-CY8 --Not using 

static struct i2c_board_info __initdata board_i2c_boardinfo_4[] = {
    {
        I2C_BOARD_INFO("melfas_ts", 0x40),// 10010(A1)(A0)  A1=PD0, A0=M(0=12bit, 1=8bit)
        .type = "melfas_ts",
        //.platform_data = &tsc2007_info,
    },
};
#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
static inline void __init board_init_ear_key(void)
{
	board_ear_key_resource.start = gpio_to_irq(OMAP_GPIO_EAR_SEND_END);
	if (gpio_request(OMAP_GPIO_EAR_SEND_END, "ear_key_irq") < 0) {
		printk(KERN_ERR
		       "\n FAILED TO REQUEST GPIO %d for POWER KEY IRQ \n",
		       OMAP_GPIO_EAR_SEND_END);
		return;
	}
	gpio_direction_input(OMAP_GPIO_EAR_SEND_END);
}
#endif

static inline void __init board_init_power_key(void)
{
	board_power_key_resources[0].start = gpio_to_irq(OMAP_GPIO_KEY_PWRON);
	if (gpio_request(OMAP_GPIO_KEY_PWRON, "power_key_irq") < 0) {
		printk(KERN_ERR
		       "\n FAILED TO REQUEST GPIO %d for POWER KEY IRQ \n",
		       OMAP_GPIO_KEY_PWRON);
		return;
	}
	board_power_key_resources[1].start = gpio_to_irq(OMAP_GPIO_KEY_HOME);
	if (gpio_request(OMAP_GPIO_KEY_HOME, "home_key_irq") < 0) {
		printk(KERN_ERR
		       "\n FAILED TO REQUEST GPIO %d for VOLDN KEY IRQ \n",
		       OMAP_GPIO_KEY_HOME);
		return;
	}
	gpio_direction_input(OMAP_GPIO_KEY_PWRON);
	gpio_direction_input(OMAP_GPIO_KEY_HOME);
}

static inline void __init board_init_battery(void)
{
	samsung_charger_resources[0].start = 0;	//gpio_to_irq(OMAP_GPIO_USBSW_NINT);;    
	if (gpio_request(OMAP_GPIO_TA_NCONNECTED, "ta_nconnected irq") < 0) {
		printk(KERN_ERR
		       "Failed to request GPIO%d for ta_nconnected IRQ\n",
		       OMAP_GPIO_TA_NCONNECTED);
		samsung_charger_resources[1].start = -1;
	} else {
		samsung_charger_resources[1].start =
		    gpio_to_irq(OMAP_GPIO_TA_NCONNECTED);
//      Daegil.im 2011-02-10 : below code was disabled for build			
//		omap_set_gpio_debounce_time(OMAP_GPIO_TA_NCONNECTED, 3);
//		omap_set_gpio_debounce(OMAP_GPIO_TA_NCONNECTED, true);
	}

	if (gpio_request(OMAP_GPIO_CHG_ING_N, "charge full irq") < 0) {
		printk(KERN_ERR
		       "Failed to request GPIO%d for charge full IRQ\n",
		       OMAP_GPIO_CHG_ING_N);
		samsung_charger_resources[2].start = -1;
	} else {
		samsung_charger_resources[2].start =
		    gpio_to_irq(OMAP_GPIO_CHG_ING_N);
//		omap_set_gpio_debounce_time(OMAP_GPIO_CHG_ING_N, 3);
//		omap_set_gpio_debounce(OMAP_GPIO_CHG_ING_N, true);
	}

	if (gpio_request(OMAP_GPIO_CHG_EN, "Charge enable gpio") < 0) {
		printk(KERN_ERR
		       "Failed to request GPIO%d for charge enable gpio\n",
		       OMAP_GPIO_CHG_EN);
		samsung_charger_resources[3].start = -1;
	} else {
		samsung_charger_resources[3].start =
		    gpio_to_irq(OMAP_GPIO_CHG_EN);
	}

	samsung_charger_resources[3].start = gpio_to_irq(OMAP_GPIO_CHG_EN);
}

static void atmel_dev_init(void)
{
	/* Set the ts_gpio pin mux */
	if (gpio_request(OMAP_GPIO_TSP_INT, "touch_atmel") < 0) {
		printk(KERN_ERR "can't get synaptics pen down GPIO\n");
		return;
	}
	gpio_direction_input(OMAP_GPIO_TSP_INT);
	
}
static int __init omap_i2c_init(void)
{
	
         /* Disable OMAP 3630 internal pull-ups for I2Ci */
    if (cpu_is_omap3630()) {

        u32 prog_io;

        prog_io = omap_ctrl_readl(OMAP343X_CONTROL_PROG_IO1);
        /* Program (bit 19)=1 to disable internal pull-up on I2C1 */
        prog_io |= OMAP3630_PRG_I2C1_PULLUPRESX;
        /* Program (bit 0)=1 to disable internal pull-up on I2C2 */
        prog_io |= OMAP3630_PRG_I2C2_PULLUPRESX;
        omap_ctrl_writel(prog_io, OMAP343X_CONTROL_PROG_IO1);

        prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO2);
        /* Program (bit 7)=1 to disable internal pull-up on I2C3 */
        prog_io |= OMAP3630_PRG_I2C3_PULLUPRESX;
        omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO2);

        prog_io = omap_ctrl_readl(OMAP36XX_CONTROL_PROG_IO_WKUP1);
        /* Program (bit 5)=1 to disable internal pull-up on I2C4(SR) */
        prog_io |= OMAP3630_PRG_SR_PULLUPRESX;
        omap_ctrl_writel(prog_io, OMAP36XX_CONTROL_PROG_IO_WKUP1);
    }

    omap_register_i2c_bus(2, 400, NULL, board_i2c_boardinfo1,
			     ARRAY_SIZE(board_i2c_boardinfo1));
    
    omap_register_i2c_bus(1, 400, NULL, omap_board_i2c_boardinfo,
                         ARRAY_SIZE(omap_board_i2c_boardinfo));
   
    omap_register_i2c_bus(3, 400, NULL, NULL, 0);

    return 0;
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_PERIPHERAL,
	.power			= 100,
};

static struct omap_uart_port_info omap_serial_platform_data[] = {
	 {
                .use_dma        = 0,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .idle_timeout   = DEFAULT_IDLE_TIMEOUT,
                .flags          = 1,
        },
        {
                .use_dma        = 0,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .idle_timeout   = DEFAULT_IDLE_TIMEOUT,
                .flags          = 1,
        },
        {
                .use_dma        = 0,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
                .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
               .idle_timeout   = DEFAULT_IDLE_TIMEOUT,
                .flags          = 1,
        },
        {
                .use_dma        = 0,
                .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
               .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
                .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
                .idle_timeout   = DEFAULT_IDLE_TIMEOUT,
                .flags          = 1,
        },
        {
                .flags          = 0
        }

};

static void enable_board_wakeup_source(void)
{
	/* T2 interrupt line (keypad) */
	omap_mux_init_signal("sys_nirq",
		OMAP_WAKEUP_EN | OMAP_PIN_INPUT_PULLUP);
}

void __init omap_board_peripherals_init(void)
{
	printk("*******board_peripherals_init*****\n");
	twl4030_get_scripts(&latona_t2scripts_data);

	omap_i2c_init();

    platform_add_devices(board_devices, ARRAY_SIZE(board_devices));
       omap_board_usb_data.sensor_dev = &samsung_pl_sensor_power_device.dev;    // Add for regulator
       
	spi_register_board_info( board_spi_board_info, ARRAY_SIZE( board_spi_board_info ) );
       
	atmel_dev_init();
	omap_serial_init(omap_serial_platform_data);
	usb_musb_init(&musb_board_data);
	board_init_power_key();
	enable_board_wakeup_source();
	board_init_battery();
#ifdef CONFIG_INPUT_ZEUS_EAR_KEY
	board_init_ear_key();
#endif
}
