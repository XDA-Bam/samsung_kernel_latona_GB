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
 * File Name : board-adam-peripherals.c
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 08/Feb/2011
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

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <plat/mcspi.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/control.h>
#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif

#define ZEUS_CAM
#ifdef ZEUS_CAM
/* include files for cam pmic (power) and cam sensor */
#include "../../../drivers/media/video/cam_pmic.h"
#include "../../../drivers/media/video/ce147.h"
#include "../../../drivers/media/video/s5ka3dfx.h"
struct ce147_platform_data omap_board_ce147_platform_data;
struct s5ka3dfx_platform_data omap_board_s5ka3dfx_platform_data;
#endif
#include "mux.h"
#include "hsmmc.h"

/* TODO: OMAP-Samsung Board-Porting Layer */
#include <mach/board-adam.h>
#include <mach/sec_log_buf.h>
#include <mach/sec_param.h>

#define OMAP_GPIO_TSP_INT 142


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
		.name		= "external",
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.power_saving	= true,
	},
	{
		.name		= "internal",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= true,
		.power_saving	= true,
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


static int omap_board_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	//mmc[0].gpio_cd = gpio + 0;

	 mmc[0].gpio_cd = 23;
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters ... we "know" the
	 * regulators will be set up only *after* we return.
	*/
	omap_board_vmmc1_supply.dev = mmc[0].dev;
	omap_board_vsim_supply.dev = mmc[0].dev;
	omap_board_vmmc2_supply.dev = mmc[1].dev;

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
	.rep = 1,
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

	{
		I2C_BOARD_INFO("max97000", 0x4d),
	},
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
#if 0
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
#if !defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
	{
		I2C_BOARD_INFO("gp2a", 0x44),
	},
#endif
#if !defined(CONFIG_INPUT_YAS529_USE_GPIO_I2C)
	{
		I2C_BOARD_INFO("yas529", 0x2E),
	},
#endif
	{
		I2C_BOARD_INFO("Si4709_driver", 0x10),			
	},
	#endif 
};
//Added for I2C3 register-CY8 --Not using 

static struct i2c_board_info __initdata board_i2c_boardinfo_4[] = {
    {
        I2C_BOARD_INFO("melfas_ts", 0x40),// 10010(A1)(A0)  A1=PD0, A0=M(0=12bit, 1=8bit)
        .type = "melfas_ts",
        //.platform_data = &tsc2007_info,
    },
};
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
	.mode			= MUSB_OTG,
	.power			= 100,
};

static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
#if defined(CONFIG_SERIAL_OMAP_UART1_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART1_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART1_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART1_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART1_DMA */
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
#if defined(CONFIG_SERIAL_OMAP_UART2_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART2_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART2_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART2_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART2_DMA */
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
#if defined(CONFIG_SERIAL_OMAP_UART3_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART3_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART3_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART3_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART3_DMA */
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
#if defined(CONFIG_SERIAL_OMAP_UART4_DMA)
		.use_dma	= CONFIG_SERIAL_OMAP_UART4_DMA,
		.dma_rx_buf_size = CONFIG_SERIAL_OMAP_UART4_RXDMA_BUFSIZE,
		.dma_rx_timeout	= CONFIG_SERIAL_OMAP_UART4_RXDMA_TIMEOUT,
#else
		.use_dma	= 0,
		.dma_rx_buf_size = 0,
		.dma_rx_timeout	= 0,
#endif /* CONFIG_SERIAL_OMAP_UART3_DMA */
		.idle_timeout	= CONFIG_SERIAL_OMAP_IDLE_TIMEOUT,
		.flags		= 1,
	},
	{
		.flags		= 0
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
      
	omap_i2c_init();
	atmel_dev_init();
	omap_serial_init(omap_serial_platform_data);
	usb_musb_init(&musb_board_data);
	enable_board_wakeup_source();
	
}
