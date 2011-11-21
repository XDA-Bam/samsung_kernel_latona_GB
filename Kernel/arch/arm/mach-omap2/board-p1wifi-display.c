/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom-peripherals.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/spi/spi.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/mcspi.h>
#include <plat/display.h>

#if 0
#define LCD_PANEL_ENABLE_GPIO		(7 + OMAP_MAX_GPIO_LINES)
#define LCD_PANEL_RESET_GPIO_PROD	96
#define LCD_PANEL_RESET_GPIO_PILOT	55
#define LCD_PANEL_QVGA_GPIO		56
#define TV_PANEL_ENABLE_GPIO		95

#endif


 struct omap_dss_device omap_board_lcd_device = {
	.name = "lcd",
	.driver_name = "sec_tune_cmc623",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines = 24,
	.platform_enable = NULL,
	.platform_disable = NULL,
};

static struct omap_dss_device *omap_board_dss_devices[] = {
	&omap_board_lcd_device,
};

static struct omap_dss_board_info omap_board_dss_data = {
	.num_devices = ARRAY_SIZE(omap_board_dss_devices),
	.devices = omap_board_dss_devices,
	.default_device = &omap_board_lcd_device,
};


//ZEUS_LCD
static struct omap2_mcspi_device_config board_lcd_mcspi_config = {
    .turbo_mode = 0,
    .single_channel = 1,    /* 0: slave, 1: master */
};

static struct spi_board_info board_spi_board_info[] __initdata = {

    [0] = {
           .modalias = "nt35510_disp_spi",
           .bus_num = 1,
           .chip_select = 0,
           .max_speed_hz = 375000,
           .controller_data = &board_lcd_mcspi_config,
           },     
};

void __init omap_board_display_init(enum omap_dss_venc_type venc_type)
{
	
	omap_display_init(&omap_board_dss_data);
	spi_register_board_info(board_spi_board_info,
				ARRAY_SIZE(board_spi_board_info));
	
}

