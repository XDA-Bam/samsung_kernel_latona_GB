/*
 * linux/arch/arm/mach-omap2/wl127x_wifi.h
 *
 * Copyright (C) 2010 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _BOARD_ZOOM2_WIFI_H
#define _BOARD_ZOOM2_WIFI_H

#define WL127X_WIFI_PMENA_GPIO	160

#if ( defined( CONFIG_MACH_SAMSUNG_LATONA ) && ( CONFIG_SAMSUNG_REL_HW_REV >= 2 ) )
#define WL127X_WIFI_IRQ_GPIO	99
#else
#define WL127X_WIFI_IRQ_GPIO	21
#endif

void config_wlan_mux(void);

#endif /* _BOARD_ZOOM2_WIFI_H */
