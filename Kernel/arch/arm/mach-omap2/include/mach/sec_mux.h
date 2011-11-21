/**
 * arch/arm/mach-omap2/sec_mux.h
 *
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
 * File Name : sec_mux.h
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 03/Mar/2011
 * Version : Baby-Raccoon
 */

#ifndef __SEC_MUX_H__
#define __SEC_MUX_H__

int sec_mux_init_padconf(void);

int sec_mux_init_gpio_out(void);

int sec_mux_set_wakeup_gpio(void);

#endif /* __SEC_MUX_H__ */
