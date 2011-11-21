/**
 * arch/arm/mach-omap2/sec_mux.c
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
 * File Name : sec_mux.c
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 03/Mar/2011
 * Version : Baby-Raccoon
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <asm/system.h>
#include <plat/control.h>
#include <plat/mux.h>
#include <plat/gpio.h>
#include <linux/interrupt.h>

#include "mux.h"
#include <mach/sec_mux.h>

extern unsigned int sec_board_output_gpio_size;
extern unsigned int (*sec_board_output_gpio_ptr)[3];
extern unsigned int sec_board_wakeup_gpio_size;
extern unsigned int *sec_board_wakeup_gpio_ptr;
extern unsigned int sec_board_mux_size;
extern struct omap_board_mux *sec_board_mux_ptr;

int __init sec_mux_init_padconf(void)
{
	int ret = 0;
	unsigned int i;

	for (i = 0; i < sec_board_mux_size; i++) {
		ret = omap_cfg_reg(i);
		if (ret) {
			pr_err("omap pad conf. fail : %d, %d\n", ret, i);
		}
	}

	return ret;
}

int __init sec_mux_init_gpio_out(void)
{
	int ret = 0;
	int err = 0;
	unsigned int i = 0;

	for (i = 0; i < sec_board_output_gpio_size; i++) {
		err = gpio_request(sec_board_output_gpio_ptr[i][0],
				   (char *)sec_board_output_gpio_ptr[i][2]);
		if (err < 0) {
			pr_err("can't get %s GPIO\n",
			       (char *)sec_board_output_gpio_ptr[i][2]);
			ret = -1;
			goto __return;
		}

		gpio_direction_output(sec_board_output_gpio_ptr[i][0],
				      sec_board_output_gpio_ptr[i][1]);
	}

__return:
	return ret;
}

int __init sec_mux_set_wakeup_gpio(void)
{
	int ret = 0;
	unsigned int i = 0;

	for (i = 0; i < sec_board_wakeup_gpio_size; i++)
		enable_irq_wake(OMAP_GPIO_IRQ(sec_board_wakeup_gpio_ptr[i]));

	return ret;
}
