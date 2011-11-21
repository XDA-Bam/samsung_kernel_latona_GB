/**
 * arch/arm/mach-omap2/include/mach/sec_log_buf.h
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
 * File Name : sec_log_buf.h
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 04/Apr/2011
 * Version : Baby-Raccoon
 */

#ifndef __SEC_LOG_BUF_H__
#define __SEC_LOG_BUF_H__

struct sec_log_buf {
	unsigned int *flag;
	unsigned int *count;
	char *data;
};

void sec_log_buf_init(void);

void __init sec_log_buf_reserve_mem(void);

#if defined (CONFIG_ARCH_OMAP3)
#define SEC_LOG_BUF_FLAG_SIZE		(4 * 1024)
#define SEC_LOG_BUF_DATA_SIZE		(1 << CONFIG_LOG_BUF_SHIFT)
#define SEC_LOG_BUF_SIZE		\
	(SEC_LOG_BUF_FLAG_SIZE + SEC_LOG_BUF_DATA_SIZE)
#define SEC_LOG_BUF_START		(0xA0000000 - SEC_LOG_BUF_SIZE)
#define SEC_LOG_BUF_MAGIC		0x404C4F47	/* @LOG */

void __init sec_log_buf_reserve_mem(void);
#endif /* CONFIG_ARCH_OMAP3 */

#endif /* __SEC_LOG_BUF_H__ */
