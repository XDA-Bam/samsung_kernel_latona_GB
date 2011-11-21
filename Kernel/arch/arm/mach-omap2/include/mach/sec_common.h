/**
 * arch/arm/mach-omap2/include/mach/sec_common.h
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
 * File Name : sec_common.h
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 11/Mar/2011
 * Version : Baby-Raccoon
 */

#ifndef __SEC_COMMON_H__
#define __SEC_COMMON_H__

#define REBOOTMODE_NORMAL 		(1 << 0)
#define REBOOTMODE_RECOVERY 		(1 << 1)
#define REBOOTMODE_FOTA 		(1 << 2)
#ifdef CONFIG_SAMSUNG_KERNEL_DEBUG
#define REBOOTMODE_KERNEL_PANIC		(1 << 3)
#endif /* CONFIG_SAMSUNG_KERNEL_DEBUG */
#define REBOOTMODE_SHUTDOWN		(1 << 4)
#define REBOOTMODE_DOWNLOAD             (1 << 5)
#ifdef CONFIG_SAMSUNG_KERNEL_DEBUG
#define REBOOTMODE_USER_PANIC 		(1 << 6)
#define REBOOTMODE_CP_CRASH		(1 << 9)
#define REBOOTMODE_FORCED_UPLOAD	(1 << 10)
#endif /* CONFIG_SAMSUNG_KERNEL_DEBUG */

/* REBOOT_MODE */
#define REBOOT_MODE_NONE		0
#define REBOOT_MODE_FACTORYTEST		1
#define REBOOT_MODE_RECOVERY		2
#define REBOOT_MODE_ARM11_FOTA		3
#define REBOOT_MODE_DOWNLOAD		4
#define REBOOT_MODE_CHARGING		5
#define REBOOT_MODE_ARM9_FOTA		6
#define REBOOT_MODE_CP_CRASH		7

int sec_common_init_early(void);

int sec_common_init(void);

int sec_common_init_post(void);

int sec_common_update_reboot_reason(char mode, const char *cmd);

#endif /* __SEC_COMMON_H__ */
