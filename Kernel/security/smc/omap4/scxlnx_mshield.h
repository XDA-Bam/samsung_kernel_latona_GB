/**
 * Copyright (c) 2010 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __SCXLNX_MSHIELD_H__
#define __SCXLNX_MSHIELD_H__

#include "scxlnx_defs.h"

int SCXLNXCtrlDeviceRegister(void);

int SCXLNXCommStart(struct SCXLNX_COMM *pComm, u32 nSDPBackingStoreAddr,
	u32 nSDPBkExtStoreAddr, u32 nDataAddr, u8 *pPABufferVAddr,
	u32 nPABufferSize, u8 *pPropertiesBuffer,
	u32 nPropertiesBufferLen);

/* Assembler entry points to/from secure */
u32 schedule_secure_world(u32 app_id, u32 proc_id, u32 flags, u32 args);
u32 rpc_handler(u32 p1, u32 p2, u32 p3, u32 p4);

/* L4 SEC clockdomain enabling/disabling */
void SCXL4SECClockDomainEnable(bool use_spin_lock);
void SCXL4SECClockDomainDisable(bool use_spin_lock);

void tf_wake_lock(void);
void tf_wake_unlock(void);

#endif
