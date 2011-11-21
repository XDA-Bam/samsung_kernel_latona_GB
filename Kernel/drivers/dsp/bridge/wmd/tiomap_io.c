/*
 * tiomap_io.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Implementation for the io read/write routines.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/dbdefs.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/dev.h>
#include <dspbridge/drv.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/cfg.h>

/*  ----------------------------------- specific to this file */
#include <dspbridge/io_sm.h>
#include "_tiomap.h"
#include "_tiomap_pwr.h"
#include "tiomap_io.h"

static u32 ul_ext_base;
static u32 ul_ext_end;

static u32 shm0_end;
static u32 ul_dyn_ext_base;
static u32 ul_trace_sec_beg;
static u32 ul_trace_sec_end;
static u32 ul_shm_base_virt;

bool symbols_reloaded = true;

/*
 *  ======== read_ext_dsp_data ========
 *      Copies DSP external memory buffers to the host side buffers.
 */
int read_ext_dsp_data(struct wmd_dev_context *hDevContext,
			     OUT u8 *pbHostBuf, u32 dwDSPAddr,
			     u32 ul_num_bytes, u32 ulMemType)
{
	int status = 0;
	struct wmd_dev_context *dev_context = hDevContext;
	u32 offset;
	u32 ul_tlb_base_virt = 0;
	u32 ul_shm_offset_virt = 0;
	u32 dw_ext_prog_virt_mem;
	u32 dw_base_addr = dev_context->dw_dsp_ext_base_addr;
	bool trace_read = false;

	if (!ul_shm_base_virt) {
		status = dev_get_symbol(dev_context->hdev_obj,
					SHMBASENAME, &ul_shm_base_virt);
	}
	DBC_ASSERT(ul_shm_base_virt != 0);

	/* Check if it is a read of Trace section */
	if (DSP_SUCCEEDED(status) && !ul_trace_sec_beg) {
		status = dev_get_symbol(dev_context->hdev_obj,
					DSP_TRACESEC_BEG, &ul_trace_sec_beg);
	}
	DBC_ASSERT(ul_trace_sec_beg != 0);

	if (DSP_SUCCEEDED(status) && !ul_trace_sec_end) {
		status = dev_get_symbol(dev_context->hdev_obj,
					DSP_TRACESEC_END, &ul_trace_sec_end);
	}
	DBC_ASSERT(ul_trace_sec_end != 0);

	if (DSP_SUCCEEDED(status)) {
		if ((dwDSPAddr <= ul_trace_sec_end) &&
		    (dwDSPAddr >= ul_trace_sec_beg))
			trace_read = true;
	}

	/* If reading from TRACE, force remap/unmap */
	if (trace_read && dw_base_addr) {
		dw_base_addr = 0;
		dev_context->dw_dsp_ext_base_addr = 0;
	}

	if (!dw_base_addr) {
		/* Initialize ul_ext_base and ul_ext_end */
		ul_ext_base = 0;
		ul_ext_end = 0;

		/* Get DYNEXT_BEG, EXT_BEG and EXT_END. */
		if (DSP_SUCCEEDED(status) && !ul_dyn_ext_base) {
			status = dev_get_symbol(dev_context->hdev_obj,
						DYNEXTBASE, &ul_dyn_ext_base);
		}
		DBC_ASSERT(ul_dyn_ext_base != 0);

		if (DSP_SUCCEEDED(status)) {
			status = dev_get_symbol(dev_context->hdev_obj,
						EXTBASE, &ul_ext_base);
		}
		DBC_ASSERT(ul_ext_base != 0);

		if (DSP_SUCCEEDED(status)) {
			status = dev_get_symbol(dev_context->hdev_obj,
						EXTEND, &ul_ext_end);
		}
		DBC_ASSERT(ul_ext_end != 0);

		/* Trace buffer is right after the shm SEG0,
		 *  so set the base address to SHMBASE */
		if (trace_read) {
			ul_ext_base = ul_shm_base_virt;
			ul_ext_end = ul_trace_sec_end;
		}

		DBC_ASSERT(ul_ext_end != 0);
		DBC_ASSERT(ul_ext_end > ul_ext_base);

		if (ul_ext_end < ul_ext_base)
			status = -EPERM;

		if (DSP_SUCCEEDED(status)) {
			ul_tlb_base_virt =
			    dev_context->atlb_entry[0].ul_dsp_va * DSPWORDSIZE;
			DBC_ASSERT(ul_tlb_base_virt <= ul_shm_base_virt);
			dw_ext_prog_virt_mem =
			    dev_context->atlb_entry[0].ul_gpp_va;

			if (!trace_read) {
				ul_shm_offset_virt =
				    ul_shm_base_virt - ul_tlb_base_virt;
				ul_shm_offset_virt +=
				    PG_ALIGN_HIGH(ul_ext_end - ul_dyn_ext_base +
						  1, HW_PAGE_SIZE64KB);
				dw_ext_prog_virt_mem -= ul_shm_offset_virt;
				dw_ext_prog_virt_mem +=
				    (ul_ext_base - ul_dyn_ext_base);
				dev_context->dw_dsp_ext_base_addr =
				    dw_ext_prog_virt_mem;

				/*
				 * This dw_dsp_ext_base_addr will get cleared
				 * only when the board is stopped.
				*/
				if (!dev_context->dw_dsp_ext_base_addr)
					status = -EPERM;
			}

			dw_base_addr = dw_ext_prog_virt_mem;
		}
	}

	if (!dw_base_addr || !ul_ext_base || !ul_ext_end)
		status = -EPERM;

	offset = dwDSPAddr - ul_ext_base;

	if (DSP_SUCCEEDED(status))
		memcpy(pbHostBuf, (u8 *) dw_base_addr + offset, ul_num_bytes);

	return status;
}

/*
 *  ======== write_dsp_data ========
 *  purpose:
 *      Copies buffers to the DSP internal/external memory.
 */
int write_dsp_data(struct wmd_dev_context *hDevContext,
			  IN u8 *pbHostBuf, u32 dwDSPAddr, u32 ul_num_bytes,
			  u32 ulMemType)
{
	u32 offset;
	u32 dw_base_addr = hDevContext->dw_dsp_base_addr;
	struct cfg_hostres *resources = hDevContext->resources;
	int status = 0;
	u32 base1, base2, base3;
	base1 = OMAP_DSP_MEM1_SIZE;
	base2 = OMAP_DSP_MEM2_BASE - OMAP_DSP_MEM1_BASE;
	base3 = OMAP_DSP_MEM3_BASE - OMAP_DSP_MEM1_BASE;

	if (!resources)
		return -EPERM;

	offset = dwDSPAddr - hDevContext->dw_dsp_start_add;
	if (offset < base1) {
		dw_base_addr = MEM_LINEAR_ADDRESS(resources->dw_mem_base[2],
						  resources->dw_mem_length[2]);
	} else if (offset > base1 && offset < base2 + OMAP_DSP_MEM2_SIZE) {
		dw_base_addr = MEM_LINEAR_ADDRESS(resources->dw_mem_base[3],
						  resources->dw_mem_length[3]);
		offset = offset - base2;
	} else if (offset >= base2 + OMAP_DSP_MEM2_SIZE &&
		   offset < base3 + OMAP_DSP_MEM3_SIZE) {
		dw_base_addr = MEM_LINEAR_ADDRESS(resources->dw_mem_base[4],
						  resources->dw_mem_length[4]);
		offset = offset - base3;
	} else {
		return -EPERM;
	}
	if (ul_num_bytes)
		memcpy((u8 *) (dw_base_addr + offset), pbHostBuf, ul_num_bytes);
	else
		*((u32 *) pbHostBuf) = dw_base_addr + offset;

	return status;
}

/*
 *  ======== write_ext_dsp_data ========
 *  purpose:
 *      Copies buffers to the external memory.
 *
 */
int write_ext_dsp_data(struct wmd_dev_context *dev_context,
			      IN u8 *pbHostBuf, u32 dwDSPAddr,
			      u32 ul_num_bytes, u32 ulMemType,
			      bool bDynamicLoad)
{
	u32 dw_base_addr = dev_context->dw_dsp_ext_base_addr;
	u32 dw_offset = 0;
	u8 temp_byte1, temp_byte2;
	u8 remain_byte[4];
	s32 i;
	int ret = 0;
	u32 dw_ext_prog_virt_mem;
	u32 ul_tlb_base_virt = 0;
	u32 ul_shm_offset_virt = 0;
	struct cfg_hostres *host_res = dev_context->resources;
	bool trace_load = false;
	temp_byte1 = 0x0;
	temp_byte2 = 0x0;

	if (symbols_reloaded) {
		/* Check if it is a load to Trace section */
		ret = dev_get_symbol(dev_context->hdev_obj,
				     DSP_TRACESEC_BEG, &ul_trace_sec_beg);
		if (DSP_SUCCEEDED(ret))
			ret = dev_get_symbol(dev_context->hdev_obj,
					     DSP_TRACESEC_END,
					     &ul_trace_sec_end);
	}
	if (DSP_SUCCEEDED(ret)) {
		if ((dwDSPAddr <= ul_trace_sec_end) &&
		    (dwDSPAddr >= ul_trace_sec_beg))
			trace_load = true;
	}

	/* If dynamic, force remap/unmap */
	if ((bDynamicLoad || trace_load) && dw_base_addr) {
		dw_base_addr = 0;
		MEM_UNMAP_LINEAR_ADDRESS((void *)
					 dev_context->dw_dsp_ext_base_addr);
		dev_context->dw_dsp_ext_base_addr = 0x0;
	}
	if (!dw_base_addr) {
		if (symbols_reloaded)
			/* Get SHM_BEG  EXT_BEG and EXT_END. */
			ret = dev_get_symbol(dev_context->hdev_obj,
					     SHMBASENAME, &ul_shm_base_virt);
		DBC_ASSERT(ul_shm_base_virt != 0);
		if (bDynamicLoad) {
			if (DSP_SUCCEEDED(ret)) {
				if (symbols_reloaded)
					ret =
					    dev_get_symbol
					    (dev_context->hdev_obj, DYNEXTBASE,
					     &ul_ext_base);
			}
			DBC_ASSERT(ul_ext_base != 0);
			if (DSP_SUCCEEDED(ret)) {
				/* DR  OMAPS00013235 : DLModules array may be
				 * in EXTMEM. It is expected that DYNEXTMEM and
				 * EXTMEM are contiguous, so checking for the
				 * upper bound at EXTEND should be Ok. */
				if (symbols_reloaded)
					ret =
					    dev_get_symbol
					    (dev_context->hdev_obj, EXTEND,
					     &ul_ext_end);
			}
		} else {
			if (symbols_reloaded) {
				if (DSP_SUCCEEDED(ret))
					ret =
					    dev_get_symbol
					    (dev_context->hdev_obj, EXTBASE,
					     &ul_ext_base);
				DBC_ASSERT(ul_ext_base != 0);
				if (DSP_SUCCEEDED(ret))
					ret =
					    dev_get_symbol
					    (dev_context->hdev_obj, EXTEND,
					     &ul_ext_end);
			}
		}
		/* Trace buffer it right after the shm SEG0, so set the
		 *      base address to SHMBASE */
		if (trace_load)
			ul_ext_base = ul_shm_base_virt;

		DBC_ASSERT(ul_ext_end != 0);
		DBC_ASSERT(ul_ext_end > ul_ext_base);
		if (ul_ext_end < ul_ext_base)
			ret = -EPERM;

		if (DSP_SUCCEEDED(ret)) {
			ul_tlb_base_virt =
			    dev_context->atlb_entry[0].ul_dsp_va * DSPWORDSIZE;
			DBC_ASSERT(ul_tlb_base_virt <= ul_shm_base_virt);

			if (symbols_reloaded) {
				if (DSP_SUCCEEDED(ret)) {
					ret =
					    dev_get_symbol
					    (dev_context->hdev_obj,
					     DSP_TRACESEC_END, &shm0_end);
				}
				if (DSP_SUCCEEDED(ret)) {
					ret =
					    dev_get_symbol
					    (dev_context->hdev_obj, DYNEXTBASE,
					     &ul_dyn_ext_base);
				}
			}
			ul_shm_offset_virt =
			    ul_shm_base_virt - ul_tlb_base_virt;
			if (trace_load) {
				dw_ext_prog_virt_mem =
				    dev_context->atlb_entry[0].ul_gpp_va;
			} else {
				dw_ext_prog_virt_mem = host_res->dw_mem_base[1];
				dw_ext_prog_virt_mem +=
				    (ul_ext_base - ul_dyn_ext_base);
			}

			dev_context->dw_dsp_ext_base_addr =
			    (u32) MEM_LINEAR_ADDRESS((void *)
						     dw_ext_prog_virt_mem,
						     ul_ext_end - ul_ext_base);
			dw_base_addr += dev_context->dw_dsp_ext_base_addr;
			/* This dw_dsp_ext_base_addr will get cleared only when
			 * the board is stopped. */
			if (!dev_context->dw_dsp_ext_base_addr)
				ret = -EPERM;
		}
	}
	if (!dw_base_addr || !ul_ext_base || !ul_ext_end)
		ret = -EPERM;

	if (DSP_SUCCEEDED(ret)) {
		for (i = 0; i < 4; i++)
			remain_byte[i] = 0x0;

		dw_offset = dwDSPAddr - ul_ext_base;
		/* Also make sure the dwDSPAddr is < ul_ext_end */
		if (dwDSPAddr > ul_ext_end || dw_offset > dwDSPAddr)
			ret = -EPERM;
	}
	if (DSP_SUCCEEDED(ret)) {
		if (ul_num_bytes)
			memcpy((u8 *) dw_base_addr + dw_offset, pbHostBuf,
			       ul_num_bytes);
		else
			*((u32 *) pbHostBuf) = dw_base_addr + dw_offset;
	}
	/* Unmap here to force remap for other Ext loads */
	if ((bDynamicLoad || trace_load) && dev_context->dw_dsp_ext_base_addr) {
		MEM_UNMAP_LINEAR_ADDRESS((void *)
					 dev_context->dw_dsp_ext_base_addr);
		dev_context->dw_dsp_ext_base_addr = 0x0;
	}
	symbols_reloaded = false;
	return ret;
}

int sm_interrupt_dsp(struct wmd_dev_context *dev_context, u16 mb_val)
{
	int status = 0;
	struct cfg_hostres *resources;
	u32 temp;
	struct dspbridge_platform_data *pdata =
		omap_dspbridge_dev->dev.platform_data;

	if (!dev_context || !dev_context->resources)
		return -EFAULT;

	if (!dev_context->mbox)
		return status;

	resources = dev_context->resources;
	if (dev_context->dw_brd_state == BRD_DSP_HIBERNATION ||
	    dev_context->dw_brd_state == BRD_HIBERNATION) {
		/* Restart the peripheral clocks */
		dsp_peripheral_clocks_enable(dev_context, NULL);

#ifdef CONFIG_BRIDGE_WDT3
		dsp_wdt_enable(true);
#endif

		/*
		 * 2:0 AUTO_IVA2_DPLL - Enabling IVA2 DPLL auto control
		 *     in CM_AUTOIDLE_PLL_IVA2 register
		 */
		(*pdata->dsp_cm_write)(1 << OMAP3430_AUTO_IVA2_DPLL_SHIFT,
				OMAP3430_IVA2_MOD, OMAP3430_CM_AUTOIDLE_PLL);

		/*
		 * 7:4 IVA2_DPLL_FREQSEL - IVA2 internal frq set to
		 *     0.75 MHz - 1.0 MHz
		 * 2:0 EN_IVA2_DPLL - Enable IVA2 DPLL in lock mode
		 */
		(*pdata->dsp_cm_rmw_bits)(OMAP3430_IVA2_DPLL_FREQSEL_MASK |
				OMAP3430_EN_IVA2_DPLL_MASK,
				0x3 << OMAP3430_IVA2_DPLL_FREQSEL_SHIFT |
				0x7 << OMAP3430_EN_IVA2_DPLL_SHIFT,
				OMAP3430_IVA2_MOD, OMAP3430_CM_CLKEN_PLL);

		/* Restore mailbox settings */
		omap_mbox_restore_ctx(dev_context->mbox);

		/* Access MMU SYS CONFIG register to generate a short wakeup */
		temp = *(reg_uword32 *) (resources->dw_dmmu_base + 0x10);

		dev_context->dw_brd_state = BRD_RUNNING;
	} else if (dev_context->dw_brd_state == BRD_RETENTION) {
		/* Restart the peripheral clocks */
		dsp_peripheral_clocks_enable(dev_context, NULL);
		dev_context->dw_brd_state = BRD_RUNNING;
	}

	status = omap_mbox_msg_send(dev_context->mbox, mb_val);

	if (status) {
		pr_err("omap_mbox_msg_send Fail and status = %d\n", status);
		status = -EPERM;
	}

	dev_dbg(bridge, "MBX: writing %x to Mailbox\n", mb_val);
	return status;
}
