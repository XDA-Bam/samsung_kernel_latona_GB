/*
 * io_sm.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * IO dispatcher for a shared memory channel driver.
 * Also, includes macros to simulate shm via port io calls.
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

#ifndef IOSM_
#define IOSM_

#include <dspbridge/_chnl_sm.h>
#include <dspbridge/host_os.h>

#include <dspbridge/iodefs.h>

#define IO_INPUT            0
#define IO_OUTPUT           1
#define IO_SERVICE          2
#define IO_MAXSERVICE       IO_SERVICE

#define DSP_FIELD_ADDR(type, field, base, wordsize) \
    ((((s32)&(((type *)0)->field)) / wordsize) + (u32)base)

/* Access can be different SM access word size (e.g. 16/32 bit words) */
#define IO_SET_VALUE(pContext, type, base, field, value) (base->field = value)
#define IO_GET_VALUE(pContext, type, base, field)	(base->field)
#define IO_OR_VALUE(pContext, type, base, field, value)  (base->field |= value)
#define IO_AND_VALUE(pContext, type, base, field, value) (base->field &= value)
#define IO_SET_LONG(pContext, type, base, field, value)  (base->field = value)
#define IO_GET_LONG(pContext, type, base, field)         (base->field)

/*
 *  ======== io_cancel_chnl ========
 *  Purpose:
 *      Cancel IO on a given channel.
 *  Parameters:
 *      hio_mgr:     IO Manager.
 *      ulChnl:     Index of channel to cancel IO on.
 *  Returns:
 *  Requires:
 *      Valid hio_mgr.
 *  Ensures:
 */
extern void io_cancel_chnl(struct io_mgr *hio_mgr, u32 ulChnl);

/*
 *  ======== io_dpc ========
 *  Purpose:
 *      Deferred procedure call for shared memory channel driver ISR.  Carries
 *      out the dispatch of I/O.
 *  Parameters:
 *      pRefData:   Pointer to reference data registered via a call to
 *                  DPC_Create().
 *  Returns:
 *  Requires:
 *      Must not block.
 *      Must not acquire resources.
 *      All data touched must be locked in memory if running in kernel mode.
 *  Ensures:
 *      Non-preemptible (but interruptible).
 */
extern void io_dpc(IN OUT unsigned long pRefData);

/*
 *  ======== io_mbox_msg ========
 *  Purpose:
 *      Main interrupt handler for the shared memory WMD channel manager.
 *      Calls the WMD's chnlsm_isr to determine if this interrupt is ours, then
 *      schedules a DPC to dispatch I/O..
 *  Parameters:
 *      pRefData:   Pointer to the channel manager object for this board.
 *                  Set in an initial call to ISR_Install().
 *  Returns:
 *      TRUE if interrupt handled; FALSE otherwise.
 *  Requires:
 *      Must be in locked memory if executing in kernel mode.
 *      Must only call functions which are in locked memory if Kernel mode.
 *      Must only call asynchronous services.
 *      Interrupts are disabled and EOI for this interrupt has been sent.
 *  Ensures:
 */
//void io_mbox_msg(u32 msg);
int io_mbox_msg(struct notifier_block *self, unsigned long len, void *msg);
/*
 *  ======== io_request_chnl ========
 *  Purpose:
 *      Request I/O from the DSP. Sets flags in shared memory, then interrupts
 *      the DSP.
 *  Parameters:
 *      hio_mgr:     IO manager handle.
 *      pchnl:      Ptr to the channel requesting I/O.
 *      iMode:      Mode of channel: {IO_INPUT | IO_OUTPUT}.
 *  Returns:
 *  Requires:
 *      pchnl != NULL
 *  Ensures:
 */

#ifdef CONFIG_BRIDGE_WDT3
/*
 *  ========  io_isr_wdt3 ========
 *  Purpose:
 *         Main interrupt handler for the WDT overflow.
 */
irqreturn_t io_isr_wdt3(int irq, void *data);
void dsp_wdt_enable(bool);
void dsp_wdt_set_timeout(unsigned);
unsigned dsp_wdt_get_timeout(void);
bool dsp_wdt_get_enable(void);
void dsp_wdt_set_enable(bool);
#endif

extern void io_request_chnl(struct io_mgr *hio_mgr,
			    struct chnl_object *pchnl,
			    u8 iMode, OUT u16 *pwMbVal);

/*
 *  ======== iosm_schedule ========
 *  Purpose:
 *      Schedule DPC for IO.
 *  Parameters:
 *      pio_mgr:     Ptr to a I/O manager.
 *  Returns:
 *  Requires:
 *      pchnl != NULL
 *  Ensures:
 */
extern void iosm_schedule(struct io_mgr *hio_mgr);

/*
 * DSP-DMA IO functions
 */

/*
 *  ======== io_ddma_init_chnl_desc ========
 *  Purpose:
 *      Initialize DSP DMA channel descriptor.
 *  Parameters:
 *      hio_mgr:         Handle to a I/O manager.
 *      uDDMAChnlId:    DDMA channel identifier.
 *      uNumDesc:       Number of buffer descriptors(equals # of IOReqs &
 *                      Chirps)
 *      pDsp:           Dsp address;
 *  Returns:
 *  Requires:
 *     uDDMAChnlId < DDMA_MAXDDMACHNLS
 *     uNumDesc > 0
 *     pVa != NULL
 *     pDspPa != NULL
 *
 *  Ensures:
 */
extern void io_ddma_init_chnl_desc(struct io_mgr *hio_mgr, u32 uDDMAChnlId,
				   u32 uNumDesc, void *pDsp);

/*
 *  ======== io_ddma_clear_chnl_desc ========
 *  Purpose:
 *      Clear DSP DMA channel descriptor.
 *  Parameters:
 *      hio_mgr:         Handle to a I/O manager.
 *      uDDMAChnlId:    DDMA channel identifier.
 *  Returns:
 *  Requires:
 *     uDDMAChnlId < DDMA_MAXDDMACHNLS
 *  Ensures:
 */
extern void io_ddma_clear_chnl_desc(struct io_mgr *hio_mgr, u32 uDDMAChnlId);

/*
 *  ======== io_ddma_request_chnl ========
 *  Purpose:
 *      Request channel DSP-DMA from the DSP. Sets up SM descriptors and
 *      control fields in shared memory.
 *  Parameters:
 *      hio_mgr:     Handle to a I/O manager.
 *      pchnl:      Ptr to channel object
 *      chnl_packet_obj:     Ptr to channel i/o request packet.
 *  Returns:
 *  Requires:
 *      pchnl != NULL
 *      pchnl->cio_reqs > 0
 *      chnl_packet_obj != NULL
 *  Ensures:
 */
extern void io_ddma_request_chnl(struct io_mgr *hio_mgr,
				 struct chnl_object *pchnl,
				 struct chnl_irp *chnl_packet_obj,
				 OUT u16 *pwMbVal);

/*
 * Zero-copy IO functions
 */

/*
 *  ======== io_ddzc_init_chnl_desc ========
 *  Purpose:
 *      Initialize ZCPY channel descriptor.
 *  Parameters:
 *      hio_mgr:     Handle to a I/O manager.
 *      uZId:       zero-copy channel identifier.
 *  Returns:
 *  Requires:
 *     uDDMAChnlId < DDMA_MAXZCPYCHNLS
 *     hio_mgr != Null
 *  Ensures:
 */
extern void io_ddzc_init_chnl_desc(struct io_mgr *hio_mgr, u32 uZId);

/*
 *  ======== io_ddzc_clear_chnl_desc ========
 *  Purpose:
 *      Clear DSP ZC channel descriptor.
 *  Parameters:
 *      hio_mgr:         Handle to a I/O manager.
 *      uChnlId:        ZC channel identifier.
 *  Returns:
 *  Requires:
 *      hio_mgr is valid
 *      uChnlId < DDMA_MAXZCPYCHNLS
 *  Ensures:
 */
extern void io_ddzc_clear_chnl_desc(struct io_mgr *hio_mgr, u32 uChnlId);

/*
 *  ======== io_ddzc_request_chnl ========
 *  Purpose:
 *      Request zero-copy channel transfer. Sets up SM descriptors and
 *      control fields in shared memory.
 *  Parameters:
 *      hio_mgr:         Handle to a I/O manager.
 *      pchnl:          Ptr to channel object
 *      chnl_packet_obj:         Ptr to channel i/o request packet.
 *  Returns:
 *  Requires:
 *      pchnl != NULL
 *      pchnl->cio_reqs > 0
 *      chnl_packet_obj != NULL
 *  Ensures:
 */
extern void io_ddzc_request_chnl(struct io_mgr *hio_mgr,
				 struct chnl_object *pchnl,
				 struct chnl_irp *chnl_packet_obj,
				 OUT u16 *pwMbVal);

/*
 *  ======== io_sh_msetting ========
 *  Purpose:
 *      Sets the shared memory setting
 *  Parameters:
 *      hio_mgr:         Handle to a I/O manager.
 *      desc:             Shared memory type
 *      pargs:          Ptr to shm setting
 *  Returns:
 *  Requires:
 *      hio_mgr != NULL
 *      pargs != NULL
 *  Ensures:
 */
extern int io_sh_msetting(struct io_mgr *hio_mgr, u8 desc, void *pargs);

/*
 *  Misc functions for the CHNL_IO shared memory library:
 */

/* Maximum channel bufsize that can be used. */
extern u32 io_buf_size(struct io_mgr *hio_mgr);

extern u32 io_read_value(struct wmd_dev_context *hDevContext, u32 dwDSPAddr);

extern void io_write_value(struct wmd_dev_context *hDevContext,
			   u32 dwDSPAddr, u32 dwValue);

extern u32 io_read_value_long(struct wmd_dev_context *hDevContext,
			      u32 dwDSPAddr);

extern void io_write_value_long(struct wmd_dev_context *hDevContext,
				u32 dwDSPAddr, u32 dwValue);

extern void io_or_set_value(struct wmd_dev_context *hDevContext,
			    u32 dwDSPAddr, u32 dwValue);

extern void io_and_set_value(struct wmd_dev_context *hDevContext,
			     u32 dwDSPAddr, u32 dwValue);

extern void io_intr_dsp2(IN struct io_mgr *pio_mgr, IN u16 mb_val);

extern void io_sm_init(void);

/*
 *  ========print_dsp_trace_buffer ========
 *      Print DSP tracebuffer.
 */
extern int print_dsp_trace_buffer(struct wmd_dev_context
					 *hwmd_context);

int dump_dsp_stack(struct wmd_dev_context *wmd_context);

void dump_dl_modules(struct wmd_dev_context *wmd_context);


#endif /* IOSM_ */
