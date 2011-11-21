/*
 * chnl.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * WCD channel interface: multiplexes data streams through the single
 * physical link managed by a mini-driver.
 *
 * See DSP API chnl.h for more details.
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

#ifndef CHNL_
#define CHNL_

#include <dspbridge/chnlpriv.h>

/*
 *  ======== chnl_close ========
 *  Purpose:
 *      Ensures all pending I/O on this channel is cancelled, discards all
 *      queued I/O completion notifications, then frees the resources allocated
 *      for this channel, and makes the corresponding logical channel id
 *      available for subsequent use.
 *  Parameters:
 *      chnl_obj:          Channel object handle.
 *  Returns:
 *      0:        Success;
 *      -EFAULT:    Invalid chnl_obj.
 *  Requires:
 *      chnl_init(void) called.
 *      No thread must be blocked on this channel's I/O completion event.
 *  Ensures:
 *      0:        The I/O completion event for this channel is freed.
 *                      chnl_obj is no longer valid.
 */
extern int chnl_close(struct chnl_object *chnl_obj);

/*
 *  ======== chnl_create ========
 *  Purpose:
 *      Create a channel manager object, responsible for opening new channels
 *      and closing old ones for a given board.
 *  Parameters:
 *      phChnlMgr:      Location to store a channel manager object on output.
 *      hdev_obj:     Handle to a device object.
 *      pMgrAttrs:      Channel manager attributes.
 *      pMgrAttrs->max_channels:   Max channels
 *      pMgrAttrs->birq:        Channel's I/O IRQ number.
 *      pMgrAttrs->irq_shared:     TRUE if the IRQ is shareable.
 *      pMgrAttrs->word_size:   DSP Word size in equivalent PC bytes..
 *  Returns:
 *      0:                Success;
 *      -EFAULT:            hdev_obj is invalid.
 *      -EINVAL:        max_channels is 0.
 *      -ENOMEM:            Insufficient memory for requested resources.
 *      -EIO:             Unable to plug channel ISR for configured IRQ.
 *      -ECHRNG:     This manager cannot handle this many channels.
 *      CHNL_E_INVALIDIRQ:      Invalid IRQ number. Must be 0 <= birq <= 15.
 *      -EINVAL: Invalid DSP word size.  Must be > 0.
 *      -EINVAL:  Invalid base address for DSP communications.
 *      -EEXIST:       Channel manager already exists for this device.
 *  Requires:
 *      chnl_init(void) called.
 *      phChnlMgr != NULL.
 *      pMgrAttrs != NULL.
 *  Ensures:
 *      0:                Subsequent calls to chnl_create() for the same
 *                              board without an intervening call to
 *                              chnl_destroy() will fail.
 */
extern int chnl_create(OUT struct chnl_mgr **phChnlMgr,
			      struct dev_object *hdev_obj,
			      IN CONST struct chnl_mgrattrs *pMgrAttrs);

/*
 *  ======== chnl_destroy ========
 *  Purpose:
 *      Close all open channels, and destroy the channel manager.
 *  Parameters:
 *      hchnl_mgr:           Channel manager object.
 *  Returns:
 *      0:            Success.
 *      -EFAULT:        hchnl_mgr was invalid.
 *  Requires:
 *      chnl_init(void) called.
 *  Ensures:
 *      0:            Cancels I/O on each open channel.
 *                          Closes each open channel.
 *                          chnl_create may subsequently be called for the
 *                          same board.
 */
extern int chnl_destroy(struct chnl_mgr *hchnl_mgr);

/*
 *  ======== chnl_exit ========
 *  Purpose:
 *      Discontinue usage of the CHNL module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      chnl_init(void) previously called.
 *  Ensures:
 *      Resources, if any acquired in chnl_init(void), are freed when the last
 *      client of CHNL calls chnl_exit(void).
 */
extern void chnl_exit(void);

/*
 *  ======== chnl_init ========
 *  Purpose:
 *      Initialize the CHNL module's private state.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occurred.
 *  Requires:
 *  Ensures:
 *      A requirement for each of the other public CHNL functions.
 */
extern bool chnl_init(void);

#endif /* CHNL_ */
