/*
 * wmdmsg.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Declares the upper edge message class library functions required by
 * all WMD / WCD driver interface tables.  These functions are
 * implemented by every class of WMD channel library.
 *
 * Notes:
 *   Function comment headers reside in wmd.h.
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

#ifndef WMDMSG_
#define WMDMSG_

#include <dspbridge/msgdefs.h>

extern int bridge_msg_create(OUT struct msg_mgr **phMsgMgr,
				    struct dev_object *hdev_obj,
				    msg_onexit msgCallback);

extern int bridge_msg_create_queue(struct msg_mgr *hmsg_mgr,
				       OUT struct msg_queue **phMsgQueue,
				       u32 msgq_id, u32 max_msgs, bhandle h);

extern void bridge_msg_delete(struct msg_mgr *hmsg_mgr);

extern void bridge_msg_delete_queue(struct msg_queue *msg_queue_obj);

extern int bridge_msg_get(struct msg_queue *msg_queue_obj,
				 struct dsp_msg *pmsg, u32 utimeout);

extern int bridge_msg_put(struct msg_queue *msg_queue_obj,
				 IN CONST struct dsp_msg *pmsg, u32 utimeout);

extern int bridge_msg_register_notify(struct msg_queue *msg_queue_obj,
					  u32 event_mask,
					  u32 notify_type,
					  struct dsp_notification
					  *hnotification);

extern void bridge_msg_set_queue_id(struct msg_queue *msg_queue_obj,
				    u32 msgq_id);

#endif /* WMDMSG_ */
