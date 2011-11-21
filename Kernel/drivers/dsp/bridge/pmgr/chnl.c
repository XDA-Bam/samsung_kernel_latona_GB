/*
 * chnl.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * WCD channel interface: multiplexes data streams through the single
 * physical link managed by a Bridge mini-driver.
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

/*  ----------------------------------- Host OS */
#include <dspbridge/host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/cfg.h>
#include <dspbridge/sync.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/proc.h>
#include <dspbridge/dev.h>

/*  ----------------------------------- Others */
#include <dspbridge/chnlpriv.h>
#include <chnlobj.h>

/*  ----------------------------------- This */
#include <dspbridge/chnl.h>

/*  ----------------------------------- Globals */
static u32 refs;

/*
 *  ======== chnl_create ========
 *  Purpose:
 *      Create a channel manager object, responsible for opening new channels
 *      and closing old ones for a given 'Bridge board.
 */
int chnl_create(OUT struct chnl_mgr **phChnlMgr,
		       struct dev_object *hdev_obj,
		       IN CONST struct chnl_mgrattrs *pMgrAttrs)
{
	int status;
	struct chnl_mgr *hchnl_mgr;
	struct chnl_mgr_ *chnl_mgr_obj = NULL;

	DBC_REQUIRE(refs > 0);
	DBC_REQUIRE(phChnlMgr != NULL);
	DBC_REQUIRE(pMgrAttrs != NULL);

	*phChnlMgr = NULL;

	/* Validate args: */
	if ((0 < pMgrAttrs->max_channels) &&
	    (pMgrAttrs->max_channels <= CHNL_MAXCHANNELS))
		status = 0;
	else if (pMgrAttrs->max_channels == 0)
		status = -EINVAL;
	else
		status = -ECHRNG;

	if (pMgrAttrs->word_size == 0)
		status = -EINVAL;

	if (DSP_SUCCEEDED(status)) {
		status = dev_get_chnl_mgr(hdev_obj, &hchnl_mgr);
		if (DSP_SUCCEEDED(status) && hchnl_mgr != NULL)
			status = -EEXIST;

	}

	if (DSP_SUCCEEDED(status)) {
		struct bridge_drv_interface *intf_fxns;
		dev_get_intf_fxns(hdev_obj, &intf_fxns);
		if (intf_fxns) {
			/* Let WMD channel module finish the create */
			status = (*intf_fxns->pfn_chnl_create)(&hchnl_mgr,
						hdev_obj, pMgrAttrs);
		}
		if (DSP_SUCCEEDED(status)) {
			/* Fill in WCD channel module's fields of the
			 * chnl_mgr structure */
			chnl_mgr_obj = (struct chnl_mgr_ *)hchnl_mgr;
			chnl_mgr_obj->intf_fxns = intf_fxns;
			/* Finally, return the new channel manager handle: */
			*phChnlMgr = hchnl_mgr;
		}
	}

	DBC_ENSURE(DSP_FAILED(status) || chnl_mgr_obj);

	return status;
}

/*
 *  ======== chnl_destroy ========
 *  Purpose:
 *      Close all open channels, and destroy the channel manager.
 */
int chnl_destroy(struct chnl_mgr *hchnl_mgr)
{
	struct chnl_mgr_ *chnl_mgr_obj = (struct chnl_mgr_ *)hchnl_mgr;
	struct bridge_drv_interface *intf_fxns;
	int status;

	DBC_REQUIRE(refs > 0);

	if (chnl_mgr_obj) {
		intf_fxns = chnl_mgr_obj->intf_fxns;
		/* Let WMD channel module destroy the chnl_mgr: */
		status = (*intf_fxns->pfn_chnl_destroy) (hchnl_mgr);
	} else {
		status = -EFAULT;
	}

	return status;
}

/*
 *  ======== chnl_exit ========
 *  Purpose:
 *      Discontinue usage of the CHNL module.
 */
void chnl_exit(void)
{
	DBC_REQUIRE(refs > 0);

	refs--;

	DBC_ENSURE(refs >= 0);
}

/*
 *  ======== chnl_init ========
 *  Purpose:
 *      Initialize the CHNL module's private state.
 */
bool chnl_init(void)
{
	bool ret = true;

	DBC_REQUIRE(refs >= 0);

	if (ret)
		refs++;

	DBC_ENSURE((ret && (refs > 0)) || (!ret && (refs >= 0)));

	return ret;
}
