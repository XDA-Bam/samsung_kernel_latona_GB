/*
 * cfg.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Implementation of platform specific config services.
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
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>

/*  ----------------------------------- OS Adaptation Layer */

/*  ----------------------------------- This */
#include <dspbridge/cfg.h>
#include <dspbridge/drv.h>

struct drv_ext {
	struct list_head link;
	char sz_string[MAXREGPATHLENGTH];
};

/*
 *  ======== cfg_exit ========
 *  Purpose:
 *      Discontinue usage of the CFG module.
 */
void cfg_exit(void)
{
	/* Do nothing */
}

/*
 *  ======== cfg_get_auto_start ========
 *  Purpose:
 *      Retreive the autostart mask, if any, for this board.
 */
int cfg_get_auto_start(struct cfg_devnode *dev_node_obj,
			      OUT u32 *pdwAutoStart)
{
	int status = 0;
	u32 dw_buf_size;
	struct drv_data *drv_datap = dev_get_drvdata(bridge);

	dw_buf_size = sizeof(*pdwAutoStart);
	if (!dev_node_obj)
		status = -EFAULT;
	if (!pdwAutoStart || !drv_datap)
		status = -EFAULT;
	if (DSP_SUCCEEDED(status))
		*pdwAutoStart = (drv_datap->base_img) ? 1 : 0;

	DBC_ENSURE((status == 0 &&
		    (*pdwAutoStart == 0 || *pdwAutoStart == 1))
		   || status != 0);
	return status;
}

/*
 *  ======== cfg_get_dev_object ========
 *  Purpose:
 *      Retrieve the Device Object handle for a given devnode.
 */
int cfg_get_dev_object(struct cfg_devnode *dev_node_obj,
			      OUT u32 *pdwValue)
{
	int status = 0;
	u32 dw_buf_size;
	struct drv_data *drv_datap = dev_get_drvdata(bridge);

	if (!drv_datap)
		status = -EPERM;

	if (!dev_node_obj)
		status = -EFAULT;

	if (!pdwValue)
		status = -EFAULT;

	dw_buf_size = sizeof(pdwValue);
	if (DSP_SUCCEEDED(status)) {

		/* check the device string and then store dev object */
		if (!
		    (strcmp
		     ((char *)((struct drv_ext *)dev_node_obj)->sz_string,
		      "TIOMAP1510")))
			*pdwValue = (u32)drv_datap->dev_object;
	}
	if (DSP_FAILED(status))
		pr_err("%s: Failed, status %i\n", __func__, status);
	return status;
}

/*
 *  ======== cfg_get_exec_file ========
 *  Purpose:
 *      Retreive the default executable, if any, for this board.
 */
int cfg_get_exec_file(struct cfg_devnode *dev_node_obj, u32 ul_buf_size,
			     OUT char *pstrExecFile)
{
	int status = 0;
	struct drv_data *drv_datap = dev_get_drvdata(bridge);

	if (!dev_node_obj)
		status = -EFAULT;
	else if (!pstrExecFile || !drv_datap)
		status = -EFAULT;

	if (DSP_FAILED(status))
		goto func_end;

	if (!drv_datap->base_img || strlen(drv_datap->base_img) > ul_buf_size)
		status = -EINVAL;

	if (DSP_SUCCEEDED(status))
		strcpy(pstrExecFile, drv_datap->base_img);

	if (DSP_FAILED(status))
		pr_err("%s: Failed, status %i\n", __func__, status);
func_end:
	DBC_ENSURE(((status == 0) &&
		    (strlen(pstrExecFile) <= ul_buf_size))
		   || (status != 0));
	return status;
}

/*
 *  ======== cfg_get_object ========
 *  Purpose:
 *      Retrieve the Object handle from the Registry
 */
int cfg_get_object(OUT u32 *pdwValue, u8 dw_type)
{
	int status = -EINVAL;
	struct drv_data *drv_datap = dev_get_drvdata(bridge);

	DBC_REQUIRE(pdwValue != NULL);

	if (!drv_datap)
		return -EPERM;

	switch (dw_type) {
	case (REG_DRV_OBJECT):
		if (drv_datap->drv_object) {
			*pdwValue = (u32)drv_datap->drv_object;
			status = 0;
		} else {
			status = -ENODATA;
		}
		break;
	case (REG_MGR_OBJECT):
		if (drv_datap->mgr_object) {
			*pdwValue = (u32)drv_datap->mgr_object;
			status = 0;
		} else {
			status = -ENODATA;
		}
		break;

	default:
		break;
	}
	if (DSP_FAILED(status)) {
		*pdwValue = 0;
		pr_err("%s: Failed, status %i\n", __func__, status);
	}
	DBC_ENSURE((DSP_SUCCEEDED(status) && *pdwValue != 0) ||
		   (DSP_FAILED(status) && *pdwValue == 0));
	return status;
}

/*
 *  ======== cfg_init ========
 *  Purpose:
 *      Initialize the CFG module's private state.
 */
bool cfg_init(void)
{
	return true;
}

/*
 *  ======== cfg_set_dev_object ========
 *  Purpose:
 *      Store the Device Object handle and dev_node pointer for a given devnode.
 */
int cfg_set_dev_object(struct cfg_devnode *dev_node_obj, u32 dwValue)
{
	int status = 0;
	struct drv_data *drv_datap = dev_get_drvdata(bridge);

	if (!drv_datap) {
		pr_err("%s: Failed, status %i\n", __func__, status);
		return -EPERM;
	}

	if (!dev_node_obj)
		status = -EFAULT;

	if (DSP_SUCCEEDED(status)) {
		/* Store the WCD device object in the Registry */

		if (!(strcmp((char *)dev_node_obj, "TIOMAP1510")))
			drv_datap->dev_object = (void *) dwValue;
	}
	if (DSP_FAILED(status))
		pr_err("%s: Failed, status %i\n", __func__, status);

	return status;
}

/*
 *  ======== cfg_set_object ========
 *  Purpose:
 *      Store the Driver Object handle
 */
int cfg_set_object(u32 dwValue, u8 dw_type)
{
	int status = -EINVAL;
	struct drv_data *drv_datap = dev_get_drvdata(bridge);

	if (!drv_datap)
		return -EPERM;

	switch (dw_type) {
	case (REG_DRV_OBJECT):
		drv_datap->drv_object = (void *)dwValue;
		status = 0;
		break;
	case (REG_MGR_OBJECT):
		drv_datap->mgr_object = (void *)dwValue;
		status = 0;
		break;
	default:
		break;
	}
	if (DSP_FAILED(status))
		pr_err("%s: Failed, status %i\n", __func__, status);
	return status;
}
