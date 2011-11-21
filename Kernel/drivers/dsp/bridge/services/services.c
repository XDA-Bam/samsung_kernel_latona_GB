/*
 * services.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Provide SERVICES loading.
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

#include <dspbridge/host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/cfg.h>
#include <dspbridge/ntfy.h>
#include <dspbridge/sync.h>
#include <dspbridge/clk.h>

/*  ----------------------------------- This */
#include <dspbridge/services.h>

/*
 *  ======== services_exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 */
void services_exit(void)
{
	/* Uninitialize all SERVICES modules here */
	clk_exit();
	cfg_exit();
}

/*
 *  ======== services_init ========
 *  Purpose:
 *      Initializes SERVICES modules.
 */
bool services_init(void)
{
	bool ret = true;
	bool fcfg;
	bool fclk;

	/* Perform required initialization of SERVICES modules. */
	fcfg = cfg_init();
	fclk = services_clk_init();

	ret = fcfg && fclk;

	if (!ret) {

		if (fclk)
			clk_exit();

		if (fcfg)
			cfg_exit();
	}

	return ret;
}
