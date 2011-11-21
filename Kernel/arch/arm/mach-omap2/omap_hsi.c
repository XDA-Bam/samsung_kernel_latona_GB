/*
 * arch/arm/mach-omap2/hsi.c
 *
 * HSI device definition
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Author: Sebastien JAN <s-jan@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/notifier.h>
#include <mach/omap_hsi.h>
#include <plat/omap_hwmod.h>
#include <plat/omap_device.h>
#include <plat/control.h>
#include <linux/hsi_driver_if.h>
#include "clock.h"
#include "mux.h"
#include <asm/clkdev.h>
#include <../drivers/staging/omap_hsi/hsi_driver.h>

#define OMAP_HSI_PLATFORM_DEVICE_DRIVER_NAME	"omap_hsi"
#define OMAP_HSI_PLATFORM_DEVICE_NAME		"omap_hsi.0"
#define OMAP_HSI_HWMOD_NAME			"hsi"
#define OMAP_HSI_HWMOD_CLASSNAME		"hsi"
#define OMAP_HSI_PADCONF_CAWAKE_PIN		"usbb1_ulpitll_clk.hsi1_cawake"



/*
 * NOTE: We abuse a little bit the struct port_ctx to use it also for
 * initialization.
 */


static struct port_ctx hsi_port_ctx[] = {
	[0] = {
	       .hst.mode = HSI_MODE_FRAME,
	       .hst.flow = HSI_FLOW_SYNCHRONIZED,
	       .hst.frame_size = HSI_FRAMESIZE_DEFAULT,
	       .hst.divisor = HSI_DIVISOR_DEFAULT,
	       .hst.channels = HSI_CHANNELS_DEFAULT,
	       .hst.arb_mode = HSI_ARBMODE_ROUNDROBIN,
	       .hsr.mode = HSI_MODE_FRAME,
	       .hsr.flow = HSI_FLOW_SYNCHRONIZED,
	       .hsr.frame_size = HSI_FRAMESIZE_DEFAULT,
	       .hsr.channels = HSI_CHANNELS_DEFAULT,
	       .hsr.divisor = HSI_DIVISOR_DEFAULT,
	       .hsr.counters = HSI_COUNTERS_FT_DEFAULT |
			       HSI_COUNTERS_TB_DEFAULT |
			       HSI_COUNTERS_FB_DEFAULT,
	       },
};

static struct ctrl_ctx hsi_ctx = {
		.sysconfig = 0,
		.gdd_gcr = 0,
		.dll = 0,
		.pctx = hsi_port_ctx,
};

static struct hsi_platform_data omap_hsi_platform_data = {
	.num_ports = ARRAY_SIZE(hsi_port_ctx),
	.hsi_gdd_chan_count = HSI_HSI_DMA_CHANNEL_MAX,
	.default_hsi_fclk = HSI_DEFAULT_FCLK,
	.ctx = &hsi_ctx,
	.device_enable = omap_device_enable,
	.device_idle = omap_device_idle,
	.device_shutdown = omap_device_shutdown,
};


static struct platform_device *hsi_get_hsi_platform_device(void)
{
	struct device *dev;
	struct platform_device *pdev;

	/* HSI_TODO: handle platform device id (or port) (0/1) */
	dev = bus_find_device_by_name(&platform_bus_type, NULL,
					OMAP_HSI_PLATFORM_DEVICE_NAME);
	if (!dev) {
		pr_debug("Could not find platform device %s\n",
		       OMAP_HSI_PLATFORM_DEVICE_NAME);
		return 0;
	}

	if (!dev->driver) {
		/* Could not find driver for platform device. */
		return 0;
	}

	pdev = to_platform_device(dev);

	return pdev;
}

static struct hsi_dev *hsi_get_hsi_controller_data(struct platform_device *pd)
{
	struct hsi_dev *hsi_ctrl;

	if (!pd)
		return 0;

	hsi_ctrl = (struct hsi_dev *) platform_get_drvdata(pd);
	if (!hsi_ctrl) {
		pr_err("Could not find HSI controller data\n");
		return 0;
	}

	return hsi_ctrl;
}

/* Note : for hsi_idle_hwmod() and hsi_enable_hwmod() :*/
/* we should normally use omap_hwmod_enable(), but this */
/* function contains a mutex lock of the OMAP HWMOD mutex and there */
/* is only one HWMOD mutex shared for the whole HWMOD table. */
/* This is not compatible with the way HSI driver has to enable the */
/* clocks (from atomic context), as the mutex can very likely be */
/* locked by another HWMOD user. Thus we bypass the mutex usage. */
/* The global mutex has been replaced by a separate mutex per HWMOD */
/* entry, then on 2.6.38 by a separate spinlock. */
/**
* hsi_idle_hwmod - This function is a used to workaround the omap_hwmod layer
*			which might sleep when omap_hwmod_idle() is called,
*			and thus cannot be called from atomic context.
*
* @od - reference to the hsi omap_device.
*
* Note : a "normal" .deactivate_func shall be omap_device_idle_hwmods()
*/
static int hsi_idle_hwmod(struct omap_device *od)
{
	/* HSI omap_device only contain one od->hwmods[0], so no need to */
	/* loop for all hwmods */
	_omap_hwmod_idle(od->hwmods[0]);
	return 0;
}

/**
* hsi_enable_hwmod - This function is a used to workaround the omap_hwmod layer
*			which might sleep when omap_hwmod_enable() is called,
*			and thus cannot be called from atomic context.
*
* @od - reference to the hsi omap_device.
*
* Note : a "normal" .activate_func shall be omap_device_enable_hwmods()
*/
static int hsi_enable_hwmod(struct omap_device *od)
{
	/* HSI omap_device only contain one od->hwmods[0], so no need to */
	/* loop for all hwmods */
	_omap_hwmod_enable(od->hwmods[0]);
	return 0;
}

/**
* omap_hsi_prepare_suspend - Prepare HSI for suspend mode (OFF)
*
* Return value : -ENODEV if HSI controller has not been found, else 0.
*
*/
int omap_hsi_prepare_suspend(void)
{
	struct platform_device *pdev;
	struct hsi_dev *hsi_ctrl;

	pdev = hsi_get_hsi_platform_device();
	hsi_ctrl = hsi_get_hsi_controller_data(pdev);

	if (!hsi_ctrl)
		return -ENODEV;

	if (!hsi_ctrl->clock_enabled)
		return 0;

	if (device_may_wakeup(&pdev->dev))
		omap_mux_enable_wakeup(OMAP_HSI_PADCONF_CAWAKE_PIN);
	else
		omap_mux_disable_wakeup(OMAP_HSI_PADCONF_CAWAKE_PIN);

	return 0;
}

/**
* omap_hsi_prepare_idle - Prepare HSI for idle to low power
*
* Return value : -ENODEV if HSI controller has not been found, else 0.
*
*/
int omap_hsi_prepare_idle(void)
{
	struct platform_device *pdev;
	struct hsi_dev *hsi_ctrl;

	pdev = hsi_get_hsi_platform_device();
	hsi_ctrl = hsi_get_hsi_controller_data(pdev);

	if (!hsi_ctrl)
		return -ENODEV;

	if (!hsi_ctrl->clock_enabled)
		return 0;

	/* If hsi_clocks_disable_channel() is used, it prevents board to */
	/* enter sleep, due to the checks of HSI controller status. */
	/* This is why we call directly the omap_device_xxx() function here */
	hsi_runtime_suspend(hsi_ctrl->dev);
	omap_device_idle(pdev);

	return 0;
}


/**
* omap_hsi_resume_idle - Prepare HSI for wakeup from low power
*
* Return value :* -ENODEV if HSI platform device or HSI controller or CAWAKE
*		  Padconf has not been found
*		* -EPERM if HSI is not allowed to wakeup the platform.
*		* else 0.
*
*/
int omap_hsi_resume_idle(void)
{
	struct platform_device *pdev;
	struct hsi_dev *hsi_ctrl;
	u16 val;

	pdev = hsi_get_hsi_platform_device();
	if (!pdev)
		return -ENODEV;
	if (!device_may_wakeup(&pdev->dev))
		return -EPERM;

	hsi_ctrl = hsi_get_hsi_controller_data(pdev);
	if (!hsi_ctrl)
		return -ENODEV;

	/* Check for IO pad wakeup */
	val = omap_mux_read_signal(OMAP_HSI_PADCONF_CAWAKE_PIN);

	if (val == -ENODEV)
		return val;

	if (val & OMAP44XX_PADCONF_WAKEUPEVENT0) {
		dev_dbg(hsi_ctrl->dev, "HSI WAKEUP DETECTED from PADCONF : "
				       "0x%04x\n", val);

		if (!hsi_ctrl->clock_enabled) {
			/* CAWAKE falling or rising edge detected */
			hsi_ctrl->hsi_port->cawake_off_event = true;
			tasklet_hi_schedule(&hsi_ctrl->hsi_port->hsi_tasklet);

			/* Disable interrupt until Bottom Half has cleared */
			/* the IRQ status register */
			disable_irq_nosync(hsi_ctrl->hsi_port->irq);
		}
	}

	return 0;
}

/* HSI_TODO : This requires some fine tuning & completion of
 * activate/deactivate latency values
 */
static struct omap_device_pm_latency omap_hsi_latency[] = {
	[0] = {
	       .deactivate_func = hsi_idle_hwmod,
	       .activate_func = hsi_enable_hwmod,
	       .flags = OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	       },
};

/* HSI device registration */
static int __init omap_hsi_init(struct omap_hwmod *oh, void *user)
{
	struct omap_device *od;
	struct hsi_platform_data *pdata = &omap_hsi_platform_data;

	if (!oh) {
		pr_err("Could not look up %s omap_hwmod\n",
		       OMAP_HSI_HWMOD_NAME);
		return -EEXIST;
	}

	od = omap_device_build(OMAP_HSI_PLATFORM_DEVICE_DRIVER_NAME, 0, oh,
			       pdata, sizeof(*pdata), omap_hsi_latency,
			       ARRAY_SIZE(omap_hsi_latency), false);
	WARN(IS_ERR(od), "Can't build omap_device for %s:%s.\n",
	     OMAP_HSI_PLATFORM_DEVICE_DRIVER_NAME, oh->name);

	pr_info("HSI: device registered as omap_hwmod: %s\n", oh->name);
	return 0;
}

/* HSI devices registration */
static int __init omap_init_hsi(void)
{
	/* Keep this for genericity, although there is only one hwmod for HSI */
	return omap_hwmod_for_each_by_class(OMAP_HSI_HWMOD_CLASSNAME,
					    omap_hsi_init, NULL);
}

/* HSI_TODO : maybe change the prio, eg. use arch_initcall() */
subsys_initcall(omap_init_hsi);
