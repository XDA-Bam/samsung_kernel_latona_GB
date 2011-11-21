/*
 * OMAP Smartreflex Defines and Routines
 *
 * Author: Thara Gopinath	<thara@ti.com>
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARM_OMAP_SMARTREFLEX_H
#define __ASM_ARM_OMAP_SMARTREFLEX_H

#include <linux/platform_device.h>
#include <plat/voltage.h>

#ifdef CONFIG_PM_DEBUG
extern struct dentry *sr_dbg_dir;
#endif

/*
 * Different Smartreflex IPs version. The v1 is the 65nm version used in
 * OMAP3430. The v2 is the update for the 45nm version of the IP
 * used in OMAP3630 and OMAP4430
 */
#define SR_TYPE_V1	1
#define SR_TYPE_V2	2

/* SMART REFLEX REG ADDRESS OFFSET */
#define SRCONFIG		0x00
#define SRSTATUS		0x04
#define SENVAL			0x08
#define SENMIN			0x0C
#define SENMAX			0x10
#define SENAVG			0x14
#define AVGWEIGHT		0x18
#define NVALUERECIPROCAL	0x1C
#define SENERROR_V1		0x20
#define ERRCONFIG_V1		0x24
#define IRQ_EOI			0x20
#define IRQSTATUS_RAW		0x24
#define IRQSTATUS		0x28
#define IRQENABLE_SET		0x2C
#define IRQENABLE_CLR		0x30
#define SENERROR_V2		0x34
#define ERRCONFIG_V2		0x38

/* Bit/Shift Positions */

/* SRCONFIG */
#define SRCONFIG_ACCUMDATA_SHIFT	22
#define SRCONFIG_SRCLKLENGTH_SHIFT	12
#define SRCONFIG_SENNENABLE_V1_SHIFT	5
#define SRCONFIG_SENPENABLE_V1_SHIFT	3
#define SRCONFIG_SENNENABLE_V2_SHIFT	1
#define SRCONFIG_SENPENABLE_V2_SHIFT	0
#define SRCONFIG_CLKCTRL_SHIFT		0

#define SRCONFIG_ACCUMDATA_MASK		(0x3FF << 22)

#define SRCONFIG_SRENABLE		BIT(11)
#define SRCONFIG_SENENABLE		BIT(10)
#define SRCONFIG_ERRGEN_EN		BIT(9)
#define SRCONFIG_MINMAXAVG_EN		BIT(8)
#define SRCONFIG_DELAYCTRL		BIT(2)



#define PHY_TO_OFF_PM_RECIEVER(p)        (p - 0x5b)
#define R_DCDC_GLOBAL_CFG        PHY_TO_OFF_PM_RECIEVER(0x61)
#define DCDC_GLOBAL_CFG_ENABLE_SRFLX        0x08


/* AVGWEIGHT */
#define AVGWEIGHT_SENPAVGWEIGHT_SHIFT	2
#define AVGWEIGHT_SENNAVGWEIGHT_SHIFT	0

/* NVALUERECIPROCAL */
#define NVALUERECIPROCAL_SENPGAIN_SHIFT	20
#define NVALUERECIPROCAL_SENNGAIN_SHIFT	16
#define NVALUERECIPROCAL_RNSENP_SHIFT	8
#define NVALUERECIPROCAL_RNSENN_SHIFT	0

/* ERRCONFIG */
#define ERRCONFIG_ERRWEIGHT_SHIFT	16
#define ERRCONFIG_ERRMAXLIMIT_SHIFT	8
#define ERRCONFIG_ERRMINLIMIT_SHIFT	0

#define SR_ERRWEIGHT_MASK		(0x07 << 16)
#define SR_ERRMAXLIMIT_MASK		(0xFF << 8)
#define SR_ERRMINLIMIT_MASK		(0xFF << 0)

#define ERRCONFIG_VPBOUNDINTEN_V1	BIT(31)
#define ERRCONFIG_VPBOUNDINTST_V1	BIT(30)
#define	ERRCONFIG_MCUACCUMINTEN		BIT(29)
#define ERRCONFIG_MCUACCUMINTST		BIT(28)
#define	ERRCONFIG_MCUVALIDINTEN		BIT(27)
#define ERRCONFIG_MCUVALIDINTST		BIT(26)
#define ERRCONFIG_MCUBOUNDINTEN		BIT(25)
#define	ERRCONFIG_MCUBOUNDINTST		BIT(24)
#define	ERRCONFIG_MCUDISACKINTEN	BIT(23)
#define ERRCONFIG_VPBOUNDINTST_V2	BIT(23)
#define ERRCONFIG_MCUDISACKINTST	BIT(22)
#define ERRCONFIG_VPBOUNDINTEN_V2	BIT(22)

#define ERRCONFIG_STATUS_V1_MASK	(ERRCONFIG_VPBOUNDINTST_V1 | \
					ERRCONFIG_MCUACCUMINTST | \
					ERRCONFIG_MCUVALIDINTST | \
					ERRCONFIG_MCUBOUNDINTST | \
					ERRCONFIG_MCUDISACKINTST)
/* IRQSTATUS */
#define IRQSTATUS_MCUACCUMINT		BIT(3)
#define IRQSTATUS_MCVALIDINT		BIT(2)
#define IRQSTATUS_MCBOUNDSINT		BIT(1)
#define IRQSTATUS_MCUDISABLEACKINT	BIT(0)

/* IRQENABLE_SET and IRQENABLE_CLEAR */
#define IRQENABLE_MCUACCUMINT		BIT(3)
#define IRQENABLE_MCUVALIDINT		BIT(2)
#define IRQENABLE_MCUBOUNDSINT		BIT(1)
#define IRQENABLE_MCUDISABLEACKINT	BIT(0)

/* Common Bit values */

#define SRCLKLENGTH_12MHZ_SYSCLK	0x3C
#define SRCLKLENGTH_13MHZ_SYSCLK	0x41
#define SRCLKLENGTH_19MHZ_SYSCLK	0x60
#define SRCLKLENGTH_26MHZ_SYSCLK	0x82
#define SRCLKLENGTH_38MHZ_SYSCLK	0xC0

/*
 * 3430 specific values. Maybe these should be passed from board file or
 * pmic structures.
 */
#define OMAP3430_SR_ACCUMDATA		0x1F4

#define OMAP3430_SR1_SENPAVGWEIGHT	0x03
#define OMAP3430_SR1_SENNAVGWEIGHT	0x03

#define OMAP3430_SR2_SENPAVGWEIGHT	0x01
#define OMAP3430_SR2_SENNAVGWEIGHT	0x01

#define OMAP3430_SR_ERRWEIGHT		0x04
#define OMAP3430_SR_ERRMAXLIMIT		0x02

/* TODO:3630/OMAP4 values if it has to come from this file */

#ifdef CONFIG_OMAP_SMARTREFLEX_TESTING
#define SR_TESTING_NVALUES	1
#else
#define SR_TESTING_NVALUES	0
#endif

/**
 * omap_smartreflex_dev_data - Smartreflex device specific data
 *
 * @volts_supported	: Number of distinct voltages possible for the VDD
 *			  associated with this smartreflex module.
 * @efuse_sr_control	: The regisrter offset of control_fuse_sr efuse
 *			  register from which sennenable and senpenable values
 *			  are obtained.
 * @sennenable_shift	: The shift in the control_fuse_sr register for
 *			  obtaining the sennenable value for this smartreflex
 *			  module.
 * @senpenable_shift	: The shift in the control_fuse_sr register for
 *			  obtaining the senpenable value for this smartreflex
 *			  module.
 * @efuse_nvalues_offs	: Array of efuse offsets from which ntarget values can
 *			  be retrieved. Number of efuse offsets in this arrray
 *			  is equal to the volts_supported value ie one efuse
 *			  register per supported voltage.
 * @test_sennenable	: SENNENABLE test value
 * @test_senpenable	: SENPENABLE test value.
 * @test_nvalues	: Array of test ntarget values.
 * @vdd_name		: Name of the voltage domain associated with this
 *			  Smartreflex device.
 * @volt_data		: Voltage table associated with this smartreflex module
 */
struct omap_sr_dev_data {
	int volts_supported;
	u32 efuse_sr_control;
	u32 sennenable_shift;
	u32 senpenable_shift;
	u32 *efuse_nvalues_offs;
	u32 test_sennenable;
	u32 test_senpenable;
	u32 *test_nvalues;
	char *vdd_name;
	struct omap_volt_data *volt_data;
};

/**
 * omap_smartreflex_pmic_data : Strucutre to be populated by pmic code to pass
 * pmic specific info to smartreflex driver
 *
 * @sr_pmic_init - API to initialize smartreflex on the PMIC side.
 */
struct omap_smartreflex_pmic_data {
	void (*sr_pmic_init) (void);
};

#ifdef CONFIG_OMAP_SMARTREFLEX
/*
 * The smart reflex driver supports CLASS1 CLASS2 and CLASS3 SR.
 * The smartreflex class driver should pass the class type.
 * Should be used to populate the class_type field of the
 * omap_smartreflex_class_data structure.
 */
#define SR_CLASS1	0x1
#define SR_CLASS2	0x2
#define SR_CLASS3	0x3
#define SR_CLASS1P5	0x4

/* Smart reflex notifiers for class drivers to use */
#define SR_NOTIFY_MCUDISACK		BIT(3)
#define SR_NOTIFY_MCUBOUND		BIT(2)
#define SR_NOTIFY_MCUVALID		BIT(1)
#define SR_NOTIFY_MCUACCUM		BIT(0)

/**
 * omap_smartreflex_class_data : Structure to be populated by
 * Smartreflex class driver with corresponding class enable disable API's
 *
 * @enable - API to enable a particular class smaartreflex.
 * @disable - API to disable a particular class smartreflex.
 * @start:		API to do class specific initialization (optional)
 * @stop:		API to do class specific deinitialization (optional)
 * @configure - API to configure a particular class smartreflex.
 * @notify - API to notify the class driver about an event in SR. Not needed
 *		for class3.
 * @notify_flags - specify the events to be notified to the class driver
 * @class_type - specify which smartreflex class. Can be used by the SR driver
 *		to take any class based decisions.
 */
struct omap_smartreflex_class_data {
	int (*enable)(struct voltagedomain *voltdm,
			struct omap_volt_data *volt_data);
	int (*disable)(struct voltagedomain *voltdm,
			struct omap_volt_data *volt_data,
			int is_volt_reset);
	int (*start)(struct voltagedomain *voltdm, void *class_priv_data);
	int (*stop)(struct voltagedomain *voltdm, void *class_priv_data);
	int (*configure)(struct voltagedomain *voltdm);
	int (*notify)(struct voltagedomain *voltdm, u32 status);
	u8 notify_flags;
	u8 class_type;
	void *class_priv_data;
};

/**
 * omap_smartreflex_data - Smartreflex platform data
 *
 * @senp_mod		: SENPENABLE value for the sr
 * @senn_mod		: SENNENABLE value for sr
 * @sr_nvalue		: array of n target values for sr
 * @enable_on_init	: whether this sr module needs to enabled at
 *			  boot up or not.
 * @voltdm		: Pointer to the voltage domain associated with the SR
 */
struct omap_sr_data {
	u32				senp_mod;
	u32				senn_mod;
	bool				enable_on_init;
	struct voltagedomain		*voltdm;
	int (*device_enable)(struct platform_device *pdev);
	int (*device_shutdown)(struct platform_device *pdev);
	int (*device_idle)(struct platform_device *pdev);
};

/*
 * Smartreflex module enable/disable interface.
 * NOTE: if smartreflex is not enabled from sysfs, these functions will not
 * do anything.
 */
void omap_smartreflex_enable(struct voltagedomain *voltdm);
void omap_smartreflex_disable(struct voltagedomain *voltdm);
void omap_smartreflex_disable_reset_volt(struct voltagedomain *voltdm);

/* Smartreflex driver hooks to be called from Smartreflex class driver */
int sr_enable(struct voltagedomain *voltdm, struct omap_volt_data *volt_data);
void sr_disable(struct voltagedomain *voltdm);
int sr_notifier_control(struct voltagedomain *voltdm, bool enable);
int sr_configure_errgen(struct voltagedomain *voltdm);
int sr_configure_minmax(struct voltagedomain *voltdm);

/* API to register the smartreflex class driver with the smartreflex driver */
int omap_sr_register_class(struct omap_smartreflex_class_data *class_data);
bool is_sr_enabled(struct voltagedomain *voltdm);

/* API to register the pmic specific data with the smartreflex driver. */
void omap_sr_register_pmic(struct omap_smartreflex_pmic_data *pmic_data);
#else
static inline void omap_smartreflex_enable(struct voltagedomain *voltdm) {}
static inline void omap_smartreflex_disable(struct voltagedomain *voltdm) {}
static inline void omap_smartreflex_disable_reset_volt(
		struct voltagedomain *voltdm) {}
static inline int sr_notifier_control(struct voltagedomain *voltdm,
		bool enable)
{
	return -EINVAL;
}

static inline void omap_sr_register_pmic(
		struct omap_smartreflex_pmic_data *pmic_data) {}
static inline bool is_sr_enabled(struct voltagedomain *voltdm)
{
	return false;
}
#endif
#endif
