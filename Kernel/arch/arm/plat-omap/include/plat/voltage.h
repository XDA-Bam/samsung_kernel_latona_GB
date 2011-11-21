/*
 * OMAP Voltage Management Routines
 *
 * Author: Thara Gopinath	<thara@ti.com>
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * Copyright (C) 2010 Motorola
 * Lun Chang <l.chang@motorola.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_VOLTAGE_H
#define __ARCH_ARM_MACH_OMAP2_VOLTAGE_H

#include <linux/notifier.h>
#include <linux/err.h>

extern u32 enable_sr_vp_debug;
#ifdef CONFIG_PM_DEBUG
extern struct dentry *pm_dbg_main_dir;
#endif

#define VOLTSCALE_VPFORCEUPDATE		1
#define VOLTSCALE_VCBYPASS		2

#define VOLTAGE_PRECHANGE	0
#define VOLTAGE_POSTCHANGE	1

struct omap_volt_pmic_info {
	char *name;
	int slew_rate;
	int step_size;
	unsigned char i2c_addr;
	unsigned char i2c_vreg;
	unsigned char i2c_cmdreg;
	unsigned long (*vsel_to_uv)(unsigned char vsel);
	unsigned char (*uv_to_vsel)(unsigned long uv);
	unsigned char (*onforce_cmd)(unsigned char vsel);
	unsigned char (*on_cmd)(unsigned char vsel);
	unsigned char (*sleepforce_cmd)(unsigned char vsel);
	unsigned char (*sleep_cmd)(unsigned char vsel);
	unsigned char vp_config_erroroffset;
	unsigned char vp_vstepmin_vstepmin;
	unsigned char vp_vstepmax_vstepmax;
	unsigned short vp_vlimitto_timeout_us;
	unsigned char vp_vlimitto_vddmin;
	unsigned char vp_vlimitto_vddmax;
};

/**
 * voltagedomain - omap voltage domain global structure
 * @name       : Name of the voltage domain which can be used as a unique
 *               identifier.
 */
struct voltagedomain {
       char *name;
};

#define OMAP3PLUS_DYNAMIC_NOMINAL_MARGIN_UV	50000

/**
 * omap_volt_data - Omap voltage specific data.
 * @voltage_nominal	: The possible voltage value in uV
 * @sr_nvalue		: Smartreflex N target value at voltage <voltage>
 * @sr_errminlimit	: Error min limit value for smartreflex. This value
 *			  differs at differnet opp and thus is linked
 *			  with voltage.
 * @vp_errorgain	: Error gain value for the voltage processor. This
 *			  field also differs according to the voltage/opp.
 * @abb_type		: Bitfield OPP_SEL.PRM_LDO_ABB_CTRL.
 */
struct omap_volt_data {
	u32	volt_nominal;
	u32	volt_calibrated;
	u32	volt_dynamic_nominal;
	u32	sr_nvalue;
	u32	sr_oppmargin;
	u8	sr_errminlimit;
	u8	vp_errgain;
	u8	abb_type;
};


/* Various voltage controller related info */
struct omap_volt_vc_data {
	u16 clksetup;
	u16 voltsetup_time1;
	u16 voltsetup_time2;
	u16 voltoffset;
	u16 voltsetup2;
/* PRM_VC_CMD_VAL_0 specific bits */
	u32 vdd0_on;
	u32 vdd0_onlp;
	u32 vdd0_ret;
	u32 vdd0_off;
/* PRM_VC_CMD_VAL_1 specific bits */
	u32 vdd1_on;
	u32 vdd1_onlp;
	u32 vdd1_ret;
	u32 vdd1_off;
/* PRM_VC_CMD_VAL_2 specific bits */
	u32 vdd2_on;
	u32 vdd2_onlp;
	u32 vdd2_ret;
	u32 vdd2_off;
};

/* Voltage change notifier structure */
struct omap_volt_change_info {
	struct omap_vdd_info *vdd_info;
	struct omap_volt_data *curr_volt;
	struct omap_volt_data *target_volt;
};

struct voltagedomain *omap_voltage_domain_get(char *name);
unsigned long omap_vp_get_curr_volt(struct voltagedomain *voltdm);
void omap_vp_enable(struct voltagedomain *voltdm);
void omap_vp_disable(struct voltagedomain *voltdm);
int omap_voltage_scale_vdd(struct voltagedomain *voltdm,
		struct omap_volt_data *target_volt);
void omap_voltage_reset(struct voltagedomain *voltdm);
int omap_voltage_get_volttable(struct voltagedomain *voltdm,
		struct omap_volt_data **volt_data);
struct omap_volt_data *omap_voltage_get_voltdata(struct voltagedomain *voltdm,
		unsigned long volt);
void omap_voltage_register_pmic(struct omap_volt_pmic_info *pmic_info,
		char *vdmname);
struct omap_volt_data *omap_voltage_get_nom_volt(struct voltagedomain *voltdm);
int omap_voltage_add_userreq(struct voltagedomain *voltdm, struct device *dev,
		unsigned long *volt);
int omap_voltage_scale(struct voltagedomain *voltdm);
bool omap_vp_is_transdone(struct voltagedomain *voltdm);
bool omap_vp_clear_transdone(struct voltagedomain *voltdm);

int omap_voltage_calib_reset(struct voltagedomain *voltdm);
int omap_vscale_pause(struct voltagedomain *voltdm, bool trylock);
int omap_vscale_unpause(struct voltagedomain *voltdm);

#ifdef CONFIG_PM
void omap_voltage_init_vc(struct omap_volt_vc_data *setup_vc);
void omap_change_voltscale_method(int voltscale_method);
int omap_voltage_register_notifier(struct voltagedomain *voltdm,
		struct notifier_block *nb);
int omap_voltage_unregister_notifier(struct voltagedomain *voltdm,
		struct notifier_block *nb);
#else
static inline void omap_voltage_init_vc(struct omap_volt_vc_data *setup_vc) {}
static inline  void omap_change_voltscale_method(int voltscale_method) {}
static inline int omap_voltage_register_notifier(
		struct voltagedomain *voltdm, struct notifier_block *nb)
{
	return 0;
}

static inline int omap_voltage_unregister_notifier(
		 struct voltagedomain *voltdm, struct notifier_block *nb)
{
	return 0;
}
#endif

/* convert volt data to the voltage for the voltage data */
static inline unsigned long omap_get_operation_voltage(
		struct omap_volt_data *vdata)
{
	if (IS_ERR_OR_NULL(vdata))
		return 0;
	return (vdata->volt_calibrated) ? vdata->volt_calibrated :
		(vdata->volt_dynamic_nominal) ? vdata->volt_dynamic_nominal :
			vdata->volt_nominal;
}

/* what is my dynamic nominal? */
static inline unsigned long omap_get_dyn_nominal(struct omap_volt_data *vdata)
{
	if (IS_ERR_OR_NULL(vdata))
		return 0;
	if (vdata->volt_calibrated) {
		unsigned long v = vdata->volt_calibrated +
			OMAP3PLUS_DYNAMIC_NOMINAL_MARGIN_UV;
		if (v > vdata->volt_nominal)
			return vdata->volt_nominal;
		return v;
	}
	return vdata->volt_nominal;
}
#endif
