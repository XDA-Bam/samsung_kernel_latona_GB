/*
 * OMAP3/OMAP4 Voltage Management Routines
 *
 * Author: Thara Gopinath	<thara@ti.com>
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 * Lesly A M <x0080970@ti.com>
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Thara Gopinath <thara@ti.com>
 *
 * Copyright (C) 2010 Motorola
 * Lun Chang <l.chang@motorola.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/spinlock.h>
#include <linux/plist.h>
#include <linux/slab.h>

#include <plat/omap-pm.h>
#include <plat/omap34xx.h>
#include <plat/opp.h>
#include <plat/clock.h>
#include <plat/common.h>
#include <plat/voltage.h>
#include <plat/smartreflex.h>
#include <plat/control.h>

#include "prm-regbits-34xx.h"
#include "prm44xx.h"
#include "prm-regbits-44xx.h"

#define VP_IDLE_TIMEOUT		200
#define VP_TRANXDONE_TIMEOUT	400

/* Settling Time for ABB Transition in us */
#define ABB_TRANXDONE_TIMEOUT  30

/* Bitfield OPP_SEL.PRM_LDO_ABB_CTRL */
#define FAST_OPP		0x1
#define NOMINAL_OPP		0x0

#ifdef CONFIG_PM_DEBUG
#include <linux/seq_file.h>
static struct dentry *voltage_dir;
#endif

/* VP SR debug support */
u32 enable_sr_vp_debug;

/* PRM voltage module */
static u32 volt_mod;

/* Voltage processor register offsets */
struct vp_reg_offs {
	u8 vpconfig;
	u8 vstepmin;
	u8 vstepmax;
	u8 vlimitto;
	u8 vstatus;
	u8 voltage;
};

/* Voltage Processor bit field values, shifts and masks */
struct vp_reg_val {
	/* VPx_VPCONFIG */
	u32 vpconfig_erroroffset;
	u16 vpconfig_errorgain;
	u32 vpconfig_errorgain_mask;
	u8 vpconfig_errorgain_shift;
	u32 vpconfig_initvoltage_mask;
	u8 vpconfig_initvoltage_shift;
	u32 vpconfig_timeouten;
	u32 vpconfig_initvdd;
	u32 vpconfig_forceupdate;
	u32 vpconfig_vpenable;
	/* VPx_VSTEPMIN */
	u8 vstepmin_stepmin;
	u16 vstepmin_smpswaittimemin;
	u8 vstepmin_stepmin_shift;
	u8 vstepmin_smpswaittimemin_shift;
	/* VPx_VSTEPMAX */
	u8 vstepmax_stepmax;
	u16 vstepmax_smpswaittimemax;
	u8 vstepmax_stepmax_shift;
	u8 vstepmax_smpswaittimemax_shift;
	/* VPx_VLIMITTO */
	u16 vlimitto_vddmin;
	u16 vlimitto_vddmax;
	u16 vlimitto_timeout;
	u16 vlimitto_vddmin_shift;
	u16 vlimitto_vddmax_shift;
	u16 vlimitto_timeout_shift;
	/* PRM_IRQSTATUS*/
	u32 tranxdone_status;
};

/**
 * omap_vdd_dep_volt - Table containing the parent vdd voltage and the
 *			dependent vdd voltage corresponding to it.
 *
 * @main_vdd_volt	: The main vdd voltage
 * @dep_vdd_volt	: The voltage at which the dependent vdd should be
 *			  when the main vdd is at <main_vdd_volt> voltage
 */
struct omap_vdd_dep_volt {
	u32 main_vdd_volt;
	u32 dep_vdd_volt;
};

/**
 *  ABB Register offsets and masks
 *
 * @prm_abb_ldo_setup_idx : PRM_LDO_ABB_SETUP Register specific to MPU/IVA
 * @prm_abb_ldo_ctrl_idx  : PRM_LDO_ABB_CTRL Register specific to MPU/IVA
 * @prm_irqstatus_mpu	  : PRM_IRQSTATUS_MPU_A9/PRM_IRQSTATUS_MPU_A9_2
 * @abb_done_st_shift	  : ABB_DONE_ST shift
 * @abb_done_st_mask	  : ABB_DONE_ST_MASK bit mask
 *
 */
struct abb_reg_val {
	u16 prm_abb_ldo_setup_idx;
	u16 prm_abb_ldo_ctrl_idx;
	u16 prm_irqstatus_mpu;
	u32 abb_done_st_shift;
	u32 abb_done_st_mask;
};

/**
 * omap_vdd_dep_info - Dependent vdd info
 *
 * @name		: Dependent vdd name
 * @voltdm		: Dependent vdd pointer
 * @dep_table		: Table containing the dependent vdd voltage
 *			  corresponding to every main vdd voltage.
 */
struct omap_vdd_dep_info{
	char *name;
	struct voltagedomain *voltdm;
	struct omap_vdd_dep_volt *dep_table;
};

/**
 * omap_vdd_user_list	- The per vdd user list
 *
 * @dev		: The device asking for the vdd to be set at a particular
 *		  voltage
 * @node	: The list head entry
 * @volt	: The voltage requested by the device <dev>
 */
struct omap_vdd_user_list {
	struct device *dev;
	struct plist_node node;
	u32 volt;
};

/**
 * omap_vdd_info - Per Voltage Domain info
 *
 * @volt_data		: voltage table having the distinct voltages supported
 *			  by the domain and other associated per voltage data.
 * @vp_offs		: structure containing the offsets for various
 *			  vp registers
 * @vp_reg		: the register values, shifts, masks for various
 *			  vp registers
 * @volt_clk		: the clock associated with the vdd.
 * @opp_dev		: the 'struct device' associated with this vdd.
 * @user_lock		: the lock to be used by the plist user_list
 * @user_list		: the list head maintaining the various users
 *			  of this vdd with the voltage requested by each user.
 * @volt_data_count	: Number of distinct voltages supported by this vdd.
 * @nominal_volt	: Nominal voltaged for this vdd.
 * cmdval_reg		: Voltage controller cmdval register.
 * @vdd_sr_reg		: The smartreflex register associated with this VDD.
 */
struct omap_vdd_info{
	struct omap_volt_data *volt_data;
	struct vp_reg_offs vp_offs;
	struct vp_reg_val vp_reg;
	struct clk *volt_clk;
	struct device *opp_dev;
	struct voltagedomain voltdm;
	struct abb_reg_val omap_abb_reg_val;
	struct omap_vdd_dep_info *dep_vdd_info;
	spinlock_t user_lock;
	struct plist_head user_list;
	struct mutex scaling_mutex;
	struct srcu_notifier_head volt_change_notify_list;
	int volt_data_count;
	int nr_dep_vdd;
	struct device **dev_list;
	int dev_count;
	struct omap_volt_data *nominal_volt;
	struct omap_volt_data *curr_volt;
	u8 cmdval_reg;
	u8 vdd_sr_reg;
	u16 ocp_mod;
	u8 prm_irqst_reg;
	struct omap_volt_pmic_info *pmic;
	struct device vdd_device;
};
static struct omap_vdd_info *vdd_info;
#ifdef CONFIG_OMAP_ABB
static int omap3_abb_change_opp(struct omap_vdd_info *vdd_info);
static int omap4_abb_change_opp(struct omap_vdd_info *vdd_info);
#endif

/*
 * Number of scalable voltage domains.
 */
static int no_scalable_vdd;

/* OMAP3 VDD sturctures */
static struct omap_vdd_info omap3_vdd_info[] = {
	{
		.vp_offs = {
			.vpconfig = OMAP3_PRM_VP1_CONFIG_OFFSET,
			.vstepmin = OMAP3_PRM_VP1_VSTEPMIN_OFFSET,
			.vstepmax = OMAP3_PRM_VP1_VSTEPMAX_OFFSET,
			.vlimitto = OMAP3_PRM_VP1_VLIMITTO_OFFSET,
			.vstatus = OMAP3_PRM_VP1_STATUS_OFFSET,
			.voltage = OMAP3_PRM_VP1_VOLTAGE_OFFSET,
		},
		.voltdm = {
			.name = "mpu",
		},
		.omap_abb_reg_val = {
			.prm_abb_ldo_setup_idx =
				OMAP3_PRM_LDO_ABB_SETUP_OFFSET,
			.prm_abb_ldo_ctrl_idx =
				OMAP3_PRM_LDO_ABB_CTRL_OFFSET,
			.prm_irqstatus_mpu = OMAP3_PRM_IRQSTATUS_MPU_OFFSET,
			.abb_done_st_shift = OMAP3630_ABB_LDO_TRANXDONE_ST_SHIFT,
			.abb_done_st_mask = OMAP3630_ABB_LDO_TRANXDONE_ST_MASK,
		},
	},
	{
		.vp_offs = {
			.vpconfig = OMAP3_PRM_VP2_CONFIG_OFFSET,
			.vstepmin = OMAP3_PRM_VP2_VSTEPMIN_OFFSET,
			.vstepmax = OMAP3_PRM_VP2_VSTEPMAX_OFFSET,
			.vlimitto = OMAP3_PRM_VP2_VLIMITTO_OFFSET,
			.vstatus = OMAP3_PRM_VP2_STATUS_OFFSET,
			.voltage = OMAP3_PRM_VP2_VOLTAGE_OFFSET,
		},
		.voltdm = {
			.name = "core",
		},
	},
};

#define OMAP3_NO_SCALABLE_VDD ARRAY_SIZE(omap3_vdd_info)

/* OMAP4 VDD sturctures */
static struct omap_vdd_info omap4_vdd_info[] = {
	{
		.vp_offs = {
			.vpconfig = OMAP4_PRM_VP_MPU_CONFIG_OFFSET,
			.vstepmin = OMAP4_PRM_VP_MPU_VSTEPMIN_OFFSET,
			.vstepmax = OMAP4_PRM_VP_MPU_VSTEPMAX_OFFSET,
			.vlimitto = OMAP4_PRM_VP_MPU_VLIMITTO_OFFSET,
			.vstatus = OMAP4_PRM_VP_MPU_STATUS_OFFSET,
			.voltage = OMAP4_PRM_VP_MPU_VOLTAGE_OFFSET,
		},
		.voltdm = {
			.name = "mpu",
		},
		.omap_abb_reg_val = {
			.prm_abb_ldo_setup_idx =
				OMAP4_PRM_LDO_ABB_MPU_SETUP_OFFSET,
			.prm_abb_ldo_ctrl_idx =
				OMAP4_PRM_LDO_ABB_MPU_CTRL_OFFSET,
			.prm_irqstatus_mpu = OMAP4_PRM_IRQSTATUS_MPU_2_OFFSET,
			.abb_done_st_shift = OMAP4430_ABB_MPU_DONE_ST_SHIFT,
			.abb_done_st_mask = OMAP4430_ABB_MPU_DONE_ST_MASK,
		},
	},
	{
		.vp_offs = {
			.vpconfig = OMAP4_PRM_VP_IVA_CONFIG_OFFSET,
			.vstepmin = OMAP4_PRM_VP_IVA_VSTEPMIN_OFFSET,
			.vstepmax = OMAP4_PRM_VP_IVA_VSTEPMAX_OFFSET,
			.vlimitto = OMAP4_PRM_VP_IVA_VLIMITTO_OFFSET,
			.vstatus = OMAP4_PRM_VP_IVA_STATUS_OFFSET,
			.voltage = OMAP4_PRM_VP_IVA_VOLTAGE_OFFSET,
		},
		.voltdm = {
			.name = "iva",
		},
		.omap_abb_reg_val = {
			.prm_abb_ldo_setup_idx =
				OMAP4_PRM_LDO_ABB_IVA_SETUP_OFFSET,
			.prm_abb_ldo_ctrl_idx =
				OMAP4_PRM_LDO_ABB_IVA_CTRL_OFFSET,
			.prm_irqstatus_mpu = OMAP4_PRM_IRQSTATUS_MPU_OFFSET,
			.abb_done_st_shift = OMAP4430_ABB_IVA_DONE_ST_SHIFT,
			.abb_done_st_mask = OMAP4430_ABB_IVA_DONE_ST_MASK,
		},
	},
	{
		.vp_offs = {
			.vpconfig = OMAP4_PRM_VP_CORE_CONFIG_OFFSET,
			.vstepmin = OMAP4_PRM_VP_CORE_VSTEPMIN_OFFSET,
			.vstepmax = OMAP4_PRM_VP_CORE_VSTEPMAX_OFFSET,
			.vlimitto = OMAP4_PRM_VP_CORE_VLIMITTO_OFFSET,
			.vstatus = OMAP4_PRM_VP_CORE_STATUS_OFFSET,
			.voltage = OMAP4_PRM_VP_CORE_VOLTAGE_OFFSET,
		},
		.voltdm = {
			.name = "core",
		},
	},
};
#define OMAP4_NO_SCALABLE_VDD ARRAY_SIZE(omap4_vdd_info)

/*
 * Default voltage controller settings.
 */
static struct omap_volt_vc_data vc_config = {
	.clksetup = 0xff,
	.voltsetup_time1 = 0xfff,
	.voltsetup_time2 = 0xfff,
	.voltoffset = 0xff,
	.voltsetup2 = 0xff,
	.vdd0_on = 1200000,        /* 1.2v */
	.vdd0_onlp = 1000000,      /* 1.0v */
	.vdd0_ret = 975000,       /* 0.975v */
	.vdd0_off = 600000,       /* 0.6v */
	.vdd1_on = 1150000,        /* 1.15v */
	.vdd1_onlp = 1000000,      /* 1.0v */
	.vdd1_ret = 975000,       /* .975v */
	.vdd1_off = 600000,       /* 0.6v */
};

/*
 * Structures containing OMAP3430/OMAP3630 voltage supported and various
 * data associated with it per voltage domain basis. Smartreflex Ntarget
 * values are left as 0 as they have to be populated by smartreflex
 * driver after reading the efuse.
 */

/* VDD1 */
static struct omap_volt_data omap34xx_vdd1_volt_data[] = {
	{.volt_nominal = 975000, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C},
	{.volt_nominal = 1075000, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C},
	{.volt_nominal = 1200000, .sr_errminlimit = 0xF9, .vp_errgain = 0x18},
	{.volt_nominal = 1270000, .sr_errminlimit = 0xF9, .vp_errgain = 0x18},
	{.volt_nominal = 1350000, .sr_errminlimit = 0xF9, .vp_errgain = 0x18},
};

static struct omap_volt_data omap36xx_vdd1_volt_data[] = {
	{.volt_nominal = 1025000, .sr_oppmargin = 37500, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C, .abb_type = NOMINAL_OPP},
	{.volt_nominal = 1200000, .sr_oppmargin = 37500, .sr_errminlimit = 0xF9, .vp_errgain = 0x16, .abb_type = NOMINAL_OPP},
	{.volt_nominal = 1330000, .sr_oppmargin = 37500, .sr_errminlimit = 0xFA, .vp_errgain = 0x23, .abb_type = NOMINAL_OPP},
	{.volt_nominal = 1387500, .sr_oppmargin = 62500,  .sr_errminlimit = 0xFA, .vp_errgain = 0x27, .abb_type = FAST_OPP},
};

/* VDD2 */
static struct omap_volt_data omap34xx_vdd2_volt_data[] = {
	{.volt_nominal = 975000, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C},
	{.volt_nominal = 1050000, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C},
	{.volt_nominal = 1150000, .sr_errminlimit = 0xF9, .vp_errgain = 0x18},
};

static struct omap_volt_data omap36xx_vdd2_volt_data[] = {
	{.volt_nominal = 930000, .sr_oppmargin = 37500,  .sr_errminlimit = 0xF4, .vp_errgain = 0x0C},
	//{.volt_nominal = 1162500, .sr_errminlimit = 0xF9, .vp_errgain = 0x16},
	{.volt_nominal = 1162500, .sr_oppmargin = 37500, .sr_errminlimit = 0xF9, .vp_errgain = 0x16},

//	{.volt_nominal = 930000, .sr_oppmargin = 0,  .sr_errminlimit = 0xF4, .vp_errgain = 0x0C},
//	{.volt_nominal = 1162500, .sr_oppmargin = 0, .sr_errminlimit = 0xF9, .vp_errgain = 0x16},

};

/*
 * Structures containing OMAP4430 voltage supported and various
 * data associated with it per voltage domain basis. Smartreflex Ntarget
 * values are left as 0 as they have to be populated by smartreflex
 * driver after reading the efuse.
 */
static struct omap_volt_data omap44xx_vdd_mpu_volt_data[] = {
	{.volt_nominal = 1005000, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C, .abb_type = NOMINAL_OPP},
	{.volt_nominal = 1025000, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C, .abb_type = NOMINAL_OPP},
	{.volt_nominal = 1200000, .sr_errminlimit = 0xF9, .vp_errgain = 0x16, .abb_type = NOMINAL_OPP},
	{.volt_nominal = 1313000, .sr_errminlimit = 0xFA, .vp_errgain = 0x23, .abb_type = NOMINAL_OPP},
	{.volt_nominal = 1375000, .sr_errminlimit = 0xFA, .vp_errgain = 0x27, .abb_type = FAST_OPP},
};

static struct omap_volt_data omap44xx_vdd_iva_volt_data[] = {
	{.volt_nominal = 1011000, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C, .abb_type = NOMINAL_OPP},
	{.volt_nominal = 1013000, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C, .abb_type = NOMINAL_OPP},
	{.volt_nominal = 1188000, .sr_errminlimit = 0xF9, .vp_errgain = 0x16, .abb_type = NOMINAL_OPP},
#ifdef CONFIG_OMAP_ABB_DEFAULT_IVA_FBB
	{.volt_nominal = 1300000, .sr_errminlimit = 0xFA, .vp_errgain = 0x23, .abb_type = FAST_OPP},
#else
	{.volt_nominal = 1300000, .sr_errminlimit = 0xFA, .vp_errgain = 0x23, .abb_type = NOMINAL_OPP},
#endif
};

static struct omap_volt_data omap44xx_vdd_core_volt_data[] = {
	{.volt_nominal = 1005000, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C},
	{.volt_nominal = 1025000, .sr_errminlimit = 0xF4, .vp_errgain = 0x0C},
	{.volt_nominal = 1200000, .sr_errminlimit = 0xF9, .vp_errgain = 0x16},
};

/* OMAP 3430 MPU Core VDD dependency table */
static struct omap_vdd_dep_volt omap34xx_vdd1_vdd2_data[] = {
	{.main_vdd_volt = 975000, .dep_vdd_volt = 1050000},
	{.main_vdd_volt = 1075000, .dep_vdd_volt = 1050000},
	{.main_vdd_volt = 1200000, .dep_vdd_volt = 1150000},
	{.main_vdd_volt = 1270000, .dep_vdd_volt = 1150000},
	{.main_vdd_volt = 1350000, .dep_vdd_volt = 1150000},
	{.main_vdd_volt = 0, .dep_vdd_volt = 0},
};

static struct omap_vdd_dep_info omap34xx_vdd1_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap34xx_vdd1_vdd2_data,
	},
};

/* OMAP 4430 MPU Core VDD dependency table */
static struct omap_vdd_dep_volt omap44xx_vddmpu_vddcore_data[] = {
	{.main_vdd_volt = 1005000, .dep_vdd_volt = 1025000},
	{.main_vdd_volt = 1025000, .dep_vdd_volt = 1025000},
	{.main_vdd_volt = 1200000, .dep_vdd_volt = 1200000},
	{.main_vdd_volt = 1313000, .dep_vdd_volt = 1200000},
	{.main_vdd_volt = 1375000, .dep_vdd_volt = 1200000},
	{.main_vdd_volt = 0, .dep_vdd_volt = 0},
};

static struct omap_vdd_dep_volt omap44xx_vddiva_vddcore_data[] = {
	{.main_vdd_volt = 1011000, .dep_vdd_volt = 1025000},
	{.main_vdd_volt = 1013000, .dep_vdd_volt = 1025000},
	{.main_vdd_volt = 1188000, .dep_vdd_volt = 1200000},
	{.main_vdd_volt = 1300000, .dep_vdd_volt = 1200000},
	{.main_vdd_volt = 0, .dep_vdd_volt = 0},
};

static struct omap_vdd_dep_info omap44xx_vddmpu_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap44xx_vddmpu_vddcore_data,
	},
};

static struct omap_vdd_dep_info omap44xx_vddiva_dep_info[] = {
	{
		.name	= "core",
		.dep_table = omap44xx_vddiva_vddcore_data,
	},
};

/* By default VPFORCEUPDATE is the chosen method of voltage scaling */
static bool voltscale_vpforceupdate = true;

static inline u32 voltage_read_reg(u8 offset)
{
	return prm_read_mod_reg(volt_mod, offset);
}

static inline void voltage_write_reg(u8 offset, u32 value)
{
	prm_write_mod_reg(value, volt_mod, offset);
}

/* Voltage debugfs support */
#ifdef CONFIG_PM_DEBUG
static int vp_debug_get(void *data, u64 *val)
{
	u16 *option = data;

	if (!option) {
		pr_warning("Wrong paramater passed\n");
		return -EINVAL;
	}

	*val = *option;
	return 0;
}

static int vp_debug_set(void *data, u64 val)
{
	if (enable_sr_vp_debug) {
		u32 *option = data;
		if (!option) {
			pr_warning("Wrong paramater passed\n");
			return -EINVAL;
		}
		*option = val;
	} else {
		pr_notice("DEBUG option not enabled!"
			"echo 1 > pm_debug/enable_sr_vp_debug - to enable\n");
	}
	return 0;
}

static int vp_volt_debug_get(void *data, u64 *val)
{
	struct omap_vdd_info *vdd = (struct omap_vdd_info *) data;
	u8 vsel;

	if (!vdd) {
		pr_warning("Wrong paramater passed\n");
		return -EINVAL;
	}

	vsel = voltage_read_reg(vdd->vp_offs.voltage);
	*val = vdd->pmic->vsel_to_uv(vsel);

	return 0;
}

static int nom_volt_debug_get(void *data, u64 *val)
{
	struct omap_vdd_info *vdd = (struct omap_vdd_info *) data;
	struct omap_volt_data *vdata;

	if (!vdd) {
		pr_warning("Wrong paramater passed\n");
		return -EINVAL;
	}

	vdata = omap_voltage_get_nom_volt(&vdd->voltdm);
	if (IS_ERR_OR_NULL(vdata))
		return -EINVAL;

	*val = vdata->volt_nominal;

	return 0;
}

static int volt_dbg_show_users(struct seq_file *s, void *unused)
{
	struct omap_vdd_info *vdd = 0;
	struct omap_vdd_user_list *user;
	int count = 0;

	vdd = (struct omap_vdd_info *)s->private ;
	plist_for_each_entry(user, &vdd->user_list, node) {
		count++;
		pr_info("VDD=%s: User=%d: Name=%s: Volt=%d\n",
			vdd->voltdm.name, count, dev_name(user->dev),
			user->volt);
	}

	return 0;
}

static int volt_users_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, volt_dbg_show_users,
		inode->i_private);
}

static int dyn_volt_debug_get(void *data, u64 *val)
{
	struct omap_vdd_info *vdd = (struct omap_vdd_info *) data;
	struct omap_volt_data *volt_data;

	if (!vdd) {
		pr_warning("Wrong paramater passed\n");
		return -EINVAL;
	}

	volt_data = omap_voltage_get_nom_volt(&vdd->voltdm);
	if (IS_ERR_OR_NULL(volt_data)) {
		pr_warning("%s: No voltage/domain?\n", __func__);
		return -ENODEV;
	}

	*val = volt_data->volt_dynamic_nominal;

	return 0;
}

static int calib_volt_debug_get(void *data, u64 *val)
{
	struct omap_vdd_info *vdd = (struct omap_vdd_info *) data;
	struct omap_volt_data *volt_data;

	if (!vdd) {
		pr_warning("Wrong paramater passed\n");
		return -EINVAL;
	}

	volt_data = omap_voltage_get_nom_volt(&vdd->voltdm);
	if (IS_ERR_OR_NULL(volt_data)) {
		pr_warning("%s: No voltage/domain?\n", __func__);
		return -ENODEV;
	}

	*val = volt_data->volt_calibrated;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vp_debug_fops, vp_debug_get, vp_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(vp_volt_debug_fops, vp_volt_debug_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(nom_volt_debug_fops, nom_volt_debug_get, NULL,
								"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(dyn_volt_debug_fops, dyn_volt_debug_get, NULL,
								"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(calib_volt_debug_fops, calib_volt_debug_get, NULL,
								"%llu\n");
static const struct file_operations volt_users_dbg_fops = {
	.open           = volt_users_dbg_open,
	.read		= seq_read,
	.llseek 	= seq_lseek,
	.release        = single_release,
};

#endif

static unsigned char omap_vdd_id(char *vdm)
{
	int i;
	for (i = 0; i < no_scalable_vdd; i++) {
		if (!strcmp(vdd_info[i].voltdm.name, vdm))
			return i;
	}
	pr_warning("%s: Error in getting vdd id for vdd_%s\n", __func__, vdm);
	return -EINVAL;
}

static void vp_latch_vsel(struct omap_vdd_info *vdd)
{
	u32 vpconfig;
	unsigned long uvdc;
	char vsel;

	uvdc = omap_get_operation_voltage(
			omap_voltage_get_nom_volt(&vdd->voltdm));
	if (!uvdc) {
		pr_warning("%s: unable to find current voltage for vdd_%s\n",
			__func__, vdd->voltdm.name);
		return;
	}
	vsel = vdd->pmic->uv_to_vsel(uvdc);
	vpconfig = voltage_read_reg(vdd->vp_offs.vpconfig);
	vpconfig &= ~(vdd->vp_reg.vpconfig_initvoltage_mask |
			vdd->vp_reg.vpconfig_initvdd);
	vpconfig |= vsel << vdd->vp_reg.vpconfig_initvoltage_shift;

	voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);

	/* Trigger initVDD value copy to voltage processor */
	voltage_write_reg(vdd->vp_offs.vpconfig,
			(vpconfig | vdd->vp_reg.vpconfig_initvdd));

	/* Clear initVDD copy trigger bit */
	voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);
}

#ifdef CONFIG_OMAP_ABB
/**
 * omap_abb_notify_voltage - voltage change notifier handler for ABB
 * @nb	 : notifier block
 * @val	 : VOLTAGE_PRECHANGE or VOLTAGE_POSTCHANGE
 * @data : struct omap_volt_change_info for a given voltage domain
 *
 * Sets ABB ldo to either bypass or Forward Body-Bias whenever a voltage
 * change notification is generated.  Voltages marked as FAST will result in
 * FBB operation of ABB ldo and voltages marked as NOMINAL will bypass the
 * ldo.  Returns 0 upon success, negative error code otherwise.
 */
static int omap_abb_notify_voltage(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct omap_volt_data *curr_volt_data, *target_volt_data;
	struct omap_volt_change_info *v_info;
	int ret = 0;

	v_info = (struct omap_volt_change_info *)data;
	target_volt_data = v_info->target_volt;
	curr_volt_data = v_info->curr_volt;

	/* nothing to do here */
	if (target_volt_data->abb_type == curr_volt_data->abb_type)
		goto out;

	if (val == VOLTAGE_PRECHANGE &&
			target_volt_data->abb_type == NOMINAL_OPP) {
		/* bypass ABB before lowering voltage */
		if (cpu_is_omap3630()) {
			prm_rmw_mod_reg_bits(OMAP3630_OPP_SEL_MASK,
			(NOMINAL_OPP << OMAP3630_OPP_SEL_SHIFT),
			OMAP3430_GR_MOD,
			v_info->vdd_info->omap_abb_reg_val.prm_abb_ldo_setup_idx);
			ret = omap3_abb_change_opp(v_info->vdd_info);
		} else { /* cpu_is_omap44xx() */
			prm_rmw_mod_reg_bits(OMAP4430_OPP_SEL_MASK,
			(NOMINAL_OPP << OMAP4430_OPP_SEL_SHIFT),
			OMAP4430_PRM_DEVICE_MOD,
			v_info->vdd_info->omap_abb_reg_val.prm_abb_ldo_ctrl_idx);
			ret = omap4_abb_change_opp(v_info->vdd_info);
		}
	} else if (val == VOLTAGE_POSTCHANGE &&
			target_volt_data->abb_type == FAST_OPP) {
		/* enable Forward Body-Bias before raising voltage */
		if (cpu_is_omap3630()) {
			prm_rmw_mod_reg_bits(OMAP3630_OPP_SEL_MASK,
			(FAST_OPP << OMAP3630_OPP_SEL_SHIFT),
			OMAP3430_GR_MOD,
			v_info->vdd_info->omap_abb_reg_val.prm_abb_ldo_setup_idx);
			ret = omap3_abb_change_opp(v_info->vdd_info);
		} else { /* cpu_is_omap44xx() */
			prm_rmw_mod_reg_bits(OMAP4430_OPP_SEL_MASK,
			(FAST_OPP << OMAP4430_OPP_SEL_SHIFT),
			OMAP4430_PRM_DEVICE_MOD,
			v_info->vdd_info->omap_abb_reg_val.prm_abb_ldo_ctrl_idx);
			ret = omap4_abb_change_opp(v_info->vdd_info);
		}
	} else
		ret = -EINVAL;

out:
	return ret;
}

static struct notifier_block abb_mpu_volt_notifier_block = {
	.notifier_call = omap_abb_notify_voltage,
};

static struct notifier_block abb_iva_volt_notifier_block = {
	.notifier_call = omap_abb_notify_voltage,
};

/*
 * omap_abb_init - initialize Adaptive Body-Bias LDO
 * @vdd_info : pointer to the voltage domain we are initializing
 *
 * Currently only supports OMAP4.  Enables active Forward Body-Bias by default
 * on VDD_MPU only, not on VDD_IVA.  Disables sleep Reverse Body-Bias and
 * active Reverse Body-Bias for both voltage domains.  Registers voltage
 * notifiers for affected voltage domains.  Returns 0 on success, negative
 * integers otherwise.
 */
static int omap_abb_init(struct omap_vdd_info *vdd_info)
{
	int ret = 0;
	struct clk *sys_ck;
	u32 sr2_wt_cnt_val;

	if (!cpu_is_omap3630() && !cpu_is_omap44xx()) {
		pr_info("%s: CPU does not support ABB feature.\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	/* VDD_CORE does not support ABB */
	if(!strcmp("core", vdd_info->voltdm.name)) {
		ret = -EINVAL;
		goto out;
	}

	if (cpu_is_omap3630())
		sys_ck = clk_get(NULL, "sys_ck");
	else /* cpu_is_omap44xx() */
		sys_ck = clk_get(NULL, "sys_clkin_ck");

	if (IS_ERR(sys_ck)) {
		pr_warning("%s: Could not get the sys clk to calculate"
			"various params\n", __func__);
		ret = -ENODEV;
		goto out;
	}

	sr2_wt_cnt_val = clk_get_rate(sys_ck);
	sr2_wt_cnt_val = sr2_wt_cnt_val / 1000000 / 16;

	if (cpu_is_omap3630()) {
		prm_rmw_mod_reg_bits(OMAP3630_SR2_WTCNT_VALUE_MASK,
				(sr2_wt_cnt_val << OMAP3630_SR2_WTCNT_VALUE_SHIFT),
				OMAP3430_GR_MOD,
				vdd_info->omap_abb_reg_val.prm_abb_ldo_ctrl_idx);

		/* enable fbb by default */
		prm_set_mod_reg_bits(OMAP3630_ACTIVE_FBB_SEL_MASK,
				OMAP3430_GR_MOD,
				vdd_info->omap_abb_reg_val.prm_abb_ldo_ctrl_idx);
	} else { /* cpu_is_omap44xx() */
		prm_rmw_mod_reg_bits(OMAP4430_SR2_WTCNT_VALUE_MASK,
				(sr2_wt_cnt_val << OMAP4430_SR2_WTCNT_VALUE_SHIFT),
				OMAP4430_PRM_DEVICE_MOD,
				vdd_info->omap_abb_reg_val.prm_abb_ldo_setup_idx);

		/* enable fbb by default */
		prm_set_mod_reg_bits(OMAP4430_ACTIVE_FBB_SEL_MASK,
				OMAP4430_PRM_DEVICE_MOD,
				vdd_info->omap_abb_reg_val.prm_abb_ldo_setup_idx);

		/* do not enable active rbb by default */
		prm_clear_mod_reg_bits(OMAP4430_ACTIVE_RBB_SEL_MASK,
				OMAP4430_PRM_DEVICE_MOD,
				vdd_info->omap_abb_reg_val.prm_abb_ldo_setup_idx);

		/* do not enable sleep rbb by default */
		prm_clear_mod_reg_bits(OMAP4430_SLEEP_RBB_SEL_MASK,
				OMAP4430_PRM_DEVICE_MOD,
				vdd_info->omap_abb_reg_val.prm_abb_ldo_setup_idx);
	}

	if(!strcmp("mpu", vdd_info->voltdm.name)) {
		omap_voltage_register_notifier(&vdd_info->voltdm,
				&abb_mpu_volt_notifier_block);
	} else if(!strcmp("iva", vdd_info->voltdm.name)) {
		omap_voltage_register_notifier(&(vdd_info->voltdm),
				&abb_iva_volt_notifier_block);
	} else {
		pr_warning("%s: Invalid VDD specified\n", __func__);
		ret = -EINVAL;
	}

out:
	return ret;
}
#endif

/* OMAP3 specific voltage init functions */
/*
 * Intializes the voltage controller registers with the PMIC and board
 * specific parameters and voltage setup times for OMAP3. If the board
 * file does not populate the voltage controller parameters through
 * omap3_pm_init_vc, default values specified in vc_config is used.
 */
static void __init omap3_init_voltagecontroller(void)
{
	u8 vsel_on, vsel_onlp, vsel_ret, vsel_off;
	u8 impu, icore;
	impu = omap_vdd_id("mpu");
	icore = omap_vdd_id("core");

	voltage_write_reg(OMAP3_PRM_VC_SMPS_SA_OFFSET,
			(vdd_info[impu].pmic->i2c_addr <<
			 OMAP3430_PRM_VC_SMPS_SA_SA0_SHIFT) |
			(vdd_info[icore].pmic->i2c_addr <<
			 OMAP3430_PRM_VC_SMPS_SA_SA1_SHIFT));
	voltage_write_reg(OMAP3_PRM_VC_SMPS_VOL_RA_OFFSET,
		(vdd_info[impu].pmic->i2c_vreg << OMAP3430_VOLRA0_SHIFT) |
		(vdd_info[icore].pmic->i2c_vreg << OMAP3430_VOLRA1_SHIFT));

	vsel_on = vdd_info[impu].pmic->uv_to_vsel(vc_config.vdd0_on);
	vsel_onlp = vdd_info[impu].pmic->uv_to_vsel(vc_config.vdd0_onlp);
	vsel_ret = vdd_info[impu].pmic->uv_to_vsel(vc_config.vdd0_ret);
	vsel_off = vdd_info[impu].pmic->uv_to_vsel(vc_config.vdd0_off);
	voltage_write_reg(OMAP3_PRM_VC_CMD_VAL_0_OFFSET,
			(vsel_on << OMAP3430_VC_CMD_ON_SHIFT) |
			(vsel_onlp << OMAP3430_VC_CMD_ONLP_SHIFT) |
			(vsel_ret << OMAP3430_VC_CMD_RET_SHIFT) |
			(vsel_off << OMAP3430_VC_CMD_OFF_SHIFT));

	vsel_on = vdd_info[icore].pmic->uv_to_vsel(vc_config.vdd1_on);
	vsel_onlp = vdd_info[icore].pmic->uv_to_vsel(vc_config.vdd1_onlp);
	vsel_ret = vdd_info[icore].pmic->uv_to_vsel(vc_config.vdd1_ret);
	vsel_off = vdd_info[icore].pmic->uv_to_vsel(vc_config.vdd1_off);
	voltage_write_reg(OMAP3_PRM_VC_CMD_VAL_1_OFFSET,
			(vsel_on << OMAP3430_VC_CMD_ON_SHIFT) |
			(vsel_onlp << OMAP3430_VC_CMD_ONLP_SHIFT) |
			(vsel_ret << OMAP3430_VC_CMD_RET_SHIFT) |
			(vsel_off << OMAP3430_VC_CMD_OFF_SHIFT));
	voltage_write_reg(OMAP3_PRM_VC_CH_CONF_OFFSET,
			OMAP3430_CMD1_MASK | OMAP3430_RAV1_MASK);
	voltage_write_reg(OMAP3_PRM_VC_I2C_CFG_OFFSET,
			OMAP3430_MCODE_SHIFT | OMAP3430_HSEN_MASK);
	/* Write setup times */
	voltage_write_reg(OMAP3_PRM_CLKSETUP_OFFSET, vc_config.clksetup);
	voltage_write_reg(OMAP3_PRM_VOLTSETUP1_OFFSET,
			(vc_config.voltsetup_time2 <<
			 OMAP3430_SETUP_TIME2_SHIFT) |
			(vc_config.voltsetup_time1 <<
			 OMAP3430_SETUP_TIME1_SHIFT));
	voltage_write_reg(OMAP3_PRM_VOLTOFFSET_OFFSET, vc_config.voltoffset);
	voltage_write_reg(OMAP3_PRM_VOLTSETUP2_OFFSET, vc_config.voltsetup2);
}

static __init struct omap_volt_data *get_init_voltage(struct voltagedomain *voltdm)
{
	struct omap_opp *opp;
	struct omap_vdd_info *vdd;
	unsigned long freq;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	freq = vdd->volt_clk->rate;
	opp = opp_find_freq_ceil(vdd->opp_dev, &freq);
	if (IS_ERR(opp)) {
		pr_warning("%s: Unable to find OPP for vdd_%s freq%ld\n",
			__func__, voltdm->name, freq);
		return 0;
	}

	/*
	 * Use higher freq voltage even if an exact match is not available
	 * we are probably masking a clock framework bug, so warn
	 */
	if (unlikely((freq / 1000000) != (vdd->volt_clk->rate / 1000000)))
		pr_debug("%s: Available freq %ld != dpll freq %ld.\n",
			__func__, freq, vdd->volt_clk->rate);

	return omap_voltage_get_voltdata(voltdm, opp_get_voltage(opp));
}

/* Sets up all the VDD related info for OMAP3 */
static void __init omap3_vdd_data_configure(struct omap_vdd_info *vdd)
{
	struct omap_volt_data *volt_data;
	struct clk *sys_ck;
	u32 sys_clk_speed, timeout_val, waittime;

	if (!strcmp(vdd->voltdm.name, "mpu")) {
		if (cpu_is_omap3630()) {
			vdd->vp_reg.vlimitto_vddmin =
					vdd->pmic->vp_vlimitto_vddmin;
			vdd->vp_reg.vlimitto_vddmax =
					vdd->pmic->vp_vlimitto_vddmax;
			vdd->volt_data = omap36xx_vdd1_volt_data;
			vdd->volt_data_count =
					ARRAY_SIZE(omap36xx_vdd1_volt_data);
		} else {
			vdd->vp_reg.vlimitto_vddmin =
					vdd->pmic->vp_vlimitto_vddmin;
			vdd->vp_reg.vlimitto_vddmax =
					vdd->pmic->vp_vlimitto_vddmax;
			vdd->volt_data = omap34xx_vdd1_volt_data;
			vdd->volt_data_count =
					ARRAY_SIZE(omap34xx_vdd1_volt_data);
			vdd->dep_vdd_info = omap34xx_vdd1_dep_info;
			vdd->nr_dep_vdd = ARRAY_SIZE(omap34xx_vdd1_dep_info);
		}
		vdd->volt_clk = clk_get(NULL, "dpll1_ck");
		WARN(IS_ERR(vdd->volt_clk), "unable to get clock for vdd_%s\n",
				vdd->voltdm.name);
		vdd->opp_dev = omap2_get_mpuss_device();
		vdd->vp_reg.tranxdone_status = OMAP3430_VP1_TRANXDONE_ST_MASK;
		vdd->cmdval_reg = OMAP3_PRM_VC_CMD_VAL_0_OFFSET;
		vdd->vdd_sr_reg = vdd->pmic->i2c_vreg;
	} else if (!strcmp(vdd->voltdm.name, "core")) {
		if (cpu_is_omap3630()) {
			vdd->vp_reg.vlimitto_vddmin =
					vdd->pmic->vp_vlimitto_vddmin;
			vdd->vp_reg.vlimitto_vddmax =
					vdd->pmic->vp_vlimitto_vddmax;
			vdd->volt_data = omap36xx_vdd2_volt_data;
			vdd->volt_data_count =
					ARRAY_SIZE(omap36xx_vdd2_volt_data);
		} else {
			vdd->vp_reg.vlimitto_vddmin =
					vdd->pmic->vp_vlimitto_vddmin;
			vdd->vp_reg.vlimitto_vddmax =
					vdd->pmic->vp_vlimitto_vddmax;
			vdd->volt_data = omap34xx_vdd2_volt_data;
			vdd->volt_data_count =
					ARRAY_SIZE(omap34xx_vdd2_volt_data);
		}
		vdd->volt_clk = clk_get(NULL, "l3_ick");
		WARN(IS_ERR(vdd->volt_clk), "unable to get clock for vdd_%s\n",
				vdd->voltdm.name);
		vdd->opp_dev = omap2_get_l3_device();
		vdd->vp_reg.tranxdone_status = OMAP3430_VP2_TRANXDONE_ST_MASK;
		vdd->cmdval_reg = OMAP3_PRM_VC_CMD_VAL_1_OFFSET;
		vdd->vdd_sr_reg = vdd->pmic->i2c_vreg;
	} else {
		pr_warning("%s: vdd_%s does not exisit in OMAP3\n",
			__func__, vdd->voltdm.name);
		return;
	}

	vdd->prm_irqst_reg = OMAP3_PRM_IRQSTATUS_MPU_OFFSET;
	vdd->ocp_mod = OCP_MOD;

	volt_data = vdd->nominal_volt = vdd->curr_volt = get_init_voltage(&vdd->voltdm);

	if (IS_ERR_OR_NULL(volt_data)) {
		pr_warning("%s: Unable to get volt table for vdd_%s at init",
			__func__, vdd->voltdm.name);
		return;
	}
	/*
	 * Sys clk rate is require to calculate vp timeout value and
	 * smpswaittimemin and smpswaittimemax.
	 */
	sys_ck = clk_get(NULL, "sys_ck");
	if (IS_ERR(sys_ck)) {
		pr_warning("%s: Could not get the sys clk to calculate"
			"various vdd_%s params\n", __func__, vdd->voltdm.name);
		return;
	}
	sys_clk_speed = clk_get_rate(sys_ck);
	clk_put(sys_ck);
	/* Divide to avoid overflow */
	sys_clk_speed /= 1000;

	/* VPCONFIG bit fields */
	vdd->vp_reg.vpconfig_erroroffset = (vdd->pmic->vp_config_erroroffset <<
				 OMAP3430_ERROROFFSET_SHIFT);
	vdd->vp_reg.vpconfig_errorgain = volt_data->vp_errgain;
	vdd->vp_reg.vpconfig_errorgain_mask = OMAP3430_ERRORGAIN_MASK;
	vdd->vp_reg.vpconfig_errorgain_shift = OMAP3430_ERRORGAIN_SHIFT;
	vdd->vp_reg.vpconfig_initvoltage_shift = OMAP3430_INITVOLTAGE_SHIFT;
	vdd->vp_reg.vpconfig_initvoltage_mask = OMAP3430_INITVOLTAGE_MASK;
	vdd->vp_reg.vpconfig_timeouten = OMAP3430_TIMEOUTEN_MASK;
	vdd->vp_reg.vpconfig_initvdd = OMAP3430_INITVDD_MASK;
	vdd->vp_reg.vpconfig_forceupdate = OMAP3430_FORCEUPDATE_MASK;
	vdd->vp_reg.vpconfig_vpenable = OMAP3430_VPENABLE_MASK;

	/* VSTEPMIN VSTEPMAX bit fields */
	waittime = ((vdd->pmic->step_size / vdd->pmic->slew_rate) *
				sys_clk_speed) / 1000;
	vdd->vp_reg.vstepmin_smpswaittimemin = waittime;
	vdd->vp_reg.vstepmax_smpswaittimemax = waittime;
	vdd->vp_reg.vstepmin_stepmin = vdd->pmic->vp_vstepmin_vstepmin;
	vdd->vp_reg.vstepmax_stepmax = vdd->pmic->vp_vstepmax_vstepmax;
	vdd->vp_reg.vstepmin_smpswaittimemin_shift =
				OMAP3430_SMPSWAITTIMEMIN_SHIFT;
	vdd->vp_reg.vstepmax_smpswaittimemax_shift =
				OMAP3430_SMPSWAITTIMEMAX_SHIFT;
	vdd->vp_reg.vstepmin_stepmin_shift = OMAP3430_VSTEPMIN_SHIFT;
	vdd->vp_reg.vstepmax_stepmax_shift = OMAP3430_VSTEPMAX_SHIFT;

	/* VLIMITTO bit fields */
	timeout_val = (sys_clk_speed * vdd->pmic->vp_vlimitto_timeout_us)
		/ 1000;
	vdd->vp_reg.vlimitto_timeout = timeout_val;
	vdd->vp_reg.vlimitto_vddmin_shift = OMAP3430_VDDMIN_SHIFT;
	vdd->vp_reg.vlimitto_vddmax_shift = OMAP3430_VDDMAX_SHIFT;
	vdd->vp_reg.vlimitto_timeout_shift = OMAP3430_TIMEOUT_SHIFT;
}

/* OMAP4 specific voltage init functions */
static void __init omap4_init_voltagecontroller(void)
{
	u8 on_cmd, onlp_cmd, ret_cmd, off_cmd;
	u8 impu, icore, iiva;
	impu = omap_vdd_id("mpu");
	icore = omap_vdd_id("core");
	iiva = omap_vdd_id("iva");

	voltage_write_reg(OMAP4_PRM_VC_SMPS_SA_OFFSET,
			(vdd_info[icore].pmic->i2c_addr <<
			 OMAP4430_SA_VDD_CORE_L_0_6_SHIFT) |
			(vdd_info[iiva].pmic->i2c_addr <<
			 OMAP4430_SA_VDD_IVA_L_PRM_VC_SMPS_SA_SHIFT) |
			(vdd_info[impu].pmic->i2c_addr <<
			 OMAP4430_SA_VDD_MPU_L_PRM_VC_SMPS_SA_SHIFT));
	voltage_write_reg(OMAP4_PRM_VC_VAL_SMPS_RA_VOL_OFFSET,
			(vdd_info[impu].pmic->i2c_vreg <<
			 OMAP4430_VOLRA_VDD_MPU_L_SHIFT) |
			(vdd_info[iiva].pmic->i2c_vreg <<
			 OMAP4430_VOLRA_VDD_IVA_L_SHIFT) |
			(vdd_info[icore].pmic->i2c_vreg <<
			 OMAP4430_VOLRA_VDD_CORE_L_SHIFT));
	voltage_write_reg(OMAP4_PRM_VC_VAL_SMPS_RA_CMD_OFFSET,
			(vdd_info[impu].pmic->i2c_cmdreg <<
			 OMAP4430_VOLRA_VDD_MPU_L_SHIFT) |
			(vdd_info[iiva].pmic->i2c_cmdreg <<
			 OMAP4430_VOLRA_VDD_IVA_L_SHIFT) |
			(vdd_info[icore].pmic->i2c_cmdreg <<
			 OMAP4430_VOLRA_VDD_CORE_L_SHIFT));
	voltage_write_reg(OMAP4_PRM_VC_CFG_CHANNEL_OFFSET,
			OMAP4430_RAV_VDD_MPU_L_MASK |
			OMAP4430_CMD_VDD_MPU_L_MASK |
			((vdd_info[impu].pmic->i2c_addr ==
				vdd_info[icore].pmic->i2c_addr) ?
					0 : OMAP4430_SA_VDD_MPU_L_MASK) |
			OMAP4430_RAV_VDD_IVA_L_MASK |
			OMAP4430_CMD_VDD_IVA_L_MASK |
			((vdd_info[iiva].pmic->i2c_addr ==
				vdd_info[icore].pmic->i2c_addr) ?
					0 : OMAP4430_SA_VDD_IVA_L_MASK) |
			OMAP4430_RAV_VDD_CORE_L_MASK |
			OMAP4430_CMD_VDD_CORE_L_MASK |
			((vdd_info[iiva].pmic->i2c_addr ==
			vdd_info[icore].pmic->i2c_addr) ?
					0 : OMAP4430_SA_VDD_CORE_L_MASK));
	/*
	 * Configure SR I2C in HS Mode. Is there really a need to configure
	 * i2c in the normal mode??
	 */
	voltage_write_reg(OMAP4_PRM_VC_CFG_I2C_CLK_OFFSET,
		(0x0B << OMAP4430_HSSCLL_SHIFT |
		0x00 << OMAP4430_HSSCLH_SHIFT |
		0x28 << OMAP4430_SCLL_SHIFT |
		0x2C << OMAP4430_SCLH_SHIFT));

	voltage_write_reg(OMAP4_PRM_VC_CFG_I2C_MODE_OFFSET,
		0x8 << OMAP4430_HSMCODE_SHIFT);


	/* setup the VOLTSETUP* registers for RET and SLEEP */
	/*
	 * TODO: Relook at the prescal and count settings. For now conservative
	 * values of 0x3 for prescal, translating to ramp-up/down counter being
	 * incremented every 512 system clock cycles and a count value of 0xF is
	 * used.
	 */
	voltage_write_reg(OMAP4_PRM_VOLTSETUP_CORE_RET_SLEEP_OFFSET,
		(0x3 << OMAP4430_RAMP_DOWN_PRESCAL_SHIFT) |
		(0x3 << OMAP4430_RAMP_UP_PRESCAL_SHIFT) |
		(0xF << OMAP4430_RAMP_DOWN_COUNT_SHIFT) |
		(0xF << OMAP4430_RAMP_UP_COUNT_SHIFT));
	voltage_write_reg(OMAP4_PRM_VOLTSETUP_IVA_RET_SLEEP_OFFSET,
		(0x3 << OMAP4430_RAMP_DOWN_PRESCAL_SHIFT) |
		(0x3 << OMAP4430_RAMP_UP_PRESCAL_SHIFT) |
		(0xF << OMAP4430_RAMP_DOWN_COUNT_SHIFT) |
		(0xF << OMAP4430_RAMP_UP_COUNT_SHIFT));
	voltage_write_reg(OMAP4_PRM_VOLTSETUP_MPU_RET_SLEEP_OFFSET,
		(0x3 << OMAP4430_RAMP_DOWN_PRESCAL_SHIFT) |
		(0x3 << OMAP4430_RAMP_UP_PRESCAL_SHIFT) |
		(0xF << OMAP4430_RAMP_DOWN_COUNT_SHIFT) |
		(0xF << OMAP4430_RAMP_UP_COUNT_SHIFT));

	/* setup the VOLTSETUP* registers for OFF */
	voltage_write_reg(OMAP4_PRM_VOLTSETUP_CORE_OFF_OFFSET,
		(0x3 << OMAP4430_RAMP_DOWN_PRESCAL_SHIFT) |
		(0x3 << OMAP4430_RAMP_UP_PRESCAL_SHIFT) |
		(0xF << OMAP4430_RAMP_DOWN_COUNT_SHIFT) |
		(0xF << OMAP4430_RAMP_UP_COUNT_SHIFT));
	voltage_write_reg(OMAP4_PRM_VOLTSETUP_IVA_OFF_OFFSET,
		(0x3 << OMAP4430_RAMP_DOWN_PRESCAL_SHIFT) |
		(0x3 << OMAP4430_RAMP_UP_PRESCAL_SHIFT) |
		(0xF << OMAP4430_RAMP_DOWN_COUNT_SHIFT) |
		(0xF << OMAP4430_RAMP_UP_COUNT_SHIFT));
	voltage_write_reg(OMAP4_PRM_VOLTSETUP_MPU_OFF_OFFSET,
		(0x3 << OMAP4430_RAMP_DOWN_PRESCAL_SHIFT) |
		(0x3 << OMAP4430_RAMP_UP_PRESCAL_SHIFT) |
		(0xF << OMAP4430_RAMP_DOWN_COUNT_SHIFT) |
		(0xF << OMAP4430_RAMP_UP_COUNT_SHIFT));

	on_cmd = vdd_info[impu].pmic->onforce_cmd(
		vdd_info[impu].pmic->uv_to_vsel(vc_config.vdd0_on));
	onlp_cmd = vdd_info[impu].pmic->sleepforce_cmd(
		vdd_info[impu].pmic->uv_to_vsel(vc_config.vdd0_onlp));
	ret_cmd = vdd_info[impu].pmic->sleepforce_cmd(
		vdd_info[impu].pmic->uv_to_vsel(vc_config.vdd0_ret));
	off_cmd = vdd_info[impu].pmic->sleepforce_cmd(
		vdd_info[impu].pmic->uv_to_vsel(vc_config.vdd0_off));
	voltage_write_reg(OMAP4_PRM_VC_VAL_CMD_VDD_MPU_L_OFFSET,
			(on_cmd << OMAP4430_ON_SHIFT) |
			(onlp_cmd << OMAP4430_ONLP_SHIFT) |
			(ret_cmd << OMAP4430_RET_SHIFT) |
			(off_cmd << OMAP4430_OFF_SHIFT));

	on_cmd = vdd_info[icore].pmic->onforce_cmd(
		vdd_info[icore].pmic->uv_to_vsel(vc_config.vdd1_on));
	onlp_cmd = vdd_info[icore].pmic->sleepforce_cmd(
		vdd_info[icore].pmic->uv_to_vsel(vc_config.vdd1_onlp));
	ret_cmd = vdd_info[icore].pmic->sleepforce_cmd(
		vdd_info[icore].pmic->uv_to_vsel(vc_config.vdd1_ret));
	off_cmd = vdd_info[icore].pmic->sleepforce_cmd(
		vdd_info[icore].pmic->uv_to_vsel(vc_config.vdd1_off));
	voltage_write_reg(OMAP4_PRM_VC_VAL_CMD_VDD_CORE_L_OFFSET,
			(on_cmd << OMAP4430_ON_SHIFT) |
			(onlp_cmd << OMAP4430_ONLP_SHIFT) |
			(ret_cmd << OMAP4430_RET_SHIFT) |
			(off_cmd << OMAP4430_OFF_SHIFT));

	on_cmd = vdd_info[iiva].pmic->onforce_cmd(
		vdd_info[iiva].pmic->uv_to_vsel(vc_config.vdd2_on));
	onlp_cmd = vdd_info[iiva].pmic->sleepforce_cmd(
		vdd_info[iiva].pmic->uv_to_vsel(vc_config.vdd2_onlp));
	ret_cmd = vdd_info[iiva].pmic->sleepforce_cmd(
		vdd_info[iiva].pmic->uv_to_vsel(vc_config.vdd2_ret));
	off_cmd = vdd_info[iiva].pmic->sleepforce_cmd(
		vdd_info[iiva].pmic->uv_to_vsel(vc_config.vdd2_off));
	voltage_write_reg(OMAP4_PRM_VC_VAL_CMD_VDD_IVA_L_OFFSET,
			(on_cmd << OMAP4430_ON_SHIFT) |
			(onlp_cmd << OMAP4430_ONLP_SHIFT) |
			(ret_cmd << OMAP4430_RET_SHIFT) |
			(off_cmd << OMAP4430_OFF_SHIFT));
}

/* Sets up all the VDD related info for OMAP4 */
static void __init omap4_vdd_data_configure(struct omap_vdd_info *vdd)
{
	struct omap_volt_data *volt_data;
	struct clk *sys_ck;
	u32 sys_clk_speed, timeout_val, waittime;

	if (!strcmp(vdd->voltdm.name, "mpu")) {
		vdd->vp_reg.vlimitto_vddmin = vdd->pmic->vp_vlimitto_vddmin;
		vdd->vp_reg.vlimitto_vddmax = vdd->pmic->vp_vlimitto_vddmax;
		vdd->volt_data = omap44xx_vdd_mpu_volt_data;
		vdd->volt_data_count = ARRAY_SIZE(omap44xx_vdd_mpu_volt_data);
		vdd->dep_vdd_info = omap44xx_vddmpu_dep_info;
		vdd->nr_dep_vdd = ARRAY_SIZE(omap44xx_vddmpu_dep_info);
		vdd->volt_clk = clk_get(NULL, "dpll_mpu_ck");
		WARN(IS_ERR(vdd->volt_clk), "unable to get clock for vdd_%s\n",
				vdd->voltdm.name);
		vdd->opp_dev = omap2_get_mpuss_device();
		vdd->vp_reg.tranxdone_status =
				OMAP4430_VP_MPU_TRANXDONE_ST_MASK;
		vdd->cmdval_reg = OMAP4_PRM_VC_VAL_CMD_VDD_MPU_L_OFFSET;
		vdd->vdd_sr_reg = vdd->pmic->i2c_vreg;
		vdd->prm_irqst_reg = OMAP4_PRM_IRQSTATUS_MPU_2_OFFSET;
	} else if (!strcmp(vdd->voltdm.name, "core")) {
		vdd->vp_reg.vlimitto_vddmin = vdd->pmic->vp_vlimitto_vddmin;
		vdd->vp_reg.vlimitto_vddmax = vdd->pmic->vp_vlimitto_vddmax;
		vdd->volt_data = omap44xx_vdd_core_volt_data;
		vdd->volt_data_count = ARRAY_SIZE(omap44xx_vdd_core_volt_data);
		vdd->volt_clk = clk_get(NULL, "l3_div_ck");
		WARN(IS_ERR(vdd->volt_clk), "unable to get clock for vdd_%s\n",
				vdd->voltdm.name);
		vdd->opp_dev = omap2_get_l3_device();
		vdd->vp_reg.tranxdone_status =
				OMAP4430_VP_CORE_TRANXDONE_ST_MASK;
		vdd->cmdval_reg = OMAP4_PRM_VC_VAL_CMD_VDD_CORE_L_OFFSET;
		vdd->vdd_sr_reg = vdd->pmic->i2c_vreg;
		vdd->prm_irqst_reg = OMAP4_PRM_IRQSTATUS_MPU_OFFSET;
	} else if (!strcmp(vdd->voltdm.name, "iva")) {
		vdd->vp_reg.vlimitto_vddmin = vdd->pmic->vp_vlimitto_vddmin;
		vdd->vp_reg.vlimitto_vddmax = vdd->pmic->vp_vlimitto_vddmax;
		vdd->volt_data = omap44xx_vdd_iva_volt_data;
		vdd->volt_data_count = ARRAY_SIZE(omap44xx_vdd_iva_volt_data);
		vdd->dep_vdd_info = omap44xx_vddiva_dep_info;
		vdd->nr_dep_vdd = ARRAY_SIZE(omap44xx_vddiva_dep_info);
		vdd->volt_clk = clk_get(NULL, "dpll_iva_m5x2_ck");
		WARN(IS_ERR(vdd->volt_clk), "unable to get clock for vdd_%s\n",
				vdd->voltdm.name);
		vdd->opp_dev = omap2_get_iva_device();
		vdd->vp_reg.tranxdone_status =
			OMAP4430_VP_IVA_TRANXDONE_ST_MASK;
		vdd->cmdval_reg = OMAP4_PRM_VC_VAL_CMD_VDD_IVA_L_OFFSET;
		vdd->vdd_sr_reg = vdd->pmic->i2c_vreg;
		vdd->prm_irqst_reg = OMAP4_PRM_IRQSTATUS_MPU_OFFSET;
	} else {
		pr_warning("%s: vdd_%s does not exisit in OMAP4\n",
			__func__, vdd->voltdm.name);
		return;
	}

	vdd->ocp_mod = OMAP4430_PRM_OCP_SOCKET_MOD;
	volt_data = vdd->nominal_volt =  vdd->curr_volt = get_init_voltage(&vdd->voltdm);
	if (IS_ERR(volt_data)) {
		pr_warning("%s: Unable to get volt table for vdd_%s at init",
			__func__, vdd->voltdm.name);
		return;
	}
	/*
	 * Sys clk rate is require to calculate vp timeout value and
	 * smpswaittimemin and smpswaittimemax.
	 */
	sys_ck = clk_get(NULL, "sys_clkin_ck");
	if (IS_ERR(sys_ck)) {
		pr_warning("%s: Could not get the sys clk to calculate"
			"various vdd_%s params\n", __func__, vdd->voltdm.name);
		return;
	}
	sys_clk_speed = clk_get_rate(sys_ck);
	clk_put(sys_ck);
	/* Divide to avoid overflow */
	sys_clk_speed /= 1000;

	/* VPCONFIG bit fields */
	vdd->vp_reg.vpconfig_erroroffset =
				(vdd->pmic->vp_config_erroroffset <<
				 OMAP4430_ERROROFFSET_SHIFT);
	vdd->vp_reg.vpconfig_errorgain = volt_data->vp_errgain;
	vdd->vp_reg.vpconfig_errorgain_mask = OMAP4430_ERRORGAIN_MASK;
	vdd->vp_reg.vpconfig_errorgain_shift = OMAP4430_ERRORGAIN_SHIFT;
	vdd->vp_reg.vpconfig_initvoltage_shift = OMAP4430_INITVOLTAGE_SHIFT;
	vdd->vp_reg.vpconfig_initvoltage_mask = OMAP4430_INITVOLTAGE_MASK;
	vdd->vp_reg.vpconfig_timeouten = OMAP4430_TIMEOUTEN_MASK;
	vdd->vp_reg.vpconfig_initvdd = OMAP4430_INITVDD_MASK;
	vdd->vp_reg.vpconfig_forceupdate = OMAP4430_FORCEUPDATE_MASK;
	vdd->vp_reg.vpconfig_vpenable = OMAP4430_VPENABLE_MASK;

	/* VSTEPMIN VSTEPMAX bit fields */
	waittime = ((vdd->pmic->step_size / vdd->pmic->slew_rate) *
				sys_clk_speed) / 1000;
	vdd->vp_reg.vstepmin_smpswaittimemin = waittime;
	vdd->vp_reg.vstepmax_smpswaittimemax = waittime;
	vdd->vp_reg.vstepmin_stepmin = vdd->pmic->vp_vstepmin_vstepmin;
	vdd->vp_reg.vstepmax_stepmax = vdd->pmic->vp_vstepmax_vstepmax;
	vdd->vp_reg.vstepmin_smpswaittimemin_shift =
		OMAP4430_SMPSWAITTIMEMIN_SHIFT;
	vdd->vp_reg.vstepmax_smpswaittimemax_shift =
		OMAP4430_SMPSWAITTIMEMAX_SHIFT;
	vdd->vp_reg.vstepmin_stepmin_shift = OMAP4430_VSTEPMIN_SHIFT;
	vdd->vp_reg.vstepmax_stepmax_shift = OMAP4430_VSTEPMAX_SHIFT;

	/* VLIMITTO bit fields */
	timeout_val = (sys_clk_speed * vdd->pmic->vp_vlimitto_timeout_us)
			/ 1000;
	vdd->vp_reg.vlimitto_timeout = timeout_val;
	vdd->vp_reg.vlimitto_vddmin_shift = OMAP4430_VDDMIN_SHIFT;
	vdd->vp_reg.vlimitto_vddmax_shift = OMAP4430_VDDMAX_SHIFT;
	vdd->vp_reg.vlimitto_timeout_shift = OMAP4430_TIMEOUT_SHIFT;
}

/* Generic voltage init functions */
static void __init init_voltageprocessor(struct omap_vdd_info *vdd)
{
	u32 vpconfig;

	vpconfig = vdd->vp_reg.vpconfig_erroroffset |
			(vdd->vp_reg.vpconfig_errorgain <<
			vdd->vp_reg.vpconfig_errorgain_shift) |
			vdd->vp_reg.vpconfig_timeouten;

	voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);

	voltage_write_reg(vdd->vp_offs.vstepmin,
			(vdd->vp_reg.vstepmin_smpswaittimemin <<
			vdd->vp_reg.vstepmin_smpswaittimemin_shift) |
			(vdd->vp_reg.vstepmin_stepmin <<
			vdd->vp_reg.vstepmin_stepmin_shift));

	voltage_write_reg(vdd->vp_offs.vstepmax,
			(vdd->vp_reg.vstepmax_smpswaittimemax <<
			vdd->vp_reg.vstepmax_smpswaittimemax_shift) |
			(vdd->vp_reg.vstepmax_stepmax <<
			vdd->vp_reg.vstepmax_stepmax_shift));

	voltage_write_reg(vdd->vp_offs.vlimitto,
			(vdd->vp_reg.vlimitto_vddmax <<
			vdd->vp_reg.vlimitto_vddmax_shift) |
			(vdd->vp_reg.vlimitto_vddmin <<
			vdd->vp_reg.vlimitto_vddmin_shift) |
			(vdd->vp_reg.vlimitto_timeout <<
			vdd->vp_reg.vlimitto_timeout_shift));

	/* Set the init voltage */
	vp_latch_vsel(vdd);

	vpconfig = voltage_read_reg(vdd->vp_offs.vpconfig);
	/* Force update of voltage */
	voltage_write_reg(vdd->vp_offs.vpconfig,
			(vpconfig | vdd->vp_reg.vpconfig_forceupdate));
	/* Clear force bit */
	voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);
}

#ifdef CONFIG_OMAP_ABB
/**
  * omap_abb_enable - Sets ABB_LDO_SETUP.SR2EN bit
  * Enables ABB.
  */
static void omap_abb_enable(struct omap_vdd_info *vdd_info)
{
	if (cpu_is_omap3630())
		prm_set_mod_reg_bits(OMAP3630_SR2EN_MASK,
			OMAP3430_GR_MOD,
			vdd_info->omap_abb_reg_val.prm_abb_ldo_ctrl_idx);
	else /* cpu_is_omap44xx() */
		prm_set_mod_reg_bits(OMAP4430_SR2EN_MASK,
			OMAP4430_PRM_DEVICE_MOD,
			vdd_info->omap_abb_reg_val.prm_abb_ldo_setup_idx);
}

/**
  * omap_abb_disable - Clears ABB_LDO_SETUP.SR2EN bit
  * Disables ABB.
  */
static void omap_abb_disable(struct omap_vdd_info *vdd_info)
{
	if (cpu_is_omap3630())
		prm_clear_mod_reg_bits(OMAP3630_SR2EN_MASK,
			OMAP3430_GR_MOD,
			vdd_info->omap_abb_reg_val.prm_abb_ldo_ctrl_idx);
	else /* cpu_is_omap44xx() */
		prm_clear_mod_reg_bits(OMAP4430_SR2EN_MASK,
			OMAP4430_PRM_DEVICE_MOD,
			vdd_info->omap_abb_reg_val.prm_abb_ldo_setup_idx);
}

/**
 * omap*_abb_change_opp - set LDO mode & poll status until transition completes
 * @vdd_info : voltage domain info for the VDD that is transitioning
 *
 * Enables ABB LDO if not done already, changes the operating mode (mode
 * programming should already be done by notification handler), polls on
 * status until change completes and clears status bits.  Returns 0 on
 * success, -EBUSY if timeout occurs.
 */
static int omap3_abb_change_opp(struct omap_vdd_info *vdd_info)
{
	int ret = 0;
	int timeout;

	/* enable ABB ldo if not done already */
	if (prm_read_mod_reg(OMAP3430_GR_MOD,
			vdd_info->omap_abb_reg_val.prm_abb_ldo_ctrl_idx) &
			OMAP3630_SR2_WTCNT_VALUE_MASK)
		omap_abb_enable(vdd_info);

	/* clear ABB ldo interrupt status */
	prm_clear_mod_reg_bits(vdd_info->omap_abb_reg_val.abb_done_st_mask,
			OCP_MOD,
			vdd_info->omap_abb_reg_val.prm_irqstatus_mpu);

	/* enable ABB LDO OPP change */
	prm_set_mod_reg_bits(OMAP3630_OPP_CHANGE_MASK,
		OMAP3430_GR_MOD,
		vdd_info->omap_abb_reg_val.prm_abb_ldo_setup_idx);

	timeout = 0;

	/* wait until OPP change completes */
	while ((timeout < ABB_TRANXDONE_TIMEOUT) &&
		(!(prm_read_mod_reg(OCP_MOD,
			vdd_info->omap_abb_reg_val.prm_irqstatus_mpu) &
			vdd_info->omap_abb_reg_val.abb_done_st_mask))) {
		udelay(1);
		timeout++;
	}

	if (timeout == ABB_TRANXDONE_TIMEOUT)
		pr_warn("ABB: TRANXDONE timed out waiting for OPP change\n");

	timeout = 0;

	/* Clear all pending TRANXDONE interrupts/status */
	while (timeout < ABB_TRANXDONE_TIMEOUT) {
		prm_write_mod_reg((1 << vdd_info->omap_abb_reg_val.abb_done_st_shift),
				OCP_MOD,
				vdd_info->omap_abb_reg_val.prm_irqstatus_mpu);

		if (!(prm_read_mod_reg(OCP_MOD,
				vdd_info->omap_abb_reg_val.prm_irqstatus_mpu)
				& vdd_info->omap_abb_reg_val.abb_done_st_mask))
			break;

		udelay(1);
		timeout++;
	}

	if (timeout == ABB_TRANXDONE_TIMEOUT) {
		pr_warn("ABB: TRANXDONE timed out trying to clear status\n");
		ret = -EBUSY;
	}

	return ret;
}

static int omap4_abb_change_opp(struct omap_vdd_info *vdd_info)
{
	int ret = 0;
	int timeout;

	/* enable ABB ldo if not done already */
	if (prm_read_mod_reg(OMAP4430_PRM_DEVICE_MOD,
			vdd_info->omap_abb_reg_val.prm_abb_ldo_setup_idx) &
			OMAP4430_SR2_WTCNT_VALUE_MASK)
		omap_abb_enable(vdd_info);

	/* clear ABB ldo interrupt status */
	prm_clear_mod_reg_bits(vdd_info->omap_abb_reg_val.abb_done_st_mask,
			OMAP4430_PRM_OCP_SOCKET_MOD,
			vdd_info->omap_abb_reg_val.prm_irqstatus_mpu);

	/* enable ABB LDO OPP change */
	prm_set_mod_reg_bits(OMAP4430_OPP_CHANGE_MASK,
		OMAP4430_PRM_DEVICE_MOD,
		vdd_info->omap_abb_reg_val.prm_abb_ldo_ctrl_idx);

	timeout = 0;

	/* wait until OPP change completes */
	while ((timeout < ABB_TRANXDONE_TIMEOUT) &&
		(!(prm_read_mod_reg(OMAP4430_PRM_OCP_SOCKET_MOD,
			vdd_info->omap_abb_reg_val.prm_irqstatus_mpu) &
			vdd_info->omap_abb_reg_val.abb_done_st_mask))) {
		udelay(1);
		timeout++;
	}

	if (timeout == ABB_TRANXDONE_TIMEOUT)
		pr_warn("ABB: TRANXDONE timed out waiting for OPP change\n");

	timeout = 0;

	/* Clear all pending TRANXDONE interrupts/status */
	while (timeout < ABB_TRANXDONE_TIMEOUT) {
		prm_write_mod_reg((1 << vdd_info->omap_abb_reg_val.abb_done_st_shift),
				OMAP4430_PRM_OCP_SOCKET_MOD,
				vdd_info->omap_abb_reg_val.prm_irqstatus_mpu);

		if (!(prm_read_mod_reg(OMAP4430_PRM_OCP_SOCKET_MOD,
				vdd_info->omap_abb_reg_val.prm_irqstatus_mpu)
				& vdd_info->omap_abb_reg_val.abb_done_st_mask))
			break;

		udelay(1);
		timeout++;
	}

	if (timeout == ABB_TRANXDONE_TIMEOUT) {
		pr_warn("ABB: TRANXDONE timed out trying to clear status\n");
		ret = -EBUSY;
	}

	return ret;
}
#endif

static void __init vdd_data_configure(struct omap_vdd_info *vdd)
{
#ifdef CONFIG_PM_DEBUG
	struct dentry *vdd_debug;
	char name[16];
#endif

	if (cpu_is_omap34xx())
		omap3_vdd_data_configure(vdd);
	else if (cpu_is_omap44xx())
		omap4_vdd_data_configure(vdd);

	/* Init the plist */
	spin_lock_init(&vdd->user_lock);
	plist_head_init(&vdd->user_list, &vdd->user_lock);
	/* Init the DVFS mutex */
	mutex_init(&vdd->scaling_mutex);

	/* Get the devices associated with this VDD */
	vdd->dev_list = opp_init_voltage_params(&vdd->voltdm, &vdd->dev_count);

	/* Init the voltage change notifier list */
	srcu_init_notifier_head(&vdd->volt_change_notify_list);

#ifdef CONFIG_PM_DEBUG
	strcpy(name, "vdd_");
	strcat(name, vdd->voltdm.name);
	vdd_debug = debugfs_create_dir(name, voltage_dir);
	(void) debugfs_create_file("vp_errorgain", S_IRUGO | S_IWUGO,
				vdd_debug,
				&(vdd->vp_reg.vpconfig_errorgain),
				&vp_debug_fops);
	(void) debugfs_create_file("vp_smpswaittimemin", S_IRUGO | S_IWUGO,
				vdd_debug,
				&(vdd->vp_reg.vstepmin_smpswaittimemin),
				&vp_debug_fops);
	(void) debugfs_create_file("vp_stepmin", S_IRUGO | S_IWUGO, vdd_debug,
				&(vdd->vp_reg.vstepmin_stepmin),
				&vp_debug_fops);
	(void) debugfs_create_file("vp_smpswaittimemax", S_IRUGO | S_IWUGO,
				vdd_debug,
				&(vdd->vp_reg.vstepmax_smpswaittimemax),
				&vp_debug_fops);
	(void) debugfs_create_file("vp_stepmax", S_IRUGO | S_IWUGO, vdd_debug,
				&(vdd->vp_reg.vstepmax_stepmax),
				&vp_debug_fops);
	(void) debugfs_create_file("vp_vddmax", S_IRUGO | S_IWUGO, vdd_debug,
				&(vdd->vp_reg.vlimitto_vddmax),
				&vp_debug_fops);
	(void) debugfs_create_file("vp_vddmin", S_IRUGO | S_IWUGO, vdd_debug,
				&(vdd->vp_reg.vlimitto_vddmin),
				&vp_debug_fops);
	(void) debugfs_create_file("vp_timeout", S_IRUGO | S_IWUGO, vdd_debug,
				&(vdd->vp_reg.vlimitto_timeout),
				&vp_debug_fops);
	(void) debugfs_create_file("curr_vp_volt", S_IRUGO, vdd_debug,
				(void *) vdd, &vp_volt_debug_fops);
	(void) debugfs_create_file("curr_nominal_volt", S_IRUGO, vdd_debug,
				(void *) vdd, &nom_volt_debug_fops);
	(void) debugfs_create_file("volt_users", S_IRUGO, vdd_debug,
				(void *) vdd, &volt_users_dbg_fops);
	(void) debugfs_create_file("curr_dyn_nominal_volt", S_IRUGO, vdd_debug,
				(void *) vdd, &dyn_volt_debug_fops);
	(void) debugfs_create_file("curr_calibrated_volt", S_IRUGO, vdd_debug,
				(void *) vdd, &calib_volt_debug_fops);
#ifdef CONFIG_OMAP_ABB
	if (cpu_is_omap44xx() && !strcmp("vdd_iva", name))
		(void) debugfs_create_u8("fbb_enable", S_IRUGO | S_IWUGO,
				vdd_debug, &(vdd->volt_data[2].abb_type));

	if ((cpu_is_omap3630() || cpu_is_omap44xx())
			&& !strcmp("vdd_mpu", name))
		(void) debugfs_create_u8("fbb_enable", S_IRUGO | S_IWUGO,
				vdd_debug, &(vdd->volt_data[3].abb_type));
#endif
#endif
}
static void __init init_voltagecontroller(void)
{
	if (cpu_is_omap34xx())
		omap3_init_voltagecontroller();
	else if (cpu_is_omap44xx())
		omap4_init_voltagecontroller();
}

/*
 * vc_bypass_scale_voltage - VC bypass method of voltage scaling
 */
static int vc_bypass_scale_voltage(struct omap_vdd_info *vdd,
		struct omap_volt_data *target_volt)
{
	u32 vc_bypass_value = 0, vc_cmdval = 0;
	u32  vc_valid = 0, vc_bypass_val_reg_offs = 0;
	u32 vp_errgain_val = 0, vc_cmd_on_mask = 0;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 smps_steps = 0, smps_delay = 0;
	u8 vc_data_shift = 0, vc_slaveaddr_shift = 0, vc_regaddr_shift = 0;
	u8 vc_cmd_on_shift = 0;
	u8 target_vsel = 0, current_vsel = 0, sr_i2c_slave_addr = 0;

	if (IS_ERR_OR_NULL(target_volt)) {
		pr_warning("%s: bad target data\n", __func__);
		return -EINVAL;
	}

	if (cpu_is_omap34xx()) {
		vc_cmd_on_shift = OMAP3430_VC_CMD_ON_SHIFT;
		vc_cmd_on_mask = OMAP3430_VC_CMD_ON_MASK;
		vc_data_shift = OMAP3430_DATA_SHIFT;
		vc_slaveaddr_shift = OMAP3430_SLAVEADDR_SHIFT;
		vc_regaddr_shift = OMAP3430_REGADDR_SHIFT;
		vc_valid = OMAP3430_VALID_MASK;
		vc_bypass_val_reg_offs = OMAP3_PRM_VC_BYPASS_VAL_OFFSET;
		sr_i2c_slave_addr = vdd->pmic->i2c_addr;
	} else {
		pr_warning("%s: vc bypass method of voltage scaling"
			"not supported\n", __func__);
		return -EINVAL;
	}

	target_vsel = vdd->pmic->uv_to_vsel(
			omap_get_operation_voltage(target_volt));
	current_vsel = voltage_read_reg(vdd->vp_offs.voltage);
	smps_steps = abs(target_vsel - current_vsel);

	/* Setting the ON voltage to the new target voltage */
	vc_cmdval = voltage_read_reg(vdd->cmdval_reg);
	vc_cmdval &= ~vc_cmd_on_mask;
	vc_cmdval |= (target_vsel << vc_cmd_on_shift);
	voltage_write_reg(vdd->cmdval_reg, vc_cmdval);

	/*
	 * Setting vp errorgain based on the voltage. If the debug option is
	 * enabled allow the override of errorgain from user side
	 */
	if (!enable_sr_vp_debug && target_volt) {
		vp_errgain_val = voltage_read_reg(vdd->vp_offs.vpconfig);
		vdd->vp_reg.vpconfig_errorgain = target_volt->vp_errgain;
		vp_errgain_val &= ~vdd->vp_reg.vpconfig_errorgain_mask;
		vp_errgain_val |= vdd->vp_reg.vpconfig_errorgain <<
				vdd->vp_reg.vpconfig_errorgain_shift;
		voltage_write_reg(vdd->vp_offs.vpconfig, vp_errgain_val);
	}

	vc_bypass_value = (target_vsel << vc_data_shift) |
			(vdd->vdd_sr_reg << vc_regaddr_shift) |
			(sr_i2c_slave_addr << vc_slaveaddr_shift);

	voltage_write_reg(vc_bypass_val_reg_offs, vc_bypass_value);

	voltage_write_reg(vc_bypass_val_reg_offs, vc_bypass_value | vc_valid);
	vc_bypass_value = voltage_read_reg(vc_bypass_val_reg_offs);

	while ((vc_bypass_value & vc_valid) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			pr_warning("%s: Loop count exceeded in check SR I2C"
				"write during voltgae scaling\n", __func__);
			return -ETIMEDOUT;
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = voltage_read_reg(vc_bypass_val_reg_offs);
	}

	/* SMPS slew rate / step size. 2us added as buffer. */
	smps_delay = ((smps_steps * vdd->pmic->step_size) /
			vdd->pmic->slew_rate) + 2;
	udelay(smps_delay);

	vdd->curr_volt = target_volt;

	return 0;
}

/* VP force update method of voltage scaling */
static int vp_forceupdate_scale_voltage(struct omap_vdd_info *vdd,
		struct omap_volt_data *target_volt)
{
	u32 vc_cmd_on_mask, vc_cmdval, vpconfig;
	u32 smps_steps = 0, smps_delay = 0;
	int timeout = 0;
	u8 target_vsel = 0, current_vsel = 0;
	u8 vc_cmd_on_shift = 0;

	if (IS_ERR_OR_NULL(target_volt)) {
		pr_warning("%s: bad target data\n", __func__);
		return -EINVAL;
	}
	if (cpu_is_omap34xx()) {
		vc_cmd_on_shift = OMAP3430_VC_CMD_ON_SHIFT;
		vc_cmd_on_mask = OMAP3430_VC_CMD_ON_MASK;
	} else if (cpu_is_omap44xx()) {
		vc_cmd_on_shift = OMAP4430_ON_SHIFT;
		vc_cmd_on_mask = OMAP4430_ON_MASK;
	}

	target_vsel = vdd->pmic->uv_to_vsel(
			omap_get_operation_voltage(target_volt));
	current_vsel = voltage_read_reg(vdd->vp_offs.voltage);
	smps_steps = abs(target_vsel - current_vsel);

	/* Setting the ON voltage to the new target voltage */
	vc_cmdval = voltage_read_reg(vdd->cmdval_reg);
	vc_cmdval &= ~vc_cmd_on_mask;
	vc_cmdval |= (target_vsel << vc_cmd_on_shift);
	voltage_write_reg(vdd->cmdval_reg, vc_cmdval);

	/*
	 * Getting  vp errorgain based on the voltage. If the debug option is
	 * enabled allow the override of errorgain from user side.
	 */
	if (!enable_sr_vp_debug && target_volt)
		vdd->vp_reg.vpconfig_errorgain =
					target_volt->vp_errgain;
	/*
	 * Clear all pending TransactionDone interrupt/status. Typical latency
	 * is <3us
	 */
	while (timeout++ < VP_TRANXDONE_TIMEOUT) {
		omap_vp_clear_transdone(&vdd->voltdm);
		if (!omap_vp_is_transdone(&vdd->voltdm))
				break;
		udelay(1);
	}

	if (timeout >= VP_TRANXDONE_TIMEOUT) {
		pr_warning("%s: vdd_%s TRANXDONE timeout exceeded."
			"Voltage change aborted", __func__, vdd->voltdm.name);
		return -ETIMEDOUT;
	}
	/* Configure for VP-Force Update */
	vpconfig = voltage_read_reg(vdd->vp_offs.vpconfig);
	vpconfig &= ~(vdd->vp_reg.vpconfig_initvdd |
			vdd->vp_reg.vpconfig_forceupdate |
			vdd->vp_reg.vpconfig_initvoltage_mask |
			vdd->vp_reg.vpconfig_errorgain_mask);
	vpconfig |= ((target_vsel <<
			vdd->vp_reg.vpconfig_initvoltage_shift) |
			(vdd->vp_reg.vpconfig_errorgain <<
			 vdd->vp_reg.vpconfig_errorgain_shift));
	voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);

	/* Trigger initVDD value copy to voltage processor */
	vpconfig |= vdd->vp_reg.vpconfig_initvdd;
	voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);

	/* Force update of voltage */
	vpconfig |= vdd->vp_reg.vpconfig_forceupdate;
	voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);

	timeout = 0;
	/*
	 * Wait for TransactionDone. Typical latency is <200us.
	 * Depends on SMPSWAITTIMEMIN/MAX and voltage change
	 */
	omap_test_timeout((prm_read_mod_reg(vdd->ocp_mod, vdd->prm_irqst_reg) &
			vdd->vp_reg.tranxdone_status),
			VP_TRANXDONE_TIMEOUT, timeout);

	if (timeout >= VP_TRANXDONE_TIMEOUT)
		pr_err("%s: vdd_%s TRANXDONE timeout exceeded."
			"TRANXDONE never got set after the voltage update\n",
			__func__, vdd->voltdm.name);
	/*
	 * Wait for voltage to settle with SW wait-loop.
	 * SMPS slew rate / step size. 2us added as buffer.
	 */
	smps_delay = ((smps_steps * vdd->pmic->step_size) /
			vdd->pmic->slew_rate) + 2;
	udelay(smps_delay);

	/*
	 * Disable TransactionDone interrupt , clear all status, clear
	 * control registers
	 */
	timeout = 0;
	while (timeout++ < VP_TRANXDONE_TIMEOUT) {
		omap_vp_clear_transdone(&vdd->voltdm);
		if (!omap_vp_is_transdone(&vdd->voltdm))
				break;
		udelay(1);
	}
	if (timeout >= VP_TRANXDONE_TIMEOUT)
		pr_warning("%s: vdd_%s TRANXDONE timeout exceeded while trying"
			"to clear the TRANXDONE status\n",
			__func__, vdd->voltdm.name);

	vpconfig = voltage_read_reg(vdd->vp_offs.vpconfig);
	/* Clear initVDD copy trigger bit */
	vpconfig &= ~vdd->vp_reg.vpconfig_initvdd;;
	voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);
	/* Clear force bit */
	vpconfig &= ~vdd->vp_reg.vpconfig_forceupdate;
	voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);

	vdd->curr_volt = target_volt;
	return 0;
}

static int calc_dep_vdd_volt(struct device *dev,
		struct omap_vdd_info *main_vdd, unsigned long main_volt)
{
	struct omap_vdd_dep_info *dep_vdds;
	int i, ret = 0;

	if (!main_vdd->dep_vdd_info) {
		pr_debug("%s: No dependent VDD's for vdd_%s\n",
			__func__, main_vdd->voltdm.name);
		return 0;
	}

	dep_vdds = main_vdd->dep_vdd_info;

	for (i = 0; i < main_vdd->nr_dep_vdd; i++) {
		struct omap_vdd_dep_volt *volt_table = dep_vdds[i].dep_table;
		int nr_volt = 0;
		unsigned long dep_volt = 0, act_volt = 0;

		while (volt_table[nr_volt].main_vdd_volt != 0) {
			if (volt_table[nr_volt].main_vdd_volt == main_volt) {
				dep_volt = volt_table[nr_volt].dep_vdd_volt;
				break;
			}
			nr_volt++;
		}
		if (!dep_volt) {
			pr_warning("%s: Not able to find a matching volt for"
				"vdd_%s corresponding to vdd_%s %ld volt\n",
				__func__, dep_vdds[i].name,
				main_vdd->voltdm.name, main_volt);
			ret = -EINVAL;
			continue;
		}

		if (!dep_vdds[i].voltdm)
			dep_vdds[i].voltdm =
				omap_voltage_domain_get(dep_vdds[i].name);

		act_volt = dep_volt;

		/* See if dep_volt is possible for the vdd*/
		ret = omap_voltage_add_userreq(dep_vdds[i].voltdm, dev,
				&act_volt);

	}

	return ret;
}

static int scale_dep_vdd(struct omap_vdd_info *main_vdd)
{
	struct omap_vdd_dep_info *dep_vdds;
	int i;

	if (!main_vdd->dep_vdd_info) {
		pr_debug("%s: No dependent VDD's for vdd_%s\n",
			__func__, main_vdd->voltdm.name);
		return 0;
	}

	dep_vdds = main_vdd->dep_vdd_info;

	for (i = 0; i < main_vdd->nr_dep_vdd; i++)
		omap_voltage_scale(dep_vdds[i].voltdm);
	return 0;
}

/* Public functions */

/**
 * omap_vscale_pause() - Pause the dvfs transition for this domain
 * @voltdom: voltage domain to pause
 * @trylock: should we return if we dvfs already in transition
 *
 * To ensure that the system accesses to internal registers of OMAP
 * modules are made safe, we need to ensure exclusivity of access
 * these module drivers request pause of the domain transition while
 * they finish off their work. The users should ensure sanity
 * between usual dvfs paths Vs usage of these APIs to prevent deadlocks
 *
 * Returns 0 if dvfs has been paused for the domain, else returns err val
 */
int omap_vscale_pause(struct voltagedomain *voltdm, bool trylock)
{
	struct omap_vdd_info *vdd;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return 0;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	if (trylock)
		return !mutex_trylock(&vdd->scaling_mutex);

	mutex_lock(&vdd->scaling_mutex);
	return 0;
}

/**
 * omap_vscale_unpause() - Free up the dvfs transitions for this domain
 * @voltdom: voltage domain to unpause
 *
 * In the cases where omap_vscale_pause operations are called, unpause
 * is used to free up the sequences
 */
int omap_vscale_unpause(struct voltagedomain *voltdm)
{
	struct omap_vdd_info *vdd;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return 0;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	mutex_unlock(&vdd->scaling_mutex);
	return 0;
}

/**
 * omap_voltage_get_nom_volt : Gets the current non-auto-compensated voltage
 * @voltdm	: pointer to the VDD for which current voltage info is needed
 *
 * API to get the current voltage data pointer for a VDD.
 */
struct omap_volt_data *omap_voltage_get_nom_volt(struct voltagedomain *voltdm)
{
	struct omap_vdd_info *vdd;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return 0;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	return vdd->curr_volt;
}

/**
 * omap_voltage_calib_reset() - reset the calibrated voltage entries
 * @voltdm: voltage domain to reset the entries for
 *
 * when the calibrated entries are no longer valid, this api allows
 * the calibrated voltages to be reset.
 */
int omap_voltage_calib_reset(struct voltagedomain *voltdm)
{
	struct omap_vdd_info *vdd;
	struct omap_volt_data *volt_data;
	int i;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);
	volt_data = vdd->volt_data;
	/* reset the calibrated voltages as 0 */
	for (i = 0; i < vdd->volt_data_count; i++) {
		volt_data->volt_calibrated = 0;
		volt_data++;
	}
	return 0;
}

/**
 * omap_vp_get_curr_volt : API to get the current vp voltage.
 * @voltdm: pointer to the VDD.
 *
 * This API returns the current voltage for the specified voltage processor
 */
unsigned long omap_vp_get_curr_volt(struct voltagedomain *voltdm)
{
	struct omap_vdd_info *vdd;
	u8 curr_vsel;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return 0;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	curr_vsel = voltage_read_reg(vdd->vp_offs.voltage);
	return vdd->pmic->vsel_to_uv(curr_vsel);
}

/**
 * omap_voltage_add_userreq : API to keep track of various requests to
 *			    scale the VDD and returns the best possible
 *			    voltage the VDD can be put to.
 * @volt_domain: pointer to the voltage domain.
 * @dev : the device pointer.
 * @volt : the voltage which is requested by the device.
 *
 * This API is to be called before the actual voltage scaling is
 * done to determine what is the best possible voltage the VDD can
 * be put to. This API adds the device <dev> in the user list of the
 * vdd <volt_domain> with <volt> as the requested voltage. The user list
 * is a plist with the priority element absolute voltage values.
 * The API then finds the maximum of all the requested voltages for
 * the VDD and returns it back through <volt> pointer itself.
 * Returns error value in case of any errors.
 */
int omap_voltage_add_userreq(struct voltagedomain *voltdm, struct device *dev,
		unsigned long *volt)
{
	struct omap_vdd_info *vdd;
	struct omap_vdd_user_list *user;
	struct plist_node *node;
	int found = 0;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	mutex_lock(&vdd->scaling_mutex);

	plist_for_each_entry(user, &vdd->user_list, node) {
		if (user->dev == dev) {
			found = 1;
			break;
		}
	}

	if (!found) {
		user = kzalloc(sizeof(struct omap_vdd_user_list), GFP_KERNEL);
		if (!user) {
			pr_err("%s: Unable to creat a new user for vdd_%s\n",
				__func__, voltdm->name);
			mutex_unlock(&vdd->scaling_mutex);
			return -ENOMEM;
		}
		user->dev = dev;
	} else {
		plist_del(&user->node, &vdd->user_list);
	}

	plist_node_init(&user->node, *volt);
	plist_add(&user->node, &vdd->user_list);
	node = plist_last(&vdd->user_list);
	*volt = node->prio;

	mutex_unlock(&vdd->scaling_mutex);

	return 0;
}

/**
 * omap_vp_enable : API to enable a particular VP
 * @voltdm: pointer to the VDD whose VP is to be enabled.
 *
 * This API enables a particular voltage processor. Needed by the smartreflex
 * class drivers.
 */
void omap_vp_enable(struct voltagedomain *voltdm)
{
	struct omap_vdd_info *vdd;
	u32 vpconfig;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	/* If VP is already enabled, do nothing. Return */
	if (voltage_read_reg(vdd->vp_offs.vpconfig) &
				vdd->vp_reg.vpconfig_vpenable)
		return;
	/*
	 * This latching is required in VC bypass method as well VP force
	 * update method to ensure that VP registers are programmed always to
	 * current Nominal vsel before enabling VP ensuring no mismatch
	 * happens b/w VP and VC regs.
	 */
	vp_latch_vsel(vdd);

	/*
	 * If debug is enabled, it is likely that the following parameters
	 * were set from user space so rewrite them.
	 */
	if (enable_sr_vp_debug) {
		vpconfig = voltage_read_reg(vdd->vp_offs.vpconfig);
		vpconfig |= (vdd->vp_reg.vpconfig_errorgain <<
			vdd->vp_reg.vpconfig_errorgain_shift);
		voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);

		voltage_write_reg(vdd->vp_offs.vstepmin,
			(vdd->vp_reg.vstepmin_smpswaittimemin <<
			vdd->vp_reg.vstepmin_smpswaittimemin_shift) |
			(vdd->vp_reg.vstepmin_stepmin <<
			vdd->vp_reg.vstepmin_stepmin_shift));

		voltage_write_reg(vdd->vp_offs.vstepmax,
			(vdd->vp_reg.vstepmax_smpswaittimemax <<
			vdd->vp_reg.vstepmax_smpswaittimemax_shift) |
			(vdd->vp_reg.vstepmax_stepmax <<
			vdd->vp_reg.vstepmax_stepmax_shift));

		voltage_write_reg(vdd->vp_offs.vlimitto,
			(vdd->vp_reg.vlimitto_vddmax <<
			vdd->vp_reg.vlimitto_vddmax_shift) |
			(vdd->vp_reg.vlimitto_vddmin <<
			vdd->vp_reg.vlimitto_vddmin_shift) |
			(vdd->vp_reg.vlimitto_timeout <<
			vdd->vp_reg.vlimitto_timeout_shift));
	}

	vpconfig = voltage_read_reg(vdd->vp_offs.vpconfig);
	/* Enable VP */
	voltage_write_reg(vdd->vp_offs.vpconfig,
				vpconfig | vdd->vp_reg.vpconfig_vpenable);
}

/**
 * omap_vp_disable : API to disable a particular VP
 * @voltdm: pointer to the VDD whose VP is to be disabled.
 *
 * This API disables a particular voltage processor. Needed by the smartreflex
 * class drivers.
 */
void omap_vp_disable(struct voltagedomain *voltdm)
{
	struct omap_vdd_info *vdd;
	u32 vpconfig;
	int timeout;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	/* If VP is already disabled, do nothing. Return */
	if (!(voltage_read_reg(vdd->vp_offs.vpconfig) &
				vdd->vp_reg.vpconfig_vpenable)) {
		pr_warning("%s: Trying to disable VP for vdd_%s when"
			"it is already disabled\n", __func__, voltdm->name);
		return;
	}

	/* Disable VP */
	vpconfig = voltage_read_reg(vdd->vp_offs.vpconfig);
	vpconfig &= ~vdd->vp_reg.vpconfig_vpenable;
	voltage_write_reg(vdd->vp_offs.vpconfig, vpconfig);

	/*
	 * Wait for VP idle Typical latency is <2us. Maximum latency is ~100us
	 */
	omap_test_timeout((voltage_read_reg(vdd->vp_offs.vstatus)),
				VP_IDLE_TIMEOUT, timeout);

	if (timeout >= VP_IDLE_TIMEOUT)
		pr_warning("%s: vdd_%s idle timedout\n",
			__func__, voltdm->name);
	return;
}

/**
 * omap_vp_is_transdone() - is voltage transfer done on vp?
 * @voltdm:	pointer to the VDD which is to be scaled.
 *
 * VP's transdone bit is the only way to ensure that the transfer
 * of the voltage value has actually been send over to the PMIC
 * This is hence useful for all users of voltage domain to precisely
 * identify once the PMIC voltage has been set by the voltage processor
 */
bool omap_vp_is_transdone(struct voltagedomain *voltdm)
{
	struct omap_vdd_info *vdd;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_warning("%s: Bad Params vdm=%p\n", __func__, voltdm);
		return false;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);
	return (prm_read_mod_reg(vdd->ocp_mod, vdd->prm_irqst_reg) &
				vdd->vp_reg.tranxdone_status) ?  true : false;
}

/**
 * omap_vp_clear_transdone() - clear voltage transfer done status on vp
 * @voltdm:	pointer to the VDD which is to be scaled.
 */
bool omap_vp_clear_transdone(struct voltagedomain *voltdm)
{
	struct omap_vdd_info *vdd;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_warning("%s: Bad Params vdm=%p\n", __func__, voltdm);
		return false;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);
	prm_write_mod_reg(vdd->vp_reg.tranxdone_status,
			vdd->ocp_mod, vdd->prm_irqst_reg);
	return true;
}

/**
 * omap_voltage_scale_vdd : API to scale voltage of a particular voltage domain.
 * @voltdm: pointer to the VDD which is to be scaled.
 * @target_volt : The target voltage of the voltage domain
 *
 * This API should be called by the kernel to do the voltage scaling
 * for a particular voltage domain during dvfs or any other situation.
 */
int omap_voltage_scale_vdd(struct voltagedomain *voltdm,
		struct omap_volt_data *target_volt)
{
	struct omap_vdd_info *vdd;
	struct omap_volt_change_info v_info;
	int ret;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	v_info.vdd_info = vdd;
	v_info.curr_volt = vdd->curr_volt;
	v_info.target_volt = target_volt;

	srcu_notifier_call_chain(&vdd->volt_change_notify_list,
		VOLTAGE_PRECHANGE, (void *)&v_info);

	if (voltscale_vpforceupdate)
		ret = vp_forceupdate_scale_voltage(vdd, target_volt);
	else
		ret =  vc_bypass_scale_voltage(vdd, target_volt);

	if (!ret)
		srcu_notifier_call_chain(&vdd->volt_change_notify_list,
			VOLTAGE_POSTCHANGE, (void *)&v_info);

	return ret;
}

/**
 * omap_voltage_reset : Resets the voltage of a particular voltage domain
 * to that of the current OPP.
 * @voltdm: pointer to the VDD whose voltage is to be reset.
 *
 * This API finds out the correct voltage the voltage domain is supposed
 * to be at and resets the voltage to that level. Should be used expecially
 * while disabling any voltage compensation modules.
 */
void omap_voltage_reset(struct voltagedomain *voltdm)
{
	struct omap_volt_data *target_uvdc;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return;
	}

	target_uvdc = omap_voltage_get_nom_volt(voltdm);
	if (IS_ERR_OR_NULL(target_uvdc)) {
		pr_err("%s: unable to find current voltage for vdd_%s\n",
			__func__, voltdm->name);
		return;
	}
	omap_voltage_scale_vdd(voltdm, target_uvdc);
}

/**
 * omap_change_voltscale_method : API to change the voltage scaling method.
 * @voltscale_method : the method to be used for voltage scaling.
 *
 * This API can be used by the board files to change the method of voltage
 * scaling between vpforceupdate and vcbypass. The parameter values are
 * defined in voltage.h
 */
void omap_change_voltscale_method(int voltscale_method)
{
	switch (voltscale_method) {
	case VOLTSCALE_VPFORCEUPDATE:
		voltscale_vpforceupdate = true;
		return;
	case VOLTSCALE_VCBYPASS:
		voltscale_vpforceupdate = false;
		return;
	default:
		pr_warning("%s: Trying to change the method of voltage scaling"
			"to an unsupported one!\n", __func__);
	}
}

/**
 * omap_voltage_init_vc - polpulates vc_config with values specified in
 *			  board file
 * @setup_vc - the structure with various vc parameters
 *
 * Updates vc_config with the voltage setup times and other parameters as
 * specified in setup_vc. vc_config is later used in init_voltagecontroller
 * to initialize the voltage controller registers. Board files should call
 * this function with the correct volatge settings corresponding
 * the particular PMIC and chip.
 */
void __init omap_voltage_init_vc(struct omap_volt_vc_data *setup_vc)
{
	if (!setup_vc)
		return;

	vc_config.clksetup = setup_vc->clksetup;
	vc_config.voltsetup_time1 = setup_vc->voltsetup_time1;
	vc_config.voltsetup_time2 = setup_vc->voltsetup_time2;
	vc_config.voltoffset = setup_vc->voltoffset;
	vc_config.voltsetup2 = setup_vc->voltsetup2;
	vc_config.vdd0_on = setup_vc->vdd0_on;
	vc_config.vdd0_onlp = setup_vc->vdd0_onlp;
	vc_config.vdd0_ret = setup_vc->vdd0_ret;
	vc_config.vdd0_off = setup_vc->vdd0_off;
	vc_config.vdd1_on = setup_vc->vdd1_on;
	vc_config.vdd1_onlp = setup_vc->vdd1_onlp;
	vc_config.vdd1_ret = setup_vc->vdd1_ret;
	vc_config.vdd1_off = setup_vc->vdd1_off;
	vc_config.vdd2_on = setup_vc->vdd2_on;
	vc_config.vdd2_onlp = setup_vc->vdd2_onlp;
	vc_config.vdd2_ret = setup_vc->vdd2_ret;
	vc_config.vdd2_off = setup_vc->vdd2_off;
}

/**
 * omap_voltage_get_volttable : API to get the voltage table associated with a
 *			    particular voltage domain.
 *
 * @voltdm: pointer to the VDD for which the voltage table is required
 * @volt_data : the voltage table for the particular vdd which is to be
 *		populated by this API
 * This API populates the voltage table associated with a VDD into the
 * passed parameter pointer. Returns the count of distinct voltages
 * supported by this vdd.
 *
 */
int omap_voltage_get_volttable(struct voltagedomain *voltdm,
		struct omap_volt_data **volt_data)
{
	struct omap_vdd_info *vdd;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return 0;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	*volt_data = vdd->volt_data;
	return vdd->volt_data_count;
}

/**
 * omap_voltage_get_voltdata : API to get the voltage table entry for a
 *				particular voltage
 * @voltdm: pointer to the VDD whose voltage table has to be searched
 * @volt : the voltage to be searched in the voltage table
 *
 * This API searches through the voltage table for the required voltage
 * domain and tries to find a matching entry for the passed voltage volt.
 * If a matching entry is found volt_data is populated with that entry.
 * This API searches only through the non-compensated voltages int the
 * voltage table.
 * Returns pointer to the voltage table entry corresponding to volt on
 * sucess. Returns -ENODATA if no voltage table exisits for the passed voltage
 * domain or if there is no matching entry.
 */
struct omap_volt_data *omap_voltage_get_voltdata(struct voltagedomain *voltdm,
		unsigned long volt)
{
	struct omap_vdd_info *vdd;
	int i;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	if (!vdd->volt_data) {
		pr_warning("%s: voltage table does not exist for vdd_%s\n",
			__func__, voltdm->name);
		return ERR_PTR(-ENODATA);
	}

	for (i = 0; i < vdd->volt_data_count; i++) {
		if (vdd->volt_data[i].volt_nominal == volt)
			return &vdd->volt_data[i];
	}

	pr_notice("%s: Unable to match the current voltage %ld with the voltage"
		"table for vdd_%s\n", __func__, volt, voltdm->name);

	return ERR_PTR(-ENODATA);
}

/**
 * omap_voltage_register_pmic : API to register PMIC specific data
 * @pmic_info : the structure containing pmic info
 *
 * This API is to be called by the borad file to specify the pmic specific
 * info as present in omap_volt_pmic_info structure. A default pmic info
 * table is maintained in the driver volt_pmic_info. If the board file do
 * not override the default table using this API, the default values wiil
 * be used in the driver.
 */
void omap_voltage_register_pmic(struct omap_volt_pmic_info *pmic_info,
				char *vdm_name)
{
	int i;
	struct omap_vdd_info *vdds = NULL;
	int nvdds = 0;

	if (cpu_is_omap34xx()) {
		vdds = omap3_vdd_info;
		nvdds = OMAP3_NO_SCALABLE_VDD;
	} else if (cpu_is_omap44xx()) {
		vdds = omap4_vdd_info;
		nvdds = OMAP4_NO_SCALABLE_VDD;
	}
	for (i = 0; i < nvdds; i++) {
		if (!strcmp(vdds[i].voltdm.name, vdm_name)) {
			vdds[i].pmic = pmic_info;
			break;
		}
	}
}

/**
 * omap_voltage_domain_get	: API to get the voltage domain pointer
 * @name : Name of the voltage domain
 *
 * This API looks up in the global vdd_info struct for the
 * existence of voltage domain <name>. If it exists, the API returns
 * a pointer to the voltage domain structure corresponding to the
 * VDD<name>. Else retuns error pointer.
 */
struct voltagedomain *omap_voltage_domain_get(char *name)
{
	int i;

	if (!vdd_info) {
		pr_err("%s: Voltage driver init not yet happened.Faulting!\n",
			__func__);
		return ERR_PTR(-EINVAL);
	}

	if (!name) {
		pr_err("%s: No name to get the votage domain!\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	for (i = 0; i < no_scalable_vdd; i++) {
		if (!(strcmp(name, vdd_info[i].voltdm.name)))
			return &vdd_info[i].voltdm;
	}

	return ERR_PTR(-EINVAL);
}

/**
 * omap_voltage_scale : API to scale the devices associated with a
 *			voltage domain vdd voltage.
 * @volt_domain : the voltage domain to be scaled
 *
 * This API runs through the list of devices associated with the
 * voltage domain and scales the device rates to those corresponding
 * to the new voltage of the voltage domain. This API also scales
 * the voltage domain voltage to the new value. Returns 0 on success
 * else the error value.
 */
int omap_voltage_scale(struct voltagedomain *voltdm)
{
	unsigned long curr_volt;
	int is_volt_scaled = 0, i;
	bool is_sr_disabled = false;
	struct omap_vdd_info *vdd;
	struct plist_node *node;
	unsigned long volt;
	unsigned long curr_nom_volt;
	struct omap_volt_data *vnom;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);

	mutex_lock(&vdd->scaling_mutex);

	curr_volt = omap_get_operation_voltage(
			omap_voltage_get_nom_volt(voltdm));

	vnom = omap_voltage_get_nom_volt(voltdm);

	curr_volt = vnom->volt_nominal;

	/* Find the highest voltage for this vdd */
	node = plist_last(&vdd->user_list);
	volt = node->prio;
#if 0
		if (curr_volt != volt)
			printk("[DEBUG] curr_volt =%d ,volt = %d  \n ", curr_volt,volt);
#endif 

	

	/* Disable smartreflex module across voltage and frequency scaling */
	if (curr_volt != volt) {
		omap_smartreflex_disable_reset_volt(voltdm);
		is_sr_disabled = true;
	}

#ifdef SMARTREFLEX_DEBUG
   	printk("[DEBUG] curr_volt =%d ,volt = %d  \n ", curr_volt,volt);
#endif 
	if (curr_volt == volt) {
		is_volt_scaled = 1;
	} else if (curr_volt < volt) {
#ifdef SMARTREFLEX_DEBUG
	printk("[DEBUG] L to H OPP Volt scal\n");
#endif
		omap_voltage_scale_vdd(voltdm,
				omap_voltage_get_voltdata(voltdm, volt));
		is_volt_scaled = 1;
	}

	for (i = 0; i < vdd->dev_count; i++) {
		struct omap_opp *opp;
		unsigned long freq;

		opp = opp_find_voltage(vdd->dev_list[i], volt, true);
		if (IS_ERR(opp)) {
			dev_err(vdd->dev_list[i], "%s: Unable to find OPP for"
				"volt%ld\n", __func__, volt);
			continue;
		}

		freq = opp_get_freq(opp);

		if (freq == opp_get_rate(vdd->dev_list[i]))
			continue;

		opp_set_rate(vdd->dev_list[i], freq);
	}

	if (!is_volt_scaled)
		{
#ifdef SMARTREFLEX_DEBUG
	printk("[DEBUG] H to L OPP Volt scal\n");
#endif 
		omap_voltage_scale_vdd(voltdm,
				omap_voltage_get_voltdata(voltdm, volt));
		}

	/* Enable Smartreflex module */
	if (is_sr_disabled)
		omap_smartreflex_enable(voltdm);

	mutex_unlock(&vdd->scaling_mutex);

	/* calculate the voltages for dependent vdd's */
	if (calc_dep_vdd_volt(&vdd->vdd_device, vdd, volt)) {
		pr_warning("%s: Error in calculating dependent vdd voltages"
			"for vdd_%s\n", __func__, voltdm->name);
		return -EINVAL;
	}

	/* Scale dependent vdds */
	scale_dep_vdd(vdd);

	return 0;
}

int omap_voltage_register_notifier(struct voltagedomain *voltdm,
		struct notifier_block *nb)
{
	struct omap_vdd_info *vdd;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);
	return srcu_notifier_chain_register(&vdd->volt_change_notify_list, nb);
}

int omap_voltage_unregister_notifier(struct voltagedomain *voltdm,
		struct notifier_block *nb)
{
	struct omap_vdd_info *vdd;

	if (!voltdm || IS_ERR(voltdm)) {
		pr_warning("%s: VDD specified does not exist!\n", __func__);
		return -EINVAL;
	}

	vdd = container_of(voltdm, struct omap_vdd_info, voltdm);
	return srcu_notifier_chain_unregister(
				&vdd->volt_change_notify_list, nb);
}

/**
 * omap_voltage_init : Volatage init API which does VP and VC init.
 */
static int __init omap_voltage_init(void)
{
	int i;
	u32 is_trimmed = 0;

	if (!(cpu_is_omap34xx() || cpu_is_omap44xx())) {
		pr_warning("%s: voltage driver support not added\n", __func__);
		return 0;
	}

	/*
	 * Some ES2.2 efuse  values for BGAP and SLDO trim
	 * are not programmed. For these units
	 * 1. we can set overide mode for SLDO trim,
	 * and program the max multiplication factor, to ensure
	 * high enough voltage on SLDO output.
	 * 2. trim VDAC value for TV output as per recomendation
	 */
	if (cpu_is_omap44xx()
		&& (omap_rev() == CHIP_IS_OMAP4430ES2_2)) {
		is_trimmed = omap_ctrl_readl(
			OMAP4_CTRL_MODULE_CORE_LDOSRAM_MPU_VOLTAGE_CTRL);
		if (!is_trimmed) {
			/* Trimm value is 0 for this unit.
			 * we set force overide, insted of efuse.
			 */
			omap_ctrl_writel(0x0400040f,
			OMAP4_CTRL_MODULE_CORE_LDOSRAM_MPU_VOLTAGE_CTRL);
			omap_ctrl_writel(0x0400040f,
			OMAP4_CTRL_MODULE_CORE_LDOSRAM_CORE_VOLTAGE_CTRL);
			omap_ctrl_writel(0x0400040f,
			OMAP4_CTRL_MODULE_CORE_LDOSRAM_IVA_VOLTAGE_CTRL);
			/* write value of 0x0 to VDAC as per trim recomendation */
			omap_ctrl_writel(0x000001c0,
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_EFUSE_1);
		} else {
			/*
			 * Set SRAM MPU/CORE/IVA LDO RETMODE
			 * Setting RETMODE for un-trimmed units cause random
			 * system hang. So enabling it only for trimmed units.
			 */
			prm_rmw_mod_reg_bits(OMAP4430_RETMODE_ENABLE_MASK,
				0x1 << OMAP4430_RETMODE_ENABLE_SHIFT,
				OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_LDO_SRAM_CORE_CTRL_OFFSET);
				prm_rmw_mod_reg_bits(OMAP4430_RETMODE_ENABLE_MASK,
				0x1 << OMAP4430_RETMODE_ENABLE_SHIFT,
			OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_LDO_SRAM_MPU_CTRL_OFFSET);
				prm_rmw_mod_reg_bits(OMAP4430_RETMODE_ENABLE_MASK,
				0x1 << OMAP4430_RETMODE_ENABLE_SHIFT,
			OMAP4430_PRM_DEVICE_MOD, OMAP4_PRM_LDO_SRAM_IVA_CTRL_OFFSET);
		}
	}

	/*
	 * for all ESx.y trimmed and untrimmed units LPDDR IO and
	 * Smart IO override efuse with P:16/N:16 and P:0/N:0 respectively
	 */
	if (cpu_is_omap44xx())
		omap_ctrl_writel(0x00084000,
			OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_EFUSE_2);


#ifdef CONFIG_PM_DEBUG
	voltage_dir = debugfs_create_dir("voltage", pm_dbg_main_dir);
#endif
	if (cpu_is_omap34xx()) {
		volt_mod = OMAP3430_GR_MOD;
		vdd_info = omap3_vdd_info;
		no_scalable_vdd = OMAP3_NO_SCALABLE_VDD;
	} else if (cpu_is_omap44xx()) {
		volt_mod = OMAP4430_PRM_DEVICE_MOD;
		vdd_info = omap4_vdd_info;
		no_scalable_vdd = OMAP4_NO_SCALABLE_VDD;
	}
	init_voltagecontroller();
	for (i = 0; i < no_scalable_vdd; i++) {
		vdd_data_configure(&vdd_info[i]);
		init_voltageprocessor(&vdd_info[i]);
#ifdef CONFIG_OMAP_ABB
		omap_abb_init(&vdd_info[i]);
#endif
	}

	return 0;
}
device_initcall(omap_voltage_init);
