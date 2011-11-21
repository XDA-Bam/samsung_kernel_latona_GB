/*
 * OMAP Power Management debug routines
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * Written by:
 * Richard Woodruff <r-woodruff2@ti.com>
 * Tony Lindgren
 * Juha Yrjola
 * Amit Kucheria <amit.kucheria@nokia.com>
 * Igor Stoppa <igor.stoppa@nokia.com>
 * Jouni Hogander
 *
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <plat/clock.h>
#include <plat/board.h>
#include <plat/powerdomain.h>
#include <plat/clockdomain.h>
#include <plat/voltage.h>
#include <plat/dmtimer.h>

#include <mach/omap4-common.h>

#include "prm-regbits-34xx.h"

#include "prm.h"
#include "cm.h"
#include "pm.h"
#include "prm-regbits-34xx.h"
//#define PAD_DEBUGGING
//#define PMIC_DEBUGGING
#ifdef CONFIG_SAMSUNG_KERNEL_DEBUG
#endif /* CONFIG_SAMSUNG_KERNEL_DEBUG */

#ifdef PAD_DEBUGGING
#include "pad_define.h"
#include <linux/i2c/twl.h>
#endif

#ifdef PMIC_DEBUGGING
u16 pmic_reg_dump;
static int pad_pmic_reg_get(void *data, u64 *val)
{
	u32 *option = data;
	int i;
	u16 value;
	*val = *option;
	#ifdef T2_REG_DUMP
		twl_i2c_read_regdump();
	#else
		printk("[pm-debug] T2_REG_DUMP is not defined. (twl.h)\n");
	#endif
	return 0;
}

struct omap_pmic_dbg {
	const char name[20];
	u8	addr_grp;
	u8	addr_type;
	u8	addr_remap;
	u8	addr_dedicated;
};

u16 pmic_dbg[12];
static int pmic_ldo_num = 16;
static struct omap_pmic_dbg board_pmic_dbg[] = {
	{"vaux1", 0x17, 0x18, 0x19, 0x1A},
	{"vaux2", 0x1B, 0x1C, 0x1D, 0x1E},
	{"vaux3", 0x1F, 0x20, 0x21, 0x22},
	{"vaux4", 0x23, 0x24, 0x25, 0x26},
	{"vmmc1", 0x27, 0x28, 0x29, 0x2A},
	{"vmmc2", 0x2B, 0x2C, 0x2D, 0x2E},
	{"vpll1", 0x2F, 0x30, 0x31, 0x32},
	{"vpll2", 0x33, 0x34, 0x35, 0x36},
	{"vsim", 0x37, 0x38, 0x39, 0x3A},
	{"vdac", 0x3B, 0x3C, 0x3D, 0x3E},
	{"vintana1", 0x3F, 0x40, 0x41, 0x42},
	{"vintana2", 0x43, 0x44, 0x45, 0x46},
	{"vintdig", 0x47, 0x48, 0x49, 0x4A},
	{"vusb1v5", 0x71, 0x72, 0x73, NULL},
	{"vusb1v8", 0x74, 0x75, 0x76, NULL},
	{"vusb3v1", 0x77, 0x78, 0x79, NULL},
	{NULL, 0xFF, 0xFF, 0xFF, 0xFF}
};

static int pad_pmic_dbg_get(void *data, u64 *val)
{
	int ret = -EINVAL;
	int i;
	u32 *option = data;
	*val = *option;
	
	for(i=0; i < board_pmic_dbg[i].name != NULL; i++)
	{
		if(option == &pmic_dbg[i])
		{
			u8 reg_grp, reg_type, reg_remap, reg_dedicated;

			twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &reg_grp, board_pmic_dbg[i].addr_grp);
			twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &reg_type, board_pmic_dbg[i].addr_type);
			twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &reg_remap, board_pmic_dbg[i].addr_remap);
			twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &reg_dedicated, board_pmic_dbg[i].addr_dedicated);

			printk("(%2x, %2x) %s , grp = %x, type = %x, remap = %x, dedicated = %x\n", 0x48, 
				board_pmic_dbg[i].addr_grp, board_pmic_dbg[i].name, reg_grp, reg_type, reg_remap, reg_dedicated);
			break;
		}
	}

	return 0;
}

static int pad_pmic_dbg_set(void *data, u64 val)
{
	u32 *option = data;
	int i;
	*option = val;	
	
	for(i=0; i < board_pmic_dbg[i].name != NULL; i++)
	{
		if(option == &pmic_dbg[i])
		{
			if(val == 0)
			{
				printk("(%x, %x) %s , turn off!\n", 0x48, board_pmic_dbg[i].addr_grp, board_pmic_dbg[i].name);
				twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, board_pmic_dbg[i].addr_grp);
				//twl_i2c_write_u8(TWL6030_MODULE_ID0, 0, board_pmic_dbg[i].addr_state);
			}
			else
			{
				printk("(%x, %x) %s , turn on!\n", 0x48, board_pmic_dbg[i].addr_grp, board_pmic_dbg[i].name);
				twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 1, board_pmic_dbg[i].addr_grp);
				//twl_i2c_write_u8(TWL6030_MODULE_ID0, (0x7 << 5) | 0x1, board_pmic_dbg[i].addr_state);
			}
			break;
		}
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_dbg_pmic_reg_dump_fops, pad_pmic_reg_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(pm_dbg_pmic_dbg_fops, pad_pmic_dbg_get, pad_pmic_dbg_set, "%llu\n");
#endif


u32 voltage_off_while_idle;
int omap2_pm_debug;
u32 enable_off_mode;
u32 sleep_while_idle;
u32 wakeup_timer_seconds;
u32 wakeup_timer_milliseconds;

u32 omap4_device_off_counter = 0;
int pmd_clks_enable;
int dpll_cascade_global_state;

#define DUMP_PRM_MOD_REG(mod, reg)    \
	regs[reg_count].name = #mod "." #reg; \
	regs[reg_count++].val = prm_read_mod_reg(mod, reg)
#define DUMP_CM_MOD_REG(mod, reg)     \
	regs[reg_count].name = #mod "." #reg; \
	regs[reg_count++].val = cm_read_mod_reg(mod, reg)
#define DUMP_PRM_REG(reg) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = __raw_readl(reg)
#define DUMP_CM_REG(reg) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = __raw_readl(reg)
#define DUMP_INTC_REG(reg, off) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = \
			 __raw_readl(OMAP2_L4_IO_ADDRESS(0x480fe000 + (off)))

void omap2_pm_dump(int mode, int resume, unsigned int us)
{
	struct reg {
		const char *name;
		u32 val;
	} regs[32];
	int reg_count = 0, i;
	const char *s1 = NULL, *s2 = NULL;

	if (!resume) {
#if 0
		/* MPU */
		DUMP_PRM_MOD_REG(OCP_MOD, OMAP2_PRM_IRQENABLE_MPU_OFFSET);
		DUMP_CM_MOD_REG(MPU_MOD, OMAP2_CM_CLKSTCTRL);
		DUMP_PRM_MOD_REG(MPU_MOD, OMAP2_PM_PWSTCTRL);
		DUMP_PRM_MOD_REG(MPU_MOD, OMAP2_PM_PWSTST);
		DUMP_PRM_MOD_REG(MPU_MOD, PM_WKDEP);
#endif
#if 0
		/* INTC */
		DUMP_INTC_REG(INTC_MIR0, 0x0084);
		DUMP_INTC_REG(INTC_MIR1, 0x00a4);
		DUMP_INTC_REG(INTC_MIR2, 0x00c4);
#endif
#if 0
		DUMP_CM_MOD_REG(CORE_MOD, CM_FCLKEN1);
		if (cpu_is_omap24xx()) {
			DUMP_CM_MOD_REG(CORE_MOD, OMAP24XX_CM_FCLKEN2);
			DUMP_PRM_MOD_REG(OMAP24XX_GR_MOD,
					OMAP2_PRCM_CLKEMUL_CTRL_OFFSET);
			DUMP_PRM_MOD_REG(OMAP24XX_GR_MOD,
					OMAP2_PRCM_CLKSRC_CTRL_OFFSET);
		}
		DUMP_CM_MOD_REG(WKUP_MOD, CM_FCLKEN);
		DUMP_CM_MOD_REG(CORE_MOD, CM_ICLKEN1);
		DUMP_CM_MOD_REG(CORE_MOD, CM_ICLKEN2);
		DUMP_CM_MOD_REG(WKUP_MOD, CM_ICLKEN);
		DUMP_CM_MOD_REG(PLL_MOD, CM_CLKEN);
		DUMP_CM_MOD_REG(PLL_MOD, CM_AUTOIDLE);
		DUMP_PRM_MOD_REG(CORE_MOD, OMAP2_PM_PWSTST);
#endif
#if 0
		/* DSP */
		if (cpu_is_omap24xx()) {
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_FCLKEN);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_ICLKEN);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_IDLEST);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_AUTOIDLE);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_CLKSEL);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, OMAP2_CM_CLKSTCTRL);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, OMAP2_RM_RSTCTRL);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, OMAP2_RM_RSTST);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, OMAP2_PM_PWSTCTRL);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, OMAP2_PM_PWSTST);
		}
#endif
	} else {
		DUMP_PRM_MOD_REG(CORE_MOD, PM_WKST1);
		if (cpu_is_omap24xx())
			DUMP_PRM_MOD_REG(CORE_MOD, OMAP24XX_PM_WKST2);
		DUMP_PRM_MOD_REG(WKUP_MOD, PM_WKST);
		DUMP_PRM_MOD_REG(OCP_MOD, OMAP2_PRCM_IRQSTATUS_MPU_OFFSET);
#if 1
		DUMP_INTC_REG(INTC_PENDING_IRQ0, 0x0098);
		DUMP_INTC_REG(INTC_PENDING_IRQ1, 0x00b8);
		DUMP_INTC_REG(INTC_PENDING_IRQ2, 0x00d8);
#endif
	}

	switch (mode) {
	case 0:
		s1 = "full";
		s2 = "retention";
		break;
	case 1:
		s1 = "MPU";
		s2 = "retention";
		break;
	case 2:
		s1 = "MPU";
		s2 = "idle";
		break;
	}

	if (!resume)
#ifdef CONFIG_NO_HZ
		printk(KERN_INFO
		       "--- Going to %s %s (next timer after %u ms)\n", s1, s2,
		       jiffies_to_msecs(get_next_timer_interrupt(jiffies) -
					jiffies));
#else
		printk(KERN_INFO "--- Going to %s %s\n", s1, s2);
#endif
	else
		printk(KERN_INFO "--- Woke up (slept for %u.%03u ms)\n",
			us / 1000, us % 1000);

	for (i = 0; i < reg_count; i++)
		printk(KERN_INFO "%-20s: 0x%08x\n", regs[i].name, regs[i].val);
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static void pm_dbg_regset_store(u32 *ptr);

static struct dentry *pm_dbg_dir;
struct dentry  *pm_dbg_main_dir;
static int pm_dbg_init_done;

static int __init pm_dbg_init(void);

enum {
	DEBUG_FILE_COUNTERS = 0,
	DEBUG_FILE_TIMERS,
};

struct pm_module_def {
	char name[16]; /* Name of the module */
	short type; /* CM or PRM */
	unsigned short offset;
	int low; /* First register address on this module */
	int high; /* Last register address on this module */
};

#define MOD_CM 0
#define MOD_PRM 1

static const struct pm_module_def *pm_dbg_reg_modules;
static const struct pm_module_def omap3_pm_reg_modules[] = {
	{ "IVA2", MOD_CM, OMAP3430_IVA2_MOD, 0, 0x4c },
	{ "OCP", MOD_CM, OCP_MOD, 0, 0x10 },
	{ "MPU", MOD_CM, MPU_MOD, 4, 0x4c },
	{ "CORE", MOD_CM, CORE_MOD, 0, 0x4c },
	{ "SGX", MOD_CM, OMAP3430ES2_SGX_MOD, 0, 0x4c },
	{ "WKUP", MOD_CM, WKUP_MOD, 0, 0x40 },
	{ "CCR", MOD_CM, PLL_MOD, 0, 0x70 },
	{ "DSS", MOD_CM, OMAP3430_DSS_MOD, 0, 0x4c },
	{ "CAM", MOD_CM, OMAP3430_CAM_MOD, 0, 0x4c },
	{ "PER", MOD_CM, OMAP3430_PER_MOD, 0, 0x4c },
	{ "EMU", MOD_CM, OMAP3430_EMU_MOD, 0x40, 0x54 },
	{ "NEON", MOD_CM, OMAP3430_NEON_MOD, 0x20, 0x48 },
	{ "USB", MOD_CM, OMAP3430ES2_USBHOST_MOD, 0, 0x4c },

	{ "IVA2", MOD_PRM, OMAP3430_IVA2_MOD, 0x50, 0xfc },
	{ "OCP", MOD_PRM, OCP_MOD, 4, 0x1c },
	{ "MPU", MOD_PRM, MPU_MOD, 0x58, 0xe8 },
	{ "CORE", MOD_PRM, CORE_MOD, 0x58, 0xf8 },
	{ "SGX", MOD_PRM, OMAP3430ES2_SGX_MOD, 0x58, 0xe8 },
	{ "WKUP", MOD_PRM, WKUP_MOD, 0xa0, 0xb0 },
	{ "CCR", MOD_PRM, PLL_MOD, 0x40, 0x70 },
	{ "DSS", MOD_PRM, OMAP3430_DSS_MOD, 0x58, 0xe8 },
	{ "CAM", MOD_PRM, OMAP3430_CAM_MOD, 0x58, 0xe8 },
	{ "PER", MOD_PRM, OMAP3430_PER_MOD, 0x58, 0xe8 },
	{ "EMU", MOD_PRM, OMAP3430_EMU_MOD, 0x58, 0xe4 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x20, 0xe4 },
	{ "NEON", MOD_PRM, OMAP3430_NEON_MOD, 0x58, 0xe8 },
	{ "USB", MOD_PRM, OMAP3430ES2_USBHOST_MOD, 0x58, 0xe8 },
	{ "", 0, 0, 0, 0 },
};

static const struct pm_module_def omap4_pm_reg_modules[] = {
	{ "OCP_CM1", MOD_CM, OMAP4430_CM1_OCP_SOCKET_MOD, 0x0, 0x40 },
	{ "CKGEN_CM1", MOD_CM, OMAP4430_CM1_CKGEN_MOD, 0x0, 0x180 },
	{ "MPU", MOD_CM, OMAP4430_CM1_MPU_MOD, 0x0, 0x20 },
	{ "TESLA", MOD_CM, OMAP4430_CM1_TESLA_MOD, 0x0, 0x20 },
	{ "ABE", MOD_CM, OMAP4430_CM1_ABE_MOD, 0x0, 0x88 },
	{ "RESTORE", MOD_CM, OMAP4430_CM1_RESTORE_MOD, 0x0, 0x40 },

	{ "OCP_CM2", MOD_CM, OMAP4430_CM2_OCP_SOCKET_MOD, 0x0, 0x40 },
	{ "CKGEN_CM2", MOD_CM, OMAP4430_CM2_CKGEN_MOD, 0x0, 0xec },
	{ "ALWAYS_ON", MOD_CM, OMAP4430_CM2_ALWAYS_ON_MOD, 0x0, 0x40 },
	{ "CORE", MOD_CM, OMAP4430_CM2_CORE_MOD, 0x0, 0x740 },
	{ "IVAHD", MOD_CM, OMAP4430_CM2_IVAHD_MOD, 0x0, 0x28 },
	{ "CAM", MOD_CM, OMAP4430_CM2_CAM_MOD, 0x0, 0x28 },
	{ "DSS", MOD_CM, OMAP4430_CM2_DSS_MOD, 0x0, 0x28 },
	{ "GFX", MOD_CM, OMAP4430_CM2_GFX_MOD, 0x0, 0x20 },
	{ "L3INIT", MOD_CM, OMAP4430_CM2_L3INIT_MOD, 0x0, 0xe0 },
	{ "L4PER", MOD_CM, OMAP4430_CM2_L4PER_MOD, 0x0, 0x1d8 },
	{ "CEFUSE", MOD_CM, OMAP4430_CM2_CEFUSE_MOD, 0x0, 0x20 },
	{ "RESTORE", MOD_CM, OMAP4430_CM2_RESTORE_MOD, 0x0, 0x5c },

	{ "EMU_CM", MOD_CM, OMAP4430_PRM_EMU_CM_MOD, 0x0, 0x20 },
	{ "WKUP_CM", MOD_CM, OMAP4430_PRM_WKUP_CM_MOD, 0x0, 0x88 },

	{ "OCP", MOD_PRM, OMAP4430_PRM_OCP_SOCKET_MOD, 0x0, 0x40 },
	{ "CKGEN", MOD_PRM, OMAP4430_PRM_CKGEN_MOD, 0x0, 0x10 },
	{ "MPU", MOD_PRM, OMAP4430_PRM_MPU_MOD, 0x0, 0x24 },
	{ "TESLA", MOD_PRM, OMAP4430_PRM_TESLA_MOD, 0x0, 0x24 },
	{ "ABE", MOD_PRM, OMAP4430_PRM_ABE_MOD, 0x0, 0x8c },
	{ "ALWAYS_ON", MOD_PRM, OMAP4430_PRM_ALWAYS_ON_MOD, 0x24, 0x3c },
	{ "CORE", MOD_PRM, OMAP4430_PRM_CORE_MOD, 0x0, 0x744 },
	{ "IVAHD", MOD_PRM, OMAP4430_PRM_IVAHD_MOD, 0x0, 0x2c },
	{ "CAM", MOD_PRM, OMAP4430_PRM_CAM_MOD, 0x0, 0x2c },
	{ "DSS", MOD_PRM, OMAP4430_PRM_DSS_MOD, 0x0, 0x2c },
	{ "GFX", MOD_PRM, OMAP4430_PRM_GFX_MOD, 0x0, 0x24 },
	{ "L3INIT", MOD_PRM, OMAP4430_PRM_L3INIT_MOD, 0x0, 0xe4 },
	{ "L4PER", MOD_PRM, OMAP4430_PRM_L4PER_MOD, 0x0, 0x1dc },
	{ "CEFUSE", MOD_PRM, OMAP4430_PRM_CEFUSE_MOD, 0x0, 0x24 },
	{ "WKUP", MOD_PRM, OMAP4430_PRM_WKUP_MOD, 0x24, 0x84 },
	{ "EMU", MOD_PRM, OMAP4430_PRM_EMU_MOD, 0x0, 0x24 },
	{ "DEVICE", MOD_PRM, OMAP4430_PRM_DEVICE_MOD, 0x0, 0xf8 },

	{ "MPU_OCP", MOD_PRM, OMAP4430_PRCM_MPU_OCP_SOCKET_PRCM_MOD, 0x0, 0x0 },
	{ "MPU_DEVICE", MOD_PRM, OMAP4430_PRCM_MPU_DEVICE_PRM_MOD, 0x0, 0x4 },
	{ "MPU_CPU0", MOD_PRM, OMAP4430_PRCM_MPU_CPU0_MOD, 0x0, 0x18 },
	{ "MPU_CPU1", MOD_PRM, OMAP4430_PRCM_MPU_CPU1_MOD, 0x0, 0x18 },
	{ "", 0, 0, 0, 0 },
};

#define PM_DBG_MAX_REG_SETS 4

static void *pm_dbg_reg_set[PM_DBG_MAX_REG_SETS];

static int pm_dbg_get_regset_size(void)
{
	static int regset_size;

	if (regset_size == 0) {
		int i = 0;

		while (pm_dbg_reg_modules[i].name[0] != 0) {
			regset_size += pm_dbg_reg_modules[i].high +
				4 - pm_dbg_reg_modules[i].low;
			i++;
		}
	}
	return regset_size;
}

static int pm_dbg_show_regs(struct seq_file *s, void *unused)
{
	int i, j;
	unsigned long val;
	int reg_set = (int)s->private;
	u32 *ptr;
	void *store = NULL;
	int regs;
	int linefeed;

	if (reg_set == 0) {
		store = kmalloc(pm_dbg_get_regset_size(), GFP_KERNEL);
		if (!store) {
			WARN_ON(1);
			return -ENOMEM;
		}
		ptr = store;
		pm_dbg_regset_store(ptr);
	} else {
		ptr = pm_dbg_reg_set[reg_set - 1];
	}

	i = 0;

	while (pm_dbg_reg_modules[i].name[0] != 0) {
		regs = 0;
		linefeed = 0;
		if (pm_dbg_reg_modules[i].type == MOD_CM)
			seq_printf(s, "MOD: CM_%s (%08x)\n",
				pm_dbg_reg_modules[i].name,
				(u32)(OMAP3430_CM_BASE +
				pm_dbg_reg_modules[i].offset));
		else
			seq_printf(s, "MOD: PRM_%s (%08x)\n",
				pm_dbg_reg_modules[i].name,
				(u32)(OMAP3430_PRM_BASE +
				pm_dbg_reg_modules[i].offset));

		for (j = pm_dbg_reg_modules[i].low;
			j <= pm_dbg_reg_modules[i].high; j += 4) {
			val = *(ptr++);
			if (val != 0) {
				regs++;
				if (linefeed) {
					seq_printf(s, "\n");
					linefeed = 0;
				}
				seq_printf(s, "  %02x => %08lx", j, val);
				if (regs % 4 == 0)
					linefeed = 1;
			}
		}
		seq_printf(s, "\n");
		i++;
	}

	if (store != NULL)
		kfree(store);

	return 0;
}

static void pm_dbg_regset_store(u32 *ptr)
{
	int i, j;
	u32 val;

	i = 0;

	while (pm_dbg_reg_modules[i].name[0] != 0) {
		for (j = pm_dbg_reg_modules[i].low;
			j <= pm_dbg_reg_modules[i].high; j += 4) {
			if (pm_dbg_reg_modules[i].type == MOD_CM)
				val = cm_read_mod_reg(
					pm_dbg_reg_modules[i].offset, j);
			else
				val = prm_read_mod_reg(
					pm_dbg_reg_modules[i].offset, j);
			*(ptr++) = val;
		}
		i++;
	}
}

int pm_dbg_regset_save(int reg_set)
{
	if (pm_dbg_reg_set[reg_set-1] == NULL)
		return -EINVAL;

	pm_dbg_regset_store(pm_dbg_reg_set[reg_set-1]);

	return 0;
}

static const char pwrdm_state_names[][PWRDM_MAX_PWRSTS] = {
	"OFF",
	"RET",
	"INA",
	"ON"
};

void pm_dbg_update_time(struct powerdomain *pwrdm, int prev)
{
	s64 t;

	if (!pm_dbg_init_done)
		return ;

	/* Update timer for previous state */
	t = sched_clock();

	pwrdm->state_timer[prev] += t - pwrdm->timer;

	pwrdm->timer = t;
}

void omap2_pm_wakeup_on_timer(u32 seconds, u32 milliseconds)
{
	u32 tick_rate, cycles;

	if (!seconds && !milliseconds)
		return;

	tick_rate = clk_get_rate(omap_dm_timer_get_fclk(gptimer_wakeup));
	cycles = tick_rate * seconds + tick_rate * milliseconds / 1000;
	omap_dm_timer_stop(gptimer_wakeup);
	omap_dm_timer_set_load_start(gptimer_wakeup, 0, 0xffffffff - cycles);

	pr_info("PM: Resume timer in %u.%03u secs"
		" (%d ticks at %d ticks/sec.)\n",
		seconds, milliseconds, cycles, tick_rate);
}

static int clkdm_dbg_show_counter(struct clockdomain *clkdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;

	if (strcmp(clkdm->name, "emu_clkdm") == 0 ||
		strcmp(clkdm->name, "wkup_clkdm") == 0 ||
		strncmp(clkdm->name, "dpll", 4) == 0)
		return 0;

	seq_printf(s, "%s->%s (%d)", clkdm->name,
			clkdm->pwrdm.ptr->name,
			atomic_read(&clkdm->usecount));
	seq_printf(s, "\n");

	return 0;
}

static int pwrdm_dbg_show_counter(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;
	int i;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	if (pwrdm->state != pwrdm_read_pwrst(pwrdm))
		printk(KERN_ERR "pwrdm state mismatch(%s) %d != %d\n",
			pwrdm->name, pwrdm->state, pwrdm_read_pwrst(pwrdm));

	seq_printf(s, "%s (%s)", pwrdm->name,
			pwrdm_state_names[pwrdm->state]);
	for (i = 0; i < PWRDM_MAX_PWRSTS; i++)
		seq_printf(s, ",%s:%d", pwrdm_state_names[i],
			pwrdm->state_counter[i]);

	seq_printf(s, ",RET-LOGIC-OFF:%d", pwrdm->ret_logic_off_counter);
	for (i = 0; i < pwrdm->banks; i++)
		seq_printf(s, ",RET-MEMBANK%d-OFF:%d", i + 1,
				pwrdm->ret_mem_off_counter[i]);

	seq_printf(s, "\n");

	return 0;
}

static int pwrdm_dbg_show_timer(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;
	int i;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	pwrdm_state_switch(pwrdm);

	seq_printf(s, "%s (%s)", pwrdm->name,
		pwrdm_state_names[pwrdm->state]);

	for (i = 0; i < 4; i++)
		seq_printf(s, ",%s:%lld", pwrdm_state_names[i],
			pwrdm->state_timer[i]);

	seq_printf(s, "\n");
	return 0;
}

static int pm_dbg_show_counters(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_counter, s);
	if (cpu_is_omap44xx())
		seq_printf(s, "DEVICE-OFF:%d\n", omap4_device_off_counter);
	clkdm_for_each(clkdm_dbg_show_counter, s);

	return 0;
}

static int pm_dbg_show_timers(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_timer, s);
	return 0;
}

static int pm_dbg_open(struct inode *inode, struct file *file)
{
	switch ((int)inode->i_private) {
	case DEBUG_FILE_COUNTERS:
		return single_open(file, pm_dbg_show_counters,
			&inode->i_private);
	case DEBUG_FILE_TIMERS:
	default:
		return single_open(file, pm_dbg_show_timers,
			&inode->i_private);
	};
}

static int pm_dbg_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pm_dbg_show_regs, inode->i_private);
}

static const struct file_operations debug_fops = {
	.open           = pm_dbg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static const struct file_operations debug_reg_fops = {
	.open           = pm_dbg_reg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

int pm_dbg_regset_init(int reg_set)
{
	char name[2];

	if (!pm_dbg_init_done)
		pm_dbg_init();

	if (reg_set < 1 || reg_set > PM_DBG_MAX_REG_SETS ||
		pm_dbg_reg_set[reg_set-1] != NULL)
		return -EINVAL;

	pm_dbg_reg_set[reg_set-1] =
		kmalloc(pm_dbg_get_regset_size(), GFP_KERNEL);

	if (pm_dbg_reg_set[reg_set-1] == NULL)
		return -ENOMEM;

	if (pm_dbg_dir != NULL) {
		sprintf(name, "%d", reg_set);

		(void) debugfs_create_file(name, S_IRUGO,
			pm_dbg_dir, (void *)reg_set, &debug_reg_fops);
	}

	return 0;
}

static int pwrdm_suspend_get(void *data, u64 *val)
{
	int ret = -EINVAL;

	if (cpu_is_omap34xx())
		ret = omap3_pm_get_suspend_state((struct powerdomain *)data);
	*val = ret;

	if (ret >= 0)
		return 0;
	return *val;
}

static int pwrdm_suspend_set(void *data, u64 val)
{
	if (cpu_is_omap34xx())
		return omap3_pm_set_suspend_state(
			(struct powerdomain *)data, (int)val);
	return -EINVAL;
}

DEFINE_SIMPLE_ATTRIBUTE(pwrdm_suspend_fops, pwrdm_suspend_get,
			pwrdm_suspend_set, "%llu\n");

static int __init pwrdms_setup(struct powerdomain *pwrdm, void *dir)
{
	int i;
	s64 t;
	struct dentry *d;

	t = sched_clock();

	for (i = 0; i < 4; i++)
		pwrdm->state_timer[i] = 0;

	pwrdm->timer = t;

	if (strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	d = debugfs_create_dir(pwrdm->name, (struct dentry *)dir);

	(void) debugfs_create_file("suspend", S_IRUGO|S_IWUSR, d,
			(void *)pwrdm, &pwrdm_suspend_fops);

	return 0;
}

static int option_get(void *data, u64 *val)
{
	u32 *option = data;

	*val = *option;

	return 0;
}

static int option_set(void *data, u64 val)
{
	u32 *option = data;

	if (option == &wakeup_timer_milliseconds && val >= 1000)
		return -EINVAL;

	if (cpu_is_omap44xx() && (omap_type() == OMAP2_DEVICE_TYPE_GP))
		*option = 0;
	else
		*option = val;

	if (option == &enable_off_mode) {
		if (cpu_is_omap34xx())
			omap3_pm_off_mode_enable(val);
		else if (cpu_is_omap44xx())
			omap4_pm_off_mode_enable(val);
	}


	if (option == &voltage_off_while_idle) {
		if (voltage_off_while_idle)
			prm_set_mod_reg_bits(OMAP3430_SEL_OFF_MASK, OMAP3430_GR_MOD,
					     OMAP3_PRM_VOLTCTRL_OFFSET);
		else
			prm_clear_mod_reg_bits(OMAP3430_SEL_OFF_MASK,
					       OMAP3430_GR_MOD,
					       OMAP3_PRM_VOLTCTRL_OFFSET);
	}
	if (option == &enable_sr_vp_debug && val)
		pr_notice("Beware that enabling this option will allow user "
			"to override the system defined vp and sr parameters "
			"All the updated parameters will take effect next "
			"time smartreflex is enabled. Also this option "
			"disables the automatic vp errorgain and sr errormin "
			"limit changes as per the voltage. Users will have "
			"to explicitly write values into the debug fs "
			"entries corresponding to these if they want to see "
			"them changing according to the VDD voltage\n");

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_dbg_option_fops, option_get, option_set, "%llu\n");

static int pmd_clks_set(void *data, u64 val)
{
	int *option = data;

	if (option != &pmd_clks_enable)
		return -EINVAL;

	if(cpu_is_omap44xx()) {
		struct clk *l3_instr_ick, *l3_main_3_ick, *ocp_wp_noc_ick;

		l3_instr_ick = clk_get(NULL, "l3_instr_ick");
		l3_main_3_ick = clk_get(NULL, "l3_main_3_ick");
		ocp_wp_noc_ick = clk_get(NULL, "ocp_wp_noc_ick");

		/* enable PMD clocks */
		if (!*option && val) {
			clk_enable(l3_instr_ick);
			clk_enable(l3_main_3_ick);
			clk_enable(ocp_wp_noc_ick);

			*option = val;
		/* disable PMD clocks */
		} else if (*option && !val) {
			clk_disable(l3_instr_ick);
			clk_disable(l3_main_3_ick);
			clk_disable(ocp_wp_noc_ick);

			*option = val;
		}
	}

	return 0;
}

static int pmd_clks_get(void *data, u64 *val)
{
	int *option = data;

	if (option != &pmd_clks_enable)
		return -EINVAL;

	*val = *option;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_debug_pmd_clks_fops, pmd_clks_get, pmd_clks_set,
		"%llu\n");

static int __init omap4_pmd_clks_init(void)
{
	struct clk *l3_instr_ick, *l3_main_3_ick, *ocp_wp_noc_ick;

	l3_instr_ick = clk_get(NULL, "l3_instr_ick");
	l3_main_3_ick = clk_get(NULL, "l3_main_3_ick");
	ocp_wp_noc_ick = clk_get(NULL, "ocp_wp_noc_ick");

#ifdef CONFIG_OMAP4_PMD_CLKS_ENABLE
	pmd_clks_enable = 1;

	if (!(l3_instr_ick->usecount))
		clk_enable(l3_instr_ick);
	if (!(l3_main_3_ick->usecount))
		clk_enable(l3_main_3_ick);
	if (!(ocp_wp_noc_ick->usecount))
		clk_enable(ocp_wp_noc_ick);
#else
	pmd_clks_enable = 0;

	if (l3_instr_ick->usecount)
		clk_disable(l3_instr_ick);
	if (l3_main_3_ick->usecount)
		clk_disable(l3_main_3_ick);
	if (ocp_wp_noc_ick->usecount)
		clk_disable(ocp_wp_noc_ick);
#endif

	return 0;
}

static int dpll_cascading_set(void *data, u64 val)
{
	int *option = data;

	if (option != &dpll_cascade_global_state)
		return -EINVAL;

	if (*option == 0 && val == 1) {
		*option = val;
		omap4_dpll_low_power_cascade_enter();
	} else if (*option == 1 && val == 0) {
		omap4_dpll_low_power_cascade_exit();
		*option = val;
	}

	return 0;
}

static int dpll_cascading_get(void *data, u64 *val)
{
	int *option = data;

	if (option != &dpll_cascade_global_state)
		return -EINVAL;

	*val = *option;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_debug_dpll_cascading_fops, dpll_cascading_get,
		dpll_cascading_set, "%llu\n");

static int __init pm_dbg_init(void)
{
	int i;
	struct dentry *d;
	char name[2];

	if (pm_dbg_init_done)
		return 0;

	if (cpu_is_omap34xx())
		pm_dbg_reg_modules = omap3_pm_reg_modules;
	else if (cpu_is_omap44xx())
		pm_dbg_reg_modules = omap4_pm_reg_modules;
	else
		printk(KERN_ERR "%s: only OMAP3 supported\n", __func__);

	d = debugfs_create_dir("pm_debug", NULL);
	if (IS_ERR(d))
		return PTR_ERR(d);

	(void) debugfs_create_file("count", S_IRUGO,
		d, (void *)DEBUG_FILE_COUNTERS, &debug_fops);
	(void) debugfs_create_file("time", S_IRUGO,
		d, (void *)DEBUG_FILE_TIMERS, &debug_fops);

	pwrdm_for_each(pwrdms_setup, (void *)d);

	pm_dbg_dir = debugfs_create_dir("registers", d);
	if (IS_ERR(pm_dbg_dir))
		return PTR_ERR(pm_dbg_dir);

	(void) debugfs_create_file("current", S_IRUGO,
		pm_dbg_dir, (void *)0, &debug_reg_fops);

	for (i = 0; i < PM_DBG_MAX_REG_SETS; i++)
		if (pm_dbg_reg_set[i] != NULL) {
			sprintf(name, "%d", i+1);
			(void) debugfs_create_file(name, S_IRUGO,
				pm_dbg_dir, (void *)(i+1), &debug_reg_fops);

		}
  
	(void) debugfs_create_file("enable_off_mode", S_IRUGO | S_IWUGO, d,
				   &enable_off_mode, &pm_dbg_option_fops);
	(void) debugfs_create_file("sleep_while_idle", S_IRUGO | S_IWUGO, d,
				   &sleep_while_idle, &pm_dbg_option_fops);
	(void) debugfs_create_file("wakeup_timer_seconds", S_IRUGO | S_IWUGO, d,
				   &wakeup_timer_seconds, &pm_dbg_option_fops);
	(void) debugfs_create_file("wakeup_timer_milliseconds",
			S_IRUGO | S_IWUGO, d, &wakeup_timer_milliseconds,
			&pm_dbg_option_fops);
	(void) debugfs_create_file("enable_sr_vp_debug",  S_IRUGO | S_IWUGO, d,
				   &enable_sr_vp_debug, &pm_dbg_option_fops);

	/* Only enable for >= 3430 ES2.1 . Going to 0V on anything under
      * ES2.1 will eventually cause a crash */
       if (omap_rev() > OMAP3430_REV_ES2_0)
              (void) debugfs_create_file("voltage_off_while_idle",
                                          S_IRUGO | S_IWUGO, d,
                                          &voltage_off_while_idle,
                                        &pm_dbg_option_fops);

#ifdef PMIC_DEBUGGING
{
	int ix = 0;
	struct dentry *pmic_dir;
	struct dentry *padconf_dir;
	pmic_dir = debugfs_create_dir("pmic", d);
	padconf_dir = debugfs_create_dir("padconf", d);

	for(ix = 0; ix < pmic_ldo_num; ix++)
	{
		(void) debugfs_create_file(board_pmic_dbg[ix].name, S_IRUGO | S_IWUGO, pmic_dir,
			&pmic_dbg[ix], &pm_dbg_pmic_dbg_fops);
	}

	(void) debugfs_create_file("pmic_reg_dump", S_IRUGO | S_IWUGO, d,
				   &pmic_reg_dump, &pm_dbg_pmic_reg_dump_fops);

	for(ix = 0; ix < PAD_CONF_SIZE; ix++)
	{
		(void) debugfs_create_file(board_pad_conf[ix].name, S_IRUGO | S_IWUGO, padconf_dir,
					   &pad_conf[ix], &pm_dbg_pad_conf_fops);
	}
}
#endif

	if (cpu_is_omap44xx()) {
		omap4_pmd_clks_init();
		debugfs_create_file("pmd_clks_enable", S_IRUGO|S_IWUGO, d,
				&pmd_clks_enable, &pm_debug_pmd_clks_fops);

		debugfs_create_file("dpll_cascade_enable", S_IRUGO|S_IWUGO, d,
				&dpll_cascade_global_state,
				&pm_debug_dpll_cascading_fops);
	}

	pm_dbg_main_dir = d;
	pm_dbg_init_done = 1;

	return 0;
}
postcore_initcall(pm_dbg_init);

#endif
