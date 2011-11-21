/*
 * OMAP3 Power Management Routines
 *
 * Copyright (C) 2006-2008 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 * Jouni Hogander
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/i2c/twl.h> 
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/bootmem.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/slab.h>


#include <linux/reboot.h>


#include <plat/sram.h>
#include <plat/clockdomain.h>
#include <plat/powerdomain.h>
#include <plat/control.h>
#include <plat/serial.h>
#include <plat/sdrc.h>
#include <plat/prcm.h>
#include <plat/gpmc.h>
#include <plat/dma.h>
#include <plat/usb.h>

#include <plat/omap3-gptimer12.h> // SAMSUNG egkim
#include <plat/mcbsp.h>
#include <asm/tlbflush.h>

#include <plat/smartreflex.h>

#include "cm.h"
#include "cm-regbits-34xx.h"
#include "prm-regbits-34xx.h"
#include "mux34xx.h"

#include "prm.h"
#include "pm.h"
#include "sdrc.h"

static struct clk *mbox_ick_handle;
static int android_suspend = 0 ;

/* modified for mp3 current -- begin */
int audio_on = 0 ;
/* modified for mp3 current -- end */
// SAMSUNG egkim [
extern void omap_acknowledge_interrupts();
static spinlock_t fleeting_lock = SPIN_LOCK_UNLOCKED;
// ]
/* Scratchpad offsets */
#define OMAP343X_TABLE_ADDRESS_OFFSET	   0xC4
#define OMAP343X_TABLE_VALUE_OFFSET	   0xC0
#define OMAP343X_CONTROL_REG_VALUE_OFFSET  0xC8
#define RTA_ERRATA_i608				(1 << 1)
#define PER_WAKEUP_ERRATA_i582			(1 << 0)
static u16 pm34xx_errata;
#define IS_PM34XX_ERRATA(id)			(pm34xx_errata & (id))
#define OMAP36XX_RTA_DISABLE	  	    0x0
#define OMAP36XX_CONTROL_MEM_RTA_CTRL	0x40C

#define OMAP3430_CM_SLEEPDEP_PER_EN_MPU                (1 << 1)
#define OMAP3430_CM_SLEEPDEP_PER_EN_IVA2               (1 << 2)

  
/* Secure ram save size - store the defaults */
static struct omap3_secure_copy_data secure_copy_data = {
	.size = 0xF040,
	/*60K + 64 Bytes header EMU/HS devices */
};

struct power_state {
	struct powerdomain *pwrdm;
	u32 next_state;
#ifdef CONFIG_SUSPEND
	u32 saved_state;
#endif
	struct list_head node;
};

static LIST_HEAD(pwrst_list);

static void (*_omap_sram_idle)(u32 *addr, int save_state);

static int (*_omap_save_secure_sram)(u32 *addr);

static struct powerdomain *mpu_pwrdm, *neon_pwrdm;
static struct powerdomain *core_pwrdm, *per_pwrdm;

static struct powerdomain *wkup_pwrdm;
static struct powerdomain *dss_pwrdm;
static struct powerdomain *cam_pwrdm;

static void omap3_enable_io_chain(void)
{
	int timeout = 0;

	if (omap_rev() >= OMAP3430_REV_ES3_1) {
		prm_set_mod_reg_bits(OMAP3430_EN_IO_CHAIN_MASK, WKUP_MOD,
				     PM_WKEN);
		/* Do a readback to assure write has been done */
		prm_read_mod_reg(WKUP_MOD, PM_WKEN);

		
		while (!(prm_read_mod_reg(WKUP_MOD, PM_WKST) &
			 OMAP3430_ST_IO_CHAIN_MASK)) {
			timeout++;
			if (timeout > 1000) {
				printk(KERN_ERR "Wake up daisy chain "
				       "activation failed.\n");
				return;
			}
			
		}
	}
}

static void omap3_disable_io_chain(void)
{
	if (omap_rev() >= OMAP3430_REV_ES3_1)
		prm_clear_mod_reg_bits(OMAP3430_EN_IO_CHAIN_MASK, WKUP_MOD,
				       PM_WKEN);
}

static void omap3_core_save_context(void)
{
	u32 control_padconf_off;

	/* Save the padconf registers */
	control_padconf_off = omap_ctrl_readl(OMAP343X_CONTROL_PADCONF_OFF);
	control_padconf_off |= START_PADCONF_SAVE;
	omap_ctrl_writel(control_padconf_off, OMAP343X_CONTROL_PADCONF_OFF);
	/* wait for the save to complete */
	while (!(omap_ctrl_readl(OMAP343X_CONTROL_GENERAL_PURPOSE_STATUS)
			& PADCONF_SAVE_DONE))
		udelay(1);

	/*
	 * Force write last pad into memory, as this can fail in some
	 * cases according to erratas 1.157, 1.185
	 */
	omap_ctrl_writel(omap_ctrl_readl(OMAP343X_PADCONF_ETK_D14),
		OMAP343X_CONTROL_MEM_WKUP + 0x2a0);

	/*
	 * override the value saved in scratchpad memory, errata i583
	 */
	if (omap_rev() <= OMAP3630_REV_ES1_1)
		omap_ctrl_writew(0x1f, OMAP343X_CONTROL_MEM_WKUP +
			OMAP3_CONTROL_PADCONF_SDRC_CKE1_OFFSET);

	/* Save the Interrupt controller context */
	omap_intc_save_context();
	/* Save the GPMC context */
	omap3_gpmc_save_context();
	/* Save the system control module context, padconf already save above*/
	omap3_control_save_context();
	omap2_dma_context_save();

}

static void omap3_core_restore_context(void)
{
	if (omap_rev() <= OMAP3630_REV_ES1_1) {
		/*
		 * errata i583 workaround, safe transition sequence for CKE1:
		 */
		omap_ctrl_writew(0x1b, OMAP2_CONTROL_PADCONFS +
			OMAP3_CONTROL_PADCONF_SDRC_CKE1_OFFSET);
		omap_ctrl_writew(0x19, OMAP2_CONTROL_PADCONFS +
			OMAP3_CONTROL_PADCONF_SDRC_CKE1_OFFSET);
		omap_ctrl_writew(0x18, OMAP2_CONTROL_PADCONFS +
			OMAP3_CONTROL_PADCONF_SDRC_CKE1_OFFSET);
	}
	/* Restore the control module context, padconf restored by h/w */
	omap3_control_restore_context();
	/* Restore the GPMC context */
	omap3_gpmc_restore_context();
	/* Restore the interrupt controller context */
	omap_intc_restore_context();
       omap2_dma_context_restore();
}

/**
 * omap3_secure_copy_data_set() - set up the secure ram copy size
 * @data - platform specific customization
 *
 * This function should be invoked by the board's init_irq function to update
 * data prior to pm_init call is invoked. This call be done to update based on
 * ppa used on that platform.
 *
 * Returns -EINVAL for bad values, and 0 if all good.
 */
int __init omap3_secure_copy_data_set(struct omap3_secure_copy_data *data)
{
	if (!data || !data->size)
		return -EINVAL;

	memcpy(&secure_copy_data, data, sizeof(secure_copy_data));

	return 0;
}
/*
 * FIXME: This function should be called before entering off-mode after
 * OMAP3 secure services have been accessed. Currently it is only called
 * once during boot sequence, but this works as we are not using secure
 * services.
 */
static void omap3_save_secure_ram_context(u32 target_mpu_state)
{
	u32 ret;
	struct clockdomain *clkd = mpu_pwrdm->pwrdm_clkdms[0];

	if (omap_type() != OMAP2_DEVICE_TYPE_GP) {
		/*
		 * MPU next state must be set to POWER_ON temporarily,
		 * otherwise the WFI executed inside the ROM code
		 * will hang the system.
		 */
		pwrdm_set_next_pwrst(mpu_pwrdm, PWRDM_POWER_ON);
		omap2_clkdm_deny_idle(clkd);
		ret = _omap_save_secure_sram((u32 *)
				__pa(omap3_secure_ram_storage));
		pwrdm_set_next_pwrst(mpu_pwrdm, target_mpu_state);
		omap2_clkdm_allow_idle(clkd);
		/* Following is for error tracking, it should not happen */
		if (ret) {
			printk(KERN_ERR "save_secure_sram() returns %08x\n",
				ret);
			while (1)
				;
		}
	}
}

/*
 * PRCM Interrupt Handler Helper Function
 *
 * The purpose of this function is to clear any wake-up events latched
 * in the PRCM PM_WKST_x registers. It is possible that a wake-up event
 * may occur whilst attempting to clear a PM_WKST_x register and thus
 * set another bit in this register. A while loop is used to ensure
 * that any peripheral wake-up events occurring while attempting to
 * clear the PM_WKST_x are detected and cleared.
 */
static int prcm_clear_mod_irqs(s16 module, u8 regs)
{
	u32 wkst, fclk, iclk, clken;
	u16 wkst_off = (regs == 3) ? OMAP3430ES2_PM_WKST3 : PM_WKST1;
	u16 fclk_off = (regs == 3) ? OMAP3430ES2_CM_FCLKEN3 : CM_FCLKEN1;
	u16 iclk_off = (regs == 3) ? CM_ICLKEN3 : CM_ICLKEN1;
	u16 grpsel_off = (regs == 3) ?
		OMAP3430ES2_PM_MPUGRPSEL3 : OMAP3430_PM_MPUGRPSEL;
	int c = 0;

	wkst = prm_read_mod_reg(module, wkst_off);
	wkst &= prm_read_mod_reg(module, grpsel_off);
	if (wkst) {
		iclk = cm_read_mod_reg(module, iclk_off);
		fclk = cm_read_mod_reg(module, fclk_off);
		while (wkst) {
			clken = wkst;
			cm_set_mod_reg_bits(clken, module, iclk_off);
			/*
			 * For USBHOST, we don't know whether HOST1 or
			 * HOST2 woke us up, so enable both f-clocks
			 */
			if (module == OMAP3430ES2_USBHOST_MOD)
				clken |= 1 << OMAP3430ES2_EN_USBHOST2_SHIFT;
			cm_set_mod_reg_bits(clken, module, fclk_off);
			prm_write_mod_reg(wkst, module, wkst_off);
			wkst = prm_read_mod_reg(module, wkst_off);
			c++;
		}
		cm_write_mod_reg(iclk, module, iclk_off);
		cm_write_mod_reg(fclk, module, fclk_off);
	}

	return c;
}

static int _prcm_int_handle_wakeup(void)
{
	int c;

	/* By OMAP3630ES1.x and OMAP3430ES3.1 TRM, S/W must clear
	 * the EN_IO and EN_IO_CHAIN bits of WKEN_WKUP. Those bits
	 * would be set again by S/W in sleep sequences.
	 */
	if (omap_rev() >= OMAP3430_REV_ES3_1)
		prm_clear_mod_reg_bits(OMAP3430_EN_IO_MASK |
				       OMAP3430_EN_IO_CHAIN_MASK,
				       WKUP_MOD, PM_WKEN);

	c = prcm_clear_mod_irqs(WKUP_MOD, 1);
	c += prcm_clear_mod_irqs(CORE_MOD, 1);
	c += prcm_clear_mod_irqs(OMAP3430_PER_MOD, 1);
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		c += prcm_clear_mod_irqs(CORE_MOD, 3);
		c += prcm_clear_mod_irqs(OMAP3430ES2_USBHOST_MOD, 1);
	}

	return c;
}

/*
 * PRCM Interrupt Handler
 *
 * The PRM_IRQSTATUS_MPU register indicates if there are any pending
 * interrupts from the PRCM for the MPU. These bits must be cleared in
 * order to clear the PRCM interrupt. The PRCM interrupt handler is
 * implemented to simply clear the PRM_IRQSTATUS_MPU in order to clear
 * the PRCM interrupt. Please note that bit 0 of the PRM_IRQSTATUS_MPU
 * register indicates that a wake-up event is pending for the MPU and
 * this bit can only be cleared if the all the wake-up events latched
 * in the various PM_WKST_x registers have been cleared. The interrupt
 * handler is implemented using a do-while loop so that if a wake-up
 * event occurred during the processing of the prcm interrupt handler
 * (setting a bit in the corresponding PM_WKST_x register and thus
 * preventing us from clearing bit 0 of the PRM_IRQSTATUS_MPU register)
 * this would be handled.
 */
static irqreturn_t prcm_interrupt_handler (int irq, void *dev_id)
{
	u32 irqenable_mpu, irqstatus_mpu;
	int c = 0;
	int ct = 0;                //OMAPS00241975

	irqenable_mpu = prm_read_mod_reg(OCP_MOD,
					 OMAP3_PRM_IRQENABLE_MPU_OFFSET);
	irqstatus_mpu = prm_read_mod_reg(OCP_MOD,
					 OMAP3_PRM_IRQSTATUS_MPU_OFFSET);
	irqstatus_mpu &= irqenable_mpu;

	do {
		if (irqstatus_mpu & (OMAP3430_WKUP_ST_MASK |
				     OMAP3430_IO_ST_MASK)) {
			c = _prcm_int_handle_wakeup();
			ct++;              //OMAPS00241975

			/*
			 * Is the MPU PRCM interrupt handler racing with the
			 * IVA2 PRCM interrupt handler ?
			 */
			WARN(!c && (ct == 1), "prcm: WARNING: PRCM indicated "        
				"MPU wakeup but no wakeup sources are "
				"marked\n");                                //OMAPS00241975
		} else {
			/* XXX we need to expand our PRCM interrupt handler */
			WARN(1, "prcm: WARNING: PRCM interrupt received, but "
			     "no code to handle it (%08x)\n", irqstatus_mpu);
		}

		prm_write_mod_reg(irqstatus_mpu, OCP_MOD,
					OMAP3_PRM_IRQSTATUS_MPU_OFFSET);

		irqstatus_mpu = prm_read_mod_reg(OCP_MOD,
					OMAP3_PRM_IRQSTATUS_MPU_OFFSET);
		irqstatus_mpu &= irqenable_mpu;

	} while (irqstatus_mpu);

	return IRQ_HANDLED;
}

static void restore_control_register(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c1, c0, 0" : : "r" (val));
}

/* Function to restore the table entry that was modified for enabling MMU */
static void restore_table_entry(void)
{
	void __iomem *scratchpad_address;
	u32 previous_value, control_reg_value;
	u32 *address;

	scratchpad_address = OMAP2_L4_IO_ADDRESS(OMAP343X_SCRATCHPAD);

	/* Get address of entry that was modified */
	address = (u32 *)__raw_readl(scratchpad_address +
				     OMAP343X_TABLE_ADDRESS_OFFSET);
	/* Get the previous value which needs to be restored */
	previous_value = __raw_readl(scratchpad_address +
				     OMAP343X_TABLE_VALUE_OFFSET);
	address = __va(address);
	*address = previous_value;
	flush_tlb_all();
	control_reg_value = __raw_readl(scratchpad_address
					+ OMAP343X_CONTROL_REG_VALUE_OFFSET);
	/* This will enable caches and prediction */
	restore_control_register(control_reg_value);
}

/* modified for mp3 current -- begin */
void omap_dpll3_errat_wa(int disable)
{
	/* enable/disable autoidle */
	if (!disable)
		cm_rmw_mod_reg_bits(OMAP3430_AUTO_CORE_DPLL_MASK,
					0x1, PLL_MOD, CM_AUTOIDLE);
	else		
		cm_rmw_mod_reg_bits(OMAP3430_AUTO_CORE_DPLL_MASK,
					0x0, PLL_MOD, CM_AUTOIDLE);
	return;
}
EXPORT_SYMBOL(omap_dpll3_errat_wa);
/* modified for mp3 current -- end */

void omap_sram_idle(void)
{
	/* Variable to tell what needs to be saved and restored
	 * in omap_sram_idle*/
	/* save_state = 0 => Nothing to save and restored */
	/* save_state = 1 => Only L1 and logic lost */
	/* save_state = 2 => Only L2 lost */
	/* save_state = 3 => L1, L2 and logic lost */
	int save_state = 0;
	int mpu_next_state = PWRDM_POWER_ON;
	int per_next_state = PWRDM_POWER_ON;
	int core_next_state = PWRDM_POWER_ON;
	int core_prev_state, per_prev_state;
	u32 sdrc_pwr = 0;
	int per_state_modified = 0;
	int per_context_saved = 0;

	int dss_next_state = PWRDM_POWER_ON;
	int dss_state_modified = 0;
//idle current optimisation 	
	u32 reg,cam_clks = 0;
//idle current optimisation 	

	if (!_omap_sram_idle)
		return;

	pwrdm_clear_all_prev_pwrst(mpu_pwrdm);
	pwrdm_clear_all_prev_pwrst(neon_pwrdm);
	pwrdm_clear_all_prev_pwrst(core_pwrdm);
	pwrdm_clear_all_prev_pwrst(per_pwrdm);
	pwrdm_clear_all_prev_pwrst(dss_pwrdm);

	mpu_next_state = pwrdm_read_next_pwrst(mpu_pwrdm);
	switch (mpu_next_state) {
	case PWRDM_POWER_ON:
	case PWRDM_POWER_RET:
		/* No need to save context */
		save_state = 0;
		break;
	case PWRDM_POWER_OFF:
		save_state = 3;
		break;
	default:
		/* Invalid state */
		printk(KERN_ERR "Invalid mpu state in sram_idle\n");
		return;
	}
	pwrdm_pre_transition();

	/* NEON control */
	if (pwrdm_read_pwrst(neon_pwrdm) == PWRDM_POWER_ON)
		pwrdm_set_next_pwrst(neon_pwrdm, mpu_next_state);

	/* Enable IO-PAD and IO-CHAIN wakeups */
	per_next_state = pwrdm_read_next_pwrst(per_pwrdm);
	core_next_state = pwrdm_read_next_pwrst(core_pwrdm);
	dss_next_state = pwrdm_read_next_pwrst(dss_pwrdm);
	if (per_next_state < PWRDM_POWER_ON ||
			core_next_state < PWRDM_POWER_ON) {
		prm_set_mod_reg_bits(OMAP3430_EN_IO_MASK, WKUP_MOD, PM_WKEN);
		omap3_enable_io_chain();
	}

	/* DSS */
	if(dss_next_state < PWRDM_POWER_ON){
		if(dss_next_state == PWRDM_POWER_OFF){
			if(core_next_state == PWRDM_POWER_ON){
				dss_next_state = PWRDM_POWER_RET;
				pwrdm_set_next_pwrst(dss_pwrdm, dss_next_state);
				dss_state_modified = 1;
			}
			/* allow dss sleep */
			clkdm_add_sleepdep(dss_pwrdm->pwrdm_clkdms[0],
					mpu_pwrdm->pwrdm_clkdms[0]);
		}else{
			dss_next_state = PWRDM_POWER_RET;
			pwrdm_set_next_pwrst(dss_pwrdm, dss_next_state);
		}
	}

	/* PER */
	if (per_next_state < PWRDM_POWER_ON) {
		if (per_next_state == PWRDM_POWER_OFF) {
			if (core_next_state == PWRDM_POWER_ON) {
				per_next_state = PWRDM_POWER_RET;
				pwrdm_set_next_pwrst(per_pwrdm, per_next_state);
				per_state_modified = 1;
			}
		}
		if (per_next_state == PWRDM_POWER_OFF)
			 per_context_saved = 1;
		
			omap2_gpio_prepare_for_idle(per_context_saved);
		    omap_uart_prepare_idle(2);
		}

	/* CORE */
	if (core_next_state <= PWRDM_POWER_RET) {

#ifdef CONFIG_PM
		omap3_save_scratchpad_contents();
#endif

             cm_rmw_mod_reg_bits(OMAP3430_AUTO_CORE_DPLL_MASK,
					0x1, PLL_MOD, CM_AUTOIDLE);
	}
	omap3_intc_prepare_idle();

	
	/* CORE */
	if (core_next_state < PWRDM_POWER_ON) {
		omap_uart_prepare_idle(0);
		omap_uart_prepare_idle(1);
		if (core_next_state == PWRDM_POWER_OFF) {
	           prm_set_mod_reg_bits(OMAP3430_AUTO_OFF_MASK,
					      OMAP3430_GR_MOD,
					     OMAP3_PRM_VOLTCTRL_OFFSET); 
			omap3_core_save_context();
			omap3_cm_save_context();
			/* Save MUSB context */
			musb_context_save_restore(save_context);
			
			/* PATCH - Off mode issue on HS device */
			if (omap_type() != OMAP2_DEVICE_TYPE_GP)
				omap3_save_secure_ram_context(mpu_next_state);
			
		}
		else if (core_next_state == PWRDM_POWER_RET) {
			prm_set_mod_reg_bits(OMAP3430_AUTO_RET_MASK,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VOLTCTRL_OFFSET);
			musb_context_save_restore(disable_clk);
		}
		 else {
			musb_context_save_restore(disable_clk);
		}
	}

//	omap3_intc_prepare_idle();

	/*
	* On EMU/HS devices ROM code restores a SRDC value
	* from scratchpad which has automatic self refresh on timeout
	* of AUTO_CNT = 1 enabled. This takes care of errata 1.142.
	* Hence store/restore the SDRC_POWER register here.
	*/
	if (omap_rev() >= OMAP3430_REV_ES3_0 &&
	    omap_type() != OMAP2_DEVICE_TYPE_GP &&
	    core_next_state == PWRDM_POWER_OFF)
		sdrc_pwr = sdrc_read_reg(SDRC_POWER);

	/*
	 * omap3_arm_context is the location where ARM registers
	 * get saved. The restore path then reads from this
	 * location and restores them back.
	 */
	 
	 if(android_suspend)
	 {
	       omap_writel((omap_readl(0x48005000) & 0xFFFFF7FF), 0x48005000); //UART functional clock enable runtime PM UART 3 issue
            
	 }
	_omap_sram_idle(omap3_arm_context, save_state);
	cpu_init();

	/* Restore normal SDRC POWER settings */
	if (omap_rev() >= OMAP3430_REV_ES3_0 &&
	    omap_type() != OMAP2_DEVICE_TYPE_GP &&
	    core_next_state == PWRDM_POWER_OFF)
		sdrc_write_reg(sdrc_pwr, SDRC_POWER);
		 if(android_suspend)
		 {
	       omap_writel((omap_readl(0x48005000) | 0x800), 0x48005000);//UART functional clock disable runtime PM UART 3 issue
		}
	/* Restore table entry modified during MMU restoration */
	if (pwrdm_read_prev_pwrst(mpu_pwrdm) == PWRDM_POWER_OFF)
		restore_table_entry();

	/* CORE */
	if (core_next_state < PWRDM_POWER_ON) {
		core_prev_state = pwrdm_read_prev_pwrst(core_pwrdm);
		if (core_prev_state == PWRDM_POWER_OFF) {
			omap3_core_restore_context();
			omap3_cm_restore_context();
			omap3_sram_restore_context();
			omap2_sms_restore_context();
			/* Restore MUSB context */
			musb_context_save_restore(restore_context);
			
		} else {
			musb_context_save_restore(enable_clk);
		}
		omap_uart_resume_idle(0);
		omap_uart_resume_idle(1);
		if (core_next_state == PWRDM_POWER_OFF)
			prm_clear_mod_reg_bits(OMAP3430_AUTO_OFF_MASK,
					       OMAP3430_GR_MOD,
					       OMAP3_PRM_VOLTCTRL_OFFSET);
		else if (core_next_state == PWRDM_POWER_RET)
			prm_clear_mod_reg_bits(OMAP3430_AUTO_RET_MASK,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VOLTCTRL_OFFSET); //TEST
	}
	omap3_intc_resume_idle();
  	 if (core_next_state <= PWRDM_POWER_RET) {
		cm_rmw_mod_reg_bits(OMAP3430_AUTO_CORE_DPLL_MASK,
						0x0, PLL_MOD, CM_AUTOIDLE);
	
	}

	/* PER */
	if (per_next_state < PWRDM_POWER_ON) {
		omap_uart_resume_idle(2);
		per_prev_state = pwrdm_read_prev_pwrst(per_pwrdm);
		
		omap2_gpio_resume_after_idle(per_context_saved);

		if (per_state_modified)
			pwrdm_set_next_pwrst(per_pwrdm, PWRDM_POWER_OFF);
		
		// modified for mp3 playback current -- begin
		if (per_prev_state == PWRDM_POWER_OFF) 
		{
		/* Don't attach mcbsp interrupt */
			prm_clear_mod_reg_bits(OMAP3430_EN_MCBSP2_MASK,
				  OMAP3430_PER_MOD, OMAP3430_PM_MPUGRPSEL);
		}
		// modified for mp3 playback current -- end

	}

	/* DSS */
	if (dss_next_state < PWRDM_POWER_ON) {
		if (dss_next_state == PWRDM_POWER_OFF){
			/* return to the previous state. */
			clkdm_del_sleepdep(dss_pwrdm->pwrdm_clkdms[0],
					mpu_pwrdm->pwrdm_clkdms[0]);
		}
		if (dss_state_modified)
			pwrdm_set_next_pwrst(dss_pwrdm, PWRDM_POWER_OFF);
	}

	/* Disable IO-PAD and IO-CHAIN wakeup */

	 if (per_next_state < PWRDM_POWER_ON ||
                       core_next_state < PWRDM_POWER_ON) {
		prm_clear_mod_reg_bits(OMAP3430_EN_IO_MASK, WKUP_MOD, PM_WKEN);
		omap3_disable_io_chain();
	}

	pwrdm_post_transition();
	
	if(!android_suspend)
		omap2_clkdm_allow_idle(mpu_pwrdm->pwrdm_clkdms[0]);
}

int omap3_can_sleep(void)
{
	if (!sleep_while_idle)
		return 0;
	if (!omap_uart_can_sleep())
		return 0;
	return 1;
}

/* This sets pwrdm state (other than mpu & core. Currently only ON &
 * RET are supported. Function is assuming that clkdm doesn't have
 * hw_sup mode enabled. */
int set_pwrdm_state(struct powerdomain *pwrdm, u32 state)
{
	u32 cur_state;
	int sleep_switch = 0;
	int ret = 0;

	if (pwrdm == NULL || IS_ERR(pwrdm))
		return -EINVAL;

	while (!(pwrdm->pwrsts & (1 << state))) {
		if (state == PWRDM_POWER_OFF)
			return ret;
		state--;
	}

	cur_state = pwrdm_read_next_pwrst(pwrdm);
	if (cur_state == state)
		return ret;

	/*
	 * Bridge pm handles dsp hibernation. just return success
	 * If OFF mode is not enabled, sleep switch is performed for IVA which is not
	 * necessary. This causes conflict between PM and bridge touching IVA reg.
	 * REVISIT: Bridge has to set powerstate based on enable_off_mode state.
	 */
	if (!strcmp(pwrdm->name, "iva2_pwrdm"))
		return 0;


	if (pwrdm_read_pwrst(pwrdm) < PWRDM_POWER_ON) {
		omap2_clkdm_wakeup(pwrdm->pwrdm_clkdms[0]);
		sleep_switch = 1;
		pwrdm_wait_transition(pwrdm);
	}

	ret = pwrdm_set_next_pwrst(pwrdm, state);
	if (ret) {
		printk(KERN_ERR "Unable to set state of powerdomain: %s\n",
		       pwrdm->name);
		goto err;
	}

	if (sleep_switch) {
		omap2_clkdm_allow_idle(pwrdm->pwrdm_clkdms[0]);
		pwrdm_wait_transition(pwrdm);
		pwrdm_state_switch(pwrdm);
	}

err:
	return ret;
}

static void omap3_pm_idle(void)
{
	local_irq_disable();
	local_fiq_disable();

	if (!omap3_can_sleep())
		goto out;

	if (omap_irq_pending() || need_resched())
		goto out;

	omap_sram_idle();

out:
	local_fiq_enable();
	local_irq_enable();
}

#ifdef CONFIG_SUSPEND
static suspend_state_t suspend_state;

static int omap3_pm_prepare(void)
{
	disable_hlt();
	return 0;
}

//TI Patch start : Wakeup source check routine
static void omap3_gpio_wkst_log(void )
{
#define	PAD_CONFIG_X_1_BASE_ADDRESS	0x48002030
#define PAD_CONFIG_X_1_END_ADDRESS	0x48002264
#define PAD_CONFIG_X_2_BASE_ADDRESS 0x480025A0
#define PAD_CONFIG_X_2_END_ADDRESS	0x480025F8

	u32 address_ptr = 0x00;
	u32 value = 0x00;
	u32 wkst = 0x00; 
	u32 start = 0x00;
	u32 end = 0x00;

	wkst = prm_read_mod_reg( WKUP_MOD, PM_WKST1 ); //WKUP ST

	if((wkst & 0x0108) == 0x00) //GPIO1 or Daisy chain GPIO (GPIO 2 ~ 6)
	{
		return;	//Not a GPIO wakeup
	}

	printk("[PM_GPIO_WAKEUP] PM_WKST_WKUP = 0x%x\n", wkst);


READ_ROUTINE:
	if(start == 0x00)
	{
		start = PAD_CONFIG_X_1_BASE_ADDRESS;
		end = PAD_CONFIG_X_1_END_ADDRESS;
	}
	else
	{
		start = PAD_CONFIG_X_2_BASE_ADDRESS;
		end = PAD_CONFIG_X_2_END_ADDRESS;
	}

	// First part
	for(address_ptr = start; address_ptr <= end; address_ptr += 0x04)
	{
		value = omap_readl(address_ptr);

		if( (value & 0x80000000) ) // Wakeup event check for the higher part
		{
			printk("[PM_GPIO_WAKEUP] ADDRESS = 0x%x (HIGH) - 0x%x\n", address_ptr, value);
		}
		
		if( (value & 0x8000) ) // Wakeup event check for the higher part
		{
			printk("[PM_GPIO_WAKEUP] ADDRESS = 0x%x (LOW) - 0x%x\n", address_ptr, value);
		}		
	}

	if(address_ptr < PAD_CONFIG_X_2_BASE_ADDRESS)
		goto READ_ROUTINE;

	return;

}
//TI Patch End : Wakeup source check routine

static int omap3_pm_suspend(void)
{
	struct power_state *pwrst;
	int state, ret = 0;
	u32 reg , reg1= 0 ;
	static int count = 1 ;

    // Checking GPtimer12 wakeup[+]
    static int gptimer_wakeup_count = 1;
	u32 wkup_st = 0;
	u32 ret_val = 0;
	u32 regval = 0;
	unsigned long fleeting_flag = 0;
    // Checking GPtimer12 wakeup[-]
	
	if (wakeup_timer_seconds || wakeup_timer_milliseconds)
		omap2_pm_wakeup_on_timer(wakeup_timer_seconds,
					 wakeup_timer_milliseconds);
#if 1 // Sleep debugging
	printk("[omap3_pm_suspend] CM_IDLEST1_CORE : %x\n", omap_readl(0x48004A20));
	printk("[omap3_pm_suspend] CM_IDLEST3_CORE : %x\n", omap_readl(0x48004A28));
	printk("[omap3_pm_suspend] CM_IDLEST_SGX : %x\n", omap_readl(0x48004B20));
	printk("[omap3_pm_suspend] CM_IDLEST_DSS : %x\n", omap_readl(0x48004E20));
	printk("[omap3_pm_suspend] CM_IDLEST_CAM : %x\n", omap_readl(0x48004F20));
	printk("[omap3_pm_suspend] CM_IDLEST_PER : %x\n", omap_readl(0x48005020));
	printk("[omap3_pm_suspend] CM_IDLEST_NEON : %x\n", omap_readl(0x48005320));
	printk("[omap3_pm_suspend] CM_IDLEST_USBHOST : %x\n", omap_readl(0x48005420));
#endif

pm_suspend_reenter: // SAMSUNG egkim
	/* Read current next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node)
		pwrst->saved_state = pwrdm_read_next_pwrst(pwrst->pwrdm);
	/* Set ones wanted by suspend */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (set_pwrdm_state(pwrst->pwrdm, pwrst->next_state))
			goto restore;
		if (pwrdm_clear_all_prev_pwrst(pwrst->pwrdm))
			goto restore;
	}

//idle current optimisation 
		omap_writel(omap_readl(0x48004E00)|0x0,0x48004E00);     //CM_FCLKEN_DSS->EN_DSS1       
		omap_writel(omap_readl(0x48004E10)|0x0, 0x48004E10);     //CM_ICLKEN_DSS->EN_DSS1	
		omap_writel((omap_readl(0x48004E44) |0x2), 0x48004E44); // CM_SLEEPDEP_DSS
//idle current optimisation 

	cm_rmw_mod_reg_bits(OMAP3430_AUTO_CORE_DPLL_MASK,
					0x8, PLL_MOD, CM_AUTOIDLE);       // AUTO_PERIPH_DPLL 
        	
	omap_uart_prepare_suspend();
	omap3_intc_suspend();

                 
	android_suspend = 1;
	omap_sram_idle();
	android_suspend = 0;
	omap3_gpio_wkst_log(); //TI Patch: Wakeup source check routine

	wkup_st = prm_read_mod_reg( WKUP_MOD, PM_WKST1 ); //WKUP ST

    // Checking GPtimer12 wakeup[+]
    if( wkup_st & 0x2 )
	{
		prm_write_mod_reg( 0x101CB, WKUP_MOD, PM_WKST1 ); //Clear WKST_WKUP REG

        ret_val = prm_read_mod_reg( OCP_MOD, OMAP2_PRCM_IRQSTATUS_MPU_OFFSET );  //MPU IRQ STATUS
        prm_write_mod_reg( ret_val, OCP_MOD, OMAP2_PRCM_IRQSTATUS_MPU_OFFSET ); //CLEAR MPU IRQ

		//printk("\nTimer ISR =%x",omap_readw(0x48304018));
		cm_write_mod_reg( cm_read_mod_reg( OCP_MOD,CM_ICLKEN3 ), OCP_MOD, CM_ICLKEN3 );

		spin_lock_irqsave( &fleeting_lock, fleeting_flag );
		ret_val = expire_gptimer12();
		spin_unlock_irqrestore( &fleeting_lock, fleeting_flag );

		omap_acknowledge_interrupts();
        gptimer_wakeup_count++;

		if ( !ret_val )
    		goto pm_suspend_reenter;
	} else{
	    #if 1
	    printk("wkup_st = 0x%04x\n" , wkup_st);
	    #else
		printk("wkup_st:%x, GPIO1:%x, GPIO2:%x, GPIO3:%x, GPIO4:%x, GPIO5:%x, GPIO6:%x\n", 
			wkup_st, omap_readl(0x48310018), omap_readl(0x49050018), omap_readl(0x49052018), 
			omap_readl(0x49054018), omap_readl(0x49056018), omap_readl(0x49058018));
		printk("GPIO1 wakeupenable : 0x%08x\n", omap_readl(0x48310020));
		#endif
	}
	
    gptimer_wakeup_count = 0;
    // Checking GPtimer12 wakeup[-]

restore:
	/* Restore next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		state = pwrdm_read_prev_pwrst(pwrst->pwrdm);
		if (state > pwrst->next_state) {
			printk(KERN_ERR "Powerdomain (%s) didn't enter "
			       "target state %d\n",
			       pwrst->pwrdm->name, pwrst->next_state);
			ret = -1;
		}
		set_pwrdm_state(pwrst->pwrdm, pwrst->saved_state);
	}
	if (ret)
		printk(KERN_ERR "Could not enter target state in pm_suspend\n");
	else
		printk(KERN_ERR "Successfully put all powerdomains "
		       "to target state\n");

	return ret;
}

static int omap3_pm_enter(suspend_state_t unused)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap3_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void omap3_pm_finish(void)
{
	enable_hlt();
}

/* Hooks to enable / disable UART interrupts during suspend */
static int omap3_pm_begin(suspend_state_t state)
{
	suspend_state = state;
	omap_uart_enable_irqs(0);
	return 0;
}

static void omap3_pm_end(void)
{
	suspend_state = PM_SUSPEND_ON;
	omap_uart_enable_irqs(1);
	return;
}

static struct platform_suspend_ops omap_pm_ops = {
	.begin		= omap3_pm_begin,
	.end		= omap3_pm_end,
	.prepare	= omap3_pm_prepare,
	.enter		= omap3_pm_enter,
	.finish		= omap3_pm_finish,
	.valid		= suspend_valid_only_mem,
};
#endif /* CONFIG_SUSPEND */


/**
 * omap3_iva_idle(): ensure IVA is in idle so it can be put into
 *                   retention
 *
 * In cases where IVA2 is activated by bootcode, it may prevent
 * full-chip retention or off-mode because it is not idle.  This
 * function forces the IVA2 into idle state so it can go
 * into retention/off and thus allow full-chip retention/off.
 *
 **/
static void __init omap3_iva_idle(void)
{
	/* ensure IVA2 clock is disabled */
	cm_write_mod_reg(0, OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* if no clock activity, nothing else to do */
	if (!(cm_read_mod_reg(OMAP3430_IVA2_MOD, OMAP3430_CM_CLKSTST) &
	      OMAP3430_CLKACTIVITY_IVA2_MASK))
		return;

	/* Reset IVA2 */
	prm_write_mod_reg(OMAP3430_RST1_IVA2_MASK |
			  OMAP3430_RST2_IVA2_MASK |
			  OMAP3430_RST3_IVA2_MASK,
			  OMAP3430_IVA2_MOD, OMAP2_RM_RSTCTRL);

	/* Enable IVA2 clock */
	cm_write_mod_reg(OMAP3430_CM_FCLKEN_IVA2_EN_IVA2_MASK,
			 OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* Set IVA2 boot mode to 'idle' */
	omap_ctrl_writel(OMAP3_IVA2_BOOTMOD_IDLE,
			 OMAP343X_CONTROL_IVA2_BOOTMOD);

	/* Un-reset IVA2 */
	prm_write_mod_reg(0, OMAP3430_IVA2_MOD, OMAP2_RM_RSTCTRL);

	/* Disable IVA2 clock */
	cm_write_mod_reg(0, OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* Reset IVA2 */
	prm_write_mod_reg(OMAP3430_RST1_IVA2_MASK |
			  OMAP3430_RST2_IVA2_MASK |
			  OMAP3430_RST3_IVA2_MASK,
			  OMAP3430_IVA2_MOD, OMAP2_RM_RSTCTRL);
}

static void __init omap3_d2d_idle(void)
{
	u16 mask, padconf;

	/* In a stand alone OMAP3430 where there is not a stacked
	 * modem for the D2D Idle Ack and D2D MStandby must be pulled
	 * high. S CONTROL_PADCONF_SAD2D_IDLEACK and
	 * CONTROL_PADCONF_SAD2D_MSTDBY to have a pull up. */
	mask = (1 << 4) | (1 << 3); /* pull-up, enabled */
	padconf = omap_ctrl_readw(OMAP3_PADCONF_SAD2D_MSTANDBY);
	padconf |= mask;
	omap_ctrl_writew(padconf, OMAP3_PADCONF_SAD2D_MSTANDBY);

	padconf = omap_ctrl_readw(OMAP3_PADCONF_SAD2D_IDLEACK);
	padconf |= mask;
	omap_ctrl_writew(padconf, OMAP3_PADCONF_SAD2D_IDLEACK);

	/* reset modem */
	prm_write_mod_reg(OMAP3430_RM_RSTCTRL_CORE_MODEM_SW_RSTPWRON_MASK |
			  OMAP3430_RM_RSTCTRL_CORE_MODEM_SW_RST_MASK,
			  CORE_MOD, OMAP2_RM_RSTCTRL);
	prm_write_mod_reg(0, CORE_MOD, OMAP2_RM_RSTCTRL);
}

static void __init prcm_setup_regs(void)
{
	u32 omap3630_auto_uart4 = cpu_is_omap3630() ? OMAP3630_AUTO_UART4 : 0;
	u32 omap3630_en_uart4 = cpu_is_omap3630() ? OMAP3630_EN_UART4 : 0;
	/* XXX Reset all wkdeps. This should be done when initializing
	 * powerdomains */
	prm_write_mod_reg(0, OMAP3430_IVA2_MOD, PM_WKDEP);
	prm_write_mod_reg(0, MPU_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_DSS_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_NEON_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_CAM_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_PER_MOD, PM_WKDEP);
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		
	/* This workaround is needed to prevent SGX and USBHOST from
	* failing to transition to RET/OFF after a warm reset in OFF
    * mode. Workaround sets a sleepdep of each of these domains
    * with MPU, waits for a min 2 sysclk cycles and clears the
    * sleepdep.
     */

             
    	cm_write_mod_reg(OMAP3430_CM_SLEEPDEP_PER_EN_MPU,
                            OMAP3430ES2_USBHOST_MOD, OMAP3430_CM_SLEEPDEP);
       	cm_write_mod_reg(OMAP3430_CM_SLEEPDEP_PER_EN_MPU,
             				OMAP3430ES2_SGX_MOD, OMAP3430_CM_SLEEPDEP);
       
       	udelay(100);
       
       	cm_write_mod_reg(0, OMAP3430ES2_USBHOST_MOD,
             				OMAP3430_CM_SLEEPDEP);
       	cm_write_mod_reg(0, OMAP3430ES2_SGX_MOD,
             				OMAP3430_CM_SLEEPDEP);

  
		prm_write_mod_reg(0, OMAP3430ES2_SGX_MOD, PM_WKDEP);
		prm_write_mod_reg(0, OMAP3430ES2_USBHOST_MOD, PM_WKDEP);
	} else
		prm_write_mod_reg(0, GFX_MOD, PM_WKDEP);

	/*
	 * Enable interface clock autoidle for all modules.
	 * Note that in the long run this should be done by clockfw
	 */
	cm_write_mod_reg(
		OMAP3430_AUTO_MODEM_MASK |
		OMAP3430ES2_AUTO_MMC3_MASK |
		OMAP3430ES2_AUTO_ICR_MASK |
		OMAP3430_AUTO_AES2_MASK |
		OMAP3430_AUTO_SHA12_MASK |
		OMAP3430_AUTO_DES2_MASK |
		OMAP3430_AUTO_MMC2_MASK |
		OMAP3430_AUTO_MMC1_MASK |
		OMAP3430_AUTO_MSPRO_MASK |
		OMAP3430_AUTO_HDQ_MASK |
		OMAP3430_AUTO_MCSPI4_MASK |
		OMAP3430_AUTO_MCSPI3_MASK |
		OMAP3430_AUTO_MCSPI2_MASK |
		OMAP3430_AUTO_MCSPI1_MASK |
		OMAP3430_AUTO_I2C3_MASK |
		OMAP3430_AUTO_I2C2_MASK |
		OMAP3430_AUTO_I2C1_MASK |
		OMAP3430_AUTO_UART2_MASK |
		OMAP3430_AUTO_UART1_MASK |
		OMAP3430_AUTO_GPT11_MASK |
		OMAP3430_AUTO_GPT10_MASK |
		OMAP3430_AUTO_MCBSP5_MASK |
		OMAP3430_AUTO_MCBSP1_MASK |
		OMAP3430ES1_AUTO_FAC_MASK | /* This is es1 only */
		OMAP3430_AUTO_MAILBOXES_MASK |
		OMAP3430_AUTO_OMAPCTRL_MASK |
		OMAP3430ES1_AUTO_FSHOSTUSB_MASK |
		OMAP3430_AUTO_HSOTGUSB_MASK |
		OMAP3430_AUTO_SAD2D_MASK |
		OMAP3430_AUTO_SSI_MASK,
		CORE_MOD, CM_AUTOIDLE1);

	cm_write_mod_reg(
		OMAP3430_AUTO_PKA_MASK |
		OMAP3430_AUTO_AES1_MASK |
		OMAP3430_AUTO_RNG_MASK |
		OMAP3430_AUTO_SHA11_MASK |
		OMAP3430_AUTO_DES1_MASK,
		CORE_MOD, CM_AUTOIDLE2);

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		cm_write_mod_reg(
			OMAP3430_AUTO_MAD2D_MASK |
			OMAP3430ES2_AUTO_USBTLL_MASK,
			CORE_MOD, CM_AUTOIDLE3);
	}

	cm_write_mod_reg(
		OMAP3430_AUTO_WDT2_MASK |
		OMAP3430_AUTO_WDT1_MASK |
		OMAP3430_AUTO_GPIO1_MASK |
		OMAP3430_AUTO_32KSYNC_MASK |
		OMAP3430_AUTO_GPT12_MASK |
		OMAP3430_AUTO_GPT1_MASK,
		WKUP_MOD, CM_AUTOIDLE);

	cm_write_mod_reg(
		OMAP3430_AUTO_DSS_MASK,
		OMAP3430_DSS_MOD,
		CM_AUTOIDLE);

	cm_write_mod_reg(
		OMAP3430_AUTO_CAM_MASK,
		OMAP3430_CAM_MOD,
		CM_AUTOIDLE);

	cm_write_mod_reg(
		omap3630_auto_uart4 |
		OMAP3430_AUTO_GPIO6_MASK |
		OMAP3430_AUTO_GPIO5_MASK |
		OMAP3430_AUTO_GPIO4_MASK |
		OMAP3430_AUTO_GPIO3_MASK |
		OMAP3430_AUTO_GPIO2_MASK |
		OMAP3430_AUTO_WDT3_MASK |
		OMAP3430_AUTO_UART3_MASK |
		OMAP3430_AUTO_GPT9_MASK |
		OMAP3430_AUTO_GPT8_MASK |
		OMAP3430_AUTO_GPT7_MASK |
		OMAP3430_AUTO_GPT6_MASK |
		OMAP3430_AUTO_GPT5_MASK |
		OMAP3430_AUTO_GPT4_MASK |
		OMAP3430_AUTO_GPT3_MASK |
		OMAP3430_AUTO_GPT2_MASK |
		OMAP3430_AUTO_MCBSP4_MASK |
		OMAP3430_AUTO_MCBSP3_MASK |
		OMAP3430_AUTO_MCBSP2_MASK,
		OMAP3430_PER_MOD,
		CM_AUTOIDLE);

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		cm_write_mod_reg(
			OMAP3430ES2_AUTO_USBHOST_MASK,
			OMAP3430ES2_USBHOST_MOD,
			CM_AUTOIDLE);
	}

	omap_ctrl_writel(OMAP3430_AUTOIDLE_MASK, OMAP2_CONTROL_SYSCONFIG);

	/*
	 * Set all plls to autoidle. This is needed until autoidle is
	 * enabled by clockfw
	 */
	cm_write_mod_reg(1 << OMAP3430_AUTO_IVA2_DPLL_SHIFT,
			 OMAP3430_IVA2_MOD, CM_AUTOIDLE2);
	cm_write_mod_reg(1 << OMAP3430_AUTO_MPU_DPLL_SHIFT,
			 MPU_MOD,
			 CM_AUTOIDLE2);
	cm_write_mod_reg((1 << OMAP3430_AUTO_PERIPH_DPLL_SHIFT) |
			 (1 << OMAP3430_AUTO_CORE_DPLL_SHIFT),
			 PLL_MOD,
			 CM_AUTOIDLE);
	cm_write_mod_reg(1 << OMAP3430ES2_AUTO_PERIPH2_DPLL_SHIFT,
			 PLL_MOD,
			 CM_AUTOIDLE2);

	/*
	 * Enable control of expternal oscillator through
	 * sys_clkreq. In the long run clock framework should
	 * take care of this.
	 */
	prm_rmw_mod_reg_bits(OMAP_AUTOEXTCLKMODE_MASK,
			     1 << OMAP_AUTOEXTCLKMODE_SHIFT,
			     OMAP3430_GR_MOD,
			     OMAP3_PRM_CLKSRC_CTRL_OFFSET);

	/* setup wakup source:
	 *  By OMAP3630ES1.x and OMAP3430ES3.1 TRM, S/W must take care
	 *  the EN_IO and EN_IO_CHAIN bits in sleep-wakeup sequences.
	 */
	prm_write_mod_reg(OMAP3430_EN_GPIO1_MASK | OMAP3430_EN_GPT1_MASK |
			  OMAP3430_EN_GPT12_MASK, WKUP_MOD, PM_WKEN);
	if (omap_rev() < OMAP3430_REV_ES3_1)
		prm_set_mod_reg_bits(OMAP3430_EN_IO_MASK, WKUP_MOD, PM_WKEN);
	/* No need to write EN_IO, that is always enabled */
	prm_write_mod_reg(OMAP3430_GRPSEL_GPIO1_MASK |
			  OMAP3430_GRPSEL_GPT1_MASK |
			  OMAP3430_GRPSEL_GPT12_MASK,
			  WKUP_MOD, OMAP3430_PM_MPUGRPSEL);
	/* For some reason IO doesn't generate wakeup event even if
	 * it is selected to mpu wakeup goup */
	prm_write_mod_reg(OMAP3430_IO_EN_MASK | OMAP3430_WKUP_EN_MASK,
			  OCP_MOD, OMAP3_PRM_IRQENABLE_MPU_OFFSET);

	/* Enable PM_WKEN to support DSS LPR */
	prm_write_mod_reg(OMAP3430_PM_WKEN_DSS_EN_DSS_MASK,
				OMAP3430_DSS_MOD, PM_WKEN);

	/* Enable wakeups in PER */
	prm_write_mod_reg(OMAP3430_EN_GPIO2_MASK | OMAP3430_EN_GPIO3_MASK |
			  OMAP3430_EN_GPIO4_MASK | OMAP3430_EN_GPIO5_MASK |
			  OMAP3430_EN_GPIO6_MASK | OMAP3430_EN_UART3_MASK |
			  OMAP3430_EN_MCBSP2_MASK | OMAP3430_EN_MCBSP3_MASK |
			  OMAP3430_EN_MCBSP4_MASK | omap3630_en_uart4,
			  OMAP3430_PER_MOD, PM_WKEN);
	/* and allow them to wake up MPU */
	prm_write_mod_reg(OMAP3430_GRPSEL_GPIO2_MASK |
			  OMAP3430_GRPSEL_GPIO3_MASK |
			  OMAP3430_GRPSEL_GPIO4_MASK |
			  OMAP3430_GRPSEL_GPIO5_MASK |
			  OMAP3430_GRPSEL_GPIO6_MASK |
			  OMAP3430_GRPSEL_UART3_MASK |
			  OMAP3430_GRPSEL_MCBSP2_MASK |
			  OMAP3430_GRPSEL_MCBSP3_MASK |
			  OMAP3430_GRPSEL_MCBSP4_MASK |
			  omap3630_en_uart4,
			  OMAP3430_PER_MOD, OMAP3430_PM_MPUGRPSEL);

	/* Don't attach IVA interrupts */
	prm_write_mod_reg(0, WKUP_MOD, OMAP3430_PM_IVAGRPSEL);
	prm_write_mod_reg(0, CORE_MOD, OMAP3430_PM_IVAGRPSEL1);
	prm_write_mod_reg(0, CORE_MOD, OMAP3430ES2_PM_IVAGRPSEL3);
	prm_write_mod_reg(0, OMAP3430_PER_MOD, OMAP3430_PM_IVAGRPSEL);

	/* Clear any pending 'reset' flags */
	prm_write_mod_reg(0xffffffff, MPU_MOD, OMAP2_RM_RSTST);
	prm_write_mod_reg(0xffffffff, CORE_MOD, OMAP2_RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_PER_MOD, OMAP2_RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_EMU_MOD, OMAP2_RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_NEON_MOD, OMAP2_RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_DSS_MOD, OMAP2_RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430ES2_USBHOST_MOD, OMAP2_RM_RSTST);

	/* Clear any pending PRCM interrupts */
	prm_write_mod_reg(0, OCP_MOD, OMAP3_PRM_IRQSTATUS_MPU_OFFSET);

	omap3_iva_idle();
	omap3_d2d_idle();
}

void omap3_pm_off_mode_enable(int enable)
{
	struct power_state *pwrst;
	u32 state;

	if (enable)
		state = PWRDM_POWER_OFF;
	else
		state = PWRDM_POWER_RET;

#ifdef CONFIG_CPU_IDLE
	omap3_cpuidle_update_states();
#endif

	list_for_each_entry(pwrst, &pwrst_list, node) {
		pwrst->next_state = state;
		if (IS_PM34XX_ERRATA(PER_WAKEUP_ERRATA_i582))
			if ((state == PWRDM_POWER_OFF) &&
			(!strcmp("per_pwrdm", pwrst->pwrdm->name)))
				continue;

		set_pwrdm_state(pwrst->pwrdm, state);
	}
}

int omap3_pm_get_suspend_state(struct powerdomain *pwrdm)
{
	struct power_state *pwrst;

	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (pwrst->pwrdm == pwrdm)
			return pwrst->next_state;
	}
	return -EINVAL;
}

int omap3_pm_set_suspend_state(struct powerdomain *pwrdm, int state)
{
	struct power_state *pwrst;

	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (pwrst->pwrdm == pwrdm) {
			pwrst->next_state = state;
			return 0;
		}
	}
	return -EINVAL;
}

static int __init pwrdms_setup(struct powerdomain *pwrdm, void *unused)
{
	struct power_state *pwrst;

	if (!pwrdm->pwrsts)
		return 0;

	pwrst = kmalloc(sizeof(struct power_state), GFP_ATOMIC);
	if (!pwrst)
		return -ENOMEM;
	pwrst->pwrdm = pwrdm;
	pwrst->next_state = PWRDM_POWER_RET;
	list_add(&pwrst->node, &pwrst_list);

	if (pwrdm_has_hdwr_sar(pwrdm))
		pwrdm_enable_hdwr_sar(pwrdm);

	return set_pwrdm_state(pwrst->pwrdm, pwrst->next_state);
}

static int __init omap3_pm_early_init(void)
{
      /* set sys_off_mode active low */
       prm_clear_mod_reg_bits(OMAP3430_OFFMODE_POL_MASK, OMAP3430_GR_MOD,
                               OMAP3_PRM_POLCTRL_OFFSET);
     return 0;
}

arch_initcall(omap3_pm_early_init);

/*
 * Enable hw supervised mode for all clockdomains if it's
 * supported. Initiate sleep transition for other clockdomains, if
 * they are not used
 */
static int __init clkdms_setup(struct clockdomain *clkdm, void *unused)
{
	clkdm_clear_all_wkdeps(clkdm);
	clkdm_clear_all_sleepdeps(clkdm);

	if (clkdm->flags & CLKDM_CAN_ENABLE_AUTO)
		omap2_clkdm_allow_idle(clkdm);
	else if (clkdm->flags & CLKDM_CAN_FORCE_SLEEP &&
		 atomic_read(&clkdm->usecount) == 0)
		omap2_clkdm_sleep(clkdm);
	return 0;
}

void omap_push_sram_idle(void)
{
	_omap_sram_idle = omap_sram_push(omap34xx_cpu_suspend,
					omap34xx_cpu_suspend_sz);
	if (omap_type() != OMAP2_DEVICE_TYPE_GP)
		_omap_save_secure_sram = omap_sram_push(save_secure_ram_context,
				save_secure_ram_context_sz);
}

static int __init omap3_pm_init(void)
{
	struct power_state *pwrst, *tmp;
	struct clockdomain *neon_clkdm, *per_clkdm, *mpu_clkdm, *core_clkdm;
	struct clockdomain *wkup_clkdm;
	int ret = 0;
	u8 uninitialized_var(RdReg);

	if (!cpu_is_omap34xx())
		return -ENODEV;

	printk(KERN_ERR "Power Management for TI OMAP3.\n");

	/* XXX prcm_setup_regs needs to be before enabling hw
	 * supervised mode for powerdomains */
	prcm_setup_regs();

	ret = request_irq(INT_34XX_PRCM_MPU_IRQ,
			  (irq_handler_t)prcm_interrupt_handler,
			  IRQF_DISABLED, "prcm", NULL);
	if (ret) {
		printk(KERN_ERR "request_irq failed to register for 0x%x\n",
		       INT_34XX_PRCM_MPU_IRQ);
		goto err1;
	}

	ret = pwrdm_for_each(pwrdms_setup, NULL);
	if (ret) {
		printk(KERN_ERR "Failed to setup powerdomains\n");
		goto err2;
	}

	(void) clkdm_for_each(clkdms_setup, NULL);

	mpu_pwrdm = pwrdm_lookup("mpu_pwrdm");
	if (mpu_pwrdm == NULL) {
		printk(KERN_ERR "Failed to get mpu_pwrdm\n");
		goto err2;
	}

	neon_pwrdm = pwrdm_lookup("neon_pwrdm");
	per_pwrdm = pwrdm_lookup("per_pwrdm");
	core_pwrdm = pwrdm_lookup("core_pwrdm");
	cam_pwrdm = pwrdm_lookup("cam_pwrdm");
	wkup_pwrdm = pwrdm_lookup("wkup_pwrdm");

	neon_clkdm = clkdm_lookup("neon_clkdm");
	mpu_clkdm = clkdm_lookup("mpu_clkdm");
	per_clkdm = clkdm_lookup("per_clkdm");
	core_clkdm = clkdm_lookup("core_clkdm");

	wkup_clkdm = clkdm_lookup("wkup_pwrdm");
	omap_push_sram_idle();
#ifdef CONFIG_SUSPEND
	suspend_set_ops(&omap_pm_ops);
#endif /* CONFIG_SUSPEND */

	pm_idle = omap3_pm_idle;
	omap3_idle_init();

	clkdm_add_wkdep(neon_clkdm, mpu_clkdm);


	/*
      * Part of fix for errata i468.
        * GPIO pad spurious transition (glitch/spike) upon wakeup
        * from SYSTEM OFF mode.
      */
      if (omap_rev() <= OMAP3630_REV_ES1_2) {
              struct clockdomain *wkup_clkdm;

               clkdm_add_wkdep(per_clkdm, core_clkdm);

              wkup_clkdm = clkdm_lookup("wkup_clkdm");
               if (wkup_clkdm)
                       clkdm_add_wkdep(per_clkdm, wkup_clkdm);
               else
                       printk(KERN_ERR "%s: failed to look up wkup clock "
                              "domain\n", __func__);
      }
	  
	  
/*  DVFS   */
	ret = twl_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &RdReg,
                                       R_DCDC_GLOBAL_CFG);
    
       RdReg |= DCDC_GLOBAL_CFG_ENABLE_SRFLX;
       ret |= twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, RdReg,
                                       R_DCDC_GLOBAL_CFG);

	omap3_save_scratchpad_contents();
err1:
	return ret;
err2:
	free_irq(INT_34XX_PRCM_MPU_IRQ, NULL);
	list_for_each_entry_safe(pwrst, tmp, &pwrst_list, node) {
		list_del(&pwrst->node);
		kfree(pwrst);
	}
	return ret;
}

late_initcall(omap3_pm_init);

void __init pm_alloc_secure_ram(void)
{
	if (omap_type() != OMAP2_DEVICE_TYPE_GP) {
		omap3_secure_ram_storage =
			alloc_bootmem_low_pages(secure_copy_data.size);
		if (!omap3_secure_ram_storage)
			pr_err("Memory alloc failed for secure RAM context.\n");
	}
}
