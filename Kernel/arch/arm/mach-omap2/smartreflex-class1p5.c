/*
 * Smart reflex Class 1.5 specific implementations
 *
 * Copyright (C) 2010-2011 Texas Instruments, Inc.
 * Nishanth Menon <nm@ti.com>
 *
 * Smart reflex class 1.5 is also called periodic SW Calibration
 * Some of the highlights are as follows:
 * – Host CPU triggers OPP calibration when transitioning to non calibrated
 *   OPP
 * – SR-AVS + VP modules are used to perform calibration
 * – Once completed, the SmartReflex-AVS module can be disabled
 * – Enables savings based on process, supply DC accuracy and aging
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <plat/omap-pm.h>

//#include <mach/gpio.h>
//#include <plat/mux.h>

#include <plat/opp.h>
#include <plat/smartreflex.h>
#include <plat/voltage.h>
#include "smartreflex-class1p5.h"
#include "prm.h"

#define MAX_VDDS 3
#define SR1P5_SAMPLING_DELAY_MS	1
#define SR1P5_STABLE_SAMPLES	5
#define SR1P5_MAX_TRIGGERS	5

#define SR_DEBUG 1

/*
 * we expect events in 10uS, if we dont get 2wice times as much,
 * we could kind of ignore this as a missed event.
 */
#define MAX_CHECK_VPTRANS_US	20

/**
 * struct sr_class1p5_work_data - data meant to be used by calibration work
 * @work:	calibration work
 * @voltdm:		voltage domain for which we are triggering
 * @vdata:	voltage data we are calibrating
 * @num_calib_triggers:	number of triggers from calibration loop
 * @num_osc_samples:	number of samples collected by isr
 * @work_active:	have we scheduled a work item?
 */
struct sr_class1p5_work_data {
	struct delayed_work work;
	struct voltagedomain *voltdm;
	struct omap_volt_data *vdata;
	struct pm_qos_request_list *qos_request;
	u8 num_calib_triggers;
	u8 num_osc_samples;
	bool work_active;
};

#if CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY
/* recal_work:	recalibration calibration work */
static struct delayed_work recal_work;
#endif

/**
 * struct sr_class1p5_data - private data for class 1p5
 * @work_data:		work item data per voltage domain
 */
struct sr_class1p5_data {
	struct sr_class1p5_work_data work_data[MAX_VDDS];
};

static void sr_class1p5_reset_calib(struct voltagedomain *voltdm, bool reset,
				    bool recal);

/* our instance of class 1p5 private data */
static struct sr_class1p5_data class_1p5_data;

static struct sr_class1p5_work_data *get_sr1p5_work(struct voltagedomain
						    *voltdm)
{
	int idx;
	for (idx = 0; idx < MAX_VDDS; idx++) {
		if (class_1p5_data.work_data[idx].voltdm && !strcmp
		    (class_1p5_data.work_data[idx].voltdm->name, voltdm->name))
			return &class_1p5_data.work_data[idx];
	}
	return ERR_PTR(-ENODATA);
}

/**
 * sr_class1p5_notify() - isr notifier for status events
 * @voltdm:	voltage domain for which we were triggered
 * @status:	notifier event to use
 *
 * This basically collects data for the work to use.
 */
static int sr_class1p5_notify(struct voltagedomain *voltdm, u32 status)
{
	struct sr_class1p5_work_data *work_data;
	int idx = 0;

	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	work_data = get_sr1p5_work(voltdm);
	if (unlikely(!work_data)) {
		pr_err("%s:%s no work data!!\n", __func__, voltdm->name);
		return -EINVAL;
	}

	/* Wait for transdone so that we know the voltage to read */
	do {
		if (omap_vp_is_transdone(voltdm))
			break;
		idx++;
		/* get some constant delay */
		udelay(1);
	} while (idx < MAX_CHECK_VPTRANS_US);

	/*
	 * If we timeout, we still read the data,
	 * if we are oscillating+irq latencies are too high, we could
	 * have scenarios where we miss transdone event. since
	 * we waited long enough, it is still safe to read the voltage
	 * as we would have waited long enough - still flag it..
	 */
	if (idx >= MAX_CHECK_VPTRANS_US)
		pr_warning("%s: timed out waiting for transdone!!\n", __func__);

	omap_vp_clear_transdone(voltdm);

	idx = (work_data->num_osc_samples) % SR1P5_STABLE_SAMPLES;
	work_data->num_osc_samples++;

	return 0;
}

#define SR_CLASS1P5_LOOP_US	100
#define MAX_STABILIZATION_COUNT 100
#define MAX_LOOP_COUNT		(MAX_STABILIZATION_COUNT * 20)

static inline void sr_udelay(u32 delay)
{
	while (delay-- > 0) {
		cpu_relax();
		udelay(1);
	};

}

static void  sr_recalibrate(struct sr_class1p5_work_data *work_data)
{

	unsigned long u_volt_current = 0;
	struct omap_volt_data *volt_data;
	struct voltagedomain *voltdm;
	
	u32 max_loop_count = MAX_LOOP_COUNT;
	u32 exit_loop_on = 0;
	u8 new_v = 0;
	u8 high_v = 0;
		
	/* Start Smart reflex */

	if (unlikely(!work_data)) {
		pr_err("sr_recalibrate %s: ooops.. null work_data?\n", __func__);
		return;
	}

	voltdm = work_data->voltdm;

	if (unlikely(!work_data->work_active)) {
#if SR_DEBUG	
		printk("sr_recalibrate %s:%s unplanned work invocation!\n", __func__,
		       voltdm->name);
#endif			   
		return;
	}
       
	/* Clear pending events */
	sr_notifier_control(voltdm, false);
	/* Clear all transdones */
	while (omap_vp_is_transdone(voltdm))
		{
		omap_vp_clear_transdone(voltdm);
		}
	/* trigger sampling */
	sr_notifier_control(voltdm, true);
	/* We need to wait for SR to stabilize before we start sampling */
	sr_udelay(MAX_STABILIZATION_COUNT * SR_CLASS1P5_LOOP_US);

	/* Ready for recalibration */
	while (max_loop_count) {
		if(strcmp(voltdm->name,"mpu")==0)
			new_v = prm_read_mod_reg(OMAP3430_GR_MOD,
				OMAP3_PRM_VP1_VOLTAGE_OFFSET);
		if(strcmp(voltdm->name,"core")==0)
			new_v = prm_read_mod_reg(OMAP3430_GR_MOD,
				OMAP3_PRM_VP2_VOLTAGE_OFFSET);

		/* handle oscillations */
		if (new_v != high_v) {
			high_v = (high_v < new_v) ? new_v : high_v;
			exit_loop_on = MAX_STABILIZATION_COUNT;
		}
		/* wait for one more stabilization loop for us to sample */
		sr_udelay(SR_CLASS1P5_LOOP_US);

		max_loop_count--;
		exit_loop_on--;
		/* Stabilization achieved.. quit */
		if (!exit_loop_on)
			break;
	}
	/*
	 * bad case where we are oscillating.. flag it,
	 * but continue with higher v
	 */
	if (!max_loop_count && exit_loop_on) {
		pr_warning("%s: %s: for nominal voltage %d  exited with voltages 0x%02x 0x%02x\n",
			__func__, voltdm->name, work_data->vdata->volt_nominal, new_v, high_v);
	}
	/* Stop Smart reflex */
	sr_notifier_control(voltdm, false);
	omap_vp_disable(voltdm);
       sr_disable(voltdm);

	volt_data = work_data->vdata;
	volt_data->volt_calibrated = (((new_v * 125) + 6000)) * 100 ;


if (cpu_is_omap3630()) {
    		volt_data->volt_calibrated += volt_data->sr_oppmargin;
		if (volt_data->volt_calibrated > volt_data->volt_nominal) {
			volt_data->volt_calibrated = volt_data->volt_nominal;
		}
	}

	u_volt_current = omap_vp_get_curr_volt(voltdm);
#if SR_DEBUG	
		printk("current voltage for %s before calibration is %ld adding margin %d ,vnom being %d \n",
		       voltdm->name,u_volt_current,volt_data->sr_oppmargin,volt_data->volt_nominal);
#endif		

	if (volt_data->volt_calibrated != u_volt_current) {
#if SR_DEBUG	
                     printk("sr_recalibrate %s:%s reconfiguring to voltage %d\n", __func__, voltdm->name, volt_data->volt_calibrated);	
#else	
			printk("%s:%s reconfiguring to voltage %d\n",
			 __func__, voltdm->name, volt_data->volt_calibrated);
#endif			 
	              omap_voltage_scale_vdd(voltdm, volt_data);
		
	

}
#if SR_DEBUG
                    u_volt_current = omap_vp_get_curr_volt(voltdm);
		      printk("sr_recalibrate Setting  calib volt %ld , calibrated voltage was  %d for %s done  \n ",u_volt_current,volt_data->volt_calibrated,voltdm->name);
#endif

	work_data->work_active = false;
	return;
}


/**
 * do_calibrate() - work which actually does the calibration
 * @work: pointer to the work
 *
 * calibration routine uses the following logic:
 * on the first trigger, we start the isr to collect sr voltages
 * wait for stabilization delay (reschdule self instead of sleeping)
 * after the delay, see if we collected any isr events
 * if none, we have calibrated voltage.
 * if there are any, we retry untill we giveup.
 * on retry timeout, select a voltage to use as safe voltage.
 */
static void do_calibrate(struct work_struct *work)
{
	struct sr_class1p5_work_data *work_data =
	    container_of(work, struct sr_class1p5_work_data, work.work);
	unsigned long u_volt_safe = 0, u_volt_current = 0;
	struct omap_volt_data *volt_data;
	struct voltagedomain *voltdm;
/*
	if(!gpio_get_value( OMAP_GPIO_TA_NCONNECTED ))
	{	
		printk("SR_WORKAROUND - return by ta or usb\n");
		return;
	}
*/
	if (unlikely(!work_data)) {
		pr_err("SR_DEBUG %s: ooops.. null work_data?\n", __func__);
		return;
	}

	/*
	 * Handle the case where we might have just been scheduled AND
	 * 1.5 disable was called.
	 */
	if (omap_vscale_pause(work_data->voltdm, true)) {
#if SR_DEBUG
		printk("SR_DEBUG do_calibrate handle race condition  %d\n",work_data->vdata->volt_nominal);
#endif		
		schedule_delayed_work(&work_data->work,
				      msecs_to_jiffies(SR1P5_SAMPLING_DELAY_MS *
						       SR1P5_STABLE_SAMPLES));
		return;
	}

	voltdm = work_data->voltdm;
	/*
	 * In the unlikely case that we did get through when unplanned,
	 * flag and return.
	 */
	if (unlikely(!work_data->work_active)) {
#if SR_DEBUG	
		printk("SR_DEBUG %s:%s unplanned work invocation!\n", __func__,
		       voltdm->name);
#endif			   
		omap_vscale_unpause(work_data->voltdm);
		return;
	}

	work_data->num_calib_triggers++;
	/* if we are triggered first time, we need to start isr to sample */
	if (work_data->num_calib_triggers == 1)
		{
		printk("SR_DEBUG start sampling  %d \n",work_data->vdata->volt_nominal);
		goto start_sampling;
		}

	/* Stop isr from interrupting our measurements :) */
	sr_notifier_control(voltdm, false);

	volt_data = work_data->vdata;

	/* if there are no samples captured.. SR is silent, aka stability! */
	if (!work_data->num_osc_samples) {
		u_volt_safe = omap_vp_get_curr_volt(voltdm);
#if SR_DEBUG
		printk("SR_DEBUG done_calib %d  volt curr %ld volt safe %ld \n",work_data->vdata->volt_nominal,u_volt_current,u_volt_safe);
#endif		
		u_volt_current = u_volt_safe;
		goto done_calib;
	}
	if (work_data->num_calib_triggers == SR1P5_MAX_TRIGGERS) {
		pr_warning("SR_DEBUG %s: %s recalib timeout!\n", __func__,
			   work_data->voltdm->name);
		goto oscillating_calib;
	}

	/* we have potential oscillations/first sample */
start_sampling:
	work_data->num_osc_samples = 0;
	/* Clear pending events */
	sr_notifier_control(voltdm, false);
	/* Clear all transdones */
	while (omap_vp_is_transdone(voltdm))
		{
#if SR_DEBUG		
		printk("SR_DEBUG Clear all transdone %d \n",work_data->vdata->volt_nominal);
#endif		
		omap_vp_clear_transdone(voltdm);
		}
	/* trigger sampling */
	sr_notifier_control(voltdm, true);
#if SR_DEBUG
	printk("SR_DEBUG  schedule after isr trigger  volt %d \n",work_data->vdata->volt_nominal);
#endif	
	schedule_delayed_work(&work_data->work,
			      msecs_to_jiffies(SR1P5_SAMPLING_DELAY_MS *
					       SR1P5_STABLE_SAMPLES));
	omap_vscale_unpause(work_data->voltdm);
	return;

oscillating_calib:
	/* Use the nominal voltage as the safe voltage */
	u_volt_safe = volt_data->volt_nominal;
	/* pick up current voltage to switch if needed */
	u_volt_current = omap_vp_get_curr_volt(voltdm);

	/* Fall through to close up common stuff */
done_calib:
	omap_vp_disable(voltdm);
	sr_disable(voltdm);

	volt_data->volt_calibrated = u_volt_safe;
	/* Setup my dynamic voltage for the next calibration for this opp */
	volt_data->volt_dynamic_nominal = omap_get_dyn_nominal(volt_data);

	/*
	 * if the voltage we decided as safe is not the current voltage,
	 * switch
	 */

	if (cpu_is_omap3630()) {
#if SR_DEBUG	
		printk("SR_DEBUG %s:%s - volt nominal %d sr opp margin %d\n", __func__, voltdm->name,volt_data->volt_nominal, volt_data->sr_oppmargin);
#endif
		volt_data->volt_calibrated += volt_data->sr_oppmargin;
		if (volt_data->volt_calibrated > volt_data->volt_nominal) {
			volt_data->volt_calibrated = volt_data->volt_nominal;
		}
	}

	if (volt_data->volt_calibrated != u_volt_current) {
#if SR_DEBUG	
printk("SR_DEBUG %s:%s reconfiguring to voltage %d\n", __func__, voltdm->name, volt_data->volt_calibrated);	
#else	
			printk("%s:%s reconfiguring to voltage %d\n",
			 __func__, voltdm->name, volt_data->volt_calibrated);
#endif			 
		omap_voltage_scale_vdd(voltdm, volt_data);
#if SR_DEBUG
		printk("SR_DEBUG Setting  calib volt %d for %s done  \n ",volt_data->volt_calibrated,voltdm->name);
#endif		
	}

	/*
	 * TODO: Setup my wakeup voltage to allow immediate going to OFF and
	 * on - Pending twl and voltage layer cleanups.
	 * This is necessary, as this is not done as part of regular
	 * Dvfs flow.
	 * vc_setup_on_voltage(voltdm, volt_data->volt_calibrated);
	 */
	work_data->work_active = false;
	omap_vscale_unpause(work_data->voltdm);

	/* Release c-state constraint */
    //omap_pm_set_max_mpu_wakeup_lat(&work_data->qos_request, -1);
    //pr_debug("%s - %s: release c-state\n", __func__, voltdm->name);
}

#if CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY
/**
 * do_recalibrate() - work which actually does the calibration
 * @work: pointer to the work
 *
 * on a periodic basis, we come and reset our calibration setup
 * so that a recalibration of the OPPs take place. This takes
 * care of aging factor in the system.
 */
static void do_recalibrate(struct work_struct *work)
{
	struct voltagedomain *voltdm;
	int idx;
	static struct sr_class1p5_work_data *work_data;

	for (idx = 0; idx < MAX_VDDS; idx++) {
		work_data = &class_1p5_data.work_data[idx];
		voltdm = work_data->voltdm;
		if (voltdm) {
			/* if sr is not enabled, we check later */
			if (!is_sr_enabled(voltdm))
				continue;

			/* if we can't pause it, check later */
			if (omap_vscale_pause(voltdm, false))
				continue;

			/* Reset and force a recalibration for current opp */
			sr_class1p5_reset_calib(voltdm, true, true);

			omap_vscale_unpause(voltdm);
		}
	}
	/* We come back again after time the usual delay */
	schedule_delayed_work(&recal_work,
	      msecs_to_jiffies(CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY));
}
#endif /* CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY */

/**
 * sr_class1p5_enable() - class 1.5 mode of enable
 * @voltdm:		voltage domain to enable SR for
 * @volt_data:	voltdata to the voltage transition taking place
 *
 * when this gets called, we use the h/w loop to setup our voltages
 * to an calibrated voltage, detect any oscillations, recover from the same
 * and finally store the optimized voltage as the calibrated voltage in the
 * system
 */
static int sr_class1p5_enable(struct voltagedomain *voltdm,
			      struct omap_volt_data *volt_data)
{
	int r;
	struct sr_class1p5_work_data *work_data;

	if (IS_ERR_OR_NULL(voltdm) || IS_ERR_OR_NULL(volt_data)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	/* if already calibrated, nothing to do here.. */
	if (volt_data->volt_calibrated)
		return 0;

	work_data = get_sr1p5_work(voltdm);
	if (unlikely(!work_data)) {
		pr_err("%s: aieeee.. bad work data??\n", __func__);
		return -EINVAL;
	}

	if (work_data->work_active)
		return 0;

	omap_vp_enable(voltdm);
	r = sr_enable(voltdm, volt_data);
	if (r) {
		pr_err("%s: sr[%s] failed\n", __func__, voltdm->name);
		omap_vp_disable(voltdm);
		return r;
	}
	work_data->vdata = volt_data;
	work_data->work_active = true;
	work_data->num_calib_triggers = 0;

	/* Hold a c-state constraint */
    //omap_pm_set_max_mpu_wakeup_lat(&work_data->qos_request, 1000);
    //pr_debug("%s - %s: hold c-state\n", __func__, voltdm->name);
	/* program the workqueue and leave it to calibrate offline.. */

#ifdef SR_WORKQUEUE_METHOD	
	schedule_delayed_work(&work_data->work,
			      msecs_to_jiffies(SR1P5_SAMPLING_DELAY_MS *
					       SR1P5_STABLE_SAMPLES));
#else
       sr_recalibrate(work_data);  // boot time calibration
#endif

	return 0;
}

/**
 * sr_class1p5_disable() - disable for class 1p5
 * @voltdm: voltage domain for the sr which needs disabling
 * @volt_data:	voltagedata to disable
 * @is_volt_reset: reset the voltage?
 *
 * we dont do anything if the class 1p5 is being used. this is because we
 * already disable sr at the end of calibration and no h/w loop is actually
 * active when this is called.
 */
static int sr_class1p5_disable(struct voltagedomain *voltdm,
			       struct omap_volt_data *volt_data,
			       int is_volt_reset)
{
	struct sr_class1p5_work_data *work_data;

	if (IS_ERR_OR_NULL(voltdm) || IS_ERR_OR_NULL(volt_data)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	work_data = get_sr1p5_work(voltdm);
	if (work_data->work_active) {
		/* if volt reset and work is active, we dont allow this */
		if (is_volt_reset)
			return -EBUSY;
		/* flag work is dead and remove the old work */
		work_data->work_active = false;
		cancel_delayed_work_sync(&work_data->work);
		sr_notifier_control(voltdm, false);
		omap_vp_disable(voltdm);
		sr_disable(voltdm);

		 /* Release c-state constraint */
        //omap_pm_set_max_mpu_wakeup_lat(&work_data->qos_request, -1);
        //pr_debug("%s - %s: release c-state\n", __func__, voltdm->name);
	}

	/* if already calibrated, nothin special to do here.. */
	if (volt_data->volt_calibrated)
		return 0;

	if (is_volt_reset)
		omap_voltage_reset(voltdm);
	return 0;
}

/**
 * sr_class1p5_configure() - configuration function
 * @voltdm:	configure for which voltage domain
 *
 * we dont do much here other than setup some registers for
 * the sr module involved.
 */
static int sr_class1p5_configure(struct voltagedomain *voltdm)
{
	if (IS_ERR_OR_NULL(voltdm)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	return sr_configure_errgen(voltdm);
}

/**
 * sr_class1p5_reset_calib() - reset all calibrated voltages
 * @voltdm:	configure for which voltage domain
 * @reset:	reset voltage before we recal?
 * @recal:	should I recalibrate my current opp?
 *
 * if we call this, it means either periodic calibration trigger was
 * fired(either from sysfs or other mechanisms) or we have disabled class 1p5,
 * meaning we cant trust the calib voltages anymore, it is better to use
 * nominal in the system
 */
static void sr_class1p5_reset_calib(struct voltagedomain *voltdm, bool reset,
				    bool recal)
{
	struct sr_class1p5_work_data *work_data;

	/* I dont need to go further if sr is not present */
	if (!is_sr_enabled(voltdm))
		return;

	work_data = get_sr1p5_work(voltdm);

	if (work_data->work_active)
		sr_class1p5_disable(voltdm, work_data->vdata, 0);

	omap_voltage_calib_reset(voltdm);

	/*
	 * I should now reset the voltages to my nominal to be safe
	 */
	if (reset)
		omap_voltage_reset(voltdm);

	/*
	 * I should fire a recalibration for current opp if needed
	 * Note: i have just reset my calibrated voltages, and if
	 * i call sr_enable equivalent, I will cause a recalibration
	 * loop, even though the function is called sr_enable.. we
	 * are in class 1.5 ;)
	 */
	if (reset && recal)
		sr_class1p5_enable(voltdm, work_data->vdata);
}

/**
 * sr_class1p5_start() - class 1p5 init
 * @voltdm:		sr voltage domain
 * @class_priv_data:	private data for the class
 *
 * we do class specific initialization like creating sysfs/debugfs entries
 * needed, spawning of a kthread if needed etc.
 */
static int sr_class1p5_start(struct voltagedomain *voltdm,
			     void *class_priv_data)
{
	struct sr_class1p5_work_data *work_data;
	int idx;

	if (IS_ERR_OR_NULL(voltdm) || IS_ERR_OR_NULL(class_priv_data)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	/* setup our work params */
	work_data = get_sr1p5_work(voltdm);
	if (!IS_ERR_OR_NULL(work_data)) {
		pr_err("%s: ooopps.. class already initialized for %s! bug??\n",
		       __func__, voltdm->name);
		return -EINVAL;
	}
	work_data = NULL;
	/* get the next spare work_data */
	for (idx = 0; idx < MAX_VDDS; idx++) {
		if (!class_1p5_data.work_data[idx].voltdm) {
			work_data = &class_1p5_data.work_data[idx];
			break;
		}
	}
	if (!work_data) {
		pr_err("%s: no more space for work data for domains!\n",
			__func__);
		return -ENOMEM;
	}
	work_data->voltdm = voltdm;
#ifdef SR_WORKQUEUE_METHOD		
	INIT_DELAYED_WORK_DEFERRABLE(&work_data->work, do_calibrate);
#endif	
	return 0;
}

/**
 * sr_class1p5_stop() - class 1p5 deinitialization
 * @voltdm:	voltage domain for which to do this.
 * @class_priv_data: class private data for deinitialiation
 *
 * currently only resets the calibrated voltage forcing DVFS voltages
 * to be used in the system
 */
static int sr_class1p5_stop(struct voltagedomain *voltdm,
			       void *class_priv_data)
{
	struct sr_class1p5_work_data *work_data;

	if (IS_ERR_OR_NULL(voltdm) || IS_ERR_OR_NULL(class_priv_data)) {
		pr_err("%s: bad parameters!\n", __func__);
		return -EINVAL;
	}

	/* setup our work params */
	work_data = get_sr1p5_work(voltdm);
	if (IS_ERR_OR_NULL(work_data)) {
		pr_err("%s: ooopps.. class not initialized for %s! bug??\n",
		       __func__, voltdm->name);
		return -EINVAL;
	}

	/*
	 * we dont have SR periodic calib anymore.. so reset calibs
	 * we are already protected by sr debugfs lock, so no lock needed
	 * here.
	 */
	sr_class1p5_reset_calib(voltdm, true, false);

	/* reset all data for this work data */
	memset(work_data, 0, sizeof(*work_data));

	return 0;
}

/* SR class1p5 structure */
static struct omap_smartreflex_class_data class1p5_data = {
	.enable = sr_class1p5_enable,
	.disable = sr_class1p5_disable,
	.configure = sr_class1p5_configure,
	.class_type = SR_CLASS1P5,
	.start = sr_class1p5_start,
	.stop = sr_class1p5_stop,
	.notify = sr_class1p5_notify,
	/*
	 * trigger for bound - this tells VP that SR has a voltage
	 * change. we should ensure transdone is set before reading
	 * vp voltage.
	 */
	.notify_flags = SR_NOTIFY_MCUBOUND,
	.class_priv_data = (void *)&class_1p5_data,
};

/**
 * sr_class1p5_init() - register class 1p5 as default
 *
 * board files call this function to use class 1p5, we register with the
 * smartreflex subsystem
 */
int __init sr_class1p5_init(void)
{
	int r;

	/* Enable this class only for OMAP3630 and OMAP4 */
	if (!(cpu_is_omap3630() || cpu_is_omap44xx()))
		return -EINVAL;

	r = omap_sr_register_class(&class1p5_data);
	if (r) {
		pr_err("SmartReflex class 1.5 driver: "
		       "failed to register with %d\n", r);
	} else {
#if CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY
		INIT_DELAYED_WORK_DEFERRABLE(&recal_work, do_recalibrate);
		schedule_delayed_work(&recal_work, msecs_to_jiffies(
				CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY));
#endif
		pr_info("SmartReflex class 1.5 driver: initialized (%dms)\n",
			CONFIG_OMAP_SR_CLASS1P5_RECALIBRATION_DELAY);
	}
	return r;
}
