/*
 *  linux/arch/arm/plat-omap/cpu-omap.c
 *
 *  CPU frequency scaling for OMAP
 *
 *  Copyright (C) 2005 Nokia Corporation
 *  Written by Tony Lindgren <tony@atomide.com>
 *
 *  Based on cpu-sa1110.c, Copyright (C) 2001 Russell King
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Updated to support OMAP3
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/cpu.h>

#include <mach/hardware.h>
#include <plat/clock.h>
#include <asm/system.h>
#include <asm/cpu.h>
#include <plat/omap_device.h>

#if defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4)
#include <plat/omap-pm.h>
#include <plat/opp.h>
#endif

#define VERY_HI_RATE	900000000

static struct cpufreq_frequency_table *freq_table;

#ifdef CONFIG_ARCH_OMAP1
#define MPU_CLK		"mpu"
#elif defined(CONFIG_ARCH_OMAP3)
#define MPU_CLK		"arm_fck"
#else
#define MPU_CLK		"virt_prcm_set"
#endif

static struct clk *mpu_clk;

#ifdef CONFIG_SMP
static cpumask_var_t omap4_cpumask;
static int cpus_initialized;
#endif

#ifdef CONFIG_SAMSUNG_LATONA_OVERCLOCK_ENABLED
static ssize_t overclock_show(struct kobject *, struct kobj_attribute *,
              char *);
static ssize_t overclock_store(struct kobject *k, struct kobj_attribute *,
			  const char *buf, size_t n);


static struct kobj_attribute overclock_opp1_attr =
    __ATTR(overclock_opp1, 0644, overclock_show, overclock_store);
static struct kobj_attribute overclock_opp2_attr =
    __ATTR(overclock_opp2, 0644, overclock_show, overclock_store);
static struct kobj_attribute overclock_opp3_attr =
    __ATTR(overclock_opp3, 0644, overclock_show, overclock_store);
static struct kobj_attribute overclock_opp4_attr =
    __ATTR(overclock_opp4, 0644, overclock_show, overclock_store);
static struct kobj_attribute overclock_opp5_attr =
    __ATTR(overclock_opp5, 0644, overclock_show, overclock_store);
#endif


/* TODO: Add support for SDRAM timing changes */

static int omap_verify_speed(struct cpufreq_policy *policy)
{
	if (freq_table)
		return cpufreq_frequency_table_verify(policy, freq_table);

	if (policy->cpu)
		return -EINVAL;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);

	policy->min = clk_round_rate(mpu_clk, policy->min * 1000) / 1000;
	policy->max = clk_round_rate(mpu_clk, policy->max * 1000) / 1000;
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int omap_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= num_online_cpus())
		return 0;

	rate = clk_get_rate(mpu_clk) / 1000;
	return rate;
}

static int omap_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
#if defined(CONFIG_ARCH_OMAP1) || defined(CONFIG_ARCH_OMAP4)
	struct cpufreq_freqs freqs;
#endif
#if defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4)
#ifdef CONFIG_SMP
	int i;
	struct cpufreq_freqs freqs_notify;
#endif
	unsigned long freq;
	struct device *mpu_dev = omap2_get_mpuss_device();
	int ret = 0;

	if (omap_get_vdd1_lock())
		return ret;
#endif

#ifdef CONFIG_SMP
	/* Wait untill all CPU's are initialized */
	if (unlikely(cpus_initialized < num_online_cpus()))
		return ret;
#endif

	/* Ensure desired rate is within allowed range.  Some govenors
	 * (ondemand) will just pass target_freq=0 to get the minimum. */
	if (target_freq < policy->min)
		target_freq = policy->min;
	if (target_freq > policy->max)
		target_freq = policy->max;

#ifdef CONFIG_ARCH_OMAP1
	freqs.old = omap_getspeed(0);
	freqs.new = clk_round_rate(mpu_clk, target_freq * 1000) / 1000;
	freqs.cpu = 0;

	if (freqs.old == freqs.new)
		return ret;
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
#ifdef CONFIG_CPU_FREQ_DEBUG
	printk(KERN_DEBUG "cpufreq-omap: transition: %u --> %u\n",
	       freqs.old, freqs.new);
#endif
	ret = clk_set_rate(mpu_clk, freqs.new * 1000);
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
#elif defined(CONFIG_ARCH_OMAP3) || defined(CONFIG_ARCH_OMAP4)
#ifdef CONFIG_SMP
	freqs.old = omap_getspeed(policy->cpu);;
	freqs_notify.new = clk_round_rate(mpu_clk, target_freq * 1000) / 1000;
	freqs.cpu = policy->cpu;

	if (freqs.old == freqs.new)
		return ret;

	/* notifiers */
	for_each_cpu(i, policy->cpus) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	}
#endif

	freq = target_freq * 1000;
	if (opp_find_freq_ceil(mpu_dev, &freq))
		omap_device_set_rate(mpu_dev, mpu_dev, freq);
#ifdef CONFIG_SMP
	/*
	 * Note that loops_per_jiffy is not updated on SMP systems in
	 * cpufreq driver. So, update the per-CPU loops_per_jiffy value
	 * on frequency transition. We need to update all dependent cpus
	 */
	freqs.new = omap_getspeed(policy->cpu);
	for_each_cpu(i, policy->cpus)
		per_cpu(cpu_data, i).loops_per_jiffy =
		cpufreq_scale(per_cpu(cpu_data, i).loops_per_jiffy,
				freqs.old, freqs.new);
#endif

#ifdef CONFIG_SMP
		/* notifiers */
	for_each_cpu(i, policy->cpus) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}
#endif
#endif

#ifdef CONFIG_BRIDGE_DVFS
	/*
	 * Inform the PM architecture of this change.  Currently the only
	 * need for this is when using DSP bridge DVFS -- to coordinate with
	 * frequency requests made by the DSP.
	 */
	omap_pm_cpu_set_freq(freq);
#endif
	return ret;
}

static int omap_cpu_init(struct cpufreq_policy *policy)
{
	int result = 0;
    	int error = -EINVAL;

	if (cpu_is_omap44xx())
		mpu_clk = clk_get(NULL, "dpll_mpu_ck");
	else
		mpu_clk = clk_get(NULL, MPU_CLK);

	if (IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	if (policy->cpu >= num_online_cpus())
		return -EINVAL;

	policy->cur = policy->min = policy->max = omap_getspeed(policy->cpu);

	if (!(cpu_is_omap34xx() || cpu_is_omap44xx())) {
		clk_init_cpufreq_table(&freq_table);
	} else {
		struct device *mpu_dev = omap2_get_mpuss_device();

		opp_init_cpufreq_table(mpu_dev, &freq_table);
	}

	if (freq_table) {
		result = cpufreq_frequency_table_cpuinfo(policy, freq_table);
		if (!result)
			cpufreq_frequency_table_get_attr(freq_table,
							policy->cpu);
	} else {
		policy->cpuinfo.min_freq = clk_round_rate(mpu_clk, 0) / 1000;
		policy->cpuinfo.max_freq = clk_round_rate(mpu_clk,
							VERY_HI_RATE) / 1000;
	}

	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;
	policy->cur = omap_getspeed(policy->cpu);

	/* FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 300 * 1000;

#ifdef CONFIG_SMP
	/*
	 * On OMAP4i, both processors share the same voltage and
	 * the same clock, but have dedicated power domains. So both
	 * cores needs to be scaled together and hence needs software
	 * co-ordination. Use cpufreq affected_cpus interface to handle
	 * this scenario.
	 */
	policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
	cpumask_or(omap4_cpumask, cpumask_of(policy->cpu), omap4_cpumask);
	cpumask_copy(policy->cpus, omap4_cpumask);
	cpus_initialized++;
#endif

	#ifdef CONFIG_SAMSUNG_LATONA_OVERCLOCK_ENABLED
	error = sysfs_create_file(power_kobj, &overclock_opp1_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(power_kobj, &overclock_opp2_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(power_kobj, &overclock_opp3_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
	error = sysfs_create_file(power_kobj, &overclock_opp4_attr.attr);
	if (error) {
		printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
		return error;
	}
		#ifdef CONFIG_SAMSUNG_LATONA_OPP5_ENABLED
		error = sysfs_create_file(power_kobj, &overclock_opp5_attr.attr);
		if (error) {
			printk(KERN_ERR "sysfs_create_file failed: %d\n", error);
			return error;
		}
		#endif
	#endif

	return 0;
}

static int omap_cpu_exit(struct cpufreq_policy *policy)
{
	if (!(cpu_is_omap34xx() || cpu_is_omap44xx()))
		clk_exit_cpufreq_table(&freq_table);
	else
		opp_exit_cpufreq_table(&freq_table);

	clk_put(mpu_clk);
	return 0;
}

static struct freq_attr *omap_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver omap_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= omap_verify_speed,
	.target		= omap_target,
	.get		= omap_getspeed,
	.init		= omap_cpu_init,
	.exit		= omap_cpu_exit,
	.name		= "omap",
	.attr		= omap_cpufreq_attr,
};

static int __init omap_cpufreq_init(void)
{
	return cpufreq_register_driver(&omap_driver);
}

#ifdef CONFIG_SAMSUNG_LATONA_OVERCLOCK_ENABLED
static ssize_t overclock_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
	unsigned int freq;
	unsigned int target_opp_nr;
	unsigned int counter;
	struct device *mpu_dev;

	mpu_dev = omap2_get_mpuss_device();
	if(IS_ERR(mpu_dev))
		return -EINVAL;

	if ( attr == &overclock_opp1_attr) {
		target_opp_nr = 0;
	}
	if ( attr == &overclock_opp2_attr) {
		target_opp_nr = 1;
	}
	if ( attr == &overclock_opp3_attr) {
		target_opp_nr = 2;
	}
	if ( attr == &overclock_opp4_attr) {
		target_opp_nr = 3;
	}
		#ifdef CONFIG_SAMSUNG_LATONA_OPP5_ENABLED
		if ( attr == &overclock_opp5_attr) {
			target_opp_nr = 4;
		}
		#endif

	//Find opp (1 MHZ steps)
	counter = 0;
	for (freq = 0; counter < (target_opp_nr+1); freq += (1000*1000)) {
		if(!IS_ERR(opp_find_freq_exact(mpu_dev, freq, true)))
			counter++;
		//Show frequency even if OPP is disabled
		if(!IS_ERR(opp_find_freq_exact(mpu_dev, freq, false)))
			counter++;
	}

	if(freq == 0)
		return -EINVAL;

	freq = freq - (1000*1000);

	return sprintf(buf, "%lu\n", freq / (1000*1000));
}

static ssize_t overclock_store(struct kobject *k,
        struct kobj_attribute *attr, const char *buf, size_t n)
{
	unsigned int freq;
	unsigned int help_freq;
	unsigned int target_opp_nr;
	unsigned int opp_lower_limit = 0;
	unsigned int opp_upper_limit = 0;
	unsigned int counter;
	struct device *mpu_dev = omap2_get_mpuss_device();
	struct omap_opp *temp_opp;
	struct cpufreq_policy *mpu_policy = cpufreq_cpu_get(0);
	struct cpufreq_frequency_table *mpu_freq_table = *omap_pm_cpu_get_freq_table();

	if(IS_ERR(mpu_dev) || IS_ERR(mpu_policy) || IS_ERR(mpu_freq_table))
		return -EINVAL;


	// Hard coded clock limits
	if ( attr == &overclock_opp1_attr) {
		target_opp_nr = 0;
		opp_lower_limit = 100;
		opp_upper_limit = 500;
	}
	if ( attr == &overclock_opp2_attr) {
		target_opp_nr = 1;
		opp_lower_limit = 501;
		opp_upper_limit = 700;
	}
	if ( attr == &overclock_opp3_attr) {
		target_opp_nr = 2;
		opp_lower_limit = 701;
		opp_upper_limit = 900;
	}
	if ( attr == &overclock_opp4_attr) {
		target_opp_nr = 3;
		opp_lower_limit = 901;
		opp_upper_limit = 1100;
	}
	#ifdef CONFIG_SAMSUNG_LATONA_OPP5_ENABLED
	if ( attr == &overclock_opp5_attr) {
		target_opp_nr = 4;
		opp_lower_limit = 1101;
		opp_upper_limit = 1500;
	}
	#endif

	//Find opp (1 MHZ steps)
	counter = 0;
	for (help_freq = 0; counter < (target_opp_nr+1); help_freq += (1000*1000)) {
		if(!IS_ERR(opp_find_freq_exact(mpu_dev, help_freq, true)))
			counter++;
	}
	if(help_freq == 0)
		return -EINVAL;
	help_freq = help_freq - (1000*1000);

	temp_opp = opp_find_freq_exact(mpu_dev, help_freq, true);
	if(IS_ERR(temp_opp))
		return -EINVAL;

	if (sscanf(buf, "%u", &freq) == 1) {
		//Enforce clock limits
		if (freq >= opp_lower_limit && freq <= opp_upper_limit) {
			//Convert Megahertz to Hertz
			freq *= (1000*1000);

			//Set new clocks
			opp_disable(temp_opp);
			temp_opp->rate = freq;
			mpu_freq_table[target_opp_nr].frequency = freq/1000;

			//Fix policy
			if(target_opp_nr == 0) {
				mpu_policy->cpuinfo.min_freq = freq/1000;
				mpu_policy->min = freq/1000;
				mpu_policy->user_policy.min = freq/1000;
			} 
			#ifdef CONFIG_SAMSUNG_LATONA_OPP5_ENABLED
			else if(target_opp_nr == 4) {
					mpu_policy->cpuinfo.max_freq = freq/1000;
					mpu_policy->max = freq/1000;
					mpu_policy->user_policy.max = freq/1000;
			}
			#else
			else if(target_opp_nr == 3) {
					mpu_policy->cpuinfo.max_freq = freq/1000;
					mpu_policy->max = freq/1000;
					mpu_policy->user_policy.max = freq/1000;
			}
			#endif

			opp_enable(temp_opp);

			//Fix freq_table
			opp_exit_cpufreq_table(&freq_table);
			freq_table = mpu_freq_table;
			opp_init_cpufreq_table(mpu_dev, &freq_table);

			//Fix stats
			cpufreq_stats_update_freq_table(freq_table, 0);
		} else
		return -EINVAL;
	} else
	return -EINVAL;
	return n;
}
#endif

late_initcall(omap_cpufreq_init);

/*
 * if ever we want to remove this, upon cleanup call:
 *
 * cpufreq_unregister_driver()
 * cpufreq_frequency_table_put_attr()
 */

