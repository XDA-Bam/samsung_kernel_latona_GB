/*
 * pm.c - Common OMAP2+ power management-related code
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Copyright (C) 2010 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>

#include <plat/omap-pm.h>
#include <plat/omap_device.h>
#include <plat/common.h>

#include "omap3-opp.h"
#include "opp44xx.h"

static struct omap_device_pm_latency *pm_lats;

static struct device *mpu_dev;
static struct device *iva_dev;
static struct device *l3_dev;
static struct device *dsp_dev;

struct device *omap2_get_mpuss_device(void)
{
	WARN_ON_ONCE(!mpu_dev);
	return mpu_dev;
}
EXPORT_SYMBOL(omap2_get_mpuss_device);

struct device *omap2_get_iva_device(void)
{
	WARN_ON_ONCE(!iva_dev);
	return iva_dev;
}
EXPORT_SYMBOL(omap2_get_iva_device);

struct device *omap2_get_l3_device(void)
{
	WARN_ON_ONCE(!l3_dev);
	return l3_dev;
}
EXPORT_SYMBOL(omap2_get_l3_device);

struct device *omap4_get_dsp_device(void)
{
	WARN_ON_ONCE(!dsp_dev);
	return dsp_dev;
}
EXPORT_SYMBOL(omap4_get_dsp_device);

#ifdef CONFIG_OMAP_PM
static ssize_t vdd_opp_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t vdd_opp_store(struct kobject *k, struct kobj_attribute *,
			  const char *buf, size_t n);

static struct kobj_attribute vdd1_opp_attr =
	__ATTR(vdd1_opp, 0444, vdd_opp_show, vdd_opp_store);
static struct kobj_attribute vdd2_opp_attr =
	__ATTR(vdd2_opp, 0444, vdd_opp_show, vdd_opp_store);
static struct kobj_attribute vdd1_lock_attr =
	__ATTR(vdd1_lock, 0644, vdd_opp_show, vdd_opp_store);
static struct kobj_attribute vdd2_lock_attr =
	__ATTR(vdd2_lock, 0644, vdd_opp_show, vdd_opp_store);
static struct kobj_attribute dsp_freq_attr =
	__ATTR(dsp_freq, 0644, vdd_opp_show, vdd_opp_store);

static int vdd1_locked = 0;
static int vdd2_locked = 0;
static struct device sysfs_cpufreq_dev;

int omap_get_vdd1_lock(void)
{
	return vdd1_locked;
}
EXPORT_SYMBOL(omap_get_vdd1_lock);

int omap_get_vdd2_lock(void)
{
	return vdd2_locked;
}
EXPORT_SYMBOL(omap_get_vdd2_lock);

static ssize_t vdd_opp_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	if (attr == &vdd1_opp_attr)
		return sprintf(buf, "%hu\n", opp_find_freq_exact(mpu_dev, opp_get_rate(mpu_dev), true)->opp_id+1);
	else if (attr == &vdd2_opp_attr)
		return sprintf(buf, "%hu\n", opp_find_freq_exact(l3_dev, opp_get_rate(l3_dev), true)->opp_id+1);
	else if (attr == &vdd1_lock_attr)
		return sprintf(buf, "%hu\n", vdd1_locked);
	else if (attr == &vdd2_lock_attr)
		return sprintf(buf, "%hu\n", vdd2_locked);
	else if (attr == &dsp_freq_attr)
		return sprintf(buf, "%lu\n", opp_get_rate(iva_dev)/1000);
	else
		return -EINVAL;
}

static ssize_t vdd_opp_store(struct kobject *kobj, struct kobj_attribute *attr,
			  const char *buf, size_t n)
{
	unsigned long value;
	static unsigned long prev_mpu_freq = 0;
	if (sscanf(buf, "%lu", &value) != 1)
		return -EINVAL;

	/* Check locks */
	if (attr == &vdd1_lock_attr) {
		if (vdd1_locked) {
			/* vdd1 currently locked */
			if (value == 0) {
				omap_pm_cpu_set_freq(prev_mpu_freq * 1000);
				vdd1_locked = 0;
				return n;
			} else {
				printk(KERN_ERR "%s: vdd1 already locked to %d\n", __func__, vdd1_locked);
				return -EINVAL;
			}
		} else {
			/* vdd1 currently unlocked */
			if (value != 0) {
				u8 i = 0;
				unsigned long freq = 0;
				struct cpufreq_frequency_table *freq_table = *omap_pm_cpu_get_freq_table();
				if (freq_table == NULL) {
					printk(KERN_ERR "%s: Could not get freq_table\n", __func__);
					return -ENODEV;
				}
				for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
					if (freq_table[i].index == value - 1) {
						freq = freq_table[i].frequency;
						break;
					}
				}
				if (freq_table[i].frequency == CPUFREQ_TABLE_END) {
					printk(KERN_ERR "%s: Invalid value [0..%d]\n", __func__, i-1);
					return -EINVAL;
				}
				prev_mpu_freq = omap_pm_cpu_get_freq();
				omap_pm_cpu_set_freq(freq * 1000);
				vdd1_locked = value;

			} else {
				printk(KERN_ERR "%s: vdd1 already unlocked\n", __func__);
				return -EINVAL;
			}
		}
	} else if (attr == &vdd2_lock_attr) {
		if (vdd2_locked) {
			/* vdd2 currently locked */
			if (value == 0) {
				int tmp_lock = vdd2_locked;
				vdd2_locked = 0;
				if (omap_pm_set_min_bus_tput(&sysfs_cpufreq_dev, OCP_INITIATOR_AGENT, -1)) {
					printk(KERN_ERR "%s: Failed to remove vdd2_lock\n", __func__);
					vdd2_locked = tmp_lock; /* restore previous lock */
				} else {
					return n;
				}
			} else {
				printk(KERN_ERR "%s: vdd2 already locked to %d\n", __func__, vdd2_locked);
				return -EINVAL;
			}
		} else {
			/* vdd2 currently unlocked */
			if (value != 0) {
				unsigned long freq = 0;
				if (cpu_is_omap3630()) {
					if(value == 1) {
						freq = 100*1000*4;
					} else if (value == 2) {
						freq = 200*1000*4;
					} else {
						printk(KERN_ERR "%s: Invalid value [1,2]\n", __func__);
						return -EINVAL;
					}
				}
				else if (cpu_is_omap44xx()) {
					if (omap_rev() <= OMAP4430_REV_ES2_0) {
						if(value == 1) {
							freq = 100*1000*4;
						} else if (value == 2) {
							freq = 200*1000*4;
						} else {
							printk(KERN_ERR "%s: Invalid value [1,2]\n", __func__);
							return -EINVAL;
						}
					} else {
						if(value == 1) {
							freq = 98304*4;
						} else if (value == 2) {
							freq = 100*1000*4;
						} else if (value == 3) {
							freq = 200*1000*4;
						} else {
							printk(KERN_ERR "%s: Invalid value [1,2,3]\n", __func__);
							return -EINVAL;
						}
					}
				} else {
					printk(KERN_ERR "%s: Unsupported HW [OMAP3630, OMAP44XX]\n", __func__);
					return -ENODEV;
				}
				if (omap_pm_set_min_bus_tput(&sysfs_cpufreq_dev, OCP_INITIATOR_AGENT, freq)) {
					printk(KERN_ERR "%s: Failed to add vdd2_lock\n", __func__);
				} else {
					vdd2_locked = value;
				}
				return n;
			} else {
				printk(KERN_ERR "%s: vdd2 already unlocked\n", __func__);
				return -EINVAL;
			}
		}
	} else if (attr == &dsp_freq_attr) {
		u8 i, opp_id = 0;
		struct omap_opp *opp_table = omap_pm_dsp_get_opp_table();
		if (opp_table == NULL) {
			printk(KERN_ERR "%s: Could not get dsp opp_table\n", __func__);
			return -ENODEV;
		}
		for (i = 1; opp_table[i].rate; i++) {
			if (opp_table[i].rate >= value) {
				opp_id = i;
				break;
			}
		}
		if (opp_id == 0) {
			printk(KERN_ERR "%s: Invalid value\n", __func__);
			return -EINVAL;
		}
		omap_pm_dsp_set_min_opp(opp_id);

	} else if (attr == &vdd1_opp_attr) {
		printk(KERN_ERR "%s: changing vdd1_opp is not supported\n", __func__);
		return -EINVAL;
	} else if (attr == &vdd2_opp_attr) {
		printk(KERN_ERR "%s: changing vdd2_opp is not supported\n", __func__);
		return -EINVAL;
	} else {
		return -EINVAL;
	}
	return n;
}
#endif

/* static int _init_omap_device(struct omap_hwmod *oh, void *user) */
static int _init_omap_device(char *name, struct device **new_dev)
{
	struct omap_hwmod *oh;
	struct omap_device *od;

	oh = omap_hwmod_lookup(name);
	if (WARN(!oh, "%s: could not find omap_hwmod for %s\n",
		 __func__, name))
		return -ENODEV;
	od = omap_device_build(oh->name, 0, oh, NULL, 0, pm_lats, 0, false);
	if (WARN(IS_ERR(od), "%s: could not build omap_device for %s\n",
		 __func__, name))
		return -ENODEV;

	*new_dev = &od->pdev.dev;

	return 0;
}

/*
 * Build omap_devices for processors and bus.
 */
static void omap2_init_processor_devices(void)
{
	struct omap_hwmod *oh;

	_init_omap_device("mpu", &mpu_dev);

	if (cpu_is_omap34xx())
		_init_omap_device("iva", &iva_dev);
	oh = omap_hwmod_lookup("iva");
	if (oh && oh->od)
		iva_dev = &oh->od->pdev.dev;

	oh = omap_hwmod_lookup("dsp");
	if (oh && oh->od)
		dsp_dev = &oh->od->pdev.dev;

	if (cpu_is_omap44xx())
		_init_omap_device("l3_main_1", &l3_dev);
	else
		_init_omap_device("l3_main", &l3_dev);
}

static int __init omap2_common_pm_init(void)
{
	omap2_init_processor_devices();
	if (cpu_is_omap34xx())
		omap3_pm_init_opp_table();
	else if (cpu_is_omap44xx())
		omap4_pm_init_opp_table();

	omap_pm_if_init();

#ifdef CONFIG_OMAP_PM
	{
		int error = -EINVAL;

		error = sysfs_create_file(power_kobj, &dsp_freq_attr.attr);
		if (error) {
			printk(KERN_ERR "%s: sysfs_create_file(dsp_freq) failed %d\n", __func__, error);
			return error;
		}
		error = sysfs_create_file(power_kobj, &vdd1_opp_attr.attr);
		if (error) {
			printk(KERN_ERR "%s: sysfs_create_file(vdd1_opp) failed %d\n", __func__, error);
			return error;
		}
		error = sysfs_create_file(power_kobj, &vdd2_opp_attr.attr);
		if (error) {
			printk(KERN_ERR "%s: sysfs_create_file(vdd2_opp) failed %d\n", __func__, error);
			return error;
		}
		error = sysfs_create_file(power_kobj, &vdd1_lock_attr.attr);
		if (error) {
			printk(KERN_ERR "%s: sysfs_create_file(vdd1_lock) failed %d\n", __func__ ,error);
			return error;
		}
		error = sysfs_create_file(power_kobj, &vdd2_lock_attr.attr);
		if (error) {
			printk(KERN_ERR "%s: sysfs_create_file(vdd2_lock) failed %d\n", __func__, error);
			return error;
		}
	}
#endif

	return 0;
}
device_initcall(omap2_common_pm_init);
