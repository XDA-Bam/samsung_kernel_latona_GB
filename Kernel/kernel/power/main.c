/*
 * kernel/power/main.c - PM subsystem core functionality.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 * 
 * This file is released under the GPLv2
 *
 */

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/resume-trace.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include "power.h"

#define USE_OMAP_DVFS_LOCK
#ifdef USE_OMAP_DVFS_LOCK
#include <plat/omap-pm.h>
// OMAP3630 OPP Clock Frequency Table
#define VDD1_OPP4_FREQ         S1000M
#define VDD1_OPP3_FREQ         S800M
#define VDD1_OPP2_FREQ         S600M
#define VDD1_OPP1_FREQ         S300M

#define S1000M  1000000000
#define S800M   800000000
#define S600M   600000000
#define S300M   300000000

static void do_dvfsunlock_timer(struct work_struct *work);
static DEFINE_MUTEX (dvfslock_ctrl_mutex);
static DECLARE_DELAYED_WORK(dvfslock_ctrl_unlock_work, do_dvfsunlock_timer);
#endif

DEFINE_MUTEX(pm_mutex);

unsigned int pm_flags;
EXPORT_SYMBOL(pm_flags);
#if 1	// added by peres to show valid current state
suspend_state_t global_state;
#endif

#ifndef FEATURE_FTM_SLEEP
#define FEATURE_FTM_SLEEP
#endif

#ifdef FEATURE_FTM_SLEEP
unsigned char ftm_sleep = 0;
EXPORT_SYMBOL(ftm_sleep);

void (*ftm_enable_usb_sw)(int mode);
EXPORT_SYMBOL(ftm_enable_usb_sw);

extern void wakelock_force_suspend(void);

static struct wake_lock ftm_wake_lock;
#endif

#ifdef CONFIG_PM_SLEEP

/* Routines for PM-transition notifications */

static BLOCKING_NOTIFIER_HEAD(pm_chain_head);

int register_pm_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pm_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_pm_notifier);

int unregister_pm_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&pm_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_pm_notifier);

int pm_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&pm_chain_head, val, NULL)
			== NOTIFY_BAD) ? -EINVAL : 0;
}

/* If set, devices may be suspended and resumed asynchronously. */
int pm_async_enabled = 1;

static ssize_t pm_async_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", pm_async_enabled);
}

static ssize_t pm_async_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;

	pm_async_enabled = val;
	return n;
}

power_attr(pm_async);

#ifdef CONFIG_PM_DEBUG
int pm_test_level = TEST_NONE;

static const char * const pm_tests[__TEST_AFTER_LAST] = {
	[TEST_NONE] = "none",
	[TEST_CORE] = "core",
	[TEST_CPUS] = "processors",
	[TEST_PLATFORM] = "platform",
	[TEST_DEVICES] = "devices",
	[TEST_FREEZER] = "freezer",
};

static ssize_t pm_test_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	char *s = buf;
	int level;

	for (level = TEST_FIRST; level <= TEST_MAX; level++)
		if (pm_tests[level]) {
			if (level == pm_test_level)
				s += sprintf(s, "[%s] ", pm_tests[level]);
			else
				s += sprintf(s, "%s ", pm_tests[level]);
		}

	if (s != buf)
		/* convert the last space to a newline */
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t pm_test_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	const char * const *s;
	int level;
	char *p;
	int len;
	int error = -EINVAL;

	p = memchr(buf, '\n', n);
	len = p ? p - buf : n;

	mutex_lock(&pm_mutex);

	level = TEST_FIRST;
	for (s = &pm_tests[level]; level <= TEST_MAX; s++, level++)
		if (*s && len == strlen(*s) && !strncmp(buf, *s, len)) {
			pm_test_level = level;
			error = 0;
			break;
		}

	mutex_unlock(&pm_mutex);

	return error ? error : n;
}

power_attr(pm_test);
#endif /* CONFIG_PM_DEBUG */

#endif /* CONFIG_PM_SLEEP */



/**
 * store_dvfslock_ctrl - make dvfs lock through application
 */
#ifdef USE_OMAP_DVFS_LOCK
//extern int g_dbs_timer_started;
int dvfsctrl_locked;
int gdDvfsctrl = 0;
struct device dvfs_ctrl_device;

static ssize_t dvfslock_ctrl(const char *buf, size_t count)
{
	unsigned int ret = -EINVAL;
	int dlevel;
	int dtime_msec;
	
	//mutex_lock(&dvfslock_ctrl_mutex);
	ret = sscanf(buf, "%u", &gdDvfsctrl);
	if (ret != 1)
		return -EINVAL;
	
	//if (!g_dbs_timer_started) return -EINVAL;
	if (gdDvfsctrl == 0) {
		if (dvfsctrl_locked) {
			omap_pm_set_min_mpu_freq(&dvfs_ctrl_device, VDD1_OPP1_FREQ);
			//s5pc110_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_6);
			dvfsctrl_locked = 0;
			return -EINVAL;		
		} else {
			return -EINVAL;		
		}
	}
	
	if (dvfsctrl_locked) return 0;
		
	dlevel = gdDvfsctrl & 0xffff0000;
	dtime_msec = gdDvfsctrl & 0x0000ffff;
	if (dtime_msec < 16) dtime_msec=16;
	
	if (dtime_msec == 0) return -EINVAL;
	if(dlevel) dlevel = 1;
	
	printk("+++++DBG dvfs lock level=%d, time=%d, scanVal=%08x\n",dlevel,dtime_msec, gdDvfsctrl);
	omap_pm_set_min_mpu_freq(&dvfs_ctrl_device, VDD1_OPP4_FREQ);
	//s5pc110_lock_dvfs_high_level(DVFS_LOCK_TOKEN_6, dlevel);
	dvfsctrl_locked=1;


	schedule_delayed_work(&dvfslock_ctrl_unlock_work, msecs_to_jiffies(dtime_msec));

	//mutex_unlock(&dvfslock_ctrl_mutex);

	return -EINVAL;
}

static void do_dvfsunlock_timer(struct work_struct *work) {
	printk("----DBG dvfs unlock\n");
	dvfsctrl_locked = 0;
	omap_pm_set_min_mpu_freq(&dvfs_ctrl_device, VDD1_OPP1_FREQ);
	//s5pc110_unlock_dvfs_high_level(DVFS_LOCK_TOKEN_6);
}


ssize_t dvfslock_ctrl_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%08x\n", gdDvfsctrl);

}

ssize_t dvfslock_ctrl_store(
	struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t n)
{
	dvfslock_ctrl(buf, 0);
	return n;
}

#endif


int i_power_off = 0;
ssize_t poweroff_flag_show(
	struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", i_power_off);

}

ssize_t poweroff_flag_store(
	struct kobject *kobj, struct kobj_attribute *attr,
	const char *buf, size_t n)
{
	int flag;
	sscanf( buf, "%d", &flag );
	i_power_off = flag;
	return n;
}

struct kobject *power_kobj;

/**
 *	state - control system power state.
 *
 *	show() returns what states are supported, which is hard-coded to
 *	'standby' (Power-On Suspend), 'mem' (Suspend-to-RAM), and
 *	'disk' (Suspend-to-Disk).
 *
 *	store() accepts one of those strings, translates it into the 
 *	proper enumerated value, and initiates a suspend transition.
 */
static ssize_t state_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char *s = buf;
#ifdef CONFIG_SUSPEND
	int i;

	for (i = 0; i < PM_SUSPEND_MAX; i++) {
		if (pm_states[i] && valid_state(i))
			s += sprintf(s,"%s ", pm_states[i]);
	}
#endif
#ifdef CONFIG_HIBERNATION
	s += sprintf(s, "%s\n", "disk");
#else
	if (s != buf)
		/* convert the last space to a newline */
		*(s-1) = '\n';
#endif
	return (s - buf);
}

static ssize_t state_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
#if 1	// added by peres to show valid current state
	const char *b = buf;
#endif
#ifdef CONFIG_SUSPEND
#ifdef CONFIG_EARLYSUSPEND
	suspend_state_t state = PM_SUSPEND_ON;
#else
	suspend_state_t state = PM_SUSPEND_STANDBY;
#endif
	const char * const *s;
#endif
	char *p;
	int len;
	int error = -EINVAL;

	p = memchr(buf, '\n', n);
	len = p ? p - buf : n;

	/* First, check if we are requested to hibernate */
	if (len == 4 && !strncmp(buf, "disk", len)) {
		error = hibernate();
  goto Exit;
	}

#ifdef CONFIG_SUSPEND
	for (s = &pm_states[state]; state < PM_SUSPEND_MAX; s++, state++) {
		if (*s && len == strlen(*s) && !strncmp(buf, *s, len))
			break;
	}
	if (state < PM_SUSPEND_MAX && *s) {
		printk(KERN_ERR "%s: state:%d (%s)\n", __func__, state, *s);
#ifdef CONFIG_EARLYSUSPEND
		if (state == PM_SUSPEND_ON || valid_state(state)) {
#if 1	// added by peres to show valid current state
			if(state == PM_SUSPEND_ON) {
				if (ftm_sleep == 1) {
					pr_info("%s: wake lock for FTM\n", __func__);
					ftm_sleep = 0;
					wake_lock_timeout(&ftm_wake_lock, 60 * HZ);
					if (ftm_enable_usb_sw)
						ftm_enable_usb_sw(1);
				}
				sprintf(b,"%s ", pm_states[PM_SUSPEND_ON]);
				global_state = PM_SUSPEND_ON;
			} else {
				if (ftm_sleep == 1) { // when ftm sleep cmd 
					if (ftm_enable_usb_sw)
						ftm_enable_usb_sw(0);
				}
				sprintf(b,"%s ", pm_states[PM_SUSPEND_MEM]);
				global_state = PM_SUSPEND_MEM;
			}
#endif
			error = 0;
			request_suspend_state(state);

#ifdef FEATURE_FTM_SLEEP
			if (ftm_sleep && global_state == PM_SUSPEND_MEM) {
				wakelock_force_suspend();
			}
#endif /* FEATURE_FTM_SLEEP */
		}
#else
		error = enter_state(state);
#endif
	}
#endif

 Exit:
	return error ? error : n;
}

power_attr(state);

#ifdef FEATURE_FTM_SLEEP /* for Factory Sleep cmd check */


#define ftm_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0777,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}


static ssize_t ftm_sleep_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char *s = buf;
#ifdef CONFIG_SUSPEND
	switch(ftm_sleep) {
		case 0 :
			s += sprintf(s,"%d ", 0);
			break;
		case 1 :
			s += sprintf(s,"%d ", 1);
			break;
	}
#endif

	if (s != buf)
		/* convert the last space to a newline */
		*(s-1) = '\n';

	return (s - buf);
}


static ssize_t ftm_sleep_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	ssize_t ret = -EINVAL;
	char *after;
	unsigned char state = (unsigned char) simple_strtoul(buf, &after, 10);

	ftm_sleep = state;

	printk("%s, ftm_sleep = %d\n", __func__, ftm_sleep);        
	return ret;

}

ftm_attr(ftm_sleep);
#endif /* FEATURE_FTM_SLEEP */


#ifdef CONFIG_PM_TRACE
int pm_trace_enabled;

static ssize_t pm_trace_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", pm_trace_enabled);
}

static ssize_t
pm_trace_store(struct kobject *kobj, struct kobj_attribute *attr,
	       const char *buf, size_t n)
{
	int val;

	if (sscanf(buf, "%d", &val) == 1) {
		pm_trace_enabled = !!val;
		return n;
	}
	return -EINVAL;
}

power_attr(pm_trace);
#endif /* CONFIG_PM_TRACE */

#ifdef CONFIG_USER_WAKELOCK
power_attr(wake_lock);
power_attr(wake_unlock);
#endif

#ifdef USE_OMAP_DVFS_LOCK
power_attr(dvfslock_ctrl);
#endif
power_attr(poweroff_flag);
static struct attribute * g[] = {
	&state_attr.attr,
#ifdef CONFIG_PM_TRACE
	&pm_trace_attr.attr,
#endif
#ifdef CONFIG_PM_SLEEP
	&pm_async_attr.attr,
#ifdef CONFIG_PM_DEBUG
	&pm_test_attr.attr,
#endif
#endif
#ifdef CONFIG_USER_WAKELOCK
	&wake_lock_attr.attr,
	&wake_unlock_attr.attr,
#endif
#ifdef USE_OMAP_DVFS_LOCK
	&dvfslock_ctrl_attr.attr,
#endif
	&ftm_sleep_attr.attr,
	&poweroff_flag_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

#ifdef CONFIG_PM_RUNTIME
struct workqueue_struct *pm_wq;
EXPORT_SYMBOL_GPL(pm_wq);

static int __init pm_start_workqueue(void)
{
	pm_wq = create_freezeable_workqueue("pm");

	return pm_wq ? 0 : -ENOMEM;
}
#else
static inline int pm_start_workqueue(void) { return 0; }
#endif

static int __init pm_init(void)
{
	int error = pm_start_workqueue();
	if (error)
		return error;
	power_kobj = kobject_create_and_add("power", NULL);
	if (!power_kobj)
		return -ENOMEM;

	wake_lock_init(&ftm_wake_lock, WAKE_LOCK_SUSPEND, "ftm_wake_lock");

	return sysfs_create_group(power_kobj, &attr_group);
}

core_initcall(pm_init);
