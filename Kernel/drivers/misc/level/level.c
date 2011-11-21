/**
 * Copyright (C) 2010-2011, Samsung Electronics, Co., Ltd. All Rights Reserved.
 *  Written by System S/W Group, Open OS S/W R&D Team,
 *  Mobile Communication Division.
 */

/**
 * Project Name : Samsung Debug-Level Control Driver
 *
 * Project Description :
 *
 * Comments : tabstop = 8, shiftwidth = 8, noexpandtab
 */

/**
 * File Name : level.c
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Renewal : 27/Jan/2011
 * Version : Baby-Raccoon
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/kernel_sec_common.h>
#include <linux/uaccess.h>
#include <linux/sysfs.h>

#if defined CONFIG_MACH_OMAP_SAMSUNG
#include <mach/sec_param.h>

/* not implemented functions in OMAP-Samsung Products */
#define kernel_sec_clear_upload_magic_number()
#define kernel_sec_set_autotest()
#endif /* CONFIG_MACH_OMAP_SAMSUNG */

#define LEVEL_DEV_NAME			"level"

#define LEVEL_DEV_IOCTL_CMD		0xEE

#define LEVEL_DEV_UNSET_UPLOAD		_IO(LEVEL_DEV_IOCTL_CMD, 0x1)
#define LEVEL_DEV_SET_AUTOTEST		_IO(LEVEL_DEV_IOCTL_CMD, 0x2)
#define LEVEL_DEV_SET_DEBUGLEVEL	_IO(LEVEL_DEV_IOCTL_CMD, 0x3)
#define LEVEL_DEV_GET_DEBUGLEVEL	_IO(LEVEL_DEV_IOCTL_CMD, 0x4)

static void set_debug_level(void);
static unsigned int get_debug_level(void);

static ssize_t show_control(struct device *d,
			    struct device_attribute *attr, char *buf);
static ssize_t store_control(struct device *d,
			     struct device_attribute *attr, const char *buf,
			     size_t count);

static DEVICE_ATTR(control, S_IRUGO | S_IWUSR | S_IWGRP, show_control, store_control);

static struct attribute *levelctl_attributes[] = {
	&dev_attr_control.attr,
	NULL
};

static const struct attribute_group levelctl_group = {
	.attrs = levelctl_attributes,
};

static ssize_t show_control(struct device *d,
			    struct device_attribute *attr, char *buf)
{
	char *p = buf;
	unsigned int val;

	val = get_debug_level();

	p += sprintf(p, "0x%4x\n", val);

	return p - buf;
}

static ssize_t store_control(struct device *d,
			     struct device_attribute *attr, const char *buf,
			     size_t count)
{

	if (!strncmp(buf, "clear", 5)) {
		/* clear upload magic number */
		kernel_sec_clear_upload_magic_number();
		return count;
	} else if (!strncmp(buf, "autotest", 8)) {
		/* set auto test */
		kernel_sec_set_autotest();
		return count;
	} else if (!strncmp(buf, "set", 3)) {
		/* set debug level */
		set_debug_level();
		return count;
	}

	return count;
}

static int level_open(struct inode *inode, struct file *filp)
{
	pr_info("level Device open\n");

	return 0;
}

static int level_ioctl(struct inode *inode, struct file *filp,
		       unsigned int cmd, unsigned long arg)
{
	unsigned int val;

	switch (cmd) {
	case LEVEL_DEV_UNSET_UPLOAD:
		kernel_sec_clear_upload_magic_number();
		return 0;

	case LEVEL_DEV_SET_AUTOTEST:
		kernel_sec_set_autotest();
		return 0;

	case LEVEL_DEV_SET_DEBUGLEVEL:
		set_debug_level();
		return 0;

	case LEVEL_DEV_GET_DEBUGLEVEL:
		val = get_debug_level();
		return copy_to_user((unsigned int *)arg, &val, sizeof(val));
	default:
		pr_info("Unknown Cmd: %x\n", cmd);
		break;
	}
	return -ENOTSUPP;
}

static void set_debug_level(void)
{
	u32 level = 0;
#if defined CONFIG_MACH_OMAP_SAMSUNG
	char debug_level[16];
	char *new_level = NULL;

	sec_get_param_value(__DEBUG_LEVEL, (void *)debug_level);
	new_level = debug_level;

	level = (u32) (*(u32 *) debug_level);
#else
	level = kernel_sec_get_debug_level_from_param();
#endif /* CONFIG_MACH_OMAP_SAMSUNG */

	switch (level) {
	case KERNEL_SEC_DEBUG_LEVEL_LOW:
		new_level = "DMID";
		break;
	case KERNEL_SEC_DEBUG_LEVEL_MID:
		new_level = "DHIG";
		break;
	case KERNEL_SEC_DEBUG_LEVEL_HIGH:
		new_level = "DLOW";
		break;
	default:
		break;
	}

#if defined CONFIG_MACH_OMAP_SAMSUNG
	sec_set_param_value(__DEBUG_LEVEL, (void *)new_level);
#else
	kernel_sec_set_debug_level(*(unsigned int *)new_level);
#endif /* CONFIG_MACH_OMAP_SAMSUNG */
}

static unsigned int get_debug_level(void)
{
	u32 level = 0;
#if defined CONFIG_MACH_OMAP_SAMSUNG
	char debug_level[16];
	unsigned int val = 0;

	sec_get_param_value(__DEBUG_LEVEL, (void *)debug_level);

	level = (u32) (*(u32 *) debug_level);
#else
	level = kernel_sec_get_debug_level_from_param();
#endif /* CONFIG_MACH_OMAP_SAMSUNG */

	switch (level) {
	case KERNEL_SEC_DEBUG_LEVEL_LOW:
		val = 0xA0A0;
		break;
	case KERNEL_SEC_DEBUG_LEVEL_MID:
		val = 0xB0B0;
		break;
	case KERNEL_SEC_DEBUG_LEVEL_HIGH:
		val = 0xC0C0;
		break;
	default:
		val = 0xFFFF;
		break;
	}

	pr_info("level : get debug level = 0x%x\n", val);

	return val;
}

static const struct file_operations level_fops = {
	.owner = THIS_MODULE,
	.open = level_open,
	.ioctl = level_ioctl,
};

static struct miscdevice level_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = LEVEL_DEV_NAME,
	.fops = &level_fops,
};

/* init & cleanup. */
static int __init level_init(void)
{
	int result;

	pr_info("level device init\n");

	result = misc_register(&level_device);
	if (result < 0)
		return result;

	result =
	    sysfs_create_group(&level_device.this_device->kobj,
			       &levelctl_group);
	if (result < 0)
		pr_info("failed to create sysfs files\n");

	return 0;
}

static void __exit level_exit(void)
{
	pr_info("level device exit\n");
	misc_deregister(&level_device);
}

module_init(level_init);
module_exit(level_exit);
