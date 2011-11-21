/*
 * common.h 
 *
 * Description: This file contains the definition of macros common to 
 * the complete code
 *
 * Author: Varun Mahajan <m.varun@samsung.com>
 */

#ifndef _COMMON_H
#define _COMMON_H

#include <linux/kernel.h>
#include <linux/types.h>

#define DEVICE_NAME "gp2a"

#define error(fmt,arg...) printk(KERN_CRIT "[PSENSOR]: " fmt "\n",## arg)

#ifdef PL_DEBUG
#define debug(fmt,arg...) printk(KERN_CRIT ">>   " fmt "\n",## arg)
#define trace_in()  debug("%s +",__FUNCTION__) 
#define trace_out() debug("%s -",__FUNCTION__) 
#define failed(x) debug("%s failed %d",__FUNCTION__, x)
#else
#define debug(fmt,arg...)
#define trace_in()
#define trace_out()
#define failed(x)
#endif

#endif
