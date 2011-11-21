
#ifndef __MICROUSBIC_DBG__
#define __MICROUSBIC_DBG__


//#define MICROUSBIC_DEBUG
#define MICROUSBIC_DPRINTK

#if 0
#if defined(MICROUSBIC_DEBUG)
	#define kmsg(arg,...)	printk("[ UIC ] %s(%d): "arg,__FUNCTION__,__LINE__, ## __VA_ARGS__)
#elif defined(MICROUSBIC_DPRINTK)
	#include "dprintk.h"
	#define kmsg(arg,...)	dprintk(USB_DBG,"[ UIC ] %s(%d): "arg,__FUNCTION__,__LINE__, ## __VA_ARGS__)
#else
	#define kmsg(arg,...)
#endif
#endif 
#endif
