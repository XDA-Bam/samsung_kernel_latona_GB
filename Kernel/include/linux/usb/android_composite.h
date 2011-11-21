/*
 * Platform data for Android USB
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef	__LINUX_USB_ANDROID_H
#define	__LINUX_USB_ANDROID_H

#include <linux/usb/composite.h>
#include <linux/if_ether.h>

struct android_usb_function {
	struct list_head	list;
	char			*name;
	int 			(*bind_config)(struct usb_configuration *c);
};

struct android_usb_product {
	/* Vendor ID for this set of functions.
	 * Default vendor_id in platform data will be used if this is zero.
	 */
	__u16 vendor_id;

	/* Product ID for this set of functions. */
	__u16 product_id;

	/* List of function names associated with this product.
	 * This is used to compute the USB product ID dynamically
	 * based on which functions are enabled.
	 */
	int num_functions;
	char **functions;
#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
/* soonyong.cho : Below variables are used for Samsung composite framework. */
        __u8 bDeviceClass;
	__u8 bDeviceSubClass;
	__u8 bDeviceProtocol;
	int  mode; /* if product id is same, you have to refer this mode value. */
	char *s;
#endif
};

struct android_usb_platform_data {
	/* USB device descriptor fields */
	__u16 vendor_id;

	/* Default product ID. */
	__u16 product_id;

	__u16 version;

	char *product_name;
	char *manufacturer_name;
	char *serial_number;

	/* List of available USB products.
	 * This is used to compute the USB product ID dynamically
	 * based on which functions are enabled.
	 * if num_products is zero or no match can be found,
	 * we use the default product ID
	 */
	int num_products;
	struct android_usb_product *products;

	/* List of all supported USB functions.
	 * This list is used to define the order in which
	 * the functions appear in the configuration's list of USB interfaces.
	 * This is necessary to avoid depending upon the order in which
	 * the individual function drivers are initialized.
	 */
	int num_functions;
	char **functions;
};

/* Platform data for "usb_mass_storage" driver. */
struct usb_mass_storage_platform_data {
	/* Contains values for the SC_INQUIRY SCSI command. */
	char *vendor;
	char *product;
	int release;

	/* number of LUNS */
	int nluns;
};

/* Platform data for USB ethernet driver. */
struct usb_ether_platform_data {
	u8	ethaddr[ETH_ALEN];
	u32	vendorID;
	const char *vendorDescr;
};

extern void android_register_function(struct android_usb_function *f);

extern void android_enable_function(struct usb_function *f, int enable);

#ifdef CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE
/* soonyong.cho : Define samsung product id and config string.
 *                Sources such as 'android.c' and 'devs.c' refered below define
 */
#  define SAMSUNG_VENDOR_ID		0x04e8

#  ifdef CONFIG_USB_ANDROID_SAMSUNG_ESCAPE
	/* USE DEVGURU HOST DRIVER */
	/* 0x6860 : MTP(0) + MS Composite (UMS) */
	/* 0x685E : UMS(0) + MS Composite (ADB) */
#  ifdef CONFIG_USB_ANDROID_SAMSUNG_KIES_UMS
#    define SAMSUNG_KIES_PRODUCT_ID	0x685e	/* ums(0) + acm(1,2) */
#  else
#    define SAMSUNG_KIES_PRODUCT_ID	0x6860	/* mtp(0) + acm(1,2) */
#  endif
#    define SAMSUNG_DEBUG_PRODUCT_ID	0x685e	/* ums(0) + acm(1,2) + adb(3) (with MS Composite) */
#    define SAMSUNG_UMS_PRODUCT_ID	0x685B  /* UMS Only */
#    define SAMSUNG_MTP_PRODUCT_ID	0x685C  /* MTP Only */
#    ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
#      define SAMSUNG_RNDIS_PRODUCT_ID	0x6861  /* RNDIS(0,1) + UMS (2) + MS Composite */
#    else
#      define SAMSUNG_RNDIS_PRODUCT_ID	0x6863  /* RNDIS only */
#    endif
#    define ANDROID_DEBUG_CONFIG_STRING	 "UMS + ACM + ADB (Debugging mode)"
#  ifdef CONFIG_USB_ANDROID_SAMSUNG_KIES_UMS
#    define ANDROID_KIES_CONFIG_STRING	 "UMS + ACM (SAMSUNG KIES mode)"
#  else
#    define ANDROID_KIES_CONFIG_STRING	 "MTP + ACM (SAMSUNG KIES mode)"
#  endif
#  else /* USE MCCI HOST DRIVER */
#    define SAMSUNG_KIES_PRODUCT_ID	0x6877	/* Shrewbury ACM+MTP*/
#    define SAMSUNG_DEBUG_PRODUCT_ID	0x681C	/* Shrewbury ACM+UMS+ADB*/
#    define SAMSUNG_UMS_PRODUCT_ID	0x681D
//#    define SAMSUNG_MTP_PRODUCT_ID	0x68A9
#    define SAMSUNG_MTP_PRODUCT_ID	0x5A0F
#    define SAMSUNG_RNDIS_PRODUCT_ID	0x6863
#    define ANDROID_DEBUG_CONFIG_STRING	 "ACM + UMS + ADB (Debugging mode)"
#    define ANDROID_KIES_CONFIG_STRING	 "ACM + MTP (SAMSUNG KIES mode)"
#  endif
#  define       ANDROID_UMS_CONFIG_STRING	 "UMS Only (Not debugging mode)"
#  define       ANDROID_MTP_CONFIG_STRING	 "MTP Only (Not debugging mode)"
#  define       ANDROID_ACC_CONFIG_STRING	 "ACCESSORY Only (ADK mode)"
#  define       ANDROID_ACC_ADB_CONFIG_STRING	 "ACCESSORY _ADB (ADK + ADB mode)"

#  ifdef CONFIG_USB_ANDROID_SAMSUNG_RNDIS_WITH_MS_COMPOSITE
#    define       ANDROID_RNDIS_CONFIG_STRING	 "RNDIS + UMS (Not debugging mode)"
#  else
#    define       ANDROID_RNDIS_CONFIG_STRING	 "RNDIS Only (Not debugging mode)"
#  endif
	/* Refered from S1, P1 */
#  define USBSTATUS_UMS				0x0
#  define USBSTATUS_SAMSUNG_KIES 		0x1
#  define USBSTATUS_MTPONLY			0x2
#  define USBSTATUS_ASKON			0x4
#  define USBSTATUS_VTP				0x8
#  define USBSTATUS_ADB				0x10
#  define USBSTATUS_ACCESSORY				0x20
#  define USBSTATUS_ACM				0x40
#  define USBSTATUS_SAMSUNG_KIES_REAL		0x80

typedef enum usb_cable_status {
	USB_CABLE_DETACHED = 0,
	USB_CABLE_ATTACHED,
	USB_OTGHOST_DETACHED,
	USB_OTGHOST_ATTACHED
} usb_cable_status;

#endif /* CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE */

#endif	/* __LINUX_USB_ANDROID_H */
