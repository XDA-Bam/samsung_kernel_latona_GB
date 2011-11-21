
#ifndef __MICROUSBIC_DRV__
#define __MICROUSBIC_DRV__

#define MICROUSBIC_5W_CHARGER		6
#define MICROUSBIC_JIG_UART_OFF		5
#define MICROUSBIC_JIG_UART_ON		4

#define MICROUSBIC_TA_CHARGER		3
#define MICROUSBIC_USB_CHARGER		2
#define MICROUSBIC_USB_CABLE		1
#define MICROUSBIC_NO_DEVICE		0
#define MICROUSBIC_PHONE_USB		-1

#define MICROUSBIC_ATTACH			1
#define MICROUSBIC_DETACH			0

struct microusbic_event
{
	int	device;
	int event;
};

int get_usbic_state(void);
int get_real_usbic_state(void);
void usbic_usb_switch(int sel);
int get_interrupt_event(struct microusbic_event * evt);

// DEBUG
#define _get_real_usbic_state() \
	do { \
		printk("[uic] >>>>> %s (%d)\n", __FUNCTION__,__LINE__); \
		get_real_usbic_state(); \
	} while(0)

#endif

