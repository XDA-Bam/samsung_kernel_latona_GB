
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel_stat.h> // kstat_cpu()
#include <linux/random.h> //add_interrupt_randomness()
  
#include <asm/io.h>
#include <asm/uaccess.h>
     

#include <plat/gpio.h>
#include <plat/microusbic.h>
#include <plat/hardware.h>
#include <plat/mux.h>
#include "microusbic_dbg.h"

struct work_struct	work_device_intr;
static int dev_id = 100;
  


//////////////////////////////////////////////////////////

static void microusbic_i2c_ackirq(u32 irq) { }
static void microusbic_i2c_disableint(u32 irq) { }
static void microusbic_i2c_enableint(u32 irq) {	}

static struct irq_chip microusbic_irq_chip =
{
	.ack 	= microusbic_i2c_ackirq,
	.mask	= microusbic_i2c_disableint,
	.unmask	= microusbic_i2c_enableint,
};


static void usbic_device_irq(u32 irq, irq_desc_t * desc)
{
	struct irqaction * action;
	const u32 cpu = smp_processor_id();

	desc->status |= IRQ_LEVEL;



	if(!desc->depth)
	{
		kstat_cpu(cpu).irqs[irq]++;

		action = desc->action;
		if(action)
		{
			int ret;
			int status = 0;
			int retval = 0;

			local_irq_enable();

			do{
				ret = action->handler(irq, action->dev_id);
				if(ret == IRQ_HANDLED)
					status |= action->flags;
				retval |= ret;
				action = action->next;
			}while(action);

			if(status & IRQF_SAMPLE_RANDOM)
			{
			//	kmsg("sample randomness\n");
				add_interrupt_randomness(irq);
			}

			local_irq_disable();

			if(retval != IRQ_HANDLED)
			; //	kmsg("Error!!! irq %d cannot handle irq\n", irq);
		}
		else
		 ;//	kmsg("irq %d has no ISR\n", irq);
	}
	else
	  ; //	kmsg("irq %d is disabled\n",irq);
}

static int install_usbic_irq_chip_handler(void)
{
	int i;

	/* install an irq handler */
	for ( i = IH_USBIC_BASE; i< IH_USBIC_END; i++)
	{
		set_irq_chip(i, &microusbic_irq_chip);
		set_irq_handler(i, usbic_device_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	return 0;
}

void work_handler_device_intr(struct work_struct * work)
{

	struct microusbic_event event;
	int dev_irq;

	if(!get_interrupt_event(&event))
		return;
	switch (event.device)
	{
		case MICROUSBIC_TA_CHARGER:
		case MICROUSBIC_USB_CHARGER: 	dev_irq = IH_USBIC_BASE + 1; break;
		case MICROUSBIC_JIG_UART_OFF:
		case MICROUSBIC_JIG_UART_ON:	dev_irq = IH_USBIC_BASE + 2; break;
		case MICROUSBIC_USB_CABLE:		dev_irq = IH_USBIC_BASE; break;
		case MICROUSBIC_NO_DEVICE:
		default:
			//	kmsg("Unknown INTR : 0x%02x, 0x%02x\n", event.device, event.event);
			return;
	}

	do{
		irq_desc_t * d = irq_desc + dev_irq;
		
		local_irq_disable();

		d->handle_irq(dev_irq, d);

		local_irq_enable();

	} while(0);
	//kmsg("2 end work handler\n\n");
}

irqreturn_t microusbic_interrupt(int irq, void *dev)
{
#if 0
	int *id = (int *)dev;
	kmsg("2 irq : %d, gpio %d level =======> %d\n", irq , irq_to_gpio(irq),
			omap_get_gpio_datain(irq_to_gpio(irq)));
#endif

	schedule_work(&work_device_intr);
	return IRQ_HANDLED;
}


int microusbic_fast_init(void)
{
	int microusbic_irq = 0;

     	printk(" Microusbic init\n");

	install_usbic_irq_chip_handler();

	INIT_WORK(&work_device_intr,  work_handler_device_intr);

	////////////////////////////////////////////////////////////
	// microusbic irq
	microusbic_irq = OMAP_GPIO_IRQ(OMAP_GPIO_JACK_NINT);
	set_irq_type(microusbic_irq, 3);          //IRQT_BOTHEDGE

	if(request_irq(microusbic_irq, microusbic_interrupt,
				IRQF_DISABLED | IRQF_SHARED, "USBIC", (void *)&dev_id))
		printk("[microusbic] Error! irq %d in use.\n", microusbic_irq);
	else
		printk("[microusbic] found microusbic at irq %d\n", microusbic_irq);
	enable_irq_wake(microusbic_irq);
	return 0;
}

void microusbic_fast_exit(void)
{
	printk(" Microusbic exit\n");
	free_irq(OMAP_GPIO_IRQ(OMAP_GPIO_JACK_NINT), (void *)dev_id);
}

arch_initcall(microusbic_fast_init);
module_exit(microusbic_fast_exit);

MODULE_AUTHOR("Hyuk Kang hyuk78.kang@samsung.com");
MODULE_DESCRIPTION("micro usb");
MODULE_LICENSE("GPL");
