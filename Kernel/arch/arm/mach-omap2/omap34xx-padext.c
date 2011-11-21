/*
 * arch/arm/mach-omap2/omap3-padext.c
 *
 * OMAP2 and OMAP3 pin multiplexing configurations
 *
 * Copyright (C) 2004 - 2008 Texas Instruments Inc.
 * Copyright (C) 2003 - 2008 Nokia Corporation
 *
 * Written by Tony Lindgren
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/spinlock.h>
#include <plat/control.h>
#include <plat/mux.h>

#define PAD_WAKEUP_EN	1<<14
#define PAD_WAKEUP_NA	0<<14

static DEFINE_SPINLOCK(pad_spin_lock); 

typedef unsigned int pad_pin_t;

                
struct gpio_pad_range {
	/* Range start GPIO # */
	u16 min;
	/* Range end GPIO # */
	u16 max;
	/* Start pad config offset */
	u16 offset;
};


static const struct gpio_pad_range gpio_pads_config[] = {
	{ 0, 0, 0x1e0 },
	{ 1, 1, 0xa06 },
	{ 2, 10, 0xa0a },
	{ 11, 11, 0xa24 },
	{ 12, 29, 0x5d8 },
	{ 30, 30, 0xa08 },
	{ 31, 31, 0xa26 },
	{ 32, 32, 0x23a },
	{ 34, 43, 0x7a },
	{ 44, 51, 0x9e },
	{ 52, 59, 0xb0 },
	{ 60, 62, 0xc6 },
	{ 63, 111, 0xce },
	{ 112, 119, 0x134 },
	{ 120, 122, 0x1a2 },
	{ 124, 125, 0x1a8 },
	{ 126, 126, 0x132 },
	{ 127, 129, 0x152 },
	{ 130, 131, 0x1ac },
	{ 132, 166, 0x15c },
	{ 167, 167, 0x130 },
	{ 168, 168, 0x1be },
	{ 169, 169, 0x1b0 },
	{ 170, 182, 0x1c6 },
	{ 183, 185, 0x1c0 },
	{ 186, 186, 0x1e2 },
	{ 187, 187, 0x238 },
	{ 188, 191, 0x1b2 },
};

static u16 omap34xx_pad_gpio_num2offset( int gpio )
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(gpio_pads_config); i++) {

		if ( gpio == gpio_pads_config[i].min )
		{
			return gpio_pads_config[i].offset;
		}
		else if ( gpio < gpio_pads_config[i].min )
		{
			if ( gpio <= gpio_pads_config[i-1].max )
				return gpio_pads_config[i-1].offset + ( ( gpio - gpio_pads_config[i-1].min ) * 2 );
			else 
				return -1;
		}
		else
		{
			;
		}
	}

	return -1;
}

#if 0
int omap34xx_pad_set_wakeup_src(int gpio, int wakeup_en)
{
	unsigned long flags;
    u16 pad_offset;
	u16 pad_val;
    int ret = 0;

	pad_offset = omap34xx_pad_gpio_num2offset(gpio);
	//pad_offset = 0x5e0; // w/a chg_ing
    if(pad_offset < 0)
    {
        printk("omap34xx_pad_set_padoff : invalid gpio!\n");
        return ret;
    }
    else
    {
        printk("omap34xx_pad_set_padoff : gpio: %d, offset: %x\n", gpio, pad_offset);
    }

	spin_lock_irqsave(&pad_spin_lock, flags);

	pad_val = omap_ctrl_readw(pad_pin);

	pad_val &= ~(PAD_WAKEUP_EN);
	pad_val |= wakeup_en;

	omap_ctrl_writew(pad_val, pad_pin);
	omap_ctrl_readw(pad_pin);

	spin_unlock_irqrestore(&pad_spin_lock, flags);
	return 0;
}
EXPORT_SYMBOL(omap3430_pad_set_padoff);
#endif

int omap34xx_pad_set_config(struct pin_config *pin_config)
{
	unsigned long flags;
	u16 pad_pin;
	u16  pad_val;

	spin_lock_irqsave(&pad_spin_lock, flags);

//ARCHER_MIGRATION
#if 0	
        pad_pin = pin_config->mux_reg;
	pad_val = pin_config->mux_val;
#endif

	omap_ctrl_writew(pad_val, pad_pin);
	omap_ctrl_readw(pad_pin);

	spin_unlock_irqrestore(&pad_spin_lock, flags);
	
	return 0;
}
EXPORT_SYMBOL(omap34xx_pad_set_config);


int omap34xx_pad_set_config_lcd(u16 pad_pin,u16 pad_val)
{
        unsigned long flags;
        spin_lock_irqsave(&pad_spin_lock, flags);
        omap_ctrl_writew(pad_val, pad_pin);
        omap_ctrl_readw(pad_pin);
        spin_unlock_irqrestore(&pad_spin_lock, flags);
        return 0;

}

EXPORT_SYMBOL(omap34xx_pad_set_config_lcd);
int omap34xx_pad_set_configs(struct pin_config *pin_configs, int n)
{
	unsigned long flags;
	u16 pad_pin;
	u16  pad_val;
	int i=0;

	if (n <= 0)
		return 0;

	spin_lock_irqsave(&pad_spin_lock, flags);

	for (i = 0; i < n; i++) 
	{
		pad_pin = pin_configs->mux_reg;
	//ARCHER_MIGRATION
	#if 0
         	pad_val = pin_configs->mux_val;
	#endif	
		omap_ctrl_writew(pad_val, pad_pin);

		pin_configs++;
	}
	spin_unlock_irqrestore(&pad_spin_lock, flags);

	return 0;
}
EXPORT_SYMBOL(omap34xx_pad_set_configs);


int omap34xx_pad_set_padoff(int gpio, int wakeup_en)
{
	unsigned long flags;
	u16 pad_val, pad_offset;
	int ret=0;

	pad_offset = omap34xx_pad_gpio_num2offset(gpio);
	if(pad_offset < 0)
	{
		printk("omap34xx_pad_get_wakeup_status : invalid gpio!\n");
		return ret;
	}

	spin_lock_irqsave(&pad_spin_lock, flags);

	pad_val = omap_ctrl_readw(pad_offset);

	pad_val &= ~(PAD_WAKEUP_EN);
	pad_val |= wakeup_en;

	omap_ctrl_writew(pad_val, pad_offset);
	omap_ctrl_readw(pad_offset);

	spin_unlock_irqrestore(&pad_spin_lock, flags);

	return 0;
}
EXPORT_SYMBOL(omap34xx_pad_set_padoff);


int omap34xx_pad_get_wakeup_status(int gpio)
{
	unsigned long flags;
	u16 pad_val, pad_offset;
	int ret=0;

	pad_offset = omap34xx_pad_gpio_num2offset(gpio);
	//pad_offset = 0x5e0; // w/a chg_ing
	if(pad_offset < 0)
	{
		printk("omap34xx_pad_get_wakeup_status : invalid gpio!\n");
		return ret;
	}
//	else
//	{
//		printk("omap34xx_pad_get_wakeup_status : gpio: %d, offset: %x\n", gpio, pad_offset);
//	}

	spin_lock_irqsave(&pad_spin_lock, flags);

	pad_val = omap_ctrl_readw(pad_offset);

	ret = (pad_val & 0x8000) >> 15;/*WAKEUP STATUS*/

	/*clear wakeup enable bit*/
	//pad_val &= ~(PAD_WAKEUP_EN);

	//omap_ctrl_writew(pad_val, pad_offset);

	spin_unlock_irqrestore(&pad_spin_lock, flags);

	return ret;
}
EXPORT_SYMBOL(omap34xx_pad_get_wakeup_status);

