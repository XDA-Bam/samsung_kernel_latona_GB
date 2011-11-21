/*
 * linux/arch/arm/plat-omap/i2c.c
 *
 * Helper module for board specific I2C bus registration
 *
 * Copyright (C) 2007 Nokia Corporation.
 *
 * Contact: Jarkko Nikula <jhnikula@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-omap.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <mach/irqs.h>
#include <plat/mux.h>
#include <plat/omap-pm.h>
#include <plat/omap_device.h>

#define OMAP_I2C_SIZE		0x3f
#define OMAP1_I2C_BASE		0xfffb3800

static const char name[] = "i2c_omap";

#define I2C_RESOURCE_BUILDER(base, irq)			\
	{						\
		.start	= (base),			\
		.end	= (base) + OMAP_I2C_SIZE,	\
		.flags	= IORESOURCE_MEM,		\
	},						\
	{						\
		.start	= (irq),			\
		.flags	= IORESOURCE_IRQ,		\
	},

static struct resource i2c_resources[][2] = {
	{ I2C_RESOURCE_BUILDER(0, 0) },
};

#define I2C_DEV_BUILDER(bus_id, res, data)		\
	{						\
		.id	= (bus_id),			\
		.name	= name,				\
		.num_resources	= ARRAY_SIZE(res),	\
		.resource	= (res),		\
		.dev		= {			\
			.platform_data	= (data),	\
		},					\
	}

#define MAX_OMAP_I2C_HWMOD_NAME_LEN	16
#define OMAP_I2C_MAX_CONTROLLERS 4
static struct omap_i2c_bus_platform_data i2c_pdata[OMAP_I2C_MAX_CONTROLLERS];
static struct platform_device omap_i2c_devices[] = {
	I2C_DEV_BUILDER(1, i2c_resources[0], &i2c_pdata[0]),
};

#define OMAP_I2C_CMDLINE_SETUP	(BIT(31))

#define I2C_ICLK	0
#define I2C_FCLK	1
static struct clk *omap_i2c_clks[ARRAY_SIZE(omap_i2c_devices)][2];

static int __init omap_i2c_nr_ports(void)
{
	int ports = 0;

	if (cpu_class_is_omap1())
		ports = 1;
	else if (cpu_is_omap24xx())
		ports = 2;
	else if (cpu_is_omap34xx())
		ports = 3;
	else if (cpu_is_omap44xx())
		ports = 4;

	return ports;
}

static int omap1_i2c_device_enable(struct platform_device *pdev)
{
	struct clk *c;
	c = omap_i2c_clks[pdev->id - 1][I2C_ICLK];
	if (c && !IS_ERR(c))
		clk_enable(c);

	c = omap_i2c_clks[pdev->id - 1][I2C_FCLK];
	if (c && !IS_ERR(c))
		clk_enable(c);

	return 0;
}

static int omap1_i2c_device_idle(struct platform_device *pdev)
{
	struct clk *c;

	c = omap_i2c_clks[pdev->id - 1][I2C_FCLK];
	if (c && !IS_ERR(c))
		clk_disable(c);

	c = omap_i2c_clks[pdev->id - 1][I2C_ICLK];
	if (c && !IS_ERR(c))
		clk_disable(c);

	return 0;
}

static inline int omap1_i2c_add_bus(int bus_id)
{
	struct platform_device *pdev;
	struct omap_i2c_bus_platform_data *pdata;

	omap1_i2c_mux_pins(bus_id);

	pdev = &omap_i2c_devices[bus_id - 1];
	pdata = &i2c_pdata[bus_id - 1];

	/* idle and shutdown share the same code */
	pdata->device_enable = omap1_i2c_device_enable;
	pdata->device_idle = omap1_i2c_device_idle;
	pdata->device_shutdown = omap1_i2c_device_idle;

	omap_i2c_clks[bus_id - 1][I2C_ICLK] = clk_get(&pdev->dev, "ick");
	omap_i2c_clks[bus_id - 1][I2C_FCLK] = clk_get(&pdev->dev, "fck");

	return platform_device_register(pdev);
}

static struct omap_device_pm_latency omap_i2c_latency[] = {
	[0] = {
		.deactivate_func	= omap_device_idle_hwmods,
		.activate_func		= omap_device_enable_hwmods,
		.flags			= OMAP_DEVICE_LATENCY_AUTO_ADJUST,
	},
};

static inline int omap2_i2c_add_bus(int bus_id)
{
	int l;
	struct omap_hwmod *oh;
	struct omap_device *od;
	char oh_name[MAX_OMAP_I2C_HWMOD_NAME_LEN];
	struct omap_i2c_bus_platform_data *pdata;

	omap2_i2c_mux_pins(bus_id);

	l = snprintf(oh_name, MAX_OMAP_I2C_HWMOD_NAME_LEN, "i2c%d", bus_id);
	WARN(l >= MAX_OMAP_I2C_HWMOD_NAME_LEN,
		"String buffer overflow in I2C%d device setup\n", bus_id);
	oh = omap_hwmod_lookup(oh_name);
	if (!oh) {
			pr_err("Could not look up %s\n", oh_name);
			return -EEXIST;
	}

	pdata = &i2c_pdata[bus_id - 1];
	pdata->device_enable = omap_device_enable;
	pdata->device_idle = omap_device_idle;
	pdata->device_shutdown = omap_device_shutdown;
	/*
	 * When waiting for completion of a i2c transfer, we need to
	 * set a wake up latency constraint for the MPU. This is to
	 * ensure quick enough wakeup from idle, when transfer
	 * completes.
	 * Only omap3 has support for constraints
	 */
#if 1 /* TI Patch - kernel panic from i2c */
	if (cpu_is_omap34xx())
		pdata->set_mpu_wkup_lat = omap_pm_set_max_mpu_wakeup_lat;
#endif
	if (cpu_is_omap44xx() && (omap_rev() > OMAP4430_REV_ES1_0))
		pdata->features |= I2C_HAS_FASTMODE_PLUS;

	od = omap_device_build(name, bus_id, oh, pdata,
			sizeof(struct omap_i2c_bus_platform_data),
			omap_i2c_latency, ARRAY_SIZE(omap_i2c_latency), 0);
	WARN(IS_ERR(od), "Could not build omap_device for %s\n", name);

	return PTR_ERR(od);
}

static int __init omap_i2c_add_bus(int bus_id)
{
	if (cpu_class_is_omap1())
		return omap1_i2c_add_bus(bus_id);
	else
		return omap2_i2c_add_bus(bus_id);
}

/**
 * omap_i2c_bus_setup - Process command line options for the I2C bus speed
 * @str: String of options
 *
 * This function allow to override the default I2C bus speed for given I2C
 * bus with a command line option.
 *
 * Format: i2c_bus=bus_id,clkrate (in kHz)
 *
 * Returns 1 on success, 0 otherwise.
 */
static int __init omap_i2c_bus_setup(char *str)
{
	int ports;
	int ints[3];

	ports = omap_i2c_nr_ports();
	get_options(str, 3, ints);
	if (ints[0] < 2 || ints[1] < 1 || ints[1] > ports)
		return 0;
	i2c_pdata[ints[1] - 1].clkrate = ints[2];
	i2c_pdata[ints[1] - 1].clkrate |= OMAP_I2C_CMDLINE_SETUP;

	return 1;
}
__setup("i2c_bus=", omap_i2c_bus_setup);

/*
 * Register busses defined in command line but that are not registered with
 * omap_register_i2c_bus from board initialization code.
 */
static int __init omap_register_i2c_bus_cmdline(void)
{
	int i, err = 0;

	for (i = 0; i < ARRAY_SIZE(i2c_pdata); i++)
		if (i2c_pdata[i].clkrate & OMAP_I2C_CMDLINE_SETUP) {
			i2c_pdata[i].clkrate &= ~OMAP_I2C_CMDLINE_SETUP;
			err = omap_i2c_add_bus(i + 1);
			if (err)
				goto out;
		}

out:
	return err;
}
subsys_initcall(omap_register_i2c_bus_cmdline);

/**
 * omap_register_i2c_bus - register I2C bus with device descriptors
 * @bus_id: bus id counting from number 1
 * @clkrate: clock rate of the bus in kHz
 * @info: pointer into I2C device descriptor table or NULL
 * @len: number of descriptors in the table
 *
 * Returns 0 on success or an error code.
 */
int __init omap_register_i2c_bus(int bus_id, u32 clkrate,
			  struct omap_i2c_bus_board_data *pdata,
			  struct i2c_board_info const *info,
			  unsigned len)
{
	int err;

	BUG_ON(bus_id < 1 || bus_id > omap_i2c_nr_ports());

	if (info) {
		err = i2c_register_board_info(bus_id, info, len);
		if (err)
			return err;
	}

	if (!i2c_pdata[bus_id - 1].clkrate)
		i2c_pdata[bus_id - 1].clkrate = clkrate;

	i2c_pdata[bus_id - 1].clkrate &= ~OMAP_I2C_CMDLINE_SETUP;

	if ((pdata != NULL) && (pdata->handle != NULL)) {
		i2c_pdata[bus_id - 1].handle = pdata->handle;
		i2c_pdata[bus_id - 1].hwspinlock_lock
						= pdata->hwspinlock_lock;
		i2c_pdata[bus_id - 1].hwspinlock_unlock
						= pdata->hwspinlock_unlock;
	}

	return omap_i2c_add_bus(bus_id);
}
