/* linux/arch/arm/mach-omap2/wl127x_wifi.c
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wifi_tiwlan.h>

#include "wl127x_wifi.h"
#include "mux.h"

static int wl127x_wifi_cd;		/* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

void config_wlan_mux(void)
{
	/* WLAN PW_EN and IRQ */
	omap_mux_init_gpio(WL127X_WIFI_PMENA_GPIO, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(WL127X_WIFI_IRQ_GPIO, OMAP_PIN_INPUT);

	/* MMC3 */
	omap_mux_init_signal("etk_clk.sdmmc3_clk", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("mcspi1_cs1.sdmmc3_cmd", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("etk_d4.sdmmc3_dat0", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("etk_d5.sdmmc3_dat1", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("etk_d6.sdmmc3_dat2", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("etk_d3.sdmmc3_dat3", OMAP_PIN_INPUT_PULLUP);
}

int omap_wifi_status_register(void (*callback)(int card_present,
						void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

int omap_wifi_status(int irq)
{
	return wl127x_wifi_cd;
}

int wl127x_wifi_set_carddetect(int val)
{
	pr_info("%s: %d\n", __func__, val);
	wl127x_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_info("%s: Nobody to notify\n", __func__);
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(wl127x_wifi_set_carddetect);
#endif

static int wl127x_wifi_power_state;

int wl127x_wifi_power(int on)
{
	pr_info("%s: %d\n", __func__, on);
	gpio_set_value(WL127X_WIFI_PMENA_GPIO, on);
	wl127x_wifi_power_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(wl127x_wifi_power);
#endif

static int wl127x_wifi_reset_state;
int wl127x_wifi_reset(int on)
{
	pr_info("%s: %d\n", __func__, on);
	wl127x_wifi_reset_state = on;
	return 0;
}
#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(wl127x_wifi_reset);
#endif

struct wifi_platform_data wl127x_wifi_control = {
	.set_power	= wl127x_wifi_power,
	.set_reset	= wl127x_wifi_reset,
	.set_carddetect	= wl127x_wifi_set_carddetect,
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct resource wl127x_wifi_resources[] = {
	[0] = {
		.name		= "device_wifi_irq",
		.start		= OMAP_GPIO_IRQ(WL127X_WIFI_IRQ_GPIO),
		.end		= OMAP_GPIO_IRQ(WL127X_WIFI_IRQ_GPIO),
		.flags		= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device wl127x_wifi_device = {
	.name		= "device_wifi",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(wl127x_wifi_resources),
	.resource	= wl127x_wifi_resources,
	.dev		= {
		.platform_data = &wl127x_wifi_control,
	},
};
#endif

static int __init wl127x_wifi_init(void)
{
	int ret;

	pr_info("%s: start\n", __func__);
	ret = gpio_request(WL127X_WIFI_IRQ_GPIO, "wifi_irq");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			WL127X_WIFI_IRQ_GPIO);
		goto out;
	}
	ret = gpio_request(WL127X_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			WL127X_WIFI_PMENA_GPIO);
		gpio_free(WL127X_WIFI_IRQ_GPIO);
		goto out;
	}
	gpio_direction_input(WL127X_WIFI_IRQ_GPIO);
	gpio_direction_output(WL127X_WIFI_PMENA_GPIO, 0);
#ifdef CONFIG_WIFI_CONTROL_FUNC
	ret = platform_device_register(&wl127x_wifi_device);
#endif
out:
	return ret;
}

device_initcall(wl127x_wifi_init);
