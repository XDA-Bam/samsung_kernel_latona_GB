
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>

#include <mach/gpio.h>
#include <mach/hardware.h>
#include "mux.h"

typedef struct __OMAP_GPIO_I2C_WR_DATA {
	u8 reg_len;
	u8 *reg_addr;
	u8 wdata_len;
	u8 *wdata;
        u8 rdata;
	u8 rdata_len;	
} OMAP_GPIO_I2C_WR_DATA;

typedef struct __OMAP_GPIO_I2C_RD_DATA {
	u8 reg_len;
	u8 *reg_addr;
	u8 rdata_len;
	u8 *rdata;
} OMAP_GPIO_I2C_RD_DATA;

typedef struct __OMAP_GPIO_I2C_CLIENT {
	int sda;
	int scl;
	int addr;
	int delay;
} OMAP_GPIO_I2C_CLIENT;

extern OMAP_GPIO_I2C_CLIENT * omap_gpio_i2c_init(int /*sda*/, int /*scl*/, int/*addr*/, int/*bps*/);
extern void omap_gpio_i2c_deinit(OMAP_GPIO_I2C_CLIENT *);
extern int omap_gpio_i2c_write(OMAP_GPIO_I2C_CLIENT *, OMAP_GPIO_I2C_WR_DATA *);
extern int omap_gpio_i2c_read(OMAP_GPIO_I2C_CLIENT *, OMAP_GPIO_I2C_RD_DATA *);
