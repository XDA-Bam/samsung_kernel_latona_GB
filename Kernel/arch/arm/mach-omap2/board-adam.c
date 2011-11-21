/**
 * Copyright (C) 2010-2011, Samsung Electronics, Co., Ltd. All Rights Reserved.
 *  Written by System S/W Group, Open OS S/W R&D Team,
 *  Mobile Communication Division.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/**
 * Project Name : OMAP-Samsung Linux Kernel for Android
 *
 * Project Description :
 *
 * Comments : tabstop = 8, shiftwidth = 8, noexpandtab
 */

/**
 * File Name : board-adam.c
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 08/Feb/2011
 * Version : Baby-Raccoon
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/bootmem.h>
#include <linux/reboot.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/board.h>
#include <plat/usb.h>
#include <plat/opp_twl_tps.h>
#include <plat/control.h>
#include <plat/timer-gp.h>
#include <plat/mux.h>
#include <plat/display.h>

#include "mux.h"
#include "sdram-hynix-h8mbx00u0mer-0em.h"
#include "smartreflex-class3.h"

/* TODO: OMAP-Samsung Board-Porting Layer */
#include <mach/board-adam.h>
#include <mach/sec_log_buf.h>
#include <mach/sec_param.h>

int i = 0;

extern int set_wakeup_gpio(void);
extern int omap_gpio_out_init(void);
extern struct omap_board_mux *omap34xx_mux_ptr;

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

void (*sec_set_param_value) (int idx, void *value);
EXPORT_SYMBOL(sec_set_param_value);

void (*sec_get_param_value) (int idx, void *value);
EXPORT_SYMBOL(sec_get_param_value);

u32 hw_revision;
EXPORT_SYMBOL(hw_revision);

struct device *sio_switch_dev;
EXPORT_SYMBOL(sio_switch_dev);

#ifdef CONFIG_PM
static struct omap_volt_vc_data vc_config = {
	/* MPU */
	.vdd0_on = 1200000,	/* 1.2v */
	.vdd0_onlp = 1000000,	/* 1.0v */
	.vdd0_ret = 975000,	/* 0.975v */
	.vdd0_off = 600000,	/* 0.6v */
	/* CORE */
	.vdd1_on = 1150000,	/* 1.15v */
	.vdd1_onlp = 1000000,	/* 1.0v */
	.vdd1_ret = 975000,	/* 0.975v */
	.vdd1_off = 600000,	/* 0.6v */

	.clksetup = 0xff,
	.voltoffset = 0xff,
	.voltsetup2 = 0xff,
	.voltsetup_time1 = 0xfff,
	.voltsetup_time2 = 0xfff,
};

#ifdef CONFIG_TWL4030_CORE
static struct omap_volt_pmic_info omap_pmic_mpu = {	/* and iva */
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x0,	/* (vdd0) VDD1 -> VDD1_CORE -> VDD_MPU */
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x14,
	.vp_vlimitto_vddmax = 0x44,
};

static struct omap_volt_pmic_info omap_pmic_core = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x1,	/* (vdd1) VDD2 -> VDD2_CORE -> VDD_CORE */
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0x18,
	.vp_vlimitto_vddmax = 0x42,
};
#endif /* CONFIG_TWL4030_CORE */
#endif /* CONFIG_PM */

static void __init __sec_omap_reserve_sdram(void)
{
	u32 paddr;
	u32 size = 0x80000;
	paddr = 0x95000000;
	paddr -= size;

	if (reserve_bootmem(paddr, size, BOOTMEM_EXCLUSIVE) < 0)
		pr_err("FB: failed to reserve VRAM\n");

#ifdef CONFIG_SAMSUNG_USE_SEC_LOG_BUF
	sec_log_buf_reserve_mem();
#endif
}

static void __init omap_board_map_io(void)
{
	omap2_set_globals_36xx();
	omap34xx_map_common_io();

	__sec_omap_reserve_sdram();
}

static struct omap_board_config_kernel omap_board_sec_config[] __initdata = {
};

static int __init msecure_init(void)
{
	int ret = 0;

#ifdef CONFIG_RTC_DRV_TWL4030
	/* 3430ES2.0 doesn't have msecure/gpio-22 line connected to T2 */
	if (omap_type() == OMAP2_DEVICE_TYPE_GP) {
		void __iomem *msecure_pad_config_reg =
		    omap_ctrl_base_get() + 0x5EC;
		int mux_mask = 0x04;
		u16 tmp;

		ret = gpio_request(OMAP_GPIO_SYS_DRM_MSECURE, "msecure");
		if (ret < 0) {
			printk(KERN_ERR "msecure_init: can't"
			       "reserve GPIO:%d !\n",
			       OMAP_GPIO_SYS_DRM_MSECURE);
			goto out;
		}
		/*
		 * TWL4030 will be in secure mode if msecure line from OMAP
		 * is low. Make msecure line high in order to change the
		 * TWL4030 RTC time and calender registers.
		 */
		tmp = __raw_readw(msecure_pad_config_reg);
		tmp &= 0xF8;	/* To enable mux mode 03/04 = GPIO_RTC */
		tmp |= mux_mask;	/* To enable mux mode 03/04 = GPIO_RTC */
		__raw_writew(tmp, msecure_pad_config_reg);

		gpio_direction_output(OMAP_GPIO_SYS_DRM_MSECURE, 1);
	}
out:
#endif

	return ret;
}

char sec_androidboot_mode[16];
EXPORT_SYMBOL(sec_androidboot_mode);

static __init int setup_androidboot_mode(char *opt)
{
	strncpy(sec_androidboot_mode, opt, 15);
	return 0;
}

__setup("androidboot.mode=", setup_androidboot_mode);

u32 sec_bootmode;
EXPORT_SYMBOL(sec_bootmode);

static __init int setup_boot_mode(char *opt)
{
	sec_bootmode = (u32) memparse(opt, &opt);
	return 0;
}

__setup("bootmode=", setup_boot_mode);

#ifdef CONFIG_SAMSUNG_KERNEL_DEBUG

typedef struct {
	char Magic[4];
	char BuildRev[12];
	char BuildDate[12];
	char BuildTime[9];
	void *Excp_reserve1;
	void *Excp_reserve2;
	void *Excp_reserve3;
	void *Excp_reserve4;
} gExcpDebugInfo_t;

void debug_info_init(void)
{
	gExcpDebugInfo_t *debug_info;

	debug_info = phys_to_virt(0x95000000) - sizeof(gExcpDebugInfo_t);

	memcpy(debug_info->Magic, "DBG", 4);
	memcpy(debug_info->BuildRev, CONFIG_REV_STR, 12);
	memcpy(debug_info->BuildDate, __DATE__, 12);
	memcpy(debug_info->BuildTime, __TIME__, 9);
}
#endif

int get_hw_revision(void)
{
	return hw_revision;
}
EXPORT_SYMBOL(get_hw_revision);
	
static void __init get_board_hw_rev(void)
{
	int ret;
#if 0
	//[ changoh.heo 2010 for checing HW_REV1,Gpio 127 is special gpio.
	u32 pbias_lte = 0, wkup_ctl = 0;
	pbias_lte = omap_readl(0x48002520);	//OMAP36XX_CONTROL_PBIAS_LITE
	pbias_lte &= ~OMAP343X_PBIASLITEVMODE1;
	pbias_lte |= OMAP343X_PBIASLITEPWRDNZ1;
	omap_writel(pbias_lte, 0x48002520);

	wkup_ctl = omap_readl(0x48002a5c);	//OMAP36XX_CONTROL_WKUP_CTRL
	wkup_ctl |= OMAP36XX_PBIASGPIO_IO_PWRDNZ;
	omap_writel(wkup_ctl, 0x48002a5c);
#if 0
	pad_gpio_127 = omap_readl(0x48002a54);	//set gpio 127 pad config
	pad_gpio_127 &= 0xFFFF0000;
	pad_gpio_127 |= 0x0104;
	omap_writel(pad_gpio_127, 0x48002a54);
#endif
	//]
#endif

	ret = gpio_request(OMAP_GPIO_HW_REV0, "HW_REV0");
	if (ret < 0) {
		printk("fail to get gpio : %d, res : %d\n", OMAP_GPIO_HW_REV0,
		       ret);
		return;
	}

	ret = gpio_request(OMAP_GPIO_HW_REV1, "HW_REV1");
	if (ret < 0) {
		printk("fail to get gpio : %d, res : %d\n", OMAP_GPIO_HW_REV1,
		       ret);
		return;
	}

	gpio_direction_input(OMAP_GPIO_HW_REV0);
	gpio_direction_input(OMAP_GPIO_HW_REV1);

	hw_revision = gpio_get_value(OMAP_GPIO_HW_REV0);
	hw_revision |= (gpio_get_value(OMAP_GPIO_HW_REV1) << 1);

	gpio_free(OMAP_GPIO_HW_REV0);
	gpio_free(OMAP_GPIO_HW_REV1);

#if defined OMAP_GPIO_HW_REV2
	ret = gpio_request(OMAP_GPIO_HW_REV2, "HW_REV2");
	if (ret < 0) {
		printk("fail to get gpio : %d, res : %d\n", OMAP_GPIO_HW_REV2,
		       ret);
		return;
	}
	gpio_direction_input(OMAP_GPIO_HW_REV2);
	hw_revision |= (gpio_get_value(OMAP_GPIO_HW_REV2) << 2);
	gpio_free(OMAP_GPIO_HW_REV2);
#endif /* OMAP_GPIO_HW_REV2 */
}

static void __init omap_board_init_irq(void)
{
	omap_board_config = omap_board_sec_config;
	omap_board_config_size = ARRAY_SIZE(omap_board_sec_config);
	omap2_init_common_hw(h8mbx00u0mer0em_sdrc_params,
			     h8mbx00u0mer0em_sdrc_params);
	omap2_gp_clockevent_set_gptimer(1);
	omap_init_irq();
}

static const struct usbhs_omap_platform_data usbhs_pdata __initconst = {
	.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset = true,
	.reset_gpio_port[0] = -EINVAL,
	.reset_gpio_port[1] = 64,
	.reset_gpio_port[2] = -EINVAL,
};

struct sec_reboot_code {
	char *cmd;
	int mode;
};

static int omap_board_reboot_call(struct notifier_block *this,
				  unsigned long code, void *cmd)
{
	int mode = REBOOT_MODE_NONE;
	int temp_mode;
	int default_switchsel = 5; 
	
	struct sec_reboot_code reboot_tbl[] = {
		{"arm11_fota", REBOOT_MODE_ARM11_FOTA},
		{"arm9_fota", REBOOT_MODE_ARM9_FOTA},
		{"recovery", REBOOT_MODE_RECOVERY},
		{"download", REBOOT_MODE_DOWNLOAD},
		{"cp_crash", REBOOT_MODE_CP_CRASH}
	};
	size_t i, n;

	if ((code == SYS_RESTART) && cmd) {
		n = sizeof(reboot_tbl) / sizeof(struct sec_reboot_code);
		for (i = 0; i < n; i++) {
			if (!strcmp((char *)cmd, reboot_tbl[i].cmd)) {
				mode = reboot_tbl[i].mode;
				break;
			}
		}
	}

	if (code != SYS_POWER_OFF)
		{
		if (sec_get_param_value && sec_set_param_value)
			{
			/*in case of RECOVERY mode we set switch_sel with default value*/
			sec_get_param_value(__REBOOT_MODE, &temp_mode);
			if(temp_mode == REBOOT_MODE_RECOVERY)
				sec_set_param_value(__SWITCH_SEL, &default_switchsel);
			}
		
		/*set normal reboot_mode when reset*/	
		if (sec_set_param_value)
			sec_set_param_value(__REBOOT_MODE, &mode);
		}

	return NOTIFY_DONE;
}

static struct notifier_block omap_board_reboot_notifier = {
	.notifier_call = omap_board_reboot_call,
};

static void __init omap_board_init(void)
{
	omap3_mux_init(omap34xx_mux_ptr, OMAP_PACKAGE_CBP);
	omap_gpio_out_init();
	set_wakeup_gpio();
	msecure_init();

#ifdef CONFIG_SAMSUNG_KERNEL_DEBUG
	debug_info_init();
#endif

	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	omap_board_peripherals_init();
	omap_board_display_init(OMAP_DSS_VENC_TYPE_COMPOSITE);
	usb_uhhtll_init(&usbhs_pdata);
	sr_class3_init();

#ifdef CONFIG_PM
#ifdef CONFIG_TWL4030_CORE
	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
#endif
	omap_voltage_init_vc(&vc_config);
#endif

	register_reboot_notifier(&omap_board_reboot_notifier);

#ifdef CONFIG_SAMSUNG_USE_SEC_LOG_BUF
	sec_log_buf_init();
#endif
}

static void __init omap_board_fixup(struct machine_desc *desc,
				    struct tag *tags, char **cmdline,
				    struct meminfo *mi)
{
	mi->bank[0].start = 0x80000000;
	mi->bank[0].size = 256 * SZ_1M;	/* DDR_CS0 256MB */
	mi->bank[0].node = 0;

	mi->bank[1].start = 0x90000000;
	mi->bank[1].size = 256 * SZ_1M;	/* DDR_CS1 256MB */
	mi->bank[1].node = 0;

	mi->nr_banks = 2;
}

MACHINE_START(ADAM, "Adam Samsung Board")
    .phys_io = 0x48000000,
    .io_pg_offst = ((0xfa000000) >> 18) & 0xfffc,
    .boot_params = 0x80000100,
    .fixup = omap_board_fixup,
    .map_io = omap_board_map_io,
    .init_irq = omap_board_init_irq,
    .init_machine = omap_board_init,
    .timer = &omap_timer,
MACHINE_END
