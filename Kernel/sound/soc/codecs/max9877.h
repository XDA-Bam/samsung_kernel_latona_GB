/*
 * max9877.h  --  amp driver for max9877
 *
 * Copyright (C) 2009 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef _MAX9877_H
#define _MAX9877_H

#define CONFIG_SND_SOC_MAX9879

#define MAX9877_INPUT_MODE              0x00
#define MAX9877_SPK_VOLUME              0x01
#define MAX9877_HPL_VOLUME              0x02
#define MAX9877_HPR_VOLUME              0x03
#define MAX9877_OUTPUT_MODE             0x04

/* MAX9877_INPUT_MODE */
#define MAX9877_INB                     (1 << 4)
#define MAX9877_INA                     (1 << 5)
#define MAX9877_ZCD                     (1 << 6)

/* MAX9877_OUTPUT_MODE */
#ifdef CONFIG_SND_SOC_MAX9879
#define MAX9877_OUTMODE_MASK            (0x3f << 0)
#else
#define MAX9877_OUTMODE_MASK            (15 << 0)
#endif

#define MAX9877_OSC_MASK                (3 << 4)
#define MAX9877_OSC_OFFSET              4
#define MAX9877_BYPASS                  (1 << 6)
#define MAX9877_SHDN                    (1 << 7)

#ifdef CONFIG_SND_SOC_MAX9879
#define MAX9879_HPEN		(0x1 << 0)
#define MAX9879_RSPKEN		(0x1 << 1)
#define MAX9879_LSPKEN		(0x1 << 2)
#define MAX9879_ENA		(0x1 << 3)
#define MAX9879_ENB		(0x1 << 4)
#endif

#define INA_SPK 0
#define INA_HP 1
#define INA_SPK_HP 2
#define INB_SPK 3
#define INB_HP 4
#define INB_SPK_HP 5
#define INA_INB_SPK 6
#define INA_INB_HP 7
#define INA_INB_SPK_HP 8
#ifdef CONFIG_SND_SOC_MAX9879
#define INA_L_SPK 9
#define INA_R_SPK 10
#define INB_L_SPK 11
#define INB_R_SPK 12
#define INA_INB_L_SPK 13
#define INA_INB_R_SPK 14
#endif

#if ( defined( CONFIG_MACH_SAMSUNG_P1LITE ) && ( CONFIG_SAMSUNG_REL_HW_REV >= 1 ) ) //real 0.1
#define USE_SPK_LINE_OUT_SEL
#endif

#ifdef USE_SPK_LINE_OUT_SEL
#define SPK_LINE_OUT_SEL OMAP_GPIO_SPK_LINE_OUT
#endif

extern int max9877_add_controls(struct snd_soc_codec *codec);
extern void max9877_power_down_mode(void);
extern void max9877_ear_amp_control(int onoff);
extern void set_amp_gain_init(void);
extern void max9877_set_force_out_mode(int mode, int output);

#endif
