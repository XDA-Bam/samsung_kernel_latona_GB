	/*
	 * max97000.h  --  amp driver for max97000
	 *
	 * Copyright (C) 2009 Samsung Electronics Co.Ltd
	 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
	 *
	 *	This program is free software; you can redistribute  it and/or modify it
	 *	under  the terms of  the GNU General  Public License as published by the
	 *	Free Software Foundation;  either version 2 of the	License, or (at your
	 *	option) any later version.
	 *
	 */
	
#ifndef _MAX97000_H
#define _MAX97000_H
	
#define MAX97000_INPUT_GAIN			0x00
#define MAX97000_HP_MIXER			0x01
#define MAX97000_SPK_MIXER			0x02
#define MAX97000_HPL_GAIN			0x03
#define MAX97000_HPR_GAIN			0x04
#define MAX97000_SPK_GAIN			0x05
#define MAX97000_LIMITER			0x07
#define MAX97000_POWER				0x08
#define MAX97000_CHARGE_PUMP		0x09

	
	/* MAX97000_INPUT_GAIN */
#define MAX97000_INADIFF			(1 << 6)
#define MAX97000_INBDIFF			(1 << 7)
	
	/* MAX97000_HP_MIXER */
#define MAX97000_HPR_INA1			(1 << 0)
#define MAX97000_HPR_INA2			(1 << 1)
#define MAX97000_HPR_INB1			(1 << 2)
#define MAX97000_HPR_INB2			(1 << 3)
#define MAX97000_HPL_INA1			(1 << 4)
#define MAX97000_HPL_INA2			(1 << 5)
#define MAX97000_HPL_INB1			(1 << 6)
#define MAX97000_HPL_INB2			(1 << 7)
	
	/* MAX97000_SPK_MIXER */
#define MAX97000_SPK_INA1			(1 << 0)
#define MAX97000_SPK_INA2			(1 << 1)
#define MAX97000_SPK_INB1			(1 << 2)
#define MAX97000_SPK_INB2			(1 << 3)
	
	/* MAX97000_HPL_GAIN */
#define MAX97000_HPLM				(1 << 5)
#define MAX97000_SLEW				(1 << 6)
#define MAX97000_ZCD				(1 << 7)
	
	/* MAX97000_HPR_GAIN */
#define MAX97000_HPLM				(1 << 5)
#define MAX97000_LPGAIN			(1 << 7)
	
	/* MAX97000_SPK_GAIN */
#define MAX97000_SPKM				(1 << 6)
#define MAX97000_FFM				(1 << 7)
	
	/* MAX97000_POWER */
#define MAX97000_SWEN				(1 << 0)
#define MAX97000_HPREN				(1 << 1)
#define MAX97000_HPLEN				(1 << 2)
#define MAX97000_SPKEN				(1 << 4)
#define MAX97000_LPMODE_INA		(1 << 5)
#define MAX97000_LPMODE_INB		(1 << 6)
#define MAX97000_LPMODE_MASK		(3 << 5)
#define MAX97000_SHDN				(1 << 7)
	
#define INA_SPK 0
#define INA_HP 1
#define INA_SPK_HP 2
#define INB_SPK 3
#define INB_HP 4
#define INB_SPK_HP 5
#define INA_INB_SPK 6
#define INA_INB_HP 7
#define INA_INB_SPK_HP 8
#define INA_L_SPK 9
#define INA_R_SPK 10
#define INB_L_SPK 11
#define INB_R_SPK 12
#define INA_INB_L_SPK 13
#define INA_INB_R_SPK 14
#define OUTPUT_OFF 15

#ifdef CONFIG_SND_SOC_MAX97000_USE_GPIO_I2C
#define MAX97000_USE_GPIO_I2C 1
#endif

extern int max97000_add_controls(struct snd_soc_codec *codec);
extern void max97000_power_down_mode(void);
extern void max97000_set_force_out_mode(int mode, int output);
void set_amp_gain_init();


#endif


