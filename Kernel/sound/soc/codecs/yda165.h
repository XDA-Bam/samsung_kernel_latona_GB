/*
 * yda165.h  --  amp driver for yda165
 *
 * Copyright (C) 2009 Samsung Electronics Co.Ltd
 *
 *	This program is free software; you can redistribute  it and/or modify it
 *	under  the terms of  the GNU General  Public License as published by the
 *	Free Software Foundation;  either version 2 of the	License, or (at your
 *	option) any later version.
 *
 */
	
#ifndef _YDA165_H
#define _YDA165_H

#define YDA165_RESET_BADR	        0x80
	
#define YDA165_RESET		        0x00
#define YDA165_HS_GAIN  		0x01
#define YDA165_DPLT     		0x02
#define YDA165_SP_GAIN			0x03
#define YDA165_INPUT_GAIN		0x04
#define YDA165_SPATT			0x05
#define YDA165_HPATT			0x06
#define YDA165_MIXER			0x07
#define YDA165_ERR		        0x08
	
	/* YDA165_SRST */
#define YDA165_VLEVEL			(1 << 0)
#define YDA165_CPMOD			(1 << 6)
#define YDA165_SRST			(1 << 7)
	
	/* YDA165_HS_GAIN */
#define YDA165_HP_GAIN		        (1 << 0)
#define YDA165_HIZ_SP		        (1 << 2)
#define YDA165_HIZ_HP		        (1 << 3)
#define YDA165_DIFB			(1 << 4)
#define YDA165_DIFA			(1 << 5)
#define YDA165_ECO_MOE                  (1 << 6)
	
	/* YDA165_DPLT */
#define YDA165_DARTRT		        (1 << 0)
#define YDA165_NG_ATRT		        (1 << 2)
#define YDA165_DPLT			(1 << 4)
	
	/* YDA165_SP_GAIN */
#define YDA165_SP_GAIN			(1 << 0)
#define YDA165_DALC			(1 << 2)
#define YDA165_NG_RATIO			(1 << 5)
	
	/* YDA165_INPUT_GAIN */
#define YDA165_VB			(1 << 0)
#define YDA165_VA			(1 << 4)
	
	/* YDA165_SPATT */
#define YDA165_SPATT			(1 << 0)
#define YDA165_SPZCSOFF			(1 << 6)
#define YDA165_SPSVOFF			(1 << 7)
	
	/* YDA165_HPATT */
#define YDA165_HPATT			(1 << 0)
#define YDA165_HPZCSOFF			(1 << 6)
#define YDA165_HPSVOFF			(1 << 7)

	/* YDA165_MIXER */
#define YDA165_HP_BMIX			(1 << 0)
#define YDA165_HP_AMIX			(1 << 1)
#define YDA165_HP_MONO			(1 << 3)
#define YDA165_SP_BMIX			(1 << 4)
#define YDA165_SP_AMIX			(1 << 5)

	/* YDA165_ERR */
#define YDA165_OCP_ERR			(1 << 0)
#define YDA165_OTP_ERR			(1 << 6)
#define YDA165_DC_ERR			(1 << 7)

/* INA = LIN2, RIN2, INB = LIN1, RIN2 */
#define INA_SPK         0
#define INA_HP          1
#define INA_SPK_HP      2
#define INB_SPK         3
#define INB_HP          4
#define INB_SPK_HP      5
#define INA_INB_SPK     6
#define INA_INB_HP      7
#define INA_INB_SPK_HP  8
#define INA_L_SPK       9
#define INA_R_SPK       10
#define INB_L_SPK       11
#define INB_R_SPK       12
#define INA_INB_L_SPK   13
#define INA_INB_R_SPK   14
#define OUTPUT_OFF      15

#ifdef CONFIG_SND_SOC_YDA165_USE_GPIO_I2C
#define YDA165_USE_GPIO_I2C 1
#endif

extern int yda165_add_controls(struct snd_soc_codec *codec);
extern void yda165_power_down_mode(void);
extern void yda165_set_force_out_mode(int mode, int output);
void set_amp_gain_init();


#endif


