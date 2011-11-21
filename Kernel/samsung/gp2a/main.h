/*
 * main.h 
 *
 * Description: This file is an interface to main.c
 *
 * Author: Varun Mahajan <m.varun@samsung.com>
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#define L_MAX_LVLS     15

#define L_NO_OF_LVLS_IDX    0
#define L_LVL1_mV_IDX       4
#define L_LVL_mV_INCR       5

extern u32 L_table []; 

extern int P_waitq_wkup_proc(void);

extern void P_enable_int(void);
extern void P_disable_int(void);

#endif

