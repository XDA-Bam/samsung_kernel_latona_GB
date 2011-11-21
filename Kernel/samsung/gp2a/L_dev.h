/*
 * L_dev.h 
 *
 * Description: This file is an interface to L_dev.c
 *
 * Author: Varun Mahajan <m.varun@samsung.com>
 */

#ifndef _L_DEV_H
#define _L_DEV_H

/*attr: L_operation*/ 
/*output values*/
#define L_SYSFS_POLLING_ON	0
#define L_SYSFS_POLLING_OFF	1

/*input values*/
#define L_SYSFS_START_POLLING	2
#define L_SYSFS_STOP_POLLING	3

/*Function prototypes*/
/**************************************************************/
extern void L_dev_sync_mech_init(void); 

extern int L_dev_init(void);
extern int L_dev_exit(void);

extern int L_dev_suspend(void);
extern int L_dev_resume(void);

extern int L_dev_get_adc_val(u32 *);
extern int L_dev_get_illum_lvl(u16 *);
extern int L_dev_polling_start( void );
extern int L_dev_polling_stop( void );
extern int L_dev_get_op_state(void);
extern void L_dev_set_op_state(u16);
extern int L_dev_get_polling_state(void);
extern unsigned long L_dev_get_polling_interval( void );
extern void L_dev_set_polling_interval(unsigned long interval);
extern int L_dev_set_timer(u16);

/**************************************************************/

#endif

