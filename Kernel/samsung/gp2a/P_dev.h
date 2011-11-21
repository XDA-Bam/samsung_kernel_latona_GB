
#ifndef _P_DEV_H
#define _P_DEV_H

#include <linux/i2c.h>
#include <linux/workqueue.h>

/*Proximity sensor power state*/
#define  P_SHUTDOWN           0
#define  P_OPERATIONAL        1

/*proximity sensor output*/
#define  P_OBJ_NOT_DETECTED   0
#define  P_OBJ_DETECTED       1

/*operation mode*/
#define  P_MODE_A             0  /*polling mode*/
#define  P_MODE_B             1  /*interrupt mode*/
#define  P_NOT_INITIALIZED    3

/*analog sleep func state*/
#define  P_AN_SLEEP_OFF       0
#define  P_AN_SLEEP_ON        1

/*detection cycle: Do not change these values, these are used in register
   bit operations*/
#define  P_CYC_8ms            0
#define  P_CYC_16ms           1
#define  P_CYC_32ms           2
#define  P_CYC_64ms           3
#define  P_CYC_128ms          4
#define  P_CYC_256ms          5
#define  P_CYC_512ms          6
#define  P_CYC_1024ms         7

/* Function prototypes */
extern void P_dev_sync_mech_init(void); 
extern void P_dev_an_func_state_init(u16);
extern void P_dev_detec_cyc_init(u16);

#if defined(CONFIG_INPUT_GP2A_USE_GPIO_I2C)
extern int P_dev_init(void);
#else
extern int P_dev_init(struct i2c_client *);
#endif
extern int P_dev_exit(void);

extern int P_dev_shutdown(void);
extern int P_dev_return_to_op(void);

extern int P_dev_reset(void);
extern void P_dev_check_wakeup_src(void);

extern int P_dev_get_mode(u16 *);
extern int P_dev_get_pwrstate_mode(u16 *, u16 *);

extern void P_dev_work_func(struct work_struct *);
extern int P_dev_powerup_set_op_mode(u16);
extern int P_dev_get_prox_output(u16 *);


#endif

