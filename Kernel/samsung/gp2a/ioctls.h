/*
 * ioctls.h 
 *
 * Description: This file defines the macros for the IOCTL commands implemented
 * by the Proximity and Light sensor modules
 *
 * Author: Varun Mahajan <m.varun@samsung.com>
 */

#ifndef _IOCTLS_H
#define _IOCTLS_H

/*****************************************************************************/
/****************           PROXIMITY SENSOR SPECIFIC          ***************/
/*****************************************************************************/
/*magic no*/
#define P_IOC_MAGIC  0xF6

/*min seq no*/
#define P_IOC_NR_MIN 10

/*max seq no*/
#define P_IOC_NR_MAX (P_IOC_NR_MIN + 5)

/*commands*/
#define P_IOC_POWERUP_SET_MODE _IOW(P_IOC_MAGIC, (P_IOC_NR_MIN + 0), unsigned short)

#define P_IOC_WAIT_ST_CHG _IO(P_IOC_MAGIC, (P_IOC_NR_MIN + 1))

#define P_IOC_GET_PROX_OP _IOR(P_IOC_MAGIC, (P_IOC_NR_MIN + 2), unsigned short)

#define P_IOC_RESET  _IO(P_IOC_MAGIC, (P_IOC_NR_MIN + 3))

#define P_IOC_SHUTDOWN  _IO(P_IOC_MAGIC, (P_IOC_NR_MIN + 4))

#define P_IOC_RETURN_TO_OP   _IO(P_IOC_MAGIC, (P_IOC_NR_MIN + 5))

/*****************************************************************************/

/*****************************************************************************/
/******************           LIGHT SENSOR SPECIFIC          *****************/
/*****************************************************************************/
/*magic no*/
#define L_IOC_MAGIC  0xF6

/*min seq no*/
#define L_IOC_NR_MIN 20

/*max seq no*/
#define L_IOC_NR_MAX (L_IOC_NR_MIN + 3)

#define L_IOC_GET_ADC_VAL _IOR(L_IOC_MAGIC, (L_IOC_NR_MIN + 0), unsigned int)

#define L_IOC_GET_ILLUM_LVL _IOR(L_IOC_MAGIC, (L_IOC_NR_MIN + 1), unsigned short)

// 20091031 ryun for polling
#define L_IOC_POLLING_TIMER_SET _IOW(L_IOC_MAGIC, (L_IOC_NR_MIN + 2), unsigned short)

#define L_IOC_POLLING_TIMER_CANCEL _IOW(L_IOC_MAGIC, (L_IOC_NR_MIN + 3), unsigned short)

/*****************************************************************************/

#endif

