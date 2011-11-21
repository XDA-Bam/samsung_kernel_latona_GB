
#ifndef _P_REGS_H
#define _P_REGS_H

/*Proximity sensor registers*/
/**************************************************************/
#define NUM_OF_REGS     0x07

#define PROX            0x00
#define GAIN            0x01
#define HYS             0x02
#define CYCLE           0x03
#define OPMOD           0x04
#define CON             0x06

/*PROX bits*/
/**************************************************************/
#define PROX_VO         (1 << 0)

/*GAIN bits*/
/**************************************************************/
#define GAIN_LED0       (1 << 3)

/*HYS bits*/
/**************************************************************/
#define HYS_HYSD        (1 << 7)
#define HYS_HYSC1       (1 << 6) 
#define HYS_HYSC0       (1 << 5)
#define HYS_HYSF3       (1 << 3) 
#define HYS_HYSF2       (1 << 2)
#define HYS_HYSF1       (1 << 1)
#define HYS_HYSF0       (1 << 0)

/*CYCLE bits*/
/**************************************************************/
#define CYCLE_CYCL2     (1 << 5)
#define CYCLE_CYCL1     (1 << 4) 
#define CYCLE_CYCL0     (1 << 3)
#define CYCLE_CYCL_MASK (CYCLE_CYCL2|CYCLE_CYCL1|CYCLE_CYCL0)
#define CYCLE_OSC       (1 << 2)

/*OPMODE bits*/
/**************************************************************/
#define OPMOD_ASD      (1 << 4)
#define OPMOD_VCON     (1 << 1)
#define OPMOD_SSD      (1 << 0)

/*CON bits*/
/**************************************************************/
#define CON_OCON1       (1 << 4)
#define CON_OCON0       (1 << 3)

/*INT clear bit*/
#define INT_CLEAR    (1 << 7)

/**************************************************************/

/*************************************************************/
static inline u8 INT_CLR(u8 reg)
{
	return (reg | INT_CLEAR);
}

static inline u8 NOT_INT_CLR(u8 reg)
{
	return (reg & ~(INT_CLEAR));
}
/**************************************************************/

#endif

