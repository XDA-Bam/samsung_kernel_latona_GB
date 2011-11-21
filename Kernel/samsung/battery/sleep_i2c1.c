#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <linux/bcd.h>
//#include "prm.h"
//#include "prm-regbits-34xx.h"

//extern struct clk i2c1_fck;
//extern struct clk i2c1_ick;

//#define ALL_TIME_REGS		6
////////////////////////////////////////////////////////////////
// I2C1
////////////////////////////////////////////////////////////////
#define I2C1_REV       *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x00))
#define I2C1_IE        *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x04))
#define I2C1_STAT      *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x08))
#define I2C1_SYSS      *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x10))
#define I2C1_BUF       *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x14))
#define I2C1_CNT       *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x18))
#define I2C1_DATA      *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x1C))
#define I2C1_SYSC      *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x20))
#define I2C1_CON       *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x24))
#define I2C1_OA0       *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x28))   // OMAP2430 only
#define I2C1_SA        *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x2C))
#define I2C1_PSC       *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x30))
#define I2C1_SCLL      *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x34))
#define I2C1_SCLH      *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x38))
#define I2C1_SYSTEST   *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x3C))
#define I2C1_BUFSTAT   *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x40))  // OMAP2430 only
#define I2C1_OA1       *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x44)) // OMAP2430 only
#define I2C1_OA2       *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x48))  // OMAP2430 only
#define I2C1_OA3       *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x4C))  // OMAP2430 only
#define I2C1_ACTOA     *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x50))  // OMAP2430 only
#define I2C1_SBLOCK    *(volatile unsigned short *)(OMAP2_L4_IO_ADDRESS(0x48070000 + 0x54))  // OMAP2430 only

extern int clk_enable(struct clk *clk);
extern void clk_disable(struct clk *clk);
extern struct clk *clk_get_sys(const char *dev_id, const char *con_id);

static void flush_fifo(void);
static u32 wait_for_bb(void);
static u16 wait_for_pin(void);

struct clk *p_i2c1_fck;
struct clk *p_i2c1_ick;

void twl4030_i2c_set_clocks(void)
{
	/*Standard Mode 100Khz*/
	I2C1_PSC = 0x17;		
	//I2C1_PSC = 0x9;		
	I2C1_SCLL = 0x0D;	
	I2C1_SCLH = 0x0F;	
	//I2C1_SCLL = 0x06;	
	//I2C1_SCLH = 0x06;	
}

int twl4030_i2c_check_status (u16 usVal)
{
	int fRet = 0;

	if (usVal & 0x0001)
	{
		// Clear the bit
		I2C1_STAT |= 0x0001;
		fRet = -1;
	}

	// No-acknowledge. bit 1
	if (usVal & 0x0002)
	{
		// Clear the bit
		I2C1_STAT |= 0x0002;
		fRet = -1;
	}

	if (usVal & 0x0020)
	{
	}

	// Address as a slave. bit 9
	if (usVal & 0x0200)
	{
	}

	// Transmit underrun. bit 10
	if (usVal & 0x0400)
		fRet = -1;

	// Receiving overrun. bit 11
	if (usVal & 0x0800)
		fRet = -1;

	return fRet;
}
void twl4030_i2c_init(void)
{
	p_i2c1_fck = clk_get_sys("i2c_omap.1", "fck");
	p_i2c1_ick = clk_get_sys("i2c_omap.1", "ick");
	
	clk_enable(p_i2c1_fck);
	clk_enable(p_i2c1_ick);

	I2C1_SYSC |= 1<<1;
	I2C1_CON = 0x8000;    //I2C1_EN
	while( !(I2C1_SYSS & 0x1));
	I2C1_CON = 0x0;    //I2C1_EN
	I2C1_OA0 = 0x00;
	// set up i2c clock registers

	twl4030_i2c_set_clocks();

	// configure own address

	// take the I2C module out of reset
	I2C1_CON = 0x8000;    //I2C1_EN
	I2C1_IE = 0x00;
}
void twl4030_i2c_disinit(void)
{

	/*added by sheom : make up i2c error*/
	I2C1_CON = 0x0;  
	I2C1_SYSC |= 1<<1;
	I2C1_CON = 0x8000;
	while( !(I2C1_SYSS & 0x1));
	I2C1_CON = 0x0;  

	clk_disable(p_i2c1_fck);
	clk_disable(p_i2c1_ick);
}

#define OMAP_I2C_STAT_XUDF	(1 << 10)	/* Transmit underflow */
#define OMAP_I2C_STAT_ROVR	(1 << 11)	/* Receive overrun */
#define OMAP_I2C_STAT_NACK	(1 << 1)	/* No ack interrupt enable */
#define OMAP_I2C_STAT_AL	(1 << 0)	/* Arbitration lost int ena */
#define OMAP_I2C_BUF_TXFIF_CLR	(1 << 6)	/* TX FIFO Clear */
int t2_wait_for_xudf (void)
{
	u16 xudf;
	int counter = 500;

	/* We are in interrupt context. Wait for XUDF for approx 7 msec */
	xudf = I2C1_STAT;
	while (!(xudf & OMAP_I2C_STAT_XUDF) && counter--) {
		if (xudf & (OMAP_I2C_STAT_ROVR | OMAP_I2C_STAT_NACK |
			    OMAP_I2C_STAT_AL))
			return -EINVAL;
		udelay(10);
		xudf = I2C1_STAT;
	}

	if (!counter) {
		/* Clear Tx FIFO */
		I2C1_BUF=OMAP_I2C_BUF_TXFIF_CLR;
		return -ETIMEDOUT;
	}

	return 0;
}
s32 t2_read(u8 devaddr, u8 *value, u8 regoffset)
{
	int error = 0;
	//int try_cnt = 10000;
	u16 status;

	/* wait until bus not busy */
	error = wait_for_bb();
	if(error)
		return error;

	/* set slave address */
	I2C1_SA = devaddr;

	I2C1_CNT = 1;

	I2C1_BUF |= (1<<6 | 1<<14);
	/* one byte only */
	/* no stop bit needed here */
	I2C1_CON = 0x8603;

	status = wait_for_pin();

	if (status & 0x10 /*I2C_STAT_XRDY*/) {
		/* Important: have to use byte access */
		if(t2_wait_for_xudf ())
			return -1;

		I2C1_DATA = regoffset;
		mdelay(2);

		if ( I2C1_STAT & 0x02){
			error = 1;
		}
	} else {
		error = 1;
	}

	if (!error) {
		int err = 10000;
		/* free bus, otherwise we can't use a combined transction */
	//	I2C1_CON = 0;

		while ((I2C1_STAT) || (I2C1_CON & 0x400)) {
			udelay(10);
			/* Have to clear pending interrupt to clear I2C_STAT */
			I2C1_STAT = 0xffff;
			if (!err--) {
				break;
			}
		}

		error = wait_for_bb();
		if(error)
			return error;
		/* set slave address */
		I2C1_SA = devaddr;

		/* read one byte from slave */
		I2C1_CNT = 1;

		/* need stop bit here */
		I2C1_CON = 0x8403;

		status = wait_for_pin();
		if (status & 0x08/*I2C_STAT_RRDY*/) {

			*value = I2C1_DATA;

			udelay(300);
			//udelay(100);
		} else {
			error = 1;
		}

		if (!error) {
			int err = 10000;
			I2C1_CON = 0x8000;

			while ( I2C1_STAT || (I2C1_CON & 0x400)){
				I2C1_STAT = 0xffff;
				udelay(10);
				if (!err--) {
					break;
				}
			}
		}
	}
	flush_fifo();
	I2C1_STAT = 0xffff;
	I2C1_CNT = 0;
	return error;
}
EXPORT_SYMBOL(t2_read);

//yul
s32 t2_read_word(u8 devaddr, u8 regoffset, u8 *value)
{
	int error = 0;
	//int try_cnt = 10000;
	u16 status;

	/* wait until bus not busy */
	error = wait_for_bb();
	if(error)
		return error;

	/* set slave address */
	I2C1_SA = devaddr;

	I2C1_CNT = 1;

	I2C1_BUF |= (1<<6 | 1<<14);
	/* one byte only */
	/* no stop bit needed here */
	I2C1_CON = 0x8603;

	status = wait_for_pin();

	if (status & 0x10 /*I2C_STAT_XRDY*/) {
		/* Important: have to use byte access */
		if(t2_wait_for_xudf ())
			return -1;

		I2C1_DATA = regoffset;
		mdelay(2);
      I2C1_DATA;

		if ( I2C1_STAT & 0x02){
			error = 1;
		}
	} else {
		error = 1;
	}

	if (!error) {
		int err = 10000;
		/* free bus, otherwise we can't use a combined transction */
	//	I2C1_CON = 0;

		while ((I2C1_STAT) || (I2C1_CON & 0x400)) {
			udelay(10);
			/* Have to clear pending interrupt to clear I2C_STAT */
			I2C1_STAT = 0xffff;
			if (!err--) {
				break;
			}
		}

		error = wait_for_bb();
		if(error)
			return error;
		/* set slave address */
		I2C1_SA = devaddr;

		/* read one byte from slave */
		I2C1_CNT = 3;

		/* need stop bit here */
		I2C1_CON = 0x8403;

		status = wait_for_pin();
		if (status & 0x08/*I2C_STAT_RRDY*/) {

			value[0] = I2C1_DATA;
			value[1] = I2C1_DATA;
			//mdelay(2);
			udelay(300);

		} else {
			error = 1;
		}

		if (!error) {
			int err = 10000;
			I2C1_CON = 0x8000;

			while ( I2C1_STAT || (I2C1_CON & 0x400)){
				udelay(10);
            I2C1_STAT = 0xffff;
				if (!err--) {
					break;
				}
			}
		}
	}
	flush_fifo();
	I2C1_STAT = 0xffff;
	I2C1_CNT = 0;
	return error;
}
EXPORT_SYMBOL(t2_read_word);


s32 t2_write(u8 devaddr, u8 value, u8 regoffset)
{
	int error = 0;
	u16 status, stat;

	/* wait until bus not busy */
	error =wait_for_bb();
	if(error)
		return error;

	/* set slave address */
	I2C1_SA = devaddr;

	/* two bytes */
	I2C1_CNT = 2;


	I2C1_BUF |= (1<<6 | 1<<14);
	/* stop bit needed here */
	//     I2C_CON_MST | I2C_CON_STT | I2C_CON_TRX | I2C_CON_STP, I2C1_CON);
	I2C1_CON |= 0x8603;

	/* wait until state change */
	status = wait_for_pin();

	if (status & 0x10 ) /*I2C_STAT_XRDY*/
	{
		if(t2_wait_for_xudf ())
			return -1;
		/* send out 1 byte */
		I2C1_DATA = regoffset;

		I2C1_STAT = 0x0010;

		status = wait_for_pin();

		if ((status & 0x10/*I2C_STAT_XRDY*/)) {
			/* send out next 1 byte */
			if(t2_wait_for_xudf())
				return -1;
			I2C1_DATA = value;

			I2C1_STAT = 0x0010;
		} else {
			error = 1;
		}

		/* must have enough delay to allow BB bit to go low */
		//mdelay(1);
		mdelay(1);
		if ( I2C1_STAT & 0x02){
			error = 1;
		}
	}
	else
	{
		error = 1;
	}

	if (!error)
	{
		int eout = 200;

		I2C1_CON = 0x8000;
		while ((stat = I2C1_STAT) || (I2C1_CON & 0x400)){
			//mdelay(1);
			mdelay(1);
			/* have to read to clear intrrupt */
			I2C1_STAT = 0xffff;
			if (--eout == 0)	/* better leave with error than hang */
				break;
		}
	}
	flush_fifo();
	I2C1_STAT = 0xffff;
	I2C1_CNT = 0;
	return error;
}
EXPORT_SYMBOL(t2_write);

static void flush_fifo(void)
{
	u16 stat;
	u8 data;

	/* note: if you try and read data when its not there or ready  you get a bus error */
	while (1) {
		stat = I2C1_STAT;
		if (stat == 0x08 /*I2C_STAT_RRDY*/) {
			data = I2C1_DATA;
			I2C1_STAT = 0x08;
			udelay(10);
		} else
			break;
	}
}
static u32 wait_for_bb(void)
{
	int timeout = 10000;
	u16 stat;

	I2C1_STAT = 0xffff;

	while (((stat = I2C1_STAT) & 0x1000) && timeout--)
	{
		I2C1_STAT = stat;
		udelay(10);
	}
	if (timeout <= 0) {
		printk(KERN_ERR"[%s] : I2C1_STAT=0x%x\n", __FUNCTION__, I2C1_STAT);
		return -1;
	}

	return 0;
}
static u16 wait_for_pin(void)
{
	u16 status;
	int timeout = 10000;

	do {
		udelay(10);
		status = I2C1_STAT;
	} while( !(status & 0xc1f) && timeout--);

	if (timeout <= 0) {
		printk(KERN_ERR"%s :  I2C_STAT=0x%x\n", __FUNCTION__, I2C1_STAT);
		I2C1_STAT = 0xffff;
	}
	return status;
}
u8 keypad_suspend_state[3];
u32 twl4030_check_keypress(void)
{
	u32 key_scan=0;
	u8  state=0;
	t2_read(0x49, &state, 0x81);

	if(state & (1<<1))
	{
		t2_read(0x4A,&keypad_suspend_state[0] ,0xDB ); /*FULL_CODE_7_0  */
		t2_read(0x4A,&keypad_suspend_state[1] ,0xDC ); /*FULL_CODE_15_8 */
		t2_read(0x4A,&keypad_suspend_state[2] ,0xDD ); /*FULL_CODE_23_16*/
		key_scan = ((keypad_suspend_state[0]<<0) | (keypad_suspend_state[1]<<8) | (keypad_suspend_state[2]<<16));
		printk(KERN_ERR"[%s] key_scan=0x%0x\n", __FUNCTION__, key_scan);
	}
	return 0;

}
