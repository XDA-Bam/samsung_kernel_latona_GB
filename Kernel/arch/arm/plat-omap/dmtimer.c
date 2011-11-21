/*
 * linux/arch/arm/plat-omap/dmtimer.c
 *
 * OMAP Dual-Mode Timers
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * Tarun Kanti DebBarma <tarun.kanti@ti.com>
 * Thara Gopinath <thara@ti.com>
 *
 * dmtimer adaptation to platform_driver.
 *
 * Copyright (C) 2005 Nokia Corporation
 * OMAP2 support by Juha Yrjola
 * API improvements and OMAP2 clock framework support by Timo Teras
 *
 * Copyright (C) 2009 Texas Instruments
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <mach/hardware.h>
#include <linux/pm_runtime.h>
#include <plat/dmtimer.h>
#include <plat/omap_device.h>
#include <plat/common.h>
#include <mach/irqs.h>

/* register offsets */
#define _OMAP_TIMER_ID_OFFSET		0x00
#define _OMAP_TIMER_OCP_CFG_OFFSET	0x10
#define _OMAP_TIMER_SYS_STAT_OFFSET	0x14
#define _OMAP_TIMER_STAT_OFFSET		0x18
#define _OMAP_TIMER_INT_EN_OFFSET	0x1c
#define _OMAP_TIMER_INT_CLR_OFFSET	0x30
#define _OMAP_TIMER_WAKEUP_EN_OFFSET	0x20
#define _OMAP_TIMER_CTRL_OFFSET		0x24
#define		OMAP_TIMER_CTRL_GPOCFG		(1 << 14)
#define		OMAP_TIMER_CTRL_CAPTMODE	(1 << 13)
#define		OMAP_TIMER_CTRL_PT		(1 << 12)
#define		OMAP_TIMER_CTRL_TCM_LOWTOHIGH	(0x1 << 8)
#define		OMAP_TIMER_CTRL_TCM_HIGHTOLOW	(0x2 << 8)
#define		OMAP_TIMER_CTRL_TCM_BOTHEDGES	(0x3 << 8)
#define		OMAP_TIMER_CTRL_SCPWM		(1 << 7)
#define		OMAP_TIMER_CTRL_CE		(1 << 6) /* compare enable */
#define		OMAP_TIMER_CTRL_PRE		(1 << 5) /* prescaler enable */
#define		OMAP_TIMER_CTRL_PTV_SHIFT	2 /* prescaler value shift */
#define		OMAP_TIMER_CTRL_POSTED		(1 << 2)
#define		OMAP_TIMER_CTRL_AR		(1 << 1) /* auto-reload enable */
#define		OMAP_TIMER_CTRL_ST		(1 << 0) /* start timer */
#define _OMAP_TIMER_COUNTER_OFFSET	0x28
#define _OMAP_TIMER_LOAD_OFFSET		0x2c
#define _OMAP_TIMER_TRIGGER_OFFSET	0x30
#define _OMAP_TIMER_WRITE_PEND_OFFSET	0x34
#define		WP_NONE			0	/* no write pending bit */
#define		WP_TCLR			(1 << 0)
#define		WP_TCRR			(1 << 1)
#define		WP_TLDR			(1 << 2)
#define		WP_TTGR			(1 << 3)
#define		WP_TMAR			(1 << 4)
#define		WP_TPIR			(1 << 5)
#define		WP_TNIR			(1 << 6)
#define		WP_TCVR			(1 << 7)
#define		WP_TOCR			(1 << 8)
#define		WP_TOWR			(1 << 9)
#define _OMAP_TIMER_MATCH_OFFSET	0x38
#define _OMAP_TIMER_CAPTURE_OFFSET	0x3c
#define _OMAP_TIMER_IF_CTRL_OFFSET	0x40
#define _OMAP_TIMER_CAPTURE2_OFFSET		0x44	/* TCAR2, 34xx only */
#define _OMAP_TIMER_TICK_POS_OFFSET		0x48	/* TPIR, 34xx only */
#define _OMAP_TIMER_TICK_NEG_OFFSET		0x4c	/* TNIR, 34xx only */
#define _OMAP_TIMER_TICK_COUNT_OFFSET		0x50	/* TCVR, 34xx only */
#define _OMAP_TIMER_TICK_INT_MASK_SET_OFFSET	0x54	/* TOCR, 34xx only */
#define _OMAP_TIMER_TICK_INT_MASK_COUNT_OFFSET	0x58	/* TOWR, 34xx only */

/* register offsets with the write pending bit encoded */
#define	WPSHIFT					16

#define OMAP_TIMER_ID_REG			(_OMAP_TIMER_ID_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_OCP_CFG_REG			(_OMAP_TIMER_OCP_CFG_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_SYS_STAT_REG			(_OMAP_TIMER_SYS_STAT_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_STAT_REG			(_OMAP_TIMER_STAT_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_INT_EN_REG			(_OMAP_TIMER_INT_EN_OFFSET \
							| (WP_NONE << WPSHIFT))

#define	OMAP_TIMER_INT_CLR_REG			(_OMAP_TIMER_INT_CLR_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_WAKEUP_EN_REG		(_OMAP_TIMER_WAKEUP_EN_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_CTRL_REG			(_OMAP_TIMER_CTRL_OFFSET \
							| (WP_TCLR << WPSHIFT))

#define OMAP_TIMER_COUNTER_REG			(_OMAP_TIMER_COUNTER_OFFSET \
							| (WP_TCRR << WPSHIFT))

#define OMAP_TIMER_LOAD_REG			(_OMAP_TIMER_LOAD_OFFSET \
							| (WP_TLDR << WPSHIFT))

#define OMAP_TIMER_TRIGGER_REG			(_OMAP_TIMER_TRIGGER_OFFSET \
							| (WP_TTGR << WPSHIFT))

#define OMAP_TIMER_WRITE_PEND_REG		(_OMAP_TIMER_WRITE_PEND_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_MATCH_REG			(_OMAP_TIMER_MATCH_OFFSET \
							| (WP_TMAR << WPSHIFT))

#define OMAP_TIMER_CAPTURE_REG			(_OMAP_TIMER_CAPTURE_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_IF_CTRL_REG			(_OMAP_TIMER_IF_CTRL_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_CAPTURE2_REG			(_OMAP_TIMER_CAPTURE2_OFFSET \
							| (WP_NONE << WPSHIFT))

#define OMAP_TIMER_TICK_POS_REG			(_OMAP_TIMER_TICK_POS_OFFSET \
							| (WP_TPIR << WPSHIFT))

#define OMAP_TIMER_TICK_NEG_REG			(_OMAP_TIMER_TICK_NEG_OFFSET \
							| (WP_TNIR << WPSHIFT))

#define OMAP_TIMER_TICK_COUNT_REG		(_OMAP_TIMER_TICK_COUNT_OFFSET \
							| (WP_TCVR << WPSHIFT))

#define OMAP_TIMER_TICK_INT_MASK_SET_REG				\
		(_OMAP_TIMER_TICK_INT_MASK_SET_OFFSET | (WP_TOCR << WPSHIFT))

#define OMAP_TIMER_TICK_INT_MASK_COUNT_REG				\
		(_OMAP_TIMER_TICK_INT_MASK_COUNT_OFFSET | (WP_TOWR << WPSHIFT))

#define MAX_WRITE_PEND_WAIT		15000 /* 15ms timeout delay */

struct omap_dm_timer {
	int irq;
	struct clk *fclk;
	void __iomem *io_base;
	unsigned reserved:1;
	unsigned enabled:1;
	unsigned posted:1;
	unsigned context:1;
	struct platform_device *pdev;
	struct list_head node;
};

struct omap_timer_regs {
	u32 tidr;
	u32 tiocp_cfg;
	u32 tistat;
	u32 tisr;
	u32 tier;
	u32 twer;
	u32 tclr;
	u32 tcrr;
	u32 tldr;
	u32 ttrg;
	u32 twps;
	u32 tmar;
	u32 tcar1;
	u32 tsicr;
	u32 tcar2;
	u32 tpir; /* timers: 1, 2, 10 */
	u32 tnir; /* timers: 1, 2, 10 */
	u32 tcvr; /* timers: 1, 2, 10 */
	u32 tocr; /* timers: 1, 2, 10 */
	u32 towr; /* timers: 1, 2, 10 */
};

#define NR_DMTIMERS_MAX			12
static struct omap_timer_regs timer_context[NR_DMTIMERS_MAX];

static LIST_HEAD(omap_timer_list);
static DEFINE_SPINLOCK(dm_timer_lock);



//TI patch start - RANDOM CRASH
#define BUS_ERROR_TEST

static inline void CheckBusError(u32 Num)
{
	if( omap_readl(0x48340028) & 0x0100 ) //INBAND_ERROR_PRIMARY
	{
		printk("[BUS] L4_TA_WKUP ERROR -%d\n", Num);
	}

	return;
}
static inline void ClockCycleWait(void )
{
	int i = 0;

	for(i = 0; i < 3; i++)
	{
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		omap_readl(0x48310000);		//GPIO 1 - GPIO_REVISION
	}

}
// TI patch end - RANDOM CRASH


/**
 * omap_dm_timer_read_reg - read timer registers in posted and non-posted mode
 * @timer:      timer pointer over which read operation to perform
 * @reg:        lowest byte holds the register offset
 *
 * The posted mode bit is encoded in reg. Note that in posted mode write
 * pending bit must be checked. Otherwise a read of a non completed write
 * will produce an error.
 */
static inline u32 omap_dm_timer_read_reg(struct omap_dm_timer *timer, u32 reg)
{
	struct dmtimer_platform_data *pdata = timer->pdev->dev.platform_data;
	int i = 0;

	if (reg >= OMAP_TIMER_WAKEUP_EN_REG)
		reg += pdata->func_offset;
	else if (reg >= OMAP_TIMER_STAT_REG)
		reg += pdata->intr_offset;

//TI patch start - RANDOM CRASH
#ifdef BUS_ERROR_TEST
		CheckBusError(1);
	
		if(NULL == timer->io_base)
			printk("[BUS] ERROR!! Read IO Base is NULL\n"); //After testing block it!!!
#endif
		
		ClockCycleWait();
//TI patch end - RANDOM CRASH

#if 0
	if (timer->posted) {
		omap_test_timeout(!(readl(timer->io_base +
			((OMAP_TIMER_WRITE_PEND_REG +
			pdata->func_offset) & 0xff)) & (reg >> WPSHIFT)),
			MAX_WRITE_PEND_WAIT, i);
		WARN_ON(i == MAX_WRITE_PEND_WAIT);
	}
#else
	if (timer->posted)
		while (readl(timer->io_base + \
				((OMAP_TIMER_WRITE_PEND_REG + pdata->func_offset) & 0xff))
				& (reg >> WPSHIFT))
		{
			//TI patch start
			ClockCycleWait();
			//TI patch end
		}
#endif

//TI patch start - RANDOM CRASH
#ifdef BUS_ERROR_TEST
		CheckBusError(2);
#endif
		ClockCycleWait();
//TI patch end - RANDOM CRASH

	return readl(timer->io_base + (reg & 0xff));
}

/**
 * omap_dm_timer_write_reg - write timer registers in posted and non-posted mode
 * @timer:      timer pointer over which write operation is to perform
 * @reg:        lowest byte holds the register offset
 * @value:      data to write into the register
 *
 * The posted mode bit is encoded in reg. Note that in posted mode the write
 * pending bit must be checked. Otherwise a write on a register which has a
 * pending write will be lost.
 */
static void omap_dm_timer_write_reg(struct omap_dm_timer *timer, u32 reg,
						u32 value)
{
	struct dmtimer_platform_data *pdata = timer->pdev->dev.platform_data;
	int i = 0;

	if (reg >= OMAP_TIMER_WAKEUP_EN_REG)
		reg += pdata->func_offset;
	else if (reg >= OMAP_TIMER_STAT_REG)
		reg += pdata->intr_offset;


	
//TI patch start - RANDOM CRASH
#ifdef BUS_ERROR_TEST
		CheckBusError(3);
	
		if(NULL == timer->io_base)
			printk("[BUS] ERROR!! Read IO Base is NULL\n"); //After testing block it!!!
#endif
	
		ClockCycleWait();
//TI patch end - RANDOM CRASH

#if 0
	if (timer->posted) {
		omap_test_timeout(!(readl(timer->io_base +
			((OMAP_TIMER_WRITE_PEND_REG +
			pdata->func_offset) & 0xff)) & (reg >> WPSHIFT)),
			MAX_WRITE_PEND_WAIT, i);
		WARN_ON(i == MAX_WRITE_PEND_WAIT);
	}
#else
	if (timer->posted)
		while (readl(timer->io_base + \
				((OMAP_TIMER_WRITE_PEND_REG + pdata->func_offset) & 0xff))
				& (reg >> WPSHIFT))
			{
			//TI patch start
				ClockCycleWait();
			//TI patch end
			}
#endif

//TI patch start - RANDOM CRASH
#ifdef BUS_ERROR_TEST
			CheckBusError(4);		
#endif
			ClockCycleWait();
//TI patch end - RANDOM CRASH

	writel(value, timer->io_base + (reg & 0xff));
}

static void omap_timer_save_context(struct omap_dm_timer *timer)
{
	timer_context[timer->pdev->id].tiocp_cfg =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_OCP_CFG_REG);
	timer_context[timer->pdev->id].tistat =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_SYS_STAT_REG);
	timer_context[timer->pdev->id].tisr =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_STAT_REG);
	timer_context[timer->pdev->id].tier =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_INT_EN_REG);
	timer_context[timer->pdev->id].twer =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_WAKEUP_EN_REG);
	timer_context[timer->pdev->id].tclr =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	timer_context[timer->pdev->id].tcrr =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_COUNTER_REG);
	timer_context[timer->pdev->id].tldr =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_LOAD_REG);
	timer_context[timer->pdev->id].tmar =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_MATCH_REG);
	timer_context[timer->pdev->id].tsicr =
		omap_dm_timer_read_reg(timer, OMAP_TIMER_IF_CTRL_REG);
}

static void omap_timer_restore_context(struct omap_dm_timer *timer)
{
	omap_dm_timer_write_reg(timer, OMAP_TIMER_OCP_CFG_REG,
				timer_context[timer->pdev->id].tiocp_cfg);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_SYS_STAT_REG,
				timer_context[timer->pdev->id].tistat);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_STAT_REG,
				timer_context[timer->pdev->id].tisr);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_INT_EN_REG,
				timer_context[timer->pdev->id].tier);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_WAKEUP_EN_REG,
				timer_context[timer->pdev->id].twer);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG,
				timer_context[timer->pdev->id].tclr);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_COUNTER_REG,
				timer_context[timer->pdev->id].tcrr);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_LOAD_REG,
				timer_context[timer->pdev->id].tldr);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_MATCH_REG,
				timer_context[timer->pdev->id].tmar);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_IF_CTRL_REG,
				timer_context[timer->pdev->id].tsicr);
}

static void omap_dm_timer_prepare(struct omap_dm_timer *timer)
{
	timer->fclk = clk_get(&timer->pdev->dev, "fck");
	if (IS_ERR_OR_NULL(timer->fclk)) {
		pr_info("omap_timer: failed to acquire fclk handle.\n");
		WARN_ON(1);
		return;
	}

	omap_dm_timer_enable(timer);

	omap_dm_timer_set_source(timer, OMAP_TIMER_SRC_32_KHZ);

	/* Match hardware reset default of posted mode */
	omap_dm_timer_write_reg(timer, OMAP_TIMER_IF_CTRL_REG,
			OMAP_TIMER_CTRL_POSTED);
	timer->posted = 1;
}

struct omap_dm_timer *omap_dm_timer_request(void)
{
	struct omap_dm_timer *timer = NULL, *t;
	unsigned long flags;

	spin_lock_irqsave(&dm_timer_lock, flags);
	list_for_each_entry(t, &omap_timer_list, node) {
		if (t->reserved)
			continue;

		timer = t;
		timer->reserved = 1;
		break;
	}
	spin_unlock_irqrestore(&dm_timer_lock, flags);

	if (timer)
		omap_dm_timer_prepare(timer);
	else
		pr_debug("%s: free timer not available.\n", __func__);

	return timer;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_request);

struct omap_dm_timer *omap_dm_timer_request_specific(int id)
{
	struct omap_dm_timer *timer = NULL, *t;
	unsigned long flags;

	spin_lock_irqsave(&dm_timer_lock, flags);
	list_for_each_entry(t, &omap_timer_list, node) {
		if (t->pdev->id == id && !t->reserved) {
			timer = t;
			timer->reserved = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&dm_timer_lock, flags);

	if (timer)
		omap_dm_timer_prepare(timer);
	else
		pr_debug("%s: timer%d not available.\n", __func__, id);

	return timer;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_request_specific);

void omap_dm_timer_free(struct omap_dm_timer *timer)
{
	omap_dm_timer_disable(timer);

	clk_put(timer->fclk);

	WARN_ON(!timer->reserved);
	timer->reserved = 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_free);

void omap_dm_timer_enable(struct omap_dm_timer *timer)
{
	struct dmtimer_platform_data *pdata = timer->pdev->dev.platform_data;

	if (timer->enabled)
		return;

	if (unlikely(pdata->is_early_init)) {
		clk_enable(timer->fclk);
		timer->enabled = 1;
		return;
	}

	if (pm_runtime_get_sync(&timer->pdev->dev) < 0) {
		dev_err(&timer->pdev->dev, "%s:pm_runtime_get_sync() FAILED\n",
			__func__);
		return;
	}

	timer->enabled = 1;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_enable);

void omap_dm_timer_disable(struct omap_dm_timer *timer)
{
	struct dmtimer_platform_data *pdata = timer->pdev->dev.platform_data;

	if (!timer->enabled)
		return;

	if (unlikely(pdata->is_early_init)) {
		clk_disable(timer->fclk);
		timer->enabled = 0;
		return;
	}

	if (pm_runtime_put_sync(&timer->pdev->dev) < 0) {
		dev_err(&timer->pdev->dev, "%s:pm_runtime_put_sync() FAILED\n",
			__func__);
		return;
	}

	timer->enabled = 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_disable);

int omap_dm_timer_get_irq(struct omap_dm_timer *timer)
{
	return timer->irq;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_get_irq);

#if defined(CONFIG_ARCH_OMAP1)

/**
 * omap_dm_timer_modify_idlect_mask - Check if any running timers use ARMXOR
 * @inputmask: current value of idlect mask
 */
__u32 omap_dm_timer_modify_idlect_mask(__u32 inputmask)
{
	int i = 0;
	struct omap_dm_timer *timer = NULL;
	unsigned long flags;

	/* If ARMXOR cannot be idled this function call is unnecessary */
	if (!(inputmask & (1 << 1)))
		return inputmask;

	/* If any active timer is using ARMXOR return modified mask */
	spin_lock_irqsave(&dm_timer_lock, flags);
	list_for_each_entry(timer, &omap_timer_list, node) {
		u32 l;

		l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
		if (l & OMAP_TIMER_CTRL_ST) {
			if (((omap_readl(MOD_CONF_CTRL_1) >> (i * 2)) & 0x03) == 0)
				inputmask &= ~(1 << 1);
			else
				inputmask &= ~(1 << 2);
		}
		i++;
	}
	spin_unlock_irqrestore(&dm_timer_lock, flags);

	return inputmask;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_modify_idlect_mask);

#else

struct clk *omap_dm_timer_get_fclk(struct omap_dm_timer *timer)
{
	return timer->fclk;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_get_fclk);

__u32 omap_dm_timer_modify_idlect_mask(__u32 inputmask)
{
	BUG();

	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_modify_idlect_mask);

#endif

void omap_dm_timer_trigger(struct omap_dm_timer *timer)
{
	omap_dm_timer_write_reg(timer, OMAP_TIMER_TRIGGER_REG, 0);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_trigger);

void omap_dm_timer_start(struct omap_dm_timer *timer)
{
	u32 l;

	if (timer->pdev->id != 1 && timer->context) {
		omap_dm_timer_enable(timer);
		omap_timer_restore_context(timer);
		timer->context = 0;
	}

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (!(l & OMAP_TIMER_CTRL_ST)) {
		l |= OMAP_TIMER_CTRL_ST;
		omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	}
}
EXPORT_SYMBOL_GPL(omap_dm_timer_start);

void omap_dm_timer_stop(struct omap_dm_timer *timer)
{
	u32 l;

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (l & OMAP_TIMER_CTRL_ST) {
		l &= ~0x1;
		omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
#ifdef CONFIG_ARCH_OMAP2PLUS
		/* Readback to make sure write has completed */
		omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
		 /*
		  * Wait for functional clock period x 3.5 to make sure that
		  * timer is stopped
		  */
		udelay(3500000 / clk_get_rate(timer->fclk) + 1);
#endif
	}
	/* Ack possibly pending interrupt */
	omap_dm_timer_write_reg(timer, OMAP_TIMER_STAT_REG,
			OMAP_TIMER_INT_OVERFLOW);

	if (timer->pdev->id != 1) {
		omap_timer_save_context(timer);
		omap_dm_timer_disable(timer);
		timer->context = 1;
	}
}
EXPORT_SYMBOL_GPL(omap_dm_timer_stop);

int omap_dm_timer_set_source(struct omap_dm_timer *timer, int source)
{
	int ret = -EINVAL;
	struct dmtimer_platform_data *pdata = timer->pdev->dev.platform_data;

	if (source < 0 || source >= 3)
		return -EINVAL;

	omap_dm_timer_disable(timer);

	/* change the timer clock source */
	ret = pdata->set_timer_src(timer->pdev, source);

	omap_dm_timer_enable(timer);

	/*
	 * When the functional clock disappears, too quick writes seem
	 * to cause an abort. XXX Is this still necessary?
	 */
	__delay(300000);

	return ret;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_source);

void omap_dm_timer_set_load(struct omap_dm_timer *timer, int autoreload,
			    unsigned int load)
{
	u32 l;

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (autoreload)
		l |= OMAP_TIMER_CTRL_AR;
	else
		l &= ~OMAP_TIMER_CTRL_AR;
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_LOAD_REG, load);

	omap_dm_timer_write_reg(timer, OMAP_TIMER_TRIGGER_REG, 0);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_load);

/* Optimized set_load which removes costly spin wait in timer_start */
void omap_dm_timer_set_load_start(struct omap_dm_timer *timer, int autoreload,
                            unsigned int load)
{
	u32 l;

	if (timer->pdev->id != 1 && timer->context) {
		omap_dm_timer_enable(timer);
		omap_timer_restore_context(timer);
		timer->context = 0;
	}

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (autoreload) {
		l |= OMAP_TIMER_CTRL_AR;
		omap_dm_timer_write_reg(timer, OMAP_TIMER_LOAD_REG, load);
	} else {
		l &= ~OMAP_TIMER_CTRL_AR;
	}
	l |= OMAP_TIMER_CTRL_ST;

	omap_dm_timer_write_reg(timer, OMAP_TIMER_COUNTER_REG, load);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_load_start);

void omap_dm_timer_set_match(struct omap_dm_timer *timer, int enable,
			     unsigned int match)
{
	u32 l;

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (enable)
		l |= OMAP_TIMER_CTRL_CE;
	else
		l &= ~OMAP_TIMER_CTRL_CE;
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_MATCH_REG, match);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_match);

void omap_dm_timer_set_pwm(struct omap_dm_timer *timer, int def_on,
			   int toggle, int trigger)
{
	u32 l;

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	l &= ~(OMAP_TIMER_CTRL_GPOCFG | OMAP_TIMER_CTRL_SCPWM |
	       OMAP_TIMER_CTRL_PT | (0x03 << 10));
	if (def_on)
		l |= OMAP_TIMER_CTRL_SCPWM;
	if (toggle)
		l |= OMAP_TIMER_CTRL_PT;
	l |= trigger << 10;
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_pwm);

void omap_dm_timer_set_prescaler(struct omap_dm_timer *timer, int prescaler)
{
	u32 l;

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	l &= ~(OMAP_TIMER_CTRL_PRE | (0x07 << 2));
	if (prescaler >= 0x00 && prescaler <= 0x07) {
		l |= OMAP_TIMER_CTRL_PRE;
		l |= prescaler << 2;
	}
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_prescaler);

void omap_dm_timer_set_int_enable(struct omap_dm_timer *timer,
				  unsigned int value)
{
	omap_dm_timer_enable(timer);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_INT_EN_REG, value);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_WAKEUP_EN_REG, value);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_int_enable);

void omap_dm_timer_set_int_disable(struct omap_dm_timer *timer,
					unsigned int value)
{
	u32 l;
	struct dmtimer_platform_data *pdata = timer->pdev->dev.platform_data;

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_WAKEUP_EN_REG);
	if (pdata->timer_ip_type == OMAP_TIMER_IP_VERSION_2) {
		l |= value;
		omap_dm_timer_write_reg(timer, OMAP_TIMER_INT_CLR_REG, value);
	} else {
		l &= ~value;
		omap_dm_timer_write_reg(timer, OMAP_TIMER_INT_EN_REG, l);
	}
	omap_dm_timer_write_reg(timer, OMAP_TIMER_WAKEUP_EN_REG, l);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_int_disable);

unsigned int omap_dm_timer_read_status(struct omap_dm_timer *timer)
{
	unsigned int l;

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_STAT_REG);

	return l;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_read_status);

void omap_dm_timer_write_status(struct omap_dm_timer *timer, unsigned int value)
{
	omap_dm_timer_enable(timer);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_STAT_REG, value);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_write_status);

unsigned int omap_dm_timer_read_counter(struct omap_dm_timer *timer)
{
	unsigned int l;

	omap_dm_timer_enable(timer);
	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_COUNTER_REG);

	return l;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_read_counter);

void omap_dm_timer_write_counter(struct omap_dm_timer *timer, unsigned int value)
{
	omap_dm_timer_write_reg(timer, OMAP_TIMER_COUNTER_REG, value);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_write_counter);

int omap_dm_timers_active(void)
{
	struct omap_dm_timer *timer;

	list_for_each_entry(timer, &omap_timer_list, node) {
		if (!timer->enabled)
			continue;

		if (omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG) &
		    OMAP_TIMER_CTRL_ST) {
			return 1;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timers_active);

void omap_dm_timer_save_context(struct omap_dm_timer *timer)
{
	omap_timer_save_context(timer);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_save_context);

/**
 * omap_dm_timer_probe - probe function called for every registered device
 * @pdev:	pointer to current timer platform device
 *
 * Called by driver framework at the end of device registration for all
 * timer devices.
 */
static int __devinit omap_dm_timer_probe(struct platform_device *pdev)
{
	int ret;
	unsigned long flags;
	struct omap_dm_timer *timer;
	struct resource *mem, *irq, *ioarea;
	struct dmtimer_platform_data *pdata = pdev->dev.platform_data;

	dev_dbg(&pdev->dev, "%s:+\n", __func__);

	if (!pdata) {
		dev_err(&pdev->dev, "%s: no platform data\n", __func__);
		return -ENODEV;
	}
	/*
	 * Early timers are already registered and in list.
	 * What we need to do during second phase of probe
	 * is to assign the newly allocated/configured pdev
	 * to already registered timer->pdev. We also call
	 * pm_runtime_enable() for each device because it
	 * could not be called during early boot because
	 * pm_runtime framework was not yet up and running.
	 */
	spin_lock_irqsave(&dm_timer_lock, flags);
	list_for_each_entry(timer, &omap_timer_list, node)
		if (timer->pdev->id == pdev->id) {
			timer->pdev = pdev;
			spin_unlock_irqrestore(&dm_timer_lock, flags);
			pm_runtime_enable(&pdev->dev);
			dev_dbg(&pdev->dev, "pm_runtime ENABLED\n");
			return 0;
		}
	spin_unlock_irqrestore(&dm_timer_lock, flags);

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (unlikely(!irq)) {
		dev_err(&pdev->dev, "%s: no IRQ resource\n", __func__);
		ret = -ENODEV;
		goto err_free_pdev;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!mem)) {
		dev_err(&pdev->dev, "%s: no memory resource\n", __func__);
		ret = -ENODEV;
		goto err_free_pdev;
	}

	ioarea = request_mem_region(mem->start, resource_size(mem),
			pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "%s: region already claimed\n", __func__);
		ret = -EBUSY;
		goto err_free_pdev;
	}

	timer = kzalloc(sizeof(struct omap_dm_timer), GFP_KERNEL);
	if (!timer) {
		dev_err(&pdev->dev, "%s: no memory for omap_dm_timer\n",
			__func__);
		ret = -ENOMEM;
		goto err_release_ioregion;
	}

	timer->io_base = ioremap(mem->start, resource_size(mem));
	if (!timer->io_base) {
		dev_err(&pdev->dev, "%s: ioremap failed\n", __func__);
		ret = -ENOMEM;
		goto err_free_mem;
	}

	timer->irq = irq->start;
	timer->pdev = pdev;
	timer->reserved = 0;

	/* add the timer element to the list */
	spin_lock_irqsave(&dm_timer_lock, flags);
	list_add_tail(&timer->node, &omap_timer_list);
	spin_unlock_irqrestore(&dm_timer_lock, flags);

	dev_dbg(&pdev->dev, " bound to its driver\n");

//TI patch start : disabling "timeout" 
	printk("[BUS]L4_TA_AGENT_CONTROL_L before: 0x%x\n", omap_readl(0x48340020));
	nop();
	nop();
	omap_writel( (omap_readl(0x48340020) & ~0x700) ,0x48340020);
	nop();
	nop();
	printk("[BUS]L4_TA_AGENT_CONTROL_L after: 0x%x\n", omap_readl(0x48340020));
//TI patch end
	

	return 0;

err_free_mem:
	kfree(timer);

err_release_ioregion:
	release_mem_region(mem->start, resource_size(mem));

err_free_pdev:
	platform_device_del(pdev);

	return ret;
	}

/**
 * omap_dm_timer_remove - cleanup a registered timer device
 * @pdev:	pointer to current timer platform device
 *
 * Called by driver framework whenever a timer device is unregistered.
 * In addition to freeing platform resources it also deletes the timer
 * entry from the local list.
 */
static int __devexit omap_dm_timer_remove(struct platform_device *pdev)
{
	struct omap_dm_timer *timer, *tmp;
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&dm_timer_lock, flags);
	list_for_each_entry_safe(timer, tmp, &omap_timer_list, node) {
		if (timer->pdev->id == pdev->id) {
			platform_device_del(timer->pdev);
			list_del(&timer->node);
			kfree(timer);
			ret = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&dm_timer_lock, flags);

	return ret;
}

static struct platform_driver omap_dm_timer_driver = {
	.probe  = omap_dm_timer_probe,
	.remove = omap_dm_timer_remove,
	.driver = {
		.name   = "omap_timer",
	},
};

static int __init omap_dm_timer_driver_init(void)
{
	return platform_driver_register(&omap_dm_timer_driver);
}

static void __exit omap_dm_timer_driver_exit(void)
{
	platform_driver_unregister(&omap_dm_timer_driver);
}

early_platform_init("earlytimer", &omap_dm_timer_driver);
module_init(omap_dm_timer_driver_init);
module_exit(omap_dm_timer_driver_exit);

MODULE_DESCRIPTION("OMAP Dual-Mode Timer Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
