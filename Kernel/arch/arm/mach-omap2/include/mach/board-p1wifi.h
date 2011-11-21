/*
 * arch/arm/plat-omap/include/mach/board-zoom2.h
 *
 * Hardware definitions for TI OMAP3 LDP.
 *
 * Copyright (C) 2008 Texas Instruments Inc.
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
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __BOARD_P1WIFI_H__
#define __BOARD_P1WIFI_H__

extern void ldp_flash_init(void);
extern void twl4030_bci_battery_init(void);


#define	IH_TWL4030_BASE		IH_BOARD_BASE
#define	IH_TWL4030_END		(IH_TWL4030_BASE+8)

#define IH_USBIC_COUNT	3

#define  CONTROL_PADCONF_SYS_CLKOUT1_GPIO 	 10      
#define OMAP3430_GPIO_MICRO_nINT				(CONTROL_PADCONF_SYS_CLKOUT1_GPIO)
//#ifdef CONFIG_TWL4030_GPIO
#if 1 
/* TWL4030 GPIO Interrupts */
#define IH_TWL4030_GPIO_BASE	(IH_TWL4030_END)
#define IH_TWL4030_GPIO_END		(IH_TWL4030_BASE+18)
	//#define NR_IRQS			(IH_TWL4030_GPIO_END)
	#define IH_USBIC_BASE		(IH_TWL4030_GPIO_END)
	#define IH_USBIC_END		(IH_USBIC_BASE + IH_USBIC_COUNT)
	//#define NR_IRQS				(IH_USBIC_END)
#else
	//#define NR_IRQS			(IH_TWL4030_END)
	#define IH_USBIC_BASE		(IH_TWL4030_END)
	#define IH_USBIC_END		(IH_USBIC_BASE + IH_USBIC_COUNT)
//	#define NR_IRQS				(IH_USBIC_END)
#endif


#define TWL4030_IRQNUM		INT_34XX_SYS_NIRQ
#define LDP3430_NAND_CS		0 

/* LAN chip details */
#define LDP_SMC911X_CS		7
#define LDP_SMC911X_GPIO	158
#define DEBUG_BASE		0x08000000
#define OMAP34XX_ETHR_START	DEBUG_BASE
#define	OMAP34XX_MUX_MODE7	7

#define OMAP3_WAKEUP (PRCM_WAKEUP_T2_KEYPAD |\
			PRCM_WAKEUP_TOUCHSCREEN | PRCM_WAKEUP_UART)

/* SBL-Revision String */
#if defined CONFIG_SAMSUNG_EMU_HW_REV
#if (CONFIG_SAMSUNG_EMU_HW_REV > 9)
#define _M_SBL_BOARD_REV_STR(rev)	"REV "#rev"-EMUL"
#else
#define _M_SBL_BOARD_REV_STR(rev)	"REV 0"#rev"-EMUL"
#endif
#elif defined CONFIG_SAMSUNG_REL_HW_REV
#if (CONFIG_SAMSUNG_REL_HW_REV > 9)
#define _M_SBL_BOARD_REV_STR(rev)	"REV "#rev"-REAL"
#else
#define _M_SBL_BOARD_REV_STR(rev)	"REV 0"#rev"-REAL"
#endif
#endif /* CONFIG_SAMSUNG_EMU_HW_REV / CONFIG_SAMSUNG_REL_HW_REV */

#define M_SBL_BOARD_REV_STR(rev)	_M_SBL_BOARD_REV_STR(rev)

#if defined CONFIG_SAMSUNG_EMU_HW_REV
#define C_SBL_BOARD_REV \
	M_SBL_BOARD_REV_STR(CONFIG_SAMSUNG_EMU_HW_REV)
#elif defined CONFIG_SAMSUNG_REL_HW_REV
#define C_SBL_BOARD_REV \
	M_SBL_BOARD_REV_STR(CONFIG_SAMSUNG_REL_HW_REV)
#endif /* CONFIG_SAMSUNG_EMU_HW_REV / CONFIG_SAMSUNG_REL_HW_REV */

/* FIXME: old-style */
#define CONFIG_REV_STR			C_SBL_BOARD_REV

extern void __init omap_board_peripherals_init(void);
extern void __init omap_board_display_init(enum omap_dss_venc_type venc_type);


#endif /* __BOARD_P1WIFI_H__ */
