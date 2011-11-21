/*
 * Copyright (C) 2009 Nokia
 * Copyright (C) 2009-2010 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_ARCH_OMAP2420
#include "mux2420.h"
#include "mux2430.h"
#endif

#ifdef CONFIG_ARCH_OMAP3
#include "mux34xx.h"
#endif

#ifdef CONFIG_ARCH_OMAP4
#include "mux44xx.h"
#endif

#define OMAP_MUX_TERMINATOR	0xffff

/* 34xx/44xx mux mode options for each pin. See TRM for options */
#define OMAP_MUX_MODE_MASK  0x7
#define OMAP_MUX_MODE0      0
#define OMAP_MUX_MODE1      1
#define OMAP_MUX_MODE2      2
#define OMAP_MUX_MODE3      3
#define OMAP_MUX_MODE4      4
#define OMAP_MUX_MODE5      5
#define OMAP_MUX_MODE6      6
#define OMAP_MUX_MODE7      7

/* 24xx/34xx mux bit defines */
#define OMAP_PULL_ENA			(1 << 3)
#define OMAP_PULL_UP			(1 << 4)
#define OMAP_ALTELECTRICALSEL		(1 << 5)

/* 34xx/44xx mux bit defines */
#define OMAP_INPUT_EN			(1 << 8)
#define OMAP_OFF_EN			(1 << 9)
#define OMAP_OFFOUT_EN			(1 << 10)
#define OMAP_OFFOUT_VAL			(1 << 11)
#define OMAP_OFF_PULL_EN		(1 << 12)
#define OMAP_OFF_PULL_UP		(1 << 13)
#define OMAP_WAKEUP_EN			(1 << 14)

/* 44xx specific mux bit defines */
#define OMAP_WAKEUP_EVENT		(1 << 15)

/* Active pin states */
#define OMAP_PIN_OUTPUT			0
#define OMAP_PIN_INPUT			OMAP_INPUT_EN
#define OMAP_PIN_INPUT_PULLUP		(OMAP_PULL_ENA | OMAP_INPUT_EN \
						| OMAP_PULL_UP)
#define OMAP_PIN_INPUT_PULLDOWN		(OMAP_PULL_ENA | OMAP_INPUT_EN)


/* Off mode states */
#define OMAP_PIN_OFF_NONE		0
#define OMAP_PIN_OFF_OUTPUT_HIGH	(OMAP_OFF_EN | OMAP_OFFOUT_VAL)
#define OMAP_PIN_OFF_OUTPUT_LOW		(OMAP_OFF_EN)
#define OMAP_PIN_OFF_INPUT_PULLUP	(OMAP_OFF_EN | OMAP_OFFOUT_EN \
						| OMAP_OFF_PULL_EN \
						| OMAP_OFF_PULL_UP)
#define OMAP_PIN_OFF_INPUT_PULLDOWN	(OMAP_OFF_EN | OMAP_OFFOUT_EN \
						| OMAP_OFF_PULL_EN)
#define OMAP_PIN_OFF_WAKEUPENABLE	OMAP_WAKEUP_EN

#define OMAP_MODE_GPIO(x)	(((x) & OMAP_MUX_MODE7) == OMAP_MUX_MODE4)

/* Flags for omapX_mux_init */
#define OMAP_PACKAGE_MASK		0xffff
#define OMAP_PACKAGE_CBS		8		/* 547-pin 0.40 0.40 */
#define OMAP_PACKAGE_CBL		7		/* 547-pin 0.40 0.40 */
#define OMAP_PACKAGE_CBP		6		/* 515-pin 0.40 0.50 */
#define OMAP_PACKAGE_CUS		5		/* 423-pin 0.65 */
#define OMAP_PACKAGE_CBB		4		/* 515-pin 0.40 0.50 */
#define OMAP_PACKAGE_CBC		3		/* 515-pin 0.50 0.65 */
#define OMAP_PACKAGE_ZAC		2		/* 24xx 447-pin POP */
#define OMAP_PACKAGE_ZAF		1		/* 2420 447-pin SIP */


#define OMAP_MUX_NR_MODES		8		/* Available modes */
#define OMAP_MUX_NR_SIDES		2		/* Bottom & top */

/*
 * omap_mux_init flags definition:
 *
 * OMAP_MUX_REG_8BIT: Ensure that access to padconf is done in 8 bits.
 * The default value is 16 bits.
 * OMAP_MUX_GPIO_IN_MODE3: The GPIO is selected in mode3.
 * The default is mode4.
 */
#define OMAP_MUX_REG_8BIT		(1 << 0)
#define OMAP_MUX_GPIO_IN_MODE3		(1 << 1)

/**
 * struct mux_partition - contain partition related information
 * @name: name of the current partition
 * @flags: flags specific to this partition
 * @phys: physical address
 * @size: partition size
 * @base: virtual address after ioremap
 * @muxmodes: list of nodes that belong to a partition
 * @node: list node for the partitions linked list
 */
struct omap_mux_partition {
	const char		*name;
	u32			flags;
	u32			phys;
	u32			size;
	void __iomem		*base;
	struct list_head	muxmodes;
	struct list_head	node;
};

/**
 * struct omap_mux - data for omap mux register offset and it's value
 * @reg_offset:	mux register offset from the mux base
 * @gpio:	GPIO number
 * @id:		Partition identifier
 * @muxnames:	available signal modes for a ball
 */
struct omap_mux {
	u16	reg_offset;
	u16	gpio;
#ifdef CONFIG_OMAP_MUX
	char	*muxnames[OMAP_MUX_NR_MODES];
#ifdef CONFIG_DEBUG_FS
	char	*balls[OMAP_MUX_NR_SIDES];
	struct omap_mux_partition *partition;
#endif
#endif
};

/**
 * struct omap_ball - data for balls on omap package
 * @reg_offset:	mux register offset from the mux base
 * @id:		Partition identifier
 * @balls:	available balls on the package
 */
struct omap_ball {
	u16	reg_offset;
	char	*balls[OMAP_MUX_NR_SIDES];
};

/**
 * struct omap_board_mux - data for initializing mux registers
 * @reg_offset:	mux register offset from the mux base
 * @mux_value:	desired mux value to set
 * @id:		Partition identifier
 */
struct omap_board_mux {
	u16	reg_offset;
	u16	value;
};

#if defined(CONFIG_OMAP_MUX)

/**
 * omap_mux_init_gpio - initialize a signal based on the GPIO number
 * @gpio:		GPIO number
 * @val:		Options for the mux register value
 */
int omap_mux_init_gpio(int gpio, int val);

/**
 * omap_mux_init_signal - initialize a signal based on the signal name
 * @muxname:		Mux name in mode0_name.signal_name format
 * @val:		Options for the mux register value
 */
int omap_mux_init_signal(const char *muxname, int val);
u16 omap_mux_read_signal(const char *muxname);
int omap_mux_enable_wakeup(const char *muxname);
int omap_mux_disable_wakeup(const char *muxname);

#else

static inline int omap_mux_init_gpio(int gpio, int val)
{
	return 0;
}
static inline int omap_mux_init_signal(char *muxname, int val)
{
	return 0;
}
static inline int omap_mux_enable_wakeup(char *muxname)
{
	return 0;
}

#endif

/**
 * omap_mux_get_gpio() - get mux register value based on GPIO number
 * @gpio:		GPIO number
 *
 */
u16 omap_mux_get_gpio(int gpio);

/**
 * omap_mux_set_gpio() - set mux register value based on GPIO number
 * @val:		New mux register value
 * @gpio:		GPIO number
 *
 */
void omap_mux_set_gpio(u16 val, int gpio);

/**
 * omap2420_mux_init() - initialize mux system with board specific set
 * @board_mux:		Board specific mux table
 * @flags:		OMAP package type used for the board
 */
int omap2420_mux_init(struct omap_board_mux *board_mux, int flags);

/**
 * omap2430_mux_init() - initialize mux system with board specific set
 * @board_mux:		Board specific mux table
 * @flags:		OMAP package type used for the board
 */
int omap2430_mux_init(struct omap_board_mux *board_mux, int flags);

/**
 * omap3_mux_init() - initialize mux system with board specific set
 * @board_mux:		Board specific mux table
 * @flags:		OMAP package type used for the board
 */
int omap3_mux_init(struct omap_board_mux *board_mux, int flags);

/**
 * omap4_mux_init() - initialize mux system with board specific set
 * @board_mux:		Board specific mux table
 * @flags:		OMAP package type used for the board
 */
int omap4_mux_init(struct omap_board_mux *board_mux, int flags);

/**
 * omap_mux_init - private mux init function, do not call
 */
int omap_mux_init(const char *name, u32 flags,
		  u32 mux_pbase, u32 mux_size,
		  struct omap_mux *superset,
		  struct omap_mux *package_subset,
		  struct omap_board_mux *board_mux,
		  struct omap_ball *package_balls);

