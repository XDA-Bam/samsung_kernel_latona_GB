/*
 * linux/arch/arm/mach-omap2/mux.c
 *
 * OMAP2, OMAP3 and OMAP4 pin multiplexing configurations
 *
 * Copyright (C) 2004 - 2010 Texas Instruments Inc.
 * Copyright (C) 2003 - 2008 Nokia Corporation
 *
 * Written by Tony Lindgren
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <asm/system.h>

#include <plat/control.h>
#include "mux.h"

#define OMAP_MUX_BASE_OFFSET		0x30	/* Offset from CTRL_BASE */
#define OMAP_MUX_BASE_SZ		0x5ca

struct omap_mux_entry {
	struct omap_mux		mux;
	struct list_head	node;
};

static LIST_HEAD(mux_partitions);
static DEFINE_MUTEX(muxmode_mutex);

static u16 omap_mux_read(struct omap_mux_partition *partition, u16 reg)
{
	if (partition->flags & OMAP_MUX_REG_8BIT)
		return __raw_readb(partition->base + reg);
	else
		return __raw_readw(partition->base + reg);
}

static void omap_mux_write(struct omap_mux_partition *partition, u16 val,
			   u16 reg)
{
	if (partition->flags & OMAP_MUX_REG_8BIT)
		__raw_writeb(val, partition->base + reg);
	else
		__raw_writew(val, partition->base + reg);
}

static void omap_mux_write_array(struct omap_mux_partition *partition,
				 struct omap_board_mux *board_mux)
{
	while (board_mux->reg_offset != OMAP_MUX_TERMINATOR) {
		omap_mux_write(partition, board_mux->value,
			       board_mux->reg_offset);
		board_mux++;
	}
}

#ifdef CONFIG_OMAP_MUX

static char *omap_mux_options;

static int __init _omap_mux_init_gpio(struct omap_mux_partition *partition,
				      int gpio, int val)
{
	struct omap_mux_entry *e;
	struct omap_mux *gpio_mux = NULL;
	u16 old_mode;
	u16 mux_mode;
	int found = 0;
	struct list_head *muxmodes = &partition->muxmodes;

	if (!gpio)
		return -EINVAL;

	list_for_each_entry(e, muxmodes, node) {
		struct omap_mux *m = &e->mux;
		if (gpio == m->gpio) {
			gpio_mux = m;
			found++;
		}
	}

	if (found == 0) {
		pr_err("%s: Could not set gpio%i\n", __func__, gpio);
		return -ENODEV;
	}

	if (found > 1) {
		pr_info("%s: Multiple gpio paths (%d) for gpio%i\n",
			__func__, found, gpio);
		return -EINVAL;
	}

	old_mode = omap_mux_read(partition, gpio_mux->reg_offset);
	mux_mode = val & ~(OMAP_MUX_NR_MODES - 1);
	if (partition->flags & OMAP_MUX_GPIO_IN_MODE3)
		mux_mode |= OMAP_MUX_MODE3;
	else
		mux_mode |= OMAP_MUX_MODE4;
	pr_debug("%s: Setting signal %s.gpio%i 0x%04x -> 0x%04x\n",
		 __func__, gpio_mux->muxnames[0], gpio,
		old_mode, mux_mode);
	omap_mux_write(partition, mux_mode, gpio_mux->reg_offset);

	return 0;
}

int __init omap_mux_init_gpio(int gpio, int val)
{
	struct omap_mux_partition *partition;
	int ret;

	list_for_each_entry(partition, &mux_partitions, node) {
		ret = _omap_mux_init_gpio(partition, gpio, val);
		if (!ret)
			return ret;
	}

	return -ENODEV;
}

static int _omap_mux_init_signal(struct omap_mux_partition *partition,
					const char *muxname, int val)
{
	struct omap_mux_entry *e;
	const char *mode_name;
	int found = 0, mode0_len = 0;
	struct list_head *muxmodes = &partition->muxmodes;

	mode_name = strchr(muxname, '.');
	if (mode_name) {
		mode0_len = strlen(muxname) - strlen(mode_name);
		mode_name++;
	} else {
		mode_name = muxname;
	}

	list_for_each_entry(e, muxmodes, node) {
		struct omap_mux *m = &e->mux;
		char *m0_entry = m->muxnames[0];
		int i;

		/* First check for full name in mode0.muxmode format */
		if (mode0_len && strncmp(muxname, m0_entry, mode0_len))
			continue;

		/* Then check for muxmode only */
		for (i = 0; i < OMAP_MUX_NR_MODES; i++) {
			char *mode_cur = m->muxnames[i];

			if (!mode_cur)
				continue;

			if (!strcmp(mode_name, mode_cur)) {
				u16 old_mode;
				u16 mux_mode;

				old_mode = omap_mux_read(partition,
							 m->reg_offset);
				mux_mode = val | i;
				pr_debug("%s: Setting signal "
					 "%s.%s 0x%04x -> 0x%04x\n",
					 __func__, m0_entry, muxname,
					old_mode, mux_mode);
				omap_mux_write(partition, mux_mode,
					       m->reg_offset);
				found++;
			}
		}
	}

	if (found == 1)
		return 0;

	if (found > 1) {
		pr_err("%s: Multiple signal paths (%i) for %s\n",
		       __func__, found, muxname);
		return -EINVAL;
	}

	pr_err("%s: Could not set signal %s\n", __func__, muxname);

	return -ENODEV;
}

u16 omap_mux_read_signal(const char *muxname)
{
	struct omap_mux_partition *partition;
	struct omap_mux_entry *e;
	const char *mode_name;
	int mode0_len = 0;
	struct list_head *muxmodes;

	mode_name = strchr(muxname, '.');
	if (mode_name) {
		mode0_len = strlen(muxname) - strlen(mode_name);
		mode_name++;
	} else {
		mode_name = muxname;
	}

	list_for_each_entry(partition, &mux_partitions, node) {
		muxmodes = &partition->muxmodes;
		list_for_each_entry(e, muxmodes, node) {
			struct omap_mux *m = &e->mux;
			char *m0_entry = m->muxnames[0];
			int i;

			/* First check for full name in mode0.muxmode format */
			if (mode0_len && strncmp(muxname, m0_entry, mode0_len))
				continue;

			/* Then check for muxmode only */
			for (i = 0; i < OMAP_MUX_NR_MODES; i++) {
				char *mode_cur = m->muxnames[i];

				if (!mode_cur)
					continue;

				if (!strcmp(mode_name, mode_cur)) {

					return omap_mux_read(partition,
							 m->reg_offset);
				}
			}
		}
	}

	pr_err("%s: Could not set signal %s\n", __func__, muxname);

	return -ENODEV;
}

int omap_mux_enable_wakeup(const char *muxname)
{
	u16 val;

	val = omap_mux_read_signal(muxname);
	if (val == -ENODEV)
		return val;
	val |= OMAP44XX_PADCONF_WAKEUPENABLE0;
	omap_mux_init_signal(muxname, val);
	return 0;
}

int omap_mux_disable_wakeup(const char *muxname)
{
	u16 val;

	val = omap_mux_read_signal(muxname);
	if (val == -ENODEV)
		return val;
	val &= ~OMAP44XX_PADCONF_WAKEUPENABLE0;
	omap_mux_init_signal(muxname, val);
	return 0;
}

int omap_mux_init_signal(const char *muxname, int val)
{
	struct omap_mux_partition *partition;
	int ret;

	list_for_each_entry(partition, &mux_partitions, node) {
		ret = _omap_mux_init_signal(partition, muxname, val);
		if (!ret)
			return ret;
	}

	return -ENODEV;
}

#ifdef CONFIG_DEBUG_FS

#define OMAP_MUX_MAX_NR_FLAGS	10
#define OMAP_MUX_TEST_FLAG(val, mask)				\
	if (((val) & (mask)) == (mask)) {			\
		i++;						\
		flags[i] =  #mask;				\
	}

/* REVISIT: Add checking for non-optimal mux settings */
static inline void omap_mux_decode(struct seq_file *s, u16 val)
{
	char *flags[OMAP_MUX_MAX_NR_FLAGS];
	char mode[sizeof("OMAP_MUX_MODE") + 1];
	int i = -1;

	sprintf(mode, "OMAP_MUX_MODE%d", val & 0x7);
	i++;
	flags[i] = mode;

	OMAP_MUX_TEST_FLAG(val, OMAP_PIN_OFF_WAKEUPENABLE);
	if (val & OMAP_OFF_EN) {
		if (val & OMAP_OFFOUT_EN) {
			if (val & OMAP_OFF_PULL_EN) {
				if (!(val & OMAP_OFF_PULL_UP)) {
					OMAP_MUX_TEST_FLAG(val,
						OMAP_PIN_OFF_INPUT_PULLDOWN);
				} else {
					OMAP_MUX_TEST_FLAG(val,
						OMAP_PIN_OFF_INPUT_PULLUP);
				}
			}
		} else {
			if (!(val & OMAP_OFFOUT_VAL)) {
				OMAP_MUX_TEST_FLAG(val,
					OMAP_PIN_OFF_OUTPUT_LOW);
			} else {
				OMAP_MUX_TEST_FLAG(val,
					OMAP_PIN_OFF_OUTPUT_HIGH);
			}
		}
	}

	if (val & OMAP_INPUT_EN) {
		if (val & OMAP_PULL_ENA) {
			if (!(val & OMAP_PULL_UP)) {
				OMAP_MUX_TEST_FLAG(val,
					OMAP_PIN_INPUT_PULLDOWN);
			} else {
				OMAP_MUX_TEST_FLAG(val, OMAP_PIN_INPUT_PULLUP);
			}
		} else {
			OMAP_MUX_TEST_FLAG(val, OMAP_PIN_INPUT);
		}
	} else {
		i++;
		flags[i] = "OMAP_PIN_OUTPUT";
	}

	do {
		seq_printf(s, "%s", flags[i]);
		if (i > 0)
			seq_printf(s, " | ");
	} while (i-- > 0);
}

#define OMAP_MUX_DEFNAME_LEN	32

static int omap_mux_dbg_board_show(struct seq_file *s, void *unused)
{
	struct omap_mux_partition *partition = s->private;
	struct omap_mux_entry *e;
	u8 omap_gen = omap_rev() >> 28;

	list_for_each_entry(e, &partition->muxmodes, node) {
		struct omap_mux *m = &e->mux;
		char m0_def[OMAP_MUX_DEFNAME_LEN];
		char *m0_name = m->muxnames[0];
		u16 val;
		int i, mode;

		if (!m0_name)
			continue;

		/* REVISIT: Needs to be updated if mode0 names get longer */
		for (i = 0; i < OMAP_MUX_DEFNAME_LEN; i++) {
			if (m0_name[i] == '\0') {
				m0_def[i] = m0_name[i];
				break;
			}
			m0_def[i] = toupper(m0_name[i]);
		}
		val = omap_mux_read(partition, m->reg_offset);
		mode = val & OMAP_MUX_MODE7;
		if (mode != 0)
			seq_printf(s, "/* %s */\n", m->muxnames[mode]);

		/*
		 * XXX: Might be revisited to support differences accross
		 * same OMAP generation.
		 */
		seq_printf(s, "OMAP%d_MUX(%s, ", omap_gen, m0_def);
		omap_mux_decode(s, val);
		seq_printf(s, "),\n");
	}

	return 0;
}

static int omap_mux_dbg_board_open(struct inode *inode, struct file *file)
{
	return single_open(file, omap_mux_dbg_board_show, inode->i_private);
}

static const struct file_operations omap_mux_dbg_board_fops = {
	.open		= omap_mux_dbg_board_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int omap_mux_dbg_signal_show(struct seq_file *s, void *unused)
{
	struct omap_mux *m = s->private;
	struct omap_mux_partition *partition = m->partition;
	const char *none = "NA";
	u16 val;
	int mode;

	val = omap_mux_read(partition, m->reg_offset);
	mode = val & OMAP_MUX_MODE7;

	seq_printf(s, "name: %s.%s (0x%08x/0x%03x = 0x%04x), b %s, t %s\n",
			m->muxnames[0], m->muxnames[mode],
			partition->phys + m->reg_offset, m->reg_offset, val,
			m->balls[0] ? m->balls[0] : none,
			m->balls[1] ? m->balls[1] : none);
	seq_printf(s, "mode: ");
	omap_mux_decode(s, val);
	seq_printf(s, "\n");
	seq_printf(s, "signals: %s | %s | %s | %s | %s | %s | %s | %s\n",
			m->muxnames[0] ? m->muxnames[0] : none,
			m->muxnames[1] ? m->muxnames[1] : none,
			m->muxnames[2] ? m->muxnames[2] : none,
			m->muxnames[3] ? m->muxnames[3] : none,
			m->muxnames[4] ? m->muxnames[4] : none,
			m->muxnames[5] ? m->muxnames[5] : none,
			m->muxnames[6] ? m->muxnames[6] : none,
			m->muxnames[7] ? m->muxnames[7] : none);

	return 0;
}

#define OMAP_MUX_MAX_ARG_CHAR  7

static ssize_t omap_mux_dbg_signal_write(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	char buf[OMAP_MUX_MAX_ARG_CHAR];
	struct seq_file *seqf;
	struct omap_mux *m;
	unsigned long val;
	int buf_size, ret;

	if (count > OMAP_MUX_MAX_ARG_CHAR)
		return -EINVAL;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);

	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	ret = strict_strtoul(buf, 0x10, &val);
	if (ret < 0)
		return ret;

	if (val > 0xffff)
		return -EINVAL;

	seqf = file->private_data;
	m = seqf->private;

	omap_mux_write(m->partition, (u16)val, m->reg_offset);
	*ppos += count;

	return count;
}

static int omap_mux_dbg_signal_open(struct inode *inode, struct file *file)
{
	return single_open(file, omap_mux_dbg_signal_show, inode->i_private);
}

static const struct file_operations omap_mux_dbg_signal_fops = {
	.open		= omap_mux_dbg_signal_open,
	.read		= seq_read,
	.write		= omap_mux_dbg_signal_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *mux_dbg_dir;

static void __init omap_mux_dbg_create_entry(
				struct omap_mux_partition *partition,
				struct dentry *mux_dbg_dir)
{
	struct omap_mux_entry *e;

	list_for_each_entry(e, &partition->muxmodes, node) {
		struct omap_mux *m = &e->mux;
		m->partition = partition;

		(void)debugfs_create_file(m->muxnames[0], S_IWUSR | S_IWGRP, mux_dbg_dir,
					  m, &omap_mux_dbg_signal_fops);
	}
}

static void __init omap_mux_dbg_init(void)
{
	struct omap_mux_partition *partition;
	static struct dentry *mux_dbg_board_dir;

	mux_dbg_dir = debugfs_create_dir("omap_mux", NULL);
	if (!mux_dbg_dir)
		return;

	mux_dbg_board_dir = debugfs_create_dir("board", mux_dbg_dir);
	if (!mux_dbg_board_dir)
		return;

	list_for_each_entry(partition, &mux_partitions, node) {
		omap_mux_dbg_create_entry(partition, mux_dbg_dir);
		(void)debugfs_create_file(partition->name, S_IRUGO,
					  mux_dbg_board_dir, partition,
					  &omap_mux_dbg_board_fops);
	}
}

#else
static inline void omap_mux_dbg_init(void)
{
}
#endif	/* CONFIG_DEBUG_FS */

static void __init omap_mux_free_names(struct omap_mux *m)
{
	int i;

	for (i = 0; i < OMAP_MUX_NR_MODES; i++)
		kfree(m->muxnames[i]);

#ifdef CONFIG_DEBUG_FS
	for (i = 0; i < OMAP_MUX_NR_SIDES; i++)
		kfree(m->balls[i]);
#endif

}

/* Free all data except for GPIO pins unless CONFIG_DEBUG_FS is set */
static int __init omap_mux_late_init(void)
{
	struct omap_mux_partition *partition;

	list_for_each_entry(partition, &mux_partitions, node) {
		struct omap_mux_entry *e, *tmp;
		list_for_each_entry_safe(e, tmp, &partition->muxmodes, node) {
			struct omap_mux *m = &e->mux;
			u16 mode = omap_mux_read(partition, m->reg_offset);

			if (OMAP_MODE_GPIO(mode))
				continue;

#ifndef CONFIG_DEBUG_FS
			mutex_lock(&muxmode_mutex);
			list_del(&e->node);
			mutex_unlock(&muxmode_mutex);
			omap_mux_free_names(m);
			kfree(m);
#endif
		}
	}

	omap_mux_dbg_init();

	return 0;
}
late_initcall(omap_mux_late_init);

static void __init omap_mux_package_fixup(struct omap_mux *p,
					struct omap_mux *superset)
{
	while (p->reg_offset !=  OMAP_MUX_TERMINATOR) {
		struct omap_mux *s = superset;
		int found = 0;

		while (s->reg_offset != OMAP_MUX_TERMINATOR) {
			if (s->reg_offset == p->reg_offset) {
				*s = *p;
				found++;
				break;
			}
			s++;
		}
		if (!found)
			pr_err("%s: Unknown entry offset 0x%x\n",
			       __func__, p->reg_offset);
		p++;
	}
}

#ifdef CONFIG_DEBUG_FS

static void __init omap_mux_package_init_balls(struct omap_ball *b,
				struct omap_mux *superset)
{
	while (b->reg_offset != OMAP_MUX_TERMINATOR) {
		struct omap_mux *s = superset;
		int found = 0;

		while (s->reg_offset != OMAP_MUX_TERMINATOR) {
			if (s->reg_offset == b->reg_offset) {
				s->balls[0] = b->balls[0];
				s->balls[1] = b->balls[1];
				found++;
				break;
			}
			s++;
		}
		if (!found)
			pr_err("%s: Unknown ball offset 0x%x\n",
			       __func__, b->reg_offset);
		b++;
	}
}

#else	/* CONFIG_DEBUG_FS */

static inline void omap_mux_package_init_balls(struct omap_ball *b,
					struct omap_mux *superset)
{
}

#endif	/* CONFIG_DEBUG_FS */

static int __init omap_mux_setup(char *options)
{
	if (!options)
		return 0;

	omap_mux_options = options;

	return 1;
}
__setup("omap_mux=", omap_mux_setup);

/*
 * Note that the omap_mux=some.signal1=0x1234,some.signal2=0x1234
 * cmdline options only override the bootloader values.
 * During development, please enable CONFIG_DEBUG_FS, and use the
 * signal specific entries under debugfs.
 */
static void __init omap_mux_set_cmdline_signals(void)
{
	char *options, *next_opt, *token;

	if (!omap_mux_options)
		return;

	options = kmalloc(strlen(omap_mux_options) + 1, GFP_KERNEL);
	if (!options)
		return;

	strcpy(options, omap_mux_options);
	next_opt = options;

	while ((token = strsep(&next_opt, ",")) != NULL) {
		char *keyval, *name;
		unsigned long val;

		keyval = token;
		name = strsep(&keyval, "=");
		if (name) {
			int res;

			res = strict_strtoul(keyval, 0x10, &val);
			if (res < 0)
				continue;

			omap_mux_init_signal(name, (u16)val);
		}
	}

	kfree(options);
}

static int __init omap_mux_copy_names(struct omap_mux *src,
				      struct omap_mux *dst)
{
	int i;

	for (i = 0; i < OMAP_MUX_NR_MODES; i++) {
		if (src->muxnames[i]) {
			dst->muxnames[i] =
				kmalloc(strlen(src->muxnames[i]) + 1,
					GFP_KERNEL);
			if (!dst->muxnames[i])
				goto free;
			strcpy(dst->muxnames[i], src->muxnames[i]);
		}
	}

#ifdef CONFIG_DEBUG_FS
	for (i = 0; i < OMAP_MUX_NR_SIDES; i++) {
		if (src->balls[i]) {
			dst->balls[i] =
				kmalloc(strlen(src->balls[i]) + 1,
					GFP_KERNEL);
			if (!dst->balls[i])
				goto free;
			strcpy(dst->balls[i], src->balls[i]);
		}
	}
#endif

	return 0;

free:
	omap_mux_free_names(dst);
	return -ENOMEM;

}

#endif	/* CONFIG_OMAP_MUX */

static struct omap_mux *omap_mux_get_by_gpio(
				struct omap_mux_partition *partition,
				int gpio)
{
	struct omap_mux_entry *e;
	struct omap_mux *ret = NULL;

	list_for_each_entry(e, &partition->muxmodes, node) {
		struct omap_mux *m = &e->mux;
		if (m->gpio == gpio) {
			ret = m;
			break;
		}
	}

	return ret;
}

/* Needed for dynamic muxing of GPIO pins for off-idle */
u16 omap_mux_get_gpio(int gpio)
{
	struct omap_mux_partition *partition;
	struct omap_mux *m = NULL;

	list_for_each_entry(partition, &mux_partitions, node) {
		m = omap_mux_get_by_gpio(partition, gpio);
		if (m)
			return omap_mux_read(partition, m->reg_offset);
	}

	if (!m || m->reg_offset == OMAP_MUX_TERMINATOR)
		pr_err("%s: Could not get gpio%i\n", __func__, gpio);

	return OMAP_MUX_TERMINATOR;
}

/* Needed for dynamic muxing of GPIO pins for off-idle */
void omap_mux_set_gpio(u16 val, int gpio)
{
	struct omap_mux_partition *partition;
	struct omap_mux *m = NULL;

	list_for_each_entry(partition, &mux_partitions, node) {
		m = omap_mux_get_by_gpio(partition, gpio);
		if (m) {
			omap_mux_write(partition, val, m->reg_offset);
			return;
		}
	}

	if (!m || m->reg_offset == OMAP_MUX_TERMINATOR)
		pr_err("%s: Could not set gpio%i\n", __func__, gpio);
}

static struct omap_mux * __init omap_mux_list_add(
					struct omap_mux_partition *partition,
					struct omap_mux *src)
{
	struct omap_mux_entry *entry;
	struct omap_mux *m;

	entry = kzalloc(sizeof(struct omap_mux_entry), GFP_KERNEL);
	if (!entry)
		return NULL;

	m = &entry->mux;
	memcpy(m, src, sizeof(struct omap_mux_entry));

#ifdef CONFIG_OMAP_MUX
	if (omap_mux_copy_names(src, m)) {
		kfree(entry);
		return NULL;
	}
#endif

	mutex_lock(&muxmode_mutex);
	list_add_tail(&entry->node, &partition->muxmodes);
	mutex_unlock(&muxmode_mutex);

	return m;
}

/*
 * Note if CONFIG_OMAP_MUX is not selected, we will only initialize
 * the GPIO to mux offset mapping that is needed for dynamic muxing
 * of GPIO pins for off-idle.
 */
static void __init omap_mux_init_list(struct omap_mux_partition *partition,
				      struct omap_mux *superset)
{
	while (superset->reg_offset !=  OMAP_MUX_TERMINATOR) {
		struct omap_mux *entry;

#ifdef CONFIG_OMAP_MUX
		if (!superset->muxnames || !superset->muxnames[0]) {
			superset++;
			continue;
		}
#else
		/* Skip pins that are not muxed as GPIO by bootloader */
		if (!OMAP_MODE_GPIO(omap_mux_read(partition,
				superset->reg_offset))) {
			superset++;
			continue;
		}
#endif

		entry = omap_mux_list_add(partition, superset);
		if (!entry) {
			pr_err("%s: Could not add entry\n", __func__);
			return;
		}
		superset++;
	}
}

#ifdef CONFIG_OMAP_MUX

static void omap_mux_init_package(struct omap_mux *superset,
				  struct omap_mux *package_subset,
				  struct omap_ball *package_balls)
{
	if (package_subset)
		omap_mux_package_fixup(package_subset, superset);
	if (package_balls)
		omap_mux_package_init_balls(package_balls, superset);
}

static void omap_mux_init_signals(struct omap_mux_partition *partition,
				  struct omap_board_mux *board_mux)
{
	omap_mux_set_cmdline_signals();
	omap_mux_write_array(partition, board_mux);
}

#else

static void omap_mux_init_package(struct omap_mux *superset,
				  struct omap_mux *package_subset,
				  struct omap_ball *package_balls)
{
}

static void omap_mux_init_signals(struct omap_mux_partition *partition,
				  struct omap_board_mux *board_mux)
{
}

#endif

static u32 mux_partitions_cnt;

int __init omap_mux_init(const char * name, u32 flags,
			 u32 mux_pbase, u32 mux_size,
			 struct omap_mux *superset,
			 struct omap_mux *package_subset,
			 struct omap_board_mux *board_mux,
			 struct omap_ball *package_balls)
{
	struct omap_mux_partition *partition;

	partition = kzalloc(sizeof(struct omap_mux_partition), GFP_KERNEL);
	if (!partition)
		return -ENOMEM;

	partition->name = name;
	partition->flags = flags;
	partition->size = mux_size;
	partition->phys = mux_pbase;
	partition->base = ioremap(mux_pbase, mux_size);
	if (!partition->base) {
		pr_err("%s: Could not ioremap mux partition at 0x%08x\n",
			__func__, partition->phys);
		return -ENODEV;
	}

	INIT_LIST_HEAD(&partition->muxmodes);

	list_add_tail(&partition->node, &mux_partitions);
	mux_partitions_cnt++;
	pr_info("%s: Add partition: #%d: %s, flags: %x\n", __func__,
		mux_partitions_cnt, partition->name, partition->flags);

	omap_mux_init_package(superset, package_subset, package_balls);
	omap_mux_init_list(partition, superset);
	omap_mux_init_signals(partition, board_mux);

	return 0;
}

