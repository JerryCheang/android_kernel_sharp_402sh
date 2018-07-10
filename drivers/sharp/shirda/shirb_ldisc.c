/* drivers/sharp/shirda/shirb_ldisc.c (sharp IrDA driver)
 *
 * Copyright (C) 2011 - 2014 SHARP CORPORATION All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/serial_core.h>

#include "shirda_kdrv.h"
#include "sharp/irda_kdrv_api.h"
#include "sharp/irb_kdrv_api.h"


#define	SHIRB_LDISC_VERSION	"00.10.00"

#define	SHIRB_LDISC_NAME	"shirb_ldisc"

spinlock_t irb_write_lock;

static int shirb_ldisc_open(struct tty_struct *tty);
static void shirb_ldisc_close(struct tty_struct *tty);
static ssize_t shirb_ldisc_write(struct tty_struct *tty, struct file *file,
					const unsigned char *buf, size_t nr);
static int shirb_ldisc_ioctl(struct tty_struct *tty, struct file * file,
					unsigned int cmd, unsigned long arg);
static void shirb_ldisc_receive_buf(struct tty_struct *tty,
				const unsigned char *cp, char *fp, int count);

static ktime_t start_time;

static int shirb_write_check_time(long limit_nsec)
{
	long run_nsec;
	ktime_t end_time;
	int ret = 0;

	end_time = ktime_get();
	if (0 < limit_nsec) {
		run_nsec = ktime_to_ns(ktime_sub(end_time, start_time));
		if (limit_nsec < run_nsec) {
			IRDALOG_ERROR("Exceed time. (%ld < %ld)\n",
							 limit_nsec, run_nsec);
			ret = -EFAULT;
		}
	}
	memcpy(&start_time, &end_time, sizeof(ktime_t));

	return ret;
}

static int shirb_ldisc_open(struct tty_struct *tty)
{
	struct uart_state *state;
	int ret;

	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);

	tty_driver_flush_buffer(tty);

	state = (struct uart_state *)tty->driver_data;
	ret = msm_hsl_set_irrc(state->uart_port, true);

	return ret;
}

static void shirb_ldisc_close(struct tty_struct *tty)
{
	struct uart_state *state;

	state = (struct uart_state *)tty->driver_data;
	msm_hsl_set_irrc(state->uart_port, false);

	return;
}

static ssize_t shirb_ldisc_write(struct tty_struct *tty, struct file *file,
					const unsigned char *buf, size_t nr)
{
	ssize_t 		ret = 0;
	long			*limit_p;
	size_t			len;
	unsigned long		flag;

	IRBLOG_INFO("IrBlaster write() ENTRY");

	spin_lock_irqsave(&irb_write_lock, flag);

	limit_p = (long *)buf;
	buf = buf + sizeof(long);
	len  = nr - sizeof(long);

	ret = shirb_write_check_time(*limit_p);
	if (ret < 0) {
		IRBLOG_ERROR("delay is detected\n");
	} else if (0 < len) {
		ret = tty->ops->write(tty, buf, len);
		if (ret > 0) {
			ret = (ssize_t)nr;
		}
	} else {
		IRBLOG_ERROR("write size error");
		ret = -EIO;
	}

	spin_unlock_irqrestore(&irb_write_lock, flag);

	return ret;
}

static int shirb_ldisc_ioctl(struct tty_struct *tty, struct file * file,
					unsigned int cmd, unsigned long arg)
{
	return 0;
}

static void shirb_ldisc_receive_buf(struct tty_struct *tty,
				const unsigned char *cp, char *fp, int count)
{
	IRBLOG_INFO("receive_buf() ENTRY\n");
	return;
}

struct tty_ldisc_ops shirb_ldisc_ops = {
	.magic        = TTY_LDISC_MAGIC,
	.name         = SHIRB_LDISC_NAME,
	.open         = shirb_ldisc_open,
	.close        = shirb_ldisc_close,
	.write        = shirb_ldisc_write,
	.ioctl        = shirb_ldisc_ioctl,
	.receive_buf  = shirb_ldisc_receive_buf,
};

static int __init n_shirb_init(void)
{
	int err;
	err = tty_register_ldisc(N_SHIRB, &shirb_ldisc_ops);
	if (err != 0) {
		IRBLOG_ERROR(
			"failed to register N_SHIRB (error:%d).\n", err);
	}
	spin_lock_init(&irb_write_lock);
	return err;
}

static void __exit n_shirb_exit(void)
{
	int err;
	err = tty_unregister_ldisc(N_SHIRB);
	if (err != 0) {
		IRBLOG_ERROR(
			"failed to unregister N_SHIRB (error:%d).\n", err);
	}
}

module_init(n_shirb_init);
module_exit(n_shirb_exit);

MODULE_VERSION(SHIRB_LDISC_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_ALIAS_LDISC(N_SHIRB);
