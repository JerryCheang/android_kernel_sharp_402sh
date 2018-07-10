/* drivers/sharp/nfc/snfc_ant.c (NFC driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <asm/current.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/major.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/wakelock.h>
#include <sharp/snfc_ant.h>
#include "nfc.h"

/* snfc_ant */
#define D_SNFC_ANT_DEVS		(1)
#define D_SNFC_ANT_DEV_NAME	("snfc_ant")

/* S-7760A device code */
#define SNFC_ANT_PPC_DEVICE_CODE							0x40			/* device code						*/

/* S-7760A command ID */
#define SNFC_ANT_PPC_RELOAD_COMMAND							0x00			/* reload command					*/
#define SNFC_ANT_PPC_CHANGE_ACCESSMODE_COMMAND				0x01			/* change access mode command		*/
#define SNFC_ANT_PPC_WRITE_TIMEENABLE_COMMAND				0x02			/* write TimeEnable command			*/
#define SNFC_ANT_PPC_ACCESS_CONTROLPORT_COMMAND				0x05			/* access ControlPort command		*/
#define SNFC_ANT_PPC_SET_TIMERSCALE_REGISTER_COMMAND		0x06			/* set TimeScale command			*/
#define SNFC_ANT_PPC_SET_TIMER_REGISTER_DO0_COMMAND			0x08			/* set Timer Register 0				*/
#define SNFC_ANT_PPC_SET_TIMER_REGISTER_DO1_COMMAND			0x09			/* set Timer Rgeister 1				*/
#define SNFC_ANT_PPC_SET_TIMER_REGISTER_DO2_COMMAND			0x0A			/* set Timer Register 2				*/
#define SNFC_ANT_PPC_SET_TIMER_REGISTER_DO3_COMMAND			0x0B			/* set Timer Register 3				*/

/* access mode */
#define ACCESSMODE_REGISTER			0
#define ACCESSMODE_E2PROM			1

#define I2C_RETRY_COUNT				3
#define I2C_INTERVAL_USEC			10000

/*
 * prototype
 */
static __init int snfc_ant_init(void);
static __exit void snfc_ant_exit(void);

static int snfc_ant_i2c_read(unsigned char cmd, unsigned char *out_data);
static int snfc_ant_i2c_write(unsigned char cmd, unsigned char data);
static int snfc_ant_change_accessmode(int accessmode);

static int snfc_ant_ppc_reload(void);
static int snfc_ant_ppc_read_controlport(int accessmode, unsigned char *data);
static int snfc_ant_ppc_write_controlport(int accessmode, unsigned char data);

/*
 * global variable
 */
static struct class *snfc_ant_class = NULL;
static struct cdev snfc_ant_cdev;
static struct i2c_client *this_client;
static struct input_dev *snfc_ant_input_dev;

/*
 * function_snfc_ant
 */
static int snfc_ant_i2c_read(unsigned char cmd, unsigned char *out_data)
{
	int ret, count;
	const unsigned char addr = SNFC_ANT_PPC_DEVICE_CODE | cmd;

	/* i2c_transfer(read) */
	struct i2c_msg mesgs[] = {
		{
			.addr	= addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= out_data,
		},
	};

	NFC_DRV_DBG_LOG("START cmd=0x%02X, addr=0x%02X", cmd, addr);

	if (!out_data) {
		NFC_DRV_ERR_LOG("out_data is null");
		return -1;
	}

	for (count = 0; count < I2C_RETRY_COUNT; count++) {
		ret = i2c_transfer(this_client->adapter, mesgs, 1);
		usleep(I2C_INTERVAL_USEC);
		if (ret >= 0) {
			break;
		} else {
			NFC_DRV_DBG_LOG("i2c_retry_count=%d", count);
			ret = -1;
		}
	}

	NFC_DRV_DBG_LOG("END ret=%d", ret);

	return ret;
}

static int snfc_ant_i2c_write(unsigned char cmd, unsigned char data)
{
	int ret, count;
	const unsigned char addr = SNFC_ANT_PPC_DEVICE_CODE | cmd;

	/* i2c_transfer(write) */
	struct i2c_msg mesgs[] = {
		{
			.addr	= addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &data,
		},
	};

	NFC_DRV_DBG_LOG("START cmd=0x%02X, addr=0x%02X, data=%02X", cmd, addr, data);

	for (count = 0; count < I2C_RETRY_COUNT; count++) {
		ret = i2c_transfer(this_client->adapter, mesgs, 1);
		usleep(I2C_INTERVAL_USEC);
		if (ret >= 0) {
			break;
		} else {
			NFC_DRV_DBG_LOG("i2c_retry_count=%d", count);
			ret = -1;
		}
	}

	NFC_DRV_DBG_LOG("END ret=%d", ret);

	return ret;
}

static int snfc_ant_change_accessmode(int accessmode)
{
	int ret;
	unsigned char data;

	NFC_DRV_DBG_LOG("START accessmode=%d", accessmode);

	if (accessmode == ACCESSMODE_E2PROM) {
		ret = snfc_ant_i2c_read(SNFC_ANT_PPC_CHANGE_ACCESSMODE_COMMAND, &data);
	} else {
		ret = snfc_ant_i2c_write(SNFC_ANT_PPC_CHANGE_ACCESSMODE_COMMAND, 0);
	}

	NFC_DRV_DBG_LOG("END ret=%d", ret);

	return ret;
}

static int snfc_ant_ppc_reload(void)
{
	int ret;

	NFC_DRV_DBG_LOG("START");

	/* change access mode */
	ret = snfc_ant_change_accessmode(ACCESSMODE_REGISTER);
	if (ret < 0) {
		NFC_DRV_ERR_LOG("change_accessmode");
		return -1;
	}

	/* reload */
	ret = snfc_ant_i2c_write(SNFC_ANT_PPC_RELOAD_COMMAND, 0);
	if (ret < 0) {
		NFC_DRV_ERR_LOG("reload");
		return -1;
	}

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static int snfc_ant_ppc_read_controlport(int accessmode, unsigned char *out_data)
{
	int ret;

	NFC_DRV_DBG_LOG("START");

	/* NULL check */
	if (!out_data) {
		NFC_DRV_ERR_LOG("out_data is null");
		return -1;
	}

	/* change access mode */
	ret = snfc_ant_change_accessmode(accessmode);
	if (ret < 0) {
		NFC_DRV_ERR_LOG("change_accessmode");
		return -1;
	}

	/* read ControlPort */
	ret = snfc_ant_i2c_read(SNFC_ANT_PPC_ACCESS_CONTROLPORT_COMMAND, out_data);
	if (ret < 0) {
		NFC_DRV_ERR_LOG("read");
		return -1;
	}

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static int snfc_ant_ppc_write_controlport(int accessmode, unsigned char data)
{
	int ret;

	NFC_DRV_DBG_LOG("START");

	/* change access mode */
	ret = snfc_ant_change_accessmode(accessmode);
	if (ret < 0) {
		NFC_DRV_ERR_LOG("change_accessmode");
		return -1;
	}

	/* write ControlPort */
	ret = snfc_ant_i2c_write(SNFC_ANT_PPC_ACCESS_CONTROLPORT_COMMAND, data);
	if (ret < 0) {
		NFC_DRV_ERR_LOG("write");
		return -1;
	}

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static ssize_t snfc_ant_read(struct file *file, char __user *buf, size_t len, loff_t *pos)
{
	int ret;
	unsigned char data[2];

	NFC_DRV_DBG_LOG("START");

	ret = snfc_ant_ppc_read_controlport(ACCESSMODE_REGISTER, &data[0]);
	if (ret < 0) {
		NFC_DRV_ERR_LOG("snfc_ant_ppc_read_controlport");
		return -EIO;
	}

	data[1] = 0x00;

	if (len > 2) {
		len = 2;
	}

	if (copy_to_user(buf, data, len)) {
		NFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}

	NFC_DRV_DBG_LOG("END data=0x%02X, len=%d", data[0], len);

	return len;
}

static ssize_t snfc_ant_write(struct file *file, const char *buf, size_t len, loff_t *pos)
{
	int ret;
	unsigned char data;

	/* length check */
	if (len < 1) {
		NFC_DRV_ERR_LOG("length check len=%d", len);
		return -EIO;
	}

	if (copy_from_user(&data, buf, 1)) {
		NFC_DRV_ERR_LOG("copy_from_user");
		return -EFAULT;
	}

	ret = snfc_ant_ppc_write_controlport(ACCESSMODE_E2PROM, data);
	if (ret == -1) {
		NFC_DRV_ERR_LOG("snfc_ant_ppc_write_controlport E2PROM");
		return -EIO;
	}

	NFC_DRV_DBG_LOG("END data=0x%02X", data);

	return len;
}

static long snfc_ant_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned char data;

	NFC_DRV_DBG_LOG("START cmd=%u", cmd);

	switch (cmd) {
	/* reload */
	case SNFC_ANT_PPC_RELOAD:
		NFC_DRV_DBG_LOG("RELOAD");

		if (snfc_ant_ppc_reload()) {
			NFC_DRV_ERR_LOG("snfc_ant_ppc_reload");
			return -EIO;
		}
		break;

	/* read ControlPort E2PROM */
	case SNFC_ANT_PPC_READ_CONTROLPORT_E2PROM:
		NFC_DRV_DBG_LOG("READ_CONTROLPORT_E2PROM");
		if (snfc_ant_ppc_read_controlport(ACCESSMODE_E2PROM, &data)) {
			NFC_DRV_ERR_LOG("read ControlPort E2PROM");
			return -EIO;
		}
	
		NFC_DRV_DBG_LOG("read data=0x%02X", data);

		if (copy_to_user((unsigned char __user *)arg, &data, 1)) {
			NFC_DRV_ERR_LOG("copy_to_user E2PROM");
			return -EFAULT;
		}
		break;

	/* read ControlPort Register */
	case SNFC_ANT_PPC_READ_CONTROLPORT_REGISTER:
		NFC_DRV_DBG_LOG("READ_CONTROLPORT_REGISTER");

		if (snfc_ant_ppc_read_controlport(ACCESSMODE_REGISTER, &data)) {
			NFC_DRV_ERR_LOG("read ControlPort Register");
			return -EIO;
		}
	
		NFC_DRV_DBG_LOG("read data=0x%02X", data);

		if (copy_to_user((unsigned char __user *)arg, &data, 1)) {
			NFC_DRV_ERR_LOG("copy_to_user Register");
			return -EFAULT;
		}
		break;

	/* write ControlPort E2PROM */
	case SNFC_ANT_PPC_WRITE_CONTROLPORT_E2PROM:
		NFC_DRV_DBG_LOG("WRITE_CONTROLPORT_E2PROM");

		if (copy_from_user(&data, (unsigned char __user *)arg, 1)) {
			NFC_DRV_ERR_LOG("copy_from_user E2PROM");
			return -EFAULT;
		}

		if (snfc_ant_ppc_write_controlport(ACCESSMODE_E2PROM, data)) {
			NFC_DRV_ERR_LOG("write ControlPort E2PROM");
			return -EIO;
		}
		break;

	/* write ControlPort Register */
	case SNFC_ANT_PPC_WRITE_CONTROLPORT_REGISTER:
		NFC_DRV_DBG_LOG("WRITE_CONTROLPORT_REGISTER");

		if (copy_from_user(&data, (unsigned char __user *)arg, 1)) {
			NFC_DRV_ERR_LOG("copy_from_user REGISTER");
			return -EFAULT;
		}

		if (snfc_ant_ppc_write_controlport(ACCESSMODE_REGISTER, data)) {
			NFC_DRV_ERR_LOG("write ControlPort REGISTER");
			return -EIO;
		}
		break;

	default:
		NFC_DRV_ERR_LOG("invalid cmd");
		return -EINVAL;
	}

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static int snfc_ant_open(struct inode *inode, struct file *filp)
{
	NFC_DRV_DBG_LOG("");
	return 0;
}

static int snfc_ant_release(struct inode *inode, struct file *filp)
{
	NFC_DRV_DBG_LOG("");
	return 0;
}

static const struct file_operations snfc_ant_fileops = {
	.owner          = THIS_MODULE,
	.read           = snfc_ant_read,
	.write          = snfc_ant_write,
	.unlocked_ioctl = snfc_ant_ioctl,
	.open           = snfc_ant_open,
	.release        = snfc_ant_release,
};

static int snfc_ant_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
	int ret = 0;
	struct device *class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	NFC_DRV_DBG_LOG("START");

	this_client = client;

	snfc_ant_input_dev = input_allocate_device();

	ret = alloc_chrdev_region(&dev, 0, D_SNFC_ANT_DEVS, D_SNFC_ANT_DEV_NAME);
	if (ret) {
		NFC_DRV_ERR_LOG("alloc_chrdev_region ret=%d", ret);
		return ret;
	}

	cdev_init(&snfc_ant_cdev, &snfc_ant_fileops);
	snfc_ant_cdev.owner = THIS_MODULE;

	ret = cdev_add(&snfc_ant_cdev, dev, D_SNFC_ANT_DEVS);
	if (ret) {
		unregister_chrdev_region(dev, D_SNFC_ANT_DEVS);
		NFC_DRV_ERR_LOG("cdev_add ret=%d", ret);
		return ret;
	}

	class_dev = device_create(snfc_ant_class, NULL, dev, NULL, D_SNFC_ANT_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&snfc_ant_cdev);
		unregister_chrdev_region(dev, D_SNFC_ANT_DEVS);
		ret = PTR_ERR(class_dev);
		NFC_DRV_ERR_LOG("device_create ret=%d", ret);
		return ret;
	}

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static int snfc_ant_remove(struct i2c_client *client)
{
	NFC_DRV_DBG_LOG("START");

	input_unregister_device(snfc_ant_input_dev);

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static const struct i2c_device_id snfc_ant_id[] = {
	{ D_SNFC_ANT_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver s7760a_driver =
{
	.driver = {
		.owner	= THIS_MODULE,
		.name	= D_SNFC_ANT_DEV_NAME,
	},
	.class	 = I2C_CLASS_HWMON,
	.probe	 = snfc_ant_probe,
	.remove	 = snfc_ant_remove,
	.id_table = snfc_ant_id,
};

/*
 * snfc_ant_init
 */
static __init int snfc_ant_init(void)
{
	int ret;

	snfc_ant_class = class_create(THIS_MODULE, "snfc_ant");

	if (IS_ERR(snfc_ant_class)) {
		return PTR_ERR(snfc_ant_class);
	}

	ret = i2c_add_driver(&s7760a_driver);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/*
 * snfc_ant_exit
 */
static __exit void snfc_ant_exit(void)
{
	class_destroy(snfc_ant_class);
	i2c_del_driver(&s7760a_driver);
}

MODULE_LICENSE("GPL v2");

module_init(snfc_ant_init);
module_exit(snfc_ant_exit);

