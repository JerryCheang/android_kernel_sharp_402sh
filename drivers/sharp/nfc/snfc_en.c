/* drivers/sharp/nfc/snfc_en.c (NFC driver)
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
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/major.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/qpnp/pin.h>
#include <asm/uaccess.h>
#include <sharp/snfc_en.h>
#include <linux/regulator/consumer.h>
#include "nfc.h"

/* snfc_en */
#define D_SNFC_EN_DEVS 			(1)
#define D_SNFC_EN_DEV_NAME 		("snfc_en")

/* NFC enable/disable */
#define D_SNFC_DISABLE	 		(0)
#define D_SNFC_ENABLE 			(1)

/* GPIO number */
#define D_UART_TX_GPIO_NO		(27)
#define D_UART_RX_GPIO_NO		(28)
#define D_UART_CTS_GPIO_NO		(29)
#define D_UART_RTS_GPIO_NO		(30)
#define D_VFEL_GPIO_NO			g_vfel_gpio_no

/* VEN */
#define D_VEN_DEV_LOW 			(0)
#define D_VEN_DEV_HIGH 			(1)

/* FIRM */
#define D_FIRM_DEV_LOW 			(0)
#define D_FIRM_DEV_HIGH			(1)

/* VFEL */
#define D_VFEL_DEV_LOW 			(0)
#define D_VFEL_DEV_HIGH 		(1)
#define D_VFEL_LOW_SLEEP_USEC	100000

/* VREG(power) enable/disable */
#define D_VREG_DISABLE 			(0)
#define D_VREG_ENABLE 			(1)
#define D_VREG_INTERVAL_USEC	1000


#define SEC_NFC_VEN_WAIT_TIME 100

#define D_POWCTRL_FLG_TRUE		(1)
#define D_POWCTRL_FLG_FALSE		(0)

/*
 * prototype
 */
static __init int snfc_en_init(void);
static __exit void snfc_en_exit(void);
static int snfc_pvdd_vreg_enable(int enable);
static int snfc_avdd_vreg_enable(int enable);
static int snfc_tvdd_vreg_enable(int enable);

/*
 * global variable
 */
static struct class *snfc_en_class = NULL;
static struct cdev snfc_en_cdev;
static struct device *snfc_en_dev = NULL;

unsigned g_ven_gpio_no = 0;
static unsigned g_vfel_gpio_no = 0;
static char g_snfc_en_state = D_SNFC_DISABLE;

static int g_snfc_powctrl_flg = D_POWCTRL_FLG_TRUE;

/*
 * function_snfc_en
 */

void snfc_change_wakeup_mode(state)
{

	NFC_DRV_DBG_LOG("START state=%d",state);

	if (state == D_WAKEUP_STATE_UP) {
		gpio_tlmm_config(GPIO_CFG(D_WAKEUP_GPIO_NO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), 1);
		NFC_DRV_DBG_LOG("WAKE UP = PULL UP");
	} else {
		gpio_tlmm_config(GPIO_CFG(D_WAKEUP_GPIO_NO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 1);
		NFC_DRV_DBG_LOG("WAKE UP = PULL DOWN");
	}

	NFC_DRV_DBG_LOG("END");
}

int snfc_get_powctrl_flg(void)
{
	return g_snfc_powctrl_flg;
}

static void snfc_gpio_free(unsigned gpio)
{
	int ret;

	NFC_DRV_DBG_LOG("START gpio=%u", gpio);

	gpio_free(gpio);

	ret = gpio_tlmm_config(GPIO_CFG(gpio, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret) {
		NFC_DRV_ERR_LOG("gpio_tlmm_config ret=%d", ret);
	}

	NFC_DRV_DBG_LOG("END");
}

static void snfc_output_disable(int sw)
{
	NFC_DRV_DBG_LOG("START");

	/* UART */
	gpio_tlmm_config(GPIO_CFG(D_UART_TX_GPIO_NO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	gpio_tlmm_config(GPIO_CFG(D_UART_RX_GPIO_NO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	gpio_tlmm_config(GPIO_CFG(D_UART_CTS_GPIO_NO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 1);
	gpio_tlmm_config(GPIO_CFG(D_UART_RTS_GPIO_NO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	gpio_set_value(D_UART_TX_GPIO_NO, 0);
	gpio_set_value(D_UART_RTS_GPIO_NO, 0);

	/* WAKEUP */
	snfc_gpio_free(D_WAKEUP_GPIO_NO);

	/* FIRMWARE */
	gpio_set_value(D_FIRM_GPIO_NO, 0);

	if(sw){
		/* PVDD */
		snfc_pvdd_vreg_enable(D_VREG_DISABLE);

		usleep(D_VREG_INTERVAL_USEC);

		/* VEN */
		gpio_set_value(D_VEN_GPIO_NO, 0);

		/* AVDD */
		snfc_avdd_vreg_enable(D_VREG_DISABLE);
	}

	/* TVDD */
	snfc_tvdd_vreg_enable(D_VREG_DISABLE);

	g_snfc_en_state = D_SNFC_DISABLE;
	g_snfc_powctrl_flg = D_POWCTRL_FLG_FALSE;

	NFC_DRV_DBG_LOG("END");
}

static void snfc_output_enable(void)
{
	int ret;

	NFC_DRV_DBG_LOG("START");

	/* WAKEUP */
	ret = gpio_request(D_WAKEUP_GPIO_NO, "WAKEUP_GPIO request");
	if (ret) {
		NFC_DRV_ERR_LOG("WAKEUP_GPIO ret=%d", ret);
	}

	snfc_change_wakeup_mode(D_WAKEUP_STATE_UP);

	/* PVDD */
	snfc_pvdd_vreg_enable(D_VREG_ENABLE);

	/* AVDD */
	snfc_avdd_vreg_enable(D_VREG_ENABLE);

	/* TVDD */
	snfc_tvdd_vreg_enable(D_VREG_ENABLE);


	g_snfc_en_state = D_SNFC_ENABLE;
	g_snfc_powctrl_flg = D_POWCTRL_FLG_TRUE;

	NFC_DRV_DBG_LOG("END");
}

static void snfc_chip_reset(void)
{
	NFC_DRV_DBG_LOG("START");

	gpio_set_value(D_VFEL_GPIO_NO, D_VFEL_DEV_HIGH);

	usleep(D_VFEL_LOW_SLEEP_USEC);

	gpio_set_value(D_VFEL_GPIO_NO, D_VFEL_DEV_LOW);

	NFC_DRV_DBG_LOG("END");
}

static int snfc_pvdd_vreg_enable(int enable)
{
	struct regulator *reg;
	struct device *dev = snfc_en_dev;
	const char *id = "8941_lvs3";
	int min_uV = 1800000, max_uV = 1800000;

	NFC_DRV_DBG_LOG("START enable=%d", enable);

	reg = regulator_get(dev, id);
	if (IS_ERR(reg)) {
		NFC_DRV_ERR_LOG("Unable to get %s regulator", id);
		return -1;
	}

	if (enable) {
	    regulator_set_voltage(reg, min_uV, max_uV);

		if (!regulator_is_enabled(reg)) {
			regulator_enable(reg);
		}
	} else {
		if (regulator_is_enabled(reg)) {
			regulator_disable(reg);
		}
	}

	regulator_put(reg);

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static int snfc_avdd_vreg_enable(int enable)
{
	struct regulator *reg;
	struct device *dev = snfc_en_dev;
	const char *id = "8941_lvs2";
	int min_uV = 1800000, max_uV = 1800000;

	NFC_DRV_DBG_LOG("START enable=%d", enable);

	reg = regulator_get(dev, id);
	if (IS_ERR(reg)) {
		NFC_DRV_ERR_LOG("Unable to get %s regulator", id);
		return -1;
	}

	if (enable) {
	    regulator_set_voltage(reg, min_uV, max_uV);

		if (!regulator_is_enabled(reg)) {
			regulator_enable(reg);
		}
	} else {
		if (regulator_is_enabled(reg)) {
			regulator_disable(reg);
		}
	}

	regulator_put(reg);

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static int snfc_tvdd_vreg_enable(int enable)
{
	struct regulator *reg;
	struct device *dev = snfc_en_dev;
	const char *id = "8941_l18";
	int min_uV = 3100000, max_uV = 3100000;

	NFC_DRV_DBG_LOG("START enable=%d", enable);

	reg = regulator_get(dev, id);
	if (IS_ERR(reg)) {
		NFC_DRV_ERR_LOG("Unable to get %s regulator", id);
		return -1;
	}

	if (enable) {
	    regulator_set_voltage(reg, min_uV, max_uV);

		if (!regulator_is_enabled(reg)) {
			regulator_enable(reg);
		}
	} else {
		if (regulator_is_enabled(reg)) {
			regulator_disable(reg);
		}
	}

	regulator_put(reg);

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static ssize_t snfc_en_read(struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
	char on[2];

	NFC_DRV_DBG_LOG("START");

	/* length check */
	if (len < 1) {
		NFC_DRV_ERR_LOG("length check len = %d", len);
		return -EIO;
	}

	on[0] = g_snfc_en_state;
	on[1] = 0x00;

	if (len > 2) {
		len = 2;
	}

	if (copy_to_user(buf, on, len)) {
		NFC_DRV_ERR_LOG("copy_to_user");
		return -EFAULT;
	}

	NFC_DRV_DBG_LOG("END on=%d, len=%d", on[0], len);

	return len;
}

ssize_t snfc_en_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char on;

	NFC_DRV_DBG_LOG("START");

	/* length check */
	if (len < 1) {
		NFC_DRV_ERR_LOG("length check len=%d", len);
		return -EIO;
	}

	if (copy_from_user(&on, data, 1)) {
		NFC_DRV_ERR_LOG("copy_from_user");
		return -EFAULT;
	}

	if (on == g_snfc_en_state) {
		NFC_DRV_DBG_LOG("g_snfc_en_state equals to on, do nothing");
	} else if (on == D_SNFC_ENABLE) {
		snfc_output_enable();
	} else if (on == D_SNFC_DISABLE) {
		snfc_output_disable(0);
	} else if (on == 2) {
		snfc_output_disable(1);
	} else {
		NFC_DRV_ERR_LOG("on=%d", on);
		return -EFAULT;
	}

	NFC_DRV_DBG_LOG("END on=%d, g_snfc_en_state=%d", on, g_snfc_en_state);

	return len;
}

static long snfc_en_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	NFC_DRV_DBG_LOG("START cmd=%u", cmd);

	switch (cmd) {
	case SHSNFC_EN_REQ_CHIPRESET:
		snfc_chip_reset();
		break;
	case SHSNFC_EN_REQ_PVDD_ENABLE:
		snfc_pvdd_vreg_enable((int)arg);
		break;
	case SHSNFC_EN_REQ_AVDD_ENABLE:
		snfc_avdd_vreg_enable((int)arg);
		break;
	case SHSNFC_EN_REQ_TVDD_ENABLE:
		snfc_tvdd_vreg_enable((int)arg);
		break;
	case SHSNFC_EN_REQ_VEN_ENABLE:
		if (arg) {
			gpio_set_value(D_VEN_GPIO_NO, D_VEN_DEV_HIGH);
		} else {
			gpio_set_value(D_VEN_GPIO_NO, D_VEN_DEV_LOW);
		}
		break;
	case SHSNFC_EN_REQ_FIRM_ENABLE:
		if (arg) {
			gpio_set_value(D_FIRM_GPIO_NO, D_FIRM_DEV_HIGH);
		} else {
			gpio_set_value(D_FIRM_GPIO_NO, D_FIRM_DEV_LOW);
		}
		break;
	case SHSNFC_EN_GET_UART_RX_VALUE:
		ret = gpio_get_value(D_UART_RX_GPIO_NO);
		return ret;
	default:
		NFC_DRV_ERR_LOG("cmd unhandled");
		return -EINVAL;
	}

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static int snfc_en_open(struct inode *inode, struct file *file)
{
	NFC_DRV_DBG_LOG("");
	return 0;
}

static int snfc_en_release(struct inode *inode, struct file *file)
{
	NFC_DRV_DBG_LOG("");
	return 0;
}

static const struct file_operations snfc_en_fileops = {
	.owner          = THIS_MODULE,
	.read           = snfc_en_read,
	.write          = snfc_en_write,
	.unlocked_ioctl = snfc_en_ioctl,
	.open           = snfc_en_open,
	.release        = snfc_en_release,
};

/*
 * snfc_en_init
 */
static __init int snfc_en_init(void)
{
	int ret = 0;
	dev_t dev = MKDEV(MISC_MAJOR, 0);

	NFC_DRV_DBG_LOG("START");

	snfc_en_class = class_create(THIS_MODULE, "snfc_en");
	if (IS_ERR(snfc_en_class)) {
		ret = PTR_ERR(snfc_en_class);
		NFC_DRV_ERR_LOG("class_create ret=%d", ret);
		return ret;
	}

	ret = alloc_chrdev_region(&dev, 0, D_SNFC_EN_DEVS, D_SNFC_EN_DEV_NAME);
	if (ret) {
		NFC_DRV_ERR_LOG("alloc_chrdev_region ret=%d", ret);
		return ret;
	}

	cdev_init(&snfc_en_cdev, &snfc_en_fileops);
	snfc_en_cdev.owner = THIS_MODULE;

	ret = cdev_add(&snfc_en_cdev, dev, D_SNFC_EN_DEVS);
	if (ret) {
		unregister_chrdev_region(dev, D_SNFC_EN_DEVS);
		NFC_DRV_ERR_LOG("cdev_add ret=%d", ret);
		return ret;
	}

	snfc_en_dev = device_create(snfc_en_class, NULL, dev, NULL, D_SNFC_EN_DEV_NAME);
	if (IS_ERR(snfc_en_dev)) {
		cdev_del(&snfc_en_cdev);
		unregister_chrdev_region(dev, D_SNFC_EN_DEVS);
		ret = PTR_ERR(snfc_en_dev);
		NFC_DRV_ERR_LOG("device_create ret=%d", ret);
		return ret;
	}

	// get GPIO number in PMIC.
	g_ven_gpio_no = qpnp_pin_map("pm8941-gpio", 8);
	g_vfel_gpio_no = qpnp_pin_map("pm8941-mpp", 8);

	// disable GPIOs.
	snfc_output_disable(1);

	NFC_DRV_DBG_LOG("END");

	return ret;
}

/*
 * snfc_en_exit
 */
static __exit void snfc_en_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);

	NFC_DRV_DBG_LOG("START");

	cdev_del(&snfc_en_cdev);
	unregister_chrdev_region(dev, D_SNFC_EN_DEVS);
	class_destroy(snfc_en_class);

	NFC_DRV_DBG_LOG("END");
}

MODULE_LICENSE("GPL v2");

module_init(snfc_en_init);
module_exit(snfc_en_exit);

