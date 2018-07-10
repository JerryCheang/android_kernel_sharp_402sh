/*
 * SAMSUNG NFC Controller
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Author: Woonki Lee <woonki84.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/wait.h>
#include <linux/delay.h>

#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/sched.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/nfc/sec_nfc.h>

#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/wakelock.h>
#include "nfc.h"

#define D_SEC_NFC_FN_DEVS				(1)

#define GPIO_PUSH_PIN (59)

enum push_state {
	PUSH_NONE,
	PUSH_ON,
};

struct sec_nfc_fn_info {
	struct miscdevice miscdev;
	struct device *dev;
	struct sec_nfc_fn_platform_data *pdata;

	struct mutex push_mutex;
	enum push_state push_irq;
	wait_queue_head_t push_wait;

	struct mutex confirm_mutex;
	enum readable_state readable;
};

static struct class *sec_nfc_fn_class = NULL;
static struct cdev sec_nfc_fn_cdev;

static struct sec_nfc_fn_info g_nfc_fn_info;

static struct wake_lock g_wake_lock;

static int sec_nfc_fn_driver_register(void);
static void sec_nfc_fn_driver_unregister(void);

static irqreturn_t sec_nfc_fn_push_thread_fn(int irq, void *dev_id)
{
	struct sec_nfc_fn_info *info = dev_id;

	dev_dbg(info->dev, "PUSH\n");
    
	mutex_lock(&info->push_mutex);
	info->push_irq = PUSH_ON;
	mutex_unlock(&info->push_mutex);

	wake_up_interruptible(&info->push_wait);

	if(!wake_lock_active(&g_wake_lock))
	{
		NFC_DRV_DBG_LOG("Set wake_lock_timeout for 800 msec. !!!");
		wake_lock_timeout(&g_wake_lock,((HZ*4)/5));
	}
	return IRQ_HANDLED;
}

static unsigned int sec_nfc_fn_poll(struct file *file, poll_table *wait)
{
	struct sec_nfc_fn_info *info = container_of(file->private_data,
						struct sec_nfc_fn_info, miscdev);
	enum push_state push;

	int ret = 0;

	dev_dbg(info->dev, "%s: info: %p\n", __func__, info);

	poll_wait(file, &info->push_wait, wait);

	mutex_lock(&info->push_mutex);

	push = info->push_irq;
	if (push == PUSH_ON)
		ret = (POLLIN | POLLRDNORM);

	mutex_unlock(&info->push_mutex);

	return ret;
}

static long sec_nfc_fn_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct sec_nfc_fn_info *info = container_of(file->private_data,
						struct sec_nfc_fn_info, miscdev);

        void __user *argp = (void __user *)arg;
	int ret = 0;
        int state = 0;

	dev_dbg(info->dev, "%s: info: %p, cmd: 0x%x\n",
			__func__, info, cmd);

	switch (cmd) {
	case SEC_NFC_GET_PUSH:
		dev_dbg(info->dev, "%s: sec_nfc_fn_GET_PUSH\n", __func__);

		mutex_lock(&info->push_mutex);

                state = gpio_get_value(GPIO_PUSH_PIN);

// Get push pin state
//		if (copy_to_user( (unsigned char *)arg, &info->push_irq,
//			sizeof(info->push_irq)) != 0) {
//			dev_err(info->dev, "copy failed to user\n");
//               }

                NFC_DRV_DBG_LOG(" copy push pin value - state : %d", state);
		if (copy_to_user(argp, &state,
			sizeof(state)) != 0) {
			dev_err(info->dev, "copy failed to user\n");
		}
		info->push_irq = PUSH_NONE;
		mutex_unlock(&info->push_mutex);

		break;
	default:
		dev_err(info->dev, "Unknow ioctl 0x%x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int sec_nfc_fn_open(struct inode *inode, struct file *file)
{
//	struct sec_nfc_fn_info *info = container_of(file->private_data,
//						struct sec_nfc_fn_info, miscdev);
	struct sec_nfc_fn_info *info = &g_nfc_fn_info;
	int ret = 0;

	file->private_data = &info->miscdev;
	NFC_DRV_DBG_LOG("START");
	dev_dbg(info->dev, "%s: info : %p" , __func__, info);

	mutex_lock(&info->push_mutex);
	info->push_irq = PUSH_NONE;
	mutex_unlock(&info->push_mutex);

	mutex_lock(&info->confirm_mutex);
	info->readable = RDABLE_YES;
	mutex_unlock(&info->confirm_mutex);

	NFC_DRV_DBG_LOG("END");
	return ret;
}

static int sec_nfc_fn_close(struct inode *inode, struct file *file)
{
	struct sec_nfc_fn_info *info = container_of(file->private_data,
						struct sec_nfc_fn_info, miscdev);

	NFC_DRV_DBG_LOG("START");
	dev_dbg(info->dev, "%s: info : %p" , __func__, info);

	mutex_lock(&info->confirm_mutex);
	info->readable = RDABLE_NO;
	mutex_unlock(&info->confirm_mutex);

	NFC_DRV_DBG_LOG("END");
	return 0;
}

static const struct file_operations sec_nfc_fn_fops = {
	.owner		= THIS_MODULE,
	.poll		= sec_nfc_fn_poll,
	.open		= sec_nfc_fn_open,
	.release	= sec_nfc_fn_close,
	.unlocked_ioctl	= sec_nfc_fn_ioctl,
};

#if 0
#ifdef CONFIG_PM
static int sec_nfc_fn_suspend(struct device *dev)
{
	struct sec_nfc_fn_platform_data *pdata = dev->platform_data;
	struct sec_nfc_fn_info *info = dev_get_drvdata(dev);

	pdata->cfg_gpio();

	mutex_lock(&info->confirm_mutex);
	info->readable = RDABLE_NO;
	mutex_unlock(&info->confirm_mutex);

	return 0;
}

static int sec_nfc_fn_resume(struct device *dev)
{
	struct sec_nfc_fn_platform_data *pdata = dev->platform_data;

	pdata->cfg_gpio();

	return 0;
}

static SIMPLE_DEV_PM_OPS(sec_nfc_fn_pm_ops, sec_nfc_fn_suspend, sec_nfc_fn_resume);
#endif

static int __devinit sec_nfc_fn_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sec_nfc_fn_info *info;
	struct sec_nfc_fn_platform_data *pdata = dev->platform_data;
	int ret = 0;

	if (!pdata) {
		dev_err(dev, "No platform data\n");
		ret = -ENOMEM;
		goto err_pdata;
	}

	info = kzalloc(sizeof(struct sec_nfc_fn_info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "failed to allocate memory for sec_nfc_fn_info\n");
		ret = -ENOMEM;
		goto err_info_alloc;
	}
	info->dev = dev;
	info->pdata = pdata;

	dev_set_drvdata(dev, info);

	pdata->cfg_gpio();

	mutex_init(&info->push_mutex);
	mutex_init(&info->confirm_mutex);
	init_waitqueue_head(&info->push_wait);
	info->push_irq = PUSH_NONE;

	ret = request_threaded_irq(pdata->push, NULL, sec_nfc_fn_push_thread_fn,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, SEC_NFC_FN_DRIVER_NAME,
			info);
	if (ret < 0 ) {
		dev_err(dev, "failed to register PUSH handler\n");
		goto err_push_req;
	}

	ret = enable_irq_wake(pdata->push);
	if (ret) {
		dev_err(dev, "failed to register wakeup irq (0x%02x), ret: %d\n",
			pdata->push, ret);
		goto err_push_wake;
	}

	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = SEC_NFC_FN_DRIVER_NAME;
	info->miscdev.fops = &sec_nfc_fn_fops;
	info->miscdev.parent = dev;
	ret = misc_register(&info->miscdev);
	if (ret < 0) {
		dev_err(dev, "failed to register Device\n");
		goto err_dev_reg;
	}

	dev_dbg(dev, "%s: info: %p, pdata %p\n", __func__, info, pdata);

	return 0;

err_dev_reg:
err_push_wake:
err_push_req:
err_info_alloc:
	kfree(info);
err_pdata:
	return ret;
}

static int __devexit sec_nfc_fn_remove(struct platform_device *pdev)
{
	struct sec_nfc_fn_info *info = dev_get_drvdata(&pdev->dev);
	struct sec_nfc_fn_platform_data *pdata = pdev->dev.platform_data;

	dev_dbg(info->dev, "%s\n", __func__);

	misc_deregister(&info->miscdev);
	free_irq(pdata->push, info);
	kfree(info);

	return 0;
}

static struct platform_device_id sec_nfc_fn_id_table[] = {
	{ SEC_NFC_FN_DRIVER_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(platform, sec_nfc_fn_id_table);
static struct platform_driver sec_nfc_fn_driver = {
	.probe = sec_nfc_fn_probe,
	.id_table = sec_nfc_fn_id_table,
	.remove = sec_nfc_fn_remove,
	.driver = {
		.name = SEC_NFC_FN_DRIVER_NAME,
#ifdef CONFIG_PM
		.pm = &sec_nfc_fn_pm_ops,
#endif
	},
};
#endif

static int __init sec_nfc_fn_init(void)
{
#if 0
	return platform_driver_register(&sec_nfc_fn_driver);
#else
	return sec_nfc_fn_driver_register();
#endif
}

static void __exit sec_nfc_fn_exit(void)
{
#if 0
	platform_driver_unregister(&sec_nfc_fn_driver);
#else
	sec_nfc_fn_driver_unregister();
#endif
}

static int sec_nfc_fn_driver_register(void)
{
	int ret = 0;
	struct device *class_dev;
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	int push = gpio_to_irq(D_WAKEUP_GPIO_NO);

	NFC_DRV_DBG_LOG("START");

	sec_nfc_fn_class =  class_create(THIS_MODULE, "sec_nfc_fn");
	if (IS_ERR(sec_nfc_fn_class)) {
		ret = PTR_ERR(sec_nfc_fn_class);
		NFC_DRV_ERR_LOG("class_create ret=%d", ret);
		return ret;
	}

	ret = alloc_chrdev_region(&dev, 0, D_SEC_NFC_FN_DEVS, SEC_NFC_FN_DRIVER_NAME);
	if (ret) {
		NFC_DRV_ERR_LOG("alloc_chrdev_region ret=%d", ret);
		return ret;
	}

	cdev_init(&sec_nfc_fn_cdev, &sec_nfc_fn_fops);
	sec_nfc_fn_cdev.owner = THIS_MODULE;

	ret = cdev_add(&sec_nfc_fn_cdev, dev, D_SEC_NFC_FN_DEVS);
	if (ret) {
		unregister_chrdev_region(dev, D_SEC_NFC_FN_DEVS);
		NFC_DRV_ERR_LOG("cdev_add ret=%d", ret);
		return ret;
	}

	class_dev = device_create(sec_nfc_fn_class, NULL, dev, NULL, SEC_NFC_FN_DRIVER_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&sec_nfc_fn_cdev);
		unregister_chrdev_region(dev, D_SEC_NFC_FN_DEVS);
		ret = PTR_ERR(class_dev);
		NFC_DRV_ERR_LOG("device_create ret=%d", ret);
		return ret;
	}

	g_nfc_fn_info.dev = class_dev;
	g_nfc_fn_info.pdata = NULL;

	mutex_init(&g_nfc_fn_info.push_mutex);
	mutex_init(&g_nfc_fn_info.confirm_mutex);
	init_waitqueue_head(&g_nfc_fn_info.push_wait);
	g_nfc_fn_info.push_irq = PUSH_NONE;

	ret = request_threaded_irq(push, NULL, sec_nfc_fn_push_thread_fn,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, SEC_NFC_FN_DRIVER_NAME,
			&g_nfc_fn_info);
	if (ret < 0 ) {
		NFC_DRV_ERR_LOG("failed to register PUSH handler");
	}

	ret = enable_irq_wake(push);
	if (ret) {
		NFC_DRV_ERR_LOG("failed to register wakeup irq (0x%02x), ret: %d",
			push, ret);
	}

	wake_lock_init(&g_wake_lock, WAKE_LOCK_SUSPEND, "NFCWAKE");

	NFC_DRV_DBG_LOG("END");

	return 0;
}

static void sec_nfc_fn_driver_unregister(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	int push = gpio_to_irq(D_WAKEUP_GPIO_NO);

	NFC_DRV_DBG_LOG("START");

	free_irq(push, &g_nfc_fn_info);

	cdev_del(&sec_nfc_fn_cdev);
	unregister_chrdev_region(dev, D_SEC_NFC_FN_DEVS);
	class_destroy(sec_nfc_fn_class);

	if (wake_lock_active(&g_wake_lock)) {
		 wake_unlock(&g_wake_lock);
	}
	wake_lock_destroy(&g_wake_lock);

	NFC_DRV_DBG_LOG("END");
}

module_init(sec_nfc_fn_init);
module_exit(sec_nfc_fn_exit);

MODULE_DESCRIPTION("Samsung sec_nfc_fn driver");
MODULE_LICENSE("GPL");

