/* drivers/sharp/shgrip/shgrip_kerl.c
 *
 * Copyright (C) 2014 Sharp Corporation
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
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/input/mt.h>
#include <linux/ioctl.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>

#ifndef SHGRIP_FACTORY_MODE_ENABLE
#include <sharp/shtps_dev.h>
#endif /* SHGRIP_FACTORY_MODE_ENABLE */

#ifdef CONFIG_SHTERM
#include <sharp/shterm_k.h>
#endif /* CONFIG_SHTERM */

#include "shgrip_kerl.h"
#include "shgrip_fw.h"

#include <linux/pm_qos.h>
#include <mach/cpuidle.h>

/* ------------------------------------------------------------------------- */
/* DEFINE                                                                    */
/* ------------------------------------------------------------------------- */
/* --------------------------------------------------------- */
/* Debug Parameter                                           */
/* --------------------------------------------------------- */

/* --------------------------------------------------------- */
/* Software Parameter                                        */
/* --------------------------------------------------------- */
#define SHGRIP_NAME							"shgrip"
#define SHGRIP_DEVNAME						"shgrip_dev"

#define SHGRIP_FW_PACKET_SIZE				(64)

#define SHGRIP_IRQ_DISABLED					(0)
#define SHGRIP_IRQ_ENABLED					(1)
#define SHGRIP_IRQ_FREE						(0)
#define SHGRIP_IRQ_REQUESTED				(1)
#define SHGRIP_WAKE_DISABLED				(0)
#define SHGRIP_WAKE_ENABLED					(1)

/* --------------------------------------------------------- */
/* System Parameter                                          */
/* --------------------------------------------------------- */
#define SPI_BUFFER							(65)
#define SPI_CLK_SPEED						(4000000)
#define SPI_BIT_WORD						(8)

#define SHGRIP_PM_QOS_LATENCY_VALUE			(1)

#define SHGRIP_GPIO_VAL_LO					(0)
#define SHGRIP_GPIO_VAL_HI					(1)

#define SHGRIP_SPI_MOSI						(53)
#define SHGRIP_SPI_MISO						(54)
#define SHGRIP_SPI_CS_N						(55)
#define SHGRIP_SPI_CLK						(56)

#define SHGRIP_GPIO_GRIP_INT				(65)

#define SHGRIP_GPIO_GRIP_RESET				qpnp_pin_map("pm8941-gpio", 16)

/* --------------------------------------------------------- */
/* Timer Parameter                                           */
/* --------------------------------------------------------- */
#define SHGRIP_SPI_WAKEUP_WAIT_US			(15)
#define SHGRIP_SPI_READ_DELAY_US			(50)
#define SHGRIP_SPI_PROGRAMMING_DELAY_US_RAM	(20)
#define SHGRIP_SPI_PROGRAMMING_DELAY_US		(10*1000)
#define SHGRIP_WAIT_BYTE_US					(50)
#define SHGRIP_WAIT_FW_WRITE_US				(12*1000)
#define SHGRIP_RESET_WAIT_US				(1000)
#define SHGRIP_RESET_START_WAIT_US			(10*1000) /* T.B.D */

#define SHGRIP_RESET_INT_WAIT_US			(2*1000)

#define SHGRIP_RESET_START_APP_WAIT_US		(20*1000)
#define SHGRIP_CHECK_SUM_WAIT_US			(100*1000)
/* --------------------------------------------------------- */
/* Device Parameter                                          */
/* --------------------------------------------------------- */
#define BIT0								(0x01)
#define BIT1								(0x02)
#define BIT2								(0x04)
#define BIT3								(0x08)
#define BIT4								(0x10)
#define BIT5								(0x20)
#define BIT6								(0x40)
#define BIT7								(0x80)

#define SHGRIP_ENABLE_CHANNEL				(BIT4 | BIT0)

#define SHGRIP_CMD_ACK						(0x22)
#define SHGRIP_CMD_NACK						(0x33)
#define SHGRIP_CMD_APP_ACK					(0x79)
#define SHGRIP_CMD_APP_NACK					(0x1F)

#define SHGRIP_CMD_FW_DL					(0x31)
#define SHGRIP_CMD_CHECK_SUM				(0xC3)

#define SHGRIP_CMD_CHG_LDR_MODE				(0x77)
#define SHGRIP_CMD_CHG_APP_MODE				(0x88)
#define SHGRIP_CMD_LDR_CHECK_SUM			(0x99)

#define SHGRIP_CMD_WRITE_ADDR				(0x8000)

#define SHGRIP_CMD_BOOTVER					(0x0000)
#define SHGRIP_CMD_DEVFWREVR1				(0x0010)

#define SHGRIP_CMD_LPCR						(0x4000)
#define SHGRIP_CMD_ACQRATER					(0x4004)
#define SHGRIP_CMD_RSTSRC					(0x4005)
#define SHGRIP_CMD_CH0CR					(0x4044)
#define SHGRIP_CMD_CH0SIGRH					(0x4046)
#define SHGRIP_CMD_CH4CR					(0x4064)
#define SHGRIP_CMD_CH4SIGRH					(0x4066)
#define SHGRIP_CMD_CHENR					(0x4090)
#define SHGRIP_CMD_TRIGCCR					(0x4091)
#define SHGRIP_CMD_GRIPSCR					(0x4200)
#define SHGRIP_CMD_GRIPSR					(0x4203)
#define SHGRIP_CMD_DELAYGRIPRH				(0x4207)
#define SHGRIP_CMD_CH0PTHRH					(0x40A0)
#define SHGRIP_CMD_CH4PTHRH					(0x40E0)
#define SHGRIP_CMD_CH0SR					(0x4045)
#define SHGRIP_CMD_CH4SR					(0x4065)
#define SHGRIP_CMD_AET_REPEATR				(0x4311)

#define SHGRIP_FW_ADDR_LDR_BLK				(0x8000)
#define SHGRIP_FW_ADDR_LDR_BLK_TERMINATE	(0x84FF)
#define SHGRIP_FW_ADDR_APP_BLK				(0x8500)
#define SHGRIP_FW_ADDR_APP_BLK_TERMINATE	(0xBFFF)

#define SHGRIP_FW_START_ADDR				SHGRIP_FW_ADDR_APP_BLK
#define SHGRIP_FW_END_ADDR					SHGRIP_FW_ADDR_APP_BLK_TERMINATE

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
static int __devinit shgrip_dev_spi_probe(struct spi_device *spi);
static int __devexit shgrip_dev_spi_remove(struct spi_device *spi);

static irqreturn_t shgrip_irq_func(int irq, void *dev);

static int shgrip_open(struct inode *inode, struct file *file);
static int shgrip_close(struct inode *inode, struct file *file);
static int shgrip_read(struct file* filp, char* buf, size_t count, loff_t* offset);
static int shgrip_write(struct file* filp, const char* buf, size_t count, loff_t* offset);
static long shgrip_ioctl(struct file *file, unsigned int cmd, unsigned long arg);


/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
/* --------------------------------------------------------- */
/* Serial Interface Parameter                                */
/* --------------------------------------------------------- */
static struct spi_device *spi_dev = NULL;

#ifdef CONFIG_OF
static const struct of_device_id shgrip_dev_dt_match[] = {
	{ .compatible = "sharp,shgrip_dev",},
	{}
};
#else
#define shgrip_dev_dt_match NULL;
#endif /* CONFIG_OF */

static int shgrip_suspend(struct spi_device *spi, pm_message_t mesg);
static int shgrip_resume(struct spi_device *spi);

static struct spi_driver shgrip_dev_spi_driver = {
	.driver = {
		.name = SHGRIP_DEVNAME,
		.owner = THIS_MODULE,
		.of_match_table = shgrip_dev_dt_match,
	},
	.probe = shgrip_dev_spi_probe,
	.remove = __devexit_p(shgrip_dev_spi_remove),
	.suspend = shgrip_suspend,
	.resume = shgrip_resume,
};

/* --------------------------------------------------------- */
/* file_operations                                           */
/* --------------------------------------------------------- */
static struct file_operations shgrip_fops = {
	.owner          = THIS_MODULE,
	.open           = shgrip_open,
	.release        = shgrip_close,
	.read           = shgrip_read,
	.write          = shgrip_write,
	.unlocked_ioctl = shgrip_ioctl,
};

/* --------------------------------------------------------- */
/* Software Parameter                                        */
/* --------------------------------------------------------- */
static bool shgrip_dev_connect = false;

static int shgrip_irq_request  = SHGRIP_IRQ_FREE;
static int shgrip_irq_enable   = SHGRIP_IRQ_DISABLED;
static int shgrip_irq_wake     = SHGRIP_WAKE_DISABLED;

static unsigned char shgrip_status = SHGRIP_STATE_POWER_OFF;
static bool shgrip_suspend_state = false;

static struct shgrip_sens_setting_params bk_adj;

static struct workqueue_struct	*shgrip_work_queue;
static struct work_struct		shgrip_work_data;

static spinlock_t shgrip_spinlock;
static struct mutex shgrip_mutex_lock;
static struct wake_lock shgrip_wake_lock;

static struct pm_qos_request shgrip_qos_cpu_dma_latency;

static struct input_dev		*shgrip_input;
static dev_t 				shgrip_dev;
static dev_t				shgrip_major = 0;
static dev_t				shgrip_minor = 0;
static struct cdev 			shgrip_cdev;
static struct class* 		shgrip_class;
static struct device*		shgrip_device;

static bool shgrip_set_sensor_adjust_flg = false;
static int shgrip_rcovery_state_flg = 0;

int shgrip_err_log  = 1;
int shgrip_warn_log = 0;
int shgrip_info_log = 0;
int shgrip_dbg_log  = 0;

#if defined (CONFIG_ANDROID_ENGINEERING)
static struct hrtimer			shgrip_threshold_timer;
static struct work_struct		th_work_data;
#endif /* CONFIG_ANDROID_ENGINEERING */

#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(shgrip_err_log,  int, 0600);
module_param(shgrip_warn_log, int, 0600);
module_param(shgrip_info_log, int, 0600);
module_param(shgrip_dbg_log,  int, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */

int shgrip_dbg_checksum = SHGRIP_FW_CHECK_SUM_VAL;
#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(shgrip_dbg_checksum,  int, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */

#if defined (CONFIG_ANDROID_ENGINEERING)
static unsigned long shgrip_th_interval_ms = 0;
static unsigned long th_msec = 0;
static unsigned long th_msec_tmp = 0;
static unsigned char hrtimer_flg = 0;
static struct wake_lock shgrip_dump_wake_lock;
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHGRIP_ERR(fmt, args...) \
		if(shgrip_err_log == 1) { \
			printk("[SHGRIP_ERROR][%s] " fmt, __func__, ## args); \
		}

#define SHGRIP_WARN(fmt, args...) \
		if(shgrip_warn_log == 1) { \
			printk("[SHGRIP_WARN][%s] " fmt, __func__, ## args); \
		}

#define SHGRIP_INFO(fmt, args...) \
		if(shgrip_info_log == 1) { \
			printk("[SHGRIP_INFO][%s] " fmt, __func__, ## args); \
		}

#define SHGRIP_DBG(fmt, args...) \
		if(shgrip_dbg_log == 1) { \
			printk("[SHGRIP_DBG][%s] " fmt, __func__, ## args); \
		}


/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */
/* ========================================================================= */
/* SHGRIP ATTRIBUTE                                                          */
/* ========================================================================= */
#ifdef CONFIG_ANDROID_ENGINEERING
static ssize_t shgrip_dev_store(struct device *dev, struct device_attribute *attr, 
			const char *buf, size_t count)
{
	unsigned long sec,nsec;
	int ret;
	
	mutex_lock(&shgrip_mutex_lock);
	
	ret = strict_strtoul(buf, 10, &shgrip_th_interval_ms);
	if (ret) {
		mutex_unlock(&shgrip_mutex_lock);
		return -EINVAL;
	}

	if (shgrip_th_interval_ms) {
		if (shgrip_th_interval_ms < 50) {
			SHGRIP_WARN("interval time is Err, Set default 50ms\n");
			shgrip_th_interval_ms = 50;
		}
		
		if(th_msec_tmp == 0){
			th_msec = shgrip_th_interval_ms;
			th_msec_tmp = shgrip_th_interval_ms;
		} else {
			th_msec_tmp = shgrip_th_interval_ms;
		}
		
		switch (shgrip_status) { 
		case SHGRIP_STATE_SENSOR_ON:
			if (th_msec >= 1000) {
				sec  = th_msec / 1000;
				nsec = (th_msec % 1000) * 1000 * 1000;
			} else {
				sec  = 0;
				nsec = th_msec * 1000 * 1000;
			}
			if (hrtimer_flg == 0) {
				wake_lock(&shgrip_dump_wake_lock);
				hrtimer_start(&shgrip_threshold_timer, ktime_set(sec, nsec), HRTIMER_MODE_REL);
				hrtimer_flg = 1;
			}
			break;
			
		case SHGRIP_STATE_POWER_OFF:
		case SHGRIP_STATE_SENSOR_OFF:
		case SHGRIP_STATE_FW_DL:
		case SHGRIP_STATE_HALT_MODE:
		default:
			th_msec = th_msec_tmp;
			break;
		}
	} else {
		th_msec = shgrip_th_interval_ms;
		th_msec_tmp = shgrip_th_interval_ms;
		hrtimer_flg = 0;
	}
	
	mutex_unlock(&shgrip_mutex_lock);
	
	return strlen(buf);
}

static DEVICE_ATTR( shgrip_dev, ( S_IWUSR | S_IWGRP ) , NULL, shgrip_dev_store );
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ========================================================================= */
/* SHGRIP System Function                                                    */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_sys_delay_us                                                       */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_delay_us(unsigned long usec)
{
	struct timespec tu;
	
	if (usec >= 1000*1000) {
		tu.tv_sec  = usec / 1000000;
		tu.tv_nsec = (usec % 1000000) * 1000;
	}
	else {
		tu.tv_sec  = 0;
		tu.tv_nsec = usec * 1000;
	}
	
	hrtimer_nanosleep(&tu, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_request_irq                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_sys_request_irq(void)
{
	int ret = 0;
	unsigned long irq_flags;
	
	if (shgrip_irq_request == SHGRIP_IRQ_FREE) {
		irq_set_status_flags(gpio_to_irq(SHGRIP_GPIO_GRIP_INT), IRQ_NOAUTOEN);
		shgrip_irq_enable = SHGRIP_IRQ_DISABLED;
		
		irq_flags = IRQF_TRIGGER_HIGH;
		ret = request_irq(gpio_to_irq(SHGRIP_GPIO_GRIP_INT), shgrip_irq_func, 
									irq_flags, SHGRIP_NAME, NULL);
		if (ret) {
			SHGRIP_ERR("request_irq is failed. ret:%d\n", ret);
		} else {
			shgrip_irq_request = SHGRIP_IRQ_REQUESTED;
			SHGRIP_DBG("request_irq(gpio_to_irq(SHGRIP_IRQ_GRIP_INT)) \n");
		}
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_free_irq                                                       */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_free_irq(void)
{
	if (shgrip_irq_request == SHGRIP_IRQ_REQUESTED) {
		free_irq(gpio_to_irq(SHGRIP_GPIO_GRIP_INT), 0);
		shgrip_irq_request = SHGRIP_IRQ_FREE;
		SHGRIP_DBG("free_irq(gpio_to_irq(SHGRIP_IRQ_GRIP_INT)) \n");
	}
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_enable_irq                                                     */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_enable_irq(void)
{
	unsigned long flags;
	
	spin_lock_irqsave(&shgrip_spinlock, flags);
	
	if (shgrip_irq_enable == SHGRIP_IRQ_DISABLED) {
		enable_irq(gpio_to_irq(SHGRIP_GPIO_GRIP_INT));
		shgrip_irq_enable = SHGRIP_IRQ_ENABLED;
		SHGRIP_DBG("enable_irq(gpio_to_irq(SHGRIP_IRQ_GRIP_INT)) \n");
	}
	
	spin_unlock_irqrestore(&shgrip_spinlock, flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_disable_irq                                                    */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_disable_irq(void)
{
	unsigned long flags;
	
	spin_lock_irqsave(&shgrip_spinlock, flags);
	
	if (shgrip_irq_enable == SHGRIP_IRQ_ENABLED) {
		disable_irq_nosync(gpio_to_irq(SHGRIP_GPIO_GRIP_INT));
		shgrip_irq_enable = SHGRIP_IRQ_DISABLED;
		SHGRIP_DBG("disable_irq(SHGRIP_IRQ_GRIP_INT) \n");
	}
	
	spin_unlock_irqrestore(&shgrip_spinlock, flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_enable_irq_wake                                                */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_enable_irq_wake(void)
{
	unsigned long flags;
	
	spin_lock_irqsave(&shgrip_spinlock, flags);
	
	if (shgrip_irq_wake == SHGRIP_WAKE_DISABLED) {
		enable_irq_wake(gpio_to_irq(SHGRIP_GPIO_GRIP_INT));
		shgrip_irq_wake = SHGRIP_WAKE_ENABLED;
	}
	
	spin_unlock_irqrestore(&shgrip_spinlock, flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_disable_irq_wake                                               */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_disable_irq_wake(void)
{
	unsigned long flags;
	
	spin_lock_irqsave(&shgrip_spinlock, flags);
	
	if (shgrip_irq_wake == SHGRIP_WAKE_ENABLED) {
		disable_irq_wake(gpio_to_irq(SHGRIP_GPIO_GRIP_INT));
		shgrip_irq_wake = SHGRIP_WAKE_DISABLED;
	}
	
	spin_unlock_irqrestore(&shgrip_spinlock, flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_qos_start                                                          */
/* ------------------------------------------------------------------------- */
void shgrip_qos_start(void)
{
	pm_qos_update_request(&shgrip_qos_cpu_dma_latency, SHGRIP_PM_QOS_LATENCY_VALUE);
}

/* ------------------------------------------------------------------------- */
/* shgrip_qos_end                                                            */
/* ------------------------------------------------------------------------- */
void shgrip_qos_end(void)
{
	pm_qos_update_request(&shgrip_qos_cpu_dma_latency, PM_QOS_DEFAULT_VALUE);
}

/* ------------------------------------------------------------------------- */
/* shgrip_suspend                                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_suspend(struct spi_device *spi, pm_message_t mesg)
{
	mutex_lock(&shgrip_mutex_lock);
	
	SHGRIP_DBG("start\n");
	
	shgrip_suspend_state = true;
	if (shgrip_status == SHGRIP_STATE_SENSOR_ON) {
		shgrip_sys_disable_irq();
		shgrip_sys_enable_irq_wake();
	}
	
	mutex_unlock(&shgrip_mutex_lock);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_resume                                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_resume(struct spi_device *spi)
{
	mutex_lock(&shgrip_mutex_lock);
	
	SHGRIP_DBG("start\n");
	
	shgrip_suspend_state = false;
	if (shgrip_status == SHGRIP_STATE_SENSOR_ON) {
		shgrip_sys_enable_irq();
		shgrip_sys_disable_irq_wake();
	}
	
	mutex_unlock(&shgrip_mutex_lock);

	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_clk_config                                                     */
/* ------------------------------------------------------------------------- */
static void shgrip_spi_clk_config(int clk_on)
{
#if 0
	int ret = 0;
	
	if (clk_on) {
		ret = gpio_tlmm_config(GPIO_CFG(SHGRIP_SPI_CLK, 1,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			SHGRIP_ERR("GPIO(SPI_CLK) set config NG clk_on:%d\n", clk_on);
		}
	} else {
		ret = gpio_tlmm_config(GPIO_CFG(SHGRIP_SPI_CLK, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (ret) {
			SHGRIP_ERR("GPIO(SPI_CLK) set config NG clk_on:%d\n", clk_on);
		}
		ret = gpio_direction_output(SHGRIP_SPI_CLK, 1);
		if (ret) {
			SHGRIP_ERR("GPIO(SPI_CLK) set direction NG\n");
		}
	}
#endif
	return;
}

/* ========================================================================= */
/* SHGRIP Serial Interface Function                                          */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_read_block                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_read_block(unsigned short addr, unsigned char* rbuf, int rlen)
{
	struct spi_message  msg;
	struct spi_transfer xfer[2];
	int i;
	int ret = 0;
	unsigned char tx_buf[2];
	unsigned char rx_buf[2];
	unsigned char readdata = 0;
	const unsigned char *data;
	
	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}
	
	if ((rlen <= 0) || (rlen >= SPI_BUFFER)) {
		SHGRIP_ERR("rlen fraud. rlen=%d\n", rlen);
		return -EINVAL;
	}
	
	memset(xfer, 0, sizeof(xfer));
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));
	
	tx_buf[0] = (addr >> 8) & 0xFF;
	tx_buf[1] = addr & 0xFF;
	
	xfer[0].tx_buf           = tx_buf;
	xfer[0].rx_buf           = rx_buf;
	xfer[0].len              = 2;
	xfer[0].bits_per_word    = SPI_BIT_WORD;
	xfer[0].speed_hz         = SPI_CLK_SPEED;
	
	shgrip_qos_start();
	shgrip_spi_clk_config(1);
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);

	udelay(SHGRIP_SPI_WAKEUP_WAIT_US);
	
	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	
	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		SHGRIP_ERR("spi_sync err ret=%d \n", ret);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		shgrip_qos_end();
		return ret;
	}
	
	if ((rx_buf[0] != SHGRIP_CMD_APP_ACK) || (rx_buf[1] != SHGRIP_CMD_APP_ACK)) {
		SHGRIP_ERR("Address send Err, MSB:%02x LSB:%02x\n", rx_buf[0], rx_buf[1]);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		shgrip_qos_end();
		return -EIO;
	}
	
	udelay(SHGRIP_SPI_READ_DELAY_US);
	
	xfer[0].tx_buf = &rlen;
	xfer[0].rx_buf = &readdata;
	xfer[0].len    = 1;
	xfer[0].bits_per_word    = SPI_BIT_WORD;
	xfer[0].speed_hz         = SPI_CLK_SPEED;
	xfer[0].deassert_wait    = 60;
	
	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	
	xfer[1].tx_buf = NULL;
	xfer[1].rx_buf = rbuf;
	xfer[1].len    = rlen;
	xfer[1].bits_per_word    = SPI_BIT_WORD;
	xfer[1].speed_hz         = SPI_CLK_SPEED;
	xfer[1].deassert_wait    = 60;
	
	spi_message_add_tail(&xfer[1], &msg);
	
	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		SHGRIP_ERR("spi_sync err ret=%d \n", ret);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		shgrip_qos_end();
		return ret;
	}
	
	if (readdata != SHGRIP_CMD_APP_ACK) {
		SHGRIP_ERR("Read Size send Err, Ack:%02x\n", readdata);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		shgrip_qos_end();
		return -EIO;
	}
	
	udelay(5);
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	shgrip_spi_clk_config(0);
	shgrip_qos_end();
	
	data = rbuf;
	
	SHGRIP_DBG("readaddr:0x%04x\n", addr);
	for(i = 0; i < rlen; i++){
		SHGRIP_DBG("readdata[%d]:0x%02x\n", i, *data);
		data++;
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_write_block                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_write_block(unsigned short addr, unsigned char *wbuf, int wlen)
{
	struct spi_message  msg;
	struct spi_transfer xfer[2];
	int i;
	int ret = 0;
	unsigned char tx_buf[2];
	unsigned char rx_buf[2];
	unsigned char readdata = 0;
	const unsigned char *data;
	unsigned char senddata[SPI_BUFFER];
	unsigned char recvdata[SPI_BUFFER];
	unsigned long programming_delay = 0;
	
	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}
	
	if ((wlen <= 0) || (wlen >= SPI_BUFFER)) {
		SHGRIP_ERR("wlen fraud. wlen=%d\n", wlen);
		return -EINVAL;
	}
	
	memset(xfer, 0, sizeof(xfer));
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));
	memset(senddata, 0, sizeof(senddata));
	memset(recvdata, 0, sizeof(recvdata));
	
	tx_buf[0] = ((addr | SHGRIP_CMD_WRITE_ADDR) >> 8) & 0xFF;
	tx_buf[1] = addr & 0xFF;
	
	xfer[0].tx_buf           = tx_buf;
	xfer[0].rx_buf           = rx_buf;
	xfer[0].len              = 2;
	xfer[0].bits_per_word    = SPI_BIT_WORD;
	xfer[0].speed_hz         = SPI_CLK_SPEED;
	
	shgrip_qos_start();
	shgrip_spi_clk_config(1);
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	udelay(SHGRIP_SPI_WAKEUP_WAIT_US);
	
	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	
	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		shgrip_qos_end();
		return ret;
	}
	
	if ((rx_buf[0] != SHGRIP_CMD_APP_ACK) || (rx_buf[1] != SHGRIP_CMD_APP_ACK)) {
		SHGRIP_ERR("Address send Err, MSB:%02x LSB:%02x\n", rx_buf[0], rx_buf[1]);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		shgrip_qos_end();
		return -EIO;
	}
	
	udelay(SHGRIP_SPI_READ_DELAY_US);
	
	data = wbuf;
	
	SHGRIP_DBG("writeaddr:0x%04x\n", ((tx_buf[0] << 8) | tx_buf[1]));
	for (i = 0; i < wlen; i++) {
		SHGRIP_DBG("senddata[%d]:0x%02x\n", i, *data);
		senddata[i] = *data;
		data++;
	}
	
	xfer[0].tx_buf = &wlen;
	xfer[0].rx_buf = &readdata;
	xfer[0].len    = 1;
	xfer[0].bits_per_word    = SPI_BIT_WORD;
	xfer[0].speed_hz         = SPI_CLK_SPEED;
	xfer[0].deassert_wait    = 60;
	
	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	
	xfer[1].tx_buf = senddata;
	xfer[1].rx_buf = &recvdata;
	xfer[1].len    = wlen;
	xfer[1].bits_per_word    = SPI_BIT_WORD;
	xfer[1].speed_hz         = SPI_CLK_SPEED;
	xfer[1].deassert_wait    = 60;

	spi_message_add_tail(&xfer[1], &msg);
	
	ret = spi_sync(spi_dev, &msg);
	if (ret) {
		SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		shgrip_qos_end();
		return ret;
	}
	
	if (readdata != SHGRIP_CMD_APP_NACK) {
		SHGRIP_ERR("Write Size send Err, ACK:%02x\n", readdata);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		shgrip_qos_end();
		return -EIO;
	}
	
	for (i = 0; i < wlen; i++) {
		if(recvdata[i] != SHGRIP_CMD_APP_ACK) {
			SHGRIP_ERR("Data send Err, ACK:%02x\n", recvdata[i]);
			gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
			shgrip_spi_clk_config(0);
			shgrip_qos_end();
			return -EIO;
		}
	}
	
	udelay(5);
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	shgrip_spi_clk_config(0);
	shgrip_qos_end();
	
	if (addr & 0x4000) {
		programming_delay = SHGRIP_SPI_PROGRAMMING_DELAY_US_RAM * wlen;
		
		if (programming_delay >= 1000) {
			shgrip_sys_delay_us(programming_delay);
		} else {
			udelay(programming_delay);
		}
	} else {
		shgrip_sys_delay_us(SHGRIP_SPI_PROGRAMMING_DELAY_US);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_ldr_read_block                                        */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_ldr_read_block(unsigned char *rbuf, int rlen)
{
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	int i;
	int ret = 0;
	unsigned char readdata=0;
	
	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}

	if ((rlen > 0) && (rlen <= SPI_BUFFER)) {
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		
		x->bits_per_word    = SPI_BIT_WORD;
		x->len              = 1;
		x->speed_hz         = SPI_CLK_SPEED;
		
		shgrip_qos_start();
		
		for (i = 0; i < rlen; i++) {
			x->rx_buf       = &readdata;
			
			spi_message_init(&msg);
			spi_message_add_tail(x, &msg);
			
			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				SHGRIP_ERR("spi_sync err ret=%d \n", ret);
			}					
			
			*rbuf = readdata;
			SHGRIP_DBG("readdata[%d]:0x%02x\n", i, *rbuf);
			rbuf++;
			
			udelay(SHGRIP_WAIT_BYTE_US);
		}
		
		shgrip_qos_end();
		
	} else {
		SHGRIP_ERR("rlen fraud. rlen=%d\n", rlen);
		ret = -EINVAL;
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_ldr_write_block                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_ldr_write_block(unsigned char *wbuf, int wlen)
{
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	const unsigned char *data;
	int i;
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}
	
	if ((wlen > 0) && (wlen <= SPI_BUFFER)) {
		data = wbuf;
		
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SPI_BIT_WORD;
		x->len              = 1;
		x->speed_hz         = SPI_CLK_SPEED;
		
		shgrip_qos_start();
		
		for (i = 0; i < wlen; i++) {
			x->tx_buf       = data;
			SHGRIP_DBG("senddata[%d]:0x%02x\n", i, *data);
			data++;
			
			spi_message_init(&msg);
			spi_message_add_tail(x, &msg);
			
			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
			}
			
			udelay(SHGRIP_WAIT_BYTE_US);
		}
		
		shgrip_qos_end();
		
	} else {
		SHGRIP_ERR("wlen fraud. wlen=%d\n", wlen);
		ret = -EINVAL;
	}
	
	return ret;
}


/* ========================================================================= */
/* SHGRIP Device Control Function                                            */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_cmd_flash_write                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_flash_write(unsigned char *databuf, int datalen)
{
	unsigned char buf[SPI_BUFFER];
	int ret = 1;
	
	if ((datalen > 0) && (datalen <= SHGRIP_FW_PACKET_SIZE)) {
		buf[0] = SHGRIP_CMD_FW_DL;
		memcpy(&buf[1], databuf, datalen);
		
		ret = shgrip_spi_transfer_ldr_write_block(buf, datalen+1);
		if (ret) {
			SHGRIP_ERR("spi send err. ret=%d\n", ret);
		}
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_check_sum                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_check_sum(void)
{
	int ret = 1;
	unsigned char wbuf[2];
	unsigned char rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_CHECK_SUM;
	wbuf[1] = shgrip_dbg_checksum;
	
	shgrip_spi_clk_config(1);
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	ret = shgrip_spi_transfer_ldr_write_block(wbuf, sizeof(wbuf));
	if (ret) {
		SHGRIP_ERR("spi send err. ret=%d\n", ret);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		return ret;
	}
	
	ret = shgrip_spi_transfer_ldr_read_block(rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("Ack Err ret:%d, ACK:0x%02X\n", ret, rbuf[0]);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		return ret;
	}
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	shgrip_spi_clk_config(0);
	
	if (rbuf[0] == SHGRIP_CMD_ACK) {
		return 0;
	} else if (rbuf[0] == SHGRIP_CMD_NACK) {
		return SHGRIP_CMD_NACK;
	} else {
		SHGRIP_ERR("Not Connect, ACK:%02x\n", rbuf[0]);
		return -1;
	}
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_devfwrevr                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_devfwrevr(struct shgrip_fw_version *ver_info)
{
	int ret = 1;
	unsigned char buf[3];
	
	SHGRIP_DBG("start\n");
	
	memset(buf, 0, sizeof(buf));
	
	ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_DEVFWREVR1, buf, sizeof(buf));
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_DEVFWREVR1 failed. ret:%d\n", ret);
		return ret;
	}
	
	ver_info->pver = ((buf[1] << 8) | buf[2]);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_bootfwrevr                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_bootfwrevr(struct shgrip_fw_version *ver_info)
{
	int ret = 1;
	unsigned char buf[3];
	
	SHGRIP_DBG("start\n");
	
	memset(buf, 0, sizeof(buf));
	
	ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_BOOTVER, buf, sizeof(buf));
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_BOOTVER failed. ret:%d\n", ret);
		return ret;
	}
	
	ver_info->lver0 = ((buf[0] << 8) | buf[2]);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_halten                                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_halten(void)
{
	int ret = 0;
	unsigned char data = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_LPCR, &data, 1);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_LPCR Read failed. ret:%d\n", ret);
		return ret;
	}
	
	data |= BIT3;
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_LPCR, &data, 1);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_LPCR Write failed. ret:%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_chenr                                                          */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_chenr(int on)
{
	int ret = 0;
	unsigned char data = 0;
	
	SHGRIP_DBG("start\n");
	
	if (on) {
		data = SHGRIP_ENABLE_CHANNEL;
	} else {
		data = 0;
	}
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_CHENR, &data, 1);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_CHENR failed. ret:%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_gripscr                                                        */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_gripscr(struct shgrip_sensor_state *ch_state)
{
	int ret = 0;
	unsigned char buf[3];
	
	SHGRIP_DBG("start\n");
	
	memset(buf, 0, sizeof(buf));
	
	ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_GRIPSCR, buf, sizeof(buf));
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_GRIPSCR failed. ret:%d\n", ret);
		return ret;
	}
	
	ch_state->state_grip = (buf[0] & BIT0) ? 1 : 0;
	ch_state->ch0 = (buf[0] & BIT1) ? 1 : 0;
	ch_state->ch2 = (buf[0] & BIT2) ? 1 : 0;
	ch_state->aet_error = (buf[0] & BIT6) ? 1 : 0;
	
	SHGRIP_INFO("GRIPSCR:%02x, %02x, %02x\n", buf[0], buf[1], buf[2] );
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_gripsr                                                        */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_gripsr(struct shgrip_sensor_state *ch_state)
{
	int ret = 0;
	unsigned char buf[3];
	
	SHGRIP_DBG("start\n");
	
	memset(buf, 0, sizeof(buf));
	
	ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_GRIPSR, buf, sizeof(buf));
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_GRIPSR failed. ret:%d\n", ret);
		return ret;
	}
	
	ch_state->state_grip = (buf[0] & BIT0) ? 1 : 0;
	ch_state->ch0 = (buf[0] & BIT1) ? 1 : 0;
	ch_state->ch2 = (buf[0] & BIT2) ? 1 : 0;
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_lpcr                                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_lpcr(unsigned char *data, int size)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_LPCR, data, size);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_LPCR failed. ret:%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_acqrater                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_acqrater(unsigned char *data, int size)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_ACQRATER, data, size);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_ACQRATER failed. ret:%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_ch0cr                                                          */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_ch0cr(unsigned char *data, int size)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_CH0CR, data, size);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_CH0CR failed. ret:%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_ch4cr                                                          */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_ch4cr(unsigned char *data, int size)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_CH4CR, data, size);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_CH4CR failed. ret:%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_trigccr                                                        */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_trigccr(unsigned char *data, int size)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_TRIGCCR, data, size);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_TRIGCCR failed. ret:%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_delaygriprh                                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_delaygriprh(unsigned char *data, int size)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_DELAYGRIPRH, data, size);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_DELAYGRIPRH failed. ret:%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_ch0pthrh                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_ch0pthrh(unsigned char* data, int size)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_CH0PTHRH, data, size);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_CH0PTHRH failed. ret:%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_ch4pthrh                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_ch4pthrh(unsigned char* data, int size)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_CH4PTHRH, data, size);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_CH4PTHRH failed. ret:%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_ch0sig                                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_ch0sig(unsigned char *rbuf)
{
	int ret = 0;
	unsigned char buf[4];
	
	SHGRIP_DBG("start\n");
	
	memset(buf, 0, sizeof(buf));
	
	ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_CH0SIGRH, buf, sizeof(buf));
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_CH0SIGRH failed. ret:%d\n", ret);
		return ret;
	}
	
	rbuf[0] = buf[0];
	rbuf[1] = buf[1];
	rbuf[2] = buf[2];
	rbuf[3] = buf[3];
	
	SHGRIP_DBG("CH0 Meas:0x%04X, Ref:0x%04X\n",
		((rbuf[0] << 8) | rbuf[1]),
		((rbuf[2] << 8) | rbuf[3]));
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_ch4sig                                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_ch4sig(unsigned char *rbuf)
{
	int ret = 0;
	unsigned char buf[4];
	
	SHGRIP_DBG("start\n");
	
	memset(buf, 0, sizeof(buf));
	
	ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_CH4SIGRH, buf, sizeof(buf));
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_CH4SIGRH failed. ret:%d\n", ret);
		return ret;
	}
	
	rbuf[0] = buf[0];
	rbuf[1] = buf[1];
	rbuf[2] = buf[2];
	rbuf[3] = buf[3];
	
	SHGRIP_DBG("CH4 Meas:0x%04X, Ref:0x%04X\n",
		((rbuf[0] << 8) | rbuf[1]),
		((rbuf[2] << 8) | rbuf[3]));
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_ch0pth                                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_ch0pth(unsigned char *rbuf)
{
	int ret = 0;
	unsigned char buf[5];
	
	SHGRIP_DBG("start\n");
	
	memset(buf, 0, sizeof(buf));
	
	ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_CH0PTHRH, buf, sizeof(buf));
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_CH0PTHRH failed. ret:%d\n", ret);
		return ret;
	}
	
	rbuf[0] = buf[0];
	rbuf[1] = buf[1];
	rbuf[2] = buf[2];
	rbuf[3] = buf[3];
	rbuf[4] = buf[4];
	
	SHGRIP_DBG("CH0 PROX TH:0x%04X, Touch TH:0x%04X, Release TH:0x%02X\n",
		((rbuf[0] << 8) | rbuf[1]),
		((rbuf[2] << 8) | rbuf[3]),
		  rbuf[4]);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_ch4pth                                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_ch4pth(unsigned char *rbuf)
{
	int ret = 0;
	unsigned char buf[5];
	
	SHGRIP_DBG("start\n");
	
	memset(buf, 0, sizeof(buf));
	
	ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_CH4PTHRH, buf, sizeof(buf));
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_CH4PTHRH failed. ret:%d\n", ret);
		return ret;
	}
	
	rbuf[0] = buf[0];
	rbuf[1] = buf[1];
	rbuf[2] = buf[2];
	rbuf[3] = buf[3];
	rbuf[4] = buf[4];
	
	SHGRIP_DBG("CH4 PROX TH:0x%04X, Touch TH:0x%04X, Release TH:0x%02X\n",
		((rbuf[0] << 8) | rbuf[1]),
		((rbuf[2] << 8) | rbuf[3]),
		  rbuf[4]);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_debug_rw_command                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_debug_rw_command(struct shgrip_dbg_command *cmd)
{
	int ret;
	
	SHGRIP_DBG("start\n");
	
	shgrip_spi_clk_config(1);
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	if (cmd->w_size) {
		ret = shgrip_spi_transfer_ldr_write_block(cmd->w_buf, cmd->w_size);
		if (ret) {
			SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
			gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
			shgrip_spi_clk_config(0);
			return ret;
		}
	}
	
	if (cmd->r_size) {
		ret = shgrip_spi_transfer_ldr_read_block(cmd->r_buf, cmd->r_size);
		if (ret) {
			SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
			gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
			shgrip_spi_clk_config(0);
			return ret;
		}
	}
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	shgrip_spi_clk_config(0);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_debug_rw_command2                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_debug_rw_command2(struct shgrip_dbg_command2 *cmd)
{
	int ret;
	
	SHGRIP_DBG("start\n");
	
	if (cmd->w_size) {
		ret = shgrip_spi_transfer_write_block(cmd->addr, cmd->w_buf, cmd->w_size);
		if (ret) {
			SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
			return ret;
		}
	}
	
	if (cmd->r_size) {
		ret = shgrip_spi_transfer_read_block(cmd->addr, cmd->r_buf, cmd->r_size);
		if (ret) {
			SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
			return ret;
		}
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_rstsrcr                                                        */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_rstsrcr(void)
{
	int ret = 0;
	unsigned char buf;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_RSTSRC, &buf, 1);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_RSTSRC failed. ret:%d\n", ret);
		return ret;
	}
	
	SHGRIP_ERR("RSTSRC:%02x\n", buf);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_cmd_aet_repeatr                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_cmd_aet_repeatr(unsigned char* data, int size)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_spi_transfer_write_block(SHGRIP_CMD_AET_REPEATR, data, size);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_AET_REPEATR failed. ret:%d\n", ret);
		return ret;
	}
	
	return 0;
}

/* ========================================================================= */
/* SHGRIP Sequence Function                                                  */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_seq_grip_power_off                                                 */
/* ------------------------------------------------------------------------- */
static void shgrip_seq_grip_power_off(void)
{
	int ret = 0;
	int i = 0;
    struct qpnp_pin_cfg param;	
	
	for (i = 0; i < 3; i++) {
		ret = shgrip_cmd_halten();
		if (ret) {
			SHGRIP_ERR("shgrip_cmd_halten err. ret=%d\n", ret);
		} else {
			break;
		}
	}
	
	if (ret) {
		param.mode = QPNP_PIN_MODE_DIG_OUT;
		param.output_type = QPNP_PIN_OUT_BUF_CMOS;
		param.invert = QPNP_PIN_INVERT_DISABLE;
		param.pull = QPNP_PIN_GPIO_PULL_NO;
		param.vin_sel = QPNP_PIN_VIN0;
		param.out_strength = QPNP_PIN_OUT_STRENGTH_HIGH;
		param.src_sel = QPNP_PIN_SEL_FUNC_CONSTANT;
		param.master_en = QPNP_PIN_MASTER_ENABLE;
		param.aout_ref = 0;
		param.ain_route = 0;
		param.cs_out = 0;
		ret = qpnp_pin_config(SHGRIP_GPIO_GRIP_RESET, &param);
		if (ret) {
			SHGRIP_ERR("GPIO(RESET) qpnp_pin config output NG\n");
		}
		
		gpio_set_value(SHGRIP_GPIO_GRIP_RESET, SHGRIP_GPIO_VAL_LO);
		shgrip_sys_delay_us(SHGRIP_RESET_WAIT_US);
		
		shgrip_set_sensor_adjust_flg = false;
		
		SHGRIP_DBG("ctrl->state: SHGRIP_STATE_POWER_OFF\n");
		shgrip_status = SHGRIP_STATE_POWER_OFF;
	} else {
		shgrip_set_sensor_adjust_flg = false;

		SHGRIP_DBG("ctrl->state: SHGRIP_STATE_HALT_MODE\n");
		shgrip_status = SHGRIP_STATE_HALT_MODE;
	}
	
	shgrip_spi_clk_config(0);
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_reset                                                          */
/* ------------------------------------------------------------------------- */
static void shgrip_seq_reset(void)
{
	int ret;
    struct qpnp_pin_cfg param;	

	param.mode = QPNP_PIN_MODE_DIG_OUT;
	param.output_type = QPNP_PIN_OUT_BUF_CMOS;
	param.invert = QPNP_PIN_INVERT_DISABLE;
	param.pull = QPNP_PIN_GPIO_PULL_NO;
	param.vin_sel = QPNP_PIN_VIN0;
	param.out_strength = QPNP_PIN_OUT_STRENGTH_HIGH;
	param.src_sel = QPNP_PIN_SEL_FUNC_CONSTANT;
	param.master_en = QPNP_PIN_MASTER_ENABLE;
	param.aout_ref = 0;
	param.ain_route = 0;
	param.cs_out = 0;
	ret = qpnp_pin_config(SHGRIP_GPIO_GRIP_RESET, &param);
	if (ret) {
		SHGRIP_ERR("GPIO(RESET) qpnp_pin config output NG\n");
	}
	
	gpio_set_value(SHGRIP_GPIO_GRIP_RESET, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_RESET_WAIT_US);
	
	gpio_set_value(SHGRIP_GPIO_GRIP_RESET, SHGRIP_GPIO_VAL_HI);
	
	param.pull = QPNP_PIN_GPIO_PULL_UP_31P5;
	ret = qpnp_pin_config(SHGRIP_GPIO_GRIP_RESET, &param);
	if (ret) {
		SHGRIP_ERR("GPIO(RESET) qpnp_pin config output PU NG\n");
	}

	param.pull = QPNP_PIN_GPIO_PULL_UP_31P5;
	param.mode = QPNP_PIN_MODE_DIG_IN;
	ret = qpnp_pin_config(SHGRIP_GPIO_GRIP_RESET, &param);
	if (ret) {
		SHGRIP_ERR("GPIO(RESET) qpnp_pin config input NG\n");
	}
	
	shgrip_sys_delay_us(SHGRIP_RESET_START_WAIT_US);
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_reset_recovery                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_reset_recovery(void)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_cmd_lpcr(bk_adj.shgrip_lpcr, sizeof(bk_adj.shgrip_lpcr));
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_lpcr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_acqrater(&bk_adj.shgrip_acqrater, 1);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_acqrater err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_ch0cr(&bk_adj.shgrip_ch0cr, 1);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch0cr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_ch4cr(&bk_adj.shgrip_ch4cr, 1);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch4cr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_trigccr(bk_adj.shgrip_trigccr, sizeof(bk_adj.shgrip_trigccr));
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_trigccr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_ch0pthrh(bk_adj.sens_ch_a, sizeof(bk_adj.sens_ch_a));
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch0pthrh err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_ch4pthrh(bk_adj.sens_ch_b, sizeof(bk_adj.sens_ch_b));
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch4pthrh err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_delaygriprh(bk_adj.shgrip_delaygriprh, sizeof(bk_adj.shgrip_delaygriprh));
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_delaygriprh err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_aet_repeatr(&bk_adj.shgrip_aetrepeatr, 1);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_aet_repeatr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_start_app                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_start_app(void)
{
	int ret = 0;
	unsigned char wbuf = 0;
	
	shgrip_seq_reset();
	
	shgrip_spi_clk_config(1);
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	wbuf = SHGRIP_CMD_CHG_APP_MODE;	
	ret = shgrip_spi_transfer_ldr_write_block(&wbuf, 1);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		shgrip_seq_grip_power_off();
		return ret; 
	
	}
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	shgrip_spi_clk_config(0);
	
	shgrip_sys_delay_us(SHGRIP_RESET_START_APP_WAIT_US);
	
	SHGRIP_DBG("ctrl->state: SHGRIP_STATE_SENSOR_OFF\n");
	shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	
	if (shgrip_set_sensor_adjust_flg == true){
		ret = shgrip_seq_reset_recovery();
		if (ret) {
			SHGRIP_ERR("shgrip_seq_reset_recovery failed\n");
			return ret;
		}
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_start_loader                                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_start_loader(void)
{
	int ret = 0;
	unsigned char wbuf = 0;
	
	shgrip_spi_clk_config(1);
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	wbuf = SHGRIP_CMD_CHG_LDR_MODE;
	ret = shgrip_spi_transfer_ldr_write_block(&wbuf, 1);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
		shgrip_spi_clk_config(0);
		return ret;
	}
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	shgrip_spi_clk_config(0);
	
	SHGRIP_DBG("ctrl->state: STATE_FW_DL\n");
	shgrip_status = SHGRIP_STATE_FW_DL;
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_start_checksum                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_start_checksum(unsigned char *sum_val)
{
	int ret = 0;
	unsigned char wbuf = 0;
	
	shgrip_seq_reset();
	
	shgrip_spi_clk_config(1);
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	wbuf = SHGRIP_CMD_LDR_CHECK_SUM;
	ret = shgrip_spi_transfer_ldr_write_block(&wbuf, 1);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		goto done;
	}
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	
	shgrip_sys_delay_us(SHGRIP_CHECK_SUM_WAIT_US);
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	ret = shgrip_spi_transfer_ldr_read_block(sum_val, 1);
	if (ret) {
		SHGRIP_ERR("spi transfer read err. ret=%d\n", ret);
		goto done;
	}
	
	SHGRIP_INFO("SUM Value:0x%02X\n", *sum_val);
	
done:
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	shgrip_spi_clk_config(0);
	
	ret = shgrip_seq_start_app();
	if (ret) {
		SHGRIP_ERR("shgrip_seq_start_app err. ret=%d\n", ret);
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_grip_sensor_on                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_grip_sensor_on(void)
{
	int ret = GRIP_RESULT_SUCCESS;
#ifdef CONFIG_ANDROID_ENGINEERING
	unsigned long sec,nsec;
#endif /* CONFIG_ANDROID_ENGINEERING */
	
	SHGRIP_DBG("start\n");
	
	if ((shgrip_status == SHGRIP_STATE_POWER_OFF) 
	 || (shgrip_status == SHGRIP_STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (shgrip_status == SHGRIP_STATE_SENSOR_ON) {
		SHGRIP_WARN("state is STATE_SENSOR_ON\n");
		return GRIP_RESULT_SUCCESS;
	}
	
	if (shgrip_status == SHGRIP_STATE_HALT_MODE) {
		shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	}
	
	ret = shgrip_cmd_chenr(SHGRIP_OFF);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_chenr off err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_chenr(SHGRIP_ON);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_chenr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	SHGRIP_DBG("state: STATE_SENSOR_ON \n");
	shgrip_status = SHGRIP_STATE_SENSOR_ON;
	
	shgrip_sys_enable_irq();
	
#ifdef CONFIG_ANDROID_ENGINEERING
	if (shgrip_th_interval_ms) {
		th_msec = shgrip_th_interval_ms;
		
		if (th_msec >= 1000) {
			sec  = th_msec / 1000;
			nsec = (th_msec % 1000) * 1000 * 1000;
		} else {
			sec  = 0;
			nsec = th_msec * 1000 * 1000;
		}
		
		if (hrtimer_flg == 0) {
			wake_lock(&shgrip_dump_wake_lock);
			hrtimer_start(&shgrip_threshold_timer, ktime_set(sec, nsec), HRTIMER_MODE_REL);
			hrtimer_flg = 1;
		}
	}
#endif /* CONFIG_ANDROID_ENGINEERING */
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_grip_sensor_off                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_grip_sensor_off(void)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((shgrip_status == SHGRIP_STATE_POWER_OFF)
	 || (shgrip_status == SHGRIP_STATE_HALT_MODE)
	 || (shgrip_status == SHGRIP_STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (shgrip_status == SHGRIP_STATE_SENSOR_OFF) {
		SHGRIP_WARN("state is STATE_SENSOR_OFF\n");
		return GRIP_RESULT_SUCCESS;
	}
	
	ret = shgrip_cmd_chenr(SHGRIP_OFF);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_chenr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	SHGRIP_DBG("state: STATE_SENSOR_OFF \n");
	shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	
	shgrip_sys_disable_irq();
	shgrip_sys_disable_irq_wake();
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	SHGRIP_DBG("msm_tps_set_grip_state called grip:0\n");
	msm_tps_set_grip_state(SHGRIP_OFF);
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_set_sensor_adjust                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_set_sensor_adjust(struct shgrip_sens_setting_params *data)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((shgrip_status == SHGRIP_STATE_POWER_OFF) 
	 || (shgrip_status == SHGRIP_STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (shgrip_status == SHGRIP_STATE_HALT_MODE) {
		shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	}
	
	ret = shgrip_cmd_lpcr(data->shgrip_lpcr, sizeof(data->shgrip_lpcr));
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_lpcr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_acqrater(&data->shgrip_acqrater, 1);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_acqrater err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_ch0cr(&data->shgrip_ch0cr, 1);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch0cr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_ch4cr(&data->shgrip_ch4cr, 1);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch4cr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_trigccr(data->shgrip_trigccr, sizeof(data->shgrip_trigccr));
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch0pthrh err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_ch0pthrh(data->sens_ch_a, sizeof(data->sens_ch_a));
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch0pthrh err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_ch4pthrh(data->sens_ch_b, sizeof(data->sens_ch_b));
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch4pthrh err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_delaygriprh(data->shgrip_delaygriprh, sizeof(data->shgrip_delaygriprh));
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_delaygriprh err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_aet_repeatr(&data->shgrip_aetrepeatr, 1);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_aet_repeatr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	memcpy(&bk_adj, data, sizeof(struct shgrip_sens_setting_params));
	
	shgrip_set_sensor_adjust_flg = true;
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_state                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_state(struct shgrip_sensor_state *state)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((shgrip_status == SHGRIP_STATE_POWER_OFF) 
	 || (shgrip_status == SHGRIP_STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (shgrip_status == SHGRIP_STATE_HALT_MODE) {
		shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	}
	
	ret = shgrip_cmd_gripsr(state);
	if (ret) {
		SHGRIP_ERR("SHGRIP_CMD_GRIPSR failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	SHGRIP_INFO("state_grip:%d, ch0:%d, ch4:%d\n", 
									state->state_grip,
									state->ch0,
									state->ch2);
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_fw_version                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_fw_version(struct shgrip_fw_version *ver_info)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((shgrip_status == SHGRIP_STATE_POWER_OFF) 
	 || (shgrip_status == SHGRIP_STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (shgrip_status == SHGRIP_STATE_SENSOR_ON) {
		SHGRIP_ERR("state is STATE_SENSOR_ON\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	if (shgrip_status == SHGRIP_STATE_HALT_MODE) {
		shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	}
	
	ver_info->lver1 = 0x0000;
	
	ret = shgrip_cmd_bootfwrevr(ver_info);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_bootfwrevr failed\n");
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_cmd_devfwrevr(ver_info);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_devfwrevr failed\n");
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_download_fw_probe                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_download_fw_probe(unsigned char *fw_buf, int fw_size)
{
	int ret = GRIP_RESULT_SUCCESS;
	int i = 0;
	int size = 0;
	int roop_count = 0;
	unsigned long dl_write_ptr = 0;
	
	SHGRIP_DBG("start\n");
	
	if (!fw_buf) {
		SHGRIP_ERR("fw_data is NULL\n");
		return GRIP_RESULT_FAILURE_USER;
	}
	
	size = (SHGRIP_FW_END_ADDR - SHGRIP_FW_START_ADDR) + 1;
	if (size != fw_size) {
		SHGRIP_ERR("Fw size Err, size:%d\n", fw_size);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (size % SHGRIP_FW_PACKET_SIZE) {
		roop_count = (size / SHGRIP_FW_PACKET_SIZE) + 1;
	} else {
		roop_count = size / SHGRIP_FW_PACKET_SIZE;
	}
	
	shgrip_seq_reset();
	
	ret = shgrip_seq_start_loader();
	if (ret) {
		SHGRIP_ERR("Loader Start Err:%d\n", ret);
		goto shgrip_fw_update_done;
	}
	
	shgrip_spi_clk_config(1);
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	for ( i = 1; i <= roop_count; i++ ) {
		if ( i == roop_count ){
			ret = shgrip_cmd_flash_write(&fw_buf[dl_write_ptr], size);
		} else {
			ret = shgrip_cmd_flash_write(&fw_buf[dl_write_ptr], SHGRIP_FW_PACKET_SIZE);
			dl_write_ptr += SHGRIP_FW_PACKET_SIZE;
			size -= SHGRIP_FW_PACKET_SIZE;
		}
		
		shgrip_sys_delay_us(SHGRIP_WAIT_FW_WRITE_US);
		
		if (ret) {
			SHGRIP_ERR("Write_block Err, roop:%d\n", i);
			gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI); 
			shgrip_spi_clk_config(0);
			goto shgrip_fw_update_done;
		}
	}
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	shgrip_spi_clk_config(0);

	ret = shgrip_cmd_check_sum();
	if (ret) {
		SHGRIP_ERR("Check Sum Err:%d\n", ret);
	}
	
shgrip_fw_update_done:
	
	shgrip_seq_start_app();
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_download_fw                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_download_fw(unsigned char *fw_buf, int fw_size)
{
	int ret = GRIP_RESULT_SUCCESS;
	int i = 0;
	int size = 0;
	int roop_count = 0;
	unsigned long dl_write_ptr = 0;
	
	SHGRIP_DBG("start\n");
	
	if ((shgrip_status == SHGRIP_STATE_POWER_OFF) 
	 || (shgrip_status == SHGRIP_STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (shgrip_status == SHGRIP_STATE_SENSOR_ON) {
		SHGRIP_ERR("state is STATE_SENSOR_ON\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	if (shgrip_status == SHGRIP_STATE_HALT_MODE) {
		shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	}
	
	if (!fw_buf) {
		SHGRIP_ERR("fw_data is NULL\n");
		return GRIP_RESULT_FAILURE_USER;
	}
	
	size = (SHGRIP_FW_END_ADDR - SHGRIP_FW_START_ADDR) + 1;
	if (size != fw_size) {
		SHGRIP_ERR("Fw size Err, size:%d\n", fw_size);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (size % SHGRIP_FW_PACKET_SIZE) {
		roop_count = (size / SHGRIP_FW_PACKET_SIZE) + 1;
	} else {
		roop_count = size / SHGRIP_FW_PACKET_SIZE;
	}
	
	shgrip_seq_reset();
	
	ret = shgrip_seq_start_loader();
	if (ret) {
		SHGRIP_ERR("Loader Start Err:%d\n", ret);
		goto shgrip_fw_update_done;
	}
	
	shgrip_spi_clk_config(1);
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	for ( i = 1; i <= roop_count; i++ ) {
		if ( i == roop_count ){
			ret = shgrip_cmd_flash_write(&fw_buf[dl_write_ptr], size);
		} else {
			ret = shgrip_cmd_flash_write(&fw_buf[dl_write_ptr], SHGRIP_FW_PACKET_SIZE);
			dl_write_ptr += SHGRIP_FW_PACKET_SIZE;
			size -= SHGRIP_FW_PACKET_SIZE;
		}
		
		shgrip_sys_delay_us(SHGRIP_WAIT_FW_WRITE_US);
		
		if (ret) {
			SHGRIP_ERR("Write_block Err, roop:%d\n", i);
			gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
			shgrip_spi_clk_config(0);
			goto shgrip_fw_update_done;
		}
	}
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	shgrip_spi_clk_config(0);
	
	/* check sum */
	ret = shgrip_cmd_check_sum();
	if (ret) {
		SHGRIP_ERR("Check Sum Err:%d\n", ret);
	}
	
shgrip_fw_update_done:
	shgrip_seq_start_app();
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_checksum		                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_checksum(unsigned char *sum_val)
{
	int ret = 0;
	
	if ((shgrip_status == SHGRIP_STATE_POWER_OFF) 
	 || (shgrip_status == SHGRIP_STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (shgrip_status == SHGRIP_STATE_SENSOR_ON) {
		SHGRIP_ERR("state is STATE_SENSOR_ON\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	if (shgrip_status == SHGRIP_STATE_HALT_MODE) {
		shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	}
	
	ret = shgrip_seq_start_checksum(sum_val);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_chenr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}

	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_debug_rw_command                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_debug_rw_command(struct shgrip_dbg_command *cmd)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((shgrip_status == SHGRIP_STATE_POWER_OFF) 
     || (shgrip_status == SHGRIP_STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (shgrip_status == SHGRIP_STATE_HALT_MODE) {
		shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	}
	
	ret = shgrip_cmd_debug_rw_command(cmd);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_debug_rw_command failed\n");
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_debug_rw_command                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_debug_rw_command2(struct shgrip_dbg_command2 *cmd)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((shgrip_status == SHGRIP_STATE_POWER_OFF) 
     || (shgrip_status == SHGRIP_STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (shgrip_status == SHGRIP_STATE_HALT_MODE) {
		shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	}
	
	ret = shgrip_cmd_debug_rw_command2(cmd);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_debug_rw_command2 failed\n");
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_start_get_chprm                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_start_get_chprm(struct shgrip_chprm *val)
{
	int ret;
	unsigned char buf[5];
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_cmd_ch0sig(buf);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch0sig err. ret:%d\n", ret);
		return ret;
	}
	val->meas_ch0.high_val		= buf[0];
	val->meas_ch0.low_val		= buf[1];
	val->ref_ch0.high_val		= buf[2];
	val->ref_ch0.low_val		= buf[3];
	
	ret = shgrip_cmd_ch4sig(buf);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch4sig err. ret:%d\n", ret);
		return ret;
	}
	val->meas_ch4.high_val		= buf[0];
	val->meas_ch4.low_val		= buf[1];
	val->ref_ch4.high_val		= buf[2];
	val->ref_ch4.low_val		= buf[3];
	
	ret = shgrip_cmd_ch0pth(buf);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch0pth err. ret:%d\n", ret);
		return ret;
	}
	val->proxth_ch0.high_val	= buf[0];
	val->proxth_ch0.low_val		= buf[1];
	val->touchth_ch0.high_val	= buf[2];
	val->touchth_ch0.low_val	= buf[3];
	val->releaseth_ch0			= buf[4];
	
	ret = shgrip_cmd_ch4pth(buf);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_ch4pth err. ret:%d\n", ret);
		return ret;
	}
	val->proxth_ch4.high_val	= buf[0];
	val->proxth_ch4.low_val		= buf[1];
	val->touchth_ch4.high_val	= buf[2];
	val->touchth_ch4.low_val	= buf[3];
	val->releaseth_ch4			= buf[4];
	
	/* Dlt T.B.D */
	val->delta_ch0.high_val		= 0x00;
	val->delta_ch0.low_val		= 0x00;
	val->delta_ch4.high_val		= 0x00;
	val->delta_ch4.low_val		= 0x00;
	
	SHGRIP_INFO("CH0 Meas:0x%04X, Ref:0x%04X, Dlt:0x%04X, PROX TH:0x%04X, Touch TH:0x%04X, Release TH:0x%02X\n",
		((val->meas_ch0.high_val  << 8)     | val->meas_ch0.low_val),
		((val->ref_ch0.high_val    << 8)    | val->ref_ch0.low_val),
		((val->delta_ch0.high_val  << 8)    | val->delta_ch0.low_val),
		((val->proxth_ch0.high_val    << 8) | val->proxth_ch0.low_val),
		((val->touchth_ch0.high_val  << 8)  | val->touchth_ch0.low_val),
		  val->releaseth_ch0);

	SHGRIP_INFO("CH4 Meas:0x%04X, Ref:0x%04X, Dlt:0x%04X, PROX TH:0x%04X, Touch TH:0x%04X, Release TH:0x%02X\n",
		((val->meas_ch4.high_val  << 8)     | val->meas_ch4.low_val),
		((val->ref_ch4.high_val    << 8)    | val->ref_ch4.low_val),
		((val->delta_ch4.high_val  << 8)    | val->delta_ch4.low_val),
		((val->proxth_ch4.high_val    << 8) | val->proxth_ch4.low_val),
		((val->touchth_ch4.high_val  << 8)  | val->touchth_ch4.low_val),
		  val->releaseth_ch0);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_chprm_value                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_chprm_value(struct shgrip_chprm *val)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((shgrip_status == SHGRIP_STATE_POWER_OFF) 
	 || (shgrip_status == SHGRIP_STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (shgrip_status == SHGRIP_STATE_HALT_MODE) {
		shgrip_status = SHGRIP_STATE_SENSOR_OFF;
	}
	
	ret = shgrip_seq_start_get_chprm(val);
	if (ret) {
		SHGRIP_DBG("shgrip_command_chprm err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}


/* ========================================================================= */
/* SHGRIP ioctl Function                                                     */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_grip_sensor_on                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_grip_sensor_on(void)
{
	SHGRIP_DBG("start\n");
	
	return shgrip_seq_grip_sensor_on();
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_grip_sensor_off                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_grip_sensor_off(void)
{
	SHGRIP_DBG("start\n");
	
	return shgrip_seq_grip_sensor_off();
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_set_sensor_adjust                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_set_sensor_adjust(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_sens_setting_params data;
	
	SHGRIP_DBG("start\n");
	
	ret = copy_from_user(&data, argp,
							sizeof(struct shgrip_sens_setting_params));
	
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_seq_set_sensor_adjust(&data);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_sensor_adjust                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_sensor_adjust(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	ret = copy_to_user(argp, &(bk_adj.setting_val), 
							sizeof(struct shgrip_user_setting));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_state                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_state(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_sensor_state state;
	
	SHGRIP_DBG("start\n");
	
	memset(&state, 0, sizeof(struct shgrip_sensor_state));
	
	ret = shgrip_seq_get_state(&state);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_state failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &state, 
							sizeof(struct shgrip_sensor_state));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_fw_version                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_fw_version(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_fw_version ver_info;
	
	SHGRIP_DBG("start\n");
	
	memset(&ver_info, 0, sizeof(struct shgrip_fw_version));
	
	ret = shgrip_seq_get_fw_version(&ver_info);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_fw_version failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &ver_info, 
							sizeof(struct shgrip_fw_version));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_download_fw                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_download_fw(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_fw_data fw_info;
	unsigned char *fw_data;
	
	ret = copy_from_user(&fw_info, argp, 
							sizeof(struct shgrip_fw_data));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed\n");
		return GRIP_RESULT_FAILURE;
	}
	
	if(fw_info.size < 0 || fw_info.size > SHGRIP_FW_SIZE){
		SHGRIP_ERR("fw_info.size err\n");
		return GRIP_RESULT_FAILURE;
	}
	
	fw_data = kmalloc(fw_info.size, GFP_KERNEL);
	if (!fw_data) {
		SHGRIP_ERR("kmalloc failed\n");
		return GRIP_RESULT_FAILURE;
	}
	
	if(!fw_info.data){
		SHGRIP_ERR("fw_info.data is NULL\n");
		kfree(fw_data);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = copy_from_user(fw_data, fw_info.data, fw_info.size);
	if (ret) {
		SHGRIP_ERR("copy_from_user fw.data failed\n");
		ret = GRIP_RESULT_FAILURE;
		goto dl_fw_func_done;
	}
	
	ret = shgrip_seq_download_fw(fw_data, fw_info.size);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_download_fw failed\n");
	}
	
dl_fw_func_done:
	kfree(fw_data);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_download_fw                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_download_fw(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	unsigned char *buf;
	int size;
	
	buf = (unsigned char *)shgrip_fw_image;
	size = sizeof(shgrip_fw_image);
	
	ret = shgrip_seq_download_fw(buf, size);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_download_fw failed\n");
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_check_sum                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_check_sum(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	unsigned char sum_val = 0;
	unsigned short send_data = 0;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_seq_checksum(&sum_val);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_start_checksum failed\n");
		return ret;
	}
	
	send_data = (unsigned short)sum_val;
	
	ret = copy_to_user(argp, &send_data, sizeof(unsigned short));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_drv_status                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_drv_status(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	unsigned char state;
	
	SHGRIP_DBG("start\n");
	
	state = shgrip_status;
	if (state == SHGRIP_STATE_HALT_MODE) {
		state = SHGRIP_STATE_POWER_OFF;
	}
	
	ret = copy_to_user(argp, &state, sizeof(unsigned char));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_debug_rw_command                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_debug_rw_command(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_dbg_command cmd;
	
	SHGRIP_DBG("start\n");
	
	memset(&cmd, 0, sizeof(struct shgrip_dbg_command));
	
	ret = copy_from_user(&cmd, argp, sizeof(struct shgrip_dbg_command));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_seq_debug_rw_command(&cmd);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_debug_rw_command failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &cmd, sizeof(struct shgrip_dbg_command));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_debug_rw_command2                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_debug_rw_command2(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_dbg_command2 cmd;
	
	SHGRIP_DBG("start\n");
	
	memset(&cmd, 0, sizeof(struct shgrip_dbg_command2));
	
	ret = copy_from_user(&cmd, argp, sizeof(struct shgrip_dbg_command2));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_seq_debug_rw_command2(&cmd);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_debug_rw_command2 failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &cmd, sizeof(struct shgrip_dbg_command2));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_set_bk_adjust                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_set_bk_adjust(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_chprm_value                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_chprm_value(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_chprm val;
	
	SHGRIP_DBG("start\n");
	
	memset(&val, 0, sizeof(struct shgrip_chprm));
	
	ret = shgrip_seq_get_chprm_value(&val);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_chprm_value failed\n");
		return ret;
	}
	ret = copy_to_user(argp, &val, sizeof(struct shgrip_chprm));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_set_reset                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_set_reset(void)
{
	SHGRIP_DBG("start\n");
	
	shgrip_seq_reset();
	
	return GRIP_RESULT_SUCCESS;
}

/* ========================================================================= */
/* SHGRIP Input Event Function                                               */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_input_event                                                        */
/* ------------------------------------------------------------------------- */
static int shgrip_input_event(void)
{
	int ret = 0;
	int code = 0;
	struct shgrip_sensor_state ch_state;
	int i;
	unsigned char wbuf = 0;
	
	memset(&ch_state, 0 , sizeof(struct shgrip_sensor_state));
	
	for (i = 0; i < 3; i++) {
		ret = shgrip_cmd_gripscr(&ch_state);
		if (ret) {
			SHGRIP_ERR("SHGRIP_CMD_GRIPSCR failed. ret:%d\n", ret);
			if(i == 2){
				SHGRIP_DBG("recovery seq 1 start\n");
				
				ch_state.state_grip = SHGRIP_OFF;
				
				shgrip_sys_delay_us(SHGRIP_RESET_INT_WAIT_US);
				
				shgrip_spi_clk_config(1);
				gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
				
				wbuf = SHGRIP_CMD_CHG_APP_MODE;	
				ret = shgrip_spi_transfer_ldr_write_block(&wbuf, 1);
				if (ret) {
					SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
					gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
					shgrip_spi_clk_config(0);
					goto recovery_done;
				}
				
				gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
				shgrip_spi_clk_config(0);
				
				shgrip_sys_delay_us(SHGRIP_RESET_START_APP_WAIT_US);

				shgrip_rcovery_state_flg = 1;
				SHGRIP_DBG("recovery seq 1 end\n");
				break;
			}
		} else {
			break;
		}
	}
	
	if (ch_state.aet_error || shgrip_rcovery_state_flg) {
		SHGRIP_DBG("recovery seq 2 start\n");
		
		shgrip_rcovery_state_flg = 0;
		ch_state.state_grip = SHGRIP_OFF;
		
		ret = shgrip_cmd_rstsrcr();
		if (ret) {
			SHGRIP_ERR("shgrip_cmd_rstsrcr err. ret=%d\n", ret);
		}
		
		ret = shgrip_seq_start_app();
		if (ret) {
			SHGRIP_ERR("shgrip_seq_start_app err. ret=%d\n", ret);
			goto recovery_done;
		}
		
		ret = shgrip_cmd_chenr(SHGRIP_OFF);
		if (ret) {
			SHGRIP_ERR("shgrip_cmd_chenr off err. ret=%d\n", ret);
			goto recovery_done;
		}
		
		ret = shgrip_cmd_chenr(SHGRIP_ON);
		if (ret) {
			SHGRIP_ERR("shgrip_cmd_chenr err. ret=%d\n", ret);
			goto recovery_done;
		}
		
		shgrip_status = SHGRIP_STATE_SENSOR_ON;

		SHGRIP_DBG("recovery seq 2 end\n");
		goto recovery_done;
	}
	
recovery_done:
	code = SW_GRIP_00;
	input_report_switch(shgrip_input, code, ch_state.state_grip);
	input_sync(shgrip_input);
	
	SHGRIP_DBG("input_event sync code:0x%04X, onoff:%d\n", code, ch_state.state_grip);
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	if ((ch_state.state_grip == SHGRIP_ON) 
	 || (ch_state.state_grip == SHGRIP_OFF)) {
		SHGRIP_DBG("msm_tps_set_grip_state called grip:%d\n", ch_state.state_grip);
		msm_tps_set_grip_state(ch_state.state_grip);
	} else {
		SHGRIP_ERR("grip state failed, grip:%d\n", ch_state.state_grip);
	}
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_work_func                                                          */
/* ------------------------------------------------------------------------- */
static void shgrip_work_func(struct work_struct *work)
{
    int ret;

	mutex_lock(&shgrip_mutex_lock);
	
	SHGRIP_DBG("start\n");
	
	if (shgrip_suspend_state) {
		SHGRIP_WARN("Already Suspended\n");
		goto shgrip_work_func_done;
	}
	
	switch (shgrip_status) { 
	case SHGRIP_STATE_SENSOR_ON:
		ret = shgrip_input_event();
		if(ret){
			shgrip_seq_grip_power_off();
		}else{
			shgrip_sys_enable_irq();
		}
		break;
	case SHGRIP_STATE_POWER_OFF:
	case SHGRIP_STATE_SENSOR_OFF:
	case SHGRIP_STATE_FW_DL:
	case SHGRIP_STATE_HALT_MODE:
	default:
		SHGRIP_WARN("state is changed state:%d\n", shgrip_status); 
		break;
	}
	
shgrip_work_func_done:
	mutex_unlock(&shgrip_mutex_lock);
	wake_unlock(&shgrip_wake_lock);
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_irq_func                                                           */
/* ------------------------------------------------------------------------- */
static irqreturn_t shgrip_irq_func(int irq, void *dev)
{
	SHGRIP_DBG("start\n");
	
	wake_lock(&shgrip_wake_lock);
	
	shgrip_sys_disable_irq();
	
	queue_work(shgrip_work_queue, &shgrip_work_data);
	return IRQ_HANDLED;
}

#ifdef CONFIG_ANDROID_ENGINEERING
/* ========================================================================= */
/* SHGRIP Debug Function                                                     */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_th_dump_func                                                       */
/* ------------------------------------------------------------------------- */
static void shgrip_th_dump_func(struct work_struct *work)
{
	int ret = 0;
	unsigned long sec,nsec;
	unsigned char ch0sig_buf[4];
	unsigned char ch4sig_buf[4];
	unsigned char ch0th_buf[5];
	unsigned char ch4th_buf[5];
	unsigned char gripsr_data = 0;
	unsigned char ch0sr_data = 0;
	unsigned char ch4sr_data = 0;
	
	mutex_lock(&shgrip_mutex_lock);
	
	SHGRIP_DBG("start\n");
	
	if (shgrip_suspend_state) {
		SHGRIP_WARN("Already Suspended\n");
		goto shgrip_work_func_done;
	}
	
	memset(ch0sig_buf, 0, sizeof(ch0sig_buf));
	memset(ch4sig_buf, 0, sizeof(ch4sig_buf));
	memset(ch0th_buf,  0, sizeof(ch0th_buf));
	memset(ch4th_buf,  0, sizeof(ch4th_buf));
	
	switch (shgrip_status) { 
	case SHGRIP_STATE_SENSOR_ON:
		if (shgrip_th_interval_ms) {
			if (th_msec >= 1000) {
				sec  = th_msec / 1000;
				nsec = (th_msec % 1000) * 1000 * 1000;
			} else {
				sec  = 0;
				nsec = th_msec * 1000 * 1000;
			}
			
			hrtimer_start(&shgrip_threshold_timer, ktime_set(sec, nsec), HRTIMER_MODE_REL);
		} else {
			th_msec = th_msec_tmp;
			hrtimer_flg = 0;
		}
		
		ret = shgrip_cmd_ch0sig(ch0sig_buf);
		if (ret) {
			SHGRIP_ERR("shgrip_cmd_ch0sig err. ret:%d\n", ret);
		}
		ret = shgrip_cmd_ch4sig(ch4sig_buf);
		if (ret) {
			SHGRIP_ERR("shgrip_cmd_ch4sig err. ret:%d\n", ret);
		}
		ret = shgrip_cmd_ch0pth(ch0th_buf);
		if (ret) {
			SHGRIP_ERR("shgrip_cmd_ch0pth err. ret:%d\n", ret);
		}
		ret = shgrip_cmd_ch4pth(ch4th_buf);
		if (ret) {
			SHGRIP_ERR("shgrip_cmd_ch4pth err. ret:%d\n", ret);
		}
		ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_GRIPSR, &gripsr_data, 1);
		if (ret) {
			SHGRIP_ERR("gripsr_data read err. ret:%d\n", ret);
		}

		ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_CH0SR, &ch0sr_data, 1);
		if (ret) {
			SHGRIP_ERR("ch0sr_data read err. ret:%d\n", ret);
		}

		ret = shgrip_spi_transfer_read_block(SHGRIP_CMD_CH4SR, &ch4sr_data, 1);
		if (ret) {
			SHGRIP_ERR("ch4sr_data read err. ret:%d\n", ret);
		}
		
		SHGRIP_INFO("CH0 Dlt:0x0000, Ref:0x%02X%02X, Meas:0x%02X%02X, PROXTH:0x%02X%02X, TouchTH:0x%02X%02X, ReleaseTH:0x%02X, GRIPSR:0x%02X, CHSR:0x%02X\n",
			ch0sig_buf[2], ch0sig_buf[3], ch0sig_buf[0], ch0sig_buf[1],
			ch0th_buf[0], ch0th_buf[1], ch0th_buf[2], ch0th_buf[3], ch0th_buf[4], gripsr_data, ch0sr_data );
		SHGRIP_INFO("CH4 Dlt:0x0000, Ref:0x%02X%02X, Meas:0x%02X%02X, PROXTH:0x%02X%02X, TouchTH:0x%02X%02X, ReleaseTH:0x%02X, GRIPSR:0x%02X, CHSR:0x%02X\n",
			ch4sig_buf[2], ch4sig_buf[3], ch4sig_buf[0], ch4sig_buf[1],
			ch4th_buf[0], ch4th_buf[1], ch4th_buf[2], ch4th_buf[3], ch4th_buf[4], gripsr_data, ch4sr_data );
		
		SHGRIP_WARN("%d, %d, 0, %d, %d, 0, %d, %d, %d, %d, %d, %d\n",
			((ch0sig_buf[2] << 8) | ch0sig_buf[3]),
			((ch0sig_buf[0] << 8) | ch0sig_buf[1]),
			((ch4sig_buf[2] << 8) | ch4sig_buf[3]),
			((ch4sig_buf[0] << 8) | ch4sig_buf[1]),
			((ch0th_buf[0] << 8) | ch0th_buf[1]),
			((ch0th_buf[2] << 8) | ch0th_buf[3]),
			  ch0th_buf[4],
			((ch4th_buf[0] << 8) | ch4th_buf[1]),
			((ch4th_buf[2] << 8) | ch4th_buf[3]),
			  ch4th_buf[4]);
		break;
	case SHGRIP_STATE_POWER_OFF:
	case SHGRIP_STATE_SENSOR_OFF:
	case SHGRIP_STATE_FW_DL:
	case SHGRIP_STATE_HALT_MODE:
	default:
		th_msec = th_msec_tmp;
		hrtimer_flg = 0;
		break;
	}
shgrip_work_func_done:
	mutex_unlock(&shgrip_mutex_lock);
	if(!hrtimer_flg) {
		wake_unlock(&shgrip_dump_wake_lock);
	}
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_th_timer_callback                                                  */
/* ------------------------------------------------------------------------- */
static enum hrtimer_restart shgrip_th_timer_callback(struct hrtimer *timer)
{
	queue_work(shgrip_work_queue, &th_work_data);
	return HRTIMER_NORESTART;
}
#endif /* CONFIG_ANDROID_ENGINEERING */

/* ========================================================================= */
/* SHGRIP Interface                                                          */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_open                                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	
	mutex_lock(&shgrip_mutex_lock);
	
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("Grip Sensor Device not connected\n");
		mutex_unlock(&shgrip_mutex_lock);
		return -1;
	}
	
	SHGRIP_DBG("shgrip_status:%d\n", shgrip_status);
	
	switch (shgrip_status) {
	case SHGRIP_STATE_POWER_OFF:
	case SHGRIP_STATE_HALT_MODE:
		ret = shgrip_seq_start_app();
		if (ret) {
			SHGRIP_ERR("shgrip_seq_start_app failed ret:%d\n", ret);
			ret = -1;
		}
		break;
	case SHGRIP_STATE_SENSOR_OFF:
	case SHGRIP_STATE_SENSOR_ON:
	case SHGRIP_STATE_FW_DL:
	default:
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		break;
	}
	
	mutex_unlock(&shgrip_mutex_lock);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_close                                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_close(struct inode *inode, struct file *file)
{
	int ret = 0;

	mutex_lock(&shgrip_mutex_lock);
	
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("Grip Sensor Device not connected\n");
		mutex_unlock(&shgrip_mutex_lock);
		return -1;
	}
	
	switch (shgrip_status) {
	case SHGRIP_STATE_SENSOR_OFF:
		shgrip_seq_grip_power_off();
		break;
	case SHGRIP_STATE_SENSOR_ON:
		ret = shgrip_seq_grip_sensor_off();
		if (ret) {
			SHGRIP_ERR("shgrip_seq_grip_sensor_off failed\n");
			SHGRIP_DBG("disable_irq(SHGRIP_IRQ_GRIP_INT) \n");
			shgrip_sys_disable_irq();
			shgrip_sys_disable_irq_wake();
		}
		shgrip_seq_grip_power_off();
		break;
	case SHGRIP_STATE_POWER_OFF:
	case SHGRIP_STATE_FW_DL:
	case SHGRIP_STATE_HALT_MODE:
	default:
		SHGRIP_ERR("state is Err state:%d\n", shgrip_status);
		break;
	}
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	SHGRIP_DBG("msm_tps_set_grip_state called grip:0\n");
	msm_tps_set_grip_state(SHGRIP_OFF);
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	mutex_unlock(&shgrip_mutex_lock);
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_read                                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_read(struct file* filp, char* buf, 
										size_t count, loff_t* offset)
{
	SHGRIP_DBG("start\n");
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_write                                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_write(struct file* filp, const char* buf, 
										size_t count, loff_t* offset)
{
	SHGRIP_DBG("start\n");
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl                                                              */
/* ------------------------------------------------------------------------- */
static long shgrip_ioctl(struct file *file, unsigned int cmd, 
										unsigned long arg)
{
	int ret;
	void __user *argp = (void __user*)arg;
	
	mutex_lock(&shgrip_mutex_lock);
	
	SHGRIP_INFO("start, cmd:%d\n", (cmd & 0x000000FF));
	
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("Grip Sensor Device not connected\n");
		mutex_unlock(&shgrip_mutex_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	switch (cmd) {
	case SHGRIP_IOCTL_GRIP_SENSOR_ON:
		ret = shgrip_ioctl_grip_sensor_on();
		break;
	case SHGRIP_IOCTL_GRIP_SENSOR_OFF:
		ret = shgrip_ioctl_grip_sensor_off();
		break;
	case SHGRIP_IOCTL_SET_SENSOR_ADJUST:
		ret = shgrip_ioctl_set_sensor_adjust(argp);
		break;
	case SHGRIP_IOCTL_GET_SENSOR_ADJUST:
		ret = shgrip_ioctl_get_sensor_adjust(argp);
		break;
	case SHGRIP_IOCTL_GET_STATE:
		ret = shgrip_ioctl_get_state(argp);
		break;
	case SHGRIP_IOCTL_GET_FW_VERSION:
		ret = shgrip_ioctl_get_fw_version(argp);
		break;
	case SHGRIP_IOCTL_DOWNLOAD_FW:
		ret = shgrip_ioctl_download_fw(argp);
		break;
	case SHGRIP_IOCTL_DIAG_DOWNLOAD_FW:
		ret = shgrip_ioctl_diag_download_fw(argp);
		break;
	case SHGRIP_IOCTL_GET_DRV_STATUS:
		ret = shgrip_ioctl_get_drv_status(argp);
		break;
	case SHGRIP_IOCTL_DIAG_CHECK_SUM:
		ret = shgrip_ioctl_diag_check_sum(argp);
		break;
	case SHGRIP_IOCTL_DEBUG_RW_COMMAND:
		ret = shgrip_ioctl_debug_rw_command(argp);
		break;
	case SHGRIP_IOCTL_DEBUG_RW_COMMAND2:
		ret = shgrip_ioctl_debug_rw_command2(argp);
		break;
	case SHGRIP_IOCTL_SET_BK_ADJUST:
		ret = shgrip_ioctl_set_bk_adjust(argp);
		break;
	case SHGRIP_IOCTL_GET_CHPRM_VALUE:
		ret = shgrip_ioctl_get_chprm_value(argp);
		break;
	case SHGRIP_IOCTL_SET_RESET:
		ret = shgrip_ioctl_set_reset();
		break;
	default:
		SHGRIP_DBG("invalid_value cmd:%d\n", (cmd & 0x000000FF));
		ret = GRIP_RESULT_FAILURE;
		break;
	}
	
	SHGRIP_INFO("end, cmd:%d, ret=%d\n", (cmd & 0x000000FF), ret);
	
	mutex_unlock(&shgrip_mutex_lock);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_dev_spi_probe                                                      */
/* ------------------------------------------------------------------------- */
static int __devinit shgrip_dev_spi_probe(struct spi_device *spi)
{
#ifdef SHGRIP_FACTORY_MODE_ENABLE
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	spi_dev = spi;
	
#ifdef CONFIG_ANDROID_ENGINEERING
	ret = device_create_file(&spi->dev, &dev_attr_shgrip_dev);
	if (ret){
		SHGRIP_ERR("device_create_file failed ret:%d\n", ret);
	}
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */
	
	shgrip_dev_connect = true;
	
	ret = shgrip_seq_start_app();
	if (ret) {
		SHGRIP_ERR("shgrip_seq_start_app err. ret=%d\n", ret);
	}
	
	ret = shgrip_cmd_chenr(SHGRIP_OFF);
	if (ret) {
		SHGRIP_ERR("shgrip_cmd_chenr err. ret=%d\n", ret);
	}
	
	shgrip_seq_grip_power_off();
	
	return ret;
#else
	int i = 0;
	int ret = 0;
	int size = 0;
	unsigned char sum_val = 0;
	unsigned char *buf;

	SHGRIP_DBG("start\n");
	
	spi_dev = spi;
	
#ifdef CONFIG_ANDROID_ENGINEERING
	ret = device_create_file(&spi->dev, &dev_attr_shgrip_dev);
	if (ret){
		SHGRIP_ERR("device_create_file failed ret:%d\n", ret);
		return ret;
	}
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */
	
	for (i = 1; i <= 3; i++) {
		shgrip_seq_reset();
		
		ret = shgrip_seq_start_loader();
		if (ret) {
			SHGRIP_ERR("Loader Start Err:%d\n", ret);
			shgrip_dev_connect = false;
			continue;
		}
		
		ret = shgrip_cmd_check_sum();
		if (!ret) {
			shgrip_dev_connect = true;
			break;
		} else if (ret == SHGRIP_CMD_NACK) {
			shgrip_dev_connect = true;
			break;
		} else {
			SHGRIP_ERR("Check Sum Err:%d Count:%d\n", ret, i);
			shgrip_dev_connect = false;
		}
	}
	
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("GripSensor Not Connect\n");
		goto shgrip_fw_work_power_off;
	}
	
	ret = shgrip_seq_start_checksum(&sum_val);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_start_checksum failed\n");
	}
	
	if (sum_val == SHGRIP_FW_CHECK_SUM_VAL) {
		goto shgrip_fw_work_power_off;
	}
	
	buf = (unsigned char *)shgrip_fw_image;
	size = sizeof(shgrip_fw_image);
	
	for (i = 0; i < 2; i++) {
		ret = shgrip_seq_download_fw_probe(buf, size);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_download_fw_probe failed\n");
			continue;
		}
		ret = shgrip_seq_start_checksum(&sum_val);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_start_checksum failed\n");
		} else {
			if (sum_val == SHGRIP_FW_CHECK_SUM_VAL) {
				break;
			} else {
				SHGRIP_ERR("SUM VALUE is Wrong, Sum:%02x\n", sum_val);
				ret = -1;
			}
		}
	}
	
	if (ret) {
		SHGRIP_ERR("DOWNLOAD FW failed\n");
		shgrip_dev_connect = false;
	}
	
shgrip_fw_work_power_off:
	shgrip_cmd_chenr(SHGRIP_OFF);
	shgrip_seq_grip_power_off();
	return ret;
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
}

/* ------------------------------------------------------------------------- */
/* shgrip_dev_spi_remove                                                     */
/* ------------------------------------------------------------------------- */
static int __devexit shgrip_dev_spi_remove(struct spi_device *spi)
{
	SHGRIP_DBG("start\n");
	
#ifdef CONFIG_ANDROID_ENGINEERING
	device_remove_file(&spi->dev, &dev_attr_shgrip_dev);
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_dev_init                                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_dev_init(void)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	gpio_request(SHGRIP_SPI_CS_N, NULL);
	ret = gpio_tlmm_config(GPIO_CFG(SHGRIP_SPI_CS_N, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret) {
		SHGRIP_ERR("GPIO(SPI_CS) set config NG\n");
		goto shgrip_gpio_cs_err;
	}
	ret = gpio_direction_output(SHGRIP_SPI_CS_N, 1);
	if (ret) {
		SHGRIP_ERR("GPIO(SPI_CS) set direction NG\n");
		goto shgrip_gpio_cs_err;
	}
	
	gpio_request(SHGRIP_GPIO_GRIP_INT, NULL);
	ret = gpio_tlmm_config(GPIO_CFG(SHGRIP_GPIO_GRIP_INT, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret) {
		SHGRIP_ERR("GPIO(TC_PULSE) set config NG\n");
		goto shgrip_gpio_int_err;
	}
	ret = gpio_direction_input(SHGRIP_GPIO_GRIP_INT);
	if (ret) {
		SHGRIP_ERR("GPIO(TC_PULSE) set direction NG\n");
		goto shgrip_gpio_int_err;
	}
	
	ret = shgrip_sys_request_irq();
	if (ret) {
		SHGRIP_ERR("GPIO(TC_PULSE) set direction NG\n");
		goto shgrip_gpio_int_err;
	}
	
	ret = spi_register_driver(&shgrip_dev_spi_driver);
	if (ret) {
		SHGRIP_ERR("spi_register_driver err\n");
		goto shgrip_spi_register_err;
	}
	
	return 0;
	
shgrip_spi_register_err:
	shgrip_sys_free_irq();
shgrip_gpio_int_err:
	gpio_free(SHGRIP_GPIO_GRIP_INT);
shgrip_gpio_cs_err:
	gpio_free(SHGRIP_SPI_CS_N);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_dev_exit                                                           */
/* ------------------------------------------------------------------------- */
static void shgrip_dev_exit(void)
{
	SHGRIP_DBG("start\n");
	
	spi_unregister_driver(&shgrip_dev_spi_driver);
	
	shgrip_sys_free_irq();
	
	gpio_free(SHGRIP_GPIO_GRIP_INT);
	gpio_free(SHGRIP_SPI_CS_N);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_drv_init                                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_drv_init(void)
{
	int ret;
	
	SHGRIP_DBG("start\n");
	
	shgrip_input = input_allocate_device();
	if (!shgrip_input) {
		SHGRIP_ERR("input_allocate_device failed\n");
		return -ENOMEM;
	}
	
	shgrip_input->name         = SHGRIP_NAME;
	shgrip_input->phys         = "shgrip/input0";
	shgrip_input->id.vendor    = 0x0001;
	shgrip_input->id.product   = 0x0001;
	shgrip_input->id.version   = 0x0001;
	
	shgrip_input->evbit[0]     = BIT_MASK(EV_SW);
	
	input_set_capability(shgrip_input, EV_SW, SW_GRIP_00);
	input_set_capability(shgrip_input, EV_SW, SW_GRIP_01);
	input_set_capability(shgrip_input, EV_SW, SW_GRIP_02);
	
	ret = input_register_device(shgrip_input);
	if (ret) {
		SHGRIP_ERR("input_register_device failed\n");
		input_free_device(shgrip_input);
		return ret;
	}
	
#ifdef CONFIG_ANDROID_ENGINEERING
	hrtimer_init(&shgrip_threshold_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	shgrip_threshold_timer.function = shgrip_th_timer_callback;
#endif /* CONFIG_ANDROID_ENGINEERING */
	
	shgrip_work_queue = create_singlethread_workqueue("shgrip_workqueue");
	if (shgrip_work_queue) {
		INIT_WORK(&shgrip_work_data, shgrip_work_func);
#ifdef CONFIG_ANDROID_ENGINEERING
		INIT_WORK(&th_work_data, shgrip_th_dump_func);
#endif /* CONFIG_ANDROID_ENGINEERING */
	} else {
		SHGRIP_ERR("create_singlethread_workqueue failed\n");
		ret = -ENOMEM;
		input_unregister_device(shgrip_input);
		return ret;
	}
	
	mutex_init(&shgrip_mutex_lock);
	spin_lock_init(&shgrip_spinlock);
	wake_lock_init(&shgrip_wake_lock, WAKE_LOCK_SUSPEND, "shgrip_wake_lock");
#ifdef CONFIG_ANDROID_ENGINEERING
	wake_lock_init(&shgrip_dump_wake_lock, WAKE_LOCK_SUSPEND, "shgrip_dump_wake_lock");
#endif /* CONFIG_ANDROID_ENGINEERING */
	
	pm_qos_add_request(&shgrip_qos_cpu_dma_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_drv_exit                                                           */
/* ------------------------------------------------------------------------- */
static void shgrip_drv_exit(void)
{
	SHGRIP_DBG("start\n");
	
	if (shgrip_work_queue) {
		flush_workqueue(shgrip_work_queue);
		destroy_workqueue(shgrip_work_queue);
		shgrip_work_queue = NULL;
	}
	
	input_unregister_device(shgrip_input);
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_init                                                               */
/* ------------------------------------------------------------------------- */
static int __init shgrip_init(void)
{
	int ret;
	
	SHGRIP_DBG("start\n");
	
	ret = alloc_chrdev_region(&shgrip_dev, 0, 1, SHGRIP_NAME);
	if (!ret) {
		shgrip_major = MAJOR(shgrip_dev);
		shgrip_minor = MINOR(shgrip_dev);
    }else{
		SHGRIP_ERR("alloc_chrdev_region failed\n");
		return ret;
	}
	
	cdev_init(&shgrip_cdev, &shgrip_fops);
	
	shgrip_cdev.owner = THIS_MODULE;
	shgrip_cdev.ops   = &shgrip_fops;
	
	ret = cdev_add(&shgrip_cdev, MKDEV(shgrip_major, 0), 1);
	if (ret) {
		SHGRIP_ERR("cdev_add failed\n");
		goto error_cdev_add;
	}
	
	shgrip_class = class_create(THIS_MODULE, SHGRIP_NAME);
	if (IS_ERR(shgrip_class)) {
		ret = PTR_ERR(shgrip_class);
		SHGRIP_ERR("class_create failed\n");
		goto error_class_create;
	}
	
	shgrip_device = device_create(shgrip_class, NULL, 
									shgrip_dev, &shgrip_cdev, SHGRIP_NAME);
	if (IS_ERR(shgrip_device)) {
		ret = PTR_ERR(shgrip_device);
		SHGRIP_ERR("device_create failed\n");
		goto error_device_create;
	}
	
	ret = shgrip_drv_init();
	if (ret) {
		SHGRIP_ERR("shgrip_drv_init failed\n");
		goto error_drv_init;
	}
	
	ret = shgrip_dev_init();
	if (ret) {
		SHGRIP_ERR("shgrip_dev_init failed\n");
		goto error_dev_init;
	}
	
	return 0;
	
error_dev_init:
	shgrip_drv_exit();
error_drv_init:
	device_destroy(shgrip_class, MKDEV(shgrip_major, 0));
error_device_create:
	class_destroy(shgrip_class);
error_class_create:
	cdev_del(&shgrip_cdev);
error_cdev_add:
	unregister_chrdev_region(MKDEV(shgrip_major, 0), 1);
	SHGRIP_ERR("FAILED\n");
	return ret;
}
module_init(shgrip_init);

/* ------------------------------------------------------------------------- */
/* shgrip_exit                                                               */
/* ------------------------------------------------------------------------- */
static void __exit shgrip_exit(void)
{
	SHGRIP_DBG("start\n");
	
	shgrip_dev_exit();
	
	shgrip_drv_exit();
	
	pm_qos_remove_request(&shgrip_qos_cpu_dma_latency);
	
	wake_unlock(&shgrip_wake_lock);
	wake_lock_destroy(&shgrip_wake_lock);
	
	device_destroy(shgrip_class, MKDEV(shgrip_major,0));
	class_destroy(shgrip_class);
	cdev_del(&shgrip_cdev);
	unregister_chrdev_region(MKDEV(shgrip_major, 0), 1);
	
	return;
}
module_exit(shgrip_exit);

/* ------------------------------------------------------------------------- */
MODULE_DESCRIPTION("SHARP GRIP SENSOR DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
