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
#define SHGRIP_SPI_MULTI_TRANSFER

/* #define SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC */

/* #define SHGRIP_FW_DATA_NOT_WRITE */
#ifndef SHGRIP_FW_DATA_NOT_WRITE
/* #define SHGRIP_LORDER_WRITE */
/* #define SHGRIP_FW_DATA_MULTI_WRITE */
#endif /* SHGRIP_FW_DATA_NOT_WRITE */

#define SHGRIP_DBG_POLLING_CHPRD2

/* --------------------------------------------------------- */
/* Software Parameter                                        */
/* --------------------------------------------------------- */
#define SHGRIP_NAME							"shgrip"
#define SHGRIP_DEVNAME						"shgrip_dev"

#define SHGRIP_CHANNEL_NUM					(2)

#define SHGRIP_RESET_RECOVERY_RETRY_COUNT	(3)

#define SHGRIP_CHG_LDR_VERIFY_COUNT			(0)
#define SHGRIP_CHG_LDR_MODE_RETRY_COUNT		(5)

#define SHGRIP_GSTCLR_RETRY_COUNT			(5)

#define SHGRIP_IRQ_DISABLED					(0)
#define SHGRIP_IRQ_ENABLED					(1)
#define SHGRIP_IRQ_FREE						(0)
#define SHGRIP_IRQ_REQUESTED				(1)

#define SHGRIP_WAKE_DISABLED				(0)
#define SHGRIP_WAKE_ENABLED					(1)

/* --------------------------------------------------------- */
/* System Parameter                                          */
/* --------------------------------------------------------- */
#define SPI_BUFFER							(259)
#define SPI_CLK_SPEED						(960000)
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
#define SHGRIP_GPIO_GRIP_COM_REQ			qpnp_pin_map("pm8941-gpio", 18)
#define SHGRIP_GPIO_GRIP_COM_READY			qpnp_pin_map("pm8941-gpio", 26)
#define SHGRIP_GPIO_GRIP_POW				qpnp_pin_map("pm8941-gpio", 32)

/* --------------------------------------------------------- */
/* Timer Parameter                                           */
/* --------------------------------------------------------- */
#define SHGRIP_STARTUP_WAIT					(125*1000)		
#define SHGRIP_RESET_START_WAIT_US			(3*1000)		

#define SHGRIP_PWR_OFF_WAIT_US				(10)			

#define SHGRIP_OCO_WAIT_US					(1*1000)		

#define SHGRIP_RESET_TIMEOUT_US				(300*1000)		
#define SHGRIP_POLL_RESET_WAIT_US			(5*1000)		

#define SHGRIP_READY_TIMEOUT_US				(100*1000)
#define SHGRIP_COMRDY_WAIT_MAX_US			(150*1000)		
#define SHGRIP_COMRDY_SEND_WAIT_MAX_US		(300*1000)		

#define SHGRIP_WAIT_BYTE_US					(280)			
#define SHGRIP_WAIT_CMD_US					(500)			
#define SHGRIP_WAIT_CMDEND_US				(2*1000)		
#define SHGRIP_CTS_TIMEOUT_US				(300)			
#define SHGRIP_SPI_SYNC_ERR_WAIT_US			(10*1000)		

#define SHGRIP_POLL_WAIT_US					(100)			
#define SHGRIP_POLL_WAIT_US_FOR_LOADER		(5)				
#define SHGRIP_ERACE_POLL_WAIT_US			(10*1000)		
#define SHGRIP_PGWRITE_POLL_WAIT_US			(1*1000)		

#define SHGRIP_CHKID_TIMEOUT_US 			(1*1000*1000)	
#define SHGRIP_CLRSTS_TIMEOUT_US			(10*1000)		
#define SHGRIP_ERASE_TIMEOUT_US				(10*1000*1000)	
#define SHGRIP_PGWRITE_TIMEOUT_US			(1*1000*1000)	
#define SHGRIP_PGREAD_TIMEOUT_US			(1*1000*1000)	
#define SHGRIP_CHGMODE_TIMEOUT_US			(10*1000)		


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

#define SHGRIP_CMD_PVER						(0x01)
#define SHGRIP_CMD_TTES						(0x06)
#define SHGRIP_CMD_RDST						(0x0E)
#define SHGRIP_CMD_ROMC						(0x0F)
#define SHGRIP_CMD_LVER						(0x10)
#define SHGRIP_CMD_LVER1					(0x1E)
#define SHGRIP_CMD_GSWR1					(0x1F)
#define SHGRIP_CMD_GSWR2					(0x20)
#define SHGRIP_CMD_GSRD1					(0x27)
#define SHGRIP_CMD_GSRD2					(0x28)
#define SHGRIP_CMD_GSTCLR					(0x21)
#define SHGRIP_CMD_GSTRST					(0x22)
#define SHGRIP_CMD_CHPRD					(0x23)
#define SHGRIP_CMD_GINT						(0x24)
#define SHGRIP_CMD_GINTON					(0x25)
#define SHGRIP_CMD_GINTOFF					(0x26)
#define SHGRIP_CMD_TSON						(0x29)
#define SHGRIP_CMD_TSOFF					(0x2A)
#define SHGRIP_CMD_PTRD						(0x2B)
#define SHGRIP_CMD_TSMP						(0x2C)
#define SHGRIP_CMD_TTESOFF					(0x2D)
#define SHGRIP_CMD_MOSTRD					(0x2E)
#define SHGRIP_CMD_TSMPRD					(0x2F)
#define SHGRIP_CMD_SFRRD1					(0x30)
#define SHGRIP_CMD_SFRRD2					(0x31)
#define SHGRIP_CMD_TH3ON					(0x32)
#define SHGRIP_CMD_TH3OFF					(0x33)
#define SHGRIP_CMD_SETTH3VAL				(0x34)
#define SHGRIP_CMD_TH3VALRD					(0x35)
#define SHGRIP_CMD_DBPORT					(0x36)
#define SHGRIP_CMD_RNDSET					(0x37)
#define SHGRIP_CMD_RNDCLR					(0x38)
#define SHGRIP_CMD_RNDVALSET				(0x39)
#define SHGRIP_CMD_RNDVALRD					(0x3A)
#define SHGRIP_CMD_DRIFTSET					(0x3B)
#define SHGRIP_CMD_DRIFTCLR					(0x3C)
#define SHGRIP_CMD_DRIFTVALRD				(0x3D)
#define SHGRIP_CMD_CHPRD2					(0x3E)
#define SHGRIP_CMD_THR3_CANCELSET			(0x3F)
#define SHGRIP_CMD_THR3_CANCELCLR			(0x40)
#define SHGRIP_CMD_THR3_CANCELVAL			(0x41)
#define SHGRIP_CMD_THR3_CANCELVALRD			(0x42)
#define SHGRIP_CMD_UPHOSEISET				(0x43)
#define SHGRIP_CMD_UPHOSEIRD				(0x44)
#define SHGRIP_CMD_DRIFTSET2				(0x45)
#define SHGRIP_CMD_SCMSASET					(0x49)
#define SHGRIP_CMD_SCMSASET2				(0x4C)
#define SHGRIP_CMD_TH4ON					(0x4E)

#define SHGRIP_CMD_CH1ON					(0x60)
#define SHGRIP_CMD_CH1OFF					(0x61)
#define SHGRIP_CMD_GSWR3					(0x62)
#define SHGRIP_CMD_GSRD3					(0x63)
#define SHGRIP_CMD_CH1STRD					(0x64)
#define SHGRIP_CMD_CHPRD4					(0x65)
#define SHGRIP_CMD_TH5ON					(0x66)
#define SHGRIP_CMD_TH5OFF					(0x67)
#define SHGRIP_CMD_TH5CLR					(0x68)
#define SHGRIP_CMD_TH5VALRD					(0x69)

#define SHGRIP_CMD_PCOVRCON					(0x6A)
#define SHGRIP_CMD_PCOVRCOFF				(0x6B)
#define SHGRIP_CMD_PCOVRCCLR				(0x6C)
#define SHGRIP_CMD_PCOVRCVALRD				(0x6D)

#define SHGRIP_CMD_MOSTRD2					(0x6E)
#define SHGRIP_CMD_UPHOSEISET2				(0x6F)
#define SHGRIP_CMD_UPHOSEISET3				(0x70)
#define SHGRIP_CMD_UPHOSEIRD2				(0x71)
#define SHGRIP_CMD_UPHOSEIRD3				(0x72)
#define SHGRIP_CMD_UPHOSEIRD4				(0x73)

#define SHGRIP_CMD_SCMSA2SET				(0x74)
#define SHGRIP_CMD_SCMSA2CLR				(0x75)
#define SHGRIP_CMD_SCMSA2VALRD				(0x76)

#define SHGRIP_CMD_SUPHOSEISET				(0x77)
#define SHGRIP_CMD_SUPHOSEISET2				(0x78)
#define SHGRIP_CMD_SCUPLIMITSET				(0x7B)

#define SHGRIP_CMD_TH1DON					(0x8E)
#define SHGRIP_CMD_TH1DVALRD				(0x8F)

#define SHGRIP_CMD_ACK						(0x06)
#define SHGRIP_CMD_NACK						(0x15)

#define SHGRIP_LDR_CMD_PAGE_READ			(0xFF)
#define SHGRIP_LDR_CMD_PAGE_PROGRAM			(0x41)
#define SHGRIP_LDR_CMD_BLOCK_ERASE			(0x20)
#define SHGRIP_LDR_CMD_READ_STS_REG			(0x70)
#define SHGRIP_LDR_CMD_CLEAR_STS_REG		(0x50)
#define SHGRIP_LDR_CMD_ID_CHECK				(0xF5)
#define SHGRIP_LDR_CMD_VERSION_INFO			(0xFB)

#define SHGRIP_LDR_ERASE_DO					(0xD0)

#define SHGRIP_LDR_FW_ID1					(0x55)
#define SHGRIP_LDR_FW_ID2					(0x4b)
#define SHGRIP_LDR_FW_ID3					(0x2d)
#define SHGRIP_LDR_FW_ID4					(0x6a)
#define SHGRIP_LDR_FW_ID5					(0x54)
#define SHGRIP_LDR_FW_ID6					(0x4f)
#define SHGRIP_LDR_FW_ID7					(0x2d)

#define SHGRIP_SMP_MIN_VAL					(0x01)
#define SHGRIP_SMP_MAX_VAL					(0x12)

/* --------------------------------------------------------- */
/* Firmware Parameter                                        */
/* --------------------------------------------------------- */
#define SHGRIP_FW_LOADER0_OLDEST_VERSION	0x0204			
#define SHGRIP_FW_LOADER0_LATEST_VERSION	SHGRIP_LVER0	

#define SHGRIP_FW_LOADER1_OLDEST_VERSION	0x0250			
#define SHGRIP_FW_LOADER1_LATEST_VERSION	SHGRIP_LVER1	

#define SHGRIP_FW_VERSION					0x0409

#define SHGRIP_LDR_ADDR_LDR_BLK				(0x0E000)
#define SHGRIP_LDR_ADDR_LDR_BLK_TERMINATE	(0x0EFFF)
#define SHGRIP_LDR_BLK_ADDR_TBL_END			(0xFFFFF)

#define SHGRIP_LDR_ADDR_APP_BLK1			(0x04000)
#define SHGRIP_LDR_ADDR_APP_BLK2			(0x08000)
#define SHGRIP_LDR_ADDR_APP_BLK3			(0x0C000)
#define SHGRIP_LDR_ADDR_APP_BLK_TERMINATE	(0x0DFFF)

#define SHGRIP_LDR_SRD_SEQ					(0x80)
#define SHGRIP_LDR_SRD_ERZ					(0x20)
#define SHGRIP_LDR_SRD_PRG					(0x10)

#define SHGRIP_LDR_SRD_ID_CHK				(0x0C)

#define SHGRIP_LDR_PAGE_SIZE				(256)


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
/* Firmware Parameter                                        */
/* --------------------------------------------------------- */
static unsigned long shgrip_blk_addr_ldr_tbl[] = 
{
	SHGRIP_LDR_ADDR_LDR_BLK,
	SHGRIP_LDR_BLK_ADDR_TBL_END
};

static unsigned long shgrip_blk_addr_app_tbl[] = 
{
	SHGRIP_LDR_ADDR_APP_BLK1,
	SHGRIP_LDR_ADDR_APP_BLK2,
	SHGRIP_LDR_ADDR_APP_BLK3,
	SHGRIP_LDR_BLK_ADDR_TBL_END
};

static unsigned long *shgrip_block_addr_table[] = 
{
	shgrip_blk_addr_ldr_tbl,
	shgrip_blk_addr_app_tbl
};

static unsigned long shgrip_block_tenminate[] = 
{
	SHGRIP_LDR_ADDR_LDR_BLK_TERMINATE,
	SHGRIP_LDR_ADDR_APP_BLK_TERMINATE
};

/* --------------------------------------------------------- */
/* Software Parameter                                        */
/* --------------------------------------------------------- */
struct shgrip_drv {
	spinlock_t					lock;
	bool						suspended;
	
	int							irq;
	int							irq_request;
	int							irq_enabled;
	int							irq_wake;
	
	struct input_dev			*input;
	
	struct workqueue_struct		*work_queue;
	struct work_struct			work_data;
#ifdef SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC
	struct work_struct			fw_work_data;
#endif /* SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC */
	
#if defined (CONFIG_ANDROID_ENGINEERING)
	struct hrtimer				shgrip_threshold_timer;
	struct work_struct			th_work_data;
#endif /* CONFIG_ANDROID_ENGINEERING */
	
	enum shgrip_state			state;
	unsigned char				ch_state[SHGRIP_CHANNEL_NUM];
	
	enum shgrip_loader_mode		loader_mode;
	
	struct shgrip_sens_setting_params bk_adj;
	struct shgrip_sensor_state	sensor_state;
	
	struct shgrip_fw_version	ver_info;
	unsigned char				fw_mode_select;
	
	const unsigned char			*fw_data;
};

static dev_t 				shgrip_dev;
static dev_t				shgrip_major = 0;
static dev_t				shgrip_minor = 0;
static struct cdev 			shgrip_cdev;
static struct class* 		shgrip_class;
static struct device*		shgrip_device;

static struct shgrip_drv grip_ctrl;

static bool shgrip_dev_connect = false;

static struct mutex shgrip_io_lock;
static struct wake_lock shgrip_wake_lock;

static struct pm_qos_request shgrip_qos_cpu_dma_latency;

int shgrip_err_log  = 1;
int shgrip_warn_log = 0;
int shgrip_info_log = 0;
int shgrip_dbg_log  = 0;

#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(shgrip_err_log,  int, 0600);
module_param(shgrip_warn_log, int, 0600);
module_param(shgrip_info_log, int, 0600);
module_param(shgrip_dbg_log,  int, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */

#if defined (CONFIG_ANDROID_ENGINEERING)
static unsigned long shgrip_th_interval_ms = 0;
static unsigned long th_msec = 0;
static unsigned long th_msec_tmp = 0;
static unsigned char hrtimer_flg = 0;
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
	struct shgrip_drv *ctrl = &grip_ctrl;
	unsigned long sec,nsec;
	int ret;
	
	mutex_lock(&shgrip_io_lock);
	
	ret = strict_strtoul(buf, 10, &shgrip_th_interval_ms);
	if (ret) {
		mutex_unlock(&shgrip_io_lock);
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
		
		switch (ctrl->state) { 
		case STATE_SENSOR_ON:
			if (th_msec >= 1000) {
				sec  = th_msec / 1000;
				nsec = (th_msec % 1000) * 1000 * 1000;
			} else {
				sec  = 0;
				nsec = th_msec * 1000 * 1000;
			}
			if (hrtimer_flg == 0) {
				hrtimer_start(&(ctrl->shgrip_threshold_timer), ktime_set(sec, nsec), HRTIMER_MODE_REL);
				hrtimer_flg = 1;
			}
			break;
			
		case STATE_POWER_OFF:
		case STATE_SENSOR_OFF:
		case STATE_FW_DL:
		default:
			th_msec = th_msec_tmp;
			break;
		}
	} else {
		th_msec = shgrip_th_interval_ms;
		th_msec_tmp = shgrip_th_interval_ms;
		hrtimer_flg = 0;
	}
	
	mutex_unlock(&shgrip_io_lock);
	
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
static int shgrip_sys_request_irq(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned long irq_flags;
	
	if (ctrl->irq_request == SHGRIP_IRQ_FREE) {
		irq_set_status_flags(gpio_to_irq(SHGRIP_GPIO_GRIP_INT), IRQ_NOAUTOEN);
		ctrl->irq_enabled = SHGRIP_IRQ_DISABLED;
		
		irq_flags = IRQF_TRIGGER_HIGH;
		ret = request_irq(gpio_to_irq(SHGRIP_GPIO_GRIP_INT), shgrip_irq_func, 
									irq_flags, SHGRIP_NAME, ctrl);
		if (ret) {
			SHGRIP_ERR("request_irq is failed. ret:%d\n", ret);
		} else {
			ctrl->irq_request = SHGRIP_IRQ_REQUESTED;
		}
	}
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_free_irq                                                       */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_free_irq(struct shgrip_drv *ctrl)
{
	if (ctrl->irq_request == SHGRIP_IRQ_REQUESTED) {
		free_irq(gpio_to_irq(SHGRIP_GPIO_GRIP_INT), ctrl);
		ctrl->irq_request = SHGRIP_IRQ_FREE;
	}
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_enable_irq                                                     */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_enable_irq(struct shgrip_drv *ctrl)
{
	unsigned long flags;
	
	spin_lock_irqsave(&(ctrl->lock), flags);
	
	if (ctrl->irq_enabled == SHGRIP_IRQ_DISABLED) {
		enable_irq(ctrl->irq);
		ctrl->irq_enabled = SHGRIP_IRQ_ENABLED;
	}
	
	spin_unlock_irqrestore(&(ctrl->lock), flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_disable_irq                                                    */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_disable_irq(struct shgrip_drv *ctrl)
{
	unsigned long flags;
	
	spin_lock_irqsave(&(ctrl->lock), flags);
	
	if (ctrl->irq_enabled == SHGRIP_IRQ_ENABLED) {
		disable_irq_nosync(ctrl->irq);
		ctrl->irq_enabled = SHGRIP_IRQ_DISABLED;
	}
	
	spin_unlock_irqrestore(&(ctrl->lock), flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_enable_irq_wake                                                */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_enable_irq_wake(struct shgrip_drv *ctrl)
{
	unsigned long flags;
	
	spin_lock_irqsave(&(ctrl->lock), flags);
	
	if (ctrl->irq_wake == SHGRIP_WAKE_DISABLED) {
		enable_irq_wake(ctrl->irq);
		ctrl->irq_wake = SHGRIP_WAKE_ENABLED;
	}
	
	spin_unlock_irqrestore(&(ctrl->lock), flags);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_sys_disable_irq_wake                                               */
/* ------------------------------------------------------------------------- */
static void shgrip_sys_disable_irq_wake(struct shgrip_drv *ctrl)
{
	unsigned long flags;
	
	spin_lock_irqsave(&(ctrl->lock), flags);
	
	if (ctrl->irq_wake == SHGRIP_WAKE_ENABLED) {
		disable_irq_wake(ctrl->irq);
		ctrl->irq_wake = SHGRIP_WAKE_DISABLED;
	}
	
	spin_unlock_irqrestore(&(ctrl->lock), flags);
	
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
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = (struct shgrip_drv *)spi_get_drvdata(spi);
	
	ctrl->suspended = true;
	if (ctrl->state == STATE_SENSOR_ON) {
		shgrip_sys_disable_irq(ctrl);
		shgrip_sys_enable_irq_wake(ctrl);
	}
	
	mutex_unlock(&shgrip_io_lock);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_resume                                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_resume(struct spi_device *spi)
{
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = (struct shgrip_drv *)spi_get_drvdata(spi);
	
	ctrl->suspended = false;
	if (ctrl->state == STATE_SENSOR_ON) {
		shgrip_sys_enable_irq(ctrl);
		shgrip_sys_disable_irq_wake(ctrl);
	}
	
	mutex_unlock(&shgrip_io_lock);

	return 0;
}

/* ========================================================================= */
/* SHGRIP Serial Interface Function                                          */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* bitflip8                                                                  */
/* ------------------------------------------------------------------------- */
#define BIT_NUM (8)
static unsigned char bitflip8(unsigned char base)
{
	unsigned char temp;
	unsigned char val;
	int i;
	
	temp = 0xFF & base;
	val = 0;
	
	for (i = 0; i < BIT_NUM; i++) {
		val |= ((temp >> i) & 0x01) << (BIT_NUM - 1 - i);
	}
	
	return val;
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_read_block                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_read_block(struct shgrip_drv *ctrl,
										unsigned char *rbuf, int rlen)
{
#ifdef SHGRIP_SPI_MULTI_TRANSFER
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	int i;
	int ret = 0;
	unsigned char readdata[SPI_BUFFER];
	
	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}

	if ((rlen > 0) && (rlen <= SPI_BUFFER)) {
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		
		x->bits_per_word    = SPI_BIT_WORD;
		x->len              = rlen;
		x->speed_hz         = SPI_CLK_SPEED;
		x->deassert_wait    = 60;
		
		spi_message_init(&msg);
		
		x->rx_buf           = readdata;
		spi_message_add_tail(x, &msg);
		
		shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
		
		ret = spi_sync(spi_dev, &msg);
		if (ret) {
			SHGRIP_ERR("spi_sync err ret=%d \n", ret);
			shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
		}

		for (i = 0; i < rlen; i++) {
			*rbuf = bitflip8(readdata[i]);
			SHGRIP_DBG("readdata[%d]:0x%02x -> 0x%02x\n", i, readdata[i], *rbuf);
			rbuf++;
		}
		
		if (ctrl->state != STATE_FW_DL) {
			shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
		}
	} else {
		SHGRIP_ERR("rlen fraud. rlen=%d\n", rlen);
		ret = -EINVAL;
	}
	
	return ret;
#else
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
		
		for (i = 0; i < rlen; i++) {
			spi_message_init(&msg);
			
			x->rx_buf           = &readdata;
			spi_message_add_tail(x, &msg);
			
			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				SHGRIP_ERR("spi_sync err ret=%d \n", ret);
				shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
			}
			
			*rbuf = bitflip8(readdata);
			SHGRIP_DBG("readdata[%d]:0x%02x -> 0x%02x\n", i, readdata, *rbuf);
			rbuf++;
			
			if (ctrl->state == STATE_FW_DL) {
				if (i < rlen-1) {
					int time = SHGRIP_CTS_TIMEOUT_US;
					while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
						if (!(time-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
							SHGRIP_ERR("cts is timeout\n");
							return -ETIMEDOUT;
						}
						shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
					}
				}
			} else {
				shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
			}
		}
	} else {
		SHGRIP_ERR("rlen fraud. rlen=%d\n", rlen);
		ret = -EINVAL;
	}
	
	return ret;
#endif /* SHGRIP_SPI_MULTI_TRANSFER */
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_write_block                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_write_block(struct shgrip_drv *ctrl,
											unsigned char *wbuf, int wlen)
{
#ifdef SHGRIP_SPI_MULTI_TRANSFER
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	const unsigned char *data;
	unsigned char senddata[SPI_BUFFER];
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
		x->len              = wlen;
		x->speed_hz         = SPI_CLK_SPEED;
		x->deassert_wait    = 60;
		
		for (i = 0; i < wlen; i++) {
			senddata[i] = bitflip8(*data);
			SHGRIP_DBG("senddata[%d]:0x%02x -> 0x%02x\n", i, *data, senddata[i]);
			data++;
		}
		
		spi_message_init(&msg);
		
		x->tx_buf           = senddata;
		
		spi_message_add_tail(x, &msg);
		
		shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
		
		ret = spi_sync(spi_dev, &msg);
		if (ret) {
			SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
			shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
		}
		if (ctrl->state != STATE_FW_DL){
			shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
		}
	} else {
		SHGRIP_ERR("wlen fraud. wlen=%d\n", wlen);
		ret = -EINVAL;
	}
	
	return ret;
#else
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	const unsigned char *data;
	unsigned char senddata;
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
		
		for (i = 0; i < wlen; i++) {
			senddata = bitflip8(*data);
			SHGRIP_DBG("senddata[%d]:0x%02x -> 0x%02x\n", i, *data, senddata);
			data++;
			
			spi_message_init(&msg);
			
			x->tx_buf           = &senddata;
			
			spi_message_add_tail(x, &msg);
			
			ret = spi_sync(spi_dev, &msg);
			if (ret) {
				SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
				shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
			}
			if (ctrl->state == STATE_FW_DL){
				if (i < wlen-1) {
					int time = SHGRIP_CTS_TIMEOUT_US;
					while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
						if (!(time-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
							SHGRIP_ERR("cts is timeout\n");
							return -ETIMEDOUT;
						}
						shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
					}
				}
			} else {
				shgrip_sys_delay_us(SHGRIP_WAIT_BYTE_US);
			}
		}
	} else {
		SHGRIP_ERR("wlen fraud. wlen=%d\n", wlen);
		ret = -EINVAL;
	}
	
	return ret;
#endif /* SHGRIP_SPI_MULTI_TRANSFER */
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_write_block_special                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_write_block_special(struct shgrip_drv *ctrl,
												unsigned char *headbuf, int headlen,
												const unsigned char *databuf, int datalen)
{
#ifdef SHGRIP_FW_DATA_NOT_WRITE
	unsigned int addr;
	int i;
	addr = ((headbuf[1] << 8) | (headbuf[2] << 16));

	SHGRIP_DBG("addr:0x%08x\n", addr);

	for (i = 1; i < (datalen + 1); i++) {
		if (!(i % 16)) {
			printk("0x%02X,\n", *databuf);
		} else {
			printk("0x%02X,", *databuf);
		}
		databuf++;
	}
	return 0;
#else /* SHGRIP_FW_DATA_NOT_WRITE */
#ifdef SHGRIP_FW_DATA_MULTI_WRITE
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	const unsigned char *data;
	int i, len;
	int ret = 0;
	unsigned char buf[SPI_BUFFER];

	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}
	
	if ((headlen > 0) && (datalen > 0) && (headlen+datalen <= SPI_BUFFER)) {
		data = headbuf;
		
		len = headlen+datalen;
		for (i = 0; i < len; i++) {
			buf[i] = bitflip8(*data);
			SHGRIP_DBG("senddata[%d]:0x%02x -> 0x%02x\n", i, *data, buf[i]);
			data++;
			
			if (i==headlen-1) {
			    data = databuf;
			}
		}
		
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SPI_BIT_WORD;
		x->len              = len;
		x->speed_hz         = SPI_CLK_SPEED;
		x->deassert_wait    = 30;
		
		spi_message_init(&msg);
		
		x->tx_buf           = buf;
		
		spi_message_add_tail(x, &msg);
		
		ret = spi_sync(spi_dev, &msg);
		if (ret) {
			SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
			shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
		}
	} else {
		SHGRIP_ERR("len fraud. headlen=%d datalen=%d\n", headlen, datalen);
		ret = -EINVAL;
	}
	return ret;
#else /* SHGRIP_FW_DATA_MULTI_WRITE */
	struct spi_message  msg;
	struct spi_transfer *x, xfer;
	const unsigned char *data;
	unsigned char senddata;
	int i, len;
	int ret = 0;
	int time = SHGRIP_CTS_TIMEOUT_US;

	SHGRIP_DBG("start\n");
	
	if (!spi_dev) {
		SHGRIP_ERR("spi_dev is NULL\n");
		return -EINVAL;
	}
	
	if ((headlen > 0) && (datalen > 0) && (headlen+datalen <= SPI_BUFFER)) {
		data = headbuf;
		
		memset(&xfer, 0, sizeof(xfer));
		x = &xfer;
		x->bits_per_word    = SPI_BIT_WORD;
		x->len              = 1;
		x->speed_hz         = SPI_CLK_SPEED;
		
		len = headlen+datalen;
		for (i = 0; i < len; i++) {
			senddata = bitflip8(*data);
			SHGRIP_DBG("senddata[%d]:0x%02x -> 0x%02x\n", i, *data, senddata);
			data++;
			
			if (i==headlen-1) {
			    data = databuf;
			}
			
			spi_message_init(&msg);
			
			x->tx_buf           = &senddata;
			
			spi_message_add_tail(x, &msg);
			
			ret = spi_sync(spi_dev, &msg);
			if (ret) {
			    SHGRIP_ERR("spi_sync err. ret=%d \n", ret);
				shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
			}
			
			if (i < len-1) {
				while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
					if (!(time-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
						SHGRIP_ERR("cts is timeout\n");
						return -ETIMEDOUT;
					}
					shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
				}
			}
		}
	} else {
		SHGRIP_ERR("len fraud. headlen=%d datalen=%d\n", headlen, datalen);
		ret = -EINVAL;
	}
	
	return ret;
#endif /* SHGRIP_FW_DATA_MULTI_WRITE */
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer(struct shgrip_drv *ctrl,
										unsigned char *wbuf, int wlen,
										unsigned char *rbuf, int rlen)
{
	int i;
	int ret;
	
	SHGRIP_DBG("start\n");
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_OCO_WAIT_US);
	
	gpio_set_value(SHGRIP_GPIO_GRIP_COM_REQ, SHGRIP_GPIO_VAL_HI);
	
	i = SHGRIP_COMRDY_WAIT_MAX_US;
	while (!(gpio_get_value(SHGRIP_GPIO_GRIP_COM_READY))) {
		if (!(i-=SHGRIP_POLL_WAIT_US)) {
			SHGRIP_ERR("COMRDY check is timeout cmd[%02X]\n", wbuf[0]);
			break;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US);
	};
	
	SHGRIP_DBG("COMRDY wait:%d us\n", (SHGRIP_COMRDY_WAIT_MAX_US - i));
	
	ret = shgrip_spi_transfer_write_block(ctrl, wbuf, wlen);
	if (ret) {
		SHGRIP_ERR("spi read err. ret=%d\n", ret);
		return ret;
	}
	
	shgrip_sys_delay_us(SHGRIP_WAIT_CMD_US);
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	
	i = SHGRIP_COMRDY_SEND_WAIT_MAX_US;
	while (!(gpio_get_value(SHGRIP_GPIO_GRIP_COM_READY))) {
		if (!(i-=SHGRIP_POLL_WAIT_US)) {
			SHGRIP_ERR("COMRDY check is timeout (SEND) cmd[%02X]\n", wbuf[0]);
			break;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US);
	};
	
	SHGRIP_DBG("COMRDY (SEND) wait:%d us\n", (SHGRIP_COMRDY_SEND_WAIT_MAX_US - i));
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_WAIT_CMD_US);
	
	ret = shgrip_spi_transfer_read_block(ctrl, rbuf, rlen);
	if (ret) {
		SHGRIP_ERR("spi read err. ret=%d\n", ret);
		return ret;
	}
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	
	gpio_set_value(SHGRIP_GPIO_GRIP_COM_REQ, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_WAIT_CMDEND_US);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_for_loader                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_for_loader(struct shgrip_drv *ctrl,
											unsigned char *wbuf, int wlen,
											unsigned char *rbuf, int rlen)
{
	int ret;
	
	SHGRIP_DBG("start\n");
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	ret = shgrip_spi_transfer_write_block(ctrl, wbuf, wlen);
	if (ret) {
		SHGRIP_ERR("spi send err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf) {
		ret = shgrip_spi_transfer_read_block(ctrl, rbuf, rlen);
		if (ret) {
			SHGRIP_ERR("spi read err. ret=%d\n", ret);
			return ret;
		}
	}
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_spi_transfer_for_loader_special_write                              */
/* ------------------------------------------------------------------------- */
static int shgrip_spi_transfer_for_loader_special_write(struct shgrip_drv *ctrl,
												unsigned char *headbuf, int headlen,
												const unsigned char *databuf, int datalen)
{
	int ret;
	
	SHGRIP_DBG("start\n");
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_LO);
	
	ret = shgrip_spi_transfer_write_block_special(ctrl, headbuf, headlen, databuf, datalen);
	if (ret) {
		SHGRIP_ERR("spi send err. ret=%d\n", ret);
		return ret;
	}
	
	gpio_set_value(SHGRIP_SPI_CS_N, SHGRIP_GPIO_VAL_HI);
	
	return 0;
}


/* ========================================================================= */
/* SHGRIP Device Control Function App Mode                                   */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_command_pver                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_pver(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_PVER;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	ctrl->ver_info.pver = ((rbuf[2] << 8) | rbuf[1]);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_ttes                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_ttes(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TTES;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_ttesoff                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_command_ttesoff(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TTESOFF;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_rdst                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_rdst(struct shgrip_drv *ctrl,
							unsigned char *grip, unsigned char *testch)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_RDST;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("RDST BAD Value %02x %02x %02x\n",
						rbuf[0], rbuf[1], rbuf[2]);
		return -EFAULT;
	}
	
	if (rbuf[1] & BIT3) {
		if (rbuf[1] & (BIT0 | BIT1)) {
			SHGRIP_ERR("THR3_ERR, THR4_ERR OR THR5_ERR, AND PCOVR_ERR READ_STATE:0x%02x\n", rbuf[1]);
		}else{
			SHGRIP_ERR("PCOVR_ERR READ_STATE:0x%02x\n", rbuf[1]);
		}
	}
	
	if (rbuf[2] & (BIT2 | BIT3)) {
		SHGRIP_ERR("THR3_ERR OR THR5_ERR READ_STATE:0x%02x\n", rbuf[2]);
	}
	
	if (rbuf[2] & BIT4) {
		SHGRIP_ERR("SCMSA_ERR READ_STATE:0x%02x\n", rbuf[2]);
	}
	
	if (rbuf[2] & (BIT5 | BIT6)) {
		SHGRIP_ERR("THR4_ERR READ_STATE:0x%02x\n", rbuf[2]);
	}
	
	SHGRIP_INFO("RDST:%02x %02x %02x\n",
							rbuf[0], rbuf[1], rbuf[2]);
	
	*grip	= rbuf[1];
	*testch	= (rbuf[2] & (BIT1 | BIT0));
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_romc                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_romc(struct shgrip_drv *ctrl, unsigned short *sum_val)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_ROMC;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf),
									rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		*sum_val = (unsigned short)((rbuf[2] << 8) | rbuf[1]);
		SHGRIP_ERR("Ack Err ack[%02X], checksum[%04X]\n", rbuf[0], *sum_val);
		return -EFAULT;
	}
	
	*sum_val = (unsigned short)((rbuf[2] << 8) | rbuf[1]);
	SHGRIP_INFO("Ack [%02X], checksum[%04X]\n", rbuf[0], *sum_val);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_lver                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_lver(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_LVER;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	ctrl->ver_info.lver0 = ((rbuf[2] << 8) | rbuf[1]);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_lver1                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_lver1(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_LVER1;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	ctrl->ver_info.lver1 = ((rbuf[2] << 8) | rbuf[1]);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_gswr1                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_gswr1(struct shgrip_drv *ctrl, 
								struct shgrip_params_val *val)
{
	int ret;
	unsigned char wbuf[16], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_GSWR1;
	wbuf[1] = val->dcount_val.high_val;
	wbuf[2] = val->dcount_val.low_val;
	wbuf[3] = val->nref_val.high_val;
	wbuf[4] = val->nref_val.low_val;
	wbuf[5] = val->prm_dci_val.high_val;
	wbuf[6] = val->prm_dci_val.low_val;
	wbuf[7] = val->prm_acd_val.high_val;
	wbuf[8] = val->prm_acd_val.low_val;
	wbuf[9] = val->df_scs_val.val;
	wbuf[10] = val->nhys_val.high_val;
	wbuf[11] = val->nhys_val.low_val;
	wbuf[12] = val->prm_msa_val.high_val;
	wbuf[13] = val->prm_msa_val.low_val;
	wbuf[14] = val->nthr_val.high_val;
	wbuf[15] = val->nthr_val.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_gswr2                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_gswr2(struct shgrip_drv *ctrl,
								struct shgrip_params_val *val)
{
	int ret;
	unsigned char wbuf[16], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_GSWR2;
	wbuf[1] = val->dcount_val.high_val;
	wbuf[2] = val->dcount_val.low_val;
	wbuf[3] = val->nref_val.high_val;
	wbuf[4] = val->nref_val.low_val;
	wbuf[5] = val->prm_dci_val.high_val;
	wbuf[6] = val->prm_dci_val.low_val;
	wbuf[7] = val->prm_acd_val.high_val;
	wbuf[8] = val->prm_acd_val.low_val;
	wbuf[9] = val->df_scs_val.val;
	wbuf[10] = val->nhys_val.high_val;
	wbuf[11] = val->nhys_val.low_val;
	wbuf[12] = val->prm_msa_val.high_val;
	wbuf[13] = val->prm_msa_val.low_val;
	wbuf[14] = val->nthr_val.high_val;
	wbuf[15] = val->nthr_val.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_gsrd1                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_gsrd1(struct shgrip_drv *ctrl,
								struct shgrip_params_val *val)
{
	int ret;
	int i;
	unsigned char wbuf[1], rbuf[16];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_GSRD1;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	val->dcount_val.high_val	= rbuf[1];
	val->dcount_val.low_val		= rbuf[2];
	val->nref_val.high_val		= rbuf[3];
	val->nref_val.low_val		= rbuf[4];
	val->prm_dci_val.high_val	= rbuf[5];
	val->prm_dci_val.low_val	= rbuf[6];
	val->prm_acd_val.high_val	= rbuf[7];
	val->prm_acd_val.low_val	= rbuf[8];
	val->df_scs_val.val			= rbuf[9];
	val->nhys_val.high_val		= rbuf[10];
	val->nhys_val.low_val		= rbuf[11];
	val->prm_msa_val.high_val	= rbuf[12];
	val->prm_msa_val.low_val	= rbuf[13];
	val->nthr_val.high_val		= rbuf[14];
	val->nthr_val.low_val		= rbuf[15];
	
	for (i = 0; i < sizeof(rbuf); i++) {
		SHGRIP_INFO("rbuf[%d]:0x%02X\n", i, rbuf[i]);
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_gsrd2                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_gsrd2(struct shgrip_drv *ctrl,
								struct shgrip_params_val *val)
{
	int ret;
	int i;
	unsigned char wbuf[1], rbuf[16];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_GSRD2;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	val->dcount_val.high_val	= rbuf[1];
	val->dcount_val.low_val		= rbuf[2];
	val->nref_val.high_val		= rbuf[3];
	val->nref_val.low_val		= rbuf[4];
	val->prm_dci_val.high_val	= rbuf[5];
	val->prm_dci_val.low_val	= rbuf[6];
	val->prm_acd_val.high_val	= rbuf[7];
	val->prm_acd_val.low_val	= rbuf[8];
	val->df_scs_val.val			= rbuf[9];
	val->nhys_val.high_val		= rbuf[10];
	val->nhys_val.low_val		= rbuf[11];
	val->prm_msa_val.high_val	= rbuf[12];
	val->prm_msa_val.low_val	= rbuf[13];
	val->nthr_val.high_val		= rbuf[14];
	val->nthr_val.low_val		= rbuf[15];
	
	for (i = 0; i < sizeof(rbuf); i++) {
		SHGRIP_INFO("rbuf[%d]:0x%02X\n", i, rbuf[i]);
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_gstclr                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_command_gstclr(struct shgrip_drv *ctrl, 
							unsigned char *grip, unsigned char *testch)
{
	int ret = 0;
	unsigned char wbuf[1], rbuf[3];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_GSTCLR;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("GSTCLR BAD Value %02x %02x %02x\n",
						rbuf[0], rbuf[1], rbuf[2]);
		return -EFAULT;
	}
	
	if (rbuf[1] & BIT3) {
		if (rbuf[1] & (BIT0 | BIT1)) {
			SHGRIP_ERR("THR3_ERR, THR4_ERR OR THR5_ERR, AND PCOVR_ERR READ_STATE:0x%02x\n", rbuf[1]);
		}else{
			SHGRIP_ERR("PCOVR_ERR READ_STATE:0x%02x\n", rbuf[1]);
		}
	}
	
	if (rbuf[2] & (BIT2 | BIT3)) {
		SHGRIP_ERR("THR3_ERR OR THR5_ERR READ_STATE:0x%02x\n", rbuf[2]);
	}
	
	if (rbuf[2] & BIT4) {
		SHGRIP_ERR("SCMSA_ERR READ_STATE:0x%02x\n", rbuf[2]);
	}
	
	if (rbuf[2] & (BIT5 | BIT6)) {
		SHGRIP_ERR("THR4_ERR READ_STATE:0x%02x\n", rbuf[2]);
	}
	
	SHGRIP_INFO("GSTCLR:%02x %02x %02x\n",
						rbuf[0], rbuf[1], rbuf[2]);
	
	*grip	= rbuf[1];
	*testch	= (rbuf[2] & (BIT1 | BIT0));
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_chprd                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_chprd(struct shgrip_drv *ctrl,
								struct shgrip_get_level *level)
{
	int ret;
	unsigned char wbuf[1], rbuf[13];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_CHPRD;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	level->ch0.dcount_val.high_val		= rbuf[1];
	level->ch0.dcount_val.low_val		= rbuf[2];
	level->ch0.nref_val.high_val		= rbuf[3];
	level->ch0.nref_val.low_val			= rbuf[4];
	level->ch0.nthr_val.high_val		= rbuf[5];
	level->ch0.nthr_val.low_val			= rbuf[6];
	
	level->ch2.dcount_val.high_val		= rbuf[7];
	level->ch2.dcount_val.low_val		= rbuf[8];
	level->ch2.nref_val.high_val		= rbuf[9];
	level->ch2.nref_val.low_val			= rbuf[10];
	level->ch2.nthr_val.high_val		= rbuf[11];
	level->ch2.nthr_val.low_val			= rbuf[12];
	
	SHGRIP_INFO("CH0 dcount:0x%04X, nref:0x%04X, nthr:0x%04X\n",
		((level->ch0.dcount_val.high_val << 8) | level->ch0.dcount_val.low_val),
		((level->ch0.nref_val.high_val   << 8) | level->ch0.nref_val.low_val),
		((level->ch0.nthr_val.high_val   << 8) | level->ch0.nthr_val.low_val));

	SHGRIP_INFO("CH2 dcount:0x%04X, nref:0x%04X, nthr:0x%04X\n",
		((level->ch2.dcount_val.high_val << 8) | level->ch2.dcount_val.low_val),
		((level->ch2.nref_val.high_val   << 8) | level->ch2.nref_val.low_val),
		((level->ch2.nthr_val.high_val   << 8) | level->ch2.nthr_val.low_val));
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_tson                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_tson(struct shgrip_drv *ctrl)
{
	int ret;
	unsigned char wbuf[1], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TSON;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_tsoff                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_tsoff(struct shgrip_drv *ctrl)
{
	int ret;
	unsigned char wbuf[1], rbuf[1];

	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TSOFF;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}


/* ------------------------------------------------------------------------- */
/* shgrip_command_tsmp                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_command_tsmp(struct shgrip_drv *ctrl, struct shgrip_sampling_val *smp)
{
	int ret;
	unsigned char wbuf[5], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TSMP;
	wbuf[1] = smp->sampling_a;
	wbuf[2] = smp->sampling_b;
	wbuf[3] = smp->sampling_c;
	wbuf[4] = smp->sampling_d;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return ret;
}


/* ------------------------------------------------------------------------- */
/* shgrip_command_th3on                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_th3on(struct shgrip_drv *ctrl, unsigned char th3onset)
{
	int ret;
	unsigned char wbuf[2], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TH3ON;
	wbuf[1] = th3onset;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_setth3val                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_command_setth3val(struct shgrip_drv *ctrl,
									struct shgrip_threshold3_reg *reg)
{
	int ret;
	unsigned char wbuf[9], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_SETTH3VAL;
	wbuf[1] = reg->nhys3_ch0.high_val;
	wbuf[2] = reg->nhys3_ch0.low_val;
	wbuf[3] = reg->nhys3_ch2.high_val;
	wbuf[4] = reg->nhys3_ch2.low_val;
	wbuf[5] = reg->nthr3_ch0.high_val;
	wbuf[6] = reg->nthr3_ch0.low_val;
	wbuf[7] = reg->nthr3_ch2.high_val;
	wbuf[8] = reg->nthr3_ch2.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_driftset                                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_command_driftset(struct shgrip_drv *ctrl, struct shgrip_drift_set *settings)
{
	int i;
	int ret;
	unsigned char wbuf[14], rbuf[9];
	
	SHGRIP_DBG("start\n");

	wbuf[0]  = SHGRIP_CMD_DRIFTSET;
	wbuf[1]  = settings->drift_on_ch;
	wbuf[2]  = settings->val.thr_val_ch0.high_val;
	wbuf[3]  = settings->val.thr_val_ch0.low_val;
	wbuf[4]  = settings->val.thr_val_ch2.high_val;
	wbuf[5]  = settings->val.thr_val_ch2.low_val;
	wbuf[6]  = settings->val.scancnt_ch0.high_val;
	wbuf[7]  = settings->val.scancnt_ch0.low_val;
	wbuf[8]  = settings->val.dcount_ch0.high_val;
	wbuf[9]  = settings->val.dcount_ch0.low_val;
	wbuf[10] = settings->val.scancnt_ch2.high_val;
	wbuf[11] = settings->val.scancnt_ch2.low_val;
	wbuf[12] = settings->val.dcount_ch2.high_val;
	wbuf[13] = settings->val.dcount_ch2.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		for( i = 1; i < sizeof(rbuf); i++){
			SHGRIP_ERR("Ack Err [%02X]\n", rbuf[i]);
		}
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_chprd                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_chprd2(struct shgrip_drv *ctrl, struct shgrip_chprd2 *val)
{
	int ret;
	unsigned char wbuf[1], rbuf[21];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_CHPRD2;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	val->dcount_ch0.high_val	= rbuf[1];
	val->dcount_ch0.low_val		= rbuf[2];
	val->nref_ch0.high_val		= rbuf[3];
	val->nref_ch0.low_val		= rbuf[4];
	val->nthr_ch0.high_val		= rbuf[5];
	val->nthr_ch0.low_val		= rbuf[6];
	val->dcount_ch2.high_val	= rbuf[7];
	val->dcount_ch2.low_val		= rbuf[8];
	val->nref_ch2.high_val		= rbuf[9];
	val->nref_ch2.low_val		= rbuf[10];
	val->nthr_ch2.high_val		= rbuf[11];
	val->nthr_ch2.low_val		= rbuf[12];
	val->ncount_ch0.high_val	= rbuf[13];
	val->ncount_ch0.low_val		= rbuf[14];
	val->scudata_ch0.high_val	= rbuf[15];
	val->scudata_ch0.low_val	= rbuf[16];
	val->ncount_ch2.high_val	= rbuf[17];
	val->ncount_ch2.low_val		= rbuf[18];
	val->scudata_ch2.high_val	= rbuf[19];
	val->scudata_ch2.low_val	= rbuf[20];
	
	SHGRIP_INFO("CH0 dcount:0x%04X, nref:0x%04X, nthr:0x%04X, ncount:0x%04X, scudata:0x%04X\n",
		((val->dcount_ch0.high_val  << 8) | val->dcount_ch0.low_val),
		((val->nref_ch0.high_val    << 8) | val->nref_ch0.low_val),
		((val->nthr_ch0.high_val    << 8) | val->nthr_ch0.low_val),
		((val->ncount_ch0.high_val  << 8) | val->ncount_ch0.low_val),
		((val->scudata_ch0.high_val << 8) | val->scudata_ch0.low_val));

	SHGRIP_INFO("CH2 dcount:0x%04X, nref:0x%04X, nthr:0x%04X, ncount:0x%04X, scudata:0x%04X\n",
		((val->dcount_ch2.high_val  << 8) | val->dcount_ch2.low_val),
		((val->nref_ch2.high_val    << 8) | val->nref_ch2.low_val),
		((val->nthr_ch2.high_val    << 8) | val->nthr_ch2.low_val),
		((val->ncount_ch2.high_val  << 8) | val->ncount_ch2.low_val),
		((val->scudata_ch2.high_val << 8) | val->scudata_ch2.low_val));
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_debug_rw_command                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_command_debug_rw_command(struct shgrip_drv *ctrl, 
											struct shgrip_dbg_command *cmd)
{
	int i;
	int ret;
	unsigned char wbuf[32+1], rbuf[32+1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = (unsigned char)(cmd->addr);
	
	if (cmd->w_size) {
		for (i = 0; i < cmd->w_size; i++) {
			wbuf[i+1] = cmd->w_buf[i];
		}
	}
	
	ret = shgrip_spi_transfer(ctrl, wbuf, (cmd->w_size + 1), rbuf, (cmd->r_size + 1));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	if (cmd->r_size) {
		for (i = 0; i < cmd->r_size; i++) {
			cmd->r_buf[i] = rbuf[i+1];
			SHGRIP_INFO("rbuf[%d]:0x%02X\n", (i + 1), cmd->r_buf[i]);
		}
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_thr3_cancelset                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_command_thr3_cancelset(struct shgrip_drv *ctrl, 
											unsigned char thr3cancel_on)
{
	int ret;
	unsigned char wbuf[2], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_THR3_CANCELSET;
	wbuf[1] = (thr3cancel_on & (BIT1 | BIT0));
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_thr3_cancelval                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_command_thr3_cancelval(struct shgrip_drv *ctrl, 
										struct shgrip_thr3_cancelval *val)
{
	int ret;
	unsigned char wbuf[7], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_THR3_CANCELVAL;
	wbuf[1] = val->scancntn_ch0.high_val;
	wbuf[2] = val->scancntn_ch0.low_val;
	wbuf[3] = val->scancntm_ch0;
	wbuf[4] = val->scancntn_ch2.high_val;
	wbuf[5] = val->scancntn_ch2.low_val;
	wbuf[6] = val->scancntm_ch2;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

#if 0
/* ------------------------------------------------------------------------- */
/* shgrip_command_uphoseiset                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_command_uphoseiset(struct shgrip_drv *ctrl, 
										struct shgrip_uphosei *val)
{
	int ret;
	unsigned char wbuf[18], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_UPHOSEISET;
	wbuf[1]  = val->hosei_onoff;
	wbuf[2]  = val->patern;
	wbuf[3]  = val->q_dtimes_ch0;
	wbuf[4]  = val->q_dtimes_ch2;
	wbuf[5]  = val->q_delta_ch0;
	wbuf[6]  = val->q_delta_ch2;
	wbuf[7]  = val->q_dfilter;
	wbuf[8]  = val->s_dtimes_ch0.high_val;
	wbuf[9]  = val->s_dtimes_ch0.low_val;
	wbuf[10] = val->s_dtimes_ch2.high_val;
	wbuf[11] = val->s_dtimes_ch2.low_val;
	wbuf[12] = val->s_delta_ch0.high_val;
	wbuf[13] = val->s_delta_ch0.low_val;
	wbuf[14] = val->s_delta_ch2.high_val;
	wbuf[15] = val->s_delta_ch2.low_val;
	wbuf[16] = val->s_dfilter.high_val;
	wbuf[17] = val->s_dfilter.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}
#endif

/* ------------------------------------------------------------------------- */
/* shgrip_command_driftset2                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_command_driftset2(struct shgrip_drv *ctrl, struct shgrip_drift_set2 *settings)
{
	int i;
	int ret;
	unsigned char wbuf[6], rbuf[5];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_DRIFTSET2;
	wbuf[1]  = (settings->drift_on_ch & (BIT1 | BIT0));
	wbuf[2]  = settings->scancnt_ch0;
	wbuf[3]  = settings->dcount_ch0;
	wbuf[4]  = settings->scancnt_ch2;
	wbuf[5]  = settings->dcount_ch2;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		for( i = 1; i < sizeof(rbuf); i++){
			SHGRIP_ERR("Ack Err [%02X]\n", rbuf[i]);
		}
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_scmsaset                                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_command_scmsaset(struct shgrip_drv *ctrl, struct shgrip_scmsa *val)
{
	int ret;
	unsigned char wbuf[17], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_SCMSASET;
	wbuf[1]  = val->scmsa_on;
	wbuf[2]  = val->triga_on;
	wbuf[3]  = val->msa.high_val;
	wbuf[4]  = val->msa.low_val;
	wbuf[5]  = val->ch0.a;
	wbuf[6]  = val->ch0.b;
	wbuf[7]  = val->ch0.x.high_val;
	wbuf[8]  = val->ch0.x.low_val;
	wbuf[9]  = val->ch0.y;
	wbuf[10] = val->ch0.z;
	wbuf[11] = val->ch2.a;
	wbuf[12] = val->ch2.b;
	wbuf[13] = val->ch2.x.high_val;
	wbuf[14] = val->ch2.x.low_val;
	wbuf[15] = val->ch2.y;
	wbuf[16] = val->ch2.z;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_scmsaset2                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_command_scmsaset2(struct shgrip_drv *ctrl, struct shgrip_scmsa2 *val)
{
	int ret;
	unsigned char wbuf[5], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0]  = SHGRIP_CMD_SCMSASET2;
	wbuf[1]  = val->count_a.high_val;
	wbuf[2]  = val->count_a.low_val;
	wbuf[3]  = val->count_t.high_val;
	wbuf[4]  = val->count_t.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_th4on                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_th4on(struct shgrip_drv *ctrl,
									struct shgrip_threshold4_reg *reg)
{
	int ret;
	unsigned char wbuf[10], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TH4ON;
	wbuf[1] = reg->th4_on;
	wbuf[2] = reg->nhys4_ch0.high_val;
	wbuf[3] = reg->nhys4_ch0.low_val;
	wbuf[4] = reg->nhys4_ch2.high_val;
	wbuf[5] = reg->nhys4_ch2.low_val;
	wbuf[6] = reg->nthr4_ch0.high_val;
	wbuf[7] = reg->nthr4_ch0.low_val;
	wbuf[8] = reg->nthr4_ch2.high_val;
	wbuf[9] = reg->nthr4_ch2.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_th5on                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_command_th5on(struct shgrip_drv *ctrl,
									struct shgrip_threshold5_reg *reg)
{
	int ret;
	unsigned char wbuf[15], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TH5ON;
	wbuf[1] = reg->thr5onset;
	wbuf[2] = reg->thr5cansel_on;
	wbuf[3] = reg->nhys5_ch0.high_val;
	wbuf[4] = reg->nhys5_ch0.low_val;
	wbuf[5] = reg->nhys5_ch2.high_val;
	wbuf[6] = reg->nhys5_ch2.low_val;
	wbuf[7] = reg->nthr5_ch0.high_val;
	wbuf[8] = reg->nthr5_ch0.low_val;
	wbuf[9] = reg->nthr5_ch2.high_val;
	wbuf[10] = reg->nthr5_ch2.low_val;
	wbuf[11] = reg->th5_s_cnt.high_val;
	wbuf[12] = reg->th5_s_cnt.low_val;
	wbuf[13] = reg->th5_t_cnt.high_val;
	wbuf[14] = reg->th5_t_cnt.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_pcovrcon                                                   */
/* ------------------------------------------------------------------------- */
static int shgrip_command_pcovrcon(struct shgrip_drv *ctrl,
									struct shgrip_pcovrc_reg *reg)
{
	int ret;
	unsigned char wbuf[5], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_PCOVRCON;
	wbuf[1] = reg->pcovrconset;
	wbuf[2] = reg->pcovrcint_on;
	wbuf[3] = reg->pcovrc_r_cnt.high_val;
	wbuf[4] = reg->pcovrc_r_cnt.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_uphoseiset2                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_command_uphoseiset2(struct shgrip_drv *ctrl, 
										struct shgrip_uphoseiset2 *val)
{
	int ret;
	unsigned char wbuf[14], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_UPHOSEISET2;
	wbuf[1] = val->hosei_onoff2;
	wbuf[2] = val->dtimes_1st.ch0;
	wbuf[3] = val->dtimes_1st.ch2;
	wbuf[4] = val->w_q_1st_delta_ch0;
	wbuf[5] = val->w_q_1st_delta_ch2;
	wbuf[6] = val->dtimes_2nd.ch0;
	wbuf[7] = val->dtimes_2nd.ch2;
	wbuf[8] = val->delta_ch0.high_val;
	wbuf[9] = val->delta_ch0.low_val;
	wbuf[10] = val->delta_ch2.high_val;
	wbuf[11] = val->delta_ch2.low_val;
	wbuf[12] = val->w_q_2nd_delay_ch0;
	wbuf[13] = val->w_q_2nd_delay_ch2;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_uphoseiset3                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_command_uphoseiset3(struct shgrip_drv *ctrl, 
										struct shgrip_uphoseiset3 *val)
{
	int ret;
	unsigned char wbuf[14], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_UPHOSEISET3;
	wbuf[1] = val->hosei_onoff3;
	wbuf[2] = val->dtimes_1st.ch0;
	wbuf[3] = val->dtimes_1st.ch2;
	wbuf[4] = val->w_s_1st_delta_ch0;
	wbuf[5] = val->w_s_1st_delta_ch2;
	wbuf[6] = val->dtimes_2nd.ch0;
	wbuf[7] = val->dtimes_2nd.ch2;
	wbuf[8] = val->delta_ch0.high_val;
	wbuf[9] = val->delta_ch0.low_val;
	wbuf[10] = val->delta_ch2.high_val;
	wbuf[11] = val->delta_ch2.low_val;
	wbuf[12] = val->w_s_2nd_delay_ch0;
	wbuf[13] = val->w_s_2nd_delay_ch2;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_scmsa2set                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_command_scmsa2set(struct shgrip_drv *ctrl, 
										struct shgrip_th_hys_6 *val)
{
	int ret;
	unsigned char wbuf[16], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_SCMSA2SET;
	wbuf[1] = val->scmsa2_on;
	wbuf[2] = val->nhys6_0.high_val;
	wbuf[3] = val->nhys6_0.low_val;
	wbuf[4] = val->nhys6_2.high_val;
	wbuf[5] = val->nhys6_2.low_val;
	wbuf[6] = val->nthr6_0.high_val;
	wbuf[7] = val->nthr6_0.low_val;
	wbuf[8] = val->nthr6_2.high_val;
	wbuf[9] = val->nthr6_2.low_val;
	wbuf[10] = val->th6_x_0;
	wbuf[11] = val->th6_x_2;
	wbuf[12] = val->hys6_y_0;
	wbuf[13] = val->hys6_y_2;
	wbuf[14] = val->ch0_alpha;
	wbuf[15] = val->ch2_alpha;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_scmsa2clr                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_command_scmsa2clr(struct shgrip_drv *ctrl, unsigned char scmsa2_off)
{
	int ret;
	unsigned char wbuf[2], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_SCMSA2CLR;
	wbuf[1] = (scmsa2_off & BIT0);
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

#if 0
/* ------------------------------------------------------------------------- */
/* shgrip_command_scmsa2valrd                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_command_scmsa2valrd(struct shgrip_drv *ctrl)
{
	int ret, i;
	unsigned char wbuf[1], rbuf[21];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_SCMSA2VALRD;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	for (i = 1; i < sizeof(rbuf); i++) {
		SHGRIP_INFO("param[%d]:0x%02x\n", i, rbuf[i]);
	}
	
	return 0;
}
#endif

/* ------------------------------------------------------------------------- */
/* shgrip_command_suphoseiset                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_command_suphoseiset(struct shgrip_drv *ctrl, 
										struct shgrip_suphosei *val)
{
	int ret;
	unsigned char wbuf[19], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_SUPHOSEISET;
	wbuf[1] = val->s_hosei_onoff;
	wbuf[2] = val->la_mode;
	wbuf[3] = val->hosei_trig;
	wbuf[4] = val->jla_mode;
	wbuf[5] = val->la_tson_ch0;
	wbuf[6] = val->la_tson_ch2;
	wbuf[7] = val->la_th3_ch0;
	wbuf[8] = val->la_th3_ch2;
	wbuf[9] = val->la_th4_ch0;
	wbuf[10] = val->la_th4_ch2;
	wbuf[11] = val->la_th5_ch0;
	wbuf[12] = val->la_th5_ch2;
	wbuf[13] = val->la_man_ch0;
	wbuf[14] = val->la_man_ch2;
	wbuf[15] = val->dtimes_ch0.high_val;
	wbuf[16] = val->dtimes_ch0.low_val;
	wbuf[17] = val->dtimes_ch2.high_val;
	wbuf[18] = val->dtimes_ch2.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_suphoseiset2                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_command_suphoseiset2(struct shgrip_drv *ctrl, 
										struct shgrip_suphosei2 *val)
{
	int ret;
	unsigned char wbuf[21], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_SUPHOSEISET2;
	wbuf[1] = val->jla_tson_ch0.high_val;
	wbuf[2] = val->jla_tson_ch0.low_val;
	wbuf[3] = val->jla_tson_ch2.high_val;
	wbuf[4] = val->jla_tson_ch2.low_val;
	wbuf[5] = val->jla_th3_ch0.high_val;
	wbuf[6] = val->jla_th3_ch0.low_val;
	wbuf[7] = val->jla_th3_ch2.high_val;
	wbuf[8] = val->jla_th3_ch2.low_val;
	wbuf[9] = val->jla_th4_ch0.high_val;
	wbuf[10] = val->jla_th4_ch0.low_val;
	wbuf[11] = val->jla_th4_ch2.high_val;
	wbuf[12] = val->jla_th4_ch2.low_val;
	wbuf[13] = val->jla_th5_ch0.high_val;
	wbuf[14] = val->jla_th5_ch0.low_val;
	wbuf[15] = val->jla_th5_ch2.high_val;
	wbuf[16] = val->jla_th5_ch2.low_val;
	wbuf[17] = val->la_abs_ch0.high_val;
	wbuf[18] = val->la_abs_ch0.low_val;
	wbuf[19] = val->la_abs_ch2.high_val;
	wbuf[20] = val->la_abs_ch2.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_scuplimitset                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_command_scuplimitset(struct shgrip_drv *ctrl, 
										struct shgrip_scuplimit *val)
{
	int ret;
	unsigned char wbuf[6], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_SCUPLIMITSET;
	wbuf[1] = val->sc_uplimit_onoff;
	wbuf[2] = val->limit_ch0.high_val;
	wbuf[3] = val->limit_ch0.low_val;
	wbuf[4] = val->limit_ch2.high_val;
	wbuf[5] = val->limit_ch2.low_val;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_command_th1don                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_command_th1don(struct shgrip_drv *ctrl, 
									struct shgrip_th1d *val)
{
	int ret;
	unsigned char wbuf[10], rbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_CMD_TH1DON;
	wbuf[1] = val->th1d_on;
	wbuf[2] = val->nhys1d_0;
	wbuf[3] = val->nhys1d_2;
	wbuf[4] = val->nthr1d_0;
	wbuf[5] = val->nthr1d_2;
	wbuf[6] = val->judgeval_0;
	wbuf[7] = val->judgeval_2;
	wbuf[8] = val->set_scancnt_0;
	wbuf[9] = val->set_scancnt_2;
	
	ret = shgrip_spi_transfer(ctrl, wbuf, sizeof(wbuf), rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	if (rbuf[0] != SHGRIP_CMD_ACK) {
		SHGRIP_ERR("Ack Err [%02X]\n", rbuf[0]);
		return -EFAULT;
	}
	
	return 0;
}


/* ========================================================================= */
/* SHGRIP Device Control Function Loader Mode                                */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_ldr_read_status                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_read_status(struct shgrip_drv *ctrl, 
							unsigned char *srd0, unsigned char *srd1)
{
	int i;
	int ret = 0;
	unsigned char wbuf[1], rbuf[2];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_READ_STS_REG;
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf),
												rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_CTS_TIMEOUT_US;
	while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
		if (!(i-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
			SHGRIP_ERR("read status is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
	};
	
	*srd0 = rbuf[0];
	*srd1 = rbuf[1];
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_clr_status                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_clr_status(struct shgrip_drv *ctrl)
{
	int i;
	int ret = 0;
	unsigned char wbuf[1];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_CLEAR_STS_REG;
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf),
												NULL, 0);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_CLRSTS_TIMEOUT_US;
	while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
		if (!(i-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
			SHGRIP_ERR("read status is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
	};
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_read_verify                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_read_verify(struct shgrip_drv *ctrl, unsigned long addr)
{
	int i;
    int ret = 0;
	unsigned char wbuf[3], rbuf[SHGRIP_LDR_PAGE_SIZE];
	unsigned long *img_ptr, *read_ptr;
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_PAGE_READ;
	wbuf[1] = (unsigned char)((addr >> 8) & 0xFF);
	wbuf[2] = (unsigned char)((addr >> 16) & 0xFF);
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf),
												rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_PGREAD_TIMEOUT_US;
	while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
		if (!(i-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
			SHGRIP_ERR("page read is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
	};
	
	i = SHGRIP_LDR_PAGE_SIZE;
	img_ptr  = (unsigned long*)&ctrl->fw_data[addr - SHGRIP_LDR_ADDR_APP_BLK1];
	read_ptr = (unsigned long*)rbuf;
	while (i-=4) {
		if (*img_ptr != *read_ptr) {
			SHGRIP_ERR("verify err  addr[%08lX], img[%08lX], reead[%08lX]\n", addr, *img_ptr, *read_ptr);
			return -EFAULT;
		}
		img_ptr++;
		read_ptr++;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_readverify_fw                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_readverify_fw(struct shgrip_drv *ctrl)
{
	unsigned long dl_read_ptr;
	int ret;
	
	SHGRIP_DBG("start\n");
	
	dl_read_ptr = shgrip_block_addr_table[ctrl->loader_mode][0];
	while (dl_read_ptr < shgrip_block_tenminate[ctrl->loader_mode]) {
		ret = shgrip_ldr_read_verify(ctrl, dl_read_ptr);
		if (!ret) {
			dl_read_ptr += SHGRIP_LDR_PAGE_SIZE;
		} else {
			SHGRIP_ERR("Err verify ret[%d], addr[%08lX]\n", ret, dl_read_ptr);
			return ret;
		}
	}
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_checkID                                                        */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_checkID(struct shgrip_drv *ctrl)
{
	int i;
	unsigned char wbuf[8], srd0, srd1;
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_ID_CHECK;
	wbuf[1] = SHGRIP_LDR_FW_ID1;
	wbuf[2] = SHGRIP_LDR_FW_ID2;
	wbuf[3] = SHGRIP_LDR_FW_ID3;
	wbuf[4] = SHGRIP_LDR_FW_ID4;
	wbuf[5] = SHGRIP_LDR_FW_ID5;
	wbuf[6] = SHGRIP_LDR_FW_ID6;
	wbuf[7] = SHGRIP_LDR_FW_ID7;
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf), NULL, 0);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_CHKID_TIMEOUT_US;
	while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
		if (!(i-=SHGRIP_POLL_WAIT_US)) {
			SHGRIP_ERR("Check ID is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US);
	};
	
	ret = shgrip_ldr_read_status(ctrl, &srd0, &srd1);
	if (ret) {
		SHGRIP_ERR("read status err. ret=%d\n", ret);
		return ret;
	}
	
	if ((srd0 & SHGRIP_LDR_SRD_SEQ) == 0) {
		SHGRIP_ERR("Busy srd0[%02X] srd1[%02X]\n", srd0, srd1);
		/* T.B.D:  retry or error */
		return -EBUSY;
	}
	
	if ((srd1 & SHGRIP_LDR_SRD_ID_CHK) != SHGRIP_LDR_SRD_ID_CHK ) {
		SHGRIP_ERR("ID err srd0[%02X] srd1[%02X]\n", srd0, srd1);
		/* T.B.D:  retry or error */
		return -EINVAL;
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_erase_blodk                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_erase_blodk(struct shgrip_drv *ctrl)
{
#ifdef SHGRIP_FW_DATA_NOT_WRITE
	return 0;
#else	
	int i;
	int ret = 0, block = 0;
	unsigned char wbuf[4], srd0, srd1;
	unsigned long addr;
	
	SHGRIP_DBG("start\n");
	
	addr = shgrip_block_addr_table[ctrl->loader_mode][block];
	
	while (addr != SHGRIP_LDR_BLK_ADDR_TBL_END) {
		ret = shgrip_ldr_clr_status(ctrl);
		if (ret) {
			SHGRIP_ERR("clear status err. ret=%d\n", ret);
			return ret;
		}
		
		wbuf[0] = SHGRIP_LDR_CMD_BLOCK_ERASE;
		wbuf[1] = (unsigned char)((addr >> 8) & 0xFF);
		wbuf[2] = (unsigned char)((addr >> 16) & 0xFF);
		wbuf[3] = SHGRIP_LDR_ERASE_DO;
		
		ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf),
													NULL, 0);
		if (ret) {
			SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
			return ret;
		}
		
		i = SHGRIP_ERASE_TIMEOUT_US;
		while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
			if (!(i-=SHGRIP_ERACE_POLL_WAIT_US)) {
				SHGRIP_ERR("Erase is timeout\n");
				return -ETIMEDOUT;
			}
			shgrip_sys_delay_us(SHGRIP_ERACE_POLL_WAIT_US);
		};
		
		ret = shgrip_ldr_read_status(ctrl, &srd0, &srd1);
		if (ret) {
			SHGRIP_ERR("read status err. ret=%d\n", ret);
			return ret;
		}
		
		if ((srd0 & SHGRIP_LDR_SRD_SEQ) == 0) {
			SHGRIP_ERR("Busy srd0[%02X] srd1[%02X]\n", srd0, srd1);
			/* T.B.D:  retry or error */
			return -EBUSY;
		}
		
		if ((srd0 & SHGRIP_LDR_SRD_ERZ) != 0) {
			SHGRIP_ERR("Erase err srd0[%02X] srd1[%02X]\n", srd0, srd1);
			/* T.B.D:  retry or error */
			return -EFAULT;
		}
		
		block++;
		addr = shgrip_block_addr_table[ctrl->loader_mode][block];
	}
	
	return 0;
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_write_page                                                     */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_write_page(struct shgrip_drv *ctrl, unsigned long addr)
{
#ifdef SHGRIP_FW_DATA_NOT_WRITE
	int ret = 0;
	unsigned char wbuf[3];
#else
	int i;
	int ret = 0;
	unsigned char wbuf[3], srd0, srd1;
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_ldr_clr_status(ctrl);
	if (ret) {
		SHGRIP_ERR("clear status err. ret=%d\n", ret);
		return ret;
	}
	
	wbuf[0] = SHGRIP_LDR_CMD_PAGE_PROGRAM;
	wbuf[1] = (unsigned char)((addr >> 8) & 0xFF);
	wbuf[2] = (unsigned char)((addr >> 16) & 0xFF);
	
	ret = shgrip_spi_transfer_for_loader_special_write(ctrl, &wbuf[0], sizeof(wbuf),
			&ctrl->fw_data[addr - SHGRIP_LDR_ADDR_APP_BLK1], SHGRIP_LDR_PAGE_SIZE);
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
#ifndef SHGRIP_FW_DATA_NOT_WRITE	
	i = SHGRIP_PGWRITE_TIMEOUT_US;
	while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
		if (!(i-=SHGRIP_PGWRITE_POLL_WAIT_US)) {
			SHGRIP_ERR("pagewrite is timeout\n");
			return-ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_PGWRITE_POLL_WAIT_US);
	};
	
	ret = shgrip_ldr_read_status(ctrl, &srd0, &srd1);
	if (ret) {
		SHGRIP_ERR("read status err. ret=%d\n", ret);
		return ret;
	}
	
	if ((srd0 & SHGRIP_LDR_SRD_SEQ) == 0) {
		SHGRIP_ERR("Busy srd0[%02X] srd1[%02X]\n", srd0, srd1);
		/* T.B.D:  retry or error */
		return -EBUSY;
	}
	
	if ((srd0 & SHGRIP_LDR_SRD_PRG) != 0) {
		SHGRIP_ERR("Write err srd0[%02X] srd1[%02X]\n", srd0, srd1);
		/* T.B.D:  retry or error */
		return -EFAULT;
	}
#endif /* SHGRIP_FW_DATA_NOT_WRITE */
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ldr_get_lver                                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_ldr_get_lver(struct shgrip_drv *ctrl)
{
	int i;
	int ret = 0;
	unsigned char wbuf[1], rbuf[7];
	
	SHGRIP_DBG("start\n");
	
	wbuf[0] = SHGRIP_LDR_CMD_VERSION_INFO;
	
	ret = shgrip_spi_transfer_for_loader(ctrl, wbuf, sizeof(wbuf), 
													rbuf, sizeof(rbuf));
	if (ret) {
		SHGRIP_ERR("spi transfer err. ret=%d\n", ret);
		return ret;
	}
	
	i = SHGRIP_CTS_TIMEOUT_US;
	while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
		if (!(i-=SHGRIP_POLL_WAIT_US_FOR_LOADER)) {
			SHGRIP_ERR("LVER is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US_FOR_LOADER);
	};
	
	ctrl->ver_info.lver0  = (unsigned short)((rbuf[0] << 8) | rbuf[1]);
	ctrl->ver_info.lver1  = (unsigned short)((rbuf[4] << 8) | rbuf[5]);
	ctrl->ver_info.pver   = (unsigned short)((rbuf[2] << 8) | rbuf[3]);
	ctrl->fw_mode_select  = rbuf[6];
	
	SHGRIP_INFO("lver lver0[%04X]  lver1[%04X]  pver[%04X]  select[%d]\n",
													ctrl->ver_info.lver0,
													ctrl->ver_info.lver1,
													ctrl->ver_info.pver,
													ctrl->fw_mode_select);
	
	return 0;
}

/* ========================================================================= */
/* SHGRIP Sequence Function                                                  */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_seq_reset_start_app                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_reset_start_app(struct shgrip_drv *ctrl)
{
	int ret;
	unsigned char grip = 0, testch = 0;
	int time = SHGRIP_RESET_TIMEOUT_US;
	int retry;
	
	SHGRIP_DBG("start\n");
	
	gpio_set_value(SHGRIP_GPIO_GRIP_POW, SHGRIP_GPIO_VAL_HI);
	
	shgrip_sys_delay_us(SHGRIP_RESET_START_WAIT_US);
	
	gpio_set_value(SHGRIP_GPIO_GRIP_RESET, SHGRIP_GPIO_VAL_LO);
	
	gpio_set_value(SHGRIP_GPIO_GRIP_COM_REQ, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_RESET_START_WAIT_US);
	
	gpio_set_value(SHGRIP_GPIO_GRIP_RESET, SHGRIP_GPIO_VAL_HI);
	
	shgrip_sys_delay_us(SHGRIP_STARTUP_WAIT);
	
	while (!(gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
		if (!(time-=SHGRIP_POLL_RESET_WAIT_US)) {
			SHGRIP_ERR("reset is timeout\n");
			break;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_RESET_WAIT_US);
	}
	
	SHGRIP_DBG("ctrl->state: STATE_SENSOR_OFF\n");
	ctrl->state = STATE_SENSOR_OFF;
	
	for (retry = 1; retry <= SHGRIP_GSTCLR_RETRY_COUNT; retry++) {
		ret = shgrip_command_gstclr(ctrl, &grip, &testch);
		if (ret) {
			SHGRIP_ERR("shgrip_command_gstclr failed, retry:%d\n", retry);
			if (retry == SHGRIP_GSTCLR_RETRY_COUNT) {
				SHGRIP_ERR("shgrip_command_gstclr retry_over\n");
				return ret;
			}
		} else {
			break;
		}
		shgrip_sys_delay_us(SHGRIP_SPI_SYNC_ERR_WAIT_US);
	}
	
#ifdef CONFIG_SHTERM
	shterm_k_set_info(SHTERM_INFO_GRIP, 1);
#endif
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_reset_start_loader                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_reset_start_loader(struct shgrip_drv *ctrl)
{
	int i;
	
	SHGRIP_DBG("start\n");
	
	gpio_set_value(SHGRIP_GPIO_GRIP_RESET, SHGRIP_GPIO_VAL_LO);
	
	gpio_set_value(SHGRIP_GPIO_GRIP_COM_REQ, SHGRIP_GPIO_VAL_HI);
	
	shgrip_sys_delay_us(SHGRIP_RESET_START_WAIT_US);
	
	gpio_set_value(SHGRIP_GPIO_GRIP_RESET, SHGRIP_GPIO_VAL_HI);
	
	i = SHGRIP_CHGMODE_TIMEOUT_US;
	while (!(gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
		if (!(i-=SHGRIP_POLL_WAIT_US)) {
			SHGRIP_ERR("changing loader mode is timeout\n");
			return -ETIMEDOUT;
		}
		udelay(SHGRIP_POLL_WAIT_US);
	};
	
	if (ctrl->loader_mode == MODE_LOADER0) {
		gpio_set_value(SHGRIP_GPIO_GRIP_COM_REQ, SHGRIP_GPIO_VAL_LO);
	}
	
	i = SHGRIP_READY_TIMEOUT_US;
	while ((gpio_get_value(SHGRIP_GPIO_GRIP_INT))) {
		if (!(i-=SHGRIP_POLL_WAIT_US)) {
			SHGRIP_ERR("ready loadermode is timeout\n");
			return -ETIMEDOUT;
		}
		shgrip_sys_delay_us(SHGRIP_POLL_WAIT_US);
	};
	
	SHGRIP_DBG("ctrl->state: STATE_FW_DL\n");
	ctrl->state = STATE_FW_DL;
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_reset_recovery                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_reset_recovery(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_command_tsoff(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_tsoff err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_tsmp(ctrl, &(ctrl->bk_adj.smp));
	if (ret) {
		SHGRIP_ERR("shgrip_command_tsmp err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_gswr1(ctrl, &(ctrl->bk_adj.ch0_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_gswr1 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_gswr2(ctrl, &(ctrl->bk_adj.ch2_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_gswr2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_driftset(ctrl, &(ctrl->bk_adj.drift_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_driftset err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_driftset2(ctrl, &(ctrl->bk_adj.drift2_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_driftset2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_setth3val(ctrl, &(ctrl->bk_adj.th3_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_setth3val err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_th3on(ctrl, ctrl->bk_adj.th3onset);
	if (ret) {
		SHGRIP_DBG("shgrip_command_th3on err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_th4on(ctrl, &(ctrl->bk_adj.th4_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_th4on err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_thr3_cancelval(ctrl, &(ctrl->bk_adj.thr3_cancelval));
	if (ret) {
		SHGRIP_DBG("shgrip_command_thr3_cancelval err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_thr3_cancelset(ctrl, ctrl->bk_adj.thr3cancel_on);
	if (ret) {
		SHGRIP_DBG("shgrip_command_thr3_cancelset err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_scmsaset(ctrl, &(ctrl->bk_adj.scmsa_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_scmsaset err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_scmsaset2(ctrl, &(ctrl->bk_adj.scmsa2_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_scmsaset2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_th5on(ctrl, &(ctrl->bk_adj.th5_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_th5on err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_pcovrcon(ctrl, &(ctrl->bk_adj.pcovrc_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_pcovrcon err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_uphoseiset2(ctrl, &(ctrl->bk_adj.uphosei2_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_uphoseiset2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_uphoseiset3(ctrl, &(ctrl->bk_adj.uphosei3_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_uphoseiset3 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_scmsa2clr(ctrl, ctrl->bk_adj.scmsa2clr);
	if (ret) {
		SHGRIP_DBG("shgrip_command_scmsa2clr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_scmsa2set(ctrl, &(ctrl->bk_adj.th_hys_6_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_scmsa2set err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_suphoseiset(ctrl, &(ctrl->bk_adj.suphosei_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_suphoseiset err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_suphoseiset2(ctrl, &(ctrl->bk_adj.suphosei2_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_suphoseiset2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_scuplimitset(ctrl, &(ctrl->bk_adj.scuplimit_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_scuplimitset err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_th1don(ctrl, &(ctrl->bk_adj.th1d_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_th1don err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_tson(ctrl);
	if (ret) {
		SHGRIP_DBG("shgrip_command_tson err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_grip_power_off                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_grip_power_off(struct shgrip_drv *ctrl)
{
	SHGRIP_DBG("start\n");
	
	gpio_set_value(SHGRIP_GPIO_GRIP_RESET, SHGRIP_GPIO_VAL_LO);
	
	shgrip_sys_delay_us(SHGRIP_PWR_OFF_WAIT_US);
	
	gpio_set_value(SHGRIP_GPIO_GRIP_POW, SHGRIP_GPIO_VAL_LO);
	
#ifdef CONFIG_SHTERM
	shterm_k_set_info(SHTERM_INFO_GRIP, 0);
#endif
	
	memset(&(ctrl->bk_adj.ch0_val), 0, sizeof(struct shgrip_params_val));
	memset(&(ctrl->bk_adj.ch2_val), 0, sizeof(struct shgrip_params_val));
	memset(&(ctrl->bk_adj.smp), 0, sizeof(struct shgrip_sampling_val));
	memset(&(ctrl->bk_adj.drift_val), 0, sizeof(struct shgrip_drift_set));
	memset(&(ctrl->bk_adj.drift2_val), 0, sizeof(struct shgrip_drift_set2));
	ctrl->bk_adj.th3onset = 0;
	memset(&(ctrl->bk_adj.th3_val), 0, sizeof(struct shgrip_threshold3_reg));
	ctrl->bk_adj.thr3cancel_on = 0;
	memset(&(ctrl->bk_adj.thr3_cancelval), 0, sizeof(struct shgrip_thr3_cancelval));
	memset(&(ctrl->bk_adj.scmsa_val), 0, sizeof(struct shgrip_scmsa));
	memset(&(ctrl->bk_adj.scmsa2_val), 0, sizeof(struct shgrip_scmsa2));
	memset(&(ctrl->bk_adj.th4_val), 0, sizeof(struct shgrip_threshold4_reg));
	memset(&(ctrl->bk_adj.th5_val), 0, sizeof(struct shgrip_threshold5_reg));
	memset(&(ctrl->bk_adj.pcovrc_val), 0, sizeof(struct shgrip_pcovrc_reg));
	memset(&(ctrl->bk_adj.uphosei2_val), 0, sizeof(struct shgrip_uphoseiset2));
	memset(&(ctrl->bk_adj.uphosei3_val), 0, sizeof(struct shgrip_uphoseiset3));
	ctrl->bk_adj.scmsa2clr = 0;
	memset(&(ctrl->bk_adj.th_hys_6_val), 0, sizeof(struct shgrip_th_hys_6));
	memset(&(ctrl->bk_adj.suphosei_val), 0, sizeof(struct shgrip_suphosei));
	memset(&(ctrl->bk_adj.suphosei2_val), 0, sizeof(struct shgrip_suphosei2));
	memset(&(ctrl->bk_adj.scuplimit_val), 0, sizeof(struct shgrip_scuplimit));
	memset(&(ctrl->bk_adj.th1d_val), 0, sizeof(struct shgrip_th1d));
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_grip_sensor_on                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_grip_sensor_on(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
#ifdef CONFIG_ANDROID_ENGINEERING
	unsigned long sec,nsec;
#endif /* CONFIG_ANDROID_ENGINEERING */
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_SENSOR_ON) {
		SHGRIP_WARN("state is STATE_SENSOR_ON\n");
		return GRIP_RESULT_SUCCESS;
	}
	
	ret = shgrip_sys_request_irq(ctrl);
	if (ret) {
		SHGRIP_ERR("request_irq err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_tson(ctrl);
	if (ret) {
		SHGRIP_DBG("shgrip_command_tson err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	SHGRIP_DBG("state: STATE_SENSOR_ON \n");
	ctrl->state = STATE_SENSOR_ON;
	
	SHGRIP_DBG("enable_irq(gpio_to_irq(SHGRIP_IRQ_GRIP_INT)) \n");
	shgrip_sys_enable_irq(ctrl);

#ifdef CONFIG_ANDROID_ENGINEERING
	if (shgrip_th_interval_ms) {
		if (th_msec >= 1000) {
			sec  = th_msec / 1000;
			nsec = (th_msec % 1000) * 1000 * 1000;
		} else {
			sec  = 0;
			nsec = th_msec * 1000 * 1000;
		}
		
		if (hrtimer_flg == 0) {
			hrtimer_start(&(ctrl->shgrip_threshold_timer), ktime_set(sec, nsec), HRTIMER_MODE_REL);
			hrtimer_flg = 1;
		}
	}
#endif /* CONFIG_ANDROID_ENGINEERING */
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_grip_sensor_off                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_grip_sensor_off(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_SENSOR_OFF) {
		SHGRIP_WARN("state is STATE_SENSOR_OFF\n");
		return GRIP_RESULT_SUCCESS;
	}
	
	ret = shgrip_command_tsoff(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_tsoff err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	SHGRIP_DBG("state: STATE_SENSOR_OFF \n");
	ctrl->state = STATE_SENSOR_OFF;
	
	SHGRIP_DBG("disable_irq(SHGRIP_IRQ_GRIP_INT) \n");
	shgrip_sys_disable_irq(ctrl);
	shgrip_sys_disable_irq_wake(ctrl);
	
	shgrip_sys_free_irq(ctrl);

#ifndef SHGRIP_FACTORY_MODE_ENABLE
	SHGRIP_DBG("msm_tps_set_grip_state called grip:0\n");
	msm_tps_set_grip_state(SHGRIP_OFF);
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_set_sensor_adjust                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_set_sensor_adjust(struct shgrip_drv *ctrl,
										struct shgrip_sens_setting_params *data)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	
	ret = shgrip_command_tsmp(ctrl, &(data->smp));
	if (ret) {
		SHGRIP_ERR("shgrip_command_tsmp err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_gswr1(ctrl, &(data->ch0_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_gswr1 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_gswr2(ctrl, &(data->ch2_val));
	if (ret) {
		SHGRIP_ERR("shgrip_command_gswr2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_driftset(ctrl, &(data->drift_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_driftset err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_driftset2(ctrl, &(data->drift2_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_driftset2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_setth3val(ctrl, &(data->th3_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_setth3val err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_th3on(ctrl, data->th3onset);
	if (ret) {
		SHGRIP_DBG("shgrip_command_th3on err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_th4on(ctrl, &(data->th4_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_th4on err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_thr3_cancelval(ctrl, &(data->thr3_cancelval));
	if (ret) {
		SHGRIP_DBG("shgrip_command_thr3_cancelval err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_thr3_cancelset(ctrl, data->thr3cancel_on);
	if (ret) {
		SHGRIP_DBG("shgrip_command_thr3_cancelset err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_scmsaset(ctrl, &(data->scmsa_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_scmsaset err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_scmsaset2(ctrl, &(data->scmsa2_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_scmsaset2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_th5on(ctrl, &(data->th5_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_th5on err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_pcovrcon(ctrl, &(data->pcovrc_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_pcovrcon err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_uphoseiset2(ctrl, &(data->uphosei2_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_uphoseiset2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_uphoseiset3(ctrl, &(data->uphosei3_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_uphoseiset3 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	

	ret = shgrip_command_scmsa2clr(ctrl, data->scmsa2clr);
	if (ret) {
		SHGRIP_DBG("shgrip_command_scmsa2clr err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_scmsa2set(ctrl, &(data->th_hys_6_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_scmsa2set err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_suphoseiset(ctrl, &(data->suphosei_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_suphoseiset err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_suphoseiset2(ctrl, &(data->suphosei2_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_suphoseiset2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_scuplimitset(ctrl, &(data->scuplimit_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_scuplimitset err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_th1don(ctrl, &(data->th1d_val));
	if (ret) {
		SHGRIP_DBG("shgrip_command_th1don err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	memcpy(&(ctrl->bk_adj), data, sizeof(struct shgrip_sens_setting_params));
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_state                                                      */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_state(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
	int i;
	unsigned char grip, testch;
	unsigned char onoff = 0;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}

	ret = shgrip_command_rdst(ctrl, &grip, &testch);
	if (ret) {
		SHGRIP_ERR("shgrip_command_rdst err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	if (grip & BIT2) {
		if (grip & (BIT0 | BIT1)) {
		SHGRIP_ERR("Scan is stopped because th3, th4 or th5 error occurred and Primary counter is overflow \n");
		onoff = SHGRIP_OFF;
		testch = 0; 
		}else{
		SHGRIP_ERR("Scan is stopped because Primary counter is overflow \n");
		onoff = SHGRIP_OFF;
		testch = 0;
		}
	} else if (grip & BIT1) {
		onoff = SHGRIP_OFF;
		testch = 0;
	
	} else if (grip & BIT3) {
		onoff = SHGRIP_OFF;
		testch = 0;
	
	} else {
		onoff = (grip & BIT0);
	}
	
	for (i = 0; i < SHGRIP_CHANNEL_NUM; i++) {
		ctrl->ch_state[i] = ((testch >> i) & 0x01);
	}
	
	ctrl->sensor_state.state_grip = onoff;
	ctrl->sensor_state.ch0 = ctrl->ch_state[SHGRIP_CH_A];
	ctrl->sensor_state.ch2 = ctrl->ch_state[SHGRIP_CH_B];
	
	SHGRIP_INFO("state_grip:%d, ch0:%d, ch2:%d\n", 
									ctrl->sensor_state.state_grip,
									ctrl->sensor_state.ch0,
									ctrl->sensor_state.ch2);
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_fw_version                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_fw_version(struct shgrip_drv *ctrl)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_SENSOR_ON) {
		SHGRIP_ERR("state is STATE_SENSOR_ON\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	ret = shgrip_command_pver(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_pver err. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_lver(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_lver err. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_command_lver1(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_command_lver1 err. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	SHGRIP_INFO("lver lver0[0x%04X]  lver1[0x%04X]  pver[0x%04X]\n",
											ctrl->ver_info.lver0,
											ctrl->ver_info.lver1,
											ctrl->ver_info.pver);
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_download_fw_proc                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_download_fw_proc(struct shgrip_drv *ctrl)
{
	int ret;
	unsigned long dl_write_ptr;
#if SHGRIP_CHG_LDR_VERIFY_COUNT
	int verify_ret; 
	int verify_count = 0;
#endif /* SHGRIP_CHG_LDR_VERIFY_COUNT */
	
	SHGRIP_DBG("start\n");
	
	ret = shgrip_ldr_checkID(ctrl);
	if (ret) {
		SHGRIP_ERR("Err check ID ret[%d]\n", ret);
		return ret;
	}
	
	if (ctrl->loader_mode == MODE_LOADER0) {
		ret = shgrip_ldr_readverify_fw(ctrl);
		if (ret) {
			SHGRIP_WARN("Start load loader1 , So Err read&verify loader1 ret[%d]\n", ret);
		} else if (ctrl->ver_info.lver1 != SHGRIP_FW_LOADER1_LATEST_VERSION){
			SHGRIP_WARN("Start load loader1 , So different version of loader1 now[%04X] img[%04X]\n",
						ctrl->ver_info.lver1, SHGRIP_FW_LOADER1_LATEST_VERSION);
		} else {
			/* Don't download if non_err, So downloading loader is only verify err or differnt version*/
			return 0;
		}
	}
	
	ret = shgrip_ldr_erase_blodk(ctrl);
	if (ret) {
		SHGRIP_ERR("Err erase block ret[%d]\n", ret);
		return ret;
	}
	
	dl_write_ptr = shgrip_block_addr_table[ctrl->loader_mode][0];
	while (1) {
		ret = shgrip_ldr_write_page(ctrl, dl_write_ptr);
		if (ret) {
			SHGRIP_ERR("Err write page ret[%d]\n", ret);
			return ret;
		}
		
#if SHGRIP_CHG_LDR_VERIFY_COUNT
		verify_ret = shgrip_ldr_read_verify(ctrl, dl_write_ptr);
		if (verify_ret == 0) {
			verify_count = 0;
#endif /* SHGRIP_CHG_LDR_VERIFY_COUNT */
			dl_write_ptr += SHGRIP_LDR_PAGE_SIZE;
			if (dl_write_ptr >= shgrip_block_tenminate[ctrl->loader_mode]) {
				break;
			}
#if SHGRIP_CHG_LDR_VERIFY_COUNT
		} else {
			verify_count++;
			if (verify_count >= SHGRIP_CHG_LDR_VERIFY_COUNT){
				SHGRIP_ERR("Err verify ret[%d], count[%d]\n", verify_ret, verify_count);
				return -EFAULT;
			}
		}
#endif /* SHGRIP_CHG_LDR_VERIFY_COUNT */
	}
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_download_fw                                                    */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_download_fw(struct shgrip_drv *ctrl, int fw_block)
{
	int ret = GRIP_RESULT_SUCCESS;
	int i = 0;
	
	SHGRIP_DBG("start\n");
	
	shgrip_qos_start();
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		shgrip_qos_end();
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_SENSOR_ON) {
		SHGRIP_ERR("state is STATE_SENSOR_ON\n");
		shgrip_qos_end();
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	if (!ctrl->fw_data) {
		SHGRIP_ERR("fw_data is NULL\n");
		shgrip_qos_end();
		return GRIP_RESULT_FAILURE_USER;
	}
	
	ctrl->loader_mode = MODE_LOADER0;
	
	i = SHGRIP_CHG_LDR_MODE_RETRY_COUNT;
	while ((ret = shgrip_seq_reset_start_loader(ctrl))) {
		if (!(i--)) {
			SHGRIP_ERR("retry err change loader mode ret[%d]\n", ret);
			shgrip_qos_end();
			return GRIP_RESULT_FAILURE;
		}
	}
	
	if (fw_block == SHGRIP_DL_ALL_BLOCK) {
#ifdef SHGRIP_LORDER_WRITE	
		ret = shgrip_ldr_get_lver(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_ldr_get_lver  by lver0 NG\n");
			goto err_loader_mode;
		}
		
		if ((int)ctrl->fw_mode_select != MODE_LOADER0) {
			SHGRIP_ERR("can't change mode to loader0 select[%d]\n", 
										ctrl->fw_mode_select);
			goto err_loader_mode;
		}
		
		if ((ctrl->ver_info.lver0 < SHGRIP_FW_LOADER0_OLDEST_VERSION) 
		 || (ctrl->ver_info.lver0 > SHGRIP_FW_LOADER0_LATEST_VERSION)) {
			SHGRIP_ERR("lver0 wrong lver0[%04X]\n",ctrl->ver_info.lver0);
			goto err_loader_mode;
		}
		
		ret = shgrip_seq_download_fw_proc(ctrl);
		if (ret) {
			SHGRIP_ERR("download_fw_proc loader NG  ret[%d]\n", ret);
			goto err_loader_mode;
		}
#endif /* SHGRIP_LORDER_WRITE */
	}
	
	ctrl->loader_mode = MODE_LOADER1;
	
	i = SHGRIP_CHG_LDR_MODE_RETRY_COUNT;
	while ((ret = shgrip_seq_reset_start_loader(ctrl))) {
		if (!(i--)) {
			SHGRIP_ERR("retry err change loader mode ret[%d]\n", ret);
			goto err_loader_mode;
		}
	}
	
	ret = shgrip_ldr_get_lver(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_ldr_get_lver by lver1 NG\n");
		goto err_loader_mode;
	}
	
	if ((int)ctrl->fw_mode_select != MODE_LOADER1) {
		SHGRIP_ERR("can't change mode to loader1 select[%d]\n",
									ctrl->fw_mode_select);
		goto err_loader_mode;
	}
	
	if ((ctrl->ver_info.lver1 < SHGRIP_FW_LOADER1_OLDEST_VERSION) 
	 || (ctrl->ver_info.lver1 > SHGRIP_FW_LOADER1_LATEST_VERSION)) {
		SHGRIP_ERR("lver1 wrong lver1[%04X]\n", ctrl->ver_info.lver1);
		goto err_loader_mode;
	}
	
	ret = shgrip_seq_download_fw_proc(ctrl);
	if (ret) {
		SHGRIP_ERR("download_fw_proc loader NG  ret[%d]\n", ret);
		goto err_loader_mode;
	}
	
	ret = shgrip_seq_reset_start_app(ctrl);
	if (ret) {
		SHGRIP_ERR("reset_start NG after download\n");
		shgrip_qos_end();
		return GRIP_RESULT_FAILURE;
	}
	
	shgrip_qos_end();
	return GRIP_RESULT_SUCCESS;
	
err_loader_mode:
	ret = shgrip_seq_reset_start_app(ctrl);
	if (ret) {
		SHGRIP_ERR("reset_start NG after download\n");
	}
	
	shgrip_qos_end();
	return GRIP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_diag_set_sensor_adjust                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_diag_set_sensor_adjust(struct shgrip_drv *ctrl,
								struct shgrip_diag_sens_setting_params *params)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (params->ch == 0) {
		ret = shgrip_command_gswr1(ctrl, &(params->val));
		if (ret) {
			SHGRIP_ERR("shgrip_command_gswr1 err. ret=%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
		memcpy(&(ctrl->bk_adj.ch0_val), &(params->val), sizeof(struct shgrip_params_val));
	} else if (params->ch == 2) {
		ret = shgrip_command_gswr2(ctrl, &(params->val));
		if (ret) {
			SHGRIP_ERR("shgrip_command_gswr2 err. ret=%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
		memcpy(&(ctrl->bk_adj.ch2_val), &(params->val), sizeof(struct shgrip_params_val));
	} else {
		SHGRIP_ERR("Channel Number Err. CH:%d\n", params->ch);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_diag_get_sensor_adjust                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_diag_get_sensor_adjust(struct shgrip_drv *ctrl,
								struct shgrip_diag_sens_setting_params *params)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (params->ch == 0) {
		ret = shgrip_command_gsrd1(ctrl, &(params->val));
		if (ret) {
			SHGRIP_ERR("shgrip_command_gsrd1 err. ret:%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
	} else if (params->ch == 2) {
		ret = shgrip_command_gsrd2(ctrl, &(params->val));
		if (ret) {
			SHGRIP_ERR("shgrip_command_gsrd2 err. ret:%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
	} else {
		SHGRIP_ERR("Channel Number Err. CH:%d\n", params->ch);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_diag_get_sensor_level                                          */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_diag_get_sensor_level(struct shgrip_drv *ctrl,
								struct shgrip_get_level *level)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	ret = shgrip_command_chprd(ctrl, level);
	if (ret) {
		SHGRIP_ERR("shgrip_command_chprd err. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_diag_change_run_mode                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_diag_change_run_mode(struct shgrip_drv *ctrl, int mode)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_SENSOR_ON) {
		SHGRIP_ERR("state is STATE_SENSOR_ON\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	if (mode == SHGRIP_RUN_MODE) {
		ret = shgrip_command_ttes(ctrl);
		if (ret) {
			SHGRIP_DBG("shgrip_command_ttes err. ret=%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
	} else if (mode == SHGRIP_NORMAL_MODE){
		ret = shgrip_command_ttesoff(ctrl);
		if (ret) {
			SHGRIP_DBG("shgrip_command_ttesoff err. ret=%d\n", ret);
			return GRIP_RESULT_FAILURE;
		}
	} else {
		SHGRIP_ERR("mode is Err mode:%d\n", mode);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_diag_check_sum                                                 */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_diag_check_sum(struct shgrip_drv *ctrl, 
										unsigned short *sum_val)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	if (ctrl->state == STATE_SENSOR_ON) {
		SHGRIP_ERR("state is STATE_SENSOR_ON\n");
		return GRIP_RESULT_FAILURE_USER_STATE;
	}
	
	ret = shgrip_command_romc(ctrl, sum_val);
	if (ret) {
		SHGRIP_DBG("shgrip_command_romc err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_get_chprd2_value                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_get_chprd2_value(struct shgrip_drv *ctrl, struct shgrip_chprd2 *val)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) 
	 || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	ret = shgrip_command_chprd2(ctrl, val);
	if (ret) {
		SHGRIP_DBG("shgrip_command_chprd2 err. ret=%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_seq_debug_rw_command                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_seq_debug_rw_command(struct shgrip_drv *ctrl, 
										struct shgrip_dbg_command *cmd)
{
	int ret = GRIP_RESULT_SUCCESS;
	
	SHGRIP_DBG("start\n");
	
	if ((ctrl->state == STATE_POWER_OFF) || (ctrl->state == STATE_FW_DL)) {
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		return GRIP_RESULT_FAILURE_USER;
	}
	
	ret = shgrip_command_debug_rw_command(ctrl, cmd);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_debug_rw_command failed\n");
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
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_grip_sensor_on(ctrl);
	
	mutex_unlock(&shgrip_io_lock);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_grip_sensor_off                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_grip_sensor_off(void)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_grip_sensor_off(ctrl);
	
	mutex_unlock(&shgrip_io_lock);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_set_sensor_adjust                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_set_sensor_adjust(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_sens_setting_params data;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&data, argp,
							sizeof(struct shgrip_sens_setting_params));
	
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_seq_set_sensor_adjust(ctrl, &data);
	
	mutex_unlock(&shgrip_io_lock);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_sensor_adjust                                            */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_sensor_adjust(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_to_user(argp, &(ctrl->bk_adj.setting_val), 
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
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_get_state(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_state failed\n");
		mutex_unlock(&shgrip_io_lock);
		return ret;
	}
	
	ret = copy_to_user(argp, &ctrl->sensor_state, 
							sizeof(struct shgrip_sensor_state));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	mutex_unlock(&shgrip_io_lock);
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_fw_version                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_fw_version(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_get_fw_version(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_fw_version failed\n");
		return ret;
	}
	
	ret = copy_to_user(argp, &(ctrl->ver_info), 
							sizeof(struct shgrip_fw_version));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_download_fw                                                  */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_download_fw(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_fw_data fw_info;
	unsigned char *fw_data;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&fw_info, argp, 
							sizeof(struct shgrip_fw_data));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed\n");
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	fw_data = kmalloc(fw_info.size, GFP_KERNEL);
	if (!fw_data) {
		SHGRIP_ERR("kmalloc failed\n");
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = copy_from_user(fw_data, fw_info.data, fw_info.size);
	if (ret) {
		SHGRIP_ERR("copy_from_user fw.data failed\n");
		ret = GRIP_RESULT_FAILURE;
		goto dl_fw_func_done;
	}
	
	ctrl->fw_data = fw_data;
	
	ret = shgrip_seq_download_fw(ctrl, SHGRIP_DL_ALL_BLOCK);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_download_fw failed\n");
	}
	
dl_fw_func_done:
	kfree(fw_data);
	mutex_unlock(&shgrip_io_lock);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_set_sensor_adjust                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_set_sensor_adjust(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_diag_sens_setting_params params;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&params, argp,
							sizeof(struct shgrip_diag_sens_setting_params));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_seq_diag_set_sensor_adjust(ctrl, &params);
	
	mutex_unlock(&shgrip_io_lock);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_get_sensor_adjust                                       */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_get_sensor_adjust(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_diag_sens_setting_params params;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&params, argp,
							sizeof(struct shgrip_diag_sens_setting_params));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	memset(&(params.val), 0, sizeof(params.val));
	
	ret = shgrip_seq_diag_get_sensor_adjust(ctrl, &params);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_diag_get_sensor_adjust failed\n");
		mutex_unlock(&shgrip_io_lock);
		return ret;
	}
	
	ret = copy_to_user(argp, &params, 
							sizeof(struct shgrip_diag_sens_setting_params));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	mutex_unlock(&shgrip_io_lock);
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_get_sensor_level                                        */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_get_sensor_level(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_get_level level;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	memset(&level, 0, sizeof(level));
	
	ret = shgrip_seq_diag_get_sensor_level(ctrl, &level);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_diag_get_sensor_level failed\n");
		mutex_unlock(&shgrip_io_lock);
		return ret;
	}
	
	ret = copy_to_user(argp, &level, sizeof(struct shgrip_get_level));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	mutex_unlock(&shgrip_io_lock);
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_change_run_mode                                         */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_change_run_mode(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	int mode = 0;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&mode, argp, sizeof(int));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_seq_diag_change_run_mode(ctrl, mode);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_download_fw                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_download_fw(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	int fw_block;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&fw_block, argp, sizeof(int));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	ctrl->fw_data = (unsigned char *)shgrip_fw_image;
	
	ret = shgrip_seq_download_fw(ctrl, fw_block);
	
	mutex_unlock(&shgrip_io_lock);
	
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_diag_check_sum                                               */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_diag_check_sum(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	unsigned short sum_val = 0;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = shgrip_seq_diag_check_sum(ctrl, &sum_val);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_diag_check_sum failed\n");
		return ret;
	}
	ret = copy_to_user(argp, &sum_val, sizeof(unsigned short));
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
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_to_user(argp, &(ctrl->state), sizeof(unsigned char));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_get_chprd2_value                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_get_chprd2_value(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_chprd2 val;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	memset(&val, 0, sizeof(struct shgrip_chprd2));
	
	ret = shgrip_seq_get_chprd2_value(ctrl, &val);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_chprd2_value failed\n");
		mutex_unlock(&shgrip_io_lock);
		return ret;
	}
	ret = copy_to_user(argp, &val, sizeof(struct shgrip_chprd2));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	mutex_unlock(&shgrip_io_lock);
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_debug_rw_command                                             */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_debug_rw_command(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	struct shgrip_dbg_command cmd;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	memset(&cmd, 0, sizeof(struct shgrip_dbg_command));
	
	ret = copy_from_user(&cmd, argp, sizeof(struct shgrip_dbg_command));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	ret = shgrip_seq_debug_rw_command(ctrl, &cmd);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_debug_rw_command failed\n");
		mutex_unlock(&shgrip_io_lock);
		return ret;
	}
	
	ret = copy_to_user(argp, &cmd, sizeof(struct shgrip_dbg_command));
	if (ret) {
		SHGRIP_ERR("copy_to_user failed. ret:%d\n", ret);
		mutex_unlock(&shgrip_io_lock);
		return GRIP_RESULT_FAILURE;
	}
	
	mutex_unlock(&shgrip_io_lock);
	
	return GRIP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shgrip_ioctl_set_bk_adjust                                                */
/* ------------------------------------------------------------------------- */
static int shgrip_ioctl_set_bk_adjust(void __user *argp)
{
	int ret = GRIP_RESULT_SUCCESS;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	ret = copy_from_user(&(ctrl->bk_adj), argp,
							sizeof(struct shgrip_sens_setting_params));
	if (ret) {
		SHGRIP_ERR("copy_from_user failed. ret:%d\n", ret);
		return GRIP_RESULT_FAILURE;
	}	
	
	return ret;
}

/* ========================================================================= */
/* SHGRIP Input Event Function                                               */
/* ========================================================================= */
/* ------------------------------------------------------------------------- */
/* shgrip_input_event                                                        */
/* ------------------------------------------------------------------------- */
static void shgrip_input_event(struct shgrip_drv *ctrl)
{
	int ret = 0;
	unsigned char grip = 0, onoff = 0, testch = 0;
	int code = 0;
	int i;
	
	ret = shgrip_command_gstclr(ctrl, &grip, &testch);
	if (ret) {
		SHGRIP_ERR("shgrip_command_gstclr failed\n");
		return;
	}
	
	if (grip & BIT2) {
		SHGRIP_ERR("Scan is stopped because Primary counter is overflow \n");
		onoff = SHGRIP_OFF;
		testch = 0;
	} else if (grip & (BIT4 | BIT5 | BIT6 | BIT7)) {
		SHGRIP_ERR("exception reset, grip:0x%02x\n", grip);
		
		i = SHGRIP_CHG_LDR_MODE_RETRY_COUNT;
		while ((ret = shgrip_seq_reset_recovery(ctrl))) {
			if (!(i--)) {
				SHGRIP_ERR("retry err reset_recovery, ret[%d]\n", ret);
				return;
			}
		}
		onoff = SHGRIP_OFF;
		testch = 0;
	} else if (grip & BIT1) {
		onoff = SHGRIP_OFF;
		testch = 0;
	
	} else if (grip & BIT3) {
		onoff = SHGRIP_OFF;
		testch = 0;
	
	} else {
		onoff = (grip & BIT0);
	}
	
	code = SW_GRIP_00;
	input_report_switch(ctrl->input, code, onoff);
	input_sync(ctrl->input);
	
	SHGRIP_DBG("input_event sync code:0x%04X, onoff:%d\n", code, onoff);
	
	for (i = 0; i < SHGRIP_CHANNEL_NUM; i++) {
		ctrl->ch_state[i] = ((testch >> i) & 0x01);
	}
	
	ctrl->sensor_state.state_grip = onoff;
	ctrl->sensor_state.ch0 = ctrl->ch_state[SHGRIP_CH_A];
	ctrl->sensor_state.ch2 = ctrl->ch_state[SHGRIP_CH_B];
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	if ((ctrl->sensor_state.state_grip == SHGRIP_ON) 
	 || (ctrl->sensor_state.state_grip == SHGRIP_OFF)) {
		SHGRIP_DBG("msm_tps_set_grip_state called grip:%d\n", ctrl->sensor_state.state_grip);
		msm_tps_set_grip_state(ctrl->sensor_state.state_grip);
	} else {
		SHGRIP_ERR("grip state failed, grip:%d\n", ctrl->sensor_state.state_grip);
	}
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_work_func                                                          */
/* ------------------------------------------------------------------------- */
static void shgrip_work_func(struct work_struct *work)
{
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = container_of(work, struct shgrip_drv, work_data);
	if (!ctrl) {
		SHGRIP_ERR("ctrl is NULL\n");
		goto shgrip_work_func_done;
	}
	
	if (ctrl->suspended) {
		SHGRIP_WARN("Already Suspended\n");
		goto shgrip_work_func_done;
	}
	
	switch (ctrl->state) { 
	case STATE_SENSOR_ON:
		shgrip_input_event(ctrl);
		shgrip_sys_enable_irq(ctrl);
		break;
	case STATE_POWER_OFF:
	case STATE_SENSOR_OFF:
	case STATE_FW_DL:
	default:
		SHGRIP_WARN("state is changed state:%d\n", ctrl->state); 
		break;
	}
	
shgrip_work_func_done:
	mutex_unlock(&shgrip_io_lock);
	wake_unlock(&shgrip_wake_lock);
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_irq_func                                                           */
/* ------------------------------------------------------------------------- */
static irqreturn_t shgrip_irq_func(int irq, void *dev)
{
	struct shgrip_drv *ctrl = dev;
	
	SHGRIP_DBG("start\n");
	
	wake_lock(&shgrip_wake_lock);
	
	shgrip_sys_disable_irq(ctrl);
	
	queue_work(ctrl->work_queue, &(ctrl->work_data));
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
	struct shgrip_drv *ctrl;
#ifdef SHGRIP_DBG_POLLING_CHPRD2
	struct shgrip_chprd2 chprd2_val;
#else
	struct shgrip_get_level level;
#endif /* SHGRIP_DBG_POLLING_CHPRD2 */
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = container_of(work, struct shgrip_drv, th_work_data);
	if (!ctrl) {
		SHGRIP_ERR("ctrl is NULL\n");
		goto shgrip_work_func_done;
	}
	
	switch (ctrl->state) { 
	case STATE_SENSOR_ON:
		if (shgrip_th_interval_ms) {
			if (th_msec >= 1000) {
				sec  = th_msec / 1000;
				nsec = (th_msec % 1000) * 1000 * 1000;
			} else {
				sec  = 0;
				nsec = th_msec * 1000 * 1000;
			}
			hrtimer_start(&(ctrl->shgrip_threshold_timer), ktime_set(sec, nsec), HRTIMER_MODE_REL);
		}
#ifdef SHGRIP_DBG_POLLING_CHPRD2
		ret = shgrip_command_chprd2(ctrl, &chprd2_val);
		if (ret) {
			SHGRIP_ERR("shgrip_command_chprd2 err. ret:%d\n", ret);
		}
		SHGRIP_WARN("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
			((chprd2_val.dcount_ch0.high_val  << 8) | chprd2_val.dcount_ch0.low_val),
			((chprd2_val.nref_ch0.high_val    << 8) | chprd2_val.nref_ch0.low_val),
			((chprd2_val.nthr_ch0.high_val    << 8) | chprd2_val.nthr_ch0.low_val),
			((chprd2_val.ncount_ch0.high_val  << 8) | chprd2_val.ncount_ch0.low_val),
			((chprd2_val.scudata_ch0.high_val << 8) | chprd2_val.scudata_ch0.low_val),
			((chprd2_val.dcount_ch2.high_val  << 8) | chprd2_val.dcount_ch2.low_val),
			((chprd2_val.nref_ch2.high_val    << 8) | chprd2_val.nref_ch2.low_val),
			((chprd2_val.nthr_ch2.high_val    << 8) | chprd2_val.nthr_ch2.low_val),
			((chprd2_val.ncount_ch2.high_val  << 8) | chprd2_val.ncount_ch2.low_val),
			((chprd2_val.scudata_ch2.high_val << 8) | chprd2_val.scudata_ch2.low_val));
#else
		ret = shgrip_command_chprd(ctrl, &level);
		if (ret) {
			SHGRIP_ERR("shgrip_command_chprd err. ret:%d\n", ret);
		}
		SHGRIP_WARN("%d, %d, %d, %d, %d, %d\n",
			((level.ch0.dcount_val.high_val << 8) | level.ch0.dcount_val.low_val),
			((level.ch0.nref_val.high_val   << 8) | level.ch0.nref_val.low_val),
			((level.ch0.nthr_val.high_val   << 8) | level.ch0.nthr_val.low_val),
			((level.ch2.dcount_val.high_val << 8) | level.ch2.dcount_val.low_val),
			((level.ch2.nref_val.high_val   << 8) | level.ch2.nref_val.low_val),
			((level.ch2.nthr_val.high_val   << 8) | level.ch2.nthr_val.low_val));
#endif /* SHGRIP_DBG_POLLING_CHPRD2 */
		break;
	case STATE_POWER_OFF:
	case STATE_SENSOR_OFF:
	case STATE_FW_DL:
	default:
		th_msec = th_msec_tmp;
		hrtimer_flg = 0;
		break;
	}
shgrip_work_func_done:
	mutex_unlock(&shgrip_io_lock);
	wake_unlock(&shgrip_wake_lock);
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_th_timer_callback                                                  */
/* ------------------------------------------------------------------------- */
static enum hrtimer_restart shgrip_th_timer_callback(struct hrtimer *timer)
{
	struct shgrip_drv *ctrl = container_of(timer, struct shgrip_drv, shgrip_threshold_timer);
	wake_lock(&shgrip_wake_lock);
	queue_work(ctrl->work_queue, &(ctrl->th_work_data));
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
	struct shgrip_drv *ctrl;
	
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("Grip Sensor Device not connected\n");
		return -1;
	}
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	switch (ctrl->state) {
	case STATE_POWER_OFF:
		ret = shgrip_seq_reset_start_app(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_reset_start_app failed ret:%d\n", ret);
			shgrip_seq_grip_power_off(ctrl);
			ctrl->state = STATE_POWER_OFF;
			mutex_unlock(&shgrip_io_lock);
			return -1;
		}
		break;
	case STATE_SENSOR_OFF:
	case STATE_SENSOR_ON:
	case STATE_FW_DL:
	default:
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		break;
	}
	
	mutex_unlock(&shgrip_io_lock);
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_close                                                              */
/* ------------------------------------------------------------------------- */
static int shgrip_close(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct shgrip_drv *ctrl;
	
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("Grip Sensor Device not connected\n");
		return -1;
	}
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;

	/* T.B.D */
	switch (ctrl->state) {
	case STATE_SENSOR_OFF:
		shgrip_seq_grip_power_off(ctrl);
		break;
	case STATE_SENSOR_ON:
		ret = shgrip_seq_grip_sensor_off(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_seq_grip_sensor_off failed\n");
			SHGRIP_DBG("disable_irq(SHGRIP_IRQ_GRIP_INT) \n");
			shgrip_sys_disable_irq(ctrl);
			shgrip_sys_disable_irq_wake(ctrl);
			shgrip_sys_free_irq(ctrl);
		}
		shgrip_seq_grip_power_off(ctrl);
		break;
	case STATE_POWER_OFF:
	case STATE_FW_DL:
	default:
		SHGRIP_ERR("state is Err state:%d\n", ctrl->state);
		mutex_unlock(&shgrip_io_lock);
		return 0;
	}
	
	ctrl->state = STATE_POWER_OFF;
	
#ifndef SHGRIP_FACTORY_MODE_ENABLE
	SHGRIP_DBG("msm_tps_set_grip_state called grip:0\n");
	msm_tps_set_grip_state(SHGRIP_OFF);
#endif /* SHGRIP_FACTORY_MODE_ENABLE */
	
	mutex_unlock(&shgrip_io_lock);
	
	wake_unlock(&shgrip_wake_lock);
	
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

	SHGRIP_INFO("start, cmd:%d\n", (cmd & 0x000000FF));
	
	if (!shgrip_dev_connect) {
		SHGRIP_ERR("Grip Sensor Device not connected\n");
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
	case SHGRIP_IOCTL_DIAG_SET_SENSOR_ADJUST:
		ret = shgrip_ioctl_diag_set_sensor_adjust(argp);
		break;
	case SHGRIP_IOCTL_DIAG_GET_SENSOR_ADJUST:
		ret = shgrip_ioctl_diag_get_sensor_adjust(argp);
		break;
	case SHGRIP_IOCTL_DIAG_GET_SENSOR_LEVEL:
		ret = shgrip_ioctl_diag_get_sensor_level(argp);
		break;
	case SHGRIP_IOCTL_DIAG_CHANGE_RUN_MODE:
		ret = shgrip_ioctl_diag_change_run_mode(argp);
		break;
	case SHGRIP_IOCTL_DIAG_DOWNLOAD_FW:
		ret = shgrip_ioctl_diag_download_fw(argp);
		break;
	case SHGRIP_IOCTL_DIAG_CHECK_SUM:
		ret = shgrip_ioctl_diag_check_sum(argp);
		break;
	case SHGRIP_IOCTL_GET_DRV_STATUS:
		ret = shgrip_ioctl_get_drv_status(argp);
		break;
	case SHGRIP_IOCTL_GET_CHPRD2_VALUE:
		ret = shgrip_ioctl_get_chprd2_value(argp);
		break;
	case SHGRIP_IOCTL_DEBUG_RW_COMMAND:
		ret = shgrip_ioctl_debug_rw_command(argp);
		break;
	case SHGRIP_IOCTL_SET_BK_ADJUST:
		ret = shgrip_ioctl_set_bk_adjust(argp);
		break;
	default:
		SHGRIP_DBG("invalid_value cmd:%d\n", (cmd & 0x000000FF));
		ret = GRIP_RESULT_FAILURE;
		break;
	}
	
	SHGRIP_INFO("end, cmd:%d, ret=%d\n", (cmd & 0x000000FF), ret);
	return ret;
}

#ifdef SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC
/* ------------------------------------------------------------------------- */
/* shgrip_fw_work_func                                                       */
/* ------------------------------------------------------------------------- */
static void shgrip_fw_work_func(struct work_struct *work)
{
	int ret = 0;
	int i = 0;
	unsigned short sum_val;
	struct shgrip_drv *ctrl;
	
	mutex_lock(&shgrip_io_lock);
	
	SHGRIP_DBG("start\n");
	
	ctrl = container_of(work, struct shgrip_drv, fw_work_data);
	if (!ctrl) {
		SHGRIP_ERR("ctrl is NULL\n");
		goto shgrip_fw_work_func_done;
	}
	
	ret = shgrip_seq_reset_start_app(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_reset_start_app failed ret:%d\n", ret);
		shgrip_dev_connect = false;
	} else {
		shgrip_dev_connect = true;
	}
	
	ret = shgrip_command_romc(ctrl, &sum_val);
	if (ret) {
		SHGRIP_ERR("SUM_VALUE is wrong\n");
		
		ctrl->loader_mode = MODE_LOADER0;
		
		i = SHGRIP_CHG_LDR_MODE_RETRY_COUNT;
		while ((ret = shgrip_seq_reset_start_loader(ctrl))) {
			if (!(i--)) {
				SHGRIP_ERR("shgrip_seq_reset_start_loader Failed ret:%d\n", ret);
				shgrip_dev_connect = false;
				ret = GRIP_RESULT_SUCCESS;
				goto shgrip_fw_work_power_off;
			}
		}
		
		ret = shgrip_ldr_get_lver(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_ldr_get_lver by lver0 NG\n");
			shgrip_dev_connect = false;
			ret = GRIP_RESULT_SUCCESS;
			goto shgrip_fw_work_power_off;
		}
		
		shgrip_dev_connect = true;
		
		if ((ctrl->ver_info.pver  != SHGRIP_PVER)
		 || (ctrl->ver_info.lver0 != SHGRIP_LVER0)
		 || (ctrl->ver_info.lver1 != SHGRIP_LVER1)) {
			ctrl->state = STATE_SENSOR_OFF;
			ctrl->fw_data = (unsigned char *)shgrip_fw_image;
			
			ret = shgrip_seq_download_fw(ctrl, SHGRIP_DL_ALL_BLOCK);
			if (ret) {
				SHGRIP_ERR("shgrip_seq_download_fw failed\n");
				goto shgrip_fw_work_power_off;
			}
			
			ret = shgrip_command_romc(ctrl, &sum_val);
			if (ret) {
				SHGRIP_ERR("shgrip_romc failed ret:%d\n", ret);
			}
		} else {
			ret = shgrip_seq_reset_start_app(ctrl);
			if (ret) {
				SHGRIP_ERR("shgrip_seq_reset_start_app failed ret:%d\n", ret);
			}
		}
		
		goto shgrip_fw_work_power_off;
	} else {
		shgrip_dev_connect = true;
	}
	
	ret = shgrip_seq_get_fw_version(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_fw_version failed\n");
		goto shgrip_fw_work_power_off;
	}
	
	if ((ctrl->ver_info.pver  >= SHGRIP_PVER)
	 && (ctrl->ver_info.lver0 >= SHGRIP_LVER0)
	 && (ctrl->ver_info.lver1 >= SHGRIP_LVER1)) {
		goto shgrip_fw_work_power_off;
	}
	
	SHGRIP_WARN("Firmware Download START\n");
	
	ctrl->fw_data = (unsigned char *)shgrip_fw_image;
	
	ret = shgrip_seq_download_fw(ctrl, SHGRIP_DL_ALL_BLOCK);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_download_fw failed\n");
		goto shgrip_fw_work_power_off;
	}
	
	ret = shgrip_command_romc(ctrl, &sum_val);
	if (ret) {
		SHGRIP_ERR("shgrip_romc failed ret:%d\n", ret);
	}
shgrip_fw_work_power_off:
	shgrip_seq_grip_power_off(ctrl);
	ctrl->state = STATE_POWER_OFF;
shgrip_fw_work_func_done:
	mutex_unlock(&shgrip_io_lock);
	return;
}
#endif /* SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC */

/* ------------------------------------------------------------------------- */
/* shgrip_dev_spi_probe                                                      */
/* ------------------------------------------------------------------------- */
static int __devinit shgrip_dev_spi_probe(struct spi_device *spi)
{
#ifdef SHGRIP_FACTORY_MODE_ENABLE
	int ret = 0;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	ctrl->irq = gpio_to_irq(SHGRIP_GPIO_GRIP_INT);
	
	spi_set_drvdata(spi, ctrl);
	spi_dev = spi;
	
	shgrip_dev_connect = true;
	ctrl->state = STATE_POWER_OFF;
	
#ifdef CONFIG_ANDROID_ENGINEERING
	ret = device_create_file(&spi->dev, &dev_attr_shgrip_dev);
	if (ret){
		SHGRIP_ERR("device_create_file failed ret:%d\n", ret);
		return ret;
	}
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */
	
	return ret;
#else
	int ret = 0;
	struct shgrip_drv *ctrl;
#ifndef SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC
	int i = 0;
	unsigned short sum_val;
#endif /* SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC */
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	ctrl->irq = gpio_to_irq(SHGRIP_GPIO_GRIP_INT);
	
	spi_set_drvdata(spi, ctrl);
	spi_dev = spi;
	
#ifdef CONFIG_ANDROID_ENGINEERING
	ret = device_create_file(&spi->dev, &dev_attr_shgrip_dev);
	if (ret){
		SHGRIP_ERR("device_create_file failed ret:%d\n", ret);
		return ret;
	}
#endif /* #ifdef CONFIG_ANDROID_ENGINEERING */
	
#ifdef SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC
	queue_work(ctrl->work_queue, &(ctrl->fw_work_data));
#else /* SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC */
	ret = shgrip_seq_reset_start_app(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_reset_start_app failed ret:%d\n", ret);
		shgrip_dev_connect = false;
	} else {
		shgrip_dev_connect = true;
	}
	ret = shgrip_command_romc(ctrl, &sum_val);
	if (ret) {
		SHGRIP_ERR("SUM_VALUE is wrong\n");
		
		ctrl->loader_mode = MODE_LOADER0;
		
		i = SHGRIP_CHG_LDR_MODE_RETRY_COUNT;
		while ((ret = shgrip_seq_reset_start_loader(ctrl))) {
			if (!(i--)) {
				SHGRIP_ERR("shgrip_seq_reset_start_loader Failed ret:%d\n", ret);
				shgrip_dev_connect = false;
				ret = GRIP_RESULT_SUCCESS;
				goto shgrip_fw_work_power_off;
			}
		}
		
		ret = shgrip_ldr_get_lver(ctrl);
		if (ret) {
			SHGRIP_ERR("shgrip_ldr_get_lver by lver0 NG\n");
			shgrip_dev_connect = false;
			ret = GRIP_RESULT_SUCCESS;
			goto shgrip_fw_work_power_off;
		}
		
		shgrip_dev_connect = true;
		
		if ((ctrl->ver_info.pver  != SHGRIP_PVER)
		 || (ctrl->ver_info.lver0 != SHGRIP_LVER0)
		 || (ctrl->ver_info.lver1 != SHGRIP_LVER1)) {
			ctrl->state = STATE_SENSOR_OFF;
			ctrl->fw_data = (unsigned char *)shgrip_fw_image;
			
			ret = shgrip_seq_download_fw(ctrl, SHGRIP_DL_ALL_BLOCK);
			if (ret) {
				SHGRIP_ERR("shgrip_seq_download_fw failed\n");
				goto shgrip_fw_work_power_off;
			}
			
			ret = shgrip_command_romc(ctrl, &sum_val);
			if (ret) {
				SHGRIP_ERR("shgrip_romc failed ret:%d\n", ret);
			}
		} else {
			ret = shgrip_seq_reset_start_app(ctrl);
			if (ret) {
				SHGRIP_ERR("shgrip_seq_reset_start_app failed ret:%d\n", ret);
			}
		}
		
		goto shgrip_fw_work_power_off;
	} else {
		shgrip_dev_connect = true;
	}
	
	ret = shgrip_seq_get_fw_version(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_get_fw_version failed\n");
		goto shgrip_fw_work_power_off;
	}
	
	if ((ctrl->ver_info.pver  >= SHGRIP_PVER)
	 && (ctrl->ver_info.lver0 >= SHGRIP_LVER0)
	 && (ctrl->ver_info.lver1 >= SHGRIP_LVER1)) {
		goto shgrip_fw_work_power_off;
	}
	
	SHGRIP_WARN("Firmware Download START\n");
	
	ctrl->fw_data = (unsigned char *)shgrip_fw_image;
	
	ret = shgrip_seq_download_fw(ctrl, SHGRIP_DL_ALL_BLOCK);
	if (ret) {
		SHGRIP_ERR("shgrip_seq_download_fw failed\n");
		goto shgrip_fw_work_power_off;
	}
	
	ret = shgrip_command_romc(ctrl, &sum_val);
	if (ret) {
		SHGRIP_ERR("shgrip_romc failed ret:%d\n", ret);
	}
	
shgrip_fw_work_power_off:
	shgrip_seq_grip_power_off(ctrl);
	ctrl->state = STATE_POWER_OFF;
#endif /* SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC */
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
static int shgrip_dev_init(struct shgrip_drv *ctrl)
{
	int ret = 0;
	
	SHGRIP_DBG("start\n");
	
	gpio_request(SHGRIP_GPIO_GRIP_INT, NULL);
	ret = gpio_tlmm_config(GPIO_CFG(SHGRIP_GPIO_GRIP_INT, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret) {
		SHGRIP_ERR("GPIO(TC_PULSE) set config NG\n");
		goto shgrip_gpio_err1;
	}
	ret = gpio_direction_input(SHGRIP_GPIO_GRIP_INT);
	if (ret) {
		SHGRIP_ERR("GPIO(TC_PULSE) set direction NG\n");
		goto shgrip_gpio_err1;
	}
	
	gpio_request(SHGRIP_SPI_CS_N, NULL);
	ret = gpio_tlmm_config(GPIO_CFG(SHGRIP_SPI_CS_N, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret) {
		SHGRIP_ERR("GPIO(SPI_CS) set config NG\n");
		goto shgrip_gpio_err2;
	}
	ret = gpio_direction_output(SHGRIP_SPI_CS_N, 1);
	if (ret) {
		SHGRIP_ERR("GPIO(SPI_CS) set direction NG\n");
		goto shgrip_gpio_err2;
	}
	
	ret = spi_register_driver(&shgrip_dev_spi_driver);
	if (ret) {
		SHGRIP_ERR("spi_register_driver err\n");
		goto shgrip_gpio_err2;
	}
	
	ctrl->state = STATE_POWER_OFF;
	
	return 0;
	
shgrip_gpio_err2:
	gpio_free(SHGRIP_SPI_CS_N);
shgrip_gpio_err1:
	gpio_free(SHGRIP_GPIO_GRIP_INT);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* shgrip_dev_exit                                                           */
/* ------------------------------------------------------------------------- */
static void shgrip_dev_exit(struct shgrip_drv *ctrl)
{
	SHGRIP_DBG("start\n");
	
	spi_unregister_driver(&shgrip_dev_spi_driver);
	
	shgrip_sys_free_irq(ctrl);
	
	gpio_free(SHGRIP_GPIO_GRIP_INT);
	gpio_free(SHGRIP_SPI_CS_N);
	
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_drv_init                                                           */
/* ------------------------------------------------------------------------- */
static int shgrip_drv_init(struct shgrip_drv *ctrl)
{
	int ret;
	
	SHGRIP_DBG("start\n");
	
	ctrl->input = input_allocate_device();
	if (!(ctrl->input)) {
		SHGRIP_ERR("input_allocate_device failed\n");
		return -ENOMEM;
	}
	
	ctrl->input->name         = SHGRIP_NAME;
	ctrl->input->phys         = "shgrip/input0";
	ctrl->input->id.vendor    = 0x0001;
	ctrl->input->id.product   = 0x0001;
	ctrl->input->id.version   = 0x0001;
	
	ctrl->input->evbit[0]     = BIT_MASK(EV_SW);
	
	input_set_capability(ctrl->input, EV_SW, SW_GRIP_00);
	input_set_capability(ctrl->input, EV_SW, SW_GRIP_01);
	input_set_capability(ctrl->input, EV_SW, SW_GRIP_02);
	
	input_set_drvdata(ctrl->input, ctrl);
	
	ret = input_register_device(ctrl->input);
	if (ret) {
		SHGRIP_ERR("input_register_device failed\n");
		input_free_device(ctrl->input);
		return ret;
	}
	
#ifdef CONFIG_ANDROID_ENGINEERING
	hrtimer_init(&ctrl->shgrip_threshold_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ctrl->shgrip_threshold_timer.function = shgrip_th_timer_callback;
#endif /* CONFIG_ANDROID_ENGINEERING */
	
	ctrl->work_queue = create_singlethread_workqueue("shgrip_workqueue");
	if (ctrl->work_queue) {
		INIT_WORK(&ctrl->work_data, shgrip_work_func);
#ifdef SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC
		INIT_WORK(&ctrl->fw_work_data, shgrip_fw_work_func);
#endif /* SHGRIP_KERNEL_INIT_FW_WRITE_ASYNC */
#ifdef CONFIG_ANDROID_ENGINEERING
		INIT_WORK(&ctrl->th_work_data, shgrip_th_dump_func);
#endif /* CONFIG_ANDROID_ENGINEERING */
	} else {
		SHGRIP_ERR("create_singlethread_workqueue failed\n");
		ret = -ENOMEM;
		input_unregister_device(ctrl->input);
		return ret;
	}
	
	mutex_init(&shgrip_io_lock);
	spin_lock_init(&(ctrl->lock));
	pm_qos_add_request(&shgrip_qos_cpu_dma_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	wake_lock_init(&shgrip_wake_lock, WAKE_LOCK_SUSPEND, "shgrip_wake_lock");
	
	return 0;
}

/* ------------------------------------------------------------------------- */
/* shgrip_drv_exit                                                           */
/* ------------------------------------------------------------------------- */
static void shgrip_drv_exit(struct shgrip_drv *ctrl)
{
	SHGRIP_DBG("start\n");
	
	if (ctrl->work_queue) {
		flush_workqueue(ctrl->work_queue);
		destroy_workqueue(ctrl->work_queue);
		ctrl->work_queue = NULL;
	}
	
	input_unregister_device(ctrl->input);
	return;
}

/* ------------------------------------------------------------------------- */
/* shgrip_init                                                               */
/* ------------------------------------------------------------------------- */
static int __init shgrip_init(void)
{
	int ret;
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
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
	
	ret = shgrip_drv_init(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_drv_init failed\n");
		goto error_drv_init;
	}
	
	ret = shgrip_dev_init(ctrl);
	if (ret) {
		SHGRIP_ERR("shgrip_dev_init failed\n");
		goto error_dev_init;
	}
	
	return 0;
	
error_dev_init:
	shgrip_drv_exit(ctrl);
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
	struct shgrip_drv *ctrl;
	
	SHGRIP_DBG("start\n");
	
	ctrl = &grip_ctrl;
	
	shgrip_dev_exit(ctrl);
	
	shgrip_drv_exit(ctrl);
	
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
