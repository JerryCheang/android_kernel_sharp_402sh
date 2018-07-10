/* drivers/sharp/nfc/nfc.h (NFC Common Header)
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

#ifndef NFC_H
#define NFC_H

/* DEBUG_LOG */
#if 0
#define DEBUG_NFC_DRV
#endif

#ifdef DEBUG_NFC_DRV
#define NFC_DRV_DBG_LOG(fmt, args...) printk(KERN_INFO "[NFC][%s]" fmt "\n", __func__, ## args)
#else
#define NFC_DRV_DBG_LOG(fmt, args...)
#endif

/* ERROR_LOG */
#define NFC_DRV_ERR_LOG(fmt, args...) printk(KERN_ERR "[NFC][%s]ERR " fmt "\n", __func__, ## args)

/* common GPIO number */
#define D_VEN_GPIO_NO			g_ven_gpio_no
#define D_FIRM_GPIO_NO			(77)
#define D_WAKEUP_GPIO_NO		(59)

/* wakeup state */
#define D_WAKEUP_STATE_DOWN		(0)
#define D_WAKEUP_STATE_UP		(1)

extern unsigned g_ven_gpio_no;

int snfc_get_powctrl_flg(void);

void snfc_change_wakeup_mode(int state);

#endif /* NFC_H */

