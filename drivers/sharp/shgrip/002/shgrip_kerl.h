

/* drivers/sharp/shgrip/shgrip_kerl.h
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
#ifndef __SHGRIP_KERL_H__
#define __SHGRIP_KERL_H__

/* ------------------------------------------------------------------------- */
/* IOCTL                                                                     */
/* ------------------------------------------------------------------------- */
#define SHGRIP_IOC_MAGIC					's'

#define SHGRIP_IOCTL_GRIP_SENSOR_ON			_IO  (SHGRIP_IOC_MAGIC,  1)
#define SHGRIP_IOCTL_GRIP_SENSOR_OFF		_IO  (SHGRIP_IOC_MAGIC,  2)
#define SHGRIP_IOCTL_SET_SENSOR_ADJUST		_IOW (SHGRIP_IOC_MAGIC,  3,  struct shgrip_sens_setting_params)
#define SHGRIP_IOCTL_GET_SENSOR_ADJUST		_IOR (SHGRIP_IOC_MAGIC,  4,  struct shgrip_user_setting)
#define SHGRIP_IOCTL_GET_STATE				_IOR (SHGRIP_IOC_MAGIC,  5,  struct shgrip_sensor_state)
#define SHGRIP_IOCTL_GET_FW_VERSION			_IOR (SHGRIP_IOC_MAGIC,  6,  struct shgrip_fw_version)
#define SHGRIP_IOCTL_DOWNLOAD_FW			_IOW (SHGRIP_IOC_MAGIC,  7,  struct shgrip_fw_data) 
#define SHGRIP_IOCTL_DIAG_SET_SENSOR_ADJUST	_IOW (SHGRIP_IOC_MAGIC,  8,  struct shgrip_diag_sens_setting_params)
#define SHGRIP_IOCTL_DIAG_GET_SENSOR_ADJUST	_IOWR(SHGRIP_IOC_MAGIC,  9, struct shgrip_diag_sens_setting_params)
#define SHGRIP_IOCTL_DIAG_GET_SENSOR_LEVEL	_IOR (SHGRIP_IOC_MAGIC,  10, struct shgrip_get_level)
#define SHGRIP_IOCTL_DIAG_CHANGE_RUN_MODE	_IOW (SHGRIP_IOC_MAGIC,  11, int)
#define SHGRIP_IOCTL_DIAG_DOWNLOAD_FW		_IOW (SHGRIP_IOC_MAGIC,  12, int)
#define SHGRIP_IOCTL_DIAG_CHECK_SUM			_IOR (SHGRIP_IOC_MAGIC,  13, unsigned short)
#define SHGRIP_IOCTL_GET_DRV_STATUS			_IOR (SHGRIP_IOC_MAGIC,  14, unsigned char)
#define SHGRIP_IOCTL_GET_CHPRD2_VALUE		_IOR (SHGRIP_IOC_MAGIC, 15, struct shgrip_chprd2)
#define SHGRIP_IOCTL_DEBUG_RW_COMMAND		_IOWR(SHGRIP_IOC_MAGIC, 16, struct shgrip_dbg_command)
#define SHGRIP_IOCTL_SET_BK_ADJUST			_IOW (SHGRIP_IOC_MAGIC, 17, struct shgrip_sens_setting_params)
#define SHGRIP_IOCTL_DEBUG_RW_COMMAND2		_IOWR(SHGRIP_IOC_MAGIC, 18, struct shgrip_dbg_command2)
#define SHGRIP_IOCTL_GET_CHPRM_VALUE		_IOR (SHGRIP_IOC_MAGIC, 19, struct shgrip_chprm)
#define SHGRIP_IOCTL_SET_RESET				_IO (SHGRIP_IOC_MAGIC, 20)

/* ------------------------------------------------------------------------- */
/* TYPE                                                                     */
/* ------------------------------------------------------------------------- */
enum {
	GRIP_RESULT_SUCCESS,
	GRIP_RESULT_FAILURE,
	GRIP_RESULT_FAILURE_USER,
	GRIP_RESULT_FAILURE_USER_STATE,
	MAX_SHGRIP_RESULT
};

enum {
	SHGRIP_TYPE_LPTIME_LEVEL00,
	SHGRIP_TYPE_LPTIME_LEVEL01,
	SHGRIP_TYPE_LPTIME_LEVEL02,
	SHGRIP_TYPE_LPTIME_LEVEL03,
	MAX_SHGRIP_TYPE_LPTIME
};

enum {
	SHGRIP_TYPE_SPTIME_LEVEL00,
	SHGRIP_TYPE_SPTIME_LEVEL01,
	SHGRIP_TYPE_SPTIME_LEVEL02,
	SHGRIP_TYPE_SPTIME_LEVEL03,
	MAX_SHGRIP_TYPE_SPTIME
};

enum {
	SHGRIP_TYPE_SENS_LEVEL00,
	SHGRIP_TYPE_SENS_LEVEL01,
	SHGRIP_TYPE_SENS_LEVEL02,
	SHGRIP_TYPE_SENS_LEVEL03,
	MAX_SHGRIP_TYPE_SENS
};

enum {
	SHGRIP_OFF,
	SHGRIP_ON
};

enum {
	SHGRIP_CH_A,
	SHGRIP_CH_B
};

enum {
	SHGRIP_DL_ALL_BLOCK,
	SHGRIP_DL_APP_BLOCK,
	MAX_SHGRIP_DL_BLOCK
};

enum {
	SHGRIP_NORMAL_MODE,
	SHGRIP_RUN_MODE,
	MAX_SHGRIP_ACTION_MODE
};

enum shgrip_state {
	SHGRIP_STATE_POWER_OFF,
	SHGRIP_STATE_SENSOR_OFF,
	SHGRIP_STATE_SENSOR_ON,
	SHGRIP_STATE_FW_DL,
	SHGRIP_STATE_HALT_MODE
};

struct shgrip_dcount {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_nref {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_prm_dci {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_prm_acd {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_df_scs {
	unsigned char val;
};

struct shgrip_nhys {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_prm_msa {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_nthr {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_user_setting {
	unsigned char ch0;
	unsigned char ch2;
	unsigned char lptime;
	unsigned char sptime;
};

struct shgrip_params_val {
	struct shgrip_dcount dcount_val;
	struct shgrip_nref nref_val;
	struct shgrip_prm_dci prm_dci_val;
	struct shgrip_prm_acd prm_acd_val;
	struct shgrip_df_scs df_scs_val;
	struct shgrip_nhys nhys_val;
	struct shgrip_prm_msa prm_msa_val;
	struct shgrip_nthr nthr_val;
};

struct shgrip_diag_sens_setting_params {
	unsigned char ch;
	struct shgrip_params_val val;
};

struct shgrip_level_val {
	struct shgrip_dcount dcount_val;
	struct shgrip_nref nref_val;
	struct shgrip_nthr nthr_val;
};

struct shgrip_get_level {
	struct shgrip_level_val ch0;
	struct shgrip_level_val ch2;
};

struct shgrip_fw_version {
	unsigned short pver;
	unsigned short lver0;
	unsigned short lver1;
};

struct shgrip_fw_data {
	unsigned long size;
	unsigned char* data;
};

struct shgrip_sensor_state {
	unsigned char state_grip;
	unsigned char ch0;
	unsigned char ch2;
	unsigned char aet_error;
};

struct shgrip_ncount {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_scudata {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_chprd2 {
	struct shgrip_dcount dcount_ch0;
	struct shgrip_nref nref_ch0;
	struct shgrip_nthr nthr_ch0;
	struct shgrip_dcount dcount_ch2;
	struct shgrip_nref nref_ch2;
	struct shgrip_nthr nthr_ch2;
	struct shgrip_ncount ncount_ch0;
	struct shgrip_scudata scudata_ch0;
	struct shgrip_ncount ncount_ch2;
	struct shgrip_scudata scudata_ch2;
};

struct shgrip_sens_setting_params {
	struct shgrip_user_setting setting_val;
	unsigned char shgrip_lpcr[3];
	unsigned char shgrip_acqrater;
	unsigned char shgrip_ch0cr;
	unsigned char shgrip_ch4cr;
	unsigned char shgrip_trigccr[11];
	unsigned char sens_ch_a[14];
	unsigned char sens_ch_b[14];
	unsigned char shgrip_delaygriprh[3];
	unsigned char shgrip_aetrepeatr;
};

struct shgrip_dbg_command {
	unsigned char  addr;
	unsigned char  w_size;
	unsigned char  w_buf[32];
	unsigned char  r_size;
	unsigned char  r_buf[32];
};

struct shgrip_dbg_command2 {
	unsigned short addr;
	unsigned char w_size;
	unsigned char w_buf[32];
	unsigned char r_size;
	unsigned char r_buf[32];
};

struct shgrip_meas {
    unsigned char high_val;
    unsigned char low_val;
};

struct shgrip_ref {
    unsigned char high_val;
    unsigned char low_val;
};

struct shgrip_delta {
    unsigned char high_val;
    unsigned char low_val;
};

struct shgrip_proxth {
    unsigned char high_val;
    unsigned char low_val;
};

struct shgrip_touchth {
    unsigned char high_val;
    unsigned char low_val;
};

struct shgrip_chprm {
    struct shgrip_meas meas_ch0;
    struct shgrip_ref ref_ch0;
    struct shgrip_delta delta_ch0;
    struct shgrip_meas meas_ch4;
    struct shgrip_ref ref_ch4;
    struct shgrip_delta delta_ch4;
    struct shgrip_proxth proxth_ch0;
    struct shgrip_touchth touchth_ch0;
    unsigned char releaseth_ch0;
    struct shgrip_proxth proxth_ch4;
    struct shgrip_touchth touchth_ch4;
    unsigned char releaseth_ch4;
};

#endif /* __SHGRIP_KERL_H__ */
