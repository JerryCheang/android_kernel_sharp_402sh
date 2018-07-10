

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
	STATE_POWER_OFF,
	STATE_SENSOR_OFF,
	STATE_SENSOR_ON,
	STATE_FW_DL
};

enum shgrip_loader_mode {
    MODE_LOADER0,
    MODE_LOADER1
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

struct shgrip_sampling_val {
	unsigned char sampling_a;
	unsigned char sampling_b;
	unsigned char sampling_c;
	unsigned char sampling_d;
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

struct shgrip_drift_thr{
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_scancnt{
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_drift_val{
	struct shgrip_drift_thr thr_val_ch0;
	struct shgrip_drift_thr thr_val_ch2;
	struct shgrip_scancnt scancnt_ch0;
	struct shgrip_dcount dcount_ch0;
	struct shgrip_scancnt scancnt_ch2;
	struct shgrip_dcount dcount_ch2;
};

struct shgrip_drift_set{
	unsigned char drift_on_ch;
	struct shgrip_drift_val val;
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
};

struct shgrip_sfr1_reg {
	unsigned char scucr0;
	unsigned char scufr;
	unsigned char scustc;
	unsigned char scuscc;
	unsigned char fra0;
};

struct shgrip_sfr2_reg {
	unsigned short addr;
	unsigned char val;
};

struct shgrip_threshold3_reg {
	struct shgrip_nhys nhys3_ch0;
	struct shgrip_nhys nhys3_ch2;
	struct shgrip_nthr nthr3_ch0;
	struct shgrip_nthr nthr3_ch2;
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

struct shgrip_scancntn {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_thr3_cancelval {
	struct shgrip_scancntn scancntn_ch0;
	unsigned char scancntm_ch0;
	struct shgrip_scancntn scancntn_ch2;
	unsigned char scancntm_ch2;
};

struct shgrip_s_dtimes {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_s_delta {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_s_dfilter {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_uphosei {
	unsigned char hosei_onoff;
	unsigned char patern;
	unsigned char q_dtimes_ch0;
	unsigned char q_dtimes_ch2;
	unsigned char q_delta_ch0;
	unsigned char q_delta_ch2;
	unsigned char q_dfilter;
	struct shgrip_s_dtimes s_dtimes_ch0;
	struct shgrip_s_dtimes s_dtimes_ch2;
	struct shgrip_s_delta s_delta_ch0;
	struct shgrip_s_delta s_delta_ch2;
	struct shgrip_s_dfilter s_dfilter;
};

struct shgrip_drift_set2 {
	unsigned char drift_on_ch;
	unsigned char scancnt_ch0;
	unsigned char dcount_ch0;
	unsigned char scancnt_ch2;
	unsigned char dcount_ch2;
};

struct shgrip_msa {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_msa_x {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_scmsa_val {
	unsigned char a;
	unsigned char b;
	struct shgrip_msa_x x;
	unsigned char y;
	unsigned char z;
};

struct shgrip_scmsa {
	unsigned char scmsa_on;
	unsigned char triga_on;
	struct shgrip_msa msa;
	struct shgrip_scmsa_val ch0;
	struct shgrip_scmsa_val ch2;
};

struct shgrip_scmsa_count {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_scmsa2 {
	struct shgrip_scmsa_count count_a;
	struct shgrip_scmsa_count count_t;
};

struct shgrip_threshold4_reg {
	unsigned char th4_on;
	struct shgrip_nhys nhys4_ch0;
	struct shgrip_nhys nhys4_ch2;
	struct shgrip_nthr nthr4_ch0;
	struct shgrip_nthr nthr4_ch2;
};

struct shgrip_chprd4 {
	struct shgrip_ncount ncount_ch0;
	struct shgrip_nref nref_ch0;
	struct shgrip_scudata scudata_ch0;
	struct shgrip_ncount ncount_ch1;
	struct shgrip_nref nref_ch1;
	struct shgrip_scudata scudata_ch1;
	struct shgrip_ncount ncount_ch2;
	struct shgrip_nref nref_ch2;
	struct shgrip_scudata scudata_ch2;
};

struct shgrip_th5_s_cnt {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_th5_t_cnt {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_threshold5_reg {
	unsigned char thr5onset;
	unsigned char thr5cansel_on;
	struct shgrip_nhys nhys5_ch0;
	struct shgrip_nhys nhys5_ch2;
	struct shgrip_nthr nthr5_ch0;
	struct shgrip_nthr nthr5_ch2;
	struct shgrip_th5_s_cnt th5_s_cnt;
	struct shgrip_th5_t_cnt th5_t_cnt;
};

struct shgrip_pcovrc_r_cnt_thr {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_pcovrc_r_cnt {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_pcovrc_p_cnt {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_pcovrc_reg {
	unsigned char pcovrconset;
	unsigned char pcovrcint_on;
	struct shgrip_pcovrc_r_cnt pcovrc_r_cnt;
};

struct shgrip_mode_state2 {
	unsigned char mode;
	unsigned char mode2;
	unsigned char mode3;
};

struct shgrip_w_q_dtimes {
	unsigned char ch0;
	unsigned char ch2;
};

struct shgrip_w_q_2nd_delta {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_uphoseiset2 {
	unsigned char hosei_onoff2;
	struct shgrip_w_q_dtimes dtimes_1st;
	unsigned char w_q_1st_delta_ch0;
	unsigned char w_q_1st_delta_ch2;
	struct shgrip_w_q_dtimes dtimes_2nd;
	struct shgrip_w_q_2nd_delta delta_ch0;
	struct shgrip_w_q_2nd_delta delta_ch2;
	unsigned char w_q_2nd_delay_ch0;
	unsigned char w_q_2nd_delay_ch2;
};

struct shgrip_w_s_dtimes {
	unsigned char ch0;
	unsigned char ch2;
};

struct shgrip_w_s_2nd_delta {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_uphoseiset3 {
	unsigned char hosei_onoff3;
	struct shgrip_w_s_dtimes dtimes_1st;
	unsigned char w_s_1st_delta_ch0;
	unsigned char w_s_1st_delta_ch2;
	struct shgrip_w_s_dtimes dtimes_2nd;
	struct shgrip_w_s_2nd_delta delta_ch0;
	struct shgrip_w_s_2nd_delta delta_ch2;
	unsigned char w_s_2nd_delay_ch0;
	unsigned char w_s_2nd_delay_ch2;
};

struct shgrip_th_hys_6 {
	unsigned char scmsa2_on;
	struct shgrip_nhys nhys6_0;
	struct shgrip_nhys nhys6_2;
	struct shgrip_nthr nthr6_0;
	struct shgrip_nthr nthr6_2;
	unsigned char th6_x_0;
	unsigned char th6_x_2;
	unsigned char hys6_y_0;
	unsigned char hys6_y_2;
	unsigned char ch0_alpha;
	unsigned char ch2_alpha;
};

struct shgrip_suphosei {
	unsigned char s_hosei_onoff;
	unsigned char la_mode;
	unsigned char hosei_trig;
	unsigned char jla_mode;
	unsigned char la_tson_ch0;
	unsigned char la_tson_ch2;
	unsigned char la_th3_ch0;
	unsigned char la_th3_ch2;
	unsigned char la_th4_ch0;
	unsigned char la_th4_ch2;
	unsigned char la_th5_ch0;
	unsigned char la_th5_ch2;
	unsigned char la_man_ch0;
	unsigned char la_man_ch2;
	struct shgrip_s_dtimes dtimes_ch0;
	struct shgrip_s_dtimes dtimes_ch2;
};

struct shgrip_jla_tson {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_jla_th3 {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_jla_th4 {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_jla_th5 {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_la_abs {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_suphosei2 {
	struct shgrip_jla_tson jla_tson_ch0;
	struct shgrip_jla_tson jla_tson_ch2;
	struct shgrip_jla_th3 jla_th3_ch0;
	struct shgrip_jla_th3 jla_th3_ch2;
	struct shgrip_jla_th4 jla_th4_ch0;
	struct shgrip_jla_th4 jla_th4_ch2;
	struct shgrip_jla_th5 jla_th5_ch0;
	struct shgrip_jla_th5 jla_th5_ch2;
	struct shgrip_la_abs la_abs_ch0;
	struct shgrip_la_abs la_abs_ch2;
};

struct shgrip_limit {
	unsigned char high_val;
	unsigned char low_val;
};

struct shgrip_scuplimit {
	unsigned char sc_uplimit_onoff;
	struct shgrip_limit limit_ch0;
	struct shgrip_limit limit_ch2;
};

struct shgrip_th1d {
	unsigned char th1d_on;
	unsigned char nhys1d_0;
	unsigned char nhys1d_2;
	unsigned char nthr1d_0;
	unsigned char nthr1d_2;
	unsigned char judgeval_0;
	unsigned char judgeval_2;
	unsigned char set_scancnt_0;
	unsigned char set_scancnt_2;
};

struct shgrip_sens_setting_params {
	struct shgrip_params_val ch0_val;
	struct shgrip_params_val ch2_val;
	struct shgrip_user_setting setting_val;
	struct shgrip_sampling_val smp;
	struct shgrip_drift_set drift_val;
	struct shgrip_drift_set2 drift2_val;
	unsigned char th3onset;
	struct shgrip_threshold3_reg th3_val;
	unsigned char thr3cancel_on;
	struct shgrip_thr3_cancelval thr3_cancelval;
	struct shgrip_scmsa scmsa_val;
	struct shgrip_scmsa2 scmsa2_val;
	struct shgrip_threshold4_reg th4_val;
	struct shgrip_threshold5_reg th5_val;
	struct shgrip_pcovrc_reg pcovrc_val;
	struct shgrip_uphoseiset2 uphosei2_val;
	struct shgrip_uphoseiset3 uphosei3_val;
	unsigned char scmsa2clr;
	struct shgrip_th_hys_6 th_hys_6_val;
	struct shgrip_suphosei suphosei_val;
	struct shgrip_suphosei2 suphosei2_val;
	struct shgrip_scuplimit scuplimit_val;
	struct shgrip_th1d th1d_val;
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
#endif /* __SHGRIP_KERL_H__ */
