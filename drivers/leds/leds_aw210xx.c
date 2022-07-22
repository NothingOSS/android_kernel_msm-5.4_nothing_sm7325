/*
 * leds-aw210xx.c
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 *  Author: hushanping <hushanping@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/random.h>
#include <linux/time.h>
#include <linux/leds.h>
#include "leds_aw210xx.h"
#include "leds_aw210xx_reg.h"
#include "leds_aw210xx_glyphs.h"

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW210XX_DRIVER_VERSION "V0.3.0"
#define AW_I2C_RETRIES 5
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1

aw210xx_cfg_t aw210xx_all_sl_cfg = {};
aw210xx_cfg_t aw210xx_all_sl_cfg_t0 = {aw21018_all_sl_t0, sizeof(aw21018_all_sl_t0)};
aw210xx_cfg_t aw210xx_all_sl_cfg_evt = {aw21018_all_sl_evt, sizeof(aw21018_all_sl_evt)};

/******************************************************
 *
 * aw210xx led parameter
 *
 ******************************************************/
aw210xx_cfg_t aw210xx_all_leds_cfg_array[] = {
	{aw21018_all_leds_off, sizeof(aw21018_all_leds_off)},
	{aw21018_all_leds_on, sizeof(aw21018_all_leds_on)}
};

static const uint16_t gamma_brightness[] = {
        0,1,3,4,5,7,8,9,11,12,14,15,17,18,20,22,29,31,33,35,38,40,42,44,47,
        49,51,54,56,59,62,64,80,83,87,90,93,97,100,104,108,111,115,119,123,
        127,131,135,139,143,147,152,156,161,165,170,175,179,184,189,194,199,
        205,210,215,221,226,232,238,244,249,255,262,268,274,281,287,294,301,
        307,314,321,329,336,343,351,359,367,374,383,391,399,408,416,425,434,
        443,452,462,471,481,491,501,511,521,531,542,553,564,575,587,598,610,
        622,634,646,659,672,685,698,711,725,739,753,767,782,796,811,827,842,
        858,874,890,907,924,941,958,976,994,1012,1031,1050,1069,1088,1108,
        1128,1149,1169,1191,1212,1234,1256,1279,1302,1325,1349,1373,1397,
        1422,1448,1473,1499,1526,1553,1580,1608,1637,1666,1695,1725,1755,
        1786,1817,1849,1881,1914,1947,1981,2016,2051,2087,2123,2160,2197,
        2235,2274,2313,2353,2394,2435,2477,2520,2563,2607,2652,2697,2744,
        2791,2839,2887,2936,2987,3038,3090,3142,3196,3250,3306,3362,3402,
        3442,3482,3522,3562,3602,3642,3682,3707,3732,3757,3782,3807,3832,
        3857,3872,3887,3902,3917,3932,3947,3962,3977,3985,3993,4001,4009,
        4017,4025,4033,4041,4045,4049,4053,4057,4061,4065,4069,4073,4075,
        4077,4079,4081,4083,4085,4087,4089,4095
};

static const uint16_t gamma_steps = sizeof(gamma_brightness)/sizeof(uint16_t);

static AW_MULTI_BREATH_DATA_STRUCT aw210xx_leds_breath_data = {{0, 500, 50, 500, 50}, 1, 4095, 0};
unsigned char breath_loop_end =0;
int dev_is_black = 1;

static DECLARE_WAIT_QUEUE_HEAD(aw210xx_nf_waitq);
int ev_happen = 0;
char ev_code = '0';

aw210xx_effect_cfg_t aw210xx_ringtone_leds_effect[] = {
	{aw21018_ringtone_1, sizeof(aw21018_ringtone_1)/sizeof(aw21018_ringtone_1[0])},
	{aw21018_ringtone_2, sizeof(aw21018_ringtone_2)/sizeof(aw21018_ringtone_2[0])},
	{aw21018_ringtone_3, sizeof(aw21018_ringtone_3)/sizeof(aw21018_ringtone_3[0])},
	{aw21018_ringtone_4, sizeof(aw21018_ringtone_4)/sizeof(aw21018_ringtone_4[0])},
	{aw21018_ringtone_5, sizeof(aw21018_ringtone_5)/sizeof(aw21018_ringtone_5[0])},
	{aw21018_ringtone_6, sizeof(aw21018_ringtone_6)/sizeof(aw21018_ringtone_6[0])},
	{aw21018_ringtone_7, sizeof(aw21018_ringtone_7)/sizeof(aw21018_ringtone_7[0])},
	{aw21018_ringtone_8, sizeof(aw21018_ringtone_8)/sizeof(aw21018_ringtone_8[0])},
	{aw21018_ringtone_9, sizeof(aw21018_ringtone_9)/sizeof(aw21018_ringtone_9[0])},
	{aw21018_ringtone_10, sizeof(aw21018_ringtone_10)/sizeof(aw21018_ringtone_10[0])},
	//{aw21018_ringtone_test, sizeof(aw21018_ringtone_test)/sizeof(aw21018_ringtone_test[0])}
};

aw210xx_effect_cfg_t aw210xx_notification_leds_effect[] = {
	{aw21018_notification_1, sizeof(aw21018_notification_1)/sizeof(aw21018_notification_1[0])},
	{aw21018_notification_2, sizeof(aw21018_notification_2)/sizeof(aw21018_notification_2[0])},
	{aw21018_notification_3, sizeof(aw21018_notification_3)/sizeof(aw21018_notification_3[0])},
	{aw21018_notification_4, sizeof(aw21018_notification_4)/sizeof(aw21018_notification_4[0])},
	{aw21018_notification_5, sizeof(aw21018_notification_5)/sizeof(aw21018_notification_5[0])},
	{aw21018_notification_6, sizeof(aw21018_notification_6)/sizeof(aw21018_notification_6[0])},
	{aw21018_notification_7, sizeof(aw21018_notification_7)/sizeof(aw21018_notification_7[0])},
	{aw21018_notification_8, sizeof(aw21018_notification_8)/sizeof(aw21018_notification_8[0])},
	{aw21018_notification_9, sizeof(aw21018_notification_9)/sizeof(aw21018_notification_9[0])},
	{aw21018_notification_10, sizeof(aw21018_notification_10)/sizeof(aw21018_notification_10[0])},
	//{aw21018_notification_test, sizeof(aw21018_notification_test)/sizeof(aw21018_notification_test[0])},
};

aw210xx_effect_cfg_t aw210xx_general_leds_effect[] = {
	{aw21018_wired_charge, sizeof(aw21018_wired_charge)/sizeof(aw21018_wired_charge[0])},
	{aw21018_reverse, sizeof(aw21018_reverse)/sizeof(aw21018_reverse[0])},
	{aw21018_FlipToGlyph, sizeof(aw21018_FlipToGlyph)/sizeof(aw21018_FlipToGlyph[0])},
};

int rt_flush_delay = 16666;
int general_effect_end = 0;

static int aw210xx_read_device_color(const char *filename, char *buf,  int length)
{
	struct file *filep;
	mm_segment_t old_fs;
	loff_t pos;
	int ret = 0;

	filep= filp_open(filename, O_RDONLY, 0);
	if(IS_ERR(filep))
	{
		AW_ERR("open %s err!\n",filename);
		return 0;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 0;

	if(filep->f_op->read){
		ret = filep->f_op->read(filep, buf, length, &pos);
	}else{
		ret = vfs_read(filep, buf, length, &pos);
	}
	set_fs(old_fs);
	filp_close(filep, NULL);

	return ret;
}

/******************************************************
 *
 * aw210xx i2c write/read
 *
 ******************************************************/
static int aw210xx_i2c_write(struct aw210xx *aw210xx,
		unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw210xx->i2c,
				reg_addr, reg_data);
		if (ret < 0)
			AW_ERR("i2c_write cnt=%d ret=%d\n", cnt, ret);
		else
			break;
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
				AW_I2C_RETRY_DELAY * 1000 + 500);
	}

	return ret;
}

static int aw210xx_i2c_read(struct aw210xx *aw210xx,
		unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw210xx->i2c, reg_addr);
		if (ret < 0) {
			AW_ERR("i2c_read cnt=%d ret=%d\n", cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
				AW_I2C_RETRY_DELAY * 1000 + 500);
	}

	return ret;
}

static int aw210xx_i2c_write_bits(struct aw210xx *aw210xx,
		unsigned char reg_addr, unsigned int mask,
		unsigned char reg_data)
{
	unsigned char reg_val;

	aw210xx_i2c_read(aw210xx, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw210xx_i2c_write(aw210xx, reg_addr, reg_val);

	return 0;
}

/*
*Perform a better visual LED breathing effect,
*we recommend using a gamma corrected value
*to set the LED intensity.
*This results in a reduced number of steps for
*the LED intensity setting, but causes the change
*in intensity to appear more linear to the human eye.
*/
uint16_t aw210xx_algorithm_get_correction(ALGO_DATA_STRUCT *p_algo_data)
{
	uint16_t start_idx = 0;
	int32_t end_idx = 0;

	if (p_algo_data->cur_frame == 0)
		start_idx = p_algo_data->data_start;
	else if ((p_algo_data->total_frames-1) == p_algo_data->cur_frame)
		start_idx = p_algo_data->data_end;
	else if (p_algo_data->data_end >= p_algo_data->data_start) {
		/* get the start index in gamma array */
		while (start_idx < gamma_steps) {
			if (gamma_brightness[start_idx] >= p_algo_data->data_start)
				break;

			start_idx++;
		}

		if (start_idx >= gamma_steps)
			start_idx = gamma_steps - 1;

		/* get the end index in gamma array */
		end_idx = gamma_steps - 1;
		while (end_idx >= 0) {
			if (gamma_brightness[end_idx] <= p_algo_data->data_end)
				break;

			end_idx--;
		}

		if (end_idx < 0)
			end_idx = 0;

		/* get current index */
		start_idx += (end_idx-start_idx) *
					 p_algo_data->cur_frame /
					 (p_algo_data->total_frames-1);
		/* get start index */
		start_idx = gamma_brightness[start_idx];
	} else {
		/* get the start index in gamma array */
		while (start_idx < gamma_steps) {
			if (gamma_brightness[start_idx] >= p_algo_data->data_end)
				break;

			start_idx++;
		}

		if (start_idx >= gamma_steps)
			start_idx = gamma_steps - 1;

		/* get the end index in gamma array */
		end_idx = gamma_steps - 1;
		while (end_idx >= 0) {
			if (gamma_brightness[end_idx] <= p_algo_data->data_start)
				break;

			end_idx--;
		}

		if (end_idx < 0)
			end_idx = 0;

		/* get currrent index */
		end_idx -= (end_idx-start_idx)*p_algo_data->cur_frame/(p_algo_data->total_frames - 1);
		/* get start index */
		start_idx = gamma_brightness[end_idx];
	}

	return start_idx;
}

/*****************************************************
* led Interface: set effect
*****************************************************/
static void aw210xx_update_cfg_array(struct aw210xx *aw210xx,
		uint8_t *p_cfg_data, uint32_t cfg_size)
{
	unsigned int i = 0;

	for (i = 0; i < cfg_size; i += 2)
		aw210xx_i2c_write(aw210xx, p_cfg_data[i], p_cfg_data[i + 1]);
}

void aw210xx_cfg_update(struct aw210xx *aw210xx)
{
	AW_LOG("aw210xx->effect = %d", aw210xx->all_leds_effect);

	aw210xx_update_cfg_array(aw210xx,
			aw210xx_all_leds_cfg_array[aw210xx->all_leds_effect].p,
			aw210xx_all_leds_cfg_array[aw210xx->all_leds_effect].count);
}

void aw210xx_uvlo_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_UVCR,
				AW210XX_BIT_UVPD_MASK,
				AW210XX_BIT_UVPD_DISENA);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_UVCR,
				AW210XX_BIT_UVDIS_MASK,
				AW210XX_BIT_UVDIS_DISENA);
	} else {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_UVCR,
				AW210XX_BIT_UVPD_MASK,
				AW210XX_BIT_UVPD_ENABLE);
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_UVCR,
				AW210XX_BIT_UVDIS_MASK,
				AW210XX_BIT_UVDIS_ENABLE);
	}
}

void aw210xx_sbmd_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR2,
				AW210XX_BIT_SBMD_MASK,
				AW210XX_BIT_SBMD_ENABLE);
		aw210xx->sdmd_flag = 1;
	} else {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR2,
				AW210XX_BIT_SBMD_MASK,
				AW210XX_BIT_SBMD_DISENA);
		aw210xx->sdmd_flag = 0;
	}
}

void aw210xx_rgbmd_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR2,
				AW210XX_BIT_RGBMD_MASK,
				AW210XX_BIT_RGBMD_ENABLE);
		aw210xx->rgbmd_flag = 1;
	} else {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR2,
				AW210XX_BIT_RGBMD_MASK,
				AW210XX_BIT_RGBMD_DISENA);
		aw210xx->rgbmd_flag = 0;
	}
}

void aw210xx_apse_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_APSE_MASK,
				AW210XX_BIT_APSE_ENABLE);
	} else {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_APSE_MASK,
				AW210XX_BIT_APSE_DISENA);
	}
}

/*****************************************************
* aw210xx led function set
*****************************************************/
int32_t aw210xx_osc_pwm_set(struct aw210xx *aw210xx)
{
	switch (aw210xx->osc_clk) {
	case CLK_FRQ_16M:
		AW_LOG("osc is 16MHz!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_16MHz);
		break;
	case CLK_FRQ_8M:
		AW_LOG("osc is 8MHz!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_8MHz);
		break;
	case CLK_FRQ_1M:
		AW_LOG("osc is 1MHz!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_1MHz);
		break;
	case CLK_FRQ_512k:
		AW_LOG("osc is 512KHz!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_512kHz);
		break;
	case CLK_FRQ_256k:
		AW_LOG("osc is 256KHz!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_256kHz);
		break;
	case CLK_FRQ_125K:
		AW_LOG("osc is 125KHz!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_125kHz);
		break;
	case CLK_FRQ_62_5K:
		AW_LOG("osc is 62.5KHz!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_62_5kHz);
		break;
	case CLK_FRQ_31_25K:
		AW_LOG("osc is 31.25KHz!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CLKFRQ_MASK,
				AW210XX_BIT_CLKFRQ_31_25kHz);
		break;
	default:
		AW_LOG("this clk_pwm is unsupported!\n");
		return -AW210XX_CLK_MODE_UNSUPPORT;
	}

	return 0;
}

int32_t aw210xx_br_res_set(struct aw210xx *aw210xx)
{
	switch (aw210xx->br_res) {
	case BR_RESOLUTION_8BIT:
		AW_LOG("br resolution select 8bit!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_PWMRES_MASK,
				AW210XX_BIT_PWMRES_8BIT);
		break;
	case BR_RESOLUTION_9BIT:
		AW_LOG("br resolution select 9bit!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_PWMRES_MASK,
				AW210XX_BIT_PWMRES_9BIT);
		break;
	case BR_RESOLUTION_12BIT:
		AW_LOG("br resolution select 12bit!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_PWMRES_MASK,
				AW210XX_BIT_PWMRES_12BIT);
		break;
	case BR_RESOLUTION_9_AND_3_BIT:
		AW_LOG("br resolution select 9+3bit!\n");
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_PWMRES_MASK,
				AW210XX_BIT_PWMRES_9_AND_3_BIT);
		break;
	default:
		AW_LOG("this br_res is unsupported!\n");
		return -AW210XX_CLK_MODE_UNSUPPORT;
	}

	return 0;
}

/*****************************************************
* aw210xx debug interface set
*****************************************************/
static void aw210xx_update(struct aw210xx *aw210xx)
{
	aw210xx_i2c_write(aw210xx, AW210XX_REG_UPDATE, AW210XX_UPDATE_BR_SL);
}

void aw210xx_global_set(struct aw210xx *aw210xx)
{
	aw210xx_i2c_write(aw210xx,
			AW210XX_REG_GCCR, aw210xx->glo_current);
}
void aw210xx_sl_set(struct aw210xx *aw210xx)
{
	aw210xx_update_cfg_array(aw210xx,aw210xx_all_sl_cfg.p,aw210xx_all_sl_cfg.count);
}
/*****************************************************
 *
 * aw210xx led cfg
 *
 *****************************************************/
static void aw210xx_brightness_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			brightness_work);

	if (aw210xx->cdev.brightness > aw210xx->cdev.max_brightness)
		aw210xx->cdev.brightness = aw210xx->cdev.max_brightness;

	aw210xx_i2c_write(aw210xx, AW210XX_REG_GCCR, aw210xx->cdev.brightness);
}

static void aw210xx_set_brightness(struct led_classdev *cdev,
		enum led_brightness brightness)
{
	struct aw210xx *aw210xx = container_of(cdev, struct aw210xx, cdev);

	aw210xx->cdev.brightness = brightness;

	schedule_work(&aw210xx->brightness_work);
}

/*****************************************************
* aw210xx basic function set
*****************************************************/
void aw210xx_chipen_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CHIPEN_MASK,
				AW210XX_BIT_CHIPEN_ENABLE);
	} else {
		aw210xx_i2c_write_bits(aw210xx,
				AW210XX_REG_GCR,
				AW210XX_BIT_CHIPEN_MASK,
				AW210XX_BIT_CHIPEN_DISENA);
	}
}

static int aw210xx_hw_enable(struct aw210xx *aw210xx, bool flag)
{
	AW_LOG("enter\n");

	if (aw210xx && gpio_is_valid(aw210xx->enable_gpio)) {
		if (flag) {
			AW_LOG("enable\n");
			gpio_set_value_cansleep(aw210xx->enable_gpio, 1);
			usleep_range(2000, 2500);
		} else {
			AW_LOG("disable\n");
			gpio_set_value_cansleep(aw210xx->enable_gpio, 0);
		}
	} else {
		AW_ERR("failed\n");
	}

	return 0;
}

static int32_t aw210xx_group_gcfg_set(struct aw210xx *aw210xx, bool flag)
{
	if (flag) {
		switch (aw210xx->chipid) {
		case AW21018_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21018_GROUP_ENABLE);
			return 0;
		case AW21012_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21012_GROUP_ENABLE);
			return 0;
		case AW21009_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21009_GROUP_ENABLE);
			return 0;
		default:
			AW_LOG("%s: chip is unsupported device!\n",
			       __func__);
			return -AW210XX_CHIPID_FAILD;
		}
	} else {
		switch (aw210xx->chipid) {
		case AW21018_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21018_GROUP_DISABLE);
			return 0;
		case AW21012_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21012_GROUP_DISABLE);
			return 0;
		case AW21009_CHIPID:
			aw210xx_i2c_write(aw210xx, AW210XX_REG_GCFG,
					  AW21009_GROUP_DISABLE);
			return 0;
		default:
			AW_LOG("%s: chip is unsupported device!\n",
			       __func__);
			return -AW210XX_CHIPID_FAILD;
		}
	}
}

static int aw210xx_led_init(struct aw210xx *aw210xx)
{
	int ret = 0;
	int gpio19 = 0;
	int gpio21 = 0;
	AW_LOG("enter\n");

	if(gpio_is_valid(355)){
		ret = devm_gpio_request_one(aw210xx->dev, 355, GPIOF_DIR_IN, "hwidgp19");
		if(ret){
			AW_LOG("%s gpio19 already request\n", __func__);
		}

		gpio19 = gpio_get_value_cansleep(355);
		AW_INFO("%s gpio19:%d\n", __func__, gpio19);
	}

	if(gpio_is_valid(357)){
		ret = devm_gpio_request_one(aw210xx->dev, 357, GPIOF_DIR_IN, "hwidgp19");
		if(ret){
			AW_LOG("%s gpio21 already request\n", __func__);
		}

		gpio21 = gpio_get_value_cansleep(357);
		AW_INFO("%s gpio21:%d\n", __func__, gpio21);
	}

	/* gpio19,gpio21:00->T0, 01->EVT, 10->DVT, 11->PVT*/
	if((gpio19 == 0) && (gpio21 == 0)){
		/*aw210xx->glo_current is 32 config in dtsi */
		aw210xx_all_sl_cfg.p = aw210xx_all_sl_cfg_t0.p;
		aw210xx_all_sl_cfg.count= aw210xx_all_sl_cfg_t0.count;
	}else{
		aw210xx->glo_current = 160;
		aw210xx_all_sl_cfg.p = aw210xx_all_sl_cfg_evt.p;
		aw210xx_all_sl_cfg.count= aw210xx_all_sl_cfg_evt.count;
	}

	aw210xx->setting_br = 2625;
	aw210xx->sdmd_flag = 0;
	aw210xx->rgbmd_flag = 0;
	/* chip enable */
	aw210xx_chipen_set(aw210xx, true);
	/* sbmd enable */
	aw210xx_sbmd_set(aw210xx, false);
	/* rgbmd enable */
	aw210xx_rgbmd_set(aw210xx, false);
	/* clk_pwm selsect */
	aw210xx_osc_pwm_set(aw210xx);
	/* br_res select */
	aw210xx_br_res_set(aw210xx);
	/* global set */
	aw210xx_global_set(aw210xx);
	/* under voltage lock out */
	aw210xx_uvlo_set(aw210xx, true);
	/* apse enable */
	aw210xx_apse_set(aw210xx, true);
	/* uvlo set */
	aw210xx_uvlo_set(aw210xx, true);
	/* group set disable */
	aw210xx_group_gcfg_set(aw210xx, false);
	/*  set sl (1-18) */
	aw210xx_sl_set(aw210xx);
	return 0;
}

uint16_t aw210xx_single_led_br_get(struct aw210xx *aw210xx,
		uint8_t led_num)
{
	uint16_t brightness = 0;
	uint8_t br_l = 0;
	uint8_t br_h = 0;

	if (aw210xx->sdmd_flag == 1) {
		if (aw210xx->rgbmd_flag == 0) {
			aw210xx_i2c_read(aw210xx,
					  AW210XX_REG_BR00L +led_num-1,
					  (unsigned char *)&brightness);
		}
	} else {
		if (aw210xx->rgbmd_flag == 0) {
			aw210xx_i2c_read(aw210xx,
					  AW210XX_REG_BR00L + (led_num-1)*2,
					   (unsigned char *)&br_l);
			aw210xx_i2c_read(aw210xx,
					  AW210XX_REG_BR00H + (led_num-1)*2,
					 (unsigned char *) &br_h);
			brightness = (br_h << 8) | br_l;
		}
	}

	return brightness;
}
void aw210xx_single_led_br_set(struct aw210xx *aw210xx,
		uint8_t led_num,
		uint16_t brightness)
{
	/* br set */
	if (aw210xx->sdmd_flag == 1) {
		if (aw210xx->rgbmd_flag == 0) {
			aw210xx_i2c_write(aw210xx,
					  AW210XX_REG_BR00L +led_num-1,
					  brightness);
		}
	} else {
		if (aw210xx->rgbmd_flag == 0) {
			aw210xx_i2c_write(aw210xx,
					  AW210XX_REG_BR00L + (led_num-1)*2,
					  brightness&0xff);
			aw210xx_i2c_write(aw210xx,
					  AW210XX_REG_BR00H + (led_num-1)*2,
					  (brightness&0x0f00)>>8);
		}
	}
}

void aw210xx_round_leds_br_set(struct aw210xx *aw210xx,
		uint16_t brightness)
{
	uint8_t led_num = 0;

	for(led_num = 2; led_num <=5; led_num++){
		aw210xx_single_led_br_set(aw210xx, led_num, brightness);
	}
}

void aw210xx_horse_race_leds_br_set(struct aw210xx *aw210xx,int start,
		uint16_t brightness)
{
	uint8_t led_num = 0;
	int led[9] ={16,13,11,9,12,10,14,15,8};

	for(led_num = start; led_num < 9; led_num++){
		aw210xx_single_led_br_set(aw210xx, led[led_num], brightness);
	}
}
void aw210xx_vline_leds_br_set(struct aw210xx *aw210xx,
		uint16_t brightness)
{
	uint8_t led_num = 0;
	int led[8] ={13,11,9,12,10,14,15,8};

	for(led_num = 0; led_num < 8; led_num++){
		aw210xx_single_led_br_set(aw210xx, led[led_num], brightness);
	}
}

void aw210xx_all_white_leds_br_set(struct aw210xx *aw210xx,
		uint16_t brightness)
{
	uint8_t led_num = 0;

	for(led_num = 1; led_num <=16; led_num++){
		if( led_num == 6)
			continue;

		aw210xx_single_led_br_set(aw210xx, led_num, brightness);
	}
}

/*****************************************************
* open short detect
*****************************************************/
void aw210xx_open_detect_cfg(struct aw210xx *aw210xx)
{
	/*enable open detect*/
	aw210xx_i2c_write(aw210xx, AW210XX_REG_OSDCR, AW210XX_OPEN_DETECT_EN);
	/*set DCPWM = 1*/
	aw210xx_i2c_write_bits(aw210xx, AW210XX_REG_SSCR,
							AW210XX_DCPWM_SET_MASK,
							AW210XX_DCPWM_SET);
	/*set Open threshold = 0.2v*/
	aw210xx_i2c_write_bits(aw210xx,
							AW210XX_REG_OSDCR,
							AW210XX_OPEN_THRESHOLD_SET_MASK,
							AW210XX_OPEN_THRESHOLD_SET);
}

void aw210xx_short_detect_cfg(struct aw210xx *aw210xx)
{

	/*enable short detect*/
	aw210xx_i2c_write(aw210xx, AW210XX_REG_OSDCR, AW210XX_SHORT_DETECT_EN);
	/*set DCPWM = 1*/
	aw210xx_i2c_write_bits(aw210xx, AW210XX_REG_SSCR,
							AW210XX_DCPWM_SET_MASK,
							AW210XX_DCPWM_SET);
	/*set Short threshold = 1v*/
	aw210xx_i2c_write_bits(aw210xx,
							AW210XX_REG_OSDCR,
							AW210XX_SHORT_THRESHOLD_SET_MASK,
							AW210XX_SHORT_THRESHOLD_SET);
}

void aw210xx_open_short_dis(struct aw210xx *aw210xx)
{
	aw210xx_i2c_write(aw210xx, AW210XX_REG_OSDCR, AW210XX_OPEN_SHORT_DIS);
	/*SET DCPWM = 0*/
	aw210xx_i2c_write_bits(aw210xx, AW210XX_REG_SSCR,
							AW210XX_DCPWM_SET_MASK,
							AW210XX_DCPWM_CLEAN);
}
void aw210xx_open_short_detect(struct aw210xx *aw210xx,
										int32_t detect_flg, u8 *reg_val)
{
	/*config for open shor detect*/
	if (detect_flg == AW210XX_OPEN_DETECT)
		aw210xx_open_detect_cfg(aw210xx);
	else if (detect_flg == AW210XX_SHORT_DETECT)
		aw210xx_short_detect_cfg(aw210xx);
	/*read detect result*/
	aw210xx_i2c_read(aw210xx, AW210XX_REG_OSST0, &reg_val[0]);
	aw210xx_i2c_read(aw210xx, AW210XX_REG_OSST1, &reg_val[1]);
	aw210xx_i2c_read(aw210xx, AW210XX_REG_OSST2, &reg_val[2]);
	/*close for open short detect*/
	aw210xx_open_short_dis(aw210xx);
}

void aw210xx_random_led(struct aw210xx *aw210xx)
{
	unsigned int num = 0;
	int brightness = aw210xx->setting_br;

	get_random_bytes(&num,sizeof(unsigned int));
	num = num%16 +1;

	aw210xx_single_led_br_set(aw210xx, num, brightness);
	aw210xx_update(aw210xx);

	usleep_range(aw210xx->random_delay_t, aw210xx->random_delay_t+1);

	aw210xx_single_led_br_set(aw210xx, num, 0);
	aw210xx_update(aw210xx);

}

/*start:start on led num; state 1:horse race, 2:on, 3:off*/
void aw210xx_charging_leds_horse_race(struct aw210xx *aw210xx, int start, int state)
{
	int led[9] ={16,13,11,9,12,10,14,15,8};
	int num =0;
	int brightness = 0;

	if(state ==1){
		brightness =aw210xx->setting_br;
		for(num = start; num <9; num++){
			if(aw210xx->wired_charging == 0){
				return;
			}
			aw210xx_single_led_br_set(aw210xx, led[num], brightness);
			aw210xx_update(aw210xx);
			usleep_range(500000, 500000);
		}
	}else if(state ==2){
		brightness =aw210xx->setting_br;
		aw210xx_horse_race_leds_br_set(aw210xx,start,brightness);
		aw210xx_update(aw210xx);
	}else if(state ==0){
		brightness =0;
		aw210xx_horse_race_leds_br_set(aw210xx,start,brightness);
		aw210xx_update(aw210xx);
	}

}
void aw210xx_round_leds_on(struct aw210xx *aw210xx, int state)
{
	int brightness = 0;

	if(state ==0){
		brightness= 0;
	}else if(state ==1){
		brightness= aw210xx->setting_br;
	}

	aw210xx_round_leds_br_set(aw210xx,brightness);
	aw210xx_update(aw210xx);
}
/*led_group:0 round 1:font camera*/
void aw210xx_leds_breath(struct aw210xx *aw210xx,int led_group)
{
	int update_frame_idx = 0;
	uint16_t brightness = 0;
	ALGO_DATA_STRUCT aw210xx_algo_data;
	AW_MULTI_BREATH_DATA_STRUCT data;
	unsigned char breath_cur_phase = 0;
	unsigned char breath_cur_loop = 0;
	unsigned char breath_phase_nums = 5;

	aw210xx_leds_breath_data.fadeh = aw210xx->setting_br;
	breath_loop_end = 0;
	data = aw210xx_leds_breath_data;

	breath_cur_phase = 0;
	aw210xx_algo_data.cur_frame = 0;
	aw210xx_algo_data.total_frames = (data.time[0] + 19) / 20 + 1;
	aw210xx_algo_data.data_start = data.fadel;
	aw210xx_algo_data.data_end = data.fadeh;

	if(led_group == 0){
		aw210xx_round_leds_br_set(aw210xx,0);
	}else if(led_group == 1){
		aw210xx_single_led_br_set(aw210xx, 1, 0);
	}else if(led_group ==2 ){
		aw210xx_all_white_leds_br_set(aw210xx, 0);
	}
	aw210xx_update(aw210xx);

	while (1) {
		usleep_range(50000, 50000);
		update_frame_idx = 1;
		aw210xx_algo_data.cur_frame++;
		if (aw210xx_algo_data.cur_frame >= aw210xx_algo_data.total_frames) {
			aw210xx_algo_data.cur_frame = 0;
			breath_cur_phase++;
			if (breath_cur_phase >= breath_phase_nums) {
				breath_cur_phase = 1;
				if (0 == data.repeat_nums)
					breath_cur_loop = 0;
				else if (breath_cur_loop >= (data.repeat_nums - 1))
					update_frame_idx = 0;
				else
					breath_cur_loop++;
			}

			if (update_frame_idx) {
				aw210xx_algo_data.total_frames =
					(data.time[breath_cur_phase])/20 + 1;
				if (breath_cur_phase== 1) {
					aw210xx_algo_data.data_start = data.fadel;
					aw210xx_algo_data.data_end = data.fadeh;
				} else if (breath_cur_phase == 2) {
					aw210xx_algo_data.data_start = data.fadeh;
					aw210xx_algo_data.data_end = data.fadeh;
				} else if (breath_cur_phase == 3) {
					aw210xx_algo_data.data_start = data.fadeh;
					aw210xx_algo_data.data_end = data.fadel;
				} else {
					aw210xx_algo_data.data_start = data.fadel;
					aw210xx_algo_data.data_end = data.fadel;
				}
				/* breath_cur_phase++; */
			} else {
				aw210xx_algo_data.cur_frame = 0;
				aw210xx_algo_data.total_frames = 1;
				aw210xx_algo_data.data_start = 0;
				aw210xx_algo_data.data_end = 0;
				breath_loop_end=1;
			}
		}

		brightness = aw210xx_algorithm_get_correction(&aw210xx_algo_data);

		if (breath_cur_phase == 0)
			brightness = 0;

		if(breath_cur_phase ==2)
			brightness = data.fadeh;

		if(led_group == 0){
			aw210xx_round_leds_br_set(aw210xx,brightness);
		}else if(led_group == 1){
			aw210xx_single_led_br_set(aw210xx, 1, brightness);
		}else if(led_group == 2){
			aw210xx_all_white_leds_br_set(aw210xx, brightness);
		}

		aw210xx_update(aw210xx);

		if(breath_loop_end == 1) break;
	}

	if(led_group == 0){
		aw210xx_round_leds_br_set(aw210xx,0);
	}else if(led_group == 1){
		aw210xx_single_led_br_set(aw210xx, 1, 0);
	}else if(led_group == 2){
		aw210xx_all_white_leds_br_set(aw210xx, 0);
	}
	aw210xx_update(aw210xx);
}

void aw210xx_round_leds_breath(struct aw210xx *aw210xx)
{
	aw210xx_leds_breath(aw210xx,0);
}

void aw210xx_front_cam_led_breath(struct aw210xx *aw210xx)
{
	aw210xx_leds_breath(aw210xx,1);
}

void aw210xx_all_white_leds_breath(struct aw210xx *aw210xx)
{
	aw210xx_leds_breath(aw210xx,2);
}
/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw210xx_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint32_t databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		if (aw210xx_reg_access[(uint8_t)databuf[0]] & REG_WR_ACCESS)
			aw210xx_i2c_write(aw210xx, (uint8_t)databuf[0],
					(uint8_t)databuf[1]);
	}

	return len;
}

static ssize_t aw210xx_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;
	uint8_t br_max = 0;
	uint8_t sl_val = 0;

	aw210xx_i2c_read(aw210xx, AW210XX_REG_GCR, &reg_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"reg:0x%02x=0x%02x\n", AW210XX_REG_GCR, reg_val);
	switch (aw210xx->chipid) {
	case AW21018_CHIPID:
		br_max = AW210XX_REG_BR17H;
		sl_val = AW210XX_REG_SL17;
		break;
	case AW21012_CHIPID:
		br_max = AW210XX_REG_BR11H;
		sl_val = AW210XX_REG_SL11;
		break;
	case AW21009_CHIPID:
		br_max = AW210XX_REG_BR08H;
		sl_val = AW210XX_REG_SL08;
		break;
	default:
		AW_LOG("chip is unsupported device!\n");
		return len;
	}

	for (i = AW210XX_REG_BR00L; i <= br_max; i++) {
		if (!(aw210xx_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw210xx_i2c_read(aw210xx, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	for (i = AW210XX_REG_SL00; i <= sl_val; i++) {
		if (!(aw210xx_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw210xx_i2c_read(aw210xx, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	for (i = AW210XX_REG_GCCR; i <= AW210XX_REG_GCFG; i++) {
		if (!(aw210xx_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw210xx_i2c_read(aw210xx, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}

	return len;
}

static ssize_t aw210xx_hwen_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val > 0)
		aw210xx_hw_enable(aw210xx, true);
	else
		aw210xx_hw_enable(aw210xx, false);

	return len;
}

static ssize_t aw210xx_hwen_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "hwen=%d\n",
			gpio_get_value(aw210xx->enable_gpio));
	return len;
}

static ssize_t aw210xx_glo_current_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw210xx->glo_current = (uint32_t)val;
	aw210xx_global_set(aw210xx);

	return len;
}

static ssize_t aw210xx_glo_current_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);

	return sprintf(buf, "%d\n", aw210xx->glo_current);
}

static ssize_t aw210xx_setting_br_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw210xx->setting_br = (uint16_t)val;
	AW_INFO("aw210xx->setting_br:%d\n",aw210xx->setting_br);
	return len;
}

static ssize_t aw210xx_setting_br_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);

	return sprintf(buf, "%d\n", aw210xx->setting_br);
}

static ssize_t aw210xx_single_led_br_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint8_t led_num = 0;
	uint16_t brightness = 0;

	if (sscanf(buf, "%d %d", &led_num, & brightness) == 2) {
		if (aw210xx->chipid == AW21018_CHIPID) {
			if (led_num > AW21018_LED_NUM)
				led_num = AW21018_LED_NUM;
		} else if (aw210xx->chipid == AW21012_CHIPID) {
			if (led_num > AW21012_LED_NUM)
				led_num = AW21012_LED_NUM;
		} else {
			if (led_num > AW21009_LED_NUM)
				led_num = AW21009_LED_NUM;
		}
		aw210xx_single_led_br_set(aw210xx, led_num, brightness);
		/* update */
		aw210xx_update(aw210xx);
	}

	return len;
}

static ssize_t aw210xx_rear_cam_led_br_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint8_t led_num = 7;
	uint16_t brightness;

	brightness = aw210xx_single_led_br_get(aw210xx, led_num);

	return sprintf(buf, "%d\n", brightness);
}

static ssize_t aw210xx_rear_cam_led_br_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint8_t led_num = 7;
	uint16_t brightness = 0;

	if (sscanf(buf, "%d", & brightness) == 1) {
		aw210xx_single_led_br_set(aw210xx, led_num, brightness);
		aw210xx_update(aw210xx);
	}

	return len;
}

static ssize_t aw210xx_front_cam_led_br_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint8_t led_num = 1;
	uint16_t brightness;

	brightness = aw210xx_single_led_br_get(aw210xx, led_num);

	return sprintf(buf, "%d\n", brightness);
}

static ssize_t aw210xx_front_cam_led_br_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint8_t led_num = 1;
	uint16_t brightness = 0;

	if (sscanf(buf, "%d", & brightness) == 1) {
		aw210xx_single_led_br_set(aw210xx, led_num, brightness);
		aw210xx_update(aw210xx);
	}

	return len;
}

static ssize_t aw210xx_horse_race_leds_br_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);

	return sprintf(buf, "%d\n", aw210xx->horse_race_leds_br);
}

static ssize_t aw210xx_horse_race_leds_br_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint16_t brightness = 0;

	if (sscanf(buf, "%d", & brightness) == 1) {
		aw210xx_horse_race_leds_br_set(aw210xx,0,brightness);
		aw210xx_update(aw210xx);
		aw210xx->horse_race_leds_br = brightness;
	}

	return len;
}

static ssize_t aw210xx_round_leds_br_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);

	return sprintf(buf, "%d\n", aw210xx->round_leds_br);
}

static ssize_t aw210xx_round_leds_br_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint16_t brightness = 0;

	if (sscanf(buf, "%d", & brightness) == 1) {
		aw210xx_round_leds_br_set(aw210xx,brightness);
		aw210xx_update(aw210xx);
		aw210xx->round_leds_br = brightness;
	}

	return len;
}

static ssize_t aw210xx_vline_leds_br_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);

	return sprintf(buf, "%d\n", aw210xx->vline_leds_br);
}

static ssize_t aw210xx_vline_leds_br_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint16_t brightness = 0;

	if (sscanf(buf, "%d", & brightness) == 1) {
		aw210xx_vline_leds_br_set(aw210xx,brightness);
		aw210xx_update(aw210xx);
		aw210xx->vline_leds_br = brightness;
	}

	return len;
}
static ssize_t aw210xx_dot_led_br_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint8_t led_num = 16;
	uint16_t brightness;

	brightness = aw210xx_single_led_br_get(aw210xx, led_num);

	return sprintf(buf, "%d\n", brightness);
}

static ssize_t aw210xx_dot_led_br_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint8_t led_num = 16;
	uint16_t brightness = 0;

	if (sscanf(buf, "%d", & brightness) == 1) {
		aw210xx_single_led_br_set(aw210xx, led_num, brightness);
		aw210xx_update(aw210xx);
	}

	return len;
}

static ssize_t aw210xx_all_white_leds_br_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);

	return sprintf(buf, "%d\n", aw210xx->all_white_leds_br);
}

static ssize_t aw210xx_all_white_leds_br_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	uint16_t brightness = 0;

	if (sscanf(buf, "%d", & brightness) == 1) {
		aw210xx_all_white_leds_br_set(aw210xx,brightness);
		aw210xx_update(aw210xx);

		aw210xx->all_white_leds_br = brightness;
	}

	return len;
}

static ssize_t aw210xx_all_leds_effect_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	ssize_t len = 0;
	unsigned int i;

	for (i = 0; i < (sizeof(aw210xx_all_leds_cfg_array) /
			sizeof(aw210xx_cfg_t)); i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "effect[%d]: %pf\n",
				i, aw210xx_all_leds_cfg_array[i].p);
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "current effect[%d]: %pf\n",
			aw210xx->all_leds_effect, aw210xx_all_leds_cfg_array[aw210xx->all_leds_effect].p);
	return len;
}
static ssize_t aw210xx_all_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;
	if ((val >= (sizeof(aw210xx_all_leds_cfg_array) /
			sizeof(aw210xx_cfg_t))) || (val < 0)) {
		pr_err("%s, store effect num error.\n", __func__);
		return -EINVAL;
	}

	aw210xx->all_leds_effect = val;
	pr_info("%s, line%d,val = %d\n", __func__, __LINE__, val);
	aw210xx_cfg_update(aw210xx);

	return len;
}

static ssize_t aw210xx_round_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;

	if (val ==1){
		aw210xx_round_leds_breath(aw210xx);//breath
	}else if(val ==2){
		aw210xx_round_leds_on(aw210xx,1);//on
	}else if(val ==0){
		aw210xx_round_leds_on(aw210xx,0);//off
	}
	return len;
}
static ssize_t aw210xx_leds_breath_set_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	int repeat_nums;
	//int time1,time2,time3,time4,repeat_nums,fadeh;

	if (sscanf(buf, "%d", &repeat_nums) == 1) {
		aw210xx_leds_breath_data.repeat_nums = repeat_nums;
	}

#if 0
	if (sscanf(buf, "%d %d %d %d %d %d %d", &time1, &time2, &time3, &time4, &repeat_nums,&fadeh) == 6) {
		aw210xx_leds_breath_data.time[1] = time1;
		aw210xx_leds_breath_data.time[2] = time2;
		aw210xx_leds_breath_data.time[3] = time3;
		aw210xx_leds_breath_data.time[4] = time4;
		aw210xx_leds_breath_data.repeat_nums = repeat_nums;
		aw210xx_leds_breath_data.fadeh = fadeh;
	}
#endif

	return len;
}

static ssize_t aw210xx_horse_race_leds_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;

	if(val ==2){
		aw210xx_charging_leds_horse_race(aw210xx, 0, 2);//on
	}else if(val ==0){
		aw210xx_charging_leds_horse_race(aw210xx, 0, 0);//off
	}
	return len;
}

/* add for wired charging start*/
static ssize_t aw210xx_wired_charging_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);

	return sprintf(buf, "%d\n", aw210xx->wired_charging);
}

static ssize_t aw210xx_wired_charging_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int state =0, bat_state =0, bat_val =0;

	mutex_lock(&aw210xx->led_mutex);
	if (sscanf(buf, "%d %d %d", &state, &bat_state, &bat_val) == 3) {
		AW_INFO("state:%d, bat_state:%d, bat_val:%d\n", state, bat_state, bat_val);
		if(aw210xx->wired_charging != ((state & 0xFF) |((bat_val&0xFF)<<16))){
			if((state ==1) && ((aw210xx->wired_charging & 0xFF) != state)){
				aw210xx->wired_charging  = (state & 0xFF) |((bat_val&0xFF)<<16);
				queue_work(aw210xx->leds_workqueue, &aw210xx->wired_charging_work);
			}else if((state ==0) && ((aw210xx->wired_charging & 0xFF) != state)){
				aw210xx->wired_charging = 0;
				cancel_work_sync(&aw210xx->wired_charging_work);
				aw210xx_all_white_leds_br_set(aw210xx, 0);
				aw210xx_update(aw210xx);
			}
		}
	}
	mutex_unlock(&aw210xx->led_mutex);
	return len;
}

static ssize_t aw210xx_wlr_charging_leds_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw210xx->led_mutex);
	AW_INFO("aw210xx->wlr_charging: %d val: %d\n", aw210xx->wlr_charging, val);
	if(aw210xx->wlr_charging != val){
		if (val ==1){
			aw210xx->wlr_charging = 1;
			queue_work(aw210xx->leds_workqueue, &aw210xx->wlr_charging_work);
		}else if(val ==0){
			aw210xx->wlr_charging = 0;
			general_effect_end = 1;
			//breath_loop_end = 1;
			cancel_work_sync(&aw210xx->wlr_charging_work);
			aw210xx_all_white_leds_br_set(aw210xx, 0);
			aw210xx_update(aw210xx);
		}
	}
	mutex_unlock(&aw210xx->led_mutex);
	return len;
}

static ssize_t aw210xx_random_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;

	aw210xx->random_delay_t = val;
	return len;
}

static ssize_t aw210xx_random_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int val=0;

	if (sscanf(buf, "%d", &val) == 1) {
		if (val ==1){
			aw210xx->random_effect = 1;
			queue_work(aw210xx->leds_workqueue, &aw210xx->random_effect_work);
		}else if(val ==0){
			aw210xx->random_effect = 0;
			aw210xx->random_delay_t = 0;
			cancel_work_sync(&aw210xx->random_effect_work);
			aw210xx_all_white_leds_br_set(aw210xx, 0);
			aw210xx_update(aw210xx);
		}
	}
	return len;
}

static ssize_t aw210xx_keybd_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int val=0;
	if (sscanf(buf, "%d", &val) == 1) {
		if (val ==1){
			aw210xx->random_delay_t = 50000;
			aw210xx_random_led(aw210xx);
		}
	}
	return len;
}

static ssize_t aw210xx_music_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int f_cam_led_br=0, r_cam_led_br=0, round_leds_br=0, vline_leds_br=0, dot_led_br=0;
	int led1_br = 0, led2_br = 0, led3_br = 0, led4_br = 0, led5_br = 0;

	mutex_lock(&aw210xx->led_mutex);
	if (sscanf(buf, "%d %d %d %d %d",
		&r_cam_led_br, &f_cam_led_br, &round_leds_br, &vline_leds_br, &dot_led_br) == 5) {

		if((aw210xx->music_effect.r_cam_led_br != r_cam_led_br)  || (aw210xx->music_effect.f_cam_led_br != f_cam_led_br)
			||(aw210xx->music_effect.round_leds_br != round_leds_br) ||(aw210xx->music_effect.vline_leds_br != vline_leds_br)
			||(aw210xx->music_effect.dot_led_br != dot_led_br)){

			aw210xx->music_effect.r_cam_led_br = r_cam_led_br;
			aw210xx->music_effect.f_cam_led_br = f_cam_led_br;
			aw210xx->music_effect.round_leds_br = round_leds_br;
			aw210xx->music_effect.vline_leds_br = vline_leds_br;
			aw210xx->music_effect.dot_led_br = dot_led_br;

			led1_br = r_cam_led_br * aw210xx->setting_br /4095;
			led2_br = f_cam_led_br * aw210xx->setting_br /4095;
			led3_br = round_leds_br * aw210xx->setting_br /4095;
			led4_br = vline_leds_br * aw210xx->setting_br /4095;
			led5_br = dot_led_br * aw210xx->setting_br /4095;

			aw210xx_single_led_br_set(aw210xx, 7, led1_br);
			aw210xx_single_led_br_set(aw210xx, 1, led2_br);
			aw210xx_round_leds_br_set(aw210xx, led3_br);
			aw210xx_vline_leds_br_set(aw210xx, led4_br);
			aw210xx_single_led_br_set(aw210xx, 16, led5_br);
			aw210xx_update(aw210xx);
		}
	}
	mutex_unlock(&aw210xx->led_mutex);
	return len;
}
static ssize_t aw210xx_video_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;

	AW_INFO("aw210xx->video_effect: %d val: %d\n", aw210xx->video_effect, val);
	if(aw210xx->video_effect != val){
		if (val == 1){
			aw210xx->video_effect = 1;
			queue_work(aw210xx->video_workqueue, &aw210xx->video_effect_work);
		}else if(val == 0){
			aw210xx->video_effect = 0;
			cancel_work_sync(&aw210xx->video_effect_work);
			aw210xx_single_led_br_set(aw210xx, 17, 0);
			aw210xx_update(aw210xx);
		}
	}
	return len;
}

static ssize_t aw210xx_ga_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc =kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw210xx->led_mutex);
	AW_INFO("aw210xx->ga_effect: %#X, val: %#X\n", aw210xx->ga_effect, val);
	if( aw210xx->ga_effect != val)
	{
		/*wakeup*/
		if((aw210xx->ga_effect & 0xFF) != (val & 0xFF ) ){
			aw210xx->ga_effect &= 0xFFFF00;
			aw210xx->ga_effect |= val & 0xFF ;
			AW_INFO("wakeup aw210xx->ga_effect: %#X\n", aw210xx->ga_effect);
			if(aw210xx->ga_effect  & 0xFF){
				aw210xx_round_leds_on(aw210xx, 1);
				usleep_range(500000, 500000);
				aw210xx_round_leds_on(aw210xx, 0);
			}
		}

		/*listen*/
		if(((aw210xx->ga_effect >>8) & 0xFF)  != ((val>>8) & 0xFF)){
			aw210xx->ga_effect &= 0xFF00FF;
			aw210xx->ga_effect |= (val & 0x00FF00) ;
			AW_INFO("listen aw210xx->ga_effect: %#X\n", aw210xx->ga_effect);

			if((aw210xx->ga_effect  >>8) & 0xFF){
				AW_INFO("listen queue work\n");
				queue_work(aw210xx->leds_workqueue, &aw210xx->ga_effect_work);
			}
			else{
				AW_INFO("listen cancel work\n");
				breath_loop_end = 1;
				cancel_work_sync(&aw210xx->ga_effect_work);
				aw210xx_leds_breath_data.repeat_nums =1;
				aw210xx_round_leds_on(aw210xx, 0);
			}
		}
	}
	mutex_unlock(&aw210xx->led_mutex);
	return len;
}

static ssize_t aw210xx_nf_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc =kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw210xx->led_mutex);
	AW_INFO("aw210xx->nf_effect: %d val: %d\n", aw210xx->nf_effect, val);
	if(aw210xx->nf_effect != val){
		if(val != 0)
		{
			aw210xx->nf_effect = val;
			queue_work(aw210xx->leds_workqueue, &aw210xx->nf_effect_work);
		}else if(val == 0){
			if((aw210xx->nf_effect -1) >= (sizeof(aw210xx_notification_leds_effect)/sizeof(aw210xx_effect_cfg_t))){
				aw210xx->nf_effect = 0;
				breath_loop_end = 1;
			}
			aw210xx->nf_effect = 0;
			cancel_work_sync(&aw210xx->nf_effect_work);
			aw210xx_all_white_leds_br_set(aw210xx, 0);
			aw210xx_update(aw210xx);
		}
	}
	mutex_unlock(&aw210xx->led_mutex);
	return len;
}

static ssize_t aw210xx_bootan_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc =kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;
	AW_INFO("val: %d\n", val);

	mutex_lock(&aw210xx->led_mutex);
	if(val == 1)
	{
		char color[6] = {0};
		aw210xx_read_device_color("/mnt/vendor/persist/color", color, 5);
		if(strcmp(color, "white") ==0){
			AW_INFO("device color is white\n");
			dev_is_black = 0;
			aw210xx->glo_current = 160;
			aw210xx->setting_br = 688;
		}else{
			AW_INFO("device color is black\n");
			dev_is_black = 1;
			aw210xx->glo_current = 160;
			aw210xx->setting_br = 2625;
		}
	}
	#if 0
		aw210xx->bootan_effect = 1;
		queue_work(aw210xx->leds_workqueue, &aw210xx->bootan_effect_work);
	}else if(val == 0){
		aw210xx->bootan_effect = 0;
		breath_loop_end = 1;
		cancel_work_sync(&aw210xx->bootan_effect_work);
		aw210xx_all_white_leds_br_set(aw210xx, 0);
		aw210xx_update(aw210xx);
	}
	#endif
	mutex_unlock(&aw210xx->led_mutex);
	return len;
}

static ssize_t aw210xx_rear_cam_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int state=0, r_cam_led_br=0, all_white_leds_br=0;
	uint16_t brightness;
	mutex_lock(&aw210xx->led_mutex);
	if (sscanf(buf, "%d %d %d", &state, &r_cam_led_br, &all_white_leds_br) == 3){
		AW_INFO("aw210xx->r_cam_effect.state:%d, state:%d, r_cam_led_br:%d, all_white_leds_br:%d\n" ,
				aw210xx->r_cam_effect.state, state, r_cam_led_br, all_white_leds_br);

		if(state ==1)
		{
			aw210xx->r_cam_effect.state = state;
			aw210xx->r_cam_effect.r_cam_led_br= r_cam_led_br;
			aw210xx->r_cam_effect.all_white_leds_br= all_white_leds_br;
			if(dev_is_black == 1){
				brightness = 3840;
			}else if(dev_is_black == 0){
				brightness = 1306;
			}
			if((aw210xx->r_cam_effect.r_cam_led_br ==0)&&(aw210xx->r_cam_effect.all_white_leds_br ==0)){
				aw210xx_all_white_leds_br_set(aw210xx, 0);
			}else if((aw210xx->r_cam_effect.r_cam_led_br ==0)&&(aw210xx->r_cam_effect.all_white_leds_br ==1)){
				aw210xx_all_white_leds_br_set(aw210xx, brightness);
			}else if((aw210xx->r_cam_effect.r_cam_led_br ==1)&&(aw210xx->r_cam_effect.all_white_leds_br ==0)){
				aw210xx_single_led_br_set(aw210xx, 7, brightness);
			}else if((aw210xx->r_cam_effect.r_cam_led_br ==1)&&(aw210xx->r_cam_effect.all_white_leds_br ==1)){
				aw210xx_all_white_leds_br_set(aw210xx, brightness);
			}
			aw210xx_update(aw210xx);
		}else if(state == 0){
			if(aw210xx->r_cam_effect.state != 0){
				aw210xx_all_white_leds_br_set(aw210xx, 0);
				aw210xx_update(aw210xx);
				aw210xx->r_cam_effect.state = 0;
				aw210xx->r_cam_effect.r_cam_led_br = 0;
				aw210xx->r_cam_effect.all_white_leds_br = 0;
			}
		}
	}
	mutex_unlock(&aw210xx->led_mutex);
	return len;
}

static ssize_t aw210xx_ringtone_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d \n", rt_flush_delay);
}

static ssize_t aw210xx_ringtone_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	int delay;

	if (sscanf(buf, "%d", &delay) == 1){
		rt_flush_delay = delay;
	}
	return len;
}

static ssize_t aw210xx_ringtone_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc =kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw210xx->led_mutex);
	AW_INFO("aw210xx->ringtone_effect: %d,val: %d\n", aw210xx->ringtone_effect, val);
	if(aw210xx->ringtone_effect != val){
		if(val != 0)
		{
			aw210xx->ringtone_effect = val;
			queue_work(aw210xx->leds_workqueue, &aw210xx->ringtone_effect_work);
		}else if(val == 0){
			aw210xx->ringtone_effect = 0;
			cancel_work_sync(&aw210xx->ringtone_effect_work);
			aw210xx_all_white_leds_br_set(aw210xx, 0);
			aw210xx_update(aw210xx);
		}
	}
	mutex_unlock(&aw210xx->led_mutex);
	return len;
}

static ssize_t aw210xx_flip_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int rc;
	unsigned int val = 0;

	rc = kstrtouint(buf, 10, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw210xx->led_mutex);
	AW_INFO("aw210xx->flip_effect: %d val: %d\n", aw210xx->flip_effect, val);
	if(aw210xx->flip_effect != val){
		if (val == 1){
			aw210xx->flip_effect = 1;
			queue_work(aw210xx->leds_workqueue, &aw210xx->flip_effect_work);
		}else if(val ==0){
			aw210xx->flip_effect = 0;
			general_effect_end = 1;
			//breath_loop_end = 1;
			cancel_work_sync(&aw210xx->flip_effect_work);
			aw210xx_all_white_leds_br_set(aw210xx, 0);
			aw210xx_update(aw210xx);
		}
	}
	mutex_unlock(&aw210xx->led_mutex);
	return len;
}

static ssize_t aw210xx_exclamation_leds_effect_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	int led[9] ={16,13,11,9,12,10,14,15,8};
	int led_br[9] = {0};
	int num =0;
	int state =0;

	AW_INFO("enter\n");

	mutex_lock(&aw210xx->led_mutex);
	if (sscanf(buf, "%d %d %d %d %d %d %d %d %d %d",
		&state, &led_br[0], &led_br[1], &led_br[2], &led_br[3], &led_br[4], &led_br[5], &led_br[6], &led_br[7], &led_br[8]) == 10) {
		if(state == 1){
			for(num = 0; num < 9; num++){
				aw210xx_single_led_br_set(aw210xx, led[num], led_br[num]*aw210xx->setting_br /4095);
			}
			aw210xx_update(aw210xx);
			aw210xx->exclamation_effect = 1;

		}else if((state == 0) && (aw210xx->exclamation_effect != 0)){
			for(num = 0; num < 9; num++){
				aw210xx_single_led_br_set(aw210xx, led[num], 0);
			}
			aw210xx_update(aw210xx);
			aw210xx->exclamation_effect = 0;
		}
	}
	mutex_unlock(&aw210xx->led_mutex);
	return len;
}

static ssize_t aw210xx_opdetect_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	ssize_t len = 0;
	int i = 0;
	uint8_t reg_val[3] = {0};

	aw210xx_open_short_detect(aw210xx, AW210XX_OPEN_DETECT, reg_val);
	for (i = 0; i < sizeof(reg_val); i++)
		len += snprintf(buf + len, PAGE_SIZE - len,
					"OSST%d:%#x\n", i, reg_val[i]);

	return len;
}

static ssize_t aw210xx_stdetect_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw210xx *aw210xx = container_of(led_cdev, struct aw210xx, cdev);
	ssize_t len = 0;
	int i = 0;
	uint8_t reg_val[3] = {0};

	aw210xx_open_short_detect(aw210xx, AW210XX_SHORT_DETECT, reg_val);
	for (i = 0; i < sizeof(reg_val); i++)
		len += snprintf(buf + len, PAGE_SIZE - len,
					"OSST%d:%#x\n", i, reg_val[i]);
	return len;
}

static DEVICE_ATTR(reg, 0664, aw210xx_reg_show, aw210xx_reg_store);
static DEVICE_ATTR(hwen, 0664, aw210xx_hwen_show, aw210xx_hwen_store);
static DEVICE_ATTR(glo_current, 0664, aw210xx_glo_current_show, aw210xx_glo_current_store);
static DEVICE_ATTR(setting_br, 0664, aw210xx_setting_br_show, aw210xx_setting_br_store);
static DEVICE_ATTR(single_led_br,0220, NULL, aw210xx_single_led_br_store);
static DEVICE_ATTR(rear_cam_led_br, 0664, aw210xx_rear_cam_led_br_show, aw210xx_rear_cam_led_br_store);
static DEVICE_ATTR(front_cam_led_br, 0664, aw210xx_front_cam_led_br_show, aw210xx_front_cam_led_br_store);
static DEVICE_ATTR(horse_race_leds_br, 0664, aw210xx_horse_race_leds_br_show, aw210xx_horse_race_leds_br_store);
static DEVICE_ATTR(round_leds_br, 0664, aw210xx_round_leds_br_show, aw210xx_round_leds_br_store);
static DEVICE_ATTR(vline_leds_br, 0664, aw210xx_vline_leds_br_show, aw210xx_vline_leds_br_store);
static DEVICE_ATTR(dot_led_br, 0664, aw210xx_dot_led_br_show, aw210xx_dot_led_br_store);
static DEVICE_ATTR(all_white_leds_br,0664, aw210xx_all_white_leds_br_show, aw210xx_all_white_leds_br_store);
static DEVICE_ATTR(all_leds_effect, 0664, aw210xx_all_leds_effect_show, aw210xx_all_leds_effect_store);
static DEVICE_ATTR(round_leds_effect, 0220, NULL, aw210xx_round_leds_effect_store);
static DEVICE_ATTR(leds_breath_set, 0220, NULL, aw210xx_leds_breath_set_store);
static DEVICE_ATTR(horse_race_leds, 0220, NULL, aw210xx_horse_race_leds_store);
static DEVICE_ATTR(wired_charging_leds, 0664, aw210xx_wired_charging_show, aw210xx_wired_charging_store);
static DEVICE_ATTR(wlr_charging_leds, 0220, NULL, aw210xx_wlr_charging_leds_store);
static DEVICE_ATTR(random_delay, 0220, NULL, aw210xx_random_delay_store);
static DEVICE_ATTR(random_leds_effect, 0220, NULL, aw210xx_random_leds_effect_store);
static DEVICE_ATTR(keybd_leds_effect, 0220, NULL, aw210xx_keybd_leds_effect_store);
static DEVICE_ATTR(music_leds_effect, 0220, NULL, aw210xx_music_leds_effect_store);
static DEVICE_ATTR(video_leds_effect, 0220, NULL, aw210xx_video_leds_effect_store);
static DEVICE_ATTR(ga_leds_effect, 0220, NULL, aw210xx_ga_leds_effect_store);
static DEVICE_ATTR(nf_leds_effect, 0220, NULL, aw210xx_nf_leds_effect_store);
static DEVICE_ATTR(bootan_leds_effect, 0220, NULL, aw210xx_bootan_leds_effect_store);
static DEVICE_ATTR(rear_cam_leds_effect, 0220, NULL, aw210xx_rear_cam_leds_effect_store);
static DEVICE_ATTR(ringtone_delay, 0664, aw210xx_ringtone_delay_show, aw210xx_ringtone_delay_store);
static DEVICE_ATTR(ringtone_leds_effect, 0220, NULL, aw210xx_ringtone_leds_effect_store);
static DEVICE_ATTR(flip_leds_effect, 0220, NULL, aw210xx_flip_leds_effect_store);
static DEVICE_ATTR(exclamation_leds_effect, 0220, NULL, aw210xx_exclamation_leds_effect_store);
static DEVICE_ATTR(opdetect, 0664, aw210xx_opdetect_show, NULL);
static DEVICE_ATTR(stdetect, 0664, aw210xx_stdetect_show, NULL);

static struct attribute *aw210xx_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_glo_current.attr,
	&dev_attr_setting_br.attr,
	&dev_attr_single_led_br.attr,
	&dev_attr_rear_cam_led_br.attr,
	&dev_attr_front_cam_led_br.attr,
	&dev_attr_horse_race_leds_br.attr,
	&dev_attr_round_leds_br.attr,
	&dev_attr_vline_leds_br.attr,
	&dev_attr_dot_led_br.attr,
	&dev_attr_all_white_leds_br.attr,
	&dev_attr_all_leds_effect.attr,
	&dev_attr_round_leds_effect.attr,
	&dev_attr_leds_breath_set.attr,
	&dev_attr_horse_race_leds.attr,
	&dev_attr_wired_charging_leds.attr,
	&dev_attr_wlr_charging_leds.attr,
	&dev_attr_random_delay.attr,
	&dev_attr_random_leds_effect.attr,
	&dev_attr_keybd_leds_effect.attr,
	&dev_attr_music_leds_effect.attr,
	&dev_attr_video_leds_effect.attr,
	&dev_attr_ga_leds_effect.attr,
	&dev_attr_nf_leds_effect.attr,
	&dev_attr_bootan_leds_effect.attr,
	&dev_attr_rear_cam_leds_effect.attr,
	&dev_attr_ringtone_delay.attr,
	&dev_attr_ringtone_leds_effect.attr,
	&dev_attr_flip_leds_effect.attr,
	&dev_attr_exclamation_leds_effect.attr,
	&dev_attr_opdetect.attr,
	&dev_attr_stdetect.attr,
	NULL,
};

static struct attribute_group aw210xx_attribute_group = {
	.attrs = aw210xx_attributes
};
/******************************************************
 *
 * led class dev
 ******************************************************/

static int aw210xx_parse_led_cdev(struct aw210xx *aw210xx,
		struct device_node *np)
{
	int ret = -1;
	struct device_node *temp;

	AW_LOG("enter\n");
	for_each_child_of_node(np, temp) {
		ret = of_property_read_string(temp, "aw210xx,name",
				&aw210xx->cdev.name);
		if (ret < 0) {
			dev_err(aw210xx->dev,
				"Failure reading led name, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw210xx,imax",
				&aw210xx->imax);
		if (ret < 0) {
			dev_err(aw210xx->dev,
				"Failure reading imax, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw210xx,brightness",
				&aw210xx->cdev.brightness);
		if (ret < 0) {
			dev_err(aw210xx->dev,
				"Failure reading brightness, ret = %d\n", ret);
			goto free_pdata;
		}
		ret = of_property_read_u32(temp, "aw210xx,max_brightness",
				&aw210xx->cdev.max_brightness);
		if (ret < 0) {
			dev_err(aw210xx->dev,
				"Failure reading max brightness, ret = %d\n",
				ret);
			goto free_pdata;
		}
	}

	INIT_WORK(&aw210xx->brightness_work, aw210xx_brightness_work);
	aw210xx->cdev.brightness_set = aw210xx_set_brightness;

	ret = led_classdev_register(aw210xx->dev, &aw210xx->cdev);
	if (ret) {
		AW_ERR("unable to register led ret=%d\n", ret);
		goto free_pdata;
	}

	ret = sysfs_create_group(&aw210xx->cdev.dev->kobj,
			&aw210xx_attribute_group);
	if (ret) {
		AW_ERR("led sysfs ret: %d\n", ret);
		goto free_class;
	}

	aw210xx_led_init(aw210xx);

	return 0;

free_class:
	led_classdev_unregister(&aw210xx->cdev);
free_pdata:
	return ret;
}

/*****************************************************
 *
 * check chip id and version
 *
 *****************************************************/
static int aw210xx_read_chipid(struct aw210xx *aw210xx)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char chipid = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		ret = aw210xx_i2c_read(aw210xx, AW210XX_REG_RESET, &chipid);
		if (ret < 0) {
			AW_ERR("failed to read chipid: %d\n", ret);
		} else {
			aw210xx->chipid = chipid;
			switch (aw210xx->chipid) {
			case AW21018_CHIPID:
				AW_LOG("AW21018, read chipid = 0x%02x!!\n",
						chipid);
				return 0;
			case AW21012_CHIPID:
				AW_LOG("AW21012, read chipid = 0x%02x!!\n",
						chipid);
				return 0;
			case AW21009_CHIPID:
				AW_LOG("AW21009, read chipid = 0x%02x!!\n",
						chipid);
				return 0;
			default:
				AW_LOG("chip is unsupported device id = %x\n",
						chipid);
				break;
			}
		}
		cnt++;
		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
				AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}

	return -EINVAL;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw210xx_parse_dt(struct device *dev, struct aw210xx *aw210xx,
		struct device_node *np)
{
	int ret = -EINVAL;

	aw210xx->enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);
	if (aw210xx->enable_gpio < 0) {
		aw210xx->enable_gpio = -1;
		AW_ERR("no enable gpio provided, HW enable unsupported\n");
		return ret;
	}

	ret = of_property_read_u32(np, "osc_clk",
			&aw210xx->osc_clk);
	if (ret < 0) {
		AW_ERR("no osc_clk provided, osc clk unsupported\n");
		return ret;
	}

	ret = of_property_read_u32(np, "br_res",
			&aw210xx->br_res);
	if (ret < 0) {
		AW_ERR("brightness resolution unsupported\n");
		return ret;
	}

	ret = of_property_read_u32(np, "global_current",
			&aw210xx->glo_current);
	if (ret < 0) {
		AW_ERR("global current resolution unsupported\n");
		return ret;
	}

	return 0;
}

static void aw210xx_update_general_leds_effect(struct aw210xx *aw210xx, int val)
{
	uint32_t i = 0;
	uint32_t led1_br = 0, led2_br = 0, led3_br = 0, led4_br = 0, led5_br = 0;
	ktime_t start, runtime, delay;

	for (i = 0; i < aw210xx_general_leds_effect[val].count; i += 5){
		start = ktime_get();

		led1_br = aw210xx_general_leds_effect[val].p[i] * aw210xx->setting_br /4095;
		led2_br = aw210xx_general_leds_effect[val].p[i+1] * aw210xx->setting_br /4095;
		led3_br = aw210xx_general_leds_effect[val].p[i+2] * aw210xx->setting_br /4095;
		led4_br = aw210xx_general_leds_effect[val].p[i+3] * aw210xx->setting_br /4095;
		led5_br = aw210xx_general_leds_effect[val].p[i+4] * aw210xx->setting_br /4095;

		aw210xx_single_led_br_set(aw210xx, 7, led1_br);
		aw210xx_single_led_br_set(aw210xx, 1, led2_br);
		aw210xx_round_leds_br_set(aw210xx, led3_br);
		aw210xx_vline_leds_br_set(aw210xx, led4_br);
		aw210xx_single_led_br_set(aw210xx, 16, led5_br);

		aw210xx_update(aw210xx);
		if(general_effect_end == 1){
			aw210xx_all_white_leds_br_set(aw210xx, 0);
			aw210xx_update(aw210xx);
			general_effect_end = 0;
			return;
		}

		runtime  = ktime_sub(ktime_get(), start);
		delay = rt_flush_delay*1000 - runtime;//ns
		if(delay > 0){
			usleep_range(delay/1000, delay/1000);
		}
	}
}

static void aw210xx_wired_charging_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			wired_charging_work);
	int led[9] ={16,13,11,9,12,10,14,15,8};
	int num =0;
	int bat_val = (aw210xx->wired_charging >> 16) & 0xFF;

	AW_INFO("enter \n");
	pm_stay_awake(aw210xx->dev);
	for(num = 0; num<9; num++){
		aw210xx_single_led_br_set(aw210xx, led[num], aw210xx->setting_br);
		aw210xx_update(aw210xx);
		usleep_range(15000, 15000);
		if((aw210xx->wired_charging & 0xFF) == 0){
			goto end;
		}
	}

	if((aw210xx->wired_charging & 0xFF) == 0){
		goto end;
	}

	for(num = 8 ; (num >= bat_val); num--){
		aw210xx_single_led_br_set(aw210xx, led[num], 0);
		aw210xx_update(aw210xx);
		usleep_range(5000, 5000);
	}
	if((aw210xx->wired_charging & 0xFF) == 0){
		goto end;
	}

	usleep_range(2000000, 2000000);

	if((aw210xx->wired_charging & 0xFF) == 0){
		goto end;
	}

	for((num = bat_val -1); num >= 0; num--){
		aw210xx_single_led_br_set(aw210xx, led[num], 0);
		aw210xx_update(aw210xx);
		usleep_range(11000, 11000);
	}

end:
	aw210xx_all_white_leds_br_set(aw210xx, 0);
	aw210xx_update(aw210xx);
	pm_relax(aw210xx->dev);
	AW_INFO("end \n");
}

static void aw210xx_wlr_charging_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			wlr_charging_work);

	AW_INFO("enter \n");

	pm_stay_awake(aw210xx->dev);
	general_effect_end = 0;
	aw210xx_update_general_leds_effect(aw210xx, 1);
	aw210xx_all_white_leds_br_set(aw210xx, 0);
	aw210xx_update(aw210xx);
	pm_relax(aw210xx->dev);

}

static void aw210xx_random_effect_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			random_effect_work);

	aw210xx_random_led(aw210xx);

	while(aw210xx->random_effect ==1){
		usleep_range(aw210xx->random_delay_t, aw210xx->random_delay_t+1);
		aw210xx_random_led(aw210xx);
	}
}

static void aw210xx_video_effect_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			video_effect_work);

	AW_INFO("aw210xx->video_effect:%d \n", aw210xx->video_effect);

	while(aw210xx->video_effect ==1){
		if(dev_is_black == 1){
			aw210xx_single_led_br_set(aw210xx, 17, 3840);
		}else if(dev_is_black == 0){
			aw210xx_single_led_br_set(aw210xx, 17, 1306);
		}
		aw210xx_update(aw210xx);
		usleep_range(500000, 500000);
		aw210xx_single_led_br_set(aw210xx, 17, 0);
		aw210xx_update(aw210xx);
		usleep_range(500000, 500000);
	}
}

static void aw210xx_ga_effect_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			ga_effect_work);

	AW_INFO("enter \n");

	if((aw210xx->ga_effect >> 8) & 0xFF){
		aw210xx_leds_breath_data.repeat_nums =0;//Infinite loop
		aw210xx_round_leds_breath(aw210xx);
	}
}

static void aw210xx_update_notification_leds_effect(struct aw210xx *aw210xx, int val)
{
	uint32_t i = 0;
	uint32_t led1_br = 0, led2_br = 0, led3_br = 0, led4_br = 0, led5_br = 0;
	ktime_t start, runtime, delay;

	for (i = 0; i < aw210xx_notification_leds_effect[val].count; i += 5){
		start = ktime_get();

		led1_br = aw210xx_notification_leds_effect[val].p[i] * aw210xx->setting_br /4095;
		led2_br = aw210xx_notification_leds_effect[val].p[i+1] * aw210xx->setting_br /4095;
		led3_br = aw210xx_notification_leds_effect[val].p[i+2] * aw210xx->setting_br /4095;
		led4_br = aw210xx_notification_leds_effect[val].p[i+3] * aw210xx->setting_br /4095;
		led5_br = aw210xx_notification_leds_effect[val].p[i+4] * aw210xx->setting_br /4095;

		aw210xx_single_led_br_set(aw210xx, 7, led1_br);
		aw210xx_single_led_br_set(aw210xx, 1, led2_br);
		aw210xx_round_leds_br_set(aw210xx, led3_br);
		aw210xx_vline_leds_br_set(aw210xx, led4_br);
		aw210xx_single_led_br_set(aw210xx, 16, led5_br);
		aw210xx_update(aw210xx);

		if(aw210xx->nf_effect == 0){
			aw210xx_all_white_leds_br_set(aw210xx, 0);
			aw210xx_update(aw210xx);
			return;
		}

		runtime  = ktime_sub(ktime_get(), start);
		delay = rt_flush_delay*1000 - runtime;//ns
		if(delay > 0){
			usleep_range(delay/1000, delay/1000);
		};
	}
}

static void aw210xx_nf_effect_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			nf_effect_work);

	AW_INFO("enter \n");

	pm_stay_awake(aw210xx->dev);
	if((aw210xx->nf_effect -1) < (sizeof(aw210xx_notification_leds_effect)/sizeof(aw210xx_effect_cfg_t))){
		if(aw210xx->nf_effect != 0){
			aw210xx_update_notification_leds_effect(aw210xx, aw210xx->nf_effect -1);
			if(aw210xx->nf_effect !=  0){
				ev_happen = 1;
				ev_code = '1';
				AW_INFO("end ev_happen:%d ev_code:%c\n",ev_happen,ev_code);
				wake_up_interruptible(&aw210xx_nf_waitq);
				aw210xx->nf_effect = 0;
			}
			aw210xx_all_white_leds_br_set(aw210xx, 0);
			aw210xx_update(aw210xx);
		}
	}else{
		aw210xx_leds_breath_data.repeat_nums =2;
		aw210xx_front_cam_led_breath(aw210xx);
		aw210xx_leds_breath_data.repeat_nums =1;
		if(aw210xx->nf_effect !=  0){
			ev_happen = 1;
			ev_code = '1';
			AW_INFO("end ev_happen:%d ev_code:%c\n",ev_happen,ev_code);
			wake_up_interruptible(&aw210xx_nf_waitq);
			aw210xx->nf_effect = 0;
		}
	}
	pm_relax(aw210xx->dev);
}

static void aw210xx_bootan_effect_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			bootan_effect_work);

	AW_INFO("enter \n");

	if(aw210xx->bootan_effect == 1){
		aw210xx_leds_breath_data.repeat_nums =0;
		aw210xx_all_white_leds_breath(aw210xx);
		aw210xx_leds_breath_data.repeat_nums =1;
	}
}

static void aw210xx_update_ringtone_leds_effect(struct aw210xx *aw210xx, int val)
{
	uint32_t i = 0;
	uint32_t led1_br = 0, led2_br = 0, led3_br = 0, led4_br = 0, led5_br = 0;
	ktime_t start, runtime, delay;

	AW_INFO("enter \n");
	for (i = 0; i < aw210xx_ringtone_leds_effect[val].count; i += 5){
		start = ktime_get();

		//AW_INFO("%d %d %d %d %d \n", aw210xx_ringtone_leds_effect[val].p[i], aw210xx_ringtone_leds_effect[val].p[i+1],
		//	aw210xx_ringtone_leds_effect[val].p[i+2], aw210xx_ringtone_leds_effect[val].p[i+3], aw210xx_ringtone_leds_effect[val].p[i+4]);

		led1_br = aw210xx_ringtone_leds_effect[val].p[i] * aw210xx->setting_br /4095;
		led2_br = aw210xx_ringtone_leds_effect[val].p[i+1] * aw210xx->setting_br /4095;
		led3_br = aw210xx_ringtone_leds_effect[val].p[i+2] * aw210xx->setting_br /4095;
		led4_br = aw210xx_ringtone_leds_effect[val].p[i+3] * aw210xx->setting_br /4095;
		led5_br = aw210xx_ringtone_leds_effect[val].p[i+4] * aw210xx->setting_br /4095;

		aw210xx_single_led_br_set(aw210xx, 7, led1_br);
		aw210xx_single_led_br_set(aw210xx, 1, led2_br);
		aw210xx_round_leds_br_set(aw210xx, led3_br);
		aw210xx_vline_leds_br_set(aw210xx, led4_br);
		aw210xx_single_led_br_set(aw210xx, 16, led5_br);

		aw210xx_update(aw210xx);
		if(aw210xx->ringtone_effect == 0){
			AW_INFO("aw210xx->ringtone_effect == 0");
			aw210xx_all_white_leds_br_set(aw210xx, 0);
			aw210xx_update(aw210xx);
			return;
		}

		runtime  = ktime_sub(ktime_get(), start);
		delay = rt_flush_delay*1000 - runtime;//ns
		if(delay > 0){
			usleep_range(delay/1000, delay/1000);
		}
	}
	AW_INFO("end \n");
}

static void aw210xx_ringtone_effect_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			ringtone_effect_work);

	int i = 0;
	AW_INFO("enter \n");

	pm_stay_awake(aw210xx->dev);
	if((aw210xx->ringtone_effect -1) < (sizeof(aw210xx_ringtone_leds_effect)/sizeof(aw210xx_effect_cfg_t))){
		while(aw210xx->ringtone_effect != 0){
			aw210xx_update_ringtone_leds_effect(aw210xx, aw210xx->ringtone_effect -1);
			for (i=0; i<25; i++){
				if(aw210xx->ringtone_effect != 0){
					usleep_range(20000, 20000);
				}else{
					AW_INFO("break \n");
					break;
				}
			}
		}
	}else{
		while(aw210xx->ringtone_effect != 0){
			aw210xx->random_delay_t = 50000;
			aw210xx_random_led(aw210xx);
			usleep_range(aw210xx->random_delay_t, aw210xx->random_delay_t + 1);
		}
	}
	aw210xx_all_white_leds_br_set(aw210xx, 0);
	aw210xx_update(aw210xx);
	pm_relax(aw210xx->dev);

	AW_INFO("end \n");
}

static void aw210xx_flip_effect_work(struct work_struct *work)
{
	struct aw210xx *aw210xx = container_of(work, struct aw210xx,
			flip_effect_work);
	AW_INFO("enter \n");
#if 0
	aw210xx_leds_breath_data.repeat_nums =1;
	if(aw210xx->flip_effect ==1){
		aw210xx_all_white_leds_breath(aw210xx);
		aw210xx->flip_effect = 0;
		aw210xx_all_white_leds_br_set(aw210xx, 0);
		aw210xx_update(aw210xx);
	}
#endif
	pm_stay_awake(aw210xx->dev);
	general_effect_end = 0;
	aw210xx_update_general_leds_effect(aw210xx, 2);
	aw210xx_all_white_leds_br_set(aw210xx, 0);
	aw210xx_update(aw210xx);
	pm_relax(aw210xx->dev);
}

static ssize_t aw210xx_nf_read(struct file *file, char __user *user, size_t size,loff_t *ppos)
{
	int ret =0;
	AW_INFO("enter\n");
	if (size != 1)
		return -EINVAL;
	wait_event_interruptible(aw210xx_nf_waitq, ev_happen);
	ret = copy_to_user(user, &ev_code, 1);
	if ( ret == 0 )
	{
		ev_code = '0';
		ev_happen = 0;
		return 1;
	}
	ev_code = '0';
	ev_happen = 0;
	return ret;
}

static unsigned int aw210xx_nf_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	poll_wait(file, &aw210xx_nf_waitq, wait);
	if(ev_happen == 1)
	{
		mask |= POLLIN | POLLRDNORM;
	}
	return mask;
}

/* File operations struct for character device */
static const struct file_operations aw210xx_nf_cdev_fops = {
	.owner		= THIS_MODULE,
	.read		= aw210xx_nf_read,
	.poll               = aw210xx_nf_poll,
};

static struct miscdevice aw210xx_nf_cdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aw210xx_nf",
	.fops = &aw210xx_nf_cdev_fops,
};

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw210xx_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{

	struct aw210xx *aw210xx;
	struct device_node *np = i2c->dev.of_node;
	int ret;

	AW_INFO("enter\n");

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		AW_ERR("check_functionality failed\n");
		return -EIO;
	}

	aw210xx = devm_kzalloc(&i2c->dev, sizeof(struct aw210xx), GFP_KERNEL);
	if (aw210xx == NULL)
		return -ENOMEM;

	aw210xx->dev = &i2c->dev;
	aw210xx->i2c = i2c;
	i2c_set_clientdata(i2c, aw210xx);

	mutex_init(&aw210xx->led_mutex);

	/* aw210xx parse device tree */
	if (np) {
		ret = aw210xx_parse_dt(&i2c->dev, aw210xx, np);
		if (ret) {
			AW_ERR("failed to parse device tree node\n");
			goto err_parse_dt;
		}
	}

	if (gpio_is_valid(aw210xx->enable_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw210xx->enable_gpio,
				GPIOF_OUT_INIT_LOW, "aw210xx_en");
		if (ret) {
			AW_ERR("enable gpio request failed\n");
			goto err_gpio_request;
		}
	}

	/* hardware enable */
	aw210xx_hw_enable(aw210xx, true);

	/* aw210xx identify */
	ret = aw210xx_read_chipid(aw210xx);
	if (ret < 0) {
		AW_ERR("aw210xx_read_chipid failed ret=%d\n", ret);
		goto err_id;
	}

	dev_set_drvdata(&i2c->dev, aw210xx);
	aw210xx_parse_led_cdev(aw210xx, np);
	if (ret < 0) {
		AW_ERR("error creating led class dev\n");
		goto err_sysfs;
	}

	aw210xx->leds_workqueue = create_singlethread_workqueue("leds_wq");
	if (!aw210xx->leds_workqueue) {
		AW_ERR("create leds workqueue fail");
	}else{
		INIT_WORK(&aw210xx->wired_charging_work, aw210xx_wired_charging_work);
		INIT_WORK(&aw210xx->wlr_charging_work, aw210xx_wlr_charging_work);
		INIT_WORK(&aw210xx->random_effect_work, aw210xx_random_effect_work);
		INIT_WORK(&aw210xx->ga_effect_work, aw210xx_ga_effect_work);
		INIT_WORK(&aw210xx->nf_effect_work, aw210xx_nf_effect_work);
		INIT_WORK(&aw210xx->bootan_effect_work, aw210xx_bootan_effect_work);
		INIT_WORK(&aw210xx->ringtone_effect_work, aw210xx_ringtone_effect_work);
		INIT_WORK(&aw210xx->flip_effect_work, aw210xx_flip_effect_work);
	}

	aw210xx->video_workqueue = create_singlethread_workqueue("video_wq");
	if (!aw210xx->leds_workqueue) {
		AW_ERR("create video workqueue fail");
	}else{
		INIT_WORK(&aw210xx->video_effect_work, aw210xx_video_effect_work);
	}

	ret = misc_register(&aw210xx_nf_cdev);
	if (ret) {
		AW_ERR("%s: misc_register failed\n", __func__);
	}

	device_init_wakeup(aw210xx->dev, true);

	AW_INFO("probe completed!\n");

	return 0;

err_sysfs:
err_id:
	devm_gpio_free(&i2c->dev, aw210xx->enable_gpio);
err_gpio_request:
err_parse_dt:
	devm_kfree(&i2c->dev, aw210xx);
	aw210xx = NULL;
	return ret;
}

static int aw210xx_i2c_remove(struct i2c_client *i2c)
{
	struct aw210xx *aw210xx = i2c_get_clientdata(i2c);

	AW_LOG("enter\n");
	sysfs_remove_group(&aw210xx->cdev.dev->kobj, &aw210xx_attribute_group);
	led_classdev_unregister(&aw210xx->cdev);
	if (gpio_is_valid(aw210xx->enable_gpio))
		devm_gpio_free(&i2c->dev, aw210xx->enable_gpio);
	devm_kfree(&i2c->dev, aw210xx);
	aw210xx = NULL;

	return 0;
}

static const struct i2c_device_id aw210xx_i2c_id[] = {
	{AW210XX_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw210xx_i2c_id);

static const struct of_device_id aw210xx_dt_match[] = {
	{.compatible = "awinic,aw210xx_led"},
	{}
};

static struct i2c_driver aw210xx_i2c_driver = {
	.driver = {
		.name = AW210XX_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw210xx_dt_match),
		},
	.probe = aw210xx_i2c_probe,
	.remove = aw210xx_i2c_remove,
	.id_table = aw210xx_i2c_id,
};

static int __init aw210xx_i2c_init(void)
{
	int ret = 0;

	AW_LOG("enter, aw210xx driver version %s\n", AW210XX_DRIVER_VERSION);

	ret = i2c_add_driver(&aw210xx_i2c_driver);
	if (ret) {
		AW_ERR("failed to register aw210xx driver!\n");
		return ret;
	}

	return 0;
}
module_init(aw210xx_i2c_init);

static void __exit aw210xx_i2c_exit(void)
{
	i2c_del_driver(&aw210xx_i2c_driver);
}
module_exit(aw210xx_i2c_exit);

MODULE_DESCRIPTION("AW210XX LED Driver");
MODULE_LICENSE("GPL v2");
