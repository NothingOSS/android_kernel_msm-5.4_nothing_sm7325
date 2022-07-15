/*
 * tfa98xx.c   tfa98xx codec module
 *
 *
 * Copyright (C) 2014-2020 NXP Semiconductors, All Rights Reserved.
 * Copyright 2020 GOODIX 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#define pr_fmt(fmt) "%s(): " fmt, __func__

#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/input.h>
#include "config.h"
#include "tfa98xx.h"
#include "tfa.h"
#include "tfa_internal.h"

 /* required for enum tfa9912_irq */
#include "tfa98xx_tfafieldnames.h"

#define TFA98XX_VERSION	TFA98XX_API_REV_STR

#define I2C_RETRIES 50
#define I2C_RETRY_DELAY 5 /* ms */
#define TFA9894_STEREO
//#define TFA98xx_calibrate
//#define TFA98xx_dsp_bypass
//#define TFA98xx_F0_tracking
//#define TFA98xx_memtrack_nondsp

/* Change volume selection behavior:
 * Uncomment following line to generate a profile change when updating
 * a volume control (also changes to the profile of the modified  volume
 * control)
 */
 /*#define TFA98XX_ALSA_CTRL_PROF_CHG_ON_VOL	1
 */

 /* Supported rates and data formats */
#define TFA98XX_RATES SNDRV_PCM_RATE_8000_48000
#define TFA98XX_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define TF98XX_MAX_DSP_START_TRY_COUNT	10

/* data accessible by all instances */
static struct kmem_cache *tfa98xx_cache = NULL;  /* Memory pool used for DSP messages */
/* Mutex protected data */
static DEFINE_MUTEX(tfa98xx_mutex);
static LIST_HEAD(tfa98xx_device_list);
static int tfa98xx_device_count = 0;
static int tfa98xx_sync_count = 0;
static LIST_HEAD(profile_list); 	   /* list of user selectable profiles */
static int tfa98xx_mixer_profiles = 0; /* number of user selectable profiles */
static int tfa98xx_mixer_profile = 0;  /* current mixer profile */
static struct snd_kcontrol_new *tfa98xx_controls;
static TfaContainer_t *tfa98xx_container = NULL;

static int tfa98xx_kmsg_regs = 0;
static int tfa98xx_ftrace_regs = 0;
#ifdef TFA98xx_calibrate
#define TFADSP_FLAG_CALIBRATE_DONE	1
struct tfa98xx *tfa98xx_global[2] = {0};
static enum Tfa98xx_Error tfa98xxCalibration(struct tfa98xx **tfa98xx, int *speakerImpedance); 
static int cal_impedence[2] = {0};
#endif
static char *fw_name = "tfa98xx.cnt";
module_param(fw_name, charp, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(fw_name, "TFA98xx DSP firmware (container file) name.");

static int trace_level = 0;
module_param(trace_level, int, S_IRUGO);
MODULE_PARM_DESC(trace_level, "TFA98xx debug trace level (0=off, bits:1=verbose,2=regdmesg,3=regftrace,4=timing).");

static char *dflt_prof_name = "";
module_param(dflt_prof_name, charp, S_IRUGO);

static int no_start = 0;
module_param(no_start, int, S_IRUGO);
MODULE_PARM_DESC(no_start, "do not start the work queue; for debugging via user\n");

static int no_reset = 0;
module_param(no_reset, int, S_IRUGO);
MODULE_PARM_DESC(no_reset, "do not use the reset line; for debugging via user\n");

static int pcm_sample_format = 1;
module_param(pcm_sample_format, int, S_IRUGO);
MODULE_PARM_DESC(pcm_sample_format, "PCM sample format: 0=S16_LE, 1=S24_LE, 2=S32_LE, 3=dynamic\n");

static int pcm_no_constraint = 1;
module_param(pcm_no_constraint, int, S_IRUGO);
MODULE_PARM_DESC(pcm_no_constraint, "do not use constraints for PCM parameters\n");

static void tfa98xx_tapdet_check_update(struct tfa98xx *tfa98xx);
static int tfa98xx_get_fssel(unsigned int rate);
static void tfa98xx_interrupt_enable(struct tfa98xx *tfa98xx, bool enable);

static int get_profile_from_list(char *buf, int id);
static int get_profile_id_for_sr(int id, unsigned int rate);

struct tfa98xx_rate {
	unsigned int rate;
	unsigned int fssel;
};
#ifdef TFA98xx_dsp_bypass
static atomic_t g_bypass;
static atomic_t g_Tx_enable;

extern int send_tfa_cal_set_bypass(void *buf, int cmd_size);
extern int send_tfa_cal_set_tx_enable(void *buf, int cmd_size);

/*************bypass control***************/
static int tfa987x_algo_get_status(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	int32_t ret = 0;
	ucontrol->value.integer.value[0] = atomic_read(&g_bypass);
	return ret;
}

static int tfa987x_algo_set_status(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	int32_t ret = 0;
	u8 buff[56] = {0}, *ptr = buff;
	((int32_t *)buff)[0] = ucontrol->value.integer.value[0];
	pr_err("%s:status data %d\n", __func__, ((int32_t *)buff)[0]);
	atomic_set(&g_bypass, ((int32_t *)buff)[0]);
	ret = send_tfa_cal_set_bypass(ptr, 4);
	return ret;
}

static int tfa987x_algo_set_tx_enable(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	int32_t ret = 0;
	u8 buff[56] = {0}, *ptr = buff;
	((int32_t *)buff)[0] = ucontrol->value.integer.value[0];
	pr_err("%s:set_tx_enable %d\n", __func__, ((int32_t *)buff)[0]);
	atomic_set(&g_Tx_enable, ((int32_t *)buff)[0]);
	ret = send_tfa_cal_set_tx_enable(ptr, 4);
	return ret;
}

static int tfa987x_algo_get_tx_status(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	int32_t ret = 0;
	ucontrol->value.integer.value[0] = atomic_read(&g_Tx_enable);
	return ret;
}

static const char *tfa987x_algo_text[] = {
	"DISABLE", "ENABLE"
};

static const char *tfa987x_tx_text[] = {
	"DISABLE", "ENABLE"
};

static const struct soc_enum tfa987x_algo_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tfa987x_algo_text),tfa987x_algo_text)
};

static const struct soc_enum tfa987x_tx_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tfa987x_tx_text),tfa987x_tx_text)
};

const struct snd_kcontrol_new tfa987x_algo_filter_mixer_controls[] = {
	SOC_ENUM_EXT("TFA987X_ALGO_STATUS", tfa987x_algo_enum[0], tfa987x_algo_get_status, tfa987x_algo_set_status),
	SOC_ENUM_EXT("TFA987X_TX_ENABLE", tfa987x_tx_enum[0], tfa987x_algo_get_tx_status, tfa987x_algo_set_tx_enable)
};
#endif

static const struct tfa98xx_rate rate_to_fssel[] = {
	{ 8000, 0 },
	{ 11025, 1 },
	{ 12000, 2 },
	{ 16000, 3 },
	{ 22050, 4 },
	{ 24000, 5 },
	{ 32000, 6 },
	{ 44100, 7 },
	{ 48000, 8 },
};


static inline char *tfa_cont_profile_name(struct tfa98xx *tfa98xx, int prof_idx)
{
	if (tfa98xx->tfa->cnt == NULL)
		return NULL;
	return tfaContProfileName(tfa98xx->tfa->cnt, tfa98xx->tfa->dev_idx, prof_idx);
}

static enum tfa_error tfa98xx_write_re25(struct tfa_device *tfa, int value)
{
	enum tfa_error err;

	/* clear MTPEX */
	err = tfa_dev_mtp_set(tfa, TFA_MTP_EX, 0);
	if (err == tfa_error_ok) {
		/* set RE25 in shadow regiser */
		err = tfa_dev_mtp_set(tfa, TFA_MTP_RE25_PRIM, value);
	}
	if (err == tfa_error_ok) {
		/* set MTPEX to copy RE25 into MTP	*/
		err = tfa_dev_mtp_set(tfa, TFA_MTP_EX, 2);
	}

	return err;
}

/* Wrapper for tfa start */
static enum tfa_error tfa98xx_tfa_start(struct tfa98xx *tfa98xx, int next_profile, int vstep)
{
	enum tfa_error err;
	ktime_t start_time = {0}, stop_time = {0};
	u64 delta_time;

	start_time = ktime_get_boottime();
	err = tfa_dev_start(tfa98xx->tfa, next_profile, vstep);

	if (trace_level & 8) {
		stop_time = ktime_get_boottime();
		delta_time = ktime_to_ns(ktime_sub(stop_time, start_time));
		do_div(delta_time, 1000);
		dev_dbg(&tfa98xx->i2c->dev, "tfa_dev_start(%d,%d) time = %lld us\n",
			next_profile, vstep, delta_time);
	}

	if ((err == tfa_error_ok) && (tfa98xx->set_mtp_cal)) {
		enum tfa_error err_cal;
		err_cal = tfa98xx_write_re25(tfa98xx->tfa, tfa98xx->cal_data);
		if (err_cal != tfa_error_ok) {
			pr_err("Error, setting calibration value in mtp, err=%d\n", err_cal);
		}
		else {
			tfa98xx->set_mtp_cal = false;
			pr_info("Calibration value (%d) set in mtp\n",
				tfa98xx->cal_data);
		}
	}

	/* Check and update tap-detection state (in case of profile change) */
	tfa98xx_tapdet_check_update(tfa98xx);

	/* Remove sticky bit by reading it once */
	tfa_get_noclk(tfa98xx->tfa);

	/* A cold start erases the configuration, including interrupts setting.
	 * Restore it if required
	 */
	tfa98xx_interrupt_enable(tfa98xx, true);

	return err;
}

static int tfa98xx_input_open(struct input_dev *dev)
{
	struct tfa98xx *tfa98xx = input_get_drvdata(dev);
	dev_dbg(tfa98xx->codec->dev, "opening device file\n");

	/* note: open function is called only once by the framework.
	 * No need to count number of open file instances.
	 */
	if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK) {
		dev_dbg(&tfa98xx->i2c->dev,
			"DSP not loaded, cannot start tap-detection\n");
		return -EIO;
	}

	/* enable tap-detection service */
	tfa98xx->tapdet_open = true;
	tfa98xx_tapdet_check_update(tfa98xx);

	return 0;
}

static void tfa98xx_input_close(struct input_dev *dev)
{
	struct tfa98xx *tfa98xx = input_get_drvdata(dev);

	dev_dbg(tfa98xx->codec->dev, "closing device file\n");

	/* Note: close function is called if the device is unregistered */

	/* disable tap-detection service */
	tfa98xx->tapdet_open = false;
	tfa98xx_tapdet_check_update(tfa98xx);
}

static int tfa98xx_register_inputdev(struct tfa98xx *tfa98xx)
{
	int err;
	struct input_dev *input;
	input = input_allocate_device();

	if (!input) {
		dev_err(tfa98xx->codec->dev, "Unable to allocate input device\n");
		return -ENOMEM;
	}

	input->evbit[0] = BIT_MASK(EV_KEY);
	input->keybit[BIT_WORD(BTN_0)] |= BIT_MASK(BTN_0);
	input->keybit[BIT_WORD(BTN_1)] |= BIT_MASK(BTN_1);
	input->keybit[BIT_WORD(BTN_2)] |= BIT_MASK(BTN_2);
	input->keybit[BIT_WORD(BTN_3)] |= BIT_MASK(BTN_3);
	input->keybit[BIT_WORD(BTN_4)] |= BIT_MASK(BTN_4);
	input->keybit[BIT_WORD(BTN_5)] |= BIT_MASK(BTN_5);
	input->keybit[BIT_WORD(BTN_6)] |= BIT_MASK(BTN_6);
	input->keybit[BIT_WORD(BTN_7)] |= BIT_MASK(BTN_7);
	input->keybit[BIT_WORD(BTN_8)] |= BIT_MASK(BTN_8);
	input->keybit[BIT_WORD(BTN_9)] |= BIT_MASK(BTN_9);

	input->open = tfa98xx_input_open;
	input->close = tfa98xx_input_close;

	input->name = "tfa98xx-tapdetect";

	input->id.bustype = BUS_I2C;
	input_set_drvdata(input, tfa98xx);

	err = input_register_device(input);
	if (err) {
		dev_err(tfa98xx->codec->dev, "Unable to register input device\n");
		goto err_free_dev;
	}

	dev_dbg(tfa98xx->codec->dev, "Input device for tap-detection registered: %s\n",
		input->name);
	tfa98xx->input = input;
	return 0;

err_free_dev:
	input_free_device(input);
	return err;
}

/*
 * Check if an input device for tap-detection can and shall be registered.
 * Register it if appropriate.
 * If already registered, check if still relevant and remove it if necessary.
 * unregister: true to request inputdev unregistration.
 */
static void __tfa98xx_inputdev_check_register(struct tfa98xx *tfa98xx, bool unregister)
{
	bool tap_profile = false;
	unsigned int i;
	for (i = 0; i < tfa_cnt_get_dev_nprof(tfa98xx->tfa); i++) {
		if (strstr(tfa_cont_profile_name(tfa98xx, i), ".tap")) {
			tap_profile = true;
			tfa98xx->tapdet_profiles |= 1 << i;
			dev_info(tfa98xx->codec->dev,
				"found a tap-detection profile (%d - %s)\n",
				i, tfa_cont_profile_name(tfa98xx, i));
		}
	}

	/* Check for device support:
	 *	- at device level
	 *	- at container (profile) level
	 */
	if (!(tfa98xx->flags & TFA98XX_FLAG_TAPDET_AVAILABLE) ||
		!tap_profile ||
		unregister) {
		/* No input device supported or required */
		if (tfa98xx->input) {
			input_unregister_device(tfa98xx->input);
			tfa98xx->input = NULL;
		}
		return;
	}

	/* input device required */
	if (tfa98xx->input)
		dev_info(tfa98xx->codec->dev, "Input device already registered, skipping\n");
	else
		tfa98xx_register_inputdev(tfa98xx);
}

static void tfa98xx_inputdev_check_register(struct tfa98xx *tfa98xx)
{
	__tfa98xx_inputdev_check_register(tfa98xx, false);
}

static void tfa98xx_inputdev_unregister(struct tfa98xx *tfa98xx)
{
	__tfa98xx_inputdev_check_register(tfa98xx, true);
}

/* OTC reporting
 * Returns the MTP0 OTC bit value
 */
static int tfa98xx_dbgfs_otc_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int value;

	mutex_lock(&tfa98xx->dsp_lock);
	value = tfa_dev_mtp_get(tfa98xx->tfa, TFA_MTP_OTC);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (value < 0) {
		pr_err("[0x%x] Unable to check DSP access: %d\n", tfa98xx->i2c->addr, value);
		return -EIO;
	}

	*val = value;
	pr_debug("[0x%x] OTC : %d\n", tfa98xx->i2c->addr, value);

	return 0;
}

static int tfa98xx_dbgfs_otc_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum tfa_error err;

	if (val != 0 && val != 1) {
		pr_err("[0x%x] Unexpected value %llu\n", tfa98xx->i2c->addr, val);
		return -EINVAL;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	err = tfa_dev_mtp_set(tfa98xx->tfa, TFA_MTP_OTC, val);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (err != tfa_error_ok) {
		pr_err("[0x%x] Unable to check DSP access: %d\n", tfa98xx->i2c->addr, err);
		return -EIO;
	}

	pr_debug("[0x%x] OTC < %llu\n", tfa98xx->i2c->addr, val);

	return 0;
}

static int tfa98xx_dbgfs_mtpex_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int value;

	mutex_lock(&tfa98xx->dsp_lock);
	value = tfa_dev_mtp_get(tfa98xx->tfa, TFA_MTP_EX);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (value < 0) {
		pr_err("[0x%x] Unable to check DSP access: %d\n", tfa98xx->i2c->addr, value);
		return -EIO;
	}


	*val = value;
	pr_debug("[0x%x] MTPEX : %d\n", tfa98xx->i2c->addr, value);

	return 0;
}

static int tfa98xx_dbgfs_mtpex_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum tfa_error err;

	if (val != 0) {
		pr_err("[0x%x] Can only clear MTPEX (0 value expected)\n", tfa98xx->i2c->addr);
		return -EINVAL;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	err = tfa_dev_mtp_set(tfa98xx->tfa, TFA_MTP_EX, val);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (err != tfa_error_ok) {
		pr_err("[0x%x] Unable to check DSP access: %d\n", tfa98xx->i2c->addr, err);
		return -EIO;
	}

	pr_debug("[0x%x] MTPEX < 0\n", tfa98xx->i2c->addr);

	return 0;
}

static int tfa98xx_dbgfs_temp_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);

	mutex_lock(&tfa98xx->dsp_lock);
	*val = tfa98xx_get_exttemp(tfa98xx->tfa);
	mutex_unlock(&tfa98xx->dsp_lock);

	pr_debug("[0x%x] TEMP : %llu\n", tfa98xx->i2c->addr, *val);

	return 0;
}

static int tfa98xx_dbgfs_temp_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);

	mutex_lock(&tfa98xx->dsp_lock);
	tfa98xx_set_exttemp(tfa98xx->tfa, (short)val);
	mutex_unlock(&tfa98xx->dsp_lock);

	pr_debug("[0x%x] TEMP < %llu\n", tfa98xx->i2c->addr, val);

	return 0;
}

static ssize_t tfa98xx_dbgfs_start_set(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum tfa_error ret;
	char buf[32];
	const char ref[] = "please calibrate now";
	int buf_size, cal_profile = 0;

	/* check string length, and account for eol */
	if (count > sizeof(ref) + 1 || count < (sizeof(ref) - 1))
		return -EINVAL;

	buf_size = min(count, (size_t)(sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	/* Compare string, excluding the trailing \0 and the potentials eol */
	if (strncmp(buf, ref, sizeof(ref) - 1))
		return -EINVAL;

	mutex_lock(&tfa98xx->dsp_lock);
	ret = tfa_calibrate(tfa98xx->tfa);
	if (ret == tfa_error_ok) {
		cal_profile = tfaContGetCalProfile(tfa98xx->tfa);
		if (cal_profile < 0) {
			pr_warn("[0x%x] Calibration profile not found\n",
				tfa98xx->i2c->addr);
		}

		ret = tfa98xx_tfa_start(tfa98xx, cal_profile, tfa98xx->vstep);
	}
	if (ret == tfa_error_ok)
		tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE, 0);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (ret) {
		pr_info("[0x%x] Calibration start failed (%d)\n", tfa98xx->i2c->addr, ret);
		return -EIO;
	}
	else {
		pr_info("[0x%x] Calibration started\n", tfa98xx->i2c->addr);
	}

	return count;
}

static ssize_t tfa98xx_dbgfs_r_read(struct file *file,
	char __user *user_buf, size_t count,
	loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	char *str = NULL;
	uint16_t status;
	int ret;

	mutex_lock(&tfa98xx->dsp_lock);

	/* Need to ensure DSP is access-able, use mtp read access for this
	 * purpose
	 */
	ret = tfa98xx_get_mtp(tfa98xx->tfa, &status);
	if (ret) {
		ret = -EIO;
		pr_err("[0x%x] MTP read failed\n", tfa98xx->i2c->addr);
		goto r_c_err;
	}

	ret = tfaRunSpeakerCalibration(tfa98xx->tfa);
	if (ret) {
		ret = -EIO;
		pr_err("[0x%x] calibration failed\n", tfa98xx->i2c->addr);
		goto r_c_err;
	}

	str = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!str) {
		ret = -ENOMEM;
		pr_err("[0x%x] memory allocation failed\n", tfa98xx->i2c->addr);
		goto r_c_err;
	}

	if (tfa98xx->tfa->spkr_count > 1) {
		ret = snprintf(str, PAGE_SIZE,
			"Prim:%d mOhms, Sec:%d mOhms\n",
			tfa98xx->tfa->mohm[0],
			tfa98xx->tfa->mohm[1]);
	}
	else {
		ret = snprintf(str, PAGE_SIZE,
			"Prim:%d mOhms\n",
			tfa98xx->tfa->mohm[0]);
	}

	pr_debug("[0x%x] calib_done: %s", tfa98xx->i2c->addr, str);

	if (ret < 0)
		goto r_err;

	ret = simple_read_from_buffer(user_buf, count, ppos, str, ret);

r_err:
	kfree(str);
r_c_err:
	mutex_unlock(&tfa98xx->dsp_lock);
	return ret;
}

static ssize_t tfa98xx_dbgfs_version_read(struct file *file,
	char __user *user_buf, size_t count,
	loff_t *ppos)
{
	char str[] = TFA98XX_VERSION "\n";
	int ret;

	ret = simple_read_from_buffer(user_buf, count, ppos, str, sizeof(str));

	return ret;
}

static ssize_t tfa98xx_dbgfs_dsp_state_get(struct file *file,
	char __user *user_buf, size_t count,
	loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int ret = 0;
	char *str = NULL;

	switch (tfa98xx->dsp_init) {
	case TFA98XX_DSP_INIT_STOPPED:
		str = "Stopped\n";
		break;
	case TFA98XX_DSP_INIT_RECOVER:
		str = "Recover requested\n";
		break;
	case TFA98XX_DSP_INIT_FAIL:
		str = "Failed init\n";
		break;
	case TFA98XX_DSP_INIT_PENDING:
		str = "Pending init\n";
		break;
	case TFA98XX_DSP_INIT_DONE:
		str = "Init complete\n";
		break;
	case TFA98XX_DSP_INIT_USEROFF:
		str = "User off\n";
		break;
	default:
		str = "Invalid\n";
	}

	pr_debug("[0x%x] dsp_state : %s\n", tfa98xx->i2c->addr, str);

	ret = simple_read_from_buffer(user_buf, count, ppos, str, strlen(str));
	return ret;
}

static ssize_t tfa98xx_dbgfs_dsp_state_set(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum tfa_error ret;
	char buf[32];
	const char start_cmd[] = "start";
	const char stop_cmd[] = "stop";
	const char mon_start_cmd[] = "monitor start";
	const char mon_stop_cmd[] = "monitor stop";
	int buf_size;

	buf_size = min(count, (size_t)(sizeof(buf) - 1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	/* Compare strings, excluding the trailing \0 */
	if (!strncmp(buf, start_cmd, sizeof(start_cmd) - 1)) {
		pr_info("[0x%x] Manual triggering of dsp start...\n", tfa98xx->i2c->addr);
		mutex_lock(&tfa98xx->dsp_lock);
		ret = tfa98xx_tfa_start(tfa98xx, tfa98xx->profile, tfa98xx->vstep);
		mutex_unlock(&tfa98xx->dsp_lock);
		pr_debug("[0x%x] tfa_dev_start complete: %d\n", tfa98xx->i2c->addr, ret);
	}
	else if (!strncmp(buf, stop_cmd, sizeof(stop_cmd) - 1)) {
		pr_info("[0x%x] Manual triggering of dsp stop...\n", tfa98xx->i2c->addr);
		mutex_lock(&tfa98xx->dsp_lock);
		ret = tfa_dev_stop(tfa98xx->tfa);
		mutex_unlock(&tfa98xx->dsp_lock);
		pr_debug("[0x%x] tfa_dev_stop complete: %d\n", tfa98xx->i2c->addr, ret);
	}
	else if (!strncmp(buf, mon_start_cmd, sizeof(mon_start_cmd) - 1)) {
		pr_info("[0x%x] Manual start of monitor thread...\n", tfa98xx->i2c->addr);
		queue_delayed_work(tfa98xx->tfa98xx_wq,
			&tfa98xx->monitor_work, HZ);
	}
	else if (!strncmp(buf, mon_stop_cmd, sizeof(mon_stop_cmd) - 1)) {
		pr_info("[0x%x] Manual stop of monitor thread...\n", tfa98xx->i2c->addr);
		cancel_delayed_work_sync(&tfa98xx->monitor_work);
	}
	else {
		return -EINVAL;
	}

	return count;
}

static ssize_t tfa98xx_dbgfs_fw_state_get(struct file *file,
	char __user *user_buf, size_t count,
	loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	char *str = NULL;

	switch (tfa98xx->dsp_fw_state) {
	case TFA98XX_DSP_FW_NONE:
		str = "None\n";
		break;
	case TFA98XX_DSP_FW_PENDING:
		str = "Pending\n";
		break;
	case TFA98XX_DSP_FW_FAIL:
		str = "Fail\n";
		break;
	case TFA98XX_DSP_FW_OK:
		str = "Ok\n";
		break;
	default:
		str = "Invalid\n";
	}

	pr_debug("[0x%x] fw_state : %s", tfa98xx->i2c->addr, str);

	return simple_read_from_buffer(user_buf, count, ppos, str, strlen(str));
}

#ifdef CONFIG_MTK_PLATFORM	
extern int mtk_spk_send_ipi_buf_to_dsp(void *buffer, uint32_t size);
extern int mtk_spk_recv_ipi_buf_from_dsp(int8_t *buffer, 
				int16_t size, uint32_t *data_len);
#endif

static ssize_t tfa98xx_dbgfs_rpc_read(struct file *file,
	char __user *user_buf, size_t count,
	loff_t *ppos)
{

#ifdef CONFIG_DEBUG_FS
	struct i2c_client *i2c = file->private_data;
#else
	struct i2c_client *i2c = PDE_DATA(file_inode(file));
#endif

	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int ret = 0;
	uint8_t *buffer = NULL;
	enum Tfa98xx_Error error;

	if (tfa98xx->tfa == NULL) {
		pr_debug("[0x%x] dsp is not available\n", tfa98xx->i2c->addr);
		return -ENODEV;
	}

	if (count == 0)
		return 0;

	buffer = kmalloc(count, GFP_KERNEL);
	if (buffer == NULL) {
		pr_debug("[0x%x] can not allocate memory\n", tfa98xx->i2c->addr);
		return -ENOMEM;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	
	if (tfa98xx->tfa->is_probus_device) {
		error = tfa98xx_recv_data_from_dsp(buffer, count);
	} else {
		error = tfa_dsp_msg_read(tfa98xx->tfa, count, buffer);
	}

	mutex_unlock(&tfa98xx->dsp_lock);
	if (error != Tfa98xx_Error_Ok) {
		pr_debug("[0x%x] tfa_dsp_msg_read error: %d\n", tfa98xx->i2c->addr, error);
		kfree(buffer);
		return -EFAULT;
	}

	ret = copy_to_user(user_buf, buffer, count);
	kfree(buffer);
	if (ret)
		return -EFAULT;

	*ppos += count;
	return count;
}

static ssize_t tfa98xx_dbgfs_rpc_send(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
#ifdef CONFIG_DEBUG_FS
	struct i2c_client *i2c = file->private_data;
#else
	struct i2c_client *i2c = PDE_DATA(file_inode(file));
#endif
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	TfaFileDsc_t *msg_file = NULL;
	enum Tfa98xx_Error error;
	int err = 0;

	if (tfa98xx->tfa == NULL) {
		pr_debug("[0x%x] dsp is not available\n", tfa98xx->i2c->addr);
		return -ENODEV;
	}

	if (count == 0)
		return 0;

	/* msg_file.name is not used */
	msg_file = kmalloc(count + sizeof(TfaFileDsc_t), GFP_KERNEL);
	if (msg_file == NULL) {
		pr_debug("[0x%x] can not allocate memory\n", tfa98xx->i2c->addr);
		return	-ENOMEM;
	}
	msg_file->size = count;

	if (copy_from_user(msg_file->data, user_buf, count)) {
//		kfree(msg_file);
		err = -EFAULT;
		goto exit;
	}

	mutex_lock(&tfa98xx->dsp_lock);

	if (tfa98xx->tfa->is_probus_device) {
		if (tfa98xx_device_count > 2 && !tfa98xx->is_primary)
			msg_file->data[0] |= 0x10;

		err = tfa98xx_send_data_to_dsp(msg_file->data, msg_file->size);

		if (err != 0)
			pr_err("[0x%x] dsp_msg error: %d\n", i2c->addr, err);

		mdelay(3);
		
	} else {	
		if ((msg_file->data[0] == 'M') && (msg_file->data[1] == 'G')) {
			error = tfaContWriteFile(tfa98xx->tfa, msg_file, 0, 0); /* int vstep_idx, int vstep_msg_idx both 0 */
			if (error != Tfa98xx_Error_Ok) {
				pr_debug("[0x%x] tfaContWriteFile error: %d\n", tfa98xx->i2c->addr, error);
				err = -EIO;
			}
		}
		else {
			error = tfa_dsp_msg(tfa98xx->tfa, msg_file->size, msg_file->data);
			if (error != Tfa98xx_Error_Ok) {
				pr_debug("[0x%x] tfa_dsp_msg error: %d\n", tfa98xx->i2c->addr, error);
				err = -EIO;
			}
		}
	}

	mutex_unlock(&tfa98xx->dsp_lock);

exit:
	kfree(msg_file);

	if (err)
		return err;
	return count;
}
/* -- RPC */

static int tfa98xx_dbgfs_pga_gain_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	unsigned int value;

	value = tfa_get_pga_gain(tfa98xx->tfa);
	if (value < 0)
		return -EINVAL;

	*val = value;
	return 0;
}

static int tfa98xx_dbgfs_pga_gain_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	uint16_t value;
	int err;

	value = val & 0xffff;
	if (value > 7)
		return -EINVAL;

	err = tfa_set_pga_gain(tfa98xx->tfa, value);
	if (err < 0)
		return -EINVAL;

	return 0;
}

#ifdef TFA98xx_calibrate
static ssize_t tfa98xx_state_store(struct file *file,
		const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	int value, i;
	char buf[32];
//	struct i2c_client *i2c = file->private_data;
//	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int buf_size;
	enum Tfa98xx_Error err;

	pr_err("%s calibrate start\n",__func__);
		if(tfa98xx_device_count==2 && (tfa98xx_global[0] == NULL || tfa98xx_global[1] == NULL))
	{
		pr_err("%s tfa98xx_global= NULL\n");
		return 0;
	}
	buf_size = min(count, (size_t)(sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	if (sscanf(buf, "%d", &value) == 1) {
		if(value == 1){
				err = tfa98xxCalibration((struct tfa98xx **)tfa98xx_global,cal_impedence);
				if (err != Tfa98xx_Error_Ok) {
					cal_impedence[0] = 0;
					cal_impedence[1] = 0;
					pr_err("calibration failed\n");
				}
				for(i = 0; i < tfa98xx_device_count; i++){
				pr_err("i2c addr is 0x%x,Calibration impedenc: %d\n", tfa98xx_global[i]->i2c->addr,cal_impedence[i]);
				}
				return count;
		}else {
			return -EINVAL;
		}
	}else {
		return -EINVAL;
	}
	
	return count;
}

static ssize_t tfa98xx_state_show(struct file *file,
		char __user *user_buf,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	char string[25];
//	itoa(cal_impedence[0],string,10);
	ret = sprintf(string,"%d:%d,%d:%d", 1, cal_impedence[0], 2, cal_impedence[1]); 
	

    pr_err("integer[0]=%d integer[1]=%d string=%s\n",cal_impedence[0],cal_impedence[1],string);
	ret = simple_read_from_buffer(user_buf, count, ppos, string, strlen(string));
	return ret;
	//return sprintf(buf, "%d\n",cal_impedence);
}
#endif
DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_calib_otc_fops, tfa98xx_dbgfs_otc_get,
	tfa98xx_dbgfs_otc_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_calib_mtpex_fops, tfa98xx_dbgfs_mtpex_get,
	tfa98xx_dbgfs_mtpex_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_calib_temp_fops, tfa98xx_dbgfs_temp_get,
	tfa98xx_dbgfs_temp_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_pga_gain_fops, tfa98xx_dbgfs_pga_gain_get,
	tfa98xx_dbgfs_pga_gain_set, "%llu\n");

static const struct file_operations tfa98xx_dbgfs_calib_start_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = tfa98xx_dbgfs_start_set,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_r_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_r_read,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_version_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_version_read,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_dsp_state_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_dsp_state_get,
	.write = tfa98xx_dbgfs_dsp_state_set,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_fw_state_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_fw_state_get,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_rpc_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_dbgfs_rpc_read,
	.write = tfa98xx_dbgfs_rpc_send,
	.llseek = default_llseek,
};

#ifdef TFA98xx_calibrate
static const struct file_operations tfa98xx_dbgfs_calib_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = tfa98xx_state_show,
	.write = tfa98xx_state_store,
	.llseek = default_llseek,
};
#endif
static void tfa98xx_debug_init(struct tfa98xx *tfa98xx, struct i2c_client *i2c)
{
	char name[50];

	scnprintf(name, MAX_CONTROL_NAME, "%s-%x", i2c->name, i2c->addr);
#ifdef CONFIG_DEBUG_FS
	tfa98xx->dbg_dir = debugfs_create_dir((char*)name, NULL);
	debugfs_create_file("OTC", S_IRUGO | S_IWUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_calib_otc_fops);
	debugfs_create_file("MTPEX", S_IRUGO | S_IWUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_calib_mtpex_fops);
	debugfs_create_file("TEMP", S_IRUGO | S_IWUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_calib_temp_fops);
	debugfs_create_file("calibrate", S_IRUGO | S_IWUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_calib_start_fops);
	debugfs_create_file("R", S_IRUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_r_fops);
	debugfs_create_file("version", S_IRUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_version_fops);
	debugfs_create_file("dsp-state", S_IRUGO | S_IWUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_dsp_state_fops);
	debugfs_create_file("fw-state", S_IRUGO | S_IWUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_fw_state_fops);
	debugfs_create_file("rpc", S_IRUGO | S_IWUGO, tfa98xx->dbg_dir,
		i2c, &tfa98xx_dbgfs_rpc_fops);
#ifdef TFA98xx_calibrate
	debugfs_create_file("impedance_state", S_IRUGO|S_IWUGO, tfa98xx->dbg_dir,
						i2c, &tfa98xx_dbgfs_calib_fops);
#endif

	if (tfa98xx->flags & TFA98XX_FLAG_SAAM_AVAILABLE) {
		dev_dbg(tfa98xx->dev, "Adding pga_gain debug interface\n");
		debugfs_create_file("pga_gain", S_IRUGO, tfa98xx->dbg_dir,
			tfa98xx->i2c,
			&tfa98xx_dbgfs_pga_gain_fops);
	}
#else
	tfa98xx->dbg_dir = proc_mkdir(name, NULL);
	proc_create_data("rpc", S_IRUGO|S_IWUGO, tfa98xx->dbg_dir,
		&tfa98xx_dbgfs_rpc_fops, i2c);
#endif
}

static void tfa98xx_debug_remove(struct tfa98xx *tfa98xx)
{
	if (tfa98xx->dbg_dir)
#ifdef CONFIG_DEBUG_FS
		debugfs_remove_recursive(tfa98xx->dbg_dir);
#else
        proc_remove(tfa98xx->dbg_dir);
#endif
}


/* copies the profile basename (i.e. part until .) into buf */
static void get_profile_basename(char* buf, char* profile)
{
	int cp_len = 0, idx = 0;
	char *pch;

	pch = strchr(profile, '.');
	idx = pch - profile;
	cp_len = (pch != NULL) ? idx : (int)strlen(profile);
	memcpy(buf, profile, cp_len);
	buf[cp_len] = 0;
}

/* return the profile name accociated with id from the profile list */
static int get_profile_from_list(char *buf, int id)
{
	struct tfa98xx_baseprofile *bprof = NULL;

	list_for_each_entry(bprof, &profile_list, list) {
		if (bprof->item_id == id) {
			strcpy(buf, bprof->basename);
			return 0;
		}
	}

	return -1;
}

/* search for the profile in the profile list */
static int is_profile_in_list(char *profile, int len)
{
	struct tfa98xx_baseprofile *bprof = NULL;

	list_for_each_entry(bprof, &profile_list, list) {

		if ((len == bprof->len) && (0 == strncmp(bprof->basename, profile, len)))
			return 1;
	}

	return 0;
}

/*
 * for the profile with id, look if the requested samplerate is
 * supported, if found return the (container)profile for this
 * samplerate, on error or if not found return -1
 */
static int get_profile_id_for_sr(int id, unsigned int rate)
{
	int idx = 0;
	struct tfa98xx_baseprofile *bprof = NULL;

	list_for_each_entry(bprof, &profile_list, list) {
		if (id == bprof->item_id) {
			idx = tfa98xx_get_fssel(rate);
			if (idx < 0) {
				/* samplerate not supported */
				return -1;
			}

			return bprof->sr_rate_sup[idx];
		}
	}

	/* profile not found */
	return -1;
}

/* check if this profile is a calibration profile */
static int is_calibration_profile(char *profile)
{
	if (strstr(profile, ".cal") != NULL)
		return 1;
	return 0;
}

/*
 * adds the (container)profile index of the samplerate found in
 * the (container)profile to a fixed samplerate table in the (mixer)profile
 */
static int add_sr_to_profile(struct tfa98xx *tfa98xx, char *basename, int len, int profile)
{
	struct tfa98xx_baseprofile *bprof = NULL;
	int idx = 0;
	unsigned int sr = 0;

	list_for_each_entry(bprof, &profile_list, list) {
		if ((len == bprof->len) && (0 == strncmp(bprof->basename, basename, len))) {
			/* add supported samplerate for this profile */
			sr = tfa98xx_get_profile_sr(tfa98xx->tfa, profile);
			if (!sr) {
				pr_err("unable to identify supported sample rate for %s\n", bprof->basename);
				return -1;
			}

			/* get the index for this samplerate */
			idx = tfa98xx_get_fssel(sr);
			if (idx < 0 || idx >= TFA98XX_NUM_RATES) {
				pr_err("invalid index for samplerate %d\n", idx);
				return -1;
			}

			/* enter the (container)profile for this samplerate at the corresponding index */
			bprof->sr_rate_sup[idx] = profile;

			pr_debug("added profile:samplerate = [%d:%d] for mixer profile: %s\n", profile, sr, bprof->basename);
		}
	}

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
static struct snd_soc_codec *snd_soc_kcontrol_codec(struct snd_kcontrol *kcontrol)
{
	return snd_kcontrol_chip(kcontrol);
}
#endif

static int tfa98xx_get_vstep(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	int mixer_profile = kcontrol->private_value;
	int ret = 0;
	int profile;

	profile = get_profile_id_for_sr(mixer_profile, tfa98xx->rate);
	if (profile < 0) {
		pr_err("tfa98xx: tfa98xx_get_vstep: invalid profile %d (mixer_profile=%d, rate=%d)\n", profile, mixer_profile, tfa98xx->rate);
		return -EINVAL;
	}

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		int vstep = tfa98xx->prof_vsteps[profile];

		ucontrol->value.integer.value[tfa98xx->tfa->dev_idx] =
			tfacont_get_max_vstep(tfa98xx->tfa, profile)
			- vstep - 1;
	}
	mutex_unlock(&tfa98xx_mutex);

	return ret;
}

static int tfa98xx_set_vstep(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	int mixer_profile = kcontrol->private_value;
	int profile;
	int err = 0;
	int change = 0;

	if (no_start != 0)
		return 0;

	profile = get_profile_id_for_sr(mixer_profile, tfa98xx->rate);
	if (profile < 0) {
		pr_err("tfa98xx: tfa98xx_set_vstep: invalid profile %d (mixer_profile=%d, rate=%d)\n", profile, mixer_profile, tfa98xx->rate);
		return -EINVAL;
	}

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		int vstep, vsteps;
		int ready = 0;
		int new_vstep;
		int value = ucontrol->value.integer.value[tfa98xx->tfa->dev_idx];

		vstep = tfa98xx->prof_vsteps[profile];
		vsteps = tfacont_get_max_vstep(tfa98xx->tfa, profile);

		if (vstep == vsteps - value - 1)
			continue;

		new_vstep = vsteps - value - 1;

		if (new_vstep < 0)
			new_vstep = 0;

		tfa98xx->prof_vsteps[profile] = new_vstep;

#ifndef TFA98XX_ALSA_CTRL_PROF_CHG_ON_VOL
		if (profile == tfa98xx->profile) {
#endif
			/* this is the active profile, program the new vstep */
			tfa98xx->vstep = new_vstep;
			mutex_lock(&tfa98xx->dsp_lock);
			tfa98xx_dsp_system_stable(tfa98xx->tfa, &ready);

			if (ready) {
				err = tfa98xx_tfa_start(tfa98xx, tfa98xx->profile, tfa98xx->vstep);
				if (err) {
					pr_err("Write vstep error: %d\n", err);
				}
				else {
					pr_debug("Succesfully changed vstep index!\n");
					change = 1;
				}
			}

			mutex_unlock(&tfa98xx->dsp_lock);
#ifndef TFA98XX_ALSA_CTRL_PROF_CHG_ON_VOL
		}
#endif
		pr_debug("%d: vstep:%d, (control value: %d) - profile %d\n",
			tfa98xx->tfa->dev_idx, new_vstep, value, profile);
	}

	if (change) {
		list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
			mutex_lock(&tfa98xx->dsp_lock);
			tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE, 0);
			mutex_unlock(&tfa98xx->dsp_lock);
		}
	}

	mutex_unlock(&tfa98xx_mutex);

	return change;
}

static int tfa98xx_info_vstep(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif

	int mixer_profile = tfa98xx_mixer_profile;
	int profile = get_profile_id_for_sr(mixer_profile, tfa98xx->rate);
	if (profile < 0) {
		pr_err("tfa98xx: tfa98xx_info_vstep: invalid profile %d (mixer_profile=%d, rate=%d)\n", profile, mixer_profile, tfa98xx->rate);
		return -EINVAL;
	}

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	mutex_lock(&tfa98xx_mutex);
	uinfo->count = tfa98xx_device_count;
	mutex_unlock(&tfa98xx_mutex);
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max(0, tfacont_get_max_vstep(tfa98xx->tfa, profile) - 1);
	pr_debug("vsteps count: %d [prof=%d]\n", tfacont_get_max_vstep(tfa98xx->tfa, profile),
		profile);
	return 0;
}

static int tfa98xx_get_profile(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&tfa98xx_mutex);
	ucontrol->value.integer.value[0] = tfa98xx_mixer_profile;
	mutex_unlock(&tfa98xx_mutex);

	return 0;
}

static int tfa98xx_set_profile(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	int change = 0;
	int new_profile;
	int prof_idx;
	int profile_count = tfa98xx_mixer_profiles;
	int profile = tfa98xx_mixer_profile;

	if (no_start != 0)
		return 0;

	new_profile = ucontrol->value.integer.value[0];
	if (new_profile == profile)
		return 0;

	if ((new_profile < 0) || (new_profile >= profile_count)) {
		pr_err("not existing profile (%d)\n", new_profile);
		return -EINVAL;
	}

	/* get the container profile for the requested sample rate */
	prof_idx = get_profile_id_for_sr(new_profile, tfa98xx->rate);
	if (prof_idx < 0) {
		pr_err("tfa98xx: sample rate [%d] not supported for this mixer profile [%d].\n", tfa98xx->rate, new_profile);
		return 0;
	}
	pr_debug("selected container profile [%d]\n", prof_idx);

	/* update mixer profile */
	tfa98xx_mixer_profile = new_profile;

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		int err;
		int ready = 0;

		/* update 'real' profile (container profile) */
		tfa98xx->profile = prof_idx;
		tfa98xx->vstep = tfa98xx->prof_vsteps[prof_idx];

		/* Don't call tfa_dev_start() if there is no clock. */
		mutex_lock(&tfa98xx->dsp_lock);
		tfa98xx_dsp_system_stable(tfa98xx->tfa, &ready);
		if (ready && (tfa_dev_get_state(tfa98xx->tfa) == TFA_STATE_OPERATING)) {
			/* Also re-enables the interrupts */
			err = tfa98xx_tfa_start(tfa98xx, prof_idx, tfa98xx->vstep);
			if (err) {
				pr_info("Write profile error: %d\n", err);
			}
			else {
				pr_debug("Changed to profile %d (vstep = %d)\n",
					prof_idx, tfa98xx->vstep);
				change = 1;
			}
		}
		mutex_unlock(&tfa98xx->dsp_lock);

		/* Flag DSP as invalidated as the profile change may invalidate the
		 * current DSP configuration. That way, further stream start can
		 * trigger a tfa_dev_start.
		 */
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_INVALIDATED;
	}

	if (change) {
		list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
			mutex_lock(&tfa98xx->dsp_lock);
			tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE, 0);
			mutex_unlock(&tfa98xx->dsp_lock);
		}
	}

	mutex_unlock(&tfa98xx_mutex);

	return change;
}

static int tfa98xx_info_profile(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	char profile_name[MAX_CONTROL_NAME] = { 0 };
	int count = tfa98xx_mixer_profiles, err = -1;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	err = get_profile_from_list(profile_name, uinfo->value.enumerated.item);
	if (err != 0)
		return -EINVAL;

	strcpy(uinfo->value.enumerated.name, profile_name);

	return 0;
}

static int tfa98xx_info_stop_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	mutex_lock(&tfa98xx_mutex);
	uinfo->count = tfa98xx_device_count;
	mutex_unlock(&tfa98xx_mutex);
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	return 0;
}

static int tfa98xx_get_stop_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tfa98xx *tfa98xx = NULL;

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		ucontrol->value.integer.value[tfa98xx->tfa->dev_idx] = (tfa98xx->dsp_init == TFA98XX_DSP_INIT_USEROFF);
	}
	mutex_unlock(&tfa98xx_mutex);

	return 0;
}

static int tfa98xx_set_stop_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tfa98xx *tfa98xx = NULL;

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry_reverse(tfa98xx, &tfa98xx_device_list, list) {
		int ready = 0;
		int i = tfa98xx->tfa->dev_idx;

		pr_debug("%d: %ld\n", i, ucontrol->value.integer.value[i]);

		tfa98xx_dsp_system_stable(tfa98xx->tfa, &ready);

		if ((ucontrol->value.integer.value[i] != 0)) {
			cancel_delayed_work_sync(&tfa98xx->monitor_work);

			cancel_delayed_work_sync(&tfa98xx->init_work);

			if(tfa98xx->flags & TFA98XX_FLAG_ADAPT_NOISE_MODE)
				cancel_delayed_work_sync(&tfa98xx->nmodeupdate_work);

			if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK)
				continue;

			mutex_lock(&tfa98xx->dsp_lock);
			if (ready) tfa_dev_stop(tfa98xx->tfa);
			tfa98xx->dsp_init = TFA98XX_DSP_INIT_USEROFF;
			mutex_unlock(&tfa98xx->dsp_lock);
		}

		ucontrol->value.integer.value[i] = 0;
	}
	mutex_unlock(&tfa98xx_mutex);

	return 1;
}

static int tfa98xx_info_cal_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	mutex_lock(&tfa98xx_mutex);
	uinfo->count = tfa98xx_device_count;
	mutex_unlock(&tfa98xx_mutex);
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xffff; /* 16 bit value */

	return 0;
}

static int tfa98xx_set_cal_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tfa98xx *tfa98xx = NULL;

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		enum tfa_error err;
		int i = tfa98xx->tfa->dev_idx;

		tfa98xx->cal_data = (uint16_t)ucontrol->value.integer.value[i];

		mutex_lock(&tfa98xx->dsp_lock);
		err = tfa98xx_write_re25(tfa98xx->tfa, tfa98xx->cal_data);
		tfa98xx->set_mtp_cal = (err != tfa_error_ok);
		if (tfa98xx->set_mtp_cal == false) {
			pr_info("Calibration value (%d) set in mtp\n",
				tfa98xx->cal_data);
		}
		mutex_unlock(&tfa98xx->dsp_lock);
	}
	mutex_unlock(&tfa98xx_mutex);

	return 1;
}

static int tfa98xx_get_cal_ctl(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tfa98xx *tfa98xx = NULL;

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		mutex_lock(&tfa98xx->dsp_lock);
		ucontrol->value.integer.value[tfa98xx->tfa->dev_idx] = tfa_dev_mtp_get(tfa98xx->tfa, TFA_MTP_RE25_PRIM);
		mutex_unlock(&tfa98xx->dsp_lock);
	}
	mutex_unlock(&tfa98xx_mutex);

	return 0;
}

#ifdef TFA9894_STEREO
#define CHIP_SELECTOR_STEREO	(0)
#define CHIP_SELECTOR_LEFT	(1)
#define CHIP_SELECTOR_RIGHT	(2)
#define CHIP_SELECTOR_NONE	(3)
#define CHIP_LEFT_ADDR		(0x34)
#define CHIP_RIGHT_ADDR		(0x35)


static int tfa98xx_info_stereo_ctl(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_info *uinfo)
{
        uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
        uinfo->count = 1;
        uinfo->value.integer.min = 0;
        uinfo->value.integer.max = 3;
        return 0;
}

static int tfa98xx_set_stereo_ctl(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
    struct tfa98xx *tfa98xx;
	int selector;

	selector = ucontrol->value.integer.value[0];
	
    mutex_lock(&tfa98xx_mutex);
    list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		pr_err("tfa98xx: set_stereo_ctl %d, 0x%x\n", selector, tfa98xx->i2c->addr);
		if (selector == CHIP_SELECTOR_LEFT) {
			if (tfa98xx->i2c->addr == CHIP_LEFT_ADDR) 
				tfa98xx->flags |= TFA98XX_FLAG_CHIP_SELECTED;
			else 
				tfa98xx->flags &= ~TFA98XX_FLAG_CHIP_SELECTED; 
		}
		else if (selector == CHIP_SELECTOR_RIGHT) {
	        if (tfa98xx->i2c->addr == CHIP_RIGHT_ADDR)
	            tfa98xx->flags |= TFA98XX_FLAG_CHIP_SELECTED;
	        else
	            tfa98xx->flags &= ~TFA98XX_FLAG_CHIP_SELECTED;			
		}
		else if (selector == CHIP_SELECTOR_NONE) {
            tfa98xx->flags &= ~TFA98XX_FLAG_CHIP_SELECTED;			
		}
		//else if (selector == CHIP_SELECTOR_RCV) {
		//	if (tfa98xx->i2c->addr == CHIP_RCV_ADDR) {
	    //        tfa98xx->flags |= TFA98XX_FLAG_CHIP_SELECTED;
		//		tfa98xx->profile = 1;
		//	}
	    //    else
	    //        tfa98xx->flags &= ~TFA98XX_FLAG_CHIP_SELECTED;
		//}
		else {
				tfa98xx->flags |= TFA98XX_FLAG_CHIP_SELECTED;
		}
	}
    mutex_unlock(&tfa98xx_mutex);

    return 1;
}

static int tfa98xx_get_stereo_ctl(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
    struct tfa98xx *tfa98xx;

    mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {  
			pr_err("tfa98xx: tfa98xx_get_stereo_ctl %d, 0x%x\n", tfa98xx->flags, tfa98xx->i2c->addr);
	    	ucontrol->value.integer.value[0] = tfa98xx->flags;
	}
	mutex_unlock(&tfa98xx_mutex);

    return 0;
}
#endif
static int tfa98xx_create_controls(struct tfa98xx *tfa98xx)
{
	int prof, nprof, mix_index = 0;
	int  nr_controls = 0, id = 0;
	char *name = NULL;
	struct tfa98xx_baseprofile *bprofile = NULL;
#ifdef TFA98xx_dsp_bypass
	int ret = 0;
#endif

	/* Create the following controls:
	 *	- enum control to select the active profile
	 *	- one volume control for each profile hosting a vstep
	 *	- Stop control on TFA1 devices
	 */

	nr_controls = 2; /* Profile and stop control */
#ifdef TFA9894_STEREO
	nr_controls +=1 ; 
#endif

	if (tfa98xx->flags & TFA98XX_FLAG_CALIBRATION_CTL)
		nr_controls += 1; /* calibration */

	/* allocate the tfa98xx_controls base on the nr of profiles */
	nprof = tfa_cnt_get_dev_nprof(tfa98xx->tfa);
	for (prof = 0; prof < nprof; prof++) {
		if (tfacont_get_max_vstep(tfa98xx->tfa, prof))
			nr_controls++; /* Playback Volume control */
	}

	tfa98xx_controls = devm_kzalloc(tfa98xx->codec->dev,
		nr_controls * sizeof(tfa98xx_controls[0]), GFP_KERNEL);
	if (!tfa98xx_controls)
		return -ENOMEM;

	/* Create a mixer item for selecting the active profile */
	name = devm_kzalloc(tfa98xx->codec->dev, MAX_CONTROL_NAME, GFP_KERNEL);
	if (!name)
		return -ENOMEM;
	scnprintf(name, MAX_CONTROL_NAME, "%s Profile", tfa98xx->fw.name);
	tfa98xx_controls[mix_index].name = name;
	tfa98xx_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	tfa98xx_controls[mix_index].info = tfa98xx_info_profile;
	tfa98xx_controls[mix_index].get = tfa98xx_get_profile;
	tfa98xx_controls[mix_index].put = tfa98xx_set_profile;
	// tfa98xx_controls[mix_index].private_value = profs; /* save number of profiles */
	mix_index++;

	/* create mixer items for each profile that has volume */
	for (prof = 0; prof < nprof; prof++) {
		/* create an new empty profile */
		bprofile = devm_kzalloc(tfa98xx->codec->dev, sizeof(*bprofile), GFP_KERNEL);
		if (!bprofile)
			return -ENOMEM;

		bprofile->len = 0;
		bprofile->item_id = -1;
		INIT_LIST_HEAD(&bprofile->list);

		/* copy profile name into basename until the . */
		get_profile_basename(bprofile->basename, tfa_cont_profile_name(tfa98xx, prof));
		bprofile->len = strlen(bprofile->basename);

		/*
		 * search the profile list for a profile with basename, if it is not found then
		 * add it to the list and add a new mixer control (if it has vsteps)
		 * also, if it is a calibration profile, do not add it to the list
		 */
		if ((is_profile_in_list(bprofile->basename, bprofile->len) == 0) &&
			is_calibration_profile(tfa_cont_profile_name(tfa98xx, prof)) == 0) {
			/* the profile is not present, add it to the list */
			list_add(&bprofile->list, &profile_list);
			bprofile->item_id = id++;

			pr_debug("profile added [%d]: %s\n", bprofile->item_id, bprofile->basename);

			if (tfacont_get_max_vstep(tfa98xx->tfa, prof)) {
				name = devm_kzalloc(tfa98xx->codec->dev, MAX_CONTROL_NAME, GFP_KERNEL);
				if (!name)
					return -ENOMEM;

				scnprintf(name, MAX_CONTROL_NAME, "%s %s Playback Volume",
					tfa98xx->fw.name, bprofile->basename);

				tfa98xx_controls[mix_index].name = name;
				tfa98xx_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
				tfa98xx_controls[mix_index].info = tfa98xx_info_vstep;
				tfa98xx_controls[mix_index].get = tfa98xx_get_vstep;
				tfa98xx_controls[mix_index].put = tfa98xx_set_vstep;
				tfa98xx_controls[mix_index].private_value = bprofile->item_id; /* save profile index */
				mix_index++;
			}
		}

		/* look for the basename profile in the list of mixer profiles and add the
		   container profile index to the supported samplerates of this mixer profile */
		add_sr_to_profile(tfa98xx, bprofile->basename, bprofile->len, prof);
	}

	/* set the number of user selectable profiles in the mixer */
	tfa98xx_mixer_profiles = id;

	/* Create a mixer item for stop control on TFA1 */
	name = devm_kzalloc(tfa98xx->codec->dev, MAX_CONTROL_NAME, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	scnprintf(name, MAX_CONTROL_NAME, "%s Stop", tfa98xx->fw.name);
	tfa98xx_controls[mix_index].name = name;
	tfa98xx_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	tfa98xx_controls[mix_index].info = tfa98xx_info_stop_ctl;
	tfa98xx_controls[mix_index].get = tfa98xx_get_stop_ctl;
	tfa98xx_controls[mix_index].put = tfa98xx_set_stop_ctl;
	mix_index++;

	if (tfa98xx->flags & TFA98XX_FLAG_CALIBRATION_CTL) {
		name = devm_kzalloc(tfa98xx->codec->dev, MAX_CONTROL_NAME, GFP_KERNEL);
		if (!name)
			return -ENOMEM;

		scnprintf(name, MAX_CONTROL_NAME, "%s Calibration", tfa98xx->fw.name);
		tfa98xx_controls[mix_index].name = name;
		tfa98xx_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
		tfa98xx_controls[mix_index].info = tfa98xx_info_cal_ctl;
		tfa98xx_controls[mix_index].get = tfa98xx_get_cal_ctl;
		tfa98xx_controls[mix_index].put = tfa98xx_set_cal_ctl;
		mix_index++;
	}
#ifdef TFA9894_STEREO
        tfa98xx_controls[mix_index].name = "TFA_CHIP_SELECTOR";
        tfa98xx_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
        tfa98xx_controls[mix_index].info = tfa98xx_info_stereo_ctl;
        tfa98xx_controls[mix_index].get = tfa98xx_get_stereo_ctl;
        tfa98xx_controls[mix_index].put = tfa98xx_set_stereo_ctl;
        mix_index++;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)

#ifdef TFA98xx_dsp_bypass
	ret = snd_soc_add_component_controls(tfa98xx->codec,
		tfa987x_algo_filter_mixer_controls, ARRAY_SIZE(tfa987x_algo_filter_mixer_controls));
#endif

	return snd_soc_add_component_controls(tfa98xx->codec,
		tfa98xx_controls, mix_index);
#else

#ifdef TFA98xx_dsp_bypass
	ret = snd_soc_add_codec_controls(tfa98xx->codec,
		tfa987x_algo_filter_mixer_controls, ARRAY_SIZE(tfa987x_algo_filter_mixer_controls));
#endif

	return snd_soc_add_codec_controls(tfa98xx->codec,
		tfa98xx_controls, mix_index);
#endif
}

static void *tfa98xx_devm_kstrdup(struct device *dev, char *buf)
{
	char *str = devm_kzalloc(dev, strlen(buf) + 1, GFP_KERNEL);
	if (!str)
		return str;
	memcpy(str, buf, strlen(buf));
	return str;
}

static int tfa98xx_append_i2c_address(struct device *dev,
	struct i2c_client *i2c,
	struct snd_soc_dapm_widget *widgets,
	int num_widgets,
	struct snd_soc_dai_driver *dai_drv,
	int num_dai)
{
	char buf[50] = {0};
	int i;
	int i2cbus = i2c->adapter->nr;
	int addr = i2c->addr;
	if (dai_drv && num_dai > 0)
		for (i = 0; i < num_dai; i++) {
			memset(buf, 0x00, sizeof(buf));
			snprintf(buf, 50, "%s-%x-%x", dai_drv[i].name, i2cbus,
				addr);
			dai_drv[i].name = tfa98xx_devm_kstrdup(dev, (char*)buf);

			memset(buf, 0x00, sizeof(buf));
			snprintf(buf, 50, "%s-%x-%x",
				dai_drv[i].playback.stream_name,
				i2cbus, addr);
			dai_drv[i].playback.stream_name = tfa98xx_devm_kstrdup(dev, (char*)buf);

			memset(buf, 0x00, sizeof(buf));
			snprintf(buf, 50, "%s-%x-%x",
				dai_drv[i].capture.stream_name,
				i2cbus, addr);
			dai_drv[i].capture.stream_name = tfa98xx_devm_kstrdup(dev, (char*)buf);
		}

	/* the idea behind this is convert:
	 * SND_SOC_DAPM_AIF_IN("AIF IN", "AIF Playback", 0, SND_SOC_NOPM, 0, 0),
	 * into:
	 * SND_SOC_DAPM_AIF_IN("AIF IN", "AIF Playback-2-36", 0, SND_SOC_NOPM, 0, 0),
	 */
	if (widgets && num_widgets > 0)
		for (i = 0; i < num_widgets; i++) {
			if (!widgets[i].sname)
				continue;
			if ((widgets[i].id == snd_soc_dapm_aif_in)
				|| (widgets[i].id == snd_soc_dapm_aif_out)) {
				snprintf(buf, 50, "%s-%x-%x", widgets[i].sname,
					i2cbus, addr);
				widgets[i].sname = tfa98xx_devm_kstrdup(dev, (char*)buf);
			}
		}

	return 0;
}

static struct snd_soc_dapm_widget tfa98xx_dapm_widgets_common[] = {
	/* Stream widgets */
	SND_SOC_DAPM_AIF_IN("AIF IN", "AIF Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF OUT", "AIF Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUTPUT("OUTL"),
	SND_SOC_DAPM_INPUT("AEC Loopback"),
};

static struct snd_soc_dapm_widget tfa98xx_dapm_widgets_stereo[] = {
	SND_SOC_DAPM_OUTPUT("OUTR"),
};

static struct snd_soc_dapm_widget tfa98xx_dapm_widgets_saam[] = {
	SND_SOC_DAPM_INPUT("SAAM MIC"),
};

static struct snd_soc_dapm_widget tfa9888_dapm_inputs[] = {
	SND_SOC_DAPM_INPUT("DMIC1"),
	SND_SOC_DAPM_INPUT("DMIC2"),
	SND_SOC_DAPM_INPUT("DMIC3"),
	SND_SOC_DAPM_INPUT("DMIC4"),
};

static const struct snd_soc_dapm_route tfa98xx_dapm_routes_common[] = {
	{ "OUTL", NULL, "AIF IN" },
	{ "AIF OUT", NULL, "AEC Loopback" },
};

static const struct snd_soc_dapm_route tfa98xx_dapm_routes_saam[] = {
	{ "AIF OUT", NULL, "SAAM MIC" },
};

static const struct snd_soc_dapm_route tfa98xx_dapm_routes_stereo[] = {
	{ "OUTR", NULL, "AIF IN" },
};

static const struct snd_soc_dapm_route tfa9888_input_dapm_routes[] = {
	{ "AIF OUT", NULL, "DMIC1" },
	{ "AIF OUT", NULL, "DMIC2" },
	{ "AIF OUT", NULL, "DMIC3" },
	{ "AIF OUT", NULL, "DMIC4" },
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,2,0)
static struct snd_soc_dapm_context *snd_soc_codec_get_dapm(struct snd_soc_codec *codec)
{
	return &codec->dapm;
}
#endif

static void tfa98xx_add_widgets(struct tfa98xx *tfa98xx)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(tfa98xx->codec);
#else
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(tfa98xx->codec);
#endif
	struct snd_soc_dapm_widget *widgets;
	unsigned int num_dapm_widgets = ARRAY_SIZE(tfa98xx_dapm_widgets_common);

	widgets = devm_kzalloc(&tfa98xx->i2c->dev,
		sizeof(struct snd_soc_dapm_widget) *
		ARRAY_SIZE(tfa98xx_dapm_widgets_common),
		GFP_KERNEL);
	if (!widgets)
		return;
	memcpy(widgets, tfa98xx_dapm_widgets_common,
		sizeof(struct snd_soc_dapm_widget) *
		ARRAY_SIZE(tfa98xx_dapm_widgets_common));

	tfa98xx_append_i2c_address(&tfa98xx->i2c->dev,
		tfa98xx->i2c,
		widgets,
		num_dapm_widgets,
		NULL,
		0);

	snd_soc_dapm_new_controls(dapm, widgets,
		ARRAY_SIZE(tfa98xx_dapm_widgets_common));
	snd_soc_dapm_add_routes(dapm, tfa98xx_dapm_routes_common,
		ARRAY_SIZE(tfa98xx_dapm_routes_common));

	if (tfa98xx->flags & TFA98XX_FLAG_STEREO_DEVICE) {
		snd_soc_dapm_new_controls(dapm, tfa98xx_dapm_widgets_stereo,
			ARRAY_SIZE(tfa98xx_dapm_widgets_stereo));
		snd_soc_dapm_add_routes(dapm, tfa98xx_dapm_routes_stereo,
			ARRAY_SIZE(tfa98xx_dapm_routes_stereo));
	}

	if (tfa98xx->flags & TFA98XX_FLAG_MULTI_MIC_INPUTS) {
		snd_soc_dapm_new_controls(dapm, tfa9888_dapm_inputs,
			ARRAY_SIZE(tfa9888_dapm_inputs));
		snd_soc_dapm_add_routes(dapm, tfa9888_input_dapm_routes,
			ARRAY_SIZE(tfa9888_input_dapm_routes));
	}

	if (tfa98xx->flags & TFA98XX_FLAG_SAAM_AVAILABLE) {
		snd_soc_dapm_new_controls(dapm, tfa98xx_dapm_widgets_saam,
			ARRAY_SIZE(tfa98xx_dapm_widgets_saam));
		snd_soc_dapm_add_routes(dapm, tfa98xx_dapm_routes_saam,
			ARRAY_SIZE(tfa98xx_dapm_routes_saam));
	}
}

/* I2C wrapper functions */
enum Tfa98xx_Error tfa98xx_write_register16(struct tfa_device *tfa,
	unsigned char subaddress,
	unsigned short value)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	struct tfa98xx *tfa98xx = NULL;
	int ret;
	int retries = I2C_RETRIES;

	if (tfa == NULL) {
		pr_err("No device available\n");
		return Tfa98xx_Error_Fail;
	}

	tfa98xx = (struct tfa98xx *)tfa->data;
	if (!tfa98xx || !tfa98xx->regmap) {
		pr_err("No tfa98xx regmap available\n");
		return Tfa98xx_Error_Bad_Parameter;
	}
retry:
	ret = regmap_write(tfa98xx->regmap, subaddress, value);
	if (ret < 0) {
		pr_warn("i2c error, retries left: %d\n", retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
		return Tfa98xx_Error_Fail;
	}
	if (tfa98xx_kmsg_regs)
		dev_dbg(&tfa98xx->i2c->dev, "  WR reg=0x%02x, val=0x%04x %s\n",
			subaddress, value,
			ret < 0 ? "Error!!" : "");

	if (tfa98xx_ftrace_regs)
		tfa98xx_trace_printk("\tWR	   reg=0x%02x, val=0x%04x %s\n",
			subaddress, value,
			ret < 0 ? "Error!!" : "");
	return error;
}

enum Tfa98xx_Error tfa98xx_read_register16(struct tfa_device *tfa,
	unsigned char subaddress,
	unsigned short *val)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	struct tfa98xx *tfa98xx = NULL;
	unsigned int value;
	int retries = I2C_RETRIES;
	int ret;

	if (tfa == NULL) {
		pr_err("No device available\n");
		return Tfa98xx_Error_Fail;
	}

	tfa98xx = (struct tfa98xx *)tfa->data;
	if (!tfa98xx || !tfa98xx->regmap) {
		pr_err("No tfa98xx regmap available\n");
		return Tfa98xx_Error_Bad_Parameter;
	}
retry:
	ret = regmap_read(tfa98xx->regmap, subaddress, &value);
	if (ret < 0) {
		pr_warn("i2c error at subaddress 0x%x, retries left: %d\n", subaddress, retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
		return Tfa98xx_Error_Fail;
	}
	*val = value & 0xffff;

	if (tfa98xx_kmsg_regs)
		dev_dbg(&tfa98xx->i2c->dev, "RD   reg=0x%02x, val=0x%04x %s\n",
			subaddress, *val,
			ret < 0 ? "Error!!" : "");
	if (tfa98xx_ftrace_regs)
		tfa98xx_trace_printk("\tRD	   reg=0x%02x, val=0x%04x %s\n",
			subaddress, *val,
			ret < 0 ? "Error!!" : "");

	return error;
}


/*
 * init external dsp
 */
enum Tfa98xx_Error
	tfa98xx_init_dsp(struct tfa_device *tfa)
{
	return Tfa98xx_Error_Not_Supported;
}

int tfa98xx_get_dsp_status(struct tfa_device *tfa)
{
	return 0;
}

/*
 * write external dsp message
 */
enum Tfa98xx_Error
	tfa98xx_write_dsp(struct tfa_device *tfa, int num_bytes, const char *command_buffer)
{
	int err = 0;
	uint8_t *buffer = NULL;

	buffer = kmalloc(num_bytes, GFP_KERNEL);
	if ( buffer == NULL ) {
		pr_err("[0x%x] can not allocate memory\n", tfa->slave_address);
		return	Tfa98xx_Error_Fail;
	}

	memcpy(buffer ,command_buffer, num_bytes);
	
	err = tfa98xx_send_data_to_dsp(buffer, (size_t)num_bytes);
	if (err) {
		pr_err("[0x%x] dsp_msg error: %d\n", tfa->slave_address, err);
	}
	
	kfree(buffer);
	mdelay(3);
	return Tfa98xx_Error_Ok;
}

/*
 * read external dsp message
 */
enum Tfa98xx_Error
	tfa98xx_read_dsp(struct tfa_device *tfa, int num_bytes, unsigned char *result_buffer)
{
	int ret = 0;
	
	ret = tfa98xx_recv_data_from_dsp(result_buffer, (size_t)num_bytes);
	
	if (ret) {
		pr_err("[0x%x] dsp_msg_read error: %d\n", tfa->slave_address, ret);
		return Tfa98xx_Error_Fail;
	}
	
	return Tfa98xx_Error_Ok;
}
/*
 * write/read external dsp message
 */
enum Tfa98xx_Error
	tfa98xx_writeread_dsp(struct tfa_device *tfa, int command_length, void *command_buffer,
		int result_length, void *result_buffer)
{
	return Tfa98xx_Error_Not_Supported;
}

enum Tfa98xx_Error tfa98xx_read_data(struct tfa_device *tfa,
	unsigned char reg,
	int len, unsigned char value[])
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	struct tfa98xx *tfa98xx = NULL;
	struct i2c_client *tfa98xx_client = NULL;
	int err;
	int tries = 0;
	unsigned char *reg_buf = NULL;
	struct i2c_msg msgs[] = {
		{
			.flags = 0,
			.len = 1,
			.buf = NULL,
		}, {
			.flags = I2C_M_RD,
			.len = len,
			.buf = value,
		},
	};
	reg_buf = (unsigned char *)kmalloc(sizeof(reg), GFP_DMA);	  //GRP_KERNEL	also works,
	if (!reg_buf) {
		return -ENOMEM;;
	}

	*reg_buf = reg;
	msgs[0].buf = reg_buf;

	if (tfa == NULL) {
		pr_err("No device available\n");
		return Tfa98xx_Error_Fail;
	}

	tfa98xx = (struct tfa98xx *)tfa->data;
	if (tfa98xx->i2c) {
		tfa98xx_client = tfa98xx->i2c;
		msgs[0].addr = tfa98xx_client->addr;
		msgs[1].addr = tfa98xx_client->addr;

		do {
			err = i2c_transfer(tfa98xx_client->adapter, msgs,
				ARRAY_SIZE(msgs));
			if (err != ARRAY_SIZE(msgs))
				msleep_interruptible(I2C_RETRY_DELAY);
		} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

		if (err != ARRAY_SIZE(msgs)) {
			dev_err(&tfa98xx_client->dev, "read transfer error %d\n",
				err);
			error = Tfa98xx_Error_Fail;
		}

		if (tfa98xx_kmsg_regs)
			dev_dbg(&tfa98xx_client->dev, "RD-DAT reg=0x%02x, len=%d\n",
				reg, len);
		if (tfa98xx_ftrace_regs)
			tfa98xx_trace_printk("\t\tRD-DAT reg=0x%02x, len=%d\n",
				reg, len);
	}
	else {
		pr_err("No device available\n");
		error = Tfa98xx_Error_Fail;
	}
	kfree(reg_buf);
	return error;
}

enum Tfa98xx_Error tfa98xx_write_raw(struct tfa_device *tfa,
	int len,
	const unsigned char data[])
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	struct tfa98xx *tfa98xx = NULL;
	int ret;
	int retries = I2C_RETRIES;


	if (tfa == NULL) {
		pr_err("No device available\n");
		return Tfa98xx_Error_Fail;
	}

	tfa98xx = (struct tfa98xx *)tfa->data;

retry:
	ret = i2c_master_send(tfa98xx->i2c, data, len);
	if (ret < 0) {
		pr_warn("i2c error, retries left: %d\n", retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
	}

	if (ret == len) {
		if (tfa98xx_kmsg_regs)
			dev_dbg(&tfa98xx->i2c->dev, "  WR-RAW len=%d\n", len);
		if (tfa98xx_ftrace_regs)
			tfa98xx_trace_printk("\t\tWR-RAW len=%d\n", len);
		return Tfa98xx_Error_Ok;
	}
	pr_err("  WR-RAW (len=%d) Error I2C send size mismatch %d\n", len, ret);
	error = Tfa98xx_Error_Fail;

	return error;
}

/* Interrupts management */

static void tfa98xx_interrupt_enable_tfa2(struct tfa98xx *tfa98xx, bool enable)
{
	/* Only for 0x72 we need to enable NOCLK interrupts */
	if (tfa98xx->flags & TFA98XX_FLAG_REMOVE_PLOP_NOISE)
		tfa_irq_ena(tfa98xx->tfa, tfa9912_irq_stnoclk, enable);

	if (tfa98xx->flags & TFA98XX_FLAG_LP_MODES) {
		tfa_irq_ena(tfa98xx->tfa, 36, enable); /* FIXME: IELP0 does not excist for 9912 */
		tfa_irq_ena(tfa98xx->tfa, tfa9912_irq_stclpr, enable);
	}
}

/* Check if tap-detection can and shall be enabled.
 * Configure SPK interrupt accordingly or setup polling mode
 * Tap-detection shall be active if:
 *	- the service is enabled (tapdet_open), AND
 *	- the current profile is a tap-detection profile
 * On TFA1 familiy of devices, activating tap-detection means enabling the SPK
 * interrupt if available.
 * We also update the tapdet_enabled and tapdet_poll variables.
 */
static void tfa98xx_tapdet_check_update(struct tfa98xx *tfa98xx)
{
	unsigned int enable = false;

	/* Support tap-detection on TFA1 family of devices */
	if ((tfa98xx->flags & TFA98XX_FLAG_TAPDET_AVAILABLE) == 0)
		return;

	if (tfa98xx->tapdet_open &&
		(tfa98xx->tapdet_profiles & (1 << tfa98xx->profile)))
		enable = true;

	if (!gpio_is_valid(tfa98xx->irq_gpio)) {
		/* interrupt not available, setup polling mode */
		tfa98xx->tapdet_poll = true;
		if (enable)
			queue_delayed_work(tfa98xx->tfa98xx_wq,
				&tfa98xx->tapdet_work, HZ / 10);
		else
			cancel_delayed_work_sync(&tfa98xx->tapdet_work);
		dev_dbg(tfa98xx->codec->dev,
			"Polling for tap-detection: %s (%d; 0x%x, %d)\n",
			enable ? "enabled" : "disabled",
			tfa98xx->tapdet_open, tfa98xx->tapdet_profiles,
			tfa98xx->profile);

	}
	else {
		dev_dbg(tfa98xx->codec->dev,
			"Interrupt for tap-detection: %s (%d; 0x%x, %d)\n",
			enable ? "enabled" : "disabled",
			tfa98xx->tapdet_open, tfa98xx->tapdet_profiles,
			tfa98xx->profile);
		/*	enabled interrupt */
		tfa_irq_ena(tfa98xx->tfa, tfa9912_irq_sttapdet, enable);
	}

	/* check disabled => enabled transition to clear pending events */
	if (!tfa98xx->tapdet_enabled && enable) {
		/* clear pending event if any */
		tfa_irq_clear(tfa98xx->tfa, tfa9912_irq_sttapdet);
	}

	if (!tfa98xx->tapdet_poll)
		tfa_irq_ena(tfa98xx->tfa, tfa9912_irq_sttapdet, 1); /* enable again */
}

/* global enable / disable interrupts */
static void tfa98xx_interrupt_enable(struct tfa98xx *tfa98xx, bool enable)
{
	if (tfa98xx->flags & TFA98XX_FLAG_SKIP_INTERRUPTS)
		return;

	if (tfa98xx->tfa->tfa_family == 2)
		tfa98xx_interrupt_enable_tfa2(tfa98xx, enable);
}

/* Firmware management */
static void tfa98xx_container_loaded(const struct firmware *cont, void *context)
{
	TfaContainer_t *container = NULL;
	struct tfa98xx *tfa98xx = context;
	enum tfa_error tfa_err;
	int container_size;
	int ret;

	tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_FAIL;

	if (!cont) {
		pr_err("Failed to read %s\n", fw_name);
		return;
	}

	pr_debug("loaded %s - size: %zu\n", fw_name, cont->size);

	mutex_lock(&tfa98xx_mutex);
	if (tfa98xx_container == NULL) {
		container = kzalloc(cont->size, GFP_KERNEL);
		if (container == NULL) {
			mutex_unlock(&tfa98xx_mutex);
			release_firmware(cont);
			pr_err("Error allocating memory\n");
			return;
		}

		container_size = cont->size;
		memcpy(container, cont->data, container_size);
		release_firmware(cont);

		pr_debug("%.2s%.2s\n", container->version, container->subversion);
		pr_debug("%.8s\n", container->customer);
		pr_debug("%.8s\n", container->application);
		pr_debug("%.8s\n", container->type);
		pr_debug("%d ndev\n", container->ndev);
		pr_debug("%d nprof\n", container->nprof);

		tfa_err = tfa_load_cnt(container, container_size);
		if (tfa_err != tfa_error_ok) {
			mutex_unlock(&tfa98xx_mutex);
			kfree(container);
			dev_err(tfa98xx->dev, "Cannot load container file, aborting\n");
			return;
		}

		tfa98xx_container = container;
	}
	else {
		pr_debug("container file already loaded...\n");
		container = tfa98xx_container;
		release_firmware(cont);
	}
	mutex_unlock(&tfa98xx_mutex);

	tfa98xx->tfa->cnt = container;

	/*
		i2c transaction limited to 64k
		(Documentation/i2c/writing-clients)
	*/
	tfa98xx->tfa->buffer_size = 65536;

	/* DSP messages via i2c */
	tfa98xx->tfa->has_msg = 0;

	if (tfa_dev_probe(tfa98xx->i2c->addr, tfa98xx->tfa) != 0) {
		dev_err(tfa98xx->dev, "Failed to probe TFA98xx @ 0x%.2x\n", tfa98xx->i2c->addr);
		return;
	}

	tfa98xx->tfa->dev_idx = tfa_cont_get_idx(tfa98xx->tfa);
	if (tfa98xx->tfa->dev_idx < 0) {
		dev_err(tfa98xx->dev, "Failed to find TFA98xx @ 0x%.2x in container file\n", tfa98xx->i2c->addr);
		return;
	}

	/* Enable debug traces */
	tfa98xx->tfa->verbose = trace_level & 1;

	/* prefix is the application name from the cnt */
	tfa_cnt_get_app_name(tfa98xx->tfa, tfa98xx->fw.name);

	/* set default profile/vstep */
	tfa98xx->profile = 0;
	tfa98xx->vstep = 0;

	/* Override default profile if requested */
	if (strcmp(dflt_prof_name, "")) {
		unsigned int i;
		int nprof = tfa_cnt_get_dev_nprof(tfa98xx->tfa);
		for (i = 0; i < nprof; i++) {
			if (strcmp(tfa_cont_profile_name(tfa98xx, i),
				dflt_prof_name) == 0) {
				tfa98xx->profile = i;
				dev_info(tfa98xx->dev,
					"changing default profile to %s (%d)\n",
					dflt_prof_name, tfa98xx->profile);
				break;
			}
		}
		if (i >= nprof)
			dev_info(tfa98xx->dev,
				"Default profile override failed (%s profile not found)\n",
				dflt_prof_name);
	}

	tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_OK;
	pr_debug("Firmware init complete\n");

	if (no_start != 0)
		return;

#ifndef DISABLE_TFA98XX_ALSA_SUPPORT
	/* Only controls for master device */
	if (tfa98xx->tfa->dev_idx == 0)
		tfa98xx_create_controls(tfa98xx);
#endif

	tfa98xx_inputdev_check_register(tfa98xx);

	if (tfa_is_cold(tfa98xx->tfa) == 0) {
		pr_debug("Warning: device 0x%.2x is still warm\n", tfa98xx->i2c->addr);
		tfa_reset(tfa98xx->tfa);
	}

	/* Preload settings using internal clock on TFA2 */
	if (tfa98xx->tfa->tfa_family == 2 &&
		tfa98xx->tfa->is_probus_device == 0) {
		mutex_lock(&tfa98xx->dsp_lock);
		ret = tfa98xx_tfa_start(tfa98xx, tfa98xx->profile, tfa98xx->vstep);
		if (ret == Tfa98xx_Error_Not_Supported)
			tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_FAIL;
		ret = tfa_dev_stop(tfa98xx->tfa);
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_STOPPED;
		mutex_unlock(&tfa98xx->dsp_lock);
	}		
	tfa98xx_interrupt_enable(tfa98xx, true);
}

static int tfa98xx_load_container(struct tfa98xx *tfa98xx)
{
	tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_PENDING;

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
		fw_name, tfa98xx->dev, GFP_KERNEL,
		tfa98xx, tfa98xx_container_loaded);
}


static void tfa98xx_tapdet(struct tfa98xx *tfa98xx)
{
	unsigned int tap_pattern;
	int btn;

	/* check tap pattern (BTN_0 is "error" wrong tap indication */
	tap_pattern = tfa_get_tap_pattern(tfa98xx->tfa);
	switch (tap_pattern) {
	case 0xffffffff:
		pr_info("More than 4 taps detected! (flagTapPattern = -1)\n");
		btn = BTN_0;
		break;
	case 0xfffffffe:
	case 0xfe:
		pr_info("Illegal tap detected!\n");
		btn = BTN_0;
		break;
	case 0:
		pr_info("Unrecognized pattern! (flagTapPattern = 0)\n");
		btn = BTN_0;
		break;
	default:
		pr_info("Detected pattern: %d\n", tap_pattern);
		btn = BTN_0 + tap_pattern;
		break;
	}

	input_report_key(tfa98xx->input, btn, 1);
	input_report_key(tfa98xx->input, btn, 0);
	input_sync(tfa98xx->input);

	/* acknowledge event done by clearing interrupt */

}

static void tfa98xx_tapdet_work(struct work_struct *work)
{
	struct tfa98xx *tfa98xx;

	//TODO check is this is still needed for tap polling
	tfa98xx = container_of(work, struct tfa98xx, tapdet_work.work);

	if (tfa_irq_get(tfa98xx->tfa, tfa9912_irq_sttapdet))
		tfa98xx_tapdet(tfa98xx);

	queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->tapdet_work, HZ / 10);
}
static void tfa98xx_nmode_update_work(struct work_struct *work)
{
/* the AP will be wake up by workQ, we should be disabled it to save power. */
/* you can enable it for debug purpose. */
#if 1
	struct tfa98xx *tfa98xx;

	//MCH_TO_TEST, checking if noise mode update is required or not
	tfa98xx = container_of(work, struct tfa98xx, nmodeupdate_work.work);
	mutex_lock(&tfa98xx->dsp_lock);	
	tfa_adapt_noisemode(tfa98xx->tfa);
	mutex_unlock(&tfa98xx->dsp_lock);
	queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->nmodeupdate_work,5 * HZ);
#endif
}
static void tfa98xx_monitor(struct work_struct *work)
{
/* the AP will be wake up by workQ, we should be disabled it to save power. */
/* you can enable it for debug purpose. */
#if 1
	struct tfa98xx *tfa98xx;
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;

	tfa98xx = container_of(work, struct tfa98xx, monitor_work.work);

	/* Check for tap-detection - bypass monitor if it is active */
	if (!tfa98xx->input) {
		mutex_lock(&tfa98xx->dsp_lock);
		error = tfa_status(tfa98xx->tfa);	
		mutex_unlock(&tfa98xx->dsp_lock);
		if (error == Tfa98xx_Error_DSP_not_running) {
			if (tfa98xx->dsp_init == TFA98XX_DSP_INIT_DONE) {
				tfa98xx->dsp_init = TFA98XX_DSP_INIT_RECOVER;
				queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->init_work, 0);
			}
		}
	}

	/* reschedule */
	queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->monitor_work, 5 * HZ);
#endif
}

#ifdef CONFIG_MTK_PLATFORM
static void tfa98xx_adsp_send_calib_values(void)
{
	struct tfa98xx *tfa98xx_pri = NULL, *tfa98xx_sec = NULL;
	int dsp_cal_value = 0, value = 0;
	unsigned char bytes[9] = {0};

	if (tfa98xx_device_count > 2)
		return;

	tfa98xx_pri = list_last_entry(&tfa98xx_device_list, struct tfa98xx, list);
	if (tfa98xx_device_count == 2) {
		/*left/right selector*/
		if (!tfa98xx_pri->is_primary){
			tfa98xx_sec = tfa98xx_pri;
			tfa98xx_pri = list_prev_entry(tfa98xx_pri, list);
		}
		else {
			tfa98xx_sec = list_prev_entry(tfa98xx_pri, list);
		}
	}

	/*No need to go further in these conditions*/
	if (tfa98xx_pri == NULL ||
		tfa98xx_pri->dsp_init == TFA98XX_DSP_INIT_DONE ||
		!tfa98xx_pri->tfa->is_probus_device) {
		return;
	}

	/* If calibration is set to once we load from MTP, else send zero's */
	if (TFA_GET_BF(tfa98xx_pri->tfa, MTPEX) == 1)
	{
		value = tfa_dev_mtp_get(tfa98xx_pri->tfa, TFA_MTP_RE25);

		/*rdc value should within 4ohm ~ 9ohm*/
		if (value > 9216 || value < 4096) value = 7399;

		dsp_cal_value = (value * 65536) / 1000;

		bytes[3] = (uint8_t)((dsp_cal_value >> 16) & 0xff);
		bytes[4] = (uint8_t)((dsp_cal_value >> 8) & 0xff);
		bytes[5] = (uint8_t)(dsp_cal_value & 0xff);
		/* We have to copy it for both channels. Even when mono! */
		bytes[6] = bytes[3];
		bytes[7] = bytes[4];
		bytes[8] = bytes[5];

		dev_info(&tfa98xx_pri->i2c->dev, "%s: cal value 0x%x\n", __func__, dsp_cal_value);
	}

	if (tfa98xx_sec != NULL &&
		TFA_GET_BF(tfa98xx_sec->tfa, MTPEX) == 1)
	{
		value = tfa_dev_mtp_get(tfa98xx_sec->tfa, TFA_MTP_RE25);
		/*rdc value should within 4ohm ~ 9ohm*/
		if (value > 9216 || value < 4096) value = 7399;

		dsp_cal_value = (value * 65536) / 1000;

		bytes[6] = (uint8_t)((dsp_cal_value >> 16) & 0xff);
		bytes[7] = (uint8_t)((dsp_cal_value >> 8) & 0xff);
		bytes[8] = (uint8_t)(dsp_cal_value & 0xff);

		dev_info(&tfa98xx_sec->i2c->dev, "%s: cal value 0x%x\n", __func__, dsp_cal_value);
	}

	if (bytes[3] != 0)
	{
		bytes[0] = 0x00;
		bytes[1] = 0x81;
		bytes[2] = 0x05;

		dev_info(&tfa98xx_pri->i2c->dev, "%s: finally send values to adsp\n", __func__);

		tfa98xx_send_data_to_dsp(bytes, sizeof(bytes));
	}
}
#else
static uint8_t bytes[3*3+1] = {0};

enum Tfa98xx_Error tfa98xx_adsp_send_calib_values(void)
{
	struct tfa98xx *tfa98xx;
	int ret = 0;
	int value = 0, nr, dsp_cal_value = 0;

	/* if the calibration value was sent to host DSP, we clear flag only (stereo case). */
	if ((tfa98xx_device_count > 1) && (tfa98xx_device_count == bytes[0])) {
		pr_info("The calibration value was sent to host DSP.\n");
		bytes[0] = 0;
		return Tfa98xx_Error_Ok;
	}

	/* read calibrated impendance from all devices. */
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		struct tfa_device *tfa = tfa98xx->tfa;
		if ((TFA_GET_BF(tfa, MTPEX) == 1) &&(tfa98xx->i2c->addr == 0x34)){
			value = tfa_dev_mtp_get(tfa, TFA_MTP_RE25);
			dsp_cal_value = (value * 65536) / 1000;
			pr_info("Device 0x%x cal value is 0x%d,value:%d\n", tfa98xx->i2c->addr, dsp_cal_value,value);
			nr = 4;
			bytes[nr++] = (uint8_t)((dsp_cal_value >> 16) & 0xff);
			bytes[nr++] = (uint8_t)((dsp_cal_value >> 8) & 0xff);
			bytes[nr++] = (uint8_t)(dsp_cal_value & 0xff);
			bytes[0] += 1;
		}
		
		if ((TFA_GET_BF(tfa, MTPEX) == 1) &&(tfa98xx->i2c->addr == 0x35)){
			value = tfa_dev_mtp_get(tfa, TFA_MTP_RE25);
			dsp_cal_value = (value * 65536) / 1000;
			pr_info("Device 0x%x cal value is 0x%d,value:%d\n", tfa98xx->i2c->addr, dsp_cal_value,value);
			nr = 7;
			bytes[nr++] = (uint8_t)((dsp_cal_value >> 16) & 0xff);
			bytes[nr++] = (uint8_t)((dsp_cal_value >> 8) & 0xff);
			bytes[nr++] = (uint8_t)(dsp_cal_value & 0xff);
			bytes[0] += 1;
			
		}
	}

	/*for mono case, we will copy primary channel data to secondary channel. */
	if (1 == tfa98xx_device_count) {
		memcpy(&bytes[7], &bytes[4], sizeof(char)*3);
	}
	pr_info("tfa98xx_device_count=%d  bytes[0]=%d\n", tfa98xx_device_count, bytes[0]);

	/* we will send it to host DSP algorithm once calibraion value loaded from all device. */
	if (tfa98xx_device_count == bytes[0]) {
		bytes[1] = 0x00;
		bytes[2] = 0x81;
		bytes[3] = 0x05;

		pr_info("calibration value send to host DSP.\n");
		ret = send_tfa_cal_in_band(&bytes[1], sizeof(bytes) - 1);
		msleep(10);

		/* for mono case, we should clear flag here. */
		if (1 == tfa98xx_device_count)
			bytes[0] = 0;

	} else {
		pr_err("load calibration data from device failed.\n");
		ret = Tfa98xx_Error_Bad_Parameter;
	}

	return ret;
}
#endif

static void tfa98xx_dsp_init(struct tfa98xx *tfa98xx)
{
	int ret;
	bool failed = false;
	bool reschedule = false;
	bool sync = false;

	if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK) {
		pr_debug("Skipping tfa_dev_start (no FW: %d)\n", tfa98xx->dsp_fw_state);
		return;
	}

	if (tfa98xx->dsp_init == TFA98XX_DSP_INIT_DONE) {
		pr_debug("Stream already started, skipping DSP power-on\n");
		return;
	}

	if (tfa98xx->dsp_init == TFA98XX_DSP_INIT_USEROFF) {
		tfa98xx_sync_count++;
		pr_debug("User set to skip DSP power-on\n");
		return;
	}

	/*send MTP values to adsp*/
	if (tfa98xx->tfa->is_probus_device) {
		tfa98xx_adsp_send_calib_values();
	}

	mutex_lock(&tfa98xx->dsp_lock);

	tfa98xx->dsp_init = TFA98XX_DSP_INIT_PENDING;

	if (tfa98xx->init_count < TF98XX_MAX_DSP_START_TRY_COUNT) {
		/* directly try to start DSP */
		ret = tfa98xx_tfa_start(tfa98xx, tfa98xx->profile, tfa98xx->vstep);
		if (ret == Tfa98xx_Error_Not_Supported) {
			tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_FAIL;
			dev_err(&tfa98xx->i2c->dev, "Failed starting device\n");
			failed = true;
		}
		else if (ret != Tfa98xx_Error_Ok) {
			/* It may fail as we may not have a valid clock at that
			 * time, so re-schedule and re-try later.
			 */
			dev_err(&tfa98xx->i2c->dev,
				"tfa_dev_start failed! (err %d) - %d\n",
				ret, tfa98xx->init_count);
			reschedule = true;
		}
		else {
			sync = true;

			/* Subsystem ready, tfa init complete */
			tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;
			dev_info(&tfa98xx->i2c->dev,
				"tfa_dev_start success (%d)\n",
				tfa98xx->init_count);
			/* cancel other pending init works */
			cancel_delayed_work(&tfa98xx->init_work);
			tfa98xx->init_count = 0;
		}
	}
	else {
		/* exceeded max number ot start tentatives, cancel start */
		dev_err(&tfa98xx->i2c->dev,
			"Failed starting device (%d)\n",
			tfa98xx->init_count);
		failed = true;
	}
	if (reschedule) {
		/* reschedule this init work for later */
		queue_delayed_work(tfa98xx->tfa98xx_wq,
			&tfa98xx->init_work,
			msecs_to_jiffies(5));
		tfa98xx->init_count++;
	}
	if (failed) {
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_FAIL;
		/* cancel other pending init works */
		cancel_delayed_work(&tfa98xx->init_work);
		tfa98xx->init_count = 0;
	}
	mutex_unlock(&tfa98xx->dsp_lock);

	if (sync) {
		/* check if all devices have started */
		bool do_sync = false;
		mutex_lock(&tfa98xx_mutex);

		if (tfa98xx_sync_count < tfa98xx_device_count)
			tfa98xx_sync_count++;

		do_sync = (tfa98xx_sync_count >= tfa98xx_device_count);
		mutex_unlock(&tfa98xx_mutex);

		/* when all devices have started then unmute */
		if (do_sync) {
			tfa98xx_sync_count = 0;
			list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
				mutex_lock(&tfa98xx->dsp_lock);
				tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE, 0);

				/*
				 * start monitor thread to check IC status bit
				 * periodically, and re-init IC to recover if
				 * needed.
				 */
				if (tfa98xx->tfa->tfa_family == 1)
					queue_delayed_work(tfa98xx->tfa98xx_wq,
						&tfa98xx->monitor_work,
						1 * HZ);
				mutex_unlock(&tfa98xx->dsp_lock);
			}
		} else {
#ifdef DISABLE_TFA98XX_ALSA_SUPPORT 
			queue_delayed_work(tfa98xx->tfa98xx_wq,
				&tfa98xx->unmute_work,
				msecs_to_jiffies(20));
#endif
		}
	}


	return;
}


static void tfa98xx_dsp_init_work(struct work_struct *work)
{
	struct tfa98xx *tfa98xx = container_of(work, struct tfa98xx, init_work.work);

	tfa98xx_dsp_init(tfa98xx);
}

#ifdef DISABLE_TFA98XX_ALSA_SUPPORT 
static void tfa98xx_unmue_target_device(struct work_struct *work)
{
	struct tfa98xx *tfa98xx = container_of(work, struct tfa98xx, unmute_work.work);
	struct tfa98xx *tfa98xx_pri = NULL, *tfa98xx_sec = NULL;

	tfa98xx_pri = list_last_entry(&tfa98xx_device_list, struct tfa98xx, list);
	if (tfa98xx_device_count == 2) {
		/*left/right selector*/
		if (!tfa98xx_pri->is_primary) {
			tfa98xx_sec = tfa98xx_pri;
			tfa98xx_pri = list_prev_entry(tfa98xx_pri, list);
		}
		else {
			tfa98xx_sec = list_prev_entry(tfa98xx_pri, list);
		}

		if ((tfa98xx_pri == NULL) || (tfa98xx_sec == NULL)) {
			pr_err("address is NULL (tfa98xx_pri=%p  tfa98xx_sec=%p)\n", tfa98xx_pri, tfa98xx_sec);
			return;
		}

		/* for stereo project, if workQ is timeout and only 1 device is enabled, we will be un-muting this device directly. */
		if (((TFA_GET_BF(tfa98xx_pri->tfa, PWDN) == 0) && (TFA_GET_BF(tfa98xx_sec->tfa, PWDN) == 1)) || \
			 ((TFA_GET_BF(tfa98xx_pri->tfa, PWDN) == 1) && (TFA_GET_BF(tfa98xx_sec->tfa, PWDN) == 0))) {
			mutex_lock(&tfa98xx->dsp_lock);
			tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE, 0);
			mutex_unlock(&tfa98xx->dsp_lock);
		}
	}
}
#endif

static void tfa98xx_interrupt(struct work_struct *work)
{
	struct tfa98xx *tfa98xx = container_of(work, struct tfa98xx, interrupt_work.work);

	pr_info("\n");

	if (tfa98xx->flags & TFA98XX_FLAG_TAPDET_AVAILABLE) {
		/* check for tap interrupt */
		if (tfa_irq_get(tfa98xx->tfa, tfa9912_irq_sttapdet)) {
			tfa98xx_tapdet(tfa98xx);

			/* clear interrupt */
			tfa_irq_clear(tfa98xx->tfa, tfa9912_irq_sttapdet);
		}
	} /* TFA98XX_FLAG_TAPDET_AVAILABLE */

	if (tfa98xx->flags & TFA98XX_FLAG_REMOVE_PLOP_NOISE) {
		int start_triggered;

		mutex_lock(&tfa98xx->dsp_lock);
		start_triggered = tfa_plop_noise_interrupt(tfa98xx->tfa, tfa98xx->profile, tfa98xx->vstep);
		/* Only enable when the return value is 1, otherwise the interrupt is triggered twice */
		if (start_triggered)
			tfa98xx_interrupt_enable(tfa98xx, true);
		mutex_unlock(&tfa98xx->dsp_lock);
	} /* TFA98XX_FLAG_REMOVE_PLOP_NOISE */

	if (tfa98xx->flags & TFA98XX_FLAG_LP_MODES) {
		tfa_lp_mode_interrupt(tfa98xx->tfa);
	} /* TFA98XX_FLAG_LP_MODES */

	/* unmask interrupts masked in IRQ handler */
	tfa_irq_unmask(tfa98xx->tfa);
}

static int tfa98xx_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	struct snd_soc_component *codec = dai->component;
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	unsigned int sr;
	int len, prof, nprof, idx = 0;
	char *basename = NULL;
	u64 formats;
	int err;

	/*
	 * Support CODEC to CODEC links,
	 * these are called with a NULL runtime pointer.
	 */
	if (!substream->runtime)
		return 0;

	if (pcm_no_constraint != 0)
		return 0;

	switch (pcm_sample_format) {
	case 0:
		formats = SNDRV_PCM_FMTBIT_S16_LE;
		break;
	case 1:
		formats = SNDRV_PCM_FMTBIT_S24_LE;
		break;
	case 2:
		formats = SNDRV_PCM_FMTBIT_S32_LE;
		break;
	default:
		formats = TFA98XX_FORMATS;
		break;
	}

	err = snd_pcm_hw_constraint_mask64(substream->runtime,
		SNDRV_PCM_HW_PARAM_FORMAT, formats);
	if (err < 0)
		return err;

	if (no_start != 0)
		return 0;

	if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK) {
		dev_info(codec->dev, "Container file not loaded\n");
		return -EINVAL;
	}

	basename = kzalloc(MAX_CONTROL_NAME, GFP_KERNEL);
	if (!basename)
		return -ENOMEM;

	/* copy profile name into basename until the . */
	get_profile_basename(basename, tfa_cont_profile_name(tfa98xx, tfa98xx->profile));
	len = strlen(basename);

	/* loop over all profiles and get the supported samples rate(s) from
	 * the profiles with the same basename
	 */
	nprof = tfa_cnt_get_dev_nprof(tfa98xx->tfa);
	tfa98xx->rate_constraint.list = &tfa98xx->rate_constraint_list[0];
	tfa98xx->rate_constraint.count = 0;
	for (prof = 0; prof < nprof; prof++) {
		if (0 == strncmp(basename, tfa_cont_profile_name(tfa98xx, prof), len)) {
			/* Check which sample rate is supported with current profile,
			 * and enforce this.
			 */
			sr = tfa98xx_get_profile_sr(tfa98xx->tfa, prof);
			if (!sr)
				dev_info(codec->dev, "Unable to identify supported sample rate\n");

			if (tfa98xx->rate_constraint.count >= TFA98XX_NUM_RATES) {
				dev_err(codec->dev, "too many sample rates\n");
			}
			else {
				tfa98xx->rate_constraint_list[idx++] = sr;
				tfa98xx->rate_constraint.count += 1;
			}
		}
	}

	kfree(basename);
#ifdef CONFIG_MTK_PLATFORM
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE,
		&tfa98xx->rate_constraint);
#else
	return 0;
#endif
}

static int tfa98xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
	int clk_id, unsigned int freq, int dir)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(codec_dai->component);
#else
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec_dai->codec);
#endif
	tfa98xx->sysclk = freq;
	return 0;
}

static int tfa98xx_set_tdm_slot(struct snd_soc_dai *dai, unsigned int tx_mask,
	unsigned int rx_mask, int slots, int slot_width)
{
	pr_debug("\n");
	return 0;
}

static int tfa98xx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(dai->component);
	struct snd_soc_component *codec = dai->component;
#else
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_codec *codec = dai->codec;
#endif
	pr_debug("fmt=0x%x\n", fmt);

	/* Supported mode: regular I2S, slave, or PDM */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
			dev_err(codec->dev, "Invalid Codec master mode\n");
			return -EINVAL;
		}
		break;
	case SND_SOC_DAIFMT_PDM:
		break;
	default:
		dev_err(codec->dev, "Unsupported DAI format %d\n",
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	tfa98xx->audio_mode = fmt & SND_SOC_DAIFMT_FORMAT_MASK;

	return 0;
}

static int tfa98xx_get_fssel(unsigned int rate)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(rate_to_fssel); i++) {
		if (rate_to_fssel[i].rate == rate) {
			return rate_to_fssel[i].fssel;
		}
	}
	return -EINVAL;
}

static int tfa98xx_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	struct snd_soc_component *codec = dai->component;
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	unsigned int rate;
	int prof_idx;

	/* Supported */
	rate = params_rate(params);
	tfa98xx->tfa->bitwidth = params_width(params);
	tfa98xx->tfa->dynamicTDMmode = pcm_sample_format;
	pr_debug("Requested rate: %d, sample size: %d, physical size: %d\n",
		rate, snd_pcm_format_width(params_format(params)),
		snd_pcm_format_physical_width(params_format(params)));

	if (no_start != 0)
		return 0;
	/* set TDM bit width */
	pr_debug("%s: Requested width: %d\n", __func__,
			params_width(params));
	if ((tfa98xx->tfa->dynamicTDMmode == 3) && tfa_dev_set_tdm_bitwidth(tfa98xx->tfa,tfa98xx->tfa->bitwidth))
		return -EINVAL;
	/* check if samplerate is supported for this mixer profile */
	prof_idx = get_profile_id_for_sr(tfa98xx_mixer_profile, rate);
	if (prof_idx < 0) {
		pr_err("tfa98xx: invalid sample rate %d.\n", rate);
		return -EINVAL;
	}
	pr_debug("mixer profile:container profile = [%d:%d]\n", tfa98xx_mixer_profile, prof_idx);


	/* update 'real' profile (container profile) */
	tfa98xx->profile = prof_idx;

	/* update to new rate */
	tfa98xx->rate = rate;

	return 0;
}

#ifdef TFA98xx_F0_tracking
typedef struct nxpTfa98xx_LiveData {
        unsigned short statusRegister;/**  status flags (from i2c register ) */
        unsigned short statusFlags;   /**  Masked bit word, see Tfa98xx_SpeakerBoostStatusFlags */
        int  speakerTemp;           		 /**  Current Speaker Temperature value */
        short icTemp;                 		/**  Current ic/die Temperature value (from i2c register ) */
        int shortOnMips;			  /**  increments each time a MIPS problem is detected on the DSP */
        int DrcSupport;				  /**  Is the DRC extension valid */
        struct Tfa98xx_DrcStateInfo drcState;   /**  DRC extension */
} nxpTfa98xx_LiveData_t;

typedef struct LiveDataList {
	char *address;
	int isMemtrackItem;
} LiveDataList_t;

int gf0[2] = {0};
int gtemp[2] = {0};

typedef enum Tfa98xx_Error Tfa98xx_Error_t;

const char* Tfa98xx_GetErrorString(Tfa98xx_Error_t error)
{
	const char* pErrStr;

	switch (error)
	{
	case Tfa98xx_Error_Ok:
		pErrStr = "Ok";
		break;
	case Tfa98xx_Error_Fail:
		pErrStr = "generic_failure";
		break;
	case Tfa98xx_Error_NoClock:
		pErrStr = "No_I2S_Clock";
		break;
	case Tfa98xx_Error_AmpOn:
		pErrStr = "amp_still_running";
		break;
	case Tfa98xx_Error_DSP_not_running:
		pErrStr = "DSP_not_running";
		break;
	case Tfa98xx_Error_Bad_Parameter:
		pErrStr = "Bad_Parameter";
		break;
	case Tfa98xx_Error_NotOpen:
		pErrStr = "NotOpen";
		break;
	case Tfa98xx_Error_InUse:
		pErrStr = "InUse";
		break;
	case Tfa98xx_Error_RpcBusy:
		pErrStr = "RpcBusy";
		break;
	case Tfa98xx_Error_RpcModId:
		pErrStr = "RpcModId";
		break;
	case Tfa98xx_Error_RpcParamId:
		pErrStr = "RpcParamId";
		break;
	case Tfa98xx_Error_RpcInvalidCC:
		pErrStr = "RpcInvalidCC";
		break;
	case Tfa98xx_Error_RpcInvalidSeq:
		pErrStr = "RpcInvalidSec";
		break;
	case Tfa98xx_Error_RpcInvalidParam:
		pErrStr = "RpcInvalidParam";
		break;
	case Tfa98xx_Error_RpcBufferOverflow:
		pErrStr = "RpcBufferOverflow";
		break;
	case Tfa98xx_Error_RpcCalibBusy:
		pErrStr = "RpcCalibBusy";
		break;
	case Tfa98xx_Error_RpcCalibFailed:
		pErrStr = "RpcCalibFailed";
		break;
	case Tfa98xx_Error_Not_Supported:
		pErrStr = "Not_Supported";
		break;
	case Tfa98xx_Error_I2C_Fatal:
		pErrStr = "I2C_Fatal";
		break;
	case Tfa98xx_Error_I2C_NonFatal:
		pErrStr = "I2C_NonFatal";
		break;
	case Tfa98xx_Error_StateTimedOut:
		pErrStr = "WaitForState_TimedOut";
		break;
	default:
		pErrStr = "Unspecified error";
		break;
	}
	return pErrStr;
}

nxpTfaLiveData_t* tfaGetLiveDataItem(nxpTfaDescPtr_t *dsc, nxpTfaContainer_t *cont)
{
        nxpTfaLiveData_t *liveD;

        liveD = (nxpTfaLiveData_t *)(dsc->offset+(uint8_t *)cont);
        return liveD;
}

int compare_strings(char *buffer, char *name, int buffersize)
{
	int i=0;
	//char new_buf[50];
	char *new_buf = kmalloc(buffersize+1, GFP_KERNEL);
	while(i < buffersize) {
		new_buf[i] = (char)tolower(buffer[i]);
		i++;
	}

	if (strcmp(new_buf, name) == 0) {
		kfree(new_buf);
		return 1;
	} else {
		kfree(new_buf);
		return 0;
	}
}

Tfa98xx_Error_t tfa98xx_get_live_data_raw(struct tfa98xx *tfa98xx,int dev_idx, unsigned char *bytes, int length)
{
    Tfa98xx_Error_t err = Tfa98xx_Error_Ok;

    /* Get memtrack */
    err = tfa_dsp_cmd_id_write_read(tfa98xx->tfa, MODULE_FRAMEWORK, FW_PAR_ID_GET_MEMTRACK, length, bytes);

    return err;
}

Tfa98xx_Error_t tfa98xx_get_live_data(struct tfa98xx *tfa98xx,int dev_idx, int *live_data, int *nr_of_items)
{
	Tfa98xx_Error_t err = Tfa98xx_Error_Ok;
	nxpTfaLiveDataList_t *lData = NULL;
	nxpTfaLiveData_t *liveD = NULL;
	nxpTfaBitfield_t bf;
	struct nxpTfa98xx_Memtrack_data *mRecord = kmalloc(sizeof(struct nxpTfa98xx_Memtrack_data), GFP_KERNEL);
	unsigned char *bytes = kmalloc(sizeof(unsigned char) * ((LSMODEL_MAX_WORDS * 3)+1),GFP_KERNEL);
	int *data = kmalloc(sizeof(int) * 151,GFP_KERNEL); /* 150 + 1*/
	//LiveDataList_t order[MEMTRACK_MAX_WORDS+1];
	LiveDataList_t *order = kmalloc(sizeof(LiveDataList_t)*(MEMTRACK_MAX_WORDS+1) ,GFP_KERNEL);
	unsigned int i, j, stateSize, length=0;

	// init
	for(i=0; i<MEMTRACK_MAX_WORDS+1; i++) {
		order[i].isMemtrackItem = 0;
	}

	/* Max2 device */
	if (tfa98xx->tfa->tfa_family == 2) {
		stateSize = MEMTRACK_MAX_WORDS;
		/* Make sure the cnt has a livedata section */
		if(tfa98xx->tfa->cnt->nliveData > 0) {
			lData=tfaContGetDevLiveDataList(tfa98xx->tfa->cnt, dev_idx, 0);
			if(lData != NULL) {
				i=0;
				length = lData->length-1u;
				/* The number of memtrack items */
				for(j=0; j<lData->length-1u; j++) {
					//liveD = tfaGetLiveDataItem(&lData->list[j], tfa98xx->tfa->cnt);
					liveD = (nxpTfaLiveData_t *)((&lData->list[j])->offset+(uint8_t *)(tfa98xx->tfa->cnt));

					if (compare_strings(liveD->name, "buildinmemtrack", 25) == 0) {
						mRecord->scalingFactor[i] = liveD->scalefactor;
						if(mRecord->scalingFactor[i] == 0) {
							pr_err("Error, Scale factor zero, possible division by zero.\n");
						}

						/* Create a list to get the order of DSP memtrack items and I2C registers/bitfields */
						/* if not in any list */
						if (liveD->addrs[0] >= '0' && liveD->addrs[0] <= '9') {
							order[i].isMemtrackItem = 1;
						} else {
							order[i].address = liveD->addrs;
						}
						i++;
					}
				}
			} else {
				/* This idx has no livedata section */
				kfree(bytes);
				kfree(data);
				kfree(order);
				kfree(mRecord);
				return Tfa98xx_Error_Other;
			}

			/* Get the raw livedata */
			/* Length = nr of items * 3 bytes (24bit dsp) + 3 bytes for Elapsed time (first 3 bytes) */
			err = tfa98xx_get_live_data_raw(tfa98xx,dev_idx, bytes, (length * 3)+3);
			if(err != Tfa98xx_Error_Ok) {
				*nr_of_items = 0;
				kfree(bytes);
				kfree(data);
				kfree(order);
				kfree(mRecord);
				return err;
			}

			/* Skip the first 3 bytes (this is the Elapsed time, not memtrack data) */
			tfa98xx_convert_bytes2data(3 * length, bytes+3, data);

			/* Take every memtrack item and devide with the scalingfactor
			 * There are lData->length-1 number of memtrack items, BUT
			 * that does not mean lData->length-1 number of DSP items!
			 * Because the list is created in DSP order we need to get the data in order
			 * (and not skip when there is a i2c item in between)
			 */
			j=0;
			for(i=0;i<length;i++) {
				//liveD = tfaGetLiveDataItem(&lData->list[i], tfa98xx->tfa->cnt);

				liveD = (nxpTfaLiveData_t *)((&lData->list[i])->offset+(uint8_t *)(tfa98xx->tfa->cnt));

				if (compare_strings(liveD->name, "buildinmemtrack", 25) != 0) {
					length -= 1;
				}

				if(order[i].isMemtrackItem) {
					pr_err("tfa98xx data[%d] = %d, factor = %d\n",j,data[j] , mRecord->scalingFactor[i]);
					live_data[i] = data[j] / mRecord->scalingFactor[i];
					j++;
				} else {
					/* Get the Bitfield value */
					bf.field = tfaContBfEnum(order[i].address, tfa98xx->tfa->rev); //TODO solve for max1
					err = tfaRunReadBitfield(tfa98xx->tfa, &bf);
					live_data[i] = bf.value / mRecord->scalingFactor[i];
					pr_err("tfa98xx data-----\n");
				}
			}
			*nr_of_items = length;

		} else {
			*nr_of_items = 0;
			kfree(bytes);
			kfree(data);
			kfree(order);
			kfree(mRecord);
			return Tfa98xx_Error_Other;
		}
	} else {
		pr_err("%s:tfa_family is one, stop work\n", __func__);
	}

	kfree(bytes);
	kfree(data);
	kfree(order);
	kfree(mRecord);
	return err;
}

static void create_memtrack_buffer_msg(struct nxpTfa98xx_Memtrack_data *mRecord, char *buffer, int *size)
{
	int i, j = 0;

	/* First copy the msg_id to the buffer */
	buffer[0] = 0;
	buffer[1] = MODULE_FRAMEWORK + 128;
	buffer[2] = FW_PAR_ID_SET_MEMTRACK;

	/* Then copy the length (number of memtrack items) */
	buffer[3] = (uint8_t) ((mRecord->length >> 16) & 0xffff);
	buffer[4] = (uint8_t) ((mRecord->length >> 8) & 0xff);
	buffer[5] = (uint8_t) (mRecord->length & 0xff);

	/* For every item take the tracker and add this infront of the adress */
	for(i=6; i<6+(mRecord->length*3); i+=3) {
		buffer[i+0] = (uint8_t) ((mRecord->trackers[j] & 0xff));
		buffer[i+1] = (uint8_t) ((mRecord->mAdresses[j] >> 8) & 0xff);
		buffer[i+2] = (uint8_t) (mRecord->mAdresses[j] & 0xff);
		j++;
	}

	*size = (6+(mRecord->length*3)) * sizeof(char);
}

enum Tfa98xx_Error tfa_append_substring(char *substring, char *str_buff, int maxlength)
{
	int substring_len = (int)(strlen(substring));
	int str_buff_len = (int)(strlen(str_buff));

	if((substring_len + str_buff_len) >= maxlength)
		return Tfa98xx_Error_Buffer_too_small;

	strcat(str_buff, substring);

	return Tfa98xx_Error_Ok;
}

Tfa98xx_Error_t tfa98xx_set_live_data(struct tfa98xx *tfa98xx,int dev_idx)
{
	Tfa98xx_Error_t err = Tfa98xx_Error_Ok;
	nxpTfaLiveDataList_t *lData;
	nxpTfaLiveData_t *liveD;
	struct nxpTfa98xx_Memtrack_data *mRecord = kmalloc(sizeof(struct nxpTfa98xx_Memtrack_data), GFP_KERNEL);
	char *buffer = kmalloc(sizeof(char) * ((MEMTRACK_MAX_WORDS * 3) + 6),GFP_KERNEL); //every word requires 3 bytes, and 6 is the msg + length
	int size=0;
	unsigned int j, k=0, skip_set_msg=0;
	memset(mRecord, 0, sizeof(*mRecord));

	if (tfa98xx->tfa->tfa_family == 2) {
		/* Get the memtrack information from the container file */
		if(tfa98xx->tfa->cnt->nliveData > 0) {
			lData=tfaContGetDevLiveDataList(tfa98xx->tfa->cnt, dev_idx, 0);
			if(lData == NULL) {
				kfree(buffer);
				kfree(mRecord);
				return err;
			} else if(lData->length-1 > MEMTRACK_MAX_WORDS) {
				pr_err("Error: There are too many memtrack registers\n");
				kfree(buffer);
				kfree(mRecord);
				return Tfa98xx_Error_Bad_Parameter;
			} else {
				mRecord->length = 0; /* initialise */
			}

			/* The number of memtrack sections in the container file */
			for(j=0; j<lData->length-1u; j++) {
				liveD = tfaGetLiveDataItem(&lData->list[j], tfa98xx->tfa->cnt);

				/* Only use DSP memtrack items. Skip the I2C registers/bitfields */
				/* We need the use k else we get empty lines in the mRecord */
				/* if not in any list */
				if (liveD->addrs[0] >= '0' && liveD->addrs[0] <= '9') {
					if (compare_strings(liveD->name, "buildinmemtrack", 25)) {
						pr_err("Found keyword: %s. Skipping the SetMemtrack! \n", liveD->name);
						skip_set_msg = 1;
					} else {
						mRecord->mAdresses[k] = (int)simple_strtol(liveD->addrs, NULL, 16);
						mRecord->trackers[k] = liveD->tracker;
						mRecord->length++;
						k++;
					}
				}
			}

			if(!skip_set_msg) {
				if(mRecord->length > 0) {
					create_memtrack_buffer_msg(mRecord, buffer, &size);
					err = dsp_msg(tfa98xx->tfa, size, buffer);
				}
			}
		}
	}

	kfree(buffer);
	kfree(mRecord);
	return err;
}

Tfa98xx_Error_t tfa98xx_get_live_data_items(struct tfa98xx *tfa98xx,int dev_idx, char *strings)
{
	Tfa98xx_Error_t error = Tfa98xx_Error_Ok;
	nxpTfaLiveDataList_t *lData;
	nxpTfaLiveData_t *liveD;
	int  maxlength = sizeof(liveD->name) * MEMTRACK_MAX_WORDS;
	char *str = NULL;
	unsigned int  j;

	str = kmalloc(256, GFP_KERNEL);
	if (!str) {
		error = Tfa98xx_Error_Fail;
		pr_err("[0x%x] memory allocation failed\n", tfa98xx->i2c->addr);
		goto error_exit;
	}

	/* Only Max1 devices */
	if (tfa98xx->tfa->tfa_family == 1) {
		pr_err("%s:tfa_family is one, stop work\n", __func__);
	}

	/* This is printed for every device */
	if(tfa98xx->tfa->cnt->nliveData > 0) {
		lData=tfaContGetDevLiveDataList(tfa98xx->tfa->cnt, dev_idx, 0);
		if(lData != NULL) {
			// Get all livedata items
			for(j=0; j<lData->length-1u; j++) {
				liveD = tfaGetLiveDataItem(&lData->list[j], tfa98xx->tfa->cnt);
				if (tfa98xx->tfa->tfa_family == 2) {
					if (compare_strings(liveD->name, "buildinmemtrack", 25) == 0) {
						sprintf(str, "%s,", liveD->name);
						error = tfa_append_substring(str, strings, maxlength);
					}
				} else  {
					sprintf(str, "%s(0x%x),", liveD->name, tfa98xx->tfa->slave_address);
					error = tfa_append_substring(str, strings, maxlength);
				}
				if(error != Tfa98xx_Error_Ok)
					goto error_exit;
			}
		}
	}

error_exit:
	kfree(str);
	return error;
}

int exTfa98xx_getf0_req( struct tfa98xx *tfa98xx)
{
	enum Tfa98xx_Error err = Tfa98xx_Error_Ok;
	int i = 0;
	//int profile = -1;
	int devcnt = 0;
	int spkr_count = 0;
	int error_found = 0;
	int nr_of_items = 0, length = 0, fResNo = 0;
	int live_data[MEMTRACK_MAX_WORDS] = {0};
	char *buffer = kmalloc((20 * MEMTRACK_MAX_WORDS) + 1, GFP_KERNEL); // Assuming each string name is 20 char (currently the biggest is 17, smallest is 3)
	buffer[0] = '\0';	// Clear the buffer before the first use!

	pr_err("Starting application.\n");

	//devs = tfa_get_device_struct();
	if(tfa98xx->tfa != NULL &&  tfa98xx->tfa->cnt != NULL) {
		devcnt = tfa98xx->tfa->cnt->ndev;

	}

	pr_err("%s:devcnt number %d\n", __func__, devcnt);
	devcnt =1;      //test

	if (0 == devcnt) {
		pr_err("devcnt error\n");
		err = Tfa98xx_Error_Device;
		error_found += (int)err;
		goto error_exit;
	}

	if(err == Tfa98xx_Error_Fail)
		goto error_exit;

	for (i=0; i < devcnt; i++ ) {
		err = tfa98xx_get_live_data_items(tfa98xx,i, buffer + length);
		if (err != Tfa98xx_Error_Ok) {
			pr_err("ERROR:0x%x:%s:%u",err, __func__, __LINE__);
			error_found+=(int)err;
			if (err == Tfa98xx_Error_Buffer_too_small)
			pr_err("Error: buffer size %d is too short! \n", (int)(strlen(buffer)));

			continue;
		}
		length = (int)(strlen(buffer));
	}

	if (length == 0) {
		pr_err("ERROR:%s:%u",__func__, __LINE__);
		err = Tfa98xx_Error_Fail;
		error_found+=(int)err;
		goto error_exit;
	}

	*(buffer+length) = '\0';

	for(i=0; i < length - 4; i++) {
		if(strncmp(buffer+i, "Fres", 4)==0) break;
		if(strncmp(buffer+i, ",", 1)==0) fResNo++;
	}

	if(0 == fResNo) {
		pr_err("Error: No f0 item\n");
		err = Tfa98xx_Error_Fail;
		goto error_exit;
	}

	for (i=0; i < devcnt; i++ ) {
		/* Send the memtrack items. Only for max2 */
		err = tfa98xx_set_live_data(tfa98xx,i);
		if (err != Tfa98xx_Error_Ok) {
			error_found+=(int)err;
			pr_err("last status: %d (%s)\n", err, Tfa98xx_GetErrorString(err));
			pr_err("ERROR 0x%x:%s:%u",err, __func__, __LINE__);
			continue;
		}
	}

	for (i=0; i < devcnt; i++ ) {
		err = tfa98xx_get_live_data(tfa98xx,i, live_data, &nr_of_items);
		if (err != Tfa98xx_Error_Ok) {
			error_found+=(int)err;
			pr_err("ERROR 0x%x:%s:%u",err, __func__, __LINE__);
			break;
		}

		if(tfa98xx_device_count == 1) {
			if (spkr_count == 1) {
				gf0[0] = live_data[fResNo];
				pr_err("Current f0 value: %dHz, %d\n", live_data[fResNo],fResNo);
			}
			else if(tfa98xx_device_count == 2){
				gf0[0] = live_data[fResNo];
				gf0[1] = live_data[fResNo+1];
				pr_err("speaker number is two, Current 0x34 f0 value: %dHz,Current 0x35 f0 value: %dHz,Current fResNo:%d \n",gf0[0],gf0[1],fResNo);
			}
			else{
				pr_err("spk wrong number\n");
			}
		
		}
		
		if(tfa98xx_device_count == 1) {
			if (spkr_count == 1) {
				gtemp[0] = live_data[4]*1000;
				pr_err("Current temp value: %d C\n", gtemp[0]);
			}
			else if(tfa98xx_device_count == 2){
				gtemp[0] = live_data[4]*1000;
				gtemp[1] = live_data[5]*1000;
				pr_err("speaker number is two, Current 0x34 temp value: %d C,Current 0x35 temp value: %d C\n",gtemp[0],gtemp[1]);
			}
			else{
				pr_err("spk wrong number\n");
			}
		
		}

error_exit:
	if (error_found!=Tfa98xx_Error_Ok) {
		pr_err("an error occured, code:%d\n",error_found);
	}
	kfree(buffer);
	return error_found;
}
}
#endif

#ifdef TFA98xx_memtrack_nondsp
static int8_t bytes[21] = {0};

const uint8_t memtrack_nondsp_send[] = {
		0x00, 0x80, 0x0b, 0x00, 0x00, 0x06,
		0x22, 0x00, 0x00, 0x22, 0x00, 0x01,
		0x22, 0x00, 0x04, 0x22, 0x00, 0x05,
		0x22, 0x00, 0x13, 0x22, 0x00, 0x14
};
static Tfa98xx_Error tfa98xx_rpc_send_nondsp(struct tfa98xx *tfa98xx)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	int i=0;

	
	const uint8_t memtrack_nondsp_read[] = {0x00,0x80,0x8b}
	
	if (!tfa98xx->tfa->is_probus_device){
		pr_err("not tfa nondsp code, could not send para\n");
		return Tfa98xx_Error_Not_Supported;
	
	}
	
	
	error = tfa98xx_send_data_to_dsp(memtrack_nondsp_send, sizeof(memtrack_nondsp_send));
	if (error != Tfa98xx_Error_Ok){
		pr_err("tfa98xx_rpc_send_nondsp tfa98xx_send_data_to_dsp error:%d\n",error);
		return Tfa98xx_Error_Fail;
	}
		//error = tfa98xx_write_dsp(tfa98xx->tfa, sizeof(memtrack_nondsp), (const char *)memtrack_nondsp);
	mdelay(50000);
	error = tfa98xx_send_data_to_dsp(memtrack_nondsp_read, sizeof(memtrack_nondsp_read));
	if (error != Tfa98xx_Error_Ok){
		pr_err("tfa98xx_rpc_send_nondsp tfa98xx_send_data_to_dsp error:%d\n",error);
		return Tfa98xx_Error_Fail;
	}
	error = tfa98xx_recv_data_from_dsp(bytes, (size_t)sizeof(bytes));
	if (error != Tfa98xx_Error_Ok){
		pr_err("tfa98xx_rpc_send_nondsp tfa98xx_recv_data_from_dsp error:%d\n",error);
		return Tfa98xx_Error_Fail;
	}

	
	for(i=0; i<sizeof(bytes); i++)
		pr_err("tfa98xx_rpc_send_nondsp,bytes[%d]: 0x%x\n", i, bytes[i]);
	
	return Tfa98xx_Error_Ok;
}

static ssize_t tfa_store_memtrack_nondsp(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &value) == 1) {
		if(value == 1){
			error = tfa98xx_rpc_send_nondsp(tfa98xx);
			if (error != Tfa98xx_Error_Ok){
				pr_err("tfa_store_memtrack_nondsp error:%d\n",error);
				return -EINVAL;
			}
			pr_err("memtrack finish\n");
			return count;
		}else {
			return -EINVAL;
		}
	}else {
		return -EINVAL;
	}
}

static ssize_t tfa_show_memtrack_nondsp(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
//	return sprintf(buf, "%d,%d\n",gf0[0],gf0[1]);
	return 0;
}

static DEVICE_ATTR(memtrack_nondsp, S_IRUGO|S_IWUSR,
	tfa_show_memtrack_nondsp, tfa_store_memtrack_nondsp);
#endif

#ifdef TFA98xx_F0_tracking
static ssize_t tfa_store_cal_f0(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &value) == 1) {
		if(value == 1){
			exTfa98xx_getf0_req(tfa98xx);
			pr_err("Calibration f0\n");
			return count;
		}else {
			return -EINVAL;
		}
	}else {
		return -EINVAL;
	}
}

static ssize_t tfa_show_cal_f0(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d,%d\n",gf0[0],gf0[1]);
}

static DEVICE_ATTR(cal_f0, S_IRUGO|S_IWUSR,
	tfa_show_cal_f0, tfa_store_cal_f0);
#endif

static int tfa98xx_mute(struct snd_soc_dai *dai, int mute, int stream)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	struct snd_soc_component *codec = dai->component;
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(codec);
#else
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	dev_dbg(&tfa98xx->i2c->dev, "%s: state: %d\n", __func__, mute);

	if (no_start) {
		pr_debug("no_start parameter set no tfa_dev_start or tfa_dev_stop, returning\n");
		return 0;
	}
#ifdef TFA98xx_dsp_bypass	
	atomic_set(&g_bypass, 0);
	atomic_set(&g_Tx_enable, 1);
#endif

	if (mute) {
		/* stop DSP only when both playback and capture streams
		 * are deactivated
		 */
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			tfa98xx->pstream = 0;
		else
			tfa98xx->cstream = 0;
//		if (tfa98xx->pstream != 0 || tfa98xx->cstream != 0)
//			return 0;

		mutex_lock(&tfa98xx_mutex);
		tfa98xx_sync_count = 0;
		mutex_unlock(&tfa98xx_mutex);

		cancel_delayed_work_sync(&tfa98xx->monitor_work);

		cancel_delayed_work_sync(&tfa98xx->init_work);
		if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK ||
		    tfa98xx->dsp_init == TFA98XX_DSP_INIT_STOPPED)
			return 0;
		mutex_lock(&tfa98xx->dsp_lock);
		tfa_dev_stop(tfa98xx->tfa);
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_STOPPED;
		mutex_unlock(&tfa98xx->dsp_lock);
		if(tfa98xx->flags & TFA98XX_FLAG_ADAPT_NOISE_MODE)
			cancel_delayed_work_sync(&tfa98xx->nmodeupdate_work);
	}
	else {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
#ifdef TFA9894_STEREO
		{
			tfa98xx->pstream = 1;


                	/* Start DSP */
                	if ((tfa98xx->flags & TFA98XX_FLAG_CHIP_SELECTED) &&
			   			(tfa98xx->dsp_init != TFA98XX_DSP_INIT_PENDING))
                        	tfa98xx_dsp_init(tfa98xx);
		}else{
			tfa98xx->cstream = 1;
            return 0;
        }
#else
			tfa98xx->pstream = 1;
		else{
			tfa98xx->cstream = 1;
			return 0;
		}

		/* Start DSP */
#if 0
		if (tfa98xx->dsp_init != TFA98XX_DSP_INIT_PENDING)
			queue_delayed_work(tfa98xx->tfa98xx_wq,
				&tfa98xx->init_work, 0);
#else
		 tfa98xx_dsp_init(tfa98xx);
#endif//

#endif	
if(tfa98xx->flags & TFA98XX_FLAG_ADAPT_NOISE_MODE)
			queue_delayed_work(tfa98xx->tfa98xx_wq,
						&tfa98xx->nmodeupdate_work,
						0);	
	}

	return 0;
}

static const struct snd_soc_dai_ops tfa98xx_dai_ops = {
	.startup = tfa98xx_startup,
	.set_fmt = tfa98xx_set_fmt,
	.set_sysclk = tfa98xx_set_dai_sysclk,
	.set_tdm_slot = tfa98xx_set_tdm_slot,
	.hw_params = tfa98xx_hw_params,
	.mute_stream = tfa98xx_mute,
};

static struct snd_soc_dai_driver tfa98xx_dai[] = {
	{
		.name = "tfa98xx-aif",
		.id = 1,
		.playback = {
			.stream_name = "AIF Playback",
			.channels_min = 1,
			.channels_max = 4,
			.rates = TFA98XX_RATES,
			.formats = TFA98XX_FORMATS,
		},
		.capture = {
			 .stream_name = "AIF Capture",
			 .channels_min = 1,
			 .channels_max = 4,
			 .rates = TFA98XX_RATES,
			 .formats = TFA98XX_FORMATS,
		 },
		.ops = &tfa98xx_dai_ops,
#ifdef CONFIG_MTK_PLATFORM
		.symmetric_rates = 1,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
		.symmetric_channels = 1,
		.symmetric_samplebits = 1,
#endif
#endif
	},
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
static int tfa98xx_probe(struct snd_soc_component *codec)
{
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(codec);
#else
static int tfa98xx_probe(struct snd_soc_codec *codec)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif	
	int ret;

	pr_debug("\n");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	snd_soc_component_init_regmap(codec, tfa98xx->regmap);
#endif
	/* setup work queue, will be used to initial DSP on first boot up */
	tfa98xx->tfa98xx_wq = create_singlethread_workqueue("tfa98xx");
	if (!tfa98xx->tfa98xx_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&tfa98xx->init_work, tfa98xx_dsp_init_work);
	INIT_DELAYED_WORK(&tfa98xx->monitor_work, tfa98xx_monitor);
	INIT_DELAYED_WORK(&tfa98xx->interrupt_work, tfa98xx_interrupt);
	INIT_DELAYED_WORK(&tfa98xx->tapdet_work, tfa98xx_tapdet_work);
	INIT_DELAYED_WORK(&tfa98xx->nmodeupdate_work, tfa98xx_nmode_update_work);
#ifdef DISABLE_TFA98XX_ALSA_SUPPORT     
	INIT_DELAYED_WORK(&tfa98xx->unmute_work, tfa98xx_unmue_target_device);
#endif
	tfa98xx->codec = codec;

	ret = tfa98xx_load_container(tfa98xx);
	pr_debug("Container loading requested: %d\n", ret);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
	codec->control_data = tfa98xx->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 8, 16, SND_SOC_REGMAP);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
#endif
	tfa98xx_add_widgets(tfa98xx);

	dev_info(codec->dev, "tfa98xx codec registered (%s)",
		tfa98xx->fw.name);

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
static void tfa98xx_remove(struct snd_soc_component *codec)
{
	struct tfa98xx *tfa98xx = snd_soc_component_get_drvdata(codec);
#else
static int tfa98xx_remove(struct snd_soc_codec *codec)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
#endif
	pr_debug("\n");

	tfa98xx_interrupt_enable(tfa98xx, false);

	tfa98xx_inputdev_unregister(tfa98xx);

	cancel_delayed_work_sync(&tfa98xx->interrupt_work);
	cancel_delayed_work_sync(&tfa98xx->monitor_work);
	cancel_delayed_work_sync(&tfa98xx->init_work);
	cancel_delayed_work_sync(&tfa98xx->tapdet_work);
	cancel_delayed_work_sync(&tfa98xx->nmodeupdate_work);
#ifdef DISABLE_TFA98XX_ALSA_SUPPORT     
	cancel_delayed_work_sync(&tfa98xx->unmute_work);
#endif
	if (tfa98xx->tfa98xx_wq)
		destroy_workqueue(tfa98xx->tfa98xx_wq);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	return;
#else
	return 0;
#endif
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(4,18,0))
static struct regmap *tfa98xx_get_regmap(struct device *dev)
{
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);

	return tfa98xx->regmap;
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
static struct snd_soc_component_driver soc_codec_dev_tfa98xx = {
#else
static struct snd_soc_codec_driver soc_codec_dev_tfa98xx = {
#endif
	.probe =	tfa98xx_probe,
	.remove =	tfa98xx_remove,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(4,18,0))
	.get_regmap = tfa98xx_get_regmap,
#endif
};


static bool tfa98xx_writeable_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return 1;
}

static bool tfa98xx_readable_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return 1;
}

static bool tfa98xx_volatile_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return 1;
}

static const struct regmap_config tfa98xx_regmap = {
	.reg_bits = 8,
	.val_bits = 16,

	.max_register = TFA98XX_MAX_REGISTER,
	.writeable_reg = tfa98xx_writeable_register,
	.readable_reg = tfa98xx_readable_register,
	.volatile_reg = tfa98xx_volatile_register,
	.cache_type = REGCACHE_NONE,
};

static void tfa98xx_irq_tfa2(struct tfa98xx *tfa98xx)
{
	pr_info("\n");

	/*
	 * mask interrupts
	 * will be unmasked after handling interrupts in workqueue
	 */
	tfa_irq_mask(tfa98xx->tfa);
	queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->interrupt_work, 0);
}


static irqreturn_t tfa98xx_irq(int irq, void *data)
{
	struct tfa98xx *tfa98xx = data;

	if (tfa98xx->tfa->tfa_family == 2)
		tfa98xx_irq_tfa2(tfa98xx);

	return IRQ_HANDLED;
}

static int tfa98xx_ext_reset(struct tfa98xx *tfa98xx)
{
	if (tfa98xx && gpio_is_valid(tfa98xx->reset_gpio)) {
		int reset = tfa98xx->reset_polarity;
		gpio_set_value_cansleep(tfa98xx->reset_gpio, reset);
		mdelay(10);
		gpio_set_value_cansleep(tfa98xx->reset_gpio, !reset);
		mdelay(10);
	}
	return 0;
}

static int tfa98xx_parse_dt(struct device *dev, struct tfa98xx *tfa98xx,
	struct device_node *np) {
	u32 value;
	int ret;
	tfa98xx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (tfa98xx->reset_gpio < 0)
		dev_dbg(dev, "No reset GPIO provided, will not HW reset device\n");

	tfa98xx->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (tfa98xx->irq_gpio < 0)
		dev_dbg(dev, "No IRQ GPIO provided.\n");
	ret = of_property_read_u32(np,"reset-polarity",&value);
	if(ret< 0)
	{
		tfa98xx->reset_polarity = HIGH;
	}else {
		tfa98xx->reset_polarity = (value == 0) ? LOW : HIGH;
	} 

	tfa98xx->is_primary = of_property_read_bool(np, "is-primary");
	
	dev_dbg(dev, "reset-polarity:%d\n",tfa98xx->reset_polarity);
	return 0;
}

static ssize_t tfa98xx_reg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);

	if (count != 1) {
		pr_debug("invalid register address");
		return -EINVAL;
	}

	tfa98xx->reg = buf[0];

	return 1;
}

static ssize_t tfa98xx_rw_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);
	u8 *data = NULL;
	int ret;
	int retries = I2C_RETRIES;

	if (count > PAGE_SIZE) {
		pr_debug("size is too big\n");
		return	-ENOMEM;		
	}

	data = kmalloc(count + 1, GFP_KERNEL);
	if (data == NULL) {
		pr_debug("can not allocate memory\n");
		return  -ENOMEM;
	}

	data[0] = tfa98xx->reg;
	memcpy(&data[1], buf, count);

retry:
	ret = i2c_master_send(tfa98xx->i2c, data, count + 1);
	if (ret < 0) {
		pr_warn("i2c error, retries left: %d\n", retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
	}

	kfree(data);

	/* the number of data bytes written without the register address */
	return ((ret > 1) ? count : -EIO);
}

static ssize_t tfa98xx_rw_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);
	struct i2c_msg msgs[] = {
		{
			.addr = tfa98xx->i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &tfa98xx->reg,
		},
		{
			.addr = tfa98xx->i2c->addr,
			.flags = I2C_M_RD,
			.len = count,
			.buf = buf,
		},
	};
	int ret;
	int retries = I2C_RETRIES;
retry:
	ret = i2c_transfer(tfa98xx->i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		pr_warn("i2c error, retries left: %d\n", retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
		return ret;
	}
	/* ret contains the number of i2c transaction */
	/* return the number of bytes read */
	return ((ret > 1) ? count : -EIO);
}

static struct bin_attribute dev_attr_rw = {
	.attr = {
		.name = "rw",
		.mode = S_IRUSR | S_IWUSR,
	},
	.size = 0,
	.read = tfa98xx_rw_read,
	.write = tfa98xx_rw_write,
};

static struct bin_attribute dev_attr_reg = {
	.attr = {
		.name = "reg",
		.mode = S_IWUSR,
	},
	.size = 0,
	.read = NULL,
	.write = tfa98xx_reg_write,
};

#ifdef TFA98xx_calibrate
static enum Tfa98xx_Error tfa98xxCalibration(struct tfa98xx **tfa98xx, int *speakerImpedance) 
{
	enum Tfa98xx_Error err;
	int imp, profile, cal_profile, i;
	char buffer[6] = {0};
	unsigned char bytes[6] = {0};

	cal_profile = tfaContGetCalProfile(tfa98xx[0]->tfa);
	if (cal_profile >= 0)
		profile = cal_profile;
	else
		profile = 0;
	/*
	for (i = 0; i < tfa98xx_device_count; i++){
	  if (tfa_dev_mtp_get(tfa98xx[i]->tfa, TFA_MTP_EX)){
		pr_info("DSP already calibrated, MTPEX==1\n");
		err = tfa_dsp_get_calibration_impedance(tfa98xx[i]->tfa);
		imp = tfa_get_calibration_info(tfa98xx->tfa, 0);
		speakerImpedance[i] = imp;
		pr_info("Calibration value: %d.%3d ohm\n", imp%1000,imp-1000*(imp%1000));

	  }
	}
	*/

	pr_info("Calibration started profile %d,dev_count: %d\n", profile, tfa98xx_device_count);
/* Clear MTPOTC to make sure calibration values are always read from DSP */
	for (i = 0; i < tfa98xx_device_count; i++){
		err = (enum Tfa98xx_Error)tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_OTC, 0);
		if (err) {
			pr_err("MTPOTC write failed\n");
			return Tfa98xx_Error_Fail;
		}

		err = (enum Tfa98xx_Error)tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_EX, 0);
		if (err) {
			pr_err("MTPEX write failed\n");
			return Tfa98xx_Error_Fail;
		}

		/* write all the registers from the profile list */
/*
		err = tfaContWriteRegsProf(tfa98xx[i]->tfa, profile);
		if (err){
			pr_err("MTP write failed\n");
			return Tfa98xx_Error_Fail;
		}
*/
	}

	/* set the DSP commands SetAlgoParams and SetMBDrc reset flag */
	tfa98xx[0]->tfa->needs_reset = 1;
	
	for (i = 0; i < tfa98xx_device_count; i++){
		err = tfaContWriteProfile(tfa98xx[i]->tfa, profile, 0);
		if (err){
			pr_err("MTP write failed\n");
			return Tfa98xx_Error_Fail;
		}
	}
	
	for (i = 0; i < tfa98xx_device_count; i++)
	{
		tfa_dev_set_swprof(tfa98xx[i]->tfa, (unsigned short)profile);
	}

	/* write all the files from the profile list (volumestep 0) */
/*
	err = tfaContWriteFilesProf(tfa98xx[0]->tfa, profile, 0);
	if (err) {
		pr_err("profile write failed\n");
		return Tfa98xx_Error_Bad_Parameter;
	}
*/

	/* clear the DSP commands SetAlgoParams and SetMBDrc reset flag */
	tfa98xx[0]->tfa->needs_reset = 0;

	/* First read the GetStatusChange to clear the status (sticky) */
	err = tfa_dsp_cmd_id_write_read(tfa98xx[0]->tfa, MODULE_FRAMEWORK, FW_PAR_ID_GET_STATUS_CHANGE, 6, (unsigned char *)buffer);

	/* We need to send zero's to trigger calibration */
	memset(bytes, 0, 6);
	err = tfa_dsp_cmd_id_write(tfa98xx[0]->tfa, MODULE_SPEAKERBOOST, SB_PARAM_SET_RE25C, sizeof(bytes), bytes);

	/* Wait a maximum of 5 seconds to get the calibration results */
	for (i=0; i < 50; i++ ) {
		/* Avoid busload */
		msleep_interruptible(100);
		
		/* Get the GetStatusChange results */
		err = tfa_dsp_cmd_id_write_read(tfa98xx[0]->tfa, MODULE_FRAMEWORK, FW_PAR_ID_GET_STATUS_CHANGE, 6, (unsigned char *)buffer);

		/* If the calibration trigger is set break the loop */
		if(buffer[5] & TFADSP_FLAG_CALIBRATE_DONE) {
			break;
		}

		/* bit1 is TFADSP_FLAG_DAMAGED_SPEAKER_P
		   bit2 is TFADSP_FLAG_DAMAGED_SPEAKER_S
		*/
		if (buffer[2] & 0x6) {
			if (buffer[2] & 0x2)
				pr_err("%s: Left SPK damaged event detected 0x%x\n", __func__, buffer[2]);
			if (buffer[2] & 0x4)
				pr_err("%s: Right SPK damaged event detected 0x%x\n", __func__, buffer[2]);
			break;
		}
	}


	err = tfa_dsp_get_calibration_impedance(tfa98xx[0]->tfa);
	for (i = 0; i < tfa98xx_device_count; i++){

		imp = (tfa_get_calibration_info(tfa98xx[0]->tfa, i));
		speakerImpedance[i] = imp;

		pr_info("%s: Calibration value: %d.%3d ohm\n", __func__, imp/1000, imp%1000);
		
		/* Set MTPEX to indicate calibration is done! */
		err = (enum Tfa98xx_Error)tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_OTC, 1);
			if (err) {
				pr_err("OTC write failed\n");
				return Tfa98xx_Error_Fail;
			}

		/* Write calibration value to MTP */
		err = (enum Tfa98xx_Error)tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_RE25, (uint16_t)((int)(imp)));
		if (err) {
			pr_err("MTP Re25 write failed\n");
			return Tfa98xx_Error_Fail;
		}
#if 0
		int value;
		value = tfa_dev_mtp_get(tfa, TFA_MTP_RE25);
		printk("imp:%d,value:%d\n",imp,value);
#endif		

	/* Set MTPEX to indicate calibration is done! */
		err = (enum Tfa98xx_Error)tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_EX, 2);
		if (err) {
			pr_err("MTPEX write failed\n");
			return Tfa98xx_Error_Fail;
		}

		/* Save the current profile */
		tfa_dev_set_swprof(tfa98xx[i]->tfa, (unsigned short)profile);
	}

	return Tfa98xx_Error_Ok;
}
#endif

#ifdef ENABLE_IOCTRL_INTERFACE

static char spk_box[64] = { 0 };
static int algo_id = 0;

static int tfa98xx_ioctrl_probe(struct tfa98xx *tfa98xx)
{
	int ret;

	(void)tfa98xx_dai;
	(void)soc_codec_dev_tfa98xx;
	(void)tfa98xx_create_controls;

	pr_debug("\n");

	/* setup work queue, will be used to initial DSP on first boot up */
	tfa98xx->tfa98xx_wq = create_singlethread_workqueue("tfa98xx");
	if (!tfa98xx->tfa98xx_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&tfa98xx->init_work, tfa98xx_dsp_init_work);
	INIT_DELAYED_WORK(&tfa98xx->monitor_work, tfa98xx_monitor);
	INIT_DELAYED_WORK(&tfa98xx->interrupt_work, tfa98xx_interrupt);
	INIT_DELAYED_WORK(&tfa98xx->tapdet_work, tfa98xx_tapdet_work);
#ifdef DISABLE_TFA98XX_ALSA_SUPPORT    
	INIT_DELAYED_WORK(&tfa98xx->unmute_work, tfa98xx_unmue_target_device);
#endif
	ret = tfa98xx_load_container(tfa98xx);
	pr_debug("Container loading requested: %d\n", ret);

	dev_info(tfa98xx->dev, "tfa98xx codec registered (%s)",
							tfa98xx->fw.name);

	return ret;
}

static int tfa98xx_ioctrl_set_profile(struct tfa98xx *tfa98xx,
				   int new_profile)
{
	int change = 0;
	int prof_idx = new_profile;
	int profile_count = (tfa98xx_container)? tfa98xx_container->nprof : 0;

	if (no_start != 0)
		return 0;

	if (new_profile == tfa98xx_mixer_profile)
		return 0;

	if ((new_profile < 0) || (new_profile >= profile_count)) {
		pr_err("not existing profile (%d)\n", new_profile);
		return -EINVAL;
	}

	pr_debug("selected container profile [%d]\n", prof_idx);

	/* update mixer profile */
	tfa98xx_mixer_profile = new_profile;

	mutex_lock(&tfa98xx_mutex);
	list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
		int err;
		int ready = 0;

		/* update 'real' profile (container profile) */
		tfa98xx->profile = prof_idx;
		tfa98xx->vstep = tfa98xx->prof_vsteps[prof_idx];

		/* Don't call tfa_dev_start() if there is no clock. */
		mutex_lock(&tfa98xx->dsp_lock);
		/*tfa98xx_dsp_system_stable(tfa98xx->tfa, &ready);*/
		if (ready) {
			/* Also re-enables the interrupts */
			err = tfa98xx_tfa_start(tfa98xx, prof_idx, tfa98xx->vstep);
			if (err) {
				pr_info("Write profile error: %d\n", err);
			} else {
				pr_debug("Changed to profile %d (vstep = %d)\n",
						 prof_idx, tfa98xx->vstep);
				change = 1;
			}
		}
		mutex_unlock(&tfa98xx->dsp_lock);

		/* Flag DSP as invalidated as the profile change may invalidate the
		 * current DSP configuration. That way, further stream start can
		 * trigger a tfa_dev_start.
		 */
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_INVALIDATED;
	}

	if (change) {
		list_for_each_entry(tfa98xx, &tfa98xx_device_list, list) {
			mutex_lock(&tfa98xx->dsp_lock);
			tfa_dev_set_state(tfa98xx->tfa, TFA_STATE_UNMUTE,0);
			mutex_unlock(&tfa98xx->dsp_lock);
		}
	}

	mutex_unlock(&tfa98xx_mutex);

	return change;
}

static int tfa98xx_ioctrl_mute(struct tfa98xx *tfa98xx, int mute)
{
	dev_info(&tfa98xx->i2c->dev, "%s: state: %d\n", __func__, mute);

	if (no_start) {
		pr_debug("no_start parameter set no tfa_dev_start or tfa_dev_stop, returning\n");
		return 0;
	}

	if (mute) {
		mutex_lock(&tfa98xx_mutex);
		tfa98xx_sync_count = 0;
		mutex_unlock(&tfa98xx_mutex);

		cancel_delayed_work_sync(&tfa98xx->monitor_work);

		cancel_delayed_work_sync(&tfa98xx->init_work);
		if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK)
			return 0;
		mutex_lock(&tfa98xx->dsp_lock);
		tfa_dev_stop(tfa98xx->tfa);
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_STOPPED;
		mutex_unlock(&tfa98xx->dsp_lock);
		if(tfa98xx->flags & TFA98XX_FLAG_ADAPT_NOISE_MODE)
			cancel_delayed_work_sync(&tfa98xx->nmodeupdate_work);
	} else {
		/* Start DSP */
#if 0		
		if (tfa98xx->dsp_init != TFA98XX_DSP_INIT_PENDING)
			queue_delayed_work(tfa98xx->tfa98xx_wq,
							   &tfa98xx->init_work, 0);
#else
		 tfa98xx_dsp_init(tfa98xx);
#endif//	
		if(tfa98xx->flags & TFA98XX_FLAG_ADAPT_NOISE_MODE)
		   queue_delayed_work(tfa98xx->tfa98xx_wq,
					   &tfa98xx->nmodeupdate_work,
					   0);
	}

	return 0;
}

static int tfa98xx_set_default_calib(struct tfa98xx *tfa98xx, int rdc)
{
	enum tfa_error err = tfa_error_ok;

	/*default value should within 4ohm ~ 9ohm*/
	if (rdc > 9216 || rdc < 4096)
		return tfa_error_bad_param;
	
	mutex_lock(&tfa98xx->dsp_lock);
	err |= tfa_dev_mtp_set(tfa98xx->tfa, TFA_MTP_OTC, 1);
	err |= tfa98xx_write_re25(tfa98xx->tfa, rdc);
	mutex_unlock(&tfa98xx->dsp_lock);
	
	return err;
}

#define TFADSP_FLAG_CALIBRATE_DONE	1

static int tfa98xx_do_calibrate(struct tfa98xx *tfa98xx_pri, struct tfa98xx *tfa98xx_sec)
{
	int profile, cal_profile, devices;
	int err, i, spk_imp = 0;
	unsigned char bytes[6] = {0};
	struct tfa98xx *tfa98xx[2] = {tfa98xx_pri, tfa98xx_sec};

	devices = (tfa98xx_device_count > 2) ? 2 : tfa98xx_device_count;
	if (devices == 2 && 
		tfa98xx_sec == NULL) {
		pr_err("device count match up failed\n");
		
		return Tfa98xx_Error_Fail;
	}
	
	cal_profile = tfaContGetCalProfile(tfa98xx[0]->tfa);
	if (cal_profile >= 0)
		profile = cal_profile;
	else
		profile = 0;

	pr_info("tfa98xx_do_calibrate started profile %d\n", profile);
	
	for (i = 0; i < devices; i++){
		err = tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_OTC, 0);
		if (err) {
			dev_err(tfa98xx[i]->dev, "MTPOTC write failed\n");
			return err;
		}
		/* write all the registers from the profile list */
		err = tfaContWriteRegsProf(tfa98xx[i]->tfa, profile);
		if (err){
			dev_err(tfa98xx[i]->dev, "MTP write failed\n");
			return err;
		}
	}

	/* set the DSP commands SetAlgoParams and SetMBDrc reset flag */
	tfa98xx[0]->tfa->needs_reset = 1;

	/* write all the files from the profile list (volumestep 0) */
	err = tfaContWriteFilesProf(tfa98xx[0]->tfa, profile, 0);
	if (err) {
		pr_err("profile write failed\n");
		return err;
	}

	/* clear the DSP commands SetAlgoParams and SetMBDrc reset flag */
	tfa98xx[0]->tfa->needs_reset = 0;

	/* First read the GetStatusChange to clear the status (sticky) */
	err = tfa_dsp_cmd_id_write_read(tfa98xx[0]->tfa, MODULE_FRAMEWORK, FW_PAR_ID_GET_STATUS_CHANGE, sizeof(bytes), bytes);

	/* We need to send zero's to trigger calibration */
	memset(bytes, 0, sizeof(bytes));
	err = tfa_dsp_cmd_id_write(tfa98xx[0]->tfa, MODULE_SPEAKERBOOST, SB_PARAM_SET_RE25C, sizeof(bytes), bytes);

	/* Wait a maximum of 5 seconds to get the calibration results */
	for (i=0; i < 50; i++ ) {
		/* Avoid busload */
		msleep_interruptible(100);

		/* Get the GetStatusChange results */
		err = tfa_dsp_cmd_id_write_read(tfa98xx[0]->tfa, MODULE_FRAMEWORK, FW_PAR_ID_GET_STATUS_CHANGE, sizeof(bytes), bytes);

		/* If the calibration trigger is set break the loop */
		if(bytes[5] & TFADSP_FLAG_CALIBRATE_DONE) {
			break;
		}
		/* bit1 is TFADSP_FLAG_DAMAGED_SPEAKER_P 
		   bit2 is TFADSP_FLAG_DAMAGED_SPEAKER_S
		*/
		if(bytes[2] & 0x6) {
			if (bytes[2] & 0x2)
				pr_info("%s: ##ERROR## Left SPK damaged event detected 0x%x\n", __func__, bytes[2]);
			if (bytes[2] & 0x4)
				pr_info("%s: ##ERROR## Right SPK damaged event detected 0x%x\n", __func__, bytes[2]);
			break;
		}
	}

	err = tfa_dsp_get_calibration_impedance(tfa98xx[0]->tfa);
	for (i = 0; i < devices; i++){

		spk_imp = tfa_get_calibration_info(tfa98xx[0]->tfa, i);
		
		dev_info(tfa98xx[0]->dev, "%s: Calibration value: %d.%03d ohm\n", __func__,
					spk_imp/1000, spk_imp%1000);
		
		if(spk_imp == 0xc00 && (bytes[2] & (2 << i))) {
			dev_info(tfa98xx[i]->dev, "%s: set 7.399 ohm to MTP due to spk damaged\n",
					__func__ );
			
			/* set the default 7.399 ohm to MTP */
			spk_imp = 7399;
		}

		/* Set MTPOTC to indicate calibration is done! */
		err = tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_OTC, 1);
		if (err) {
			dev_err(tfa98xx[i]->dev, "OTC write failed\n");
			return err;
		}

		/* Write calibration value to MTP */
		err = tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_RE25, spk_imp);
		if (err) {
			dev_err(tfa98xx[i]->dev, "MTP Re25 write failed\n");
			return err;
		}

		/* Set MTPEX to indicate calibration is done! */
		err = tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_EX, 2);
		if (err) {
			dev_err(tfa98xx[i]->dev, "MTPEX write failed\n");
			return err;
		}

		/* Save the current profile */
		tfa_dev_set_swprof(tfa98xx[i]->tfa, (unsigned short)profile);
	}
	
	return Tfa98xx_Error_Ok;

}

static int tfa98xx_do_regdump(struct tfa98xx *tfa98xx)
{
	int ret = 0;
	unsigned int reg, val;
	if (!tfa98xx || !tfa98xx->regmap) {
		pr_err("No tfa98xx regmap available\n");
		return -2;
	}

	for (reg = 0x00; reg < 0x80; reg++) {
		ret = regmap_read(tfa98xx->regmap, reg, &val);
		if (ret < 0) {
			dev_err(tfa98xx->dev, "i2c error at reg 0x%x\n", reg);
			return -3;
		}
		else {
			dev_info(tfa98xx->dev, "%02x:%04x\n", reg, val & 0xffff);
		}
	}

	return ret;
}

static int tfa9894_do_calibrate(struct tfa98xx *tfa98xx_pri, struct tfa98xx *tfa98xx_sec)
{
	int profile, cal_profile, return_to_profile;
	int err, i, spk_imp = 0, devices;

	struct tfa98xx *tfa98xx[2] = {tfa98xx_pri, tfa98xx_sec};

	devices = (tfa98xx_device_count > 2) ? 2 : tfa98xx_device_count;
	if (devices == 2 && 
		tfa98xx_sec == NULL) {
		pr_err("device count match up failed\n");
		
		return Tfa98xx_Error_Fail;
	}
	
	pr_info("tfa98xx_do_calibrate started profile %d\n", profile);
	
	for (i = 0; i < devices; i++){
		return_to_profile = tfa_dev_get_swprof(tfa98xx[i]->tfa);

		cal_profile = tfaContGetCalProfile(tfa98xx[i]->tfa);
		if (cal_profile >= 0)
			profile = cal_profile;
		else
			profile = 0;

		/* write all the registers from the profile list */
		err = tfaContWriteRegsProf(tfa98xx[i]->tfa, profile);
		if (err){
			dev_err(tfa98xx[i]->dev, "MTP write failed\n");
			return err;
		}

		err = tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_OTC, 1);
		if (err) {
			dev_err(tfa98xx[i]->dev, "OTC write failed\n");
			return err;
		}

		err = tfa_dev_mtp_set(tfa98xx[i]->tfa, TFA_MTP_EX, 0);
		if (err) {
			dev_err(tfa98xx[i]->dev, "MTPOTC write failed\n");
			return err;
		}

		/* Configure devide into cold boot state for re-calibration */
		err = tfaRunColdStartup(tfa98xx[i]->tfa, profile);
		if (err) {
			dev_err(tfa98xx[i]->dev, "It is not possible to set device into cold-boot state!\n");
			return err;
		}

		/* Startup the device, ending in initCF state */
		err = tfaRunSpeakerBoost(tfa98xx[i]->tfa, 0, profile);
		if (err) {
			dev_err(tfa98xx[i]->dev, "It is not possible to start device up!\n");
			return err;
		}

		/* Check if CF is not in bypass */
		if (!tfa_cf_enabled(tfa98xx[i]->tfa)) {
			dev_err(tfa98xx[i]->dev, "It is not possible to calibrate with CF in bypass! \n");
			return Tfa98xx_Error_DSP_not_running;
		}

		tfa_dev_set_state(tfa98xx[i]->tfa, TFA_STATE_OPERATING, 1);

		/*Make sure all information is stored in MTP*/
		regmap_write(tfa98xx[i]->regmap, 0xa3, 0x20);

		if (tfa_dev_mtp_get(tfa98xx[i]->tfa, TFA_MTP_EX) ) {
			/* Go to the Operating state */
			err = tfa_dsp_get_calibration_impedance(tfa98xx[i]->tfa);
		}
		else
		{
			dev_err(tfa98xx[i]->dev, "****Calibrate Failed****\n");
			return Tfa98xx_Error_Fail;
		}

		spk_imp = tfa_get_calibration_info(tfa98xx[i]->tfa, 0);

		dev_info(tfa98xx[i]->dev, "%s: Calibration value: %d.%03d ohm\n", __func__,
					spk_imp/1024, (spk_imp%1024)*1000/1024);

		/* Unmute after calibration */
		tfa98xx_set_mute(tfa98xx[i]->tfa, Tfa98xx_Mute_Off);

		/* After loading calibration profile we need to return to previous profile */
		err = tfaContWriteProfile(tfa98xx[i]->tfa, return_to_profile, 0);

		/* Always search and apply filters after a startup */
		err = tfa_set_filters(tfa98xx[i]->tfa, return_to_profile);

		/* Save the current profile */
		tfa_dev_set_swprof(tfa98xx[i]->tfa, (unsigned short)return_to_profile);
	}

	return err;
}

static char firmware_file[64] = { 0 };
static long tfa98xx_ioctl(struct file *f, 
	unsigned int cmd, void __user *arg)
{
	struct tfa98xx *tfa98xx_pri = NULL, *tfa98xx_sec = NULL;
	int ret = 0, value = 0;

	tfa98xx_pri = list_last_entry(&tfa98xx_device_list, struct tfa98xx, list);
	if (tfa98xx_device_count >= 2) {
		/*left/right selector*/
		if (!tfa98xx_pri->is_primary){
			tfa98xx_sec = tfa98xx_pri;
			tfa98xx_pri = list_prev_entry(tfa98xx_pri, list);
		}
		else {
			tfa98xx_sec = list_prev_entry(tfa98xx_pri, list);
		}
	}

	if (tfa98xx_pri == NULL) {
		return -1;
	}
	
	switch(cmd) {
		case TFA98XX_SWITCH_FIRMWARE_L:
		{
			uint8_t len = 0;
			char *pstr = (char*)arg;
			if (copy_from_user(&len, pstr, sizeof(uint8_t))) {
				dev_err(tfa98xx_pri->dev, "copy from user failed\n");
				ret = -EFAULT;
				goto exit;
			}
			
			/* check target buffer size */
			if (len > (sizeof(spk_box)-1)) {
				dev_err(tfa98xx_pri->dev, "firmware name too long (%d)\n", len);
				ret = -EFAULT;
				goto exit;
			}

			memset(spk_box, 0x00, sizeof(spk_box));
			if (copy_from_user(spk_box, pstr+1, len)) {
				dev_err(tfa98xx_pri->dev, "copy from user failed\n");
				ret = -EFAULT;
				goto exit;
			}

			fw_name = firmware_file;

			memset(firmware_file, 0x00, sizeof(firmware_file));
			if ((tfa98xx_pri->rev & 0xff) == 0x74) {
				sprintf(firmware_file, "tfa98xx_%s.cnt", "NONE");
			} else {
				sprintf(firmware_file, "tfa98xx_%s.cnt", spk_box);
			}
			dev_info(tfa98xx_pri->dev, "TFA98XX_SWITCH_FIRMWARE_L  fw_name=[%s]  box_name=[%s]\n",
					 fw_name, spk_box);
			ret = tfa98xx_ioctrl_probe(tfa98xx_pri);
			if(ret < 0) {
				dev_err(tfa98xx_pri->dev, "Failed to initialize Primary TFA98XX Ampilier: %d", ret);
				goto exit;
			}
			/*Also init the second channel if it exist*/
			if (tfa98xx_sec) {
				ret = tfa98xx_ioctrl_probe(tfa98xx_sec);
				if(ret < 0) {
					dev_err(tfa98xx_sec->dev, "Failed to initialize Primary TFA98XX Ampilier: %d", ret);
					goto exit;
				}
			}
			break;
		}
		case TFA98XX_SWITCH_CONFIG_L:
			if(copy_from_user(&value, arg, sizeof(value))) {
				dev_err(tfa98xx_pri->dev, "copy from user failed\n");
				ret = -EFAULT;
				goto exit;
			}
			if (tfa98xx_pri->tfa->is_probus_device) {
				mutex_lock(&tfa98xx_mutex);
				algo_id = (value >> 4) & 0xf;
				mutex_unlock(&tfa98xx_mutex);
				tfa98xx_ioctrl_set_profile(tfa98xx_pri, value & 0xf);
			} else {
				tfa98xx_ioctrl_set_profile(tfa98xx_pri, (value >> 4) & 0xf);
			}
			tfa98xx_ioctrl_mute(tfa98xx_pri, 0);
			break;	
		case TFA98XX_POWER_OFF_L:
			tfa98xx_ioctrl_mute(tfa98xx_pri, 1);
			break;	 
		case TFA98XX_GET_CALIB_STATE_L:
			if (tfa98xx_pri->dsp_fw_state == TFA98XX_DSP_FW_OK) {
				mutex_lock(&tfa98xx_pri->dsp_lock);
				if(tfa_get_bf(tfa98xx_pri->tfa, TFA2_BF_MTPOTC) &&
				   tfa_get_bf(tfa98xx_pri->tfa, TFA2_BF_MTPEX)) {
					value = 1; //default calibration has been set
				} else {
					value = 0; //default calibration has never been set
				}
				mutex_unlock(&tfa98xx_pri->dsp_lock);
			}
			else { /*tfa98xx device is not ready*/
				value = 1;
			}
			
			ret = copy_to_user(arg, &value, sizeof(value)); 		
			break;			 
		case TFA98XX_SET_RE25C_L:
			if(copy_from_user(&value, arg, sizeof(value))) {
				dev_err(tfa98xx_pri->dev, "copy from user failed\n");
				ret = -EFAULT;
				goto exit;
			}

			if (tfa98xx_pri->dsp_fw_state == TFA98XX_DSP_FW_OK) {
				ret = tfa98xx_set_default_calib(tfa98xx_pri, value);
				if(ret != 0) {
					dev_err(tfa98xx_pri->dev, "Failed to set default calibration value: %d", ret);
				}
			}
			break;
		case TFA98XX_SWITCH_FIRMWARE_R:
			if (tfa98xx_sec == NULL)
				return -2;
			
			ret = tfa98xx_ioctrl_probe(tfa98xx_sec);
			if(ret < 0) {
				dev_err(tfa98xx_sec->dev, "Failed to initialize Primary TFA98XX Ampilier: %d", ret);
				goto exit;
			}
			break;
		case TFA98XX_SWITCH_CONFIG_R:
			if (tfa98xx_sec == NULL)
				return -2;
						
			if(copy_from_user(&value, arg, sizeof(value))) {
				dev_err(tfa98xx_sec->dev, "copy from user failed\n");
				ret = -EFAULT;
				goto exit;
			}
			
			if (tfa98xx_sec->tfa->is_probus_device) {
				tfa98xx_ioctrl_set_profile(tfa98xx_sec, value & 0xf);
			} else {
				tfa98xx_ioctrl_set_profile(tfa98xx_sec, (value >> 4) & 0xf);
			}
			tfa98xx_ioctrl_mute(tfa98xx_sec, 0);
			break;	
		case TFA98XX_POWER_OFF_R:
			if (tfa98xx_sec == NULL)
				return -2;
						
			tfa98xx_ioctrl_mute(tfa98xx_sec, 1);
			break;	 
		case TFA98XX_GET_CALIB_STATE_R:
			if (tfa98xx_sec == NULL)
				return -2;
						
			if (tfa98xx_sec->dsp_fw_state == TFA98XX_DSP_FW_OK) {
				mutex_lock(&tfa98xx_sec->dsp_lock);
				if(tfa_get_bf(tfa98xx_sec->tfa, TFA2_BF_MTPOTC) &&
				   tfa_get_bf(tfa98xx_sec->tfa, TFA2_BF_MTPEX)) {
					value = 1; //default calibration has been set
				} else {
					value = 0; //default calibration has never been set
				}
				mutex_unlock(&tfa98xx_sec->dsp_lock);
			}
			else { /*tfa98xx device is not ready*/
				value = 1;
			}
			
			ret = copy_to_user(arg, &value, sizeof(value)); 		
			break;			 
		case TFA98XX_SET_RE25C_R:
			if (tfa98xx_sec == NULL)
				return -2;
						
			if(copy_from_user(&value, arg, sizeof(value))) {
				dev_err(tfa98xx_sec->dev, "copy from user failed\n");
				ret = -EFAULT;
				goto exit;
			}
		
			if (tfa98xx_sec->dsp_fw_state == TFA98XX_DSP_FW_OK) {
				ret = tfa98xx_set_default_calib(tfa98xx_sec, value);
				if(ret != 0) {
					dev_err(tfa98xx_sec->dev, "Failed to set default calibration value: %d", ret);
				}
			}
			break;
		case TFA98XX_SET_ALGO_ID:
			if(copy_from_user(&value, arg, sizeof(value))) {
				dev_err(tfa98xx_pri->dev, "copy from user failed\n");
				ret = -EFAULT;
				goto exit;
			}
			mutex_lock(&tfa98xx_mutex);
			algo_id = (value >> 4) & 0xf;
			dev_info(tfa98xx_pri->dev, "TFA98XX_SET_ALGO_ID algo_id=%d\n", algo_id);
			mutex_unlock(&tfa98xx_mutex);
			break;
		case TFA98XX_GET_BOX_ID:
		{
			uint8_t len = 0;
			/* read buffer length from 1st byte. */
			if (copy_from_user(&len, arg, sizeof(uint8_t))) {
				dev_err(tfa98xx_pri->dev, "copy from user failed\n");
				ret = -EFAULT;
				goto exit;
			}
			
			dev_info(tfa98xx_pri->dev, "TFA98XX_GET_BOX_ID user buffer len=%d, box name=[%s]\n", len, spk_box);
			if (len < strlen(spk_box)) {
				dev_err(tfa98xx_pri->dev, "user buffer is less than string length\n");
				ret = -ENOMEM;
				goto exit;
			}
			/* copy box name to user space */
			ret = copy_to_user(arg, spk_box, strlen(spk_box)+1);
			break;
		}
		case TFA98XX_GET_ALGO_ID:
			mutex_lock(&tfa98xx_mutex);
			value = algo_id;
			dev_info(tfa98xx_pri->dev, "TFA98XX_GET_ALGO_ID algo_id=%d\n", algo_id);
			mutex_unlock(&tfa98xx_mutex);

			ret = copy_to_user(arg, &value, sizeof(value));
			break;
		case TFA98XX_GET_DEV_NUM:
			ret = copy_to_user(arg, &tfa98xx_device_count, sizeof(tfa98xx_device_count));

			break;
		case TFA98XX_DO_CALIBRATE:
			if (tfa98xx_pri->tfa->is_probus_device) {
				ret = tfa98xx_do_calibrate(tfa98xx_pri, tfa98xx_sec);
				if(ret != 0) {
					dev_err(tfa98xx_pri->dev, "tfa98xx_do_calibrate failed with return: %d", ret);
				}
			}
			else 
			{
				ret = tfa9894_do_calibrate(tfa98xx_pri, tfa98xx_sec);
				if(ret != 0) {
					dev_err(tfa98xx_pri->dev, "tfa9894_do_calibrate failed with return: %d", ret);
				}			 
			}
			ret = copy_to_user(arg, &tfa98xx_device_count, sizeof(tfa98xx_device_count));
			break;
		case TFA98XX_DO_REGDUMP:
			ret = tfa98xx_do_regdump(tfa98xx_pri);
			if(ret != 0) {
				pr_err("tfa98xx_do_regdump pri failed with return: %d", ret);
			}

			if (tfa98xx_sec != NULL) {
				ret = tfa98xx_do_regdump(tfa98xx_sec);
				if(ret != 0) {
					pr_err("tfa98xx_do_regdump sec failed with return: %d", ret);
				}
			}

			break;
		default:
			dev_info(tfa98xx_pri->dev, "COMMAND = 0x%08x\n", cmd);
			break;
		}
exit:
	return ret;
}

static long tfa98xx_unlocked_ioctl(struct file *f, 
	unsigned int cmd, unsigned long arg)
{
	return tfa98xx_ioctl(f, cmd, (void __user *)arg);
}

#ifdef CONFIG_COMPAT
static long tfa98xx_compat_ioctl(struct file *f, 
	unsigned int cmd, unsigned long arg)
{
	struct miscdevice *dev = f->private_data;
	struct tfa98xx *tfa98xx;
	unsigned int cmd64;

	tfa98xx = container_of(dev, struct tfa98xx, misc_dev);
	switch (cmd) {
	case TFA98XX_SWITCH_FIRMWARE_L_COMPAT:
		cmd64 = TFA98XX_SWITCH_FIRMWARE_L;
		break;
	case TFA98XX_SWITCH_CONFIG_L_COMPAT:
		cmd64 = TFA98XX_SWITCH_CONFIG_L;
		break;
	case TFA98XX_POWER_OFF_L_COMPAT:
		cmd64 = TFA98XX_POWER_OFF_L;
		break;
	case TFA98XX_GET_CALIB_STATE_L_COMPAT:
		cmd64 = TFA98XX_GET_CALIB_STATE_L;
		break;			 
	case TFA98XX_SET_RE25C_L_COMPAT:
		cmd64 = TFA98XX_SET_RE25C_L;
		break;
	case TFA98XX_SWITCH_FIRMWARE_R_COMPAT:
		cmd64 = TFA98XX_SWITCH_FIRMWARE_R;
		break;
	case TFA98XX_SWITCH_CONFIG_R_COMPAT:
		cmd64 = TFA98XX_SWITCH_CONFIG_R;
		break;
	case TFA98XX_POWER_OFF_R_COMPAT:
		cmd64 = TFA98XX_POWER_OFF_R;
		break;
	case TFA98XX_GET_CALIB_STATE_R_COMPAT:
		cmd64 = TFA98XX_GET_CALIB_STATE_R;
		break;			 
	case TFA98XX_SET_RE25C_R_COMPAT:
		cmd64 = TFA98XX_SET_RE25C_R;
		break;
	case TFA98XX_SET_ALGO_ID_COMPAT:
		cmd64 = TFA98XX_SET_ALGO_ID;
		break;
	case TFA98XX_GET_BOX_ID_COMPAT:
		cmd64 = TFA98XX_GET_BOX_ID;
		break;
	case TFA98XX_GET_ALGO_ID_COMPAT:
		cmd64 = TFA98XX_GET_ALGO_ID;
		break;
	case TFA98XX_GET_DEV_NUM_COMPAT:
		cmd64 = TFA98XX_GET_DEV_NUM;
		break;
	case TFA98XX_DO_CALIBRATE_COMPAT:
		cmd64 = TFA98XX_DO_CALIBRATE;
		break;
	case TFA98XX_DO_REGDUMP_COMPAT:
		cmd64 = TFA98XX_DO_REGDUMP;
		break;

	default:
		dev_info(tfa98xx->dev, "COMMAND = 0x%08x Not Imple\n", cmd);
		return 0;
	}
	
	return tfa98xx_ioctl(f, cmd64, (void __user *)arg);
}
#endif

static ssize_t tfa98xx_ioctrl_rpc_read(struct file *file,
					 char __user *user_buf, size_t count,
					 loff_t *ppos)
{
	struct tfa98xx *tfa98xx_pri = NULL, *tfa98xx_sec = NULL;
	int ret = 0, i;
	uint8_t *buffer = NULL;
	enum Tfa98xx_Error error;

	tfa98xx_pri = list_last_entry(&tfa98xx_device_list, struct tfa98xx, list);
	if (tfa98xx_device_count == 2) {
		/*left/right selector*/
		if (!tfa98xx_pri->is_primary){
			tfa98xx_sec = tfa98xx_pri;
			tfa98xx_pri = list_prev_entry(tfa98xx_pri, list);
		}
		else {
			tfa98xx_sec = list_prev_entry(tfa98xx_pri, list);
		}
	}

	if (tfa98xx_pri == NULL ||
		tfa98xx_pri->tfa == NULL) {
		pr_debug("[0x%x] dsp is not available\n", tfa98xx_pri->i2c->addr);
		return -ENODEV;
	}

	if (count == 0)
		return 0;

	buffer = kmalloc(count*2, GFP_KERNEL|__GFP_ZERO);
	if (buffer == NULL) {
		pr_debug("[0x%x] can not allocate memory\n", tfa98xx_pri->i2c->addr);
		return -ENOMEM;
	}

	mutex_lock(&tfa98xx_pri->dsp_lock);
	
	if (tfa98xx_pri->tfa->is_probus_device) {
		error = tfa98xx_recv_data_from_dsp(buffer, count);
	} else {
		error = tfa_dsp_msg_read(tfa98xx_pri->tfa, count, buffer);
	}

	mutex_unlock(&tfa98xx_pri->dsp_lock);

	/*Read secodary channel if available*/
	if (tfa98xx_sec && tfa98xx_sec->tfa->is_probus_device == 0) {
		mutex_lock(&tfa98xx_sec->dsp_lock);
		error = tfa_dsp_msg_read(tfa98xx_sec->tfa, count, &buffer[count]);
		mutex_unlock(&tfa98xx_sec->dsp_lock);
	}
	
	if (error != Tfa98xx_Error_Ok) {
		pr_debug("[0x%x] dsp_msg_read error: %d\n", tfa98xx_pri->i2c->addr, error);
		kfree(buffer);
		return -EFAULT;
	}

	/*Interleave two channels data together*/
	if (tfa98xx_pri->tfa->is_probus_device == 0) {
		uint8_t offset = count % 6;
		uint8_t *srcl = &buffer[offset];
		uint8_t *srcr = &buffer[count+offset];
		uint8_t *dst  = &buffer[offset];
		
		for(i = count/6 - 1; i >= 0; i--) {
			dst[i*6+5] = srcr[i*3+2];
			dst[i*6+4] = srcr[i*3+1];
			dst[i*6+3] = srcr[i*3+0];
			dst[i*6+2] = srcl[i*3+2];
			dst[i*6+1] = srcl[i*3+1];
			dst[i*6+0] = srcl[i*3+0];
		}	
	}

	ret = copy_to_user(user_buf, buffer, count);
	
	kfree(buffer);
	if (ret)
		return -EFAULT;

	*ppos += count;
	return count;
}

static uint8_t tfa9894_memtrack[] = {
	 0x04, 0x80, 0x0b, 0x00, 0x00, 0x03,
	 0x02, 0x07, 0x0b, 0x02, 0x07, 0x05,
	 0x02, 0x06, 0xfc, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static ssize_t tfa98xx_ioctrl_rpc_send(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct tfa98xx *tfa98xx_pri = NULL, *tfa98xx_sec = NULL;
	TfaFileDsc_t *msg_file = NULL;
	enum Tfa98xx_Error error;
	int err = 0;

	tfa98xx_pri = list_last_entry(&tfa98xx_device_list, struct tfa98xx, list);
	if (tfa98xx_device_count == 2) {
		/*left/right selector*/
		if (!tfa98xx_pri->is_primary){
			tfa98xx_sec = tfa98xx_pri;
			tfa98xx_pri = list_prev_entry(tfa98xx_pri, list);
		}
		else {
			tfa98xx_sec = list_prev_entry(tfa98xx_pri, list);
		}
	}

	if (tfa98xx_pri == NULL || 
		tfa98xx_pri->tfa == NULL) {
		pr_debug("[0x%x] dsp is not available\n", tfa98xx_pri->i2c->addr);
		return -ENODEV;
	}

	if (count == 0)
		return 0;

	/* msg_file.name is not used */
	msg_file = kmalloc(count + sizeof(TfaFileDsc_t), GFP_KERNEL|__GFP_ZERO);
	if ( msg_file == NULL ) {
		pr_debug("[0x%x] can not allocate memory\n", tfa98xx_pri->i2c->addr);
		return	-ENOMEM;
	}
	msg_file->size = count;

	if (copy_from_user(msg_file->data, user_buf, count)) {
		err = -EFAULT;
		goto exit;
	}

	mutex_lock(&tfa98xx_pri->dsp_lock);
	
	if (tfa98xx_pri->tfa->is_probus_device) {
		err = tfa98xx_send_data_to_dsp(msg_file->data, msg_file->size);

		if (err != 0)
			pr_err("[0x%x] dsp_msg error: %d\n", tfa98xx_pri->i2c->addr, err);

		mdelay(3);
		
	} else {
		if ((msg_file->data[0] == 'M') && (msg_file->data[1] == 'G')) {
			error = tfaContWriteFile(tfa98xx_pri->tfa, msg_file, 0, 0); /* int vstep_idx, int vstep_msg_idx both 0 */
			if (error != Tfa98xx_Error_Ok) {
				pr_debug("[0x%x] tfaContWriteFile error: %d\n", tfa98xx_pri->i2c->addr, error);
				err = -EIO;
			}
		} else {
			if (msg_file->data[1] == 0x80 && 
				(msg_file->data[2] == 0x0b || msg_file->data[2] == 0x3f)) {
				memcpy(msg_file->data, tfa9894_memtrack, count);
			}
			else {
				msg_file->data[0] = 0x04;
			}
			
			error = tfa_dsp_msg(tfa98xx_pri->tfa, msg_file->size, msg_file->data);
			if (error != Tfa98xx_Error_Ok) {
				pr_debug("[0x%x] dsp_msg error: %d\n", tfa98xx_pri->i2c->addr, error);
				err = -EIO;
			}
		}
		
	}
	
	mutex_unlock(&tfa98xx_pri->dsp_lock);

	/*Write secodary channel if available*/
	if (tfa98xx_sec && tfa98xx_sec->tfa->is_probus_device == 0) {
		mutex_lock(&tfa98xx_sec->dsp_lock);

		error = tfa_dsp_msg(tfa98xx_sec->tfa, msg_file->size, msg_file->data);
		if (error != Tfa98xx_Error_Ok) {
			pr_debug("[0x%x] dsp_msg error: %d\n", tfa98xx_sec->i2c->addr, error);
			err = -EIO;
		}

		mutex_unlock(&tfa98xx_sec->dsp_lock);
	}
		
exit:
	kfree(msg_file);

	if (err)
		return err;
	return count;
}

static const struct file_operations tfa98xx_fops = {
	.owner = THIS_MODULE,
	.read = tfa98xx_ioctrl_rpc_read,
	.write = tfa98xx_ioctrl_rpc_send,
	.unlocked_ioctl = tfa98xx_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tfa98xx_compat_ioctl,
#endif
};

#define VENDOR_GOODIX		1

extern void tfa98xx_set_vendor_info(int id, char *chip) __attribute__((weak));

void tfa98xx_set_vendor_info(int id, char *chip){
	(void)id;
	(void)chip;
	pr_info("This is weak symbol, do nothing\n");
}

#endif /*ENABLE_IOCTRL_INTERFACE*/

static int tfa98xx_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct snd_soc_dai_driver *dai	= NULL;
	struct tfa98xx *tfa98xx 		= NULL;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags;
	unsigned int reg;
	int ret;

	pr_info("addr=0x%x\n", i2c->addr);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	tfa98xx = devm_kzalloc(&i2c->dev, sizeof(struct tfa98xx), GFP_KERNEL);
	if (tfa98xx == NULL)
		return -ENOMEM;

	tfa98xx->dev = &i2c->dev;
	tfa98xx->i2c = i2c;
	tfa98xx->dsp_init = TFA98XX_DSP_INIT_STOPPED;
	tfa98xx->rate = 48000; /* init to the default sample rate (48kHz) */
	tfa98xx->tfa = NULL;

	tfa98xx->regmap = devm_regmap_init_i2c(i2c, &tfa98xx_regmap);
	if (IS_ERR(tfa98xx->regmap)) {
		ret = PTR_ERR(tfa98xx->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	i2c_set_clientdata(i2c, tfa98xx);
	mutex_init(&tfa98xx->dsp_lock);
	init_waitqueue_head(&tfa98xx->wq);

	if (np) {
		ret = tfa98xx_parse_dt(&i2c->dev, tfa98xx, np);
		if (ret) {
			dev_err(&i2c->dev, "Failed to parse DT node\n");
			return ret;
		}
		if (no_start)
			tfa98xx->irq_gpio = -1;
		if (no_reset)
			tfa98xx->reset_gpio = -1;
	}
	else {
		tfa98xx->reset_gpio = -1;
		tfa98xx->irq_gpio = -1;
	}

	if (gpio_is_valid(tfa98xx->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, tfa98xx->reset_gpio,
			GPIOF_OUT_INIT_LOW, "TFA98XX_RST");
		if (ret)
			return ret;
	}

	if (gpio_is_valid(tfa98xx->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, tfa98xx->irq_gpio,
			GPIOF_DIR_IN, "TFA98XX_INT");
		if (ret)
			return ret;
	}

	/* Power up! */
	tfa98xx_ext_reset(tfa98xx);

	if ((no_start == 0) && (no_reset == 0)) {
		ret = regmap_read(tfa98xx->regmap, 0x03, &reg);
		if (ret < 0) {
			dev_err(&i2c->dev, "Failed to read Revision register: %d\n",
				ret);
			return -EIO;
		}

		tfa98xx->rev = reg & 0xffff;

		switch (reg & 0xff) {
		case 0x72: /* tfa9872 */
			pr_info("TFA9872 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_CALIBRATION_CTL;
			tfa98xx->flags |= TFA98XX_FLAG_REMOVE_PLOP_NOISE;
			/* tfa98xx->flags |= TFA98XX_FLAG_LP_MODES; */
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x73: /* tfa9873 */
			pr_info("TFA9873 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_CALIBRATION_CTL;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			tfa98xx->flags |= TFA98XX_FLAG_ADAPT_NOISE_MODE; /***MCH_TO_TEST***/
			break;
		case 0x74: /* tfa9874 */
			pr_info("TFA9874 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_CALIBRATION_CTL;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x78: /* tfa9878 */
			pr_info("TFA9878 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_CALIBRATION_CTL;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x88: /* tfa9888 */
			pr_info("TFA9888 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_STEREO_DEVICE;
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x13: /* tfa9912 */
			pr_info("TFA9912 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			/* tfa98xx->flags |= TFA98XX_FLAG_TAPDET_AVAILABLE; */
			break;
		case 0x94: /* tfa9894 */
			pr_info("TFA9894 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x80: /* tfa9890 */
		case 0x81: /* tfa9890 */
			pr_info("TFA9890 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			break;
		case 0x92: /* tfa9891 */
			pr_info("TFA9891 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_SAAM_AVAILABLE;
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			break;
		case 0x12: /* tfa9895 */
			pr_info("TFA9895 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			break;
		case 0x97:
			pr_info("TFA9897 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		case 0x96:
			pr_info("TFA9896 detected\n");
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			tfa98xx->flags |= TFA98XX_FLAG_TDM_DEVICE;
			break;
		default:
			pr_info("Unsupported device revision (0x%x)\n", reg & 0xff);
			if (gpio_is_valid(tfa98xx->irq_gpio))
				devm_gpio_free(&i2c->dev, tfa98xx->irq_gpio);
			if (gpio_is_valid(tfa98xx->reset_gpio))
				devm_gpio_free(&i2c->dev, tfa98xx->reset_gpio); 		   
			return -EINVAL;
		}
	}
#ifdef TFA9894_STEREO
        tfa98xx->flags |= TFA98XX_FLAG_CHIP_SELECTED;
#endif

	tfa98xx->tfa = devm_kzalloc(&i2c->dev, sizeof(struct tfa_device), GFP_KERNEL);
	if (tfa98xx->tfa == NULL)
		return -ENOMEM;

	tfa98xx->tfa->data = (void *)tfa98xx;
	tfa98xx->tfa->cachep = tfa98xx_cache;

#ifndef DISABLE_TFA98XX_ALSA_SUPPORT

	/* Modify the stream names, by appending the i2c device address.
	 * This is used with multicodec, in order to discriminate the devices.
	 * Stream names appear in the dai definition and in the stream		 .
	 * We create copies of original structures because each device will
	 * have its own instance of this structure, with its own address.
	 */
	dai = devm_kzalloc(&i2c->dev, sizeof(tfa98xx_dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;
	memcpy(dai, tfa98xx_dai, sizeof(tfa98xx_dai));

	tfa98xx_append_i2c_address(&i2c->dev,
		i2c,
		NULL,
		0,
		dai,
		ARRAY_SIZE(tfa98xx_dai));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	ret = devm_snd_soc_register_component(&i2c->dev,
				&soc_codec_dev_tfa98xx, dai,
				ARRAY_SIZE(tfa98xx_dai));
#else
	ret = snd_soc_register_codec(&i2c->dev,
				&soc_codec_dev_tfa98xx, dai,
				ARRAY_SIZE(tfa98xx_dai));
#endif
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to register TFA98xx: %d\n", ret);
		return ret;
	}

#endif 

	if (gpio_is_valid(tfa98xx->irq_gpio) &&
		!(tfa98xx->flags & TFA98XX_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
			gpio_to_irq(tfa98xx->irq_gpio),
			NULL, tfa98xx_irq, irq_flags,
			"tfa98xx", tfa98xx);
		if (ret != 0) {
			dev_err(&i2c->dev, "Failed to request IRQ %d: %d\n",
				gpio_to_irq(tfa98xx->irq_gpio), ret);
			return ret;
		}
	}
	else {
		dev_info(&i2c->dev, "Skipping IRQ registration\n");
		/* disable feature support if gpio was invalid */
		tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
	}

	if (no_start == 0)
		tfa98xx_debug_init(tfa98xx, i2c);

	/* Register the sysfs files for climax backdoor access */
	ret = device_create_bin_file(&i2c->dev, &dev_attr_rw);
	if (ret)
		dev_info(&i2c->dev, "error creating sysfs files\n");
	ret = device_create_bin_file(&i2c->dev, &dev_attr_reg);
	if (ret)
		dev_info(&i2c->dev, "error creating sysfs files\n");
		
#ifdef ENABLE_IOCTRL_INTERFACE
		if (tfa98xx_device_count == 0) {
			tfa98xx->misc_dev.minor = MISC_DYNAMIC_MINOR;
			tfa98xx->misc_dev.name = "tfa98xx";
			tfa98xx->misc_dev.fops = &tfa98xx_fops;
	
			ret = misc_register(&tfa98xx->misc_dev);
			if (ret)
				dev_info(&i2c->dev, "error creating misc device\n");
		}
		
		(void)dai;
		
		tfa98xx_set_vendor_info(VENDOR_GOODIX, "tfa98xx");
#endif /*ENABLE_IOCTRL_INTERFACE*/

#ifdef TFA98xx_F0_tracking
	ret = device_create_file(&i2c->dev,&dev_attr_cal_f0);
	if (ret) {
		dev_err(&i2c->dev,"failed to create device file for cal f0\n");
		return ret;
	}
#endif

#ifdef TFA98xx_memtrack_nondsp
	ret = device_create_file(&i2c->dev,&dev_attr_memtrack_nondsp);
	if (ret) {
		dev_err(&i2c->dev,"failed to create device file for cal f0\n");
		return ret;
	}
#endif

	pr_info("%s Probe completed successfully!\n", __func__);

	INIT_LIST_HEAD(&tfa98xx->list);

	mutex_lock(&tfa98xx_mutex);
#ifdef TFA98xx_calibrate
	tfa98xx_global[tfa98xx_device_count] = tfa98xx;
#endif
	tfa98xx_device_count++;
	list_add(&tfa98xx->list, &tfa98xx_device_list);
	mutex_unlock(&tfa98xx_mutex);

	return 0;
}

static int tfa98xx_i2c_remove(struct i2c_client *i2c)
{
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);

	pr_debug("addr=0x%x\n", i2c->addr);

	tfa98xx_interrupt_enable(tfa98xx, false);

	cancel_delayed_work_sync(&tfa98xx->interrupt_work);
	cancel_delayed_work_sync(&tfa98xx->monitor_work);
	cancel_delayed_work_sync(&tfa98xx->init_work);
	cancel_delayed_work_sync(&tfa98xx->tapdet_work);
	cancel_delayed_work_sync(&tfa98xx->nmodeupdate_work);
#ifdef DISABLE_TFA98XX_ALSA_SUPPORT    
	cancel_delayed_work_sync(&tfa98xx->unmute_work);
#endif
	device_remove_bin_file(&i2c->dev, &dev_attr_reg);
	device_remove_bin_file(&i2c->dev, &dev_attr_rw);
#ifdef TFA98xx_F0_tracking
	device_remove_file(&i2c->dev, &dev_attr_cal_f0);
#endif

#ifdef TFA98xx_memtrack_nondsp
	device_remove_file(&i2c->dev, &dev_attr_memtrack_nondsp);
#endif

	tfa98xx_debug_remove(tfa98xx);


#ifdef ENABLE_IOCTRL_INTERFACE
	misc_deregister(&tfa98xx->misc_dev);
#else

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
	snd_soc_unregister_component(&i2c->dev);
#else
	snd_soc_unregister_codec(&i2c->dev);
#endif

#endif
	if (gpio_is_valid(tfa98xx->irq_gpio))
		devm_gpio_free(&i2c->dev, tfa98xx->irq_gpio);
	if (gpio_is_valid(tfa98xx->reset_gpio))
		devm_gpio_free(&i2c->dev, tfa98xx->reset_gpio);

	mutex_lock(&tfa98xx_mutex);
	list_del(&tfa98xx->list);
	tfa98xx_device_count--;
	if (tfa98xx_device_count == 0) {
		kfree(tfa98xx_container);
		tfa98xx_container = NULL;
	}
	mutex_unlock(&tfa98xx_mutex);

	return 0;
}

static const struct i2c_device_id tfa98xx_i2c_id[] = {
	{ "tfa98xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa98xx_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id tfa98xx_dt_match[] = {
	{.compatible = "tfa,tfa98xx" },
	{.compatible = "tfa,tfa9872" },
	{.compatible = "tfa,tfa9873" },
	{.compatible = "tfa,tfa9874" },
	{.compatible = "tfa,tfa9878" },
	{.compatible = "tfa,tfa9888" },
	{.compatible = "tfa,tfa9890" },
	{.compatible = "tfa,tfa9891" },
	{.compatible = "tfa,tfa9894" },
	{.compatible = "tfa,tfa9895" },
	{.compatible = "tfa,tfa9896" },
	{.compatible = "tfa,tfa9897" },
	{.compatible = "tfa,tfa9912" },
	{ },
};
#endif

static struct i2c_driver tfa98xx_i2c_driver = {
	.driver = {
		.name = "tfa98xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tfa98xx_dt_match),
	},
	.probe = tfa98xx_i2c_probe,
	.remove = tfa98xx_i2c_remove,
	.id_table = tfa98xx_i2c_id,
};

static int __init tfa98xx_i2c_init(void)
{
	int ret = 0;

	pr_info("TFA98XX driver version %s\n", TFA98XX_VERSION);

	/* Enable debug traces */
	tfa98xx_kmsg_regs = trace_level & 2;
	tfa98xx_ftrace_regs = trace_level & 4;

	/* Initialize kmem_cache */
	tfa98xx_cache = kmem_cache_create("tfa98xx_cache", /* Cache name /proc/slabinfo */
		PAGE_SIZE, /* Structure size, we should fit in single page */
		0, /* Structure alignment */
		(SLAB_HWCACHE_ALIGN | SLAB_RECLAIM_ACCOUNT |
			SLAB_MEM_SPREAD), /* Cache property */
		NULL); /* Object constructor */
	if (!tfa98xx_cache) {
		pr_err("tfa98xx can't create memory pool\n");
		ret = -ENOMEM;
	}

	ret = i2c_add_driver(&tfa98xx_i2c_driver);

	return ret;
}

#ifndef ENABLE_IOCTRL_INTERFACE
module_init(tfa98xx_i2c_init);
#else
late_initcall_sync(tfa98xx_i2c_init);
#endif

static void __exit tfa98xx_i2c_exit(void)
{
	i2c_del_driver(&tfa98xx_i2c_driver);
	kmem_cache_destroy(tfa98xx_cache);
}
module_exit(tfa98xx_i2c_exit);

MODULE_DESCRIPTION("ASoC TFA98XX driver");
MODULE_LICENSE("GPL");

