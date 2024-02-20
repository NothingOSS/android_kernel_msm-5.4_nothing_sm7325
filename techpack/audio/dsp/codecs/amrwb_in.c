// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011-2012, 2014, 2016-2017, 2020 The Linux Foundation. All rights reserved.
 */

#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <audio/linux/msm_audio_amrwb.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/compat.h>
#include <linux/atomic.h>
#include <asm/ioctls.h>
#include "audio_utils.h"

/* Buffer with meta*/
#define PCM_BUF_SIZE		(4096 + sizeof(struct meta_in))

/* Maximum 10 frames in buffer with meta */
#define FRAME_SIZE		(1 + ((61+sizeof(struct meta_out_dsp)) * 10))

static long amrwb_in_ioctl_shared(struct file *file,
				  unsigned int cmd, void *arg)
{
	struct q6audio_in  *audio = file->private_data;
	int rc = 0;
	int cnt = 0;

	switch (cmd) {
	case AUDIO_START: {
		struct msm_audio_amrwb_enc_config *enc_cfg;

		enc_cfg = audio->enc_cfg;
		pr_debug("%s:session id %d: default buf alloc[%d]\n", __func__,
				audio->ac->session, audio->buf_alloc);
		if (audio->enabled == 1) {
			pr_info("%s:AUDIO_START already over\n", __func__);
			rc = 0;
			break;
		}
		rc = audio_in_buf_alloc(audio);
		if (rc < 0) {
			pr_err("%s:session id %d: buffer allocation failed\n",
				__func__, audio->ac->session);
			break;
		}

		rc = q6asm_enc_cfg_blk_amrwb(audio->ac,
			audio->buf_cfg.frames_per_buf,
			enc_cfg->band_mode,
			enc_cfg->dtx_enable);

		if (rc < 0) {
			pr_err("%s:session id %d: cmd amrwb media format block failed\n",
				__func__, audio->ac->session);
			break;
		}
		if (audio->feedback == NON_TUNNEL_MODE) {
			rc = q6asm_media_format_block_pcm(audio->ac,
				audio->pcm_cfg.sample_rate,
				audio->pcm_cfg.channel_count);

			if (rc < 0) {
				pr_err("%s:session id %d: media format block failed\n",
				__func__, audio->ac->session);
				break;
			}
		}
		pr_debug("%s:session id %d: AUDIO_START enable[%d]\n",
				__func__, audio->ac->session,
				audio->enabled);
		rc = audio_in_enable(audio);
		if (!rc) {
			audio->enabled = 1;
		} else {
			audio->enabled = 0;
			pr_err("%s:session id %d: Audio Start procedure failed rc=%d\n",
				__func__, audio->ac->session, rc);
			break;
		}
		while (cnt++ < audio->str_cfg.buffer_count)
			q6asm_read(audio->ac); /* Push buffer to DSP */
		rc = 0;
		pr_debug("%s:session id %d: AUDIO_START success enable[%d]\n",
				__func__, audio->ac->session, audio->enabled);
		break;
	}
	case AUDIO_STOP: {
		pr_debug("%s:AUDIO_STOP\n", __func__);
		rc = audio_in_disable(audio);
		if (rc  < 0) {
			pr_err("%s:session id %d: Audio Stop procedure failed rc=%d\n",
				__func__, audio->ac->session, rc);
			break;
		}
		break;
	}
	case AUDIO_SET_AMRWB_ENC_CONFIG: {
		struct msm_audio_amrwb_enc_config *cfg;
		struct msm_audio_amrwb_enc_config *enc_cfg;

		enc_cfg = audio->enc_cfg;
		cfg = (struct msm_audio_amrwb_enc_config *)arg;
		if (cfg == NULL) {
			pr_err("%s: NULL config pointer for %s\n",
					__func__, "AUDIO_SET_AMRWB_ENC_CONFIG");
			rc = -EINVAL;
			break;
		}

		if (cfg->band_mode > 8) {
			pr_err("%s:session id %d: invalid band mode\n",
				__func__, audio->ac->session);
			rc = -EINVAL;
			break;
		}
		/* ToDo: AMR WB encoder accepts values between 0-8
		 * while openmax provides value between 9-17
		 * as per spec
		 */
		enc_cfg->band_mode = cfg->band_mode;
		enc_cfg->dtx_enable = (cfg->dtx_enable ? 1 : 0);
		/* Currently DSP does not support different frameformat */
		enc_cfg->frame_format = 0;
		pr_debug("%s:session id %d: band_mode = 0x%x dtx_enable=0x%x\n",
				__func__, audio->ac->session,
				enc_cfg->band_mode, enc_cfg->dtx_enable);
		break;
	}
	default:
		pr_err("%s: Unknown ioctl cmd = %d", __func__, cmd);
		rc = -EINVAL;
	}
	return rc;
}

static long amrwb_in_ioctl(struct file *file,
				  unsigned int cmd, unsigned long arg)
{
	struct q6audio_in  *audio = file->private_data;
	int rc = 0;

	switch (cmd) {
	case AUDIO_START:
	case AUDIO_STOP: {
		rc = amrwb_in_ioctl_shared(file, cmd, NULL);
		break;
	}
	case AUDIO_GET_AMRWB_ENC_CONFIG: {
		if (copy_to_user((void *)arg, audio->enc_cfg,
				sizeof(struct msm_audio_amrwb_enc_config)))
			pr_err("%s: copy_to_user for AUDIO_GET_AMRWB_ENC_CONFIG failed\n",
				__func__);
			rc = -EFAULT;
		break;
	}
	case AUDIO_SET_AMRWB_ENC_CONFIG: {
		struct msm_audio_amrwb_enc_config cfg;

		if (copy_from_user(&cfg, (void *) arg,
				sizeof(cfg))) {
			pr_err("%s: copy_from_user for AUDIO_SET_AMRWB_ENC_CONFIG failed\n",
				__func__);
			rc = -EFAULT;
			break;
		}
		rc = amrwb_in_ioctl_shared(file, cmd, &cfg);
		if (rc)
			pr_err("%s:AUDIO_SET_AAC_ENC_CONFIG failed. rc=%d\n",
				__func__, rc);
		break;
	}
	default:
		pr_err("%s: Unknown ioctl cmd = %d", __func__, cmd);
		rc = -EINVAL;
	}
	return rc;
}

#ifdef CONFIG_COMPAT
struct msm_audio_amrwb_enc_config_32 {
	u32 band_mode;
	u32 dtx_enable;
	u32 frame_format;
};

enum {
	AUDIO_GET_AMRWB_ENC_CONFIG_32 = _IOW(AUDIO_IOCTL_MAGIC,
		(AUDIO_MAX_COMMON_IOCTL_NUM+0),
		struct msm_audio_amrwb_enc_config_32),
	AUDIO_SET_AMRWB_ENC_CONFIG_32 = _IOR(AUDIO_IOCTL_MAGIC,
		(AUDIO_MAX_COMMON_IOCTL_NUM+1),
		struct msm_audio_amrwb_enc_config_32)
};

static long amrwb_in_compat_ioctl(struct file *file,
				  unsigned int cmd, unsigned long arg)
{
	struct q6audio_in  *audio = file->private_data;
	int rc = 0;

	switch (cmd) {
	case AUDIO_START:
	case AUDIO_STOP: {
		rc = amrwb_in_ioctl_shared(file, cmd, NULL);
		break;
	}
	case AUDIO_GET_AMRWB_ENC_CONFIG_32: {
		struct msm_audio_amrwb_enc_config *amrwb_config;
		struct msm_audio_amrwb_enc_config_32 amrwb_config_32;

		memset(&amrwb_config_32, 0, sizeof(amrwb_config_32));

		amrwb_config =
		(struct msm_audio_amrwb_enc_config *)audio->enc_cfg;
		amrwb_config_32.band_mode = amrwb_config->band_mode;
		amrwb_config_32.dtx_enable = amrwb_config->dtx_enable;
		amrwb_config_32.frame_format = amrwb_config->frame_format;

		if (copy_to_user((void *)arg, &amrwb_config_32,
			sizeof(struct msm_audio_amrwb_enc_config_32))) {
			pr_err("%s: copy_to_user for AUDIO_GET_AMRWB_ENC_CONFIG_32 failed\n",
				__func__);
			rc = -EFAULT;
		}
		break;
	}
	case AUDIO_SET_AMRWB_ENC_CONFIG_32: {
		struct msm_audio_amrwb_enc_config cfg_32;

		if (copy_from_user(&cfg_32, (void *) arg,
				sizeof(cfg_32))) {
			pr_err("%s: copy_from_user for AUDIO_SET_AMRWB_ENC_CONFIG_32 failed\n",
				__func__);
			rc = -EFAULT;
			break;
		}
		cmd = AUDIO_SET_AMRWB_ENC_CONFIG;
		rc = amrwb_in_ioctl_shared(file, cmd, &cfg_32);
		if (rc)
			pr_err("%s:AUDIO_SET_AAC_ENC_CONFIG failed. rc=%d\n",
				__func__, rc);
		break;
	}
	default:
		pr_err("%s: Unknown ioctl cmd = %d", __func__, cmd);
		rc = -EINVAL;
	}
	return rc;
}
#else
#define amrwb_in_compat_ioctl NULL
#endif

static int amrwb_in_open(struct inode *inode, struct file *file)
{
	struct q6audio_in *audio = NULL;
	struct msm_audio_amrwb_enc_config *enc_cfg;
	int rc = 0;

	audio = kzalloc(sizeof(struct q6audio_in), GFP_KERNEL);

	if (audio == NULL)
		return -ENOMEM;

	/* Allocate memory for encoder config param */
	audio->enc_cfg = kzalloc(sizeof(struct msm_audio_amrwb_enc_config),
				GFP_KERNEL);
	if (audio->enc_cfg == NULL) {
		kfree(audio);
		return -ENOMEM;
	}
	enc_cfg = audio->enc_cfg;

	mutex_init(&audio->lock);
	mutex_init(&audio->read_lock);
	mutex_init(&audio->write_lock);
	spin_lock_init(&audio->dsp_lock);
	init_waitqueue_head(&audio->read_wait);
	init_waitqueue_head(&audio->write_wait);

	/* Settings will be re-config at AUDIO_SET_CONFIG,
	 * but at least we need to have initial config
	 */
	audio->str_cfg.buffer_size = FRAME_SIZE;
	audio->str_cfg.buffer_count = FRAME_NUM;
	audio->min_frame_size = 32;
	audio->max_frames_per_buf = 10;
	audio->pcm_cfg.buffer_size = PCM_BUF_SIZE;
	audio->pcm_cfg.buffer_count = PCM_BUF_COUNT;
	enc_cfg->band_mode = 8;
	enc_cfg->dtx_enable = 0;
	audio->pcm_cfg.channel_count = 1;
	audio->pcm_cfg.sample_rate = 16000;
	audio->buf_cfg.meta_info_enable = 0x01;
	audio->buf_cfg.frames_per_buf = 0x01;

	audio->ac = q6asm_audio_client_alloc((app_cb)q6asm_in_cb,
				(void *)audio);

	if (!audio->ac) {
		pr_err("%s:audio[%pK]: Could not allocate memory for audio client\n",
			__func__, audio);
		kfree(audio->enc_cfg);
		kfree(audio);
		return -ENOMEM;
	}

	/* open amrwb encoder in T/NT mode */
	if ((file->f_mode & FMODE_WRITE) &&
		(file->f_mode & FMODE_READ)) {
		audio->feedback = NON_TUNNEL_MODE;
		rc = q6asm_open_read_write(audio->ac, FORMAT_AMRWB,
					FORMAT_LINEAR_PCM);
		if (rc < 0) {
			pr_err("%s:session id %d: NT mode Open failed rc=%d\n",
				__func__, audio->ac->session, rc);
			rc = -ENODEV;
			goto fail;
		}
		pr_info("%s:session id %d: NT mode encoder success\n",
				__func__, audio->ac->session);
	} else if (!(file->f_mode & FMODE_WRITE) &&
				(file->f_mode & FMODE_READ)) {
		audio->feedback = TUNNEL_MODE;
		rc = q6asm_open_read(audio->ac, FORMAT_AMRWB);
		if (rc < 0) {
			pr_err("%s:session id %d: T mode Open failed rc=%d\n",
				__func__, audio->ac->session, rc);
			rc = -ENODEV;
			goto fail;
		}
		/* register for tx overflow (valid for tunnel mode only) */
		rc = q6asm_reg_tx_overflow(audio->ac, 0x01);
		if (rc < 0) {
			pr_err("%s:session id %d: TX Overflow registration failed rc=%d\n",
				__func__, audio->ac->session,
				rc);
			rc = -ENODEV;
			goto fail;
		}
		pr_info("%s:session id %d: T mode encoder success\n",
				__func__, audio->ac->session);
	} else {
		pr_err("%s:session id %d: Unexpected mode\n", __func__,
				audio->ac->session);
		rc = -EACCES;
		goto fail;
	}

	audio->opened = 1;
	atomic_set(&audio->in_count, PCM_BUF_COUNT);
	atomic_set(&audio->out_count, 0x00);
	audio->enc_compat_ioctl = amrwb_in_compat_ioctl;
	audio->enc_ioctl = amrwb_in_ioctl;
	file->private_data = audio;

	pr_info("%s:session id %d: success\n", __func__, audio->ac->session);
	return 0;
fail:
	q6asm_audio_client_free(audio->ac);
	kfree(audio->enc_cfg);
	kfree(audio);
	return rc;
}

static const struct file_operations audio_in_fops = {
	.owner		= THIS_MODULE,
	.open		= amrwb_in_open,
	.release	= audio_in_release,
	.read		= audio_in_read,
	.write		= audio_in_write,
	.unlocked_ioctl	= audio_in_ioctl,
	.compat_ioctl   = audio_in_compat_ioctl
};

struct miscdevice audio_amrwb_in_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_amrwb_in",
	.fops	= &audio_in_fops,
};

int __init amrwb_in_init(void)
{
	return misc_register(&audio_amrwb_in_misc);
}

void amrwb_in_exit(void)
{
	misc_deregister(&audio_amrwb_in_misc);
}
