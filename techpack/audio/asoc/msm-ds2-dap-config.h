/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2013-2014, 2017, 2019 The Linux Foundation. All rights reserved.
 */

#ifndef _MSM_DS2_DAP_CONFIG_H_
#define _MSM_DS2_DAP_CONFIG_H_

#include <sound/soc.h>
#include "msm-dolby-common.h"
#include <sound/hwdep.h>
#include <uapi/audio/sound/devdep_params.h>

#ifdef CONFIG_COMPAT
struct dolby_param_data32 {
	s32 version;
	s32 device_id;
	s32 be_id;
	s32 param_id;
	s32 length;
	compat_uptr_t data;
};

struct dolby_param_license32 {
	compat_uptr_t dmid;
	compat_uptr_t license_key;
};

#define SNDRV_DEVDEP_DAP_IOCTL_SET_PARAM32\
		_IOWR('U', 0x10, struct dolby_param_data32)
#define SNDRV_DEVDEP_DAP_IOCTL_GET_PARAM32\
		_IOR('U', 0x11, struct dolby_param_data32)
#define SNDRV_DEVDEP_DAP_IOCTL_DAP_COMMAND32\
		_IOWR('U', 0x13, struct dolby_param_data32)
#define SNDRV_DEVDEP_DAP_IOCTL_DAP_LICENSE32\
		_IOWR('U', 0x14, struct dolby_param_license32)
#define SNDRV_DEVDEP_DAP_IOCTL_GET_VISUALIZER32\
		_IOR('U', 0x15, struct dolby_param_data32)
#endif

#if defined(CONFIG_DOLBY_DS2) || defined(CONFIG_DOLBY_LICENSE)
/* DOLBY DOLBY GUIDS */
#define DS2_MODULE_ID			0x00010775

#define DS2_DSP_SUPPORTED_ENDP_DEVICE		17
#define DS2_DEVICES_ALL				32 /* enum val is 4 bytes */

enum {

	DAP_CMD_COMMIT_ALL         = 0,
	DAP_CMD_COMMIT_CHANGED     = 1,
	DAP_CMD_USE_CACHE_FOR_INIT = 2,
	DAP_CMD_SET_BYPASS         = 3,
	DAP_CMD_SET_ACTIVE_DEVICE  = 4,
	DAP_CMD_SET_BYPASS_TYPE    = 5,
};

struct custom_stereo_param {
	/* Index is 32-bit param in little endian */
	u16 index;
	u16 reserved;

	/* For stereo mixing, the number of out channels */
	u16 num_out_ch;
	/* For stereo mixing, the number of in channels */
	u16 num_in_ch;

	/* Out channel map FL/FR*/
	u16 out_fl;
	u16 out_fr;

	/* In channel map FL/FR*/
	u16 in_fl;
	u16 in_fr;

	/*
	 * Weighting coefficients. Mixing will be done according to
	 * these coefficients.
	 */
	u16 op_FL_ip_FL_weight;
	u16 op_FL_ip_FR_weight;
	u16 op_FR_ip_FL_weight;
	u16 op_FR_ip_FR_weight;
};

#define DOLBY_PARAM_INT_ENDP_LENGTH             1
#define DOLBY_PARAM_INT_ENDP_OFFSET		(DOLBY_PARAM_PSTG_OFFSET + \
							DOLBY_PARAM_PSTG_LENGTH)
#define MAX_DS2_PARAMS				48
#define MAX_DS2_CTRL_PARAMS			4
#define ALL_DS2_PARAMS				(MAX_DS2_PARAMS + \
							MAX_DS2_CTRL_PARAMS)
#define TOTAL_LENGTH_DS2_PARAM (TOTAL_LENGTH_DOLBY_PARAM + 1)

int msm_ds2_dap_update_port_parameters(struct snd_hwdep *hw,  struct file *file,
				       bool open);
int msm_ds2_dap_ioctl(struct snd_hwdep *hw, struct file *file,
		      u32 cmd, void *arg);
int msm_ds2_dap_compat_ioctl(struct snd_hwdep *hw,
			     struct file *file,
			     u32 cmd, void *arg);
int msm_ds2_dap_init(int port_id, int copp_idx, int channels,
		     bool is_custom_stereo_on);
void msm_ds2_dap_deinit(int port_id);
int msm_ds2_dap_set_custom_stereo_onoff(int port_id, int copp_idx,
					bool is_custom_stereo_enabled);
/* Dolby DOLBY end */
#else

static inline int msm_ds2_dap_update_port_parameters(struct snd_hwdep *hw,
					       struct file *file,
					       bool open)
{
	return 0;
}

static inline int msm_ds2_dap_ioctl(struct snd_hwdep *hw, struct file *file,
				    u32 cmd, void *arg)
{
	return 0;
}

static inline int msm_ds2_dap_compat_ioctl(struct snd_hwdep *hw,
					   struct file *file,
					   u32 cmd, void *arg)
{
	return 0;
}
static inline int msm_ds2_dap_init(int port_id, int copp_idx, int channels,
		     bool is_custom_stereo_on)
{
	return 0;
}

static inline void msm_ds2_dap_deinit(int port_id) { }

static inline int msm_ds2_dap_set_custom_stereo_onoff(int port_id, int copp_idx,
				    bool is_custom_stereo_enabled)
{
	return 0;
}
#endif
#endif
