/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
 */

#ifndef __WCD934X_DSD_H__
#define __WCD934X_DSD_H__

#include <sound/soc.h>
#include "wcd934x.h"

enum {
	DSD0,
	DSD1,
	DSD_MAX,
};

enum {
	DSD_INP_SEL_ZERO = 0,
	DSD_INP_SEL_RX0,
	DSD_INP_SEL_RX1,
	DSD_INP_SEL_RX2,
	DSD_INP_SEL_RX3,
	DSD_INP_SEL_RX4,
	DSD_INP_SEL_RX5,
	DSD_INP_SEL_RX6,
	DSD_INP_SEL_RX7,
};

struct tavil_dsd_config {
	struct snd_soc_component *component;
	unsigned int dsd_interp_mixer[INTERP_MAX];
	u32 base_sample_rate[DSD_MAX];
	int volume[DSD_MAX];
	struct mutex vol_mutex;
	int version;
};

#if IS_ENABLED(CONFIG_SND_SOC_WCD934X_DSD)
int tavil_dsd_set_mixer_value(struct tavil_dsd_config *dsd_conf,
			      int interp_num, int sw_value);
int tavil_dsd_get_current_mixer_value(struct tavil_dsd_config *dsd_conf,
				      int interp_num);
int tavil_dsd_set_out_select(struct tavil_dsd_config *dsd_conf,
			     int interp_num);
void tavil_dsd_reset(struct tavil_dsd_config *dsd_conf);
void tavil_dsd_set_interp_rate(struct tavil_dsd_config *dsd_conf, u16 rx_port,
			       u32 sample_rate, u8 sample_rate_val);
struct tavil_dsd_config *tavil_dsd_init(struct snd_soc_component *component);
void tavil_dsd_deinit(struct tavil_dsd_config *dsd_config);
int tavil_dsd_post_ssr_init(struct tavil_dsd_config *dsd_config);
#else
int tavil_dsd_set_mixer_value(struct tavil_dsd_config *dsd_conf,
			      int interp_num, int sw_value)
{
	return 0;
}

int tavil_dsd_get_current_mixer_value(struct tavil_dsd_config *dsd_conf,
				      int interp_num)
{
	return 0;
}

int tavil_dsd_set_out_select(struct tavil_dsd_config *dsd_conf,
			     int interp_num)
{
	return 0;
}

void tavil_dsd_reset(struct tavil_dsd_config *dsd_conf)
{  }

void tavil_dsd_set_interp_rate(struct tavil_dsd_config *dsd_conf, u16 rx_port,
			       u32 sample_rate, u8 sample_rate_val)
{  }

struct tavil_dsd_config *tavil_dsd_init(struct snd_soc_component *component)
{
	return NULL;
}

void tavil_dsd_deinit(struct tavil_dsd_config *dsd_config)
{  }
int tavil_dsd_post_ssr_init(struct tavil_dsd_config *dsd_config)
{
	return 0;
}
#endif
#endif
