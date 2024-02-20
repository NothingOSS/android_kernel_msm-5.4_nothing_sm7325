/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2013-2016, 2020 The Linux Foundation. All rights reserved.
 */

#ifndef _MSM_AUDIO_EFFECTS_H
#define _MSM_AUDIO_EFFECTS_H

#include <audio/sound/audio_effects.h>

#define MAX_PP_PARAMS_SZ   128

bool msm_audio_effects_is_effmodule_supp_in_top(int effect_module,
						int topology);

int msm_audio_effects_enable_extn(struct audio_client *ac,
			     struct msm_nt_eff_all_config *effects,
			     bool flag);

int msm_audio_effects_reverb_handler(struct audio_client *ac,
				     struct reverb_params *reverb,
				     long *values);

int msm_audio_effects_bass_boost_handler(struct audio_client *ac,
					struct bass_boost_params *bass_boost,
					long *values);

int msm_audio_effects_pbe_handler(struct audio_client *ac,
					struct pbe_params *pbe,
					long *values);

int msm_audio_effects_virtualizer_handler(struct audio_client *ac,
				struct virtualizer_params *virtualizer,
				long *values);

int msm_audio_effects_popless_eq_handler(struct audio_client *ac,
					 struct eq_params *eq,
					 long *values);

int msm_audio_effects_volume_handler(struct audio_client *ac,
				     struct soft_volume_params *vol,
				     long *values);

int msm_audio_effects_volume_handler_v2(struct audio_client *ac,
					struct soft_volume_params *vol,
					long *values, int instance);
#endif /*_MSM_AUDIO_EFFECTS_H*/
