/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
 */
#ifndef __WCD934X_MBHC_H__
#define __WCD934X_MBHC_H__
#include <asoc/wcd-mbhc-v2.h>

enum wcd934x_on_demand_supply_name {
	WCD934X_ON_DEMAND_MICBIAS = 0,
	WCD934X_ON_DEMAND_SUPPLIES_MAX,
};

struct wcd934x_on_demand_supply {
	struct regulator *supply;
	int ondemand_supply_count;
};

struct wcd934x_mbhc {
	struct wcd_mbhc wcd_mbhc;
	struct blocking_notifier_head notifier;
	struct wcd934x_on_demand_supply on_demand_list[
			WCD934X_ON_DEMAND_SUPPLIES_MAX];
	struct wcd9xxx *wcd9xxx;
	struct fw_info *fw_data;
	bool mbhc_started;
	bool is_hph_recover;
};

#if IS_ENABLED(CONFIG_SND_SOC_WCD934X_MBHC)
extern int tavil_mbhc_init(struct wcd934x_mbhc **mbhc,
			   struct snd_soc_component *component,
			   struct fw_info *fw_data);
extern void tavil_mbhc_hs_detect_exit(struct snd_soc_component *component);
extern int tavil_mbhc_hs_detect(struct snd_soc_component *component,
				struct wcd_mbhc_config *mbhc_cfg);
extern void tavil_mbhc_deinit(struct snd_soc_component *component);
extern int tavil_mbhc_post_ssr_init(struct wcd934x_mbhc *mbhc,
				    struct snd_soc_component *component);
extern int tavil_mbhc_get_impedance(struct wcd934x_mbhc *wcd934x_mbhc,
				    uint32_t *zl, uint32_t *zr);
#else
static inline int tavil_mbhc_init(struct wcd934x_mbhc **mbhc,
				  struct snd_soc_component *component,
				  struct fw_info *fw_data)
{
	return 0;
}
static inline void tavil_mbhc_hs_detect_exit(
			struct snd_soc_component *component)
{
}
static inline int tavil_mbhc_hs_detect(struct snd_soc_component *component,
				       struct wcd_mbhc_config *mbhc_cfg)
{
		return 0;
}
static inline void tavil_mbhc_deinit(struct snd_soc_component *component)
{
}
static inline int tavil_mbhc_post_ssr_init(struct wcd934x_mbhc *mbhc,
					   struct snd_soc_component *component)
{
	return 0;
}
static inline int tavil_mbhc_get_impedance(struct wcd934x_mbhc *wcd934x_mbhc,
					   uint32_t *zl, uint32_t *zr)
{
	if (zl)
		*zl = 0;
	if (zr)
		*zr = 0;
	return -EINVAL;
}
#endif

#endif /* __WCD934X_MBHC_H__ */
