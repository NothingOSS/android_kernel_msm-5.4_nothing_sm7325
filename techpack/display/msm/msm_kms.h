/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MSM_KMS_H__
#define __MSM_KMS_H__

#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#include "msm_drv.h"

#define MAX_PLANE	4

/**
 * Device Private DRM Mode Flags
 * drm_mode->private_flags
 */
/* Connector has interpreted seamless transition request as dynamic fps */
#define MSM_MODE_FLAG_SEAMLESS_DYNAMIC_FPS	(1<<0)
/* Transition to new mode requires a wait-for-vblank before the modeset */
#define MSM_MODE_FLAG_VBLANK_PRE_MODESET	(1<<1)
/* Request to switch the connector mode */
#define MSM_MODE_FLAG_SEAMLESS_DMS			(1<<2)
/* Request to switch the fps */
#define MSM_MODE_FLAG_SEAMLESS_VRR			(1<<3)
/* Request to switch the panel mode */
#define MSM_MODE_FLAG_SEAMLESS_POMS			(1<<4)
/* Request to switch the bit clk */
#define MSM_MODE_FLAG_SEAMLESS_DYN_CLK			(1<<5)

/* As there are different display controller blocks depending on the
 * snapdragon version, the kms support is split out and the appropriate
 * implementation is loaded at runtime.  The kms module is responsible
 * for constructing the appropriate planes/crtcs/encoders/connectors.
 */
struct msm_kms_funcs {
	/* hw initialization: */
	int (*hw_init)(struct msm_kms *kms);
	int (*postinit)(struct msm_kms *kms);
	/* irq handling: */
	void (*irq_preinstall)(struct msm_kms *kms);
	int (*irq_postinstall)(struct msm_kms *kms);
	void (*irq_uninstall)(struct msm_kms *kms);
	irqreturn_t (*irq)(struct msm_kms *kms);
	int (*enable_vblank)(struct msm_kms *kms, struct drm_crtc *crtc);
	void (*disable_vblank)(struct msm_kms *kms, struct drm_crtc *crtc);
	/* modeset, bracketing atomic_commit(): */
	void (*prepare_fence)(struct msm_kms *kms,
			struct drm_atomic_state *state);
	void (*prepare_commit)(struct msm_kms *kms,
			struct drm_atomic_state *state);
	void (*commit)(struct msm_kms *kms, struct drm_atomic_state *state);
	void (*complete_commit)(struct msm_kms *kms,
			struct drm_atomic_state *state);
	/* functions to wait for atomic commit completed on each CRTC */
	void (*wait_for_crtc_commit_done)(struct msm_kms *kms,
					struct drm_crtc *crtc);
	/* function pointer to wait for pixel transfer to panel to complete*/
	void (*wait_for_tx_complete)(struct msm_kms *kms,
					struct drm_crtc *crtc);
	/* get msm_format w/ optional format modifiers from drm_mode_fb_cmd2 */
	const struct msm_format *(*get_format)(struct msm_kms *kms,
					const uint32_t format,
					const uint64_t modifier);
	/* do format checking on format modified through fb_cmd2 modifiers */
	int (*check_modified_format)(const struct msm_kms *kms,
			const struct msm_format *msm_fmt,
			const struct drm_mode_fb_cmd2 *cmd,
			struct drm_gem_object **bos);
	/* perform complete atomic check of given atomic state */
	int (*atomic_check)(struct msm_kms *kms,
			struct drm_atomic_state *state);
	/* misc: */
	long (*round_pixclk)(struct msm_kms *kms, unsigned long rate,
			struct drm_encoder *encoder);
	int (*set_split_display)(struct msm_kms *kms,
			struct drm_encoder *encoder,
			struct drm_encoder *slave_encoder,
			bool is_cmd_mode);
	void (*postopen)(struct msm_kms *kms, struct drm_file *file);
	void (*preclose)(struct msm_kms *kms, struct drm_file *file);
	void (*postclose)(struct msm_kms *kms, struct drm_file *file);
	void (*lastclose)(struct msm_kms *kms);
	int (*register_events)(struct msm_kms *kms,
			struct drm_mode_object *obj, u32 event, bool en);
	void (*set_encoder_mode)(struct msm_kms *kms,
				 struct drm_encoder *encoder,
				 bool cmd_mode);
	void (*display_early_wakeup)(struct drm_device *dev,
				const int32_t connector_id);
	/* pm suspend/resume hooks */
	int (*pm_suspend)(struct device *dev);
	int (*pm_resume)(struct device *dev);
	/* cleanup: */
	void (*destroy)(struct msm_kms *kms);
	/* get address space */
	struct msm_gem_address_space *(*get_address_space)(
			struct msm_kms *kms,
			unsigned int domain);
	struct device *(*get_address_space_device)(
			struct msm_kms *kms,
			unsigned int domain);
#ifdef CONFIG_DEBUG_FS
	/* debugfs: */
	int (*debugfs_init)(struct msm_kms *kms, struct drm_minor *minor);
#endif
	/* destroys debugfs */
	void (*debugfs_destroy)(struct msm_kms *kms);
	/* handle continuous splash  */
	int (*cont_splash_config)(struct msm_kms *kms,
			struct drm_atomic_state *state);
	/* check for continuous splash status */
	bool (*check_for_splash)(struct msm_kms *kms, struct drm_crtc *crtc);
	/* topology lm information */
	int (*get_mixer_count)(const struct msm_kms *kms,
			const struct drm_display_mode *mode,
			const struct msm_resource_caps_info *res, u32 *num_lm);
	/* topology dsc information */
	int (*get_dsc_count)(const struct msm_kms *kms,
			u32 hdisplay, u32 *num_dsc);
};

struct msm_kms {
	const struct msm_kms_funcs *funcs;

	/* irq number to be passed on to drm_irq_install */
	int irq;

	/* mapper-id used to request GEM buffer mapped for scanout: */
	struct msm_gem_address_space *aspace;

	/* DRM client used for lastclose cleanup */
	struct drm_client_dev client;
};

/**
 * Subclass of drm_atomic_state, to allow kms backend to have driver
 * private global state.  The kms backend can do whatever it wants
 * with the ->state ptr.  On ->atomic_state_clear() the ->state ptr
 * is kfree'd and set back to NULL.
 */
struct msm_kms_state {
	struct drm_atomic_state base;
	void *state;
};
#define to_kms_state(x) container_of(x, struct msm_kms_state, base)

static inline void msm_kms_init(struct msm_kms *kms,
		const struct msm_kms_funcs *funcs)
{
	kms->funcs = funcs;
}

#ifdef CONFIG_DRM_MSM_MDP4
struct msm_kms *mdp4_kms_init(struct drm_device *dev);
#else
static inline
struct msm_kms *mdp4_kms_init(struct drm_device *dev) { return NULL; };
#endif
#if IS_ENABLED(CONFIG_DRM_MSM_MDP5)
struct msm_kms *mdp5_kms_init(struct drm_device *dev);
int msm_mdss_init(struct drm_device *dev);
void msm_mdss_destroy(struct drm_device *dev);
struct msm_kms *mdp5_kms_init(struct drm_device *dev);
int msm_mdss_enable(struct msm_mdss *mdss);
int msm_mdss_disable(struct msm_mdss *mdss);
#else
static inline int msm_mdss_init(struct drm_device *dev)
{
	return 0;
}
static inline void msm_mdss_destroy(struct drm_device *dev)
{
}
static inline struct msm_kms *mdp5_kms_init(struct drm_device *dev)
{
	return NULL;
}
static inline int msm_mdss_enable(struct msm_mdss *mdss)
{
	return 0;
}
static inline int msm_mdss_disable(struct msm_mdss *mdss)
{
	return 0;
}
#endif /* CONFIG_DRM_MSM_MDP5 */

struct msm_kms *sde_kms_init(struct drm_device *dev);


/**
 * Mode Set Utility Functions
 */
static inline bool msm_is_mode_seamless(const struct drm_display_mode *mode)
{
	return (mode->flags & DRM_MODE_FLAG_SEAMLESS);
}

static inline bool msm_is_mode_seamless_dms(const struct drm_display_mode *mode)
{
	return mode ? (mode->private_flags & MSM_MODE_FLAG_SEAMLESS_DMS)
		: false;
}

static inline bool msm_is_mode_dynamic_fps(const struct drm_display_mode *mode)
{
	return ((mode->flags & DRM_MODE_FLAG_SEAMLESS) &&
		(mode->private_flags & MSM_MODE_FLAG_SEAMLESS_DYNAMIC_FPS));
}

static inline bool msm_is_mode_seamless_vrr(const struct drm_display_mode *mode)
{
	return mode ? (mode->private_flags & MSM_MODE_FLAG_SEAMLESS_VRR)
		: false;
}

static inline bool msm_is_mode_seamless_poms(
		const struct drm_display_mode *mode)
{
	return mode ? (mode->private_flags & MSM_MODE_FLAG_SEAMLESS_POMS)
		: false;
}

static inline bool msm_is_mode_seamless_dyn_clk(
					const struct drm_display_mode *mode)
{
	return mode ? (mode->private_flags & MSM_MODE_FLAG_SEAMLESS_DYN_CLK)
		: false;
}

static inline bool msm_needs_vblank_pre_modeset(
		const struct drm_display_mode *mode)
{
	return (mode->private_flags & MSM_MODE_FLAG_VBLANK_PRE_MODESET);
}
#endif /* __MSM_KMS_H__ */
