/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
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

#include <linux/dma-buf.h>
#include <linux/msm_ion.h>
#include <drm/drm_crtc.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_probe_helper.h>

#include "msm_drv.h"
#include "msm_kms.h"
#include "msm_gem.h"

struct msm_framebuffer {
	struct drm_framebuffer base;
	const struct msm_format *format;
};
#define to_msm_framebuffer(x) container_of(x, struct msm_framebuffer, base)

static const struct drm_framebuffer_funcs msm_framebuffer_funcs = {
	.create_handle = drm_gem_fb_create_handle,
	.destroy = drm_gem_fb_destroy,
	.dirty = drm_atomic_helper_dirtyfb,
};

#ifdef CONFIG_DEBUG_FS
void msm_framebuffer_describe(struct drm_framebuffer *fb, struct seq_file *m)
{
	struct msm_framebuffer *msm_fb;
	int i, n;

	if (!fb) {
		DRM_ERROR("from:%pS null fb\n", __builtin_return_address(0));
		return;
	}

	msm_fb = to_msm_framebuffer(fb);
	n = fb->format->num_planes;
	seq_printf(m, "fb: %dx%d@%4.4s (%2d, ID:%d)\n",
			fb->width, fb->height, (char *)&fb->format->format,
			drm_framebuffer_read_refcount(fb), fb->base.id);

	for (i = 0; i < n; i++) {
		seq_printf(m, "   %d: offset=%d pitch=%d, obj: ",
				i, fb->offsets[i], fb->pitches[i]);
		msm_gem_describe(fb->obj[i], m);
	}
}
#endif

void msm_framebuffer_set_keepattrs(struct drm_framebuffer *fb, bool enable)
{
	struct msm_framebuffer *msm_fb;
	int i, n;
	struct drm_gem_object *bo;
	struct msm_gem_object *msm_obj;

	if (!fb) {
		DRM_ERROR("from:%pS null fb\n", __builtin_return_address(0));
		return;
	}

	if (!fb->format) {
		DRM_ERROR("from:%pS null fb->format\n",
				__builtin_return_address(0));
		return;
	}

	msm_fb = to_msm_framebuffer(fb);
	n = fb->format->num_planes;
	for (i = 0; i < n; i++) {
		bo = msm_framebuffer_bo(fb, i);
		if (bo) {
			msm_obj = to_msm_bo(bo);
			if (enable)
				msm_obj->flags |= MSM_BO_KEEPATTRS;
			else
				msm_obj->flags &= ~MSM_BO_KEEPATTRS;
		}
	}
}

/* prepare/pin all the fb's bo's for scanout.  Note that it is not valid
 * to prepare an fb more multiple different initiator 'id's.  But that
 * should be fine, since only the scanout (mdpN) side of things needs
 * this, the gpu doesn't care about fb's.
 */
int msm_framebuffer_prepare(struct drm_framebuffer *fb,
		struct msm_gem_address_space *aspace)
{
	struct msm_framebuffer *msm_fb;
	int ret, i, n;
	uint64_t iova;

	if (!fb) {
		DRM_ERROR("from:%pS null fb\n", __builtin_return_address(0));
		return -EINVAL;
	}

	msm_fb = to_msm_framebuffer(fb);
	n = fb->format->num_planes;
	for (i = 0; i < n; i++) {
		ret = msm_gem_get_iova(fb->obj[i], aspace, &iova);
		DBG("FB[%u]: iova[%d]: %08llx (%d)", fb->base.id, i, iova, ret);
		if (ret)
			return ret;
	}

	return 0;
}

void msm_framebuffer_cleanup(struct drm_framebuffer *fb,
		struct msm_gem_address_space *aspace)
{
	struct msm_framebuffer *msm_fb;
	int i, n;

	if (fb == NULL) {
		DRM_ERROR("from:%pS null fb\n", __builtin_return_address(0));
		return;
	}

	msm_fb = to_msm_framebuffer(fb);
	n = fb->format->num_planes;

	for (i = 0; i < n; i++)
		msm_gem_put_iova(fb->obj[i], aspace);
}

uint32_t msm_framebuffer_iova(struct drm_framebuffer *fb,
		struct msm_gem_address_space *aspace, int plane)
{

	if (!fb) {
		DRM_ERROR("from:%pS null fb\n", __builtin_return_address(0));
		return -EINVAL;
	}

	if (!fb->obj[plane])
		return 0;

	return msm_gem_iova(fb->obj[plane], aspace) + fb->offsets[plane];
}

uint32_t msm_framebuffer_phys(struct drm_framebuffer *fb,
		int plane)
{
	struct msm_framebuffer *msm_fb;
	dma_addr_t phys_addr;

	if (!fb) {
		DRM_ERROR("from:%pS null fb\n", __builtin_return_address(0));
		return -EINVAL;
	}

	msm_fb = to_msm_framebuffer(fb);

	if (!msm_fb->base.obj[plane])
		return 0;

	phys_addr = msm_gem_get_dma_addr(msm_fb->base.obj[plane]);
	if (!phys_addr)
		return 0;

	return phys_addr + fb->offsets[plane];
}

struct drm_gem_object *msm_framebuffer_bo(struct drm_framebuffer *fb, int plane)
{
	if (!fb) {
		DRM_ERROR("from:%pS null fb\n", __builtin_return_address(0));
		return ERR_PTR(-EINVAL);
	}

	return drm_gem_fb_get_obj(fb, plane);
}

const struct msm_format *msm_framebuffer_format(struct drm_framebuffer *fb)
{
	return fb ? (to_msm_framebuffer(fb))->format : NULL;
}

struct drm_framebuffer *msm_framebuffer_create(struct drm_device *dev,
		struct drm_file *file, const struct drm_mode_fb_cmd2 *mode_cmd)
{
	const struct drm_format_info *info = drm_get_format_info(dev,
								mode_cmd);
	struct drm_gem_object *bos[4] = {0};
	struct drm_framebuffer *fb;
	int ret, i, n = info->num_planes;

	for (i = 0; i < n; i++) {
		bos[i] = drm_gem_object_lookup(file, mode_cmd->handles[i]);
		if (!bos[i]) {
			ret = -ENXIO;
			goto out_unref;
		}
	}

	fb = msm_framebuffer_init(dev, mode_cmd, bos);
	if (IS_ERR(fb)) {
		ret = PTR_ERR(fb);
		goto out_unref;
	}

	return fb;

out_unref:
	for (i = 0; i < n; i++)
		drm_gem_object_put_unlocked(bos[i]);
	return ERR_PTR(ret);
}

struct drm_framebuffer *msm_framebuffer_init(struct drm_device *dev,
		const struct drm_mode_fb_cmd2 *mode_cmd,
		struct drm_gem_object **bos)
{
	const struct drm_format_info *info = drm_get_format_info(dev,
								mode_cmd);
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	struct msm_framebuffer *msm_fb = NULL;
	struct drm_framebuffer *fb;
	const struct msm_format *format;
	int ret, i, num_planes;
	bool is_modified = false;

	DBG("create framebuffer: dev=%pK, mode_cmd=%pK (%dx%d@%4.4s)",
			dev, mode_cmd, mode_cmd->width, mode_cmd->height,
			(char *)&mode_cmd->pixel_format);

	num_planes = info->num_planes;

	format = kms->funcs->get_format(kms, mode_cmd->pixel_format,
			mode_cmd->modifier[0]);
	if (!format) {
		dev_err(dev->dev, "unsupported pixel format: %4.4s\n",
				(char *)&mode_cmd->pixel_format);
		ret = -EINVAL;
		goto fail;
	}

	msm_fb = kzalloc(sizeof(*msm_fb), GFP_KERNEL);
	if (!msm_fb) {
		ret = -ENOMEM;
		goto fail;
	}

	fb = &msm_fb->base;

	msm_fb->format = format;

	if (mode_cmd->flags & DRM_MODE_FB_MODIFIERS) {
		for (i = 0; i < ARRAY_SIZE(mode_cmd->modifier); i++) {
			if (mode_cmd->modifier[i]) {
				is_modified = true;
				break;
			}
		}
	}

	if (num_planes > ARRAY_SIZE(fb->obj)) {
		ret = -EINVAL;
		goto fail;
	}

	if (is_modified) {
		if (!kms->funcs->check_modified_format) {
			dev_err(dev->dev, "can't check modified fb format\n");
			ret = -EINVAL;
			goto fail;
		} else {
			ret = kms->funcs->check_modified_format(
					kms, msm_fb->format, mode_cmd, bos);
			if (ret)
				goto fail;
		}
	} else {
		const struct drm_format_info *info;

		info = drm_format_info(mode_cmd->pixel_format);
		if (!info || num_planes > ARRAY_SIZE(info->cpp)) {
			ret = -EINVAL;
			goto fail;
		}

		for (i = 0; i < num_planes; i++) {
			unsigned int width = mode_cmd->width / (i ?
					info->hsub : 1);
			unsigned int height = mode_cmd->height / (i ?
					info->vsub : 1);
			unsigned int min_size;

			min_size = (height - 1) * mode_cmd->pitches[i]
				+ width * info->cpp[i]
				+ mode_cmd->offsets[i];

			if (!bos[i] || bos[i]->size < min_size) {
				ret = -EINVAL;
				goto fail;
			}
		}
	}

	for (i = 0; i < num_planes; i++)
		msm_fb->base.obj[i] = bos[i];

	drm_helper_mode_fill_fb_struct(dev, fb, mode_cmd);

	ret = drm_framebuffer_init(dev, fb, &msm_framebuffer_funcs);
	if (ret) {
		dev_err(dev->dev, "framebuffer init failed: %d\n", ret);
		goto fail;
	}

	DBG("create: FB ID: %d (%pK)", fb->base.id, fb);

	return fb;

fail:
	kfree(msm_fb);

	return ERR_PTR(ret);
}

struct drm_framebuffer *
msm_alloc_stolen_fb(struct drm_device *dev, int w, int h, int p, uint32_t format)
{
	struct drm_mode_fb_cmd2 mode_cmd = {
		.pixel_format = format,
		.width = w,
		.height = h,
		.pitches = { p },
	};
	struct drm_gem_object *bo;
	struct drm_framebuffer *fb;
	int size;

	/* allocate backing bo */
	size = mode_cmd.pitches[0] * mode_cmd.height;
	DBG("allocating %d bytes for fb %d", size, dev->primary->index);
	bo = msm_gem_new(dev, size, MSM_BO_SCANOUT | MSM_BO_WC | MSM_BO_STOLEN);
	if (IS_ERR(bo)) {
		dev_warn(dev->dev, "could not allocate stolen bo\n");
		/* try regular bo: */
		bo = msm_gem_new(dev, size, MSM_BO_SCANOUT | MSM_BO_WC);
	}
	if (IS_ERR(bo)) {
		dev_err(dev->dev, "failed to allocate buffer object\n");
		return ERR_CAST(bo);
	}

	msm_gem_object_set_name(bo, "stolenfb");

	fb = msm_framebuffer_init(dev, &mode_cmd, &bo);
	if (IS_ERR(fb)) {
		dev_err(dev->dev, "failed to allocate fb\n");
		/* note: if fb creation failed, we can't rely on fb destroy
		 * to unref the bo:
		 */
		drm_gem_object_put_unlocked(bo);
		return ERR_CAST(fb);
	}

	return fb;
}

int msm_fb_obj_get_attrs(struct drm_gem_object *obj, int *fb_ns,
	 int *fb_sec, int *fb_sec_dir, unsigned long *flags)
{

	struct msm_gem_object *msm_obj = to_msm_bo(obj);
	int ret = 0;

	if (!obj->import_attach) {
		DRM_DEBUG("NULL attachment in drm gem object flags:0x%x\n", msm_obj->flags);
		return -EINVAL;
	}

	ret = dma_buf_get_flags(obj->import_attach->dmabuf, flags);
	if (ret) {
		DRM_ERROR("dma_buf_get_flags failure, err=%d\n", ret);
		return ret;
	}

	if (!(*flags & ION_FLAG_SECURE))
		*fb_ns = 1;
	else if (*flags & ION_FLAG_CP_PIXEL)
		*fb_sec = 1;
	else if (*flags & (ION_FLAG_CP_SEC_DISPLAY |
			ION_FLAG_CP_CAMERA_PREVIEW))
		*fb_sec_dir = 1;

	return ret;
}
