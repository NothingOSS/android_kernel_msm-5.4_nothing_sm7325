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
/*
 * Copyright (c) 2016 Intel Corporation
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission. The copyright holders make no representations
 * about the suitability of this software for any purpose. It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#include <linux/of_address.h>
#include <linux/kthread.h>
#include <uapi/linux/sched/types.h>
#include <drm/drm_of.h>
#include <drm/drm_auth.h>
#include <drm/drm_probe_helper.h>

#include "msm_drv.h"
#include "msm_gem.h"
#include "msm_kms.h"
#include "msm_mmu.h"
#include "sde_wb.h"
#include "sde_dbg.h"

/*
 * MSM driver version:
 * - 1.0.0 - initial interface
 * - 1.1.0 - adds madvise, and support for submits with > 4 cmd buffers
 * - 1.2.0 - adds explicit fence support for submit ioctl
 * - 1.3.0 - adds GMEM_BASE + NR_RINGS params, SUBMITQUEUE_NEW +
 *           SUBMITQUEUE_CLOSE ioctls, and MSM_INFO_IOVA flag for
 *           MSM_GEM_INFO ioctl.
 * - 1.4.0 - softpin, MSM_RELOC_BO_DUMP, and GEM_INFO support to set/get
 *           GEM object's debug name
 */
#define MSM_VERSION_MAJOR	1
#define MSM_VERSION_MINOR	4
#define MSM_VERSION_PATCHLEVEL	0

#define LASTCLOSE_TIMEOUT_MS	500

#define msm_wait_event_timeout(waitq, cond, timeout_ms, ret)		\
	do {								\
		ktime_t cur_ktime;					\
		ktime_t exp_ktime;					\
		s64 wait_time_jiffies = msecs_to_jiffies(timeout_ms);	\
\
		exp_ktime = ktime_add_ms(ktime_get(), timeout_ms);	\
		do {							\
			ret = wait_event_timeout(waitq, cond,		\
					wait_time_jiffies);		\
			cur_ktime = ktime_get();			\
		} while ((!cond) && (ret == 0) &&			\
			(ktime_compare_safe(exp_ktime, cur_ktime) > 0));\
	} while (0)

static DEFINE_MUTEX(msm_release_lock);

static void msm_fb_output_poll_changed(struct drm_device *dev)
{
	struct msm_drm_private *priv = NULL;

	if (!dev) {
		DRM_ERROR("output_poll_changed failed, invalid input\n");
		return;
	}

	priv = dev->dev_private;

	if (priv->fbdev)
		drm_fb_helper_hotplug_event(priv->fbdev);
}

/**
 * msm_atomic_helper_check - validate state object
 * @dev: DRM device
 * @state: the driver state object
 *
 * This is a wrapper for the drm_atomic_helper_check to check the modeset
 * and state checking for planes. Additionally it checks if any secure
 * transition(moving CRTC and planes between secure and non-secure states and
 * vice versa) is allowed or not. When going to secure state, planes
 * with fb_mode as dir translated only can be staged on the CRTC, and only one
 * CRTC should be active.
 * Also mixing of secure and non-secure is not allowed.
 *
 * RETURNS
 * Zero for success or -errorno.
 */
int msm_atomic_check(struct drm_device *dev,
			    struct drm_atomic_state *state)
{
	struct msm_drm_private *priv;

	priv = dev->dev_private;
	if (priv && priv->kms && priv->kms->funcs &&
			priv->kms->funcs->atomic_check)
		return priv->kms->funcs->atomic_check(priv->kms, state);

	return drm_atomic_helper_check(dev, state);
}

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = msm_framebuffer_create,
	.output_poll_changed = msm_fb_output_poll_changed,
	.atomic_check = msm_atomic_check,
	.atomic_commit = msm_atomic_commit,
	.atomic_state_alloc = msm_atomic_state_alloc,
	.atomic_state_clear = msm_atomic_state_clear,
	.atomic_state_free = msm_atomic_state_free,
};

static const struct drm_mode_config_helper_funcs mode_config_helper_funcs = {
	.atomic_commit_tail = msm_atomic_commit_tail,
};

#ifdef CONFIG_DRM_MSM_REGISTER_LOGGING
static bool reglog = false;
MODULE_PARM_DESC(reglog, "Enable register read/write logging");
module_param(reglog, bool, 0600);
#else
#define reglog 0
#endif

#ifdef CONFIG_DRM_FBDEV_EMULATION
static bool fbdev = true;
MODULE_PARM_DESC(fbdev, "Enable fbdev compat layer");
module_param(fbdev, bool, 0600);
#endif

static char *vram = "16m";
MODULE_PARM_DESC(vram, "Configure VRAM size (for devices without IOMMU/GPUMMU)");
module_param(vram, charp, 0);

bool dumpstate = false;
MODULE_PARM_DESC(dumpstate, "Dump KMS state on errors");
module_param(dumpstate, bool, 0600);

static bool modeset = true;
MODULE_PARM_DESC(modeset, "Use kernel modesetting [KMS] (1=on (default), 0=disable)");
module_param(modeset, bool, 0600);

/*
 * Util/helpers:
 */

int msm_clk_bulk_get(struct device *dev, struct clk_bulk_data **bulk)
{
	struct property *prop;
	const char *name;
	struct clk_bulk_data *local;
	int i = 0, ret, count;

	count = of_property_count_strings(dev->of_node, "clock-names");
	if (count < 1)
		return 0;

	local = devm_kcalloc(dev, sizeof(struct clk_bulk_data *),
		count, GFP_KERNEL);
	if (!local)
		return -ENOMEM;

	of_property_for_each_string(dev->of_node, "clock-names", prop, name) {
		local[i].id = devm_kstrdup(dev, name, GFP_KERNEL);
		if (!local[i].id) {
			devm_kfree(dev, local);
			return -ENOMEM;
		}

		i++;
	}

	ret = devm_clk_bulk_get(dev, count, local);

	if (ret) {
		for (i = 0; i < count; i++)
			devm_kfree(dev, (void *) local[i].id);
		devm_kfree(dev, local);

		return ret;
	}

	*bulk = local;
	return count;
}

struct clk *msm_clk_bulk_get_clock(struct clk_bulk_data *bulk, int count,
		const char *name)
{
	int i;
	char n[32];

	snprintf(n, sizeof(n), "%s_clk", name);

	for (i = 0; bulk && i < count; i++) {
		if (!strcmp(bulk[i].id, name) || !strcmp(bulk[i].id, n))
			return bulk[i].clk;
	}


	return NULL;
}

struct clk *msm_clk_get(struct platform_device *pdev, const char *name)
{
	struct clk *clk;
	char name2[32];

	clk = devm_clk_get(&pdev->dev, name);
	if (!IS_ERR(clk) || PTR_ERR(clk) == -EPROBE_DEFER)
		return clk;

	snprintf(name2, sizeof(name2), "%s_clk", name);

	clk = devm_clk_get(&pdev->dev, name2);
	if (!IS_ERR(clk))
		dev_warn(&pdev->dev, "Using legacy clk name binding.  Use "
				"\"%s\" instead of \"%s\"\n", name, name2);

	return clk;
}

void __iomem *msm_ioremap(struct platform_device *pdev, const char *name,
		const char *dbgname)
{
	struct resource *res;
	unsigned long size;
	void __iomem *ptr;

	if (name)
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	else
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		dev_dbg(&pdev->dev, "failed to get memory resource: %s\n",
									name);
		return ERR_PTR(-EINVAL);
	}

	size = resource_size(res);

	ptr = devm_ioremap_nocache(&pdev->dev, res->start, size);
	if (!ptr) {
		dev_err(&pdev->dev, "failed to ioremap: %s\n", name);
		return ERR_PTR(-ENOMEM);
	}

	if (reglog)
		dev_dbg(&pdev->dev, "IO:region %s %pK %08lx\n",
			dbgname, ptr, size);

	return ptr;
}

unsigned long msm_iomap_size(struct platform_device *pdev, const char *name)
{
	struct resource *res;

	if (name)
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	else
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		dev_dbg(&pdev->dev, "failed to get memory resource: %s\n",
									name);
		return 0;
	}

	return resource_size(res);
}

void msm_iounmap(struct platform_device *pdev, void __iomem *addr)
{
	devm_iounmap(&pdev->dev, addr);
}

void msm_writel(u32 data, void __iomem *addr)
{
	if (reglog)
		pr_debug("IO:W %pK %08x\n", addr, data);
	writel(data, addr);
}

u32 msm_readl(const void __iomem *addr)
{
	u32 val = readl(addr);

	if (reglog)
		pr_err("IO:R %pK %08x\n", addr, val);
	return val;
}

int msm_get_src_bpc(int chroma_format,
	int bpc)
{
	int src_bpp;

	switch (chroma_format) {
	case MSM_CHROMA_444:
		src_bpp = bpc * 3;
		break;
	case MSM_CHROMA_422:
		src_bpp = bpc * 2;
		break;
	case MSM_CHROMA_420:
		src_bpp = mult_frac(bpc, 3, 2);
		break;
	default:
		src_bpp = bpc * 3;
		break;
	}
	return src_bpp;
}

struct vblank_work {
	struct kthread_work work;
	int crtc_id;
	bool enable;
	struct msm_drm_private *priv;
};

static void vblank_ctrl_worker(struct kthread_work *work)
{
	struct vblank_work *cur_work = container_of(work,
					struct vblank_work, work);
	struct msm_drm_private *priv = cur_work->priv;
	struct msm_kms *kms = priv->kms;

	if (cur_work->enable)
		kms->funcs->enable_vblank(kms, priv->crtcs[cur_work->crtc_id]);
	else
		kms->funcs->disable_vblank(kms, priv->crtcs[cur_work->crtc_id]);

	kfree(cur_work);
}

static int vblank_ctrl_queue_work(struct msm_drm_private *priv,
					int crtc_id, bool enable)
{
	struct vblank_work *cur_work;
	struct kthread_worker *worker;

	if (!priv || crtc_id >= priv->num_crtcs)
		return -EINVAL;

	cur_work = kzalloc(sizeof(*cur_work), GFP_ATOMIC);
	if (!cur_work)
		return -ENOMEM;

	kthread_init_work(&cur_work->work, vblank_ctrl_worker);
	cur_work->crtc_id = crtc_id;
	cur_work->enable = enable;
	cur_work->priv = priv;
	worker = &priv->event_thread[crtc_id].worker;

	kthread_queue_work(worker, &cur_work->work);
	return 0;
}

static int msm_drm_uninit(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *ddev = platform_get_drvdata(pdev);
	struct msm_drm_private *priv = ddev->dev_private;
	struct msm_kms *kms = priv->kms;
	struct msm_vm_client_entry *client_entry, *tmp;
	int i;

	flush_workqueue(priv->wq);
	pm_runtime_get_sync(dev);

	/* clean up display commit/event worker threads */
	for (i = 0; i < priv->num_crtcs; i++) {
		if (priv->disp_thread[i].thread) {
			kthread_flush_worker(&priv->disp_thread[i].worker);
			kthread_stop(priv->disp_thread[i].thread);
			priv->disp_thread[i].thread = NULL;
		}

		if (priv->event_thread[i].thread) {
			kthread_flush_worker(&priv->event_thread[i].worker);
			kthread_stop(priv->event_thread[i].thread);
			priv->event_thread[i].thread = NULL;
		}
	}

	drm_kms_helper_poll_fini(ddev);
	if (kms && kms->funcs)
		kms->funcs->debugfs_destroy(kms);

	sde_dbg_destroy();
	debugfs_remove_recursive(priv->debug_root);
	drm_mode_config_cleanup(ddev);

	if (priv->registered) {
		drm_dev_unregister(ddev);
		priv->registered = false;
	}

#ifdef CONFIG_DRM_FBDEV_EMULATION
	if (fbdev && priv->fbdev)
		msm_fbdev_free(ddev);
#endif
	drm_atomic_helper_shutdown(ddev);
	drm_irq_uninstall(ddev);

	if (kms && kms->funcs)
		kms->funcs->destroy(kms);

	if (priv->vram.paddr) {
		unsigned long attrs = DMA_ATTR_NO_KERNEL_MAPPING;
		drm_mm_takedown(&priv->vram.mm);
		dma_free_attrs(dev, priv->vram.size, NULL,
			       priv->vram.paddr, attrs);
	}

	component_unbind_all(dev, ddev);
	pm_runtime_put_sync(dev);

	sde_power_resource_deinit(pdev, &priv->phandle);

	mutex_lock(&priv->vm_client_lock);

	/* clean up any unregistered clients */
	list_for_each_entry_safe(client_entry, tmp, &priv->vm_client_list,
				 list) {
		list_del(&client_entry->list);
		kfree(client_entry);
	}

	mutex_unlock(&priv->vm_client_lock);

	msm_mdss_destroy(ddev);

	ddev->dev_private = NULL;
	destroy_workqueue(priv->wq);
	kfree(priv);

	drm_dev_put(ddev);

	return 0;
}

#define KMS_MDP4 4
#define KMS_MDP5 5
#define KMS_SDE  3

static int get_mdp_ver(struct platform_device *pdev)
{
#ifdef CONFIG_OF
	static const struct of_device_id match_types[] = { {
		.compatible = "qcom,mdss_mdp",
		.data	= (void	*)KMS_MDP5,
	},
	{
		.compatible = "qcom,sde-kms",
		.data	= (void	*)KMS_SDE,
	},
	{},
	};
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;

	match = of_match_node(match_types, dev->of_node);
	if (match)
		return (int)(unsigned long)match->data;
#endif
	return KMS_MDP4;
}

static int msm_init_vram(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct device_node *node;
	unsigned long size = 0;
	int ret = 0;

	/* In the device-tree world, we could have a 'memory-region'
	 * phandle, which gives us a link to our "vram".  Allocating
	 * is all nicely abstracted behind the dma api, but we need
	 * to know the entire size to allocate it all in one go. There
	 * are two cases:
	 *  1) device with no IOMMU, in which case we need exclusive
	 *     access to a VRAM carveout big enough for all gpu
	 *     buffers
	 *  2) device with IOMMU, but where the bootloader puts up
	 *     a splash screen.  In this case, the VRAM carveout
	 *     need only be large enough for fbdev fb.  But we need
	 *     exclusive access to the buffer to avoid the kernel
	 *     using those pages for other purposes (which appears
	 *     as corruption on screen before we have a chance to
	 *     load and do initial modeset)
	 */

	node = of_parse_phandle(dev->dev->of_node, "memory-region", 0);
	if (node) {
		struct resource r;
		ret = of_address_to_resource(node, 0, &r);
		of_node_put(node);
		if (ret)
			return ret;
		size = r.end - r.start;
		DRM_INFO("using VRAM carveout: %lx@%pa\n", size, &r.start);

		/* if we have no IOMMU, then we need to use carveout allocator.
		 * Grab the entire CMA chunk carved out in early startup in
		 * mach-msm:
		 */
	} else if (!iommu_present(&platform_bus_type)) {
		u32 vram_size;

		ret = of_property_read_u32(dev->dev->of_node,
					"qcom,vram-size", &vram_size);
		size = (ret < 0) ? memparse(vram, NULL) : vram_size;
		DRM_INFO("using 0x%x VRAM carveout\n", size);
		ret = 0;
	}

	if (size) {
		unsigned long attrs = 0;
		void *p;

		priv->vram.size = size;

		drm_mm_init(&priv->vram.mm, 0, (size >> PAGE_SHIFT) - 1);
		spin_lock_init(&priv->vram.lock);

		attrs |= DMA_ATTR_NO_KERNEL_MAPPING;
		attrs |= DMA_ATTR_WRITE_COMBINE;

		/* note that for no-kernel-mapping, the vaddr returned
		 * is bogus, but non-null if allocation succeeded:
		 */
		p = dma_alloc_attrs(dev->dev, size,
				&priv->vram.paddr, GFP_KERNEL, attrs);
		if (!p) {
			dev_err(dev->dev, "failed to allocate VRAM\n");
			priv->vram.paddr = 0;
			return -ENOMEM;
		}

		dev_info(dev->dev, "VRAM: %08x->%08x\n",
				(uint32_t)priv->vram.paddr,
				(uint32_t)(priv->vram.paddr + size));
	}

	return ret;
}

#ifdef CONFIG_OF
static int msm_component_bind_all(struct device *dev,
				struct drm_device *drm_dev)
{
	int ret;

	ret = component_bind_all(dev, drm_dev);
	if (ret)
		DRM_ERROR("component_bind_all failed: %d\n", ret);

	return ret;
}
#else
static int msm_component_bind_all(struct device *dev,
				struct drm_device *drm_dev)
{
	return 0;
}
#endif

static int msm_drm_display_thread_create(struct sched_param param,
	struct msm_drm_private *priv, struct drm_device *ddev,
	struct device *dev)
{
	int i, ret = 0;

	/**
	 * this priority was found during empiric testing to have appropriate
	 * realtime scheduling to process display updates and interact with
	 * other real time and normal priority task
	 */
	param.sched_priority = 16;
	for (i = 0; i < priv->num_crtcs; i++) {

		/* initialize display thread */
		priv->disp_thread[i].crtc_id = priv->crtcs[i]->base.id;
		kthread_init_worker(&priv->disp_thread[i].worker);
		priv->disp_thread[i].dev = ddev;
		priv->disp_thread[i].thread =
			kthread_run(kthread_worker_fn,
				&priv->disp_thread[i].worker,
				"crtc_commit:%d", priv->disp_thread[i].crtc_id);
		ret = sched_setscheduler(priv->disp_thread[i].thread,
							SCHED_FIFO, &param);
		if (ret)
			pr_warn("display thread priority update failed: %d\n",
									ret);

		if (IS_ERR(priv->disp_thread[i].thread)) {
			dev_err(dev, "failed to create crtc_commit kthread\n");
			priv->disp_thread[i].thread = NULL;
		}

		/* initialize event thread */
		priv->event_thread[i].crtc_id = priv->crtcs[i]->base.id;
		kthread_init_worker(&priv->event_thread[i].worker);
		priv->event_thread[i].dev = ddev;
		priv->event_thread[i].thread =
			kthread_run(kthread_worker_fn,
				&priv->event_thread[i].worker,
				"crtc_event:%d", priv->event_thread[i].crtc_id);
		/**
		 * event thread should also run at same priority as disp_thread
		 * because it is handling frame_done events. A lower priority
		 * event thread and higher priority disp_thread can causes
		 * frame_pending counters beyond 2. This can lead to commit
		 * failure at crtc commit level.
		 */
		ret = sched_setscheduler(priv->event_thread[i].thread,
							SCHED_FIFO, &param);
		if (ret)
			pr_warn("display event thread priority update failed: %d\n",
									ret);

		if (IS_ERR(priv->event_thread[i].thread)) {
			dev_err(dev, "failed to create crtc_event kthread\n");
			priv->event_thread[i].thread = NULL;
		}

		if ((!priv->disp_thread[i].thread) ||
				!priv->event_thread[i].thread) {
			/* clean up previously created threads if any */
			for ( ; i >= 0; i--) {
				if (priv->disp_thread[i].thread) {
					kthread_stop(
						priv->disp_thread[i].thread);
					priv->disp_thread[i].thread = NULL;
				}

				if (priv->event_thread[i].thread) {
					kthread_stop(
						priv->event_thread[i].thread);
					priv->event_thread[i].thread = NULL;
				}
			}
			return -EINVAL;
		}
	}

	/**
	 * Since pp interrupt is heavy weight, try to queue the work
	 * into a dedicated worker thread, so that they dont interrupt
	 * other important events.
	 */
	kthread_init_worker(&priv->pp_event_worker);
	priv->pp_event_thread = kthread_run(kthread_worker_fn,
			&priv->pp_event_worker, "pp_event");

	ret = sched_setscheduler(priv->pp_event_thread,
						SCHED_FIFO, &param);
	if (ret)
		pr_warn("pp_event thread priority update failed: %d\n",
								ret);

	if (IS_ERR(priv->pp_event_thread)) {
		dev_err(dev, "failed to create pp_event kthread\n");
		ret = PTR_ERR(priv->pp_event_thread);
		priv->pp_event_thread = NULL;
		return ret;
	}

	return 0;

}
static struct msm_kms *_msm_drm_component_init_helper(
		struct msm_drm_private *priv,
		struct drm_device *ddev, struct device *dev,
		struct platform_device *pdev)
{
	int ret;
	struct msm_kms *kms;

	switch (get_mdp_ver(pdev)) {
	case KMS_MDP4:
		kms = mdp4_kms_init(ddev);
		break;
	case KMS_MDP5:
		kms = mdp5_kms_init(ddev);
		break;
	case KMS_SDE:
		kms = sde_kms_init(ddev);
		break;
	default:
		kms = ERR_PTR(-ENODEV);
		break;
	}

	if (IS_ERR_OR_NULL(kms)) {
		/*
		 * NOTE: once we have GPU support, having no kms should not
		 * be considered fatal.. ideally we would still support gpu
		 * and (for example) use dmabuf/prime to share buffers with
		 * imx drm driver on iMX5
		 */
		dev_err(dev, "failed to load kms\n");
		return kms;
	}
	priv->kms = kms;

	/**
	 * Since kms->funcs->hw_init(kms) might call
	 * drm_object_property_set_value to initialize some custom
	 * properties we need to make sure mode_config.funcs are populated
	 * beforehand to avoid dereferencing an unset value during the
	 * drm_drv_uses_atomic_modeset check.
	 */
	ddev->mode_config.funcs = &mode_config_funcs;

	ret = (kms)->funcs->hw_init(kms);
	if (ret) {
		dev_err(dev, "kms hw init failed: %d\n", ret);
		return ERR_PTR(ret);
	}

	return kms;
}

static int msm_drm_device_init(struct platform_device *pdev,
		struct drm_driver *drv)
{
	struct device *dev = &pdev->dev;
	struct drm_device *ddev;
	struct msm_drm_private *priv;
	int i, ret;

	ddev = drm_dev_alloc(drv, dev);
	if (IS_ERR(ddev)) {
		dev_err(dev, "failed to allocate drm_device\n");
		return PTR_ERR(ddev);
	}

	drm_mode_config_init(ddev);
	platform_set_drvdata(pdev, ddev);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto priv_alloc_fail;
	}

	ddev->dev_private = priv;
	priv->dev = ddev;

	ret = sde_power_resource_init(pdev, &priv->phandle);
	if (ret) {
		pr_err("sde power resource init failed\n");
		goto power_init_fail;
	}

	ret = sde_dbg_init(&pdev->dev);
	if (ret) {
		dev_err(dev, "failed to init sde dbg: %d\n", ret);
		goto dbg_init_fail;
	}

	pm_runtime_enable(dev);

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "resource enable failed: %d\n", ret);
		goto pm_runtime_error;
	}

	for (i = 0; i < SDE_POWER_HANDLE_DBUS_ID_MAX; i++)
		sde_power_data_bus_set_quota(&priv->phandle, i,
			SDE_POWER_HANDLE_CONT_SPLASH_BUS_AB_QUOTA,
			SDE_POWER_HANDLE_CONT_SPLASH_BUS_IB_QUOTA);

	return ret;

pm_runtime_error:
	sde_dbg_destroy();
dbg_init_fail:
	sde_power_resource_deinit(pdev, &priv->phandle);
power_init_fail:
priv_alloc_fail:
	drm_dev_put(ddev);
	kfree(priv);
	return ret;
}

static int msm_drm_component_init(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *ddev = platform_get_drvdata(pdev);
	struct msm_drm_private *priv = ddev->dev_private;
	struct msm_kms *kms = NULL;
	int ret;
	struct sched_param param = { 0 };
	struct drm_crtc *crtc;

	ret = msm_mdss_init(ddev);
	if (ret)
		goto mdss_init_fail;

	priv->wq = alloc_ordered_workqueue("msm_drm", 0);
	init_waitqueue_head(&priv->pending_crtcs_event);

	INIT_LIST_HEAD(&priv->client_event_list);
	INIT_LIST_HEAD(&priv->inactive_list);
	INIT_LIST_HEAD(&priv->vm_client_list);

	mutex_init(&priv->vm_client_lock);

	/* Bind all our sub-components: */
	ret = msm_component_bind_all(dev, ddev);
	if (ret)
		goto bind_fail;

	ret = msm_init_vram(ddev);
	if (ret)
		goto fail;

	ddev->mode_config.funcs = &mode_config_funcs;
	ddev->mode_config.helper_private = &mode_config_helper_funcs;

	kms = _msm_drm_component_init_helper(priv, ddev, dev, pdev);
	if (IS_ERR_OR_NULL(kms)) {
		dev_err(dev, "msm_drm_component_init_helper failed\n");
		goto fail;
	}

	/* Register rotator platform driver only after genpd init */
	sde_rotator_register();
	sde_rotator_smmu_driver_register();

	ret = msm_drm_display_thread_create(param, priv, ddev, dev);
	if (ret) {
		dev_err(dev, "msm_drm_display_thread_create failed\n");
		goto fail;
	}

	ret = drm_vblank_init(ddev, priv->num_crtcs);
	if (ret < 0) {
		dev_err(dev, "failed to initialize vblank\n");
		goto fail;
	}

	drm_for_each_crtc(crtc, ddev)
		drm_crtc_vblank_reset(crtc);

	if (kms) {
		pm_runtime_get_sync(dev);
		ret = drm_irq_install(ddev, platform_get_irq(pdev, 0));
		pm_runtime_put_sync(dev);
		if (ret < 0) {
			dev_err(dev, "failed to install IRQ handler\n");
			goto fail;
		}
	}

	drm_mode_config_reset(ddev);

	ret = drm_dev_register(ddev, 0);
	if (ret)
		goto fail;
	priv->registered = true;

	if (kms && kms->funcs && kms->funcs->cont_splash_config) {
		ret = kms->funcs->cont_splash_config(kms, NULL);
		if (ret) {
			dev_err(dev, "kms cont_splash config failed.\n");
			goto fail;
		}
	}

#ifdef CONFIG_DRM_FBDEV_EMULATION
	if (fbdev)
		priv->fbdev = msm_fbdev_init(ddev);
#endif

	/* create drm client only when fbdev is not supported */
	if (!priv->fbdev) {
		ret = drm_client_init(ddev, &kms->client, "kms_client", NULL);
		if (ret) {
			DRM_ERROR("failed to init kms_client: %d\n", ret);
			kms->client.dev = NULL;
			goto fail;
		}

		drm_client_register(&kms->client);
	}

	ret = sde_dbg_debugfs_register(dev);
	if (ret) {
		dev_err(dev, "failed to reg sde dbg debugfs: %d\n", ret);
		goto fail;
	}

	/* perform subdriver post initialization */
	if (kms && kms->funcs && kms->funcs->postinit) {
		ret = kms->funcs->postinit(kms);
		if (ret) {
			pr_err("kms post init failed: %d\n", ret);
			goto fail;
		}
	}

	drm_kms_helper_poll_init(ddev);

	return 0;

fail:
	msm_drm_uninit(dev);
	return ret;
bind_fail:
	msm_mdss_destroy(ddev);
mdss_init_fail:
	sde_dbg_destroy();
	sde_power_resource_deinit(pdev, &priv->phandle);
	drm_dev_put(ddev);
	kfree(priv);

	return ret;
}

/*
 * DRM operations:
 */

static int context_init(struct drm_device *dev, struct drm_file *file)
{
	struct msm_file_private *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mutex_init(&ctx->power_lock);

	file->driver_priv = ctx;

	if (dev && dev->dev_private) {
		struct msm_drm_private *priv = dev->dev_private;
		struct msm_kms *kms;

		kms = priv->kms;
		if (kms && kms->funcs && kms->funcs->postopen)
			kms->funcs->postopen(kms, file);
	}

	return 0;
}

static int msm_open(struct drm_device *dev, struct drm_file *file)
{
	return context_init(dev, file);
}

static void context_close(struct msm_file_private *ctx)
{
	kfree(ctx);
}

static void msm_preclose(struct drm_device *dev, struct drm_file *file)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;

	if (kms && kms->funcs && kms->funcs->preclose)
		kms->funcs->preclose(kms, file);
}

static void msm_postclose(struct drm_device *dev, struct drm_file *file)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_file_private *ctx = file->driver_priv;
	struct msm_kms *kms = priv->kms;

	if (!kms)
		return;

	if (kms->funcs && kms->funcs->postclose)
		kms->funcs->postclose(kms, file);

	mutex_lock(&dev->struct_mutex);
	if (ctx == priv->lastctx)
		priv->lastctx = NULL;
	mutex_unlock(&dev->struct_mutex);

	mutex_lock(&ctx->power_lock);
	if (ctx->enable_refcnt) {
		SDE_EVT32(ctx->enable_refcnt);
		pm_runtime_put_sync(dev->dev);
	}
	mutex_unlock(&ctx->power_lock);

	context_close(ctx);
}

static void msm_lastclose(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	int i, rc;

	if (!kms)
		return;

	/* check for splash status before triggering cleanup
	 * if we end up here with splash status ON i.e before first
	 * commit then ignore the last close call
	 */
	if (kms->funcs && kms->funcs->check_for_splash
		&& kms->funcs->check_for_splash(kms, NULL))
		return;

	/*
	 * clean up vblank disable immediately as this is the last close.
	 */
	for (i = 0; i < dev->num_crtcs; i++) {
		struct drm_vblank_crtc *vblank = &dev->vblank[i];
		struct timer_list *disable_timer = &vblank->disable_timer;

		if (del_timer_sync(disable_timer))
			disable_timer->function(disable_timer);
	}

	/* wait for pending vblank requests to be executed by worker thread */
	flush_workqueue(priv->wq);

	/* wait for any pending crtcs to finish before lastclose commit */
	msm_wait_event_timeout(priv->pending_crtcs_event, !priv->pending_crtcs,
			LASTCLOSE_TIMEOUT_MS, rc);
	if (!rc)
		DRM_INFO("wait for crtc mask 0x%x failed, commit anyway...\n",
				priv->pending_crtcs);

	if (priv->fbdev) {
		rc = drm_fb_helper_restore_fbdev_mode_unlocked(priv->fbdev);
		if (rc)
			DRM_ERROR("restore FBDEV mode failed: %d\n", rc);
	} else if (kms && kms->client.dev) {
		rc = drm_client_modeset_commit_force(&kms->client);
		if (rc)
			DRM_ERROR("client modeset commit failed: %d\n", rc);
	}

	/* wait again, before kms driver does it's lastclose commit */
	msm_wait_event_timeout(priv->pending_crtcs_event, !priv->pending_crtcs,
			LASTCLOSE_TIMEOUT_MS, rc);
	if (!rc)
		DRM_INFO("wait for crtc mask 0x%x failed, commit anyway...\n",
				priv->pending_crtcs);

	if (kms->funcs && kms->funcs->lastclose)
		kms->funcs->lastclose(kms);
}

static irqreturn_t msm_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	BUG_ON(!kms);
	return kms->funcs->irq(kms);
}

static void msm_irq_preinstall(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	BUG_ON(!kms);
	kms->funcs->irq_preinstall(kms);
}

static int msm_irq_postinstall(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	BUG_ON(!kms);

	if (kms->funcs->irq_postinstall)
		return kms->funcs->irq_postinstall(kms);

	return 0;
}

static void msm_irq_uninstall(struct drm_device *dev)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	BUG_ON(!kms);
	kms->funcs->irq_uninstall(kms);
}

static int msm_enable_vblank(struct drm_device *dev, unsigned int pipe)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	if (!kms)
		return -ENXIO;
	DBG("dev=%pK, crtc=%u", dev, pipe);
	return vblank_ctrl_queue_work(priv, pipe, true);
}

static void msm_disable_vblank(struct drm_device *dev, unsigned int pipe)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	if (!kms)
		return;
	DBG("dev=%pK, crtc=%u", dev, pipe);
	vblank_ctrl_queue_work(priv, pipe, false);
}

/*
 * DRM ioctls:
 */

static int msm_ioctl_gem_new(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_new *args = data;

	if (args->flags & ~MSM_BO_FLAGS) {
		DRM_ERROR("invalid flags: %08x\n", args->flags);
		return -EINVAL;
	}

	return msm_gem_new_handle(dev, file, args->size,
			args->flags, &args->handle, NULL);
}

static inline ktime_t to_ktime(struct drm_msm_timespec timeout)
{
	return ktime_set(timeout.tv_sec, timeout.tv_nsec);
}

static int msm_ioctl_gem_cpu_prep(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_cpu_prep *args = data;
	struct drm_gem_object *obj;
	ktime_t timeout = to_ktime(args->timeout);
	int ret;

	if (args->op & ~MSM_PREP_FLAGS) {
		DRM_ERROR("invalid op: %08x\n", args->op);
		return -EINVAL;
	}

	obj = drm_gem_object_lookup(file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = msm_gem_cpu_prep(obj, args->op, &timeout);

	drm_gem_object_put_unlocked(obj);

	return ret;
}

static int msm_ioctl_gem_cpu_fini(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_cpu_fini *args = data;
	struct drm_gem_object *obj;
	int ret;

	obj = drm_gem_object_lookup(file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = msm_gem_cpu_fini(obj);

	drm_gem_object_put_unlocked(obj);

	return ret;
}

static int msm_ioctl_gem_madvise(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_msm_gem_madvise *args = data;
	struct drm_gem_object *obj;
	int ret;

	switch (args->madv) {
	case MSM_MADV_DONTNEED:
	case MSM_MADV_WILLNEED:
		break;
	default:
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	obj = drm_gem_object_lookup(file, args->handle);
	if (!obj) {
		ret = -ENOENT;
		goto unlock;
	}

	ret = msm_gem_madvise(obj, args->madv);
	if (ret >= 0) {
		args->retained = ret;
		ret = 0;
	}

	drm_gem_object_put(obj);

unlock:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}

static int msm_drm_object_supports_event(struct drm_device *dev,
		struct drm_msm_event_req *req)
{
	int ret = -EINVAL;
	struct drm_mode_object *arg_obj;

	arg_obj = drm_mode_object_find(dev, NULL, req->object_id,
				req->object_type);
	if (!arg_obj)
		return -ENOENT;

	switch (arg_obj->type) {
	case DRM_MODE_OBJECT_CRTC:
	case DRM_MODE_OBJECT_CONNECTOR:
		ret = 0;
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	drm_mode_object_put(arg_obj);

	return ret;
}

static int msm_register_event(struct drm_device *dev,
	struct drm_msm_event_req *req, struct drm_file *file, bool en)
{
	int ret = -EINVAL;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	struct drm_mode_object *arg_obj;

	arg_obj = drm_mode_object_find(dev, file, req->object_id,
				req->object_type);
	if (!arg_obj)
		return -ENOENT;

	ret = kms->funcs->register_events(kms, arg_obj, req->event, en);

	drm_mode_object_put(arg_obj);

	return ret;
}

static int msm_event_client_count(struct drm_device *dev,
		struct drm_msm_event_req *req_event, bool locked)
{
	struct msm_drm_private *priv = dev->dev_private;
	unsigned long flag = 0;
	struct msm_drm_event *node;
	int count = 0;

	if (!locked)
		spin_lock_irqsave(&dev->event_lock, flag);
	list_for_each_entry(node, &priv->client_event_list, base.link) {
		if (node->event.base.type == req_event->event &&
			node->event.info.object_id == req_event->object_id)
			count++;
	}
	if (!locked)
		spin_unlock_irqrestore(&dev->event_lock, flag);

	return count;
}

static int msm_ioctl_register_event(struct drm_device *dev, void *data,
				    struct drm_file *file)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct drm_msm_event_req *req_event = data;
	struct msm_drm_event *client, *node;
	unsigned long flag = 0;
	bool dup_request = false;
	int ret = 0, count = 0;

	ret = msm_drm_object_supports_event(dev, req_event);
	if (ret) {
		DRM_ERROR("unsupported event %x object %x object id %d\n",
			req_event->event, req_event->object_type,
			req_event->object_id);
		return ret;
	}

	spin_lock_irqsave(&dev->event_lock, flag);
	list_for_each_entry(node, &priv->client_event_list, base.link) {
		if (node->base.file_priv != file)
			continue;
		if (node->event.base.type == req_event->event &&
			node->event.info.object_id == req_event->object_id) {
			DRM_DEBUG("duplicate request for event %x obj id %d\n",
				node->event.base.type,
				node->event.info.object_id);
			dup_request = true;
			break;
		}
	}
	spin_unlock_irqrestore(&dev->event_lock, flag);

	if (dup_request)
		return -EALREADY;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	client->base.file_priv = file;
	client->base.event = &client->event.base;
	client->event.base.type = req_event->event;
	memcpy(&client->event.info, req_event, sizeof(client->event.info));

	/* Get the count of clients that have registered for event.
	 * Event should be enabled for first client, for subsequent enable
	 * calls add to client list and return.
	 */
	count = msm_event_client_count(dev, req_event, false);
	if (count) {
		/* Add current client to list */
		spin_lock_irqsave(&dev->event_lock, flag);
		list_add_tail(&client->base.link, &priv->client_event_list);
		spin_unlock_irqrestore(&dev->event_lock, flag);
		return 0;
	}

	ret = msm_register_event(dev, req_event, file, true);
	if (ret) {
		DRM_ERROR("failed to enable event %x object %x object id %d\n",
			req_event->event, req_event->object_type,
			req_event->object_id);
		kfree(client);
	} else {
		/* Add current client to list */
		spin_lock_irqsave(&dev->event_lock, flag);
		list_add_tail(&client->base.link, &priv->client_event_list);
		spin_unlock_irqrestore(&dev->event_lock, flag);
	}

	return ret;
}

static int msm_ioctl_deregister_event(struct drm_device *dev, void *data,
				      struct drm_file *file)
{
	struct msm_drm_private *priv = dev->dev_private;
	struct drm_msm_event_req *req_event = data;
	struct msm_drm_event *client = NULL, *node, *temp;
	unsigned long flag = 0;
	int count = 0;
	bool found = false;
	int ret = 0;

	ret = msm_drm_object_supports_event(dev, req_event);
	if (ret) {
		DRM_ERROR("unsupported event %x object %x object id %d\n",
			req_event->event, req_event->object_type,
			req_event->object_id);
		return ret;
	}

	spin_lock_irqsave(&dev->event_lock, flag);
	list_for_each_entry_safe(node, temp, &priv->client_event_list,
			base.link) {
		if (node->event.base.type == req_event->event &&
		    node->event.info.object_id == req_event->object_id &&
		    node->base.file_priv == file) {
			client = node;
			list_del(&client->base.link);
			found = true;
			kfree(client);
			break;
		}
	}
	spin_unlock_irqrestore(&dev->event_lock, flag);

	if (!found)
		return -ENOENT;

	count = msm_event_client_count(dev, req_event, false);
	if (!count)
		ret = msm_register_event(dev, req_event, file, false);

	return ret;
}

void msm_mode_object_event_notify(struct drm_mode_object *obj,
		struct drm_device *dev, struct drm_event *event, u8 *payload)
{
	struct msm_drm_private *priv = NULL;
	unsigned long flags;
	struct msm_drm_event *notify, *node;
	int len = 0, ret;

	if (!obj || !event || !event->length || !payload) {
		DRM_ERROR("err param obj %pK event %pK len %d payload %pK\n",
			obj, event, ((event) ? (event->length) : -1),
			payload);
		return;
	}
	priv = (dev) ? dev->dev_private : NULL;
	if (!dev || !priv) {
		DRM_ERROR("invalid dev %pK priv %pK\n", dev, priv);
		return;
	}

	spin_lock_irqsave(&dev->event_lock, flags);
	list_for_each_entry(node, &priv->client_event_list, base.link) {
		if (node->event.base.type != event->type ||
			obj->id != node->event.info.object_id)
			continue;
		len = event->length + sizeof(struct msm_drm_event);
		if (node->base.file_priv->event_space < len) {
			DRM_ERROR("Insufficient space %d for event %x len %d\n",
				node->base.file_priv->event_space, event->type,
				len);
			continue;
		}
		notify = kzalloc(len, GFP_ATOMIC);
		if (!notify)
			continue;
		notify->base.file_priv = node->base.file_priv;
		notify->base.event = &notify->event.base;
		notify->event.base.type = node->event.base.type;
		notify->event.base.length = event->length +
					sizeof(struct drm_msm_event_resp);
		memcpy(&notify->event.info, &node->event.info,
			sizeof(notify->event.info));
		memcpy(notify->event.data, payload, event->length);
		ret = drm_event_reserve_init_locked(dev, node->base.file_priv,
			&notify->base, &notify->event.base);
		if (ret) {
			kfree(notify);
			continue;
		}
		drm_send_event_locked(dev, &notify->base);
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

static int msm_release(struct inode *inode, struct file *filp)
{
	struct drm_file *file_priv;
	struct drm_minor *minor;
	struct drm_device *dev;
	struct msm_drm_private *priv;
	struct msm_drm_event *node, *temp, *tmp_node;
	u32 count;
	unsigned long flags;
	LIST_HEAD(tmp_head);
	int ret = 0;

	mutex_lock(&msm_release_lock);

	file_priv = filp->private_data;
	if (!file_priv) {
		ret = -EINVAL;
		goto end;
	}

	minor = file_priv->minor;
	dev = minor->dev;
	priv = dev->dev_private;

	spin_lock_irqsave(&dev->event_lock, flags);
	list_for_each_entry_safe(node, temp, &priv->client_event_list,
			base.link) {
		if (node->base.file_priv != file_priv)
			continue;
		list_del(&node->base.link);
		list_add_tail(&node->base.link, &tmp_head);
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);

	list_for_each_entry_safe(node, temp, &tmp_head,
			base.link) {
		list_del(&node->base.link);
		count = msm_event_client_count(dev, &node->event.info, false);

		list_for_each_entry(tmp_node, &tmp_head, base.link) {
			if (tmp_node->event.base.type ==
					node->event.info.event &&
					tmp_node->event.info.object_id ==
					node->event.info.object_id)
				count++;
		}
		if (!count)
			msm_register_event(dev, &node->event.info, file_priv,
						false);
		kfree(node);
	}

	/**
	 * Handle preclose operation here for removing fb's whose
	 * refcount > 1. This operation is not triggered from upstream
	 * drm as msm_driver does not support DRIVER_LEGACY feature.
	 */
	if (drm_is_current_master(file_priv))
		msm_preclose(dev, file_priv);

	ret = drm_release(inode, filp);
	filp->private_data = NULL;
end:
	mutex_unlock(&msm_release_lock);
	return ret;
}

/**
 * msm_ioctl_rmfb2 - remove an FB from the configuration
 * @dev: drm device for the ioctl
 * @data: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 * Remove the FB specified by the user.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int msm_ioctl_rmfb2(struct drm_device *dev, void *data,
		    struct drm_file *file_priv)
{
	struct drm_framebuffer *fb = NULL;
	struct drm_framebuffer *fbl = NULL;
	uint32_t *id = data;
	int found = 0;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	fb = drm_framebuffer_lookup(dev, file_priv, *id);
	if (!fb)
		return -ENOENT;

	/* drop extra ref from traversing drm_framebuffer_lookup */
	drm_framebuffer_put(fb);

	mutex_lock(&file_priv->fbs_lock);
	list_for_each_entry(fbl, &file_priv->fbs, filp_head)
		if (fb == fbl)
			found = 1;
	if (!found) {
		mutex_unlock(&file_priv->fbs_lock);
		return -ENOENT;
	}

	list_del_init(&fb->filp_head);
	mutex_unlock(&file_priv->fbs_lock);

	drm_framebuffer_put(fb);

	return 0;
}
EXPORT_SYMBOL(msm_ioctl_rmfb2);

/**
 * msm_ioctl_power_ctrl - enable/disable power vote on MDSS Hw
 * @dev: drm device for the ioctl
 * @data: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 */
int msm_ioctl_power_ctrl(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct msm_file_private *ctx = file_priv->driver_priv;
	struct msm_drm_private *priv;
	struct drm_msm_power_ctrl *power_ctrl = data;
	bool vote_req = false;
	int old_cnt;
	int rc = 0;

	if (unlikely(!power_ctrl)) {
		DRM_ERROR("invalid ioctl data\n");
		return -EINVAL;
	}

	priv = dev->dev_private;

	mutex_lock(&ctx->power_lock);

	old_cnt = ctx->enable_refcnt;
	if (power_ctrl->enable) {
		if (!ctx->enable_refcnt)
			vote_req = true;
		ctx->enable_refcnt++;
	} else if (ctx->enable_refcnt) {
		ctx->enable_refcnt--;
		if (!ctx->enable_refcnt)
			vote_req = true;
	} else {
		pr_err("ignoring, unbalanced disable\n");
	}

	if (vote_req) {
		if (power_ctrl->enable)
			rc = pm_runtime_get_sync(dev->dev);
		else
			pm_runtime_put_sync(dev->dev);

		if (rc < 0)
			ctx->enable_refcnt = old_cnt;
		else
			rc = 0;
	}

	pr_debug("pid %d enable %d, refcnt %d, vote_req %d\n",
			current->pid, power_ctrl->enable, ctx->enable_refcnt,
			vote_req);
	SDE_EVT32(current->pid, power_ctrl->enable, ctx->enable_refcnt,
			vote_req);
	mutex_unlock(&ctx->power_lock);
	return rc;
}

/**
 * msm_ioctl_display_early_wakeup - early wakeup display.
 * @dev: drm device for the ioctl
 * @data: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 */
int msm_ioctl_display_hint_ops(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct drm_msm_display_hint *display_hint = data;
	struct drm_msm_early_wakeup early_wakeup;
	void __user *early_wakeup_usr;
	struct msm_drm_private *priv;
	struct msm_kms *kms;

	priv = dev->dev_private;
	kms = priv->kms;

	if (unlikely(!display_hint)) {
		DRM_ERROR("invalid ioctl data\n");
		return -EINVAL;
	}

	SDE_EVT32(display_hint->hint_flags);

	if (display_hint->hint_flags == DRM_MSM_DISPLAY_EARLY_WAKEUP_HINT) {
		if (!display_hint->data) {
			DRM_ERROR("early_wakeup: wrong parameter\n");
			return -EINVAL;
		}

		early_wakeup_usr =
			(void __user *)((uintptr_t)display_hint->data);

		if (copy_from_user(&early_wakeup, early_wakeup_usr,
				sizeof(early_wakeup))) {
			DRM_ERROR("early_wakeup: copy from user failed\n");
			return -EINVAL;
		}

		SDE_EVT32(early_wakeup.wakeup_hint);
		if (kms && kms->funcs && kms->funcs->display_early_wakeup
						&& early_wakeup.wakeup_hint)
			kms->funcs->display_early_wakeup(dev,
					early_wakeup.connector_id);
	}

	return 0;
}

static const struct drm_ioctl_desc msm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(MSM_GEM_NEW,      msm_ioctl_gem_new,      DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_CPU_PREP, msm_ioctl_gem_cpu_prep, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_CPU_FINI, msm_ioctl_gem_cpu_fini, DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_GEM_MADVISE,  msm_ioctl_gem_madvise,  DRM_AUTH|DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(SDE_WB_CONFIG, sde_wb_config, DRM_UNLOCKED|DRM_AUTH),
	DRM_IOCTL_DEF_DRV(MSM_REGISTER_EVENT,  msm_ioctl_register_event,
			  DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(MSM_DEREGISTER_EVENT,  msm_ioctl_deregister_event,
			  DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(MSM_RMFB2, msm_ioctl_rmfb2, DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(MSM_POWER_CTRL, msm_ioctl_power_ctrl,
			DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(MSM_DISPLAY_HINT, msm_ioctl_display_hint_ops,
			DRM_UNLOCKED),
};

static const struct vm_operations_struct vm_ops = {
	.fault = msm_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations fops = {
	.owner              = THIS_MODULE,
	.open               = drm_open,
	.release            = msm_release,
	.unlocked_ioctl     = drm_ioctl,
	.compat_ioctl       = drm_compat_ioctl,
	.poll               = drm_poll,
	.read               = drm_read,
	.llseek             = no_llseek,
	.mmap               = msm_gem_mmap,
};

static struct drm_driver msm_driver = {
	.driver_features    = DRIVER_GEM |
				DRIVER_RENDER |
				DRIVER_ATOMIC |
				DRIVER_MODESET,
	.open               = msm_open,
	.postclose          = msm_postclose,
	.lastclose          = msm_lastclose,
	.irq_handler        = msm_irq,
	.irq_preinstall     = msm_irq_preinstall,
	.irq_postinstall    = msm_irq_postinstall,
	.irq_uninstall      = msm_irq_uninstall,
	.enable_vblank      = msm_enable_vblank,
	.disable_vblank     = msm_disable_vblank,
	.gem_free_object    = msm_gem_free_object,
	.gem_vm_ops         = &vm_ops,
	.dumb_create        = msm_gem_dumb_create,
	.dumb_map_offset    = msm_gem_dumb_map_offset,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export   = drm_gem_prime_export,
	.gem_prime_import   = msm_gem_prime_import,
	.gem_prime_pin      = msm_gem_prime_pin,
	.gem_prime_unpin    = msm_gem_prime_unpin,
	.gem_prime_get_sg_table = msm_gem_prime_get_sg_table,
	.gem_prime_import_sg_table = msm_gem_prime_import_sg_table,
	.gem_prime_vmap     = msm_gem_prime_vmap,
	.gem_prime_vunmap   = msm_gem_prime_vunmap,
	.gem_prime_mmap     = msm_gem_prime_mmap,
	.ioctls             = msm_ioctls,
	.num_ioctls         = ARRAY_SIZE(msm_ioctls),
	.fops               = &fops,
	.name               = "msm_drm",
	.desc               = "MSM Snapdragon DRM",
	.date               = "20130625",
	.major              = MSM_VERSION_MAJOR,
	.minor              = MSM_VERSION_MINOR,
	.patchlevel         = MSM_VERSION_PATCHLEVEL,
};

#ifdef CONFIG_PM_SLEEP
static int msm_pm_suspend(struct device *dev)
{
	struct drm_device *ddev;
	struct msm_drm_private *priv;
	struct msm_kms *kms;

	if (!dev)
		return -EINVAL;

	ddev = dev_get_drvdata(dev);
	if (!ddev || !ddev->dev_private)
		return -EINVAL;

	priv = ddev->dev_private;
	kms = priv->kms;

	if (kms && kms->funcs && kms->funcs->pm_suspend)
		return kms->funcs->pm_suspend(dev);

	/* disable hot-plug polling */
	drm_kms_helper_poll_disable(ddev);

	return 0;
}

static int msm_pm_resume(struct device *dev)
{
	struct drm_device *ddev;
	struct msm_drm_private *priv;
	struct msm_kms *kms;

	if (!dev)
		return -EINVAL;

	ddev = dev_get_drvdata(dev);
	if (!ddev || !ddev->dev_private)
		return -EINVAL;

	priv = ddev->dev_private;
	kms = priv->kms;

	if (kms && kms->funcs && kms->funcs->pm_resume)
		return kms->funcs->pm_resume(dev);

	/* enable hot-plug polling */
	drm_kms_helper_poll_enable(ddev);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int msm_runtime_suspend(struct device *dev)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct msm_drm_private *priv = ddev->dev_private;

	DBG("");

	if (priv->mdss)
		msm_mdss_disable(priv->mdss);
	else
		sde_power_resource_enable(&priv->phandle, false);

	return 0;
}

static int msm_runtime_resume(struct device *dev)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct msm_drm_private *priv = ddev->dev_private;
	int ret;

	DBG("");

	if (priv->mdss)
		ret = msm_mdss_enable(priv->mdss);
	else
		ret = sde_power_resource_enable(&priv->phandle, true);

	return ret;
}
#endif

static const struct dev_pm_ops msm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(msm_pm_suspend, msm_pm_resume)
	SET_RUNTIME_PM_OPS(msm_runtime_suspend, msm_runtime_resume, NULL)
};

/*
 * Componentized driver support:
 */

/*
 * NOTE: duplication of the same code as exynos or imx (or probably any other).
 * so probably some room for some helpers
 */
static int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/*
 * Identify what components need to be added by parsing what remote-endpoints
 * our MDP output ports are connected to. In the case of LVDS on MDP4, there
 * is no external component that we need to add since LVDS is within MDP4
 * itself.
 */
static int add_components_mdp(struct device *mdp_dev,
			      struct component_match **matchptr)
{
	struct device_node *np = mdp_dev->of_node;
	struct device_node *ep_node;
	struct device *master_dev;

	/*
	 * on MDP4 based platforms, the MDP platform device is the component
	 * master that adds other display interface components to itself.
	 *
	 * on MDP5 based platforms, the MDSS platform device is the component
	 * master that adds MDP5 and other display interface components to
	 * itself.
	 */
	if (of_device_is_compatible(np, "qcom,mdp4"))
		master_dev = mdp_dev;
	else
		master_dev = mdp_dev->parent;

	for_each_endpoint_of_node(np, ep_node) {
		struct device_node *intf;
		struct of_endpoint ep;
		int ret;

		ret = of_graph_parse_endpoint(ep_node, &ep);
		if (ret) {
			dev_err(mdp_dev, "unable to parse port endpoint\n");
			of_node_put(ep_node);
			return ret;
		}

		/*
		 * The LCDC/LVDS port on MDP4 is a speacial case where the
		 * remote-endpoint isn't a component that we need to add
		 */
		if (of_device_is_compatible(np, "qcom,mdp4") &&
		    ep.port == 0)
			continue;

		/*
		 * It's okay if some of the ports don't have a remote endpoint
		 * specified. It just means that the port isn't connected to
		 * any external interface.
		 */
		intf = of_graph_get_remote_port_parent(ep_node);
		if (!intf)
			continue;

		if (of_device_is_available(intf))
			drm_of_component_match_add(master_dev, matchptr,
						   compare_of, intf);
		of_node_put(intf);
	}

	return 0;
}

static int compare_name_mdp(struct device *dev, void *data)
{
	return (strnstr(dev_name(dev), "mdp", strlen("mdp")) != NULL);
}

static int add_display_components(struct device *dev,
				  struct component_match **matchptr)
{
	struct device *mdp_dev = NULL;
	struct device_node *node;
	int ret;

	if (of_device_is_compatible(dev->of_node, "qcom,sde-kms")) {
		struct device_node *np = dev->of_node;
		unsigned int i;

		for (i = 0; ; i++) {
			node = of_parse_phandle(np, "connectors", i);
			if (!node)
				break;

			component_match_add(dev, matchptr, compare_of, node);
		}

		return 0;
	}

	/*
	 * MDP5 based devices don't have a flat hierarchy. There is a top level
	 * parent: MDSS, and children: MDP5, DSI, HDMI, eDP etc. Populate the
	 * children devices, find the MDP5 node, and then add the interfaces
	 * to our components list.
	 */
	if (of_device_is_compatible(dev->of_node, "qcom,mdss")) {
		ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
		if (ret) {
			dev_err(dev, "failed to populate children devices\n");
			return ret;
		}

		mdp_dev = device_find_child(dev, NULL, compare_name_mdp);
		if (!mdp_dev) {
			dev_err(dev, "failed to find MDSS MDP node\n");
			of_platform_depopulate(dev);
			return -ENODEV;
		}

		put_device(mdp_dev);

		/* add the MDP component itself */
		component_match_add(dev, matchptr, compare_of,
				   mdp_dev->of_node);
	} else {
		/* MDP4 */
		mdp_dev = dev;
	}

	ret = add_components_mdp(mdp_dev, matchptr);
	if (ret)
		of_platform_depopulate(dev);

	return ret;
}

struct msm_gem_address_space *
msm_gem_smmu_address_space_get(struct drm_device *dev,
		unsigned int domain)
{
	struct msm_drm_private *priv = NULL;
	struct msm_kms *kms;
	const struct msm_kms_funcs *funcs;
	struct msm_gem_address_space *aspace;

	if (!iommu_present(&platform_bus_type))
		return ERR_PTR(-ENODEV);

	if ((!dev) || (!dev->dev_private))
		return ERR_PTR(-EINVAL);

	priv = dev->dev_private;
	kms = priv->kms;
	if (!kms)
		return ERR_PTR(-EINVAL);

	funcs = kms->funcs;

	if ((!funcs) || (!funcs->get_address_space))
		return ERR_PTR(-EINVAL);

	aspace = funcs->get_address_space(priv->kms, domain);
	return aspace ? aspace : ERR_PTR(-EINVAL);
}

int msm_get_mixer_count(struct msm_drm_private *priv,
		const struct drm_display_mode *mode,
		const struct msm_resource_caps_info *res, u32 *num_lm)
{
	struct msm_kms *kms;
	const struct msm_kms_funcs *funcs;

	if (!priv) {
		DRM_ERROR("invalid drm private struct\n");
		return -EINVAL;
	}

	kms = priv->kms;
	if (!kms) {
		DRM_ERROR("invalid msm kms struct\n");
		return -EINVAL;
	}

	funcs = kms->funcs;
	if (!funcs || !funcs->get_mixer_count) {
		DRM_ERROR("invalid function pointers\n");
		return -EINVAL;
	}

	return funcs->get_mixer_count(priv->kms, mode, res, num_lm);
}

int msm_get_dsc_count(struct msm_drm_private *priv,
		u32 hdisplay, u32 *num_dsc)
{
	struct msm_kms *kms;
	const struct msm_kms_funcs *funcs;

	if (!priv) {
		DRM_ERROR("invalid drm private struct\n");
		return -EINVAL;
	}

	kms = priv->kms;
	if (!kms) {
		DRM_ERROR("invalid msm kms struct\n");
		return -EINVAL;
	}

	funcs = kms->funcs;
	if (!funcs || !funcs->get_dsc_count) {
		DRM_ERROR("invalid function pointers\n");
		return -EINVAL;
	}

	return funcs->get_dsc_count(priv->kms, hdisplay, num_dsc);
}

static int msm_drm_bind(struct device *dev)
{
	return msm_drm_component_init(dev);
}

static void msm_drm_unbind(struct device *dev)
{
	msm_drm_uninit(dev);
}

static const struct component_master_ops msm_drm_ops = {
	.bind = msm_drm_bind,
	.unbind = msm_drm_unbind,
};

static int msm_drm_component_dependency_check(struct device *dev)
{
	struct device_node *node;
	struct device_node *np = dev->of_node;
	unsigned int i;

	if (!of_device_is_compatible(dev->of_node, "qcom,sde-kms"))
		return 0;

	for (i = 0; ; i++) {
		node = of_parse_phandle(np, "connectors", i);
		if (!node)
			break;

		if (of_node_name_eq(node,"qcom,sde_rscc") &&
				of_device_is_available(node) &&
				of_node_check_flag(node, OF_POPULATED)) {
			struct platform_device *pdev =
					of_find_device_by_node(node);
			if (!platform_get_drvdata(pdev)) {
				dev_err(dev,
					"qcom,sde_rscc not probed yet\n");
				return -EPROBE_DEFER;
			} else {
				return 0;
			}
		}
	}

	return 0;
}
/*
 * Platform driver:
 */

static int msm_pdev_probe(struct platform_device *pdev)
{
	int ret;
	struct component_match *match = NULL;

	ret = msm_drm_component_dependency_check(&pdev->dev);
	if (ret)
		return ret;

	ret = msm_drm_device_init(pdev, &msm_driver);
	if (ret)
		return ret;

	ret = add_display_components(&pdev->dev, &match);
	if (ret)
		return ret;
	if (!match)
		return -ENODEV;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	return component_master_add_with_match(&pdev->dev, &msm_drm_ops, match);
}

static int msm_pdev_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &msm_drm_ops);
	of_platform_depopulate(&pdev->dev);

	return 0;
}

static void msm_pdev_shutdown(struct platform_device *pdev)
{
	struct drm_device *ddev = platform_get_drvdata(pdev);
	struct msm_drm_private *priv = NULL;

	if (!ddev) {
		DRM_ERROR("invalid drm device node\n");
		return;
	}

	priv = ddev->dev_private;
	if (!priv) {
		DRM_ERROR("invalid msm drm private node\n");
		return;
	}

	msm_lastclose(ddev);

	/* set this after lastclose to allow kickoff from lastclose */
	priv->shutdown_in_progress = true;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "qcom,mdp4", .data = (void *)KMS_MDP4 },
	{ .compatible = "qcom,mdss", .data = (void *)KMS_MDP5 },
	{ .compatible = "qcom,sde-kms", .data = (void *)KMS_SDE },
	{},
};
MODULE_DEVICE_TABLE(of, dt_match);

static struct platform_driver msm_platform_driver = {
	.probe      = msm_pdev_probe,
	.remove     = msm_pdev_remove,
	.shutdown   = msm_pdev_shutdown,
	.driver     = {
		.name   = "msm_drm",
		.of_match_table = dt_match,
		.pm     = &msm_pm_ops,
		.suppress_bind_attrs = true,
	},
};

static int __init msm_drm_register(void)
{
	if (!modeset)
		return -EINVAL;

	DBG("init");
	sde_rsc_rpmh_register();
	sde_rsc_register();
	dsi_display_register();
	msm_hdcp_register();
	dp_display_register();
	msm_smmu_driver_init();
	msm_dsi_register();
	msm_edp_register();
	msm_hdmi_register();
	sde_wb_register();
	return platform_driver_register(&msm_platform_driver);
}

static void __exit msm_drm_unregister(void)
{
	DBG("fini");
	platform_driver_unregister(&msm_platform_driver);
	sde_wb_unregister();
	msm_hdmi_unregister();
	msm_edp_unregister();
	msm_dsi_unregister();
	sde_rotator_smmu_driver_unregister();
	sde_rotator_unregister();
	msm_smmu_driver_cleanup();
	msm_hdcp_unregister();
	dp_display_unregister();
	dsi_display_unregister();
	sde_rsc_unregister();
}

module_init(msm_drm_register);
module_exit(msm_drm_unregister);

MODULE_AUTHOR("Rob Clark <robdclark@gmail.com");
MODULE_DESCRIPTION("MSM DRM Driver");
MODULE_LICENSE("GPL");
