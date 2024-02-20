// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__
#include <linux/sync_file.h>
#include <linux/dma-fence.h>
#include "msm_drv.h"
#include "sde_kms.h"
#include "sde_fence.h"

#define TIMELINE_VAL_LENGTH		128

void *sde_sync_get(uint64_t fd)
{
	/* force signed compare, fdget accepts an int argument */
	return (signed int)fd >= 0 ? sync_file_get_fence(fd) : NULL;
}

void sde_sync_put(void *fence)
{
	if (fence)
		dma_fence_put(fence);
}

signed long sde_sync_wait(void *fnc, long timeout_ms)
{
	struct dma_fence *fence = fnc;
	int rc;
	char timeline_str[TIMELINE_VAL_LENGTH];

	if (!fence)
		return -EINVAL;
	else if (dma_fence_is_signaled(fence))
		return timeout_ms ? msecs_to_jiffies(timeout_ms) : 1;

	rc = dma_fence_wait_timeout(fence, true,
				msecs_to_jiffies(timeout_ms));
	if (!rc || (rc == -EINVAL)) {
		if (fence->ops->timeline_value_str)
			fence->ops->timeline_value_str(fence,
					timeline_str, TIMELINE_VAL_LENGTH);

		SDE_ERROR(
			"fence driver name:%s timeline name:%s seqno:0x%llx timeline:%s signaled:0x%x\n",
			fence->ops->get_driver_name(fence),
			fence->ops->get_timeline_name(fence),
			fence->seqno, timeline_str,
			fence->ops->signaled ?
				fence->ops->signaled(fence) : 0xffffffff);
	}

	return rc;
}

uint32_t sde_sync_get_name_prefix(void *fence)
{
	const char *name;
	uint32_t i, prefix;
	struct dma_fence *f = fence;

	if (!fence)
		return 0;

	name = f->ops->get_driver_name(f);
	if (!name)
		return 0;

	prefix = 0x0;
	for (i = 0; i < sizeof(uint32_t) && name[i]; ++i)
		prefix = (prefix << CHAR_BIT) | name[i];

	return prefix;
}

/**
 * struct sde_fence - release/retire fence structure
 * @fence: base fence structure
 * @name: name of each fence- it is fence timeline + commit_count
 * @fence_list: list to associated this fence on timeline/context
 * @fd: fd attached to this fence - debugging purpose.
 */
struct sde_fence {
	struct dma_fence base;
	struct sde_fence_context *ctx;
	char name[SDE_FENCE_NAME_SIZE];
	struct list_head	fence_list;
	int fd;
};

static void sde_fence_destroy(struct kref *kref)
{
	struct sde_fence_context *ctx;

	if (!kref) {
		SDE_ERROR("received invalid kref\n");
		return;
	}

	ctx = container_of(kref, struct sde_fence_context, kref);
	kfree(ctx);
}

static inline struct sde_fence *to_sde_fence(struct dma_fence *fence)
{
	return container_of(fence, struct sde_fence, base);
}

static const char *sde_fence_get_driver_name(struct dma_fence *fence)
{
	struct sde_fence *f = to_sde_fence(fence);

	return f->name;
}

static const char *sde_fence_get_timeline_name(struct dma_fence *fence)
{
	struct sde_fence *f = to_sde_fence(fence);

	return f->ctx->name;
}

static bool sde_fence_enable_signaling(struct dma_fence *fence)
{
	return true;
}

static bool sde_fence_signaled(struct dma_fence *fence)
{
	struct sde_fence *f = to_sde_fence(fence);
	bool status;

	status = ((int)(fence->seqno - f->ctx->done_count) <= 0);
	SDE_DEBUG("status:%d fence seq:%llu and timeline:%d\n",
			status, fence->seqno, f->ctx->done_count);
	return status;
}

static void sde_fence_release(struct dma_fence *fence)
{
	struct sde_fence *f;

	if (fence) {
		f = to_sde_fence(fence);
		kref_put(&f->ctx->kref, sde_fence_destroy);
		kfree(f);
	}
}

static void sde_fence_value_str(struct dma_fence *fence, char *str, int size)
{
	if (!fence || !str)
		return;

	snprintf(str, size, "%llu", fence->seqno);
}

static void sde_fence_timeline_value_str(struct dma_fence *fence, char *str,
		int size)
{
	struct sde_fence *f = to_sde_fence(fence);

	if (!fence || !f->ctx || !str)
		return;

	snprintf(str, size, "%d", f->ctx->done_count);
}

static struct dma_fence_ops sde_fence_ops = {
	.get_driver_name = sde_fence_get_driver_name,
	.get_timeline_name = sde_fence_get_timeline_name,
	.enable_signaling = sde_fence_enable_signaling,
	.signaled = sde_fence_signaled,
	.wait = dma_fence_default_wait,
	.release = sde_fence_release,
	.fence_value_str = sde_fence_value_str,
	.timeline_value_str = sde_fence_timeline_value_str,
};

/**
 * _sde_fence_create_fd - create fence object and return an fd for it
 * This function is NOT thread-safe.
 * @timeline: Timeline to associate with fence
 * @val: Timeline value at which to signal the fence
 * Return: File descriptor on success, or error code on error
 */
static int _sde_fence_create_fd(void *fence_ctx, uint32_t val)
{
	struct sde_fence *sde_fence;
	struct sync_file *sync_file;
	signed int fd = -EINVAL;
	struct sde_fence_context *ctx = fence_ctx;

	if (!ctx) {
		SDE_ERROR("invalid context\n");
		goto exit;
	}

	sde_fence = kzalloc(sizeof(*sde_fence), GFP_KERNEL);
	if (!sde_fence)
		return -ENOMEM;

	sde_fence->ctx = fence_ctx;
	snprintf(sde_fence->name, SDE_FENCE_NAME_SIZE, "sde_fence:%s:%u",
						sde_fence->ctx->name, val);
	dma_fence_init(&sde_fence->base, &sde_fence_ops, &ctx->lock,
		ctx->context, val);
	kref_get(&ctx->kref);

	/* create fd */
	fd = get_unused_fd_flags(0);
	if (fd < 0) {
		SDE_ERROR("failed to get_unused_fd_flags(), %s\n",
							sde_fence->name);
		dma_fence_put(&sde_fence->base);
		goto exit;
	}

	/* create fence */
	sync_file = sync_file_create(&sde_fence->base);
	if (sync_file == NULL) {
		put_unused_fd(fd);
		fd = -EINVAL;
		SDE_ERROR("couldn't create fence, %s\n", sde_fence->name);
		dma_fence_put(&sde_fence->base);
		goto exit;
	}

	fd_install(fd, sync_file->file);
	sde_fence->fd = fd;

	spin_lock(&ctx->list_lock);
	list_add_tail(&sde_fence->fence_list, &ctx->fence_list_head);
	spin_unlock(&ctx->list_lock);

exit:
	return fd;
}

struct sde_fence_context *sde_fence_init(const char *name, uint32_t drm_id)
{
	struct sde_fence_context *ctx;

	if (!name) {
		SDE_ERROR("invalid argument(s)\n");
		return ERR_PTR(-EINVAL);
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);

	if (!ctx) {
		SDE_ERROR("failed to alloc fence ctx\n");
		return ERR_PTR(-ENOMEM);
	}

	strlcpy(ctx->name, name, ARRAY_SIZE(ctx->name));
	ctx->drm_id = drm_id;
	kref_init(&ctx->kref);
	ctx->context = dma_fence_context_alloc(1);

	spin_lock_init(&ctx->lock);
	spin_lock_init(&ctx->list_lock);
	INIT_LIST_HEAD(&ctx->fence_list_head);

	return ctx;
}

void sde_fence_deinit(struct sde_fence_context *ctx)
{
	if (!ctx) {
		SDE_ERROR("invalid fence\n");
		return;
	}

	kref_put(&ctx->kref, sde_fence_destroy);
}

void sde_fence_prepare(struct sde_fence_context *ctx)
{
	unsigned long flags;

	if (!ctx) {
		SDE_ERROR("invalid argument(s), fence %pK\n", ctx);
	} else {
		spin_lock_irqsave(&ctx->lock, flags);
		++ctx->commit_count;
		spin_unlock_irqrestore(&ctx->lock, flags);
	}
}

static void _sde_fence_trigger(struct sde_fence_context *ctx, bool error)
{
	unsigned long flags;
	struct sde_fence *fc, *next;
	bool is_signaled = false;

	kref_get(&ctx->kref);

	spin_lock(&ctx->list_lock);
	if (list_empty(&ctx->fence_list_head)) {
		SDE_DEBUG("nothing to trigger!\n");
		goto end;
	}

	list_for_each_entry_safe(fc, next, &ctx->fence_list_head, fence_list) {
		spin_lock_irqsave(&ctx->lock, flags);
		if (error)
			dma_fence_set_error(&fc->base, -EBUSY);
		is_signaled = dma_fence_is_signaled_locked(&fc->base);
		spin_unlock_irqrestore(&ctx->lock, flags);

		if (is_signaled) {
			list_del_init(&fc->fence_list);
			dma_fence_put(&fc->base);
		}
	}
end:
	spin_unlock(&ctx->list_lock);
	kref_put(&ctx->kref, sde_fence_destroy);
}

int sde_fence_create(struct sde_fence_context *ctx, uint64_t *val,
							uint32_t offset)
{
	uint32_t trigger_value;
	int fd, rc = -EINVAL;
	unsigned long flags;

	if (!ctx || !val) {
		SDE_ERROR("invalid argument(s), fence %d, pval %d\n",
				ctx != NULL, val != NULL);
		return rc;
	}

	/*
	 * Allow created fences to have a constant offset with respect
	 * to the timeline. This allows us to delay the fence signalling
	 * w.r.t. the commit completion (e.g., an offset of +1 would
	 * cause fences returned during a particular commit to signal
	 * after an additional delay of one commit, rather than at the
	 * end of the current one.
	 */
	spin_lock_irqsave(&ctx->lock, flags);
	trigger_value = ctx->commit_count + offset;
	spin_unlock_irqrestore(&ctx->lock, flags);

	fd = _sde_fence_create_fd(ctx, trigger_value);
	*val = fd;
	SDE_DEBUG("fd:%d trigger:%d commit:%d offset:%d\n",
			fd, trigger_value, ctx->commit_count, offset);

	SDE_EVT32(ctx->drm_id, trigger_value, fd);
	rc = (fd >= 0) ? 0 : fd;

	return rc;
}

void sde_fence_signal(struct sde_fence_context *ctx, ktime_t ts,
		enum sde_fence_event fence_event)
{
	unsigned long flags;

	if (!ctx) {
		SDE_ERROR("invalid ctx, %pK\n", ctx);
		return;
	}

	spin_lock_irqsave(&ctx->lock, flags);
	if (fence_event == SDE_FENCE_RESET_TIMELINE) {
		if ((int)(ctx->done_count - ctx->commit_count) < 0) {
			SDE_DEBUG(
			  "timeline reset attempt! done count:%d commit:%d\n",
				ctx->done_count, ctx->commit_count);
			ctx->done_count = ctx->commit_count;
			SDE_EVT32(ctx->drm_id, ctx->done_count,
				ctx->commit_count, ktime_to_us(ts),
				fence_event, SDE_EVTLOG_FUNC_CASE1);
		} else {
			spin_unlock_irqrestore(&ctx->lock, flags);
			return;
		}
	} else if ((int)(ctx->done_count - ctx->commit_count) < 0) {
		++ctx->done_count;
		SDE_DEBUG("fence_signal:done count:%d commit count:%d\n",
					ctx->done_count, ctx->commit_count);
	} else {
		SDE_ERROR("extra signal attempt! done count:%d commit:%d\n",
					ctx->done_count, ctx->commit_count);
		SDE_EVT32(ctx->drm_id, ctx->done_count, ctx->commit_count,
			ktime_to_us(ts), fence_event, SDE_EVTLOG_FATAL);
		spin_unlock_irqrestore(&ctx->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&ctx->lock, flags);

	SDE_EVT32(ctx->drm_id, ctx->done_count, ctx->commit_count,
			ktime_to_us(ts));

	_sde_fence_trigger(ctx, (fence_event == SDE_FENCE_SIGNAL_ERROR));
}

void sde_fence_timeline_status(struct sde_fence_context *ctx,
					struct drm_mode_object *drm_obj)
{
	char *obj_name;

	if (!ctx || !drm_obj) {
		SDE_ERROR("invalid input params\n");
		return;
	}

	switch (drm_obj->type) {
	case DRM_MODE_OBJECT_CRTC:
		obj_name = "crtc";
		break;
	case DRM_MODE_OBJECT_CONNECTOR:
		obj_name = "connector";
		break;
	default:
		obj_name = "unknown";
		break;
	}

	SDE_ERROR("drm obj:%s id:%d type:0x%x done_count:%d commit_count:%d\n",
		obj_name, drm_obj->id, drm_obj->type, ctx->done_count,
		ctx->commit_count);
}

void sde_fence_list_dump(struct dma_fence *fence, struct seq_file **s)
{
	char timeline_str[TIMELINE_VAL_LENGTH];

	if (fence->ops->timeline_value_str)
		fence->ops->timeline_value_str(fence,
		timeline_str, TIMELINE_VAL_LENGTH);

	seq_printf(*s, "fence name:%s timeline name:%s seqno:0x%llx timeline:%s signaled:0x%x\n",
		fence->ops->get_driver_name(fence),
		fence->ops->get_timeline_name(fence),
		fence->seqno, timeline_str,
		fence->ops->signaled ?
		fence->ops->signaled(fence) : 0xffffffff);
}

void sde_debugfs_timeline_dump(struct sde_fence_context *ctx,
		struct drm_mode_object *drm_obj, struct seq_file **s)
{
	char *obj_name;
	struct sde_fence *fc, *next;
	struct dma_fence *fence;

	if (!ctx || !drm_obj) {
		SDE_ERROR("invalid input params\n");
		return;
	}

	switch (drm_obj->type) {
	case DRM_MODE_OBJECT_CRTC:
		obj_name = "crtc";
		break;
	case DRM_MODE_OBJECT_CONNECTOR:
		obj_name = "connector";
		break;
	default:
		obj_name = "unknown";
		break;
	}

	seq_printf(*s, "drm obj:%s id:%d type:0x%x done_count:%d commit_count:%d\n",
		obj_name, drm_obj->id, drm_obj->type, ctx->done_count,
		ctx->commit_count);

	spin_lock(&ctx->list_lock);
	list_for_each_entry_safe(fc, next, &ctx->fence_list_head, fence_list) {
		fence = &fc->base;
		sde_fence_list_dump(fence, s);
	}
	spin_unlock(&ctx->list_lock);
}
