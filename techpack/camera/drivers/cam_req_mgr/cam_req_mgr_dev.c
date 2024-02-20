// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/highmem.h>
#include <linux/types.h>
#include <linux/rwsem.h>

#include <mm/slab.h>

#include <media/v4l2-fh.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/cam_req_mgr.h>
#include <media/cam_defs.h>
#include <linux/list_sort.h>

#include "cam_req_mgr_dev.h"
#include "cam_req_mgr_util.h"
#include "cam_req_mgr_core.h"
#include "cam_subdev.h"
#include "cam_mem_mgr.h"
#include "cam_debug_util.h"
#include "cam_common_util.h"
#include "cam_compat.h"
#include "cam_cpas_hw.h"

#define CAM_REQ_MGR_EVENT_MAX 30

static struct cam_req_mgr_device g_dev;
struct kmem_cache *g_cam_req_mgr_timer_cachep;
static struct list_head cam_req_mgr_ordered_sd_list;

DECLARE_RWSEM(rwsem_lock);

static struct device_attribute camera_debug_sysfs_attr =
	__ATTR(debug_node, 0600, NULL, cam_debug_sysfs_node_store);

static int cam_media_device_setup(struct device *dev)
{
	int rc;

	g_dev.v4l2_dev->mdev = kzalloc(sizeof(*g_dev.v4l2_dev->mdev),
		GFP_KERNEL);
	if (!g_dev.v4l2_dev->mdev) {
		rc = -ENOMEM;
		goto mdev_fail;
	}

	media_device_init(g_dev.v4l2_dev->mdev);
	g_dev.v4l2_dev->mdev->dev = dev;
	strlcpy(g_dev.v4l2_dev->mdev->model, CAM_REQ_MGR_VNODE_NAME,
		sizeof(g_dev.v4l2_dev->mdev->model));

	rc = media_device_register(g_dev.v4l2_dev->mdev);
	if (rc)
		goto media_fail;

	return rc;

media_fail:
	kfree(g_dev.v4l2_dev->mdev);
	g_dev.v4l2_dev->mdev = NULL;
mdev_fail:
	return rc;
}

static void cam_media_device_cleanup(void)
{
	media_device_unregister(g_dev.v4l2_dev->mdev);
	media_device_cleanup(g_dev.v4l2_dev->mdev);
	kfree(g_dev.v4l2_dev->mdev);
	g_dev.v4l2_dev->mdev = NULL;
}

static int cam_v4l2_device_setup(struct device *dev)
{
	int rc;

	g_dev.v4l2_dev = kzalloc(sizeof(*g_dev.v4l2_dev),
		GFP_KERNEL);
	if (!g_dev.v4l2_dev)
		return -ENOMEM;

	rc = v4l2_device_register(dev, g_dev.v4l2_dev);
	if (rc)
		goto reg_fail;

	return rc;

reg_fail:
	kfree(g_dev.v4l2_dev);
	g_dev.v4l2_dev = NULL;
	return rc;
}

static void cam_v4l2_device_cleanup(void)
{
	v4l2_device_unregister(g_dev.v4l2_dev);
	kfree(g_dev.v4l2_dev);
	g_dev.v4l2_dev = NULL;
}


void cam_req_mgr_rwsem_read_op(enum cam_subdev_rwsem lock)
{
	if (lock == CAM_SUBDEV_LOCK)
		down_read(&rwsem_lock);
	else if (lock == CAM_SUBDEV_UNLOCK)
		up_read(&rwsem_lock);
}

static void cam_req_mgr_rwsem_write_op(enum cam_subdev_rwsem lock)
{
	if (lock == CAM_SUBDEV_LOCK)
		down_write(&rwsem_lock);
	else if (lock == CAM_SUBDEV_UNLOCK)
		up_write(&rwsem_lock);
}

static int cam_req_mgr_open(struct file *filep)
{
	int rc;

	cam_req_mgr_rwsem_write_op(CAM_SUBDEV_LOCK);

	mutex_lock(&g_dev.cam_lock);
	if (g_dev.open_cnt >= 1) {
		rc = -EALREADY;
		goto end;
	}

	rc = v4l2_fh_open(filep);
	if (rc) {
		CAM_ERR(CAM_CRM, "v4l2_fh_open failed: %d", rc);
		goto end;
	}

	spin_lock_bh(&g_dev.cam_eventq_lock);
	g_dev.cam_eventq = filep->private_data;
	spin_unlock_bh(&g_dev.cam_eventq_lock);

	g_dev.open_cnt++;
	g_dev.read_active_dev_id_hdls = 0;
	rc = cam_mem_mgr_init();
	if (rc) {
		g_dev.open_cnt--;
		CAM_ERR(CAM_CRM, "mem mgr init failed");
		goto mem_mgr_init_fail;
	}

	mutex_unlock(&g_dev.cam_lock);
	cam_req_mgr_rwsem_write_op(CAM_SUBDEV_UNLOCK);
	return rc;

mem_mgr_init_fail:
	v4l2_fh_release(filep);
end:
	mutex_unlock(&g_dev.cam_lock);
	cam_req_mgr_rwsem_write_op(CAM_SUBDEV_UNLOCK);
	return rc;
}

static unsigned int cam_req_mgr_poll(struct file *f,
	struct poll_table_struct *pll_table)
{
	int rc = 0;
	struct v4l2_fh *eventq = f->private_data;

	if (!eventq)
		return -EINVAL;

	poll_wait(f, &eventq->wait, pll_table);
	if (v4l2_event_pending(eventq))
		rc = POLLPRI;

	return rc;
}

static int cam_req_mgr_close(struct file *filep)
{
	struct v4l2_subdev *sd;
	struct cam_subdev *csd;
	struct v4l2_fh *vfh = filep->private_data;
	struct v4l2_subdev_fh *subdev_fh = to_v4l2_subdev_fh(vfh);

	CAM_WARN(CAM_CRM,
		"release invoked associated userspace process has died, open_cnt: %d",
		g_dev.open_cnt);

	cam_req_mgr_rwsem_write_op(CAM_SUBDEV_LOCK);

	mutex_lock(&g_dev.cam_lock);

	if (g_dev.open_cnt <= 0) {
		mutex_unlock(&g_dev.cam_lock);
		cam_req_mgr_rwsem_write_op(CAM_SUBDEV_UNLOCK);
		return -EINVAL;
	}

	cam_req_mgr_handle_core_shutdown();
	g_dev.shutdown_state = true;

	list_for_each_entry(csd, &cam_req_mgr_ordered_sd_list, list) {
		sd = &csd->sd;
		if (!(sd->flags & V4L2_SUBDEV_FL_HAS_DEVNODE))
			continue;
		if (sd->internal_ops) {
			CAM_DBG(CAM_CRM, "Invoke subdev close for device %s",
				sd->name);
			v4l2_subdev_call(sd, core, ioctl,
				CAM_SD_SHUTDOWN, subdev_fh);
		}
	}

	g_dev.open_cnt--;
	g_dev.shutdown_state = false;
	g_dev.read_active_dev_id_hdls = 0;
	v4l2_fh_release(filep);

	spin_lock_bh(&g_dev.cam_eventq_lock);
	g_dev.cam_eventq = NULL;
	spin_unlock_bh(&g_dev.cam_eventq_lock);

	cam_req_mgr_util_free_hdls();
	cam_mem_mgr_deinit();
	mutex_unlock(&g_dev.cam_lock);

	cam_req_mgr_rwsem_write_op(CAM_SUBDEV_UNLOCK);

	return 0;
}

static struct v4l2_file_operations g_cam_fops = {
	.owner  = THIS_MODULE,
	.open   = cam_req_mgr_open,
	.poll   = cam_req_mgr_poll,
	.release = cam_req_mgr_close,
	.unlocked_ioctl   = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
};

static void cam_v4l2_event_queue_notify_error(const struct v4l2_event *old,
	struct v4l2_event *new)
{
	struct cam_req_mgr_message *ev_header;

	ev_header = CAM_REQ_MGR_GET_PAYLOAD_PTR((*old),
		struct cam_req_mgr_message);

	switch (old->id) {
	case V4L_EVENT_CAM_REQ_MGR_SOF:
	case V4L_EVENT_CAM_REQ_MGR_SOF_BOOT_TS:
		if (ev_header->u.frame_msg.request_id)
			CAM_ERR(CAM_CRM,
				"Failed to notify %s Sess %X FrameId %lld FrameMeta %d ReqId %lld link %X",
				((old->id == V4L_EVENT_CAM_REQ_MGR_SOF) ?
				"SOF_TS" : "BOOT_TS"),
				ev_header->session_hdl,
				ev_header->u.frame_msg.frame_id,
				ev_header->u.frame_msg.frame_id_meta,
				ev_header->u.frame_msg.request_id,
				ev_header->u.frame_msg.link_hdl);
		else
			CAM_WARN_RATE_LIMIT_CUSTOM(CAM_CRM, 5, 1,
				"Failed to notify %s Sess %X FrameId %lld FrameMeta %d ReqId %lld link %X",
				((old->id == V4L_EVENT_CAM_REQ_MGR_SOF) ?
				"SOF_TS" : "BOOT_TS"),
				ev_header->session_hdl,
				ev_header->u.frame_msg.frame_id,
				ev_header->u.frame_msg.frame_id_meta,
				ev_header->u.frame_msg.request_id,
				ev_header->u.frame_msg.link_hdl);
		break;
	case V4L_EVENT_CAM_REQ_MGR_ERROR:
		CAM_ERR_RATE_LIMIT(CAM_CRM,
			"Failed to notify ERROR Sess %X ReqId %d Link %X Type %d",
			ev_header->session_hdl,
			ev_header->u.err_msg.request_id,
			ev_header->u.err_msg.link_hdl,
			ev_header->u.err_msg.error_type);
		break;
	default:
		CAM_ERR(CAM_CRM, "Failed to notify crm event id %d",
			old->id);
	}
}

static struct v4l2_subscribed_event_ops g_cam_v4l2_ops = {
	.merge = cam_v4l2_event_queue_notify_error,
};

static int cam_subscribe_event(struct v4l2_fh *fh,
	const struct v4l2_event_subscription *sub)
{
	return v4l2_event_subscribe(fh, sub, CAM_REQ_MGR_EVENT_MAX,
		&g_cam_v4l2_ops);
}

static int cam_unsubscribe_event(struct v4l2_fh *fh,
	const struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

static long cam_private_ioctl(struct file *file, void *fh,
	bool valid_prio, unsigned int cmd, void *arg)
{
	int rc;
	struct cam_control *k_ioctl;

	if ((!arg) || (cmd != VIDIOC_CAM_CONTROL))
		return -EINVAL;

	k_ioctl = (struct cam_control *)arg;

	if (!k_ioctl->handle)
		return -EINVAL;

	switch (k_ioctl->op_code) {
	case CAM_REQ_MGR_CREATE_SESSION: {
		struct cam_req_mgr_session_info ses_info;

		if (k_ioctl->size != sizeof(ses_info))
			return -EINVAL;

		if (copy_from_user(&ses_info,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_req_mgr_session_info))) {
			return -EFAULT;
		}

		rc = cam_req_mgr_create_session(&ses_info);
		if (!rc)
			if (copy_to_user(
				u64_to_user_ptr(k_ioctl->handle),
				&ses_info,
				sizeof(struct cam_req_mgr_session_info)))
				rc = -EFAULT;
		}
		break;

	case CAM_REQ_MGR_DESTROY_SESSION: {
		struct cam_req_mgr_session_info ses_info;

		if (k_ioctl->size != sizeof(ses_info))
			return -EINVAL;

		if (copy_from_user(&ses_info,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_req_mgr_session_info))) {
			return -EFAULT;
		}

		rc = cam_req_mgr_destroy_session(&ses_info, false);
		}
		break;

	case CAM_REQ_MGR_LINK: {
		struct cam_req_mgr_ver_info ver_info;

		if (k_ioctl->size != sizeof(ver_info.u.link_info_v1))
			return -EINVAL;

		if (copy_from_user(&ver_info.u.link_info_v1,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_req_mgr_link_info))) {
			return -EFAULT;
		}
		ver_info.version = VERSION_1;
		rc = cam_req_mgr_link(&ver_info);
		if (!rc)
			if (copy_to_user(
				u64_to_user_ptr(k_ioctl->handle),
				&ver_info.u.link_info_v1,
				sizeof(struct cam_req_mgr_link_info)))
				rc = -EFAULT;
		}
		break;

	case CAM_REQ_MGR_LINK_V2: {
		struct cam_req_mgr_ver_info ver_info;

		if (k_ioctl->size != sizeof(ver_info.u.link_info_v2))
			return -EINVAL;

		if (copy_from_user(&ver_info.u.link_info_v2,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_req_mgr_link_info_v2))) {
			return -EFAULT;
		}

		ver_info.version = VERSION_2;
		rc = cam_req_mgr_link_v2(&ver_info);
		if (!rc)
			if (copy_to_user(
				u64_to_user_ptr(k_ioctl->handle),
				&ver_info.u.link_info_v2,
				sizeof(struct cam_req_mgr_link_info_v2)))
				rc = -EFAULT;
			}
		break;

	case CAM_REQ_MGR_UNLINK: {
		struct cam_req_mgr_unlink_info unlink_info;

		if (k_ioctl->size != sizeof(unlink_info))
			return -EINVAL;

		if (copy_from_user(&unlink_info,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_req_mgr_unlink_info))) {
			return -EFAULT;
		}

		rc = cam_req_mgr_unlink(&unlink_info);
		}
		break;

	case CAM_REQ_MGR_SCHED_REQ: {
		struct cam_req_mgr_sched_request sched_req;

		if (k_ioctl->size != sizeof(sched_req))
			return -EINVAL;

		if (copy_from_user(&sched_req,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_req_mgr_sched_request))) {
			return -EFAULT;
		}

		rc = cam_req_mgr_schedule_request(&sched_req);
		}
		break;

	case CAM_REQ_MGR_FLUSH_REQ: {
		struct cam_req_mgr_flush_info flush_info;

		if (k_ioctl->size != sizeof(flush_info))
			return -EINVAL;

		if (copy_from_user(&flush_info,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_req_mgr_flush_info))) {
			return -EFAULT;
		}

		rc = cam_req_mgr_flush_requests(&flush_info);
		}
		break;

	case CAM_REQ_MGR_SYNC_MODE: {
		struct cam_req_mgr_sync_mode sync_info;

		if (k_ioctl->size != sizeof(sync_info))
			return -EINVAL;

		if (copy_from_user(&sync_info,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_req_mgr_sync_mode))) {
			return -EFAULT;
		}

		rc = cam_req_mgr_sync_config(&sync_info);
		}
		break;
	case CAM_REQ_MGR_ALLOC_BUF: {
		struct cam_mem_mgr_alloc_cmd cmd;

		if (k_ioctl->size != sizeof(cmd))
			return -EINVAL;

		if (copy_from_user(&cmd,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_mem_mgr_alloc_cmd))) {
			rc = -EFAULT;
			break;
		}

		rc = cam_mem_mgr_alloc_and_map(&cmd);
		if (!rc)
			if (copy_to_user(
				u64_to_user_ptr(k_ioctl->handle),
				&cmd, sizeof(struct cam_mem_mgr_alloc_cmd))) {
				rc = -EFAULT;
				break;
			}
		}
		break;
	case CAM_REQ_MGR_MAP_BUF: {
		struct cam_mem_mgr_map_cmd cmd;

		if (k_ioctl->size != sizeof(cmd))
			return -EINVAL;

		if (copy_from_user(&cmd,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_mem_mgr_map_cmd))) {
			rc = -EFAULT;
			break;
		}

		rc = cam_mem_mgr_map(&cmd);
		if (!rc)
			if (copy_to_user(
				u64_to_user_ptr(k_ioctl->handle),
				&cmd, sizeof(struct cam_mem_mgr_map_cmd))) {
				rc = -EFAULT;
				break;
			}
		}
		break;
	case CAM_REQ_MGR_RELEASE_BUF: {
		struct cam_mem_mgr_release_cmd cmd;

		if (k_ioctl->size != sizeof(cmd))
			return -EINVAL;

		if (copy_from_user(&cmd,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_mem_mgr_release_cmd))) {
			rc = -EFAULT;
			break;
		}

		rc = cam_mem_mgr_release(&cmd);
		}
		break;
	case CAM_REQ_MGR_CACHE_OPS: {
		struct cam_mem_cache_ops_cmd cmd;

		if (k_ioctl->size != sizeof(cmd))
			return -EINVAL;

		if (copy_from_user(&cmd,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_mem_cache_ops_cmd))) {
			rc = -EFAULT;
			break;
		}

		rc = cam_mem_mgr_cache_ops(&cmd);
		if (rc)
			rc = -EINVAL;
		}
		break;
	case CAM_REQ_MGR_LINK_CONTROL: {
		struct cam_req_mgr_link_control cmd;

		if (k_ioctl->size != sizeof(cmd))
			return -EINVAL;

		if (copy_from_user(&cmd,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_req_mgr_link_control))) {
			rc = -EFAULT;
			break;
		}

		rc = cam_req_mgr_link_control(&cmd);
		if (rc)
			rc = -EINVAL;
		}
		break;
	case CAM_REQ_MGR_REQUEST_DUMP: {
		struct cam_dump_req_cmd cmd;

		if (k_ioctl->size != sizeof(cmd))
			return -EINVAL;

		if (copy_from_user(&cmd,
			u64_to_user_ptr(k_ioctl->handle),
			sizeof(struct cam_dump_req_cmd))) {
			rc = -EFAULT;
			break;
		}
		rc = cam_req_mgr_dump_request(&cmd);
		if (rc) {
			CAM_ERR(CAM_CORE, "dump fail for dev %d req %llu rc %d",
				cmd.dev_handle, cmd.issue_req_id, rc);
			break;
		}
		if (copy_to_user(
			u64_to_user_ptr(k_ioctl->handle),
			&cmd, sizeof(struct cam_dump_req_cmd)))
			rc = -EFAULT;
		}
		break;
	default:
		return -ENOIOCTLCMD;
	}

	return rc;
}

static const struct v4l2_ioctl_ops g_cam_ioctl_ops = {
	.vidioc_subscribe_event = cam_subscribe_event,
	.vidioc_unsubscribe_event = cam_unsubscribe_event,
	.vidioc_default = cam_private_ioctl,
};

static int cam_video_device_setup(void)
{
	int rc;

	g_dev.video = video_device_alloc();
	if (!g_dev.video) {
		rc = -ENOMEM;
		goto video_fail;
	}

	g_dev.video->v4l2_dev = g_dev.v4l2_dev;

	strlcpy(g_dev.video->name, "cam-req-mgr",
		sizeof(g_dev.video->name));
	g_dev.video->release = video_device_release_empty;
	g_dev.video->fops = &g_cam_fops;
	g_dev.video->ioctl_ops = &g_cam_ioctl_ops;
	g_dev.video->minor = -1;
	g_dev.video->vfl_type = VFL_TYPE_GRABBER;
	g_dev.video->device_caps |= V4L2_CAP_VIDEO_CAPTURE;
	rc = video_register_device(g_dev.video, VFL_TYPE_GRABBER, -1);
	if (rc) {
		CAM_ERR(CAM_CRM,
			"video device registration failure rc = %d, name = %s, device_caps = %d",
			rc, g_dev.video->name, g_dev.video->device_caps);
		goto v4l2_fail;
	}

	rc = media_entity_pads_init(&g_dev.video->entity, 0, NULL);
	if (rc)
		goto entity_fail;

	g_dev.video->entity.function = CAM_VNODE_DEVICE_TYPE;
	g_dev.video->entity.name = video_device_node_name(g_dev.video);

	return rc;

entity_fail:
	video_unregister_device(g_dev.video);
v4l2_fail:
	video_device_release(g_dev.video);
	g_dev.video = NULL;
video_fail:
	return rc;
}

int cam_req_mgr_notify_message(struct cam_req_mgr_message *msg,
	uint32_t id,
	uint32_t type)
{
	struct v4l2_event event;
	struct cam_req_mgr_message *ev_header;

	if (!msg)
		return -EINVAL;

	event.id = id;
	event.type = type;
	ev_header = CAM_REQ_MGR_GET_PAYLOAD_PTR(event,
		struct cam_req_mgr_message);
	memcpy(ev_header, msg, sizeof(struct cam_req_mgr_message));
	v4l2_event_queue(g_dev.video, &event);

	return 0;
}
EXPORT_SYMBOL(cam_req_mgr_notify_message);

void cam_video_device_cleanup(void)
{
	media_entity_cleanup(&g_dev.video->entity);
	video_unregister_device(g_dev.video);
	video_device_release(g_dev.video);
	g_dev.video = NULL;
}

void cam_subdev_notify_message(u32 subdev_type,
		enum cam_subdev_message_type_t message_type,
		uint32_t data)
{
	struct v4l2_subdev *sd = NULL;
	struct cam_subdev *csd = NULL;

	list_for_each_entry(sd, &g_dev.v4l2_dev->subdevs, list) {
		if (sd->entity.function == subdev_type) {
			csd = container_of(sd, struct cam_subdev, sd);
			if (csd->msg_cb != NULL)
				csd->msg_cb(sd, message_type, data);
		}
	}
}
EXPORT_SYMBOL(cam_subdev_notify_message);


static int cam_req_mgr_ordered_list_cmp(void *priv,
	struct list_head *head_1, struct list_head *head_2)
{
	struct cam_subdev *entry_1 =
		list_entry(head_1, struct cam_subdev, list);
	struct cam_subdev *entry_2 =
		list_entry(head_2, struct cam_subdev, list);
	int ret = -1;

	if (entry_1->close_seq_prior > entry_2->close_seq_prior)
		return 1;
	else if (entry_1->close_seq_prior < entry_2->close_seq_prior)
		return ret;
	else
		return 0;
}

bool cam_req_mgr_is_open(uint64_t dev_id)
{
	bool crm_status;
	bool dev_id_status;

	mutex_lock(&g_dev.cam_lock);
	crm_status = g_dev.open_cnt ? true : false;

	if (!g_dev.read_active_dev_id_hdls) {
		g_dev.active_dev_id_hdls = cam_get_dev_handle_status();
		g_dev.read_active_dev_id_hdls++;
	}

	dev_id_status = (g_dev.active_dev_id_hdls & dev_id) ? true : false;
	crm_status &=  dev_id_status;
	mutex_unlock(&g_dev.cam_lock);

	return crm_status;
}
EXPORT_SYMBOL(cam_req_mgr_is_open);

bool cam_req_mgr_is_shutdown(void)
{
	return g_dev.shutdown_state;
}
EXPORT_SYMBOL(cam_req_mgr_is_shutdown);

int cam_register_subdev(struct cam_subdev *csd)
{
	struct v4l2_subdev *sd;
	int rc;

	if (!g_dev.state) {
		CAM_DBG(CAM_CRM, "camera root device not ready yet");
		return -EPROBE_DEFER;
	}

	if (!csd || !csd->name) {
		CAM_ERR(CAM_CRM, "invalid arguments");
		return -EINVAL;
	}

	mutex_lock(&g_dev.dev_lock);

	sd = &csd->sd;
	v4l2_subdev_init(sd, csd->ops);
	sd->internal_ops = csd->internal_ops;
	snprintf(sd->name, V4L2_SUBDEV_NAME_SIZE, "%s", csd->name);
	v4l2_set_subdevdata(sd, csd->token);

	sd->flags = csd->sd_flags;
	sd->entity.num_pads = 0;
	sd->entity.pads = NULL;
	sd->entity.function = csd->ent_function;

	list_add(&csd->list, &cam_req_mgr_ordered_sd_list);
	list_sort(NULL, &cam_req_mgr_ordered_sd_list,
		cam_req_mgr_ordered_list_cmp);

	rc = v4l2_device_register_subdev(g_dev.v4l2_dev, sd);
	if (rc) {
		CAM_ERR(CAM_CRM, "register subdev failed");
		goto reg_fail;
	}

	rc = v4l2_device_register_subdev_nodes(g_dev.v4l2_dev);
	if (rc) {
		CAM_ERR(CAM_CRM, "Failed to register subdev node: %s, rc: %d",
			sd->name, rc);
		goto reg_fail;
	}

	if (sd->flags & V4L2_SUBDEV_FL_HAS_DEVNODE) {
		sd->entity.name = video_device_node_name(sd->devnode);
		CAM_DBG(CAM_CRM, "created node :%s", sd->entity.name);
	}

	g_dev.count++;

reg_fail:
	mutex_unlock(&g_dev.dev_lock);
	return rc;
}
EXPORT_SYMBOL(cam_register_subdev);

int cam_unregister_subdev(struct cam_subdev *csd)
{
	if (!g_dev.state) {
		CAM_ERR(CAM_CRM, "camera root device not ready yet");
		return -ENODEV;
	}

	mutex_lock(&g_dev.dev_lock);
	v4l2_device_unregister_subdev(&csd->sd);
	g_dev.count--;
	mutex_unlock(&g_dev.dev_lock);

	return 0;
}
EXPORT_SYMBOL(cam_unregister_subdev);

static int cam_req_mgr_component_master_bind(struct device *dev)
{
	int rc = 0;

	CAM_DBG(CAM_CRM, "Master bind called");
	rc = cam_v4l2_device_setup(dev);
	if (rc)
		return rc;

	rc = cam_media_device_setup(dev);
	if (rc)
		goto media_setup_fail;

	rc = cam_video_device_setup();
	if (rc)
		goto video_setup_fail;

	g_dev.open_cnt = 0;
	g_dev.shutdown_state = false;
	mutex_init(&g_dev.cam_lock);
	spin_lock_init(&g_dev.cam_eventq_lock);
	mutex_init(&g_dev.dev_lock);

	rc = cam_req_mgr_util_init();
	if (rc) {
		CAM_ERR(CAM_CRM, "cam req mgr util init is failed");
		goto req_mgr_util_fail;
	}

	rc = cam_req_mgr_core_device_init();
	if (rc) {
		CAM_ERR(CAM_CRM, "core device setup failed");
		goto req_mgr_core_fail;
	}

	g_dev.state = true;
	INIT_LIST_HEAD(&cam_req_mgr_ordered_sd_list);

	if (g_cam_req_mgr_timer_cachep == NULL) {
		g_cam_req_mgr_timer_cachep = kmem_cache_create("crm_timer",
			sizeof(struct cam_req_mgr_timer), 64,
			SLAB_CONSISTENCY_CHECKS | SLAB_RED_ZONE |
			SLAB_POISON | SLAB_STORE_USER, NULL);
		if (!g_cam_req_mgr_timer_cachep)
			CAM_ERR(CAM_CRM,
				"Failed to create kmem_cache for crm_timer");
		else
			CAM_DBG(CAM_CRM, "Name : %s",
				g_cam_req_mgr_timer_cachep->name);
	}

	CAM_INFO(CAM_CRM, "All probes done, binding slave components");
	rc = component_bind_all(dev, NULL);
	if (rc) {
		CAM_ERR(CAM_CRM,
			"Error in binding all components rc: %d, Camera initialization failed!",
			rc);
		goto req_mgr_device_deinit;
	}

	CAM_DBG(CAM_CRM, "All camera components bound successfully");
	rc = sysfs_create_file(&dev->kobj, &camera_debug_sysfs_attr.attr);
	if (rc < 0) {
		CAM_ERR(CAM_CPAS,
			"Failed to create debug attribute, rc=%d\n", rc);
		goto sysfs_fail;
	}

	return rc;

sysfs_fail:
	sysfs_remove_file(&dev->kobj, &camera_debug_sysfs_attr.attr);
req_mgr_device_deinit:
	cam_req_mgr_core_device_deinit();
req_mgr_core_fail:
	cam_req_mgr_util_deinit();
req_mgr_util_fail:
	mutex_destroy(&g_dev.dev_lock);
	mutex_destroy(&g_dev.cam_lock);
	cam_video_device_cleanup();
video_setup_fail:
	cam_media_device_cleanup();
media_setup_fail:
	cam_v4l2_device_cleanup();
	g_dev.state = false;
	return rc;
}

static void cam_req_mgr_component_master_unbind(struct device *dev)
{
	/* Unbinding all slave components first */
	component_unbind_all(dev, NULL);

	/* Now proceed with unbinding master */
	sysfs_remove_file(&dev->kobj, &camera_debug_sysfs_attr.attr);
	cam_req_mgr_core_device_deinit();
	cam_req_mgr_util_deinit();
	cam_media_device_cleanup();
	cam_video_device_cleanup();
	cam_v4l2_device_cleanup();
	mutex_destroy(&g_dev.dev_lock);
	g_dev.state = false;
}

static const struct component_master_ops cam_req_mgr_component_master_ops = {
	.bind = cam_req_mgr_component_master_bind,
	.unbind = cam_req_mgr_component_master_unbind,
};

static int cam_req_mgr_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &cam_req_mgr_component_master_ops);
	return 0;
}

static int cam_req_mgr_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct component_match *match_list = NULL;
	struct device *dev = &pdev->dev;

	rc = camera_component_match_add_drivers(dev, &match_list);
	if (rc) {
		CAM_ERR(CAM_CRM,
			"Unable to match components, probe failed rc: %d",
			rc);
		goto end;
	}

	/* Supply match_list to master for handing over control */
	rc = component_master_add_with_match(dev,
		&cam_req_mgr_component_master_ops, match_list);
	if (rc) {
		CAM_ERR(CAM_CRM,
			"Unable to add master, probe failed rc: %d",
			rc);
		goto end;
	}

end:
	return rc;
}

static const struct of_device_id cam_req_mgr_dt_match[] = {
	{.compatible = "qcom,cam-req-mgr"},
	{}
};
MODULE_DEVICE_TABLE(of, cam_req_mgr_dt_match);

struct platform_driver cam_req_mgr_driver = {
	.probe = cam_req_mgr_probe,
	.remove = cam_req_mgr_remove,
	.driver = {
		.name = "cam_req_mgr",
		.owner = THIS_MODULE,
		.of_match_table = cam_req_mgr_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_req_mgr_init(void)
{
	return platform_driver_register(&cam_req_mgr_driver);
}
EXPORT_SYMBOL(cam_req_mgr_init);

void cam_req_mgr_exit(void)
{
	platform_driver_unregister(&cam_req_mgr_driver);
}

MODULE_DESCRIPTION("Camera Request Manager");
MODULE_LICENSE("GPL v2");
