// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include "cam_subdev.h"
#include "cam_node.h"
#include "cam_lrme_context.h"
#include "cam_lrme_hw_mgr.h"
#include "cam_lrme_hw_mgr_intf.h"
#include "camera_main.h"

#define CAM_LRME_DEV_NAME "cam-lrme"

/**
 * struct cam_lrme_dev
 *
 * @sd       : Subdev information
 * @ctx      : List of base contexts
 * @lrme_ctx : List of LRME contexts
 * @lock     : Mutex for LRME subdev
 * @open_cnt : Open count of LRME subdev
 */
struct cam_lrme_dev {
	struct cam_subdev        sd;
	struct cam_context       ctx[CAM_CTX_MAX];
	struct cam_lrme_context  lrme_ctx[CAM_CTX_MAX];
	struct mutex             lock;
	uint32_t                 open_cnt;
};

static struct cam_lrme_dev *g_lrme_dev;

static int cam_lrme_dev_buf_done_cb(void *ctxt_to_hw_map, uint32_t evt_id,
	void *evt_data)
{
	uint64_t index;
	struct cam_context *ctx;
	int rc;

	index = CAM_LRME_DECODE_CTX_INDEX(ctxt_to_hw_map);
	CAM_DBG(CAM_LRME, "ctx index %llu, evt_id %u\n", index, evt_id);
	ctx = &g_lrme_dev->ctx[index];
	rc = ctx->irq_cb_intf(ctx, evt_id, evt_data);
	if (rc)
		CAM_ERR(CAM_LRME, "irq callback failed");

	return rc;
}

static int cam_lrme_dev_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_lrme_dev *lrme_dev = g_lrme_dev;

	cam_req_mgr_rwsem_read_op(CAM_SUBDEV_LOCK);

	if (!lrme_dev) {
		CAM_ERR(CAM_LRME,
			"LRME Dev not initialized, dev=%pK", lrme_dev);
		cam_req_mgr_rwsem_read_op(CAM_SUBDEV_UNLOCK);
		return -ENODEV;
	}

	mutex_lock(&lrme_dev->lock);
	lrme_dev->open_cnt++;
	mutex_unlock(&lrme_dev->lock);

	cam_req_mgr_rwsem_read_op(CAM_SUBDEV_UNLOCK);

	return 0;
}

int cam_lrme_dev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	int rc = 0;
	struct cam_lrme_dev *lrme_dev = g_lrme_dev;
	struct cam_node *node = v4l2_get_subdevdata(sd);

	if (!lrme_dev) {
		CAM_ERR(CAM_LRME, "Invalid args");
		return -ENODEV;
	}

	mutex_lock(&lrme_dev->lock);
	if (lrme_dev->open_cnt <= 0) {
		CAM_DBG(CAM_LRME, "LRME subdev is already closed");
		rc = -EINVAL;
		goto end;
	}

	lrme_dev->open_cnt--;
	if (!node) {
		CAM_ERR(CAM_LRME, "Node is NULL");
		rc = -EINVAL;
		goto end;
	}

	if (lrme_dev->open_cnt == 0)
		cam_node_shutdown(node);

end:
	mutex_unlock(&lrme_dev->lock);
	return rc;
}

static int cam_lrme_dev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open(CAM_LRME);

	if (crm_active) {
		CAM_DBG(CAM_LRME, "CRM is ACTIVE, close should be from CRM");
		return 0;
	}
	return cam_lrme_dev_close_internal(sd, fh);
}

static const struct v4l2_subdev_internal_ops cam_lrme_subdev_internal_ops = {
	.open = cam_lrme_dev_open,
	.close = cam_lrme_dev_close,
};

static int cam_lrme_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int rc;
	int i;
	struct cam_hw_mgr_intf hw_mgr_intf;
	struct cam_node *node;
	struct platform_device *pdev = to_platform_device(dev);

	g_lrme_dev = kzalloc(sizeof(struct cam_lrme_dev), GFP_KERNEL);
	if (!g_lrme_dev) {
		CAM_ERR(CAM_LRME, "No memory");
		return -ENOMEM;
	}
	g_lrme_dev->sd.internal_ops = &cam_lrme_subdev_internal_ops;
	g_lrme_dev->sd.close_seq_prior = CAM_SD_CLOSE_MEDIUM_PRIORITY;

	mutex_init(&g_lrme_dev->lock);

	rc = cam_subdev_probe(&g_lrme_dev->sd, pdev, CAM_LRME_DEV_NAME,
		CAM_LRME_DEVICE_TYPE);
	if (rc) {
		CAM_ERR(CAM_LRME, "LRME cam_subdev_probe failed");
		goto free_mem;
	}
	node = (struct cam_node *)g_lrme_dev->sd.token;

	rc = cam_lrme_hw_mgr_init(&hw_mgr_intf, cam_lrme_dev_buf_done_cb);
	if (rc) {
		CAM_ERR(CAM_LRME, "Can not initialized LRME HW manager");
		goto unregister;
	}

	for (i = 0; i < CAM_CTX_MAX; i++) {
		rc = cam_lrme_context_init(&g_lrme_dev->lrme_ctx[i],
				&g_lrme_dev->ctx[i],
				&node->hw_mgr_intf, i);
		if (rc) {
			CAM_ERR(CAM_LRME, "LRME context init failed");
			goto deinit_ctx;
		}
	}

	rc = cam_node_init(node, &hw_mgr_intf, g_lrme_dev->ctx, CAM_CTX_MAX,
		CAM_LRME_DEV_NAME);
	if (rc) {
		CAM_ERR(CAM_LRME, "LRME node init failed");
		goto deinit_ctx;
	}

	CAM_DBG(CAM_LRME, "Component bound successfully");

	return 0;

deinit_ctx:
	for (--i; i >= 0; i--) {
		if (cam_lrme_context_deinit(&g_lrme_dev->lrme_ctx[i]))
			CAM_ERR(CAM_LRME, "LRME context %d deinit failed", i);
	}
unregister:
	if (cam_subdev_remove(&g_lrme_dev->sd))
		CAM_ERR(CAM_LRME, "Failed in subdev remove");
free_mem:
	kfree(g_lrme_dev);

	return rc;
}

static void cam_lrme_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	int i;
	int rc = 0;

	for (i = 0; i < CAM_CTX_MAX; i++) {
		rc = cam_lrme_context_deinit(&g_lrme_dev->lrme_ctx[i]);
		if (rc)
			CAM_ERR(CAM_LRME, "LRME context %d deinit failed", i);
	}

	rc = cam_lrme_hw_mgr_deinit();
	if (rc)
		CAM_ERR(CAM_LRME, "Failed in hw mgr deinit, rc=%d", rc);

	rc = cam_subdev_remove(&g_lrme_dev->sd);
	if (rc)
		CAM_ERR(CAM_LRME, "Unregister failed rc: %d", rc);

	mutex_destroy(&g_lrme_dev->lock);
	kfree(g_lrme_dev);
	g_lrme_dev = NULL;
}

const static struct component_ops cam_lrme_component_ops = {
	.bind = cam_lrme_component_bind,
	.unbind = cam_lrme_component_unbind,
};

static int cam_lrme_dev_probe(struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_LRME, "Adding LRME component");
	rc = component_add(&pdev->dev, &cam_lrme_component_ops);
	if (rc)
		CAM_ERR(CAM_LRME, "failed to add component rc: %d", rc);

	return rc;
}

static int cam_lrme_dev_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_lrme_component_ops);
	return 0;
}

static const struct of_device_id cam_lrme_dt_match[] = {
	{
		.compatible = "qcom,cam-lrme"
	},
	{}
};

struct platform_driver cam_lrme_driver = {
	.probe = cam_lrme_dev_probe,
	.remove = cam_lrme_dev_remove,
	.driver = {
		.name = "cam_lrme",
		.owner = THIS_MODULE,
		.of_match_table = cam_lrme_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_lrme_dev_init_module(void)
{
	return platform_driver_register(&cam_lrme_driver);
}

void cam_lrme_dev_exit_module(void)
{
	platform_driver_unregister(&cam_lrme_driver);
}

MODULE_DESCRIPTION("MSM LRME driver");
MODULE_LICENSE("GPL v2");
