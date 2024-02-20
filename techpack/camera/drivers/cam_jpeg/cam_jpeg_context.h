/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#ifndef _CAM_JPEG_CONTEXT_H_
#define _CAM_JPEG_CONTEXT_H_

#include <media/cam_jpeg.h>

#include "cam_context.h"
#include "cam_jpeg_hw_mgr_intf.h"

#define CAM_JPEG_HW_EVENT_MAX 20

/**
 * struct cam_jpeg_context - Jpeg context
 * @base: Base jpeg cam context object
 * @req_base: Common request structure
 */
struct cam_jpeg_context {
	struct cam_context *base;
	struct cam_ctx_request req_base[CAM_CTX_REQ_MAX];
};

/* cam jpeg context irq handling function type */
typedef int (*cam_jpeg_hw_event_cb_func)(
	struct cam_jpeg_context *ctx_jpeg,
	void *evt_data);

/**
 * struct cam_jpeg_ctx_irq_ops - Function table for handling IRQ callbacks
 *
 * @irq_ops: Array of handle function pointers.
 *
 */
struct cam_jpeg_ctx_irq_ops {
	cam_jpeg_hw_event_cb_func irq_ops[CAM_JPEG_HW_EVENT_MAX];
};

/**
 * cam_jpeg_context_init()
 *
 * @brief: Initialization function for the JPEG context
 *
 * @ctx: JPEG context obj to be initialized
 * @ctx_base: Context base from cam_context
 * @hw_intf: JPEG hw manager interface
 * @ctx_id: ID for this context
 *
 */
int cam_jpeg_context_init(struct cam_jpeg_context *ctx,
	struct cam_context *ctx_base,
	struct cam_hw_mgr_intf *hw_intf,
	uint32_t ctx_id);

/**
 * cam_jpeg_context_deinit()
 *
 * @brief: Deinitialize function for the JPEG context
 *
 * @ctx: JPEG context obj to be deinitialized
 *
 */
int cam_jpeg_context_deinit(struct cam_jpeg_context *ctx);

/**
 * cam_jpeg_subdev_close_internal()
 *
 * @brief: Close function for the jpeg dev
 *
 * @sd: Pointer to struct v4l2_subdev
 * @fh: Pointer to struct v4l2_subdev_fh
 *
 */
int cam_jpeg_subdev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh);

#endif  /* __CAM_JPEG_CONTEXT_H__ */
