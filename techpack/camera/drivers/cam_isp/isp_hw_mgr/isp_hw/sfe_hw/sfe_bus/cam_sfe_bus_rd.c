// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/ratelimit.h>
#include <linux/slab.h>

#include <media/cam_isp.h>

#include "cam_io_util.h"
#include "cam_debug_util.h"
#include "cam_hw_intf.h"
#include "cam_ife_hw_mgr.h"
#include "cam_sfe_hw_intf.h"
#include "cam_tasklet_util.h"
#include "cam_sfe_bus.h"
#include "cam_sfe_bus_rd.h"
#include "cam_sfe_core.h"
#include "cam_debug_util.h"
#include "cam_cpas_api.h"

static const char drv_name[] = "sfe_bus_rd";

#define MAX_BUF_UPDATE_REG_NUM   \
	(sizeof(struct cam_sfe_bus_rd_reg_offset_bus_client)/4)

#define MAX_REG_VAL_PAIR_SIZE    \
	(MAX_BUF_UPDATE_REG_NUM * 2 * CAM_PACKET_MAX_PLANES)

#define BUS_RD_DEFAULT_LATENCY_BUF_ALLOC   512
#define CAM_SFE_BUS_RD_PAYLOAD_MAX         16

static uint32_t bus_rd_error_irq_mask[1] = {
	0x00000001,
};

enum cam_sfe_bus_rd_unpacker_format {
	BUS_RD_UNPACKER_FMT_PLAIN_128             = 0x0,
	BUS_RD_UNPACKER_FMT_PLAIN_8               = 0x1,
	BUS_RD_UNPACKER_FMT_PLAIN_16_10BPP        = 0x2,
	BUS_RD_UNPACKER_FMT_PLAIN_16_12BPP        = 0x3,
	BUS_RD_UNPACKER_FMT_PLAIN_16_14BPP        = 0x4,
	BUS_RD_UNPACKER_FMT_PLAIN_32_20BPP        = 0x5,
	BUS_RD_UNPACKER_FMT_ARGB_10               = 0x6,
	BUS_RD_UNPACKER_FMT_ARGB_12               = 0x7,
	BUS_RD_UNPACKER_FMT_ARGB_14               = 0x8,
	BUS_RD_UNPACKER_FMT_PLAIN_32              = 0x9,
	BUS_RD_UNPACKER_FMT_PLAIN_64              = 0xA,
	BUS_RD_UNPACKER_FMT_TP_10                 = 0xB,
	BUS_RD_UNPACKER_FMT_MIPI8                 = 0xC,
	BUS_RD_UNPACKER_FMT_MIPI10                = 0xD,
	BUS_RD_UNPACKER_FMT_MIPI12                = 0xE,
	BUS_RD_UNPACKER_FMT_MIPI14                = 0xF,
	BUS_RD_UNPACKER_FMT_PLAIN_16_16BPP        = 0x10,
	BUS_RD_UNPACKER_FMT_PLAIN_128_ODD_EVEN    = 0x11,
	BUS_RD_UNPACKER_FMT_PLAIN_8_ODD_EVEN      = 0x12,
	BUS_RD_UNPACKER_FMT_MAX                   = 0x13,
};

struct cam_sfe_bus_rd_common_data {
	uint32_t                                    core_index;
	void __iomem                               *mem_base;
	struct cam_hw_intf                         *hw_intf;
	struct cam_sfe_bus_rd_reg_offset_common    *common_reg;
	uint32_t                                    io_buf_update[
		MAX_REG_VAL_PAIR_SIZE];
	void                                       *bus_irq_controller;
	void                                       *sfe_irq_controller;
	spinlock_t                                  spin_lock;
	struct list_head                            free_payload_list;
	struct cam_sfe_bus_rd_irq_evt_payload       evt_payload[
		CAM_SFE_BUS_RD_PAYLOAD_MAX];
	cam_hw_mgr_event_cb_func                    event_cb;
};

struct cam_sfe_bus_rd_rm_resource_data {
	uint32_t             index;
	struct cam_sfe_bus_rd_common_data            *common_data;
	struct cam_sfe_bus_rd_reg_offset_bus_client  *hw_regs;
	void                *ctx;
	uint32_t             min_vbi;
	uint32_t             fs_mode;
	uint32_t             hbi_count;
	uint32_t             width;
	uint32_t             height;
	uint32_t             stride;
	uint32_t             format;
	uint32_t             latency_buf_allocation;
	uint32_t             unpacker_cfg;
	uint32_t             burst_len;
	uint32_t             en_cfg;
	uint32_t             img_addr;
	uint32_t             input_if_cmd;
};

struct cam_sfe_bus_rd_data {
	uint32_t                           bus_rd_type;
	struct cam_sfe_bus_rd_common_data *common_data;

	uint32_t                           num_rm;
	struct cam_isp_resource_node      *rm_res[PLANE_MAX];
	uint32_t                           format;
	uint32_t                           max_width;
	uint32_t                           max_height;
	struct cam_cdm_utils_ops          *cdm_util_ops;
	void                              *priv;
	uint32_t                           secure_mode;
	uint32_t                           fs_sync_enable;
	uint32_t                           fs_line_sync_en;
	bool                               is_offline;
};

struct cam_sfe_bus_rd_priv {
	struct cam_sfe_bus_rd_common_data   common_data;
	uint32_t                            num_client;
	uint32_t                            num_bus_rd_resc;

	struct cam_isp_resource_node  bus_client[
		CAM_SFE_BUS_RD_MAX_CLIENTS];
	struct cam_isp_resource_node  sfe_bus_rd[
		CAM_SFE_BUS_RD_MAX];

	int                                 irq_handle;
	int                                 error_irq_handle;
	void                               *tasklet_info;
	uint32_t                            top_irq_shift;
};

static void cam_sfe_bus_rd_pxls_to_bytes(uint32_t pxls, uint32_t fmt,
	uint32_t *bytes)
{
	switch (fmt) {
	case BUS_RD_UNPACKER_FMT_PLAIN_128:
		*bytes = pxls * 16;
		break;
	case BUS_RD_UNPACKER_FMT_PLAIN_8:
	case BUS_RD_UNPACKER_FMT_PLAIN_8_ODD_EVEN:
	case BUS_RD_UNPACKER_FMT_PLAIN_16_10BPP:
	case BUS_RD_UNPACKER_FMT_PLAIN_16_12BPP:
	case BUS_RD_UNPACKER_FMT_PLAIN_16_14BPP:
	case BUS_RD_UNPACKER_FMT_PLAIN_16_16BPP:
	case BUS_RD_UNPACKER_FMT_ARGB_10:
	case BUS_RD_UNPACKER_FMT_ARGB_12:
	case BUS_RD_UNPACKER_FMT_ARGB_14:
		*bytes = pxls * 2;
		break;
	case BUS_RD_UNPACKER_FMT_PLAIN_32_20BPP:
	case BUS_RD_UNPACKER_FMT_PLAIN_32:
		*bytes = pxls * 4;
		break;
	case BUS_RD_UNPACKER_FMT_PLAIN_64:
		*bytes = pxls * 8;
		break;
	case BUS_RD_UNPACKER_FMT_TP_10:
		*bytes = ALIGNUP(pxls, 3) * 4 / 3;
		break;
	case BUS_RD_UNPACKER_FMT_MIPI8:
		*bytes = pxls;
		break;
	case BUS_RD_UNPACKER_FMT_MIPI10:
		*bytes = ALIGNUP(pxls * 5, 4) / 4;
		break;
	case BUS_RD_UNPACKER_FMT_MIPI12:
		*bytes = ALIGNUP(pxls * 3, 2) / 2;
		break;
	case BUS_RD_UNPACKER_FMT_MIPI14:
		*bytes = ALIGNUP(pxls * 7, 4) / 4;
		break;
	default:
		CAM_ERR(CAM_SFE, "Invalid unpacker_fmt:%d", fmt);
		break;
	}
}

static enum cam_sfe_bus_rd_unpacker_format
	cam_sfe_bus_get_unpacker_fmt(uint32_t unpack_fmt)
{
	switch (unpack_fmt) {
	case CAM_FORMAT_PLAIN128:
		return BUS_RD_UNPACKER_FMT_PLAIN_128;
	case CAM_FORMAT_PLAIN8:
		return BUS_RD_UNPACKER_FMT_PLAIN_8;
	case CAM_FORMAT_PLAIN16_10:
		return BUS_RD_UNPACKER_FMT_PLAIN_16_10BPP;
	case CAM_FORMAT_PLAIN16_12:
		return BUS_RD_UNPACKER_FMT_PLAIN_16_12BPP;
	case CAM_FORMAT_PLAIN16_14:
		return BUS_RD_UNPACKER_FMT_PLAIN_16_14BPP;
	case CAM_FORMAT_PLAIN32_20:
		return BUS_RD_UNPACKER_FMT_PLAIN_32_20BPP;
	case CAM_FORMAT_ARGB_10:
		return BUS_RD_UNPACKER_FMT_ARGB_10;
	case CAM_FORMAT_ARGB_12:
		return BUS_RD_UNPACKER_FMT_ARGB_12;
	case CAM_FORMAT_ARGB_14:
		return BUS_RD_UNPACKER_FMT_ARGB_14;
	case CAM_FORMAT_PLAIN32:
	case CAM_FORMAT_ARGB:
		return BUS_RD_UNPACKER_FMT_PLAIN_32;
	case CAM_FORMAT_PLAIN64:
	case CAM_FORMAT_ARGB_16:
	case CAM_FORMAT_PD10:
		return BUS_RD_UNPACKER_FMT_PLAIN_64;
	case CAM_FORMAT_TP10:
		return BUS_RD_UNPACKER_FMT_TP_10;
	case CAM_FORMAT_MIPI_RAW_8:
		return BUS_RD_UNPACKER_FMT_MIPI8;
	case CAM_FORMAT_MIPI_RAW_10:
		return BUS_RD_UNPACKER_FMT_MIPI10;
	case CAM_FORMAT_MIPI_RAW_12:
		return BUS_RD_UNPACKER_FMT_MIPI12;
	case CAM_FORMAT_MIPI_RAW_14:
		return BUS_RD_UNPACKER_FMT_MIPI14;
	case CAM_FORMAT_PLAIN16_16:
		return BUS_RD_UNPACKER_FMT_PLAIN_16_16BPP;
	case CAM_FORMAT_PLAIN8_SWAP:
		return BUS_RD_UNPACKER_FMT_PLAIN_8_ODD_EVEN;
	default:
		return BUS_RD_UNPACKER_FMT_MAX;
	}
}

static enum cam_sfe_bus_rd_type
	cam_sfe_bus_get_bus_rd_res_id(uint32_t res_type)
{
	switch (res_type) {
	case CAM_ISP_HW_SFE_IN_RD0:
		return CAM_SFE_BUS_RD_RDI0;
	case CAM_ISP_HW_SFE_IN_RD1:
		return CAM_SFE_BUS_RD_RDI1;
	case CAM_ISP_HW_SFE_IN_RD2:
		return CAM_SFE_BUS_RD_RDI2;
	default:
		return CAM_SFE_BUS_RD_MAX;
	}
}

static int cam_sfe_bus_get_num_rm(
	enum cam_sfe_bus_rd_type res_type)
{
	switch (res_type) {
	case CAM_SFE_BUS_RD_RDI0:
	case CAM_SFE_BUS_RD_RDI1:
	case CAM_SFE_BUS_RD_RDI2:
		return 1;
	default:
		CAM_ERR(CAM_SFE, "Unsupported resource_type %u", res_type);
		return -EINVAL;
	}
}

static int cam_sfe_bus_get_rm_idx(
	enum cam_sfe_bus_rd_type    sfe_bus_rd_res_id,
	enum cam_sfe_bus_plane_type plane)
{
	int rm_idx = -1;

	switch (sfe_bus_rd_res_id) {
	case CAM_SFE_BUS_RD_RDI0:
		switch (plane) {
		case PLANE_Y:
			rm_idx = 0;
			break;
		default:
			break;
		}
		break;
	case CAM_SFE_BUS_RD_RDI1:
		switch (plane) {
		case PLANE_Y:
			rm_idx = 1;
			break;
		default:
			break;
		}
		break;
	case CAM_SFE_BUS_RD_RDI2:
		switch (plane) {
		case PLANE_Y:
			rm_idx = 2;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return rm_idx;
}

static int cam_sfe_bus_rd_get_evt_payload(
	struct cam_sfe_bus_rd_common_data      *common_data,
	struct cam_sfe_bus_rd_irq_evt_payload **evt_payload)
{
	int rc = 0;

	spin_lock(&common_data->spin_lock);

	if (list_empty(&common_data->free_payload_list)) {
		CAM_ERR_RATE_LIMIT(CAM_SFE,
			"No free BUS RD event payload");
		*evt_payload = NULL;
		rc = -ENODEV;
		goto done;
	}

	*evt_payload = list_first_entry(&common_data->free_payload_list,
		struct cam_sfe_bus_rd_irq_evt_payload, list);
	list_del_init(&(*evt_payload)->list);

done:
	spin_unlock(&common_data->spin_lock);
	return rc;
}

static int cam_sfe_bus_rd_put_evt_payload(
	struct cam_sfe_bus_rd_common_data       *common_data,
	struct cam_sfe_bus_rd_irq_evt_payload  **evt_payload)
{
	unsigned long flags;

	if (!common_data) {
		CAM_ERR(CAM_SFE, "Invalid param common_data NULL");
		return -EINVAL;
	}

	if (*evt_payload == NULL) {
		CAM_ERR(CAM_SFE, "No payload to put");
		return -EINVAL;
	}

	spin_lock_irqsave(&common_data->spin_lock, flags);
		list_add_tail(&(*evt_payload)->list,
			&common_data->free_payload_list);
	spin_unlock_irqrestore(&common_data->spin_lock, flags);

	*evt_payload = NULL;

	return 0;
}

static int cam_sfe_bus_rd_handle_irq(
	uint32_t    evt_id,
	struct cam_irq_th_payload *th_payload)
{
	struct cam_sfe_bus_rd_priv *bus_priv;
	int rc = 0;

	bus_priv = th_payload->handler_priv;
	CAM_DBG(CAM_SFE, "Top Bus RD IRQ Received");

	rc = cam_irq_controller_handle_irq(evt_id,
		bus_priv->common_data.bus_irq_controller);

	return (rc == IRQ_HANDLED) ? 0 : -EINVAL;
}

static int cam_sfe_bus_acquire_rm(
	struct cam_sfe_bus_rd_priv             *bus_rd_priv,
	void                                   *tasklet,
	void                                   *ctx,
	enum cam_sfe_bus_rd_type                sfe_bus_rd_res_id,
	enum cam_sfe_bus_plane_type             plane,
	struct cam_isp_resource_node          **rm_res,
	uint32_t                                unpacker_fmt)
{
	uint32_t                                rm_idx = 0;
	struct cam_isp_resource_node           *rm_res_local = NULL;
	struct cam_sfe_bus_rd_rm_resource_data *rsrc_data = NULL;

	*rm_res = NULL;

	rm_idx = cam_sfe_bus_get_rm_idx(sfe_bus_rd_res_id, plane);
	if (rm_idx < 0 || rm_idx >= bus_rd_priv->num_client) {
		CAM_ERR(CAM_SFE, "Unsupported SFE RM:%d plane:%d",
			sfe_bus_rd_res_id, plane);
		return -EINVAL;
	}

	rm_res_local = &bus_rd_priv->bus_client[rm_idx];
	if (rm_res_local->res_state != CAM_ISP_RESOURCE_STATE_AVAILABLE) {
		CAM_ERR(CAM_SFE, "SFE:%d RM:%d res not available state:%d",
			bus_rd_priv->common_data.core_index, rm_idx,
			rm_res_local->res_state);
		return -EALREADY;
	}
	rm_res_local->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
	rm_res_local->tasklet_info = tasklet;

	rsrc_data = rm_res_local->res_priv;
	rsrc_data->ctx = ctx;
	rsrc_data->unpacker_cfg =
		cam_sfe_bus_get_unpacker_fmt(unpacker_fmt);
	rsrc_data->latency_buf_allocation =
		BUS_RD_DEFAULT_LATENCY_BUF_ALLOC;

	*rm_res = rm_res_local;

	CAM_DBG(CAM_SFE, "SFE:%d RM:%d Acquired",
		rsrc_data->common_data->core_index, rsrc_data->index);
	return 0;
}

static int cam_sfe_bus_release_rm(void          *bus_priv,
	struct cam_isp_resource_node                *rm_res)
{
	struct cam_sfe_bus_rd_rm_resource_data *rsrc_data =
		rm_res->res_priv;

	rsrc_data->width = 0;
	rsrc_data->height = 0;
	rsrc_data->stride = 0;
	rsrc_data->format = 0;
	rsrc_data->unpacker_cfg = 0;
	rsrc_data->burst_len = 0;
	rsrc_data->en_cfg = 0;

	rm_res->tasklet_info = NULL;
	rm_res->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;

	CAM_DBG(CAM_SFE, "SFE:%d RM:%d released",
		rsrc_data->common_data->core_index, rsrc_data->index);
	return 0;
}

static int cam_sfe_bus_start_rm(struct cam_isp_resource_node *rm_res)
{
	struct cam_sfe_bus_rd_rm_resource_data  *rm_data;
	struct cam_sfe_bus_rd_common_data       *common_data;

	rm_data = rm_res->res_priv;
	common_data = rm_data->common_data;

	cam_io_w_mb(rm_data->width, common_data->mem_base +
		rm_data->hw_regs->buf_width);
	cam_io_w_mb(rm_data->height, common_data->mem_base +
		rm_data->hw_regs->buf_height);
	cam_io_w_mb(rm_data->width, common_data->mem_base +
		rm_data->hw_regs->stride);
	cam_io_w_mb(rm_data->unpacker_cfg, common_data->mem_base +
		rm_data->hw_regs->unpacker_cfg);
	cam_io_w_mb(rm_data->latency_buf_allocation, common_data->mem_base +
		rm_data->hw_regs->latency_buf_allocation);
	cam_io_w_mb(0x1, common_data->mem_base + rm_data->hw_regs->cfg);

	rm_res->res_state = CAM_ISP_RESOURCE_STATE_STREAMING;

	CAM_DBG(CAM_SFE,
		"Start SFE:%d RM:%d offset:0x%X en_cfg:0x%X width:%d height:%d",
		rm_data->common_data->core_index, rm_data->index,
		(uint32_t) rm_data->hw_regs->cfg, rm_data->en_cfg,
		rm_data->width, rm_data->height);
	CAM_DBG(CAM_SFE, "RM:%d pk_fmt:%d stride:%d", rm_data->index,
		rm_data->unpacker_cfg, rm_data->stride);

	return 0;
}

static int cam_sfe_bus_stop_rm(struct cam_isp_resource_node *rm_res)
{
	struct cam_sfe_bus_rd_rm_resource_data *rsrc_data =
		rm_res->res_priv;
	struct cam_sfe_bus_rd_common_data      *common_data =
		rsrc_data->common_data;

	/* Disable RM */
	cam_io_w_mb(0x0, common_data->mem_base + rsrc_data->hw_regs->cfg);

	rm_res->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;

	CAM_DBG(CAM_SFE, "SFE:%d RM:%d stopped",
		rsrc_data->common_data->core_index, rsrc_data->index);

	return 0;
}

static int cam_sfe_bus_init_rm_resource(uint32_t index,
	struct cam_sfe_bus_rd_priv     *bus_rd_priv,
	struct cam_sfe_bus_rd_hw_info  *bus_rd_hw_info,
	struct cam_isp_resource_node   *rm_res)
{
	struct cam_sfe_bus_rd_rm_resource_data *rsrc_data;

	rsrc_data = kzalloc(sizeof(struct cam_sfe_bus_rd_rm_resource_data),
		GFP_KERNEL);
	if (!rsrc_data) {
		CAM_DBG(CAM_SFE, "Failed to alloc SFE:%d RM res priv",
			bus_rd_priv->common_data.core_index);
		return -ENOMEM;
	}
	rm_res->res_priv = rsrc_data;

	rsrc_data->index = index;
	rsrc_data->hw_regs = &bus_rd_hw_info->bus_client_reg[index];
	rsrc_data->common_data = &bus_rd_priv->common_data;

	rm_res->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;
	INIT_LIST_HEAD(&rm_res->list);

	rm_res->start = cam_sfe_bus_start_rm;
	rm_res->stop = cam_sfe_bus_stop_rm;
	rm_res->hw_intf = bus_rd_priv->common_data.hw_intf;

	return 0;
}

static int cam_sfe_bus_deinit_rm_resource(
	struct cam_isp_resource_node    *rm_res)
{
	struct cam_sfe_bus_rd_rm_resource_data *rsrc_data;

	rm_res->res_state = CAM_ISP_RESOURCE_STATE_UNAVAILABLE;
	INIT_LIST_HEAD(&rm_res->list);

	rm_res->start = NULL;
	rm_res->stop = NULL;
	rm_res->top_half_handler = NULL;
	rm_res->bottom_half_handler = NULL;
	rm_res->hw_intf = NULL;

	rsrc_data = rm_res->res_priv;
	rm_res->res_priv = NULL;
	if (!rsrc_data)
		return -ENOMEM;
	kfree(rsrc_data);

	return 0;
}

static int cam_sfe_bus_rd_handle_irq_top_half(uint32_t evt_id,
	struct cam_irq_th_payload *th_payload)
{
	int i, rc = 0;
	struct cam_sfe_bus_rd_priv *bus_priv;
	struct cam_sfe_bus_rd_irq_evt_payload *evt_payload = NULL;

	bus_priv = th_payload->handler_priv;
	rc  = cam_sfe_bus_rd_get_evt_payload(
		&bus_priv->common_data, &evt_payload);
	if (rc)
		return rc;

	for (i = 0; i < th_payload->num_registers; i++) {
		evt_payload->irq_reg_val[i] =
			th_payload->evt_status_arr[i];
		CAM_DBG(CAM_SFE, "SFE:%d Bus RD IRQ status_%d: 0x%x",
			bus_priv->common_data.core_index, i,
			th_payload->evt_status_arr[i]);
	}

	evt_payload->constraint_violation = cam_io_r_mb(
		bus_priv->common_data.mem_base +
		bus_priv->common_data.common_reg->cons_violation_status);
	if (evt_payload->constraint_violation) {
		CAM_ERR(CAM_SFE, "SFE:%d constraint violation:0x%x",
			bus_priv->common_data.core_index,
			evt_payload->constraint_violation);
		cam_irq_controller_disable_irq(
			bus_priv->common_data.bus_irq_controller,
			 bus_priv->error_irq_handle);
		cam_irq_controller_clear_and_mask(evt_id,
			bus_priv->common_data.bus_irq_controller);
	}

	cam_isp_hw_get_timestamp(&evt_payload->ts);
	th_payload->evt_payload_priv = evt_payload;
	return 0;
}

static int cam_sfe_bus_rd_handle_irq_bottom_half(
	void  *handler_priv, void *evt_payload_priv)
{
	struct cam_sfe_bus_rd_irq_evt_payload *evt_payload;
	struct cam_sfe_bus_rd_priv            *bus_priv;
	struct cam_sfe_bus_rd_common_data     *common_data = NULL;
	struct cam_isp_hw_event_info           evt_info;
	uint32_t status = 0, constraint_violation = 0;

	if (!handler_priv || !evt_payload_priv)
		return -EINVAL;

	bus_priv = (struct cam_sfe_bus_rd_priv *)handler_priv;
	evt_payload = (struct cam_sfe_bus_rd_irq_evt_payload *)evt_payload_priv;
	common_data = &bus_priv->common_data;
	status = evt_payload->irq_reg_val[CAM_SFE_IRQ_BUS_RD_REG_STATUS0];
	constraint_violation = evt_payload->constraint_violation;
	cam_sfe_bus_rd_put_evt_payload(common_data, &evt_payload);

	if (status & 0x2)
		CAM_DBG(CAM_SFE, "Received SFE:%d BUS RD RUP",
			bus_priv->common_data.core_index);

	if (status & 0x4)
		CAM_DBG(CAM_SFE, "Received SFE:%d BUS RD0 BUF DONE",
			bus_priv->common_data.core_index);

	if (status & 0x8)
		CAM_DBG(CAM_SFE, "Received SFE:%d BUS RD1 BUF DONE",
			bus_priv->common_data.core_index);

	if (status & 0x10)
		CAM_DBG(CAM_SFE, "Received SFE:%d BUS RD2 BUF DONE",
			bus_priv->common_data.core_index);

	if (status & 0x1) {
		CAM_ERR(CAM_SFE, "SFE:%d Constraint violation status:0x%x",
			bus_priv->common_data.core_index,
			constraint_violation);

		evt_info.hw_idx = bus_priv->common_data.core_index;
		evt_info.res_type = CAM_ISP_RESOURCE_SFE_RD;
		evt_info.res_id = CAM_SFE_BUS_RD_MAX;
		evt_info.err_type = CAM_SFE_IRQ_STATUS_VIOLATION;

		if (common_data->event_cb)
			common_data->event_cb(NULL,
				CAM_ISP_HW_EVENT_ERROR, (void *)&evt_info);
	}

	return 0;
}

static int cam_sfe_bus_rd_get_secure_mode(void *priv, void *cmd_args,
	uint32_t arg_size)
{
	struct cam_isp_hw_get_cmd_update      *secure_mode = cmd_args;
	struct cam_sfe_bus_rd_data            *rsrc_data = NULL;
	uint32_t                              *mode;

	rsrc_data = (struct cam_sfe_bus_rd_data *)
		secure_mode->res->res_priv;
	mode = (uint32_t *)secure_mode->data;
	*mode = (rsrc_data->secure_mode == CAM_SECURE_MODE_SECURE) ?
		true : false;

	return 0;
}

static int cam_sfe_bus_acquire_bus_rd(void *bus_priv, void *acquire_args,
	uint32_t args_size)
{
	int                                           rc = -ENODEV;
	int                                           i;
	enum cam_sfe_bus_rd_type                      bus_rd_res_id;
	int                                           num_rm;
	struct cam_sfe_bus_rd_priv                   *bus_rd_priv =
		bus_priv;
	struct cam_sfe_acquire_args                  *acq_args = acquire_args;
	struct cam_sfe_hw_sfe_bus_rd_acquire_args    *bus_rd_acquire_args;
	struct cam_isp_resource_node                 *rsrc_node = NULL;
	struct cam_sfe_bus_rd_data                   *rsrc_data = NULL;

	if (!bus_priv || !acquire_args) {
		CAM_ERR(CAM_SFE, "Invalid Param");
		return -EINVAL;
	}

	bus_rd_acquire_args = &acq_args->sfe_rd;

	bus_rd_res_id = cam_sfe_bus_get_bus_rd_res_id(
		bus_rd_acquire_args->res_id);
	if (bus_rd_res_id == CAM_SFE_BUS_RD_MAX)
		return -ENODEV;

	num_rm = cam_sfe_bus_get_num_rm(bus_rd_res_id);
	if (num_rm < 1)
		return -EINVAL;

	rsrc_node = &bus_rd_priv->sfe_bus_rd[bus_rd_res_id];
	if (rsrc_node->res_state != CAM_ISP_RESOURCE_STATE_AVAILABLE) {
		CAM_ERR(CAM_SFE, "SFE:%d RM:0x%x not available state:%d",
			bus_rd_priv->common_data.core_index,
			acq_args->rsrc_type, rsrc_node->res_state);
		return -EBUSY;
	}

	rsrc_node->res_id = bus_rd_acquire_args->res_id;
	rsrc_data = rsrc_node->res_priv;

	CAM_DBG(CAM_SFE, "SFE:%d acquire RD type:0x%x",
		rsrc_data->common_data->core_index, acq_args->rsrc_type);

	rsrc_data->num_rm = num_rm;
	rsrc_node->tasklet_info = acq_args->tasklet;
	bus_rd_priv->tasklet_info = acq_args->tasklet;
	rsrc_node->cdm_ops = bus_rd_acquire_args->cdm_ops;
	rsrc_data->cdm_util_ops = bus_rd_acquire_args->cdm_ops;
	rsrc_data->common_data->event_cb = acq_args->event_cb;
	rsrc_data->priv = acq_args->priv;
	rsrc_data->is_offline = bus_rd_acquire_args->is_offline;
	if (!rsrc_data->secure_mode) {
		rsrc_data->secure_mode =
			bus_rd_acquire_args->secure_mode;
	} else if (rsrc_data->secure_mode !=
		bus_rd_acquire_args->secure_mode) {
		CAM_ERR(CAM_SFE,
			"Current Mode %u Requesting Mode %u",
			rsrc_data->secure_mode,
			bus_rd_acquire_args->secure_mode);
		return -EINVAL;
	}

	for (i = 0; i < num_rm; i++) {
		rc = cam_sfe_bus_acquire_rm(bus_rd_priv,
			acq_args->tasklet,
			acq_args->priv,
			bus_rd_res_id,
			i,
			&rsrc_data->rm_res[i],
			bus_rd_acquire_args->unpacker_fmt);
		if (rc) {
			CAM_ERR(CAM_SFE,
				"SFE:%d RM:%d acquire failed rc:%d",
				rsrc_data->common_data->core_index,
				bus_rd_res_id, rc);
			goto release_rm;
		}
	}

	rsrc_node->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;
	bus_rd_acquire_args->rsrc_node = rsrc_node;

	CAM_DBG(CAM_SFE, "SFE:%d acquire RD 0x%x successful",
		rsrc_data->common_data->core_index, acq_args->rsrc_type);
	return rc;

release_rm:
	for (i--; i >= 0; i--)
		cam_sfe_bus_release_rm(bus_rd_priv, rsrc_data->rm_res[i]);
	return rc;
}

static int cam_sfe_bus_release_bus_rd(void *bus_priv, void *release_args,
	uint32_t args_size)
{
	uint32_t i;
	struct cam_isp_resource_node   *sfe_bus_rd = NULL;
	struct cam_sfe_bus_rd_data     *rsrc_data = NULL;

	if (!bus_priv || !release_args) {
		CAM_ERR(CAM_SFE, "Invalid input bus_priv %pK release_args %pK",
			bus_priv, release_args);
		return -EINVAL;
	}

	sfe_bus_rd = release_args;
	rsrc_data = sfe_bus_rd->res_priv;

	if (sfe_bus_rd->res_state != CAM_ISP_RESOURCE_STATE_RESERVED) {
		CAM_ERR(CAM_SFE,
			"SFE:%d RD type:0x%x invalid resource state:%d",
			rsrc_data->common_data->core_index,
			sfe_bus_rd->res_id, sfe_bus_rd->res_state);
	}

	for (i = 0; i < rsrc_data->num_rm; i++)
		cam_sfe_bus_release_rm(bus_priv, rsrc_data->rm_res[i]);
	rsrc_data->num_rm = 0;

	sfe_bus_rd->tasklet_info = NULL;
	sfe_bus_rd->cdm_ops = NULL;
	rsrc_data->cdm_util_ops = NULL;
	rsrc_data->secure_mode = 0;

	if (sfe_bus_rd->res_state == CAM_ISP_RESOURCE_STATE_RESERVED)
		sfe_bus_rd->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;

	return 0;
}

static int cam_sfe_bus_start_bus_rd(
	void *hw_priv, void *stop_hw_args, uint32_t arg_size)
{
	int rc = -ENODEV, i;
	struct cam_isp_resource_node *sfe_bus_rd = NULL;
	struct cam_sfe_bus_rd_data *rsrc_data = NULL;
	struct cam_sfe_bus_rd_common_data *common_data = NULL;
	uint32_t val = 0;

	if (!hw_priv) {
		CAM_ERR(CAM_SFE, "Invalid input");
		return -EINVAL;
	}

	sfe_bus_rd = (struct cam_isp_resource_node *)hw_priv;
	rsrc_data = sfe_bus_rd->res_priv;
	common_data = rsrc_data->common_data;

	CAM_DBG(CAM_SFE, "SFE:%d start RD type:0x%x", sfe_bus_rd->res_id);

	if (sfe_bus_rd->res_state != CAM_ISP_RESOURCE_STATE_RESERVED) {
		CAM_ERR(CAM_SFE, "Invalid resource state:%d",
			sfe_bus_rd->res_state);
		return -EACCES;
	}

	if (!rsrc_data->is_offline) {
		val = (rsrc_data->fs_sync_enable << 5);
		cam_io_w_mb(val, common_data->mem_base +
			common_data->common_reg->input_if_cmd);
	}

	if (rsrc_data->secure_mode == CAM_SECURE_MODE_SECURE)
		cam_io_w_mb(1, common_data->mem_base +
			common_data->common_reg->security_cfg);

	for (i = 0; i < rsrc_data->num_rm; i++)
		rc = cam_sfe_bus_start_rm(rsrc_data->rm_res[i]);

	/* TO DO Subscribe mask for buf_done */
	if (!rc)
		sfe_bus_rd->res_state = CAM_ISP_RESOURCE_STATE_STREAMING;

	return rc;
}

static int cam_sfe_bus_stop_bus_rd(
	void *hw_priv, void *stop_hw_args, uint32_t arg_size)
{
	int rc = 0, i;
	struct cam_isp_resource_node *sfe_bus_rd = NULL;
	struct cam_sfe_bus_rd_data   *rsrc_data = NULL;

	if (!hw_priv) {
		CAM_ERR(CAM_SFE, "Invalid input");
		return -EINVAL;
	}

	sfe_bus_rd = (struct cam_isp_resource_node *)hw_priv;
	rsrc_data = sfe_bus_rd->res_priv;

	if (sfe_bus_rd->res_state == CAM_ISP_RESOURCE_STATE_AVAILABLE ||
		sfe_bus_rd->res_state == CAM_ISP_RESOURCE_STATE_RESERVED) {
		CAM_DBG(CAM_SFE, "SFE: %d res_id: 0x%x state: %d",
			rsrc_data->common_data->core_index, sfe_bus_rd->res_id,
			sfe_bus_rd->res_state);
		return rc;
	}
	for (i = 0; i < rsrc_data->num_rm; i++)
		rc = cam_sfe_bus_stop_rm(rsrc_data->rm_res[i]);

	sfe_bus_rd->res_state = CAM_ISP_RESOURCE_STATE_RESERVED;

	/* Unsubscribe mask for this wm IRQ controller */
	CAM_DBG(CAM_SFE, "SFE:%d stopped Bus RD:0x%x",
		rsrc_data->common_data->core_index,
		sfe_bus_rd->res_id);
	return rc;
}

static int cam_sfe_bus_init_sfe_bus_read_resource(
	uint32_t                           index,
	struct cam_sfe_bus_rd_priv        *bus_rd_priv,
	struct cam_sfe_bus_rd_hw_info     *bus_rd_hw_info)
{
	struct cam_isp_resource_node         *sfe_bus_rd = NULL;
	struct cam_sfe_bus_rd_data           *rsrc_data = NULL;
	int rc = 0;
	int32_t sfe_bus_rd_resc_type =
		bus_rd_hw_info->sfe_bus_rd_info[index].sfe_bus_rd_type;

	if (sfe_bus_rd_resc_type < 0 ||
		sfe_bus_rd_resc_type >= CAM_SFE_BUS_RD_MAX) {
		CAM_ERR(CAM_SFE, "Init SFE RD failed, Invalid type=%d",
			sfe_bus_rd_resc_type);
		return -EINVAL;
	}

	sfe_bus_rd = &bus_rd_priv->sfe_bus_rd[sfe_bus_rd_resc_type];
	if (sfe_bus_rd->res_state != CAM_ISP_RESOURCE_STATE_UNAVAILABLE ||
		sfe_bus_rd->res_priv) {
		CAM_ERR(CAM_SFE,
			"Error. Looks like same resource is init again");
		return -EFAULT;
	}

	rsrc_data = kzalloc(sizeof(struct cam_sfe_bus_rd_data),
		GFP_KERNEL);
	if (!rsrc_data) {
		rc = -ENOMEM;
		return rc;
	}

	sfe_bus_rd->res_priv = rsrc_data;

	sfe_bus_rd->res_type = CAM_ISP_RESOURCE_SFE_RD;
	sfe_bus_rd->res_state = CAM_ISP_RESOURCE_STATE_AVAILABLE;
	INIT_LIST_HEAD(&sfe_bus_rd->list);

	rsrc_data->bus_rd_type    =
		bus_rd_hw_info->sfe_bus_rd_info[index].sfe_bus_rd_type;
	rsrc_data->common_data = &bus_rd_priv->common_data;
	rsrc_data->max_width   =
		bus_rd_hw_info->sfe_bus_rd_info[index].max_width;
	rsrc_data->max_height  =
		bus_rd_hw_info->sfe_bus_rd_info[index].max_height;
	rsrc_data->secure_mode = CAM_SECURE_MODE_NON_SECURE;
	sfe_bus_rd->hw_intf = bus_rd_priv->common_data.hw_intf;

	return 0;
}

static int cam_sfe_bus_deinit_sfe_bus_rd_resource(
	struct cam_isp_resource_node    *sfe_bus_rd_res)
{
	struct cam_sfe_bus_rd_data *rsrc_data =
		sfe_bus_rd_res->res_priv;

	if (sfe_bus_rd_res->res_state ==
		CAM_ISP_RESOURCE_STATE_UNAVAILABLE) {
		/*
		 * This is not error. It can happen if the resource is
		 * never supported in the HW.
		 */
		CAM_DBG(CAM_SFE, "HW%d Res %d already deinitialized");
		return 0;
	}

	sfe_bus_rd_res->start = NULL;
	sfe_bus_rd_res->stop = NULL;
	sfe_bus_rd_res->top_half_handler = NULL;
	sfe_bus_rd_res->bottom_half_handler = NULL;
	sfe_bus_rd_res->hw_intf = NULL;

	sfe_bus_rd_res->res_state = CAM_ISP_RESOURCE_STATE_UNAVAILABLE;
	INIT_LIST_HEAD(&sfe_bus_rd_res->list);
	sfe_bus_rd_res->res_priv = NULL;

	if (!rsrc_data)
		return -ENOMEM;
	kfree(rsrc_data);

	return 0;
}

static int cam_sfe_bus_rd_update_rm(void *priv, void *cmd_args,
	uint32_t arg_size)
{
	struct cam_sfe_bus_rd_priv             *bus_priv;
	struct cam_isp_hw_get_cmd_update       *update_buf;
	struct cam_buf_io_cfg                  *io_cfg = NULL;
	struct cam_sfe_bus_rd_data             *sfe_bus_rd_data = NULL;
	struct cam_sfe_bus_rd_rm_resource_data *rm_data = NULL;
	uint32_t *reg_val_pair;
	uint32_t width = 0, height = 0, stride = 0;
	uint32_t  i, j, size = 0;

	bus_priv = (struct cam_sfe_bus_rd_priv  *) priv;
	update_buf =  (struct cam_isp_hw_get_cmd_update *) cmd_args;

	sfe_bus_rd_data = (struct cam_sfe_bus_rd_data *)
		update_buf->res->res_priv;

	if (!sfe_bus_rd_data || !sfe_bus_rd_data->cdm_util_ops) {
		CAM_ERR(CAM_SFE, "Failed! Invalid data");
		return -EINVAL;
	}

	CAM_DBG(CAM_SFE, "#of RM: %d scratch_buf_cfg: %s",
		sfe_bus_rd_data->num_rm,
		update_buf->use_scratch_cfg ? "true" : "false");
	if (update_buf->rm_update->num_buf != sfe_bus_rd_data->num_rm) {
		CAM_ERR(CAM_SFE,
			"Failed! Invalid number buffers:%d required:%d",
			update_buf->rm_update->num_buf,
			sfe_bus_rd_data->num_rm);
		return -EINVAL;
	}

	reg_val_pair = &sfe_bus_rd_data->common_data->io_buf_update[0];
	if (!update_buf->use_scratch_cfg)
		io_cfg = update_buf->rm_update->io_cfg;

	for (i = 0, j = 0; i < sfe_bus_rd_data->num_rm; i++) {
		if (j >= (MAX_REG_VAL_PAIR_SIZE - MAX_BUF_UPDATE_REG_NUM * 2)) {
			CAM_ERR(CAM_SFE,
				"reg_val_pair %d exceeds the array limit %lu",
				j, MAX_REG_VAL_PAIR_SIZE);
			return -ENOMEM;
		}

		rm_data = sfe_bus_rd_data->rm_res[i]->res_priv;
		if (update_buf->use_scratch_cfg) {
			stride = update_buf->rm_update->stride;
			width = update_buf->rm_update->width;
			height = update_buf->rm_update->height;
		} else {
			stride = io_cfg->planes[i].plane_stride;
			width = io_cfg->planes[i].width;
			height = io_cfg->planes[i].height;
		}

		/* update size register */
		cam_sfe_bus_rd_pxls_to_bytes(width,
			rm_data->unpacker_cfg, &rm_data->width);
		rm_data->height = height;

		CAM_SFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
			rm_data->hw_regs->buf_width, rm_data->width);
		CAM_SFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
			rm_data->hw_regs->buf_height, rm_data->height);
		CAM_DBG(CAM_SFE, "SFE:%d RM:%d width:0x%X height:0x%X",
			rm_data->common_data->core_index,
			rm_data->index, rm_data->width,
			rm_data->height);

		rm_data->stride = stride;
		CAM_SFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
			rm_data->hw_regs->stride, rm_data->stride);
		CAM_DBG(CAM_SFE, "SFE:%d RM:%d image_stride:0x%X",
			rm_data->common_data->core_index,
			rm_data->index, reg_val_pair[j-1]);

		CAM_SFE_ADD_REG_VAL_PAIR(reg_val_pair, j,
			rm_data->hw_regs->image_addr,
			update_buf->rm_update->image_buf[i]);
		CAM_DBG(CAM_SFE, "SFE:%d RM:%d image_address:0x%X",
			rm_data->common_data->core_index,
			rm_data->index, reg_val_pair[j-1]);
		rm_data->img_addr = reg_val_pair[j-1];
	}

	size = sfe_bus_rd_data->cdm_util_ops->cdm_required_size_reg_random(j/2);

	/* cdm util returns dwords, need to convert to bytes */
	if ((size * 4) > update_buf->cmd.size) {
		CAM_ERR(CAM_SFE,
			"Failed! Buf size:%d insufficient, expected size:%d",
			update_buf->cmd.size, size);
		return -ENOMEM;
	}

	sfe_bus_rd_data->cdm_util_ops->cdm_write_regrandom(
		update_buf->cmd.cmd_buf_addr, j/2, reg_val_pair);

	/* cdm util returns dwords, need to convert to bytes */
	update_buf->cmd.used_bytes = size * 4;

	return 0;
}

static int cam_sfe_bus_rd_update_fs_cfg(void *priv, void *cmd_args,
	uint32_t arg_size)
{
	struct cam_sfe_bus_rd_priv              *bus_priv;
	struct cam_sfe_bus_rd_data              *sfe_bus_rd_data = NULL;
	struct cam_sfe_bus_rd_rm_resource_data  *rm_data = NULL;
	struct cam_sfe_fe_update_args           *fe_upd_args;
	struct cam_fe_config                    *fe_cfg;
	struct cam_sfe_bus_rd_common_data       *common_data;
	int i = 0;

	bus_priv = (struct cam_sfe_bus_rd_priv  *) priv;
	fe_upd_args = (struct cam_sfe_fe_update_args *)cmd_args;

	sfe_bus_rd_data = (struct cam_sfe_bus_rd_data *)
		fe_upd_args->node_res->res_priv;

	if (!sfe_bus_rd_data || !sfe_bus_rd_data->cdm_util_ops) {
		CAM_ERR(CAM_SFE, "Failed! Invalid data");
		return -EINVAL;
	}

	fe_cfg = &fe_upd_args->fe_config;

	for (i = 0; i < sfe_bus_rd_data->num_rm; i++) {

		rm_data = sfe_bus_rd_data->rm_res[i]->res_priv;
		common_data = rm_data->common_data;

		rm_data->format = fe_cfg->format;
		rm_data->unpacker_cfg = fe_cfg->unpacker_cfg;
		rm_data->latency_buf_allocation = fe_cfg->latency_buf_size;
		rm_data->stride = fe_cfg->stride;
		rm_data->hbi_count = fe_cfg->hbi_count;
		rm_data->fs_mode = fe_cfg->fs_mode;
		rm_data->min_vbi = fe_cfg->min_vbi;
		sfe_bus_rd_data->fs_sync_enable =
			fe_cfg->fs_sync_enable;
		sfe_bus_rd_data->fs_line_sync_en =
			fe_cfg->fs_line_sync_en;

		CAM_DBG(CAM_SFE,
			"SFE:%d RM:%d format:0x%x unpacker_cfg:0x%x",
			rm_data->format, rm_data->unpacker_cfg);
		CAM_DBG(CAM_SFE,
			"latency_buf_alloc:0x%x stride:0x%x",
			rm_data->latency_buf_allocation, rm_data->stride);
		CAM_DBG(CAM_SFE,
			"fs_sync_en:%d hbi_cnt:0x%x fs_mode:0x%x min_vbi:0x%x",
			sfe_bus_rd_data->fs_sync_enable,
			rm_data->hbi_count, rm_data->fs_mode,
			rm_data->min_vbi);
	}
	return 0;
}

static int cam_sfe_bus_init_hw(void *hw_priv,
	void *init_hw_args, uint32_t arg_size)
{
	struct cam_sfe_bus_rd_priv    *bus_priv = hw_priv;
	uint32_t                       offset = 0;
	struct cam_sfe_bus_rd_reg_offset_common  *common_reg;
	uint32_t sfe_top_irq_mask[CAM_SFE_IRQ_REGISTERS_MAX] = {0};

	if (!bus_priv) {
		CAM_ERR(CAM_SFE, "Invalid args");
		return -EINVAL;
	}
	common_reg = bus_priv->common_data.common_reg;
	sfe_top_irq_mask[0] = (1 << bus_priv->top_irq_shift);

	/* Subscribe for top IRQ */
	bus_priv->irq_handle = cam_irq_controller_subscribe_irq(
		bus_priv->common_data.sfe_irq_controller,
		CAM_IRQ_PRIORITY_2,
		sfe_top_irq_mask,
		bus_priv,
		cam_sfe_bus_rd_handle_irq,
		NULL,
		NULL,
		NULL);

	if (bus_priv->irq_handle < 1) {
		CAM_ERR(CAM_SFE,
			"Failed to subscribe TOP IRQ for BUS RD");
		bus_priv->irq_handle = 0;
		return -EFAULT;
	}

	if (bus_priv->tasklet_info != NULL) {
		bus_priv->error_irq_handle = cam_irq_controller_subscribe_irq(
			bus_priv->common_data.bus_irq_controller,
			CAM_IRQ_PRIORITY_1,
			bus_rd_error_irq_mask,
			bus_priv,
			cam_sfe_bus_rd_handle_irq_top_half,
			cam_sfe_bus_rd_handle_irq_bottom_half,
			bus_priv->tasklet_info,
			&tasklet_bh_api);

		if (bus_priv->error_irq_handle < 1) {
			CAM_ERR(CAM_SFE, "Failed to subscribe error IRQ");
			bus_priv->error_irq_handle = 0;
			return -EFAULT;
		}
	}
	/* BUS_RD_TEST_BUS_CTRL disabling test bus */
	offset = common_reg->test_bus_ctrl;
	cam_io_w_mb(0x0, bus_priv->common_data.mem_base + offset);

	return 0;
}

static int cam_sfe_bus_deinit_hw(void *hw_priv,
	void *deinit_hw_args, uint32_t arg_size)
{
	struct cam_sfe_bus_rd_priv    *bus_priv = hw_priv;
	int                            rc = 0;

	if (!bus_priv) {
		CAM_ERR(CAM_SFE, "Error: Invalid args");
		return -EINVAL;
	}

	/* Unsubscribe irqs */
	if (bus_priv->irq_handle) {
		rc =   cam_irq_controller_unsubscribe_irq(
				bus_priv->common_data.sfe_irq_controller,
				bus_priv->irq_handle);
		if (rc)
			CAM_ERR(CAM_SFE, "Failed to unsubscribe top irq");
		bus_priv->irq_handle = 0;
	}

	if (bus_priv->error_irq_handle) {
		rc =   cam_irq_controller_unsubscribe_irq(
				bus_priv->common_data.bus_irq_controller,
				bus_priv->error_irq_handle);
		if (rc)
			CAM_ERR(CAM_SFE, "Failed to unsubscribe error irq");
		bus_priv->error_irq_handle = 0;
	}

	return rc;
}

static int cam_sfe_bus_rd_process_cmd(
	void *priv, uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size)
{
	int rc = -EINVAL;

	if (!priv || !cmd_args) {
		CAM_ERR_RATE_LIMIT(CAM_SFE,
			"Invalid input arguments");
		return -EINVAL;
	}

	switch (cmd_type) {
	case CAM_ISP_HW_CMD_GET_BUF_UPDATE_RM:
		rc = cam_sfe_bus_rd_update_rm(priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_GET_SECURE_MODE:
		rc = cam_sfe_bus_rd_get_secure_mode(priv, cmd_args, arg_size);
		break;
	case CAM_ISP_HW_CMD_FE_UPDATE_BUS_RD:
		rc = cam_sfe_bus_rd_update_fs_cfg(priv, cmd_args, arg_size);
		break;
	default:
		CAM_ERR_RATE_LIMIT(CAM_SFE,
			"Invalid SFE BUS RD command type: %d",
			cmd_type);
		break;
	}

	return rc;
}

int cam_sfe_bus_rd_init(
	struct cam_hw_soc_info               *soc_info,
	struct cam_hw_intf                   *hw_intf,
	void                                 *bus_hw_info,
	void                                 *sfe_irq_controller,
	struct cam_sfe_bus                  **sfe_bus)
{
	int i, rc = 0;
	struct cam_sfe_bus_rd_priv    *bus_priv = NULL;
	struct cam_sfe_bus            *sfe_bus_local;
	struct cam_sfe_bus_rd_hw_info *bus_rd_hw_info = bus_hw_info;

	if (!soc_info || !hw_intf || !bus_hw_info) {
		CAM_ERR(CAM_SFE,
			"Invalid_params soc_info:%pK hw_intf:%pK hw_info:%pK data:%pK",
			soc_info, hw_intf, bus_rd_hw_info);
		rc = -EINVAL;
		goto end;
	}

	sfe_bus_local = kzalloc(sizeof(struct cam_sfe_bus), GFP_KERNEL);
	if (!sfe_bus_local) {
		CAM_DBG(CAM_SFE, "Failed to alloc for sfe_bus");
		rc = -ENOMEM;
		goto end;
	}

	bus_priv = kzalloc(sizeof(struct cam_sfe_bus_rd_priv),
		GFP_KERNEL);
	if (!bus_priv) {
		CAM_DBG(CAM_SFE, "Failed to alloc for sfe_bus_priv");
		rc = -ENOMEM;
		goto free_bus_local;
	}

	sfe_bus_local->bus_priv = bus_priv;
	bus_priv->num_client                    = bus_rd_hw_info->num_client;
	bus_priv->num_bus_rd_resc               =
		bus_rd_hw_info->num_bus_rd_resc;
	bus_priv->common_data.core_index        = soc_info->index;
	bus_priv->common_data.mem_base          =
		CAM_SOC_GET_REG_MAP_START(soc_info, SFE_CORE_BASE_IDX);
	bus_priv->common_data.hw_intf           = hw_intf;
	bus_priv->common_data.sfe_irq_controller = sfe_irq_controller;
	bus_priv->common_data.common_reg        = &bus_rd_hw_info->common_reg;
	bus_priv->top_irq_shift                 = bus_rd_hw_info->top_irq_shift;

	rc = cam_irq_controller_init(drv_name,
		bus_priv->common_data.mem_base,
		&bus_rd_hw_info->common_reg.irq_reg_info,
		&bus_priv->common_data.bus_irq_controller, true);
	if (rc) {
		CAM_ERR(CAM_SFE, "IRQ controller init failed");
		goto free_bus_priv;
	}

	for (i = 0; i < bus_priv->num_client; i++) {
		rc = cam_sfe_bus_init_rm_resource(i, bus_priv, bus_hw_info,
			&bus_priv->bus_client[i]);
		if (rc < 0) {
			CAM_ERR(CAM_SFE, "Init RM failed rc=%d", rc);
			goto deinit_rm;
		}
	}

	for (i = 0; i < bus_priv->num_bus_rd_resc; i++) {
		rc = cam_sfe_bus_init_sfe_bus_read_resource(i, bus_priv,
			bus_rd_hw_info);
		if (rc < 0) {
			CAM_ERR(CAM_SFE, "Init SFE RD failed rc=%d", rc);
			goto deinit_sfe_bus_rd;
		}
	}

	spin_lock_init(&bus_priv->common_data.spin_lock);
	INIT_LIST_HEAD(&bus_priv->common_data.free_payload_list);
	for (i = 0; i < CAM_SFE_BUS_RD_PAYLOAD_MAX; i++) {
		INIT_LIST_HEAD(&bus_priv->common_data.evt_payload[i].list);
		list_add_tail(&bus_priv->common_data.evt_payload[i].list,
			&bus_priv->common_data.free_payload_list);
	}

	sfe_bus_local->hw_ops.reserve      = cam_sfe_bus_acquire_bus_rd;
	sfe_bus_local->hw_ops.release      = cam_sfe_bus_release_bus_rd;
	sfe_bus_local->hw_ops.start        = cam_sfe_bus_start_bus_rd;
	sfe_bus_local->hw_ops.stop         = cam_sfe_bus_stop_bus_rd;
	sfe_bus_local->hw_ops.init         = cam_sfe_bus_init_hw;
	sfe_bus_local->hw_ops.deinit       = cam_sfe_bus_deinit_hw;
	sfe_bus_local->top_half_handler    = cam_sfe_bus_rd_handle_irq;
	sfe_bus_local->bottom_half_handler = NULL;
	sfe_bus_local->hw_ops.process_cmd  = cam_sfe_bus_rd_process_cmd;

	*sfe_bus = sfe_bus_local;

	return rc;

deinit_sfe_bus_rd:
	if (i < 0)
		i = CAM_SFE_BUS_RD_MAX;
	for (--i; i >= 0; i--)
		cam_sfe_bus_deinit_sfe_bus_rd_resource(
			&bus_priv->sfe_bus_rd[i]);
deinit_rm:
	if (i < 0)
		i = bus_priv->num_client;
	for (--i; i >= 0; i--)
		cam_sfe_bus_deinit_rm_resource(&bus_priv->bus_client[i]);

free_bus_priv:
	kfree(sfe_bus_local->bus_priv);

free_bus_local:
	kfree(sfe_bus_local);

end:
	return rc;
}

int cam_sfe_bus_rd_deinit(
	struct cam_sfe_bus                  **sfe_bus)
{
	int i, rc = 0;
	unsigned long flags;
	struct cam_sfe_bus_rd_priv    *bus_priv = NULL;
	struct cam_sfe_bus            *sfe_bus_local;

	if (!sfe_bus || !*sfe_bus) {
		CAM_ERR(CAM_SFE, "Invalid input");
		return -EINVAL;
	}
	sfe_bus_local = *sfe_bus;

	bus_priv = sfe_bus_local->bus_priv;
	if (!bus_priv) {
		CAM_ERR(CAM_SFE, "bus_priv is NULL");
		rc = -ENODEV;
		goto free_bus_local;
	}

	spin_lock_irqsave(&bus_priv->common_data.spin_lock, flags);
	INIT_LIST_HEAD(&bus_priv->common_data.free_payload_list);
	for (i = 0; i < CAM_SFE_BUS_RD_PAYLOAD_MAX; i++)
		INIT_LIST_HEAD(&bus_priv->common_data.evt_payload[i].list);
	spin_unlock_irqrestore(&bus_priv->common_data.spin_lock, flags);

	for (i = 0; i < bus_priv->num_client; i++) {
		rc = cam_sfe_bus_deinit_rm_resource(&bus_priv->bus_client[i]);
		if (rc < 0)
			CAM_ERR(CAM_SFE,
				"Deinit RM failed rc=%d", rc);
	}
	for (i = 0; i < CAM_SFE_BUS_RD_MAX; i++) {
		rc = cam_sfe_bus_deinit_sfe_bus_rd_resource(
			&bus_priv->sfe_bus_rd[i]);
		if (rc < 0)
			CAM_ERR(CAM_SFE,
				"Deinit SFE RD failed rc=%d", rc);
	}

	rc = cam_irq_controller_deinit(
		&bus_priv->common_data.bus_irq_controller);
	if (rc)
		CAM_ERR(CAM_SFE,
			"Deinit IRQ Controller failed rc=%d", rc);

	kfree(sfe_bus_local->bus_priv);

free_bus_local:
	kfree(sfe_bus_local);
	*sfe_bus = NULL;

	return rc;
}
