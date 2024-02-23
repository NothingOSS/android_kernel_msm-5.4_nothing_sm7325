// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/debugfs.h>
#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ratelimit.h>

#include "cam_mem_mgr.h"
#include "cam_sync_api.h"
#include "cam_req_mgr_dev.h"
#include "cam_trace.h"
#include "cam_debug_util.h"
#include "cam_packet_util.h"
#include "cam_context_utils.h"
#include "cam_cdm_util.h"
#include "cam_isp_context.h"
#include "cam_common_util.h"
#include "cam_req_mgr_debug.h"
#include "cam_cpas_api.h"
#include "cam_subdev.h"

static const char isp_dev_name[] = "cam-isp";

static struct cam_isp_ctx_debug isp_ctx_debug;

#define INC_HEAD(head, max_entries, ret) \
	div_u64_rem(atomic64_add_return(1, head),\
	max_entries, (ret))

static int cam_isp_context_dump_requests(void *data,
	struct cam_smmu_pf_info *pf_info);

static int cam_isp_context_handle_message(void *context,
	uint32_t msg_type, uint32_t *data);

static int __cam_isp_ctx_start_dev_in_ready(struct cam_context *ctx,
	struct cam_start_stop_dev_cmd *cmd);

static const char *__cam_isp_evt_val_to_type(
	uint32_t evt_id)
{
	switch (evt_id) {
	case CAM_ISP_CTX_EVENT_SUBMIT:
		return "SUBMIT";
	case CAM_ISP_CTX_EVENT_APPLY:
		return "APPLY";
	case CAM_ISP_CTX_EVENT_EPOCH:
		return "EPOCH";
	case CAM_ISP_CTX_EVENT_RUP:
		return "RUP";
	case CAM_ISP_CTX_EVENT_BUFDONE:
		return "BUFDONE";
	default:
		return "CAM_ISP_EVENT_INVALID";
	}
}

static void __cam_isp_ctx_update_event_record(
	struct cam_isp_context *ctx_isp,
	enum cam_isp_ctx_event  event,
	struct cam_ctx_request *req)
{
	int                      iterator = 0;
	ktime_t                  cur_time;
	struct cam_isp_ctx_req  *req_isp;

	if (!ctx_isp) {
		CAM_ERR(CAM_ISP, "Invalid Args");
		return;
	}
	switch (event) {
	case CAM_ISP_CTX_EVENT_EPOCH:
	case CAM_ISP_CTX_EVENT_RUP:
	case CAM_ISP_CTX_EVENT_BUFDONE:
		break;
	case CAM_ISP_CTX_EVENT_SUBMIT:
	case CAM_ISP_CTX_EVENT_APPLY:
		if (!req) {
			CAM_ERR(CAM_ISP, "Invalid arg for event %d", event);
			return;
		}
		break;
	default:
		break;
	}

	INC_HEAD(&ctx_isp->event_record_head[event],
		CAM_ISP_CTX_EVENT_RECORD_MAX_ENTRIES, &iterator);
	cur_time = ktime_get();
	if (req) {
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		ctx_isp->event_record[event][iterator].req_id =
			req->request_id;
		req_isp->event_timestamp[event] = cur_time;
	} else {
		ctx_isp->event_record[event][iterator].req_id = 0;
	}
	ctx_isp->event_record[event][iterator].timestamp  = cur_time;
}

static int __cam_isp_ctx_dump_event_record(
	struct cam_isp_context *ctx_isp,
	uintptr_t               cpu_addr,
	size_t                  buf_len,
	size_t                 *offset)
{
	int                                  i, j;
	int                                  index;
	size_t                               remain_len;
	uint8_t                             *dst;
	uint32_t                             oldest_entry, num_entries;
	uint32_t                             min_len;
	uint64_t                            *addr, *start;
	uint64_t                             state_head;
	struct timespec64                    ts;
	struct cam_isp_context_dump_header  *hdr;
	struct cam_isp_context_event_record *record;

	if (!cpu_addr || !buf_len || !offset || !ctx_isp) {
		CAM_ERR(CAM_ISP, "Invalid args %pK %zu %pK %pK",
			cpu_addr, buf_len, offset, ctx_isp);
		return -EINVAL;
	}
	for (i = 0; i < CAM_ISP_CTX_EVENT_MAX; i++) {
		state_head = atomic64_read(&ctx_isp->event_record_head[i]);

		if (state_head == -1) {
			return 0;
		} else if (state_head < CAM_ISP_CTX_EVENT_RECORD_MAX_ENTRIES) {
			num_entries = state_head + 1;
			oldest_entry = 0;
		} else {
			num_entries = CAM_ISP_CTX_EVENT_RECORD_MAX_ENTRIES;
			div_u64_rem(state_head + 1,
				CAM_ISP_CTX_EVENT_RECORD_MAX_ENTRIES,
				&oldest_entry);
		}
		index = oldest_entry;

		if (buf_len <= *offset) {
			CAM_WARN(CAM_ISP,
				"Dump buffer overshoot len %zu offset %zu",
				buf_len, *offset);
			return -ENOSPC;
		}

		min_len = sizeof(struct cam_isp_context_dump_header) +
			((num_entries * CAM_ISP_CTX_DUMP_EVENT_NUM_WORDS) *
			sizeof(uint64_t));
		remain_len = buf_len - *offset;

		if (remain_len < min_len) {
			CAM_WARN(CAM_ISP,
				"Dump buffer exhaust remain %zu min %u",
				remain_len, min_len);
			return -ENOSPC;
		}
		dst = (uint8_t *)cpu_addr + *offset;
		hdr = (struct cam_isp_context_dump_header *)dst;
		scnprintf(hdr->tag,
			CAM_ISP_CONTEXT_DUMP_TAG_MAX_LEN, "ISP_EVT_%s:",
			__cam_isp_evt_val_to_type(i));
		hdr->word_size = sizeof(uint64_t);
		addr = (uint64_t *)(dst +
			sizeof(struct cam_isp_context_dump_header));
		start = addr;
		for (j = 0; j <  num_entries; j++) {
			record  = &ctx_isp->event_record[i][index];
			ts      = ktime_to_timespec64(record->timestamp);
			*addr++ = record->req_id;
			*addr++ = ts.tv_sec;
			*addr++ = ts.tv_nsec/NSEC_PER_USEC;
			index = (index + 1) %
				CAM_ISP_CTX_EVENT_RECORD_MAX_ENTRIES;
		}
		hdr->size = hdr->word_size * (addr - start);
		*offset += hdr->size +
			sizeof(struct cam_isp_context_dump_header);
	}
	return 0;
}

static void __cam_isp_ctx_update_state_monitor_array(
	struct cam_isp_context *ctx_isp,
	enum cam_isp_state_change_trigger trigger_type,
	uint64_t req_id)
{
	int iterator;

	INC_HEAD(&ctx_isp->state_monitor_head,
		CAM_ISP_CTX_STATE_MONITOR_MAX_ENTRIES, &iterator);

	ctx_isp->cam_isp_ctx_state_monitor[iterator].curr_state =
		ctx_isp->substate_activated;
	ctx_isp->cam_isp_ctx_state_monitor[iterator].frame_id =
		ctx_isp->frame_id;
	ctx_isp->cam_isp_ctx_state_monitor[iterator].trigger =
		trigger_type;
	ctx_isp->cam_isp_ctx_state_monitor[iterator].req_id =
		req_id;
	ctx_isp->cam_isp_ctx_state_monitor[iterator].evt_time_stamp =
		jiffies_to_msecs(jiffies) - ctx_isp->init_timestamp;
}

static const char *__cam_isp_ctx_substate_val_to_type(
	enum cam_isp_ctx_activated_substate type)
{
	switch (type) {
	case CAM_ISP_CTX_ACTIVATED_SOF:
		return "SOF";
	case CAM_ISP_CTX_ACTIVATED_APPLIED:
		return "APPLIED";
	case CAM_ISP_CTX_ACTIVATED_EPOCH:
		return "EPOCH";
	case CAM_ISP_CTX_ACTIVATED_BUBBLE:
		return "BUBBLE";
	case CAM_ISP_CTX_ACTIVATED_BUBBLE_APPLIED:
		return "BUBBLE_APPLIED";
	case CAM_ISP_CTX_ACTIVATED_HW_ERROR:
		return "HW_ERROR";
	case CAM_ISP_CTX_ACTIVATED_HALT:
		return "HALT";
	default:
		return "INVALID";
	}
}

static const char *__cam_isp_hw_evt_val_to_type(
	uint32_t evt_id)
{
	switch (evt_id) {
	case CAM_ISP_STATE_CHANGE_TRIGGER_ERROR:
		return "ERROR";
	case CAM_ISP_STATE_CHANGE_TRIGGER_APPLIED:
		return "APPLIED";
	case CAM_ISP_STATE_CHANGE_TRIGGER_SOF:
		return "SOF";
	case CAM_ISP_STATE_CHANGE_TRIGGER_REG_UPDATE:
		return "REG_UPDATE";
	case CAM_ISP_STATE_CHANGE_TRIGGER_EPOCH:
		return "EPOCH";
	case CAM_ISP_STATE_CHANGE_TRIGGER_EOF:
		return "EOF";
	case CAM_ISP_STATE_CHANGE_TRIGGER_DONE:
		return "DONE";
	case CAM_ISP_STATE_CHANGE_TRIGGER_FLUSH:
		return "FLUSH";
	default:
		return "CAM_ISP_EVENT_INVALID";
	}
}

static void __cam_isp_ctx_dump_state_monitor_array(
	struct cam_isp_context *ctx_isp)
{
	int i = 0;
	int64_t state_head = 0;
	uint32_t index, num_entries, oldest_entry;

	state_head = atomic64_read(&ctx_isp->state_monitor_head);

	if (state_head == -1) {
		return;
	} else if (state_head < CAM_ISP_CTX_STATE_MONITOR_MAX_ENTRIES) {
		num_entries = state_head;
		oldest_entry = 0;
	} else {
		num_entries = CAM_ISP_CTX_STATE_MONITOR_MAX_ENTRIES;
		div_u64_rem(state_head + 1,
			CAM_ISP_CTX_STATE_MONITOR_MAX_ENTRIES, &oldest_entry);
	}

	CAM_ERR(CAM_ISP,
		"Dumping state information for preceding requests");

	index = oldest_entry;

	for (i = 0; i < num_entries; i++) {
		CAM_ERR(CAM_ISP,
		"Index[%d] time[%d] : Substate[%s] Frame[%lld] ReqId[%llu] evt_type[%s]",
		index,
		ctx_isp->cam_isp_ctx_state_monitor[index].evt_time_stamp,
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->cam_isp_ctx_state_monitor[index].curr_state),
		ctx_isp->cam_isp_ctx_state_monitor[index].frame_id,
		ctx_isp->cam_isp_ctx_state_monitor[index].req_id,
		__cam_isp_hw_evt_val_to_type(
		ctx_isp->cam_isp_ctx_state_monitor[index].trigger));

		index = (index + 1) % CAM_ISP_CTX_STATE_MONITOR_MAX_ENTRIES;
	}
}

static int cam_isp_context_info_dump(void *context,
	enum cam_context_dump_id id)
{
	struct cam_context *ctx = (struct cam_context *)context;

	switch (id) {
	case CAM_CTX_DUMP_ACQ_INFO: {
		cam_context_dump_hw_acq_info(ctx);
		break;
	}
	default:
		CAM_DBG(CAM_ISP, "DUMP id not valid %u", id);
		break;
	}

	return 0;
}

static int cam_isp_ctx_dump_req(
	struct cam_isp_ctx_req  *req_isp,
	uintptr_t                cpu_addr,
	size_t                   buf_len,
	size_t                  *offset,
	bool                     dump_to_buff)
{
	int i, rc = 0;
	size_t len = 0;
	uint32_t *buf_addr;
	uint32_t *buf_start, *buf_end;
	size_t remain_len = 0;
	struct cam_cdm_cmd_buf_dump_info dump_info;

	for (i = 0; i < req_isp->num_cfg; i++) {
		rc = cam_packet_util_get_cmd_mem_addr(
			req_isp->cfg[i].handle, &buf_addr, &len);
		if (rc) {
			CAM_ERR_RATE_LIMIT(CAM_ISP,
				"Failed to get_cmd_mem_addr, rc=%d",
				rc);
		} else {
			if (req_isp->cfg[i].offset >= ((uint32_t)len)) {
				CAM_ERR(CAM_ISP,
					"Invalid offset exp %u actual %u",
					req_isp->cfg[i].offset, (uint32_t)len);
				return -EINVAL;
			}
			remain_len = len - req_isp->cfg[i].offset;

			if (req_isp->cfg[i].len >
				((uint32_t)remain_len)) {
				CAM_ERR(CAM_ISP,
					"Invalid len exp %u remain_len %u",
					req_isp->cfg[i].len,
					(uint32_t)remain_len);
				return -EINVAL;
			}

			buf_start = (uint32_t *)((uint8_t *) buf_addr +
				req_isp->cfg[i].offset);
			buf_end = (uint32_t *)((uint8_t *) buf_start +
				req_isp->cfg[i].len - 1);

			if (dump_to_buff) {
				if (!cpu_addr || !offset || !buf_len) {
					CAM_ERR(CAM_ISP, "Invalid args");
					break;
				}
				dump_info.src_start = buf_start;
				dump_info.src_end =   buf_end;
				dump_info.dst_start = cpu_addr;
				dump_info.dst_offset = *offset;
				dump_info.dst_max_size = buf_len;
				rc = cam_cdm_util_dump_cmd_bufs_v2(
					&dump_info);
				*offset = dump_info.dst_offset;
				if (rc)
					return rc;
			} else
				cam_cdm_util_dump_cmd_buf(buf_start, buf_end);
		}
	}
	return rc;
}

static int __cam_isp_ctx_enqueue_request_in_order(
	struct cam_context *ctx, struct cam_ctx_request *req)
{
	struct cam_ctx_request           *req_current;
	struct cam_ctx_request           *req_prev;
	struct list_head                  temp_list;
	struct cam_isp_context           *ctx_isp;

	INIT_LIST_HEAD(&temp_list);
	spin_lock_bh(&ctx->lock);
	if (list_empty(&ctx->pending_req_list)) {
		list_add_tail(&req->list, &ctx->pending_req_list);
	} else {
		list_for_each_entry_safe_reverse(
			req_current, req_prev, &ctx->pending_req_list, list) {
			if (req->request_id < req_current->request_id) {
				list_del_init(&req_current->list);
				list_add(&req_current->list, &temp_list);
				continue;
			} else if (req->request_id == req_current->request_id) {
				CAM_WARN(CAM_ISP,
					"Received duplicated request %lld",
					req->request_id);
			}
			break;
		}
		list_add_tail(&req->list, &ctx->pending_req_list);

		if (!list_empty(&temp_list)) {
			list_for_each_entry_safe(
				req_current, req_prev, &temp_list, list) {
				list_del_init(&req_current->list);
				list_add_tail(&req_current->list,
					&ctx->pending_req_list);
			}
		}
	}
	ctx_isp = (struct cam_isp_context *) ctx->ctx_priv;
	__cam_isp_ctx_update_event_record(ctx_isp,
		CAM_ISP_CTX_EVENT_SUBMIT, req);
	spin_unlock_bh(&ctx->lock);
	return 0;
}

static int __cam_isp_ctx_enqueue_init_request(
	struct cam_context *ctx, struct cam_ctx_request *req)
{
	int rc = 0;
	struct cam_ctx_request                *req_old;
	struct cam_isp_ctx_req                *req_isp_old;
	struct cam_isp_ctx_req                *req_isp_new;
	struct cam_isp_prepare_hw_update_data *req_update_old;
	struct cam_isp_prepare_hw_update_data *req_update_new;
	struct cam_isp_prepare_hw_update_data *hw_update_data;

	spin_lock_bh(&ctx->lock);
	if (list_empty(&ctx->pending_req_list)) {
		list_add_tail(&req->list, &ctx->pending_req_list);
		CAM_DBG(CAM_ISP, "INIT packet added req id= %d",
			req->request_id);
		goto end;
	}

	req_old = list_first_entry(&ctx->pending_req_list,
		struct cam_ctx_request, list);
	req_isp_old = (struct cam_isp_ctx_req *) req_old->req_priv;
	req_isp_new = (struct cam_isp_ctx_req *) req->req_priv;
	if (req_isp_old->hw_update_data.packet_opcode_type ==
		CAM_ISP_PACKET_INIT_DEV) {
		if ((req_isp_old->num_cfg + req_isp_new->num_cfg) >=
			CAM_ISP_CTX_CFG_MAX) {
			CAM_WARN(CAM_ISP,
				"Can not merge INIT pkt num_cfgs = %d",
				(req_isp_old->num_cfg +
					req_isp_new->num_cfg));
			rc = -ENOMEM;
		}

		if (req_isp_old->num_fence_map_out != 0 ||
			req_isp_old->num_fence_map_in != 0) {
			CAM_WARN(CAM_ISP, "Invalid INIT pkt sequence");
			rc = -EINVAL;
		}

		if (!rc) {
			memcpy(req_isp_old->fence_map_out,
				req_isp_new->fence_map_out,
				sizeof(req_isp_new->fence_map_out[0])*
				req_isp_new->num_fence_map_out);
			req_isp_old->num_fence_map_out =
				req_isp_new->num_fence_map_out;

			memcpy(req_isp_old->fence_map_in,
				req_isp_new->fence_map_in,
				sizeof(req_isp_new->fence_map_in[0])*
				req_isp_new->num_fence_map_in);
			req_isp_old->num_fence_map_in =
				req_isp_new->num_fence_map_in;

			memcpy(&req_isp_old->cfg[req_isp_old->num_cfg],
				req_isp_new->cfg,
				sizeof(req_isp_new->cfg[0]) *
				req_isp_new->num_cfg);
			req_isp_old->num_cfg += req_isp_new->num_cfg;

			memcpy(&req_old->pf_data, &req->pf_data,
				sizeof(struct cam_hw_mgr_dump_pf_data));

			if (req_isp_new->hw_update_data.num_reg_dump_buf) {
				req_update_new = &req_isp_new->hw_update_data;
				req_update_old = &req_isp_old->hw_update_data;
				memcpy(&req_update_old->reg_dump_buf_desc,
					&req_update_new->reg_dump_buf_desc,
					sizeof(struct cam_cmd_buf_desc) *
					req_update_new->num_reg_dump_buf);
				req_update_old->num_reg_dump_buf =
					req_update_new->num_reg_dump_buf;
			}

			/* Update frame header params for EPCR */
			hw_update_data = &req_isp_new->hw_update_data;
			req_isp_old->hw_update_data.frame_header_res_id =
				req_isp_new->hw_update_data.frame_header_res_id;
			req_isp_old->hw_update_data.frame_header_cpu_addr =
				hw_update_data->frame_header_cpu_addr;

			req_old->request_id = req->request_id;

			list_add_tail(&req->list, &ctx->free_req_list);
		}
	} else {
		CAM_WARN(CAM_ISP,
			"Received Update pkt before INIT pkt. req_id= %lld",
			req->request_id);
		rc = -EINVAL;
	}
end:
	spin_unlock_bh(&ctx->lock);
	return rc;
}

static const char *__cam_isp_resource_handle_id_to_type(
	uint32_t resource_handle)
{
	switch (resource_handle) {
	case CAM_ISP_IFE_OUT_RES_FULL:
		return "FULL";
	case CAM_ISP_IFE_OUT_RES_DS4:
		return "DS4";
	case CAM_ISP_IFE_OUT_RES_DS16:
		return "DS16";
	case CAM_ISP_IFE_OUT_RES_RAW_DUMP:
		return "RAW_DUMP";
	case CAM_ISP_IFE_OUT_RES_FD:
		return "FD";
	case CAM_ISP_IFE_OUT_RES_PDAF:
		return "PDAF";
	case CAM_ISP_IFE_OUT_RES_RDI_0:
		return "RDI_0";
	case CAM_ISP_IFE_OUT_RES_RDI_1:
		return "RDI_1";
	case CAM_ISP_IFE_OUT_RES_RDI_2:
		return "RDI_2";
	case CAM_ISP_IFE_OUT_RES_RDI_3:
		return "RDI_3";
	case CAM_ISP_IFE_OUT_RES_STATS_HDR_BE:
		return "STATS_HDR_BE";
	case CAM_ISP_IFE_OUT_RES_STATS_HDR_BHIST:
		return "STATS_HDR_BHIST";
	case CAM_ISP_IFE_OUT_RES_STATS_TL_BG:
		return "STATS_TL_BG";
	case CAM_ISP_IFE_OUT_RES_STATS_BF:
		return "STATS_BF";
	case CAM_ISP_IFE_OUT_RES_STATS_AWB_BG:
		return "STATS_AWB_BG";
	case CAM_ISP_IFE_OUT_RES_STATS_BHIST:
		return "STATS_BHIST";
	case CAM_ISP_IFE_OUT_RES_STATS_RS:
		return "STATS_RS";
	case CAM_ISP_IFE_OUT_RES_STATS_CS:
		return "STATS_CS";
	case CAM_ISP_IFE_OUT_RES_STATS_IHIST:
		return "STATS_IHIST";
	case CAM_ISP_IFE_OUT_RES_FULL_DISP:
		return "FULL_DISP";
	case CAM_ISP_IFE_OUT_RES_DS4_DISP:
		return "DS4_DISP";
	case CAM_ISP_IFE_OUT_RES_DS16_DISP:
		return "DS16_DISP";
	case CAM_ISP_IFE_OUT_RES_2PD:
		return "2PD";
	case CAM_ISP_IFE_OUT_RES_RDI_RD:
		return "RDI_RD";
	case CAM_ISP_IFE_OUT_RES_LCR:
		return "LCR";
	case CAM_ISP_IFE_OUT_RES_AWB_BFW:
		return "AWB_BFW";
	case CAM_ISP_IFE_OUT_RES_2PD_STATS:
		return "2PD_STATS";
	case CAM_ISP_IFE_OUT_RES_STATS_AEC_BE:
		return "STATS_AEC_BE";
	case CAM_ISP_IFE_OUT_RES_LTM_STATS:
		return "LTM_STATS";
	case CAM_ISP_IFE_OUT_RES_STATS_GTM_BHIST:
		return "STATS_GTM_BHIST";
	case CAM_ISP_IFE_LITE_OUT_RES_STATS_BE:
		return "STATS_BE";
	case CAM_ISP_IFE_LITE_OUT_RES_GAMMA:
		return "GAMMA";
	default:
		return "CAM_ISP_Invalid_Resource_Type";
	}
}

static const char *__cam_isp_tfe_resource_handle_id_to_type(
	uint32_t resource_handle)
{

	switch (resource_handle) {
	case CAM_ISP_TFE_OUT_RES_FULL:
		return "FULL";
	case CAM_ISP_TFE_OUT_RES_RAW_DUMP:
		return "RAW_DUMP";
	case CAM_ISP_TFE_OUT_RES_PDAF:
		return "PDAF";
	case CAM_ISP_TFE_OUT_RES_RDI_0:
		return "RDI_0";
	case CAM_ISP_TFE_OUT_RES_RDI_1:
		return "RDI_1";
	case CAM_ISP_TFE_OUT_RES_RDI_2:
		return "RDI_2";
	case CAM_ISP_TFE_OUT_RES_STATS_HDR_BE:
		return "STATS_HDR_BE";
	case CAM_ISP_TFE_OUT_RES_STATS_HDR_BHIST:
		return "STATS_HDR_BHIST";
	case CAM_ISP_TFE_OUT_RES_STATS_TL_BG:
		return "STATS_TL_BG";
	case CAM_ISP_TFE_OUT_RES_STATS_BF:
		return "STATS_BF";
	case CAM_ISP_TFE_OUT_RES_STATS_AWB_BG:
		return "STATS_AWB_BG";
	default:
		return "CAM_ISP_Invalid_Resource_Type";
	}
}

static uint64_t __cam_isp_ctx_get_event_ts(uint32_t evt_id, void *evt_data)
{
	uint64_t ts = 0;

	if (!evt_data)
		return 0;

	switch (evt_id) {
	case CAM_ISP_HW_EVENT_ERROR:
		ts = ((struct cam_isp_hw_error_event_data *)evt_data)->
			timestamp;
		break;
	case CAM_ISP_HW_EVENT_SOF:
		ts = ((struct cam_isp_hw_sof_event_data *)evt_data)->
			timestamp;
		break;
	case CAM_ISP_HW_EVENT_REG_UPDATE:
		ts = ((struct cam_isp_hw_reg_update_event_data *)evt_data)->
			timestamp;
		break;
	case CAM_ISP_HW_EVENT_EPOCH:
		ts = ((struct cam_isp_hw_epoch_event_data *)evt_data)->
			timestamp;
		break;
	case CAM_ISP_HW_EVENT_EOF:
		ts = ((struct cam_isp_hw_eof_event_data *)evt_data)->
			timestamp;
		break;
	case CAM_ISP_HW_EVENT_DONE:
		break;
	default:
		CAM_DBG(CAM_ISP, "Invalid Event Type %d", evt_id);
	}

	return ts;
}

static void __cam_isp_ctx_send_sof_boot_timestamp(
	struct cam_isp_context *ctx_isp, uint64_t request_id,
	uint32_t sof_event_status)
{
	struct cam_req_mgr_message   req_msg;

	req_msg.session_hdl = ctx_isp->base->session_hdl;
	req_msg.u.frame_msg.frame_id = ctx_isp->frame_id;
	req_msg.u.frame_msg.request_id = request_id;
	req_msg.u.frame_msg.timestamp = ctx_isp->boot_timestamp;
	req_msg.u.frame_msg.link_hdl = ctx_isp->base->link_hdl;
	req_msg.u.frame_msg.sof_status = sof_event_status;
	req_msg.u.frame_msg.frame_id_meta = ctx_isp->frame_id_meta;

	CAM_DBG(CAM_ISP,
		"request id:%lld frame number:%lld boot time stamp:0x%llx status:%u",
		 request_id, ctx_isp->frame_id,
		 ctx_isp->boot_timestamp, sof_event_status);

	if (cam_req_mgr_notify_message(&req_msg,
		V4L_EVENT_CAM_REQ_MGR_SOF_BOOT_TS,
		V4L_EVENT_CAM_REQ_MGR_EVENT))
		CAM_ERR(CAM_ISP,
			"Error in notifying the boot time for req id:%lld",
			request_id);
}

static void __cam_isp_ctx_send_sof_timestamp_frame_header(
	struct cam_isp_context *ctx_isp, uint32_t *frame_header_cpu_addr,
	uint64_t request_id, uint32_t sof_event_status)
{
	uint32_t *time32 = NULL;
	uint64_t timestamp = 0;
	struct cam_req_mgr_message   req_msg;

	time32 = frame_header_cpu_addr;
	timestamp = (uint64_t) time32[1];
	timestamp = timestamp << 24;
	timestamp |= (uint64_t)(time32[0] >> 8);
	timestamp = mul_u64_u32_div(timestamp,
			CAM_IFE_QTIMER_MUL_FACTOR,
			CAM_IFE_QTIMER_DIV_FACTOR);

	ctx_isp->sof_timestamp_val = timestamp;
	req_msg.session_hdl = ctx_isp->base->session_hdl;
	req_msg.u.frame_msg.frame_id = ctx_isp->frame_id;
	req_msg.u.frame_msg.request_id = request_id;
	req_msg.u.frame_msg.timestamp = ctx_isp->sof_timestamp_val;
	req_msg.u.frame_msg.link_hdl = ctx_isp->base->link_hdl;
	req_msg.u.frame_msg.sof_status = sof_event_status;

	CAM_DBG(CAM_ISP,
		"request id:%lld frame number:%lld SOF time stamp:0x%llx status:%u",
		 request_id, ctx_isp->frame_id,
		ctx_isp->sof_timestamp_val, sof_event_status);

	if (cam_req_mgr_notify_message(&req_msg,
		V4L_EVENT_CAM_REQ_MGR_SOF, V4L_EVENT_CAM_REQ_MGR_EVENT))
		CAM_ERR(CAM_ISP,
			"Error in notifying the sof time for req id:%lld",
			request_id);
}

static void __cam_isp_ctx_send_sof_timestamp(
	struct cam_isp_context *ctx_isp, uint64_t request_id,
	uint32_t sof_event_status)
{
	struct cam_req_mgr_message   req_msg;

	if ((ctx_isp->use_frame_header_ts) || (request_id == 0))
		goto end;

	req_msg.session_hdl = ctx_isp->base->session_hdl;
	req_msg.u.frame_msg.frame_id = ctx_isp->frame_id;
	req_msg.u.frame_msg.request_id = request_id;
	req_msg.u.frame_msg.timestamp = ctx_isp->sof_timestamp_val;
	req_msg.u.frame_msg.link_hdl = ctx_isp->base->link_hdl;
	req_msg.u.frame_msg.sof_status = sof_event_status;
	req_msg.u.frame_msg.frame_id_meta = ctx_isp->frame_id_meta;

	CAM_DBG(CAM_ISP,
		"request id:%lld frame number:%lld SOF time stamp:0x%llx status:%u",
		 request_id, ctx_isp->frame_id,
		ctx_isp->sof_timestamp_val, sof_event_status);

	if (cam_req_mgr_notify_message(&req_msg,
		V4L_EVENT_CAM_REQ_MGR_SOF, V4L_EVENT_CAM_REQ_MGR_EVENT))
		CAM_ERR(CAM_ISP,
			"Error in notifying the sof time for req id:%lld",
			request_id);

end:
	__cam_isp_ctx_send_sof_boot_timestamp(ctx_isp,
		request_id, sof_event_status);
}

static void __cam_isp_ctx_handle_buf_done_fail_log(
	uint64_t request_id, struct cam_isp_ctx_req *req_isp,
	uint32_t isp_device_type)
{
	int i;
	const char *handle_type;

	if (req_isp->num_fence_map_out >= CAM_ISP_CTX_RES_MAX) {
		CAM_ERR(CAM_ISP,
			"Num Resources exceed mMAX %d >= %d ",
			req_isp->num_fence_map_out, CAM_ISP_CTX_RES_MAX);
		return;
	}

	CAM_WARN_RATE_LIMIT(CAM_ISP,
		"Prev Req[%lld] : num_out=%d, num_acked=%d, bubble : report=%d, detected=%d",
		request_id, req_isp->num_fence_map_out, req_isp->num_acked,
		req_isp->bubble_report, req_isp->bubble_detected);
	CAM_WARN_RATE_LIMIT(CAM_ISP,
		"Resource Handles that fail to generate buf_done in prev frame");
	for (i = 0; i < req_isp->num_fence_map_out; i++) {
		if (req_isp->fence_map_out[i].sync_id != -1) {
			if (isp_device_type == CAM_IFE_DEVICE_TYPE) {
				handle_type =
				__cam_isp_resource_handle_id_to_type(
				req_isp->fence_map_out[i].resource_handle);

				trace_cam_log_event("Buf_done Congestion",
				__cam_isp_resource_handle_id_to_type(
				req_isp->fence_map_out[i].resource_handle),
				request_id, req_isp->fence_map_out[i].sync_id);
			} else {
				handle_type =
				__cam_isp_tfe_resource_handle_id_to_type(
				req_isp->fence_map_out[i].resource_handle);

				trace_cam_log_event("Buf_done Congestion",
				__cam_isp_tfe_resource_handle_id_to_type(
				req_isp->fence_map_out[i].resource_handle),
				request_id, req_isp->fence_map_out[i].sync_id);
			}

			CAM_WARN_RATE_LIMIT(CAM_ISP,
			"Resource_Handle: [%s][0x%x] Sync_ID: [0x%x]",
			handle_type,
			req_isp->fence_map_out[i].resource_handle,
			req_isp->fence_map_out[i].sync_id);
		}
	}
}

static int __cam_isp_ctx_handle_buf_done_for_req_list(
	struct cam_isp_context *ctx_isp,
	struct cam_ctx_request *req)
{
	int rc = 0, i;
	uint64_t buf_done_req_id;
	struct cam_isp_ctx_req *req_isp;
	struct cam_context *ctx = ctx_isp->base;

	req_isp = (struct cam_isp_ctx_req *) req->req_priv;
	ctx_isp->active_req_cnt--;
	buf_done_req_id = req->request_id;

	if (req_isp->bubble_detected && req_isp->bubble_report) {
		req_isp->num_acked = 0;
		req_isp->num_deferred_acks = 0;
		req_isp->bubble_detected = false;
		list_del_init(&req->list);
		atomic_set(&ctx_isp->process_bubble, 0);
		req_isp->cdm_reset_before_apply = false;
		ctx_isp->bubble_frame_cnt = 0;

		if (buf_done_req_id <= ctx->last_flush_req) {
			for (i = 0; i < req_isp->num_fence_map_out; i++)
				rc = cam_sync_signal(
					req_isp->fence_map_out[i].sync_id,
					CAM_SYNC_STATE_SIGNALED_ERROR,
					CAM_SYNC_ISP_EVENT_BUBBLE);

			list_add_tail(&req->list, &ctx->free_req_list);
			CAM_DBG(CAM_REQ,
				"Move active request %lld to free list(cnt = %d) [flushed], ctx %u",
				buf_done_req_id, ctx_isp->active_req_cnt,
				ctx->ctx_id);
			ctx_isp->last_bufdone_err_apply_req_id = 0;
		} else {
			list_add(&req->list, &ctx->pending_req_list);
			CAM_DBG(CAM_REQ,
				"Move active request %lld to pending list(cnt = %d) [bubble recovery], ctx %u",
				req->request_id, ctx_isp->active_req_cnt,
				ctx->ctx_id);
		}
	} else {
		if (!ctx_isp->use_frame_header_ts) {
			if (ctx_isp->reported_req_id < buf_done_req_id) {
				ctx_isp->reported_req_id = buf_done_req_id;
				__cam_isp_ctx_send_sof_timestamp(ctx_isp,
					buf_done_req_id,
					CAM_REQ_MGR_SOF_EVENT_SUCCESS);
			}
		}
		list_del_init(&req->list);
		list_add_tail(&req->list, &ctx->free_req_list);
		req_isp->reapply = false;
		req_isp->cdm_reset_before_apply = false;
		req_isp->num_acked = 0;
		req_isp->num_deferred_acks = 0;
		/*
		 * Only update the process_bubble and bubble_frame_cnt
		 * when bubble is detected on this req, in case the other
		 * request is processing bubble.
		 */
		if (req_isp->bubble_detected) {
			atomic_set(&ctx_isp->process_bubble, 0);
			ctx_isp->bubble_frame_cnt = 0;
			req_isp->bubble_detected = false;
		}

		CAM_DBG(CAM_REQ,
			"Move active request %lld to free list(cnt = %d) [all fences done], ctx %u",
			buf_done_req_id, ctx_isp->active_req_cnt, ctx->ctx_id);
		ctx_isp->req_info.last_bufdone_req_id = req->request_id;
		ctx_isp->last_bufdone_err_apply_req_id = 0;
	}

	cam_cpas_notify_event("IFE BufDone", buf_done_req_id);

	__cam_isp_ctx_update_state_monitor_array(ctx_isp,
		CAM_ISP_STATE_CHANGE_TRIGGER_DONE, buf_done_req_id);

	__cam_isp_ctx_update_event_record(ctx_isp,
		CAM_ISP_CTX_EVENT_BUFDONE, req);
	return rc;
}

static int __cam_isp_ctx_handle_early_buf_done(
	struct cam_isp_context *ctx_isp,
	struct cam_ctx_request  *req)
{
	int i, rc = 0;

	struct cam_isp_ctx_req *req_isp = (struct cam_isp_ctx_req *) req->req_priv;
	struct cam_context *ctx = ctx_isp->base;

	for (i = 0; i < req_isp->num_fence_map_out; i++) {
		if (req_isp->early_fence_map_index[i] == 1) {
			rc = cam_sync_signal(req_isp->fence_map_out[i].sync_id,
				CAM_SYNC_STATE_SIGNALED_SUCCESS,
				CAM_SYNC_COMMON_EVENT_SUCCESS);
			req_isp->early_fence_map_index[i] = 0;
			req_isp->fence_map_out[i].sync_id = -1;
			CAM_DBG(CAM_ISP,
				"Sync :req %lld res 0x%x fd 0x%x idx %d ctx %u",
				req->request_id,
				req_isp->fence_map_out[i].resource_handle,
				req_isp->fence_map_out[i].sync_id, i,
				ctx->ctx_id);
			req_isp->flag_sync_set = true;
		}
	}
	return rc;
}
static int __cam_isp_ctx_handle_buf_done_for_request(
	struct cam_isp_context *ctx_isp,
	struct cam_ctx_request  *req,
	struct cam_isp_hw_done_event_data *done,
	uint32_t bubble_state,
	struct cam_isp_hw_done_event_data *done_next_req)
{
	int rc = 0;
	int i, j;
	struct cam_isp_ctx_req *req_isp;
	struct cam_context *ctx = ctx_isp->base;
	const char *handle_type;

	trace_cam_buf_done("ISP", ctx, req);

	req_isp = (struct cam_isp_ctx_req *) req->req_priv;

	CAM_DBG(CAM_ISP,
		"Enter with bubble_state %d, req_bubble_detected %d evt_param = %d",
		bubble_state, req_isp->bubble_detected, done->evt_param);

	done_next_req->num_handles = 0;
	done_next_req->timestamp = done->timestamp;

	for (i = 0; i < done->num_handles; i++) {
		for (j = 0; j < req_isp->num_fence_map_out; j++) {
			if (done->resource_handle[i] ==
				req_isp->fence_map_out[j].resource_handle)
				break;
		}

		if (j == req_isp->num_fence_map_out) {
			/*
			 * If not found in current request, it could be
			 * belonging to next request, this can happen if
			 * IRQ delay happens. It is only valid when the
			 * platform doesn't have last consumed address.
			 */
			CAM_WARN(CAM_ISP,
				"BUF_DONE for res %s not found in Req %lld ",
				__cam_isp_resource_handle_id_to_type(
				done->resource_handle[i]),
				req->request_id);

			done_next_req->resource_handle
				[done_next_req->num_handles++] =
				done->resource_handle[i];
			continue;
		}

		if (req_isp->fence_map_out[j].sync_id == -1) {
			if (ctx_isp->isp_device_type == CAM_IFE_DEVICE_TYPE)
				handle_type =
				__cam_isp_resource_handle_id_to_type(
				req_isp->fence_map_out[i].resource_handle);
			else
				handle_type =
				__cam_isp_tfe_resource_handle_id_to_type(
				req_isp->fence_map_out[i].resource_handle);

			CAM_WARN(CAM_ISP,
				"Duplicate BUF_DONE for req %lld : i=%d, j=%d, res=%s",
				req->request_id, i, j, handle_type);

			trace_cam_log_event("Duplicate BufDone",
				handle_type, req->request_id, ctx->ctx_id);

			done_next_req->resource_handle
				[done_next_req->num_handles++] =
				done->resource_handle[i];
			continue;
		}

		if (!req_isp->bubble_detected) {
			/* Signal fence for early bufdones got during reapply of a bubble req*/
			if ((req_isp->num_acked != 0) && (!req_isp->flag_sync_set)) {
				rc = __cam_isp_ctx_handle_early_buf_done(ctx_isp, req);
				if (rc)
					CAM_ERR(CAM_ISP, "Sync early bufdone failed rc=%d", rc);
			}

			CAM_DBG(CAM_ISP,
				"Sync with success: req %lld res 0x%x fd 0x%x, ctx %u, Bad_frame %u",
				req->request_id,
				req_isp->fence_map_out[j].resource_handle,
				req_isp->fence_map_out[j].sync_id,
				ctx->ctx_id, done->evt_param);

			if (done->evt_param == 1) {

				CAM_WARN(CAM_ISP,
					"Bad frame Sync with success: req %lld res 0x%x fd 0x%x, ctx %u",
					req->request_id,
					req_isp->fence_map_out[j].resource_handle,
					req_isp->fence_map_out[j].sync_id,
					ctx->ctx_id);
				rc = cam_sync_signal(req_isp->fence_map_out[j].sync_id,
					CAM_SYNC_STATE_SIGNALED_SUCCESS,
					CAM_SYNC_ISP_EVENT_BAD_FRAME);
			} else {
				rc = cam_sync_signal(req_isp->fence_map_out[j].sync_id,
					CAM_SYNC_STATE_SIGNALED_SUCCESS,
					CAM_SYNC_COMMON_EVENT_SUCCESS);
			}
			if (rc)
				CAM_DBG(CAM_ISP, "Sync failed with rc = %d",
					rc);
		} else if (!req_isp->bubble_report) {
			CAM_DBG(CAM_ISP,
				"Sync with failure: req %lld res 0x%x fd 0x%x, ctx %u",
				req->request_id,
				req_isp->fence_map_out[j].resource_handle,
				req_isp->fence_map_out[j].sync_id,
				ctx->ctx_id);

			rc = cam_sync_signal(req_isp->fence_map_out[j].sync_id,
				CAM_SYNC_STATE_SIGNALED_ERROR,
				CAM_SYNC_ISP_EVENT_BUBBLE);
			if (rc)
				CAM_ERR(CAM_ISP, "Sync failed with rc = %d",
					rc);
		} else {
			/*
			 * Ignore the buffer done if bubble detect is on
			 * Increment the ack number here, and queue the
			 * request back to pending list whenever all the
			 * buffers are done.
			 */
			req_isp->num_acked++;
			CAM_DBG(CAM_ISP,
				"buf done with bubble state %d recovery %d num_acked %d ctx %u",
				bubble_state, req_isp->bubble_report,
				req_isp->num_acked, ctx->ctx_id);
			continue;
		}

		if (!rc) {
			req_isp->num_acked++;
			req_isp->fence_map_out[j].sync_id = -1;
		}
		CAM_DBG(CAM_ISP, "rc %d num_acked %d req %lld, reset sync id 0x%x ctx %u",
			rc, req_isp->num_acked, req->request_id,
			req_isp->fence_map_out[j].sync_id, ctx->ctx_id);

		if ((ctx_isp->use_frame_header_ts) &&
			(req_isp->hw_update_data.frame_header_res_id ==
			req_isp->fence_map_out[j].resource_handle))
			__cam_isp_ctx_send_sof_timestamp_frame_header(
				ctx_isp,
				req_isp->hw_update_data.frame_header_cpu_addr,
				req->request_id, CAM_REQ_MGR_SOF_EVENT_SUCCESS);
	}

	if (req_isp->num_acked > req_isp->num_fence_map_out) {
		/* Should not happen */
		CAM_ERR(CAM_ISP,
			"WARNING: req_id %lld num_acked %d > map_out %d, ctx %u",
			req->request_id, req_isp->num_acked,
			req_isp->num_fence_map_out, ctx->ctx_id);
		WARN_ON(req_isp->num_acked > req_isp->num_fence_map_out);
	}

	if (req_isp->num_acked != req_isp->num_fence_map_out)
		return rc;

	rc = __cam_isp_ctx_handle_buf_done_for_req_list(ctx_isp, req);
	return rc;
}

static int __cam_isp_handle_deferred_buf_done(
	struct cam_isp_context *ctx_isp,
	struct cam_ctx_request  *req,
	bool bubble_handling,
	uint32_t status, uint32_t event_cause)
{
	int i, j;
	int rc = 0;
	struct cam_isp_ctx_req *req_isp =
		(struct cam_isp_ctx_req *) req->req_priv;
	struct cam_context *ctx = ctx_isp->base;

	CAM_DBG(CAM_ISP,
		"ctx[%d] : Req %llu : Handling %d deferred buf_dones num_acked=%d, bubble_handling=%d",
		ctx->ctx_id, req->request_id, req_isp->num_deferred_acks,
		req_isp->num_acked, bubble_handling);

	for (i = 0; i < req_isp->num_deferred_acks; i++) {
		j = req_isp->deferred_fence_map_index[i];

		CAM_DBG(CAM_ISP,
			"ctx[%d] : Sync with status=%d, event_cause=%d: req %lld res 0x%x sync_id 0x%x",
			ctx->ctx_id, status, event_cause,
			req->request_id,
			req_isp->fence_map_out[j].resource_handle,
			req_isp->fence_map_out[j].sync_id);

		if (req_isp->fence_map_out[j].sync_id == -1) {
			CAM_WARN(CAM_ISP,
				"ctx[%d Deferred buf_done already signalled, req_id=%llu, j=%d, res=0x%x",
				ctx->ctx_id, req->request_id, j,
				req_isp->fence_map_out[j].resource_handle);
			continue;
		}

		if (!bubble_handling) {
			CAM_WARN(CAM_ISP,
				"ctx[%d] : Req %llu, status=%d res=0x%x should never happen",
				ctx->ctx_id, req->request_id, status,
				req_isp->fence_map_out[j].resource_handle);

			rc = cam_sync_signal(req_isp->fence_map_out[j].sync_id,
				status, event_cause);
			if (rc) {
				CAM_ERR(CAM_ISP,
					"ctx[%d] : Sync signal for Req %llu, sync_id %d status=%d failed with rc = %d",
					ctx->ctx_id, req->request_id,
					req_isp->fence_map_out[j].sync_id,
					status, rc);
			} else {
				req_isp->num_acked++;
				req_isp->fence_map_out[j].sync_id = -1;
			}
		} else {
			req_isp->num_acked++;
		}
	}

	req_isp->num_deferred_acks = 0;

	return rc;
}
static int __cam_isp_ctx_handle_buf_done_for_request_verify_addr(
	struct cam_isp_context *ctx_isp,
	struct cam_ctx_request  *req,
	struct cam_isp_hw_done_event_data *done,
	uint32_t bubble_state,
	bool verify_consumed_addr,
	bool defer_buf_done)
{
	int rc = 0;
	int i, j;
	struct cam_isp_ctx_req *req_isp;
	struct cam_context *ctx = ctx_isp->base;
	const char *handle_type;
	uint32_t event_cause = CAM_SYNC_COMMON_EVENT_SUCCESS;

	trace_cam_buf_done("ISP", ctx, req);

	req_isp = (struct cam_isp_ctx_req *) req->req_priv;

	CAM_DBG(CAM_ISP,
		"Enter with bubble_state %d, req_bubble_detected %d evt_param %d",
		bubble_state, req_isp->bubble_detected, done->evt_param);

	for (i = 0; i < done->num_handles; i++) {
		for (j = 0; j < req_isp->num_fence_map_out; j++) {
			if (verify_consumed_addr &&
				(done->last_consumed_addr[i] !=
				 req_isp->fence_map_out[j].image_buf_addr[0]))
				continue;

			if (done->resource_handle[i] ==
				req_isp->fence_map_out[j].resource_handle)
				break;
		}

		if (j == req_isp->num_fence_map_out) {
			/*
			 * If not found in current request, it could be
			 * belonging to next request, this can happen if
			 * IRQ delay happens. It is only valid when the
			 * platform doesn't have last consumed address.
			 */
			CAM_DBG(CAM_ISP,
				"BUF_DONE for res %s not found in Req %lld ",
				__cam_isp_resource_handle_id_to_type(
				done->resource_handle[i]),
				req->request_id);
			continue;
		}

		if (req_isp->fence_map_out[j].sync_id == -1) {
			if (ctx_isp->isp_device_type == CAM_IFE_DEVICE_TYPE)
				handle_type =
				__cam_isp_resource_handle_id_to_type(
				req_isp->fence_map_out[i].resource_handle);
			else
				handle_type =
				__cam_isp_tfe_resource_handle_id_to_type(
				req_isp->fence_map_out[i].resource_handle);

			CAM_WARN(CAM_ISP,
				"Duplicate BUF_DONE for req %lld : i=%d, j=%d, res=%s",
				req->request_id, i, j, handle_type);

			trace_cam_log_event("Duplicate BufDone",
				handle_type, req->request_id, ctx->ctx_id);
			continue;
		}

		if (defer_buf_done) {
			uint32_t deferred_indx = req_isp->num_deferred_acks;

			/*
			 * If we are handling this BUF_DONE event for a request
			 * that is still in wait_list, do not signal now,
			 * instead mark it as done and handle it later -
			 * if this request is going into BUBBLE state later
			 * it will automatically be re-applied. If this is not
			 * going into BUBBLE, signal fences later.
			 * Note - we will come here only if the last consumed
			 * address matches with this ports buffer.
			 */
			req_isp->deferred_fence_map_index[deferred_indx] = j;
			req_isp->num_deferred_acks++;
			CAM_DBG(CAM_ISP,
				"ctx[%d] : Deferred buf done for %llu with bubble state %d recovery %d",
				ctx->ctx_id, req->request_id, bubble_state,
				req_isp->bubble_report);
			CAM_DBG(CAM_ISP,
				"ctx[%d] : Deferred info : num_acks=%d, fence_map_index=%d, resource_handle=0x%x, sync_id=%d",
				ctx->ctx_id, req_isp->num_deferred_acks, j,
				req_isp->fence_map_out[j].resource_handle,
				req_isp->fence_map_out[j].sync_id);
			continue;
		} else if (!req_isp->bubble_detected) {
			CAM_DBG(CAM_ISP,
				"Sync with success: req %lld res 0x%x fd 0x%x, ctx %u isBadFrame %u",
				req->request_id,
				req_isp->fence_map_out[j].resource_handle,
				req_isp->fence_map_out[j].sync_id,
				ctx->ctx_id, done->evt_param);
			if (done->evt_param == 1) {
				CAM_WARN(CAM_ISP,
					"Bad Frame Sync with success: req %lld res 0x%x fd 0x%x, ctx %u",
					req->request_id,
					req_isp->fence_map_out[j].resource_handle,
					req_isp->fence_map_out[j].sync_id,
					ctx->ctx_id);
				rc = cam_sync_signal(req_isp->fence_map_out[j].sync_id,
					CAM_SYNC_STATE_SIGNALED_SUCCESS,
					CAM_SYNC_ISP_EVENT_BAD_FRAME);
				event_cause = CAM_SYNC_ISP_EVENT_BAD_FRAME;
			} else {
				rc = cam_sync_signal(req_isp->fence_map_out[j].sync_id,
					CAM_SYNC_STATE_SIGNALED_SUCCESS,
					CAM_SYNC_COMMON_EVENT_SUCCESS);
				event_cause = CAM_SYNC_COMMON_EVENT_SUCCESS;
			}
			if (rc) {
				CAM_DBG(CAM_ISP, "Sync failed with rc = %d",
				 rc);
			} else if (req_isp->num_deferred_acks) {
				/* Process deferred buf_done acks */
				__cam_isp_handle_deferred_buf_done(ctx_isp,
					req, false,
					CAM_SYNC_STATE_SIGNALED_SUCCESS,
					event_cause);
			}
		} else if (!req_isp->bubble_report) {
			CAM_DBG(CAM_ISP,
				"Sync with failure: req %lld res 0x%x fd 0x%x, ctx %u",
				req->request_id,
				req_isp->fence_map_out[j].resource_handle,
				req_isp->fence_map_out[j].sync_id,
				ctx->ctx_id);

			rc = cam_sync_signal(req_isp->fence_map_out[j].sync_id,
				CAM_SYNC_STATE_SIGNALED_ERROR,
				CAM_SYNC_ISP_EVENT_BUBBLE);
			if (rc) {
				CAM_ERR(CAM_ISP, "Sync failed with rc = %d",
					rc);
			} else if (req_isp->num_deferred_acks) {
				/* Process deferred buf_done acks */
				__cam_isp_handle_deferred_buf_done(ctx_isp, req,
					false,
					CAM_SYNC_STATE_SIGNALED_ERROR,
					CAM_SYNC_ISP_EVENT_BUBBLE);
			}
		} else {
			/*
			 * Ignore the buffer done if bubble detect is on
			 * Increment the ack number here, and queue the
			 * request back to pending list whenever all the
			 * buffers are done.
			 */
			req_isp->num_acked++;
			CAM_DBG(CAM_ISP,
				"num_acked %d buf done with bubble state %d recovery %d ctx %d",
				req_isp->num_acked, bubble_state,
				req_isp->bubble_report,
				ctx->ctx_id);
				/* Process deferred buf_done acks */

			if (req_isp->num_deferred_acks)
				__cam_isp_handle_deferred_buf_done(ctx_isp, req,
					true,
					CAM_SYNC_STATE_SIGNALED_ERROR,
					CAM_SYNC_ISP_EVENT_BUBBLE);

			continue;
		}

		if (!rc) {
			req_isp->num_acked++;
			req_isp->fence_map_out[j].sync_id = -1;
		}
		CAM_DBG(CAM_ISP, "num_acked %d req %lld, reset sync id 0x%x ctx %u",
			req_isp->num_acked, req->request_id,
			req_isp->fence_map_out[j].sync_id, ctx->ctx_id);

		if ((ctx_isp->use_frame_header_ts) &&
			(req_isp->hw_update_data.frame_header_res_id ==
			req_isp->fence_map_out[j].resource_handle))
			__cam_isp_ctx_send_sof_timestamp_frame_header(
				ctx_isp,
				req_isp->hw_update_data.frame_header_cpu_addr,
				req->request_id, CAM_REQ_MGR_SOF_EVENT_SUCCESS);
	}

	if (req_isp->num_acked > req_isp->num_fence_map_out) {
		/* Should not happen */
		CAM_ERR(CAM_ISP,
			"WARNING: req_id %lld num_acked %d > map_out %d, ctx %u",
			req->request_id, req_isp->num_acked,
			req_isp->num_fence_map_out, ctx->ctx_id);
	}

	if (req_isp->num_acked != req_isp->num_fence_map_out)
		return rc;

	rc = __cam_isp_ctx_handle_buf_done_for_req_list(ctx_isp, req);
	return rc;
}

static int __cam_isp_ctx_handle_buf_done(
	struct cam_isp_context *ctx_isp,
	struct cam_isp_hw_done_event_data *done,
	uint32_t bubble_state)
{
	int rc = 0;
	struct cam_ctx_request *req;
	struct cam_context *ctx = ctx_isp->base;
	struct cam_isp_hw_done_event_data done_next_req;
	int i, j;

	if (list_empty(&ctx->active_req_list)) {
		CAM_WARN(CAM_ISP, "Buf done with no active request");

		if (!list_empty(&ctx->wait_req_list)) {
			struct cam_isp_ctx_req *req_isp;

			req = list_first_entry(&ctx->wait_req_list,
					struct cam_ctx_request, list);

			if (ctx_isp->last_applied_req_id !=
				ctx_isp->last_bufdone_err_apply_req_id) {
				CAM_WARN(CAM_ISP,
					"Buf done with req %llu in wait list apply id:%lld last err id:%lld",
					req->request_id,
					ctx_isp->last_applied_req_id,
					ctx_isp->last_bufdone_err_apply_req_id);
				ctx_isp->last_bufdone_err_apply_req_id =
					ctx_isp->last_applied_req_id;
			}

			req_isp = (struct cam_isp_ctx_req *) req->req_priv;

			/*
			 * do not signal the fence as this request may go into
			 * Bubble state eventually.
			 */
			for (i = 0; i < done->num_handles; i++) {
				for (j = 0; j < req_isp->num_fence_map_out; j++) {
					if (done->resource_handle[i] ==
						req_isp->fence_map_out[j].resource_handle) {
						req_isp->num_acked++;
						/*
						 * save the fence map out index for signalling
						 * fence during re-apply of bubble request.
						 */
						req_isp->early_fence_map_index[j]++;

						CAM_WARN(CAM_ISP, "Early done req %lld res 0x%x",
							req->request_id,
							done->resource_handle[i]);

						CAM_WARN(CAM_ISP, "ctx %u ack %d total %d idx %d",
							ctx->ctx_id,
							req_isp->num_acked,
							req_isp->num_fence_map_out, j);
						break;
					}
				}
			}
		} else {
			CAM_WARN(CAM_ISP, "Buf done with no request in wait as well");
		}
		return 0;
	}

	req = list_first_entry(&ctx->active_req_list,
			struct cam_ctx_request, list);

	rc = __cam_isp_ctx_handle_buf_done_for_request(ctx_isp, req, done,
		bubble_state, &done_next_req);

	if (done_next_req.num_handles) {
		struct cam_isp_hw_done_event_data unhandled_res;
		struct cam_ctx_request  *next_req = list_last_entry(
			&ctx->active_req_list, struct cam_ctx_request, list);

		if (next_req->request_id != req->request_id) {
			/*
			 * Few resource handles are already signalled in the
			 * current request, lets check if there is another
			 * request waiting for these resources. This can
			 * happen if handling some of next request's buf done
			 * events are happening first before handling current
			 * request's remaining buf dones due to IRQ scheduling.
			 * Lets check only one more request as we will have
			 * maximum of 2 requests in active_list at any time.
			 */

			CAM_WARN(CAM_ISP,
				"Unhandled buf done resources for req %lld, trying next request %lld in active_list",
				req->request_id, next_req->request_id);

			__cam_isp_ctx_handle_buf_done_for_request(ctx_isp,
				next_req, &done_next_req,
				bubble_state, &unhandled_res);

			if (unhandled_res.num_handles == 0)
				CAM_INFO(CAM_ISP,
					"BUF Done event handed for next request %lld",
					next_req->request_id);
			else
				CAM_ERR(CAM_ISP,
					"BUF Done not handled for next request %lld",
					next_req->request_id);
		} else {
			CAM_WARN(CAM_ISP,
				"Req %lld only active request, spurious buf_done rxd",
				req->request_id);
		}
	}

	return rc;
}

static void __cam_isp_ctx_buf_done_match_req(
	struct cam_ctx_request *req,
	struct cam_isp_hw_done_event_data *done,
	bool *irq_delay_detected)
{
	int i, j;
	uint32_t match_count = 0;
	struct cam_isp_ctx_req *req_isp;

	req_isp = (struct cam_isp_ctx_req *) req->req_priv;

	for (i = 0; i < done->num_handles; i++) {
		for (j = 0; j < req_isp->num_fence_map_out; j++) {
			if ((done->resource_handle[i] ==
				 req_isp->fence_map_out[j].resource_handle) &&
				(done->last_consumed_addr[i] ==
				 req_isp->fence_map_out[j].image_buf_addr[0])) {
				match_count++;
				break;
			}
		}
	}

	if (match_count > 0)
		*irq_delay_detected = true;
	else
		*irq_delay_detected = false;

	CAM_DBG(CAM_ISP,
		"buf done num handles %d match count %d for next req:%lld",
		done->num_handles, match_count, req->request_id);
	CAM_DBG(CAM_ISP,
		"irq_delay_detected %d", *irq_delay_detected);
}

static int __cam_isp_ctx_handle_buf_done_verify_addr(
	struct cam_isp_context *ctx_isp,
	struct cam_isp_hw_done_event_data *done,
	uint32_t bubble_state)
{
	int rc = 0;
	bool irq_delay_detected = false;
	struct cam_ctx_request *req;
	struct cam_ctx_request *next_req = NULL;
	struct cam_context *ctx = ctx_isp->base;
	bool  req_in_wait_list = false;

	if (list_empty(&ctx->active_req_list)) {

		if (!list_empty(&ctx->wait_req_list)) {
			struct cam_isp_ctx_req *req_isp;

			req = list_first_entry(&ctx->wait_req_list,
				struct cam_ctx_request, list);

			req_in_wait_list = true;
			if (ctx_isp->last_applied_req_id !=
				ctx_isp->last_bufdone_err_apply_req_id) {
				CAM_WARN(CAM_ISP,
					"Buf done with no active request but with req in wait list, req %llu last apply id:%lld last err id:%lld",
					req->request_id,
					ctx_isp->last_applied_req_id,
					ctx_isp->last_bufdone_err_apply_req_id);
				ctx_isp->last_bufdone_err_apply_req_id =
					ctx_isp->last_applied_req_id;
			}

			req_isp = (struct cam_isp_ctx_req *) req->req_priv;

			/*
			 * Verify consumed address for this request to make sure
			 * we are handling the buf_done for the correct
			 * buffer. Also defer actual buf_done handling, i.e
			 * do not signal the fence as this request may go into
			 * Bubble state eventully.
			 */
			rc =
			__cam_isp_ctx_handle_buf_done_for_request_verify_addr(
				ctx_isp, req, done, bubble_state, true, true);
		}

		if (!req_in_wait_list  && (ctx_isp->last_applied_req_id !=
			ctx_isp->last_bufdone_err_apply_req_id)) {
			CAM_WARN(CAM_ISP,
				"Buf done with no active request bubble_state=%d last_applied_req_id:%lld ",
				bubble_state, ctx_isp->last_applied_req_id);
			ctx_isp->last_bufdone_err_apply_req_id =
					ctx_isp->last_applied_req_id;
		}
		return 0;
	}

	req = list_first_entry(&ctx->active_req_list,
			struct cam_ctx_request, list);

	if (ctx_isp->active_req_cnt > 1) {
		next_req = list_last_entry(
			&ctx->active_req_list,
			struct cam_ctx_request, list);

		if (next_req->request_id != req->request_id)
			__cam_isp_ctx_buf_done_match_req(next_req, done,
				&irq_delay_detected);
		else
			CAM_WARN(CAM_ISP,
				"Req %lld only active request, spurious buf_done rxd",
				req->request_id);
	}

	/*
	 * If irq delay isn't detected, then we need to verify
	 * the consumed address for current req, otherwise, we
	 * can't verify the consumed address.
	 */
	rc = __cam_isp_ctx_handle_buf_done_for_request_verify_addr(
		ctx_isp, req, done, bubble_state,
		!irq_delay_detected, false);

	/*
	 * Verify the consumed address for next req all the time,
	 * since the reported buf done event may belong to current
	 * req, then we can't signal this event for next req.
	 */
	if (!rc && irq_delay_detected)
		rc = __cam_isp_ctx_handle_buf_done_for_request_verify_addr(
			ctx_isp, next_req, done,
			bubble_state, true, false);

	return rc;
}

static int __cam_isp_ctx_handle_buf_done_in_activated_state(
	struct cam_isp_context *ctx_isp,
	struct cam_isp_hw_done_event_data *done,
	uint32_t bubble_state)
{
	int rc = 0;

	if (ctx_isp->support_consumed_addr)
		rc = __cam_isp_ctx_handle_buf_done_verify_addr(
			ctx_isp, done, bubble_state);
	else
		rc = __cam_isp_ctx_handle_buf_done(
			ctx_isp, done, bubble_state);

	return rc;
}

static int __cam_isp_ctx_apply_req_offline(
	void *priv, void *data)
{
	int rc = 0;
	int64_t prev_applied_req;
	struct cam_context *ctx = NULL;
	struct cam_isp_context *ctx_isp = priv;
	struct cam_ctx_request *req;
	struct cam_isp_ctx_req *req_isp;
	struct cam_hw_config_args cfg;

	if (!ctx_isp) {
		CAM_ERR(CAM_ISP, "Invalid ctx_isp:%pK", ctx);
		rc = -EINVAL;
		goto end;
	}
	ctx = ctx_isp->base;

	if (list_empty(&ctx->pending_req_list)) {
		CAM_DBG(CAM_ISP, "No pending requests to apply");
		rc = -EFAULT;
		goto end;
	}

	if ((ctx->state != CAM_CTX_ACTIVATED) ||
		(!atomic_read(&ctx_isp->rxd_epoch)) ||
		(ctx_isp->substate_activated == CAM_ISP_CTX_ACTIVATED_APPLIED))
		goto end;

	if (ctx_isp->active_req_cnt >= 2)
		goto end;

	spin_lock_bh(&ctx->lock);
	req = list_first_entry(&ctx->pending_req_list, struct cam_ctx_request,
		list);
	spin_unlock_bh(&ctx->lock);

	CAM_DBG(CAM_REQ, "Apply request %lld in substate %d ctx %u",
		req->request_id, ctx_isp->substate_activated, ctx->ctx_id);
	req_isp = (struct cam_isp_ctx_req *) req->req_priv;

	memset(&cfg, 0, sizeof(cfg));

	cfg.ctxt_to_hw_map = ctx_isp->hw_ctx;
	cfg.request_id = req->request_id;
	cfg.hw_update_entries = req_isp->cfg;
	cfg.num_hw_update_entries = req_isp->num_cfg;
	cfg.priv  = &req_isp->hw_update_data;
	cfg.init_packet = 0;

	/*
	 * Offline mode may receive the SOF and REG_UPD earlier than
	 * CDM processing return back, so we set the substate before
	 * apply setting.
	 */
	spin_lock_bh(&ctx->lock);

	atomic_set(&ctx_isp->rxd_epoch, 0);
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_APPLIED;
	prev_applied_req = ctx_isp->last_applied_req_id;
	ctx_isp->last_applied_req_id = req->request_id;
	atomic_set(&ctx_isp->apply_in_progress, 1);

	list_del_init(&req->list);
	list_add_tail(&req->list, &ctx->wait_req_list);

	spin_unlock_bh(&ctx->lock);

	rc = ctx->hw_mgr_intf->hw_config(ctx->hw_mgr_intf->hw_mgr_priv, &cfg);
	if (rc) {
		CAM_ERR_RATE_LIMIT(CAM_ISP, "Can not apply the configuration");
		spin_lock_bh(&ctx->lock);

		ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;
		ctx_isp->last_applied_req_id = prev_applied_req;
		atomic_set(&ctx_isp->apply_in_progress, 0);

		list_del_init(&req->list);
		list_add(&req->list, &ctx->pending_req_list);

		spin_unlock_bh(&ctx->lock);
	} else {
		atomic_set(&ctx_isp->apply_in_progress, 0);
		CAM_DBG(CAM_ISP, "New substate state %d, applied req %lld",
			CAM_ISP_CTX_ACTIVATED_APPLIED,
			ctx_isp->last_applied_req_id);

		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_APPLIED,
			req->request_id);
	}

end:
	return rc;
}

static int __cam_isp_ctx_schedule_apply_req_offline(
	struct cam_isp_context *ctx_isp)
{
	int rc = 0;
	struct crm_workq_task *task;

	task = cam_req_mgr_workq_get_task(ctx_isp->workq);
	if (!task) {
		CAM_ERR(CAM_ISP, "No task for worker");
		return -ENOMEM;
	}

	task->process_cb = __cam_isp_ctx_apply_req_offline;
	rc = cam_req_mgr_workq_enqueue_task(task, ctx_isp, CRM_TASK_PRIORITY_0);
	if (rc)
		CAM_ERR(CAM_ISP, "Failed to schedule task rc:%d", rc);

	return rc;
}

static int __cam_isp_ctx_offline_epoch_in_activated_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	struct cam_context *ctx = ctx_isp->base;
	struct cam_ctx_request *req, *req_temp;
	uint64_t request_id = 0;

	atomic_set(&ctx_isp->rxd_epoch, 1);

	CAM_DBG(CAM_ISP, "SOF frame %lld ctx %u", ctx_isp->frame_id,
		ctx->ctx_id);

	list_for_each_entry_safe(req, req_temp, &ctx->active_req_list, list) {
		if (req->request_id > ctx_isp->reported_req_id) {
			request_id = req->request_id;
			ctx_isp->reported_req_id = request_id;
			break;
		}
	}

	__cam_isp_ctx_schedule_apply_req_offline(ctx_isp);

	__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
		CAM_REQ_MGR_SOF_EVENT_SUCCESS);

	__cam_isp_ctx_update_state_monitor_array(ctx_isp,
		CAM_ISP_STATE_CHANGE_TRIGGER_EPOCH,
		request_id);

	return 0;
}

static int __cam_isp_ctx_reg_upd_in_epoch_bubble_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	if (ctx_isp->frame_id == 1)
		CAM_DBG(CAM_ISP, "Reg update in Substate[%s] for early PCR",
			__cam_isp_ctx_substate_val_to_type(
			ctx_isp->substate_activated));
	else
		CAM_WARN_RATE_LIMIT(CAM_ISP,
			"ctx_id:%d Unexpected reg update in activated Substate[%s] for frame_id:%lld",
			ctx_isp->base->ctx_id,
			__cam_isp_ctx_substate_val_to_type(
			ctx_isp->substate_activated),
			ctx_isp->frame_id);
	return 0;
}

static int __cam_isp_ctx_reg_upd_in_applied_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_ctx_request  *req;
	struct cam_context      *ctx = ctx_isp->base;
	struct cam_isp_ctx_req  *req_isp;
	uint64_t                 request_id = 0;

	if (list_empty(&ctx->wait_req_list)) {
		CAM_ERR(CAM_ISP, "Reg upd ack with no waiting request");
		goto end;
	}
	req = list_first_entry(&ctx->wait_req_list,
			struct cam_ctx_request, list);
	list_del_init(&req->list);

	req_isp = (struct cam_isp_ctx_req *) req->req_priv;
	if (req_isp->num_fence_map_out != 0) {
		list_add_tail(&req->list, &ctx->active_req_list);
		ctx_isp->active_req_cnt++;
		request_id = req->request_id;
		CAM_DBG(CAM_REQ,
			"move request %lld to active list(cnt = %d), ctx %u",
			req->request_id, ctx_isp->active_req_cnt, ctx->ctx_id);
		__cam_isp_ctx_update_event_record(ctx_isp,
			CAM_ISP_CTX_EVENT_RUP, req);
	} else {
		/* no io config, so the request is completed. */
		list_add_tail(&req->list, &ctx->free_req_list);
		CAM_DBG(CAM_ISP,
			"move active request %lld to free list(cnt = %d), ctx %u",
			req->request_id, ctx_isp->active_req_cnt, ctx->ctx_id);
	}

	/*
	 * This function only called directly from applied and bubble applied
	 * state so change substate here.
	 */
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_EPOCH;
	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	__cam_isp_ctx_update_state_monitor_array(ctx_isp,
		CAM_ISP_STATE_CHANGE_TRIGGER_REG_UPDATE, request_id);

end:
	return rc;
}

static int __cam_isp_ctx_notify_sof_in_activated_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	uint64_t  request_id  = 0;
	struct cam_req_mgr_trigger_notify  notify;
	struct cam_context *ctx = ctx_isp->base;
	struct cam_ctx_request  *req;
	struct cam_isp_ctx_req  *req_isp;
	struct cam_hw_cmd_args   hw_cmd_args;
	struct cam_isp_hw_cmd_args  isp_hw_cmd_args;
	uint64_t last_cdm_done_req = 0;
	struct cam_isp_hw_epoch_event_data *epoch_done_event_data =
			(struct cam_isp_hw_epoch_event_data *)evt_data;

	if (!evt_data) {
		CAM_ERR(CAM_ISP, "invalid event data");
		return -EINVAL;
	}

	ctx_isp->frame_id_meta = epoch_done_event_data->frame_id_meta;

	if (atomic_read(&ctx_isp->process_bubble)) {
		if (list_empty(&ctx->active_req_list)) {
			CAM_ERR(CAM_ISP,
				"No available active req in bubble");
			atomic_set(&ctx_isp->process_bubble, 0);
			ctx_isp->bubble_frame_cnt = 0;
			rc = -EINVAL;
			return rc;
		}

		if (ctx_isp->last_sof_timestamp ==
			ctx_isp->sof_timestamp_val) {
			CAM_DBG(CAM_ISP,
				"Tasklet delay detected! Bubble frame check skipped, sof_timestamp: %lld",
				ctx_isp->sof_timestamp_val);
			goto notify_only;
		}

		req = list_first_entry(&ctx->active_req_list,
			struct cam_ctx_request, list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;

		if (ctx_isp->bubble_frame_cnt >= 1 &&
			req_isp->bubble_detected) {
			hw_cmd_args.ctxt_to_hw_map = ctx_isp->hw_ctx;
			hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
			isp_hw_cmd_args.cmd_type =
				CAM_ISP_HW_MGR_GET_LAST_CDM_DONE;
			hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;
			rc = ctx->hw_mgr_intf->hw_cmd(
				ctx->hw_mgr_intf->hw_mgr_priv,
				&hw_cmd_args);
			if (rc) {
				CAM_ERR(CAM_ISP, "HW command failed");
				return rc;
			}

			last_cdm_done_req = isp_hw_cmd_args.u.last_cdm_done;
			CAM_DBG(CAM_ISP, "last_cdm_done req: %d",
				last_cdm_done_req);

			if (last_cdm_done_req >= req->request_id) {
				CAM_DBG(CAM_ISP,
					"CDM callback detected for req: %lld, possible buf_done delay, waiting for buf_done",
					req->request_id);
				ctx_isp->bubble_frame_cnt = 0;
			} else {
				CAM_DBG(CAM_ISP,
					"CDM callback not happened for req: %lld, possible CDM stuck or workqueue delay",
					req->request_id);
				req_isp->num_acked = 0;
				req_isp->num_deferred_acks = 0;
				ctx_isp->bubble_frame_cnt = 0;
				req_isp->bubble_detected = false;
				req_isp->cdm_reset_before_apply = true;
				list_del_init(&req->list);
				list_add(&req->list, &ctx->pending_req_list);
				atomic_set(&ctx_isp->process_bubble, 0);
				ctx_isp->active_req_cnt--;
				CAM_DBG(CAM_REQ,
					"Move active req: %lld to pending list(cnt = %d) [bubble re-apply],ctx %u",
					req->request_id,
					ctx_isp->active_req_cnt, ctx->ctx_id);
			}
		} else if (req_isp->bubble_detected) {
			ctx_isp->bubble_frame_cnt++;
			CAM_DBG(CAM_ISP,
				"Waiting on bufdone for bubble req: %lld, since frame_cnt = %lld",
				req->request_id,
				ctx_isp->bubble_frame_cnt);
		} else {
			CAM_DBG(CAM_ISP, "Delayed bufdone for req: %lld",
				req->request_id);
		}
	}

notify_only:
	/*
	 * notify reqmgr with sof signal. Note, due to scheduling delay
	 * we can run into situation that two active requests has already
	 * be in the active queue while we try to do the notification.
	 * In this case, we need to skip the current notification. This
	 * helps the state machine to catch up the delay.
	 */
	if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_trigger &&
		ctx_isp->active_req_cnt <= 2) {
		if (ctx_isp->subscribe_event & CAM_TRIGGER_POINT_SOF) {
			notify.link_hdl = ctx->link_hdl;
			notify.dev_hdl = ctx->dev_hdl;
			notify.frame_id = ctx_isp->frame_id;
			notify.trigger = CAM_TRIGGER_POINT_SOF;
			notify.req_id = ctx_isp->req_info.last_bufdone_req_id;
			notify.sof_timestamp_val = ctx_isp->sof_timestamp_val;
			notify.trigger_id = ctx_isp->trigger_id;

			ctx->ctx_crm_intf->notify_trigger(&notify);
			CAM_DBG(CAM_ISP, "Notify CRM  SOF frame %lld ctx %u",
				ctx_isp->frame_id, ctx->ctx_id);
		}

		list_for_each_entry(req, &ctx->active_req_list, list) {
			req_isp = (struct cam_isp_ctx_req *) req->req_priv;
			if ((!req_isp->bubble_detected) &&
				(req->request_id > ctx_isp->reported_req_id)) {
				request_id = req->request_id;
				__cam_isp_ctx_update_event_record(ctx_isp,
					CAM_ISP_CTX_EVENT_EPOCH, req);
				break;
			}
		}

		if (ctx_isp->substate_activated == CAM_ISP_CTX_ACTIVATED_BUBBLE)
			request_id = 0;

		if (request_id != 0)
			ctx_isp->reported_req_id = request_id;

		__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_SUCCESS);

		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_EPOCH,
			request_id);
	} else {
		CAM_ERR_RATE_LIMIT(CAM_ISP,
			"Can not notify SOF to CRM for ctx %u",
			ctx->ctx_id);
		rc = -EFAULT;
	}
	ctx_isp->last_sof_timestamp = ctx_isp->sof_timestamp_val;
	return 0;
}

static int __cam_isp_ctx_notify_eof_in_activated_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_req_mgr_trigger_notify  notify;
	struct cam_context *ctx = ctx_isp->base;

	if (!(ctx_isp->subscribe_event & CAM_TRIGGER_POINT_EOF))
		return rc;

	/* notify reqmgr with eof signal */
	if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_trigger) {
		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.frame_id = ctx_isp->frame_id;
		notify.trigger = CAM_TRIGGER_POINT_EOF;
		notify.trigger_id = ctx_isp->trigger_id;

		ctx->ctx_crm_intf->notify_trigger(&notify);
		CAM_DBG(CAM_ISP, "Notify CRM EOF frame %lld ctx %u",
			ctx_isp->frame_id, ctx->ctx_id);

		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_EOF, 0);
	} else {
		CAM_ERR(CAM_ISP, "Can not notify EOF to CRM for ctx %u",
			ctx->ctx_id);
		rc = -EFAULT;
	}

	return rc;
}

static int __cam_isp_ctx_reg_upd_in_hw_error(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;
	return 0;
}

static int __cam_isp_ctx_sof_in_activated_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_sof_event_data      *sof_event_data = evt_data;
	struct cam_ctx_request *req = NULL;
	struct cam_context *ctx = ctx_isp->base;
	uint64_t request_id = 0;

	/* First check if there is a valid request in active list */
	list_for_each_entry(req, &ctx->active_req_list, list) {
		if (req->request_id > ctx_isp->reported_req_id) {
			request_id = req->request_id;
			break;
		}
	}

	/*
	 * If nothing in active list, current request might have not moved
	 * from wait to active list. This could happen if REG_UPDATE to sw
	 * is coming immediately after SOF
	 */
	if (request_id == 0) {
		req = list_first_entry(&ctx->wait_req_list,
			struct cam_ctx_request, list);
		if (req)
			request_id = req->request_id;
	}

	if (!evt_data) {
		CAM_ERR(CAM_ISP, "in valid sof event data");
		return -EINVAL;
	}

	ctx_isp->frame_id++;
	ctx_isp->sof_timestamp_val = sof_event_data->timestamp;
	ctx_isp->boot_timestamp = sof_event_data->boot_time;

	__cam_isp_ctx_update_state_monitor_array(ctx_isp,
		CAM_ISP_STATE_CHANGE_TRIGGER_SOF, request_id);

	CAM_DBG(CAM_ISP, "frame id: %lld time stamp:0x%llx, ctx %u",
		ctx_isp->frame_id, ctx_isp->sof_timestamp_val, ctx->ctx_id);

	return rc;
}

static int __cam_isp_ctx_reg_upd_in_sof(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;
	struct cam_ctx_request *req = NULL;
	struct cam_isp_ctx_req *req_isp;
	struct cam_context *ctx = ctx_isp->base;

	if (ctx->state != CAM_CTX_ACTIVATED && ctx_isp->frame_id > 1) {
		CAM_DBG(CAM_ISP, "invalid RUP");
		goto end;
	}

	/*
	 * This is for the first update. The initial setting will
	 * cause the reg_upd in the first frame.
	 */
	if (!list_empty(&ctx->wait_req_list)) {
		req = list_first_entry(&ctx->wait_req_list,
			struct cam_ctx_request, list);
		list_del_init(&req->list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		if (req_isp->num_fence_map_out == req_isp->num_acked)
			list_add_tail(&req->list, &ctx->free_req_list);
		else
			CAM_ERR(CAM_ISP,
				"receive rup in unexpected state");
	}
	if (req != NULL) {
		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_REG_UPDATE,
			req->request_id);
	}
end:
	return rc;
}

static int __cam_isp_ctx_epoch_in_applied(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	uint64_t request_id = 0;
	uint32_t i;
	uint32_t sof_event_status = CAM_REQ_MGR_SOF_EVENT_SUCCESS;
	struct cam_req_mgr_trigger_notify   notify;
	struct cam_ctx_request             *req;
	struct cam_isp_ctx_req             *req_isp;
	struct cam_context                 *ctx = ctx_isp->base;
	struct cam_isp_hw_epoch_event_data *epoch_done_event_data =
		(struct cam_isp_hw_epoch_event_data *)evt_data;

	if (!evt_data) {
		CAM_ERR(CAM_ISP, "invalid event data");
		return -EINVAL;
	}

	ctx_isp->frame_id_meta = epoch_done_event_data->frame_id_meta;
	if (list_empty(&ctx->wait_req_list)) {
		/*
		 * If no wait req in epoch, this is an error case.
		 * The recovery is to go back to sof state
		 */
		CAM_ERR(CAM_ISP, "Ctx:%d No wait request", ctx->ctx_id);
		ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;

		/* Send SOF event as empty frame*/
		__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_SUCCESS);
		__cam_isp_ctx_update_event_record(ctx_isp,
			CAM_ISP_CTX_EVENT_EPOCH, NULL);
		goto end;
	}

	req = list_first_entry(&ctx->wait_req_list, struct cam_ctx_request,
		list);
	req_isp = (struct cam_isp_ctx_req *)req->req_priv;
	req_isp->bubble_detected = true;
	req_isp->reapply = true;
	req_isp->cdm_reset_before_apply = false;

	/* reset early_fence_map_index array */
	for (i = 0; i < req_isp->num_fence_map_out; i++)
		req_isp->early_fence_map_index[i] = 0;

	CAM_INFO_RATE_LIMIT(CAM_ISP, "ctx:%d Report Bubble flag %d req id:%lld",
		ctx->ctx_id, req_isp->bubble_report, req->request_id);
	if (req_isp->bubble_report && ctx->ctx_crm_intf &&
		ctx->ctx_crm_intf->notify_err) {
		struct cam_req_mgr_error_notify notify;

		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.req_id = req->request_id;
		notify.error = CRM_KMD_ERR_BUBBLE;
		notify.trigger = 0;
		if (ctx_isp->subscribe_event & CAM_TRIGGER_POINT_SOF)
			notify.trigger = CAM_TRIGGER_POINT_SOF;
		notify.frame_id = ctx_isp->frame_id;
		notify.sof_timestamp_val = ctx_isp->sof_timestamp_val;
		CAM_WARN_RATE_LIMIT(CAM_ISP,
			"Notify CRM about Bubble req %lld frame %lld, ctx %u",
			req->request_id, ctx_isp->frame_id, ctx->ctx_id);
		trace_cam_log_event("Bubble", "Rcvd epoch in applied state",
			req->request_id, ctx->ctx_id);
		ctx->ctx_crm_intf->notify_err(&notify);
		atomic_set(&ctx_isp->process_bubble, 1);
	} else {
		req_isp->bubble_report = 0;
		CAM_DBG(CAM_ISP, "Skip bubble recovery for req %lld ctx %u",
			req->request_id, ctx->ctx_id);
		if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_trigger &&
			ctx_isp->active_req_cnt <= 1) {
			if (ctx_isp->subscribe_event & CAM_TRIGGER_POINT_SOF) {
				notify.link_hdl = ctx->link_hdl;
				notify.dev_hdl = ctx->dev_hdl;
				notify.frame_id = ctx_isp->frame_id;
				notify.trigger = CAM_TRIGGER_POINT_SOF;
				notify.req_id =
					ctx_isp->req_info.last_bufdone_req_id;
				notify.sof_timestamp_val =
					ctx_isp->sof_timestamp_val;
				notify.trigger_id = ctx_isp->trigger_id;

				ctx->ctx_crm_intf->notify_trigger(&notify);
				CAM_DBG(CAM_ISP,
					"Notify CRM  SOF frame %lld ctx %u",
					ctx_isp->frame_id, ctx->ctx_id);
			}
		}
		atomic_set(&ctx_isp->process_bubble, 1);
	}

	/*
	 * Always move the request to active list. Let buf done
	 * function handles the rest.
	 */
	list_del_init(&req->list);
	list_add_tail(&req->list, &ctx->active_req_list);
	ctx_isp->active_req_cnt++;
	CAM_DBG(CAM_REQ, "move request %lld to active list(cnt = %d), ctx %u",
		req->request_id, ctx_isp->active_req_cnt, ctx->ctx_id);

	/*
	 * Update the record before req pointer to
	 * other invalid req.
	 */
	__cam_isp_ctx_update_event_record(ctx_isp,
		CAM_ISP_CTX_EVENT_EPOCH, req);

	/*
	 * Get the req again from active_req_list in case
	 * the active req cnt is 2.
	 */
	list_for_each_entry(req, &ctx->active_req_list, list) {
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		if ((!req_isp->bubble_report) &&
			(req->request_id > ctx_isp->reported_req_id)) {
			request_id = req->request_id;
			ctx_isp->reported_req_id = request_id;
			CAM_DBG(CAM_ISP,
				"ctx %d reported_req_id update to %lld",
				ctx->ctx_id, ctx_isp->reported_req_id);
			break;
		}
	}

	if ((request_id != 0) && req_isp->bubble_detected)
		sof_event_status = CAM_REQ_MGR_SOF_EVENT_ERROR;

	__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
		sof_event_status);

	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_BUBBLE;
	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	cam_req_mgr_debug_delay_detect();
	trace_cam_delay_detect("ISP",
		"bubble epoch_in_applied", req->request_id,
		ctx->ctx_id, ctx->link_hdl, ctx->session_hdl,
		CAM_DEFAULT_VALUE);
end:
	if (request_id == 0) {
		req = list_last_entry(&ctx->active_req_list,
			struct cam_ctx_request, list);
		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_EPOCH, req->request_id);
	} else {
		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_EPOCH, request_id);
	}
	return 0;
}


static int __cam_isp_ctx_buf_done_in_applied(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_done_event_data *done =
		(struct cam_isp_hw_done_event_data *) evt_data;

	rc = __cam_isp_ctx_handle_buf_done_in_activated_state(ctx_isp, done, 0);
	return rc;
}


static int __cam_isp_ctx_sof_in_epoch(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;
	struct cam_context                    *ctx = ctx_isp->base;
	struct cam_isp_hw_sof_event_data      *sof_event_data = evt_data;
	struct cam_ctx_request *req;

	if (!evt_data) {
		CAM_ERR(CAM_ISP, "in valid sof event data");
		return -EINVAL;
	}

	if (atomic_read(&ctx_isp->apply_in_progress))
		CAM_INFO(CAM_ISP, "Apply is in progress at the time of SOF");

	ctx_isp->frame_id++;
	ctx_isp->sof_timestamp_val = sof_event_data->timestamp;
	ctx_isp->boot_timestamp = sof_event_data->boot_time;

	if (list_empty(&ctx->active_req_list))
		ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;
	else
		CAM_DBG(CAM_ISP, "Still need to wait for the buf done");

	req = list_last_entry(&ctx->active_req_list,
		struct cam_ctx_request, list);
	if (req)
		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_SOF,
			req->request_id);

	if (ctx_isp->frame_id == 1)
		CAM_INFO(CAM_ISP,
			"First SOF in EPCR ctx:%d frame_id:%lld next substate %s",
			ctx->ctx_id, ctx_isp->frame_id,
			__cam_isp_ctx_substate_val_to_type(
			ctx_isp->substate_activated));

	CAM_DBG(CAM_ISP, "SOF in epoch ctx:%d frame_id:%lld next substate:%s",
		ctx->ctx_id, ctx_isp->frame_id,
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	return rc;
}

static int __cam_isp_ctx_buf_done_in_epoch(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_done_event_data *done =
		(struct cam_isp_hw_done_event_data *) evt_data;

	rc = __cam_isp_ctx_handle_buf_done_in_activated_state(ctx_isp, done, 0);
	return rc;
}

static int __cam_isp_ctx_buf_done_in_bubble(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_done_event_data *done =
		(struct cam_isp_hw_done_event_data *) evt_data;

	rc = __cam_isp_ctx_handle_buf_done_in_activated_state(ctx_isp, done, 1);
	return rc;
}

static int __cam_isp_ctx_epoch_in_bubble_applied(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	uint64_t  request_id = 0;
	struct cam_req_mgr_trigger_notify   notify;
	struct cam_ctx_request             *req;
	struct cam_isp_ctx_req             *req_isp;
	struct cam_context                 *ctx = ctx_isp->base;
	struct cam_isp_hw_epoch_event_data *epoch_done_event_data =
		(struct cam_isp_hw_epoch_event_data *)evt_data;

	if (!evt_data) {
		CAM_ERR(CAM_ISP, "invalid event data");
		return -EINVAL;
	}

	ctx_isp->frame_id_meta = epoch_done_event_data->frame_id_meta;

	/*
	 * This means we missed the reg upd ack. So we need to
	 * transition to BUBBLE state again.
	 */

	if (list_empty(&ctx->wait_req_list)) {
		/*
		 * If no pending req in epoch, this is an error case.
		 * Just go back to the bubble state.
		 */
		CAM_ERR(CAM_ISP, "ctx:%d No pending request.", ctx->ctx_id);
		__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_SUCCESS);
		__cam_isp_ctx_update_event_record(ctx_isp,
			CAM_ISP_CTX_EVENT_EPOCH, NULL);

		ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_BUBBLE;
		goto end;
	}

	req = list_first_entry(&ctx->wait_req_list, struct cam_ctx_request,
		list);
	req_isp = (struct cam_isp_ctx_req *)req->req_priv;
	req_isp->bubble_detected = true;
	CAM_INFO_RATE_LIMIT(CAM_ISP, "Ctx:%d Report Bubble flag %d req id:%lld",
		ctx->ctx_id, req_isp->bubble_report, req->request_id);
	req_isp->reapply = true;
	req_isp->cdm_reset_before_apply = false;

	if (req_isp->bubble_report && ctx->ctx_crm_intf &&
		ctx->ctx_crm_intf->notify_err) {
		struct cam_req_mgr_error_notify notify;

		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.req_id = req->request_id;
		notify.error = CRM_KMD_ERR_BUBBLE;
		notify.trigger = 0;
		if (ctx_isp->subscribe_event & CAM_TRIGGER_POINT_SOF)
			notify.trigger = CAM_TRIGGER_POINT_SOF;
		notify.frame_id = ctx_isp->frame_id;
		notify.sof_timestamp_val = ctx_isp->sof_timestamp_val;
		CAM_WARN_RATE_LIMIT(CAM_REQ,
			"Notify CRM about Bubble req_id %llu frame %lld, ctx %u",
			req->request_id, ctx_isp->frame_id, ctx->ctx_id);
		ctx->ctx_crm_intf->notify_err(&notify);
		atomic_set(&ctx_isp->process_bubble, 1);
	} else {
		req_isp->bubble_report = 0;
		CAM_DBG(CAM_ISP, "Skip bubble recovery for req %lld ctx %u",
			req->request_id, ctx->ctx_id);
		if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_trigger &&
			ctx_isp->active_req_cnt <= 1) {
			if (ctx_isp->subscribe_event & CAM_TRIGGER_POINT_SOF) {
				notify.link_hdl = ctx->link_hdl;
				notify.dev_hdl = ctx->dev_hdl;
				notify.frame_id = ctx_isp->frame_id;
				notify.trigger = CAM_TRIGGER_POINT_SOF;
				notify.req_id =
					ctx_isp->req_info.last_bufdone_req_id;
				notify.sof_timestamp_val =
					ctx_isp->sof_timestamp_val;
				notify.trigger_id = ctx_isp->trigger_id;

				ctx->ctx_crm_intf->notify_trigger(&notify);
				CAM_DBG(CAM_ISP,
					"Notify CRM  SOF frame %lld ctx %u",
					ctx_isp->frame_id, ctx->ctx_id);
			}
		}
		atomic_set(&ctx_isp->process_bubble, 1);
	}

	/*
	 * Always move the request to active list. Let buf done
	 * function handles the rest.
	 */
	list_del_init(&req->list);
	list_add_tail(&req->list, &ctx->active_req_list);
	ctx_isp->active_req_cnt++;
	CAM_DBG(CAM_ISP, "move request %lld to active list(cnt = %d) ctx %u",
		req->request_id, ctx_isp->active_req_cnt);

	if (!req_isp->bubble_report) {
		if (req->request_id > ctx_isp->reported_req_id) {
			request_id = req->request_id;
			ctx_isp->reported_req_id = request_id;
			__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_ERROR);

			__cam_isp_ctx_update_event_record(ctx_isp,
				CAM_ISP_CTX_EVENT_EPOCH, req);
		} else {
			__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
				CAM_REQ_MGR_SOF_EVENT_SUCCESS);
			__cam_isp_ctx_update_event_record(ctx_isp,
				CAM_ISP_CTX_EVENT_EPOCH, NULL);
		}
	} else {
		__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_SUCCESS);
		__cam_isp_ctx_update_event_record(ctx_isp,
			CAM_ISP_CTX_EVENT_EPOCH, NULL);
	}
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_BUBBLE;
	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	cam_req_mgr_debug_delay_detect();
	trace_cam_delay_detect("ISP",
		"bubble epoch_in_bubble_applied",
		req->request_id, ctx->ctx_id,
		ctx->link_hdl, ctx->session_hdl,
		CAM_DEFAULT_VALUE);
end:
	req = list_last_entry(&ctx->active_req_list, struct cam_ctx_request,
		list);
	if (req)
		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_EPOCH, req->request_id);
	return 0;
}

static int __cam_isp_ctx_buf_done_in_bubble_applied(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_done_event_data *done =
		(struct cam_isp_hw_done_event_data *) evt_data;

	rc = __cam_isp_ctx_handle_buf_done_in_activated_state(ctx_isp, done, 1);

	return rc;
}

static uint32_t get_evt_param(uint32_t error_type)
{
	switch (error_type) {
	case CAM_ISP_HW_ERROR_OVERFLOW:
		return CAM_SYNC_ISP_EVENT_OVERFLOW;
	case CAM_ISP_HW_ERROR_P2I_ERROR:
		return CAM_SYNC_ISP_EVENT_P2I_ERROR;
	case CAM_ISP_HW_ERROR_VIOLATION:
		return CAM_SYNC_ISP_EVENT_VIOLATION;
	case CAM_ISP_HW_ERROR_BUSIF_OVERFLOW:
		return CAM_SYNC_ISP_EVENT_BUSIF_OVERFLOW;
	default:
		return CAM_SYNC_ISP_EVENT_UNKNOWN;
	}
}

static int __cam_isp_ctx_handle_error(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int                              rc = 0;
	uint32_t                         i = 0;
	bool                             found = 0;
	struct cam_ctx_request          *req = NULL;
	struct cam_ctx_request          *req_to_report = NULL;
	struct cam_ctx_request          *req_to_dump = NULL;
	struct cam_ctx_request          *req_temp;
	struct cam_isp_ctx_req          *req_isp = NULL;
	struct cam_isp_ctx_req          *req_isp_to_report = NULL;
	struct cam_req_mgr_error_notify  notify = {};
	uint64_t                         error_request_id;
	struct cam_hw_fence_map_entry   *fence_map_out = NULL;
	struct cam_req_mgr_message       req_msg;
	uint32_t                         evt_param;

	struct cam_context *ctx = ctx_isp->base;
	struct cam_isp_hw_error_event_data  *error_event_data =
			(struct cam_isp_hw_error_event_data *)evt_data;

	uint32_t error_type = error_event_data->error_type;

	CAM_DBG(CAM_ISP, "Enter error_type = %d", error_type);

	if ((error_type == CAM_ISP_HW_ERROR_OVERFLOW) ||
		(error_type == CAM_ISP_HW_ERROR_BUSIF_OVERFLOW) ||
		(error_type == CAM_ISP_HW_ERROR_VIOLATION)) {
		struct cam_hw_cmd_args hw_cmd_args;

		memset(&hw_cmd_args, 0, sizeof(hw_cmd_args));
		hw_cmd_args.ctxt_to_hw_map = ctx->ctxt_to_hw_map;
		hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_REG_DUMP_ON_ERROR;
		rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
			&hw_cmd_args);
		if (rc) {
			CAM_ERR(CAM_ISP, "Reg dump on error failed rc: %d", rc);
			rc = 0;
		}
	}

	evt_param = get_evt_param(error_type);

	/*
	 * The error is likely caused by first request on the active list.
	 * If active list is empty check wait list (maybe error hit as soon
	 * as RUP and we handle error before RUP.
	 */
	if (list_empty(&ctx->active_req_list)) {
		CAM_DBG(CAM_ISP,
			"handling error with no active request");
		if (list_empty(&ctx->wait_req_list)) {
			CAM_ERR_RATE_LIMIT(CAM_ISP,
				"Error with no active/wait request");
			goto end;
		} else {
			req_to_dump = list_first_entry(&ctx->wait_req_list,
				struct cam_ctx_request, list);
		}
	} else {
		req_to_dump = list_first_entry(&ctx->active_req_list,
			struct cam_ctx_request, list);
	}

	req_isp = (struct cam_isp_ctx_req *) req_to_dump->req_priv;

	if (error_event_data->enable_req_dump)
		rc = cam_isp_ctx_dump_req(req_isp, 0, 0, NULL, false);

	__cam_isp_ctx_update_state_monitor_array(ctx_isp,
		CAM_ISP_STATE_CHANGE_TRIGGER_ERROR, req_to_dump->request_id);

	list_for_each_entry_safe(req, req_temp,
		&ctx->active_req_list, list) {
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		if (!req_isp->bubble_report) {
			for (i = 0; i < req_isp->num_fence_map_out; i++) {
				fence_map_out =
					&req_isp->fence_map_out[i];
				CAM_ERR(CAM_ISP,
					"req %llu, Sync fd 0x%x ctx %u",
					req->request_id,
					req_isp->fence_map_out[i].sync_id,
					ctx->ctx_id);
				if (req_isp->fence_map_out[i].sync_id != -1) {
					rc = cam_sync_signal(
						fence_map_out->sync_id,
						CAM_SYNC_STATE_SIGNALED_ERROR,
						evt_param);
					fence_map_out->sync_id = -1;
				}
			}
			list_del_init(&req->list);
			list_add_tail(&req->list, &ctx->free_req_list);
			ctx_isp->active_req_cnt--;
		} else {
			found = 1;
			break;
		}
	}

	if (found)
		goto move_to_pending;

	list_for_each_entry_safe(req, req_temp,
		&ctx->wait_req_list, list) {
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		if (!req_isp->bubble_report) {
			for (i = 0; i < req_isp->num_fence_map_out; i++) {
				fence_map_out =
					&req_isp->fence_map_out[i];
				CAM_ERR(CAM_ISP,
					"req %llu, Sync fd 0x%x ctx %u",
					req->request_id,
					req_isp->fence_map_out[i].sync_id,
					ctx->ctx_id);
				if (req_isp->fence_map_out[i].sync_id != -1) {
					rc = cam_sync_signal(
						fence_map_out->sync_id,
						CAM_SYNC_STATE_SIGNALED_ERROR,
						evt_param);
					fence_map_out->sync_id = -1;
				}
			}
			list_del_init(&req->list);
			list_add_tail(&req->list, &ctx->free_req_list);
			ctx_isp->active_req_cnt--;
		} else {
			found = 1;
			break;
		}
	}

move_to_pending:
	/*
	 * If bubble recovery is enabled on any request we need to move that
	 * request and all the subsequent requests to the pending list.
	 * Note:
	 * We need to traverse the active list in reverse order and add
	 * to head of pending list.
	 * e.g. pending current state: 10, 11 | active current state: 8, 9
	 * intermittent for loop iteration- pending: 9, 10, 11 | active: 8
	 * final state - pending: 8, 9, 10, 11 | active: NULL
	 */
	if (found) {
		list_for_each_entry_safe_reverse(req, req_temp,
			&ctx->active_req_list, list) {
			req_isp = (struct cam_isp_ctx_req *) req->req_priv;
			list_del_init(&req->list);
			list_add(&req->list, &ctx->pending_req_list);
			ctx_isp->active_req_cnt--;
		}
		list_for_each_entry_safe_reverse(req, req_temp,
			&ctx->wait_req_list, list) {
			req_isp = (struct cam_isp_ctx_req *) req->req_priv;
			list_del_init(&req->list);
			list_add(&req->list, &ctx->pending_req_list);
			ctx_isp->active_req_cnt--;
		}
	}

end:
	do {
		if (list_empty(&ctx->pending_req_list)) {
			error_request_id = ctx_isp->last_applied_req_id;
			req_isp = NULL;
			break;
		}
		req = list_first_entry(&ctx->pending_req_list,
			struct cam_ctx_request, list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		error_request_id = ctx_isp->last_applied_req_id;

		if (req_isp->bubble_report) {
			req_to_report = req;
			req_isp_to_report = req_to_report->req_priv;
			break;
		}

		for (i = 0; i < req_isp->num_fence_map_out; i++) {
			if (req_isp->fence_map_out[i].sync_id != -1)
				rc = cam_sync_signal(
					req_isp->fence_map_out[i].sync_id,
					CAM_SYNC_STATE_SIGNALED_ERROR,
					evt_param);
			req_isp->fence_map_out[i].sync_id = -1;
		}
		list_del_init(&req->list);
		list_add_tail(&req->list, &ctx->free_req_list);

	} while (req->request_id < ctx_isp->last_applied_req_id);

	if (ctx_isp->offline_context)
		goto exit;

	if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_err) {
		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.req_id = error_request_id;
		notify.error = CRM_KMD_ERR_FATAL;

		if (req_isp_to_report && req_isp_to_report->bubble_report)
			if (error_event_data->recovery_enabled)
				notify.error = CRM_KMD_ERR_BUBBLE;

		CAM_WARN(CAM_ISP,
			"Notify CRM: req %lld, frame %lld ctx %u, error %d",
			error_request_id, ctx_isp->frame_id, ctx->ctx_id,
			notify.error);

		ctx->ctx_crm_intf->notify_err(&notify);

		/*
		 * Need to send error occurred in KMD
		 * This will help UMD to take necessary action
		 * and to dump relevant info
		 */

		if (notify.error == CRM_KMD_ERR_FATAL) {
			req_msg.session_hdl = ctx_isp->base->session_hdl;
			req_msg.u.err_msg.device_hdl = ctx_isp->base->dev_hdl;

			if (error_type == CAM_ISP_HW_ERROR_CSID_FATAL)
				req_msg.u.err_msg.error_type =
					CAM_REQ_MGR_ERROR_TYPE_FULL_RECOVERY;
			else
				req_msg.u.err_msg.error_type =
					CAM_REQ_MGR_ERROR_TYPE_RECOVERY;

			req_msg.u.err_msg.link_hdl = ctx_isp->base->link_hdl;
			req_msg.u.err_msg.request_id = error_request_id;
			req_msg.u.err_msg.resource_size = 0x0;

			if (cam_req_mgr_notify_message(&req_msg,
					V4L_EVENT_CAM_REQ_MGR_ERROR,
					V4L_EVENT_CAM_REQ_MGR_EVENT))
				CAM_ERR(CAM_ISP,
					"Error in notifying the error time for req id:%lld ctx %u",
						ctx_isp->last_applied_req_id,
						ctx->ctx_id);
		}
		ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_HW_ERROR;
	} else {
		CAM_ERR_RATE_LIMIT(CAM_ISP,
			"Can not notify ERRROR to CRM for ctx %u",
			ctx->ctx_id);
		rc = -EFAULT;
	}

	CAM_DBG(CAM_ISP, "Exit");

exit:
	return rc;
}

static int __cam_isp_ctx_fs2_sof_in_sof_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_sof_event_data      *sof_event_data = evt_data;
	struct cam_ctx_request *req;
	struct cam_context *ctx = ctx_isp->base;
	struct cam_req_mgr_trigger_notify  notify;
	uint64_t  request_id  = 0;

	if (!evt_data) {
		CAM_ERR(CAM_ISP, "in valid sof event data");
		return -EINVAL;
	}

	ctx_isp->frame_id++;
	ctx_isp->sof_timestamp_val = sof_event_data->timestamp;
	ctx_isp->boot_timestamp = sof_event_data->boot_time;

	CAM_DBG(CAM_ISP, "frame id: %lld time stamp:0x%llx",
		ctx_isp->frame_id, ctx_isp->sof_timestamp_val);

	if (!(list_empty(&ctx->wait_req_list)))
		goto end;

	if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_trigger &&
		ctx_isp->active_req_cnt <= 2) {
		if (ctx_isp->subscribe_event & CAM_TRIGGER_POINT_SOF) {
			notify.link_hdl = ctx->link_hdl;
			notify.dev_hdl = ctx->dev_hdl;
			notify.frame_id = ctx_isp->frame_id;
			notify.trigger = CAM_TRIGGER_POINT_SOF;
			notify.req_id = ctx_isp->req_info.last_bufdone_req_id;
			notify.sof_timestamp_val = ctx_isp->sof_timestamp_val;

			ctx->ctx_crm_intf->notify_trigger(&notify);
			CAM_DBG(CAM_ISP, "Notify CRM  SOF frame %lld",
				ctx_isp->frame_id);
		}

		list_for_each_entry(req, &ctx->active_req_list, list) {
			if (req->request_id > ctx_isp->reported_req_id) {
				request_id = req->request_id;
				ctx_isp->reported_req_id = request_id;
				break;
			}
		}

		__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_SUCCESS);

		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_SOF, request_id);
	} else {
		CAM_ERR_RATE_LIMIT(CAM_ISP, "Can not notify SOF to CRM");
		rc = -EFAULT;
	}

end:
	return rc;
}

static int __cam_isp_ctx_fs2_buf_done(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;
	struct cam_isp_hw_done_event_data *done =
		(struct cam_isp_hw_done_event_data *) evt_data;
	struct cam_context *ctx = ctx_isp->base;
	int prev_active_req_cnt = 0;
	int curr_req_id = 0;
	struct cam_ctx_request  *req;

	prev_active_req_cnt = ctx_isp->active_req_cnt;
	req = list_first_entry(&ctx->active_req_list,
		struct cam_ctx_request, list);
	if (req)
		curr_req_id = req->request_id;

	rc = __cam_isp_ctx_handle_buf_done_in_activated_state(ctx_isp, done, 0);

	if (prev_active_req_cnt == ctx_isp->active_req_cnt + 1) {
		if (list_empty(&ctx->wait_req_list) &&
			list_empty(&ctx->active_req_list)) {
			CAM_DBG(CAM_ISP, "No request, move to SOF");
			ctx_isp->substate_activated =
				CAM_ISP_CTX_ACTIVATED_SOF;
			if (ctx_isp->reported_req_id < curr_req_id) {
				ctx_isp->reported_req_id = curr_req_id;
				__cam_isp_ctx_send_sof_timestamp(ctx_isp,
					curr_req_id,
					CAM_REQ_MGR_SOF_EVENT_SUCCESS);
			}
		}
	}

	return rc;
}

static int __cam_isp_ctx_fs2_buf_done_in_epoch(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;

	rc =  __cam_isp_ctx_fs2_buf_done(ctx_isp, evt_data);
	return rc;
}

static int __cam_isp_ctx_fs2_buf_done_in_applied(
	struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;

	rc =  __cam_isp_ctx_fs2_buf_done(ctx_isp, evt_data);
	return rc;
}

static int __cam_isp_ctx_fs2_reg_upd_in_sof(struct cam_isp_context *ctx_isp,
	void *evt_data)
{
	int rc = 0;
	struct cam_ctx_request *req = NULL;
	struct cam_isp_ctx_req *req_isp;
	struct cam_context *ctx = ctx_isp->base;

	if (ctx->state != CAM_CTX_ACTIVATED && ctx_isp->frame_id > 1) {
		CAM_DBG(CAM_ISP, "invalid RUP");
		goto end;
	}

	/*
	 * This is for the first update. The initial setting will
	 * cause the reg_upd in the first frame.
	 */
	if (!list_empty(&ctx->wait_req_list)) {
		req = list_first_entry(&ctx->wait_req_list,
			struct cam_ctx_request, list);
		list_del_init(&req->list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		if (req_isp->num_fence_map_out == req_isp->num_acked)
			list_add_tail(&req->list, &ctx->free_req_list);
		else
			CAM_ERR(CAM_ISP,
				"receive rup in unexpected state");
	}
	if (req != NULL) {
		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_REG_UPDATE,
			req->request_id);
	}
end:
	return rc;
}

static int __cam_isp_ctx_fs2_reg_upd_in_applied_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_ctx_request  *req = NULL;
	struct cam_context      *ctx = ctx_isp->base;
	struct cam_isp_ctx_req  *req_isp;
	struct cam_req_mgr_trigger_notify  notify;
	uint64_t  request_id  = 0;

	if (list_empty(&ctx->wait_req_list)) {
		CAM_ERR(CAM_ISP, "Reg upd ack with no waiting request");
		goto end;
	}
	req = list_first_entry(&ctx->wait_req_list,
			struct cam_ctx_request, list);
	list_del_init(&req->list);

	req_isp = (struct cam_isp_ctx_req *) req->req_priv;
	if (req_isp->num_fence_map_out != 0) {
		list_add_tail(&req->list, &ctx->active_req_list);
		ctx_isp->active_req_cnt++;
		CAM_DBG(CAM_REQ, "move request %lld to active list(cnt = %d)",
			 req->request_id, ctx_isp->active_req_cnt);
	} else {
		/* no io config, so the request is completed. */
		list_add_tail(&req->list, &ctx->free_req_list);
	}

	/*
	 * This function only called directly from applied and bubble applied
	 * state so change substate here.
	 */
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_EPOCH;
	if (req_isp->num_fence_map_out != 1)
		goto end;

	if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_trigger &&
		ctx_isp->active_req_cnt <= 2) {
		list_for_each_entry(req, &ctx->active_req_list, list) {
			if (req->request_id > ctx_isp->reported_req_id) {
				request_id = req->request_id;
				ctx_isp->reported_req_id = request_id;
				break;
			}
		}

		__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_SUCCESS);

		if (ctx_isp->subscribe_event & CAM_TRIGGER_POINT_SOF) {
			notify.link_hdl = ctx->link_hdl;
			notify.dev_hdl = ctx->dev_hdl;
			notify.frame_id = ctx_isp->frame_id;
			notify.trigger = CAM_TRIGGER_POINT_SOF;
			notify.req_id = ctx_isp->req_info.last_bufdone_req_id;
			notify.sof_timestamp_val = ctx_isp->sof_timestamp_val;

			ctx->ctx_crm_intf->notify_trigger(&notify);
			CAM_DBG(CAM_ISP, "Notify CRM  SOF frame %lld",
				ctx_isp->frame_id);
		}
	} else {
		CAM_ERR_RATE_LIMIT(CAM_ISP, "Can not notify SOF to CRM");
		rc = -EFAULT;
	}

	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));
end:
	if (req != NULL && !rc) {
		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_REG_UPDATE,
			req->request_id);
	}
	return rc;
}

static struct cam_isp_ctx_irq_ops
	cam_isp_ctx_activated_state_machine_irq[CAM_ISP_CTX_ACTIVATED_MAX] = {
	/* SOF */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_reg_upd_in_sof,
			__cam_isp_ctx_notify_sof_in_activated_state,
			__cam_isp_ctx_notify_eof_in_activated_state,
			NULL,
		},
	},
	/* APPLIED */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_reg_upd_in_applied_state,
			__cam_isp_ctx_epoch_in_applied,
			__cam_isp_ctx_notify_eof_in_activated_state,
			__cam_isp_ctx_buf_done_in_applied,
		},
	},
	/* EPOCH */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_epoch,
			__cam_isp_ctx_reg_upd_in_epoch_bubble_state,
			__cam_isp_ctx_notify_sof_in_activated_state,
			__cam_isp_ctx_notify_eof_in_activated_state,
			__cam_isp_ctx_buf_done_in_epoch,
		},
	},
	/* BUBBLE */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_reg_upd_in_epoch_bubble_state,
			__cam_isp_ctx_notify_sof_in_activated_state,
			__cam_isp_ctx_notify_eof_in_activated_state,
			__cam_isp_ctx_buf_done_in_bubble,
		},
	},
	/* Bubble Applied */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_reg_upd_in_applied_state,
			__cam_isp_ctx_epoch_in_bubble_applied,
			NULL,
			__cam_isp_ctx_buf_done_in_bubble_applied,
		},
	},
	/* HW ERROR */
	{
		.irq_ops = {
			NULL,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_reg_upd_in_hw_error,
			NULL,
			NULL,
			NULL,
		},
	},
	/* HALT */
	{
	},
};

static struct cam_isp_ctx_irq_ops
	cam_isp_ctx_fs2_state_machine_irq[CAM_ISP_CTX_ACTIVATED_MAX] = {
	/* SOF */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_fs2_sof_in_sof_state,
			__cam_isp_ctx_fs2_reg_upd_in_sof,
			__cam_isp_ctx_fs2_sof_in_sof_state,
			__cam_isp_ctx_notify_eof_in_activated_state,
			NULL,
		},
	},
	/* APPLIED */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_fs2_reg_upd_in_applied_state,
			__cam_isp_ctx_epoch_in_applied,
			__cam_isp_ctx_notify_eof_in_activated_state,
			__cam_isp_ctx_fs2_buf_done_in_applied,
		},
	},
	/* EPOCH */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_epoch,
			__cam_isp_ctx_reg_upd_in_epoch_bubble_state,
			__cam_isp_ctx_notify_sof_in_activated_state,
			__cam_isp_ctx_notify_eof_in_activated_state,
			__cam_isp_ctx_fs2_buf_done_in_epoch,
		},
	},
	/* BUBBLE */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_reg_upd_in_epoch_bubble_state,
			__cam_isp_ctx_notify_sof_in_activated_state,
			__cam_isp_ctx_notify_eof_in_activated_state,
			__cam_isp_ctx_buf_done_in_bubble,
		},
	},
	/* Bubble Applied */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_reg_upd_in_applied_state,
			__cam_isp_ctx_epoch_in_bubble_applied,
			NULL,
			__cam_isp_ctx_buf_done_in_bubble_applied,
		},
	},
	/* HW ERROR */
	{
		.irq_ops = {
			NULL,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_reg_upd_in_hw_error,
			NULL,
			NULL,
			NULL,
		},
	},
	/* HALT */
	{
	},
};

static struct cam_isp_ctx_irq_ops
	cam_isp_ctx_offline_state_machine_irq[CAM_ISP_CTX_ACTIVATED_MAX] = {
	/* SOF */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			NULL,
			NULL,
			NULL,
			NULL,
			NULL,
		},
	},
	/* APPLIED */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_reg_upd_in_applied_state,
			NULL,
			NULL,
			__cam_isp_ctx_buf_done_in_applied,
		},
	},
	/* EPOCH */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			NULL,
			NULL,
			__cam_isp_ctx_offline_epoch_in_activated_state,
			NULL,
			__cam_isp_ctx_buf_done_in_epoch,
		},
	},
	/* BUBBLE */
	{
	},
	/* Bubble Applied */
	{
	},
	/* HW ERROR */
	{
		.irq_ops = {
			NULL,
			__cam_isp_ctx_sof_in_activated_state,
			__cam_isp_ctx_reg_upd_in_hw_error,
			NULL,
			NULL,
			NULL,
		},
	},
	/* HALT */
	{
	},
};

static int __cam_isp_ctx_apply_req_in_activated_state(
	struct cam_context *ctx, struct cam_req_mgr_apply_request *apply,
	enum cam_isp_ctx_activated_substate next_state)
{
	int rc = 0, i;
	struct cam_ctx_request          *req;
	struct cam_ctx_request          *active_req = NULL;
	struct cam_isp_ctx_req          *req_isp;
	struct cam_isp_ctx_req          *active_req_isp;
	struct cam_isp_context          *ctx_isp = NULL;
	struct cam_hw_config_args        cfg = {0};

	ctx_isp = (struct cam_isp_context *) ctx->ctx_priv;

	if (apply->re_apply)
		if (apply->request_id <= ctx_isp->last_applied_req_id) {
			CAM_INFO_RATE_LIMIT(CAM_ISP,
				"ctx_id:%d Trying to reapply the same request %llu again",
				ctx->ctx_id,
				apply->request_id);
			return 0;
		}

	if (list_empty(&ctx->pending_req_list)) {
		CAM_ERR_RATE_LIMIT(CAM_ISP,
			"ctx_id:%d No available request for Apply id %lld",
			ctx->ctx_id,
			apply->request_id);
		rc = -EFAULT;
		goto end;
	}

	/*
	 * When the pipeline has issue, the requests can be queued up in the
	 * pipeline. In this case, we should reject the additional request.
	 * The maximum number of request allowed to be outstanding is 2.
	 *
	 */
	if (atomic_read(&ctx_isp->process_bubble)) {
		CAM_INFO_RATE_LIMIT(CAM_ISP,
			"ctx_id:%d Processing bubble cannot apply Request Id %llu",
			ctx->ctx_id,
			apply->request_id);
		rc = -EAGAIN;
		goto end;
	}

	spin_lock_bh(&ctx->lock);
	req = list_first_entry(&ctx->pending_req_list, struct cam_ctx_request,
		list);
	spin_unlock_bh(&ctx->lock);

	/*
	 * Check whether the request id is matching the tip, if not, this means
	 * we are in the middle of the error handling. Need to reject this apply
	 */
	if (req->request_id != apply->request_id) {
		CAM_ERR_RATE_LIMIT(CAM_ISP,
			"ctx_id:%d Invalid Request Id asking %llu existing %llu",
			ctx->ctx_id,
			apply->request_id, req->request_id);
		rc = -EFAULT;
		goto end;
	}

	CAM_DBG(CAM_REQ, "Apply request %lld in Substate[%s] ctx %u",
		req->request_id,
		__cam_isp_ctx_substate_val_to_type(ctx_isp->substate_activated),
		ctx->ctx_id);
	req_isp = (struct cam_isp_ctx_req *) req->req_priv;

	if (ctx_isp->active_req_cnt >=  2) {
		CAM_WARN_RATE_LIMIT(CAM_ISP,
			"Reject apply request (id %lld) due to congestion(cnt = %d) ctx %u",
			req->request_id,
			ctx_isp->active_req_cnt,
			ctx->ctx_id);

		spin_lock_bh(&ctx->lock);
		if (!list_empty(&ctx->active_req_list))
			active_req = list_first_entry(&ctx->active_req_list,
				struct cam_ctx_request, list);
		else
			CAM_ERR_RATE_LIMIT(CAM_ISP,
				"WARNING: should not happen (cnt = %d) but active_list empty",
				ctx_isp->active_req_cnt);
		spin_unlock_bh(&ctx->lock);

		if (active_req) {
			active_req_isp =
				(struct cam_isp_ctx_req *) active_req->req_priv;
			__cam_isp_ctx_handle_buf_done_fail_log(
				active_req->request_id, active_req_isp,
				ctx_isp->isp_device_type);
		}

		rc = -EFAULT;
		goto end;
	}
	req_isp->bubble_report = apply->report_if_bubble;

	/* reset early_fence_map_index */
	for (i = 0; i <  req_isp->num_fence_map_out; i++)
		req_isp->early_fence_map_index[i] = 0;

	req_isp->flag_sync_set = false;

	cfg.ctxt_to_hw_map = ctx_isp->hw_ctx;
	cfg.request_id = req->request_id;
	cfg.hw_update_entries = req_isp->cfg;
	cfg.num_hw_update_entries = req_isp->num_cfg;
	cfg.priv  = &req_isp->hw_update_data;
	cfg.init_packet = 0;
	cfg.reapply = req_isp->reapply;
	cfg.cdm_reset_before_apply = req_isp->cdm_reset_before_apply;

	atomic_set(&ctx_isp->apply_in_progress, 1);

	rc = ctx->hw_mgr_intf->hw_config(ctx->hw_mgr_intf->hw_mgr_priv, &cfg);
	if (!rc) {
		spin_lock_bh(&ctx->lock);
		ctx_isp->substate_activated = next_state;
		ctx_isp->last_applied_req_id = apply->request_id;
		list_del_init(&req->list);
		list_add_tail(&req->list, &ctx->wait_req_list);
		CAM_DBG(CAM_ISP, "new substate Substate[%s], applied req %lld",
			__cam_isp_ctx_substate_val_to_type(next_state),
			ctx_isp->last_applied_req_id);
		spin_unlock_bh(&ctx->lock);

		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_APPLIED,
			req->request_id);
		__cam_isp_ctx_update_event_record(ctx_isp,
			CAM_ISP_CTX_EVENT_APPLY, req);
	} else if (rc == -EALREADY) {
		spin_lock_bh(&ctx->lock);
		req_isp->bubble_detected = true;
		req_isp->cdm_reset_before_apply = false;
		atomic_set(&ctx_isp->process_bubble, 1);
		list_del_init(&req->list);
		list_add(&req->list, &ctx->active_req_list);
		ctx_isp->active_req_cnt++;
		spin_unlock_bh(&ctx->lock);
		CAM_DBG(CAM_REQ,
			"move request %lld to active list(cnt = %d), ctx %u",
			req->request_id, ctx_isp->active_req_cnt, ctx->ctx_id);
	} else {
		CAM_ERR_RATE_LIMIT(CAM_ISP,
			"ctx_id:%d ,Can not apply (req %lld) the configuration, rc %d",
			ctx->ctx_id, apply->request_id, rc);
	}
	atomic_set(&ctx_isp->apply_in_progress, 0);
end:
	return rc;
}

static int __cam_isp_ctx_apply_req_in_sof(
	struct cam_context *ctx, struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	CAM_DBG(CAM_ISP, "current Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));
	rc = __cam_isp_ctx_apply_req_in_activated_state(ctx, apply,
		CAM_ISP_CTX_ACTIVATED_APPLIED);
	CAM_DBG(CAM_ISP, "new Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	if (rc)
		CAM_DBG(CAM_ISP, "Apply failed in Substate[%s], rc %d",
			__cam_isp_ctx_substate_val_to_type(
			ctx_isp->substate_activated), rc);

	return rc;
}

static int __cam_isp_ctx_apply_req_in_epoch(
	struct cam_context *ctx, struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	CAM_DBG(CAM_ISP, "current Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));
	rc = __cam_isp_ctx_apply_req_in_activated_state(ctx, apply,
		CAM_ISP_CTX_ACTIVATED_APPLIED);
	CAM_DBG(CAM_ISP, "new Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	if (rc)
		CAM_DBG(CAM_ISP, "Apply failed in Substate[%s], rc %d",
			__cam_isp_ctx_substate_val_to_type(
			ctx_isp->substate_activated), rc);

	return rc;
}

static int __cam_isp_ctx_apply_req_in_bubble(
	struct cam_context *ctx, struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	CAM_DBG(CAM_ISP, "current Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));
	rc = __cam_isp_ctx_apply_req_in_activated_state(ctx, apply,
		CAM_ISP_CTX_ACTIVATED_BUBBLE_APPLIED);
	CAM_DBG(CAM_ISP, "new Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	if (rc)
		CAM_DBG(CAM_ISP, "Apply failed in Substate[%s], rc %d",
			__cam_isp_ctx_substate_val_to_type(
			ctx_isp->substate_activated), rc);

	return rc;
}

static int __cam_isp_ctx_dump_req_info(
	struct cam_context     *ctx,
	struct cam_ctx_request *req,
	uintptr_t               cpu_addr,
	size_t                  buf_len,
	size_t                 *offset)
{
	int                                 i, rc;
	uint8_t                            *dst;
	int32_t                            *addr, *start;
	uint32_t                            min_len;
	size_t                              remain_len;
	struct cam_isp_ctx_req             *req_isp;
	struct cam_isp_context             *ctx_isp;
	struct cam_isp_context_dump_header *hdr;

	if (!req || !ctx || !offset || !cpu_addr || !buf_len) {
		CAM_ERR(CAM_ISP, "Invalid parameters %pK %pK %pK %zu",
			req, ctx, offset, buf_len);
		return -EINVAL;
	}
	req_isp = (struct cam_isp_ctx_req *)req->req_priv;
	ctx_isp = (struct cam_isp_context *)ctx->ctx_priv;

	if (buf_len <= *offset) {
		CAM_WARN(CAM_ISP, "Dump buffer overshoot len %zu offset %zu",
			buf_len, *offset);
		return -ENOSPC;
	}

	remain_len = buf_len - *offset;
	min_len = sizeof(struct cam_isp_context_dump_header) +
		(CAM_ISP_CTX_DUMP_REQUEST_NUM_WORDS *
		 req_isp->num_fence_map_out *
		sizeof(int32_t));

	if (remain_len < min_len) {
		CAM_WARN(CAM_ISP, "Dump buffer exhaust remain %zu min %u",
			remain_len, min_len);
		return -ENOSPC;
	}

	dst = (uint8_t *)cpu_addr + *offset;
	hdr = (struct cam_isp_context_dump_header *)dst;
	hdr->word_size = sizeof(int32_t);
	scnprintf(hdr->tag, CAM_ISP_CONTEXT_DUMP_TAG_MAX_LEN,
		"ISP_OUT_FENCE:");
	addr = (int32_t *)(dst + sizeof(struct cam_isp_context_dump_header));
	start = addr;
	for (i = 0; i < req_isp->num_fence_map_out; i++) {
		*addr++ = req_isp->fence_map_out[i].resource_handle;
		*addr++ = req_isp->fence_map_out[i].sync_id;
	}
	hdr->size = hdr->word_size * (addr - start);
	*offset += hdr->size + sizeof(struct cam_isp_context_dump_header);
	rc = cam_isp_ctx_dump_req(req_isp, cpu_addr, buf_len,
		offset, true);
	return rc;
}

static int __cam_isp_ctx_dump_in_top_state(
	struct cam_context           *ctx,
	struct cam_req_mgr_dump_info *dump_info)
{
	int                                 rc = 0;
	bool                                dump_only_event_record = false;
	size_t                              buf_len;
	size_t                              remain_len;
	uint8_t                            *dst;
	ktime_t                             cur_time;
	uint32_t                            min_len;
	uint64_t                            diff;
	uint64_t                           *addr, *start;
	uintptr_t                           cpu_addr;
	struct timespec64                   ts;
	struct cam_isp_context             *ctx_isp;
	struct cam_ctx_request             *req = NULL;
	struct cam_isp_ctx_req             *req_isp;
	struct cam_ctx_request             *req_temp;
	struct cam_hw_dump_args             dump_args;
	struct cam_isp_context_dump_header *hdr;

	spin_lock_bh(&ctx->lock);
	list_for_each_entry_safe(req, req_temp,
		&ctx->active_req_list, list) {
		if (req->request_id == dump_info->req_id) {
			CAM_INFO(CAM_ISP, "isp dump active list req: %lld",
			    dump_info->req_id);
			goto hw_dump;
		}
	}
	list_for_each_entry_safe(req, req_temp,
		&ctx->wait_req_list, list) {
		if (req->request_id == dump_info->req_id) {
			CAM_INFO(CAM_ISP, "isp dump wait list req: %lld",
			    dump_info->req_id);
			goto hw_dump;
		}
	}
	spin_unlock_bh(&ctx->lock);
	return rc;
hw_dump:
	rc  = cam_mem_get_cpu_buf(dump_info->buf_handle,
		&cpu_addr, &buf_len);
	if (rc) {
		CAM_ERR(CAM_ISP, "Invalid handle %u rc %d",
			dump_info->buf_handle, rc);
		spin_unlock_bh(&ctx->lock);
		return rc;
	}
	if (buf_len <= dump_info->offset) {
		spin_unlock_bh(&ctx->lock);
		CAM_WARN(CAM_ISP, "Dump buffer overshoot len %zu offset %zu",
			buf_len, dump_info->offset);
		return -ENOSPC;
	}

	remain_len = buf_len - dump_info->offset;
	min_len = sizeof(struct cam_isp_context_dump_header) +
		(CAM_ISP_CTX_DUMP_NUM_WORDS * sizeof(uint64_t));

	if (remain_len < min_len) {
		spin_unlock_bh(&ctx->lock);
		CAM_WARN(CAM_ISP, "Dump buffer exhaust remain %zu min %u",
			remain_len, min_len);
		return -ENOSPC;
	}

	ctx_isp = (struct cam_isp_context *) ctx->ctx_priv;
	req_isp = (struct cam_isp_ctx_req *) req->req_priv;
	cur_time = ktime_get();
	diff = ktime_us_delta(
		req_isp->event_timestamp[CAM_ISP_CTX_EVENT_APPLY],
		cur_time);
	if (diff < CAM_ISP_CTX_RESPONSE_TIME_THRESHOLD) {
		CAM_INFO(CAM_ISP, "req %lld found no error",
			req->request_id);
		dump_only_event_record = true;
	}
	dst = (uint8_t *)cpu_addr + dump_info->offset;
	hdr = (struct cam_isp_context_dump_header *)dst;
	scnprintf(hdr->tag, CAM_ISP_CONTEXT_DUMP_TAG_MAX_LEN,
		"ISP_CTX_DUMP:");
	hdr->word_size = sizeof(uint64_t);
	addr = (uint64_t *)(dst +
		sizeof(struct cam_isp_context_dump_header));
	start = addr;
	*addr++ = req->request_id;
	ts      = ktime_to_timespec64(
		req_isp->event_timestamp[CAM_ISP_CTX_EVENT_APPLY]);
	*addr++ = ts.tv_sec;
	*addr++ = ts.tv_nsec/NSEC_PER_USEC;
	ts      = ktime_to_timespec64(cur_time);
	*addr++ = ts.tv_sec;
	*addr++ = ts.tv_nsec/NSEC_PER_USEC;
	hdr->size = hdr->word_size * (addr - start);
	dump_info->offset += hdr->size +
		sizeof(struct cam_isp_context_dump_header);

	rc = __cam_isp_ctx_dump_event_record(ctx_isp, cpu_addr,
		buf_len, &dump_info->offset);
	if (rc) {
		CAM_ERR(CAM_ISP, "Dump event fail %lld",
			req->request_id);
		spin_unlock_bh(&ctx->lock);
		return rc;
	}
	if (dump_only_event_record) {
		spin_unlock_bh(&ctx->lock);
		return rc;
	}
	rc = __cam_isp_ctx_dump_req_info(ctx, req, cpu_addr,
		buf_len, &dump_info->offset);
	if (rc) {
		CAM_ERR(CAM_ISP, "Dump Req info fail %lld",
			req->request_id);
		spin_unlock_bh(&ctx->lock);
		return rc;
	}
	spin_unlock_bh(&ctx->lock);

	if (ctx->hw_mgr_intf->hw_dump) {
		dump_args.offset = dump_info->offset;
		dump_args.request_id = dump_info->req_id;
		dump_args.buf_handle = dump_info->buf_handle;
		dump_args.ctxt_to_hw_map = ctx_isp->hw_ctx;
		rc = ctx->hw_mgr_intf->hw_dump(
			ctx->hw_mgr_intf->hw_mgr_priv,
			&dump_args);
		dump_info->offset = dump_args.offset;
	}
	return rc;
}

static int __cam_isp_ctx_flush_req_in_flushed_state(
	struct cam_context               *ctx,
	struct cam_req_mgr_flush_request *flush_req)
{
	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_ALL) {
		CAM_INFO(CAM_ISP, "Flush in flushed state req id %lld ctx_id:%d",
			flush_req->req_id, ctx->ctx_id);
		ctx->last_flush_req = flush_req->req_id;
	}

	return 0;
}

static int __cam_isp_ctx_flush_req(struct cam_context *ctx,
	struct list_head *req_list, struct cam_req_mgr_flush_request *flush_req)
{
	int i, rc, tmp = 0;
	uint32_t cancel_req_id_found = 0;
	struct cam_ctx_request           *req;
	struct cam_ctx_request           *req_temp;
	struct cam_isp_ctx_req           *req_isp;
	struct list_head                  flush_list;
	struct cam_isp_context           *ctx_isp = NULL;

	ctx_isp = (struct cam_isp_context *) ctx->ctx_priv;

	INIT_LIST_HEAD(&flush_list);
	if (list_empty(req_list)) {
		CAM_DBG(CAM_ISP, "request list is empty");
		if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ) {
			CAM_ERR(CAM_ISP, "no request to cancel");
			return -EINVAL;
		} else
			return 0;
	}

	CAM_DBG(CAM_REQ, "Flush [%u] in progress for req_id %llu",
		flush_req->type, flush_req->req_id);
	list_for_each_entry_safe(req, req_temp, req_list, list) {
		if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ) {
			if (req->request_id != flush_req->req_id) {
				continue;
			} else {
				list_del_init(&req->list);
				list_add_tail(&req->list, &flush_list);
				cancel_req_id_found = 1;
				__cam_isp_ctx_update_state_monitor_array(
					ctx_isp,
					CAM_ISP_STATE_CHANGE_TRIGGER_FLUSH,
					req->request_id);
				break;
			}
		}
		list_del_init(&req->list);
		list_add_tail(&req->list, &flush_list);
		__cam_isp_ctx_update_state_monitor_array(ctx_isp,
			CAM_ISP_STATE_CHANGE_TRIGGER_FLUSH, req->request_id);
	}

	if (list_empty(&flush_list)) {
		/*
		 * Maybe the req isn't sent to KMD since UMD already skip
		 * req in CSL layer.
		 */
		CAM_INFO(CAM_ISP,
			"flush list is empty, flush type %d for req %llu",
			flush_req->type, flush_req->req_id);
		return 0;
	}

	list_for_each_entry_safe(req, req_temp, &flush_list, list) {
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		for (i = 0; i < req_isp->num_fence_map_out; i++) {
			if (req_isp->fence_map_out[i].sync_id != -1) {
				CAM_DBG(CAM_ISP, "Flush req 0x%llx, fence %d",
					req->request_id,
					req_isp->fence_map_out[i].sync_id);
				rc = cam_sync_signal(
					req_isp->fence_map_out[i].sync_id,
					CAM_SYNC_STATE_SIGNALED_CANCEL,
					CAM_SYNC_ISP_EVENT_FLUSH);
				if (rc) {
					tmp = req_isp->fence_map_out[i].sync_id;
					CAM_ERR_RATE_LIMIT(CAM_ISP,
						"signal fence %d failed", tmp);
				}
				req_isp->fence_map_out[i].sync_id = -1;
			}
		}
		req_isp->reapply = false;
		req_isp->cdm_reset_before_apply = false;
		list_del_init(&req->list);
		list_add_tail(&req->list, &ctx->free_req_list);
	}

	return 0;
}

static int __cam_isp_ctx_flush_req_in_top_state(
	struct cam_context               *ctx,
	struct cam_req_mgr_flush_request *flush_req)
{
	int                               rc = 0;
	struct cam_isp_context           *ctx_isp;
	struct cam_isp_stop_args          stop_isp;
	struct cam_hw_stop_args           stop_args;
	struct cam_hw_reset_args          reset_args;
	struct cam_hw_cmd_args            hw_cmd_args;
	struct cam_req_mgr_timer_notify    timer;

	ctx_isp = (struct cam_isp_context *) ctx->ctx_priv;

	CAM_DBG(CAM_ISP, "Flush pending list");
	spin_lock_bh(&ctx->lock);
	rc = __cam_isp_ctx_flush_req(ctx, &ctx->pending_req_list, flush_req);
	spin_unlock_bh(&ctx->lock);

	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_ALL) {
		if (ctx->state <= CAM_CTX_READY) {
			ctx->state = CAM_CTX_ACQUIRED;
			goto end;
		}

		spin_lock_bh(&ctx->lock);
		ctx->state = CAM_CTX_FLUSHED;
		ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_HALT;
		spin_unlock_bh(&ctx->lock);

		CAM_INFO(CAM_ISP, "Last request id to flush is %lld, ctx_id:%d",
			flush_req->req_id, ctx->ctx_id);
		ctx->last_flush_req = flush_req->req_id;

		memset(&hw_cmd_args, 0, sizeof(hw_cmd_args));
		hw_cmd_args.ctxt_to_hw_map = ctx->ctxt_to_hw_map;
		hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_REG_DUMP_ON_FLUSH;
		rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
			&hw_cmd_args);
		if (rc) {
			CAM_ERR(CAM_ISP, "Reg dump on flush failed rc: %d", rc);
			rc = 0;
		}

		stop_args.ctxt_to_hw_map = ctx_isp->hw_ctx;
		stop_isp.hw_stop_cmd = CAM_ISP_HW_STOP_IMMEDIATELY;
		stop_isp.stop_only = true;
		stop_args.args = (void *)&stop_isp;
		rc = ctx->hw_mgr_intf->hw_stop(ctx->hw_mgr_intf->hw_mgr_priv,
			&stop_args);
		if (rc)
			CAM_ERR(CAM_ISP, "Failed to stop HW in Flush rc: %d",
				rc);

		CAM_INFO(CAM_ISP, "Stop HW complete. Reset HW next.");
		CAM_DBG(CAM_ISP, "Flush wait and active lists");

		if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_timer) {
			timer.link_hdl = ctx->link_hdl;
			timer.dev_hdl = ctx->dev_hdl;
			timer.state = false;
			ctx->ctx_crm_intf->notify_timer(&timer);
		}

		spin_lock_bh(&ctx->lock);
		if (!list_empty(&ctx->wait_req_list))
			rc = __cam_isp_ctx_flush_req(ctx, &ctx->wait_req_list,
				flush_req);

		if (!list_empty(&ctx->active_req_list))
			rc = __cam_isp_ctx_flush_req(ctx, &ctx->active_req_list,
				flush_req);

		ctx_isp->active_req_cnt = 0;
		spin_unlock_bh(&ctx->lock);

		reset_args.ctxt_to_hw_map = ctx_isp->hw_ctx;
		rc = ctx->hw_mgr_intf->hw_reset(ctx->hw_mgr_intf->hw_mgr_priv,
			&reset_args);
		if (rc)
			CAM_ERR(CAM_ISP, "Failed to reset HW rc: %d", rc);

		ctx_isp->init_received = false;
	}

end:
	ctx_isp->bubble_frame_cnt = 0;
	atomic_set(&ctx_isp->process_bubble, 0);
	atomic_set(&ctx_isp->rxd_epoch, 0);
	return rc;
}

static int __cam_isp_ctx_flush_req_in_ready(
	struct cam_context *ctx,
	struct cam_req_mgr_flush_request *flush_req)
{
	int rc = 0;

	CAM_DBG(CAM_ISP, "try to flush pending list");
	spin_lock_bh(&ctx->lock);
	rc = __cam_isp_ctx_flush_req(ctx, &ctx->pending_req_list, flush_req);

	/* if nothing is in pending req list, change state to acquire */
	if (list_empty(&ctx->pending_req_list))
		ctx->state = CAM_CTX_ACQUIRED;
	spin_unlock_bh(&ctx->lock);

	trace_cam_context_state("ISP", ctx);

	CAM_DBG(CAM_ISP, "Flush request in ready state. next state %d",
		 ctx->state);
	return rc;
}

static struct cam_ctx_ops
	cam_isp_ctx_activated_state_machine[CAM_ISP_CTX_ACTIVATED_MAX] = {
	/* SOF */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_apply_req_in_sof,
		},
		.irq_ops = NULL,
	},
	/* APPLIED */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* EPOCH */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_apply_req_in_epoch,
		},
		.irq_ops = NULL,
	},
	/* BUBBLE */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_apply_req_in_bubble,
		},
		.irq_ops = NULL,
	},
	/* Bubble Applied */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* HW ERROR */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* HALT */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
};

static struct cam_ctx_ops
	cam_isp_ctx_fs2_state_machine[CAM_ISP_CTX_ACTIVATED_MAX] = {
	/* SOF */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_apply_req_in_sof,
		},
		.irq_ops = NULL,
	},
	/* APPLIED */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* EPOCH */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_apply_req_in_epoch,
		},
		.irq_ops = NULL,
	},
	/* BUBBLE */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_apply_req_in_bubble,
		},
		.irq_ops = NULL,
	},
	/* Bubble Applied */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* HW ERROR */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* HALT */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
};

static int __cam_isp_ctx_rdi_only_sof_in_top_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	int rc = 0;
	struct cam_context                    *ctx = ctx_isp->base;
	struct cam_req_mgr_trigger_notify      notify;
	struct cam_isp_hw_sof_event_data      *sof_event_data = evt_data;
	uint64_t                               request_id  = 0;

	if (!evt_data) {
		CAM_ERR(CAM_ISP, "in valid sof event data");
		return -EINVAL;
	}

	ctx_isp->frame_id++;
	ctx_isp->sof_timestamp_val = sof_event_data->timestamp;
	ctx_isp->boot_timestamp = sof_event_data->boot_time;

	CAM_DBG(CAM_ISP, "frame id: %lld time stamp:0x%llx",
		ctx_isp->frame_id, ctx_isp->sof_timestamp_val);

	/*
	 * notify reqmgr with sof signal. Note, due to scheduling delay
	 * we can run into situation that two active requests has already
	 * be in the active queue while we try to do the notification.
	 * In this case, we need to skip the current notification. This
	 * helps the state machine to catch up the delay.
	 */
	if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_trigger &&
		ctx_isp->active_req_cnt <= 2) {
		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.frame_id = ctx_isp->frame_id;
		notify.trigger = CAM_TRIGGER_POINT_SOF;
		notify.req_id = ctx_isp->req_info.last_bufdone_req_id;
		notify.sof_timestamp_val = ctx_isp->sof_timestamp_val;

		ctx->ctx_crm_intf->notify_trigger(&notify);
		CAM_DBG(CAM_ISP, "Notify CRM  SOF frame %lld",
			ctx_isp->frame_id);

		/*
		 * It is idle frame with out any applied request id, send
		 * request id as zero
		 */
		__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_SUCCESS);
	} else {
		CAM_ERR_RATE_LIMIT(CAM_ISP, "Can not notify SOF to CRM");
	}

	if (list_empty(&ctx->active_req_list))
		ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;
	else
		CAM_DBG(CAM_ISP, "Still need to wait for the buf done");

	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));
	return rc;
}

static int __cam_isp_ctx_rdi_only_sof_in_applied_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	struct cam_isp_hw_sof_event_data      *sof_event_data = evt_data;

	if (!evt_data) {
		CAM_ERR(CAM_ISP, "in valid sof event data");
		return -EINVAL;
	}

	ctx_isp->frame_id++;
	ctx_isp->sof_timestamp_val = sof_event_data->timestamp;
	ctx_isp->boot_timestamp = sof_event_data->boot_time;
	CAM_DBG(CAM_ISP, "frame id: %lld time stamp:0x%llx",
		ctx_isp->frame_id, ctx_isp->sof_timestamp_val);

	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_BUBBLE_APPLIED;
	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	return 0;
}

static int __cam_isp_ctx_rdi_only_sof_in_bubble_applied(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	struct cam_ctx_request    *req;
	struct cam_isp_ctx_req    *req_isp;
	struct cam_context        *ctx = ctx_isp->base;
	struct cam_isp_hw_sof_event_data      *sof_event_data = evt_data;
	uint64_t  request_id = 0;

	/*
	 * Sof in bubble applied state means, reg update not received.
	 * before increment frame id and override time stamp value, send
	 * the previous sof time stamp that got captured in the
	 * sof in applied state.
	 */
	CAM_DBG(CAM_ISP, "frame id: %lld time stamp:0x%llx",
		ctx_isp->frame_id, ctx_isp->sof_timestamp_val);
	__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
		CAM_REQ_MGR_SOF_EVENT_SUCCESS);

	ctx_isp->frame_id++;
	ctx_isp->sof_timestamp_val = sof_event_data->timestamp;
	ctx_isp->boot_timestamp = sof_event_data->boot_time;
	CAM_DBG(CAM_ISP, "frame id: %lld time stamp:0x%llx",
		ctx_isp->frame_id, ctx_isp->sof_timestamp_val);

	if (list_empty(&ctx->wait_req_list)) {
		/*
		 * If no pending req in epoch, this is an error case.
		 * The recovery is to go back to sof state
		 */
		CAM_ERR(CAM_ISP, "No wait request");
		ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;

		/* Send SOF event as empty frame*/
		__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_SUCCESS);

		goto end;
	}

	req = list_first_entry(&ctx->wait_req_list, struct cam_ctx_request,
		list);
	req_isp = (struct cam_isp_ctx_req *)req->req_priv;
	req_isp->bubble_detected = true;
	CAM_INFO_RATE_LIMIT(CAM_ISP, "Ctx:%d Report Bubble flag %d req id:%lld",
		ctx->ctx_id, req_isp->bubble_report, req->request_id);
	req_isp->reapply = true;
	req_isp->cdm_reset_before_apply = false;

	if (req_isp->bubble_report && ctx->ctx_crm_intf &&
		ctx->ctx_crm_intf->notify_err) {
		struct cam_req_mgr_error_notify notify;

		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.req_id = req->request_id;
		notify.error = CRM_KMD_ERR_BUBBLE;
		notify.trigger = 0;
		if (ctx_isp->subscribe_event & CAM_TRIGGER_POINT_SOF)
			notify.trigger = CAM_TRIGGER_POINT_SOF;
		notify.frame_id = ctx_isp->frame_id;
		notify.sof_timestamp_val = ctx_isp->sof_timestamp_val;
		CAM_WARN_RATE_LIMIT(CAM_ISP,
			"Notify CRM about Bubble req %lld frame %lld ctx %u",
			req->request_id,
			ctx_isp->frame_id,
			ctx->ctx_id);
		ctx->ctx_crm_intf->notify_err(&notify);
		atomic_set(&ctx_isp->process_bubble, 1);
	} else {
		req_isp->bubble_report = 0;
	}

	/*
	 * Always move the request to active list. Let buf done
	 * function handles the rest.
	 */
	list_del_init(&req->list);
	list_add_tail(&req->list, &ctx->active_req_list);
	ctx_isp->active_req_cnt++;
	CAM_DBG(CAM_ISP, "move request %lld to active list(cnt = %d)",
			req->request_id, ctx_isp->active_req_cnt);

	if (!req_isp->bubble_report) {
		if (req->request_id > ctx_isp->reported_req_id) {
			request_id = req->request_id;
			ctx_isp->reported_req_id = request_id;
			__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_ERROR);
		} else
			__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
				CAM_REQ_MGR_SOF_EVENT_SUCCESS);
	} else
		__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
			CAM_REQ_MGR_SOF_EVENT_SUCCESS);

	/* change the state to bubble, as reg update has not come */
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_BUBBLE;
	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));
end:
	return 0;
}

static int __cam_isp_ctx_rdi_only_sof_in_bubble_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	uint32_t i;
	struct cam_ctx_request                *req;
	struct cam_context                    *ctx = ctx_isp->base;
	struct cam_req_mgr_trigger_notify      notify;
	struct cam_isp_hw_sof_event_data      *sof_event_data = evt_data;
	struct cam_isp_ctx_req                *req_isp;
	struct cam_hw_cmd_args                 hw_cmd_args;
	struct cam_isp_hw_cmd_args             isp_hw_cmd_args;
	uint64_t                               request_id  = 0;
	uint64_t                               last_cdm_done_req = 0;
	int                                    rc = 0;

	if (!evt_data) {
		CAM_ERR(CAM_ISP, "in valid sof event data");
		return -EINVAL;
	}

	ctx_isp->frame_id++;
	ctx_isp->sof_timestamp_val = sof_event_data->timestamp;
	ctx_isp->boot_timestamp = sof_event_data->boot_time;
	CAM_DBG(CAM_ISP, "frame id: %lld time stamp:0x%llx",
		ctx_isp->frame_id, ctx_isp->sof_timestamp_val);


	if (atomic_read(&ctx_isp->process_bubble)) {
		if (list_empty(&ctx->active_req_list)) {
			CAM_ERR(CAM_ISP, "No available active req in bubble");
			atomic_set(&ctx_isp->process_bubble, 0);
			return -EINVAL;
		}

		if (ctx_isp->last_sof_timestamp ==
			ctx_isp->sof_timestamp_val) {
			CAM_DBG(CAM_ISP,
				"Tasklet delay detected! Bubble frame: %lld check skipped, sof_timestamp: %lld, ctx_id: %d",
				ctx_isp->frame_id,
				ctx_isp->sof_timestamp_val,
				ctx->ctx_id);
			goto end;
		}

		req = list_first_entry(&ctx->active_req_list,
				struct cam_ctx_request, list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;

		if (req_isp->bubble_detected) {
			hw_cmd_args.ctxt_to_hw_map = ctx_isp->hw_ctx;
			hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
			isp_hw_cmd_args.cmd_type =
				CAM_ISP_HW_MGR_GET_LAST_CDM_DONE;
			hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;
			rc = ctx->hw_mgr_intf->hw_cmd(
				ctx->hw_mgr_intf->hw_mgr_priv,
				&hw_cmd_args);
			if (rc) {
				CAM_ERR(CAM_ISP, "HW command failed");
				return rc;
			}

			last_cdm_done_req = isp_hw_cmd_args.u.last_cdm_done;
			CAM_DBG(CAM_ISP, "last_cdm_done req: %d ctx_id: %d",
				last_cdm_done_req, ctx->ctx_id);

			if (last_cdm_done_req >= req->request_id) {
				CAM_DBG(CAM_ISP,
					"CDM callback detected for req: %lld, possible buf_done delay, waiting for buf_done",
					req->request_id);
				if (req_isp->num_fence_map_out ==
					req_isp->num_deferred_acks) {
					__cam_isp_handle_deferred_buf_done(ctx_isp, req,
						true,
						CAM_SYNC_STATE_SIGNALED_ERROR,
						CAM_SYNC_ISP_EVENT_BUBBLE);

					__cam_isp_ctx_handle_buf_done_for_req_list(
						ctx_isp, req);
				}
				goto end;
			} else {
				CAM_WARN(CAM_ISP,
					"CDM callback not happened for req: %lld, possible CDM stuck or workqueue delay",
					req->request_id);
				req_isp->num_acked = 0;
				req_isp->num_deferred_acks = 0;
				req_isp->bubble_detected = false;
				req_isp->cdm_reset_before_apply = true;
				list_del_init(&req->list);
				list_add(&req->list, &ctx->pending_req_list);
				atomic_set(&ctx_isp->process_bubble, 0);
				ctx_isp->active_req_cnt--;
				CAM_DBG(CAM_REQ,
					"Move active req: %lld to pending list(cnt = %d) [bubble re-apply],ctx %u",
					req->request_id,
					ctx_isp->active_req_cnt, ctx->ctx_id);
			}
			goto end;
		}
	}

	/*
	 * Signal all active requests with error and move the  all the active
	 * requests to free list
	 */
	while (!list_empty(&ctx->active_req_list)) {
		req = list_first_entry(&ctx->active_req_list,
				struct cam_ctx_request, list);
		list_del_init(&req->list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		CAM_DBG(CAM_ISP, "signal fence in active list. fence num %d",
			req_isp->num_fence_map_out);
		for (i = 0; i < req_isp->num_fence_map_out; i++)
			if (req_isp->fence_map_out[i].sync_id != -1) {
				cam_sync_signal(
					req_isp->fence_map_out[i].sync_id,
					CAM_SYNC_STATE_SIGNALED_ERROR,
					CAM_SYNC_ISP_EVENT_BUBBLE);
			}
		list_add_tail(&req->list, &ctx->free_req_list);
		ctx_isp->active_req_cnt--;
	}

end:
	/* notify reqmgr with sof signal */
	if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_trigger) {
		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.frame_id = ctx_isp->frame_id;
		notify.trigger = CAM_TRIGGER_POINT_SOF;
		notify.req_id = ctx_isp->req_info.last_bufdone_req_id;
		notify.sof_timestamp_val = ctx_isp->sof_timestamp_val;

		ctx->ctx_crm_intf->notify_trigger(&notify);
		CAM_DBG(CAM_ISP, "Notify CRM  SOF frame %lld",
			ctx_isp->frame_id);

	} else {
		CAM_ERR(CAM_ISP, "Can not notify SOF to CRM");
	}

	/*
	 * It is idle frame with out any applied request id, send
	 * request id as zero
	 */
	__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
		CAM_REQ_MGR_SOF_EVENT_SUCCESS);

	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;

	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	ctx_isp->last_sof_timestamp = ctx_isp->sof_timestamp_val;
	return 0;
}


static int __cam_isp_ctx_rdi_only_reg_upd_in_bubble_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	struct cam_ctx_request  *req = NULL;
	struct cam_context      *ctx = ctx_isp->base;

	req = list_first_entry(&ctx->active_req_list,
		struct cam_ctx_request, list);

	CAM_INFO(CAM_ISP, "Received RUP for Bubble Request", req->request_id);

	return 0;
}

static int __cam_isp_ctx_rdi_only_reg_upd_in_bubble_applied_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	struct cam_ctx_request  *req = NULL;
	struct cam_context      *ctx = ctx_isp->base;
	struct cam_isp_ctx_req  *req_isp;
	struct cam_req_mgr_trigger_notify  notify;
	uint64_t  request_id  = 0;

	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_EPOCH;
	/* notify reqmgr with sof signal*/
	if (ctx->ctx_crm_intf && ctx->ctx_crm_intf->notify_trigger) {
		if (list_empty(&ctx->wait_req_list)) {
			CAM_ERR(CAM_ISP, "Reg upd ack with no waiting request");
			goto error;
		}
		req = list_first_entry(&ctx->wait_req_list,
				struct cam_ctx_request, list);
		list_del_init(&req->list);

		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		request_id =
			(req_isp->hw_update_data.packet_opcode_type ==
				CAM_ISP_PACKET_INIT_DEV) ?
			0 : req->request_id;

		if (req_isp->num_fence_map_out != 0) {
			list_add_tail(&req->list, &ctx->active_req_list);
			ctx_isp->active_req_cnt++;
			CAM_DBG(CAM_ISP,
				"move request %lld to active list(cnt = %d)",
				req->request_id, ctx_isp->active_req_cnt);
			/* if packet has buffers, set correct request id */
			request_id = req->request_id;
		} else {
			/* no io config, so the request is completed. */
			list_add_tail(&req->list, &ctx->free_req_list);
			CAM_DBG(CAM_ISP,
				"move active req %lld to free list(cnt=%d)",
				req->request_id, ctx_isp->active_req_cnt);
		}

		notify.link_hdl = ctx->link_hdl;
		notify.dev_hdl = ctx->dev_hdl;
		notify.frame_id = ctx_isp->frame_id;
		notify.trigger = CAM_TRIGGER_POINT_SOF;
		notify.req_id = ctx_isp->req_info.last_bufdone_req_id;
		notify.sof_timestamp_val = ctx_isp->sof_timestamp_val;

		ctx->ctx_crm_intf->notify_trigger(&notify);
		CAM_DBG(CAM_ISP, "Notify CRM  SOF frame %lld",
			ctx_isp->frame_id);
	} else {
		CAM_ERR(CAM_ISP, "Can not notify SOF to CRM");
	}
	if (request_id)
		ctx_isp->reported_req_id = request_id;

	__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
		CAM_REQ_MGR_SOF_EVENT_SUCCESS);
	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));
	__cam_isp_ctx_update_event_record(ctx_isp,
		CAM_ISP_CTX_EVENT_RUP, req);
	return 0;
error:
	/* Send SOF event as idle frame*/
	__cam_isp_ctx_send_sof_timestamp(ctx_isp, request_id,
		CAM_REQ_MGR_SOF_EVENT_SUCCESS);
	__cam_isp_ctx_update_event_record(ctx_isp,
		CAM_ISP_CTX_EVENT_RUP, NULL);

	/*
	 * There is no request in the pending list, move the sub state machine
	 * to SOF sub state
	 */
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;

	return 0;
}

static int __cam_isp_ctx_rdi_only_reg_upd_in_applied_state(
	struct cam_isp_context *ctx_isp, void *evt_data)
{
	struct cam_ctx_request  *req = NULL;
	struct cam_context      *ctx = ctx_isp->base;
	struct cam_isp_ctx_req  *req_isp;
	uint64_t  request_id  = 0;

	if (list_empty(&ctx->wait_req_list)) {
		CAM_ERR(CAM_ISP, "Reg upd ack with no waiting request");
		goto end;
	}
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_EPOCH;

	req = list_first_entry(&ctx->wait_req_list,
			struct cam_ctx_request, list);
	list_del_init(&req->list);

	req_isp = (struct cam_isp_ctx_req *) req->req_priv;

	request_id = (req_isp->hw_update_data.packet_opcode_type ==
			CAM_ISP_PACKET_INIT_DEV) ? 0 : req->request_id;

	if (req_isp->num_fence_map_out != 0) {
		list_add_tail(&req->list, &ctx->active_req_list);
		ctx_isp->active_req_cnt++;
		request_id = req->request_id;
		CAM_DBG(CAM_REQ,
			"move request %lld to active list(cnt = %d), ctx %u",
			req->request_id, ctx_isp->active_req_cnt, ctx->ctx_id);
		__cam_isp_ctx_update_event_record(ctx_isp,
			CAM_ISP_CTX_EVENT_RUP, req);
	} else {
		/* no io config, so the request is completed. */
		list_add_tail(&req->list, &ctx->free_req_list);
		CAM_DBG(CAM_ISP,
			"move active request %lld to free list(cnt = %d), ctx %u",
			req->request_id, ctx_isp->active_req_cnt, ctx->ctx_id);
	}

	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	__cam_isp_ctx_update_event_record(ctx_isp,
		CAM_ISP_CTX_EVENT_RUP, req);

	return 0;
end:
	__cam_isp_ctx_update_event_record(ctx_isp,
		CAM_ISP_CTX_EVENT_RUP, NULL);
	/*
	 * There is no request in the pending list, move the sub state machine
	 * to SOF sub state
	 */
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;
	return 0;
}

static struct cam_isp_ctx_irq_ops
	cam_isp_ctx_rdi_only_activated_state_machine_irq
			[CAM_ISP_CTX_ACTIVATED_MAX] = {
	/* SOF */
	{
		.irq_ops = {
			NULL,
			__cam_isp_ctx_rdi_only_sof_in_top_state,
			__cam_isp_ctx_reg_upd_in_sof,
			NULL,
			__cam_isp_ctx_notify_eof_in_activated_state,
			NULL,
		},
	},
	/* APPLIED */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_rdi_only_sof_in_applied_state,
			__cam_isp_ctx_rdi_only_reg_upd_in_applied_state,
			NULL,
			__cam_isp_ctx_notify_eof_in_activated_state,
			__cam_isp_ctx_buf_done_in_applied,
		},
	},
	/* EPOCH */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_rdi_only_sof_in_top_state,
			NULL,
			NULL,
			__cam_isp_ctx_notify_eof_in_activated_state,
			__cam_isp_ctx_buf_done_in_epoch,
		},
	},
	/* BUBBLE*/
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_rdi_only_sof_in_bubble_state,
			__cam_isp_ctx_rdi_only_reg_upd_in_bubble_state,
			NULL,
			__cam_isp_ctx_notify_eof_in_activated_state,
			__cam_isp_ctx_buf_done_in_bubble,
		},
	},
	/* BUBBLE APPLIED ie PRE_BUBBLE */
	{
		.irq_ops = {
			__cam_isp_ctx_handle_error,
			__cam_isp_ctx_rdi_only_sof_in_bubble_applied,
			__cam_isp_ctx_rdi_only_reg_upd_in_bubble_applied_state,
			NULL,
			__cam_isp_ctx_notify_eof_in_activated_state,
			__cam_isp_ctx_buf_done_in_bubble_applied,
		},
	},
	/* HW ERROR */
	{
	},
	/* HALT */
	{
	},
};

static int __cam_isp_ctx_rdi_only_apply_req_top_state(
	struct cam_context *ctx, struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	CAM_DBG(CAM_ISP, "current Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));
	rc = __cam_isp_ctx_apply_req_in_activated_state(ctx, apply,
		CAM_ISP_CTX_ACTIVATED_APPLIED);
	CAM_DBG(CAM_ISP, "new Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	if (rc)
		CAM_ERR_RATE_LIMIT(CAM_ISP,
			"ctx_id:%d Apply failed in Substate[%s], rc %d",
			ctx->ctx_id,
			__cam_isp_ctx_substate_val_to_type(
			ctx_isp->substate_activated), rc);

	return rc;
}

static struct cam_ctx_ops
	cam_isp_ctx_rdi_only_activated_state_machine
		[CAM_ISP_CTX_ACTIVATED_MAX] = {
	/* SOF */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_rdi_only_apply_req_top_state,
		},
		.irq_ops = NULL,
	},
	/* APPLIED */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* EPOCH */
	{
		.ioctl_ops = {},
		.crm_ops = {
			.apply_req = __cam_isp_ctx_rdi_only_apply_req_top_state,
		},
		.irq_ops = NULL,
	},
	/* PRE BUBBLE */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* BUBBLE */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* HW ERROR */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* HALT */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
};

static int __cam_isp_ctx_release_hw_in_top_state(struct cam_context *ctx,
	void *cmd)
{
	int rc = 0;
	struct cam_hw_release_args       rel_arg;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;
	struct cam_req_mgr_flush_request flush_req;
	int i;

	if (ctx_isp->hw_ctx) {
		rel_arg.ctxt_to_hw_map = ctx_isp->hw_ctx;
		ctx->hw_mgr_intf->hw_release(ctx->hw_mgr_intf->hw_mgr_priv,
			&rel_arg);
		ctx_isp->hw_ctx = NULL;
	} else {
		CAM_ERR(CAM_ISP, "No hw resources acquired for ctx[%u]", ctx->ctx_id);
	}

	ctx->last_flush_req = 0;
	ctx_isp->custom_enabled = false;
	ctx_isp->use_frame_header_ts = false;
	ctx_isp->frame_id = 0;
	ctx_isp->active_req_cnt = 0;
	ctx_isp->reported_req_id = 0;
	ctx_isp->hw_acquired = false;
	ctx_isp->init_received = false;
	ctx_isp->support_consumed_addr = false;
	ctx_isp->req_info.last_bufdone_req_id = 0;

	atomic64_set(&ctx_isp->state_monitor_head, -1);

	for (i = 0; i < CAM_ISP_CTX_EVENT_MAX; i++)
		atomic64_set(&ctx_isp->event_record_head[i], -1);
	/*
	 * Ideally, we should never have any active request here.
	 * But we still add some sanity check code here to help the debug
	 */
	if (!list_empty(&ctx->active_req_list))
		CAM_WARN(CAM_ISP, "Active list is not empty");

	/* Flush all the pending request list  */
	flush_req.type = CAM_REQ_MGR_FLUSH_TYPE_ALL;
	flush_req.link_hdl = ctx->link_hdl;
	flush_req.dev_hdl = ctx->dev_hdl;
	flush_req.req_id = 0;

	CAM_DBG(CAM_ISP, "try to flush pending list");
	spin_lock_bh(&ctx->lock);
	rc = __cam_isp_ctx_flush_req(ctx, &ctx->pending_req_list, &flush_req);
	spin_unlock_bh(&ctx->lock);
	ctx->state = CAM_CTX_ACQUIRED;

	trace_cam_context_state("ISP", ctx);
	CAM_DBG(CAM_ISP, "Release device success[%u] next state %d",
		ctx->ctx_id, ctx->state);
	return rc;
}

/* top level state machine */
static int __cam_isp_ctx_release_dev_in_top_state(struct cam_context *ctx,
	struct cam_release_dev_cmd *cmd)
{
	int rc = 0;
	int i;
	struct cam_hw_release_args       rel_arg;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;
	struct cam_req_mgr_flush_request flush_req;

	if (cmd && ctx_isp->hw_ctx) {
		CAM_ERR(CAM_ISP, "releasing hw");
		__cam_isp_ctx_release_hw_in_top_state(ctx, NULL);
	}

	if (ctx_isp->hw_ctx) {
		rel_arg.ctxt_to_hw_map = ctx_isp->hw_ctx;
		ctx->hw_mgr_intf->hw_release(ctx->hw_mgr_intf->hw_mgr_priv,
			&rel_arg);
		ctx_isp->hw_ctx = NULL;
	}

	ctx->session_hdl = -1;
	ctx->dev_hdl = -1;
	ctx->link_hdl = -1;
	ctx->ctx_crm_intf = NULL;
	ctx->last_flush_req = 0;
	ctx_isp->frame_id = 0;
	ctx_isp->active_req_cnt = 0;
	ctx_isp->reported_req_id = 0;
	ctx_isp->hw_acquired = false;
	ctx_isp->init_received = false;
	ctx_isp->offline_context = false;
	ctx_isp->rdi_only_context = false;
	ctx_isp->req_info.last_bufdone_req_id = 0;

	atomic64_set(&ctx_isp->state_monitor_head, -1);
	for (i = 0; i < CAM_ISP_CTX_EVENT_MAX; i++)
		atomic64_set(&ctx_isp->event_record_head[i], -1);
	/*
	 * Ideally, we should never have any active request here.
	 * But we still add some sanity check code here to help the debug
	 */
	if (!list_empty(&ctx->active_req_list))
		CAM_ERR(CAM_ISP, "Active list is not empty");

	/* Flush all the pending request list  */
	flush_req.type = CAM_REQ_MGR_FLUSH_TYPE_ALL;
	flush_req.link_hdl = ctx->link_hdl;
	flush_req.dev_hdl = ctx->dev_hdl;
	flush_req.req_id = 0;

	CAM_DBG(CAM_ISP, "try to flush pending list");
	spin_lock_bh(&ctx->lock);
	rc = __cam_isp_ctx_flush_req(ctx, &ctx->pending_req_list, &flush_req);
	spin_unlock_bh(&ctx->lock);
	ctx->state = CAM_CTX_AVAILABLE;

	trace_cam_context_state("ISP", ctx);
	CAM_DBG(CAM_ISP, "Release device success[%u] next state %d",
		ctx->ctx_id, ctx->state);
	return rc;
}

static int __cam_isp_ctx_config_dev_in_top_state(
	struct cam_context *ctx, struct cam_config_dev_cmd *cmd)
{
	int rc = 0, i;
	struct cam_ctx_request           *req = NULL;
	struct cam_isp_ctx_req           *req_isp;
	uintptr_t                         packet_addr;
	struct cam_packet                *packet;
	size_t                            len = 0;
	size_t                            remain_len = 0;
	struct cam_hw_prepare_update_args cfg = {0};
	struct cam_req_mgr_add_request    add_req;
	struct cam_isp_context           *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;
	struct cam_hw_cmd_args           hw_cmd_args;
	struct cam_isp_hw_cmd_args       isp_hw_cmd_args;
	uint32_t                         packet_opcode = 0;

	CAM_DBG(CAM_ISP, "get free request object......");

	/* get free request */
	spin_lock_bh(&ctx->lock);
	if (!list_empty(&ctx->free_req_list)) {
		req = list_first_entry(&ctx->free_req_list,
				struct cam_ctx_request, list);
		list_del_init(&req->list);
	}
	spin_unlock_bh(&ctx->lock);

	if (!req) {
		CAM_ERR(CAM_ISP, "No more request obj free");
		return -ENOMEM;
	}

	req_isp = (struct cam_isp_ctx_req *) req->req_priv;

	/* for config dev, only memory handle is supported */
	/* map packet from the memhandle */
	rc = cam_mem_get_cpu_buf((int32_t) cmd->packet_handle,
		&packet_addr, &len);
	if (rc != 0) {
		CAM_ERR(CAM_ISP, "Can not get packet address");
		rc = -EINVAL;
		goto free_req;
	}

	remain_len = len;
	if ((len < sizeof(struct cam_packet)) ||
		((size_t)cmd->offset >= len - sizeof(struct cam_packet))) {
		CAM_ERR(CAM_ISP, "invalid buff length: %zu or offset", len);
		rc = -EINVAL;
		goto free_req;
	}

	remain_len -= (size_t)cmd->offset;
	packet = (struct cam_packet *)(packet_addr + (uint32_t)cmd->offset);
	CAM_DBG(CAM_ISP, "pack_handle %llx", cmd->packet_handle);
	CAM_DBG(CAM_ISP, "packet address is 0x%zx", packet_addr);
	CAM_DBG(CAM_ISP, "packet with length %zu, offset 0x%llx",
		len, cmd->offset);
	CAM_DBG(CAM_ISP, "Packet request id %lld",
		packet->header.request_id);
	CAM_DBG(CAM_ISP, "Packet size 0x%x", packet->header.size);
	CAM_DBG(CAM_ISP, "packet op %d", packet->header.op_code);

	/* Query the packet opcode */
	hw_cmd_args.ctxt_to_hw_map = ctx_isp->hw_ctx;
	hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
	isp_hw_cmd_args.cmd_type = CAM_ISP_HW_MGR_GET_PACKET_OPCODE;
	isp_hw_cmd_args.cmd_data = (void *)packet;
	hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;
	rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
		&hw_cmd_args);
	if (rc) {
		CAM_ERR(CAM_ISP, "HW command failed");
		goto free_req;
	}

	packet_opcode = isp_hw_cmd_args.u.packet_op_code;
	CAM_DBG(CAM_ISP, "packet op %d", packet_opcode);

	if ((packet_opcode == CAM_ISP_PACKET_UPDATE_DEV)
		&& (packet->header.request_id <= ctx->last_flush_req)) {
		CAM_INFO(CAM_ISP,
			"request %lld has been flushed, reject packet",
			packet->header.request_id);
		rc = -EBADR;
		goto free_req;
	} else if ((packet_opcode == CAM_ISP_PACKET_INIT_DEV)
		&& (packet->header.request_id <= ctx->last_flush_req)
		&& ctx->last_flush_req && packet->header.request_id) {
		CAM_WARN(CAM_ISP,
			"last flushed req is %lld, config dev(init) for req %lld",
			ctx->last_flush_req, packet->header.request_id);
		rc = -EBADR;
		goto free_req;
	}

	cfg.packet = packet;
	cfg.remain_len = remain_len;
	cfg.ctxt_to_hw_map = ctx_isp->hw_ctx;
	cfg.max_hw_update_entries = CAM_ISP_CTX_CFG_MAX;
	cfg.hw_update_entries = req_isp->cfg;
	cfg.max_out_map_entries = CAM_ISP_CTX_RES_MAX;
	cfg.max_in_map_entries = CAM_ISP_CTX_RES_MAX;
	cfg.out_map_entries = req_isp->fence_map_out;
	cfg.in_map_entries = req_isp->fence_map_in;
	cfg.priv  = &req_isp->hw_update_data;
	cfg.pf_data = &(req->pf_data);

	CAM_DBG(CAM_ISP, "try to prepare config packet......");

	rc = ctx->hw_mgr_intf->hw_prepare_update(
		ctx->hw_mgr_intf->hw_mgr_priv, &cfg);
	if (rc != 0) {
		CAM_ERR(CAM_ISP, "Prepare config packet failed in HW layer");
		rc = -EFAULT;
		goto free_req;
	}

	req_isp->num_cfg = cfg.num_hw_update_entries;
	req_isp->num_fence_map_out = cfg.num_out_map_entries;
	req_isp->num_fence_map_in = cfg.num_in_map_entries;
	req_isp->num_acked = 0;
	req_isp->num_deferred_acks = 0;
	req_isp->bubble_detected = false;
	req_isp->cdm_reset_before_apply = false;
	req_isp->hw_update_data.packet = packet;

	for (i = 0; i < req_isp->num_fence_map_out; i++) {
		rc = cam_sync_get_obj_ref(req_isp->fence_map_out[i].sync_id);
		if (rc) {
			CAM_ERR(CAM_ISP, "Can't get ref for fence %d",
				req_isp->fence_map_out[i].sync_id);
			goto put_ref;
		}
	}

	CAM_DBG(CAM_ISP, "num_entry: %d, num fence out: %d, num fence in: %d",
		req_isp->num_cfg, req_isp->num_fence_map_out,
		req_isp->num_fence_map_in);

	req->request_id = packet->header.request_id;
	req->status = 1;

	CAM_DBG(CAM_ISP, "Packet request id %lld packet opcode:%d",
		packet->header.request_id,
		req_isp->hw_update_data.packet_opcode_type);

	if (req_isp->hw_update_data.packet_opcode_type ==
		CAM_ISP_PACKET_INIT_DEV) {
		if (ctx->state < CAM_CTX_ACTIVATED) {
			rc = __cam_isp_ctx_enqueue_init_request(ctx, req);
			if (rc)
				CAM_ERR(CAM_ISP, "Enqueue INIT pkt failed");
			ctx_isp->init_received = true;
		} else {
			rc = -EINVAL;
			CAM_ERR(CAM_ISP, "Recevied INIT pkt in wrong state:%d",
				ctx->state);
		}
	} else {
		if (ctx_isp->offline_context) {
			__cam_isp_ctx_enqueue_request_in_order(ctx, req);
		} else if ((ctx->state != CAM_CTX_FLUSHED) &&
			(ctx->state >= CAM_CTX_READY) &&
			ctx->ctx_crm_intf->add_req) {
			memset(&add_req, 0, sizeof(add_req));
			add_req.link_hdl = ctx->link_hdl;
			add_req.dev_hdl  = ctx->dev_hdl;
			add_req.req_id   = req->request_id;
			rc = ctx->ctx_crm_intf->add_req(&add_req);
			if (rc) {
				CAM_ERR(CAM_ISP, "Add req failed: req id=%llu",
					req->request_id);
			} else {
				__cam_isp_ctx_enqueue_request_in_order(
					ctx, req);
			}
		} else {
			rc = -EINVAL;
			CAM_ERR(CAM_ISP,
				"Recevied update req %lld in wrong state:%d",
				req->request_id, ctx->state);
		}
	}
	if (rc)
		goto put_ref;

	CAM_DBG(CAM_REQ,
		"Preprocessing Config req_id %lld successful on ctx %u",
		req->request_id, ctx->ctx_id);

	if (ctx_isp->offline_context && atomic_read(&ctx_isp->rxd_epoch)) {
		__cam_isp_ctx_schedule_apply_req_offline(ctx_isp);
	}

	return rc;

put_ref:
	for (--i; i >= 0; i--) {
		if (cam_sync_put_obj_ref(req_isp->fence_map_out[i].sync_id))
			CAM_ERR(CAM_CTXT, "Failed to put ref of fence %d",
				req_isp->fence_map_out[i].sync_id);
	}
free_req:
	spin_lock_bh(&ctx->lock);
	list_add_tail(&req->list, &ctx->free_req_list);
	spin_unlock_bh(&ctx->lock);

	return rc;
}

static int __cam_isp_ctx_acquire_dev_in_available(struct cam_context *ctx,
	struct cam_acquire_dev_cmd *cmd)
{
	int rc = 0;
	int i;
	struct cam_hw_acquire_args       param;
	struct cam_isp_resource         *isp_res = NULL;
	struct cam_create_dev_hdl        req_hdl_param;
	struct cam_hw_release_args       release;
	struct cam_isp_context          *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;
	struct cam_hw_cmd_args           hw_cmd_args;
	struct cam_isp_hw_cmd_args       isp_hw_cmd_args;

	if (!ctx->hw_mgr_intf) {
		CAM_ERR(CAM_ISP, "HW interface is not ready");
		rc = -EFAULT;
		goto end;
	}

	CAM_DBG(CAM_ISP,
		"session_hdl 0x%x, num_resources %d, hdl type %d, res %lld",
		cmd->session_handle, cmd->num_resources,
		cmd->handle_type, cmd->resource_hdl);

	if (cmd->num_resources == CAM_API_COMPAT_CONSTANT) {
		ctx_isp->split_acquire = true;
		CAM_DBG(CAM_ISP, "Acquire dev handle");
		goto get_dev_handle;
	}

	if (cmd->num_resources > CAM_ISP_CTX_RES_MAX) {
		CAM_ERR(CAM_ISP, "Too much resources in the acquire");
		rc = -ENOMEM;
		goto end;
	}

	/* for now we only support user pointer */
	if (cmd->handle_type != 1)  {
		CAM_ERR(CAM_ISP, "Only user pointer is supported");
		rc = -EINVAL;
		goto end;
	}

	isp_res = kzalloc(
		sizeof(*isp_res)*cmd->num_resources, GFP_KERNEL);
	if (!isp_res) {
		rc = -ENOMEM;
		goto end;
	}

	CAM_DBG(CAM_ISP, "start copy %d resources from user",
		 cmd->num_resources);

	if (copy_from_user(isp_res, u64_to_user_ptr(cmd->resource_hdl),
		sizeof(*isp_res)*cmd->num_resources)) {
		rc = -EFAULT;
		goto free_res;
	}

	param.context_data = ctx;
	param.event_cb = ctx->irq_cb_intf;
	param.num_acq = cmd->num_resources;
	param.acquire_info = (uintptr_t) isp_res;

	/* call HW manager to reserve the resource */
	rc = ctx->hw_mgr_intf->hw_acquire(ctx->hw_mgr_intf->hw_mgr_priv,
		&param);
	if (rc != 0) {
		CAM_ERR(CAM_ISP, "Acquire device failed");
		goto free_res;
	}

	/* Query the context has rdi only resource */
	hw_cmd_args.ctxt_to_hw_map = param.ctxt_to_hw_map;
	hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
	isp_hw_cmd_args.cmd_type = CAM_ISP_HW_MGR_CMD_CTX_TYPE;
	hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;
	rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
				&hw_cmd_args);
	if (rc) {
		CAM_ERR(CAM_ISP, "HW command failed");
		goto free_hw;
	}

	if (isp_hw_cmd_args.u.ctx_type == CAM_ISP_CTX_RDI) {
		/*
		 * this context has rdi only resource assign rdi only
		 * state machine
		 */
		CAM_DBG(CAM_ISP, "RDI only session Context");

		ctx_isp->substate_machine_irq =
			cam_isp_ctx_rdi_only_activated_state_machine_irq;
		ctx_isp->substate_machine =
			cam_isp_ctx_rdi_only_activated_state_machine;
		ctx_isp->rdi_only_context = true;
	} else if (isp_hw_cmd_args.u.ctx_type == CAM_ISP_CTX_FS2) {
		CAM_DBG(CAM_ISP, "FS2 Session has PIX, RD and RDI");
		ctx_isp->substate_machine_irq =
			cam_isp_ctx_fs2_state_machine_irq;
		ctx_isp->substate_machine =
			cam_isp_ctx_fs2_state_machine;
	} else if (isp_hw_cmd_args.u.ctx_type == CAM_ISP_CTX_OFFLINE) {
		CAM_DBG(CAM_ISP, "offline Session has PIX and RD resources");
		ctx_isp->substate_machine_irq =
			cam_isp_ctx_offline_state_machine_irq;
	} else {
		CAM_DBG(CAM_ISP, "Session has PIX or PIX and RDI resources");
		ctx_isp->substate_machine_irq =
			cam_isp_ctx_activated_state_machine_irq;
		ctx_isp->substate_machine =
			cam_isp_ctx_activated_state_machine;
	}

	ctx_isp->hw_ctx = param.ctxt_to_hw_map;
	ctx_isp->hw_acquired = true;
	ctx_isp->split_acquire = false;
	ctx->ctxt_to_hw_map = param.ctxt_to_hw_map;

	atomic64_set(&ctx_isp->state_monitor_head, -1);
	for (i = 0; i < CAM_ISP_CTX_EVENT_MAX; i++)
		atomic64_set(&ctx_isp->event_record_head[i], -1);

	kfree(isp_res);
	isp_res = NULL;

get_dev_handle:

	req_hdl_param.session_hdl = cmd->session_handle;
	/* bridge is not ready for these flags. so false for now */
	req_hdl_param.v4l2_sub_dev_flag = 0;
	req_hdl_param.media_entity_flag = 0;
	req_hdl_param.ops = ctx->crm_ctx_intf;
	req_hdl_param.priv = ctx;
	req_hdl_param.dev_id = CAM_ISP;
	CAM_DBG(CAM_ISP, "get device handle form bridge");
	ctx->dev_hdl = cam_create_device_hdl(&req_hdl_param);
	if (ctx->dev_hdl <= 0) {
		rc = -EFAULT;
		CAM_ERR(CAM_ISP, "Can not create device handle");
		goto free_hw;
	}
	cmd->dev_handle = ctx->dev_hdl;

	/* store session information */
	ctx->session_hdl = cmd->session_handle;
	ctx->state = CAM_CTX_ACQUIRED;

	trace_cam_context_state("ISP", ctx);
	CAM_DBG(CAM_ISP,
		"Acquire success on session_hdl 0x%x num_rsrces %d ctx %u",
		cmd->session_handle, cmd->num_resources, ctx->ctx_id);

	return rc;

free_hw:
	release.ctxt_to_hw_map = ctx_isp->hw_ctx;
	if (ctx_isp->hw_acquired)
		ctx->hw_mgr_intf->hw_release(ctx->hw_mgr_intf->hw_mgr_priv,
			&release);
	ctx_isp->hw_ctx = NULL;
	ctx_isp->hw_acquired = false;
free_res:
	kfree(isp_res);
end:
	return rc;
}

static int __cam_isp_ctx_acquire_hw_v1(struct cam_context *ctx,
	void *args)
{
	int rc = 0;
	int i;
	struct cam_acquire_hw_cmd_v1 *cmd =
		(struct cam_acquire_hw_cmd_v1 *)args;
	struct cam_hw_acquire_args       param;
	struct cam_hw_release_args       release;
	struct cam_isp_context          *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;
	struct cam_hw_cmd_args           hw_cmd_args;
	struct cam_isp_hw_cmd_args       isp_hw_cmd_args;
	struct cam_isp_acquire_hw_info  *acquire_hw_info = NULL;

	if (!ctx->hw_mgr_intf) {
		CAM_ERR(CAM_ISP, "HW interface is not ready");
		rc = -EFAULT;
		goto end;
	}

	CAM_DBG(CAM_ISP,
		"session_hdl 0x%x, hdl type %d, res %lld",
		cmd->session_handle, cmd->handle_type, cmd->resource_hdl);

	/* for now we only support user pointer */
	if (cmd->handle_type != 1)  {
		CAM_ERR(CAM_ISP, "Only user pointer is supported");
		rc = -EINVAL;
		goto end;
	}

	if (cmd->data_size < sizeof(*acquire_hw_info)) {
		CAM_ERR(CAM_ISP, "data_size is not a valid value");
		goto end;
	}

	acquire_hw_info = kzalloc(cmd->data_size, GFP_KERNEL);
	if (!acquire_hw_info) {
		rc = -ENOMEM;
		goto end;
	}

	CAM_DBG(CAM_ISP, "start copy resources from user");

	if (copy_from_user(acquire_hw_info, (void __user *)cmd->resource_hdl,
		cmd->data_size)) {
		rc = -EFAULT;
		goto free_res;
	}

	memset(&param, 0, sizeof(param));
	param.context_data = ctx;
	param.event_cb = ctx->irq_cb_intf;
	param.num_acq = CAM_API_COMPAT_CONSTANT;
	param.acquire_info_size = cmd->data_size;
	param.acquire_info = (uint64_t) acquire_hw_info;

	/* call HW manager to reserve the resource */
	rc = ctx->hw_mgr_intf->hw_acquire(ctx->hw_mgr_intf->hw_mgr_priv,
		&param);
	if (rc != 0) {
		CAM_ERR(CAM_ISP, "Acquire device failed");
		goto free_res;
	}

	ctx_isp->support_consumed_addr =
		param.support_consumed_addr;

	/* Query the context has rdi only resource */
	hw_cmd_args.ctxt_to_hw_map = param.ctxt_to_hw_map;
	hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
	isp_hw_cmd_args.cmd_type = CAM_ISP_HW_MGR_CMD_CTX_TYPE;
	hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;
	rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
				&hw_cmd_args);
	if (rc) {
		CAM_ERR(CAM_ISP, "HW command failed");
		goto free_hw;
	}

	if (isp_hw_cmd_args.u.ctx_type == CAM_ISP_CTX_RDI) {
		/*
		 * this context has rdi only resource assign rdi only
		 * state machine
		 */
		CAM_DBG(CAM_ISP, "RDI only session Context");

		ctx_isp->substate_machine_irq =
			cam_isp_ctx_rdi_only_activated_state_machine_irq;
		ctx_isp->substate_machine =
			cam_isp_ctx_rdi_only_activated_state_machine;
		ctx_isp->rdi_only_context = true;
	} else if (isp_hw_cmd_args.u.ctx_type == CAM_ISP_CTX_FS2) {
		CAM_DBG(CAM_ISP, "FS2 Session has PIX, RD and RDI");
		ctx_isp->substate_machine_irq =
			cam_isp_ctx_fs2_state_machine_irq;
		ctx_isp->substate_machine =
			cam_isp_ctx_fs2_state_machine;
	} else if (isp_hw_cmd_args.u.ctx_type == CAM_ISP_CTX_OFFLINE) {
		CAM_DBG(CAM_ISP, "Offline session has PIX and RD resources");
		ctx_isp->substate_machine_irq =
			cam_isp_ctx_offline_state_machine_irq;
		ctx_isp->substate_machine = NULL;
	} else {
		CAM_DBG(CAM_ISP, "Session has PIX or PIX and RDI resources");
		ctx_isp->substate_machine_irq =
			cam_isp_ctx_activated_state_machine_irq;
		ctx_isp->substate_machine =
			cam_isp_ctx_activated_state_machine;
	}

	ctx_isp->hw_ctx = param.ctxt_to_hw_map;
	ctx_isp->hw_acquired = true;
	ctx->ctxt_to_hw_map = param.ctxt_to_hw_map;

	atomic64_set(&ctx_isp->state_monitor_head, -1);

	for (i = 0; i < CAM_ISP_CTX_EVENT_MAX; i++)
		atomic64_set(&ctx_isp->event_record_head[i], -1);

	trace_cam_context_state("ISP", ctx);
	CAM_DBG(CAM_ISP,
		"Acquire success on session_hdl 0x%xs ctx_type %d ctx_id %u",
		ctx->session_hdl, isp_hw_cmd_args.u.ctx_type, ctx->ctx_id);
	kfree(acquire_hw_info);
	return rc;

free_hw:
	release.ctxt_to_hw_map = ctx_isp->hw_ctx;
	ctx->hw_mgr_intf->hw_release(ctx->hw_mgr_intf->hw_mgr_priv, &release);
	ctx_isp->hw_ctx = NULL;
	ctx_isp->hw_acquired = false;
free_res:
	kfree(acquire_hw_info);
end:
	return rc;
}

static void cam_req_mgr_process_workq_offline_ife_worker(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

static int __cam_isp_ctx_acquire_hw_v2(struct cam_context *ctx,
	void *args)
{
	int rc = 0, i, j;
	struct cam_acquire_hw_cmd_v2 *cmd =
		(struct cam_acquire_hw_cmd_v2 *)args;
	struct cam_hw_acquire_args       param;
	struct cam_hw_release_args       release;
	struct cam_isp_context          *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;
	struct cam_hw_cmd_args           hw_cmd_args;
	struct cam_isp_hw_cmd_args       isp_hw_cmd_args;
	struct cam_isp_acquire_hw_info  *acquire_hw_info = NULL;

	if (!ctx->hw_mgr_intf) {
		CAM_ERR(CAM_ISP, "HW interface is not ready");
		rc = -EFAULT;
		goto end;
	}

	CAM_DBG(CAM_ISP,
		"session_hdl 0x%x, hdl type %d, res %lld",
		cmd->session_handle, cmd->handle_type, cmd->resource_hdl);

	/* for now we only support user pointer */
	if (cmd->handle_type != 1)  {
		CAM_ERR(CAM_ISP, "Only user pointer is supported");
		rc = -EINVAL;
		goto end;
	}

	if (cmd->data_size < sizeof(*acquire_hw_info)) {
		CAM_ERR(CAM_ISP, "data_size is not a valid value");
		goto end;
	}

	acquire_hw_info = kzalloc(cmd->data_size, GFP_KERNEL);
	if (!acquire_hw_info) {
		rc = -ENOMEM;
		goto end;
	}

	CAM_DBG(CAM_ISP, "start copy resources from user");

	if (copy_from_user(acquire_hw_info, (void __user *)cmd->resource_hdl,
		cmd->data_size)) {
		rc = -EFAULT;
		goto free_res;
	}

	memset(&param, 0, sizeof(param));
	param.context_data = ctx;
	param.event_cb = ctx->irq_cb_intf;
	param.num_acq = CAM_API_COMPAT_CONSTANT;
	param.acquire_info_size = cmd->data_size;
	param.acquire_info = (uint64_t) acquire_hw_info;

	/* call HW manager to reserve the resource */
	rc = ctx->hw_mgr_intf->hw_acquire(ctx->hw_mgr_intf->hw_mgr_priv,
		&param);
	if (rc != 0) {
		CAM_ERR(CAM_ISP, "Acquire device failed");
		goto free_res;
	}

	/* Set custom flag if applicable
	 * custom hw is supported only on v2
	 */
	ctx_isp->custom_enabled = param.custom_enabled;
	ctx_isp->use_frame_header_ts = param.use_frame_header_ts;
	ctx_isp->support_consumed_addr =
		param.support_consumed_addr;

	/* Query the context has rdi only resource */
	hw_cmd_args.ctxt_to_hw_map = param.ctxt_to_hw_map;
	hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
	isp_hw_cmd_args.cmd_type = CAM_ISP_HW_MGR_CMD_CTX_TYPE;
	hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;
	rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
				&hw_cmd_args);
	if (rc) {
		CAM_ERR(CAM_ISP, "HW command failed");
		goto free_hw;
	}

	if (param.valid_acquired_hw) {
		for (i = 0; i < CAM_MAX_ACQ_RES; i++)
			cmd->hw_info.acquired_hw_id[i] =
				param.acquired_hw_id[i];

		for (i = 0; i < CAM_MAX_ACQ_RES; i++)
			for (j = 0; j < CAM_MAX_HW_SPLIT; j++)
				cmd->hw_info.acquired_hw_path[i][j] =
					param.acquired_hw_path[i][j];
	}
	cmd->hw_info.valid_acquired_hw =
		param.valid_acquired_hw;

	cmd->hw_info.valid_acquired_hw = param.valid_acquired_hw;

	if (isp_hw_cmd_args.u.ctx_type == CAM_ISP_CTX_RDI) {
		/*
		 * this context has rdi only resource assign rdi only
		 * state machine
		 */
		CAM_DBG(CAM_ISP, "RDI only session Context");

		ctx_isp->substate_machine_irq =
			cam_isp_ctx_rdi_only_activated_state_machine_irq;
		ctx_isp->substate_machine =
			cam_isp_ctx_rdi_only_activated_state_machine;
		ctx_isp->rdi_only_context = true;
	} else if (isp_hw_cmd_args.u.ctx_type == CAM_ISP_CTX_FS2) {
		CAM_DBG(CAM_ISP, "FS2 Session has PIX, RD and RDI");
		ctx_isp->substate_machine_irq =
			cam_isp_ctx_fs2_state_machine_irq;
		ctx_isp->substate_machine =
			cam_isp_ctx_fs2_state_machine;
	} else if (isp_hw_cmd_args.u.ctx_type == CAM_ISP_CTX_OFFLINE) {
		CAM_DBG(CAM_ISP, "Offline Session has PIX and RD resources");
		ctx_isp->substate_machine_irq =
			cam_isp_ctx_offline_state_machine_irq;
		ctx_isp->substate_machine = NULL;
		ctx_isp->offline_context = true;

		rc = cam_req_mgr_workq_create("offline_ife", 20,
			&ctx_isp->workq, CRM_WORKQ_USAGE_IRQ, 0,
			cam_req_mgr_process_workq_offline_ife_worker);
		if (rc)
			CAM_ERR(CAM_ISP,
				"Failed to create workq for offline IFE rc:%d",
				rc);
	} else {
		CAM_DBG(CAM_ISP, "Session has PIX or PIX and RDI resources");
		ctx_isp->substate_machine_irq =
			cam_isp_ctx_activated_state_machine_irq;
		ctx_isp->substate_machine =
			cam_isp_ctx_activated_state_machine;
	}

	ctx_isp->hw_ctx = param.ctxt_to_hw_map;
	ctx_isp->hw_acquired = true;
	ctx->ctxt_to_hw_map = param.ctxt_to_hw_map;

	trace_cam_context_state("ISP", ctx);
	CAM_DBG(CAM_ISP,
		"Acquire success on session_hdl 0x%xs ctx_type %d ctx_id %u",
		ctx->session_hdl, isp_hw_cmd_args.u.ctx_type, ctx->ctx_id);
	kfree(acquire_hw_info);
	return rc;

free_hw:
	release.ctxt_to_hw_map = ctx_isp->hw_ctx;
	ctx->hw_mgr_intf->hw_release(ctx->hw_mgr_intf->hw_mgr_priv, &release);
	ctx_isp->hw_ctx = NULL;
	ctx_isp->hw_acquired = false;
free_res:
	kfree(acquire_hw_info);
end:
	return rc;
}

static int __cam_isp_ctx_acquire_hw_in_acquired(struct cam_context *ctx,
	void *args)
{
	int rc = -EINVAL;
	uint32_t api_version;

	if (!ctx || !args) {
		CAM_ERR(CAM_ISP, "Invalid input pointer");
		return rc;
	}

	api_version = *((uint32_t *)args);
	if (api_version == 1)
		rc = __cam_isp_ctx_acquire_hw_v1(ctx, args);
	else if (api_version == 2)
		rc = __cam_isp_ctx_acquire_hw_v2(ctx, args);
	else
		CAM_ERR(CAM_ISP, "Unsupported api version %d", api_version);

	return rc;
}

static int __cam_isp_ctx_config_dev_in_acquired(struct cam_context *ctx,
	struct cam_config_dev_cmd *cmd)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	if (!ctx_isp->hw_acquired) {
		CAM_ERR(CAM_ISP, "HW is not acquired, reject packet");
		return -EINVAL;
	}

	rc = __cam_isp_ctx_config_dev_in_top_state(ctx, cmd);

	if (!rc && ((ctx->link_hdl >= 0) || ctx_isp->offline_context)) {
		ctx->state = CAM_CTX_READY;
		trace_cam_context_state("ISP", ctx);
	}

	CAM_DBG(CAM_ISP, "next state %d", ctx->state);
	return rc;
}

static int __cam_isp_ctx_config_dev_in_flushed(struct cam_context *ctx,
	struct cam_config_dev_cmd *cmd)
{
	int rc = 0;
	struct cam_start_stop_dev_cmd start_cmd;
	struct cam_hw_cmd_args hw_cmd_args;
	struct cam_isp_hw_cmd_args isp_hw_cmd_args;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	if (!ctx_isp->hw_acquired) {
		CAM_ERR(CAM_ISP, "HW is not acquired, reject packet");
		rc = -EINVAL;
		goto end;
	}

	rc = __cam_isp_ctx_config_dev_in_top_state(ctx, cmd);
	if (rc)
		goto end;

	if (!ctx_isp->init_received) {
		CAM_WARN(CAM_ISP,
			"Received update packet in flushed state, skip start");
		goto end;
	}

	hw_cmd_args.ctxt_to_hw_map = ctx_isp->hw_ctx;
	hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
	isp_hw_cmd_args.cmd_type = CAM_ISP_HW_MGR_CMD_RESUME_HW;
	hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;
	rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
		&hw_cmd_args);
	if (rc) {
		CAM_ERR(CAM_ISP, "Failed to resume HW rc: %d", rc);
		goto end;
	}

	start_cmd.dev_handle = cmd->dev_handle;
	start_cmd.session_handle = cmd->session_handle;
	rc = __cam_isp_ctx_start_dev_in_ready(ctx, &start_cmd);
	if (rc)
		CAM_ERR(CAM_ISP,
			"Failed to re-start HW after flush rc: %d", rc);
	else
		CAM_INFO(CAM_ISP,
			"Received init after flush. Re-start HW complete in ctx:%d",
			ctx->ctx_id);

end:
	CAM_DBG(CAM_ISP, "next state %d sub_state:%d", ctx->state,
		ctx_isp->substate_activated);
	return rc;
}

static int __cam_isp_ctx_link_in_acquired(struct cam_context *ctx,
	struct cam_req_mgr_core_dev_link_setup *link)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	CAM_DBG(CAM_ISP, "Enter.........");

	ctx->link_hdl = link->link_hdl;
	ctx->ctx_crm_intf = link->crm_cb;
	ctx_isp->subscribe_event =
		CAM_TRIGGER_POINT_SOF | CAM_TRIGGER_POINT_EOF;
	ctx_isp->trigger_id = link->trigger_id;

	/* change state only if we had the init config */
	if (ctx_isp->init_received) {
		ctx->state = CAM_CTX_READY;
		trace_cam_context_state("ISP", ctx);
	}

	CAM_DBG(CAM_ISP, "next state %d", ctx->state);

	return rc;
}

static int __cam_isp_ctx_unlink_in_acquired(struct cam_context *ctx,
	struct cam_req_mgr_core_dev_link_setup *unlink)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	ctx->link_hdl = -1;
	ctx->ctx_crm_intf = NULL;
	ctx_isp->trigger_id = -1;

	return rc;
}

static int __cam_isp_ctx_get_dev_info_in_acquired(struct cam_context *ctx,
	struct cam_req_mgr_device_info *dev_info)
{
	int rc = 0;

	dev_info->dev_hdl = ctx->dev_hdl;
	strlcpy(dev_info->name, CAM_ISP_DEV_NAME, sizeof(dev_info->name));
	dev_info->dev_id = CAM_REQ_MGR_DEVICE_IFE;
	dev_info->p_delay = 1;
	dev_info->trigger = CAM_TRIGGER_POINT_SOF;
	dev_info->trigger_on = true;

	return rc;
}

static int __cam_isp_ctx_start_dev_in_ready(struct cam_context *ctx,
	struct cam_start_stop_dev_cmd *cmd)
{
	int rc = 0;
	int i;
	struct cam_isp_start_args        start_isp;
	struct cam_ctx_request          *req;
	struct cam_isp_ctx_req          *req_isp;
	struct cam_isp_context          *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	if (cmd->session_handle != ctx->session_hdl ||
		cmd->dev_handle != ctx->dev_hdl) {
		rc = -EPERM;
		goto end;
	}

	if (list_empty(&ctx->pending_req_list)) {
		/* should never happen */
		CAM_ERR(CAM_ISP, "Start device with empty configuration");
		rc = -EFAULT;
		goto end;
	} else {
		req = list_first_entry(&ctx->pending_req_list,
			struct cam_ctx_request, list);
	}
	req_isp = (struct cam_isp_ctx_req *) req->req_priv;

	if (!ctx_isp->hw_ctx) {
		CAM_ERR(CAM_ISP, "Wrong hw context pointer.");
		rc = -EFAULT;
		goto end;
	}

	start_isp.hw_config.ctxt_to_hw_map = ctx_isp->hw_ctx;
	start_isp.hw_config.request_id = req->request_id;
	start_isp.hw_config.hw_update_entries = req_isp->cfg;
	start_isp.hw_config.num_hw_update_entries = req_isp->num_cfg;
	start_isp.hw_config.priv  = &req_isp->hw_update_data;
	start_isp.hw_config.init_packet = 1;
	start_isp.hw_config.reapply = 0;
	start_isp.hw_config.cdm_reset_before_apply = false;

	ctx_isp->last_applied_req_id = req->request_id;

	if (ctx->state == CAM_CTX_FLUSHED)
		start_isp.start_only = true;
	else
		start_isp.start_only = false;

	atomic_set(&ctx_isp->process_bubble, 0);
	atomic_set(&ctx_isp->rxd_epoch, 0);
	ctx_isp->frame_id = 0;
	ctx_isp->active_req_cnt = 0;
	ctx_isp->reported_req_id = 0;
	ctx_isp->bubble_frame_cnt = 0;
	ctx_isp->substate_activated = ctx_isp->rdi_only_context ?
		CAM_ISP_CTX_ACTIVATED_APPLIED :
		(req_isp->num_fence_map_out) ? CAM_ISP_CTX_ACTIVATED_EPOCH :
		CAM_ISP_CTX_ACTIVATED_SOF;

	atomic64_set(&ctx_isp->state_monitor_head, -1);

	for (i = 0; i < CAM_ISP_CTX_EVENT_MAX; i++)
		atomic64_set(&ctx_isp->event_record_head[i], -1);

	/*
	 * In case of CSID TPG we might receive SOF and RUP IRQs
	 * before hw_mgr_intf->hw_start has returned. So move
	 * req out of pending list before hw_start and add it
	 * back to pending list if hw_start fails.
	 */
	list_del_init(&req->list);

	if (ctx_isp->offline_context && !req_isp->num_fence_map_out) {
		list_add_tail(&req->list, &ctx->free_req_list);
		atomic_set(&ctx_isp->rxd_epoch, 1);
		CAM_DBG(CAM_REQ,
			"Move pending req: %lld to free list(cnt: %d) offline ctx %u",
			req->request_id, ctx_isp->active_req_cnt, ctx->ctx_id);
	} else if (ctx_isp->rdi_only_context || !req_isp->num_fence_map_out) {
		list_add_tail(&req->list, &ctx->wait_req_list);
		CAM_DBG(CAM_REQ,
			"Move pending req: %lld to wait list(cnt: %d) ctx %u",
			req->request_id, ctx_isp->active_req_cnt, ctx->ctx_id);
	} else {
		list_add_tail(&req->list, &ctx->active_req_list);
		ctx_isp->active_req_cnt++;
		CAM_DBG(CAM_REQ,
			"Move pending req: %lld to active list(cnt: %d) ctx %u offline %d",
			req->request_id, ctx_isp->active_req_cnt, ctx->ctx_id,
			ctx_isp->offline_context);
	}

	/*
	 * Only place to change state before calling the hw due to
	 * hardware tasklet has higher priority that can cause the
	 * irq handling comes early
	 */
	ctx->state = CAM_CTX_ACTIVATED;
	trace_cam_context_state("ISP", ctx);
	rc = ctx->hw_mgr_intf->hw_start(ctx->hw_mgr_intf->hw_mgr_priv,
		&start_isp);
	if (rc) {
		/* HW failure. user need to clean up the resource */
		CAM_ERR(CAM_ISP, "Start HW failed");
		ctx->state = CAM_CTX_READY;
		if ((rc == -ETIMEDOUT) &&
			(isp_ctx_debug.enable_cdm_cmd_buff_dump))
			rc = cam_isp_ctx_dump_req(req_isp, 0, 0, NULL, false);

		trace_cam_context_state("ISP", ctx);
		list_del_init(&req->list);
		list_add(&req->list, &ctx->pending_req_list);
		goto end;
	}
	CAM_DBG(CAM_ISP, "start device success ctx %u", ctx->ctx_id);

end:
	return rc;
}

static int __cam_isp_ctx_unlink_in_ready(struct cam_context *ctx,
	struct cam_req_mgr_core_dev_link_setup *unlink)
{
	int rc = 0;

	ctx->link_hdl = -1;
	ctx->ctx_crm_intf = NULL;
	ctx->state = CAM_CTX_ACQUIRED;
	trace_cam_context_state("ISP", ctx);

	return rc;
}

static int __cam_isp_ctx_stop_dev_in_activated_unlock(
	struct cam_context *ctx, struct cam_start_stop_dev_cmd *stop_cmd)
{
	int rc = 0;
	uint32_t i;
	struct cam_hw_stop_args          stop;
	struct cam_ctx_request          *req;
	struct cam_isp_ctx_req          *req_isp;
	struct cam_isp_context          *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;
	struct cam_isp_stop_args         stop_isp;

	/* Mask off all the incoming hardware events */
	spin_lock_bh(&ctx->lock);
	ctx_isp->substate_activated = CAM_ISP_CTX_ACTIVATED_HALT;
	spin_unlock_bh(&ctx->lock);

	/* stop hw first */
	if (ctx_isp->hw_ctx) {
		stop.ctxt_to_hw_map = ctx_isp->hw_ctx;

		stop_isp.hw_stop_cmd = CAM_ISP_HW_STOP_IMMEDIATELY;
		stop_isp.stop_only = false;

		stop.args = (void *) &stop_isp;
		ctx->hw_mgr_intf->hw_stop(ctx->hw_mgr_intf->hw_mgr_priv,
			&stop);
	}

	CAM_DBG(CAM_ISP, "next Substate[%s]",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));

	if (ctx->ctx_crm_intf &&
		ctx->ctx_crm_intf->notify_stop) {
		struct cam_req_mgr_notify_stop notify;

		notify.link_hdl = ctx->link_hdl;
		CAM_DBG(CAM_ISP,
			"Notify CRM about device stop ctx %u link 0x%x",
			ctx->ctx_id, ctx->link_hdl);
		ctx->ctx_crm_intf->notify_stop(&notify);
	} else
		CAM_ERR(CAM_ISP, "cb not present");

	while (!list_empty(&ctx->pending_req_list)) {
		req = list_first_entry(&ctx->pending_req_list,
				struct cam_ctx_request, list);
		list_del_init(&req->list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		CAM_DBG(CAM_ISP, "signal fence in pending list. fence num %d",
			 req_isp->num_fence_map_out);
		for (i = 0; i < req_isp->num_fence_map_out; i++)
			if (req_isp->fence_map_out[i].sync_id != -1) {
				cam_sync_signal(
					req_isp->fence_map_out[i].sync_id,
					CAM_SYNC_STATE_SIGNALED_CANCEL,
					CAM_SYNC_ISP_EVENT_HW_STOP);
			}
		list_add_tail(&req->list, &ctx->free_req_list);
	}

	while (!list_empty(&ctx->wait_req_list)) {
		req = list_first_entry(&ctx->wait_req_list,
				struct cam_ctx_request, list);
		list_del_init(&req->list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		CAM_DBG(CAM_ISP, "signal fence in wait list. fence num %d",
			 req_isp->num_fence_map_out);
		for (i = 0; i < req_isp->num_fence_map_out; i++)
			if (req_isp->fence_map_out[i].sync_id != -1) {
				cam_sync_signal(
					req_isp->fence_map_out[i].sync_id,
					CAM_SYNC_STATE_SIGNALED_CANCEL,
					CAM_SYNC_ISP_EVENT_HW_STOP);
			}
		list_add_tail(&req->list, &ctx->free_req_list);
	}

	while (!list_empty(&ctx->active_req_list)) {
		req = list_first_entry(&ctx->active_req_list,
				struct cam_ctx_request, list);
		list_del_init(&req->list);
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		CAM_DBG(CAM_ISP, "signal fence in active list. fence num %d",
			 req_isp->num_fence_map_out);
		for (i = 0; i < req_isp->num_fence_map_out; i++)
			if (req_isp->fence_map_out[i].sync_id != -1) {
				cam_sync_signal(
					req_isp->fence_map_out[i].sync_id,
					CAM_SYNC_STATE_SIGNALED_CANCEL,
					CAM_SYNC_ISP_EVENT_HW_STOP);
			}
		list_add_tail(&req->list, &ctx->free_req_list);
	}

	ctx_isp->frame_id = 0;
	ctx_isp->active_req_cnt = 0;
	ctx_isp->reported_req_id = 0;
	ctx_isp->last_applied_req_id = 0;
	ctx_isp->req_info.last_bufdone_req_id = 0;
	ctx_isp->bubble_frame_cnt = 0;
	atomic_set(&ctx_isp->process_bubble, 0);
	atomic_set(&ctx_isp->rxd_epoch, 0);
	atomic64_set(&ctx_isp->state_monitor_head, -1);

	for (i = 0; i < CAM_ISP_CTX_EVENT_MAX; i++)
		atomic64_set(&ctx_isp->event_record_head[i], -1);

	CAM_DBG(CAM_ISP, "Stop device success next state %d on ctx %u",
		ctx->state, ctx->ctx_id);

	if (!stop_cmd) {
		rc = __cam_isp_ctx_unlink_in_ready(ctx, NULL);
		if (rc)
			CAM_ERR(CAM_ISP, "Unlink failed rc=%d", rc);
	}
	return rc;
}

static int __cam_isp_ctx_stop_dev_in_activated(struct cam_context *ctx,
	struct cam_start_stop_dev_cmd *cmd)
{
	int rc = 0;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *)ctx->ctx_priv;

	__cam_isp_ctx_stop_dev_in_activated_unlock(ctx, cmd);
	ctx_isp->init_received = false;
	ctx->state = CAM_CTX_ACQUIRED;
	trace_cam_context_state("ISP", ctx);
	return rc;
}

static int __cam_isp_ctx_release_dev_in_activated(struct cam_context *ctx,
	struct cam_release_dev_cmd *cmd)
{
	int rc = 0;

	rc = __cam_isp_ctx_stop_dev_in_activated_unlock(ctx, NULL);
	if (rc)
		CAM_ERR(CAM_ISP, "Stop device failed rc=%d", rc);

	rc = __cam_isp_ctx_release_dev_in_top_state(ctx, cmd);
	if (rc)
		CAM_ERR(CAM_ISP, "Release device failed rc=%d", rc);

	return rc;
}

static int __cam_isp_ctx_release_hw_in_activated(struct cam_context *ctx,
	void *cmd)
{
	int rc = 0;

	rc = __cam_isp_ctx_stop_dev_in_activated_unlock(ctx, NULL);
	if (rc)
		CAM_ERR(CAM_ISP, "Stop device failed rc=%d", rc);

	rc = __cam_isp_ctx_release_hw_in_top_state(ctx, cmd);
	if (rc)
		CAM_ERR(CAM_ISP, "Release hw failed rc=%d", rc);

	return rc;
}

static int __cam_isp_ctx_link_pause(struct cam_context *ctx)
{
	int rc = 0;
	struct cam_hw_cmd_args       hw_cmd_args;
	struct cam_isp_hw_cmd_args   isp_hw_cmd_args;
	struct cam_isp_context      *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	hw_cmd_args.ctxt_to_hw_map = ctx_isp->hw_ctx;
	hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
	isp_hw_cmd_args.cmd_type = CAM_ISP_HW_MGR_CMD_PAUSE_HW;
	hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;
	rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
		&hw_cmd_args);

	return rc;
}

static int __cam_isp_ctx_link_resume(struct cam_context *ctx)
{
	int rc = 0;
	struct cam_hw_cmd_args       hw_cmd_args;
	struct cam_isp_hw_cmd_args   isp_hw_cmd_args;
	struct cam_isp_context      *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	hw_cmd_args.ctxt_to_hw_map = ctx_isp->hw_ctx;
	hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
	isp_hw_cmd_args.cmd_type = CAM_ISP_HW_MGR_CMD_RESUME_HW;
	hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;
	rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
		&hw_cmd_args);

	return rc;
}

static int __cam_isp_ctx_handle_sof_freeze_evt(
	struct cam_context *ctx)
{
	int rc = 0;
	struct cam_hw_cmd_args       hw_cmd_args;
	struct cam_isp_hw_cmd_args   isp_hw_cmd_args;
	struct cam_isp_context      *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	hw_cmd_args.ctxt_to_hw_map = ctx_isp->hw_ctx;
	hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
	isp_hw_cmd_args.cmd_type = CAM_ISP_HW_MGR_CMD_SOF_DEBUG;
	isp_hw_cmd_args.u.sof_irq_enable = 1;
	hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;

	rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
		&hw_cmd_args);

	return rc;
}

static int __cam_isp_ctx_process_evt(struct cam_context *ctx,
	struct cam_req_mgr_link_evt_data *link_evt_data)
{
	int rc = 0;

	switch (link_evt_data->evt_type) {
	case CAM_REQ_MGR_LINK_EVT_ERR:
		/* No need to handle this message now */
		break;
	case CAM_REQ_MGR_LINK_EVT_PAUSE:
		__cam_isp_ctx_link_pause(ctx);
		break;
	case CAM_REQ_MGR_LINK_EVT_RESUME:
		__cam_isp_ctx_link_resume(ctx);
		break;
	case CAM_REQ_MGR_LINK_EVT_SOF_FREEZE:
		__cam_isp_ctx_handle_sof_freeze_evt(ctx);
		break;
	default:
		CAM_WARN(CAM_ISP, "Unknown event from CRM");
		break;
	}
	return rc;
}

static int __cam_isp_ctx_unlink_in_activated(struct cam_context *ctx,
	struct cam_req_mgr_core_dev_link_setup *unlink)
{
	int rc = 0;

	CAM_WARN(CAM_ISP,
		"Received unlink in activated state. It's unexpected");

	rc = __cam_isp_ctx_stop_dev_in_activated_unlock(ctx, NULL);
	if (rc)
		CAM_WARN(CAM_ISP, "Stop device failed rc=%d", rc);

	rc = __cam_isp_ctx_unlink_in_ready(ctx, unlink);
	if (rc)
		CAM_ERR(CAM_ISP, "Unlink failed rc=%d", rc);

	return rc;
}

static int __cam_isp_ctx_apply_req(struct cam_context *ctx,
	struct cam_req_mgr_apply_request *apply)
{
	int rc = 0;
	struct cam_ctx_ops *ctx_ops = NULL;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *) ctx->ctx_priv;

	trace_cam_apply_req("ISP", apply->request_id);
	CAM_DBG(CAM_ISP, "Enter: apply req in Substate[%s] request_id:%lld",
		__cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated), apply->request_id);
	ctx_ops = &ctx_isp->substate_machine[ctx_isp->substate_activated];
	if (ctx_ops->crm_ops.apply_req) {
		rc = ctx_ops->crm_ops.apply_req(ctx, apply);
	} else {
		CAM_WARN_RATE_LIMIT(CAM_ISP,
			"No handle function in activated Substate[%s]",
			__cam_isp_ctx_substate_val_to_type(
			ctx_isp->substate_activated));
		rc = -EFAULT;
	}

	if (rc)
		CAM_WARN_RATE_LIMIT(CAM_ISP,
			"Apply failed in active Substate[%s] rc %d",
			__cam_isp_ctx_substate_val_to_type(
			ctx_isp->substate_activated), rc);
	return rc;
}



static int __cam_isp_ctx_handle_irq_in_activated(void *context,
	uint32_t evt_id, void *evt_data)
{
	int rc = 0;
	struct cam_isp_ctx_irq_ops *irq_ops = NULL;
	struct cam_context *ctx = (struct cam_context *)context;
	struct cam_isp_context *ctx_isp =
		(struct cam_isp_context *)ctx->ctx_priv;

	spin_lock(&ctx->lock);

	trace_cam_isp_activated_irq(ctx, ctx_isp->substate_activated, evt_id,
		__cam_isp_ctx_get_event_ts(evt_id, evt_data));

	CAM_DBG(CAM_ISP, "Enter: State %d, Substate[%s], evt id %d",
		ctx->state, __cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated), evt_id);
	irq_ops = &ctx_isp->substate_machine_irq[ctx_isp->substate_activated];
	if (irq_ops->irq_ops[evt_id]) {
		rc = irq_ops->irq_ops[evt_id](ctx_isp, evt_data);
	} else {
		CAM_DBG(CAM_ISP, "No handle function for Substate[%s]",
			__cam_isp_ctx_substate_val_to_type(
			ctx_isp->substate_activated));
		if (isp_ctx_debug.enable_state_monitor_dump)
			__cam_isp_ctx_dump_state_monitor_array(ctx_isp);
	}

	CAM_DBG(CAM_ISP, "Exit: State %d Substate[%s]",
		ctx->state, __cam_isp_ctx_substate_val_to_type(
		ctx_isp->substate_activated));
	spin_unlock(&ctx->lock);
	return rc;
}

static int __cam_isp_shutdown_dev(
	struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return cam_isp_subdev_close_internal(sd, fh);
}

/* top state machine */
static struct cam_ctx_ops
	cam_isp_ctx_top_state_machine[CAM_CTX_STATE_MAX] = {
	/* Uninit */
	{
		.ioctl_ops = {},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* Available */
	{
		.ioctl_ops = {
			.acquire_dev = __cam_isp_ctx_acquire_dev_in_available,
			.shutdown_dev = __cam_isp_shutdown_dev,
		},
		.crm_ops = {},
		.irq_ops = NULL,
	},
	/* Acquired */
	{
		.ioctl_ops = {
			.acquire_hw = __cam_isp_ctx_acquire_hw_in_acquired,
			.release_dev = __cam_isp_ctx_release_dev_in_top_state,
			.config_dev = __cam_isp_ctx_config_dev_in_acquired,
			.release_hw = __cam_isp_ctx_release_hw_in_top_state,
			.shutdown_dev = __cam_isp_shutdown_dev,
		},
		.crm_ops = {
			.link = __cam_isp_ctx_link_in_acquired,
			.unlink = __cam_isp_ctx_unlink_in_acquired,
			.get_dev_info = __cam_isp_ctx_get_dev_info_in_acquired,
			.flush_req = __cam_isp_ctx_flush_req_in_top_state,
			.dump_req = __cam_isp_ctx_dump_in_top_state,
		},
		.irq_ops = NULL,
		.pagefault_ops = cam_isp_context_dump_requests,
		.dumpinfo_ops = cam_isp_context_info_dump,
	},
	/* Ready */
	{
		.ioctl_ops = {
			.start_dev = __cam_isp_ctx_start_dev_in_ready,
			.release_dev = __cam_isp_ctx_release_dev_in_top_state,
			.config_dev = __cam_isp_ctx_config_dev_in_top_state,
			.release_hw = __cam_isp_ctx_release_hw_in_top_state,
			.shutdown_dev = __cam_isp_shutdown_dev,
		},
		.crm_ops = {
			.unlink = __cam_isp_ctx_unlink_in_ready,
			.flush_req = __cam_isp_ctx_flush_req_in_ready,
			.dump_req = __cam_isp_ctx_dump_in_top_state,
		},
		.irq_ops = NULL,
		.pagefault_ops = cam_isp_context_dump_requests,
		.dumpinfo_ops = cam_isp_context_info_dump,
	},
	/* Flushed */
	{
		.ioctl_ops = {
			.stop_dev = __cam_isp_ctx_stop_dev_in_activated,
			.release_dev = __cam_isp_ctx_release_dev_in_activated,
			.config_dev = __cam_isp_ctx_config_dev_in_flushed,
			.release_hw = __cam_isp_ctx_release_hw_in_activated,
			.shutdown_dev = __cam_isp_shutdown_dev,
		},
		.crm_ops = {
			.unlink = __cam_isp_ctx_unlink_in_ready,
			.process_evt = __cam_isp_ctx_process_evt,
			.flush_req = __cam_isp_ctx_flush_req_in_flushed_state,
		},
		.irq_ops = NULL,
		.pagefault_ops = cam_isp_context_dump_requests,
		.dumpinfo_ops = cam_isp_context_info_dump,
		.msg_cb_ops = cam_isp_context_handle_message,
	},
	/* Activated */
	{
		.ioctl_ops = {
			.stop_dev = __cam_isp_ctx_stop_dev_in_activated,
			.release_dev = __cam_isp_ctx_release_dev_in_activated,
			.config_dev = __cam_isp_ctx_config_dev_in_top_state,
			.release_hw = __cam_isp_ctx_release_hw_in_activated,
			.shutdown_dev = __cam_isp_shutdown_dev,
		},
		.crm_ops = {
			.unlink = __cam_isp_ctx_unlink_in_activated,
			.apply_req = __cam_isp_ctx_apply_req,
			.flush_req = __cam_isp_ctx_flush_req_in_top_state,
			.process_evt = __cam_isp_ctx_process_evt,
			.dump_req = __cam_isp_ctx_dump_in_top_state,
		},
		.irq_ops = __cam_isp_ctx_handle_irq_in_activated,
		.pagefault_ops = cam_isp_context_dump_requests,
		.dumpinfo_ops = cam_isp_context_info_dump,
		.msg_cb_ops = cam_isp_context_handle_message,
	},
};

static int cam_isp_context_dump_requests(void *data,
	struct cam_smmu_pf_info *pf_info)
{

	struct cam_context *ctx = (struct cam_context *)data;
	struct cam_ctx_request *req = NULL;
	struct cam_ctx_request *req_temp = NULL;
	struct cam_isp_ctx_req *req_isp  = NULL;
	struct cam_isp_prepare_hw_update_data *hw_update_data = NULL;
	struct cam_hw_mgr_dump_pf_data *pf_dbg_entry = NULL;
	struct cam_req_mgr_message       req_msg;
	struct cam_isp_context          *ctx_isp;
	uint32_t  resource_type = 0;
	bool mem_found = false, ctx_found = false, send_error = false;
	int rc = 0;

	struct cam_isp_context *isp_ctx =
		(struct cam_isp_context *)ctx->ctx_priv;

	if (!isp_ctx) {
		CAM_ERR(CAM_ISP, "Invalid isp ctx");
		return -EINVAL;
	}

	CAM_INFO(CAM_ISP, "iommu fault handler for isp ctx %d state %d",
		ctx->ctx_id, ctx->state);

	list_for_each_entry_safe(req, req_temp,
		&ctx->active_req_list, list) {
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		hw_update_data = &req_isp->hw_update_data;
		pf_dbg_entry = &(req->pf_data);
		CAM_INFO(CAM_ISP, "Active List: req_id : %lld ",
			req->request_id);

		rc = cam_context_dump_pf_info_to_hw(ctx, pf_dbg_entry,
			&mem_found, &ctx_found, &resource_type, pf_info);
		if (rc)
			CAM_ERR(CAM_ISP, "Failed to dump pf info");

		if (ctx_found)
			send_error = true;
	}

	CAM_INFO(CAM_ISP, "Iterating over wait_list of isp ctx %d state %d",
			ctx->ctx_id, ctx->state);

	list_for_each_entry_safe(req, req_temp,
		&ctx->wait_req_list, list) {
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		hw_update_data = &req_isp->hw_update_data;
		pf_dbg_entry = &(req->pf_data);
		CAM_INFO(CAM_ISP, "Wait List: req_id : %lld ", req->request_id);

		rc = cam_context_dump_pf_info_to_hw(ctx, pf_dbg_entry,
			&mem_found, &ctx_found, &resource_type, pf_info);
		if (rc)
			CAM_ERR(CAM_ISP, "Failed to dump pf info");

		if (ctx_found)
			send_error = true;
	}

	/*
	 * In certain scenarios we observe both overflow and SMMU pagefault
	 * for a particular request. If overflow is handled before page fault
	 * we need to traverse through pending request list because if
	 * bubble recovery is enabled on any request we move that request
	 * and all the subsequent requests to the pending list while handling
	 * overflow error.
	 */

	CAM_INFO(CAM_ISP,
		"Iterating over pending req list of isp ctx %d state %d",
		ctx->ctx_id, ctx->state);

	list_for_each_entry_safe(req, req_temp,
		&ctx->pending_req_list, list) {
		req_isp = (struct cam_isp_ctx_req *) req->req_priv;
		hw_update_data = &req_isp->hw_update_data;
		pf_dbg_entry = &(req->pf_data);
		CAM_INFO(CAM_ISP, "Pending List: req_id : %lld ",
			req->request_id);

		rc = cam_context_dump_pf_info_to_hw(ctx, pf_dbg_entry,
			&mem_found, &ctx_found, &resource_type, pf_info);
		if (rc)
			CAM_ERR(CAM_ISP, "Failed to dump pf info");

		if (ctx_found)
			send_error = true;
	}

	if (resource_type) {
		ctx_isp = (struct cam_isp_context *) ctx->ctx_priv;
		if (ctx_isp->isp_device_type == CAM_IFE_DEVICE_TYPE)
			CAM_ERR(CAM_ISP,
				"Page fault on resource id:%s (0x%x) ctx id:%d frame id:%d reported id:%lld applied id:%lld",
				__cam_isp_resource_handle_id_to_type(
				resource_type),
				resource_type, ctx->ctx_id, ctx_isp->frame_id,
				ctx_isp->reported_req_id,
				ctx_isp->last_applied_req_id);
		else
			CAM_ERR(CAM_ISP,
				"Page fault on resource id:%s (0x%x) ctx id:%d frame id:%d reported id:%lld applied id:%lld",
				__cam_isp_tfe_resource_handle_id_to_type(
				resource_type),
				resource_type, ctx->ctx_id, ctx_isp->frame_id,
				ctx_isp->reported_req_id,
				ctx_isp->last_applied_req_id);

	}

	if (send_error) {
		CAM_INFO(CAM_ISP,
			"page fault notifying to umd ctx %u session_hdl:%d device_hdl:%d link_hdl:%d",
			ctx->ctx_id, ctx->session_hdl,
			ctx->dev_hdl, ctx->link_hdl);

		req_msg.session_hdl = ctx->session_hdl;
		req_msg.u.err_msg.device_hdl = ctx->dev_hdl;
		req_msg.u.err_msg.error_type =
			CAM_REQ_MGR_ERROR_TYPE_PAGE_FAULT;
		req_msg.u.err_msg.link_hdl = ctx->link_hdl;
		req_msg.u.err_msg.request_id = 0;
		req_msg.u.err_msg.resource_size = 0x0;

		if (cam_req_mgr_notify_message(&req_msg,
				V4L_EVENT_CAM_REQ_MGR_ERROR,
				V4L_EVENT_CAM_REQ_MGR_EVENT))
			CAM_ERR(CAM_ISP,
				"could not send page fault notification ctx %u session_hdl:%d device_hdl:%d link_hdl:%d",
				ctx->ctx_id, ctx->session_hdl,
				ctx->dev_hdl, ctx->link_hdl);
	}
	return rc;
}

static int cam_isp_context_handle_message(void *context,
	uint32_t msg_type, uint32_t *data)
{
	int                            rc = -EINVAL;
	struct cam_hw_cmd_args         hw_cmd_args;
	struct cam_isp_hw_cmd_args     isp_hw_cmd_args;
	struct cam_context            *ctx = (struct cam_context *)context;

	memset(&hw_cmd_args, 0, sizeof(hw_cmd_args));
	hw_cmd_args.ctxt_to_hw_map = ctx->ctxt_to_hw_map;

	switch (msg_type) {
	case CAM_SUBDEV_MESSAGE_CLOCK_UPDATE:
		hw_cmd_args.cmd_type = CAM_HW_MGR_CMD_INTERNAL;
		isp_hw_cmd_args.cmd_type = CAM_ISP_HW_MGR_CMD_UPDATE_CLOCK;
		isp_hw_cmd_args.cmd_data = (void *)data;
		hw_cmd_args.u.internal_args = (void *)&isp_hw_cmd_args;
		rc = ctx->hw_mgr_intf->hw_cmd(ctx->hw_mgr_intf->hw_mgr_priv,
			&hw_cmd_args);
		if (rc)
			CAM_ERR(CAM_ISP, "Update clock rate failed rc: %d", rc);
		break;
	default:
		CAM_ERR(CAM_ISP, "Invalid message type %d", msg_type);
	}
	return rc;
}

static int cam_isp_context_debug_register(void)
{
	int rc = 0;
	struct dentry *dbgfileptr = NULL;

	dbgfileptr = debugfs_create_dir("camera_isp_ctx", NULL);
	if (!dbgfileptr) {
		CAM_ERR(CAM_ISP, "DebugFS could not create directory!");
		rc = -ENOENT;
		goto end;
	}
	/* Store parent inode for cleanup in caller */
	isp_ctx_debug.dentry = dbgfileptr;

	dbgfileptr = debugfs_create_u32("enable_state_monitor_dump", 0644,
		isp_ctx_debug.dentry, &isp_ctx_debug.enable_state_monitor_dump);
	dbgfileptr = debugfs_create_u8("enable_cdm_cmd_buffer_dump", 0644,
		isp_ctx_debug.dentry, &isp_ctx_debug.enable_cdm_cmd_buff_dump);
	if (IS_ERR(dbgfileptr)) {
		if (PTR_ERR(dbgfileptr) == -ENODEV)
			CAM_WARN(CAM_ISP, "DebugFS not enabled in kernel!");
		else
			rc = PTR_ERR(dbgfileptr);
	}
end:
	return rc;
}

int cam_isp_context_init(struct cam_isp_context *ctx,
	struct cam_context *ctx_base,
	struct cam_req_mgr_kmd_ops *crm_node_intf,
	struct cam_hw_mgr_intf *hw_intf,
	uint32_t ctx_id,
	uint32_t isp_device_type)

{
	int rc = -1;
	int i;

	if (!ctx || !ctx_base) {
		CAM_ERR(CAM_ISP, "Invalid Context");
		goto err;
	}

	/* ISP context setup */
	memset(ctx, 0, sizeof(*ctx));

	ctx->base = ctx_base;
	ctx->frame_id = 0;
	ctx->custom_enabled = false;
	ctx->use_frame_header_ts = false;
	ctx->active_req_cnt = 0;
	ctx->reported_req_id = 0;
	ctx->bubble_frame_cnt = 0;
	ctx->req_info.last_bufdone_req_id = 0;

	ctx->hw_ctx = NULL;
	ctx->substate_activated = CAM_ISP_CTX_ACTIVATED_SOF;
	ctx->substate_machine = cam_isp_ctx_activated_state_machine;
	ctx->substate_machine_irq = cam_isp_ctx_activated_state_machine_irq;
	ctx->init_timestamp = jiffies_to_msecs(jiffies);
	ctx->isp_device_type = isp_device_type;

	for (i = 0; i < CAM_ISP_CTX_REQ_MAX; i++) {
		ctx->req_base[i].req_priv = &ctx->req_isp[i];
		ctx->req_isp[i].base = &ctx->req_base[i];
	}

	/* camera context setup */
	rc = cam_context_init(ctx_base, isp_dev_name, CAM_ISP, ctx_id,
		crm_node_intf, hw_intf, ctx->req_base, CAM_ISP_CTX_REQ_MAX);
	if (rc) {
		CAM_ERR(CAM_ISP, "Camera Context Base init failed");
		goto err;
	}

	/* link camera context with isp context */
	ctx_base->state_machine = cam_isp_ctx_top_state_machine;
	ctx_base->ctx_priv = ctx;

	/* initializing current state for error logging */
	for (i = 0; i < CAM_ISP_CTX_STATE_MONITOR_MAX_ENTRIES; i++) {
		ctx->cam_isp_ctx_state_monitor[i].curr_state =
		CAM_ISP_CTX_ACTIVATED_MAX;
	}
	atomic64_set(&ctx->state_monitor_head, -1);

	for (i = 0; i < CAM_ISP_CTX_EVENT_MAX; i++)
		atomic64_set(&ctx->event_record_head[i], -1);

	if (!isp_ctx_debug.dentry)
		cam_isp_context_debug_register();

err:
	return rc;
}

int cam_isp_context_deinit(struct cam_isp_context *ctx)
{
	if (ctx->base)
		cam_context_deinit(ctx->base);

	if (ctx->substate_activated != CAM_ISP_CTX_ACTIVATED_SOF)
		CAM_ERR(CAM_ISP, "ISP context Substate[%s] is invalid",
			__cam_isp_ctx_substate_val_to_type(
			ctx->substate_activated));

	debugfs_remove_recursive(isp_ctx_debug.dentry);
	isp_ctx_debug.dentry = NULL;
	memset(ctx, 0, sizeof(*ctx));

	return 0;
}
