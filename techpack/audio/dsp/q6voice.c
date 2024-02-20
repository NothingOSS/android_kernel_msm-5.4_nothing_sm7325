// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/mutex.h>

#include <soc/qcom/socinfo.h>

#include <dsp/msm_audio_ion.h>
#include <dsp/q6audio-v2.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/q6afe-v2.h>
#include <dsp/q6common.h>
#include <dsp/audio_cal_utils.h>
#include <dsp/q6core.h>
#include <dsp/q6voice.h>
#include <ipc/apr_tal.h>
#include "adsp_err.h"
#include <dsp/voice_mhi.h>
#include <soc/qcom/secure_buffer.h>

#define TIMEOUT_MS 1000


#define CMD_STATUS_SUCCESS 0
#define CMD_STATUS_FAIL 1
#define NUM_CHANNELS_MONO 1
#define NUM_CHANNELS_STEREO 2
#define NUM_CHANNELS_THREE 3
#define NUM_CHANNELS_QUAD 4
#define CVP_VERSION_2 2
#define GAIN_Q14_FORMAT(a) (a << 14)

enum {
	VOC_TOKEN_NONE,
	VOIP_MEM_MAP_TOKEN,
	VOC_CAL_MEM_MAP_TOKEN,
	VOC_VOICE_HOST_PCM_MAP_TOKEN,
	VOC_RTAC_MEM_MAP_TOKEN,
	VOC_SOURCE_TRACKING_MEM_MAP_TOKEN
};

struct cvd_version_table cvd_version_table_mapping[CVD_INT_VERSION_MAX] = {
		{CVD_VERSION_DEFAULT, CVD_INT_VERSION_DEFAULT},
		{CVD_VERSION_0_0, CVD_INT_VERSION_0_0},
		{CVD_VERSION_2_1, CVD_INT_VERSION_2_1},
		{CVD_VERSION_2_2, CVD_INT_VERSION_2_2},
		{CVD_VERSION_2_3, CVD_INT_VERSION_2_3},
		{CVD_VERSION_2_4, CVD_INT_VERSION_2_4},
};

static struct common_data common;
static bool module_initialized;
static bool hyp_assigned;

static int voice_send_enable_vocproc_cmd(struct voice_data *v);
static int voice_send_netid_timing_cmd(struct voice_data *v);
static int voice_send_attach_vocproc_cmd(struct voice_data *v);
static int voice_send_set_device_cmd(struct voice_data *v);
static int voice_send_vol_step_cmd(struct voice_data *v);
static int voice_send_mvm_unmap_memory_physical_cmd(struct voice_data *v,
						    uint32_t mem_handle);
static int voice_send_mvm_cal_network_cmd(struct voice_data *v);
static int voice_send_mvm_media_type_cmd(struct voice_data *v);
static int voice_send_mvm_cvd_version_cmd(struct voice_data *v);
static int voice_send_mvm_event_class_cmd(struct voice_data *v,
					   uint32_t event_id,
					   uint32_t class_id);
static int voice_send_cvs_data_exchange_mode_cmd(struct voice_data *v);
static int voice_send_cvs_packet_exchange_config_cmd(struct voice_data *v);
static int voice_set_packet_exchange_mode_and_config(uint32_t session_id,
						     uint32_t mode);

static int voice_send_cvs_register_cal_cmd(struct voice_data *v);
static int voice_send_cvs_deregister_cal_cmd(struct voice_data *v);
static int voice_send_cvp_create_cmd(struct voice_data *v);
static int voice_send_cvp_register_dev_cfg_cmd(struct voice_data *v);
static int voice_send_cvp_deregister_dev_cfg_cmd(struct voice_data *v);
static int voice_send_cvp_register_cal_cmd(struct voice_data *v);
static int voice_send_cvp_deregister_cal_cmd(struct voice_data *v);
static int voice_send_cvp_register_vol_cal_cmd(struct voice_data *v);
static int voice_send_cvp_deregister_vol_cal_cmd(struct voice_data *v);
static int voice_send_cvp_media_fmt_info_cmd(struct voice_data *v);
static int voice_send_cvp_device_channels_cmd(struct voice_data *v);
static int voice_send_cvp_media_format_cmd(struct voice_data *v,
					   uint32_t param_type);
static int voice_send_cvp_topology_commit_cmd(struct voice_data *v);
static int voice_send_cvp_channel_info_cmd(struct voice_data *v);
static int voice_send_cvp_channel_info_v2(struct voice_data *v,
					  uint32_t param_type);
static int voice_get_avcs_version_per_service(uint32_t service_id);

static void voice_load_topo_modules(int cal_index);
static void voice_unload_topo_modules(void);

static int voice_cvs_stop_playback(struct voice_data *v);
static int voice_cvs_start_playback(struct voice_data *v);
static int voice_cvs_start_record(struct voice_data *v, uint32_t rec_mode,
					uint32_t port_id);
static int voice_cvs_stop_record(struct voice_data *v);

static int32_t qdsp_mvm_callback(struct apr_client_data *data, void *priv);
static int32_t qdsp_cvs_callback(struct apr_client_data *data, void *priv);
static int32_t qdsp_cvp_callback(struct apr_client_data *data, void *priv);

static int voice_send_set_pp_enable_cmd(
	struct voice_data *v, struct module_instance_info mod_inst_info,
	int enable);

static int voice_send_cvp_ecns_enable_cmd(struct voice_data *v,
	uint32_t module_id, int enable);

static int is_cal_memory_allocated(void);
static bool is_cvd_version_queried(void);
static bool is_voip_memory_allocated(void);
static int voice_get_cvd_int_version(char *cvd_ver_string);
static int voice_alloc_cal_mem_map_table(void);
static int voice_alloc_rtac_mem_map_table(void);
static int voice_alloc_oob_shared_mem(void);
static int voice_free_oob_shared_mem(void);
static int voice_alloc_oob_mem_table(void);
static int voice_alloc_and_map_oob_mem(struct voice_data *v);

static struct voice_data *voice_get_session_by_idx(int idx);

static int remap_cal_data(struct cal_block_data *cal_block,
			  uint32_t session_id);
static int voice_unmap_cal_memory(int32_t cal_type,
				  struct cal_block_data *cal_block);

static bool is_source_tracking_shared_memomry_allocated(void);
static int voice_alloc_source_tracking_shared_memory(void);
static int voice_alloc_and_map_source_tracking_shared_memory(
						struct voice_data *v);
static int voice_unmap_and_free_source_tracking_shared_memory(
						struct voice_data *v);
static int voice_send_set_sound_focus_cmd(struct voice_data *v,
				struct sound_focus_param soundFocusData);
static int voice_send_get_sound_focus_cmd(struct voice_data *v,
				struct sound_focus_param *soundFocusData);
static int voice_send_get_source_tracking_cmd(struct voice_data *v,
			struct source_tracking_param *sourceTrackingData);
static int voice_send_get_fnn_source_track_cmd(struct voice_data *v,
			struct fluence_nn_source_tracking_param *FnnSourceTrackingData);
static int voice_pack_and_set_cvp_param(struct voice_data *v,
					struct param_hdr_v3 param_hdr,
					u8 *param_data);
static int voice_pack_and_set_cvs_ui_property(struct voice_data *v,
					      struct param_hdr_v3 param_hdr,
					      u8 *param_data);

static void voice_itr_init(struct voice_session_itr *itr,
			   u32 session_id)
{
	if (itr == NULL)
		return;
	itr->session_idx = voice_get_idx_for_session(session_id);
	if (session_id == ALL_SESSION_VSID)
		itr->cur_idx = 0;
	else
		itr->cur_idx = itr->session_idx;

}

static bool voice_itr_get_next_session(struct voice_session_itr *itr,
					struct voice_data **voice)
{
	bool ret = false;

	if (itr == NULL)
		return false;
	pr_debug("%s : cur idx = %d session idx = %d\n",
			 __func__, itr->cur_idx, itr->session_idx);

	if (itr->cur_idx <= itr->session_idx) {
		ret = true;
		*voice = voice_get_session_by_idx(itr->cur_idx);
		itr->cur_idx++;
	} else {
		*voice = NULL;
	}

	return ret;
}

static bool voice_is_valid_session_id(uint32_t session_id)
{
	bool ret = false;

	switch (session_id) {
	case VOICE_SESSION_VSID:
	case VOICE2_SESSION_VSID:
	case VOLTE_SESSION_VSID:
	case VOIP_SESSION_VSID:
	case QCHAT_SESSION_VSID:
	case VOWLAN_SESSION_VSID:
	case VOICEMMODE1_VSID:
	case VOICEMMODE2_VSID:
	case ALL_SESSION_VSID:
		ret = true;
		break;
	default:
		pr_err("%s: Invalid session_id : %x\n", __func__, session_id);

		break;
	}

	return ret;
}

static u16 voice_get_mvm_handle(struct voice_data *v)
{
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return 0;
	}

	pr_debug("%s: mvm_handle %d\n", __func__, v->mvm_handle);

	return v->mvm_handle;
}

static void voice_set_mvm_handle(struct voice_data *v, u16 mvm_handle)
{
	pr_debug("%s: mvm_handle %d\n", __func__, mvm_handle);
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return;
	}

	v->mvm_handle = mvm_handle;
}

static u16 voice_get_cvs_handle(struct voice_data *v)
{
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return 0;
	}

	pr_debug("%s: cvs_handle %d\n", __func__, v->cvs_handle);

	return v->cvs_handle;
}

static void voice_set_cvs_handle(struct voice_data *v, u16 cvs_handle)
{
	pr_debug("%s: cvs_handle %d\n", __func__, cvs_handle);
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return;
	}

	v->cvs_handle = cvs_handle;
}

static u16 voice_get_cvp_handle(struct voice_data *v)
{
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return 0;
	}

	pr_debug("%s: cvp_handle %d\n", __func__, v->cvp_handle);

	return v->cvp_handle;
}

static void voice_set_cvp_handle(struct voice_data *v, u16 cvp_handle)
{
	pr_debug("%s: cvp_handle %d\n", __func__, cvp_handle);
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return;
	}

	v->cvp_handle = cvp_handle;
}

char *voc_get_session_name(u32 session_id)
{
	char *session_name = NULL;

	if (session_id == common.voice[VOC_PATH_PASSIVE].session_id) {
		session_name = VOICE_SESSION_NAME;
	} else if (session_id ==
			common.voice[VOC_PATH_VOLTE_PASSIVE].session_id) {
		session_name = VOLTE_SESSION_NAME;
	} else if (session_id ==
			common.voice[VOC_PATH_QCHAT_PASSIVE].session_id) {
		session_name = QCHAT_SESSION_NAME;
	} else if (session_id ==
			common.voice[VOC_PATH_VOWLAN_PASSIVE].session_id) {
		session_name = VOWLAN_SESSION_NAME;
	} else if (session_id ==
		common.voice[VOC_PATH_VOICEMMODE1_PASSIVE].session_id) {
		session_name = VOICEMMODE1_NAME;
	} else if (session_id ==
		common.voice[VOC_PATH_VOICEMMODE2_PASSIVE].session_id) {
		session_name = VOICEMMODE2_NAME;
	} else if (session_id == common.voice[VOC_PATH_FULL].session_id) {
		session_name = VOIP_SESSION_NAME;
	}
	return session_name;
}

/**
 * voc_get_session_id -
 *      Get session ID of given voice session name
 *
 * @name: voice session name
 *
 * Returns session id for valid session or 0 if invalid.
 */
uint32_t voc_get_session_id(char *name)
{
	u32 session_id = 0;

	if (name != NULL) {
		if (!strcmp(name, "Voice session"))
			session_id = common.voice[VOC_PATH_PASSIVE].session_id;
		else if (!strcmp(name, "Voice2 session"))
			session_id =
			common.voice[VOC_PATH_VOICE2_PASSIVE].session_id;
		else if (!strcmp(name, "VoLTE session"))
			session_id =
			common.voice[VOC_PATH_VOLTE_PASSIVE].session_id;
		else if (!strcmp(name, "QCHAT session"))
			session_id =
			common.voice[VOC_PATH_QCHAT_PASSIVE].session_id;
		else if (!strcmp(name, "VoWLAN session"))
			session_id =
			common.voice[VOC_PATH_VOWLAN_PASSIVE].session_id;
		else if (!strcmp(name, "VoiceMMode1"))
			session_id =
			common.voice[VOC_PATH_VOICEMMODE1_PASSIVE].session_id;
		else if (!strcmp(name, "VoiceMMode2"))
			session_id =
			common.voice[VOC_PATH_VOICEMMODE2_PASSIVE].session_id;
		else
			session_id = common.voice[VOC_PATH_FULL].session_id;

		pr_debug("%s: %s has session id 0x%x\n", __func__, name,
				session_id);
	}

	return session_id;
}
EXPORT_SYMBOL(voc_get_session_id);

static struct voice_data *voice_get_session(u32 session_id)
{
	struct voice_data *v = NULL;

	switch (session_id) {
	case VOICE_SESSION_VSID:
		v = &common.voice[VOC_PATH_PASSIVE];
		break;

	case VOICE2_SESSION_VSID:
		v = &common.voice[VOC_PATH_VOICE2_PASSIVE];
		break;

	case VOLTE_SESSION_VSID:
		v = &common.voice[VOC_PATH_VOLTE_PASSIVE];
		break;

	case VOIP_SESSION_VSID:
		v = &common.voice[VOC_PATH_FULL];
		break;

	case QCHAT_SESSION_VSID:
		v = &common.voice[VOC_PATH_QCHAT_PASSIVE];
		break;

	case VOWLAN_SESSION_VSID:
		v = &common.voice[VOC_PATH_VOWLAN_PASSIVE];
		break;

	case VOICEMMODE1_VSID:
		v = &common.voice[VOC_PATH_VOICEMMODE1_PASSIVE];
		break;

	case VOICEMMODE2_VSID:
		v = &common.voice[VOC_PATH_VOICEMMODE2_PASSIVE];
		break;

	case ALL_SESSION_VSID:
		break;

	default:
		pr_err("%s: Invalid session_id : %x\n", __func__, session_id);

		break;
	}

	pr_debug("%s:session_id 0x%x session handle %pK\n",
		__func__, session_id, v);

	return v;
}

int voice_get_idx_for_session(u32 session_id)
{
	int idx = 0;

	switch (session_id) {
	case VOICE_SESSION_VSID:
		idx = VOC_PATH_PASSIVE;
		break;

	case VOICE2_SESSION_VSID:
		idx = VOC_PATH_VOICE2_PASSIVE;
		break;

	case VOLTE_SESSION_VSID:
		idx = VOC_PATH_VOLTE_PASSIVE;
		break;

	case VOIP_SESSION_VSID:
		idx = VOC_PATH_FULL;
		break;

	case QCHAT_SESSION_VSID:
		idx = VOC_PATH_QCHAT_PASSIVE;
		break;

	case VOWLAN_SESSION_VSID:
		idx = VOC_PATH_VOWLAN_PASSIVE;
		break;

	case VOICEMMODE1_VSID:
		idx = VOC_PATH_VOICEMMODE1_PASSIVE;
		break;

	case VOICEMMODE2_VSID:
		idx = VOC_PATH_VOICEMMODE2_PASSIVE;
		break;

	case ALL_SESSION_VSID:
		idx = MAX_VOC_SESSIONS - 1;
		break;

	default:
		pr_err("%s: Invalid session_id : %x\n", __func__, session_id);

		break;
	}

	return idx;
}

static struct voice_data *voice_get_session_by_idx(int idx)
{
	return ((idx < 0 || idx >= MAX_VOC_SESSIONS) ?
				NULL : &common.voice[idx]);
}

static bool is_voip_session(u32 session_id)
{
	return (session_id == common.voice[VOC_PATH_FULL].session_id);
}

static bool is_volte_session(u32 session_id)
{
	return (session_id == common.voice[VOC_PATH_VOLTE_PASSIVE].session_id);
}

static bool is_voice2_session(u32 session_id)
{
	return (session_id == common.voice[VOC_PATH_VOICE2_PASSIVE].session_id);
}

static bool is_qchat_session(u32 session_id)
{
	return (session_id == common.voice[VOC_PATH_QCHAT_PASSIVE].session_id);
}

static bool is_vowlan_session(u32 session_id)
{
	return (session_id == common.voice[VOC_PATH_VOWLAN_PASSIVE].session_id);
}

static bool is_voicemmode1(u32 session_id)
{
	return session_id ==
			common.voice[VOC_PATH_VOICEMMODE1_PASSIVE].session_id;
}

static bool is_voicemmode2(u32 session_id)
{
	return session_id ==
			common.voice[VOC_PATH_VOICEMMODE2_PASSIVE].session_id;
}

static bool is_voc_state_active(int voc_state)
{
	if ((voc_state == VOC_RUN) ||
		(voc_state == VOC_CHANGE) ||
		(voc_state == VOC_STANDBY))
		return true;

	return false;
}

static void voc_set_error_state(uint16_t reset_proc)
{
	struct voice_data *v = NULL;
	int i;

	for (i = 0; i < MAX_VOC_SESSIONS; i++) {
		v = &common.voice[i];
		if (v != NULL) {
			v->voc_state = VOC_ERROR;
			v->rec_info.recording = 0;
			v->music_info.playing = 0;
			v->music_info.force = 0;
		}
	}
}

static bool is_other_session_active(u32 session_id)
{
	int i;
	bool ret = false;

	/* Check if there is other active session except the input one */
	for (i = 0; i < MAX_VOC_SESSIONS; i++) {
		if (common.voice[i].session_id == session_id)
			continue;

		if (is_voc_state_active(common.voice[i].voc_state)) {
			ret = true;
			break;
		}
	}
	pr_debug("%s: ret %d\n", __func__, ret);

	return ret;
}

static bool is_sub1_vsid(u32 session_id)
{
	bool ret;

	switch (session_id) {
	case VOICE_SESSION_VSID:
	case VOLTE_SESSION_VSID:
	case VOWLAN_SESSION_VSID:
	case VOICEMMODE1_VSID:
		ret = true;
		break;
	default:
		ret = false;
	}

	return ret;
}

static bool is_sub2_vsid(u32 session_id)
{
	bool ret;

	switch (session_id) {
	case VOICE2_SESSION_VSID:
	case VOICEMMODE2_VSID:
		ret = true;
		break;
	default:
		ret = false;
	}

	return ret;
}

static bool is_voice_app_id(u32 session_id)
{
	return is_sub1_vsid(session_id) || is_sub2_vsid(session_id);
}

static void init_session_id(void)
{
	common.voice[VOC_PATH_PASSIVE].session_id = VOICE_SESSION_VSID;
	common.voice[VOC_PATH_VOLTE_PASSIVE].session_id = VOLTE_SESSION_VSID;
	common.voice[VOC_PATH_VOICE2_PASSIVE].session_id = VOICE2_SESSION_VSID;
	common.voice[VOC_PATH_FULL].session_id = VOIP_SESSION_VSID;
	common.voice[VOC_PATH_QCHAT_PASSIVE].session_id = QCHAT_SESSION_VSID;
	common.voice[VOC_PATH_VOWLAN_PASSIVE].session_id = VOWLAN_SESSION_VSID;
	common.voice[VOC_PATH_VOICEMMODE1_PASSIVE].session_id =
							VOICEMMODE1_VSID;
	common.voice[VOC_PATH_VOICEMMODE2_PASSIVE].session_id =
							VOICEMMODE2_VSID;
}

static bool is_cvd_version_queried(void)
{
	bool ret = 0;

	if (!strcmp(common.cvd_version, CVD_VERSION_DEFAULT))
		ret = false;
	else
		ret = true;

	return ret;
}

static int voice_get_cvd_int_version(char *cvd_ver_string)
{
	unsigned int idx;
	int cvd_int_ver = CVD_INT_VERSION_DEFAULT;

	for (idx = 0; idx < CVD_INT_VERSION_MAX; idx++) {
		if (strcmp((char *)cvd_ver_string,
			  cvd_version_table_mapping[idx].cvd_ver) == 0) {
			cvd_int_ver =
			cvd_version_table_mapping[idx].cvd_ver_int;
			break;
		}
	}
	return cvd_int_ver;
}

static int voice_apr_register(uint32_t session_id)
{

	pr_debug("%s\n", __func__);

	mutex_lock(&common.common_lock);

	/* register callback to APR */
	if (common.apr_q6_mvm == NULL) {
		pr_debug("%s: Start to register MVM callback\n", __func__);

		common.apr_q6_mvm = apr_register("ADSP", "MVM",
						 qdsp_mvm_callback,
						 0xFFFFFFFF, &common);

		if (common.apr_q6_mvm == NULL) {
			pr_err("%s: Unable to register MVM\n", __func__);
			goto err;
		}
	}

	if (common.apr_q6_cvs == NULL) {
		pr_debug("%s: Start to register CVS callback\n", __func__);

		common.apr_q6_cvs = apr_register("ADSP", "CVS",
						 qdsp_cvs_callback,
						 0xFFFFFFFF, &common);

		if (common.apr_q6_cvs == NULL) {
			pr_err("%s: Unable to register CVS\n", __func__);
			goto err;
		}
		rtac_set_voice_handle(RTAC_CVS, common.apr_q6_cvs);
	}

	if (common.apr_q6_cvp == NULL) {
		pr_debug("%s: Start to register CVP callback\n", __func__);

		common.apr_q6_cvp = apr_register("ADSP", "CVP",
						 qdsp_cvp_callback,
						 0xFFFFFFFF, &common);

		if (common.apr_q6_cvp == NULL) {
			pr_err("%s: Unable to register CVP\n", __func__);
			goto err;
		}
		rtac_set_voice_handle(RTAC_CVP, common.apr_q6_cvp);
	}

	mutex_unlock(&common.common_lock);

	return 0;

err:
	if (common.apr_q6_cvs != NULL) {
		apr_deregister(common.apr_q6_cvs);
		common.apr_q6_cvs = NULL;
		rtac_set_voice_handle(RTAC_CVS, NULL);
	}
	if (common.apr_q6_mvm != NULL) {
		apr_deregister(common.apr_q6_mvm);
		common.apr_q6_mvm = NULL;
	}

	mutex_unlock(&common.common_lock);

	return -ENODEV;
}

static int voice_send_mvm_cvd_version_cmd(struct voice_data *v)
{
	int ret;
	struct apr_hdr cvd_version_get_cmd;
	void *apr_mvm;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	apr_mvm = common.apr_q6_mvm;
	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	/* Send command to CVD to retrieve Version */
	cvd_version_get_cmd.hdr_field = APR_HDR_FIELD(
				APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE),
				APR_PKT_VER);
	cvd_version_get_cmd.pkt_size = APR_PKT_SIZE(
				APR_HDR_SIZE,
				sizeof(cvd_version_get_cmd) -
				APR_HDR_SIZE);
	cvd_version_get_cmd.src_port =
		voice_get_idx_for_session(v->session_id);
	cvd_version_get_cmd.dest_port = 0;
	cvd_version_get_cmd.token = 0;
	cvd_version_get_cmd.opcode = VSS_IVERSION_CMD_GET;

	pr_debug("%s: send CVD version get cmd, pkt size = %d\n",
		 __func__, cvd_version_get_cmd.pkt_size);

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm,
			   (uint32_t *) &cvd_version_get_cmd);
	if (ret < 0) {
		pr_err("%s: Error sending command\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	ret = wait_event_timeout(v->mvm_wait,
			(v->mvm_state == CMD_STATUS_SUCCESS),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout, fall back to default\n",
		       __func__);

		ret = -EINVAL;
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			v->async_err));
		ret = adsp_err_get_lnx_err_code(
			v->async_err);
		goto done;
	}
	ret = 0;

done:
	if (ret) {
		strlcpy(common.cvd_version, CVD_VERSION_0_0,
				sizeof(common.cvd_version));
	}
	pr_debug("%s: CVD Version retrieved=%s\n",
		 __func__, common.cvd_version);

	return ret;
}

static int voice_send_mvm_event_class_cmd(struct voice_data *v,
					   uint32_t event_id,
					   uint32_t class_id)
{
	struct vss_inotify_cmd_event_class_t mvm_event;
	int ret = 0;
	void *apr_mvm = NULL;
	u16 mvm_handle = 0;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}

	apr_mvm = common.apr_q6_mvm;
	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		return -EINVAL;
	}

	memset(&mvm_event, 0, sizeof(mvm_event));
	mvm_handle = voice_get_mvm_handle(v);
	mvm_event.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	mvm_event.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_event) - APR_HDR_SIZE);
	mvm_event.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	mvm_event.hdr.dest_port = mvm_handle;
	mvm_event.hdr.token = 0;
	mvm_event.hdr.opcode = event_id;
	mvm_event.class_id = class_id;

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_event);
	if (ret < 0) {
		pr_err("%s: Error %d sending %x event\n", __func__, ret,
			event_id);
		goto fail;
	}

	ret = wait_event_timeout(v->mvm_wait,
				(v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout %d\n", __func__, ret);
		ret = -ETIMEDOUT;
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}
	return 0;
fail:
	return ret;
}

static int voice_send_dual_control_cmd(struct voice_data *v)
{
	int ret = 0;
	struct mvm_modem_dual_control_session_cmd mvm_voice_ctl_cmd;
	void *apr_mvm;
	u16 mvm_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;
	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: Send Dual Control command to MVM\n", __func__);
	if (!is_voip_session(v->session_id)) {
		mvm_handle = voice_get_mvm_handle(v);
		mvm_voice_ctl_cmd.hdr.hdr_field = APR_HDR_FIELD(
						APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
		mvm_voice_ctl_cmd.hdr.pkt_size = APR_PKT_SIZE(
						APR_HDR_SIZE,
						sizeof(mvm_voice_ctl_cmd) -
						APR_HDR_SIZE);
		pr_debug("%s: send mvm Voice Ctl pkt size = %d\n",
			__func__, mvm_voice_ctl_cmd.hdr.pkt_size);
		mvm_voice_ctl_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
		mvm_voice_ctl_cmd.hdr.dest_port = mvm_handle;
		mvm_voice_ctl_cmd.hdr.token = 0;
		mvm_voice_ctl_cmd.hdr.opcode =
					VSS_IMVM_CMD_SET_POLICY_DUAL_CONTROL;
		mvm_voice_ctl_cmd.voice_ctl.enable_flag = true;
		v->mvm_state = CMD_STATUS_FAIL;
		v->async_err = 0;

		ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_voice_ctl_cmd);
		if (ret < 0) {
			pr_err("%s: Error sending MVM Voice CTL CMD\n",
							__func__);
			ret = -EINVAL;
			goto fail;
		}
		ret = wait_event_timeout(v->mvm_wait,
				(v->mvm_state == CMD_STATUS_SUCCESS),
				msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);
			ret = -EINVAL;
			goto fail;
		}
		if (v->async_err > 0) {
			pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
			ret = adsp_err_get_lnx_err_code(
					v->async_err);
			goto fail;
		}
	}
	ret = 0;
fail:
	return ret;
}


static int voice_create_mvm_cvs_session(struct voice_data *v)
{
	int ret = 0;
	struct mvm_create_ctl_session_cmd mvm_session_cmd;
	struct cvs_create_passive_ctl_session_cmd cvs_session_cmd;
	struct cvs_create_full_ctl_session_cmd cvs_full_ctl_cmd;
	struct mvm_attach_stream_cmd attach_stream_cmd;
	void *apr_mvm, *apr_cvs, *apr_cvp;
	u16 mvm_handle, cvs_handle, cvp_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;
	apr_cvs = common.apr_q6_cvs;
	apr_cvp = common.apr_q6_cvp;

	if (!apr_mvm || !apr_cvs || !apr_cvp) {
		pr_err("%s: apr_mvm or apr_cvs or apr_cvp is NULL\n", __func__);
		return -EINVAL;
	}
	mvm_handle = voice_get_mvm_handle(v);
	cvs_handle = voice_get_cvs_handle(v);
	cvp_handle = voice_get_cvp_handle(v);

	pr_debug("%s: mvm_hdl=%d, cvs_hdl=%d\n", __func__,
		mvm_handle, cvs_handle);
	/* send cmd to create mvm session and wait for response */

	if (!mvm_handle) {
		memset(mvm_session_cmd.mvm_session.name, 0,
			sizeof(mvm_session_cmd.mvm_session.name));
		if (!is_voip_session(v->session_id)) {
			mvm_session_cmd.hdr.hdr_field = APR_HDR_FIELD(
						APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
			mvm_session_cmd.hdr.pkt_size = APR_PKT_SIZE(
						APR_HDR_SIZE,
						sizeof(mvm_session_cmd) -
						APR_HDR_SIZE);
			pr_debug("%s: send mvm create session pkt size = %d\n",
				 __func__, mvm_session_cmd.hdr.pkt_size);
			mvm_session_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
			mvm_session_cmd.hdr.dest_port = 0;
			mvm_session_cmd.hdr.token = 0;
			mvm_session_cmd.hdr.opcode =
				VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION;
			if (is_volte_session(v->session_id)) {
				strlcpy(mvm_session_cmd.mvm_session.name,
				"default volte voice",
				strlen("default volte voice")+1);
			} else if (is_voice2_session(v->session_id)) {
				strlcpy(mvm_session_cmd.mvm_session.name,
				VOICE2_SESSION_VSID_STR,
				strlen(VOICE2_SESSION_VSID_STR)+1);
			} else if (is_qchat_session(v->session_id)) {
				strlcpy(mvm_session_cmd.mvm_session.name,
				QCHAT_SESSION_VSID_STR,
				strlen(QCHAT_SESSION_VSID_STR)+1);
			} else if (is_vowlan_session(v->session_id)) {
				strlcpy(mvm_session_cmd.mvm_session.name,
				VOWLAN_SESSION_VSID_STR,
				strlen(VOWLAN_SESSION_VSID_STR)+1);
			} else if (is_voicemmode1(v->session_id)) {
				strlcpy(mvm_session_cmd.mvm_session.name,
				VOICEMMODE1_VSID_STR,
				strlen(VOICEMMODE1_VSID_STR) + 1);
			} else if (is_voicemmode2(v->session_id)) {
				strlcpy(mvm_session_cmd.mvm_session.name,
				VOICEMMODE2_VSID_STR,
				strlen(VOICEMMODE2_VSID_STR) + 1);
			} else {
				strlcpy(mvm_session_cmd.mvm_session.name,
				"default modem voice",
				strlen("default modem voice")+1);
			}

			v->mvm_state = CMD_STATUS_FAIL;
			v->async_err = 0;

			ret = apr_send_pkt(apr_mvm,
					(uint32_t *) &mvm_session_cmd);
			if (ret < 0) {
				pr_err("%s: Error sending MVM_CONTROL_SESSION\n",
				       __func__);
				goto fail;
			}
			ret = wait_event_timeout(v->mvm_wait,
					(v->mvm_state == CMD_STATUS_SUCCESS),
					msecs_to_jiffies(TIMEOUT_MS));
			if (!ret) {
				pr_err("%s: wait_event timeout\n", __func__);
				goto fail;
			}
			if (v->async_err > 0) {
				pr_err("%s: DSP returned error[%s]\n",
					__func__, adsp_err_get_err_str(
					v->async_err));
				ret = adsp_err_get_lnx_err_code(
						v->async_err);
				goto fail;
			}
		} else {
			pr_debug("%s: creating MVM full ctrl\n", __func__);
			mvm_session_cmd.hdr.hdr_field =
					APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
			mvm_session_cmd.hdr.pkt_size =
					APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(mvm_session_cmd) -
					APR_HDR_SIZE);
			mvm_session_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
			mvm_session_cmd.hdr.dest_port = 0;
			mvm_session_cmd.hdr.token = 0;
			mvm_session_cmd.hdr.opcode =
				VSS_IMVM_CMD_CREATE_FULL_CONTROL_SESSION;
			strlcpy(mvm_session_cmd.mvm_session.name,
				"default voip",
				strlen("default voip")+1);

			v->mvm_state = CMD_STATUS_FAIL;
			v->async_err = 0;

			ret = apr_send_pkt(apr_mvm,
					(uint32_t *) &mvm_session_cmd);
			if (ret < 0) {
				pr_err("Fail in sending MVM_CONTROL_SESSION\n");
				goto fail;
			}
			ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
			if (!ret) {
				pr_err("%s: wait_event timeout\n", __func__);
				goto fail;
			}
			if (v->async_err > 0) {
				pr_err("%s: DSP returned error[%s]\n",
					__func__, adsp_err_get_err_str(
					v->async_err));
				ret = adsp_err_get_lnx_err_code(
						v->async_err);
				goto fail;
			}
		}
		/* Get the created MVM handle. */
		mvm_handle = voice_get_mvm_handle(v);
	}
	/* send cmd to create cvs session */
	if (!cvs_handle) {
		memset(cvs_session_cmd.cvs_session.name, 0,
			sizeof(cvs_session_cmd.cvs_session.name));
		if (!is_voip_session(v->session_id)) {
			pr_debug("%s: creating CVS passive session\n",
				 __func__);

			cvs_session_cmd.hdr.hdr_field = APR_HDR_FIELD(
						APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
			cvs_session_cmd.hdr.pkt_size =
						APR_PKT_SIZE(APR_HDR_SIZE,
						sizeof(cvs_session_cmd) -
						APR_HDR_SIZE);
			cvs_session_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
			cvs_session_cmd.hdr.dest_port = 0;
			cvs_session_cmd.hdr.token = 0;
			cvs_session_cmd.hdr.opcode =
				VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION;
			if (is_volte_session(v->session_id)) {
				strlcpy(cvs_session_cmd.cvs_session.name,
				"default volte voice",
				strlen("default volte voice")+1);
			} else if (is_voice2_session(v->session_id)) {
				strlcpy(cvs_session_cmd.cvs_session.name,
				VOICE2_SESSION_VSID_STR,
				strlen(VOICE2_SESSION_VSID_STR)+1);
			} else if (is_qchat_session(v->session_id)) {
				strlcpy(cvs_session_cmd.cvs_session.name,
				QCHAT_SESSION_VSID_STR,
				strlen(QCHAT_SESSION_VSID_STR)+1);
			} else if (is_vowlan_session(v->session_id)) {
				strlcpy(cvs_session_cmd.cvs_session.name,
				VOWLAN_SESSION_VSID_STR,
				strlen(VOWLAN_SESSION_VSID_STR)+1);
			} else if (is_voicemmode1(v->session_id)) {
				strlcpy(cvs_session_cmd.cvs_session.name,
				VOICEMMODE1_VSID_STR,
				strlen(VOICEMMODE1_VSID_STR) + 1);
			} else if (is_voicemmode2(v->session_id)) {
				strlcpy(cvs_session_cmd.cvs_session.name,
				VOICEMMODE2_VSID_STR,
				strlen(VOICEMMODE2_VSID_STR) + 1);
			} else {
			strlcpy(cvs_session_cmd.cvs_session.name,
				"default modem voice",
				strlen("default modem voice")+1);
			}
			v->cvs_state = CMD_STATUS_FAIL;
			v->async_err = 0;

			ret = apr_send_pkt(apr_cvs,
					(uint32_t *) &cvs_session_cmd);
			if (ret < 0) {
				pr_err("Fail in sending STREAM_CONTROL_SESSION\n");
				goto fail;
			}
			ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
			if (!ret) {
				pr_err("%s: wait_event timeout\n", __func__);
				goto fail;
			}
			if (v->async_err > 0) {
				pr_err("%s: DSP returned error[%s]\n",
					__func__, adsp_err_get_err_str(
					v->async_err));
				ret = adsp_err_get_lnx_err_code(
						v->async_err);
				goto fail;
			}
			/* Get the created CVS handle. */
			cvs_handle = voice_get_cvs_handle(v);

		} else {
			pr_debug("%s: creating CVS full session\n", __func__);

			cvs_full_ctl_cmd.hdr.hdr_field =
					APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE),
					APR_PKT_VER);

			cvs_full_ctl_cmd.hdr.pkt_size =
					APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(cvs_full_ctl_cmd) -
					APR_HDR_SIZE);

			cvs_full_ctl_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
			cvs_full_ctl_cmd.hdr.dest_port = 0;
			cvs_full_ctl_cmd.hdr.token = 0;
			cvs_full_ctl_cmd.hdr.opcode =
				VSS_ISTREAM_CMD_CREATE_FULL_CONTROL_SESSION;
			cvs_full_ctl_cmd.cvs_session.direction = 2;
			cvs_full_ctl_cmd.cvs_session.enc_media_type =
						common.mvs_info.media_type;
			cvs_full_ctl_cmd.cvs_session.dec_media_type =
						common.mvs_info.media_type;
			cvs_full_ctl_cmd.cvs_session.network_id =
					       common.mvs_info.network_type;

			memset(cvs_full_ctl_cmd.cvs_session.name, 0,
				sizeof(cvs_full_ctl_cmd.cvs_session.name));

			strlcpy(cvs_full_ctl_cmd.cvs_session.name,
				"default q6 voice",
				strlen("default q6 voice")+1);

			v->cvs_state = CMD_STATUS_FAIL;
			v->async_err = 0;

			ret = apr_send_pkt(apr_cvs,
					   (uint32_t *) &cvs_full_ctl_cmd);

			if (ret < 0) {
				pr_err("%s: Err %d sending CREATE_FULL_CTRL\n",
					__func__, ret);
				goto fail;
			}
			ret = wait_event_timeout(v->cvs_wait,
					(v->cvs_state == CMD_STATUS_SUCCESS),
					msecs_to_jiffies(TIMEOUT_MS));
			if (!ret) {
				pr_err("%s: wait_event timeout\n", __func__);
				goto fail;
			}
			if (v->async_err > 0) {
				pr_err("%s: DSP returned error[%s]\n",
					__func__, adsp_err_get_err_str(
					v->async_err));
				ret = adsp_err_get_lnx_err_code(
						v->async_err);
				goto fail;
			}
			/* Get the created CVS handle. */
			cvs_handle = voice_get_cvs_handle(v);

			/* Attach MVM to CVS. */
			pr_debug("%s: Attach MVM to stream\n", __func__);

			attach_stream_cmd.hdr.hdr_field =
					APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE),
					APR_PKT_VER);
			attach_stream_cmd.hdr.pkt_size =
					APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(attach_stream_cmd) -
					APR_HDR_SIZE);
			attach_stream_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
			attach_stream_cmd.hdr.dest_port = mvm_handle;
			attach_stream_cmd.hdr.token = 0;
			attach_stream_cmd.hdr.opcode =
						VSS_IMVM_CMD_ATTACH_STREAM;
			attach_stream_cmd.attach_stream.handle = cvs_handle;

			v->mvm_state = CMD_STATUS_FAIL;
			v->async_err = 0;
			ret = apr_send_pkt(apr_mvm,
					   (uint32_t *) &attach_stream_cmd);
			if (ret < 0) {
				pr_err("%s: Error %d sending ATTACH_STREAM\n",
				       __func__, ret);
				goto fail;
			}
			ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
			if (!ret) {
				pr_err("%s: wait_event timeout\n", __func__);
				goto fail;
			}
			if (v->async_err > 0) {
				pr_err("%s: DSP returned error[%s]\n",
					__func__, adsp_err_get_err_str(
					v->async_err));
				ret = adsp_err_get_lnx_err_code(
						v->async_err);
				goto fail;
			}
		}
	}
	return 0;

fail:
	return ret;
}

static int voice_unmap_cal_block(struct voice_data *v, int cal_index)
{
	int result = 0;
	struct cal_block_data *cal_block;
	int dest_perms[1] = {PERM_READ | PERM_WRITE | PERM_EXEC};
	int source_vm[2] = {VMID_LPASS, VMID_ADSP_HEAP};
	int dest_vm[1] = {VMID_HLOS};

	if (common.cal_data[cal_index] == NULL) {
		pr_err("%s: Cal type is NULL, index %d!\n",
			__func__, cal_index);

		goto done;
	}

	mutex_lock(&common.cal_data[cal_index]->lock);
	cal_block = cal_utils_get_only_cal_block(
		common.cal_data[cal_index]);
	if (cal_block == NULL) {
		pr_err("%s: Cal block is NULL, index %d!\n",
			__func__, cal_index);

		result = -EINVAL;
		goto unlock;
	}

	if (cal_block->map_data.q6map_handle == 0) {
		pr_debug("%s: Q6 handle is not set!\n", __func__);

		result = -EINVAL;
		goto unlock;
	}

	result = voice_send_mvm_unmap_memory_physical_cmd(
		v, cal_block->map_data.q6map_handle);
	if (result)
		pr_err("%s: Voice_send_mvm_unmap_memory_physical_cmd failed for session 0x%x, err %d!\n",
			__func__, v->session_id, result);

	pr_debug("%s: use hyp assigned %d\n",__func__, hyp_assigned);
	if (cal_block->cma_mem && hyp_assigned) {
		if (cal_block->cal_data.paddr == 0 ||
		    cal_block->map_data.map_size <= 0) {
			pr_err("%s: No address to map!\n", __func__);
			cal_block->map_data.q6map_handle = 0;
			result = -EINVAL;
			goto unlock;
		}
		result = hyp_assign_phys(cal_block->cal_data.paddr,
					 cal_block->map_data.map_size,
					 source_vm, 2, dest_vm, dest_perms, 1);
		if (result < 0) {
			pr_err("%s: hyp_assign_phys failed result = %d addr = 0x%pK size = %d\n",
				__func__, result, cal_block->cal_data.paddr,
				cal_block->map_data.map_size);
			cal_block->map_data.q6map_handle = 0;
			result = -EINVAL;
			goto unlock;
		}
		hyp_assigned = false;
		pr_debug("%s: hyp_assign_phys success\n", __func__);
	}
	cal_block->map_data.q6map_handle = 0;
unlock:
	mutex_unlock(&common.cal_data[cal_index]->lock);
done:
	return result;
}

static int voice_destroy_mvm_cvs_session(struct voice_data *v)
{
	int ret = 0, result = 0;
	struct mvm_detach_stream_cmd detach_stream;
	struct apr_hdr mvm_destroy;
	struct apr_hdr cvs_destroy;
	void *apr_mvm, *apr_cvs;
	u16 mvm_handle, cvs_handle;
	struct cal_block_data *cal_block;
	int dest_perms[1] = {PERM_READ | PERM_WRITE | PERM_EXEC};
	int source_vm[2] = {VMID_LPASS, VMID_ADSP_HEAP};
	int dest_vm[1] = {VMID_HLOS};

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;
	apr_cvs = common.apr_q6_cvs;

	if (!apr_mvm || !apr_cvs) {
		pr_err("%s: apr_mvm or apr_cvs is NULL\n", __func__);
		ret = -EINVAL;
		goto fail;
	}
	mvm_handle = voice_get_mvm_handle(v);
	cvs_handle = voice_get_cvs_handle(v);

	/* MVM, CVS sessions are destroyed only for Full control sessions. */
	if (is_voip_session(v->session_id)) {
		pr_debug("%s: MVM detach stream, VOC_STATE: %d\n", __func__,
				v->voc_state);

		/* Detach voice stream. */
		detach_stream.hdr.hdr_field =
					APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE),
					APR_PKT_VER);
		detach_stream.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(detach_stream) - APR_HDR_SIZE);
		detach_stream.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
		detach_stream.hdr.dest_port = mvm_handle;
		detach_stream.hdr.token = 0;
		detach_stream.hdr.opcode = VSS_IMVM_CMD_DETACH_STREAM;
		detach_stream.detach_stream.handle = cvs_handle;

		v->mvm_state = CMD_STATUS_FAIL;
		v->async_err = 0;
		ret = apr_send_pkt(apr_mvm, (uint32_t *) &detach_stream);
		if (ret < 0) {
			pr_err("%s: Error %d sending DETACH_STREAM\n",
			       __func__, ret);

			goto fail;
		}
		ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait event timeout\n", __func__);

			goto fail;
		}
		if (v->async_err > 0) {
			pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
			ret = adsp_err_get_lnx_err_code(
					v->async_err);
			goto fail;
		}

		/* Unmap memory */
		if (v->shmem_info.mem_handle != 0) {
			ret = voice_send_mvm_unmap_memory_physical_cmd(v,
						v->shmem_info.mem_handle);
			if (ret < 0) {
				pr_err("%s Memory_unmap for voip failed %d\n",
				       __func__, ret);

				goto fail;
			}
			v->shmem_info.mem_handle = 0;
		}
	}

	/* Unmap Source Tracking shared memory if mapped earlier */
	voice_unmap_and_free_source_tracking_shared_memory(v);

	if (is_voip_session(v->session_id) ||
	    is_qchat_session(v->session_id) ||
	    is_volte_session(v->session_id) ||
	    is_vowlan_session(v->session_id) ||
	    v->voc_state == VOC_ERROR || common.is_destroy_cvd) {
		/* Destroy CVS. */
		pr_debug("%s: CVS destroy session\n", __func__);

		cvs_destroy.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						      APR_HDR_LEN(APR_HDR_SIZE),
						      APR_PKT_VER);
		cvs_destroy.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(cvs_destroy) - APR_HDR_SIZE);
		cvs_destroy.src_port =
				voice_get_idx_for_session(v->session_id);
		cvs_destroy.dest_port = cvs_handle;
		cvs_destroy.token = 0;
		cvs_destroy.opcode = APRV2_IBASIC_CMD_DESTROY_SESSION;

		v->cvs_state = CMD_STATUS_FAIL;
		v->async_err = 0;
		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_destroy);
		if (ret < 0) {
			pr_err("%s: Error %d sending CVS DESTROY\n",
			       __func__, ret);

			goto fail;
		}
		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait event timeout\n", __func__);

			goto fail;
		}
		if (v->async_err > 0) {
			pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
			ret = adsp_err_get_lnx_err_code(
					v->async_err);
			goto fail;
		}
		cvs_handle = 0;
		voice_set_cvs_handle(v, cvs_handle);

		/* Unmap physical memory for all calibration buffers */
		if (!is_other_session_active(v->session_id)) {
			if (voice_unmap_cal_block(v, CVP_VOCPROC_CAL))
				pr_err("%s: Unmap VOCPROC cal failed\n",
					__func__);
			if (voice_unmap_cal_block(v, CVP_VOCVOL_CAL))
				pr_err("%s: Unmap VOCVOL cal failed\n",
					__func__);
			if (voice_unmap_cal_block(v, CVP_VOCDEV_CFG_CAL))
				pr_err("%s: Unmap VOCDEV_CFG cal failed\n",
					__func__);
			if (voice_unmap_cal_block(v, CVS_VOCSTRM_CAL))
				pr_err("%s: Unmap VOCSTRM cal failed\n",
					__func__);
		}

		/* Destroy MVM. */
		pr_debug("%s: MVM destroy session\n", __func__);

		mvm_destroy.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						      APR_HDR_LEN(APR_HDR_SIZE),
						      APR_PKT_VER);
		mvm_destroy.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					    sizeof(mvm_destroy) - APR_HDR_SIZE);
		mvm_destroy.src_port =
				voice_get_idx_for_session(v->session_id);
		mvm_destroy.dest_port = mvm_handle;
		mvm_destroy.token = 0;
		mvm_destroy.opcode = APRV2_IBASIC_CMD_DESTROY_SESSION;

		v->mvm_state = CMD_STATUS_FAIL;
		v->async_err = 0;

		ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_destroy);
		if (ret < 0) {
			pr_err("%s: Error %d sending MVM DESTROY\n",
			       __func__, ret);

			goto fail;
		}
		ret = wait_event_timeout(v->mvm_wait,
					 (v->mvm_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait event timeout\n", __func__);
			goto fail;
		}
		if (v->async_err > 0) {
			pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
			ret = adsp_err_get_lnx_err_code(
					v->async_err);
			goto fail;
		}
		mvm_handle = 0;
		voice_set_mvm_handle(v, mvm_handle);
	}
	return 0;
fail:
	pr_debug("%s: use hyp assigned %d\n",__func__, hyp_assigned);
	if (hyp_assigned) {
		mutex_lock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
		cal_block = cal_utils_get_only_cal_block(
				common.cal_data[CVP_VOCPROC_CAL]);
		if (cal_block == NULL) {
			pr_err("%s: Cal block NULL, CVP_VOCPROC_CAL!\n",
				__func__);
			mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
			return -EINVAL;
		}
		if (cal_block->cma_mem) {
			if (cal_block->cal_data.paddr == 0 ||
			    cal_block->map_data.map_size <= 0) {
				pr_err("%s: No address to map!\n", __func__);
				mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
				return -EINVAL;
			}
			result = hyp_assign_phys(
					cal_block->cal_data.paddr,
					cal_block->map_data.map_size,
					source_vm, 2, dest_vm, dest_perms, 1);
			if (result < 0) {
				pr_err("%s: hyp_assign_phys failed result = %d addr = 0x%pK size = %d\n",
					__func__, result,
					cal_block->cal_data.paddr,
					cal_block->map_data.map_size);
				mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
				return -EINVAL;
			}
			hyp_assigned = false;
			pr_debug("%s: hyp_assign_phys success\n", __func__);
			mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
		}
	}
	return ret;
}

static int voice_send_tty_mode_cmd(struct voice_data *v)
{
	int ret = 0;
	struct mvm_set_tty_mode_cmd mvm_tty_mode_cmd;
	void *apr_mvm;
	u16 mvm_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;

	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		return -EINVAL;
	}
	mvm_handle = voice_get_mvm_handle(v);

	/* send tty mode cmd to mvm */
	mvm_tty_mode_cmd.hdr.hdr_field = APR_HDR_FIELD(
					APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE),
					APR_PKT_VER);
	mvm_tty_mode_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(mvm_tty_mode_cmd) -
					APR_HDR_SIZE);
	pr_debug("%s: pkt size = %d\n",
		 __func__, mvm_tty_mode_cmd.hdr.pkt_size);
	mvm_tty_mode_cmd.hdr.src_port =
			voice_get_idx_for_session(v->session_id);
	mvm_tty_mode_cmd.hdr.dest_port = mvm_handle;
	mvm_tty_mode_cmd.hdr.token = 0;
	mvm_tty_mode_cmd.hdr.opcode = VSS_ISTREAM_CMD_SET_TTY_MODE;
	mvm_tty_mode_cmd.tty_mode.mode = v->tty_mode;
	pr_debug("tty mode =%d\n", mvm_tty_mode_cmd.tty_mode.mode);

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_tty_mode_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d sending SET_TTY_MODE\n",
		       __func__, ret);
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}
	return 0;
fail:
	return ret;
}

static int voice_send_cvp_ecns_enable_cmd(struct voice_data *v,
	uint32_t module_id, int enable)
{
	int ret;
	struct cvp_set_channel_ecns_cmd_v2 cvp_set_ch_ecns_cmd;
	void *apr_cvp;
	u16 cvp_handle;
	struct vss_icommon_param_data_ecns_t *cvp_config_param_data =
				&cvp_set_ch_ecns_cmd.
				cvp_set_ecns.param_data;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	apr_cvp = common.apr_q6_cvp;

	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	cvp_handle = voice_get_cvp_handle(v);
	memset(&cvp_set_ch_ecns_cmd, 0,
		sizeof(cvp_set_ch_ecns_cmd));

	cvp_set_ch_ecns_cmd.hdr.hdr_field =
			APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE),
			APR_PKT_VER);
	cvp_set_ch_ecns_cmd.hdr.pkt_size =
			APR_PKT_SIZE(APR_HDR_SIZE,
			sizeof(cvp_set_ch_ecns_cmd) - APR_HDR_SIZE);
	cvp_set_ch_ecns_cmd.hdr.src_svc = 0;
	cvp_set_ch_ecns_cmd.hdr.src_domain = APR_DOMAIN_APPS;
	cvp_set_ch_ecns_cmd.hdr.src_port =
			voice_get_idx_for_session(v->session_id);
	cvp_set_ch_ecns_cmd.hdr.dest_svc = 0;
	cvp_set_ch_ecns_cmd.hdr.dest_domain = APR_DOMAIN_ADSP;
	cvp_set_ch_ecns_cmd.hdr.dest_port = cvp_handle;
	cvp_set_ch_ecns_cmd.hdr.token = 0;
	cvp_set_ch_ecns_cmd.hdr.opcode = VSS_ICOMMON_CMD_SET_PARAM_V2;
	cvp_set_ch_ecns_cmd.cvp_set_ecns.mem_size =
			sizeof(struct vss_icommon_param_data_ecns_t);

	cvp_config_param_data->module_id = module_id;
	cvp_config_param_data->param_id = VOICE_PARAM_MOD_ENABLE;
	cvp_config_param_data->param_size = MOD_ENABLE_PARAM_LEN;
	cvp_config_param_data->reserved = 0;
	cvp_config_param_data->enable = enable;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *)&cvp_set_ch_ecns_cmd);

	if (ret < 0) {
		pr_err("%s: Failed to send VSS_ICOMMON_CMD_SET_PARAM_V2 %d\n",
		       __func__, ret);
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
				(v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));

	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s] handle = %d\n", __func__,
		       adsp_err_get_err_str(v->async_err), cvp_handle);
		ret = adsp_err_get_lnx_err_code(v->async_err);
		goto done;
	}
	ret = 0;
done:
	return ret;
}

/**
 * voc_set_ecns_enable -
 *       Command to set ECNS for voice module
 *
 * @session_id: voice session ID to send this command
 * @module_id: voice module id
 * @enable: enable/disable flag
 *
 * Returns 0 on success or error on failure
 */
int voc_set_ecns_enable(uint32_t session_id, uint32_t module_id,
	 uint32_t enable)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);
		return -EINVAL;
	}
	mutex_lock(&v->lock);
	v->ecns_enable = enable;
	v->ecns_module_id = module_id;

	if (is_voc_state_active(v->voc_state))
		ret = voice_send_cvp_ecns_enable_cmd(v,
				v->ecns_module_id, v->ecns_enable);

	mutex_unlock(&v->lock);

	return ret;
}
EXPORT_SYMBOL(voc_set_ecns_enable);

static int voice_send_set_pp_enable_cmd(
	struct voice_data *v, struct module_instance_info mod_inst_info,
	int enable)
{
	struct enable_param enable_param;
	struct param_hdr_v3 param_hdr;
	int ret = 0;

	memset(&enable_param, 0, sizeof(enable_param));
	memset(&param_hdr, 0, sizeof(param_hdr));
	param_hdr.module_id = mod_inst_info.module_id;
	param_hdr.instance_id = mod_inst_info.instance_id;
	param_hdr.param_id = VOICE_PARAM_MOD_ENABLE;
	param_hdr.param_size = sizeof(enable_param);
	enable_param.enable = enable ? 1 : 0;

	pr_debug("%s: module_id=%d, instance_id=%d, enable=%d\n",
		__func__, mod_inst_info.module_id, mod_inst_info.instance_id,
		enable);

	ret = voice_pack_and_set_cvs_ui_property(v, param_hdr,
						 (uint8_t *) &enable_param);
	if (ret < 0)
		pr_err("Fail: sending cvs set pp enable\n");

	return ret;
}

static int voice_send_hd_cmd(struct voice_data *v, int enable)
{
	struct mvm_set_hd_enable_cmd mvm_set_hd_cmd;
	int ret = 0;
	void *apr_mvm;
	u16 mvm_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	apr_mvm = common.apr_q6_mvm;
	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	mvm_handle = voice_get_mvm_handle(v);
	if (!mvm_handle) {
		pr_err("%s: mvm_handle is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	mvm_set_hd_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						     APR_HDR_LEN(APR_HDR_SIZE),
						     APR_PKT_VER);
	mvm_set_hd_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
						   sizeof(mvm_set_hd_cmd) -
						   APR_HDR_SIZE);
	mvm_set_hd_cmd.hdr.src_port = voice_get_idx_for_session(v->session_id);
	mvm_set_hd_cmd.hdr.dest_port = mvm_handle;
	mvm_set_hd_cmd.hdr.token = 0;

	if (enable)
		mvm_set_hd_cmd.hdr.opcode = VSS_IHDVOICE_CMD_ENABLE;
	else
		mvm_set_hd_cmd.hdr.opcode = VSS_IHDVOICE_CMD_DISABLE;

	pr_debug("%s: enable=%d\n", __func__, enable);

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_set_hd_cmd);
	if (ret < 0) {
		pr_err("%s: Failed to sending mvm set HD Voice enable %d\n",
		       __func__, ret);

		ret = -EINVAL;
		goto done;
	}

	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;
}

static int voice_set_dtx(struct voice_data *v)
{
	int ret = 0;
	void *apr_cvs;
	u16 cvs_handle;
	struct cvs_set_enc_dtx_mode_cmd cvs_set_dtx;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvs = common.apr_q6_cvs;

	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);
		return -EINVAL;
	}

	cvs_handle = voice_get_cvs_handle(v);

	/* Set DTX */
	cvs_set_dtx.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);
	cvs_set_dtx.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(cvs_set_dtx) - APR_HDR_SIZE);
	cvs_set_dtx.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvs_set_dtx.hdr.dest_port = cvs_handle;
	cvs_set_dtx.hdr.token = 0;
	cvs_set_dtx.hdr.opcode = VSS_ISTREAM_CMD_SET_ENC_DTX_MODE;
	cvs_set_dtx.dtx_mode.enable = common.mvs_info.dtx_mode;

	pr_debug("%s: Setting DTX %d\n", __func__, common.mvs_info.dtx_mode);

	v->cvs_state = CMD_STATUS_FAIL;
	v->async_err = 0;

	ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_dtx);
	if (ret < 0) {
		pr_err("%s: Error %d sending SET_DTX\n", __func__, ret);
		return -EINVAL;
	}

	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		return -EINVAL;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		return ret;
	}

	return 0;
}

static int voice_send_mvm_media_type_cmd(struct voice_data *v)
{
	struct vss_imvm_cmd_set_cal_media_type_t mvm_set_cal_media_type;
	int ret = 0;
	void *apr_mvm;
	u16 mvm_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;

	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		return -EINVAL;
	}
	mvm_handle = voice_get_mvm_handle(v);

	mvm_set_cal_media_type.hdr.hdr_field =
					APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE),
					APR_PKT_VER);
	mvm_set_cal_media_type.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(mvm_set_cal_media_type) -
					APR_HDR_SIZE);
	mvm_set_cal_media_type.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	mvm_set_cal_media_type.hdr.dest_port = mvm_handle;
	mvm_set_cal_media_type.hdr.token = 0;
	mvm_set_cal_media_type.hdr.opcode = VSS_IMVM_CMD_SET_CAL_MEDIA_TYPE;
	mvm_set_cal_media_type.media_id = common.mvs_info.media_type;
	pr_debug("%s: setting media_id as %x\n",
		 __func__, mvm_set_cal_media_type.media_id);

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_set_cal_media_type);
	if (ret < 0) {
		pr_err("%s: Error %d sending media type\n", __func__, ret);
		goto fail;
	}

	ret = wait_event_timeout(v->mvm_wait,
				(v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout %d\n", __func__, ret);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}
	return 0;
fail:
	return ret;
}

static int voice_send_dtmf_rx_detection_cmd(struct voice_data *v,
					    uint32_t enable)
{
	int ret = 0;
	void *apr_cvs;
	u16 cvs_handle;
	struct cvs_set_rx_dtmf_detection_cmd cvs_dtmf_rx_detection;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvs = common.apr_q6_cvs;

	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);
		return -EINVAL;
	}

	cvs_handle = voice_get_cvs_handle(v);

	/* Set SET_DTMF_RX_DETECTION */
	cvs_dtmf_rx_detection.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);
	cvs_dtmf_rx_detection.hdr.pkt_size =
				APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvs_dtmf_rx_detection) - APR_HDR_SIZE);
	cvs_dtmf_rx_detection.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvs_dtmf_rx_detection.hdr.dest_port = cvs_handle;
	cvs_dtmf_rx_detection.hdr.token = 0;
	cvs_dtmf_rx_detection.hdr.opcode =
					VSS_ISTREAM_CMD_SET_RX_DTMF_DETECTION;
	cvs_dtmf_rx_detection.cvs_dtmf_det.enable = enable;

	v->cvs_state = CMD_STATUS_FAIL;
	v->async_err = 0;

	ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_dtmf_rx_detection);
	if (ret < 0) {
		pr_err("%s: Error %d sending SET_DTMF_RX_DETECTION\n",
		       __func__,
		       ret);
		return -EINVAL;
	}

	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));

	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		return -EINVAL;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		return ret;
	}

	return ret;
}

/**
 * voc_disable_dtmf_det_on_active_sessions -
 *       command to disable DTMF detection for voice sessions
 *
 */
void voc_disable_dtmf_det_on_active_sessions(void)
{
	struct voice_data *v = NULL;
	int i;

	for (i = 0; i < MAX_VOC_SESSIONS; i++) {
		v = &common.voice[i];
		if ((v->dtmf_rx_detect_en) &&
			is_voc_state_active(v->voc_state)) {

			pr_debug("disable dtmf det on ses_id=%d\n",
				 v->session_id);
			voice_send_dtmf_rx_detection_cmd(v, 0);
		}
	}
}
EXPORT_SYMBOL(voc_disable_dtmf_det_on_active_sessions);

/**
 * voc_enable_dtmf_rx_detection -
 *       command to set DTMF RX detection
 *
 * @session_id: voice session ID to send this command
 * @enable: Enable or Disable detection
 *
 * Returns 0 on success or error on failure
 */
int voc_enable_dtmf_rx_detection(uint32_t session_id, uint32_t enable)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);
		return -EINVAL;
	}

	mutex_lock(&v->lock);
	v->dtmf_rx_detect_en = enable;

	if (is_voc_state_active(v->voc_state))
		ret = voice_send_dtmf_rx_detection_cmd(v,
						       v->dtmf_rx_detect_en);

	mutex_unlock(&v->lock);

	return ret;
}
EXPORT_SYMBOL(voc_enable_dtmf_rx_detection);

/**
 * voc_set_destroy_cvd_flag -
 *      set flag for destroy CVD session
 *
 * @is_destroy_cvd: bool value used to indicate
 *                  destroy CVD session or not.
 *
 */
void voc_set_destroy_cvd_flag(bool is_destroy_cvd)
{
	pr_debug("%s: %d\n", __func__, is_destroy_cvd);
	common.is_destroy_cvd = is_destroy_cvd;
}
EXPORT_SYMBOL(voc_set_destroy_cvd_flag);

/**
 * voc_alloc_cal_shared_memory -
 *       Alloc mem map table for calibration
 *
 * Returns 0 on success or error on failure
 */
int voc_alloc_cal_shared_memory(void)
{
	int rc = 0;

	mutex_lock(&common.common_lock);
	if (is_cal_memory_allocated()) {
		pr_debug("%s: Calibration shared buffer already allocated",
			 __func__);
	} else {
		/* Allocate memory for calibration memory map table. */
		rc = voice_alloc_cal_mem_map_table();
		if ((rc < 0) && (rc != -EPROBE_DEFER)) {
			pr_err("%s: Failed to allocate cal memory, err=%d",
			       __func__, rc);
		}
	}
	mutex_unlock(&common.common_lock);

	return rc;
}
EXPORT_SYMBOL(voc_alloc_cal_shared_memory);

/**
 * voc_alloc_voip_shared_memory -
 *       Alloc mem map table for OOB
 *
 * Returns 0 on success or error on failure
 */
int voc_alloc_voip_shared_memory(void)
{
	int rc = 0;

	/* Allocate shared memory for OOB Voip */
	rc = voice_alloc_oob_shared_mem();
	if (rc < 0) {
		pr_err("%s: Failed to alloc shared memory for OOB rc:%d\n",
			   __func__, rc);
	} else {
		/* Allocate mem map table for OOB */
		rc = voice_alloc_oob_mem_table();
		if (rc < 0) {
			pr_err("%s: Failed to alloc mem map talbe rc:%d\n",
			       __func__, rc);

			voice_free_oob_shared_mem();
		}
	}

	return rc;
}
EXPORT_SYMBOL(voc_alloc_voip_shared_memory);

static int is_cal_memory_allocated(void)
{
	bool ret;

	if (common.cal_mem_map_table.dma_buf != NULL)
		ret = true;
	else
		ret = false;

	return ret;
}


static int free_cal_map_table(void)
{
	int ret = 0;

	if (common.cal_mem_map_table.dma_buf == NULL)
		goto done;

	ret = msm_audio_ion_free(common.cal_mem_map_table.dma_buf);
	if (ret < 0)
		pr_err("%s: msm_audio_ion_free failed:\n", __func__);

done:
	common.cal_mem_map_table.dma_buf = NULL;
	return ret;
}

static int is_rtac_memory_allocated(void)
{
	bool ret;

	if (common.rtac_mem_map_table.dma_buf != NULL)
		ret = true;
	else
		ret = false;

	return ret;
}

static int free_rtac_map_table(void)
{
	int ret = 0;

	if (common.rtac_mem_map_table.dma_buf == NULL)
		goto done;

	ret = msm_audio_ion_free(common.rtac_mem_map_table.dma_buf);
	if (ret < 0)
		pr_err("%s: msm_audio_ion_free failed:\n", __func__);

done:
	common.rtac_mem_map_table.dma_buf = NULL;
	return ret;
}


static bool is_voip_memory_allocated(void)
{
	bool ret;
	struct voice_data *v = voice_get_session(
				common.voice[VOC_PATH_FULL].session_id);

	if (v == NULL) {
		pr_err("%s: v is NULL, session_id:%d\n", __func__,
		common.voice[VOC_PATH_FULL].session_id);

		ret = false;
		goto done;
	}

	mutex_lock(&common.common_lock);
	if (v->shmem_info.sh_buf.dma_buf != NULL)
		ret = true;
	else
		ret = false;
	mutex_unlock(&common.common_lock);

done:
	return ret;
}

static int voice_config_cvs_vocoder_amr_rate(struct voice_data *v)
{
	int ret = 0;
	void *apr_cvs;
	u16 cvs_handle;
	struct cvs_set_amr_enc_rate_cmd cvs_set_amr_rate;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	apr_cvs = common.apr_q6_cvs;

	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	cvs_handle = voice_get_cvs_handle(v);

	pr_debug("%s: Setting AMR rate. Media Type: %d\n", __func__,
		 common.mvs_info.media_type);

	cvs_set_amr_rate.hdr.hdr_field =
			APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE),
			APR_PKT_VER);
	cvs_set_amr_rate.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
			       sizeof(cvs_set_amr_rate) - APR_HDR_SIZE);
	cvs_set_amr_rate.hdr.src_port =
			voice_get_idx_for_session(v->session_id);
	cvs_set_amr_rate.hdr.dest_port = cvs_handle;
	cvs_set_amr_rate.hdr.token = 0;

	if (common.mvs_info.media_type == VSS_MEDIA_ID_AMR_NB_MODEM)
		cvs_set_amr_rate.hdr.opcode =
				VSS_ISTREAM_CMD_VOC_AMR_SET_ENC_RATE;
	else if (common.mvs_info.media_type == VSS_MEDIA_ID_AMR_WB_MODEM)
		cvs_set_amr_rate.hdr.opcode =
				VSS_ISTREAM_CMD_VOC_AMRWB_SET_ENC_RATE;

	cvs_set_amr_rate.amr_rate.mode = common.mvs_info.rate;

	v->cvs_state = CMD_STATUS_FAIL;
	v->async_err = 0;

	ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_amr_rate);
	if (ret < 0) {
		pr_err("%s: Error %d sending SET_AMR_RATE\n",
		       __func__, ret);

		goto done;
	}
	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

	return 0;
done:
	return ret;
}

static int voice_config_cvs_vocoder(struct voice_data *v)
{
	int ret = 0;
	void *apr_cvs;
	u16 cvs_handle;
	/* Set media type. */
	struct cvs_set_media_type_cmd cvs_set_media_cmd;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvs = common.apr_q6_cvs;

	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);
		return -EINVAL;
	}

	cvs_handle = voice_get_cvs_handle(v);

	cvs_set_media_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	cvs_set_media_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
						sizeof(cvs_set_media_cmd) -
						APR_HDR_SIZE);
	cvs_set_media_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvs_set_media_cmd.hdr.dest_port = cvs_handle;
	cvs_set_media_cmd.hdr.token = 0;
	cvs_set_media_cmd.hdr.opcode = VSS_ISTREAM_CMD_SET_MEDIA_TYPE;
	cvs_set_media_cmd.media_type.tx_media_id = common.mvs_info.media_type;
	cvs_set_media_cmd.media_type.rx_media_id = common.mvs_info.media_type;

	v->cvs_state = CMD_STATUS_FAIL;
	v->async_err = 0;

	ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_media_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d sending SET_MEDIA_TYPE\n",
			__func__, ret);

		goto fail;
	}
	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);

		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}
	/* Set encoder properties. */
	switch (common.mvs_info.media_type) {
	case VSS_MEDIA_ID_EVRC_MODEM:
	case VSS_MEDIA_ID_4GV_NB_MODEM:
	case VSS_MEDIA_ID_4GV_WB_MODEM:
	case VSS_MEDIA_ID_4GV_NW_MODEM: {
		struct cvs_set_cdma_enc_minmax_rate_cmd cvs_set_cdma_rate;

		pr_debug("Setting EVRC min-max rate\n");

		cvs_set_cdma_rate.hdr.hdr_field =
					APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE),
					APR_PKT_VER);
		cvs_set_cdma_rate.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				      sizeof(cvs_set_cdma_rate) - APR_HDR_SIZE);
		cvs_set_cdma_rate.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
		cvs_set_cdma_rate.hdr.dest_port = cvs_handle;
		cvs_set_cdma_rate.hdr.token = 0;
		cvs_set_cdma_rate.hdr.opcode =
				VSS_ISTREAM_CMD_CDMA_SET_ENC_MINMAX_RATE;
		cvs_set_cdma_rate.cdma_rate.min_rate =
				common.mvs_info.evrc_min_rate;
		cvs_set_cdma_rate.cdma_rate.max_rate =
				common.mvs_info.evrc_max_rate;

		v->cvs_state = CMD_STATUS_FAIL;
		v->async_err = 0;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_set_cdma_rate);
		if (ret < 0) {
			pr_err("%s: Error %d sending SET_EVRC_MINMAX_RATE\n",
			       __func__, ret);
			goto fail;
		}
		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}
		if (v->async_err > 0) {
			pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
			ret = adsp_err_get_lnx_err_code(
					v->async_err);
			goto fail;
		}

		if (common.mvs_info.media_type != VSS_MEDIA_ID_EVRC_MODEM) {
			ret = voice_set_dtx(v);
			if (ret < 0)
				goto fail;
		}

		break;
	}
	case VSS_MEDIA_ID_AMR_NB_MODEM:
	case VSS_MEDIA_ID_AMR_WB_MODEM: {
		ret = voice_config_cvs_vocoder_amr_rate(v);
		if (ret) {
			pr_err("%s: Failed to update vocoder rate. %d\n",
			       __func__, ret);

			goto fail;
		}

		ret = voice_set_dtx(v);
		if (ret < 0)
			goto fail;

		break;
	}
	case VSS_MEDIA_ID_G729:
	case VSS_MEDIA_ID_G711_ALAW:
	case VSS_MEDIA_ID_G711_MULAW: {
		ret = voice_set_dtx(v);

		break;
	}
	default:
		/* Do nothing. */
		break;
	}
	return 0;

fail:
	return ret;
}

/**
 * voc_update_amr_vocoder_rate -
 *       command to update AMR rate for voice session
 *
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_update_amr_vocoder_rate(uint32_t session_id)
{
	int ret = 0;
	struct voice_data *v;

	pr_debug("%s: session_id:%d", __func__, session_id);

	v = voice_get_session(session_id);

	if (v == NULL) {
		pr_err("%s: v is NULL, session_id:%d\n", __func__,
		       session_id);

		ret = -EINVAL;
		goto done;
	}

	mutex_lock(&v->lock);
	ret = voice_config_cvs_vocoder_amr_rate(v);
	mutex_unlock(&v->lock);

done:
	return ret;
}
EXPORT_SYMBOL(voc_update_amr_vocoder_rate);

static int voice_send_start_voice_cmd(struct voice_data *v)
{
	struct apr_hdr mvm_start_voice_cmd;
	int ret = 0;
	void *apr_mvm;
	u16 mvm_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;

	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		return -EINVAL;
	}
	mvm_handle = voice_get_mvm_handle(v);

	mvm_start_voice_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	mvm_start_voice_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_start_voice_cmd) - APR_HDR_SIZE);
	pr_debug("send mvm_start_voice_cmd pkt size = %d\n",
				mvm_start_voice_cmd.pkt_size);
	mvm_start_voice_cmd.src_port =
				voice_get_idx_for_session(v->session_id);
	mvm_start_voice_cmd.dest_port = mvm_handle;
	mvm_start_voice_cmd.token = 0;
	mvm_start_voice_cmd.opcode = VSS_IMVM_CMD_START_VOICE;

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_start_voice_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_IMVM_CMD_START_VOICE\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}
	return 0;
fail:
	return ret;
}

static void voc_get_tx_rx_topology(struct voice_data *v,
				   uint32_t *tx_topology_id,
				   uint32_t *rx_topology_id)
{
	uint32_t tx_id = 0;
	uint32_t rx_id = 0;

	if (v->lch_mode == VOICE_LCH_START || v->disable_topology) {
		pr_debug("%s: Setting TX and RX topology to NONE for LCH\n",
			 __func__);

		tx_id = VSS_IVOCPROC_TOPOLOGY_ID_NONE;
		rx_id = VSS_IVOCPROC_TOPOLOGY_ID_NONE;
	} else {
		tx_id = voice_get_topology(CVP_VOC_TX_TOPOLOGY_CAL);
		rx_id = voice_get_topology(CVP_VOC_RX_TOPOLOGY_CAL);
	}

	*tx_topology_id = tx_id;
	*rx_topology_id = rx_id;
}

static int voice_send_set_device_cmd(struct voice_data *v)
{
	struct cvp_set_device_cmd  cvp_setdev_cmd;
	int ret = 0;
	void *apr_cvp;
	u16 cvp_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvp = common.apr_q6_cvp;

	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);
		return -EINVAL;
	}
	cvp_handle = voice_get_cvp_handle(v);

	/* set device and wait for response */
	cvp_setdev_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	cvp_setdev_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_setdev_cmd) - APR_HDR_SIZE);
	pr_debug(" send create cvp setdev, pkt size = %d\n",
			cvp_setdev_cmd.hdr.pkt_size);
	cvp_setdev_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_setdev_cmd.hdr.dest_port = cvp_handle;
	cvp_setdev_cmd.hdr.token = 0;

	if (voice_get_cvd_int_version(common.cvd_version) >=
	    CVD_INT_VERSION_2_2)
		cvp_setdev_cmd.hdr.opcode =
				VSS_IVOCPROC_CMD_SET_DEVICE_V3;
	else
		cvp_setdev_cmd.hdr.opcode =
				VSS_IVOCPROC_CMD_SET_DEVICE_V2;

	voc_get_tx_rx_topology(v,
			&cvp_setdev_cmd.cvp_set_device_v2.tx_topology_id,
			&cvp_setdev_cmd.cvp_set_device_v2.rx_topology_id);

	voice_set_topology_specific_info(v, CVP_VOC_RX_TOPOLOGY_CAL);
	voice_set_topology_specific_info(v, CVP_VOC_TX_TOPOLOGY_CAL);

	cvp_setdev_cmd.cvp_set_device_v2.tx_port_id = v->dev_tx.port_id;
	cvp_setdev_cmd.cvp_set_device_v2.rx_port_id = v->dev_rx.port_id;

	if (common.ec_ref_ext) {
		cvp_setdev_cmd.cvp_set_device_v2.vocproc_mode =
				VSS_IVOCPROC_VOCPROC_MODE_EC_EXT_MIXING;
		cvp_setdev_cmd.cvp_set_device_v2.ec_ref_port_id =
				common.ec_media_fmt_info.port_id;
	} else {
		cvp_setdev_cmd.cvp_set_device_v2.vocproc_mode =
				    VSS_IVOCPROC_VOCPROC_MODE_EC_INT_MIXING;
		cvp_setdev_cmd.cvp_set_device_v2.ec_ref_port_id =
				    VSS_IVOCPROC_PORT_ID_NONE;
	}
	pr_debug("topology=%d , tx_port_id=%d, rx_port_id=%d\n",
		cvp_setdev_cmd.cvp_set_device_v2.tx_topology_id,
		cvp_setdev_cmd.cvp_set_device_v2.tx_port_id,
		cvp_setdev_cmd.cvp_set_device_v2.rx_port_id);

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_setdev_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_IVOCPROC_CMD_SET_DEVICE\n");
		goto fail;
	}

	ret = wait_event_timeout(v->cvp_wait,
			(v->cvp_state == CMD_STATUS_SUCCESS),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	return 0;
fail:
	return ret;
}

static int voice_send_stop_voice_cmd(struct voice_data *v)
{
	struct apr_hdr mvm_stop_voice_cmd;
	int ret = 0;
	void *apr_mvm;
	u16 mvm_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;

	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		return -EINVAL;
	}
	mvm_handle = voice_get_mvm_handle(v);

	mvm_stop_voice_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	mvm_stop_voice_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_stop_voice_cmd) - APR_HDR_SIZE);
	pr_debug("send mvm_stop_voice_cmd pkt size = %d\n",
				mvm_stop_voice_cmd.pkt_size);
	mvm_stop_voice_cmd.src_port =
				voice_get_idx_for_session(v->session_id);
	mvm_stop_voice_cmd.dest_port = mvm_handle;
	mvm_stop_voice_cmd.token = 0;
	mvm_stop_voice_cmd.opcode = VSS_IMVM_CMD_STOP_VOICE;

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_stop_voice_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_IMVM_CMD_STOP_VOICE\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	return 0;
fail:
	return ret;
}
static int voice_get_cal(struct cal_block_data **cal_block,
			 int cal_block_idx,
			 struct cal_block_data **col_data,
			 int col_data_idx, int session_id)
{
	int ret = 0;
	int dest_perms[2] = {PERM_READ | PERM_WRITE, PERM_READ | PERM_WRITE};
	int source_vm[1] = {VMID_HLOS};
	int dest_vm[2] = {VMID_LPASS, VMID_ADSP_HEAP};
	*cal_block = cal_utils_get_only_cal_block(
		common.cal_data[cal_block_idx]);
	if (*cal_block == NULL) {
		pr_err("%s: No cal data for cal %d!\n",
			__func__, cal_block_idx);

		ret = -ENODEV;
		goto done;
	}

	if ((*cal_block)->cma_mem) {
		if ((*cal_block)->cal_data.paddr == 0 ||
		    (*cal_block)->map_data.map_size <= 0) {
			pr_err("%s: No address to map!\n", __func__);

			ret = -EINVAL;
			goto done;
		}
		ret = hyp_assign_phys((*cal_block)->cal_data.paddr,
				      (*cal_block)->map_data.map_size,
				      source_vm, 1, dest_vm, dest_perms, 2);
		if (ret < 0) {
			pr_err("%s: hyp_assign_phys failed ret = %d addr = 0x%pK size = %d\n",
				__func__, ret, (*cal_block)->cal_data.paddr,
				(*cal_block)->map_data.map_size);
			ret = -EINVAL;
			goto done;
		}
		hyp_assigned = true;
		pr_debug("%s: hyp_assign_phys success\n", __func__);
	}
	ret = remap_cal_data(*cal_block, session_id);
	if (ret < 0) {
		pr_err("%s: Remap_cal_data failed for cal %d!\n",
			__func__, cal_block_idx);

		ret = -ENODEV;
		goto done;
	}

	if (col_data == NULL)
		goto done;

	*col_data = cal_utils_get_only_cal_block(
		common.cal_data[col_data_idx]);
	if (*col_data == NULL) {
		pr_err("%s: No cal data for cal %d!\n",
			__func__, col_data_idx);

		ret = -ENODEV;
		goto done;
	}
done:
	return ret;
}

static int voice_send_cvs_register_cal_cmd(struct voice_data *v)
{
	struct cvs_register_cal_data_cmd cvs_reg_cal_cmd;
	struct cal_block_data *cal_block = NULL;
	struct cal_block_data *col_data = NULL;
	int ret = 0;

	memset(&cvs_reg_cal_cmd, 0, sizeof(cvs_reg_cal_cmd));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (!common.apr_q6_cvs) {
		pr_err("%s: apr_cvs is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	mutex_lock(&common.cal_data[CVS_VOCSTRM_CAL]->lock);
	mutex_lock(&common.cal_data[CVS_VOCSTRM_COL_CAL]->lock);

	ret = voice_get_cal(&cal_block, CVS_VOCSTRM_CAL, &col_data,
		CVS_VOCSTRM_COL_CAL, v->session_id);
	if (ret < 0) {
		pr_err("%s: Voice_get_cal failed for cal %d!\n",
			__func__, CVS_VOCSTRM_CAL);

		goto unlock;
	}

	if (col_data->cal_data.size >= MAX_COL_INFO_SIZE) {
		pr_err("%s: Invalid cal data size %d!\n",
			__func__, col_data->cal_data.size);
		ret = -EINVAL;
		goto unlock;
	}

	memcpy(&cvs_reg_cal_cmd.cvs_cal_data.column_info[0],
	       (void *) &((struct audio_cal_info_voc_col *)
	       col_data->cal_info)->data,
	       col_data->cal_data.size);

	cvs_reg_cal_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvs_reg_cal_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvs_reg_cal_cmd) - APR_HDR_SIZE);
	cvs_reg_cal_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvs_reg_cal_cmd.hdr.dest_port = voice_get_cvs_handle(v);
	cvs_reg_cal_cmd.hdr.token = 0;
	if (common.is_per_vocoder_cal_enabled)
		cvs_reg_cal_cmd.hdr.opcode =
			VSS_ISTREAM_CMD_REGISTER_STATIC_CALIBRATION_DATA;
	else
		cvs_reg_cal_cmd.hdr.opcode =
			VSS_ISTREAM_CMD_REGISTER_CALIBRATION_DATA_V2;

	cvs_reg_cal_cmd.cvs_cal_data.cal_mem_handle =
		cal_block->map_data.q6map_handle;
	cvs_reg_cal_cmd.cvs_cal_data.cal_mem_address_lsw =
		lower_32_bits(cal_block->cal_data.paddr);
	cvs_reg_cal_cmd.cvs_cal_data.cal_mem_address_msw =
		msm_audio_populate_upper_32_bits(cal_block->cal_data.paddr);
	cvs_reg_cal_cmd.cvs_cal_data.cal_mem_size =
		cal_block->cal_data.size;

	v->cvs_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_cvs, (uint32_t *) &cvs_reg_cal_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d registering CVS cal\n", __func__, ret);

		ret = -EINVAL;
		goto unlock;
	}
	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);

		ret = -EINVAL;
		goto unlock;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto unlock;
	}
unlock:
	mutex_unlock(&common.cal_data[CVS_VOCSTRM_COL_CAL]->lock);
	mutex_unlock(&common.cal_data[CVS_VOCSTRM_CAL]->lock);
done:
	return ret;
}

static int voice_send_cvs_deregister_cal_cmd(struct voice_data *v)
{
	struct cvs_deregister_cal_data_cmd cvs_dereg_cal_cmd;
	int ret = 0;

	memset(&cvs_dereg_cal_cmd, 0, sizeof(cvs_dereg_cal_cmd));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (!common.apr_q6_cvs) {
		pr_err("%s: apr_cvs is NULL\n", __func__);

		ret = -EPERM;
		goto done;
	}

	cvs_dereg_cal_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvs_dereg_cal_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvs_dereg_cal_cmd) - APR_HDR_SIZE);
	cvs_dereg_cal_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvs_dereg_cal_cmd.hdr.dest_port = voice_get_cvs_handle(v);
	cvs_dereg_cal_cmd.hdr.token = 0;
	if (common.is_per_vocoder_cal_enabled)
		cvs_dereg_cal_cmd.hdr.opcode =
			VSS_ISTREAM_CMD_DEREGISTER_STATIC_CALIBRATION_DATA;
	else
		cvs_dereg_cal_cmd.hdr.opcode =
			VSS_ISTREAM_CMD_DEREGISTER_CALIBRATION_DATA;

	v->cvs_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_cvs, (uint32_t *) &cvs_dereg_cal_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d de-registering CVS cal\n", __func__, ret);
		goto done;
	}
	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command  timeout\n", __func__);
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;

}

static void voice_load_topo_modules(int cal_index)
{
	uint32_t topology_id;
	int ret;

	topology_id = voice_get_topology(cal_index);
	ret = q6core_load_unload_topo_modules(topology_id, CORE_LOAD_TOPOLOGY);
	if (ret < 0)
		pr_debug("%s ret:%d load topo modules %d failed\n",
			__func__, ret, topology_id);

}

static void voice_unload_topo_modules(void)
{
	uint32_t topology_id;
	int i, ret;

	for (i = CVP_VOC_RX_TOPOLOGY_CAL; i <= CVP_VOC_TX_TOPOLOGY_CAL; i++) {
		topology_id = voice_get_topology(i);
		ret = q6core_load_unload_topo_modules(topology_id,
				CORE_UNLOAD_TOPOLOGY);
		if (ret < 0) {
			pr_debug("%s ret:%d unload topo modules %d failed\n",
				  __func__, ret, topology_id);
		}
	}
}

static int voice_send_cvp_create_cmd(struct voice_data *v)
{
	struct cvp_create_full_ctl_session_cmd cvp_session_cmd;
	void *apr_cvp;
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	apr_cvp = common.apr_q6_cvp;
	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	/* create cvp session and wait for response */
	cvp_session_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	cvp_session_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_session_cmd) - APR_HDR_SIZE);
	pr_debug("%s: send create cvp session, pkt size = %d\n",
		 __func__, cvp_session_cmd.hdr.pkt_size);
	cvp_session_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_session_cmd.hdr.dest_port = 0;
	cvp_session_cmd.hdr.token = 0;

	if (voice_get_cvd_int_version(common.cvd_version) >=
	    CVD_INT_VERSION_2_2)
		cvp_session_cmd.hdr.opcode =
				VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V3;
	else
		cvp_session_cmd.hdr.opcode =
				VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V2;

	voc_get_tx_rx_topology(v,
			&cvp_session_cmd.cvp_session.tx_topology_id,
			&cvp_session_cmd.cvp_session.rx_topology_id);

	voice_set_topology_specific_info(v, CVP_VOC_RX_TOPOLOGY_CAL);
	voice_set_topology_specific_info(v, CVP_VOC_TX_TOPOLOGY_CAL);

	cvp_session_cmd.cvp_session.direction = 2; /*tx and rx*/
	cvp_session_cmd.cvp_session.tx_port_id = v->dev_tx.port_id;
	cvp_session_cmd.cvp_session.rx_port_id = v->dev_rx.port_id;
	cvp_session_cmd.cvp_session.profile_id =
					 VSS_ICOMMON_CAL_NETWORK_ID_NONE;
	if (common.ec_ref_ext) {
		cvp_session_cmd.cvp_session.vocproc_mode =
				VSS_IVOCPROC_VOCPROC_MODE_EC_EXT_MIXING;
		cvp_session_cmd.cvp_session.ec_ref_port_id =
				common.ec_media_fmt_info.port_id;
	} else {
		cvp_session_cmd.cvp_session.vocproc_mode =
				 VSS_IVOCPROC_VOCPROC_MODE_EC_INT_MIXING;
		cvp_session_cmd.cvp_session.ec_ref_port_id =
						 VSS_IVOCPROC_PORT_ID_NONE;
	}

	memset(cvp_session_cmd.cvp_session.name, 0,
                        sizeof(cvp_session_cmd.cvp_session.name));

	pr_debug("tx_topology: %d tx_port_id=%d, rx_port_id=%d, mode: 0x%x\n",
		cvp_session_cmd.cvp_session.tx_topology_id,
		cvp_session_cmd.cvp_session.tx_port_id,
		cvp_session_cmd.cvp_session.rx_port_id,
		cvp_session_cmd.cvp_session.vocproc_mode);
	pr_debug("rx_topology: %d, profile_id: 0x%x, pkt_size: %d\n",
		cvp_session_cmd.cvp_session.rx_topology_id,
		cvp_session_cmd.cvp_session.profile_id,
		cvp_session_cmd.hdr.pkt_size);

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_session_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VOCPROC_FULL_CONTROL_SESSION\n");

		ret = -EINVAL;
		goto done;
	}

	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;
}

static int voice_send_cvp_register_dev_cfg_cmd(struct voice_data *v)
{
	struct cvp_register_dev_cfg_cmd cvp_reg_dev_cfg_cmd;
	struct cal_block_data *cal_block = NULL;
	int ret = 0;

	memset(&cvp_reg_dev_cfg_cmd, 0, sizeof(cvp_reg_dev_cfg_cmd));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (!common.apr_q6_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);

		ret = -EPERM;
		goto done;
	}

	mutex_lock(&common.cal_data[CVP_VOCDEV_CFG_CAL]->lock);

	ret = voice_get_cal(&cal_block, CVP_VOCDEV_CFG_CAL, NULL,
		0, v->session_id);
	if (ret < 0) {
		pr_err("%s: Voice_get_cal failed for cal %d!\n",
			__func__, CVP_VOCDEV_CFG_CAL);

		goto unlock;
	}

	cvp_reg_dev_cfg_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_reg_dev_cfg_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_reg_dev_cfg_cmd) - APR_HDR_SIZE);
	cvp_reg_dev_cfg_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_reg_dev_cfg_cmd.hdr.dest_port = voice_get_cvp_handle(v);
	cvp_reg_dev_cfg_cmd.hdr.token = 0;
	cvp_reg_dev_cfg_cmd.hdr.opcode =
					VSS_IVOCPROC_CMD_REGISTER_DEVICE_CONFIG;

	cvp_reg_dev_cfg_cmd.cvp_dev_cfg_data.mem_handle =
		cal_block->map_data.q6map_handle;
	cvp_reg_dev_cfg_cmd.cvp_dev_cfg_data.mem_address_lsw =
		lower_32_bits(cal_block->cal_data.paddr);
	cvp_reg_dev_cfg_cmd.cvp_dev_cfg_data.mem_address_msw =
		msm_audio_populate_upper_32_bits(cal_block->cal_data.paddr);
	cvp_reg_dev_cfg_cmd.cvp_dev_cfg_data.mem_size =
		cal_block->cal_data.size;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_cvp,
			   (uint32_t *) &cvp_reg_dev_cfg_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d registering CVP dev cfg cal\n",
		       __func__, ret);

		ret = -EINVAL;
		goto unlock;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);

		ret = -EINVAL;
		goto unlock;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto unlock;
	}
unlock:
	mutex_unlock(&common.cal_data[CVP_VOCDEV_CFG_CAL]->lock);
done:
	return ret;
}

static int voice_send_cvp_deregister_dev_cfg_cmd(struct voice_data *v)
{
	struct cvp_deregister_dev_cfg_cmd cvp_dereg_dev_cfg_cmd;
	int ret = 0;

	memset(&cvp_dereg_dev_cfg_cmd, 0, sizeof(cvp_dereg_dev_cfg_cmd));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (!common.apr_q6_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);

		ret = -EPERM;
		goto done;
	}

	cvp_dereg_dev_cfg_cmd.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_dereg_dev_cfg_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_dereg_dev_cfg_cmd) - APR_HDR_SIZE);
	cvp_dereg_dev_cfg_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_dereg_dev_cfg_cmd.hdr.dest_port = voice_get_cvp_handle(v);
	cvp_dereg_dev_cfg_cmd.hdr.token = 0;
	cvp_dereg_dev_cfg_cmd.hdr.opcode =
				VSS_IVOCPROC_CMD_DEREGISTER_DEVICE_CONFIG;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_cvp,
			   (uint32_t *) &cvp_dereg_dev_cfg_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d de-registering CVP dev cfg cal\n",
		       __func__, ret);
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;
}

static int voice_send_cvp_register_cal_cmd(struct voice_data *v)
{
	struct cvp_register_cal_data_cmd cvp_reg_cal_cmd;
	struct cal_block_data *cal_block = NULL;
	struct cal_block_data *col_data = NULL;
	int ret = 0;

	memset(&cvp_reg_cal_cmd, 0, sizeof(cvp_reg_cal_cmd));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (!common.apr_q6_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	mutex_lock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
	mutex_lock(&common.cal_data[CVP_VOCPROC_COL_CAL]->lock);

	ret = voice_get_cal(&cal_block, CVP_VOCPROC_CAL, &col_data,
		CVP_VOCPROC_COL_CAL, v->session_id);
	if (ret < 0) {
		pr_err("%s: Voice_get_cal failed for cal %d!\n",
			__func__, CVP_VOCPROC_CAL);

		goto unlock;
	}

	v->dev_tx.dev_id = ((struct audio_cal_info_vocproc *)
				cal_block->cal_info)->tx_acdb_id;
	v->dev_rx.dev_id = ((struct audio_cal_info_vocproc *)
				cal_block->cal_info)->rx_acdb_id;
	pr_debug("%s: %s: Tx acdb id = %d and Rx acdb id = %d", __func__,
		 voc_get_session_name(v->session_id), v->dev_tx.dev_id,
		 v->dev_rx.dev_id);

	memcpy(&cvp_reg_cal_cmd.cvp_cal_data.column_info[0],
	       (void *) &((struct audio_cal_info_voc_col *)
	       col_data->cal_info)->data,
	       col_data->cal_data.size);

	cvp_reg_cal_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_reg_cal_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_reg_cal_cmd) - APR_HDR_SIZE);
	cvp_reg_cal_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_reg_cal_cmd.hdr.dest_port = voice_get_cvp_handle(v);
	cvp_reg_cal_cmd.hdr.token = 0;
	if (common.is_per_vocoder_cal_enabled)
		cvp_reg_cal_cmd.hdr.opcode =
			VSS_IVOCPROC_CMD_REGISTER_STATIC_CALIBRATION_DATA;
	else
		cvp_reg_cal_cmd.hdr.opcode =
			VSS_IVOCPROC_CMD_REGISTER_CALIBRATION_DATA_V2;

	cvp_reg_cal_cmd.cvp_cal_data.cal_mem_handle =
		cal_block->map_data.q6map_handle;
	cvp_reg_cal_cmd.cvp_cal_data.cal_mem_address_lsw =
		lower_32_bits(cal_block->cal_data.paddr);
	cvp_reg_cal_cmd.cvp_cal_data.cal_mem_address_msw =
		msm_audio_populate_upper_32_bits(cal_block->cal_data.paddr);
	cvp_reg_cal_cmd.cvp_cal_data.cal_mem_size =
		cal_block->cal_data.size;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_cvp, (uint32_t *) &cvp_reg_cal_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d registering CVP cal\n", __func__, ret);

		ret = -EINVAL;
		goto unlock;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);

		ret = -EINVAL;
		goto unlock;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto unlock;
	}
unlock:
	mutex_unlock(&common.cal_data[CVP_VOCPROC_COL_CAL]->lock);
	mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
done:
	return ret;
}

static int voice_send_cvp_deregister_cal_cmd(struct voice_data *v)
{
	struct cvp_deregister_cal_data_cmd cvp_dereg_cal_cmd;
	int ret = 0;
	int cal_index = CVP_VOCPROC_CAL;
	struct cal_block_data *cal_block;
	int dest_perms[1] = {PERM_READ | PERM_WRITE | PERM_EXEC};
	int source_vm[2] = {VMID_LPASS, VMID_ADSP_HEAP};
	int dest_vm[1] = {VMID_HLOS};

	memset(&cvp_dereg_cal_cmd, 0, sizeof(cvp_dereg_cal_cmd));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (!common.apr_q6_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);

		ret = -EPERM;
		goto done;
	}

	cvp_dereg_cal_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_dereg_cal_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_dereg_cal_cmd) - APR_HDR_SIZE);
	cvp_dereg_cal_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_dereg_cal_cmd.hdr.dest_port = voice_get_cvp_handle(v);
	cvp_dereg_cal_cmd.hdr.token = 0;
	if (common.is_per_vocoder_cal_enabled)
		cvp_dereg_cal_cmd.hdr.opcode =
			VSS_IVOCPROC_CMD_DEREGISTER_STATIC_CALIBRATION_DATA;
	else
		cvp_dereg_cal_cmd.hdr.opcode =
			VSS_IVOCPROC_CMD_DEREGISTER_CALIBRATION_DATA;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_cvp, (uint32_t *) &cvp_dereg_cal_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d de-registering CVP cal\n", __func__, ret);
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);
		goto done;
	}

	cal_block = cal_utils_get_only_cal_block(
			common.cal_data[cal_index]);
	if (cal_block == NULL) {
		pr_err("%s: Cal block is NULL, index %d!\n",
			__func__, cal_index);
		ret = -EINVAL;
		goto done;
	}
	pr_debug("%s: use hyp assigned %d\n",__func__, hyp_assigned);
	if (cal_block->cma_mem && hyp_assigned) {
		if (cal_block->cal_data.paddr == 0 ||
		    cal_block->map_data.map_size <= 0) {
			pr_err("%s: No address to map!\n", __func__);
			ret = -EINVAL;
			goto done;
		}
		ret = hyp_assign_phys(cal_block->cal_data.paddr,
				      cal_block->map_data.map_size,
				      source_vm, 2, dest_vm, dest_perms, 1);
		if (ret < 0) {
			pr_err("%s: hyp_assign_phys failed result = %d addr = 0x%pK size = %d\n",
				__func__, ret, cal_block->cal_data.paddr,
				cal_block->map_data.map_size);
			ret = -EINVAL;
			goto done;
		} else {
			hyp_assigned = false;
			pr_debug("%s: hyp_assign_phys success\n", __func__);
		}
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;
}

static int voice_send_cvp_register_vol_cal_cmd(struct voice_data *v)
{
	struct cvp_register_vol_cal_data_cmd cvp_reg_vol_cal_cmd;
	struct cal_block_data *cal_block = NULL;
	struct cal_block_data *col_data = NULL;
	int ret = 0;

	memset(&cvp_reg_vol_cal_cmd, 0, sizeof(cvp_reg_vol_cal_cmd));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (!common.apr_q6_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	mutex_lock(&common.cal_data[CVP_VOCVOL_CAL]->lock);
	mutex_lock(&common.cal_data[CVP_VOCVOL_COL_CAL]->lock);

	ret = voice_get_cal(&cal_block, CVP_VOCVOL_CAL, &col_data,
		CVP_VOCVOL_COL_CAL, v->session_id);
	if (ret < 0) {
		pr_err("%s: Voice_get_cal failed for cal %d!\n",
			__func__, CVP_VOCVOL_CAL);

		goto unlock;
	}

	memcpy(&cvp_reg_vol_cal_cmd.cvp_vol_cal_data.column_info[0],
	       (void *) &((struct audio_cal_info_voc_col *)
	       col_data->cal_info)->data,
	       col_data->cal_data.size);

	cvp_reg_vol_cal_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_reg_vol_cal_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_reg_vol_cal_cmd) - APR_HDR_SIZE);
	cvp_reg_vol_cal_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_reg_vol_cal_cmd.hdr.dest_port = voice_get_cvp_handle(v);
	cvp_reg_vol_cal_cmd.hdr.token = 0;
	if (common.is_per_vocoder_cal_enabled)
		cvp_reg_vol_cal_cmd.hdr.opcode =
			VSS_IVOCPROC_CMD_REGISTER_DYNAMIC_CALIBRATION_DATA;
	else
		cvp_reg_vol_cal_cmd.hdr.opcode =
			VSS_IVOCPROC_CMD_REGISTER_VOL_CALIBRATION_DATA;

	cvp_reg_vol_cal_cmd.cvp_vol_cal_data.cal_mem_handle =
		cal_block->map_data.q6map_handle;
	cvp_reg_vol_cal_cmd.cvp_vol_cal_data.cal_mem_address_lsw =
		lower_32_bits(cal_block->cal_data.paddr);
	cvp_reg_vol_cal_cmd.cvp_vol_cal_data.cal_mem_address_msw =
		msm_audio_populate_upper_32_bits(cal_block->cal_data.paddr);
	cvp_reg_vol_cal_cmd.cvp_vol_cal_data.cal_mem_size =
		cal_block->cal_data.size;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_cvp,
			   (uint32_t *) &cvp_reg_vol_cal_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d registering CVP vol cal\n", __func__, ret);

		ret = -EINVAL;
		goto unlock;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);

		ret = -EINVAL;
		goto unlock;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto unlock;
	}
unlock:
	mutex_unlock(&common.cal_data[CVP_VOCVOL_COL_CAL]->lock);
	mutex_unlock(&common.cal_data[CVP_VOCVOL_CAL]->lock);
done:
	return ret;
}

static int voice_send_cvp_deregister_vol_cal_cmd(struct voice_data *v)
{
	struct cvp_deregister_vol_cal_data_cmd cvp_dereg_vol_cal_cmd;
	int ret = 0;

	memset(&cvp_dereg_vol_cal_cmd, 0, sizeof(cvp_dereg_vol_cal_cmd));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (!common.apr_q6_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);

		ret = -EPERM;
		goto done;
	}

	cvp_dereg_vol_cal_cmd.hdr.hdr_field =
			APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_dereg_vol_cal_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_dereg_vol_cal_cmd) - APR_HDR_SIZE);
	cvp_dereg_vol_cal_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_dereg_vol_cal_cmd.hdr.dest_port = voice_get_cvp_handle(v);
	cvp_dereg_vol_cal_cmd.hdr.token = 0;
	if (common.is_per_vocoder_cal_enabled)
		cvp_dereg_vol_cal_cmd.hdr.opcode =
			VSS_IVOCPROC_CMD_DEREGISTER_DYNAMIC_CALIBRATION_DATA;
	else
		cvp_dereg_vol_cal_cmd.hdr.opcode =
			VSS_IVOCPROC_CMD_DEREGISTER_VOL_CALIBRATION_DATA;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_cvp,
			   (uint32_t *) &cvp_dereg_vol_cal_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d de-registering CVP vol cal\n",
		       __func__, ret);
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;
}

static int voice_map_memory_physical_cmd(struct voice_data *v,
					 struct mem_map_table *table_info,
					 dma_addr_t phys,
					 uint32_t size,
					 uint32_t token)
{
	struct vss_imemory_cmd_map_physical_t mvm_map_phys_cmd;
	uint32_t *memtable;
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto fail;
	}

	if (!common.apr_q6_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		ret = -EINVAL;
		goto fail;
	}

	if (!table_info->data) {
		pr_err("%s: memory table is NULL.\n", __func__);
		ret = -EINVAL;
		goto fail;
	}

	memtable = (uint32_t *) table_info->data;

	/*
	 * Store next table descriptor's address(64 bit) as NULL as there
	 * is only one memory block
	 */
	memtable[0] = 0;
	memtable[1] = 0;

	/* Store next table descriptor's size */
	memtable[2] = 0;

	/* Store shared mem adddress (64 bit) */
	memtable[3] = lower_32_bits(phys);
	memtable[4] = msm_audio_populate_upper_32_bits(phys);

	/* Store shared memory size */
	memtable[5] = size;

	mvm_map_phys_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_map_phys_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_map_phys_cmd) - APR_HDR_SIZE);
	mvm_map_phys_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	mvm_map_phys_cmd.hdr.dest_port = voice_get_mvm_handle(v);
	mvm_map_phys_cmd.hdr.token = token;
	mvm_map_phys_cmd.hdr.opcode = VSS_IMEMORY_CMD_MAP_PHYSICAL;

	mvm_map_phys_cmd.table_descriptor.mem_address_lsw =
			lower_32_bits(table_info->phys);
	mvm_map_phys_cmd.table_descriptor.mem_address_msw =
			msm_audio_populate_upper_32_bits(table_info->phys);
	mvm_map_phys_cmd.table_descriptor.mem_size =
			sizeof(struct vss_imemory_block_t) +
			sizeof(struct vss_imemory_table_descriptor_t);
	mvm_map_phys_cmd.is_cached = true;
	mvm_map_phys_cmd.cache_line_size = 128;
	mvm_map_phys_cmd.access_mask = 3;
	mvm_map_phys_cmd.page_align = 4096;
	mvm_map_phys_cmd.min_data_width = 8;
	mvm_map_phys_cmd.max_data_width = 64;

	pr_debug("%s: next table desc: add: %lld, size: %d\n",
		 __func__, *((uint64_t *) memtable),
		 *(((uint32_t *) memtable) + 2));
	pr_debug("%s: phy add of of mem being mapped LSW:0x%x, MSW:0x%x size: %d\n",
		 __func__, *(((uint32_t *) memtable) + 3),
		*(((uint32_t *) memtable) + 4), *(((uint32_t *) memtable) + 5));

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_mvm, (uint32_t *) &mvm_map_phys_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d sending mvm map phy cmd\n", __func__, ret);

		goto fail;
	}

	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);

		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	return 0;

fail:
	return ret;
}

static int voice_pause_voice_call(struct voice_data *v)
{
	struct apr_hdr	mvm_pause_voice_cmd;
	void		*apr_mvm;
	int		ret = 0;

	pr_debug("%s\n", __func__);

	if (v == NULL) {
		pr_err("%s: Voice data is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	apr_mvm = common.apr_q6_mvm;
	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	mvm_pause_voice_cmd.hdr_field =
		APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	mvm_pause_voice_cmd.pkt_size =
		APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(mvm_pause_voice_cmd) - APR_HDR_SIZE);
	mvm_pause_voice_cmd.src_port =
			voice_get_idx_for_session(v->session_id);
	mvm_pause_voice_cmd.dest_port = voice_get_mvm_handle(v);
	mvm_pause_voice_cmd.token = 0;
	mvm_pause_voice_cmd.opcode = VSS_IMVM_CMD_PAUSE_VOICE;
	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;

	pr_debug("%s: send mvm_pause_voice_cmd pkt size = %d\n",
		__func__, mvm_pause_voice_cmd.pkt_size);

	ret = apr_send_pkt(apr_mvm,
		(uint32_t *)&mvm_pause_voice_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_IMVM_CMD_PAUSE_VOICE\n");

		ret = -EINVAL;
		goto done;
	}

	ret = wait_event_timeout(v->mvm_wait,
		(v->mvm_state == CMD_STATUS_SUCCESS),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;
}

static int voice_map_cal_memory(struct cal_block_data *cal_block,
				uint32_t session_id)
{
	int result = 0;
	int voc_index;
	struct voice_data *v = NULL;

	pr_debug("%s\n", __func__);

	if (cal_block == NULL) {
		pr_err("%s: Cal block is NULL!\n", __func__);

		result = -EINVAL;
		goto done;
	}

	if (cal_block->cal_data.paddr == 0) {
		pr_debug("%s: No address to map!\n", __func__);

		result = -EINVAL;
		goto done;
	}

	if (cal_block->map_data.map_size == 0) {
		pr_debug("%s: Map size is 0!\n", __func__);

		result = -EINVAL;
		goto done;
	}

	voc_index = voice_get_idx_for_session(session_id);
	if (voc_index < 0) {
		pr_err("%s:  Invalid session ID %d\n", __func__, session_id);

		goto done;
	}

	v = &common.voice[voc_index];

	result = voice_map_memory_physical_cmd(v,
		&common.cal_mem_map_table,
		(dma_addr_t)cal_block->cal_data.paddr,
		cal_block->map_data.map_size,
		VOC_CAL_MEM_MAP_TOKEN);
	if (result < 0) {
		pr_err("%s: Mmap did not work! addr = 0x%pK, size = %zd\n",
			__func__,
			&cal_block->cal_data.paddr,
			cal_block->map_data.map_size);

		goto done;
	}

	cal_block->map_data.q6map_handle = common.cal_mem_handle;
done:
	return result;
}

static int remap_cal_data(struct cal_block_data *cal_block,
			   uint32_t session_id)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	if (cal_block->map_data.dma_buf == NULL) {
		pr_err("%s: No ION allocation for session_id %d!\n",
			__func__, session_id);
		ret = -EINVAL;
		goto done;
	}

	if ((cal_block->map_data.map_size > 0) &&
		(cal_block->map_data.q6map_handle == 0)) {

		/* cal type not used */
		ret = voice_map_cal_memory(cal_block, session_id);
		if (ret < 0) {
			pr_err("%s: Mmap did not work! size = %zd\n",
				__func__, cal_block->map_data.map_size);

			goto done;
		}
	} else {
		pr_debug("%s:  Cal block 0x%pK, size %zd already mapped. Q6 map handle = %d\n",
			__func__, &cal_block->cal_data.paddr,
			cal_block->map_data.map_size,
			cal_block->map_data.q6map_handle);
	}
done:
	return ret;
}

static int voice_unmap_cal_memory(int32_t cal_type,
				  struct cal_block_data *cal_block)
{
	int result = 0;
	int result2 = 0;
	int i;
	struct voice_data *v = NULL;

	pr_debug("%s\n", __func__);

	if (cal_block == NULL) {
		pr_err("%s: Cal block is NULL!\n", __func__);

		result = -EINVAL;
		goto done;
	}

	if (cal_block->map_data.q6map_handle == 0) {
		pr_debug("%s: Q6 handle is not set!\n", __func__);

		result = -EINVAL;
		goto done;
	}

	mutex_lock(&common.common_lock);

	for (i = 0; i < MAX_VOC_SESSIONS; i++) {
		v = &common.voice[i];

		mutex_lock(&v->lock);
		if (is_voc_state_active(v->voc_state)) {
			result2 = voice_pause_voice_call(v);
			if (result2 < 0) {
				pr_err("%s: Voice_pause_voice_call failed for session 0x%x, err %d!\n",
					__func__, v->session_id, result2);

				result = result2;
			}

			if (cal_type == CVP_VOCPROC_DYNAMIC_CAL_TYPE)
				voice_send_cvp_deregister_vol_cal_cmd(v);
			else if (cal_type == CVP_VOCPROC_STATIC_CAL_TYPE)
				voice_send_cvp_deregister_cal_cmd(v);
			else if (cal_type == CVP_VOCDEV_CFG_CAL_TYPE)
				voice_send_cvp_deregister_dev_cfg_cmd(v);
			else if (cal_type == CVS_VOCSTRM_STATIC_CAL_TYPE)
				voice_send_cvs_deregister_cal_cmd(v);
			else
				pr_err("%s: Invalid cal type %d!\n",
					__func__, cal_type);
		}

		if ((cal_block->map_data.q6map_handle != 0) &&
			(!is_other_session_active(v->session_id))) {

			result2 = voice_send_mvm_unmap_memory_physical_cmd(
				v, cal_block->map_data.q6map_handle);
			if (result2) {
				pr_err("%s: Voice_send_mvm_unmap_memory_physical_cmd failed for session 0x%x, err %d!\n",
					__func__, v->session_id, result2);

				result = result2;
			}
			cal_block->map_data.q6map_handle = 0;
		}
		mutex_unlock(&v->lock);
	}
	mutex_unlock(&common.common_lock);
done:
	return result;
}

int voc_register_vocproc_vol_table(void)
{
	int			result = 0;
	int			result2 = 0;
	int			i;
	struct voice_data	*v = NULL;

	pr_debug("%s\n", __func__);

	mutex_lock(&common.common_lock);
	for (i = 0; i < MAX_VOC_SESSIONS; i++) {
		v = &common.voice[i];

		mutex_lock(&v->lock);
		if (is_voc_state_active(v->voc_state)) {
			result2 = voice_send_cvp_register_vol_cal_cmd(v);
			if (result2 < 0) {
				pr_err("%s: Failed to register vocvol table for session 0x%x!\n",
					__func__, v->session_id);

				result = result2;
				/* Still try to register other sessions */
			}
		}
		mutex_unlock(&v->lock);
	}

	mutex_unlock(&common.common_lock);
	return result;
}

int voc_deregister_vocproc_vol_table(void)
{
	int			result = 0;
	int			success = 0;
	int			i;
	struct voice_data	*v = NULL;

	pr_debug("%s\n", __func__);

	mutex_lock(&common.common_lock);
	for (i = 0; i < MAX_VOC_SESSIONS; i++) {
		v = &common.voice[i];

		mutex_lock(&v->lock);
		if (is_voc_state_active(v->voc_state)) {
			result = voice_send_cvp_deregister_vol_cal_cmd(v);
			if (result < 0) {
				pr_err("%s: Failed to deregister vocvol table for session 0x%x!\n",
					__func__, v->session_id);

				mutex_unlock(&v->lock);
				mutex_unlock(&common.common_lock);
				if (success) {
					pr_err("%s: Try to re-register all deregistered sessions!\n",
						__func__);

					voc_register_vocproc_vol_table();
				}
				goto done;
			} else {
				success = 1;
			}
		}
		mutex_unlock(&v->lock);
	}
	mutex_unlock(&common.common_lock);
done:
	return result;
}

int voc_map_rtac_block(struct rtac_cal_block_data *cal_block)
{
	int			result = 0;
	struct voice_data	*v = NULL;

	pr_debug("%s\n", __func__);

	if (cal_block == NULL) {
		pr_err("%s: cal_block is NULL!\n",
			__func__);

		result = -EINVAL;
		goto done;
	}

	if (cal_block->cal_data.paddr == 0) {
		pr_debug("%s: No address to map!\n",
			__func__);

		result = -EINVAL;
		goto done;
	}

	if (cal_block->map_data.map_size == 0) {
		pr_debug("%s: map size is 0!\n",
			__func__);

		result = -EINVAL;
		goto done;
	}

	mutex_lock(&common.common_lock);
	/* use first session */
	v = &common.voice[0];
	mutex_lock(&v->lock);

	if (!is_rtac_memory_allocated()) {
		result = voice_alloc_rtac_mem_map_table();
		if (result < 0) {
			pr_err("%s: RTAC alloc mem map table did not work! addr = 0x%pK, size = %d\n",
				__func__,
				&cal_block->cal_data.paddr,
				cal_block->map_data.map_size);

			common.rtac_mem_map_table.dma_buf = NULL;
			goto done_unlock;
		}
	}

	result = voice_map_memory_physical_cmd(v,
		&common.rtac_mem_map_table,
		(dma_addr_t)cal_block->cal_data.paddr,
		cal_block->map_data.map_size,
		VOC_RTAC_MEM_MAP_TOKEN);
	if (result < 0) {
		pr_err("%s: RTAC mmap did not work! addr = 0x%pK, size = %d\n",
			__func__,
			&cal_block->cal_data.paddr,
			cal_block->map_data.map_size);

		free_rtac_map_table();
		goto done_unlock;
	}

	cal_block->map_data.map_handle = common.rtac_mem_handle;
done_unlock:
	mutex_unlock(&v->lock);
	mutex_unlock(&common.common_lock);
done:
	return result;
}

int voc_unmap_rtac_block(uint32_t *mem_map_handle)
{
	int			result = 0;
	struct voice_data	*v = NULL;

	pr_debug("%s\n", __func__);

	if (mem_map_handle == NULL) {
		pr_debug("%s: Map handle is NULL, nothing to unmap\n",
			__func__);

		goto done;
	}

	if (*mem_map_handle == 0) {
		pr_debug("%s: Map handle is 0, nothing to unmap\n",
			__func__);

		goto done;
	}

	mutex_lock(&common.common_lock);
	/* Use first session */
	/* Only used for apr wait lock */
	v = &common.voice[0];
	mutex_lock(&v->lock);

	result = voice_send_mvm_unmap_memory_physical_cmd(
			v, *mem_map_handle);
	if (result) {
		pr_err("%s: voice_send_mvm_unmap_memory_physical_cmd Failed for session 0x%x!\n",
			__func__, v->session_id);
	} else {
		*mem_map_handle = 0;
		common.rtac_mem_handle = 0;
		free_rtac_map_table();
	}
	mutex_unlock(&v->lock);
	mutex_unlock(&common.common_lock);
done:
	return result;
}

static int voice_send_cvp_channel_info_v2(struct voice_data *v,
					  uint32_t param_type)
{
	int ret;
	struct cvp_set_channel_info_cmd_v2 cvp_set_channel_info_cmd;
	void *apr_cvp;
	u16 cvp_handle;
	struct vss_icommon_param_data_channel_info_v2_t
		*channel_info_param_data =
			&cvp_set_channel_info_cmd.
			cvp_set_ch_info_param_v2.param_data;
	struct vss_param_vocproc_dev_channel_info_t *channel_info =
			&channel_info_param_data->channel_info;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	apr_cvp = common.apr_q6_cvp;
	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	cvp_handle = voice_get_cvp_handle(v);
	memset(&cvp_set_channel_info_cmd, 0, sizeof(cvp_set_channel_info_cmd));

	cvp_set_channel_info_cmd.hdr.hdr_field =
		APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD, APR_HDR_LEN(APR_HDR_SIZE),
			      APR_PKT_VER);
	cvp_set_channel_info_cmd.hdr.pkt_size =
		APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(cvp_set_channel_info_cmd) - APR_HDR_SIZE);
	cvp_set_channel_info_cmd.hdr.src_svc = 0;
	cvp_set_channel_info_cmd.hdr.src_domain = APR_DOMAIN_APPS;
	cvp_set_channel_info_cmd.hdr.src_port =
		voice_get_idx_for_session(v->session_id);
	cvp_set_channel_info_cmd.hdr.dest_svc = 0;
	cvp_set_channel_info_cmd.hdr.dest_domain = APR_DOMAIN_ADSP;
	cvp_set_channel_info_cmd.hdr.dest_port = cvp_handle;
	cvp_set_channel_info_cmd.hdr.token = 0;
	cvp_set_channel_info_cmd.hdr.opcode =  VSS_ICOMMON_CMD_SET_PARAM_V2;

	cvp_set_channel_info_cmd.cvp_set_ch_info_param_v2.mem_size =
			sizeof(struct vss_icommon_param_data_channel_info_v2_t);

	channel_info_param_data->module_id = VSS_MODULE_CVD_GENERIC;
	channel_info_param_data->param_size =
		sizeof(struct vss_param_vocproc_dev_channel_info_t);

	 /* Device specific data */
	switch (param_type) {
	case RX_PATH:
		channel_info_param_data->param_id =
			VSS_PARAM_VOCPROC_RX_CHANNEL_INFO;
		channel_info->num_channels = v->dev_rx.no_of_channels;
		channel_info->bits_per_sample = v->dev_rx.bits_per_sample;
		memcpy(&channel_info->channel_mapping,
		       v->dev_rx.channel_mapping,
		       VSS_NUM_CHANNELS_MAX * sizeof(uint8_t));
		break;

	case TX_PATH:
		channel_info_param_data->param_id =
			VSS_PARAM_VOCPROC_TX_CHANNEL_INFO;
		channel_info->num_channels = v->dev_tx.no_of_channels;
		channel_info->bits_per_sample = v->dev_tx.bits_per_sample;
		memcpy(&channel_info->channel_mapping,
		       v->dev_tx.channel_mapping,
		       VSS_NUM_CHANNELS_MAX * sizeof(uint8_t));
		break;

	case EC_REF_PATH:
		channel_info_param_data->param_id =
			VSS_PARAM_VOCPROC_EC_REF_CHANNEL_INFO;
		channel_info->num_channels = v->dev_rx.no_of_channels;
		channel_info->bits_per_sample = v->dev_rx.bits_per_sample;
		memcpy(&channel_info->channel_mapping,
		       v->dev_rx.channel_mapping,
		       VSS_NUM_CHANNELS_MAX * sizeof(uint8_t));
		break;
	default:
		pr_err("%s: Invalid param type\n",
		       __func__);
		ret = -EINVAL;
		goto done;
	}


	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_set_channel_info_cmd);
	if (ret < 0) {
		pr_err("%s: Failed to send VSS_ICOMMON_CMD_SET_PARAM_V2\n",
		       __func__);
		goto done;
	}

	ret = wait_event_timeout(v->cvp_wait,
				(v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s] handle = %d\n", __func__,
		       adsp_err_get_err_str(v->async_err), cvp_handle);
		ret = adsp_err_get_lnx_err_code(v->async_err);
		goto done;
	}
	ret = 0;
done:
	return ret;
}

static int voice_send_cvp_channel_info_cmd(struct voice_data *v)
{
	int ret = 0;

	ret = voice_send_cvp_channel_info_v2(v, RX_PATH);
	if (ret < 0) {
		pr_err("%s: Error in sending cvp_channel_info RX: %d\n",
		       __func__, ret);
		goto done;
	}

	ret = voice_send_cvp_channel_info_v2(v, TX_PATH);
	if (ret < 0) {
		pr_err("%s: Error in sending cvp_channel_info TX: %d\n",
		       __func__, ret);
		goto done;
	}

	ret = voice_send_cvp_channel_info_v2(v, EC_REF_PATH);
	if (ret < 0) {
		pr_err("%s: Error in sending cvp_channel_info EC Ref: %d\n",
		       __func__, ret);
		goto done;
	}
done:
	return ret;
}

static int voice_send_cvp_ch_mixer_info_v2(struct voice_data *v)
{
	int ret;
	struct cvp_set_channel_mixer_info_cmd_v2 cvp_set_ch_mixer_info_cmd;
	void *apr_cvp;
	u16 cvp_handle;
	struct vss_icommon_param_data_ch_mixer_v2_t *cvp_config_param_data =
			&cvp_set_ch_mixer_info_cmd.
			cvp_set_ch_mixer_param_v2.param_data;
	struct vss_param_channel_mixer_info_t *ch_mixer_info =
			&cvp_config_param_data->ch_mixer_info;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	apr_cvp = common.apr_q6_cvp;
	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	cvp_handle = voice_get_cvp_handle(v);
	memset(&cvp_set_ch_mixer_info_cmd, 0,
	       sizeof(cvp_set_ch_mixer_info_cmd));

	cvp_set_ch_mixer_info_cmd.hdr.hdr_field =
			APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE),
			APR_PKT_VER);
	cvp_set_ch_mixer_info_cmd.hdr.pkt_size =
			APR_PKT_SIZE(APR_HDR_SIZE,
			sizeof(cvp_set_ch_mixer_info_cmd) - APR_HDR_SIZE);
	cvp_set_ch_mixer_info_cmd.hdr.src_svc = 0;
	cvp_set_ch_mixer_info_cmd.hdr.src_domain = APR_DOMAIN_APPS;
	cvp_set_ch_mixer_info_cmd.hdr.src_port =
			voice_get_idx_for_session(v->session_id);
	cvp_set_ch_mixer_info_cmd.hdr.dest_svc = 0;
	cvp_set_ch_mixer_info_cmd.hdr.dest_domain = APR_DOMAIN_ADSP;
	cvp_set_ch_mixer_info_cmd.hdr.dest_port = cvp_handle;
	cvp_set_ch_mixer_info_cmd.hdr.token = VOC_GENERIC_SET_PARAM_TOKEN;
	cvp_set_ch_mixer_info_cmd.hdr.opcode = VSS_ICOMMON_CMD_SET_PARAM_V2;
	cvp_set_ch_mixer_info_cmd.cvp_set_ch_mixer_param_v2.mem_size =
			sizeof(struct vss_icommon_param_data_ch_mixer_v2_t);

	cvp_config_param_data->module_id = AUDPROC_MODULE_ID_MFC;
	cvp_config_param_data->param_id =
			AUDPROC_CHMIXER_PARAM_ID_COEFF;
	cvp_config_param_data->param_size =
			sizeof(struct vss_param_channel_mixer_info_t);

	ch_mixer_info->index = 0;
	ch_mixer_info->num_output_channels = v->dev_rx.no_of_channels;
	/*
	 * Configure Rx input to be mono for channel mixer as the DSP
	 * configures vocproc input as mono.
	 */
	ch_mixer_info->num_input_channels = NUM_CHANNELS_MONO;
	ch_mixer_info->out_channel_map[0] = PCM_CHANNEL_L;
	ch_mixer_info->out_channel_map[1] = PCM_CHANNEL_R;
	ch_mixer_info->in_channel_map[0] = PCM_CHANNEL_L;
	ch_mixer_info->channel_weight_coeff[0][0] = GAIN_Q14_FORMAT(1);
	ch_mixer_info->channel_weight_coeff[1][0] = GAIN_Q14_FORMAT(1);
	ch_mixer_info->reserved = 0;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *)&cvp_set_ch_mixer_info_cmd);
	if (ret < 0) {
		pr_err("%s: Failed to send VSS_ICOMMON_CMD_SET_PARAM_V2 %d\n",
		       __func__, ret);
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
				(v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));

	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s] handle = %d\n", __func__,
		       adsp_err_get_err_str(v->async_err), cvp_handle);
		ret = adsp_err_get_lnx_err_code(v->async_err);
		goto done;
	}
	ret = 0;
done:
	return ret;
}

static int voice_send_cvp_mfc_config_v2(struct voice_data *v)
{
	int ret;
	struct cvp_set_mfc_config_cmd_v2 cvp_set_mfc_config_cmd;
	void *apr_cvp;
	u16 cvp_handle;
	uint8_t ch_idx;
	struct vss_icommon_param_data_mfc_config_v2_t *cvp_config_param_data =
		&cvp_set_mfc_config_cmd.cvp_set_mfc_param_v2.param_data;
	struct vss_param_mfc_config_info_t *mfc_config_info =
		&cvp_config_param_data->mfc_config_info;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	apr_cvp  = common.apr_q6_cvp;
	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	cvp_handle = voice_get_cvp_handle(v);
	memset(&cvp_set_mfc_config_cmd, 0, sizeof(cvp_set_mfc_config_cmd));

	cvp_set_mfc_config_cmd.hdr.hdr_field =
		APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD, APR_HDR_LEN(APR_HDR_SIZE),
			      APR_PKT_VER);
	cvp_set_mfc_config_cmd.hdr.pkt_size =
		APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(cvp_set_mfc_config_cmd) - APR_HDR_SIZE);
	cvp_set_mfc_config_cmd.hdr.src_svc = 0;
	cvp_set_mfc_config_cmd.hdr.src_domain = APR_DOMAIN_APPS;
	cvp_set_mfc_config_cmd.hdr.src_port =
		voice_get_idx_for_session(v->session_id);
	cvp_set_mfc_config_cmd.hdr.dest_svc = 0;
	cvp_set_mfc_config_cmd.hdr.dest_domain = APR_DOMAIN_ADSP;
	cvp_set_mfc_config_cmd.hdr.dest_port = cvp_handle;
	cvp_set_mfc_config_cmd.hdr.token = 0;
	cvp_set_mfc_config_cmd.hdr.opcode = VSS_ICOMMON_CMD_SET_PARAM_V2;
	cvp_set_mfc_config_cmd.cvp_set_mfc_param_v2.mem_size =
		sizeof(struct vss_icommon_param_data_mfc_config_v2_t);

	cvp_config_param_data->module_id = AUDPROC_MODULE_ID_MFC;
	cvp_config_param_data->param_id =
		AUDPROC_PARAM_ID_MFC_OUTPUT_MEDIA_FORMAT;
	cvp_config_param_data->param_size =
		sizeof(struct vss_param_mfc_config_info_t);

	mfc_config_info->num_channels = v->dev_rx.no_of_channels;
	mfc_config_info->bits_per_sample = 16;
	mfc_config_info->sample_rate = v->dev_rx.sample_rate;

	/*
	 * Do not use memcpy here as channel_type in mfc_config structure is a
	 * uint16_t array while channel_mapping array of device is of uint8_t
	 */
	for (ch_idx = 0; ch_idx < VSS_NUM_CHANNELS_MAX; ch_idx++) {
		mfc_config_info->channel_type[ch_idx] =
					v->dev_rx.channel_mapping[ch_idx];
	}

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *)&cvp_set_mfc_config_cmd);
	if (ret < 0) {
		pr_err("%s: Failed to send VSS_ICOMMON_CMD_SET_PARAM_V2 %d\n",
		       __func__, ret);
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
				(v->cvp_state == CMD_STATUS_SUCCESS),
				msecs_to_jiffies(TIMEOUT_MS));

	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s] handle = %d\n", __func__,
		       adsp_err_get_err_str(v->async_err), cvp_handle);
		ret = adsp_err_get_lnx_err_code(v->async_err);
		goto done;
	}
	ret = 0;
done:
	return ret;
}

static int voice_send_cvp_mfc_config_cmd(struct voice_data *v)
{
	int ret = 0;

	if (common.cvp_version >= CVP_VERSION_2) {
		ret = voice_send_cvp_ch_mixer_info_v2(v);
		if (ret < 0)
			pr_warn("%s: Set channel mixer config failed err:%d",
				__func__, ret);

		ret = voice_send_cvp_mfc_config_v2(v);
		if (ret < 0)
			pr_warn("%s: Set MFC config failed err:%d",
				__func__, ret);
	} else {
		pr_warn("%s: CVP Version not supported\n", __func__);
		ret = -EINVAL;
	}

	return ret;
}

static int voice_get_avcs_version_per_service(uint32_t service_id)
{
	int ret = 0;
	size_t ver_size;
	struct avcs_fwk_ver_info *ver_info = NULL;

	if (service_id == AVCS_SERVICE_ID_ALL) {
		pr_err("%s: Invalid service id: %d", __func__,
		       AVCS_SERVICE_ID_ALL);
		return -EINVAL;
	}

	ver_size = sizeof(struct avcs_get_fwk_version) +
		   sizeof(struct avs_svc_api_info);
	ver_info = kzalloc(ver_size, GFP_KERNEL);
	if (ver_info == NULL)
		return -ENOMEM;

	ret = q6core_get_service_version(service_id, ver_info, ver_size);
	if (ret < 0)
		goto done;

	ret = ver_info->services[0].api_version;
	common.is_avcs_version_queried = true;
done:
	kfree(ver_info);
	return ret;
}

static void voice_mic_break_work_fn(struct work_struct *work)
{
	int ret = 0;
	char event[25] = "";
	struct voice_data *v = container_of(work, struct voice_data,
						voice_mic_break_work);

	snprintf(event, sizeof(event), "MIC_BREAK_STATUS=%s",
			v->mic_break_status ? "TRUE" : "FALSE");

	mutex_lock(&common.common_lock);
	ret = q6core_send_uevent(common.uevent_data, event);
	if (ret)
		pr_err("%s: Send UEvent %s failed :%d\n", __func__, event, ret);
	mutex_unlock(&common.common_lock);
}

static int voice_setup_vocproc(struct voice_data *v)
{
	struct module_instance_info mod_inst_info;
	int ret = 0;

	memset(&mod_inst_info, 0, sizeof(mod_inst_info));
	ret = voice_send_cvp_create_cmd(v);
	if (ret < 0) {
		pr_err("%s: CVP create failed err:%d\n", __func__, ret);
		goto fail;
	}

	if (common.is_avcs_version_queried == false)
		common.cvp_version = voice_get_avcs_version_per_service(
				     APRV2_IDS_SERVICE_ID_ADSP_CVP_V);

	if (common.cvp_version < 0) {
		pr_err("%s: Invalid CVP version %d\n",
		       __func__, common.cvp_version);
		ret = -EINVAL;
		goto fail;
	}
	pr_debug("%s: CVP Version %d\n", __func__, common.cvp_version);

	ret = voice_send_cvp_media_fmt_info_cmd(v);
	if (ret < 0) {
		pr_err("%s: Set media format info failed err:%d\n", __func__,
		       ret);
		goto fail;
	}

	ret = voice_send_cvp_topology_commit_cmd(v);
	if (ret < 0) {
		pr_err("%s: Set topology commit failed err:%d\n",
		       __func__, ret);
		goto fail;
	}

	/* Send MFC config only when the no of channels are more than 1 */
	if (v->dev_rx.no_of_channels > NUM_CHANNELS_MONO) {
		ret = voice_send_cvp_mfc_config_cmd(v);
		if (ret < 0) {
			pr_warn("%s: Set mfc config failed err:%d\n",
				__func__, ret);
		}
	}

	mod_inst_info.module_id = MODULE_ID_VOICE_MODULE_ST;
	mod_inst_info.instance_id = INSTANCE_ID_0;

	voice_send_cvs_register_cal_cmd(v);
	voice_send_cvp_register_dev_cfg_cmd(v);
	voice_send_cvp_register_cal_cmd(v);
	voice_send_cvp_register_vol_cal_cmd(v);

	/* enable vocproc */
	ret = voice_send_enable_vocproc_cmd(v);
	if (ret < 0)
		goto fail;

	/* attach vocproc */
	ret = voice_send_attach_vocproc_cmd(v);
	if (ret < 0)
		goto fail;

	/* send tty mode if tty device is used */
	voice_send_tty_mode_cmd(v);

	if (is_voip_session(v->session_id)) {
		ret = voice_send_mvm_cal_network_cmd(v);
		if (ret < 0)
			pr_err("%s: voice_send_mvm_cal_network_cmd: %d\n",
				__func__, ret);

		ret = voice_send_mvm_media_type_cmd(v);
		if (ret < 0)
			pr_err("%s: voice_send_mvm_media_type_cmd: %d\n",
				__func__, ret);

		voice_send_netid_timing_cmd(v);
	}

	if (v->st_enable && !v->tty_mode)
		voice_send_set_pp_enable_cmd(v, mod_inst_info, v->st_enable);
	/* Start in-call music delivery if this feature is enabled */
	if (v->music_info.play_enable)
		voice_cvs_start_playback(v);

	/* Start in-call recording if this feature is enabled */
	if (v->rec_info.rec_enable)
		voice_cvs_start_record(v, v->rec_info.rec_mode,
					v->rec_info.port_id);

	if (v->dtmf_rx_detect_en)
		voice_send_dtmf_rx_detection_cmd(v, v->dtmf_rx_detect_en);

	if (v->ecns_enable)
		voice_send_cvp_ecns_enable_cmd(v, v->ecns_module_id,
			v->ecns_enable);

	if (v->hd_enable)
		voice_send_hd_cmd(v, v->hd_enable);

	if (common.mic_break_enable)
		voice_send_mvm_event_class_cmd(v,
			VSS_INOTIFY_CMD_LISTEN_FOR_EVENT_CLASS,
			VSS_ICOMMON_EVENT_CLASS_VOICE_ACTIVITY_UPDATE);

	rtac_add_voice(voice_get_cvs_handle(v),
		voice_get_cvp_handle(v),
		v->dev_rx.port_id, v->dev_tx.port_id,
		v->dev_rx.dev_id, v->dev_tx.dev_id,
		v->session_id);

	return 0;

fail:
	return ret;
}

static int voice_send_cvp_device_channels_cmd(struct voice_data *v)
{
	int ret = 0;
	struct  cvp_set_dev_channels_cmd cvp_set_dev_channels_cmd;
	void *apr_cvp;
	u16 cvp_handle;

	if (!(voice_get_cvd_int_version(common.cvd_version) >=
	      CVD_INT_VERSION_2_2)) {
		pr_debug("%s CVD ver %s doesn't support send_device_channels cmd\n",
			 __func__, common.cvd_version);

		goto done;
	}

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	apr_cvp = common.apr_q6_cvp;
	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	cvp_handle = voice_get_cvp_handle(v);
	cvp_set_dev_channels_cmd.hdr.hdr_field =
			APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE),
			APR_PKT_VER);
	cvp_set_dev_channels_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
			sizeof(cvp_set_dev_channels_cmd) - APR_HDR_SIZE);
	cvp_set_dev_channels_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_set_dev_channels_cmd.hdr.dest_port = cvp_handle;
	cvp_set_dev_channels_cmd.hdr.token = 0;
	cvp_set_dev_channels_cmd.hdr.opcode =
				VSS_IVOCPROC_CMD_TOPOLOGY_SET_DEV_CHANNELS;
	cvp_set_dev_channels_cmd.cvp_set_channels.rx_num_channels =
				VSS_NUM_DEV_CHANNELS_1;
	cvp_set_dev_channels_cmd.cvp_set_channels.tx_num_channels =
				v->dev_tx.no_of_channels;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_set_dev_channels_cmd);
	if (ret < 0) {
		pr_err("%s: Fail in sending VSS_IVOCPROC_CMD_TOPOLOGY_SET_DEV_CHANNELS\n",
		       __func__);

		ret = -EINVAL;
		goto done;
	}

	ret = wait_event_timeout(v->cvp_wait,
				(v->cvp_state == CMD_STATUS_SUCCESS),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;
}

static int voice_send_cvp_media_fmt_info_cmd(struct voice_data *v)
{
	int ret = 0;

	if (common.cvp_version < CVP_VERSION_2)
		ret = voice_send_cvp_device_channels_cmd(v);
	else
		ret = voice_send_cvp_channel_info_cmd(v);

	if (ret < 0) {
		pr_err("%s: Set channel info failed err: %d\n", __func__,
		       ret);
		goto done;
	}

	if (voice_get_cvd_int_version(common.cvd_version) >=
	    CVD_INT_VERSION_2_3) {
		ret = voice_send_cvp_media_format_cmd(v, RX_PATH);
		if (ret < 0)
			goto done;

		ret = voice_send_cvp_media_format_cmd(v, TX_PATH);
		if (ret < 0)
			goto done;

		if (common.ec_ref_ext)
			ret = voice_send_cvp_media_format_cmd(v, EC_REF_PATH);
	}

done:
	return ret;
}

static int voice_send_cvp_media_format_cmd(struct voice_data *v,
					   uint32_t param_type)
{
	struct vss_param_endpoint_media_format_info media_fmt_info;
	struct param_hdr_v3 param_hdr;
	int ret = 0;

	memset(&media_fmt_info, 0, sizeof(media_fmt_info));
	memset(&param_hdr, 0, sizeof(param_hdr));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	param_hdr.module_id = VSS_MODULE_CVD_GENERIC;
	param_hdr.instance_id = INSTANCE_ID_0;
	param_hdr.param_size = sizeof(media_fmt_info);

	switch (param_type) {
	case RX_PATH:
		param_hdr.param_id = VSS_PARAM_RX_PORT_ENDPOINT_MEDIA_INFO;
		media_fmt_info.port_id = v->dev_rx.port_id;
		media_fmt_info.num_channels = v->dev_rx.no_of_channels;
		media_fmt_info.bits_per_sample = v->dev_rx.bits_per_sample;
		media_fmt_info.sample_rate = v->dev_rx.sample_rate;
		memcpy(&media_fmt_info.channel_mapping,
		       &v->dev_rx.channel_mapping, VSS_CHANNEL_MAPPING_SIZE);
		break;

	case TX_PATH:
		param_hdr.param_id = VSS_PARAM_TX_PORT_ENDPOINT_MEDIA_INFO;
		media_fmt_info.port_id = v->dev_tx.port_id;
		media_fmt_info.num_channels = v->dev_tx.no_of_channels;
		media_fmt_info.bits_per_sample = v->dev_tx.bits_per_sample;
		media_fmt_info.sample_rate = v->dev_tx.sample_rate;
		memcpy(&media_fmt_info.channel_mapping,
		       &v->dev_tx.channel_mapping, VSS_CHANNEL_MAPPING_SIZE);
		break;

	case EC_REF_PATH:
		param_hdr.param_id = VSS_PARAM_EC_REF_PORT_ENDPOINT_MEDIA_INFO;
		media_fmt_info.port_id = common.ec_media_fmt_info.port_id;
		media_fmt_info.num_channels =
			common.ec_media_fmt_info.num_channels;
		media_fmt_info.bits_per_sample =
			common.ec_media_fmt_info.bits_per_sample;
		media_fmt_info.sample_rate =
			common.ec_media_fmt_info.sample_rate;
		memcpy(&media_fmt_info.channel_mapping,
		       &common.ec_media_fmt_info.channel_mapping,
		       VSS_CHANNEL_MAPPING_SIZE);
		break;

	default:
		pr_err("%s: Invalid param type %d\n", __func__, param_type);
		ret = -EINVAL;
		goto done;
	}

	ret = voice_pack_and_set_cvp_param(v, param_hdr,
					   (u8 *) &media_fmt_info);
	if (ret)
		pr_err("%s: Failed to set media format params on CVP, err %d\n",
		       __func__, ret);

done:
	return ret;
}

static int voice_send_cvp_topology_commit_cmd(struct voice_data *v)
{
	int ret = 0;
	struct apr_hdr cvp_topology_commit_cmd;
	void *apr_cvp;
	u16 cvp_handle;

	if (!(voice_get_cvd_int_version(common.cvd_version) >=
	      CVD_INT_VERSION_2_2)) {
		pr_debug("%s CVD version string %s doesn't support this command\n",
			 __func__, common.cvd_version);

		goto done;
	}

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	apr_cvp = common.apr_q6_cvp;
	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	cvp_handle = voice_get_cvp_handle(v);
	cvp_topology_commit_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	cvp_topology_commit_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_topology_commit_cmd) - APR_HDR_SIZE);
	cvp_topology_commit_cmd.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_topology_commit_cmd.dest_port = cvp_handle;
	cvp_topology_commit_cmd.token = 0;
	cvp_topology_commit_cmd.opcode = VSS_IVOCPROC_CMD_TOPOLOGY_COMMIT;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_topology_commit_cmd);
	if (ret < 0) {
		pr_err("%s: Fail in sending VSS_IVOCPROC_CMD_TOPOLOGY_COMMIT\n",
		       __func__);

		ret = -EINVAL;
		goto done;
	}

	ret = wait_event_timeout(v->cvp_wait,
				(v->cvp_state == CMD_STATUS_SUCCESS),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -EINVAL;
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;
}

static int voice_send_enable_vocproc_cmd(struct voice_data *v)
{
	int ret = 0;
	struct apr_hdr cvp_enable_cmd;
	void *apr_cvp;
	u16 cvp_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvp = common.apr_q6_cvp;

	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);
		return -EINVAL;
	}
	cvp_handle = voice_get_cvp_handle(v);

	/* enable vocproc and wait for respose */
	cvp_enable_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	cvp_enable_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_enable_cmd) - APR_HDR_SIZE);
	pr_debug("cvp_enable_cmd pkt size = %d, cvp_handle=%d\n",
		cvp_enable_cmd.pkt_size, cvp_handle);
	cvp_enable_cmd.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_enable_cmd.dest_port = cvp_handle;
	cvp_enable_cmd.token = 0;
	cvp_enable_cmd.opcode = VSS_IVOCPROC_CMD_ENABLE;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_enable_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_IVOCPROC_CMD_ENABLE\n");
		goto fail;
	}
	ret = wait_event_timeout(v->cvp_wait,
				(v->cvp_state == CMD_STATUS_SUCCESS),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	return 0;
fail:
	return ret;
}

static int voice_send_mvm_cal_network_cmd(struct voice_data *v)
{
	struct vss_imvm_cmd_set_cal_network_t mvm_set_cal_network;
	int ret = 0;
	void *apr_mvm;
	u16 mvm_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;

	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		return -EINVAL;
	}
	mvm_handle = voice_get_mvm_handle(v);

	mvm_set_cal_network.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	mvm_set_cal_network.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_set_cal_network) - APR_HDR_SIZE);
	mvm_set_cal_network.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	mvm_set_cal_network.hdr.dest_port = mvm_handle;
	mvm_set_cal_network.hdr.token = 0;
	mvm_set_cal_network.hdr.opcode = VSS_IMVM_CMD_SET_CAL_NETWORK;
	mvm_set_cal_network.network_id = VSS_ICOMMON_CAL_NETWORK_ID_NONE;

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_set_cal_network);
	if (ret < 0) {
		pr_err("%s: Error %d sending SET_NETWORK\n", __func__, ret);
		goto fail;
	}

	ret = wait_event_timeout(v->mvm_wait,
				(v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout %d\n", __func__, ret);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}
	return 0;
fail:
	return ret;
}

static int voice_send_netid_timing_cmd(struct voice_data *v)
{
	int ret = 0;
	void *apr_mvm;
	u16 mvm_handle;
	struct mvm_set_network_cmd mvm_set_network;
	struct mvm_set_voice_timing_cmd mvm_set_voice_timing;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;

	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		return -EINVAL;
	}
	mvm_handle = voice_get_mvm_handle(v);

	ret = voice_config_cvs_vocoder(v);
	if (ret < 0) {
		pr_err("%s: Error %d configuring CVS voc",
					__func__, ret);
		goto fail;
	}
	/* Set network ID. */
	pr_debug("Setting network ID %x\n", common.mvs_info.network_type);

	mvm_set_network.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	mvm_set_network.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(mvm_set_network) - APR_HDR_SIZE);
	mvm_set_network.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	mvm_set_network.hdr.dest_port = mvm_handle;
	mvm_set_network.hdr.token = 0;
	mvm_set_network.hdr.opcode = VSS_IMVM_CMD_SET_CAL_NETWORK;
	mvm_set_network.network.network_id = common.mvs_info.network_type;

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_set_network);
	if (ret < 0) {
		pr_err("%s: Error %d sending SET_NETWORK\n", __func__, ret);
		goto fail;
	}

	ret = wait_event_timeout(v->mvm_wait,
				(v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	/* Set voice timing. */
	 pr_debug("Setting voice timing\n");

	mvm_set_voice_timing.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	mvm_set_voice_timing.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
						sizeof(mvm_set_voice_timing) -
						APR_HDR_SIZE);
	mvm_set_voice_timing.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	mvm_set_voice_timing.hdr.dest_port = mvm_handle;
	mvm_set_voice_timing.hdr.token = 0;
	mvm_set_voice_timing.hdr.opcode = VSS_ICOMMON_CMD_SET_VOICE_TIMING;
	mvm_set_voice_timing.timing.mode = 0;
	mvm_set_voice_timing.timing.enc_offset = 8000;
	mvm_set_voice_timing.timing.dec_req_offset = 3300;
	mvm_set_voice_timing.timing.dec_offset = 8300;

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;

	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_set_voice_timing);
	if (ret < 0) {
		pr_err("%s: Error %d sending SET_TIMING\n", __func__, ret);
		goto fail;
	}

	ret = wait_event_timeout(v->mvm_wait,
				(v->mvm_state == CMD_STATUS_SUCCESS),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	return 0;
fail:
	return ret;
}

static int voice_send_attach_vocproc_cmd(struct voice_data *v)
{
	int ret = 0;
	struct mvm_attach_vocproc_cmd mvm_a_vocproc_cmd;
	void *apr_mvm;
	u16 mvm_handle, cvp_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;

	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		return -EINVAL;
	}
	mvm_handle = voice_get_mvm_handle(v);
	cvp_handle = voice_get_cvp_handle(v);

	/* attach vocproc and wait for response */
	mvm_a_vocproc_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	mvm_a_vocproc_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_a_vocproc_cmd) - APR_HDR_SIZE);
	pr_debug("send mvm_a_vocproc_cmd pkt size = %d\n",
		mvm_a_vocproc_cmd.hdr.pkt_size);
	mvm_a_vocproc_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	mvm_a_vocproc_cmd.hdr.dest_port = mvm_handle;
	mvm_a_vocproc_cmd.hdr.token = 0;
	mvm_a_vocproc_cmd.hdr.opcode = VSS_IMVM_CMD_ATTACH_VOCPROC;
	mvm_a_vocproc_cmd.mvm_attach_cvp_handle.handle = cvp_handle;

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_a_vocproc_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_IMVM_CMD_ATTACH_VOCPROC\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	return 0;
fail:
	return ret;
}

static void voc_update_session_params(struct voice_data *v)
{
	/* reset LCH mode */
	v->lch_mode = 0;

	/* clear disable topology setting */
	v->disable_topology = false;

	/* clear mute setting */
	v->dev_rx.dev_mute =  common.default_mute_val;
	v->dev_tx.dev_mute =  common.default_mute_val;
	v->stream_rx.stream_mute = common.default_mute_val;
	v->stream_tx.stream_mute = common.default_mute_val;
}

static int voice_destroy_vocproc(struct voice_data *v)
{
	struct mvm_detach_vocproc_cmd mvm_d_vocproc_cmd;
	struct apr_hdr cvp_destroy_session_cmd;
	struct module_instance_info mod_inst_info;
	int ret = 0;
	void *apr_mvm, *apr_cvp;
	u16 mvm_handle, cvp_handle;

	memset(&mod_inst_info, 0, sizeof(mod_inst_info));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;
	apr_cvp = common.apr_q6_cvp;

	if (!apr_mvm || !apr_cvp) {
		pr_err("%s: apr_mvm or apr_cvp is NULL.\n", __func__);
		return -EINVAL;
	}
	mvm_handle = voice_get_mvm_handle(v);
	cvp_handle = voice_get_cvp_handle(v);

	mod_inst_info.module_id = MODULE_ID_VOICE_MODULE_ST;
	mod_inst_info.instance_id = INSTANCE_ID_0;

	/* disable slowtalk if st_enable is set */
	if (v->st_enable)
		voice_send_set_pp_enable_cmd(v, mod_inst_info, 0);

	/* Disable HD Voice if hd_enable is set */
	if (v->hd_enable)
		voice_send_hd_cmd(v, 0);

	/* stop playback or recording */
	v->music_info.force = 1;
	voice_cvs_stop_playback(v);
	voice_cvs_stop_record(v);
	/* If voice call is active during VoLTE, SRVCC happens.
	 * Start recording on voice session if recording started during VoLTE.
	 */
	if (is_volte_session(v->session_id) &&
	    ((common.voice[VOC_PATH_PASSIVE].voc_state == VOC_RUN) ||
	     (common.voice[VOC_PATH_PASSIVE].voc_state == VOC_CHANGE))) {
		if (v->rec_info.rec_enable) {
			voice_cvs_start_record(
				&common.voice[VOC_PATH_PASSIVE],
				v->rec_info.rec_mode,
				v->rec_info.port_id);
			common.srvcc_rec_flag = true;

			pr_debug("%s: switch recording, srvcc_rec_flag %d\n",
				 __func__, common.srvcc_rec_flag);
		}
	}

	/* send stop voice cmd */
	voice_send_stop_voice_cmd(v);

	/* send stop dtmf detecton cmd */
	if (v->dtmf_rx_detect_en)
		voice_send_dtmf_rx_detection_cmd(v, 0);

	if (v->ecns_enable)
		voice_send_cvp_ecns_enable_cmd(v, v->ecns_module_id, 0);

	/* detach VOCPROC and wait for response from mvm */
	mvm_d_vocproc_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	mvm_d_vocproc_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mvm_d_vocproc_cmd) - APR_HDR_SIZE);
	pr_debug("mvm_d_vocproc_cmd  pkt size = %d\n",
		mvm_d_vocproc_cmd.hdr.pkt_size);
	mvm_d_vocproc_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	mvm_d_vocproc_cmd.hdr.dest_port = mvm_handle;
	mvm_d_vocproc_cmd.hdr.token = 0;
	mvm_d_vocproc_cmd.hdr.opcode = VSS_IMVM_CMD_DETACH_VOCPROC;
	mvm_d_vocproc_cmd.mvm_detach_cvp_handle.handle = cvp_handle;

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mvm_d_vocproc_cmd);
	if (ret < 0) {
		pr_err("Fail in sending VSS_IMVM_CMD_DETACH_VOCPROC\n");
		goto fail;
	}
	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	voice_send_cvp_deregister_vol_cal_cmd(v);
	mutex_lock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
	voice_send_cvp_deregister_cal_cmd(v);
	mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
	voice_send_cvp_deregister_dev_cfg_cmd(v);
	voice_send_cvs_deregister_cal_cmd(v);

	/* Unload topology modules */
	voice_unload_topo_modules();

	if (common.mic_break_enable)
		voice_send_mvm_event_class_cmd(v,
			VSS_INOTIFY_CMD_CANCEL_EVENT_CLASS,
			VSS_ICOMMON_EVENT_CLASS_VOICE_ACTIVITY_UPDATE);

	/* destrop cvp session */
	cvp_destroy_session_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	cvp_destroy_session_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_destroy_session_cmd) - APR_HDR_SIZE);
	pr_debug("cvp_destroy_session_cmd pkt size = %d\n",
		cvp_destroy_session_cmd.pkt_size);
	cvp_destroy_session_cmd.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_destroy_session_cmd.dest_port = cvp_handle;
	cvp_destroy_session_cmd.token = 0;
	cvp_destroy_session_cmd.opcode = APRV2_IBASIC_CMD_DESTROY_SESSION;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_destroy_session_cmd);
	if (ret < 0) {
		pr_err("Fail in sending APRV2_IBASIC_CMD_DESTROY_SESSION\n");
		goto fail;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	rtac_remove_voice(voice_get_cvs_handle(v));
	cvp_handle = 0;
	voice_set_cvp_handle(v, cvp_handle);
	return 0;
fail:
	return ret;
}

static int voice_send_mvm_unmap_memory_physical_cmd(struct voice_data *v,
						    uint32_t mem_handle)
{
	struct vss_imemory_cmd_unmap_t mem_unmap;
	int ret = 0;
	void *apr_mvm;
	u16 mvm_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_mvm = common.apr_q6_mvm;

	if (!apr_mvm) {
		pr_err("%s: apr_mvm is NULL.\n", __func__);
		return -EINVAL;
	}
	mvm_handle = voice_get_mvm_handle(v);

	mem_unmap.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	mem_unmap.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(mem_unmap) - APR_HDR_SIZE);
	mem_unmap.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	mem_unmap.hdr.dest_port = mvm_handle;
	mem_unmap.hdr.token = 0;
	mem_unmap.hdr.opcode = VSS_IMEMORY_CMD_UNMAP;
	mem_unmap.mem_handle = mem_handle;

	pr_debug("%s: mem_handle: 0x%x\n", __func__, mem_unmap.mem_handle);

	v->mvm_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_mvm, (uint32_t *) &mem_unmap);
	if (ret < 0) {
		pr_err("mem_unmap op[0x%x]ret[%d]\n",
			mem_unmap.hdr.opcode, ret);
		goto fail;
	}

	ret = wait_event_timeout(v->mvm_wait,
				 (v->mvm_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout %d\n", __func__, ret);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}
	return 0;

fail:
	return ret;
}

static int voice_send_cvs_packet_exchange_config_cmd(struct voice_data *v)
{
	struct vss_istream_cmd_set_oob_packet_exchange_config_t
						 packet_exchange_config_pkt;
	int ret = 0;
	void *apr_cvs;
	u16 cvs_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}

	apr_cvs = common.apr_q6_cvs;

	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);
		return -EINVAL;
	}
	cvs_handle = voice_get_cvs_handle(v);

	packet_exchange_config_pkt.hdr.hdr_field = APR_HDR_FIELD(
						APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	packet_exchange_config_pkt.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(packet_exchange_config_pkt) -
					 APR_HDR_SIZE);
	packet_exchange_config_pkt.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	packet_exchange_config_pkt.hdr.dest_port = cvs_handle;
	packet_exchange_config_pkt.hdr.token = 0;
	packet_exchange_config_pkt.hdr.opcode =
			 VSS_ISTREAM_CMD_SET_OOB_PACKET_EXCHANGE_CONFIG;
	packet_exchange_config_pkt.mem_handle = v->shmem_info.mem_handle;
	/* dec buffer address */
	packet_exchange_config_pkt.dec_buf_addr_lsw =
		lower_32_bits(v->shmem_info.sh_buf.buf[0].phys);
	packet_exchange_config_pkt.dec_buf_addr_msw =
		msm_audio_populate_upper_32_bits(
					v->shmem_info.sh_buf.buf[0].phys);
	packet_exchange_config_pkt.dec_buf_size = 4096;
	/* enc buffer address */
	packet_exchange_config_pkt.enc_buf_addr_lsw =
		lower_32_bits(v->shmem_info.sh_buf.buf[1].phys);
	packet_exchange_config_pkt.enc_buf_addr_msw =
		msm_audio_populate_upper_32_bits(
					v->shmem_info.sh_buf.buf[1].phys);
	packet_exchange_config_pkt.enc_buf_size = 4096;

	pr_debug("%s: dec buf add: lsw %0x msw %0x, size %d, enc buf add: lsw %0x msw %0x, size %d\n",
		__func__,
		packet_exchange_config_pkt.dec_buf_addr_lsw,
		packet_exchange_config_pkt.dec_buf_addr_msw,
		packet_exchange_config_pkt.dec_buf_size,
		packet_exchange_config_pkt.enc_buf_addr_lsw,
		packet_exchange_config_pkt.enc_buf_addr_msw,
		packet_exchange_config_pkt.enc_buf_size);

	v->cvs_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvs, (uint32_t *) &packet_exchange_config_pkt);
	if (ret < 0) {
		pr_err("Failed to send packet exchange config cmd %d\n", ret);
		goto fail;
	}

	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret)
		pr_err("%s: wait_event timeout %d\n", __func__, ret);

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	return 0;
fail:
	return ret;
}

static int voice_send_cvs_data_exchange_mode_cmd(struct voice_data *v)
{
	struct vss_istream_cmd_set_packet_exchange_mode_t data_exchange_pkt;
	int ret = 0;
	void *apr_cvs;
	u16 cvs_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvs = common.apr_q6_cvs;

	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);
		return -EINVAL;
	}
	cvs_handle = voice_get_cvs_handle(v);

	data_exchange_pkt.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	data_exchange_pkt.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(data_exchange_pkt) - APR_HDR_SIZE);
	data_exchange_pkt.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	data_exchange_pkt.hdr.dest_port = cvs_handle;
	data_exchange_pkt.hdr.token = 0;
	data_exchange_pkt.hdr.opcode = VSS_ISTREAM_CMD_SET_PACKET_EXCHANGE_MODE;
	data_exchange_pkt.mode = VSS_ISTREAM_PACKET_EXCHANGE_MODE_OUT_OF_BAND;

	v->cvs_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvs, (uint32_t *) &data_exchange_pkt);
	if (ret < 0) {
		pr_err("Failed to send data exchange mode %d\n", ret);
		goto fail;
	}

	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret)
		pr_err("%s: wait_event timeout %d\n", __func__, ret);

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}
	return 0;
fail:
	return ret;
}

static int voice_send_stream_mute_cmd(struct voice_data *v, uint16_t direction,
				     uint16_t mute_flag, uint32_t ramp_duration)
{
	struct cvs_set_mute_cmd cvs_mute_cmd;
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto fail;
	}

	if (!common.apr_q6_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);
		ret = -EINVAL;
		goto fail;
	}

	/* send mute/unmute to cvs */
	cvs_mute_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	cvs_mute_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(cvs_mute_cmd) - APR_HDR_SIZE);
	cvs_mute_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvs_mute_cmd.hdr.dest_port = voice_get_cvs_handle(v);
	cvs_mute_cmd.hdr.token = 0;
	cvs_mute_cmd.hdr.opcode = VSS_IVOLUME_CMD_MUTE_V2;
	cvs_mute_cmd.cvs_set_mute.direction = direction;
	cvs_mute_cmd.cvs_set_mute.mute_flag = mute_flag;
	cvs_mute_cmd.cvs_set_mute.ramp_duration_ms = ramp_duration;

	v->cvs_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_cvs, (uint32_t *) &cvs_mute_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d sending stream mute\n", __func__, ret);

		goto fail;
	}
	ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	return 0;

fail:
	return ret;
}

static int voice_send_device_mute_cmd(struct voice_data *v, uint16_t direction,
				     uint16_t mute_flag, uint32_t ramp_duration)
{
	struct cvp_set_mute_cmd cvp_mute_cmd;
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto fail;
	}

	if (!common.apr_q6_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);
		ret = -EINVAL;
		goto fail;
	}

	cvp_mute_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	cvp_mute_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(cvp_mute_cmd) - APR_HDR_SIZE);
	cvp_mute_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_mute_cmd.hdr.dest_port = voice_get_cvp_handle(v);
	cvp_mute_cmd.hdr.token = 0;
	cvp_mute_cmd.hdr.opcode = VSS_IVOLUME_CMD_MUTE_V2;
	cvp_mute_cmd.cvp_set_mute.direction = direction;
	cvp_mute_cmd.cvp_set_mute.mute_flag = mute_flag;
	cvp_mute_cmd.cvp_set_mute.ramp_duration_ms = ramp_duration;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(common.apr_q6_cvp, (uint32_t *) &cvp_mute_cmd);
	if (ret < 0) {
		pr_err("%s: Error %d sending rx device cmd\n", __func__, ret);

		goto fail;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: Command timeout\n", __func__);
		goto fail;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto fail;
	}

	return 0;

fail:
	return ret;
}

static int voice_send_vol_step_cmd(struct voice_data *v)
{
	struct cvp_set_rx_volume_step_cmd cvp_vol_step_cmd;
	int ret = 0;
	void *apr_cvp;
	u16 cvp_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvp = common.apr_q6_cvp;

	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);
		return -EINVAL;
	}
	cvp_handle = voice_get_cvp_handle(v);

	/* send volume index to cvp */
	cvp_vol_step_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
						APR_HDR_LEN(APR_HDR_SIZE),
						APR_PKT_VER);
	cvp_vol_step_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvp_vol_step_cmd) - APR_HDR_SIZE);
	cvp_vol_step_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_vol_step_cmd.hdr.dest_port = cvp_handle;
	cvp_vol_step_cmd.hdr.token = 0;
	cvp_vol_step_cmd.hdr.opcode = VSS_IVOLUME_CMD_SET_STEP;
	cvp_vol_step_cmd.cvp_set_vol_step.direction = VSS_IVOLUME_DIRECTION_RX;
	cvp_vol_step_cmd.cvp_set_vol_step.value = v->dev_rx.volume_step_value;
	cvp_vol_step_cmd.cvp_set_vol_step.ramp_duration_ms =
					v->dev_rx.volume_ramp_duration_ms;
	 pr_debug("%s step_value:%d, ramp_duration_ms:%d",
			__func__,
			cvp_vol_step_cmd.cvp_set_vol_step.value,
			cvp_vol_step_cmd.cvp_set_vol_step.ramp_duration_ms);

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_vol_step_cmd);
	if (ret < 0) {
		pr_err("Fail in sending RX VOL step\n");
		return -EINVAL;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		return -EINVAL;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		return ret;
	}
	return 0;
}

static int voice_cvs_start_record(struct voice_data *v, uint32_t rec_mode,
					uint32_t port_id)
{
	int ret = 0;
	void *apr_cvs;
	u16 cvs_handle;

	struct cvs_start_record_cmd cvs_start_record;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvs = common.apr_q6_cvs;

	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);
		return -EINVAL;
	}

	cvs_handle = voice_get_cvs_handle(v);

	if (!v->rec_info.recording) {
		cvs_start_record.hdr.hdr_field = APR_HDR_FIELD(
					APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE),
					APR_PKT_VER);
		cvs_start_record.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				  sizeof(cvs_start_record) - APR_HDR_SIZE);
		cvs_start_record.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
		cvs_start_record.hdr.dest_port = cvs_handle;
		cvs_start_record.hdr.token = 0;
		cvs_start_record.hdr.opcode = VSS_IRECORD_CMD_START;

		cvs_start_record.rec_mode.port_id =
					VSS_IRECORD_PORT_ID_DEFAULT;
		if (rec_mode == VOC_REC_UPLINK) {
			cvs_start_record.rec_mode.rx_tap_point =
					VSS_IRECORD_TAP_POINT_NONE;
			cvs_start_record.rec_mode.tx_tap_point =
					VSS_IRECORD_TAP_POINT_STREAM_END;
		} else if (rec_mode == VOC_REC_DOWNLINK) {
			cvs_start_record.rec_mode.rx_tap_point =
					VSS_IRECORD_TAP_POINT_STREAM_END;
			cvs_start_record.rec_mode.tx_tap_point =
					VSS_IRECORD_TAP_POINT_NONE;
		} else if (rec_mode == VOC_REC_BOTH) {
			cvs_start_record.rec_mode.rx_tap_point =
					VSS_IRECORD_TAP_POINT_STREAM_END;
			cvs_start_record.rec_mode.tx_tap_point =
					VSS_IRECORD_TAP_POINT_STREAM_END;
			if (common.rec_channel_count ==
					NUM_CHANNELS_STEREO) {
			/*
			 * if channel count is not stereo,
			 * then default port_id and mode
			 * (mono) will be used
			 */
				cvs_start_record.rec_mode.mode =
					VSS_IRECORD_MODE_TX_RX_STEREO;
				cvs_start_record.rec_mode.port_id =
					port_id;
			}
		} else {
			pr_err("%s: Invalid in-call rec_mode %d\n", __func__,
				rec_mode);

			ret = -EINVAL;
			goto fail;
		}

		v->cvs_state = CMD_STATUS_FAIL;
		v->async_err = 0;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_start_record);
		if (ret < 0) {
			pr_err("%s: Error %d sending START_RECORD\n", __func__,
				ret);

			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));

		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}
		if (v->async_err > 0) {
			pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
			ret = adsp_err_get_lnx_err_code(
					v->async_err);
			goto fail;
		}
		v->rec_info.recording = 1;
	} else {
		pr_debug("%s: Start record already sent\n", __func__);
	}

	return 0;

fail:
	return ret;
}

static int voice_cvs_stop_record(struct voice_data *v)
{
	int ret = 0;
	void *apr_cvs;
	u16 cvs_handle;
	struct apr_hdr cvs_stop_record;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvs = common.apr_q6_cvs;

	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);
		return -EINVAL;
	}

	cvs_handle = voice_get_cvs_handle(v);

	if (v->rec_info.recording) {
		cvs_stop_record.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				  APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		cvs_stop_record.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				  sizeof(cvs_stop_record) - APR_HDR_SIZE);
		cvs_stop_record.src_port =
				voice_get_idx_for_session(v->session_id);
		cvs_stop_record.dest_port = cvs_handle;
		cvs_stop_record.token = 0;
		cvs_stop_record.opcode = VSS_IRECORD_CMD_STOP;

		v->cvs_state = CMD_STATUS_FAIL;
		v->async_err = 0;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_stop_record);
		if (ret < 0) {
			pr_err("%s: Error %d sending STOP_RECORD\n",
				__func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}
		if (v->async_err > 0) {
			pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
			ret = adsp_err_get_lnx_err_code(
					v->async_err);
			goto fail;
		}
		v->rec_info.recording = 0;
	} else {
		pr_debug("%s: Stop record already sent\n", __func__);
	}

	return 0;

fail:
	return ret;
}

/**
 * voc_start_record -
 *       command to set record for voice session
 *
 * @port_id: Pseudo Port ID for record data
 * @set: Enable or Disable for record start/stop
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_start_record(uint32_t port_id, uint32_t set, uint32_t session_id)
{
	int ret = 0;
	int rec_mode = 0;
	u16 cvs_handle;
	int rec_set = 0;
	struct voice_session_itr itr;
	struct voice_data *v = NULL;

	/* check if session_id is valid */
	if (!voice_is_valid_session_id(session_id)) {
		pr_err("%s: Invalid session id:%u\n", __func__,
		       session_id);

		return -EINVAL;
	}

	voice_itr_init(&itr, session_id);
	pr_debug("%s: session_id:%u\n", __func__, session_id);

	while (voice_itr_get_next_session(&itr, &v)) {
		if (v == NULL) {
			pr_err("%s: v is NULL, sessionid:%u\n", __func__,
				session_id);

			break;
		}
		pr_debug("%s: port_id: %d, set: %d, v: %pK\n",
			 __func__, port_id, set, v);

		mutex_lock(&v->lock);
		rec_mode = v->rec_info.rec_mode;
		v->rec_info.port_id = port_id;
		rec_set = set;
		if (set) {
			if ((v->rec_route_state.ul_flag != 0) &&
				(v->rec_route_state.dl_flag != 0)) {
				pr_debug("%s: rec mode already set.\n",
					__func__);

				mutex_unlock(&v->lock);
				continue;
			}

			if (port_id == VOICE_RECORD_TX) {
				if ((v->rec_route_state.ul_flag == 0)
				&& (v->rec_route_state.dl_flag == 0)) {
					rec_mode = VOC_REC_UPLINK;
					v->rec_route_state.ul_flag = 1;
				} else if ((v->rec_route_state.ul_flag == 0)
					&& (v->rec_route_state.dl_flag != 0)) {
					voice_cvs_stop_record(v);
					rec_mode = VOC_REC_BOTH;
					v->rec_route_state.ul_flag = 1;
				}
			} else if (port_id == VOICE_RECORD_RX) {
				if ((v->rec_route_state.ul_flag == 0)
					&& (v->rec_route_state.dl_flag == 0)) {
					rec_mode = VOC_REC_DOWNLINK;
					v->rec_route_state.dl_flag = 1;
				} else if ((v->rec_route_state.ul_flag != 0)
					&& (v->rec_route_state.dl_flag == 0)) {
					voice_cvs_stop_record(v);
					rec_mode = VOC_REC_BOTH;
					v->rec_route_state.dl_flag = 1;
				}
			}
			rec_set = 1;
		} else {
			if ((v->rec_route_state.ul_flag == 0) &&
				(v->rec_route_state.dl_flag == 0)) {
				pr_debug("%s: rec already stops.\n",
					__func__);
				mutex_unlock(&v->lock);
				continue;
			}

			if (port_id == VOICE_RECORD_TX) {
				if ((v->rec_route_state.ul_flag != 0)
					&& (v->rec_route_state.dl_flag == 0)) {
					v->rec_route_state.ul_flag = 0;
					rec_set = 0;
				} else if ((v->rec_route_state.ul_flag != 0)
					&& (v->rec_route_state.dl_flag != 0)) {
					voice_cvs_stop_record(v);
					v->rec_route_state.ul_flag = 0;
					rec_mode = VOC_REC_DOWNLINK;
					rec_set = 1;
				}
			} else if (port_id == VOICE_RECORD_RX) {
				if ((v->rec_route_state.ul_flag == 0)
					&& (v->rec_route_state.dl_flag != 0)) {
					v->rec_route_state.dl_flag = 0;
					rec_set = 0;
				} else if ((v->rec_route_state.ul_flag != 0)
					&& (v->rec_route_state.dl_flag != 0)) {
					voice_cvs_stop_record(v);
					v->rec_route_state.dl_flag = 0;
					rec_mode = VOC_REC_UPLINK;
					rec_set = 1;
				}
			}
		}
		pr_debug("%s: mode =%d, set =%d\n", __func__,
			 rec_mode, rec_set);
		cvs_handle = voice_get_cvs_handle(v);

		if (cvs_handle != 0) {
			if (rec_set)
				ret = voice_cvs_start_record(v, rec_mode,
						port_id);
			else
				ret = voice_cvs_stop_record(v);
		}

		/* During SRVCC, recording will switch from VoLTE session to
		 * voice session.
		 * Then stop recording, need to stop recording on voice session.
		 */
		if ((!rec_set) && common.srvcc_rec_flag) {
			pr_debug("%s, srvcc_rec_flag:%d\n",  __func__,
				 common.srvcc_rec_flag);

			voice_cvs_stop_record(&common.voice[VOC_PATH_PASSIVE]);
			common.srvcc_rec_flag = false;
		}

		/* Cache the value */
		v->rec_info.rec_enable = rec_set;
		v->rec_info.rec_mode = rec_mode;

		mutex_unlock(&v->lock);
	}

	return ret;
}
EXPORT_SYMBOL(voc_start_record);

static int voice_cvs_start_playback(struct voice_data *v)
{
	int ret = 0;
	struct cvs_start_playback_cmd cvs_start_playback;
	void *apr_cvs;
	u16 cvs_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvs = common.apr_q6_cvs;

	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);
		return -EINVAL;
	}

	cvs_handle = voice_get_cvs_handle(v);

	if (!v->music_info.playing && v->music_info.count) {
		cvs_start_playback.hdr.hdr_field = APR_HDR_FIELD(
					APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE),
					APR_PKT_VER);
		cvs_start_playback.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvs_start_playback) - APR_HDR_SIZE);
		cvs_start_playback.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
		cvs_start_playback.hdr.dest_port = cvs_handle;
		cvs_start_playback.hdr.token = 0;
		cvs_start_playback.hdr.opcode = VSS_IPLAYBACK_CMD_START;
		cvs_start_playback.playback_mode.port_id =
						v->music_info.port_id;

		v->cvs_state = CMD_STATUS_FAIL;
		v->async_err = 0;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_start_playback);

		if (ret < 0) {
			pr_err("%s: Error %d sending START_PLAYBACK\n",
				__func__, ret);

			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
				 (v->cvs_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}
		if (v->async_err > 0) {
			pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
			ret = adsp_err_get_lnx_err_code(
					v->async_err);
			goto fail;
		}

		v->music_info.playing = 1;
	} else {
		pr_debug("%s: Start playback already sent\n", __func__);
	}

	return 0;

fail:
	return ret;
}

static int voice_cvs_stop_playback(struct voice_data *v)
{
	 int ret = 0;
	 struct apr_hdr cvs_stop_playback;
	 void *apr_cvs;
	 u16 cvs_handle;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvs = common.apr_q6_cvs;

	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL.\n", __func__);
		return -EINVAL;
	}

	cvs_handle = voice_get_cvs_handle(v);

	if (v->music_info.playing && ((!v->music_info.count) ||
						(v->music_info.force))) {
		cvs_stop_playback.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		cvs_stop_playback.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(cvs_stop_playback) - APR_HDR_SIZE);
		cvs_stop_playback.src_port =
				voice_get_idx_for_session(v->session_id);
		cvs_stop_playback.dest_port = cvs_handle;
		cvs_stop_playback.token = 0;

		cvs_stop_playback.opcode = VSS_IPLAYBACK_CMD_STOP;

		v->cvs_state = CMD_STATUS_FAIL;
		v->async_err = 0;

		ret = apr_send_pkt(apr_cvs, (uint32_t *) &cvs_stop_playback);
		if (ret < 0) {
			pr_err("%s: Error %d sending STOP_PLAYBACK\n",
			       __func__, ret);


			goto fail;
		}

		ret = wait_event_timeout(v->cvs_wait,
					 (v->cvs_state == CMD_STATUS_SUCCESS),
					 msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout\n", __func__);

			goto fail;
		}
		if (v->async_err > 0) {
			pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
			ret = adsp_err_get_lnx_err_code(
					v->async_err);
			goto fail;
		}

		v->music_info.playing = 0;
		v->music_info.force = 0;
	} else {
		pr_debug("%s: Stop playback already sent\n", __func__);
	}

	return 0;

fail:
	return ret;
}

static int voc_lch_ops(struct voice_data *v, enum voice_lch_mode lch_mode)
{
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	switch (lch_mode) {
	case VOICE_LCH_START:

		ret = voc_end_voice_call(v->session_id);
		if (ret < 0)
			pr_err("%s: voice call end failed %d\n",
				__func__, ret);
		break;
	case VOICE_LCH_STOP:

		ret = voc_start_voice_call(v->session_id);
		if (ret < 0) {
			pr_err("%s: voice call start failed %d\n",
				__func__, ret);
			goto done;
		}
		break;
	default:
		pr_err("%s: Invalid LCH mode: %d\n",
			__func__, v->lch_mode);
		break;
	}
done:
	return ret;
}

/**
 * voc_start_playback -
 *       command to set playback for voice session
 *
 * @set: Enable or Disable for playback start/stop
 * @port_id: Pseudo Port ID for playback data
 *
 * Returns 0 on success or error on failure
 */
int voc_start_playback(uint32_t set, uint16_t port_id)
{
	struct voice_data *v = NULL;
	int ret = 0;
	struct voice_session_itr itr;
	u16 cvs_handle;

	pr_debug("%s port_id = %#x set = %d", __func__, port_id, set);

	voice_itr_init(&itr, ALL_SESSION_VSID);
	while (voice_itr_get_next_session(&itr, &v)) {
		if ((v != NULL) &&
		    (((port_id == VOICE_PLAYBACK_TX) &&
		       is_sub1_vsid(v->session_id)) ||
		     ((port_id == VOICE2_PLAYBACK_TX) &&
		       is_sub2_vsid(v->session_id)))) {

			mutex_lock(&v->lock);
			v->music_info.port_id = port_id;
			v->music_info.play_enable = set;
			if (set)
				v->music_info.count++;
			else
				v->music_info.count--;
			pr_debug("%s: music_info count=%d\n", __func__,
				 v->music_info.count);

			cvs_handle = voice_get_cvs_handle(v);
			if (cvs_handle != 0) {
				if (set)
					ret = voice_cvs_start_playback(v);
				else
					ret = voice_cvs_stop_playback(v);
			}
			mutex_unlock(&v->lock);
		} else {
			pr_err("%s: Invalid session\n", __func__);
		}
	}

	return ret;
}
EXPORT_SYMBOL(voc_start_playback);

/**
 * voc_disable_topology -
 *       disable topology for voice session
 *
 * @session_id: voice session ID to send this command
 * @disable: disable value
 *
 * Returns 0 on success or error on failure
 */
int voc_disable_topology(uint32_t session_id, uint32_t disable)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);

		return -EINVAL;
	}

	mutex_lock(&v->lock);

	v->disable_topology = disable;

	mutex_unlock(&v->lock);

	return ret;
}
EXPORT_SYMBOL(voc_disable_topology);

/**
 * voc_set_incall_capture_channel_config -
 *		command to set channel count for record
 *
 * @channel_count: number of channels
 *
 */
void voc_set_incall_capture_channel_config(int channel_count)
{
	common.rec_channel_count = channel_count;
}
EXPORT_SYMBOL(voc_set_incall_capture_channel_config);

/**
 * voc_get_incall_capture_channel_config -
 *		command to get channel count for record
 *
 * Returns number of channels configured for record
 */
int voc_get_incall_capture_channel_config(void)
{
	return common.rec_channel_count;
}
EXPORT_SYMBOL(voc_get_incall_capture_channel_config);

static int voice_set_packet_exchange_mode_and_config(uint32_t session_id,
						 uint32_t mode)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);
		return -EINVAL;
	}

	if (v->voc_state != VOC_RUN)
		ret = voice_send_cvs_data_exchange_mode_cmd(v);

	if (ret) {
		pr_err("%s: Error voice_send_data_exchange_mode_cmd %d\n",
			__func__, ret);
		goto fail;
	}

	ret = voice_send_cvs_packet_exchange_config_cmd(v);
	if (ret) {
		pr_err("%s: Error: voice_send_packet_exchange_config_cmd %d\n",
			__func__, ret);
		goto fail;
	}

	return ret;
fail:
	return -EINVAL;
}

/**
 * voc_set_tx_mute -
 *       command to send TX mute for voice session
 *
 * @session_id: voice session ID to send this command
 * @dir: RX or TX
 * @mute: TX mute value
 * @ramp_duration: Ramp duration in ms
 *
 * Returns 0 on success or error on failure
 */
int voc_set_tx_mute(uint32_t session_id, uint32_t dir, uint32_t mute,
		    uint32_t ramp_duration)
{
	struct voice_data *v = NULL;
	int ret = 0;
	struct voice_session_itr itr;

	voice_itr_init(&itr, session_id);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v != NULL) {
			mutex_lock(&v->lock);
			v->stream_tx.stream_mute = mute;
			v->stream_tx.stream_mute_ramp_duration_ms =
								ramp_duration;
			if (is_voc_state_active(v->voc_state) &&
				(v->lch_mode == 0))
				ret = voice_send_stream_mute_cmd(v,
				VSS_IVOLUME_DIRECTION_TX,
				v->stream_tx.stream_mute,
				v->stream_tx.stream_mute_ramp_duration_ms);
			mutex_unlock(&v->lock);
		} else {
			pr_err("%s: invalid session_id 0x%x\n", __func__,
				session_id);

			ret = -EINVAL;
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL(voc_set_tx_mute);

/**
 * voc_set_device_mute -
 *       command to set device mute for voice session
 *
 * @session_id: voice session ID to send this command
 * @dir: RX or TX
 * @mute: mute value
 * @ramp_duration: Ramp duration in ms
 *
 * Returns 0 on success or error on failure
 */
int voc_set_device_mute(uint32_t session_id, uint32_t dir, uint32_t mute,
			uint32_t ramp_duration)
{
	struct voice_data *v = NULL;
	int ret = 0;
	struct voice_session_itr itr;

	voice_itr_init(&itr, session_id);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v != NULL) {
			mutex_lock(&v->lock);
			if (dir == VSS_IVOLUME_DIRECTION_TX) {
				v->dev_tx.dev_mute = mute;
				v->dev_tx.dev_mute_ramp_duration_ms =
							ramp_duration;
			} else {
				v->dev_rx.dev_mute = mute;
				v->dev_rx.dev_mute_ramp_duration_ms =
							ramp_duration;
			}

			if (((v->voc_state == VOC_RUN) ||
				(v->voc_state == VOC_STANDBY)) &&
				(v->lch_mode == 0))
				ret = voice_send_device_mute_cmd(v,
							dir,
							mute,
							ramp_duration);
			mutex_unlock(&v->lock);
		} else {
			pr_err("%s: invalid session_id 0x%x\n", __func__,
				session_id);

			ret = -EINVAL;
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL(voc_set_device_mute);

int voc_get_rx_device_mute(uint32_t session_id)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);

		return -EINVAL;
	}

	mutex_lock(&v->lock);

	ret = v->dev_rx.dev_mute;

	mutex_unlock(&v->lock);

	return ret;
}

/**
 * voc_set_tty_mode -
 *       Update tty mode for voice session
 *
 * @session_id: voice session ID
 * @tty_mode: TTY mode value
 *
 * Returns 0 on success or error on failure
 */
int voc_set_tty_mode(uint32_t session_id, uint8_t tty_mode)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);

		return -EINVAL;
	}

	mutex_lock(&v->lock);

	v->tty_mode = tty_mode;

	mutex_unlock(&v->lock);

	return ret;
}
EXPORT_SYMBOL(voc_set_tty_mode);

/**
 * voc_get_tty_mode -
 *       Retrieve tty mode for voice session
 *
 * @session_id: voice session ID
 *
 * Returns 0 on success or error on failure
 */
uint8_t voc_get_tty_mode(uint32_t session_id)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);

		return -EINVAL;
	}

	mutex_lock(&v->lock);

	ret = v->tty_mode;

	mutex_unlock(&v->lock);

	return ret;
}
EXPORT_SYMBOL(voc_get_tty_mode);

/**
 * voc_set_pp_enable -
 *       Command to set PP for voice module
 *
 * @session_id: voice session ID to send this command
 * @module_id: voice module id
 * @enable: enable/disable flag
 *
 * Returns 0 on success or error on failure
 */
int voc_set_pp_enable(uint32_t session_id,
		      struct module_instance_info mod_inst_info,
		      uint32_t enable)
{
	struct voice_data *v = NULL;
	int ret = 0;
	struct voice_session_itr itr;
	int mid = mod_inst_info.module_id;
	int iid = mod_inst_info.instance_id;

	voice_itr_init(&itr, session_id);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v != NULL) {
			if (!(is_voice_app_id(v->session_id)))
				continue;

			mutex_lock(&v->lock);
			if (mid == MODULE_ID_VOICE_MODULE_ST &&
			    iid == INSTANCE_ID_0)
				v->st_enable = enable;

			if (v->voc_state == VOC_RUN) {
				if ((mid == MODULE_ID_VOICE_MODULE_ST) &&
				    iid == INSTANCE_ID_0 && (!v->tty_mode))
					ret = voice_send_set_pp_enable_cmd(
						v, mod_inst_info, enable);
			}
			mutex_unlock(&v->lock);
		} else {
			pr_err("%s: invalid session_id 0x%x\n", __func__,
								session_id);
			ret =  -EINVAL;
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL(voc_set_pp_enable);

/**
 * voc_set_hd_enable -
 *       Command to set HD for voice session
 *
 * @session_id: voice session ID to send this command
 * @enable: enable/disable flag
 *
 * Returns 0 on success or error on failure
 */
int voc_set_hd_enable(uint32_t session_id, uint32_t enable)
{
	struct voice_data *v = NULL;
	int ret = 0;
	struct voice_session_itr itr;

	voice_itr_init(&itr, session_id);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v != NULL) {
			mutex_lock(&v->lock);
			v->hd_enable = enable;

			if (v->voc_state == VOC_RUN)
				ret = voice_send_hd_cmd(v, enable);

			mutex_unlock(&v->lock);
		} else {
			pr_err("%s: invalid session_id 0x%x\n", __func__,
			       session_id);
			ret =  -EINVAL;
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL(voc_set_hd_enable);

/**
 * voc_set_afe_sidetone -
 *       Command to set sidetone at AFE
 *
 * @session_id: voice session ID to send this command
 * @sidetone_enable: enable/disable flag for sidetone
 *
 * Returns 0 on success or error on failure
 */
int voc_set_afe_sidetone(uint32_t session_id, bool sidetone_enable)
{
	struct voice_data *v = NULL;
	int ret = -EINVAL;
	struct voice_session_itr itr;
	u16 rx_port, tx_port;

	common.sidetone_enable = sidetone_enable;
	voice_itr_init(&itr, session_id);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v == NULL) {
			pr_err("%s: invalid session_id 0x%x\n", __func__,
				  session_id);
			ret = -EINVAL;
			break;
		}
		mutex_lock(&v->lock);
		if (v->voc_state != VOC_RUN) {
			mutex_unlock(&v->lock);
			continue;
		}
		rx_port = v->dev_rx.port_id;
		tx_port = v->dev_tx.port_id;
		ret = afe_sidetone_enable(tx_port, rx_port,
					  sidetone_enable);
		if (!ret) {
			mutex_unlock(&v->lock);
			break;
		}
		mutex_unlock(&v->lock);
	}
	return ret;
}
EXPORT_SYMBOL(voc_set_afe_sidetone);

/**
 * voc_get_afe_sidetone -
 *       Retrieve sidetone status at AFE
 *
 * Returns sidetone enable status
 */
bool voc_get_afe_sidetone(void)
{
	bool ret;

	ret = common.sidetone_enable;
	return ret;
}
EXPORT_SYMBOL(voc_get_afe_sidetone);
int voc_get_pp_enable(uint32_t session_id,
		      struct module_instance_info mod_inst_info)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);

		return -EINVAL;
	}

	mutex_lock(&v->lock);
	if (mod_inst_info.module_id == MODULE_ID_VOICE_MODULE_ST &&
	    mod_inst_info.instance_id == INSTANCE_ID_0)
		ret = v->st_enable;
	mutex_unlock(&v->lock);

	return ret;
}

/**
 * voc_set_rx_vol_step -
 *       command to send voice RX volume in step value
 *
 * @session_id: voice session ID
 * @dir: direction RX or TX
 * @vol_step: Volume step value
 * @ramp_duration: Ramp duration in ms
 *
 * Returns 0 on success or -EINVAL on failure
 */
int voc_set_rx_vol_step(uint32_t session_id, uint32_t dir, uint32_t vol_step,
			uint32_t ramp_duration)
{
	struct voice_data *v = NULL;
	int ret = 0;
	struct voice_session_itr itr;

	pr_debug("%s session id = %#x vol = %u", __func__, session_id,
		vol_step);

	voice_itr_init(&itr, session_id);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v != NULL) {
			mutex_lock(&v->lock);
			v->dev_rx.volume_step_value = vol_step;
			v->dev_rx.volume_ramp_duration_ms = ramp_duration;
			if (is_voc_state_active(v->voc_state))
				ret = voice_send_vol_step_cmd(v);
			mutex_unlock(&v->lock);
		} else {
			pr_err("%s: invalid session_id 0x%x\n", __func__,
				session_id);

			ret = -EINVAL;
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL(voc_set_rx_vol_step);

/**
 * voc_set_device_config -
 *       Set voice path config for RX or TX
 *
 * @session_id: voice session ID
 * @path_dir: direction RX or TX
 * @finfo: format config info
 *
 * Returns 0 on success or -EINVAL on failure
 */
int voc_set_device_config(uint32_t session_id, uint8_t path_dir,
			  struct media_format_info *finfo)
{
	struct voice_data *v = voice_get_session(session_id);

	if (v == NULL) {
		pr_err("%s: Invalid session_id 0x%x\n", __func__, session_id);

		return -EINVAL;
	}

	pr_debug("%s: path_dir=%d port_id=%x, channels=%d, sample_rate=%d, bits_per_sample=%d\n",
		__func__, path_dir, finfo->port_id, finfo->num_channels,
		finfo->sample_rate, finfo->bits_per_sample);

	mutex_lock(&v->lock);
	switch (path_dir) {
	case RX_PATH:
		v->dev_rx.port_id = q6audio_get_port_id(finfo->port_id);
		v->dev_rx.no_of_channels = finfo->num_channels;
		v->dev_rx.sample_rate = finfo->sample_rate;
		v->dev_rx.bits_per_sample = finfo->bits_per_sample;
		memcpy(&v->dev_rx.channel_mapping, &finfo->channel_mapping,
		       VSS_CHANNEL_MAPPING_SIZE);
		break;
	case TX_PATH:
		v->dev_tx.port_id = q6audio_get_port_id(finfo->port_id);
		v->dev_tx.no_of_channels = finfo->num_channels;
		v->dev_tx.sample_rate = finfo->sample_rate;
		v->dev_tx.bits_per_sample = finfo->bits_per_sample;
		memcpy(&v->dev_tx.channel_mapping, &finfo->channel_mapping,
		       VSS_CHANNEL_MAPPING_SIZE);
		break;
	default:
		pr_err("%s: Invalid path_dir %d\n", __func__, path_dir);
		return -EINVAL;
	}

	mutex_unlock(&v->lock);

	return 0;
}
EXPORT_SYMBOL(voc_set_device_config);

/**
 * voc_set_ext_ec_ref_media_fmt_info -
 *       Update voice EC media format info
 *
 * @finfo: media format info
 *
 */
int voc_set_ext_ec_ref_media_fmt_info(struct media_format_info *finfo)
{
	mutex_lock(&common.common_lock);
	if (common.ec_ref_ext) {
		common.ec_media_fmt_info.num_channels = finfo->num_channels;
		common.ec_media_fmt_info.bits_per_sample =
			finfo->bits_per_sample;
		common.ec_media_fmt_info.sample_rate = finfo->sample_rate;
		memcpy(&common.ec_media_fmt_info.channel_mapping,
		       &finfo->channel_mapping, VSS_CHANNEL_MAPPING_SIZE);
	} else {
		pr_debug("%s: Ext Ec Ref not active, returning", __func__);
	}
	mutex_unlock(&common.common_lock);
	return 0;
}
EXPORT_SYMBOL(voc_set_ext_ec_ref_media_fmt_info);

/**
 * voc_set_route_flag -
 *       Set voice route state for RX or TX
 *
 * @session_id: voice session ID
 * @path_dir: direction RX or TX
 * @set: Value of route state to set
 *
 * Returns 0 on success or -EINVAL on failure
 */
int voc_set_route_flag(uint32_t session_id, uint8_t path_dir, uint8_t set)
{
	struct voice_data *v = voice_get_session(session_id);

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);

		return -EINVAL;
	}

	pr_debug("%s: path_dir=%d, set=%d\n", __func__, path_dir, set);

	mutex_lock(&v->lock);

	if (path_dir == RX_PATH)
		v->voc_route_state.rx_route_flag = set;
	else
		v->voc_route_state.tx_route_flag = set;

	mutex_unlock(&v->lock);

	return 0;
}
EXPORT_SYMBOL(voc_set_route_flag);

/**
 * voc_get_route_flag -
 *       Retrieve voice route state for RX or TX
 *
 * @session_id: voice session ID
 * @path_dir: direction RX or TX
 *
 * Returns route state on success or 0 on failure
 */
uint8_t voc_get_route_flag(uint32_t session_id, uint8_t path_dir)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);

		return 0;
	}

	mutex_lock(&v->lock);

	if (path_dir == RX_PATH)
		ret = v->voc_route_state.rx_route_flag;
	else
		ret = v->voc_route_state.tx_route_flag;

	mutex_unlock(&v->lock);

	return ret;
}
EXPORT_SYMBOL(voc_get_route_flag);

/**
 * voc_get_mbd_enable -
 *       Retrieve mic break detection enable state
 *
 * Returns true if mic break detection is enabled or false if disabled
 */
bool voc_get_mbd_enable(void)
{
	bool enable = false;

	mutex_lock(&common.common_lock);
	enable = common.mic_break_enable;
	mutex_unlock(&common.common_lock);

	return enable;
}
EXPORT_SYMBOL(voc_get_mbd_enable);

/**
 * voc_set_mbd_enable -
 *       Set mic break detection enable state
 *
 * @enable: mic break detection state to set
 *
 * Returns 0
 */
uint8_t voc_set_mbd_enable(bool enable)
{
	struct voice_data *v = NULL;
	struct voice_session_itr itr;
	bool check_and_send_event = false;
	uint32_t event_id = VSS_INOTIFY_CMD_LISTEN_FOR_EVENT_CLASS;
	uint32_t class_id = VSS_ICOMMON_EVENT_CLASS_VOICE_ACTIVITY_UPDATE;

	mutex_lock(&common.common_lock);
	if (common.mic_break_enable != enable)
		check_and_send_event = true;
	common.mic_break_enable = enable;
	mutex_unlock(&common.common_lock);

	if (!check_and_send_event)
		return 0;

	if (!enable)
		event_id = VSS_INOTIFY_CMD_CANCEL_EVENT_CLASS;

	memset(&itr, 0, sizeof(itr));

	voice_itr_init(&itr, ALL_SESSION_VSID);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v != NULL) {
			mutex_lock(&v->lock);
			if (is_voc_state_active(v->voc_state)) {
				voice_send_mvm_event_class_cmd(v, event_id,
								class_id);
			}
			mutex_unlock(&v->lock);
		}
	}

	return 0;
}
EXPORT_SYMBOL(voc_set_mbd_enable);

/**
 * voc_end_voice_call -
 *       command to end voice call
 *
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_end_voice_call(uint32_t session_id)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);

		return -EINVAL;
	}

	mutex_lock(&v->lock);

	if (v->voc_state == VOC_RUN || v->voc_state == VOC_ERROR ||
	    v->voc_state == VOC_CHANGE || v->voc_state == VOC_STANDBY) {

		pr_debug("%s: VOC_STATE: %d\n", __func__, v->voc_state);

		ret = voice_destroy_vocproc(v);
		if (ret < 0)
			pr_err("%s:  destroy voice failed\n", __func__);

		voc_update_session_params(v);

		voice_destroy_mvm_cvs_session(v);

		ret = voice_mhi_end();
		if (ret < 0)
			pr_debug("%s: voice_mhi_end failed! %d\n",
				 __func__, ret);
		v->voc_state = VOC_RELEASE;
	} else {
		pr_err("%s: Error: End voice called in state %d\n",
			__func__, v->voc_state);

		ret = -EINVAL;
	}

	mutex_unlock(&v->lock);
	return ret;
}
EXPORT_SYMBOL(voc_end_voice_call);

/**
 * voc_standby_voice_call -
 *       command to standy voice call
 *
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_standby_voice_call(uint32_t session_id)
{
	struct voice_data *v = voice_get_session(session_id);
	struct apr_hdr mvm_standby_voice_cmd;
	void *apr_mvm;
	u16 mvm_handle;
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: voc state=%d", __func__, v->voc_state);

	if (v->voc_state == VOC_RUN) {
		apr_mvm = common.apr_q6_mvm;
		if (!apr_mvm) {
			pr_err("%s: apr_mvm is NULL.\n", __func__);
			ret = -EINVAL;
			goto fail;
		}
		mvm_handle = voice_get_mvm_handle(v);
		mvm_standby_voice_cmd.hdr_field =
			APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		mvm_standby_voice_cmd.pkt_size =
			APR_PKT_SIZE(APR_HDR_SIZE,
			sizeof(mvm_standby_voice_cmd) - APR_HDR_SIZE);
		pr_debug("send mvm_standby_voice_cmd pkt size = %d\n",
			 mvm_standby_voice_cmd.pkt_size);
		mvm_standby_voice_cmd.src_port =
				voice_get_idx_for_session(v->session_id);
		mvm_standby_voice_cmd.dest_port = mvm_handle;
		mvm_standby_voice_cmd.token = 0;
		mvm_standby_voice_cmd.opcode = VSS_IMVM_CMD_STANDBY_VOICE;
		v->mvm_state = CMD_STATUS_FAIL;
		ret = apr_send_pkt(apr_mvm,
				(uint32_t *)&mvm_standby_voice_cmd);
		if (ret < 0) {
			pr_err("Fail in sending VSS_IMVM_CMD_STANDBY_VOICE\n");
			ret = -EINVAL;
			goto fail;
		}
		v->voc_state = VOC_STANDBY;
	}
fail:
	return ret;
}
EXPORT_SYMBOL(voc_standby_voice_call);

/**
 * voc_disable_device -
 *       command to pause call and disable voice path
 *
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_disable_device(uint32_t session_id)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0, result = 0;
	struct cal_block_data *cal_block;
	int dest_perms[1] = {PERM_READ | PERM_WRITE | PERM_EXEC};
	int source_vm[2] = {VMID_LPASS, VMID_ADSP_HEAP};
	int dest_vm[1] = {VMID_HLOS};

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: voc state=%d\n", __func__, v->voc_state);

	mutex_lock(&v->lock);
	if (v->voc_state == VOC_RUN) {
		ret = voice_pause_voice_call(v);
		if (ret < 0) {
			pr_err("%s: Pause Voice Call failed for session 0x%x, err %d!\n",
			       __func__, v->session_id, ret);
			goto fail;
		}
		rtac_remove_voice(voice_get_cvs_handle(v));
		voice_send_cvp_deregister_vol_cal_cmd(v);
		mutex_lock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
		voice_send_cvp_deregister_cal_cmd(v);
		mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
		voice_send_cvp_deregister_dev_cfg_cmd(v);

		/* Unload topology modules */
		voice_unload_topo_modules();

		v->voc_state = VOC_CHANGE;
	} else {
		pr_debug("%s: called in voc state=%d, No_OP\n",
			 __func__, v->voc_state);
	}

fail:
	if (hyp_assigned) {
		mutex_lock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
		cal_block = cal_utils_get_only_cal_block(
				common.cal_data[CVP_VOCPROC_CAL]);
		if (cal_block == NULL) {
			pr_err("%s: Cal block NULL, CVP_VOCPROC_CAL!\n",
				__func__);
			ret = -EINVAL;
			mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
			goto done;
		}
		if (cal_block->cma_mem) {
			if (cal_block->cal_data.paddr == 0 ||
			    cal_block->map_data.map_size <= 0) {
				pr_err("%s: No address to map!\n", __func__);
				ret = -EINVAL;
				mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
				goto done;
			}
			result = hyp_assign_phys(
					cal_block->cal_data.paddr,
					cal_block->map_data.map_size,
					source_vm, 2, dest_vm, dest_perms, 1);
			if (result < 0) {
				pr_err("%s: hyp_assign_phys failed result = %d addr = 0x%pK size = %d\n",
					__func__,
					result,
					cal_block->cal_data.paddr,
					cal_block->map_data.map_size);
				ret = -EINVAL;
				mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
				goto done;
			}
			hyp_assigned = false;
			pr_debug("%s: hyp_assign_phys success\n", __func__);
			mutex_unlock(&common.cal_data[CVP_VOCPROC_CAL]->lock);
		}
	}
done:
	mutex_unlock(&v->lock);

	return ret;
}
EXPORT_SYMBOL(voc_disable_device);

/**
 * voc_enable_device -
 *       command to enable voice path and start call
 *
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_enable_device(uint32_t session_id)
{
	struct voice_data *v = voice_get_session(session_id);
	struct module_instance_info mod_inst_info;
	int ret = 0;

	memset(&mod_inst_info, 0, sizeof(mod_inst_info));

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: voc state=%d\n", __func__, v->voc_state);
	mutex_lock(&v->lock);
	if (v->voc_state == VOC_CHANGE) {
		ret = voice_send_tty_mode_cmd(v);
		if (ret < 0) {
			pr_err("%s: Sending TTY mode failed, ret=%d\n",
			       __func__, ret);
			/* Not a critical error, allow voice call to continue */
		}

		mod_inst_info.module_id = MODULE_ID_VOICE_MODULE_ST;
		mod_inst_info.instance_id = INSTANCE_ID_0;

		if (v->tty_mode) {
			/* disable slowtalk */
			voice_send_set_pp_enable_cmd(v, mod_inst_info, 0);
		} else {
			/* restore slowtalk */
			voice_send_set_pp_enable_cmd(v, mod_inst_info,
						     v->st_enable);
		}

		ret = voice_send_set_device_cmd(v);
		if (ret < 0) {
			pr_err("%s: Set device failed, ret=%d\n",
			       __func__, ret);
			goto done;
		}

		ret = voice_send_cvp_media_fmt_info_cmd(v);
		if (ret < 0) {
			pr_err("%s: Set format failed err:%d\n", __func__, ret);
			goto done;
		}

		ret = voice_send_cvp_topology_commit_cmd(v);
		if (ret < 0) {
			pr_err("%s:  Set topology commit failed\n", __func__);
			goto done;
		}

		/* Send MFC config only when the no of channels are > 1 */
		if (v->dev_rx.no_of_channels > NUM_CHANNELS_MONO) {
			ret = voice_send_cvp_mfc_config_cmd(v);
			if (ret < 0) {
				pr_warn("%s: Set mfc config failed err: %d\n",
					__func__, ret);
			}
		}

		voice_send_cvp_register_dev_cfg_cmd(v);
		voice_send_cvp_register_cal_cmd(v);
		voice_send_cvp_register_vol_cal_cmd(v);

		rtac_add_voice(voice_get_cvs_handle(v),
			       voice_get_cvp_handle(v),
			       v->dev_rx.port_id, v->dev_tx.port_id,
			       v->dev_rx.dev_id, v->dev_tx.dev_id,
			       v->session_id);

		ret = voice_send_start_voice_cmd(v);
		if (ret < 0) {
			pr_err("%s: Fail in sending START_VOICE, ret=%d\n",
			       __func__, ret);
			goto done;
		}
		v->voc_state = VOC_RUN;

		if (v->lch_mode == 0) {
			pr_debug("%s: dev_mute = %d, ramp_duration = %d ms\n",
				__func__, v->dev_rx.dev_mute,
				 v->dev_rx.dev_mute_ramp_duration_ms);
			ret = voice_send_device_mute_cmd(v,
					VSS_IVOLUME_DIRECTION_RX,
					v->dev_rx.dev_mute,
					v->dev_rx.dev_mute_ramp_duration_ms);
		}
	} else {
		pr_debug("%s: called in voc state=%d, No_OP\n",
			 __func__, v->voc_state);
	}

done:
	mutex_unlock(&v->lock);

	return ret;
}
EXPORT_SYMBOL(voc_enable_device);

/**
 * voc_set_lch -
 *       command to set hold/unhold call state
 *
 * @session_id: voice session ID to send this command
 * @lch_mode: LCH mode to set
 *
 * Returns 0 on success or error on failure
 */
int voc_set_lch(uint32_t session_id, enum voice_lch_mode lch_mode)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: Invalid session_id 0x%x\n", __func__, session_id);

		ret = -EINVAL;
		goto done;
	}

	mutex_lock(&v->lock);
	if (v->lch_mode == lch_mode) {
		pr_debug("%s: Session %d already in LCH mode %d\n",
				 __func__, session_id, lch_mode);

		mutex_unlock(&v->lock);
		goto done;
	}

	v->lch_mode = lch_mode;
	mutex_unlock(&v->lock);

	ret = voc_lch_ops(v, v->lch_mode);
	if (ret < 0) {
		pr_err("%s: lch ops failed %d\n", __func__, ret);
		goto done;
	}

done:
	return ret;
}
EXPORT_SYMBOL(voc_set_lch);

/**
 * voc_resume_voice_call -
 *       command to resume voice call
 *
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_resume_voice_call(uint32_t session_id)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	ret = voice_send_start_voice_cmd(v);
	if (ret < 0) {
		pr_err("Fail in sending START_VOICE\n");
		goto fail;
	}
	v->voc_state = VOC_RUN;
	return 0;
fail:
	return -EINVAL;
}
EXPORT_SYMBOL(voc_resume_voice_call);

/**
 * voc_start_voice_call -
 *       command to start voice call
 *
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_start_voice_call(uint32_t session_id)
{
	struct voice_data *v = voice_get_session(session_id);
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n", __func__, session_id);

		return -EINVAL;
	}

	mutex_lock(&v->lock);

	if (v->voc_state == VOC_ERROR) {
		pr_debug("%s: VOC in ERR state\n", __func__);

		voice_destroy_mvm_cvs_session(v);
		v->voc_state = VOC_INIT;
	}

	if ((v->voc_state == VOC_INIT) ||
		(v->voc_state == VOC_RELEASE)) {
		ret = voice_apr_register(session_id);
		if (ret < 0) {
			pr_err("%s:  apr register failed\n", __func__);
			goto fail;
		}

		if (is_cvd_version_queried()) {
			pr_debug("%s: Returning the cached value %s\n",
				 __func__, common.cvd_version);
		} else {
			ret = voice_send_mvm_cvd_version_cmd(v);
			if (ret < 0)
				pr_debug("%s: Error retrieving CVD version %d\n",
					 __func__, ret);
		}

		ret = voice_mhi_start();
		if (ret < 0) {
			pr_debug("%s: voice_mhi_start failed! %d\n",
				 __func__, ret);
			goto fail;
		}

		ret = voice_create_mvm_cvs_session(v);
		if (ret < 0) {
			pr_err("create mvm and cvs failed\n");
			goto fail;
		}

		if (is_voip_session(session_id)) {
			/* Allocate oob mem if not already allocated and
			 * memory map the oob memory block.
			 */
			ret = voice_alloc_and_map_oob_mem(v);
			if (ret < 0) {
				pr_err("%s: voice_alloc_and_map_oob_mem() failed, ret:%d\n",
				       __func__, ret);

				goto fail;
			}

			ret = voice_set_packet_exchange_mode_and_config(
				session_id,
				VSS_ISTREAM_PACKET_EXCHANGE_MODE_OUT_OF_BAND);
			if (ret) {
				pr_err("%s: Err: exchange_mode_and_config  %d\n",
					__func__, ret);

				goto fail;
			}
		}
		ret = voice_send_dual_control_cmd(v);
		if (ret < 0) {
			pr_err("Err Dual command failed\n");
			goto fail;
		}
		ret = voice_setup_vocproc(v);
		if (ret < 0) {
			pr_err("setup voice failed\n");
			goto fail;
		}

		ret = voice_send_vol_step_cmd(v);
		if (ret < 0)
			pr_err("voice volume failed\n");

		ret = voice_send_stream_mute_cmd(v,
				VSS_IVOLUME_DIRECTION_TX,
				v->stream_tx.stream_mute,
				v->stream_tx.stream_mute_ramp_duration_ms);
		if (ret < 0)
			pr_err("voice mute failed\n");

		ret = voice_send_start_voice_cmd(v);
		if (ret < 0) {
			pr_err("start voice failed\n");
			goto fail;
		}

		v->voc_state = VOC_RUN;
	} else {
		pr_err("%s: Error: Start voice called in state %d\n",
			__func__, v->voc_state);

		ret = -EINVAL;
		goto fail;
	}
fail:
	mutex_unlock(&v->lock);
	return ret;
}
EXPORT_SYMBOL(voc_start_voice_call);

/**
 * voc_set_ext_ec_ref_port_id -
 *       Set EC ref port id
 *
 * Returns 0 on success or -EINVAL on failure
 */
int voc_set_ext_ec_ref_port_id(uint16_t port_id, bool state)
{
	int ret = 0;

	mutex_lock(&common.common_lock);
	if (state == true) {
		if (port_id == AFE_PORT_INVALID) {
			pr_err("%s: Invalid port id", __func__);
			ret = -EINVAL;
			goto exit;
		}
		common.ec_ref_ext = true;
	} else {
		common.ec_ref_ext = false;
	}
	/* Cache EC Fromat Info in common */
	common.ec_media_fmt_info.port_id = port_id;
exit:
	mutex_unlock(&common.common_lock);
	return ret;
}
EXPORT_SYMBOL(voc_set_ext_ec_ref_port_id);

/**
 * voc_get_ext_ec_ref_port_id -
 *       Retrieve EC ref port id
 *
 * Returns EC Ref port id if present
 *     otherwise AFE_PORT_INVALID
 */
int voc_get_ext_ec_ref_port_id(void)
{
	if (common.ec_ref_ext)
		return common.ec_media_fmt_info.port_id;
	else
		return AFE_PORT_INVALID;
}
EXPORT_SYMBOL(voc_get_ext_ec_ref_port_id);

/**
 * voc_register_mvs_cb -
 *       Update callback info for mvs
 *
 * @ul_cb: Uplink callback fn
 * @dl_cb: downlink callback fn
 * ssr_cb: SSR callback fn
 * @private_data: private data of mvs
 *
 */
void voc_register_mvs_cb(ul_cb_fn ul_cb,
			   dl_cb_fn dl_cb,
			   voip_ssr_cb ssr_cb,
			   void *private_data)
{
	common.mvs_info.ul_cb = ul_cb;
	common.mvs_info.dl_cb = dl_cb;
	common.mvs_info.ssr_cb = ssr_cb;
	common.mvs_info.private_data = private_data;
}
EXPORT_SYMBOL(voc_register_mvs_cb);

/**
 * voc_register_dtmf_rx_detection_cb -
 *       Update callback info for dtmf
 *
 * @dtmf_rx_ul_cb: DTMF uplink RX callback fn
 * @private_data: private data of dtmf info
 *
 */
void voc_register_dtmf_rx_detection_cb(dtmf_rx_det_cb_fn dtmf_rx_ul_cb,
				       void *private_data)
{
	common.dtmf_info.dtmf_rx_ul_cb = dtmf_rx_ul_cb;
	common.dtmf_info.private_data = private_data;
}
EXPORT_SYMBOL(voc_register_dtmf_rx_detection_cb);

/**
 * voc_config_vocoder -
 *       Update config for mvs params.
 */
void voc_config_vocoder(uint32_t media_type,
			uint32_t rate,
			uint32_t network_type,
			uint32_t dtx_mode,
			uint32_t evrc_min_rate,
			uint32_t evrc_max_rate)
{
	common.mvs_info.media_type = media_type;
	common.mvs_info.rate = rate;
	common.mvs_info.network_type = network_type;
	common.mvs_info.dtx_mode = dtx_mode;
	common.mvs_info.evrc_min_rate = evrc_min_rate;
	common.mvs_info.evrc_max_rate = evrc_max_rate;
}
EXPORT_SYMBOL(voc_config_vocoder);

static int32_t qdsp_mvm_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *ptr = NULL, min_payload_size = 0;
	struct common_data *c = NULL;
	struct voice_data *v = NULL;
	struct vss_evt_voice_activity *voice_act_update = NULL;
	int i = 0;
	struct vss_iversion_rsp_get_t *version_rsp = NULL;

	if ((data == NULL) || (priv == NULL)) {
		pr_err("%s: data or priv is NULL\n", __func__);
		return -EINVAL;
	}

	c = priv;

	pr_debug("%s: Payload Length = %d, opcode=%x\n", __func__,
		data->payload_size, data->opcode);

	if (data->opcode == RESET_EVENTS) {
		pr_debug("%s: Reset event received in Voice service\n",
				__func__);

		if (common.mvs_info.ssr_cb) {
			pr_debug("%s: Informing reset event to VoIP\n",
					__func__);
			common.mvs_info.ssr_cb(data->opcode,
					common.mvs_info.private_data);
		}

		apr_reset(c->apr_q6_mvm);
		c->apr_q6_mvm = NULL;

		/* clean up memory handle */
		c->cal_mem_handle = 0;
		c->rtac_mem_handle = 0;
		cal_utils_clear_cal_block_q6maps(MAX_VOICE_CAL_TYPES,
				common.cal_data);
		rtac_clear_mapping(VOICE_RTAC_CAL);

		/* Sub-system restart is applicable to all sessions. */
		for (i = 0; i < MAX_VOC_SESSIONS; i++) {
			c->voice[i].mvm_handle = 0;
			c->voice[i].shmem_info.mem_handle = 0;
		}

		/* Free the ION memory and clear handles for Source Tracking */
		if (is_source_tracking_shared_memomry_allocated()) {
			msm_audio_ion_free(
			common.source_tracking_sh_mem.sh_mem_block.dma_buf);
			common.source_tracking_sh_mem.mem_handle = 0;
			common.source_tracking_sh_mem.sh_mem_block.dma_buf =
									NULL;
		}
		/* clean up srvcc rec flag */
		c->srvcc_rec_flag = false;
		voc_set_error_state(data->reset_proc);
		return 0;
	}

	pr_debug("%s: session_idx 0x%x\n", __func__, data->dest_port);

	v = voice_get_session_by_idx(data->dest_port);
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		return -EINVAL;
	}

	if (data->opcode == APR_BASIC_RSP_RESULT) {
		if (data->payload_size >= sizeof(ptr[0]) * 2) {
			ptr = data->payload;

			pr_debug("%x %x\n", ptr[0], ptr[1]);
			/* ping mvm service ACK */
			switch (ptr[0]) {
			case VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION:
			case VSS_IMVM_CMD_CREATE_FULL_CONTROL_SESSION:
				/* Passive session is used for CS call
				 * Full session is used for VoIP call.
				 */
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				if (!ptr[1]) {
					pr_debug("%s: MVM handle is %d\n",
						 __func__, data->src_port);
					voice_set_mvm_handle(v, data->src_port);
				} else
					pr_err("got NACK for sending MVM create session\n");
				v->mvm_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->mvm_wait);
				break;
			case VSS_IMVM_CMD_START_VOICE:
			case VSS_IMVM_CMD_ATTACH_VOCPROC:
			case VSS_IMVM_CMD_STOP_VOICE:
			case VSS_IMVM_CMD_DETACH_VOCPROC:
			case VSS_ISTREAM_CMD_SET_TTY_MODE:
			case APRV2_IBASIC_CMD_DESTROY_SESSION:
			case VSS_IMVM_CMD_ATTACH_STREAM:
			case VSS_IMVM_CMD_DETACH_STREAM:
			case VSS_ICOMMON_CMD_SET_NETWORK:
			case VSS_ICOMMON_CMD_SET_VOICE_TIMING:
			case VSS_IMVM_CMD_SET_POLICY_DUAL_CONTROL:
			case VSS_IMVM_CMD_SET_CAL_NETWORK:
			case VSS_IMVM_CMD_SET_CAL_MEDIA_TYPE:
			case VSS_IMEMORY_CMD_MAP_PHYSICAL:
			case VSS_IMEMORY_CMD_UNMAP:
			case VSS_IMVM_CMD_PAUSE_VOICE:
			case VSS_IMVM_CMD_STANDBY_VOICE:
			case VSS_IHDVOICE_CMD_ENABLE:
			case VSS_IHDVOICE_CMD_DISABLE:
			case VSS_INOTIFY_CMD_LISTEN_FOR_EVENT_CLASS:
			case VSS_INOTIFY_CMD_CANCEL_EVENT_CLASS:
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				v->mvm_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->mvm_wait);
				break;
			case VSS_IVERSION_CMD_GET:
				pr_debug("%s: Error retrieving CVD Version, error:%d\n",
					 __func__, ptr[1]);

				strlcpy(common.cvd_version, CVD_VERSION_0_0,
					sizeof(common.cvd_version));
				pr_debug("%s: Fall back to default value, CVD Version = %s\n",
					 __func__, common.cvd_version);

				v->mvm_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->mvm_wait);
				break;
			default:
				pr_debug("%s: not match cmd = 0x%x\n",
					__func__, ptr[0]);
				break;
			}
		}
	} else if (data->opcode == VSS_IMEMORY_RSP_MAP) {
		pr_debug("%s, Revd VSS_IMEMORY_RSP_MAP response\n", __func__);

		if (data->payload_size < sizeof(ptr[0])) {
			pr_err("%s: payload has invalid size[%d]\n", __func__,
			       data->payload_size);
			return -EINVAL;
		}

		if (data->token == VOIP_MEM_MAP_TOKEN) {
			ptr = data->payload;
			if (ptr[0]) {
				v->shmem_info.mem_handle = ptr[0];
				pr_debug("%s: shared mem_handle: 0x[%x]\n",
					 __func__, v->shmem_info.mem_handle);
				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			}
		} else if (data->payload_size &&
			   data->token == VOC_CAL_MEM_MAP_TOKEN) {
			ptr = data->payload;
			if (ptr[0]) {
				c->cal_mem_handle = ptr[0];

				pr_debug("%s: cal mem handle 0x%x\n",
					 __func__, c->cal_mem_handle);

				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			}
		} else if (data->payload_size &&
			   data->token == VOC_VOICE_HOST_PCM_MAP_TOKEN) {
			ptr = data->payload;
			if (ptr[0]) {
				common.voice_host_pcm_mem_handle = ptr[0];

				pr_debug("%s: vhpcm mem handle 0x%x\n",
					 __func__,
					 common.voice_host_pcm_mem_handle);
				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			}
		} else if (data->payload_size &&
				data->token == VOC_RTAC_MEM_MAP_TOKEN) {
			ptr = data->payload;
			if (ptr[0]) {
				c->rtac_mem_handle = ptr[0];

				pr_debug("%s: cal mem handle 0x%x\n",
					 __func__, c->rtac_mem_handle);

				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			}
		} else if (data->payload_size &&
			   data->token == VOC_SOURCE_TRACKING_MEM_MAP_TOKEN) {
			ptr = data->payload;
			if (ptr[0]) {
				common.source_tracking_sh_mem.mem_handle =
									ptr[0];

				pr_debug("%s: Source Tracking shared mem handle 0x%x\n",
					 __func__,
				   common.source_tracking_sh_mem.mem_handle);

				v->mvm_state = CMD_STATUS_SUCCESS;
				wake_up(&v->mvm_wait);
			}
		} else {
			pr_err("%s: Unknown mem map token %d\n",
			       __func__, data->token);
		}
	} else if (data->opcode == VSS_IVERSION_RSP_GET) {
		pr_debug("%s: Received VSS_IVERSION_RSP_GET\n", __func__);

		if (data->payload_size) {
			min_payload_size = min_t(u32, (int)data->payload_size,
					       CVD_VERSION_STRING_MAX_SIZE);
			version_rsp =
				(struct vss_iversion_rsp_get_t *)data->payload;
			memcpy(common.cvd_version, version_rsp->version,
			       min_payload_size);
			common.cvd_version[min_payload_size - 1] = '\0';
			pr_debug("%s: CVD Version = %s\n",
				 __func__, common.cvd_version);

			v->mvm_state = CMD_STATUS_SUCCESS;
			wake_up(&v->mvm_wait);
		}
	} else if (data->opcode == VSS_ICOMMON_EVT_VOICE_ACTIVITY_UPDATE) {
		if (data->payload_size == sizeof(struct vss_evt_voice_activity)) {
			voice_act_update =
				(struct vss_evt_voice_activity *)
				data->payload;

			/* Drop notifications other than Mic Break */
			if ((voice_act_update->activity
				     != VSS_ICOMMON_VOICE_ACTIVITY_MIC_BREAK)
				&& (voice_act_update->activity
				     != VSS_ICOMMON_VOICE_ACITIVTY_MIC_UNBREAK))
				return 0;

			switch (voice_act_update->activity) {
			case VSS_ICOMMON_VOICE_ACTIVITY_MIC_BREAK:
				v->mic_break_status = true;
				break;
			case VSS_ICOMMON_VOICE_ACITIVTY_MIC_UNBREAK:
				v->mic_break_status = false;
				break;
			}

			if (c->mic_break_enable)
				schedule_work(&(v->voice_mic_break_work));
		}
	}

	return 0;
}

static int32_t qdsp_cvs_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *ptr = NULL;
	struct common_data *c = NULL;
	struct voice_data *v = NULL;
	int i = 0;

	if ((data == NULL) || (priv == NULL)) {
		pr_err("%s: data or priv is NULL\n", __func__);
		return -EINVAL;
	}

	c = priv;

	pr_debug("%s: session_id 0x%x\n", __func__, data->dest_port);
	pr_debug("%s: Payload Length = %d, opcode=%x\n", __func__,
		data->payload_size, data->opcode);

	if (data->opcode == RESET_EVENTS) {
		pr_debug("%s: Reset event received in Voice service\n",
				__func__);

		apr_reset(c->apr_q6_cvs);
		c->apr_q6_cvs = NULL;

		/* Sub-system restart is applicable to all sessions. */
		for (i = 0; i < MAX_VOC_SESSIONS; i++)
			c->voice[i].cvs_handle = 0;

		cal_utils_clear_cal_block_q6maps(MAX_VOICE_CAL_TYPES,
				common.cal_data);

		/* Free the ION memory and clear handles for Source Tracking */
		if (is_source_tracking_shared_memomry_allocated()) {
			msm_audio_ion_free(
			common.source_tracking_sh_mem.sh_mem_block.dma_buf);
			common.source_tracking_sh_mem.mem_handle = 0;
			common.source_tracking_sh_mem.sh_mem_block.dma_buf =
									NULL;
		}
		voc_set_error_state(data->reset_proc);
		return 0;
	}

	v = voice_get_session_by_idx(data->dest_port);
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		return -EINVAL;
	}

	if (data->opcode == APR_BASIC_RSP_RESULT) {
		if (data->payload_size) {
			ptr = data->payload;

			pr_debug("%x %x\n", ptr[0], ptr[1]);
			if (ptr[1] != 0) {
				pr_err("%s: cmd = 0x%x returned error = 0x%x\n",
					__func__, ptr[0], ptr[1]);
			}
			/*response from  CVS */
			switch (ptr[0]) {
			case VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION:
			case VSS_ISTREAM_CMD_CREATE_FULL_CONTROL_SESSION:
				if (!ptr[1]) {
					pr_debug("%s: CVS handle is %d\n",
						 __func__, data->src_port);
					voice_set_cvs_handle(v, data->src_port);
				} else
					pr_err("got NACK for sending CVS create session\n");
				v->cvs_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->cvs_wait);
				break;
			case VSS_IVOLUME_CMD_MUTE_V2:
			case VSS_ISTREAM_CMD_SET_MEDIA_TYPE:
			case VSS_ISTREAM_CMD_VOC_AMR_SET_ENC_RATE:
			case VSS_ISTREAM_CMD_VOC_AMRWB_SET_ENC_RATE:
			case VSS_ISTREAM_CMD_SET_ENC_DTX_MODE:
			case VSS_ISTREAM_CMD_CDMA_SET_ENC_MINMAX_RATE:
			case APRV2_IBASIC_CMD_DESTROY_SESSION:
			case VSS_ISTREAM_CMD_REGISTER_CALIBRATION_DATA_V2:
			case VSS_ISTREAM_CMD_DEREGISTER_CALIBRATION_DATA:
			case VSS_ISTREAM_CMD_REGISTER_STATIC_CALIBRATION_DATA:
			case VSS_ISTREAM_CMD_DEREGISTER_STATIC_CALIBRATION_DATA:
			case VSS_ICOMMON_CMD_MAP_MEMORY:
			case VSS_ICOMMON_CMD_UNMAP_MEMORY:
			case VSS_ICOMMON_CMD_SET_UI_PROPERTY:
			case VSS_ICOMMON_CMD_SET_UI_PROPERTY_V2:
			case VSS_IPLAYBACK_CMD_START:
			case VSS_IPLAYBACK_CMD_STOP:
			case VSS_IRECORD_CMD_START:
			case VSS_IRECORD_CMD_STOP:
			case VSS_ISTREAM_CMD_SET_PACKET_EXCHANGE_MODE:
			case VSS_ISTREAM_CMD_SET_OOB_PACKET_EXCHANGE_CONFIG:
			case VSS_ISTREAM_CMD_SET_RX_DTMF_DETECTION:
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				v->cvs_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->cvs_wait);
				break;
			case VSS_ICOMMON_CMD_SET_PARAM_V2:
			case VSS_ICOMMON_CMD_SET_PARAM_V3:
				pr_debug("%s: VSS_ICOMMON_CMD_SET_PARAM\n",
					 __func__);
				rtac_make_voice_callback(RTAC_CVS, ptr,
							data->payload_size);
				break;
			case VSS_ICOMMON_CMD_GET_PARAM_V2:
			case VSS_ICOMMON_CMD_GET_PARAM_V3:
				pr_debug("%s: VSS_ICOMMON_CMD_GET_PARAM_V2\n",
					 __func__);
				/* Should only come here if there is an APR */
				/* error or malformed APR packet. Otherwise */
				/* response will be returned as */
				/* VSS_ICOMMON_RSP_GET_PARAM */
				if (ptr[1] != 0) {
					pr_err("%s: CVP get param error = %d, resuming\n",
						__func__, ptr[1]);
					rtac_make_voice_callback(RTAC_CVP,
						data->payload,
						data->payload_size);
				}
				break;
			default:
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				break;
			}
		}
	} else if (data->opcode ==
			 VSS_ISTREAM_EVT_OOB_NOTIFY_ENC_BUFFER_READY) {
		int ret = 0;
		u16 cvs_handle;
		uint32_t *cvs_voc_pkt;
		struct cvs_enc_buffer_consumed_cmd send_enc_buf_consumed_cmd;
		void *apr_cvs;

		pr_debug("Encoder buffer is ready\n");

		apr_cvs = common.apr_q6_cvs;
		if (!apr_cvs) {
			pr_err("%s: apr_cvs is NULL\n", __func__);
			return -EINVAL;
		}
		cvs_handle = voice_get_cvs_handle(v);

		send_enc_buf_consumed_cmd.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE),
				APR_PKT_VER);
		send_enc_buf_consumed_cmd.hdr.pkt_size =
			APR_PKT_SIZE(APR_HDR_SIZE,
			sizeof(send_enc_buf_consumed_cmd) - APR_HDR_SIZE);

		send_enc_buf_consumed_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
		send_enc_buf_consumed_cmd.hdr.dest_port = cvs_handle;
		send_enc_buf_consumed_cmd.hdr.token = 0;
		send_enc_buf_consumed_cmd.hdr.opcode =
			VSS_ISTREAM_EVT_OOB_NOTIFY_ENC_BUFFER_CONSUMED;

		cvs_voc_pkt = v->shmem_info.sh_buf.buf[1].data;
		if (cvs_voc_pkt != NULL &&  common.mvs_info.ul_cb != NULL) {
			if (v->shmem_info.sh_buf.buf[1].size <
			    ((3 * sizeof(uint32_t)) + cvs_voc_pkt[2])) {
				pr_err("%s: invalid voc pkt size\n", __func__);
				return -EINVAL;
			}
			/* cvs_voc_pkt[0] contains tx timestamp */
			common.mvs_info.ul_cb((uint8_t *)&cvs_voc_pkt[3],
					      cvs_voc_pkt[2],
					      cvs_voc_pkt[0],
					      common.mvs_info.private_data);
		} else
			pr_err("%s: cvs_voc_pkt or ul_cb is NULL\n", __func__);

		ret = apr_send_pkt(apr_cvs,
			(uint32_t *) &send_enc_buf_consumed_cmd);
		if (ret < 0) {
			pr_err("%s: Err send ENC_BUF_CONSUMED_NOTIFY %d\n",
				__func__, ret);
			goto fail;
		}
	} else if (data->opcode == VSS_ISTREAM_EVT_SEND_ENC_BUFFER) {
		pr_debug("Recd VSS_ISTREAM_EVT_SEND_ENC_BUFFER\n");
	} else if (data->opcode ==
			 VSS_ISTREAM_EVT_OOB_NOTIFY_DEC_BUFFER_REQUEST) {
		int ret = 0;
		u16 cvs_handle;
		uint32_t *cvs_voc_pkt;
		struct cvs_dec_buffer_ready_cmd send_dec_buf;
		void *apr_cvs;

		apr_cvs = common.apr_q6_cvs;

		if (!apr_cvs) {
			pr_err("%s: apr_cvs is NULL\n", __func__);
			return -EINVAL;
		}
		cvs_handle = voice_get_cvs_handle(v);

		send_dec_buf.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE),
				APR_PKT_VER);

		send_dec_buf.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					sizeof(send_dec_buf) - APR_HDR_SIZE);

		send_dec_buf.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
		send_dec_buf.hdr.dest_port = cvs_handle;
		send_dec_buf.hdr.token = 0;
		send_dec_buf.hdr.opcode =
				 VSS_ISTREAM_EVT_OOB_NOTIFY_DEC_BUFFER_READY;

		cvs_voc_pkt = (uint32_t *)(v->shmem_info.sh_buf.buf[0].data);
		if (cvs_voc_pkt != NULL && common.mvs_info.dl_cb != NULL) {
			/* Set timestamp to 0 and advance the pointer */
			cvs_voc_pkt[0] = 0;
			/* Set media_type and advance the pointer */
			cvs_voc_pkt[1] = common.mvs_info.media_type;
			common.mvs_info.dl_cb(
					      (uint8_t *)&cvs_voc_pkt[2],
					      common.mvs_info.private_data);
			ret = apr_send_pkt(apr_cvs, (uint32_t *) &send_dec_buf);
			if (ret < 0) {
				pr_err("%s: Err send DEC_BUF_READY_NOTIFI %d\n",
					__func__, ret);
				goto fail;
			}
		} else {
			pr_debug("%s: voc_pkt or dl_cb is NULL\n", __func__);
			goto fail;
		}
	} else if (data->opcode == VSS_ISTREAM_EVT_REQUEST_DEC_BUFFER) {
		pr_debug("Recd VSS_ISTREAM_EVT_REQUEST_DEC_BUFFER\n");
	} else if (data->opcode == VSS_ISTREAM_EVT_SEND_DEC_BUFFER) {
		pr_debug("Send dec buf resp\n");
	} else if (data->opcode == APR_RSP_ACCEPTED) {
		ptr = data->payload;
		if (ptr[0])
			pr_debug("%s: APR_RSP_ACCEPTED for 0x%x:\n",
				 __func__, ptr[0]);
	} else if (data->opcode == VSS_ISTREAM_EVT_NOT_READY) {
		pr_debug("Recd VSS_ISTREAM_EVT_NOT_READY\n");
	} else if (data->opcode == VSS_ISTREAM_EVT_READY) {
		pr_debug("Recd VSS_ISTREAM_EVT_READY\n");
	} else if (data->opcode == VSS_ICOMMON_RSP_GET_PARAM ||
		   data->opcode == VSS_ICOMMON_RSP_GET_PARAM_V3) {
		pr_debug("%s: VSS_ICOMMON_RSP_GET_PARAM\n", __func__);
		ptr = data->payload;
		if (ptr[0] != 0) {
			pr_err("%s: VSS_ICOMMON_RSP_GET_PARAM returned error = 0x%x\n",
			       __func__, ptr[0]);
		}
		rtac_make_voice_callback(RTAC_CVS, data->payload,
					data->payload_size);
	}  else if (data->opcode == VSS_ISTREAM_EVT_RX_DTMF_DETECTED) {
		struct vss_istream_evt_rx_dtmf_detected *dtmf_rx_detected;
		uint32_t *voc_pkt = data->payload;
		uint32_t pkt_len = data->payload_size;

		if ((voc_pkt != NULL) &&
		    (pkt_len ==
			sizeof(struct vss_istream_evt_rx_dtmf_detected))) {

			dtmf_rx_detected =
			(struct vss_istream_evt_rx_dtmf_detected *) voc_pkt;
			pr_debug("RX_DTMF_DETECTED low_freq=%d high_freq=%d\n",
				 dtmf_rx_detected->low_freq,
				 dtmf_rx_detected->high_freq);
			if (c->dtmf_info.dtmf_rx_ul_cb)
				c->dtmf_info.dtmf_rx_ul_cb((uint8_t *)voc_pkt,
					voc_get_session_name(v->session_id),
					c->dtmf_info.private_data);
		} else {
			pr_err("Invalid packet\n");
		}
	}  else
		pr_debug("Unknown opcode 0x%x\n", data->opcode);

fail:
	return 0;
}

static int32_t qdsp_cvp_callback(struct apr_client_data *data, void *priv)
{
	uint32_t *ptr = NULL;
	struct common_data *c = NULL;
	struct voice_data *v = NULL;
	int i = 0;

	if ((data == NULL) || (priv == NULL)) {
		pr_err("%s: data or priv is NULL\n", __func__);
		return -EINVAL;
	}

	c = priv;

	if (data->opcode == RESET_EVENTS) {
		pr_debug("%s: Reset event received in Voice service\n",
				__func__);

		apr_reset(c->apr_q6_cvp);
		c->apr_q6_cvp = NULL;
		cal_utils_clear_cal_block_q6maps(MAX_VOICE_CAL_TYPES,
				common.cal_data);

		/* Sub-system restart is applicable to all sessions. */
		for (i = 0; i < MAX_VOC_SESSIONS; i++)
			c->voice[i].cvp_handle = 0;

		/*
		 * Free the ION memory and clear handles for
		 * Source Tracking
		 */
		if (is_source_tracking_shared_memomry_allocated()) {
			msm_audio_ion_free(
			common.source_tracking_sh_mem.sh_mem_block.dma_buf);
			common.source_tracking_sh_mem.mem_handle = 0;
			common.source_tracking_sh_mem.sh_mem_block.dma_buf =
									NULL;
		}
		voc_set_error_state(data->reset_proc);
		return 0;
	}

	v = voice_get_session_by_idx(data->dest_port);
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		return -EINVAL;
	}

	if (data->opcode == APR_BASIC_RSP_RESULT) {
		if (data->payload_size >= (2 * sizeof(uint32_t))) {
			ptr = data->payload;

			pr_debug("%x %x\n", ptr[0], ptr[1]);
			if (ptr[1] != 0) {
				pr_err("%s: cmd = 0x%x returned error = 0x%x\n",
					__func__, ptr[0], ptr[1]);
			}
			switch (ptr[0]) {
			case VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V2:
			case VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V3:
			/*response from  CVP */
				pr_debug("%s: cmd = 0x%x\n", __func__, ptr[0]);
				if (!ptr[1]) {
					voice_set_cvp_handle(v, data->src_port);
					pr_debug("status: %d, cvphdl=%d\n",
						 ptr[1], data->src_port);
				} else
					pr_err("got NACK from CVP create session response\n");
				v->cvp_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->cvp_wait);
				break;
			case VSS_IVOCPROC_CMD_SET_DEVICE_V2:
			case VSS_IVOCPROC_CMD_SET_DEVICE_V3:
			case VSS_IVOLUME_CMD_SET_STEP:
			case VSS_IVOCPROC_CMD_ENABLE:
			case VSS_IVOCPROC_CMD_DISABLE:
			case APRV2_IBASIC_CMD_DESTROY_SESSION:
			case VSS_IVOCPROC_CMD_REGISTER_VOL_CALIBRATION_DATA:
			case VSS_IVOCPROC_CMD_DEREGISTER_VOL_CALIBRATION_DATA:
			case VSS_IVOCPROC_CMD_REGISTER_CALIBRATION_DATA_V2:
			case VSS_IVOCPROC_CMD_DEREGISTER_CALIBRATION_DATA:
			case VSS_IVOCPROC_CMD_REGISTER_DYNAMIC_CALIBRATION_DATA:
			case VSS_IVOCPROC_CMD_DEREGISTER_DYNAMIC_CALIBRATION_DATA:
			case VSS_IVOCPROC_CMD_REGISTER_STATIC_CALIBRATION_DATA:
			case VSS_IVOCPROC_CMD_DEREGISTER_STATIC_CALIBRATION_DATA:
			case VSS_IVOCPROC_CMD_REGISTER_DEVICE_CONFIG:
			case VSS_IVOCPROC_CMD_DEREGISTER_DEVICE_CONFIG:
			case VSS_ICOMMON_CMD_MAP_MEMORY:
			case VSS_ICOMMON_CMD_UNMAP_MEMORY:
			case VSS_IVOLUME_CMD_MUTE_V2:
			case VSS_IVPCM_CMD_START_V2:
			case VSS_IVPCM_CMD_STOP:
			case VSS_IVOCPROC_CMD_TOPOLOGY_SET_DEV_CHANNELS:
			case VSS_IVOCPROC_CMD_TOPOLOGY_COMMIT:
				v->cvp_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->cvp_wait);
				break;
			case VSS_IVPCM_EVT_PUSH_BUFFER_V2:
				break;
			case VSS_ICOMMON_CMD_SET_PARAM_V2:
			case VSS_ICOMMON_CMD_SET_PARAM_V3:
				switch (data->token) {
				case VOC_SET_MEDIA_FORMAT_PARAM_TOKEN:
				case VOC_GENERIC_SET_PARAM_TOKEN:
					pr_debug("%s: VSS_ICOMMON_CMD_SET_PARAM called by voice_send_cvp_media_format_cmd\n",
						__func__);
					v->cvp_state = CMD_STATUS_SUCCESS;
					v->async_err = ptr[1];
					wake_up(&v->cvp_wait);
					break;
				case VOC_RTAC_SET_PARAM_TOKEN:
					pr_debug("%s: VSS_ICOMMON_CMD_SET_PARAM called by rtac\n",
						__func__);
					rtac_make_voice_callback(
						RTAC_CVP, ptr,
						data->payload_size);
					break;
				default:
					pr_debug("%s: invalid token for command VSS_ICOMMON_CMD_SET_PARAM: %d\n",
						__func__, data->token);
					break;
				}
				break;
			case VSS_ICOMMON_CMD_GET_PARAM_V2:
			case VSS_ICOMMON_CMD_GET_PARAM_V3:
				pr_debug("%s: VSS_ICOMMON_CMD_GET_PARAM_V2\n",
					 __func__);
				/* Should only come here if there is an APR */
				/* error or malformed APR packet. Otherwise */
				/* response will be returned as */
				/* VSS_ICOMMON_RSP_GET_PARAM */
				if (ptr[1] != 0) {
					pr_err("%s: CVP get param error = %d, resuming\n",
						__func__, ptr[1]);
					rtac_make_voice_callback(RTAC_CVP,
						data->payload,
						data->payload_size);
				}
				break;
			case VSS_ISOUNDFOCUS_CMD_SET_SECTORS:
				if (!ptr[1])
					common.is_sound_focus_resp_success =
									true;
				else
					common.is_sound_focus_resp_success =
									false;
				v->cvp_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->cvp_wait);
				break;
			case VSS_ISOUNDFOCUS_CMD_GET_SECTORS:
				/*
				 * Should only come here if there is an error
				 * response received from ADSP. Otherwise
				 * response will be returned as
				 * VSS_ISOUNDFOCUS_RSP_GET_SECTORS
				 */
				pr_err("%s: VSS_ISOUNDFOCUS_CMD_GET_SECTORS failed\n",
					__func__);

				common.is_sound_focus_resp_success = false;
				v->cvp_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->cvp_wait);
				break;
			case VSS_ISOURCETRACK_CMD_GET_ACTIVITY:
				if (!ptr[1]) {
					/* Read data from shared memory */
					memcpy(&common.sourceTrackingResponse,
					       common.source_tracking_sh_mem.
							sh_mem_block.data,
					       sizeof(struct
					 vss_isourcetrack_activity_data_t));
					common.is_source_tracking_resp_success =
									true;
				} else {
					common.is_source_tracking_resp_success =
									false;
					pr_err("%s: Error received for source tracking params\n",
						__func__);
				}
				v->cvp_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->cvp_wait);
				break;
			case VSS_IFNN_ST_CMD_GET_ACTIVITY:
				if (!ptr[1]) {
					/* Read data from shared memory */
					common.is_source_tracking_resp_success = true;
				} else {
					common.is_source_tracking_resp_success = false;
					pr_err("%s: Error received for source tracking params\n",
						__func__);
				}
				v->cvp_state = CMD_STATUS_SUCCESS;
				v->async_err = ptr[1];
				wake_up(&v->cvp_wait);
				break;
			default:
				pr_debug("%s: not match cmd = 0x%x\n",
					  __func__, ptr[0]);
				break;
			}
		}
	} else if (data->opcode == VSS_ICOMMON_RSP_GET_PARAM ||
		   data->opcode == VSS_ICOMMON_RSP_GET_PARAM_V3) {
		pr_debug("%s: VSS_ICOMMON_RSP_GET_PARAM\n", __func__);
		ptr = data->payload;
		if (ptr[0] != 0) {
			pr_err("%s: VSS_ICOMMON_RSP_GET_PARAM returned error = 0x%x\n",
			       __func__, ptr[0]);
		}
		rtac_make_voice_callback(RTAC_CVP, data->payload,
			data->payload_size);
	} else if (data->opcode == VSS_IVPCM_EVT_NOTIFY_V2) {
		if ((data->payload != NULL) && data->payload_size ==
		    sizeof(struct vss_ivpcm_evt_notify_v2_t) &&
		    common.hostpcm_info.hostpcm_evt_cb != NULL) {
			common.hostpcm_info.hostpcm_evt_cb(data->payload,
					voc_get_session_name(v->session_id),
					common.hostpcm_info.private_data);
		}
	} else if (data->opcode == VSS_ISOUNDFOCUS_RSP_GET_SECTORS) {
		if (data->payload && (data->payload_size ==
			sizeof(struct vss_isoundfocus_rsp_get_sectors_t))) {
			common.is_sound_focus_resp_success = true;
			memcpy(&common.soundFocusResponse,
			       (struct vss_isoundfocus_rsp_get_sectors_t *)
			       data->payload,
			       sizeof(struct
					 vss_isoundfocus_rsp_get_sectors_t));
		} else {
			common.is_sound_focus_resp_success = false;
			pr_debug("%s: Invalid payload received from CVD\n",
				 __func__);
		}
		v->cvp_state = CMD_STATUS_SUCCESS;
		wake_up(&v->cvp_wait);
	}
	return 0;
}

static int voice_free_oob_shared_mem(void)
{
	int rc = 0;
	int cnt = 0;
	int bufcnt = NUM_OF_BUFFERS;
	struct voice_data *v = voice_get_session(
				common.voice[VOC_PATH_FULL].session_id);

	mutex_lock(&common.common_lock);
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		rc = -EINVAL;
		goto done;
	}

	rc = msm_audio_ion_free(v->shmem_info.sh_buf.dma_buf);
	v->shmem_info.sh_buf.dma_buf = NULL;
	if (rc < 0) {
		pr_err("%s: Error:%d freeing memory\n", __func__, rc);

		goto done;
	}


	while (cnt < bufcnt) {
		v->shmem_info.sh_buf.buf[cnt].data =  NULL;
		v->shmem_info.sh_buf.buf[cnt].phys =  0;
		cnt++;
	}

	v->shmem_info.sh_buf.dma_buf = NULL;

done:
	mutex_unlock(&common.common_lock);
	return rc;
}

static int voice_alloc_oob_shared_mem(void)
{
	int cnt = 0;
	int rc = 0;
	size_t len;
	void *mem_addr = NULL;
	dma_addr_t phys;
	int bufsz = BUFFER_BLOCK_SIZE;
	int bufcnt = NUM_OF_BUFFERS;
	struct voice_data *v = voice_get_session(
				common.voice[VOC_PATH_FULL].session_id);

	mutex_lock(&common.common_lock);
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		rc = -EINVAL;
		goto done;
	}

	rc = msm_audio_ion_alloc(&(v->shmem_info.sh_buf.dma_buf),
			bufsz * bufcnt,
			&phys, &len,
			&mem_addr);
	if (rc) {
		pr_err("%s: audio ION alloc failed, rc = %d\n",
			__func__, rc);

		rc = -EINVAL;
		goto done;
	}

	while (cnt < bufcnt) {
		v->shmem_info.sh_buf.buf[cnt].data =  mem_addr  + (cnt * bufsz);
		v->shmem_info.sh_buf.buf[cnt].phys =  phys + (cnt * bufsz);
		v->shmem_info.sh_buf.buf[cnt].size = bufsz;
		cnt++;
	}

	pr_debug("%s buf[0].data:[%pK], buf[0].phys:[%pK], &buf[0].phys:[%pK],\n",
		 __func__,
		(void *)v->shmem_info.sh_buf.buf[0].data,
		&v->shmem_info.sh_buf.buf[0].phys,
		(void *)&v->shmem_info.sh_buf.buf[0].phys);
	pr_debug("%s: buf[1].data:[%pK], buf[1].phys[%pK], &buf[1].phys[%pK]\n",
		__func__,
		(void *)v->shmem_info.sh_buf.buf[1].data,
		&v->shmem_info.sh_buf.buf[1].phys,
		(void *)&v->shmem_info.sh_buf.buf[1].phys);

	memset((void *)v->shmem_info.sh_buf.buf[0].data, 0, (bufsz * bufcnt));

done:
	mutex_unlock(&common.common_lock);
	return rc;
}

static int voice_alloc_oob_mem_table(void)
{
	int rc = 0;
	size_t len;
	struct voice_data *v = voice_get_session(
				common.voice[VOC_PATH_FULL].session_id);

	mutex_lock(&common.common_lock);
	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		rc = -EINVAL;
		goto done;
	}

	rc = msm_audio_ion_alloc(&(v->shmem_info.memtbl.dma_buf),
				sizeof(struct vss_imemory_table_t),
				&v->shmem_info.memtbl.phys,
				&len,
				&(v->shmem_info.memtbl.data));
	if (rc) {
		pr_err("%s: audio ION alloc failed, rc = %d\n",
			__func__, rc);

		rc = -EINVAL;
		goto done;
	}

	v->shmem_info.memtbl.size = sizeof(struct vss_imemory_table_t);
	pr_debug("%s data[%pK]phys[%pK][%pK]\n", __func__,
		 (void *)v->shmem_info.memtbl.data,
		 &v->shmem_info.memtbl.phys,
		 (void *)&v->shmem_info.memtbl.phys);

done:
	mutex_unlock(&common.common_lock);
	return rc;
}

/**
 * voc_send_cvp_start_vocpcm -
 *       command to start voice hpcm
 *
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_send_cvp_start_vocpcm(uint32_t session_id,
			      struct vss_ivpcm_tap_point *vpcm_tp,
			      uint32_t no_of_tp)
{
	struct cvp_start_cmd cvp_start_cmd;
	int ret = 0;
	void *apr_cvp;
	u16 cvp_handle;
	struct voice_data *v = voice_get_session(session_id);
	int i = 0;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}
	apr_cvp = common.apr_q6_cvp;

	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	cvp_handle = voice_get_cvp_handle(v);

	/* Fill the header */
	cvp_start_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
		APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	cvp_start_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
		sizeof(struct vss_ivpcm_tap_point) * no_of_tp) +
		sizeof(cvp_start_cmd.vpcm_start_cmd.num_tap_points) +
		sizeof(cvp_start_cmd.vpcm_start_cmd.mem_handle);
	cvp_start_cmd.hdr.src_port = voice_get_idx_for_session(v->session_id);
	cvp_start_cmd.hdr.dest_port = cvp_handle;
	cvp_start_cmd.hdr.token = 0;
	cvp_start_cmd.hdr.opcode = VSS_IVPCM_CMD_START_V2;

	for (i = 0; i < no_of_tp; i++) {
		cvp_start_cmd.vpcm_start_cmd.tap_points[i].tap_point =
							vpcm_tp[i].tap_point;
		cvp_start_cmd.vpcm_start_cmd.tap_points[i].direction =
							vpcm_tp[i].direction;
		cvp_start_cmd.vpcm_start_cmd.tap_points[i].sampling_rate =
						    vpcm_tp[i].sampling_rate;
		cvp_start_cmd.vpcm_start_cmd.tap_points[i].duration = 0;
	}

	cvp_start_cmd.vpcm_start_cmd.mem_handle =
				common.voice_host_pcm_mem_handle;
	cvp_start_cmd.vpcm_start_cmd.num_tap_points = no_of_tp;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &cvp_start_cmd);
	if (ret < 0) {
		pr_err("%s: Fail: sending vocpcm map memory,\n", __func__);
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
			(v->cvp_state == CMD_STATUS_SUCCESS),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto done;
	}
	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;
}
EXPORT_SYMBOL(voc_send_cvp_start_vocpcm);

/**
 * voc_send_cvp_stop_vocpcm -
 *       command to stop voice hpcm
 *
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_send_cvp_stop_vocpcm(uint32_t session_id)
{
	struct cvp_command vpcm_stop_cmd;
	int ret = 0;
	void *apr_cvp;
	u16 cvp_handle;
	struct voice_data *v = voice_get_session(session_id);

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}
	apr_cvp = common.apr_q6_cvp;

	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	cvp_handle = voice_get_cvp_handle(v);

	/* fill in the header */
	vpcm_stop_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				 APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	vpcm_stop_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
			 sizeof(vpcm_stop_cmd) - APR_HDR_SIZE);
	vpcm_stop_cmd.hdr.src_port = voice_get_idx_for_session(v->session_id);
	vpcm_stop_cmd.hdr.dest_port = cvp_handle;
	vpcm_stop_cmd.hdr.token = 0;
	vpcm_stop_cmd.hdr.opcode = VSS_IVPCM_CMD_STOP;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *) &vpcm_stop_cmd);
	if (ret < 0) {
		pr_err("Fail: sending vocpcm stop,\n");
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
			(v->cvp_state == CMD_STATUS_SUCCESS),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

done:
	return ret;
}
EXPORT_SYMBOL(voc_send_cvp_stop_vocpcm);

/**
 * voc_send_cvp_map_vocpcm_memory -
 *       command to map memory for voice hpcm
 *
 * @session_id: voice session ID to send this command
 * @tp_mem_table: tap point memory table of hpcm
 * paddr: Physical address of hpcm memory mapped area.
 * bufsize: Buffer size of memory mapped area
 *
 * Returns 0 on success or error on failure
 */
int voc_send_cvp_map_vocpcm_memory(uint32_t session_id,
				   struct mem_map_table *tp_mem_table,
				   phys_addr_t paddr, uint32_t bufsize)
{
	return  voice_map_memory_physical_cmd(voice_get_session(session_id),
					      tp_mem_table,
					      (dma_addr_t) paddr, bufsize,
					      VOC_VOICE_HOST_PCM_MAP_TOKEN);
}
EXPORT_SYMBOL(voc_send_cvp_map_vocpcm_memory);

/**
 * voc_send_cvp_unmap_vocpcm_memory -
 *       command to unmap memory for voice hpcm
 *
 * @session_id: voice session ID to send this command
 *
 * Returns 0 on success or error on failure
 */
int voc_send_cvp_unmap_vocpcm_memory(uint32_t session_id)
{
	int ret = 0;

	ret =  voice_send_mvm_unmap_memory_physical_cmd(
				voice_get_session(session_id),
				common.voice_host_pcm_mem_handle);

	if (ret == 0)
		common.voice_host_pcm_mem_handle = 0;

	return ret;
}
EXPORT_SYMBOL(voc_send_cvp_unmap_vocpcm_memory);

/**
 * voc_send_cvp_vocpcm_push_buf_evt - Send buf event command
 *
 * @session_id: voice session ID to send this command
 * @push_buff_evt: pointer with buffer event details
 *
 * Returns 0 on success or error on failure
 */
int voc_send_cvp_vocpcm_push_buf_evt(uint32_t session_id,
			struct vss_ivpcm_evt_push_buffer_v2_t *push_buff_evt)
{
	struct cvp_push_buf_cmd vpcm_push_buf_cmd;
	int ret = 0;
	void *apr_cvp;
	u16 cvp_handle;
	struct voice_data *v = voice_get_session(session_id);

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}
	apr_cvp = common.apr_q6_cvp;

	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	memset(&vpcm_push_buf_cmd, 0, sizeof(vpcm_push_buf_cmd));
	cvp_handle = voice_get_cvp_handle(v);

	/* fill in the header */
	vpcm_push_buf_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	vpcm_push_buf_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
				sizeof(vpcm_push_buf_cmd) - APR_HDR_SIZE);
	vpcm_push_buf_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	vpcm_push_buf_cmd.hdr.dest_port = cvp_handle;
	vpcm_push_buf_cmd.hdr.token = 0;
	vpcm_push_buf_cmd.hdr.opcode = VSS_IVPCM_EVT_PUSH_BUFFER_V2;

	vpcm_push_buf_cmd.vpcm_evt_push_buffer.tap_point =
					push_buff_evt->tap_point;
	vpcm_push_buf_cmd.vpcm_evt_push_buffer.push_buf_mask =
					push_buff_evt->push_buf_mask;
	vpcm_push_buf_cmd.vpcm_evt_push_buffer.out_buf_mem_address =
					push_buff_evt->out_buf_mem_address;
	vpcm_push_buf_cmd.vpcm_evt_push_buffer.in_buf_mem_address =
					push_buff_evt->in_buf_mem_address;
	vpcm_push_buf_cmd.vpcm_evt_push_buffer.out_buf_mem_size =
					push_buff_evt->out_buf_mem_size;
	vpcm_push_buf_cmd.vpcm_evt_push_buffer.in_buf_mem_size =
					push_buff_evt->in_buf_mem_size;
	vpcm_push_buf_cmd.vpcm_evt_push_buffer.sampling_rate =
					push_buff_evt->sampling_rate;
	vpcm_push_buf_cmd.vpcm_evt_push_buffer.num_in_channels =
					push_buff_evt->num_in_channels;

	ret = apr_send_pkt(apr_cvp, (uint32_t *) &vpcm_push_buf_cmd);
	if (ret < 0) {
		pr_err("Fail: sending vocpcm map memory,\n");
		goto done;
	}

done:
	return ret;
}
EXPORT_SYMBOL(voc_send_cvp_vocpcm_push_buf_evt);

/**
 * voc_register_hpcm_evt_cb - Updates hostpcm info.
 *
 * @hostpcm_cb: callback function for hostpcm event
 * @private_data: private data for hostpcm
 *
 */
void voc_register_hpcm_evt_cb(hostpcm_cb_fn hostpcm_cb,
			      void *private_data)
{
	common.hostpcm_info.hostpcm_evt_cb = hostpcm_cb;
	common.hostpcm_info.private_data = private_data;
}
EXPORT_SYMBOL(voc_register_hpcm_evt_cb);

/**
 * voc_deregister_hpcm_evt_cb - resets hostpcm info.
 *
 */
void voc_deregister_hpcm_evt_cb(void)
{
	common.hostpcm_info.hostpcm_evt_cb = NULL;
	common.hostpcm_info.private_data = NULL;
}
EXPORT_SYMBOL(voc_deregister_hpcm_evt_cb);

/**
 * voc_get_cvd_version - retrieve CVD version.
 *
 * @cvd_version: pointer to be updated with CVD version info.
 *
 * Returns 0 on success or error on failure
 */
int voc_get_cvd_version(char *cvd_version)
{
	int ret = 0;
	struct voice_data *v = voice_get_session(VOICE_SESSION_VSID);


	if (v == NULL) {
		pr_err("%s: invalid session_id 0x%x\n",
		       __func__, VOICE_SESSION_VSID);

		ret = -EINVAL;
		goto done;
	}

	if (is_cvd_version_queried()) {
		pr_debug("%s: Returning the cached value %s\n",
			 __func__, common.cvd_version);

		goto done;
	}

	/* Register callback to APR */
	ret = voice_apr_register(VOICE_SESSION_VSID);
	if (ret < 0) {
		pr_err("%s: apr register failed\n", __func__);
		goto done;
	}

	mutex_lock(&common.common_lock);
	mutex_lock(&v->lock);
	ret = voice_send_mvm_cvd_version_cmd(v);
	if (ret < 0) {
		pr_err("%s: voice_send_mvm_cvd_version_cmd failed\n", __func__);
		goto unlock;
	}
	ret = 0;

unlock:
	mutex_unlock(&v->lock);
	mutex_unlock(&common.common_lock);

done:
	if (cvd_version)
		memcpy(cvd_version, common.cvd_version,
		       CVD_VERSION_STRING_MAX_SIZE);

	return ret;
}
EXPORT_SYMBOL(voc_get_cvd_version);

static int voice_alloc_cal_mem_map_table(void)
{
	int ret = 0;
	size_t len;

	ret = msm_audio_ion_alloc(&(common.cal_mem_map_table.dma_buf),
				sizeof(struct vss_imemory_table_t),
				&common.cal_mem_map_table.phys,
				&len,
				&(common.cal_mem_map_table.data));
	if ((ret) && (ret != -EPROBE_DEFER)) {
		pr_err("%s: audio ION alloc failed, rc = %d\n",
			__func__, ret);
		ret = -EINVAL;
		goto done;
	}

	common.cal_mem_map_table.size = sizeof(struct vss_imemory_table_t);
	pr_debug("%s: data %pK phys %pK\n", __func__,
		 common.cal_mem_map_table.data,
		 &common.cal_mem_map_table.phys);

done:
	return ret;
}

static int voice_alloc_rtac_mem_map_table(void)
{
	int ret = 0;
	size_t len;

	ret = msm_audio_ion_alloc(
			&(common.rtac_mem_map_table.dma_buf),
			sizeof(struct vss_imemory_table_t),
			&common.rtac_mem_map_table.phys,
			&len,
			&(common.rtac_mem_map_table.data));
	if (ret) {
		pr_err("%s: audio ION alloc failed, rc = %d\n",
			__func__, ret);
		ret = -EINVAL;
		goto done;
	}

	common.rtac_mem_map_table.size = sizeof(struct vss_imemory_table_t);
	pr_debug("%s: data %pK phys %pK\n", __func__,
		 common.rtac_mem_map_table.data,
		 &common.rtac_mem_map_table.phys);

done:
	return ret;
}

static int voice_alloc_and_map_oob_mem(struct voice_data *v)
{
	int ret = 0;

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		return -EINVAL;
	}

	if (!is_voip_memory_allocated()) {
		ret = voc_alloc_voip_shared_memory();
		if (ret < 0) {
			pr_err("%s: Failed to create voip oob memory %d\n",
				   __func__, ret);

			goto done;
		}
	}

	ret = voice_map_memory_physical_cmd(v,
			&v->shmem_info.memtbl,
			v->shmem_info.sh_buf.buf[0].phys,
			v->shmem_info.sh_buf.buf[0].size * NUM_OF_BUFFERS,
			VOIP_MEM_MAP_TOKEN);
	if (ret) {
		pr_err("%s: mvm_map_memory_phy failed %d\n",
			   __func__, ret);

		goto done;
	}

done:
	return ret;
}

uint32_t voice_get_topology(uint32_t topology_idx)
{
	uint32_t topology = VSS_IVOCPROC_TOPOLOGY_ID_RX_DEFAULT;
	struct cal_block_data *cal_block = NULL;

	/* initialize as default topology */
	if (topology_idx == CVP_VOC_RX_TOPOLOGY_CAL) {
		topology = VSS_IVOCPROC_TOPOLOGY_ID_RX_DEFAULT;
	} else if (topology_idx == CVP_VOC_TX_TOPOLOGY_CAL) {
		topology = VSS_IVOCPROC_TOPOLOGY_ID_NONE;
	} else {
		pr_err("%s: cal index %x is invalid!\n",
			__func__, topology_idx);

		goto done;
	}

	if (common.cal_data[topology_idx] == NULL) {
		pr_err("%s: cal type is NULL for cal index %x\n",
			__func__, topology_idx);

		goto done;
	}

	mutex_lock(&common.cal_data[topology_idx]->lock);
	cal_block = cal_utils_get_only_cal_block(
		common.cal_data[topology_idx]);
	if (cal_block == NULL) {
		pr_debug("%s: cal_block not found for cal index %x\n",
			__func__, topology_idx);

		goto unlock;
	}

	topology = ((struct audio_cal_info_voc_top *)
		cal_block->cal_info)->topology;
unlock:
	mutex_unlock(&common.cal_data[topology_idx]->lock);
done:
	pr_debug("%s: Using topology %d\n", __func__, topology);

	return topology;
}

int voice_set_topology_specific_info(struct voice_data *v,
				     uint32_t topology_idx)
{
	struct cal_block_data *cal_block = NULL;
	int ret = 0;
	uint32_t topo_channels;

	if (common.cal_data[topology_idx] == NULL) {
		pr_err("%s: cal type is NULL for cal index %x\n",
			__func__, topology_idx);
		ret = -EINVAL;
		goto done;
	}

	mutex_lock(&common.cal_data[topology_idx]->lock);
	cal_block = cal_utils_get_only_cal_block(
		common.cal_data[topology_idx]);
	if (cal_block == NULL) {
		pr_debug("%s: cal_block not found for cal index %x\n",
			__func__, topology_idx);
		ret = -EINVAL;
		goto unlock;
	}

	if (topology_idx == CVP_VOC_RX_TOPOLOGY_CAL) {
		topo_channels = ((struct audio_cal_info_voc_top *)
				cal_block->cal_info)->num_channels;
		if (topo_channels > 0) {
			v->dev_rx.no_of_channels = topo_channels;
			pr_debug("%s: Topology Rx no of channels: %d",
				 __func__, v->dev_rx.no_of_channels);
			memcpy(&v->dev_rx.channel_mapping,
			       &((struct audio_cal_info_voc_top *)
			       cal_block->cal_info)->channel_mapping,
			       VSS_CHANNEL_MAPPING_SIZE);
		} else {
			pr_debug("%s: cal data is zero, default to Rx backend config\n",
				 __func__);
			if (v->dev_rx.no_of_channels == NUM_CHANNELS_MONO) {
				v->dev_rx.channel_mapping[0] = PCM_CHANNEL_FC;
			} else if (v->dev_rx.no_of_channels ==
							NUM_CHANNELS_STEREO) {
				v->dev_rx.channel_mapping[0] = PCM_CHANNEL_FL;
				v->dev_rx.channel_mapping[1] = PCM_CHANNEL_FR;
			} else {
				pr_warn("%s: Unsupported Rx num channels: %d\n",
					__func__, v->dev_rx.no_of_channels);
			}
		}
	} else if (topology_idx == CVP_VOC_TX_TOPOLOGY_CAL) {
		topo_channels = ((struct audio_cal_info_voc_top *)
				cal_block->cal_info)->num_channels;
		if (topo_channels > 0) {
			v->dev_tx.no_of_channels = topo_channels;
			pr_debug("%s: Topology Tx no of channels: %d",
				 __func__, v->dev_tx.no_of_channels);
			memcpy(&v->dev_tx.channel_mapping,
			       &((struct audio_cal_info_voc_top *)
			       cal_block->cal_info)->channel_mapping,
			       VSS_CHANNEL_MAPPING_SIZE);
		} else {
			pr_debug("%s: cal data is zero, default to Tx backend config\n",
				 __func__);
			if (v->dev_tx.no_of_channels == NUM_CHANNELS_MONO) {
				v->dev_tx.channel_mapping[0] = PCM_CHANNEL_FC;
			} else if (v->dev_tx.no_of_channels ==
							NUM_CHANNELS_STEREO) {
				v->dev_tx.channel_mapping[0] = PCM_CHANNEL_FL;
				v->dev_tx.channel_mapping[1] = PCM_CHANNEL_FR;
			} else if (v->dev_tx.no_of_channels ==
							NUM_CHANNELS_THREE) {
				v->dev_tx.channel_mapping[0] = PCM_CHANNEL_FL;
				v->dev_tx.channel_mapping[1] = PCM_CHANNEL_FR;
				v->dev_tx.channel_mapping[2] = PCM_CHANNEL_FC;
			} else if (v->dev_tx.no_of_channels ==
							NUM_CHANNELS_QUAD) {
				v->dev_tx.channel_mapping[0] = PCM_CHANNEL_FL;
				v->dev_tx.channel_mapping[1] = PCM_CHANNEL_FR;
				v->dev_tx.channel_mapping[2] = PCM_CHANNEL_LS;
				v->dev_tx.channel_mapping[3] = PCM_CHANNEL_RS;
			} else {
				pr_warn("%s: Unsupported Tx num channels: %d\n",
					__func__, v->dev_tx.no_of_channels);
			}
		}
	} else {
		pr_err("%s: topology index %x is invalid\n",
		       __func__, topology_idx);
	}
unlock:
	mutex_unlock(&common.cal_data[topology_idx]->lock);
done:
	return ret;
}

static int get_cal_type_index(int32_t cal_type)
{
	int ret = -EINVAL;

	switch (cal_type) {
	case CVP_VOC_RX_TOPOLOGY_CAL_TYPE:
		ret = CVP_VOC_RX_TOPOLOGY_CAL;
		break;
	case CVP_VOC_TX_TOPOLOGY_CAL_TYPE:
		ret = CVP_VOC_TX_TOPOLOGY_CAL;
		break;
	case CVP_VOCPROC_STATIC_CAL_TYPE:
		ret = CVP_VOCPROC_CAL;
		break;
	case CVP_VOCPROC_DYNAMIC_CAL_TYPE:
		ret = CVP_VOCVOL_CAL;
		break;
	case CVS_VOCSTRM_STATIC_CAL_TYPE:
		ret = CVS_VOCSTRM_CAL;
		break;
	case CVP_VOCDEV_CFG_CAL_TYPE:
		ret = CVP_VOCDEV_CFG_CAL;
		break;
	case CVP_VOCPROC_STATIC_COL_CAL_TYPE:
		ret = CVP_VOCPROC_COL_CAL;
		break;
	case CVP_VOCPROC_DYNAMIC_COL_CAL_TYPE:
		ret = CVP_VOCVOL_COL_CAL;
		break;
	case CVS_VOCSTRM_STATIC_COL_CAL_TYPE:
		ret = CVS_VOCSTRM_COL_CAL;
		break;
	case VOICE_RTAC_INFO_CAL_TYPE:
		ret = VOICE_RTAC_INFO_CAL;
		break;
	case VOICE_RTAC_APR_CAL_TYPE:
		ret = VOICE_RTAC_APR_CAL;
		break;
	default:
		pr_err("%s: Invalid cal type %d!\n", __func__, cal_type);
	}
	return ret;
}

static int voice_prepare_volume_boost(int32_t cal_type,
					size_t data_size, void *data)
{
	return voc_deregister_vocproc_vol_table();
}

static int voice_enable_volume_boost(int32_t cal_type,
				size_t data_size, void *data)
{
	return voc_register_vocproc_vol_table();
}

static int voice_alloc_cal(int32_t cal_type,
			   size_t data_size, void *data)
{
	int ret = 0;
	int cal_index;
	int cal_version;

	pr_debug("%s\n", __func__);

	cal_version = cal_utils_get_cal_type_version(data);
	common.is_per_vocoder_cal_enabled =
			!!(cal_version & PER_VOCODER_CAL_BIT_MASK);

	cal_index = get_cal_type_index(cal_type);
	if (cal_index < 0) {
		pr_err("%s: Could not get cal index %d!\n",
			__func__, cal_index);
		ret = -EINVAL;
		goto done;
	}

	ret = cal_utils_alloc_cal(data_size, data,
		common.cal_data[cal_index], 0, NULL);
	if (ret < 0) {
		pr_err("%s: Cal_utils_alloc_block failed, ret = %d, cal type = %d!\n",
			__func__, ret, cal_type);
		ret = -EINVAL;
		goto done;
	}
done:
	return ret;
}

static int voice_dealloc_cal(int32_t cal_type,
			     size_t data_size, void *data)
{
	int ret = 0;
	int cal_index;

	pr_debug("%s\n", __func__);

	cal_index = get_cal_type_index(cal_type);
	if (cal_index < 0) {
		pr_err("%s: Could not get cal index %d!\n",
			__func__, cal_index);

		ret = -EINVAL;
		goto done;
	}

	ret = cal_utils_dealloc_cal(data_size, data,
		common.cal_data[cal_index]);
	if (ret < 0) {
		pr_err("%s: Cal_utils_dealloc_block failed, ret = %d, cal type = %d!\n",
			__func__, ret, cal_type);

		ret = -EINVAL;
		goto done;
	}
done:
	return ret;
}

static int voice_set_cal(int32_t cal_type,
			 size_t data_size, void *data)
{
	int ret = 0;
	int cal_index;

	pr_debug("%s\n", __func__);

	cal_index = get_cal_type_index(cal_type);
	if (cal_index < 0) {
		pr_err("%s: Could not get cal index %d!\n",
			__func__, cal_index);

		ret = -EINVAL;
		goto done;
	}

	ret = cal_utils_set_cal(data_size, data,
		common.cal_data[cal_index], 0, NULL);
	if (ret < 0) {
		pr_err("%s: Cal_utils_set_cal failed, ret = %d, cal type = %d!\n",
			__func__, ret, cal_type);

		ret = -EINVAL;
		goto done;
	}
	/* Pre-load if it is voice Rx or Tx topology */
	if ((cal_index == CVP_VOC_RX_TOPOLOGY_CAL) ||
		(cal_index == CVP_VOC_TX_TOPOLOGY_CAL)) {
		voice_load_topo_modules(cal_index);
	}
done:
	return ret;
}

static void voice_delete_cal_data(void)
{
	pr_debug("%s\n", __func__);

	cal_utils_destroy_cal_types(MAX_VOICE_CAL_TYPES, common.cal_data);
}

static int voice_init_cal_data(void)
{
	int ret = 0;
	struct cal_type_info cal_type_info[] = {
		{{CVP_VOC_RX_TOPOLOGY_CAL_TYPE,
		{NULL, NULL, NULL, voice_set_cal, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} },

		{{CVP_VOC_TX_TOPOLOGY_CAL_TYPE,
		{NULL, NULL, NULL, voice_set_cal, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} },

		{{CVP_VOCPROC_STATIC_CAL_TYPE,
		{voice_alloc_cal, voice_dealloc_cal, NULL,
		voice_set_cal, NULL, NULL} },
		{NULL, voice_unmap_cal_memory,
		cal_utils_match_buf_num} },

		{{CVP_VOCPROC_DYNAMIC_CAL_TYPE,
		{voice_alloc_cal, voice_dealloc_cal,
		voice_prepare_volume_boost,
		voice_set_cal, NULL,
		voice_enable_volume_boost} },
		{NULL, voice_unmap_cal_memory,
		cal_utils_match_buf_num} },

		{{CVP_VOCDEV_CFG_CAL_TYPE,
		{voice_alloc_cal, voice_dealloc_cal, NULL,
		voice_set_cal, NULL, NULL} },
		{NULL, voice_unmap_cal_memory,
		cal_utils_match_buf_num} },

		{{CVP_VOCPROC_STATIC_COL_CAL_TYPE,
		{NULL, NULL, NULL, voice_set_cal, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} },

		{{CVP_VOCPROC_DYNAMIC_COL_CAL_TYPE,
		{NULL, NULL, NULL, voice_set_cal, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} },

		{{CVS_VOCSTRM_STATIC_CAL_TYPE,
		{voice_alloc_cal, voice_dealloc_cal, NULL,
		voice_set_cal, NULL, NULL} },
		{NULL, voice_unmap_cal_memory,
		cal_utils_match_buf_num} },

		{{CVS_VOCSTRM_STATIC_COL_CAL_TYPE,
		{NULL, NULL, NULL, voice_set_cal, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} },

		{{VOICE_RTAC_INFO_CAL_TYPE,
		{NULL, NULL, NULL, NULL, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} },

		{{VOICE_RTAC_APR_CAL_TYPE,
		{NULL, NULL, NULL, NULL, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} },
	};

	ret = cal_utils_create_cal_types(MAX_VOICE_CAL_TYPES, common.cal_data,
		cal_type_info);
	if (ret < 0) {
		pr_err("%s: Could not create cal type!\n",
			__func__);

		ret = -EINVAL;
		goto err;
	}

	return ret;
err:
	voice_delete_cal_data();
	memset(&common, 0, sizeof(struct common_data));
	return ret;
}

static int voice_send_set_sound_focus_cmd(struct voice_data *v,
				 struct sound_focus_param soundFocusData)
{
	struct cvp_set_sound_focus_param_cmd_t cvp_set_sound_focus_param_cmd;
	int ret = 0;
	void *apr_cvp;
	u16 cvp_handle;
	int i;

	pr_debug("%s: Enter\n", __func__);

	if (v == NULL) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	apr_cvp = common.apr_q6_cvp;

	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	cvp_handle = voice_get_cvp_handle(v);

	/* send Sound Focus Params to cvp */
	cvp_set_sound_focus_param_cmd.hdr.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);
	cvp_set_sound_focus_param_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
			sizeof(cvp_set_sound_focus_param_cmd) - APR_HDR_SIZE);
	cvp_set_sound_focus_param_cmd.hdr.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_set_sound_focus_param_cmd.hdr.dest_port = cvp_handle;
	cvp_set_sound_focus_param_cmd.hdr.token = 0;
	cvp_set_sound_focus_param_cmd.hdr.opcode =
					 VSS_ISOUNDFOCUS_CMD_SET_SECTORS;

	memset(&(cvp_set_sound_focus_param_cmd.cvp_set_sound_focus_param), 0xFF,
		sizeof(struct vss_isoundfocus_cmd_set_sectors_t));
	for (i = 0; i < MAX_SECTORS; i++) {
		cvp_set_sound_focus_param_cmd.cvp_set_sound_focus_param.
			start_angles[i] = soundFocusData.start_angle[i];
		cvp_set_sound_focus_param_cmd.cvp_set_sound_focus_param.
			enables[i] = soundFocusData.enable[i];
		pr_debug("%s: start_angle[%d] = %d\n",
			  __func__, i, soundFocusData.start_angle[i]);
		pr_debug("%s: enable[%d] = %d\n",
			  __func__, i, soundFocusData.enable[i]);
	}
	cvp_set_sound_focus_param_cmd.cvp_set_sound_focus_param.gain_step =
					soundFocusData.gain_step;
	pr_debug("%s: gain_step = %d\n", __func__, soundFocusData.gain_step);

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;

	ret = apr_send_pkt(apr_cvp, (uint32_t *)&cvp_set_sound_focus_param_cmd);
	if (ret < 0) {
		pr_err("%s: Error in sending APR command\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

	if (common.is_sound_focus_resp_success) {
		ret = 0;
	} else {
		pr_err("%s: Error in setting sound focus params\n", __func__);

		ret = -EINVAL;
	}

done:
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}

/**
 * voc_set_sound_focus - sends sound focus data.
 *
 * @soundFocusData: sound focus data.
 *
 * Returns 0 on success or error on failure
 */
int voc_set_sound_focus(struct sound_focus_param soundFocusData)
{
	struct voice_data *v = NULL;
	int ret = -EINVAL;
	struct voice_session_itr itr;

	pr_debug("%s: Enter\n", __func__);

	mutex_lock(&common.common_lock);
	voice_itr_init(&itr, ALL_SESSION_VSID);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v != NULL) {
			mutex_lock(&v->lock);
			if (is_voc_state_active(v->voc_state) &&
				(v->lch_mode != VOICE_LCH_START) &&
				!v->disable_topology)
				ret = voice_send_set_sound_focus_cmd(v,
							soundFocusData);
			mutex_unlock(&v->lock);
		} else {
			pr_err("%s: invalid session\n", __func__);

			ret = -EINVAL;
			break;
		}
	}
	mutex_unlock(&common.common_lock);
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL(voc_set_sound_focus);

static int voice_send_get_sound_focus_cmd(struct voice_data *v,
				struct sound_focus_param *soundFocusData)
{
	struct apr_hdr cvp_get_sound_focus_param_cmd;
	int ret = 0;
	void *apr_cvp;
	u16 cvp_handle;
	int i;

	pr_debug("%s: Enter\n", __func__);

	if (!v) {
		pr_err("%s: v is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	apr_cvp = common.apr_q6_cvp;

	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	cvp_handle = voice_get_cvp_handle(v);

	/* send APR command to retrieve Sound Focus Params */
	cvp_get_sound_focus_param_cmd.hdr_field =
				APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					      APR_HDR_LEN(APR_HDR_SIZE),
					      APR_PKT_VER);
	cvp_get_sound_focus_param_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
			sizeof(cvp_get_sound_focus_param_cmd) - APR_HDR_SIZE);
	cvp_get_sound_focus_param_cmd.src_port =
				voice_get_idx_for_session(v->session_id);
	cvp_get_sound_focus_param_cmd.dest_port = cvp_handle;
	cvp_get_sound_focus_param_cmd.token = 0;
	cvp_get_sound_focus_param_cmd.opcode = VSS_ISOUNDFOCUS_CMD_GET_SECTORS;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (uint32_t *)&cvp_get_sound_focus_param_cmd);
	if (ret < 0) {
		pr_err("%s: Error in sending APR command\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

	if (common.is_sound_focus_resp_success) {
		for (i = 0; i < MAX_SECTORS; i++) {
			soundFocusData->start_angle[i] =
				common.soundFocusResponse.start_angles[i];
			soundFocusData->enable[i] =
				common.soundFocusResponse.enables[i];
			pr_debug("%s: start_angle[%d] = %d\n",
				  __func__, i, soundFocusData->start_angle[i]);
			pr_debug("%s: enable[%d] = %d\n",
				  __func__, i, soundFocusData->enable[i]);
		}
		soundFocusData->gain_step = common.soundFocusResponse.gain_step;
		pr_debug("%s: gain_step = %d\n", __func__,
			  soundFocusData->gain_step);

		common.is_sound_focus_resp_success = false;
		ret = 0;
	} else {
		pr_err("%s: Invalid payload received from CVD\n", __func__);

		ret = -EINVAL;
	}
done:
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}

/**
 * voc_get_sound_focus - retrieves sound focus data.
 *
 * @soundFocusData: pointer to be updated with sound focus data.
 *
 * Returns 0 on success or error on failure
 */
int voc_get_sound_focus(struct sound_focus_param *soundFocusData)
{
	struct voice_data *v = NULL;
	int ret = -EINVAL;
	struct voice_session_itr itr;

	pr_debug("%s: Enter\n", __func__);

	mutex_lock(&common.common_lock);
	voice_itr_init(&itr, ALL_SESSION_VSID);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v) {
			mutex_lock(&v->lock);
			if (is_voc_state_active(v->voc_state) &&
				(v->lch_mode != VOICE_LCH_START) &&
				!v->disable_topology)
				ret = voice_send_get_sound_focus_cmd(v,
							soundFocusData);
			mutex_unlock(&v->lock);
		} else {
			pr_err("%s: invalid session\n", __func__);

			ret =  -EINVAL;
			break;
		}
	}
	mutex_unlock(&common.common_lock);
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL(voc_get_sound_focus);

static bool is_source_tracking_shared_memomry_allocated(void)
{
	bool ret;

	pr_debug("%s: Enter\n", __func__);

	if (common.source_tracking_sh_mem.sh_mem_block.dma_buf != NULL)
		ret = true;
	else
		ret = false;

	pr_debug("%s: Exit\n", __func__);

	return ret;
}

static int voice_alloc_source_tracking_shared_memory(void)
{
	int ret = 0;

	pr_debug("%s: Enter\n", __func__);

	ret = msm_audio_ion_alloc(
		&(common.source_tracking_sh_mem.sh_mem_block.dma_buf),
		BUFFER_BLOCK_SIZE,
		&(common.source_tracking_sh_mem.sh_mem_block.phys),
		(size_t *)&(common.source_tracking_sh_mem.sh_mem_block.size),
		&(common.source_tracking_sh_mem.sh_mem_block.data));
	if (ret) {
		pr_err("%s: audio ION alloc failed for sh_mem block, ret = %d\n",
			__func__, ret);

		ret = -EINVAL;
		goto done;
	}
	memset((void *)(common.source_tracking_sh_mem.sh_mem_block.data), 0,
		   common.source_tracking_sh_mem.sh_mem_block.size);

	pr_debug("%s: sh_mem_block: phys:[%pK], data:[0x%pK], size:[%zd]\n",
		 __func__,
		&(common.source_tracking_sh_mem.sh_mem_block.phys),
		(void *)(common.source_tracking_sh_mem.sh_mem_block.data),
		(size_t)(common.source_tracking_sh_mem.sh_mem_block.size));

	ret = msm_audio_ion_alloc(
		&(common.source_tracking_sh_mem.sh_mem_table.dma_buf),
		sizeof(struct vss_imemory_table_t),
		&(common.source_tracking_sh_mem.sh_mem_table.phys),
		(size_t *)&(common.source_tracking_sh_mem.sh_mem_table.size),
		&(common.source_tracking_sh_mem.sh_mem_table.data));
	if (ret) {
		pr_err("%s: audio ION alloc failed for sh_mem table, ret = %d\n",
			__func__, ret);

		ret = msm_audio_ion_free(
			common.source_tracking_sh_mem.sh_mem_block.dma_buf);
		common.source_tracking_sh_mem.sh_mem_block.dma_buf = NULL;
		if (ret < 0)
			pr_err("%s: Error:%d freeing memory\n", __func__, ret);

		ret = -EINVAL;
		goto done;
	}
	memset((void *)(common.source_tracking_sh_mem.sh_mem_table.data), 0,
		common.source_tracking_sh_mem.sh_mem_table.size);

	pr_debug("%s sh_mem_table: phys:[%pK], data:[0x%pK], size:[%zd],\n",
		 __func__,
		&(common.source_tracking_sh_mem.sh_mem_table.phys),
		(void *)(common.source_tracking_sh_mem.sh_mem_table.data),
		(size_t)(common.source_tracking_sh_mem.sh_mem_table.size));

done:
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}

static int voice_alloc_and_map_source_tracking_shared_memory(
						struct voice_data *v)
{
	int ret = 0;

	pr_debug("%s: Enter\n", __func__);

	ret = voice_alloc_source_tracking_shared_memory();
	if (ret) {
		pr_err("%s: Failed to allocate shared memory %d\n",
			__func__, ret);

		ret = -EINVAL;
		goto done;
	}

	ret = voice_map_memory_physical_cmd(v,
			&(common.source_tracking_sh_mem.sh_mem_table),
			common.source_tracking_sh_mem.sh_mem_block.phys,
			common.source_tracking_sh_mem.sh_mem_block.size,
			VOC_SOURCE_TRACKING_MEM_MAP_TOKEN);
	if (ret) {
		pr_err("%s: memory mapping failed %d\n",
			__func__, ret);

		ret = -EINVAL;
		goto done;
	}

done:
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}

static int voice_unmap_and_free_source_tracking_shared_memory(
							struct voice_data *v)
{
	int ret = 0;

	pr_debug("%s: Enter\n", __func__);

	if (common.source_tracking_sh_mem.mem_handle != 0) {
		ret = voice_send_mvm_unmap_memory_physical_cmd(v,
				common.source_tracking_sh_mem.mem_handle);
		if (ret < 0) {
			pr_err("%s: Memory_unmap failed err %d\n",
				 __func__, ret);

			ret = -EINVAL;
			goto done;
		}
	}

	if (common.source_tracking_sh_mem.sh_mem_block.dma_buf == NULL)
		goto done;

	ret = msm_audio_ion_free(
			common.source_tracking_sh_mem.sh_mem_block.dma_buf);
	if (ret < 0) {
		pr_err("%s: Error:%d freeing memory\n", __func__, ret);

		ret = -EINVAL;
		goto done;
	}

done:
	common.source_tracking_sh_mem.mem_handle = 0;
	common.source_tracking_sh_mem.sh_mem_block.dma_buf = NULL;
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}

static int voice_send_get_source_tracking_cmd(struct voice_data *v,
			struct source_tracking_param *sourceTrackingData)
{
	struct cvp_get_source_tracking_param_cmd_t st_cmd;
	int ret = 0;
	void *apr_cvp;
	u16 cvp_handle;
	int i;

	pr_debug("%s: Enter\n", __func__);

	if (!v) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}
	apr_cvp = common.apr_q6_cvp;

	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);
		return -EINVAL;
	}

	cvp_handle = voice_get_cvp_handle(v);

	if (!is_source_tracking_shared_memomry_allocated()) {
		ret = voice_alloc_and_map_source_tracking_shared_memory(v);
		if (ret) {
			pr_err("%s: Fail in allocating/mapping shared memory\n",
				__func__);

			ret = -EINVAL;
			goto done;
		}
	}
	st_cmd.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					     APR_HDR_LEN(APR_HDR_SIZE),
					     APR_PKT_VER);
	st_cmd.hdr.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					   sizeof(st_cmd) - APR_HDR_SIZE);
	st_cmd.hdr.src_port = voice_get_idx_for_session(v->session_id);
	st_cmd.hdr.dest_port = cvp_handle;
	st_cmd.hdr.token = 0;
	st_cmd.hdr.opcode = VSS_ISOURCETRACK_CMD_GET_ACTIVITY;

	st_cmd.cvp_get_source_tracking_param.mem_handle	=
				 common.source_tracking_sh_mem.mem_handle;
	st_cmd.cvp_get_source_tracking_param.mem_address_lsw =
		lower_32_bits(common.source_tracking_sh_mem.sh_mem_block.phys);
	st_cmd.cvp_get_source_tracking_param.mem_address_msw =
		msm_audio_populate_upper_32_bits(common.source_tracking_sh_mem.
					sh_mem_block.phys);
	st_cmd.cvp_get_source_tracking_param.mem_size =
		(uint32_t)common.source_tracking_sh_mem.sh_mem_block.size;
	pr_debug("%s: mem_handle=0x%x, mem_address_lsw=0x%x, msw=0x%x, mem_size=%d\n",
		 __func__,
		 st_cmd.cvp_get_source_tracking_param.mem_handle,
		 st_cmd.cvp_get_source_tracking_param.mem_address_lsw,
		 st_cmd.cvp_get_source_tracking_param.mem_address_msw,
		 (uint32_t)st_cmd.cvp_get_source_tracking_param.mem_size);

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp,
			   (uint32_t *) &st_cmd);
	if (ret < 0) {
		pr_err("%s: Error in sending APR command\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

	if (common.is_source_tracking_resp_success) {
		for (i = 0; i < MAX_SECTORS; i++) {
			sourceTrackingData->vad[i] =
				common.sourceTrackingResponse.voice_active[i];
			pr_debug("%s: vad[%d] = %d\n",
				  __func__, i, sourceTrackingData->vad[i]);
		}
		sourceTrackingData->doa_speech =
				common.sourceTrackingResponse.talker_doa;
		pr_debug("%s: doa_speech = %d\n",
			  __func__, sourceTrackingData->doa_speech);

		for (i = 0; i < MAX_NOISE_SOURCE_INDICATORS; i++) {
			sourceTrackingData->doa_noise[i] =
			 common.sourceTrackingResponse.interferer_doa[i];
			pr_debug("%s: doa_noise[%d] = %d\n",
			 __func__, i, sourceTrackingData->doa_noise[i]);
		}
		for (i = 0; i < MAX_POLAR_ACTIVITY_INDICATORS; i++) {
			sourceTrackingData->polar_activity[i] =
			 common.sourceTrackingResponse.sound_strength[i];
			pr_debug("%s: polar_activity[%d] = %d\n",
			 __func__, i, sourceTrackingData->polar_activity[i]);
		}
		common.is_source_tracking_resp_success = false;
		ret = 0;
	} else {
		pr_err("%s: Error response received from CVD\n", __func__);

		ret = -EINVAL;
	}
done:
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}

/**
 * voc_get_source_tracking - retrieves source track data.
 *
 * @sourceTrackingData: pointer to be updated with source track data.
 *
 * Returns 0 on success or error on failure
 */
int voc_get_source_tracking(struct source_tracking_param *sourceTrackingData)
{
	struct voice_data *v = NULL;
	int ret = -EINVAL;
	struct voice_session_itr itr;

	pr_debug("%s: Enter\n", __func__);

	mutex_lock(&common.common_lock);

	voice_itr_init(&itr, ALL_SESSION_VSID);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v != NULL) {
			mutex_lock(&v->lock);
			if (is_voc_state_active(v->voc_state) &&
				(v->lch_mode != VOICE_LCH_START) &&
				!v->disable_topology)
				ret = voice_send_get_source_tracking_cmd(v,
							sourceTrackingData);
			mutex_unlock(&v->lock);
		} else {
			pr_err("%s: invalid session\n", __func__);

			break;
		}
	}

	mutex_unlock(&common.common_lock);
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL(voc_get_source_tracking);

static int voice_send_get_fnn_source_track_cmd(struct voice_data *v,
			struct fluence_nn_source_tracking_param *FnnSourceTrackingData)
{
	struct apr_hdr cvp_get_fnn_st_param_cmd;
	int ret = 0;
	void *apr_cvp;
	u16 cvp_handle;
	int i;

	if (!v) {
		pr_err("%s: v is NULL\n", __func__);
		return -EINVAL;
	}

	apr_cvp = common.apr_q6_cvp;
	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL.\n", __func__);
		return -EINVAL;
	}

	if (!FnnSourceTrackingData) {
		pr_err("%s: Caught NULL pointer \n", __func__);
		ret = -EINVAL;
		goto done;
	}

	cvp_handle = voice_get_cvp_handle(v);

	/* send APR command to retrieve Sound Track Params from fluence NN Module */
	cvp_get_fnn_st_param_cmd.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					     APR_HDR_LEN(APR_HDR_SIZE),
					     APR_PKT_VER);
	cvp_get_fnn_st_param_cmd.pkt_size = APR_PKT_SIZE(APR_HDR_SIZE,
					   sizeof(cvp_get_fnn_st_param_cmd) - APR_HDR_SIZE);
	cvp_get_fnn_st_param_cmd.src_port = voice_get_idx_for_session(v->session_id);
	cvp_get_fnn_st_param_cmd.dest_port = cvp_handle;
	cvp_get_fnn_st_param_cmd.token = 0;
	cvp_get_fnn_st_param_cmd.opcode = VSS_IFNN_ST_CMD_GET_ACTIVITY;

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp,
			   (uint32_t *) &cvp_get_fnn_st_param_cmd);
	if (ret < 0) {
		pr_err("%s: Error in sending APR command\n", __func__);

		ret = -EINVAL;
		goto done;
	}
	ret = wait_event_timeout(v->cvp_wait,
				 (v->cvp_state == CMD_STATUS_SUCCESS),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);

		ret = -ETIME;
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				v->async_err));
		ret = adsp_err_get_lnx_err_code(
				v->async_err);
		goto done;
	}

	if (!common.is_fnn_source_tracking_resp_success) {
		pr_err("%s: Error response received from CVD\n", __func__);

		ret = -EINVAL;
		goto done;
	}

	FnnSourceTrackingData->speech_probablity_q20 =
		common.FnnSourceTrackingResponse.speech_probablity_q20;
	pr_debug("%s: speech_probablity = %d\n",
		  __func__, FnnSourceTrackingData->speech_probablity_q20);

	for (i = 0; i < MAX_TOP_SPEAKERS; i++) {
		FnnSourceTrackingData->speakers[i] =
		 common.FnnSourceTrackingResponse.speakers[i];
		pr_debug("%s: speakers[%d] = %d\n",
		 __func__, i, FnnSourceTrackingData->speakers[i]);
	}

	for (i = 0; i < MAX_POLAR_ACTIVITY_INDICATORS; i++) {
		FnnSourceTrackingData->polarActivity[i] =
		 common.FnnSourceTrackingResponse.polarActivity[i];
		pr_debug("%s: polarActivity[%d] = %d\n",
		 __func__, i, FnnSourceTrackingData->polarActivity[i]);
	}

	FnnSourceTrackingData->session_time_lsw =
			common.FnnSourceTrackingResponse.session_time_lsw;
	pr_debug("%s: session_time_lsw = %d\n",
		  __func__, FnnSourceTrackingData->session_time_lsw);

	FnnSourceTrackingData->session_time_msw =
			common.FnnSourceTrackingResponse.session_time_msw;
	pr_debug("%s: session_time_msw = %d\n",
		  __func__, FnnSourceTrackingData->session_time_msw);

	common.is_fnn_source_tracking_resp_success = false;

done:
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}

/**
 * voc_get_fnn_source_tracking - retrieves source track data.
 *
 * @sourceTrackingData: pointer to be updated with source track data.
 *
 * Returns 0 on success or error on failure
 */
int voc_get_fnn_source_tracking(struct fluence_nn_source_tracking_param *FnnSourceTrackingData)
{
	struct voice_data *v = NULL;
	int ret = -EINVAL;
	struct voice_session_itr itr;

	mutex_lock(&common.common_lock);

	voice_itr_init(&itr, ALL_SESSION_VSID);
	while (voice_itr_get_next_session(&itr, &v)) {
		if (v == NULL) {
			pr_err("%s: invalid session\n", __func__);

			break;
		}

		mutex_lock(&v->lock);
		if (is_voc_state_active(v->voc_state) &&
			(v->lch_mode != VOICE_LCH_START) &&
			!v->disable_topology)
			ret = voice_send_get_fnn_source_track_cmd(v,
					FnnSourceTrackingData);
		mutex_unlock(&v->lock);

	}

	mutex_unlock(&common.common_lock);
	pr_debug("%s: Exit, ret=%d\n", __func__, ret);

	return ret;
}
EXPORT_SYMBOL(voc_get_fnn_source_tracking);

static int voice_set_cvp_param(struct voice_data *v,
			       struct vss_icommon_mem_mapping_hdr *mem_hdr,
			       u32 *param_data, u32 param_size)
{
	struct vss_icommon_cmd_set_param *set_param = NULL;
	uint32_t pkt_size = sizeof(struct vss_icommon_cmd_set_param);
	void *apr_cvp;
	int ret = 0;

	apr_cvp = common.apr_q6_cvp;
	if (!apr_cvp) {
		pr_err("%s: apr_cvp is NULL\n", __func__);
		return -EINVAL;
	}

	if (param_data != NULL)
		pkt_size += param_size;
	set_param = kzalloc(pkt_size, GFP_KERNEL);
	if (!set_param)
		return -ENOMEM;

	set_param->apr_hdr.hdr_field =
		APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD, APR_HDR_LEN(APR_HDR_SIZE),
			      APR_PKT_VER);
	set_param->apr_hdr.pkt_size =
		APR_PKT_SIZE(APR_HDR_SIZE, pkt_size - APR_HDR_SIZE);
	set_param->apr_hdr.src_svc = 0;
	set_param->apr_hdr.src_domain = APR_DOMAIN_APPS;
	set_param->apr_hdr.src_port = voice_get_idx_for_session(v->session_id);
	set_param->apr_hdr.dest_svc = 0;
	set_param->apr_hdr.dest_domain = APR_DOMAIN_ADSP;
	set_param->apr_hdr.dest_port = voice_get_cvp_handle(v);
	set_param->apr_hdr.token = VOC_SET_MEDIA_FORMAT_PARAM_TOKEN;
	set_param->apr_hdr.opcode = q6common_is_instance_id_supported() ?
					    VSS_ICOMMON_CMD_SET_PARAM_V3 :
					    VSS_ICOMMON_CMD_SET_PARAM_V2;

	set_param->payload_size = param_size;

	if (mem_hdr != NULL) {
		set_param->mem_hdr = *mem_hdr;
	} else if (param_data != NULL) {
		memcpy(set_param->param_data, param_data, param_size);
	} else {
		pr_err("%s: Both memory header and param data are NULL\n",
		       __func__);
		ret = -EINVAL;
		goto done;
	}

	v->cvp_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvp, (u32 *) set_param);
	if (ret < 0) {
		pr_err("%s: Failed to send apr packet, error %d\n", __func__,
		       ret);
		goto done;
	}

	ret = wait_event_timeout(v->cvp_wait,
				 v->cvp_state == CMD_STATUS_SUCCESS,
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n", __func__,
		       adsp_err_get_err_str(v->async_err));
		ret = adsp_err_get_lnx_err_code(v->async_err);
		goto done;
	}
	ret = 0;

done:
	kfree(set_param);
	return ret;
}

static int voice_pack_and_set_cvp_param(struct voice_data *v,
					struct param_hdr_v3 param_hdr,
					u8 *param_data)
{
	u8 *packed_data = NULL;
	u32 total_size = 0;
	int ret = 0;

	total_size = sizeof(union param_hdrs) + param_hdr.param_size;
	packed_data = kzalloc(total_size, GFP_KERNEL);
	if (!packed_data)
		return -ENOMEM;

	ret = q6common_pack_pp_params(packed_data, &param_hdr, param_data,
				    &total_size);
	if (ret) {
		pr_err("%s: Failed to pack params, error %d", __func__, ret);
		goto done;
	}

	ret = voice_set_cvp_param(v, NULL, (u32 *) packed_data, total_size);

done:
	kfree(packed_data);
	return ret;
}

/*
 * Out of band is not supported and there are currently no pre-packed cases,
 * so pack and set in the same function. When needed, split up.
 */
static int voice_pack_and_set_cvs_ui_property(struct voice_data *v,
					      struct param_hdr_v3 param_hdr,
					      u8 *param_data)
{
	struct vss_icommon_cmd_set_ui_property *set_ui_property = NULL;
	u32 total_size = 0;
	u32 pkt_size = 0;
	u32 param_size = 0;
	bool iid_supported = q6common_is_instance_id_supported();
	void *apr_cvs;
	int ret = 0;

	apr_cvs = common.apr_q6_cvs;
	if (!apr_cvs) {
		pr_err("%s: apr_cvs is NULL\n", __func__);
		return -EINVAL;
	}

	pkt_size = sizeof(struct vss_icommon_cmd_set_ui_property);
	param_size = sizeof(union param_hdrs) + param_hdr.param_size;
	total_size = pkt_size + param_size;
	set_ui_property = kzalloc(total_size, GFP_KERNEL);
	if (!set_ui_property)
		return -ENOMEM;

	ret = q6common_pack_pp_params(set_ui_property->param_data, &param_hdr,
				    param_data, &param_size);
	if (ret) {
		pr_err("%s: Failed to pack params, error %d", __func__, ret);
		goto done;
	}

	/*
	 * Pack the APR header after packing the data so we have the actual
	 * total size of the payload
	 */
	total_size = pkt_size + param_size;
	set_ui_property->apr_hdr.hdr_field =
		APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD, APR_HDR_LEN(APR_HDR_SIZE),
			      APR_PKT_VER);
	set_ui_property->apr_hdr.pkt_size = total_size;
	set_ui_property->apr_hdr.src_svc = 0;
	set_ui_property->apr_hdr.src_domain = APR_DOMAIN_APPS;
	set_ui_property->apr_hdr.src_port =
		voice_get_idx_for_session(v->session_id);
	set_ui_property->apr_hdr.dest_svc = 0;
	set_ui_property->apr_hdr.dest_domain = APR_DOMAIN_ADSP;
	set_ui_property->apr_hdr.dest_port = voice_get_cvs_handle(v);
	set_ui_property->apr_hdr.token = 0;

	set_ui_property->apr_hdr.opcode =
		iid_supported ? VSS_ICOMMON_CMD_SET_UI_PROPERTY_V2 :
				VSS_ICOMMON_CMD_SET_UI_PROPERTY;

	v->cvs_state = CMD_STATUS_FAIL;
	v->async_err = 0;
	ret = apr_send_pkt(apr_cvs, (u32 *) set_ui_property);
	if (ret < 0) {
		pr_err("%s: Failed to send apr packet, error %d\n", __func__,
		       ret);
		goto done;
	}

	ret = wait_event_timeout(v->cvs_wait,
				 v->cvs_state == CMD_STATUS_SUCCESS,
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto done;
	}

	if (v->async_err > 0) {
		pr_err("%s: DSP returned error[%s]\n", __func__,
		       adsp_err_get_err_str(v->async_err));
		ret = adsp_err_get_lnx_err_code(v->async_err);
		goto done;
	}
	ret = 0;
done:
	kfree(set_ui_property);
	return ret;
}

static void voc_release_uevent_data(struct kobject *kobj)
{
	struct audio_uevent_data *data = container_of(kobj,
						      struct audio_uevent_data,
						      kobj);
	kfree(data);
}

/**
 * is_voc_initialized:
 *
 * Returns voice module init status
 *
 */
int is_voc_initialized(void)
{
	return module_initialized;
}
EXPORT_SYMBOL(is_voc_initialized);

int __init voice_init(void)
{
	int rc = 0, i = 0;

	memset(&common, 0, sizeof(struct common_data));

	/* set default value */
	common.default_mute_val = 0;  /* default is un-mute */
	common.default_sample_val = 8000;
	common.default_vol_step_val = 0;
	common.default_vol_ramp_duration_ms = DEFAULT_VOLUME_RAMP_DURATION;
	common.default_mute_ramp_duration_ms = DEFAULT_MUTE_RAMP_DURATION;
	common.cvp_version = 0;
	common.is_avcs_version_queried = false;
	/* Initialize EC Ref media format info */
	common.ec_ref_ext = false;
	common.ec_media_fmt_info.port_id = AFE_PORT_INVALID;
	common.ec_media_fmt_info.num_channels = 0;
	common.ec_media_fmt_info.bits_per_sample = 16;
	common.ec_media_fmt_info.sample_rate = 8000;
	memset(&common.ec_media_fmt_info.channel_mapping, 0,
	       VSS_CHANNEL_MAPPING_SIZE);

	/* Initialize AFE Sidetone Enable */
	common.sidetone_enable = false;

	/* Initialize MVS info. */
	common.mvs_info.network_type = VSS_NETWORK_ID_DEFAULT;

	/* Initialize is low memory flag */
	common.is_destroy_cvd = false;

	/* Initialize CVD version */
	strlcpy(common.cvd_version, CVD_VERSION_DEFAULT,
		sizeof(common.cvd_version));
	/* Initialize Per-Vocoder Calibration flag */
	common.is_per_vocoder_cal_enabled = false;

	/*
	 * Initialize in call record channel config
	 * to mono
	 */
	common.rec_channel_count = NUM_CHANNELS_MONO;

	mutex_init(&common.common_lock);

	common.uevent_data = kzalloc(sizeof(*(common.uevent_data)), GFP_KERNEL);
	if (!common.uevent_data)
		return -ENOMEM;

	/*
	 * Set release function to cleanup memory related to kobject
	 * before initializing the kobject.
	 */
	common.uevent_data->ktype.release = voc_release_uevent_data;
	q6core_init_uevent_data(common.uevent_data, "q6voice_uevent");
	common.mic_break_enable = false;

	/* Initialize session id with vsid */
	init_session_id();

	for (i = 0; i < MAX_VOC_SESSIONS; i++) {

		/* initialize dev_rx and dev_tx */
		common.voice[i].dev_rx.dev_mute =  common.default_mute_val;
		common.voice[i].dev_tx.dev_mute =  common.default_mute_val;
		common.voice[i].dev_rx.volume_step_value =
					common.default_vol_step_val;
		common.voice[i].dev_rx.volume_ramp_duration_ms =
					common.default_vol_ramp_duration_ms;
		common.voice[i].dev_rx.dev_mute_ramp_duration_ms =
					common.default_mute_ramp_duration_ms;
		common.voice[i].dev_tx.dev_mute_ramp_duration_ms =
					common.default_mute_ramp_duration_ms;
		common.voice[i].stream_rx.stream_mute = common.default_mute_val;
		common.voice[i].stream_tx.stream_mute = common.default_mute_val;

		common.voice[i].dev_tx.port_id = 0x100B;
		common.voice[i].dev_rx.port_id = 0x100A;
		common.voice[i].dev_tx.dev_id = 0;
		common.voice[i].dev_rx.dev_id = 0;
		common.voice[i].dev_tx.no_of_channels = 0;
		common.voice[i].dev_rx.no_of_channels = 0;
		common.voice[i].dev_tx.sample_rate = 8000;
		common.voice[i].dev_rx.sample_rate = 8000;
		common.voice[i].dev_tx.bits_per_sample = 16;
		common.voice[i].dev_rx.bits_per_sample = 16;
		memset(&common.voice[i].dev_tx.channel_mapping, 0,
		       VSS_CHANNEL_MAPPING_SIZE);
		memset(&common.voice[i].dev_rx.channel_mapping, 0,
		       VSS_CHANNEL_MAPPING_SIZE);
		common.voice[i].sidetone_gain = 0x512;
		common.voice[i].dtmf_rx_detect_en = 0;
		common.voice[i].lch_mode = 0;
		common.voice[i].disable_topology = false;

		common.voice[i].voc_state = VOC_INIT;

		INIT_WORK(&common.voice[i].voice_mic_break_work,
				voice_mic_break_work_fn);

		init_waitqueue_head(&common.voice[i].mvm_wait);
		init_waitqueue_head(&common.voice[i].cvs_wait);
		init_waitqueue_head(&common.voice[i].cvp_wait);

		mutex_init(&common.voice[i].lock);
	}

	if (voice_init_cal_data())
		pr_err("%s: Could not init cal data!\n", __func__);

	if (rc == 0) {
		module_initialized = true;
		hyp_assigned = false;
	}

	pr_debug("%s: rc=%d\n", __func__, rc);
	return rc;
}


void voice_exit(void)
{
	int i;
	q6core_destroy_uevent_data(common.uevent_data);
	voice_delete_cal_data();
	free_cal_map_table();
	mutex_destroy(&common.common_lock);
	for (i = 0; i < MAX_VOC_SESSIONS; i++)
		mutex_destroy(&common.voice[i].lock);
}
