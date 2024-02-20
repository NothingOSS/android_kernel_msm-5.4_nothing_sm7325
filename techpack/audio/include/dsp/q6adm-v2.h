/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __Q6_ADM_V2_H__
#define __Q6_ADM_V2_H__


#define ADM_PATH_PLAYBACK 0x1
#define ADM_PATH_LIVE_REC 0x2
#define ADM_PATH_NONLIVE_REC 0x3
#define ADM_PATH_COMPRESSED_RX 0x5
#define ADM_PATH_COMPRESSED_TX 0x6
#include <dsp/rtac.h>
#include <dsp/q6afe-v2.h>
#include <dsp/q6audio-v2.h>

#define MAX_MODULES_IN_TOPO 16
#define MAX_FE_ID           33
#define ADM_GET_TOPO_MODULE_LIST_LENGTH\
		((MAX_MODULES_IN_TOPO + 1) * sizeof(uint32_t))
#define ADM_GET_TOPO_MODULE_INSTANCE_LIST_LENGTH                               \
	((MAX_MODULES_IN_TOPO + 1) * 2 * sizeof(uint32_t))
#define AUD_PROC_BLOCK_SIZE	4096
#define AUD_VOL_BLOCK_SIZE	4096
#define AUD_PROC_PERSIST_BLOCK_SIZE	(2 * 1024 * 1020)
#define AUDIO_RX_CALIBRATION_SIZE	(AUD_PROC_BLOCK_SIZE + \
						AUD_VOL_BLOCK_SIZE)
enum {
	ADM_CUSTOM_TOP_CAL = 0,
	ADM_AUDPROC_CAL,
	ADM_LSM_AUDPROC_CAL,
	ADM_AUDVOL_CAL,
	ADM_RTAC_INFO_CAL,
	ADM_RTAC_APR_CAL,
	ADM_SRS_TRUMEDIA,
	ADM_RTAC_AUDVOL_CAL,
	ADM_LSM_AUDPROC_PERSISTENT_CAL,
	ADM_AUDPROC_PERSISTENT_CAL,
	ADM_MAX_CAL_TYPES
};

enum {
	ADM_MEM_MAP_INDEX_SOURCE_TRACKING = ADM_MAX_CAL_TYPES,
	ADM_MEM_MAP_INDEX_MAX
};

enum {
	ADM_CLIENT_ID_DEFAULT = 0,
	ADM_CLIENT_ID_SOURCE_TRACKING,
	ADM_CLIENT_ID_MAX,
};

#define MAX_COPPS_PER_PORT 0x8
#define ADM_MAX_CHANNELS 32

#define ADSP_ADM_API_VERSION_V3 3

struct msm_ec_ref_port_cfg {
	int rx;
	int port_id;
	int ch;
	int bit_width;
	int sampling_rate;
};

/* multiple copp per stream. */
struct route_payload {
	unsigned int copp_idx[MAX_COPPS_PER_PORT];
	unsigned int port_id[MAX_COPPS_PER_PORT];
	int app_type[MAX_COPPS_PER_PORT];
	int acdb_dev_id[MAX_COPPS_PER_PORT];
	int sample_rate[MAX_COPPS_PER_PORT];
	unsigned short num_copps;
	unsigned int session_id;
};

struct default_chmixer_param_id_coeff {
	uint32_t index;
	uint16_t num_output_channels;
	uint16_t num_input_channels;
};

struct msm_pcm_channel_mixer {
	int output_channel;
	int input_channels[ADM_MAX_CHANNELS];
	bool enable;
	int rule;
	int channel_weight[ADM_MAX_CHANNELS][ADM_MAX_CHANNELS];
	int port_idx;
	int input_channel;
	uint16_t in_ch_map[ADM_MAX_CHANNELS];
	uint16_t out_ch_map[ADM_MAX_CHANNELS];
	bool override_in_ch_map;
	bool override_out_ch_map;
};

struct ffv_spf_freeze_param_t {
	uint16_t freeze;
	uint16_t source_id;
};

int srs_trumedia_open(int port_id, int copp_idx, __s32 srs_tech_id,
		      void *srs_params);

int adm_dts_eagle_set(int port_id, int copp_idx, int param_id,
		      void *data, uint32_t size);

int adm_dts_eagle_get(int port_id, int copp_idx, int param_id,
		      void *data, uint32_t size);

void adm_copp_mfc_cfg(int port_id, int copp_idx, int dst_sample_rate);

int adm_get_pp_params(int port_id, int copp_idx, uint32_t client_id,
		      struct mem_mapping_hdr *mem_hdr,
		      struct param_hdr_v3 *param_hdr, u8 *returned_param_data);

int adm_send_params_v5(int port_id, int copp_idx, char *params,
			      uint32_t params_length);

int adm_set_pp_params(int port_id, int copp_idx,
		      struct mem_mapping_hdr *mem_hdr, u8 *param_data,
		      u32 params_size);

int adm_pack_and_set_one_pp_param(int port_id, int copp_idx,
				  struct param_hdr_v3 param_hdr,
				  u8 *param_data);

int adm_open(int port, int path, int rate, int mode, int topology,
			   int perf_mode, uint16_t bits_per_sample,
			   int app_type, int acdbdev_id, int session_type,
			   uint32_t pass_thr, uint32_t copp_token);

int adm_open_v2(int port, int path, int rate, int mode, int topology,
			   int perf_mode, uint16_t bits_per_sample,
			   int app_type, int acdbdev_id, int session_type,
			   uint32_t pass_thr, uint32_t copp_token,
			   struct msm_ec_ref_port_cfg *ec_ref_port_cfg,
			   struct msm_pcm_channel_mixer *ec_ref_chmix_cfg);

int adm_map_rtac_block(struct rtac_cal_block_data *cal_block);

int adm_unmap_rtac_block(uint32_t *mem_map_handle);

int adm_close(int port, int topology, int perf_mode);

int adm_matrix_map(int fedai_id, int path, struct route_payload payload_map,
		   int perf_mode, uint32_t passthr_mode);

int adm_connect_afe_port(int mode, int session_id, int port_id);

void adm_ec_ref_rx_id(int  port_id);

void adm_num_ec_ref_rx_chans(int num_chans);

void adm_num_ec_ref_rx_chans_downmixed(int num_chans);

int adm_ec_ref_chmixer_weights(int out_channel_idx,
			uint16_t *weights, int count);

void adm_ec_ref_rx_bit_width(int bit_width);

void adm_ec_ref_rx_sampling_rate(int sampling_rate);

int adm_get_lowlatency_copp_id(int port_id);

int adm_set_multi_ch_map(char *channel_map, int path);

int adm_get_multi_ch_map(char *channel_map, int path);

void adm_set_port_multi_ch_map(char *channel_map, int port_id);

int adm_validate_and_get_port_index(int port_id);

int adm_get_default_copp_idx(int port_id);

int adm_get_topology_for_port_from_copp_id(int port_id, int copp_id);

int adm_get_topology_for_port_copp_idx(int port_id, int copp_idx);

int adm_get_indexes_from_copp_id(int copp_id, int *port_idx, int *copp_idx);

int adm_set_stereo_to_custom_stereo(int port_id, int copp_idx,
				    unsigned int session_id,
				    char *params, uint32_t params_length);

int adm_get_pp_topo_module_list(int port_id, int copp_idx, int32_t param_length,
				char *params);

int adm_get_pp_topo_module_list_v2(int port_id, int copp_idx,
				   int32_t param_length,
				   int32_t *returned_params);

int adm_set_volume(int port_id, int copp_idx, int volume);

int adm_set_softvolume(int port_id, int copp_idx,
		       struct audproc_softvolume_params *softvol_param);

int adm_set_mic_gain(int port_id, int copp_idx, int volume);

int adm_send_set_multichannel_ec_primary_mic_ch(int port_id, int copp_idx,
				int primary_mic_ch);

int adm_set_ffecns_effect(int effect);

int adm_param_enable(int port_id, int copp_idx, int module_id,  int enable);

int adm_param_enable_v2(int port_id, int copp_idx,
			struct module_instance_info mod_inst_info, int enable);

int adm_send_calibration(int port_id, int copp_idx, int path, int perf_mode,
			 int cal_type, char *params, int size);

int adm_set_wait_parameters(int port_id, int copp_idx);

int adm_reset_wait_parameters(int port_id, int copp_idx);

int adm_wait_timeout(int port_id, int copp_idx, int wait_time);

int adm_store_cal_data(int port_id, int copp_idx, int path, int perf_mode,
		       int cal_type, char *params, int *size);

int adm_send_compressed_device_mute(int port_id, int copp_idx, bool mute_on);

int adm_send_compressed_device_latency(int port_id, int copp_idx, int latency);
int adm_set_sound_focus(int port_id, int copp_idx,
			struct sound_focus_param soundFocusData);
int adm_get_sound_focus(int port_id, int copp_idx,
			struct sound_focus_param *soundFocusData);
int adm_get_source_tracking(int port_id, int copp_idx,
			    struct source_tracking_param *sourceTrackingData);
int adm_get_fnn_source_tracking(int port_id, int copp_idx,
			    struct fluence_nn_source_tracking_param *FnnSourceTrackingData);
int adm_get_doa_tracking_mon(int port_id, int copp_idx,
			    struct doa_tracking_mon_param *doa_tracking_data);
int adm_set_custom_chmix_cfg(int port_id, int copp_idx,
			     unsigned int session_id, char *params,
			     uint32_t params_length, int direction,
				 int stream_type);
int adm_swap_speaker_channels(int port_id, int copp_idx, int sample_rate,
				bool spk_swap);
int adm_programable_channel_mixer(int port_id, int copp_idx, int session_id,
			int session_type,
			struct msm_pcm_channel_mixer *ch_mixer,
			int channel_index);
void msm_dts_srs_acquire_lock(void);
void msm_dts_srs_release_lock(void);
void adm_set_native_mode(int mode);
int adm_set_ffecns_freeze_event(bool ffecns_freeze_event);
int adm_apr_send_pkt(void *data, wait_queue_head_t *wait,
			int port_idx, int copp_idx, int opcode);
void q6adm_register_callback(void *cb);
void q6adm_clear_callback(void);
int q6adm_send_event_register_cmd(int port_id, int copp_idx, u8 *data,
					int param_size, int opcode);
int q6adm_update_rtd_info(void *rtd, int port_id,
			int copp_idx, int fe_id, int enable);
#endif /* __Q6_ADM_V2_H__ */
