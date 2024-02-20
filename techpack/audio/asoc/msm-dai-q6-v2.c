// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/q6afe-v2.h>
#include <dsp/sp_params.h>
#include <dsp/q6core.h>
#include "msm-dai-q6-v2.h"
#include <asoc/core.h>

#define MSM_DAI_PRI_AUXPCM_DT_DEV_ID 1
#define MSM_DAI_SEC_AUXPCM_DT_DEV_ID 2
#define MSM_DAI_TERT_AUXPCM_DT_DEV_ID 3
#define MSM_DAI_QUAT_AUXPCM_DT_DEV_ID 4
#define MSM_DAI_QUIN_AUXPCM_DT_DEV_ID 5
#define MSM_DAI_SEN_AUXPCM_DT_DEV_ID 6

#define MSM_DAI_TWS_CHANNEL_MODE_ONE 1
#define MSM_DAI_TWS_CHANNEL_MODE_TWO 2

#define MSM_DAI_LC3_CHANNEL_MODE_ONE 1
#define MSM_DAI_LC3_CHANNEL_MODE_TWO 2

#define spdif_clock_value(rate) (2*rate*32*2)
#define CHANNEL_STATUS_SIZE 24
#define CHANNEL_STATUS_MASK_INIT 0x0
#define CHANNEL_STATUS_MASK 0x4
#define PREEMPH_MASK 0x38
#define PREEMPH_SHIFT 3
#define GET_PREEMPH(b) ((b & PREEMPH_MASK) >> PREEMPH_SHIFT)
#define AFE_API_VERSION_CLOCK_SET 1
#define MSM_DAI_SYSFS_ENTRY_MAX_LEN 64

#define DAI_FORMATS_S16_S24_S32_LE (SNDRV_PCM_FMTBIT_S16_LE | \
				    SNDRV_PCM_FMTBIT_S24_LE | \
				    SNDRV_PCM_FMTBIT_S32_LE)

static int msm_mi2s_get_port_id(u32 mi2s_id, int stream, u16 *port_id);

enum {
	ENC_FMT_NONE,
	DEC_FMT_NONE = ENC_FMT_NONE,
	ENC_FMT_SBC = ASM_MEDIA_FMT_SBC,
	DEC_FMT_SBC = ASM_MEDIA_FMT_SBC,
	ENC_FMT_AAC_V2 = ASM_MEDIA_FMT_AAC_V2,
	DEC_FMT_AAC_V2 = ASM_MEDIA_FMT_AAC_V2,
	ENC_FMT_APTX = ASM_MEDIA_FMT_APTX,
	ENC_FMT_APTX_HD = ASM_MEDIA_FMT_APTX_HD,
	ENC_FMT_CELT = ASM_MEDIA_FMT_CELT,
	ENC_FMT_LDAC = ASM_MEDIA_FMT_LDAC,
	ENC_FMT_APTX_ADAPTIVE = ASM_MEDIA_FMT_APTX_ADAPTIVE,
	DEC_FMT_APTX_ADAPTIVE = ASM_MEDIA_FMT_APTX_ADAPTIVE,
	DEC_FMT_MP3 = ASM_MEDIA_FMT_MP3,
	ENC_FMT_APTX_AD_SPEECH = ASM_MEDIA_FMT_APTX_AD_SPEECH,
	DEC_FMT_APTX_AD_SPEECH = ASM_MEDIA_FMT_APTX_AD_SPEECH,
	ENC_FMT_LC3 = ASM_MEDIA_FMT_LC3,
};

enum {
	SPKR_1,
	SPKR_2,
};

static const struct afe_clk_set lpass_clk_set_default = {
	AFE_API_VERSION_CLOCK_SET,
	Q6AFE_LPASS_CLK_ID_PRI_PCM_IBIT,
	Q6AFE_LPASS_OSR_CLK_2_P048_MHZ,
	Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	0,
};

static const struct afe_clk_cfg lpass_clk_cfg_default = {
	AFE_API_VERSION_I2S_CONFIG,
	Q6AFE_LPASS_OSR_CLK_2_P048_MHZ,
	0,
	Q6AFE_LPASS_CLK_SRC_INTERNAL,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	Q6AFE_LPASS_MODE_CLK1_VALID,
	0,
};
enum {
	STATUS_PORT_STARTED, /* track if AFE port has started */
	/* track AFE Tx port status for bi-directional transfers */
	STATUS_TX_PORT,
	/* track AFE Rx port status for bi-directional transfers */
	STATUS_RX_PORT,
	STATUS_MAX
};

enum {
	RATE_8KHZ,
	RATE_16KHZ,
	RATE_MAX_NUM_OF_AUX_PCM_RATES,
};

enum {
	IDX_PRIMARY_TDM_RX_0,
	IDX_PRIMARY_TDM_RX_1,
	IDX_PRIMARY_TDM_RX_2,
	IDX_PRIMARY_TDM_RX_3,
	IDX_PRIMARY_TDM_RX_4,
	IDX_PRIMARY_TDM_RX_5,
	IDX_PRIMARY_TDM_RX_6,
	IDX_PRIMARY_TDM_RX_7,
	IDX_PRIMARY_TDM_TX_0,
	IDX_PRIMARY_TDM_TX_1,
	IDX_PRIMARY_TDM_TX_2,
	IDX_PRIMARY_TDM_TX_3,
	IDX_PRIMARY_TDM_TX_4,
	IDX_PRIMARY_TDM_TX_5,
	IDX_PRIMARY_TDM_TX_6,
	IDX_PRIMARY_TDM_TX_7,
	IDX_SECONDARY_TDM_RX_0,
	IDX_SECONDARY_TDM_RX_1,
	IDX_SECONDARY_TDM_RX_2,
	IDX_SECONDARY_TDM_RX_3,
	IDX_SECONDARY_TDM_RX_4,
	IDX_SECONDARY_TDM_RX_5,
	IDX_SECONDARY_TDM_RX_6,
	IDX_SECONDARY_TDM_RX_7,
	IDX_SECONDARY_TDM_TX_0,
	IDX_SECONDARY_TDM_TX_1,
	IDX_SECONDARY_TDM_TX_2,
	IDX_SECONDARY_TDM_TX_3,
	IDX_SECONDARY_TDM_TX_4,
	IDX_SECONDARY_TDM_TX_5,
	IDX_SECONDARY_TDM_TX_6,
	IDX_SECONDARY_TDM_TX_7,
	IDX_TERTIARY_TDM_RX_0,
	IDX_TERTIARY_TDM_RX_1,
	IDX_TERTIARY_TDM_RX_2,
	IDX_TERTIARY_TDM_RX_3,
	IDX_TERTIARY_TDM_RX_4,
	IDX_TERTIARY_TDM_RX_5,
	IDX_TERTIARY_TDM_RX_6,
	IDX_TERTIARY_TDM_RX_7,
	IDX_TERTIARY_TDM_TX_0,
	IDX_TERTIARY_TDM_TX_1,
	IDX_TERTIARY_TDM_TX_2,
	IDX_TERTIARY_TDM_TX_3,
	IDX_TERTIARY_TDM_TX_4,
	IDX_TERTIARY_TDM_TX_5,
	IDX_TERTIARY_TDM_TX_6,
	IDX_TERTIARY_TDM_TX_7,
	IDX_QUATERNARY_TDM_RX_0,
	IDX_QUATERNARY_TDM_RX_1,
	IDX_QUATERNARY_TDM_RX_2,
	IDX_QUATERNARY_TDM_RX_3,
	IDX_QUATERNARY_TDM_RX_4,
	IDX_QUATERNARY_TDM_RX_5,
	IDX_QUATERNARY_TDM_RX_6,
	IDX_QUATERNARY_TDM_RX_7,
	IDX_QUATERNARY_TDM_TX_0,
	IDX_QUATERNARY_TDM_TX_1,
	IDX_QUATERNARY_TDM_TX_2,
	IDX_QUATERNARY_TDM_TX_3,
	IDX_QUATERNARY_TDM_TX_4,
	IDX_QUATERNARY_TDM_TX_5,
	IDX_QUATERNARY_TDM_TX_6,
	IDX_QUATERNARY_TDM_TX_7,
	IDX_QUINARY_TDM_RX_0,
	IDX_QUINARY_TDM_RX_1,
	IDX_QUINARY_TDM_RX_2,
	IDX_QUINARY_TDM_RX_3,
	IDX_QUINARY_TDM_RX_4,
	IDX_QUINARY_TDM_RX_5,
	IDX_QUINARY_TDM_RX_6,
	IDX_QUINARY_TDM_RX_7,
	IDX_QUINARY_TDM_TX_0,
	IDX_QUINARY_TDM_TX_1,
	IDX_QUINARY_TDM_TX_2,
	IDX_QUINARY_TDM_TX_3,
	IDX_QUINARY_TDM_TX_4,
	IDX_QUINARY_TDM_TX_5,
	IDX_QUINARY_TDM_TX_6,
	IDX_QUINARY_TDM_TX_7,
	IDX_SENARY_TDM_RX_0,
	IDX_SENARY_TDM_RX_1,
	IDX_SENARY_TDM_RX_2,
	IDX_SENARY_TDM_RX_3,
	IDX_SENARY_TDM_RX_4,
	IDX_SENARY_TDM_RX_5,
	IDX_SENARY_TDM_RX_6,
	IDX_SENARY_TDM_RX_7,
	IDX_SENARY_TDM_TX_0,
	IDX_SENARY_TDM_TX_1,
	IDX_SENARY_TDM_TX_2,
	IDX_SENARY_TDM_TX_3,
	IDX_SENARY_TDM_TX_4,
	IDX_SENARY_TDM_TX_5,
	IDX_SENARY_TDM_TX_6,
	IDX_SENARY_TDM_TX_7,
	IDX_SEPTENARY_TDM_RX_0,
	IDX_SEPTENARY_TDM_RX_1,
	IDX_SEPTENARY_TDM_RX_2,
	IDX_SEPTENARY_TDM_RX_3,
	IDX_SEPTENARY_TDM_RX_4,
	IDX_SEPTENARY_TDM_RX_5,
	IDX_SEPTENARY_TDM_RX_6,
	IDX_SEPTENARY_TDM_RX_7,
	IDX_SEPTENARY_TDM_TX_0,
	IDX_SEPTENARY_TDM_TX_1,
	IDX_SEPTENARY_TDM_TX_2,
	IDX_SEPTENARY_TDM_TX_3,
	IDX_SEPTENARY_TDM_TX_4,
	IDX_SEPTENARY_TDM_TX_5,
	IDX_SEPTENARY_TDM_TX_6,
	IDX_SEPTENARY_TDM_TX_7,

	IDX_HSIF0_TDM_RX_0,
	IDX_HSIF0_TDM_RX_1,
	IDX_HSIF0_TDM_RX_2,
	IDX_HSIF0_TDM_RX_3,
	IDX_HSIF0_TDM_RX_4,
	IDX_HSIF0_TDM_RX_5,
	IDX_HSIF0_TDM_RX_6,
	IDX_HSIF0_TDM_RX_7,
	IDX_HSIF0_TDM_TX_0,
	IDX_HSIF0_TDM_TX_1,
	IDX_HSIF0_TDM_TX_2,
	IDX_HSIF0_TDM_TX_3,
	IDX_HSIF0_TDM_TX_4,
	IDX_HSIF0_TDM_TX_5,
	IDX_HSIF0_TDM_TX_6,
	IDX_HSIF0_TDM_TX_7,

	IDX_HSIF1_TDM_RX_0,
	IDX_HSIF1_TDM_RX_1,
	IDX_HSIF1_TDM_RX_2,
	IDX_HSIF1_TDM_RX_3,
	IDX_HSIF1_TDM_RX_4,
	IDX_HSIF1_TDM_RX_5,
	IDX_HSIF1_TDM_RX_6,
	IDX_HSIF1_TDM_RX_7,
	IDX_HSIF1_TDM_TX_0,
	IDX_HSIF1_TDM_TX_1,
	IDX_HSIF1_TDM_TX_2,
	IDX_HSIF1_TDM_TX_3,
	IDX_HSIF1_TDM_TX_4,
	IDX_HSIF1_TDM_TX_5,
	IDX_HSIF1_TDM_TX_6,
	IDX_HSIF1_TDM_TX_7,

	IDX_HSIF2_TDM_RX_0,
	IDX_HSIF2_TDM_RX_1,
	IDX_HSIF2_TDM_RX_2,
	IDX_HSIF2_TDM_RX_3,
	IDX_HSIF2_TDM_RX_4,
	IDX_HSIF2_TDM_RX_5,
	IDX_HSIF2_TDM_RX_6,
	IDX_HSIF2_TDM_RX_7,
	IDX_HSIF2_TDM_TX_0,
	IDX_HSIF2_TDM_TX_1,
	IDX_HSIF2_TDM_TX_2,
	IDX_HSIF2_TDM_TX_3,
	IDX_HSIF2_TDM_TX_4,
	IDX_HSIF2_TDM_TX_5,
	IDX_HSIF2_TDM_TX_6,
	IDX_HSIF2_TDM_TX_7,

	IDX_TDM_MAX,
};

enum {
	IDX_GROUP_PRIMARY_TDM_RX,
	IDX_GROUP_PRIMARY_TDM_TX,
	IDX_GROUP_SECONDARY_TDM_RX,
	IDX_GROUP_SECONDARY_TDM_TX,
	IDX_GROUP_TERTIARY_TDM_RX,
	IDX_GROUP_TERTIARY_TDM_TX,
	IDX_GROUP_QUATERNARY_TDM_RX,
	IDX_GROUP_QUATERNARY_TDM_TX,
	IDX_GROUP_QUINARY_TDM_RX,
	IDX_GROUP_QUINARY_TDM_TX,
	IDX_GROUP_SENARY_TDM_RX,
	IDX_GROUP_SENARY_TDM_TX,
	IDX_GROUP_SEPTENARY_TDM_RX,
	IDX_GROUP_SEPTENARY_TDM_TX,
	IDX_GROUP_HSIF0_TDM_RX,
	IDX_GROUP_HSIF0_TDM_TX,
	IDX_GROUP_HSIF1_TDM_RX,
	IDX_GROUP_HSIF1_TDM_TX,
	IDX_GROUP_HSIF2_TDM_RX,
	IDX_GROUP_HSIF2_TDM_TX,
	IDX_GROUP_TDM_MAX,
};

struct msm_dai_q6_dai_data {
	DECLARE_BITMAP(status_mask, STATUS_MAX);
	DECLARE_BITMAP(hwfree_status, STATUS_MAX);
	u32 rate;
	u32 channels;
	u32 bitwidth;
	u32 cal_mode;
	u32 afe_rx_in_channels;
	u16 afe_rx_in_bitformat;
	u32 afe_tx_out_channels;
	u16 afe_tx_out_bitformat;
	struct afe_enc_config enc_config;
	struct afe_dec_config dec_config;
	struct afe_ttp_config ttp_config;
	union afe_port_config port_config;
	u16 vi_feed_mono;
	u32 xt_logging_disable;
};

struct msm_dai_q6_spdif_dai_data {
	DECLARE_BITMAP(status_mask, STATUS_MAX);
	u32 rate;
	u32 channels;
	u32 bitwidth;
	u16 port_id;
	struct afe_spdif_port_config spdif_port;
	struct afe_event_fmt_update fmt_event;
	struct kobject *kobj;
};

struct msm_dai_q6_spdif_event_msg {
	struct afe_port_mod_evt_rsp_hdr  evt_hdr;
	struct afe_event_fmt_update      fmt_event;
};

struct msm_dai_q6_mi2s_dai_config {
	u16 pdata_mi2s_lines;
	struct msm_dai_q6_dai_data mi2s_dai_data;
};

struct msm_dai_q6_mi2s_dai_data {
	u32 is_island_dai;
	struct msm_dai_q6_mi2s_dai_config mi2s_dai;
};

struct msm_dai_q6_meta_mi2s_dai_data {
	DECLARE_BITMAP(status_mask, STATUS_MAX);
	u16 num_member_ports;
	u16 member_port_id[MAX_NUM_I2S_META_PORT_MEMBER_PORTS];
	u16 channel_mode[MAX_NUM_I2S_META_PORT_MEMBER_PORTS];
	u32 rate;
	u32 channels;
	u32 bitwidth;
	union afe_port_config port_config;
};

struct msm_dai_q6_cdc_dma_dai_data {
	DECLARE_BITMAP(status_mask, STATUS_MAX);
	DECLARE_BITMAP(hwfree_status, STATUS_MAX);
	u32 rate;
	u32 channels;
	u32 bitwidth;
	u32 is_island_dai;
	u32 xt_logging_disable;
	union afe_port_config port_config;
	u32 cdc_dma_data_align;
};

struct msm_dai_q6_auxpcm_dai_data {
	/* BITMAP to track Rx and Tx port usage count */
	DECLARE_BITMAP(auxpcm_port_status, STATUS_MAX);
	struct mutex rlock; /* auxpcm dev resource lock */
	u16 rx_pid; /* AUXPCM RX AFE port ID */
	u16 tx_pid; /* AUXPCM TX AFE port ID */
	u16 afe_clk_ver;
	u32 is_island_dai;
	struct afe_clk_cfg clk_cfg; /* hold LPASS clock configuration */
	struct afe_clk_set clk_set; /* hold LPASS clock configuration */
	struct msm_dai_q6_dai_data bdai_data; /* incoporate base DAI data */
};

struct msm_dai_q6_tdm_dai_data {
	DECLARE_BITMAP(status_mask, STATUS_MAX);
	u32 rate;
	u32 channels;
	u32 bitwidth;
	u32 num_group_ports;
	u32 is_island_dai;
	struct afe_clk_set clk_set; /* hold LPASS clock config. */
	union afe_port_group_config group_cfg; /* hold tdm group config */
	struct afe_tdm_port_config port_cfg; /* hold tdm config */
	struct afe_param_id_tdm_lane_cfg lane_cfg; /* hold tdm lane config */
};

/* MI2S format field for AFE_PORT_CMD_I2S_CONFIG command
 *  0: linear PCM
 *  1: non-linear PCM
 *  2: PCM data in IEC 60968 container
 *  3: compressed data in IEC 60958 container
 *  9: DSD over PCM (DoP) with marker byte
 */
static const char *const mi2s_format[] = {
	"LPCM",
	"Compr",
	"LPCM-60958",
	"Compr-60958",
	"NA4",
	"NA5",
	"NA6",
	"NA7",
	"NA8",
	"DSD_DOP_W_MARKER",
	"NATIVE_DSD_DATA"
};

static const char *const mi2s_vi_feed_mono[] = {
	"Left",
	"Right",
};

static const struct soc_enum mi2s_config_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mi2s_format), mi2s_format),
	SOC_ENUM_SINGLE_EXT(2, mi2s_vi_feed_mono),
};

static const char *const cdc_dma_format[] = {
	"UNPACKED",
	"UNPACKED_NON_LINEAR",
	"PACKED_LINEAR_60958",
	"PACKED_NON_LINEAR_60958",
	"NULL",
	"NULL",
        "PACKED_16B",
};

static const struct soc_enum cdc_dma_config_enum[] = {
	SOC_ENUM_SINGLE_EXT(7, cdc_dma_format),
};

static const char *const sb_format[] = {
	"UNPACKED",
	"PACKED_16B",
	"DSD_DOP",
};

static const struct soc_enum sb_config_enum[] = {
	SOC_ENUM_SINGLE_EXT(3, sb_format),
};

static const char * const xt_logging_disable_text[] = {
	"FALSE",
	"TRUE",
};

static const struct soc_enum xt_logging_disable_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, xt_logging_disable_text),
};

static const char *const tdm_data_format[] = {
	"LPCM",
	"Compr",
	"Gen Compr"
};

static const char *const tdm_header_type[] = {
	"Invalid",
	"Default",
	"Entertainment",
};

static const struct soc_enum tdm_config_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tdm_data_format), tdm_data_format),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tdm_header_type), tdm_header_type),
};

static u16 afe_port_logging_port_id = 0x9000;

static bool afe_port_logging_item[IDX_TDM_MAX];

static int afe_port_loggging_control_added;

static int afe_port_limiter_control_added;

static int afe_dyn_mclk_control_added;

static DEFINE_MUTEX(tdm_mutex);

static atomic_t tdm_group_ref[IDX_GROUP_TDM_MAX];

static struct afe_param_id_tdm_lane_cfg tdm_lane_cfg = {
	AFE_GROUP_DEVICE_ID_QUINARY_TDM_RX,
	0x0,
};

#define LIMITER_PARM_MAX 3
static u16 limiter_param[LIMITER_PARM_MAX];

/* cache of group cfg per parent node */
static struct afe_param_id_group_device_tdm_cfg tdm_group_cfg = {
	AFE_API_VERSION_GROUP_DEVICE_TDM_CONFIG,
	AFE_GROUP_DEVICE_ID_QUATERNARY_TDM_RX,
	0,
	{AFE_PORT_ID_QUATERNARY_TDM_RX,
	AFE_PORT_ID_QUATERNARY_TDM_RX_1,
	AFE_PORT_ID_QUATERNARY_TDM_RX_2,
	AFE_PORT_ID_QUATERNARY_TDM_RX_3,
	AFE_PORT_ID_QUATERNARY_TDM_RX_4,
	AFE_PORT_ID_QUATERNARY_TDM_RX_5,
	AFE_PORT_ID_QUATERNARY_TDM_RX_6,
	AFE_PORT_ID_QUATERNARY_TDM_RX_7},
	8,
	48000,
	32,
	8,
	32,
	0xFF,
};

static u32 num_tdm_group_ports;

static struct afe_clk_set tdm_clk_set = {
	AFE_API_VERSION_CLOCK_SET,
	Q6AFE_LPASS_CLK_ID_QUAD_TDM_EBIT,
	Q6AFE_LPASS_IBIT_CLK_DISABLE,
	Q6AFE_LPASS_CLK_ATTRIBUTE_INVERT_COUPLE_NO,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	0,
};

static int clk_id_index;
static int clk_root_index;
static int clk_attri_index;
static int global_dyn_mclk_cfg_portid;
static bool jitter_cleaner_enable = false; // jitter cleaner ext clock enable/disable
struct afe_param_id_clock_set_v2_t global_dyn_mclk_cfg = {
	.clk_set_minor_version = Q6AFE_LPASS_CLK_CONFIG_API_VERSION,
	.clk_id = Q6AFE_LPASS_CLK_ID_TER_PCM_IBIT,
	.clk_freq_in_hz = 12288000,
	.clk_attri = Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
	.clk_root = Q6AFE_LPASS_MCLK_IN0,
	.divider_2x = AFE_CLOCK_DEFAULT_INTEGER_DIVIDER,
	.m = AFE_CLOCK_DEFAULT_M_VALUE,
	.n = AFE_CLOCK_DEFAULT_N_VALUE,
	.d = AFE_CLOCK_DEFAULT_D_VALUE,
	.enable = 0,
};

static int afe_dyn_clk_root_enum[] = {
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	Q6AFE_LPASS_MCLK_IN0,
	Q6AFE_LPASS_MCLK_IN1,
};

static int afe_dyn_clk_attri_enum[] = {
	Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_NO,
	Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_DIVIDEND,
	Q6AFE_LPASS_CLK_ATTRIBUTE_COUPLE_DIVISOR,
	Q6AFE_LPASS_CLK_ATTRIBUTE_INVERT_COUPLE_NO,
};

static int afe_dyn_clk_id_enum[] = {
	Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT, Q6AFE_LPASS_CLK_ID_PRI_MI2S_EBIT,
	Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT, Q6AFE_LPASS_CLK_ID_SEC_MI2S_EBIT,
	Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT, Q6AFE_LPASS_CLK_ID_TER_MI2S_EBIT,
	Q6AFE_LPASS_CLK_ID_QUAD_MI2S_IBIT, Q6AFE_LPASS_CLK_ID_QUAD_MI2S_EBIT,
	Q6AFE_LPASS_CLK_ID_SPEAKER_I2S_IBIT, Q6AFE_LPASS_CLK_ID_SPEAKER_I2S_EBIT,
	Q6AFE_LPASS_CLK_ID_SPEAKER_I2S_OSR,
	Q6AFE_LPASS_CLK_ID_PRI_PCM_IBIT, Q6AFE_LPASS_CLK_ID_PRI_PCM_EBIT,
	Q6AFE_LPASS_CLK_ID_SEC_PCM_IBIT, Q6AFE_LPASS_CLK_ID_SEC_PCM_EBIT,
	Q6AFE_LPASS_CLK_ID_TER_PCM_IBIT, Q6AFE_LPASS_CLK_ID_TER_PCM_EBIT,
	Q6AFE_LPASS_CLK_ID_QUAD_PCM_IBIT, Q6AFE_LPASS_CLK_ID_QUAD_PCM_EBIT,
	Q6AFE_LPASS_CLK_ID_MCLK_1, Q6AFE_LPASS_CLK_ID_MCLK_2,
	Q6AFE_LPASS_CLK_ID_MCLK_3, Q6AFE_LPASS_CLK_ID_MCLK_4,
	Q6AFE_LPASS_CLK_ID_MCLK_5, Q6AFE_LPASS_CLK_ID_AHB_HDMI_INPUT,
	Q6AFE_LPASS_CLK_ID_SPDIF_CORE
};

static int msm_dai_q6_get_tdm_clk_ref(u16 id)
{
	switch (id) {
	case IDX_GROUP_PRIMARY_TDM_RX:
	case IDX_GROUP_PRIMARY_TDM_TX:
		return atomic_read(&tdm_group_ref[IDX_GROUP_PRIMARY_TDM_RX]) +
			atomic_read(&tdm_group_ref[IDX_GROUP_PRIMARY_TDM_TX]);
	case IDX_GROUP_SECONDARY_TDM_RX:
	case IDX_GROUP_SECONDARY_TDM_TX:
		return atomic_read(&tdm_group_ref[IDX_GROUP_SECONDARY_TDM_RX]) +
			atomic_read(&tdm_group_ref[IDX_GROUP_SECONDARY_TDM_TX]);
	case IDX_GROUP_TERTIARY_TDM_RX:
	case IDX_GROUP_TERTIARY_TDM_TX:
		return atomic_read(&tdm_group_ref[IDX_GROUP_TERTIARY_TDM_RX]) +
			atomic_read(&tdm_group_ref[IDX_GROUP_TERTIARY_TDM_TX]);
	case IDX_GROUP_QUATERNARY_TDM_RX:
	case IDX_GROUP_QUATERNARY_TDM_TX:
		return atomic_read(&tdm_group_ref[IDX_GROUP_QUATERNARY_TDM_RX]) +
			atomic_read(&tdm_group_ref[IDX_GROUP_QUATERNARY_TDM_TX]);
	case IDX_GROUP_QUINARY_TDM_RX:
	case IDX_GROUP_QUINARY_TDM_TX:
		return atomic_read(&tdm_group_ref[IDX_GROUP_QUINARY_TDM_RX]) +
			atomic_read(&tdm_group_ref[IDX_GROUP_QUINARY_TDM_TX]);
	case IDX_GROUP_SENARY_TDM_RX:
	case IDX_GROUP_SENARY_TDM_TX:
		return atomic_read(&tdm_group_ref[IDX_GROUP_SENARY_TDM_RX]) +
			atomic_read(&tdm_group_ref[IDX_GROUP_SENARY_TDM_TX]);
	case IDX_GROUP_SEPTENARY_TDM_RX:
	case IDX_GROUP_SEPTENARY_TDM_TX:
		return atomic_read(&tdm_group_ref[IDX_GROUP_SEPTENARY_TDM_RX]) +
			atomic_read(&tdm_group_ref[IDX_GROUP_SEPTENARY_TDM_TX]);
	case IDX_GROUP_HSIF0_TDM_RX:
	case IDX_GROUP_HSIF0_TDM_TX:
		return atomic_read(&tdm_group_ref[IDX_GROUP_HSIF0_TDM_RX]) +
			atomic_read(&tdm_group_ref[IDX_GROUP_HSIF0_TDM_TX]);
	case IDX_GROUP_HSIF1_TDM_RX:
	case IDX_GROUP_HSIF1_TDM_TX:
		return atomic_read(&tdm_group_ref[IDX_GROUP_HSIF1_TDM_RX]) +
			atomic_read(&tdm_group_ref[IDX_GROUP_HSIF1_TDM_TX]);
	case IDX_GROUP_HSIF2_TDM_RX:
	case IDX_GROUP_HSIF2_TDM_TX:
		return atomic_read(&tdm_group_ref[IDX_GROUP_HSIF2_TDM_RX]) +
			atomic_read(&tdm_group_ref[IDX_GROUP_HSIF2_TDM_TX]);
	default: return -EINVAL;
	}
}

int msm_dai_q6_get_group_idx(u16 id)
{
	switch (id) {
	case AFE_GROUP_DEVICE_ID_PRIMARY_TDM_RX:
	case AFE_PORT_ID_PRIMARY_TDM_RX:
	case AFE_PORT_ID_PRIMARY_TDM_RX_1:
	case AFE_PORT_ID_PRIMARY_TDM_RX_2:
	case AFE_PORT_ID_PRIMARY_TDM_RX_3:
	case AFE_PORT_ID_PRIMARY_TDM_RX_4:
	case AFE_PORT_ID_PRIMARY_TDM_RX_5:
	case AFE_PORT_ID_PRIMARY_TDM_RX_6:
	case AFE_PORT_ID_PRIMARY_TDM_RX_7:
		return IDX_GROUP_PRIMARY_TDM_RX;
	case AFE_GROUP_DEVICE_ID_PRIMARY_TDM_TX:
	case AFE_PORT_ID_PRIMARY_TDM_TX:
	case AFE_PORT_ID_PRIMARY_TDM_TX_1:
	case AFE_PORT_ID_PRIMARY_TDM_TX_2:
	case AFE_PORT_ID_PRIMARY_TDM_TX_3:
	case AFE_PORT_ID_PRIMARY_TDM_TX_4:
	case AFE_PORT_ID_PRIMARY_TDM_TX_5:
	case AFE_PORT_ID_PRIMARY_TDM_TX_6:
	case AFE_PORT_ID_PRIMARY_TDM_TX_7:
		return IDX_GROUP_PRIMARY_TDM_TX;
	case AFE_GROUP_DEVICE_ID_SECONDARY_TDM_RX:
	case AFE_PORT_ID_SECONDARY_TDM_RX:
	case AFE_PORT_ID_SECONDARY_TDM_RX_1:
	case AFE_PORT_ID_SECONDARY_TDM_RX_2:
	case AFE_PORT_ID_SECONDARY_TDM_RX_3:
	case AFE_PORT_ID_SECONDARY_TDM_RX_4:
	case AFE_PORT_ID_SECONDARY_TDM_RX_5:
	case AFE_PORT_ID_SECONDARY_TDM_RX_6:
	case AFE_PORT_ID_SECONDARY_TDM_RX_7:
		return IDX_GROUP_SECONDARY_TDM_RX;
	case AFE_GROUP_DEVICE_ID_SECONDARY_TDM_TX:
	case AFE_PORT_ID_SECONDARY_TDM_TX:
	case AFE_PORT_ID_SECONDARY_TDM_TX_1:
	case AFE_PORT_ID_SECONDARY_TDM_TX_2:
	case AFE_PORT_ID_SECONDARY_TDM_TX_3:
	case AFE_PORT_ID_SECONDARY_TDM_TX_4:
	case AFE_PORT_ID_SECONDARY_TDM_TX_5:
	case AFE_PORT_ID_SECONDARY_TDM_TX_6:
	case AFE_PORT_ID_SECONDARY_TDM_TX_7:
		return IDX_GROUP_SECONDARY_TDM_TX;
	case AFE_GROUP_DEVICE_ID_TERTIARY_TDM_RX:
	case AFE_PORT_ID_TERTIARY_TDM_RX:
	case AFE_PORT_ID_TERTIARY_TDM_RX_1:
	case AFE_PORT_ID_TERTIARY_TDM_RX_2:
	case AFE_PORT_ID_TERTIARY_TDM_RX_3:
	case AFE_PORT_ID_TERTIARY_TDM_RX_4:
	case AFE_PORT_ID_TERTIARY_TDM_RX_5:
	case AFE_PORT_ID_TERTIARY_TDM_RX_6:
	case AFE_PORT_ID_TERTIARY_TDM_RX_7:
		return IDX_GROUP_TERTIARY_TDM_RX;
	case AFE_GROUP_DEVICE_ID_TERTIARY_TDM_TX:
	case AFE_PORT_ID_TERTIARY_TDM_TX:
	case AFE_PORT_ID_TERTIARY_TDM_TX_1:
	case AFE_PORT_ID_TERTIARY_TDM_TX_2:
	case AFE_PORT_ID_TERTIARY_TDM_TX_3:
	case AFE_PORT_ID_TERTIARY_TDM_TX_4:
	case AFE_PORT_ID_TERTIARY_TDM_TX_5:
	case AFE_PORT_ID_TERTIARY_TDM_TX_6:
	case AFE_PORT_ID_TERTIARY_TDM_TX_7:
		return IDX_GROUP_TERTIARY_TDM_TX;
	case AFE_GROUP_DEVICE_ID_QUATERNARY_TDM_RX:
	case AFE_PORT_ID_QUATERNARY_TDM_RX:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_1:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_2:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_3:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_4:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_5:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_6:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_7:
		return IDX_GROUP_QUATERNARY_TDM_RX;
	case AFE_GROUP_DEVICE_ID_QUATERNARY_TDM_TX:
	case AFE_PORT_ID_QUATERNARY_TDM_TX:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_1:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_2:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_3:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_4:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_5:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_6:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_7:
		return IDX_GROUP_QUATERNARY_TDM_TX;
	case AFE_GROUP_DEVICE_ID_QUINARY_TDM_RX:
	case AFE_PORT_ID_QUINARY_TDM_RX:
	case AFE_PORT_ID_QUINARY_TDM_RX_1:
	case AFE_PORT_ID_QUINARY_TDM_RX_2:
	case AFE_PORT_ID_QUINARY_TDM_RX_3:
	case AFE_PORT_ID_QUINARY_TDM_RX_4:
	case AFE_PORT_ID_QUINARY_TDM_RX_5:
	case AFE_PORT_ID_QUINARY_TDM_RX_6:
	case AFE_PORT_ID_QUINARY_TDM_RX_7:
		return IDX_GROUP_QUINARY_TDM_RX;
	case AFE_GROUP_DEVICE_ID_QUINARY_TDM_TX:
	case AFE_PORT_ID_QUINARY_TDM_TX:
	case AFE_PORT_ID_QUINARY_TDM_TX_1:
	case AFE_PORT_ID_QUINARY_TDM_TX_2:
	case AFE_PORT_ID_QUINARY_TDM_TX_3:
	case AFE_PORT_ID_QUINARY_TDM_TX_4:
	case AFE_PORT_ID_QUINARY_TDM_TX_5:
	case AFE_PORT_ID_QUINARY_TDM_TX_6:
	case AFE_PORT_ID_QUINARY_TDM_TX_7:
		return IDX_GROUP_QUINARY_TDM_TX;
	case AFE_GROUP_DEVICE_ID_SENARY_TDM_RX:
	case AFE_PORT_ID_SENARY_TDM_RX:
	case AFE_PORT_ID_SENARY_TDM_RX_1:
	case AFE_PORT_ID_SENARY_TDM_RX_2:
	case AFE_PORT_ID_SENARY_TDM_RX_3:
	case AFE_PORT_ID_SENARY_TDM_RX_4:
	case AFE_PORT_ID_SENARY_TDM_RX_5:
	case AFE_PORT_ID_SENARY_TDM_RX_6:
	case AFE_PORT_ID_SENARY_TDM_RX_7:
		return IDX_GROUP_SENARY_TDM_RX;
	case AFE_GROUP_DEVICE_ID_SENARY_TDM_TX:
	case AFE_PORT_ID_SENARY_TDM_TX:
	case AFE_PORT_ID_SENARY_TDM_TX_1:
	case AFE_PORT_ID_SENARY_TDM_TX_2:
	case AFE_PORT_ID_SENARY_TDM_TX_3:
	case AFE_PORT_ID_SENARY_TDM_TX_4:
	case AFE_PORT_ID_SENARY_TDM_TX_5:
	case AFE_PORT_ID_SENARY_TDM_TX_6:
	case AFE_PORT_ID_SENARY_TDM_TX_7:
		return IDX_GROUP_SENARY_TDM_TX;
	case AFE_GROUP_DEVICE_ID_SEPTENARY_TDM_RX:
	case AFE_PORT_ID_SEPTENARY_TDM_RX:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_1:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_2:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_3:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_4:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_5:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_6:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_7:
		return IDX_GROUP_SEPTENARY_TDM_RX;
	case AFE_GROUP_DEVICE_ID_SEPTENARY_TDM_TX:
	case AFE_PORT_ID_SEPTENARY_TDM_TX:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_1:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_2:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_3:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_4:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_5:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_6:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_7:
		return IDX_GROUP_SEPTENARY_TDM_TX;
	case AFE_GROUP_DEVICE_ID_HSIF0_TDM_RX:
	case AFE_PORT_ID_HSIF0_TDM_RX:
	case AFE_PORT_ID_HSIF0_TDM_RX_1:
	case AFE_PORT_ID_HSIF0_TDM_RX_2:
	case AFE_PORT_ID_HSIF0_TDM_RX_3:
	case AFE_PORT_ID_HSIF0_TDM_RX_4:
	case AFE_PORT_ID_HSIF0_TDM_RX_5:
	case AFE_PORT_ID_HSIF0_TDM_RX_6:
	case AFE_PORT_ID_HSIF0_TDM_RX_7:
		return IDX_GROUP_HSIF0_TDM_RX;
	case AFE_GROUP_DEVICE_ID_HSIF0_TDM_TX:
	case AFE_PORT_ID_HSIF0_TDM_TX:
	case AFE_PORT_ID_HSIF0_TDM_TX_1:
	case AFE_PORT_ID_HSIF0_TDM_TX_2:
	case AFE_PORT_ID_HSIF0_TDM_TX_3:
	case AFE_PORT_ID_HSIF0_TDM_TX_4:
	case AFE_PORT_ID_HSIF0_TDM_TX_5:
	case AFE_PORT_ID_HSIF0_TDM_TX_6:
	case AFE_PORT_ID_HSIF0_TDM_TX_7:
		return IDX_GROUP_HSIF0_TDM_TX;
	case AFE_GROUP_DEVICE_ID_HSIF1_TDM_RX:
	case AFE_PORT_ID_HSIF1_TDM_RX:
	case AFE_PORT_ID_HSIF1_TDM_RX_1:
	case AFE_PORT_ID_HSIF1_TDM_RX_2:
	case AFE_PORT_ID_HSIF1_TDM_RX_3:
	case AFE_PORT_ID_HSIF1_TDM_RX_4:
	case AFE_PORT_ID_HSIF1_TDM_RX_5:
	case AFE_PORT_ID_HSIF1_TDM_RX_6:
	case AFE_PORT_ID_HSIF1_TDM_RX_7:
		return IDX_GROUP_HSIF1_TDM_RX;
	case AFE_GROUP_DEVICE_ID_HSIF1_TDM_TX:
	case AFE_PORT_ID_HSIF1_TDM_TX:
	case AFE_PORT_ID_HSIF1_TDM_TX_1:
	case AFE_PORT_ID_HSIF1_TDM_TX_2:
	case AFE_PORT_ID_HSIF1_TDM_TX_3:
	case AFE_PORT_ID_HSIF1_TDM_TX_4:
	case AFE_PORT_ID_HSIF1_TDM_TX_5:
	case AFE_PORT_ID_HSIF1_TDM_TX_6:
	case AFE_PORT_ID_HSIF1_TDM_TX_7:
		return IDX_GROUP_HSIF1_TDM_TX;
	case AFE_GROUP_DEVICE_ID_HSIF2_TDM_RX:
	case AFE_PORT_ID_HSIF2_TDM_RX:
	case AFE_PORT_ID_HSIF2_TDM_RX_1:
	case AFE_PORT_ID_HSIF2_TDM_RX_2:
	case AFE_PORT_ID_HSIF2_TDM_RX_3:
	case AFE_PORT_ID_HSIF2_TDM_RX_4:
	case AFE_PORT_ID_HSIF2_TDM_RX_5:
	case AFE_PORT_ID_HSIF2_TDM_RX_6:
	case AFE_PORT_ID_HSIF2_TDM_RX_7:
		return IDX_GROUP_HSIF2_TDM_RX;
	case AFE_GROUP_DEVICE_ID_HSIF2_TDM_TX:
	case AFE_PORT_ID_HSIF2_TDM_TX:
	case AFE_PORT_ID_HSIF2_TDM_TX_1:
	case AFE_PORT_ID_HSIF2_TDM_TX_2:
	case AFE_PORT_ID_HSIF2_TDM_TX_3:
	case AFE_PORT_ID_HSIF2_TDM_TX_4:
	case AFE_PORT_ID_HSIF2_TDM_TX_5:
	case AFE_PORT_ID_HSIF2_TDM_TX_6:
	case AFE_PORT_ID_HSIF2_TDM_TX_7:
		return IDX_GROUP_HSIF2_TDM_TX;
	default: return -EINVAL;
	}
}

int msm_dai_q6_get_port_idx(u16 id)
{
	switch (id) {
	case AFE_PORT_ID_PRIMARY_TDM_RX:
		return IDX_PRIMARY_TDM_RX_0;
	case AFE_PORT_ID_PRIMARY_TDM_TX:
		return IDX_PRIMARY_TDM_TX_0;
	case AFE_PORT_ID_PRIMARY_TDM_RX_1:
		return IDX_PRIMARY_TDM_RX_1;
	case AFE_PORT_ID_PRIMARY_TDM_TX_1:
		return IDX_PRIMARY_TDM_TX_1;
	case AFE_PORT_ID_PRIMARY_TDM_RX_2:
		return IDX_PRIMARY_TDM_RX_2;
	case AFE_PORT_ID_PRIMARY_TDM_TX_2:
		return IDX_PRIMARY_TDM_TX_2;
	case AFE_PORT_ID_PRIMARY_TDM_RX_3:
		return IDX_PRIMARY_TDM_RX_3;
	case AFE_PORT_ID_PRIMARY_TDM_TX_3:
		return IDX_PRIMARY_TDM_TX_3;
	case AFE_PORT_ID_PRIMARY_TDM_RX_4:
		return IDX_PRIMARY_TDM_RX_4;
	case AFE_PORT_ID_PRIMARY_TDM_TX_4:
		return IDX_PRIMARY_TDM_TX_4;
	case AFE_PORT_ID_PRIMARY_TDM_RX_5:
		return IDX_PRIMARY_TDM_RX_5;
	case AFE_PORT_ID_PRIMARY_TDM_TX_5:
		return IDX_PRIMARY_TDM_TX_5;
	case AFE_PORT_ID_PRIMARY_TDM_RX_6:
		return IDX_PRIMARY_TDM_RX_6;
	case AFE_PORT_ID_PRIMARY_TDM_TX_6:
		return IDX_PRIMARY_TDM_TX_6;
	case AFE_PORT_ID_PRIMARY_TDM_RX_7:
		return IDX_PRIMARY_TDM_RX_7;
	case AFE_PORT_ID_PRIMARY_TDM_TX_7:
		return IDX_PRIMARY_TDM_TX_7;
	case AFE_PORT_ID_SECONDARY_TDM_RX:
		return IDX_SECONDARY_TDM_RX_0;
	case AFE_PORT_ID_SECONDARY_TDM_TX:
		return IDX_SECONDARY_TDM_TX_0;
	case AFE_PORT_ID_SECONDARY_TDM_RX_1:
		return IDX_SECONDARY_TDM_RX_1;
	case AFE_PORT_ID_SECONDARY_TDM_TX_1:
		return IDX_SECONDARY_TDM_TX_1;
	case AFE_PORT_ID_SECONDARY_TDM_RX_2:
		return IDX_SECONDARY_TDM_RX_2;
	case AFE_PORT_ID_SECONDARY_TDM_TX_2:
		return IDX_SECONDARY_TDM_TX_2;
	case AFE_PORT_ID_SECONDARY_TDM_RX_3:
		return IDX_SECONDARY_TDM_RX_3;
	case AFE_PORT_ID_SECONDARY_TDM_TX_3:
		return IDX_SECONDARY_TDM_TX_3;
	case AFE_PORT_ID_SECONDARY_TDM_RX_4:
		return IDX_SECONDARY_TDM_RX_4;
	case AFE_PORT_ID_SECONDARY_TDM_TX_4:
		return IDX_SECONDARY_TDM_TX_4;
	case AFE_PORT_ID_SECONDARY_TDM_RX_5:
		return IDX_SECONDARY_TDM_RX_5;
	case AFE_PORT_ID_SECONDARY_TDM_TX_5:
		return IDX_SECONDARY_TDM_TX_5;
	case AFE_PORT_ID_SECONDARY_TDM_RX_6:
		return IDX_SECONDARY_TDM_RX_6;
	case AFE_PORT_ID_SECONDARY_TDM_TX_6:
		return IDX_SECONDARY_TDM_TX_6;
	case AFE_PORT_ID_SECONDARY_TDM_RX_7:
		return IDX_SECONDARY_TDM_RX_7;
	case AFE_PORT_ID_SECONDARY_TDM_TX_7:
		return IDX_SECONDARY_TDM_TX_7;
	case AFE_PORT_ID_TERTIARY_TDM_RX:
		return IDX_TERTIARY_TDM_RX_0;
	case AFE_PORT_ID_TERTIARY_TDM_TX:
		return IDX_TERTIARY_TDM_TX_0;
	case AFE_PORT_ID_TERTIARY_TDM_RX_1:
		return IDX_TERTIARY_TDM_RX_1;
	case AFE_PORT_ID_TERTIARY_TDM_TX_1:
		return IDX_TERTIARY_TDM_TX_1;
	case AFE_PORT_ID_TERTIARY_TDM_RX_2:
		return IDX_TERTIARY_TDM_RX_2;
	case AFE_PORT_ID_TERTIARY_TDM_TX_2:
		return IDX_TERTIARY_TDM_TX_2;
	case AFE_PORT_ID_TERTIARY_TDM_RX_3:
		return IDX_TERTIARY_TDM_RX_3;
	case AFE_PORT_ID_TERTIARY_TDM_TX_3:
		return IDX_TERTIARY_TDM_TX_3;
	case AFE_PORT_ID_TERTIARY_TDM_RX_4:
		return IDX_TERTIARY_TDM_RX_4;
	case AFE_PORT_ID_TERTIARY_TDM_TX_4:
		return IDX_TERTIARY_TDM_TX_4;
	case AFE_PORT_ID_TERTIARY_TDM_RX_5:
		return IDX_TERTIARY_TDM_RX_5;
	case AFE_PORT_ID_TERTIARY_TDM_TX_5:
		return IDX_TERTIARY_TDM_TX_5;
	case AFE_PORT_ID_TERTIARY_TDM_RX_6:
		return IDX_TERTIARY_TDM_RX_6;
	case AFE_PORT_ID_TERTIARY_TDM_TX_6:
		return IDX_TERTIARY_TDM_TX_6;
	case AFE_PORT_ID_TERTIARY_TDM_RX_7:
		return IDX_TERTIARY_TDM_RX_7;
	case AFE_PORT_ID_TERTIARY_TDM_TX_7:
		return IDX_TERTIARY_TDM_TX_7;
	case AFE_PORT_ID_QUATERNARY_TDM_RX:
		return IDX_QUATERNARY_TDM_RX_0;
	case AFE_PORT_ID_QUATERNARY_TDM_TX:
		return IDX_QUATERNARY_TDM_TX_0;
	case AFE_PORT_ID_QUATERNARY_TDM_RX_1:
		return IDX_QUATERNARY_TDM_RX_1;
	case AFE_PORT_ID_QUATERNARY_TDM_TX_1:
		return IDX_QUATERNARY_TDM_TX_1;
	case AFE_PORT_ID_QUATERNARY_TDM_RX_2:
		return IDX_QUATERNARY_TDM_RX_2;
	case AFE_PORT_ID_QUATERNARY_TDM_TX_2:
		return IDX_QUATERNARY_TDM_TX_2;
	case AFE_PORT_ID_QUATERNARY_TDM_RX_3:
		return IDX_QUATERNARY_TDM_RX_3;
	case AFE_PORT_ID_QUATERNARY_TDM_TX_3:
		return IDX_QUATERNARY_TDM_TX_3;
	case AFE_PORT_ID_QUATERNARY_TDM_RX_4:
		return IDX_QUATERNARY_TDM_RX_4;
	case AFE_PORT_ID_QUATERNARY_TDM_TX_4:
		return IDX_QUATERNARY_TDM_TX_4;
	case AFE_PORT_ID_QUATERNARY_TDM_RX_5:
		return IDX_QUATERNARY_TDM_RX_5;
	case AFE_PORT_ID_QUATERNARY_TDM_TX_5:
		return IDX_QUATERNARY_TDM_TX_5;
	case AFE_PORT_ID_QUATERNARY_TDM_RX_6:
		return IDX_QUATERNARY_TDM_RX_6;
	case AFE_PORT_ID_QUATERNARY_TDM_TX_6:
		return IDX_QUATERNARY_TDM_TX_6;
	case AFE_PORT_ID_QUATERNARY_TDM_RX_7:
		return IDX_QUATERNARY_TDM_RX_7;
	case AFE_PORT_ID_QUATERNARY_TDM_TX_7:
		return IDX_QUATERNARY_TDM_TX_7;
	case AFE_PORT_ID_QUINARY_TDM_RX:
		return IDX_QUINARY_TDM_RX_0;
	case AFE_PORT_ID_QUINARY_TDM_TX:
		return IDX_QUINARY_TDM_TX_0;
	case AFE_PORT_ID_QUINARY_TDM_RX_1:
		return IDX_QUINARY_TDM_RX_1;
	case AFE_PORT_ID_QUINARY_TDM_TX_1:
		return IDX_QUINARY_TDM_TX_1;
	case AFE_PORT_ID_QUINARY_TDM_RX_2:
		return IDX_QUINARY_TDM_RX_2;
	case AFE_PORT_ID_QUINARY_TDM_TX_2:
		return IDX_QUINARY_TDM_TX_2;
	case AFE_PORT_ID_QUINARY_TDM_RX_3:
		return IDX_QUINARY_TDM_RX_3;
	case AFE_PORT_ID_QUINARY_TDM_TX_3:
		return IDX_QUINARY_TDM_TX_3;
	case AFE_PORT_ID_QUINARY_TDM_RX_4:
		return IDX_QUINARY_TDM_RX_4;
	case AFE_PORT_ID_QUINARY_TDM_TX_4:
		return IDX_QUINARY_TDM_TX_4;
	case AFE_PORT_ID_QUINARY_TDM_RX_5:
		return IDX_QUINARY_TDM_RX_5;
	case AFE_PORT_ID_QUINARY_TDM_TX_5:
		return IDX_QUINARY_TDM_TX_5;
	case AFE_PORT_ID_QUINARY_TDM_RX_6:
		return IDX_QUINARY_TDM_RX_6;
	case AFE_PORT_ID_QUINARY_TDM_TX_6:
		return IDX_QUINARY_TDM_TX_6;
	case AFE_PORT_ID_QUINARY_TDM_RX_7:
		return IDX_QUINARY_TDM_RX_7;
	case AFE_PORT_ID_QUINARY_TDM_TX_7:
		return IDX_QUINARY_TDM_TX_7;
	case AFE_PORT_ID_SENARY_TDM_RX:
		return IDX_SENARY_TDM_RX_0;
	case AFE_PORT_ID_SENARY_TDM_TX:
		return IDX_SENARY_TDM_TX_0;
	case AFE_PORT_ID_SENARY_TDM_RX_1:
		return IDX_SENARY_TDM_RX_1;
	case AFE_PORT_ID_SENARY_TDM_TX_1:
		return IDX_SENARY_TDM_TX_1;
	case AFE_PORT_ID_SENARY_TDM_RX_2:
		return IDX_SENARY_TDM_RX_2;
	case AFE_PORT_ID_SENARY_TDM_TX_2:
		return IDX_SENARY_TDM_TX_2;
	case AFE_PORT_ID_SENARY_TDM_RX_3:
		return IDX_SENARY_TDM_RX_3;
	case AFE_PORT_ID_SENARY_TDM_TX_3:
		return IDX_SENARY_TDM_TX_3;
	case AFE_PORT_ID_SENARY_TDM_RX_4:
		return IDX_SENARY_TDM_RX_4;
	case AFE_PORT_ID_SENARY_TDM_TX_4:
		return IDX_SENARY_TDM_TX_4;
	case AFE_PORT_ID_SENARY_TDM_RX_5:
		return IDX_SENARY_TDM_RX_5;
	case AFE_PORT_ID_SENARY_TDM_TX_5:
		return IDX_SENARY_TDM_TX_5;
	case AFE_PORT_ID_SENARY_TDM_RX_6:
		return IDX_SENARY_TDM_RX_6;
	case AFE_PORT_ID_SENARY_TDM_TX_6:
		return IDX_SENARY_TDM_TX_6;
	case AFE_PORT_ID_SENARY_TDM_RX_7:
		return IDX_SENARY_TDM_RX_7;
	case AFE_PORT_ID_SENARY_TDM_TX_7:
		return IDX_SENARY_TDM_TX_7;
	case AFE_PORT_ID_SEPTENARY_TDM_RX:
		return IDX_SEPTENARY_TDM_RX_0;
	case AFE_PORT_ID_SEPTENARY_TDM_TX:
		return IDX_SEPTENARY_TDM_TX_0;
	case AFE_PORT_ID_SEPTENARY_TDM_RX_1:
		return IDX_SEPTENARY_TDM_RX_1;
	case AFE_PORT_ID_SEPTENARY_TDM_TX_1:
		return IDX_SEPTENARY_TDM_TX_1;
	case AFE_PORT_ID_SEPTENARY_TDM_RX_2:
		return IDX_SEPTENARY_TDM_RX_2;
	case AFE_PORT_ID_SEPTENARY_TDM_TX_2:
		return IDX_SEPTENARY_TDM_TX_2;
	case AFE_PORT_ID_SEPTENARY_TDM_RX_3:
		return IDX_SEPTENARY_TDM_RX_3;
	case AFE_PORT_ID_SEPTENARY_TDM_TX_3:
		return IDX_SEPTENARY_TDM_TX_3;
	case AFE_PORT_ID_SEPTENARY_TDM_RX_4:
		return IDX_SEPTENARY_TDM_RX_4;
	case AFE_PORT_ID_SEPTENARY_TDM_TX_4:
		return IDX_SEPTENARY_TDM_TX_4;
	case AFE_PORT_ID_SEPTENARY_TDM_RX_5:
		return IDX_SEPTENARY_TDM_RX_5;
	case AFE_PORT_ID_SEPTENARY_TDM_TX_5:
		return IDX_SEPTENARY_TDM_TX_5;
	case AFE_PORT_ID_SEPTENARY_TDM_RX_6:
		return IDX_SEPTENARY_TDM_RX_6;
	case AFE_PORT_ID_SEPTENARY_TDM_TX_6:
		return IDX_SEPTENARY_TDM_TX_6;
	case AFE_PORT_ID_SEPTENARY_TDM_RX_7:
		return IDX_SEPTENARY_TDM_RX_7;
	case AFE_PORT_ID_SEPTENARY_TDM_TX_7:
		return IDX_SEPTENARY_TDM_TX_7;
	case AFE_PORT_ID_HSIF0_TDM_RX:
		return IDX_HSIF0_TDM_RX_0;
	case AFE_PORT_ID_HSIF0_TDM_TX:
		return IDX_HSIF0_TDM_TX_0;
	case AFE_PORT_ID_HSIF0_TDM_RX_1:
		return IDX_HSIF0_TDM_RX_1;
	case AFE_PORT_ID_HSIF0_TDM_TX_1:
		return IDX_HSIF0_TDM_TX_1;
	case AFE_PORT_ID_HSIF0_TDM_RX_2:
		return IDX_HSIF0_TDM_RX_2;
	case AFE_PORT_ID_HSIF0_TDM_TX_2:
		return IDX_HSIF0_TDM_TX_2;
	case AFE_PORT_ID_HSIF0_TDM_RX_3:
		return IDX_HSIF0_TDM_RX_3;
	case AFE_PORT_ID_HSIF0_TDM_TX_3:
		return IDX_HSIF0_TDM_TX_3;
	case AFE_PORT_ID_HSIF0_TDM_RX_4:
		return IDX_HSIF0_TDM_RX_4;
	case AFE_PORT_ID_HSIF0_TDM_TX_4:
		return IDX_HSIF0_TDM_TX_4;
	case AFE_PORT_ID_HSIF0_TDM_RX_5:
		return IDX_HSIF0_TDM_RX_5;
	case AFE_PORT_ID_HSIF0_TDM_TX_5:
		return IDX_HSIF0_TDM_TX_5;
	case AFE_PORT_ID_HSIF0_TDM_RX_6:
		return IDX_HSIF0_TDM_RX_6;
	case AFE_PORT_ID_HSIF0_TDM_TX_6:
		return IDX_HSIF0_TDM_TX_6;
	case AFE_PORT_ID_HSIF0_TDM_RX_7:
		return IDX_HSIF0_TDM_RX_7;
	case AFE_PORT_ID_HSIF0_TDM_TX_7:
		return IDX_HSIF0_TDM_TX_7;
	case AFE_PORT_ID_HSIF1_TDM_RX:
		return IDX_HSIF1_TDM_RX_0;
	case AFE_PORT_ID_HSIF1_TDM_TX:
		return IDX_HSIF1_TDM_TX_0;
	case AFE_PORT_ID_HSIF1_TDM_RX_1:
		return IDX_HSIF1_TDM_RX_1;
	case AFE_PORT_ID_HSIF1_TDM_TX_1:
		return IDX_HSIF1_TDM_TX_1;
	case AFE_PORT_ID_HSIF1_TDM_RX_2:
		return IDX_HSIF1_TDM_RX_2;
	case AFE_PORT_ID_HSIF1_TDM_TX_2:
		return IDX_HSIF1_TDM_TX_2;
	case AFE_PORT_ID_HSIF1_TDM_RX_3:
		return IDX_HSIF1_TDM_RX_3;
	case AFE_PORT_ID_HSIF1_TDM_TX_3:
		return IDX_HSIF1_TDM_TX_3;
	case AFE_PORT_ID_HSIF1_TDM_RX_4:
		return IDX_HSIF1_TDM_RX_4;
	case AFE_PORT_ID_HSIF1_TDM_TX_4:
		return IDX_HSIF1_TDM_TX_4;
	case AFE_PORT_ID_HSIF1_TDM_RX_5:
		return IDX_HSIF1_TDM_RX_5;
	case AFE_PORT_ID_HSIF1_TDM_TX_5:
		return IDX_HSIF1_TDM_TX_5;
	case AFE_PORT_ID_HSIF1_TDM_RX_6:
		return IDX_HSIF1_TDM_RX_6;
	case AFE_PORT_ID_HSIF1_TDM_TX_6:
		return IDX_HSIF1_TDM_TX_6;
	case AFE_PORT_ID_HSIF1_TDM_RX_7:
		return IDX_HSIF1_TDM_RX_7;
	case AFE_PORT_ID_HSIF1_TDM_TX_7:
		return IDX_HSIF1_TDM_TX_7;
	case AFE_PORT_ID_HSIF2_TDM_RX:
		return IDX_HSIF2_TDM_RX_0;
	case AFE_PORT_ID_HSIF2_TDM_TX:
		return IDX_HSIF2_TDM_TX_0;
	case AFE_PORT_ID_HSIF2_TDM_RX_1:
		return IDX_HSIF2_TDM_RX_1;
	case AFE_PORT_ID_HSIF2_TDM_TX_1:
		return IDX_HSIF2_TDM_TX_1;
	case AFE_PORT_ID_HSIF2_TDM_RX_2:
		return IDX_HSIF2_TDM_RX_2;
	case AFE_PORT_ID_HSIF2_TDM_TX_2:
		return IDX_HSIF2_TDM_TX_2;
	case AFE_PORT_ID_HSIF2_TDM_RX_3:
		return IDX_HSIF2_TDM_RX_3;
	case AFE_PORT_ID_HSIF2_TDM_TX_3:
		return IDX_HSIF2_TDM_TX_3;
	case AFE_PORT_ID_HSIF2_TDM_RX_4:
		return IDX_HSIF2_TDM_RX_4;
	case AFE_PORT_ID_HSIF2_TDM_TX_4:
		return IDX_HSIF2_TDM_TX_4;
	case AFE_PORT_ID_HSIF2_TDM_RX_5:
		return IDX_HSIF2_TDM_RX_5;
	case AFE_PORT_ID_HSIF2_TDM_TX_5:
		return IDX_HSIF2_TDM_TX_5;
	case AFE_PORT_ID_HSIF2_TDM_RX_6:
		return IDX_HSIF2_TDM_RX_6;
	case AFE_PORT_ID_HSIF2_TDM_TX_6:
		return IDX_HSIF2_TDM_TX_6;
	case AFE_PORT_ID_HSIF2_TDM_RX_7:
		return IDX_HSIF2_TDM_RX_7;
	case AFE_PORT_ID_HSIF2_TDM_TX_7:
		return IDX_HSIF2_TDM_TX_7;
	default: return -EINVAL;
	}
}

static u16 msm_dai_q6_max_num_slot(int frame_rate)
{
	/* Max num of slots is bits per frame divided
	 * by bits per sample which is 16
	 */
	switch (frame_rate) {
	case AFE_PORT_PCM_BITS_PER_FRAME_8:
		return 0;
	case AFE_PORT_PCM_BITS_PER_FRAME_16:
		return 1;
	case AFE_PORT_PCM_BITS_PER_FRAME_32:
		return 2;
	case AFE_PORT_PCM_BITS_PER_FRAME_64:
		return 4;
	case AFE_PORT_PCM_BITS_PER_FRAME_128:
		return 8;
	case AFE_PORT_PCM_BITS_PER_FRAME_256:
		return 16;
	default:
		pr_err("%s Invalid bits per frame %d\n",
			__func__, frame_rate);
		return 0;
	}
}

static int msm_dai_q6_dai_add_route(struct snd_soc_dai *dai)
{
	struct snd_soc_dapm_route intercon;
	struct snd_soc_dapm_context *dapm;

	if (!dai) {
		pr_err("%s: Invalid params dai\n", __func__);
		return -EINVAL;
	}
	if (!dai->driver) {
		pr_err("%s: Invalid params dai driver\n", __func__);
		return -EINVAL;
	}
	dapm = snd_soc_component_get_dapm(dai->component);
	memset(&intercon, 0, sizeof(intercon));
	if (dai->driver->playback.stream_name &&
		dai->driver->playback.aif_name) {
		dev_dbg(dai->dev, "%s: add route for widget %s",
				__func__, dai->driver->playback.stream_name);
		intercon.source = dai->driver->playback.aif_name;
		intercon.sink = dai->driver->playback.stream_name;
		dev_dbg(dai->dev, "%s: src %s sink %s\n",
				__func__, intercon.source, intercon.sink);
		snd_soc_dapm_add_routes(dapm, &intercon, 1);
		snd_soc_dapm_ignore_suspend(dapm, intercon.sink);
	}
	if (dai->driver->capture.stream_name &&
		dai->driver->capture.aif_name) {
		dev_dbg(dai->dev, "%s: add route for widget %s",
				__func__, dai->driver->capture.stream_name);
		intercon.sink = dai->driver->capture.aif_name;
		intercon.source = dai->driver->capture.stream_name;
		dev_dbg(dai->dev, "%s: src %s sink %s\n",
				__func__, intercon.source, intercon.sink);
		snd_soc_dapm_add_routes(dapm, &intercon, 1);
		snd_soc_dapm_ignore_suspend(dapm, intercon.source);
	}
	return 0;
}

static int msm_dai_q6_auxpcm_hw_params(
				struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct msm_dai_q6_auxpcm_dai_data *aux_dai_data =
			dev_get_drvdata(dai->dev);
	struct msm_dai_q6_dai_data *dai_data = &aux_dai_data->bdai_data;
	struct msm_dai_auxpcm_pdata *auxpcm_pdata =
			(struct msm_dai_auxpcm_pdata *) dai->dev->platform_data;
	int rc = 0, slot_mapping_copy_len = 0;

	if (params_channels(params) != 1 || (params_rate(params) != 8000 &&
	    params_rate(params) != 16000)) {
		dev_err(dai->dev, "%s: invalid param chan %d rate %d\n",
			__func__, params_channels(params), params_rate(params));
		return -EINVAL;
	}

	mutex_lock(&aux_dai_data->rlock);

	if (test_bit(STATUS_TX_PORT, aux_dai_data->auxpcm_port_status) ||
	    test_bit(STATUS_RX_PORT, aux_dai_data->auxpcm_port_status)) {
		/* AUXPCM DAI in use */
		if (dai_data->rate != params_rate(params)) {
			dev_err(dai->dev, "%s: rate mismatch of running DAI\n",
			__func__);
			rc = -EINVAL;
		}
		mutex_unlock(&aux_dai_data->rlock);
		return rc;
	}

	dai_data->channels = params_channels(params);
	dai_data->rate = params_rate(params);

	if (dai_data->rate == 8000) {
		dai_data->port_config.pcm.pcm_cfg_minor_version =
				AFE_API_VERSION_PCM_CONFIG;
		dai_data->port_config.pcm.aux_mode = auxpcm_pdata->mode_8k.mode;
		dai_data->port_config.pcm.sync_src = auxpcm_pdata->mode_8k.sync;
		dai_data->port_config.pcm.frame_setting =
					auxpcm_pdata->mode_8k.frame;
		dai_data->port_config.pcm.quantype =
					 auxpcm_pdata->mode_8k.quant;
		dai_data->port_config.pcm.ctrl_data_out_enable =
					 auxpcm_pdata->mode_8k.data;
		dai_data->port_config.pcm.sample_rate = dai_data->rate;
		dai_data->port_config.pcm.num_channels = dai_data->channels;
		dai_data->port_config.pcm.bit_width = 16;
		if (ARRAY_SIZE(dai_data->port_config.pcm.slot_number_mapping) <=
		    auxpcm_pdata->mode_8k.num_slots)
			slot_mapping_copy_len =
				ARRAY_SIZE(
				dai_data->port_config.pcm.slot_number_mapping)
				 * sizeof(uint16_t);
		else
			slot_mapping_copy_len = auxpcm_pdata->mode_8k.num_slots
				* sizeof(uint16_t);

		if (auxpcm_pdata->mode_8k.slot_mapping) {
			memcpy(dai_data->port_config.pcm.slot_number_mapping,
			       auxpcm_pdata->mode_8k.slot_mapping,
			       slot_mapping_copy_len);
		} else {
			dev_err(dai->dev, "%s 8khz slot mapping is NULL\n",
				__func__);
			mutex_unlock(&aux_dai_data->rlock);
			return -EINVAL;
		}
	} else {
		dai_data->port_config.pcm.pcm_cfg_minor_version =
				AFE_API_VERSION_PCM_CONFIG;
		dai_data->port_config.pcm.aux_mode =
					auxpcm_pdata->mode_16k.mode;
		dai_data->port_config.pcm.sync_src =
					auxpcm_pdata->mode_16k.sync;
		dai_data->port_config.pcm.frame_setting =
					auxpcm_pdata->mode_16k.frame;
		dai_data->port_config.pcm.quantype =
					auxpcm_pdata->mode_16k.quant;
		dai_data->port_config.pcm.ctrl_data_out_enable =
					auxpcm_pdata->mode_16k.data;
		dai_data->port_config.pcm.sample_rate = dai_data->rate;
		dai_data->port_config.pcm.num_channels = dai_data->channels;
		dai_data->port_config.pcm.bit_width = 16;
		if (ARRAY_SIZE(dai_data->port_config.pcm.slot_number_mapping) <=
		    auxpcm_pdata->mode_16k.num_slots)
			slot_mapping_copy_len =
				ARRAY_SIZE(
				dai_data->port_config.pcm.slot_number_mapping)
				 * sizeof(uint16_t);
		else
			slot_mapping_copy_len = auxpcm_pdata->mode_16k.num_slots
				* sizeof(uint16_t);

		if (auxpcm_pdata->mode_16k.slot_mapping) {
			memcpy(dai_data->port_config.pcm.slot_number_mapping,
			       auxpcm_pdata->mode_16k.slot_mapping,
			       slot_mapping_copy_len);
		} else {
			dev_err(dai->dev, "%s 16khz slot mapping is NULL\n",
				__func__);
			mutex_unlock(&aux_dai_data->rlock);
			return -EINVAL;
		}
	}

	dev_dbg(dai->dev, "%s: aux_mode 0x%x sync_src 0x%x frame_setting 0x%x\n",
		__func__, dai_data->port_config.pcm.aux_mode,
		dai_data->port_config.pcm.sync_src,
		dai_data->port_config.pcm.frame_setting);
	dev_dbg(dai->dev, "%s: qtype 0x%x dout 0x%x num_map[0] 0x%x\n"
		"num_map[1] 0x%x num_map[2] 0x%x num_map[3] 0x%x\n",
		__func__, dai_data->port_config.pcm.quantype,
		dai_data->port_config.pcm.ctrl_data_out_enable,
		dai_data->port_config.pcm.slot_number_mapping[0],
		dai_data->port_config.pcm.slot_number_mapping[1],
		dai_data->port_config.pcm.slot_number_mapping[2],
		dai_data->port_config.pcm.slot_number_mapping[3]);

	mutex_unlock(&aux_dai_data->rlock);
	return rc;
}

static int msm_dai_q6_auxpcm_set_clk(
		struct msm_dai_q6_auxpcm_dai_data *aux_dai_data,
		u16 port_id, bool enable)
{
	int rc;

	pr_debug("%s: afe_clk_ver: %d, port_id: %d, enable: %d\n", __func__,
		 aux_dai_data->afe_clk_ver, port_id, enable);
	if (aux_dai_data->afe_clk_ver == AFE_CLK_VERSION_V2) {
		aux_dai_data->clk_set.enable = enable;
		rc = afe_set_lpass_clock_v2(port_id,
					&aux_dai_data->clk_set);
	} else {
		if (!enable)
			aux_dai_data->clk_cfg.clk_val1 = 0;
		rc = afe_set_lpass_clock(port_id,
					&aux_dai_data->clk_cfg);
	}
	return rc;
}

static void msm_dai_q6_auxpcm_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	int rc = 0;
	struct msm_dai_q6_auxpcm_dai_data *aux_dai_data =
		dev_get_drvdata(dai->dev);

	mutex_lock(&aux_dai_data->rlock);

	if (!(test_bit(STATUS_TX_PORT, aux_dai_data->auxpcm_port_status) ||
	      test_bit(STATUS_RX_PORT, aux_dai_data->auxpcm_port_status))) {
		dev_dbg(dai->dev, "%s(): dai->id %d PCM ports already closed\n",
				__func__, dai->id);
		goto exit;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (test_bit(STATUS_TX_PORT, aux_dai_data->auxpcm_port_status))
			clear_bit(STATUS_TX_PORT,
				  aux_dai_data->auxpcm_port_status);
		else {
			dev_dbg(dai->dev, "%s: PCM_TX port already closed\n",
				__func__);
			goto exit;
		}
	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (test_bit(STATUS_RX_PORT, aux_dai_data->auxpcm_port_status))
			clear_bit(STATUS_RX_PORT,
				  aux_dai_data->auxpcm_port_status);
		else {
			dev_dbg(dai->dev, "%s: PCM_RX port already closed\n",
				__func__);
			goto exit;
		}
	}
	if (test_bit(STATUS_TX_PORT, aux_dai_data->auxpcm_port_status) ||
	    test_bit(STATUS_RX_PORT, aux_dai_data->auxpcm_port_status)) {
		dev_dbg(dai->dev, "%s: cannot shutdown PCM ports\n",
			__func__);
		goto exit;
	}

	dev_dbg(dai->dev, "%s: dai->id = %d closing PCM AFE ports\n",
			__func__, dai->id);

	rc = afe_close(aux_dai_data->rx_pid); /* can block */
	if (rc < 0)
		dev_err(dai->dev, "fail to close PCM_RX  AFE port\n");

	rc = afe_close(aux_dai_data->tx_pid);
	if (rc < 0)
		dev_err(dai->dev, "fail to close AUX PCM TX port\n");

	msm_dai_q6_auxpcm_set_clk(aux_dai_data, aux_dai_data->rx_pid, false);
	msm_dai_q6_auxpcm_set_clk(aux_dai_data, aux_dai_data->tx_pid, false);
exit:
	mutex_unlock(&aux_dai_data->rlock);
}

static int msm_dai_q6_auxpcm_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_auxpcm_dai_data *aux_dai_data =
		dev_get_drvdata(dai->dev);
	struct msm_dai_q6_dai_data *dai_data = &aux_dai_data->bdai_data;
	struct msm_dai_auxpcm_pdata *auxpcm_pdata = NULL;
	int rc = 0;
	u32 pcm_clk_rate;

	auxpcm_pdata = dai->dev->platform_data;
	mutex_lock(&aux_dai_data->rlock);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (test_bit(STATUS_TX_PORT,
				aux_dai_data->auxpcm_port_status)) {
			dev_dbg(dai->dev, "%s: PCM_TX port already ON\n",
				__func__);
			goto exit;
		} else
			set_bit(STATUS_TX_PORT,
				  aux_dai_data->auxpcm_port_status);
	} else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (test_bit(STATUS_RX_PORT,
				aux_dai_data->auxpcm_port_status)) {
			dev_dbg(dai->dev, "%s: PCM_RX port already ON\n",
				__func__);
			goto exit;
		} else
			set_bit(STATUS_RX_PORT,
				  aux_dai_data->auxpcm_port_status);
	}
	if (test_bit(STATUS_TX_PORT, aux_dai_data->auxpcm_port_status) &&
	    test_bit(STATUS_RX_PORT, aux_dai_data->auxpcm_port_status)) {
		dev_dbg(dai->dev, "%s: PCM ports already set\n", __func__);
		goto exit;
	}

	dev_dbg(dai->dev, "%s: dai->id:%d  opening afe ports\n",
			__func__, dai->id);

	rc = afe_q6_interface_prepare();
	if (rc < 0) {
		dev_err(dai->dev, "fail to open AFE APR\n");
		goto fail;
	}

	/*
	 * For AUX PCM Interface the below sequence of clk
	 * settings and afe_open is a strict requirement.
	 *
	 * Also using afe_open instead of afe_port_start_nowait
	 * to make sure the port is open before deasserting the
	 * clock line. This is required because pcm register is
	 * not written before clock deassert. Hence the hw does
	 * not get updated with new setting if the below clock
	 * assert/deasset and afe_open sequence is not followed.
	 */

	if (dai_data->rate == 8000) {
		pcm_clk_rate = auxpcm_pdata->mode_8k.pcm_clk_rate;
	} else if (dai_data->rate == 16000) {
		pcm_clk_rate = (auxpcm_pdata->mode_16k.pcm_clk_rate);
	} else {
		dev_err(dai->dev, "%s: Invalid AUX PCM rate %d\n", __func__,
			dai_data->rate);
		rc = -EINVAL;
		goto fail;
	}
	if (aux_dai_data->afe_clk_ver == AFE_CLK_VERSION_V2) {
		memcpy(&aux_dai_data->clk_set, &lpass_clk_set_default,
				sizeof(struct afe_clk_set));
		aux_dai_data->clk_set.clk_freq_in_hz = pcm_clk_rate;

		switch (dai->id) {
		case MSM_DAI_PRI_AUXPCM_DT_DEV_ID:
			if (pcm_clk_rate)
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_PRI_PCM_IBIT;
			else
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_PRI_PCM_EBIT;
			break;
		case MSM_DAI_SEC_AUXPCM_DT_DEV_ID:
			if (pcm_clk_rate)
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_SEC_PCM_IBIT;
			else
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_SEC_PCM_EBIT;
			break;
		case MSM_DAI_TERT_AUXPCM_DT_DEV_ID:
			if (pcm_clk_rate)
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_TER_PCM_IBIT;
			else
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_TER_PCM_EBIT;
			break;
		case MSM_DAI_QUAT_AUXPCM_DT_DEV_ID:
			if (pcm_clk_rate)
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_QUAD_PCM_IBIT;
			else
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_QUAD_PCM_EBIT;
			break;
		case MSM_DAI_QUIN_AUXPCM_DT_DEV_ID:
			if (pcm_clk_rate)
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_QUIN_PCM_IBIT;
			else
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_QUIN_PCM_EBIT;
			break;
		case MSM_DAI_SEN_AUXPCM_DT_DEV_ID:
			if (pcm_clk_rate)
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_SEN_PCM_IBIT;
			else
				aux_dai_data->clk_set.clk_id =
					Q6AFE_LPASS_CLK_ID_SEN_PCM_EBIT;
			break;
		default:
			dev_err(dai->dev, "%s: AUXPCM id: %d not supported\n",
				__func__, dai->id);
			break;
		}
	} else {
		memcpy(&aux_dai_data->clk_cfg, &lpass_clk_cfg_default,
				sizeof(struct afe_clk_cfg));
		aux_dai_data->clk_cfg.clk_val1 = pcm_clk_rate;
	}

	rc = msm_dai_q6_auxpcm_set_clk(aux_dai_data,
				       aux_dai_data->rx_pid, true);
	if (rc < 0) {
		dev_err(dai->dev,
			"%s:afe_set_lpass_clock on RX pcm_src_clk failed\n",
			__func__);
		goto fail;
	}

	rc = msm_dai_q6_auxpcm_set_clk(aux_dai_data,
				       aux_dai_data->tx_pid, true);
	if (rc < 0) {
		dev_err(dai->dev,
			"%s:afe_set_lpass_clock on TX pcm_src_clk failed\n",
			__func__);
		goto fail;
	}

	afe_open(aux_dai_data->rx_pid, &dai_data->port_config, dai_data->rate);
	afe_open(aux_dai_data->tx_pid, &dai_data->port_config, dai_data->rate);
	goto exit;

fail:
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		clear_bit(STATUS_TX_PORT, aux_dai_data->auxpcm_port_status);
	else if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		clear_bit(STATUS_RX_PORT, aux_dai_data->auxpcm_port_status);

exit:
	mutex_unlock(&aux_dai_data->rlock);
	return rc;
}

static int msm_dai_q6_auxpcm_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	int rc = 0;

	pr_debug("%s:port:%d  cmd:%d\n",
		__func__, dai->id, cmd);

	switch (cmd) {

	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/* afe_open will be called from prepare */
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		return 0;

	default:
		pr_err("%s: cmd %d\n", __func__, cmd);
		rc = -EINVAL;
	}

	return rc;

}

static int msm_dai_q6_dai_auxpcm_remove(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_auxpcm_dai_data *aux_dai_data;
	int rc;

	aux_dai_data = dev_get_drvdata(dai->dev);

	dev_dbg(dai->dev, "%s: dai->id %d closing afe\n",
		__func__, dai->id);

	if (test_bit(STATUS_TX_PORT, aux_dai_data->auxpcm_port_status) ||
	    test_bit(STATUS_RX_PORT, aux_dai_data->auxpcm_port_status)) {
		rc = afe_close(aux_dai_data->rx_pid); /* can block */
		if (rc < 0)
			dev_err(dai->dev, "fail to close AUXPCM RX AFE port\n");
		rc = afe_close(aux_dai_data->tx_pid);
		if (rc < 0)
			dev_err(dai->dev, "fail to close AUXPCM TX AFE port\n");
		clear_bit(STATUS_TX_PORT, aux_dai_data->auxpcm_port_status);
		clear_bit(STATUS_RX_PORT, aux_dai_data->auxpcm_port_status);
	}
	msm_dai_q6_auxpcm_set_clk(aux_dai_data, aux_dai_data->rx_pid, false);
	msm_dai_q6_auxpcm_set_clk(aux_dai_data, aux_dai_data->tx_pid, false);
	return 0;
}

static int msm_dai_q6_power_mode_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	u16 port_id = (u16)kcontrol->private_value;

	pr_debug("%s: power mode = %d\n", __func__, value);

	afe_set_power_mode_cfg(port_id, value);
	return 0;
}

static int msm_dai_q6_power_mode_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int value;
	u16 port_id = (u16)kcontrol->private_value;

	afe_get_power_mode_cfg(port_id, &value);
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static void power_mode_mx_ctl_private_free(struct snd_kcontrol *kcontrol)
{
	struct snd_kcontrol_new *knew = snd_kcontrol_chip(kcontrol);
	kfree(knew);
}

static int msm_dai_q6_add_power_mode_mx_ctls(struct snd_card *card,
					 const char *dai_name,
					 int dai_id, void *dai_data)
{
	const char *mx_ctl_name = "Power Mode";
	char *mixer_str = NULL;
	int dai_str_len = 0, ctl_len = 0;
	int rc = 0;
	struct snd_kcontrol_new *knew = NULL;
	struct snd_kcontrol *kctl = NULL;

	dai_str_len = strlen(dai_name) + 1;

	ctl_len = dai_str_len + strlen(mx_ctl_name) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);
	if (!mixer_str)
		return -ENOMEM;

	snprintf(mixer_str, ctl_len, "%s %s", dai_name, mx_ctl_name);

	knew = kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!knew) {
		kfree(mixer_str);
		return -ENOMEM;
	}
	knew->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	knew->info = snd_ctl_boolean_mono_info;
	knew->get = msm_dai_q6_power_mode_get;
	knew->put = msm_dai_q6_power_mode_put;
	knew->name = mixer_str;
	knew->private_value = dai_id;
	kctl = snd_ctl_new1(knew, knew);
	if (!kctl) {
		kfree(knew);
		kfree(mixer_str);
		return -ENOMEM;
	}
	kctl->private_free = power_mode_mx_ctl_private_free;
	rc = snd_ctl_add(card, kctl);
	if (rc < 0)
		pr_err("%s: err add config ctl, DAI = %s\n",
			__func__, dai_name);
	kfree(mixer_str);

	return rc;
}

static int msm_dai_q6_island_mode_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	u16 port_id = (u16)kcontrol->private_value;

	pr_debug("%s: island mode = %d\n", __func__, value);

	afe_set_island_mode_cfg(port_id, value);
	return 0;
}

static int msm_dai_q6_island_mode_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int value;
	u16 port_id = (u16)kcontrol->private_value;

	afe_get_island_mode_cfg(port_id, &value);
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static void island_mx_ctl_private_free(struct snd_kcontrol *kcontrol)
{
	struct snd_kcontrol_new *knew = snd_kcontrol_chip(kcontrol);

	kfree(knew);
}

static int msm_dai_q6_add_island_mx_ctls(struct snd_card *card,
				      const char *dai_name,
				      int dai_id, void *dai_data)
{
	const char *mx_ctl_name = "TX island";
	char *mixer_str = NULL;
	int dai_str_len = 0, ctl_len = 0;
	int rc = 0;
	struct snd_kcontrol_new *knew = NULL;
	struct snd_kcontrol *kctl = NULL;

	dai_str_len = strlen(dai_name) + 1;

	/* Add island related mixer controls */
	ctl_len = dai_str_len + strlen(mx_ctl_name) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);
	if (!mixer_str)
		return -ENOMEM;

	snprintf(mixer_str, ctl_len, "%s %s", dai_name, mx_ctl_name);

	knew = kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!knew) {
		kfree(mixer_str);
		return -ENOMEM;
	}
	knew->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	knew->info = snd_ctl_boolean_mono_info;
	knew->get = msm_dai_q6_island_mode_get;
	knew->put = msm_dai_q6_island_mode_put;
	knew->name = mixer_str;
	knew->private_value = dai_id;
	kctl = snd_ctl_new1(knew, knew);
	if (!kctl) {
		kfree(knew);
		kfree(mixer_str);
		return -ENOMEM;
	}
	kctl->private_free = island_mx_ctl_private_free;
	rc = snd_ctl_add(card, kctl);
	if (rc < 0)
		pr_err("%s: err add config ctl, DAI = %s\n",
			__func__, dai_name);
	kfree(mixer_str);

	return rc;
}

static int msm_dai_q6_add_isconfig_config_mx_ctls(struct snd_card *card,
						const char *dai_name,
						int dai_id, void *dai_data)

{
	const char *mx_ctl_name = "Island Config";
	char *mixer_str = NULL;
	int dai_str_len = 0, ctl_len = 0;
	int rc = 0;
	struct snd_kcontrol_new *knew = NULL;
	struct snd_kcontrol *kctl = NULL;

	dai_str_len = strlen(dai_name) + 1;

	ctl_len = dai_str_len + strlen(mx_ctl_name) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);
	if (!mixer_str)
		return -ENOMEM;

	snprintf(mixer_str, ctl_len, "%s %s", dai_name, mx_ctl_name);

	knew = kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!knew) {
		kfree(mixer_str);
		return -ENOMEM;
	}
	knew->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	knew->info = snd_ctl_boolean_mono_info;
	knew->get = msm_dai_q6_island_mode_get;
	knew->put = msm_dai_q6_island_mode_put;
	knew->name = mixer_str;
	knew->private_value = dai_id;
	kctl = snd_ctl_new1(knew, knew);
	if (!kctl) {
		kfree(knew);
		kfree(mixer_str);
		return -ENOMEM;
	}
	kctl->private_free = island_mx_ctl_private_free;
	rc = snd_ctl_add(card, kctl);
	if (rc < 0)
		pr_err("%s: err add config ctl, DAI = %s\n",
			__func__, dai_name);
	kfree(mixer_str);

	return rc;
}

/*
 * For single CPU DAI registration, the dai id needs to be
 * set explicitly in the dai probe as ASoC does not read
 * the cpu->driver->id field rather it assigns the dai id
 * from the device name that is in the form %s.%d. This dai
 * id should be assigned to back-end AFE port id and used
 * during dai prepare. For multiple dai registration, it
 * is not required to call this function, however the dai->
 * driver->id field must be defined and set to corresponding
 * AFE Port id.
 */
static inline void msm_dai_q6_set_dai_id(struct snd_soc_dai *dai)
{
	if (!dai->driver) {
		dev_err(dai->dev, "DAI driver is not set\n");
		return;
	}
	if (!dai->driver->id) {
		dev_dbg(dai->dev, "DAI driver id is not set\n");
		return;
	}
	dai->id = dai->driver->id;
}

static int msm_dai_q6_aux_pcm_probe(struct snd_soc_dai *dai)
{
	int rc = 0;
	struct msm_dai_q6_auxpcm_dai_data *dai_data = NULL;

	if (!dai) {
		pr_err("%s: Invalid params dai\n", __func__);
		return -EINVAL;
	}
	if (!dai->dev) {
		pr_err("%s: Invalid params dai dev\n", __func__);
		return -EINVAL;
	}

	msm_dai_q6_set_dai_id(dai);
	dai_data = dev_get_drvdata(dai->dev);

	if (dai_data->is_island_dai)
		rc = msm_dai_q6_add_island_mx_ctls(
						dai->component->card->snd_card,
						dai->name, dai_data->tx_pid,
						(void *)dai_data);

	rc = msm_dai_q6_dai_add_route(dai);
	return rc;
}

static struct snd_soc_dai_ops msm_dai_q6_auxpcm_ops = {
	.prepare	= msm_dai_q6_auxpcm_prepare,
	.trigger	= msm_dai_q6_auxpcm_trigger,
	.hw_params	= msm_dai_q6_auxpcm_hw_params,
	.shutdown	= msm_dai_q6_auxpcm_shutdown,
};

static const struct snd_soc_component_driver
	msm_dai_q6_aux_pcm_dai_component = {
	.name		= "msm-auxpcm-dev",
};

static struct snd_soc_dai_driver msm_dai_q6_aux_pcm_dai[] = {
	{
		.playback = {
			.stream_name = "AUX PCM Playback",
			.aif_name = "AUX_PCM_RX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.capture = {
			.stream_name = "AUX PCM Capture",
			.aif_name = "AUX_PCM_TX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.id = MSM_DAI_PRI_AUXPCM_DT_DEV_ID,
		.name = "Pri AUX PCM",
		.ops = &msm_dai_q6_auxpcm_ops,
		.probe = msm_dai_q6_aux_pcm_probe,
		.remove = msm_dai_q6_dai_auxpcm_remove,
	},
	{
		.playback = {
			.stream_name = "Sec AUX PCM Playback",
			.aif_name = "SEC_AUX_PCM_RX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.capture = {
			.stream_name = "Sec AUX PCM Capture",
			.aif_name = "SEC_AUX_PCM_TX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.id = MSM_DAI_SEC_AUXPCM_DT_DEV_ID,
		.name = "Sec AUX PCM",
		.ops = &msm_dai_q6_auxpcm_ops,
		.probe = msm_dai_q6_aux_pcm_probe,
		.remove = msm_dai_q6_dai_auxpcm_remove,
	},
	{
		.playback = {
			.stream_name = "Tert AUX PCM Playback",
			.aif_name = "TERT_AUX_PCM_RX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.capture = {
			.stream_name = "Tert AUX PCM Capture",
			.aif_name = "TERT_AUX_PCM_TX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.id = MSM_DAI_TERT_AUXPCM_DT_DEV_ID,
		.name = "Tert AUX PCM",
		.ops = &msm_dai_q6_auxpcm_ops,
		.probe = msm_dai_q6_aux_pcm_probe,
		.remove = msm_dai_q6_dai_auxpcm_remove,
	},
	{
		.playback = {
			.stream_name = "Quat AUX PCM Playback",
			.aif_name = "QUAT_AUX_PCM_RX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.capture = {
			.stream_name = "Quat AUX PCM Capture",
			.aif_name = "QUAT_AUX_PCM_TX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.id = MSM_DAI_QUAT_AUXPCM_DT_DEV_ID,
		.name = "Quat AUX PCM",
		.ops = &msm_dai_q6_auxpcm_ops,
		.probe = msm_dai_q6_aux_pcm_probe,
		.remove = msm_dai_q6_dai_auxpcm_remove,
	},
	{
		.playback = {
			.stream_name = "Quin AUX PCM Playback",
			.aif_name = "QUIN_AUX_PCM_RX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.capture = {
			.stream_name = "Quin AUX PCM Capture",
			.aif_name = "QUIN_AUX_PCM_TX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.id = MSM_DAI_QUIN_AUXPCM_DT_DEV_ID,
		.name = "Quin AUX PCM",
		.ops = &msm_dai_q6_auxpcm_ops,
		.probe = msm_dai_q6_aux_pcm_probe,
		.remove = msm_dai_q6_dai_auxpcm_remove,
	},
	{
		.playback = {
			.stream_name = "Sen AUX PCM Playback",
			.aif_name = "SEN_AUX_PCM_RX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.capture = {
			.stream_name = "Sen AUX PCM Capture",
			.aif_name = "SEN_AUX_PCM_TX",
			.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_max = 16000,
			.rate_min = 8000,
		},
		.id = MSM_DAI_SEN_AUXPCM_DT_DEV_ID,
		.name = "Sen AUX PCM",
		.ops = &msm_dai_q6_auxpcm_ops,
		.probe = msm_dai_q6_aux_pcm_probe,
		.remove = msm_dai_q6_dai_auxpcm_remove,
	},
};

static int msm_dai_q6_spdif_format_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{

	struct msm_dai_q6_spdif_dai_data *dai_data = kcontrol->private_data;
	int value = ucontrol->value.integer.value[0];

	dai_data->spdif_port.cfg.data_format = value;
	pr_debug("%s: value = %d\n", __func__, value);
	return 0;
}

static int msm_dai_q6_spdif_format_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{

	struct msm_dai_q6_spdif_dai_data *dai_data = kcontrol->private_data;

	ucontrol->value.integer.value[0] =
		dai_data->spdif_port.cfg.data_format;
	return 0;
}

static int msm_dai_q6_spdif_source_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{

	struct msm_dai_q6_spdif_dai_data *dai_data = kcontrol->private_data;
	int value = ucontrol->value.integer.value[0];

	dai_data->spdif_port.cfg.src_sel = value;
	pr_debug("%s: value = %d\n", __func__, value);
	return 0;
}

static int msm_dai_q6_spdif_source_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{

	struct msm_dai_q6_spdif_dai_data *dai_data = kcontrol->private_data;

	ucontrol->value.integer.value[0] =
		dai_data->spdif_port.cfg.src_sel;
	return 0;
}

static const char * const spdif_format[] = {
	"LPCM",
	"Compr"
};

static const char * const spdif_source[] = {
	"Optical", "EXT-ARC", "Coaxial", "VT-ARC"
};

static const struct soc_enum spdif_rx_config_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spdif_format), spdif_format),
};

static const struct soc_enum spdif_tx_config_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spdif_source), spdif_source),
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(spdif_format), spdif_format),
};

static int msm_dai_q6_spdif_chstatus_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_spdif_dai_data *dai_data = kcontrol->private_data;
	int ret = 0;

	dai_data->spdif_port.ch_status.status_type =
		AFE_API_VERSION_SPDIF_CH_STATUS_CONFIG;
	memset(dai_data->spdif_port.ch_status.status_mask,
			CHANNEL_STATUS_MASK_INIT, CHANNEL_STATUS_SIZE);
	dai_data->spdif_port.ch_status.status_mask[0] =
		CHANNEL_STATUS_MASK;

	memcpy(dai_data->spdif_port.ch_status.status_bits,
			ucontrol->value.iec958.status, CHANNEL_STATUS_SIZE);

	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		pr_debug("%s: Port already started. Dynamic update\n",
				__func__);
		ret = afe_send_spdif_ch_status_cfg(
				&dai_data->spdif_port.ch_status,
				dai_data->port_id);
	}
	return ret;
}

static int msm_dai_q6_spdif_chstatus_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{

	struct msm_dai_q6_spdif_dai_data *dai_data = kcontrol->private_data;

	memcpy(ucontrol->value.iec958.status,
			dai_data->spdif_port.ch_status.status_bits,
			CHANNEL_STATUS_SIZE);
	return 0;
}

static int msm_dai_q6_spdif_chstatus_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	return 0;
}

static const struct snd_kcontrol_new spdif_rx_config_controls[] = {
	/* Primary SPDIF output */
	{
		.access = (SNDRV_CTL_ELEM_ACCESS_READWRITE |
				SNDRV_CTL_ELEM_ACCESS_INACTIVE),
		.iface  =   SNDRV_CTL_ELEM_IFACE_PCM,
		.name   =   SNDRV_CTL_NAME_IEC958("", PLAYBACK, PCM_STREAM),
		.info   =   msm_dai_q6_spdif_chstatus_info,
		.get    =   msm_dai_q6_spdif_chstatus_get,
		.put    =   msm_dai_q6_spdif_chstatus_put,
	},
	SOC_ENUM_EXT("PRI SPDIF RX Format", spdif_rx_config_enum[0],
			msm_dai_q6_spdif_format_get,
			msm_dai_q6_spdif_format_put),
	/* Secondary SPDIF output */
	{
		.access = (SNDRV_CTL_ELEM_ACCESS_READWRITE |
				SNDRV_CTL_ELEM_ACCESS_INACTIVE),
		.iface  =   SNDRV_CTL_ELEM_IFACE_PCM,
		.name   =   SNDRV_CTL_NAME_IEC958("SEC", PLAYBACK, PCM_STREAM),
		.info   =   msm_dai_q6_spdif_chstatus_info,
		.get    =   msm_dai_q6_spdif_chstatus_get,
		.put    =   msm_dai_q6_spdif_chstatus_put,
	},
	SOC_ENUM_EXT("SEC SPDIF RX Format", spdif_rx_config_enum[0],
			msm_dai_q6_spdif_format_get,
			msm_dai_q6_spdif_format_put)
};

static const struct snd_kcontrol_new spdif_tx_config_controls[] = {
	SOC_ENUM_EXT("PRI SPDIF TX Source", spdif_tx_config_enum[0],
			msm_dai_q6_spdif_source_get,
			msm_dai_q6_spdif_source_put),
	SOC_ENUM_EXT("PRI SPDIF TX Format", spdif_tx_config_enum[1],
			msm_dai_q6_spdif_format_get,
			msm_dai_q6_spdif_format_put),
	SOC_ENUM_EXT("SEC SPDIF TX Source", spdif_tx_config_enum[0],
			msm_dai_q6_spdif_source_get,
			msm_dai_q6_spdif_source_put),
	SOC_ENUM_EXT("SEC SPDIF TX Format", spdif_tx_config_enum[1],
			msm_dai_q6_spdif_format_get,
			msm_dai_q6_spdif_format_put)
};

static void msm_dai_q6_spdif_process_event(uint32_t opcode, uint32_t token,
		uint32_t *payload, void *private_data)
{
	struct msm_dai_q6_spdif_event_msg *evt;
	struct msm_dai_q6_spdif_dai_data *dai_data;
	int preemph_old = 0;
	int preemph_new = 0;

	evt = (struct msm_dai_q6_spdif_event_msg *)payload;
	dai_data = (struct msm_dai_q6_spdif_dai_data *)private_data;

	preemph_old = GET_PREEMPH(dai_data->fmt_event.channel_status[0]);
	preemph_new = GET_PREEMPH(evt->fmt_event.channel_status[0]);

	pr_debug("%s: old state %d, fmt %d, rate %d, preemph %d\n",
			__func__, dai_data->fmt_event.status,
			dai_data->fmt_event.data_format,
			dai_data->fmt_event.sample_rate,
			preemph_old);
	pr_debug("%s: new state %d, fmt %d, rate %d, preemph %d\n",
			__func__, evt->fmt_event.status,
			evt->fmt_event.data_format,
			evt->fmt_event.sample_rate,
			preemph_new);

	dai_data->fmt_event.status = evt->fmt_event.status;
	dai_data->fmt_event.data_format = evt->fmt_event.data_format;
	dai_data->fmt_event.sample_rate = evt->fmt_event.sample_rate;
	dai_data->fmt_event.channel_status[0] =
		evt->fmt_event.channel_status[0];
	dai_data->fmt_event.channel_status[1] =
		evt->fmt_event.channel_status[1];
	dai_data->fmt_event.channel_status[2] =
		evt->fmt_event.channel_status[2];
	dai_data->fmt_event.channel_status[3] =
		evt->fmt_event.channel_status[3];
	dai_data->fmt_event.channel_status[4] =
		evt->fmt_event.channel_status[4];
	dai_data->fmt_event.channel_status[5] =
		evt->fmt_event.channel_status[5];
}

static int msm_dai_q6_spdif_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_spdif_dai_data *dai_data = dev_get_drvdata(dai->dev);

	dai_data->channels = params_channels(params);
	dai_data->spdif_port.cfg.num_channels = dai_data->channels;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dai_data->spdif_port.cfg.bit_width = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		dai_data->spdif_port.cfg.bit_width = 24;
		break;
	default:
		pr_err("%s: format %d\n",
			__func__, params_format(params));
		return -EINVAL;
	}

	dai_data->rate = params_rate(params);
	dai_data->bitwidth = dai_data->spdif_port.cfg.bit_width;
	dai_data->spdif_port.cfg.sample_rate = dai_data->rate;
	dai_data->spdif_port.cfg.spdif_cfg_minor_version =
		AFE_API_VERSION_SPDIF_CONFIG_V2;
	dev_dbg(dai->dev, " channel %d sample rate %d bit width %d\n",
			dai_data->channels, dai_data->rate,
			dai_data->spdif_port.cfg.bit_width);
	dai_data->spdif_port.cfg.reserved = 0;
	return 0;
}

static void msm_dai_q6_spdif_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_spdif_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int rc = 0;

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		pr_info("%s:  afe port not started. dai_data->status_mask = %ld\n",
				__func__, *dai_data->status_mask);
		return;
	}

	rc = afe_close(dai->id);
	if (rc < 0)
		dev_err(dai->dev, "fail to close AFE port\n");

	dai_data->fmt_event.status = 0; /* report invalid line state */

	pr_debug("%s: dai_data->status_mask = %ld\n", __func__,
			*dai_data->status_mask);

	clear_bit(STATUS_PORT_STARTED, dai_data->status_mask);
}

static int msm_dai_q6_spdif_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_spdif_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int rc = 0;

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		rc = afe_spdif_reg_event_cfg(dai->id,
				AFE_MODULE_REGISTER_EVENT_FLAG,
				msm_dai_q6_spdif_process_event,
				dai_data);
		if (rc < 0)
			dev_err(dai->dev,
				"fail to register event for port 0x%x\n",
				dai->id);

		rc = afe_spdif_port_start(dai->id, &dai_data->spdif_port,
				dai_data->rate);
		if (rc < 0)
			dev_err(dai->dev, "fail to open AFE port 0x%x\n",
					dai->id);
		else
			set_bit(STATUS_PORT_STARTED,
					dai_data->status_mask);
	}

	return rc;
}

static ssize_t msm_dai_q6_spdif_sysfs_rda_audio_state(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct msm_dai_q6_spdif_dai_data *dai_data = dev_get_drvdata(dev);

	if (!dai_data) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	ret = snprintf(buf, MSM_DAI_SYSFS_ENTRY_MAX_LEN, "%d\n",
		dai_data->fmt_event.status);
	pr_debug("%s: '%d'\n", __func__, dai_data->fmt_event.status);

	return ret;
}

static ssize_t msm_dai_q6_spdif_sysfs_rda_audio_format(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct msm_dai_q6_spdif_dai_data *dai_data = dev_get_drvdata(dev);

	if (!dai_data) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	ret = snprintf(buf, MSM_DAI_SYSFS_ENTRY_MAX_LEN, "%d\n",
		dai_data->fmt_event.data_format);
	pr_debug("%s: '%d'\n", __func__, dai_data->fmt_event.data_format);

	return ret;
}

static ssize_t msm_dai_q6_spdif_sysfs_rda_audio_rate(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct msm_dai_q6_spdif_dai_data *dai_data = dev_get_drvdata(dev);

	if (!dai_data) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	ret = snprintf(buf, MSM_DAI_SYSFS_ENTRY_MAX_LEN, "%d\n",
		dai_data->fmt_event.sample_rate);
	pr_debug("%s: '%d'\n", __func__, dai_data->fmt_event.sample_rate);

	return ret;
}

static ssize_t msm_dai_q6_spdif_sysfs_rda_audio_preemph(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct msm_dai_q6_spdif_dai_data *dai_data = dev_get_drvdata(dev);
	int preemph = 0;

	if (!dai_data) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	preemph = GET_PREEMPH(dai_data->fmt_event.channel_status[0]);

	ret = snprintf(buf, MSM_DAI_SYSFS_ENTRY_MAX_LEN, "%d\n", preemph);
	pr_debug("%s: '%d'\n", __func__, preemph);

	return ret;
}

static DEVICE_ATTR(audio_state, 0444, msm_dai_q6_spdif_sysfs_rda_audio_state,
	NULL);
static DEVICE_ATTR(audio_format, 0444, msm_dai_q6_spdif_sysfs_rda_audio_format,
	NULL);
static DEVICE_ATTR(audio_rate, 0444, msm_dai_q6_spdif_sysfs_rda_audio_rate,
	NULL);
static DEVICE_ATTR(audio_preemph, 0444,
	msm_dai_q6_spdif_sysfs_rda_audio_preemph, NULL);

static struct attribute *msm_dai_q6_spdif_fs_attrs[] = {
	&dev_attr_audio_state.attr,
	&dev_attr_audio_format.attr,
	&dev_attr_audio_rate.attr,
	&dev_attr_audio_preemph.attr,
	NULL,
};
static struct attribute_group msm_dai_q6_spdif_fs_attrs_group = {
	.attrs = msm_dai_q6_spdif_fs_attrs,
};

static int msm_dai_q6_spdif_sysfs_create(struct snd_soc_dai *dai,
	struct msm_dai_q6_spdif_dai_data *dai_data)
{
	int rc;

	rc = sysfs_create_group(&dai->dev->kobj,
		&msm_dai_q6_spdif_fs_attrs_group);
	if (rc) {
		pr_err("%s: failed, rc=%d\n", __func__, rc);
		return rc;
	}
	dai_data->kobj = &dai->dev->kobj;

	return 0;
}

static void msm_dai_q6_spdif_sysfs_remove(struct snd_soc_dai *dai,
	struct msm_dai_q6_spdif_dai_data *dai_data)
{
	if (dai_data->kobj)
		sysfs_remove_group(dai_data->kobj,
			&msm_dai_q6_spdif_fs_attrs_group);
	dai_data->kobj = NULL;
}

static int msm_dai_q6_spdif_dai_probe(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_spdif_dai_data *dai_data;
	int rc = 0;
	struct snd_soc_dapm_route intercon;
	struct snd_soc_dapm_context *dapm;

	if (!dai) {
		pr_err("%s: dai not found!!\n", __func__);
		return -EINVAL;
	}
	if (!dai->dev) {
		pr_err("%s: Invalid params dai dev\n", __func__);
		return -EINVAL;
	}

	dai_data = kzalloc(sizeof(struct msm_dai_q6_spdif_dai_data),
			GFP_KERNEL);

	if (!dai_data)
		return -ENOMEM;
	else
		dev_set_drvdata(dai->dev, dai_data);

	msm_dai_q6_set_dai_id(dai);
	dai_data->port_id = dai->id;

	switch (dai->id) {
	case AFE_PORT_ID_PRIMARY_SPDIF_RX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&spdif_rx_config_controls[1],
				 dai_data));
		break;
	case AFE_PORT_ID_SECONDARY_SPDIF_RX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&spdif_rx_config_controls[3],
				 dai_data));
		break;
	case AFE_PORT_ID_PRIMARY_SPDIF_TX:
		rc = msm_dai_q6_spdif_sysfs_create(dai, dai_data);

		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(&spdif_tx_config_controls[0],
				dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(&spdif_tx_config_controls[1],
				dai_data));
		break;
	case AFE_PORT_ID_SECONDARY_SPDIF_TX:
		rc = msm_dai_q6_spdif_sysfs_create(dai, dai_data);

		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(&spdif_tx_config_controls[2],
				dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(&spdif_tx_config_controls[3],
				dai_data));
		break;
	}
	if (rc < 0)
		dev_err(dai->dev,
			"%s: err add config ctl, DAI = %s\n",
			__func__, dai->name);

	dapm = snd_soc_component_get_dapm(dai->component);

	memset(&intercon, 0, sizeof(intercon));
	if (!rc && dai && dai->driver) {
		if (dai->driver->playback.stream_name &&
				dai->driver->playback.aif_name) {
			dev_dbg(dai->dev, "%s: add route for widget %s",
				__func__, dai->driver->playback.stream_name);
			intercon.source = dai->driver->playback.aif_name;
			intercon.sink = dai->driver->playback.stream_name;
			dev_dbg(dai->dev, "%s: src %s sink %s\n",
				__func__, intercon.source, intercon.sink);
			snd_soc_dapm_add_routes(dapm, &intercon, 1);
		}
		if (dai->driver->capture.stream_name &&
				dai->driver->capture.aif_name) {
			dev_dbg(dai->dev, "%s: add route for widget %s",
				__func__, dai->driver->capture.stream_name);
			intercon.sink = dai->driver->capture.aif_name;
			intercon.source = dai->driver->capture.stream_name;
			dev_dbg(dai->dev, "%s: src %s sink %s\n",
				__func__, intercon.source, intercon.sink);
			snd_soc_dapm_add_routes(dapm, &intercon, 1);
		}
	}
	return rc;
}

static int msm_dai_q6_spdif_dai_remove(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_spdif_dai_data *dai_data;
	int rc;

	dai_data = dev_get_drvdata(dai->dev);

	/* If AFE port is still up, close it */
	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		rc = afe_spdif_reg_event_cfg(dai->id,
				AFE_MODULE_DEREGISTER_EVENT_FLAG,
				NULL,
				dai_data);
		if (rc < 0)
			dev_err(dai->dev,
				"fail to deregister event for port 0x%x\n",
				dai->id);

		rc = afe_close(dai->id); /* can block */
		if (rc < 0)
			dev_err(dai->dev, "fail to close AFE port\n");

		clear_bit(STATUS_PORT_STARTED, dai_data->status_mask);
	}

	msm_dai_q6_spdif_sysfs_remove(dai, dai_data);

	kfree(dai_data);

	return 0;
}


static struct snd_soc_dai_ops msm_dai_q6_spdif_ops = {
	.prepare	= msm_dai_q6_spdif_prepare,
	.hw_params	= msm_dai_q6_spdif_hw_params,
	.shutdown	= msm_dai_q6_spdif_shutdown,
};

static struct snd_soc_dai_driver msm_dai_q6_spdif_spdif_rx_dai[] = {
	{
		.playback = {
			.stream_name = "Primary SPDIF Playback",
			.aif_name = "PRI_SPDIF_RX",
			.rates = SNDRV_PCM_RATE_32000 |
				 SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 |
				 SNDRV_PCM_RATE_88200 |
				 SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_176400 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 32000,
			.rate_max = 192000,
		},
		.name = "PRI_SPDIF_RX",
		.ops = &msm_dai_q6_spdif_ops,
		.id = AFE_PORT_ID_PRIMARY_SPDIF_RX,
		.probe = msm_dai_q6_spdif_dai_probe,
		.remove = msm_dai_q6_spdif_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary SPDIF Playback",
			.aif_name = "SEC_SPDIF_RX",
			.rates = SNDRV_PCM_RATE_32000 |
				 SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 |
				 SNDRV_PCM_RATE_88200 |
				 SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_176400 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 32000,
			.rate_max = 192000,
		},
		.name = "SEC_SPDIF_RX",
		.ops = &msm_dai_q6_spdif_ops,
		.id = AFE_PORT_ID_SECONDARY_SPDIF_RX,
		.probe = msm_dai_q6_spdif_dai_probe,
		.remove = msm_dai_q6_spdif_dai_remove,
	},
};

static struct snd_soc_dai_driver msm_dai_q6_spdif_spdif_tx_dai[] = {
	{
		.capture = {
			.stream_name = "Primary SPDIF Capture",
			.aif_name = "PRI_SPDIF_TX",
			.rates = SNDRV_PCM_RATE_32000 |
				 SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 |
				 SNDRV_PCM_RATE_88200 |
				 SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_176400 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 32000,
			.rate_max = 192000,
		},
		.name = "PRI_SPDIF_TX",
		.ops = &msm_dai_q6_spdif_ops,
		.id = AFE_PORT_ID_PRIMARY_SPDIF_TX,
		.probe = msm_dai_q6_spdif_dai_probe,
		.remove = msm_dai_q6_spdif_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Secondary SPDIF Capture",
			.aif_name = "SEC_SPDIF_TX",
			.rates = SNDRV_PCM_RATE_32000 |
				 SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 |
				 SNDRV_PCM_RATE_88200 |
				 SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_176400 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 32000,
			.rate_max = 192000,
		},
		.name = "SEC_SPDIF_TX",
		.ops = &msm_dai_q6_spdif_ops,
		.id = AFE_PORT_ID_SECONDARY_SPDIF_TX,
		.probe = msm_dai_q6_spdif_dai_probe,
		.remove = msm_dai_q6_spdif_dai_remove,
	},
};

static const struct snd_soc_component_driver msm_dai_spdif_q6_component = {
	.name		= "msm-dai-q6-spdif",
};

static int msm_dai_q6_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int rc = 0;
	uint16_t ttp_gen_enable = dai_data->ttp_config.ttp_gen_enable.enable;

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		if (dai_data->enc_config.format != ENC_FMT_NONE) {
			int bitwidth = 0;

			switch (dai_data->afe_rx_in_bitformat) {
			case SNDRV_PCM_FORMAT_S32_LE:
				bitwidth = 32;
				break;
			case SNDRV_PCM_FORMAT_S24_LE:
				bitwidth = 24;
				break;
			case SNDRV_PCM_FORMAT_S16_LE:
			default:
				bitwidth = 16;
				break;
			}
			pr_debug("%s: calling AFE_PORT_START_V2 with enc_format: %d\n",
				 __func__, dai_data->enc_config.format);
			rc = afe_port_start_v2(dai->id, &dai_data->port_config,
					       dai_data->rate,
					       dai_data->afe_rx_in_channels,
					       bitwidth,
					       &dai_data->enc_config, NULL);
			if (rc < 0)
				pr_err("%s: afe_port_start_v2 failed error: %d\n",
					__func__, rc);
		} else if (dai_data->dec_config.format != DEC_FMT_NONE) {
			int bitwidth = 0;

			/*
			 * If bitwidth is not configured set default value to
			 * zero, so that decoder port config uses slim device
			 * bit width value in afe decoder config.
			 */
			switch (dai_data->afe_tx_out_bitformat) {
			case SNDRV_PCM_FORMAT_S32_LE:
				bitwidth = 32;
				break;
			case SNDRV_PCM_FORMAT_S24_LE:
				bitwidth = 24;
				break;
			case SNDRV_PCM_FORMAT_S16_LE:
				bitwidth = 16;
				break;
			default:
				bitwidth = 0;
				break;
			}

			if (ttp_gen_enable == true) {
				pr_debug("%s: calling AFE_PORT_START_V3 with dec format: %d\n",
					 __func__, dai_data->dec_config.format);
				rc = afe_port_start_v3(dai->id,
						&dai_data->port_config,
						dai_data->rate,
						dai_data->afe_tx_out_channels,
						bitwidth,
						NULL, &dai_data->dec_config,
						&dai_data->ttp_config);
			} else {
				pr_debug("%s: calling AFE_PORT_START_V2 with dec format: %d\n",
					 __func__, dai_data->dec_config.format);
				rc = afe_port_start_v2(dai->id,
						&dai_data->port_config,
						dai_data->rate,
						dai_data->afe_tx_out_channels,
						bitwidth,
						NULL, &dai_data->dec_config);
			}
			if (rc < 0) {
				pr_err("%s: fail to open AFE port 0x%x\n",
					__func__, dai->id);
			}
		} else {
			rc = afe_port_start(dai->id, &dai_data->port_config,
						dai_data->rate);
		}
		if (rc < 0)
			dev_err(dai->dev, "fail to open AFE port 0x%x\n",
				dai->id);
		else
			set_bit(STATUS_PORT_STARTED,
				dai_data->status_mask);
	}
	return rc;
}

static int msm_dai_q6_cdc_hw_params(struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai, int stream)
{
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);

	dai_data->channels = params_channels(params);
	switch (dai_data->channels) {
	case 2:
		dai_data->port_config.i2s.mono_stereo = MSM_AFE_STEREO;
		break;
	case 1:
		dai_data->port_config.i2s.mono_stereo = MSM_AFE_MONO;
		break;
	default:
		return -EINVAL;
		pr_err("%s: err channels %d\n",
			__func__, dai_data->channels);
		break;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_SPECIAL:
		dai_data->port_config.i2s.bit_width = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		dai_data->port_config.i2s.bit_width = 24;
		break;
	default:
		pr_err("%s: format %d\n",
			__func__, params_format(params));
		return -EINVAL;
	}

	dai_data->rate = params_rate(params);
	dai_data->port_config.i2s.sample_rate = dai_data->rate;
	dai_data->port_config.i2s.i2s_cfg_minor_version =
						AFE_API_VERSION_I2S_CONFIG;
	dai_data->port_config.i2s.data_format =  AFE_LINEAR_PCM_DATA;
	dev_dbg(dai->dev, " channel %d sample rate %d entered\n",
	dai_data->channels, dai_data->rate);

	dai_data->port_config.i2s.channel_mode = 1;
	return 0;
}

u16 num_of_bits_set(u16 sd_line_mask)
{
	u8 num_bits_set = 0;

	while (sd_line_mask) {
		num_bits_set++;
		sd_line_mask = sd_line_mask & (sd_line_mask - 1);
	}
	return num_bits_set;
}
EXPORT_SYMBOL(num_of_bits_set);

static int msm_dai_q6_i2s_hw_params(struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai, int stream)
{
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);
	struct msm_i2s_data *i2s_pdata =
			(struct msm_i2s_data *) dai->dev->platform_data;

	dai_data->channels = params_channels(params);
	if (num_of_bits_set(i2s_pdata->sd_lines) == 1) {
		switch (dai_data->channels) {
		case 2:
			dai_data->port_config.i2s.mono_stereo = MSM_AFE_STEREO;
			break;
		case 1:
			dai_data->port_config.i2s.mono_stereo = MSM_AFE_MONO;
			break;
		default:
			pr_warn("%s: greater than stereo has not been validated %d",
				__func__, dai_data->channels);
			break;
		}
	}
	dai_data->rate = params_rate(params);
	dai_data->port_config.i2s.sample_rate = dai_data->rate;
	dai_data->port_config.i2s.i2s_cfg_minor_version =
						AFE_API_VERSION_I2S_CONFIG;
	dai_data->port_config.i2s.data_format =  AFE_LINEAR_PCM_DATA;
	/* Q6 only supports 16 as now */
	dai_data->port_config.i2s.bit_width = 16;
	dai_data->port_config.i2s.channel_mode = 1;

	return 0;
}

static int msm_dai_q6_slim_bus_hw_params(struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai, int stream)
{
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);

	dai_data->channels = params_channels(params);
	dai_data->rate = params_rate(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_SPECIAL:
		dai_data->port_config.slim_sch.bit_width = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		dai_data->port_config.slim_sch.bit_width = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		dai_data->port_config.slim_sch.bit_width = 32;
		break;
	default:
		pr_err("%s: format %d\n",
			__func__, params_format(params));
		return -EINVAL;
	}

	dai_data->port_config.slim_sch.sb_cfg_minor_version =
				AFE_API_VERSION_SLIMBUS_CONFIG;
	dai_data->port_config.slim_sch.sample_rate = dai_data->rate;
	dai_data->port_config.slim_sch.num_channels = dai_data->channels;

	dev_dbg(dai->dev, "%s:slimbus_dev_id[%hu] bit_wd[%hu] format[%hu]\n"
		"num_channel %hu  shared_ch_mapping[0]  %hu\n"
		"slave_port_mapping[1]  %hu slave_port_mapping[2]  %hu\n"
		"sample_rate %d\n", __func__,
		dai_data->port_config.slim_sch.slimbus_dev_id,
		dai_data->port_config.slim_sch.bit_width,
		dai_data->port_config.slim_sch.data_format,
		dai_data->port_config.slim_sch.num_channels,
		dai_data->port_config.slim_sch.shared_ch_mapping[0],
		dai_data->port_config.slim_sch.shared_ch_mapping[1],
		dai_data->port_config.slim_sch.shared_ch_mapping[2],
		dai_data->rate);

	return 0;
}

static int msm_dai_q6_usb_audio_hw_params(struct snd_pcm_hw_params *params,
					  struct snd_soc_dai *dai, int stream)
{
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);

	dai_data->channels = params_channels(params);
	dai_data->rate = params_rate(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_SPECIAL:
		dai_data->port_config.usb_audio.bit_width = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		dai_data->port_config.usb_audio.bit_width = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		dai_data->port_config.usb_audio.bit_width = 32;
		break;

	default:
		dev_err(dai->dev, "%s: invalid format %d\n",
			__func__, params_format(params));
		return -EINVAL;
	}
	dai_data->port_config.usb_audio.cfg_minor_version =
					AFE_API_MINOR_VERSION_USB_AUDIO_CONFIG;
	dai_data->port_config.usb_audio.num_channels = dai_data->channels;
	dai_data->port_config.usb_audio.sample_rate = dai_data->rate;

	dev_dbg(dai->dev, "%s: dev_id[0x%x] bit_wd[%hu] format[%hu]\n"
		"num_channel %hu  sample_rate %d\n", __func__,
		dai_data->port_config.usb_audio.dev_token,
		dai_data->port_config.usb_audio.bit_width,
		dai_data->port_config.usb_audio.data_format,
		dai_data->port_config.usb_audio.num_channels,
		dai_data->port_config.usb_audio.sample_rate);

	return 0;
}

static int msm_dai_q6_bt_fm_hw_params(struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai, int stream)
{
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);

	dai_data->channels = params_channels(params);
	dai_data->rate = params_rate(params);

	dev_dbg(dai->dev, "channels %d sample rate %d entered\n",
		dai_data->channels, dai_data->rate);

	memset(&dai_data->port_config, 0, sizeof(dai_data->port_config));

	pr_debug("%s: setting bt_fm parameters\n", __func__);

	dai_data->port_config.int_bt_fm.bt_fm_cfg_minor_version =
					AFE_API_VERSION_INTERNAL_BT_FM_CONFIG;
	dai_data->port_config.int_bt_fm.num_channels = dai_data->channels;
	dai_data->port_config.int_bt_fm.sample_rate = dai_data->rate;
	dai_data->port_config.int_bt_fm.bit_width = 16;

	return 0;
}

static int msm_dai_q6_afe_rtproxy_hw_params(struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);

	dai_data->rate = params_rate(params);
	dai_data->port_config.rtproxy.num_channels = params_channels(params);
	dai_data->port_config.rtproxy.sample_rate = params_rate(params);

	pr_debug("channel %d entered,dai_id: %d,rate: %d\n",
	dai_data->port_config.rtproxy.num_channels, dai->id, dai_data->rate);

	dai_data->port_config.rtproxy.rt_proxy_cfg_minor_version =
				AFE_API_VERSION_RT_PROXY_CONFIG;
	dai_data->port_config.rtproxy.bit_width = 16; /* Q6 only supports 16 */
	dai_data->port_config.rtproxy.interleaved = 1;
	dai_data->port_config.rtproxy.frame_size = params_period_bytes(params);
	dai_data->port_config.rtproxy.jitter_allowance =
				dai_data->port_config.rtproxy.frame_size/2;
	dai_data->port_config.rtproxy.low_water_mark = 0;
	dai_data->port_config.rtproxy.high_water_mark = 0;

	return 0;
}

static int msm_dai_q6_pseudo_port_hw_params(struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai, int stream)
{
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);

	dai_data->channels = params_channels(params);
	dai_data->rate = params_rate(params);

	/* Q6 only supports 16 as now */
	dai_data->port_config.pseudo_port.pseud_port_cfg_minor_version =
				AFE_API_VERSION_PSEUDO_PORT_CONFIG;
	dai_data->port_config.pseudo_port.num_channels =
				params_channels(params);
	dai_data->port_config.pseudo_port.bit_width = 16;
	dai_data->port_config.pseudo_port.data_format = 0;
	dai_data->port_config.pseudo_port.timing_mode =
				AFE_PSEUDOPORT_TIMING_MODE_TIMER;
	dai_data->port_config.pseudo_port.sample_rate = params_rate(params);

	dev_dbg(dai->dev, "%s: bit_wd[%hu] num_channels [%hu] format[%hu]\n"
		"timing Mode %hu sample_rate %d\n", __func__,
		dai_data->port_config.pseudo_port.bit_width,
		dai_data->port_config.pseudo_port.num_channels,
		dai_data->port_config.pseudo_port.data_format,
		dai_data->port_config.pseudo_port.timing_mode,
		dai_data->port_config.pseudo_port.sample_rate);

	return 0;
}

/* Current implementation assumes hw_param is called once
 * This may not be the case but what to do when ADM and AFE
 * port are already opened and parameter changes
 */
static int msm_dai_q6_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	int rc = 0;

	switch (dai->id) {
	case PRIMARY_I2S_TX:
	case PRIMARY_I2S_RX:
	case SECONDARY_I2S_RX:
		rc = msm_dai_q6_cdc_hw_params(params, dai, substream->stream);
		break;
	case MI2S_RX:
		rc = msm_dai_q6_i2s_hw_params(params, dai, substream->stream);
		break;
	case SLIMBUS_0_RX:
	case SLIMBUS_1_RX:
	case SLIMBUS_2_RX:
	case SLIMBUS_3_RX:
	case SLIMBUS_4_RX:
	case SLIMBUS_5_RX:
	case SLIMBUS_6_RX:
	case SLIMBUS_7_RX:
	case SLIMBUS_8_RX:
	case SLIMBUS_9_RX:
	case SLIMBUS_0_TX:
	case SLIMBUS_1_TX:
	case SLIMBUS_2_TX:
	case SLIMBUS_3_TX:
	case SLIMBUS_4_TX:
	case SLIMBUS_5_TX:
	case SLIMBUS_6_TX:
	case SLIMBUS_7_TX:
	case SLIMBUS_8_TX:
	case SLIMBUS_9_TX:
		rc = msm_dai_q6_slim_bus_hw_params(params, dai,
				substream->stream);
		break;
	case INT_BT_SCO_RX:
	case INT_BT_SCO_TX:
	case INT_BT_A2DP_RX:
	case INT_FM_RX:
	case INT_FM_TX:
		rc = msm_dai_q6_bt_fm_hw_params(params, dai, substream->stream);
		break;
	case AFE_PORT_ID_USB_RX:
	case AFE_PORT_ID_USB_TX:
		rc = msm_dai_q6_usb_audio_hw_params(params, dai,
						    substream->stream);
		break;
	case RT_PROXY_DAI_001_TX:
	case RT_PROXY_DAI_001_RX:
	case RT_PROXY_DAI_002_TX:
	case RT_PROXY_DAI_002_RX:
	case RT_PROXY_DAI_003_TX:
	case RT_PROXY_DAI_003_RX:
	case RT_PROXY_PORT_002_TX:
	case RT_PROXY_PORT_002_RX:
		rc = msm_dai_q6_afe_rtproxy_hw_params(params, dai);
		break;
	case VOICE_PLAYBACK_TX:
	case VOICE2_PLAYBACK_TX:
	case VOICE_RECORD_RX:
	case VOICE_RECORD_TX:
		rc = msm_dai_q6_pseudo_port_hw_params(params,
						dai, substream->stream);
		break;
	default:
		dev_err(dai->dev, "invalid AFE port ID 0x%x\n", dai->id);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static void msm_dai_q6_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int rc = 0;

	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		pr_debug("%s: stop pseudo port:%d\n", __func__,  dai->id);
		rc = afe_close(dai->id); /* can block */
		if (rc < 0)
			dev_err(dai->dev, "fail to close AFE port\n");
		pr_debug("%s: dai_data->status_mask = %ld\n", __func__,
			*dai_data->status_mask);
		clear_bit(STATUS_PORT_STARTED, dai_data->status_mask);
	}
}

static int msm_dai_q6_cdc_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		dai_data->port_config.i2s.ws_src = 1; /* CPU is master */
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		dai_data->port_config.i2s.ws_src = 0; /* CPU is slave */
		break;
	default:
		pr_err("%s: fmt 0x%x\n",
			__func__, fmt & SND_SOC_DAIFMT_MASTER_MASK);
		return -EINVAL;
	}

	return 0;
}

static int msm_dai_q6_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	int rc = 0;

	dev_dbg(dai->dev, "%s: id = %d fmt[%d]\n", __func__,
							dai->id, fmt);
	switch (dai->id) {
	case PRIMARY_I2S_TX:
	case PRIMARY_I2S_RX:
	case MI2S_RX:
	case SECONDARY_I2S_RX:
		rc = msm_dai_q6_cdc_set_fmt(dai, fmt);
		break;
	default:
		dev_err(dai->dev, "invalid cpu_dai id 0x%x\n", dai->id);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int msm_dai_q6_set_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)

{
	int rc = 0;
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);
	unsigned int i = 0;

	dev_dbg(dai->dev, "%s: id = %d\n", __func__, dai->id);
	switch (dai->id) {
	case SLIMBUS_0_RX:
	case SLIMBUS_1_RX:
	case SLIMBUS_2_RX:
	case SLIMBUS_3_RX:
	case SLIMBUS_4_RX:
	case SLIMBUS_5_RX:
	case SLIMBUS_6_RX:
	case SLIMBUS_7_RX:
	case SLIMBUS_8_RX:
	case SLIMBUS_9_RX:
		/*
		 * channel number to be between 128 and 255.
		 * For RX port use channel numbers
		 * from 138 to 144 for pre-Taiko
		 * from 144 to 159 for Taiko
		 */
		if (!rx_slot) {
			pr_err("%s: rx slot not found\n", __func__);
			return -EINVAL;
		}
		if (rx_num > AFE_PORT_MAX_AUDIO_CHAN_CNT) {
			pr_err("%s: invalid rx num %d\n", __func__, rx_num);
			return -EINVAL;
		}

		for (i = 0; i < rx_num; i++) {
			dai_data->port_config.slim_sch.shared_ch_mapping[i] =
			    rx_slot[i];
			pr_debug("%s: find number of channels[%d] ch[%d]\n",
			       __func__, i, rx_slot[i]);
		}
		dai_data->port_config.slim_sch.num_channels = rx_num;
		pr_debug("%s: SLIMBUS_%d_RX cnt[%d] ch[%d %d]\n", __func__,
			(dai->id - SLIMBUS_0_RX) / 2, rx_num,
			dai_data->port_config.slim_sch.shared_ch_mapping[0],
			dai_data->port_config.slim_sch.shared_ch_mapping[1]);

		break;
	case SLIMBUS_0_TX:
	case SLIMBUS_1_TX:
	case SLIMBUS_2_TX:
	case SLIMBUS_3_TX:
	case SLIMBUS_4_TX:
	case SLIMBUS_5_TX:
	case SLIMBUS_6_TX:
	case SLIMBUS_7_TX:
	case SLIMBUS_8_TX:
	case SLIMBUS_9_TX:
		/*
		 * channel number to be between 128 and 255.
		 * For TX port use channel numbers
		 * from 128 to 137 for pre-Taiko
		 * from 128 to 143 for Taiko
		 */
		if (!tx_slot) {
			pr_err("%s: tx slot not found\n", __func__);
			return -EINVAL;
		}
		if (tx_num > AFE_PORT_MAX_AUDIO_CHAN_CNT) {
			pr_err("%s: invalid tx num %d\n", __func__, tx_num);
			return -EINVAL;
		}

		for (i = 0; i < tx_num; i++) {
			dai_data->port_config.slim_sch.shared_ch_mapping[i] =
			    tx_slot[i];
			pr_debug("%s: find number of channels[%d] ch[%d]\n",
				 __func__, i, tx_slot[i]);
		}
		dai_data->port_config.slim_sch.num_channels = tx_num;
		pr_debug("%s:SLIMBUS_%d_TX cnt[%d] ch[%d %d]\n", __func__,
			(dai->id - SLIMBUS_0_TX) / 2, tx_num,
			dai_data->port_config.slim_sch.shared_ch_mapping[0],
			dai_data->port_config.slim_sch.shared_ch_mapping[1]);
		break;
	default:
		dev_err(dai->dev, "invalid cpu_dai id 0x%x\n", dai->id);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int msm_dai_q6_spk_digital_mute(struct snd_soc_dai *dai,
				       int mute)
{
	int port_id = dai->id;
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);

	if (mute && !dai_data->xt_logging_disable)
		afe_get_sp_xt_logging_data(port_id);

	return 0;
}

static struct snd_soc_dai_ops msm_dai_q6_ops = {
	.prepare	= msm_dai_q6_prepare,
	.hw_params	= msm_dai_q6_hw_params,
	.shutdown	= msm_dai_q6_shutdown,
	.set_fmt	= msm_dai_q6_set_fmt,
	.set_channel_map = msm_dai_q6_set_channel_map,
};

static struct snd_soc_dai_ops msm_dai_slimbus_0_rx_ops = {
	.prepare	= msm_dai_q6_prepare,
	.hw_params	= msm_dai_q6_hw_params,
	.shutdown	= msm_dai_q6_shutdown,
	.set_fmt	= msm_dai_q6_set_fmt,
	.set_channel_map = msm_dai_q6_set_channel_map,
	.digital_mute = msm_dai_q6_spk_digital_mute,
};

static int msm_dai_q6_cal_info_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	u16 port_id = ((struct soc_enum *)
					kcontrol->private_value)->reg;

	dai_data->cal_mode = ucontrol->value.integer.value[0];
	pr_debug("%s: setting cal_mode to %d\n",
		__func__, dai_data->cal_mode);
	afe_set_cal_mode(port_id, dai_data->cal_mode);

	return 0;
}

static int msm_dai_q6_cal_info_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	ucontrol->value.integer.value[0] = dai_data->cal_mode;
	return 0;
}

static int msm_dai_q6_cdc_dma_xt_logging_disable_put(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_cdc_dma_dai_data *dai_data = kcontrol->private_data;

	if (dai_data) {
		dai_data->xt_logging_disable = ucontrol->value.integer.value[0];
		pr_debug("%s: setting xt logging disable to %d\n",
			__func__, dai_data->xt_logging_disable);
	}

	return 0;
}

static int msm_dai_q6_cdc_dma_xt_logging_disable_get(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_cdc_dma_dai_data *dai_data = kcontrol->private_data;

	if (dai_data)
		ucontrol->value.integer.value[0] = dai_data->xt_logging_disable;
	return 0;
}

static int msm_dai_q6_sb_xt_logging_disable_put(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data) {
		dai_data->xt_logging_disable = ucontrol->value.integer.value[0];
		pr_debug("%s: setting xt logging disable to %d\n",
			__func__, dai_data->xt_logging_disable);
	}

	return 0;
}

static int msm_dai_q6_sb_xt_logging_disable_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data)
		ucontrol->value.integer.value[0] = dai_data->xt_logging_disable;
	return 0;
}

static int msm_dai_q6_sb_format_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	int value = ucontrol->value.integer.value[0];

	if (dai_data) {
		dai_data->port_config.slim_sch.data_format = value;
		pr_debug("%s: format = %d\n",  __func__, value);
	}

	return 0;
}

static int msm_dai_q6_sb_format_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data)
		ucontrol->value.integer.value[0] =
			dai_data->port_config.slim_sch.data_format;

	return 0;
}

static int msm_dai_q6_usb_audio_cfg_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	u32 val = ucontrol->value.integer.value[0];

	if (dai_data) {
		dai_data->port_config.usb_audio.dev_token = val;
		pr_debug("%s: dev_token = 0x%x\n",  __func__,
				 dai_data->port_config.usb_audio.dev_token);
	} else {
		pr_err("%s: dai_data is NULL\n", __func__);
	}

	return 0;
}

static int msm_dai_q6_usb_audio_cfg_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data) {
		ucontrol->value.integer.value[0] =
			 dai_data->port_config.usb_audio.dev_token;
		pr_debug("%s: dev_token = 0x%x\n",  __func__,
				 dai_data->port_config.usb_audio.dev_token);
	} else {
		pr_err("%s: dai_data is NULL\n", __func__);
	}

	return 0;
}

static int msm_dai_q6_usb_audio_endian_cfg_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	u32 val = ucontrol->value.integer.value[0];

	if (dai_data) {
		dai_data->port_config.usb_audio.endian = val;
		pr_debug("%s: endian = 0x%x\n",  __func__,
				 dai_data->port_config.usb_audio.endian);
	} else {
		pr_err("%s: dai_data is NULL\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int msm_dai_q6_usb_audio_endian_cfg_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data) {
		ucontrol->value.integer.value[0] =
			 dai_data->port_config.usb_audio.endian;
		pr_debug("%s: endian = 0x%x\n",  __func__,
				 dai_data->port_config.usb_audio.endian);
	} else {
		pr_err("%s: dai_data is NULL\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int msm_dai_q6_usb_audio_svc_interval_put(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	u32 val = ucontrol->value.integer.value[0];

	if (!dai_data) {
		pr_err("%s: dai_data is NULL\n", __func__);
		return -EINVAL;
	}
	dai_data->port_config.usb_audio.service_interval = val;
	pr_debug("%s: new service interval = %u\n",  __func__,
		dai_data->port_config.usb_audio.service_interval);
	return 0;
}

static int msm_dai_q6_usb_audio_svc_interval_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: dai_data is NULL\n", __func__);
		return -EINVAL;
	}
	ucontrol->value.integer.value[0] =
		 dai_data->port_config.usb_audio.service_interval;
	pr_debug("%s: service interval = %d\n",  __func__,
		     dai_data->port_config.usb_audio.service_interval);
	return 0;
}

static int  msm_dai_q6_afe_enc_cfg_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct afe_enc_config);

	return 0;
}

static int msm_dai_q6_afe_enc_cfg_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data) {
		int format_size = sizeof(dai_data->enc_config.format);

		pr_debug("%s: encoder config for %d format\n",
			 __func__, dai_data->enc_config.format);
		memcpy(ucontrol->value.bytes.data,
			&dai_data->enc_config.format,
			format_size);
		switch (dai_data->enc_config.format) {
		case ENC_FMT_SBC:
			memcpy(ucontrol->value.bytes.data + format_size,
				&dai_data->enc_config.data,
				sizeof(struct asm_sbc_enc_cfg_t));
			break;
		case ENC_FMT_AAC_V2:
			memcpy(ucontrol->value.bytes.data + format_size,
				&dai_data->enc_config.data,
				sizeof(struct asm_aac_enc_cfg_t));
			break;
		case ENC_FMT_APTX:
			memcpy(ucontrol->value.bytes.data + format_size,
				&dai_data->enc_config.data,
				sizeof(struct asm_aptx_enc_cfg_t));
			break;
		case ENC_FMT_APTX_HD:
			memcpy(ucontrol->value.bytes.data + format_size,
				&dai_data->enc_config.data,
				sizeof(struct asm_custom_enc_cfg_t));
			break;
		case ENC_FMT_CELT:
			memcpy(ucontrol->value.bytes.data + format_size,
				&dai_data->enc_config.data,
				sizeof(struct asm_celt_enc_cfg_t));
			break;
		case ENC_FMT_LDAC:
			memcpy(ucontrol->value.bytes.data + format_size,
				&dai_data->enc_config.data,
				sizeof(struct asm_ldac_enc_cfg_t));
			break;
		case ENC_FMT_APTX_ADAPTIVE:
			memcpy(ucontrol->value.bytes.data + format_size,
				&dai_data->enc_config.data,
				sizeof(struct asm_aptx_ad_enc_cfg_t));
			break;
		case ENC_FMT_APTX_AD_SPEECH:
			memcpy(ucontrol->value.bytes.data + format_size,
				&dai_data->enc_config.data,
				sizeof(struct asm_aptx_ad_speech_enc_cfg_t));
			break;
		case ENC_FMT_LC3:
			memcpy(ucontrol->value.bytes.data + format_size,
				&dai_data->enc_config.data,
				sizeof(struct asm_enc_lc3_cfg_t));
			break;
		default:
			pr_debug("%s: unknown format = %d\n",
				 __func__, dai_data->enc_config.format);
			ret = -EINVAL;
			break;
		}
	}

	return ret;
}

static int msm_dai_q6_afe_enc_cfg_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data) {
		int format_size = sizeof(dai_data->enc_config.format);

		memset(&dai_data->enc_config, 0x0,
			sizeof(struct afe_enc_config));
		memcpy(&dai_data->enc_config.format,
			ucontrol->value.bytes.data,
			format_size);
		pr_debug("%s: Received encoder config for %d format\n",
			 __func__, dai_data->enc_config.format);
		switch (dai_data->enc_config.format) {
		case ENC_FMT_SBC:
			memcpy(&dai_data->enc_config.data,
				ucontrol->value.bytes.data + format_size,
				sizeof(struct asm_sbc_enc_cfg_t));
			break;
		case ENC_FMT_AAC_V2:
			memcpy(&dai_data->enc_config.data,
				ucontrol->value.bytes.data + format_size,
				sizeof(struct asm_aac_enc_cfg_t));
			break;
		case ENC_FMT_APTX:
			memcpy(&dai_data->enc_config.data,
				ucontrol->value.bytes.data + format_size,
				sizeof(struct asm_aptx_enc_cfg_t));
			break;
		case ENC_FMT_APTX_HD:
			memcpy(&dai_data->enc_config.data,
				ucontrol->value.bytes.data + format_size,
				sizeof(struct asm_custom_enc_cfg_t));
			break;
		case ENC_FMT_CELT:
			memcpy(&dai_data->enc_config.data,
				ucontrol->value.bytes.data + format_size,
				sizeof(struct asm_celt_enc_cfg_t));
			break;
		case ENC_FMT_LDAC:
			memcpy(&dai_data->enc_config.data,
				ucontrol->value.bytes.data + format_size,
				sizeof(struct asm_ldac_enc_cfg_t));
			break;
		case ENC_FMT_APTX_ADAPTIVE:
			memcpy(&dai_data->enc_config.data,
				ucontrol->value.bytes.data + format_size,
				sizeof(struct asm_aptx_ad_enc_cfg_t));
			break;
		case ENC_FMT_APTX_AD_SPEECH:
			memcpy(&dai_data->enc_config.data,
				ucontrol->value.bytes.data + format_size,
				sizeof(struct asm_aptx_ad_speech_enc_cfg_t));
			break;
		case ENC_FMT_LC3:
			memcpy(&dai_data->enc_config.data,
				ucontrol->value.bytes.data + format_size,
				sizeof(struct asm_enc_lc3_cfg_t));
			break;

		default:
			pr_debug("%s: Ignore enc config for unknown format = %d\n",
				 __func__, dai_data->enc_config.format);
			ret = -EINVAL;
			break;
		}
	} else
		ret = -EINVAL;

	return ret;
}

static const char *const afe_chs_text[] = {"Zero", "One", "Two"};

static const struct soc_enum afe_chs_enum[] = {
	SOC_ENUM_SINGLE_EXT(3, afe_chs_text),
};

static const char *const afe_bit_format_text[] = {"S16_LE", "S24_LE",
							"S32_LE"};

static const struct soc_enum afe_bit_format_enum[] = {
	SOC_ENUM_SINGLE_EXT(3, afe_bit_format_text),
};

static const char *const tws_chs_mode_text[] = {"Zero", "One", "Two"};

static const struct soc_enum tws_chs_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tws_chs_mode_text), tws_chs_mode_text),
};

static const char *const lc3_chs_mode_text[] = {"Zero", "One", "Two"};

static const struct soc_enum lc3_chs_mode_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(lc3_chs_mode_text), lc3_chs_mode_text),
};

static int msm_dai_q6_afe_input_channel_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data) {
		ucontrol->value.integer.value[0] = dai_data->afe_rx_in_channels;
		pr_debug("%s:afe input channel = %d\n",
			  __func__, dai_data->afe_rx_in_channels);
	}

	return 0;
}

static int msm_dai_q6_afe_input_channel_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data) {
		dai_data->afe_rx_in_channels = ucontrol->value.integer.value[0];
		pr_debug("%s: updating afe input channel : %d\n",
			__func__, dai_data->afe_rx_in_channels);
	}

	return 0;
}

static int msm_dai_q6_tws_channel_mode_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = kcontrol->private_data;
	struct msm_dai_q6_dai_data *dai_data = NULL;

	if (dai)
		dai_data = dev_get_drvdata(dai->dev);

	if (dai_data) {
		ucontrol->value.integer.value[0] =
				dai_data->enc_config.mono_mode;
		pr_debug("%s:tws channel mode = %d\n",
			 __func__, dai_data->enc_config.mono_mode);
	}

	return 0;
}

static int msm_dai_q6_tws_channel_mode_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = kcontrol->private_data;
	struct msm_dai_q6_dai_data *dai_data = NULL;
	int ret = 0;
	u32 format = 0;

	if (dai)
		dai_data = dev_get_drvdata(dai->dev);

	if (dai_data)
		format = dai_data->enc_config.format;
	else
		goto exit;

	if (format == ENC_FMT_APTX || format == ENC_FMT_APTX_ADAPTIVE) {
		if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
			ret = afe_set_tws_channel_mode(format,
				dai->id, ucontrol->value.integer.value[0]);
			if (ret < 0) {
				pr_err("%s: channel mode setting failed for TWS\n",
				__func__);
				goto exit;
			} else {
				pr_debug("%s: updating tws channel mode : %d\n",
				__func__, dai_data->enc_config.mono_mode);
			}
		}
		if (ucontrol->value.integer.value[0] ==
			MSM_DAI_TWS_CHANNEL_MODE_ONE ||
			ucontrol->value.integer.value[0] ==
			MSM_DAI_TWS_CHANNEL_MODE_TWO)
			dai_data->enc_config.mono_mode =
				ucontrol->value.integer.value[0];
		else
			return -EINVAL;
	}
exit:
	return ret;
}

static int msm_dai_q6_lc3_channel_mode_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = kcontrol->private_data;
	struct msm_dai_q6_dai_data *dai_data = NULL;

	if (dai)
		dai_data = dev_get_drvdata(dai->dev);

	if (dai_data) {
		ucontrol->value.integer.value[0] =
				dai_data->enc_config.lc3_mono_mode;
		pr_debug("%s:lc3 channel mode = %d\n",
			 __func__, dai_data->enc_config.lc3_mono_mode);
	}

	return 0;
}

static int msm_dai_q6_lc3_channel_mode_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = kcontrol->private_data;
	struct msm_dai_q6_dai_data *dai_data = NULL;
	int ret = 0;
	u32 format = 0;

	if (dai)
		dai_data = dev_get_drvdata(dai->dev);

	if (dai_data)
		format = dai_data->enc_config.format;
	else
		goto exit;

	if (format == ENC_FMT_LC3) {
		if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
			ret = afe_set_lc3_channel_mode(format,
				dai->id, ucontrol->value.integer.value[0]);
			if (ret < 0) {
				pr_err("%s: channel mode setting failed for LC3\n",
				__func__);
				goto exit;
			} else {
				pr_debug("%s: updating lc3 channel mode : %d\n",
				__func__, dai_data->enc_config.lc3_mono_mode);
			}
		}
		if (ucontrol->value.integer.value[0] ==
			MSM_DAI_LC3_CHANNEL_MODE_ONE ||
			ucontrol->value.integer.value[0] ==
			MSM_DAI_LC3_CHANNEL_MODE_TWO)
			dai_data->enc_config.lc3_mono_mode =
				ucontrol->value.integer.value[0];
		else
			return -EINVAL;
	}
exit:
	return ret;
}

static int msm_dai_q6_afe_input_bit_format_get(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}

	switch (dai_data->afe_rx_in_bitformat) {
	case SNDRV_PCM_FORMAT_S32_LE:
		ucontrol->value.integer.value[0] = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		ucontrol->value.integer.value[0] = 1;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
	default:
		ucontrol->value.integer.value[0] = 0;
		break;
	}
	pr_debug("%s: afe input bit format : %ld\n",
		  __func__, ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_dai_q6_afe_input_bit_format_put(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}
	switch (ucontrol->value.integer.value[0]) {
	case 2:
		dai_data->afe_rx_in_bitformat = SNDRV_PCM_FORMAT_S32_LE;
		break;
	case 1:
		dai_data->afe_rx_in_bitformat = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 0:
	default:
		dai_data->afe_rx_in_bitformat = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}
	pr_debug("%s: updating afe input bit format : %d\n",
		__func__, dai_data->afe_rx_in_bitformat);

	return 0;
}

static int msm_dai_q6_afe_output_bit_format_get(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}

	switch (dai_data->afe_tx_out_bitformat) {
	case SNDRV_PCM_FORMAT_S32_LE:
		ucontrol->value.integer.value[0] = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		ucontrol->value.integer.value[0] = 1;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
	default:
		ucontrol->value.integer.value[0] = 0;
		break;
	}
	pr_debug("%s: afe output bit format : %ld\n",
		  __func__, ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_dai_q6_afe_output_bit_format_put(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}
	switch (ucontrol->value.integer.value[0]) {
	case 2:
		dai_data->afe_tx_out_bitformat = SNDRV_PCM_FORMAT_S32_LE;
		break;
	case 1:
		dai_data->afe_tx_out_bitformat = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 0:
	default:
		dai_data->afe_tx_out_bitformat = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}
	pr_debug("%s: updating afe output bit format : %d\n",
		__func__, dai_data->afe_tx_out_bitformat);

	return 0;
}

static int msm_dai_q6_afe_output_channel_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data) {
		ucontrol->value.integer.value[0] =
			dai_data->afe_tx_out_channels;
		pr_debug("%s:afe output channel = %d\n",
			  __func__, dai_data->afe_tx_out_channels);
	}
	return 0;
}

static int msm_dai_q6_afe_output_channel_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (dai_data) {
		dai_data->afe_tx_out_channels =
			ucontrol->value.integer.value[0];
		pr_debug("%s: updating afe output channel : %d\n",
			__func__, dai_data->afe_tx_out_channels);
	}
	return 0;
}

static int msm_dai_q6_afe_scrambler_mode_get(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}
	ucontrol->value.integer.value[0] = dai_data->enc_config.scrambler_mode;

	return 0;
}

static int msm_dai_q6_afe_scrambler_mode_put(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}
	dai_data->enc_config.scrambler_mode = ucontrol->value.integer.value[0];
	pr_debug("%s: afe scrambler mode : %d\n",
		  __func__, dai_data->enc_config.scrambler_mode);
	return 0;
}

static const struct snd_kcontrol_new afe_enc_config_controls[] = {
	{
		.access = (SNDRV_CTL_ELEM_ACCESS_READWRITE |
			   SNDRV_CTL_ELEM_ACCESS_INACTIVE),
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = "SLIM_7_RX Encoder Config",
		.info = msm_dai_q6_afe_enc_cfg_info,
		.get = msm_dai_q6_afe_enc_cfg_get,
		.put = msm_dai_q6_afe_enc_cfg_put,
	},
	SOC_ENUM_EXT("AFE Input Channels", afe_chs_enum[0],
		     msm_dai_q6_afe_input_channel_get,
		     msm_dai_q6_afe_input_channel_put),
	SOC_ENUM_EXT("AFE Input Bit Format", afe_bit_format_enum[0],
		     msm_dai_q6_afe_input_bit_format_get,
		     msm_dai_q6_afe_input_bit_format_put),
	SOC_SINGLE_EXT("AFE Scrambler Mode",
		       0, 0, 1, 0,
		       msm_dai_q6_afe_scrambler_mode_get,
		       msm_dai_q6_afe_scrambler_mode_put),
	SOC_ENUM_EXT("TWS Channel Mode", tws_chs_mode_enum[0],
		       msm_dai_q6_tws_channel_mode_get,
		       msm_dai_q6_tws_channel_mode_put),
	{
		.access = (SNDRV_CTL_ELEM_ACCESS_READWRITE |
			   SNDRV_CTL_ELEM_ACCESS_INACTIVE),
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = "SLIM_7_RX APTX_AD Enc Cfg",
		.info = msm_dai_q6_afe_enc_cfg_info,
		.get = msm_dai_q6_afe_enc_cfg_get,
		.put = msm_dai_q6_afe_enc_cfg_put,
	},
	SOC_ENUM_EXT("LC3 Channel Mode", lc3_chs_mode_enum[0],
			msm_dai_q6_lc3_channel_mode_get,
			msm_dai_q6_lc3_channel_mode_put)
};

static int  msm_dai_q6_afe_dec_cfg_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct afe_dec_config);

	return 0;
}

static int msm_dai_q6_afe_feedback_dec_cfg_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	u32 format_size = 0;
	u32 abr_size = 0;

	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}

	format_size = sizeof(dai_data->dec_config.format);
	memcpy(ucontrol->value.bytes.data,
		&dai_data->dec_config.format,
		format_size);

	pr_debug("%s: abr_dec_cfg for %d format\n",
			__func__, dai_data->dec_config.format);
	abr_size = sizeof(dai_data->dec_config.abr_dec_cfg.imc_info);
	memcpy(ucontrol->value.bytes.data + format_size,
		&dai_data->dec_config.abr_dec_cfg,
		sizeof(struct afe_imc_dec_enc_info));

	switch (dai_data->dec_config.format) {
	case DEC_FMT_APTX_AD_SPEECH:
		pr_debug("%s: afe_dec_cfg for %d format\n",
				__func__, dai_data->dec_config.format);
		memcpy(ucontrol->value.bytes.data + format_size + abr_size,
			&dai_data->dec_config.data,
			sizeof(struct asm_aptx_ad_speech_dec_cfg_t));
		break;
	case ASM_MEDIA_FMT_LC3:
		pr_debug("%s: afe_dec_cfg for %d format\n",
				__func__, dai_data->dec_config.format);
		memcpy(ucontrol->value.bytes.data + format_size + abr_size,
			&dai_data->dec_config.data,
			sizeof(struct asm_lc3_dec_cfg_t));
		break;
	default:
		pr_debug("%s: no afe_dec_cfg for format %d\n",
				__func__, dai_data->dec_config.format);
		break;
	}

	return 0;
}

static int msm_dai_q6_afe_feedback_dec_cfg_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	u32 format_size = 0;
	u32 abr_size = 0;

	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}

	memset(&dai_data->dec_config, 0x0,
		sizeof(struct afe_dec_config));
	format_size = sizeof(dai_data->dec_config.format);
	memcpy(&dai_data->dec_config.format,
		ucontrol->value.bytes.data,
		format_size);

	pr_debug("%s: abr_dec_cfg for %d format\n",
			__func__, dai_data->dec_config.format);
	abr_size = sizeof(dai_data->dec_config.abr_dec_cfg.imc_info);
	memcpy(&dai_data->dec_config.abr_dec_cfg,
		ucontrol->value.bytes.data + format_size,
		sizeof(struct afe_imc_dec_enc_info));
	dai_data->dec_config.abr_dec_cfg.is_abr_enabled = true;

	switch (dai_data->dec_config.format) {
	case DEC_FMT_APTX_AD_SPEECH:
		pr_debug("%s: afe_dec_cfg for %d format\n",
				__func__, dai_data->dec_config.format);
		memcpy(&dai_data->dec_config.data,
			ucontrol->value.bytes.data + format_size + abr_size,
			sizeof(struct asm_aptx_ad_speech_dec_cfg_t));
		break;
	case ASM_MEDIA_FMT_LC3:
		pr_debug("%s: afe_dec_cfg for %d format\n",
				__func__, dai_data->dec_config.format);
		memcpy(&dai_data->dec_config.data,
			ucontrol->value.bytes.data + format_size + abr_size,
			sizeof(struct asm_lc3_dec_cfg_t));
		break;
	default:
		pr_debug("%s: no afe_dec_cfg for format %d\n",
				__func__, dai_data->dec_config.format);
		break;
	}
	return 0;
}

static int msm_dai_q6_afe_dec_cfg_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	u32 format_size = 0;
	int ret = 0;

	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}

	format_size = sizeof(dai_data->dec_config.format);
	memcpy(ucontrol->value.bytes.data,
		&dai_data->dec_config.format,
		format_size);
	switch (dai_data->dec_config.format) {
	case DEC_FMT_AAC_V2:
		memcpy(ucontrol->value.bytes.data + format_size,
			&dai_data->dec_config.data,
			sizeof(struct asm_aac_dec_cfg_v2_t));
		break;
	case DEC_FMT_APTX_ADAPTIVE:
		memcpy(ucontrol->value.bytes.data + format_size,
			&dai_data->dec_config.data,
			sizeof(struct asm_aptx_ad_dec_cfg_t));
		break;
	case DEC_FMT_SBC:
	case DEC_FMT_MP3:
		/* No decoder specific data available */
		break;
	default:
		pr_err("%s: Invalid format %d\n",
				__func__, dai_data->dec_config.format);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int msm_dai_q6_afe_dec_cfg_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	u32 format_size = 0;
	int ret = 0;

	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}

	memset(&dai_data->dec_config, 0x0,
		sizeof(struct afe_dec_config));
	format_size = sizeof(dai_data->dec_config.format);
	memcpy(&dai_data->dec_config.format,
		ucontrol->value.bytes.data,
		format_size);
	pr_debug("%s: Received decoder config for %d format\n",
			__func__, dai_data->dec_config.format);
	switch (dai_data->dec_config.format) {
	case DEC_FMT_AAC_V2:
		memcpy(&dai_data->dec_config.data,
			ucontrol->value.bytes.data + format_size,
			sizeof(struct asm_aac_dec_cfg_v2_t));
		break;
	case DEC_FMT_SBC:
		memcpy(&dai_data->dec_config.data,
			ucontrol->value.bytes.data + format_size,
			sizeof(struct asm_sbc_dec_cfg_t));
		break;
	case DEC_FMT_APTX_ADAPTIVE:
		memcpy(&dai_data->dec_config.data,
			ucontrol->value.bytes.data + format_size,
			sizeof(struct asm_aptx_ad_dec_cfg_t));
		break;
	default:
		pr_err("%s: Invalid format %d\n",
				__func__, dai_data->dec_config.format);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int  msm_dai_q6_afe_enable_ttp_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct afe_ttp_gen_enable_t);

	return 0;
}

static int msm_dai_q6_afe_enable_ttp_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	pr_debug("%s:\n", __func__);
	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}

	memcpy(ucontrol->value.bytes.data,
	       &dai_data->ttp_config.ttp_gen_enable,
	       sizeof(struct afe_ttp_gen_enable_t));
	return 0;
}

static int msm_dai_q6_afe_enable_ttp_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	pr_debug("%s:\n", __func__);
	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}

	memcpy(&dai_data->ttp_config.ttp_gen_enable,
		ucontrol->value.bytes.data,
		sizeof(struct afe_ttp_gen_enable_t));
	return 0;
}

static int  msm_dai_q6_afe_ttp_cfg_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct afe_ttp_gen_cfg_t);

	return 0;
}

static int msm_dai_q6_afe_ttp_cfg_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	pr_debug("%s:\n", __func__);
	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}

	memcpy(ucontrol->value.bytes.data,
	       &dai_data->ttp_config.ttp_gen_cfg,
	       sizeof(struct afe_ttp_gen_cfg_t));
	return 0;
}

static int msm_dai_q6_afe_ttp_cfg_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	pr_debug("%s: Received ttp config\n", __func__);
	if (!dai_data) {
		pr_err("%s: Invalid dai data\n", __func__);
		return -EINVAL;
	}

	memcpy(&dai_data->ttp_config.ttp_gen_cfg,
		ucontrol->value.bytes.data, sizeof(struct afe_ttp_gen_cfg_t));
	return 0;
}

static const struct snd_kcontrol_new afe_dec_config_controls[] = {
	{
		.access = (SNDRV_CTL_ELEM_ACCESS_READWRITE |
			   SNDRV_CTL_ELEM_ACCESS_INACTIVE),
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = "SLIM_7_TX Decoder Config",
		.info = msm_dai_q6_afe_dec_cfg_info,
		.get = msm_dai_q6_afe_feedback_dec_cfg_get,
		.put = msm_dai_q6_afe_feedback_dec_cfg_put,
	},
	{
		.access = (SNDRV_CTL_ELEM_ACCESS_READWRITE |
			   SNDRV_CTL_ELEM_ACCESS_INACTIVE),
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = "SLIM_9_TX Decoder Config",
		.info = msm_dai_q6_afe_dec_cfg_info,
		.get = msm_dai_q6_afe_dec_cfg_get,
		.put = msm_dai_q6_afe_dec_cfg_put,
	},
	SOC_ENUM_EXT("AFE Output Channels", afe_chs_enum[0],
		     msm_dai_q6_afe_output_channel_get,
		     msm_dai_q6_afe_output_channel_put),
	SOC_ENUM_EXT("AFE Output Bit Format", afe_bit_format_enum[0],
		     msm_dai_q6_afe_output_bit_format_get,
		     msm_dai_q6_afe_output_bit_format_put),
	SOC_ENUM_EXT("AFE Output Channels SLIM7", afe_chs_enum[0],
		     msm_dai_q6_afe_output_channel_get,
		     msm_dai_q6_afe_output_channel_put),
};

static const struct snd_kcontrol_new afe_ttp_config_controls[] = {
	{
		.access = (SNDRV_CTL_ELEM_ACCESS_READWRITE |
			   SNDRV_CTL_ELEM_ACCESS_INACTIVE),
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = "TTP Enable",
		.info = msm_dai_q6_afe_enable_ttp_info,
		.get = msm_dai_q6_afe_enable_ttp_get,
		.put = msm_dai_q6_afe_enable_ttp_put,
	},
	{
		.access = (SNDRV_CTL_ELEM_ACCESS_READWRITE |
			   SNDRV_CTL_ELEM_ACCESS_INACTIVE),
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = "AFE TTP config",
		.info = msm_dai_q6_afe_ttp_cfg_info,
		.get = msm_dai_q6_afe_ttp_cfg_get,
		.put = msm_dai_q6_afe_ttp_cfg_put,
	},
};

static int msm_dai_q6_slim_rx_drift_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct afe_param_id_dev_timing_stats);

	return 0;
}

static int msm_dai_q6_slim_rx_drift_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;
	struct afe_param_id_dev_timing_stats timing_stats;
	struct snd_soc_dai *dai = kcontrol->private_data;
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		pr_debug("%s: afe port not started. dai_data->status_mask = %ld\n",
			__func__, *dai_data->status_mask);
		goto done;
	}

	memset(&timing_stats, 0, sizeof(struct afe_param_id_dev_timing_stats));
	ret = afe_get_av_dev_drift(&timing_stats, dai->id);
	if (ret) {
		pr_err("%s: Error getting AFE Drift for port %d, err=%d\n",
			__func__, dai->id, ret);

		goto done;
	}

	memcpy(ucontrol->value.bytes.data, (void *)&timing_stats,
			sizeof(struct afe_param_id_dev_timing_stats));
done:
	return ret;
}

static const char * const afe_cal_mode_text[] = {
	"CAL_MODE_DEFAULT", "CAL_MODE_NONE"
};

static const struct soc_enum slim_2_rx_enum =
	SOC_ENUM_SINGLE(SLIMBUS_2_RX, 0, ARRAY_SIZE(afe_cal_mode_text),
			afe_cal_mode_text);

static const struct soc_enum rt_proxy_1_rx_enum =
	SOC_ENUM_SINGLE(RT_PROXY_PORT_001_RX, 0, ARRAY_SIZE(afe_cal_mode_text),
			afe_cal_mode_text);

static const struct soc_enum rt_proxy_2_rx_enum =
	SOC_ENUM_SINGLE(RT_PROXY_PORT_002_RX, 0, ARRAY_SIZE(afe_cal_mode_text),
			afe_cal_mode_text);

static const struct soc_enum rt_proxy_1_tx_enum =
	SOC_ENUM_SINGLE(RT_PROXY_PORT_001_TX, 0, ARRAY_SIZE(afe_cal_mode_text),
			afe_cal_mode_text);

static const struct snd_kcontrol_new sb_config_controls[] = {
	SOC_ENUM_EXT("SLIM_4_TX Format", sb_config_enum[0],
		     msm_dai_q6_sb_format_get,
		     msm_dai_q6_sb_format_put),
	SOC_ENUM_EXT("SLIM_2_RX SetCalMode", slim_2_rx_enum,
		     msm_dai_q6_cal_info_get,
		     msm_dai_q6_cal_info_put),
	SOC_ENUM_EXT("SLIM_2_RX Format", sb_config_enum[0],
		     msm_dai_q6_sb_format_get,
		     msm_dai_q6_sb_format_put),
	SOC_ENUM_EXT("SLIM_0_RX XTLoggingDisable", xt_logging_disable_enum[0],
		     msm_dai_q6_sb_xt_logging_disable_get,
		     msm_dai_q6_sb_xt_logging_disable_put),
};

static const struct snd_kcontrol_new rt_proxy_config_controls[] = {
	SOC_ENUM_EXT("RT_PROXY_1_RX SetCalMode", rt_proxy_1_rx_enum,
			msm_dai_q6_cal_info_get,
			msm_dai_q6_cal_info_put),
	SOC_ENUM_EXT("RT_PROXY_1_TX SetCalMode", rt_proxy_1_tx_enum,
			msm_dai_q6_cal_info_get,
			msm_dai_q6_cal_info_put),
	SOC_ENUM_EXT("RT_PROXY_2_RX SetCalMode", rt_proxy_2_rx_enum,
			msm_dai_q6_cal_info_get,
			msm_dai_q6_cal_info_put),
};

static const struct snd_kcontrol_new usb_audio_cfg_controls[] = {
	SOC_SINGLE_EXT("USB_AUDIO_RX dev_token", 0, 0, UINT_MAX, 0,
			msm_dai_q6_usb_audio_cfg_get,
			msm_dai_q6_usb_audio_cfg_put),
	SOC_SINGLE_EXT("USB_AUDIO_RX endian", 0, 0, 1, 0,
			msm_dai_q6_usb_audio_endian_cfg_get,
			msm_dai_q6_usb_audio_endian_cfg_put),
	SOC_SINGLE_EXT("USB_AUDIO_TX dev_token", 0, 0, UINT_MAX, 0,
			msm_dai_q6_usb_audio_cfg_get,
			msm_dai_q6_usb_audio_cfg_put),
	SOC_SINGLE_EXT("USB_AUDIO_TX endian", 0, 0, 1, 0,
			msm_dai_q6_usb_audio_endian_cfg_get,
			msm_dai_q6_usb_audio_endian_cfg_put),
	SOC_SINGLE_EXT("USB_AUDIO_RX service_interval", SND_SOC_NOPM, 0,
			UINT_MAX, 0,
			msm_dai_q6_usb_audio_svc_interval_get,
			msm_dai_q6_usb_audio_svc_interval_put),
};

static const struct snd_kcontrol_new avd_drift_config_controls[] = {
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_PCM,
		.name	= "SLIMBUS_0_RX DRIFT",
		.info	= msm_dai_q6_slim_rx_drift_info,
		.get	= msm_dai_q6_slim_rx_drift_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_PCM,
		.name	= "SLIMBUS_6_RX DRIFT",
		.info	= msm_dai_q6_slim_rx_drift_info,
		.get	= msm_dai_q6_slim_rx_drift_get,
	},
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_PCM,
		.name	= "SLIMBUS_7_RX DRIFT",
		.info	= msm_dai_q6_slim_rx_drift_info,
		.get	= msm_dai_q6_slim_rx_drift_get,
	},
};

static inline void msm_dai_q6_set_slim_dev_id(struct snd_soc_dai *dai)
{
	int rc = 0;
	int slim_dev_id = 0;
	const char *q6_slim_dev_id = "qcom,msm-dai-q6-slim-dev-id";
	struct msm_dai_q6_dai_data *dai_data = dev_get_drvdata(dai->dev);

	dai_data->port_config.slim_sch.slimbus_dev_id = AFE_SLIMBUS_DEVICE_1;

	rc = of_property_read_u32(dai->dev->of_node, q6_slim_dev_id,
				  &slim_dev_id);
	if (rc) {
		dev_dbg(dai->dev,
			"%s: missing %s in dt node\n", __func__, q6_slim_dev_id);
		return;
	}

	dev_dbg(dai->dev, "%s: slim_dev_id = %d\n", __func__, slim_dev_id);

	if (slim_dev_id >= AFE_SLIMBUS_DEVICE_1 &&
	    slim_dev_id <= AFE_SLIMBUS_DEVICE_2)
		dai_data->port_config.slim_sch.slimbus_dev_id = slim_dev_id;
}

static int msm_dai_q6_dai_probe(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_dai_data *dai_data;
	int rc = 0;

	if (!dai) {
		pr_err("%s: Invalid params dai\n", __func__);
		return -EINVAL;
	}
	if (!dai->dev) {
		pr_err("%s: Invalid params dai dev\n", __func__);
		return -EINVAL;
	}

	dai_data = kzalloc(sizeof(struct msm_dai_q6_dai_data), GFP_KERNEL);

	if (!dai_data)
		return -ENOMEM;
	else
		dev_set_drvdata(dai->dev, dai_data);

	msm_dai_q6_set_dai_id(dai);

	if ((dai->id >= SLIMBUS_0_RX) && (dai->id <= SLIMBUS_9_TX))
		msm_dai_q6_set_slim_dev_id(dai);

	switch (dai->id) {
	case SLIMBUS_4_TX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&sb_config_controls[0],
				 dai_data));
		break;
	case SLIMBUS_2_RX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&sb_config_controls[1],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&sb_config_controls[2],
				 dai_data));
		break;
	case SLIMBUS_7_RX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_enc_config_controls[0],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_enc_config_controls[1],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_enc_config_controls[2],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_enc_config_controls[3],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_enc_config_controls[4],
				 dai));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_enc_config_controls[5],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(&afe_enc_config_controls[6],
				dai));
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(&avd_drift_config_controls[2],
					dai));
		break;
	case SLIMBUS_7_TX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_dec_config_controls[0],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_dec_config_controls[4],
				 dai_data));
		break;
	case SLIMBUS_9_TX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_dec_config_controls[1],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_dec_config_controls[2],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_dec_config_controls[3],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_ttp_config_controls[0],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&afe_ttp_config_controls[1],
				 dai_data));
		break;
	case RT_PROXY_DAI_001_RX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&rt_proxy_config_controls[0],
				 dai_data));
		break;
	case RT_PROXY_DAI_001_TX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&rt_proxy_config_controls[1],
				 dai_data));
		break;
	case RT_PROXY_DAI_003_RX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(&rt_proxy_config_controls[2],
				dai_data));
		break;
	case AFE_PORT_ID_USB_RX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&usb_audio_cfg_controls[0],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&usb_audio_cfg_controls[1],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&usb_audio_cfg_controls[4],
				 dai_data));
		break;
	case AFE_PORT_ID_USB_TX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&usb_audio_cfg_controls[2],
				 dai_data));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&usb_audio_cfg_controls[3],
				 dai_data));
		break;
	case SLIMBUS_0_RX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(&avd_drift_config_controls[0],
					dai));
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&sb_config_controls[3],
				 dai_data));
		break;
	case SLIMBUS_6_RX:
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(&avd_drift_config_controls[1],
					dai));
		break;
	}
	if (rc < 0)
		dev_err(dai->dev, "%s: err add config ctl, DAI = %s\n",
			__func__, dai->name);

	rc = msm_dai_q6_dai_add_route(dai);
	return rc;
}

static int msm_dai_q6_dai_remove(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_dai_data *dai_data;
	int rc;

	dai_data = dev_get_drvdata(dai->dev);

	/* If AFE port is still up, close it */
	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		pr_debug("%s: stop pseudo port:%d\n", __func__,  dai->id);
		rc = afe_close(dai->id); /* can block */
		if (rc < 0)
			dev_err(dai->dev, "fail to close AFE port\n");
		clear_bit(STATUS_PORT_STARTED, dai_data->status_mask);
	}
	kfree(dai_data);

	return 0;
}

static struct snd_soc_dai_driver msm_dai_q6_afe_rx_dai[] = {
	{
		.playback = {
			.stream_name = "AFE Playback",
			.aif_name = "PCM_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =	48000,
		},
		.ops = &msm_dai_q6_ops,
		.id = RT_PROXY_DAI_001_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			 .stream_name = "AFE-PROXY RX",
			 .rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			 SNDRV_PCM_RATE_16000,
			 .formats = SNDRV_PCM_FMTBIT_S16_LE |
			 SNDRV_PCM_FMTBIT_S24_LE,
			 .channels_min = 1,
			 .channels_max = 10,
			 .rate_min =     8000,
			 .rate_max =	48000,
		},
		.ops = &msm_dai_q6_ops,
		.id = RT_PROXY_DAI_002_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "AFE Playback1",
			.aif_name = "PCM_RX1",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_ops,
		.id = RT_PROXY_DAI_003_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
};

static struct snd_soc_dai_driver msm_dai_q6_afe_lb_tx_dai[] = {
	{
		.capture = {
			.stream_name = "AFE Loopback Capture",
			.aif_name = "AFE_LOOPBACK_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
			 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
			 SNDRV_PCM_RATE_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_3LE |
			SNDRV_PCM_FMTBIT_S32_LE ),
			.channels_min = 1,
			.channels_max = 8,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.id = AFE_LOOPBACK_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
};

static struct snd_soc_dai_driver msm_dai_q6_afe_tx_dai[] = {
	{
		.capture = {
			.stream_name = "AFE Capture",
			.aif_name = "PCM_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_ops,
		.id = RT_PROXY_DAI_002_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "AFE-PROXY TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_ops,
		.id = RT_PROXY_DAI_001_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
};

static struct snd_soc_dai_driver msm_dai_q6_afe_cap_dai = {
	.capture = {
		.stream_name = "AFE-PROXY TX1",
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
		SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 8,
		.rate_min =     8000,
		.rate_max =     48000,
	},
	.ops = &msm_dai_q6_ops,
	.id = RT_PROXY_DAI_003_TX,
	.probe = msm_dai_q6_dai_probe,
	.remove = msm_dai_q6_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_bt_sco_rx_dai = {
	.playback = {
		.stream_name = "Internal BT-SCO Playback",
		.aif_name = "INT_BT_SCO_RX",
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 1,
		.rate_max = 16000,
		.rate_min = 8000,
	},
	.ops = &msm_dai_q6_ops,
	.id = INT_BT_SCO_RX,
	.probe = msm_dai_q6_dai_probe,
	.remove = msm_dai_q6_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_bt_a2dp_rx_dai = {
	.playback = {
		.stream_name = "Internal BT-A2DP Playback",
		.aif_name = "INT_BT_A2DP_RX",
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 2,
		.rate_max = 48000,
		.rate_min = 48000,
	},
	.ops = &msm_dai_q6_ops,
	.id = INT_BT_A2DP_RX,
	.probe = msm_dai_q6_dai_probe,
	.remove = msm_dai_q6_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_bt_sco_tx_dai = {
	.capture = {
		.stream_name = "Internal BT-SCO Capture",
		.aif_name = "INT_BT_SCO_TX",
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 1,
		.rate_max = 16000,
		.rate_min = 8000,
	},
	.ops = &msm_dai_q6_ops,
	.id = INT_BT_SCO_TX,
	.probe = msm_dai_q6_dai_probe,
	.remove = msm_dai_q6_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_fm_rx_dai = {
	.playback = {
		.stream_name = "Internal FM Playback",
		.aif_name = "INT_FM_RX",
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
		SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 2,
		.channels_max = 2,
		.rate_max = 48000,
		.rate_min = 8000,
	},
	.ops = &msm_dai_q6_ops,
	.id = INT_FM_RX,
	.probe = msm_dai_q6_dai_probe,
	.remove = msm_dai_q6_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_fm_tx_dai = {
	.capture = {
		.stream_name = "Internal FM Capture",
		.aif_name = "INT_FM_TX",
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
		SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 2,
		.channels_max = 2,
		.rate_max = 48000,
		.rate_min = 8000,
	},
	.ops = &msm_dai_q6_ops,
	.id = INT_FM_TX,
	.probe = msm_dai_q6_dai_probe,
	.remove = msm_dai_q6_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_voc_playback_dai[] = {
	{
		.playback = {
			.stream_name = "Voice Farend Playback",
			.aif_name = "VOICE_PLAYBACK_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				 SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_ops,
		.id = VOICE_PLAYBACK_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Voice2 Farend Playback",
			.aif_name = "VOICE2_PLAYBACK_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				 SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_ops,
		.id = VOICE2_PLAYBACK_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
};

static struct snd_soc_dai_driver msm_dai_q6_incall_record_dai[] = {
	{
		.capture = {
			.stream_name = "Voice Uplink Capture",
			.aif_name = "INCALL_RECORD_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_ops,
		.id = VOICE_RECORD_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Voice Downlink Capture",
			.aif_name = "INCALL_RECORD_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_ops,
		.id = VOICE_RECORD_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
};

static struct snd_soc_dai_driver msm_dai_q6_proxy_tx_dai = {
	.capture = {
		.stream_name = "Proxy Capture",
		.aif_name = "PROXY_TX",
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 2,
		.rate_min =     8000,
		.rate_max =     48000,
	},
	.ops = &msm_dai_q6_ops,
	.id = RT_PROXY_PORT_002_TX,
	.probe = msm_dai_q6_dai_probe,
	.remove = msm_dai_q6_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_proxy_rx_dai = {
	.playback = {
		.stream_name = "Proxy Playback",
		.aif_name = "PROXY_RX",
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
		.channels_min = 1,
		.channels_max = 2,
		.rate_min =     8000,
		.rate_max =     48000,
	},
	.ops = &msm_dai_q6_ops,
	.id = RT_PROXY_PORT_002_RX,
	.probe = msm_dai_q6_dai_probe,
	.remove = msm_dai_q6_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_usb_rx_dai = {
	.playback = {
		.stream_name = "USB Audio Playback",
		.aif_name = "USB_AUDIO_RX",
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
			 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			 SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
			 SNDRV_PCM_RATE_384000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE,
		.channels_min = 1,
		.channels_max = 8,
		.rate_max = 384000,
		.rate_min = 8000,
	},
	.ops = &msm_dai_q6_ops,
	.id = AFE_PORT_ID_USB_RX,
	.probe = msm_dai_q6_dai_probe,
	.remove = msm_dai_q6_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_usb_tx_dai = {
	.capture = {
		.stream_name = "USB Audio Capture",
		.aif_name = "USB_AUDIO_TX",
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
			 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
			 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			 SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
			 SNDRV_PCM_RATE_384000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE,
		.channels_min = 1,
		.channels_max = 8,
		.rate_max = 384000,
		.rate_min = 8000,
	},
	.ops = &msm_dai_q6_ops,
	.id = AFE_PORT_ID_USB_TX,
	.probe = msm_dai_q6_dai_probe,
	.remove = msm_dai_q6_dai_remove,
};

static int msm_auxpcm_dev_probe(struct platform_device *pdev)
{
	struct msm_dai_q6_auxpcm_dai_data *dai_data;
	struct msm_dai_auxpcm_pdata *auxpcm_pdata;
	uint32_t val_array[RATE_MAX_NUM_OF_AUX_PCM_RATES];
	uint32_t val = 0;
	const char *intf_name;
	int rc = 0, i = 0, len = 0;
	const uint32_t *slot_mapping_array = NULL;
	u32 array_length = 0;

	dai_data = kzalloc(sizeof(struct msm_dai_q6_auxpcm_dai_data),
			   GFP_KERNEL);
	if (!dai_data)
		return -ENOMEM;

	rc = of_property_read_u32(pdev->dev.of_node,
				    "qcom,msm-dai-is-island-supported",
				    &dai_data->is_island_dai);
	if (rc)
		dev_dbg(&pdev->dev, "island supported entry not found\n");

	auxpcm_pdata = kzalloc(sizeof(struct msm_dai_auxpcm_pdata),
				GFP_KERNEL);

	if (!auxpcm_pdata) {
		dev_err(&pdev->dev, "Failed to allocate memory for platform data\n");
		goto fail_pdata_nomem;
	}

	dev_dbg(&pdev->dev, "%s: dev %pK, dai_data %pK, auxpcm_pdata %pK\n",
		__func__, &pdev->dev, dai_data, auxpcm_pdata);

	rc = of_property_read_u32_array(pdev->dev.of_node,
			"qcom,msm-cpudai-auxpcm-mode",
			val_array, RATE_MAX_NUM_OF_AUX_PCM_RATES);
	if (rc) {
		dev_err(&pdev->dev, "%s: qcom,msm-cpudai-auxpcm-mode missing in DT node\n",
			__func__);
		goto fail_invalid_dt;
	}
	auxpcm_pdata->mode_8k.mode = (u16)val_array[RATE_8KHZ];
	auxpcm_pdata->mode_16k.mode = (u16)val_array[RATE_16KHZ];

	rc = of_property_read_u32_array(pdev->dev.of_node,
			"qcom,msm-cpudai-auxpcm-sync",
			val_array, RATE_MAX_NUM_OF_AUX_PCM_RATES);
	if (rc) {
		dev_err(&pdev->dev, "%s: qcom,msm-cpudai-auxpcm-sync missing in DT node\n",
			__func__);
		goto fail_invalid_dt;
	}
	auxpcm_pdata->mode_8k.sync = (u16)val_array[RATE_8KHZ];
	auxpcm_pdata->mode_16k.sync = (u16)val_array[RATE_16KHZ];

	rc = of_property_read_u32_array(pdev->dev.of_node,
			"qcom,msm-cpudai-auxpcm-frame",
			val_array, RATE_MAX_NUM_OF_AUX_PCM_RATES);

	if (rc) {
		dev_err(&pdev->dev, "%s: qcom,msm-cpudai-auxpcm-frame missing in DT node\n",
			__func__);
		goto fail_invalid_dt;
	}
	auxpcm_pdata->mode_8k.frame = (u16)val_array[RATE_8KHZ];
	auxpcm_pdata->mode_16k.frame = (u16)val_array[RATE_16KHZ];

	rc = of_property_read_u32_array(pdev->dev.of_node,
			"qcom,msm-cpudai-auxpcm-quant",
			val_array, RATE_MAX_NUM_OF_AUX_PCM_RATES);
	if (rc) {
		dev_err(&pdev->dev, "%s: qcom,msm-cpudai-auxpcm-quant missing in DT node\n",
			__func__);
		goto fail_invalid_dt;
	}
	auxpcm_pdata->mode_8k.quant = (u16)val_array[RATE_8KHZ];
	auxpcm_pdata->mode_16k.quant = (u16)val_array[RATE_16KHZ];

	rc = of_property_read_u32_array(pdev->dev.of_node,
			"qcom,msm-cpudai-auxpcm-num-slots",
			val_array, RATE_MAX_NUM_OF_AUX_PCM_RATES);
	if (rc) {
		dev_err(&pdev->dev, "%s: qcom,msm-cpudai-auxpcm-num-slots missing in DT node\n",
			__func__);
		goto fail_invalid_dt;
	}
	auxpcm_pdata->mode_8k.num_slots = (u16)val_array[RATE_8KHZ];

	if (auxpcm_pdata->mode_8k.num_slots >
	    msm_dai_q6_max_num_slot(auxpcm_pdata->mode_8k.frame)) {
		dev_err(&pdev->dev, "%s Max slots %d greater than DT node %d\n",
			 __func__,
			msm_dai_q6_max_num_slot(auxpcm_pdata->mode_8k.frame),
			auxpcm_pdata->mode_8k.num_slots);
		rc = -EINVAL;
		goto fail_invalid_dt;
	}
	auxpcm_pdata->mode_16k.num_slots = (u16)val_array[RATE_16KHZ];

	if (auxpcm_pdata->mode_16k.num_slots >
	    msm_dai_q6_max_num_slot(auxpcm_pdata->mode_16k.frame)) {
		dev_err(&pdev->dev, "%s Max slots %d greater than DT node %d\n",
			__func__,
			msm_dai_q6_max_num_slot(auxpcm_pdata->mode_16k.frame),
			auxpcm_pdata->mode_16k.num_slots);
		rc = -EINVAL;
		goto fail_invalid_dt;
	}

	slot_mapping_array = of_get_property(pdev->dev.of_node,
				"qcom,msm-cpudai-auxpcm-slot-mapping", &len);

	if (slot_mapping_array == NULL) {
		dev_err(&pdev->dev, "%s slot_mapping_array is not valid\n",
			__func__);
		rc = -EINVAL;
		goto fail_invalid_dt;
	}

	array_length = auxpcm_pdata->mode_8k.num_slots +
		       auxpcm_pdata->mode_16k.num_slots;

	if (len != sizeof(uint32_t) * array_length) {
		dev_err(&pdev->dev, "%s Length is %d and expected is %zd\n",
			__func__, len, sizeof(uint32_t) * array_length);
		rc = -EINVAL;
		goto fail_invalid_dt;
	}

	auxpcm_pdata->mode_8k.slot_mapping =
					kzalloc(sizeof(uint16_t) *
					    auxpcm_pdata->mode_8k.num_slots,
					    GFP_KERNEL);
	if (!auxpcm_pdata->mode_8k.slot_mapping) {
		dev_err(&pdev->dev, "%s No mem for mode_8k slot mapping\n",
			__func__);
		rc = -ENOMEM;
		goto fail_invalid_dt;
	}

	for (i = 0; i < auxpcm_pdata->mode_8k.num_slots; i++)
		auxpcm_pdata->mode_8k.slot_mapping[i] =
				(u16)be32_to_cpu(slot_mapping_array[i]);

	auxpcm_pdata->mode_16k.slot_mapping =
					kzalloc(sizeof(uint16_t) *
					     auxpcm_pdata->mode_16k.num_slots,
					     GFP_KERNEL);

	if (!auxpcm_pdata->mode_16k.slot_mapping) {
		dev_err(&pdev->dev, "%s No mem for mode_16k slot mapping\n",
			__func__);
		rc = -ENOMEM;
		goto fail_invalid_16k_slot_mapping;
	}

	for (i = 0; i < auxpcm_pdata->mode_16k.num_slots; i++)
		auxpcm_pdata->mode_16k.slot_mapping[i] =
			(u16)be32_to_cpu(slot_mapping_array[i +
					auxpcm_pdata->mode_8k.num_slots]);

	rc = of_property_read_u32_array(pdev->dev.of_node,
			"qcom,msm-cpudai-auxpcm-data",
			val_array, RATE_MAX_NUM_OF_AUX_PCM_RATES);
	if (rc) {
		dev_err(&pdev->dev, "%s: qcom,msm-cpudai-auxpcm-data missing in DT node\n",
			__func__);
		goto fail_invalid_dt1;
	}
	auxpcm_pdata->mode_8k.data = (u16)val_array[RATE_8KHZ];
	auxpcm_pdata->mode_16k.data = (u16)val_array[RATE_16KHZ];

	rc = of_property_read_u32_array(pdev->dev.of_node,
			"qcom,msm-cpudai-auxpcm-pcm-clk-rate",
			val_array, RATE_MAX_NUM_OF_AUX_PCM_RATES);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: qcom,msm-cpudai-auxpcm-pcm-clk-rate missing in DT\n",
			__func__);
		goto fail_invalid_dt1;
	}
	auxpcm_pdata->mode_8k.pcm_clk_rate = (int)val_array[RATE_8KHZ];
	auxpcm_pdata->mode_16k.pcm_clk_rate = (int)val_array[RATE_16KHZ];

	rc = of_property_read_string(pdev->dev.of_node,
			"qcom,msm-auxpcm-interface", &intf_name);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: qcom,msm-auxpcm-interface missing in DT node\n",
			__func__);
		goto fail_nodev_intf;
	}

	if (!strcmp(intf_name, "primary")) {
		dai_data->rx_pid = AFE_PORT_ID_PRIMARY_PCM_RX;
		dai_data->tx_pid = AFE_PORT_ID_PRIMARY_PCM_TX;
		pdev->id = MSM_DAI_PRI_AUXPCM_DT_DEV_ID;
		i = 0;
	} else if (!strcmp(intf_name, "secondary")) {
		dai_data->rx_pid = AFE_PORT_ID_SECONDARY_PCM_RX;
		dai_data->tx_pid = AFE_PORT_ID_SECONDARY_PCM_TX;
		pdev->id = MSM_DAI_SEC_AUXPCM_DT_DEV_ID;
		i = 1;
	} else if (!strcmp(intf_name, "tertiary")) {
		dai_data->rx_pid = AFE_PORT_ID_TERTIARY_PCM_RX;
		dai_data->tx_pid = AFE_PORT_ID_TERTIARY_PCM_TX;
		pdev->id = MSM_DAI_TERT_AUXPCM_DT_DEV_ID;
		i = 2;
	} else if (!strcmp(intf_name, "quaternary")) {
		dai_data->rx_pid = AFE_PORT_ID_QUATERNARY_PCM_RX;
		dai_data->tx_pid = AFE_PORT_ID_QUATERNARY_PCM_TX;
		pdev->id = MSM_DAI_QUAT_AUXPCM_DT_DEV_ID;
		i = 3;
	} else if (!strcmp(intf_name, "quinary")) {
		dai_data->rx_pid = AFE_PORT_ID_QUINARY_PCM_RX;
		dai_data->tx_pid = AFE_PORT_ID_QUINARY_PCM_TX;
		pdev->id = MSM_DAI_QUIN_AUXPCM_DT_DEV_ID;
		i = 4;
	} else if (!strcmp(intf_name, "senary")) {
		dai_data->rx_pid = AFE_PORT_ID_SENARY_PCM_RX;
		dai_data->tx_pid = AFE_PORT_ID_SENARY_PCM_TX;
		pdev->id = MSM_DAI_SEN_AUXPCM_DT_DEV_ID;
		i = 5;
	} else {
		dev_err(&pdev->dev, "%s: invalid DT intf name %s\n",
			__func__, intf_name);
		goto fail_invalid_intf;
	}
	rc = of_property_read_u32(pdev->dev.of_node,
				  "qcom,msm-cpudai-afe-clk-ver", &val);
	if (rc)
		dai_data->afe_clk_ver = AFE_CLK_VERSION_V1;
	else
		dai_data->afe_clk_ver = val;

	mutex_init(&dai_data->rlock);
	dev_dbg(&pdev->dev, "dev name %s\n", dev_name(&pdev->dev));

	dev_set_drvdata(&pdev->dev, dai_data);
	pdev->dev.platform_data = (void *) auxpcm_pdata;

	rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_aux_pcm_dai_component,
			&msm_dai_q6_aux_pcm_dai[i], 1);
	if (rc) {
		dev_err(&pdev->dev, "%s: auxpcm dai reg failed, rc=%d\n",
				__func__, rc);
		goto fail_reg_dai;
	}

	return rc;

fail_reg_dai:
fail_invalid_intf:
fail_nodev_intf:
fail_invalid_dt1:
	kfree(auxpcm_pdata->mode_16k.slot_mapping);
fail_invalid_16k_slot_mapping:
	kfree(auxpcm_pdata->mode_8k.slot_mapping);
fail_invalid_dt:
	kfree(auxpcm_pdata);
fail_pdata_nomem:
	kfree(dai_data);
	return rc;
}

static int msm_auxpcm_dev_remove(struct platform_device *pdev)
{
	struct msm_dai_q6_auxpcm_dai_data *dai_data;

	dai_data = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);

	mutex_destroy(&dai_data->rlock);
	kfree(dai_data);
	kfree(pdev->dev.platform_data);

	return 0;
}

static const struct of_device_id msm_auxpcm_dev_dt_match[] = {
	{ .compatible = "qcom,msm-auxpcm-dev", },
	{}
};


static struct platform_driver msm_auxpcm_dev_driver = {
	.probe  = msm_auxpcm_dev_probe,
	.remove = msm_auxpcm_dev_remove,
	.driver = {
		.name = "msm-auxpcm-dev",
		.owner = THIS_MODULE,
		.of_match_table = msm_auxpcm_dev_dt_match,
		.suppress_bind_attrs = true,
	},
};

static struct snd_soc_dai_driver msm_dai_q6_slimbus_rx_dai[] = {
	{
		.playback = {
			.stream_name = "Slimbus Playback",
			.aif_name = "SLIMBUS_0_RX",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = DAI_FORMATS_S16_S24_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_dai_slimbus_0_rx_ops,
		.id = SLIMBUS_0_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Slimbus1 Playback",
			.aif_name = "SLIMBUS_1_RX",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = DAI_FORMATS_S16_S24_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_1_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Slimbus2 Playback",
			.aif_name = "SLIMBUS_2_RX",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = DAI_FORMATS_S16_S24_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_2_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Slimbus3 Playback",
			.aif_name = "SLIMBUS_3_RX",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = DAI_FORMATS_S16_S24_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_3_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Slimbus4 Playback",
			.aif_name = "SLIMBUS_4_RX",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = DAI_FORMATS_S16_S24_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_4_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Slimbus6 Playback",
			.aif_name = "SLIMBUS_6_RX",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = DAI_FORMATS_S16_S24_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_6_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Slimbus5 Playback",
			.aif_name = "SLIMBUS_5_RX",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = DAI_FORMATS_S16_S24_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_5_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Slimbus7 Playback",
			.aif_name = "SLIMBUS_7_RX",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = DAI_FORMATS_S16_S24_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_7_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Slimbus8 Playback",
			.aif_name = "SLIMBUS_8_RX",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = DAI_FORMATS_S16_S24_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_8_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Slimbus9 Playback",
			.aif_name = "SLIMBUS_9_RX",
			.rates = SNDRV_PCM_RATE_8000_384000,
			.formats = DAI_FORMATS_S16_S24_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_9_RX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
};

static struct snd_soc_dai_driver msm_dai_q6_slimbus_tx_dai[] = {
	{
		.capture = {
			.stream_name = "Slimbus Capture",
			.aif_name = "SLIMBUS_0_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 192000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_0_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Slimbus1 Capture",
			.aif_name = "SLIMBUS_1_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 192000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_1_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Slimbus2 Capture",
			.aif_name = "SLIMBUS_2_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 192000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_2_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Slimbus3 Capture",
			.aif_name = "SLIMBUS_3_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 2,
			.channels_max = 4,
			.rate_min = 8000,
			.rate_max = 192000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_3_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Slimbus4 Capture",
			.aif_name = "SLIMBUS_4_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 2,
			.channels_max = 4,
			.rate_min = 8000,
			.rate_max = 192000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_4_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Slimbus5 Capture",
			.aif_name = "SLIMBUS_5_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 192000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_5_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Slimbus6 Capture",
			.aif_name = "SLIMBUS_6_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 192000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_6_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Slimbus7 Capture",
			.aif_name = "SLIMBUS_7_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 192000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_7_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Slimbus8 Capture",
			.aif_name = "SLIMBUS_8_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 192000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_8_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
	{
		.capture = {
			.stream_name = "Slimbus9 Capture",
			.aif_name = "SLIMBUS_9_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 192000,
		},
		.ops = &msm_dai_q6_ops,
		.id = SLIMBUS_9_TX,
		.probe = msm_dai_q6_dai_probe,
		.remove = msm_dai_q6_dai_remove,
	},
};

static int msm_dai_q6_mi2s_format_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	int value = ucontrol->value.integer.value[0];

	dai_data->port_config.i2s.data_format = value;
	pr_debug("%s: value = %d, channel = %d, line = %d\n",
		 __func__, value, dai_data->port_config.i2s.mono_stereo,
		 dai_data->port_config.i2s.channel_mode);
	return 0;
}

static int msm_dai_q6_mi2s_format_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	ucontrol->value.integer.value[0] =
		dai_data->port_config.i2s.data_format;
	return 0;
}

static int msm_dai_q6_mi2s_vi_feed_mono_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;
	int value = ucontrol->value.integer.value[0];

	dai_data->vi_feed_mono = value;
	pr_debug("%s: value = %d\n", __func__, value);
	return 0;
}

static int msm_dai_q6_mi2s_vi_feed_mono_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_dai_data *dai_data = kcontrol->private_data;

	ucontrol->value.integer.value[0] = dai_data->vi_feed_mono;
	return 0;
}

static const struct snd_kcontrol_new mi2s_config_controls[] = {
	SOC_ENUM_EXT("PRI MI2S RX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("SEC MI2S RX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("TERT MI2S RX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("QUAT MI2S RX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("QUIN MI2S RX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("SENARY MI2S RX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("PRI MI2S TX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("SEC MI2S TX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("TERT MI2S TX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("QUAT MI2S TX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("QUIN MI2S TX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("SENARY MI2S TX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
	SOC_ENUM_EXT("INT5 MI2S TX Format", mi2s_config_enum[0],
		     msm_dai_q6_mi2s_format_get,
		     msm_dai_q6_mi2s_format_put),
};

static const struct snd_kcontrol_new mi2s_vi_feed_controls[] = {
	SOC_ENUM_EXT("INT5 MI2S VI MONO", mi2s_config_enum[1],
		     msm_dai_q6_mi2s_vi_feed_mono_get,
		     msm_dai_q6_mi2s_vi_feed_mono_put),
};

static int msm_dai_q6_dai_mi2s_probe(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_mi2s_dai_data *mi2s_dai_data =
			dev_get_drvdata(dai->dev);
	struct msm_mi2s_pdata *mi2s_pdata =
			(struct msm_mi2s_pdata *) dai->dev->platform_data;
	struct snd_kcontrol *kcontrol = NULL;
	int rc = 0;
	const struct snd_kcontrol_new *ctrl = NULL;
	const struct snd_kcontrol_new *vi_feed_ctrl = NULL;
	u16 dai_id = 0;

	dai->id = mi2s_pdata->intf_id;
	if (mi2s_dai_data->mi2s_dai.mi2s_dai_data.port_config.i2s.channel_mode) {
		if (dai->id == MSM_PRIM_MI2S_RX)
			ctrl = &mi2s_config_controls[0];
		if (dai->id == MSM_SEC_MI2S_RX)
			ctrl = &mi2s_config_controls[1];
		if (dai->id == MSM_TERT_MI2S_RX)
			ctrl = &mi2s_config_controls[2];
		if (dai->id == MSM_QUAT_MI2S_RX)
			ctrl = &mi2s_config_controls[3];
		if (dai->id == MSM_QUIN_MI2S_RX)
			ctrl = &mi2s_config_controls[4];
		if (dai->id == MSM_SENARY_MI2S_RX)
			ctrl = &mi2s_config_controls[5];
		if (dai->id == MSM_PRIM_MI2S_TX)
			ctrl = &mi2s_config_controls[6];
		if (dai->id == MSM_SEC_MI2S_TX)
			ctrl = &mi2s_config_controls[7];
		if (dai->id == MSM_TERT_MI2S_TX)
			ctrl = &mi2s_config_controls[8];
		if (dai->id == MSM_QUAT_MI2S_TX)
			ctrl = &mi2s_config_controls[9];
		if (dai->id == MSM_QUIN_MI2S_TX)
			ctrl = &mi2s_config_controls[10];
		if (dai->id == MSM_SENARY_MI2S_TX)
			ctrl = &mi2s_config_controls[11];
		if (dai->id == MSM_INT5_MI2S_TX)
			ctrl = &mi2s_config_controls[12];
	}

	if (ctrl) {
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(ctrl,
				&mi2s_dai_data->mi2s_dai.mi2s_dai_data));
		if (rc < 0) {
			if (kcontrol)
				snd_ctl_remove(dai->component->card->snd_card,
						kcontrol);
			dev_err(dai->dev, "%s: err add MI2S fmt ctl DAI = %s\n",
				__func__, dai->name);
		}
	}

	if (dai->id == MSM_INT5_MI2S_TX)
		vi_feed_ctrl = &mi2s_vi_feed_controls[0];

	if (vi_feed_ctrl) {
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(vi_feed_ctrl,
				&mi2s_dai_data->mi2s_dai.mi2s_dai_data));

		if (rc < 0) {
			dev_err(dai->dev, "%s: err add TX vi feed channel ctl DAI = %s\n",
				__func__, dai->name);
		}
	}

	if (mi2s_dai_data->is_island_dai) {
		msm_mi2s_get_port_id(dai->id, SNDRV_PCM_STREAM_CAPTURE,
				     &dai_id);
		rc = msm_dai_q6_add_island_mx_ctls(
						dai->component->card->snd_card,
						dai->name, dai_id,
						(void *)mi2s_dai_data);
	}

	rc = msm_dai_q6_dai_add_route(dai);
	return rc;
}


static int msm_dai_q6_dai_mi2s_remove(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_mi2s_dai_data *mi2s_dai_data =
		dev_get_drvdata(dai->dev);
	int rc;
	bool rx_port_en = false;

	/* If AFE port is still up, close it */
	if (strnstr(dev_name(dai->dev), "rx", strlen(dev_name(dai->dev)))
	    != NULL)
		rx_port_en = true;
	if (test_bit(STATUS_PORT_STARTED,
		     mi2s_dai_data->mi2s_dai.mi2s_dai_data.status_mask)) {
		if (rx_port_en)
			rc = afe_close(MI2S_RX); /* can block */
		else
			rc = afe_close(MI2S_TX); /* can block */
		if (rc < 0)
			dev_err(dai->dev, "fail to close MI2S_RX port\n");
		clear_bit(STATUS_PORT_STARTED,
			  mi2s_dai_data->mi2s_dai.mi2s_dai_data.status_mask);
	}
	return 0;
}

static int msm_dai_q6_mi2s_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{

	return 0;
}

static int msm_mi2s_get_port_id(u32 mi2s_id, int stream, u16 *port_id)
{
	int ret = 0;

	switch (stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		switch (mi2s_id) {
		case MSM_PRIM_MI2S_RX:
			*port_id = AFE_PORT_ID_PRIMARY_MI2S_RX;
			break;
		case MSM_SEC_MI2S_RX:
			*port_id = AFE_PORT_ID_SECONDARY_MI2S_RX;
			break;
		case MSM_TERT_MI2S_RX:
			*port_id = AFE_PORT_ID_TERTIARY_MI2S_RX;
			break;
		case MSM_QUAT_MI2S_RX:
			*port_id = AFE_PORT_ID_QUATERNARY_MI2S_RX;
			break;
		case MSM_SEC_MI2S_SD1:
			*port_id = AFE_PORT_ID_SECONDARY_MI2S_RX_SD1;
			break;
		case MSM_QUIN_MI2S_RX:
			*port_id = AFE_PORT_ID_QUINARY_MI2S_RX;
			break;
		case MSM_SENARY_MI2S_RX:
			*port_id = AFE_PORT_ID_SENARY_MI2S_RX;
			break;
		case MSM_INT0_MI2S_RX:
			*port_id = AFE_PORT_ID_INT0_MI2S_RX;
			break;
		case MSM_INT1_MI2S_RX:
			*port_id = AFE_PORT_ID_INT1_MI2S_RX;
			break;
		case MSM_INT2_MI2S_RX:
			*port_id = AFE_PORT_ID_INT2_MI2S_RX;
			break;
		case MSM_INT3_MI2S_RX:
			*port_id = AFE_PORT_ID_INT3_MI2S_RX;
			break;
		case MSM_INT4_MI2S_RX:
			*port_id = AFE_PORT_ID_INT4_MI2S_RX;
			break;
		case MSM_INT5_MI2S_RX:
			*port_id = AFE_PORT_ID_INT5_MI2S_RX;
			break;
		case MSM_INT6_MI2S_RX:
			*port_id = AFE_PORT_ID_INT6_MI2S_RX;
			break;
		default:
			pr_err("%s: playback err id 0x%x\n",
				__func__, mi2s_id);
			ret = -1;
			break;
		}
	break;
	case SNDRV_PCM_STREAM_CAPTURE:
		switch (mi2s_id) {
		case MSM_PRIM_MI2S_TX:
			*port_id = AFE_PORT_ID_PRIMARY_MI2S_TX;
			break;
		case MSM_SEC_MI2S_TX:
			*port_id = AFE_PORT_ID_SECONDARY_MI2S_TX;
			break;
		case MSM_TERT_MI2S_TX:
			*port_id = AFE_PORT_ID_TERTIARY_MI2S_TX;
			break;
		case MSM_QUAT_MI2S_TX:
			*port_id = AFE_PORT_ID_QUATERNARY_MI2S_TX;
			break;
		case MSM_QUIN_MI2S_TX:
			*port_id = AFE_PORT_ID_QUINARY_MI2S_TX;
			break;
		case MSM_SENARY_MI2S_TX:
			*port_id = AFE_PORT_ID_SENARY_MI2S_TX;
			break;
		case MSM_INT0_MI2S_TX:
			*port_id = AFE_PORT_ID_INT0_MI2S_TX;
			break;
		case MSM_INT1_MI2S_TX:
			*port_id = AFE_PORT_ID_INT1_MI2S_TX;
			break;
		case MSM_INT2_MI2S_TX:
			*port_id = AFE_PORT_ID_INT2_MI2S_TX;
			break;
		case MSM_INT3_MI2S_TX:
			*port_id = AFE_PORT_ID_INT3_MI2S_TX;
			break;
		case MSM_INT4_MI2S_TX:
			*port_id = AFE_PORT_ID_INT4_MI2S_TX;
			break;
		case MSM_INT5_MI2S_TX:
			*port_id = AFE_PORT_ID_INT5_MI2S_TX;
			break;
		case MSM_INT6_MI2S_TX:
			*port_id = AFE_PORT_ID_INT6_MI2S_TX;
			break;
		default:
			pr_err("%s: capture err id 0x%x\n", __func__, mi2s_id);
			ret = -1;
			break;
		}
	break;
	default:
		pr_err("%s: default err %d\n", __func__, stream);
		ret = -1;
	break;
	}
	pr_debug("%s: port_id = 0x%x\n", __func__, *port_id);
	return ret;
}

static int msm_dai_q6_mi2s_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_mi2s_dai_data *mi2s_dai_data =
		dev_get_drvdata(dai->dev);
	struct msm_dai_q6_dai_data *dai_data =
		 &mi2s_dai_data->mi2s_dai.mi2s_dai_data;
	u16 port_id = 0;
	int rc = 0;

	if (msm_mi2s_get_port_id(dai->id, substream->stream,
				 &port_id) != 0) {
		dev_err(dai->dev, "%s: Invalid Port ID 0x%x\n",
				__func__, port_id);
		return -EINVAL;
	}

	dev_dbg(dai->dev, "%s: dai id %d, afe port id = 0x%x\n"
		"dai_data->channels = %u sample_rate = %u\n", __func__,
		dai->id, port_id, dai_data->channels, dai_data->rate);

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		/* PORT START should be set if prepare called
		 * in active state.
		 */
		rc = afe_port_start(port_id, &dai_data->port_config,
				    dai_data->rate);
		if (rc < 0)
			dev_err(dai->dev, "fail to open AFE port 0x%x\n",
				dai->id);
		else
			set_bit(STATUS_PORT_STARTED,
				dai_data->status_mask);
	}
	if (!test_bit(STATUS_PORT_STARTED, dai_data->hwfree_status)) {
		set_bit(STATUS_PORT_STARTED, dai_data->hwfree_status);
		dev_dbg(dai->dev, "%s: set hwfree_status to started\n",
				__func__);
	}
	return rc;
}

static int msm_dai_q6_mi2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct msm_dai_q6_mi2s_dai_data *mi2s_dai_data =
		dev_get_drvdata(dai->dev);
	struct msm_dai_q6_mi2s_dai_config *mi2s_dai_config =
		&mi2s_dai_data->mi2s_dai;
	struct msm_dai_q6_dai_data *dai_data = &mi2s_dai_config->mi2s_dai_data;
	struct afe_param_id_i2s_cfg *i2s = &dai_data->port_config.i2s;

	dai_data->channels = params_channels(params);
	switch (dai_data->channels) {
	case 15:
	case 16:
		switch (mi2s_dai_config->pdata_mi2s_lines) {
		case AFE_PORT_I2S_16CHS:
			dai_data->port_config.i2s.channel_mode
				= AFE_PORT_I2S_16CHS;
			break;
		default:
			goto error_invalid_data;
		};
		break;
	case 13:
	case 14:
		switch (mi2s_dai_config->pdata_mi2s_lines) {
		case AFE_PORT_I2S_14CHS:
		case AFE_PORT_I2S_16CHS:
			dai_data->port_config.i2s.channel_mode
				= AFE_PORT_I2S_14CHS;
			break;
		default:
			goto error_invalid_data;
		};
		break;
	case 11:
	case 12:
		switch (mi2s_dai_config->pdata_mi2s_lines) {
		case AFE_PORT_I2S_12CHS:
		case AFE_PORT_I2S_14CHS:
		case AFE_PORT_I2S_16CHS:
			dai_data->port_config.i2s.channel_mode
				= AFE_PORT_I2S_12CHS;
			break;
		default:
			goto error_invalid_data;
		};
		break;
	case 9:
	case 10:
		switch (mi2s_dai_config->pdata_mi2s_lines) {
		case AFE_PORT_I2S_10CHS:
		case AFE_PORT_I2S_12CHS:
		case AFE_PORT_I2S_14CHS:
		case AFE_PORT_I2S_16CHS:
			dai_data->port_config.i2s.channel_mode
				= AFE_PORT_I2S_10CHS;
			break;
		default:
			goto error_invalid_data;
		};
		break;
	case 8:
	case 7:
		if (mi2s_dai_config->pdata_mi2s_lines < AFE_PORT_I2S_8CHS)
			goto error_invalid_data;
		else
			if (mi2s_dai_config->pdata_mi2s_lines
					== AFE_PORT_I2S_8CHS_2)
				dai_data->port_config.i2s.channel_mode =
						AFE_PORT_I2S_8CHS_2;
			else
				dai_data->port_config.i2s.channel_mode =
						AFE_PORT_I2S_8CHS;
		break;
	case 6:
	case 5:
		if (mi2s_dai_config->pdata_mi2s_lines < AFE_PORT_I2S_6CHS)
			goto error_invalid_data;
		dai_data->port_config.i2s.channel_mode = AFE_PORT_I2S_6CHS;
		break;
	case 4:
	case 3:
		switch (mi2s_dai_config->pdata_mi2s_lines) {
		case AFE_PORT_I2S_SD0:
		case AFE_PORT_I2S_SD1:
		case AFE_PORT_I2S_SD2:
		case AFE_PORT_I2S_SD3:
		case AFE_PORT_I2S_SD4:
		case AFE_PORT_I2S_SD5:
		case AFE_PORT_I2S_SD6:
		case AFE_PORT_I2S_SD7:
			goto error_invalid_data;
			break;
		case AFE_PORT_I2S_QUAD01:
		case AFE_PORT_I2S_QUAD23:
		case AFE_PORT_I2S_QUAD45:
		case AFE_PORT_I2S_QUAD67:
			dai_data->port_config.i2s.channel_mode =
				mi2s_dai_config->pdata_mi2s_lines;
			break;
		case AFE_PORT_I2S_8CHS_2:
			dai_data->port_config.i2s.channel_mode =
					AFE_PORT_I2S_QUAD45;
			break;
		default:
			dai_data->port_config.i2s.channel_mode =
					AFE_PORT_I2S_QUAD01;
			break;
		};
		break;
	case 2:
	case 1:
		if (mi2s_dai_config->pdata_mi2s_lines < AFE_PORT_I2S_SD0)
			goto error_invalid_data;
		switch (mi2s_dai_config->pdata_mi2s_lines) {
		case AFE_PORT_I2S_SD0:
		case AFE_PORT_I2S_SD1:
		case AFE_PORT_I2S_SD2:
		case AFE_PORT_I2S_SD3:
		case AFE_PORT_I2S_SD4:
		case AFE_PORT_I2S_SD5:
		case AFE_PORT_I2S_SD6:
		case AFE_PORT_I2S_SD7:
			dai_data->port_config.i2s.channel_mode =
				mi2s_dai_config->pdata_mi2s_lines;
			break;
		case AFE_PORT_I2S_QUAD01:
		case AFE_PORT_I2S_6CHS:
		case AFE_PORT_I2S_8CHS:
		case AFE_PORT_I2S_10CHS:
		case AFE_PORT_I2S_12CHS:
		case AFE_PORT_I2S_14CHS:
		case AFE_PORT_I2S_16CHS:
			if (dai_data->vi_feed_mono == SPKR_1)
				dai_data->port_config.i2s.channel_mode =
							AFE_PORT_I2S_SD0;
			else
				dai_data->port_config.i2s.channel_mode =
							AFE_PORT_I2S_SD1;
			break;
		case AFE_PORT_I2S_QUAD23:
			dai_data->port_config.i2s.channel_mode =
						AFE_PORT_I2S_SD2;
			break;
		case AFE_PORT_I2S_QUAD45:
			dai_data->port_config.i2s.channel_mode =
						AFE_PORT_I2S_SD4;
			break;
		case AFE_PORT_I2S_QUAD67:
			dai_data->port_config.i2s.channel_mode =
						AFE_PORT_I2S_SD6;
			break;
		}
		if (dai_data->channels == 2)
			dai_data->port_config.i2s.mono_stereo =
						MSM_AFE_CH_STEREO;
		else
			dai_data->port_config.i2s.mono_stereo = MSM_AFE_MONO;
		break;
	default:
		pr_err("%s: default err channels %d\n",
			__func__, dai_data->channels);
		goto error_invalid_data;
	}
	dai_data->rate = params_rate(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_SPECIAL:
		dai_data->port_config.i2s.bit_width = 16;
		dai_data->bitwidth = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		dai_data->port_config.i2s.bit_width = 24;
		dai_data->bitwidth = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		dai_data->port_config.i2s.bit_width = 32;
		dai_data->bitwidth = 32;
		break;
	default:
		pr_err("%s: format %d\n",
			__func__, params_format(params));
		return -EINVAL;
	}

	dai_data->port_config.i2s.i2s_cfg_minor_version =
			AFE_API_VERSION_I2S_CONFIG;
	dai_data->port_config.i2s.sample_rate = dai_data->rate;

	dev_dbg(dai->dev, "%s: dai id %d dai_data->channels = %d\n"
		"sample_rate = %u i2s_cfg_minor_version = 0x%x\n"
		"bit_width = %hu  channel_mode = 0x%x mono_stereo = %#x\n"
		"ws_src = 0x%x sample_rate = %u data_format = 0x%x\n"
		"reserved = %u\n", __func__, dai->id, dai_data->channels,
		dai_data->rate, i2s->i2s_cfg_minor_version, i2s->bit_width,
		i2s->channel_mode, i2s->mono_stereo, i2s->ws_src,
		i2s->sample_rate, i2s->data_format, i2s->reserved);

	return 0;

error_invalid_data:
	pr_err("%s: dai_data->channels = %d channel_mode = %d\n", __func__,
		 dai_data->channels, dai_data->port_config.i2s.channel_mode);
	return -EINVAL;
}


static int msm_dai_q6_mi2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct msm_dai_q6_mi2s_dai_data *mi2s_dai_data =
	dev_get_drvdata(dai->dev);

	if (test_bit(STATUS_PORT_STARTED,
	    mi2s_dai_data->mi2s_dai.mi2s_dai_data.status_mask)) {
		dev_err(dai->dev, "%s: err chg i2s mode while dai running",
			__func__);
		return -EPERM;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
	case SND_SOC_DAIFMT_CBM_CFS:
		mi2s_dai_data->mi2s_dai.mi2s_dai_data.port_config.i2s.ws_src = 1;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		mi2s_dai_data->mi2s_dai.mi2s_dai_data.port_config.i2s.ws_src = 0;
		break;
	default:
		pr_err("%s: fmt %d\n",
			__func__, fmt & SND_SOC_DAIFMT_MASTER_MASK);
		return -EINVAL;
	}

	return 0;
}

static int msm_dai_q6_mi2s_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_mi2s_dai_data *mi2s_dai_data =
			dev_get_drvdata(dai->dev);
	struct msm_dai_q6_dai_data *dai_data =
		 &mi2s_dai_data->mi2s_dai.mi2s_dai_data;

	if (test_bit(STATUS_PORT_STARTED, dai_data->hwfree_status)) {
		clear_bit(STATUS_PORT_STARTED, dai_data->hwfree_status);
		dev_dbg(dai->dev, "%s: clear hwfree_status\n", __func__);
	}
	return 0;
}

static void msm_dai_q6_mi2s_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct msm_dai_q6_mi2s_dai_data *mi2s_dai_data =
			dev_get_drvdata(dai->dev);
	struct msm_dai_q6_dai_data *dai_data =
		 &mi2s_dai_data->mi2s_dai.mi2s_dai_data;
	 u16 port_id = 0;
	int rc = 0;

	if (msm_mi2s_get_port_id(dai->id, substream->stream,
				 &port_id) != 0) {
		dev_err(dai->dev, "%s: Invalid Port ID 0x%x\n",
				__func__, port_id);
	}

	dev_dbg(dai->dev, "%s: closing afe port id = 0x%x\n",
			__func__, port_id);

	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		rc = afe_close(port_id);
		if (rc < 0)
			dev_err(dai->dev, "fail to close AFE port\n");
		clear_bit(STATUS_PORT_STARTED, dai_data->status_mask);
	}
	if (test_bit(STATUS_PORT_STARTED, dai_data->hwfree_status))
		clear_bit(STATUS_PORT_STARTED, dai_data->hwfree_status);
}

static struct snd_soc_dai_ops msm_dai_q6_mi2s_ops = {
	.startup	= msm_dai_q6_mi2s_startup,
	.prepare	= msm_dai_q6_mi2s_prepare,
	.hw_params	= msm_dai_q6_mi2s_hw_params,
	.hw_free	= msm_dai_q6_mi2s_hw_free,
	.set_fmt	= msm_dai_q6_mi2s_set_fmt,
	.shutdown	= msm_dai_q6_mi2s_shutdown,
};

/* Channel min and max are initialized base on platform data */
static struct snd_soc_dai_driver msm_dai_q6_mi2s_dai[] = {
	{
		.playback = {
			.stream_name = "Primary MI2S Playback",
			.aif_name = "PRI_MI2S_RX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.rate_min =     8000,
			.rate_max =     384000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Primary MI2S_RX",
		.id = MSM_PRIM_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "Primary MI2S Capture",
			.aif_name = "PRI_MI2S_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Primary MI2S_TX",
		.id = MSM_PRIM_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary MI2S Playback",
			.aif_name = "SEC_MI2S_RX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Secondary MI2S_RX",
		.id = MSM_SEC_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "Secondary MI2S Capture",
			.aif_name = "SEC_MI2S_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Secondary MI2S_TX",
		.id = MSM_SEC_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "Tertiary MI2S Playback",
			.aif_name = "TERT_MI2S_RX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Tertiary MI2S_RX",
		.id = MSM_TERT_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "Tertiary MI2S Capture",
			.aif_name = "TERT_MI2S_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Tertiary MI2S_TX",
		.id = MSM_TERT_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "Quaternary MI2S Playback",
			.aif_name = "QUAT_MI2S_RX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Quaternary MI2S_RX",
		.id = MSM_QUAT_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "Quaternary MI2S Capture",
			.aif_name = "QUAT_MI2S_TX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Quaternary MI2S_TX",
		.id = MSM_QUAT_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "Quinary MI2S Playback",
			.aif_name = "QUIN_MI2S_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Quinary MI2S_RX",
		.id = MSM_QUIN_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "Quinary MI2S Capture",
			.aif_name = "QUIN_MI2S_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Quinary MI2S_TX",
		.id = MSM_QUIN_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "Senary MI2S Playback",
			.aif_name = "SEN_MI2S_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Senary MI2S_RX",
		.id = MSM_SENARY_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "Senary MI2S Capture",
			.aif_name = "SENARY_MI2S_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "Senary MI2S_TX",
		.id = MSM_SENARY_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary MI2S Playback SD1",
			.aif_name = "SEC_MI2S_RX_SD1",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.id = MSM_SEC_MI2S_SD1,
	},
	{
		.playback = {
			.stream_name = "INT0 MI2S Playback",
			.aif_name = "INT0_MI2S_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT0 MI2S_RX",
		.id = MSM_INT0_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "INT0 MI2S Capture",
			.aif_name = "INT0_MI2S_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT0 MI2S_TX",
		.id = MSM_INT0_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "INT1 MI2S Playback",
			.aif_name = "INT1_MI2S_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT1 MI2S_RX",
		.id = MSM_INT1_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "INT1 MI2S Capture",
			.aif_name = "INT1_MI2S_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT1 MI2S_TX",
		.id = MSM_INT1_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "INT2 MI2S Playback",
			.aif_name = "INT2_MI2S_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT2 MI2S_RX",
		.id = MSM_INT2_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "INT2 MI2S Capture",
			.aif_name = "INT2_MI2S_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT2 MI2S_TX",
		.id = MSM_INT2_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "INT3 MI2S Playback",
			.aif_name = "INT3_MI2S_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT3 MI2S_RX",
		.id = MSM_INT3_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "INT3 MI2S Capture",
			.aif_name = "INT3_MI2S_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT3 MI2S_TX",
		.id = MSM_INT3_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "INT4 MI2S Playback",
			.aif_name = "INT4_MI2S_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT4 MI2S_RX",
		.id = MSM_INT4_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "INT4 MI2S Capture",
			.aif_name = "INT4_MI2S_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT4 MI2S_TX",
		.id = MSM_INT4_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "INT5 MI2S Playback",
			.aif_name = "INT5_MI2S_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT5 MI2S_RX",
		.id = MSM_INT5_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "INT5 MI2S Capture",
			.aif_name = "INT5_MI2S_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT5 MI2S_TX",
		.id = MSM_INT5_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "INT6 MI2S Playback",
			.aif_name = "INT6_MI2S_RX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT6 MI2S_RX",
		.id = MSM_INT6_MI2S_RX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
	{
		.capture = {
			.stream_name = "INT6 MI2S Capture",
			.aif_name = "INT6_MI2S_TX",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
			SNDRV_PCM_RATE_16000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     48000,
		},
		.ops = &msm_dai_q6_mi2s_ops,
		.name = "INT6 MI2S_TX",
		.id = MSM_INT6_MI2S_TX,
		.probe = msm_dai_q6_dai_mi2s_probe,
		.remove = msm_dai_q6_dai_mi2s_remove,
	},
};


static int msm_dai_q6_mi2s_get_lineconfig(u16 sd_lines, u16 *config_ptr,
					  unsigned int *ch_cnt)
{
	u8 num_of_sd_lines;

	num_of_sd_lines = num_of_bits_set(sd_lines);
	switch (num_of_sd_lines) {
	case 0:
		pr_debug("%s: no line is assigned\n", __func__);
		break;
	case 1:
		switch (sd_lines) {
		case MSM_MI2S_SD0:
			*config_ptr = AFE_PORT_I2S_SD0;
			break;
		case MSM_MI2S_SD1:
			*config_ptr = AFE_PORT_I2S_SD1;
			break;
		case MSM_MI2S_SD2:
			*config_ptr = AFE_PORT_I2S_SD2;
			break;
		case MSM_MI2S_SD3:
			*config_ptr = AFE_PORT_I2S_SD3;
			break;
		case MSM_MI2S_SD4:
			*config_ptr = AFE_PORT_I2S_SD4;
			break;
		case MSM_MI2S_SD5:
			*config_ptr = AFE_PORT_I2S_SD5;
			break;
		case MSM_MI2S_SD6:
			*config_ptr = AFE_PORT_I2S_SD6;
			break;
		case MSM_MI2S_SD7:
			*config_ptr = AFE_PORT_I2S_SD7;
			break;
		default:
			pr_err("%s: invalid SD lines %d\n",
				   __func__, sd_lines);
			goto error_invalid_data;
		}
		break;
	case 2:
		switch (sd_lines) {
		case MSM_MI2S_SD0 | MSM_MI2S_SD1:
			*config_ptr = AFE_PORT_I2S_QUAD01;
			break;
		case MSM_MI2S_SD2 | MSM_MI2S_SD3:
			*config_ptr = AFE_PORT_I2S_QUAD23;
			break;
		case MSM_MI2S_SD4 | MSM_MI2S_SD5:
			*config_ptr = AFE_PORT_I2S_QUAD45;
			break;
		case MSM_MI2S_SD6 | MSM_MI2S_SD7:
			*config_ptr = AFE_PORT_I2S_QUAD67;
			break;
		default:
			pr_err("%s: invalid SD lines %d\n",
				   __func__, sd_lines);
			goto error_invalid_data;
		}
		break;
	case 3:
		switch (sd_lines) {
		case MSM_MI2S_SD0 | MSM_MI2S_SD1 | MSM_MI2S_SD2:
			*config_ptr = AFE_PORT_I2S_6CHS;
			break;
		default:
			pr_err("%s: invalid SD lines %d\n",
				   __func__, sd_lines);
			goto error_invalid_data;
		}
		break;
	case 4:
		switch (sd_lines) {
		case MSM_MI2S_SD0 | MSM_MI2S_SD1 | MSM_MI2S_SD2 | MSM_MI2S_SD3:
			*config_ptr = AFE_PORT_I2S_8CHS;
			break;
		case MSM_MI2S_SD4 | MSM_MI2S_SD5 | MSM_MI2S_SD6 | MSM_MI2S_SD7:
			*config_ptr = AFE_PORT_I2S_8CHS_2;
			break;
		default:
			pr_err("%s: invalid SD lines %d\n",
				   __func__, sd_lines);
			goto error_invalid_data;
		}
		break;
	case 5:
		switch (sd_lines) {
		case MSM_MI2S_SD0 | MSM_MI2S_SD1 | MSM_MI2S_SD2
		   | MSM_MI2S_SD3 | MSM_MI2S_SD4:
			*config_ptr = AFE_PORT_I2S_10CHS;
			break;
		default:
			pr_err("%s: invalid SD lines %d\n",
				   __func__, sd_lines);
			goto error_invalid_data;
		}
		break;
	case 6:
		switch (sd_lines) {
		case MSM_MI2S_SD0 | MSM_MI2S_SD1 | MSM_MI2S_SD2
		   | MSM_MI2S_SD3 | MSM_MI2S_SD4 | MSM_MI2S_SD5:
			*config_ptr = AFE_PORT_I2S_12CHS;
			break;
		default:
			pr_err("%s: invalid SD lines %d\n",
				   __func__, sd_lines);
			goto error_invalid_data;
		}
		break;
	case 7:
		switch (sd_lines) {
		case MSM_MI2S_SD0 | MSM_MI2S_SD1 | MSM_MI2S_SD2 | MSM_MI2S_SD3
		   | MSM_MI2S_SD4 | MSM_MI2S_SD5 | MSM_MI2S_SD6:
			*config_ptr = AFE_PORT_I2S_14CHS;
			break;
		default:
			pr_err("%s: invalid SD lines %d\n",
				   __func__, sd_lines);
			goto error_invalid_data;
		}
		break;
	case 8:
		switch (sd_lines) {
		case MSM_MI2S_SD0 | MSM_MI2S_SD1 | MSM_MI2S_SD2 | MSM_MI2S_SD3
		   | MSM_MI2S_SD4 | MSM_MI2S_SD5 | MSM_MI2S_SD6 | MSM_MI2S_SD7:
			*config_ptr = AFE_PORT_I2S_16CHS;
			break;
		default:
			pr_err("%s: invalid SD lines %d\n",
				   __func__, sd_lines);
			goto error_invalid_data;
		}
		break;
	default:
		pr_err("%s: invalid SD lines %d\n", __func__, num_of_sd_lines);
		goto error_invalid_data;
	}
	*ch_cnt = num_of_sd_lines;
	return 0;

error_invalid_data:
	pr_err("%s: invalid data\n", __func__);
	return -EINVAL;
}

static u16 msm_dai_q6_mi2s_get_num_channels(u16 config)
{
	switch (config) {
	case AFE_PORT_I2S_SD0:
	case AFE_PORT_I2S_SD1:
	case AFE_PORT_I2S_SD2:
	case AFE_PORT_I2S_SD3:
	case AFE_PORT_I2S_SD4:
	case AFE_PORT_I2S_SD5:
	case AFE_PORT_I2S_SD6:
	case AFE_PORT_I2S_SD7:
		return 2;
	case AFE_PORT_I2S_QUAD01:
	case AFE_PORT_I2S_QUAD23:
	case AFE_PORT_I2S_QUAD45:
	case AFE_PORT_I2S_QUAD67:
		return 4;
	case AFE_PORT_I2S_6CHS:
		return 6;
	case AFE_PORT_I2S_8CHS:
	case AFE_PORT_I2S_8CHS_2:
		return 8;
	case AFE_PORT_I2S_10CHS:
		return 10;
	case AFE_PORT_I2S_12CHS:
		return 12;
	case AFE_PORT_I2S_14CHS:
		return 14;
	case AFE_PORT_I2S_16CHS:
		return 16;
	default:
		pr_err("%s: invalid config\n", __func__);
		return 0;
	}
}

static int msm_dai_q6_mi2s_platform_data_validation(
	struct platform_device *pdev, struct snd_soc_dai_driver *dai_driver)
{
	struct msm_dai_q6_mi2s_dai_data *dai_data = dev_get_drvdata(&pdev->dev);
	struct msm_mi2s_pdata *mi2s_pdata =
			(struct msm_mi2s_pdata *) pdev->dev.platform_data;
	unsigned int ch_cnt;
	int rc = 0;
	u16 sd_line;

	if (mi2s_pdata == NULL) {
		pr_err("%s: mi2s_pdata NULL", __func__);
		return -EINVAL;
	}

	rc = msm_dai_q6_mi2s_get_lineconfig(mi2s_pdata->sd_lines,
				&sd_line, &ch_cnt);
	if (rc < 0) {
		dev_err(&pdev->dev, "invalid MI2S RX sd line config\n");
		goto rtn;
	}

	if (strnstr(dev_name(&pdev->dev), "rx", strlen(dev_name(&pdev->dev)))
		!= NULL) {
		if (ch_cnt) {
			dai_data->mi2s_dai.mi2s_dai_data.port_config.i2s.channel_mode =
			sd_line;
			dai_data->mi2s_dai.pdata_mi2s_lines = sd_line;
			dai_driver->playback.channels_min = 1;
			dai_driver->playback.channels_max = ch_cnt << 1;
		} else {
			dai_driver->playback.channels_min = 0;
			dai_driver->playback.channels_max = 0;
		}
	} else {
		if (ch_cnt) {
			dai_data->mi2s_dai.mi2s_dai_data.port_config.i2s.channel_mode =
			sd_line;
			dai_data->mi2s_dai.pdata_mi2s_lines = sd_line;
			dai_driver->capture.channels_min = 1;
			dai_driver->capture.channels_max = ch_cnt << 1;
		} else {
			dai_driver->capture.channels_min = 0;
			dai_driver->capture.channels_max = 0;
		}
	}
	dev_dbg(&pdev->dev, "%s: sdlines 0x%x\n",
		__func__, dai_data->mi2s_dai.pdata_mi2s_lines);
	dev_dbg(&pdev->dev, "%s: playback ch_max %d capture ch_mx %d\n",
		__func__, dai_driver->playback.channels_max,
		dai_driver->capture.channels_max);
rtn:
	return rc;
}

static const struct snd_soc_component_driver msm_q6_mi2s_dai_component = {
	.name		= "msm-dai-q6-mi2s",
};
static int msm_dai_q6_mi2s_dev_probe(struct platform_device *pdev)
{
	struct msm_dai_q6_mi2s_dai_data *dai_data;
	const char *q6_mi2s_dev_id = "qcom,msm-dai-q6-mi2s-dev-id";
	u32 mi2s_intf_line = 0;
	u32 mi2s_intf = 0;
	struct msm_mi2s_pdata *mi2s_pdata;
	int rc;

	rc = of_property_read_u32(pdev->dev.of_node, q6_mi2s_dev_id,
				  &mi2s_intf);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: missing 0x%x in dt node\n", __func__, mi2s_intf);
		goto rtn;
	}

	dev_dbg(&pdev->dev, "dev name %s dev id 0x%x\n",
		dev_name(&pdev->dev), mi2s_intf);

	if ((mi2s_intf < MSM_MI2S_MIN || mi2s_intf > MSM_MI2S_MAX)
		|| (mi2s_intf >= ARRAY_SIZE(msm_dai_q6_mi2s_dai))) {
		dev_err(&pdev->dev,
			"%s: Invalid MI2S ID %u from Device Tree\n",
			__func__, mi2s_intf);
		rc = -ENXIO;
		goto rtn;
	}

	pdev->id = mi2s_intf;

	mi2s_pdata = kzalloc(sizeof(struct msm_mi2s_pdata), GFP_KERNEL);
	if (!mi2s_pdata) {
		rc = -ENOMEM;
		goto rtn;
	}

	rc = of_property_read_u32(pdev->dev.of_node,
			"qcom,msm-mi2s-lines", &mi2s_intf_line);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: Mi2s line from DT file %s\n", __func__,
			"qcom,msm-mi2s-lines");
		goto free_pdata;
	}

	mi2s_pdata->sd_lines = mi2s_intf_line;
	mi2s_pdata->intf_id = mi2s_intf;

	dai_data = kzalloc(sizeof(struct msm_dai_q6_mi2s_dai_data),
			   GFP_KERNEL);
	if (!dai_data) {
		rc = -ENOMEM;
		goto free_pdata;
	} else
		dev_set_drvdata(&pdev->dev, dai_data);

	rc = of_property_read_u32(pdev->dev.of_node,
				    "qcom,msm-dai-is-island-supported",
				    &dai_data->is_island_dai);
	if (rc)
		dev_dbg(&pdev->dev, "island supported entry not found\n");

	pdev->dev.platform_data = mi2s_pdata;

	rc = msm_dai_q6_mi2s_platform_data_validation(pdev,
			&msm_dai_q6_mi2s_dai[mi2s_intf]);
	if (rc < 0)
		goto free_dai_data;

	rc = snd_soc_register_component(&pdev->dev, &msm_q6_mi2s_dai_component,
	&msm_dai_q6_mi2s_dai[mi2s_intf], 1);
	if (rc < 0)
		goto err_register;
	return 0;

err_register:
	dev_err(&pdev->dev, "fail to msm_dai_q6_mi2s_dev_probe\n");
free_dai_data:
	kfree(dai_data);
free_pdata:
	kfree(mi2s_pdata);
rtn:
	return rc;
}

static int msm_dai_q6_mi2s_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static int msm_dai_q6_dai_meta_mi2s_probe(struct snd_soc_dai *dai)
{
	struct msm_meta_mi2s_pdata *meta_mi2s_pdata =
			(struct msm_meta_mi2s_pdata *) dai->dev->platform_data;
	int rc = 0;

	dai->id = meta_mi2s_pdata->intf_id;
	rc = msm_dai_q6_dai_add_route(dai);
	return rc;
}

static int msm_dai_q6_dai_meta_mi2s_remove(struct snd_soc_dai *dai)
{
	return 0;
}

static int msm_dai_q6_meta_mi2s_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	return 0;
}

static int msm_meta_mi2s_get_port_id(u32 mi2s_id, int stream, u16 *port_id)
{
	int ret = 0;

	switch (stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		switch (mi2s_id) {
		case MSM_PRIM_META_MI2S:
			*port_id = AFE_PORT_ID_PRIMARY_META_MI2S_RX;
			break;
		case MSM_SEC_META_MI2S:
			*port_id = AFE_PORT_ID_SECONDARY_META_MI2S_RX;
			break;
		default:
			pr_err("%s: playback err id 0x%x\n",
				__func__, mi2s_id);
			ret = -1;
			break;
		}
		break;

	case SNDRV_PCM_STREAM_CAPTURE:
		switch (mi2s_id) {
		default:
			pr_err("%s: capture err id 0x%x\n", __func__, mi2s_id);
			ret = -1;
			break;
		}
		break;

	default:
		pr_err("%s: default err %d\n", __func__, stream);
		ret = -1;
		break;
	}
	pr_debug("%s: port_id = 0x%x\n", __func__, *port_id);
	return ret;
}

static int msm_dai_q6_meta_mi2s_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_meta_mi2s_dai_data *dai_data =
		dev_get_drvdata(dai->dev);
	u16 port_id = 0;
	int rc = 0;

	if (msm_meta_mi2s_get_port_id(dai->id, substream->stream,
		&port_id) != 0) {
		dev_err(dai->dev, "%s: Invalid Port ID 0x%x\n",
			__func__, port_id);
		return -EINVAL;
	}

	dev_dbg(dai->dev, "%s: dai id %d, afe port id = 0x%x\n"
		"dai_data->channels = %u sample_rate = %u\n", __func__,
		dai->id, port_id, dai_data->channels, dai_data->rate);

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		/* PORT START should be set if prepare called
		 * in active state.
		 */
		rc = afe_port_start(port_id, &dai_data->port_config,
				    dai_data->rate);
		if (rc < 0)
			dev_err(dai->dev, "fail to open AFE port 0x%x\n",
				dai->id);
		else
			set_bit(STATUS_PORT_STARTED,
				dai_data->status_mask);
	}

	return rc;
}

static int msm_dai_q6_meta_mi2s_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct msm_dai_q6_meta_mi2s_dai_data *dai_data =
		dev_get_drvdata(dai->dev);
	struct afe_param_id_meta_i2s_cfg *port_cfg =
		&dai_data->port_config.meta_i2s;
	int idx = 0;
	u16 port_channels = 0;
	u16 channels_left = 0;

	dai_data->channels = params_channels(params);
	channels_left = dai_data->channels;

	/* map requested channels to channels that member ports provide */
	for (idx = 0; idx < dai_data->num_member_ports; idx++) {
		port_channels = msm_dai_q6_mi2s_get_num_channels(
			dai_data->channel_mode[idx]);

		if (channels_left >= port_channels) {
			port_cfg->member_port_id[idx] =
				dai_data->member_port_id[idx];
			port_cfg->member_port_channel_mode[idx] =
				dai_data->channel_mode[idx];
			channels_left -= port_channels;
		} else {
			switch (channels_left) {
			case 15:
			case 16:
				switch (dai_data->channel_mode[idx]) {
				case AFE_PORT_I2S_16CHS:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_16CHS;
					break;
				default:
					goto error_invalid_data;
				};
				break;
			case 13:
			case 14:
				switch (dai_data->channel_mode[idx]) {
				case AFE_PORT_I2S_14CHS:
				case AFE_PORT_I2S_16CHS:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_14CHS;
					break;
				default:
					goto error_invalid_data;
				};
				break;
			case 11:
			case 12:
				switch (dai_data->channel_mode[idx]) {
				case AFE_PORT_I2S_12CHS:
				case AFE_PORT_I2S_14CHS:
				case AFE_PORT_I2S_16CHS:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_12CHS;
					break;
				default:
					goto error_invalid_data;
				};
				break;
			case 9:
			case 10:
				switch (dai_data->channel_mode[idx]) {
				case AFE_PORT_I2S_10CHS:
				case AFE_PORT_I2S_12CHS:
				case AFE_PORT_I2S_14CHS:
				case AFE_PORT_I2S_16CHS:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_10CHS;
					break;
				default:
					goto error_invalid_data;
				};
				break;
			case 8:
			case 7:
				switch (dai_data->channel_mode[idx]) {
				case AFE_PORT_I2S_8CHS:
				case AFE_PORT_I2S_10CHS:
				case AFE_PORT_I2S_12CHS:
				case AFE_PORT_I2S_14CHS:
				case AFE_PORT_I2S_16CHS:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_8CHS;
					break;
				case AFE_PORT_I2S_8CHS_2:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_8CHS_2;
					break;
				default:
					goto error_invalid_data;
				};
				break;
			case 6:
			case 5:
				switch (dai_data->channel_mode[idx]) {
				case AFE_PORT_I2S_6CHS:
				case AFE_PORT_I2S_8CHS:
				case AFE_PORT_I2S_10CHS:
				case AFE_PORT_I2S_12CHS:
				case AFE_PORT_I2S_14CHS:
				case AFE_PORT_I2S_16CHS:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_6CHS;
					break;
				default:
					goto error_invalid_data;
				};
				break;
			case 4:
			case 3:
				switch (dai_data->channel_mode[idx]) {
				case AFE_PORT_I2S_SD0:
				case AFE_PORT_I2S_SD1:
				case AFE_PORT_I2S_SD2:
				case AFE_PORT_I2S_SD3:
				case AFE_PORT_I2S_SD4:
				case AFE_PORT_I2S_SD5:
				case AFE_PORT_I2S_SD6:
				case AFE_PORT_I2S_SD7:
					goto error_invalid_data;
				case AFE_PORT_I2S_QUAD01:
				case AFE_PORT_I2S_QUAD23:
				case AFE_PORT_I2S_QUAD45:
				case AFE_PORT_I2S_QUAD67:
					port_cfg->member_port_channel_mode[idx]
						= dai_data->channel_mode[idx];
					break;
				case AFE_PORT_I2S_8CHS_2:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_QUAD45;
					break;
				default:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_QUAD01;
				};
				break;
			case 2:
			case 1:
				if (dai_data->channel_mode[idx] <
					AFE_PORT_I2S_SD0)
					goto error_invalid_data;
				switch (dai_data->channel_mode[idx]) {
				case AFE_PORT_I2S_SD0:
				case AFE_PORT_I2S_SD1:
				case AFE_PORT_I2S_SD2:
				case AFE_PORT_I2S_SD3:
				case AFE_PORT_I2S_SD4:
				case AFE_PORT_I2S_SD5:
				case AFE_PORT_I2S_SD6:
				case AFE_PORT_I2S_SD7:
					port_cfg->member_port_channel_mode[idx]
						= dai_data->channel_mode[idx];
					break;
				case AFE_PORT_I2S_QUAD01:
				case AFE_PORT_I2S_6CHS:
				case AFE_PORT_I2S_8CHS:
				case AFE_PORT_I2S_10CHS:
				case AFE_PORT_I2S_12CHS:
				case AFE_PORT_I2S_14CHS:
				case AFE_PORT_I2S_16CHS:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_SD0;
					break;
				case AFE_PORT_I2S_QUAD23:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_SD2;
					break;
				case AFE_PORT_I2S_QUAD45:
				case AFE_PORT_I2S_8CHS_2:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_SD4;
					break;
				case AFE_PORT_I2S_QUAD67:
					port_cfg->member_port_channel_mode[idx]
						= AFE_PORT_I2S_SD6;
					break;
				}
				break;
			case 0:
				port_cfg->member_port_channel_mode[idx] = 0;
			}
			if (port_cfg->member_port_channel_mode[idx] == 0) {
				port_cfg->member_port_id[idx] =
					AFE_PORT_ID_INVALID;
			} else {
				port_cfg->member_port_id[idx] =
					dai_data->member_port_id[idx];
				channels_left -=
					msm_dai_q6_mi2s_get_num_channels(
					port_cfg->member_port_channel_mode[idx]);
			}
		}
	}

	if (channels_left > 0) {
		pr_err("%s: too many channels %d\n",
			__func__, dai_data->channels);
		return -EINVAL;
	}

	dai_data->rate = params_rate(params);
	port_cfg->sample_rate = dai_data->rate;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_SPECIAL:
		port_cfg->bit_width = 16;
		dai_data->bitwidth = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		port_cfg->bit_width = 24;
		dai_data->bitwidth = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		port_cfg->bit_width = 32;
		dai_data->bitwidth = 32;
		break;
	default:
		pr_err("%s: format %d\n",
			__func__, params_format(params));
		return -EINVAL;
	}

	port_cfg->minor_version = AFE_API_VERSION_META_I2S_CONFIG;
	port_cfg->data_format = AFE_LINEAR_PCM_DATA;

	dev_dbg(dai->dev, "%s: dai id %d dai_data->channels = %d\n"
		"bit_width = %hu ws_src = 0x%x sample_rate = %u\n"
		"member_ports 0x%x 0x%x 0x%x 0x%x\n"
		"sd_lines 0x%x 0x%x 0x%x 0x%x\n",
		__func__, dai->id, dai_data->channels,
		port_cfg->bit_width, port_cfg->ws_src, port_cfg->sample_rate,
		port_cfg->member_port_id[0],
		port_cfg->member_port_id[1],
		port_cfg->member_port_id[2],
		port_cfg->member_port_id[3],
		port_cfg->member_port_channel_mode[0],
		port_cfg->member_port_channel_mode[1],
		port_cfg->member_port_channel_mode[2],
		port_cfg->member_port_channel_mode[3]);
	return 0;

error_invalid_data:
	pr_err("%s: error when assigning member port %d channels (channels_left %d)\n",
		__func__, idx, channels_left);
	return -EINVAL;
}

static int msm_dai_q6_meta_mi2s_set_fmt(struct snd_soc_dai *dai,
	unsigned int fmt)
{
	struct msm_dai_q6_meta_mi2s_dai_data *dai_data =
		dev_get_drvdata(dai->dev);

	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		dev_err(dai->dev, "%s: err chg meta i2s mode while dai running",
			__func__);
		return -EPERM;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		dai_data->port_config.meta_i2s.ws_src = 1;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		dai_data->port_config.meta_i2s.ws_src = 0;
		break;
	default:
		pr_err("%s: fmt %d\n",
			__func__, fmt & SND_SOC_DAIFMT_MASTER_MASK);
		return -EINVAL;
	}

	return 0;
}

static void msm_dai_q6_meta_mi2s_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct msm_dai_q6_meta_mi2s_dai_data *dai_data =
		dev_get_drvdata(dai->dev);
	u16 port_id = 0;
	int rc = 0;

	if (msm_meta_mi2s_get_port_id(dai->id, substream->stream,
				 &port_id) != 0) {
		dev_err(dai->dev, "%s: Invalid Port ID 0x%x\n",
			__func__, port_id);
	}

	dev_dbg(dai->dev, "%s: closing afe port id = 0x%x\n",
			__func__, port_id);

	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		rc = afe_close(port_id);
		if (rc < 0)
			dev_err(dai->dev, "fail to close AFE port\n");
		clear_bit(STATUS_PORT_STARTED, dai_data->status_mask);
	}
}

static struct snd_soc_dai_ops msm_dai_q6_meta_mi2s_ops = {
	.startup	= msm_dai_q6_meta_mi2s_startup,
	.prepare	= msm_dai_q6_meta_mi2s_prepare,
	.hw_params	= msm_dai_q6_meta_mi2s_hw_params,
	.set_fmt	= msm_dai_q6_meta_mi2s_set_fmt,
	.shutdown	= msm_dai_q6_meta_mi2s_shutdown,
};

/* Channel min and max are initialized base on platform data */
static struct snd_soc_dai_driver msm_dai_q6_meta_mi2s_dai[] = {
	{
		.playback = {
			.stream_name = "Primary META MI2S Playback",
			.aif_name = "PRI_META_MI2S_RX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S24_3LE,
			.rate_min =     8000,
			.rate_max =     384000,
		},
		.ops = &msm_dai_q6_meta_mi2s_ops,
		.name = "Primary META MI2S",
		.id = AFE_PORT_ID_PRIMARY_META_MI2S_RX,
		.probe = msm_dai_q6_dai_meta_mi2s_probe,
		.remove = msm_dai_q6_dai_meta_mi2s_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary META MI2S Playback",
			.aif_name = "SEC_META_MI2S_RX",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				 SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
			.rate_min =     8000,
			.rate_max =     192000,
		},
		.ops = &msm_dai_q6_meta_mi2s_ops,
		.name = "Secondary META MI2S",
		.id = AFE_PORT_ID_SECONDARY_META_MI2S_RX,
		.probe = msm_dai_q6_dai_meta_mi2s_probe,
		.remove = msm_dai_q6_dai_meta_mi2s_remove,
	},
};

static int msm_dai_q6_meta_mi2s_platform_data_validation(
	struct platform_device *pdev, struct snd_soc_dai_driver *dai_driver)
{
	struct msm_dai_q6_meta_mi2s_dai_data *dai_data =
		dev_get_drvdata(&pdev->dev);
	struct msm_meta_mi2s_pdata *meta_mi2s_pdata =
		(struct msm_meta_mi2s_pdata *) pdev->dev.platform_data;
	int rc = 0;
	int idx = 0;
	u16 channel_mode = 0;
	unsigned int ch_cnt = 0;
	unsigned int ch_cnt_sum = 0;
	struct afe_param_id_meta_i2s_cfg *port_cfg =
		&dai_data->port_config.meta_i2s;

	if (meta_mi2s_pdata == NULL) {
		pr_err("%s: meta_mi2s_pdata NULL", __func__);
		return -EINVAL;
	}

	dai_data->num_member_ports = meta_mi2s_pdata->num_member_ports;
	for (idx = 0; idx < meta_mi2s_pdata->num_member_ports; idx++) {
		rc = msm_dai_q6_mi2s_get_lineconfig(
			meta_mi2s_pdata->sd_lines[idx],
			&channel_mode,
			&ch_cnt);
		if (rc < 0) {
			dev_err(&pdev->dev, "invalid META MI2S RX sd line config\n");
			goto rtn;
		}
		if (ch_cnt) {
			msm_mi2s_get_port_id(meta_mi2s_pdata->member_port[idx],
				SNDRV_PCM_STREAM_PLAYBACK,
				&dai_data->member_port_id[idx]);
			dai_data->channel_mode[idx] = channel_mode;
			port_cfg->member_port_id[idx] =
				dai_data->member_port_id[idx];
			port_cfg->member_port_channel_mode[idx] = channel_mode;
		}
		ch_cnt_sum += ch_cnt;
	}

	if (ch_cnt_sum) {
		dai_driver->playback.channels_min = 1;
		dai_driver->playback.channels_max = ch_cnt_sum << 1;
	} else {
		dai_driver->playback.channels_min = 0;
		dai_driver->playback.channels_max = 0;
	}

	dev_dbg(&pdev->dev, "%s: sdline 0x%x 0x%x 0x%x 0x%x\n", __func__,
		dai_data->channel_mode[0], dai_data->channel_mode[1],
		dai_data->channel_mode[2], dai_data->channel_mode[3]);
	dev_dbg(&pdev->dev, "%s: playback ch_max %d\n",
		__func__, dai_driver->playback.channels_max);
rtn:
	return rc;
}

static const struct snd_soc_component_driver msm_q6_meta_mi2s_dai_component = {
	.name		= "msm-dai-q6-meta-mi2s",
};

static int msm_dai_q6_meta_mi2s_dev_probe(struct platform_device *pdev)
{
	struct msm_dai_q6_meta_mi2s_dai_data *dai_data;
	const char *q6_meta_mi2s_dev_id = "qcom,msm-dai-q6-meta-mi2s-dev-id";
	u32 dev_id = 0;
	u32 meta_mi2s_intf = 0;
	struct msm_meta_mi2s_pdata *meta_mi2s_pdata;
	int rc;

	rc = of_property_read_u32(pdev->dev.of_node, q6_meta_mi2s_dev_id,
				  &dev_id);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: missing %s in dt node\n", __func__,
			q6_meta_mi2s_dev_id);
		goto rtn;
	}

	dev_dbg(&pdev->dev, "dev name %s dev id 0x%x\n", dev_name(&pdev->dev),
		dev_id);

	switch (dev_id) {
	case AFE_PORT_ID_PRIMARY_META_MI2S_RX:
		meta_mi2s_intf = 0;
		break;
	case AFE_PORT_ID_SECONDARY_META_MI2S_RX:
		meta_mi2s_intf = 1;
		break;
	default:
		dev_err(&pdev->dev,
			"%s: Invalid META MI2S ID 0x%x from Device Tree\n",
			__func__, dev_id);
		rc = -ENXIO;
		goto rtn;
	}

	pdev->id = dev_id;

	meta_mi2s_pdata = kzalloc(sizeof(struct msm_meta_mi2s_pdata),
		GFP_KERNEL);
	if (!meta_mi2s_pdata) {
		rc = -ENOMEM;
		goto rtn;
	}

	rc = of_property_read_u32(pdev->dev.of_node,
		"qcom,msm-mi2s-num-members",
		&meta_mi2s_pdata->num_member_ports);
	if (rc) {
		dev_err(&pdev->dev, "%s: invalid num from DT file %s\n",
			__func__, "qcom,msm-mi2s-num-members");
		goto free_pdata;
	}

	if (meta_mi2s_pdata->num_member_ports >
		MAX_NUM_I2S_META_PORT_MEMBER_PORTS) {
		dev_err(&pdev->dev, "%s: num-members %d too large from DT file\n",
			__func__, meta_mi2s_pdata->num_member_ports);
		goto free_pdata;
	}

	rc = of_property_read_u32_array(pdev->dev.of_node,
		"qcom,msm-mi2s-member-id",
		meta_mi2s_pdata->member_port,
		meta_mi2s_pdata->num_member_ports);
	if (rc) {
		dev_err(&pdev->dev, "%s: member-id from DT file %s\n",
			__func__, "qcom,msm-mi2s-member-id");
		goto free_pdata;
	}

	rc = of_property_read_u32_array(pdev->dev.of_node,
		"qcom,msm-mi2s-rx-lines",
		meta_mi2s_pdata->sd_lines,
		meta_mi2s_pdata->num_member_ports);
	if (rc) {
		dev_err(&pdev->dev, "%s: Rx line from DT file %s\n",
			__func__, "qcom,msm-mi2s-rx-lines");
		goto free_pdata;
	}

	dev_dbg(&pdev->dev, "dev name %s num-members=%d\n",
		dev_name(&pdev->dev), meta_mi2s_pdata->num_member_ports);
	dev_dbg(&pdev->dev, "member array (%d, %d, %d, %d)\n",
		meta_mi2s_pdata->member_port[0],
		meta_mi2s_pdata->member_port[1],
		meta_mi2s_pdata->member_port[2],
		meta_mi2s_pdata->member_port[3]);
	dev_dbg(&pdev->dev, "sd-lines array (0x%x, 0x%x, 0x%x, 0x%x)\n",
		meta_mi2s_pdata->sd_lines[0],
		meta_mi2s_pdata->sd_lines[1],
		meta_mi2s_pdata->sd_lines[2],
		meta_mi2s_pdata->sd_lines[3]);

	meta_mi2s_pdata->intf_id = meta_mi2s_intf;

	dai_data = kzalloc(sizeof(struct msm_dai_q6_meta_mi2s_dai_data),
			   GFP_KERNEL);
	if (!dai_data) {
		rc = -ENOMEM;
		goto free_pdata;
	} else
		dev_set_drvdata(&pdev->dev, dai_data);

	pdev->dev.platform_data = meta_mi2s_pdata;

	rc = msm_dai_q6_meta_mi2s_platform_data_validation(pdev,
		&msm_dai_q6_meta_mi2s_dai[meta_mi2s_intf]);
	if (rc < 0)
		goto free_dai_data;

	rc = snd_soc_register_component(&pdev->dev,
		&msm_q6_meta_mi2s_dai_component,
		&msm_dai_q6_meta_mi2s_dai[meta_mi2s_intf], 1);
	if (rc < 0)
		goto err_register;
	return 0;

err_register:
	dev_err(&pdev->dev, "fail to %s\n", __func__);
free_dai_data:
	kfree(dai_data);
free_pdata:
	kfree(meta_mi2s_pdata);
rtn:
	return rc;
}

static int msm_dai_q6_meta_mi2s_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct snd_soc_component_driver msm_dai_q6_component = {
	.name		= "msm-dai-q6-dev",
};

static int msm_dai_q6_dev_probe(struct platform_device *pdev)
{
	int rc, id, i, len;
	const char *q6_dev_id = "qcom,msm-dai-q6-dev-id";
	char stream_name[80];

	rc = of_property_read_u32(pdev->dev.of_node, q6_dev_id, &id);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: missing %s in dt node\n", __func__, q6_dev_id);
		return rc;
	}

	pdev->id = id;

	pr_debug("%s: dev name %s, id:%d\n", __func__,
		 dev_name(&pdev->dev), pdev->id);

	switch (id) {
	case SLIMBUS_0_RX:
		strlcpy(stream_name, "Slimbus Playback", 80);
		goto register_slim_playback;
	case SLIMBUS_2_RX:
		strlcpy(stream_name, "Slimbus2 Playback", 80);
		goto register_slim_playback;
	case SLIMBUS_1_RX:
		strlcpy(stream_name, "Slimbus1 Playback", 80);
		goto register_slim_playback;
	case SLIMBUS_3_RX:
		strlcpy(stream_name, "Slimbus3 Playback", 80);
		goto register_slim_playback;
	case SLIMBUS_4_RX:
		strlcpy(stream_name, "Slimbus4 Playback", 80);
		goto register_slim_playback;
	case SLIMBUS_5_RX:
		strlcpy(stream_name, "Slimbus5 Playback", 80);
		goto register_slim_playback;
	case SLIMBUS_6_RX:
		strlcpy(stream_name, "Slimbus6 Playback", 80);
		goto register_slim_playback;
	case SLIMBUS_7_RX:
		strlcpy(stream_name, "Slimbus7 Playback", sizeof(stream_name));
		goto register_slim_playback;
	case SLIMBUS_8_RX:
		strlcpy(stream_name, "Slimbus8 Playback", sizeof(stream_name));
		goto register_slim_playback;
	case SLIMBUS_9_RX:
		strlcpy(stream_name, "Slimbus9 Playback", sizeof(stream_name));
		goto register_slim_playback;
register_slim_playback:
		rc = -ENODEV;
		len = strnlen(stream_name, 80);
		for (i = 0; i < ARRAY_SIZE(msm_dai_q6_slimbus_rx_dai); i++) {
			if (msm_dai_q6_slimbus_rx_dai[i].playback.stream_name &&
				!strcmp(stream_name,
				msm_dai_q6_slimbus_rx_dai[i]
				.playback.stream_name)) {
				rc = snd_soc_register_component(&pdev->dev,
				&msm_dai_q6_component,
				&msm_dai_q6_slimbus_rx_dai[i], 1);
				break;
			}
		}
		if (rc)
			pr_err("%s: Device not found stream name %s\n",
				__func__, stream_name);
		break;
	case SLIMBUS_0_TX:
		strlcpy(stream_name, "Slimbus Capture", 80);
		goto register_slim_capture;
	case SLIMBUS_1_TX:
		strlcpy(stream_name, "Slimbus1 Capture", 80);
		goto register_slim_capture;
	case SLIMBUS_2_TX:
		strlcpy(stream_name, "Slimbus2 Capture", 80);
		goto register_slim_capture;
	case SLIMBUS_3_TX:
		strlcpy(stream_name, "Slimbus3 Capture", 80);
		goto register_slim_capture;
	case SLIMBUS_4_TX:
		strlcpy(stream_name, "Slimbus4 Capture", 80);
		goto register_slim_capture;
	case SLIMBUS_5_TX:
		strlcpy(stream_name, "Slimbus5 Capture", 80);
		goto register_slim_capture;
	case SLIMBUS_6_TX:
		strlcpy(stream_name, "Slimbus6 Capture", 80);
		goto register_slim_capture;
	case SLIMBUS_7_TX:
		strlcpy(stream_name, "Slimbus7 Capture", sizeof(stream_name));
		goto register_slim_capture;
	case SLIMBUS_8_TX:
		strlcpy(stream_name, "Slimbus8 Capture", sizeof(stream_name));
		goto register_slim_capture;
	case SLIMBUS_9_TX:
		strlcpy(stream_name, "Slimbus9 Capture", sizeof(stream_name));
		goto register_slim_capture;
register_slim_capture:
		rc = -ENODEV;
		len = strnlen(stream_name, 80);
		for (i = 0; i < ARRAY_SIZE(msm_dai_q6_slimbus_tx_dai); i++) {
			if (msm_dai_q6_slimbus_tx_dai[i].capture.stream_name &&
				!strcmp(stream_name,
				msm_dai_q6_slimbus_tx_dai[i]
				.capture.stream_name)) {
				rc = snd_soc_register_component(&pdev->dev,
				&msm_dai_q6_component,
				&msm_dai_q6_slimbus_tx_dai[i], 1);
				break;
			}
		}
		if (rc)
			pr_err("%s: Device not found stream name %s\n",
				__func__, stream_name);
		break;
	case AFE_LOOPBACK_TX:
		rc = snd_soc_register_component(&pdev->dev,
						&msm_dai_q6_component,
						&msm_dai_q6_afe_lb_tx_dai[0],
						1);
		break;
	case INT_BT_SCO_RX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_component, &msm_dai_q6_bt_sco_rx_dai, 1);
		break;
	case INT_BT_SCO_TX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_component, &msm_dai_q6_bt_sco_tx_dai, 1);
		break;
	case INT_BT_A2DP_RX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_component, &msm_dai_q6_bt_a2dp_rx_dai, 1);
		break;
	case INT_FM_RX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_component, &msm_dai_q6_fm_rx_dai, 1);
		break;
	case INT_FM_TX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_component, &msm_dai_q6_fm_tx_dai, 1);
		break;
	case AFE_PORT_ID_USB_RX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_component, &msm_dai_q6_usb_rx_dai, 1);
		break;
	case AFE_PORT_ID_USB_TX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_component, &msm_dai_q6_usb_tx_dai, 1);
		break;
	case RT_PROXY_DAI_001_RX:
		strlcpy(stream_name, "AFE Playback", 80);
		goto register_afe_playback;
	case RT_PROXY_DAI_003_RX:
		strlcpy(stream_name, "AFE Playback1", 80);
		goto register_afe_playback;
	case RT_PROXY_DAI_002_RX:
		strlcpy(stream_name, "AFE-PROXY RX", 80);
register_afe_playback:
		rc = -ENODEV;
		len = strnlen(stream_name, 80);
		for (i = 0; i < ARRAY_SIZE(msm_dai_q6_afe_rx_dai); i++) {
			if (msm_dai_q6_afe_rx_dai[i].playback.stream_name &&
			    !strcmp(stream_name,
			    msm_dai_q6_afe_rx_dai[i].playback.stream_name)) {
				rc = snd_soc_register_component(&pdev->dev,
					&msm_dai_q6_component,
					&msm_dai_q6_afe_rx_dai[i], 1);
				break;
			}
		}
		if (rc)
			pr_err("%s: Device not found stream name %s\n",
			__func__, stream_name);
		break;
	case RT_PROXY_DAI_001_TX:
		strlcpy(stream_name, "AFE-PROXY TX", 80);
		goto register_afe_capture;
	case RT_PROXY_DAI_002_TX:
		strlcpy(stream_name, "AFE Capture", 80);
register_afe_capture:
		rc = -ENODEV;
		len = strnlen(stream_name, 80);
		for (i = 0; i < ARRAY_SIZE(msm_dai_q6_afe_tx_dai); i++) {
			if (msm_dai_q6_afe_tx_dai[i].capture.stream_name &&
				!strcmp(stream_name,
				msm_dai_q6_afe_tx_dai[i].capture.stream_name)) {
				rc = snd_soc_register_component(&pdev->dev,
					&msm_dai_q6_component,
					&msm_dai_q6_afe_tx_dai[i], 1);
				break;
			}
		}
		if (rc)
			pr_err("%s: Device not found stream name %s\n",
			__func__, stream_name);
		break;
        case RT_PROXY_DAI_003_TX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_component, &msm_dai_q6_afe_cap_dai, 1);
		break;
	case VOICE_PLAYBACK_TX:
		strlcpy(stream_name, "Voice Farend Playback", 80);
		goto register_voice_playback;
	case VOICE2_PLAYBACK_TX:
		strlcpy(stream_name, "Voice2 Farend Playback", 80);
register_voice_playback:
		rc = -ENODEV;
		len = strnlen(stream_name, 80);
		for (i = 0; i < ARRAY_SIZE(msm_dai_q6_voc_playback_dai); i++) {
			if (msm_dai_q6_voc_playback_dai[i].playback.stream_name
			    && !strcmp(stream_name,
			 msm_dai_q6_voc_playback_dai[i].playback.stream_name)) {
				rc = snd_soc_register_component(&pdev->dev,
					&msm_dai_q6_component,
					&msm_dai_q6_voc_playback_dai[i], 1);
				break;
			}
		}
		if (rc)
			pr_err("%s Device not found stream name %s\n",
			       __func__, stream_name);
		break;
	case VOICE_RECORD_RX:
		strlcpy(stream_name, "Voice Downlink Capture", 80);
		goto register_uplink_capture;
	case VOICE_RECORD_TX:
		strlcpy(stream_name, "Voice Uplink Capture", 80);
register_uplink_capture:
		rc = -ENODEV;
		len = strnlen(stream_name, 80);
		for (i = 0; i < ARRAY_SIZE(msm_dai_q6_incall_record_dai); i++) {
			if (msm_dai_q6_incall_record_dai[i].capture.stream_name
			    && !strcmp(stream_name,
			    msm_dai_q6_incall_record_dai[i].
			    capture.stream_name)) {
				rc = snd_soc_register_component(&pdev->dev,
					&msm_dai_q6_component,
					&msm_dai_q6_incall_record_dai[i], 1);
				break;
			}
		}
		if (rc)
			pr_err("%s: Device not found stream name %s\n",
			__func__, stream_name);
		break;
	case RT_PROXY_PORT_002_RX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_component, &msm_dai_q6_proxy_rx_dai, 1);
		break;
	case RT_PROXY_PORT_002_TX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_q6_component, &msm_dai_q6_proxy_tx_dai, 1);
		break;
	default:
		rc = -ENODEV;
		break;
	}

	return rc;
}

static int msm_dai_q6_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_dai_q6_dev_dt_match[] = {
	{ .compatible = "qcom,msm-dai-q6-dev", },
	{ }
};
MODULE_DEVICE_TABLE(of, msm_dai_q6_dev_dt_match);

static struct platform_driver msm_dai_q6_dev = {
	.probe  = msm_dai_q6_dev_probe,
	.remove = msm_dai_q6_dev_remove,
	.driver = {
		.name = "msm-dai-q6-dev",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_q6_dev_dt_match,
		.suppress_bind_attrs = true,
	},
};

static int msm_dai_q6_probe(struct platform_device *pdev)
{
	int rc;

	pr_debug("%s: dev name %s, id:%d\n", __func__,
		 dev_name(&pdev->dev), pdev->id);
	rc = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (rc) {
		dev_err(&pdev->dev, "%s: failed to add child nodes, rc=%d\n",
			__func__, rc);
	} else
		dev_dbg(&pdev->dev, "%s: added child node\n", __func__);

	return rc;
}

static int msm_dai_q6_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_dai_q6_dt_match[] = {
	{ .compatible = "qcom,msm-dai-q6", },
	{ }
};
MODULE_DEVICE_TABLE(of, msm_dai_q6_dt_match);
static struct platform_driver msm_dai_q6 = {
	.probe  = msm_dai_q6_probe,
	.remove = msm_dai_q6_remove,
	.driver = {
		.name = "msm-dai-q6",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_q6_dt_match,
		.suppress_bind_attrs = true,
	},
};

static int msm_dai_mi2s_q6_probe(struct platform_device *pdev)
{
	int rc;

	rc = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (rc) {
		dev_err(&pdev->dev, "%s: failed to add child nodes, rc=%d\n",
			__func__, rc);
	} else
		dev_dbg(&pdev->dev, "%s: added child node\n", __func__);
	return rc;
}

static int msm_dai_mi2s_q6_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id msm_dai_mi2s_dt_match[] = {
	{ .compatible = "qcom,msm-dai-mi2s", },
	{ }
};

MODULE_DEVICE_TABLE(of, msm_dai_mi2s_dt_match);

static struct platform_driver msm_dai_mi2s_q6 = {
	.probe  = msm_dai_mi2s_q6_probe,
	.remove = msm_dai_mi2s_q6_remove,
	.driver = {
		.name = "msm-dai-mi2s",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_mi2s_dt_match,
		.suppress_bind_attrs = true,
	},
};

static const struct of_device_id msm_dai_q6_mi2s_dev_dt_match[] = {
	{ .compatible = "qcom,msm-dai-q6-mi2s", },
	{ }
};

MODULE_DEVICE_TABLE(of, msm_dai_q6_mi2s_dev_dt_match);

static struct platform_driver msm_dai_q6_mi2s_driver = {
	.probe  = msm_dai_q6_mi2s_dev_probe,
	.remove  = msm_dai_q6_mi2s_dev_remove,
	.driver = {
		.name = "msm-dai-q6-mi2s",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_q6_mi2s_dev_dt_match,
		.suppress_bind_attrs = true,
	},
};

static const struct of_device_id msm_dai_q6_meta_mi2s_dev_dt_match[] = {
	{ .compatible = "qcom,msm-dai-q6-meta-mi2s", },
	{ }
};

MODULE_DEVICE_TABLE(of, msm_dai_q6_meta_mi2s_dev_dt_match);

static struct platform_driver msm_dai_q6_meta_mi2s_driver = {
	.probe  = msm_dai_q6_meta_mi2s_dev_probe,
	.remove  = msm_dai_q6_meta_mi2s_dev_remove,
	.driver = {
		.name = "msm-dai-q6-meta-mi2s",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_q6_meta_mi2s_dev_dt_match,
		.suppress_bind_attrs = true,
	},
};

static int msm_dai_q6_spdif_dev_probe(struct platform_device *pdev)
{
	int rc, id;
	const char *q6_dev_id = "qcom,msm-dai-q6-dev-id";

	rc = of_property_read_u32(pdev->dev.of_node, q6_dev_id, &id);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: missing %s in dt node\n", __func__, q6_dev_id);
		return rc;
	}

	pdev->id = id;

	pr_debug("%s: dev name %s, id:%d\n", __func__,
			dev_name(&pdev->dev), pdev->id);

	switch (pdev->id) {
	case AFE_PORT_ID_PRIMARY_SPDIF_RX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_spdif_q6_component,
			&msm_dai_q6_spdif_spdif_rx_dai[0], 1);
		break;
	case AFE_PORT_ID_SECONDARY_SPDIF_RX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_spdif_q6_component,
			&msm_dai_q6_spdif_spdif_rx_dai[1], 1);
		break;
	case AFE_PORT_ID_PRIMARY_SPDIF_TX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_spdif_q6_component,
			&msm_dai_q6_spdif_spdif_tx_dai[0], 1);
		break;
	case AFE_PORT_ID_SECONDARY_SPDIF_TX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_spdif_q6_component,
			&msm_dai_q6_spdif_spdif_tx_dai[1], 1);
		break;
	default:
		dev_err(&pdev->dev, "invalid device ID %d\n", pdev->id);
		rc = -ENODEV;
		break;
	}

	return rc;
}

static int msm_dai_q6_spdif_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_dai_q6_spdif_dt_match[] = {
	{.compatible = "qcom,msm-dai-q6-spdif"},
	{}
};
MODULE_DEVICE_TABLE(of, msm_dai_q6_spdif_dt_match);

static struct platform_driver msm_dai_q6_spdif_driver = {
	.probe  = msm_dai_q6_spdif_dev_probe,
	.remove = msm_dai_q6_spdif_dev_remove,
	.driver = {
		.name = "msm-dai-q6-spdif",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_q6_spdif_dt_match,
		.suppress_bind_attrs = true,
	},
};

static int msm_dai_q6_tdm_set_clk_param(u32 group_id,
					struct afe_clk_set *clk_set, u32 mode)
{
	switch (group_id) {
	case AFE_GROUP_DEVICE_ID_PRIMARY_TDM_RX:
	case AFE_GROUP_DEVICE_ID_PRIMARY_TDM_TX:
		if (mode)
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_PRI_TDM_IBIT;
		else
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_PRI_TDM_EBIT;
		break;
	case AFE_GROUP_DEVICE_ID_SECONDARY_TDM_RX:
	case AFE_GROUP_DEVICE_ID_SECONDARY_TDM_TX:
		if (mode)
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_SEC_TDM_IBIT;
		else
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_SEC_TDM_EBIT;
		break;
	case AFE_GROUP_DEVICE_ID_TERTIARY_TDM_RX:
	case AFE_GROUP_DEVICE_ID_TERTIARY_TDM_TX:
		if (mode)
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_TER_TDM_IBIT;
		else
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_TER_TDM_EBIT;
		break;
	case AFE_GROUP_DEVICE_ID_QUATERNARY_TDM_RX:
	case AFE_GROUP_DEVICE_ID_QUATERNARY_TDM_TX:
		if (mode)
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_QUAD_TDM_IBIT;
		else
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_QUAD_TDM_EBIT;
		break;
	case AFE_GROUP_DEVICE_ID_QUINARY_TDM_RX:
	case AFE_GROUP_DEVICE_ID_QUINARY_TDM_TX:
		if (mode)
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_QUIN_TDM_IBIT;
		else
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_QUIN_TDM_EBIT;
		break;
	case AFE_GROUP_DEVICE_ID_SENARY_TDM_RX:
	case AFE_GROUP_DEVICE_ID_SENARY_TDM_TX:
		if (mode)
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_SEN_TDM_IBIT;
		else
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_SEN_TDM_EBIT;
		break;
	case AFE_GROUP_DEVICE_ID_SEPTENARY_TDM_RX:
	case AFE_GROUP_DEVICE_ID_SEPTENARY_TDM_TX:
		if (mode)
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_SEP_TDM_IBIT;
		else
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_SEP_TDM_EBIT;
		break;
	case AFE_GROUP_DEVICE_ID_HSIF0_TDM_RX:
	case AFE_GROUP_DEVICE_ID_HSIF0_TDM_TX:
		if (mode)
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_HSIF0_TDM_IBIT;
		else
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_HSIF0_TDM_EBIT;
		break;
	case AFE_GROUP_DEVICE_ID_HSIF1_TDM_RX:
	case AFE_GROUP_DEVICE_ID_HSIF1_TDM_TX:
		if (mode)
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_HSIF1_TDM_IBIT;
		else
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_HSIF1_TDM_EBIT;
		break;
	case AFE_GROUP_DEVICE_ID_HSIF2_TDM_RX:
	case AFE_GROUP_DEVICE_ID_HSIF2_TDM_TX:
		if (mode)
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_HSIF2_TDM_IBIT;
		else
			clk_set->clk_id = Q6AFE_LPASS_CLK_ID_HSIF2_TDM_EBIT;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int msm_dai_tdm_q6_probe(struct platform_device *pdev)
{
	int rc = 0;
	const uint32_t *port_id_array = NULL;
	uint32_t array_length = 0;
	int i = 0;
	int group_idx = 0;
	u32 clk_mode = 0;

	/* extract tdm group info into static */
	rc = of_property_read_u32(pdev->dev.of_node,
		"qcom,msm-cpudai-tdm-group-id",
		(u32 *)&tdm_group_cfg.group_id);
	if (rc) {
		dev_err(&pdev->dev, "%s: Group ID from DT file %s\n",
			__func__, "qcom,msm-cpudai-tdm-group-id");
		goto rtn;
	}
	dev_dbg(&pdev->dev, "%s: Group ID from DT file 0x%x\n",
		__func__, tdm_group_cfg.group_id);

	rc = of_property_read_u32(pdev->dev.of_node,
		"qcom,msm-cpudai-tdm-group-num-ports",
		&num_tdm_group_ports);
	if (rc) {
		dev_err(&pdev->dev, "%s: Group Num Ports from DT file %s\n",
			__func__, "qcom,msm-cpudai-tdm-group-num-ports");
		goto rtn;
	}
	dev_dbg(&pdev->dev, "%s: Group Num Ports from DT file 0x%x\n",
		__func__, num_tdm_group_ports);

	if (num_tdm_group_ports > AFE_GROUP_DEVICE_NUM_PORTS) {
		dev_err(&pdev->dev, "%s Group Num Ports %d greater than Max %d\n",
			__func__, num_tdm_group_ports,
			AFE_GROUP_DEVICE_NUM_PORTS);
		rc = -EINVAL;
		goto rtn;
	}

	port_id_array = of_get_property(pdev->dev.of_node,
		"qcom,msm-cpudai-tdm-group-port-id",
		&array_length);
	if (port_id_array == NULL) {
		dev_err(&pdev->dev, "%s port_id_array is not valid\n",
			__func__);
		rc = -EINVAL;
		goto rtn;
	}
	if (array_length != sizeof(uint32_t) * num_tdm_group_ports) {
		dev_err(&pdev->dev, "%s array_length is %d, expected is %zd\n",
			__func__, array_length,
			sizeof(uint32_t) * num_tdm_group_ports);
		rc = -EINVAL;
		goto rtn;
	}

	for (i = 0; i < num_tdm_group_ports; i++)
		tdm_group_cfg.port_id[i] =
			(u16)be32_to_cpu(port_id_array[i]);
	/* Unused index should be filled with 0 or AFE_PORT_INVALID */
	for (i = num_tdm_group_ports; i < AFE_GROUP_DEVICE_NUM_PORTS; i++)
		tdm_group_cfg.port_id[i] =
			AFE_PORT_INVALID;

	/* extract tdm clk info into static */
	rc = of_property_read_u32(pdev->dev.of_node,
		"qcom,msm-cpudai-tdm-clk-rate",
		&tdm_clk_set.clk_freq_in_hz);
	if (rc) {
		dev_err(&pdev->dev, "%s: Clk Rate from DT file %s\n",
			__func__, "qcom,msm-cpudai-tdm-clk-rate");
		goto rtn;
	}
	dev_dbg(&pdev->dev, "%s: Clk Rate from DT file %d\n",
		__func__, tdm_clk_set.clk_freq_in_hz);

	/* initialize static tdm clk attribute to default value */
	tdm_clk_set.clk_attri = Q6AFE_LPASS_CLK_ATTRIBUTE_INVERT_COUPLE_NO;

	/* extract tdm clk attribute into static */
	if (of_find_property(pdev->dev.of_node,
			"qcom,msm-cpudai-tdm-clk-attribute", NULL)) {
		rc = of_property_read_u16(pdev->dev.of_node,
			"qcom,msm-cpudai-tdm-clk-attribute",
			&tdm_clk_set.clk_attri);
		if (rc) {
			dev_err(&pdev->dev, "%s: value for clk attribute not found %s\n",
				__func__, "qcom,msm-cpudai-tdm-clk-attribute");
			goto rtn;
		}
		dev_dbg(&pdev->dev, "%s: clk attribute from DT file %d\n",
			__func__, tdm_clk_set.clk_attri);
	} else
		dev_dbg(&pdev->dev, "%s: clk attribute not found\n", __func__);

	/* extract tdm lane cfg to static */
	tdm_lane_cfg.port_id = tdm_group_cfg.group_id;
	tdm_lane_cfg.lane_mask = AFE_LANE_MASK_INVALID;
	if (of_find_property(pdev->dev.of_node,
			"qcom,msm-cpudai-tdm-lane-mask", NULL)) {
		rc = of_property_read_u16(pdev->dev.of_node,
			"qcom,msm-cpudai-tdm-lane-mask",
			&tdm_lane_cfg.lane_mask);
		if (rc) {
			dev_err(&pdev->dev, "%s: value for tdm lane mask not found %s\n",
				__func__, "qcom,msm-cpudai-tdm-lane-mask");
			goto rtn;
		}
		dev_dbg(&pdev->dev, "%s: tdm lane mask from DT file %d\n",
			__func__, tdm_lane_cfg.lane_mask);
	} else
		dev_dbg(&pdev->dev, "%s: tdm lane mask not found\n", __func__);

	/* extract tdm clk src master/slave info into static */
	rc = of_property_read_u32(pdev->dev.of_node,
		"qcom,msm-cpudai-tdm-clk-internal",
		&clk_mode);
	if (rc) {
		dev_err(&pdev->dev, "%s: Clk id from DT file %s\n",
			__func__, "qcom,msm-cpudai-tdm-clk-internal");
		goto rtn;
	}
	dev_dbg(&pdev->dev, "%s: Clk id from DT file %d\n",
		__func__, clk_mode);

	rc = msm_dai_q6_tdm_set_clk_param(tdm_group_cfg.group_id,
					  &tdm_clk_set, clk_mode);
	if (rc) {
		dev_err(&pdev->dev, "%s: group id not supported 0x%x\n",
			__func__, tdm_group_cfg.group_id);
		goto rtn;
	}

	/* other initializations within device group */
	group_idx = msm_dai_q6_get_group_idx(tdm_group_cfg.group_id);
	if (group_idx < 0) {
		dev_err(&pdev->dev, "%s: group id 0x%x not supported\n",
			__func__, tdm_group_cfg.group_id);
		rc = -EINVAL;
		goto rtn;
	}
	atomic_set(&tdm_group_ref[group_idx], 0);

	/* probe child node info */
	rc = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (rc) {
		dev_err(&pdev->dev, "%s: failed to add child nodes, rc=%d\n",
			__func__, rc);
		goto rtn;
	} else
		dev_dbg(&pdev->dev, "%s: added child node\n", __func__);

rtn:
	return rc;
}

static int msm_dai_tdm_q6_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id msm_dai_tdm_dt_match[] = {
	{ .compatible = "qcom,msm-dai-tdm", },
	{}
};

MODULE_DEVICE_TABLE(of, msm_dai_tdm_dt_match);

static struct platform_driver msm_dai_tdm_q6 = {
	.probe  = msm_dai_tdm_q6_probe,
	.remove = msm_dai_tdm_q6_remove,
	.driver = {
		.name = "msm-dai-tdm",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_tdm_dt_match,
		.suppress_bind_attrs = true,
	},
};

static int msm_dai_q6_tdm_data_format_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_tdm_dai_data *dai_data = kcontrol->private_data;
	int value = ucontrol->value.integer.value[0];

	switch (value) {
	case 0:
		dai_data->port_cfg.tdm.data_format = AFE_LINEAR_PCM_DATA;
		break;
	case 1:
		dai_data->port_cfg.tdm.data_format = AFE_NON_LINEAR_DATA;
		break;
	case 2:
		dai_data->port_cfg.tdm.data_format = AFE_GENERIC_COMPRESSED;
		break;
	default:
		pr_err("%s: data_format invalid\n", __func__);
		break;
	}
	pr_debug("%s: data_format = %d\n",
		__func__, dai_data->port_cfg.tdm.data_format);
	return 0;
}

static int msm_dai_q6_tdm_data_format_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_tdm_dai_data *dai_data = kcontrol->private_data;

	ucontrol->value.integer.value[0] =
		dai_data->port_cfg.tdm.data_format;
	pr_debug("%s: data_format = %d\n",
		__func__, dai_data->port_cfg.tdm.data_format);
	return 0;
}

static int msm_dai_q6_tdm_header_type_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_tdm_dai_data *dai_data = kcontrol->private_data;
	int value = ucontrol->value.integer.value[0];

	dai_data->port_cfg.custom_tdm_header.header_type = value;
	pr_debug("%s: header_type = %d\n",
		__func__,
		dai_data->port_cfg.custom_tdm_header.header_type);
	return 0;
}

static int msm_dai_q6_tdm_header_type_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_tdm_dai_data *dai_data = kcontrol->private_data;

	ucontrol->value.integer.value[0] =
		dai_data->port_cfg.custom_tdm_header.header_type;
	pr_debug("%s: header_type = %d\n",
		__func__,
		dai_data->port_cfg.custom_tdm_header.header_type);
	return 0;
}

static int msm_dai_q6_tdm_header_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_tdm_dai_data *dai_data = kcontrol->private_data;
	int i = 0;

	for (i = 0; i < AFE_CUSTOM_TDM_HEADER_MAX_CNT; i++) {
		dai_data->port_cfg.custom_tdm_header.header[i] =
			(u16)ucontrol->value.integer.value[i];
		pr_debug("%s: header #%d = 0x%x\n",
			__func__, i,
			dai_data->port_cfg.custom_tdm_header.header[i]);
	}
	return 0;
}

static int msm_dai_q6_tdm_header_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_tdm_dai_data *dai_data = kcontrol->private_data;
	int i = 0;

	for (i = 0; i < AFE_CUSTOM_TDM_HEADER_MAX_CNT; i++) {
		ucontrol->value.integer.value[i] =
			dai_data->port_cfg.custom_tdm_header.header[i];
		pr_debug("%s: header #%d = 0x%x\n",
			__func__, i,
			dai_data->port_cfg.custom_tdm_header.header[i]);
	}
	return 0;
}

static const struct snd_kcontrol_new tdm_config_controls_data_format[] = {
	SOC_ENUM_EXT("PRI_TDM_RX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_RX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_RX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_RX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_RX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_RX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_RX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_RX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_TX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_TX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_TX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_TX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_TX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_TX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_TX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("PRI_TDM_TX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_RX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_RX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_RX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_RX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_RX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_RX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_RX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_RX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_TX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_TX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_TX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_TX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_TX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_TX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_TX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEC_TDM_TX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_RX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_RX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_RX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_RX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_RX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_RX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_RX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_RX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_TX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_TX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_TX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_TX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_TX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_TX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_TX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("TERT_TDM_TX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_RX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_RX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_RX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_RX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_RX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_RX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_RX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_RX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_TX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_TX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_TX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_TX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_TX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_TX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_TX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEN_TDM_TX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_RX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_RX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_RX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_RX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_RX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_RX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_RX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_RX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_TX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_TX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_TX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_TX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_TX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_TX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_TX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("SEP_TDM_TX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_0 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_1 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_2 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_3 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_4 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_5 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_6 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_7 Data Format", tdm_config_enum[0],
			msm_dai_q6_tdm_data_format_get,
			msm_dai_q6_tdm_data_format_put),
};

static const struct snd_kcontrol_new tdm_config_controls_header_type[] = {
	SOC_ENUM_EXT("PRI_TDM_RX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_RX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_RX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_RX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_RX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_RX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_RX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_RX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_TX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_TX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_TX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_TX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_TX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_TX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_TX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("PRI_TDM_TX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_RX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_RX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_RX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_RX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_RX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_RX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_RX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_RX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_TX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_TX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_TX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_TX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_TX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_TX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_TX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEC_TDM_TX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_RX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_RX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_RX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_RX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_RX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_RX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_RX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_RX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_TX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_TX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_TX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_TX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_TX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_TX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_TX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("TERT_TDM_TX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_RX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUAT_TDM_TX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_RX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("QUIN_TDM_TX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_RX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_RX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_RX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_RX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_RX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_RX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_RX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_RX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_TX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_TX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_TX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_TX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_TX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_TX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_TX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEN_TDM_TX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_RX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_RX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_RX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_RX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_RX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_RX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_RX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_RX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_TX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_TX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_TX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_TX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_TX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_TX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_TX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("SEP_TDM_TX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_RX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF0_TDM_TX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_RX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF1_TDM_TX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_RX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_0 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_1 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_2 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_3 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_4 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_5 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_6 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
	SOC_ENUM_EXT("HSIF2_TDM_TX_7 Header Type", tdm_config_enum[1],
			msm_dai_q6_tdm_header_type_get,
			msm_dai_q6_tdm_header_type_put),
};

static const struct snd_kcontrol_new tdm_config_controls_header[] = {
	SOC_SINGLE_MULTI_EXT("PRI_TDM_RX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_RX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_RX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_RX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_RX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_RX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_RX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_RX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_TX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_TX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_TX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_TX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_TX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_TX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_TX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("PRI_TDM_TX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_RX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_RX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_RX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_RX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_RX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_RX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_RX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_RX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_TX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_TX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_TX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_TX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_TX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_TX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_TX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEC_TDM_TX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_RX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_RX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_RX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_RX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_RX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_RX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_RX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_RX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_TX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_TX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_TX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_TX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_TX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_TX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_TX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("TERT_TDM_TX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_RX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_RX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_RX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_RX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_RX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_RX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_RX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_RX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_TX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_TX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_TX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_TX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_TX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_TX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_TX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUAT_TDM_TX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_RX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_RX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_RX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_RX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_RX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_RX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_RX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_RX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_TX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_TX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_TX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_TX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_TX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_TX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_TX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("QUIN_TDM_TX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_RX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_RX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_RX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_RX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_RX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_RX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_RX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_RX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_TX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_TX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_TX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_TX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_TX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_TX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_TX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEN_TDM_TX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_RX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_RX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_RX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_RX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_RX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_RX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_RX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_RX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_TX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_TX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_TX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_TX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_TX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_TX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_TX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("SEP_TDM_TX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_RX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_RX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_RX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_RX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_RX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_RX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_RX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_RX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_TX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_TX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_TX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_TX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_TX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_TX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_TX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF0_TDM_TX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_RX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_RX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_RX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_RX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_RX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_RX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_RX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_RX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_TX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_TX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_TX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_TX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_TX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_TX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_TX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF1_TDM_TX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_RX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_RX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_RX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_RX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_RX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_RX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_RX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_RX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_TX_0 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_TX_1 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_TX_2 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_TX_3 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_TX_4 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_TX_5 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_TX_6 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
	SOC_SINGLE_MULTI_EXT("HSIF2_TDM_TX_7 Header",
			SND_SOC_NOPM, 0, 0xFFFFFFFF, 0, 8,
			msm_dai_q6_tdm_header_get,
			msm_dai_q6_tdm_header_put),
};

static int msm_dai_q6_tdm_set_clk(
		struct msm_dai_q6_tdm_dai_data *dai_data,
		u16 port_id, bool enable)
{
	int rc = 0;

	dai_data->clk_set.enable = enable;

	rc = afe_set_lpass_clock_v2(port_id,
		&dai_data->clk_set);
	if (rc < 0)
		pr_err("%s: afe lpass clock failed, err:%d\n",
			__func__, rc);

	return rc;
}

static int msm_pcm_afe_port_logging_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info* ucontrol)
{
	ucontrol->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	/* two int values: port_id and enable/disable */
	ucontrol->count = 2;
	/* Valid range is all positive values to support above controls */
	ucontrol->value.integer.min = 0;
	ucontrol->value.integer.max = INT_MAX;
	return 0;
}

static int msm_pcm_afe_port_logging_ctl_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int port_idx = afe_port_logging_port_id - AFE_PORT_ID_TDM_PORT_RANGE_START;

	ucontrol->value.integer.value[0] = afe_port_logging_port_id;
	if (port_idx < 0 || port_idx >= IDX_TDM_MAX) {
		pr_err_ratelimited("%s: port_idx = %d\n", __func__, port_idx);
		return -EINVAL;
	}
	ucontrol->value.integer.value[1] = afe_port_logging_item[port_idx];

	return 0;
}

static int msm_pcm_afe_port_logging_ctl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	u16 port_id;
	struct afe_param_id_port_data_log_disable_t log_disable;
	struct param_hdr_v3 param_hdr;
	int ret = -EINVAL, port_idx;

	pr_debug("%s: enter\n", __func__);
	memset(&param_hdr, 0, sizeof(param_hdr));

	port_id = ucontrol->value.integer.value[0];
	log_disable.disable_logging_flag = ucontrol->value.integer.value[1];

	ret = afe_port_send_logging_cfg(port_id, &log_disable);
	if (ret)
		pr_err("%s: AFE port logging setting for port 0x%x failed %d\n",
			__func__, port_id, ret);

	afe_port_logging_port_id = port_id;
	port_idx = port_id - AFE_PORT_ID_TDM_PORT_RANGE_START;
	if (port_idx < 0 || port_idx >= IDX_TDM_MAX) {
		pr_err_ratelimited("%s: port_idx = %d\n", __func__, port_idx);
		return -EINVAL;
	}
	afe_port_logging_item[port_idx] = ucontrol->value.integer.value[1];

	return ret;
}

static int msm_pcm_add_afe_port_logging_control(struct snd_soc_dai *dai)
{
	const char* afe_port_logging_ctl_name = "AFE_port_logging_disable";
	int rc = 0;
	struct snd_kcontrol_new *knew = NULL;
	struct snd_kcontrol* kctl = NULL;

	/* Add AFE port logging controls */
	knew = kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!knew) {
		return -ENOMEM;
	}
	knew->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	knew->info = msm_pcm_afe_port_logging_info;
	knew->get = msm_pcm_afe_port_logging_ctl_get;
	knew->put = msm_pcm_afe_port_logging_ctl_put;
	knew->name = afe_port_logging_ctl_name;
	knew->private_value = dai->id;
	kctl = snd_ctl_new1(knew, knew);
	if (!kctl) {
		kfree(knew);
		return -ENOMEM;
	}

	rc = snd_ctl_add(dai->component->card->snd_card, kctl);
	if (rc < 0)
		pr_err("%s: err add AFE port logging disable control, DAI = %s\n",
			__func__, dai->name);

	return rc;
}

static int get_global_clk_root(int value)
{
	if ((value < 0) || (value > ARRAY_SIZE(afe_dyn_clk_root_enum) - 1)) {
		pr_err("%s, %d set clk_root range failed\n", __func__, __LINE__);
		return global_dyn_mclk_cfg.clk_root;
	}
	clk_root_index = value;
	return  afe_dyn_clk_root_enum[clk_root_index];
}

static int get_global_clk_attri(int value)
{
	if ((value < 0) || (value > ARRAY_SIZE(afe_dyn_clk_attri_enum) - 1)) {
		pr_err("%s, %d set clk_attri range failed\n", __func__, __LINE__);
		return global_dyn_mclk_cfg.clk_attri;
	}
	clk_attri_index = value;
	return  afe_dyn_clk_attri_enum[clk_attri_index];
}

static int get_global_clk_id(int value)
{
	if ((value < 0) || (value > ARRAY_SIZE(afe_dyn_clk_id_enum) - 1)) {
		pr_err("%s, %d set clk_id range failed\n", __func__, __LINE__);
		return global_dyn_mclk_cfg.clk_id;
	}
	clk_id_index = value;
	return afe_dyn_clk_id_enum[clk_id_index];
}

static int msm_pcm_afe_dyn_mclk_ctl_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *ucontrol)
{
	ucontrol->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	/* 11 int values:clk_set_minor_version, clk_id, clk_freq_in_hz, clk_attri,
	 * clk_root, enable, divider_2x, m, n, d
	 */
	ucontrol->count = 11;
	ucontrol->value.integer.min = 0;
	ucontrol->value.integer.max = INT_MAX;
	/* Valid range is all positive values to support above controls */
	pr_debug("%s,%d\n", __func__, __LINE__);

	return 0;
}

static int msm_pcm_afe_dyn_mclk_ctl_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = global_dyn_mclk_cfg_portid;
	ucontrol->value.integer.value[1] = clk_id_index;
	ucontrol->value.integer.value[2] = global_dyn_mclk_cfg.clk_freq_in_hz;
	ucontrol->value.integer.value[3] = clk_attri_index;
	ucontrol->value.integer.value[4] = clk_root_index;
	ucontrol->value.integer.value[5] = global_dyn_mclk_cfg.enable;
	ucontrol->value.integer.value[6] = global_dyn_mclk_cfg.divider_2x;
	ucontrol->value.integer.value[7] = global_dyn_mclk_cfg.m;
	ucontrol->value.integer.value[8] = global_dyn_mclk_cfg.n;
	ucontrol->value.integer.value[9] = global_dyn_mclk_cfg.d;
	ucontrol->value.integer.value[10] = global_dyn_mclk_cfg.clk_set_minor_version;

	pr_debug("%s:%d.... portid:%d, clk_id:%d, clk_freq_in_hz:%d, clk_attri:%d, clk_root:%d, enable:%d, divider_2x:%d, m:%d, n:%d, d:%d, version:%d\n",
		__func__, __LINE__, global_dyn_mclk_cfg_portid, global_dyn_mclk_cfg.clk_id,
		global_dyn_mclk_cfg.clk_freq_in_hz, global_dyn_mclk_cfg.clk_attri,
		global_dyn_mclk_cfg.clk_root, global_dyn_mclk_cfg.enable,
		global_dyn_mclk_cfg.divider_2x, global_dyn_mclk_cfg.m, global_dyn_mclk_cfg.n,
		global_dyn_mclk_cfg.d, global_dyn_mclk_cfg.clk_set_minor_version);

	return 0;
}

static int msm_pcm_afe_dyn_mclk_ctl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;

	pr_debug("%s: enter\n", __func__);

	global_dyn_mclk_cfg_portid = ucontrol->value.integer.value[0];

	global_dyn_mclk_cfg.clk_id = get_global_clk_id(ucontrol->value.integer.value[1]);

	if (ucontrol->value.integer.value[2] >= 0)
		global_dyn_mclk_cfg.clk_freq_in_hz = ucontrol->value.integer.value[2];

	global_dyn_mclk_cfg.clk_attri = get_global_clk_attri(ucontrol->value.integer.value[3]);

	global_dyn_mclk_cfg.clk_root = get_global_clk_root(ucontrol->value.integer.value[4]);

	if ((ucontrol->value.integer.value[5] <= 1) && (ucontrol->value.integer.value[5] >= 0))
		global_dyn_mclk_cfg.enable = ucontrol->value.integer.value[5];

	global_dyn_mclk_cfg.divider_2x = ucontrol->value.integer.value[6];
	global_dyn_mclk_cfg.m = ucontrol->value.integer.value[7];
	global_dyn_mclk_cfg.n = ucontrol->value.integer.value[8];
	global_dyn_mclk_cfg.d = ucontrol->value.integer.value[9];

	pr_debug("%s:%d.... portid:%d, clk_id_index:%d, clk_freq_in_hz:%d, clk_attri_index:%d, clk_root_index:%d, enable:%d, divider_2x:%d, m:%d, n:%d, d:%d, version:%d\n",
		__func__, __LINE__, ucontrol->value.integer.value[0],
		 ucontrol->value.integer.value[1],
		ucontrol->value.integer.value[2], ucontrol->value.integer.value[3],
		ucontrol->value.integer.value[4], ucontrol->value.integer.value[5],
		ucontrol->value.integer.value[6], ucontrol->value.integer.value[7],
		ucontrol->value.integer.value[8], ucontrol->value.integer.value[9]);

	ret = afe_set_lpass_clk_cfg_ext_mclk_v2(global_dyn_mclk_cfg_portid,
		&global_dyn_mclk_cfg, 0);
	if (ret)
		pr_err("%s: AFE port logging setting for port 0x%x failed %d\n",
			__func__, global_dyn_mclk_cfg_portid, ret);

	return ret;
}

static int msm_pcm_add_afe_dyn_mclk_control(struct snd_soc_dai *dai)
{
	int rc = 0;
	struct snd_kcontrol_new *knew = NULL;
	struct snd_kcontrol *kctl = NULL;
	const char *afe_dyn_mclk_ctl_name = "AFE_dyn_switch_mclk_source";

	/* Add AFE port logging controls */
	knew = kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!knew)
		return -ENOMEM;

	knew->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	knew->info = msm_pcm_afe_dyn_mclk_ctl_info;
	knew->get = msm_pcm_afe_dyn_mclk_ctl_get;
	knew->put = msm_pcm_afe_dyn_mclk_ctl_put;
	knew->name = afe_dyn_mclk_ctl_name;
	knew->private_value = dai->id;
	kctl = snd_ctl_new1(knew, knew);
	if (!kctl) {
		kfree(knew);
		return -ENOMEM;
	}

	rc = snd_ctl_add(dai->component->card->snd_card, kctl);
	if (rc < 0)
		pr_err("%s: err add AFE dyn mclk control, DAI = %s\n",
			__func__, dai->name);

	return rc;
}

int jitter_cleaner_afe_enable_mclk_and_get_info_cb_func(void *private_data,
			uint32_t enable, uint32_t mclk_freq,
			struct afe_param_id_clock_set_v2_t *dyn_mclk_cfg)
{
	pr_debug("%s,%d\n", __func__, __LINE__);
	return 0;
}

static int msm_pcm_afe_limiter_param_ctl_info(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info* ucontrol)
{
	ucontrol->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	/* two int values: port_id and enable/disable */
	ucontrol->count = LIMITER_PARM_MAX;
	/* Valid range is all positive values to support above controls */
	ucontrol->value.integer.min = 0;
	ucontrol->value.integer.max = INT_MAX;
	return 0;
}

static int msm_pcm_afe_limiter_param_ctl_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = limiter_param[0];
	ucontrol->value.integer.value[1] = limiter_param[1];
	ucontrol->value.integer.value[2] = limiter_param[2];
	return 0;
}

static int msm_pcm_afe_limiter_param_ctl_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	u16 port_id;
	struct afe_param_id_port_afe_limiter_disable_t afe_limiter_disable;
	struct param_hdr_v3 param_hdr;
	int ret = -EINVAL;

	limiter_param[0] = ucontrol->value.integer.value[0];
	limiter_param[1] = ucontrol->value.integer.value[1];
	limiter_param[2] = ucontrol->value.integer.value[2];


	if(limiter_param[2] != LIMITER_PARM_MAX - 1)
	{
		return 0;
	}

	pr_debug("%s: enter\n", __func__);
	memset(&param_hdr, 0, sizeof(param_hdr));

	port_id = limiter_param[0];
	afe_limiter_disable.disable_afe_limiter = limiter_param[1];

	ret = afe_port_send_afe_limiter_param(port_id, &afe_limiter_disable);
	if (ret)
		pr_err("%s: AFE port logging setting for port 0x%x failed %d\n",
			__func__, port_id, ret);

	//resetting number of parameters to zero
	limiter_param[2] = 0;

	return ret;
}

static int msm_pcm_add_afe_port_limiter_control(struct snd_soc_dai *dai)
{
	const char* afe_port_limiter_ctl_name = "AFE_port_limiter_disable";
	int rc = 0;
	struct snd_kcontrol_new *knew = NULL;
	struct snd_kcontrol* kctl = NULL;

	/* Add AFE port logging controls */
	knew = kzalloc(sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!knew) {
		return -ENOMEM;
	}
	knew->iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	knew->info = msm_pcm_afe_limiter_param_ctl_info;
	knew->get = msm_pcm_afe_limiter_param_ctl_get;
	knew->put = msm_pcm_afe_limiter_param_ctl_put;
	knew->name = afe_port_limiter_ctl_name;
	knew->private_value = dai->id;
	kctl = snd_ctl_new1(knew, knew);
	if (!kctl) {
		kfree(knew);
		return -ENOMEM;
	}

	rc = snd_ctl_add(dai->component->card->snd_card, kctl);
	if (rc < 0)
		pr_err("%s: err add AFE port limiter disable control, DAI = %s\n",
			__func__, dai->name);

	return rc;
}

static int msm_dai_q6_dai_tdm_probe(struct snd_soc_dai *dai)
{
	int rc = 0;
	int port_idx = 0;
	struct msm_dai_q6_tdm_dai_data *tdm_dai_data = NULL;
	struct snd_kcontrol *data_format_kcontrol = NULL;
	struct snd_kcontrol *header_type_kcontrol = NULL;
	struct snd_kcontrol *header_kcontrol = NULL;
	const struct snd_kcontrol_new *data_format_ctrl = NULL;
	const struct snd_kcontrol_new *header_type_ctrl = NULL;
	const struct snd_kcontrol_new *header_ctrl = NULL;

	tdm_dai_data = dev_get_drvdata(dai->dev);

	msm_dai_q6_set_dai_id(dai);

	port_idx = msm_dai_q6_get_port_idx(dai->id);
	if (port_idx < 0) {
		dev_err(dai->dev, "%s port id 0x%x not supported\n",
			__func__, dai->id);
		rc = -EINVAL;
		goto rtn;
	}

	data_format_ctrl =
		&tdm_config_controls_data_format[port_idx];
	header_type_ctrl =
		&tdm_config_controls_header_type[port_idx];
	header_ctrl =
		&tdm_config_controls_header[port_idx];

	if (data_format_ctrl) {
		data_format_kcontrol = snd_ctl_new1(data_format_ctrl,
					tdm_dai_data);
		rc = snd_ctl_add(dai->component->card->snd_card,
				 data_format_kcontrol);
		if (rc < 0) {
			dev_err(dai->dev, "%s: err add data format ctrl DAI = %s\n",
				__func__, dai->name);
			goto rtn;
		}
	}

	if (header_type_ctrl) {
		header_type_kcontrol = snd_ctl_new1(header_type_ctrl,
					tdm_dai_data);
		rc = snd_ctl_add(dai->component->card->snd_card,
				 header_type_kcontrol);
		if (rc < 0) {
			if (data_format_kcontrol)
				snd_ctl_remove(dai->component->card->snd_card,
					data_format_kcontrol);
			dev_err(dai->dev, "%s: err add header type ctrl DAI = %s\n",
				__func__, dai->name);
			goto rtn;
		}
	}

	if (header_ctrl) {
		header_kcontrol = snd_ctl_new1(header_ctrl,
					tdm_dai_data);
		rc = snd_ctl_add(dai->component->card->snd_card,
				 header_kcontrol);
		if (rc < 0) {
			if (header_type_kcontrol)
				snd_ctl_remove(dai->component->card->snd_card,
					header_type_kcontrol);
			if (data_format_kcontrol)
				snd_ctl_remove(dai->component->card->snd_card,
					data_format_kcontrol);
			dev_err(dai->dev, "%s: err add header ctrl DAI = %s\n",
				__func__, dai->name);
			goto rtn;
		}
	}

	/* add AFE port logging controls */
	if (!afe_port_loggging_control_added) {
		rc = msm_pcm_add_afe_port_logging_control(dai);
		if (rc < 0) {
			dev_err(dai->dev, "%s: add AFE port logging control failed DAI: %s\n",
				__func__, dai->name);
			goto rtn;
		}

		afe_port_loggging_control_added = 1;
	}

	/* add AFE dyn mclk controls */
	if ((!afe_dyn_mclk_control_added) && (jitter_cleaner_enable)) {
		rc = msm_pcm_add_afe_dyn_mclk_control(dai);
		if (rc < 0) {
			dev_err(dai->dev, "%s: add AFE dyn mclk control failed DAI: %s\n",
				__func__, dai->name);
			goto rtn;
		}

		afe_dyn_mclk_control_added = 1;

		rc = afe_register_ext_mclk_cb(jitter_cleaner_afe_enable_mclk_and_get_info_cb_func,
			(void *)&global_dyn_mclk_cfg);
		if (rc < 0) {
			dev_err(dai->dev, "%s: add AFE afe_register_ext_mclk_cb failed : %s\n",
				__func__, dai->name);
			goto rtn;
		}
	}

	if (!afe_port_limiter_control_added) {
		rc = msm_pcm_add_afe_port_limiter_control(dai);
		if (rc < 0) {
			dev_err(dai->dev, "%s: add AFE port logging control failed DAI: %s\n",
				__func__, dai->name);
			goto rtn;
		}

		afe_port_limiter_control_added = 1;
	}

	if (tdm_dai_data->is_island_dai)
		rc = msm_dai_q6_add_island_mx_ctls(
						dai->component->card->snd_card,
						dai->name,
						dai->id, (void *)tdm_dai_data);

	rc = msm_dai_q6_dai_add_route(dai);

rtn:
	return rc;
}


static int msm_dai_q6_dai_tdm_remove(struct snd_soc_dai *dai)
{
	int rc = 0;
	struct msm_dai_q6_tdm_dai_data *tdm_dai_data =
		dev_get_drvdata(dai->dev);
	u16 group_id = tdm_dai_data->group_cfg.tdm_cfg.group_id;
	int group_idx = 0;
	atomic_t *group_ref = NULL;

	group_idx = msm_dai_q6_get_group_idx(dai->id);
	if (group_idx < 0) {
		dev_err(dai->dev, "%s port id 0x%x not supported\n",
			__func__, dai->id);
		return -EINVAL;
	}

	group_ref = &tdm_group_ref[group_idx];

	/* If AFE port is still up, close it */
	if (test_bit(STATUS_PORT_STARTED, tdm_dai_data->status_mask)) {
		rc = afe_close(dai->id); /* can block */
		if (rc < 0) {
			dev_err(dai->dev, "%s: fail to close AFE port 0x%x\n",
				__func__, dai->id);
		}
		atomic_dec(group_ref);
		clear_bit(STATUS_PORT_STARTED,
			  tdm_dai_data->status_mask);

		if (atomic_read(group_ref) == 0) {
			rc = afe_port_group_enable(group_id,
				NULL, false, NULL);
			if (rc < 0) {
				dev_err(dai->dev, "fail to disable AFE group 0x%x\n",
					group_id);
			}
		}

		if (msm_dai_q6_get_tdm_clk_ref(group_idx) == 0) {
			rc = msm_dai_q6_tdm_set_clk(tdm_dai_data,
				dai->id, false);
			if (rc < 0) {
				dev_err(dai->dev, "%s: fail to disable AFE clk 0x%x\n",
					__func__, dai->id);
			}
		}
	}

	return 0;
}

static int msm_dai_q6_tdm_set_tdm_slot(struct snd_soc_dai *dai,
				unsigned int tx_mask,
				unsigned int rx_mask,
				int slots, int slot_width)
{
	int rc = 0;
	struct msm_dai_q6_tdm_dai_data *dai_data =
		dev_get_drvdata(dai->dev);
	struct afe_param_id_group_device_tdm_cfg *tdm_group =
		&dai_data->group_cfg.tdm_cfg;
	unsigned int cap_mask;

	dev_dbg(dai->dev, "%s: dai id = 0x%x\n", __func__, dai->id);

	/* HW only supports 16 and 32 bit slot width configuration */
	if ((slot_width != 16) && (slot_width != 32)) {
		dev_err(dai->dev, "%s: invalid slot_width %d\n",
			__func__, slot_width);
		return -EINVAL;
	}

	/* HW supports 1-32 slots configuration. Typical: 1, 2, 4, 8, 16, 32 */
	switch (slots) {
	case 1:
		cap_mask = 0x01;
		break;
	case 2:
		cap_mask = 0x03;
		break;
	case 4:
		cap_mask = 0x0F;
		break;
	case 8:
		cap_mask = 0xFF;
		break;
	case 16:
		cap_mask = 0xFFFF;
		break;
	case 32:
		cap_mask = 0xFFFFFFFF;
		break;
	default:
		dev_err(dai->dev, "%s: invalid slots %d\n",
			__func__, slots);
		return -EINVAL;
	}

	switch (dai->id) {
	case AFE_PORT_ID_PRIMARY_TDM_RX:
	case AFE_PORT_ID_PRIMARY_TDM_RX_1:
	case AFE_PORT_ID_PRIMARY_TDM_RX_2:
	case AFE_PORT_ID_PRIMARY_TDM_RX_3:
	case AFE_PORT_ID_PRIMARY_TDM_RX_4:
	case AFE_PORT_ID_PRIMARY_TDM_RX_5:
	case AFE_PORT_ID_PRIMARY_TDM_RX_6:
	case AFE_PORT_ID_PRIMARY_TDM_RX_7:
	case AFE_PORT_ID_SECONDARY_TDM_RX:
	case AFE_PORT_ID_SECONDARY_TDM_RX_1:
	case AFE_PORT_ID_SECONDARY_TDM_RX_2:
	case AFE_PORT_ID_SECONDARY_TDM_RX_3:
	case AFE_PORT_ID_SECONDARY_TDM_RX_4:
	case AFE_PORT_ID_SECONDARY_TDM_RX_5:
	case AFE_PORT_ID_SECONDARY_TDM_RX_6:
	case AFE_PORT_ID_SECONDARY_TDM_RX_7:
	case AFE_PORT_ID_TERTIARY_TDM_RX:
	case AFE_PORT_ID_TERTIARY_TDM_RX_1:
	case AFE_PORT_ID_TERTIARY_TDM_RX_2:
	case AFE_PORT_ID_TERTIARY_TDM_RX_3:
	case AFE_PORT_ID_TERTIARY_TDM_RX_4:
	case AFE_PORT_ID_TERTIARY_TDM_RX_5:
	case AFE_PORT_ID_TERTIARY_TDM_RX_6:
	case AFE_PORT_ID_TERTIARY_TDM_RX_7:
	case AFE_PORT_ID_QUATERNARY_TDM_RX:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_1:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_2:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_3:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_4:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_5:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_6:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_7:
	case AFE_PORT_ID_QUINARY_TDM_RX:
	case AFE_PORT_ID_QUINARY_TDM_RX_1:
	case AFE_PORT_ID_QUINARY_TDM_RX_2:
	case AFE_PORT_ID_QUINARY_TDM_RX_3:
	case AFE_PORT_ID_QUINARY_TDM_RX_4:
	case AFE_PORT_ID_QUINARY_TDM_RX_5:
	case AFE_PORT_ID_QUINARY_TDM_RX_6:
	case AFE_PORT_ID_QUINARY_TDM_RX_7:
	case AFE_PORT_ID_SENARY_TDM_RX:
	case AFE_PORT_ID_SENARY_TDM_RX_1:
	case AFE_PORT_ID_SENARY_TDM_RX_2:
	case AFE_PORT_ID_SENARY_TDM_RX_3:
	case AFE_PORT_ID_SENARY_TDM_RX_4:
	case AFE_PORT_ID_SENARY_TDM_RX_5:
	case AFE_PORT_ID_SENARY_TDM_RX_6:
	case AFE_PORT_ID_SENARY_TDM_RX_7:
	case AFE_PORT_ID_SEPTENARY_TDM_RX:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_1:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_2:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_3:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_4:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_5:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_6:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_7:
	case AFE_PORT_ID_HSIF0_TDM_RX:
	case AFE_PORT_ID_HSIF0_TDM_RX_1:
	case AFE_PORT_ID_HSIF0_TDM_RX_2:
	case AFE_PORT_ID_HSIF0_TDM_RX_3:
	case AFE_PORT_ID_HSIF0_TDM_RX_4:
	case AFE_PORT_ID_HSIF0_TDM_RX_5:
	case AFE_PORT_ID_HSIF0_TDM_RX_6:
	case AFE_PORT_ID_HSIF0_TDM_RX_7:
	case AFE_PORT_ID_HSIF1_TDM_RX:
	case AFE_PORT_ID_HSIF1_TDM_RX_1:
	case AFE_PORT_ID_HSIF1_TDM_RX_2:
	case AFE_PORT_ID_HSIF1_TDM_RX_3:
	case AFE_PORT_ID_HSIF1_TDM_RX_4:
	case AFE_PORT_ID_HSIF1_TDM_RX_5:
	case AFE_PORT_ID_HSIF1_TDM_RX_6:
	case AFE_PORT_ID_HSIF1_TDM_RX_7:
	case AFE_PORT_ID_HSIF2_TDM_RX:
	case AFE_PORT_ID_HSIF2_TDM_RX_1:
	case AFE_PORT_ID_HSIF2_TDM_RX_2:
	case AFE_PORT_ID_HSIF2_TDM_RX_3:
	case AFE_PORT_ID_HSIF2_TDM_RX_4:
	case AFE_PORT_ID_HSIF2_TDM_RX_5:
	case AFE_PORT_ID_HSIF2_TDM_RX_6:
	case AFE_PORT_ID_HSIF2_TDM_RX_7:
		tdm_group->nslots_per_frame = slots;
		tdm_group->slot_width = slot_width;
		tdm_group->slot_mask = rx_mask & cap_mask;
		break;
	case AFE_PORT_ID_PRIMARY_TDM_TX:
	case AFE_PORT_ID_PRIMARY_TDM_TX_1:
	case AFE_PORT_ID_PRIMARY_TDM_TX_2:
	case AFE_PORT_ID_PRIMARY_TDM_TX_3:
	case AFE_PORT_ID_PRIMARY_TDM_TX_4:
	case AFE_PORT_ID_PRIMARY_TDM_TX_5:
	case AFE_PORT_ID_PRIMARY_TDM_TX_6:
	case AFE_PORT_ID_PRIMARY_TDM_TX_7:
	case AFE_PORT_ID_SECONDARY_TDM_TX:
	case AFE_PORT_ID_SECONDARY_TDM_TX_1:
	case AFE_PORT_ID_SECONDARY_TDM_TX_2:
	case AFE_PORT_ID_SECONDARY_TDM_TX_3:
	case AFE_PORT_ID_SECONDARY_TDM_TX_4:
	case AFE_PORT_ID_SECONDARY_TDM_TX_5:
	case AFE_PORT_ID_SECONDARY_TDM_TX_6:
	case AFE_PORT_ID_SECONDARY_TDM_TX_7:
	case AFE_PORT_ID_TERTIARY_TDM_TX:
	case AFE_PORT_ID_TERTIARY_TDM_TX_1:
	case AFE_PORT_ID_TERTIARY_TDM_TX_2:
	case AFE_PORT_ID_TERTIARY_TDM_TX_3:
	case AFE_PORT_ID_TERTIARY_TDM_TX_4:
	case AFE_PORT_ID_TERTIARY_TDM_TX_5:
	case AFE_PORT_ID_TERTIARY_TDM_TX_6:
	case AFE_PORT_ID_TERTIARY_TDM_TX_7:
	case AFE_PORT_ID_QUATERNARY_TDM_TX:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_1:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_2:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_3:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_4:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_5:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_6:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_7:
	case AFE_PORT_ID_QUINARY_TDM_TX:
	case AFE_PORT_ID_QUINARY_TDM_TX_1:
	case AFE_PORT_ID_QUINARY_TDM_TX_2:
	case AFE_PORT_ID_QUINARY_TDM_TX_3:
	case AFE_PORT_ID_QUINARY_TDM_TX_4:
	case AFE_PORT_ID_QUINARY_TDM_TX_5:
	case AFE_PORT_ID_QUINARY_TDM_TX_6:
	case AFE_PORT_ID_QUINARY_TDM_TX_7:
	case AFE_PORT_ID_SENARY_TDM_TX:
	case AFE_PORT_ID_SENARY_TDM_TX_1:
	case AFE_PORT_ID_SENARY_TDM_TX_2:
	case AFE_PORT_ID_SENARY_TDM_TX_3:
	case AFE_PORT_ID_SENARY_TDM_TX_4:
	case AFE_PORT_ID_SENARY_TDM_TX_5:
	case AFE_PORT_ID_SENARY_TDM_TX_6:
	case AFE_PORT_ID_SENARY_TDM_TX_7:
	case AFE_PORT_ID_SEPTENARY_TDM_TX:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_1:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_2:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_3:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_4:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_5:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_6:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_7:
	case AFE_PORT_ID_HSIF0_TDM_TX:
	case AFE_PORT_ID_HSIF0_TDM_TX_1:
	case AFE_PORT_ID_HSIF0_TDM_TX_2:
	case AFE_PORT_ID_HSIF0_TDM_TX_3:
	case AFE_PORT_ID_HSIF0_TDM_TX_4:
	case AFE_PORT_ID_HSIF0_TDM_TX_5:
	case AFE_PORT_ID_HSIF0_TDM_TX_6:
	case AFE_PORT_ID_HSIF0_TDM_TX_7:
	case AFE_PORT_ID_HSIF1_TDM_TX:
	case AFE_PORT_ID_HSIF1_TDM_TX_1:
	case AFE_PORT_ID_HSIF1_TDM_TX_2:
	case AFE_PORT_ID_HSIF1_TDM_TX_3:
	case AFE_PORT_ID_HSIF1_TDM_TX_4:
	case AFE_PORT_ID_HSIF1_TDM_TX_5:
	case AFE_PORT_ID_HSIF1_TDM_TX_6:
	case AFE_PORT_ID_HSIF1_TDM_TX_7:
	case AFE_PORT_ID_HSIF2_TDM_TX:
	case AFE_PORT_ID_HSIF2_TDM_TX_1:
	case AFE_PORT_ID_HSIF2_TDM_TX_2:
	case AFE_PORT_ID_HSIF2_TDM_TX_3:
	case AFE_PORT_ID_HSIF2_TDM_TX_4:
	case AFE_PORT_ID_HSIF2_TDM_TX_5:
	case AFE_PORT_ID_HSIF2_TDM_TX_6:
	case AFE_PORT_ID_HSIF2_TDM_TX_7:
		tdm_group->nslots_per_frame = slots;
		tdm_group->slot_width = slot_width;
		tdm_group->slot_mask = tx_mask & cap_mask;
		break;
	default:
		dev_err(dai->dev, "%s: invalid dai id 0x%x\n",
			__func__, dai->id);
		return -EINVAL;
	}

	return rc;
}

static int msm_dai_q6_tdm_set_sysclk(struct snd_soc_dai *dai,
				int clk_id, unsigned int freq, int dir)
{
	struct msm_dai_q6_tdm_dai_data *dai_data =
		dev_get_drvdata(dai->dev);

	if ((dai->id >= AFE_PORT_ID_PRIMARY_TDM_RX) &&
		(dai->id <= AFE_PORT_ID_HSIF2_TDM_TX_7)) {
		dai_data->clk_set.clk_freq_in_hz = freq;
	} else {
		dev_err(dai->dev, "%s: invalid dai id 0x%x\n",
			__func__, dai->id);
		return -EINVAL;
	}

	dev_dbg(dai->dev, "%s: dai id = 0x%x, group clk_freq = %d\n",
			__func__, dai->id, freq);
	return 0;
}

static int msm_dai_q6_tdm_set_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)
{
	int rc = 0;
	struct msm_dai_q6_tdm_dai_data *dai_data =
		dev_get_drvdata(dai->dev);
	struct afe_param_id_slot_mapping_cfg *slot_mapping =
		&dai_data->port_cfg.slot_mapping;
	struct afe_param_id_slot_mapping_cfg_v2 *slot_mapping_v2 =
		&dai_data->port_cfg.slot_mapping_v2;
	int i = 0;

	dev_dbg(dai->dev, "%s: dai id = 0x%x\n", __func__, dai->id);

	switch (dai->id) {
	case AFE_PORT_ID_PRIMARY_TDM_RX:
	case AFE_PORT_ID_PRIMARY_TDM_RX_1:
	case AFE_PORT_ID_PRIMARY_TDM_RX_2:
	case AFE_PORT_ID_PRIMARY_TDM_RX_3:
	case AFE_PORT_ID_PRIMARY_TDM_RX_4:
	case AFE_PORT_ID_PRIMARY_TDM_RX_5:
	case AFE_PORT_ID_PRIMARY_TDM_RX_6:
	case AFE_PORT_ID_PRIMARY_TDM_RX_7:
	case AFE_PORT_ID_SECONDARY_TDM_RX:
	case AFE_PORT_ID_SECONDARY_TDM_RX_1:
	case AFE_PORT_ID_SECONDARY_TDM_RX_2:
	case AFE_PORT_ID_SECONDARY_TDM_RX_3:
	case AFE_PORT_ID_SECONDARY_TDM_RX_4:
	case AFE_PORT_ID_SECONDARY_TDM_RX_5:
	case AFE_PORT_ID_SECONDARY_TDM_RX_6:
	case AFE_PORT_ID_SECONDARY_TDM_RX_7:
	case AFE_PORT_ID_TERTIARY_TDM_RX:
	case AFE_PORT_ID_TERTIARY_TDM_RX_1:
	case AFE_PORT_ID_TERTIARY_TDM_RX_2:
	case AFE_PORT_ID_TERTIARY_TDM_RX_3:
	case AFE_PORT_ID_TERTIARY_TDM_RX_4:
	case AFE_PORT_ID_TERTIARY_TDM_RX_5:
	case AFE_PORT_ID_TERTIARY_TDM_RX_6:
	case AFE_PORT_ID_TERTIARY_TDM_RX_7:
	case AFE_PORT_ID_QUATERNARY_TDM_RX:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_1:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_2:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_3:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_4:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_5:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_6:
	case AFE_PORT_ID_QUATERNARY_TDM_RX_7:
	case AFE_PORT_ID_QUINARY_TDM_RX:
	case AFE_PORT_ID_QUINARY_TDM_RX_1:
	case AFE_PORT_ID_QUINARY_TDM_RX_2:
	case AFE_PORT_ID_QUINARY_TDM_RX_3:
	case AFE_PORT_ID_QUINARY_TDM_RX_4:
	case AFE_PORT_ID_QUINARY_TDM_RX_5:
	case AFE_PORT_ID_QUINARY_TDM_RX_6:
	case AFE_PORT_ID_QUINARY_TDM_RX_7:
	case AFE_PORT_ID_SENARY_TDM_RX:
	case AFE_PORT_ID_SENARY_TDM_RX_1:
	case AFE_PORT_ID_SENARY_TDM_RX_2:
	case AFE_PORT_ID_SENARY_TDM_RX_3:
	case AFE_PORT_ID_SENARY_TDM_RX_4:
	case AFE_PORT_ID_SENARY_TDM_RX_5:
	case AFE_PORT_ID_SENARY_TDM_RX_6:
	case AFE_PORT_ID_SENARY_TDM_RX_7:
	case AFE_PORT_ID_SEPTENARY_TDM_RX:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_1:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_2:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_3:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_4:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_5:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_6:
	case AFE_PORT_ID_SEPTENARY_TDM_RX_7:
	case AFE_PORT_ID_HSIF0_TDM_RX:
	case AFE_PORT_ID_HSIF0_TDM_RX_1:
	case AFE_PORT_ID_HSIF0_TDM_RX_2:
	case AFE_PORT_ID_HSIF0_TDM_RX_3:
	case AFE_PORT_ID_HSIF0_TDM_RX_4:
	case AFE_PORT_ID_HSIF0_TDM_RX_5:
	case AFE_PORT_ID_HSIF0_TDM_RX_6:
	case AFE_PORT_ID_HSIF0_TDM_RX_7:
	case AFE_PORT_ID_HSIF1_TDM_RX:
	case AFE_PORT_ID_HSIF1_TDM_RX_1:
	case AFE_PORT_ID_HSIF1_TDM_RX_2:
	case AFE_PORT_ID_HSIF1_TDM_RX_3:
	case AFE_PORT_ID_HSIF1_TDM_RX_4:
	case AFE_PORT_ID_HSIF1_TDM_RX_5:
	case AFE_PORT_ID_HSIF1_TDM_RX_6:
	case AFE_PORT_ID_HSIF1_TDM_RX_7:
	case AFE_PORT_ID_HSIF2_TDM_RX:
	case AFE_PORT_ID_HSIF2_TDM_RX_1:
	case AFE_PORT_ID_HSIF2_TDM_RX_2:
	case AFE_PORT_ID_HSIF2_TDM_RX_3:
	case AFE_PORT_ID_HSIF2_TDM_RX_4:
	case AFE_PORT_ID_HSIF2_TDM_RX_5:
	case AFE_PORT_ID_HSIF2_TDM_RX_6:
	case AFE_PORT_ID_HSIF2_TDM_RX_7:
		if (q6core_get_avcs_api_version_per_service(
			APRV2_IDS_SERVICE_ID_ADSP_AFE_V) >= AFE_API_VERSION_V3) {
			if (!rx_slot) {
				dev_err(dai->dev, "%s: rx slot not found\n",
						__func__);
				return -EINVAL;
			}
			if (rx_num > AFE_PORT_MAX_AUDIO_CHAN_CNT_V2) {
				dev_err(dai->dev, "%s: invalid rx num %d\n",
						__func__,
					rx_num);
				return -EINVAL;
			}

			for (i = 0; i < rx_num; i++)
				slot_mapping_v2->offset[i] = rx_slot[i];
			for (i = rx_num; i < AFE_PORT_MAX_AUDIO_CHAN_CNT_V2;
					i++)
				slot_mapping_v2->offset[i] =
					AFE_SLOT_MAPPING_OFFSET_INVALID;

			slot_mapping_v2->num_channel = rx_num;
		} else {
			if (!rx_slot) {
				dev_err(dai->dev, "%s: rx slot not found\n",
						__func__);
				return -EINVAL;
			}
			if (rx_num > AFE_PORT_MAX_AUDIO_CHAN_CNT) {
				dev_err(dai->dev, "%s: invalid rx num %d\n",
						__func__,
					rx_num);
				return -EINVAL;
			}

			for (i = 0; i < rx_num; i++)
				slot_mapping->offset[i] = rx_slot[i];
			for (i = rx_num; i < AFE_PORT_MAX_AUDIO_CHAN_CNT; i++)
				slot_mapping->offset[i] =
					AFE_SLOT_MAPPING_OFFSET_INVALID;

			slot_mapping->num_channel = rx_num;
		}
		break;
	case AFE_PORT_ID_PRIMARY_TDM_TX:
	case AFE_PORT_ID_PRIMARY_TDM_TX_1:
	case AFE_PORT_ID_PRIMARY_TDM_TX_2:
	case AFE_PORT_ID_PRIMARY_TDM_TX_3:
	case AFE_PORT_ID_PRIMARY_TDM_TX_4:
	case AFE_PORT_ID_PRIMARY_TDM_TX_5:
	case AFE_PORT_ID_PRIMARY_TDM_TX_6:
	case AFE_PORT_ID_PRIMARY_TDM_TX_7:
	case AFE_PORT_ID_SECONDARY_TDM_TX:
	case AFE_PORT_ID_SECONDARY_TDM_TX_1:
	case AFE_PORT_ID_SECONDARY_TDM_TX_2:
	case AFE_PORT_ID_SECONDARY_TDM_TX_3:
	case AFE_PORT_ID_SECONDARY_TDM_TX_4:
	case AFE_PORT_ID_SECONDARY_TDM_TX_5:
	case AFE_PORT_ID_SECONDARY_TDM_TX_6:
	case AFE_PORT_ID_SECONDARY_TDM_TX_7:
	case AFE_PORT_ID_TERTIARY_TDM_TX:
	case AFE_PORT_ID_TERTIARY_TDM_TX_1:
	case AFE_PORT_ID_TERTIARY_TDM_TX_2:
	case AFE_PORT_ID_TERTIARY_TDM_TX_3:
	case AFE_PORT_ID_TERTIARY_TDM_TX_4:
	case AFE_PORT_ID_TERTIARY_TDM_TX_5:
	case AFE_PORT_ID_TERTIARY_TDM_TX_6:
	case AFE_PORT_ID_TERTIARY_TDM_TX_7:
	case AFE_PORT_ID_QUATERNARY_TDM_TX:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_1:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_2:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_3:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_4:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_5:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_6:
	case AFE_PORT_ID_QUATERNARY_TDM_TX_7:
	case AFE_PORT_ID_QUINARY_TDM_TX:
	case AFE_PORT_ID_QUINARY_TDM_TX_1:
	case AFE_PORT_ID_QUINARY_TDM_TX_2:
	case AFE_PORT_ID_QUINARY_TDM_TX_3:
	case AFE_PORT_ID_QUINARY_TDM_TX_4:
	case AFE_PORT_ID_QUINARY_TDM_TX_5:
	case AFE_PORT_ID_QUINARY_TDM_TX_6:
	case AFE_PORT_ID_QUINARY_TDM_TX_7:
	case AFE_PORT_ID_SENARY_TDM_TX:
	case AFE_PORT_ID_SENARY_TDM_TX_1:
	case AFE_PORT_ID_SENARY_TDM_TX_2:
	case AFE_PORT_ID_SENARY_TDM_TX_3:
	case AFE_PORT_ID_SENARY_TDM_TX_4:
	case AFE_PORT_ID_SENARY_TDM_TX_5:
	case AFE_PORT_ID_SENARY_TDM_TX_6:
	case AFE_PORT_ID_SENARY_TDM_TX_7:
	case AFE_PORT_ID_SEPTENARY_TDM_TX:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_1:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_2:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_3:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_4:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_5:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_6:
	case AFE_PORT_ID_SEPTENARY_TDM_TX_7:
	case AFE_PORT_ID_HSIF0_TDM_TX:
	case AFE_PORT_ID_HSIF0_TDM_TX_1:
	case AFE_PORT_ID_HSIF0_TDM_TX_2:
	case AFE_PORT_ID_HSIF0_TDM_TX_3:
	case AFE_PORT_ID_HSIF0_TDM_TX_4:
	case AFE_PORT_ID_HSIF0_TDM_TX_5:
	case AFE_PORT_ID_HSIF0_TDM_TX_6:
	case AFE_PORT_ID_HSIF0_TDM_TX_7:
	case AFE_PORT_ID_HSIF1_TDM_TX:
	case AFE_PORT_ID_HSIF1_TDM_TX_1:
	case AFE_PORT_ID_HSIF1_TDM_TX_2:
	case AFE_PORT_ID_HSIF1_TDM_TX_3:
	case AFE_PORT_ID_HSIF1_TDM_TX_4:
	case AFE_PORT_ID_HSIF1_TDM_TX_5:
	case AFE_PORT_ID_HSIF1_TDM_TX_6:
	case AFE_PORT_ID_HSIF1_TDM_TX_7:
	case AFE_PORT_ID_HSIF2_TDM_TX:
	case AFE_PORT_ID_HSIF2_TDM_TX_1:
	case AFE_PORT_ID_HSIF2_TDM_TX_2:
	case AFE_PORT_ID_HSIF2_TDM_TX_3:
	case AFE_PORT_ID_HSIF2_TDM_TX_4:
	case AFE_PORT_ID_HSIF2_TDM_TX_5:
	case AFE_PORT_ID_HSIF2_TDM_TX_6:
	case AFE_PORT_ID_HSIF2_TDM_TX_7:
		if (q6core_get_avcs_api_version_per_service(
			APRV2_IDS_SERVICE_ID_ADSP_AFE_V) >= AFE_API_VERSION_V3) {
			if (!tx_slot) {
				dev_err(dai->dev, "%s: tx slot not found\n",
						__func__);
				return -EINVAL;
			}
			if (tx_num > AFE_PORT_MAX_AUDIO_CHAN_CNT_V2) {
				dev_err(dai->dev, "%s: invalid tx num %d\n",
						__func__,
					tx_num);
				return -EINVAL;
			}

			for (i = 0; i < tx_num; i++)
				slot_mapping_v2->offset[i] = tx_slot[i];
			for (i = tx_num; i < AFE_PORT_MAX_AUDIO_CHAN_CNT_V2;
					i++)
				slot_mapping_v2->offset[i] =
					AFE_SLOT_MAPPING_OFFSET_INVALID;

			slot_mapping_v2->num_channel = tx_num;
		} else {
			if (!tx_slot) {
				dev_err(dai->dev, "%s: tx slot not found\n",
						__func__);
				return -EINVAL;
			}
			if (tx_num > AFE_PORT_MAX_AUDIO_CHAN_CNT) {
				dev_err(dai->dev, "%s: invalid tx num %d\n",
						__func__,
					tx_num);
				return -EINVAL;
			}

			for (i = 0; i < tx_num; i++)
				slot_mapping->offset[i] = tx_slot[i];
			for (i = tx_num; i < AFE_PORT_MAX_AUDIO_CHAN_CNT; i++)
				slot_mapping->offset[i] =
					AFE_SLOT_MAPPING_OFFSET_INVALID;

			slot_mapping->num_channel = tx_num;
		}
		break;
	default:
		dev_err(dai->dev, "%s: invalid dai id 0x%x\n",
			__func__, dai->id);
		return -EINVAL;
	}

	return rc;
}

static unsigned int tdm_param_set_slot_mask(u16 *slot_offset, int slot_width,
					    int slots_per_frame, int num_channels)
{
	unsigned int i = 0;
	unsigned int slot_index = 0;
	unsigned long slot_mask = 0;
	unsigned int slot_width_bytes = slot_width / 8;
	unsigned int channel_count = AFE_PORT_MAX_AUDIO_CHAN_CNT;

	if (q6core_get_avcs_api_version_per_service(
		APRV2_IDS_SERVICE_ID_ADSP_AFE_V) >= AFE_API_VERSION_V3)
		channel_count = AFE_PORT_MAX_AUDIO_CHAN_CNT_V2;

	if (slot_width_bytes == 0) {
		pr_err("%s: slot width is zero\n", __func__);
		return slot_mask;
	}

	if (num_channels != slots_per_frame) {
		pr_debug("%s: multi lane is enabled, use the slot mask of tdm group\n", __func__);
		return slot_mask;
	}

	for (i = 0; i < channel_count; i++) {
		if (slot_offset[i] != AFE_SLOT_MAPPING_OFFSET_INVALID) {
			slot_index = slot_offset[i] / slot_width_bytes;
			if (slot_index < slots_per_frame)
				set_bit(slot_index, &slot_mask);
			else {
				pr_err("%s: invalid slot map setting\n",
				       __func__);
				return 0;
			}
		} else {
			break;
		}
	}

	return slot_mask;
}

static int msm_dai_q6_tdm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct msm_dai_q6_tdm_dai_data *dai_data =
		dev_get_drvdata(dai->dev);

	struct afe_param_id_group_device_tdm_cfg *tdm_group =
		&dai_data->group_cfg.tdm_cfg;
	struct afe_param_id_tdm_cfg *tdm =
		&dai_data->port_cfg.tdm;
	struct afe_param_id_slot_mapping_cfg *slot_mapping =
		&dai_data->port_cfg.slot_mapping;
	struct afe_param_id_slot_mapping_cfg_v2 *slot_mapping_v2 =
		&dai_data->port_cfg.slot_mapping_v2;
	struct afe_param_id_custom_tdm_header_cfg *custom_tdm_header =
		&dai_data->port_cfg.custom_tdm_header;

	pr_debug("%s: dev_name: %s\n",
		__func__, dev_name(dai->dev));

	if ((params_channels(params) == 0) ||
		(params_channels(params) > 32)) {
		dev_err(dai->dev, "%s: invalid param channels %d\n",
			__func__, params_channels(params));
		return -EINVAL;
	}
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dai_data->bitwidth = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		dai_data->bitwidth = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		dai_data->bitwidth = 32;
		break;
	default:
		dev_err(dai->dev, "%s: invalid param format 0x%x\n",
			__func__, params_format(params));
		return -EINVAL;
	}
	dai_data->channels = params_channels(params);
	dai_data->rate = params_rate(params);

	/*
	 * update tdm group config param
	 * NOTE: group config is set to the same as slot config.
	 */
	tdm_group->bit_width = tdm_group->slot_width;

	/*
	 * for multi lane scenario
	 * Total number of active channels = number of active lanes * number of active slots.
	 */
	if (dai_data->lane_cfg.lane_mask != AFE_LANE_MASK_INVALID)
		tdm_group->num_channels = tdm_group->nslots_per_frame
			* num_of_bits_set(dai_data->lane_cfg.lane_mask);
	else
		tdm_group->num_channels = tdm_group->nslots_per_frame;

	tdm_group->sample_rate = dai_data->rate;

	pr_debug("%s: TDM GROUP:\n"
		"num_channels=%d sample_rate=%d bit_width=%d\n"
		"nslots_per_frame=%d slot_width=%d slot_mask=0x%x\n",
		__func__,
		tdm_group->num_channels,
		tdm_group->sample_rate,
		tdm_group->bit_width,
		tdm_group->nslots_per_frame,
		tdm_group->slot_width,
		tdm_group->slot_mask);
	pr_debug("%s: TDM GROUP:\n"
		"port_id[0]=0x%x port_id[1]=0x%x port_id[2]=0x%x port_id[3]=0x%x\n"
		"port_id[4]=0x%x port_id[5]=0x%x port_id[6]=0x%x port_id[7]=0x%x\n",
		__func__,
		tdm_group->port_id[0],
		tdm_group->port_id[1],
		tdm_group->port_id[2],
		tdm_group->port_id[3],
		tdm_group->port_id[4],
		tdm_group->port_id[5],
		tdm_group->port_id[6],
		tdm_group->port_id[7]);
	pr_debug("%s: TDM GROUP ID 0x%x lane mask 0x%x:\n",
		__func__,
		tdm_group->group_id,
		dai_data->lane_cfg.lane_mask);

	/*
	 * update tdm config param
	 * NOTE: channels/rate/bitwidth are per stream property
	 */
	tdm->num_channels = dai_data->channels;
	tdm->sample_rate = dai_data->rate;
	tdm->bit_width = dai_data->bitwidth;
	/*
	 * port slot config is the same as group slot config
	 * port slot mask should be set according to offset
	 */
	tdm->nslots_per_frame = tdm_group->nslots_per_frame;
	tdm->slot_width = tdm_group->slot_width;
	if (q6core_get_avcs_api_version_per_service(
		APRV2_IDS_SERVICE_ID_ADSP_AFE_V) >= AFE_API_VERSION_V3)
		tdm->slot_mask = tdm_param_set_slot_mask(
					slot_mapping_v2->offset,
					tdm_group->slot_width,
					tdm_group->nslots_per_frame,
					tdm_group->num_channels);
	else
		tdm->slot_mask = tdm_param_set_slot_mask(slot_mapping->offset,
					tdm_group->slot_width,
					tdm_group->nslots_per_frame,
					tdm_group->num_channels);

	pr_debug("%s: TDM:\n"
		"num_channels=%d sample_rate=%d bit_width=%d\n"
		"nslots_per_frame=%d slot_width=%d slot_mask=0x%x\n"
		"data_format=0x%x sync_mode=0x%x sync_src=0x%x\n"
		"data_out=0x%x invert_sync=0x%x data_delay=0x%x\n",
		__func__,
		tdm->num_channels,
		tdm->sample_rate,
		tdm->bit_width,
		tdm->nslots_per_frame,
		tdm->slot_width,
		tdm->slot_mask,
		tdm->data_format,
		tdm->sync_mode,
		tdm->sync_src,
		tdm->ctrl_data_out_enable,
		tdm->ctrl_invert_sync_pulse,
		tdm->ctrl_sync_data_delay);
	if (q6core_get_avcs_api_version_per_service(
		APRV2_IDS_SERVICE_ID_ADSP_AFE_V) >= AFE_API_VERSION_V3) {
		/*
		 * update slot mapping v2 config param
		 * NOTE: channels/rate/bitwidth are per stream property
		 */
		slot_mapping_v2->bitwidth = dai_data->bitwidth;

	pr_debug("%s: SLOT MAPPING_V2:\n"
		"num_channel=%d bitwidth=%d data_align=0x%x\n",
		__func__,
		slot_mapping_v2->num_channel,
		slot_mapping_v2->bitwidth,
		slot_mapping_v2->data_align_type);
	pr_debug("%s: SLOT MAPPING V2:\n"
		"offset[0]=0x%x offset[1]=0x%x offset[2]=0x%x offset[3]=0x%x\n"
		"offset[4]=0x%x offset[5]=0x%x offset[6]=0x%x offset[7]=0x%x\n"
		"offset[8]=0x%x offset[9]=0x%x offset[10]=0x%x offset[11]=0x%x\n"
		"offset[12]=0x%x offset[13]=0x%x offset[14]=0x%x offset[15]=0x%x\n"
		"offset[16]=0x%x offset[17]=0x%x offset[18]=0x%x offset[19]=0x%x\n"
		"offset[20]=0x%x offset[21]=0x%x offset[22]=0x%x offset[23]=0x%x\n"
		"offset[24]=0x%x offset[25]=0x%x offset[26]=0x%x offset[27]=0x%x\n"
		"offset[28]=0x%x offset[29]=0x%x offset[30]=0x%x offset[31]=0x%x\n",
		__func__,
		slot_mapping_v2->offset[0],
		slot_mapping_v2->offset[1],
		slot_mapping_v2->offset[2],
		slot_mapping_v2->offset[3],
		slot_mapping_v2->offset[4],
		slot_mapping_v2->offset[5],
		slot_mapping_v2->offset[6],
		slot_mapping_v2->offset[7],
		slot_mapping_v2->offset[8],
		slot_mapping_v2->offset[9],
		slot_mapping_v2->offset[10],
		slot_mapping_v2->offset[11],
		slot_mapping_v2->offset[12],
		slot_mapping_v2->offset[13],
		slot_mapping_v2->offset[14],
		slot_mapping_v2->offset[15],
		slot_mapping_v2->offset[16],
		slot_mapping_v2->offset[17],
		slot_mapping_v2->offset[18],
		slot_mapping_v2->offset[19],
		slot_mapping_v2->offset[20],
		slot_mapping_v2->offset[21],
		slot_mapping_v2->offset[22],
		slot_mapping_v2->offset[23],
		slot_mapping_v2->offset[24],
		slot_mapping_v2->offset[25],
		slot_mapping_v2->offset[26],
		slot_mapping_v2->offset[27],
		slot_mapping_v2->offset[28],
		slot_mapping_v2->offset[29],
		slot_mapping_v2->offset[30],
		slot_mapping_v2->offset[31]);
	} else {
		/*
		 * update slot mapping config param
		 * NOTE: channels/rate/bitwidth are per stream property
		 */
		slot_mapping->bitwidth = dai_data->bitwidth;

		pr_debug("%s: SLOT MAPPING:\n"
			"num_channel=%d bitwidth=%d data_align=0x%x\n",
			__func__,
			slot_mapping->num_channel,
			slot_mapping->bitwidth,
			slot_mapping->data_align_type);
		pr_debug("%s: SLOT MAPPING:\n"
			"offset[0]=0x%x offset[1]=0x%x offset[2]=0x%x offset[3]=0x%x\n"
			"offset[4]=0x%x offset[5]=0x%x offset[6]=0x%x offset[7]=0x%x\n",
			__func__,
			slot_mapping->offset[0],
			slot_mapping->offset[1],
			slot_mapping->offset[2],
			slot_mapping->offset[3],
			slot_mapping->offset[4],
			slot_mapping->offset[5],
			slot_mapping->offset[6],
			slot_mapping->offset[7]);
	}
	/*
	 * update custom header config param
	 * NOTE: channels/rate/bitwidth are per playback stream property.
	 * custom tdm header only applicable to playback stream.
	 */
	if (custom_tdm_header->header_type !=
		AFE_CUSTOM_TDM_HEADER_TYPE_INVALID) {
		pr_debug("%s: CUSTOM TDM HEADER:\n"
			"start_offset=0x%x header_width=%d\n"
			"num_frame_repeat=%d header_type=0x%x\n",
			__func__,
			custom_tdm_header->start_offset,
			custom_tdm_header->header_width,
			custom_tdm_header->num_frame_repeat,
			custom_tdm_header->header_type);
		pr_debug("%s: CUSTOM TDM HEADER:\n"
			"header[0]=0x%x header[1]=0x%x header[2]=0x%x header[3]=0x%x\n"
			"header[4]=0x%x header[5]=0x%x header[6]=0x%x header[7]=0x%x\n",
			__func__,
			custom_tdm_header->header[0],
			custom_tdm_header->header[1],
			custom_tdm_header->header[2],
			custom_tdm_header->header[3],
			custom_tdm_header->header[4],
			custom_tdm_header->header[5],
			custom_tdm_header->header[6],
			custom_tdm_header->header[7]);
	}

	return 0;
}

static int msm_dai_q6_tdm_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	int rc = 0;
	struct msm_dai_q6_tdm_dai_data *dai_data =
		dev_get_drvdata(dai->dev);
	u16 group_id = dai_data->group_cfg.tdm_cfg.group_id;
	int group_idx = 0;
	atomic_t *group_ref = NULL;
	int intf_idx =  PORT_ID_TO_INTF_IDX(dai->id);

	dev_dbg(dai->dev, "%s: dev_name: %s dev_id: 0x%x group_id: 0x%x\n",
		 __func__, dev_name(dai->dev), dai->dev->id, group_id);

	if (dai_data->port_cfg.custom_tdm_header.minor_version == 0)
		dev_dbg(dai->dev,
			 "%s: Custom tdm header not supported\n", __func__);

	group_idx = msm_dai_q6_get_group_idx(dai->id);
	if (group_idx < 0) {
		dev_err(dai->dev, "%s port id 0x%x not supported\n",
			__func__, dai->id);
		return -EINVAL;
	}

	mutex_lock(&tdm_mutex);

	group_ref = &tdm_group_ref[group_idx];

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		if (msm_dai_q6_get_tdm_clk_ref(group_idx) == 0) {
			/* TX and RX share the same clk. So enable the clk
			 * per TDM interface. */
			rc = msm_dai_q6_tdm_set_clk(dai_data,
				dai->id, true);
			if (rc < 0) {
				dev_err(dai->dev, "%s: fail to enable AFE clk 0x%x\n",
					__func__, dai->id);
				goto rtn;
			}
		}
		/* Paired Rx Port Start */
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			rc = afe_paired_rx_tdm_port_ops(intf_idx, true, tdm_group_ref);
			if (rc < 0) {
				pr_debug("%s:Paired Rx Port failed to start\n",__func__);
				if (msm_dai_q6_get_tdm_clk_ref(group_idx) == 0) {
					msm_dai_q6_tdm_set_clk(dai_data,
						dai->id, false);
				}
				goto rtn;
			}
		}
		/* PORT START should be set if prepare called
		 * in active state.
		 */
		if (atomic_read(group_ref) == 0) {
			/*
			 * if only one port, don't do group enable as there
			 * is no group need for only one port
			 */
			if (dai_data->num_group_ports > 1) {
				rc = afe_port_group_enable(group_id,
					&dai_data->group_cfg, true,
					&dai_data->lane_cfg);
				if (rc < 0) {
					dev_err(dai->dev,
					"%s: fail to enable AFE group 0x%x\n",
					__func__, group_id);
					goto rtn;
				}
			}
		}

		rc = afe_tdm_port_start(dai->id, &dai_data->port_cfg,
			dai_data->rate, dai_data->num_group_ports);
		if (rc < 0) {
			rc = afe_paired_rx_tdm_port_ops(intf_idx, false, tdm_group_ref);
			if (rc < 0)
				pr_err("%s:Unable to close Paired Rx due to tx usecase start failure", __func__);
			if (atomic_read(group_ref) == 0) {
				afe_port_group_enable(group_id,
					NULL, false, NULL);
			}
			if (msm_dai_q6_get_tdm_clk_ref(group_idx) == 0) {
				msm_dai_q6_tdm_set_clk(dai_data,
					dai->id, false);
			}
			dev_err(dai->dev, "%s: fail to open AFE port 0x%x\n",
				__func__, dai->id);
		} else {
			set_bit(STATUS_PORT_STARTED,
				dai_data->status_mask);
			atomic_inc(group_ref);
			pr_debug("%s: Group ref count: group_ref : %d", __func__, group_ref->counter);
		}

		/* TODO: need to monitor PCM/MI2S/TDM HW status */
		/* NOTE: AFE should error out if HW resource contention */

	}

rtn:
	mutex_unlock(&tdm_mutex);
	return rc;
}

static void msm_dai_q6_tdm_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	int rc = 0;
	struct msm_dai_q6_tdm_dai_data *dai_data =
		dev_get_drvdata(dai->dev);
	u16 group_id = dai_data->group_cfg.tdm_cfg.group_id;
	int group_idx = 0;
	atomic_t *group_ref = NULL;
	int intf_idx =  PORT_ID_TO_INTF_IDX(dai->id);

	group_idx = msm_dai_q6_get_group_idx(dai->id);
	if (group_idx < 0) {
		dev_err(dai->dev, "%s port id 0x%x not supported\n",
			__func__, dai->id);
		return;
	}

	mutex_lock(&tdm_mutex);

	group_ref = &tdm_group_ref[group_idx];

	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		rc = afe_close(dai->id);
		if (rc < 0) {
			dev_err(dai->dev, "%s: fail to close AFE port 0x%x\n",
				__func__, dai->id);
		}
		atomic_dec(group_ref);
		pr_debug("%s: group_ref : %d \n",__func__, group_ref->counter);
		clear_bit(STATUS_PORT_STARTED,
			dai_data->status_mask);

		if (atomic_read(group_ref) == 0) {
			rc = afe_port_group_enable(group_id,
				NULL, false, NULL);
			if (rc < 0) {
				dev_err(dai->dev, "%s: fail to disable AFE group 0x%x\n",
					__func__, group_id);
			}
		}
		/* Paired Rx Stop */
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			rc = afe_paired_rx_tdm_port_ops(intf_idx, false, tdm_group_ref);
			if (rc < 0){
				if (msm_dai_q6_get_tdm_clk_ref(group_idx) == 0) {
					msm_dai_q6_tdm_set_clk(dai_data,
						dai->id, false);
				}
				pr_err("%s:AFE Paired Rx Port Not disabled");
			}
		}
		if (msm_dai_q6_get_tdm_clk_ref(group_idx) == 0) {
			rc = msm_dai_q6_tdm_set_clk(dai_data,
				dai->id, false);
			if (rc < 0) {
				dev_err(dai->dev, "%s: fail to disable AFE clk 0x%x\n",
					__func__, dai->id);
			}
		}

		/* TODO: need to monitor PCM/MI2S/TDM HW status */
		/* NOTE: AFE should error out if HW resource contention */

	}

	mutex_unlock(&tdm_mutex);
}

static struct snd_soc_dai_ops msm_dai_q6_tdm_ops = {
	.prepare          = msm_dai_q6_tdm_prepare,
	.hw_params        = msm_dai_q6_tdm_hw_params,
	.set_tdm_slot     = msm_dai_q6_tdm_set_tdm_slot,
	.set_channel_map  = msm_dai_q6_tdm_set_channel_map,
	.set_sysclk       = msm_dai_q6_tdm_set_sysclk,
	.shutdown         = msm_dai_q6_tdm_shutdown,
};

static struct snd_soc_dai_driver msm_dai_q6_tdm_dai[] = {
	{
		.playback = {
			.stream_name = "Primary TDM0 Playback",
			.aif_name = "PRI_TDM_RX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_RX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_RX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Primary TDM1 Playback",
			.aif_name = "PRI_TDM_RX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_RX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_RX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Primary TDM2 Playback",
			.aif_name = "PRI_TDM_RX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_RX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_RX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Primary TDM3 Playback",
			.aif_name = "PRI_TDM_RX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_RX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_RX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Primary TDM4 Playback",
			.aif_name = "PRI_TDM_RX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_RX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_RX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Primary TDM5 Playback",
			.aif_name = "PRI_TDM_RX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_RX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_RX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Primary TDM6 Playback",
			.aif_name = "PRI_TDM_RX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_RX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_RX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Primary TDM7 Playback",
			.aif_name = "PRI_TDM_RX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_RX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_RX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Primary TDM0 Capture",
			.aif_name = "PRI_TDM_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_TX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_TX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Primary TDM1 Capture",
			.aif_name = "PRI_TDM_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_TX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_TX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Primary TDM2 Capture",
			.aif_name = "PRI_TDM_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_TX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_TX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Primary TDM3 Capture",
			.aif_name = "PRI_TDM_TX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_TX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_TX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Primary TDM4 Capture",
			.aif_name = "PRI_TDM_TX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_TX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_TX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Primary TDM5 Capture",
			.aif_name = "PRI_TDM_TX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_TX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_TX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Primary TDM6 Capture",
			.aif_name = "PRI_TDM_TX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_TX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_TX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Primary TDM7 Capture",
			.aif_name = "PRI_TDM_TX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "PRI_TDM_TX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_PRIMARY_TDM_TX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM0 Playback",
			.aif_name = "SEC_TDM_RX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_RX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_RX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM1 Playback",
			.aif_name = "SEC_TDM_RX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_RX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_RX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM2 Playback",
			.aif_name = "SEC_TDM_RX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_RX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_RX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM3 Playback",
			.aif_name = "SEC_TDM_RX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_RX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_RX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM4 Playback",
			.aif_name = "SEC_TDM_RX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_RX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_RX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM5 Playback",
			.aif_name = "SEC_TDM_RX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_RX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_RX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM6 Playback",
			.aif_name = "SEC_TDM_RX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_RX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_RX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Secondary TDM7 Playback",
			.aif_name = "SEC_TDM_RX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_RX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_RX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM0 Capture",
			.aif_name = "SEC_TDM_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_TX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_TX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM1 Capture",
			.aif_name = "SEC_TDM_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_TX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_TX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM2 Capture",
			.aif_name = "SEC_TDM_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_TX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_TX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM3 Capture",
			.aif_name = "SEC_TDM_TX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_TX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_TX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM4 Capture",
			.aif_name = "SEC_TDM_TX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_TX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_TX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM5 Capture",
			.aif_name = "SEC_TDM_TX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_TX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_TX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM6 Capture",
			.aif_name = "SEC_TDM_TX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_TX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_TX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Secondary TDM7 Capture",
			.aif_name = "SEC_TDM_TX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEC_TDM_TX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SECONDARY_TDM_TX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Tertiary TDM0 Playback",
			.aif_name = "TERT_TDM_RX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_RX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_RX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Tertiary TDM1 Playback",
			.aif_name = "TERT_TDM_RX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_RX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_RX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Tertiary TDM2 Playback",
			.aif_name = "TERT_TDM_RX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_RX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_RX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Tertiary TDM3 Playback",
			.aif_name = "TERT_TDM_RX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_RX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_RX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Tertiary TDM4 Playback",
			.aif_name = "TERT_TDM_RX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_RX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_RX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Tertiary TDM5 Playback",
			.aif_name = "TERT_TDM_RX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_RX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_RX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Tertiary TDM6 Playback",
			.aif_name = "TERT_TDM_RX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_RX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_RX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Tertiary TDM7 Playback",
			.aif_name = "TERT_TDM_RX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_RX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_RX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Tertiary TDM0 Capture",
			.aif_name = "TERT_TDM_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_TX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_TX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Tertiary TDM1 Capture",
			.aif_name = "TERT_TDM_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_TX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_TX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Tertiary TDM2 Capture",
			.aif_name = "TERT_TDM_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_TX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_TX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Tertiary TDM3 Capture",
			.aif_name = "TERT_TDM_TX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_TX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_TX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Tertiary TDM4 Capture",
			.aif_name = "TERT_TDM_TX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_TX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_TX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Tertiary TDM5 Capture",
			.aif_name = "TERT_TDM_TX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_TX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_TX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Tertiary TDM6 Capture",
			.aif_name = "TERT_TDM_TX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_TX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_TX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Tertiary TDM7 Capture",
			.aif_name = "TERT_TDM_TX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "TERT_TDM_TX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_TERTIARY_TDM_TX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quaternary TDM0 Playback",
			.aif_name = "QUAT_TDM_RX_0",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_RX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_RX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quaternary TDM1 Playback",
			.aif_name = "QUAT_TDM_RX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_RX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_RX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quaternary TDM2 Playback",
			.aif_name = "QUAT_TDM_RX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_RX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_RX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quaternary TDM3 Playback",
			.aif_name = "QUAT_TDM_RX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_RX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_RX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quaternary TDM4 Playback",
			.aif_name = "QUAT_TDM_RX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_RX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_RX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quaternary TDM5 Playback",
			.aif_name = "QUAT_TDM_RX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_RX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_RX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quaternary TDM6 Playback",
			.aif_name = "QUAT_TDM_RX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_RX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_RX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quaternary TDM7 Playback",
			.aif_name = "QUAT_TDM_RX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_RX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_RX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quaternary TDM0 Capture",
			.aif_name = "QUAT_TDM_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_TX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_TX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quaternary TDM1 Capture",
			.aif_name = "QUAT_TDM_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_TX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_TX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quaternary TDM2 Capture",
			.aif_name = "QUAT_TDM_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_TX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_TX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quaternary TDM3 Capture",
			.aif_name = "QUAT_TDM_TX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_TX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_TX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quaternary TDM4 Capture",
			.aif_name = "QUAT_TDM_TX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_TX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_TX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quaternary TDM5 Capture",
			.aif_name = "QUAT_TDM_TX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_TX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_TX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quaternary TDM6 Capture",
			.aif_name = "QUAT_TDM_TX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_TX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_TX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quaternary TDM7 Capture",
			.aif_name = "QUAT_TDM_TX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUAT_TDM_TX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUATERNARY_TDM_TX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quinary TDM0 Playback",
			.aif_name = "QUIN_TDM_RX_0",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_RX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_RX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quinary TDM1 Playback",
			.aif_name = "QUIN_TDM_RX_1",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_RX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_RX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quinary TDM2 Playback",
			.aif_name = "QUIN_TDM_RX_2",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_RX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_RX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quinary TDM3 Playback",
			.aif_name = "QUIN_TDM_RX_3",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_RX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_RX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quinary TDM4 Playback",
			.aif_name = "QUIN_TDM_RX_4",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_RX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_RX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quinary TDM5 Playback",
			.aif_name = "QUIN_TDM_RX_5",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_RX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_RX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quinary TDM6 Playback",
			.aif_name = "QUIN_TDM_RX_6",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_RX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_RX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Quinary TDM7 Playback",
			.aif_name = "QUIN_TDM_RX_7",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_RX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_RX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quinary TDM0 Capture",
			.aif_name = "QUIN_TDM_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_TX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_TX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quinary TDM1 Capture",
			.aif_name = "QUIN_TDM_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_TX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_TX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quinary TDM2 Capture",
			.aif_name = "QUIN_TDM_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_TX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_TX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quinary TDM3 Capture",
			.aif_name = "QUIN_TDM_TX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_TX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_TX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quinary TDM4 Capture",
			.aif_name = "QUIN_TDM_TX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_TX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_TX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quinary TDM5 Capture",
			.aif_name = "QUIN_TDM_TX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_TX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_TX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quinary TDM6 Capture",
			.aif_name = "QUIN_TDM_TX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_TX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_TX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Quinary TDM7 Capture",
			.aif_name = "QUIN_TDM_TX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "QUIN_TDM_TX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_QUINARY_TDM_TX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Senary TDM0 Playback",
			.aif_name = "SEN_TDM_RX_0",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_RX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_RX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Senary TDM1 Playback",
			.aif_name = "SEN_TDM_RX_1",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_RX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_RX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Senary TDM2 Playback",
			.aif_name = "SEN_TDM_RX_2",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_RX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_RX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Senary TDM3 Playback",
			.aif_name = "SEN_TDM_RX_3",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_RX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_RX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Senary TDM4 Playback",
			.aif_name = "SEN_TDM_RX_4",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_RX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_RX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Senary TDM5 Playback",
			.aif_name = "SEN_TDM_RX_5",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_RX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_RX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Senary TDM6 Playback",
			.aif_name = "SEN_TDM_RX_6",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_RX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_RX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Senary TDM7 Playback",
			.aif_name = "SEN_TDM_RX_7",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_RX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_RX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Senary TDM0 Capture",
			.aif_name = "SEN_TDM_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_TX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_TX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Senary TDM1 Capture",
			.aif_name = "SEN_TDM_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_TX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_TX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Senary TDM2 Capture",
			.aif_name = "SEN_TDM_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_TX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_TX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Senary TDM3 Capture",
			.aif_name = "SEN_TDM_TX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_TX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_TX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Senary TDM4 Capture",
			.aif_name = "SEN_TDM_TX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_TX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_TX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Senary TDM5 Capture",
			.aif_name = "SEN_TDM_TX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_TX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_TX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Senary TDM6 Capture",
			.aif_name = "SEN_TDM_TX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_TX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_TX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Senary TDM7 Capture",
			.aif_name = "SEN_TDM_TX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEN_TDM_TX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SENARY_TDM_TX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Septenary TDM0 Playback",
			.aif_name = "SEP_TDM_RX_0",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_RX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_RX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Septenary TDM1 Playback",
			.aif_name = "SEP_TDM_RX_1",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_RX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_RX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Septenary TDM2 Playback",
			.aif_name = "SEP_TDM_RX_2",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_RX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_RX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Septenary TDM3 Playback",
			.aif_name = "SEP_TDM_RX_3",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_RX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_RX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Septenary TDM4 Playback",
			.aif_name = "SEP_TDM_RX_4",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_RX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_RX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Septenary TDM5 Playback",
			.aif_name = "SEP_TDM_RX_5",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_RX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_RX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Septenary TDM6 Playback",
			.aif_name = "SEP_TDM_RX_6",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_RX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_RX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Septenary TDM7 Playback",
			.aif_name = "SEP_TDM_RX_7",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_RX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_RX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Septenary TDM0 Capture",
			.aif_name = "SEP_TDM_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_TX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_TX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Septenary TDM1 Capture",
			.aif_name = "SEP_TDM_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_TX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_TX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Septenary TDM2 Capture",
			.aif_name = "SEP_TDM_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_TX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_TX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Septenary TDM3 Capture",
			.aif_name = "SEP_TDM_TX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_TX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_TX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Septenary TDM4 Capture",
			.aif_name = "SEP_TDM_TX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_TX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_TX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Septenary TDM5 Capture",
			.aif_name = "SEP_TDM_TX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_TX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_TX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Septenary TDM6 Capture",
			.aif_name = "SEP_TDM_TX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_TX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_TX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Septenary TDM7 Capture",
			.aif_name = "SEP_TDM_TX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "SEP_TDM_TX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_SEPTENARY_TDM_TX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif0 TDM0 Playback",
			.aif_name = "HSIF0_TDM_RX_0",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_RX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_RX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif0 TDM1 Playback",
			.aif_name = "HSIF0_TDM_RX_1",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_RX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_RX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif0 TDM2 Playback",
			.aif_name = "HSIF0_TDM_RX_2",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_RX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_RX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif0 TDM3 Playback",
			.aif_name = "HSIF0_TDM_RX_3",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_RX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_RX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif0 TDM4 Playback",
			.aif_name = "HSIF0_TDM_RX_4",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_RX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_RX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif0 TDM5 Playback",
			.aif_name = "HSIF0_TDM_RX_5",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_RX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_RX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif0 TDM6 Playback",
			.aif_name = "HSIF0_TDM_RX_6",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_RX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_RX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif0 TDM7 Playback",
			.aif_name = "HSIF0_TDM_RX_7",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_RX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_RX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif0 TDM0 Capture",
			.aif_name = "HSIF0_TDM_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_TX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_TX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif0 TDM1 Capture",
			.aif_name = "HSIF0_TDM_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_TX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_TX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif0 TDM2 Capture",
			.aif_name = "HSIF0_TDM_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_TX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_TX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif0 TDM3 Capture",
			.aif_name = "HSIF0_TDM_TX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_TX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_TX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif0 TDM4 Capture",
			.aif_name = "HSIF0_TDM_TX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_TX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_TX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif0 TDM5 Capture",
			.aif_name = "HSIF0_TDM_TX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_TX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_TX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif0 TDM6 Capture",
			.aif_name = "HSIF0_TDM_TX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_TX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_TX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif0 TDM7 Capture",
			.aif_name = "HSIF0_TDM_TX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF0_TDM_TX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF0_TDM_TX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif1 TDM0 Playback",
			.aif_name = "HSIF1_TDM_RX_0",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_RX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_RX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif1 TDM1 Playback",
			.aif_name = "HSIF1_TDM_RX_1",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_RX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_RX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif1 TDM2 Playback",
			.aif_name = "HSIF1_TDM_RX_2",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_RX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_RX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif1 TDM3 Playback",
			.aif_name = "HSIF1_TDM_RX_3",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_RX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_RX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif1 TDM4 Playback",
			.aif_name = "HSIF1_TDM_RX_4",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_RX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_RX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif1 TDM5 Playback",
			.aif_name = "HSIF1_TDM_RX_5",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_RX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_RX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif1 TDM6 Playback",
			.aif_name = "HSIF1_TDM_RX_6",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_RX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_RX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif1 TDM7 Playback",
			.aif_name = "HSIF1_TDM_RX_7",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_RX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_RX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif1 TDM0 Capture",
			.aif_name = "HSIF1_TDM_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_TX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_TX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif1 TDM1 Capture",
			.aif_name = "HSIF1_TDM_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_TX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_TX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif1 TDM2 Capture",
			.aif_name = "HSIF1_TDM_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_TX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_TX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif1 TDM3 Capture",
			.aif_name = "HSIF1_TDM_TX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_TX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_TX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif1 TDM4 Capture",
			.aif_name = "HSIF1_TDM_TX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_TX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_TX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif1 TDM5 Capture",
			.aif_name = "HSIF1_TDM_TX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_TX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_TX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif1 TDM6 Capture",
			.aif_name = "HSIF1_TDM_TX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_TX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_TX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif1 TDM7 Capture",
			.aif_name = "HSIF1_TDM_TX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF1_TDM_TX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF1_TDM_TX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif2 TDM0 Playback",
			.aif_name = "HSIF2_TDM_RX_0",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_RX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_RX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif2 TDM1 Playback",
			.aif_name = "HSIF2_TDM_RX_1",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_RX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_RX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif2 TDM2 Playback",
			.aif_name = "HSIF2_TDM_RX_2",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_RX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_RX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif2 TDM3 Playback",
			.aif_name = "HSIF2_TDM_RX_3",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_RX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_RX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif2 TDM4 Playback",
			.aif_name = "HSIF2_TDM_RX_4",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_RX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_RX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif2 TDM5 Playback",
			.aif_name = "HSIF2_TDM_RX_5",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_RX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_RX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif2 TDM6 Playback",
			.aif_name = "HSIF2_TDM_RX_6",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_RX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_RX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.playback = {
			.stream_name = "Hsif2 TDM7 Playback",
			.aif_name = "HSIF2_TDM_RX_7",
			.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_RX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_RX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif2 TDM0 Capture",
			.aif_name = "HSIF2_TDM_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_TX_0",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_TX,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif2 TDM1 Capture",
			.aif_name = "HSIF2_TDM_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_TX_1",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_TX_1,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif2 TDM2 Capture",
			.aif_name = "HSIF2_TDM_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_TX_2",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_TX_2,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif2 TDM3 Capture",
			.aif_name = "HSIF2_TDM_TX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_TX_3",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_TX_3,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif2 TDM4 Capture",
			.aif_name = "HSIF2_TDM_TX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_TX_4",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_TX_4,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif2 TDM5 Capture",
			.aif_name = "HSIF2_TDM_TX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_TX_5",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_TX_5,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif2 TDM6 Capture",
			.aif_name = "HSIF2_TDM_TX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_TX_6",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_TX_6,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
	{
		.capture = {
			.stream_name = "Hsif2 TDM7 Capture",
			.aif_name = "HSIF2_TDM_TX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 16,
			.rate_min = 8000,
			.rate_max = 352800,
		},
		.name = "HSIF2_TDM_TX_7",
		.ops = &msm_dai_q6_tdm_ops,
		.id = AFE_PORT_ID_HSIF2_TDM_TX_7,
		.probe = msm_dai_q6_dai_tdm_probe,
		.remove = msm_dai_q6_dai_tdm_remove,
	},
};

static const struct snd_soc_component_driver msm_q6_tdm_dai_component = {
	.name		= "msm-dai-q6-tdm",
};

static int msm_dai_q6_tdm_dev_probe(struct platform_device *pdev)
{
	struct msm_dai_q6_tdm_dai_data *dai_data = NULL;
	struct afe_param_id_custom_tdm_header_cfg *custom_tdm_header = NULL;
	int rc = 0;
	u32 tdm_dev_id = 0;
	int port_idx = 0;
	struct device_node *tdm_parent_node = NULL;

	/* retrieve device/afe id */
	rc = of_property_read_u32(pdev->dev.of_node,
		"qcom,msm-cpudai-tdm-dev-id",
		&tdm_dev_id);
	if (rc) {
		dev_err(&pdev->dev, "%s: Device ID missing in DT file\n",
			__func__);
		goto rtn;
	}
	if ((tdm_dev_id < AFE_PORT_ID_TDM_PORT_RANGE_START) ||
		(tdm_dev_id > AFE_PORT_ID_TDM_PORT_RANGE_END)) {
		dev_err(&pdev->dev, "%s: Invalid TDM Device ID 0x%x in DT file\n",
			__func__, tdm_dev_id);
		rc = -ENXIO;
		goto rtn;
	}
	pdev->id = tdm_dev_id;

	dai_data = kzalloc(sizeof(struct msm_dai_q6_tdm_dai_data),
				GFP_KERNEL);
	if (!dai_data) {
		rc = -ENOMEM;
		dev_err(&pdev->dev,
			"%s Failed to allocate memory for tdm dai_data\n",
			__func__);
		goto rtn;
	}
	memset(dai_data, 0, sizeof(*dai_data));

	rc = of_property_read_u32(pdev->dev.of_node,
				    "qcom,msm-dai-is-island-supported",
				    &dai_data->is_island_dai);
	if (rc)
		dev_dbg(&pdev->dev, "island supported entry not found\n");

	/* TDM CFG */
	tdm_parent_node = of_get_parent(pdev->dev.of_node);
	rc = of_property_read_u32(tdm_parent_node,
		"qcom,msm-cpudai-tdm-sync-mode",
		(u32 *)&dai_data->port_cfg.tdm.sync_mode);
	if (rc) {
		dev_err(&pdev->dev, "%s: Sync Mode from DT file %s\n",
			__func__, "qcom,msm-cpudai-tdm-sync-mode");
		goto free_dai_data;
	}
	dev_dbg(&pdev->dev, "%s: Sync Mode from DT file 0x%x\n",
		__func__, dai_data->port_cfg.tdm.sync_mode);

	rc = of_property_read_u32(tdm_parent_node,
		"qcom,msm-cpudai-tdm-sync-src",
		(u32 *)&dai_data->port_cfg.tdm.sync_src);
	if (rc) {
		dev_err(&pdev->dev, "%s: Sync Src from DT file %s\n",
			__func__, "qcom,msm-cpudai-tdm-sync-src");
		goto free_dai_data;
	}
	dev_dbg(&pdev->dev, "%s: Sync Src from DT file 0x%x\n",
		__func__, dai_data->port_cfg.tdm.sync_src);

	rc = of_property_read_u32(tdm_parent_node,
		"qcom,msm-cpudai-tdm-data-out",
		(u32 *)&dai_data->port_cfg.tdm.ctrl_data_out_enable);
	if (rc) {
		dev_err(&pdev->dev, "%s: Data Out from DT file %s\n",
			__func__, "qcom,msm-cpudai-tdm-data-out");
		goto free_dai_data;
	}
	dev_dbg(&pdev->dev, "%s: Data Out from DT file 0x%x\n",
		__func__, dai_data->port_cfg.tdm.ctrl_data_out_enable);

	rc = of_property_read_u32(tdm_parent_node,
		"qcom,msm-cpudai-tdm-invert-sync",
		(u32 *)&dai_data->port_cfg.tdm.ctrl_invert_sync_pulse);
	if (rc) {
		dev_err(&pdev->dev, "%s: Invert Sync from DT file %s\n",
			__func__, "qcom,msm-cpudai-tdm-invert-sync");
		goto free_dai_data;
	}
	dev_dbg(&pdev->dev, "%s: Invert Sync from DT file 0x%x\n",
		__func__, dai_data->port_cfg.tdm.ctrl_invert_sync_pulse);

	rc = of_property_read_u32(tdm_parent_node,
		"qcom,msm-cpudai-tdm-data-delay",
		(u32 *)&dai_data->port_cfg.tdm.ctrl_sync_data_delay);
	if (rc) {
		dev_err(&pdev->dev, "%s: Data Delay from DT file %s\n",
			__func__, "qcom,msm-cpudai-tdm-data-delay");
		goto free_dai_data;
	}
	dev_dbg(&pdev->dev, "%s: Data Delay from DT file 0x%x\n",
		__func__, dai_data->port_cfg.tdm.ctrl_sync_data_delay);

	/* TDM CFG -- set default */
	dai_data->port_cfg.tdm.data_format = AFE_LINEAR_PCM_DATA;
	dai_data->port_cfg.tdm.tdm_cfg_minor_version =
		AFE_API_VERSION_TDM_CONFIG;

	/* TDM SLOT MAPPING CFG */
	rc = of_property_read_u32(pdev->dev.of_node,
		"qcom,msm-cpudai-tdm-data-align",
		&dai_data->port_cfg.slot_mapping.data_align_type);
	if (rc) {
		dev_err(&pdev->dev, "%s: Data Align from DT file %s\n",
			__func__,
			"qcom,msm-cpudai-tdm-data-align");
		goto free_dai_data;
	}
	dev_dbg(&pdev->dev, "%s: Data Align from DT file 0x%x\n",
		__func__, dai_data->port_cfg.slot_mapping.data_align_type);

	/* TDM SLOT MAPPING CFG -- set default */
	dai_data->port_cfg.slot_mapping.minor_version =
		AFE_API_VERSION_SLOT_MAPPING_CONFIG;
	dai_data->port_cfg.slot_mapping_v2.minor_version =
		AFE_API_VERSION_SLOT_MAPPING_CONFIG_V2;

	/* CUSTOM TDM HEADER CFG */
	custom_tdm_header = &dai_data->port_cfg.custom_tdm_header;
	if (of_find_property(pdev->dev.of_node,
			"qcom,msm-cpudai-tdm-header-start-offset", NULL) &&
		of_find_property(pdev->dev.of_node,
			"qcom,msm-cpudai-tdm-header-width", NULL) &&
		of_find_property(pdev->dev.of_node,
			"qcom,msm-cpudai-tdm-header-num-frame-repeat", NULL)) {
		/* if the property exist */
		rc = of_property_read_u32(pdev->dev.of_node,
			"qcom,msm-cpudai-tdm-header-start-offset",
			(u32 *)&custom_tdm_header->start_offset);
		if (rc) {
			dev_err(&pdev->dev, "%s: Header Start Offset from DT file %s\n",
				__func__,
				"qcom,msm-cpudai-tdm-header-start-offset");
			goto free_dai_data;
		}
		dev_dbg(&pdev->dev, "%s: Header Start Offset from DT file 0x%x\n",
			__func__, custom_tdm_header->start_offset);

		rc = of_property_read_u32(pdev->dev.of_node,
			"qcom,msm-cpudai-tdm-header-width",
			(u32 *)&custom_tdm_header->header_width);
		if (rc) {
			dev_err(&pdev->dev, "%s: Header Width from DT file %s\n",
				__func__, "qcom,msm-cpudai-tdm-header-width");
			goto free_dai_data;
		}
		dev_dbg(&pdev->dev, "%s: Header Width from DT file 0x%x\n",
			__func__, custom_tdm_header->header_width);

		rc = of_property_read_u32(pdev->dev.of_node,
			"qcom,msm-cpudai-tdm-header-num-frame-repeat",
			(u32 *)&custom_tdm_header->num_frame_repeat);
		if (rc) {
			dev_err(&pdev->dev, "%s: Header Num Frame Repeat from DT file %s\n",
				__func__,
				"qcom,msm-cpudai-tdm-header-num-frame-repeat");
			goto free_dai_data;
		}
		dev_dbg(&pdev->dev, "%s: Header Num Frame Repeat from DT file 0x%x\n",
			__func__, custom_tdm_header->num_frame_repeat);

		/* CUSTOM TDM HEADER CFG -- set default */
		custom_tdm_header->minor_version =
			AFE_API_VERSION_CUSTOM_TDM_HEADER_CONFIG;
		custom_tdm_header->header_type =
			AFE_CUSTOM_TDM_HEADER_TYPE_INVALID;
	} else {
		/* CUSTOM TDM HEADER CFG -- set default */
		custom_tdm_header->header_type =
			AFE_CUSTOM_TDM_HEADER_TYPE_INVALID;
		/* proceed with probe */
	}

	/* copy static clk per parent node */
	dai_data->clk_set = tdm_clk_set;
	/* copy static group cfg per parent node */
	dai_data->group_cfg.tdm_cfg = tdm_group_cfg;
	/* copy static num group ports per parent node */
	dai_data->num_group_ports = num_tdm_group_ports;
	dai_data->lane_cfg = tdm_lane_cfg;

	dev_set_drvdata(&pdev->dev, dai_data);

	port_idx = msm_dai_q6_get_port_idx(tdm_dev_id);
	if (port_idx < 0) {
		dev_err(&pdev->dev, "%s Port id 0x%x not supported\n",
			__func__, tdm_dev_id);
		rc = -EINVAL;
		goto free_dai_data;
	}

	rc = snd_soc_register_component(&pdev->dev,
		&msm_q6_tdm_dai_component,
		&msm_dai_q6_tdm_dai[port_idx], 1);

	if (rc) {
		dev_err(&pdev->dev, "%s: TDM dai 0x%x register failed, rc=%d\n",
			__func__, tdm_dev_id, rc);
		goto err_register;
	}

	return 0;

err_register:
free_dai_data:
	kfree(dai_data);
rtn:
	return rc;
}

static int msm_dai_q6_tdm_dev_remove(struct platform_device *pdev)
{
	struct msm_dai_q6_tdm_dai_data *dai_data =
		dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_component(&pdev->dev);

	kfree(dai_data);

	return 0;
}

static const struct of_device_id msm_dai_q6_tdm_dev_dt_match[] = {
	{ .compatible = "qcom,msm-dai-q6-tdm", },
	{}
};

MODULE_DEVICE_TABLE(of, msm_dai_q6_tdm_dev_dt_match);

static struct platform_driver msm_dai_q6_tdm_driver = {
	.probe  = msm_dai_q6_tdm_dev_probe,
	.remove  = msm_dai_q6_tdm_dev_remove,
	.driver = {
		.name = "msm-dai-q6-tdm",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_q6_tdm_dev_dt_match,
		.suppress_bind_attrs = true,
	},
};

static int msm_dai_q6_cdc_dma_format_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_cdc_dma_dai_data *dai_data = kcontrol->private_data;
	int value = ucontrol->value.integer.value[0];

	dai_data->port_config.cdc_dma.data_format = value;
	pr_debug("%s: format = %d\n", __func__, value);
	return 0;
}

static int msm_dai_q6_cdc_dma_format_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_cdc_dma_dai_data *dai_data = kcontrol->private_data;

	ucontrol->value.integer.value[0] =
		dai_data->port_config.cdc_dma.data_format;
	return 0;
}

static const struct snd_kcontrol_new cdc_dma_config_controls[] = {
	SOC_ENUM_EXT("WSA_CDC_DMA_0 TX Format", cdc_dma_config_enum[0],
		     msm_dai_q6_cdc_dma_format_get,
		     msm_dai_q6_cdc_dma_format_put),
	SOC_ENUM_EXT("WSA_CDC_DMA_0 RX XTLoggingDisable",
		     xt_logging_disable_enum[0],
		     msm_dai_q6_cdc_dma_xt_logging_disable_get,
		     msm_dai_q6_cdc_dma_xt_logging_disable_put),
};

/* SOC probe for codec DMA interface */
static int msm_dai_q6_dai_cdc_dma_probe(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_cdc_dma_dai_data *dai_data = NULL;
	int rc = 0;

	if (!dai) {
		pr_err("%s: Invalid params dai\n", __func__);
		return -EINVAL;
	}
	if (!dai->dev) {
		pr_err("%s: Invalid params dai dev\n", __func__);
		return -EINVAL;
	}

	msm_dai_q6_set_dai_id(dai);
	dai_data = dev_get_drvdata(dai->dev);

	switch (dai->id) {
	case AFE_PORT_ID_WSA_CODEC_DMA_TX_0:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&cdc_dma_config_controls[0],
				 dai_data));
		break;
	case AFE_PORT_ID_WSA_CODEC_DMA_RX_0:
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(&cdc_dma_config_controls[1],
				 dai_data));
		break;
	default:
		break;
	}

	if (rc < 0)
		dev_err(dai->dev, "%s: err add config ctl, DAI = %s\n",
			__func__, dai->name);

	if (dai_data->is_island_dai)
		rc = msm_dai_q6_add_island_mx_ctls(
						dai->component->card->snd_card,
						dai->name, dai->id,
						(void *)dai_data);
	rc = msm_dai_q6_add_power_mode_mx_ctls(
						dai->component->card->snd_card,
						dai->name, dai->id,
						(void *)dai_data);
	rc= msm_dai_q6_add_isconfig_config_mx_ctls(
						dai->component->card->snd_card,
						dai->name, dai->id,
						(void *)dai_data);

	rc = msm_dai_q6_dai_add_route(dai);
	return rc;
}

static int msm_dai_q6_dai_cdc_dma_remove(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_cdc_dma_dai_data *dai_data =
		dev_get_drvdata(dai->dev);
	int rc = 0;

	/* If AFE port is still up, close it */
	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		dev_dbg(dai->dev, "%s: stop codec dma port:%d\n", __func__,
			dai->id);
		rc = afe_close(dai->id); /* can block */
		if (rc < 0)
			dev_err(dai->dev, "fail to close AFE port\n");
		clear_bit(STATUS_PORT_STARTED, dai_data->status_mask);
	}
	return rc;
}

static int msm_dai_q6_cdc_dma_set_channel_map(struct snd_soc_dai *dai,
			unsigned int tx_num_ch, unsigned int *tx_ch_mask,
			unsigned int rx_num_ch, unsigned int *rx_ch_mask)

{
	int rc = 0;
	struct msm_dai_q6_cdc_dma_dai_data *dai_data =
						dev_get_drvdata(dai->dev);
	unsigned int ch_mask = 0, ch_num = 0;

	dev_dbg(dai->dev, "%s: id = %d\n", __func__, dai->id);
	switch (dai->id) {
	case AFE_PORT_ID_WSA_CODEC_DMA_RX_0:
	case AFE_PORT_ID_WSA_CODEC_DMA_RX_1:
	case AFE_PORT_ID_RX_CODEC_DMA_RX_0:
	case AFE_PORT_ID_RX_CODEC_DMA_RX_1:
	case AFE_PORT_ID_RX_CODEC_DMA_RX_2:
	case AFE_PORT_ID_RX_CODEC_DMA_RX_3:
	case AFE_PORT_ID_RX_CODEC_DMA_RX_4:
	case AFE_PORT_ID_RX_CODEC_DMA_RX_5:
	case AFE_PORT_ID_RX_CODEC_DMA_RX_6:
	case AFE_PORT_ID_RX_CODEC_DMA_RX_7:
		if (!rx_ch_mask) {
			dev_err(dai->dev, "%s: invalid rx ch mask\n", __func__);
			return -EINVAL;
		}
		if (rx_num_ch > AFE_PORT_MAX_AUDIO_CHAN_CNT) {
			dev_err(dai->dev, "%s: invalid rx_num_ch %d\n",
				__func__, rx_num_ch);
			return -EINVAL;
		}
		ch_mask = *rx_ch_mask;
		ch_num = rx_num_ch;
		break;
	case AFE_PORT_ID_WSA_CODEC_DMA_TX_0:
	case AFE_PORT_ID_WSA_CODEC_DMA_TX_1:
	case AFE_PORT_ID_WSA_CODEC_DMA_TX_2:
	case AFE_PORT_ID_VA_CODEC_DMA_TX_0:
	case AFE_PORT_ID_VA_CODEC_DMA_TX_1:
	case AFE_PORT_ID_TX_CODEC_DMA_TX_0:
	case AFE_PORT_ID_TX_CODEC_DMA_TX_1:
	case AFE_PORT_ID_TX_CODEC_DMA_TX_2:
	case AFE_PORT_ID_TX_CODEC_DMA_TX_3:
	case AFE_PORT_ID_TX_CODEC_DMA_TX_4:
	case AFE_PORT_ID_TX_CODEC_DMA_TX_5:
		if (!tx_ch_mask) {
			dev_err(dai->dev, "%s: invalid tx ch mask\n", __func__);
			return -EINVAL;
		}
		if (tx_num_ch > AFE_PORT_MAX_AUDIO_CHAN_CNT) {
			dev_err(dai->dev, "%s: invalid tx_num_ch %d\n",
				__func__, tx_num_ch);
			return -EINVAL;
		}
		ch_mask = *tx_ch_mask;
		ch_num = tx_num_ch;
		break;
	default:
		dev_err(dai->dev, "%s: invalid dai id %d\n", __func__, dai->id);
		return -EINVAL;
	}

	dai_data->port_config.cdc_dma.active_channels_mask = ch_mask;
	dev_dbg(dai->dev, "%s: CDC_DMA_%d_ch cnt[%d] ch mask[0x%x]\n", __func__,
			dai->id, ch_num, ch_mask);
	return rc;
}

static int msm_dai_q6_cdc_dma_hw_params(
		struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_cdc_dma_dai_data *dai_data =
						dev_get_drvdata(dai->dev);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_SPECIAL:
		dai_data->port_config.cdc_dma.bit_width = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		dai_data->port_config.cdc_dma.bit_width = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		dai_data->port_config.cdc_dma.bit_width = 32;
		break;
	default:
		dev_err(dai->dev, "%s: format %d\n",
			__func__, params_format(params));
		return -EINVAL;
	}

	dai_data->rate = params_rate(params);
	dai_data->channels = params_channels(params);

	dai_data->port_config.cdc_dma.cdc_dma_cfg_minor_version =
				AFE_API_VERSION_CODEC_DMA_CONFIG;
	dai_data->port_config.cdc_dma.sample_rate = dai_data->rate;
	dai_data->port_config.cdc_dma.num_channels = dai_data->channels;
	dev_dbg(dai->dev, "%s: bit_wd[%hu] format[%hu]\n"
		"num_channel %hu sample_rate %d\n", __func__,
		dai_data->port_config.cdc_dma.bit_width,
		dai_data->port_config.cdc_dma.data_format,
		dai_data->port_config.cdc_dma.num_channels,
		dai_data->rate);

	return 0;
}

static int msm_dai_q6_cdc_dma_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_cdc_dma_dai_data *dai_data =
						dev_get_drvdata(dai->dev);
	int rc = 0;

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		if ((dai->id == AFE_PORT_ID_WSA_CODEC_DMA_TX_0) &&
			(dai_data->port_config.cdc_dma.data_format == 1))
			dai_data->port_config.cdc_dma.data_format =
				AFE_LINEAR_PCM_DATA_PACKED_16BIT;

		if (dai_data->cdc_dma_data_align) {
			rc = afe_send_cdc_dma_data_align(dai->id,
				dai_data->cdc_dma_data_align);
			if (rc)
				pr_debug("%s: afe send data alignment failed %d\n",
					__func__, rc);
		}
		rc = afe_port_start(dai->id, &dai_data->port_config,
						dai_data->rate);
		if (rc < 0)
			dev_err(dai->dev, "fail to open AFE port 0x%x\n",
				dai->id);
		else
			set_bit(STATUS_PORT_STARTED,
				dai_data->status_mask);
	}
	return rc;
}


static void msm_dai_q6_cdc_dma_shutdown(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct msm_dai_q6_cdc_dma_dai_data *dai_data =
					dev_get_drvdata(dai->dev);
	int rc = 0;

	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		dev_dbg(dai->dev, "%s: stop AFE port:%d\n", __func__,
			dai->id);
		rc = afe_close(dai->id); /* can block */
		if (rc < 0)
			dev_err(dai->dev, "fail to close AFE port\n");

		dev_dbg(dai->dev, "%s: dai_data->status_mask = %ld\n", __func__,
			*dai_data->status_mask);
		clear_bit(STATUS_PORT_STARTED, dai_data->status_mask);
	}

	if (test_bit(STATUS_PORT_STARTED, dai_data->hwfree_status))
		clear_bit(STATUS_PORT_STARTED, dai_data->hwfree_status);
}

static int msm_dai_q6_cdc_dma_digital_mute(struct snd_soc_dai *dai,
				       int mute)
{
	int port_id = dai->id;
	struct msm_dai_q6_cdc_dma_dai_data *dai_data =
					dev_get_drvdata(dai->dev);

	if (mute && !dai_data->xt_logging_disable)
		afe_get_sp_xt_logging_data(port_id);

	return 0;
}

static struct snd_soc_dai_ops msm_dai_q6_cdc_dma_ops = {
	.prepare          = msm_dai_q6_cdc_dma_prepare,
	.hw_params        = msm_dai_q6_cdc_dma_hw_params,
	.shutdown         = msm_dai_q6_cdc_dma_shutdown,
	.set_channel_map = msm_dai_q6_cdc_dma_set_channel_map,
};

static struct snd_soc_dai_ops msm_dai_q6_cdc_wsa_dma_ops = {
	.prepare          = msm_dai_q6_cdc_dma_prepare,
	.hw_params        = msm_dai_q6_cdc_dma_hw_params,
	.shutdown         = msm_dai_q6_cdc_dma_shutdown,
	.set_channel_map = msm_dai_q6_cdc_dma_set_channel_map,
	.digital_mute = msm_dai_q6_cdc_dma_digital_mute,
};

static struct snd_soc_dai_driver msm_dai_q6_cdc_dma_dai[] = {
	{
		.playback = {
			.stream_name = "WSA CDC DMA0 Playback",
			.aif_name = "WSA_CDC_DMA_RX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "WSA_CDC_DMA_RX_0",
		.ops = &msm_dai_q6_cdc_wsa_dma_ops,
		.id = AFE_PORT_ID_WSA_CODEC_DMA_RX_0,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "WSA CDC DMA0 Capture",
			.aif_name = "WSA_CDC_DMA_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "WSA_CDC_DMA_TX_0",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_WSA_CODEC_DMA_TX_0,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
		},
	{
		.playback = {
			.stream_name = "WSA CDC DMA1 Playback",
			.aif_name = "WSA_CDC_DMA_RX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "WSA_CDC_DMA_RX_1",
		.ops = &msm_dai_q6_cdc_wsa_dma_ops,
		.id = AFE_PORT_ID_WSA_CODEC_DMA_RX_1,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "WSA CDC DMA1 Capture",
			.aif_name = "WSA_CDC_DMA_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "WSA_CDC_DMA_TX_1",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_WSA_CODEC_DMA_TX_1,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "WSA CDC DMA2 Capture",
			.aif_name = "WSA_CDC_DMA_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "WSA_CDC_DMA_TX_2",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_WSA_CODEC_DMA_TX_2,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "VA CDC DMA0 Capture",
			.aif_name = "VA_CDC_DMA_TX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "VA_CDC_DMA_TX_0",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_VA_CODEC_DMA_TX_0,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "VA CDC DMA1 Capture",
			.aif_name = "VA_CDC_DMA_TX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "VA_CDC_DMA_TX_1",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_VA_CODEC_DMA_TX_1,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "VA CDC DMA2 Capture",
			.aif_name = "VA_CDC_DMA_TX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "VA_CDC_DMA_TX_2",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_VA_CODEC_DMA_TX_2,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.playback = {
			.stream_name = "RX CDC DMA0 Playback",
			.aif_name = "RX_CDC_DMA_RX_0",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "RX_CDC_DMA_RX_0",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_RX_CODEC_DMA_RX_0,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "TX CDC DMA0 Capture",
			.aif_name = "TX_CDC_DMA_TX_0",
			.rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_192000 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 3,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "TX_CDC_DMA_TX_0",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_TX_CODEC_DMA_TX_0,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.playback = {
			.stream_name = "RX CDC DMA1 Playback",
			.aif_name = "RX_CDC_DMA_RX_1",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "RX_CDC_DMA_RX_1",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_RX_CODEC_DMA_RX_1,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "TX CDC DMA1 Capture",
			.aif_name = "TX_CDC_DMA_TX_1",
			.rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_192000 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 3,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "TX_CDC_DMA_TX_1",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_TX_CODEC_DMA_TX_1,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.playback = {
			.stream_name = "RX CDC DMA2 Playback",
			.aif_name = "RX_CDC_DMA_RX_2",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "RX_CDC_DMA_RX_2",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_RX_CODEC_DMA_RX_2,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "TX CDC DMA2 Capture",
			.aif_name = "TX_CDC_DMA_TX_2",
			.rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_192000 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "TX_CDC_DMA_TX_2",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_TX_CODEC_DMA_TX_2,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},	{
		.playback = {
			.stream_name = "RX CDC DMA3 Playback",
			.aif_name = "RX_CDC_DMA_RX_3",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "RX_CDC_DMA_RX_3",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_RX_CODEC_DMA_RX_3,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "TX CDC DMA3 Capture",
			.aif_name = "TX_CDC_DMA_TX_3",
			.rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_192000 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "TX_CDC_DMA_TX_3",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_TX_CODEC_DMA_TX_3,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.playback = {
			.stream_name = "RX CDC DMA4 Playback",
			.aif_name = "RX_CDC_DMA_RX_4",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 6,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "RX_CDC_DMA_RX_4",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_RX_CODEC_DMA_RX_4,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "TX CDC DMA4 Capture",
			.aif_name = "TX_CDC_DMA_TX_4",
			.rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_192000 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "TX_CDC_DMA_TX_4",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_TX_CODEC_DMA_TX_4,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.playback = {
			.stream_name = "RX CDC DMA5 Playback",
			.aif_name = "RX_CDC_DMA_RX_5",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 1,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "RX_CDC_DMA_RX_5",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_RX_CODEC_DMA_RX_5,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.capture = {
			.stream_name = "TX CDC DMA5 Capture",
			.aif_name = "TX_CDC_DMA_TX_5",
			.rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 |
				SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_192000 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "TX_CDC_DMA_TX_5",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_TX_CODEC_DMA_TX_5,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.playback = {
			.stream_name = "RX CDC DMA6 Playback",
			.aif_name = "RX_CDC_DMA_RX_6",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 4,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "RX_CDC_DMA_RX_6",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_RX_CODEC_DMA_RX_6,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
	{
		.playback = {
			.stream_name = "RX CDC DMA7 Playback",
			.aif_name = "RX_CDC_DMA_RX_7",
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |
				SNDRV_PCM_RATE_384000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
			.channels_min = 1,
			.channels_max = 2,
			.rate_min = 8000,
			.rate_max = 384000,
		},
		.name = "RX_CDC_DMA_RX_7",
		.ops = &msm_dai_q6_cdc_dma_ops,
		.id = AFE_PORT_ID_RX_CODEC_DMA_RX_7,
		.probe = msm_dai_q6_dai_cdc_dma_probe,
		.remove = msm_dai_q6_dai_cdc_dma_remove,
	},
};

static const struct snd_soc_component_driver msm_q6_cdc_dma_dai_component = {
	.name = "msm-dai-cdc-dma-dev",
};

/* DT related probe for each codec DMA interface device */
static int msm_dai_q6_cdc_dma_dev_probe(struct platform_device *pdev)
{
	const char *q6_cdc_dma_dev_id = "qcom,msm-dai-cdc-dma-dev-id";
	u32 cdc_dma_id = 0;
	int i;
	int rc = 0;
	struct msm_dai_q6_cdc_dma_dai_data *dai_data = NULL;

	rc = of_property_read_u32(pdev->dev.of_node, q6_cdc_dma_dev_id,
				  &cdc_dma_id);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: missing 0x%x in dt node\n", __func__, cdc_dma_id);
		return rc;
	}

	dev_dbg(&pdev->dev, "%s: dev name %s dev id 0x%x\n", __func__,
		dev_name(&pdev->dev), cdc_dma_id);

	pdev->id = cdc_dma_id;

	dai_data = devm_kzalloc(&pdev->dev,
				sizeof(struct msm_dai_q6_cdc_dma_dai_data),
				GFP_KERNEL);

	if (!dai_data)
		return -ENOMEM;

	rc = of_property_read_u32(pdev->dev.of_node,
				    "qcom,msm-dai-is-island-supported",
				    &dai_data->is_island_dai);
	if (rc)
		dev_dbg(&pdev->dev, "island supported entry not found\n");

	rc = of_property_read_u32(pdev->dev.of_node,
				"qcom,msm-cdc-dma-data-align",
				&dai_data->cdc_dma_data_align);
	if (rc)
		dev_dbg(&pdev->dev, "cdc dma data align supported entry not found\n");

	dev_set_drvdata(&pdev->dev, dai_data);

	for (i = 0; i < ARRAY_SIZE(msm_dai_q6_cdc_dma_dai); i++) {
		if (msm_dai_q6_cdc_dma_dai[i].id == cdc_dma_id) {
			return snd_soc_register_component(&pdev->dev,
				&msm_q6_cdc_dma_dai_component,
				&msm_dai_q6_cdc_dma_dai[i], 1);
		}
	}
	return -ENODEV;
}

static int msm_dai_q6_cdc_dma_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_dai_q6_cdc_dma_dev_dt_match[] = {
	{ .compatible = "qcom,msm-dai-cdc-dma-dev", },
	{ }
};

MODULE_DEVICE_TABLE(of, msm_dai_q6_cdc_dma_dev_dt_match);

static struct platform_driver msm_dai_q6_cdc_dma_driver = {
	.probe  = msm_dai_q6_cdc_dma_dev_probe,
	.remove  = msm_dai_q6_cdc_dma_dev_remove,
	.driver = {
		.name = "msm-dai-cdc-dma-dev",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_q6_cdc_dma_dev_dt_match,
		.suppress_bind_attrs = true,
	},
};

/* DT related probe for codec DMA interface device group */
static int msm_dai_cdc_dma_q6_probe(struct platform_device *pdev)
{
	int rc;

	rc = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (rc) {
		dev_err(&pdev->dev, "%s: failed to add child nodes, rc=%d\n",
			__func__, rc);
	} else
		dev_dbg(&pdev->dev, "%s: added child node\n", __func__);
	return rc;
}

static int msm_dai_cdc_dma_q6_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_dai_cdc_dma_dt_match[] = {
	{ .compatible = "qcom,msm-dai-cdc-dma", },
	{ }
};

MODULE_DEVICE_TABLE(of, msm_dai_cdc_dma_dt_match);

static struct platform_driver msm_dai_cdc_dma_q6 = {
	.probe  = msm_dai_cdc_dma_q6_probe,
	.remove = msm_dai_cdc_dma_q6_remove,
	.driver = {
		.name = "msm-dai-cdc-dma",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_cdc_dma_dt_match,
		.suppress_bind_attrs = true,
	},
};

int __init msm_dai_q6_init(void)
{
	int rc;
#ifndef CONFIG_AUXPCM_DISABLE
	rc = platform_driver_register(&msm_auxpcm_dev_driver);
	if (rc) {
		pr_err("%s: fail to register auxpcm dev driver", __func__);
		return rc;
	}
#endif
	rc = platform_driver_register(&msm_dai_q6);
	if (rc) {
		pr_err("%s: fail to register dai q6 driver", __func__);
		goto dai_q6_fail;
	}

	rc = platform_driver_register(&msm_dai_q6_dev);
	if (rc) {
		pr_err("%s: fail to register dai q6 dev driver", __func__);
		goto dai_q6_dev_fail;
	}

	rc = platform_driver_register(&msm_dai_q6_mi2s_driver);
	if (rc) {
		pr_err("%s: fail to register dai MI2S dev drv\n", __func__);
		goto dai_q6_mi2s_drv_fail;
	}

	rc = platform_driver_register(&msm_dai_q6_meta_mi2s_driver);
	if (rc) {
		pr_err("%s: fail to register dai META MI2S dev drv\n",
			__func__);
		goto dai_q6_meta_mi2s_drv_fail;
	}

	rc = platform_driver_register(&msm_dai_mi2s_q6);
	if (rc) {
		pr_err("%s: fail to register dai MI2S\n", __func__);
		goto dai_mi2s_q6_fail;
	}

	rc = platform_driver_register(&msm_dai_q6_spdif_driver);
	if (rc) {
		pr_err("%s: fail to register dai SPDIF\n", __func__);
		goto dai_spdif_q6_fail;
	}

	rc = platform_driver_register(&msm_dai_q6_tdm_driver);
	if (rc) {
		pr_err("%s: fail to register dai TDM dev drv\n", __func__);
		goto dai_q6_tdm_drv_fail;
	}

	rc = platform_driver_register(&msm_dai_tdm_q6);
	if (rc) {
		pr_err("%s: fail to register dai TDM\n", __func__);
		goto dai_tdm_q6_fail;
	}

	rc = platform_driver_register(&msm_dai_q6_cdc_dma_driver);
	if (rc) {
		pr_err("%s: fail to register dai CDC DMA dev\n", __func__);
		goto dai_cdc_dma_q6_dev_fail;
	}


	rc = platform_driver_register(&msm_dai_cdc_dma_q6);
	if (rc) {
		pr_err("%s: fail to register dai CDC DMA\n", __func__);
		goto dai_cdc_dma_q6_fail;
	}
	return rc;

dai_cdc_dma_q6_fail:
	platform_driver_unregister(&msm_dai_q6_cdc_dma_driver);
dai_cdc_dma_q6_dev_fail:
	platform_driver_unregister(&msm_dai_tdm_q6);
dai_tdm_q6_fail:
	platform_driver_unregister(&msm_dai_q6_tdm_driver);
dai_q6_tdm_drv_fail:
	platform_driver_unregister(&msm_dai_q6_spdif_driver);
dai_spdif_q6_fail:
	platform_driver_unregister(&msm_dai_mi2s_q6);
dai_mi2s_q6_fail:
	platform_driver_unregister(&msm_dai_q6_meta_mi2s_driver);
dai_q6_meta_mi2s_drv_fail:
	platform_driver_unregister(&msm_dai_q6_mi2s_driver);
dai_q6_mi2s_drv_fail:
	platform_driver_unregister(&msm_dai_q6_dev);
dai_q6_dev_fail:
	platform_driver_unregister(&msm_dai_q6);
dai_q6_fail:
	platform_driver_unregister(&msm_auxpcm_dev_driver);
	return rc;
}

void msm_dai_q6_exit(void)
{
	platform_driver_unregister(&msm_dai_cdc_dma_q6);
	platform_driver_unregister(&msm_dai_q6_cdc_dma_driver);
	platform_driver_unregister(&msm_dai_tdm_q6);
	platform_driver_unregister(&msm_dai_q6_tdm_driver);
	platform_driver_unregister(&msm_dai_q6_spdif_driver);
	platform_driver_unregister(&msm_dai_mi2s_q6);
	platform_driver_unregister(&msm_dai_q6_meta_mi2s_driver);
	platform_driver_unregister(&msm_dai_q6_mi2s_driver);
	platform_driver_unregister(&msm_dai_q6_dev);
	platform_driver_unregister(&msm_dai_q6);
	platform_driver_unregister(&msm_auxpcm_dev_driver);
}

/* Module information */
MODULE_DESCRIPTION("MSM DSP DAI driver");
MODULE_LICENSE("GPL v2");
