/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2013-2021, The Linux Foundation. All rights reserved.
 */

#ifndef __MSM_VIDC_RESOURCES_H__
#define __MSM_VIDC_RESOURCES_H__

#include <linux/platform_device.h>
#include "msm_vidc.h"
#include <linux/soc/qcom/llcc-qcom.h>
#include <soc/qcom/cx_ipeak.h>

#define MAX_BUFFER_TYPES 32

struct reg_value_pair {
	u32 reg;
	u32 value;
	u32 mask;
};

struct reg_set {
	struct reg_value_pair *reg_tbl;
	int count;
};

struct addr_range {
	u32 start;
	u32 size;
};

struct addr_set {
	struct addr_range *addr_tbl;
	int count;
};

struct context_bank_info {
	struct list_head list;
	const char *name;
	u32 buffer_type;
	bool is_secure;
	struct addr_range addr_range;
	struct device *dev;
	struct iommu_domain *domain;
};

struct buffer_usage_table {
	u32 buffer_type;
	u32 tz_usage;
};

struct buffer_usage_set {
	struct buffer_usage_table *buffer_usage_tbl;
	u32 count;
};

struct regulator_info {
	struct regulator *regulator;
	bool has_hw_power_collapse;
	char *name;
};

struct regulator_set {
	struct regulator_info *regulator_tbl;
	u32 count;
};

struct clock_info {
	const char *name;
	struct clk *clk;
	u32 count;
	bool has_scaling;
	bool has_mem_retention;
};

struct clock_set {
	struct clock_info *clock_tbl;
	u32 count;
};

struct bus_info {
	const char *name;
	unsigned int range[2];
	struct device *dev;
	struct icc_path *path;
};

struct bus_set {
	struct bus_info *bus_tbl;
	u32 count;
};

struct reset_info {
	struct reset_control *rst;
	const char *name;
};

struct reset_set {
	struct reset_info *reset_tbl;
	u32 count;
};

struct allowed_clock_rates_table {
	u32 clock_rate;
};

struct memory_limit_table {
	u32 ddr_size; /* mega bytes */
	u32 mem_limit; /* mega bytes */
};

struct clock_profile_entry {
	u32 codec_mask;
	u32 vpp_cycles;
	u32 vsp_cycles;
	u32 low_power_cycles;
};

struct clock_freq_table {
	struct clock_profile_entry *clk_prof_entries;
	u32 count;
};

struct subcache_info {
	const char *name;
	bool isactive;
	bool isset;
	struct llcc_slice_desc *subcache;
};

struct subcache_set {
	struct subcache_info *subcache_tbl;
	u32 count;
};

struct msm_vidc_mem_cdsp {
	struct device *dev;
};

struct msm_vidc_platform_resources {
	phys_addr_t firmware_base;
	phys_addr_t register_base;
	uint32_t register_size;
	uint32_t irq;
	uint32_t sku_version;
	struct allowed_clock_rates_table *allowed_clks_tbl;
	u32 allowed_clks_tbl_size;
	struct clock_freq_table clock_freq_tbl;
	struct memory_limit_table *mem_limit_tbl;
	u32 memory_limit_table_size;
	bool sys_cache_present;
	bool sys_cache_res_set;
	struct subcache_set subcache_set;
	struct reg_set reg_set;
	struct addr_set qdss_addr_set;
	struct buffer_usage_set buffer_usage_set;
	uint32_t max_load;
	uint32_t max_image_load;
	uint32_t max_mbpf;
	uint32_t max_hq_mbs_per_frame;
	uint32_t max_hq_mbs_per_sec;
	uint32_t max_bframe_mbs_per_frame;
	uint32_t max_bframe_mbs_per_sec;
	struct platform_device *pdev;
	struct regulator_set regulator_set;
	struct clock_set clock_set;
	struct bus_set bus_set;
	struct reset_set reset_set;
	bool sw_power_collapsible;
	bool slave_side_cp;
	struct list_head context_banks;
	struct mutex cb_lock;
	bool thermal_mitigable;
	const char *fw_name;
	const char *hfi_version;
	bool never_unload_fw;
	bool debug_timeout;
	uint32_t max_inst_count;
	uint32_t max_secure_inst_count;
	uint32_t prefetch_pix_buf_count;
	uint32_t prefetch_pix_buf_size;
	uint32_t prefetch_non_pix_buf_count;
	uint32_t prefetch_non_pix_buf_size;
	int msm_vidc_hw_rsp_timeout;
	int msm_vidc_firmware_unload_delay;
	uint32_t msm_vidc_pwr_collapse_delay;
	bool non_fatal_pagefaults;
	bool cache_pagetables;
	bool decode_batching;
	uint32_t batch_timeout;
	bool dcvs;
	struct msm_vidc_codec_data *codec_data;
	int codec_data_count;
	struct msm_vidc_codec *codecs;
	uint32_t codecs_count;
	struct msm_vidc_codec_capability *codec_caps;
	uint32_t codec_caps_count;
	struct msm_vidc_vpss_capability *vpss_caps;
	uint32_t vpss_caps_count;
	struct msm_vidc_csc_coeff *csc_coeff_data;
	struct msm_vidc_mem_cdsp mem_cdsp;
	uint32_t vpu_ver;
	uint32_t fw_cycles;
	uint32_t fw_vpp_cycles;
	uint32_t avsync_window_size;
	struct msm_vidc_ubwc_config_data *ubwc_config;
	uint32_t clk_freq_threshold;
	struct cx_ipeak_client *cx_ipeak_context;
	uint32_t ubwc_stats_in_fbd;
	uint32_t has_vpp_delay;
	bool enc_auto_dynamic_fps;
	bool no_cvp;
};

static inline bool is_iommu_present(struct msm_vidc_platform_resources *res)
{
	return !list_empty(&res->context_banks);
}

#endif

