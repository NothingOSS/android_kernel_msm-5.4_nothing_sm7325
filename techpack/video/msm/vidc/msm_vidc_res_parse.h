/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2012-2018, The Linux Foundation. All rights reserved.
 */

#ifndef DT_PARSE
#define DT_PARSE
#include <linux/of.h>
#include "msm_vidc_resources.h"
#include "msm_vidc_common.h"
void msm_vidc_free_platform_resources(
		struct msm_vidc_platform_resources *res);

int read_hfi_type(struct platform_device *pdev);

int read_platform_resources_from_drv_data(
		struct msm_vidc_core *core);
int read_platform_resources_from_dt(
		struct msm_vidc_platform_resources *res);

int read_context_bank_resources_from_dt(struct platform_device *pdev);

int read_bus_resources_from_dt(struct platform_device *pdev);
int read_mem_cdsp_resources_from_dt(struct platform_device *pdev);

int msm_vidc_load_u32_table(struct platform_device *pdev,
		struct device_node *of_node, char *table_name, int struct_size,
		u32 **table, u32 *num_elements);

#endif
