// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <media/cam_defs.h>
#include <media/cam_icp.h>
#include "ipe_soc.h"
#include "cam_soc_util.h"
#include "cam_debug_util.h"


int cam_ipe_transfer_gdsc_control(struct cam_hw_soc_info *soc_info)
{
	int i;
	int rc;

	for (i = 0; i < soc_info->num_rgltr; i++) {
		rc = regulator_set_mode(soc_info->rgltr[i],
			REGULATOR_MODE_FAST);
		if (rc) {
			CAM_ERR(CAM_ICP, "Regulator set mode %s failed",
				soc_info->rgltr_name[i]);
			goto rgltr_set_mode_failed;
		}
	}
	return 0;

rgltr_set_mode_failed:
	for (i = i - 1; i >= 0; i--)
		if (soc_info->rgltr[i])
			regulator_set_mode(soc_info->rgltr[i],
				REGULATOR_MODE_NORMAL);

	return rc;
}

int cam_ipe_get_gdsc_control(struct cam_hw_soc_info *soc_info)
{
	int i;
	int rc;

	for (i = 0; i < soc_info->num_rgltr; i++) {
		rc = regulator_set_mode(soc_info->rgltr[i],
					REGULATOR_MODE_NORMAL);
		if (rc) {
			CAM_ERR(CAM_ICP, "Regulator set mode %s failed",
				soc_info->rgltr_name[i]);
			goto rgltr_set_mode_failed;
		}
	}
	return 0;

rgltr_set_mode_failed:
	for (i = i - 1; i >= 0; i--)
		if (soc_info->rgltr[i])
			regulator_set_mode(soc_info->rgltr[i],
					REGULATOR_MODE_FAST);

	return rc;
}

static int cam_ipe_get_dt_properties(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc < 0)
		CAM_ERR(CAM_ICP, "get ipe dt prop is failed");

	return rc;
}

static int cam_ipe_request_platform_resource(
	struct cam_hw_soc_info *soc_info,
	irq_handler_t ipe_irq_handler, void *irq_data)
{
	int rc = 0;

	rc = cam_soc_util_request_platform_resource(soc_info, ipe_irq_handler,
		irq_data);

	return rc;
}

int cam_ipe_init_soc_resources(struct cam_hw_soc_info *soc_info,
	irq_handler_t ipe_irq_handler, void *irq_data)
{
	int rc = 0;

	rc = cam_ipe_get_dt_properties(soc_info);
	if (rc < 0)
		return rc;

	rc = cam_ipe_request_platform_resource(soc_info, ipe_irq_handler,
		irq_data);
	if (rc < 0)
		return rc;

	return rc;
}

void cam_ipe_deinit_soc_resources(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = cam_soc_util_release_platform_resource(soc_info);
	if (rc)
		CAM_WARN(CAM_ICP, "release platform resources fail");
}

int cam_ipe_enable_soc_resources(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;

	rc = cam_soc_util_enable_platform_resource(soc_info, true,
		CAM_SVS_VOTE, false);
	if (rc) {
		CAM_ERR(CAM_ICP, "enable platform failed");
		return rc;
	}

	return rc;
}

int cam_ipe_disable_soc_resources(struct cam_hw_soc_info *soc_info,
	bool disable_clk)
{
	int rc = 0;

	rc = cam_soc_util_disable_platform_resource(soc_info, disable_clk,
		false);
	if (rc)
		CAM_ERR(CAM_ICP, "enable platform failed");

	return rc;
}

int cam_ipe_update_clk_rate(struct cam_hw_soc_info *soc_info,
	uint32_t *clk_rate)
{
	int32_t src_clk_idx;

	if (!soc_info)
		return -EINVAL;

	src_clk_idx = soc_info->src_clk_idx;

	if ((soc_info->clk_level_valid[CAM_TURBO_VOTE] == true) &&
		(soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx] != 0) &&
		(*clk_rate > soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx])) {
		CAM_DBG(CAM_PERF, "clk_rate %d greater than max, reset to %d",
			*clk_rate,
			soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx]);
		*clk_rate = soc_info->clk_rate[CAM_TURBO_VOTE][src_clk_idx];
	}

	return cam_soc_util_set_src_clk_rate(soc_info, *clk_rate);
}

int cam_ipe_toggle_clk(struct cam_hw_soc_info *soc_info, bool clk_enable)
{
	int rc = 0;

	if (clk_enable)
		rc = cam_soc_util_clk_enable_default(soc_info, CAM_SVS_VOTE);
	else
		cam_soc_util_clk_disable_default(soc_info);

	return rc;
}
