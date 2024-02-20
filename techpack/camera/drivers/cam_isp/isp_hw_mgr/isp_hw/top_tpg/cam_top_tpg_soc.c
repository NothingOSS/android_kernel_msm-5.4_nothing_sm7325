// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 */
#include <linux/slab.h>
#include "cam_top_tpg_soc.h"
#include "cam_cpas_api.h"
#include "cam_debug_util.h"

int cam_top_tpg_init_soc_resources(struct cam_hw_soc_info *soc_info,
	irq_handler_t tpg_irq_handler, void *irq_data)
{
	int rc = 0;
	struct cam_cpas_register_params   cpas_register_param;
	struct cam_top_tpg_soc_private    *soc_private;

	soc_private = kzalloc(sizeof(struct cam_top_tpg_soc_private),
		GFP_KERNEL);
	if (!soc_private)
		return -ENOMEM;

	soc_info->soc_private = soc_private;

	rc = cam_soc_util_get_dt_properties(soc_info);
	if (rc < 0)
		return rc;

	/* Need to see if we want post process the clock list */
	rc = cam_soc_util_request_platform_resource(soc_info, tpg_irq_handler,
		irq_data);

	if (rc < 0) {
		CAM_ERR(CAM_ISP,
			"Error Request platform resources failed rc=%d", rc);
		goto free_soc_private;
	}

	memset(&cpas_register_param, 0, sizeof(cpas_register_param));
	strlcpy(cpas_register_param.identifier, "tpg",
		CAM_HW_IDENTIFIER_LENGTH);
	cpas_register_param.cell_index = soc_info->index;
	cpas_register_param.dev = soc_info->dev;
	rc = cam_cpas_register_client(&cpas_register_param);
	if (rc) {
		CAM_ERR(CAM_ISP, "CPAS registration failed rc=%d", rc);
		goto release_soc;
	} else {
		soc_private->cpas_handle = cpas_register_param.client_handle;
	}

	return rc;

release_soc:
	cam_soc_util_release_platform_resource(soc_info);
free_soc_private:
	kfree(soc_private);

	return rc;
}

int cam_top_tpg_deinit_soc_resources(
	struct cam_hw_soc_info *soc_info)
{
	int rc = 0;
	struct cam_top_tpg_soc_private       *soc_private;

	soc_private = soc_info->soc_private;
	if (!soc_private) {
		CAM_ERR(CAM_ISP, "Error soc_private NULL");
		return -ENODEV;
	}

	rc = cam_cpas_unregister_client(soc_private->cpas_handle);
	if (rc)
		CAM_ERR(CAM_ISP, "CPAS unregistration failed rc=%d", rc);

	rc = cam_soc_util_release_platform_resource(soc_info);

	return rc;
}

int cam_top_tpg_enable_soc_resources(
	struct cam_hw_soc_info *soc_info, enum cam_vote_level clk_level)
{
	int rc = 0;
	struct cam_top_tpg_soc_private       *soc_private;
	struct cam_ahb_vote ahb_vote;
	struct cam_axi_vote axi_vote = {0};

	soc_private = soc_info->soc_private;

	ahb_vote.type = CAM_VOTE_ABSOLUTE;
	ahb_vote.vote.level = CAM_SVS_VOTE;
	axi_vote.num_paths = 1;
	axi_vote.axi_path[0].path_data_type = CAM_AXI_PATH_DATA_ALL;
	axi_vote.axi_path[0].transac_type = CAM_AXI_TRANSACTION_WRITE;

	axi_vote.axi_path[0].camnoc_bw = CAM_CPAS_DEFAULT_AXI_BW;
	axi_vote.axi_path[0].mnoc_ab_bw = CAM_CPAS_DEFAULT_AXI_BW;
	axi_vote.axi_path[0].mnoc_ib_bw = CAM_CPAS_DEFAULT_AXI_BW;

	CAM_DBG(CAM_ISP, "csid camnoc_bw:%lld mnoc_ab_bw:%lld mnoc_ib_bw:%lld ",
		axi_vote.axi_path[0].camnoc_bw,
		axi_vote.axi_path[0].mnoc_ab_bw,
		axi_vote.axi_path[0].mnoc_ib_bw);

	rc = cam_cpas_start(soc_private->cpas_handle, &ahb_vote, &axi_vote);
	if (rc) {
		CAM_ERR(CAM_ISP, "Error CPAS start failed");
		rc = -EFAULT;
		goto end;
	}

	rc = cam_soc_util_enable_platform_resource(soc_info, true,
		clk_level, false);
	if (rc) {
		CAM_ERR(CAM_ISP, "enable platform failed");
		goto stop_cpas;
	}

	return rc;

stop_cpas:
	cam_cpas_stop(soc_private->cpas_handle);
end:
	return rc;
}

int cam_top_tpg_disable_soc_resources(struct cam_hw_soc_info *soc_info)
{
	int rc = 0;
	struct cam_top_tpg_soc_private       *soc_private;

	if (!soc_info) {
		CAM_ERR(CAM_ISP, "Error Invalid params");
		return -EINVAL;
	}
	soc_private = soc_info->soc_private;

	rc = cam_soc_util_disable_platform_resource(soc_info, true, false);
	if (rc)
		CAM_ERR(CAM_ISP, "Disable platform failed");

	rc = cam_cpas_stop(soc_private->cpas_handle);
	if (rc) {
		CAM_ERR(CAM_ISP, "Error CPAS stop failed rc=%d", rc);
		return rc;
	}

	return rc;
}

