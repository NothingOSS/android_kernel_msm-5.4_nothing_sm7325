/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 */

#ifndef CAM_IPE_HW_INTF_H
#define CAM_IPE_HW_INTF_H

#include <media/cam_defs.h>
#include <media/cam_icp.h>
#include "cam_hw_mgr_intf.h"
#include "cam_icp_hw_intf.h"

/* IPE registers */
#define IPE_TOP_RST_CMD              0x1008
#define IPE_CDM_RST_CMD              0x10
#define IPE_CDM_IRQ_STATUS           0x44
#define IPE_TOP_IRQ_STATUS           0x100C

/* IPE CDM/TOP status register */
#define IPE_RST_DONE_IRQ_STATUS_BIT  0x1

enum cam_icp_ipe_cmd_type {
	CAM_ICP_IPE_CMD_FW_DOWNLOAD,
	CAM_ICP_IPE_CMD_POWER_COLLAPSE,
	CAM_ICP_IPE_CMD_POWER_RESUME,
	CAM_ICP_IPE_CMD_SET_FW_BUF,
	CAM_ICP_IPE_CMD_VOTE_CPAS,
	CAM_ICP_IPE_CMD_CPAS_START,
	CAM_ICP_IPE_CMD_CPAS_STOP,
	CAM_ICP_IPE_CMD_UPDATE_CLK,
	CAM_ICP_IPE_CMD_DISABLE_CLK,
	CAM_ICP_IPE_CMD_RESET,
	CAM_ICP_IPE_CMD_MAX,
};

#endif /* CAM_IPE_HW_INTF_H */
