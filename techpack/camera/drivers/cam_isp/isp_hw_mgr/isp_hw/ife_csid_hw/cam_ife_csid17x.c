// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */


#include <linux/module.h>
#include "cam_ife_csid_core.h"
#include "cam_ife_csid170.h"
#include "cam_ife_csid170_200.h"
#include "cam_ife_csid175.h"
#include "cam_ife_csid175_200.h"
#include "cam_ife_csid480.h"
#include "cam_ife_csid_dev.h"
#include "camera_main.h"

#define CAM_CSID_DRV_NAME                    "csid_17x"
#define CAM_CSID_VERSION_V170                 0x10070000
#define CAM_CSID_VERSION_V175                 0x10070050
#define CAM_CSID_VERSION_V480                 0x40080000
#define CAM_CSID_VERSION_V165                 0x10060050

static struct cam_ife_csid_hw_info cam_ife_csid170_hw_info = {
	.csid_reg = &cam_ife_csid_170_reg_offset,
	.hw_dts_version = CAM_CSID_VERSION_V170,
};

static struct cam_ife_csid_hw_info cam_ife_csid170_200_hw_info = {
	.csid_reg = &cam_ife_csid_170_200_reg_offset,
	.hw_dts_version = CAM_CSID_VERSION_V170,
};

static struct cam_ife_csid_hw_info cam_ife_csid175_hw_info = {
	.csid_reg = &cam_ife_csid_175_reg_offset,
	.hw_dts_version = CAM_CSID_VERSION_V175,
};

static struct cam_ife_csid_hw_info cam_ife_csid175_200_hw_info = {
	.csid_reg = &cam_ife_csid_175_200_reg_offset,
	.hw_dts_version = CAM_CSID_VERSION_V175,
};

static struct cam_ife_csid_hw_info cam_ife_csid480_hw_info = {
	.csid_reg = &cam_ife_csid_480_reg_offset,
	.hw_dts_version = CAM_CSID_VERSION_V480,
};

static struct cam_ife_csid_hw_info cam_ife_csid165_204_hw_info = {
	.csid_reg = &cam_ife_csid_175_200_reg_offset,
	.hw_dts_version = CAM_CSID_VERSION_V165,
};

static const struct of_device_id cam_ife_csid17x_dt_match[] = {
	{
		.compatible = "qcom,csid170",
		.data = &cam_ife_csid170_hw_info,
	},
	{
		.compatible = "qcom,csid170_200",
		.data = &cam_ife_csid170_200_hw_info,
	},
	{
		.compatible = "qcom,csid175",
		.data = &cam_ife_csid175_hw_info,
	},
	{
		.compatible = "qcom,csid175_200",
		.data = &cam_ife_csid175_200_hw_info,
	},
	{
		.compatible = "qcom,csid480",
		.data = &cam_ife_csid480_hw_info,
	},
	{
		.compatible = "qcom,csid580",
		.data = &cam_ife_csid480_hw_info,
	},
	{
		.compatible = "qcom,csid165_204",
		.data = &cam_ife_csid165_204_hw_info,
	},
	{}
};

MODULE_DEVICE_TABLE(of, cam_ife_csid17x_dt_match);

struct platform_driver cam_ife_csid17x_driver = {
	.probe = cam_ife_csid_probe,
	.remove = cam_ife_csid_remove,
	.driver = {
		.name = CAM_CSID_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cam_ife_csid17x_dt_match,
		.suppress_bind_attrs = true,
	},
};

int cam_ife_csid17x_init_module(void)
{
	return platform_driver_register(&cam_ife_csid17x_driver);
}

void cam_ife_csid17x_exit_module(void)
{
	platform_driver_unregister(&cam_ife_csid17x_driver);
}

MODULE_DESCRIPTION("CAM IFE_CSID17X driver");
MODULE_LICENSE("GPL v2");
