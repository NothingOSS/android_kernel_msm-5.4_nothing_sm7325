/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2019,2021 The Linux Foundation. All rights reserved.
 */

#ifndef _DSI_PWR_H_
#define _DSI_PWR_H_

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>

struct dsi_parser_utils;

/**
 * struct dsi_vreg - regulator information for DSI regulators
 * @vreg:            Handle to the regulator.
 * @vreg_name:       Regulator name.
 * @min_voltage:     Minimum voltage in uV.
 * @max_voltage:     Maximum voltage in uV.
 * @enable_load:     Load, in uA, when enabled.
 * @disable_load:    Load, in uA, when disabled.
 * @ulp_load:        Load, in uA, when ulp is enabled.
 * @off_min_voltage: Minimum voltage in uV when regulator is disabled.
 * @pre_on_sleep:    Sleep, in ms, before enabling the regulator.
 * @post_on_sleep:   Sleep, in ms, after enabling the regulator.
 * @pre_off_sleep:   Sleep, in ms, before disabling the regulator.
 * @post_off_sleep:  Sleep, in ms, after disabling the regulator.
 */
struct dsi_vreg {
	struct regulator *vreg;
	char vreg_name[32];
	u32 min_voltage;
	u32 max_voltage;
	u32 enable_load;
	u32 disable_load;
	u32 ulp_load;
	u32 off_min_voltage;
	u32 pre_on_sleep;
	u32 post_on_sleep;
	u32 pre_off_sleep;
	u32 post_off_sleep;
};

/**
 * struct dsi_regulator_info - set of vregs that are turned on/off together.
 * @vregs:       Array of dsi_vreg structures.
 * @count:       Number of vregs.
 * @refcount:    Reference counting for enabling.
 */
struct dsi_regulator_info {
	struct dsi_vreg *vregs;
	u32 count;
	u32 refcount;
};

/**
 * dsi_pwr_of_get_vreg_data - parse regulator supply information
 * @of_node:        Device of node to parse for supply information.
 * @regs:           Pointer where regulator information will be copied to.
 * @supply_name:    Name of the supply node.
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_pwr_of_get_vreg_data(struct dsi_parser_utils *utils,
				 struct dsi_regulator_info *regs,
				 char *supply_name);

/**
 * dsi_pwr_get_dt_vreg_data - parse regulator supply information
 * @dev:            Device whose of_node needs to be parsed.
 * @regs:           Pointer where regulator information will be copied to.
 * @supply_name:    Name of the supply node.
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_pwr_get_dt_vreg_data(struct device *dev,
				 struct dsi_regulator_info *regs,
				 char *supply_name);

/**
 * dsi_pwr_enable_regulator() - enable a set of regulators
 * @regs:       Pointer to set of regulators to enable or disable.
 * @enable:     Enable/Disable regulators.
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_pwr_enable_regulator(struct dsi_regulator_info *regs, bool enable);

/**
 * dsi_pwr_panel_regulator_mode_set()
 * set regulator mode for OLED panel
 * @regs:       Pointer to set of regulators to enable or disable.
 * @reg_name:		Panel regulator name
 * @regulator_mode:	Regulator mode values, like:
 *                  REGULATOR_MODE_INVALID
 *                  REGULATOR_MODE_FAST
 *                  REGULATOR_MODE_NORMAL
 *                  REGULATOR_MODE_IDLE
 *                  REGULATOR_MODE_STANDBY
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_pwr_panel_regulator_mode_set(struct dsi_regulator_info *regs,
						const char *reg_name,
						int regulator_mode);

/**
 * dsi_pwr_config_vreg_opt_mode()
 * set regulator load
 * @regs:       Pointer to set of regulators to enable or disable.
 * @enable:     enable ulp_load or disable
 * return: error code in case of failure or 0 for success.
 */
int dsi_pwr_config_vreg_opt_mode(struct dsi_regulator_info *regs, bool enable);
#endif /* _DSI_PWR_H_ */
