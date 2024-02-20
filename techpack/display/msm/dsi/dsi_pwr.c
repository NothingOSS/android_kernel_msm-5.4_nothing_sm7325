// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2020,2021 The Linux Foundation. All rights reserved.
 */

#include <linux/of.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "dsi_pwr.h"
#include "dsi_parser.h"
#include "dsi_defs.h"

/*
 * dsi_pwr_parse_supply_node() - parse power supply node from root device node
 */
static int dsi_pwr_parse_supply_node(struct dsi_parser_utils *utils,
				     struct device_node *root,
				     struct dsi_regulator_info *regs)
{
	int rc = 0;
	int i = 0;
	u32 tmp = 0;
	struct device_node *node = NULL;

	dsi_for_each_child_node(root, node) {
		const char *st = NULL;

		rc = utils->read_string(node, "qcom,supply-name", &st);
		if (rc) {
			DSI_ERR("failed to read name, rc = %d\n", rc);
			goto error;
		}

		snprintf(regs->vregs[i].vreg_name,
			 ARRAY_SIZE(regs->vregs[i].vreg_name),
			 "%s", st);

		rc = utils->read_u32(node, "qcom,supply-min-voltage", &tmp);
		if (rc) {
			DSI_ERR("failed to read min voltage, rc = %d\n", rc);
			goto error;
		}
		regs->vregs[i].min_voltage = tmp;

		rc = utils->read_u32(node, "qcom,supply-max-voltage", &tmp);
		if (rc) {
			DSI_ERR("failed to read max voltage, rc = %d\n", rc);
			goto error;
		}
		regs->vregs[i].max_voltage = tmp;

		rc = utils->read_u32(node, "qcom,supply-enable-load", &tmp);
		if (rc) {
			DSI_ERR("failed to read enable load, rc = %d\n", rc);
			goto error;
		}
		regs->vregs[i].enable_load = tmp;

		rc = utils->read_u32(node, "qcom,supply-disable-load", &tmp);
		if (rc) {
			DSI_ERR("failed to read disable load, rc = %d\n", rc);
			goto error;
		}
		regs->vregs[i].disable_load = tmp;

		/* Optional values */
		rc = utils->read_u32(node, "qcom,supply-ulp-load", &tmp);
		if (rc) {
			DSI_DEBUG("ulp-load not specified\n");
		}
		regs->vregs[i].ulp_load = (!rc ? tmp :
			regs->vregs[i].enable_load);

		rc = utils->read_u32(node, "qcom,supply-off-min-voltage", &tmp);
		if (rc) {
			DSI_DEBUG("off-min-voltage not specified\n");
			rc = 0;
		} else {
			regs->vregs[i].off_min_voltage = tmp;
		}

		rc = utils->read_u32(node, "qcom,supply-pre-on-sleep", &tmp);
		if (rc) {
			DSI_DEBUG("pre-on-sleep not specified\n");
			rc = 0;
		} else {
			regs->vregs[i].pre_on_sleep = tmp;
		}

		rc = utils->read_u32(node, "qcom,supply-pre-off-sleep", &tmp);
		if (rc) {
			DSI_DEBUG("pre-off-sleep not specified\n");
			rc = 0;
		} else {
			regs->vregs[i].pre_off_sleep = tmp;
		}

		rc = utils->read_u32(node, "qcom,supply-post-on-sleep", &tmp);
		if (rc) {
			DSI_DEBUG("post-on-sleep not specified\n");
			rc = 0;
		} else {
			regs->vregs[i].post_on_sleep = tmp;
		}

		rc = utils->read_u32(node, "qcom,supply-post-off-sleep", &tmp);
		if (rc) {
			DSI_DEBUG("post-off-sleep not specified\n");
			rc = 0;
		} else {
			regs->vregs[i].post_off_sleep = tmp;
		}

		DSI_DEBUG("[%s] minv=%d maxv=%d, en_load=%d, dis_load=%d\n",
			 regs->vregs[i].vreg_name,
			 regs->vregs[i].min_voltage,
			 regs->vregs[i].max_voltage,
			 regs->vregs[i].enable_load,
			 regs->vregs[i].disable_load);
		++i;
	}

error:
	return rc;
}

int dsi_pwr_config_vreg_opt_mode(struct dsi_regulator_info *regs,
				bool enable)
{
	int i = 0, rc = 0;
	struct dsi_vreg *vreg;
	u32 mode;

	for (i = 0; i < regs->count; i++) {
		vreg = &regs->vregs[i];
		mode = enable ? vreg->ulp_load : vreg->enable_load;

		DSI_DEBUG(" Setting optimum mode for %s load = %d\n",
				vreg->vreg_name, mode);

		rc = regulator_set_load(vreg->vreg, mode);
		if (rc < 0) {
			DSI_ERR("Set opt mode failed for %s",
				vreg->vreg_name);
			return rc;
		}
	}
	return rc;
}

/**
 * dsi_pwr_enable_vregs() - enable/disable regulators
 */
static int dsi_pwr_enable_vregs(struct dsi_regulator_info *regs, bool enable)
{
	int rc = 0, i = 0;
	struct dsi_vreg *vreg;
	int num_of_v = 0;
	u32 pre_on_ms, post_on_ms;
	u32 pre_off_ms, post_off_ms;

	if (enable) {
		for (i = 0; i < regs->count; i++) {
			vreg = &regs->vregs[i];
			pre_on_ms = vreg->pre_on_sleep;
			post_on_ms = vreg->post_on_sleep;

			if (vreg->pre_on_sleep)
				usleep_range((pre_on_ms * 1000),
						(pre_on_ms * 1000) + 10);

			rc = regulator_set_load(vreg->vreg,
						vreg->enable_load);
			if (rc < 0) {
				DSI_ERR("Setting optimum mode failed for %s\n",
				       vreg->vreg_name);
				goto error;
			}
			num_of_v = regulator_count_voltages(vreg->vreg);
			if (num_of_v > 0) {
				rc = regulator_set_voltage(vreg->vreg,
							   vreg->min_voltage,
							   vreg->max_voltage);
				if (rc) {
					DSI_ERR("Set voltage(%s) fail, rc=%d\n",
						 vreg->vreg_name, rc);
					goto error_disable_opt_mode;
				}
			}

			rc = regulator_enable(vreg->vreg);
			if (rc) {
				DSI_ERR("enable failed for %s, rc=%d\n",
				       vreg->vreg_name, rc);
				goto error_disable_voltage;
			}

			if (vreg->post_on_sleep)
				usleep_range((post_on_ms * 1000),
						(post_on_ms * 1000) + 10);
		}
	} else {
		for (i = (regs->count - 1); i >= 0; i--) {
			vreg = &regs->vregs[i];
			pre_off_ms = vreg->pre_off_sleep;
			post_off_ms = vreg->post_off_sleep;

			if (pre_off_ms)
				usleep_range((pre_off_ms * 1000),
						(pre_off_ms * 1000) + 10);

			(void)regulator_disable(regs->vregs[i].vreg);

			if (post_off_ms)
				usleep_range((post_off_ms * 1000),
						(post_off_ms * 1000) + 10);

			(void)regulator_set_load(regs->vregs[i].vreg,
						regs->vregs[i].disable_load);

			num_of_v = regulator_count_voltages(vreg->vreg);
			if (num_of_v > 0)
				(void)regulator_set_voltage(regs->vregs[i].vreg,
						regs->vregs[i].off_min_voltage,
						regs->vregs[i].max_voltage);

		}
	}

	return 0;
error_disable_opt_mode:
	(void)regulator_set_load(regs->vregs[i].vreg,
				 regs->vregs[i].disable_load);

error_disable_voltage:
	if (num_of_v > 0)
		(void)regulator_set_voltage(regs->vregs[i].vreg,
					    0, regs->vregs[i].max_voltage);
error:
	for (i--; i >= 0; i--) {
		vreg = &regs->vregs[i];
		pre_off_ms = vreg->pre_off_sleep;
		post_off_ms = vreg->post_off_sleep;

		if (pre_off_ms)
			usleep_range((pre_off_ms * 1000),
					(pre_off_ms * 1000) + 10);

		(void)regulator_disable(regs->vregs[i].vreg);

		if (post_off_ms)
			usleep_range((post_off_ms * 1000),
					(post_off_ms * 1000) + 10);

		(void)regulator_set_load(regs->vregs[i].vreg,
					 regs->vregs[i].disable_load);

		num_of_v = regulator_count_voltages(regs->vregs[i].vreg);
		if (num_of_v > 0)
			(void)regulator_set_voltage(regs->vregs[i].vreg,
				0, regs->vregs[i].max_voltage);

	}

	return rc;
}

/**
 * dsi_pwr_of_get_vreg_data - Parse regulator supply information
 * @of_node:        Device of node to parse for supply information.
 * @regs:           Pointer where regulator information will be copied to.
 * @supply_name:    Name of the supply node.
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_pwr_of_get_vreg_data(struct dsi_parser_utils *utils,
				 struct dsi_regulator_info *regs,
				 char *supply_name)
{
	int rc = 0;
	struct device_node *supply_root_node = NULL;

	if (!utils || !regs) {
		DSI_ERR("Bad params\n");
		return -EINVAL;
	}

	regs->count = 0;
	supply_root_node = utils->get_child_by_name(utils->data, supply_name);
	if (!supply_root_node) {
		supply_root_node = of_parse_phandle(utils->node,
					supply_name, 0);
		if (!supply_root_node) {
			DSI_DEBUG("No supply entry present for %s\n",
					supply_name);
			return -EINVAL;
		}
	}

	regs->count = utils->get_available_child_count(supply_root_node);
	if (regs->count == 0) {
		DSI_ERR("No vregs defined for %s\n", supply_name);
		return -EINVAL;
	}

	regs->vregs = kcalloc(regs->count, sizeof(*regs->vregs), GFP_KERNEL);
	if (!regs->vregs) {
		regs->count = 0;
		return -ENOMEM;
	}

	rc = dsi_pwr_parse_supply_node(utils, supply_root_node, regs);
	if (rc) {
		DSI_ERR("failed to parse supply node for %s, rc = %d\n",
			supply_name, rc);

		kfree(regs->vregs);
		regs->vregs = NULL;
		regs->count = 0;
	}

	return rc;
}

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
				 char *supply_name)
{
	int rc = 0;
	struct device_node *of_node = NULL;
	struct device_node *supply_node = NULL;
	struct device_node *supply_root_node = NULL;
	struct dsi_parser_utils utils = *dsi_parser_get_of_utils();

	if (!dev || !regs) {
		DSI_ERR("Bad params\n");
		return -EINVAL;
	}

	of_node = dev->of_node;
	regs->count = 0;
	supply_root_node = of_get_child_by_name(of_node, supply_name);
	if (!supply_root_node) {
		supply_root_node = of_parse_phandle(of_node, supply_name, 0);
		if (!supply_root_node) {
			DSI_DEBUG("No supply entry present for %s\n",
					supply_name);
			return -EINVAL;
		}
	}

	for_each_child_of_node(supply_root_node, supply_node)
		regs->count++;

	if (regs->count == 0) {
		DSI_ERR("No vregs defined for %s\n", supply_name);
		return -EINVAL;
	}

	regs->vregs = devm_kcalloc(dev, regs->count, sizeof(*regs->vregs),
				   GFP_KERNEL);
	if (!regs->vregs) {
		regs->count = 0;
		return -ENOMEM;
	}

	utils.data = of_node;
	utils.node = of_node;

	rc = dsi_pwr_parse_supply_node(&utils, supply_root_node, regs);
	if (rc) {
		DSI_ERR("failed to parse supply node for %s, rc = %d\n",
		       supply_name, rc);
		devm_kfree(dev, regs->vregs);
		regs->vregs = NULL;
		regs->count = 0;
	}

	return rc;
}

/**
 * dsi_pwr_enable_regulator() - enable a set of regulators
 * @regs:       Pointer to set of regulators to enable or disable.
 * @enable:     Enable/Disable regulators.
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_pwr_enable_regulator(struct dsi_regulator_info *regs, bool enable)
{
	int rc = 0;

	if (regs->count == 0) {
		DSI_DEBUG("No valid regulators to enable\n");
		return 0;
	}

	if (!regs->vregs) {
		DSI_ERR("Invalid params\n");
		return -EINVAL;
	}

	if (enable) {
		if (regs->refcount == 0) {
			rc = dsi_pwr_enable_vregs(regs, true);
			if (rc)
				DSI_ERR("failed to enable regulators\n");
		}
		regs->refcount++;
	} else {
		if (regs->refcount == 0) {
			DSI_ERR("Unbalanced regulator off:%s\n",
					regs->vregs->vreg_name);
		} else {
			regs->refcount--;
			if (regs->refcount == 0) {
				rc = dsi_pwr_enable_vregs(regs, false);
				if (rc)
					DSI_ERR("failed to disable vregs\n");
			}
		}
	}

	return rc;
}

/*
 * dsi_pwr_panel_regulator_mode_set()
 * set the AB/IBB regulator mode for OLED panel
 * AOD mode entry and exit
 * @regs:	Pointer to set of regulators to enable or disable.
 * @reg_name:	Name of panel power we want to set.
 * @retulator_mode:	Regulator mode values, like:
 *	REGULATOR_MODE_INVALID
 *	REGULATOR_MODE_FAST
 *	REGULATOR_MODE_NORMAL
 *	REGULATOR_MODE_IDLE
 *	REGULATOR_MODE_STANDBY
 *
 * return: error code in case of failure or 0 for success.
 */
int dsi_pwr_panel_regulator_mode_set(struct dsi_regulator_info *regs,
						const char *reg_name,
						int regulator_mode)
{
	int i = 0, rc = 0;
	struct dsi_vreg *vreg;

	if (regs->count == 0)
		return -EINVAL;

	if (!regs->vregs)
		return -EINVAL;

	for (i = 0; i < regs->count; i++) {
		vreg = &regs->vregs[i];
		if (!strcmp(vreg->vreg_name, reg_name)) {
			rc = regulator_set_mode(vreg->vreg,
							regulator_mode);
			if (rc)
				DSI_ERR("Regulator %s set mode %d failed\n",
					vreg->vreg_name, rc);
			break;
		}
	}

	if (i >= regs->count) {
		DSI_ERR("Regulator %s was not found\n", reg_name);
		return -EINVAL;
	}

	return rc;
}
