// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <asoc/msm-cdc-supply.h>
#include <sound/soc.h>

#define CODEC_DT_MAX_PROP_SIZE 40

static int msm_cdc_dt_parse_vreg_info(struct device *dev,
				      struct cdc_regulator *cdc_vreg,
				      const char *name, bool is_ond)
{
	char prop_name[CODEC_DT_MAX_PROP_SIZE];
	struct device_node *regulator_node = NULL;
	const __be32 *prop;
	int len, rc;
	u32 prop_val;

	/* Parse supply name */
	snprintf(prop_name, CODEC_DT_MAX_PROP_SIZE, "%s-supply", name);

	regulator_node = of_parse_phandle(dev->of_node, prop_name, 0);
	if (!regulator_node) {
		dev_err(dev, "%s: Looking up %s property in node %s failed",
			__func__, prop_name, dev->of_node->full_name);
		rc = -EINVAL;
		goto done;
	}
	cdc_vreg->name = name;
	cdc_vreg->ondemand = is_ond;

	/* Parse supply - voltage */
	snprintf(prop_name, CODEC_DT_MAX_PROP_SIZE, "qcom,%s-voltage", name);
	prop = of_get_property(dev->of_node, prop_name, &len);
	if (!prop || (len != (2 * sizeof(__be32)))) {
		dev_err(dev, "%s: %s %s property\n", __func__,
			prop ? "invalid format" : "no", prop_name);
		rc = -EINVAL;
		goto done;
	} else {
		cdc_vreg->min_uV = be32_to_cpup(&prop[0]);
		cdc_vreg->max_uV = be32_to_cpup(&prop[1]);
	}

	/* Parse supply - current */
	snprintf(prop_name, CODEC_DT_MAX_PROP_SIZE, "qcom,%s-current", name);
	rc = of_property_read_u32(dev->of_node, prop_name, &prop_val);
	if (rc) {
		dev_err(dev, "%s: Looking up %s property in node %s failed",
			__func__, prop_name, dev->of_node->full_name);
		goto done;
	}
	cdc_vreg->optimum_uA = prop_val;

	/* Parse supply - LPM or NOM mode(default NOM) */
	snprintf(prop_name, CODEC_DT_MAX_PROP_SIZE, "qcom,%s-lpm-supported", name);
	rc = of_property_read_u32(dev->of_node, prop_name, &prop_val);
	if (rc) {
		dev_dbg(dev, "%s: Looking up %s property in node %s failed",
			__func__, prop_name, dev->of_node->full_name);
		cdc_vreg->lpm_supported = 0;
		rc = 0;
	} else {
		cdc_vreg->lpm_supported = prop_val;
	}

	dev_info(dev, "%s: %s: vol=[%d %d]uV, curr=[%d]uA, ond %d lpm %d\n",
		 __func__, cdc_vreg->name, cdc_vreg->min_uV, cdc_vreg->max_uV,
		 cdc_vreg->optimum_uA, cdc_vreg->ondemand,
		 cdc_vreg->lpm_supported);

done:
	return rc;
}

static int msm_cdc_parse_supplies(struct device *dev,
				  struct cdc_regulator *cdc_reg,
				  const char *sup_list, int sup_cnt,
				  bool is_ond)
{
	int idx, rc = 0;
	const char *name = NULL;

	for (idx = 0; idx < sup_cnt; idx++) {
		rc = of_property_read_string_index(dev->of_node, sup_list, idx,
						   &name);
		if (rc) {
			dev_err(dev, "%s: read string %s[%d] error (%d)\n",
				__func__, sup_list, idx, rc);
			goto done;
		}

		dev_dbg(dev, "%s: Found cdc supply %s as part of %s\n",
			__func__, name, sup_list);

		rc = msm_cdc_dt_parse_vreg_info(dev, &cdc_reg[idx], name,
						is_ond);
		if (rc) {
			dev_err(dev, "%s: parse %s vreg info failed (%d)\n",
				__func__, name, rc);
			goto done;
		}
	}

done:
	return rc;
}

static int msm_cdc_check_supply_param(struct device *dev,
				      struct cdc_regulator *cdc_vreg,
				      int num_supplies)
{
	if (!dev) {
		pr_err("%s: device is NULL\n", __func__);
		return -ENODEV;
	}

	if (!cdc_vreg || (num_supplies <= 0)) {
		dev_err(dev, "%s: supply check failed: vreg: %pK, num_supplies: %d\n",
			__func__, cdc_vreg, num_supplies);
		return -EINVAL;
	}

	return 0;
}

/*
 * msm_cdc_is_ondemand_supply:
 *	return if ondemand supply true or not
 *
 * @dev: pointer to codec device
 * @supplies: pointer to regulator bulk data
 * @cdc_vreg: pointer to platform regulator data
 * @num_supplies: number of supplies
 * @supply_name: supply name to be checked
 *
 * Return true/false
 */
bool msm_cdc_is_ondemand_supply(struct device *dev,
				struct regulator_bulk_data *supplies,
				struct cdc_regulator *cdc_vreg,
				int num_supplies,
				char *supply_name)
{
	bool rc = false;
	int ret, i;

	if ((!supply_name) || (!supplies)) {
		pr_err("%s: either dev or supplies or cdc_vreg is NULL\n",
				__func__);
		return rc;
	}
	/* input parameter validation */
	ret = msm_cdc_check_supply_param(dev, cdc_vreg, num_supplies);
	if (ret)
		return rc;

	for (i = 0; i < num_supplies; i++) {
		if (cdc_vreg[i].ondemand &&
			!strcmp(cdc_vreg[i].name, supply_name))
			return true;
	}

	return rc;
}
EXPORT_SYMBOL(msm_cdc_is_ondemand_supply);

/*
 * msm_cdc_set_supply_min_voltage:
 *	Set min supply voltage for particular supply
 *
 * @dev: pointer to codec device
 * @supplies: pointer to regulator bulk data
 * @cdc_vreg: pointer to platform regulator data
 * @num_supplies: number of supplies
 * @supply_name: Supply name to change voltage for
 * @vval_min: Min voltage to be set in uV
 * @override_min_vol: True if override min voltage from default
 * Return error code if unable to set voltage
 */
int msm_cdc_set_supply_min_voltage(struct device *dev,
				    struct regulator_bulk_data *supplies,
				    struct cdc_regulator *cdc_vreg,
				    int num_supplies, char *supply_name,
				    int vval_min, bool override_min_vol)
{
	int rc = 0, i;

	if ((!supply_name) || (!supplies)) {
		pr_err("%s: either dev or supplies or cdc_vreg is NULL\n",
				__func__);
		return -EINVAL;
	}
	/* input parameter validation */
	rc = msm_cdc_check_supply_param(dev, cdc_vreg, num_supplies);
	if (rc)
		return rc;
	for (i = 0; i < num_supplies; i++) {
		if (!strcmp(cdc_vreg[i].name, supply_name)) {
			if (override_min_vol)
				regulator_set_voltage(supplies[i].consumer,
					vval_min, cdc_vreg[i].max_uV);
			else
				regulator_set_voltage(supplies[i].consumer,
				    cdc_vreg[i].min_uV, cdc_vreg[i].max_uV);
			break;
		}
	}

	return rc;
}
EXPORT_SYMBOL(msm_cdc_set_supply_min_voltage);

/*
 * msm_cdc_disable_ondemand_supply:
 *	Disable codec ondemand supply
 *
 * @dev: pointer to codec device
 * @supplies: pointer to regulator bulk data
 * @cdc_vreg: pointer to platform regulator data
 * @num_supplies: number of supplies
 * @supply_name: Ondemand supply name to be enabled
 *
 * Return error code if supply disable is failed
 */
int msm_cdc_disable_ondemand_supply(struct device *dev,
				    struct regulator_bulk_data *supplies,
				    struct cdc_regulator *cdc_vreg,
				    int num_supplies,
				    char *supply_name)
{
	int rc, i;

	if ((!supply_name) || (!supplies)) {
		pr_err("%s: either dev or supplies or cdc_vreg is NULL\n",
				__func__);
		return -EINVAL;
	}
	/* input parameter validation */
	rc = msm_cdc_check_supply_param(dev, cdc_vreg, num_supplies);
	if (rc)
		return rc;

	for (i = 0; i < num_supplies; i++) {
		if (cdc_vreg[i].ondemand &&
			!strcmp(cdc_vreg[i].name, supply_name)) {
			rc = regulator_disable(supplies[i].consumer);
			if (rc)
				dev_err(dev, "%s: failed to disable supply %s, err:%d\n",
					__func__, supplies[i].supply, rc);
			break;
		}
	}
	if (i == num_supplies) {
		dev_err(dev, "%s: not able to find supply %s\n",
			__func__, supply_name);
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(msm_cdc_disable_ondemand_supply);

/*
 * msm_cdc_enable_ondemand_supply:
 *	Enable codec ondemand supply
 *
 * @dev: pointer to codec device
 * @supplies: pointer to regulator bulk data
 * @cdc_vreg: pointer to platform regulator data
 * @num_supplies: number of supplies
 * @supply_name: Ondemand supply name to be enabled
 *
 * Return error code if supply enable is failed
 */
int msm_cdc_enable_ondemand_supply(struct device *dev,
				   struct regulator_bulk_data *supplies,
				   struct cdc_regulator *cdc_vreg,
				   int num_supplies,
				   char *supply_name)
{
	int rc, i;

	if ((!supply_name) || (!supplies)) {
		pr_err("%s: either dev or supplies or cdc_vreg is NULL\n",
				__func__);
		return -EINVAL;
	}
	/* input parameter validation */
	rc = msm_cdc_check_supply_param(dev, cdc_vreg, num_supplies);
	if (rc)
		return rc;

	for (i = 0; i < num_supplies; i++) {
		if (cdc_vreg[i].ondemand &&
			!strcmp(cdc_vreg[i].name, supply_name)) {
			rc = regulator_enable(supplies[i].consumer);
			if (rc)
				dev_err(dev, "%s: failed to enable supply %s, rc: %d\n",
					__func__, supplies[i].supply, rc);
			break;
		}
	}
	if (i == num_supplies) {
		dev_err(dev, "%s: not able to find supply %s\n",
			__func__, supply_name);
		rc = -EINVAL;
	}

	return rc;
}
EXPORT_SYMBOL(msm_cdc_enable_ondemand_supply);

/*
 * msm_cdc_set_supplies_lpm_mode:
 *	Update load for given supply string
 *
 * @dev: pointer to codec device
 * @supplies: pointer to regulator bulk data
 * @cdc_vreg: pointer to platform regulator data
 * @num_supplies: number of supplies
 * @supply_name: supply name to be checked
 * @min_max: Apply optimum or 0 current
 *
 * Return error code if set current fail
 */
int msm_cdc_set_supplies_lpm_mode(struct device *dev,
				struct regulator_bulk_data *supplies,
				struct cdc_regulator *cdc_vreg,
				int num_supplies,
				bool flag)
{
	int rc = 0, i;

	if (!supplies) {
		pr_err("%s: supplies is NULL\n",
				__func__);
		return -EINVAL;
	}
	/* input parameter validation */
	rc = msm_cdc_check_supply_param(dev, cdc_vreg, num_supplies);
	if (rc)
		return rc;

	for (i = 0; i < num_supplies; i++) {
		if (cdc_vreg[i].lpm_supported) {
			rc = regulator_set_load(
				supplies[i].consumer,
				flag ? 0 : cdc_vreg[i].optimum_uA);
			if (rc)
				dev_err(dev,
					"%s: failed to set supply %s to %s, err:%d\n",
					__func__, supplies[i].supply,
					flag ? "LPM" : "NOM",
					rc);
			else
				dev_dbg(dev, "%s: regulator %s load set to %s\n",
					__func__, supplies[i].supply,
					flag ? "LPM" : "NOM");
		}
	}

	return rc;
}
EXPORT_SYMBOL(msm_cdc_set_supplies_lpm_mode);

/*
 * msm_cdc_disable_static_supplies:
 *	Disable codec static supplies
 *
 * @dev: pointer to codec device
 * @supplies: pointer to regulator bulk data
 * @cdc_vreg: pointer to platform regulator data
 * @num_supplies: number of supplies
 *
 * Return error code if supply disable is failed
 */
int msm_cdc_disable_static_supplies(struct device *dev,
				    struct regulator_bulk_data *supplies,
				    struct cdc_regulator *cdc_vreg,
				    int num_supplies)
{
	int rc, i;

	if ((!dev) || (!supplies) || (!cdc_vreg)) {
		pr_err("%s: either dev or supplies or cdc_vreg is NULL\n",
				__func__);
		return -EINVAL;
	}
	/* input parameter validation */
	rc = msm_cdc_check_supply_param(dev, cdc_vreg, num_supplies);
	if (rc)
		return rc;

	for (i = 0; i < num_supplies; i++) {
		if (cdc_vreg[i].ondemand)
			continue;

		rc = regulator_disable(supplies[i].consumer);
		if (rc)
			dev_err(dev, "%s: failed to disable supply %s, err:%d\n",
				__func__, supplies[i].supply, rc);
		else
			dev_dbg(dev, "%s: disabled regulator %s\n",
				__func__, supplies[i].supply);
	}

	return rc;
}
EXPORT_SYMBOL(msm_cdc_disable_static_supplies);

/*
 * msm_cdc_release_supplies:
 *	Release codec power supplies
 *
 * @dev: pointer to codec device
 * @supplies: pointer to regulator bulk data
 * @cdc_vreg: pointer to platform regulator data
 * @num_supplies: number of supplies
 *
 * Return error code if supply disable is failed
 */
int msm_cdc_release_supplies(struct device *dev,
			     struct regulator_bulk_data *supplies,
			     struct cdc_regulator *cdc_vreg,
			     int num_supplies)
{
	int rc = 0;
	int i;

	if ((!dev) || (!supplies) || (!cdc_vreg)) {
		pr_err("%s: either dev or supplies or cdc_vreg is NULL\n",
				__func__);
		return -EINVAL;
	}
	/* input parameter validation */
	rc = msm_cdc_check_supply_param(dev, cdc_vreg, num_supplies);
	if (rc)
		return rc;

	msm_cdc_disable_static_supplies(dev, supplies, cdc_vreg,
					num_supplies);
	for (i = 0; i < num_supplies; i++) {
		if (regulator_count_voltages(supplies[i].consumer) < 0)
			continue;

		regulator_set_voltage(supplies[i].consumer, 0,
				      cdc_vreg[i].max_uV);
		regulator_set_load(supplies[i].consumer, 0);
	}

	return rc;
}
EXPORT_SYMBOL(msm_cdc_release_supplies);

/*
 * msm_cdc_enable_static_supplies:
 *	Enable codec static supplies
 *
 * @dev: pointer to codec device
 * @supplies: pointer to regulator bulk data
 * @cdc_vreg: pointer to platform regulator data
 * @num_supplies: number of supplies
 *
 * Return error code if supply enable is failed
 */
int msm_cdc_enable_static_supplies(struct device *dev,
				   struct regulator_bulk_data *supplies,
				   struct cdc_regulator *cdc_vreg,
				   int num_supplies)
{
	int rc, i;

	if ((!dev) || (!supplies) || (!cdc_vreg)) {
		pr_err("%s: either dev or supplies or cdc_vreg is NULL\n",
				__func__);
		return -EINVAL;
	}
	/* input parameter validation */
	rc = msm_cdc_check_supply_param(dev, cdc_vreg, num_supplies);
	if (rc)
		return rc;

	for (i = 0; i < num_supplies; i++) {
		if (cdc_vreg[i].ondemand)
			continue;

		rc = regulator_enable(supplies[i].consumer);
		if (rc) {
			dev_err(dev, "%s: failed to enable supply %s, rc: %d\n",
				__func__, supplies[i].supply, rc);
			break;
		}
	}

	while (rc && i--)
		if (!cdc_vreg[i].ondemand)
			regulator_disable(supplies[i].consumer);

	return rc;
}
EXPORT_SYMBOL(msm_cdc_enable_static_supplies);

/*
 * msm_cdc_init_supplies:
 *	Initialize codec static supplies
 *
 * @dev: pointer to codec device
 * @supplies: pointer to regulator bulk data
 * @cdc_vreg: pointer to platform regulator data
 * @num_supplies: number of supplies
 *
 * Return error code if supply init is failed
 */
int msm_cdc_init_supplies(struct device *dev,
			  struct regulator_bulk_data **supplies,
			  struct cdc_regulator *cdc_vreg,
			  int num_supplies)
{
	return msm_cdc_init_supplies_v2(dev, supplies, cdc_vreg,
					num_supplies, false);
}
EXPORT_SYMBOL(msm_cdc_init_supplies);

/*
 * msm_cdc_init_supplies_v2:
 *	Initialize codec static supplies.
 *	Initialize codec dynamic supplies based on vote_regulator_on_demand
 *
 * @dev: pointer to codec device
 * @supplies: pointer to regulator bulk data
 * @cdc_vreg: pointer to platform regulator data
 * @num_supplies: number of supplies
 * @vote_regulator_on_demand: initialize codec dynamic supplies at runtime
 *
 * Return error code if supply init is failed
 */
int msm_cdc_init_supplies_v2(struct device *dev,
			  struct regulator_bulk_data **supplies,
			  struct cdc_regulator *cdc_vreg,
			  int num_supplies, u32 vote_regulator_on_demand)
{
	struct regulator_bulk_data *vsup;
	int rc;
	int i;

	if (!dev || !cdc_vreg) {
		pr_err("%s: device pointer or dce_vreg is NULL\n",
				__func__);
		return -EINVAL;
	}
	/* input parameter validation */
	rc = msm_cdc_check_supply_param(dev, cdc_vreg, num_supplies);
	if (rc)
		return rc;

	vsup = devm_kcalloc(dev, num_supplies,
			    sizeof(struct regulator_bulk_data),
			    GFP_KERNEL);
	if (!vsup)
		return -ENOMEM;

	for (i = 0; i < num_supplies; i++) {
		if (!cdc_vreg[i].name) {
			dev_err(dev, "%s: supply name not defined\n",
				__func__);
			rc = -EINVAL;
			goto err_supply;
		}
		vsup[i].supply = cdc_vreg[i].name;
	}

	rc = devm_regulator_bulk_get(dev, num_supplies, vsup);
	if (rc) {
		dev_err(dev, "%s: failed to get supplies (%d)\n",
			__func__, rc);
		goto err_supply;
	}

	/* Set voltage and current on regulators */
	for (i = 0; i < num_supplies; i++) {
		if (regulator_count_voltages(vsup[i].consumer) < 0)
			continue;

		if (cdc_vreg[i].ondemand && vote_regulator_on_demand)
			continue;

		rc = regulator_set_voltage(vsup[i].consumer,
					   cdc_vreg[i].min_uV,
					   cdc_vreg[i].max_uV);
		if (rc) {
			dev_err(dev, "%s: set regulator voltage failed for %s, err:%d\n",
				__func__, vsup[i].supply, rc);
			goto err_supply;
		}
		rc = regulator_set_load(vsup[i].consumer,
					cdc_vreg[i].optimum_uA);
		if (rc < 0) {
			dev_err(dev, "%s: set regulator optimum mode failed for %s, err:%d\n",
				__func__, vsup[i].supply, rc);
			goto err_supply;
		}
	}

	*supplies = vsup;

	return 0;

err_supply:
	return rc;
}
EXPORT_SYMBOL(msm_cdc_init_supplies_v2);

/*
 * msm_cdc_get_power_supplies:
 *	Get codec power supplies from device tree.
 *	Allocate memory to hold regulator data for
 *	all power supplies.
 *
 * @dev: pointer to codec device
 * @cdc_vreg: pointer to codec regulator
 * @total_num_supplies: total number of supplies read from DT
 *
 * Return error code if supply disable is failed
 */
int msm_cdc_get_power_supplies(struct device *dev,
			       struct cdc_regulator **cdc_vreg,
			       int *total_num_supplies)
{
	const char *static_prop_name = "qcom,cdc-static-supplies";
	const char *ond_prop_name = "qcom,cdc-on-demand-supplies";
	const char *cp_prop_name = "qcom,cdc-cp-supplies";
	int static_sup_cnt = 0;
	int ond_sup_cnt = 0;
	int cp_sup_cnt = 0;
	int num_supplies = 0;
	struct cdc_regulator *cdc_reg;
	int rc;

	if (!dev) {
		pr_err("%s: device pointer is NULL\n", __func__);
		return -EINVAL;
	}
	static_sup_cnt = of_property_count_strings(dev->of_node,
						   static_prop_name);
	if (static_sup_cnt < 0) {
		dev_err(dev, "%s: Failed to get static supplies(%d)\n",
			__func__, static_sup_cnt);
		rc = static_sup_cnt;
		goto err_supply_cnt;
	}
	ond_sup_cnt = of_property_count_strings(dev->of_node, ond_prop_name);
	if (ond_sup_cnt < 0)
		ond_sup_cnt = 0;

	cp_sup_cnt = of_property_count_strings(dev->of_node,
					       cp_prop_name);
	if (cp_sup_cnt < 0)
		cp_sup_cnt = 0;

	num_supplies = static_sup_cnt + ond_sup_cnt + cp_sup_cnt;
	if (num_supplies <= 0) {
		dev_err(dev, "%s: supply count is 0 or negative\n", __func__);
		rc = -EINVAL;
		goto err_supply_cnt;
	}

	cdc_reg = devm_kcalloc(dev, num_supplies,
			       sizeof(struct cdc_regulator),
			       GFP_KERNEL);
	if (!cdc_reg) {
		rc = -ENOMEM;
		goto err_mem_alloc;
	}

	rc = msm_cdc_parse_supplies(dev, cdc_reg, static_prop_name,
				    static_sup_cnt, false);
	if (rc) {
		dev_err(dev, "%s: failed to parse static supplies(%d)\n",
				__func__, rc);
		goto err_sup;
	}

	rc = msm_cdc_parse_supplies(dev, &cdc_reg[static_sup_cnt],
				    ond_prop_name, ond_sup_cnt,
				    true);
	if (rc) {
		dev_err(dev, "%s: failed to parse demand supplies(%d)\n",
				__func__, rc);
		goto err_sup;
	}

	rc = msm_cdc_parse_supplies(dev,
				    &cdc_reg[static_sup_cnt + ond_sup_cnt],
				    cp_prop_name, cp_sup_cnt, true);
	if (rc) {
		dev_err(dev, "%s: failed to parse cp supplies(%d)\n",
				__func__, rc);
		goto err_sup;
	}

	*cdc_vreg = cdc_reg;
	*total_num_supplies = num_supplies;

	return 0;

err_sup:
err_supply_cnt:
err_mem_alloc:
	return rc;
}
EXPORT_SYMBOL(msm_cdc_get_power_supplies);

/*
 * msm_cdc_init_wcd_supply:
 *	Initialize wcd supply parameters.
 *
 * @np: device node pointer to codec device
 * @name: power supply name
 * @cdc_supply: codec supply struct to hold wcd params
 *
 * Return error code if init failed
 */
int msm_cdc_init_wcd_supply(struct device_node *np, const char *name,
			    struct cdc_wcd_supply *cdc_supply)
{
	struct platform_device *pdev = NULL;

	if (!np || !cdc_supply)
		return -EINVAL;

	pdev = of_find_device_by_node(np);
	if (!pdev)
		return -EINVAL;

	cdc_supply->dev = &pdev->dev;
	cdc_supply->name = name;
	cdc_supply->component = snd_soc_lookup_component(&pdev->dev, NULL);

	return 0;
}
EXPORT_SYMBOL(msm_cdc_init_wcd_supply);

/*
 * msm_cdc_enable_wcd_supply:
 *	Enable/Disable wcd supply.
 *
 * @cdc_supply: codec supply struct to hold wcd params
 * @enable: bool to inform whether to enable or disable
 *
 * Return error code if enable/disable failed
 */
int msm_cdc_enable_wcd_supply(struct cdc_wcd_supply *cdc_supply, bool enable)
{
	struct snd_soc_component *component = cdc_supply->component;
	int rc;

	if (!component) {
		pr_err("%s: Component memory is NULL\n", __func__);
		return -EINVAL;
	}

	if (enable)
		rc = snd_soc_dapm_force_enable_pin(
					snd_soc_component_get_dapm(component),
					cdc_supply->name);
	else
		rc = snd_soc_dapm_disable_pin(
					snd_soc_component_get_dapm(component),
					cdc_supply->name);

	if (!rc)
		snd_soc_dapm_sync(snd_soc_component_get_dapm(component));
	else
		dev_err(component->dev, "%s: micbias %s force %s pin failed\n",
			__func__, cdc_supply->name, (enable ? "enable" : "disable"));

	return rc;
}
EXPORT_SYMBOL(msm_cdc_enable_wcd_supply);
