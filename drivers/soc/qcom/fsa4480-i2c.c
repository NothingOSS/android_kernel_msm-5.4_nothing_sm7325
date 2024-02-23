// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/usb/typec.h>
#include <linux/usb/ucsi_glink.h>
#include <linux/soc/qcom/fsa4480-i2c.h>
#include <linux/iio/consumer.h>

#define FSA4480_I2C_NAME	"fsa4480-driver"

#define FSA4480_SWITCH_SETTINGS 0x04
#define FSA4480_SWITCH_CONTROL  0x05
#define FSA4480_SWITCH_STATUS   0X06
#define FSA4480_SWITCH_STATUS1  0x07
#define FSA4480_SLOW_L          0x08
#define FSA4480_SLOW_R          0x09
#define FSA4480_SLOW_MIC        0x0A
#define FSA4480_SLOW_SENSE      0x0B
#define FSA4480_SLOW_GND        0x0C
#define FSA4480_DELAY_L_R       0x0D
#define FSA4480_DELAY_L_MIC     0x0E
#define FSA4480_DELAY_L_SENSE   0x0F
#define FSA4480_DELAY_L_AGND    0x10
#define FSA4480_FUNCTION_ENABLE 0X12
#define FSA4480_JACK_STATUS     0X17
#define FSA4480_JACK_FLAG       0X18
#define FSA4480_RESET           0x1E

struct fsa4480_priv {
	struct regmap *regmap;
	struct device *dev;
	struct power_supply *usb_psy;
	struct notifier_block nb;
	struct iio_channel *iio_ch;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct blocking_notifier_head fsa4480_notifier;
	struct mutex notification_lock;
	u32 use_powersupply;
	int switch_control;
};

struct fsa4480_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config fsa4480_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FSA4480_RESET,
};

static const struct fsa4480_reg_val fsa_reg_i2c_defaults[] = {
	{FSA4480_SLOW_L, 0x00},
	{FSA4480_SLOW_R, 0x00},
	{FSA4480_SLOW_MIC, 0x00},
	{FSA4480_SLOW_SENSE, 0x00},
	{FSA4480_SLOW_GND, 0x00},
	{FSA4480_DELAY_L_R, 0x00},
	{FSA4480_DELAY_L_MIC, 0x00},
	{FSA4480_DELAY_L_SENSE, 0x00},
	{FSA4480_DELAY_L_AGND, 0x09},
	{FSA4480_SWITCH_SETTINGS, 0x98},
};

static void fsa4480_usbc_update_settings(struct fsa4480_priv *fsa_priv,
		u32 switch_control, u32 switch_enable)
{
	u32 prev_control, prev_enable;

	if (!fsa_priv->regmap) {
		dev_err(fsa_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, &prev_control);
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, &prev_enable);

	if (prev_control == switch_control && prev_enable == switch_enable) {
		dev_dbg(fsa_priv->dev, "%s: settings unchanged\n", __func__);
		return;
	}

	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x80);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, switch_control);
	/* FSA4480 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, switch_enable);
}

static int fsa4480_usbc_event_changed_psupply(struct fsa4480_priv *fsa_priv,
				      unsigned long evt, void *ptr)
{
	struct device *dev = NULL;

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;
	dev_dbg(dev, "%s: queueing usbc_analog_work\n",
		__func__);
	pm_stay_awake(fsa_priv->dev);
	queue_work(system_freezable_wq, &fsa_priv->usbc_analog_work);

	return 0;
}

static int fsa4480_usbc_event_changed_ucsi(struct fsa4480_priv *fsa_priv,
				      unsigned long evt, void *ptr)
{
	struct device *dev;
	enum typec_accessory acc = ((struct ucsi_glink_constat_info *)ptr)->acc;

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	dev_dbg(dev, "%s: USB change event received, supply mode %d, usbc mode %ld, expected %d\n",
			__func__, acc, fsa_priv->usbc_mode.counter,
			TYPEC_ACCESSORY_AUDIO);

	switch (acc) {
	case TYPEC_ACCESSORY_AUDIO:
	case TYPEC_ACCESSORY_NONE:
		if (atomic_read(&(fsa_priv->usbc_mode)) == acc)
			break; /* filter notifications received before */
		atomic_set(&(fsa_priv->usbc_mode), acc);

		dev_dbg(dev, "%s: queueing usbc_analog_work\n",
			__func__);
		pm_stay_awake(fsa_priv->dev);
		queue_work(system_freezable_wq, &fsa_priv->usbc_analog_work);
		break;
	default:
		break;
	}

	return 0;
}

static int fsa4480_usbc_event_changed(struct notifier_block *nb_ptr,
				      unsigned long evt, void *ptr)
{
	struct fsa4480_priv *fsa_priv =
			container_of(nb_ptr, struct fsa4480_priv, nb);
	struct device *dev;

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	if (fsa_priv->use_powersupply)
		return fsa4480_usbc_event_changed_psupply(fsa_priv, evt, ptr);
	else
		return fsa4480_usbc_event_changed_ucsi(fsa_priv, evt, ptr);
}

static int fsa4480_usbc_analog_setup_switches_psupply(
						struct fsa4480_priv *fsa_priv)
{
	int rc = 0;
	union power_supply_propval mode;
	struct device *dev;

	if (!fsa_priv)
		return -EINVAL;
	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	rc = iio_read_channel_processed(fsa_priv->iio_ch, &mode.intval);

	mutex_lock(&fsa_priv->notification_lock);
	/* get latest mode again within locked context */
	if (rc < 0) {
		dev_err(dev, "%s: Unable to read USB TYPEC_MODE: %d\n",
			__func__, rc);
		goto done;
	}

	dev_dbg(dev, "%s: setting GPIOs active = %d rcvd intval 0x%X\n",
		__func__, mode.intval != TYPEC_ACCESSORY_NONE, mode.intval);
	atomic_set(&(fsa_priv->usbc_mode), mode.intval);

	switch (mode.intval) {
	/* add all modes FSA should notify for in here */
	case TYPEC_ACCESSORY_AUDIO:
		/* activate switches */
		fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);

		/* notify call chain on event */
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
		mode.intval, NULL);
		break;
	case TYPEC_ACCESSORY_NONE:
		/* notify call chain on event */
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
				TYPEC_ACCESSORY_NONE, NULL);

		/* deactivate switches */
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		break;
	default:
		/* ignore other usb connection modes */
		break;
	}

done:
	mutex_unlock(&fsa_priv->notification_lock);
	return rc;
}

static int fsa4480_usbc_analog_setup_switches_ucsi(
						struct fsa4480_priv *fsa_priv)
{
	int rc = 0;
	int mode;
	int value = 0;
	struct device *dev;

	if (!fsa_priv)
		return -EINVAL;
	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	mutex_lock(&fsa_priv->notification_lock);
	/* get latest mode again within locked context */
	mode = atomic_read(&(fsa_priv->usbc_mode));

	dev_dbg(dev, "%s: setting GPIOs active = %d\n",
		__func__, mode != TYPEC_ACCESSORY_NONE);
	regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);
	msleep(1);

	switch (mode) {
	/* add all modes FSA should notify for in here */
	case TYPEC_ACCESSORY_AUDIO:
		/* activate switches */
		regmap_read(fsa_priv->regmap, 0x00, &value);
		dev_dbg(fsa_priv->dev, "%s: reg[0x00]=0x%x\n",__func__, value);
		if(value == 241)
		{
			dev_dbg(fsa_priv->dev, "%s:DIO headset detection .\n",__func__);
			regmap_write(fsa_priv->regmap, FSA4480_SLOW_L, 0x4f);
			regmap_read(fsa_priv->regmap, FSA4480_SLOW_L, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x08]=0x%x\n",__func__, value);
			regmap_write(fsa_priv->regmap, FSA4480_SLOW_R, 0x4f);
			regmap_read(fsa_priv->regmap, FSA4480_SLOW_R, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x09]=0x%x\n",__func__, value);
			regmap_write(fsa_priv->regmap, FSA4480_SLOW_MIC, 0x4f);
			regmap_read(fsa_priv->regmap, FSA4480_SLOW_MIC, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x0a]=0x%x\n",__func__, value);
			regmap_write(fsa_priv->regmap, FSA4480_SLOW_SENSE, 0x4f);
			regmap_read(fsa_priv->regmap, FSA4480_SLOW_SENSE, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x0b]=0x%x\n",__func__, value);
			regmap_write(fsa_priv->regmap, FSA4480_SLOW_GND, 0x4f);
			regmap_read(fsa_priv->regmap, FSA4480_SLOW_GND, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x0c]=0x%x\n",__func__, value);
			regmap_write(fsa_priv->regmap, FSA4480_FUNCTION_ENABLE, 0x09);
			dev_dbg(fsa_priv->dev, "%s: set reg[0x12] done.\n",__func__);
			regmap_read(fsa_priv->regmap, FSA4480_FUNCTION_ENABLE, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x12]=0x%x\n",__func__, value);
			msleep(1000);
			regmap_read(fsa_priv->regmap, FSA4480_JACK_FLAG, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x18]=0x%x\n",__func__, value);
			msleep(1);
			regmap_read(fsa_priv->regmap, FSA4480_JACK_STATUS, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x17]=0x%x\n",__func__, value);
			regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x6]=0x%x\n",__func__, value);
			regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x7]=0x%x\n",__func__, value);
		} else {
			dev_dbg(fsa_priv->dev, "%s:FSA headset detection.\n",__func__);
			fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
			regmap_write(fsa_priv->regmap, FSA4480_FUNCTION_ENABLE, 0x01);
			dev_dbg(fsa_priv->dev, "%s: set reg[0x12] done.\n",__func__);
			regmap_read(fsa_priv->regmap, FSA4480_JACK_STATUS, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x17]=0x%x\n",__func__, value);
			regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x6]=0x%x\n",__func__, value);
			regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &value);
			dev_dbg(fsa_priv->dev, "%s: reg[0x7]=0x%x\n",__func__, value);
		}
		/* notify call chain on event */
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
					     mode, NULL);
		break;
	case TYPEC_ACCESSORY_NONE:
		/* notify call chain on event */
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
				TYPEC_ACCESSORY_NONE, NULL);

		/* deactivate switches */
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		regmap_write(fsa_priv->regmap, FSA4480_FUNCTION_ENABLE, 0x00);
		dev_dbg(fsa_priv->dev, "%s: set reg[0x12] reset.\n",__func__);
		break;
	default:
		/* ignore other usb connection modes */
		break;
	}

	mutex_unlock(&fsa_priv->notification_lock);
	return rc;
}

static int fsa4480_usbc_analog_setup_switches(struct fsa4480_priv *fsa_priv)
{
	if (fsa_priv->use_powersupply)
		return fsa4480_usbc_analog_setup_switches_psupply(fsa_priv);
	else
		return fsa4480_usbc_analog_setup_switches_ucsi(fsa_priv);
}

/*
 * fsa4480_reg_notifier - register notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on success, or error code
 */
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	rc = blocking_notifier_chain_register
				(&fsa_priv->fsa4480_notifier, nb);
	if (rc)
		return rc;

	/*
	 * as part of the init sequence check if there is a connected
	 * USB C analog adapter
	 */
	dev_dbg(fsa_priv->dev, "%s: verify if USB adapter is already inserted\n",
		__func__);
	rc = fsa4480_usbc_analog_setup_switches(fsa_priv);
	regmap_update_bits(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, 0x07,
			   fsa_priv->switch_control);
	return rc;
}
EXPORT_SYMBOL(fsa4480_reg_notifier);

/*
 * fsa4480_unreg_notifier - unregister notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on pass, or error code
 */
int fsa4480_unreg_notifier(struct notifier_block *nb,
			     struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	mutex_lock(&fsa_priv->notification_lock);

	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	rc = blocking_notifier_chain_unregister
				(&fsa_priv->fsa4480_notifier, nb);
	mutex_unlock(&fsa_priv->notification_lock);

	return rc;
}
EXPORT_SYMBOL(fsa4480_unreg_notifier);

static int fsa4480_validate_display_port_settings(struct fsa4480_priv *fsa_priv)
{
	u32 switch_status = 0;

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		pr_err("AUX SBU1/2 switch status is invalid = %u\n",
				switch_status);
		return -EIO;
	}

	return 0;
}
/*
 * fsa4480_switch_event - configure FSA switch position based on event
 *
 * @node - phandle node to fsa4480 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;
	if (!fsa_priv->regmap)
		return -EINVAL;

	switch (event) {
	case FSA_MIC_GND_SWAP:
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL,
				&fsa_priv->switch_control);
		if ((fsa_priv->switch_control & 0x07) == 0x07)
			fsa_priv->switch_control = 0x0;
		else
			fsa_priv->switch_control = 0x7;
		fsa4480_usbc_update_settings(fsa_priv, fsa_priv->switch_control,
					     0x9F);
		break;
	case FSA_USBC_ORIENTATION_CC1:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_ORIENTATION_CC2:
		fsa4480_usbc_update_settings(fsa_priv, 0x78, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_DISPLAYPORT_DISCONNECTED:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_event);

static void fsa4480_usbc_analog_work_fn(struct work_struct *work)
{
	struct fsa4480_priv *fsa_priv =
		container_of(work, struct fsa4480_priv, usbc_analog_work);

	if (!fsa_priv) {
		pr_err("%s: fsa container invalid\n", __func__);
		return;
	}
	fsa4480_usbc_analog_setup_switches(fsa_priv);
	pm_relax(fsa_priv->dev);
}

static void fsa4480_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
		regmap_write(regmap, fsa_reg_i2c_defaults[i].reg,
				   fsa_reg_i2c_defaults[i].val);
}

static int fsa4480_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct fsa4480_priv *fsa_priv;
	u32 use_powersupply = 0;
	int rc = 0;

	fsa_priv = devm_kzalloc(&i2c->dev, sizeof(*fsa_priv),
				GFP_KERNEL);
	if (!fsa_priv)
		return -ENOMEM;

	memset(fsa_priv, 0, sizeof(struct fsa4480_priv));
	fsa_priv->dev = &i2c->dev;

	fsa_priv->regmap = devm_regmap_init_i2c(i2c, &fsa4480_regmap_config);
	if (IS_ERR_OR_NULL(fsa_priv->regmap)) {
		dev_err(fsa_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!fsa_priv->regmap) {
			rc = -EINVAL;
			goto err_data;
		}
		rc = PTR_ERR(fsa_priv->regmap);
		goto err_data;
	}

	fsa4480_update_reg_defaults(fsa_priv->regmap);

	fsa_priv->nb.notifier_call = fsa4480_usbc_event_changed;
	fsa_priv->nb.priority = 0;
	rc = of_property_read_u32(fsa_priv->dev->of_node,
			"qcom,use-power-supply", &use_powersupply);
	if (rc || use_powersupply == 0) {
		dev_dbg(fsa_priv->dev,
			"%s: Looking up %s property failed or disabled\n",
			__func__, "qcom,use-power-supply");

		fsa_priv->use_powersupply = 0;
		rc = register_ucsi_glink_notifier(&fsa_priv->nb);
		if (rc) {
			dev_err(fsa_priv->dev,
			  "%s: ucsi glink notifier registration failed: %d\n",
			  __func__, rc);
			goto err_data;
		}
	} else {
		fsa_priv->use_powersupply = 1;
		fsa_priv->usb_psy = power_supply_get_by_name("usb");
		if (!fsa_priv->usb_psy) {
			rc = -EPROBE_DEFER;
			dev_dbg(fsa_priv->dev,
				"%s: could not get USB psy info: %d\n",
				__func__, rc);
			goto err_data;
		}

		fsa_priv->iio_ch = iio_channel_get(fsa_priv->dev, "typec_mode");
		if (!fsa_priv->iio_ch) {
			dev_err(fsa_priv->dev,
				"%s: iio_channel_get failed for typec_mode\n",
				__func__);
			goto err_supply;
		}
		rc = power_supply_reg_notifier(&fsa_priv->nb);
		if (rc) {
			dev_err(fsa_priv->dev,
				"%s: power supply reg failed: %d\n",
			__func__, rc);
			goto err_supply;
		}
	}

	mutex_init(&fsa_priv->notification_lock);
	i2c_set_clientdata(i2c, fsa_priv);

	INIT_WORK(&fsa_priv->usbc_analog_work,
		  fsa4480_usbc_analog_work_fn);

	fsa_priv->fsa4480_notifier.rwsem =
		(struct rw_semaphore)__RWSEM_INITIALIZER
		((fsa_priv->fsa4480_notifier).rwsem);
	fsa_priv->fsa4480_notifier.head = NULL;

	return 0;

err_supply:
	power_supply_put(fsa_priv->usb_psy);
err_data:
	devm_kfree(&i2c->dev, fsa_priv);
	return rc;
}

static int fsa4480_remove(struct i2c_client *i2c)
{
	struct fsa4480_priv *fsa_priv =
			(struct fsa4480_priv *)i2c_get_clientdata(i2c);

	if (!fsa_priv)
		return -EINVAL;
	regmap_write(fsa_priv->regmap, FSA4480_FUNCTION_ENABLE, 0x00);
	dev_dbg(fsa_priv->dev, "%s: set reg[0x12] reset.\n",__func__);

	if (fsa_priv->use_powersupply) {
		/* deregister from PMI */
		power_supply_unreg_notifier(&fsa_priv->nb);
		power_supply_put(fsa_priv->usb_psy);
	} else {
		unregister_ucsi_glink_notifier(&fsa_priv->nb);
	}
	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	cancel_work_sync(&fsa_priv->usbc_analog_work);
	pm_relax(fsa_priv->dev);
	mutex_destroy(&fsa_priv->notification_lock);
	dev_set_drvdata(&i2c->dev, NULL);

	return 0;
}

static const struct of_device_id fsa4480_i2c_dt_match[] = {
	{
		.compatible = "qcom,fsa4480-i2c",
	},
	{}
};

static struct i2c_driver fsa4480_i2c_driver = {
	.driver = {
		.name = FSA4480_I2C_NAME,
		.of_match_table = fsa4480_i2c_dt_match,
	},
	.probe = fsa4480_probe,
	.remove = fsa4480_remove,
};

static int __init fsa4480_init(void)
{
	int rc;

	rc = i2c_add_driver(&fsa4480_i2c_driver);
	if (rc)
		pr_err("fsa4480: Failed to register I2C driver: %d\n", rc);

	return rc;
}
module_init(fsa4480_init);

static void __exit fsa4480_exit(void)
{
	i2c_del_driver(&fsa4480_i2c_driver);
}
module_exit(fsa4480_exit);

MODULE_DESCRIPTION("FSA4480 I2C driver");
MODULE_LICENSE("GPL v2");
