// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>
#include <soc/soundwire.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <asoc/msm-cdc-pinctrl.h>
#include "wsa881x.h"
#include "wsa881x-temp-sensor.h"
#include "asoc/bolero-slave-internal.h"

#define WSA881X_NUM_RETRY	5

#define MAX_NAME_LEN 30
#define WSA881X_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 |\
			SNDRV_PCM_RATE_384000)
/* Fractional Rates */
#define WSA881X_FRAC_RATES (SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_88200 |\
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_352800)

#define WSA881X_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

enum {
	G_18DB = 0,
	G_16P5DB,
	G_15DB,
	G_13P5DB,
	G_12DB,
	G_10P5DB,
	G_9DB,
	G_7P5DB,
	G_6DB,
	G_4P5DB,
	G_3DB,
	G_1P5DB,
	G_0DB,
};

enum {
	DISABLE = 0,
	ENABLE,
};

enum {
	SWR_DAC_PORT,
	SWR_COMP_PORT,
	SWR_BOOST_PORT,
	SWR_VISENSE_PORT,
};

struct swr_port {
	u8 port_id;
	u8 ch_mask;
	u32 ch_rate;
	u8 num_ch;
	u8 port_type;
};

enum {
	WSA881X_DEV_DOWN,
	WSA881X_DEV_UP,
	WSA881X_DEV_READY,
};

/*
 * Private data Structure for wsa881x. All parameters related to
 * WSA881X codec needs to be defined here.
 */
struct wsa881x_priv {
	struct regmap *regmap;
	struct device *dev;
	struct swr_device *swr_slave;
	struct snd_soc_component *component;
	bool comp_enable;
	bool boost_enable;
	bool visense_enable;
	u8 pa_gain;
	struct swr_port port[WSA881X_MAX_SWR_PORTS];
	int pd_gpio;
	struct wsa881x_tz_priv tz_pdata;
	int bg_cnt;
	int clk_cnt;
	int version;
	struct mutex bg_lock;
	struct mutex res_lock;
	struct mutex temp_lock;
	struct snd_info_entry *entry;
	struct snd_info_entry *version_entry;
	int state;
	struct delayed_work ocp_ctl_work;
	struct device_node *wsa_rst_np;
	int pa_mute;
	struct device_node *bolero_np;
	struct platform_device* bolero_dev;
	struct notifier_block bolero_nblock;
	void *handle;
	int (*register_notifier)(void *handle,
				 struct notifier_block *nblock,
				 bool enable);
	struct dentry *debugfs_dent;
	struct dentry *debugfs_peek;
	struct dentry *debugfs_poke;
	struct dentry *debugfs_reg_dump;
	unsigned int read_data;
	char *wsa881x_name_prefix;
	struct snd_soc_dai_driver *dai_driver;
	struct snd_soc_component_driver *driver;
};

struct wsa_ctrl_platform_data {
	void *handle;
	int (*update_wsa_event)(void *handle, u16 event, u32 data);
	int (*register_notifier)(void *handle,
				 struct notifier_block *nblock,
				 bool enable);
};

#define SWR_SLV_MAX_REG_ADDR	0x390
#define SWR_SLV_START_REG_ADDR	0x40
#define SWR_SLV_MAX_BUF_LEN	25
#define BYTES_PER_LINE		12
#define SWR_SLV_RD_BUF_LEN	8
#define SWR_SLV_WR_BUF_LEN	32
#define SWR_SLV_MAX_DEVICES	2

#define WSA881X_VERSION_ENTRY_SIZE 27
#define WSA881X_OCP_CTL_TIMER_SEC 2
#define WSA881X_OCP_CTL_TEMP_CELSIUS 25
#define WSA881X_OCP_CTL_POLL_TIMER_SEC 60

static int wsa881x_ocp_poll_timer_sec = WSA881X_OCP_CTL_POLL_TIMER_SEC;
module_param(wsa881x_ocp_poll_timer_sec, int, 0664);
MODULE_PARM_DESC(wsa881x_ocp_poll_timer_sec, "timer for ocp ctl polling");

static int32_t wsa881x_resource_acquire(struct snd_soc_component *component,
						bool enable);

static const char * const wsa_pa_gain_text[] = {
	"G_18_DB", "G_16P5_DB", "G_15_DB", "G_13P5_DB", "G_12_DB", "G_10P5_DB",
	"G_9_DB", "G_7P5_DB", "G_6_DB", "G_4P5_DB", "G_3_DB", "G_1P5_DB",
	"G_0_DB"
};

static const struct soc_enum wsa_pa_gain_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(wsa_pa_gain_text), wsa_pa_gain_text);

static int wsa_pa_gain_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa881x->pa_gain;

	dev_dbg(component->dev, "%s: PA gain = 0x%x\n", __func__,
			wsa881x->pa_gain);

	return 0;
}

static int wsa_pa_gain_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	wsa881x->pa_gain =  ucontrol->value.integer.value[0];

	return 0;
}

static int wsa881x_get_mute(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa881x->pa_mute;

	return 0;
}

static int wsa881x_set_mute(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: mute current %d, new %d\n",
		__func__, wsa881x->pa_mute, value);

	if (value)
		snd_soc_component_update_bits(component, WSA881X_SPKR_DRV_EN,
				0x80, 0x00);
	wsa881x->pa_mute = value;

	return 0;
}

static int wsa881x_get_t0_init(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	struct wsa881x_tz_priv *pdata = &wsa881x->tz_pdata;

	ucontrol->value.integer.value[0] = pdata->t0_init;
	dev_dbg(component->dev, "%s: t0 init %d\n", __func__, pdata->t0_init);

	return 0;
}

static int wsa881x_set_t0_init(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	struct wsa881x_tz_priv *pdata = &wsa881x->tz_pdata;

	pdata->t0_init = ucontrol->value.integer.value[0];
	dev_dbg(component->dev, "%s: t0 init %d\n", __func__, pdata->t0_init);

	return 0;
}

static const struct snd_kcontrol_new wsa_snd_controls[] = {
	SOC_ENUM_EXT("WSA PA Gain", wsa_pa_gain_enum,
		     wsa_pa_gain_get, wsa_pa_gain_put),
	SOC_SINGLE_EXT("WSA PA Mute", SND_SOC_NOPM, 0, 1, 0,
		wsa881x_get_mute, wsa881x_set_mute),
	SOC_SINGLE_EXT("WSA T0 Init", SND_SOC_NOPM, 0, 1, 0,
		wsa881x_get_t0_init, wsa881x_set_t0_init),
};

static int codec_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int get_parameters(char *buf, u32 *param1, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");
	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token) {
			if ((token[1] == 'x') || (token[1] == 'X'))
				base = 16;
			else
				base = 10;

			if (kstrtou32(token, base, &param1[cnt]) != 0)
				return -EINVAL;

			token = strsep(&buf, " ");
		} else
			return -EINVAL;
	}
	return 0;
}

static ssize_t wsa881x_codec_version_read(struct snd_info_entry *entry,
			       void *file_private_data, struct file *file,
			       char __user *buf, size_t count, loff_t pos)
{
	struct wsa881x_priv *wsa881x;
	char buffer[WSA881X_VERSION_ENTRY_SIZE];
	int len;

	wsa881x = (struct wsa881x_priv *) entry->private_data;
	if (!wsa881x) {
		pr_err("%s: wsa881x priv is null\n", __func__);
		return -EINVAL;
	}

	len = snprintf(buffer, sizeof(buffer), "WSA881X-SOUNDWIRE_2_0\n");

	return simple_read_from_buffer(buf, count, &pos, buffer, len);
}

static struct snd_info_entry_ops wsa881x_codec_info_ops = {
	.read = wsa881x_codec_version_read,
};

/*
 * wsa881x_codec_info_create_codec_entry - creates wsa881x module
 * @codec_root: The parent directory
 * @component: Codec instance
 *
 * Creates wsa881x module and version entry under the given
 * parent directory.
 *
 * Return: 0 on success or negative error code on failure.
 */
int wsa881x_codec_info_create_codec_entry(struct snd_info_entry *codec_root,
					  struct snd_soc_component *component)
{
	struct snd_info_entry *version_entry;
	struct wsa881x_priv *wsa881x;
	struct snd_soc_card *card;
	char name[80];

	if (!codec_root || !component)
		return -EINVAL;

	wsa881x = snd_soc_component_get_drvdata(component);
	card = component->card;
	snprintf(name, sizeof(name), "%s.%x", "wsa881x",
		 (u32)wsa881x->swr_slave->addr);

	wsa881x->entry = snd_info_create_module_entry(codec_root->module,
						(const char *)name,
						codec_root);
	if (!wsa881x->entry) {
		dev_dbg(component->dev, "%s: failed to create wsa881x entry\n",
			__func__);
		return -ENOMEM;
	}
	wsa881x->entry->mode = S_IFDIR | 0555;
	if (snd_info_register(wsa881x->entry) < 0) {
		snd_info_free_entry(wsa881x->entry);
		return -ENOMEM;
	}

	version_entry = snd_info_create_card_entry(card->snd_card,
						   "version",
						   wsa881x->entry);
	if (!version_entry) {
		dev_dbg(component->dev, "%s: failed to create wsa881x version entry\n",
			__func__);
		snd_info_free_entry(wsa881x->entry);
		return -ENOMEM;
	}

	version_entry->private_data = wsa881x;
	version_entry->size = WSA881X_VERSION_ENTRY_SIZE;
	version_entry->content = SNDRV_INFO_CONTENT_DATA;
	version_entry->c.ops = &wsa881x_codec_info_ops;

	if (snd_info_register(version_entry) < 0) {
		snd_info_free_entry(version_entry);
		snd_info_free_entry(wsa881x->entry);
		return -ENOMEM;
	}
	wsa881x->version_entry = version_entry;

	return 0;
}
EXPORT_SYMBOL(wsa881x_codec_info_create_codec_entry);

static bool is_swr_slv_reg_readable(int reg)
{
	bool ret = true;

	if (((reg > 0x46) && (reg < 0x4A)) ||
	    ((reg > 0x4A) && (reg < 0x50)) ||
	    ((reg > 0x55) && (reg < 0xE0)) ||
	    ((reg > 0xE0) && (reg < 0xF0)) ||
	    ((reg > 0xF0) && (reg < 0x100)) ||
	    ((reg > 0x105) && (reg < 0x120)) ||
	    ((reg > 0x128) && (reg < 0x130)) ||
	    ((reg > 0x138) && (reg < 0x200)) ||
	    ((reg > 0x205) && (reg < 0x220)) ||
	    ((reg > 0x228) && (reg < 0x230)) ||
	    ((reg > 0x238) && (reg < 0x300)) ||
	    ((reg > 0x305) && (reg < 0x320)) ||
	    ((reg > 0x328) && (reg < 0x330)) ||
	    ((reg > 0x338) && (reg < 0x400)) ||
	    ((reg > 0x405) && (reg < 0x420)))
		ret = false;

	return ret;
}

static ssize_t wsa881x_swrslave_reg_show(struct swr_device *pdev, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	int i, reg_val, len;
	ssize_t total = 0;
	char tmp_buf[SWR_SLV_MAX_BUF_LEN];

	if (!ubuf || !ppos)
		return 0;

	for (i = (((int) *ppos / BYTES_PER_LINE) + SWR_SLV_START_REG_ADDR);
			i <= SWR_SLV_MAX_REG_ADDR; i++) {
		if (!is_swr_slv_reg_readable(i))
			continue;
		swr_read(pdev, pdev->dev_num, i, &reg_val, 1);
		len = snprintf(tmp_buf, sizeof(tmp_buf), "0x%.3x: 0x%.2x\n", i,
				(reg_val & 0xFF));
		if (len < 0) {
			pr_err("%s: fail to fill the buffer\n", __func__);
			total = -EFAULT;
			goto copy_err;
		}
		if ((total + len) >= count - 1)
			break;
		if (copy_to_user((ubuf + total), tmp_buf, len)) {
			pr_err("%s: fail to copy reg dump\n", __func__);
			total = -EFAULT;
			goto copy_err;
		}
		total += len;
		*ppos += len;
	}

copy_err:
	*ppos = SWR_SLV_MAX_REG_ADDR * BYTES_PER_LINE;
	return total;
}

static ssize_t codec_debug_dump(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct swr_device *pdev;

	if (!count || !file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	return wsa881x_swrslave_reg_show(pdev, ubuf, count, ppos);
}

static ssize_t codec_debug_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	char lbuf[SWR_SLV_RD_BUF_LEN];
	struct swr_device *pdev = NULL;
	struct wsa881x_priv *wsa881x = NULL;

	if (!count || !file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	snprintf(lbuf, sizeof(lbuf), "0x%x\n",
			(wsa881x->read_data & 0xFF));

	return simple_read_from_buffer(ubuf, count, ppos, lbuf,
			strnlen(lbuf, 7));
}

static ssize_t codec_debug_peek_write(struct file *file,
		const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char lbuf[SWR_SLV_WR_BUF_LEN];
	int rc = 0;
	u32 param[5];
	struct swr_device *pdev = NULL;
	struct wsa881x_priv *wsa881x = NULL;

	if (!cnt || !file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';
	rc = get_parameters(lbuf, param, 1);
	if (!((param[0] <= SWR_SLV_MAX_REG_ADDR) && (rc == 0)))
		return -EINVAL;
	swr_read(pdev, pdev->dev_num, param[0], &wsa881x->read_data, 1);
	if (rc == 0)
		rc = cnt;
	else
		pr_err("%s: rc = %d\n", __func__, rc);

	return rc;
}

static ssize_t codec_debug_write(struct file *file,
		const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char lbuf[SWR_SLV_WR_BUF_LEN];
	int rc = 0;
	u32 param[5];
	struct swr_device *pdev;

	if (!file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';
	rc = get_parameters(lbuf, param, 2);
	if (!((param[0] <= SWR_SLV_MAX_REG_ADDR) &&
				(param[1] <= 0xFF) && (rc == 0)))
		return -EINVAL;
	swr_write(pdev, pdev->dev_num, param[0], &param[1]);
	if (rc == 0)
		rc = cnt;
	else
		pr_err("%s: rc = %d\n", __func__, rc);

	return rc;
}

static const struct file_operations codec_debug_write_ops = {
	.open = codec_debug_open,
	.write = codec_debug_write,
};

static const struct file_operations codec_debug_read_ops = {
	.open = codec_debug_open,
	.read = codec_debug_read,
	.write = codec_debug_peek_write,
};

static const struct file_operations codec_debug_dump_ops = {
	.open = codec_debug_open,
	.read = codec_debug_dump,
};
static void wsa881x_regcache_sync(struct wsa881x_priv *wsa881x)
{
	mutex_lock(&wsa881x->res_lock);
	if (wsa881x->state != WSA881X_DEV_READY) {
		regcache_mark_dirty(wsa881x->regmap);
		regcache_sync(wsa881x->regmap);
		wsa881x->state = WSA881X_DEV_READY;
	}
	mutex_unlock(&wsa881x->res_lock);
}

static const struct reg_sequence wsa881x_pre_pmu_pa[] = {
	{WSA881X_SPKR_DRV_GAIN, 0x41, 0},
	{WSA881X_SPKR_MISC_CTL1, 0x01, 0},
	{WSA881X_ADC_EN_DET_TEST_I, 0x01, 0},
	{WSA881X_ADC_EN_MODU_V, 0x02, 0},
	{WSA881X_ADC_EN_DET_TEST_V, 0x10, 0},
	{WSA881X_SPKR_PWRSTG_DBG, 0xA0, 0},
};

static const struct reg_sequence wsa881x_pre_pmu_pa_2_0[] = {
	{WSA881X_SPKR_DRV_GAIN, 0x41, 0},
	{WSA881X_SPKR_MISC_CTL1, 0x87, 0},
};

static const struct reg_sequence wsa881x_post_pmu_pa[] = {
	{WSA881X_SPKR_PWRSTG_DBG, 0x00, 0},
	{WSA881X_ADC_EN_DET_TEST_V, 0x00, 0},
	{WSA881X_ADC_EN_MODU_V, 0x00, 0},
	{WSA881X_ADC_EN_DET_TEST_I, 0x00, 0},
};

static const struct reg_sequence wsa881x_vi_txfe_en[] = {
	{WSA881X_SPKR_PROT_FE_VSENSE_VCM, 0x85, 0},
	{WSA881X_SPKR_PROT_ATEST2, 0x0A, 0},
	{WSA881X_SPKR_PROT_FE_GAIN, 0xCF, 0},
};

static const struct reg_sequence wsa881x_vi_txfe_en_2_0[] = {
	{WSA881X_SPKR_PROT_FE_VSENSE_VCM, 0x85, 0},
	{WSA881X_SPKR_PROT_ATEST2, 0x0A, 0},
	{WSA881X_SPKR_PROT_FE_GAIN, 0x47, 0},
};

static int wsa881x_boost_ctrl(struct snd_soc_component *component, bool enable)
{
	dev_dbg(component->dev, "%s: enable:%d\n", __func__, enable);
	if (enable)
		snd_soc_component_update_bits(component, WSA881X_BOOST_EN_CTL,
				0x80, 0x80);
	else
		snd_soc_component_update_bits(component, WSA881X_BOOST_EN_CTL,
				0x80, 0x00);
	/*
	 * 1.5ms sleep is needed after boost enable/disable as per
	 * HW requirement
	 */
	usleep_range(1500, 1510);
	return 0;
}

static int wsa881x_visense_txfe_ctrl(struct snd_soc_component *component,
				     bool enable, u8 isense1_gain,
				     u8 isense2_gain, u8 vsense_gain)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev,
		"%s: enable:%d, isense1 gain: %d, isense2 gain: %d, vsense_gain %d\n",
		__func__, enable, isense1_gain, isense2_gain, vsense_gain);

	if (enable) {
		regmap_multi_reg_write(wsa881x->regmap,
				wsa881x_vi_txfe_en_2_0,
				ARRAY_SIZE(wsa881x_vi_txfe_en_2_0));
	} else {
		snd_soc_component_update_bits(component,
				WSA881X_SPKR_PROT_FE_VSENSE_VCM,
				0x08, 0x08);
		/*
		 * 200us sleep is needed after visense txfe disable as per
		 * HW requirement.
		 */
		usleep_range(200, 210);
		snd_soc_component_update_bits(component,
				WSA881X_SPKR_PROT_FE_GAIN,
				0x01, 0x00);
	}
	return 0;
}

static int wsa881x_visense_adc_ctrl(struct snd_soc_component *component,
				    bool enable)
{

	dev_dbg(component->dev, "%s: enable:%d\n", __func__, enable);
	snd_soc_component_update_bits(component, WSA881X_ADC_EN_MODU_V,
			(0x01 << 7), (enable << 7));
	snd_soc_component_update_bits(component, WSA881X_ADC_EN_MODU_I,
			(0x01 << 7), (enable << 7));
	return 0;
}

static void wsa881x_bandgap_ctrl(struct snd_soc_component *component,
				 bool enable)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: enable:%d, bg_count:%d\n", __func__,
		enable, wsa881x->bg_cnt);
	mutex_lock(&wsa881x->bg_lock);
	if (enable) {
		++wsa881x->bg_cnt;
		if (wsa881x->bg_cnt == 1) {
			snd_soc_component_update_bits(component,
					WSA881X_TEMP_OP,
					0x08, 0x08);
			/* 400usec sleep is needed as per HW requirement */
			usleep_range(400, 410);
			snd_soc_component_update_bits(component,
						WSA881X_TEMP_OP,
						0x04, 0x04);
		}
	} else {
		--wsa881x->bg_cnt;
		if (wsa881x->bg_cnt <= 0) {
			WARN_ON(wsa881x->bg_cnt < 0);
			wsa881x->bg_cnt = 0;
			snd_soc_component_update_bits(component,
					WSA881X_TEMP_OP, 0x04, 0x00);
			snd_soc_component_update_bits(component,
					WSA881X_TEMP_OP, 0x08, 0x00);
		}
	}
	mutex_unlock(&wsa881x->bg_lock);
}

static void wsa881x_clk_ctrl(struct snd_soc_component *component, bool enable)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: enable:%d, clk_count:%d\n", __func__,
		enable, wsa881x->clk_cnt);
	mutex_lock(&wsa881x->res_lock);
	if (enable) {
		++wsa881x->clk_cnt;
		if (wsa881x->clk_cnt == 1) {
			snd_soc_component_write(component,
					WSA881X_CDC_DIG_CLK_CTL, 0x01);
			snd_soc_component_write(component,
					WSA881X_CDC_ANA_CLK_CTL, 0x01);
		}
	} else {
		--wsa881x->clk_cnt;
		if (wsa881x->clk_cnt <= 0) {
			WARN_ON(wsa881x->clk_cnt < 0);
			wsa881x->clk_cnt = 0;
			snd_soc_component_write(component,
					WSA881X_CDC_DIG_CLK_CTL, 0x00);
			snd_soc_component_write(component,
					WSA881X_CDC_ANA_CLK_CTL, 0x00);
		}
	}
	mutex_unlock(&wsa881x->res_lock);
}

static int wsa881x_get_compander(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa881x->comp_enable;
	return 0;
}

static int wsa881x_set_compander(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: Compander enable current %d, new %d\n",
		 __func__, wsa881x->comp_enable, value);
	wsa881x->comp_enable = value;
	return 0;
}

static int wsa881x_get_boost(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa881x->boost_enable;
	return 0;
}

static int wsa881x_set_boost(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: Boost enable current %d, new %d\n",
		 __func__, wsa881x->boost_enable, value);
	wsa881x->boost_enable = value;
	return 0;
}

static int wsa881x_get_visense(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = wsa881x->visense_enable;
	return 0;
}

static int wsa881x_set_visense(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	int value = ucontrol->value.integer.value[0];

	dev_dbg(component->dev, "%s: VIsense enable current %d, new %d\n",
		 __func__, wsa881x->visense_enable, value);
	wsa881x->visense_enable = value;
	return 0;
}

static int wsa881x_set_boost_level(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
				snd_soc_kcontrol_component(kcontrol);
	u8 wsa_boost_level = 0;

	dev_dbg(component->dev, "%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	wsa_boost_level = ucontrol->value.integer.value[0];
	snd_soc_component_update_bits(component, WSA881X_BOOST_PRESET_OUT1,
			0xff, wsa_boost_level);

	return 0;
}

static int wsa881x_get_boost_level(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
			snd_soc_kcontrol_component(kcontrol);
	u8 wsa_boost_level = 0;

	wsa_boost_level = snd_soc_component_read32(component,
				WSA881X_BOOST_PRESET_OUT1);
	ucontrol->value.integer.value[0] = wsa_boost_level;
	dev_dbg(component->dev, "%s: boost level = 0x%x\n", __func__,
		wsa_boost_level);

	return 0;
}

static const struct snd_kcontrol_new wsa881x_snd_controls[] = {
	SOC_SINGLE_EXT("COMP Switch", SND_SOC_NOPM, 0, 1, 0,
		wsa881x_get_compander, wsa881x_set_compander),

	SOC_SINGLE_EXT("BOOST Switch", SND_SOC_NOPM, 0, 1, 0,
		wsa881x_get_boost, wsa881x_set_boost),

	SOC_SINGLE_EXT("VISENSE Switch", SND_SOC_NOPM, 0, 1, 0,
		wsa881x_get_visense, wsa881x_set_visense),

	SOC_SINGLE_EXT("Boost Level", SND_SOC_NOPM, 0, 0xff, 0,
		wsa881x_get_boost_level, wsa881x_set_boost_level),
};

static const struct snd_kcontrol_new swr_dac_port[] = {
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0)
};

static int wsa881x_set_port(struct snd_soc_component *component, int port_idx,
			u8 *port_id, u8 *num_ch, u8 *ch_mask, u32 *ch_rate,
			u8 *port_type)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	*port_id = wsa881x->port[port_idx].port_id;
	*num_ch = wsa881x->port[port_idx].num_ch;
	*ch_mask = wsa881x->port[port_idx].ch_mask;
	*ch_rate = wsa881x->port[port_idx].ch_rate;
	*port_type = wsa881x->port[port_idx].port_type;
	return 0;
}

static int wsa881x_enable_swr_dac_port(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	u8 port_id[WSA881X_MAX_SWR_PORTS];
	u8 num_ch[WSA881X_MAX_SWR_PORTS];
	u8 ch_mask[WSA881X_MAX_SWR_PORTS];
	u32 ch_rate[WSA881X_MAX_SWR_PORTS];
	u8 port_type[WSA881X_MAX_SWR_PORTS];
	u8 num_port = 0;

	dev_dbg(component->dev, "%s: event %d name %s\n", __func__,
		event, w->name);
	if (wsa881x == NULL)
		return -EINVAL;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wsa881x_set_port(component, SWR_DAC_PORT,
				&port_id[num_port], &num_ch[num_port],
				&ch_mask[num_port], &ch_rate[num_port],
				&port_type[num_port]);
		++num_port;

		if (wsa881x->comp_enable) {
			wsa881x_set_port(component, SWR_COMP_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		if (wsa881x->boost_enable) {
			wsa881x_set_port(component, SWR_BOOST_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		if (wsa881x->visense_enable) {
			wsa881x_set_port(component, SWR_VISENSE_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		swr_connect_port(wsa881x->swr_slave, &port_id[0], num_port,
				&ch_mask[0], &ch_rate[0], &num_ch[0],
					&port_type[0]);
		break;
	case SND_SOC_DAPM_POST_PMU:
		break;
	case SND_SOC_DAPM_PRE_PMD:
		break;
	case SND_SOC_DAPM_POST_PMD:
		wsa881x_set_port(component, SWR_DAC_PORT,
				&port_id[num_port], &num_ch[num_port],
				&ch_mask[num_port], &ch_rate[num_port],
				&port_type[num_port]);
		++num_port;

		if (wsa881x->comp_enable) {
			wsa881x_set_port(component, SWR_COMP_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		if (wsa881x->boost_enable) {
			wsa881x_set_port(component, SWR_BOOST_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		if (wsa881x->visense_enable) {
			wsa881x_set_port(component, SWR_VISENSE_PORT,
					&port_id[num_port], &num_ch[num_port],
					&ch_mask[num_port], &ch_rate[num_port],
					&port_type[num_port]);
			++num_port;
		}
		swr_disconnect_port(wsa881x->swr_slave, &port_id[0], num_port,
				&ch_mask[0], &port_type[0]);
		break;
	default:
		break;
	}
	return 0;
}

static int wsa881x_rdac_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	dev_dbg(component->dev, "%s: %s %d boost %d visense %d\n", __func__,
		w->name, event,	wsa881x->boost_enable,
		wsa881x->visense_enable);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		mutex_lock(&wsa881x->temp_lock);
		wsa881x_resource_acquire(component, ENABLE);
		mutex_unlock(&wsa881x->temp_lock);
		wsa881x_boost_ctrl(component, ENABLE);
		break;
	case SND_SOC_DAPM_POST_PMD:
		swr_slvdev_datapath_control(wsa881x->swr_slave,
					    wsa881x->swr_slave->dev_num,
					    false);
		wsa881x_boost_ctrl(component, DISABLE);
		mutex_lock(&wsa881x->temp_lock);
		wsa881x_resource_acquire(component, DISABLE);
		mutex_unlock(&wsa881x->temp_lock);
		break;
	}
	return 0;
}

static int wsa881x_ramp_pa_gain(struct snd_soc_component *component,
				int min_gain, int max_gain, int udelay)
{
	int val;

	for (val = min_gain; max_gain <= val; val--) {
		snd_soc_component_update_bits(component, WSA881X_SPKR_DRV_GAIN,
				    0xF0, val << 4);
		/*
		 * 1ms delay is needed for every step change in gain as per
		 * HW requirement.
		 */
		usleep_range(udelay, udelay+10);
	}
	return 0;
}

static void wsa881x_ocp_ctl_work(struct work_struct *work)
{
	struct wsa881x_priv *wsa881x;
	struct delayed_work *dwork;
	struct snd_soc_component *component;
	int temp_val;

	dwork = to_delayed_work(work);
	wsa881x = container_of(dwork, struct wsa881x_priv, ocp_ctl_work);

	if (wsa881x->state == WSA881X_DEV_DOWN)
		return;

	component = wsa881x->component;
	wsa881x_get_temp(wsa881x->tz_pdata.tz_dev, &temp_val);
	dev_dbg(component->dev, " temp = %d\n", temp_val);

	if (temp_val <= WSA881X_OCP_CTL_TEMP_CELSIUS)
		snd_soc_component_update_bits(component, WSA881X_SPKR_OCP_CTL,
					0xC0, 0x00);
	else
		snd_soc_component_update_bits(component, WSA881X_SPKR_OCP_CTL,
					0xC0, 0xC0);

	schedule_delayed_work(&wsa881x->ocp_ctl_work,
			msecs_to_jiffies(wsa881x_ocp_poll_timer_sec * 1000));
}

static int wsa881x_spkr_pa_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component =
			snd_soc_dapm_to_component(w->dapm);
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	int min_gain, max_gain;

	dev_dbg(component->dev, "%s: %s %d\n", __func__, w->name, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_component_update_bits(component, WSA881X_SPKR_OCP_CTL,
				0xC0, 0x80);
		regmap_multi_reg_write(wsa881x->regmap,
				wsa881x_pre_pmu_pa_2_0,
				ARRAY_SIZE(wsa881x_pre_pmu_pa_2_0));
		swr_slvdev_datapath_control(wsa881x->swr_slave,
					    wsa881x->swr_slave->dev_num,
					    true);
		/* Set register mode if compander is not enabled */
		if (!wsa881x->comp_enable)
			snd_soc_component_update_bits(component,
					WSA881X_SPKR_DRV_GAIN,
					0x08, 0x08);
		else
			snd_soc_component_update_bits(component,
					WSA881X_SPKR_DRV_GAIN,
					0x08, 0x00);

		break;
	case SND_SOC_DAPM_POST_PMU:
		if (!wsa881x->bolero_dev)
			snd_soc_component_update_bits(component,
					      WSA881X_SPKR_DRV_EN,
					      0x80, 0x80);
		if (!wsa881x->comp_enable) {
			max_gain = wsa881x->pa_gain;
			/*
			 * Gain has to set incrementally in 4 steps
			 * as per HW sequence
			 */
			if (max_gain > G_4P5DB)
				min_gain = G_0DB;
			else
				min_gain = max_gain + 3;
			/*
			 * 1ms delay is needed before change in gain
			 * as per HW requirement.
			 */
			usleep_range(1000, 1010);
			wsa881x_ramp_pa_gain(component, min_gain, max_gain,
					1000);
		}
		if (wsa881x->visense_enable) {
			wsa881x_visense_txfe_ctrl(component, ENABLE,
						0x00, 0x03, 0x01);
			snd_soc_component_update_bits(component,
						WSA881X_ADC_EN_SEL_IBAIS,
						0x07, 0x01);
			wsa881x_visense_adc_ctrl(component, ENABLE);
		}
		schedule_delayed_work(&wsa881x->ocp_ctl_work,
			msecs_to_jiffies(WSA881X_OCP_CTL_TIMER_SEC * 1000));
		/* Force remove group */
		swr_remove_from_group(wsa881x->swr_slave,
				      wsa881x->swr_slave->dev_num);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_component_update_bits(component,
					      WSA881X_SPKR_DRV_EN,
					      0x80, 0x00);
		if (wsa881x->visense_enable) {
			wsa881x_visense_adc_ctrl(component, DISABLE);
			snd_soc_component_update_bits(component,
						WSA881X_ADC_EN_SEL_IBAIS,
						0x07, 0x00);
			wsa881x_visense_txfe_ctrl(component, DISABLE,
						0x00, 0x01, 0x01);
		}
		cancel_delayed_work_sync(&wsa881x->ocp_ctl_work);
		snd_soc_component_update_bits(component, WSA881X_SPKR_OCP_CTL,
				0xC0, 0xC0);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget wsa881x_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN"),

	SND_SOC_DAPM_MIXER_E("SWR DAC_Port", SND_SOC_NOPM, 0, 0, swr_dac_port,
		ARRAY_SIZE(swr_dac_port), wsa881x_enable_swr_dac_port,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("RDAC", NULL, WSA881X_SPKR_DAC_CTL, 7, 0,
		wsa881x_rdac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("SPKR PGA", SND_SOC_NOPM, 0, 0, NULL, 0,
			wsa881x_spkr_pa_event, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_OUTPUT("SPKR"),
};

static const struct snd_soc_dapm_route wsa881x_audio_map[] = {
	{"SWR DAC_Port", "Switch", "IN"},
	{"RDAC", NULL, "SWR DAC_Port"},
	{"SPKR PGA", NULL, "RDAC"},
	{"SPKR", NULL, "SPKR PGA"},
};

int wsa881x_set_channel_map(struct snd_soc_component *component, u8 *port,
			    u8 num_port, unsigned int *ch_mask,
			    unsigned int *ch_rate, u8 *port_type)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	int i;

	if (!port || !ch_mask || !ch_rate ||
		(num_port > WSA881X_MAX_SWR_PORTS)) {
		dev_err(component->dev,
			"%s: Invalid port=%pK, ch_mask=%pK, ch_rate=%pK\n",
			__func__, port, ch_mask, ch_rate);
		return -EINVAL;
	}
	for (i = 0; i < num_port; i++) {
		wsa881x->port[i].port_id = port[i];
		wsa881x->port[i].ch_mask = ch_mask[i];
		wsa881x->port[i].ch_rate = ch_rate[i];
		wsa881x->port[i].num_ch = __sw_hweight8(ch_mask[i]);
		if (port_type)
			wsa881x->port[i].port_type = port_type[i];
	}
	return 0;
}
EXPORT_SYMBOL(wsa881x_set_channel_map);

static void wsa881x_init(struct snd_soc_component *component)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	wsa881x->version =
			snd_soc_component_read32(component, WSA881X_CHIP_ID1);
	wsa881x_regmap_defaults(wsa881x->regmap, wsa881x->version);
	/* Enable software reset output from soundwire slave */
	snd_soc_component_update_bits(component, WSA881X_SWR_RESET_EN,
			0x07, 0x07);
	/* Bring out of analog reset */
	snd_soc_component_update_bits(component, WSA881X_CDC_RST_CTL,
			0x02, 0x02);
	/* Bring out of digital reset */
	snd_soc_component_update_bits(component, WSA881X_CDC_RST_CTL,
			0x01, 0x01);

	snd_soc_component_update_bits(component, WSA881X_CLOCK_CONFIG,
			0x10, 0x10);
	snd_soc_component_update_bits(component, WSA881X_SPKR_OCP_CTL,
			0x02, 0x02);
	snd_soc_component_update_bits(component, WSA881X_SPKR_MISC_CTL1,
			0xC0, 0x80);
	snd_soc_component_update_bits(component, WSA881X_SPKR_MISC_CTL1,
			0x06, 0x06);
	snd_soc_component_update_bits(component, WSA881X_SPKR_BIAS_INT,
			0xFF, 0x00);
	snd_soc_component_update_bits(component, WSA881X_SPKR_PA_INT,
			0xF0, 0x40);
	snd_soc_component_update_bits(component, WSA881X_SPKR_PA_INT,
			0x0E, 0x0E);
	snd_soc_component_update_bits(component, WSA881X_BOOST_LOOP_STABILITY,
			0x03, 0x03);
	snd_soc_component_update_bits(component, WSA881X_BOOST_MISC2_CTL,
			0xFF, 0x14);
	snd_soc_component_update_bits(component, WSA881X_BOOST_START_CTL,
			0x80, 0x80);
	snd_soc_component_update_bits(component, WSA881X_BOOST_START_CTL,
			0x03, 0x00);
	snd_soc_component_update_bits(component,
			WSA881X_BOOST_SLOPE_COMP_ISENSE_FB,
			0x0C, 0x04);
	snd_soc_component_update_bits(component,
			WSA881X_BOOST_SLOPE_COMP_ISENSE_FB,
			0x03, 0x00);
	if (snd_soc_component_read32(component, WSA881X_OTP_REG_0))
		snd_soc_component_update_bits(component,
				WSA881X_BOOST_PRESET_OUT1,
				0xF0, 0x70);
	snd_soc_component_update_bits(component, WSA881X_BOOST_PRESET_OUT2,
			0xF0, 0x30);
	snd_soc_component_update_bits(component, WSA881X_SPKR_DRV_EN,
			0x08, 0x08);
	snd_soc_component_update_bits(component, WSA881X_BOOST_CURRENT_LIMIT,
			0x0F, 0x08);
	snd_soc_component_update_bits(component, WSA881X_SPKR_OCP_CTL,
			0x30, 0x30);
	snd_soc_component_update_bits(component, WSA881X_SPKR_OCP_CTL,
			0x0C, 0x00);
	snd_soc_component_update_bits(component, WSA881X_OTP_REG_28,
			0x3F, 0x3A);
	snd_soc_component_update_bits(component, WSA881X_BONGO_RESRV_REG1,
			0xFF, 0xB2);
	snd_soc_component_update_bits(component, WSA881X_BONGO_RESRV_REG2,
			0xFF, 0x05);
}

static int32_t wsa881x_resource_acquire(struct snd_soc_component *component,
						bool enable)
{
	wsa881x_clk_ctrl(component, enable);
	wsa881x_bandgap_ctrl(component, enable);
	return 0;
}

static int32_t wsa881x_temp_reg_read(struct snd_soc_component *component,
				     struct wsa_temp_register *wsa_temp_reg)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	struct swr_device *dev;
	u8 retry = WSA881X_NUM_RETRY;
	u8 devnum = 0;

	if (!wsa881x) {
		dev_err(component->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}
	dev = wsa881x->swr_slave;
	if (dev && (wsa881x->state == WSA881X_DEV_DOWN)) {
		while (swr_get_logical_dev_num(dev, dev->addr, &devnum) &&
		       retry--) {
			/* Retry after 1 msec delay */
			usleep_range(1000, 1100);
		}
		if (retry == 0) {
			dev_err(component->dev,
				"%s get devnum %d for dev addr %lx failed\n",
				__func__, devnum, dev->addr);
			return -EINVAL;
		}
	}
	wsa881x_regcache_sync(wsa881x);
	mutex_lock(&wsa881x->temp_lock);
	wsa881x_resource_acquire(component, ENABLE);

	snd_soc_component_update_bits(component, WSA881X_TADC_VALUE_CTL,
				0x01, 0x00);
	wsa_temp_reg->dmeas_msb = snd_soc_component_read32(
					component, WSA881X_TEMP_MSB);
	wsa_temp_reg->dmeas_lsb = snd_soc_component_read32(
					component, WSA881X_TEMP_LSB);
	snd_soc_component_update_bits(component, WSA881X_TADC_VALUE_CTL,
					0x01, 0x01);
	wsa_temp_reg->d1_msb = snd_soc_component_read32(
					component, WSA881X_OTP_REG_1);
	wsa_temp_reg->d1_lsb = snd_soc_component_read32(
					component, WSA881X_OTP_REG_2);
	wsa_temp_reg->d2_msb = snd_soc_component_read32(
					component, WSA881X_OTP_REG_3);
	wsa_temp_reg->d2_lsb = snd_soc_component_read32(
					component, WSA881X_OTP_REG_4);

	wsa881x_resource_acquire(component, DISABLE);
	mutex_unlock(&wsa881x->temp_lock);

	return 0;
}

static int wsa881x_probe(struct snd_soc_component *component)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);
	struct swr_device *dev;

	if (!wsa881x)
		return -EINVAL;
	snd_soc_component_init_regmap(component, wsa881x->regmap);

	dev = wsa881x->swr_slave;
	wsa881x->component = component;
	mutex_init(&wsa881x->bg_lock);
	wsa881x_init(component);
	snprintf(wsa881x->tz_pdata.name, sizeof(wsa881x->tz_pdata.name),
		"%s.%x", "wsatz", (u8)dev->addr);
	wsa881x->bg_cnt = 0;
	wsa881x->clk_cnt = 0;
	wsa881x->tz_pdata.component = component;
	wsa881x->tz_pdata.wsa_temp_reg_read = wsa881x_temp_reg_read;
	wsa881x_init_thermal(&wsa881x->tz_pdata);
	snd_soc_add_component_controls(component, wsa_snd_controls,
				   ARRAY_SIZE(wsa_snd_controls));
	INIT_DELAYED_WORK(&wsa881x->ocp_ctl_work, wsa881x_ocp_ctl_work);
	return 0;
}

static void wsa881x_remove(struct snd_soc_component *component)
{
	struct wsa881x_priv *wsa881x = snd_soc_component_get_drvdata(component);

	if (wsa881x->tz_pdata.tz_dev)
		wsa881x_deinit_thermal(wsa881x->tz_pdata.tz_dev);
	mutex_destroy(&wsa881x->bg_lock);

	return;
}

static const struct snd_soc_component_driver soc_codec_dev_wsa881x = {
	.name = "wsa-codec",
	.probe = wsa881x_probe,
	.remove = wsa881x_remove,
	.controls = wsa881x_snd_controls,
	.num_controls = ARRAY_SIZE(wsa881x_snd_controls),
	.dapm_widgets = wsa881x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wsa881x_dapm_widgets),
	.dapm_routes = wsa881x_audio_map,
	.num_dapm_routes = ARRAY_SIZE(wsa881x_audio_map),
};

static struct snd_soc_dai_driver wsa_dai[] = {
	{
		.name = "wsa_rx",
		.playback = {
			.stream_name = "WSA881X_AIF Playback",
			.rates = WSA881X_RATES | WSA881X_FRAC_RATES,
			.formats = WSA881X_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
	},
};

static int wsa881x_gpio_ctrl(struct wsa881x_priv *wsa881x, bool enable)
{
	int ret = 0;

	if (wsa881x->pd_gpio < 0) {
		dev_err(wsa881x->dev, "%s: gpio is not valid %d\n",
			__func__, wsa881x->pd_gpio);
		return -EINVAL;
	}

	if (wsa881x->wsa_rst_np) {
		if (enable)
			ret = msm_cdc_pinctrl_select_active_state(
							wsa881x->wsa_rst_np);
		else
			ret = msm_cdc_pinctrl_select_sleep_state(
							wsa881x->wsa_rst_np);
		if (ret != 0)
			dev_err(wsa881x->dev,
				"%s: Failed to turn state %d; ret=%d\n",
				__func__, enable, ret);
	} else {
		if (gpio_is_valid(wsa881x->pd_gpio))
			gpio_direction_output(wsa881x->pd_gpio, enable);
	}

	return ret;
}

static int wsa881x_gpio_init(struct swr_device *pdev)
{
	int ret = 0;
	struct wsa881x_priv *wsa881x;

	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x) {
		dev_err(&pdev->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "%s: gpio %d request with name %s\n",
		__func__, wsa881x->pd_gpio, dev_name(&pdev->dev));
	ret = gpio_request(wsa881x->pd_gpio, dev_name(&pdev->dev));
	if (ret) {
		if (ret == -EBUSY) {
			/* GPIO was already requested */
			dev_dbg(&pdev->dev,
				 "%s: gpio %d is already set to high\n",
				 __func__, wsa881x->pd_gpio);
			ret = 0;
		} else {
			dev_err(&pdev->dev, "%s: Failed to request gpio %d, err: %d\n",
				__func__, wsa881x->pd_gpio, ret);
		}
	}
	return ret;
}

static int wsa881x_event_notify(struct notifier_block *nb,
				unsigned long val, void *ptr)
{
	u16 event = (val & 0xffff);
	struct wsa881x_priv *wsa881x = container_of(nb, struct wsa881x_priv,
						    bolero_nblock);

	if (!wsa881x)
		return -EINVAL;

	switch (event) {
	case BOLERO_SLV_EVT_PA_OFF_PRE_SSR:
		snd_soc_component_update_bits(wsa881x->component,
					      WSA881X_SPKR_DRV_GAIN,
					      0xF0, 0xC0);
		snd_soc_component_update_bits(wsa881x->component,
					      WSA881X_SPKR_DRV_EN,
					      0x80, 0x00);
		break;
	case BOLERO_SLV_EVT_PA_ON_POST_FSCLK:
	case BOLERO_SLV_EVT_PA_ON_POST_FSCLK_ADIE_LB:
		if ((snd_soc_component_read32(wsa881x->component,
				WSA881X_SPKR_DAC_CTL) & 0x80) == 0x80)
			snd_soc_component_update_bits(wsa881x->component,
					      WSA881X_SPKR_DRV_EN,
					      0x80, 0x80);
		break;
	default:
		break;
	}

	return 0;
}

static int wsa881x_swr_probe(struct swr_device *pdev)
{
	int ret = 0;
	struct wsa881x_priv *wsa881x = NULL;
	struct snd_soc_component *component;
	u8 devnum = 0;
	int dev_index = 0;
	bool pin_state_current = false;
	char buffer[MAX_NAME_LEN];
	const char *wsa881x_name_prefix_of = NULL;
	struct wsa_ctrl_platform_data *plat_data = NULL;

	wsa881x = devm_kzalloc(&pdev->dev, sizeof(struct wsa881x_priv),
			    GFP_KERNEL);
	if (!wsa881x)
		return -ENOMEM;
	wsa881x->wsa_rst_np = of_parse_phandle(pdev->dev.of_node,
					     "qcom,spkr-sd-n-node", 0);
	if (!wsa881x->wsa_rst_np) {
		dev_dbg(&pdev->dev, "%s: Not using pinctrl, fallback to gpio\n",
			__func__);
		wsa881x->pd_gpio = of_get_named_gpio(pdev->dev.of_node,
						     "qcom,spkr-sd-n-gpio", 0);
		if (wsa881x->pd_gpio < 0) {
			dev_err(&pdev->dev, "%s: %s property is not found %d\n",
				__func__, "qcom,spkr-sd-n-gpio",
				wsa881x->pd_gpio);
			goto err;
		}
		dev_dbg(&pdev->dev, "%s: reset gpio %d\n", __func__,
			wsa881x->pd_gpio);
	}
	swr_set_dev_data(pdev, wsa881x);

	wsa881x->swr_slave = pdev;

	if (!wsa881x->wsa_rst_np) {
		ret = wsa881x_gpio_init(pdev);
		if (ret)
			goto err;
	}
	if (wsa881x->wsa_rst_np)
		pin_state_current = msm_cdc_pinctrl_get_state(
						wsa881x->wsa_rst_np);
	wsa881x_gpio_ctrl(wsa881x, true);
	wsa881x->state = WSA881X_DEV_UP;

	if (!wsa881x->debugfs_dent) {
		wsa881x->debugfs_dent = debugfs_create_dir(
				dev_name(&pdev->dev), 0);
		if (!IS_ERR(wsa881x->debugfs_dent)) {
			wsa881x->debugfs_peek =
				debugfs_create_file("swrslave_peek",
						S_IFREG | 0444,
						wsa881x->debugfs_dent,
						(void *) pdev,
						&codec_debug_read_ops);

			wsa881x->debugfs_poke =
				debugfs_create_file("swrslave_poke",
						S_IFREG | 0444,
						wsa881x->debugfs_dent,
						(void *) pdev,
						&codec_debug_write_ops);

			wsa881x->debugfs_reg_dump =
				debugfs_create_file(
						"swrslave_reg_dump",
						S_IFREG | 0444,
						wsa881x->debugfs_dent,
						(void *) pdev,
						&codec_debug_dump_ops);
		}
	}

	/*
	 * Add 5msec delay to provide sufficient time for
	 * soundwire auto enumeration of slave devices as
	 * as per HW requirement.
	 */
	usleep_range(5000, 5010);
	ret = swr_get_logical_dev_num(pdev, pdev->addr, &devnum);
	if (ret) {
		dev_dbg(&pdev->dev,
			"%s get devnum %d for dev addr %lx failed\n",
			__func__, devnum, pdev->addr);
		goto dev_err;
	}
	pdev->dev_num = devnum;

	wsa881x->regmap = devm_regmap_init_swr(pdev,
					       &wsa881x_regmap_config);
	if (IS_ERR(wsa881x->regmap)) {
		ret = PTR_ERR(wsa881x->regmap);
		dev_err(&pdev->dev, "%s: regmap_init failed %d\n",
			__func__, ret);
		goto dev_err;
	}

	ret = of_property_read_string(pdev->dev.of_node,
			"qcom,wsa-prefix", &wsa881x_name_prefix_of);
	if (ret) {
		dev_err(&pdev->dev,
			"%s: Looking up %s property in node %s failed\n",
			__func__, "qcom,wsa-prefix",
			pdev->dev.of_node->full_name);
		goto dev_err;
	}

	wsa881x->driver = devm_kzalloc(&pdev->dev,
				sizeof(struct snd_soc_component_driver),
				GFP_KERNEL);
	if (!wsa881x->driver) {
		ret = -ENOMEM;
		goto err_mem;
	}

	memcpy(wsa881x->driver, &soc_codec_dev_wsa881x,
			sizeof(struct snd_soc_component_driver));

	wsa881x->dai_driver = devm_kzalloc(&pdev->dev,
				sizeof(struct snd_soc_dai_driver),
				GFP_KERNEL);
	if (!wsa881x->dai_driver) {
		ret = -ENOMEM;
		goto err_mem;
	}

	memcpy(wsa881x->dai_driver, wsa_dai,
		sizeof(struct snd_soc_dai_driver));

	dev_index = (int)((char)(pdev->addr & 0xF));

	snprintf(buffer, sizeof(buffer), "wsa-codec.%d", dev_index);
	wsa881x->driver->name = kstrndup(buffer,
				       strlen(buffer), GFP_KERNEL);

	snprintf(buffer, sizeof(buffer), "wsa_rx%d", dev_index);
	wsa881x->dai_driver->name =
			kstrndup(buffer, strlen(buffer), GFP_KERNEL);

	snprintf(buffer, sizeof(buffer),
		 "WSA881X_AIF%d Playback", dev_index);
	wsa881x->dai_driver->playback.stream_name =
			kstrndup(buffer, strlen(buffer), GFP_KERNEL);

	/* Number of DAI's used is 1 */
	ret = snd_soc_register_component(&pdev->dev,
				wsa881x->driver, wsa881x->dai_driver, 1);
	if (ret) {
		dev_err(&pdev->dev, "%s: Codec registration failed\n",
			__func__);
		goto err_mem;
	}

	wsa881x->wsa881x_name_prefix = kstrndup(wsa881x_name_prefix_of,
		strlen(wsa881x_name_prefix_of), GFP_KERNEL);

	component = snd_soc_lookup_component(&pdev->dev, wsa881x->driver->name);
	if (!component) {
		dev_err(&pdev->dev, "%s: component is NULL \n", __func__);
		ret = -EINVAL;
		goto err_mem;
	}

	component->name_prefix = wsa881x->wsa881x_name_prefix;

	wsa881x->bolero_np = of_parse_phandle(pdev->dev.of_node,
					      "qcom,bolero-handle", 0);
	if (wsa881x->bolero_np) {
		wsa881x->bolero_dev =
				of_find_device_by_node(wsa881x->bolero_np);
		if (wsa881x->bolero_dev) {
			plat_data = dev_get_platdata(&wsa881x->bolero_dev->dev);
			if (plat_data) {
				wsa881x->bolero_nblock.notifier_call =
							wsa881x_event_notify;
				if (plat_data->register_notifier)
					plat_data->register_notifier(
						plat_data->handle,
						&wsa881x->bolero_nblock,
						true);
				wsa881x->register_notifier =
						plat_data->register_notifier;
				wsa881x->handle = plat_data->handle;
			} else {
				dev_err(&pdev->dev, "%s: plat data not found\n",
					__func__);
			}
		} else {
			dev_err(&pdev->dev, "%s: bolero dev not found\n",
				__func__);
		}
	} else {
		dev_info(&pdev->dev, "%s: bolero node not found\n", __func__);
	}

	mutex_init(&wsa881x->res_lock);
	mutex_init(&wsa881x->temp_lock);

	return 0;

err_mem:
	kfree(wsa881x->wsa881x_name_prefix);
	if (wsa881x->dai_driver) {
		devm_kfree(&pdev->dev, wsa881x->dai_driver->name);
		devm_kfree(&pdev->dev, wsa881x->dai_driver->playback.stream_name);
		devm_kfree(&pdev->dev, wsa881x->dai_driver);
	}
	if (wsa881x->driver) {
		devm_kfree(&pdev->dev, wsa881x->driver->name);
		devm_kfree(&pdev->dev, wsa881x->driver);
	}
dev_err:
	if (pin_state_current == false)
		wsa881x_gpio_ctrl(wsa881x, false);
	swr_remove_device(pdev);
err:
	return ret;
}

static int wsa881x_swr_remove(struct swr_device *pdev)
{
	struct wsa881x_priv *wsa881x;

	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x) {
		dev_err(&pdev->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}

	if (wsa881x->register_notifier)
		wsa881x->register_notifier(wsa881x->handle,
					   &wsa881x->bolero_nblock, false);
	debugfs_remove_recursive(wsa881x->debugfs_dent);
	wsa881x->debugfs_dent = NULL;
	mutex_destroy(&wsa881x->res_lock);
	mutex_destroy(&wsa881x->temp_lock);
	snd_soc_unregister_component(&pdev->dev);
	if (wsa881x->pd_gpio)
		gpio_free(wsa881x->pd_gpio);
	swr_set_dev_data(pdev, NULL);
	kfree(wsa881x->wsa881x_name_prefix);
	if (wsa881x->dai_driver) {
		devm_kfree(&pdev->dev, wsa881x->dai_driver->name);
		devm_kfree(&pdev->dev, wsa881x->dai_driver->playback.stream_name);
		devm_kfree(&pdev->dev, wsa881x->dai_driver);
	}
	if (wsa881x->driver) {
		devm_kfree(&pdev->dev, wsa881x->driver->name);
		devm_kfree(&pdev->dev, wsa881x->driver);
	}
	return 0;
}

static int wsa881x_swr_up(struct swr_device *pdev)
{
	int ret;
	struct wsa881x_priv *wsa881x;

	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x) {
		dev_err(&pdev->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}
	ret = wsa881x_gpio_ctrl(wsa881x, true);
	if (ret)
		dev_err(&pdev->dev, "%s: Failed to enable gpio\n", __func__);
	else
		wsa881x->state = WSA881X_DEV_UP;

	return ret;
}

static int wsa881x_swr_down(struct swr_device *pdev)
{
	struct wsa881x_priv *wsa881x;
	int ret;

	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x) {
		dev_err(&pdev->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}
	ret = wsa881x_gpio_ctrl(wsa881x, false);
	if (ret)
		dev_err(&pdev->dev, "%s: Failed to disable gpio\n", __func__);
	else
		wsa881x->state = WSA881X_DEV_DOWN;

	return ret;
}

static int wsa881x_swr_reset(struct swr_device *pdev)
{
	struct wsa881x_priv *wsa881x;
	u8 retry = WSA881X_NUM_RETRY;
	u8 devnum = 0;

	wsa881x = swr_get_dev_data(pdev);
	if (!wsa881x) {
		dev_err(&pdev->dev, "%s: wsa881x is NULL\n", __func__);
		return -EINVAL;
	}
	if (wsa881x->state == WSA881X_DEV_READY) {
		dev_dbg(&pdev->dev, "%s: device already active\n", __func__);
		return 0;
	}

	wsa881x->bg_cnt = 0;
	wsa881x->clk_cnt = 0;
	while (swr_get_logical_dev_num(pdev, pdev->addr, &devnum) && retry--) {
		/* Retry after 1 msec delay */
		usleep_range(1000, 1100);
	}
	pdev->dev_num = devnum;
	wsa881x_regcache_sync(wsa881x);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wsa881x_swr_suspend(struct device *dev)
{
	dev_dbg(dev, "%s: system suspend\n", __func__);
	return 0;
}

static int wsa881x_swr_resume(struct device *dev)
{
	struct wsa881x_priv *wsa881x = swr_get_dev_data(to_swr_device(dev));

	if (!wsa881x) {
		dev_err(dev, "%s: wsa881x private data is NULL\n", __func__);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: system resume\n", __func__);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops wsa881x_swr_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(wsa881x_swr_suspend, wsa881x_swr_resume)
};

static const struct swr_device_id wsa881x_swr_id[] = {
	{"wsa881x", 0},
	{}
};

static const struct of_device_id wsa881x_swr_dt_match[] = {
	{
		.compatible = "qcom,wsa881x",
	},
	{}
};

static struct swr_driver wsa881x_codec_driver = {
	.driver = {
		.name = "wsa881x",
		.owner = THIS_MODULE,
		.pm = &wsa881x_swr_pm_ops,
		.of_match_table = wsa881x_swr_dt_match,
	},
	.probe = wsa881x_swr_probe,
	.remove = wsa881x_swr_remove,
	.id_table = wsa881x_swr_id,
	.device_up = wsa881x_swr_up,
	.device_down = wsa881x_swr_down,
	.reset_device = wsa881x_swr_reset,
};

static int __init wsa881x_codec_init(void)
{
	return swr_driver_register(&wsa881x_codec_driver);
}

static void __exit wsa881x_codec_exit(void)
{
	swr_driver_unregister(&wsa881x_codec_driver);
}

module_init(wsa881x_codec_init);
module_exit(wsa881x_codec_exit);

MODULE_DESCRIPTION("WSA881x Codec driver");
MODULE_LICENSE("GPL v2");
