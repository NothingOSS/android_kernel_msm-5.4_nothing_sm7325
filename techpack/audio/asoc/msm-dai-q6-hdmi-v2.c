// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2012-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/q6afe-v2.h>
#include "msm-dai-q6-v2.h"

#define HDMI_RX_CA_MAX 0x32

enum {
	DP_CONTROLLER0 = 0,
	DP_CONTROLLER1,
	DP_CONTROLLER_MAX,
};

enum {
	DP_STREAM0 = 0,
	DP_STREAM1,
	DP_STREAM_MAX,
};

enum {
	STATUS_PORT_STARTED, /* track if AFE port has started */
	STATUS_MAX
};

struct msm_ext_disp_ca {
	bool set_ca;
	u32 ca;
};

struct msm_dai_q6_hdmi_dai_data {
	DECLARE_BITMAP(status_mask, STATUS_MAX);
	u32 rate;
	u32 channels;
	u32 stream_idx;
	u32 ctl_idx;
	struct msm_ext_disp_ca ca;
	union afe_port_config port_config;
};

static int get_port_id(int dai_id)
{
	/* Currently, display devices share a common AFE port */
	if (dai_id != HDMI_RX || dai_id != HDMI_RX_MS)
		return DISPLAY_PORT_RX;

	return dai_id;
}

static int msm_dai_q6_ext_disp_format_put(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data = kcontrol->private_data;
	int value = ucontrol->value.integer.value[0];

	if (!dai_data) {
		pr_err("%s: dai_data is NULL\n", __func__);
		return -EINVAL;
	}

	dai_data->port_config.hdmi_multi_ch.datatype = value;
	pr_debug("%s: value = %d\n", __func__, value);

	return 0;
}

static int msm_dai_q6_ext_disp_format_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: dai_data is NULL\n", __func__);
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] =
		dai_data->port_config.hdmi_multi_ch.datatype;
	pr_debug("%s: value = %ld\n",
		 __func__, ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_dai_q6_ext_disp_device_idx_put(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: dai_data is NULL\n", __func__);
		return -EINVAL;
	}

	if ((ucontrol->value.integer.value[0] > (DP_CONTROLLER_MAX - 1)) ||
		(ucontrol->value.integer.value[1] > (DP_STREAM_MAX - 1)) ||
		(ucontrol->value.integer.value[0] < 0) ||
		(ucontrol->value.integer.value[1] < 0)) {
		pr_err("%s: DP control index invalid\n", __func__);
		return -EINVAL;
	}

	dai_data->ctl_idx = ucontrol->value.integer.value[0];
	dai_data->stream_idx = ucontrol->value.integer.value[1];
	pr_debug("%s: DP ctl id %d stream id %d\n", __func__,
		 dai_data->ctl_idx, dai_data->stream_idx);

	return 0;
}

static int msm_dai_q6_ext_disp_device_idx_get(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: dai_data is NULL\n", __func__);
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = dai_data->ctl_idx;
	ucontrol->value.integer.value[1] = dai_data->stream_idx;
	pr_debug("%s: DP ctl id %d stream id %d\n", __func__,
		 dai_data->ctl_idx, dai_data->stream_idx);

	return 0;
}

static int msm_dai_q6_ext_disp_ca_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: dai_data is NULL\n", __func__);
		return -EINVAL;
	}

	dai_data->ca.ca = ucontrol->value.integer.value[0];
	dai_data->ca.set_ca = true;
	pr_debug("%s: ca = %d\n", __func__, dai_data->ca.ca);
	return 0;
}

static int msm_dai_q6_ext_disp_ca_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data = kcontrol->private_data;

	if (!dai_data) {
		pr_err("%s: dai_data is NULL\n", __func__);
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = dai_data->ca.ca;
	pr_debug("%s: ca = %d\n", __func__, dai_data->ca.ca);
	return 0;
}

/* HDMI format field for AFE_PORT_MULTI_CHAN_HDMI_AUDIO_IF_CONFIG command
 *  0: linear PCM
 *  1: non-linear PCM
 */
static const char * const hdmi_format[] = {
	"LPCM",
	"Compr"
};

static const struct soc_enum hdmi_config_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, hdmi_format),
};

static int msm_dai_q6_ext_disp_drift_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = sizeof(struct afe_param_id_dev_timing_stats);

	return 0;
}

static int msm_dai_q6_ext_disp_drift_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = -EINVAL;
	struct afe_param_id_dev_timing_stats timing_stats;
	struct snd_soc_dai *dai = kcontrol->private_data;
	struct msm_dai_q6_hdmi_dai_data *dai_data = dev_get_drvdata(dai->dev);

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		pr_debug("%s:  afe port not started. status_mask = %ld\n",
			__func__, *dai_data->status_mask);
		goto done;
	}

	memset(&timing_stats, 0, sizeof(struct afe_param_id_dev_timing_stats));
	ret = afe_get_av_dev_drift(&timing_stats, get_port_id(dai->id));
	if (ret) {
		pr_err("%s: Error getting AFE Drift for port %d, err=%d\n",
			__func__, get_port_id(dai->id), ret);

		ret = -EINVAL;
		goto done;
	}

	memcpy(ucontrol->value.bytes.data, (void *)&timing_stats,
	       sizeof(struct afe_param_id_dev_timing_stats));
done:
	return ret;
}

static const struct snd_kcontrol_new hdmi_config_controls[] = {
	SOC_ENUM_EXT("HDMI RX Format", hdmi_config_enum[0],
				 msm_dai_q6_ext_disp_format_get,
				 msm_dai_q6_ext_disp_format_put),
	SOC_SINGLE_MULTI_EXT("HDMI RX CA", SND_SOC_NOPM, 0,
				 HDMI_RX_CA_MAX, 0, 1,
				 msm_dai_q6_ext_disp_ca_get,
				 msm_dai_q6_ext_disp_ca_put),
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_PCM,
		.name	= "HDMI DRIFT",
		.info	= msm_dai_q6_ext_disp_drift_info,
		.get	= msm_dai_q6_ext_disp_drift_get,
	},
};

static const struct snd_kcontrol_new display_port_config_controls[] = {
	SOC_ENUM_EXT("Display Port RX Format", hdmi_config_enum[0],
				 msm_dai_q6_ext_disp_format_get,
				 msm_dai_q6_ext_disp_format_put),
	SOC_SINGLE_MULTI_EXT("Display Port RX CA", SND_SOC_NOPM, 0,
				 HDMI_RX_CA_MAX, 0, 1,
				 msm_dai_q6_ext_disp_ca_get,
				 msm_dai_q6_ext_disp_ca_put),
	SOC_SINGLE_MULTI_EXT("Display Port RX DEVICE IDX", SND_SOC_NOPM, 0,
				 1, 0, 2,
				 msm_dai_q6_ext_disp_device_idx_get,
				 msm_dai_q6_ext_disp_device_idx_put),
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_PCM,
		.name	= "DISPLAY_PORT DRIFT",
		.info	= msm_dai_q6_ext_disp_drift_info,
		.get	= msm_dai_q6_ext_disp_drift_get,
	},
	SOC_ENUM_EXT("Display Port1 RX Format", hdmi_config_enum[0],
				 msm_dai_q6_ext_disp_format_get,
				 msm_dai_q6_ext_disp_format_put),
	SOC_SINGLE_MULTI_EXT("Display Port1 RX CA", SND_SOC_NOPM, 0,
				 HDMI_RX_CA_MAX, 0, 1,
				 msm_dai_q6_ext_disp_ca_get,
				 msm_dai_q6_ext_disp_ca_put),
	SOC_SINGLE_MULTI_EXT("Display Port1 RX DEVICE IDX", SND_SOC_NOPM, 0,
				 1, 0, 2,
				 msm_dai_q6_ext_disp_device_idx_get,
				 msm_dai_q6_ext_disp_device_idx_put),
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	= SNDRV_CTL_ELEM_IFACE_PCM,
		.name	= "DISPLAY_PORT1 DRIFT",
		.info	= msm_dai_q6_ext_disp_drift_info,
		.get	= msm_dai_q6_ext_disp_drift_get,
	},
};

/* Current implementation assumes hw_param is called once
 * This may not be the case but what to do when ADM and AFE
 * port are already opened and parameter changes
 */
static int msm_dai_q6_hdmi_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data = dev_get_drvdata(dai->dev);

	dai_data->channels = params_channels(params);
	dai_data->rate = params_rate(params);
	dai_data->port_config.hdmi_multi_ch.reserved = 0;
	dai_data->port_config.hdmi_multi_ch.hdmi_cfg_minor_version = 1;
	dai_data->port_config.hdmi_multi_ch.sample_rate = dai_data->rate;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		dai_data->port_config.hdmi_multi_ch.bit_width = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		dai_data->port_config.hdmi_multi_ch.bit_width = 24;
		break;
	}

	/*refer to HDMI spec CEA-861-E: Table 28 Audio InfoFrame Data Byte 4*/
	switch (dai_data->channels) {
	case 2:
		dai_data->port_config.hdmi_multi_ch.channel_allocation = 0;
		break;
	case 3:
		dai_data->port_config.hdmi_multi_ch.channel_allocation = 0x02;
		break;
	case 4:
		dai_data->port_config.hdmi_multi_ch.channel_allocation = 0x06;
		break;
	case 5:
		dai_data->port_config.hdmi_multi_ch.channel_allocation = 0x0A;
		break;
	case 6:
		dai_data->port_config.hdmi_multi_ch.channel_allocation = 0x0B;
		break;
	case 7:
		dai_data->port_config.hdmi_multi_ch.channel_allocation = 0x12;
		break;
	case 8:
		dai_data->port_config.hdmi_multi_ch.channel_allocation = 0x13;
		break;
	default:
		dev_err(dai->dev, "invalid Channels = %u\n",
				dai_data->channels);
		return -EINVAL;
	}
	dev_dbg(dai->dev, "%s() minor version: %u samplerate: %u bitwidth: %u\n"
		"num_ch = %u channel_allocation = %u datatype = %d\n", __func__,
		dai_data->port_config.hdmi_multi_ch.hdmi_cfg_minor_version,
		dai_data->port_config.hdmi_multi_ch.sample_rate,
		dai_data->port_config.hdmi_multi_ch.bit_width,
		dai_data->channels,
		dai_data->port_config.hdmi_multi_ch.channel_allocation,
		dai_data->port_config.hdmi_multi_ch.datatype);

	return 0;
}


static void msm_dai_q6_hdmi_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int rc = 0;

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		pr_info("%s:  afe port not started. dai_data->status_mask = %ld\n",
		 __func__, *dai_data->status_mask);
		return;
	}

	rc = afe_close(get_port_id(dai->id)); /* can block */
	if (rc < 0)
		dev_err(dai->dev, "fail to close AFE port\n");

	pr_debug("%s: dai_data->status_mask = %ld\n", __func__,
			*dai_data->status_mask);

	clear_bit(STATUS_PORT_STARTED, dai_data->status_mask);
}


static int msm_dai_q6_hdmi_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data = dev_get_drvdata(dai->dev);
	int rc = 0;

	if (dai_data->ca.set_ca)
		dai_data->port_config.hdmi_multi_ch.channel_allocation =
							      dai_data->ca.ca;

	if (!test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		rc = afe_set_display_stream(get_port_id(dai->id), dai_data->stream_idx,
						dai_data->ctl_idx);
		if (rc < 0) {
			dev_err(dai->dev, "fail to set AFE ctl, stream ID params %x\n",
				dai->id);
			if (rc != -EOPNOTSUPP) {
				dev_err(dai->dev, "not starting AFE port\n");
				goto err;
			}
		}

		rc = afe_port_start(get_port_id(dai->id), &dai_data->port_config,
				    dai_data->rate);
		if (rc < 0)
			dev_err(dai->dev, "fail to open AFE port %x\n",
				get_port_id(dai->id));
		else
			set_bit(STATUS_PORT_STARTED,
				dai_data->status_mask);
	}

err:
	return rc;
}

static inline void msm_dai_q6_hdmi_set_dai_id(struct snd_soc_dai *dai)
{
	if (!dai->driver->id) {
		dev_warn(dai->dev, "DAI driver id is not set\n");
		return;
	}
	dai->id = dai->driver->id;
}

static int msm_dai_q6_hdmi_dai_probe(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data;
	const struct snd_kcontrol_new *kcontrol;
	int rc = 0;
	struct snd_soc_dapm_route intercon;
	struct snd_soc_dapm_context *dapm;

	if (!dai || !dai->driver) {
		pr_err("%s: dai or dai->driver is NULL\n", __func__);
		return -EINVAL;
	}
	dai_data = kzalloc(sizeof(struct msm_dai_q6_hdmi_dai_data),
		GFP_KERNEL);

	if (!dai_data) {
		dev_err(dai->dev, "DAI-%d: fail to allocate dai data\n",
		dai->id);
		rc = -ENOMEM;
	} else
		dev_set_drvdata(dai->dev, dai_data);

	msm_dai_q6_hdmi_set_dai_id(dai);

	if (dai->driver->id == HDMI_RX || dai->driver->id == HDMI_RX_MS) {
		kcontrol = &hdmi_config_controls[0];
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(kcontrol, dai_data));

		kcontrol = &hdmi_config_controls[1];
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(kcontrol, dai_data));

		kcontrol = &hdmi_config_controls[2];
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(kcontrol, dai));
	} else if (dai->driver->id == MSM_DISPLAY_PORT) {
		kcontrol = &display_port_config_controls[0];
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(kcontrol, dai_data));

		kcontrol = &display_port_config_controls[1];
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(kcontrol, dai_data));

		kcontrol = &display_port_config_controls[2];
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(kcontrol, dai_data));

		kcontrol = &display_port_config_controls[3];
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(kcontrol, dai));
	} else if (dai->driver->id == MSM_DISPLAY_PORT1) {
		kcontrol = &display_port_config_controls[4];
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(kcontrol, dai_data));

		kcontrol = &display_port_config_controls[5];
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(kcontrol, dai_data));

		kcontrol = &display_port_config_controls[6];
		rc = snd_ctl_add(dai->component->card->snd_card,
				 snd_ctl_new1(kcontrol, dai_data));

		kcontrol = &display_port_config_controls[7];
		rc = snd_ctl_add(dai->component->card->snd_card,
				snd_ctl_new1(kcontrol, dai));
	} else {
		dev_err(dai->dev, "%s: Invalid id:%d\n",
			__func__, dai->driver->id);
		kfree(dai_data);
		dev_set_drvdata(dai->dev, NULL);
		return -EINVAL;
	}

	dapm = snd_soc_component_get_dapm(dai->component);
	memset(&intercon, 0, sizeof(intercon));
	if (!rc) {
		if (dai->driver->playback.stream_name &&
			dai->driver->playback.aif_name) {
			dev_dbg(dai->dev, "%s add route for widget %s",
				   __func__, dai->driver->playback.stream_name);
			intercon.source = dai->driver->playback.aif_name;
			intercon.sink = dai->driver->playback.stream_name;
			dev_dbg(dai->dev, "%s src %s sink %s\n",
				   __func__, intercon.source, intercon.sink);
			snd_soc_dapm_add_routes(dapm, &intercon, 1);
		}
		if (dai->driver->capture.stream_name &&
		   dai->driver->capture.aif_name) {
			dev_dbg(dai->dev, "%s add route for widget %s",
				   __func__, dai->driver->capture.stream_name);
			intercon.sink = dai->driver->capture.aif_name;
			intercon.source = dai->driver->capture.stream_name;
			dev_dbg(dai->dev, "%s src %s sink %s\n",
				   __func__, intercon.source, intercon.sink);
			snd_soc_dapm_add_routes(dapm, &intercon, 1);
		}
	}
	return rc;
}

static int msm_dai_q6_hdmi_dai_remove(struct snd_soc_dai *dai)
{
	struct msm_dai_q6_hdmi_dai_data *dai_data;
	int rc;

	dai_data = dev_get_drvdata(dai->dev);

	/* If AFE port is still up, close it */
	if (test_bit(STATUS_PORT_STARTED, dai_data->status_mask)) {
		rc = afe_close(get_port_id(dai->id)); /* can block */
		if (rc < 0)
			dev_err(dai->dev, "fail to close AFE port\n");

		clear_bit(STATUS_PORT_STARTED, dai_data->status_mask);
	}
	kfree(dai_data);

	return 0;
}

static struct snd_soc_dai_ops msm_dai_q6_hdmi_ops = {
	.prepare	= msm_dai_q6_hdmi_prepare,
	.hw_params	= msm_dai_q6_hdmi_hw_params,
	.shutdown	= msm_dai_q6_hdmi_shutdown,
};

static struct snd_soc_dai_driver msm_dai_q6_hdmi_hdmi_rx_dai = {
	.playback = {
		.stream_name = "HDMI Playback",
		.aif_name = "HDMI",
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			 SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S24_3LE,
		.channels_min = 2,
		.channels_max = 8,
		.rate_max = 192000,
		.rate_min = 48000,
	},
	.ops = &msm_dai_q6_hdmi_ops,
	.id = HDMI_RX,
	.probe = msm_dai_q6_hdmi_dai_probe,
	.remove = msm_dai_q6_hdmi_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_hdmi_hdmi_ms_rx_dai = {
	.playback = {
		.stream_name = "HDMI MS Playback",
		.aif_name = "HDMI_MS",
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			 SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S24_3LE,
		.channels_min = 2,
		.channels_max = 8,
		.rate_max = 192000,
		.rate_min = 48000,
	},
	.ops = &msm_dai_q6_hdmi_ops,
	.id = HDMI_RX_MS,
	.probe = msm_dai_q6_hdmi_dai_probe,
	.remove = msm_dai_q6_hdmi_dai_remove,
};

static struct snd_soc_dai_driver msm_dai_q6_display_port_rx_dai[] = {
	{
		.playback = {
			.stream_name = "Display Port Playback",
			.aif_name = "DISPLAY_PORT",
			.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE,
			.channels_min = 2,
			.channels_max = 8,
			.rate_max =     192000,
			.rate_min =     48000,
		},
		.ops = &msm_dai_q6_hdmi_ops,
		.id = MSM_DISPLAY_PORT,
		.probe = msm_dai_q6_hdmi_dai_probe,
		.remove = msm_dai_q6_hdmi_dai_remove,
	},
	{
		.playback = {
			.stream_name = "Display Port1 Playback",
			.aif_name = "DISPLAY_PORT1",
			.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
				 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
				 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
				 SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE,
			.channels_min = 2,
			.channels_max = 8,
			.rate_max =     192000,
			.rate_min =     48000,
		},
		.ops = &msm_dai_q6_hdmi_ops,
		.id = MSM_DISPLAY_PORT1,
		.probe = msm_dai_q6_hdmi_dai_probe,
		.remove = msm_dai_q6_hdmi_dai_remove,
	},
};

static const struct snd_soc_component_driver msm_dai_hdmi_q6_component = {
	.name		= "msm-dai-q6-hdmi",
};

/* To do: change to register DAIs as batch */
static int msm_dai_q6_hdmi_dev_probe(struct platform_device *pdev)
{
	int rc, id;
	const char *q6_dev_id = "qcom,msm-dai-q6-dev-id";

	rc = of_property_read_u32(pdev->dev.of_node, q6_dev_id, &id);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: missing %s in dt node\n", __func__, q6_dev_id);
		return rc;
	}

	pdev->id = id;

	pr_debug("%s: dev name %s, id:%d\n", __func__,
			dev_name(&pdev->dev), pdev->id);

	switch (pdev->id) {
	case HDMI_RX:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_hdmi_q6_component,
			&msm_dai_q6_hdmi_hdmi_rx_dai, 1);
		break;
	case MSM_DISPLAY_PORT:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_hdmi_q6_component,
			&msm_dai_q6_display_port_rx_dai[0], 1);
		break;
	case MSM_DISPLAY_PORT1:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_hdmi_q6_component,
			&msm_dai_q6_display_port_rx_dai[1], 1);
		break;
	case HDMI_RX_MS:
		rc = snd_soc_register_component(&pdev->dev,
			&msm_dai_hdmi_q6_component,
			&msm_dai_q6_hdmi_hdmi_ms_rx_dai, 1);
		break;

	default:
		dev_err(&pdev->dev, "invalid device ID %d\n", pdev->id);
		rc = -ENODEV;
		break;
	}
	return rc;
}

static int msm_dai_q6_hdmi_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_dai_q6_hdmi_dt_match[] = {
	{.compatible = "qcom,msm-dai-q6-hdmi"},
	{}
};
MODULE_DEVICE_TABLE(of, msm_dai_q6_hdmi_dt_match);

static struct platform_driver msm_dai_q6_hdmi_driver = {
	.probe  = msm_dai_q6_hdmi_dev_probe,
	.remove = msm_dai_q6_hdmi_dev_remove,
	.driver = {
		.name = "msm-dai-q6-hdmi",
		.owner = THIS_MODULE,
		.of_match_table = msm_dai_q6_hdmi_dt_match,
		.suppress_bind_attrs = true,
	},
};

int __init msm_dai_q6_hdmi_init(void)
{
	return platform_driver_register(&msm_dai_q6_hdmi_driver);
}

void msm_dai_q6_hdmi_exit(void)
{
	platform_driver_unregister(&msm_dai_q6_hdmi_driver);
}

/* Module information */
MODULE_DESCRIPTION("MSM DSP HDMI DAI driver");
MODULE_LICENSE("GPL v2");
