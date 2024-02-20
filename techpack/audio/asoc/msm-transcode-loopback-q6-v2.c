// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <audio/sound/audio_effects.h>
#include <sound/pcm_params.h>
#include <sound/timer.h>
#include <sound/tlv.h>
#include <sound/compress_params.h>
#include <sound/compress_offload.h>
#include <sound/compress_driver.h>
#include <dsp/msm_audio_ion.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/q6asm-v2.h>
#include <dsp/q6audio-v2.h>
#include <dsp/msm-audio-effects-q6-v2.h>

#include "msm-pcm-routing-v2.h"
#include "msm-qti-pp-config.h"

#define DRV_NAME "msm-transcode-loopback-v2"

#define LOOPBACK_SESSION_MAX_NUM_STREAMS 2
/* Max volume corresponding to 24dB */
#define TRANSCODE_LR_VOL_MAX_DB 0xFFFF

#define APP_TYPE_CONFIG_IDX_APP_TYPE 0
#define APP_TYPE_CONFIG_IDX_ACDB_ID 1
#define APP_TYPE_CONFIG_IDX_SAMPLE_RATE 2
#define APP_TYPE_CONFIG_IDX_BE_ID 3

struct msm_transcode_audio_effects {
	struct bass_boost_params bass_boost;
	struct pbe_params pbe;
	struct virtualizer_params virtualizer;
	struct reverb_params reverb;
	struct eq_params equalizer;
	struct soft_volume_params volume;
};

struct trans_loopback_pdata {
	struct snd_compr_stream *cstream[MSM_FRONTEND_DAI_MAX];
	uint32_t master_gain;
	int perf_mode[MSM_FRONTEND_DAI_MAX];
	struct msm_transcode_audio_effects *audio_effects[MSM_FRONTEND_DAI_MAX];
};

struct loopback_stream {
	struct snd_compr_stream *cstream;
	uint32_t codec_format;
	bool start;
	int perf_mode;
};

enum loopback_session_state {
	/* One or both streams not opened */
	LOOPBACK_SESSION_CLOSE = 0,
	/* Loopback streams opened */
	LOOPBACK_SESSION_READY,
	/* Loopback streams opened and formats configured */
	LOOPBACK_SESSION_START,
	/* Trigger issued on either of streams when in START state */
	LOOPBACK_SESSION_RUN
};

struct msm_transcode_loopback {
	struct loopback_stream source;
	struct loopback_stream sink;

	struct snd_compr_caps source_compr_cap;
	struct snd_compr_caps sink_compr_cap;

	uint32_t instance;
	uint32_t num_streams;
	int session_state;

	struct mutex lock;

	int session_id;
	struct audio_client *audio_client;
};

/* Transcode loopback global info struct */
static struct msm_transcode_loopback transcode_info;

static void loopback_event_handler(uint32_t opcode,
		uint32_t token, uint32_t *payload, void *priv)
{
	struct msm_transcode_loopback *trans =
			(struct msm_transcode_loopback *)priv;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_compr_stream *cstream;
	struct audio_client *ac;
	int stream_id;
	int ret;

	if (!trans || !payload) {
		pr_err("%s: rtd or payload is NULL\n", __func__);
		return;
	}

	cstream = trans->sink.cstream;
	ac = trans->audio_client;

	/*
	 * Token for rest of the compressed commands use to set
	 * session id, stream id, dir etc.
	 */
	stream_id = q6asm_get_stream_id_from_token(token);

	switch (opcode) {
	case ASM_STREAM_CMD_ENCDEC_EVENTS:
	case ASM_IEC_61937_MEDIA_FMT_EVENT:
		pr_debug("%s: Handling stream event : 0X%x\n",
			__func__, opcode);
		rtd = cstream->private_data;
		if (!rtd) {
			pr_err("%s: rtd is NULL\n", __func__);
			return;
		}

		ret = msm_adsp_inform_mixer_ctl(rtd, payload);
		if (ret) {
			pr_err("%s: failed to inform mixer ctrl. err = %d\n",
				__func__, ret);
			return;
		}
		break;
	case APR_BASIC_RSP_RESULT: {
		switch (payload[0]) {
		case ASM_SESSION_CMD_RUN_V2:
			pr_debug("%s: ASM_SESSION_CMD_RUN_V2:", __func__);
			pr_debug("token 0x%x, stream id %d\n", token,
				  stream_id);
			break;
		case ASM_STREAM_CMD_CLOSE:
			pr_debug("%s: ASM_DATA_CMD_CLOSE:", __func__);
			pr_debug("token 0x%x, stream id %d\n", token,
				  stream_id);
			break;
		default:
			break;
		}
		break;
	}
	default:
		pr_debug("%s: Not Supported Event opcode[0x%x]\n",
			  __func__, opcode);
		break;
	}
}

static void populate_codec_list(struct msm_transcode_loopback *trans,
				struct snd_compr_stream *cstream)
{
	struct snd_compr_caps compr_cap;

	pr_debug("%s\n", __func__);

	memset(&compr_cap, 0, sizeof(struct snd_compr_caps));

	if (cstream->direction == SND_COMPRESS_CAPTURE) {
		compr_cap.direction = SND_COMPRESS_CAPTURE;
		compr_cap.num_codecs = 4;
		compr_cap.codecs[0] = SND_AUDIOCODEC_PCM;
		compr_cap.codecs[1] = SND_AUDIOCODEC_AC3;
		compr_cap.codecs[2] = SND_AUDIOCODEC_EAC3;
		compr_cap.codecs[3] = SND_AUDIOCODEC_TRUEHD;
		memcpy(&trans->source_compr_cap, &compr_cap,
				sizeof(struct snd_compr_caps));
	}

	if (cstream->direction == SND_COMPRESS_PLAYBACK) {
		compr_cap.direction = SND_COMPRESS_PLAYBACK;
		compr_cap.num_codecs = 1;
		compr_cap.codecs[0] = SND_AUDIOCODEC_PCM;
		memcpy(&trans->sink_compr_cap, &compr_cap,
				sizeof(struct snd_compr_caps));
	}
}

static int msm_transcode_loopback_open(struct snd_compr_stream *cstream)
{
	int ret = 0;
	struct snd_compr_runtime *runtime;
	struct snd_soc_pcm_runtime *rtd;
	struct msm_transcode_loopback *trans = &transcode_info;
	struct trans_loopback_pdata *pdata;
	struct snd_soc_component *component;

	if (cstream == NULL) {
		pr_err("%s: Invalid substream\n", __func__);
		return -EINVAL;
	}
	runtime = cstream->runtime;
	rtd = snd_pcm_substream_chip(cstream);
	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	pdata = snd_soc_component_get_drvdata(component);
	pdata->cstream[rtd->dai_link->id] = cstream;
	pdata->audio_effects[rtd->dai_link->id] =
				kzalloc(sizeof(struct msm_transcode_audio_effects), GFP_KERNEL);

	if (pdata->audio_effects[rtd->dai_link->id] == NULL) {
		ret = -ENOMEM;
		goto effect_error;
	}

	mutex_lock(&trans->lock);
	if (trans->num_streams > LOOPBACK_SESSION_MAX_NUM_STREAMS) {
		pr_err("msm_transcode_open failed..invalid stream\n");
		ret = -EINVAL;
		goto exit;
	}

	if (cstream->direction == SND_COMPRESS_CAPTURE) {
		if (trans->source.cstream == NULL) {
			trans->source.cstream = cstream;
			trans->num_streams++;
		} else {
			pr_err("%s: capture stream already opened\n",
				__func__);
			ret = -EINVAL;
			goto exit;
		}
	} else if (cstream->direction == SND_COMPRESS_PLAYBACK) {
		if (trans->sink.cstream == NULL) {
			trans->sink.cstream = cstream;
			trans->num_streams++;
		} else {
			pr_debug("%s: playback stream already opened\n",
				__func__);
			ret = -EINVAL;
			goto exit;
		}
		msm_adsp_init_mixer_ctl_pp_event_queue(rtd);
	}

	pr_debug("%s: num stream%d, stream name %s\n", __func__,
		 trans->num_streams, cstream->name);

	populate_codec_list(trans, cstream);

	if (trans->num_streams == LOOPBACK_SESSION_MAX_NUM_STREAMS)	{
		pr_debug("%s: Moving loopback session to READY state %d\n",
			 __func__, trans->session_state);
		trans->session_state = LOOPBACK_SESSION_READY;
	}

	runtime->private_data = trans;

exit:
	mutex_unlock(&trans->lock);
	if ((pdata->audio_effects[rtd->dai_link->id] != NULL) && (ret < 0)) {
		kfree(pdata->audio_effects[rtd->dai_link->id]);
		pdata->audio_effects[rtd->dai_link->id] = NULL;
	}
effect_error:
	return ret;
}

static void stop_transcoding(struct msm_transcode_loopback *trans)
{
	struct snd_soc_pcm_runtime *soc_pcm_rx;
	struct snd_soc_pcm_runtime *soc_pcm_tx;

	if (trans->audio_client != NULL) {
		q6asm_cmd(trans->audio_client, CMD_CLOSE);

		if (trans->sink.cstream != NULL) {
			soc_pcm_rx = trans->sink.cstream->private_data;
			msm_pcm_routing_dereg_phy_stream(
					soc_pcm_rx->dai_link->id,
					SND_COMPRESS_PLAYBACK);
		}
		if (trans->source.cstream != NULL) {
			soc_pcm_tx = trans->source.cstream->private_data;
			msm_pcm_routing_dereg_phy_stream(
					soc_pcm_tx->dai_link->id,
					SND_COMPRESS_CAPTURE);
		}
		q6asm_audio_client_free(trans->audio_client);
		trans->audio_client = NULL;
	}
}

static int msm_transcode_loopback_free(struct snd_compr_stream *cstream)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_transcode_loopback *trans = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(cstream);
	struct snd_soc_component *component;
	struct trans_loopback_pdata *pdata;
	int ret = 0;

	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	pdata = snd_soc_component_get_drvdata(component);

	mutex_lock(&trans->lock);

	if (pdata->audio_effects[rtd->dai_link->id] != NULL) {
		kfree(pdata->audio_effects[rtd->dai_link->id]);
		pdata->audio_effects[rtd->dai_link->id] = NULL;
	}

	pr_debug("%s: Transcode loopback end:%d, streams %d\n", __func__,
		  cstream->direction, trans->num_streams);
	trans->num_streams--;
	stop_transcoding(trans);

	if (cstream->direction == SND_COMPRESS_PLAYBACK) {
		memset(&trans->sink, 0, sizeof(struct loopback_stream));
		msm_adsp_clean_mixer_ctl_pp_event_queue(rtd);
	} else if (cstream->direction == SND_COMPRESS_CAPTURE) {
		memset(&trans->source, 0, sizeof(struct loopback_stream));
	}

	trans->session_state = LOOPBACK_SESSION_CLOSE;
	mutex_unlock(&trans->lock);
	return ret;
}

static int msm_transcode_loopback_trigger(struct snd_compr_stream *cstream,
					  int cmd)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_transcode_loopback *trans = runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:

		if (trans->session_state == LOOPBACK_SESSION_START) {
			pr_debug("%s: Issue Loopback session %d RUN\n",
				  __func__, trans->instance);
			q6asm_run_nowait(trans->audio_client, 0, 0, 0);
			trans->session_state = LOOPBACK_SESSION_RUN;
		}
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_STOP:
		pr_debug("%s: Issue Loopback session %d STOP\n", __func__,
			  trans->instance);
		if (trans->session_state == LOOPBACK_SESSION_RUN)
			q6asm_cmd_nowait(trans->audio_client, CMD_PAUSE);
		trans->session_state = LOOPBACK_SESSION_START;
		break;

	default:
		break;
	}
	return 0;
}

static int msm_transcode_set_render_window(struct audio_client *ac,
					uint32_t ws_lsw, uint32_t ws_msw,
					uint32_t we_lsw, uint32_t we_msw)
{
	int ret = -EINVAL;
	struct asm_session_mtmx_strtr_param_window_v2_t asm_mtmx_strtr_window;
	uint32_t param_id;

	pr_debug("%s, ws_lsw 0x%x ws_msw 0x%x we_lsw 0x%x we_msw 0x%x\n",
		__func__, ws_lsw, ws_msw, we_lsw, we_msw);

	memset(&asm_mtmx_strtr_window, 0,
		sizeof(struct asm_session_mtmx_strtr_param_window_v2_t));
	asm_mtmx_strtr_window.window_lsw = ws_lsw;
	asm_mtmx_strtr_window.window_msw = ws_msw;
	param_id = ASM_SESSION_MTMX_STRTR_PARAM_RENDER_WINDOW_START_V2;
	ret = q6asm_send_mtmx_strtr_window(ac, &asm_mtmx_strtr_window, param_id);
	if (ret) {
		pr_err("%s, start window can't be set error %d\n", __func__, ret);
		goto exit;
	}

	asm_mtmx_strtr_window.window_lsw = we_lsw;
	asm_mtmx_strtr_window.window_msw = we_msw;
	param_id = ASM_SESSION_MTMX_STRTR_PARAM_RENDER_WINDOW_END_V2;
	ret = q6asm_send_mtmx_strtr_window(ac, &asm_mtmx_strtr_window, param_id);
	if (ret)
		pr_err("%s, end window can't be set error %d\n", __func__, ret);

exit:
	return ret;
}

static int msm_transcode_loopback_set_params(struct snd_compr_stream *cstream,
				struct snd_compr_params *codec_param)
{

	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_transcode_loopback *trans = runtime->private_data;
	struct snd_soc_pcm_runtime *soc_pcm_rx;
	struct snd_soc_pcm_runtime *soc_pcm_tx;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_component *component;
	struct trans_loopback_pdata *pdata;
	uint32_t bit_width = 16;
	int ret = 0;
	enum apr_subsys_state subsys_state;

	if (trans == NULL) {
		pr_err("%s: Invalid param\n", __func__);
		return -EINVAL;
	}

	subsys_state = apr_get_subsys_state();
	if (subsys_state == APR_SUBSYS_DOWN) {
		pr_debug("%s: adsp is down\n", __func__);
		return -ENETRESET;
	}

	mutex_lock(&trans->lock);

	rtd = snd_pcm_substream_chip(cstream);
	if (!rtd) {
		pr_err("%s: rtd is NULL\n", __func__);
		return -EINVAL;
	}
	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}
	pdata = snd_soc_component_get_drvdata(component);

	if (cstream->direction == SND_COMPRESS_PLAYBACK) {
		if (codec_param->codec.id == SND_AUDIOCODEC_PCM) {
			trans->sink.codec_format =
				FORMAT_LINEAR_PCM;
			switch (codec_param->codec.format) {
			case SNDRV_PCM_FORMAT_S32_LE:
				bit_width = 32;
				break;
			case SNDRV_PCM_FORMAT_S24_LE:
				bit_width = 24;
				break;
			case SNDRV_PCM_FORMAT_S24_3LE:
				bit_width = 24;
				break;
			case SNDRV_PCM_FORMAT_S16_LE:
			default:
				bit_width = 16;
				break;
			}
		} else {
			pr_debug("%s: unknown sink codec\n", __func__);
			ret = -EINVAL;
			goto exit;
		}
		trans->sink.start = true;
		trans->sink.perf_mode = pdata->perf_mode[rtd->dai_link->id];
	}

	if (cstream->direction == SND_COMPRESS_CAPTURE) {
		switch (codec_param->codec.id) {
		case SND_AUDIOCODEC_PCM:
			pr_debug("Source SND_AUDIOCODEC_PCM\n");
			trans->source.codec_format =
				FORMAT_LINEAR_PCM;
			break;
		case SND_AUDIOCODEC_AC3:
			pr_debug("Source SND_AUDIOCODEC_AC3\n");
			trans->source.codec_format =
				FORMAT_AC3;
			break;
		case SND_AUDIOCODEC_EAC3:
			pr_debug("Source SND_AUDIOCODEC_EAC3\n");
			trans->source.codec_format =
				FORMAT_EAC3;
			break;
		case SND_AUDIOCODEC_TRUEHD:
			pr_debug("Source SND_AUDIOCODEC_TRUEHD\n");
			trans->source.codec_format =
				FORMAT_TRUEHD;
			break;
		default:
			pr_debug("%s: unknown source codec\n", __func__);
			ret = -EINVAL;
			goto exit;
		}
		trans->source.start = true;
		trans->source.perf_mode = pdata->perf_mode[rtd->dai_link->id];
	}

	pr_debug("%s: trans->source.start %d trans->sink.start %d trans->source.cstream %pK trans->sink.cstream %pK trans->session_state %d\n",
			__func__, trans->source.start, trans->sink.start,
			trans->source.cstream, trans->sink.cstream,
			trans->session_state);

	if ((trans->session_state == LOOPBACK_SESSION_READY) &&
			trans->source.start && trans->sink.start) {
		pr_debug("%s: Moving loopback session to start state\n",
			  __func__);
		trans->session_state = LOOPBACK_SESSION_START;
	}

	if (trans->session_state == LOOPBACK_SESSION_START) {
		if (trans->audio_client != NULL) {
			pr_debug("%s: ASM client already opened, closing\n",
				 __func__);
			stop_transcoding(trans);
		}

		trans->audio_client = q6asm_audio_client_alloc(
				(app_cb)loopback_event_handler, trans);
		if (!trans->audio_client) {
			pr_err("%s: Could not allocate memory\n", __func__);
			ret = -EINVAL;
			goto exit;
		}
		pr_debug("%s: ASM client allocated, callback %pK\n", __func__,
						loopback_event_handler);
		trans->session_id = trans->audio_client->session;
		trans->audio_client->perf_mode = trans->sink.perf_mode;
		trans->audio_client->fedai_id = rtd->dai_link->id;
		ret = q6asm_open_transcode_loopback(trans->audio_client,
					bit_width,
					trans->source.codec_format,
					trans->sink.codec_format);
		if (ret < 0) {
			pr_err("%s: Session transcode loopback open failed\n",
				__func__);
			q6asm_audio_client_free(trans->audio_client);
			trans->audio_client = NULL;
			goto exit;
		}

		pr_debug("%s: Starting ADM open for loopback\n", __func__);
		soc_pcm_rx = trans->sink.cstream->private_data;
		soc_pcm_tx = trans->source.cstream->private_data;
		if (trans->source.codec_format != FORMAT_LINEAR_PCM)
			msm_pcm_routing_reg_phy_compr_stream(
					soc_pcm_tx->dai_link->id,
					LEGACY_PCM_MODE,
					trans->session_id,
					SNDRV_PCM_STREAM_CAPTURE,
					COMPRESSED_PASSTHROUGH_GEN);
		else
			msm_pcm_routing_reg_phy_stream(
					soc_pcm_tx->dai_link->id,
					trans->source.perf_mode,
					trans->session_id,
					SNDRV_PCM_STREAM_CAPTURE);
		/* Opening Rx ADM in LOW_LATENCY mode by default */
		msm_pcm_routing_reg_phy_stream(
					soc_pcm_rx->dai_link->id,
					trans->sink.perf_mode,
					trans->session_id,
					SNDRV_PCM_STREAM_PLAYBACK);
		pr_debug("%s: Successfully opened ADM sessions\n", __func__);
	}
exit:
	mutex_unlock(&trans->lock);
	return ret;
}

static int msm_transcode_loopback_get_caps(struct snd_compr_stream *cstream,
				struct snd_compr_caps *arg)
{
	struct snd_compr_runtime *runtime;
	struct msm_transcode_loopback *trans;

	if (!arg || !cstream) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -EINVAL;
	}

	runtime = cstream->runtime;
	trans = runtime->private_data;
	pr_debug("%s\n", __func__);
	if (cstream->direction == SND_COMPRESS_CAPTURE)
		memcpy(arg, &trans->source_compr_cap,
		       sizeof(struct snd_compr_caps));
	else
		memcpy(arg, &trans->sink_compr_cap,
		       sizeof(struct snd_compr_caps));
	return 0;
}

static int msm_transcode_loopback_set_metadata(struct snd_compr_stream *cstream,
				struct snd_compr_metadata *metadata)
{
	struct snd_soc_pcm_runtime *rtd;
	struct trans_loopback_pdata *pdata;
	struct msm_transcode_loopback *prtd = NULL;
	struct snd_soc_component *component;
	struct audio_client *ac = NULL;

	if (!metadata || !cstream) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -EINVAL;
	}

	rtd = snd_pcm_substream_chip(cstream);
	if (!rtd) {
		pr_err("%s: rtd is NULL\n", __func__);
		return -EINVAL;
	}
	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}
	pdata = snd_soc_component_get_drvdata(component);

	prtd = cstream->runtime->private_data;

	if (!prtd || !prtd->audio_client) {
		pr_err("%s: prtd or audio client is NULL\n", __func__);
		return -EINVAL;
	}

	ac = prtd->audio_client;

	switch (metadata->key) {
	case SNDRV_COMPRESS_LATENCY_MODE:
	{
		switch (metadata->value[0]) {
		case SNDRV_COMPRESS_LEGACY_LATENCY_MODE:
			pdata->perf_mode[rtd->dai_link->id] = LEGACY_PCM_MODE;
			break;
		case SNDRV_COMPRESS_LOW_LATENCY_MODE:
			pdata->perf_mode[rtd->dai_link->id] =
					LOW_LATENCY_PCM_MODE;
			break;
		default:
			pr_debug("%s: Unsupported latency mode %d, default to Legacy\n",
					__func__, metadata->value[0]);
			pdata->perf_mode[rtd->dai_link->id] = LEGACY_PCM_MODE;
			break;
		}
		break;
	}
	case SNDRV_COMPRESS_RENDER_WINDOW:
	{
		return msm_transcode_set_render_window(
						ac,
						metadata->value[0],
						metadata->value[1],
						metadata->value[2],
						metadata->value[3]);
	}
	default:
		pr_debug("%s: Unsupported metadata %d\n",
				__func__, metadata->key);
		break;
	}
	return 0;
}

static int msm_transcode_stream_cmd_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;
	struct trans_loopback_pdata *pdata = (struct trans_loopback_pdata *)
				snd_soc_component_get_drvdata(comp);
	struct snd_compr_stream *cstream = NULL;
	struct msm_transcode_loopback *prtd;
	int ret = 0;
	struct msm_adsp_event_data *event_data = NULL;

	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received invalid fe_id %lu\n",
			__func__, fe_id);
		ret = -EINVAL;
		goto done;
	}

	cstream = pdata->cstream[fe_id];
	if (cstream == NULL) {
		pr_err("%s cstream is null.\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	prtd = cstream->runtime->private_data;
	if (!prtd) {
		pr_err("%s: prtd is null.\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	if (prtd->audio_client == NULL) {
		pr_err("%s: audio_client is null.\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	event_data = (struct msm_adsp_event_data *)ucontrol->value.bytes.data;
	if ((event_data->event_type < ADSP_STREAM_PP_EVENT) ||
	    (event_data->event_type >= ADSP_STREAM_EVENT_MAX)) {
		pr_err("%s: invalid event_type=%d",
			 __func__, event_data->event_type);
		ret = -EINVAL;
		goto done;
	}

	if (event_data->payload_len > sizeof(ucontrol->value.bytes.data)
			- sizeof(struct msm_adsp_event_data)) {
		pr_err("%s param length=%d  exceeds limit",
			 __func__, event_data->payload_len);
		ret = -EINVAL;
		goto done;
	}

	ret = q6asm_send_stream_cmd(prtd->audio_client, event_data);
	if (ret < 0)
		pr_err("%s: failed to send stream event cmd, err = %d\n",
			__func__, ret);
done:
	return ret;
}

static int msm_transcode_ion_fd_map_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;
	struct trans_loopback_pdata *pdata = (struct trans_loopback_pdata *)
				snd_soc_component_get_drvdata(comp);
	struct snd_compr_stream *cstream = NULL;
	struct msm_transcode_loopback *prtd;
	int fd;
	int ret = 0;

	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received out of bounds invalid fe_id %lu\n",
			__func__, fe_id);
		ret = -EINVAL;
		goto done;
	}

	cstream = pdata->cstream[fe_id];
	if (cstream == NULL) {
		pr_err("%s cstream is null\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	prtd = cstream->runtime->private_data;
	if (!prtd) {
		pr_err("%s: prtd is null\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	if (prtd->audio_client == NULL) {
		pr_err("%s: audio_client is null\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	memcpy(&fd, ucontrol->value.bytes.data, sizeof(fd));
	ret = q6asm_send_ion_fd(prtd->audio_client, fd);
	if (ret < 0)
		pr_err("%s: failed to register ion fd\n", __func__);
done:
	return ret;
}

static int msm_transcode_rtic_event_ack_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;
	struct trans_loopback_pdata *pdata = (struct trans_loopback_pdata *)
					snd_soc_component_get_drvdata(comp);
	struct snd_compr_stream *cstream = NULL;
	struct msm_transcode_loopback *prtd;
	int ret = 0;
	int param_length = 0;

	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received invalid fe_id %lu\n",
			__func__, fe_id);
		ret = -EINVAL;
		goto done;
	}

	cstream = pdata->cstream[fe_id];
	if (cstream == NULL) {
		pr_err("%s cstream is null\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	prtd = cstream->runtime->private_data;
	if (!prtd) {
		pr_err("%s: prtd is null\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	if (prtd->audio_client == NULL) {
		pr_err("%s: audio_client is null\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	memcpy(&param_length, ucontrol->value.bytes.data,
		sizeof(param_length));
	if ((param_length + sizeof(param_length))
		>= sizeof(ucontrol->value.bytes.data)) {
		pr_err("%s param length=%d  exceeds limit",
			__func__, param_length);
		ret = -EINVAL;
		goto done;
	}

	ret = q6asm_send_rtic_event_ack(prtd->audio_client,
			ucontrol->value.bytes.data + sizeof(param_length),
			param_length);
	if (ret < 0)
		pr_err("%s: failed to send rtic event ack, err = %d\n",
			__func__, ret);
done:
	return ret;
}

static int msm_transcode_playback_app_type_cfg_put(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	u64 fe_id = kcontrol->private_value;
	int session_type = SESSION_TYPE_RX;
	int be_id = ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_BE_ID];
	struct msm_pcm_stream_app_type_cfg cfg_data = {0, 0, 48000};
	int ret = 0;

	cfg_data.app_type = ucontrol->value.integer.value[
			    APP_TYPE_CONFIG_IDX_APP_TYPE];
	cfg_data.acdb_dev_id = ucontrol->value.integer.value[
			       APP_TYPE_CONFIG_IDX_ACDB_ID];
	if (ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_SAMPLE_RATE] != 0)
		cfg_data.sample_rate = ucontrol->value.integer.value[
				       APP_TYPE_CONFIG_IDX_SAMPLE_RATE];
	pr_debug("%s: fe_id %llu session_type %d be_id %d app_type %d acdb_dev_id %d sample_rate- %d\n",
		__func__, fe_id, session_type, be_id,
		cfg_data.app_type, cfg_data.acdb_dev_id, cfg_data.sample_rate);
	ret = msm_pcm_routing_reg_stream_app_type_cfg(fe_id, session_type,
						      be_id, &cfg_data);
	if (ret < 0)
		pr_err("%s: msm_transcode_playback_stream_app_type_cfg set failed returned %d\n",
			__func__, ret);

	return ret;
}

static int msm_transcode_playback_app_type_cfg_get(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	u64 fe_id = kcontrol->private_value;
	int session_type = SESSION_TYPE_RX;
	int be_id = 0;
	struct msm_pcm_stream_app_type_cfg cfg_data = {0};
	int ret = 0;

	ret = msm_pcm_routing_get_stream_app_type_cfg(fe_id, session_type,
						      &be_id, &cfg_data);
	if (ret < 0) {
		pr_err("%s: msm_transcode_playback_stream_app_type_cfg get failed returned %d\n",
			__func__, ret);
		goto done;
	}

	ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_APP_TYPE] =
					cfg_data.app_type;
	ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_ACDB_ID] =
					cfg_data.acdb_dev_id;
	ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_SAMPLE_RATE] =
					cfg_data.sample_rate;
	ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_BE_ID] = be_id;
	pr_debug("%s: fedai_id %llu, session_type %d, be_id %d, app_type %d, acdb_dev_id %d, sample_rate %d\n",
		__func__, fe_id, session_type, be_id,
		cfg_data.app_type, cfg_data.acdb_dev_id, cfg_data.sample_rate);
done:
	return ret;
}

static int msm_transcode_capture_app_type_cfg_put(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	u64 fe_id = kcontrol->private_value;
	int session_type = SESSION_TYPE_TX;
	int be_id = ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_BE_ID];
	struct msm_pcm_stream_app_type_cfg cfg_data = {0, 0, 48000};
	int ret = 0;

	cfg_data.app_type = ucontrol->value.integer.value[
			    APP_TYPE_CONFIG_IDX_APP_TYPE];
	cfg_data.acdb_dev_id = ucontrol->value.integer.value[
			       APP_TYPE_CONFIG_IDX_ACDB_ID];
	if (ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_SAMPLE_RATE] != 0)
		cfg_data.sample_rate = ucontrol->value.integer.value[
				       APP_TYPE_CONFIG_IDX_SAMPLE_RATE];
	pr_debug("%s: fe_id %llu session_type %d be_id %d app_type %d acdb_dev_id %d sample_rate- %d\n",
		__func__, fe_id, session_type, be_id,
		cfg_data.app_type, cfg_data.acdb_dev_id, cfg_data.sample_rate);
	ret = msm_pcm_routing_reg_stream_app_type_cfg(fe_id, session_type,
						      be_id, &cfg_data);
	if (ret < 0)
		pr_err("%s: register stream app type cfg failed, returned %d\n",
			__func__, ret);

	return ret;
}

static int msm_transcode_capture_app_type_cfg_get(
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	u64 fe_id = kcontrol->private_value;
	int session_type = SESSION_TYPE_TX;
	int be_id = 0;
	struct msm_pcm_stream_app_type_cfg cfg_data = {0};
	int ret = 0;

	ret = msm_pcm_routing_get_stream_app_type_cfg(fe_id, session_type,
						      &be_id, &cfg_data);
	if (ret < 0) {
		pr_err("%s: get stream app type cfg failed, returned %d\n",
			__func__, ret);
		goto done;
	}

	ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_APP_TYPE] =
					cfg_data.app_type;
	ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_ACDB_ID] =
					cfg_data.acdb_dev_id;
	ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_SAMPLE_RATE] =
					cfg_data.sample_rate;
	ucontrol->value.integer.value[APP_TYPE_CONFIG_IDX_BE_ID] = be_id;
	pr_debug("%s: fedai_id %llu, session_type %d, be_id %d, app_type %d, acdb_dev_id %d, sample_rate %d\n",
		__func__, fe_id, session_type, be_id,
		cfg_data.app_type, cfg_data.acdb_dev_id, cfg_data.sample_rate);
done:
	return ret;
}

static int msm_transcode_set_volume(struct snd_compr_stream *cstream,
				uint32_t master_gain)
{
	int rc = 0;
	struct msm_transcode_loopback *prtd;
	struct snd_soc_pcm_runtime *rtd;

	pr_debug("%s: master_gain %d\n", __func__, master_gain);
	if (!cstream || !cstream->runtime) {
		pr_err("%s: session not active\n", __func__);
		return -EINVAL;
	}
	rtd = cstream->private_data;
	prtd = cstream->runtime->private_data;

	if (!rtd || !prtd || !prtd->audio_client) {
		pr_err("%s: invalid rtd, prtd or audio client", __func__);
		return -EINVAL;
	}

	rc = q6asm_set_volume(prtd->audio_client, master_gain);
	if (rc < 0)
		pr_err("%s: Send vol gain command failed rc=%d\n",
		       __func__, rc);

	return rc;
}

static int msm_transcode_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;
	struct trans_loopback_pdata *pdata = (struct trans_loopback_pdata *)
			snd_soc_component_get_drvdata(comp);
	struct snd_compr_stream *cstream = NULL;
	uint32_t ret = 0;

	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received out of bounds fe_id %lu\n",
			__func__, fe_id);
		return -EINVAL;
	}

	cstream = pdata->cstream[fe_id];
	pdata->master_gain = ucontrol->value.integer.value[0];

	pr_debug("%s: fe_id %lu master_gain %d\n",
		 __func__, fe_id, pdata->master_gain);
	if (cstream)
		ret = msm_transcode_set_volume(cstream, pdata->master_gain);
	return ret;
}

static int msm_transcode_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;

	struct trans_loopback_pdata *pdata = (struct trans_loopback_pdata *)
			snd_soc_component_get_drvdata(comp);

	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received out of bound fe_id %lu\n", __func__, fe_id);
		return -EINVAL;
	}

	pr_debug("%s: fe_id %lu\n", __func__, fe_id);
	ucontrol->value.integer.value[0] = pdata->master_gain;

	return 0;
}

static int msm_transcode_audio_effects_config_info(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = MAX_PP_PARAMS_SZ;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xFFFFFFFF;
	return 0;
}

static int msm_transcode_audio_effects_config_get(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;
	struct trans_loopback_pdata *pdata = (struct trans_loopback_pdata *)
					      snd_soc_component_get_drvdata(comp);
	struct msm_transcode_audio_effects *audio_effects = NULL;
	struct snd_compr_stream *cstream = NULL;

	pr_debug("%s: fe_id: %lu\n", __func__, fe_id);
	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received out of bounds fe_id %lu\n",
			__func__, fe_id);
		return -EINVAL;
	}
	cstream = pdata->cstream[fe_id];
	audio_effects = pdata->audio_effects[fe_id];
	if (!cstream || !audio_effects) {
		pr_err("%s: stream or effects inactive\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int msm_transcode_audio_effects_config_put(struct snd_kcontrol *kcontrol,
						struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;
	struct trans_loopback_pdata *pdata = (struct trans_loopback_pdata *)
					      snd_soc_component_get_drvdata(comp);
	struct msm_transcode_audio_effects *audio_effects = NULL;
	struct snd_compr_stream *cstream = NULL;
	struct msm_transcode_loopback *prtd = NULL;
	long *values = &(ucontrol->value.integer.value[0]);
	int effects_module;
	int ret = 0;

	pr_debug("%s: fe_id: %lu\n", __func__, fe_id);
	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received out of bounds fe_id %lu\n",
			__func__, fe_id);
		ret = -EINVAL;
		goto exit;
	}
	cstream = pdata->cstream[fe_id];
	audio_effects = pdata->audio_effects[fe_id];
	if (!cstream || !audio_effects) {
		pr_err("%s: stream or effects inactive\n", __func__);
		ret = -EINVAL;
		goto exit;
	}
	prtd = cstream->runtime->private_data;
	if (!prtd) {
		pr_err("%s: cannot set audio effects\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	effects_module = *values++;
	switch (effects_module) {
	case VIRTUALIZER_MODULE:
		pr_debug("%s: VIRTUALIZER_MODULE\n", __func__);
		if (msm_audio_effects_is_effmodule_supp_in_top(effects_module,
						prtd->audio_client->topology))
			ret = msm_audio_effects_virtualizer_handler(
							prtd->audio_client,
							&(audio_effects->virtualizer),
							values);
		break;
	case REVERB_MODULE:
		pr_debug("%s: REVERB_MODULE\n", __func__);
		if (msm_audio_effects_is_effmodule_supp_in_top(effects_module,
						prtd->audio_client->topology))
			ret = msm_audio_effects_reverb_handler(prtd->audio_client,
							&(audio_effects->reverb),
							values);
		break;
	case BASS_BOOST_MODULE:
		pr_debug("%s: BASS_BOOST_MODULE\n", __func__);
		if (msm_audio_effects_is_effmodule_supp_in_top(effects_module,
						prtd->audio_client->topology))
			ret = msm_audio_effects_bass_boost_handler(prtd->audio_client,
							&(audio_effects->bass_boost),
							values);
		break;
	case PBE_MODULE:
		pr_debug("%s: PBE_MODULE\n", __func__);
		if (msm_audio_effects_is_effmodule_supp_in_top(effects_module,
						prtd->audio_client->topology))
			ret = msm_audio_effects_pbe_handler(prtd->audio_client,
							&(audio_effects->pbe),
							values);
		break;
	case EQ_MODULE:
		pr_debug("%s: EQ_MODULE\n", __func__);
		if (msm_audio_effects_is_effmodule_supp_in_top(effects_module,
						prtd->audio_client->topology))
			ret = msm_audio_effects_popless_eq_handler(prtd->audio_client,
							&(audio_effects->equalizer),
							values);
		break;
	case SOFT_VOLUME_MODULE:
		pr_debug("%s: SOFT_VOLUME_MODULE\n", __func__);
		break;
	case SOFT_VOLUME2_MODULE:
		pr_debug("%s: SOFT_VOLUME2_MODULE\n", __func__);
		if (msm_audio_effects_is_effmodule_supp_in_top(effects_module,
						prtd->audio_client->topology))
			ret = msm_audio_effects_volume_handler_v2(prtd->audio_client,
							&(audio_effects->volume),
							values, SOFT_VOLUME_INSTANCE_2);
		break;
	default:
		pr_err("%s Invalid effects config module\n", __func__);
		ret = -EINVAL;
	}

exit:
	return ret;
}

static int msm_transcode_add_audio_effects_control(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = NULL;
	const char *mixer_ctl_name = "Audio Effects Config";
	const char *deviceNo       = "NN";
	char *mixer_str = NULL;
	int ctl_len = 0;
	int ret = 0;
	struct snd_kcontrol_new fe_audio_effects_config_control[1] = {
		{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "?",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = msm_transcode_audio_effects_config_info,
		.get = msm_transcode_audio_effects_config_get,
		.put = msm_transcode_audio_effects_config_put,
		.private_value = 0,
		}
	};

	if (!rtd) {
		pr_err("%s NULL rtd\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: added new compr FE with name %s, id %d, cpu dai %s, device no %d\n", __func__,
						rtd->dai_link->name, rtd->dai_link->id,
						rtd->dai_link->cpus->dai_name, rtd->pcm->device);

	ctl_len = strlen(mixer_ctl_name) + 1 + strlen(deviceNo) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);

	if (!mixer_str) {
		ret = -ENOMEM;
		goto done;
	}

	snprintf(mixer_str, ctl_len, "%s %d", mixer_ctl_name, rtd->pcm->device);

	fe_audio_effects_config_control[0].name = mixer_str;
	fe_audio_effects_config_control[0].private_value = rtd->dai_link->id;
	ret = snd_soc_add_component_controls(component,
					    fe_audio_effects_config_control,
					    ARRAY_SIZE(fe_audio_effects_config_control));
	if (ret < 0)
		pr_err("%s: failed to add ctl %s. err = %d\n", __func__, mixer_str, ret);

	kfree(mixer_str);
done:
	return ret;
}

static int msm_transcode_stream_cmd_control(
			struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = NULL;
	const char *mixer_ctl_name = DSP_STREAM_CMD;
	const char *deviceNo = "NN";
	char *mixer_str = NULL;
	int ctl_len = 0, ret = 0;
	struct snd_kcontrol_new fe_loopback_stream_cmd_config_control[1] = {
		{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "?",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = msm_adsp_stream_cmd_info,
		.put = msm_transcode_stream_cmd_put,
		.private_value = 0,
		}
	};

	if (!rtd) {
		pr_err("%s NULL rtd\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	ctl_len = strlen(mixer_ctl_name) + 1 + strlen(deviceNo) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);
	if (!mixer_str) {
		ret = -ENOMEM;
		goto done;
	}

	snprintf(mixer_str, ctl_len, "%s %d", mixer_ctl_name, rtd->pcm->device);
	fe_loopback_stream_cmd_config_control[0].name = mixer_str;
	fe_loopback_stream_cmd_config_control[0].private_value =
				rtd->dai_link->id;
	pr_debug("%s: Registering new mixer ctl %s\n", __func__, mixer_str);
	ret = snd_soc_add_component_controls(component,
		fe_loopback_stream_cmd_config_control,
		ARRAY_SIZE(fe_loopback_stream_cmd_config_control));
	if (ret < 0)
		pr_err("%s: failed to add ctl %s. err = %d\n",
			__func__, mixer_str, ret);

	kfree(mixer_str);
done:
	return ret;
}

static int msm_transcode_stream_callback_control(
			struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = NULL;
	const char *mixer_ctl_name = DSP_STREAM_CALLBACK;
	const char *deviceNo = "NN";
	char *mixer_str = NULL;
	int ctl_len = 0, ret = 0;
	struct snd_kcontrol *kctl;

	struct snd_kcontrol_new fe_loopback_callback_config_control[1] = {
		{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "?",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = msm_adsp_stream_callback_info,
		.get = msm_adsp_stream_callback_get,
		.private_value = 0,
		}
	};

	if (!rtd) {
		pr_err("%s: rtd is  NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	ctl_len = strlen(mixer_ctl_name) + 1 + strlen(deviceNo) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);
	if (!mixer_str) {
		ret = -ENOMEM;
		goto done;
	}

	snprintf(mixer_str, ctl_len, "%s %d", mixer_ctl_name, rtd->pcm->device);
	fe_loopback_callback_config_control[0].name = mixer_str;
	fe_loopback_callback_config_control[0].private_value =
					rtd->dai_link->id;
	pr_debug("%s: Registering new mixer ctl %s\n", __func__, mixer_str);
	ret = snd_soc_add_component_controls(component,
			fe_loopback_callback_config_control,
			ARRAY_SIZE(fe_loopback_callback_config_control));
	if (ret < 0) {
		pr_err("%s: failed to add ctl %s. err = %d\n",
			__func__, mixer_str, ret);
		ret = -EINVAL;
		goto free_mixer_str;
	}

	kctl = snd_soc_card_get_kcontrol(rtd->card, mixer_str);
	if (!kctl) {
		pr_err("%s: failed to get kctl %s.\n", __func__, mixer_str);
		ret = -EINVAL;
		goto free_mixer_str;
	}

	kctl->private_data = NULL;
free_mixer_str:
	kfree(mixer_str);
done:
	return ret;
}

static int msm_transcode_add_ion_fd_cmd_control(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = NULL;
	const char *mixer_ctl_name = "Playback ION FD";
	const char *deviceNo = "NN";
	char *mixer_str = NULL;
	int ctl_len = 0, ret = 0;
	struct snd_kcontrol_new fe_ion_fd_config_control[1] = {
		{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "?",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = msm_adsp_stream_cmd_info,
		.put = msm_transcode_ion_fd_map_put,
		.private_value = 0,
		}
	};

	if (!rtd) {
		pr_err("%s NULL rtd\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	ctl_len = strlen(mixer_ctl_name) + 1 + strlen(deviceNo) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);
	if (!mixer_str) {
		ret = -ENOMEM;
		goto done;
	}

	snprintf(mixer_str, ctl_len, "%s %d", mixer_ctl_name, rtd->pcm->device);
	fe_ion_fd_config_control[0].name = mixer_str;
	fe_ion_fd_config_control[0].private_value = rtd->dai_link->id;
	pr_debug("%s: Registering new mixer ctl %s\n", __func__, mixer_str);
	ret = snd_soc_add_component_controls(component,
				fe_ion_fd_config_control,
				ARRAY_SIZE(fe_ion_fd_config_control));
	if (ret < 0)
		pr_err("%s: failed to add ctl %s\n", __func__, mixer_str);

	kfree(mixer_str);
done:
	return ret;
}

static int msm_transcode_add_event_ack_cmd_control(
					struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = NULL;
	const char *mixer_ctl_name = "Playback Event Ack";
	const char *deviceNo = "NN";
	char *mixer_str = NULL;
	int ctl_len = 0, ret = 0;
	struct snd_kcontrol_new fe_event_ack_config_control[1] = {
		{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "?",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = msm_adsp_stream_cmd_info,
		.put = msm_transcode_rtic_event_ack_put,
		.private_value = 0,
		}
	};

	if (!rtd) {
		pr_err("%s NULL rtd\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	ctl_len = strlen(mixer_ctl_name) + 1 + strlen(deviceNo) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);
	if (!mixer_str) {
		ret = -ENOMEM;
		goto done;
	}

	snprintf(mixer_str, ctl_len, "%s %d", mixer_ctl_name, rtd->pcm->device);
	fe_event_ack_config_control[0].name = mixer_str;
	fe_event_ack_config_control[0].private_value = rtd->dai_link->id;
	pr_debug("%s: Registering new mixer ctl %s\n", __func__, mixer_str);
	ret = snd_soc_add_component_controls(component,
				fe_event_ack_config_control,
				ARRAY_SIZE(fe_event_ack_config_control));
	if (ret < 0)
		pr_err("%s: failed to add ctl %s\n", __func__, mixer_str);

	kfree(mixer_str);
done:
	return ret;
}

static int msm_transcode_app_type_cfg_info(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 5;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xFFFFFFFF;
	return 0;
}

static int msm_transcode_add_app_type_cfg_control(
			struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = NULL;
	char mixer_str[128];
	struct snd_kcontrol_new fe_app_type_cfg_control[1] = {
		{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = msm_transcode_app_type_cfg_info,
		.private_value = 0,
		}
	};

	if (!rtd) {
		pr_err("%s NULL rtd\n", __func__);
		return -EINVAL;
	}

	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	if (rtd->compr->direction == SND_COMPRESS_PLAYBACK) {
		snprintf(mixer_str, sizeof(mixer_str),
			"Audio Stream %d App Type Cfg",
			 rtd->pcm->device);

		fe_app_type_cfg_control[0].name = mixer_str;
		fe_app_type_cfg_control[0].private_value = rtd->dai_link->id;

		fe_app_type_cfg_control[0].put =
				msm_transcode_playback_app_type_cfg_put;
		fe_app_type_cfg_control[0].get =
				msm_transcode_playback_app_type_cfg_get;

		pr_debug("Registering new mixer ctl %s", mixer_str);
		snd_soc_add_component_controls(component,
					fe_app_type_cfg_control,
					ARRAY_SIZE(fe_app_type_cfg_control));
	} else if (rtd->compr->direction == SND_COMPRESS_CAPTURE) {
		snprintf(mixer_str, sizeof(mixer_str),
			"Audio Stream Capture %d App Type Cfg",
			 rtd->pcm->device);

		fe_app_type_cfg_control[0].name = mixer_str;
		fe_app_type_cfg_control[0].private_value = rtd->dai_link->id;

		fe_app_type_cfg_control[0].put =
				msm_transcode_capture_app_type_cfg_put;
		fe_app_type_cfg_control[0].get =
				msm_transcode_capture_app_type_cfg_get;

		pr_debug("Registering new mixer ctl %s", mixer_str);
		snd_soc_add_component_controls(component,
					fe_app_type_cfg_control,
					ARRAY_SIZE(fe_app_type_cfg_control));
	}

	return 0;
}

static int msm_transcode_volume_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = TRANSCODE_LR_VOL_MAX_DB;
	return 0;
}

static int msm_transcode_add_volume_control(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = NULL;
	struct snd_kcontrol_new fe_volume_control[1] = {
		{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Transcode Loopback Rx Volume",
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			  SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = msm_transcode_volume_info,
		.get = msm_transcode_volume_get,
		.put = msm_transcode_volume_put,
		.private_value = 0,
		}
	};

	if (!rtd) {
		pr_err("%s NULL rtd\n", __func__);
		return -EINVAL;
	}

	component = snd_soc_rtdcom_lookup(rtd, DRV_NAME);
	if (!component) {
		pr_err("%s: component is NULL\n", __func__);
		return -EINVAL;
	}

	if (rtd->compr->direction == SND_COMPRESS_PLAYBACK) {
		fe_volume_control[0].private_value = rtd->dai_link->id;
		pr_debug("Registering new mixer ctl %s",
			     fe_volume_control[0].name);
		snd_soc_add_component_controls(component, fe_volume_control,
						ARRAY_SIZE(fe_volume_control));
	}
	return 0;
}

static int msm_transcode_loopback_new(struct snd_soc_pcm_runtime *rtd)
{
	int rc;

	rc = msm_transcode_add_audio_effects_control(rtd);
	if (rc)
		pr_err("%s: Could not add Compr Audio Effects Control\n",
			__func__);

	rc = msm_transcode_stream_cmd_control(rtd);
	if (rc)
		pr_err("%s: ADSP Stream Cmd Control open failed\n", __func__);

	rc = msm_transcode_stream_callback_control(rtd);
	if (rc)
		pr_err("%s: ADSP Stream callback Control open failed\n",
			__func__);

	rc = msm_transcode_add_ion_fd_cmd_control(rtd);
	if (rc)
		pr_err("%s: Could not add transcode ion fd Control\n",
			__func__);

	rc = msm_transcode_add_event_ack_cmd_control(rtd);
	if (rc)
		pr_err("%s: Could not add transcode event ack Control\n",
			__func__);

	rc = msm_transcode_add_app_type_cfg_control(rtd);
	if (rc)
		pr_err("%s: Could not add Compr App Type Cfg Control\n",
			__func__);

	rc = msm_transcode_add_volume_control(rtd);
	if (rc)
		pr_err("%s: Could not add transcode volume Control\n",
			__func__);

	return 0;
}

static struct snd_compr_ops msm_transcode_loopback_ops = {
	.open			= msm_transcode_loopback_open,
	.free			= msm_transcode_loopback_free,
	.trigger		= msm_transcode_loopback_trigger,
	.set_params		= msm_transcode_loopback_set_params,
	.get_caps		= msm_transcode_loopback_get_caps,
	.set_metadata		= msm_transcode_loopback_set_metadata,
};


static int msm_transcode_loopback_probe(struct snd_soc_component *component)
{
	struct trans_loopback_pdata *pdata = NULL;
	int i;

	pr_debug("%s\n", __func__);
	pdata = (struct trans_loopback_pdata *)
			kzalloc(sizeof(struct trans_loopback_pdata),
			GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	for (i = 0; i < MSM_FRONTEND_DAI_MAX; i++) {
		pdata->audio_effects[i] = NULL;
		pdata->perf_mode[i] = LOW_LATENCY_PCM_MODE;
	}

	snd_soc_component_set_drvdata(component, pdata);
	return 0;
}

static void msm_transcode_loopback_remove(struct snd_soc_component *component)
{
	struct trans_loopback_pdata *pdata = NULL;

	pdata = (struct trans_loopback_pdata *)
			snd_soc_component_get_drvdata(component);
	kfree(pdata);
	return;
}

static struct snd_soc_component_driver msm_soc_component = {
	.name		= DRV_NAME,
	.probe		= msm_transcode_loopback_probe,
	.compr_ops	= &msm_transcode_loopback_ops,
	.pcm_new	= msm_transcode_loopback_new,
	.remove		= msm_transcode_loopback_remove,
};

static int msm_transcode_dev_probe(struct platform_device *pdev)
{
	pr_debug("%s: dev name %s\n", __func__, dev_name(&pdev->dev));

	return snd_soc_register_component(&pdev->dev,
					&msm_soc_component,
					NULL, 0);
}

static int msm_transcode_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_transcode_loopback_dt_match[] = {
	{.compatible = "qcom,msm-transcode-loopback"},
	{}
};
MODULE_DEVICE_TABLE(of, msm_transcode_loopback_dt_match);

static struct platform_driver msm_transcode_loopback_driver = {
	.driver = {
		.name = "msm-transcode-loopback",
		.owner = THIS_MODULE,
		.of_match_table = msm_transcode_loopback_dt_match,
		.suppress_bind_attrs = true,
	},
	.probe = msm_transcode_dev_probe,
	.remove = msm_transcode_remove,
};

int __init msm_transcode_loopback_init(void)
{
	memset(&transcode_info, 0, sizeof(struct msm_transcode_loopback));
	mutex_init(&transcode_info.lock);
	return platform_driver_register(&msm_transcode_loopback_driver);
}

void msm_transcode_loopback_exit(void)
{
	mutex_destroy(&transcode_info.lock);
	platform_driver_unregister(&msm_transcode_loopback_driver);
}

MODULE_DESCRIPTION("Transcode loopback platform driver");
MODULE_LICENSE("GPL v2");
