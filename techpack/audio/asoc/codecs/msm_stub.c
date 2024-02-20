// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2011-2014, 2017-2019 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#define DRV_NAME "msm-stub-codec"

/* A dummy driver useful only to advertise hardware parameters */
static struct snd_soc_dai_driver msm_stub_dais[] = {
	{
		.name = "msm-stub-rx",
		.playback = { /* Support maximum range */
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 32,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
		},
	},
	{
		.name = "msm-stub-tx",
		.capture = { /* Support maximum range */
			.stream_name = "Record",
			.channels_min = 1,
			.channels_max = 32,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
				    SNDRV_PCM_FMTBIT_S24_LE),
		},
	},
};

static const struct snd_soc_component_driver soc_msm_stub = {
	.name = DRV_NAME,
};

static int msm_stub_dev_probe(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "dev name %s\n", dev_name(&pdev->dev));

	return snd_soc_register_component(&pdev->dev,
	&soc_msm_stub, msm_stub_dais, ARRAY_SIZE(msm_stub_dais));
}

static int msm_stub_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}
static const struct of_device_id msm_stub_codec_dt_match[] = {
	{ .compatible = "qcom,msm-stub-codec", },
	{}
};

static struct platform_driver msm_stub_driver = {
	.driver = {
		.name = "msm-stub-codec",
		.owner = THIS_MODULE,
		.of_match_table = msm_stub_codec_dt_match,
		.suppress_bind_attrs = true,
	},
	.probe = msm_stub_dev_probe,
	.remove = msm_stub_dev_remove,
};

static int __init msm_stub_init(void)
{
	return platform_driver_register(&msm_stub_driver);
}
module_init(msm_stub_init);

static void __exit msm_stub_exit(void)
{
	platform_driver_unregister(&msm_stub_driver);
}
module_exit(msm_stub_exit);

MODULE_DESCRIPTION("Generic MSM CODEC driver");
MODULE_LICENSE("GPL v2");
