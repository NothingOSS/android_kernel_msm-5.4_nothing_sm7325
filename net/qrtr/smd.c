// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015, Sony Mobile Communications Inc.
 * Copyright (c) 2013, 2018-2019 The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/rpmsg.h>
#include <linux/of.h>

#include "qrtr.h"

struct qrtr_smd_dev {
	struct qrtr_endpoint ep;
	struct rpmsg_endpoint *channel;
	struct device *dev;
};

/* from smd to qrtr */
static int qcom_smd_qrtr_callback(struct rpmsg_device *rpdev,
				  void *data, int len, void *priv, u32 addr)
{
	struct qrtr_smd_dev *qdev = dev_get_drvdata(&rpdev->dev);
	int rc;

	if (!qdev) {
		pr_err("%s:Not ready\n", __func__);
		return -EAGAIN;
	}

	rc = qrtr_endpoint_post(&qdev->ep, data, len);
	if (rc == -EINVAL) {
		dev_err(qdev->dev, "invalid ipcrouter packet\n");
		/* return 0 to let smd drop the packet */
		rc = 0;
	}

	return rc;
}

/* from qrtr to smd */
static int qcom_smd_qrtr_send(struct qrtr_endpoint *ep, struct sk_buff *skb)
{
	struct qrtr_smd_dev *qdev = container_of(ep, struct qrtr_smd_dev, ep);
	int rc;

	rc = skb_linearize(skb);
	if (rc)
		goto out;

	rc = rpmsg_send(qdev->channel, skb->data, skb->len);

out:
	if (rc)
		kfree_skb(skb);
	else
		consume_skb(skb);
	return rc;
}

static int qcom_smd_qrtr_probe(struct rpmsg_device *rpdev)
{
	struct qrtr_smd_dev *qdev;
	u32 net_id;
	bool rt;
	int rc, size;
	u32 *svc_arr = NULL;
	pr_info("%s:Entered\n", __func__);

	qdev = devm_kzalloc(&rpdev->dev, sizeof(*qdev), GFP_KERNEL);
	if (!qdev)
		return -ENOMEM;

	qdev->channel = rpdev->ept;
	qdev->dev = &rpdev->dev;
	qdev->ep.xmit = qcom_smd_qrtr_send;

	rc = of_property_read_u32(rpdev->dev.of_node, "qcom,net-id", &net_id);
	if (rc < 0)
		net_id = QRTR_EP_NET_ID_AUTO;

	rt = of_property_read_bool(rpdev->dev.of_node, "qcom,low-latency");

	size = of_property_count_u32_elems(rpdev->dev.of_node, "qcom,non-wake-svc");

	if (size > 0) {
		if (size > MAX_NON_WAKE_SVC_LEN)
			size = MAX_NON_WAKE_SVC_LEN;
		svc_arr = kmalloc_array(size, sizeof(u32), GFP_KERNEL);
		if (!svc_arr) {
			devm_kfree(&rpdev->dev, qdev);
			return -ENOMEM;
		}

		of_property_read_u32_array(rpdev->dev.of_node, "qcom,non-wake-svc",
					   svc_arr, size);
	}
	rc = qrtr_endpoint_register(&qdev->ep, net_id, rt, svc_arr, size);
	kfree(svc_arr);
	if (rc) {
		devm_kfree(&rpdev->dev, qdev);
		return rc;
	}

	dev_set_drvdata(&rpdev->dev, qdev);

	pr_info("%s:SMD QRTR driver probed\n", __func__);

	return 0;
}

static void qcom_smd_qrtr_remove(struct rpmsg_device *rpdev)
{
	struct qrtr_smd_dev *qdev = dev_get_drvdata(&rpdev->dev);

	qrtr_endpoint_unregister(&qdev->ep);

	dev_set_drvdata(&rpdev->dev, NULL);
}

static const struct rpmsg_device_id qcom_smd_qrtr_smd_match[] = {
	{ "IPCRTR" },
	{}
};

static struct rpmsg_driver qcom_smd_qrtr_driver = {
	.probe = qcom_smd_qrtr_probe,
	.remove = qcom_smd_qrtr_remove,
	.callback = qcom_smd_qrtr_callback,
	.id_table = qcom_smd_qrtr_smd_match,
	.drv = {
		.name = "qcom_smd_qrtr",
	},
};

module_rpmsg_driver(qcom_smd_qrtr_driver);

MODULE_ALIAS("rpmsg:IPCRTR");
MODULE_DESCRIPTION("Qualcomm IPC-Router SMD interface driver");
MODULE_LICENSE("GPL v2");
