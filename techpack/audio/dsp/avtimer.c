// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2015, 2017-2021 The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <audio/linux/avtimer.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/wait.h>
#include <linux/sched.h>
#if IS_ENABLED(CONFIG_AVTIMER_LEGACY)
#include <media/msmb_isp.h>
#endif
#include <ipc/apr.h>
#include <dsp/q6core.h>

#define DEVICE_NAME "avtimer"
#define TIMEOUT_MS 1000
#define CORE_CLIENT 1
#define TEMP_PORT ((CORE_CLIENT << 8) | 0x0001)
#define SSR_WAKETIME 1000
#define Q6_READY_RETRY 250
#define Q6_READY_MAX_RETRIES 40

#define AVCS_CMD_REMOTE_AVTIMER_VOTE_REQUEST 0x00012914
#define AVCS_CMD_RSP_REMOTE_AVTIMER_VOTE_REQUEST 0x00012915
#define AVCS_CMD_REMOTE_AVTIMER_RELEASE_REQUEST 0x00012916
#define AVTIMER_REG_CNT 2

struct adsp_avt_timer {
	struct apr_hdr hdr;
	union {
		char client_name[8];
		u32 avtimer_handle;
	};
} __packed;

static int major;

struct avtimer_t {
	struct apr_svc *core_handle_q;
	struct cdev myc;
	struct class *avtimer_class;
	struct mutex avtimer_lock;
	int avtimer_open_cnt;
	struct delayed_work ssr_dwork;
	wait_queue_head_t adsp_resp_wait;
	int enable_timer_resp_received;
	int timer_handle;
	void __iomem *p_avtimer_msw;
	void __iomem *p_avtimer_lsw;
	uint32_t clk_div;
	uint32_t clk_mult;
	atomic_t adsp_ready;
	int num_retries;
};

static struct avtimer_t avtimer;
static void avcs_set_isp_fptr(bool enable);

static int32_t aprv2_core_fn_q(struct apr_client_data *data, void *priv)
{
	uint32_t *payload1;

	if (!data) {
		pr_err("%s: Invalid params\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: core msg: payload len = %u, apr resp opcode = 0x%X\n",
		__func__, data->payload_size, data->opcode);

	switch (data->opcode) {

	case APR_BASIC_RSP_RESULT:{

		if (!data->payload_size) {
			pr_err("%s: APR_BASIC_RSP_RESULT No Payload ",
					__func__);
			return 0;
		}

		payload1 = data->payload;

		if (data->payload_size < 2 * sizeof(uint32_t)) {
			pr_err("%s: payload has invalid size %d\n",
				__func__, data->payload_size);
			return -EINVAL;
		}

		switch (payload1[0]) {
		case AVCS_CMD_REMOTE_AVTIMER_RELEASE_REQUEST:
			pr_debug("%s: Cmd = TIMER RELEASE status[0x%x]\n",
			__func__, payload1[1]);
			break;
		default:
			pr_err("Invalid cmd rsp[0x%x][0x%x]\n",
					payload1[0], payload1[1]);
			break;
		}
		break;
	}

	case RESET_EVENTS:{
		pr_debug("%s: Reset event received in AV timer\n", __func__);
		apr_reset(avtimer.core_handle_q);
		avtimer.core_handle_q = NULL;
		avtimer.avtimer_open_cnt = 0;
		atomic_set(&avtimer.adsp_ready, 0);
		schedule_delayed_work(&avtimer.ssr_dwork,
				  msecs_to_jiffies(SSR_WAKETIME));
		break;
	}

	case AVCS_CMD_RSP_REMOTE_AVTIMER_VOTE_REQUEST:
		if (data->payload_size < sizeof(uint32_t)) {
			pr_err("%s: payload has invalid size %d\n",
				__func__, data->payload_size);
			return -EINVAL;
		}
		payload1 = data->payload;
		pr_debug("%s: RSP_REMOTE_AVTIMER_VOTE_REQUEST handle %x\n",
			__func__, payload1[0]);
		avtimer.timer_handle = payload1[0];
		avtimer.enable_timer_resp_received = 1;
		wake_up(&avtimer.adsp_resp_wait);
		break;
	default:
		pr_err("%s: Message adspcore svc: %d\n",
				__func__, data->opcode);
		break;
	}

	return 0;
}

int avcs_core_open(void)
{
	if (!avtimer.core_handle_q)
		avtimer.core_handle_q = apr_register("ADSP", "CORE",
					aprv2_core_fn_q, TEMP_PORT, NULL);
	pr_debug("%s: Open_q %p\n", __func__, avtimer.core_handle_q);
	if (!avtimer.core_handle_q) {
		pr_err("%s: Unable to register CORE\n", __func__);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(avcs_core_open);

static int avcs_core_disable_avtimer(int timerhandle)
{
	int rc = -EINVAL;
	struct adsp_avt_timer payload;

	if (!timerhandle) {
		pr_err("%s: Invalid timer handle\n", __func__);
		return -EINVAL;
	}
	memset(&payload, 0, sizeof(payload));
	rc = avcs_core_open();
	if (!rc && avtimer.core_handle_q) {
		payload.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		payload.hdr.pkt_size =
			sizeof(struct adsp_avt_timer);
		payload.hdr.src_svc = avtimer.core_handle_q->id;
		payload.hdr.src_domain = APR_DOMAIN_APPS;
		payload.hdr.dest_domain = APR_DOMAIN_ADSP;
		payload.hdr.dest_svc = APR_SVC_ADSP_CORE;
		payload.hdr.src_port = TEMP_PORT;
		payload.hdr.dest_port = TEMP_PORT;
		payload.hdr.token = CORE_CLIENT;
		payload.hdr.opcode = AVCS_CMD_REMOTE_AVTIMER_RELEASE_REQUEST;
		payload.avtimer_handle = timerhandle;
		pr_debug("%s: disable avtimer opcode %x handle %x\n",
			__func__, payload.hdr.opcode, payload.avtimer_handle);
		rc = apr_send_pkt(avtimer.core_handle_q,
						(uint32_t *)&payload);
		if (rc < 0)
			pr_err("%s: Enable AVtimer failed op[0x%x]rc[%d]\n",
				__func__, payload.hdr.opcode, rc);
		else
			rc = 0;
	}
	return rc;
}

static int avcs_core_enable_avtimer(char *client_name)
{
	int rc = -EINVAL, ret = -EINVAL;
	struct adsp_avt_timer payload;

	if (!client_name) {
		pr_err("%s: Invalid params\n", __func__);
		return -EINVAL;
	}
	memset(&payload, 0, sizeof(payload));
	rc = avcs_core_open();
	if (!rc && avtimer.core_handle_q) {
		avtimer.enable_timer_resp_received = 0;
		payload.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_EVENT,
			APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
		payload.hdr.pkt_size =
			sizeof(struct adsp_avt_timer);
		payload.hdr.src_svc = avtimer.core_handle_q->id;
		payload.hdr.src_domain = APR_DOMAIN_APPS;
		payload.hdr.dest_domain = APR_DOMAIN_ADSP;
		payload.hdr.dest_svc = APR_SVC_ADSP_CORE;
		payload.hdr.src_port = TEMP_PORT;
		payload.hdr.dest_port = TEMP_PORT;
		payload.hdr.token = CORE_CLIENT;
		payload.hdr.opcode = AVCS_CMD_REMOTE_AVTIMER_VOTE_REQUEST;
		strlcpy(payload.client_name, client_name,
			   sizeof(payload.client_name));
		pr_debug("%s: enable avtimer opcode %x client name %s\n",
			__func__, payload.hdr.opcode, payload.client_name);
		rc = apr_send_pkt(avtimer.core_handle_q,
						(uint32_t *)&payload);
		if (rc < 0) {
			pr_err("%s: Enable AVtimer failed op[0x%x]rc[%d]\n",
				__func__, payload.hdr.opcode, rc);
			goto bail;
		} else
			rc = 0;
		ret = wait_event_timeout(avtimer.adsp_resp_wait,
			(avtimer.enable_timer_resp_received == 1),
			msecs_to_jiffies(TIMEOUT_MS));
		if (!ret) {
			pr_err("%s: wait_event timeout for Enable timer\n",
					__func__);
			rc = -ETIMEDOUT;
		}
		if (rc)
			avtimer.timer_handle = 0;
	}
bail:
	return rc;
}

int avcs_core_disable_power_collapse(int enable)
{
	int rc = 0;

	mutex_lock(&avtimer.avtimer_lock);
	if (enable) {
		if (avtimer.avtimer_open_cnt) {
			avtimer.avtimer_open_cnt++;
			pr_debug("%s: opened avtimer open count=%d\n",
				__func__, avtimer.avtimer_open_cnt);
			rc = 0;
			goto done;
		}
		rc = avcs_core_enable_avtimer("timer");
		if (!rc) {
			avtimer.avtimer_open_cnt++;
			atomic_set(&avtimer.adsp_ready, 1);
		}
	} else {
		if (avtimer.avtimer_open_cnt > 0) {
			avtimer.avtimer_open_cnt--;
			if (!avtimer.avtimer_open_cnt) {
				rc = avcs_core_disable_avtimer(
				avtimer.timer_handle);
				avtimer.timer_handle = 0;
				atomic_set(&avtimer.adsp_ready, 0);
			}
		}
	}
done:
	mutex_unlock(&avtimer.avtimer_lock);
	return rc;
}
EXPORT_SYMBOL(avcs_core_disable_power_collapse);

static void reset_work(struct work_struct *work)
{
	if (q6core_is_adsp_ready()) {
		avcs_core_disable_power_collapse(1);
		avtimer.num_retries = Q6_READY_MAX_RETRIES;
		return;
	}
	pr_debug("%s:Q6 not ready-retry after sometime\n", __func__);
	if (--avtimer.num_retries > 0) {
		schedule_delayed_work(&avtimer.ssr_dwork,
			  msecs_to_jiffies(Q6_READY_RETRY));
	} else {
		pr_err("%s: Q6 failed responding after multiple retries\n",
							__func__);
		avtimer.num_retries = Q6_READY_MAX_RETRIES;
	}
}

int avcs_core_query_timer(uint64_t *avtimer_tick)
{
	uint32_t avtimer_msw = 0, avtimer_lsw = 0;
	uint64_t avtimer_tick_temp;

	if (!atomic_read(&avtimer.adsp_ready)) {
		pr_debug("%s:In SSR, return\n", __func__);
		return -ENETRESET;
	}
	avtimer_lsw = ioread32(avtimer.p_avtimer_lsw);
	avtimer_msw = ioread32(avtimer.p_avtimer_msw);

	avtimer_tick_temp = (uint64_t)((uint64_t)avtimer_msw << 32)
			    | avtimer_lsw;
	*avtimer_tick = mul_u64_u32_div(avtimer_tick_temp, avtimer.clk_mult,
					avtimer.clk_div);
	pr_debug_ratelimited("%s:Avtimer: msw: %u, lsw: %u, tick: %llu\n",
			__func__,
			avtimer_msw, avtimer_lsw, *avtimer_tick);
	return 0;
}
EXPORT_SYMBOL(avcs_core_query_timer);

/*
 * avcs_core_query_timer_offset:
 *       derive offset between system clock & avtimer clock
 *
 * @ avoffset: offset between system clock & avtimer clock
 * @ clock_id: clock id to get the system time
 *
 */
int avcs_core_query_timer_offset(int64_t *av_offset, int32_t clock_id)
{
	uint32_t avtimer_msw = 0, avtimer_lsw = 0;
	uint64_t avtimer_tick_temp, avtimer_tick, sys_time = 0;
	struct timespec64 ts;

	if (!atomic_read(&avtimer.adsp_ready)) {
		pr_debug("%s:In SSR, return\n", __func__);
		return -ENETRESET;
	}

	if ((avtimer.p_avtimer_lsw == NULL) ||
	    (avtimer.p_avtimer_msw == NULL)) {
		return -EINVAL;
	}

	memset(&ts, 0, sizeof(struct timespec64));
	avtimer_lsw = ioread32(avtimer.p_avtimer_lsw);
	avtimer_msw = ioread32(avtimer.p_avtimer_msw);

	switch (clock_id) {
	case CLOCK_MONOTONIC_RAW:
		ktime_get_raw_ts64(&ts);
		break;
	case CLOCK_BOOTTIME:
		ktime_get_boottime_ts64(&ts);
		break;
	case CLOCK_MONOTONIC:
		ktime_get_ts64(&ts);
		break;
	case CLOCK_REALTIME:
		ktime_get_real_ts64(&ts);
		break;
	default:
		pr_debug("%s: unsupported clock id %d\n", __func__, clock_id);
		return -EINVAL;
	}

	sys_time = ts.tv_sec * 1000000LL + div64_u64(ts.tv_nsec, 1000);
	avtimer_tick_temp = (uint64_t)((uint64_t)avtimer_msw << 32) |
						 avtimer_lsw;

	avtimer_tick = mul_u64_u32_div(avtimer_tick_temp, avtimer.clk_mult,
					avtimer.clk_div);
	*av_offset = sys_time - avtimer_tick;
	pr_debug("%s: sys_time: %llu, offset %lld, avtimer tick %lld\n",
		 __func__, sys_time, *av_offset, avtimer_tick);

	return 0;
}
EXPORT_SYMBOL(avcs_core_query_timer_offset);

#if IS_ENABLED(CONFIG_AVTIMER_LEGACY)
static void avcs_set_isp_fptr(bool enable)
{
	struct avtimer_fptr_t av_fptr;

	if (enable) {
		av_fptr.fptr_avtimer_open = avcs_core_open;
		av_fptr.fptr_avtimer_enable = avcs_core_disable_power_collapse;
		av_fptr.fptr_avtimer_get_time = avcs_core_query_timer;
		msm_isp_set_avtimer_fptr(av_fptr);
	} else {
		av_fptr.fptr_avtimer_open = NULL;
		av_fptr.fptr_avtimer_enable = NULL;
		av_fptr.fptr_avtimer_get_time = NULL;
		msm_isp_set_avtimer_fptr(av_fptr);
	}
}
#else
static void avcs_set_isp_fptr(bool enable)
{
}
#endif

static int avtimer_open(struct inode *inode, struct file *file)
{
	return avcs_core_disable_power_collapse(1);
}

static int avtimer_release(struct inode *inode, struct file *file)
{
	return avcs_core_disable_power_collapse(0);
}

/*
 * ioctl call provides GET_AVTIMER
 */
static long avtimer_ioctl(struct file *file, unsigned int ioctl_num,
				unsigned long ioctl_param)
{
	switch (ioctl_num) {
	case IOCTL_GET_AVTIMER_TICK:
	{
		uint64_t avtimer_tick = 0;
		int rc;

		rc = avcs_core_query_timer(&avtimer_tick);

		if (rc) {
			pr_err("%s: Error: Invalid AV Timer tick, rc = %d\n",
				__func__, rc);
			return rc;
		}

		pr_debug_ratelimited("%s: AV Timer tick: time %llx\n",
		__func__, avtimer_tick);
		if (copy_to_user((void __user *)ioctl_param, &avtimer_tick,
		    sizeof(avtimer_tick))) {
			pr_err("%s: copy_to_user failed\n", __func__);
			return -EFAULT;
		}
	}
		break;

	default:
		pr_err("%s: invalid cmd\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static const struct file_operations avtimer_fops = {
	.unlocked_ioctl = avtimer_ioctl,
	.compat_ioctl = avtimer_ioctl,
	.open = avtimer_open,
	.release = avtimer_release
};

static int dev_avtimer_probe(struct platform_device *pdev)
{
	int result = 0;
	dev_t dev = MKDEV(major, 0);
	struct device *device_handle;
	struct resource *reg_lsb = NULL, *reg_msb = NULL;
	uint32_t clk_div_val;
	uint32_t clk_mult_val;

	if (!pdev) {
		pr_err("%s: Invalid params\n", __func__);
		return -EINVAL;
	}
	reg_lsb = platform_get_resource_byname(pdev,
		IORESOURCE_MEM, "avtimer_lsb_addr");
	if (!reg_lsb) {
		dev_err(&pdev->dev, "%s: Looking up %s property",
			"avtimer_lsb_addr", __func__);
		return -EINVAL;
	}
	reg_msb = platform_get_resource_byname(pdev,
		IORESOURCE_MEM, "avtimer_msb_addr");
	if (!reg_msb) {
		dev_err(&pdev->dev, "%s: Looking up %s property",
			"avtimer_msb_addr", __func__);
		return -EINVAL;
	}
	INIT_DELAYED_WORK(&avtimer.ssr_dwork, reset_work);

	avtimer.p_avtimer_lsw = devm_ioremap_nocache(&pdev->dev,
				reg_lsb->start, resource_size(reg_lsb));
	if (!avtimer.p_avtimer_lsw) {
		dev_err(&pdev->dev, "%s: ioremap failed for lsb avtimer register",
			__func__);
		return -ENOMEM;
	}

	avtimer.p_avtimer_msw = devm_ioremap_nocache(&pdev->dev,
				reg_msb->start, resource_size(reg_msb));
	if (!avtimer.p_avtimer_msw) {
		dev_err(&pdev->dev, "%s: ioremap failed for msb avtimer register",
			__func__);
		goto unmap;
	}
	avtimer.num_retries = Q6_READY_MAX_RETRIES;
	/* get the device number */
	if (major)
		result = register_chrdev_region(dev, 1, DEVICE_NAME);
	else {
		result = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
		major = MAJOR(dev);
	}

	if (result < 0) {
		dev_err(&pdev->dev, "%s: Registering avtimer device failed\n",
			__func__);
		goto unmap;
	}

	avtimer.avtimer_class = class_create(THIS_MODULE, "avtimer");
	if (IS_ERR(avtimer.avtimer_class)) {
		result = PTR_ERR(avtimer.avtimer_class);
		dev_err(&pdev->dev, "%s: Error creating avtimer class: %d\n",
			__func__, result);
		goto unregister_chrdev_region;
	}

	cdev_init(&avtimer.myc, &avtimer_fops);
	result = cdev_add(&avtimer.myc, dev, 1);

	if (result < 0) {
		dev_err(&pdev->dev, "%s: Registering file operations failed\n",
			__func__);
		goto class_destroy;
	}

	device_handle = device_create(avtimer.avtimer_class,
			NULL, avtimer.myc.dev, NULL, "avtimer");
	if (IS_ERR(device_handle)) {
		result = PTR_ERR(device_handle);
		pr_err("%s: device_create failed: %d\n", __func__, result);
		goto class_destroy;
	}
	init_waitqueue_head(&avtimer.adsp_resp_wait);
	mutex_init(&avtimer.avtimer_lock);
	avtimer.avtimer_open_cnt = 0;

	pr_debug("%s: Device create done for avtimer major=%d\n",
			__func__, major);

	if (of_property_read_u32(pdev->dev.of_node,
			"qcom,clk-div", &clk_div_val))
		avtimer.clk_div = 1;
	else
		avtimer.clk_div = clk_div_val;

	if (of_property_read_u32(pdev->dev.of_node,
			"qcom,clk-mult", &clk_mult_val))
		avtimer.clk_mult = 1;
	else
		avtimer.clk_mult = clk_mult_val;

	avcs_set_isp_fptr(true);

	pr_debug("%s: avtimer.clk_div = %d, avtimer.clk_mult = %d\n",
		 __func__, avtimer.clk_div, avtimer.clk_mult);
	return 0;

class_destroy:
	class_destroy(avtimer.avtimer_class);
unregister_chrdev_region:
	unregister_chrdev_region(MKDEV(major, 0), 1);
unmap:
	if (avtimer.p_avtimer_lsw)
		devm_iounmap(&pdev->dev, avtimer.p_avtimer_lsw);
	if (avtimer.p_avtimer_msw)
		devm_iounmap(&pdev->dev, avtimer.p_avtimer_msw);
	avtimer.p_avtimer_lsw = NULL;
	avtimer.p_avtimer_msw = NULL;
	return result;

}

static int dev_avtimer_remove(struct platform_device *pdev)
{
	pr_debug("%s: dev_avtimer_remove\n", __func__);

	if (avtimer.p_avtimer_lsw)
		devm_iounmap(&pdev->dev, avtimer.p_avtimer_lsw);
	if (avtimer.p_avtimer_msw)
		devm_iounmap(&pdev->dev, avtimer.p_avtimer_msw);
	device_destroy(avtimer.avtimer_class, avtimer.myc.dev);
	cdev_del(&avtimer.myc);
	class_destroy(avtimer.avtimer_class);
	unregister_chrdev_region(MKDEV(major, 0), 1);
	avcs_set_isp_fptr(false);

	return 0;
}

static const struct of_device_id avtimer_machine_of_match[]  = {
	{ .compatible = "qcom,avtimer", },
	{},
};
static struct platform_driver dev_avtimer_driver = {
	.probe = dev_avtimer_probe,
	.remove = dev_avtimer_remove,
	.driver = {
		.name = "dev_avtimer",
		.of_match_table = avtimer_machine_of_match,
		.suppress_bind_attrs = true,
	},
};

int  __init avtimer_init(void)
{
	s32 rc;

	rc = platform_driver_register(&dev_avtimer_driver);
	if (rc < 0) {
		pr_err("%s: platform_driver_register failed\n", __func__);
		goto error_platform_driver;
	}
	pr_debug("%s: dev_avtimer_init : done\n", __func__);

	return 0;
error_platform_driver:

	pr_err("%s: encounterd error\n", __func__);
	return rc;
}

void avtimer_exit(void)
{
	platform_driver_unregister(&dev_avtimer_driver);
}

MODULE_DESCRIPTION("avtimer driver");
MODULE_LICENSE("GPL v2");
