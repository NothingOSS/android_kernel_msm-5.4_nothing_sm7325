// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2010-2014, 2016-2021 The Linux Foundation. All rights reserved.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/ipc_logging.h>
#include <linux/of_platform.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/snd_event.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/audio_notifier.h>
#include <ipc/apr.h>
#include <ipc/apr_tal.h>
#include <ipc/aprv2_vm.h>
#include <linux/habmm.h>
#include <uapi/linux/sched/types.h>

#define APR_PKT_IPC_LOG_PAGE_CNT 2
#define APR_VM_CB_THREAD_NAME "apr_vm_cb_thread"
#define APR_TX_BUF_SIZE 4096
#define APR_RX_BUF_SIZE 4096

static struct apr_q6 q6;
static struct apr_client client[APR_DEST_MAX][APR_CLIENT_MAX];
static void *apr_pkt_ctx;
static wait_queue_head_t modem_wait;
static bool is_modem_up;
static char *subsys_name = NULL;
/* Subsystem restart: QDSP6 data, functions */
static struct workqueue_struct *apr_reset_workqueue;
static void apr_reset_deregister(struct work_struct *work);
static void dispatch_event(unsigned long code, uint16_t proc);
struct apr_reset_work {
	void *handle;
	struct work_struct work;
};

struct apr_chld_device {
	struct platform_device *pdev;
	struct list_head node;
};

struct apr_private {
	struct device *dev;
	spinlock_t apr_lock;
	bool is_initial_boot;
	struct work_struct add_chld_dev_work;
};

static struct apr_private *apr_priv;
static bool apr_cf_debug;

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_apr_debug;
static ssize_t apr_debug_write(struct file *filp, const char __user *ubuf,
			       size_t cnt, loff_t *ppos)
{
	char cmd;

	if (copy_from_user(&cmd, ubuf, 1))
		return -EFAULT;

	apr_cf_debug = (cmd == '1') ? true : false;

	return cnt;
}

static const struct file_operations apr_debug_ops = {
	.write = apr_debug_write,
};
#endif

#define APR_PKT_INFO(x...) \
do { \
	if (apr_pkt_ctx) \
		ipc_log_string(apr_pkt_ctx, "<APR>: "x); \
} while (0)

/* hab handle */
static uint32_t hab_handle_tx;
static uint32_t hab_handle_rx;
static char apr_tx_buf[APR_TX_BUF_SIZE];
static char apr_rx_buf[APR_RX_BUF_SIZE];
static spinlock_t hab_tx_lock;

/* apr callback thread task */
static struct task_struct *apr_vm_cb_thread_task;
static int pid;


struct apr_svc_table {
	char name[64];
	int idx;
	int id;
	int dest_svc;
	int client_id;
	int handle;
};

/*
 * src svc should be assigned dynamically through apr registration:
 * 1. replace with a proper string name for registration.
 *    e.g. "qcom.apps.lnx." + name
 * 2. register apr BE, retrieve dynamic src svc address,
 *    apr handle and store in svc tbl.
 */


static struct apr_svc_table svc_tbl_qdsp6[] = {
	{
		.name = "AFE",
		.idx = 0,
		.id = 0,
		.dest_svc = APR_SVC_AFE,
		.client_id = APR_CLIENT_AUDIO,
		.handle = 0,
	},
	{
		.name = "ASM",
		.idx = 1,
		.id = 0,
		.dest_svc = APR_SVC_ASM,
		.client_id = APR_CLIENT_AUDIO,
		.handle = 0,
	},
	{
		.name = "ADM",
		.idx = 2,
		.id = 0,
		.dest_svc = APR_SVC_ADM,
		.client_id = APR_CLIENT_AUDIO,
		.handle = 0,
	},
	{
		.name = "CORE",
		.idx = 3,
		.id = 0,
		.dest_svc = APR_SVC_ADSP_CORE,
		.client_id = APR_CLIENT_AUDIO,
		.handle = 0,
	},
	{
		.name = "TEST",
		.idx = 4,
		.id = 0,
		.dest_svc = APR_SVC_TEST_CLIENT,
		.client_id = APR_CLIENT_AUDIO,
		.handle = 0,
	},
	{
		.name = "MVM",
		.idx = 5,
		.id = 0,
		.dest_svc = APR_SVC_ADSP_MVM,
		.client_id = APR_CLIENT_AUDIO,
		.handle = 0,
	},
	{
		.name = "CVS",
		.idx = 6,
		.id = 0,
		.dest_svc = APR_SVC_ADSP_CVS,
		.client_id = APR_CLIENT_AUDIO,
		.handle = 0,
	},
	{
		.name = "CVP",
		.idx = 7,
		.id = 0,
		.dest_svc = APR_SVC_ADSP_CVP,
		.client_id = APR_CLIENT_AUDIO,
		.handle = 0,
	},
	{
		.name = "USM",
		.idx = 8,
		.id = 0,
		.dest_svc = APR_SVC_USM,
		.client_id = APR_CLIENT_AUDIO,
		.handle = 0,
	},
	{
		.name = "VIDC",
		.idx = 9,
		.id = 0,
		.dest_svc = APR_SVC_VIDC,
		.handle = 0,
	},
	{
		.name = "LSM",
		.idx = 10,
		.id = 0,
		.dest_svc = APR_SVC_LSM,
		.client_id = APR_CLIENT_AUDIO,
		.handle = 0,
	},
};


static struct apr_svc_table svc_tbl_voice[] = {
	{
		.name = "VSM",
		.idx = 0,
		.id = 0,
		.dest_svc = APR_SVC_VSM,
		.client_id = APR_CLIENT_VOICE,
		.handle = 0,
	},
	{
		.name = "VPM",
		.idx = 1,
		.id = 0,
		.dest_svc = APR_SVC_VPM,
		.client_id = APR_CLIENT_VOICE,
		.handle = 0,
	},
	{
		.name = "MVS",
		.idx = 2,
		.id = 0,
		.dest_svc = APR_SVC_MVS,
		.client_id = APR_CLIENT_VOICE,
		.handle = 0,
	},
	{
		.name = "MVM",
		.idx = 3,
		.id = 0,
		.dest_svc = APR_SVC_MVM,
		.client_id = APR_CLIENT_VOICE,
		.handle = 0,
	},
	{
		.name = "CVS",
		.idx = 4,
		.id = 0,
		.dest_svc = APR_SVC_CVS,
		.client_id = APR_CLIENT_VOICE,
		.handle = 0,
	},
	{
		.name = "CVP",
		.idx = 5,
		.id = 0,
		.dest_svc = APR_SVC_CVP,
		.client_id = APR_CLIENT_VOICE,
		.handle = 0,
	},
	{
		.name = "SRD",
		.idx = 6,
		.id = 0,
		.dest_svc = APR_SVC_SRD,
		.client_id = APR_CLIENT_VOICE,
		.handle = 0,
	},
	{
		.name = "TEST",
		.idx = 7,
		.id = 0,
		.dest_svc = APR_SVC_TEST_CLIENT,
		.client_id = APR_CLIENT_VOICE,
		.handle = 0,
	},
};

/**
 * apr_get_modem_state:
 *
 * Returns current modem load status
 *
 */
enum apr_subsys_state apr_get_modem_state(void)
{
	return atomic_read(&q6.modem_state);
}
EXPORT_SYMBOL(apr_get_modem_state);

/**
 * apr_set_modem_state - Update modem load status.
 *
 * @state: State to update modem load status
 *
 */
void apr_set_modem_state(enum apr_subsys_state state)
{
	atomic_set(&q6.modem_state, state);
}
EXPORT_SYMBOL(apr_set_modem_state);

enum apr_subsys_state apr_cmpxchg_modem_state(enum apr_subsys_state prev,
					      enum apr_subsys_state new)
{
	return atomic_cmpxchg(&q6.modem_state, prev, new);
}

static void apr_modem_down(unsigned long opcode)
{
	apr_set_modem_state(APR_SUBSYS_DOWN);
	dispatch_event(opcode, APR_DEST_MODEM);
}

static void apr_modem_up(void)
{
	if (apr_cmpxchg_modem_state(APR_SUBSYS_DOWN, APR_SUBSYS_UP) ==
							APR_SUBSYS_DOWN)
		wake_up(&modem_wait);
	is_modem_up = 1;
}

enum apr_subsys_state apr_get_q6_state(void)
{
	return atomic_read(&q6.q6_state);
}
EXPORT_SYMBOL(apr_get_q6_state);

int apr_set_q6_state(enum apr_subsys_state state)
{
	pr_debug("%s: setting adsp state %d\n", __func__, state);
	if (state < APR_SUBSYS_DOWN || state > APR_SUBSYS_LOADED)
		return -EINVAL;
	atomic_set(&q6.q6_state, state);
	return 0;
}
EXPORT_SYMBOL(apr_set_q6_state);

static void apr_ssr_disable(struct device *dev, void *data)
{
	apr_set_q6_state(APR_SUBSYS_DOWN);
}

static const struct snd_event_ops apr_ssr_ops = {
	.disable = apr_ssr_disable,
};

static void apr_adsp_down(unsigned long opcode)
{
	pr_info("%s: Q6 is Down\n", __func__);
	snd_event_notify(apr_priv->dev, SND_EVENT_DOWN);
	apr_set_q6_state(APR_SUBSYS_DOWN);
	dispatch_event(opcode, APR_DEST_QDSP6);
}

static void apr_add_child_devices(struct work_struct *work)
{
	int ret;

	ret = of_platform_populate(apr_priv->dev->of_node,
			NULL, NULL, apr_priv->dev);
	if (ret)
		dev_err(apr_priv->dev, "%s: failed to add child nodes, ret=%d\n",
			__func__, ret);
}

static void apr_adsp_up(void)
{
	pr_info("%s: Q6 is Up\n", __func__);
	apr_set_q6_state(APR_SUBSYS_LOADED);

	spin_lock(&apr_priv->apr_lock);
	if (apr_priv->is_initial_boot)
		schedule_work(&apr_priv->add_chld_dev_work);
	spin_unlock(&apr_priv->apr_lock);
	snd_event_notify(apr_priv->dev, SND_EVENT_UP);
}

int apr_load_adsp_image(void)
{
	int rc = 0;

	mutex_lock(&q6.lock);
	if (apr_get_q6_state() == APR_SUBSYS_UP) {
		q6.pil = subsystem_get("adsp");
		if (IS_ERR(q6.pil)) {
			rc = PTR_ERR(q6.pil);
			pr_err("APR: Unable to load q6 image, error:%d\n", rc);
		} else {
			apr_set_q6_state(APR_SUBSYS_LOADED);
			pr_debug("APR: Image is loaded, stated\n");
		}
	} else if (apr_get_q6_state() == APR_SUBSYS_LOADED) {
		pr_debug("APR: q6 image already loaded\n");
	} else {
		pr_debug("APR: cannot load state %d\n", apr_get_q6_state());
	}
	mutex_unlock(&q6.lock);
	return rc;
}

struct apr_client *apr_get_client(int dest_id, int client_id)
{
	return &client[dest_id][client_id];
}

static int apr_vm_nb_receive(int32_t handle, void *dest_buff,
	uint32_t *size_bytes, uint32_t timeout)
{
	int rc;
	uint32_t dest_buff_bytes = *size_bytes;
	unsigned long delay = jiffies + (HZ / 2);

	do {
		*size_bytes = dest_buff_bytes;
		rc = habmm_socket_recv(handle,
				dest_buff,
				size_bytes,
				timeout,
				HABMM_SOCKET_RECV_FLAGS_NON_BLOCKING);
	} while (time_before(jiffies, delay) && (rc == -EAGAIN) &&
		(*size_bytes == 0));

	return rc;
}

static int apr_vm_cb_process_evt(char *buf, int len)
{
	struct apr_client_data data = {0,};
	struct apr_client *apr_client;
	struct apr_svc *c_svc;
	struct apr_hdr *hdr;
	uint16_t hdr_size;
	uint16_t msg_type;
	uint16_t ver;
	uint16_t src;
	uint16_t svc;
	uint16_t clnt;
	int i;
	int temp_port = 0;
	uint32_t *ptr;
	uint32_t evt_id;

	pr_debug("APR: len = %d\n", len);
	ptr = (uint32_t *)buf;
	pr_debug("\n*****************\n");
	for (i = 0; i < len/4; i++)
		pr_debug("%x  ", ptr[i]);
	pr_debug("\n");
	pr_debug("\n*****************\n");

	if (!buf || len <= APR_HDR_SIZE + sizeof(uint32_t)) {
		pr_err("APR: Improper apr pkt received: %p %d\n", buf, len);
		return -EINVAL;
	}

	evt_id = *((int32_t *)buf);
	if (evt_id != APRV2_VM_EVT_RX_PKT_AVAILABLE) {
		pr_err("APR: Wrong evt id: %d\n", evt_id);
		return -EINVAL;
	}
	hdr = (struct apr_hdr *)(buf + sizeof(uint32_t));

	ver = hdr->hdr_field;
	ver = (ver & 0x000F);
	if (ver > APR_PKT_VER + 1) {
		pr_err("APR: Wrong version: %d\n", ver);
		return -EINVAL;
	}

	hdr_size = hdr->hdr_field;
	hdr_size = ((hdr_size & 0x00F0) >> 0x4) * 4;
	if (hdr_size < APR_HDR_SIZE) {
		pr_err("APR: Wrong hdr size:%d\n", hdr_size);
		return -EINVAL;
	}

	if (hdr->pkt_size < APR_HDR_SIZE) {
		pr_err("APR: Wrong paket size\n");
		return -EINVAL;
	}

	msg_type = hdr->hdr_field;
	msg_type = (msg_type >> 0x08) & 0x0003;
	if (msg_type >= APR_MSG_TYPE_MAX && msg_type != APR_BASIC_RSP_RESULT) {
		pr_err("APR: Wrong message type: %d\n", msg_type);
		return -EINVAL;
	}

	/*
	 * dest_svc is dynamic created by apr service
	 * no need to check the range of dest_svc
	 */
	if (hdr->src_domain >= APR_DOMAIN_MAX ||
		hdr->dest_domain >= APR_DOMAIN_MAX ||
		hdr->src_svc >= APR_SVC_MAX) {
		pr_err("APR: Wrong APR header\n");
		return -EINVAL;
	}

	svc = hdr->dest_svc;
	if (hdr->src_domain == APR_DOMAIN_MODEM)
		clnt = APR_CLIENT_VOICE;
	else if (hdr->src_domain == APR_DOMAIN_ADSP)
		clnt = APR_CLIENT_AUDIO;
	else {
		pr_err("APR: Pkt from wrong source: %d\n", hdr->src_domain);
		return -EINVAL;
	}

	src = apr_get_data_src(hdr);
	if (src == APR_DEST_MAX)
		return -EINVAL;

	pr_debug("src =%d clnt = %d\n", src, clnt);
	apr_client = &client[src][clnt];
	for (i = 0; i < APR_SVC_MAX; i++)
		if (apr_client->svc[i].id == svc) {
			pr_debug("svc_id = %d\n", apr_client->svc[i].id);
			c_svc = &apr_client->svc[i];
			break;
		}

	if (i == APR_SVC_MAX) {
		pr_err("APR: service is not registered\n");
		return -ENXIO;
	}

	pr_debug("svc_idx = %d\n", i);
	pr_debug("%x %x %x %p %p\n", c_svc->id, c_svc->dest_id,
		 c_svc->client_id, c_svc->fn, c_svc->priv);

	data.payload_size = hdr->pkt_size - hdr_size;
	data.opcode = hdr->opcode;
	data.src = src;
	data.src_port = hdr->src_port;
	data.dest_port = hdr->dest_port;
	data.token = hdr->token;
	data.msg_type = msg_type;
	if (data.payload_size > 0)
		data.payload = (char *)hdr + hdr_size;

	if (unlikely(apr_cf_debug)) {
		if (hdr->opcode == APR_BASIC_RSP_RESULT && data.payload) {
			uint32_t *ptr = data.payload;

			APR_PKT_INFO(
			"Rx: src_addr[0x%X] dest_addr[0x%X] opcode[0x%X] token[0x%X] rc[0x%X]",
			(hdr->src_domain << 8) | hdr->src_svc,
			(hdr->dest_domain << 8) | hdr->dest_svc,
			hdr->opcode, hdr->token, ptr[1]);
		} else {
			APR_PKT_INFO(
			"Rx: src_addr[0x%X] dest_addr[0x%X] opcode[0x%X] token[0x%X]",
			(hdr->src_domain << 8) | hdr->src_svc,
			(hdr->dest_domain << 8) | hdr->dest_svc, hdr->opcode,
			hdr->token);
		}
	}

	temp_port = ((data.dest_port >> 8) * 8) + (data.dest_port & 0xFF);
	pr_debug("port = %d t_port = %d\n", data.src_port, temp_port);
	if (((temp_port >= 0) && (temp_port < APR_MAX_PORTS))
		&& (c_svc->port_cnt && c_svc->port_fn[temp_port]))
		c_svc->port_fn[temp_port](&data, c_svc->port_priv[temp_port]);
	else if (c_svc->fn)
		c_svc->fn(&data, c_svc->priv);
	else
		pr_err("APR: Rxed a packet for NULL callback\n");

	return 0;
}

static int apr_vm_cb_thread(void *data)
{
	uint32_t apr_rx_buf_len;
#ifdef APRV2_VM_BE_ASYNC_SEND_RSP
	struct aprv2_vm_ack_rx_pkt_available_t apr_ack;
#endif
	unsigned long delay = jiffies + (HZ / 2);
	int status = 0;
	int ret = 0;
	struct sched_param param = {.sched_priority = 3};

	sched_setscheduler(current, SCHED_FIFO, &param);

	while (1) {
		do {
			apr_rx_buf_len = sizeof(apr_rx_buf);
			ret = habmm_socket_recv(hab_handle_rx,
					(void *)&apr_rx_buf,
					&apr_rx_buf_len,
					0xFFFFFFFF,
					0);
		} while (time_before(jiffies, delay) && (ret == -EINTR) &&
			(apr_rx_buf_len == 0));
		if (ret) {
			pr_err("%s: habmm_socket_recv failed %d\n",
					__func__, ret);
			break;
		}

		status = apr_vm_cb_process_evt(apr_rx_buf, apr_rx_buf_len);

#ifdef APRV2_VM_BE_ASYNC_SEND_RSP
		apr_ack.status = status;
		ret = habmm_socket_send(hab_handle_rx,
				(void *)&apr_ack,
				sizeof(apr_ack),
				0);
#else
		ret = status;
#endif
		if (ret) {
			pr_err("%s: habmm_socket_send failed %d\n",
					__func__, ret);
			/* TODO: break if send failed ? */
			break;
		}
	}

	return ret;
}

static int apr_vm_get_svc(const char *svc_name, int domain_id, int *client_id,
		int *svc_idx, int *svc_id, int *dest_svc, int *handle)
{
	int i;
	int size;
	struct apr_svc_table *tbl;
	struct aprv2_vm_cmd_register_rsp_t apr_rsp;
	uint32_t apr_len;
	int ret = 0;
	unsigned long flags;
	struct {
		uint32_t cmd_id;
		struct aprv2_vm_cmd_register_t reg_cmd;
	} tx_data;

	if (domain_id == APR_DOMAIN_ADSP) {
		tbl = svc_tbl_qdsp6;
		size = ARRAY_SIZE(svc_tbl_qdsp6);
	} else {
		tbl = svc_tbl_voice;
		size = ARRAY_SIZE(svc_tbl_voice);
	}
	spin_lock_irqsave(&hab_tx_lock, flags);
	for (i = 0; i < size; i++) {
		if (!strcmp(svc_name, tbl[i].name)) {
			*client_id = tbl[i].client_id;
			*svc_idx = tbl[i].idx;
			if (!tbl[i].id && !tbl[i].handle) {
				/* need to register a new service */
				memset((void *) &tx_data, 0, sizeof(tx_data));

				apr_len = sizeof(tx_data);
				tx_data.cmd_id = APRV2_VM_CMDID_REGISTER;
				tx_data.reg_cmd.name_size = snprintf(
						tx_data.reg_cmd.name,
						APRV2_VM_MAX_DNS_SIZE,
						"qcom.apps.lnx.%s",
						svc_name);
				tx_data.reg_cmd.addr = 0;
				ret = habmm_socket_send(hab_handle_tx,
						(void *) &tx_data,
						apr_len,
						0);
				if (ret) {
					pr_err("%s: habmm_socket_send failed %d\n",
						__func__, ret);
					spin_unlock_irqrestore(&hab_tx_lock, flags);
					return ret;
				}
				/* wait for response */
				apr_len = sizeof(apr_rsp);
				ret = apr_vm_nb_receive(hab_handle_tx,
						(void *)&apr_rsp,
						&apr_len,
						0xFFFFFFFF);
				if (ret) {
					pr_err("%s: apr_vm_nb_receive failed %d\n",
						__func__, ret);
					spin_unlock_irqrestore(&hab_tx_lock, flags);
					return ret;
				}
				if (apr_rsp.status) {
					pr_err("%s: apr_vm_nb_receive status %d\n",
						__func__, apr_rsp.status);
					ret = apr_rsp.status;
					spin_unlock_irqrestore(&hab_tx_lock, flags);
					return ret;
				}
				/* update svc table */
				tbl[i].handle = apr_rsp.handle;
				tbl[i].id = apr_rsp.addr &
						APRV2_VM_PKT_SERVICE_ID_MASK;
			}
			*svc_id = tbl[i].id;
			*dest_svc = tbl[i].dest_svc;
			*handle = tbl[i].handle;
			break;
		}
	}
	spin_unlock_irqrestore(&hab_tx_lock, flags);

	pr_debug("%s: svc_name = %s client_id = %d domain_id = %d\n",
		 __func__, svc_name, *client_id, domain_id);
	pr_debug("%s: src_svc = %d dest_svc = %d handle = %d\n",
		 __func__, *svc_id, *dest_svc, *handle);

	if (i == size) {
		pr_err("%s: APR: Wrong svc name %s\n", __func__, svc_name);
		ret = -EINVAL;
	}

	return ret;
}

static int apr_vm_rel_svc(int domain_id, int svc_id, int handle)
{
	int i;
	int size;
	struct apr_svc_table *tbl;
	struct aprv2_vm_cmd_deregister_rsp_t apr_rsp;
	uint32_t apr_len;
	int ret = 0;
	unsigned long flags;
	struct {
		uint32_t cmd_id;
		struct aprv2_vm_cmd_deregister_t dereg_cmd;
	} tx_data;

	if (domain_id == APR_DOMAIN_ADSP) {
		tbl = svc_tbl_qdsp6;
		size = ARRAY_SIZE(svc_tbl_qdsp6);
	} else {
		tbl = svc_tbl_voice;
		size = ARRAY_SIZE(svc_tbl_voice);
	}

	spin_lock_irqsave(&hab_tx_lock, flags);
	for (i = 0; i < size; i++) {
		if (tbl[i].id == svc_id && tbl[i].handle == handle) {
			/* need to deregister a service */
			memset((void *) &tx_data, 0, sizeof(tx_data));

			apr_len = sizeof(tx_data);
			tx_data.cmd_id = APRV2_VM_CMDID_DEREGISTER;
			tx_data.dereg_cmd.handle = handle;
			ret = habmm_socket_send(hab_handle_tx,
					(void *) &tx_data,
					apr_len,
					0);
			if (ret)
				pr_err("%s: habmm_socket_send failed %d\n",
					__func__, ret);
			/*
			 * TODO: if send failed, should not wait for recv.
			 *       should clear regardless?
			 */
			/* wait for response */
			apr_len = sizeof(apr_rsp);
			ret = apr_vm_nb_receive(hab_handle_tx,
					(void *)&apr_rsp,
					&apr_len,
					0xFFFFFFFF);
			if (ret)
				pr_err("%s: apr_vm_nb_receive failed %d\n",
					__func__, ret);
			if (apr_rsp.status) {
				pr_err("%s: apr_vm_nb_receive status %d\n",
					__func__, apr_rsp.status);
				ret = apr_rsp.status;
			}
			/* clear svc table */
			tbl[i].handle = 0;
			tbl[i].id = 0;
			break;
		}
	}
	spin_unlock_irqrestore(&hab_tx_lock, flags);
	if (i == size) {
		pr_err("%s: APR: Wrong svc id %d handle %d\n",
				__func__, svc_id, handle);
		ret = -EINVAL;
	}

	return ret;
}

static void apr_vm_set_subsys_state(void)
{
	/* set default subsys state in vm env.
	 * Both q6 and modem should be in LOADED state,
	 * since vm boots up at late stage after pm.
	 */
	apr_set_q6_state(APR_SUBSYS_LOADED);
	apr_set_modem_state(APR_SUBSYS_LOADED);

	spin_lock(&apr_priv->apr_lock);
	if (apr_priv->is_initial_boot)
		schedule_work(&apr_priv->add_chld_dev_work);
	apr_priv->is_initial_boot = false;
	spin_unlock(&apr_priv->apr_lock);
}

/**
 * apr_send_pkt - Clients call to send packet
 * to destination processor.
 *
 * @handle: APR service handle
 * @buf: payload to send to destination processor.
 *
 * Returns Bytes(>0)pkt_size on success or error on failure.
 */
int apr_send_pkt(void *handle, uint32_t *buf)
{
	struct apr_svc *svc = handle;
	struct apr_hdr *hdr;
	unsigned long flags;
	uint32_t *cmd_id = (uint32_t *)apr_tx_buf;
	struct aprv2_vm_cmd_async_send_t *apr_send =
		(struct aprv2_vm_cmd_async_send_t *)(apr_tx_buf +
			sizeof(uint32_t));
	uint32_t apr_send_len;
#ifdef APRV2_VM_BE_ASYNC_SEND_RSP
	struct aprv2_vm_cmd_async_send_rsp_t apr_rsp;
	uint32_t apr_rsp_len;
#endif
	int ret = 0;

	if (!handle || !buf) {
		pr_err("APR: Wrong parameters\n");
		return -EINVAL;
	}
	if (svc->need_reset) {
		pr_err("apr: send_pkt service need reset\n");
		return -ENETRESET;
	}

	if ((svc->dest_id == APR_DEST_QDSP6) &&
	    (apr_get_q6_state() != APR_SUBSYS_LOADED)) {
		pr_err("%s: Still dsp is not Up\n", __func__);
		return -ENETRESET;
	} else if ((svc->dest_id == APR_DEST_MODEM) &&
		   (apr_get_modem_state() == APR_SUBSYS_DOWN)) {
		pr_err("apr: Still Modem is not Up\n");
		return -ENETRESET;
	}

	spin_lock_irqsave(&hab_tx_lock, flags);
	if (!svc->id || !svc->vm_handle) {
		pr_err("APR: Still service is not yet opened\n");
		ret = -EINVAL;
		goto done;
	}
	hdr = (struct apr_hdr *)buf;

	hdr->src_domain = APR_DOMAIN_APPS;
	hdr->src_svc = svc->id;
	hdr->dest_domain = svc->dest_domain;
	hdr->dest_svc = svc->vm_dest_svc;

	if (unlikely(apr_cf_debug)) {
		APR_PKT_INFO(
		"Tx: src_addr[0x%X] dest_addr[0x%X] opcode[0x%X] token[0x%X]",
		(hdr->src_domain << 8) | hdr->src_svc,
		(hdr->dest_domain << 8) | hdr->dest_svc, hdr->opcode,
		hdr->token);
	}

	memset((void *)&apr_tx_buf, 0, sizeof(apr_tx_buf));
	/* pkt_size + cmd_id + handle */
	apr_send_len = hdr->pkt_size + sizeof(uint32_t) * 2;
	*cmd_id = APRV2_VM_CMDID_ASYNC_SEND;
	apr_send->handle = svc->vm_handle;

	/* safe check */
	if (hdr->pkt_size > APR_TX_BUF_SIZE - (sizeof(uint32_t) * 2)) {
		pr_err("APR: Wrong pkt size %d\n", hdr->pkt_size);
		ret = -ENOMEM;
		goto done;
	}
	memcpy(&apr_send->pkt_header, buf, hdr->pkt_size);

	ret = habmm_socket_send(hab_handle_tx,
			(void *)&apr_tx_buf,
			apr_send_len,
			0);
	if (ret) {
		pr_err("%s: habmm_socket_send failed %d\n",
				__func__, ret);
		goto done;
	}
#ifdef APRV2_VM_BE_ASYNC_SEND_RSP
	/* wait for response */
	apr_rsp_len = sizeof(apr_rsp);
	ret = apr_vm_nb_receive(hab_handle_tx,
			(void *)&apr_rsp,
			&apr_rsp_len,
			0xFFFFFFFF);
	if (ret) {
		pr_err("%s: apr_vm_nb_receive failed %d\n",
				__func__, ret);
		goto done;
	}
	if (apr_rsp.status) {
		pr_err("%s: apr_vm_nb_receive status %d\n",
				__func__, apr_rsp.status);
		/* should translate status properly */
		ret = -ECOMM;
		goto done;
	}
#endif

	/* upon successful send, return packet size */
	ret = hdr->pkt_size;

done:
	spin_unlock_irqrestore(&hab_tx_lock, flags);
	return ret;
}
EXPORT_SYMBOL(apr_send_pkt);

/**
 * apr_register - Clients call to register
 * to APR.
 *
 * @dest: destination processor
 * @svc_name: name of service to register as
 * @svc_fn: callback function to trigger when response
 *   ack or packets received from destination processor.
 * @src_port: Port number within a service
 * @priv: private data of client, passed back in cb fn.
 *
 * Returns apr_svc handle on success or NULL on failure.
 */
struct apr_svc *apr_register(char *dest, char *svc_name, apr_fn svc_fn,
				uint32_t src_port, void *priv)
{
	struct apr_client *clnt;
	int client_id = 0;
	int svc_idx = 0;
	int svc_id = 0;
	int dest_id = 0;
	int domain_id = 0;
	int temp_port = 0;
	struct apr_svc *svc = NULL;
	int rc = 0;
	bool can_open_channel = true;
	int dest_svc = 0;
	int handle = 0;

	if (!dest || !svc_name || !svc_fn)
		return NULL;

	if (!strcmp(dest, "ADSP"))
		domain_id = APR_DOMAIN_ADSP;
	else if (!strcmp(dest, "MODEM")) {
		/* Don't request for SMD channels if destination is MODEM,
		 * as these channels are no longer used and these clients
		 * are to listen only for MODEM SSR events
		 */
		can_open_channel = false;
		domain_id = APR_DOMAIN_MODEM;
	} else {
		pr_err("APR: wrong destination\n");
		goto done;
	}

	dest_id = apr_get_dest_id(dest);

	if (dest_id == APR_DEST_QDSP6) {
		if (apr_get_q6_state() != APR_SUBSYS_LOADED) {
			pr_err("%s: adsp not up\n", __func__);
			return NULL;
		}
		pr_debug("%s: adsp Up\n", __func__);
	} else if (dest_id == APR_DEST_MODEM) {
		if (apr_get_modem_state() == APR_SUBSYS_DOWN) {
			if (is_modem_up) {
				pr_err("%s: modem shutdown due to SSR, ret",
					__func__);
				return NULL;
			}
			pr_debug("%s: Wait for modem to bootup\n", __func__);
			rc = wait_event_interruptible_timeout(modem_wait,
						(apr_get_modem_state() == APR_SUBSYS_UP),
						(1 * HZ));
			if (rc == 0) {
				pr_err("%s: Modem is not Up\n", __func__);
				return NULL;
			}
		}
		pr_debug("%s: modem Up\n", __func__);
	}

	if (apr_vm_get_svc(svc_name, domain_id, &client_id, &svc_idx, &svc_id,
			&dest_svc, &handle)) {
		pr_err("%s: apr_vm_get_svc failed\n", __func__);
		goto done;
	}

	clnt = &client[dest_id][client_id];
	svc = &clnt->svc[svc_idx];
	mutex_lock(&svc->m_lock);
	clnt->id = client_id;
	if (svc->need_reset) {
		mutex_unlock(&svc->m_lock);
		pr_err("APR: Service needs reset\n");
		svc = NULL;
		goto done;
	}
	svc->id = svc_id;
	svc->vm_dest_svc = dest_svc;
	svc->dest_id = dest_id;
	svc->client_id = client_id;
	svc->dest_domain = domain_id;
	svc->pkt_owner = APR_PKT_OWNER_DRIVER;
	svc->vm_handle = handle;

	if (src_port != 0xFFFFFFFF) {
		temp_port = ((src_port >> 8) * 8) + (src_port & 0xFF);
		pr_debug("port = %d t_port = %d\n", src_port, temp_port);
		if (temp_port >= APR_MAX_PORTS || temp_port < 0) {
			pr_err("APR: temp_port out of bounds\n");
			mutex_unlock(&svc->m_lock);
			return NULL;
		}
		if (!svc->port_cnt && !svc->svc_cnt)
			clnt->svc_cnt++;
		svc->port_cnt++;
		svc->port_fn[temp_port] = svc_fn;
		svc->port_priv[temp_port] = priv;
	} else {
		if (!svc->fn) {
			if (!svc->port_cnt && !svc->svc_cnt)
				clnt->svc_cnt++;
			svc->fn = svc_fn;
			if (svc->port_cnt)
				svc->svc_cnt++;
			svc->priv = priv;
		}
	}

	mutex_unlock(&svc->m_lock);
done:
	return svc;
}
EXPORT_SYMBOL(apr_register);

static void apr_reset_deregister(struct work_struct *work)
{
	struct apr_svc *handle = NULL;
	struct apr_reset_work *apr_reset =
			container_of(work, struct apr_reset_work, work);

	handle = apr_reset->handle;
	pr_debug("%s:handle[%pK]\n", __func__, handle);
	apr_deregister(handle);
	kfree(apr_reset);
}

void apr_register_adsp_state_cb(void *adsp_cb, void *client_handle)
{
	q6.state_notify_cb = adsp_cb;
	q6.client_handle = client_handle;
}
EXPORT_SYMBOL(apr_register_adsp_state_cb);

/**
 * apr_start_rx_rt - Clients call to vote for thread
 * priority upgrade whenever needed.
 *
 * @handle: APR service handle
 *
 * Returns 0 on success or error otherwise.
 */
int apr_start_rx_rt(void *handle)
{
	int rc = 0;
	return rc;
}
EXPORT_SYMBOL(apr_start_rx_rt);

/**
 * apr_end_rx_rt - Clients call to unvote for thread
 * priority upgrade (perviously voted with
 * apr_start_rx_rt()).
 *
 * @handle: APR service handle
 *
 * Returns 0 on success or error otherwise.
 */
int apr_end_rx_rt(void *handle)
{
	int rc = 0;
	return rc;
}
EXPORT_SYMBOL(apr_end_rx_rt);


/**
 * apr_deregister - Clients call to de-register
 * from APR.
 *
 * @handle: APR service handle to de-register
 *
 * Returns 0 on success or -EINVAL on error.
 */
int apr_deregister(void *handle)
{
	struct apr_svc *svc = handle;
	struct apr_client *clnt;
	uint16_t dest_id;
	uint16_t client_id;

	if (!handle)
		return -EINVAL;

	mutex_lock(&svc->m_lock);
	dest_id = svc->dest_id;
	client_id = svc->client_id;
	clnt = &client[dest_id][client_id];

	if (svc->port_cnt > 0 || svc->svc_cnt > 0) {
		if (svc->port_cnt)
			svc->port_cnt--;
		else if (svc->svc_cnt)
			svc->svc_cnt--;
		if (!svc->port_cnt && !svc->svc_cnt) {
			client[dest_id][client_id].svc_cnt--;
			svc->need_reset = 0x0;
		}
	} else if (client[dest_id][client_id].svc_cnt > 0) {
		client[dest_id][client_id].svc_cnt--;
		if (!client[dest_id][client_id].svc_cnt) {
			svc->need_reset = 0x0;
			pr_debug("%s: service is reset %p\n", __func__, svc);
		}
	}

	if (!svc->port_cnt && !svc->svc_cnt) {
		if (apr_vm_rel_svc(svc->dest_domain, svc->id, svc->vm_handle))
			pr_err("%s: apr_vm_rel_svc failed\n", __func__);
		svc->priv = NULL;
		svc->id = 0;
		svc->vm_dest_svc = 0;
		svc->fn = NULL;
		svc->dest_id = 0;
		svc->client_id = 0;
		svc->need_reset = 0x0;
		svc->vm_handle = 0;
	}
	mutex_unlock(&svc->m_lock);

	return 0;
}
EXPORT_SYMBOL(apr_deregister);

/**
 * apr_reset - sets up workqueue to de-register
 * the given APR service handle.
 *
 * @handle: APR service handle
 *
 */
void apr_reset(void *handle)
{
	struct apr_reset_work *apr_reset_worker = NULL;

	if (!handle)
		return;
	pr_debug("%s: handle[%pK]\n", __func__, handle);

	if (apr_reset_workqueue == NULL) {
		pr_err("%s: apr_reset_workqueue is NULL\n", __func__);
		return;
	}

	apr_reset_worker = kzalloc(sizeof(struct apr_reset_work),
							GFP_ATOMIC);

	if (apr_reset_worker == NULL) {
		pr_err("%s: mem failure\n", __func__);
		return;
	}

	apr_reset_worker->handle = handle;
	INIT_WORK(&apr_reset_worker->work, apr_reset_deregister);
	queue_work(apr_reset_workqueue, &apr_reset_worker->work);
}
EXPORT_SYMBOL(apr_reset);

/* Dispatch the Reset events to Modem and audio clients */
static void dispatch_event(unsigned long code, uint16_t proc)
{
	struct apr_client *apr_client;
	struct apr_client_data data;
	struct apr_svc *svc;
	uint16_t clnt;
	int i, j;

	memset(&data, 0, sizeof(data));
	data.opcode = RESET_EVENTS;
	data.reset_event = code;

	/* Service domain can be different from the processor */
	data.reset_proc = apr_get_reset_domain(proc);

	clnt = APR_CLIENT_AUDIO;
	apr_client = &client[proc][clnt];
	for (i = 0; i < APR_SVC_MAX; i++) {
		mutex_lock(&apr_client->svc[i].m_lock);
		if (apr_client->svc[i].fn) {
			apr_client->svc[i].need_reset = 0x1;
			apr_client->svc[i].fn(&data, apr_client->svc[i].priv);
		}
		if (apr_client->svc[i].port_cnt) {
			svc = &(apr_client->svc[i]);
			svc->need_reset = 0x1;
			for (j = 0; j < APR_MAX_PORTS; j++)
				if (svc->port_fn[j])
					svc->port_fn[j](&data,
						svc->port_priv[j]);
		}
		mutex_unlock(&apr_client->svc[i].m_lock);
	}

	clnt = APR_CLIENT_VOICE;
	apr_client = &client[proc][clnt];
	for (i = 0; i < APR_SVC_MAX; i++) {
		mutex_lock(&apr_client->svc[i].m_lock);
		if (apr_client->svc[i].fn) {
			apr_client->svc[i].need_reset = 0x1;
			apr_client->svc[i].fn(&data, apr_client->svc[i].priv);
		}
		if (apr_client->svc[i].port_cnt) {
			svc = &(apr_client->svc[i]);
			svc->need_reset = 0x1;
			for (j = 0; j < APR_MAX_PORTS; j++)
				if (svc->port_fn[j])
					svc->port_fn[j](&data,
						svc->port_priv[j]);
		}
		mutex_unlock(&apr_client->svc[i].m_lock);
	}
}

static int apr_notifier_service_cb(struct notifier_block *this,
				   unsigned long opcode, void *data)
{
	struct audio_notifier_cb_data *cb_data = data;

	if (cb_data == NULL) {
		pr_err("%s: Callback data is NULL!\n", __func__);
		goto done;
	}

	pr_debug("%s: Service opcode 0x%lx, domain %d\n",
		__func__, opcode, cb_data->domain);

	switch (opcode) {
	case AUDIO_NOTIFIER_SERVICE_DOWN:
		/*
		 * Use flag to ignore down notifications during
		 * initial boot. There is no benefit from error
		 * recovery notifications during initial boot
		 * up since everything is expected to be down.
		 */
		spin_lock(&apr_priv->apr_lock);
		if (apr_priv->is_initial_boot) {
			spin_unlock(&apr_priv->apr_lock);
			break;
		}
		spin_unlock(&apr_priv->apr_lock);
		if (cb_data->domain == AUDIO_NOTIFIER_MODEM_DOMAIN)
			apr_modem_down(opcode);
		else
			apr_adsp_down(opcode);
		break;
	case AUDIO_NOTIFIER_SERVICE_UP:
		if (cb_data->domain == AUDIO_NOTIFIER_MODEM_DOMAIN)
			apr_modem_up();
		else
			apr_adsp_up();
		spin_lock(&apr_priv->apr_lock);
		apr_priv->is_initial_boot = false;
		spin_unlock(&apr_priv->apr_lock);
		break;
	default:
		break;
	}
done:
	return NOTIFY_OK;
}

static struct notifier_block adsp_service_nb = {
	.notifier_call  = apr_notifier_service_cb,
	.priority = 0,
};

static struct notifier_block modem_service_nb = {
	.notifier_call  = apr_notifier_service_cb,
	.priority = 0,
};

#ifdef CONFIG_DEBUG_FS
static int __init apr_debug_init(void)
{
	debugfs_apr_debug = debugfs_create_file("msm_apr_debug",
						 S_IFREG | 0444, NULL, NULL,
						 &apr_debug_ops);
	return 0;
}
#else
static int __init apr_debug_init(void)
{
	return 0;
}
#endif

static void apr_cleanup(void)
{
	int i, j, k;

	of_platform_depopulate(apr_priv->dev);
	subsys_notif_deregister(subsys_name);
	if (apr_reset_workqueue) {
		flush_workqueue(apr_reset_workqueue);
		destroy_workqueue(apr_reset_workqueue);
	}
	mutex_destroy(&q6.lock);
	for (i = 0; i < APR_DEST_MAX; i++) {
		for (j = 0; j < APR_CLIENT_MAX; j++) {
			mutex_destroy(&client[i][j].m_lock);
			for (k = 0; k < APR_SVC_MAX; k++)
				mutex_destroy(&client[i][j].svc[k].m_lock);
		}
	}
#ifdef CONFIG_DEBUG_FS
	debugfs_remove(debugfs_apr_debug);
#endif
}

static int apr_probe(struct platform_device *pdev)
{
	int i, j, k, ret = 0;

	init_waitqueue_head(&modem_wait);

	apr_priv = devm_kzalloc(&pdev->dev, sizeof(*apr_priv), GFP_KERNEL);
	if (!apr_priv)
		return -ENOMEM;

	apr_priv->dev = &pdev->dev;
	spin_lock_init(&apr_priv->apr_lock);
	INIT_WORK(&apr_priv->add_chld_dev_work, apr_add_child_devices);

	/* open apr channel tx and rx, store as global */
	ret = habmm_socket_open(&hab_handle_tx,
			MM_AUD_1,
			0xFFFFFFFF,
			HABMM_SOCKET_OPEN_FLAGS_SINGLE_BE_SINGLE_FE);
	if (ret) {
		pr_err("%s: habmm_socket_open tx failed %d\n", __func__, ret);
		return ret;
	}
	spin_lock_init(&hab_tx_lock);

	ret = habmm_socket_open(&hab_handle_rx,
			MM_AUD_2,
			0xFFFFFFFF,
			HABMM_SOCKET_OPEN_FLAGS_SINGLE_BE_SINGLE_FE);
	if (ret) {
		pr_err("%s: habmm_socket_open rx failed %d\n", __func__, ret);
		habmm_socket_close(hab_handle_tx);
		return ret;
	}
	pr_info("%s: hab_handle_tx %x hab_handle_rx %x\n",
			__func__, hab_handle_tx, hab_handle_rx);

	/* create apr ch rx cb thread */
	apr_vm_cb_thread_task = kthread_run(apr_vm_cb_thread,
			NULL,
			APR_VM_CB_THREAD_NAME);
	if (IS_ERR(apr_vm_cb_thread_task)) {
		ret = PTR_ERR(apr_vm_cb_thread_task);
		pr_err("%s: kthread_run failed %d\n", __func__, ret);
		habmm_socket_close(hab_handle_tx);
		habmm_socket_close(hab_handle_rx);
	    return ret;
	}
	pid = apr_vm_cb_thread_task->pid;
	pr_info("%s: apr_vm_cb_thread started pid %d\n",
			__func__, pid);

	for (i = 0; i < APR_DEST_MAX; i++)
		for (j = 0; j < APR_CLIENT_MAX; j++) {
			mutex_init(&client[i][j].m_lock);
			for (k = 0; k < APR_SVC_MAX; k++) {
				mutex_init(&client[i][j].svc[k].m_lock);
			}
		}
	spin_lock(&apr_priv->apr_lock);
	apr_priv->is_initial_boot = true;
	spin_unlock(&apr_priv->apr_lock);
	apr_vm_set_subsys_state();
	mutex_init(&q6.lock);
	apr_reset_workqueue = create_singlethread_workqueue("apr_driver");
	if (!apr_reset_workqueue) {
		habmm_socket_close(hab_handle_tx);
		habmm_socket_close(hab_handle_rx);
		kthread_stop(apr_vm_cb_thread_task);
		apr_priv = NULL;
		return -ENOMEM;
	}

#ifdef CONFIG_IPC_LOGGING
	apr_pkt_ctx = ipc_log_context_create(APR_PKT_IPC_LOG_PAGE_CNT,
						"apr", 0);
	if (!apr_pkt_ctx)
		pr_err("%s: Unable to create ipc log context\n", __func__);
#endif  /* CONFIG_IPC_LOGGING */

	ret = of_property_read_string(pdev->dev.of_node,
				      "qcom,subsys-name",
				      (const char **)(&subsys_name));
	if (ret) {
		pr_err("%s: missing subsys-name entry in dt node\n", __func__);
		return -EINVAL;
	}

	if (!strcmp(subsys_name, "apr_adsp")) {
		subsys_notif_register("apr_adsp",
				       AUDIO_NOTIFIER_ADSP_DOMAIN,
				       &adsp_service_nb);
	} else if (!strcmp(subsys_name, "apr_modem")) {
		subsys_notif_register("apr_modem",
				       AUDIO_NOTIFIER_MODEM_DOMAIN,
				       &modem_service_nb);
	} else {
		pr_err("%s: invalid subsys-name %s\n", __func__, subsys_name);
		return -EINVAL;
	}

	ret = snd_event_client_register(&pdev->dev, &apr_ssr_ops, NULL);
	if (ret) {
		pr_err("%s: Registration with SND event fwk failed ret = %d\n",
			__func__, ret);
		ret = 0;
	} else {
		snd_event_notify(apr_priv->dev, SND_EVENT_UP);
	}

	return apr_debug_init();
}

static int apr_remove(struct platform_device *pdev)
{
	habmm_socket_close(hab_handle_tx);
	habmm_socket_close(hab_handle_rx);
	kthread_stop(apr_vm_cb_thread_task);
	snd_event_client_deregister(&pdev->dev);
	apr_cleanup();
	apr_priv = NULL;
	return 0;
}

static const struct of_device_id apr_machine_of_match[]  = {
	{ .compatible = "qcom,msm-audio-apr", },
	{},
};

static struct platform_driver apr_driver = {
	.probe = apr_probe,
	.remove = apr_remove,
	.driver = {
		.name = "audio_apr",
		.owner = THIS_MODULE,
		.of_match_table = apr_machine_of_match,
	}
};

module_platform_driver(apr_driver);

MODULE_DESCRIPTION("APR DRIVER");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, apr_machine_of_match);
