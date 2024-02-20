// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/time.h>
#include <linux/atomic.h>
#include <linux/mm.h>

#include <asm/ioctls.h>

#include <linux/memory.h>

#include <sound/compress_params.h>

#include <dsp/msm_audio_ion.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/audio_cal_utils.h>
#include <dsp/q6asm-v2.h>
#include <dsp/q6audio-v2.h>
#include <dsp/q6common.h>
#include <dsp/q6core.h>
#include "adsp_err.h"

#define TIMEOUT_MS  1000
#define TRUE        0x01
#define FALSE       0x00
#define SESSION_MAX 8

#define ENC_FRAMES_PER_BUFFER 0x01

enum {
	ASM_TOPOLOGY_CAL = 0,
	ASM_CUSTOM_TOP_CAL,
	ASM_AUDSTRM_CAL,
	ASM_RTAC_APR_CAL,
	ASM_MAX_CAL_TYPES
};

union asm_token_struct {
	struct {
		u8 stream_id;
		u8 session_id;
		u8 buf_index;
		u8 flags;
	} _token;
	u32 token;
} __packed;


enum {
	ASM_DIRECTION_OFFSET,
	ASM_CMD_NO_WAIT_OFFSET,
	/*
	 * Offset is limited to 7 because flags is stored in u8
	 * field in asm_token_structure defined above. The offset
	 * starts from 0.
	 */
	ASM_MAX_OFFSET = 7,
};

enum {
	WAIT_CMD,
	NO_WAIT_CMD
};

#define ASM_SET_BIT(n, x)	(n |= 1 << x)
#define ASM_TEST_BIT(n, x)	((n >> x) & 1)

/* TODO, combine them together */
static DEFINE_MUTEX(session_lock);
struct asm_mmap {
	atomic_t ref_cnt;
	void *apr;
};

static struct asm_mmap this_mmap;

struct audio_session {
	struct audio_client *ac;
	spinlock_t session_lock;
	struct mutex mutex_lock_per_session;
	bool ignore;
};
/* session id: 0 reserved */
static struct audio_session session[ASM_ACTIVE_STREAMS_ALLOWED + 1];

struct asm_buffer_node {
	struct list_head list;
	phys_addr_t buf_phys_addr;
	uint32_t  mmap_hdl;
};
static int32_t q6asm_srvc_callback(struct apr_client_data *data, void *priv);
static int32_t q6asm_callback(struct apr_client_data *data, void *priv);
static void q6asm_add_hdr(struct audio_client *ac, struct apr_hdr *hdr,
			uint32_t pkt_size, uint32_t cmd_flg);
static void q6asm_add_hdr_custom_topology(struct audio_client *ac,
					  struct apr_hdr *hdr,
					  uint32_t pkt_size);
static void q6asm_add_hdr_async(struct audio_client *ac, struct apr_hdr *hdr,
			uint32_t pkt_size, uint32_t cmd_flg);
static int q6asm_memory_map_regions(struct audio_client *ac, int dir,
				uint32_t bufsz, uint32_t bufcnt,
				bool is_contiguous);
static int q6asm_memory_unmap_regions(struct audio_client *ac, int dir);
static void q6asm_reset_buf_state(struct audio_client *ac);

void *q6asm_mmap_apr_reg(void);

static int q6asm_is_valid_session(struct apr_client_data *data, void *priv);
static int q6asm_get_asm_topology_apptype(struct q6asm_cal_info *cal_info, struct audio_client *ac);

/* for ASM custom topology */
static struct cal_type_data *cal_data[ASM_MAX_CAL_TYPES];
static struct audio_buffer common_buf[2];
static struct audio_client common_client;
static int set_custom_topology;
static int topology_map_handle;

struct generic_get_data_ {
	int valid;
	int is_inband;
	int size_in_ints;
	int ints[];
};
static struct generic_get_data_ *generic_get_data;

static uint32_t adsp_reg_event_opcode[] = {
	ASM_STREAM_CMD_REGISTER_PP_EVENTS,
	ASM_STREAM_CMD_REGISTER_ENCDEC_EVENTS,
	ASM_STREAM_CMD_REGISTER_IEC_61937_FMT_UPDATE };

static uint32_t adsp_raise_event_opcode[] = {
	ASM_STREAM_PP_EVENT,
	ASM_STREAM_CMD_ENCDEC_EVENTS,
	ASM_IEC_61937_MEDIA_FMT_EVENT };

static int is_adsp_reg_event(uint32_t cmd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adsp_reg_event_opcode); i++) {
		if (cmd == adsp_reg_event_opcode[i])
			return i;
	}
	return -EINVAL;
}

static int is_adsp_raise_event(uint32_t cmd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(adsp_raise_event_opcode); i++) {
		if (cmd == adsp_raise_event_opcode[i])
			return i;
	}
	return -EINVAL;
}
static inline void q6asm_set_flag_in_token(union asm_token_struct *asm_token,
					   int flag, int flag_offset)
{
	if (flag)
		ASM_SET_BIT(asm_token->_token.flags, flag_offset);
}

static inline void q6asm_update_token(u32 *token, u8 session_id, u8 stream_id,
				      u8 buf_index, u8 dir, u8 nowait_flag)
{
	union asm_token_struct asm_token;

	asm_token.token = 0;
	asm_token._token.session_id = session_id;
	asm_token._token.stream_id = stream_id;
	asm_token._token.buf_index = buf_index;
	q6asm_set_flag_in_token(&asm_token, dir, ASM_DIRECTION_OFFSET);
	q6asm_set_flag_in_token(&asm_token, nowait_flag,
				  ASM_CMD_NO_WAIT_OFFSET);
	*token = asm_token.token;
}
static inline uint32_t q6asm_get_pcm_format_id(uint32_t media_format_block_ver)
{
	uint32_t pcm_format_id;

	switch (media_format_block_ver) {
	case PCM_MEDIA_FORMAT_V5:
		pcm_format_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V5;
		break;
	case PCM_MEDIA_FORMAT_V4:
		pcm_format_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V4;
		break;
	case PCM_MEDIA_FORMAT_V3:
		pcm_format_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V3;
		break;
	case PCM_MEDIA_FORMAT_V2:
	default:
		pcm_format_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2;
		break;
	}
	return pcm_format_id;
}
/*
 * q6asm_get_buf_index_from_token:
 *       Retrieve buffer index from token.
 *
 * @token: token value sent to ASM service on q6.
 * Returns buffer index in the read/write commands.
 */
uint8_t q6asm_get_buf_index_from_token(uint32_t token)
{
	union asm_token_struct asm_token;

	asm_token.token = token;
	return asm_token._token.buf_index;
}
EXPORT_SYMBOL(q6asm_get_buf_index_from_token);
/*
 * q6asm_get_stream_id_from_token:
 *       Retrieve stream id from token.
 *
 * @token: token value sent to ASM service on q6.
 * Returns stream id.
 */
uint8_t q6asm_get_stream_id_from_token(uint32_t token)
{
	union asm_token_struct asm_token;

	asm_token.token = token;
	return asm_token._token.stream_id;
}
EXPORT_SYMBOL(q6asm_get_stream_id_from_token);

static inline int q6asm_get_flag_from_token(union asm_token_struct *asm_token,
					    int flag_offset)
{
	return ASM_TEST_BIT(asm_token->_token.flags, flag_offset);
}


#ifdef CONFIG_DEBUG_FS
#define OUT_BUFFER_SIZE 56
#define IN_BUFFER_SIZE 24

static struct timeval out_cold_tv;
static struct timeval out_warm_tv;
static struct timeval out_cont_tv;
static struct timeval in_cont_tv;
static long out_enable_flag;
static long in_enable_flag;
static struct dentry *out_dentry;
static struct dentry *in_dentry;
static int in_cont_index;
/*This var is used to keep track of first write done for cold output latency */
static int out_cold_index;
static char *out_buffer;
static char *in_buffer;

static int audio_output_latency_dbgfs_open(struct inode *inode,
							struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t audio_output_latency_dbgfs_read(struct file *file,
				char __user *buf, size_t count, loff_t *ppos)
{
	if (out_buffer == NULL) {
		pr_err("%s: out_buffer is null\n", __func__);
		return 0;
	}
	if (count < OUT_BUFFER_SIZE) {
		pr_err("%s: read size %d exceeds buf size %zd\n", __func__,
						OUT_BUFFER_SIZE, count);
		return 0;
	}
	snprintf(out_buffer, OUT_BUFFER_SIZE, "%ld,%ld,%ld,%ld,%ld,%ld,",
		out_cold_tv.tv_sec, out_cold_tv.tv_usec, out_warm_tv.tv_sec,
		out_warm_tv.tv_usec, out_cont_tv.tv_sec, out_cont_tv.tv_usec);
	return  simple_read_from_buffer(buf, OUT_BUFFER_SIZE, ppos,
						out_buffer, OUT_BUFFER_SIZE);
}
static ssize_t audio_output_latency_dbgfs_write(struct file *file,
			const char __user *buf, size_t count, loff_t *ppos)
{
	char *temp;

	if (count > 2*sizeof(char)) {
		pr_err("%s: err count is more %zd\n", __func__, count);
		return -EINVAL;
	}
	temp  = kmalloc(2*sizeof(char), GFP_KERNEL);

	out_cold_index = 0;

	if (temp) {
		if (copy_from_user(temp, buf, 2*sizeof(char))) {
			pr_err("%s: copy from user failed for size %zd\n",
				__func__, 2*sizeof(char));
			kfree(temp);
			return -EFAULT;
		}
		if (!kstrtol(temp, 10, &out_enable_flag)) {
			kfree(temp);
			return count;
		}
		kfree(temp);
	}
	return -EINVAL;
}
static const struct file_operations audio_output_latency_debug_fops = {
	.open = audio_output_latency_dbgfs_open,
	.read = audio_output_latency_dbgfs_read,
	.write = audio_output_latency_dbgfs_write
};
static int audio_input_latency_dbgfs_open(struct inode *inode,
							struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t audio_input_latency_dbgfs_read(struct file *file,
				char __user *buf, size_t count, loff_t *ppos)
{
	if (in_buffer == NULL) {
		pr_err("%s: in_buffer is null\n", __func__);
		return 0;
	}
	if (count < IN_BUFFER_SIZE) {
		pr_err("%s: read size %d exceeds buf size %zd\n", __func__,
						IN_BUFFER_SIZE, count);
		return 0;
	}
	snprintf(in_buffer, IN_BUFFER_SIZE, "%ld,%ld,",
				in_cont_tv.tv_sec, in_cont_tv.tv_usec);
	return  simple_read_from_buffer(buf, IN_BUFFER_SIZE, ppos,
						in_buffer, IN_BUFFER_SIZE);
}
static ssize_t audio_input_latency_dbgfs_write(struct file *file,
			const char __user *buf, size_t count, loff_t *ppos)
{
	char *temp;

	if (count > 2*sizeof(char)) {
		pr_err("%s: err count is more %zd\n", __func__, count);
		return -EINVAL;
	}
	temp  = kmalloc(2*sizeof(char), GFP_KERNEL);

	if (temp) {
		if (copy_from_user(temp, buf, 2*sizeof(char))) {
			pr_err("%s: copy from user failed for size %zd\n",
				__func__, 2*sizeof(char));
			kfree(temp);
			return -EFAULT;
		}
		if (!kstrtol(temp, 10, &in_enable_flag)) {
			kfree(temp);
			return count;
		}
		kfree(temp);
	}
	return -EINVAL;
}
static const struct file_operations audio_input_latency_debug_fops = {
	.open = audio_input_latency_dbgfs_open,
	.read = audio_input_latency_dbgfs_read,
	.write = audio_input_latency_dbgfs_write
};

/*
 * get_monotonic_timeval -
 *       This method returns a structure in timeval
 *       format (sec,microsec) by using ktime kernel
 *       API to get time in nano secs and then converts
 *       it to timeval format
 *
 * ktime_get [nsec]-> ktime_to_timespec [sec,nsec]-> timeval[sec,usec]
 *
 * Returns struct timeval
*/
static struct timeval get_monotonic_timeval(void)
{
	static struct timeval out_tval;

	/* Get time from monotonic clock in nanoseconds */
	ktime_t kTimeNsec = ktime_get();

	/* Convert it to timespec format and later to timeval as expected by audio HAL */
	struct timespec temp_tspec = ktime_to_timespec(kTimeNsec);

	/* Time returned above is in sec,nanosec format, needs to convert to sec,microsec */
	out_tval.tv_usec = temp_tspec.tv_nsec/1000;
	out_tval.tv_sec = temp_tspec.tv_sec;
	return out_tval;
}

static void config_debug_fs_write_cb(void)
{
	if (out_enable_flag) {
		/* For first Write done log the time and reset
		 * out_cold_index
		 */
		if (out_cold_index != 1) {
			out_cold_tv = get_monotonic_timeval();
			pr_debug("COLD: apr_send_pkt at %ld sec %ld microsec\n",
				out_cold_tv.tv_sec,
				out_cold_tv.tv_usec);
			out_cold_index = 1;
		}
		pr_debug("%s: out_enable_flag %ld\n",
			__func__, out_enable_flag);
	}
}
static void config_debug_fs_read_cb(void)
{
	if (in_enable_flag) {
		/* when in_cont_index == 7, DSP would be
		 * writing into the 8th 512 byte buffer and this
		 * timestamp is tapped here.Once done it then writes
		 * to 9th 512 byte buffer.These two buffers(8th, 9th)
		 * reach the test application in 5th iteration and that
		 * timestamp is tapped at user level. The difference
		 * of these two timestamps gives us the time between
		 * the time at which dsp started filling the sample
		 * required and when it reached the test application.
		 * Hence continuous input latency
		 */
		if (in_cont_index == 7) {
			in_cont_tv = get_monotonic_timeval();
			pr_info("%s: read buffer at %ld sec %ld microsec\n",
				__func__,
				in_cont_tv.tv_sec, in_cont_tv.tv_usec);
		}
		in_cont_index++;
	}
}

static void config_debug_fs_reset_index(void)
{
	in_cont_index = 0;
}

static void config_debug_fs_run(void)
{
	if (out_enable_flag) {
		out_cold_tv = get_monotonic_timeval();
		pr_debug("%s: COLD apr_send_pkt at %ld sec %ld microsec\n",
			__func__, out_cold_tv.tv_sec, out_cold_tv.tv_usec);
	}
}

static void config_debug_fs_write(struct audio_buffer *ab)
{
	if (out_enable_flag) {
		char zero_pattern[2] = {0x00, 0x00};
		/* If First two byte is non zero and last two byte
		 * is zero then it is warm output pattern
		 */
		if ((strcmp(((char *)ab->data), zero_pattern)) &&
		(!strcmp(((char *)ab->data + 2), zero_pattern))) {
			out_warm_tv = get_monotonic_timeval();
			pr_debug("%s: WARM:apr_send_pkt at %ld sec %ld microsec\n",
			 __func__,
			 out_warm_tv.tv_sec,
			out_warm_tv.tv_usec);
			pr_debug("%s: Warm Pattern Matched\n", __func__);
		}
		/* If First two byte is zero and last two byte is
		 * non zero then it is cont output pattern
		 */
		else if ((!strcmp(((char *)ab->data), zero_pattern))
		&& (strcmp(((char *)ab->data + 2), zero_pattern))) {
			out_cont_tv = get_monotonic_timeval();
			pr_debug("%s: CONT:apr_send_pkt at %ld sec %ld microsec\n",
			__func__,
			out_cont_tv.tv_sec,
			out_cont_tv.tv_usec);
			pr_debug("%s: Cont Pattern Matched\n", __func__);
		}
	}
}
static void config_debug_fs_init(void)
{
	out_buffer = kzalloc(OUT_BUFFER_SIZE, GFP_KERNEL);
	if (out_buffer == NULL)
		goto outbuf_fail;

	in_buffer = kzalloc(IN_BUFFER_SIZE, GFP_KERNEL);
	if (in_buffer == NULL)
		goto inbuf_fail;

	out_dentry = debugfs_create_file("audio_out_latency_measurement_node",
				0664,
				NULL, NULL, &audio_output_latency_debug_fops);
	if (IS_ERR(out_dentry)) {
		pr_err("%s: debugfs_create_file failed\n", __func__);
		goto file_fail;
	}
	in_dentry = debugfs_create_file("audio_in_latency_measurement_node",
				0664,
				NULL, NULL, &audio_input_latency_debug_fops);
	if (IS_ERR(in_dentry)) {
		pr_err("%s: debugfs_create_file failed\n", __func__);
		goto file_fail;
	}
	return;
file_fail:
	kfree(in_buffer);
inbuf_fail:
	kfree(out_buffer);
outbuf_fail:
	in_buffer = NULL;
	out_buffer = NULL;
}
#else
static void config_debug_fs_write(struct audio_buffer *ab)
{
}
static void config_debug_fs_run(void)
{
}
static void config_debug_fs_reset_index(void)
{
}
static void config_debug_fs_read_cb(void)
{
}
static void config_debug_fs_write_cb(void)
{
}
static void config_debug_fs_init(void)
{
}

#endif /*CONFIG_DEBUG_FS*/

int q6asm_mmap_apr_dereg(void)
{
	int c;

	c = atomic_sub_return(1, &this_mmap.ref_cnt);
	if (c == 0) {
		apr_deregister(this_mmap.apr);
		common_client.mmap_apr = NULL;
		pr_debug("%s: APR De-Register common port\n", __func__);
	} else if (c < 0) {
		pr_err("%s: APR Common Port Already Closed %d\n",
			__func__, c);
		atomic_set(&this_mmap.ref_cnt, 0);
	}

	return 0;
}

static int q6asm_session_alloc(struct audio_client *ac)
{
	int n;

	for (n = 1; n <= ASM_ACTIVE_STREAMS_ALLOWED; n++) {
		if (!(session[n].ac)) {
			session[n].ac = ac;
			return n;
		}
	}
	pr_err("%s: session not available\n", __func__);
	return -ENOMEM;
}

static void q6asm_session_set_ignore(int id)
{
	session[id].ignore = true;
}

static void q6asm_session_clean_ignore(void)
{
	int i;

	for (i = 1; i <= ASM_ACTIVE_STREAMS_ALLOWED; i++)
		session[i].ignore = false;
}

static void q6asm_session_deregister(struct audio_client *ac)
{
	rtac_set_asm_handle(ac->session, NULL);
	apr_deregister(ac->apr2);
	apr_deregister(ac->apr);
	q6asm_mmap_apr_dereg();
	ac->apr2 = NULL;
	ac->apr = NULL;
	ac->mmap_apr = NULL;
}

static int q6asm_session_register(struct audio_client *ac)
{
	ac->apr = apr_register("ADSP", "ASM",
			(apr_fn)q6asm_callback,
			((ac->session) << 8 | 0x0001),
			ac);
	if (ac->apr == NULL) {
		pr_err("%s: Registration with APR failed\n", __func__);
		goto fail_apr1;
	}

	ac->apr2 = apr_register("ADSP", "ASM",
			(apr_fn)q6asm_callback,
			((ac->session) << 8 | 0x0002),
			ac);
	if (ac->apr2 == NULL) {
		pr_err("%s: Registration with APR-2 failed\n", __func__);
		goto fail_apr2;
	}

	rtac_set_asm_handle(ac->session, ac->apr);

	pr_debug("%s: Registering the common port with APR\n", __func__);
	ac->mmap_apr = q6asm_mmap_apr_reg();
	if (ac->mmap_apr == NULL)
		goto fail_mmap;

	return 0;

fail_mmap:
	apr_deregister(ac->apr2);
fail_apr2:
	apr_deregister(ac->apr);
fail_apr1:
	return -EINVAL;
}

static int q6asm_session_try_next(struct audio_client *ac)
{
	int n;
	int s = ac->session;

	for (n = 1; n <= ASM_ACTIVE_STREAMS_ALLOWED; n++) {
		if (++s > ASM_ACTIVE_STREAMS_ALLOWED)
			s -= ASM_ACTIVE_STREAMS_ALLOWED;
		if (!session[s].ignore && !session[s].ac) {
			q6asm_session_deregister(ac);
			session[ac->session].ac = NULL;
			session[s].ac = ac;
			ac->session = s;
			return q6asm_session_register(ac);
		}
	}
	pr_debug("%s: session not available\n", __func__);
	return -EINVAL;
}

static int q6asm_get_session_id_from_audio_client(struct audio_client *ac)
{
	int n;

	for (n = 1; n <= ASM_ACTIVE_STREAMS_ALLOWED; n++) {
		if (session[n].ac == ac)
			return n;
	}

	pr_debug("%s: cannot find matching audio client. ac = %pK\n",
		__func__, ac);

	return 0;
}

static bool q6asm_is_valid_audio_client(struct audio_client *ac)
{
	return q6asm_get_session_id_from_audio_client(ac) ? 1 : 0;
}

static void q6asm_session_free(struct audio_client *ac)
{
	int session_id;
	unsigned long flags = 0;

	pr_debug("%s: sessionid[%d]\n", __func__, ac->session);
	session_id = ac->session;
	mutex_lock(&session[session_id].mutex_lock_per_session);
	rtac_remove_popp_from_adm_devices(ac->session);
	spin_lock_irqsave(&(session[session_id].session_lock), flags);
	session[ac->session].ac = NULL;
	ac->session = 0;
	ac->perf_mode = LEGACY_PCM_MODE;
	ac->fptr_cache_ops = NULL;
	ac->cb = NULL;
	ac->priv = NULL;
	kfree(ac);
	ac = NULL;
	spin_unlock_irqrestore(&(session[session_id].session_lock), flags);
	mutex_unlock(&session[session_id].mutex_lock_per_session);
}

static uint32_t q6asm_get_next_buf(struct audio_client *ac,
		uint32_t curr_buf, uint32_t max_buf_cnt)
{
	dev_vdbg(ac->dev, "%s: curr_buf = %d, max_buf_cnt = %d\n",
		 __func__, curr_buf, max_buf_cnt);
	curr_buf += 1;
	return (curr_buf >= max_buf_cnt) ? 0 : curr_buf;
}

static int q6asm_map_cal_memory(int32_t cal_type,
	struct cal_block_data *cal_block)
{
	int result = 0;
	struct asm_buffer_node *buf_node = NULL;
	struct list_head *ptr, *next;

	if (cal_block == NULL) {
		pr_err("%s: cal_block is NULL!\n",
			__func__);
		goto done;
	}

	if (cal_block->cal_data.paddr == 0) {
		pr_debug("%s: No address to map!\n",
			__func__);
		goto done;
	}

	common_client.mmap_apr = q6asm_mmap_apr_reg();
	if (common_client.mmap_apr == NULL) {
		pr_err("%s: q6asm_mmap_apr_reg failed\n",
			__func__);
		result = -EPERM;
		goto done;
	}
	common_client.apr = common_client.mmap_apr;
	if (cal_block->map_data.map_size == 0) {
		pr_debug("%s: map size is 0!\n",
			__func__);
		goto done;
	}

	/* Use first asm buf to map memory */
	if (common_client.port[IN].buf == NULL) {
		pr_err("%s: common buf is NULL\n",
			__func__);
		result = -EINVAL;
		goto done;
	}

	common_client.port[IN].buf->phys = cal_block->cal_data.paddr;

	result = q6asm_memory_map_regions(&common_client,
			IN, cal_block->map_data.map_size, 1, 1);
	if (result < 0) {
		pr_err("%s: mmap did not work! size = %zd result %d\n",
			__func__,
			cal_block->map_data.map_size, result);
		pr_debug("%s: mmap did not work! addr = 0x%pK, size = %zd\n",
			__func__,
			&cal_block->cal_data.paddr,
			cal_block->map_data.map_size);
		goto done;
	}

	list_for_each_safe(ptr, next,
		&common_client.port[IN].mem_map_handle) {
		buf_node = list_entry(ptr, struct asm_buffer_node,
					list);
		if (buf_node->buf_phys_addr == cal_block->cal_data.paddr) {
			cal_block->map_data.q6map_handle =  buf_node->mmap_hdl;
			break;
		}
	}
done:
	return result;
}

static int remap_cal_data(int32_t cal_type, struct cal_block_data *cal_block)
{
	int ret = 0;

	if (cal_block->map_data.dma_buf == NULL) {
		pr_err("%s: No ION allocation for cal type %d!\n",
			__func__, cal_type);
		ret = -EINVAL;
		goto done;
	}

	if ((cal_block->map_data.map_size > 0) &&
		(cal_block->map_data.q6map_handle == 0)) {

		ret = q6asm_map_cal_memory(cal_type, cal_block);
		if (ret < 0) {
			pr_err("%s: mmap did not work! size = %zd ret %d\n",
				__func__, cal_block->map_data.map_size, ret);
			goto done;
		}
	}
done:
	return ret;
}

static int q6asm_unmap_cal_memory(int32_t cal_type,
	struct cal_block_data *cal_block)
{
	int			result = 0;
	int			result2 = 0;

	if (cal_block == NULL) {
		pr_err("%s: cal_block is NULL!\n",
			__func__);
		result = -EINVAL;
		goto done;
	}

	if (cal_block->map_data.q6map_handle == 0) {
		pr_debug("%s: No address to unmap!\n",
			__func__);
		result = -EINVAL;
		goto done;
	}

	if (common_client.mmap_apr == NULL) {
		common_client.mmap_apr = q6asm_mmap_apr_reg();
		if (common_client.mmap_apr == NULL) {
			pr_err("%s: q6asm_mmap_apr_reg failed\n",
				__func__);
			result = -EPERM;
			goto done;
		}
	}

	common_client.port[IN].buf->phys = cal_block->cal_data.paddr;

	result2 = q6asm_memory_unmap_regions(&common_client, IN);
	if (result2 < 0) {
		pr_err("%s: unmap failed, err %d\n",
			__func__, result2);
		result = result2;
	}

	cal_block->map_data.q6map_handle = 0;
done:
	return result;
}

static struct cal_block_data *q6asm_find_cal_by_app_type(int cal_index,
							  int app_type)
{
	struct list_head *ptr, *next;
	struct cal_block_data *cal_block = NULL;
	struct audio_cal_info_audstrm *audstrm_info = NULL;

	pr_debug("%s\n", __func__);

	list_for_each_safe(ptr, next,
			&cal_data[cal_index]->cal_blocks) {

		cal_block = list_entry(ptr,
				struct cal_block_data, list);

		if (cal_index == ASM_AUDSTRM_CAL) {
			audstrm_info = cal_block->cal_info;
			if (audstrm_info->app_type == app_type)
				return cal_block;
		}
	}
	pr_err("%s: Can't find ASM Cal for cal_index %d app_type %d\n",
			__func__, cal_index, app_type);
	return NULL;
}

static bool is_q6asm_topology_cal_send_by_v6(int path, int usecase)
{
	struct list_head *ptr, *next;
	struct cal_block_data *cal_block = NULL;
	int buffer_idx_asm;

	buffer_idx_asm = path + MAX_PATH_TYPE * usecase + ASM_CAL_SEND_VERSION_OFFSET;
	pr_debug("%s: path(%d), usecase(%d), buffer_idx_asm(%d)\n",
			__func__, path, usecase, buffer_idx_asm);

	list_for_each_safe(ptr, next,
			&cal_data[ASM_TOPOLOGY_CAL]->cal_blocks) {

		cal_block = list_entry(ptr,
				struct cal_block_data, list);

		if (cal_block != NULL) {
			if (cal_block->buffer_number == buffer_idx_asm)
				return true;
		}
	}
	return false;
}

static struct cal_block_data *q6asm_find_cal_by_buf_number(int cal_index,
							  int app_type, int path, int usecase)
{
	struct list_head *ptr, *next;
	struct cal_block_data *cal_block = NULL;
	struct audio_cal_info_audstrm *audstrm_info = NULL;
	int buffer_idx_w_path = usecase;

	pr_debug("%s\n", __func__);

	if (cal_index == ASM_AUDSTRM_CAL)
		buffer_idx_w_path = path + MAX_PATH_TYPE * usecase;

	list_for_each_safe(ptr, next,
			&cal_data[cal_index]->cal_blocks) {

		cal_block = list_entry(ptr,
				struct cal_block_data, list);

		if (cal_index == ASM_AUDSTRM_CAL) {
			audstrm_info = cal_block->cal_info;
			if ((audstrm_info->app_type == app_type) &&
				(cal_block->buffer_number == buffer_idx_w_path)) {
					return cal_block;
			}
		} else {
			if (cal_block->buffer_number == buffer_idx_w_path) {
					return cal_block;
			}
		}
	}
	pr_err("%s: Can't find ASM Cal for cal_index %d app_type %d buffer_number %d\n",
			__func__, cal_index, app_type, buffer_idx_w_path);

	return NULL;
}

int q6asm_unmap_cal_data(int cal_type, struct cal_block_data *cal_block)
{
	int ret = 0;

	if ((cal_block->map_data.map_size > 0) &&
		(cal_block->map_data.q6map_handle != 0)) {

		ret = q6asm_unmap_cal_memory(cal_type, cal_block);
		if (ret < 0) {
			pr_err("%s: unmap did not work! size = %zd ret %d\n",
				__func__, cal_block->map_data.map_size, ret);
			goto done;
		}
	}
done:
	return ret;
}

int send_asm_custom_topology(struct audio_client *ac)
{
	struct cal_block_data		*cal_block = NULL;
	struct cmd_set_topologies	asm_top;
	int result = 0;
	int result1 = 0;

	if (cal_data[ASM_CUSTOM_TOP_CAL] == NULL)
		goto done;

	mutex_lock(&cal_data[ASM_CUSTOM_TOP_CAL]->lock);
	if (!set_custom_topology)
		goto unlock;
	set_custom_topology = 0;

	cal_block = cal_utils_get_only_cal_block(cal_data[ASM_CUSTOM_TOP_CAL]);
	if (cal_block == NULL || cal_utils_is_cal_stale(cal_block))
		goto unlock;

	if (cal_block->cal_data.size == 0) {
		pr_debug("%s: No cal to send!\n", __func__);
		goto unlock;
	}

	pr_debug("%s: Sending cal_index %d\n", __func__, ASM_CUSTOM_TOP_CAL);

	result = remap_cal_data(ASM_CUST_TOPOLOGY_CAL_TYPE, cal_block);
	if (result) {
		pr_err("%s: Remap_cal_data failed for cal %d!\n",
			__func__, ASM_CUSTOM_TOP_CAL);
		goto unlock;
	}

	q6asm_add_hdr_custom_topology(ac, &asm_top.hdr, sizeof(asm_top));
	atomic_set(&ac->mem_state, -1);
	asm_top.hdr.opcode = ASM_CMD_ADD_TOPOLOGIES;
	asm_top.payload_addr_lsw = lower_32_bits(cal_block->cal_data.paddr);
	asm_top.payload_addr_msw = msm_audio_populate_upper_32_bits(
						cal_block->cal_data.paddr);
	asm_top.mem_map_handle = cal_block->map_data.q6map_handle;
	asm_top.payload_size = cal_block->cal_data.size;

	 pr_debug("%s: Sending ASM_CMD_ADD_TOPOLOGIES payload = %pK, size = %d, map handle = 0x%x\n",
		__func__, &cal_block->cal_data.paddr,
		asm_top.payload_size, asm_top.mem_map_handle);

	result = apr_send_pkt(ac->apr, (uint32_t *) &asm_top);
	if (result < 0) {
		pr_err("%s: Set topologies failed result %d\n",
			__func__, result);
		pr_debug("%s: Set topologies failed payload = 0x%pK\n",
			__func__, &cal_block->cal_data.paddr);
		goto unmap;

	}

	result = wait_event_timeout(ac->mem_wait,
			(atomic_read(&ac->mem_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!result) {
		pr_err("%s: Set topologies failed timeout\n", __func__);
		pr_debug("%s: Set topologies failed after timedout payload = 0x%pK\n",
			__func__, &cal_block->cal_data.paddr);
		result = -ETIMEDOUT;
		goto unmap;
	}
	if (atomic_read(&ac->mem_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			atomic_read(&ac->mem_state)));
		result = adsp_err_get_lnx_err_code(
			atomic_read(&ac->mem_state));
		goto unmap;
	}

unmap:
	result1 = q6asm_unmap_cal_memory(ASM_CUST_TOPOLOGY_CAL_TYPE,
		cal_block);
	if (result1 < 0) {
		result = result1;
		pr_debug("%s: unmap cal failed! %d\n", __func__, result);
	}
unlock:
	mutex_unlock(&cal_data[ASM_CUSTOM_TOP_CAL]->lock);
done:
	return result;
}

int q6asm_map_rtac_block(struct rtac_cal_block_data *cal_block)
{
	int result = 0;
	struct asm_buffer_node *buf_node = NULL;
	struct list_head *ptr, *next;

	pr_debug("%s:\n", __func__);

	if (cal_block == NULL) {
		pr_err("%s: cal_block is NULL!\n",
			__func__);
		result = -EINVAL;
		goto done;
	}

	if (cal_block->cal_data.paddr == 0) {
		pr_debug("%s: No address to map!\n",
			__func__);
		result = -EINVAL;
		goto done;
	}

	if (common_client.mmap_apr == NULL) {
		common_client.mmap_apr = q6asm_mmap_apr_reg();
		if (common_client.mmap_apr == NULL) {
			pr_err("%s: q6asm_mmap_apr_reg failed\n",
				__func__);
			result = -EPERM;
			goto done;
		}
	}

	if (cal_block->map_data.map_size == 0) {
		pr_debug("%s: map size is 0!\n",
			__func__);
		result = -EINVAL;
		goto done;
	}

	/* Use second asm buf to map memory */
	if (common_client.port[OUT].buf == NULL) {
		pr_err("%s: common buf is NULL\n",
			__func__);
		result = -EINVAL;
		goto done;
	}

	common_client.port[OUT].buf->phys = cal_block->cal_data.paddr;

	result = q6asm_memory_map_regions(&common_client,
			OUT, cal_block->map_data.map_size, 1, 1);
	if (result < 0) {
		pr_err("%s: mmap did not work! size = %d result %d\n",
			__func__,
			cal_block->map_data.map_size, result);
		pr_debug("%s: mmap did not work! addr = 0x%pK, size = %d\n",
			__func__,
			&cal_block->cal_data.paddr,
			cal_block->map_data.map_size);
		goto done;
	}

	list_for_each_safe(ptr, next,
		&common_client.port[OUT].mem_map_handle) {
		buf_node = list_entry(ptr, struct asm_buffer_node,
					list);
		if (buf_node->buf_phys_addr == cal_block->cal_data.paddr) {
			cal_block->map_data.map_handle =  buf_node->mmap_hdl;
			break;
		}
	}
done:
	return result;
}

int q6asm_unmap_rtac_block(uint32_t *mem_map_handle)
{
	int result = 0;
	int result2 = 0;

	pr_debug("%s:\n", __func__);

	if (mem_map_handle == NULL) {
		pr_debug("%s: Map handle is NULL, nothing to unmap\n",
			__func__);
		goto done;
	}

	if (*mem_map_handle == 0) {
		pr_debug("%s: Map handle is 0, nothing to unmap\n",
			__func__);
		goto done;
	}

	if (common_client.mmap_apr == NULL) {
		common_client.mmap_apr = q6asm_mmap_apr_reg();
		if (common_client.mmap_apr == NULL) {
			pr_err("%s: q6asm_mmap_apr_reg failed\n",
				__func__);
			result = -EPERM;
			goto done;
		}
	}


	result2 = q6asm_memory_unmap_regions(&common_client, OUT);
	if (result2 < 0) {
		pr_err("%s: unmap failed, err %d\n",
			__func__, result2);
		result = result2;
	} else {
		*mem_map_handle = 0;
	}

	result2 = q6asm_mmap_apr_dereg();
	if (result2 < 0) {
		pr_err("%s: q6asm_mmap_apr_dereg failed, err %d\n",
			__func__, result2);
		result = result2;
	}
done:
	return result;
}

int q6asm_audio_client_buf_free(unsigned int dir,
			struct audio_client *ac)
{
	struct audio_port_data *port;
	int cnt = 0;
	int rc = 0;

	pr_debug("%s: Session id %d\n", __func__, ac->session);
	mutex_lock(&ac->cmd_lock);
	if (ac->io_mode & SYNC_IO_MODE) {
		port = &ac->port[dir];
		if (!port->buf) {
			pr_err("%s: buf NULL\n", __func__);
			mutex_unlock(&ac->cmd_lock);
			return 0;
		}
		cnt = port->max_buf_cnt - 1;

		if (cnt >= 0) {
			rc = q6asm_memory_unmap_regions(ac, dir);
			if (rc < 0)
				pr_err("%s: Memory_unmap_regions failed %d\n",
								__func__, rc);
		}

		while (cnt >= 0) {
			if (port->buf[cnt].data) {
				if (!rc || atomic_read(&ac->reset))
					msm_audio_ion_free(
						port->buf[cnt].dma_buf);

				port->buf[cnt].dma_buf = NULL;
				port->buf[cnt].data = NULL;
				port->buf[cnt].phys = 0;
				--(port->max_buf_cnt);
			}
			--cnt;
		}
		kfree(port->buf);
		port->buf = NULL;
	}
	mutex_unlock(&ac->cmd_lock);
	return 0;
}

/**
 * q6asm_audio_client_buf_free_contiguous -
 *       frees the memory buffers for ASM
 *
 * @dir: RX or TX direction
 * @ac: audio client handle
 *
 * Returns 0 on success or error on failure
 */
int q6asm_audio_client_buf_free_contiguous(unsigned int dir,
			struct audio_client *ac)
{
	struct audio_port_data *port;
	int cnt = 0;
	int rc = 0;

	pr_debug("%s: Session id %d\n", __func__, ac->session);
	mutex_lock(&ac->cmd_lock);
	port = &ac->port[dir];
	if (!port->buf) {
		mutex_unlock(&ac->cmd_lock);
		return 0;
	}
	cnt = port->max_buf_cnt - 1;

	if (cnt >= 0) {
		rc = q6asm_memory_unmap(ac, port->buf[0].phys, dir);
		if (rc < 0)
			pr_err("%s: Memory_unmap_regions failed %d\n",
							__func__, rc);
	}

	if (port->buf[0].data) {
		pr_debug("%s: data[%pK], phys[%pK], dma_buf[%pK]\n",
			__func__,
			port->buf[0].data,
			&port->buf[0].phys,
			port->buf[0].dma_buf);
		if (!rc || atomic_read(&ac->reset))
			msm_audio_ion_free(port->buf[0].dma_buf);
		port->buf[0].dma_buf = NULL;
	}

	while (cnt >= 0) {
		port->buf[cnt].data = NULL;
		port->buf[cnt].phys = 0;
		cnt--;
	}
	port->max_buf_cnt = 0;
	kfree(port->buf);
	port->buf = NULL;
	mutex_unlock(&ac->cmd_lock);
	return 0;
}
EXPORT_SYMBOL(q6asm_audio_client_buf_free_contiguous);

/**
 * q6asm_audio_client_free -
 *       frees the audio client for ASM
 *
 * @ac: audio client handle
 *
 */
void q6asm_audio_client_free(struct audio_client *ac)
{
	int loopcnt;
	struct audio_port_data *port;

	if (!ac) {
		pr_err("%s: ac %pK\n", __func__, ac);
		return;
	}
	if (!ac->session) {
		pr_err("%s: ac session invalid\n", __func__);
		return;
	}

	mutex_lock(&session_lock);

	pr_debug("%s: Session id %d\n", __func__, ac->session);
	if (ac->io_mode & SYNC_IO_MODE) {
		for (loopcnt = 0; loopcnt <= OUT; loopcnt++) {
			port = &ac->port[loopcnt];
			if (!port->buf)
				continue;
			pr_debug("%s: loopcnt = %d\n",
				__func__, loopcnt);
			q6asm_audio_client_buf_free(loopcnt, ac);
		}
	}

	q6asm_session_deregister(ac);
	q6asm_session_free(ac);

	pr_debug("%s: APR De-Register\n", __func__);

/*done:*/
	mutex_unlock(&session_lock);
}
EXPORT_SYMBOL(q6asm_audio_client_free);

/**
 * q6asm_set_io_mode -
 *       Update IO mode for ASM
 *
 * @ac: audio client handle
 * @mode1: IO mode to update
 *
 * Returns 0 on success or error on failure
 */
int q6asm_set_io_mode(struct audio_client *ac, uint32_t mode1)
{
	uint32_t mode;
	int ret = 0;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}

	ac->io_mode &= 0xFF00;
	mode = (mode1 & 0xF);

	pr_debug("%s: ac->mode after anding with FF00:0x%x,\n",
		__func__, ac->io_mode);

	if ((mode == ASYNC_IO_MODE) || (mode == SYNC_IO_MODE)) {
		ac->io_mode |= mode1;
		pr_debug("%s: Set Mode to 0x%x\n", __func__, ac->io_mode);
	} else {
		pr_err("%s: Not an valid IO Mode:%d\n", __func__, ac->io_mode);
		ret = -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL(q6asm_set_io_mode);

void *q6asm_mmap_apr_reg(void)
{
	if ((atomic_read(&this_mmap.ref_cnt) == 0) ||
	    (this_mmap.apr == NULL)) {
		this_mmap.apr = apr_register("ADSP", "ASM",
					(apr_fn)q6asm_srvc_callback,
					0x0FFFFFFFF, &this_mmap);
		if (this_mmap.apr == NULL) {
			pr_debug("%s: Unable to register APR ASM common port\n",
			 __func__);
			goto fail;
		}
	}
	atomic_inc(&this_mmap.ref_cnt);

	return this_mmap.apr;
fail:
	return NULL;
}

/**
 * q6asm_send_stream_cmd -
 *       command to send for ASM stream
 *
 * @ac: audio client handle
 * @data: event data
 *
 * Returns 0 on success or error on failure
 */
int q6asm_send_stream_cmd(struct audio_client *ac,
			  struct msm_adsp_event_data *data)
{
	char *asm_params = NULL;
	struct apr_hdr hdr;
	int rc;
	uint32_t sz = 0;
	uint64_t actual_sz = 0;
	int session_id = 0;

	if (!data || !ac) {
		pr_err("%s: %s is NULL\n", __func__,
			(!data) ? "data" : "ac");
		rc = -EINVAL;
		goto done;
	}

	session_id = q6asm_get_session_id_from_audio_client(ac);
	if (!session_id) {
		rc = -EINVAL;
		goto done;
	}

	if (data->event_type >= ARRAY_SIZE(adsp_reg_event_opcode)) {
		pr_err("%s: event %u out of boundary of array size of (%lu)\n",
		       __func__, data->event_type,
		       (long)ARRAY_SIZE(adsp_reg_event_opcode));
		rc = -EINVAL;
		goto done;
	}

	actual_sz = sizeof(struct apr_hdr) + data->payload_len;
	if (actual_sz > U32_MAX) {
		pr_err("%s: payload size 0x%X exceeds limit\n",
		       __func__, data->payload_len);
		rc = -EINVAL;
		goto done;
	}

	sz = (uint32_t)actual_sz;
	asm_params = kzalloc(sz, GFP_KERNEL);
	if (!asm_params) {
		rc = -ENOMEM;
		goto done;
	}

	mutex_lock(&session[session_id].mutex_lock_per_session);
	if (!q6asm_is_valid_audio_client(ac)) {
		rc = -EINVAL;
		goto fail_send_param;
	}

	q6asm_add_hdr_async(ac, &hdr, sz, TRUE);
	atomic_set(&ac->cmd_state_pp, -1);
	hdr.opcode = adsp_reg_event_opcode[data->event_type];
	memcpy(asm_params, &hdr, sizeof(struct apr_hdr));
	memcpy(asm_params + sizeof(struct apr_hdr),
		data->payload, data->payload_len);
	rc = apr_send_pkt(ac->apr, (uint32_t *) asm_params);
	if (rc < 0) {
		pr_err("%s: stream event cmd apr pkt failed\n", __func__);
		rc = -EINVAL;
		goto fail_send_param;
	}

	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state_pp) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout for stream event cmd resp\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_send_param;
	}

	if (atomic_read(&ac->cmd_state_pp) > 0) {
		pr_err("%s: DSP returned error[%s] for stream event cmd\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state_pp)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state_pp));
		goto fail_send_param;
	}

	rc = 0;
fail_send_param:
	mutex_unlock(&session[session_id].mutex_lock_per_session);
	kfree(asm_params);
done:
	return rc;
}
EXPORT_SYMBOL(q6asm_send_stream_cmd);

/**
 * q6asm_audio_client_alloc -
 *       Alloc audio client for ASM
 *
 * @cb: callback fn
 * @priv: private data
 *
 * Returns ac pointer on success or NULL on failure
 */
struct audio_client *q6asm_audio_client_alloc(app_cb cb, void *priv)
{
	struct audio_client *ac;
	int n;
	int lcnt = 0;
	int rc = 0;

	ac = kzalloc(sizeof(struct audio_client), GFP_KERNEL);
	if (!ac)
		return NULL;

	mutex_lock(&session_lock);
	n = q6asm_session_alloc(ac);
	if (n <= 0) {
		pr_err("%s: ASM Session alloc fail n=%d\n", __func__, n);
		mutex_unlock(&session_lock);
		kfree(ac);
		goto fail_session;
	}
	ac->session = n;
	ac->cb = cb;
	ac->path_delay = UINT_MAX;
	ac->priv = priv;
	ac->io_mode = SYNC_IO_MODE;
	ac->perf_mode = LEGACY_PCM_MODE;
	ac->fptr_cache_ops = NULL;
	/* DSP expects stream id from 1 */
	ac->stream_id = 1;

	rc = q6asm_session_register(ac);
	if (rc < 0) {
		mutex_unlock(&session_lock);
		goto fail_register;
	}

	init_waitqueue_head(&ac->cmd_wait);
	init_waitqueue_head(&ac->time_wait);
	init_waitqueue_head(&ac->mem_wait);
	atomic_set(&ac->time_flag, 1);
	atomic_set(&ac->reset, 0);
	INIT_LIST_HEAD(&ac->port[0].mem_map_handle);
	INIT_LIST_HEAD(&ac->port[1].mem_map_handle);
	pr_debug("%s: mem_map_handle list init'ed\n", __func__);
	mutex_init(&ac->cmd_lock);
	for (lcnt = 0; lcnt <= OUT; lcnt++) {
		mutex_init(&ac->port[lcnt].lock);
		spin_lock_init(&ac->port[lcnt].dsp_lock);
	}
	atomic_set(&ac->cmd_state, 0);
	atomic_set(&ac->cmd_state_pp, 0);
	atomic_set(&ac->mem_state, 0);

	rc = send_asm_custom_topology(ac);
	if (rc < 0) {
		mutex_unlock(&session_lock);
		goto fail_send;
	}

	pr_debug("%s: session[%d]\n", __func__, ac->session);

	mutex_unlock(&session_lock);

	return ac;
fail_send:
	q6asm_session_deregister(ac);
fail_register:
	q6asm_session_free(ac);
fail_session:
	return NULL;
}
EXPORT_SYMBOL(q6asm_audio_client_alloc);

/**
 * q6asm_get_audio_client -
 *       Retrieve audio client for ASM
 *
 * @session_id: ASM session id
 *
 * Returns valid pointer on success or NULL on failure
 */
struct audio_client *q6asm_get_audio_client(int session_id)
{
	if (session_id == ASM_CONTROL_SESSION)
		return &common_client;

	if ((session_id <= 0) || (session_id > ASM_ACTIVE_STREAMS_ALLOWED)) {
		pr_err("%s: invalid session: %d\n", __func__, session_id);
		goto err;
	}

	if (!(session[session_id].ac)) {
		pr_err("%s: session not active: %d\n", __func__, session_id);
		goto err;
	}
	return session[session_id].ac;
err:
	return NULL;
}
EXPORT_SYMBOL(q6asm_get_audio_client);

/**
 * q6asm_audio_client_buf_alloc -
 *       Allocs memory from ION for ASM
 *
 * @dir: RX or TX direction
 * @ac: Audio client handle
 * @bufsz: size of each buffer
 * @bufcnt: number of buffers to alloc
 *
 * Returns 0 on success or error on failure
 */
int q6asm_audio_client_buf_alloc(unsigned int dir,
			struct audio_client *ac,
			unsigned int bufsz,
			uint32_t bufcnt)
{
	int cnt = 0;
	int rc = 0;
	struct audio_buffer *buf;
	size_t len;

	if (!(ac) || !(bufsz) || ((dir != IN) && (dir != OUT))) {
		pr_err("%s: ac %pK bufsz %d dir %d\n", __func__, ac, bufsz,
			dir);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]bufsz[%d]bufcnt[%d]\n", __func__, ac->session,
		bufsz, bufcnt);

	if (ac->session <= 0 || ac->session > 8) {
		pr_err("%s: Session ID is invalid, session = %d\n", __func__,
			ac->session);
		goto fail;
	}

	if (ac->io_mode & SYNC_IO_MODE) {
		if (ac->port[dir].buf) {
			pr_debug("%s: buffer already allocated\n", __func__);
			return 0;
		}
		mutex_lock(&ac->cmd_lock);
		if (bufcnt > (U32_MAX/sizeof(struct audio_buffer))) {
			pr_err("%s: Buffer size overflows", __func__);
			mutex_unlock(&ac->cmd_lock);
			goto fail;
		}
		buf = kzalloc(((sizeof(struct audio_buffer))*bufcnt),
				GFP_KERNEL);

		if (!buf) {
			mutex_unlock(&ac->cmd_lock);
			goto fail;
		}

		ac->port[dir].buf = buf;

		while (cnt < bufcnt) {
			if (bufsz > 0) {
				if (!buf[cnt].data) {
					rc = msm_audio_ion_alloc(
					      &buf[cnt].dma_buf,
					      bufsz,
					      &buf[cnt].phys,
					      &len,
					      &buf[cnt].data);
					if (rc) {
						pr_err("%s: ION Get Physical for AUDIO failed, rc = %d\n",
							__func__, rc);
						mutex_unlock(&ac->cmd_lock);
					goto fail;
					}

					buf[cnt].used = 1;
					buf[cnt].size = bufsz;
					buf[cnt].actual_size = bufsz;
					pr_debug("%s: data[%pK]phys[%pK][%pK]\n",
						__func__,
					   buf[cnt].data,
					   &buf[cnt].phys,
					   &buf[cnt].phys);
					cnt++;
				}
			}
		}
		ac->port[dir].max_buf_cnt = cnt;

		mutex_unlock(&ac->cmd_lock);
		rc = q6asm_memory_map_regions(ac, dir, bufsz, cnt, 0);
		if (rc < 0) {
			pr_err("%s: CMD Memory_map_regions failed %d for size %d\n",
				__func__, rc, bufsz);
			goto fail;
		}
	}
	return 0;
fail:
	q6asm_audio_client_buf_free(dir, ac);
	return -EINVAL;
}
EXPORT_SYMBOL(q6asm_audio_client_buf_alloc);

/**
 * q6asm_audio_client_buf_alloc_contiguous -
 *       Alloc contiguous memory from ION for ASM
 *
 * @dir: RX or TX direction
 * @ac: Audio client handle
 * @bufsz: size of each buffer
 * @bufcnt: number of buffers to alloc
 *
 * Returns 0 on success or error on failure
 */
int q6asm_audio_client_buf_alloc_contiguous(unsigned int dir,
			struct audio_client *ac,
			unsigned int bufsz,
			unsigned int bufcnt)
{
	int cnt = 0;
	int rc = 0;
	struct audio_buffer *buf;
	size_t len;
	int bytes_to_alloc;

	if (!(ac) || ((dir != IN) && (dir != OUT))) {
		pr_err("%s: ac %pK dir %d\n", __func__, ac, dir);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]bufsz[%d]bufcnt[%d]\n",
			__func__, ac->session,
			bufsz, bufcnt);

	if (ac->session <= 0 || ac->session > ASM_ACTIVE_STREAMS_ALLOWED) {
		pr_err("%s: Session ID is invalid, session = %d\n", __func__,
			ac->session);
		goto fail;
	}

	if (ac->port[dir].buf) {
		pr_err("%s: buffer already allocated\n", __func__);
		return 0;
	}
	mutex_lock(&ac->cmd_lock);
	buf = kzalloc(((sizeof(struct audio_buffer))*bufcnt),
			GFP_KERNEL);

	if (!buf) {
		pr_err("%s: buffer allocation failed\n", __func__);
		mutex_unlock(&ac->cmd_lock);
		goto fail;
	}

	ac->port[dir].buf = buf;

	/* check for integer overflow */
	if ((bufcnt > 0) && ((INT_MAX / bufcnt) < bufsz)) {
		pr_err("%s: integer overflow\n", __func__);
		mutex_unlock(&ac->cmd_lock);
		goto fail;
	}
	bytes_to_alloc = bufsz * bufcnt;

	/* The size to allocate should be multiple of 4K bytes */
	bytes_to_alloc = PAGE_ALIGN(bytes_to_alloc);

	rc = msm_audio_ion_alloc(&buf[0].dma_buf,
		bytes_to_alloc,
		&buf[0].phys, &len,
		&buf[0].data);
	if (rc) {
		pr_err("%s: Audio ION alloc is failed, rc = %d\n",
			__func__, rc);
		mutex_unlock(&ac->cmd_lock);
		goto fail;
	}

	buf[0].used = dir ^ 1;
	buf[0].size = bufsz;
	buf[0].actual_size = bufsz;
	cnt = 1;
	while (cnt < bufcnt) {
		if (bufsz > 0) {
			buf[cnt].data =  buf[0].data + (cnt * bufsz);
			buf[cnt].phys =  buf[0].phys + (cnt * bufsz);
			if (!buf[cnt].data) {
				pr_err("%s: Buf alloc failed\n",
							__func__);
				mutex_unlock(&ac->cmd_lock);
				goto fail;
			}
			buf[cnt].used = dir ^ 1;
			buf[cnt].size = bufsz;
			buf[cnt].actual_size = bufsz;
			pr_debug("%s: data[%pK]phys[%pK][%pK]\n",
				__func__,
				buf[cnt].data,
				&buf[cnt].phys,
				&buf[cnt].phys);
		}
		cnt++;
	}
	ac->port[dir].max_buf_cnt = cnt;
	mutex_unlock(&ac->cmd_lock);
	rc = q6asm_memory_map_regions(ac, dir, bufsz, cnt, 1);
	if (rc < 0) {
		pr_err("%s: CMD Memory_map_regions failed %d for size %d\n",
			__func__, rc, bufsz);
		goto fail;
	}
	return 0;
fail:
	q6asm_audio_client_buf_free_contiguous(dir, ac);
	return -EINVAL;
}
EXPORT_SYMBOL(q6asm_audio_client_buf_alloc_contiguous);

static int32_t q6asm_srvc_callback(struct apr_client_data *data, void *priv)
{
	uint32_t dir = 0;
	uint32_t i = IN;
	uint32_t *payload;
	unsigned long dsp_flags = 0;
	unsigned long flags = 0;
	struct asm_buffer_node *buf_node = NULL;
	struct list_head *ptr, *next;
	union asm_token_struct asm_token;

	struct audio_client *ac = NULL;
	struct audio_port_data *port;

	int session_id;

	if (!data) {
		pr_err("%s: Invalid CB\n", __func__);
		return 0;
	}

	payload = data->payload;

	if (data->opcode == RESET_EVENTS) {
		pr_debug("%s: Reset event is received: %d %d apr[%pK]\n",
				__func__,
				data->reset_event,
				data->reset_proc,
				this_mmap.apr);
		atomic_set(&this_mmap.ref_cnt, 0);
		apr_reset(this_mmap.apr);
		this_mmap.apr = NULL;
		for (; i <= OUT; i++) {
			list_for_each_safe(ptr, next,
				&common_client.port[i].mem_map_handle) {
				buf_node = list_entry(ptr,
						struct asm_buffer_node,
						list);
				list_del(&buf_node->list);
				kfree(buf_node);
			}
			pr_debug("%s: Clearing custom topology\n", __func__);
		}

		cal_utils_clear_cal_block_q6maps(ASM_MAX_CAL_TYPES, cal_data);
		common_client.mmap_apr = NULL;
		mutex_lock(&cal_data[ASM_CUSTOM_TOP_CAL]->lock);
		set_custom_topology = 1;
		mutex_unlock(&cal_data[ASM_CUSTOM_TOP_CAL]->lock);
		topology_map_handle = 0;
		rtac_clear_mapping(ASM_RTAC_CAL);
		return 0;
	}
	asm_token.token = data->token;
	session_id = asm_token._token.session_id;

	if ((session_id > 0 && session_id <= ASM_ACTIVE_STREAMS_ALLOWED))
		spin_lock_irqsave(&(session[session_id].session_lock), flags);

	ac = q6asm_get_audio_client(session_id);
	dir = q6asm_get_flag_from_token(&asm_token, ASM_DIRECTION_OFFSET);

	if (!ac) {
		pr_debug("%s: session[%d] already freed\n",
			 __func__, session_id);
		if ((session_id > 0 &&
			session_id <= ASM_ACTIVE_STREAMS_ALLOWED))
			spin_unlock_irqrestore(
				&(session[session_id].session_lock), flags);
		return 0;
	}

	if (data->payload_size >= 2 * sizeof(uint32_t)) {
		pr_debug("%s:ptr0[0x%x]ptr1[0x%x]opcode[0x%x] token[0x%x]payload_s[%d] src[%d] dest[%d]sid[%d]dir[%d]\n",
			__func__, payload[0], payload[1], data->opcode,
			data->token, data->payload_size, data->src_port,
			data->dest_port, asm_token._token.session_id, dir);
		pr_debug("%s:Payload = [0x%x] status[0x%x]\n",
			__func__, payload[0], payload[1]);
	} else if (data->payload_size == sizeof(uint32_t)) {
		pr_debug("%s:ptr0[0x%x]opcode[0x%x] token[0x%x]payload_s[%d] src[%d] dest[%d]sid[%d]dir[%d]\n",
			__func__, payload[0], data->opcode,
			data->token, data->payload_size, data->src_port,
			data->dest_port, asm_token._token.session_id, dir);
		pr_debug("%s:Payload = [0x%x]\n",
			__func__, payload[0]);
	}

	if (data->opcode == APR_BASIC_RSP_RESULT) {
		switch (payload[0]) {
		case ASM_CMD_SHARED_MEM_MAP_REGIONS:
		case ASM_CMD_SHARED_MEM_UNMAP_REGIONS:
		case ASM_CMD_ADD_TOPOLOGIES:
			if (data->payload_size >= 2 * sizeof(uint32_t) && payload[1] != 0) {
				pr_err("%s: cmd = 0x%x returned error = 0x%x sid:%d\n",
				       __func__, payload[0], payload[1],
				       asm_token._token.session_id);
				if (payload[0] ==
				    ASM_CMD_SHARED_MEM_UNMAP_REGIONS)
					atomic_set(&ac->unmap_cb_success, 0);

				atomic_set(&ac->mem_state, payload[1]);
				wake_up(&ac->mem_wait);
			} else {
				if (payload[0] ==
				    ASM_CMD_SHARED_MEM_UNMAP_REGIONS)
					atomic_set(&ac->unmap_cb_success, 1);
			}

			if (atomic_cmpxchg(&ac->mem_state, -1, 0) == -1)
				wake_up(&ac->mem_wait);
			if (data->payload_size >= 2 * sizeof(uint32_t))
				dev_vdbg(ac->dev, "%s: Payload = [0x%x] status[0x%x]\n",
					__func__, payload[0], payload[1]);
			else
				dev_vdbg(ac->dev, "%s: Payload size of %d is less than expected.\n",
					__func__, data->payload_size);
			break;
		default:
			pr_debug("%s: command[0x%x] not expecting rsp\n",
						__func__, payload[0]);
			break;
		}
		if ((session_id > 0 &&
			session_id <= ASM_ACTIVE_STREAMS_ALLOWED))
			spin_unlock_irqrestore(
				&(session[session_id].session_lock), flags);
		return 0;
	}

	port = &ac->port[dir];

	switch (data->opcode) {
	case ASM_CMDRSP_SHARED_MEM_MAP_REGIONS:{
		pr_debug("%s:PL#0[0x%x] dir=0x%x s_id=0x%x\n",
		       __func__, payload[0], dir, asm_token._token.session_id);
		spin_lock_irqsave(&port->dsp_lock, dsp_flags);
		if (atomic_cmpxchg(&ac->mem_state, -1, 0) == -1) {
			ac->port[dir].tmp_hdl = payload[0];
			wake_up(&ac->mem_wait);
		}
		spin_unlock_irqrestore(&port->dsp_lock, dsp_flags);
		break;
	}
	case ASM_CMD_SHARED_MEM_UNMAP_REGIONS:{
		if (data->payload_size >= 2 * sizeof(uint32_t))
			pr_debug("%s: PL#0[0x%x]PL#1 [0x%x]\n",
						__func__, payload[0], payload[1]);
		else
			pr_debug("%s: Payload size of %d is less than expected.\n",
				__func__, data->payload_size);

		spin_lock_irqsave(&port->dsp_lock, dsp_flags);
		if (atomic_cmpxchg(&ac->mem_state, -1, 0) == -1)
			wake_up(&ac->mem_wait);
		spin_unlock_irqrestore(&port->dsp_lock, dsp_flags);

		break;
	}
	default:
		if (data->payload_size >= 2 * sizeof(uint32_t))
			pr_debug("%s: command[0x%x]success [0x%x]\n",
				__func__, payload[0], payload[1]);
		else
			pr_debug("%s: Payload size of %d is less than expected.\n",
				__func__, data->payload_size);
	}
	if (ac->cb)
		ac->cb(data->opcode, data->token,
			data->payload, ac->priv);
	if ((session_id > 0 && session_id <= ASM_ACTIVE_STREAMS_ALLOWED))
		spin_unlock_irqrestore(
			&(session[session_id].session_lock), flags);

	return 0;
}

static void q6asm_process_mtmx_get_param_rsp(struct audio_client *ac,
				struct asm_mtmx_strtr_get_params_cmdrsp *cmdrsp)
{
	struct asm_session_mtmx_strtr_param_session_time_v3_t *time;

	if (cmdrsp->err_code) {
		dev_err_ratelimited(ac->dev,
				    "%s: err=%x, mod_id=%x, param_id=%x\n",
				    __func__, cmdrsp->err_code,
				    cmdrsp->param_info.module_id,
				    cmdrsp->param_info.param_id);
		return;
	}
	dev_dbg_ratelimited(ac->dev,
			    "%s: mod_id=%x, param_id=%x\n", __func__,
			    cmdrsp->param_info.module_id,
			    cmdrsp->param_info.param_id);

	switch (cmdrsp->param_info.module_id) {
	case ASM_SESSION_MTMX_STRTR_MODULE_ID_AVSYNC:
		switch (cmdrsp->param_info.param_id) {
		case ASM_SESSION_MTMX_STRTR_PARAM_SESSION_TIME_V3:
			time = &cmdrsp->param_data.session_time;
			dev_vdbg(ac->dev, "%s: GET_TIME_V3, time_lsw=%x, time_msw=%x, abs l %x, m %x\n",
				 __func__, time->session_time_lsw,
				 time->session_time_msw,
				 time->absolute_time_lsw,
				 time->absolute_time_msw);
			ac->dsp_ts.abs_time_stamp = (uint64_t)(((uint64_t)
					time->absolute_time_msw << 32) |
					time->absolute_time_lsw);
			ac->dsp_ts.time_stamp = (uint64_t)(((uint64_t)
					 time->session_time_msw << 32) |
					 time->session_time_lsw);
			ac->dsp_ts.last_time_stamp = (uint64_t)(((uint64_t)
					 time->time_stamp_msw << 32) |
					 time->time_stamp_lsw);
			if (time->flags &
			    ASM_SESSION_MTMX_STRTR_PARAM_STIME_TSTMP_FLG_BMASK)
				dev_warn_ratelimited(ac->dev,
						     "%s: recv inval tstmp\n",
						     __func__);
			if (atomic_cmpxchg(&ac->time_flag, 1, 0))
				wake_up(&ac->time_wait);

			break;
		default:
			dev_err(ac->dev, "%s: unexpected param_id %x\n",
				__func__, cmdrsp->param_info.param_id);
			break;
		}
		break;
	default:
		dev_err(ac->dev, "%s: unexpected mod_id %x\n",  __func__,
			cmdrsp->param_info.module_id);
		break;
	}
}

static int32_t q6asm_callback(struct apr_client_data *data, void *priv)
{
	int i = 0;
	struct audio_client *ac = (struct audio_client *)priv;
	unsigned long dsp_flags = 0;
	uint32_t *payload;
	uint32_t wakeup_flag = 1;
	int32_t  ret = 0;
	union asm_token_struct asm_token;
	uint8_t buf_index;
	struct msm_adsp_event_data *pp_event_package = NULL;
	uint32_t payload_size = 0;
	unsigned long flags = 0;
	int session_id;

	if (ac == NULL) {
		pr_err("%s: ac NULL\n", __func__);
		return -EINVAL;
	}
	if (data == NULL) {
		pr_err("%s: data NULL\n", __func__);
		return -EINVAL;
	}

	session_id = q6asm_get_session_id_from_audio_client(ac);
	if (session_id <= 0 || session_id > ASM_ACTIVE_STREAMS_ALLOWED) {
		pr_err("%s: Session ID is invalid, session = %d\n", __func__,
			session_id);
		return -EINVAL;
	}
	spin_lock_irqsave(&(session[session_id].session_lock), flags);

	if (!q6asm_is_valid_audio_client(ac)) {
		pr_err("%s: audio client pointer is invalid, ac = %pK\n",
				__func__, ac);
		spin_unlock_irqrestore(
			&(session[session_id].session_lock), flags);
		return -EINVAL;
	}

	payload = data->payload;
	asm_token.token = data->token;
	if (q6asm_get_flag_from_token(&asm_token, ASM_CMD_NO_WAIT_OFFSET)) {
		pr_debug("%s: No wait command opcode[0x%x] cmd_opcode:%x\n",
			 __func__, data->opcode, payload ? payload[0] : 0);
		wakeup_flag = 0;
	}

	if (data->opcode == RESET_EVENTS) {
		atomic_set(&ac->reset, 1);
		if (ac->apr == NULL) {
			ac->apr = ac->apr2;
			ac->apr2 = NULL;
		}
		pr_debug("%s: Reset event is received: %d %d apr[%pK]\n",
			__func__,
			data->reset_event, data->reset_proc, ac->apr);
		if (ac->cb)
			ac->cb(data->opcode, data->token,
				(uint32_t *)data->payload, ac->priv);
		apr_reset(ac->apr);
		ac->apr = NULL;
		atomic_set(&ac->time_flag, 0);
		atomic_set(&ac->cmd_state, 0);
		atomic_set(&ac->mem_state, 0);
		atomic_set(&ac->cmd_state_pp, 0);
		wake_up(&ac->time_wait);
		wake_up(&ac->cmd_wait);
		wake_up(&ac->mem_wait);
		spin_unlock_irqrestore(
			&(session[session_id].session_lock), flags);
		return 0;
	}

	dev_vdbg(ac->dev, "%s: session[%d]opcode[0x%x] token[0x%x]payload_size[%d] src[%d] dest[%d]\n",
		 __func__,
		ac->session, data->opcode,
		data->token, data->payload_size, data->src_port,
		data->dest_port);
	if ((data->opcode != ASM_DATA_EVENT_RENDERED_EOS) &&
	    (data->opcode != ASM_DATA_EVENT_RENDERED_EOS_V2) &&
	    (data->opcode != ASM_DATA_EVENT_EOS) &&
	    (data->opcode != ASM_SESSION_EVENTX_OVERFLOW) &&
	    (data->opcode != ASM_SESSION_EVENT_RX_UNDERFLOW)) {
		if (payload == NULL) {
			pr_err("%s: payload is null\n", __func__);
			spin_unlock_irqrestore(
				&(session[session_id].session_lock), flags);
			return -EINVAL;
		}
		if(data->payload_size >= 2 * sizeof(uint32_t))
			dev_vdbg(ac->dev, "%s: Payload = [0x%x] status[0x%x] opcode 0x%x\n",
				__func__, payload[0], payload[1], data->opcode);
		else
			dev_vdbg(ac->dev, "%s: Payload size of %d is less than expected.\n",
				__func__, data->payload_size);
	}
	if (data->opcode == APR_BASIC_RSP_RESULT) {
		switch (payload[0]) {
		case ASM_STREAM_CMD_SET_PP_PARAMS_V2:
		case ASM_STREAM_CMD_SET_PP_PARAMS_V3:
			//cmd_state_pp : wait=-1 , non wait=0
			if (atomic_read(&ac->cmd_state_pp) != -1) {
				if (rtac_make_asm_callback(ac->session, payload,
					data->payload_size))
					break;
			}
		case ASM_SESSION_CMD_PAUSE:
		case ASM_SESSION_CMD_SUSPEND:
		case ASM_DATA_CMD_EOS:
		case ASM_DATA_CMD_EOS_V2:
		case ASM_STREAM_CMD_CLOSE:
		case ASM_STREAM_CMD_FLUSH:
		case ASM_SESSION_CMD_RUN_V2:
		case ASM_SESSION_CMD_REGISTER_FORX_OVERFLOW_EVENTS:
		case ASM_STREAM_CMD_FLUSH_READBUFS:
		pr_debug("%s: session %d opcode 0x%x token 0x%x Payload = [0x%x] src %d dest %d\n",
			__func__, ac->session, data->opcode, data->token,
			payload[0], data->src_port, data->dest_port);
		ret = q6asm_is_valid_session(data, priv);
		if (ret != 0) {
			pr_err("%s: session invalid %d\n", __func__, ret);
			spin_unlock_irqrestore(
				&(session[session_id].session_lock), flags);
			return ret;
		}
		case ASM_SESSION_CMD_SET_MTMX_STRTR_PARAMS_V2:
		case ASM_STREAM_CMD_OPEN_READ_V3:
		case ASM_STREAM_CMD_OPEN_WRITE_V3:
		case ASM_STREAM_CMD_OPEN_PULL_MODE_WRITE:
		case ASM_STREAM_CMD_OPEN_PUSH_MODE_READ:
		case ASM_STREAM_CMD_OPEN_READWRITE_V2:
		case ASM_STREAM_CMD_OPEN_LOOPBACK_V2:
		case ASM_STREAM_CMD_OPEN_TRANSCODE_LOOPBACK:
		case ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2:
		case ASM_DATA_CMD_IEC_60958_MEDIA_FMT:
		case ASM_STREAM_CMD_SET_ENCDEC_PARAM:
		case ASM_STREAM_CMD_SET_ENCDEC_PARAM_V2:
		case ASM_STREAM_CMD_REGISTER_ENCDEC_EVENTS:
		case ASM_STREAM_CMD_REGISTER_IEC_61937_FMT_UPDATE:
		case ASM_DATA_CMD_REMOVE_INITIAL_SILENCE:
		case ASM_DATA_CMD_REMOVE_TRAILING_SILENCE:
		case ASM_SESSION_CMD_REGISTER_FOR_RX_UNDERFLOW_EVENTS:
		case ASM_STREAM_CMD_OPEN_READ_COMPRESSED:
		case ASM_STREAM_CMD_OPEN_WRITE_COMPRESSED:
			if (data->payload_size >= 2 * sizeof(uint32_t)) {
				pr_debug("%s: session %d opcode 0x%x token 0x%x Payload = [0x%x] stat 0x%x src %d dest %d\n",
					__func__, ac->session,
					data->opcode, data->token,
					payload[0], payload[1],
					data->src_port, data->dest_port);
				if (payload[1] != 0) {
					if (adsp_err_get_lnx_err_code(payload[1]) != -EALREADY)
						pr_err("%s: cmd = 0x%x returned error = 0x%x\n",
							__func__, payload[0], payload[1]);
					else
						pr_debug("%s: cmd = 0x%x returned error = 0x%x\n",
							__func__, payload[0], payload[1]);
					if (wakeup_flag) {
						if ((is_adsp_reg_event(payload[0]) >=
						     0) ||
						    (payload[0] ==
						     ASM_STREAM_CMD_SET_PP_PARAMS_V2) ||
						    (payload[0] ==
						     ASM_STREAM_CMD_SET_PP_PARAMS_V3))
							atomic_set(&ac->cmd_state_pp,
								payload[1]);
						else
							atomic_set(&ac->cmd_state,
								payload[1]);
						wake_up(&ac->cmd_wait);
					}
					spin_unlock_irqrestore(
						&(session[session_id].session_lock),
						flags);
					return 0;
				}
			} else {
				pr_err("%s: payload size of %x is less than expected.\n",
					__func__, data->payload_size);
			}
			if ((is_adsp_reg_event(payload[0]) >= 0) ||
			    (payload[0] == ASM_STREAM_CMD_SET_PP_PARAMS_V2) ||
			    (payload[0] == ASM_STREAM_CMD_SET_PP_PARAMS_V3)) {
				if (atomic_read(&ac->cmd_state_pp) &&
					wakeup_flag) {
					atomic_set(&ac->cmd_state_pp, 0);
					wake_up(&ac->cmd_wait);
				}
			} else {
				if (atomic_read(&ac->cmd_state) &&
					wakeup_flag) {
					atomic_set(&ac->cmd_state, 0);
					wake_up(&ac->cmd_wait);
				}
			}
			if (ac->cb)
				ac->cb(data->opcode, data->token,
					(uint32_t *)data->payload, ac->priv);
			break;
		case ASM_CMD_ADD_TOPOLOGIES:
			if (data->payload_size >= 2 * sizeof(uint32_t)) {
				pr_debug("%s:Payload = [0x%x]stat[0x%x]\n",
						__func__, payload[0], payload[1]);
				if (payload[1] != 0) {
					pr_err("%s: cmd = 0x%x returned error = 0x%x\n",
						 __func__, payload[0], payload[1]);
					if (wakeup_flag) {
						atomic_set(&ac->mem_state, payload[1]);
						wake_up(&ac->mem_wait);
					}
					spin_unlock_irqrestore(
						&(session[session_id].session_lock),
						flags);
					return 0;
				}
			} else {
				pr_err("%s: payload size of %x is less than expected.\n",
					__func__, data->payload_size);
			}
			if (atomic_read(&ac->mem_state) && wakeup_flag) {
				atomic_set(&ac->mem_state, 0);
				wake_up(&ac->mem_wait);
			}
			if (ac->cb)
				ac->cb(data->opcode, data->token,
					(uint32_t *)data->payload, ac->priv);
			break;
		case ASM_DATA_EVENT_WATERMARK: {
			if (data->payload_size >= 2 * sizeof(uint32_t))
				pr_debug("%s: Watermark opcode[0x%x] status[0x%x]",
					__func__, payload[0], payload[1]);
			else
				pr_err("%s: payload size of %x is less than expected.\n",
					__func__, data->payload_size);
			break;
		}
		case ASM_STREAM_CMD_GET_PP_PARAMS_V2:
		case ASM_STREAM_CMD_GET_PP_PARAMS_V3:
			pr_debug("%s: ASM_STREAM_CMD_GET_PP_PARAMS session %d opcode 0x%x token 0x%x src %d dest %d\n",
				__func__, ac->session, data->opcode,
				data->token, data->src_port, data->dest_port);
			/* Should only come here if there is an APR */
			/* error or malformed APR packet. Otherwise */
			/* response will be returned as */
			/* ASM_STREAM_CMDRSP_GET_PP_PARAMS_V2 */
			if (data->payload_size >= 2 * sizeof(uint32_t)) {
				if (payload[1] != 0) {
					pr_err("%s: ASM get param error = %d, resuming\n",
						__func__, payload[1]);
					rtac_make_asm_callback(ac->session, payload,
								data->payload_size);
				}
			} else {
				pr_err("%s: payload size of %x is less than expected.\n",
					__func__, data->payload_size);
			}
			break;
		case ASM_STREAM_CMD_REGISTER_PP_EVENTS:
			pr_debug("%s: ASM_STREAM_CMD_REGISTER_PP_EVENTS session %d opcode 0x%x token 0x%x src %d dest %d\n",
				__func__, ac->session,
				data->opcode, data->token,
				data->src_port, data->dest_port);
			if (data->payload_size >= 2 * sizeof(uint32_t)) {
				if (payload[1] != 0)
					pr_err("%s: ASM get param error = %d, resuming\n",
						__func__, payload[1]);
				atomic_set(&ac->cmd_state_pp, payload[1]);
				wake_up(&ac->cmd_wait);
			} else {
				pr_err("%s: payload size of %x is less than expected.\n",
					__func__, data->payload_size);
			}
			break;
		default:
			pr_debug("%s: command[0x%x] not expecting rsp\n",
							__func__, payload[0]);
			break;
		}

		spin_unlock_irqrestore(
			&(session[session_id].session_lock), flags);
		return 0;
	}

	switch (data->opcode) {
	case ASM_DATA_EVENT_WRITE_DONE_V2:{
		struct audio_port_data *port = &ac->port[IN];
		if (data->payload_size >= 2 * sizeof(uint32_t))
			dev_vdbg(ac->dev, "%s: Rxed opcode[0x%x] status[0x%x] token[%d]",
					__func__, payload[0], payload[1],
					data->token);
		else
			dev_err(ac->dev, "%s: payload size of %x is less than expected.\n",
				__func__, data->payload_size);
		if (ac->io_mode & SYNC_IO_MODE) {
			if (port->buf == NULL) {
				pr_err("%s: Unexpected Write Done\n",
								__func__);
				spin_unlock_irqrestore(
					&(session[session_id].session_lock),
					flags);
				return -EINVAL;
			}
			spin_lock_irqsave(&port->dsp_lock, dsp_flags);
			buf_index = asm_token._token.buf_index;
			if (buf_index < 0 || buf_index >= port->max_buf_cnt) {
				pr_debug("%s: Invalid buffer index %u\n",
					__func__, buf_index);
				spin_unlock_irqrestore(&port->dsp_lock,
								dsp_flags);
				spin_unlock_irqrestore(
					&(session[session_id].session_lock),
					flags);
				return -EINVAL;
			}
			if ( data->payload_size >= 2 * sizeof(uint32_t) &&
				(lower_32_bits(port->buf[buf_index].phys) !=
				payload[0] || 
				msm_audio_populate_upper_32_bits(
					port->buf[buf_index].phys) != payload[1])) {
				pr_debug("%s: Expected addr %pK\n",
				__func__, &port->buf[buf_index].phys);
				pr_err("%s: rxedl[0x%x] rxedu [0x%x]\n",
					__func__, payload[0], payload[1]);
				spin_unlock_irqrestore(&port->dsp_lock,
								dsp_flags);
				spin_unlock_irqrestore(
					&(session[session_id].session_lock),
					flags);
				return -EINVAL;
			}
			port->buf[buf_index].used = 1;
			spin_unlock_irqrestore(&port->dsp_lock, dsp_flags);

			config_debug_fs_write_cb();

			for (i = 0; i < port->max_buf_cnt; i++)
				dev_vdbg(ac->dev, "%s %d\n",
					__func__, port->buf[i].used);

		}
		break;
	}
	case ASM_STREAM_CMDRSP_GET_PP_PARAMS_V2:
	case ASM_STREAM_CMDRSP_GET_PP_PARAMS_V3:
		pr_debug("%s: ASM_STREAM_CMDRSP_GET_PP_PARAMS session %d opcode 0x%x token 0x%x src %d dest %d\n",
			__func__, ac->session, data->opcode, data->token,
			data->src_port, data->dest_port);
		if (payload[0] != 0) {
			pr_err("%s: ASM_STREAM_CMDRSP_GET_PP_PARAMS returned error = 0x%x\n",
			       __func__, payload[0]);
		} else if (generic_get_data) {
			generic_get_data->valid = 1;
			if (generic_get_data->is_inband) {
				if (data->payload_size >= 4 * sizeof(uint32_t))
					pr_debug("%s: payload[1] = 0x%x, payload[2]=0x%x, payload[3]=0x%x\n",
							__func__, payload[1], payload[2], payload[3]);
				else
					pr_err("%s: payload size of %x is less than expected.\n",
						__func__,
						data->payload_size);

				if (data->payload_size >= (4 + (payload[3]>>2)) * sizeof(uint32_t)) {
					generic_get_data->size_in_ints = payload[3]>>2;
					for (i = 0; i < payload[3]>>2; i++) {
						generic_get_data->ints[i] =
									   payload[4+i];
						pr_debug("%s: ASM callback val %i = %i\n",
							 __func__, i, payload[4+i]);
					}
				} else {
					pr_err("%s: payload size of %x is less than expected.\n",
						__func__, data->payload_size);
				}
				pr_debug("%s: callback size in ints = %i\n",
					 __func__,
					generic_get_data->size_in_ints);
			}
			if (atomic_read(&ac->cmd_state) && wakeup_flag) {
				atomic_set(&ac->cmd_state, 0);
				wake_up(&ac->cmd_wait);
			}
			break;
		}
		rtac_make_asm_callback(ac->session, payload,
			data->payload_size);
		break;
	case ASM_DATA_EVENT_READ_DONE_V2:{

		struct audio_port_data *port = &ac->port[OUT];

		config_debug_fs_read_cb();

		if (data->payload_size != (READDONE_IDX_SEQ_ID + 1) * sizeof(uint32_t)) {
			pr_err("%s:  payload size of %d is less than expected %d.\n",
					__func__, data->payload_size,
					((READDONE_IDX_SEQ_ID + 1) * sizeof(uint32_t)));
			spin_unlock_irqrestore(
				&(session[session_id].session_lock),
				flags);
			return -EINVAL;
		}
		dev_vdbg(ac->dev, "%s: ReadDone: status=%d buff_add=0x%x act_size=%d offset=%d\n",
				__func__, payload[READDONE_IDX_STATUS],
				payload[READDONE_IDX_BUFADD_LSW],
				payload[READDONE_IDX_SIZE],
				payload[READDONE_IDX_OFFSET]);

		dev_vdbg(ac->dev, "%s: ReadDone:msw_ts=%d lsw_ts=%d memmap_hdl=0x%x flags=%d id=%d num=%d\n",
				__func__, payload[READDONE_IDX_MSW_TS],
				payload[READDONE_IDX_LSW_TS],
				payload[READDONE_IDX_MEMMAP_HDL],
				payload[READDONE_IDX_FLAGS],
				payload[READDONE_IDX_SEQ_ID],
				payload[READDONE_IDX_NUMFRAMES]);

		if (ac->io_mode & SYNC_IO_MODE) {
			if (port->buf == NULL) {
				pr_err("%s: Unexpected Read Done\n", __func__);
				spin_unlock_irqrestore(
					&(session[session_id].session_lock),
					flags);
				return -EINVAL;
			}
			spin_lock_irqsave(&port->dsp_lock, dsp_flags);
			buf_index = asm_token._token.buf_index;
			if (buf_index < 0 || buf_index >= port->max_buf_cnt) {
				pr_debug("%s: Invalid buffer index %u\n",
					__func__, buf_index);
				spin_unlock_irqrestore(&port->dsp_lock,
								dsp_flags);
				spin_unlock_irqrestore(
					&(session[session_id].session_lock),
					flags);
				return -EINVAL;
			}
			port->buf[buf_index].used = 0;
			if (lower_32_bits(port->buf[buf_index].phys) !=
			payload[READDONE_IDX_BUFADD_LSW] ||
			msm_audio_populate_upper_32_bits(
				port->buf[buf_index].phys) !=
					payload[READDONE_IDX_BUFADD_MSW]) {
				dev_vdbg(ac->dev, "%s: Expected addr %pK\n",
					__func__, &port->buf[buf_index].phys);
				pr_err("%s: rxedl[0x%x] rxedu[0x%x]\n",
					__func__,
				payload[READDONE_IDX_BUFADD_LSW],
				payload[READDONE_IDX_BUFADD_MSW]);
				spin_unlock_irqrestore(&port->dsp_lock,
							dsp_flags);
				break;
			}
			port->buf[buf_index].actual_size =
				payload[READDONE_IDX_SIZE];
			spin_unlock_irqrestore(&port->dsp_lock, dsp_flags);
		}
		break;
	}
	case ASM_DATA_EVENT_EOS:
	case ASM_DATA_EVENT_RENDERED_EOS:
	case ASM_DATA_EVENT_RENDERED_EOS_V2:
		pr_debug("%s: EOS ACK received: rxed session %d opcode 0x%x token 0x%x src %d dest %d\n",
				__func__, ac->session,
				data->opcode, data->token,
				data->src_port, data->dest_port);
		break;
	case ASM_SESSION_EVENTX_OVERFLOW:
		pr_debug("%s: ASM_SESSION_EVENTX_OVERFLOW session %d opcode 0x%x token 0x%x src %d dest %d\n",
				__func__, ac->session,
				data->opcode, data->token,
				data->src_port, data->dest_port);
		break;
	case ASM_SESSION_EVENT_RX_UNDERFLOW:
		pr_debug("%s: ASM_SESSION_EVENT_RX_UNDERFLOW session %d opcode 0x%x token 0x%x src %d dest %d\n",
				__func__, ac->session,
				data->opcode, data->token,
				data->src_port, data->dest_port);
		break;
	case ASM_SESSION_CMDRSP_GET_SESSIONTIME_V3:
		if (data->payload_size >= 3 * sizeof(uint32_t)) {
			dev_vdbg(ac->dev, "%s: ASM_SESSION_CMDRSP_GET_SESSIONTIME_V3, payload[0] = %d, payload[1] = %d, payload[2] = %d\n",
					 __func__,
					 payload[0], payload[1], payload[2]);
			ac->dsp_ts.time_stamp =
				(uint64_t)(((uint64_t)payload[2] << 32) |
				payload[1]);
		} else {
			dev_err(ac->dev, "%s: payload size of %x is less than expected.n",
				__func__, data->payload_size);
		}
		if (atomic_cmpxchg(&ac->time_flag, 1, 0))
			wake_up(&ac->time_wait);
		break;
	case ASM_DATA_EVENT_SR_CM_CHANGE_NOTIFY:
	case ASM_DATA_EVENT_ENC_SR_CM_CHANGE_NOTIFY:
		pr_debug("%s: ASM_DATA_EVENT_SR_CM_CHANGE_NOTIFY session %d opcode 0x%x token 0x%x src %d dest %d\n",
				__func__, ac->session,
				data->opcode, data->token,
				data->src_port, data->dest_port);
		if (data->payload_size >= 4 * sizeof(uint32_t))
			pr_debug("%s: ASM_DATA_EVENT_SR_CM_CHANGE_NOTIFY, payload[0] = %d, payload[1] = %d, payload[2] = %d, payload[3] = %d\n",
					 __func__,
					payload[0], payload[1], payload[2],
					payload[3]);
		else
			pr_debug("%s: payload size of %x is less than expected.\n",
				__func__, data->payload_size);
		break;
	case ASM_SESSION_CMDRSP_GET_MTMX_STRTR_PARAMS_V2:
		payload_size = sizeof(struct asm_mtmx_strtr_get_params_cmdrsp);
		if (data->payload_size < payload_size) {
			pr_err("%s: insufficient payload size = %d\n",
				__func__, data->payload_size);
			spin_unlock_irqrestore(
				&(session[session_id].session_lock), flags);
			return -EINVAL;
		}
		q6asm_process_mtmx_get_param_rsp(ac,
			(struct asm_mtmx_strtr_get_params_cmdrsp *) payload);
		break;
	case ASM_STREAM_PP_EVENT:
	case ASM_STREAM_CMD_ENCDEC_EVENTS:
	case ASM_IEC_61937_MEDIA_FMT_EVENT:
		if (data->payload_size >= 2 * sizeof(uint32_t))
			pr_debug("%s: ASM_STREAM_EVENT payload[0][0x%x] payload[1][0x%x]",
					 __func__, payload[0], payload[1]);
		else if (data->payload_size >= sizeof(uint32_t))
			pr_debug("%s: ASM_STREAM_EVENT payload[0][0x%x]",
				__func__, payload[0]);
		else
			pr_debug("%s: payload size of %x is less than expected.\n",
				__func__, data->payload_size);
		i = is_adsp_raise_event(data->opcode);
		if (i < 0) {
			spin_unlock_irqrestore(
				&(session[session_id].session_lock), flags);
			return 0;
		}

		/* repack payload for asm_stream_pp_event
		 * package is composed of event type + size + actual payload
		 */
		payload_size = data->payload_size;
		if (payload_size > UINT_MAX - sizeof(struct msm_adsp_event_data)) {
			pr_err("%s: payload size = %d exceeds limit.\n",
				__func__, payload_size);
			spin_unlock_irqrestore(
				&(session[session_id].session_lock), flags);
			return -EINVAL;
		}

		pp_event_package = kzalloc(payload_size
				+ sizeof(struct msm_adsp_event_data),
				GFP_ATOMIC);
		if (!pp_event_package) {
			spin_unlock_irqrestore(
				&(session[session_id].session_lock), flags);
			return -ENOMEM;
		}

		pp_event_package->event_type = i;
		pp_event_package->payload_len = payload_size;
		memcpy((void *)pp_event_package->payload,
			data->payload, payload_size);
		if ((data->opcode == ASM_IEC_61937_MEDIA_FMT_EVENT) &&
		    (payload_size == 4)) {
			switch (payload[0]) {
			case ASM_MEDIA_FMT_AC3:
				((uint32_t *)pp_event_package->payload)[0] =
					SND_AUDIOCODEC_AC3;
				break;
			case ASM_MEDIA_FMT_EAC3:
				((uint32_t *)pp_event_package->payload)[0] =
					SND_AUDIOCODEC_EAC3;
				break;
			case ASM_MEDIA_FMT_DTS:
				((uint32_t *)pp_event_package->payload)[0] =
					SND_AUDIOCODEC_DTS;
				break;
			case ASM_MEDIA_FMT_TRUEHD:
				((uint32_t *)pp_event_package->payload)[0] =
					SND_AUDIOCODEC_TRUEHD;
				break;
			case ASM_MEDIA_FMT_AAC_V2:
				((uint32_t *)pp_event_package->payload)[0] =
					SND_AUDIOCODEC_AAC;
				break;
			default:
				pr_debug("%s: Event with unknown media_fmt 0x%x\n",
					__func__, payload[0]);
			}
		}
		ac->cb(data->opcode, data->token,
			(void *)pp_event_package, ac->priv);
		kfree(pp_event_package);
		spin_unlock_irqrestore(
			&(session[session_id].session_lock), flags);
		return 0;
	case ASM_SESSION_CMDRSP_ADJUST_SESSION_CLOCK_V2:
		if (data->payload_size >= 3 * sizeof(uint32_t))
			pr_debug("%s: ASM_SESSION_CMDRSP_ADJUST_SESSION_CLOCK_V2 sesion %d status 0x%x msw %u lsw %u\n",
				 __func__, ac->session, payload[0], payload[2],
				 payload[1]);
		else
			pr_err("%s: payload size of %x is less than expected.\n",
				__func__, data->payload_size);
		wake_up(&ac->cmd_wait);
		break;
	case ASM_SESSION_CMDRSP_GET_PATH_DELAY_V2:
		if (data->payload_size >= 3 * sizeof(uint32_t))
			pr_debug("%s: ASM_SESSION_CMDRSP_GET_PATH_DELAY_V2 session %d status 0x%x msw %u lsw %u\n",
					__func__, ac->session, payload[0], payload[2],
					payload[1]);
		else
			pr_err("%s: payload size of %x is less than expected.\n",
				__func__, data->payload_size);
		if (payload[0] == 0 && data->payload_size >= 2 * sizeof(uint32_t)) {
			atomic_set(&ac->cmd_state, 0);
			/* ignore msw, as a delay that large shouldn't happen */
			ac->path_delay = payload[1];
		} else {
			atomic_set(&ac->cmd_state, payload[0]);
			ac->path_delay = UINT_MAX;
		}
		wake_up(&ac->cmd_wait);
		break;
	}
	if (ac->cb)
		ac->cb(data->opcode, data->token,
			data->payload, ac->priv);
	spin_unlock_irqrestore(
		&(session[session_id].session_lock), flags);
	return 0;
}

/**
 * q6asm_is_cpu_buf_avail -
 *       retrieve next CPU buf avail
 *
 * @dir: RX or TX direction
 * @ac: Audio client handle
 * @size: size pointer to be updated with size of buffer
 * @index: index pointer to be updated with
 * 	CPU buffer index available
 *
 * Returns buffer pointer on success or NULL on failure
 */
void *q6asm_is_cpu_buf_avail(int dir, struct audio_client *ac, uint32_t *size,
				uint32_t *index)
{
	void *data;
	unsigned char idx;
	struct audio_port_data *port;

	if (!ac || ((dir != IN) && (dir != OUT))) {
		pr_err("%s: ac %pK dir %d\n", __func__, ac, dir);
		return NULL;
	}

	if (ac->io_mode & SYNC_IO_MODE) {
		port = &ac->port[dir];

		mutex_lock(&port->lock);
		idx = port->cpu_buf;
		if (port->buf == NULL) {
			pr_err("%s: Buffer pointer null\n", __func__);
			mutex_unlock(&port->lock);
			return NULL;
		}
		/* dir 0: used = 0 means buf in use
		 * dir 1: used = 1 means buf in use
		 */
		if (port->buf[idx].used == dir) {
			/* To make it more robust, we could loop and get the
			 * next avail buf, its risky though
			 */
			pr_debug("%s: Next buf idx[0x%x] not available, dir[%d]\n",
			 __func__, idx, dir);
			mutex_unlock(&port->lock);
			return NULL;
		}
		*size = port->buf[idx].actual_size;
		*index = port->cpu_buf;
		data = port->buf[idx].data;
		dev_vdbg(ac->dev, "%s: session[%d]index[%d] data[%pK]size[%d]\n",
						__func__,
						ac->session,
						port->cpu_buf,
						data, *size);
		/* By default increase the cpu_buf cnt
		 * user accesses this function,increase cpu
		 * buf(to avoid another api)
		 */
		port->buf[idx].used = dir;
		port->cpu_buf = q6asm_get_next_buf(ac, port->cpu_buf,
						   port->max_buf_cnt);
		mutex_unlock(&port->lock);
		return data;
	}
	return NULL;
}
EXPORT_SYMBOL(q6asm_is_cpu_buf_avail);

/**
 * q6asm_cpu_buf_release -
 *       releases cpu buffer for ASM
 *
 * @dir: RX or TX direction
 * @ac: Audio client handle
 *
 * Returns 0 on success or error on failure
 */
int q6asm_cpu_buf_release(int dir, struct audio_client *ac)
{
	struct audio_port_data *port;
	int ret = 0;
	int idx;

	if (!ac || ((dir != IN) && (dir != OUT))) {
		pr_err("%s: ac %pK dir %d\n", __func__, ac, dir);
		ret = -EINVAL;
		goto exit;
	}

	if (ac->io_mode & SYNC_IO_MODE) {
		port = &ac->port[dir];
		mutex_lock(&port->lock);
		idx = port->cpu_buf;
		if (port->cpu_buf == 0) {
			port->cpu_buf = port->max_buf_cnt - 1;
		} else if (port->cpu_buf < port->max_buf_cnt) {
			port->cpu_buf = port->cpu_buf - 1;
		} else {
			pr_err("%s: buffer index(%d) out of range\n",
			       __func__, port->cpu_buf);
			ret = -EINVAL;
			mutex_unlock(&port->lock);
			goto exit;
		}
		port->buf[port->cpu_buf].used = dir ^ 1;
		mutex_unlock(&port->lock);
	}
exit:
	return ret;
}
EXPORT_SYMBOL(q6asm_cpu_buf_release);

/**
 * q6asm_cpu_buf_release_nolock -
 *       releases cpu buffer for ASM
 *
 * @dir: RX or TX direction
 * @ac: Audio client handle
 *
 * Returns 0 on success or error on failure
 **/

int q6asm_cpu_buf_release_nolock(int dir, struct audio_client *ac)
{
	struct audio_port_data *port;
	int ret = 0;
	int idx;

	if (!ac || ((dir != IN) && (dir != OUT))) {
		pr_err("%s: ac %pK dir %d\n", __func__, ac, dir);
		ret = -EINVAL;
		goto exit;
	}

	if (ac->io_mode & SYNC_IO_MODE) {
		port = &ac->port[dir];
		idx = port->cpu_buf;
		if (port->cpu_buf == 0) {
			port->cpu_buf = port->max_buf_cnt - 1;
		} else if (port->cpu_buf < port->max_buf_cnt) {
			port->cpu_buf = port->cpu_buf - 1;
		} else {
			pr_err("%s: buffer index(%d) out of range\n",
				__func__, port->cpu_buf);
			ret = -EINVAL;
			goto exit;
		}
		port->buf[port->cpu_buf].used = dir ^ 1;
	}
exit:
	return ret;
}
EXPORT_SYMBOL(q6asm_cpu_buf_release_nolock);

/**
 * q6asm_is_cpu_buf_avail_nolock -
 *       retrieve next CPU buf avail without lock acquire
 *
 * @dir: RX or TX direction
 * @ac: Audio client handle
 * @size: size pointer to be updated with size of buffer
 * @index: index pointer to be updated with
 * 	CPU buffer index available
 *
 * Returns buffer pointer on success or NULL on failure
 */
void *q6asm_is_cpu_buf_avail_nolock(int dir, struct audio_client *ac,
					uint32_t *size, uint32_t *index)
{
	void *data;
	unsigned char idx;
	struct audio_port_data *port;

	if (!ac || ((dir != IN) && (dir != OUT))) {
		pr_err("%s: ac %pK dir %d\n", __func__, ac, dir);
		return NULL;
	}

	port = &ac->port[dir];

	idx = port->cpu_buf;
	if (port->buf == NULL) {
		pr_err("%s: Buffer pointer null\n", __func__);
		return NULL;
	}
	/*
	 * dir 0: used = 0 means buf in use
	 * dir 1: used = 1 means buf in use
	 */
	if (port->buf[idx].used == dir) {
		/*
		 * To make it more robust, we could loop and get the
		 * next avail buf, its risky though
		 */
		pr_err("%s: Next buf idx[0x%x] not available, dir[%d]\n",
		 __func__, idx, dir);
		return NULL;
	}
	*size = port->buf[idx].actual_size;
	*index = port->cpu_buf;
	data = port->buf[idx].data;
	dev_vdbg(ac->dev, "%s: session[%d]index[%d] data[%pK]size[%d]\n",
		__func__, ac->session, port->cpu_buf,
		data, *size);
	/*
	 * By default increase the cpu_buf cnt
	 * user accesses this function,increase cpu
	 * buf(to avoid another api)
	 */
	port->buf[idx].used = dir;
	port->cpu_buf = q6asm_get_next_buf(ac, port->cpu_buf,
					   port->max_buf_cnt);
	return data;
}
EXPORT_SYMBOL(q6asm_is_cpu_buf_avail_nolock);

int q6asm_is_dsp_buf_avail(int dir, struct audio_client *ac)
{
	int ret = -1;
	struct audio_port_data *port;
	uint32_t idx;

	if (!ac || (dir != OUT)) {
		pr_err("%s: ac %pK dir %d\n", __func__, ac, dir);
		return ret;
	}

	if (ac->io_mode & SYNC_IO_MODE) {
		port = &ac->port[dir];

		mutex_lock(&port->lock);
		idx = port->dsp_buf;

		if (port->buf[idx].used == (dir ^ 1)) {
			/* To make it more robust, we could loop and get the
			 * next avail buf, its risky though
			 */
			pr_err("%s: Next buf idx[0x%x] not available, dir[%d]\n",
				__func__, idx, dir);
			mutex_unlock(&port->lock);
			return ret;
		}
		dev_vdbg(ac->dev, "%s: session[%d]dsp_buf=%d cpu_buf=%d\n",
			__func__,
			ac->session, port->dsp_buf, port->cpu_buf);
		ret = ((port->dsp_buf != port->cpu_buf) ? 0 : -1);
		mutex_unlock(&port->lock);
	}
	return ret;
}

static void __q6asm_add_hdr(struct audio_client *ac, struct apr_hdr *hdr,
			uint32_t pkt_size, uint32_t cmd_flg, uint32_t stream_id)
{
	unsigned long flags = 0;

	dev_vdbg(ac->dev, "%s: pkt_size=%d cmd_flg=%d session=%d stream_id=%d\n",
			__func__, pkt_size, cmd_flg, ac->session, stream_id);
	mutex_lock(&ac->cmd_lock);
	spin_lock_irqsave(&(session[ac->session].session_lock), flags);
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL", __func__);
		spin_unlock_irqrestore(
			&(session[ac->session].session_lock), flags);
		mutex_unlock(&ac->cmd_lock);
		return;
	}

	hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(sizeof(struct apr_hdr)),
			APR_PKT_VER);
	hdr->src_svc = ((struct apr_svc *)ac->apr)->id;
	hdr->src_domain = APR_DOMAIN_APPS;
	hdr->dest_svc = APR_SVC_ASM;
	hdr->dest_domain = APR_DOMAIN_ADSP;
	hdr->src_port = ((ac->session << 8) & 0xFF00) | (stream_id);
	hdr->dest_port = ((ac->session << 8) & 0xFF00) | (stream_id);
	if (cmd_flg)
		q6asm_update_token(&hdr->token,
				   ac->session,
				   0, /* Stream ID is NA */
				   0, /* Buffer index is NA */
				   0, /* Direction flag is NA */
				   WAIT_CMD);

	hdr->pkt_size  = pkt_size;
	spin_unlock_irqrestore(
		&(session[ac->session].session_lock), flags);
	mutex_unlock(&ac->cmd_lock);
}

static void q6asm_add_hdr(struct audio_client *ac, struct apr_hdr *hdr,
			uint32_t pkt_size, uint32_t cmd_flg)
{
	__q6asm_add_hdr(ac, hdr, pkt_size, cmd_flg, ac->stream_id);
}

static void q6asm_stream_add_hdr(struct audio_client *ac, struct apr_hdr *hdr,
			uint32_t pkt_size, uint32_t cmd_flg, int32_t stream_id)
{
	__q6asm_add_hdr(ac, hdr, pkt_size, cmd_flg, stream_id);
}

static void __q6asm_add_hdr_async(struct audio_client *ac, struct apr_hdr *hdr,
				  uint32_t pkt_size, uint32_t cmd_flg,
				  uint32_t stream_id, u8 no_wait_flag)
{
	dev_vdbg(ac->dev, "%s: pkt_size = %d, cmd_flg = %d, session = %d stream_id=%d\n",
			__func__, pkt_size, cmd_flg, ac->session, stream_id);
	hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(sizeof(struct apr_hdr)),
			APR_PKT_VER);
	if (ac->apr == NULL) {
		pr_err("%s: AC APR is NULL", __func__);
		return;
	}
	hdr->src_svc = ((struct apr_svc *)ac->apr)->id;
	hdr->src_domain = APR_DOMAIN_APPS;
	hdr->dest_svc = APR_SVC_ASM;
	hdr->dest_domain = APR_DOMAIN_ADSP;
	hdr->src_port = ((ac->session << 8) & 0xFF00) | (stream_id);
	hdr->dest_port = ((ac->session << 8) & 0xFF00) | (stream_id);
	if (cmd_flg) {
		q6asm_update_token(&hdr->token,
				   ac->session,
				   0, /* Stream ID is NA */
				   0, /* Buffer index is NA */
				   0, /* Direction flag is NA */
				   no_wait_flag);

	}
	hdr->pkt_size  = pkt_size;
}

static void q6asm_add_hdr_async(struct audio_client *ac, struct apr_hdr *hdr,
				uint32_t pkt_size, uint32_t cmd_flg)
{
	__q6asm_add_hdr_async(ac, hdr, pkt_size, cmd_flg,
			      ac->stream_id, WAIT_CMD);
}

static void q6asm_stream_add_hdr_async(struct audio_client *ac,
					struct apr_hdr *hdr, uint32_t pkt_size,
					uint32_t cmd_flg, int32_t stream_id)
{
	__q6asm_add_hdr_async(ac, hdr, pkt_size, cmd_flg,
			      stream_id, NO_WAIT_CMD);
}

static void q6asm_add_hdr_custom_topology(struct audio_client *ac,
					  struct apr_hdr *hdr,
					  uint32_t pkt_size)
{
	pr_debug("%s: pkt_size=%d session=%d\n",
			__func__, pkt_size, ac->session);
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return;
	}

	mutex_lock(&ac->cmd_lock);
	hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
			APR_HDR_LEN(sizeof(struct apr_hdr)),
			APR_PKT_VER);
	hdr->src_svc = ((struct apr_svc *)ac->apr)->id;
	hdr->src_domain = APR_DOMAIN_APPS;
	hdr->dest_svc = APR_SVC_ASM;
	hdr->dest_domain = APR_DOMAIN_ADSP;
	hdr->src_port = ((ac->session << 8) & 0xFF00) | 0x01;
	hdr->dest_port = 0;
	q6asm_update_token(&hdr->token,
			   ac->session,
			   0, /* Stream ID is NA */
			   0, /* Buffer index is NA */
			   0, /* Direction flag is NA */
			   WAIT_CMD);
	hdr->pkt_size  = pkt_size;
	mutex_unlock(&ac->cmd_lock);
}

static void q6asm_add_mmaphdr(struct audio_client *ac, struct apr_hdr *hdr,
			u32 pkt_size, int dir)
{
	pr_debug("%s: pkt size=%d\n",
		__func__, pkt_size);
	hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	hdr->src_port = 0;
	hdr->dest_port = 0;
	q6asm_update_token(&hdr->token,
			   ac->session,
			   0, /* Stream ID is NA */
			   0, /* Buffer index is NA */
			   dir,
			   WAIT_CMD);
	hdr->pkt_size  = pkt_size;
}

/**
 * q6asm_set_pp_params
 *	command to set ASM parameter data
 *	send memory mapping header for out of band case
 *	send pre-packed parameter data for in band case
 *
 * @ac: audio client handle
 * @mem_hdr: memory mapping header
 * @param_data: pre-packed parameter payload
 * @param_size: size of pre-packed parameter data
 *
 * Returns 0 on success or error on failure
 */
int q6asm_set_pp_params(struct audio_client *ac,
			struct mem_mapping_hdr *mem_hdr, u8 *param_data,
			u32 param_size)
{
	struct asm_stream_cmd_set_pp_params *asm_set_param = NULL;
	int pkt_size = 0;
	int ret = 0;
	int session_id = 0;

	if (ac == NULL) {
		pr_err("%s: Audio Client is NULL\n", __func__);
		return -EINVAL;
	} else if (ac->apr == NULL) {
		pr_err("%s: APR pointer is NULL\n", __func__);
		return -EINVAL;
	}

	session_id = q6asm_get_session_id_from_audio_client(ac);
	if (!session_id)
		return -EINVAL;

	pkt_size = sizeof(struct asm_stream_cmd_set_pp_params);
	/* Add param size to packet size when sending in-band only */
	if (param_data != NULL)
		pkt_size += param_size;
	asm_set_param = kzalloc(pkt_size, GFP_KERNEL);
	if (!asm_set_param)
		return -ENOMEM;

	mutex_lock(&session[session_id].mutex_lock_per_session);
	if (!q6asm_is_valid_audio_client(ac)) {
		ret = -EINVAL;
		goto done;
	}

	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		ret = -EINVAL;
		goto done;
	}

	q6asm_add_hdr_async(ac, &asm_set_param->apr_hdr, pkt_size, TRUE);

	/* With pre-packed data, only the opcode differs from V2 and V3. */
	if (q6common_is_instance_id_supported())
		asm_set_param->apr_hdr.opcode = ASM_STREAM_CMD_SET_PP_PARAMS_V3;
	else
		asm_set_param->apr_hdr.opcode = ASM_STREAM_CMD_SET_PP_PARAMS_V2;

	asm_set_param->payload_size = param_size;

	if (mem_hdr != NULL) {
		/* Out of band case */
		asm_set_param->mem_hdr = *mem_hdr;
	} else if (param_data != NULL) {
		/*
		 * In band case. Parameter data must be pre-packed with its
		 * header before calling this function. Use
		 * q6common_pack_pp_params to pack parameter data and header
		 * correctly.
		 */
		memcpy(&asm_set_param->param_data, param_data, param_size);
	} else {
		pr_err("%s: Received NULL pointers for both mem header and param data\n",
		       __func__);
		ret = -EINVAL;
		goto done;
	}

	atomic_set(&ac->cmd_state_pp, -1);
	ret = apr_send_pkt(ac->apr, (uint32_t *)asm_set_param);
	if (ret < 0) {
		pr_err("%s: apr send failed rc %d\n", __func__, ret);
		ret = -EINVAL;
		goto done;
	}

	ret = wait_event_timeout(ac->cmd_wait,
				 atomic_read(&ac->cmd_state_pp) >= 0,
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: timeout sending apr pkt\n", __func__);
		ret = -ETIMEDOUT;
		goto done;
	}

	if (atomic_read(&ac->cmd_state_pp) > 0) {
		pr_err("%s: DSP returned error[%s]\n", __func__,
		       adsp_err_get_err_str(atomic_read(&ac->cmd_state_pp)));
		ret = adsp_err_get_lnx_err_code(atomic_read(&ac->cmd_state_pp));
		goto done;
	}
	ret = 0;
done:
	mutex_unlock(&session[session_id].mutex_lock_per_session);
	kfree(asm_set_param);
	return ret;
}
EXPORT_SYMBOL(q6asm_set_pp_params);

/**
 * q6asm_pack_and_set_pp_param_in_band
 *	command to pack and set parameter data for in band case
 *
 * @ac: audio client handle
 * @param_hdr: parameter header
 * @param_data: parameter data
 *
 * Returns 0 on success or error on failure
 */
int q6asm_pack_and_set_pp_param_in_band(struct audio_client *ac,
					struct param_hdr_v3 param_hdr,
					u8 *param_data)
{
	u8 *packed_data = NULL;
	u32 packed_size = sizeof(union param_hdrs) + param_hdr.param_size;
	int ret = 0;

        if (ac == NULL) {
                pr_err("%s: Audio Client is NULL\n", __func__);
                return -EINVAL;
        }

	packed_data = kzalloc(packed_size, GFP_KERNEL);
	if (packed_data == NULL)
		return -ENOMEM;

	ret = q6common_pack_pp_params(packed_data, &param_hdr, param_data,
				      &packed_size);
	if (ret) {
		pr_err("%s: Failed to pack params, error %d\n", __func__, ret);
		goto done;
	}

	ret = q6asm_set_pp_params(ac, NULL, packed_data, packed_size);
done:
	kfree(packed_data);
	return ret;
}
EXPORT_SYMBOL(q6asm_pack_and_set_pp_param_in_band);

/**
 * q6asm_set_soft_volume_module_instance_ids
 *	command to set module and instance ids for soft volume
 *
 * @instance: soft volume instance
 * @param_hdr: parameter header
 *
 * Returns 0 on success or error on failure
 */
int q6asm_set_soft_volume_module_instance_ids(int instance,
					      struct param_hdr_v3 *param_hdr)
{
	if (param_hdr == NULL) {
		pr_err("%s: Param header is NULL\n", __func__);
		return -EINVAL;
	}

	switch (instance) {
	case SOFT_VOLUME_INSTANCE_2:
		param_hdr->module_id = ASM_MODULE_ID_VOL_CTRL2;
		param_hdr->instance_id = INSTANCE_ID_0;
		return 0;
	case SOFT_VOLUME_INSTANCE_1:
		param_hdr->module_id = ASM_MODULE_ID_VOL_CTRL;
		param_hdr->instance_id = INSTANCE_ID_0;
		return 0;
	default:
		pr_err("%s: Invalid instance %d\n", __func__, instance);
		return -EINVAL;
	}
}
EXPORT_SYMBOL(q6asm_set_soft_volume_module_instance_ids);

/**
 * q6asm_open_read_compressed -
 *       command to open ASM in compressed read mode
 *
 * @ac: Audio client handle
 * @format: capture format for ASM
 * @passthrough_flag: flag to indicate passthrough option
 *
 * Returns 0 on success or error on failure
 */
int q6asm_open_read_compressed(struct audio_client *ac, uint32_t format,
			       uint32_t passthrough_flag)
{
	int rc = 0;
	struct asm_stream_cmd_open_read_compressed open;

	if (ac == NULL) {
		pr_err("%s: ac[%pK] NULL\n",  __func__, ac);
		rc = -EINVAL;
		goto fail_cmd;
	}

	if (ac->apr == NULL) {
		pr_err("%s: APR handle[%pK] NULL\n", __func__,  ac->apr);
		rc = -EINVAL;
		goto fail_cmd;
	}
	pr_debug("%s: session[%d] wr_format[0x%x]\n", __func__, ac->session,
		format);

	q6asm_add_hdr(ac, &open.hdr, sizeof(open), TRUE);
	open.hdr.opcode = ASM_STREAM_CMD_OPEN_READ_COMPRESSED;
	atomic_set(&ac->cmd_state, -1);

	/*
	 * Below flag indicates whether DSP shall keep IEC61937 packing or
	 * unpack to raw compressed format
	 */
	if (format == FORMAT_IEC61937) {
		open.mode_flags = 0x1;
		open.frames_per_buf = 1;
		pr_debug("%s: Flag 1 IEC61937 output\n", __func__);
	} else if (format == FORMAT_DSD) {
		open.mode_flags = ASM_DSD_FORMAT_FLAG;
		open.frames_per_buf = 1;
		pr_debug("%s: Flag 2 DSD output\n", __func__);
	} else {
		open.mode_flags = 0;
		open.frames_per_buf = 1;
		pr_debug("%s: Flag 0 RAW_COMPR output\n", __func__);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &open);
	if (rc < 0) {
		pr_err("%s: open failed op[0x%x]rc[%d]\n",
			__func__, open.hdr.opcode, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for OPEN_READ_COMPR rc[%d]\n",
			__func__, rc);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
			atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}

	return 0;

fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_open_read_compressed);

static int __q6asm_open_read(struct audio_client *ac,
			     uint32_t format, uint16_t bits_per_sample,
			     uint32_t pcm_format_block_ver,
			     bool ts_mode, uint32_t enc_cfg_id)
{
	int rc = 0x00;
	struct asm_stream_cmd_open_read_v3 open;
	struct q6asm_cal_info cal_info;

	config_debug_fs_reset_index();

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: session[%d]\n", __func__, ac->session);

	q6asm_add_hdr(ac, &open.hdr, sizeof(open), TRUE);
	atomic_set(&ac->cmd_state, -1);
	open.hdr.opcode = ASM_STREAM_CMD_OPEN_READ_V3;
	/* Stream prio : High, provide meta info with encoded frames */
	open.src_endpointype = ASM_END_POINT_DEVICE_MATRIX;

	rc = q6asm_get_asm_topology_apptype(&cal_info, ac);
	open.preprocopo_id = cal_info.topology_id;


	open.bits_per_sample = bits_per_sample;
	open.mode_flags = 0x0;

	ac->topology = open.preprocopo_id;
	ac->app_type = cal_info.app_type;
	if (ac->perf_mode == LOW_LATENCY_PCM_MODE) {
		open.mode_flags |= ASM_LOW_LATENCY_TX_STREAM_SESSION <<
			ASM_SHIFT_STREAM_PERF_MODE_FLAG_IN_OPEN_READ;
	} 
	else if (ac->perf_mode == LOW_LATENCY_PCM_NOPROC_MODE) {
		open.mode_flags |= ASM_LOW_LATENCY_NPROC_TX_STREAM_SESSION <<
			ASM_SHIFT_STREAM_PERF_MODE_FLAG_IN_OPEN_READ;
	}
	else {
		open.mode_flags |= ASM_LEGACY_STREAM_SESSION <<
			ASM_SHIFT_STREAM_PERF_MODE_FLAG_IN_OPEN_READ;
	}

	switch (format) {
	case FORMAT_LINEAR_PCM:
		open.mode_flags |= 0x00;
		open.enc_cfg_id = q6asm_get_pcm_format_id(pcm_format_block_ver);
		if (ts_mode)
			open.mode_flags |= ABSOLUTE_TIMESTAMP_ENABLE;
		break;
	case FORMAT_MPEG4_AAC:
		open.mode_flags |= BUFFER_META_ENABLE;
		open.enc_cfg_id = ASM_MEDIA_FMT_AAC_V2;
		break;
	case FORMAT_G711_ALAW_FS:
		open.mode_flags |= BUFFER_META_ENABLE;
		open.enc_cfg_id = ASM_MEDIA_FMT_G711_ALAW_FS;
		break;
	case FORMAT_G711_MLAW_FS:
		open.mode_flags |= BUFFER_META_ENABLE;
		open.enc_cfg_id = ASM_MEDIA_FMT_G711_MLAW_FS;
		break;
	case FORMAT_V13K:
		open.mode_flags |= BUFFER_META_ENABLE;
		open.enc_cfg_id = ASM_MEDIA_FMT_V13K_FS;
		break;
	case FORMAT_EVRC:
		open.mode_flags |= BUFFER_META_ENABLE;
		open.enc_cfg_id = ASM_MEDIA_FMT_EVRC_FS;
		break;
	case FORMAT_AMRNB:
		open.mode_flags |= BUFFER_META_ENABLE;
		open.enc_cfg_id = ASM_MEDIA_FMT_AMRNB_FS;
		break;
	case FORMAT_AMRWB:
		open.mode_flags |= BUFFER_META_ENABLE;
		open.enc_cfg_id = ASM_MEDIA_FMT_AMRWB_FS;
		break;
	case FORMAT_BESPOKE:
		open.mode_flags |= BUFFER_META_ENABLE;
		open.enc_cfg_id = enc_cfg_id;
		if (ts_mode)
			open.mode_flags |= ABSOLUTE_TIMESTAMP_ENABLE;
		break;
	default:
		pr_err("%s: Invalid format 0x%x\n",
			__func__, format);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = apr_send_pkt(ac->apr, (uint32_t *) &open);
	if (rc < 0) {
		pr_err("%s: open failed op[0x%x]rc[%d]\n",
				__func__, open.hdr.opcode, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for open read\n",
				__func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		if (rc != -EALREADY)
			pr_err("%s: DSP returned error[%s]\n",
					__func__, adsp_err_get_err_str(
					atomic_read(&ac->cmd_state)));
		goto fail_cmd;
	}

	ac->io_mode |= TUN_READ_IO_MODE;

	return 0;
fail_cmd:
	return rc;
}

/**
 * q6asm_open_read -
 *       command to open ASM in read mode
 *
 * @ac: Audio client handle
 * @format: capture format for ASM
 *
 * Returns 0 on success or error on failure
 */
int q6asm_open_read(struct audio_client *ac,
		uint32_t format)
{
	return __q6asm_open_read(ac, format, 16,
				PCM_MEDIA_FORMAT_V2 /*media fmt block ver*/,
				false/*ts_mode*/, ENC_CFG_ID_NONE);
}
EXPORT_SYMBOL(q6asm_open_read);

int q6asm_open_read_v2(struct audio_client *ac, uint32_t format,
			uint16_t bits_per_sample)
{
	return __q6asm_open_read(ac, format, bits_per_sample,
				 PCM_MEDIA_FORMAT_V2 /*media fmt block ver*/,
				 false/*ts_mode*/, ENC_CFG_ID_NONE);
}

/*
 * asm_open_read_v3 - Opens audio capture session
 *
 * @ac: Client session handle
 * @format: encoder format
 * @bits_per_sample: bit width of capture session
 */
int q6asm_open_read_v3(struct audio_client *ac, uint32_t format,
			uint16_t bits_per_sample)
{
	return __q6asm_open_read(ac, format, bits_per_sample,
				 PCM_MEDIA_FORMAT_V3/*media fmt block ver*/,
				 false/*ts_mode*/, ENC_CFG_ID_NONE);
}
EXPORT_SYMBOL(q6asm_open_read_v3);

/*
 * asm_open_read_v4 - Opens audio capture session
 *
 * @ac: Client session handle
 * @format: encoder format
 * @bits_per_sample: bit width of capture session
 * @ts_mode: timestamp mode
 */
int q6asm_open_read_v4(struct audio_client *ac, uint32_t format,
			uint16_t bits_per_sample, bool ts_mode,
			uint32_t enc_cfg_id)
{
	return __q6asm_open_read(ac, format, bits_per_sample,
				 PCM_MEDIA_FORMAT_V4 /*media fmt block ver*/,
				 ts_mode, enc_cfg_id);
}
EXPORT_SYMBOL(q6asm_open_read_v4);


/*
 * asm_open_read_v5 - Opens audio capture session
 *
 * @ac: Client session handle
 * @format: encoder format
 * @bits_per_sample: bit width of capture session
 * @ts_mode: timestamp mode
 */
int q6asm_open_read_v5(struct audio_client *ac, uint32_t format,
			uint16_t bits_per_sample, bool ts_mode, uint32_t enc_cfg_id)
{
	return __q6asm_open_read(ac, format, bits_per_sample,
				 PCM_MEDIA_FORMAT_V5 /*media fmt block ver*/,
				 ts_mode, enc_cfg_id);
}
EXPORT_SYMBOL(q6asm_open_read_v5);

static int q6asm_open_read_version_adaptor(struct audio_client *ac,
			uint32_t format, uint16_t bits_per_sample, bool ts_mode)
{
	if (q6core_get_avcs_api_version_per_service(
			APRV2_IDS_SERVICE_ID_ADSP_ASM_V) >=
			ADSP_ASM_API_VERSION_V2)
		return q6asm_open_read_v5(ac, FORMAT_LINEAR_PCM,
						bits_per_sample, false, ENC_CFG_ID_NONE);
	else
		return q6asm_open_read_v4(ac, FORMAT_LINEAR_PCM,
						bits_per_sample, false, ENC_CFG_ID_NONE);
}

/*
 * q6asm_open_read_with_retry - Opens audio capture session, with retrying
 * in case of session ID conflict
 *
 * @ac: Client session handle
 * @format: encoder format
 * @bits_per_sample: bit width of capture session
 * @ts_mode: timestamp mode
 */
int q6asm_open_read_with_retry(struct audio_client *ac, uint32_t format,
			uint16_t bits_per_sample, bool ts_mode)
{
	int i = 0, rc = 0;

	mutex_lock(&session_lock);
	for (i = 0; i < ASM_ACTIVE_STREAMS_ALLOWED; i++) {
		rc = q6asm_open_read_version_adaptor(ac, format,
				bits_per_sample, ts_mode);
		if (rc != -EALREADY)
			break;

		pr_debug("%s: session %d is occupied, try next\n",
				__func__, ac->session);
		q6asm_session_set_ignore(ac->session);
		rc = q6asm_session_try_next(ac);
		if (rc < 0)
			break;
	}
	q6asm_session_clean_ignore();
	mutex_unlock(&session_lock);

	return rc;
}
EXPORT_SYMBOL(q6asm_open_read_with_retry);

/**
 * q6asm_open_write_compressed -
 *       command to open ASM in compressed write mode
 *
 * @ac: Audio client handle
 * @format: playback format for ASM
 * @passthrough_flag: flag to indicate passthrough option
 *
 * Returns 0 on success or error on failure
 */
int q6asm_open_write_compressed(struct audio_client *ac, uint32_t format,
				uint32_t passthrough_flag)
{
	int rc = 0;
	struct asm_stream_cmd_open_write_compressed open;

	if (ac == NULL) {
		pr_err("%s: ac[%pK] NULL\n",  __func__, ac);
		rc = -EINVAL;
		goto fail_cmd;
	}

	if (ac->apr == NULL) {
		pr_err("%s: APR handle[%pK] NULL\n", __func__,  ac->apr);
		rc = -EINVAL;
		goto fail_cmd;
	}
	pr_debug("%s: session[%d] wr_format[0x%x]", __func__, ac->session,
		format);

	q6asm_add_hdr(ac, &open.hdr, sizeof(open), TRUE);
	open.hdr.opcode = ASM_STREAM_CMD_OPEN_WRITE_COMPRESSED;
	atomic_set(&ac->cmd_state, -1);

	switch (format) {
	case FORMAT_AC3:
		open.fmt_id = ASM_MEDIA_FMT_AC3;
		break;
	case FORMAT_EAC3:
		open.fmt_id = ASM_MEDIA_FMT_EAC3;
		break;
	case FORMAT_DTS:
		open.fmt_id = ASM_MEDIA_FMT_DTS;
		break;
	case FORMAT_DSD:
		open.fmt_id = ASM_MEDIA_FMT_DSD;
		break;
	case FORMAT_GEN_COMPR:
		open.fmt_id = ASM_MEDIA_FMT_GENERIC_COMPRESSED;
		break;
	case FORMAT_TRUEHD:
		open.fmt_id = ASM_MEDIA_FMT_TRUEHD;
		break;
	case FORMAT_IEC61937:
		open.fmt_id = ASM_MEDIA_FMT_IEC;
		break;
	default:
		pr_err("%s: Invalid format[%d]\n", __func__, format);
		rc = -EINVAL;
		goto fail_cmd;
	}
	/* Below flag indicates the DSP that Compressed audio input
	 * stream is not IEC 61937 or IEC 60958 packetizied
	 */
	if (passthrough_flag == COMPRESSED_PASSTHROUGH ||
		passthrough_flag == COMPRESSED_PASSTHROUGH_DSD ||
		passthrough_flag == COMPRESSED_PASSTHROUGH_GEN) {
		open.flags = 0x0;
		pr_debug("%s: Flag 0 COMPRESSED_PASSTHROUGH\n", __func__);
	} else if (passthrough_flag == COMPRESSED_PASSTHROUGH_CONVERT) {
		open.flags = 0x8;
		pr_debug("%s: Flag 8 - COMPRESSED_PASSTHROUGH_CONVERT\n",
			 __func__);
	} else if (passthrough_flag == COMPRESSED_PASSTHROUGH_IEC61937) {
		open.flags = 0x1;
		pr_debug("%s: Flag 1 - COMPRESSED_PASSTHROUGH_IEC61937\n",
			 __func__);
	} else {
		pr_err("%s: Invalid passthrough type[%d]\n",
			__func__, passthrough_flag);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = apr_send_pkt(ac->apr, (uint32_t *) &open);
	if (rc < 0) {
		pr_err("%s: open failed op[0x%x]rc[%d]\n",
			__func__, open.hdr.opcode, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
		(atomic_read(&ac->cmd_state) >= 0),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for OPEN_WRITE_COMPR rc[%d]\n",
			__func__, rc);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}

	return 0;

fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_open_write_compressed);

static int __q6asm_open_write(struct audio_client *ac, uint32_t format,
			      uint16_t bits_per_sample, uint32_t stream_id,
			      bool is_gapless_mode,
			      uint32_t pcm_format_block_ver)
{
	int rc = 0x00;
	struct asm_stream_cmd_open_write_v3 open;
	struct q6asm_cal_info cal_info;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}

	dev_vdbg(ac->dev, "%s: session[%d] wr_format[0x%x]\n",
		__func__, ac->session, format);

	q6asm_stream_add_hdr(ac, &open.hdr, sizeof(open), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);
	/*
	 * Updated the token field with stream/session for compressed playback
	 * Platform driver must know the the stream with which the command is
	 * associated
	 */
	if (ac->io_mode & COMPRESSED_STREAM_IO)
		q6asm_update_token(&open.hdr.token,
				   ac->session,
				   stream_id,
				   0, /* Buffer index is NA */
				   0, /* Direction flag is NA */
				   WAIT_CMD);

	dev_vdbg(ac->dev, "%s: token = 0x%x, stream_id  %d, session 0x%x\n",
			__func__, open.hdr.token, stream_id, ac->session);
	open.hdr.opcode = ASM_STREAM_CMD_OPEN_WRITE_V3;
	open.mode_flags = 0x00;
	if (ac->perf_mode == ULL_POST_PROCESSING_PCM_MODE)
		open.mode_flags |= ASM_ULL_POST_PROCESSING_STREAM_SESSION;
	else if (ac->perf_mode == ULTRA_LOW_LATENCY_PCM_MODE)
		open.mode_flags |= ASM_ULTRA_LOW_LATENCY_STREAM_SESSION;
	else if (ac->perf_mode == LOW_LATENCY_PCM_MODE)
		open.mode_flags |= ASM_LOW_LATENCY_STREAM_SESSION;
	else if (ac->perf_mode == LOW_LATENCY_PCM_NOPROC_MODE)
		open.mode_flags |= ASM_ULTRA_LOW_LATENCY_NPROC_STREAM_SESSION;
	else {
		open.mode_flags |= ASM_LEGACY_STREAM_SESSION;
		if (is_gapless_mode)
			open.mode_flags |= 1 << ASM_SHIFT_GAPLESS_MODE_FLAG;
	}

	/* source endpoint : matrix */
	open.sink_endpointype = ASM_END_POINT_DEVICE_MATRIX;
	open.bits_per_sample = bits_per_sample;

	rc = q6asm_get_asm_topology_apptype(&cal_info, ac);
	open.postprocopo_id = cal_info.topology_id;

	if ((ac->perf_mode != LEGACY_PCM_MODE) && (ac->perf_mode != LOW_LATENCY_PCM_MODE))
		open.postprocopo_id = ASM_STREAM_POSTPROCOPO_ID_NONE;

	pr_debug("%s: perf_mode %d asm_topology 0x%x bps %d\n", __func__,
		 ac->perf_mode, open.postprocopo_id, open.bits_per_sample);

	/*
	 * For Gapless playback it will use the same session for next stream,
	 * So use the same topology
	 */
	if (!ac->topology) {
		ac->topology = open.postprocopo_id;
		ac->app_type = cal_info.app_type;
	}
	switch (format) {
	case FORMAT_LINEAR_PCM:
		open.dec_fmt_id = q6asm_get_pcm_format_id(pcm_format_block_ver);
		break;
	case FORMAT_MPEG4_AAC:
		open.dec_fmt_id = ASM_MEDIA_FMT_AAC_V2;
		break;
	case FORMAT_MPEG4_MULTI_AAC:
		open.dec_fmt_id = ASM_MEDIA_FMT_AAC_V2;
		break;
	case FORMAT_WMA_V9:
		open.dec_fmt_id = ASM_MEDIA_FMT_WMA_V9_V2;
		break;
	case FORMAT_WMA_V10PRO:
		open.dec_fmt_id = ASM_MEDIA_FMT_WMA_V10PRO_V2;
		break;
	case FORMAT_AMRNB:
		open.dec_fmt_id = ASM_MEDIA_FMT_AMRNB_FS;
		break;
	case FORMAT_AMRWB:
		open.dec_fmt_id = ASM_MEDIA_FMT_AMRWB_FS;
		break;
	case FORMAT_AMR_WB_PLUS:
		open.dec_fmt_id = ASM_MEDIA_FMT_AMR_WB_PLUS_V2;
		break;
	case FORMAT_MP3:
		open.dec_fmt_id = ASM_MEDIA_FMT_MP3;
		break;
	case FORMAT_AC3:
		open.dec_fmt_id = ASM_MEDIA_FMT_AC3;
		break;
	case FORMAT_EAC3:
		open.dec_fmt_id = ASM_MEDIA_FMT_EAC3;
		break;
	case FORMAT_MP2:
		open.dec_fmt_id = ASM_MEDIA_FMT_MP2;
		break;
	case FORMAT_FLAC:
		open.dec_fmt_id = ASM_MEDIA_FMT_FLAC;
		break;
	case FORMAT_ALAC:
		open.dec_fmt_id = ASM_MEDIA_FMT_ALAC;
		break;
	case FORMAT_VORBIS:
		open.dec_fmt_id = ASM_MEDIA_FMT_VORBIS;
		break;
	case FORMAT_APE:
		open.dec_fmt_id = ASM_MEDIA_FMT_APE;
		break;
	case FORMAT_DSD:
		open.dec_fmt_id = ASM_MEDIA_FMT_DSD;
		break;
	case FORMAT_APTX:
		open.dec_fmt_id = ASM_MEDIA_FMT_APTX;
		break;
	case FORMAT_GEN_COMPR:
		open.dec_fmt_id = ASM_MEDIA_FMT_GENERIC_COMPRESSED;
		break;
	default:
		pr_err("%s: Invalid format 0x%x\n", __func__, format);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = apr_send_pkt(ac->apr, (uint32_t *) &open);
	if (rc < 0) {
		pr_err("%s: open failed op[0x%x]rc[%d]\n",
				__func__, open.hdr.opcode, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for open write\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		if (rc != -EALREADY)
			pr_err("%s: DSP returned error[%s]\n",
					__func__, adsp_err_get_err_str(
					atomic_read(&ac->cmd_state)));
		goto fail_cmd;
	}
	ac->io_mode |= TUN_WRITE_IO_MODE;

	return 0;
fail_cmd:
	return rc;
}

int q6asm_open_write(struct audio_client *ac, uint32_t format)
{
	return __q6asm_open_write(ac, format, 16, ac->stream_id,
				  false /*gapless*/,
				  PCM_MEDIA_FORMAT_V2 /*pcm_format_block_ver*/);
}
EXPORT_SYMBOL(q6asm_open_write);

int q6asm_open_write_v2(struct audio_client *ac, uint32_t format,
			uint16_t bits_per_sample)
{
	return __q6asm_open_write(ac, format, bits_per_sample,
				  ac->stream_id, false /*gapless*/,
				  PCM_MEDIA_FORMAT_V2 /*pcm_format_block_ver*/);
}

/*
 * q6asm_open_write_v3 - Opens audio playback session
 *
 * @ac: Client session handle
 * @format: decoder format
 * @bits_per_sample: bit width of playback session
 */
int q6asm_open_write_v3(struct audio_client *ac, uint32_t format,
			uint16_t bits_per_sample)
{
	return __q6asm_open_write(ac, format, bits_per_sample,
				  ac->stream_id, false /*gapless*/,
				  PCM_MEDIA_FORMAT_V3 /*pcm_format_block_ver*/);
}
EXPORT_SYMBOL(q6asm_open_write_v3);

/*
 * q6asm_open_write_v4 - Opens audio playback session
 *
 * @ac: Client session handle
 * @format: decoder format
 * @bits_per_sample: bit width of playback session
 */
int q6asm_open_write_v4(struct audio_client *ac, uint32_t format,
			uint16_t bits_per_sample)
{
	return __q6asm_open_write(ac, format, bits_per_sample,
				  ac->stream_id, false /*gapless*/,
				  PCM_MEDIA_FORMAT_V4 /*pcm_format_block_ver*/);
}
EXPORT_SYMBOL(q6asm_open_write_v4);

static int q6asm_open_write_version_adaptor(struct audio_client *ac,
			uint32_t format, uint16_t bits_per_sample)
{
	if (q6core_get_avcs_api_version_per_service(
			APRV2_IDS_SERVICE_ID_ADSP_ASM_V) >=
			ADSP_ASM_API_VERSION_V2)
		return q6asm_open_write_v5(ac, format, bits_per_sample);
	else
		return q6asm_open_write_v4(ac, format, bits_per_sample);
}

/*
 * q6asm_open_write_with_retry - Opens audio playback session, with retrying
 * in case of session ID conflict
 *
 * @ac: Client session handle
 * @format: decoder format
 * @bits_per_sample: bit width of playback session
 */
int q6asm_open_write_with_retry(struct audio_client *ac, uint32_t format,
			uint16_t bits_per_sample)
{
	int i = 0, rc = 0;

	mutex_lock(&session_lock);
	for (i = 0; i < ASM_ACTIVE_STREAMS_ALLOWED; i++) {
		rc = q6asm_open_write_version_adaptor(ac, format,
						bits_per_sample);
		if (rc != -EALREADY)
			break;

		pr_debug("%s: session %d is occupied, try next\n",
				__func__, ac->session);
		q6asm_session_set_ignore(ac->session);
		rc = q6asm_session_try_next(ac);
		if (rc < 0)
			break;
	}
	q6asm_session_clean_ignore();
	mutex_unlock(&session_lock);

	return rc;
}
EXPORT_SYMBOL(q6asm_open_write_with_retry);

int q6asm_stream_open_write_v2(struct audio_client *ac, uint32_t format,
			       uint16_t bits_per_sample, int32_t stream_id,
			       bool is_gapless_mode)
{
	return __q6asm_open_write(ac, format, bits_per_sample,
				  stream_id, is_gapless_mode,
				  PCM_MEDIA_FORMAT_V2 /*pcm_format_block_ver*/);
}

/*
 * q6asm_stream_open_write_v3 - Creates audio stream for playback
 *
 * @ac: Client session handle
 * @format: asm playback format
 * @bits_per_sample: bit width of requested stream
 * @stream_id: stream id of stream to be associated with this session
 * @is_gapless_mode: true if gapless mode needs to be enabled
 */
int q6asm_stream_open_write_v3(struct audio_client *ac, uint32_t format,
			       uint16_t bits_per_sample, int32_t stream_id,
			       bool is_gapless_mode)
{
	return __q6asm_open_write(ac, format, bits_per_sample,
				  stream_id, is_gapless_mode,
				  PCM_MEDIA_FORMAT_V3 /*pcm_format_block_ver*/);
}
EXPORT_SYMBOL(q6asm_stream_open_write_v3);

/*
 * q6asm_open_write_v5 - Opens audio playback session
 *
 * @ac: Client session handle
 * @format: decoder format
 * @bits_per_sample: bit width of playback session
 */
int q6asm_open_write_v5(struct audio_client *ac, uint32_t format,
			uint16_t bits_per_sample)
{
	return __q6asm_open_write(ac, format, bits_per_sample,
				  ac->stream_id, false /*gapless*/,
				  PCM_MEDIA_FORMAT_V5 /*pcm_format_block_ver*/);
}
EXPORT_SYMBOL(q6asm_open_write_v5);


/*
 * q6asm_stream_open_write_v4 - Creates audio stream for playback
 *
 * @ac: Client session handle
 * @format: asm playback format
 * @bits_per_sample: bit width of requested stream
 * @stream_id: stream id of stream to be associated with this session
 * @is_gapless_mode: true if gapless mode needs to be enabled
 */
int q6asm_stream_open_write_v4(struct audio_client *ac, uint32_t format,
			       uint16_t bits_per_sample, int32_t stream_id,
			       bool is_gapless_mode)
{
	return __q6asm_open_write(ac, format, bits_per_sample,
				  stream_id, is_gapless_mode,
				  PCM_MEDIA_FORMAT_V4 /*pcm_format_block_ver*/);
}
EXPORT_SYMBOL(q6asm_stream_open_write_v4);

/*
 * q6asm_stream_open_write_v5 - Creates audio stream for playback
 *
 * @ac: Client session handle
 * @format: asm playback format
 * @bits_per_sample: bit width of requested stream
 * @stream_id: stream id of stream to be associated with this session
 * @is_gapless_mode: true if gapless mode needs to be enabled
 */
int q6asm_stream_open_write_v5(struct audio_client *ac, uint32_t format,
			       uint16_t bits_per_sample, int32_t stream_id,
			       bool is_gapless_mode)
{
	return __q6asm_open_write(ac, format, bits_per_sample,
				  stream_id, is_gapless_mode,
				  PCM_MEDIA_FORMAT_V5 /*pcm_format_block_ver*/);
}
EXPORT_SYMBOL(q6asm_stream_open_write_v5);


static int __q6asm_open_read_write(struct audio_client *ac, uint32_t rd_format,
				   uint32_t wr_format, bool is_meta_data_mode,
				   uint32_t bits_per_sample,
				   bool overwrite_topology, int topology)
{
	int rc = 0x00;
	struct asm_stream_cmd_open_readwrite_v2 open;
	struct q6asm_cal_info cal_info;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: session[%d]\n", __func__, ac->session);
	pr_debug("%s: wr_format[0x%x]rd_format[0x%x]\n",
			__func__, wr_format, rd_format);

	ac->io_mode |= NT_MODE;
	q6asm_add_hdr(ac, &open.hdr, sizeof(open), TRUE);
	atomic_set(&ac->cmd_state, -1);
	open.hdr.opcode = ASM_STREAM_CMD_OPEN_READWRITE_V2;

	open.mode_flags = is_meta_data_mode ? BUFFER_META_ENABLE : 0;
	open.bits_per_sample = bits_per_sample;
	/* source endpoint : matrix */
	rc = q6asm_get_asm_topology_apptype(&cal_info, ac);
	open.postprocopo_id = cal_info.topology_id;

	open.postprocopo_id = overwrite_topology ?
			      topology : open.postprocopo_id;
	ac->topology = open.postprocopo_id;
	ac->app_type = cal_info.app_type;


	switch (wr_format) {
	case FORMAT_LINEAR_PCM:
	case FORMAT_MULTI_CHANNEL_LINEAR_PCM:
		open.dec_fmt_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2;
		break;
	case FORMAT_MPEG4_AAC:
		open.dec_fmt_id = ASM_MEDIA_FMT_AAC_V2;
		break;
	case FORMAT_MPEG4_MULTI_AAC:
		open.dec_fmt_id = ASM_MEDIA_FMT_AAC_V2;
		break;
	case FORMAT_WMA_V9:
		open.dec_fmt_id = ASM_MEDIA_FMT_WMA_V9_V2;
		break;
	case FORMAT_WMA_V10PRO:
		open.dec_fmt_id = ASM_MEDIA_FMT_WMA_V10PRO_V2;
		break;
	case FORMAT_AMRNB:
		open.dec_fmt_id = ASM_MEDIA_FMT_AMRNB_FS;
		break;
	case FORMAT_AMRWB:
		open.dec_fmt_id = ASM_MEDIA_FMT_AMRWB_FS;
		break;
	case FORMAT_AMR_WB_PLUS:
		open.dec_fmt_id = ASM_MEDIA_FMT_AMR_WB_PLUS_V2;
		break;
	case FORMAT_V13K:
		open.dec_fmt_id = ASM_MEDIA_FMT_V13K_FS;
		break;
	case FORMAT_EVRC:
		open.dec_fmt_id = ASM_MEDIA_FMT_EVRC_FS;
		break;
	case FORMAT_EVRCB:
		open.dec_fmt_id = ASM_MEDIA_FMT_EVRCB_FS;
		break;
	case FORMAT_EVRCWB:
		open.dec_fmt_id = ASM_MEDIA_FMT_EVRCWB_FS;
		break;
	case FORMAT_MP3:
		open.dec_fmt_id = ASM_MEDIA_FMT_MP3;
		break;
	case FORMAT_ALAC:
		open.dec_fmt_id = ASM_MEDIA_FMT_ALAC;
		break;
	case FORMAT_APE:
		open.dec_fmt_id = ASM_MEDIA_FMT_APE;
		break;
	case FORMAT_DSD:
		open.dec_fmt_id = ASM_MEDIA_FMT_DSD;
		break;
	case FORMAT_G711_ALAW_FS:
		open.dec_fmt_id = ASM_MEDIA_FMT_G711_ALAW_FS;
		break;
	case FORMAT_G711_MLAW_FS:
		open.dec_fmt_id = ASM_MEDIA_FMT_G711_MLAW_FS;
		break;
	default:
		pr_err("%s: Invalid format 0x%x\n",
				__func__, wr_format);
		rc = -EINVAL;
		goto fail_cmd;
	}

	switch (rd_format) {
	case FORMAT_LINEAR_PCM:
	case FORMAT_MULTI_CHANNEL_LINEAR_PCM:
		open.enc_cfg_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2;
		break;
	case FORMAT_MPEG4_AAC:
		open.enc_cfg_id = ASM_MEDIA_FMT_AAC_V2;
		break;
	case FORMAT_G711_ALAW_FS:
		open.enc_cfg_id = ASM_MEDIA_FMT_G711_ALAW_FS;
		break;
	case FORMAT_G711_MLAW_FS:
		open.enc_cfg_id = ASM_MEDIA_FMT_G711_MLAW_FS;
		break;
	case FORMAT_V13K:
		open.enc_cfg_id = ASM_MEDIA_FMT_V13K_FS;
		break;
	case FORMAT_EVRC:
		open.enc_cfg_id = ASM_MEDIA_FMT_EVRC_FS;
		break;
	case FORMAT_AMRNB:
		open.enc_cfg_id = ASM_MEDIA_FMT_AMRNB_FS;
		break;
	case FORMAT_AMRWB:
		open.enc_cfg_id = ASM_MEDIA_FMT_AMRWB_FS;
		break;
	case FORMAT_ALAC:
		open.enc_cfg_id = ASM_MEDIA_FMT_ALAC;
		break;
	case FORMAT_APE:
		open.enc_cfg_id = ASM_MEDIA_FMT_APE;
		break;
	default:
		pr_err("%s: Invalid format 0x%x\n",
				__func__, rd_format);
		rc = -EINVAL;
		goto fail_cmd;
	}
	dev_vdbg(ac->dev, "%s: rdformat[0x%x]wrformat[0x%x]\n", __func__,
			open.enc_cfg_id, open.dec_fmt_id);

	rc = apr_send_pkt(ac->apr, (uint32_t *) &open);
	if (rc < 0) {
		pr_err("%s: open failed op[0x%x]rc[%d]\n",
				__func__, open.hdr.opcode, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for open read-write\n",
				__func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		if (rc != -EALREADY)
			pr_err("%s: DSP returned error[%s]\n",
					__func__, adsp_err_get_err_str(
					atomic_read(&ac->cmd_state)));
		goto fail_cmd;
	}

	return 0;
fail_cmd:
	return rc;
}

/**
 * q6asm_open_read_write -
 *       command to open ASM in read/write mode
 *
 * @ac: Audio client handle
 * @rd_format: capture format for ASM
 * @wr_format: playback format for ASM
 *
 * Returns 0 on success or error on failure
 */
int q6asm_open_read_write(struct audio_client *ac, uint32_t rd_format,
			  uint32_t wr_format)
{
	return __q6asm_open_read_write(ac, rd_format, wr_format,
				       true/*meta data mode*/,
				       16 /*bits_per_sample*/,
				       false /*overwrite_topology*/, 0);
}
EXPORT_SYMBOL(q6asm_open_read_write);

/**
 * q6asm_open_read_write_v2 -
 *       command to open ASM in bi-directional read/write mode
 *
 * @ac: Audio client handle
 * @rd_format: capture format for ASM
 * @wr_format: playback format for ASM
 * @is_meta_data_mode: mode to indicate if meta data present
 * @bits_per_sample: number of bits per sample
 * @overwrite_topology: topology to be overwritten flag
 * @topology: Topology for ASM
 *
 * Returns 0 on success or error on failure
 */
int q6asm_open_read_write_v2(struct audio_client *ac, uint32_t rd_format,
			     uint32_t wr_format, bool is_meta_data_mode,
			     uint32_t bits_per_sample, bool overwrite_topology,
			     int topology)
{
	return __q6asm_open_read_write(ac, rd_format, wr_format,
				       is_meta_data_mode, bits_per_sample,
				       overwrite_topology, topology);
}
EXPORT_SYMBOL(q6asm_open_read_write_v2);

/**
 * q6asm_open_loopback_v2 -
 *       command to open ASM in loopback mode
 *
 * @ac: Audio client handle
 * @bits_per_sample: number of bits per sample
 *
 * Returns 0 on success or error on failure
 */
int q6asm_open_loopback_v2(struct audio_client *ac, uint16_t bits_per_sample)
{
	int rc = 0x00;
	struct q6asm_cal_info cal_info;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: session[%d]\n", __func__, ac->session);

	if (ac->perf_mode == LOW_LATENCY_PCM_MODE) {
		struct asm_stream_cmd_open_transcode_loopback_t open;

		q6asm_add_hdr(ac, &open.hdr, sizeof(open), TRUE);
		atomic_set(&ac->cmd_state, -1);
		open.hdr.opcode = ASM_STREAM_CMD_OPEN_TRANSCODE_LOOPBACK;

		open.mode_flags = 0;
		open.src_endpoint_type = 0;
		open.sink_endpoint_type = 0;
		open.src_format_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2;
		open.sink_format_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V2;
		/* source endpoint : matrix */
		rc = q6asm_get_asm_topology_apptype(&cal_info, ac);
		open.audproc_topo_id = cal_info.topology_id;

		ac->app_type = cal_info.app_type;
		if (ac->perf_mode == LOW_LATENCY_PCM_MODE)
			open.mode_flags |= ASM_LOW_LATENCY_STREAM_SESSION;
		else
			open.mode_flags |= ASM_LEGACY_STREAM_SESSION;
		ac->topology = open.audproc_topo_id;
		open.bits_per_sample = bits_per_sample;
		open.reserved = 0;
		pr_debug("%s: opening a transcode_loopback with mode_flags =[%d] session[%d]\n",
				__func__, open.mode_flags, ac->session);

		rc = apr_send_pkt(ac->apr, (uint32_t *) &open);
		if (rc < 0) {
			pr_err("%s: open failed op[0x%x]rc[%d]\n",
					__func__, open.hdr.opcode, rc);
			rc = -EINVAL;
			goto fail_cmd;
		}
	} else {/*if(ac->perf_mode == LEGACY_PCM_MODE)*/
		struct asm_stream_cmd_open_loopback_v2 open;

		q6asm_add_hdr(ac, &open.hdr, sizeof(open), TRUE);
		atomic_set(&ac->cmd_state, -1);
		open.hdr.opcode = ASM_STREAM_CMD_OPEN_LOOPBACK_V2;

		open.mode_flags = 0;
		open.src_endpointype = 0;
		open.sink_endpointype = 0;
		/* source endpoint : matrix */
		rc = q6asm_get_asm_topology_apptype(&cal_info, ac);
		open.postprocopo_id = cal_info.topology_id;

		ac->app_type = cal_info.app_type;
		ac->topology = open.postprocopo_id;
		open.bits_per_sample = bits_per_sample;
		open.reserved = 0;
		pr_debug("%s: opening a loopback_v2 with mode_flags =[%d] session[%d]\n",
				__func__, open.mode_flags, ac->session);

		rc = apr_send_pkt(ac->apr, (uint32_t *) &open);
		if (rc < 0) {
			pr_err("%s: open failed op[0x%x]rc[%d]\n",
					__func__, open.hdr.opcode, rc);
			rc = -EINVAL;
			goto fail_cmd;
		}
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for open_loopback\n",
				__func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}

	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_open_loopback_v2);

/*
 * q6asm_open_loopback_with_retry - Opens audio loopback session, with retrying
 * in case of session ID conflict
 *
 * @ac: Client session handle
 * @bits_per_sample: bit width of sample
 */
int q6asm_open_loopback_with_retry(struct audio_client *ac,
				uint16_t bits_per_sample)
{
	int i = 0, rc = 0;

	mutex_lock(&session_lock);
	for (i = 0; i < ASM_ACTIVE_STREAMS_ALLOWED; i++) {
		rc = q6asm_open_loopback_v2(ac, bits_per_sample);
		if (rc != -EALREADY)
			break;
		pr_debug("%s: session %d is occupied, try next\n",
					__func__, ac->session);
		q6asm_session_set_ignore(ac->session);
		rc = q6asm_session_try_next(ac);
		if (rc < 0)
			break;
	}
	q6asm_session_clean_ignore();
	mutex_unlock(&session_lock);

	return rc;
}
EXPORT_SYMBOL(q6asm_open_loopback_with_retry);

/**
 * q6asm_open_transcode_loopback -
 *       command to open ASM in transcode loopback mode
 *
 * @ac: Audio client handle
 * @bits_per_sample: number of bits per sample
 * @source_format: Format of clip
 * @sink_format: end device supported format
 *
 * Returns 0 on success or error on failure
 */
int q6asm_open_transcode_loopback(struct audio_client *ac,
			uint16_t bits_per_sample,
			uint32_t source_format, uint32_t sink_format)
{
	int rc = 0x00;
	struct asm_stream_cmd_open_transcode_loopback_t open;
	struct q6asm_cal_info cal_info;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]\n", __func__, ac->session);

	q6asm_add_hdr(ac, &open.hdr, sizeof(open), TRUE);
	atomic_set(&ac->cmd_state, -1);
	open.hdr.opcode = ASM_STREAM_CMD_OPEN_TRANSCODE_LOOPBACK;

	open.mode_flags = 0;
	open.src_endpoint_type = 0;
	open.sink_endpoint_type = 0;
	switch (source_format) {
	case FORMAT_LINEAR_PCM:
	case FORMAT_MULTI_CHANNEL_LINEAR_PCM:
		open.src_format_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V3;
		break;
	case FORMAT_AC3:
		open.src_format_id = ASM_MEDIA_FMT_AC3;
		break;
	case FORMAT_EAC3:
		open.src_format_id = ASM_MEDIA_FMT_EAC3;
		break;
	default:
		pr_err("%s: Unsupported src fmt [%d]\n",
		       __func__, source_format);
		return -EINVAL;
	}
	switch (sink_format) {
	case FORMAT_LINEAR_PCM:
	case FORMAT_MULTI_CHANNEL_LINEAR_PCM:
		open.sink_format_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V3;
		break;
	default:
		pr_err("%s: Unsupported sink fmt [%d]\n",
		       __func__, sink_format);
		return -EINVAL;
	}

	/* source endpoint : matrix */
	rc = q6asm_get_asm_topology_apptype(&cal_info, ac);
	open.audproc_topo_id = cal_info.topology_id;


	ac->app_type = cal_info.app_type;
	if (ac->perf_mode == LOW_LATENCY_PCM_MODE)
		open.mode_flags |= ASM_LOW_LATENCY_STREAM_SESSION;
	else
		open.mode_flags |= ASM_LEGACY_STREAM_SESSION;
	ac->topology = open.audproc_topo_id;
	open.bits_per_sample = bits_per_sample;
	open.reserved = 0;
	pr_debug("%s: opening a transcode_loopback with mode_flags =[%d] session[%d]\n",
		__func__, open.mode_flags, ac->session);

	rc = apr_send_pkt(ac->apr, (uint32_t *) &open);
	if (rc < 0) {
		pr_err("%s: open failed op[0x%x]rc[%d]\n",
				__func__, open.hdr.opcode, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for open_transcode_loopback\n",
			__func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
					atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}

	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_open_transcode_loopback);

static
int q6asm_set_shared_circ_buff(struct audio_client *ac,
			       struct asm_stream_cmd_open_shared_io *open,
			       int bufsz, int bufcnt,
			       int dir)
{
	struct audio_buffer *buf_circ;
	int bytes_to_alloc, rc;
	size_t len;

	mutex_lock(&ac->cmd_lock);

	if (ac->port[dir].buf) {
		pr_err("%s: Buffer already allocated\n", __func__);
		rc = -EINVAL;
		goto done;
	}

	buf_circ = kzalloc(sizeof(struct audio_buffer), GFP_KERNEL);

	if (!buf_circ) {
		rc = -ENOMEM;
		goto done;
	}

	bytes_to_alloc = bufsz * bufcnt;
	bytes_to_alloc = PAGE_ALIGN(bytes_to_alloc);

	rc = msm_audio_ion_alloc(&buf_circ->dma_buf,
			bytes_to_alloc,
			&buf_circ->phys,
			&len, &buf_circ->data);

	if (rc) {
		pr_err("%s: Audio ION alloc is failed, rc = %d\n", __func__,
				rc);
		kfree(buf_circ);
		goto done;
	}

	ac->port[dir].buf = buf_circ;
	buf_circ->used = dir ^ 1;
	buf_circ->size = bytes_to_alloc;
	buf_circ->actual_size = bytes_to_alloc;
	memset(buf_circ->data, 0, buf_circ->actual_size);

	ac->port[dir].max_buf_cnt = 1;

	open->shared_circ_buf_mem_pool_id = ADSP_MEMORY_MAP_SHMEM8_4K_POOL;
	open->shared_circ_buf_num_regions = 1;
	open->shared_circ_buf_property_flag = 0x00;
	open->shared_circ_buf_start_phy_addr_lsw =
			lower_32_bits(buf_circ->phys);
	open->shared_circ_buf_start_phy_addr_msw =
			msm_audio_populate_upper_32_bits(buf_circ->phys);
	open->shared_circ_buf_size = bufsz * bufcnt;

	open->map_region_circ_buf.shm_addr_lsw = lower_32_bits(buf_circ->phys);
	open->map_region_circ_buf.shm_addr_msw =
			msm_audio_populate_upper_32_bits(buf_circ->phys);
	open->map_region_circ_buf.mem_size_bytes = bytes_to_alloc;

done:
	mutex_unlock(&ac->cmd_lock);
	return rc;
}


static
int q6asm_set_shared_pos_buff(struct audio_client *ac,
			       struct asm_stream_cmd_open_shared_io *open,
			       int dir)
{
	struct audio_buffer *buf_pos = &ac->shared_pos_buf;
	int rc;
	size_t len;
	int bytes_to_alloc = sizeof(struct asm_shared_position_buffer);

	mutex_lock(&ac->cmd_lock);

	bytes_to_alloc = PAGE_ALIGN(bytes_to_alloc);

	rc = msm_audio_ion_alloc(&buf_pos->dma_buf,
			bytes_to_alloc,
			&buf_pos->phys, &len,
			&buf_pos->data);

	if (rc) {
		pr_err("%s: Audio pos buf ION alloc is failed, rc = %d\n",
				__func__, rc);
		goto done;
	}

	buf_pos->used = dir ^ 1;
	buf_pos->size = bytes_to_alloc;
	buf_pos->actual_size = bytes_to_alloc;

	open->shared_pos_buf_mem_pool_id = ADSP_MEMORY_MAP_SHMEM8_4K_POOL;
	open->shared_pos_buf_num_regions = 1;
	open->shared_pos_buf_property_flag = 0x00;
	open->shared_pos_buf_phy_addr_lsw = lower_32_bits(buf_pos->phys);
	open->shared_pos_buf_phy_addr_msw =
			msm_audio_populate_upper_32_bits(buf_pos->phys);

	open->map_region_pos_buf.shm_addr_lsw = lower_32_bits(buf_pos->phys);
	open->map_region_pos_buf.shm_addr_msw =
			msm_audio_populate_upper_32_bits(buf_pos->phys);
	open->map_region_pos_buf.mem_size_bytes = bytes_to_alloc;

done:
	mutex_unlock(&ac->cmd_lock);
	return rc;
}

/*
 * q6asm_open_shared_io: Open an ASM session for pull mode (playback)
 * or push mode (capture).
 * parameters
 *   config - session parameters (channels, bits_per_sample, sr)
 *   dir - stream direction (IN for playback, OUT for capture)
 *   use_default_chmap: true if default channel map to be used
 *   channel_map: input channel map
 * returns 0 if successful, error code otherwise
 */
int q6asm_open_shared_io(struct audio_client *ac,
			 struct shared_io_config *config,
			 int dir, bool use_default_chmap, u8 *channel_map)
{
	struct asm_stream_cmd_open_shared_io *open;
	u8 *channel_mapping;
	int i, size_of_open, num_watermarks, bufsz, bufcnt, rc, flags = 0;
	struct q6asm_cal_info cal_info;

	if (!ac || !config)
		return -EINVAL;

	if (!use_default_chmap && (channel_map == NULL)) {
		pr_err("%s: No valid chan map and can't use default\n",
			__func__);
		return -EINVAL;
	}

	if (config->channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__,
			config->channels);
		return -EINVAL;
	}

	bufsz = config->bufsz;
	bufcnt = config->bufcnt;
	num_watermarks = 0;

	ac->config = *config;

	if (ac->session <= 0 || ac->session > SESSION_MAX) {
		pr_err("%s: Session %d is out of bounds\n",
			__func__, ac->session);
		return -EINVAL;
	}

	size_of_open = sizeof(struct asm_stream_cmd_open_shared_io) +
		(sizeof(struct asm_shared_watermark_level) * num_watermarks);

	open = kzalloc(PAGE_ALIGN(size_of_open), GFP_KERNEL);
	if (!open)
		return -ENOMEM;

	q6asm_stream_add_hdr(ac, &open->hdr, size_of_open, TRUE,
				ac->stream_id);

	atomic_set(&ac->cmd_state, 1);

	pr_debug("%s: token = 0x%x, stream_id %d, session 0x%x, perf %d\n",
		 __func__, open->hdr.token, ac->stream_id, ac->session,
		 ac->perf_mode);

	open->hdr.opcode =
		dir == IN ? ASM_STREAM_CMD_OPEN_PULL_MODE_WRITE :
		ASM_STREAM_CMD_OPEN_PUSH_MODE_READ;

	pr_debug("%s perf_mode %d\n", __func__, ac->perf_mode);
	if (dir == IN)
		if (ac->perf_mode == ULL_POST_PROCESSING_PCM_MODE)
			flags = 4 << ASM_SHIFT_STREAM_PERF_FLAG_PULL_MODE_WRITE;
		else if (ac->perf_mode == ULTRA_LOW_LATENCY_PCM_MODE)
			flags = 2 << ASM_SHIFT_STREAM_PERF_FLAG_PULL_MODE_WRITE;
		else if (ac->perf_mode == LOW_LATENCY_PCM_MODE)
			flags = 1 << ASM_SHIFT_STREAM_PERF_FLAG_PULL_MODE_WRITE;
		else
			pr_err("Invalid perf mode for pull write\n");
	else
		if (ac->perf_mode == LOW_LATENCY_PCM_MODE)
			flags = ASM_LOW_LATENCY_TX_STREAM_SESSION <<
				ASM_SHIFT_STREAM_PERF_FLAG_PUSH_MODE_READ;
		else
			pr_err("Invalid perf mode for push read\n");

	if (flags == 0) {
		pr_err("%s: Invalid mode[%d]\n", __func__,
		       ac->perf_mode);
		kfree(open);
		return -EINVAL;

	}

	pr_debug("open.mode_flags = 0x%x\n", flags);
	open->mode_flags = flags;
	open->endpoint_type = ASM_END_POINT_DEVICE_MATRIX;
	open->topo_bits_per_sample = config->bits_per_sample;

	rc = q6asm_get_asm_topology_apptype(&cal_info, ac);
	open->topo_id = cal_info.topology_id;

	if (config->format == FORMAT_LINEAR_PCM)
		open->fmt_id = ASM_MEDIA_FMT_MULTI_CHANNEL_PCM_V3;
	else {
		pr_err("%s: Invalid format[%d]\n", __func__, config->format);
		rc = -EINVAL;
		goto done;
	}

	rc = q6asm_set_shared_circ_buff(ac, open, bufsz, bufcnt, dir);

	if (rc)
		goto done;

	ac->port[dir].tmp_hdl = 0;

	rc = q6asm_set_shared_pos_buff(ac, open, dir);

	if (rc)
		goto done;

	/* asm_multi_channel_pcm_fmt_blk_v3 */
	open->fmt.num_channels = config->channels;
	open->fmt.bits_per_sample = config->bits_per_sample;
	open->fmt.sample_rate = config->rate;
	open->fmt.is_signed = 1;
	open->fmt.sample_word_size = config->sample_word_size;

	channel_mapping = open->fmt.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		rc = q6asm_map_channels(channel_mapping, config->channels,
					false);
		if (rc) {
			pr_err("%s: Map channels failed, ret: %d\n",
				 __func__, rc);
			goto done;
		}
	} else {
		memcpy(channel_mapping, channel_map,
				PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	open->num_watermark_levels = num_watermarks;
	for (i = 0; i < num_watermarks; i++) {
		open->watermark[i].watermark_level_bytes = i *
				((bufsz * bufcnt) / num_watermarks);
		pr_debug("%s: Watermark level set for %i\n",
				__func__,
				open->watermark[i].watermark_level_bytes);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) open);
	if (rc < 0) {
		pr_err_ratelimited("%s: Open failed op[0x%x]rc[%d]\n",
		       __func__, open->hdr.opcode, rc);
		goto done;
	}

	pr_debug("%s: sent open apr pkt\n", __func__);
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) <= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: Timeout. Waited for open write apr pkt rc[%d]\n",
		       __func__, rc);
		rc = -ETIMEDOUT;
		goto done;
	}

	if (atomic_read(&ac->cmd_state) < 0) {
		pr_err("%s: DSP returned error [%d]\n", __func__,
				atomic_read(&ac->cmd_state));
		rc = -EINVAL;
		goto done;
	}

	ac->io_mode |= TUN_WRITE_IO_MODE;
	rc = 0;
done:
	kfree(open);
	return rc;
}
EXPORT_SYMBOL(q6asm_open_shared_io);

/*
 * q6asm_shared_io_buf: Returns handle to the shared circular buffer being
 * used for pull/push mode.
 * parameters
 *   dir - used to identify input/output port
 * returns buffer handle
 */
struct audio_buffer *q6asm_shared_io_buf(struct audio_client *ac,
					 int dir)
{
	struct audio_port_data *port;

	if (!ac) {
		pr_err("%s: ac is null\n", __func__);
		return NULL;
	}
	port = &ac->port[dir];
	return port->buf;
}
EXPORT_SYMBOL(q6asm_shared_io_buf);

/*
 * q6asm_shared_io_free: Frees memory allocated for a pull/push session
 * parameters
 *  dir - port direction
 * returns 0 if successful, error otherwise
 */
int q6asm_shared_io_free(struct audio_client *ac, int dir)
{
	struct audio_port_data *port;

	if (!ac) {
		pr_err("%s: audio client is null\n", __func__);
		return -EINVAL;
	}
	port = &ac->port[dir];
	mutex_lock(&ac->cmd_lock);
	if (port->buf && port->buf->data) {
		msm_audio_ion_free(port->buf->dma_buf);
		port->buf->dma_buf = NULL;
		port->max_buf_cnt = 0;
		kfree(port->buf);
		port->buf = NULL;
	}
	if (ac->shared_pos_buf.data) {
		msm_audio_ion_free(ac->shared_pos_buf.dma_buf);
		ac->shared_pos_buf.dma_buf = NULL;
	}
	mutex_unlock(&ac->cmd_lock);
	return 0;
}
EXPORT_SYMBOL(q6asm_shared_io_free);

/*
 * q6asm_get_shared_pos: Returns current read index/write index as observed
 * by the DSP. Note that this is an offset and iterates from [0,BUF_SIZE - 1]
 * parameters - (all output)
 *   read_index - offset
 *   wall_clk_msw1 - ADSP wallclock msw
 *   wall_clk_lsw1 - ADSP wallclock lsw
 * returns 0 if successful, -EAGAIN if DSP failed to update after some
 * retries
 */
int q6asm_get_shared_pos(struct audio_client *ac, uint32_t *read_index,
			 uint32_t *wall_clk_msw1, uint32_t *wall_clk_lsw1)
{
	struct asm_shared_position_buffer *pos_buf;
	uint32_t frame_cnt1, frame_cnt2;
	int i, j;

	if (!ac) {
		pr_err("%s: audio client is null\n", __func__);
		return -EINVAL;
	}

	pos_buf = ac->shared_pos_buf.data;

	/* always try to get the latest update in the shared pos buffer */
	for (i = 0; i < 2; i++) {
		/* retry until there is an update from DSP */
		for (j = 0; j < 5; j++) {
			frame_cnt1 = pos_buf->frame_counter;
			if (frame_cnt1 != 0)
				break;
		}

		*wall_clk_msw1 = pos_buf->wall_clock_us_msw;
		*wall_clk_lsw1 = pos_buf->wall_clock_us_lsw;
		*read_index = pos_buf->index;
		frame_cnt2 = pos_buf->frame_counter;

		if (frame_cnt1 != frame_cnt2)
			continue;
		return 0;
	}
	pr_err("%s out of tries trying to get a good read, try again\n",
	       __func__);
	return -EAGAIN;
}
EXPORT_SYMBOL(q6asm_get_shared_pos);

/**
 * q6asm_run -
 *       command to set ASM to run state
 *
 * @ac: Audio client handle
 * @flags: Flags for session
 * @msw_ts: upper 32bits timestamp
 * @lsw_ts: lower 32bits timestamp
 *
 * Returns 0 on success or error on failure
 */
int q6asm_run(struct audio_client *ac, uint32_t flags,
		uint32_t msw_ts, uint32_t lsw_ts)
{
	struct asm_session_cmd_run_v2 run;
	int rc;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: session[%d]\n", __func__, ac->session);

	q6asm_add_hdr(ac, &run.hdr, sizeof(run), TRUE);
	atomic_set(&ac->cmd_state, -1);

	run.hdr.opcode = ASM_SESSION_CMD_RUN_V2;
	run.flags    = flags;
	run.time_lsw = lsw_ts;
	run.time_msw = msw_ts;

	config_debug_fs_run();

	rc = apr_send_pkt(ac->apr, (uint32_t *) &run);
	if (rc < 0) {
		pr_err("%s: Commmand run failed[%d]",
				__func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}

	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for run success",
				__func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}

	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_run);

static int __q6asm_run_nowait(struct audio_client *ac, uint32_t flags,
		uint32_t msw_ts, uint32_t lsw_ts, uint32_t stream_id)
{
	struct asm_session_cmd_run_v2 run;
	int rc;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: session[%d]\n", __func__, ac->session);

	q6asm_stream_add_hdr_async(ac, &run.hdr, sizeof(run), TRUE, stream_id);
	atomic_set(&ac->cmd_state, 1);
	run.hdr.opcode = ASM_SESSION_CMD_RUN_V2;
	run.flags    = flags;
	run.time_lsw = lsw_ts;
	run.time_msw = msw_ts;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &run);
	if (rc < 0) {
		pr_err("%s: Commmand run failed[%d]", __func__, rc);
		return -EINVAL;
	}
	return 0;
}

/**
 * q6asm_run_nowait -
 *       command to set ASM to run state with no wait for ack
 *
 * @ac: Audio client handle
 * @flags: Flags for session
 * @msw_ts: upper 32bits timestamp
 * @lsw_ts: lower 32bits timestamp
 *
 * Returns 0 on success or error on failure
 */
int q6asm_run_nowait(struct audio_client *ac, uint32_t flags,
			uint32_t msw_ts, uint32_t lsw_ts)
{
	return __q6asm_run_nowait(ac, flags, msw_ts, lsw_ts, ac->stream_id);
}
EXPORT_SYMBOL(q6asm_run_nowait);

int q6asm_stream_run_nowait(struct audio_client *ac, uint32_t flags,
			uint32_t msw_ts, uint32_t lsw_ts, uint32_t stream_id)
{
	return __q6asm_run_nowait(ac, flags, msw_ts, lsw_ts, stream_id);
}

/**
 * q6asm_enc_cfg_blk_custom -
 *       command to set encode cfg block for custom
 *
 * @ac: Audio client handle
 * @sample_rate: Sample rate
 * @channels: number of ASM channels
 * @format: custom format flag
 * @cfg: generic encoder config
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enc_cfg_blk_custom(struct audio_client *ac,
			uint32_t sample_rate, uint32_t channels,
			uint32_t format, void *cfg)
{
	struct asm_custom_enc_cfg_t_v2 enc_cfg;
	int rc = 0;
	uint32_t custom_size;
	struct snd_enc_generic *enc_generic = (struct snd_enc_generic *) cfg;

	custom_size = enc_generic->reserved[1];

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: session[%d] size[%d] res[2]=[%d] res[3]=[%d]\n",
		 __func__, ac->session, custom_size, enc_generic->reserved[2],
		 enc_generic->reserved[3]);

	pr_debug("%s: res[4]=[%d] sr[%d] ch[%d] format[%d]\n",
		 __func__, enc_generic->reserved[4], sample_rate,
		 channels, format);

	memset(&enc_cfg, 0, sizeof(struct asm_custom_enc_cfg_t_v2));
	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);

	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(struct asm_custom_enc_cfg_t_v2) -
				sizeof(struct asm_stream_cmd_set_encdec_param);
	enc_cfg.encblk.frames_per_buf = ENC_FRAMES_PER_BUFFER;
	enc_cfg.encblk.enc_cfg_blk_size  = enc_cfg.encdec.param_size -
				sizeof(struct asm_enc_cfg_blk_param_v2);

	enc_cfg.num_channels = channels;
	enc_cfg.sample_rate = sample_rate;

	if (q6asm_map_channels(enc_cfg.channel_mapping, channels, false)) {
		pr_err("%s: map channels failed %d\n",
		       __func__, channels);
		rc = -EINVAL;
		goto fail_cmd;
	}

	if (format == FORMAT_BESPOKE && custom_size &&
		custom_size <= sizeof(enc_cfg.custom_data)) {
		memcpy(enc_cfg.custom_data, &enc_generic->reserved[2],
			   custom_size);
		enc_cfg.custom_size = custom_size;
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Comamnd %d failed %d\n",
		       __func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for FORMAT_UPDATE\n",
			__func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
					__func__, adsp_err_get_err_str(
					atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_custom);

/**
 * q6asm_enc_cfg_blk_aac -
 *       command to set encode cfg block for aac
 *
 * @ac: Audio client handle
 * @frames_per_buf: number of frames per buffer
 * @sample_rate: Sample rate
 * @channels: number of ASM channels
 * @bit_rate: Bit rate info
 * @mode: mode of AAC stream encode
 * @format: aac format flag
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enc_cfg_blk_aac(struct audio_client *ac,
			 uint32_t frames_per_buf,
			uint32_t sample_rate, uint32_t channels,
			uint32_t bit_rate, uint32_t mode, uint32_t format)
{
	struct asm_aac_enc_cfg_v2 enc_cfg;
	int rc = 0;

	pr_debug("%s: session[%d]frames[%d]SR[%d]ch[%d]bitrate[%d]mode[%d] format[%d]\n",
		 __func__, ac->session, frames_per_buf,
		sample_rate, channels, bit_rate, mode, format);

	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);

	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(struct asm_aac_enc_cfg_v2) -
				sizeof(struct asm_stream_cmd_set_encdec_param);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size  = enc_cfg.encdec.param_size -
				sizeof(struct asm_enc_cfg_blk_param_v2);
	enc_cfg.bit_rate = bit_rate;
	enc_cfg.enc_mode = mode;
	enc_cfg.aac_fmt_flag = format;
	enc_cfg.channel_cfg = channels;
	enc_cfg.sample_rate = sample_rate;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Comamnd %d failed %d\n",
			__func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for FORMAT_UPDATE\n",
			__func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_aac);

/**
 * q6asm_enc_cfg_blk_g711 -
 *       command to set encode cfg block for g711
 *
 * @ac: Audio client handle
 * @frames_per_buf: number of frames per buffer
 * @sample_rate: Sample rate
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enc_cfg_blk_g711(struct audio_client *ac,
			uint32_t frames_per_buf,
			uint32_t sample_rate)
{
	struct asm_g711_enc_cfg_v2 enc_cfg;
	int rc = 0;

	pr_debug("%s: session[%d]frames[%d]SR[%d]\n",
		 __func__, ac->session, frames_per_buf,
		sample_rate);

	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);

	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(struct asm_g711_enc_cfg_v2) -
				sizeof(struct asm_stream_cmd_set_encdec_param);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size  = enc_cfg.encdec.param_size -
				sizeof(struct asm_enc_cfg_blk_param_v2);
	enc_cfg.sample_rate = sample_rate;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Comamnd %d failed %d\n",
			__func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for FORMAT_UPDATE\n",
			__func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_g711);

/**
 * q6asm_set_encdec_chan_map -
 *       command to set encdec channel map
 *
 * @ac: Audio client handle
 * @channels: number of channels
 *
 * Returns 0 on success or error on failure
 */
int q6asm_set_encdec_chan_map(struct audio_client *ac,
			uint32_t num_channels)
{
	struct asm_dec_out_chan_map_param chan_map;
	u8 *channel_mapping;
	int rc = 0;

	if (num_channels > MAX_CHAN_MAP_CHANNELS) {
		pr_err("%s: Invalid channel count %d\n", __func__,
			num_channels);
		return -EINVAL;
	}

	pr_debug("%s: Session %d, num_channels = %d\n",
			 __func__, ac->session, num_channels);
	q6asm_add_hdr(ac, &chan_map.hdr, sizeof(chan_map), TRUE);
	atomic_set(&ac->cmd_state, -1);
	chan_map.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	chan_map.encdec.param_id = ASM_PARAM_ID_DEC_OUTPUT_CHAN_MAP;
	chan_map.encdec.param_size = sizeof(struct asm_dec_out_chan_map_param) -
			 (sizeof(struct apr_hdr) +
			 sizeof(struct asm_stream_cmd_set_encdec_param));
	chan_map.num_channels = num_channels;
	channel_mapping = chan_map.channel_mapping;
	memset(channel_mapping, PCM_CHANNEL_NULL, MAX_CHAN_MAP_CHANNELS);

	if (q6asm_map_channels(channel_mapping, num_channels, false)) {
		pr_err("%s: map channels failed %d\n", __func__, num_channels);
		return -EINVAL;
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &chan_map);
	if (rc < 0) {
		pr_err("%s: Command opcode[0x%x]paramid[0x%x] failed %d\n",
			   __func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM,
			   ASM_PARAM_ID_DEC_OUTPUT_CHAN_MAP, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout opcode[0x%x]\n", __func__,
			   chan_map.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
		return rc;
}
EXPORT_SYMBOL(q6asm_set_encdec_chan_map);

/*
 * q6asm_enc_cfg_blk_pcm_v5 - sends encoder configuration parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @use_default_chmap: true if default channel map  to be used
 * @use_back_flavor: to configure back left and right channel
 * @channel_map: input channel map
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 * @endianness: endianness of the pcm data
 * @mode: Mode to provide additional info about the pcm input data
 */
static int q6asm_enc_cfg_blk_pcm_v5(struct audio_client *ac,
			     uint32_t rate, uint32_t channels,
			     uint16_t bits_per_sample, bool use_default_chmap,
			     bool use_back_flavor, u8 *channel_map,
			     uint16_t sample_word_size, uint16_t endianness,
			     uint16_t mode)
{
	struct asm_multi_channel_pcm_enc_cfg_v5 enc_cfg;
	struct asm_enc_cfg_blk_param_v2 enc_fg_blk;
	u8 *channel_mapping;
	u32 frames_per_buf = 0;
	int rc;

	if (!use_default_chmap && (channel_map == NULL)) {
		pr_err("%s: No valid chan map and can't use default\n",
				__func__);
		rc = -EINVAL;
		goto fail_cmd;
	}

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL_V8) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		rc = -EINVAL;
		goto fail_cmd;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]bps[%d]wordsize[%d]\n", __func__,
		 ac->session, rate, channels,
		 bits_per_sample, sample_word_size);

	memset(&enc_cfg, 0, sizeof(enc_cfg));
	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);
	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(enc_cfg) - sizeof(enc_cfg.hdr) -
				    sizeof(enc_cfg.encdec);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size = enc_cfg.encdec.param_size -
					  sizeof(enc_fg_blk);
	enc_cfg.num_channels = channels;
	enc_cfg.bits_per_sample = bits_per_sample;
	enc_cfg.sample_rate = rate;
	enc_cfg.is_signed = 1;
	enc_cfg.sample_word_size = sample_word_size;
	enc_cfg.endianness = endianness;
	enc_cfg.mode = mode;
	channel_mapping = enc_cfg.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8);

	if (use_default_chmap) {
		pr_debug("%s: setting default channel map for %d channels",
			 __func__, channels);
		if (q6asm_map_channels(channel_mapping, channels,
					use_back_flavor)) {
			pr_err("%s: map channels failed %d\n",
			       __func__, channels);
			rc = -EINVAL;
			goto fail_cmd;
		}
	} else {
		pr_debug("%s: Using pre-defined channel map", __func__);
		memcpy(channel_mapping, channel_map,
			PCM_FORMAT_MAX_NUM_CHANNEL_V8);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Command open failed %d\n", __func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0), 5*HZ);
	if (!rc) {
		pr_err("%s: timeout opcode[0x%x]\n",
		       __func__, enc_cfg.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
		       __func__, adsp_err_get_err_str(
		       atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}

/*
 * q6asm_enc_cfg_blk_pcm_v4 - sends encoder configuration parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @use_default_chmap: true if default channel map  to be used
 * @use_back_flavor: to configure back left and right channel
 * @channel_map: input channel map
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 * @endianness: endianness of the pcm data
 * @mode: Mode to provide additional info about the pcm input data
 */
int q6asm_enc_cfg_blk_pcm_v4(struct audio_client *ac,
			     uint32_t rate, uint32_t channels,
			     uint16_t bits_per_sample, bool use_default_chmap,
			     bool use_back_flavor, u8 *channel_map,
			     uint16_t sample_word_size, uint16_t endianness,
			     uint16_t mode)
{
	struct asm_multi_channel_pcm_enc_cfg_v4 enc_cfg;
	struct asm_enc_cfg_blk_param_v2 enc_fg_blk;
	u8 *channel_mapping;
	u32 frames_per_buf = 0;
	int rc;

	if (!use_default_chmap && (channel_map == NULL)) {
		pr_err("%s: No valid chan map and can't use default\n",
				__func__);
		rc = -EINVAL;
		goto fail_cmd;
	}

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		rc = -EINVAL;
		goto fail_cmd;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]bps[%d]wordsize[%d]\n", __func__,
		 ac->session, rate, channels,
		 bits_per_sample, sample_word_size);

	memset(&enc_cfg, 0, sizeof(enc_cfg));
	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);
	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(enc_cfg) - sizeof(enc_cfg.hdr) -
				    sizeof(enc_cfg.encdec);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size = enc_cfg.encdec.param_size -
					  sizeof(enc_fg_blk);
	enc_cfg.num_channels = channels;
	enc_cfg.bits_per_sample = bits_per_sample;
	enc_cfg.sample_rate = rate;
	enc_cfg.is_signed = 1;
	enc_cfg.sample_word_size = sample_word_size;
	enc_cfg.endianness = endianness;
	enc_cfg.mode = mode;
	channel_mapping = enc_cfg.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		pr_debug("%s: setting default channel map for %d channels",
			 __func__, channels);
		if (q6asm_map_channels(channel_mapping, channels,
					use_back_flavor)) {
			pr_err("%s: map channels failed %d\n",
			       __func__, channels);
			rc = -EINVAL;
			goto fail_cmd;
		}
	} else {
		pr_debug("%s: Using pre-defined channel map", __func__);
		memcpy(channel_mapping, channel_map,
			PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Command open failed %d\n", __func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout opcode[0x%x]\n",
		       __func__, enc_cfg.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
		       __func__, adsp_err_get_err_str(
		       atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_pcm_v4);

/*
 * q6asm_enc_cfg_blk_pcm_v3 - sends encoder configuration parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @use_default_chmap: true if default channel map  to be used
 * @use_back_flavor: to configure back left and right channel
 * @channel_map: input channel map
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 */
int q6asm_enc_cfg_blk_pcm_v3(struct audio_client *ac,
			     uint32_t rate, uint32_t channels,
			     uint16_t bits_per_sample, bool use_default_chmap,
			     bool use_back_flavor, u8 *channel_map,
			     uint16_t sample_word_size)
{
	struct asm_multi_channel_pcm_enc_cfg_v3 enc_cfg;
	struct asm_enc_cfg_blk_param_v2 enc_fg_blk;
	u8 *channel_mapping;
	u32 frames_per_buf = 0;
	int rc;

	if (!use_default_chmap && (channel_map == NULL)) {
		pr_err("%s: No valid chan map and can't use default\n",
				__func__);
		rc = -EINVAL;
		goto fail_cmd;
	}

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		rc = -EINVAL;
		goto fail_cmd;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]bps[%d]wordsize[%d]\n", __func__,
		 ac->session, rate, channels,
		 bits_per_sample, sample_word_size);

	memset(&enc_cfg, 0, sizeof(enc_cfg));
	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);
	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(enc_cfg) - sizeof(enc_cfg.hdr) -
				    sizeof(enc_cfg.encdec);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size = enc_cfg.encdec.param_size -
					  sizeof(enc_fg_blk);
	enc_cfg.num_channels = channels;
	enc_cfg.bits_per_sample = bits_per_sample;
	enc_cfg.sample_rate = rate;
	enc_cfg.is_signed = 1;
	enc_cfg.sample_word_size = sample_word_size;
	channel_mapping = enc_cfg.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		pr_debug("%s: setting default channel map for %d channels",
			 __func__, channels);
		if (q6asm_map_channels(channel_mapping, channels,
					use_back_flavor)) {
			pr_err("%s: map channels failed %d\n",
			       __func__, channels);
			rc = -EINVAL;
			goto fail_cmd;
		}
	} else {
		pr_debug("%s: Using pre-defined channel map", __func__);
		memcpy(channel_mapping, channel_map,
			PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout opcode[0x%x]\n",
		       __func__, enc_cfg.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
		       __func__, adsp_err_get_err_str(
		       atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_pcm_v3);

/**
 * q6asm_enc_cfg_blk_pcm_v2 -
 *       command to set encode config block for pcm_v2
 *
 * @ac: Audio client handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: number of bits per sample
 * @use_default_chmap: Flag indicating to use default ch_map or not
 * @use_back_flavor: back flavor flag
 * @channel_map: Custom channel map settings
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enc_cfg_blk_pcm_v2(struct audio_client *ac,
		uint32_t rate, uint32_t channels, uint16_t bits_per_sample,
		bool use_default_chmap, bool use_back_flavor, u8 *channel_map)
{
	struct asm_multi_channel_pcm_enc_cfg_v2  enc_cfg;
	u8 *channel_mapping;
	u32 frames_per_buf = 0;

	int rc = 0;

	if (!use_default_chmap && (channel_map == NULL)) {
		pr_err("%s: No valid chan map and can't use default\n",
				__func__);
		return -EINVAL;
	}

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: Session %d, rate = %d, channels = %d\n", __func__,
			 ac->session, rate, channels);

	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);
	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(enc_cfg) - sizeof(enc_cfg.hdr) -
				sizeof(enc_cfg.encdec);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size  = enc_cfg.encdec.param_size -
					sizeof(struct asm_enc_cfg_blk_param_v2);

	enc_cfg.num_channels = channels;
	enc_cfg.bits_per_sample = bits_per_sample;
	enc_cfg.sample_rate = rate;
	enc_cfg.is_signed = 1;
	channel_mapping = enc_cfg.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		pr_debug("%s: setting default channel map for %d channels",
		__func__, channels);
		if (q6asm_map_channels(channel_mapping, channels,
					use_back_flavor)) {
			pr_err("%s: map channels failed %d\n",
			 __func__, channels);
			return -EINVAL;
		}
	} else {
		pr_debug("%s: Using pre-defined channel map", __func__);
		memcpy(channel_mapping, channel_map,
			PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout opcode[0x%x]\n",
			__func__, enc_cfg.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_pcm_v2);

static int __q6asm_enc_cfg_blk_pcm_v5(struct audio_client *ac,
				      uint32_t rate, uint32_t channels,
				      bool use_default_chmap,
				      char *channel_map,
				      uint16_t bits_per_sample,
				      uint16_t sample_word_size,
				      uint16_t endianness,
				      uint16_t mode)
{
	return q6asm_enc_cfg_blk_pcm_v5(ac, rate, channels,
					bits_per_sample,
					use_default_chmap, false,
					channel_map,
					sample_word_size, endianness, mode);
}

static int __q6asm_enc_cfg_blk_pcm_v4(struct audio_client *ac,
				      uint32_t rate, uint32_t channels,
				      uint16_t bits_per_sample,
				      uint16_t sample_word_size,
				      uint16_t endianness,
				      uint16_t mode)
{
	return q6asm_enc_cfg_blk_pcm_v4(ac, rate, channels,
					bits_per_sample, true, false, NULL,
					sample_word_size, endianness, mode);
}

static int __q6asm_enc_cfg_blk_pcm_v3(struct audio_client *ac,
				      uint32_t rate, uint32_t channels,
				      uint16_t bits_per_sample,
				      uint16_t sample_word_size)
{
	return q6asm_enc_cfg_blk_pcm_v3(ac, rate, channels,
					bits_per_sample, true, false, NULL,
					sample_word_size);
}

static int __q6asm_enc_cfg_blk_pcm(struct audio_client *ac,
		uint32_t rate, uint32_t channels, uint16_t bits_per_sample)
{
	return q6asm_enc_cfg_blk_pcm_v2(ac, rate, channels,
					bits_per_sample, true, false, NULL);
}

/**
 * q6asm_enc_cfg_blk_pcm -
 *       command to set encode config block for pcm
 *
 * @ac: Audio client handle
 * @rate: sample rate
 * @channels: number of channels
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enc_cfg_blk_pcm(struct audio_client *ac,
			uint32_t rate, uint32_t channels)
{
	return __q6asm_enc_cfg_blk_pcm(ac, rate, channels, 16);
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_pcm);

int q6asm_enc_cfg_blk_pcm_format_support(struct audio_client *ac,
		uint32_t rate, uint32_t channels, uint16_t bits_per_sample)
{
	return __q6asm_enc_cfg_blk_pcm(ac, rate, channels, bits_per_sample);
}

/*
 * q6asm_enc_cfg_blk_pcm_format_support_v3 - sends encoder configuration
 *                                           parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 */
int q6asm_enc_cfg_blk_pcm_format_support_v3(struct audio_client *ac,
					    uint32_t rate, uint32_t channels,
					    uint16_t bits_per_sample,
					    uint16_t sample_word_size)
{
	return __q6asm_enc_cfg_blk_pcm_v3(ac, rate, channels,
					  bits_per_sample, sample_word_size);
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_pcm_format_support_v3);

/*
 * q6asm_enc_cfg_blk_pcm_format_support_v4 - sends encoder configuration
 *                                           parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 * @endianness: endianness of the pcm data
 * @mode: Mode to provide additional info about the pcm input data
 */
int q6asm_enc_cfg_blk_pcm_format_support_v4(struct audio_client *ac,
					    uint32_t rate, uint32_t channels,
					    uint16_t bits_per_sample,
					    uint16_t sample_word_size,
					    uint16_t endianness,
					    uint16_t mode)
{
	return __q6asm_enc_cfg_blk_pcm_v4(ac, rate, channels,
					   bits_per_sample, sample_word_size,
					   endianness, mode);
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_pcm_format_support_v4);

/*
 * q6asm_enc_cfg_blk_pcm_format_support_v5 - sends encoder configuration
 *                                           parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @use_default_chmap: true if default channel map to be used
 * @channel_map: input channel map
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 * @endianness: endianness of the pcm data
 * @mode: Mode to provide additional info about the pcm input data
 */
int q6asm_enc_cfg_blk_pcm_format_support_v5(struct audio_client *ac,
					    uint32_t rate, uint32_t channels,
					    bool use_default_chmap,
					    char *channel_map,
					    uint16_t bits_per_sample,
					    uint16_t sample_word_size,
					    uint16_t endianness,
					    uint16_t mode)
{
	 return __q6asm_enc_cfg_blk_pcm_v5(ac, rate, channels,
					   use_default_chmap,
					   channel_map,
					   bits_per_sample,
					   sample_word_size,
					   endianness,
					   mode);
}

EXPORT_SYMBOL(q6asm_enc_cfg_blk_pcm_format_support_v5);
/**
 * q6asm_enc_cfg_blk_pcm_native -
 *       command to set encode config block for pcm_native
 *
 * @ac: Audio client handle
 * @rate: sample rate
 * @channels: number of channels
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enc_cfg_blk_pcm_native(struct audio_client *ac,
			uint32_t rate, uint32_t channels)
{
	struct asm_multi_channel_pcm_enc_cfg_v2  enc_cfg;
	u8 *channel_mapping;
	u32 frames_per_buf = 0;
	int rc = 0;

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: Session %d, rate = %d, channels = %d\n", __func__,
			 ac->session, rate, channels);

	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);
	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(enc_cfg) - sizeof(enc_cfg.hdr) -
				 sizeof(enc_cfg.encdec);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size  = enc_cfg.encdec.param_size -
				sizeof(struct asm_enc_cfg_blk_param_v2);

	enc_cfg.num_channels = 0;/*channels;*/
	enc_cfg.bits_per_sample = 16;
	enc_cfg.sample_rate = 0;/*rate;*/
	enc_cfg.is_signed = 1;
	channel_mapping = enc_cfg.channel_mapping;


	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (q6asm_map_channels(channel_mapping, channels, false)) {
		pr_err("%s: map channels failed %d\n", __func__, channels);
		return -EINVAL;
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout opcode[0x%x]\n",
			__func__, enc_cfg.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_pcm_native);

/*
 * q6asm_map_channels:
 *     Provide default asm channel mapping for given channel count.
 *
 * @channel_mapping: buffer pointer to write back channel maps.
 * @channels: channel count for which channel map is required.
 * @use_back_flavor: use back channels instead of surround channels.
 * Returns 0 for success, -EINVAL for unsupported channel count.
 */
int q6asm_map_channels(u8 *channel_mapping, uint32_t channels,
		bool use_back_flavor)
{
	u8 *lchannel_mapping;

	lchannel_mapping = channel_mapping;
	pr_debug("%s:  channels passed: %d\n", __func__, channels);
	if (channels == 1)  {
		lchannel_mapping[0] = PCM_CHANNEL_FC;
	} else if (channels == 2) {
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
	} else if (channels == 3) {
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = PCM_CHANNEL_FC;
	} else if (channels == 4) {
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = use_back_flavor ?
			PCM_CHANNEL_LB : PCM_CHANNEL_LS;
		lchannel_mapping[3] = use_back_flavor ?
			PCM_CHANNEL_RB : PCM_CHANNEL_RS;
	} else if (channels == 5) {
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = PCM_CHANNEL_FC;
		lchannel_mapping[3] = use_back_flavor ?
			PCM_CHANNEL_LB : PCM_CHANNEL_LS;
		lchannel_mapping[4] = use_back_flavor ?
			PCM_CHANNEL_RB : PCM_CHANNEL_RS;
	} else if (channels == 6) {
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = PCM_CHANNEL_FC;
		lchannel_mapping[3] = PCM_CHANNEL_LFE;
		lchannel_mapping[4] = use_back_flavor ?
			PCM_CHANNEL_LB : PCM_CHANNEL_LS;
		lchannel_mapping[5] = use_back_flavor ?
			PCM_CHANNEL_RB : PCM_CHANNEL_RS;
	} else if (channels == 7) {
		/*
		 * Configured for 5.1 channel mapping + 1 channel for debug
		 * Can be customized based on DSP.
		 */
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = PCM_CHANNEL_FC;
		lchannel_mapping[3] = PCM_CHANNEL_LFE;
		lchannel_mapping[4] = use_back_flavor ?
			PCM_CHANNEL_LB : PCM_CHANNEL_LS;
		lchannel_mapping[5] = use_back_flavor ?
			PCM_CHANNEL_RB : PCM_CHANNEL_RS;
		lchannel_mapping[6] = PCM_CHANNEL_CS;
	} else if (channels == 8) {
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = PCM_CHANNEL_FC;
		lchannel_mapping[3] = PCM_CHANNEL_LFE;
		lchannel_mapping[4] = PCM_CHANNEL_LB;
		lchannel_mapping[5] = PCM_CHANNEL_RB;
		lchannel_mapping[6] = PCM_CHANNEL_LS;
		lchannel_mapping[7] = PCM_CHANNEL_RS;
	} else if (channels == 10) {
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = PCM_CHANNEL_LFE;
		lchannel_mapping[3] = PCM_CHANNEL_FC;
		lchannel_mapping[4] = PCM_CHANNEL_LB;
		lchannel_mapping[5] = PCM_CHANNEL_RB;
		lchannel_mapping[6] = PCM_CHANNEL_LS;
		lchannel_mapping[7] = PCM_CHANNEL_RS;
		lchannel_mapping[8] = PCM_CHANNEL_TFL;
		lchannel_mapping[9] = PCM_CHANNEL_TFR;
	} else if (channels == 12) {
		/*
		 * Configured for 7.1.4 channel mapping
		 * Todo: Needs to be checked
		 */
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = PCM_CHANNEL_FC;
		lchannel_mapping[3] = PCM_CHANNEL_LFE;
		lchannel_mapping[4] = PCM_CHANNEL_LB;
		lchannel_mapping[5] = PCM_CHANNEL_RB;
		lchannel_mapping[6] = PCM_CHANNEL_LS;
		lchannel_mapping[7] = PCM_CHANNEL_RS;
		lchannel_mapping[8] = PCM_CHANNEL_TFL;
		lchannel_mapping[9] = PCM_CHANNEL_TFR;
		lchannel_mapping[10] = PCM_CHANNEL_TSL;
		lchannel_mapping[11] = PCM_CHANNEL_TSR;
	} else if (channels == 14) {
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = PCM_CHANNEL_LFE;
		lchannel_mapping[3] = PCM_CHANNEL_FC;
		lchannel_mapping[4] = PCM_CHANNEL_LB;
		lchannel_mapping[5] = PCM_CHANNEL_RB;
		lchannel_mapping[6] = PCM_CHANNEL_LS;
		lchannel_mapping[7] = PCM_CHANNEL_RS;
		lchannel_mapping[8] = PCM_CHANNEL_TFL;
		lchannel_mapping[9] = PCM_CHANNEL_TFR;
		lchannel_mapping[10] = PCM_CHANNEL_TSL;
		lchannel_mapping[11] = PCM_CHANNEL_TSR;
		lchannel_mapping[12] = PCM_CHANNEL_FLC;
		lchannel_mapping[13] = PCM_CHANNEL_FRC;
	} else if (channels == 16) {
		/*
		 * Configured for 7.1.8 channel mapping
		 * Todo: Needs to be checked
		 */
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = PCM_CHANNEL_FC;
		lchannel_mapping[3] = PCM_CHANNEL_LFE;
		lchannel_mapping[4] = PCM_CHANNEL_LB;
		lchannel_mapping[5] = PCM_CHANNEL_RB;
		lchannel_mapping[6] = PCM_CHANNEL_LS;
		lchannel_mapping[7] = PCM_CHANNEL_RS;
		lchannel_mapping[8] = PCM_CHANNEL_TFL;
		lchannel_mapping[9] = PCM_CHANNEL_TFR;
		lchannel_mapping[10] = PCM_CHANNEL_TSL;
		lchannel_mapping[11] = PCM_CHANNEL_TSR;
		lchannel_mapping[12] = PCM_CHANNEL_FLC;
		lchannel_mapping[13] = PCM_CHANNEL_FRC;
		lchannel_mapping[14] = PCM_CHANNEL_RLC;
		lchannel_mapping[15] = PCM_CHANNEL_RRC;
	} else if (channels == 32) {
		lchannel_mapping[0] = PCM_CHANNEL_FL;
		lchannel_mapping[1] = PCM_CHANNEL_FR;
		lchannel_mapping[2] = PCM_CHANNEL_LFE;
		lchannel_mapping[3] = PCM_CHANNEL_FC;
		lchannel_mapping[4] = PCM_CHANNEL_LS;
		lchannel_mapping[5] = PCM_CHANNEL_RS;
		lchannel_mapping[6] = PCM_CHANNEL_LB;
		lchannel_mapping[7] = PCM_CHANNEL_RB;
		lchannel_mapping[8] = PCM_CHANNEL_CS;
		lchannel_mapping[9] = PCM_CHANNEL_TS;
		lchannel_mapping[10] = PCM_CHANNEL_CVH;
		lchannel_mapping[11] = PCM_CHANNEL_MS;
		lchannel_mapping[12] = PCM_CHANNEL_FLC;
		lchannel_mapping[13] = PCM_CHANNEL_FRC;
		lchannel_mapping[14] = PCM_CHANNEL_RLC;
		lchannel_mapping[15] = PCM_CHANNEL_RRC;
		lchannel_mapping[16] = PCM_CHANNEL_LFE2;
		lchannel_mapping[17] = PCM_CHANNEL_SL;
		lchannel_mapping[18] = PCM_CHANNEL_SR;
		lchannel_mapping[19] = PCM_CHANNEL_TFL;
		lchannel_mapping[20] = PCM_CHANNEL_TFR;
		lchannel_mapping[21] = PCM_CHANNEL_TC;
		lchannel_mapping[22] = PCM_CHANNEL_TBL;
		lchannel_mapping[23] = PCM_CHANNEL_TBR;
		lchannel_mapping[24] = PCM_CHANNEL_TSL;
		lchannel_mapping[25] = PCM_CHANNEL_TSR;
		lchannel_mapping[26] = PCM_CHANNEL_TBC;
		lchannel_mapping[27] = PCM_CHANNEL_BFC;
		lchannel_mapping[28] = PCM_CHANNEL_BFL;
		lchannel_mapping[29] = PCM_CHANNEL_BFR;
		lchannel_mapping[30] = PCM_CHANNEL_LW;
		lchannel_mapping[31] = PCM_CHANNEL_RW;
	} else {
		pr_err("%s: ERROR.unsupported num_ch = %u\n",
		 __func__, channels);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(q6asm_map_channels);

/**
 * q6asm_enable_sbrps -
 *       command to enable sbrps for ASM
 *
 * @ac: Audio client handle
 * @sbr_ps_enable: flag for sbr_ps enable or disable
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enable_sbrps(struct audio_client *ac,
			uint32_t sbr_ps_enable)
{
	struct asm_aac_sbr_ps_flag_param  sbrps;
	u32 frames_per_buf = 0;

	int rc = 0;

	pr_debug("%s: Session %d\n", __func__, ac->session);

	q6asm_add_hdr(ac, &sbrps.hdr, sizeof(sbrps), TRUE);
	atomic_set(&ac->cmd_state, -1);

	sbrps.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	sbrps.encdec.param_id = ASM_PARAM_ID_AAC_SBR_PS_FLAG;
	sbrps.encdec.param_size = sizeof(struct asm_aac_sbr_ps_flag_param) -
				sizeof(struct asm_stream_cmd_set_encdec_param);
	sbrps.encblk.frames_per_buf = frames_per_buf;
	sbrps.encblk.enc_cfg_blk_size  = sbrps.encdec.param_size -
				sizeof(struct asm_enc_cfg_blk_param_v2);

	sbrps.sbr_ps_flag = sbr_ps_enable;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &sbrps);
	if (rc < 0) {
		pr_err("%s: Command opcode[0x%x]paramid[0x%x] failed %d\n",
				__func__,
				ASM_STREAM_CMD_SET_ENCDEC_PARAM,
				ASM_PARAM_ID_AAC_SBR_PS_FLAG, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout opcode[0x%x] ", __func__, sbrps.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enable_sbrps);

/**
 * q6asm_cfg_dual_mono_aac -
 *       command to set config for dual mono aac
 *
 * @ac: Audio client handle
 * @sce_left: left sce val
 * @sce_right: right sce val
 *
 * Returns 0 on success or error on failure
 */
int q6asm_cfg_dual_mono_aac(struct audio_client *ac,
			uint16_t sce_left, uint16_t sce_right)
{
	struct asm_aac_dual_mono_mapping_param dual_mono;

	int rc = 0;

	pr_debug("%s: Session %d, sce_left = %d, sce_right = %d\n",
			 __func__, ac->session, sce_left, sce_right);

	q6asm_add_hdr(ac, &dual_mono.hdr, sizeof(dual_mono), TRUE);
	atomic_set(&ac->cmd_state, -1);

	dual_mono.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	dual_mono.encdec.param_id = ASM_PARAM_ID_AAC_DUAL_MONO_MAPPING;
	dual_mono.encdec.param_size = sizeof(dual_mono.left_channel_sce) +
				      sizeof(dual_mono.right_channel_sce);
	dual_mono.left_channel_sce = sce_left;
	dual_mono.right_channel_sce = sce_right;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &dual_mono);
	if (rc < 0) {
		pr_err("%s: Command opcode[0x%x]paramid[0x%x] failed %d\n",
				__func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM,
				ASM_PARAM_ID_AAC_DUAL_MONO_MAPPING, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout opcode[0x%x]\n", __func__,
						dual_mono.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_cfg_dual_mono_aac);

/* Support for selecting stereo mixing coefficients for B family not done */
int q6asm_cfg_aac_sel_mix_coef(struct audio_client *ac, uint32_t mix_coeff)
{
	struct asm_aac_stereo_mix_coeff_selection_param_v2 aac_mix_coeff;
	int rc = 0;

	q6asm_add_hdr(ac, &aac_mix_coeff.hdr, sizeof(aac_mix_coeff), TRUE);
	atomic_set(&ac->cmd_state, -1);
	aac_mix_coeff.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	aac_mix_coeff.param_id =
		ASM_PARAM_ID_AAC_STEREO_MIX_COEFF_SELECTION_FLAG_V2;
	aac_mix_coeff.param_size =
		sizeof(struct asm_aac_stereo_mix_coeff_selection_param_v2);
	aac_mix_coeff.aac_stereo_mix_coeff_flag = mix_coeff;
	pr_debug("%s: mix_coeff = %u\n", __func__, mix_coeff);
	rc = apr_send_pkt(ac->apr, (uint32_t *) &aac_mix_coeff);
	if (rc < 0) {
		pr_err("%s: Command opcode[0x%x]paramid[0x%x] failed %d\n",
			__func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM,
			ASM_PARAM_ID_AAC_STEREO_MIX_COEFF_SELECTION_FLAG_V2,
			rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
		(atomic_read(&ac->cmd_state) >= 0),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout opcode[0x%x]\n",
			__func__, aac_mix_coeff.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_cfg_aac_sel_mix_coef);

/**
 * q6asm_enc_cfg_blk_qcelp -
 *       command to set encode config block for QCELP
 *
 * @ac: Audio client handle
 * @frames_per_buf: Number of frames per buffer
 * @min_rate: Minimum Enc rate
 * @max_rate: Maximum Enc rate
 * reduced_rate_level: Reduced rate level
 * @rate_modulation_cmd: rate modulation command
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enc_cfg_blk_qcelp(struct audio_client *ac, uint32_t frames_per_buf,
		uint16_t min_rate, uint16_t max_rate,
		uint16_t reduced_rate_level, uint16_t rate_modulation_cmd)
{
	struct asm_v13k_enc_cfg enc_cfg;
	int rc = 0;

	pr_debug("%s: session[%d]frames[%d]min_rate[0x%4x]max_rate[0x%4x] reduced_rate_level[0x%4x]rate_modulation_cmd[0x%4x]\n",
		 __func__,
		ac->session, frames_per_buf, min_rate, max_rate,
		reduced_rate_level, rate_modulation_cmd);

	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);
	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(struct asm_v13k_enc_cfg) -
				sizeof(struct asm_stream_cmd_set_encdec_param);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size  = enc_cfg.encdec.param_size -
				sizeof(struct asm_enc_cfg_blk_param_v2);

	enc_cfg.min_rate = min_rate;
	enc_cfg.max_rate = max_rate;
	enc_cfg.reduced_rate_cmd = reduced_rate_level;
	enc_cfg.rate_mod_cmd = rate_modulation_cmd;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Comamnd %d failed %d\n",
			__func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for setencdec v13k resp\n",
			__func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_qcelp);

/**
 * q6asm_enc_cfg_blk_evrc -
 *       command to set encode config block for EVRC
 *
 * @ac: Audio client handle
 * @frames_per_buf: Number of frames per buffer
 * @min_rate: Minimum Enc rate
 * @max_rate: Maximum Enc rate
 * @rate_modulation_cmd: rate modulation command
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enc_cfg_blk_evrc(struct audio_client *ac, uint32_t frames_per_buf,
		uint16_t min_rate, uint16_t max_rate,
		uint16_t rate_modulation_cmd)
{
	struct asm_evrc_enc_cfg enc_cfg;
	int rc = 0;

	pr_debug("%s: session[%d]frames[%d]min_rate[0x%4x]max_rate[0x%4x] rate_modulation_cmd[0x%4x]\n",
		 __func__, ac->session,
		frames_per_buf,	min_rate, max_rate, rate_modulation_cmd);

	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);
	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(struct asm_evrc_enc_cfg) -
				sizeof(struct asm_stream_cmd_set_encdec_param);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size  = enc_cfg.encdec.param_size -
				sizeof(struct asm_enc_cfg_blk_param_v2);

	enc_cfg.min_rate = min_rate;
	enc_cfg.max_rate = max_rate;
	enc_cfg.rate_mod_cmd = rate_modulation_cmd;
	enc_cfg.reserved = 0;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Comamnd %d failed %d\n",
			__func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for encdec evrc\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_evrc);

/**
 * q6asm_enc_cfg_blk_amrnb -
 *       command to set encode config block for AMRNB
 *
 * @ac: Audio client handle
 * @frames_per_buf: Number of frames per buffer
 * @band_mode: Band mode used
 * @dtx_enable: DTX en flag
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enc_cfg_blk_amrnb(struct audio_client *ac, uint32_t frames_per_buf,
			uint16_t band_mode, uint16_t dtx_enable)
{
	struct asm_amrnb_enc_cfg enc_cfg;
	int rc = 0;

	pr_debug("%s: session[%d]frames[%d]band_mode[0x%4x]dtx_enable[0x%4x]\n",
		__func__, ac->session, frames_per_buf, band_mode, dtx_enable);

	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);
	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(struct asm_amrnb_enc_cfg) -
				sizeof(struct asm_stream_cmd_set_encdec_param);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size  = enc_cfg.encdec.param_size -
				sizeof(struct asm_enc_cfg_blk_param_v2);

	enc_cfg.enc_mode = band_mode;
	enc_cfg.dtx_mode = dtx_enable;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Comamnd %d failed %d\n",
			__func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for set encdec amrnb\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_amrnb);

/**
 * q6asm_enc_cfg_blk_amrwb -
 *       command to set encode config block for AMRWB
 *
 * @ac: Audio client handle
 * @frames_per_buf: Number of frames per buffer
 * @band_mode: Band mode used
 * @dtx_enable: DTX en flag
 *
 * Returns 0 on success or error on failure
 */
int q6asm_enc_cfg_blk_amrwb(struct audio_client *ac, uint32_t frames_per_buf,
			uint16_t band_mode, uint16_t dtx_enable)
{
	struct asm_amrwb_enc_cfg enc_cfg;
	int rc = 0;

	pr_debug("%s: session[%d]frames[%d]band_mode[0x%4x]dtx_enable[0x%4x]\n",
		__func__, ac->session, frames_per_buf, band_mode, dtx_enable);

	q6asm_add_hdr(ac, &enc_cfg.hdr, sizeof(enc_cfg), TRUE);
	atomic_set(&ac->cmd_state, -1);
	enc_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	enc_cfg.encdec.param_id = ASM_PARAM_ID_ENCDEC_ENC_CFG_BLK_V2;
	enc_cfg.encdec.param_size = sizeof(struct asm_amrwb_enc_cfg) -
				sizeof(struct asm_stream_cmd_set_encdec_param);
	enc_cfg.encblk.frames_per_buf = frames_per_buf;
	enc_cfg.encblk.enc_cfg_blk_size  = enc_cfg.encdec.param_size -
				sizeof(struct asm_enc_cfg_blk_param_v2);

	enc_cfg.enc_mode = band_mode;
	enc_cfg.dtx_mode = dtx_enable;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &enc_cfg);
	if (rc < 0) {
		pr_err("%s: Comamnd %d failed %d\n",
			__func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_enc_cfg_blk_amrwb);


static int __q6asm_media_format_block_pcm(struct audio_client *ac,
				uint32_t rate, uint32_t channels,
				uint16_t bits_per_sample, int stream_id,
				bool use_default_chmap, char *channel_map)
{
	struct asm_multi_channel_pcm_fmt_blk_v2 fmt;
	u8 *channel_mapping;
	int rc = 0;

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]\n", __func__, ac->session, rate,
		channels);

	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);
	/*
	 * Updated the token field with stream/session for compressed playback
	 * Platform driver must know the the stream with which the command is
	 * associated
	 */
	if (ac->io_mode & COMPRESSED_STREAM_IO)
		q6asm_update_token(&fmt.hdr.token,
				   ac->session,
				   stream_id,
				   0, /* Buffer index is NA */
				   0, /* Direction flag is NA */
				   WAIT_CMD);

	pr_debug("%s: token = 0x%x, stream_id  %d, session 0x%x\n",
		  __func__, fmt.hdr.token, stream_id, ac->session);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmt_blk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmt_blk);
	fmt.num_channels = channels;
	fmt.bits_per_sample = bits_per_sample;
	fmt.sample_rate = rate;
	fmt.is_signed = 1;

	channel_mapping = fmt.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		if (q6asm_map_channels(channel_mapping, channels, false)) {
			pr_err("%s: map channels failed %d\n",
				__func__, channels);
			return -EINVAL;
		}
	} else {
		memcpy(channel_mapping, channel_map,
			 PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for format update\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}

static int __q6asm_media_format_block_pcm_v3(struct audio_client *ac,
					     uint32_t rate, uint32_t channels,
					     uint16_t bits_per_sample,
					     int stream_id,
					     bool use_default_chmap,
					     char *channel_map,
					     uint16_t sample_word_size)
{
	struct asm_multi_channel_pcm_fmt_blk_param_v3 fmt;
	u8 *channel_mapping;
	int rc;

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]bps[%d]wordsize[%d]\n", __func__,
		 ac->session, rate, channels,
		 bits_per_sample, sample_word_size);

	memset(&fmt, 0, sizeof(fmt));
	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);
	/*
	 * Updated the token field with stream/session for compressed playback
	 * Platform driver must know the the stream with which the command is
	 * associated
	 */
	if (ac->io_mode & COMPRESSED_STREAM_IO)
		fmt.hdr.token = ((ac->session << 8) & 0xFFFF00) |
				(stream_id & 0xFF);

	pr_debug("%s: token = 0x%x, stream_id  %d, session 0x%x\n",
		 __func__, fmt.hdr.token, stream_id, ac->session);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmt_blk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmt_blk);
	fmt.param.num_channels = channels;
	fmt.param.bits_per_sample = bits_per_sample;
	fmt.param.sample_rate = rate;
	fmt.param.is_signed = 1;
	fmt.param.sample_word_size = sample_word_size;
	channel_mapping = fmt.param.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		if (q6asm_map_channels(channel_mapping, channels, false)) {
			pr_err("%s: map channels failed %d\n",
			       __func__, channels);
			rc = -EINVAL;
			goto fail_cmd;
		}
	} else {
		memcpy(channel_mapping, channel_map,
			 PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for format update\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}

static int __q6asm_media_format_block_pcm_v4(struct audio_client *ac,
					     uint32_t rate, uint32_t channels,
					     uint16_t bits_per_sample,
					     int stream_id,
					     bool use_default_chmap,
					     char *channel_map,
					     uint16_t sample_word_size,
					     uint16_t endianness,
					     uint16_t mode)
{
	struct asm_multi_channel_pcm_fmt_blk_param_v4 fmt;
	u8 *channel_mapping;
	int rc;

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]bps[%d]wordsize[%d]\n", __func__,
		 ac->session, rate, channels,
		 bits_per_sample, sample_word_size);

	memset(&fmt, 0, sizeof(fmt));
	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);
	/*
	 * Updated the token field with stream/session for compressed playback
	 * Platform driver must know the the stream with which the command is
	 * associated
	 */
	if (ac->io_mode & COMPRESSED_STREAM_IO)
		fmt.hdr.token = ((ac->session << 8) & 0xFFFF00) |
				(stream_id & 0xFF);

	pr_debug("%s: token = 0x%x, stream_id  %d, session 0x%x\n",
		 __func__, fmt.hdr.token, stream_id, ac->session);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmt_blk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmt_blk);
	fmt.param.num_channels = channels;
	fmt.param.bits_per_sample = bits_per_sample;
	fmt.param.sample_rate = rate;
	fmt.param.is_signed = 1;
	fmt.param.sample_word_size = sample_word_size;
	fmt.param.endianness = endianness;
	fmt.param.mode = mode;
	channel_mapping = fmt.param.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		if (q6asm_map_channels(channel_mapping, channels, false)) {
			pr_err("%s: map channels failed %d\n",
			       __func__, channels);
			rc = -EINVAL;
			goto fail_cmd;
		}
	} else {
		memcpy(channel_mapping, channel_map,
			 PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for format update\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}


static int __q6asm_media_format_block_pcm_v5(struct audio_client *ac,
					     uint32_t rate, uint32_t channels,
					     uint16_t bits_per_sample,
					     int stream_id,
					     bool use_default_chmap,
					     char *channel_map,
					     uint16_t sample_word_size,
					     uint16_t endianness,
					     uint16_t mode)
{
	struct asm_multi_channel_pcm_fmt_blk_param_v5 fmt;
	u8 *channel_mapping;
	int rc;

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL_V8) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]bps[%d]wordsize[%d]\n", __func__,
		 ac->session, rate, channels,
		 bits_per_sample, sample_word_size);

	memset(&fmt, 0, sizeof(fmt));
	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);
	/*
	 * Updated the token field with stream/session for compressed playback
	 * Platform driver must know the the stream with which the command is
	 * associated
	 */
	if (ac->io_mode & COMPRESSED_STREAM_IO)
		fmt.hdr.token = ((ac->session << 8) & 0xFFFF00) |
				(stream_id & 0xFF);

	pr_debug("%s: token = 0x%x, stream_id  %d, session 0x%x\n",
		 __func__, fmt.hdr.token, stream_id, ac->session);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmt_blk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmt_blk);
	fmt.param.num_channels = (uint16_t) channels & 0xFFFF;
	fmt.param.bits_per_sample = bits_per_sample;
	fmt.param.sample_rate = rate;
	fmt.param.is_signed = 1;
	fmt.param.sample_word_size = sample_word_size;
	fmt.param.endianness = endianness;
	fmt.param.mode = mode;
	channel_mapping = fmt.param.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8);

	if (use_default_chmap) {
		if (q6asm_map_channels(channel_mapping, fmt.param.num_channels, false)) {
			pr_err("%s: map channels failed %d\n",
			       __func__, channels);
			rc = -EINVAL;
			goto fail_cmd;
		}
	} else {
		memcpy(channel_mapping, channel_map,
			 PCM_FORMAT_MAX_NUM_CHANNEL_V8);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0), 5*HZ);
	if (!rc) {
		pr_err("%s: timeout. waited for format update\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}

/**
 * q6asm_media_format_block_pcm -
 *       command to set mediafmt block for PCM on ASM stream
 *
 * @ac: Audio client handle
 * @rate: sample rate
 * @channels: number of ASM channels
 *
 * Returns 0 on success or error on failure
 */
int q6asm_media_format_block_pcm(struct audio_client *ac,
				uint32_t rate, uint32_t channels)
{
	return __q6asm_media_format_block_pcm(ac, rate,
				channels, 16, ac->stream_id,
				true, NULL);
}
EXPORT_SYMBOL(q6asm_media_format_block_pcm);

/**
 * q6asm_media_format_block_pcm_format_support -
 *       command to set mediafmt block for PCM format support
 *
 * @ac: Audio client handle
 * @rate: sample rate
 * @channels: number of ASM channels
 * @bits_per_sample: number of bits per sample
 *
 * Returns 0 on success or error on failure
 */
int q6asm_media_format_block_pcm_format_support(struct audio_client *ac,
				uint32_t rate, uint32_t channels,
				uint16_t bits_per_sample)
{
	return __q6asm_media_format_block_pcm(ac, rate,
				channels, bits_per_sample, ac->stream_id,
				true, NULL);
}
EXPORT_SYMBOL(q6asm_media_format_block_pcm_format_support);

int q6asm_media_format_block_pcm_format_support_v2(struct audio_client *ac,
				uint32_t rate, uint32_t channels,
				uint16_t bits_per_sample, int stream_id,
				bool use_default_chmap, char *channel_map)
{
	if (!use_default_chmap && (channel_map == NULL)) {
		pr_err("%s: No valid chan map and can't use default\n",
			__func__);
		return -EINVAL;
	}
	return __q6asm_media_format_block_pcm(ac, rate,
				channels, bits_per_sample, stream_id,
				use_default_chmap, channel_map);
}

/*
 * q6asm_media_format_block_pcm_format_support_v3- sends pcm decoder
 *						    configuration parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @stream_id: stream id of stream to be associated with this session
 * @use_default_chmap: true if default channel map  to be used
 * @channel_map: input channel map
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 */
int q6asm_media_format_block_pcm_format_support_v3(struct audio_client *ac,
						   uint32_t rate,
						   uint32_t channels,
						   uint16_t bits_per_sample,
						   int stream_id,
						   bool use_default_chmap,
						   char *channel_map,
						   uint16_t sample_word_size)
{
	if (!use_default_chmap && (channel_map == NULL)) {
		pr_err("%s: No valid chan map and can't use default\n",
			__func__);
		return -EINVAL;
	}
	return __q6asm_media_format_block_pcm_v3(ac, rate,
				channels, bits_per_sample, stream_id,
				use_default_chmap, channel_map,
				sample_word_size);

}
EXPORT_SYMBOL(q6asm_media_format_block_pcm_format_support_v3);

/*
 * q6asm_media_format_block_pcm_format_support_v4- sends pcm decoder
 *						    configuration parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @stream_id: stream id of stream to be associated with this session
 * @use_default_chmap: true if default channel map  to be used
 * @channel_map: input channel map
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 * @endianness: endianness of the pcm data
 * @mode: Mode to provide additional info about the pcm input data
 */
int q6asm_media_format_block_pcm_format_support_v4(struct audio_client *ac,
						   uint32_t rate,
						   uint32_t channels,
						   uint16_t bits_per_sample,
						   int stream_id,
						   bool use_default_chmap,
						   char *channel_map,
						   uint16_t sample_word_size,
						   uint16_t endianness,
						   uint16_t mode)
{
	if (!use_default_chmap && (channel_map == NULL)) {
		pr_err("%s: No valid chan map and can't use default\n",
			__func__);
		return -EINVAL;
	}
	return __q6asm_media_format_block_pcm_v4(ac, rate,
				channels, bits_per_sample, stream_id,
				use_default_chmap, channel_map,
				sample_word_size, endianness,
				mode);

}
EXPORT_SYMBOL(q6asm_media_format_block_pcm_format_support_v4);


/*
 * q6asm_media_format_block_pcm_format_support_v5- sends pcm decoder
 *						    configuration parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @stream_id: stream id of stream to be associated with this session
 * @use_default_chmap: true if default channel map  to be used
 * @channel_map: input channel map
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 * @endianness: endianness of the pcm data
 * @mode: Mode to provide additional info about the pcm input data
 */
int q6asm_media_format_block_pcm_format_support_v5(struct audio_client *ac,
						   uint32_t rate,
						   uint32_t channels,
						   uint16_t bits_per_sample,
						   int stream_id,
						   bool use_default_chmap,
						   char *channel_map,
						   uint16_t sample_word_size,
						   uint16_t endianness,
						   uint16_t mode)
{
	if (!use_default_chmap && (channel_map == NULL)) {
		pr_err("%s: No valid chan map and can't use default\n",
			__func__);
		return -EINVAL;
	}
	return __q6asm_media_format_block_pcm_v5(ac, rate,
				channels, bits_per_sample, stream_id,
				use_default_chmap, channel_map,
				sample_word_size, endianness,
				mode);

}
EXPORT_SYMBOL(q6asm_media_format_block_pcm_format_support_v5);


static int __q6asm_media_format_block_multi_ch_pcm(struct audio_client *ac,
				uint32_t rate, uint32_t channels,
				bool use_default_chmap, char *channel_map,
				uint16_t bits_per_sample)
{
	struct asm_multi_channel_pcm_fmt_blk_v2 fmt;
	u8 *channel_mapping;
	int rc = 0;

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]\n", __func__, ac->session, rate,
		channels);

	q6asm_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmt_blk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmt_blk);
	fmt.num_channels = channels;
	fmt.bits_per_sample = bits_per_sample;
	fmt.sample_rate = rate;
	fmt.is_signed = 1;

	channel_mapping = fmt.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		if (q6asm_map_channels(channel_mapping, channels, false)) {
			pr_err("%s: map channels failed %d\n",
				__func__, channels);
			return -EINVAL;
		}
	} else {
		memcpy(channel_mapping, channel_map,
			 PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for format update\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}

static int __q6asm_media_format_block_multi_ch_pcm_v3(struct audio_client *ac,
						      uint32_t rate,
						      uint32_t channels,
						      bool use_default_chmap,
						      char *channel_map,
						      uint16_t bits_per_sample,
						      uint16_t sample_word_size)
{
	struct asm_multi_channel_pcm_fmt_blk_param_v3 fmt;
	u8 *channel_mapping;
	int rc;

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]bps[%d]wordsize[%d]\n", __func__,
		 ac->session, rate, channels,
		 bits_per_sample, sample_word_size);

	memset(&fmt, 0, sizeof(fmt));
	q6asm_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmt_blk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmt_blk);
	fmt.param.num_channels = channels;
	fmt.param.bits_per_sample = bits_per_sample;
	fmt.param.sample_rate = rate;
	fmt.param.is_signed = 1;
	fmt.param.sample_word_size = sample_word_size;
	channel_mapping = fmt.param.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		if (q6asm_map_channels(channel_mapping, channels, false)) {
			pr_err("%s: map channels failed %d\n",
			       __func__, channels);
			rc = -EINVAL;
			goto fail_cmd;
		}
	} else {
		memcpy(channel_mapping, channel_map,
			 PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for format update\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
		       __func__, adsp_err_get_err_str(
		       atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}

static int __q6asm_media_format_block_multi_ch_pcm_v4(struct audio_client *ac,
						      uint32_t rate,
						      uint32_t channels,
						      bool use_default_chmap,
						      char *channel_map,
						      uint16_t bits_per_sample,
						      uint16_t sample_word_size,
						      uint16_t endianness,
						      uint16_t mode)
{
	struct asm_multi_channel_pcm_fmt_blk_param_v4 fmt;
	u8 *channel_mapping;
	int rc;

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]bps[%d]wordsize[%d]\n", __func__,
		 ac->session, rate, channels,
		 bits_per_sample, sample_word_size);

	memset(&fmt, 0, sizeof(fmt));
	q6asm_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmt_blk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmt_blk);
	fmt.param.num_channels = channels;
	fmt.param.bits_per_sample = bits_per_sample;
	fmt.param.sample_rate = rate;
	fmt.param.is_signed = 1;
	fmt.param.sample_word_size = sample_word_size;
	fmt.param.endianness = endianness;
	fmt.param.mode = mode;
	channel_mapping = fmt.param.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		if (q6asm_map_channels(channel_mapping, channels, false)) {
			pr_err("%s: map channels failed %d\n",
			       __func__, channels);
			rc = -EINVAL;
			goto fail_cmd;
		}
	} else {
		memcpy(channel_mapping, channel_map,
			 PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for format update\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
		       __func__, adsp_err_get_err_str(
		       atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}

static int __q6asm_media_format_block_multi_ch_pcm_v5(struct audio_client *ac,
						      uint32_t rate,
						      uint32_t channels,
						      bool use_default_chmap,
						      char *channel_map,
						      uint16_t bits_per_sample,
						      uint16_t sample_word_size,
						      uint16_t endianness,
						      uint16_t mode)
{
	struct asm_multi_channel_pcm_fmt_blk_param_v5 fmt;
	u8 *channel_mapping;
	int rc;

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL_V8) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]bps[%d]wordsize[%d]\n", __func__,
		 ac->session, rate, channels,
		 bits_per_sample, sample_word_size);

	memset(&fmt, 0, sizeof(fmt));
	q6asm_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmt_blk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmt_blk);
	fmt.param.num_channels = channels;
	fmt.param.bits_per_sample = bits_per_sample;
	fmt.param.sample_rate = rate;
	fmt.param.is_signed = 1;
	fmt.param.sample_word_size = sample_word_size;
	fmt.param.endianness = endianness;
	fmt.param.mode = mode;
	channel_mapping = fmt.param.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL_V8);

	if (use_default_chmap) {
		if (q6asm_map_channels(channel_mapping, channels, false)) {
			pr_err("%s: map channels failed %d\n",
			       __func__, channels);
			rc = -EINVAL;
			goto fail_cmd;
		}
	} else {
		memcpy(channel_mapping, channel_map,
			 PCM_FORMAT_MAX_NUM_CHANNEL_V8);
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0), 5*HZ);
	if (!rc) {
		pr_err("%s: timeout. waited for format update\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
		       __func__, adsp_err_get_err_str(
		       atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}

int q6asm_media_format_block_multi_ch_pcm(struct audio_client *ac,
		uint32_t rate, uint32_t channels,
		bool use_default_chmap, char *channel_map)
{
	return __q6asm_media_format_block_multi_ch_pcm(ac, rate,
			channels, use_default_chmap, channel_map, 16);
}

int q6asm_media_format_block_multi_ch_pcm_v2(
		struct audio_client *ac,
		uint32_t rate, uint32_t channels,
		bool use_default_chmap, char *channel_map,
		uint16_t bits_per_sample)
{
	return __q6asm_media_format_block_multi_ch_pcm(ac, rate,
			channels, use_default_chmap, channel_map,
			bits_per_sample);
}

/*
 * q6asm_media_format_block_multi_ch_pcm_v3 - sends pcm decoder configuration
 *                                            parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @use_default_chmap: true if default channel map  to be used
 * @channel_map: input channel map
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 */
int q6asm_media_format_block_multi_ch_pcm_v3(struct audio_client *ac,
					     uint32_t rate, uint32_t channels,
					     bool use_default_chmap,
					     char *channel_map,
					     uint16_t bits_per_sample,
					     uint16_t sample_word_size)
{
	return __q6asm_media_format_block_multi_ch_pcm_v3(ac, rate, channels,
							  use_default_chmap,
							  channel_map,
							  bits_per_sample,
							  sample_word_size);
}
EXPORT_SYMBOL(q6asm_media_format_block_multi_ch_pcm_v3);

/*
 * q6asm_media_format_block_multi_ch_pcm_v4 - sends pcm decoder configuration
 *                                            parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @use_default_chmap: true if default channel map  to be used
 * @channel_map: input channel map
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 * @endianness: endianness of the pcm data
 * @mode: Mode to provide additional info about the pcm input data
 */
int q6asm_media_format_block_multi_ch_pcm_v4(struct audio_client *ac,
					     uint32_t rate, uint32_t channels,
					     bool use_default_chmap,
					     char *channel_map,
					     uint16_t bits_per_sample,
					     uint16_t sample_word_size,
					     uint16_t endianness,
					     uint16_t mode)
{
	return __q6asm_media_format_block_multi_ch_pcm_v4(ac, rate, channels,
							  use_default_chmap,
							  channel_map,
							  bits_per_sample,
							  sample_word_size,
							  endianness,
							  mode);
}
EXPORT_SYMBOL(q6asm_media_format_block_multi_ch_pcm_v4);


/*
 * q6asm_media_format_block_multi_ch_pcm_v5 - sends pcm decoder configuration
 *                                            parameters
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @bits_per_sample: bit width of encoder session
 * @use_default_chmap: true if default channel map  to be used
 * @channel_map: input channel map
 * @sample_word_size: Size in bits of the word that holds a sample of a channel
 * @endianness: endianness of the pcm data
 * @mode: Mode to provide additional info about the pcm input data
 */
int q6asm_media_format_block_multi_ch_pcm_v5(struct audio_client *ac,
					     uint32_t rate, uint32_t channels,
					     bool use_default_chmap,
					     char *channel_map,
					     uint16_t bits_per_sample,
					     uint16_t sample_word_size,
					     uint16_t endianness,
					     uint16_t mode)
{
	return __q6asm_media_format_block_multi_ch_pcm_v5(ac, rate, channels,
							  use_default_chmap,
							  channel_map,
							  bits_per_sample,
							  sample_word_size,
							  endianness,
							  mode);
}
EXPORT_SYMBOL(q6asm_media_format_block_multi_ch_pcm_v5);


/*
 * q6asm_media_format_block_gen_compr - set up generic compress format params
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 * @use_default_chmap: true if default channel map to be used
 * @channel_map: input channel map
 * @bits_per_sample: bit width of gen compress stream
 */
int q6asm_media_format_block_gen_compr(struct audio_client *ac,
				uint32_t rate, uint32_t channels,
				bool use_default_chmap, char *channel_map,
				uint16_t bits_per_sample)
{
	struct asm_generic_compressed_fmt_blk_t fmt;
	u8 *channel_mapping;
	int rc = 0;

	if (channels > PCM_FORMAT_MAX_NUM_CHANNEL) {
		pr_err("%s: Invalid channel count %d\n", __func__, channels);
		return -EINVAL;
	}

	pr_debug("%s: session[%d]rate[%d]ch[%d]bps[%d]\n",
		 __func__, ac->session, rate,
		 channels, bits_per_sample);

	memset(&fmt, 0, sizeof(fmt));
	q6asm_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmt_blk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmt_blk);
	fmt.num_channels = channels;
	fmt.bits_per_sample = bits_per_sample;
	fmt.sampling_rate = rate;

	channel_mapping = fmt.channel_mapping;

	memset(channel_mapping, 0, PCM_FORMAT_MAX_NUM_CHANNEL);

	if (use_default_chmap) {
		if (q6asm_map_channels(channel_mapping, channels, false)) {
			pr_err("%s: map channels failed %d\n",
				__func__, channels);
			return -EINVAL;
		}
	} else {
		memcpy(channel_mapping, channel_map,
		       PCM_FORMAT_MAX_NUM_CHANNEL);
	}

	atomic_set(&ac->cmd_state, -1);
	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for format update\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_media_format_block_gen_compr);


/*
 * q6asm_media_format_block_iec - set up IEC61937 (compressed) or IEC60958
 *                                (pcm) format params. Both audio standards
 *                                use the same format and are used for
 *                                HDMI or SPDIF.
 *
 * @ac: Client session handle
 * @rate: sample rate
 * @channels: number of channels
 */
int q6asm_media_format_block_iec(struct audio_client *ac,
				uint32_t rate, uint32_t channels)
{
	struct asm_iec_compressed_fmt_blk_t fmt;
	int rc = 0;

	pr_debug("%s: session[%d]rate[%d]ch[%d]\n",
		 __func__, ac->session, rate,
		 channels);

	memset(&fmt, 0, sizeof(fmt));
	q6asm_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE);

	fmt.hdr.opcode = ASM_DATA_CMD_IEC_60958_MEDIA_FMT;
	fmt.num_channels = channels;
	fmt.sampling_rate = rate;

	atomic_set(&ac->cmd_state, -1);
	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for format update\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
			__func__, adsp_err_get_err_str(
			atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_media_format_block_iec);

static int __q6asm_media_format_block_multi_aac(struct audio_client *ac,
				struct asm_aac_cfg *cfg, int stream_id)
{
	struct asm_aac_fmt_blk_v2 fmt;
	int rc = 0;

	pr_debug("%s: session[%d]rate[%d]ch[%d]\n", __func__, ac->session,
		cfg->sample_rate, cfg->ch_cfg);

	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);
	/*
	 * Updated the token field with stream/session for compressed playback
	 * Platform driver must know the the stream with which the command is
	 * associated
	 */
	if (ac->io_mode & COMPRESSED_STREAM_IO)
		q6asm_update_token(&fmt.hdr.token,
				   ac->session,
				   stream_id,
				   0, /* Buffer index is NA */
				   0, /* Direction flag is NA */
				   WAIT_CMD);

	pr_debug("%s: token = 0x%x, stream_id  %d, session 0x%x\n",
		  __func__, fmt.hdr.token, stream_id, ac->session);
	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmt_blk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmt_blk);
	fmt.aac_fmt_flag = cfg->format;
	fmt.audio_objype = cfg->aot;
	/* If zero, PCE is assumed to be available in bitstream*/
	fmt.total_size_of_PCE_bits = 0;
	fmt.channel_config = cfg->ch_cfg;
	fmt.sample_rate = cfg->sample_rate;

	pr_debug("%s: format=0x%x cfg_size=%d aac-cfg=0x%x aot=%d ch=%d sr=%d\n",
			__func__, fmt.aac_fmt_flag, fmt.fmt_blk.fmt_blk_size,
			fmt.aac_fmt_flag,
			fmt.audio_objype,
			fmt.channel_config,
			fmt.sample_rate);
	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}

/**
 * q6asm_media_format_block_multi_aac -
 *       command to set mediafmt block for multi_aac on ASM stream
 *
 * @ac: Audio client handle
 * @cfg: multi_aac config
 *
 * Returns 0 on success or error on failure
 */
int q6asm_media_format_block_multi_aac(struct audio_client *ac,
				struct asm_aac_cfg *cfg)
{
	return __q6asm_media_format_block_multi_aac(ac, cfg, ac->stream_id);
}
EXPORT_SYMBOL(q6asm_media_format_block_multi_aac);

/**
 * q6asm_media_format_block_aac -
 *       command to set mediafmt block for aac on ASM
 *
 * @ac: Audio client handle
 * @cfg: aac config
 *
 * Returns 0 on success or error on failure
 */
int q6asm_media_format_block_aac(struct audio_client *ac,
			struct asm_aac_cfg *cfg)
{
	return __q6asm_media_format_block_multi_aac(ac, cfg, ac->stream_id);
}
EXPORT_SYMBOL(q6asm_media_format_block_aac);

/**
 * q6asm_stream_media_format_block_aac -
 *       command to set mediafmt block for aac on ASM stream
 *
 * @ac: Audio client handle
 * @cfg: aac config
 * @stream_id: stream ID info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_stream_media_format_block_aac(struct audio_client *ac,
			struct asm_aac_cfg *cfg, int stream_id)
{
	return __q6asm_media_format_block_multi_aac(ac, cfg, stream_id);
}
EXPORT_SYMBOL(q6asm_stream_media_format_block_aac);

/**
 * q6asm_media_format_block_wma -
 *       command to set mediafmt block for wma on ASM stream
 *
 * @ac: Audio client handle
 * @cfg: wma config
 * @stream_id: stream ID info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_media_format_block_wma(struct audio_client *ac,
				void *cfg, int stream_id)
{
	struct asm_wmastdv9_fmt_blk_v2 fmt;
	struct asm_wma_cfg *wma_cfg = (struct asm_wma_cfg *)cfg;
	int rc = 0;

	pr_debug("session[%d]format_tag[0x%4x] rate[%d] ch[0x%4x] bps[%d], balign[0x%4x], bit_sample[0x%4x], ch_msk[%d], enc_opt[0x%4x]\n",
		ac->session, wma_cfg->format_tag, wma_cfg->sample_rate,
		wma_cfg->ch_cfg, wma_cfg->avg_bytes_per_sec,
		wma_cfg->block_align, wma_cfg->valid_bits_per_sample,
		wma_cfg->ch_mask, wma_cfg->encode_opt);

	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmtblk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmtblk);
	fmt.fmtag = wma_cfg->format_tag;
	fmt.num_channels = wma_cfg->ch_cfg;
	fmt.sample_rate = wma_cfg->sample_rate;
	fmt.avg_bytes_per_sec = wma_cfg->avg_bytes_per_sec;
	fmt.blk_align = wma_cfg->block_align;
	fmt.bits_per_sample =
			wma_cfg->valid_bits_per_sample;
	fmt.channel_mask = wma_cfg->ch_mask;
	fmt.enc_options = wma_cfg->encode_opt;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_media_format_block_wma);

/**
 * q6asm_media_format_block_wmapro -
 *       command to set mediafmt block for wmapro on ASM stream
 *
 * @ac: Audio client handle
 * @cfg: wmapro config
 * @stream_id: stream ID info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_media_format_block_wmapro(struct audio_client *ac,
				void *cfg, int stream_id)
{
	struct asm_wmaprov10_fmt_blk_v2 fmt;
	struct asm_wmapro_cfg *wmapro_cfg = (struct asm_wmapro_cfg *)cfg;
	int rc = 0;

	pr_debug("%s: session[%d]format_tag[0x%4x] rate[%d] ch[0x%4x] bps[%d], balign[0x%4x], bit_sample[0x%4x], ch_msk[%d], enc_opt[0x%4x], adv_enc_opt[0x%4x], adv_enc_opt2[0x%8x]\n",
		__func__,
		ac->session, wmapro_cfg->format_tag, wmapro_cfg->sample_rate,
		wmapro_cfg->ch_cfg,  wmapro_cfg->avg_bytes_per_sec,
		wmapro_cfg->block_align, wmapro_cfg->valid_bits_per_sample,
		wmapro_cfg->ch_mask, wmapro_cfg->encode_opt,
		wmapro_cfg->adv_encode_opt, wmapro_cfg->adv_encode_opt2);

	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmtblk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
						sizeof(fmt.fmtblk);

	fmt.fmtag = wmapro_cfg->format_tag;
	fmt.num_channels = wmapro_cfg->ch_cfg;
	fmt.sample_rate = wmapro_cfg->sample_rate;
	fmt.avg_bytes_per_sec =
				wmapro_cfg->avg_bytes_per_sec;
	fmt.blk_align = wmapro_cfg->block_align;
	fmt.bits_per_sample = wmapro_cfg->valid_bits_per_sample;
	fmt.channel_mask = wmapro_cfg->ch_mask;
	fmt.enc_options = wmapro_cfg->encode_opt;
	fmt.usAdvancedEncodeOpt = wmapro_cfg->adv_encode_opt;
	fmt.advanced_enc_options2 = wmapro_cfg->adv_encode_opt2;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd open failed %d\n", __func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_media_format_block_wmapro);

/**
 * q6asm_media_format_block_amrwbplus -
 *       command to set mediafmt block for amrwbplus on ASM stream
 *
 * @ac: Audio client handle
 * @cfg: amrwbplus config
 * @stream_id: stream ID info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_media_format_block_amrwbplus(struct audio_client *ac,
				struct asm_amrwbplus_cfg *cfg)
{
	struct asm_amrwbplus_fmt_blk_v2 fmt;
	int rc = 0;

	pr_debug("%s: session[%d]band-mode[%d]frame-fmt[%d]ch[%d]\n",
		__func__,
		ac->session,
		cfg->amr_band_mode,
		cfg->amr_frame_fmt,
		cfg->num_channels);

	q6asm_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmtblk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmtblk);
	fmt.amr_frame_fmt = cfg->amr_frame_fmt;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Comamnd media format update failed.. %d\n",
			__func__, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_media_format_block_amrwbplus);

/**
 * q6asm_stream_media_format_block_flac -
 *       command to set mediafmt block for flac on ASM stream
 *
 * @ac: Audio client handle
 * @cfg: FLAC config
 * @stream_id: stream ID info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_stream_media_format_block_flac(struct audio_client *ac,
				struct asm_flac_cfg *cfg, int stream_id)
{
	struct asm_flac_fmt_blk_v2 fmt;
	int rc = 0;

	pr_debug("%s :session[%d] rate[%d] ch[%d] size[%d] stream_id[%d]\n",
		__func__, ac->session, cfg->sample_rate, cfg->ch_cfg,
		cfg->sample_size, stream_id);

	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmtblk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
						sizeof(fmt.fmtblk);

	fmt.is_stream_info_present = cfg->stream_info_present;
	fmt.num_channels = cfg->ch_cfg;
	fmt.min_blk_size = cfg->min_blk_size;
	fmt.max_blk_size = cfg->max_blk_size;
	fmt.sample_rate = cfg->sample_rate;
	fmt.min_frame_size = cfg->min_frame_size;
	fmt.max_frame_size = cfg->max_frame_size;
	fmt.sample_size = cfg->sample_size;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s :Comamnd media format update failed %d\n",
				__func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s :timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_stream_media_format_block_flac);

/**
 * q6asm_media_format_block_alac -
 *       command to set mediafmt block for alac on ASM stream
 *
 * @ac: Audio client handle
 * @cfg: ALAC config
 * @stream_id: stream ID info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_media_format_block_alac(struct audio_client *ac,
				struct asm_alac_cfg *cfg, int stream_id)
{
	struct asm_alac_fmt_blk_v2 fmt;
	int rc = 0;

	pr_debug("%s :session[%d]rate[%d]ch[%d]\n", __func__,
		ac->session, cfg->sample_rate, cfg->num_channels);

	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmtblk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
						sizeof(fmt.fmtblk);

	fmt.frame_length = cfg->frame_length;
	fmt.compatible_version = cfg->compatible_version;
	fmt.bit_depth = cfg->bit_depth;
	fmt.pb = cfg->pb;
	fmt.mb = cfg->mb;
	fmt.kb = cfg->kb;
	fmt.num_channels = cfg->num_channels;
	fmt.max_run = cfg->max_run;
	fmt.max_frame_bytes = cfg->max_frame_bytes;
	fmt.avg_bit_rate = cfg->avg_bit_rate;
	fmt.sample_rate = cfg->sample_rate;
	fmt.channel_layout_tag = cfg->channel_layout_tag;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s :Comamnd media format update failed %d\n",
				__func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s :timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_media_format_block_alac);

/*
 * q6asm_media_format_block_g711 - sends g711 decoder configuration
 *                                            parameters
 * @ac: Client session handle
 * @cfg: Audio stream manager configuration parameters
 * @stream_id: Stream id
 */
int q6asm_media_format_block_g711(struct audio_client *ac,
				struct asm_g711_dec_cfg *cfg, int stream_id)
{
	struct asm_g711_dec_fmt_blk_v2 fmt;
	int rc = 0;

	if (!ac) {
		pr_err("%s: audio client is null\n", __func__);
		return -EINVAL;
	}
	if (!cfg) {
		pr_err("%s: Invalid ASM config\n", __func__);
		return -EINVAL;
	}

	if (stream_id <= 0) {
		pr_err("%s: Invalid stream id\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s :session[%d]rate[%d]\n", __func__,
		ac->session, cfg->sample_rate);

	memset(&fmt, 0, sizeof(struct asm_g711_dec_fmt_blk_v2));

	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmtblk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
						sizeof(fmt.fmtblk);

	fmt.sample_rate = cfg->sample_rate;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s :Command media format update failed %d\n",
				__func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s :timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_media_format_block_g711);

/**
 * q6asm_stream_media_format_block_vorbis -
 *       command to set mediafmt block for vorbis on ASM stream
 *
 * @ac: Audio client handle
 * @cfg: vorbis config
 * @stream_id: stream ID info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_stream_media_format_block_vorbis(struct audio_client *ac,
				struct asm_vorbis_cfg *cfg, int stream_id)
{
	struct asm_vorbis_fmt_blk_v2 fmt;
	int rc = 0;

	pr_debug("%s :session[%d] bit_stream_fmt[%d] stream_id[%d]\n",
		__func__, ac->session, cfg->bit_stream_fmt, stream_id);

	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmtblk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
						sizeof(fmt.fmtblk);

	fmt.bit_stream_fmt = cfg->bit_stream_fmt;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s :Comamnd media format update failed %d\n",
				__func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s :timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_stream_media_format_block_vorbis);

/**
 * q6asm_media_format_block_ape -
 *       command to set mediafmt block for APE on ASM stream
 *
 * @ac: Audio client handle
 * @cfg: APE config
 * @stream_id: stream ID info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_media_format_block_ape(struct audio_client *ac,
				struct asm_ape_cfg *cfg, int stream_id)
{
	struct asm_ape_fmt_blk_v2 fmt;
	int rc = 0;

	pr_debug("%s :session[%d]rate[%d]ch[%d]\n", __func__,
			ac->session, cfg->sample_rate, cfg->num_channels);

	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmtblk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
		sizeof(fmt.fmtblk);

	fmt.compatible_version = cfg->compatible_version;
	fmt.compression_level = cfg->compression_level;
	fmt.format_flags = cfg->format_flags;
	fmt.blocks_per_frame = cfg->blocks_per_frame;
	fmt.final_frame_blocks = cfg->final_frame_blocks;
	fmt.total_frames = cfg->total_frames;
	fmt.bits_per_sample = cfg->bits_per_sample;
	fmt.num_channels = cfg->num_channels;
	fmt.sample_rate = cfg->sample_rate;
	fmt.seek_table_present = cfg->seek_table_present;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s :Comamnd media format update failed %d\n",
				__func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s :timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_media_format_block_ape);

/*
 * q6asm_media_format_block_dsd- Sends DSD Decoder
 * configuration parameters
 *
 * @ac: Client session handle
 * @cfg: DSD Media Format Configuration.
 * @stream_id: stream id of stream to be associated with this session
 *
 * Return 0 on success or negative error code on failure
 */
int q6asm_media_format_block_dsd(struct audio_client *ac,
				struct asm_dsd_cfg *cfg, int stream_id)
{
	struct asm_dsd_fmt_blk_v2 fmt;
	int rc;

	pr_debug("%s: session[%d] data_rate[%d] ch[%d]\n", __func__,
		 ac->session, cfg->dsd_data_rate, cfg->num_channels);

	memset(&fmt, 0, sizeof(fmt));
	q6asm_stream_add_hdr(ac, &fmt.hdr, sizeof(fmt), TRUE, stream_id);

	fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	fmt.fmtblk.fmt_blk_size = sizeof(fmt) - sizeof(fmt.hdr) -
					sizeof(fmt.fmtblk);

	fmt.num_version = cfg->num_version;
	fmt.is_bitwise_big_endian = cfg->is_bitwise_big_endian;
	fmt.dsd_channel_block_size = cfg->dsd_channel_block_size;
	fmt.num_channels = cfg->num_channels;
	fmt.dsd_data_rate = cfg->dsd_data_rate;
	atomic_set(&ac->cmd_state, -1);
	rc = apr_send_pkt(ac->apr, (uint32_t *) &fmt);
	if (rc < 0) {
		pr_err("%s: Command DSD media format update failed, err: %d\n",
			__func__, rc);
		goto done;
	}
	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for DSD FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto done;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto done;
	}
	return 0;
done:
	return rc;
}
EXPORT_SYMBOL(q6asm_media_format_block_dsd);

/**
 * q6asm_stream_media_format_block_aptx_dec -
 *       command to set mediafmt block for APTX dec on ASM stream
 *
 * @ac: Audio client handle
 * @srate: sample rate
 * @stream_id: stream ID info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_stream_media_format_block_aptx_dec(struct audio_client *ac,
						uint32_t srate, int stream_id)
{
	struct asm_aptx_dec_fmt_blk_v2 aptx_fmt;
	int rc = 0;

	if (!ac->session) {
		pr_err("%s: ac session invalid\n", __func__);
		rc = -EINVAL;
		goto fail_cmd;
	}
	pr_debug("%s :session[%d] rate[%d] stream_id[%d]\n",
		__func__, ac->session, srate, stream_id);

	q6asm_stream_add_hdr(ac, &aptx_fmt.hdr, sizeof(aptx_fmt), TRUE,
				stream_id);
	atomic_set(&ac->cmd_state, -1);

	aptx_fmt.hdr.opcode = ASM_DATA_CMD_MEDIA_FMT_UPDATE_V2;
	aptx_fmt.fmtblk.fmt_blk_size = sizeof(aptx_fmt) - sizeof(aptx_fmt.hdr) -
						sizeof(aptx_fmt.fmtblk);

	aptx_fmt.sample_rate = srate;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &aptx_fmt);
	if (rc < 0) {
		pr_err("%s :Comamnd media format update failed %d\n",
				__func__, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s :timeout. waited for FORMAT_UPDATE\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	rc = 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_stream_media_format_block_aptx_dec);

static int __q6asm_ds1_set_endp_params(struct audio_client *ac, int param_id,
				int param_value, int stream_id)
{
	struct asm_dec_ddp_endp_param_v2 ddp_cfg;
	int rc = 0;

	pr_debug("%s: session[%d] stream[%d],param_id[%d]param_value[%d]",
		 __func__, ac->session, stream_id, param_id, param_value);

	q6asm_stream_add_hdr(ac, &ddp_cfg.hdr, sizeof(ddp_cfg), TRUE,
			     stream_id);
	atomic_set(&ac->cmd_state, -1);
	/*
	 * Updated the token field with stream/session for compressed playback
	 * Platform driver must know the stream with which the command is
	 * associated
	 */
	if (ac->io_mode & COMPRESSED_STREAM_IO)
		q6asm_update_token(&ddp_cfg.hdr.token,
				   ac->session,
				   stream_id,
				   0, /* Buffer index is NA */
				   0, /* Direction flag is NA */
				   WAIT_CMD);
	ddp_cfg.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	ddp_cfg.encdec.param_id = param_id;
	ddp_cfg.encdec.param_size = sizeof(struct asm_dec_ddp_endp_param_v2) -
				(sizeof(struct apr_hdr) +
				sizeof(struct asm_stream_cmd_set_encdec_param));
	ddp_cfg.endp_param_value = param_value;
	rc = apr_send_pkt(ac->apr, (uint32_t *) &ddp_cfg);
	if (rc < 0) {
		pr_err("%s: Command opcode[0x%x] failed %d\n",
			__func__, ASM_STREAM_CMD_SET_ENCDEC_PARAM, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
		(atomic_read(&ac->cmd_state) >= 0),
		msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout opcode[0x%x]\n", __func__,
			ddp_cfg.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return rc;
}

/**
 * q6asm_ds1_set_endp_params -
 *       command to set DS1 params for ASM
 *
 * @ac: Audio client handle
 * @param_id: param id
 * @param_value: value of param
 *
 * Returns 0 on success or error on failure
 */
int q6asm_ds1_set_endp_params(struct audio_client *ac,
			      int param_id, int param_value)
{
	return __q6asm_ds1_set_endp_params(ac, param_id, param_value,
					   ac->stream_id);
}

/**
 * q6asm_ds1_set_stream_endp_params -
 *       command to set DS1 params for ASM stream
 *
 * @ac: Audio client handle
 * @param_id: param id
 * @param_value: value of param
 * @stream_id: stream ID info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_ds1_set_stream_endp_params(struct audio_client *ac,
				     int param_id, int param_value,
				     int stream_id)
{
	return __q6asm_ds1_set_endp_params(ac, param_id, param_value,
					   stream_id);
}
EXPORT_SYMBOL(q6asm_ds1_set_stream_endp_params);

/**
 * q6asm_memory_map -
 *       command to send memory map for ASM
 *
 * @ac: Audio client handle
 * @buf_add: buffer address to map
 * @dir: RX or TX session
 * @bufsz: size of each buffer
 * @bufcnt: buffer count
 *
 * Returns 0 on success or error on failure
 */
int q6asm_memory_map(struct audio_client *ac, phys_addr_t buf_add, int dir,
				uint32_t bufsz, uint32_t bufcnt)
{
	struct avs_cmd_shared_mem_map_regions *mmap_regions = NULL;
	struct avs_shared_map_region_payload  *mregions = NULL;
	struct audio_port_data *port = NULL;
	void	*mmap_region_cmd = NULL;
	void	*payload = NULL;
	struct asm_buffer_node *buffer_node = NULL;
	int	rc = 0;
	int	cmd_size = 0;

	if (!ac) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->mmap_apr == NULL) {
		pr_err("%s: mmap APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: Session[%d]\n", __func__, ac->session);

	buffer_node = kmalloc(sizeof(struct asm_buffer_node), GFP_KERNEL);
	if (!buffer_node)
		return -ENOMEM;

	cmd_size = sizeof(struct avs_cmd_shared_mem_map_regions)
			+ sizeof(struct avs_shared_map_region_payload) * bufcnt;

	mmap_region_cmd = kzalloc(cmd_size, GFP_KERNEL);
	if (mmap_region_cmd == NULL) {
		rc = -EINVAL;
		kfree(buffer_node);
		return rc;
	}
	mmap_regions = (struct avs_cmd_shared_mem_map_regions *)
							mmap_region_cmd;
	q6asm_add_mmaphdr(ac, &mmap_regions->hdr, cmd_size, dir);
	atomic_set(&ac->mem_state, -1);
	mmap_regions->hdr.opcode = ASM_CMD_SHARED_MEM_MAP_REGIONS;
	mmap_regions->mem_pool_id = ADSP_MEMORY_MAP_SHMEM8_4K_POOL;
	mmap_regions->num_regions = bufcnt & 0x00ff;
	mmap_regions->property_flag = 0x00;
	payload = ((u8 *) mmap_region_cmd +
		sizeof(struct avs_cmd_shared_mem_map_regions));
	mregions = (struct avs_shared_map_region_payload *)payload;

	ac->port[dir].tmp_hdl = 0;
	port = &ac->port[dir];
	pr_debug("%s: buf_add 0x%pK, bufsz: %d\n", __func__,
		&buf_add, bufsz);
	mregions->shm_addr_lsw = lower_32_bits(buf_add);
	mregions->shm_addr_msw = msm_audio_populate_upper_32_bits(buf_add);
	mregions->mem_size_bytes = bufsz;
	++mregions;

	rc = apr_send_pkt(ac->mmap_apr, (uint32_t *) mmap_region_cmd);
	if (rc < 0) {
		pr_err("%s: mmap op[0x%x]rc[%d]\n", __func__,
					mmap_regions->hdr.opcode, rc);
		rc = -EINVAL;
		kfree(buffer_node);
		goto fail_cmd;
	}

	rc = wait_event_timeout(ac->mem_wait,
			(atomic_read(&ac->mem_state) >= 0 &&
			 ac->port[dir].tmp_hdl),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for memory_map\n", __func__);
		rc = -ETIMEDOUT;
		kfree(buffer_node);
		goto fail_cmd;
	}
	if (atomic_read(&ac->mem_state) > 0) {
		pr_err("%s: DSP returned error[%s] for memory_map\n",
			__func__, adsp_err_get_err_str(
			atomic_read(&ac->mem_state)));
		rc = adsp_err_get_lnx_err_code(
			atomic_read(&ac->mem_state));
		kfree(buffer_node);
		goto fail_cmd;
	}
	buffer_node->buf_phys_addr = buf_add;
	buffer_node->mmap_hdl = ac->port[dir].tmp_hdl;
	list_add_tail(&buffer_node->list, &ac->port[dir].mem_map_handle);
	ac->port[dir].tmp_hdl = 0;
	rc = 0;

fail_cmd:
	kfree(mmap_region_cmd);
	return rc;
}
EXPORT_SYMBOL(q6asm_memory_map);

/**
 * q6asm_memory_unmap -
 *       command to send memory unmap for ASM
 *
 * @ac: Audio client handle
 * @buf_add: buffer address to unmap
 * @dir: RX or TX session
 *
 * Returns 0 on success or error on failure
 */
int q6asm_memory_unmap(struct audio_client *ac, phys_addr_t buf_add, int dir)
{
	struct avs_cmd_shared_mem_unmap_regions mem_unmap;
	struct asm_buffer_node *buf_node = NULL;
	struct list_head *ptr, *next;

	int rc = 0;

	if (!ac) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (this_mmap.apr == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: Session[%d]\n", __func__, ac->session);

	q6asm_add_mmaphdr(ac, &mem_unmap.hdr,
			sizeof(struct avs_cmd_shared_mem_unmap_regions),
			dir);
	atomic_set(&ac->mem_state, -1);
	mem_unmap.hdr.opcode = ASM_CMD_SHARED_MEM_UNMAP_REGIONS;
	mem_unmap.mem_map_handle = 0;
	list_for_each_safe(ptr, next, &ac->port[dir].mem_map_handle) {
		buf_node = list_entry(ptr, struct asm_buffer_node,
						list);
		if (buf_node->buf_phys_addr == buf_add) {
			pr_debug("%s: Found the element\n", __func__);
			mem_unmap.mem_map_handle = buf_node->mmap_hdl;
			break;
		}
	}
	pr_debug("%s: mem_unmap-mem_map_handle: 0x%x\n",
		__func__, mem_unmap.mem_map_handle);

	if (mem_unmap.mem_map_handle == 0) {
		pr_err("%s: Do not send null mem handle to DSP\n", __func__);
		rc = 0;
		goto fail_cmd;
	}
	rc = apr_send_pkt(ac->mmap_apr, (uint32_t *) &mem_unmap);
	if (rc < 0) {
		pr_err("%s: mem_unmap op[0x%x]rc[%d]\n", __func__,
					mem_unmap.hdr.opcode, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}

	rc = wait_event_timeout(ac->mem_wait,
			(atomic_read(&ac->mem_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for memory_unmap of handle 0x%x\n",
			__func__, mem_unmap.mem_map_handle);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	} else if (atomic_read(&ac->mem_state) > 0) {
		pr_err("%s DSP returned error [%s] map handle 0x%x\n",
			__func__, adsp_err_get_err_str(
			atomic_read(&ac->mem_state)),
			mem_unmap.mem_map_handle);
		rc = adsp_err_get_lnx_err_code(
			atomic_read(&ac->mem_state));
		goto fail_cmd;
	} else if (atomic_read(&ac->unmap_cb_success) == 0) {
		pr_err("%s: Error in mem unmap callback of handle 0x%x\n",
			__func__, mem_unmap.mem_map_handle);
		rc = -EINVAL;
		goto fail_cmd;
	}

	rc = 0;
fail_cmd:
	list_for_each_safe(ptr, next, &ac->port[dir].mem_map_handle) {
		buf_node = list_entry(ptr, struct asm_buffer_node,
						list);
		if (buf_node->buf_phys_addr == buf_add) {
			list_del(&buf_node->list);
			kfree(buf_node);
			break;
		}
	}
	return rc;
}
EXPORT_SYMBOL(q6asm_memory_unmap);

/**
 * q6asm_memory_map_regions -
 *       command to send memory map regions for ASM
 *
 * @ac: Audio client handle
 * @dir: RX or TX session
 * @bufsz: size of each buffer
 * @bufcnt: buffer count
 * @is_contiguous: alloc contiguous mem or not
 *
 * Returns 0 on success or error on failure
 */
static int q6asm_memory_map_regions(struct audio_client *ac, int dir,
				uint32_t bufsz, uint32_t bufcnt,
				bool is_contiguous)
{
	struct avs_cmd_shared_mem_map_regions *mmap_regions = NULL;
	struct avs_shared_map_region_payload  *mregions = NULL;
	struct audio_port_data *port = NULL;
	struct audio_buffer *ab = NULL;
	void	*mmap_region_cmd = NULL;
	void	*payload = NULL;
	struct asm_buffer_node *buffer_node = NULL;
	int	rc = 0;
	int    i = 0;
	uint32_t cmd_size = 0;
	uint32_t bufcnt_t;
	uint32_t bufsz_t;

	if (!ac) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->mmap_apr == NULL) {
		pr_err("%s: mmap APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: Session[%d]\n", __func__, ac->session);

	bufcnt_t = (is_contiguous) ? 1 : bufcnt;
	bufsz_t = (is_contiguous) ? (bufsz * bufcnt) : bufsz;

	if (is_contiguous) {
		/* The size to memory map should be multiple of 4K bytes */
		bufsz_t = PAGE_ALIGN(bufsz_t);
	}

	if (bufcnt_t > (UINT_MAX
			- sizeof(struct avs_cmd_shared_mem_map_regions))
			/ sizeof(struct avs_shared_map_region_payload)) {
		pr_err("%s: Unsigned Integer Overflow. bufcnt_t = %u\n",
				__func__, bufcnt_t);
		return -EINVAL;
	}

	cmd_size = sizeof(struct avs_cmd_shared_mem_map_regions)
			+ (sizeof(struct avs_shared_map_region_payload)
							* bufcnt_t);


	if (bufcnt > (UINT_MAX / sizeof(struct asm_buffer_node))) {
		pr_err("%s: Unsigned Integer Overflow. bufcnt = %u\n",
				__func__, bufcnt);
		return -EINVAL;
	}

	buffer_node = kzalloc(sizeof(struct asm_buffer_node) * bufcnt,
				GFP_KERNEL);
	if (!buffer_node)
		return -ENOMEM;

	mmap_region_cmd = kzalloc(cmd_size, GFP_KERNEL);
	if (mmap_region_cmd == NULL) {
		rc = -EINVAL;
		kfree(buffer_node);
		buffer_node = NULL;
		return rc;
	}
	mmap_regions = (struct avs_cmd_shared_mem_map_regions *)
							mmap_region_cmd;
	q6asm_add_mmaphdr(ac, &mmap_regions->hdr, cmd_size, dir);
	atomic_set(&ac->mem_state, -1);
	pr_debug("%s: mmap_region=0x%pK token=0x%x\n", __func__,
		mmap_regions, ((ac->session << 8) | dir));

	mmap_regions->hdr.opcode = ASM_CMD_SHARED_MEM_MAP_REGIONS;
	mmap_regions->mem_pool_id = ADSP_MEMORY_MAP_SHMEM8_4K_POOL;
	mmap_regions->num_regions = bufcnt_t; /*bufcnt & 0x00ff; */
	mmap_regions->property_flag = 0x00;
	pr_debug("%s: map_regions->nregions = %d\n", __func__,
		mmap_regions->num_regions);
	payload = ((u8 *) mmap_region_cmd +
		sizeof(struct avs_cmd_shared_mem_map_regions));
	mregions = (struct avs_shared_map_region_payload *)payload;

	ac->port[dir].tmp_hdl = 0;
	port = &ac->port[dir];
	for (i = 0; i < bufcnt_t; i++) {
		ab = &port->buf[i];
		mregions->shm_addr_lsw = lower_32_bits(ab->phys);
		mregions->shm_addr_msw =
				msm_audio_populate_upper_32_bits(ab->phys);
		mregions->mem_size_bytes = bufsz_t;
		++mregions;
	}

	rc = apr_send_pkt(ac->mmap_apr, (uint32_t *) mmap_region_cmd);
	if (rc < 0) {
		pr_err("%s: mmap_regions op[0x%x]rc[%d]\n", __func__,
					mmap_regions->hdr.opcode, rc);
		rc = -EINVAL;
		kfree(buffer_node);
		buffer_node = NULL;
		goto fail_cmd;
	}

	rc = wait_event_timeout(ac->mem_wait,
			(atomic_read(&ac->mem_state) >= 0 &&
			 ac->port[dir].tmp_hdl),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for memory_map\n", __func__);
		rc = -ETIMEDOUT;
		kfree(buffer_node);
		buffer_node = NULL;
		goto fail_cmd;
	}
	if (atomic_read(&ac->mem_state) > 0) {
		pr_err("%s DSP returned error for memory_map [%s]\n",
			__func__, adsp_err_get_err_str(
			atomic_read(&ac->mem_state)));
		rc = adsp_err_get_lnx_err_code(
			atomic_read(&ac->mem_state));
		kfree(buffer_node);
		buffer_node = NULL;
		goto fail_cmd;
	}
	mutex_lock(&ac->cmd_lock);

	for (i = 0; i < bufcnt; i++) {
		ab = &port->buf[i];
		buffer_node[i].buf_phys_addr = ab->phys;
		buffer_node[i].mmap_hdl = ac->port[dir].tmp_hdl;
		list_add_tail(&buffer_node[i].list,
			&ac->port[dir].mem_map_handle);
		pr_debug("%s: i=%d, bufadd[i] = 0x%pK, maphdl[i] = 0x%x\n",
			__func__, i, &buffer_node[i].buf_phys_addr,
			buffer_node[i].mmap_hdl);
	}
	ac->port[dir].tmp_hdl = 0;
	mutex_unlock(&ac->cmd_lock);
	rc = 0;
fail_cmd:
	kfree(mmap_region_cmd);
	mmap_region_cmd = NULL;
	return rc;
}

/**
 * q6asm_memory_unmap_regions -
 *       command to send memory unmap regions for ASM
 *
 * @ac: Audio client handle
 * @dir: RX or TX session
 *
 * Returns 0 on success or error on failure
 */
static int q6asm_memory_unmap_regions(struct audio_client *ac, int dir)
{
	struct avs_cmd_shared_mem_unmap_regions mem_unmap;
	struct audio_port_data *port = NULL;
	struct asm_buffer_node *buf_node = NULL;
	struct list_head *ptr, *next;
	phys_addr_t buf_add;
	int	rc = 0;
	int	cmd_size = 0;

	if (!ac) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->mmap_apr == NULL) {
		pr_err("%s: mmap APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: Session[%d]\n", __func__, ac->session);

	cmd_size = sizeof(struct avs_cmd_shared_mem_unmap_regions);
	q6asm_add_mmaphdr(ac, &mem_unmap.hdr, cmd_size, dir);
	atomic_set(&ac->mem_state, -1);
	port = &ac->port[dir];
	buf_add = port->buf->phys;
	mem_unmap.hdr.opcode = ASM_CMD_SHARED_MEM_UNMAP_REGIONS;
	mem_unmap.mem_map_handle = 0;
	list_for_each_safe(ptr, next, &ac->port[dir].mem_map_handle) {
		buf_node = list_entry(ptr, struct asm_buffer_node,
						list);
		if (buf_node->buf_phys_addr == buf_add) {
			pr_debug("%s: Found the element\n", __func__);
			mem_unmap.mem_map_handle = buf_node->mmap_hdl;
			break;
		}
	}

	pr_debug("%s: mem_unmap-mem_map_handle: 0x%x\n",
			__func__, mem_unmap.mem_map_handle);

	if (mem_unmap.mem_map_handle == 0) {
		pr_err("%s: Do not send null mem handle to DSP\n", __func__);
		rc = 0;
		goto fail_cmd;
	}
	rc = apr_send_pkt(ac->mmap_apr, (uint32_t *) &mem_unmap);
	if (rc < 0) {
		pr_err("mmap_regions op[0x%x]rc[%d]\n",
				mem_unmap.hdr.opcode, rc);
		goto fail_cmd;
	}

	rc = wait_event_timeout(ac->mem_wait,
			(atomic_read(&ac->mem_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for memory_unmap of handle 0x%x\n",
			__func__, mem_unmap.mem_map_handle);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	} else if (atomic_read(&ac->mem_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->mem_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->mem_state));
		goto fail_cmd;
	} else if (atomic_read(&ac->unmap_cb_success) == 0) {
		pr_err("%s: Error in mem unmap callback of handle 0x%x\n",
			__func__, mem_unmap.mem_map_handle);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = 0;

fail_cmd:
	list_for_each_safe(ptr, next, &ac->port[dir].mem_map_handle) {
		buf_node = list_entry(ptr, struct asm_buffer_node,
						list);
		if (buf_node->buf_phys_addr == buf_add) {
			list_del(&buf_node->list);
			kfree(buf_node);
			buf_node = NULL;
			break;
		}
	}
	return rc;
}

int q6asm_set_lrgain(struct audio_client *ac, int left_gain, int right_gain)
{
	struct asm_volume_ctrl_multichannel_gain multi_ch_gain;
	struct param_hdr_v3 param_info;
	int rc  = 0;

	memset(&param_info, 0, sizeof(param_info));
	memset(&multi_ch_gain, 0, sizeof(multi_ch_gain));

	param_info.module_id = ASM_MODULE_ID_VOL_CTRL;
	param_info.instance_id = INSTANCE_ID_0;
	param_info.param_id = ASM_PARAM_ID_MULTICHANNEL_GAIN;
	param_info.param_size = sizeof(multi_ch_gain);

	multi_ch_gain.gain_data[0].channeltype = PCM_CHANNEL_FL;
	multi_ch_gain.gain_data[0].gain = left_gain << 15;
	multi_ch_gain.gain_data[1].channeltype = PCM_CHANNEL_FR;
	multi_ch_gain.gain_data[1].gain = right_gain << 15;
	multi_ch_gain.num_channels = 2;
	rc = q6asm_pack_and_set_pp_param_in_band(ac, param_info,
						 (u8 *) &multi_ch_gain);
	if (rc < 0)
		pr_err("%s: set-params send failed paramid[0x%x] rc %d\n",
		       __func__, param_info.param_id, rc);

	return rc;
}

/*
 * q6asm_set_multich_gain: set multiple channel gains on an ASM session
 * @ac: audio client handle
 * @channels: number of channels caller intends to set gains
 * @gains: list of gains of audio channels
 * @ch_map: list of channel mapping. Only valid if use_default is false
 * @use_default: flag to indicate whether to use default mapping
 */
int q6asm_set_multich_gain(struct audio_client *ac, uint32_t channels,
			   uint32_t *gains, uint8_t *ch_map, bool use_default)
{
	struct asm_volume_ctrl_multichannel_gain multich_gain;
	struct param_hdr_v3 param_info;
	int rc  = 0;
	int i;
	u8 default_chmap[VOLUME_CONTROL_MAX_CHANNELS];

	if (ac == NULL) {
		pr_err("%s: Audio client is NULL\n", __func__);
		return -EINVAL;
	}
	if (gains == NULL) {
		dev_err(ac->dev, "%s: gain_list is NULL\n", __func__);
		rc = -EINVAL;
		goto done;
	}
	if (channels > VOLUME_CONTROL_MAX_CHANNELS) {
		dev_err(ac->dev, "%s: Invalid channel count %d\n",
			__func__, channels);
		rc = -EINVAL;
		goto done;
	}
	if (!use_default && ch_map == NULL) {
		dev_err(ac->dev, "%s: NULL channel map\n", __func__);
		rc = -EINVAL;
		goto done;
	}

	memset(&param_info, 0, sizeof(param_info));
	memset(&multich_gain, 0, sizeof(multich_gain));
	param_info.module_id = ASM_MODULE_ID_VOL_CTRL;
	param_info.instance_id = INSTANCE_ID_0;
	param_info.param_id = ASM_PARAM_ID_MULTICHANNEL_GAIN;
	param_info.param_size = sizeof(multich_gain);

	if (use_default) {
		rc = q6asm_map_channels(default_chmap, channels, false);
		if (rc < 0)
			goto done;
		for (i = 0; i < channels; i++) {
			multich_gain.gain_data[i].channeltype =
				default_chmap[i];
			multich_gain.gain_data[i].gain = gains[i] << 15;
		}
	} else {
		for (i = 0; i < channels; i++) {
			multich_gain.gain_data[i].channeltype = ch_map[i];
			multich_gain.gain_data[i].gain = gains[i] << 15;
		}
	}
	multich_gain.num_channels = channels;

	rc = q6asm_pack_and_set_pp_param_in_band(ac, param_info,
						 (u8 *) &multich_gain);
	if (rc)
		pr_err("%s: set-params send failed paramid[0x%x] rc %d\n",
		       __func__, param_info.param_id, rc);
done:
	return rc;
}
EXPORT_SYMBOL(q6asm_set_multich_gain);

/**
 * q6asm_set_mute -
 *       command to set mute for ASM
 *
 * @ac: Audio client handle
 * @muteflag: mute value
 *
 * Returns 0 on success or error on failure
 */
int q6asm_set_mute(struct audio_client *ac, int muteflag)
{
	struct asm_volume_ctrl_mute_config mute;
	struct param_hdr_v3 param_info;
	int rc  = 0;

	memset(&mute, 0, sizeof(mute));
	memset(&param_info, 0, sizeof(param_info));
	param_info.module_id = ASM_MODULE_ID_VOL_CTRL;
	param_info.instance_id = INSTANCE_ID_0;
	param_info.param_id = ASM_PARAM_ID_VOL_CTRL_MUTE_CONFIG;
	param_info.param_size = sizeof(mute);
	mute.mute_flag = muteflag;

	rc = q6asm_pack_and_set_pp_param_in_band(ac, param_info, (u8 *) &mute);
	if (rc)
		pr_err("%s: set-params send failed paramid[0x%x] rc %d\n",
		       __func__, param_info.param_id, rc);
	return rc;
}
EXPORT_SYMBOL(q6asm_set_mute);

static int __q6asm_set_volume(struct audio_client *ac, int volume, int instance)
{
	struct asm_volume_ctrl_master_gain vol;
	struct param_hdr_v3 param_info;
	int rc = 0;

	memset(&vol, 0, sizeof(vol));
	memset(&param_info, 0, sizeof(param_info));

	rc = q6asm_set_soft_volume_module_instance_ids(instance, &param_info);
	if (rc) {
		pr_err("%s: Failed to pack soft volume module and instance IDs, error %d\n",
		       __func__, rc);
		return rc;
	}

	param_info.param_id = ASM_PARAM_ID_VOL_CTRL_MASTER_GAIN;
	param_info.param_size = sizeof(vol);
	vol.master_gain = volume;

	rc = q6asm_pack_and_set_pp_param_in_band(ac, param_info, (u8 *) &vol);
	if (rc)
		pr_err("%s: set-params send failed paramid[0x%x] rc %d\n",
		       __func__, param_info.param_id, rc);

	return rc;
}

/**
 * q6asm_set_volume -
 *       command to set volume for ASM
 *
 * @ac: Audio client handle
 * @volume: volume level
 *
 * Returns 0 on success or error on failure
 */
int q6asm_set_volume(struct audio_client *ac, int volume)
{
	return __q6asm_set_volume(ac, volume, SOFT_VOLUME_INSTANCE_1);
}
EXPORT_SYMBOL(q6asm_set_volume);

int q6asm_set_volume_v2(struct audio_client *ac, int volume, int instance)
{
	return __q6asm_set_volume(ac, volume, instance);
}

/**
 * q6asm_set_aptx_dec_bt_addr -
 *       command to aptx decoder BT addr for ASM
 *
 * @ac: Audio client handle
 * @cfg: APTX decoder bt addr config
 *
 * Returns 0 on success or error on failure
 */
int q6asm_set_aptx_dec_bt_addr(struct audio_client *ac,
				struct aptx_dec_bt_addr_cfg *cfg)
{
	struct aptx_dec_bt_dev_addr paylod;
	int sz = 0;
	int rc = 0;

	pr_debug("%s: BT addr nap %d, uap %d, lap %d\n", __func__, cfg->nap,
			cfg->uap, cfg->lap);

	if (ac == NULL) {
		pr_err("%s: AC handle NULL\n", __func__);
		rc = -EINVAL;
		goto fail_cmd;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		rc = -EINVAL;
		goto fail_cmd;
	}

	sz = sizeof(struct aptx_dec_bt_dev_addr);
	q6asm_add_hdr_async(ac, &paylod.hdr, sz, TRUE);
	atomic_set(&ac->cmd_state, -1);
	paylod.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM;
	paylod.encdec.param_id = APTX_DECODER_BT_ADDRESS;
	paylod.encdec.param_size = sz - sizeof(paylod.hdr)
					- sizeof(paylod.encdec);
	paylod.bt_addr_cfg.lap = cfg->lap;
	paylod.bt_addr_cfg.uap = cfg->uap;
	paylod.bt_addr_cfg.nap = cfg->nap;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &paylod);
	if (rc < 0) {
		pr_err("%s: set-params send failed paramid[0x%x] rc %d\n",
				__func__, paylod.encdec.param_id, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}

	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout, set-params paramid[0x%x]\n", __func__,
			paylod.encdec.param_id);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s] set-params paramid[0x%x]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)),
				paylod.encdec.param_id);
		rc = adsp_err_get_lnx_err_code(
			atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	pr_debug("%s: set BT addr is success\n", __func__);
	rc = 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_set_aptx_dec_bt_addr);

/**
 * q6asm_send_ion_fd -
 *       command to send ION memory map for ASM
 *
 * @ac: Audio client handle
 * @fd: ION file desc
 *
 * Returns 0 on success or error on failure
 */
int q6asm_send_ion_fd(struct audio_client *ac, int fd)
{
	struct dma_buf *dma_buf;
	dma_addr_t paddr;
	size_t pa_len = 0;
	void *vaddr;
	int ret;
	int sz = 0;
	struct avs_rtic_shared_mem_addr shm;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = msm_audio_ion_import(&dma_buf,
				   fd,
				   NULL,
				   0,
				   &paddr,
				   &pa_len,
				   &vaddr);
	if (ret) {
		pr_err("%s: audio ION import failed, rc = %d\n",
		       __func__, ret);
		ret = -ENOMEM;
		goto fail_cmd;
	}
	/* get payload length */
	sz = sizeof(struct avs_rtic_shared_mem_addr);
	q6asm_add_hdr_async(ac, &shm.hdr, sz, TRUE);
	atomic_set(&ac->cmd_state, -1);
	shm.shm_buf_addr_lsw = lower_32_bits(paddr);
	shm.shm_buf_addr_msw = msm_audio_populate_upper_32_bits(paddr);
	shm.buf_size = pa_len;
	shm.shm_buf_num_regions = 1;
	shm.shm_buf_mem_pool_id = ADSP_MEMORY_MAP_SHMEM8_4K_POOL;
	shm.shm_buf_flag = 0x00;
	shm.encdec.param_id = AVS_PARAM_ID_RTIC_SHARED_MEMORY_ADDR;
	shm.encdec.param_size = sizeof(struct avs_rtic_shared_mem_addr) -
						sizeof(struct apr_hdr) -
			sizeof(struct asm_stream_cmd_set_encdec_param_v2);
	shm.encdec.service_id = OUT;
	shm.encdec.reserved = 0;
	shm.map_region.shm_addr_lsw = shm.shm_buf_addr_lsw;
	shm.map_region.shm_addr_msw = shm.shm_buf_addr_msw;
	shm.map_region.mem_size_bytes = pa_len;
	shm.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM_V2;
	ret = apr_send_pkt(ac->apr, (uint32_t *) &shm);
	if (ret < 0) {
		pr_err("%s: set-params send failed paramid[0x%x] rc %d\n",
		       __func__, shm.encdec.param_id, ret);
		ret = -EINVAL;
		goto fail_cmd;
	}

	ret = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		pr_err("%s: timeout, shm.encdec paramid[0x%x]\n", __func__,
		       shm.encdec.param_id);
		ret = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s] shm.encdec paramid[0x%x]\n",
		       __func__,
		       adsp_err_get_err_str(atomic_read(&ac->cmd_state)),
		       shm.encdec.param_id);
		ret = adsp_err_get_lnx_err_code(atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	ret = 0;
fail_cmd:
	return ret;
}
EXPORT_SYMBOL(q6asm_send_ion_fd);

/**
 * q6asm_send_rtic_event_ack -
 *       command to send RTIC event ack
 *
 * @ac: Audio client handle
 * @param: params for event ack
 * @params_length: length of params
 *
 * Returns 0 on success or error on failure
 */
int q6asm_send_rtic_event_ack(struct audio_client *ac,
			      void *param, uint32_t params_length)
{
	char *asm_params = NULL;
	int sz, rc;
	struct avs_param_rtic_event_ack ack;

	if (!param || !ac) {
		pr_err("%s: %s is NULL\n", __func__,
			(!param) ? "param" : "ac");
		rc = -EINVAL;
		goto done;
	}

	sz = sizeof(struct avs_param_rtic_event_ack) + params_length;
	asm_params = kzalloc(sz, GFP_KERNEL);
	if (!asm_params) {
		rc = -ENOMEM;
		goto done;
	}

	q6asm_add_hdr_async(ac, &ack.hdr,
			    sizeof(struct avs_param_rtic_event_ack) +
			    params_length, TRUE);
	atomic_set(&ac->cmd_state, -1);
	ack.hdr.opcode = ASM_STREAM_CMD_SET_ENCDEC_PARAM_V2;
	ack.encdec.param_id = AVS_PARAM_ID_RTIC_EVENT_ACK;
	ack.encdec.param_size = params_length;
	ack.encdec.reserved = 0;
	ack.encdec.service_id = OUT;
	memcpy(asm_params, &ack, sizeof(struct avs_param_rtic_event_ack));
	memcpy(asm_params + sizeof(struct avs_param_rtic_event_ack),
		param, params_length);
	rc = apr_send_pkt(ac->apr, (uint32_t *) asm_params);
	if (rc < 0) {
		pr_err("%s: apr pkt failed for rtic event ack\n", __func__);
		rc = -EINVAL;
		goto fail_send_param;
	}

	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout for rtic event ack cmd\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_send_param;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s] for rtic event ack cmd\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_send_param;
	}
	rc = 0;

fail_send_param:
	kfree(asm_params);
done:
	return rc;
}
EXPORT_SYMBOL(q6asm_send_rtic_event_ack);

/**
 * q6asm_set_softpause -
 *       command to set pause for ASM
 *
 * @ac: Audio client handle
 * @pause_param: params for pause
 *
 * Returns 0 on success or error on failure
 */
int q6asm_set_softpause(struct audio_client *ac,
			struct asm_softpause_params *pause_param)
{
	struct asm_soft_pause_params softpause;
	struct param_hdr_v3 param_info;
	int rc  = 0;

	memset(&softpause, 0, sizeof(softpause));
	memset(&param_info, 0, sizeof(param_info));
	param_info.module_id = ASM_MODULE_ID_VOL_CTRL;
	param_info.instance_id = INSTANCE_ID_0;
	param_info.param_id = ASM_PARAM_ID_SOFT_PAUSE_PARAMETERS;
	param_info.param_size = sizeof(softpause);

	softpause.enable_flag = pause_param->enable;
	softpause.period = pause_param->period;
	softpause.step = pause_param->step;
	softpause.ramping_curve = pause_param->rampingcurve;

	rc = q6asm_pack_and_set_pp_param_in_band(ac, param_info,
						 (u8 *) &softpause);
	if (rc)
		pr_err("%s: set-params send failed paramid[0x%x] rc %d\n",
		       __func__, param_info.param_id, rc);

	return rc;
}
EXPORT_SYMBOL(q6asm_set_softpause);

static int __q6asm_set_softvolume(struct audio_client *ac,
				  struct asm_softvolume_params *softvol_param,
				  int instance)
{
	struct asm_soft_step_volume_params softvol;
	struct param_hdr_v3 param_info;
	int rc = 0;

	memset(&softvol, 0, sizeof(softvol));
	memset(&param_info, 0, sizeof(param_info));

	rc = q6asm_set_soft_volume_module_instance_ids(instance, &param_info);
	if (rc) {
		pr_err("%s: Failed to pack soft volume module and instance IDs, error %d\n",
		       __func__, rc);
		return rc;
	}

	param_info.param_id = ASM_PARAM_ID_SOFT_VOL_STEPPING_PARAMETERS;
	param_info.param_size = sizeof(softvol);

	softvol.period = softvol_param->period;
	softvol.step = softvol_param->step;
	softvol.ramping_curve = softvol_param->rampingcurve;

	rc = q6asm_pack_and_set_pp_param_in_band(ac, param_info,
						 (u8 *) &softvol);
	if (rc)
		pr_err("%s: set-params send failed paramid[0x%x] rc %d\n",
		       __func__, param_info.param_id, rc);

	return rc;
}

/**
 * q6asm_set_softvolume -
 *       command to set softvolume for ASM
 *
 * @ac: Audio client handle
 * @softvol_param: params for softvol
 *
 * Returns 0 on success or error on failure
 */
int q6asm_set_softvolume(struct audio_client *ac,
			 struct asm_softvolume_params *softvol_param)
{
	return __q6asm_set_softvolume(ac, softvol_param,
				      SOFT_VOLUME_INSTANCE_1);
}
EXPORT_SYMBOL(q6asm_set_softvolume);

/**
 * q6asm_set_softvolume_v2 -
 *       command to set softvolume V2 for ASM
 *
 * @ac: Audio client handle
 * @softvol_param: params for softvol
 * @instance: instance to apply softvol
 *
 * Returns 0 on success or error on failure
 */
int q6asm_set_softvolume_v2(struct audio_client *ac,
			    struct asm_softvolume_params *softvol_param,
			    int instance)
{
	return __q6asm_set_softvolume(ac, softvol_param, instance);
}
EXPORT_SYMBOL(q6asm_set_softvolume_v2);

/**
 * q6asm_equalizer -
 *       command to set equalizer for ASM
 *
 * @ac: Audio client handle
 * @eq_p: Equalizer params
 *
 * Returns 0 on success or error on failure
 */
int q6asm_equalizer(struct audio_client *ac, void *eq_p)
{
	struct asm_eq_params eq;
	struct msm_audio_eq_stream_config *eq_params = NULL;
	struct param_hdr_v3 param_info;
	int i  = 0;
	int rc  = 0;

	if (ac == NULL) {
		pr_err("%s: Audio client is NULL\n", __func__);
		return -EINVAL;
	}
	if (eq_p == NULL) {
		pr_err("%s: [%d]: Invalid Eq param\n", __func__, ac->session);
		rc = -EINVAL;
		goto fail_cmd;
	}

	memset(&eq, 0, sizeof(eq));
	memset(&param_info, 0, sizeof(param_info));
	eq_params = (struct msm_audio_eq_stream_config *) eq_p;
	param_info.module_id = ASM_MODULE_ID_EQUALIZER;
	param_info.instance_id = INSTANCE_ID_0;
	param_info.param_id = ASM_PARAM_ID_EQUALIZER_PARAMETERS;
	param_info.param_size = sizeof(eq);
	eq.enable_flag = eq_params->enable;
	eq.num_bands = eq_params->num_bands;

	pr_debug("%s: enable:%d numbands:%d\n", __func__, eq_params->enable,
			eq_params->num_bands);
	for (i = 0; i < eq_params->num_bands; i++) {
		eq.eq_bands[i].band_idx =
			eq_params->eq_bands[i].band_idx;
		eq.eq_bands[i].filterype =
			eq_params->eq_bands[i].filter_type;
		eq.eq_bands[i].center_freq_hz =
			eq_params->eq_bands[i].center_freq_hz;
		eq.eq_bands[i].filter_gain =
			eq_params->eq_bands[i].filter_gain;
		eq.eq_bands[i].q_factor =
			eq_params->eq_bands[i].q_factor;
		pr_debug("%s: filter_type:%u bandnum:%d\n", __func__,
				eq_params->eq_bands[i].filter_type, i);
		pr_debug("%s: center_freq_hz:%u bandnum:%d\n", __func__,
				eq_params->eq_bands[i].center_freq_hz, i);
		pr_debug("%s: filter_gain:%d bandnum:%d\n", __func__,
				eq_params->eq_bands[i].filter_gain, i);
		pr_debug("%s: q_factor:%d bandnum:%d\n", __func__,
				eq_params->eq_bands[i].q_factor, i);
	}
	rc = q6asm_pack_and_set_pp_param_in_band(ac, param_info, (u8 *) &eq);
	if (rc)
		pr_err("%s: set-params send failed paramid[0x%x] rc %d\n",
		       __func__, param_info.param_id, rc);

fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_equalizer);

static int __q6asm_read(struct audio_client *ac, bool is_custom_len_reqd,
			int len)
{
	struct asm_data_cmd_read_v2 read;
	struct asm_buffer_node *buf_node = NULL;
	struct list_head *ptr, *next;
	struct audio_buffer        *ab;
	int dsp_buf;
	struct audio_port_data     *port;
	int rc;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}

	if (ac->io_mode & SYNC_IO_MODE) {
		port = &ac->port[OUT];

		q6asm_add_hdr(ac, &read.hdr, sizeof(read), FALSE);

		mutex_lock(&port->lock);

		dsp_buf = port->dsp_buf;
		if (port->buf == NULL) {
			pr_err("%s: buf is NULL\n", __func__);
			mutex_unlock(&port->lock);
			return -EINVAL;
		}
		ab = &port->buf[dsp_buf];

		dev_vdbg(ac->dev, "%s: session[%d]dsp-buf[%d][%pK]cpu_buf[%d][%pK]\n",
				__func__,
				ac->session,
				dsp_buf,
				port->buf[dsp_buf].data,
				port->cpu_buf,
				&port->buf[port->cpu_buf].phys);

		read.hdr.opcode = ASM_DATA_CMD_READ_V2;
		read.buf_addr_lsw = lower_32_bits(ab->phys);
		read.buf_addr_msw = msm_audio_populate_upper_32_bits(ab->phys);

		list_for_each_safe(ptr, next, &ac->port[OUT].mem_map_handle) {
			buf_node = list_entry(ptr, struct asm_buffer_node,
					list);
			if (buf_node->buf_phys_addr == ab->phys) {
				read.mem_map_handle = buf_node->mmap_hdl;
				break;
			}
		}
		dev_vdbg(ac->dev, "memory_map handle in q6asm_read: [%0x]:",
				read.mem_map_handle);
		read.buf_size = is_custom_len_reqd ? len : ab->size;
		read.seq_id = port->dsp_buf;
		q6asm_update_token(&read.hdr.token,
				   0, /* Session ID is NA */
				   0, /* Stream ID is NA */
				   port->dsp_buf,
				   0, /* Direction flag is NA */
				   WAIT_CMD);
		port->dsp_buf = q6asm_get_next_buf(ac, port->dsp_buf,
						   port->max_buf_cnt);
		mutex_unlock(&port->lock);
		dev_vdbg(ac->dev, "%s: buf add[%pK] token[0x%x] uid[%d]\n",
				__func__, &ab->phys, read.hdr.token,
				read.seq_id);
		rc = apr_send_pkt(ac->apr, (uint32_t *) &read);
		if (rc < 0) {
			pr_err("%s: read op[0x%x]rc[%d]\n",
					__func__, read.hdr.opcode, rc);
			goto fail_cmd;
		}
		return 0;
	}
fail_cmd:
	return -EINVAL;
}

/**
 * q6asm_read -
 *       command to read buffer data from DSP
 *
 * @ac: Audio client handle
 *
 * Returns 0 on success or error on failure
 */
int q6asm_read(struct audio_client *ac)
{
	return __q6asm_read(ac, false/*is_custom_len_reqd*/, 0);
}
EXPORT_SYMBOL(q6asm_read);


/**
 * q6asm_read_v2 -
 *       command to read buffer data from DSP
 *
 * @ac: Audio client handle
 * @len: buffer size to read
 *
 * Returns 0 on success or error on failure
 */
int q6asm_read_v2(struct audio_client *ac, uint32_t len)
{
	return __q6asm_read(ac, true /*is_custom_len_reqd*/, len);
}
EXPORT_SYMBOL(q6asm_read_v2);

/**
 * q6asm_read_nolock -
 *       command to read buffer data from DSP
 *       with no wait for ack.
 *
 * @ac: Audio client handle
 *
 * Returns 0 on success or error on failure
 */
int q6asm_read_nolock(struct audio_client *ac)
{
	struct asm_data_cmd_read_v2 read;
	struct asm_buffer_node *buf_node = NULL;
	struct list_head *ptr, *next;
	struct audio_buffer        *ab;
	int dsp_buf;
	struct audio_port_data     *port;
	int rc;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}

	if (ac->io_mode & SYNC_IO_MODE) {
		port = &ac->port[OUT];

		q6asm_add_hdr_async(ac, &read.hdr, sizeof(read), FALSE);


		dsp_buf = port->dsp_buf;
		ab = &port->buf[dsp_buf];

		dev_vdbg(ac->dev, "%s: session[%d]dsp-buf[%d][%pK]cpu_buf[%d][%pK]\n",
				__func__,
				ac->session,
				dsp_buf,
				port->buf[dsp_buf].data,
				port->cpu_buf,
				&port->buf[port->cpu_buf].phys);

		read.hdr.opcode = ASM_DATA_CMD_READ_V2;
		read.buf_addr_lsw = lower_32_bits(ab->phys);
		read.buf_addr_msw = msm_audio_populate_upper_32_bits(ab->phys);
		read.buf_size = ab->size;
		read.seq_id = port->dsp_buf;
		q6asm_update_token(&read.hdr.token,
				   0, /* Session ID is NA */
				   0, /* Stream ID is NA */
				   port->dsp_buf,
				   0, /* Direction flag is NA */
				   WAIT_CMD);

		list_for_each_safe(ptr, next, &ac->port[OUT].mem_map_handle) {
			buf_node = list_entry(ptr, struct asm_buffer_node,
					list);
			if (buf_node->buf_phys_addr == ab->phys) {
				read.mem_map_handle = buf_node->mmap_hdl;
				break;
			}
		}

		port->dsp_buf = q6asm_get_next_buf(ac, port->dsp_buf,
						   port->max_buf_cnt);
		dev_vdbg(ac->dev, "%s: buf add[%pK] token[0x%x] uid[%d]\n",
				__func__, &ab->phys, read.hdr.token,
				read.seq_id);
		rc = apr_send_pkt(ac->apr, (uint32_t *) &read);
		if (rc < 0) {
			pr_err("%s: read op[0x%x]rc[%d]\n",
					__func__, read.hdr.opcode, rc);
			goto fail_cmd;
		}
		return 0;
	}
fail_cmd:
	return -EINVAL;
}
EXPORT_SYMBOL(q6asm_read_nolock);

/**
 * q6asm_async_write -
 *       command to write DSP buffer
 *
 * @ac: Audio client handle
 * @param: params for async write
 *
 * Returns 0 on success or error on failure
 */
int q6asm_async_write(struct audio_client *ac,
					  struct audio_aio_write_param *param)
{
	int rc = 0;
	struct asm_data_cmd_write_v2 write;
	struct asm_buffer_node *buf_node = NULL;
	struct list_head *ptr, *next;
	struct audio_buffer        *ab;
	struct audio_port_data     *port;
	phys_addr_t lbuf_phys_addr;
	u32 liomode;
	u32 io_compressed;
	u32 io_compressed_stream;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err_ratelimited("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}

	q6asm_stream_add_hdr_async(
			ac, &write.hdr, sizeof(write), TRUE, ac->stream_id);
	port = &ac->port[IN];
	ab = &port->buf[port->dsp_buf];

	/* Pass session id as token for AIO scheme */
	write.hdr.token = param->uid;
	write.hdr.opcode = ASM_DATA_CMD_WRITE_V2;
	write.buf_addr_lsw = lower_32_bits(param->paddr);
	write.buf_addr_msw = msm_audio_populate_upper_32_bits(param->paddr);
	write.buf_size = param->len;
	write.timestamp_msw = param->msw_ts;
	write.timestamp_lsw = param->lsw_ts;
	liomode = (ASYNC_IO_MODE | NT_MODE);
	io_compressed = (ASYNC_IO_MODE | COMPRESSED_IO);
	io_compressed_stream = (ASYNC_IO_MODE | COMPRESSED_STREAM_IO);

	if (ac->io_mode == liomode)
		lbuf_phys_addr = (param->paddr - 32);
	else if (ac->io_mode == io_compressed ||
			ac->io_mode == io_compressed_stream)
		lbuf_phys_addr = (param->paddr - param->metadata_len);
	else {
		if (param->flags & SET_TIMESTAMP)
			lbuf_phys_addr = param->paddr -
				sizeof(struct snd_codec_metadata);
		else
			lbuf_phys_addr = param->paddr;
	}
	dev_vdbg(ac->dev, "%s: token[0x%x], buf_addr[%pK], buf_size[0x%x], ts_msw[0x%x], ts_lsw[0x%x], lbuf_phys_addr: 0x[%pK]\n",
			__func__,
			write.hdr.token, &param->paddr,
			write.buf_size, write.timestamp_msw,
			write.timestamp_lsw, &lbuf_phys_addr);

	/* Use 0xFF00 for disabling timestamps */
	if (param->flags == 0xFF00)
		write.flags = (0x00000000 | (param->flags & 0x800000FF));
	else
		write.flags = (0x80000000 | param->flags);
	write.flags |= param->last_buffer << ASM_SHIFT_LAST_BUFFER_FLAG;
	write.seq_id = param->uid;
	list_for_each_safe(ptr, next, &ac->port[IN].mem_map_handle) {
		buf_node = list_entry(ptr, struct asm_buffer_node,
				list);
		if (buf_node->buf_phys_addr == lbuf_phys_addr) {
			write.mem_map_handle = buf_node->mmap_hdl;
			break;
		}
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &write);
	if (rc < 0) {
		pr_err("%s: write op[0x%x]rc[%d]\n", __func__,
				write.hdr.opcode, rc);
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return -EINVAL;
}
EXPORT_SYMBOL(q6asm_async_write);

/**
 * q6asm_async_read -
 *       command to read DSP buffer
 *
 * @ac: Audio client handle
 * @param: params for async read
 *
 * Returns 0 on success or error on failure
 */
int q6asm_async_read(struct audio_client *ac,
					  struct audio_aio_read_param *param)
{
	int rc = 0;
	struct asm_data_cmd_read_v2 read;
	struct asm_buffer_node *buf_node = NULL;
	struct list_head *ptr, *next;
	phys_addr_t lbuf_phys_addr;
	u32 liomode;
	u32 io_compressed;
	int dir = 0;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err_ratelimited("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}

	q6asm_add_hdr_async(ac, &read.hdr, sizeof(read), FALSE);

	/* Pass session id as token for AIO scheme */
	read.hdr.token = param->uid;
	read.hdr.opcode = ASM_DATA_CMD_READ_V2;
	read.buf_addr_lsw = lower_32_bits(param->paddr);
	read.buf_addr_msw = msm_audio_populate_upper_32_bits(param->paddr);
	read.buf_size = param->len;
	read.seq_id = param->uid;
	liomode = (NT_MODE | ASYNC_IO_MODE);
	io_compressed = (ASYNC_IO_MODE | COMPRESSED_IO);
	if (ac->io_mode == liomode) {
		lbuf_phys_addr = (param->paddr - 32);
		/*legacy wma driver case*/
		dir = IN;
	} else if (ac->io_mode == io_compressed) {
		lbuf_phys_addr = (param->paddr - 64);
		dir = OUT;
	} else {
		if (param->flags & COMPRESSED_TIMESTAMP_FLAG)
			lbuf_phys_addr = param->paddr -
				 sizeof(struct snd_codec_metadata);
		else
			lbuf_phys_addr = param->paddr;
		dir = OUT;
	}

	list_for_each_safe(ptr, next, &ac->port[dir].mem_map_handle) {
		buf_node = list_entry(ptr, struct asm_buffer_node,
				list);
		if (buf_node->buf_phys_addr == lbuf_phys_addr) {
			read.mem_map_handle = buf_node->mmap_hdl;
			break;
		}
	}

	rc = apr_send_pkt(ac->apr, (uint32_t *) &read);
	if (rc < 0 && rc != -ENETRESET) {
		pr_err_ratelimited("%s: read op[0x%x]rc[%d]\n", __func__,
				read.hdr.opcode, rc);
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return -EINVAL;
}
EXPORT_SYMBOL(q6asm_async_read);

/**
 * q6asm_write -
 *       command to write buffer data to DSP
 *
 * @ac: Audio client handle
 * @len: buffer size
 * @msw_ts: upper 32bits of timestamp
 * @lsw_ts: lower 32bits of timestamp
 * @flags: Flags for timestamp mode
 *
 * Returns 0 on success or error on failure
 */
int q6asm_write(struct audio_client *ac, uint32_t len, uint32_t msw_ts,
		uint32_t lsw_ts, uint32_t flags)
{
	int rc = 0;
	struct asm_data_cmd_write_v2 write;
	struct asm_buffer_node *buf_node = NULL;
	struct audio_port_data *port;
	struct audio_buffer    *ab;
	int dsp_buf = 0;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}

	dev_vdbg(ac->dev, "%s: session[%d] len=%d\n",
			__func__, ac->session, len);
	if (ac->io_mode & SYNC_IO_MODE) {
		port = &ac->port[IN];

		q6asm_add_hdr(ac, &write.hdr, sizeof(write),
				FALSE);
		mutex_lock(&port->lock);

		dsp_buf = port->dsp_buf;
		ab = &port->buf[dsp_buf];

		q6asm_update_token(&write.hdr.token,
				   0, /* Session ID is NA */
				   0, /* Stream ID is NA */
				   port->dsp_buf,
				   0, /* Direction flag is NA */
				   NO_WAIT_CMD);
		write.hdr.opcode = ASM_DATA_CMD_WRITE_V2;
		write.buf_addr_lsw = lower_32_bits(ab->phys);
		write.buf_addr_msw = msm_audio_populate_upper_32_bits(ab->phys);
		write.buf_size = len;
		write.seq_id = port->dsp_buf;
		write.timestamp_lsw = lsw_ts;
		write.timestamp_msw = msw_ts;
		/* Use 0xFF00 for disabling timestamps */
		if (flags == 0xFF00)
			write.flags = (0x00000000 | (flags & 0x800000FF));
		else
			write.flags = (0x80000000 | flags);
		port->dsp_buf = q6asm_get_next_buf(ac, port->dsp_buf,
						   port->max_buf_cnt);
		buf_node = list_first_entry(&ac->port[IN].mem_map_handle,
				struct asm_buffer_node,
				list);
		write.mem_map_handle = buf_node->mmap_hdl;

		dev_vdbg(ac->dev, "%s: ab->phys[%pK]bufadd[0x%x] token[0x%x]buf_id[0x%x]buf_size[0x%x]mmaphdl[0x%x]"
				, __func__,
				&ab->phys,
				write.buf_addr_lsw,
				write.hdr.token,
				write.seq_id,
				write.buf_size,
				write.mem_map_handle);
		mutex_unlock(&port->lock);

		config_debug_fs_write(ab);

		rc = apr_send_pkt(ac->apr, (uint32_t *) &write);
		if (rc < 0) {
			pr_err("%s: write op[0x%x]rc[%d]\n",
					__func__, write.hdr.opcode, rc);
			goto fail_cmd;
		}
		return 0;
	}
fail_cmd:
	return -EINVAL;
}
EXPORT_SYMBOL(q6asm_write);

/**
 * q6asm_write_nolock -
 *       command to write buffer data to DSP
 *       with no wait for ack.
 *
 * @ac: Audio client handle
 * @len: buffer size
 * @msw_ts: upper 32bits of timestamp
 * @lsw_ts: lower 32bits of timestamp
 * @flags: Flags for timestamp mode
 *
 * Returns 0 on success or error on failure
 */
int q6asm_write_nolock(struct audio_client *ac, uint32_t len, uint32_t msw_ts,
			uint32_t lsw_ts, uint32_t flags)
{
	int rc = 0;
	struct asm_data_cmd_write_v2 write;
	struct asm_buffer_node *buf_node = NULL;
	struct audio_port_data *port;
	struct audio_buffer    *ab;
	int dsp_buf = 0;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}

	dev_vdbg(ac->dev, "%s: session[%d] len=%d\n",
			__func__, ac->session, len);
	if (ac->io_mode & SYNC_IO_MODE) {
		port = &ac->port[IN];

		q6asm_add_hdr_async(ac, &write.hdr, sizeof(write),
				FALSE);

		dsp_buf = port->dsp_buf;
		ab = &port->buf[dsp_buf];

		q6asm_update_token(&write.hdr.token,
				   0, /* Session ID is NA */
				   0, /* Stream ID is NA */
				   port->dsp_buf,
				   0, /* Direction flag is NA */
				   NO_WAIT_CMD);

		write.hdr.opcode = ASM_DATA_CMD_WRITE_V2;
		write.buf_addr_lsw = lower_32_bits(ab->phys);
		write.buf_addr_msw = msm_audio_populate_upper_32_bits(ab->phys);
		write.buf_size = len;
		write.seq_id = port->dsp_buf;
		write.timestamp_lsw = lsw_ts;
		write.timestamp_msw = msw_ts;
		buf_node = list_first_entry(&ac->port[IN].mem_map_handle,
				struct asm_buffer_node,
				list);
		write.mem_map_handle = buf_node->mmap_hdl;
		/* Use 0xFF00 for disabling timestamps */
		if (flags == 0xFF00)
			write.flags = (0x00000000 | (flags & 0x800000FF));
		else
			write.flags = (0x80000000 | flags);
		port->dsp_buf = q6asm_get_next_buf(ac, port->dsp_buf,
						   port->max_buf_cnt);

		dev_vdbg(ac->dev, "%s: ab->phys[%pK]bufadd[0x%x]token[0x%x] buf_id[0x%x]buf_size[0x%x]mmaphdl[0x%x]"
				, __func__,
				&ab->phys,
				write.buf_addr_lsw,
				write.hdr.token,
				write.seq_id,
				write.buf_size,
				write.mem_map_handle);

		rc = apr_send_pkt(ac->apr, (uint32_t *) &write);
		if (rc < 0) {
			pr_err("%s: write op[0x%x]rc[%d]\n",
					__func__, write.hdr.opcode, rc);
			goto fail_cmd;
		}
		return 0;
	}
fail_cmd:
	return -EINVAL;
}
EXPORT_SYMBOL(q6asm_write_nolock);

/**
 * q6asm_get_session_time_v2 -
 *       command to retrieve timestamp info
 *
 * @ac: Audio client handle
 * @ses_time: pointer to fill with session timestamp info
 * @abs_time: pointer to fill with AVS timestamp info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_get_session_time_v2(struct audio_client *ac, uint64_t *ses_time,
			      uint64_t *abs_time)
{
	struct asm_mtmx_strtr_get_params mtmx_params;
	int rc;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ses_time == NULL) {
		pr_err("%s: tstamp args are NULL\n", __func__);
		return -EINVAL;
	}

	q6asm_add_hdr(ac, &mtmx_params.hdr, sizeof(mtmx_params), TRUE);
	mtmx_params.hdr.opcode = ASM_SESSION_CMD_GET_MTMX_STRTR_PARAMS_V2;
	mtmx_params.param_info.data_payload_addr_lsw = 0;
	mtmx_params.param_info.data_payload_addr_msw = 0;
	mtmx_params.param_info.mem_map_handle = 0;
	mtmx_params.param_info.direction = (ac->io_mode & TUN_READ_IO_MODE
					    ? 1 : 0);
	mtmx_params.param_info.module_id =
		ASM_SESSION_MTMX_STRTR_MODULE_ID_AVSYNC;
	mtmx_params.param_info.param_id =
		ASM_SESSION_MTMX_STRTR_PARAM_SESSION_TIME_V3;
	mtmx_params.param_info.param_max_size =
		sizeof(struct param_hdr_v1) +
		sizeof(struct asm_session_mtmx_strtr_param_session_time_v3_t);
	atomic_set(&ac->time_flag, 1);

	dev_vdbg(ac->dev, "%s: session[%d]opcode[0x%x]\n", __func__,
		 ac->session, mtmx_params.hdr.opcode);
	rc = apr_send_pkt(ac->apr, (uint32_t *) &mtmx_params);
	if (rc < 0) {
		dev_err_ratelimited(ac->dev, "%s: Get Session Time failed %d\n",
				    __func__, rc);
		return rc;
	}

	rc = wait_event_timeout(ac->time_wait,
			(atomic_read(&ac->time_flag) == 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout in getting session time from DSP\n",
		       __func__);
		goto fail_cmd;
	}

	*ses_time = ac->dsp_ts.time_stamp;
	if (abs_time != NULL)
		*abs_time = ac->dsp_ts.abs_time_stamp;
	return 0;

fail_cmd:
	return -EINVAL;
}
EXPORT_SYMBOL(q6asm_get_session_time_v2);

/**
 * q6asm_get_session_time -
 *       command to retrieve timestamp info
 *
 * @ac: Audio client handle
 * @tstamp: pointer to fill with timestamp info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_get_session_time(struct audio_client *ac, uint64_t *tstamp)
{
	return q6asm_get_session_time_v2(ac, tstamp, NULL);
}
EXPORT_SYMBOL(q6asm_get_session_time);

/**
 * q6asm_get_session_time_legacy -
 *       command to retrieve timestamp info
 *
 * @ac: Audio client handle
 * @tstamp: pointer to fill with timestamp info
 *
 * Returns 0 on success or error on failure
 */
int q6asm_get_session_time_legacy(struct audio_client *ac, uint64_t *tstamp)
{
	struct apr_hdr hdr;
	int rc;

	if (ac == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (tstamp == NULL) {
		pr_err("%s: tstamp NULL\n", __func__);
		return -EINVAL;
	}

	q6asm_add_hdr(ac, &hdr, sizeof(hdr), TRUE);
	hdr.opcode = ASM_SESSION_CMD_GET_SESSIONTIME_V3;
	atomic_set(&ac->time_flag, 1);

	dev_vdbg(ac->dev, "%s: session[%d]opcode[0x%x]\n", __func__,
			ac->session,
			hdr.opcode);
	rc = apr_send_pkt(ac->apr, (uint32_t *) &hdr);
	if (rc < 0) {
		pr_err("%s: Commmand 0x%x failed %d\n",
				__func__, hdr.opcode, rc);
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->time_wait,
			(atomic_read(&ac->time_flag) == 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout in getting session time from DSP\n",
				__func__);
		goto fail_cmd;
	}

	*tstamp = ac->dsp_ts.time_stamp;
	return 0;

fail_cmd:
	return -EINVAL;
}
EXPORT_SYMBOL(q6asm_get_session_time_legacy);

/**
 * q6asm_send_mtmx_strtr_window -
 *       command to send matrix for window params
 *
 * @ac: Audio client handle
 * @window_param: window params
 * @param_id: param id for window
 *
 * Returns 0 on success or error on failure
 */
int q6asm_send_mtmx_strtr_window(struct audio_client *ac,
		struct asm_session_mtmx_strtr_param_window_v2_t *window_param,
		uint32_t param_id)
{
	struct asm_mtmx_strtr_params matrix;
	int sz = 0;
	int rc  = 0;

	pr_debug("%s: Window lsw is %d, window msw is %d\n", __func__,
		  window_param->window_lsw, window_param->window_msw);

	if (!ac) {
		pr_err("%s: audio client handle is NULL\n", __func__);
		rc = -EINVAL;
		goto fail_cmd;
	}

	if (ac->apr == NULL) {
		pr_err("%s: ac->apr is NULL", __func__);
		rc = -EINVAL;
		goto fail_cmd;
	}

	sz = sizeof(struct asm_mtmx_strtr_params);
	q6asm_add_hdr(ac, &matrix.hdr, sz, TRUE);
	atomic_set(&ac->cmd_state, -1);
	matrix.hdr.opcode = ASM_SESSION_CMD_SET_MTMX_STRTR_PARAMS_V2;

	matrix.param.data_payload_addr_lsw = 0;
	matrix.param.data_payload_addr_msw = 0;
	matrix.param.mem_map_handle = 0;
	matrix.param.data_payload_size =
		sizeof(struct param_hdr_v1) +
		sizeof(struct asm_session_mtmx_strtr_param_window_v2_t);
	matrix.param.direction = 0; /* RX */
	matrix.data.module_id = ASM_SESSION_MTMX_STRTR_MODULE_ID_AVSYNC;
	matrix.data.param_id = param_id;
	matrix.data.param_size =
		sizeof(struct asm_session_mtmx_strtr_param_window_v2_t);
	matrix.data.reserved = 0;
	memcpy(&(matrix.config.window_param),
	       window_param,
	       sizeof(struct asm_session_mtmx_strtr_param_window_v2_t));

	rc = apr_send_pkt(ac->apr, (uint32_t *) &matrix);
	if (rc < 0) {
		pr_err("%s: Render window start send failed paramid [0x%x]\n",
			__func__, matrix.data.param_id);
		rc = -EINVAL;
		goto fail_cmd;
	}

	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout, Render window start paramid[0x%x]\n",
			__func__, matrix.data.param_id);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	rc = 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_send_mtmx_strtr_window);

/**
 * q6asm_send_mtmx_strtr_render_mode -
 *       command to send matrix for render mode
 *
 * @ac: Audio client handle
 * @render_mode: rendering mode
 *
 * Returns 0 on success or error on failure
 */
int q6asm_send_mtmx_strtr_render_mode(struct audio_client *ac,
		uint32_t render_mode)
{
	struct asm_mtmx_strtr_params matrix;
	struct asm_session_mtmx_strtr_param_render_mode_t render_param;
	int sz = 0;
	int rc  = 0;

	pr_debug("%s: render mode is %d\n", __func__, render_mode);

	if (!ac) {
		pr_err("%s: audio client handle is NULL\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	if (ac->apr == NULL) {
		pr_err("%s: ac->apr is NULL\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	if ((render_mode != ASM_SESSION_MTMX_STRTR_PARAM_RENDER_DEFAULT) &&
	    (render_mode != ASM_SESSION_MTMX_STRTR_PARAM_RENDER_LOCAL_STC)) {
		pr_err("%s: Invalid render mode %d\n", __func__, render_mode);
		rc = -EINVAL;
		goto exit;
	}

	memset(&render_param, 0,
	       sizeof(struct asm_session_mtmx_strtr_param_render_mode_t));
	render_param.flags = render_mode;

	memset(&matrix, 0, sizeof(struct asm_mtmx_strtr_params));
	sz = sizeof(struct asm_mtmx_strtr_params);
	q6asm_add_hdr(ac, &matrix.hdr, sz, TRUE);
	atomic_set(&ac->cmd_state, -1);
	matrix.hdr.opcode = ASM_SESSION_CMD_SET_MTMX_STRTR_PARAMS_V2;

	matrix.param.data_payload_addr_lsw = 0;
	matrix.param.data_payload_addr_msw = 0;
	matrix.param.mem_map_handle = 0;
	matrix.param.data_payload_size =
		sizeof(struct param_hdr_v1) +
		sizeof(struct asm_session_mtmx_strtr_param_render_mode_t);
	matrix.param.direction = 0; /* RX */
	matrix.data.module_id = ASM_SESSION_MTMX_STRTR_MODULE_ID_AVSYNC;
	matrix.data.param_id = ASM_SESSION_MTMX_STRTR_PARAM_RENDER_MODE_CMD;
	matrix.data.param_size =
		sizeof(struct asm_session_mtmx_strtr_param_render_mode_t);
	matrix.data.reserved = 0;
	memcpy(&(matrix.config.render_param),
	       &render_param,
	       sizeof(struct asm_session_mtmx_strtr_param_render_mode_t));

	rc = apr_send_pkt(ac->apr, (uint32_t *) &matrix);
	if (rc < 0) {
		pr_err("%s: Render mode send failed paramid [0x%x]\n",
			__func__, matrix.data.param_id);
		rc = -EINVAL;
		goto exit;
	}

	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout, Render mode send paramid [0x%x]\n",
			__func__, matrix.data.param_id);
		rc = -ETIMEDOUT;
		goto exit;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto exit;
	}
	rc = 0;
exit:
	return rc;
}
EXPORT_SYMBOL(q6asm_send_mtmx_strtr_render_mode);

/**
 * q6asm_send_mtmx_strtr_clk_rec_mode -
 *       command to send matrix for clock rec
 *
 * @ac: Audio client handle
 * @clk_rec_mode: mode for clock rec
 *
 * Returns 0 on success or error on failure
 */
int q6asm_send_mtmx_strtr_clk_rec_mode(struct audio_client *ac,
		uint32_t clk_rec_mode)
{
	struct asm_mtmx_strtr_params matrix;
	struct asm_session_mtmx_strtr_param_clk_rec_t clk_rec_param;
	int sz = 0;
	int rc  = 0;

	pr_debug("%s: clk rec mode is %d\n", __func__, clk_rec_mode);

	if (!ac) {
		pr_err("%s: audio client handle is NULL\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	if (ac->apr == NULL) {
		pr_err("%s: ac->apr is NULL\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	if ((clk_rec_mode != ASM_SESSION_MTMX_STRTR_PARAM_CLK_REC_NONE) &&
	    (clk_rec_mode != ASM_SESSION_MTMX_STRTR_PARAM_CLK_REC_AUTO)) {
		pr_err("%s: Invalid clk rec mode %d\n", __func__, clk_rec_mode);
		rc = -EINVAL;
		goto exit;
	}

	memset(&clk_rec_param, 0,
	       sizeof(struct asm_session_mtmx_strtr_param_clk_rec_t));
	clk_rec_param.flags = clk_rec_mode;

	memset(&matrix, 0, sizeof(struct asm_mtmx_strtr_params));
	sz = sizeof(struct asm_mtmx_strtr_params);
	q6asm_add_hdr(ac, &matrix.hdr, sz, TRUE);
	atomic_set(&ac->cmd_state, -1);
	matrix.hdr.opcode = ASM_SESSION_CMD_SET_MTMX_STRTR_PARAMS_V2;

	matrix.param.data_payload_addr_lsw = 0;
	matrix.param.data_payload_addr_msw = 0;
	matrix.param.mem_map_handle = 0;
	matrix.param.data_payload_size =
		sizeof(struct param_hdr_v1) +
		sizeof(struct asm_session_mtmx_strtr_param_clk_rec_t);
	matrix.param.direction = 0; /* RX */
	matrix.data.module_id = ASM_SESSION_MTMX_STRTR_MODULE_ID_AVSYNC;
	matrix.data.param_id = ASM_SESSION_MTMX_STRTR_PARAM_CLK_REC_CMD;
	matrix.data.param_size =
		sizeof(struct asm_session_mtmx_strtr_param_clk_rec_t);
	matrix.data.reserved = 0;
	memcpy(&(matrix.config.clk_rec_param),
	       &clk_rec_param,
	       sizeof(struct asm_session_mtmx_strtr_param_clk_rec_t));

	rc = apr_send_pkt(ac->apr, (uint32_t *) &matrix);
	if (rc < 0) {
		pr_err("%s: clk rec mode send failed paramid [0x%x]\n",
			__func__, matrix.data.param_id);
		rc = -EINVAL;
		goto exit;
	}

	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout, clk rec mode send paramid [0x%x]\n",
			__func__, matrix.data.param_id);
		rc = -ETIMEDOUT;
		goto exit;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto exit;
	}
	rc = 0;
exit:
	return rc;
}
EXPORT_SYMBOL(q6asm_send_mtmx_strtr_clk_rec_mode);

/**
 * q6asm_send_mtmx_strtr_enable_adjust_session_clock -
 *       command to send matrix for adjust time
 *
 * @ac: Audio client handle
 * @enable: flag to adjust time or not
 *
 * Returns 0 on success or error on failure
 */
int q6asm_send_mtmx_strtr_enable_adjust_session_clock(struct audio_client *ac,
		bool enable)
{
	struct asm_mtmx_strtr_params matrix;
	struct asm_session_mtmx_param_adjust_session_time_ctl_t adjust_time;
	int sz = 0;
	int rc  = 0;

	pr_debug("%s: adjust session enable %d\n", __func__, enable);

	if (!ac) {
		pr_err("%s: audio client handle is NULL\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	if (ac->apr == NULL) {
		pr_err("%s: ac->apr is NULL\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	adjust_time.enable = enable;
	memset(&matrix, 0, sizeof(struct asm_mtmx_strtr_params));
	sz = sizeof(struct asm_mtmx_strtr_params);
	q6asm_add_hdr(ac, &matrix.hdr, sz, TRUE);
	atomic_set(&ac->cmd_state, -1);
	matrix.hdr.opcode = ASM_SESSION_CMD_SET_MTMX_STRTR_PARAMS_V2;

	matrix.param.data_payload_addr_lsw = 0;
	matrix.param.data_payload_addr_msw = 0;
	matrix.param.mem_map_handle = 0;
	matrix.param.data_payload_size =
		sizeof(struct param_hdr_v1) +
		sizeof(struct asm_session_mtmx_param_adjust_session_time_ctl_t);
	matrix.param.direction = 0; /* RX */
	matrix.data.module_id = ASM_SESSION_MTMX_STRTR_MODULE_ID_AVSYNC;
	matrix.data.param_id = ASM_SESSION_MTMX_PARAM_ADJUST_SESSION_TIME_CTL;
	matrix.data.param_size =
		sizeof(struct asm_session_mtmx_param_adjust_session_time_ctl_t);
	matrix.data.reserved = 0;
	matrix.config.adj_time_param.enable = adjust_time.enable;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &matrix);
	if (rc < 0) {
		pr_err("%s: enable adjust session failed failed paramid [0x%x]\n",
			__func__, matrix.data.param_id);
		rc = -EINVAL;
		goto exit;
	}

	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: enable adjust session failed failed paramid [0x%x]\n",
			__func__, matrix.data.param_id);
		rc = -ETIMEDOUT;
		goto exit;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto exit;
	}
	rc = 0;
exit:
	return rc;
}
EXPORT_SYMBOL(q6asm_send_mtmx_strtr_enable_adjust_session_clock);


static int __q6asm_cmd(struct audio_client *ac, int cmd, uint32_t stream_id)
{
	struct apr_hdr hdr;
	int rc;
	atomic_t *state;
	int cnt = 0;

	if (!ac) {
		pr_err_ratelimited("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err_ratelimited("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	q6asm_stream_add_hdr(ac, &hdr, sizeof(hdr), TRUE, stream_id);
	atomic_set(&ac->cmd_state, -1);
	/*
	 * Updated the token field with stream/session for compressed playback
	 * Platform driver must know the the stream with which the command is
	 * associated
	 */
	if (ac->io_mode & COMPRESSED_STREAM_IO)
		q6asm_update_token(&hdr.token,
				   ac->session,
				   stream_id,
				   0, /* Buffer index is NA */
				   0, /* Direction flag is NA */
				   WAIT_CMD);
	pr_debug("%s: token = 0x%x, stream_id  %d, session 0x%x\n",
			__func__, hdr.token, stream_id, ac->session);
	switch (cmd) {
	case CMD_PAUSE:
		pr_debug("%s: CMD_PAUSE\n", __func__);
		hdr.opcode = ASM_SESSION_CMD_PAUSE;
		state = &ac->cmd_state;
		break;
	case CMD_SUSPEND:
		pr_debug("%s: CMD_SUSPEND\n", __func__);
		hdr.opcode = ASM_SESSION_CMD_SUSPEND;
		state = &ac->cmd_state;
		break;
	case CMD_FLUSH:
		pr_debug("%s: CMD_FLUSH\n", __func__);
		hdr.opcode = ASM_STREAM_CMD_FLUSH;
		state = &ac->cmd_state;
		break;
	case CMD_OUT_FLUSH:
		pr_debug("%s: CMD_OUT_FLUSH\n", __func__);
		hdr.opcode = ASM_STREAM_CMD_FLUSH_READBUFS;
		state = &ac->cmd_state;
		break;
	case CMD_EOS:
		pr_debug("%s: CMD_EOS\n", __func__);
		hdr.opcode = ASM_DATA_CMD_EOS_V2;
		atomic_set(&ac->cmd_state, 0);
		state = &ac->cmd_state;
		break;
	case CMD_CLOSE:
		pr_debug("%s: CMD_CLOSE\n", __func__);
		hdr.opcode = ASM_STREAM_CMD_CLOSE;
		state = &ac->cmd_state;
		break;
	default:
		pr_err("%s: Invalid format[%d]\n", __func__, cmd);
		rc = -EINVAL;
		goto fail_cmd;
	}
	pr_debug("%s: session[%d]opcode[0x%x]\n", __func__,
			ac->session,
			hdr.opcode);
	rc = apr_send_pkt(ac->apr, (uint32_t *) &hdr);
	if (rc < 0) {
		pr_err_ratelimited("%s: Commmand 0x%x failed %d\n",
				__func__, hdr.opcode, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait, (atomic_read(state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for response opcode[0x%x]\n",
				__func__, hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(state) > 0) {
		pr_err("%s: DSP returned error[%s] opcode %d\n",
					__func__, adsp_err_get_err_str(
					atomic_read(state)),
					hdr.opcode);
		rc = adsp_err_get_lnx_err_code(atomic_read(state));
		goto fail_cmd;
	}

	if (cmd == CMD_FLUSH)
		q6asm_reset_buf_state(ac);
	if (cmd == CMD_CLOSE) {
		/* check if DSP return all buffers */
		if (ac->port[IN].buf) {
			for (cnt = 0; cnt < ac->port[IN].max_buf_cnt;
					cnt++) {
				if (ac->port[IN].buf[cnt].used == IN) {
					dev_vdbg(ac->dev, "Write Buf[%d] not returned\n",
							cnt);
				}
			}
		}
		if (ac->port[OUT].buf) {
			for (cnt = 0; cnt < ac->port[OUT].max_buf_cnt; cnt++) {
				if (ac->port[OUT].buf[cnt].used == OUT) {
					dev_vdbg(ac->dev, "Read Buf[%d] not returned\n",
							cnt);
				}
			}
		}
	}
	return 0;
fail_cmd:
	return rc;
}

/**
 * q6asm_cmd -
 *       Function used to send commands for
 *       ASM with wait for ack.
 *
 * @ac: Audio client handle
 * @cmd: command to send
 *
 * Returns 0 on success or error on failure
 */
int q6asm_cmd(struct audio_client *ac, int cmd)
{
	return __q6asm_cmd(ac, cmd, ac->stream_id);
}
EXPORT_SYMBOL(q6asm_cmd);

/**
 * q6asm_stream_cmd -
 *       Function used to send commands for
 *       ASM stream with wait for ack.
 *
 * @ac: Audio client handle
 * @cmd: command to send
 * @stream_id: Stream ID
 *
 * Returns 0 on success or error on failure
 */
int q6asm_stream_cmd(struct audio_client *ac, int cmd, uint32_t stream_id)
{
	return __q6asm_cmd(ac, cmd, stream_id);
}
EXPORT_SYMBOL(q6asm_stream_cmd);

/**
 * q6asm_cmd_nowait -
 *       Function used to send commands for
 *       ASM stream without wait for ack.
 *
 * @ac: Audio client handle
 * @cmd: command to send
 * @stream_id: Stream ID
 *
 * Returns 0 on success or error on failure
 */
static int __q6asm_cmd_nowait(struct audio_client *ac, int cmd,
			      uint32_t stream_id)
{
	struct apr_hdr hdr;
	int rc;

	if (!ac) {
		pr_err_ratelimited("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err_ratelimited("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	q6asm_stream_add_hdr_async(ac, &hdr, sizeof(hdr), TRUE, stream_id);
	atomic_set(&ac->cmd_state, 1);
	/*
	 * Updated the token field with stream/session for compressed playback
	 * Platform driver must know the the stream with which the command is
	 * associated
	 */
	if (ac->io_mode & COMPRESSED_STREAM_IO)
		q6asm_update_token(&hdr.token,
				   ac->session,
				   stream_id,
				   0, /* Buffer index is NA */
				   0, /* Direction flag is NA */
				   NO_WAIT_CMD);

	pr_debug("%s: token = 0x%x, stream_id  %d, session 0x%x\n",
			__func__, hdr.token, stream_id, ac->session);
	switch (cmd) {
	case CMD_PAUSE:
		pr_debug("%s: CMD_PAUSE\n", __func__);
		hdr.opcode = ASM_SESSION_CMD_PAUSE;
		break;
	case CMD_EOS:
		pr_debug("%s: CMD_EOS\n", __func__);
		hdr.opcode = ASM_DATA_CMD_EOS_V2;
		break;
	case CMD_CLOSE:
		pr_debug("%s: CMD_CLOSE\n", __func__);
		hdr.opcode = ASM_STREAM_CMD_CLOSE;
		break;
	default:
		pr_err("%s: Invalid format[%d]\n", __func__, cmd);
		goto fail_cmd;
	}
	pr_debug("%s: session[%d]opcode[0x%x]\n", __func__,
			ac->session,
			hdr.opcode);

	rc = apr_send_pkt(ac->apr, (uint32_t *) &hdr);
	if (rc < 0) {
		pr_err("%s: Commmand 0x%x failed %d\n",
				__func__, hdr.opcode, rc);
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return -EINVAL;
}

int q6asm_cmd_nowait(struct audio_client *ac, int cmd)
{
	pr_debug("%s: stream_id: %d\n", __func__, ac->stream_id);
	return __q6asm_cmd_nowait(ac, cmd, ac->stream_id);
}
EXPORT_SYMBOL(q6asm_cmd_nowait);

/**
 * q6asm_stream_cmd_nowait -
 *       Function used to send commands for
 *       ASM stream without wait for ack.
 *
 * @ac: Audio client handle
 * @cmd: command to send
 * @stream_id: Stream ID
 *
 * Returns 0 on success or error on failure
 */
int q6asm_stream_cmd_nowait(struct audio_client *ac, int cmd,
			    uint32_t stream_id)
{
	pr_debug("%s: stream_id: %d\n", __func__, stream_id);
	return __q6asm_cmd_nowait(ac, cmd, stream_id);
}
EXPORT_SYMBOL(q6asm_stream_cmd_nowait);

int __q6asm_send_meta_data(struct audio_client *ac, uint32_t stream_id,
			  uint32_t initial_samples, uint32_t trailing_samples)
{
	struct asm_data_cmd_remove_silence silence;
	int rc = 0;

	if (!ac) {
		pr_err_ratelimited("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err_ratelimited("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: session[%d]\n", __func__, ac->session);
	q6asm_stream_add_hdr_async(ac, &silence.hdr, sizeof(silence), TRUE,
			stream_id);

	/*
	 * Updated the token field with stream/session for compressed playback
	 * Platform driver must know the the stream with which the command is
	 * associated
	 */
	if (ac->io_mode & COMPRESSED_STREAM_IO)
		q6asm_update_token(&silence.hdr.token,
				   ac->session,
				   stream_id,
				   0, /* Buffer index is NA */
				   0, /* Direction flag is NA */
				   NO_WAIT_CMD);
	pr_debug("%s: token = 0x%x, stream_id  %d, session 0x%x\n",
			__func__, silence.hdr.token, stream_id, ac->session);

	silence.hdr.opcode = ASM_DATA_CMD_REMOVE_INITIAL_SILENCE;
	silence.num_samples_to_remove    = initial_samples;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &silence);
	if (rc < 0) {
		pr_err("%s: Commmand silence failed[%d]", __func__, rc);

		goto fail_cmd;
	}

	silence.hdr.opcode = ASM_DATA_CMD_REMOVE_TRAILING_SILENCE;
	silence.num_samples_to_remove    = trailing_samples;


	rc = apr_send_pkt(ac->apr, (uint32_t *) &silence);
	if (rc < 0) {
		pr_err("%s: Commmand silence failed[%d]", __func__, rc);
		goto fail_cmd;
	}

	return 0;
fail_cmd:
	return -EINVAL;
}

/**
 * q6asm_stream_send_meta_data -
 *       command to send meta data for stream
 *
 * @ac: Audio client handle
 * @stream_id: Stream ID
 * @initial_samples: Initial samples of stream
 * @trailing_samples: Trailing samples of stream
 *
 * Returns 0 on success or error on failure
 */
int q6asm_stream_send_meta_data(struct audio_client *ac, uint32_t stream_id,
		uint32_t initial_samples, uint32_t trailing_samples)
{
	return __q6asm_send_meta_data(ac, stream_id, initial_samples,
				     trailing_samples);
}
EXPORT_SYMBOL(q6asm_stream_send_meta_data);

int q6asm_send_meta_data(struct audio_client *ac, uint32_t initial_samples,
		uint32_t trailing_samples)
{
	return __q6asm_send_meta_data(ac, ac->stream_id, initial_samples,
				     trailing_samples);
}

static void q6asm_reset_buf_state(struct audio_client *ac)
{
	int cnt = 0;
	int loopcnt = 0;
	int used;
	struct audio_port_data *port = NULL;

	if (ac->io_mode & SYNC_IO_MODE) {
		used = (ac->io_mode & TUN_WRITE_IO_MODE ? 1 : 0);
		mutex_lock(&ac->cmd_lock);
		for (loopcnt = 0; loopcnt <= OUT; loopcnt++) {
			port = &ac->port[loopcnt];
			cnt = port->max_buf_cnt - 1;
			port->dsp_buf = 0;
			port->cpu_buf = 0;
			while (cnt >= 0) {
				if (!port->buf)
					continue;
				port->buf[cnt].used = used;
				cnt--;
			}
		}
		mutex_unlock(&ac->cmd_lock);
	}
}

/**
 * q6asm_reg_tx_overflow -
 *       command to register for TX overflow events
 *
 * @ac: Audio client handle
 * @enable: flag to enable or disable events
 *
 * Returns 0 on success or error on failure
 */
int q6asm_reg_tx_overflow(struct audio_client *ac, uint16_t enable)
{
	struct asm_session_cmd_regx_overflow tx_overflow;
	int rc;

	if (!ac) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: session[%d]enable[%d]\n", __func__,
			ac->session, enable);
	q6asm_add_hdr(ac, &tx_overflow.hdr, sizeof(tx_overflow), TRUE);
	atomic_set(&ac->cmd_state, -1);

	tx_overflow.hdr.opcode =
				 ASM_SESSION_CMD_REGISTER_FORX_OVERFLOW_EVENTS;
	/* tx overflow event: enable */
	tx_overflow.enable_flag = enable;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &tx_overflow);
	if (rc < 0) {
		pr_err("%s: tx overflow op[0x%x]rc[%d]\n",
				__func__, tx_overflow.hdr.opcode, rc);
		rc = -EINVAL;
		goto fail_cmd;
	}
	rc = wait_event_timeout(ac->cmd_wait,
				(atomic_read(&ac->cmd_state) >= 0),
				msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for tx overflow\n", __func__);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}
	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}

	return 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_reg_tx_overflow);

int q6asm_reg_rx_underflow(struct audio_client *ac, uint16_t enable)
{
	struct asm_session_cmd_rgstr_rx_underflow rx_underflow;
	int rc;

	if (!ac) {
		pr_err("%s: AC APR handle NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->apr == NULL) {
		pr_err("%s: APR handle NULL\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: session[%d]enable[%d]\n", __func__,
			ac->session, enable);
	q6asm_add_hdr_async(ac, &rx_underflow.hdr, sizeof(rx_underflow), FALSE);

	rx_underflow.hdr.opcode =
		ASM_SESSION_CMD_REGISTER_FOR_RX_UNDERFLOW_EVENTS;
	/* tx overflow event: enable */
	rx_underflow.enable_flag = enable;

	rc = apr_send_pkt(ac->apr, (uint32_t *) &rx_underflow);
	if (rc < 0) {
		pr_err("%s: tx overflow op[0x%x]rc[%d]\n",
				__func__, rx_underflow.hdr.opcode, rc);
		goto fail_cmd;
	}
	return 0;
fail_cmd:
	return -EINVAL;
}

/**
 * q6asm_adjust_session_clock -
 *       command to adjust session clock
 *
 * @ac: Audio client handle
 * @adjust_time_lsw: lower 32bits
 * @adjust_time_msw: upper 32bits
 *
 * Returns 0 on success or error on failure
 */
int q6asm_adjust_session_clock(struct audio_client *ac,
		uint32_t adjust_time_lsw,
		uint32_t adjust_time_msw)
{
	int rc = 0;
	int sz = 0;
	struct asm_session_cmd_adjust_session_clock_v2 adjust_clock;

	pr_debug("%s: adjust_time_lsw is %x, adjust_time_msw is %x\n", __func__,
		  adjust_time_lsw, adjust_time_msw);

	if (!ac) {
		pr_err("%s: audio client handle is NULL\n", __func__);
		rc = -EINVAL;
		goto fail_cmd;
	}

	if (ac->apr == NULL) {
		pr_err("%s: ac->apr is NULL", __func__);
		rc = -EINVAL;
		goto fail_cmd;
	}

	sz = sizeof(struct asm_session_cmd_adjust_session_clock_v2);
	q6asm_add_hdr(ac, &adjust_clock.hdr, sz, TRUE);
	atomic_set(&ac->cmd_state, -1);
	adjust_clock.hdr.opcode = ASM_SESSION_CMD_ADJUST_SESSION_CLOCK_V2;

	adjust_clock.adjustime_lsw = adjust_time_lsw;
	adjust_clock.adjustime_msw = adjust_time_msw;


	rc = apr_send_pkt(ac->apr, (uint32_t *) &adjust_clock);
	if (rc < 0) {
		pr_err("%s: adjust_clock send failed paramid [0x%x]\n",
			__func__, adjust_clock.hdr.opcode);
		rc = -EINVAL;
		goto fail_cmd;
	}

	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout, adjust_clock paramid[0x%x]\n",
			__func__, adjust_clock.hdr.opcode);
		rc = -ETIMEDOUT;
		goto fail_cmd;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		goto fail_cmd;
	}
	rc = 0;
fail_cmd:
	return rc;
}
EXPORT_SYMBOL(q6asm_adjust_session_clock);

/*
 * q6asm_get_path_delay() - get the path delay for an audio session
 * @ac: audio client handle
 *
 * Retrieves the current audio DSP path delay for the given audio session.
 *
 * Return: 0 on success, error code otherwise
 */
int q6asm_get_path_delay(struct audio_client *ac)
{
	int rc = 0;
	struct apr_hdr hdr;

	if (!ac || ac->apr == NULL) {
		pr_err("%s: invalid audio client\n", __func__);
		return -EINVAL;
	}

	hdr.opcode = ASM_SESSION_CMD_GET_PATH_DELAY_V2;
	q6asm_add_hdr(ac, &hdr, sizeof(hdr), TRUE);
	atomic_set(&ac->cmd_state, -1);

	rc = apr_send_pkt(ac->apr, (uint32_t *) &hdr);
	if (rc < 0) {
		pr_err("%s: Commmand 0x%x failed %d\n", __func__,
				hdr.opcode, rc);
		return rc;
	}

	rc = wait_event_timeout(ac->cmd_wait,
			(atomic_read(&ac->cmd_state) >= 0),
			msecs_to_jiffies(TIMEOUT_MS));
	if (!rc) {
		pr_err("%s: timeout. waited for response opcode[0x%x]\n",
				__func__, hdr.opcode);
		return -ETIMEDOUT;
	}

	if (atomic_read(&ac->cmd_state) > 0) {
		pr_err("%s: DSP returned error[%s]\n",
				__func__, adsp_err_get_err_str(
				atomic_read(&ac->cmd_state)));
		rc = adsp_err_get_lnx_err_code(
				atomic_read(&ac->cmd_state));
		return rc;
	}

	return 0;
}
EXPORT_SYMBOL(q6asm_get_path_delay);

int q6asm_get_apr_service_id(int session_id)
{
	int service_id;

	pr_debug("%s:\n", __func__);

	if (session_id <= 0 || session_id > ASM_ACTIVE_STREAMS_ALLOWED) {
		pr_err("%s: invalid session_id = %d\n", __func__, session_id);
		return -EINVAL;
	}
	mutex_lock(&session[session_id].mutex_lock_per_session);
	if (session[session_id].ac != NULL)
		if ((session[session_id].ac)->apr != NULL) {
			service_id = ((struct apr_svc *)(session[session_id].ac)->apr)->id;
			mutex_unlock(&session[session_id].mutex_lock_per_session);
			return service_id;
	}
	mutex_unlock(&session[session_id].mutex_lock_per_session);
	return -EINVAL;
}

uint8_t q6asm_get_asm_stream_id(int session_id)
{
	uint8_t stream_id = 1;
	pr_debug("%s:\n", __func__);

	if (session_id <= 0 || session_id > ASM_ACTIVE_STREAMS_ALLOWED) {
		pr_err("%s: invalid session_id = %d\n", __func__, session_id);
		goto done;
	}
	if (session[session_id].ac == NULL) {
		pr_err("%s: session not created for session id = %d\n",
		       __func__, session_id);
		goto done;
	}
	stream_id = (session[session_id].ac)->stream_id;

done:
	return stream_id;
}

int q6asm_get_asm_topology(int session_id)
{
	int topology = -EINVAL;

	if (session_id <= 0 || session_id > ASM_ACTIVE_STREAMS_ALLOWED) {
		pr_err("%s: invalid session_id = %d\n", __func__, session_id);
		goto done;
	}
	if (session[session_id].ac == NULL) {
		pr_err("%s: session not created for session id = %d\n",
		       __func__, session_id);
		goto done;
	}
	topology = (session[session_id].ac)->topology;
done:
	return topology;
}

int q6asm_get_asm_app_type(int session_id)
{
	int app_type = -EINVAL;

	if (session_id <= 0 || session_id > ASM_ACTIVE_STREAMS_ALLOWED) {
		pr_err("%s: invalid session_id = %d\n", __func__, session_id);
		goto done;
	}
	if (session[session_id].ac == NULL) {
		pr_err("%s: session not created for session id = %d\n",
		       __func__, session_id);
		goto done;
	}
	app_type = (session[session_id].ac)->app_type;
done:
	return app_type;
}

/*
 * Retrieving cal_block will mark cal_block as stale.
 * Hence it cannot be reused or resent unless the flag
 * is reset.
 */
static int q6asm_get_asm_topology_apptype(struct q6asm_cal_info *cal_info, struct audio_client *ac)
{
	int path = 0;
	int cal_buf_index = 0;
	struct cal_block_data *cal_block = NULL;

	cal_info->topology_id = DEFAULT_POPP_TOPOLOGY;
	cal_info->app_type = DEFAULT_APP_TYPE;

	if (cal_data[ASM_TOPOLOGY_CAL] == NULL)
		goto done;
	if (!ac) {
		pr_err("%s: Audio client is NULL\n", __func__);
		goto done;
	}
	if (ac->stream_type == SNDRV_PCM_STREAM_PLAYBACK)
		path = RX_DEVICE;
	else
		path = TX_DEVICE;

	mutex_lock(&cal_data[ASM_TOPOLOGY_CAL]->lock);
	if (is_q6asm_topology_cal_send_by_v6(path, ac->fedai_id)) {
		pr_debug("%s: ASM topology cal block is send by acdb_loader_send_audio_cal_v6!\n",
				 __func__);
		cal_buf_index = path + MAX_PATH_TYPE * ac->fedai_id + ASM_CAL_SEND_VERSION_OFFSET;
		cal_block = q6asm_find_cal_by_buf_number(ASM_TOPOLOGY_CAL,
						   ac->app_type,
						   path,
						   cal_buf_index);
		if (cal_block == NULL) {
			pr_err("%s: Couldn't find cal_block with buf_number!\n", __func__);
			goto unlock;
		}
	} else {
		cal_block = q6asm_find_cal_by_buf_number(ASM_TOPOLOGY_CAL, 0, 0, path);
		if (cal_block == NULL) {
			pr_debug("%s: Couldn't find cal_block with buf_number, re-routing "
				"search using CAL type only\n", __func__);
			cal_block = cal_utils_get_only_cal_block(cal_data[ASM_TOPOLOGY_CAL]);
		}
		if (cal_block == NULL || cal_utils_is_cal_stale(cal_block))
			goto unlock;
	}
	cal_info->topology_id = ((struct audio_cal_info_asm_top *)
		cal_block->cal_info)->topology;
	cal_info->app_type = ((struct audio_cal_info_asm_top *)
		cal_block->cal_info)->app_type;

	cal_utils_mark_cal_used(cal_block);

unlock:
	mutex_unlock(&cal_data[ASM_TOPOLOGY_CAL]->lock);
done:
	pr_debug("%s: Using topology %d app_type %d\n", __func__,
			cal_info->topology_id, cal_info->app_type);

	return 0;
}

/**
 * q6asm_send_cal -
 *       command to send ASM calibration
 *
 * @ac: Audio client handle
 *
 * Returns 0 on success or error on failure
 */
int q6asm_send_cal(struct audio_client *ac)
{
	struct cal_block_data *cal_block = NULL;
	struct mem_mapping_hdr mem_hdr;
	u32 payload_size = 0;
	int path = 0;
	int rc = -EINVAL;
	pr_debug("%s:\n", __func__);

	if (!ac) {
		pr_err("%s: Audio client is NULL\n", __func__);
		return -EINVAL;
	}
	if (ac->io_mode & NT_MODE) {
		pr_debug("%s: called for NT MODE, exiting\n", __func__);
		goto done;
	}

	if (cal_data[ASM_AUDSTRM_CAL] == NULL)
		goto done;

	if (ac->perf_mode == ULTRA_LOW_LATENCY_PCM_MODE) {
		rc = 0; /* no cal is required, not error case */
		goto done;
	}

	if (ac->stream_type == SNDRV_PCM_STREAM_PLAYBACK)
		path = RX_DEVICE;
	else
		path = TX_DEVICE;

	memset(&mem_hdr, 0, sizeof(mem_hdr));
	mutex_lock(&cal_data[ASM_AUDSTRM_CAL]->lock);
	cal_block = q6asm_find_cal_by_buf_number(ASM_AUDSTRM_CAL, ac->app_type, path, ac->fedai_id);
	if (cal_block == NULL) {
		pr_debug("%s: Couldn't find cal_block with buf_number, re-routing "
			"search using app_type only\n", __func__);
		cal_block = q6asm_find_cal_by_app_type(ASM_AUDSTRM_CAL, ac->app_type);
		if (cal_block == NULL) {
			pr_err("%s: cal_block is NULL\n",
				__func__);
			goto unlock;
		}
	}
	if (cal_utils_is_cal_stale(cal_block)) {
		rc = 0; /* not error case */
		pr_debug("%s: cal_block is stale\n",
			__func__);
		goto unlock;
	}

	if (cal_block->cal_data.size == 0) {
		rc = 0; /* not error case */
		pr_debug("%s: cal_data.size is 0, don't send cal data\n",
			__func__);
		goto unlock;
	}

	rc = remap_cal_data(ASM_AUDSTRM_CAL_TYPE, cal_block);
	if (rc) {
		pr_err("%s: Remap_cal_data failed for cal %d!\n",
			__func__, ASM_AUDSTRM_CAL);
		goto unlock;
	}

	mem_hdr.data_payload_addr_lsw =
		lower_32_bits(cal_block->cal_data.paddr);
	mem_hdr.data_payload_addr_msw =
		msm_audio_populate_upper_32_bits(cal_block->cal_data.paddr);
	mem_hdr.mem_map_handle = cal_block->map_data.q6map_handle;
	payload_size = cal_block->cal_data.size;

	pr_debug("%s: phyaddr lsw = %x msw = %x, maphdl = %x calsize = %d\n",
		 __func__, mem_hdr.data_payload_addr_lsw,
		 mem_hdr.data_payload_addr_msw, mem_hdr.mem_map_handle,
		 payload_size);

	rc = q6asm_set_pp_params(ac, &mem_hdr, NULL, payload_size);
	if (rc) {
		pr_err("%s: audio audstrm cal send failed\n", __func__);
		goto unlock;
	}

	if (cal_block)
		cal_utils_mark_cal_used(cal_block);
	rc = 0;

unlock:
	mutex_unlock(&cal_data[ASM_AUDSTRM_CAL]->lock);
done:
	return rc;
}
EXPORT_SYMBOL(q6asm_send_cal);

static int get_cal_type_index(int32_t cal_type)
{
	int ret = -EINVAL;

	switch (cal_type) {
	case ASM_TOPOLOGY_CAL_TYPE:
		ret = ASM_TOPOLOGY_CAL;
		break;
	case ASM_CUST_TOPOLOGY_CAL_TYPE:
		ret = ASM_CUSTOM_TOP_CAL;
		break;
	case ASM_AUDSTRM_CAL_TYPE:
		ret = ASM_AUDSTRM_CAL;
		break;
	case ASM_RTAC_APR_CAL_TYPE:
		ret = ASM_RTAC_APR_CAL;
		break;
	default:
		pr_err("%s: invalid cal type %d!\n", __func__, cal_type);
	}
	return ret;
}

static int q6asm_alloc_cal(int32_t cal_type,
				size_t data_size, void *data)
{
	int ret = 0;
	int cal_index;

	pr_debug("%s:\n", __func__);

	cal_index = get_cal_type_index(cal_type);
	if (cal_index < 0) {
		pr_err("%s: could not get cal index %d!\n",
			__func__, cal_index);
		ret = -EINVAL;
		goto done;
	}

	ret = cal_utils_alloc_cal(data_size, data,
		cal_data[cal_index], 0, NULL);
	if (ret < 0) {
		pr_err("%s: cal_utils_alloc_block failed, ret = %d, cal type = %d!\n",
			__func__, ret, cal_type);
		ret = -EINVAL;
		goto done;
	}
done:
	return ret;
}

static int q6asm_dealloc_cal(int32_t cal_type,
				size_t data_size, void *data)
{
	int ret = 0;
	int cal_index;

	pr_debug("%s:\n", __func__);

	cal_index = get_cal_type_index(cal_type);
	if (cal_index < 0) {
		pr_err("%s: could not get cal index %d!\n",
			__func__, cal_index);
		ret = -EINVAL;
		goto done;
	}

	ret = cal_utils_dealloc_cal(data_size, data,
		cal_data[cal_index]);
	if (ret < 0) {
		pr_err("%s: cal_utils_dealloc_block failed, ret = %d, cal type = %d!\n",
			__func__, ret, cal_type);
		ret = -EINVAL;
		goto done;
	}
done:
	return ret;
}

static int q6asm_set_cal(int32_t cal_type,
			size_t data_size, void *data)
{
	int ret = 0;
	int cal_index;

	pr_debug("%s:\n", __func__);

	cal_index = get_cal_type_index(cal_type);
	if (cal_index < 0) {
		pr_err("%s: could not get cal index %d!\n",
			__func__, cal_index);
		ret = -EINVAL;
		goto done;
	}

	ret = cal_utils_set_cal(data_size, data,
		cal_data[cal_index], 0, NULL);
	if (ret < 0) {
		pr_err("%s: cal_utils_set_cal failed, ret = %d, cal type = %d!\n",
			__func__, ret, cal_type);
		ret = -EINVAL;
		goto done;
	}

	if (cal_index == ASM_CUSTOM_TOP_CAL) {
		mutex_lock(&cal_data[ASM_CUSTOM_TOP_CAL]->lock);
		set_custom_topology = 1;
		mutex_unlock(&cal_data[ASM_CUSTOM_TOP_CAL]->lock);
	}
done:
	return ret;
}

static void q6asm_delete_cal_data(void)
{
	pr_debug("%s:\n", __func__);
	cal_utils_destroy_cal_types(ASM_MAX_CAL_TYPES, cal_data);
}

static int q6asm_init_cal_data(void)
{
	int ret = 0;
	struct cal_type_info	cal_type_info[] = {
		{{ASM_TOPOLOGY_CAL_TYPE,
		{NULL, NULL, NULL,
		q6asm_set_cal, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} },

		{{ASM_CUST_TOPOLOGY_CAL_TYPE,
		{q6asm_alloc_cal, q6asm_dealloc_cal, NULL,
		q6asm_set_cal, NULL, NULL} },
		{NULL, q6asm_unmap_cal_memory, cal_utils_match_buf_num} },

		{{ASM_AUDSTRM_CAL_TYPE,
		{q6asm_alloc_cal, q6asm_dealloc_cal, NULL,
		q6asm_set_cal, NULL, NULL} },
		{NULL, q6asm_unmap_cal_memory, cal_utils_match_buf_num} },

		{{ASM_RTAC_APR_CAL_TYPE,
		{NULL, NULL, NULL, NULL, NULL, NULL} },
		{NULL, NULL, cal_utils_match_buf_num} }
	};
	pr_debug("%s\n", __func__);

	ret = cal_utils_create_cal_types(ASM_MAX_CAL_TYPES, cal_data,
		cal_type_info);
	if (ret < 0) {
		pr_err("%s: could not create cal type! %d\n",
			__func__, ret);
		ret = -EINVAL;
		goto err;
	}

	return ret;
err:
	q6asm_delete_cal_data();
	return ret;
}

static int q6asm_is_valid_session(struct apr_client_data *data, void *priv)
{
	struct audio_client *ac = (struct audio_client *)priv;
	union asm_token_struct asm_token;

	asm_token.token = data->token;
	if (asm_token._token.session_id != ac->session) {
		pr_err("%s: Invalid session[%d] rxed expected[%d]",
			__func__, asm_token._token.session_id, ac->session);
		return -EINVAL;
	}
	return 0;
}

int __init q6asm_init(void)
{
	int lcnt, ret;

	pr_debug("%s:\n", __func__);

	memset(session, 0, sizeof(struct audio_session) *
		(ASM_ACTIVE_STREAMS_ALLOWED + 1));
	for (lcnt = 0; lcnt <= ASM_ACTIVE_STREAMS_ALLOWED; lcnt++) {
		spin_lock_init(&(session[lcnt].session_lock));
		mutex_init(&(session[lcnt].mutex_lock_per_session));
	}
	set_custom_topology = 1;

	/*setup common client used for cal mem map */
	common_client.session = ASM_CONTROL_SESSION;
	common_client.port[0].buf = &common_buf[0];
	common_client.port[1].buf = &common_buf[1];
	init_waitqueue_head(&common_client.cmd_wait);
	init_waitqueue_head(&common_client.time_wait);
	init_waitqueue_head(&common_client.mem_wait);
	atomic_set(&common_client.time_flag, 1);
	INIT_LIST_HEAD(&common_client.port[0].mem_map_handle);
	INIT_LIST_HEAD(&common_client.port[1].mem_map_handle);
	mutex_init(&common_client.cmd_lock);
	for (lcnt = 0; lcnt <= OUT; lcnt++) {
		mutex_init(&common_client.port[lcnt].lock);
		spin_lock_init(&common_client.port[lcnt].dsp_lock);
	}
	atomic_set(&common_client.cmd_state, 0);
	atomic_set(&common_client.mem_state, 0);

	ret = q6asm_init_cal_data();
	if (ret)
		pr_err("%s: could not init cal data! ret %d\n",
			__func__, ret);

	config_debug_fs_init();

	return 0;
}

void q6asm_exit(void)
{
	int lcnt;
	q6asm_delete_cal_data();
	for (lcnt = 0; lcnt <= OUT; lcnt++)
		mutex_destroy(&common_client.port[lcnt].lock);

	mutex_destroy(&common_client.cmd_lock);

	for (lcnt = 0; lcnt <= ASM_ACTIVE_STREAMS_ALLOWED; lcnt++)
		mutex_destroy(&(session[lcnt].mutex_lock_per_session));
}
