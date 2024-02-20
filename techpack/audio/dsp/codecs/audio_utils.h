/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2010-2015, 2017, 2020 The Linux Foundation. All rights reserved.
 */
#include <audio/linux/msm_audio.h>
#include <linux/compat.h>
#include "q6audio_common.h"

#define FRAME_NUM	(8)

#define PCM_BUF_COUNT		(2)

#define AUD_EOS_SET  0x01
#define TUNNEL_MODE     0x0000
#define NON_TUNNEL_MODE 0x0001

#define NO_BUF_ALLOC	0x00
#define BUF_ALLOC_IN    0x01
#define BUF_ALLOC_OUT   0x02
#define BUF_ALLOC_INOUT 0x03
#define ALIGN_BUF_SIZE(size) ((size + 4095) & (~4095))

struct timestamp {
	u32 lowpart;
	u32 highpart;
} __packed;

struct meta_in {
	unsigned short offset;
	struct timestamp ntimestamp;
	unsigned int nflags;
} __packed;

struct meta_out_dsp {
	u32 offset_to_frame;
	u32 frame_size;
	u32 encoded_pcm_samples;
	u32 msw_ts;
	u32 lsw_ts;
	u32 nflags;
} __packed;

struct meta_out {
	unsigned char num_of_frames;
	struct meta_out_dsp meta_out_dsp[];
} __packed;

struct q6audio_in {
	spinlock_t			dsp_lock;
	atomic_t			in_bytes;
	atomic_t			in_samples;

	struct mutex			lock;
	struct mutex			read_lock;
	struct mutex			write_lock;
	wait_queue_head_t		read_wait;
	wait_queue_head_t		write_wait;

	struct audio_client             *ac;
	struct msm_audio_stream_config  str_cfg;
	void				*enc_cfg;
	struct msm_audio_buf_cfg        buf_cfg;
	struct msm_audio_config		pcm_cfg;
	void				*codec_cfg;

	/* number of buffers available to read/write */
	atomic_t			in_count;
	atomic_t			out_count;

	/* first idx: num of frames per buf, second idx: offset to frame */
	uint32_t			out_frame_info[FRAME_NUM][2];
	int				eos_rsp;
	int				opened;
	int				enabled;
	int				stopped;
	int				event_abort;
	int				feedback; /* Flag indicates whether used
						   * in Non Tunnel mode
						   */
	int				rflush;
	int				wflush;
	int				buf_alloc;
	uint16_t			min_frame_size;
	uint16_t			max_frames_per_buf;
	bool				reset_event;
	long (*enc_ioctl)(struct file *, unsigned int, unsigned long);
	long (*enc_compat_ioctl)(struct file *, unsigned int, unsigned long);
};

int audio_in_enable(struct q6audio_in  *audio);
int audio_in_disable(struct q6audio_in  *audio);
int audio_in_buf_alloc(struct q6audio_in *audio);
long audio_in_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg);
#ifdef CONFIG_COMPAT
long audio_in_compat_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg);
#else
#define audio_in_compat_ioctl NULL
#endif
ssize_t audio_in_read(struct file *file, char __user *buf,
		size_t count, loff_t *pos);
ssize_t audio_in_write(struct file *file, const char __user *buf,
		size_t count, loff_t *pos);
int audio_in_release(struct inode *inode, struct file *file);
int audio_in_set_config(struct file *file, struct msm_audio_config *cfg);
int aac_in_init(void);
int amrnb_in_init(void);
int amrwb_in_init(void);
int audio_aac_init(void);
int audio_alac_init(void);
int audio_amrnb_init(void);
int audio_amrwb_init(void);
int audio_amrwbplus_init(void);
int audio_ape_init(void);
int audio_evrc_init(void);
int audio_g711alaw_init(void);
int audio_g711mlaw_init(void);
int audio_effects_init(void);
int audio_mp3_init(void);
int audio_multiaac_init(void);
int audio_qcelp_init(void);
int audio_wma_init(void);
int audio_wmapro_init(void);
int evrc_in_init(void);
int g711alaw_in_init(void);
int g711mlaw_in_init(void);
int qcelp_in_init(void);
void aac_in_exit(void);
void amrnb_in_exit(void);
void amrwb_in_exit(void);
void audio_aac_exit(void);
void audio_alac_exit(void);
void audio_amrnb_exit(void);
void audio_amrwb_exit(void);
void audio_amrwbplus_exit(void);
void audio_ape_exit(void);
void audio_evrc_exit(void);
void audio_g711alaw_exit(void);
void audio_g711mlaw_exit(void);
void audio_effects_exit(void);
void audio_mp3_exit(void);
void audio_multiaac_exit(void);
void audio_qcelp_exit(void);
void audio_wma_exit(void);
void audio_wmapro_exit(void);
void evrc_in_exit(void);
void g711alaw_in_exit(void);
void g711mlaw_in_exit(void);
void qcelp_in_exit(void);
