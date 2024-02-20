/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2014, 2018, 2020 The Linux Foundation. All rights reserved.
 */
#ifndef _AUDIO_CAL_UTILS_H
#define _AUDIO_CAL_UTILS_H

#include <linux/msm_ion.h>
#include <audio/linux/msm_audio_calibration.h>
#include <dsp/msm_audio_ion.h>
#include <dsp/audio_calibration.h>

struct cal_data {
	size_t		size;
	void		*kvaddr;
	phys_addr_t	paddr;
};

struct mem_map_data {
	size_t			map_size;
	int32_t			q6map_handle;
	int32_t			ion_map_handle;
	struct dma_buf		*dma_buf;
};

struct cal_block_data {
	size_t			client_info_size;
	void			*client_info;
	void			*cal_info;
	struct list_head	list;
	struct cal_data		cal_data;
	bool			cal_stale;
	bool			cma_mem;
	struct mem_map_data	map_data;
	int32_t			buffer_number;
};

struct cal_util_callbacks {
	int (*map_cal)
		(int32_t cal_type, struct cal_block_data *cal_block);
	int (*unmap_cal)
		(int32_t cal_type, struct cal_block_data *cal_block);
	bool (*match_block)
		(struct cal_block_data *cal_block, void *user_data);
};

struct cal_type_info {
	struct audio_cal_reg		reg;
	struct cal_util_callbacks	cal_util_callbacks;
};

struct cal_type_data {
	struct cal_type_info		info;
	struct mutex			lock;
	struct list_head		cal_blocks;
};


/* to register & degregister with cal util driver */
int cal_utils_create_cal_types(int num_cal_types,
			struct cal_type_data **cal_type,
			struct cal_type_info *info);
void cal_utils_destroy_cal_types(int num_cal_types,
			struct cal_type_data **cal_type);

/* common functions for callbacks */
int cal_utils_alloc_cal(size_t data_size, void *data,
			struct cal_type_data *cal_type,
			size_t client_info_size, void *client_info);
int cal_utils_dealloc_cal(size_t data_size, void *data,
			struct cal_type_data *cal_type);
int cal_utils_set_cal(size_t data_size, void *data,
			struct cal_type_data *cal_type,
			size_t client_info_size, void *client_info);

/* use for SSR */
void cal_utils_clear_cal_block_q6maps(int num_cal_types,
					struct cal_type_data **cal_type);


/* common matching functions used to add blocks */
bool cal_utils_match_buf_num(struct cal_block_data *cal_block,
					void *user_data);

/* common matching functions to find cal blocks */
struct cal_block_data *cal_utils_get_only_cal_block(
			struct cal_type_data *cal_type);

/* Size of calibration specific data */
size_t get_cal_info_size(int32_t cal_type);
size_t get_user_cal_type_size(int32_t cal_type);

/* Version of the cal type*/
int32_t cal_utils_get_cal_type_version(void *cal_type_data);

void cal_utils_mark_cal_used(struct cal_block_data *cal_block);

bool cal_utils_is_cal_stale(struct cal_block_data *cal_block);

int cal_utils_init(void);
#endif
