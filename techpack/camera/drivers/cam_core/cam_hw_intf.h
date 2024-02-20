/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 */

#ifndef _CAM_HW_INTF_H_
#define _CAM_HW_INTF_H_

#include <linux/types.h>

/*
 * This file declares Constants, Enums, Structures and APIs to be used as
 * Interface between HW driver and HW Manager.
 */

/**
 * struct cam_hw_ops - Hardware layer interface functions
 *
 * @get_hw_caps:           Function pointer for get hw caps
 * @init:                  Function poniter for initialize hardware
 * @deinit:                Function pointer for deinitialize hardware
 * @reset:                 Function pointer for reset hardware
 * @reserve:               Function pointer for reserve hardware
 * @release:               Function pointer for release hardware
 * @start:                 Function pointer for start hardware
 * @stop:                  Function pointer for stop hardware
 * @read:                  Function pointer for read hardware registers
 * @write:                 Function pointer for Write hardware registers
 * @process_cmd:           Function pointer for additional hardware controls
 * @flush_cmd:             Function pointer for flush requests
 *
 */
struct cam_hw_ops {
	int (*get_hw_caps)(void *hw_priv,
		void *get_hw_cap_args, uint32_t arg_size);
	int (*init)(void *hw_priv,
		void *init_hw_args, uint32_t arg_size);
	int (*deinit)(void *hw_priv,
		void *init_hw_args, uint32_t arg_size);
	int (*reset)(void *hw_priv,
		void *reset_core_args, uint32_t arg_size);
	int (*reserve)(void *hw_priv,
		void *reserve_args, uint32_t arg_size);
	int (*release)(void *hw_priv,
		void *release_args, uint32_t arg_size);
	int (*start)(void *hw_priv,
		void *start_args, uint32_t arg_size);
	int (*stop)(void *hw_priv,
		void *stop_args, uint32_t arg_size);
	int (*read)(void *hw_priv,
		void *read_args, uint32_t arg_size);
	int (*write)(void *hw_priv,
		void *write_args, uint32_t arg_size);
	int (*process_cmd)(void *hw_priv,
		uint32_t cmd_type, void *cmd_args, uint32_t arg_size);
	int (*flush)(void *hw_priv,
		void *flush_args, uint32_t arg_size);
};

/**
 * struct cam_hw_intf - Common hardware node
 *
 * @hw_type:               Hardware type
 * @hw_idx:                Hardware ID
 * @hw_ops:                Hardware interface function table
 * @hw_priv:               Private hardware node pointer
 *
 */
struct cam_hw_intf {
	uint32_t                     hw_type;
	uint32_t                     hw_idx;
	struct cam_hw_ops            hw_ops;
	void                        *hw_priv;
};

/* hardware event callback function type */
typedef int (*cam_hw_mgr_event_cb_func)(void *priv, uint32_t evt_id,
	void *evt_data);

#endif /* _CAM_HW_INTF_H_ */
