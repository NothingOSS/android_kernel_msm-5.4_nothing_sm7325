/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _SDE_RSC_PRIV_H_
#define _SDE_RSC_PRIV_H_

#include <linux/kernel.h>
#include <linux/sde_io_util.h>
#include <linux/sde_rsc.h>

#include <soc/qcom/tcs.h>
#include "sde_power_handle.h"

#define SDE_RSC_COMPATIBLE "disp_rscc"

#define MAX_RSC_COUNT		5

#define ALL_MODES_DISABLED	0x0
#define ONLY_MODE_0_ENABLED	0x1
#define ONLY_MODE_0_1_ENABLED	0x3
#define ALL_MODES_ENABLED	0x7

#define MAX_COUNT_SIZE_SUPPORTED	128

#define SDE_RSC_REV_1			0x1
#define SDE_RSC_REV_2			0x2
#define SDE_RSC_REV_3			0x3
#define SDE_RSC_REV_4			0x4

#define SDE_RSC_HW_MAJOR_MINOR_STEP(major, minor, step) \
	(((major & 0xff) << 16) |\
	((minor & 0xff) << 8) | \
	(step & 0xff))

struct sde_rsc_priv;

/**
 * rsc_mode_req: sde rsc mode request information
 * MODE_READ: read vsync status
 * MODE_UPDATE: mode timeslot update
 *            0x0: all modes are disabled.
 *            0x1: Mode-0 is enabled and other two modes are disabled.
 *            0x3: Mode-0 & Mode-1 are enabled and mode-2 is disabled.
 *            0x7: all modes are enabled.
 */
enum rsc_mode_req {
	MODE_READ,
	MODE_UPDATE = 0x1,
};

/**
 * rsc_vsync_req: sde rsc vsync request information
 * VSYNC_READ: read vsync status
 * VSYNC_READ_VSYNC0: read value vsync0 timestamp (cast to int from u32)
 * VSYNC_ENABLE: enable rsc wrapper vsync status
 * VSYNC_DISABLE: disable rsc wrapper vsync status
 */
enum rsc_vsync_req {
	VSYNC_READ,
	VSYNC_READ_VSYNC0,
	VSYNC_ENABLE,
	VSYNC_DISABLE,
};

/**
 * struct sde_rsc_hw_ops - sde resource state coordinator hardware ops
 * @init:			Initialize the sequencer, solver, qtimer,
				etc. hardware blocks on RSC.
 * @timer_update:		update the static wrapper time and pdc/rsc
				backoff time.
 * @tcs_wait:			Waits for TCS block OK to allow sending a
 *				TCS command.
 * @hw_vsync:			Enables the vsync on RSC block.
 * @tcs_use_ok:			set TCS set to high to allow RSC to use it.
 * @bwi_status:			It updates the BW increase/decrease status.
 * @is_amc_mode:		Check current amc mode status
 * @debug_dump:			dump debug bus registers or enable debug bus
 * @state_update:		Enable/override the solver based on rsc state
 *                              status (command/video)
 * @debug_show:			Show current debug status.
 * @mode_ctrl:			shows current mode status, mode0/1/2
 * @setup_counters:		Enable/disable RSC profiling counters
 * @get_counters:		Get current status of profiling counters
 */

struct sde_rsc_hw_ops {
	int (*init)(struct sde_rsc_priv *rsc);
	int (*timer_update)(struct sde_rsc_priv *rsc);
	int (*tcs_wait)(struct sde_rsc_priv *rsc);
	int (*hw_vsync)(struct sde_rsc_priv *rsc, enum rsc_vsync_req request,
		char *buffer, int buffer_size, u32 mode);
	int (*tcs_use_ok)(struct sde_rsc_priv *rsc);
	int (*bwi_status)(struct sde_rsc_priv *rsc, bool bw_indication);
	bool (*is_amc_mode)(struct sde_rsc_priv *rsc);
	void (*debug_dump)(struct sde_rsc_priv *rsc, u32 mux_sel);
	int (*state_update)(struct sde_rsc_priv *rsc, enum sde_rsc_state state);
	int (*debug_show)(struct seq_file *s, struct sde_rsc_priv *rsc);
	int (*mode_ctrl)(struct sde_rsc_priv *rsc, enum rsc_mode_req request,
		char *buffer, int buffer_size, u32 mode);
	int (*setup_counters)(struct sde_rsc_priv *rsc, bool enable);
	int (*get_counters)(struct sde_rsc_priv *rsc, u32 *counters);
};

/**
 * struct sde_rsc_timer_config: this is internal configuration between
 * rsc and rsc_hw API.
 *
 * @static_wakeup_time_ns:	wrapper backoff time in nano seconds
 * @rsc_backoff_time_ns:	rsc backoff time in nano seconds
 * @pdc_backoff_time_ns:	pdc backoff time in nano seconds
 * @rsc_mode_threshold_time_ns:	rsc mode threshold time in nano seconds
 * @rsc_time_slot_0_ns:		mode-0 time slot threshold in nano seconds
 * @rsc_time_slot_1_ns:		mode-1 time slot threshold in nano seconds
 * @rsc_time_slot_2_ns:		mode-2 time slot threshold in nano seconds
 *
 * @min_threshold_time_ns:	minimum time required to enter & exit mode0
 * @bwi_threshold_time_ns:	worst case time to increase the BW vote
 */
struct sde_rsc_timer_config {
	u32 static_wakeup_time_ns;

	u32 rsc_backoff_time_ns;
	u32 pdc_backoff_time_ns;
	u32 rsc_mode_threshold_time_ns;
	u32 rsc_time_slot_0_ns;
	u32 rsc_time_slot_1_ns;
	u32 rsc_time_slot_2_ns;

	u32 min_threshold_time_ns;
	u32 bwi_threshold_time_ns;
};

/**
 * struct sde_rsc_bw_config: bandwidth configuration
 *
 * @ab_vote:	Stored ab_vote for SDE_POWER_HANDLE_DBUS_ID_MAX
 * @ib_vote:	Stored ib_vote for SDE_POWER_HANDLE_DBUS_ID_MAX
 * @new_ab_vote:	ab_vote for incoming frame.
 * @new_ib_vote:	ib_vote for incoming frame.
 */
struct sde_rsc_bw_config {
	u64	ab_vote[SDE_POWER_HANDLE_DBUS_ID_MAX];
	u64	ib_vote[SDE_POWER_HANDLE_DBUS_ID_MAX];

	u64	new_ab_vote[SDE_POWER_HANDLE_DBUS_ID_MAX];
	u64	new_ib_vote[SDE_POWER_HANDLE_DBUS_ID_MAX];
};
/**
 * struct sde_rsc_priv: sde resource state coordinator(rsc) private handle
 * @version:		rsc sequence version
 * @hw_drv_ver:		rscc hw version
 * @phandle:		module power handle for clocks
 * @fs:			"MDSS GDSC" handle
 * @sw_fs_enabled:	track "MDSS GDSC" sw vote during probe
 * @need_hwinit:	rsc hw init is required for the next update
 * @hw_reinit:		rsc hw reinit support enable
 *
 * @rpmh_dev:		rpmh device node
 * @drv_io:		sde drv io data mapping
 * @wrapper_io:		wrapper io data mapping
 *
 * @client_list:	current rsc client list handle
 * @event_list:		current rsc event list handle
 * @client_lock:	current rsc client synchronization lock
 *
 * timer_config:	current rsc timer configuration
 * cmd_config:		current panel config
 * current_state:	current rsc state (video/command), solver
 *                      override/enabled.
 * vsync_source:	Interface index to provide the vsync ticks
 * update_tcs_content:	WAKE & SLEEP / AMC TCS content needs update with same BW
 *			vote if RSC state updates.
 * debug_mode:		enables the logging for each register read/write
 * debugfs_root:	debugfs file system root node
 *
 * hw_ops:		sde rsc hardware operations
 * power_collapse:	if all clients are in IDLE state then it enters in
 *			mode2 state and enable the power collapse state
 * power_collapse_block:By default, rsc move to mode-2 if all clients are in
 *			invalid state. It can be blocked by this boolean entry.
 * primary_client:	A client which is allowed to make command state request
 *			and ab/ib vote on display rsc
 * single_tcs_execution_time: worst case time to execute one tcs vote
 *			(sleep/wake)
 * backoff_time_ns:	time to only wake tcs in any mode
 * mode_threshold_time_ns: time to wake TCS in mode-0, must be greater than
 *			backoff time
 * time_slot_0_ns:	time for sleep & wake TCS in mode-1
 * rsc_vsync_wait:   Refcount to indicate if we have to wait for the vsync.
 * rsc_vsync_waitq:   Queue to wait for the vsync.
 * bw_config:		check sde_rsc_bw_config structure description.
 * dev:			rsc device node
 * resource_refcount:	Track rsc resource refcount
 * profiling_supp:	Indicates if HW has support for profiling counters
 * profiling_en:	Flag for rsc lpm profiling counters, true=enabled
 * post_poms:		bool if a panel mode change occurred
 */
struct sde_rsc_priv {
	u32 version;
	u32 hw_drv_ver;
	struct sde_power_handle phandle;
	struct regulator *fs;
	bool sw_fs_enabled;
	bool need_hwinit;
	bool hw_reinit;

	struct device *rpmh_dev;
	struct dss_io_data drv_io;
	struct dss_io_data wrapper_io;

	struct list_head client_list;
	struct list_head event_list;
	struct mutex client_lock;

	struct sde_rsc_timer_config timer_config;
	struct sde_rsc_cmd_config cmd_config;
	u32 current_state;
	u32 vsync_source;
	bool update_tcs_content;

	u32 debug_mode;
	struct dentry *debugfs_root;

	struct sde_rsc_hw_ops hw_ops;
	bool power_collapse;
	bool power_collapse_block;
	struct sde_rsc_client *primary_client;

	u32 single_tcs_execution_time;
	u32 backoff_time_ns;
	u32 mode_threshold_time_ns;
	u32 time_slot_0_ns;

	atomic_t rsc_vsync_wait;
	wait_queue_head_t rsc_vsync_waitq;

	struct sde_rsc_bw_config bw_config;
	struct device *dev;
	atomic_t resource_refcount;
	bool profiling_supp;
	bool profiling_en;

	bool post_poms;
};

/**
 * sde_rsc_hw_register() - register hardware API. It manages V1 and V2 support.
 *
 * @client:	 Client pointer provided by sde_rsc_client_create().
 *
 * Return: error code.
 */
int sde_rsc_hw_register(struct sde_rsc_priv *rsc);

/**
 * sde_rsc_hw_register_v3() - register hardware API. It manages V3 support.
 *
 * @client:	 Client pointer provided by sde_rsc_client_create().
 *
 * Return: error code.
 */
int sde_rsc_hw_register_v3(struct sde_rsc_priv *rsc);

#endif /* _SDE_RSC_PRIV_H_ */
