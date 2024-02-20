/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2020, The Linux Foundation. All rights reserved.
 */

#ifndef _SDE_HW_TOP_H
#define _SDE_HW_TOP_H

#include "sde_hw_catalog.h"
#include "sde_hw_mdss.h"
#include "sde_hw_util.h"
#include "sde_hw_blk.h"

struct sde_hw_mdp;

/**
 * struct traffic_shaper_cfg: traffic shaper configuration
 * @en        : enable/disable traffic shaper
 * @rd_client : true if read client; false if write client
 * @client_id : client identifier
 * @bpc_denom : denominator of byte per clk
 * @bpc_numer : numerator of byte per clk
 */
struct traffic_shaper_cfg {
	bool en;
	bool rd_client;
	u32 client_id;
	u32 bpc_denom;
	u64 bpc_numer;
};

/**
 * struct split_pipe_cfg - pipe configuration for dual display panels
 * @en        : Enable/disable dual pipe confguration
 * @mode      : Panel interface mode
 * @intf      : Interface id for main control path
 * @pp_split_slave: Slave interface for ping pong split, INTF_MAX to disable
 * @pp_split_idx:   Ping pong index for ping pong split
 * @split_flush_en: Allows both the paths to be flushed when master path is
 *              flushed
 * @split_link_en:  Check if split link is enabled
 */
struct split_pipe_cfg {
	bool en;
	enum sde_intf_mode mode;
	enum sde_intf intf;
	enum sde_intf pp_split_slave;
	u32 pp_split_index;
	bool split_flush_en;
	bool split_link_en;
};

/**
 * struct cdm_output_cfg: output configuration for cdm
 * @wb_en     : enable/disable writeback output
 * @intf_en   : enable/disable interface output
 */
struct cdm_output_cfg {
	bool wb_en;
	bool intf_en;
};

/**
 * struct sde_danger_safe_status: danger and safe status signals
 * @mdp: top level status
 * @sspp: source pipe status
 * @wb: writebck output status
 */
struct sde_danger_safe_status {
	u8 mdp;
	u8 sspp[SSPP_MAX];
	u8 wb[WB_MAX];
};

/**
 * struct sde_vsync_source_cfg - configure vsync source and configure the
 *                                    watchdog timers if required.
 * @pp_count: number of ping pongs active
 * @frame_rate: Display frame rate
 * @ppnumber: ping pong index array
 * @vsync_source: vsync source selection
 * @is_dummy: a dummy source of vsync selection. It must not be selected for
 *           any case other than sde rsc idle request.
 */
struct sde_vsync_source_cfg {
	u32 pp_count;
	u32 frame_rate;
	u32 ppnumber[PINGPONG_MAX];
	u32 vsync_source;
	bool is_dummy;
};

/**
 * struct sde_hw_mdp_ops - interface to the MDP TOP Hw driver functions
 * Assumption is these functions will be called after clocks are enabled.
 * @setup_split_pipe : Programs the pipe control registers
 * @setup_pp_split : Programs the pp split control registers
 * @setup_cdm_output : programs cdm control
 * @setup_traffic_shaper : programs traffic shaper control
 */
struct sde_hw_mdp_ops {
	/** setup_split_pipe() : Regsiters are not double buffered, thisk
	 * function should be called before timing control enable
	 * @mdp  : mdp top context driver
	 * @cfg  : upper and lower part of pipe configuration
	 */
	void (*setup_split_pipe)(struct sde_hw_mdp *mdp,
			struct split_pipe_cfg *p);

	/** setup_pp_split() : Configure pp split related registers
	 * @mdp  : mdp top context driver
	 * @cfg  : upper and lower part of pipe configuration
	 */
	void (*setup_pp_split)(struct sde_hw_mdp *mdp,
			struct split_pipe_cfg *cfg);

	/**
	 * setup_cdm_output() : Setup selection control of the cdm data path
	 * @mdp  : mdp top context driver
	 * @cfg  : cdm output configuration
	 */
	void (*setup_cdm_output)(struct sde_hw_mdp *mdp,
			struct cdm_output_cfg *cfg);

	/**
	 * setup_traffic_shaper() : Setup traffic shaper control
	 * @mdp  : mdp top context driver
	 * @cfg  : traffic shaper configuration
	 */
	void (*setup_traffic_shaper)(struct sde_hw_mdp *mdp,
			struct traffic_shaper_cfg *cfg);

	/**
	 * setup_clk_force_ctrl - set clock force control
	 * @mdp: mdp top context driver
	 * @clk_ctrl: clock to be controlled
	 * @enable: force on enable
	 * @return: if the clock is forced-on by this function
	 */
	bool (*setup_clk_force_ctrl)(struct sde_hw_mdp *mdp,
			enum sde_clk_ctrl_type clk_ctrl, bool enable);

	/**
	 * get_clk_ctrl_status - get clock control status
	 * @mdp: mdp top context driver
	 * @clk_ctrl: clock to be controlled
	 * @status: returns true if clock is on
	 * @return: 0 if success, otherwise return code
	 */
	int (*get_clk_ctrl_status)(struct sde_hw_mdp *mdp,
			enum sde_clk_ctrl_type clk_ctrl, bool *status);

	/**
	 * setup_dce - set DCE mux for DSC ctrl path
	 * @mdp: mdp top context driver
	 * @dce_sel: dce_mux value
	 */
	void (*setup_dce)(struct sde_hw_mdp *mdp, u32 dce_sel);

	/**
	 * get_danger_status - get danger status
	 * @mdp: mdp top context driver
	 * @status: Pointer to danger safe status
	 */
	void (*get_danger_status)(struct sde_hw_mdp *mdp,
			struct sde_danger_safe_status *status);

	/**
	 * setup_vsync_source - setup vsync source configuration details
	 * @mdp: mdp top context driver
	 * @cfg: vsync source selection configuration
	 */
	void (*setup_vsync_source)(struct sde_hw_mdp *mdp,
				struct sde_vsync_source_cfg *cfg);

	/**
	 * get_safe_status - get safe status
	 * @mdp: mdp top context driver
	 * @status: Pointer to danger safe status
	 */
	void (*get_safe_status)(struct sde_hw_mdp *mdp,
			struct sde_danger_safe_status *status);

	/**
	 * get_split_flush_status - get split flush status
	 * @mdp: mdp top context driver
	 */
	u32 (*get_split_flush_status)(struct sde_hw_mdp *mdp);

	/**
	 * reset_ubwc - reset top level UBWC configuration
	 * @mdp: mdp top context driver
	 * @m: pointer to mdss catalog data
	 */
	void (*reset_ubwc)(struct sde_hw_mdp *mdp, struct sde_mdss_cfg *m);

	/**
	 * intf_audio_select - select the external interface for audio
	 * @mdp: mdp top context driver
	 */
	void (*intf_audio_select)(struct sde_hw_mdp *mdp);

	/**
	 * set_mdp_hw_events - enable qdss hardware events for mdp
	 * @mdp: mdp top context driver
	 * @enable: enable/disable hw events
	 */
	void (*set_mdp_hw_events)(struct sde_hw_mdp *mdp, bool enable);

	/**
	 * set_cwb_ppb_cntl - select the data point for CWB
	 * @mdp: mdp top context driver
	 * @dual: indicates if dual pipe line needs to be programmed
	 * @dspp_out : true if dspp output required. LM is default tap point
	 */
	void (*set_cwb_ppb_cntl)(struct sde_hw_mdp *mdp,
			bool dual, bool dspp_out);

	/**
	 * set_hdr_plus_metadata - program the dynamic hdr metadata
	 * @mdp:     mdp top context driver
	 * @payload: pointer to payload data
	 * @len:     size of the valid data within payload
	 * @stream_id: stream ID for MST (0 or 1)
	 */
	void (*set_hdr_plus_metadata)(struct sde_hw_mdp *mdp,
			u8 *payload, u32 len, u32 stream_id);

	/**
	 * get_autorefresh_status - get autorefresh status
	 * @mdp:     mdp top context driver
	 * @intf_idx:  intf block index for relative information
	 */
	u32 (*get_autorefresh_status)(struct sde_hw_mdp *mdp,
			u32 intf_idx);
};

struct sde_hw_mdp {
	struct sde_hw_blk base;
	struct sde_hw_blk_reg_map hw;

	/* top */
	enum sde_mdp idx;
	const struct sde_mdp_cfg *caps;

	/* ops */
	struct sde_hw_mdp_ops ops;
};

struct sde_hw_sid {
	/* rotator base */
	struct sde_hw_blk_reg_map hw;
};

/**
 * sde_hw_sid_init - initialize the sid blk reg map
 * @addr: Mapped register io address
 * @sid_len: Length of block
 * @m: Pointer to mdss catalog data
 */
struct sde_hw_sid *sde_hw_sid_init(void __iomem *addr,
		u32 sid_len, const struct sde_mdss_cfg *m);

/**
 * sde_hw_set_rotator_sid - set sid values for rotator
 * sid: sde_hw_sid passed from kms
 */
void sde_hw_set_rotator_sid(struct sde_hw_sid *sid);

/**
 * sde_hw_set_sspp_sid - set sid values for the pipes
 * sid: sde_hw_sid passed from kms
 * pipe: sspp id
 * vm: vm id to set for SIDs
 */
void sde_hw_set_sspp_sid(struct sde_hw_sid *sid, u32 pipe, u32 vm);

/**
 * sde_hw_set_lutdma_sid - set sid values for the pipes
 * sid: sde_hw_sid passed from kms
 * vm: vm id to set for SIDs
 */
void sde_hw_set_lutdma_sid(struct sde_hw_sid *sid, u32 vm);

/**
 * to_sde_hw_mdp - convert base object sde_hw_base to container
 * @hw: Pointer to base hardware block
 * return: Pointer to hardware block container
 */
static inline struct sde_hw_mdp *to_sde_hw_mdp(struct sde_hw_blk *hw)
{
	return container_of(hw, struct sde_hw_mdp, base);
}

/**
 * sde_hw_mdptop_init - initializes the top driver for the passed idx
 * @idx:  Interface index for which driver object is required
 * @addr: Mapped register io address of MDP
 * @m:    Pointer to mdss catalog data
 */
struct sde_hw_mdp *sde_hw_mdptop_init(enum sde_mdp idx,
		void __iomem *addr,
		const struct sde_mdss_cfg *m);

void sde_hw_mdp_destroy(struct sde_hw_mdp *mdp);

#endif /*_SDE_HW_TOP_H */
