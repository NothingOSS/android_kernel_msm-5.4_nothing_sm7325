/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */

#ifndef _SDE_REG_DMA_H
#define _SDE_REG_DMA_H

#include "msm_drv.h"
#include "sde_hw_catalog.h"
#include "sde_hw_mdss.h"
#include "sde_hw_top.h"
#include "sde_hw_util.h"

/**
 * enum sde_reg_dma_op - defines operations supported by reg dma
 * @REG_DMA_READ: Read the histogram into buffer provided
 * @REG_DMA_WRITE: Write the reg dma configuration into MDP block
 * @REG_DMA_OP_MAX: Max operation which indicates that op is invalid
 */
enum sde_reg_dma_op {
	REG_DMA_READ,
	REG_DMA_WRITE,
	REG_DMA_OP_MAX
};

/**
 * enum sde_reg_dma_read_sel - defines the blocks for histogram read
 * @DSPP0_HIST: select dspp0
 * @DSPP1_HIST: select dspp1
 * @DSPP2_HIST: select dspp2
 * @DSPP3_HIST: select dspp3
 * @DSPP_HIST_MAX: invalid selection
 */
enum sde_reg_dma_read_sel {
	DSPP0_HIST,
	DSPP1_HIST,
	DSPP2_HIST,
	DSPP3_HIST,
	DSPP_HIST_MAX,
};

/**
 * enum sde_reg_dma_features - defines features supported by reg dma
 * @QSEED: qseed feature
 * @GAMUT: gamut feature
 * @IGC: inverse gamma correction
 * @PCC: polynomical color correction
 * @VLUT: PA vlut
 * @MEMC_SKIN: memory color skin
 * @MEMC_SKY: memory color sky
 * @MEMC_FOLIAGE: memory color foliage
 * @MEMC_PROT: memory color protect
 * @SIX_ZONE: six zone
 * @HSIC: Hue, saturation and contrast
 * @GC: gamma correction
 * @SPR_INIT: Sub pixel rendering init feature
 * @SPR_PU: Sub pixel rendering partial update feature
 * @LTM_INIT: LTM INIT
 * @LTM_ROI: LTM ROI
 * @LTM_VLUT: LTM VLUT
 * @RC_DATA: Rounded corner data
 * @DEMURA_CFG: Demura feature
 * @REG_DMA_FEATURES_MAX: invalid selection
 */
enum sde_reg_dma_features {
	QSEED,
	GAMUT,
	IGC,
	PCC,
	VLUT,
	MEMC_SKIN,
	MEMC_SKY,
	MEMC_FOLIAGE,
	MEMC_PROT,
	SIX_ZONE,
	HSIC,
	GC,
	SPR_INIT,
	SPR_PU_CFG,
	LTM_INIT,
	LTM_ROI,
	LTM_VLUT,
	RC_DATA,
	DEMURA_CFG,
	REG_DMA_FEATURES_MAX,
};

/**
 * enum sde_reg_dma_queue - defines reg dma write queue values
 * @DMA_CTL_QUEUE0: select queue0
 * @DMA_CTL_QUEUE1: select queue1
 * @DMA_CTL_QUEUE_MAX: invalid selection
 */
enum sde_reg_dma_queue {
	DMA_CTL_QUEUE0,
	DMA_CTL_QUEUE1,
	DMA_CTL_QUEUE_MAX,
};

#define LUTBUS_TABLE_SELECT_MAX 2
#define LUTBUS_IGC_TRANS_SIZE 3
#define LUTBUS_GAMUT_TRANS_SIZE 6

/**
 * enum sde_reg_dma_lutbus_block - block select values for lutbus op
 * @LUTBUS_BLOCK_IGC: select IGC block
 * @LUTBUS_BLOCK_GAMUT: select GAMUT block
 * @LUTBUS_BLOCK_MAX: invalid selection
 */
enum sde_reg_dma_lutbus_block {
	LUTBUS_BLOCK_IGC = 0,
	LUTBUS_BLOCK_GAMUT,
	LUTBUS_BLOCK_MAX,
};

/**
 * enum sde_reg_dma_trigger_mode - defines reg dma ops trigger mode
 * @WRITE_IMMEDIATE: trigger write op immediately
 * @WRITE_TRIGGER: trigger write op when sw trigger is issued
 * @READ_IMMEDIATE: trigger read op immediately
 * @READ_TRIGGER: trigger read op when sw trigger is issued
 * @TIGGER_MAX: invalid trigger selection
 */
enum sde_reg_dma_trigger_mode {
	WRITE_IMMEDIATE,
	WRITE_TRIGGER,
	READ_IMMEDIATE,
	READ_TRIGGER,
	TIGGER_MAX,
};

/**
 * enum sde_reg_dma_setup_ops - defines reg dma write configuration
 * @HW_BLK_SELECT: op for selecting the hardware block
 * @REG_SINGLE_WRITE: op for writing single register value
 *                    at the address provided
 * @REG_BLK_WRITE_SINGLE: op for writing multiple registers using auto address
 *                     increment
 * @REG_BLK_WRITE_INC: op for writing multiple registers using hw index
 *                        register
 * @REG_BLK_WRITE_MULTIPLE: op for writing hw index based registers at
 *                         non-consecutive location
 * @REG_SINGLE_MODIFY: op for modifying single register value with bitmask at
 *                    the address provided(Reg = (Reg & Mask) | Data),
 *                    broadcast feature is not supported with this opcode.
 * @REG_BLK_LUT_WRITE: op for specific faster LUT writes, currently only
 *                     supports DSPP/SSPP Gamut and DSPP IGC.
 * @REG_DMA_SETUP_OPS_MAX: invalid operation
 */
enum sde_reg_dma_setup_ops {
	HW_BLK_SELECT,
	REG_SINGLE_WRITE,
	REG_BLK_WRITE_SINGLE,
	REG_BLK_WRITE_INC,
	REG_BLK_WRITE_MULTIPLE,
	REG_SINGLE_MODIFY,
	REG_BLK_LUT_WRITE,
	REG_DMA_SETUP_OPS_MAX,
};

#define REG_DMA_BLK_MAX 32

/**
 * enum sde_reg_dma_blk - defines blocks for which reg dma op should be
 *                        performed
 * @VIG0: select vig0 block
 * @VIG1: select vig1 block
 * @VIG2: select vig2 block
 * @VIG3: select vig3 block
 * @LM0: select lm0 block
 * @LM1: select lm1 block
 * @LM2: select lm2 block
 * @LM3: select lm3 block
 * @DSPP0: select dspp0 block
 * @DSPP1: select dspp1 block
 * @DSPP2: select dspp2 block
 * @DSPP3: select dspp3 block
 * @DMA0: select dma0 block
 * @DMA1: select dma1 block
 * @DMA2: select dma2 block
 * @DMA3: select dma3 block
 * @SSPP_IGC: select sspp igc block
 * @DSPP_IGC: select dspp igc block
 * @LTM0: select LTM0 block
 * @LTM1: select LTM1 block
 * @MDSS: select mdss block
 */
enum sde_reg_dma_blk {
	VIG0  = BIT(0),
	VIG1  = BIT(1),
	VIG2  = BIT(2),
	VIG3  = BIT(3),
	LM0   = BIT(4),
	LM1   = BIT(5),
	LM2   = BIT(6),
	LM3   = BIT(7),
	DSPP0 = BIT(8),
	DSPP1 = BIT(9),
	DSPP2 = BIT(10),
	DSPP3 = BIT(11),
	DMA0  = BIT(12),
	DMA1  = BIT(13),
	DMA2  = BIT(14),
	DMA3  = BIT(15),
	SSPP_IGC = BIT(16),
	DSPP_IGC = BIT(17),
	LTM0 = BIT(18),
	LTM1 = BIT(19),
	MDSS  = BIT(31)
};

/**
 * enum sde_reg_dma_last_cmd_mode - defines enums for kick off mode.
 * @REG_DMA_WAIT4_COMP: last_command api will wait for max of 1 msec allowing
 *			reg dma trigger to complete.
 * @REG_DMA_NOWAIT: last_command api will not wait for reg dma trigger
 *		    completion.
 */
enum sde_reg_dma_last_cmd_mode {
	REG_DMA_WAIT4_COMP,
	REG_DMA_NOWAIT,
};

/**
 * struct sde_reg_dma_buffer - defines reg dma buffer structure.
 * @drm_gem_object *buf: drm gem handle for the buffer
 * @asapce : pointer to address space
 * @buffer_size: buffer size
 * @index: write pointer index
 * @iova: device address
 * @vaddr: cpu address
 * @next_op_allowed: operation allowed on the buffer
 * @ops_completed: operations completed on buffer
 */
struct sde_reg_dma_buffer {
	struct drm_gem_object *buf;
	struct msm_gem_address_space *aspace;
	u32 buffer_size;
	u32 index;
	u64 iova;
	void *vaddr;
	u32 next_op_allowed;
	u32 ops_completed;
};

/**
 * struct sde_reg_dma_setup_ops_cfg - defines structure for reg dma ops on the
 *                                    reg dma buffer.
 * @sde_reg_dma_setup_ops ops: ops to be performed
 * @sde_reg_dma_blk blk: block on which op needs to be performed
 * @sde_reg_dma_features feature: feature on which op needs to be done
 * @wrap_size: valid for REG_BLK_WRITE_MULTIPLE, indicates reg index location
 *             size
 * @inc: valid for REG_BLK_WRITE_MULTIPLE indicates whether reg index location
 *       needs an increment or decrement.
 *       0 - decrement
 *       1 - increment
 * @blk_offset: offset for blk, valid for HW_BLK_SELECT op only
 * @sde_reg_dma_buffer *dma_buf: reg dma buffer on which op needs to be
 *                                performed
 * @data: pointer to payload which has to be written into reg dma buffer for
 *        selected op.
 * @mask: mask value for REG_SINGLE_MODIFY op
 * @data_size: size of payload in data
 * @table_sel: table select value for REG_BLK_LUT_WRITE opcode
 * @block_sel: block select value for REG_BLK_LUT_WRITE opcode
 * @trans_size: transfer size for REG_BLK_LUT_WRITE opcode
 * @lut_size: lut size in terms of transfer size
 */
struct sde_reg_dma_setup_ops_cfg {
	enum sde_reg_dma_setup_ops ops;
	enum sde_reg_dma_blk blk;
	enum sde_reg_dma_features feature;
	u32 wrap_size;
	u32 inc;
	u32 blk_offset;
	struct sde_reg_dma_buffer *dma_buf;
	u32 *data;
	u32 mask;
	u32 data_size;
	u32 table_sel;
	u32 block_sel;
	u32 trans_size;
	u32 lut_size;
};

/**
 * struct sde_reg_dma_kickoff_cfg - commit reg dma buffer to hw engine
 * @ctl: ctl for which reg dma buffer needs to be committed.
 * @dma_buf: reg dma buffer with iova address and size info
 * @block_select: histogram read select
 * @trigger_mode: reg dma ops trigger mode
 * @queue_select: queue on which reg dma buffer will be submitted
 * @dma_type: DB or SB LUT DMA block selection
 * @feature: feature the provided kickoff buffer belongs to
 * @last_command: last command for this vsync
 */
struct sde_reg_dma_kickoff_cfg {
	struct sde_hw_ctl *ctl;
	enum sde_reg_dma_op op;
	struct sde_reg_dma_buffer *dma_buf;
	enum sde_reg_dma_read_sel block_select;
	enum sde_reg_dma_trigger_mode trigger_mode;
	enum sde_reg_dma_queue queue_select;
	enum sde_reg_dma_type dma_type;
	enum sde_reg_dma_features feature;
	u32 last_command;
};

/**
 * struct sde_hw_reg_dma_ops - ops supported by reg dma frame work, based on
 *                             version of reg dma appropriate ops will be
 *                             installed during driver probe.
 * @check_support: checks if reg dma is supported on this platform for a
 *                 feature
 * @setup_payload: setup reg dma buffer based on ops and payload provided by
 *                 client
 * @kick_off: submit the reg dma buffer to hw enginge
 * @reset: reset the reg dma hw enginge for a ctl
 * @alloc_reg_dma_buf: allocate reg dma buffer
 * @dealloc_reg_dma: de-allocate reg dma buffer
 * @reset_reg_dma_buf: reset the buffer to init state
 * @last_command: notify control that last command is queued
 * @last_command_sb: notify control that last command for SB LUTDMA is queued
 * @dump_regs: dump reg dma registers
 */
struct sde_hw_reg_dma_ops {
	int (*check_support)(enum sde_reg_dma_features feature,
			     enum sde_reg_dma_blk blk,
			     bool *is_supported);
	int (*setup_payload)(struct sde_reg_dma_setup_ops_cfg *cfg);
	int (*kick_off)(struct sde_reg_dma_kickoff_cfg *cfg);
	int (*reset)(struct sde_hw_ctl *ctl);
	struct sde_reg_dma_buffer* (*alloc_reg_dma_buf)(u32 size);
	int (*dealloc_reg_dma)(struct sde_reg_dma_buffer *lut_buf);
	int (*reset_reg_dma_buf)(struct sde_reg_dma_buffer *buf);
	int (*last_command)(struct sde_hw_ctl *ctl, enum sde_reg_dma_queue q,
			enum sde_reg_dma_last_cmd_mode mode);
	int (*last_command_sb)(struct sde_hw_ctl *ctl, enum sde_reg_dma_queue q,
			enum sde_reg_dma_last_cmd_mode mode);
	void (*dump_regs)(void);
};

/**
 * struct sde_hw_reg_dma - structure to hold reg dma hw info
 * @drm_dev: drm driver dev handle
 * @reg_dma_count: number of LUTDMA hw instances
 * @caps: LUTDMA hw caps on the platform
 * @ops: reg dma ops supported on the platform
 * @addr: reg dma hw block base address
 */
struct sde_hw_reg_dma {
	struct drm_device *drm_dev;
	u32 reg_dma_count;
	const struct sde_reg_dma_cfg *caps;
	struct sde_hw_reg_dma_ops ops;
	void __iomem *addr;
};

/**
 * sde_reg_dma_init() - function called to initialize reg dma during sde
 *                         drm driver probe. If reg dma is supported by sde
 *                         ops for reg dma version will be installed.
 *                         if reg dma is not supported by sde default ops will
 *                         be installed. check_support of default ops will
 *                         return false, hence the clients should fall back to
 *                         AHB programming.
 * @addr: reg dma block base address
 * @m: catalog which contains sde hw capabilities and offsets
 * @dev: drm driver device handle
 */
int sde_reg_dma_init(void __iomem *addr, struct sde_mdss_cfg *m,
		struct drm_device *dev);

/**
 * sde_reg_dma_get_ops() - singleton module, ops is returned to the clients
 *                            who call this api.
 */
struct sde_hw_reg_dma_ops *sde_reg_dma_get_ops(void);

/**
 * sde_reg_dma_deinit() - de-initialize the reg dma
 */
void sde_reg_dma_deinit(void);
#endif /* _SDE_REG_DMA_H */
