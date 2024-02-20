/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 */

#ifndef __UAPI_OPE_H__
#define __UAPI_OPE_H__

#include <camera/media/cam_defs.h>
#include <camera/media/cam_cpas.h>

#define OPE_DEV_NAME_SIZE                  128

/* OPE HW TYPE */
#define OPE_HW_TYPE_OPE                     0x1
#define OPE_HW_TYPE_OPE_CDM                 0x2
#define OPE_HW_TYPE_MAX                     0x3

/* OPE Device type */
#define OPE_DEV_TYPE_OPE_RT                 0x1
#define OPE_DEV_TYPE_OPE_NRT                0x2
#define OPE_DEV_TYPE_OPE_SEMI_RT            0x3
#define OPE_DEV_TYPE_MAX                    0x4

/* OPE Input Res Ports */
#define OPE_IN_RES_FULL                     0x1
#define OPE_IN_RES_MAX                      OPE_IN_RES_FULL

/* OPE Output Res Ports */
#define OPE_OUT_RES_VIDEO                   0x1
#define OPE_OUT_RES_DISP                    0x2
#define OPE_OUT_RES_ARGB                    0x3
#define OPE_OUT_RES_STATS_RS                0x4
#define OPE_OUT_RES_STATS_IHIST             0x5
#define OPE_OUT_RES_STATS_LTM               0x6
#define OPE_OUT_RES_MAX                     OPE_OUT_RES_STATS_LTM

/* OPE packet opcodes */
#define OPE_OPCODE_CONFIG                   0x1

/* OPE Command Buffer Scope */
#define OPE_CMD_BUF_SCOPE_BATCH             0x1
#define OPE_CMD_BUF_SCOPE_FRAME             0x2
#define OPE_CMD_BUF_SCOPE_PASS              0x3
#define OPE_CMD_BUF_SCOPE_STRIPE            0x4

/* OPE Command Buffer Types */
#define OPE_CMD_BUF_TYPE_DIRECT             0x1
#define OPE_CMD_BUF_TYPE_INDIRECT           0x2

/* OPE Command Buffer Usage */
#define OPE_CMD_BUF_UMD                     0x1
#define OPE_CMD_BUF_KMD                     0x2
#define OPE_CMD_BUF_DEBUG                   0x3

/* OPE Single/Double Buffered */
#define OPE_CMD_BUF_SINGLE_BUFFERED         0x1
#define OPE_CMD_BUF_DOUBLE_BUFFERED         0x2

/* Command meta types */
#define OPE_CMD_META_GENERIC_BLOB           0x1

/* Generic blob types */
#define OPE_CMD_GENERIC_BLOB_CLK_V2         0x1

/* Stripe location */
#define OPE_STRIPE_FULL                     0x0
#define OPE_STRIPE_LEFT                     0x1
#define OPE_STRIPE_RIGHT                    0x2
#define OPE_STRIPE_MIDDLE                   0x3

#define OPE_MAX_CMD_BUFS                    64
#define OPE_MAX_IO_BUFS                     (OPE_OUT_RES_MAX + OPE_IN_RES_MAX)
#define OPE_MAX_PASS                        1
#define OPE_MAX_PLANES                      2
#define OPE_MAX_STRIPES                     64
#define OPE_MAX_BATCH_SIZE                  16

/**
 * struct ope_stripe_info - OPE stripe Info
 *
 * @offset:          Offset in Bytes
 * @x_init:          X_init
 * @stripe_location: Stripe location (OPE_STRIPE_XXX)
 * @width:           Width of a stripe
 * @height:          Height of a stripe
 * @disable_bus:     Flag to disable BUS master
 * @reserved:        Reserved
 *
 */
struct ope_stripe_info {
	__u32 offset;
	__u32 x_init;
	__u32 stripe_location;
	__u32 width;
	__u32 height;
	__u32 disable_bus;
	__u32 reserved;
};

/**
 * struct ope_io_buf_info - OPE IO buffers meta
 *
 * @direction:     Direction of a buffer of a port(Input/Output)
 * @resource_type: Port type
 * @num_planes:    Number of planes for a port
 * @pix_pattern:   Pixel pattern for raw input
 * @num_stripes:   Stripes per plane
 * @mem_handle:    Memhandles of each Input/Output Port
 * @plane_offset:  Offsets of planes
 * @length:        Length of a plane buffer
 * @plane_stride:  Plane stride
 * @height:        Height of a plane buffer
 * @format:        Format
 * @fence:         Fence of a Port
 * @stripe_info:   Stripe Info
 *
 */
struct ope_io_buf_info {
	__u32 direction;
	__u32 resource_type;
	__u32 num_planes;
	__u32 pix_pattern;
	__u32 num_stripes[OPE_MAX_PLANES];
	__u32 mem_handle[OPE_MAX_PLANES];
	__u32 plane_offset[OPE_MAX_PLANES];
	__u32 length[OPE_MAX_PLANES];
	__u32 plane_stride[OPE_MAX_PLANES];
	__u32 height[OPE_MAX_PLANES];
	__u32 format;
	__u32 fence;
	struct ope_stripe_info stripe_info[OPE_MAX_PLANES][OPE_MAX_STRIPES];
};

/**
 * struct ope_frame_set - OPE frameset
 *
 * @num_io_bufs: Number of I/O buffers
 * @reserved:    Reserved
 * @io_buf:      IO buffer info for all Input and Output ports
 *
 */
struct ope_frame_set {
	__u32 num_io_bufs;
	__u32 reserved;
	struct ope_io_buf_info io_buf[OPE_MAX_IO_BUFS];
};

/**
 * struct ope_cmd_buf_info - OPE command buffer meta
 *
 * @mem_handle:              Memory handle for command buffer
 * @offset:                  Offset of a command buffer
 * @size:                    Size of command buffer
 * @length:                  Length of a command buffer
 * @cmd_buf_scope :          Scope of a command buffer (OPE_CMD_BUF_SCOPE_XXX)
 * @type:                    Command buffer type (OPE_CMD_BUF_TYPE_XXX)
 * @cmd_buf_usage:           Usage of command buffer ( OPE_CMD_BUF_USAGE_XXX)
 * @stripe_idx:              Stripe index in a req, It is valid for SCOPE_STRIPE
 * @cmd_buf_pass_idx:        Pass index
 * @prefetch_disable:        Prefecth disable flag
 *
 */

struct ope_cmd_buf_info {
	__u32 mem_handle;
	__u32 offset;
	__u32 size;
	__u32 length;
	__u32 cmd_buf_scope;
	__u32 type;
	__u32 cmd_buf_usage;
	__u32 cmd_buf_buffered;
	__u32 stripe_idx;
	__u32 cmd_buf_pass_idx;
	__u32 prefetch_disable;
};

/**
 * struct ope_packet_payload - payload for a request
 *
 * @num_cmd_bufs:         Number of command buffers
 * @batch_size:           Batch size in HFR mode
 * @ope_cmd_buf_info:     Command buffer meta data
 * @ope_io_buf_info:      Io buffer Info
 *
 */
struct ope_frame_process {
	__u32 num_cmd_bufs[OPE_MAX_BATCH_SIZE];
	__u32 batch_size;
	struct ope_cmd_buf_info cmd_buf[OPE_MAX_BATCH_SIZE][OPE_MAX_CMD_BUFS];
	struct ope_frame_set frame_set[OPE_MAX_BATCH_SIZE];
};

/**
 * struct ope_clk_bw_request_v2 - clock and bandwidth for a request
 *
 * @budget_ns:       Time required to process frame
 * @frame_cycles:    Frame cycles needed to process the frame
 * @rt_flag:         Flag to indicate real time stream
 * @reserved:        For memory alignment
 * @axi_path:        Per path vote info for OPE
 *
 */
struct ope_clk_bw_request_v2 {
	__u64 budget_ns;
	__u32 frame_cycles;
	__u32 rt_flag;
	__u32 reserved;
	__u32 num_paths;
	struct cam_axi_per_path_bw_vote   axi_path[1];
};

/**
 * struct ope_hw_ver - Device information for OPE
 *
 * This is used to get device version info of
 * OPE, CDM and use this info in CAM_QUERY_CAP IOCTL
 *
 * @hw_type:  Hardware type for the cap info(OPE_HW_TYPE_XXX)
 * @reserved: Reserved field
 * @hw_ver:   Major, minor and incr values of a hardware version
 *
 */
struct ope_hw_ver {
	__u32 hw_type;
	__u32 reserved;
	struct cam_hw_version hw_ver;
};

/**
 * struct ope_query_cap_cmd - OPE query device capability payload
 *
 * @dev_iommu_handle: OPE iommu handles for secure/non secure modes
 * @cdm_iommu_handle: CDM iommu handles for secure/non secure modes
 * @num_ope:          Number of OPEs
 * @reserved:         Reserved Parameter
 * @hw_ver:           Hardware capability array
 */
struct ope_query_cap_cmd {
	struct cam_iommu_handle dev_iommu_handle;
	struct cam_iommu_handle cdm_iommu_handle;
	__u32 num_ope;
	__u32 reserved;
	struct ope_hw_ver hw_ver[OPE_DEV_TYPE_MAX];
};

/**
 * struct ope_out_res_info - OPE Output resource info
 *
 * @res_id:            Resource ID
 * @format:            Output resource format
 * @width:             Output width
 * @height:            Output Height
 * @alignment:         Alignment
 * @packer_format:     Packer format
 * @subsample_period:  Subsample period in HFR
 * @subsample_pattern: Subsample pattern in HFR
 * @pixel_pattern:     Pixel pattern
 * @reserved:          Reserved Parameter
 *
 */
struct ope_out_res_info {
	__u32 res_id;
	__u32 format;
	__u32 width;
	__u32 height;
	__u32 alignment;
	__u32 packer_format;
	__u32 subsample_period;
	__u32 subsample_pattern;
	__u32 pixel_pattern;
	__u32 reserved;
};

/**
 * struct ope_in_res_info - OPE Input resource info
 *
 * @res_id:           Resource ID
 * @format:           Input resource format
 * @width:            Input width
 * @height:           Input Height
 * @pixel_pattern:    Pixel pattern
 * @alignment:        Alignment
 * @unpacker_format:  Unpacker format
 * @max_stripe_size:  Max stripe size supported for this instance configuration
 * @fps:              Frames per second
 * @reserved:         Reserved Parameter
 *
 */
struct ope_in_res_info {
	__u32 res_id;
	__u32 format;
	__u32 width;
	__u32 height;
	__u32 pixel_pattern;
	__u32 alignment;
	__u32 unpacker_format;
	__u32 max_stripe_size;
	__u32 fps;
	__u32 reserved;
};

/**
 * struct ope_acquire_dev_info - OPE Acquire Info
 *
 * @hw_type:             OPE HW Types (OPE)
 * @dev_type:            Nature of Device Instance (RT/NRT)
 * @dev_name:            Name of Device Instance
 * @nrt_stripes_for_arb: Program num stripes in OPE CDM for NRT device
 *                       before setting ARB bit
 * @secure_mode:         Mode of Device operation (Secure or Non Secure)
 * @batch_size:          Batch size
 * @num_in_res:          Number of input resources (OPE_IN_RES_XXX)
 * @in_res:              Input resource info
 * @num_out_res:         Number of output resources (OPE_OUT_RES_XXX)
 * @reserved:            Reserved Parameter
 * @out_res:             Output resource info
 *
 */
struct ope_acquire_dev_info {
	__u32                   hw_type;
	__u32                   dev_type;
	char                    dev_name[OPE_DEV_NAME_SIZE];
	__u32                   nrt_stripes_for_arb;
	__u32                   secure_mode;
	__u32                   batch_size;
	__u32                   num_in_res;
	struct ope_in_res_info  in_res[OPE_IN_RES_MAX];
	__u32                   num_out_res;
	__u32                   reserved;
	struct ope_out_res_info out_res[OPE_OUT_RES_MAX];
} __attribute__((__packed__));

#endif /* __UAPI_OPE_H__ */
