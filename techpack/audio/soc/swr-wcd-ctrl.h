/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
 */

#ifndef _SWR_WCD_CTRL_H
#define _SWR_WCD_CTRL_H
#include <linux/module.h>
#include <soc/swr-wcd.h>

#define SWR_MAX_ROW		0 /* Rows = 48 */
#define SWR_MAX_COL		7 /* Cols = 16 */
#define SWR_MIN_COL		0 /* Cols = 2 */

#define SWR_WCD_NAME	"swr-wcd"

#define SWR_MSTR_PORT_LEN	8 /* Number of master ports */

#define SWR_MAX_SLAVE_DEVICES 11

#define SWRM_VERSION_1_0 0x01010000
#define SWRM_VERSION_1_2 0x01030000
#define SWRM_VERSION_1_3 0x01040000

enum {
	SWR_MSTR_PAUSE,
	SWR_MSTR_RESUME,
	SWR_MSTR_UP,
	SWR_MSTR_DOWN,
	SWR_MSTR_SSR,
};

enum {
	SWR_IRQ_FREE,
	SWR_IRQ_REGISTER,
};

enum {
	SWR_DAC_PORT,
	SWR_COMP_PORT,
	SWR_BOOST_PORT,
	SWR_VISENSE_PORT,
};

struct usecase {
	u8 num_port;
	u8 num_ch;
	u32 chrate;
};

struct port_params {
	u8 si;
	u8 off1;
	u8 off2;
};

struct swrm_mports {
	struct list_head list;
	u8 id;
};

struct swr_ctrl_platform_data {
	void *handle; /* holds priv data */
	int (*read)(void *handle, int reg);
	int (*write)(void *handle, int reg, int val);
	int (*bulk_write)(void *handle, u32 *reg, u32 *val, size_t len);
	int (*clk)(void *handle, bool enable);
	int (*reg_irq)(void *handle, irqreturn_t(*irq_handler)(int irq,
			void *data), void *swr_handle, int type);
};

struct swr_mstr_ctrl {
	struct swr_master master;
	struct device *dev;
	struct resource *supplies;
	struct clk *mclk;
	int clk_ref_count;
	struct completion reset;
	struct completion broadcast;
	struct mutex mlock;
	struct mutex reslock;
	u8 rcmd_id;
	u8 wcmd_id;
	void *handle; /* SWR Master handle from client for read and writes */
	int (*read)(void *handle, int reg);
	int (*write)(void *handle, int reg, int val);
	int (*bulk_write)(void *handle, u32 *reg, u32 *val, size_t len);
	int (*clk)(void *handle, bool enable);
	int (*reg_irq)(void *handle, irqreturn_t(*irq_handler)(int irq,
			void *data), void *swr_handle, int type);
	int irq;
	int version;
	u32 num_dev;
	int num_enum_slaves;
	int slave_status;
	struct swr_mstr_port *mstr_port;
	struct list_head mport_list;
	int state;
	struct platform_device *pdev;
	int num_rx_chs;
	u8 num_cfg_devs;
	struct mutex force_down_lock;
	int force_down_state;

	struct notifier_block event_notifier;
	struct work_struct dc_presence_work;
};

#endif /* _SWR_WCD_CTRL_H */
