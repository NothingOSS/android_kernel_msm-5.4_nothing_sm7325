/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#ifndef _CAM_CCI_DEV_H_
#define _CAM_CCI_DEV_H_

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/irqreturn.h>
#include <linux/iommu.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <media/cam_sensor.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <cam_sensor_cmn_header.h>
#include <cam_io_util.h>
#include <cam_sensor_util.h>
#include "cam_subdev.h"
#include <cam_cpas_api.h>
#include "cam_cci_hwreg.h"
#include "cam_soc_util.h"
#include "cam_debug_util.h"
#include "cam_req_mgr_workq.h"

#define CCI_I2C_QUEUE_0_SIZE 128
#define CCI_I2C_QUEUE_1_SIZE 32
#define CCI_I2C_QUEUE_0_SIZE_V_1_2 64
#define CCI_I2C_QUEUE_1_SIZE_V_1_2 16
#define CYCLES_PER_MICRO_SEC_DEFAULT 4915
#define CCI_MAX_DELAY 1000000

#define CCI_TIMEOUT msecs_to_jiffies(1500)
#define NUM_QUEUES 2

#define MSM_CCI_WRITE_DATA_PAYLOAD_SIZE_11 11
#define BURST_MIN_FREE_SIZE 8

/* Max bytes that can be read per CCI read transaction */
#define CCI_READ_MAX 256
#define CCI_READ_MAX_V_1_2 0xE
#define CCI_I2C_READ_MAX_RETRIES 3
#define CCI_I2C_MAX_READ 20480
#define CCI_I2C_MAX_WRITE 20480
#define CCI_I2C_MAX_BYTE_COUNT 65535

#define CAMX_CCI_DEV_NAME "cam-cci-driver"

#define MAX_CCI 2

#define PRIORITY_QUEUE (QUEUE_0)
#define SYNC_QUEUE (QUEUE_1)

#define CAM_CCI_NACK_DUMP_EN      BIT(1)
#define CAM_CCI_TIMEOUT_DUMP_EN   BIT(2)

#define CCI_VERSION_1_2_9 0x10020009
#define REPORT_IDSIZE 16
enum cci_i2c_sync {
	MSM_SYNC_DISABLE,
	MSM_SYNC_ENABLE,
};

enum cam_cci_cmd_type {
	MSM_CCI_INIT,
	MSM_CCI_RELEASE,
	MSM_CCI_SET_SID,
	MSM_CCI_SET_FREQ,
	MSM_CCI_SET_SYNC_CID,
	MSM_CCI_I2C_READ,
	MSM_CCI_I2C_WRITE,
	MSM_CCI_I2C_WRITE_SEQ,
	MSM_CCI_I2C_WRITE_BURST,
	MSM_CCI_I2C_WRITE_ASYNC,
	MSM_CCI_GPIO_WRITE,
	MSM_CCI_I2C_WRITE_SYNC,
	MSM_CCI_I2C_WRITE_SYNC_BLOCK,
};

enum cci_i2c_queue_t {
	QUEUE_0,
	QUEUE_1,
	QUEUE_INVALID,
};

struct cam_cci_wait_sync_cfg {
	uint16_t cid;
	int16_t csid;
	uint16_t line;
	uint16_t delay;
};

struct cam_cci_gpio_cfg {
	uint16_t gpio_queue;
	uint16_t i2c_queue;
};

struct cam_cci_read_cfg {
	uint32_t addr;
	uint16_t addr_type;
	uint8_t *data;
	uint16_t num_byte;
	uint16_t data_type;
};

struct cam_cci_i2c_queue_info {
	uint32_t max_queue_size;
	uint32_t report_id;
	uint32_t irq_en;
	uint32_t capture_rep_data;
};

struct cam_cci_master_info {
	int32_t status;
	atomic_t q_free[NUM_QUEUES];
	uint8_t q_lock[NUM_QUEUES];
	uint8_t reset_pending;
	struct mutex mutex;
	struct completion reset_complete;
	struct completion rd_done;
	struct completion th_complete;
	struct mutex mutex_q[NUM_QUEUES];
	struct completion report_q[NUM_QUEUES];
	atomic_t done_pending[NUM_QUEUES];
	spinlock_t lock_q[NUM_QUEUES];
	struct semaphore master_sem;
	spinlock_t freq_cnt_lock;
	uint16_t freq_ref_cnt;
	bool is_initilized;
};

struct cam_cci_clk_params_t {
	uint16_t hw_thigh;
	uint16_t hw_tlow;
	uint16_t hw_tsu_sto;
	uint16_t hw_tsu_sta;
	uint16_t hw_thd_dat;
	uint16_t hw_thd_sta;
	uint16_t hw_tbuf;
	uint8_t hw_scl_stretch_en;
	uint8_t hw_trdhld;
	uint8_t hw_tsp;
	uint32_t cci_clk_src;
};

enum cam_cci_state_t {
	CCI_STATE_ENABLED,
	CCI_STATE_DISABLED,
};

/**
 * struct cci_device
 * @pdev:                       Platform device
 * @subdev:                     V4L2 sub device
 * @base:                       Base address of CCI device
 * @hw_version:                 Hardware version
 * @ref_count:                  Reference Count
 * @cci_state:                  CCI state machine
 * @num_clk:                    Number of CCI clock
 * @cci_clk:                    CCI clock structure
 * @cci_clk_info:               CCI clock information
 * @cam_cci_i2c_queue_info:     CCI queue information
 * @i2c_freq_mode:              I2C frequency of operations
 * @master_active_slave:        Number of active/connected slaves for master
 * @cci_clk_params:             CCI hw clk params
 * @cci_gpio_tbl:               CCI GPIO table
 * @cci_gpio_tbl_size:          GPIO table size
 * @cci_pinctrl:                Pinctrl structure
 * @cci_pinctrl_status:         CCI pinctrl status
 * @cci_clk_src:                CCI clk src rate
 * @cci_vreg:                   CCI regulator structure
 * @cci_reg_ptr:                CCI individual regulator structure
 * @regulator_count:            Regulator count
 * @support_seq_write:          Set this flag when sequential write is enabled
 * @write_wq:                   Work queue structure
 * @valid_sync:                 Is it a valid sync with CSID
 * @v4l2_dev_str:               V4L2 device structure
 * @cci_wait_sync_cfg:          CCI sync config
 * @cycles_per_us:              Cycles per micro sec
 * @payload_size:               CCI packet payload size
 * @irq_status1:                Store irq_status1 to be cleared after
 *                              draining FIFO buffer for burst read
 * @lock_status:                to protect changes to irq_status1
 * @is_burst_read:              Flag to determine if we are performing
 *                              a burst read operation or not
 * @irqs_disabled:              Mask for IRQs that are disabled
 * @init_mutex:                 Mutex for maintaining refcount for attached
 *                              devices to cci during init/deinit.
 * @dump_en:                    To enable the selective dump
 */
struct cci_device {
	struct v4l2_subdev subdev;
	struct cam_hw_soc_info soc_info;
	uint32_t hw_version;
	uint8_t ref_count;
	enum cam_cci_state_t cci_state;
	struct cam_cci_i2c_queue_info
		cci_i2c_queue_info[MASTER_MAX][NUM_QUEUES];
	struct cam_cci_master_info cci_master_info[MASTER_MAX];
	enum i2c_freq_mode i2c_freq_mode[MASTER_MAX];
	uint8_t master_active_slave[MASTER_MAX];
	struct cam_cci_clk_params_t cci_clk_params[I2C_MAX_MODES];
	struct msm_pinctrl_info cci_pinctrl;
	uint8_t cci_pinctrl_status;
	uint8_t support_seq_write;
	struct workqueue_struct *write_wq[MASTER_MAX];
	struct cam_cci_wait_sync_cfg cci_wait_sync_cfg;
	uint8_t valid_sync;
	struct cam_subdev v4l2_dev_str;
	uint32_t cycles_per_us;
	int32_t clk_level_index;
	uint8_t payload_size;
	char device_name[20];
	uint32_t cpas_handle;
	uint32_t irq_status1;
	spinlock_t lock_status;
	bool is_burst_read[MASTER_MAX];
	uint32_t irqs_disabled;
	struct mutex init_mutex;
	uint64_t  dump_en;
};

enum cam_cci_i2c_cmd_type {
	CCI_I2C_SET_PARAM_CMD = 1,
	CCI_I2C_WAIT_CMD,
	CCI_I2C_WAIT_SYNC_CMD,
	CCI_I2C_WAIT_GPIO_EVENT_CMD,
	CCI_I2C_TRIG_I2C_EVENT_CMD,
	CCI_I2C_LOCK_CMD,
	CCI_I2C_UNLOCK_CMD,
	CCI_I2C_REPORT_CMD,
	CCI_I2C_WRITE_CMD,
	CCI_I2C_READ_CMD,
	CCI_I2C_WRITE_DISABLE_P_CMD,
	CCI_I2C_READ_DISABLE_P_CMD,
	CCI_I2C_WRITE_CMD2,
	CCI_I2C_WRITE_CMD3,
	CCI_I2C_REPEAT_CMD,
	CCI_I2C_INVALID_CMD,
};

enum cam_cci_gpio_cmd_type {
	CCI_GPIO_SET_PARAM_CMD = 1,
	CCI_GPIO_WAIT_CMD,
	CCI_GPIO_WAIT_SYNC_CMD,
	CCI_GPIO_WAIT_GPIO_IN_EVENT_CMD,
	CCI_GPIO_WAIT_I2C_Q_TRIG_EVENT_CMD,
	CCI_GPIO_OUT_CMD,
	CCI_GPIO_TRIG_EVENT_CMD,
	CCI_GPIO_REPORT_CMD,
	CCI_GPIO_REPEAT_CMD,
	CCI_GPIO_CONTINUE_CMD,
	CCI_GPIO_INVALID_CMD,
};

struct cam_sensor_cci_client {
	struct v4l2_subdev *cci_subdev;
	uint32_t freq;
	enum i2c_freq_mode i2c_freq_mode;
	enum cci_i2c_master_t cci_i2c_master;
	uint16_t sid;
	uint16_t cid;
	uint32_t timeout;
	uint16_t retries;
	uint16_t id_map;
	uint16_t cci_device;
};

struct cam_cci_ctrl {
	int32_t status;
	struct cam_sensor_cci_client *cci_info;
	enum cam_cci_cmd_type cmd;
	union {
		struct cam_sensor_i2c_reg_setting cci_i2c_write_cfg;
		struct cam_cci_read_cfg cci_i2c_read_cfg;
		struct cam_cci_wait_sync_cfg cci_wait_sync_cfg;
		struct cam_cci_gpio_cfg gpio_cfg;
	} cfg;
};

struct cci_write_async {
	struct cci_device *cci_dev;
	struct cam_cci_ctrl c_ctrl;
	enum cci_i2c_queue_t queue;
	struct work_struct work;
	ktime_t workq_scheduled_ts;
	enum cci_i2c_sync sync_en;
};

irqreturn_t cam_cci_irq(int irq_num, void *data);

struct v4l2_subdev *cam_cci_get_subdev(int cci_dev_index);
void cam_cci_dump_registers(struct cci_device *cci_dev,
		enum cci_i2c_master_t master, enum cci_i2c_queue_t queue);

/**
 * @brief : API to register CCI hw to platform framework.
 * @return struct platform_device pointer on on success, or ERR_PTR() on error.
 */
int cam_cci_init_module(void);

/**
 * @brief : API to remove CCI Hw from platform framework.
 */
void cam_cci_exit_module(void);
#define VIDIOC_MSM_CCI_CFG \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 23, struct cam_cci_ctrl)

#endif /* _CAM_CCI_DEV_H_ */
