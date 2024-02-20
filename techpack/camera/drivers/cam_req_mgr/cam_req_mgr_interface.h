/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

#ifndef _CAM_REQ_MGR_INTERFACE_H
#define _CAM_REQ_MGR_INTERFACE_H

#include <linux/types.h>
#include <media/cam_req_mgr.h>
#include "cam_req_mgr_core_defs.h"
#include "cam_req_mgr_util.h"

struct cam_req_mgr_trigger_notify;
struct cam_req_mgr_error_notify;
struct cam_req_mgr_add_request;
struct cam_req_mgr_timer_notify;
struct cam_req_mgr_notify_stop;
struct cam_req_mgr_device_info;
struct cam_req_mgr_core_dev_link_setup;
struct cam_req_mgr_apply_request;
struct cam_req_mgr_flush_request;
struct cam_req_mgr_link_evt_data;
struct cam_req_mgr_dump_info;

/* Request Manager -- camera device driver interface */
/**
 * @brief: camera kernel drivers to cam req mgr communication
 *
 * @cam_req_mgr_notify_trigger: for device which generates trigger to inform CRM
 * @cam_req_mgr_notify_err    : device use this to inform about different errors
 * @cam_req_mgr_add_req       : to info CRM about new rqeuest received from
 *                              userspace
 * @cam_req_mgr_notify_timer  : start the timer
 */
typedef int (*cam_req_mgr_notify_trigger)(struct cam_req_mgr_trigger_notify *);
typedef int (*cam_req_mgr_notify_err)(struct cam_req_mgr_error_notify *);
typedef int (*cam_req_mgr_add_req)(struct cam_req_mgr_add_request *);
typedef int (*cam_req_mgr_notify_timer)(struct cam_req_mgr_timer_notify *);
typedef int (*cam_req_mgr_notify_stop)(struct cam_req_mgr_notify_stop *);

/**
 * @brief: cam req mgr to camera device drivers
 *
 * @cam_req_mgr_get_dev_info     : to fetch details about device linked
 * @cam_req_mgr_link_setup       : to establish link with device for a session
 * @cam_req_mgr_apply_req        : CRM asks device to apply certain request id
 * @cam_req_mgr_notify_frame_skip: CRM asks device to apply setting for
 *                                 frame skip
 * @cam_req_mgr_flush_req        : Flush or cancel request
 * cam_req_mgr_process_evt       : generic events
 * @cam_req_mgr_dump_req         : dump request
 */
typedef int (*cam_req_mgr_get_dev_info) (struct cam_req_mgr_device_info *);
typedef int (*cam_req_mgr_link_setup)(struct cam_req_mgr_core_dev_link_setup *);
typedef int (*cam_req_mgr_apply_req)(struct cam_req_mgr_apply_request *);
typedef int (*cam_req_mgr_notify_frame_skip)(
	struct cam_req_mgr_apply_request *);
typedef int (*cam_req_mgr_flush_req)(struct cam_req_mgr_flush_request *);
typedef int (*cam_req_mgr_process_evt)(struct cam_req_mgr_link_evt_data *);
typedef int (*cam_req_mgr_dump_req)(struct cam_req_mgr_dump_info *);

/**
 * @brief          : cam_req_mgr_crm_cb - func table
 *
 * @notify_trigger : payload for trigger indication event
 * @notify_err     : payload for different error occurred at device
 * @add_req        : payload to inform which device and what request is received
 * @notify_timer   : payload for timer start event
 * @notify_stop    : payload to inform stop event
 */
struct cam_req_mgr_crm_cb {
	cam_req_mgr_notify_trigger  notify_trigger;
	cam_req_mgr_notify_err      notify_err;
	cam_req_mgr_add_req         add_req;
	cam_req_mgr_notify_timer    notify_timer;
	cam_req_mgr_notify_stop     notify_stop;
};

/**
 * @brief        : cam_req_mgr_kmd_ops - func table
 *
 * @get_dev_info     : payload to fetch device details
 * @link_setup       : payload to establish link with device
 * @apply_req        : payload to apply request id on a device linked
 * @notify_frame_skip: payload to notify frame skip
 * @flush_req        : payload to flush request
 * @process_evt      : payload to generic event
 * @dump_req         : payload to dump request
 */
struct cam_req_mgr_kmd_ops {
	cam_req_mgr_get_dev_info      get_dev_info;
	cam_req_mgr_link_setup        link_setup;
	cam_req_mgr_apply_req         apply_req;
	cam_req_mgr_notify_frame_skip notify_frame_skip;
	cam_req_mgr_flush_req         flush_req;
	cam_req_mgr_process_evt       process_evt;
	cam_req_mgr_dump_req          dump_req;
};

/**
 * enum cam_pipeline_delay
 * @brief     : enumerator for different pipeline delays in camera
 *
 * @DELAY_0   : device processed settings on same frame
 * @DELAY_1   : device processed settings after 1 frame
 * @DELAY_2   : device processed settings after 2 frames
 * @DELAY_MAX : maximum supported pipeline delay
 */
enum cam_pipeline_delay {
	CAM_PIPELINE_DELAY_0,
	CAM_PIPELINE_DELAY_1,
	CAM_PIPELINE_DELAY_2,
	CAM_PIPELINE_DELAY_MAX,
};

/**
 * @CAM_TRIGGER_POINT_SOF   : Trigger point for Start Of Frame
 * @CAM_TRIGGER_POINT_EOF   : Trigger point for End Of Frame
 * @CAM_TRIGGER_MAX_POINTS  : Maximum number of trigger point
 */
#define CAM_TRIGGER_POINT_SOF     (1 << 0)
#define CAM_TRIGGER_POINT_EOF     (1 << 1)
#define CAM_TRIGGER_MAX_POINTS    2

/**
 * enum cam_req_status
 * @brief   : enumerator for request status
 *
 * @SUCCESS : device processed settings successfully
 * @FAILED  : device processed settings failed
 * @MAX     : invalid status value
 */
enum cam_req_status {
	CAM_REQ_STATUS_SUCCESS,
	CAM_REQ_STATUS_FAILED,
	CAM_REQ_STATUS_MAX,
};

/**
 * enum cam_req_mgr_device_error
 * @brief      : enumerator for different errors occurred at device
 *
 * @NOT_FOUND  : settings asked by request manager is not found
 * @BUBBLE     : device hit timing issue and is able to recover
 * @FATAL      : device is in bad shape and can not recover from error
 * @PAGE_FAULT : Page fault while accessing memory
 * @OVERFLOW   : Bus Overflow for IFE/VFE
 * @TIMEOUT    : Timeout from cci or bus.
 * @MAX        : Invalid error value
 */
enum cam_req_mgr_device_error {
	CRM_KMD_ERR_NOT_FOUND,
	CRM_KMD_ERR_BUBBLE,
	CRM_KMD_ERR_FATAL,
	CRM_KMD_ERR_PAGE_FAULT,
	CRM_KMD_ERR_OVERFLOW,
	CRM_KMD_ERR_TIMEOUT,
	CRM_KMD_ERR_STOPPED,
	CRM_KMD_ERR_MAX,
};

/**
 * enum cam_req_mgr_device_id
 * @brief       : enumerator for different devices in subsystem
 *
 * @CAM_REQ_MGR : request manager itself
 * @SENSOR      : sensor device
 * @FLASH       : LED flash or dual LED device
 * @ACTUATOR    : lens mover
 * @IFE         : Image processing device
 * @CUSTOM      : Custom HW block
 * @EXTERNAL_1  : third party device
 * @EXTERNAL_2  : third party device
 * @EXTERNAL_3  : third party device
 * @MAX         : invalid device id
 */
enum cam_req_mgr_device_id {
	CAM_REQ_MGR_DEVICE,
	CAM_REQ_MGR_DEVICE_SENSOR,
	CAM_REQ_MGR_DEVICE_FLASH,
	CAM_REQ_MGR_DEVICE_ACTUATOR,
	CAM_REQ_MGR_DEVICE_IFE,
	CAM_REQ_MGR_DEVICE_CUSTOM_HW,
	CAM_REQ_MGR_DEVICE_EXTERNAL_1,
	CAM_REQ_MGR_DEVICE_EXTERNAL_2,
	CAM_REQ_MGR_DEVICE_EXTERNAL_3,
	CAM_REQ_MGR_DEVICE_ID_MAX,
};

/* Camera device driver to Req Mgr device interface */

/**
 * enum cam_req_mgr_link_evt_type
 * @CAM_REQ_MGR_LINK_EVT_ERR             : error on the link from any of the
 *                                         connected devices
 * @CAM_REQ_MGR_LINK_EVT_PAUSE           : to pause the link
 * @CAM_REQ_MGR_LINK_EVT_RESUME          : resumes the link which was paused
 * @CAM_REQ_MGR_LINK_EVT_SOF_FREEZE      : request manager has detected an
 *                                         sof freeze
 * @CAM_REQ_MGR_LINK_EVT_MAX             : invalid event type
 */
enum cam_req_mgr_link_evt_type {
	CAM_REQ_MGR_LINK_EVT_ERR,
	CAM_REQ_MGR_LINK_EVT_PAUSE,
	CAM_REQ_MGR_LINK_EVT_RESUME,
	CAM_REQ_MGR_LINK_EVT_SOF_FREEZE,
	CAM_REQ_MGR_LINK_EVT_MAX,
};

/**
 * struct cam_req_mgr_trigger_notify
 * @link_hdl : link identifier
 * @dev_hdl  : device handle which has sent this req id
 * @frame_id : frame id for internal tracking
 * @trigger  : trigger point of this notification, CRM will send apply
 *             only to the devices which subscribe to this point.
 * @sof_timestamp_val: Captured time stamp value at sof hw event
 * @req_id   : req id which returned buf_done
 * @trigger_id: ID to differentiate between the trigger devices
 */
struct cam_req_mgr_trigger_notify {
	int32_t  link_hdl;
	int32_t  dev_hdl;
	int64_t  frame_id;
	uint32_t trigger;
	uint64_t sof_timestamp_val;
	uint64_t req_id;
	int32_t  trigger_id;
};

/**
 * struct cam_req_mgr_timer_notify
 * @link_hdl : link identifier
 * @dev_hdl  : device handle which has sent this req id
 * @state    : timer state i.e ON or OFF
 */
struct cam_req_mgr_timer_notify {
	int32_t  link_hdl;
	int32_t  dev_hdl;
	bool     state;
};

/**
 * struct cam_req_mgr_error_notify
 * @link_hdl : link identifier
 * @dev_hdl  : device handle which has sent this req id
 * @req_id   : req id which hit error
 * @frame_id : frame id for internal tracking
 * @trigger  : trigger point of this notification, CRM will send apply
 * @sof_timestamp_val : Captured time stamp value at sof hw event
 * @error    : what error device hit while processing this req
 */
struct cam_req_mgr_error_notify {
	int32_t  link_hdl;
	int32_t  dev_hdl;
	uint64_t req_id;
	int64_t  frame_id;
	uint32_t trigger;
	uint64_t sof_timestamp_val;
	enum cam_req_mgr_device_error error;
};

/**
 * struct cam_req_mgr_add_request
 * @link_hdl             : link identifier
 * @dev_hdl              : device handle which has sent this req id
 * @req_id               : req id which device is ready to process
 * @skip_at_sof          : before applying req mgr introduce bubble
 *                         by not sending request to devices. ex: IFE and Flash
 * @skip_at_eof          : before applying req mgr introduce bubble
 *                         by not sending request to devices. ex: IFE and Flash
 * @trigger_eof          : to identify that one of the device at this slot needs
 *                         to be apply at EOF
 */
struct cam_req_mgr_add_request {
	int32_t  link_hdl;
	int32_t  dev_hdl;
	uint64_t req_id;
	uint32_t skip_at_sof;
	uint32_t skip_at_eof;
	bool     trigger_eof;
};


/**
 * struct cam_req_mgr_notify_stop
 * @link_hdl             : link identifier
 *
 */
struct cam_req_mgr_notify_stop {
	int32_t  link_hdl;
};


/* CRM to KMD devices */
/**
 * struct cam_req_mgr_device_info
 * @dev_hdl : Input_param : device handle for reference
 * @name    : link link or unlink
 * @dev_id  : device id info
 * @p_delay : delay between time settings applied and take effect
 * @trigger : Trigger point for the client
 * @trigger_on : This device provides trigger
 */
struct cam_req_mgr_device_info {
	int32_t                     dev_hdl;
	char                        name[256];
	enum cam_req_mgr_device_id  dev_id;
	enum cam_pipeline_delay     p_delay;
	uint32_t                    trigger;
	bool                        trigger_on;
};

/**
 * struct cam_req_mgr_core_dev_link_setup
 * @link_enable     : link or unlink
 * @link_hdl        : link identifier
 * @dev_hdl         : device handle for reference
 * @max_delay       : max pipeline delay on this link
 * @crm_cb          : callback funcs to communicate with req mgr
 * @trigger_id      : Unique ID provided to the triggering device
 */
struct cam_req_mgr_core_dev_link_setup {
	int32_t                    link_enable;
	int32_t                    link_hdl;
	int32_t                    dev_hdl;
	enum cam_pipeline_delay    max_delay;
	struct cam_req_mgr_crm_cb *crm_cb;
	int32_t                    trigger_id;
};

/**
 * struct cam_req_mgr_apply_request
 * @link_hdl         : link identifier
 * @dev_hdl          : device handle for cross check
 * @request_id       : request id settings to apply
 * @report_if_bubble : report to crm if failure in applying
 * @trigger_point    : the trigger point of this apply
 * @re_apply         : to skip re_apply for buf_done request
 *
 */
struct cam_req_mgr_apply_request {
	int32_t    link_hdl;
	int32_t    dev_hdl;
	uint64_t   request_id;
	int32_t    report_if_bubble;
	uint32_t   trigger_point;
	bool       re_apply;
};

/**
 * struct cam_req_mgr_flush_request
 * @link_hdl    : link identifier
 * @dev_hdl     : device handle for cross check
 * @type        : cancel request type flush all or a request
 * @req_id      : request id to cancel
 *
 */
struct cam_req_mgr_flush_request {
	int32_t     link_hdl;
	int32_t     dev_hdl;
	uint32_t    type;
	uint64_t    req_id;
};

/**
 * struct cam_req_mgr_event_data
 * @link_hdl          : link handle
 * @req_id            : request id
 * @evt_type          : link event
 */
struct cam_req_mgr_link_evt_data {
	int32_t  link_hdl;
	int32_t  dev_hdl;
	uint64_t req_id;
	enum cam_req_mgr_link_evt_type evt_type;
	union {
		enum cam_req_mgr_device_error error;
	} u;
};

/**
 * struct cam_req_mgr_send_request
 * @link_hdl   : link identifier
 * @in_q       : input request queue pointer
 */
struct cam_req_mgr_send_request {
	int32_t    link_hdl;
	struct cam_req_mgr_req_queue *in_q;
};

/**
 * struct cam_req_mgr_dump_info
 * @req_id      : request id to dump
 * @offset      : offset of buffer
 * @error_type  : error type
 * @buf_handle  : buf handle
 * @link_hdl    : link identifier
 * @dev_hdl     : device handle for cross check
 *
 */
struct cam_req_mgr_dump_info {
	uint64_t    req_id;
	size_t      offset;
	uint32_t    error_type;
	uint32_t    buf_handle;
	int32_t     link_hdl;
	int32_t     dev_hdl;
};
#endif
