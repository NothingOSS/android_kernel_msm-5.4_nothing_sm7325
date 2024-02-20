/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */

#ifndef __CAM_SYNC_API_H__
#define __CAM_SYNC_API_H__

#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/completion.h>
#include <linux/videodev2.h>
#include <media/cam_sync.h>

#define SYNC_DEBUG_NAME_LEN 63
typedef void (*sync_callback)(int32_t sync_obj, int status, void *data);

/* Kernel APIs */

/**
 * @brief: Creates a sync object
 *
 *  The newly created sync obj is assigned to sync_obj.
 *  sync object.
 *
 * @param sync_obj   : Pointer to int referencing the sync object.
 * @param name : Optional parameter associating a name with the sync object for
 * debug purposes. Only first SYNC_DEBUG_NAME_LEN bytes are accepted,
 * rest will be ignored.
 *
 * @return Status of operation. Zero in case of success.
 * -EINVAL will be returned if sync_obj is an invalid pointer.
 * -ENOMEM will be returned if the kernel can't allocate space for
 * sync object.
 */
int cam_sync_create(int32_t *sync_obj, const char *name);

/**
 * @brief: Registers a callback with a sync object
 *
 * @param cb_func:  Pointer to callback to be registered
 * @param userdata: Opaque pointer which will be passed back with callback.
 * @param sync_obj: int referencing the sync object.
 *
 * @return Status of operation. Zero in case of success.
 * -EINVAL will be returned if userdata is invalid.
 * -ENOMEM will be returned if cb_func is invalid.
 *
 */
int cam_sync_register_callback(sync_callback cb_func,
	void *userdata, int32_t sync_obj);

/**
 * @brief: De-registers a callback with a sync object
 *
 * @param cb_func:  Pointer to callback to be de-registered
 * @param userdata: Opaque pointer which will be passed back with callback.
 * @param sync_obj: int referencing the sync object.
 *
 * @return Status of operation. Zero in case of success.
 * -EINVAL will be returned if userdata is invalid.
 * -ENOMEM will be returned if cb_func is invalid.
 */
int cam_sync_deregister_callback(sync_callback cb_func,
	void *userdata, int32_t sync_obj);

/**
 * @brief: Signals a sync object with the status argument.
 *
 * This function will signal the sync object referenced by the sync_obj
 * parameter and when doing so, will trigger callbacks in both user space and
 * kernel. Callbacks will triggered asynchronously and their order of execution
 * is not guaranteed. The status parameter will indicate whether the entity
 * performing the signaling wants to convey an error case or a success case.
 *
 * @param sync_obj: int referencing the sync object.
 * @param status: Status of the signaling. Can be either SYNC_SIGNAL_ERROR or
 * SYNC_SIGNAL_SUCCESS.
 * @param evt_param: Event parameter
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_signal(int32_t sync_obj, uint32_t status, uint32_t evt_param);

/**
 * @brief: Merges multiple sync objects
 *
 * This function will merge multiple sync objects into a sync group.
 *
 * @param sync_obj: pointer to a block of ints to be merged
 * @param num_objs: Number of ints in the block
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_merge(int32_t *sync_obj, uint32_t num_objs, int32_t *merged_obj);

/**
 * @brief: get ref count of sync obj
 *
 * This function will increment ref count for the sync object, and the ref
 * count will be decremented when this sync object is signaled.
 *
 * @param sync_obj: sync object
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_get_obj_ref(int32_t sync_obj);

/**
 * @brief: put ref count of sync obj
 *
 * This function will decrement ref count for the sync object.
 *
 * @param sync_obj: sync object
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_put_obj_ref(int32_t sync_obj);

/**
 * @brief: Destroys a sync object
 *
 * @param sync_obj: int referencing the sync object to be destroyed
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_sync_destroy(int32_t sync_obj);

/**
 * @brief: Waits for a sync object synchronously
 *
 * Does a wait on the sync object identified by sync_obj for a maximum
 * of timeout_ms milliseconds. Must not be called from interrupt context as
 * this API can sleep. Should be called from process context only.
 *
 * @param sync_obj: int referencing the sync object to be waited upon
 * @timeout_ms sync_obj: Timeout in ms.
 *
 * @return 0 upon success, -EINVAL if sync object is in bad state or arguments
 * are invalid, -ETIMEDOUT if wait times out.
 */
int cam_sync_wait(int32_t sync_obj, uint64_t timeout_ms);

/**
 * @brief: Check if sync object is valid
 *
 * @param sync_obj: int referencing the sync object to be checked
 *
 * @return 0 upon success, -EINVAL if sync object is in bad state or arguments
 * are invalid
 */
int cam_sync_check_valid(int32_t sync_obj);

/**
 * @brief : API to register SYNC to platform framework.
 * @return struct platform_device pointer on on success, or ERR_PTR() on error.
 */
int cam_sync_init(void);

/**
 * @brief : API to remove SYNC from platform framework.
 */
void cam_sync_exit(void);
#endif /* __CAM_SYNC_API_H__ */
