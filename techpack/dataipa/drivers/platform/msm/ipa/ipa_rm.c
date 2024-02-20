// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/ipa.h>
#include "ipa_rm_dependency_graph.h"
#include "ipa_rm_i.h"
#include "ipa_common_i.h"

static const char *resource_name_to_str[IPA_RM_RESOURCE_MAX] = {
	__stringify(IPA_RM_RESOURCE_Q6_PROD),
	__stringify(IPA_RM_RESOURCE_Q6_CONS),
	__stringify(IPA_RM_RESOURCE_USB_PROD),
	__stringify(IPA_RM_RESOURCE_USB_CONS),
	__stringify(IPA_RM_RESOURCE_USB_DPL_DUMMY_PROD),
	__stringify(IPA_RM_RESOURCE_USB_DPL_CONS),
	__stringify(IPA_RM_RESOURCE_HSIC_PROD),
	__stringify(IPA_RM_RESOURCE_HSIC_CONS),
	__stringify(IPA_RM_RESOURCE_STD_ECM_PROD),
	__stringify(IPA_RM_RESOURCE_APPS_CONS),
	__stringify(IPA_RM_RESOURCE_RNDIS_PROD),
	__stringify(RESERVED_CONS_11),
	__stringify(IPA_RM_RESOURCE_WWAN_0_PROD),
	__stringify(RESERVED_CONS_13),
	__stringify(IPA_RM_RESOURCE_WLAN_PROD),
	__stringify(IPA_RM_RESOURCE_WLAN_CONS),
	__stringify(IPA_RM_RESOURCE_ODU_ADAPT_PROD),
	__stringify(IPA_RM_RESOURCE_ODU_ADAPT_CONS),
	__stringify(IPA_RM_RESOURCE_MHI_PROD),
	__stringify(IPA_RM_RESOURCE_MHI_CONS),
	__stringify(IPA_RM_RESOURCE_ETHERNET_PROD),
	__stringify(IPA_RM_RESOURCE_ETHERNET_CONS),
};

struct ipa_rm_profile_vote_type {
	enum ipa_voltage_level volt[IPA_RM_RESOURCE_MAX];
	enum ipa_voltage_level curr_volt;
	u32 bw_resources[IPA_RM_RESOURCE_MAX];
	u32 curr_bw;
};

struct ipa_rm_context_type {
	struct ipa_rm_dep_graph *dep_graph;
	struct workqueue_struct *ipa_rm_wq;
	spinlock_t ipa_rm_lock;
	struct ipa_rm_profile_vote_type prof_vote;
};
static struct ipa_rm_context_type *ipa_rm_ctx;

struct ipa_rm_notify_ipa_work_type {
	struct work_struct		work;
	enum ipa_voltage_level		volt;
	u32				bandwidth_mbps;
};

static int _ipa_rm_add_dependency(enum ipa_rm_resource_name resource_name,
			enum ipa_rm_resource_name depends_on_name,
			bool userspace_dep)
{
	unsigned long flags;
	int result;

	if (unlikely(!ipa_rm_ctx)) {
		IPA_RM_ERR("IPA RM was not initialized\n");
		return -EINVAL;
	}

	IPA_RM_DBG("%s -> %s\n", ipa_rm_resource_str(resource_name),
				 ipa_rm_resource_str(depends_on_name));
	spin_lock_irqsave(&ipa_rm_ctx->ipa_rm_lock, flags);
	result = ipa_rm_dep_graph_add_dependency(
						ipa_rm_ctx->dep_graph,
						resource_name,
						depends_on_name,
						userspace_dep);
	spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
	IPA_RM_DBG("EXIT with %d\n", result);

	return result;
}

/**
 * ipa_rm_add_dependency_from_ioctl() - create dependency between 2 resources
 * @resource_name: name of dependent resource
 * @depends_on_name: name of its dependency
 *
 * This function is expected to be called from IOCTL and the dependency will be
 * marked as is was added by the userspace.
 *
 * Returns: 0 on success, negative on failure
 *
 * Side effects: IPA_RM_RESORCE_GRANTED could be generated
 * in case client registered with IPA RM
 */
int ipa_rm_add_dependency_from_ioctl(enum ipa_rm_resource_name resource_name,
			enum ipa_rm_resource_name depends_on_name)
{
	return _ipa_rm_add_dependency(resource_name, depends_on_name, true);
}

static int _ipa_rm_add_dependency_sync(enum ipa_rm_resource_name resource_name,
		enum ipa_rm_resource_name depends_on_name,
		bool userspsace_dep)
{
	int result;
	struct ipa_rm_resource *consumer;
	unsigned long time;
	unsigned long flags;

	if (unlikely(!ipa_rm_ctx)) {
		IPA_RM_ERR("IPA RM was not initialized\n");
		return -EINVAL;
	}

	IPA_RM_DBG("%s -> %s\n", ipa_rm_resource_str(resource_name),
				 ipa_rm_resource_str(depends_on_name));
	spin_lock_irqsave(&ipa_rm_ctx->ipa_rm_lock, flags);
	result = ipa_rm_dep_graph_add_dependency(
						ipa_rm_ctx->dep_graph,
						resource_name,
						depends_on_name,
						userspsace_dep);
	spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
	if (result == -EINPROGRESS) {
		ipa_rm_dep_graph_get_resource(ipa_rm_ctx->dep_graph,
				depends_on_name,
				&consumer);
		IPA_RM_DBG("%s waits for GRANT of %s.\n",
				ipa_rm_resource_str(resource_name),
				ipa_rm_resource_str(depends_on_name));
		time = wait_for_completion_timeout(
				&((struct ipa_rm_resource_cons *)consumer)->
				request_consumer_in_progress,
				HZ * 5);
		result = 0;
		if (!time) {
			IPA_RM_ERR("TIMEOUT waiting for %s GRANT event.",
					ipa_rm_resource_str(depends_on_name));
			result = -ETIME;
		} else {
			IPA_RM_DBG("%s waited for %s GRANT %lu time.\n",
				ipa_rm_resource_str(resource_name),
				ipa_rm_resource_str(depends_on_name),
				time);
		}
	}
	IPA_RM_DBG("EXIT with %d\n", result);

	return result;
}

/**
 * ipa_rm_add_dependency_sync_from_ioctl() - Create a dependency between 2
 * resources in a synchronized fashion. In case a producer resource is in
 * GRANTED state and the newly added consumer resource is in RELEASED state,
 * the consumer entity will be requested and the function will block until
 * the consumer is granted.
 * @resource_name: name of dependent resource
 * @depends_on_name: name of its dependency
 *
 * Returns: 0 on success, negative on failure
 *
 * Side effects: May block. See documentation above.
 */
int ipa_rm_add_dependency_sync_from_ioctl(
	enum ipa_rm_resource_name resource_name,
	enum ipa_rm_resource_name depends_on_name)
{
	return _ipa_rm_add_dependency_sync(resource_name, depends_on_name,
		true);
}

static int _ipa_rm_delete_dependency(enum ipa_rm_resource_name resource_name,
			enum ipa_rm_resource_name depends_on_name,
			bool userspace_dep)
{
	unsigned long flags;
	int result;

	if (unlikely(!ipa_rm_ctx)) {
		IPA_RM_ERR("IPA RM was not initialized\n");
		return -EINVAL;
	}

	IPA_RM_DBG("%s -> %s\n", ipa_rm_resource_str(resource_name),
				 ipa_rm_resource_str(depends_on_name));
	spin_lock_irqsave(&ipa_rm_ctx->ipa_rm_lock, flags);
	result = ipa_rm_dep_graph_delete_dependency(
			  ipa_rm_ctx->dep_graph,
			  resource_name,
			  depends_on_name,
			  userspace_dep);
	spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
	IPA_RM_DBG("EXIT with %d\n", result);

	return result;
}

/**
 * ipa_rm_delete_dependency_fron_ioctl() - delete dependency between 2 resources
 * @resource_name: name of dependent resource
 * @depends_on_name: name of its dependency
 *
 * This function is expected to be called from IOCTL and the dependency will be
 * marked as is was added by the userspace.
 *
 * Returns: 0 on success, negative on failure
 *
 * Side effects: IPA_RM_RESORCE_GRANTED could be generated
 * in case client registered with IPA RM
 */
int ipa_rm_delete_dependency_from_ioctl(enum ipa_rm_resource_name resource_name,
			enum ipa_rm_resource_name depends_on_name)
{
	return _ipa_rm_delete_dependency(resource_name, depends_on_name, true);
}

void delayed_release_work_func(struct work_struct *work)
{
	unsigned long flags;
	struct ipa_rm_resource *resource;
	struct ipa_rm_delayed_release_work_type *rwork = container_of(
			to_delayed_work(work),
			struct ipa_rm_delayed_release_work_type,
			work);

	if (!IPA_RM_RESORCE_IS_CONS(rwork->resource_name)) {
		IPA_RM_ERR("can be called on CONS only\n");
		kfree(rwork);
		return;
	}
	spin_lock_irqsave(&ipa_rm_ctx->ipa_rm_lock, flags);
	if (ipa_rm_dep_graph_get_resource(ipa_rm_ctx->dep_graph,
					rwork->resource_name,
					&resource) != 0) {
		IPA_RM_ERR("resource does not exists\n");
		goto bail;
	}

	ipa_rm_resource_consumer_release(
		(struct ipa_rm_resource_cons *)resource, rwork->needed_bw,
		rwork->dec_usage_count);

bail:
	spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
	kfree(rwork);

}

/**
 * ipa_rm_request_resource_with_timer() - requests the specified consumer
 * resource and releases it after 1 second
 * @resource_name: name of the requested resource
 *
 * Returns: 0 on success, negative on failure
 */
int ipa_rm_request_resource_with_timer(enum ipa_rm_resource_name resource_name)
{
	unsigned long flags;
	struct ipa_rm_resource *resource;
	struct ipa_rm_delayed_release_work_type *release_work;
	int result;

	if (!IPA_RM_RESORCE_IS_CONS(resource_name)) {
		IPA_RM_ERR("can be called on CONS only\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&ipa_rm_ctx->ipa_rm_lock, flags);
	if (ipa_rm_dep_graph_get_resource(ipa_rm_ctx->dep_graph,
			resource_name,
			&resource) != 0) {
		IPA_RM_ERR("resource does not exists\n");
		result = -EPERM;
		goto bail;
	}
	result = ipa_rm_resource_consumer_request(
		(struct ipa_rm_resource_cons *)resource, 0, false, true);
	if (result != 0 && result != -EINPROGRESS) {
		IPA_RM_ERR("consumer request returned error %d\n", result);
		result = -EPERM;
		goto bail;
	}

	release_work = kzalloc(sizeof(*release_work), GFP_ATOMIC);
	if (!release_work) {
		result = -ENOMEM;
		goto bail;
	}
	release_work->resource_name = resource->name;
	release_work->needed_bw = 0;
	release_work->dec_usage_count = false;
	INIT_DELAYED_WORK(&release_work->work, delayed_release_work_func);
	schedule_delayed_work(&release_work->work,
			msecs_to_jiffies(IPA_RM_RELEASE_DELAY_IN_MSEC));
	result = 0;
bail:
	spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);

	return result;
}

static void ipa_rm_wq_handler(struct work_struct *work)
{
	unsigned long flags;
	struct ipa_rm_resource *resource;
	struct ipa_rm_wq_work_type *ipa_rm_work =
			container_of(work,
					struct ipa_rm_wq_work_type,
					work);
	IPA_RM_DBG_LOW("%s cmd=%d event=%d notify_registered_only=%d\n",
		ipa_rm_resource_str(ipa_rm_work->resource_name),
		ipa_rm_work->wq_cmd,
		ipa_rm_work->event,
		ipa_rm_work->notify_registered_only);
	switch (ipa_rm_work->wq_cmd) {
	case IPA_RM_WQ_NOTIFY_PROD:
		if (!IPA_RM_RESORCE_IS_PROD(ipa_rm_work->resource_name)) {
			IPA_RM_ERR("resource is not PROD\n");
			goto free_work;
		}
		spin_lock_irqsave(&ipa_rm_ctx->ipa_rm_lock, flags);
		if (ipa_rm_dep_graph_get_resource(ipa_rm_ctx->dep_graph,
						ipa_rm_work->resource_name,
						&resource) != 0){
			IPA_RM_ERR("resource does not exists\n");
			spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
			goto free_work;
		}
		ipa_rm_resource_producer_notify_clients(
				(struct ipa_rm_resource_prod *)resource,
				ipa_rm_work->event,
				ipa_rm_work->notify_registered_only);
		spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
		break;
	case IPA_RM_WQ_NOTIFY_CONS:
		break;
	case IPA_RM_WQ_RESOURCE_CB:
		spin_lock_irqsave(&ipa_rm_ctx->ipa_rm_lock, flags);
		if (ipa_rm_dep_graph_get_resource(ipa_rm_ctx->dep_graph,
						ipa_rm_work->resource_name,
						&resource) != 0){
			IPA_RM_ERR("resource does not exists\n");
			spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
			goto free_work;
		}
		ipa_rm_resource_consumer_handle_cb(
				(struct ipa_rm_resource_cons *)resource,
				ipa_rm_work->event);
		spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
		break;
	default:
		break;
	}

free_work:
	kfree((void *) work);
}

static void ipa_rm_wq_resume_handler(struct work_struct *work)
{
	unsigned long flags;
	struct ipa_rm_resource *resource;
	struct ipa_rm_wq_suspend_resume_work_type *ipa_rm_work =
			container_of(work,
			struct ipa_rm_wq_suspend_resume_work_type,
			work);
		IPA_RM_DBG_LOW("resume work handler: %s",
		ipa_rm_resource_str(ipa_rm_work->resource_name));

	if (!IPA_RM_RESORCE_IS_CONS(ipa_rm_work->resource_name)) {
		IPA_RM_ERR("resource is not CONS\n");
		return;
	}
	IPA_ACTIVE_CLIENTS_INC_RESOURCE(ipa_rm_resource_str(
			ipa_rm_work->resource_name));
	spin_lock_irqsave(&ipa_rm_ctx->ipa_rm_lock, flags);
	if (ipa_rm_dep_graph_get_resource(ipa_rm_ctx->dep_graph,
					ipa_rm_work->resource_name,
					&resource) != 0){
		IPA_RM_ERR("resource does not exists\n");
		spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
		IPA_ACTIVE_CLIENTS_DEC_RESOURCE(ipa_rm_resource_str(
				ipa_rm_work->resource_name));
		goto bail;
	}
	ipa_rm_resource_consumer_request_work(
			(struct ipa_rm_resource_cons *)resource,
			ipa_rm_work->prev_state, ipa_rm_work->needed_bw, true,
			ipa_rm_work->inc_usage_count);
	spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
bail:
	kfree(ipa_rm_work);
}


static void ipa_rm_wq_suspend_handler(struct work_struct *work)
{
	unsigned long flags;
	struct ipa_rm_resource *resource;
	struct ipa_rm_wq_suspend_resume_work_type *ipa_rm_work =
			container_of(work,
			struct ipa_rm_wq_suspend_resume_work_type,
			work);
		IPA_RM_DBG_LOW("suspend work handler: %s",
		ipa_rm_resource_str(ipa_rm_work->resource_name));

	if (!IPA_RM_RESORCE_IS_CONS(ipa_rm_work->resource_name)) {
		IPA_RM_ERR("resource is not CONS\n");
		return;
	}
	ipa3_suspend_resource_sync(ipa_rm_work->resource_name);
	spin_lock_irqsave(&ipa_rm_ctx->ipa_rm_lock, flags);
	if (ipa_rm_dep_graph_get_resource(ipa_rm_ctx->dep_graph,
					ipa_rm_work->resource_name,
					&resource) != 0){
		IPA_RM_ERR("resource does not exists\n");
		spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);
		return;
	}
	ipa_rm_resource_consumer_release_work(
			(struct ipa_rm_resource_cons *)resource,
			ipa_rm_work->prev_state,
			true);
	spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);

	kfree(ipa_rm_work);
}

/**
 * ipa_rm_wq_send_cmd() - send a command for deferred work
 * @wq_cmd: command that should be executed
 * @resource_name: resource on which command should be executed
 * @notify_registered_only: notify only clients registered by
 *	ipa_rm_register()
 *
 * Returns: 0 on success, negative otherwise
 */
int ipa_rm_wq_send_cmd(enum ipa_rm_wq_cmd wq_cmd,
		enum ipa_rm_resource_name resource_name,
		enum ipa_rm_event event,
		bool notify_registered_only)
{
	int result = -ENOMEM;
	struct ipa_rm_wq_work_type *work = kzalloc(sizeof(*work), GFP_ATOMIC);

	if (work) {
		INIT_WORK((struct work_struct *)work, ipa_rm_wq_handler);
		work->wq_cmd = wq_cmd;
		work->resource_name = resource_name;
		work->event = event;
		work->notify_registered_only = notify_registered_only;
		result = queue_work(ipa_rm_ctx->ipa_rm_wq,
				(struct work_struct *)work);
	}

	return result;
}

int ipa_rm_wq_send_suspend_cmd(enum ipa_rm_resource_name resource_name,
		enum ipa_rm_resource_state prev_state,
		u32 needed_bw)
{
	int result = -ENOMEM;
	struct ipa_rm_wq_suspend_resume_work_type *work = kzalloc(sizeof(*work),
			GFP_ATOMIC);
	if (work) {
		INIT_WORK((struct work_struct *)work,
				ipa_rm_wq_suspend_handler);
		work->resource_name = resource_name;
		work->prev_state = prev_state;
		work->needed_bw = needed_bw;
		result = queue_work(ipa_rm_ctx->ipa_rm_wq,
				(struct work_struct *)work);
	}

	return result;
}

int ipa_rm_wq_send_resume_cmd(enum ipa_rm_resource_name resource_name,
		enum ipa_rm_resource_state prev_state,
		u32 needed_bw,
		bool inc_usage_count)
{
	int result = -ENOMEM;
	struct ipa_rm_wq_suspend_resume_work_type *work = kzalloc(sizeof(*work),
			GFP_ATOMIC);
	if (work) {
		INIT_WORK((struct work_struct *)work, ipa_rm_wq_resume_handler);
		work->resource_name = resource_name;
		work->prev_state = prev_state;
		work->needed_bw = needed_bw;
		work->inc_usage_count = inc_usage_count;
		result = queue_work(ipa_rm_ctx->ipa_rm_wq,
				(struct work_struct *)work);
	} else {
		IPA_RM_ERR("no mem\n");
	}

	return result;
}
/**
 * ipa_rm_initialize() - initialize IPA RM component
 *
 * Returns: 0 on success, negative otherwise
 */
int ipa_rm_initialize(void)
{
	int result;

	ipa_rm_ctx = kzalloc(sizeof(*ipa_rm_ctx), GFP_KERNEL);
	if (!ipa_rm_ctx) {
		IPA_RM_ERR("no mem\n");
		result = -ENOMEM;
		goto bail;
	}
	ipa_rm_ctx->ipa_rm_wq = create_singlethread_workqueue("ipa_rm_wq");
	if (!ipa_rm_ctx->ipa_rm_wq) {
		IPA_RM_ERR("create workqueue failed\n");
		result = -ENOMEM;
		goto create_wq_fail;
	}
	result = ipa_rm_dep_graph_create(&(ipa_rm_ctx->dep_graph));
	if (result) {
		IPA_RM_ERR("create dependency graph failed\n");
		goto graph_alloc_fail;
	}
	spin_lock_init(&ipa_rm_ctx->ipa_rm_lock);
	IPA_RM_DBG("SUCCESS\n");

	return 0;
graph_alloc_fail:
	destroy_workqueue(ipa_rm_ctx->ipa_rm_wq);
create_wq_fail:
	kfree(ipa_rm_ctx);
bail:
	return result;
}

/**
 * ipa_rm_stat() - print RM stat
 * @buf: [in] The user buff used to print
 * @size: [in] The size of buf
 * Returns: number of bytes used on success, negative on failure
 *
 * This function is called by ipa_debugfs in order to receive
 * a full picture of the current state of the RM
 */

int ipa_rm_stat(char *buf, int size)
{
	unsigned long flags;
	int i, cnt = 0, result = EINVAL;
	struct ipa_rm_resource *resource = NULL;
	u32 sum_bw_prod = 0;
	u32 sum_bw_cons = 0;

	if (!buf || size < 0)
		return result;

	spin_lock_irqsave(&ipa_rm_ctx->ipa_rm_lock, flags);
	for (i = 0; i < IPA_RM_RESOURCE_MAX; ++i) {
		if (!IPA_RM_RESORCE_IS_PROD(i))
			continue;
		result = ipa_rm_dep_graph_get_resource(
				ipa_rm_ctx->dep_graph,
				i,
				&resource);
		if (!result) {
			result = ipa_rm_resource_producer_print_stat(
							resource, buf + cnt,
							size-cnt);
			if (result < 0)
				goto bail;
			cnt += result;
		}
	}

	for (i = 0; i < IPA_RM_RESOURCE_MAX; i++) {
		if (IPA_RM_RESORCE_IS_PROD(i))
			sum_bw_prod += ipa_rm_ctx->prof_vote.bw_resources[i];
		else
			sum_bw_cons += ipa_rm_ctx->prof_vote.bw_resources[i];
	}

	result = scnprintf(buf + cnt, size - cnt,
		"All prod bandwidth: %d, All cons bandwidth: %d\n",
		sum_bw_prod, sum_bw_cons);
	cnt += result;

	result = scnprintf(buf + cnt, size - cnt,
		"Voting: voltage %d, bandwidth %d\n",
		ipa_rm_ctx->prof_vote.curr_volt,
		ipa_rm_ctx->prof_vote.curr_bw);
	cnt += result;

	result = cnt;
bail:
	spin_unlock_irqrestore(&ipa_rm_ctx->ipa_rm_lock, flags);

	return result;
}

/**
 * ipa_rm_resource_str() - returns string that represent the resource
 * @resource_name: [in] resource name
 */
const char *ipa_rm_resource_str(enum ipa_rm_resource_name resource_name)
{
	if (resource_name < 0 || resource_name >= IPA_RM_RESOURCE_MAX)
		return "INVALID RESOURCE";

	return resource_name_to_str[resource_name];
};

static void ipa_rm_perf_profile_notify_to_ipa_work(struct work_struct *work)
{
	struct ipa_rm_notify_ipa_work_type *notify_work = container_of(work,
				struct ipa_rm_notify_ipa_work_type,
				work);
	int res;

	IPA_RM_DBG_LOW("calling to IPA driver. voltage %d bandwidth %d\n",
		notify_work->volt, notify_work->bandwidth_mbps);

	res = ipa3_set_required_perf_profile(notify_work->volt,
		notify_work->bandwidth_mbps);
	if (res) {
		IPA_RM_ERR("ipa3_set_required_perf_profile failed %d\n", res);
		goto bail;
	}

	IPA_RM_DBG_LOW("IPA driver notified\n");
bail:
	kfree(notify_work);
}

static void ipa_rm_perf_profile_notify_to_ipa(enum ipa_voltage_level volt,
					      u32 bandwidth)
{
	struct ipa_rm_notify_ipa_work_type *work;

	work = kzalloc(sizeof(*work), GFP_ATOMIC);
	if (!work)
		return;

	INIT_WORK(&work->work, ipa_rm_perf_profile_notify_to_ipa_work);
	work->volt = volt;
	work->bandwidth_mbps = bandwidth;
	queue_work(ipa_rm_ctx->ipa_rm_wq, &work->work);
}

/**
 * ipa_rm_perf_profile_change() - change performance profile vote for resource
 * @resource_name: [in] resource name
 *
 * change bandwidth and voltage vote based on resource state.
 */
void ipa_rm_perf_profile_change(enum ipa_rm_resource_name resource_name)
{
	enum ipa_voltage_level old_volt;
	u32 *bw_ptr;
	u32 old_bw;
	struct ipa_rm_resource *resource;
	int i;
	u32 sum_bw_prod = 0;
	u32 sum_bw_cons = 0;

	IPA_RM_DBG_LOW("%s\n", ipa_rm_resource_str(resource_name));

	if (ipa_rm_dep_graph_get_resource(ipa_rm_ctx->dep_graph,
					  resource_name,
					  &resource) != 0) {
		IPA_RM_ERR("resource does not exists\n");
		WARN_ON(1);
		return;
	}

	old_volt = ipa_rm_ctx->prof_vote.curr_volt;
	old_bw = ipa_rm_ctx->prof_vote.curr_bw;

	bw_ptr = &ipa_rm_ctx->prof_vote.bw_resources[resource_name];

	switch (resource->state) {
	case IPA_RM_GRANTED:
	case IPA_RM_REQUEST_IN_PROGRESS:
		IPA_RM_DBG_LOW("max_bw = %d, needed_bw = %d\n",
			resource->max_bw, resource->needed_bw);
		*bw_ptr = min(resource->max_bw, resource->needed_bw);
		ipa_rm_ctx->prof_vote.volt[resource_name] =
						resource->floor_voltage;
		break;

	case IPA_RM_RELEASE_IN_PROGRESS:
	case IPA_RM_RELEASED:
		*bw_ptr = 0;
		ipa_rm_ctx->prof_vote.volt[resource_name] = 0;
		break;

	default:
		IPA_RM_ERR("unknown state %d\n", resource->state);
		WARN_ON(1);
	return;
	}
	IPA_RM_DBG_LOW("resource bandwidth: %d voltage: %d\n", *bw_ptr,
					resource->floor_voltage);

	ipa_rm_ctx->prof_vote.curr_volt = IPA_VOLTAGE_UNSPECIFIED;
	for (i = 0; i < IPA_RM_RESOURCE_MAX; i++) {
		if (ipa_rm_ctx->prof_vote.volt[i] >
				ipa_rm_ctx->prof_vote.curr_volt) {
			ipa_rm_ctx->prof_vote.curr_volt =
				ipa_rm_ctx->prof_vote.volt[i];
		}
	}

	for (i = 0; i < IPA_RM_RESOURCE_MAX; i++) {
		if (IPA_RM_RESORCE_IS_PROD(i))
			sum_bw_prod += ipa_rm_ctx->prof_vote.bw_resources[i];
		else
			sum_bw_cons += ipa_rm_ctx->prof_vote.bw_resources[i];
	}

	IPA_RM_DBG_LOW("all prod bandwidth: %d all cons bandwidth: %d\n",
		sum_bw_prod, sum_bw_cons);
	ipa_rm_ctx->prof_vote.curr_bw = min(sum_bw_prod, sum_bw_cons);

	if (ipa_rm_ctx->prof_vote.curr_volt == old_volt &&
		ipa_rm_ctx->prof_vote.curr_bw == old_bw) {
		IPA_RM_DBG_LOW("same voting\n");
		return;
	}

	IPA_RM_DBG_LOW("new voting: voltage %d bandwidth %d\n",
		ipa_rm_ctx->prof_vote.curr_volt,
		ipa_rm_ctx->prof_vote.curr_bw);

	ipa_rm_perf_profile_notify_to_ipa(ipa_rm_ctx->prof_vote.curr_volt,
			ipa_rm_ctx->prof_vote.curr_bw);

	return;
};
/**
 * ipa_rm_exit() - free all IPA RM resources
 */
void ipa_rm_exit(void)
{
	IPA_RM_DBG("ENTER\n");
	ipa_rm_dep_graph_delete(ipa_rm_ctx->dep_graph);
	destroy_workqueue(ipa_rm_ctx->ipa_rm_wq);
	kfree(ipa_rm_ctx);
	ipa_rm_ctx = NULL;
	IPA_RM_DBG("EXIT\n");
}
