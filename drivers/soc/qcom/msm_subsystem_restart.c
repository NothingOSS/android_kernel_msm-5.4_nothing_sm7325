// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt) "subsys-restart: %s(): " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/suspend.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/sysmon.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <asm/current.h>
#include <linux/timer.h>
#define ENABLE_MODEM_RESTART
#ifdef ENABLE_MODEM_RESTART
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif
#define CREATE_TRACE_POINTS
#include <trace/events/trace_msm_ssr_event.h>

#include "peripheral-loader.h"

#define DISABLE_SSR 0x9889deed
/* If set to 0x9889deed, call to subsystem_restart_dev() returns immediately */
static uint disable_restart_work;
module_param(disable_restart_work, uint, 0644);

/* The maximum shutdown timeout is the product of MAX_LOOPS and DELAY_MS. */
#define SHUTDOWN_ACK_MAX_LOOPS	100
#define SHUTDOWN_ACK_DELAY_MS	100

#ifdef CONFIG_SETUP_SSR_NOTIF_TIMEOUTS
/* Timeout used for detection of notification hangs. In seconds.*/
#define SYSMON_COMM_TIMEOUT	CONFIG_SSR_SYSMON_NOTIF_TIMEOUT
#define SUBSYS_NOTIF_TIMEOUT	CONFIG_SSR_SUBSYS_NOTIF_TIMEOUT

#define setup_timeout(dest_ss, source_ss, comm_type) \
	_setup_timeout(dest_ss, source_ss, comm_type)
#define cancel_timeout(subsys) del_timer_sync(&subsys->timeout_data.timer)
#define init_subsys_timer(subsys) _init_subsys_timer(subsys)

/* Timeout values */
static unsigned long timeout_vals[NUM_SSR_COMMS] = {
	[SUBSYS_TO_SUBSYS_SYSMON] = SYSMON_COMM_TIMEOUT,
	[SUBSYS_TO_HLOS] = SUBSYS_NOTIF_TIMEOUT,
	[HLOS_TO_SUBSYS_SYSMON_SHUTDOWN] = SYSMON_COMM_TIMEOUT,
};

#ifdef CONFIG_PANIC_ON_SSR_NOTIF_TIMEOUT
#define SSR_NOTIF_TIMEOUT_WARN(fmt...) panic(fmt)
#else /* CONFIG_PANIC_ON_SSR_NOTIF_TIMEOUT */
#define SSR_NOTIF_TIMEOUT_WARN(fmt...) WARN(1, fmt)
#endif /* CONFIG_PANIC_ON_SSR_NOTIF_TIMEOUT */

#else /* CONFIG_SETUP_SSR_NOTIF_TIMEOUTS */
#define setup_timeout(dest_ss, source_ss, sysmon_comm)
#define cancel_timeout(subsys)
#define init_subsys_timer(subsys)
#endif /* CONFIG_SETUP_SSR_NOTIF_TIMEOUTS */

/**
 * enum p_subsys_state - state of a subsystem (private)
 * @SUBSYS_NORMAL: subsystem is operating normally
 * @SUBSYS_CRASHED: subsystem has crashed and hasn't been shutdown
 * @SUBSYS_RESTARTING: subsystem has been shutdown and is now restarting
 *
 * The 'private' side of the subsytem state used to determine where in the
 * restart process the subsystem is.
 */
enum p_subsys_state {
	SUBSYS_NORMAL,
	SUBSYS_CRASHED,
	SUBSYS_RESTARTING,
};

/**
 * enum subsys_state - state of a subsystem (public)
 * @SUBSYS_OFFLINING: subsystem is offlining
 * @SUBSYS_OFFLINE: subsystem is offline
 * @SUBSYS_ONLINE: subsystem is online
 *
 * The 'public' side of the subsytem state, exposed to userspace.
 */
enum subsys_state {
	SUBSYS_OFFLINING,
	SUBSYS_OFFLINE,
	SUBSYS_ONLINE,
};

static const char * const subsys_states[] = {
	[SUBSYS_OFFLINING] = "OFFLINING",
	[SUBSYS_OFFLINE] = "OFFLINE",
	[SUBSYS_ONLINE] = "ONLINE",
};

static const char * const restart_levels[] = {
	[RESET_SOC] = "SYSTEM",
	[RESET_SUBSYS_COUPLED] = "RELATED",
};

/**
 * struct subsys_tracking - track state of a subsystem or restart order
 * @p_state: private state of subsystem/order
 * @state: public state of subsystem/order
 * @s_lock: protects p_state
 * @lock: protects subsystem/order callbacks and state
 *
 * Tracks the state of a subsystem or a set of subsystems (restart order).
 * Doing this avoids the need to grab each subsystem's lock and update
 * each subsystems state when restarting an order.
 */
struct subsys_tracking {
	enum p_subsys_state p_state;
	spinlock_t s_lock;
	enum subsys_state state;
	struct mutex lock;
};

/**
 * struct subsys_soc_restart_order - subsystem restart order
 * @subsystem_list: names of subsystems in this restart order
 * @count: number of subsystems in order
 * @track: state tracking and locking
 * @subsys_ptrs: pointers to subsystems in this restart order
 */
struct subsys_soc_restart_order {
	struct device_node **device_ptrs;
	int count;

	struct subsys_tracking track;
	struct subsys_device **subsys_ptrs;
	struct list_head list;
};

struct restart_log {
	struct timespec64 time;
	struct subsys_device *dev;
	struct list_head list;
};

/**
 * struct subsys_device - subsystem device
 * @desc: subsystem descriptor
 * @work: context for subsystem_restart_wq_func() for this device
 * @ssr_wlock: prevents suspend during subsystem_restart()
 * @wlname: name of wakeup source
 * @device_restart_work: work struct for device restart
 * @track: state tracking and locking
 * @notify: subsys notify handle
 * @dev: device
 * @owner: module that provides @desc
 * @count: reference count of subsystem_get()/subsystem_put()
 * @id: ida
 * @restart_level: restart level (0 - panic, 1 - related, 2 - independent, etc.)
 * @restart_order: order of other devices this devices restarts with
 * @crash_count: number of times the device has crashed
 * @do_ramdump_on_put: ramdump on subsystem_put() if true
 * @crashed: indicates if subsystem has crashed
 * @notif_state: current state of subsystem in terms of subsys notifications
 */
struct subsys_device {
	struct subsys_desc *desc;
	struct work_struct work;
	struct wakeup_source *ssr_wlock;
	char wlname[64];
	struct work_struct device_restart_work;
	struct subsys_tracking track;

	void *notify;
	void *early_notify;
	struct device dev;
	struct module *owner;
	int count;
	int id;
	int restart_level;
	int crash_count;
	struct subsys_soc_restart_order *restart_order;
	bool do_ramdump_on_put;
	struct cdev char_dev;
	dev_t dev_no;
	enum crash_status crashed;
	int notif_state;
	struct list_head list;
};

static struct subsys_device *to_subsys(struct device *d)
{
	return container_of(d, struct subsys_device, dev);
}

static struct subsys_tracking *subsys_get_track(struct subsys_device *subsys)
{
	struct subsys_soc_restart_order *order = subsys->restart_order;

	if (order)
		return &order->track;
	else
		return &subsys->track;
}

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", to_subsys(dev)->desc->name);
}
static DEVICE_ATTR_RO(name);

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	enum subsys_state state = to_subsys(dev)->track.state;

	return snprintf(buf, PAGE_SIZE, "%s\n", subsys_states[state]);
}
static DEVICE_ATTR_RO(state);

static ssize_t crash_count_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", to_subsys(dev)->crash_count);
}
static DEVICE_ATTR_RO(crash_count);

static ssize_t
restart_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int level = to_subsys(dev)->restart_level;

	return snprintf(buf, PAGE_SIZE, "%s\n", restart_levels[level]);
}

static ssize_t restart_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);
	const char *p;
	int i, orig_count = count;

	p = memchr(buf, '\n', count);
	if (p)
		count = p - buf;

	for (i = 0; i < ARRAY_SIZE(restart_levels); i++)
		if (!strncasecmp(buf, restart_levels[i], count)) {
			subsys->restart_level = i;
			trace_pil_restart_level(i, subsys->desc->fw_name);
			return orig_count;
		}
	return -EPERM;
}
static DEVICE_ATTR_RW(restart_level);

static ssize_t firmware_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", to_subsys(dev)->desc->fw_name);
}

static ssize_t firmware_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);
	struct subsys_tracking *track = subsys_get_track(subsys);
	const char *p;
	int orig_count = count;

	p = memchr(buf, '\n', count);
	if (p)
		count = p - buf;

	pr_info("Changing subsys fw_name to %s\n", buf);
	mutex_lock(&track->lock);
	strlcpy(subsys->desc->fw_name, buf,
			min(count + 1, sizeof(subsys->desc->fw_name)));
	mutex_unlock(&track->lock);
	return orig_count;
}
static DEVICE_ATTR_RW(firmware_name);

static ssize_t system_debug_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct subsys_device *subsys = to_subsys(dev);
	char p[6] = "set";

	if (!subsys->desc->system_debug)
		strlcpy(p, "reset", sizeof(p));

	return snprintf(buf, PAGE_SIZE, "%s\n", p);
}

static ssize_t system_debug_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);
	const char *p;
	int orig_count = count;

	p = memchr(buf, '\n', count);
	if (p)
		count = p - buf;

	if (!strncasecmp(buf, "set", count))
		subsys->desc->system_debug = true;
	else if (!strncasecmp(buf, "reset", count))
		subsys->desc->system_debug = false;
	else
		return -EPERM;
	return orig_count;
}
static DEVICE_ATTR_RW(system_debug);

int subsys_get_restart_level(struct subsys_device *dev)
{
	return dev->restart_level;
}
EXPORT_SYMBOL(subsys_get_restart_level);

static void subsys_set_state(struct subsys_device *subsys,
			     enum subsys_state state)
{
	unsigned long flags;

	spin_lock_irqsave(&subsys->track.s_lock, flags);
	if (subsys->track.state != state) {
		subsys->track.state = state;
		spin_unlock_irqrestore(&subsys->track.s_lock, flags);
		sysfs_notify(&subsys->dev.kobj, NULL, "state");
		return;
	}
	spin_unlock_irqrestore(&subsys->track.s_lock, flags);
}

static struct attribute *subsys_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_state.attr,
	&dev_attr_crash_count.attr,
	&dev_attr_restart_level.attr,
	&dev_attr_firmware_name.attr,
	&dev_attr_system_debug.attr,
	NULL,
};

ATTRIBUTE_GROUPS(subsys);

struct bus_type subsys_bus_type = {
	.name		= "msm_subsys",
	.dev_groups	= subsys_groups,
};
EXPORT_SYMBOL(subsys_bus_type);

static DEFINE_IDA(subsys_ida);

static int enable_ramdumps;
module_param(enable_ramdumps, int, 0644);

static int enable_mini_ramdumps;
module_param(enable_mini_ramdumps, int, 0644);

static struct workqueue_struct *ssr_wq;
static struct class *char_class;

static LIST_HEAD(restart_log_list);
static LIST_HEAD(subsys_list);
static LIST_HEAD(ssr_order_list);
static DEFINE_MUTEX(soc_order_reg_lock);
static DEFINE_MUTEX(restart_log_mutex);
static DEFINE_MUTEX(subsys_list_lock);
static DEFINE_MUTEX(char_device_lock);
static DEFINE_MUTEX(ssr_order_mutex);

static struct subsys_soc_restart_order *
update_restart_order(struct subsys_device *dev)
{
	int i;
	struct subsys_soc_restart_order *order;
	struct device_node *device = dev->desc->dev->of_node;

	mutex_lock(&soc_order_reg_lock);
	list_for_each_entry(order, &ssr_order_list, list) {
		for (i = 0; i < order->count; i++) {
			if (order->device_ptrs[i] == device) {
				order->subsys_ptrs[i] = dev;
				goto found;
			}
		}
	}
	order = NULL;
found:
	mutex_unlock(&soc_order_reg_lock);

	return order;
}

static int max_restarts;
module_param(max_restarts, int, 0644);

static long max_history_time = 3600;
module_param(max_history_time, long, 0644);

static void do_epoch_check(struct subsys_device *dev)
{
	int n = 0;
	struct timespec64 *time_first = NULL, *curr_time;
	struct restart_log *r_log, *temp;
	static int max_restarts_check;
	static long max_history_time_check;

	mutex_lock(&restart_log_mutex);

	max_restarts_check = max_restarts;
	max_history_time_check = max_history_time;

	/* Check if epoch checking is enabled */
	if (!max_restarts_check)
		goto out;

	r_log = kmalloc(sizeof(struct restart_log), GFP_KERNEL);
	if (!r_log)
		goto out;
	r_log->dev = dev;
	ktime_get_real_ts64(&r_log->time);
	curr_time = &r_log->time;
	INIT_LIST_HEAD(&r_log->list);

	list_add_tail(&r_log->list, &restart_log_list);

	list_for_each_entry_safe(r_log, temp, &restart_log_list, list) {

		if ((curr_time->tv_sec - r_log->time.tv_sec) >
				max_history_time_check) {

			pr_debug("Deleted node with restart_time = %lld\n",
					r_log->time.tv_sec);
			list_del(&r_log->list);
			kfree(r_log);
			continue;
		}
		if (!n) {
			time_first = &r_log->time;
			pr_debug("Time_first: %lld\n", time_first->tv_sec);
		}
		n++;
		pr_debug("Restart_time: %lld\n", r_log->time.tv_sec);
	}

	if (time_first && n >= max_restarts_check) {
		if ((curr_time->tv_sec - time_first->tv_sec) <
				max_history_time_check)
			panic("Subsystems have crashed %d times in less than %ld seconds!",
				max_restarts_check, max_history_time_check);
	}

out:
	mutex_unlock(&restart_log_mutex);
}

static int is_ramdump_enabled(struct subsys_device *dev)
{
	if (dev->desc->ramdump_disable_irq)
		return !dev->desc->ramdump_disable;

	return enable_ramdumps;
}

#ifdef CONFIG_SETUP_SSR_NOTIF_TIMEOUTS
static void notif_timeout_handler(struct timer_list *t)
{
	char *sysmon_msg = "Sysmon communication from %s to %s taking too long";
	char *subsys_notif_msg = "Subsys notifier chain for %s taking too long";
	char *sysmon_shutdwn_msg = "sysmon_send_shutdown to %s taking too long";
	char *unknown_err_msg = "Unknown communication occurred";
	struct subsys_notif_timeout *timeout_data =
		from_timer(timeout_data, t, timer);
	enum ssr_comm comm_type = timeout_data->comm_type;

	switch (comm_type) {
	case SUBSYS_TO_SUBSYS_SYSMON:
		SSR_NOTIF_TIMEOUT_WARN(sysmon_msg, timeout_data->source_name,
				       timeout_data->dest_name);
		break;
	case SUBSYS_TO_HLOS:
		SSR_NOTIF_TIMEOUT_WARN(subsys_notif_msg,
				       timeout_data->source_name);
		break;
	case HLOS_TO_SUBSYS_SYSMON_SHUTDOWN:
		SSR_NOTIF_TIMEOUT_WARN(sysmon_shutdwn_msg,
				       timeout_data->dest_name);
		break;
	default:
		SSR_NOTIF_TIMEOUT_WARN(unknown_err_msg);
	}

}

static void _setup_timeout(struct subsys_desc *source_ss,
			   struct subsys_desc *dest_ss, enum ssr_comm comm_type)
{
	struct subsys_notif_timeout *timeout_data;
	unsigned long timeout;

	switch (comm_type) {
	case SUBSYS_TO_SUBSYS_SYSMON:
		timeout_data = &source_ss->timeout_data;
		timeout_data->dest_name = dest_ss->name;
		timeout_data->source_name = source_ss->name;
		break;
	case SUBSYS_TO_HLOS:
		timeout_data = &source_ss->timeout_data;
		timeout_data->dest_name = NULL;
		timeout_data->source_name = source_ss->name;
		break;
	case HLOS_TO_SUBSYS_SYSMON_SHUTDOWN:
		timeout_data = &dest_ss->timeout_data;
		timeout_data->dest_name = dest_ss->name;
		timeout_data->source_name = NULL;
		break;
	default:
		return;
	}

	timeout_data->comm_type = comm_type;
	timeout = jiffies + msecs_to_jiffies(timeout_vals[comm_type]);
	mod_timer(&timeout_data->timer, timeout);
}

static void _init_subsys_timer(struct subsys_desc *subsys)
{
	timer_setup(&subsys->timeout_data.timer, notif_timeout_handler, 0);
}

#endif /* CONFIG_SETUP_SSR_NOTIF_TIMEOUTS */

static void send_sysmon_notif(struct subsys_device *dev)
{
	struct subsys_device *subsys;

	mutex_lock(&subsys_list_lock);
	list_for_each_entry(subsys, &subsys_list, list)
		if ((subsys->notif_state > 0) && (subsys != dev)) {
			setup_timeout(subsys->desc, dev->desc,
				      SUBSYS_TO_SUBSYS_SYSMON);
			sysmon_send_event(dev->desc, subsys->desc,
						subsys->notif_state);
			cancel_timeout(subsys->desc);
		}
	mutex_unlock(&subsys_list_lock);
}

static int for_each_subsys_device(struct subsys_device **list,
		unsigned int count, void *data,
		int (*fn)(struct subsys_device *, void *))
{
	int ret;

	while (count--) {
		struct subsys_device *dev = *list++;

		if (!dev)
			continue;
		ret = fn(dev, data);
		if (ret)
			return ret;
	}
	return 0;
}

static void notify_each_subsys_device(struct subsys_device **list,
		unsigned int count,
		enum subsys_notif_type notif, void *data)
{
	struct subsys_device *subsys;

	while (count--) {
		struct subsys_device *dev = *list++;
		struct notif_data notif_data;
		struct platform_device *pdev;

		if (!dev)
			continue;

		pdev = container_of(dev->desc->dev, struct platform_device,
									dev);
		dev->notif_state = notif;

		mutex_lock(&subsys_list_lock);
		list_for_each_entry(subsys, &subsys_list, list)
			if (dev != subsys &&
				subsys->track.state == SUBSYS_ONLINE) {
				setup_timeout(dev->desc, subsys->desc,
					      SUBSYS_TO_SUBSYS_SYSMON);
				sysmon_send_event(subsys->desc, dev->desc,
						  notif);
				cancel_timeout(dev->desc);
			}
		mutex_unlock(&subsys_list_lock);

		if (notif == SUBSYS_AFTER_POWERUP &&
				dev->track.state == SUBSYS_ONLINE)
			send_sysmon_notif(dev);

		notif_data.crashed = subsys_get_crash_status(dev);
		notif_data.enable_ramdump = is_ramdump_enabled(dev);
		notif_data.enable_mini_ramdumps = enable_mini_ramdumps;
		notif_data.no_auth = dev->desc->no_auth;
		notif_data.pdev = pdev;

		trace_pil_notif("before_send_notif", notif, dev->desc->fw_name);
		setup_timeout(dev->desc, NULL, SUBSYS_TO_HLOS);
		subsys_notif_queue_notification(dev->notify, notif,
								&notif_data);
		cancel_timeout(dev->desc);
		trace_pil_notif("after_send_notif", notif, dev->desc->fw_name);
	}
}

static int subsystem_shutdown(struct subsys_device *dev, void *data)
{
	const char *name = dev->desc->name;
	int ret;

	pr_info("[%s:%d]: Shutting down %s\n",
			current->comm, current->pid, name);
	ret = dev->desc->shutdown(dev->desc, true);
	if (ret < 0) {
		if (!dev->desc->ignore_ssr_failure) {
			panic("subsys-restart: [%s:%d]: Failed to shutdown %s!",
				current->comm, current->pid, name);
		} else {
			pr_err("Shutdown failure on %s\n", name);
			return ret;
		}
	}
	dev->crash_count++;
	subsys_set_state(dev, SUBSYS_OFFLINE);

#ifdef CONFIG_QGKI_MSM_BOOT_TIME_MARKER
	if (!strcmp(dev->desc->name, "modem"))
		update_marker("M - Modem Stop");
#endif

	return 0;
}

static int subsystem_ramdump(struct subsys_device *dev, void *data)
{
	const char *name = dev->desc->name;

	if (dev->desc->ramdump)
		if (dev->desc->ramdump(is_ramdump_enabled(dev), dev->desc) < 0)
			pr_warn("%s[%s:%d]: Ramdump failed.\n",
				name, current->comm, current->pid);
	dev->do_ramdump_on_put = false;
	return 0;
}

static int subsystem_free_memory(struct subsys_device *dev, void *data)
{
	if (dev->desc->free_memory)
		dev->desc->free_memory(dev->desc);
	return 0;
}

static int subsystem_powerup(struct subsys_device *dev, void *data)
{
	const char *name = dev->desc->name;
	int ret;

	pr_info("[%s:%d]: Powering up %s\n", current->comm, current->pid, name);

	ret = dev->desc->powerup(dev->desc);
	if (ret < 0) {
		notify_each_subsys_device(&dev, 1, SUBSYS_POWERUP_FAILURE,
								NULL);
		if (system_state == SYSTEM_RESTART
			|| system_state == SYSTEM_POWER_OFF) {
			WARN(1, "SSR aborted: %s, system reboot/shutdown is under way\n",
				name);
		} else if (!dev->desc->ignore_ssr_failure) {
			panic("[%s:%d]: Powerup error: %s!",
				current->comm, current->pid, name);
		} else {
			pr_err("Powerup failure on %s(rc:%d)\n", name, ret);
			dump_stack();
		}
		return ret;
	}

	if (ret) {
		notify_each_subsys_device(&dev, 1, SUBSYS_POWERUP_FAILURE,
								NULL);
		if (!dev->desc->ignore_ssr_failure)
			panic("[%s:%d]: Timed out waiting for error ready: %s!",
				current->comm, current->pid, name);
		else
			return ret;
	}
	subsys_set_state(dev, SUBSYS_ONLINE);
	subsys_set_crash_status(dev, CRASH_STATUS_NO_CRASH);

	return 0;
}

static int __find_subsys_device(struct device *dev, const void *data)
{
	struct subsys_device *subsys = to_subsys(dev);

	return !strcmp(subsys->desc->name, data);
}

struct subsys_device *find_subsys_device(const char *str)
{
	struct device *dev;

	if (!str)
		return NULL;

	dev = bus_find_device(&subsys_bus_type, NULL, (void *)str,
			__find_subsys_device);
	return dev ? to_subsys(dev) : NULL;
}
EXPORT_SYMBOL(find_subsys_device);

static int subsys_start(struct subsys_device *subsys)
{
	int ret;

	notify_each_subsys_device(&subsys, 1, SUBSYS_BEFORE_POWERUP,
								NULL);
	ret = subsys->desc->powerup(subsys->desc);
	if (ret) {
		notify_each_subsys_device(&subsys, 1, SUBSYS_POWERUP_FAILURE,
									NULL);
		return ret;
	}
	subsys_set_state(subsys, SUBSYS_ONLINE);

	notify_each_subsys_device(&subsys, 1, SUBSYS_AFTER_POWERUP,
								NULL);
	return ret;
}

static void subsys_stop(struct subsys_device *subsys)
{
	const char *name = subsys->desc->name;

	notify_each_subsys_device(&subsys, 1, SUBSYS_BEFORE_SHUTDOWN, NULL);
	if (!of_property_read_bool(subsys->desc->dev->of_node,
					"qcom,pil-force-shutdown")) {
		subsys_set_state(subsys, SUBSYS_OFFLINING);
		setup_timeout(NULL, subsys->desc,
			      HLOS_TO_SUBSYS_SYSMON_SHUTDOWN);
		subsys->desc->sysmon_shutdown_ret =
				sysmon_send_shutdown(subsys->desc);
		cancel_timeout(subsys->desc);
		if (subsys->desc->sysmon_shutdown_ret)
			pr_debug("Graceful shutdown failed for %s\n", name);
	}

	subsys->desc->shutdown(subsys->desc, false);
	subsys_set_state(subsys, SUBSYS_OFFLINE);
	notify_each_subsys_device(&subsys, 1, SUBSYS_AFTER_SHUTDOWN, NULL);
}

int subsystem_set_fwname(const char *name, const char *fw_name)
{
	struct subsys_device *subsys;

	if (!name)
		return -EINVAL;

	if (!fw_name)
		return -EINVAL;

	subsys = find_subsys_device(name);
	if (!subsys)
		return -EINVAL;

	pr_debug("Changing subsys [%s] fw_name to [%s]\n", name, fw_name);
	strlcpy(subsys->desc->fw_name, fw_name,
		sizeof(subsys->desc->fw_name));

	return 0;
}
EXPORT_SYMBOL(subsystem_set_fwname);

static void *__subsystem_get(const char *name, const char *fw_name)
{
	struct subsys_device *subsys;
	struct subsys_device *subsys_d;
	int ret;
	void *retval;
	struct subsys_tracking *track;

	if (!name)
		return NULL;

	subsys = retval = find_subsys_device(name);
	if (!subsys)
		return ERR_PTR(-ENODEV);
	if (!try_module_get(subsys->owner)) {
		retval = ERR_PTR(-ENODEV);
		goto err_module;
	}

	subsys_d = subsystem_get(subsys->desc->pon_depends_on);
	if (IS_ERR(subsys_d)) {
		retval = subsys_d;
		goto err_depends;
	}

	track = subsys_get_track(subsys);
	mutex_lock(&track->lock);
	if (!subsys->count) {
		if (fw_name) {
			pr_info("Changing subsys fw_name to %s\n", fw_name);
			strlcpy(subsys->desc->fw_name, fw_name,
				sizeof(subsys->desc->fw_name));
		}
		ret = subsys_start(subsys);
		if (ret) {
			retval = ERR_PTR(ret);
			goto err_start;
		}
	}
	subsys->count++;
	mutex_unlock(&track->lock);
	return retval;
err_start:
	mutex_unlock(&track->lock);
	subsystem_put(subsys_d);
err_depends:
	module_put(subsys->owner);
err_module:
	put_device(&subsys->dev);
	return retval;
}

/**
 * subsytem_get() - Boot a subsystem
 * @name: pointer to a string containing the name of the subsystem to boot
 *
 * This function returns a pointer if it succeeds. If an error occurs an
 * ERR_PTR is returned.
 *
 * If this feature is disable, the value %NULL will be returned.
 */
void *subsystem_get(const char *name)
{
	return __subsystem_get(name, NULL);
}
EXPORT_SYMBOL(subsystem_get);

/**
 * subsystem_get_with_fwname() - Boot a subsystem using the firmware name passed
 * @name: pointer to a string containing the name of the subsystem to boot
 * @fw_name: pointer to a string containing the subsystem firmware image name
 *
 * This function returns a pointer if it succeeds. If an error occurs an
 * ERR_PTR is returned.
 *
 * If this feature is disable, the value %NULL will be returned.
 */
void *subsystem_get_with_fwname(const char *name, const char *fw_name)
{
	return __subsystem_get(name, fw_name);
}
EXPORT_SYMBOL(subsystem_get_with_fwname);

/**
 * subsystem_put() - Shutdown a subsystem
 * @peripheral_handle: pointer from a previous call to subsystem_get()
 *
 * This doesn't imply that a subsystem is shutdown until all callers of
 * subsystem_get() have called subsystem_put().
 */
void subsystem_put(void *subsystem)
{
	struct subsys_device *subsys_d, *subsys = subsystem;
	struct subsys_tracking *track;

	if (IS_ERR_OR_NULL(subsys))
		return;

	subsys_d = find_subsys_device(subsys->desc->poff_depends_on);
	if (subsys_d)
		subsystem_put(subsys_d);

	track = subsys_get_track(subsys);
	mutex_lock(&track->lock);
	if (WARN(!subsys->count, "%s: %s: Reference count mismatch\n",
			subsys->desc->name, __func__))
		goto err_out;
	if (!--subsys->count) {
		subsys_stop(subsys);
		if (subsys->do_ramdump_on_put)
			subsystem_ramdump(subsys, NULL);
		subsystem_free_memory(subsys, NULL);
	}
	mutex_unlock(&track->lock);

	module_put(subsys->owner);
	put_device(&subsys->dev);
	return;
err_out:
	mutex_unlock(&track->lock);
}
EXPORT_SYMBOL(subsystem_put);

static void subsystem_restart_wq_func(struct work_struct *work)
{
	struct subsys_device *dev = container_of(work,
						struct subsys_device, work);
	struct subsys_device **list;
	struct subsys_desc *desc = dev->desc;
	struct subsys_soc_restart_order *order = dev->restart_order;
	struct subsys_tracking *track;
	unsigned int count;
	unsigned long flags;
	int ret;

	/*
	 * It's OK to not take the registration lock at this point.
	 * This is because the subsystem list inside the relevant
	 * restart order is not being traversed.
	 */
	if (order) {
		list = order->subsys_ptrs;
		count = order->count;
		track = &order->track;
	} else {
		list = &dev;
		count = 1;
		track = &dev->track;
	}

	/*
	 * If a system reboot/shutdown is under way, ignore subsystem errors.
	 * However, print a message so that we know that a subsystem behaved
	 * unexpectedly here.
	 */
	if (system_state == SYSTEM_RESTART
		|| system_state == SYSTEM_POWER_OFF) {
		WARN(1, "SSR aborted: %s, system reboot/shutdown is under way\n",
			desc->name);
		return;
	}

	mutex_lock(&track->lock);
	do_epoch_check(dev);

	if (dev->track.state == SUBSYS_OFFLINE) {
		mutex_unlock(&track->lock);
		WARN(1, "SSR aborted: %s subsystem not online\n", desc->name);
		return;
	}

	/*
	 * It's necessary to take the registration lock because the subsystem
	 * list in the SoC restart order will be traversed and it shouldn't be
	 * changed until _this_ restart sequence completes.
	 */
	mutex_lock(&soc_order_reg_lock);

	pr_debug("[%s:%d]: Starting restart sequence for %s\n",
			current->comm, current->pid, desc->name);
	notify_each_subsys_device(list, count, SUBSYS_BEFORE_SHUTDOWN, NULL);
	ret = for_each_subsys_device(list, count, NULL, subsystem_shutdown);
	if (ret)
		goto err;
	notify_each_subsys_device(list, count, SUBSYS_AFTER_SHUTDOWN, NULL);

	notify_each_subsys_device(list, count, SUBSYS_RAMDUMP_NOTIFICATION,
									NULL);

	spin_lock_irqsave(&track->s_lock, flags);
	track->p_state = SUBSYS_RESTARTING;
	spin_unlock_irqrestore(&track->s_lock, flags);

	/* Collect ram dumps for all subsystems in order here */
	for_each_subsys_device(list, count, NULL, subsystem_ramdump);

	for_each_subsys_device(list, count, NULL, subsystem_free_memory);

	notify_each_subsys_device(list, count, SUBSYS_BEFORE_POWERUP, NULL);
	ret = for_each_subsys_device(list, count, NULL, subsystem_powerup);
	if (ret)
		goto err;
	notify_each_subsys_device(list, count, SUBSYS_AFTER_POWERUP, NULL);

	pr_info("[%s:%d]: Restart sequence for %s completed.\n",
			current->comm, current->pid, desc->name);

err:
	/* Reset subsys count */
	if (ret)
		dev->count = 0;

	mutex_unlock(&soc_order_reg_lock);
	mutex_unlock(&track->lock);

	spin_lock_irqsave(&track->s_lock, flags);
	track->p_state = SUBSYS_NORMAL;
	__pm_relax(dev->ssr_wlock);
	spin_unlock_irqrestore(&track->s_lock, flags);
}

static void __subsystem_restart_dev(struct subsys_device *dev)
{
	struct subsys_desc *desc = dev->desc;
	const char *name = dev->desc->name;
	struct subsys_tracking *track;
	unsigned long flags;

	pr_debug("Restarting %s [level=%s]!\n", desc->name,
			restart_levels[dev->restart_level]);

	track = subsys_get_track(dev);
	/*
	 * Allow drivers to call subsystem_restart{_dev}() as many times as
	 * they want up until the point where the subsystem is shutdown.
	 */
	spin_lock_irqsave(&track->s_lock, flags);
	if (track->p_state != SUBSYS_CRASHED &&
					dev->track.state == SUBSYS_ONLINE) {
		if (track->p_state != SUBSYS_RESTARTING) {
			track->p_state = SUBSYS_CRASHED;
			__pm_stay_awake(dev->ssr_wlock);
			queue_work(ssr_wq, &dev->work);
		} else {
			panic("Subsystem %s crashed during SSR!", name);
		}
	} else
		WARN(dev->track.state == SUBSYS_OFFLINE,
			"SSR aborted: %s subsystem not online\n", name);
	spin_unlock_irqrestore(&track->s_lock, flags);
}

static void device_restart_work_hdlr(struct work_struct *work)
{
	struct subsys_device *dev = container_of(work, struct subsys_device,
							device_restart_work);

	notify_each_subsys_device(&dev, 1, SUBSYS_SOC_RESET, NULL);
	/*
	 * Temporary workaround until ramdump userspace application calls
	 * sync() and fclose() on attempting the dump.
	 */
	msleep(100);
	panic("subsys-restart: Resetting the SoC - %s crashed.",
							dev->desc->name);
}

int subsystem_restart_dev(struct subsys_device *dev)
{
	const char *name;

	if (!get_device(&dev->dev))
		return -ENODEV;

	if (!try_module_get(dev->owner)) {
		put_device(&dev->dev);
		return -ENODEV;
	}

	name = dev->desc->name;

	subsys_send_early_notifications(dev->early_notify);

	/*
	 * If a system reboot/shutdown is underway, ignore subsystem errors.
	 * However, print a message so that we know that a subsystem behaved
	 * unexpectedly here.
	 */
	if (system_state == SYSTEM_RESTART
		|| system_state == SYSTEM_POWER_OFF) {
		pr_err("%s crashed during a system poweroff/shutdown.\n", name);
		return -EBUSY;
	}

	pr_info("Restart sequence requested for %s, restart_level = %s.\n",
		name, restart_levels[dev->restart_level]);

	if (disable_restart_work == DISABLE_SSR) {
		pr_warn("subsys-restart: Ignoring restart request for %s\n",
									name);
		return 0;
	}

	switch (dev->restart_level) {

	case RESET_SUBSYS_COUPLED:
		__subsystem_restart_dev(dev);
		break;
	case RESET_SOC:
		__pm_stay_awake(dev->ssr_wlock);
		schedule_work(&dev->device_restart_work);
		return 0;
	default:
		panic("subsys-restart: Unknown restart level!\n");
		break;
	}
	module_put(dev->owner);
	put_device(&dev->dev);

	return 0;
}
EXPORT_SYMBOL(subsystem_restart_dev);

int subsystem_restart(const char *name)
{
	int ret;
	struct subsys_device *dev = find_subsys_device(name);

	if (!dev)
		return -ENODEV;

	ret = subsystem_restart_dev(dev);
	put_device(&dev->dev);
	return ret;
}
EXPORT_SYMBOL(subsystem_restart);

int subsystem_crashed(const char *name)
{
	struct subsys_device *dev = find_subsys_device(name);
	struct subsys_tracking *track;

	if (!dev)
		return -ENODEV;

	if (!get_device(&dev->dev))
		return -ENODEV;

	track = subsys_get_track(dev);

	mutex_lock(&track->lock);
	dev->do_ramdump_on_put = true;
	/*
	 * TODO: Make this work with multiple consumers where one is calling
	 * subsystem_restart() and another is calling this function. To do
	 * so would require updating private state, etc.
	 */
	mutex_unlock(&track->lock);

	put_device(&dev->dev);
	return 0;
}
EXPORT_SYMBOL(subsystem_crashed);

void subsys_set_crash_status(struct subsys_device *dev,
				enum crash_status crashed)
{
	dev->crashed = crashed;
}
EXPORT_SYMBOL(subsys_set_crash_status);

enum crash_status subsys_get_crash_status(struct subsys_device *dev)
{
	return dev->crashed;
}
EXPORT_SYMBOL(subsys_get_crash_status);

static struct subsys_device *desc_to_subsys(struct device *d)
{
	struct subsys_device *device, *subsys_dev = NULL;

	mutex_lock(&subsys_list_lock);
	list_for_each_entry(device, &subsys_list, list)
		if (device->desc->dev == d)
			subsys_dev = device;
	mutex_unlock(&subsys_list_lock);
	return subsys_dev;
}

void notify_proxy_vote(struct device *device)
{
	struct subsys_device *dev = desc_to_subsys(device);

	if (dev)
		notify_each_subsys_device(&dev, 1, SUBSYS_PROXY_VOTE, NULL);
}
EXPORT_SYMBOL(notify_proxy_vote);

void notify_proxy_unvote(struct device *device)
{
	struct subsys_device *dev = desc_to_subsys(device);

	if (dev)
		notify_each_subsys_device(&dev, 1, SUBSYS_PROXY_UNVOTE, NULL);
}
EXPORT_SYMBOL(notify_proxy_unvote);

void notify_before_auth_and_reset(struct device *device)
{
	struct subsys_device *dev = desc_to_subsys(device);

	if (dev)
		notify_each_subsys_device(&dev, 1,
			SUBSYS_BEFORE_AUTH_AND_RESET, NULL);
}
EXPORT_SYMBOL(notify_before_auth_and_reset);

static int subsys_device_open(struct inode *inode, struct file *file)
{
	struct subsys_device *device, *subsys_dev = NULL;
	void *retval;

	mutex_lock(&subsys_list_lock);
	list_for_each_entry(device, &subsys_list, list)
		if (MINOR(device->dev_no) == iminor(inode))
			subsys_dev = device;
	mutex_unlock(&subsys_list_lock);

	if (!subsys_dev)
		return -EINVAL;

	retval = subsystem_get_with_fwname(subsys_dev->desc->name,
					subsys_dev->desc->fw_name);
	if (PTR_ERR_OR_ZERO(retval))
		return PTR_ERR(retval);

	return 0;
}

static int subsys_device_close(struct inode *inode, struct file *file)
{
	struct subsys_device *device, *subsys_dev = NULL;

	mutex_lock(&subsys_list_lock);
	list_for_each_entry(device, &subsys_list, list)
		if (MINOR(device->dev_no) == iminor(inode))
			subsys_dev = device;
	mutex_unlock(&subsys_list_lock);

	if (!subsys_dev)
		return -EINVAL;

	subsystem_put(subsys_dev);
	return 0;
}

static const struct file_operations subsys_device_fops = {
		.owner = THIS_MODULE,
		.open = subsys_device_open,
		.release = subsys_device_close,
};

static void subsys_device_release(struct device *dev)
{
	struct subsys_device *subsys = to_subsys(dev);

	mutex_destroy(&subsys->track.lock);
	ida_simple_remove(&subsys_ida, subsys->id);
	kfree(subsys);
}

static int subsys_char_device_add(struct subsys_device *subsys_dev)
{
	int ret = 0;
	static int major, minor;
	dev_t dev_no;

	mutex_lock(&char_device_lock);
	if (!major) {
		ret = alloc_chrdev_region(&dev_no, 0, 4, "subsys");
		if (ret < 0) {
			pr_err("Failed to alloc subsys_dev region, err %d\n",
									ret);
			goto fail;
		}
		major = MAJOR(dev_no);
		minor = MINOR(dev_no);
	} else
		dev_no = MKDEV(major, minor);

	if (!device_create(char_class, subsys_dev->desc->dev, dev_no,
			NULL, "subsys_%s", subsys_dev->desc->name)) {
		pr_err("Failed to create subsys_%s device\n",
						subsys_dev->desc->name);
		goto fail_unregister_cdev_region;
	}

	cdev_init(&subsys_dev->char_dev, &subsys_device_fops);
	subsys_dev->char_dev.owner = THIS_MODULE;
	ret = cdev_add(&subsys_dev->char_dev, dev_no, 1);
	if (ret < 0)
		goto fail_destroy_device;

	subsys_dev->dev_no = dev_no;
	minor++;
	mutex_unlock(&char_device_lock);

	return 0;

fail_destroy_device:
	device_destroy(char_class, dev_no);
fail_unregister_cdev_region:
	unregister_chrdev_region(dev_no, 1);
fail:
	mutex_unlock(&char_device_lock);
	return ret;
}

static void subsys_char_device_remove(struct subsys_device *subsys_dev)
{
	cdev_del(&subsys_dev->char_dev);
	device_destroy(char_class, subsys_dev->dev_no);
	unregister_chrdev_region(subsys_dev->dev_no, 1);
}

static void subsys_remove_restart_order(struct device_node *device)
{
	struct subsys_soc_restart_order *order;
	int i;

	mutex_lock(&ssr_order_mutex);
	list_for_each_entry(order, &ssr_order_list, list)
		for (i = 0; i < order->count; i++)
			if (order->device_ptrs[i] == device)
				order->subsys_ptrs[i] = NULL;
	mutex_unlock(&ssr_order_mutex);
}

static struct subsys_soc_restart_order *ssr_parse_restart_orders(struct
							subsys_desc * desc)
{
	int i, j, count, num = 0;
	struct subsys_soc_restart_order *order, *tmp;
	struct device *dev = desc->dev;
	struct device_node *ssr_node;
	uint32_t len;

	if (!of_get_property(dev->of_node, "qcom,restart-group", &len))
		return NULL;

	count = len/sizeof(uint32_t);

	order = devm_kzalloc(dev, sizeof(*order), GFP_KERNEL);
	if (!order)
		return ERR_PTR(-ENOMEM);

	order->subsys_ptrs = devm_kzalloc(dev,
				count * sizeof(struct subsys_device *),
				GFP_KERNEL);
	if (!order->subsys_ptrs)
		return ERR_PTR(-ENOMEM);

	order->device_ptrs = devm_kzalloc(dev,
				count * sizeof(struct device_node *),
				GFP_KERNEL);
	if (!order->device_ptrs)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < count; i++) {
		ssr_node = of_parse_phandle(dev->of_node,
						"qcom,restart-group", i);
		if (!ssr_node)
			return ERR_PTR(-ENXIO);
		of_node_put(ssr_node);
		pr_info("%s device has been added to %s's restart group\n",
						ssr_node->name, desc->name);
		order->device_ptrs[i] = ssr_node;
	}

	/*
	 * Check for similar restart groups. If found, return
	 * without adding the new group to the ssr_order_list.
	 */
	mutex_lock(&ssr_order_mutex);
	list_for_each_entry(tmp, &ssr_order_list, list) {
		for (i = 0; i < count; i++) {
			for (j = 0; j < count; j++) {
				if (order->device_ptrs[j] !=
					tmp->device_ptrs[i])
					continue;
				else
					num++;
			}
		}

		if (num == count && tmp->count == count)
			goto err;
		else if (num) {
			tmp = ERR_PTR(-EINVAL);
			goto err;
		}
	}

	order->count = count;
	mutex_init(&order->track.lock);
	spin_lock_init(&order->track.s_lock);

	INIT_LIST_HEAD(&order->list);
	list_add_tail(&order->list, &ssr_order_list);
	mutex_unlock(&ssr_order_mutex);

	return order;
err:
	mutex_unlock(&ssr_order_mutex);
	return tmp;
}

static int subsys_parse_devicetree(struct subsys_desc *desc)
{
	struct subsys_soc_restart_order *order;
	struct platform_device *pdev = container_of(desc->dev,
					struct platform_device, dev);

	desc->ignore_ssr_failure = of_property_read_bool(pdev->dev.of_node,
						"qcom,ignore-ssr-failure");

	order = ssr_parse_restart_orders(desc);
	if (IS_ERR(order)) {
		pr_err("Could not initialize SSR restart order, err = %ld\n",
							PTR_ERR(order));
		return PTR_ERR(order);
	}

	if (of_property_read_string(pdev->dev.of_node, "qcom,pon-depends-on",
				&desc->pon_depends_on))
		pr_debug("pon-depends-on not set for %s\n", desc->name);

	if (of_property_read_string(pdev->dev.of_node, "qcom,poff-depends-on",
				&desc->poff_depends_on))
		pr_debug("poff-depends-on not set for %s\n", desc->name);

	return 0;
}

struct subsys_device *subsys_register(struct subsys_desc *desc)
{
	struct subsys_device *subsys;
	struct device_node *ofnode = desc->dev->of_node;
	int ret;

	subsys = kzalloc(sizeof(*subsys), GFP_KERNEL);
	if (!subsys)
		return ERR_PTR(-ENOMEM);

	subsys->desc = desc;
	subsys->owner = desc->owner;
	subsys->dev.parent = desc->dev;
	subsys->dev.bus = &subsys_bus_type;
	subsys->dev.release = subsys_device_release;
	subsys->notif_state = -1;
	subsys->desc->sysmon_pid = -1;
	subsys->desc->state = NULL;
	strlcpy(subsys->desc->fw_name, desc->name,
			sizeof(subsys->desc->fw_name));

	subsys->notify = subsys_notif_add_subsys(desc->name);
	subsys->early_notify = subsys_get_early_notif_info(desc->name);

	snprintf(subsys->wlname, sizeof(subsys->wlname), "ssr(%s)", desc->name);

	INIT_WORK(&subsys->work, subsystem_restart_wq_func);
	INIT_WORK(&subsys->device_restart_work, device_restart_work_hdlr);
	spin_lock_init(&subsys->track.s_lock);
	init_subsys_timer(desc);

	subsys->id = ida_simple_get(&subsys_ida, 0, 0, GFP_KERNEL);
	if (subsys->id < 0) {
		ret = subsys->id;
		kfree(subsys);
		return ERR_PTR(ret);
	}

	dev_set_name(&subsys->dev, "subsys%d", subsys->id);

	mutex_init(&subsys->track.lock);

	ret = device_register(&subsys->dev);
	if (ret) {
		put_device(&subsys->dev);
		return ERR_PTR(ret);
	}

	subsys->ssr_wlock =
		wakeup_source_register(&subsys->dev, subsys->wlname);
	if (!subsys->ssr_wlock) {
		ret = -ENOMEM;
		goto err_wakeup_src_register;
	}

	ret = subsys_char_device_add(subsys);
	if (ret)
		goto err_register;

	if (ofnode) {
		ret = subsys_parse_devicetree(desc);
		if (ret)
			goto err_parse_device_tree;

		subsys->restart_order = update_restart_order(subsys);

		if (of_property_read_u32(ofnode, "qcom,ssctl-instance-id",
					&desc->ssctl_instance_id))
			pr_debug("Reading instance-id for %s failed\n",
								desc->name);

		if (of_property_read_u32(ofnode, "qcom,sysmon-id",
					&subsys->desc->sysmon_pid))
			pr_debug("Reading sysmon-id for %s failed\n",
								desc->name);

		subsys->desc->edge = of_get_property(ofnode, "qcom,edge",
									NULL);
		if (!subsys->desc->edge)
			pr_debug("Reading qcom,edge for %s failed\n",
								desc->name);
	}

	ret = sysmon_notifier_register(desc);
	if (ret < 0)
		goto err_sysmon_notifier;

	if (subsys->desc->edge) {
		ret = sysmon_glink_register(desc);
		if (ret < 0)
			goto err_sysmon_glink_register;
	}
	mutex_lock(&subsys_list_lock);
	INIT_LIST_HEAD(&subsys->list);
	list_add_tail(&subsys->list, &subsys_list);
	mutex_unlock(&subsys_list_lock);

	return subsys;
err_sysmon_glink_register:
	sysmon_notifier_unregister(subsys->desc);
err_sysmon_notifier:
	if (ofnode)
		subsys_remove_restart_order(ofnode);
err_parse_device_tree:
	subsys_char_device_remove(subsys);
err_register:
	wakeup_source_unregister(subsys->ssr_wlock);
err_wakeup_src_register:
	device_unregister(&subsys->dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(subsys_register);

void subsys_unregister(struct subsys_device *subsys)
{
	struct subsys_device *subsys_dev, *tmp;
	struct device_node *device = subsys->desc->dev->of_node;

	if (IS_ERR_OR_NULL(subsys))
		return;

	if (get_device(&subsys->dev)) {
		mutex_lock(&subsys_list_lock);
		list_for_each_entry_safe(subsys_dev, tmp, &subsys_list, list)
			if (subsys_dev == subsys)
				list_del(&subsys->list);
		mutex_unlock(&subsys_list_lock);

		if (device) {
			subsys_remove_restart_order(device);
		}
		mutex_lock(&subsys->track.lock);
		WARN_ON(subsys->count);
		device_unregister(&subsys->dev);
		mutex_unlock(&subsys->track.lock);
		subsys_char_device_remove(subsys);
		sysmon_notifier_unregister(subsys->desc);
		if (subsys->desc->edge)
			sysmon_glink_unregister(subsys->desc);
		wakeup_source_unregister(subsys->ssr_wlock);
		put_device(&subsys->dev);
	}
}
EXPORT_SYMBOL(subsys_unregister);

static int subsys_panic(struct device *dev, void *data)
{
	struct subsys_device *subsys = to_subsys(dev);

	if (subsys->desc->crash_shutdown)
		subsys->desc->crash_shutdown(subsys->desc);
	return 0;
}

static int ssr_panic_handler(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	bus_for_each_dev(&subsys_bus_type, NULL, NULL, subsys_panic);
	return NOTIFY_DONE;
}

static struct notifier_block panic_nb = {
	.notifier_call  = ssr_panic_handler,
};

#ifdef ENABLE_MODEM_RESTART
static int modem_restart(void)
{
	int ret;
	int orig_level;  //add for change back the default restart level after triggering modem restart
	struct subsys_device *dev = find_subsys_device("modem");

	if (!dev)
		return -ENODEV;

	pr_err("modem restart triggered\n");
	orig_level = dev->restart_level;
	dev->restart_level = RESET_SUBSYS_COUPLED;
	ret = subsystem_restart_dev(dev);
	dev->restart_level = orig_level;
	put_device(&dev->dev);
	return ret;
}

static int modem_restart_show(struct seq_file *m, void *v)
{
	seq_printf(m, "trigger modem restart\n");
	modem_restart();
	return 0;
}

static int modem_restart_open(struct inode *inode, struct file *file)
{
	return single_open(file, modem_restart_show, inode->i_private);
}

static const struct file_operations modem_restart_fops = {
	.open = modem_restart_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int __init subsys_restart_init(void)
{
	int ret;
#ifdef ENABLE_MODEM_RESTART
	struct proc_dir_entry *dir;
#endif

	ssr_wq = alloc_workqueue("ssr_wq",
		WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 0);
	BUG_ON(!ssr_wq);

	ret = bus_register(&subsys_bus_type);
	if (ret)
		goto err_bus;

	char_class = class_create(THIS_MODULE, "subsys");
	if (IS_ERR(char_class)) {
		ret = -ENOMEM;
		pr_err("Failed to create subsys_dev class\n");
		goto err_class;
	}

	ret = atomic_notifier_chain_register(&panic_notifier_list, &panic_nb);
	if (ret)
		goto err_soc;

#ifdef ENABLE_MODEM_RESTART
	dir = proc_create("modem_restart", 0440, NULL, &modem_restart_fops);
	if (!dir) {
		pr_alert("Now in %s. proc_create.dir = %p\n", __func__, dir);
		return -1;
	}
#endif

	return 0;

err_soc:
	class_destroy(char_class);
err_class:
	bus_unregister(&subsys_bus_type);
err_bus:
	destroy_workqueue(ssr_wq);
	return ret;
}
arch_initcall(subsys_restart_init);

static void __exit subsys_restart_exit(void)
{
	atomic_notifier_chain_unregister(&panic_notifier_list, &panic_nb);
	class_destroy(char_class);
	bus_unregister(&subsys_bus_type);
	destroy_workqueue(ssr_wq);
}
module_exit(subsys_restart_exit);

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. Subsystem Restart Driver");
MODULE_LICENSE("GPL v2");
