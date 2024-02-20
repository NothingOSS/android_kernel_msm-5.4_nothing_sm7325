// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"sde_dbg:[%s] " fmt, __func__

#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <linux/slab.h>
#include <linux/sched/clock.h>

#include "sde_dbg.h"
#include "sde_trace.h"

#define SDE_EVTLOG_FILTER_STRSIZE	64

struct sde_evtlog_filter {
	struct list_head list;
	char filter[SDE_EVTLOG_FILTER_STRSIZE];
};

static bool _sde_evtlog_is_filtered_no_lock(
		struct sde_dbg_evtlog *evtlog, const char *str)
{
	struct sde_evtlog_filter *filter_node;
	size_t len;
	bool rc;

	if (!str)
		return true;

	len = strlen(str);

	/*
	 * Filter the incoming string IFF the list is not empty AND
	 * a matching entry is not in the list.
	 */
	rc = !list_empty(&evtlog->filter_list);
	list_for_each_entry(filter_node, &evtlog->filter_list, list)
		if (strnstr(str, filter_node->filter, len)) {
			rc = false;
			break;
		}

	return rc;
}

bool sde_evtlog_is_enabled(struct sde_dbg_evtlog *evtlog, u32 flag)
{
	return evtlog && (evtlog->enable & flag);
}

void sde_evtlog_log(struct sde_dbg_evtlog *evtlog, const char *name, int line,
		int flag, ...)
{
	unsigned long flags;
	int i, val = 0;
	va_list args;
	struct sde_dbg_evtlog_log *log;

	if (!evtlog)
		return;

	if (!sde_evtlog_is_enabled(evtlog, flag))
		return;

	spin_lock_irqsave(&evtlog->spin_lock, flags);

	if (_sde_evtlog_is_filtered_no_lock(evtlog, name))
		goto exit;

	log = &evtlog->logs[evtlog->curr];
	log->time = local_clock();
	log->name = name;
	log->line = line;
	log->data_cnt = 0;
	log->pid = current->pid;
	log->cpu = raw_smp_processor_id();

	va_start(args, flag);
	for (i = 0; i < SDE_EVTLOG_MAX_DATA; i++) {

		val = va_arg(args, int);
		if (val == SDE_EVTLOG_DATA_LIMITER)
			break;

		log->data[i] = val;
	}
	va_end(args);
	log->data_cnt = i;
	evtlog->curr = (evtlog->curr + 1) % SDE_EVTLOG_ENTRY;
	evtlog->last++;

	trace_sde_evtlog(name, line, log->data_cnt, log->data);
exit:
	spin_unlock_irqrestore(&evtlog->spin_lock, flags);
}

void sde_reglog_log(u8 blk_id, u32 val, u32 addr)
{
	struct sde_dbg_reglog_log *log;
	struct sde_dbg_reglog *reglog = sde_dbg_base_reglog;
	int index;

	if (!reglog)
		return;

	index = abs(atomic64_inc_return(&reglog->curr) % SDE_REGLOG_ENTRY);

	log = &reglog->logs[index];
	log->blk_id = blk_id;
	log->val = val;
	log->addr = addr;
	log->time = local_clock();
	log->pid = current->pid;
	reglog->last++;
}

/* always dump the last entries which are not dumped yet */
static bool _sde_evtlog_dump_calc_range(struct sde_dbg_evtlog *evtlog,
		bool update_last_entry, bool full_dump)
{
	int max_entries = full_dump ? SDE_EVTLOG_ENTRY : SDE_EVTLOG_PRINT_ENTRY;

	if (!evtlog)
		return false;

	evtlog->first = evtlog->next;

	if (update_last_entry)
		evtlog->last_dump = evtlog->last;

	if (evtlog->last_dump == evtlog->first)
		return false;

	if (evtlog->last_dump < evtlog->first) {
		evtlog->first %= SDE_EVTLOG_ENTRY;
		if (evtlog->last_dump < evtlog->first)
			evtlog->last_dump += SDE_EVTLOG_ENTRY;
	}

	if ((evtlog->last_dump - evtlog->first) > max_entries) {
		pr_info("evtlog skipping %d entries, last=%d\n",
			evtlog->last_dump - evtlog->first -
			max_entries, evtlog->last_dump - 1);
		evtlog->first = evtlog->last_dump - max_entries;
	}
	evtlog->next = evtlog->first + 1;

	return true;
}

ssize_t sde_evtlog_dump_to_buffer(struct sde_dbg_evtlog *evtlog,
		char *evtlog_buf, ssize_t evtlog_buf_size,
		bool update_last_entry, bool full_dump)
{
	int i;
	ssize_t off = 0;
	struct sde_dbg_evtlog_log *log, *prev_log;
	unsigned long flags;

	if (!evtlog || !evtlog_buf)
		return 0;

	spin_lock_irqsave(&evtlog->spin_lock, flags);

	/* update markers, exit if nothing to print */
	if (!_sde_evtlog_dump_calc_range(evtlog, update_last_entry, full_dump))
		goto exit;

	log = &evtlog->logs[evtlog->first % SDE_EVTLOG_ENTRY];

	prev_log = &evtlog->logs[(evtlog->first - 1) % SDE_EVTLOG_ENTRY];

	off = snprintf((evtlog_buf + off), (evtlog_buf_size - off), "%s:%-4d",
		log->name, log->line);

	if (off < SDE_EVTLOG_BUF_ALIGN) {
		memset((evtlog_buf + off), 0x20, (SDE_EVTLOG_BUF_ALIGN - off));
		off = SDE_EVTLOG_BUF_ALIGN;
	}

	off += snprintf((evtlog_buf + off), (evtlog_buf_size - off),
		"=>[%-8d:%-11llu:%9llu][%-4d]:[%-4d]:", evtlog->first,
		log->time, (log->time - prev_log->time), log->pid, log->cpu);

	for (i = 0; i < log->data_cnt; i++)
		off += snprintf((evtlog_buf + off), (evtlog_buf_size - off),
			"%x ", log->data[i]);

	off += snprintf((evtlog_buf + off), (evtlog_buf_size - off), "\n");
exit:
	spin_unlock_irqrestore(&evtlog->spin_lock, flags);

	return off;
}

void sde_evtlog_dump_all(struct sde_dbg_evtlog *evtlog)
{
	char buf[SDE_EVTLOG_BUF_MAX];
	bool update_last_entry = true;

	if (!evtlog)
		return;

	while (sde_evtlog_dump_to_buffer(evtlog, buf, sizeof(buf),
				update_last_entry, false)) {
		pr_info("%s\n", buf);
		update_last_entry = false;
	}
}

struct sde_dbg_evtlog *sde_evtlog_init(void)
{
	struct sde_dbg_evtlog *evtlog;

	evtlog = kzalloc(sizeof(*evtlog), GFP_KERNEL);
	if (!evtlog)
		return ERR_PTR(-ENOMEM);

	if (sde_mini_dump_add_region("evt_log", sizeof(*evtlog),
			evtlog) < 0)
		pr_err("minidump add region failed for evtlog\n");

	spin_lock_init(&evtlog->spin_lock);
	evtlog->enable = SDE_EVTLOG_DEFAULT_ENABLE;

	INIT_LIST_HEAD(&evtlog->filter_list);

	return evtlog;
}

struct sde_dbg_reglog *sde_reglog_init(void)
{
	struct sde_dbg_reglog *reglog;

	reglog = kzalloc(sizeof(*reglog), GFP_KERNEL);
	if (!reglog)
		return ERR_PTR(-ENOMEM);

	atomic64_set(&reglog->curr, 0);
	if (sde_mini_dump_add_region("reg_log", sizeof(*reglog),
			reglog) < 0)
		pr_err("minidump add region failed for reglog\n");

	return reglog;
}

int sde_evtlog_get_filter(struct sde_dbg_evtlog *evtlog, int index,
		char *buf, size_t bufsz)
{
	struct sde_evtlog_filter *filter_node;
	unsigned long flags;
	int rc = -EFAULT;

	if (!evtlog || !buf || !bufsz || index < 0)
		return -EINVAL;

	spin_lock_irqsave(&evtlog->spin_lock, flags);
	list_for_each_entry(filter_node, &evtlog->filter_list, list) {
		if (index--)
			continue;

		/* don't care about return value */
		(void)strlcpy(buf, filter_node->filter, bufsz);
		rc = 0;
		break;
	}
	spin_unlock_irqrestore(&evtlog->spin_lock, flags);

	return rc;
}

void sde_evtlog_set_filter(struct sde_dbg_evtlog *evtlog, char *filter)
{
	struct sde_evtlog_filter *filter_node, *tmp;
	struct list_head free_list;
	unsigned long flags;
	char *flt;

	if (!evtlog)
		return;

	INIT_LIST_HEAD(&free_list);

	/*
	 * Clear active filter list and cache filter_nodes locally
	 * to reduce memory fragmentation.
	 */
	spin_lock_irqsave(&evtlog->spin_lock, flags);
	list_for_each_entry_safe(filter_node, tmp, &evtlog->filter_list, list) {
		list_del_init(&filter_node->list);
		list_add_tail(&filter_node->list, &free_list);
	}
	spin_unlock_irqrestore(&evtlog->spin_lock, flags);

	/*
	 * Parse incoming filter request string and build up a new
	 * filter list. New filter nodes are taken from the local
	 * free list, if available, and allocated from the system
	 * heap once the free list is empty.
	 */
	while (filter && (flt = strsep(&filter, "|\r\n\t ")) != NULL) {
		if (!*flt)
			continue;

		if (list_empty(&free_list)) {
			filter_node = kzalloc(sizeof(*filter_node), GFP_KERNEL);
			if (!filter_node)
				break;

			INIT_LIST_HEAD(&filter_node->list);
		} else {
			filter_node = list_first_entry(&free_list,
					struct sde_evtlog_filter, list);
			list_del_init(&filter_node->list);
		}

		/* don't care if copy truncated */
		(void)strlcpy(filter_node->filter, flt,
				SDE_EVTLOG_FILTER_STRSIZE);

		spin_lock_irqsave(&evtlog->spin_lock, flags);
		list_add_tail(&filter_node->list, &evtlog->filter_list);
		spin_unlock_irqrestore(&evtlog->spin_lock, flags);
	}

	/*
	 * Free any unused filter_nodes back to the system.
	 */
	list_for_each_entry_safe(filter_node, tmp, &free_list, list) {
		list_del(&filter_node->list);
		kfree(filter_node);
	}
}

void sde_evtlog_destroy(struct sde_dbg_evtlog *evtlog)
{
	struct sde_evtlog_filter *filter_node, *tmp;

	if (!evtlog)
		return;

	list_for_each_entry_safe(filter_node, tmp, &evtlog->filter_list, list) {
		list_del(&filter_node->list);
		kfree(filter_node);
	}
	kfree(evtlog);
}

void sde_reglog_destroy(struct sde_dbg_reglog *reglog)
{
	if (!reglog)
		return;

	kfree(reglog);
}
