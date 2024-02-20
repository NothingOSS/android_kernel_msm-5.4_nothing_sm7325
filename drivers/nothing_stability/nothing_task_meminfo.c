#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/fs_struct.h>
#include <linux/fs_context.h>
#include <linux/seq_file.h>
#include <linux/sched/clock.h>
#include <linux/oom.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/dma-buf.h>
#include <linux/dma-resv.h>
#include <linux/fdtable.h>
#include <linux/hashtable.h>
#include <linux/list_sort.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/kernel.h>

struct dma_info {
	struct dma_buf *dmabuf;
	struct hlist_node head;
};

struct dma_proc {
	char name[TASK_COMM_LEN];
	pid_t pid;
	size_t size;
	struct hlist_head dma_bufs[1 << 10];
	struct list_head head;
};

struct dma_buf_priv {
	int count;
	size_t size;
	struct seq_file *s;
};

static int show_all_process_thread(struct seq_file *m, void *p)
{
	struct task_struct *t;

	seq_printf(m, "[ pid ]   uid  tgid total_vm     rss(KB) anon(kB) file(KB)  swap(KB) sheme(KB) nptes(KB) adj   s name\n");

	rcu_read_lock();
	for_each_process(t) {
		if (!t)
			continue;
		if (!t->mm) {
			continue;
		}

		seq_printf(m, "[%5d] %5d %5d %8lu %8lu %8lu %8lu %8lu %8lu %8lu %11hd %c %s\n",
			t->pid,
			from_kuid(&init_user_ns, task_uid(t)),
			t->tgid,
			t->mm->total_vm,
			get_mm_rss(t->mm) * SZ_4,
			get_mm_counter(t->mm, MM_ANONPAGES) * SZ_4,
			get_mm_counter(t->mm, MM_FILEPAGES) * SZ_4,
			get_mm_counter(t->mm, MM_SWAPENTS) * SZ_4,
			get_mm_counter(t->mm, MM_SHMEMPAGES) * SZ_4,
			mm_pgtables_bytes(t->mm) / SZ_1K,
			t->signal->oom_score_adj,
			task_state_to_char(t),
			t->comm
			);
	}

	rcu_read_unlock();
	return 0;
}

static void free_proc(struct dma_proc *proc)
{
	struct dma_info *tmp;
	struct hlist_node *n;
	int i;

	hash_for_each_safe(proc->dma_bufs, i, n, tmp, head) {
		fput(tmp->dmabuf->file);
		hash_del(&tmp->head);
		kfree(tmp);
	}
	kfree(proc);
}

static void write_proc(struct seq_file *s, struct dma_proc *proc)
{
	struct dma_info *tmp;
	int i;

	seq_printf(s, "\n%s (PID %d) size(kB): %ld\nDMA Buffers:\n",
		proc->name, proc->pid, proc->size / SZ_1K);
	seq_printf(s, "%-8s\t%-8s\t%-8s\t%-10s\t%-10s\t%-s\n",
		"ino", "size", "count", "flags", "mode", "exp_name");

	hash_for_each(proc->dma_bufs, i, tmp, head) {
		struct dma_buf *dmabuf = tmp->dmabuf;

		spin_lock((spinlock_t *)&dmabuf->name_lock);
		seq_printf(s, "%08lu\t%-8zu\t%-8ld\t0x%08x\t0x%08x\t%-s\t%-s\n",
			file_inode(dmabuf->file)->i_ino,
			dmabuf->size / SZ_1K,
			file_count(dmabuf->file),
			dmabuf->file->f_flags, dmabuf->file->f_mode,
			dmabuf->exp_name,
			dmabuf->name ?: "");
		spin_unlock((spinlock_t *)&dmabuf->name_lock);
	}
}

static int get_dma_info(const void *data, struct file *file, unsigned int n)
{
	struct dma_proc *dma_proc;
	struct dma_info *dma_info;
	dma_proc = (struct dma_proc *)data;
	if (!is_dma_buf_file(file))
		return 0;

	hash_for_each_possible(dma_proc->dma_bufs, dma_info,
			       head, (unsigned long)file->private_data)
		if (file->private_data == dma_info->dmabuf)
			return 0;

	dma_info = kzalloc(sizeof(*dma_info), GFP_ATOMIC);
	if (!dma_info)
		return -ENOMEM;

	get_file(file);
	dma_info->dmabuf = file->private_data;
	dma_proc->size += dma_info->dmabuf->size;
	hash_add(dma_proc->dma_bufs, &dma_info->head,
			(unsigned long)dma_info->dmabuf);
	return 0;
}

static int dma_buf_show(const struct dma_buf *buf_obj, void *private)
{
	int ret;
	struct dma_buf_attachment *attach_obj;
	struct dma_resv *robj;
	struct dma_resv_list *fobj;
	struct dma_fence *fence;
	unsigned seq;
	int attach_count, shared_count, i;
	struct dma_buf_priv *buf = (struct dma_buf_priv *)private;
	struct seq_file *s = buf->s;

	ret = dma_resv_lock_interruptible(buf_obj->resv, NULL);

	if (ret)
		goto err;

	spin_lock((spinlock_t *)&buf_obj->name_lock);
	seq_printf(s, "%08zu\t%08x\t%08x\t%08ld\t%s\t%08lu\t%s\n",
		   buf_obj->size,
		   buf_obj->file->f_flags, buf_obj->file->f_mode,
		   file_count(buf_obj->file),
		   buf_obj->exp_name,
		   file_inode(buf_obj->file)->i_ino,
		   buf_obj->name ?: "");
	spin_unlock((spinlock_t *)&buf_obj->name_lock);

	robj = buf_obj->resv;
	while (true) {
		seq = read_seqcount_begin(&robj->seq);
		rcu_read_lock();
		fobj = rcu_dereference(robj->fence);
		shared_count = fobj ? fobj->shared_count : 0;
		fence = rcu_dereference(robj->fence_excl);
		if (!read_seqcount_retry(&robj->seq, seq))
			break;
		rcu_read_unlock();
	}

	if (fence)
		seq_printf(s, "\tExclusive fence: %s %s %ssignalled\n",
			   fence->ops->get_driver_name(fence),
			   fence->ops->get_timeline_name(fence),
			   dma_fence_is_signaled(fence) ? "" : "un");
	for (i = 0; i < shared_count; i++) {
		fence = rcu_dereference(fobj->shared[i]);
		if (!dma_fence_get_rcu(fence))
			continue;
		seq_printf(s, "\tShared fence: %s %s %ssignalled\n",
			   fence->ops->get_driver_name(fence),
			   fence->ops->get_timeline_name(fence),
			   dma_fence_is_signaled(fence) ? "" : "un");
		dma_fence_put(fence);
	}
	rcu_read_unlock();

	seq_puts(s, "\tAttached Devices:\n");
	attach_count = 0;

	list_for_each_entry(attach_obj, &buf_obj->attachments, node) {
		seq_printf(s, "\t%s\n", dev_name(attach_obj->dev));
		attach_count++;
	}
	dma_resv_unlock(buf_obj->resv);

	seq_printf(s, "Total %d devices attached\n\n", attach_count);

	buf->count++;
	buf->size += buf_obj->size;

	return 0;
err:
	return ret;
}


static int show_all_dma_thread(struct seq_file *m, void *p)
{
	struct dma_buf_priv dma_buf_priv;

	dma_buf_priv.count = 0;
	dma_buf_priv.size = 0;
	dma_buf_priv.s = m;

	seq_puts(m, "\nDma-buf Objects:\n");
	seq_printf(m, "%-8s\t%-8s\t%-8s\t%-8s\texp_name\t%-8s\n",
		   "size", "flags", "mode", "count", "ino");

	get_each_dmabuf(dma_buf_show, &dma_buf_priv);

	seq_printf(m, "\nTotal %d objects, %zu bytes\n",
		   dma_buf_priv.count, dma_buf_priv.size);
	return 0;
}

static int proccmp(void *unused, struct list_head *a, struct list_head *b)
{
	struct dma_proc *a_proc, *b_proc;
	a_proc = list_entry(a, struct dma_proc, head);
	b_proc = list_entry(b, struct dma_proc, head);
	return b_proc->size - a_proc->size;
}

static int dma_procs_debug_show(struct seq_file *s, void *unused)
{
	struct task_struct *task, *thread;
	struct files_struct *files;
	int ret = 0;
	struct dma_proc *tmp, *n;
	LIST_HEAD(plist);

	rcu_read_lock();
	for_each_process(task) {
		struct files_struct *group_leader_files = NULL;

		tmp = kzalloc(sizeof(*tmp), GFP_ATOMIC);
		if (!tmp) {
			ret = -ENOMEM;
			rcu_read_unlock();
			goto mem_err;
		}
		hash_init(tmp->dma_bufs);
		for_each_thread(task, thread) {
			task_lock(thread);
			if (unlikely(!group_leader_files))
				group_leader_files = task->group_leader->files;
			files = thread->files;
			if (files && (group_leader_files != files ||
				      thread == task->group_leader))
				ret = iterate_fd(files, 0, get_dma_info, tmp);
			task_unlock(thread);
		}

		if (ret || hash_empty(tmp->dma_bufs))
			goto skip;

		get_task_comm(tmp->name, task);
		tmp->pid = task->tgid;
		list_add(&tmp->head, &plist);
		continue;
skip:
		free_proc(tmp);
	}
	rcu_read_unlock();
	list_sort(NULL, &plist, proccmp);
	list_for_each_entry(tmp, &plist, head)
		write_proc(s, tmp);

	ret = 0;
mem_err:
	list_for_each_entry_safe(tmp, n, &plist, head) {
		list_del(&tmp->head);
		free_proc(tmp);
	}
	return ret;
}

static int show_all_tasks_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_all_process_thread, NULL);
}

static int show_all_dma_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_all_dma_thread, NULL);
}

static int show_all_dma_task_open(struct inode *inode, struct file *file)
{
	return single_open(file, dma_procs_debug_show, NULL);
}

static const struct file_operations show_all_tasks_fops = {
	.open       = show_all_tasks_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static const struct file_operations show_all_dma_fops = {
	.open       = show_all_dma_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static const struct file_operations show_all_task_dma_fops = {
	.open       = show_all_dma_task_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int __init nt_meminfo_init(void)
{
	struct proc_dir_entry *root, *pentry;

	root = proc_mkdir("nt_meminfo", NULL);
	if(!root){
		pr_err("mkdir nt_meminfo failed!\n");
		return -1;
	}
	pentry = proc_create("show_all_tasks", S_IRUGO, root, &show_all_tasks_fops);
	if(!pentry) {
		pr_err("create node show_all_tasks node failed!\n");
		return -1;
	}
	pentry = proc_create("show_all_dmabuf", S_IRUGO, root, &show_all_dma_fops);
	if(!pentry) {
		pr_err("create node dump_all_dma node failed!\n");
		return -1;
	}
	pentry = proc_create("show_all_task_dma", S_IRUGO, root, &show_all_task_dma_fops);
	if(!pentry) {
		pr_err("create node show_all_task_dma node failed!\n");
		return -1;
	}

	return 0;
}
device_initcall(nt_meminfo_init);

MODULE_LICENSE("GPL v2");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("<BSP_CORE@nothing.tech>");
MODULE_DESCRIPTION("NOTHING memory information");