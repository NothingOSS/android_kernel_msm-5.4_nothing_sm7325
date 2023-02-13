#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/sched/clock.h>
#include <linux/oom.h>
#include <linux/uaccess.h>
#include <linux/mm.h>

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

static int show_all_tasks_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_all_process_thread, NULL);
}

static const struct file_operations show_all_tasks_fops = {
	.open     = show_all_tasks_open,
	.read     = seq_read,
	.llseek    = seq_lseek,
	.release  = single_release,
};

static int __init nt_meminfo_init(void)
{
	struct proc_dir_entry *root, *pentry;

	root = proc_mkdir("nt_meminfo", NULL);
	if(!root){
		pr_err("mkdir nt_meminfo failed!\n");
		return -1;
	}
	pentry = proc_create("show_all_tasks", S_IRUGO|S_IWUGO, root, &show_all_tasks_fops);
	if(!pentry) {
		pr_err("create node dump_all_tasks node failed!\n");
		return -1;
	}
	return 0;
}
device_initcall(nt_meminfo_init);

MODULE_LICENSE("GPL v2");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("<BSP_CORE@nothing.tech>");
MODULE_DESCRIPTION("NOTHING memory information");