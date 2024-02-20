#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <linux/mutex.h>

DEFINE_MUTEX(dlock);

#define hungtask_test_print(fmt,arg...) \
pr_err("[hungtask_test] "fmt, ##arg)

static struct task_struct *tsk;


static int deadlock_test_run(void *args)
{
	hungtask_test_print("deadlock_test_run Start\n");
	mutex_lock(&dlock);
	mutex_lock(&dlock);
	return 0;
}

static ssize_t write_deadlock_node(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if (count) {
		char proc_data;
		if (get_user(proc_data, buf))
			return -EFAULT;

		if (proc_data == '1') {
			tsk = kthread_run(deadlock_test_run, NULL, "deadlock_test");
			if (!tsk)
				hungtask_test_print("kthread init failed\n");
			hungtask_test_print("kthread init done\n");
		}
	}

	return count;
}

static const struct file_operations deadlock_test_proc_fops = {
	.write		= write_deadlock_node,
	.llseek		= noop_llseek,
};

static int hungtask_test_run(void *args)
{
	hungtask_test_print("set task to state-D\n");
	set_current_state(TASK_UNINTERRUPTIBLE);
	hungtask_test_print("schedule out \n");
	schedule();
	hungtask_test_print("wake up!!!!\n");
	return 0;
}

static ssize_t write_hungtask_node(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if (count) {
		char proc_data;
		if (get_user(proc_data, buf))
			return -EFAULT;

		if (proc_data == '1') {
			tsk = kthread_run(hungtask_test_run, NULL, "hungtask_test");
			if (!tsk)
				hungtask_test_print("kthread init failed\n");
			hungtask_test_print("kthread init done\n");
		}
	}

	return count;
}

static const struct file_operations hungtask_test_proc_fops = {
	.write		= write_hungtask_node,
	.llseek		= noop_llseek,
};

int hungtask_test_init(struct proc_dir_entry *nt_stability_test_dir)
{
	if(!proc_create("hungtask_test", 0644, nt_stability_test_dir, &hungtask_test_proc_fops))
		hungtask_test_print("Failed to register hungtask_test node\n");
	if(!proc_create("deadlock_test", 0644, nt_stability_test_dir, &deadlock_test_proc_fops))
		hungtask_test_print("Failed to register deadlock_test node\n");

	return 0;
}

MODULE_LICENSE("GPL v2");
MODULE_LICENSE("Dual BSD/GPL");
