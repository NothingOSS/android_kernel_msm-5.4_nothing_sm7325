#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int secure_state = 0;

static int secure_state_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n",secure_state);
	return 0;
}

static int secure_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, secure_state_show, inode->i_private);
}

static const struct file_operations secure_state_fops = {
	.open = secure_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init init_secure_state(void)
{
	struct proc_dir_entry *dir;
	dir = proc_create("secure_state", 0444, NULL, &secure_state_fops);
	if (!dir) {
		pr_alert("Now in %s. proc_create.dir = %p\n", __func__, dir);
		return -1;
	}
	return 0;
}

static int __init setup_secure_state(char *str)
{
	int id;
	if (get_option(&str, &id)) {
		secure_state = id;
		return 0;
	}
	return -EINVAL;
}

early_param("is_secure_fuse", setup_secure_state);
module_init(init_secure_state);
