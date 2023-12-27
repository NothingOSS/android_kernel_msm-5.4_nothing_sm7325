/*
 * Nothing Kernel Logger helper
 *
 */

#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#define PROC_BUF_SIZE 256

#define NOTH_LOGGER_HELPER_DEVICE_NAME "noth_logger"
#define NOTH_LOGGER_HELPER_ALARM_NODE "alarm"
#define NOTH_LOGGER_HELPER_STANDBY_NODE "standby"

#define NOTH_LOGGER_INFO(fmt, args...) do { \
	printk(KERN_INFO "[NOTH/I]%s:"fmt"\n", __func__, ##args); \
} while (0)

#define NOTH_LOGGER_ERROR(fmt, args...) do { \
	printk(KERN_ERR "[NOTH/E]%s:"fmt"\n", __func__, ##args); \
} while (0)

struct noth_logger_helper_data {
	struct proc_dir_entry *noth_logger_helper_entry;
	struct proc_dir_entry *alarm_logger_entry;
	struct proc_dir_entry *standby_logger_entry;
};

struct noth_logger_helper_data local_noth_logger_helper_data;
int noth_alarm_logger = 0;
int noth_standby_logger = 0;

EXPORT_SYMBOL(noth_alarm_logger);
EXPORT_SYMBOL(noth_standby_logger);

static ssize_t alarm_logger_read(
	struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	return seq_read(filp, buff, count, ppos);
}

static ssize_t alarm_logger_write(
	struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	char buffs[PROC_BUF_SIZE];
	int noth_logger_input = 0;

	if (count > sizeof(buffs))
		count = sizeof(buffs) - 1;

	if (copy_from_user(buffs, buff, count))
		return -EFAULT;

	sscanf(buffs, "%d", &noth_logger_input);

	noth_alarm_logger = (noth_logger_input != 0) ? 1 : 0;

	NOTH_LOGGER_INFO("alarm loggger: %d", noth_alarm_logger);

	return count;
}

static const struct file_operations alarm_logger_fops = {
	.owner  = THIS_MODULE,
	.read   = alarm_logger_read,
	.write  = alarm_logger_write,
};

static ssize_t standby_logger_read(
	struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	return seq_read(filp, buff, count, ppos);
}

static ssize_t standby_logger_write(
	struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	char buffs[PROC_BUF_SIZE];
	int noth_logger_input = 0;

	if (count > sizeof(buffs))
		count = sizeof(buffs) - 1;

	if (copy_from_user(buffs, buff, count))
		return -EFAULT;

	sscanf(buffs, "%d", &noth_logger_input);

	noth_standby_logger = (noth_logger_input != 0) ? 1 : 0;

	NOTH_LOGGER_INFO("standby loggger: %d", noth_standby_logger);

	return count;
}

static const struct file_operations standby_logger_fops = {
	.owner  = THIS_MODULE,
	.read   = standby_logger_read,
	.write  = standby_logger_write,
};

static int noth_logger_helper_create_proc_node(struct noth_logger_helper_data *data)
{
	data->noth_logger_helper_entry = proc_mkdir(NOTH_LOGGER_HELPER_DEVICE_NAME, NULL);
	if (data->noth_logger_helper_entry == NULL) {
		NOTH_LOGGER_INFO("fail to create dir %s", NOTH_LOGGER_HELPER_DEVICE_NAME);
		return -ENOMEM;
	}
	data->alarm_logger_entry = proc_create_data(NOTH_LOGGER_HELPER_ALARM_NODE, (S_IWUSR|S_IWGRP), data->noth_logger_helper_entry, &alarm_logger_fops, data);
	if (data->alarm_logger_entry == NULL) {
		NOTH_LOGGER_ERROR("create alarm proc entry fail");
		return -ENOMEM;
	}
	data->standby_logger_entry = proc_create_data(NOTH_LOGGER_HELPER_STANDBY_NODE, (S_IWUSR|S_IWGRP), data->noth_logger_helper_entry, &standby_logger_fops, data);
	if (data->standby_logger_entry == NULL) {
		NOTH_LOGGER_ERROR("create standby proc entry fail");
		return -ENOMEM;
	}

	return 0;
}

static int noth_logger_helper_release_proc_node(struct noth_logger_helper_data *data)
{
	if (data->alarm_logger_entry) {
		proc_remove(data->alarm_logger_entry);
		data->alarm_logger_entry = NULL;
	}
	if (data->standby_logger_entry) {
		proc_remove(data->standby_logger_entry);
		data->standby_logger_entry = NULL;
	}
	if (data->noth_logger_helper_entry) {
		proc_remove(data->noth_logger_helper_entry);
		data->noth_logger_helper_entry = NULL;
	}

	return 0;
}

static int noth_logger_helper_probe(struct platform_device *dev)
{
	NOTH_LOGGER_INFO("noth_logger_helper_probe");

	noth_logger_helper_create_proc_node(&local_noth_logger_helper_data);

	return 0;
}

static int noth_logger_helper_remove(struct platform_device *dev)
{
	NOTH_LOGGER_INFO("noth_logger_helper_remove");

	noth_logger_helper_release_proc_node(&local_noth_logger_helper_data);

	return 0;
}

static struct platform_driver noth_logger_helper_driver= {
	.driver = {
		.name = NOTH_LOGGER_HELPER_DEVICE_NAME,
	},
	.probe = noth_logger_helper_probe,
	.remove = noth_logger_helper_remove,
};

static struct platform_device noth_logger_helper_device =
{
	.name = NOTH_LOGGER_HELPER_DEVICE_NAME,
	.id = -1,
};

static int noth_logger_helper_init(void)
{
	int ret = 0;

	NOTH_LOGGER_INFO("noth_logger_helper_init");
	ret = platform_device_register(&noth_logger_helper_device);
	if (ret) {
		NOTH_LOGGER_ERROR("noth_logger_helper_device ERROR(%d)", ret);
	}

	ret = platform_driver_register(&noth_logger_helper_driver);
	if (ret) {
		NOTH_LOGGER_ERROR("noth_logger_helper_driver ERROR(%d)", ret);
	}

	return ret;
}

static void noth_logger_helper_exit(void)
{
	NOTH_LOGGER_INFO("noth_logger_helper_exit");

	platform_driver_unregister(&noth_logger_helper_driver);
	platform_device_unregister(&noth_logger_helper_device);

	return;
}

MODULE_LICENSE("GPL");
module_init(noth_logger_helper_init);
module_exit(noth_logger_helper_exit);
