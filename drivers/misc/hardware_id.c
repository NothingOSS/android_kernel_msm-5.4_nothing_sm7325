#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/pm.h>

#include <linux/err.h>
#include <linux/input.h>
#include <linux/jiffies.h>

#include <linux/of_gpio.h>

#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

/*HWID | GPIO19 | GPIO21*/
/* T0     |  0          |  0        */
/* EVT   |  0          |  1        */
/* DVT   |  1          |  0        */
/* PVT   |  1          |  1        */

/*id1: GPIO19*/
/*id2: GPIO21*/

static char *hw_ver[]     = {"T0","EVT","DVT","PVT"};
static int ver_gpioflag = 0;

static int hwid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", hw_ver[ver_gpioflag]);
	return 0;
}

static int hwid_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwid_show, inode->i_private);
}

static const struct file_operations hwid_fops = {
	.open		= hwid_open,
	.read		= seq_read,
};

static int create_hwid_proc_file(void)
{
	struct proc_dir_entry *dir;

	dir = proc_create("hwid", 0440, NULL, &hwid_fops);
	if (!dir) {
		pr_err("Unable to create /proc/hwid");
		return -1;
	}

	return 0;
}

static int hwid_probe(struct platform_device *pdev)
{
	int ret;
	int id1;
	int id2;
	int id1_val;
	int id2_val;

	enum of_gpio_flags flags ;

	printk(KERN_EMERG "enter hwid_probe\n");

	id1 = of_get_named_gpio_flags(pdev->dev.of_node, "gpio_id1", 0, &flags);

	ret = gpio_direction_input(id1);
	if (ret)
	{
		pr_err("set gpio id1 failed\n");
	}

	id1_val = gpio_get_value(id1);

	id2 = of_get_named_gpio_flags(pdev->dev.of_node, "gpio_id2", 0, &flags);

	ret = gpio_direction_input(id2);
	if (ret)
	{
		pr_err("set gpio id2 failed\n");
	}

	id2_val = gpio_get_value(id2);

	ver_gpioflag = (id1_val << 1) | (id2_val);

	printk(KERN_INFO"%s ver_gpioflag is %d\n", __func__,ver_gpioflag);

	/* create proc for hardware id */
	create_hwid_proc_file();

       return 0;
}

static const struct of_device_id hwid_of_match[] = {
	{ .compatible = "nt,hwid", },
	{ },
};

MODULE_DEVICE_TABLE(of, hwid_of_match);
static struct platform_driver hwid_driver = {
	.probe		= hwid_probe,
	.driver		= {
		.name	= "nt-hwid",
		.of_match_table = hwid_of_match,
	}
};

static int __init hwid_init(void)
{
	printk(KERN_EMERG"%s enter.....\n", __func__);
	return platform_driver_register(&hwid_driver);
}

module_init(hwid_init);
MODULE_DESCRIPTION("hardware id function");
