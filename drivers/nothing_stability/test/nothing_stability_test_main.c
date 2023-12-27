#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>

#include "hungtask_test_driver.h"

#define nt_stability_test_print(fmt,arg...) \
pr_err("[nt_stability_test] "fmt, ##arg)

#define PROC_NAME "nothing_stability_test"

static int __init nothing_stability_test_init(void)
{
	struct proc_dir_entry *nt_stability_test_dir;

	nt_stability_test_print("!!! Testing Only !!!\n");
	nt_stability_test_print("Shouldn't having such module for production use!\n");

	nt_stability_test_dir = proc_mkdir(PROC_NAME, NULL);
	if (nt_stability_test_dir) {
		hungtask_test_init(nt_stability_test_dir);
	}
	return 0;
}

module_init(nothing_stability_test_init);

static void __exit nothing_stability_test_exit(void)
{
	nt_stability_test_print("module exit\n");
}

module_exit(nothing_stability_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("<BSP_CORE@nothing.tech>");
MODULE_DESCRIPTION("nothing stability tester");
