#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/reboot.h>

#include <soc/qcom/restart.h>

#define MAX_SZ_DIAG_ERR_MSG 	200

struct nothing_restart_notify {
	struct notifier_block panic_nb;
	struct notifier_block reboot_nb;
};

struct nothing_restart_info {
	char msg;
};

static struct nothing_restart_info *restart_info;
static unsigned rst_msg_size;

static inline void set_restart_msg(const char *msg)
{
	if (msg) {
		pr_info("%s: %s \n", __func__, msg);
		strncpy(&restart_info->msg, msg, rst_msg_size - 1);
	}
	else {
		strncpy(&restart_info->msg, "", rst_msg_size - 1);
	}
	mb();
}

static int nothing_notifier_panic(struct notifier_block *this, unsigned long event,
			      void *ptr)
{
	char kernel_panic_msg[MAX_SZ_DIAG_ERR_MSG] = "Kernel Panic";
	pr_warn("%s: Get panic notify", __func__);
	if (ptr)
		snprintf(kernel_panic_msg, rst_msg_size - 1, "%s", (char *)ptr);
	set_restart_msg(kernel_panic_msg);
	return NOTIFY_OK;
}

static int nothing_notifier_reboot(struct notifier_block *this, unsigned long event,
			      void *ptr)
{
	char reboot_msg[MAX_SZ_DIAG_ERR_MSG];
	pr_warn("%s: Get reboot notify", __func__);
	if (ptr)
		snprintf(reboot_msg, rst_msg_size - 1, "Reboot: %s", (char *)ptr);
	set_restart_msg(reboot_msg);

	return NOTIFY_OK;
}

static int nothing_restart_info_probe(struct platform_device *pdev)
{
	struct device_node *np;
	unsigned nothing_rst_info_size;
	int ret = 0;
	struct nothing_restart_notify *restart_notify;

	pr_info("%s Start \n", __func__);

	restart_notify = devm_kzalloc(&pdev->dev, sizeof(*restart_notify), GFP_KERNEL);
	if (!restart_notify) {
		pr_err("%s: Unable to allocate memory for restart_notify\n", __func__);
		return 0;
	}

	np = of_find_compatible_node(NULL, NULL,
				"nothing,msm-imem-restart_info");
	if (!np) {
		pr_err("unable to find imem nothing,imem-restart-info\n");
		return 0;
	} else {
		restart_info = of_iomap(np, 0);
		if(!restart_info) {
			pr_err("%s: nothing restart_info not valid\n");
			return 0;
		} else {
			ret = of_property_read_u32(np, "info_size", &nothing_rst_info_size);
			if (ret != 0) {
				pr_err("%s: Failed to find info_size property in nothing restart info device node %d\n"
					, __func__, ret);
				return 0;
				}
		}
	}
	rst_msg_size = nothing_rst_info_size - offsetof(struct nothing_restart_info, msg);
	if (rst_msg_size > MAX_SZ_DIAG_ERR_MSG)
		rst_msg_size = MAX_SZ_DIAG_ERR_MSG;

	set_restart_msg("Unknown");

	restart_notify->panic_nb.notifier_call = nothing_notifier_panic;
	restart_notify->panic_nb.priority = INT_MAX;
	atomic_notifier_chain_register(&panic_notifier_list,
				       &restart_notify->panic_nb);

	restart_notify->reboot_nb.notifier_call = nothing_notifier_reboot;
	restart_notify->reboot_nb.priority = 255;
	register_reboot_notifier(&restart_notify->reboot_nb);

	platform_set_drvdata(pdev, restart_notify);

	return 0;
}

static int nothing_restart_info_remove(struct platform_device *pdev)
{
	struct nothing_restart_notify *restart_notify = platform_get_drvdata(pdev);

	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &restart_notify->panic_nb);
	unregister_reboot_notifier(&restart_notify->reboot_nb);

	if (restart_info)
		iounmap(restart_info);

	return 0;
}

static const struct of_device_id of_nothing_restart_info_match[] = {
	{ .compatible = "qcom,msm-imem", },
	{},
};
MODULE_DEVICE_TABLE(of, of_nothing_restart_info_match);

static struct platform_driver nothing_restart_info_driver = {
	.probe = nothing_restart_info_probe,
	.remove = nothing_restart_info_remove,
	.driver = {
		.name = "nothing-restart-info",
		.of_match_table = of_nothing_restart_info_match,
	},
};

static int __init nothing_restart_info_driver_init(void)
{
	pr_warn("nothing_restart_info_driver_init\n");

	return platform_driver_register(&nothing_restart_info_driver);
}

module_init(nothing_restart_info_driver_init);

static void __exit nothing_restart_info_driver_exit(void)
{
	return platform_driver_unregister(&nothing_restart_info_driver);
}
module_exit(nothing_restart_info_driver_exit);

MODULE_DESCRIPTION("Nothing Restart Info Driver");
MODULE_LICENSE("GPL v2");
