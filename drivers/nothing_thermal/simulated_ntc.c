// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Nothing. All rights reserved.
 */

#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/thermal.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/spinlock.h>
#include <linux/seq_file.h>
#include <linux/version.h>

enum {
	SHELL_FRONT = 0,
	SHELL_FRAME,
	SHELL_BACK,
	SHELL_MAX,
};

/* trips config */
#define TRIPS_NUM 2
#define TRIPS_HYST 1000
#define TRIPS_TEMP 125000

static DEFINE_IDA(shell_temp_ida);
static DEFINE_SPINLOCK(slntc_lock);
static int shell_temp[SHELL_MAX];

struct sl_thermal_trip {
        int temperature;
        int hysteresis;
        enum thermal_trip_type type;
};


struct slntc_shell_temp {
	struct thermal_zone_device *tzd;
	int shell_id;

	/* trip data */
	int ntrips;
	struct sl_thermal_trip *trips;
	int prev_low_trip;
	int prev_high_trip;
	int prev_temp;

	spinlock_t trips_lock;
};

static struct slntc_shell_temp *shell_list[SHELL_MAX];

static const struct of_device_id slntc_shell_of_match[] = {
	{ .compatible = "nothing,simulated-ntc" },
	{},
};

static int slntc_get_shell_temp(struct thermal_zone_device *tz,
					int *temp)
{
	struct slntc_shell_temp *hst;

	if (!temp || !tz)
		return -EINVAL;

	hst = tz->devdata;
	*temp = shell_temp[hst->shell_id];
	return 0;
}

static int slntc_get_trip_type(struct thermal_zone_device *tz, int trip,
				    enum thermal_trip_type *type)
{
	struct slntc_shell_temp *hst = tz->devdata;

	if (trip >= hst->ntrips || trip < 0)
		return -EDOM;

	*type = hst->trips[trip].type;

	return 0;
}

static int slntc_get_trip_temp(struct thermal_zone_device *tz, int trip,
				    int *temp)
{
	struct slntc_shell_temp *hst = tz->devdata;

	if (trip >= hst->ntrips || trip < 0)
		return -EDOM;

	*temp = hst->trips[trip].temperature;

	return 0;
}

static int slntc_set_trip_temp(struct thermal_zone_device *tz, int trip,
				    int temp)
{
	struct slntc_shell_temp *hst = tz->devdata;
	unsigned long flags;

	if (trip >= hst->ntrips || trip < 0)
		return -EDOM;

	spin_lock_irqsave(&hst->trips_lock, flags);
	hst->trips[trip].temperature = temp;
	spin_unlock_irqrestore(&hst->trips_lock, flags);

	pr_info("trips[trip].temperature = %d", trip, temp);

	return 0;
}

static int slntc_get_trip_hyst(struct thermal_zone_device *tz, int trip,
				    int *hyst)
{
	struct slntc_shell_temp *hst = tz->devdata;

	if (trip >= hst->ntrips || trip < 0)
		return -EDOM;

	*hyst = hst->trips[trip].hysteresis;
	return 0;
}

static int slntc_set_trip_hyst(struct thermal_zone_device *tz, int trip,
				    int hyst)
{
	struct slntc_shell_temp *hst = tz->devdata;
	unsigned long flags;

	if (trip >= hst->ntrips || trip < 0)
		return -EDOM;

	spin_lock_irqsave(&hst->trips_lock, flags);
	hst->trips[trip].hysteresis = hyst;
	spin_unlock_irqrestore(&hst->trips_lock, flags);

	pr_info("trips[trip].hysteresis = %d", trip, hyst);

	return 0;
}

struct thermal_zone_device_ops shell_thermal_zone_ops = {
	.get_temp = slntc_get_shell_temp,
	.get_trip_type = slntc_get_trip_type,
	.get_trip_temp = slntc_get_trip_temp,
	.set_trip_temp = slntc_set_trip_temp,
	.get_trip_hyst = slntc_get_trip_hyst,
	.set_trip_hyst = slntc_set_trip_hyst,
};

static struct thermal_zone_params sltntc_thermal_params = {
    .governor_name = "user_space",
    .no_hwmon = true,
};

static int build_trips(struct slntc_shell_temp *hst)
{
	int i, ret = 0;

	/*2 trips for qcom thermal-engine and thermal-hal*/
	hst->ntrips = TRIPS_NUM;

	hst->trips = kcalloc(hst->ntrips, sizeof(*hst->trips), GFP_KERNEL);
	if (!hst->trips) {
		ret = -ENOMEM;
		return ret;
	}

	/*Initialize the trip configuration*/
	for(i = 0; i < hst->ntrips; i++){
		hst->trips[i].hysteresis = TRIPS_HYST;
		hst->trips[i].temperature = TRIPS_TEMP;
		hst->trips[i].type = THERMAL_TRIP_PASSIVE;
	}

	return ret;
}

static int slntc_shell_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dev_node = dev->of_node;
	struct thermal_zone_device *tz_dev;
	struct slntc_shell_temp *hst;
	int ret = 0;
	int mask = 0;
	int result, i;

	if (!of_device_is_available(dev_node)) {
		pr_err("shell-temp dev not found\n");
		return -ENODEV;
	}

	hst = kzalloc(sizeof(struct slntc_shell_temp), GFP_KERNEL);
	if (!hst) {
		pr_err("alloc slntc mem failed\n");
		return -ENOMEM;
	}

	result = ida_simple_get(&shell_temp_ida, 0, 0, GFP_KERNEL);
	if (result < 0) {
		pr_err("genernal slntc id failed\n");
		ret = -EINVAL;
		goto err_free_mem;
	}

	hst->shell_id = result;

	result = build_trips(hst);
	if(result < 0){
		pr_err("build_trips failed\n");
		ret = -EINVAL;
		goto err_remove_id;
	}

	for (i = 0; i < hst->ntrips; i++)
		mask |= 1 << i;

	tz_dev = thermal_zone_device_register(dev_node->name,
			hst->ntrips, mask, hst, &shell_thermal_zone_ops, &sltntc_thermal_params, 0, 0);
	if (IS_ERR_OR_NULL(tz_dev)) {
		pr_err("register thermal zone for shell failed\n");
		ret = -ENODEV;
		goto err_free_trip;
	}
	hst->tzd = tz_dev;
	hst->prev_high_trip = INT_MAX;
	hst->prev_low_trip = -INT_MAX;
	hst->prev_temp = 0;
	spin_lock_init(&hst->trips_lock);

	shell_list[hst->shell_id] = hst;

	platform_set_drvdata(pdev, hst);

	return 0;
err_free_trip:
	kfree(hst->trips);
err_remove_id:
	ida_simple_remove(&shell_temp_ida, result);
err_free_mem:
	kfree(hst);
	return ret;
}

static int slntc_shell_remove(struct platform_device *pdev)
{
	struct slntc_shell_temp *hst = platform_get_drvdata(pdev);

	if (hst) {
		platform_set_drvdata(pdev, NULL);
		thermal_zone_device_unregister(hst->tzd);
		if(hst->trips)
			kfree(hst->trips);
		kfree(hst);
	}

	return 0;
}

static struct platform_driver slntc_shell_platdrv = {
	.driver = {
		.name		= "slntc-shell-temp",
		.owner		= THIS_MODULE,
		.of_match_table = slntc_shell_of_match,
	},
	.probe	= slntc_shell_probe,
	.remove = slntc_shell_remove,
};

#define BUF_LEN		256
static ssize_t proc_shell_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *pos)
{
	int ret, temp, len, i;
	unsigned int index = 0;
	char tmp[BUF_LEN + 1];
	unsigned long flags;
	int low = -INT_MAX;
	int high = INT_MAX;
	bool is_trigeer = false;

	if (count == 0)
		return 0;

	len = count > BUF_LEN ? BUF_LEN : count;

	ret = copy_from_user(tmp, buf, len);
	if (ret) {
		pr_err("copy_from_user failed, ret=%d\n", ret);
		return count;
	}

	if (tmp[len - 1] == '\n')
		tmp[len - 1] = '\0';
	else
		tmp[len] = '\0';

	ret = sscanf(tmp, "%d %d", &index, &temp);
	if (ret < 2) {
		pr_err("write failed, ret=%d\n", ret);
		return count;
	}

	if (index >= SHELL_MAX) {
		pr_err("write invalid para\n");
		return count;
	}

	if(shell_list[index]->trips != NULL){
		spin_lock_irqsave(&shell_list[index]->trips_lock, flags);
		for(i = 0; i < TRIPS_NUM; i++){
			int trip_low, trip_high;

			trip_low = shell_list[index]->trips[i].temperature - shell_list[index]->trips[i].hysteresis;
			trip_high = shell_list[index]->trips[i].temperature;

			if (trip_low < temp && trip_low > low)
				low = trip_low;

			if (trip_high > temp && trip_high < high)
				high = trip_high;

		}

		if(temp >= shell_list[index]->prev_high_trip || temp <= shell_list[index]->prev_low_trip){
			pr_info("trigger is true !shell_list[%d]: prev_low_trip = %d, prev_high_trip = %d\n", index, shell_list[index]->prev_low_trip, shell_list[index]->prev_high_trip);
			pr_info("shell_list[%d]: trip_low = %d, trip_high = %d, temp = %d\n", index, low, high, temp);
			is_trigeer = true;
		}

		shell_list[index]->prev_low_trip = low;
		shell_list[index]->prev_high_trip = high;

		spin_unlock_irqrestore(&shell_list[index]->trips_lock, flags);
	}

	if(abs(shell_list[index]->prev_temp -temp) >= 1000)
		is_trigeer = true;

	spin_lock_irqsave(&slntc_lock, flags);
	shell_temp[index] = temp;
	spin_unlock_irqrestore(&slntc_lock, flags);
	shell_list[index]->prev_temp = temp;

	if(is_trigeer && shell_list[index]->tzd != NULL)
		thermal_zone_device_update(shell_list[index]->tzd, THERMAL_TRIP_VIOLATED);

	return count;
}

static int proc_shell_show(struct seq_file *m, void *v)
{
	int temp = 0, index;
	unsigned long flags;

	spin_lock_irqsave(&slntc_lock, flags);
	for (index = 0; index < SHELL_MAX; index++) {
		if (shell_temp[index] > temp)
			temp = shell_temp[index];
	}
	spin_unlock_irqrestore(&slntc_lock, flags);
	seq_printf(m, "%d", temp);
	return 0;
}

static int proc_shell_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_shell_show, NULL);
}


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
static const struct proc_ops proc_shell_fops = {
	.proc_open = proc_shell_open,
	.proc_write = proc_shell_write,
	.proc_read = seq_read,
	.proc_release = single_release,
};
#else
static const struct file_operations proc_shell_fops = {
	.open = proc_shell_open,
	.write = proc_shell_write,
	.read = seq_read,
	.release = single_release,
};
#endif

static int __init slntc_shell_init(void)
{
	struct proc_dir_entry *shell_proc_entry;

	shell_proc_entry = proc_create("shell-temp", 0666, NULL, &proc_shell_fops);
	if (!shell_proc_entry) {
		pr_err("shell-temp proc create failed\n");
		return -EINVAL;
	}

	spin_lock_init(&slntc_lock);

	return platform_driver_register(&slntc_shell_platdrv);
}

static void __exit slntc_shell_exit(void)
{
	platform_driver_unregister(&slntc_shell_platdrv);
}


module_init(slntc_shell_init);
module_exit(slntc_shell_exit);
MODULE_LICENSE("GPL v2");
