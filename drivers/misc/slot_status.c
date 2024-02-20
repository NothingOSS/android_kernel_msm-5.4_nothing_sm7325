#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define CLASS_NAME "slot_check"

struct slot_info {
	struct platform_device *pdev;
	struct class *slot_class;
	int gpio112;
	int gpio116;
};

static struct slot_info *slot;

static ssize_t gpio112_show(struct class *class, struct class_attribute *attr,
                            char *buf)
{
	int value;
	value = gpio_get_value(slot->gpio112);
	return sprintf(buf, "%d\n",value);
}

static ssize_t gpio116_show(struct class *class, struct class_attribute *attr,
                            char *buf)
{
	int value;
	value = gpio_get_value(slot->gpio116);
	return sprintf(buf, "%d\n",value);
}

static CLASS_ATTR_RO(gpio112);
static CLASS_ATTR_RO(gpio116);

static struct attribute *slot_class_attrs[] = {
		&class_attr_gpio112.attr,
		&class_attr_gpio116.attr,
		NULL,
};

ATTRIBUTE_GROUPS(slot_class);

static struct class slot_class = {
	.name = CLASS_NAME,
	.owner = THIS_MODULE,
	.class_groups = slot_class_groups,
};

static int slot_parse_dt()
{
	slot->gpio112 = of_get_named_gpio(slot->pdev->dev.of_node,"gpio112",0);
	if(slot->gpio112 < 0){
		dev_err(&slot->pdev->dev,"can not get gpio112");
		return -EINVAL;
	}

	slot->gpio116 = of_get_named_gpio(slot->pdev->dev.of_node,"gpio116",0);
	if(slot->gpio116 < 0){
		dev_err(&slot->pdev->dev,"can not get gpio116");
		return -EINVAL;
	}
	return 0;
}

static int slot_class_probe(struct platform_device *pdev){

	int ret;

	slot = kzalloc(sizeof(struct slot_info), GFP_KERNEL);
	slot->pdev = pdev;
	slot->slot_class = &slot_class;

	ret = slot_parse_dt();
	if(ret){
		dev_err(&slot->pdev->dev,"slot_parse_dt failed");
		goto err;
	}

	ret = class_register(&slot_class);
	if(ret){
		dev_err(&slot->pdev->dev,"class_register fail\n");
		goto err;
	}

	return ret;
err:
	kzfree(slot);
	return ret;
}

static const struct of_device_id slot_of_match[] = {
	{ .compatible = "nt,slot_check", },
	{ },
};

MODULE_DEVICE_TABLE(of, slot_of_match);

static struct platform_driver slot_class_driver = {
	.probe		= slot_class_probe,
	.driver		= {
		.name	= "slot_class",
		.of_match_table = slot_of_match,
	}
};
static int __init slot_check_init(void)
{
	return platform_driver_register(&slot_class_driver);
}

static void __exit slot_check_exit(void)
{
	class_unregister(&slot_class);
	platform_driver_unregister(&slot_class_driver);
	kzfree(slot);
}


module_init(slot_check_init);
module_exit(slot_check_exit);
MODULE_DESCRIPTION("slot -check function");
