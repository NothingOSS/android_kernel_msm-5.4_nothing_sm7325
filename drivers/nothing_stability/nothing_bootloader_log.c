
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/pstore.h>

#define BOOTLOADER_LOGGER_COMPATIBLE_NAME "nothing,bootloader_log"
#define PROC_NAME "bootloader_log"

char *log_buf;
unsigned long log_buf_len;

static int bootloader_logger_proc_show(struct seq_file *m, void *v)
{
	log_buf[log_buf_len-1] = '\0';
	seq_printf(m, "%s\n", log_buf);
	return 0;
}

static int bootloader_logger_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bootloader_logger_proc_show, NULL);
}

static struct file_operations bootloader_logger_proc_fops = {
	.open		= bootloader_logger_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int bootloader_logger_proc_init(void)
{
	proc_create(PROC_NAME, 0, NULL, &bootloader_logger_proc_fops);
	return 0;
}

static int bootloader_logger_probe(struct platform_device *pdev)
{
	struct resource res;
	struct device_node *np;
	phys_addr_t phys_addr;
	char* virt_addr;
	unsigned long mem_size;
	int rc;
	u32 log_offset_size[2];

	pr_warn("[%s]: Enterd\n", __func__);

	if(pdev->dev.of_node) {
		np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
		if (!np) {
			pr_err("[%s]: no memory-region node\n", __func__);
			goto err;
		}
		rc = of_address_to_resource(np, 0, &res);
		if (rc) {
			pr_err("[%s]: failed to get reserve-memory resource\n", __func__);
			goto err;
		}
		of_node_put(np);


		rc = of_property_read_u32_array(pdev->dev.of_node, "log,offset_size", log_offset_size, 2);
		if (rc < 0) {
			pr_err("[%s]: Cannot get log,offset_size resource, directly use memory-region\n", __func__);
			mem_size = resource_size(&res);
			phys_addr = res.start;
		}
		else {
			// base address of reserved_xbl_uefi + offset
			pr_err("[%s]: Get log offset = %x\n", __func__, log_offset_size[0]);
			phys_addr = res.start + log_offset_size[0];
			mem_size = log_offset_size[1];
		}

		virt_addr = ioremap(phys_addr, mem_size);
		log_buf = kzalloc(mem_size, GFP_KERNEL);
		if(!log_buf) {
			pr_err("[%s]: failed to alloc log_buf\n");
			return -ENOMEM;
		}
		log_buf_len = mem_size;
		memcpy(log_buf, virt_addr, mem_size);

		bootloader_logger_proc_init();
	}
	pr_warn("[%s]: End\n", __func__);
	return 0;
err:
	return -ENOMEM;
}

static int bootloader_logger_remove(struct platform_device *pdev)
{
	remove_proc_entry(PROC_NAME, NULL);
	kfree(log_buf);
	return 0;
}

static const struct of_device_id bootloader_logger_id[] = {
	{ .compatible = BOOTLOADER_LOGGER_COMPATIBLE_NAME, },
	{ },
};
MODULE_DEVICE_TABLE(of, bootloader_logger_id);

/* Description of bootloader_logger driver */
static struct platform_driver bootloader_logger = {
	.probe = bootloader_logger_probe,
	.remove = bootloader_logger_remove,
	.driver = {
		.name = BOOTLOADER_LOGGER_COMPATIBLE_NAME,
		.of_match_table = bootloader_logger_id,
	},
};

static int __init bootloader_logger_init(void)
{
	int ret;
	pr_warn("%s: Hello. NOTHING_BOOTLODER_LOG !\n", __func__);
	ret = platform_driver_register(&bootloader_logger);
	if(ret != 0)
		platform_driver_unregister(&bootloader_logger);
	return ret;
}

postcore_initcall(bootloader_logger_init);

static void __exit bootloader_logger_exit(void)
{
	platform_driver_unregister(&bootloader_logger);
}
module_exit(bootloader_logger_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("<BSP_CORE@nothing.tech>");
MODULE_DESCRIPTION("bootloader logger/driver");
