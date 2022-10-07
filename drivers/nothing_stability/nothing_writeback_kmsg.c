#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/types.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/buffer_head.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/kmsg_dump.h>
#include <linux/mount.h>
#include <linux/kdev_t.h>
#include <linux/uio.h>
#include <linux/printk.h>

#define NT_rkl_info_print(fmt,arg...) \
pr_info("[NT_reserve_kernel_log] "fmt, ##arg)

#define NT_rkl_err_print(fmt,arg...) \
pr_err("[NT_reserve_kernel_log] "fmt, ##arg)

#define RETRY_COUNT_FOR_GET_DEV_T 30
#define PER_LOOP_MS 30

#define NT_MAJOR 14
#define NT_MINOR 7

#define NT_K_MULT 1024
#define NT_M_MULT 1048576
#define NT_512M_MULT (512 * NT_M_MULT)

/* offset list, total 512M */
#define NT_MINIDUMP_OFFSET 0
#define NT_MINIDUMP_SIZE (256 * NT_M_MULT)

#define NT_AP_LOG_OFFSET (NT_MINIDUMP_OFFSET + NT_MINIDUMP_SIZE)
#define NT_AP_LOG_SIZE (128 * NT_M_MULT)

#define NT_RESERVE_N_HEADER_OFFSET (NT_AP_LOG_OFFSET + NT_AP_LOG_SIZE)
#define NT_RESERVE_N_HEADER_SIZE (32 * NT_M_MULT)

#define NT_PANIC_KMSG_LOG_OFFSET (NT_RESERVE_N_HEADER_OFFSET + NT_RESERVE_N_HEADER_SIZE)
#define NT_PANIC_KMSG_LOG_SIZE (32 * NT_M_MULT)

#define NT_BOOT_KMSG_LOG_OFFSET (NT_PANIC_KMSG_LOG_OFFSET + NT_PANIC_KMSG_LOG_SIZE)
#define NT_BOOT_KMSG_LOG_SIZE (32 * NT_M_MULT)

#define NT_BOOTLOADER_LOG_OFFSET (NT_BOOT_KMSG_LOG_OFFSET + NT_BOOT_KMSG_LOG_SIZE)
#define NT_BOOTLOADER_LOG_SIZE (32 * NT_M_MULT)

#define NT_CHECK_LAYOUT_END_OFFSET (NT_BOOTLOADER_LOG_OFFSET + NT_BOOTLOADER_LOG_SIZE)
/* offset list end */

#define PREFIX_BUFFER_SIZE 32
#define NT_PAGE_SIZE 4096


#define BOOT_LOG_SIZE (512 * NT_K_MULT) //per boot log size
#define MAX_BOOT_LOG_COUNT 16 // The boot log can record 16 kmsg logs.
#define BOOT_LOG_PAGES (BOOT_LOG_SIZE / NT_PAGE_SIZE) //per boot log size
#define BOOT_LOG_SIZE_OF_LINE 2048
#define COMPUTE_BOOT_LOG_OFFSET(x) ((NT_BOOT_KMSG_LOG_OFFSET)+(((x)%(MAX_BOOT_LOG_COUNT))*(BOOT_LOG_SIZE)))
/* ex.
 * boot_count = 0 start offset is NT_BOOT_KMSG_LOG_OFFSET + 0
 * boot_count = 1 start offset is NT_BOOT_KMSG_LOG_OFFSET + 512K
 * boot_count = 2 start offset is NT_BOOT_KMSG_LOG_OFFSET + 1024K
 * ....etc
 * //cycle run , boot_count = 32 equal boot_count = 0
 * boot_count = 32 start offset is NT_BOOT_KMSG_LOG_OFFSET + 0
 * boot_count = 33 start offset is NT_BOOT_KMSG_LOG_OFFSET + 512K
 * boot_count = 34 start offset is NT_BOOT_KMSG_LOG_OFFSET + 1024K
 */

#define PANIC_LOG_SIZE (256 * NT_K_MULT) //per panic log size, kmsg size 256K default in kernel
#define MAX_PANIC_LOG_COUNT 16 // The panic log can record 16 kmsg logs.
#define COMPUTE_PANIC_LOG_OFFSET(x) ((NT_PANIC_KMSG_LOG_OFFSET)+(((x)%(MAX_PANIC_LOG_COUNT))*(PANIC_LOG_SIZE)))
/* ex. panic_count = 0 start offset is NT_PANIC_KMSG_LOG_OFFSET + 0
 * panic_count = 1 start offset is NT_PANIC_KMSG_LOG_OFFSET + 256K
 * panic_count = 2 start offset is NT_PANIC_KMSG_LOG_OFFSET + 512K....etc
 */

#define NT_MAGIC_SIZE 16 // need follow 2^n

const char* LOG_THREAD_MAGIC = "NOTING_RKL_V0";
const char* TARGET_PARTITION_LABEL = "PARTLABEL=logdump";
const struct file_operations f_op = {.fsync = blkdev_fsync};

/* If someone needs to modify the NT_reserve_kernel_log_header struct.
 * you also need to change the LOG_THREAD_MAGIC!
 * ex. NOTING_IS_RKL_V0 -> NOTING_IS_RKL_V1
 * This means that the header needs to be "re-initialized"
 */
typedef struct
{
	char magic[NT_MAGIC_SIZE]; //NOTING_IS_RKL_V0
	unsigned char boot_count;
	unsigned char panic_count;
	unsigned char last_reboot_is_panic;
	unsigned char last_boot_is_fail;
} NT_reserve_kernel_log_header;

typedef enum {
	NT_COMPARE_HEADER_MAGIC,
	NT_RESET_HEADER,
	NT_ADD_BOOT_COUNT,
	NT_ADD_PANIC_COUNT,
	NT_SET_LAST_BOOT_IS_FAILED,
	NT_SET_LAST_REBOOT_IS_PANIC,
} NT_reserve_kernel_log_header_actions;


static struct task_struct *tsk;
NT_reserve_kernel_log_header rkl_header;

static struct device *nt_sys_dev;
static struct class *nt_sys_dev_class;
static dev_t nt_sys_device_devt;

static unsigned int is_boot_system_server = 0;

int __verify_target_partition_layout(){
	 //NT_CHECK_LAYOUT_END_OFFSET need equal logdump partition size = 512MB
	if(NT_CHECK_LAYOUT_END_OFFSET == NT_512M_MULT)
		return 1;
	NT_rkl_err_print("Default offset is %d ,true offset is %d\n", NT_CHECK_LAYOUT_END_OFFSET, NT_512M_MULT);
	return 0;
}
void __open_target_partition(struct block_device *bdev, struct file *target_partition_file) {
	target_partition_file->f_mapping = bdev->bd_inode->i_mapping;
	target_partition_file->f_flags = O_DSYNC | __O_SYNC | O_NOATIME;
	target_partition_file->f_inode = bdev->bd_inode;
	target_partition_file->f_lock = bdev->bd_inode->i_lock;
}

int __read_buf_from_target_partition(struct file *target_partition_file, void *head, int len, int pos) {
	struct kiocb kiocb;
	struct iov_iter iter;
	struct kvec iov;
	int read_size = 0;

	init_sync_kiocb(&kiocb, target_partition_file);
	kiocb.ki_pos = pos;
	iov.iov_base = head;
	iov.iov_len = len;
	iov_iter_kvec(&iter, READ, &iov, 1, len);

	read_size = generic_file_read_iter(&kiocb, &iter);

	if (read_size <= 0) {
		NT_rkl_err_print("Read buf failed, call by %pS\n", __builtin_return_address(0));
		return 1;
	}
	return 0;
}

int __write_buf_to_target_partition(struct file *target_partition_file, void *head, int len, int pos) {
	struct kiocb kiocb;
	struct iov_iter iter;
	struct kvec iov;
	int ret = 0;

	if (in_interrupt() || irqs_disabled())
		return -EBUSY;
	init_sync_kiocb(&kiocb, target_partition_file);
	kiocb.ki_pos = pos;
	iov.iov_base = head;
	iov.iov_len = len;
	iov_iter_kvec(&iter, WRITE, &iov, 1, len);

	ret = generic_write_checks(&kiocb, &iter);
	if (ret > 0)
		ret = generic_perform_write(target_partition_file, &iter, kiocb.ki_pos);
	if (ret > 0) {
		target_partition_file->f_op = &f_op;
		kiocb.ki_pos += ret;
		ret = generic_write_sync(&kiocb, ret);
		if (ret < 0) {
			NT_rkl_err_print("Write buf failed, call by %pS\n", __builtin_return_address(0));
			return 1;
		}
	}
	return 0;
}

struct block_device *get_target_partition_block_device()
{
	struct block_device *bdev = NULL;
	int retry_wait_for_device = RETRY_COUNT_FOR_GET_DEV_T;
	dev_t devt;


	while (retry_wait_for_device > 0){
		devt = name_to_dev_t(TARGET_PARTITION_LABEL);
		bdev = blkdev_get_by_dev(devt, FMODE_READ | FMODE_WRITE, NULL);
			if (!IS_ERR(bdev)) {
				NT_rkl_err_print("Get partition: %s\n", (char *)bdev->bd_part->info->volname, devt);
				return bdev;
			} else {
				NT_rkl_err_print("Get partition failed, retry count %d\n", retry_wait_for_device);
				msleep_interruptible(PER_LOOP_MS);
			}
		--retry_wait_for_device;
		}
	return NULL;
}

int action_reserve_kernel_log_header(struct file *target_partition_file, int action) {

	if(__read_buf_from_target_partition(target_partition_file, &rkl_header
		, sizeof(NT_reserve_kernel_log_header), NT_RESERVE_N_HEADER_OFFSET))
		goto out;

	switch(action) {
		case NT_COMPARE_HEADER_MAGIC:
			return memcmp(rkl_header.magic, LOG_THREAD_MAGIC, strlen(LOG_THREAD_MAGIC));
		break;
		case NT_RESET_HEADER:
			memset(&rkl_header, 0, sizeof(NT_reserve_kernel_log_header));
			memcpy(rkl_header.magic, LOG_THREAD_MAGIC, strlen(LOG_THREAD_MAGIC));
			NT_rkl_err_print("Reset header, boot_count is %d\n", rkl_header.boot_count);
		break;
		case NT_ADD_BOOT_COUNT:
			++rkl_header.boot_count;
			NT_rkl_err_print("boot_count is %d\n", rkl_header.boot_count);
		break;
		case NT_ADD_PANIC_COUNT:
			++rkl_header.panic_count;
			NT_rkl_err_print("panic_count is %d\n", rkl_header.panic_count);
		break;
		case NT_SET_LAST_BOOT_IS_FAILED:
			rkl_header.last_boot_is_fail = 1;
			NT_rkl_err_print("last_boot_is_fail %d, boot_count is %d\n"
				, rkl_header.last_boot_is_fail, rkl_header.boot_count);
		break;
		case NT_SET_LAST_REBOOT_IS_PANIC:
			rkl_header.last_reboot_is_panic = 1;
			NT_rkl_err_print("last_reboot_is_panic %d, panic_count is %d\n"
				, rkl_header.last_reboot_is_panic, rkl_header.panic_count);
		break;
		default:
			NT_rkl_err_print("The action is nothing~ nothing~\n");
			goto out;
		break;
	}

	if(__write_buf_to_target_partition(target_partition_file, &rkl_header
		, sizeof(NT_reserve_kernel_log_header), NT_RESERVE_N_HEADER_OFFSET))
		goto out;

	return 0;
	out:
		return 1;
}

int clear_kernel_boot_log(struct file *target_partition_file, int offset) {
	int i = 0;
	char* buf = NULL;

	buf = (char*)vzalloc(NT_PAGE_SIZE * sizeof(char));
	if(!buf) {
		NT_rkl_err_print("Failed to allocate boot log buffer!\n");
		goto out;
	}
	for(i = 0; i < BOOT_LOG_PAGES; i++) {
		if( __write_buf_to_target_partition(target_partition_file, buf, NT_PAGE_SIZE, offset+(i * BOOT_LOG_PAGES))){
			goto out;
		}
	}

out:
	if(buf) {
		vfree(buf);
		buf = NULL;
	}
	return 0;
}

void write_back_kernel_boot_log(struct file *target_partition_file) {
	char* buf = NULL;
	char *line_buf = NULL;
	char prefix_buf[PREFIX_BUFFER_SIZE] = {0};
	struct kmsg_dumper kmsg_dumper;
	unsigned int offset = 0;
	unsigned int ori_offset = 0;
	int delta_4K_offset = 0;
	int final_write = 0;
	size_t len = 0;
	size_t now_len = 0;

	NT_rkl_err_print("%s start!\n", __func__);

	if(!target_partition_file->f_inode) { //check target partition is init.
		NT_rkl_err_print("The block device isn't init!\n");
		goto out;
	}
	ori_offset = offset = COMPUTE_BOOT_LOG_OFFSET(rkl_header.boot_count);

	if(clear_kernel_boot_log(target_partition_file, offset)) {
		goto out;
	}
	buf = (char*)vzalloc(NT_PAGE_SIZE * sizeof(char));
	if(!buf) {
		NT_rkl_err_print("Failed to allocate boot log buffer!\n");
		goto out;
	}
	line_buf = (char*)vzalloc(BOOT_LOG_SIZE_OF_LINE * sizeof(char));
	if(!line_buf) {
		NT_rkl_err_print("Failed to allocate boot log line buffer!\n");
		goto out;
	}

	if(snprintf(prefix_buf, PREFIX_BUFFER_SIZE, "\nNT_boot_count:%d\n", rkl_header.boot_count) <= 0) {
		NT_rkl_err_print("Store prefix failed\n");
	} else {
		strncpy(buf, prefix_buf, PREFIX_BUFFER_SIZE);
	}
	now_len = strlen(buf);
	kmsg_dumper.active = true;

	RE_LOOP:
	while (kmsg_dump_get_line(&kmsg_dumper, true, line_buf, BOOT_LOG_SIZE_OF_LINE, &len)){
		delta_4K_offset = now_len + len - (NT_PAGE_SIZE );
		if (delta_4K_offset > 0) {
			memcpy(buf + now_len, line_buf, len - delta_4K_offset);
			__write_buf_to_target_partition(target_partition_file, buf, NT_PAGE_SIZE, offset);
			offset += NT_PAGE_SIZE;
			if ((offset - ori_offset) >= BOOT_LOG_SIZE) {
				NT_rkl_err_print("boot log full %d %d\n", offset, COMPUTE_BOOT_LOG_OFFSET(rkl_header.boot_count));
				goto out;
			}
			now_len = 0;
			memset(buf, 0, NT_PAGE_SIZE);
			memcpy(buf + now_len, line_buf + (len - delta_4K_offset), delta_4K_offset);
			now_len += delta_4K_offset;
		} else {
			memcpy(buf + now_len, line_buf, len);
			now_len += len;
		}
	}
	__write_buf_to_target_partition(target_partition_file, buf, NT_PAGE_SIZE, offset);
	msleep_interruptible(PER_LOOP_MS);
	if(!is_boot_system_server) {
		goto RE_LOOP;
	} else if(!final_write) {
		NT_rkl_err_print("Boot into system_server,Stop record log!\n");
		final_write = 1;
		goto RE_LOOP;
	}

out:
	if(buf) {
		vfree(buf);
		buf = NULL;
	}
	NT_rkl_err_print("%s end!\n", __func__);
}

int init_reserve_kernel_log_header(struct file *target_partition_file) {

	if (!action_reserve_kernel_log_header(target_partition_file, NT_COMPARE_HEADER_MAGIC)) {
		NT_rkl_err_print("Match magic %s\n", rkl_header.magic);
		return action_reserve_kernel_log_header(target_partition_file, NT_ADD_BOOT_COUNT);
	} else {
		NT_rkl_err_print("No match magic need init\n");
		 return action_reserve_kernel_log_header(target_partition_file, NT_RESET_HEADER);
	}

	return 1;
}

static ssize_t boot_stage_systemserver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, 10, "%d\n", is_boot_system_server);
}

static ssize_t boot_stage_systemserver_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned int value;
	int rc;

	if (size > 10)
		return -EINVAL;

	rc = kstrtou32(buf, 10, &value);
	if (rc) {
		NT_rkl_err_print("%s input wrang value!\n", __func__);
		return rc;
	}
	is_boot_system_server = value;

	return size;
}

static DEVICE_ATTR(boot_stage_systemserver, 0664, boot_stage_systemserver_show, boot_stage_systemserver_store);

int create_NT_device() {

	nt_sys_device_devt = MKDEV(NT_MAJOR, NT_MINOR);

	nt_sys_dev_class = class_create(THIS_MODULE,"NT");
	if (IS_ERR_OR_NULL(nt_sys_dev_class)) {
		NT_rkl_err_print("sysfs device class creation failed\n");
		goto out;
	}

	nt_sys_dev = device_create(nt_sys_dev_class, NULL,nt_sys_device_devt, "nt_sys_drvdata", "NT_reserve_kernel_log");
	if (IS_ERR_OR_NULL(nt_sys_dev)) {
		NT_rkl_err_print("sysfs device creation failed\n");
		goto out_class_destroy;
	}

	if (device_create_file(nt_sys_dev, &dev_attr_boot_stage_systemserver)) {
		NT_rkl_err_print("sysfs boot_stage_systemserver file creation failed\n");
		goto out_device_destroy;
	}
	return 0;

out_device_destroy:
	device_destroy(nt_sys_dev_class, nt_sys_device_devt);

out_class_destroy:
	class_destroy(nt_sys_dev_class);
out:
	return 1;
}

void remove_NT_device() {

	NT_rkl_err_print("%s start!\n", __func__);
	device_remove_file(nt_sys_dev, &dev_attr_boot_stage_systemserver);

	device_destroy(nt_sys_dev_class, nt_sys_device_devt);

	class_destroy(nt_sys_dev_class);
	NT_rkl_err_print("%s end!\n", __func__);
}

static int NT_reserve_kernel_log_main(void *arg)
{
	struct block_device *bdev = NULL;
	struct file target_partition_file;

	if(!__verify_target_partition_layout()){
		NT_rkl_err_print("Error setting, plz check your layout setting!\n");
		return 1;
	}

	if(create_NT_device()){
		NT_rkl_err_print("Failed to create NT device!\n");
		return 1;
	}


	bdev = get_target_partition_block_device();
	if(!bdev) {
		NT_rkl_err_print("Get target partition block device failed!\n");
		return 1;
	}

	memset(&target_partition_file, 0, sizeof(struct file));

	__open_target_partition(bdev, &target_partition_file);

	if(init_reserve_kernel_log_header(&target_partition_file)){
		NT_rkl_err_print("Init header failed!\n");
		return 1;
	}
	write_back_kernel_boot_log(&target_partition_file);

	remove_NT_device();

	return 0;
}

static int __init NT_reserve_kernel_log_init(void)
{
	tsk = kthread_run(NT_reserve_kernel_log_main, NULL, "NT_reserve_kernel_log");
	if (!tsk)
		NT_rkl_err_print("kthread init failed\n");

	NT_rkl_info_print("kthread init done\n");
	return 0;
}

module_init(NT_reserve_kernel_log_init);

static void __exit NT_reserve_kernel_log_exit(void)
{
	NT_rkl_info_print("Hello bye~bye!\n");
}
module_exit(NT_reserve_kernel_log_exit);

MODULE_LICENSE("GPL v2");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("<BSP_CORE@nothing.tech>");
MODULE_DESCRIPTION("NOTHING record boot KMSG");