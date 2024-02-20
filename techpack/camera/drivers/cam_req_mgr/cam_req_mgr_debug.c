// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 */

#include "cam_req_mgr_debug.h"

#define MAX_SESS_INFO_LINE_BUFF_LEN 256

static char sess_info_buffer[MAX_SESS_INFO_LINE_BUFF_LEN];
static int cam_debug_mgr_delay_detect;

static int cam_req_mgr_debug_set_bubble_recovery(void *data, u64 val)
{
	struct cam_req_mgr_core_device  *core_dev = data;
	struct cam_req_mgr_core_session *session;
	int rc = 0;

	mutex_lock(&core_dev->crm_lock);

	if (!list_empty(&core_dev->session_head)) {
		list_for_each_entry(session,
			&core_dev->session_head, entry) {
			session->force_err_recovery = val;
		}
	}

	mutex_unlock(&core_dev->crm_lock);

	return rc;
}

static int cam_req_mgr_debug_get_bubble_recovery(void *data, u64 *val)
{
	struct cam_req_mgr_core_device *core_dev = data;
	struct cam_req_mgr_core_session *session;

	mutex_lock(&core_dev->crm_lock);

	if (!list_empty(&core_dev->session_head)) {
		session = list_first_entry(&core_dev->session_head,
			struct cam_req_mgr_core_session,
			entry);
		*val = session->force_err_recovery;
	}
	mutex_unlock(&core_dev->crm_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bubble_recovery, cam_req_mgr_debug_get_bubble_recovery,
	cam_req_mgr_debug_set_bubble_recovery, "%lld\n");

static int session_info_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t session_info_read(struct file *t_file, char *t_char,
	size_t t_size_t, loff_t *t_loff_t)
{
	int i;
	char *out_buffer = sess_info_buffer;
	char line_buffer[MAX_SESS_INFO_LINE_BUFF_LEN] = {0};
	struct cam_req_mgr_core_device *core_dev =
		(struct cam_req_mgr_core_device *) t_file->private_data;
	struct cam_req_mgr_core_session *session;

	memset(out_buffer, 0, MAX_SESS_INFO_LINE_BUFF_LEN);

	mutex_lock(&core_dev->crm_lock);

	if (!list_empty(&core_dev->session_head)) {
		list_for_each_entry(session,
			&core_dev->session_head, entry) {
			snprintf(line_buffer, sizeof(line_buffer),
				"session_hdl = %x \t"
				"num_links = %d\n",
				session->session_hdl, session->num_links);
			strlcat(out_buffer, line_buffer,
				sizeof(sess_info_buffer));
			for (i = 0; i < session->num_links; i++) {
				snprintf(line_buffer, sizeof(line_buffer),
					"link_hdl[%d] = 0x%x, num_devs connected = %d\n",
					i, session->links[i]->link_hdl,
					session->links[i]->num_devs);
				strlcat(out_buffer, line_buffer,
					sizeof(sess_info_buffer));
			}
		}
	}

	mutex_unlock(&core_dev->crm_lock);

	return simple_read_from_buffer(t_char, t_size_t,
		t_loff_t, out_buffer, strlen(out_buffer));
}

static ssize_t session_info_write(struct file *t_file,
	const char *t_char, size_t t_size_t, loff_t *t_loff_t)
{
	memset(sess_info_buffer, 0, MAX_SESS_INFO_LINE_BUFF_LEN);

	return 0;
}

static const struct file_operations session_info = {
	.open = session_info_open,
	.read = session_info_read,
	.write = session_info_write,
};

static struct dentry *debugfs_root;
int cam_req_mgr_debug_register(struct cam_req_mgr_core_device *core_dev)
{
	int rc = 0;
	struct dentry *dbgfileptr = NULL;

	dbgfileptr = debugfs_create_dir("cam_req_mgr", NULL);
	if (!dbgfileptr) {
		CAM_ERR(CAM_MEM,"DebugFS could not create directory!");
		rc = -ENOENT;
		goto end;
	}
	/* Store parent inode for cleanup in caller */
	debugfs_root = dbgfileptr;

	dbgfileptr = debugfs_create_file("sessions_info", 0644, debugfs_root,
		core_dev, &session_info);
	dbgfileptr = debugfs_create_file("bubble_recovery", 0644,
		debugfs_root, core_dev, &bubble_recovery);
	dbgfileptr = debugfs_create_bool("recovery_on_apply_fail", 0644,
		debugfs_root, &core_dev->recovery_on_apply_fail);
	dbgfileptr = debugfs_create_u32("delay_detect_count", 0644,
		debugfs_root, &cam_debug_mgr_delay_detect);
	if (IS_ERR(dbgfileptr)) {
		if (PTR_ERR(dbgfileptr) == -ENODEV)
			CAM_WARN(CAM_MEM, "DebugFS not enabled in kernel!");
		else
			rc = PTR_ERR(dbgfileptr);
	}
end:
	return rc;
}

int cam_req_mgr_debug_unregister(void)
{
	debugfs_remove_recursive(debugfs_root);
	return 0;
}

void cam_req_mgr_debug_delay_detect(void)
{
	cam_debug_mgr_delay_detect += 1;
}
