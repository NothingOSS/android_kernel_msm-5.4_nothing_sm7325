/*
 * Universal Flash Storage Feature Support
 *
 * Copyright (C) 2017-2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Yongmyung Lee <ymhungry.lee@samsung.com>
 *	Jinyoung Choi <j-young.choi@samsung.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * See the COPYING file in the top-level directory or visit
 * <http://www.gnu.org/licenses/gpl-2.0.html>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This program is provided "AS IS" and "WITH ALL FAULTS" and
 * without warranty of any kind. You are solely responsible for
 * determining the appropriateness of using and distributing
 * the program and assume all risks associated with your exercise
 * of rights with respect to the program, including but not limited
 * to infringement of third party rights, the risks and costs of
 * program errors, damage to or loss of data, programs or equipment,
 * and unavailability or interruption of operations. Under no
 * circumstances will the contributor of this Program be liable for
 * any damages of any kind arising from your use or distribution of
 * this program.
 *
 * The Linux Foundation chooses to take subject only to the GPLv2
 * license terms, and distributes only under these terms.
 */

#ifndef _UFSFEATURE_H_
#define _UFSFEATURE_H_

#include <scsi/scsi_cmnd.h>

#include "ufs.h"

#include "ufshid.h"

/* Version info */
#define UFSFEATURE_DD_VER			0x010004
#define UFSFEATURE_DD_VER_POST			""

#define UFSF_QUERY_REQ_RETRIES			1

/* Description */
#define UFSF_QUERY_DESC_DEVICE_MAX_SIZE		0x5F

#define UFSFEATURE_SELECTOR			0x01

#define INFO_MSG(msg, args...)		pr_info("%s:%d info: " msg "\n", \
					       __func__, __LINE__, ##args)
#define ERR_MSG(msg, args...)		pr_err("%s:%d err: " msg "\n", \
					       __func__, __LINE__, ##args)
#define WARN_MSG(msg, args...)		pr_warn("%s:%d warn: " msg "\n", \
					       __func__, __LINE__, ##args)

#define seq_scan_lu(lun) for (lun = 0; lun < UFS_UPIU_MAX_GENERAL_LUN; lun++)

#define TMSG(ufsf, lun, msg, args...)					\
	do { if (ufsf->sdev_ufs_lu[lun] &&				\
		 ufsf->sdev_ufs_lu[lun]->request_queue)			\
		blk_add_trace_msg(					\
			ufsf->sdev_ufs_lu[lun]->request_queue,		\
			msg, ##args);					\
	} while (0)							\

struct ufsf_feature {
	struct ufs_hba *hba;
	int num_lu;
	int slave_conf_cnt;
	struct scsi_device *sdev_ufs_lu[UFS_UPIU_MAX_GENERAL_LUN];

#if defined(CONFIG_UFSHID)
	atomic_t hid_state;
	struct ufshid_dev *hid_dev;
#endif
};

struct ufs_hba;
struct ufshcd_lrb;
struct ufs_ioctl_query_data;

void ufsf_device_check(struct ufs_hba *hba);
int ufsf_query_attr_retry(struct ufs_hba *hba, enum query_opcode opcode,
			  enum attr_idn idn, u8 idx, u32 *attr_val);
void ufsf_reset_lu(struct ufsf_feature *ufsf);
void ufsf_reset_host(struct ufsf_feature *ufsf);
void ufsf_init(struct ufsf_feature *ufsf);
void ufsf_reset(struct ufsf_feature *ufsf);
void ufsf_remove(struct ufsf_feature *ufsf);
void ufsf_set_init_state(struct ufsf_feature *ufsf);
void ufsf_suspend(struct ufsf_feature *ufsf);
void ufsf_resume(struct ufsf_feature *ufsf);
void ufsf_on_idle(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp);

/* for hid */
void ufsf_hid_acc_io_stat(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp);
#endif /* End of Header */
