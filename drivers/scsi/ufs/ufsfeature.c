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

#include "ufsfeature.h"
#include "ufshcd.h"

static inline bool ufsf_is_valid_lun(int lun)
{
	return lun < UFS_UPIU_MAX_GENERAL_LUN;
}

inline void ufsf_slave_configure(struct ufsf_feature *ufsf,
				 struct scsi_device *sdev)
{
	if (ufsf_is_valid_lun(sdev->lun)) {
		ufsf->sdev_ufs_lu[sdev->lun] = sdev;
		ufsf->slave_conf_cnt++;
		INFO_MSG("lun[%d] sdev(%p) q(%p) slave_conf_cnt(%d/%d)",
			 (int)sdev->lun, sdev, sdev->request_queue,
			 ufsf->slave_conf_cnt, ufsf->num_lu);
	}
}
int ufsf_query_attr_retry(struct ufs_hba *hba, enum query_opcode opcode,
			  enum attr_idn idn, u8 idx, u32 *attr_val)
{
	int ret;
	int retries;

	for (retries = 0; retries < UFSF_QUERY_REQ_RETRIES; retries++) {
		ret = ufshcd_query_attr(hba, opcode, idn, idx,
					UFSFEATURE_SELECTOR, attr_val);
		if (ret)
			dev_dbg(hba->dev,
				"%s: failed with error %d, retries %d\n",
				__func__, ret, retries);
		else
			break;
	}
	if (ret) {
		dev_err(hba->dev,
			"%s: query attr, opcode %d, idn %d, failed with error %d after %d retires\n",
			__func__, opcode, idn, ret, retries);
		dev_err(hba->dev, "%s: UFS state (POWER = %d LINK = %d)",
			__func__, hba->curr_dev_pwr_mode, hba->uic_link_state);
	}
	return ret;
}

static int ufsf_read_desc(struct ufs_hba *hba, u8 desc_id, u8 desc_index,
			  u8 selector, u8 *desc_buf, u32 size)
{
	int err = 0;

	pm_runtime_get_sync(hba->dev);

	err = ufshcd_query_descriptor_retry(hba, UPIU_QUERY_OPCODE_READ_DESC,
					    desc_id, desc_index,
					    selector,
					    desc_buf, &size);
	if (err)
		ERR_MSG("reading Device Desc failed. err = %d", err);

	pm_runtime_put_sync(hba->dev);

	return err;
}

static int ufsf_read_dev_desc(struct ufsf_feature *ufsf, u8 selector)
{
	u8 desc_buf[UFSF_QUERY_DESC_DEVICE_MAX_SIZE];
	int ret;

	ret = ufsf_read_desc(ufsf->hba, QUERY_DESC_IDN_DEVICE, 0, selector,
			     desc_buf, UFSF_QUERY_DESC_DEVICE_MAX_SIZE);
	if (ret)
		return ret;

	ufsf->num_lu = desc_buf[DEVICE_DESC_PARAM_NUM_LU];
	INFO_MSG("device lu count %d", ufsf->num_lu);

	INFO_MSG("sel=%u length=%u(0x%x) bSupport=0x%.2x, extend=0x%.2x_%.2x",
		  selector, desc_buf[DEVICE_DESC_PARAM_LEN],
		  desc_buf[DEVICE_DESC_PARAM_LEN],
		  desc_buf[DEVICE_DESC_PARAM_UFS_FEAT],
		  desc_buf[DEVICE_DESC_PARAM_EX_FEAT_SUP+2],
		  desc_buf[DEVICE_DESC_PARAM_EX_FEAT_SUP+3]);

	INFO_MSG("Driver Feature Version : (%.6X%s)", UFSFEATURE_DD_VER,
		 UFSFEATURE_DD_VER_POST);
#if defined(CONFIG_UFSHID)
	ufshid_get_dev_info(ufsf, desc_buf);
#endif
	return 0;
}

void ufsf_device_check(struct ufs_hba *hba)
{
	struct ufsf_feature *ufsf = &hba->ufsf;
	int ret;
	u32 status;

	ufsf->hba = hba;

	ufshcd_query_attr(ufsf->hba, UPIU_QUERY_OPCODE_READ_ATTR,
			  QUERY_ATTR_IDN_SUP_VENDOR_OPTIONS, 0, 0, &status);
	INFO_MSG("UFS FEATURE SELECTOR Dev %d - D/D %d", status,
		  UFSFEATURE_SELECTOR);

	ret = ufsf_read_dev_desc(ufsf, UFSFEATURE_SELECTOR);
	if (ret)
		return;
}

inline void ufsf_reset_host(struct ufsf_feature *ufsf)
{
#if defined(CONFIG_UFSHID)
	INFO_MSG("run reset_host.. hid_state(%d) -> HID_RESET",
		 ufshid_get_state(ufsf));
	if (ufshid_get_state(ufsf) == HID_PRESENT)
		ufshid_reset_host(ufsf);
#endif
}

inline void ufsf_init(struct ufsf_feature *ufsf)
{
#if defined(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_NEED_INIT)
		ufshid_init(ufsf);
#endif
}

inline void ufsf_reset(struct ufsf_feature *ufsf)
{
#if defined(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_RESET)
		ufshid_reset(ufsf);
#endif
}

inline void ufsf_remove(struct ufsf_feature *ufsf)
{
#if defined(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_PRESENT)
		ufshid_remove(ufsf);
#endif
}

inline void ufsf_set_init_state(struct ufsf_feature *ufsf)
{
#if defined(CONFIG_UFSHID)
	ufshid_set_state(ufsf, HID_NEED_INIT);
#endif
}

inline void ufsf_suspend(struct ufsf_feature *ufsf)
{
#if defined(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_PRESENT)
		ufshid_suspend(ufsf);
#endif
}

inline void ufsf_resume(struct ufsf_feature *ufsf)
{
#if defined(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_SUSPEND)
		ufshid_resume(ufsf);
#endif
}

inline void ufsf_on_idle(struct ufsf_feature *ufsf, struct ufshcd_lrb *lrbp)
{
	struct ufs_hba *hba = ufsf->hba;

	if (hba && (hba->outstanding_tasks ||
	    !(hba->outstanding_reqs == (1 << lrbp->task_tag))))
		return;

#if defined(CONFIG_UFSHID)
	if (ufshid_get_state(ufsf) == HID_PRESENT)
		ufshid_on_idle(ufsf);
#endif
}
