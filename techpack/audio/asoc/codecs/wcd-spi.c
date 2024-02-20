// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/component.h>
#include <linux/ratelimit.h>
#include <linux/platform_device.h>
#include <sound/wcd-dsp-mgr.h>
#include <sound/wcd-spi.h>
#include <soc/wcd-spi-ac.h>
#include "wcd-spi-registers.h"

/* Byte manipulations */
#define SHIFT_1_BYTES    (8)
#define SHIFT_2_BYTES    (16)
#define SHIFT_3_BYTES    (24)

/* Command opcodes */
#define WCD_SPI_CMD_NOP     (0x00)
#define WCD_SPI_CMD_WREN    (0x06)
#define WCD_SPI_CMD_CLKREQ  (0xDA)
#define WCD_SPI_CMD_RDSR    (0x05)
#define WCD_SPI_CMD_IRR     (0x81)
#define WCD_SPI_CMD_IRW     (0x82)
#define WCD_SPI_CMD_MIOR    (0x83)
#define WCD_SPI_CMD_FREAD   (0x0B)
#define WCD_SPI_CMD_MIOW    (0x02)
#define WCD_SPI_WRITE_FRAME_OPCODE \
	(WCD_SPI_CMD_MIOW << SHIFT_3_BYTES)
#define WCD_SPI_READ_FRAME_OPCODE \
	(WCD_SPI_CMD_MIOR << SHIFT_3_BYTES)
#define WCD_SPI_FREAD_FRAME_OPCODE \
	(WCD_SPI_CMD_FREAD << SHIFT_3_BYTES)

/* Command lengths */
#define WCD_SPI_OPCODE_LEN       (0x01)
#define WCD_SPI_CMD_NOP_LEN      (0x01)
#define WCD_SPI_CMD_WREN_LEN     (0x01)
#define WCD_SPI_CMD_CLKREQ_LEN   (0x04)
#define WCD_SPI_CMD_IRR_LEN      (0x04)
#define WCD_SPI_CMD_IRW_LEN      (0x06)
#define WCD_SPI_WRITE_SINGLE_LEN (0x08)
#define WCD_SPI_READ_SINGLE_LEN  (0x13)
#define WCD_SPI_CMD_FREAD_LEN    (0x13)

/* Command delays */
#define WCD_SPI_CLKREQ_DELAY_USECS (500)
#define WCD_SPI_CLK_OFF_TIMER_MS   (500)
#define WCD_SPI_RESUME_TIMEOUT_MS 100

/* Command masks */
#define WCD_CMD_ADDR_MASK            \
	(0xFF |                      \
	 (0xFF << SHIFT_1_BYTES) |   \
	 (0xFF << SHIFT_2_BYTES))

/* Clock ctrl request related */
#define WCD_SPI_CLK_ENABLE true
#define WCD_SPI_CLK_DISABLE false
#define WCD_SPI_CLK_FLAG_DELAYED    (1 << 0)
#define WCD_SPI_CLK_FLAG_IMMEDIATE  (1 << 1)

/* Internal addresses */
#define WCD_SPI_ADDR_IPC_CTL_HOST (0x012014)

/* Word sizes and min/max lengths */
#define WCD_SPI_WORD_BYTE_CNT (4)
#define WCD_SPI_RW_MULTI_MIN_LEN (16)

/* Max size is 32 bytes less than 64Kbytes */
#define WCD_SPI_RW_MULTI_MAX_LEN ((64 * 1024) - 32)

/*
 * Max size for the pre-allocated buffers is the max
 * possible read/write length + 32 bytes for the SPI
 * read/write command header itself.
 */
#define WCD_SPI_RW_MAX_BUF_SIZE (WCD_SPI_RW_MULTI_MAX_LEN + 32)

/* Alignment requirements */
#define WCD_SPI_RW_MIN_ALIGN    WCD_SPI_WORD_BYTE_CNT
#define WCD_SPI_RW_MULTI_ALIGN  (16)

/* Status mask bits */
#define WCD_SPI_CLK_STATE_ENABLED BIT(0)
#define WCD_SPI_IS_SUSPENDED BIT(1)

/* Locking related */
#define WCD_SPI_MUTEX_LOCK(spi, lock)              \
{                                                  \
	dev_vdbg(&spi->dev, "%s: mutex_lock(%s)\n", \
		 __func__, __stringify_1(lock));    \
	mutex_lock(&lock);                         \
}

#define WCD_SPI_MUTEX_UNLOCK(spi, lock)              \
{                                                    \
	dev_vdbg(&spi->dev, "%s: mutex_unlock(%s)\n", \
		 __func__, __stringify_1(lock));      \
	mutex_unlock(&lock);                         \
}

struct wcd_spi_debug_data {
	struct dentry *dir;
	u32 addr;
	u32 size;
};

struct wcd_spi_priv {
	struct spi_device *spi;
	u32 mem_base_addr;

	struct regmap *regmap;

	/* Message for single transfer */
	struct spi_message msg1;
	struct spi_transfer xfer1;

	/* Message for two transfers */
	struct spi_message msg2;
	struct spi_transfer xfer2[2];

	/* Register access related */
	u32 reg_bytes;
	u32 val_bytes;

	/* Clock requests related */
	struct mutex clk_mutex;
	int clk_users;
	unsigned long status_mask;
	struct delayed_work clk_dwork;

	/* Transaction related */
	struct mutex xfer_mutex;

	struct device *m_dev;
	struct wdsp_mgr_ops *m_ops;

	/* Debugfs related information */
	struct wcd_spi_debug_data debug_data;

	/* Completion object to indicate system resume completion */
	struct completion resume_comp;

	/* Buffers to hold memory used for transfers */
	void *tx_buf;
	void *rx_buf;

	/* DMA handles for transfer buffers */
	dma_addr_t tx_dma;
	dma_addr_t rx_dma;
	/* Handle to child (qmi client) device */
	struct device *ac_dev;
};

enum xfer_request {
	WCD_SPI_XFER_WRITE,
	WCD_SPI_XFER_READ,
};


static char *wcd_spi_xfer_req_str(enum xfer_request req)
{
	if (req == WCD_SPI_XFER_WRITE)
		return "xfer_write";
	else if (req == WCD_SPI_XFER_READ)
		return "xfer_read";
	else
		return "xfer_invalid";
}

static void wcd_spi_reinit_xfer(struct spi_transfer *xfer)
{
	xfer->tx_buf = NULL;
	xfer->rx_buf = NULL;
	xfer->delay_usecs = 0;
	xfer->len = 0;
}

static bool wcd_spi_is_suspended(struct wcd_spi_priv *wcd_spi)
{
	return test_bit(WCD_SPI_IS_SUSPENDED, &wcd_spi->status_mask);
}

static bool wcd_spi_can_suspend(struct wcd_spi_priv *wcd_spi)
{
	struct spi_device *spi = wcd_spi->spi;

	if (wcd_spi->clk_users > 0 ||
	    test_bit(WCD_SPI_CLK_STATE_ENABLED, &wcd_spi->status_mask)) {
		dev_err(&spi->dev, "%s: cannot suspend, clk_users = %d\n",
			__func__, wcd_spi->clk_users);
		return false;
	}

	return true;
}

static int wcd_spi_wait_for_resume(struct wcd_spi_priv *wcd_spi)
{
	struct spi_device *spi = wcd_spi->spi;
	int rc = 0;

	WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);
	/* If the system is already in resumed state, return right away */
	if (!wcd_spi_is_suspended(wcd_spi))
		goto done;

	/* If suspended then wait for resume to happen */
	reinit_completion(&wcd_spi->resume_comp);
	WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);
	rc = wait_for_completion_timeout(&wcd_spi->resume_comp,
				msecs_to_jiffies(WCD_SPI_RESUME_TIMEOUT_MS));
	WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);
	if (rc == 0) {
		dev_err(&spi->dev, "%s: failed to resume in %u msec\n",
			__func__, WCD_SPI_RESUME_TIMEOUT_MS);
		rc = -EIO;
		goto done;
	}

	dev_dbg(&spi->dev, "%s: resume successful\n", __func__);
	rc = 0;
done:
	WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);
	return rc;
}

static int wcd_spi_read_single(struct spi_device *spi,
			       u32 remote_addr, u32 *val)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct spi_transfer *tx_xfer = &wcd_spi->xfer2[0];
	struct spi_transfer *rx_xfer = &wcd_spi->xfer2[1];
	u8 *tx_buf = wcd_spi->tx_buf;
	u8 *rx_buf = wcd_spi->rx_buf;
	u32 frame = 0;
	int ret;

	dev_dbg(&spi->dev, "%s: remote_addr = 0x%x\n",
		__func__, remote_addr);

	if (!tx_buf) {
		dev_err(&spi->dev, "%s: tx_buf not allocated\n",
			__func__);
		return -ENOMEM;
	}

	frame |= WCD_SPI_READ_FRAME_OPCODE;
	frame |= remote_addr & WCD_CMD_ADDR_MASK;

	wcd_spi_reinit_xfer(tx_xfer);
	frame = cpu_to_be32(frame);
	memcpy(tx_buf, &frame, sizeof(frame));
	tx_xfer->tx_buf = tx_buf;
	tx_xfer->len = WCD_SPI_READ_SINGLE_LEN;

	wcd_spi_reinit_xfer(rx_xfer);
	rx_xfer->rx_buf = rx_buf;
	rx_xfer->len = sizeof(*val);

	ret = spi_sync(spi, &wcd_spi->msg2);
	if (ret)
		dev_err(&spi->dev, "%s: spi_sync failed, err %d\n",
			__func__, ret);
	else
		memcpy((u8*) val, rx_buf, sizeof(*val));

	return ret;
}

static int wcd_spi_read_multi(struct spi_device *spi,
			      u32 remote_addr, u8 *data,
			      size_t len)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct spi_transfer *xfer = &wcd_spi->xfer1;
	u8 *tx_buf = wcd_spi->tx_buf;
	u8 *rx_buf = wcd_spi->rx_buf;
	u32 frame = 0;
	int ret;

	dev_dbg(&spi->dev,  "%s: addr 0x%x, len = %zd\n",
		__func__, remote_addr, len);

	frame |= WCD_SPI_FREAD_FRAME_OPCODE;
	frame |= remote_addr & WCD_CMD_ADDR_MASK;

	if (!tx_buf || !rx_buf) {
		dev_err(&spi->dev, "%s: %s not allocated\n", __func__,
			(!tx_buf) ? "tx_buf" : "rx_buf");
		return -ENOMEM;
	}

	wcd_spi_reinit_xfer(xfer);
	frame = cpu_to_be32(frame);
	memcpy(tx_buf, &frame, sizeof(frame));
	xfer->tx_buf = tx_buf;
	xfer->rx_buf = rx_buf;
	xfer->len = WCD_SPI_CMD_FREAD_LEN + len;

	ret = spi_sync(spi, &wcd_spi->msg1);
	if (ret) {
		dev_err(&spi->dev, "%s: failed, err = %d\n",
			__func__, ret);
		goto done;
	}

	memcpy(data, rx_buf + WCD_SPI_CMD_FREAD_LEN, len);
done:
	return ret;
}

static int wcd_spi_write_single(struct spi_device *spi,
				u32 remote_addr, u32 val)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct spi_transfer *xfer = &wcd_spi->xfer1;
	u8 *tx_buf = wcd_spi->tx_buf;
	u32 frame = 0;

	dev_dbg(&spi->dev, "%s: remote_addr = 0x%x, val = 0x%x\n",
		__func__, remote_addr, val);

	memset(tx_buf, 0, WCD_SPI_WRITE_SINGLE_LEN);
	frame |= WCD_SPI_WRITE_FRAME_OPCODE;
	frame |= (remote_addr & WCD_CMD_ADDR_MASK);

	frame = cpu_to_be32(frame);
	memcpy(tx_buf, &frame, sizeof(frame));
	memcpy(tx_buf + sizeof(frame), &val, sizeof(val));

	wcd_spi_reinit_xfer(xfer);
	xfer->tx_buf = tx_buf;
	xfer->len = WCD_SPI_WRITE_SINGLE_LEN;

	return spi_sync(spi, &wcd_spi->msg1);
}

static int wcd_spi_write_multi(struct spi_device *spi,
			       u32 remote_addr, u8 *data,
			       size_t len)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct spi_transfer *xfer = &wcd_spi->xfer1;
	u32 frame = 0;
	u8 *tx_buf = wcd_spi->tx_buf;
	int xfer_len, ret;

	dev_dbg(&spi->dev, "%s: addr = 0x%x len = %zd\n",
		__func__, remote_addr, len);

	frame |= WCD_SPI_WRITE_FRAME_OPCODE;
	frame |= (remote_addr & WCD_CMD_ADDR_MASK);

	frame = cpu_to_be32(frame);
	xfer_len = len + sizeof(frame);

	if (!tx_buf) {
		dev_err(&spi->dev, "%s: tx_buf not allocated\n",
			__func__);
		return -ENOMEM;
	}

	memcpy(tx_buf, &frame, sizeof(frame));
	memcpy(tx_buf + sizeof(frame), data, len);

	wcd_spi_reinit_xfer(xfer);
	xfer->tx_buf = tx_buf;
	xfer->len = xfer_len;

	ret = spi_sync(spi, &wcd_spi->msg1);
	if (ret < 0)
		dev_err(&spi->dev,
			"%s: Failed, addr = 0x%x, len = %zd\n",
			__func__, remote_addr, len);
	return ret;
}

static int wcd_spi_transfer_split(struct spi_device *spi,
				  struct wcd_spi_msg *data_msg,
				  enum xfer_request xfer_req)
{
	u32 addr = data_msg->remote_addr;
	u8 *data = data_msg->data;
	int remain_size = data_msg->len;
	int to_xfer, loop_cnt, ret = 0;

	/* Perform single writes until multi word alignment is met */
	loop_cnt = 1;
	while (remain_size &&
	       !IS_ALIGNED(addr, WCD_SPI_RW_MULTI_ALIGN)) {
		if (xfer_req == WCD_SPI_XFER_WRITE)
			ret = wcd_spi_write_single(spi, addr,
						   (*(u32 *)data));
		else
			ret = wcd_spi_read_single(spi, addr,
						  (u32 *)data);
		if (ret < 0) {
			dev_err(&spi->dev,
				"%s: %s fail iter(%d) start-word addr (0x%x)\n",
				__func__, wcd_spi_xfer_req_str(xfer_req),
				loop_cnt, addr);
			goto done;
		}

		addr += WCD_SPI_WORD_BYTE_CNT;
		data += WCD_SPI_WORD_BYTE_CNT;
		remain_size -= WCD_SPI_WORD_BYTE_CNT;
		loop_cnt++;
	}

	/* Perform multi writes for max allowed multi writes */
	loop_cnt = 1;
	while (remain_size >= WCD_SPI_RW_MULTI_MAX_LEN) {
		if (xfer_req == WCD_SPI_XFER_WRITE)
			ret = wcd_spi_write_multi(spi, addr, data,
						  WCD_SPI_RW_MULTI_MAX_LEN);
		else
			ret = wcd_spi_read_multi(spi, addr, data,
						 WCD_SPI_RW_MULTI_MAX_LEN);
		if (ret < 0) {
			dev_err(&spi->dev,
				"%s: %s fail iter(%d) max-write addr (0x%x)\n",
				__func__, wcd_spi_xfer_req_str(xfer_req),
				loop_cnt, addr);
			goto done;
		}

		addr += WCD_SPI_RW_MULTI_MAX_LEN;
		data += WCD_SPI_RW_MULTI_MAX_LEN;
		remain_size -= WCD_SPI_RW_MULTI_MAX_LEN;
		loop_cnt++;
	}

	/*
	 * Perform write for max possible data that is multiple
	 * of the minimum size for multi-write commands.
	 */
	to_xfer = remain_size - (remain_size % WCD_SPI_RW_MULTI_MIN_LEN);
	if (remain_size >= WCD_SPI_RW_MULTI_MIN_LEN &&
	    to_xfer > 0) {
		if (xfer_req == WCD_SPI_XFER_WRITE)
			ret = wcd_spi_write_multi(spi, addr, data, to_xfer);
		else
			ret = wcd_spi_read_multi(spi, addr, data, to_xfer);
		if (ret < 0) {
			dev_err(&spi->dev,
				"%s: %s fail write addr (0x%x), size (0x%x)\n",
				__func__, wcd_spi_xfer_req_str(xfer_req),
				addr, to_xfer);
			goto done;
		}

		addr += to_xfer;
		data += to_xfer;
		remain_size -= to_xfer;
	}

	/* Perform single writes for the last remaining data */
	loop_cnt = 1;
	while (remain_size > 0) {
		if (xfer_req == WCD_SPI_XFER_WRITE)
			ret = wcd_spi_write_single(spi, addr, (*((u32 *)data)));
		else
			ret = wcd_spi_read_single(spi, addr,  (u32 *) data);
		if (ret < 0) {
			dev_err(&spi->dev,
				"%s: %s fail iter(%d) end-write addr (0x%x)\n",
				__func__, wcd_spi_xfer_req_str(xfer_req),
				loop_cnt, addr);
			goto done;
		}

		addr += WCD_SPI_WORD_BYTE_CNT;
		data += WCD_SPI_WORD_BYTE_CNT;
		remain_size -= WCD_SPI_WORD_BYTE_CNT;
		loop_cnt++;
	}

done:
	return ret;
}

static int wcd_spi_cmd_nop(struct spi_device *spi)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	u8 *tx_buf = wcd_spi->tx_buf;

	tx_buf[0] = WCD_SPI_CMD_NOP;

	return spi_write(spi, tx_buf, WCD_SPI_CMD_NOP_LEN);
}

static int wcd_spi_cmd_clkreq(struct spi_device *spi)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct spi_transfer *xfer = &wcd_spi->xfer1;
	u8 *tx_buf = wcd_spi->tx_buf;
	u8 cmd[WCD_SPI_CMD_CLKREQ_LEN] = {
		WCD_SPI_CMD_CLKREQ,
		0xBA, 0x80, 0x00};

	memcpy(tx_buf, cmd, WCD_SPI_CMD_CLKREQ_LEN);
	wcd_spi_reinit_xfer(xfer);
	xfer->tx_buf = tx_buf;
	xfer->len = WCD_SPI_CMD_CLKREQ_LEN;
	xfer->delay_usecs = WCD_SPI_CLKREQ_DELAY_USECS;

	return spi_sync(spi, &wcd_spi->msg1);
}

static int wcd_spi_cmd_wr_en(struct spi_device *spi)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	u8 *tx_buf = wcd_spi->tx_buf;

	tx_buf[0] = WCD_SPI_CMD_WREN;

	return spi_write(spi, tx_buf, WCD_SPI_CMD_WREN_LEN);
}

static int wcd_spi_cmd_rdsr(struct spi_device *spi,
			    u32 *rdsr_status)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct spi_transfer *tx_xfer = &wcd_spi->xfer2[0];
	struct spi_transfer *rx_xfer = &wcd_spi->xfer2[1];
	u8 *tx_buf = wcd_spi->tx_buf;
	u8 *rx_buf = wcd_spi->rx_buf;
	int ret;

	tx_buf[0] = WCD_SPI_CMD_RDSR;
	wcd_spi_reinit_xfer(tx_xfer);
	tx_xfer->tx_buf = tx_buf;
	tx_xfer->len = WCD_SPI_OPCODE_LEN;

	memset(rx_buf, 0, sizeof(*rdsr_status));
	wcd_spi_reinit_xfer(rx_xfer);
	rx_xfer->rx_buf = rx_buf;
	rx_xfer->len = sizeof(*rdsr_status);

	ret = spi_sync(spi, &wcd_spi->msg2);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: RDSR failed, err = %d\n",
			__func__, ret);
		goto done;
	}

	*rdsr_status = be32_to_cpu(*((u32*)rx_buf));

	dev_dbg(&spi->dev, "%s: RDSR success, value = 0x%x\n",
		__func__, *rdsr_status);
done:
	return ret;
}

static int wcd_spi_clk_enable(struct spi_device *spi)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	int ret;
	u32 rd_status = 0;

	/* Get the SPI access first */
	if (wcd_spi->ac_dev) {
		ret = wcd_spi_access_ctl(wcd_spi->ac_dev,
					 WCD_SPI_ACCESS_REQUEST,
					 WCD_SPI_AC_DATA_TRANSFER);
		if (ret) {
			dev_err(&spi->dev,
				"%s: Can't get spi access, err = %d\n",
				__func__, ret);
			return ret;
		}
	}

	ret = wcd_spi_cmd_nop(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: NOP1 failed, err = %d\n",
			__func__, ret);
		goto done;
	}

	ret = wcd_spi_cmd_clkreq(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: CLK_REQ failed, err = %d\n",
			__func__, ret);
		goto done;
	}

	ret = wcd_spi_cmd_nop(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: NOP2 failed, err = %d\n",
			__func__, ret);
		goto done;
	}
	wcd_spi_cmd_rdsr(spi, &rd_status);
	/*
	 * Read status zero means reads are not
	 * happenning on the bus, possibly because
	 * clock request failed.
	 */
	if (rd_status) {
		set_bit(WCD_SPI_CLK_STATE_ENABLED,
			&wcd_spi->status_mask);
	} else {
		dev_err(&spi->dev, "%s: RDSR status is zero\n",
			__func__);
		ret = -EIO;
	}
done:
	return ret;
}

static int wcd_spi_clk_disable(struct spi_device *spi)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	int ret;

	ret = wcd_spi_write_single(spi, WCD_SPI_ADDR_IPC_CTL_HOST, 0x01);
	if (ret < 0)
		dev_err(&spi->dev, "%s: Failed, err = %d\n",
			__func__, ret);
	/*
	 * clear this bit even if clock disable failed
	 * as the source clocks might get turned off.
	 */
	clear_bit(WCD_SPI_CLK_STATE_ENABLED, &wcd_spi->status_mask);

	/* once the clock is released, SPI access can be released as well */
	if (wcd_spi->ac_dev) {
		ret = wcd_spi_access_ctl(wcd_spi->ac_dev,
					 WCD_SPI_ACCESS_RELEASE,
					 WCD_SPI_AC_DATA_TRANSFER);
		if (ret)
			dev_err(&spi->dev,
				"%s: SPI access release failed, err = %d\n",
				__func__, ret);
	}

	return ret;
}

static int wcd_spi_clk_ctrl(struct spi_device *spi,
			    bool request, u32 flags)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	int ret = 0;
	const char *delay_str;

	delay_str = (flags == WCD_SPI_CLK_FLAG_DELAYED) ?
		    "delayed" : "immediate";

	WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);

	/* Reject any unbalanced disable request */
	if (wcd_spi->clk_users < 0 ||
	    (!request && wcd_spi->clk_users == 0)) {
		dev_err(&spi->dev, "%s: Unbalanced clk_users %d for %s\n",
			 __func__, wcd_spi->clk_users,
			request ? "enable" : "disable");
		ret = -EINVAL;

		/* Reset the clk_users to 0 */
		wcd_spi->clk_users = 0;

		goto done;
	}

	if (request == WCD_SPI_CLK_ENABLE) {
		/*
		 * If the SPI bus is suspended, then return error
		 * as the transaction cannot be completed.
		 */
		if (wcd_spi_is_suspended(wcd_spi)) {
			dev_err(&spi->dev,
				"%s: SPI suspended, cannot enable clk\n",
				__func__);
			ret = -EIO;
			goto done;
		}

		/* Cancel the disable clk work */
		WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);
		cancel_delayed_work_sync(&wcd_spi->clk_dwork);
		WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);

		wcd_spi->clk_users++;

		/*
		 * If clk state is already set,
		 * then clk wasnt really disabled
		 */
		if (test_bit(WCD_SPI_CLK_STATE_ENABLED, &wcd_spi->status_mask))
			goto done;
		else if (wcd_spi->clk_users == 1)
			ret = wcd_spi_clk_enable(spi);

	} else {
		wcd_spi->clk_users--;

		/* Clock is still voted for */
		if (wcd_spi->clk_users > 0)
			goto done;

		/*
		 * If we are here, clk_users must be 0 and needs
		 * to be disabled. Call the disable based on the
		 * flags.
		 */
		if (flags == WCD_SPI_CLK_FLAG_DELAYED) {
			schedule_delayed_work(&wcd_spi->clk_dwork,
				msecs_to_jiffies(WCD_SPI_CLK_OFF_TIMER_MS));
		} else {
			ret = wcd_spi_clk_disable(spi);
			if (ret < 0)
				dev_err(&spi->dev,
					"%s: Failed to disable clk err = %d\n",
					__func__, ret);
		}
	}

done:
	dev_dbg(&spi->dev, "%s: updated clk_users = %d, request_%s %s\n",
		__func__, wcd_spi->clk_users, request ? "enable" : "disable",
		request ? "" : delay_str);
	WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);

	return ret;
}

static int wcd_spi_init(struct spi_device *spi)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	int ret;

	ret = wcd_spi_clk_ctrl(spi, WCD_SPI_CLK_ENABLE,
			       WCD_SPI_CLK_FLAG_IMMEDIATE);
	if (ret < 0)
		goto done;

	ret = wcd_spi_cmd_wr_en(spi);
	if (ret < 0)
		goto err_wr_en;

	/*
	 * In case spi_init is called after component deinit,
	 * it is possible hardware register state is also reset.
	 * Sync the regcache here so hardware state is updated
	 * to reflect the cache.
	 */
	regcache_sync(wcd_spi->regmap);

	regmap_write(wcd_spi->regmap, WCD_SPI_SLAVE_CONFIG,
		     0x0F3D0800);

	/* Write the MTU to max allowed size */
	regmap_update_bits(wcd_spi->regmap,
			   WCD_SPI_SLAVE_TRNS_LEN,
			   0xFFFF0000, 0xFFFF0000);
err_wr_en:
	wcd_spi_clk_ctrl(spi, WCD_SPI_CLK_DISABLE,
			 WCD_SPI_CLK_FLAG_IMMEDIATE);
done:
	return ret;
}

static void wcd_spi_clk_work(struct work_struct *work)
{
	struct delayed_work *dwork;
	struct wcd_spi_priv *wcd_spi;
	struct spi_device *spi;
	int ret;

	dwork = to_delayed_work(work);
	wcd_spi = container_of(dwork, struct wcd_spi_priv, clk_dwork);
	spi = wcd_spi->spi;

	WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);
	ret = wcd_spi_clk_disable(spi);
	if (ret < 0)
		dev_err(&spi->dev,
			"%s: Failed to disable clk, err = %d\n",
			__func__, ret);
	WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);
}

static int __wcd_spi_data_xfer(struct spi_device *spi,
			       struct wcd_spi_msg *msg,
			       enum xfer_request xfer_req)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	int ret;

	/* Check for minimum alignment requirements */
	if (!IS_ALIGNED(msg->remote_addr, WCD_SPI_RW_MIN_ALIGN)) {
		dev_err(&spi->dev,
			"%s addr 0x%x is not aligned to 0x%x\n",
			__func__, msg->remote_addr, WCD_SPI_RW_MIN_ALIGN);
		return -EINVAL;
	} else if (msg->len % WCD_SPI_WORD_BYTE_CNT) {
		dev_err(&spi->dev,
			"%s len 0x%zx is not multiple of %d\n",
			__func__, msg->len, WCD_SPI_WORD_BYTE_CNT);
		return -EINVAL;
	}

	WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);
	if (wcd_spi_is_suspended(wcd_spi)) {
		dev_dbg(&spi->dev,
			"%s: SPI suspended, cannot perform transfer\n",
			__func__);
		ret = -EIO;
		goto done;
	}

	WCD_SPI_MUTEX_LOCK(spi, wcd_spi->xfer_mutex);
	if (msg->len == WCD_SPI_WORD_BYTE_CNT) {
		if (xfer_req == WCD_SPI_XFER_WRITE)
			ret = wcd_spi_write_single(spi, msg->remote_addr,
						   (*((u32 *)msg->data)));
		else
			ret = wcd_spi_read_single(spi, msg->remote_addr,
						  (u32 *) msg->data);
	} else {
		ret = wcd_spi_transfer_split(spi, msg, xfer_req);
	}
	WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->xfer_mutex);
done:
	WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);
	return ret;
}

static int wcd_spi_data_xfer(struct spi_device *spi,
			     struct wcd_spi_msg *msg,
			     enum xfer_request req)
{
	int ret, ret1;

	if (msg->len <= 0) {
		dev_err(&spi->dev, "%s: Invalid size %zd\n",
			__func__, msg->len);
		return -EINVAL;
	}

	/* Request for clock */
	ret = wcd_spi_clk_ctrl(spi, WCD_SPI_CLK_ENABLE,
			       WCD_SPI_CLK_FLAG_IMMEDIATE);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: clk enable failed %d\n",
			__func__, ret);
		goto done;
	}

	/* Perform the transaction */
	ret = __wcd_spi_data_xfer(spi, msg, req);
	if (ret < 0)
		dev_err(&spi->dev,
			"%s: Failed %s, addr = 0x%x, size = 0x%zx, err = %d\n",
			__func__, wcd_spi_xfer_req_str(req),
			msg->remote_addr, msg->len, ret);

	/* Release the clock even if xfer failed */
	ret1 = wcd_spi_clk_ctrl(spi, WCD_SPI_CLK_DISABLE,
				WCD_SPI_CLK_FLAG_DELAYED);
	if (ret1 < 0)
		dev_err(&spi->dev, "%s: clk disable failed %d\n",
			__func__, ret1);
done:
	return ret;
}

/*
 * wcd_spi_data_write: Write data to WCD SPI
 * @spi: spi_device struct
 * @msg: msg that needs to be written to WCD
 *
 * This API writes length of data to address specified. These details
 * about the write are encapsulated in @msg. Write size should be multiple
 * of 4 bytes and write address should be 4-byte aligned.
 */
static int wcd_spi_data_write(struct spi_device *spi,
		       struct wcd_spi_msg *msg)
{
	if (!spi || !msg) {
		pr_err("%s: Invalid %s\n", __func__,
			(!spi) ? "spi device" : "msg");
		return -EINVAL;
	}

	dev_dbg_ratelimited(&spi->dev, "%s: addr = 0x%x, len = %zu\n",
			    __func__, msg->remote_addr, msg->len);
	return wcd_spi_data_xfer(spi, msg, WCD_SPI_XFER_WRITE);
}

/*
 * wcd_spi_data_read: Read data from WCD SPI
 * @spi: spi_device struct
 * @msg: msg that needs to be read from WCD
 *
 * This API reads length of data from address specified. These details
 * about the read are encapsulated in @msg. Read size should be multiple
 * of 4 bytes and read address should be 4-byte aligned.
 */
static int wcd_spi_data_read(struct spi_device *spi,
		      struct wcd_spi_msg *msg)
{
	if (!spi || !msg) {
		pr_err("%s: Invalid %s\n", __func__,
			(!spi) ? "spi device" : "msg");
		return -EINVAL;
	}

	dev_dbg_ratelimited(&spi->dev, "%s: addr = 0x%x,len = %zu\n",
			    __func__, msg->remote_addr, msg->len);
	return wcd_spi_data_xfer(spi, msg, WCD_SPI_XFER_READ);
}

static int wdsp_spi_dload_section(struct spi_device *spi,
				  void *data)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct wdsp_img_section *sec = data;
	struct wcd_spi_msg msg;
	int ret;

	dev_dbg(&spi->dev, "%s: addr = 0x%x, size = 0x%zx\n",
		__func__, sec->addr, sec->size);

	msg.remote_addr = sec->addr + wcd_spi->mem_base_addr;
	msg.data = sec->data;
	msg.len = sec->size;

	ret = __wcd_spi_data_xfer(spi, &msg, WCD_SPI_XFER_WRITE);
	if (ret < 0)
		dev_err(&spi->dev, "%s: fail addr (0x%x) size (0x%zx)\n",
			__func__, msg.remote_addr, msg.len);
	return ret;
}

static int wdsp_spi_read_section(struct spi_device *spi, void *data)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct wdsp_img_section *sec = data;
	struct wcd_spi_msg msg;
	int ret;

	msg.remote_addr = sec->addr + wcd_spi->mem_base_addr;
	msg.data = sec->data;
	msg.len = sec->size;

	dev_dbg(&spi->dev, "%s: addr = 0x%x, size = 0x%zx\n",
		__func__, msg.remote_addr, msg.len);

	ret = wcd_spi_data_xfer(spi, &msg, WCD_SPI_XFER_READ);
	if (ret < 0)
		dev_err(&spi->dev, "%s: fail addr (0x%x) size (0x%zx)\n",
			__func__, msg.remote_addr, msg.len);
	return ret;
}

static int wdsp_spi_event_handler(struct device *dev, void *priv_data,
				  enum wdsp_event_type event,
				  void *data)
{
	struct spi_device *spi = to_spi_device(dev);
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct wcd_spi_ops *spi_ops;
	int ret = 0;

	dev_dbg(&spi->dev, "%s: event type %d\n",
		__func__, event);

	switch (event) {
	case WDSP_EVENT_PRE_SHUTDOWN:
		if (wcd_spi->ac_dev) {
			ret = wcd_spi_access_ctl(wcd_spi->ac_dev,
					 WCD_SPI_ACCESS_REQUEST,
					 WCD_SPI_AC_REMOTE_DOWN);
			if (ret)
				dev_err(&spi->dev,
					"%s: request access failed %d\n",
					__func__, ret);
		}
		break;

	case WDSP_EVENT_POST_SHUTDOWN:
		cancel_delayed_work_sync(&wcd_spi->clk_dwork);
		WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);
		if (test_bit(WCD_SPI_CLK_STATE_ENABLED, &wcd_spi->status_mask))
			wcd_spi_clk_disable(spi);
		wcd_spi->clk_users = 0;
		WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);
		break;

	case WDSP_EVENT_POST_BOOTUP:
		if (wcd_spi->ac_dev) {
			ret = wcd_spi_access_ctl(wcd_spi->ac_dev,
					 WCD_SPI_ACCESS_RELEASE,
					 WCD_SPI_AC_REMOTE_DOWN);
			if (ret)
				dev_err(&spi->dev,
					"%s: release access failed %d\n",
					__func__, ret);
		}
		break;

	case WDSP_EVENT_PRE_DLOAD_CODE:
	case WDSP_EVENT_PRE_DLOAD_DATA:
		ret = wcd_spi_clk_ctrl(spi, WCD_SPI_CLK_ENABLE,
				       WCD_SPI_CLK_FLAG_IMMEDIATE);
		if (ret < 0)
			dev_err(&spi->dev, "%s: clk_req failed %d\n",
				__func__, ret);
		break;

	case WDSP_EVENT_POST_DLOAD_CODE:
	case WDSP_EVENT_POST_DLOAD_DATA:
	case WDSP_EVENT_DLOAD_FAILED:

		ret = wcd_spi_clk_ctrl(spi, WCD_SPI_CLK_DISABLE,
				       WCD_SPI_CLK_FLAG_IMMEDIATE);
		if (ret < 0)
			dev_err(&spi->dev, "%s: clk unvote failed %d\n",
				__func__, ret);
		break;

	case WDSP_EVENT_DLOAD_SECTION:
		ret = wdsp_spi_dload_section(spi, data);
		break;

	case WDSP_EVENT_READ_SECTION:
		ret = wdsp_spi_read_section(spi, data);
		break;

	case WDSP_EVENT_SUSPEND:
		WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);
		if (!wcd_spi_can_suspend(wcd_spi))
			ret = -EBUSY;
		WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);
		break;

	case WDSP_EVENT_RESUME:
		ret = wcd_spi_wait_for_resume(wcd_spi);
		break;

	case WDSP_EVENT_GET_DEVOPS:
		if (!data) {
			dev_err(&spi->dev, "%s: invalid data\n",
				__func__);
			ret = -EINVAL;
			break;
		}

		spi_ops = (struct wcd_spi_ops *) data;
		spi_ops->spi_dev = spi;
		spi_ops->read_dev = wcd_spi_data_read;
		spi_ops->write_dev = wcd_spi_data_write;
		break;

	default:
		dev_dbg(&spi->dev, "%s: Unhandled event %d\n",
			__func__, event);
		break;
	}

	return ret;
}

static int wcd_spi_bus_gwrite(void *context, const void *reg,
			      size_t reg_len, const void *val,
			      size_t val_len)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	u8 *tx_buf = wcd_spi->tx_buf;

	if (!reg || !val || reg_len != wcd_spi->reg_bytes ||
	    val_len != wcd_spi->val_bytes) {
		dev_err(&spi->dev,
			"%s: Invalid input, reg_len = %zd, val_len = %zd",
			__func__, reg_len, val_len);
		return -EINVAL;
	}

	memset(tx_buf, 0, WCD_SPI_CMD_IRW_LEN);
	tx_buf[0] = WCD_SPI_CMD_IRW;
	tx_buf[1] = *((u8 *)reg);
	memcpy(tx_buf + WCD_SPI_OPCODE_LEN + reg_len,
	       val, val_len);

	return spi_write(spi, tx_buf, WCD_SPI_CMD_IRW_LEN);
}

static int wcd_spi_bus_write(void *context, const void *data,
			     size_t count)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);

	if (count < (wcd_spi->reg_bytes + wcd_spi->val_bytes)) {
		dev_err(&spi->dev, "%s: Invalid size %zd\n",
			__func__, count);
		WARN_ON(1);
		return -EINVAL;
	}

	return wcd_spi_bus_gwrite(context, data, wcd_spi->reg_bytes,
				  data + wcd_spi->reg_bytes,
				  count - wcd_spi->reg_bytes);
}

static int wcd_spi_bus_read(void *context, const void *reg,
			    size_t reg_len, void *val,
			    size_t val_len)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct spi_transfer *tx_xfer = &wcd_spi->xfer2[0];
	struct spi_transfer *rx_xfer = &wcd_spi->xfer2[1];
	u8 *tx_buf = wcd_spi->tx_buf;
	u8 *rx_buf = wcd_spi->rx_buf;
	int ret = 0;

	if (!reg || !val || reg_len != wcd_spi->reg_bytes ||
	    val_len != wcd_spi->val_bytes) {
		dev_err(&spi->dev,
			"%s: Invalid input, reg_len = %zd, val_len = %zd",
			__func__, reg_len, val_len);
		return -EINVAL;
	}

	memset(tx_buf, 0, WCD_SPI_CMD_IRR_LEN);
	tx_buf[0] = WCD_SPI_CMD_IRR;
	tx_buf[1] = *((u8 *)reg);

	wcd_spi_reinit_xfer(tx_xfer);
	tx_xfer->tx_buf = tx_buf;
	tx_xfer->rx_buf = NULL;
	tx_xfer->len = WCD_SPI_CMD_IRR_LEN;

	wcd_spi_reinit_xfer(rx_xfer);
	rx_xfer->tx_buf = NULL;
	rx_xfer->rx_buf = rx_buf;
	rx_xfer->len = val_len;

	ret = spi_sync(spi, &wcd_spi->msg2);
	if (ret) {
		dev_err(&spi->dev, "%s: spi_sync failed, err %d\n",
			__func__, ret);
		goto done;
	}

	memcpy(val, rx_buf, val_len);

done:
	return ret;
}

static struct regmap_bus wcd_spi_regmap_bus = {
	.write = wcd_spi_bus_write,
	.gather_write = wcd_spi_bus_gwrite,
	.read = wcd_spi_bus_read,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static int wcd_spi_state_show(struct seq_file *f, void *ptr)
{
	struct spi_device *spi = f->private;
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	const char *clk_state, *clk_mutex, *xfer_mutex;

	if (test_bit(WCD_SPI_CLK_STATE_ENABLED, &wcd_spi->status_mask))
		clk_state = "enabled";
	else
		clk_state = "disabled";

	clk_mutex = mutex_is_locked(&wcd_spi->clk_mutex) ?
		    "locked" : "unlocked";

	xfer_mutex = mutex_is_locked(&wcd_spi->xfer_mutex) ?
		     "locked" : "unlocked";

	seq_printf(f, "clk_state = %s\nclk_users = %d\n"
		   "clk_mutex = %s\nxfer_mutex = %s\n",
		   clk_state, wcd_spi->clk_users, clk_mutex,
		   xfer_mutex);
	return 0;
}

static int wcd_spi_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, wcd_spi_state_show, inode->i_private);
}

static const struct file_operations state_fops = {
	.open = wcd_spi_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t wcd_spi_debugfs_mem_read(struct file *file, char __user *ubuf,
					size_t count, loff_t *ppos)
{
	struct spi_device *spi = file->private_data;
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct wcd_spi_debug_data *dbg_data = &wcd_spi->debug_data;
	struct wcd_spi_msg msg;
	ssize_t buf_size, read_count = 0;
	char *buf;
	int ret;

	if (*ppos < 0 || !count)
		return -EINVAL;

	if (dbg_data->size == 0 || dbg_data->addr == 0) {
		dev_err(&spi->dev,
			"%s: Invalid request, size = %u, addr = 0x%x\n",
			__func__, dbg_data->size, dbg_data->addr);
		return 0;
	}

	buf_size = count < dbg_data->size ? count : dbg_data->size;
	buf = kzalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	msg.data = buf;
	msg.remote_addr = dbg_data->addr;
	msg.len = buf_size;
	msg.flags = 0;

	ret = wcd_spi_data_read(spi, &msg);
	if (ret < 0) {
		dev_err(&spi->dev,
			"%s: Failed to read %zu bytes from addr 0x%x\n",
			__func__, buf_size, msg.remote_addr);
		goto done;
	}

	read_count = simple_read_from_buffer(ubuf, count, ppos, buf, buf_size);

done:
	kfree(buf);
	if (ret < 0)
		return ret;
	else
		return read_count;
}

static const struct file_operations mem_read_fops = {
	.open = simple_open,
	.read = wcd_spi_debugfs_mem_read,
};

static int wcd_spi_debugfs_init(struct spi_device *spi)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct wcd_spi_debug_data *dbg_data = &wcd_spi->debug_data;
	int rc = 0;

	dbg_data->dir = debugfs_create_dir("wcd_spi", NULL);
	if (IS_ERR_OR_NULL(dbg_data->dir)) {
		dbg_data->dir = NULL;
		rc = -ENODEV;
		goto done;
	}

	debugfs_create_file("state", 0444, dbg_data->dir, spi, &state_fops);
	debugfs_create_u32("addr", 0644, dbg_data->dir,
			   &dbg_data->addr);
	debugfs_create_u32("size", 0644, dbg_data->dir,
			   &dbg_data->size);

	debugfs_create_file("mem_read", 0444, dbg_data->dir,
			    spi, &mem_read_fops);
done:
	return rc;
}


static const struct reg_default wcd_spi_defaults[] = {
	{WCD_SPI_SLAVE_SANITY, 0xDEADBEEF},
	{WCD_SPI_SLAVE_DEVICE_ID, 0x00500000},
	{WCD_SPI_SLAVE_STATUS, 0x80100000},
	{WCD_SPI_SLAVE_CONFIG, 0x0F200808},
	{WCD_SPI_SLAVE_SW_RESET, 0x00000000},
	{WCD_SPI_SLAVE_IRQ_STATUS, 0x00000000},
	{WCD_SPI_SLAVE_IRQ_EN, 0x00000000},
	{WCD_SPI_SLAVE_IRQ_CLR, 0x00000000},
	{WCD_SPI_SLAVE_IRQ_FORCE, 0x00000000},
	{WCD_SPI_SLAVE_TX, 0x00000000},
	{WCD_SPI_SLAVE_TEST_BUS_DATA, 0x00000000},
	{WCD_SPI_SLAVE_TEST_BUS_CTRL, 0x00000000},
	{WCD_SPI_SLAVE_SW_RST_IRQ, 0x00000000},
	{WCD_SPI_SLAVE_CHAR_CFG, 0x00000000},
	{WCD_SPI_SLAVE_CHAR_DATA_MOSI, 0x00000000},
	{WCD_SPI_SLAVE_CHAR_DATA_CS_N, 0x00000000},
	{WCD_SPI_SLAVE_CHAR_DATA_MISO, 0x00000000},
	{WCD_SPI_SLAVE_TRNS_BYTE_CNT, 0x00000000},
	{WCD_SPI_SLAVE_TRNS_LEN, 0x00000000},
	{WCD_SPI_SLAVE_FIFO_LEVEL, 0x00000000},
	{WCD_SPI_SLAVE_GENERICS, 0x80000000},
	{WCD_SPI_SLAVE_EXT_BASE_ADDR, 0x00000000},
};

static bool wcd_spi_is_volatile_reg(struct device *dev,
				    unsigned int reg)
{
	switch (reg) {
	case WCD_SPI_SLAVE_SANITY:
	case WCD_SPI_SLAVE_STATUS:
	case WCD_SPI_SLAVE_IRQ_STATUS:
	case WCD_SPI_SLAVE_TX:
	case WCD_SPI_SLAVE_SW_RST_IRQ:
	case WCD_SPI_SLAVE_TRNS_BYTE_CNT:
	case WCD_SPI_SLAVE_FIFO_LEVEL:
	case WCD_SPI_SLAVE_GENERICS:
		return true;
	}

	return false;
}

static bool wcd_spi_is_readable_reg(struct device *dev,
				    unsigned int reg)
{
	switch (reg) {
	case WCD_SPI_SLAVE_SW_RESET:
	case WCD_SPI_SLAVE_IRQ_CLR:
	case WCD_SPI_SLAVE_IRQ_FORCE:
		return false;
	}

	return true;
}

static struct regmap_config wcd_spi_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 32,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = wcd_spi_defaults,
	.num_reg_defaults = ARRAY_SIZE(wcd_spi_defaults),
	.max_register = WCD_SPI_MAX_REGISTER,
	.volatile_reg = wcd_spi_is_volatile_reg,
	.readable_reg = wcd_spi_is_readable_reg,
};

static int wcd_spi_add_ac_dev(struct device *dev,
			       struct device_node *node)
{
	struct spi_device *spi = to_spi_device(dev);
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct platform_device *pdev;
	int ret = 0;

	pdev = platform_device_alloc("wcd-spi-ac", -1);
	if (IS_ERR_OR_NULL(pdev)) {
		ret = PTR_ERR(pdev);
		dev_err(dev, "%s: pdev alloc failed, ret = %d\n",
			__func__, ret);
		return ret;
	}

	pdev->dev.parent = dev;
	pdev->dev.of_node = node;

	ret = platform_device_add(pdev);
	if (ret) {
		dev_err(dev, "%s: pdev add failed, ret = %d\n",
			__func__, ret);
		goto dealloc_pdev;
	}

	wcd_spi->ac_dev = &pdev->dev;
	return 0;

dealloc_pdev:
	platform_device_put(pdev);
	return ret;
}

static int wdsp_spi_init(struct device *dev, void *priv_data)
{
	struct spi_device *spi = to_spi_device(dev);
	int ret;
	struct device_node *node;

	for_each_child_of_node(dev->of_node, node) {
		if (!strcmp(node->name, "wcd_spi_ac"))
			wcd_spi_add_ac_dev(dev, node);
	}

	ret = wcd_spi_init(spi);
	if (ret < 0)
		dev_err(&spi->dev, "%s: Init failed, err = %d\n",
			__func__, ret);
	return ret;
}

static int wdsp_spi_deinit(struct device *dev, void *priv_data)
{
	struct spi_device *spi = to_spi_device(dev);
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);

	/*
	 * Deinit means the hardware is reset. Mark the cache
	 * as dirty here, so init will sync the cache
	 */
	regcache_mark_dirty(wcd_spi->regmap);

	return 0;
}

static struct wdsp_cmpnt_ops wdsp_spi_ops = {
	.init = wdsp_spi_init,
	.deinit = wdsp_spi_deinit,
	.event_handler = wdsp_spi_event_handler,
};

static int wcd_spi_component_bind(struct device *dev,
				  struct device *master,
				  void *data)
{
	struct spi_device *spi = to_spi_device(dev);
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	int ret = 0;

	wcd_spi->m_dev = master;
	wcd_spi->m_ops = data;

	if (wcd_spi->m_ops &&
	    wcd_spi->m_ops->register_cmpnt_ops)
		ret = wcd_spi->m_ops->register_cmpnt_ops(master, dev,
							 wcd_spi,
							 &wdsp_spi_ops);
	if (ret) {
		dev_err(dev, "%s: register_cmpnt_ops failed, err = %d\n",
			__func__, ret);
		goto done;
	}

	wcd_spi->reg_bytes = DIV_ROUND_UP(wcd_spi_regmap_cfg.reg_bits, 8);
	wcd_spi->val_bytes = DIV_ROUND_UP(wcd_spi_regmap_cfg.val_bits, 8);

	wcd_spi->regmap = devm_regmap_init(&spi->dev, &wcd_spi_regmap_bus,
					   &spi->dev, &wcd_spi_regmap_cfg);
	if (IS_ERR(wcd_spi->regmap)) {
		ret = PTR_ERR(wcd_spi->regmap);
		dev_err(&spi->dev, "%s: Failed to allocate regmap, err = %d\n",
			__func__, ret);
		goto done;
	}

	if (wcd_spi_debugfs_init(spi))
		dev_err(&spi->dev, "%s: Failed debugfs init\n", __func__);

	spi_message_init(&wcd_spi->msg1);
	spi_message_add_tail(&wcd_spi->xfer1, &wcd_spi->msg1);

	spi_message_init(&wcd_spi->msg2);
	spi_message_add_tail(&wcd_spi->xfer2[0], &wcd_spi->msg2);
	spi_message_add_tail(&wcd_spi->xfer2[1], &wcd_spi->msg2);

	/* Pre-allocate the buffers */
	wcd_spi->tx_buf = dma_alloc_coherent(&spi->dev,
					      WCD_SPI_RW_MAX_BUF_SIZE,
					      &wcd_spi->tx_dma, GFP_KERNEL);
	if (!wcd_spi->tx_buf) {
		ret = -ENOMEM;
		goto done;
	}

	wcd_spi->rx_buf = dma_alloc_coherent(&spi->dev,
					      WCD_SPI_RW_MAX_BUF_SIZE,
					      &wcd_spi->rx_dma, GFP_KERNEL);
	if (!wcd_spi->rx_buf) {
		dma_free_coherent(&spi->dev, WCD_SPI_RW_MAX_BUF_SIZE,
				  wcd_spi->tx_buf, wcd_spi->tx_dma);
		wcd_spi->tx_buf = NULL;
		ret = -ENOMEM;
		goto done;
	}
done:
	return ret;
}

static void wcd_spi_component_unbind(struct device *dev,
				     struct device *master,
				     void *data)
{
	struct spi_device *spi = to_spi_device(dev);
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	struct wcd_spi_debug_data *dbg_data = &wcd_spi->debug_data;

	debugfs_remove_recursive(dbg_data->dir);
	dbg_data->dir = NULL;

	wcd_spi->m_dev = NULL;
	wcd_spi->m_ops = NULL;

	spi_transfer_del(&wcd_spi->xfer1);
	spi_transfer_del(&wcd_spi->xfer2[0]);
	spi_transfer_del(&wcd_spi->xfer2[1]);

	dma_free_coherent(&spi->dev, WCD_SPI_RW_MAX_BUF_SIZE,
			  wcd_spi->tx_buf, wcd_spi->tx_dma);
	dma_free_coherent(&spi->dev, WCD_SPI_RW_MAX_BUF_SIZE,
			  wcd_spi->rx_buf, wcd_spi->rx_dma);
	wcd_spi->tx_buf = NULL;
	wcd_spi->rx_buf = NULL;
}

static const struct component_ops wcd_spi_component_ops = {
	.bind = wcd_spi_component_bind,
	.unbind = wcd_spi_component_unbind,
};

static int wcd_spi_probe(struct spi_device *spi)
{
	struct wcd_spi_priv *wcd_spi;
	int ret = 0;

	wcd_spi = devm_kzalloc(&spi->dev, sizeof(*wcd_spi),
			       GFP_KERNEL);
	if (!wcd_spi)
		return -ENOMEM;

	ret = of_property_read_u32(spi->dev.of_node,
				   "qcom,mem-base-addr",
				   &wcd_spi->mem_base_addr);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: Missing %s DT entry",
			__func__, "qcom,mem-base-addr");
		goto err_ret;
	}

	dev_dbg(&spi->dev,
		"%s: mem_base_addr 0x%x\n", __func__, wcd_spi->mem_base_addr);

	mutex_init(&wcd_spi->clk_mutex);
	mutex_init(&wcd_spi->xfer_mutex);
	INIT_DELAYED_WORK(&wcd_spi->clk_dwork, wcd_spi_clk_work);
	init_completion(&wcd_spi->resume_comp);
	arch_setup_dma_ops(&spi->dev, 0, 0, NULL, true);

	wcd_spi->spi = spi;
	spi_set_drvdata(spi, wcd_spi);

	ret = component_add(&spi->dev, &wcd_spi_component_ops);
	if (ret) {
		dev_err(&spi->dev, "%s: component_add failed err = %d\n",
			__func__, ret);
		goto err_component_add;
	}

	return ret;

err_component_add:
	mutex_destroy(&wcd_spi->clk_mutex);
	mutex_destroy(&wcd_spi->xfer_mutex);
err_ret:
	devm_kfree(&spi->dev, wcd_spi);
	spi_set_drvdata(spi, NULL);
	return ret;
}

static int wcd_spi_remove(struct spi_device *spi)
{
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);

	component_del(&spi->dev, &wcd_spi_component_ops);

	mutex_destroy(&wcd_spi->clk_mutex);
	mutex_destroy(&wcd_spi->xfer_mutex);

	devm_kfree(&spi->dev, wcd_spi);
	spi_set_drvdata(spi, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int wcd_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);
	int rc = 0;

	WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);
	if (!wcd_spi_can_suspend(wcd_spi)) {
		rc = -EBUSY;
		goto done;
	}

	/*
	 * If we are here, it is okay to let the suspend go
	 * through for this driver. But, still need to notify
	 * the master to make sure all other components can suspend
	 * as well.
	 */
	if (wcd_spi->m_dev && wcd_spi->m_ops &&
	  wcd_spi->m_ops->suspend) {
		WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);
		rc = wcd_spi->m_ops->suspend(wcd_spi->m_dev);
		WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);
	}

	if (rc == 0)
		set_bit(WCD_SPI_IS_SUSPENDED, &wcd_spi->status_mask);
	else
		dev_dbg(&spi->dev, "%s: cannot suspend, err = %d\n",
			__func__, rc);
done:
	WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);
	return rc;
}

static int wcd_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct wcd_spi_priv *wcd_spi = spi_get_drvdata(spi);

	WCD_SPI_MUTEX_LOCK(spi, wcd_spi->clk_mutex);
	clear_bit(WCD_SPI_IS_SUSPENDED, &wcd_spi->status_mask);
	complete(&wcd_spi->resume_comp);
	WCD_SPI_MUTEX_UNLOCK(spi, wcd_spi->clk_mutex);

	return 0;
}

static const struct dev_pm_ops wcd_spi_pm_ops = {
	.suspend = wcd_spi_suspend,
	.resume = wcd_spi_resume,
};
#endif

static const struct of_device_id wcd_spi_of_match[] = {
	{ .compatible = "qcom,wcd-spi-v2", },
	{ }
};
MODULE_DEVICE_TABLE(of, wcd_spi_of_match);

static struct spi_driver wcd_spi_driver = {
	.driver = {
		.name = "wcd-spi-v2",
		.of_match_table = wcd_spi_of_match,
#ifdef CONFIG_PM
		.pm = &wcd_spi_pm_ops,
#endif
	},
	.probe = wcd_spi_probe,
	.remove = wcd_spi_remove,
};

module_spi_driver(wcd_spi_driver);

MODULE_DESCRIPTION("WCD SPI driver");
MODULE_LICENSE("GPL v2");
