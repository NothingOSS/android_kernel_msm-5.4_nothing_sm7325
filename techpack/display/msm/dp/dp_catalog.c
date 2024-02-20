// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */


#include <linux/delay.h>
#include <linux/iopoll.h>

#include "dp_catalog.h"
#include "dp_reg.h"
#include "dp_debug.h"

#define DP_GET_MSB(x)	(x >> 8)
#define DP_GET_LSB(x)	(x & 0xff)

#define DP_PHY_READY BIT(1)

#define dp_catalog_get_priv(x) ({ \
	struct dp_catalog *dp_catalog; \
	dp_catalog = container_of(x, struct dp_catalog, x); \
	container_of(dp_catalog, struct dp_catalog_private, \
				dp_catalog); \
})

#define DP_INTERRUPT_STATUS1 \
	(DP_INTR_AUX_I2C_DONE| \
	DP_INTR_WRONG_ADDR | DP_INTR_TIMEOUT | \
	DP_INTR_NACK_DEFER | DP_INTR_WRONG_DATA_CNT | \
	DP_INTR_I2C_NACK | DP_INTR_I2C_DEFER | \
	DP_INTR_PLL_UNLOCKED | DP_INTR_AUX_ERROR)

#define DP_INTR_MASK1		(DP_INTERRUPT_STATUS1 << 2)

#define DP_INTERRUPT_STATUS2 \
	(DP_INTR_READY_FOR_VIDEO | DP_INTR_IDLE_PATTERN_SENT | \
	DP_INTR_FRAME_END | DP_INTR_CRC_UPDATED)

#define DP_INTR_MASK2		(DP_INTERRUPT_STATUS2 << 2)

#define DP_INTERRUPT_STATUS5 \
	(DP_INTR_MST_DP0_VCPF_SENT | DP_INTR_MST_DP1_VCPF_SENT)

#define DP_INTR_MASK5		(DP_INTERRUPT_STATUS5 << 2)

#define dp_catalog_fill_io(x) { \
	catalog->io.x = parser->get_io(parser, #x); \
}

#define dp_catalog_fill_io_buf(x) { \
	parser->get_io_buf(parser, #x); \
}

#define dp_read(x) ({ \
	catalog->read(catalog, io_data, x); \
})

#define dp_write(x, y) ({ \
	catalog->write(catalog, io_data, x, y); \
})

static u8 const vm_pre_emphasis[4][4] = {
	{0x00, 0x0B, 0x12, 0xFF},       /* pe0, 0 db */
	{0x00, 0x0A, 0x12, 0xFF},       /* pe1, 3.5 db */
	{0x00, 0x0C, 0xFF, 0xFF},       /* pe2, 6.0 db */
	{0xFF, 0xFF, 0xFF, 0xFF}        /* pe3, 9.5 db */
};

/* voltage swing, 0.2v and 1.0v are not support */
static u8 const vm_voltage_swing[4][4] = {
	{0x07, 0x0F, 0x14, 0xFF}, /* sw0, 0.4v  */
	{0x11, 0x1D, 0x1F, 0xFF}, /* sw1, 0.6 v */
	{0x18, 0x1F, 0xFF, 0xFF}, /* sw1, 0.8 v */
	{0xFF, 0xFF, 0xFF, 0xFF}  /* sw1, 1.2 v, optional */
};

static u8 const vm_pre_emphasis_hbr3_hbr2[4][4] = {
	{0x00, 0x0C, 0x15, 0x1A},
	{0x02, 0x0E, 0x16, 0xFF},
	{0x02, 0x11, 0xFF, 0xFF},
	{0x04, 0xFF, 0xFF, 0xFF}
};

static u8 const vm_voltage_swing_hbr3_hbr2[4][4] = {
	{0x02, 0x12, 0x16, 0x1A},
	{0x09, 0x19, 0x1F, 0xFF},
	{0x10, 0x1F, 0xFF, 0xFF},
	{0x1F, 0xFF, 0xFF, 0xFF}
};

static u8 const vm_pre_emphasis_hbr_rbr[4][4] = {
	{0x00, 0x0C, 0x14, 0x19},
	{0x00, 0x0B, 0x12, 0xFF},
	{0x00, 0x0B, 0xFF, 0xFF},
	{0x04, 0xFF, 0xFF, 0xFF}
};

static u8 const vm_voltage_swing_hbr_rbr[4][4] = {
	{0x08, 0x0F, 0x16, 0x1F},
	{0x11, 0x1E, 0x1F, 0xFF},
	{0x19, 0x1F, 0xFF, 0xFF},
	{0x1F, 0xFF, 0xFF, 0xFF}
};

enum dp_flush_bit {
	DP_PPS_FLUSH,
	DP_DHDR_FLUSH,
};

/* audio related catalog functions */
struct dp_catalog_private {
	struct device *dev;
	struct dp_catalog_io io;
	struct dp_parser *parser;

	u32 (*read)(struct dp_catalog_private *catalog,
		struct dp_io_data *io_data, u32 offset);
	void (*write)(struct dp_catalog_private *catlog,
		struct dp_io_data *io_data, u32 offset, u32 data);

	u32 (*audio_map)[DP_AUDIO_SDP_HEADER_MAX];
	struct dp_catalog dp_catalog;

	char exe_mode[SZ_4];
};

static u32 dp_read_sw(struct dp_catalog_private *catalog,
		struct dp_io_data *io_data, u32 offset)
{
	u32 data = 0;

	if (io_data->buf)
		memcpy(&data, io_data->buf + offset, sizeof(offset));

	return data;
}

static void dp_write_sw(struct dp_catalog_private *catalog,
	struct dp_io_data *io_data, u32 offset, u32 data)
{
	if (io_data->buf)
		memcpy(io_data->buf + offset, &data, sizeof(data));
}

static u32 dp_read_hw(struct dp_catalog_private *catalog,
	struct dp_io_data *io_data, u32 offset)
{
	u32 data = 0;

	data = readl_relaxed(io_data->io.base + offset);

	return data;
}

static void dp_write_hw(struct dp_catalog_private *catalog,
	struct dp_io_data *io_data, u32 offset, u32 data)
{
	writel_relaxed(data, io_data->io.base + offset);
}

static u32 dp_read_sub_sw(struct dp_catalog *dp_catalog,
		struct dp_io_data *io_data, u32 offset)
{
	struct dp_catalog_private *catalog = container_of(dp_catalog,
			struct dp_catalog_private, dp_catalog);

	return dp_read_sw(catalog, io_data, offset);
}

static void dp_write_sub_sw(struct dp_catalog *dp_catalog,
	struct dp_io_data *io_data, u32 offset, u32 data)
{
	struct dp_catalog_private *catalog = container_of(dp_catalog,
			struct dp_catalog_private, dp_catalog);

	dp_write_sw(catalog, io_data, offset, data);
}

static u32 dp_read_sub_hw(struct dp_catalog *dp_catalog,
	struct dp_io_data *io_data, u32 offset)
{
	struct dp_catalog_private *catalog = container_of(dp_catalog,
			struct dp_catalog_private, dp_catalog);

	return dp_read_hw(catalog, io_data, offset);
}

static void dp_write_sub_hw(struct dp_catalog *dp_catalog,
	struct dp_io_data *io_data, u32 offset, u32 data)
{
	struct dp_catalog_private *catalog = container_of(dp_catalog,
			struct dp_catalog_private, dp_catalog);

	dp_write_hw(catalog, io_data, offset, data);
}

/* aux related catalog functions */
static u32 dp_catalog_aux_read_data(struct dp_catalog_aux *aux)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!aux) {
		DP_ERR("invalid input\n");
		goto end;
	}

	catalog = dp_catalog_get_priv(aux);
	io_data = catalog->io.dp_aux;

	return dp_read(DP_AUX_DATA);
end:
	return 0;
}

static int dp_catalog_aux_write_data(struct dp_catalog_aux *aux)
{
	int rc = 0;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!aux) {
		DP_ERR("invalid input\n");
		rc = -EINVAL;
		goto end;
	}

	catalog = dp_catalog_get_priv(aux);
	io_data = catalog->io.dp_aux;

	dp_write(DP_AUX_DATA, aux->data);
end:
	return rc;
}

static int dp_catalog_aux_write_trans(struct dp_catalog_aux *aux)
{
	int rc = 0;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!aux) {
		DP_ERR("invalid input\n");
		rc = -EINVAL;
		goto end;
	}

	catalog = dp_catalog_get_priv(aux);
	io_data = catalog->io.dp_aux;

	dp_write(DP_AUX_TRANS_CTRL, aux->data);
end:
	return rc;
}

static int dp_catalog_aux_clear_trans(struct dp_catalog_aux *aux, bool read)
{
	int rc = 0;
	u32 data = 0;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!aux) {
		DP_ERR("invalid input\n");
		rc = -EINVAL;
		goto end;
	}

	catalog = dp_catalog_get_priv(aux);
	io_data = catalog->io.dp_aux;

	if (read) {
		data = dp_read(DP_AUX_TRANS_CTRL);
		data &= ~BIT(9);
		dp_write(DP_AUX_TRANS_CTRL, data);
	} else {
		dp_write(DP_AUX_TRANS_CTRL, 0);
	}
end:
	return rc;
}

static void dp_catalog_aux_clear_hw_interrupts(struct dp_catalog_aux *aux)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 data = 0;

	if (!aux) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(aux);
	io_data = catalog->io.dp_phy;

	data = dp_read(DP_PHY_AUX_INTERRUPT_STATUS);

	dp_write(DP_PHY_AUX_INTERRUPT_CLEAR, 0x1f);
	wmb(); /* make sure 0x1f is written before next write */
	dp_write(DP_PHY_AUX_INTERRUPT_CLEAR, 0x9f);
	wmb(); /* make sure 0x9f is written before next write */
	dp_write(DP_PHY_AUX_INTERRUPT_CLEAR, 0);
	wmb(); /* make sure register is cleared */
}

static void dp_catalog_aux_reset(struct dp_catalog_aux *aux)
{
	u32 aux_ctrl;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!aux) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(aux);
	io_data = catalog->io.dp_aux;

	aux_ctrl = dp_read(DP_AUX_CTRL);

	aux_ctrl |= BIT(1);
	dp_write(DP_AUX_CTRL, aux_ctrl);
	usleep_range(1000, 1010); /* h/w recommended delay */

	aux_ctrl &= ~BIT(1);

	dp_write(DP_AUX_CTRL, aux_ctrl);
	wmb(); /* make sure AUX reset is done here */
}

static void dp_catalog_aux_enable(struct dp_catalog_aux *aux, bool enable)
{
	u32 aux_ctrl;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!aux) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(aux);
	io_data = catalog->io.dp_aux;

	aux_ctrl = dp_read(DP_AUX_CTRL);

	if (enable) {
		aux_ctrl |= BIT(0);
		dp_write(DP_AUX_CTRL, aux_ctrl);
		wmb(); /* make sure AUX module is enabled */

		dp_write(DP_TIMEOUT_COUNT, 0xffff);
		dp_write(DP_AUX_LIMITS, 0xffff);
	} else {
		aux_ctrl &= ~BIT(0);
		dp_write(DP_AUX_CTRL, aux_ctrl);
	}
}

static void dp_catalog_aux_update_cfg(struct dp_catalog_aux *aux,
		struct dp_aux_cfg *cfg, enum dp_phy_aux_config_type type)
{
	struct dp_catalog_private *catalog;
	u32 new_index = 0, current_index = 0;
	struct dp_io_data *io_data;

	if (!aux || !cfg || (type >= PHY_AUX_CFG_MAX)) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(aux);

	io_data = catalog->io.dp_phy;

	current_index = cfg[type].current_index;
	new_index = (current_index + 1) % cfg[type].cfg_cnt;
	DP_DEBUG("Updating %s from 0x%08x to 0x%08x\n",
		dp_phy_aux_config_type_to_string(type),
	cfg[type].lut[current_index], cfg[type].lut[new_index]);

	dp_write(cfg[type].offset, cfg[type].lut[new_index]);
	cfg[type].current_index = new_index;
}

static void dp_catalog_aux_setup(struct dp_catalog_aux *aux,
		struct dp_aux_cfg *cfg)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	int i = 0;

	if (!aux || !cfg) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(aux);

	io_data = catalog->io.dp_phy;
	dp_write(DP_PHY_PD_CTL, 0x65);
	wmb(); /* make sure PD programming happened */

	/* Turn on BIAS current for PHY/PLL */
	io_data = catalog->io.dp_pll;
	dp_write(QSERDES_COM_BIAS_EN_CLKBUFLR_EN, 0x1b);

	io_data = catalog->io.dp_phy;
	dp_write(DP_PHY_PD_CTL, 0x02);
	wmb(); /* make sure PD programming happened */
	dp_write(DP_PHY_PD_CTL, 0x7d);

	/* Turn on BIAS current for PHY/PLL */
	io_data = catalog->io.dp_pll;
	dp_write(QSERDES_COM_BIAS_EN_CLKBUFLR_EN, 0x3f);

	/* DP AUX CFG register programming */
	io_data = catalog->io.dp_phy;
	for (i = 0; i < PHY_AUX_CFG_MAX; i++)
		dp_write(cfg[i].offset, cfg[i].lut[cfg[i].current_index]);

	dp_write(DP_PHY_AUX_INTERRUPT_MASK, 0x1F);
	wmb(); /* make sure AUX configuration is done before enabling it */
}

static void dp_catalog_aux_get_irq(struct dp_catalog_aux *aux, bool cmd_busy)
{
	u32 ack;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!aux) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(aux);
	io_data = catalog->io.dp_ahb;

	aux->isr = dp_read(DP_INTR_STATUS);
	aux->isr &= ~DP_INTR_MASK1;
	ack = aux->isr & DP_INTERRUPT_STATUS1;
	ack <<= 1;
	ack |= DP_INTR_MASK1;
	dp_write(DP_INTR_STATUS, ack);
}

static bool dp_catalog_ctrl_wait_for_phy_ready(
		struct dp_catalog_private *catalog)
{
	u32 reg = DP_PHY_STATUS, state;
	void __iomem *base = catalog->io.dp_phy->io.base;
	bool success = true;
	u32 const poll_sleep_us = 500;
	u32 const pll_timeout_us = 10000;

	if (readl_poll_timeout_atomic((base + reg), state,
			((state & DP_PHY_READY) > 0),
			poll_sleep_us, pll_timeout_us)) {
		DP_ERR("PHY status failed, status=%x\n", state);

		success = false;
	}

	return success;
}

/* controller related catalog functions */
static int dp_catalog_ctrl_late_phy_init(struct dp_catalog_ctrl *ctrl,
					u8 lane_cnt, bool flipped)
{
	int rc = 0;
	u32 bias0_en, drvr0_en, bias1_en, drvr1_en;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return -EINVAL;
	}

	catalog = dp_catalog_get_priv(ctrl);

	switch (lane_cnt) {
	case 1:
		drvr0_en = flipped ? 0x13 : 0x10;
		bias0_en = flipped ? 0x3E : 0x15;
		drvr1_en = flipped ? 0x10 : 0x13;
		bias1_en = flipped ? 0x15 : 0x3E;
		break;
	case 2:
		drvr0_en = flipped ? 0x10 : 0x10;
		bias0_en = flipped ? 0x3F : 0x15;
		drvr1_en = flipped ? 0x10 : 0x10;
		bias1_en = flipped ? 0x15 : 0x3F;
		break;
	case 4:
	default:
		drvr0_en = 0x10;
		bias0_en = 0x3F;
		drvr1_en = 0x10;
		bias1_en = 0x3F;
		break;
	}

	io_data = catalog->io.dp_ln_tx0;
	dp_write(TXn_HIGHZ_DRVR_EN_V420, drvr0_en);
	dp_write(TXn_TRANSCEIVER_BIAS_EN_V420, bias0_en);

	io_data = catalog->io.dp_ln_tx1;
	dp_write(TXn_HIGHZ_DRVR_EN_V420, drvr1_en);
	dp_write(TXn_TRANSCEIVER_BIAS_EN_V420, bias1_en);

	io_data = catalog->io.dp_phy;
	dp_write(DP_PHY_CFG, 0x18);
	/* add hardware recommended delay */
	udelay(2000);
	dp_write(DP_PHY_CFG, 0x19);

	/*
	 * Make sure all the register writes are completed before
	 * doing any other operation
	 */
	wmb();

	if (!dp_catalog_ctrl_wait_for_phy_ready(catalog)) {
		rc = -EINVAL;
		goto lock_err;
	}

	io_data = catalog->io.dp_ln_tx0;
	dp_write(TXn_TX_POL_INV_V420, 0x0a);
	io_data = catalog->io.dp_ln_tx1;
	dp_write(TXn_TX_POL_INV_V420, 0x0a);

	io_data = catalog->io.dp_ln_tx0;
	dp_write(TXn_TX_DRV_LVL_V420, 0x27);
	io_data = catalog->io.dp_ln_tx1;
	dp_write(TXn_TX_DRV_LVL_V420, 0x27);

	io_data = catalog->io.dp_ln_tx0;
	dp_write(TXn_TX_EMP_POST1_LVL, 0x20);
	io_data = catalog->io.dp_ln_tx1;
	dp_write(TXn_TX_EMP_POST1_LVL, 0x20);
	/* Make sure the PHY register writes are done */
	wmb();
lock_err:
	return rc;
}

static u32 dp_catalog_ctrl_read_hdcp_status(struct dp_catalog_ctrl *ctrl)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return -EINVAL;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_ahb;

	return dp_read(DP_HDCP_STATUS);
}

static void dp_catalog_panel_sdp_update(struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 sdp_cfg3_off = 0;

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	if (panel->stream_id == DP_STREAM_1)
		sdp_cfg3_off = MMSS_DP1_SDP_CFG3 - MMSS_DP_SDP_CFG3;

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	dp_write(MMSS_DP_SDP_CFG3 + sdp_cfg3_off, 0x01);
	dp_write(MMSS_DP_SDP_CFG3 + sdp_cfg3_off, 0x00);
}

static void dp_catalog_panel_setup_vsif_infoframe_sdp(
		struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct drm_msm_ext_hdr_metadata *hdr;
	struct dp_io_data *io_data;
	u32 header, parity, data, mst_offset = 0;
	u8 buf[SZ_64], off = 0;

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	if (panel->stream_id == DP_STREAM_1)
		mst_offset = MMSS_DP1_VSCEXT_0 - MMSS_DP_VSCEXT_0;

	catalog = dp_catalog_get_priv(panel);
	hdr = &panel->hdr_meta;
	io_data = catalog->io.dp_link;

	/* HEADER BYTE 1 */
	header = panel->dhdr_vsif_sdp.HB1;
	parity = dp_header_get_parity(header);
	data   = ((header << HEADER_BYTE_1_BIT)
			| (parity << PARITY_BYTE_1_BIT));
	dp_write(MMSS_DP_VSCEXT_0 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	/* HEADER BYTE 2 */
	header = panel->dhdr_vsif_sdp.HB2;
	parity = dp_header_get_parity(header);
	data   = ((header << HEADER_BYTE_2_BIT)
			| (parity << PARITY_BYTE_2_BIT));
	dp_write(MMSS_DP_VSCEXT_1 + mst_offset, data);

	/* HEADER BYTE 3 */
	header = panel->dhdr_vsif_sdp.HB3;
	parity = dp_header_get_parity(header);
	data   = ((header << HEADER_BYTE_3_BIT)
			| (parity << PARITY_BYTE_3_BIT));
	data |= dp_read(MMSS_DP_VSCEXT_1 + mst_offset);
	dp_write(MMSS_DP_VSCEXT_1 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	print_hex_dump_debug("[drm-dp] VSCEXT: ",
			DUMP_PREFIX_NONE, 16, 4, buf, off, false);
}

static void dp_catalog_panel_setup_hdr_infoframe_sdp(
		struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct drm_msm_ext_hdr_metadata *hdr;
	struct dp_io_data *io_data;
	u32 header, parity, data, mst_offset = 0;
	u8 buf[SZ_64], off = 0;
	u32 const version = 0x01;
	u32 const length = 0x1a;

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	if (panel->stream_id == DP_STREAM_1)
		mst_offset = MMSS_DP1_GENERIC2_0 - MMSS_DP_GENERIC2_0;

	catalog = dp_catalog_get_priv(panel);
	hdr = &panel->hdr_meta;
	io_data = catalog->io.dp_link;

	/* HEADER BYTE 1 */
	header = panel->shdr_if_sdp.HB1;
	parity = dp_header_get_parity(header);
	data   = ((header << HEADER_BYTE_1_BIT)
			| (parity << PARITY_BYTE_1_BIT));
	dp_write(MMSS_DP_GENERIC2_0 + mst_offset,
			data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	/* HEADER BYTE 2 */
	header = panel->shdr_if_sdp.HB2;
	parity = dp_header_get_parity(header);
	data   = ((header << HEADER_BYTE_2_BIT)
			| (parity << PARITY_BYTE_2_BIT));
	dp_write(MMSS_DP_GENERIC2_1 + mst_offset, data);

	/* HEADER BYTE 3 */
	header = panel->shdr_if_sdp.HB3;
	parity = dp_header_get_parity(header);
	data   = ((header << HEADER_BYTE_3_BIT)
			| (parity << PARITY_BYTE_3_BIT));
	data |= dp_read(MMSS_DP_VSCEXT_1 + mst_offset);
	dp_write(MMSS_DP_GENERIC2_1 + mst_offset,
			data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = version;
	data |= length << 8;
	data |= hdr->eotf << 16;
	dp_write(MMSS_DP_GENERIC2_2 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = (DP_GET_LSB(hdr->display_primaries_x[0]) |
		(DP_GET_MSB(hdr->display_primaries_x[0]) << 8) |
		(DP_GET_LSB(hdr->display_primaries_y[0]) << 16) |
		(DP_GET_MSB(hdr->display_primaries_y[0]) << 24));
	dp_write(MMSS_DP_GENERIC2_3 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = (DP_GET_LSB(hdr->display_primaries_x[1]) |
		(DP_GET_MSB(hdr->display_primaries_x[1]) << 8) |
		(DP_GET_LSB(hdr->display_primaries_y[1]) << 16) |
		(DP_GET_MSB(hdr->display_primaries_y[1]) << 24));
	dp_write(MMSS_DP_GENERIC2_4 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = (DP_GET_LSB(hdr->display_primaries_x[2]) |
		(DP_GET_MSB(hdr->display_primaries_x[2]) << 8) |
		(DP_GET_LSB(hdr->display_primaries_y[2]) << 16) |
		(DP_GET_MSB(hdr->display_primaries_y[2]) << 24));
	dp_write(MMSS_DP_GENERIC2_5 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = (DP_GET_LSB(hdr->white_point_x) |
		(DP_GET_MSB(hdr->white_point_x) << 8) |
		(DP_GET_LSB(hdr->white_point_y) << 16) |
		(DP_GET_MSB(hdr->white_point_y) << 24));
	dp_write(MMSS_DP_GENERIC2_6 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = (DP_GET_LSB(hdr->max_luminance) |
		(DP_GET_MSB(hdr->max_luminance) << 8) |
		(DP_GET_LSB(hdr->min_luminance) << 16) |
		(DP_GET_MSB(hdr->min_luminance) << 24));
	dp_write(MMSS_DP_GENERIC2_7 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = (DP_GET_LSB(hdr->max_content_light_level) |
		(DP_GET_MSB(hdr->max_content_light_level) << 8) |
		(DP_GET_LSB(hdr->max_average_light_level) << 16) |
		(DP_GET_MSB(hdr->max_average_light_level) << 24));
	dp_write(MMSS_DP_GENERIC2_8 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = 0;
	dp_write(MMSS_DP_GENERIC2_9 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	print_hex_dump_debug("[drm-dp] HDR: ",
			DUMP_PREFIX_NONE, 16, 4, buf, off, false);
}

static void dp_catalog_panel_setup_vsc_sdp(struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 header, parity, data, mst_offset = 0;
	u8 off = 0;
	u8 buf[SZ_128];

	if (!panel) {
		DP_ERR("invalid input\n");
		return;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	if (panel->stream_id == DP_STREAM_1)
		mst_offset = MMSS_DP1_GENERIC0_0 - MMSS_DP_GENERIC0_0;

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	/* HEADER BYTE 1 */
	header = panel->vsc_colorimetry.header.HB1;
	parity = dp_header_get_parity(header);
	data   = ((header << HEADER_BYTE_1_BIT)
			| (parity << PARITY_BYTE_1_BIT));
	dp_write(MMSS_DP_GENERIC0_0 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	/* HEADER BYTE 2 */
	header = panel->vsc_colorimetry.header.HB2;
	parity = dp_header_get_parity(header);
	data   = ((header << HEADER_BYTE_2_BIT)
			| (parity << PARITY_BYTE_2_BIT));
	dp_write(MMSS_DP_GENERIC0_1 + mst_offset, data);

	/* HEADER BYTE 3 */
	header = panel->vsc_colorimetry.header.HB3;
	parity = dp_header_get_parity(header);
	data   = ((header << HEADER_BYTE_3_BIT)
			| (parity << PARITY_BYTE_3_BIT));
	data |= dp_read(MMSS_DP_GENERIC0_1 + mst_offset);
	dp_write(MMSS_DP_GENERIC0_1 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = 0;
	dp_write(MMSS_DP_GENERIC0_2 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	dp_write(MMSS_DP_GENERIC0_3 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	dp_write(MMSS_DP_GENERIC0_4 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	dp_write(MMSS_DP_GENERIC0_5 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = (panel->vsc_colorimetry.data[16] & 0xFF) |
		((panel->vsc_colorimetry.data[17] & 0xFF) << 8) |
		((panel->vsc_colorimetry.data[18] & 0x7) << 16);

	dp_write(MMSS_DP_GENERIC0_6 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	data = 0;
	dp_write(MMSS_DP_GENERIC0_7 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	dp_write(MMSS_DP_GENERIC0_8 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	dp_write(MMSS_DP_GENERIC0_9 + mst_offset, data);
	memcpy(buf + off, &data, sizeof(data));
	off += sizeof(data);

	print_hex_dump_debug("[drm-dp] VSC: ",
			DUMP_PREFIX_NONE, 16, 4, buf, off, false);
}

static void dp_catalog_panel_config_sdp(struct dp_catalog_panel *panel,
	bool en)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 cfg, cfg2;
	u32 sdp_cfg_off = 0;
	u32 sdp_cfg2_off = 0;

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	if (panel->stream_id == DP_STREAM_1) {
		sdp_cfg_off = MMSS_DP1_SDP_CFG - MMSS_DP_SDP_CFG;
		sdp_cfg2_off = MMSS_DP1_SDP_CFG2 - MMSS_DP_SDP_CFG2;
	}

	cfg = dp_read(MMSS_DP_SDP_CFG + sdp_cfg_off);
	cfg2 = dp_read(MMSS_DP_SDP_CFG2 + sdp_cfg2_off);

	if (en) {
		/* GEN0_SDP_EN */
		cfg |= BIT(17);
		dp_write(MMSS_DP_SDP_CFG + sdp_cfg_off, cfg);

		/* GENERIC0_SDPSIZE */
		cfg2 |= BIT(16);
		dp_write(MMSS_DP_SDP_CFG2 + sdp_cfg2_off, cfg2);

		/* setup the GENERIC0 in case of en = true */
		dp_catalog_panel_setup_vsc_sdp(panel);

	} else {
		/* GEN0_SDP_EN */
		cfg &= ~BIT(17);
		dp_write(MMSS_DP_SDP_CFG + sdp_cfg_off, cfg);

		/* GENERIC0_SDPSIZE */
		cfg2 &= ~BIT(16);
		dp_write(MMSS_DP_SDP_CFG2 + sdp_cfg2_off, cfg2);
	}

	dp_catalog_panel_sdp_update(panel);
}

static void dp_catalog_panel_config_misc(struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 reg_offset = 0;

	if (!panel) {
		DP_ERR("invalid input\n");
		return;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	if (panel->stream_id == DP_STREAM_1)
		reg_offset = DP1_MISC1_MISC0 - DP_MISC1_MISC0;

	DP_DEBUG("misc settings = 0x%x\n", panel->misc_val);
	dp_write(DP_MISC1_MISC0 + reg_offset, panel->misc_val);
}

static int dp_catalog_panel_set_colorspace(struct dp_catalog_panel *panel,
bool vsc_supported)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!panel) {
		DP_ERR("invalid input\n");
		return -EINVAL;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return -EINVAL;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	if (vsc_supported) {
		dp_catalog_panel_setup_vsc_sdp(panel);
		dp_catalog_panel_sdp_update(panel);
	} else
		dp_catalog_panel_config_misc(panel);

	return 0;
}

static void dp_catalog_panel_config_hdr(struct dp_catalog_panel *panel, bool en,
	u32 dhdr_max_pkts, bool flush)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 cfg, cfg2, cfg4, misc;
	u32 sdp_cfg_off = 0;
	u32 sdp_cfg2_off = 0;
	u32 sdp_cfg4_off = 0;
	u32 misc1_misc0_off = 0;

	if (!panel) {
		DP_ERR("invalid input\n");
		return;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	if (panel->stream_id == DP_STREAM_1) {
		sdp_cfg_off = MMSS_DP1_SDP_CFG - MMSS_DP_SDP_CFG;
		sdp_cfg2_off = MMSS_DP1_SDP_CFG2 - MMSS_DP_SDP_CFG2;
		sdp_cfg4_off = MMSS_DP1_SDP_CFG4 - MMSS_DP_SDP_CFG4;
		misc1_misc0_off = DP1_MISC1_MISC0 - DP_MISC1_MISC0;
	}

	cfg = dp_read(MMSS_DP_SDP_CFG + sdp_cfg_off);
	cfg2 = dp_read(MMSS_DP_SDP_CFG2 + sdp_cfg2_off);
	misc = dp_read(DP_MISC1_MISC0 + misc1_misc0_off);

	if (en) {
		if (dhdr_max_pkts) {
			/* VSCEXT_SDP_EN */
			cfg |= BIT(16);
			/* DHDR_EN, DHDR_PACKET_LIMIT */
			cfg4 = (dhdr_max_pkts << 1) | BIT(0);
			dp_write(MMSS_DP_SDP_CFG4 + sdp_cfg4_off, cfg4);
			dp_catalog_panel_setup_vsif_infoframe_sdp(panel);
		}

		/* GEN2_SDP_EN */
		cfg |= BIT(19);
		dp_write(MMSS_DP_SDP_CFG + sdp_cfg_off, cfg);

		/* GENERIC2_SDPSIZE */
		cfg2 |= BIT(20);
		dp_write(MMSS_DP_SDP_CFG2 + sdp_cfg2_off, cfg2);

		dp_catalog_panel_setup_hdr_infoframe_sdp(panel);

		if (panel->hdr_meta.eotf)
			DP_DEBUG("Enabled\n");
		else
			DP_DEBUG("Reset\n");
	} else {
		/* VSCEXT_SDP_ENG */
		cfg &= ~BIT(16) & ~BIT(19);
		dp_write(MMSS_DP_SDP_CFG + sdp_cfg_off, cfg);

		/* GENERIC0_SDPSIZE GENERIC2_SDPSIZE */
		cfg2 &= ~BIT(20);
		dp_write(MMSS_DP_SDP_CFG2 + sdp_cfg2_off, cfg2);

		/* DHDR_EN, DHDR_PACKET_LIMIT */
		cfg4 = 0;
		dp_write(MMSS_DP_SDP_CFG4 + sdp_cfg4_off, cfg4);

		DP_DEBUG("Disabled\n");
	}

	if (flush) {
		DP_DEBUG("flushing HDR metadata\n");
		dp_catalog_panel_sdp_update(panel);
	}
}

static void dp_catalog_panel_update_transfer_unit(
		struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!panel || panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	dp_write(DP_VALID_BOUNDARY, panel->valid_boundary);
	dp_write(DP_TU, panel->dp_tu);
	dp_write(DP_VALID_BOUNDARY_2, panel->valid_boundary2);
}

static void dp_catalog_ctrl_state_ctrl(struct dp_catalog_ctrl *ctrl, u32 state)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_link;

	dp_write(DP_STATE_CTRL, state);
	/* make sure to change the hw state */
	wmb();
}

static void dp_catalog_ctrl_config_ctrl(struct dp_catalog_ctrl *ctrl, u8 ln_cnt)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 cfg;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_link;

	cfg = dp_read(DP_CONFIGURATION_CTRL);
	cfg &= ~(BIT(4) | BIT(5));
	cfg |= (ln_cnt - 1) << 4;
	dp_write(DP_CONFIGURATION_CTRL, cfg);

	cfg = dp_read(DP_MAINLINK_CTRL);
	cfg |= 0x02000000;
	dp_write(DP_MAINLINK_CTRL, cfg);

	DP_DEBUG("DP_MAINLINK_CTRL=0x%x\n", cfg);
}

static void dp_catalog_panel_config_ctrl(struct dp_catalog_panel *panel,
		u32 cfg)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 strm_reg_off = 0, mainlink_ctrl;

	if (!panel) {
		DP_ERR("invalid input\n");
		return;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	if (panel->stream_id == DP_STREAM_1)
		strm_reg_off = DP1_CONFIGURATION_CTRL - DP_CONFIGURATION_CTRL;

	DP_DEBUG("DP_CONFIGURATION_CTRL=0x%x\n", cfg);

	dp_write(DP_CONFIGURATION_CTRL + strm_reg_off, cfg);

	mainlink_ctrl = dp_read(DP_MAINLINK_CTRL);

	if (panel->stream_id == DP_STREAM_0)
		io_data = catalog->io.dp_p0;
	else if (panel->stream_id == DP_STREAM_1)
		io_data = catalog->io.dp_p1;

	if (mainlink_ctrl & BIT(8))
		dp_write(MMSS_DP_ASYNC_FIFO_CONFIG, 0x01);
	else
		dp_write(MMSS_DP_ASYNC_FIFO_CONFIG, 0x00);
}

static void dp_catalog_panel_config_dto(struct dp_catalog_panel *panel,
					bool ack)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 dsc_dto;

	if (!panel) {
		DP_ERR("invalid input\n");
		return;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	switch (panel->stream_id) {
	case DP_STREAM_0:
		io_data = catalog->io.dp_p0;
		break;
	case DP_STREAM_1:
		io_data = catalog->io.dp_p1;
		break;
	default:
		DP_ERR("invalid stream id\n");
		return;
	}

	dsc_dto = dp_read(MMSS_DP_DSC_DTO);
	if (ack)
		dsc_dto = BIT(1);
	else
		dsc_dto &= ~BIT(1);
	dp_write(MMSS_DP_DSC_DTO, dsc_dto);
}

static void dp_catalog_ctrl_lane_mapping(struct dp_catalog_ctrl *ctrl,
						bool flipped, char *lane_map)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_link;

	dp_write(DP_LOGICAL2PHYSICAL_LANE_MAPPING, 0xe4);
}

static void dp_catalog_ctrl_lane_pnswap(struct dp_catalog_ctrl *ctrl,
						u8 ln_pnswap)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 cfg0, cfg1;

	catalog = dp_catalog_get_priv(ctrl);

	cfg0 = 0x0a;
	cfg1 = 0x0a;

	cfg0 |= ((ln_pnswap >> 0) & 0x1) << 0;
	cfg0 |= ((ln_pnswap >> 1) & 0x1) << 2;
	cfg1 |= ((ln_pnswap >> 2) & 0x1) << 0;
	cfg1 |= ((ln_pnswap >> 3) & 0x1) << 2;

	io_data = catalog->io.dp_ln_tx0;
	dp_write(TXn_TX_POL_INV, cfg0);

	io_data = catalog->io.dp_ln_tx1;
	dp_write(TXn_TX_POL_INV, cfg1);
}

static void dp_catalog_ctrl_mainlink_ctrl(struct dp_catalog_ctrl *ctrl,
						bool enable)
{
	u32 mainlink_ctrl, reg;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_link;

	if (enable) {
		reg = dp_read(DP_MAINLINK_CTRL);
		mainlink_ctrl = reg & ~(0x03);
		dp_write(DP_MAINLINK_CTRL, mainlink_ctrl);
		wmb(); /* make sure mainlink is turned off before reset */
		mainlink_ctrl = reg | 0x02;
		dp_write(DP_MAINLINK_CTRL, mainlink_ctrl);
		wmb(); /* make sure mainlink entered reset */
		mainlink_ctrl = reg & ~(0x03);
		dp_write(DP_MAINLINK_CTRL, mainlink_ctrl);
		wmb(); /* make sure mainlink reset done */
		mainlink_ctrl = reg | 0x01;
		dp_write(DP_MAINLINK_CTRL, mainlink_ctrl);
		wmb(); /* make sure mainlink turned on */
	} else {
		mainlink_ctrl = dp_read(DP_MAINLINK_CTRL);
		mainlink_ctrl &= ~BIT(0);
		dp_write(DP_MAINLINK_CTRL, mainlink_ctrl);
	}
}

static void dp_catalog_panel_config_msa(struct dp_catalog_panel *panel,
					u32 rate, u32 stream_rate_khz)
{
	u32 pixel_m, pixel_n;
	u32 mvid, nvid;
	u32 const nvid_fixed = 0x8000;
	u32 const link_rate_hbr2 = 540000;
	u32 const link_rate_hbr3 = 810000;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 strm_reg_off = 0;
	u32 mvid_reg_off = 0, nvid_reg_off = 0;

	if (!panel) {
		DP_ERR("invalid input\n");
		return;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_mmss_cc;

	if (panel->stream_id == DP_STREAM_1)
		strm_reg_off = MMSS_DP_PIXEL1_M - MMSS_DP_PIXEL_M;

	pixel_m = dp_read(MMSS_DP_PIXEL_M + strm_reg_off);
	pixel_n = dp_read(MMSS_DP_PIXEL_N + strm_reg_off);
	DP_DEBUG("pixel_m=0x%x, pixel_n=0x%x\n", pixel_m, pixel_n);

	mvid = (pixel_m & 0xFFFF) * 5;
	nvid = (0xFFFF & (~pixel_n)) + (pixel_m & 0xFFFF);

	if (nvid < nvid_fixed) {
		u32 temp;

		temp = (nvid_fixed / nvid) * nvid;
		mvid = (nvid_fixed / nvid) * mvid;
		nvid = temp;
	}

	DP_DEBUG("rate = %d\n", rate);

	if (panel->widebus_en)
		mvid <<= 1;

	if (link_rate_hbr2 == rate)
		nvid *= 2;

	if (link_rate_hbr3 == rate)
		nvid *= 3;

	io_data = catalog->io.dp_link;

	if (panel->stream_id == DP_STREAM_1) {
		mvid_reg_off = DP1_SOFTWARE_MVID - DP_SOFTWARE_MVID;
		nvid_reg_off = DP1_SOFTWARE_NVID - DP_SOFTWARE_NVID;
	}

	DP_DEBUG("mvid=0x%x, nvid=0x%x\n", mvid, nvid);
	dp_write(DP_SOFTWARE_MVID + mvid_reg_off, mvid);
	dp_write(DP_SOFTWARE_NVID + nvid_reg_off, nvid);
}

static void dp_catalog_ctrl_set_pattern(struct dp_catalog_ctrl *ctrl,
					u32 pattern)
{
	int bit, cnt = 10;
	u32 data;
	const u32 link_training_offset = 3;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_link;

	switch (pattern) {
	case DP_TRAINING_PATTERN_4:
		bit = 3;
		break;
	case DP_TRAINING_PATTERN_3:
	case DP_TRAINING_PATTERN_2:
	case DP_TRAINING_PATTERN_1:
		bit = pattern - 1;
		break;
	default:
		DP_ERR("invalid pattern\n");
		return;
	}

	DP_DEBUG("hw: bit=%d train=%d\n", bit, pattern);
	dp_write(DP_STATE_CTRL, BIT(bit));

	bit += link_training_offset;

	while (cnt--) {
		data = dp_read(DP_MAINLINK_READY);
		if (data & BIT(bit))
			break;
	}

	if (cnt == 0)
		DP_ERR("set link_train=%d failed\n", pattern);
}

static void dp_catalog_ctrl_usb_reset(struct dp_catalog_ctrl *ctrl, bool flip)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);

	io_data = catalog->io.usb3_dp_com;

	DP_DEBUG("Program PHYMODE to DP only\n");
	dp_write(USB3_DP_COM_RESET_OVRD_CTRL, 0x0a);
	dp_write(USB3_DP_COM_PHY_MODE_CTRL, 0x02);
	dp_write(USB3_DP_COM_SW_RESET, 0x01);
	/* make sure usb3 com phy software reset is done */
	wmb();

	if (!flip) /* CC1 */
		dp_write(USB3_DP_COM_TYPEC_CTRL, 0x02);
	else /* CC2 */
		dp_write(USB3_DP_COM_TYPEC_CTRL, 0x03);

	dp_write(USB3_DP_COM_SWI_CTRL, 0x00);
	dp_write(USB3_DP_COM_SW_RESET, 0x00);
	/* make sure the software reset is done */
	wmb();

	dp_write(USB3_DP_COM_POWER_DOWN_CTRL, 0x01);
	dp_write(USB3_DP_COM_RESET_OVRD_CTRL, 0x00);
	/* make sure phy is brought out of reset */
	wmb();
}

static void dp_catalog_panel_tpg_cfg(struct dp_catalog_panel *panel,
	bool enable)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!panel) {
		DP_ERR("invalid input\n");
		return;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	catalog = dp_catalog_get_priv(panel);

	if (panel->stream_id == DP_STREAM_0)
		io_data = catalog->io.dp_p0;
	else if (panel->stream_id == DP_STREAM_1)
		io_data = catalog->io.dp_p1;

	if (!enable) {
		dp_write(MMSS_DP_TPG_MAIN_CONTROL, 0x0);
		dp_write(MMSS_DP_BIST_ENABLE, 0x0);
		dp_write(MMSS_DP_TIMING_ENGINE_EN, 0x0);
		wmb(); /* ensure Timing generator is turned off */
		return;
	}

	dp_write(MMSS_DP_INTF_CONFIG, 0x0);
	dp_write(MMSS_DP_INTF_HSYNC_CTL,
			panel->hsync_ctl);
	dp_write(MMSS_DP_INTF_VSYNC_PERIOD_F0,
			panel->vsync_period * panel->hsync_period);
	dp_write(MMSS_DP_INTF_VSYNC_PULSE_WIDTH_F0,
			panel->v_sync_width * panel->hsync_period);
	dp_write(MMSS_DP_INTF_VSYNC_PERIOD_F1, 0);
	dp_write(MMSS_DP_INTF_VSYNC_PULSE_WIDTH_F1, 0);
	dp_write(MMSS_DP_INTF_DISPLAY_HCTL, panel->display_hctl);
	dp_write(MMSS_DP_INTF_ACTIVE_HCTL, 0);
	dp_write(MMSS_INTF_DISPLAY_V_START_F0, panel->display_v_start);
	dp_write(MMSS_DP_INTF_DISPLAY_V_END_F0, panel->display_v_end);
	dp_write(MMSS_INTF_DISPLAY_V_START_F1, 0);
	dp_write(MMSS_DP_INTF_DISPLAY_V_END_F1, 0);
	dp_write(MMSS_DP_INTF_ACTIVE_V_START_F0, 0);
	dp_write(MMSS_DP_INTF_ACTIVE_V_END_F0, 0);
	dp_write(MMSS_DP_INTF_ACTIVE_V_START_F1, 0);
	dp_write(MMSS_DP_INTF_ACTIVE_V_END_F1, 0);
	dp_write(MMSS_DP_INTF_POLARITY_CTL, 0);
	wmb(); /* ensure TPG registers are programmed */

	dp_write(MMSS_DP_TPG_MAIN_CONTROL, 0x100);
	dp_write(MMSS_DP_TPG_VIDEO_CONFIG, 0x5);
	wmb(); /* ensure TPG config is programmed */
	dp_write(MMSS_DP_BIST_ENABLE, 0x1);
	dp_write(MMSS_DP_TIMING_ENGINE_EN, 0x1);
	wmb(); /* ensure Timing generator is turned on */
}

static void dp_catalog_panel_dsc_cfg(struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 reg, offset;
	int i;

	if (!panel) {
		DP_ERR("invalid input\n");
		return;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	catalog = dp_catalog_get_priv(panel);

	if (panel->stream_id == DP_STREAM_0)
		io_data = catalog->io.dp_p0;
	else
		io_data = catalog->io.dp_p1;

	dp_write(MMSS_DP_DSC_DTO_COUNT, panel->dsc.dto_count);

	reg = dp_read(MMSS_DP_DSC_DTO);
	if (panel->dsc.dto_en) {
		reg |= BIT(0);
		reg |= BIT(3);
		reg |= (panel->dsc.dto_n << 8);
		reg |= (panel->dsc.dto_d << 16);
	}
	dp_write(MMSS_DP_DSC_DTO, reg);

	io_data = catalog->io.dp_link;

	if (panel->stream_id == DP_STREAM_0)
		offset = 0;
	else
		offset = DP1_COMPRESSION_MODE_CTRL - DP_COMPRESSION_MODE_CTRL;

	dp_write(DP_PPS_HB_0_3 + offset, 0x7F1000);
	dp_write(DP_PPS_PB_0_3 + offset, 0xA22300);

	for (i = 0; i < panel->dsc.parity_word_len; i++)
		dp_write(DP_PPS_PB_4_7 + (i << 2) + offset,
				panel->dsc.parity_word[i]);

	for (i = 0; i < panel->dsc.pps_word_len; i++)
		dp_write(DP_PPS_PPS_0_3 + (i << 2) + offset,
				panel->dsc.pps_word[i]);

	reg = 0;
	if (panel->dsc.dsc_en) {
		reg = BIT(0);
		reg |= (panel->dsc.eol_byte_num << 3);
		reg |= (panel->dsc.slice_per_pkt << 5);
		reg |= (panel->dsc.bytes_per_pkt << 16);
		reg |= (panel->dsc.be_in_lane << 10);
	}
	dp_write(DP_COMPRESSION_MODE_CTRL + offset, reg);

	DP_DEBUG("compression:0x%x for stream:%d\n",
			reg, panel->stream_id);
}

static void dp_catalog_panel_dp_flush(struct dp_catalog_panel *panel,
		enum dp_flush_bit flush_bit)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 dp_flush, offset;
	struct dp_dsc_cfg_data *dsc;

	if (!panel) {
		DP_ERR("invalid input\n");
		return;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;
	dsc = &panel->dsc;

	if (panel->stream_id == DP_STREAM_0)
		offset = 0;
	else
		offset = MMSS_DP1_FLUSH - MMSS_DP_FLUSH;

	dp_flush = dp_read(MMSS_DP_FLUSH + offset);

	if ((flush_bit == DP_PPS_FLUSH) &&
		dsc->continuous_pps)
		dp_flush &= ~BIT(2);

	dp_flush |= BIT(flush_bit);
	dp_write(MMSS_DP_FLUSH + offset, dp_flush);
}

static void dp_catalog_panel_pps_flush(struct dp_catalog_panel *panel)
{
	dp_catalog_panel_dp_flush(panel, DP_PPS_FLUSH);
	DP_DEBUG("pps flush for stream:%d\n", panel->stream_id);
}

static void dp_catalog_panel_dhdr_flush(struct dp_catalog_panel *panel)
{
	dp_catalog_panel_dp_flush(panel, DP_DHDR_FLUSH);
	DP_DEBUG("dhdr flush for stream:%d\n", panel->stream_id);
}


static bool dp_catalog_panel_dhdr_busy(struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 dp_flush, offset;

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		return false;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	if (panel->stream_id == DP_STREAM_0)
		offset = 0;
	else
		offset = MMSS_DP1_FLUSH - MMSS_DP_FLUSH;

	dp_flush = dp_read(MMSS_DP_FLUSH + offset);

	return dp_flush & BIT(DP_DHDR_FLUSH) ? true : false;
}

static void dp_catalog_ctrl_reset(struct dp_catalog_ctrl *ctrl)
{
	u32 sw_reset;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_ahb;

	sw_reset = dp_read(DP_SW_RESET);

	sw_reset |= BIT(0);
	dp_write(DP_SW_RESET, sw_reset);
	usleep_range(1000, 1010); /* h/w recommended delay */

	sw_reset &= ~BIT(0);
	dp_write(DP_SW_RESET, sw_reset);
}

static bool dp_catalog_ctrl_mainlink_ready(struct dp_catalog_ctrl *ctrl)
{
	u32 data;
	int cnt = 10;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		goto end;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_link;

	while (--cnt) {
		/* DP_MAINLINK_READY */
		data = dp_read(DP_MAINLINK_READY);
		if (data & BIT(0))
			return true;

		usleep_range(1000, 1010); /* 1ms wait before next reg read */
	}
	DP_ERR("mainlink not ready\n");
end:
	return false;
}

static void dp_catalog_ctrl_enable_irq(struct dp_catalog_ctrl *ctrl,
						bool enable)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_ahb;

	if (enable) {
		dp_write(DP_INTR_STATUS, DP_INTR_MASK1);
		dp_write(DP_INTR_STATUS2, DP_INTR_MASK2);
		dp_write(DP_INTR_STATUS5, DP_INTR_MASK5);
	} else {
		dp_write(DP_INTR_STATUS, 0x00);
		dp_write(DP_INTR_STATUS2, 0x00);
		dp_write(DP_INTR_STATUS5, 0x00);
	}
}

static void dp_catalog_ctrl_get_interrupt(struct dp_catalog_ctrl *ctrl)
{
	u32 ack = 0;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_ahb;

	ctrl->isr = dp_read(DP_INTR_STATUS2);
	ctrl->isr &= ~DP_INTR_MASK2;
	ack = ctrl->isr & DP_INTERRUPT_STATUS2;
	ack <<= 1;
	ack |= DP_INTR_MASK2;
	dp_write(DP_INTR_STATUS2, ack);

	ctrl->isr5 = dp_read(DP_INTR_STATUS5);
	ctrl->isr5 &= ~DP_INTR_MASK5;
	ack = ctrl->isr5 & DP_INTERRUPT_STATUS5;
	ack <<= 1;
	ack |= DP_INTR_MASK5;
	dp_write(DP_INTR_STATUS5, ack);
}

static void dp_catalog_ctrl_phy_reset(struct dp_catalog_ctrl *ctrl)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_ahb;

	dp_write(DP_PHY_CTRL, 0x5); /* bit 0 & 2 */
	usleep_range(1000, 1010); /* h/w recommended delay */
	dp_write(DP_PHY_CTRL, 0x0);
	wmb(); /* make sure PHY reset done */
}

static void dp_catalog_ctrl_phy_lane_cfg(struct dp_catalog_ctrl *ctrl,
		bool flipped, u8 ln_cnt)
{
	u32 info = 0x0;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u8 orientation = BIT(!!flipped);

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);

	io_data = catalog->io.dp_phy;

	info |= (ln_cnt & 0x0F);
	info |= ((orientation & 0x0F) << 4);
	DP_DEBUG("Shared Info = 0x%x\n", info);

	dp_write(DP_PHY_SPARE0, info);
}

static void dp_catalog_ctrl_update_vx_px(struct dp_catalog_ctrl *ctrl,
		u8 v_level, u8 p_level, bool high)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u8 value0, value1;
	u32 version;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);

	DP_DEBUG("hw: v=%d p=%d\n", v_level, p_level);

	io_data = catalog->io.dp_ahb;
	version = dp_read(DP_HW_VERSION);

	if (version == 0x10020004) {
		if (high) {
			value0 = vm_voltage_swing_hbr3_hbr2[v_level][p_level];
			value1 = vm_pre_emphasis_hbr3_hbr2[v_level][p_level];
		} else {
			value0 = vm_voltage_swing_hbr_rbr[v_level][p_level];
			value1 = vm_pre_emphasis_hbr_rbr[v_level][p_level];
		}
	} else {
		value0 = vm_voltage_swing[v_level][p_level];
		value1 = vm_pre_emphasis[v_level][p_level];
	}

	/* program default setting first */

	io_data = catalog->io.dp_ln_tx0;
	dp_write(TXn_TX_DRV_LVL, 0x2A);
	dp_write(TXn_TX_EMP_POST1_LVL, 0x20);

	io_data = catalog->io.dp_ln_tx1;
	dp_write(TXn_TX_DRV_LVL, 0x2A);
	dp_write(TXn_TX_EMP_POST1_LVL, 0x20);

	/* Enable MUX to use Cursor values from these registers */
	value0 |= BIT(5);
	value1 |= BIT(5);

	/* Configure host and panel only if both values are allowed */
	if (value0 != 0xFF && value1 != 0xFF) {
		io_data = catalog->io.dp_ln_tx0;
		dp_write(TXn_TX_DRV_LVL, value0);
		dp_write(TXn_TX_EMP_POST1_LVL, value1);

		io_data = catalog->io.dp_ln_tx1;
		dp_write(TXn_TX_DRV_LVL, value0);
		dp_write(TXn_TX_EMP_POST1_LVL, value1);

		DP_DEBUG("hw: vx_value=0x%x px_value=0x%x\n",
			value0, value1);
	} else {
		DP_ERR("invalid vx (0x%x=0x%x), px (0x%x=0x%x\n",
			v_level, value0, p_level, value1);
	}
}

static void dp_catalog_ctrl_send_phy_pattern(struct dp_catalog_ctrl *ctrl,
			u32 pattern)
{
	struct dp_catalog_private *catalog;
	u32 value = 0x0;
	struct dp_io_data *io_data = NULL;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);

	io_data = catalog->io.dp_link;

	dp_write(DP_STATE_CTRL, 0x0);

	switch (pattern) {
	case DP_TEST_PHY_PATTERN_D10_2_NO_SCRAMBLING:
		dp_write(DP_STATE_CTRL, 0x1);
		break;
	case DP_TEST_PHY_PATTERN_SYMBOL_ERR_MEASUREMENT_CNT:
		value &= ~(1 << 16);
		dp_write(DP_HBR2_COMPLIANCE_SCRAMBLER_RESET, value);
		value |= 0xFC;
		dp_write(DP_HBR2_COMPLIANCE_SCRAMBLER_RESET, value);
		dp_write(DP_MAINLINK_LEVELS, 0x2);
		dp_write(DP_STATE_CTRL, 0x10);
		break;
	case DP_TEST_PHY_PATTERN_PRBS7:
		dp_write(DP_STATE_CTRL, 0x20);
		break;
	case DP_TEST_PHY_PATTERN_80_BIT_CUSTOM_PATTERN:
		dp_write(DP_STATE_CTRL, 0x40);
		/* 00111110000011111000001111100000 */
		dp_write(DP_TEST_80BIT_CUSTOM_PATTERN_REG0, 0x3E0F83E0);
		/* 00001111100000111110000011111000 */
		dp_write(DP_TEST_80BIT_CUSTOM_PATTERN_REG1, 0x0F83E0F8);
		/* 1111100000111110 */
		dp_write(DP_TEST_80BIT_CUSTOM_PATTERN_REG2, 0x0000F83E);
		break;
	case DP_TEST_PHY_PATTERN_CP2520_PATTERN_1:
		value = dp_read(DP_MAINLINK_CTRL);
		value &= ~BIT(4);
		dp_write(DP_MAINLINK_CTRL, value);

		value = BIT(16);
		dp_write(DP_HBR2_COMPLIANCE_SCRAMBLER_RESET, value);
		value |= 0xFC;
		dp_write(DP_HBR2_COMPLIANCE_SCRAMBLER_RESET, value);
		dp_write(DP_MAINLINK_LEVELS, 0x2);
		dp_write(DP_STATE_CTRL, 0x10);

		value = dp_read(DP_MAINLINK_CTRL);
		value |= BIT(0);
		dp_write(DP_MAINLINK_CTRL, value);
		break;
	case DP_TEST_PHY_PATTERN_CP2520_PATTERN_3:
		dp_write(DP_MAINLINK_CTRL, 0x01);
		dp_write(DP_STATE_CTRL, 0x8);
		break;
	default:
		DP_DEBUG("No valid test pattern requested: 0x%x\n", pattern);
		return;
	}

	/* Make sure the test pattern is programmed in the hardware */
	wmb();
}

static u32 dp_catalog_ctrl_read_phy_pattern(struct dp_catalog_ctrl *ctrl)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data = NULL;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return 0;
	}

	catalog = dp_catalog_get_priv(ctrl);

	io_data = catalog->io.dp_link;

	return dp_read(DP_MAINLINK_READY);
}

static void dp_catalog_ctrl_fec_config(struct dp_catalog_ctrl *ctrl,
		bool enable)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data = NULL;
	u32 reg;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);
	io_data = catalog->io.dp_link;

	reg = dp_read(DP_MAINLINK_CTRL);

	/*
	 * fec_en = BIT(12)
	 * fec_seq_mode = BIT(22)
	 * sde_flush = BIT(23) | BIT(24)
	 * fb_boundary_sel = BIT(25)
	 */
	if (enable)
		reg |= BIT(12) | BIT(22) | BIT(23) | BIT(24) | BIT(25);
	else
		reg &= ~BIT(12);

	dp_write(DP_MAINLINK_CTRL, reg);
	/* make sure mainlink configuration is updated with fec sequence */
	wmb();
}

static int dp_catalog_reg_dump(struct dp_catalog *dp_catalog,
		char *name, u8 **out_buf, u32 *out_buf_len)
{
	int ret = 0;
	u8 *buf;
	u32 len;
	struct dp_io_data *io_data;
	struct dp_catalog_private *catalog;
	struct dp_parser *parser;

	if (!dp_catalog) {
		DP_ERR("invalid input\n");
		return -EINVAL;
	}

	catalog = container_of(dp_catalog, struct dp_catalog_private,
		dp_catalog);

	parser = catalog->parser;
	parser->get_io_buf(parser, name);
	io_data = parser->get_io(parser, name);
	if (!io_data) {
		DP_ERR("IO %s not found\n", name);
		ret = -EINVAL;
		goto end;
	}

	buf = io_data->buf;
	len = io_data->io.len;

	if (!buf || !len) {
		DP_ERR("no buffer available\n");
		ret = -ENOMEM;
		goto end;
	}

	if (!strcmp(catalog->exe_mode, "hw") ||
	    !strcmp(catalog->exe_mode, "all")) {
		u32 i, data;
		u32 const rowsize = 4;
		void __iomem *addr = io_data->io.base;

		memset(buf, 0, len);

		for (i = 0; i < len / rowsize; i++) {
			data = readl_relaxed(addr);
			memcpy(buf + (rowsize * i), &data, sizeof(u32));

			addr += rowsize;
		}
	}

	*out_buf = buf;
	*out_buf_len = len;
end:
	if (ret)
		parser->clear_io_buf(parser);

	return ret;
}

static void dp_catalog_ctrl_mst_config(struct dp_catalog_ctrl *ctrl,
		bool enable)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data = NULL;
	u32 reg;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);

	io_data = catalog->io.dp_link;

	reg = dp_read(DP_MAINLINK_CTRL);
	if (enable)
		reg |= (0x04000100);
	else
		reg &= ~(0x04000100);

	dp_write(DP_MAINLINK_CTRL, reg);
	/* make sure mainlink MST configuration is updated */
	wmb();
}

static void dp_catalog_ctrl_trigger_act(struct dp_catalog_ctrl *ctrl)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data = NULL;

	if (!ctrl) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);

	io_data = catalog->io.dp_link;

	dp_write(DP_MST_ACT, 0x1);
	/* make sure ACT signal is performed */
	wmb();
}

static void dp_catalog_ctrl_read_act_complete_sts(struct dp_catalog_ctrl *ctrl,
		bool *sts)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data = NULL;
	u32 reg;

	if (!ctrl || !sts) {
		DP_ERR("invalid input\n");
		return;
	}

	*sts = false;

	catalog = dp_catalog_get_priv(ctrl);

	io_data = catalog->io.dp_link;

	reg = dp_read(DP_MST_ACT);

	if (!reg)
		*sts = true;
}

static void dp_catalog_ctrl_channel_alloc(struct dp_catalog_ctrl *ctrl,
			u32 ch, u32 ch_start_slot, u32 tot_slot_cnt)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data = NULL;
	u32 i, slot_reg_1, slot_reg_2, slot;
	u32 reg_off = 0;
	int const num_slots_per_reg = 32;

	if (!ctrl || ch >= DP_STREAM_MAX) {
		DP_ERR("invalid input. ch %d\n", ch);
		return;
	}

	if (ch_start_slot > DP_MAX_TIME_SLOTS ||
			(ch_start_slot + tot_slot_cnt > DP_MAX_TIME_SLOTS)) {
		DP_ERR("invalid slots start %d, tot %d\n",
			ch_start_slot, tot_slot_cnt);
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);

	io_data = catalog->io.dp_link;

	DP_DEBUG("ch %d, start_slot %d, tot_slot %d\n",
			ch, ch_start_slot, tot_slot_cnt);

	if (ch == DP_STREAM_1)
		reg_off = DP_DP1_TIMESLOT_1_32 - DP_DP0_TIMESLOT_1_32;

	slot_reg_1 = 0;
	slot_reg_2 = 0;

	if (ch_start_slot && tot_slot_cnt) {
		ch_start_slot--;
		for (i = 0; i < tot_slot_cnt; i++) {
			if (ch_start_slot < num_slots_per_reg) {
				slot_reg_1 |= BIT(ch_start_slot);
			} else {
				slot = ch_start_slot - num_slots_per_reg;
				slot_reg_2 |= BIT(slot);
			}
			ch_start_slot++;
		}
	}

	DP_DEBUG("ch:%d slot_reg_1:%d, slot_reg_2:%d\n", ch,
			slot_reg_1, slot_reg_2);

	dp_write(DP_DP0_TIMESLOT_1_32 + reg_off, slot_reg_1);
	dp_write(DP_DP0_TIMESLOT_33_63 + reg_off, slot_reg_2);
}

static void dp_catalog_ctrl_channel_dealloc(struct dp_catalog_ctrl *ctrl,
			u32 ch, u32 ch_start_slot, u32 tot_slot_cnt)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data = NULL;
	u32 i, slot_reg_1, slot_reg_2, slot;
	u32 reg_off = 0;

	if (!ctrl || ch >= DP_STREAM_MAX) {
		DP_ERR("invalid input. ch %d\n", ch);
		return;
	}

	if (ch_start_slot > DP_MAX_TIME_SLOTS ||
			(ch_start_slot + tot_slot_cnt > DP_MAX_TIME_SLOTS)) {
		DP_ERR("invalid slots start %d, tot %d\n",
			ch_start_slot, tot_slot_cnt);
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);

	io_data = catalog->io.dp_link;

	DP_DEBUG("dealloc ch %d, start_slot %d, tot_slot %d\n",
			ch, ch_start_slot, tot_slot_cnt);

	if (ch == DP_STREAM_1)
		reg_off = DP_DP1_TIMESLOT_1_32 - DP_DP0_TIMESLOT_1_32;

	slot_reg_1 = dp_read(DP_DP0_TIMESLOT_1_32 + reg_off);
	slot_reg_2 = dp_read(DP_DP0_TIMESLOT_33_63 + reg_off);

	ch_start_slot = ch_start_slot - 1;
	for (i = 0; i < tot_slot_cnt; i++) {
		if (ch_start_slot < 33) {
			slot_reg_1 &= ~BIT(ch_start_slot);
		} else {
			slot = ch_start_slot - 33;
			slot_reg_2 &= ~BIT(slot);
		}
		ch_start_slot++;
	}

	DP_DEBUG("dealloc ch:%d slot_reg_1:%d, slot_reg_2:%d\n", ch,
			slot_reg_1, slot_reg_2);

	dp_write(DP_DP0_TIMESLOT_1_32 + reg_off, slot_reg_1);
	dp_write(DP_DP0_TIMESLOT_33_63 + reg_off, slot_reg_2);
}

static void dp_catalog_ctrl_update_rg(struct dp_catalog_ctrl *ctrl, u32 ch,
		u32 x_int, u32 y_frac_enum)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data = NULL;
	u32 rg, reg_off = 0;

	if (!ctrl || ch >= DP_STREAM_MAX) {
		DP_ERR("invalid input. ch %d\n", ch);
		return;
	}

	catalog = dp_catalog_get_priv(ctrl);

	io_data = catalog->io.dp_link;

	rg = y_frac_enum;
	rg |= (x_int << 16);

	DP_DEBUG("ch: %d x_int:%d y_frac_enum:%d rg:%d\n", ch, x_int,
			y_frac_enum, rg);

	if (ch == DP_STREAM_1)
		reg_off = DP_DP1_RG - DP_DP0_RG;

	dp_write(DP_DP0_RG + reg_off, rg);
}

static void dp_catalog_ctrl_mainlink_levels(struct dp_catalog_ctrl *ctrl,
		u8 lane_cnt)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 mainlink_levels, safe_to_exit_level = 14;

	catalog = dp_catalog_get_priv(ctrl);

	io_data   = catalog->io.dp_link;

	switch (lane_cnt) {
	case 1:
		safe_to_exit_level = 14;
		break;
	case 2:
		safe_to_exit_level = 8;
		break;
	case 4:
		safe_to_exit_level = 5;
		break;
	default:
		DP_DEBUG("setting the default safe_to_exit_level = %u\n",
				safe_to_exit_level);
		break;
	}

	mainlink_levels = dp_read(DP_MAINLINK_LEVELS);
	mainlink_levels &= 0xFE0;
	mainlink_levels |= safe_to_exit_level;

	DP_DEBUG("mainlink_level = 0x%x, safe_to_exit_level = 0x%x\n",
			mainlink_levels, safe_to_exit_level);

	dp_write(DP_MAINLINK_LEVELS, mainlink_levels);
}


/* panel related catalog functions */
static int dp_catalog_panel_timing_cfg(struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 offset = 0, reg;

	if (!panel) {
		DP_ERR("invalid input\n");
		goto end;
	}

	if (panel->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream_id:%d\n", panel->stream_id);
		goto end;
	}

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	if (panel->stream_id == DP_STREAM_1)
		offset = DP1_TOTAL_HOR_VER - DP_TOTAL_HOR_VER;

	dp_write(DP_TOTAL_HOR_VER + offset, panel->total);
	dp_write(DP_START_HOR_VER_FROM_SYNC + offset, panel->sync_start);
	dp_write(DP_HSYNC_VSYNC_WIDTH_POLARITY + offset, panel->width_blanking);
	dp_write(DP_ACTIVE_HOR_VER + offset, panel->dp_active);

	if (panel->stream_id == DP_STREAM_0)
		io_data = catalog->io.dp_p0;
	else
		io_data = catalog->io.dp_p1;

	reg = dp_read(MMSS_DP_INTF_CONFIG);

	if (panel->widebus_en)
		reg |= BIT(4);
	else
		reg &= ~BIT(4);

	dp_write(MMSS_DP_INTF_CONFIG, reg);
end:
	return 0;
}

static void dp_catalog_hpd_config_hpd(struct dp_catalog_hpd *hpd, bool en)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!hpd) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = dp_catalog_get_priv(hpd);
	io_data = catalog->io.dp_aux;

	if (en) {
		u32 reftimer = dp_read(DP_DP_HPD_REFTIMER);

		/* Arm only the UNPLUG and HPD_IRQ interrupts */
		dp_write(DP_DP_HPD_INT_ACK, 0xF);
		dp_write(DP_DP_HPD_INT_MASK, 0xA);

		/* Enable REFTIMER to count 1ms */
		reftimer |= BIT(16);
		dp_write(DP_DP_HPD_REFTIMER, reftimer);

		 /* Connect_time is 250us & disconnect_time is 2ms */
		dp_write(DP_DP_HPD_EVENT_TIME_0, 0x3E800FA);
		dp_write(DP_DP_HPD_EVENT_TIME_1, 0x1F407D0);

		/* Enable HPD */
		dp_write(DP_DP_HPD_CTRL, 0x1);

	} else {
		/* Disable HPD */
		dp_write(DP_DP_HPD_CTRL, 0x0);
	}
}

static u32 dp_catalog_hpd_get_interrupt(struct dp_catalog_hpd *hpd)
{
	u32 isr = 0;
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;

	if (!hpd) {
		DP_ERR("invalid input\n");
		return isr;
	}

	catalog = dp_catalog_get_priv(hpd);

	io_data = catalog->io.dp_aux;
	isr = dp_read(DP_DP_HPD_INT_STATUS);
	dp_write(DP_DP_HPD_INT_ACK, (isr & 0xf));

	return isr;
}

static void dp_catalog_audio_init(struct dp_catalog_audio *audio)
{
	struct dp_catalog_private *catalog;
	static u32 sdp_map[][DP_AUDIO_SDP_HEADER_MAX] = {
		{
			MMSS_DP_AUDIO_STREAM_0,
			MMSS_DP_AUDIO_STREAM_1,
			MMSS_DP_AUDIO_STREAM_1,
		},
		{
			MMSS_DP_AUDIO_TIMESTAMP_0,
			MMSS_DP_AUDIO_TIMESTAMP_1,
			MMSS_DP_AUDIO_TIMESTAMP_1,
		},
		{
			MMSS_DP_AUDIO_INFOFRAME_0,
			MMSS_DP_AUDIO_INFOFRAME_1,
			MMSS_DP_AUDIO_INFOFRAME_1,
		},
		{
			MMSS_DP_AUDIO_COPYMANAGEMENT_0,
			MMSS_DP_AUDIO_COPYMANAGEMENT_1,
			MMSS_DP_AUDIO_COPYMANAGEMENT_1,
		},
		{
			MMSS_DP_AUDIO_ISRC_0,
			MMSS_DP_AUDIO_ISRC_1,
			MMSS_DP_AUDIO_ISRC_1,
		},
	};

	if (!audio)
		return;

	catalog = dp_catalog_get_priv(audio);

	catalog->audio_map = sdp_map;
}

static void dp_catalog_audio_config_sdp(struct dp_catalog_audio *audio)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 sdp_cfg = 0, sdp_cfg_off = 0;
	u32 sdp_cfg2 = 0, sdp_cfg2_off = 0;

	if (!audio)
		return;

	if (audio->stream_id >= DP_STREAM_MAX) {
		DP_ERR("invalid stream id:%d\n", audio->stream_id);
		return;
	}

	if (audio->stream_id == DP_STREAM_1) {
		sdp_cfg_off = MMSS_DP1_SDP_CFG - MMSS_DP_SDP_CFG;
		sdp_cfg2_off = MMSS_DP1_SDP_CFG2 - MMSS_DP_SDP_CFG2;
	}

	catalog = dp_catalog_get_priv(audio);
	io_data = catalog->io.dp_link;

	sdp_cfg = dp_read(MMSS_DP_SDP_CFG + sdp_cfg_off);

	/* AUDIO_TIMESTAMP_SDP_EN */
	sdp_cfg |= BIT(1);
	/* AUDIO_STREAM_SDP_EN */
	sdp_cfg |= BIT(2);
	/* AUDIO_COPY_MANAGEMENT_SDP_EN */
	sdp_cfg |= BIT(5);
	/* AUDIO_ISRC_SDP_EN  */
	sdp_cfg |= BIT(6);
	/* AUDIO_INFOFRAME_SDP_EN  */
	sdp_cfg |= BIT(20);

	DP_DEBUG("sdp_cfg = 0x%x\n", sdp_cfg);
	dp_write(MMSS_DP_SDP_CFG + sdp_cfg_off, sdp_cfg);

	sdp_cfg2 = dp_read(MMSS_DP_SDP_CFG2 + sdp_cfg_off);
	/* IFRM_REGSRC -> Do not use reg values */
	sdp_cfg2 &= ~BIT(0);
	/* AUDIO_STREAM_HB3_REGSRC-> Do not use reg values */
	sdp_cfg2 &= ~BIT(1);

	DP_DEBUG("sdp_cfg2 = 0x%x\n", sdp_cfg2);
	dp_write(MMSS_DP_SDP_CFG2 + sdp_cfg_off, sdp_cfg2);
}

static void dp_catalog_audio_get_header(struct dp_catalog_audio *audio)
{
	struct dp_catalog_private *catalog;
	u32 (*sdp_map)[DP_AUDIO_SDP_HEADER_MAX];
	struct dp_io_data *io_data;
	enum dp_catalog_audio_sdp_type sdp;
	enum dp_catalog_audio_header_type header;

	if (!audio)
		return;

	catalog = dp_catalog_get_priv(audio);

	io_data    = catalog->io.dp_link;
	sdp_map = catalog->audio_map;
	sdp     = audio->sdp_type;
	header  = audio->sdp_header;

	audio->data = dp_read(sdp_map[sdp][header]);
}

static void dp_catalog_audio_set_header(struct dp_catalog_audio *audio)
{
	struct dp_catalog_private *catalog;
	u32 (*sdp_map)[DP_AUDIO_SDP_HEADER_MAX];
	struct dp_io_data *io_data;
	enum dp_catalog_audio_sdp_type sdp;
	enum dp_catalog_audio_header_type header;
	u32 data;

	if (!audio)
		return;

	catalog = dp_catalog_get_priv(audio);

	io_data    = catalog->io.dp_link;
	sdp_map = catalog->audio_map;
	sdp     = audio->sdp_type;
	header  = audio->sdp_header;
	data    = audio->data;

	dp_write(sdp_map[sdp][header], data);
}

static void dp_catalog_audio_config_acr(struct dp_catalog_audio *audio)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 acr_ctrl, select;

	catalog = dp_catalog_get_priv(audio);

	select = audio->data;
	io_data   = catalog->io.dp_link;

	acr_ctrl = select << 4 | BIT(31) | BIT(8) | BIT(14);

	DP_DEBUG("select = 0x%x, acr_ctrl = 0x%x\n", select, acr_ctrl);

	dp_write(MMSS_DP_AUDIO_ACR_CTRL, acr_ctrl);
}

static void dp_catalog_audio_enable(struct dp_catalog_audio *audio)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	bool enable;
	u32 audio_ctrl;

	catalog = dp_catalog_get_priv(audio);

	io_data = catalog->io.dp_link;
	enable = !!audio->data;

	audio_ctrl = dp_read(MMSS_DP_AUDIO_CFG);

	if (enable)
		audio_ctrl |= BIT(0);
	else
		audio_ctrl &= ~BIT(0);

	DP_DEBUG("dp_audio_cfg = 0x%x\n", audio_ctrl);
	dp_write(MMSS_DP_AUDIO_CFG, audio_ctrl);

	/* make sure audio engine is disabled */
	wmb();
}

static void dp_catalog_config_spd_header(struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 value, new_value, offset = 0;
	u8 parity_byte;

	if (!panel || panel->stream_id >= DP_STREAM_MAX)
		return;

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	if (panel->stream_id == DP_STREAM_1)
		offset = MMSS_DP1_GENERIC0_0 - MMSS_DP_GENERIC0_0;

	/* Config header and parity byte 1 */
	value = dp_read(MMSS_DP_GENERIC1_0 + offset);

	new_value = 0x83;
	parity_byte = dp_header_get_parity(new_value);
	value |= ((new_value << HEADER_BYTE_1_BIT)
			| (parity_byte << PARITY_BYTE_1_BIT));
	DP_DEBUG("Header Byte 1: value = 0x%x, parity_byte = 0x%x\n",
			value, parity_byte);
	dp_write(MMSS_DP_GENERIC1_0 + offset, value);

	/* Config header and parity byte 2 */
	value = dp_read(MMSS_DP_GENERIC1_1 + offset);

	new_value = 0x1b;
	parity_byte = dp_header_get_parity(new_value);
	value |= ((new_value << HEADER_BYTE_2_BIT)
			| (parity_byte << PARITY_BYTE_2_BIT));
	DP_DEBUG("Header Byte 2: value = 0x%x, parity_byte = 0x%x\n",
			value, parity_byte);
	dp_write(MMSS_DP_GENERIC1_1 + offset, value);

	/* Config header and parity byte 3 */
	value = dp_read(MMSS_DP_GENERIC1_1 + offset);

	new_value = (0x0 | (0x12 << 2));
	parity_byte = dp_header_get_parity(new_value);
	value |= ((new_value << HEADER_BYTE_3_BIT)
			| (parity_byte << PARITY_BYTE_3_BIT));
	DP_DEBUG("Header Byte 3: value = 0x%x, parity_byte = 0x%x\n",
			new_value, parity_byte);
	dp_write(MMSS_DP_GENERIC1_1 + offset, value);
}

static void dp_catalog_panel_config_spd(struct dp_catalog_panel *panel)
{
	struct dp_catalog_private *catalog;
	struct dp_io_data *io_data;
	u32 spd_cfg = 0, spd_cfg2 = 0;
	u8 *vendor = NULL, *product = NULL;
	u32 offset = 0;
	u32 sdp_cfg_off = 0;
	u32 sdp_cfg2_off = 0;

	/*
	 * Source Device Information
	 * 00h unknown
	 * 01h Digital STB
	 * 02h DVD
	 * 03h D-VHS
	 * 04h HDD Video
	 * 05h DVC
	 * 06h DSC
	 * 07h Video CD
	 * 08h Game
	 * 09h PC general
	 * 0ah Bluray-Disc
	 * 0bh Super Audio CD
	 * 0ch HD DVD
	 * 0dh PMP
	 * 0eh-ffh reserved
	 */
	u32 device_type = 0;

	if (!panel || panel->stream_id >= DP_STREAM_MAX)
		return;

	catalog = dp_catalog_get_priv(panel);
	io_data = catalog->io.dp_link;

	if (panel->stream_id == DP_STREAM_1)
		offset = MMSS_DP1_GENERIC0_0 - MMSS_DP_GENERIC0_0;

	dp_catalog_config_spd_header(panel);

	vendor = panel->spd_vendor_name;
	product = panel->spd_product_description;

	dp_write(MMSS_DP_GENERIC1_2 + offset,
			((vendor[0] & 0x7f) |
			((vendor[1] & 0x7f) << 8) |
			((vendor[2] & 0x7f) << 16) |
			((vendor[3] & 0x7f) << 24)));
	dp_write(MMSS_DP_GENERIC1_3 + offset,
			((vendor[4] & 0x7f) |
			((vendor[5] & 0x7f) << 8) |
			((vendor[6] & 0x7f) << 16) |
			((vendor[7] & 0x7f) << 24)));
	dp_write(MMSS_DP_GENERIC1_4 + offset,
			((product[0] & 0x7f) |
			((product[1] & 0x7f) << 8) |
			((product[2] & 0x7f) << 16) |
			((product[3] & 0x7f) << 24)));
	dp_write(MMSS_DP_GENERIC1_5 + offset,
			((product[4] & 0x7f) |
			((product[5] & 0x7f) << 8) |
			((product[6] & 0x7f) << 16) |
			((product[7] & 0x7f) << 24)));
	dp_write(MMSS_DP_GENERIC1_6 + offset,
			((product[8] & 0x7f) |
			((product[9] & 0x7f) << 8) |
			((product[10] & 0x7f) << 16) |
			((product[11] & 0x7f) << 24)));
	dp_write(MMSS_DP_GENERIC1_7 + offset,
			((product[12] & 0x7f) |
			((product[13] & 0x7f) << 8) |
			((product[14] & 0x7f) << 16) |
			((product[15] & 0x7f) << 24)));
	dp_write(MMSS_DP_GENERIC1_8 + offset, device_type);
	dp_write(MMSS_DP_GENERIC1_9 + offset, 0x00);

	if (panel->stream_id == DP_STREAM_1) {
		sdp_cfg_off = MMSS_DP1_SDP_CFG - MMSS_DP_SDP_CFG;
		sdp_cfg2_off = MMSS_DP1_SDP_CFG2 - MMSS_DP_SDP_CFG2;
	}

	spd_cfg = dp_read(MMSS_DP_SDP_CFG + sdp_cfg_off);
	/* GENERIC1_SDP for SPD Infoframe */
	spd_cfg |= BIT(18);
	dp_write(MMSS_DP_SDP_CFG + sdp_cfg_off, spd_cfg);

	spd_cfg2 = dp_read(MMSS_DP_SDP_CFG2 + sdp_cfg2_off);
	/* 28 data bytes for SPD Infoframe with GENERIC1 set */
	spd_cfg2 |= BIT(17);
	dp_write(MMSS_DP_SDP_CFG2 + sdp_cfg2_off, spd_cfg2);

	dp_catalog_panel_sdp_update(panel);
}

static void dp_catalog_get_io_buf(struct dp_catalog_private *catalog)
{
	struct dp_parser *parser = catalog->parser;

	dp_catalog_fill_io_buf(dp_ahb);
	dp_catalog_fill_io_buf(dp_aux);
	dp_catalog_fill_io_buf(dp_link);
	dp_catalog_fill_io_buf(dp_p0);
	dp_catalog_fill_io_buf(dp_phy);
	dp_catalog_fill_io_buf(dp_ln_tx0);
	dp_catalog_fill_io_buf(dp_ln_tx1);
	dp_catalog_fill_io_buf(dp_pll);
	dp_catalog_fill_io_buf(usb3_dp_com);
	dp_catalog_fill_io_buf(dp_mmss_cc);
	dp_catalog_fill_io_buf(hdcp_physical);
	dp_catalog_fill_io_buf(dp_p1);
	dp_catalog_fill_io_buf(dp_tcsr);
}

static void dp_catalog_get_io(struct dp_catalog_private *catalog)
{
	struct dp_parser *parser = catalog->parser;

	dp_catalog_fill_io(dp_ahb);
	dp_catalog_fill_io(dp_aux);
	dp_catalog_fill_io(dp_link);
	dp_catalog_fill_io(dp_p0);
	dp_catalog_fill_io(dp_phy);
	dp_catalog_fill_io(dp_ln_tx0);
	dp_catalog_fill_io(dp_ln_tx1);
	dp_catalog_fill_io(dp_pll);
	dp_catalog_fill_io(usb3_dp_com);
	dp_catalog_fill_io(dp_mmss_cc);
	dp_catalog_fill_io(hdcp_physical);
	dp_catalog_fill_io(dp_p1);
	dp_catalog_fill_io(dp_tcsr);
}

static void dp_catalog_set_exe_mode(struct dp_catalog *dp_catalog, char *mode)
{
	struct dp_catalog_private *catalog;

	if (!dp_catalog) {
		DP_ERR("invalid input\n");
		return;
	}

	catalog = container_of(dp_catalog, struct dp_catalog_private,
		dp_catalog);

	strlcpy(catalog->exe_mode, mode, sizeof(catalog->exe_mode));

	if (!strcmp(catalog->exe_mode, "hw"))
		catalog->parser->clear_io_buf(catalog->parser);
	else
		dp_catalog_get_io_buf(catalog);

	if (!strcmp(catalog->exe_mode, "hw") ||
		!strcmp(catalog->exe_mode, "all")) {
		catalog->read = dp_read_hw;
		catalog->write = dp_write_hw;

		dp_catalog->sub->read = dp_read_sub_hw;
		dp_catalog->sub->write = dp_write_sub_hw;
	} else {
		catalog->read = dp_read_sw;
		catalog->write = dp_write_sw;

		dp_catalog->sub->read = dp_read_sub_sw;
		dp_catalog->sub->write = dp_write_sub_sw;
	}
}

static int dp_catalog_init(struct device *dev, struct dp_catalog *dp_catalog,
			struct dp_parser *parser)
{
	int rc = 0;
	struct dp_catalog_private *catalog = container_of(dp_catalog,
				struct dp_catalog_private, dp_catalog);

	switch (parser->hw_cfg.phy_version) {
	case DP_PHY_VERSION_4_2_0:
		dp_catalog->sub = dp_catalog_get_v420(dev, dp_catalog,
					&catalog->io);
		break;
	case DP_PHY_VERSION_2_0_0:
		dp_catalog->sub = dp_catalog_get_v200(dev, dp_catalog,
					&catalog->io);
		break;
	default:
		goto end;
	}

	if (IS_ERR(dp_catalog->sub)) {
		rc = PTR_ERR(dp_catalog->sub);
		dp_catalog->sub = NULL;
	} else {
		dp_catalog->sub->read = dp_read_sub_hw;
		dp_catalog->sub->write = dp_write_sub_hw;
	}
end:
	return rc;
}

void dp_catalog_put(struct dp_catalog *dp_catalog)
{
	struct dp_catalog_private *catalog;

	if (!dp_catalog)
		return;

	catalog = container_of(dp_catalog, struct dp_catalog_private,
				dp_catalog);

	if (dp_catalog->sub && dp_catalog->sub->put)
		dp_catalog->sub->put(dp_catalog);

	catalog->parser->clear_io_buf(catalog->parser);
	devm_kfree(catalog->dev, catalog);
}

struct dp_catalog *dp_catalog_get(struct device *dev, struct dp_parser *parser)
{
	int rc = 0;
	struct dp_catalog *dp_catalog;
	struct dp_catalog_private *catalog;
	struct dp_catalog_aux aux = {
		.read_data     = dp_catalog_aux_read_data,
		.write_data    = dp_catalog_aux_write_data,
		.write_trans   = dp_catalog_aux_write_trans,
		.clear_trans   = dp_catalog_aux_clear_trans,
		.reset         = dp_catalog_aux_reset,
		.update_aux_cfg = dp_catalog_aux_update_cfg,
		.enable        = dp_catalog_aux_enable,
		.setup         = dp_catalog_aux_setup,
		.get_irq       = dp_catalog_aux_get_irq,
		.clear_hw_interrupts = dp_catalog_aux_clear_hw_interrupts,
	};
	struct dp_catalog_ctrl ctrl = {
		.state_ctrl     = dp_catalog_ctrl_state_ctrl,
		.config_ctrl    = dp_catalog_ctrl_config_ctrl,
		.lane_mapping   = dp_catalog_ctrl_lane_mapping,
		.lane_pnswap    = dp_catalog_ctrl_lane_pnswap,
		.mainlink_ctrl  = dp_catalog_ctrl_mainlink_ctrl,
		.set_pattern    = dp_catalog_ctrl_set_pattern,
		.reset          = dp_catalog_ctrl_reset,
		.usb_reset      = dp_catalog_ctrl_usb_reset,
		.mainlink_ready = dp_catalog_ctrl_mainlink_ready,
		.enable_irq     = dp_catalog_ctrl_enable_irq,
		.phy_reset      = dp_catalog_ctrl_phy_reset,
		.phy_lane_cfg   = dp_catalog_ctrl_phy_lane_cfg,
		.update_vx_px   = dp_catalog_ctrl_update_vx_px,
		.get_interrupt  = dp_catalog_ctrl_get_interrupt,
		.read_hdcp_status     = dp_catalog_ctrl_read_hdcp_status,
		.send_phy_pattern    = dp_catalog_ctrl_send_phy_pattern,
		.read_phy_pattern = dp_catalog_ctrl_read_phy_pattern,
		.mst_config = dp_catalog_ctrl_mst_config,
		.trigger_act = dp_catalog_ctrl_trigger_act,
		.read_act_complete_sts = dp_catalog_ctrl_read_act_complete_sts,
		.channel_alloc = dp_catalog_ctrl_channel_alloc,
		.update_rg = dp_catalog_ctrl_update_rg,
		.channel_dealloc = dp_catalog_ctrl_channel_dealloc,
		.fec_config = dp_catalog_ctrl_fec_config,
		.mainlink_levels = dp_catalog_ctrl_mainlink_levels,
		.late_phy_init = dp_catalog_ctrl_late_phy_init,
	};
	struct dp_catalog_hpd hpd = {
		.config_hpd	= dp_catalog_hpd_config_hpd,
		.get_interrupt	= dp_catalog_hpd_get_interrupt,
	};
	struct dp_catalog_audio audio = {
		.init       = dp_catalog_audio_init,
		.config_acr = dp_catalog_audio_config_acr,
		.enable     = dp_catalog_audio_enable,
		.config_sdp = dp_catalog_audio_config_sdp,
		.set_header = dp_catalog_audio_set_header,
		.get_header = dp_catalog_audio_get_header,
	};
	struct dp_catalog_panel panel = {
		.timing_cfg = dp_catalog_panel_timing_cfg,
		.config_hdr = dp_catalog_panel_config_hdr,
		.config_sdp = dp_catalog_panel_config_sdp,
		.tpg_config = dp_catalog_panel_tpg_cfg,
		.config_spd = dp_catalog_panel_config_spd,
		.config_misc = dp_catalog_panel_config_misc,
		.set_colorspace = dp_catalog_panel_set_colorspace,
		.config_msa = dp_catalog_panel_config_msa,
		.update_transfer_unit = dp_catalog_panel_update_transfer_unit,
		.config_ctrl = dp_catalog_panel_config_ctrl,
		.config_dto = dp_catalog_panel_config_dto,
		.dsc_cfg = dp_catalog_panel_dsc_cfg,
		.pps_flush = dp_catalog_panel_pps_flush,
		.dhdr_flush = dp_catalog_panel_dhdr_flush,
		.dhdr_busy = dp_catalog_panel_dhdr_busy,
	};

	if (!dev || !parser) {
		DP_ERR("invalid input\n");
		rc = -EINVAL;
		goto error;
	}

	catalog  = devm_kzalloc(dev, sizeof(*catalog), GFP_KERNEL);
	if (!catalog) {
		rc = -ENOMEM;
		goto error;
	}

	catalog->dev = dev;
	catalog->parser = parser;

	catalog->read = dp_read_hw;
	catalog->write = dp_write_hw;

	dp_catalog_get_io(catalog);

	strlcpy(catalog->exe_mode, "hw", sizeof(catalog->exe_mode));

	dp_catalog = &catalog->dp_catalog;

	dp_catalog->aux   = aux;
	dp_catalog->ctrl  = ctrl;
	dp_catalog->hpd   = hpd;
	dp_catalog->audio = audio;
	dp_catalog->panel = panel;

	rc = dp_catalog_init(dev, dp_catalog, parser);
	if (rc) {
		dp_catalog_put(dp_catalog);
		goto error;
	}

	dp_catalog->set_exe_mode = dp_catalog_set_exe_mode;
	dp_catalog->get_reg_dump = dp_catalog_reg_dump;

	return dp_catalog;
error:
	return ERR_PTR(rc);
}
