// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2020, The Linux Foundation. All rights reserved.
 */
#include <linux/iopoll.h>

#include "sde_hwio.h"
#include "sde_hw_catalog.h"
#include "sde_hw_intf.h"
#include "sde_dbg.h"

#define INTF_TIMING_ENGINE_EN           0x000
#define INTF_CONFIG                     0x004
#define INTF_HSYNC_CTL                  0x008
#define INTF_VSYNC_PERIOD_F0            0x00C
#define INTF_VSYNC_PERIOD_F1            0x010
#define INTF_VSYNC_PULSE_WIDTH_F0       0x014
#define INTF_VSYNC_PULSE_WIDTH_F1       0x018
#define INTF_DISPLAY_V_START_F0         0x01C
#define INTF_DISPLAY_V_START_F1         0x020
#define INTF_DISPLAY_V_END_F0           0x024
#define INTF_DISPLAY_V_END_F1           0x028
#define INTF_ACTIVE_V_START_F0          0x02C
#define INTF_ACTIVE_V_START_F1          0x030
#define INTF_ACTIVE_V_END_F0            0x034
#define INTF_ACTIVE_V_END_F1            0x038
#define INTF_DISPLAY_HCTL               0x03C
#define INTF_ACTIVE_HCTL                0x040
#define INTF_BORDER_COLOR               0x044
#define INTF_UNDERFLOW_COLOR            0x048
#define INTF_HSYNC_SKEW                 0x04C
#define INTF_POLARITY_CTL               0x050
#define INTF_TEST_CTL                   0x054
#define INTF_TP_COLOR0                  0x058
#define INTF_TP_COLOR1                  0x05C
#define INTF_CONFIG2                    0x060
#define INTF_DISPLAY_DATA_HCTL          0x064
#define INTF_ACTIVE_DATA_HCTL           0x068
#define INTF_FRAME_LINE_COUNT_EN        0x0A8
#define INTF_FRAME_COUNT                0x0AC
#define INTF_LINE_COUNT                 0x0B0

#define INTF_DEFLICKER_CONFIG           0x0F0
#define INTF_DEFLICKER_STRNG_COEFF      0x0F4
#define INTF_DEFLICKER_WEAK_COEFF       0x0F8

#define INTF_REG_SPLIT_LINK             0x080
#define INTF_DSI_CMD_MODE_TRIGGER_EN    0x084
#define INTF_PANEL_FORMAT               0x090
#define INTF_TPG_ENABLE                 0x100
#define INTF_TPG_MAIN_CONTROL           0x104
#define INTF_TPG_VIDEO_CONFIG           0x108
#define INTF_TPG_COMPONENT_LIMITS       0x10C
#define INTF_TPG_RECTANGLE              0x110
#define INTF_TPG_INITIAL_VALUE          0x114
#define INTF_TPG_BLK_WHITE_PATTERN_FRAMES   0x118
#define INTF_TPG_RGB_MAPPING            0x11C
#define INTF_PROG_FETCH_START           0x170
#define INTF_PROG_ROT_START             0x174

#define INTF_MISR_CTRL                  0x180
#define INTF_MISR_SIGNATURE             0x184

#define INTF_MUX                        0x25C
#define INTF_UNDERRUN_COUNT             0x268
#define INTF_STATUS                     0x26C
#define INTF_AVR_CONTROL                0x270
#define INTF_AVR_MODE                   0x274
#define INTF_AVR_TRIGGER                0x278
#define INTF_AVR_VTOTAL                 0x27C
#define INTF_TEAR_MDP_VSYNC_SEL         0x280
#define INTF_TEAR_TEAR_CHECK_EN         0x284
#define INTF_TEAR_SYNC_CONFIG_VSYNC     0x288
#define INTF_TEAR_SYNC_CONFIG_HEIGHT    0x28C
#define INTF_TEAR_SYNC_WRCOUNT          0x290
#define INTF_TEAR_VSYNC_INIT_VAL        0x294
#define INTF_TEAR_INT_COUNT_VAL         0x298
#define INTF_TEAR_SYNC_THRESH           0x29C
#define INTF_TEAR_START_POS             0x2A0
#define INTF_TEAR_RD_PTR_IRQ            0x2A4
#define INTF_TEAR_WR_PTR_IRQ            0x2A8
#define INTF_TEAR_OUT_LINE_COUNT        0x2AC
#define INTF_TEAR_LINE_COUNT            0x2B0
#define INTF_TEAR_AUTOREFRESH_CONFIG    0x2B4
#define INTF_TEAR_TEAR_DETECT_CTRL      0x2B8

static struct sde_intf_cfg *_intf_offset(enum sde_intf intf,
		struct sde_mdss_cfg *m,
		void __iomem *addr,
		struct sde_hw_blk_reg_map *b)
{
	int i;

	for (i = 0; i < m->intf_count; i++) {
		if ((intf == m->intf[i].id) &&
		(m->intf[i].type != INTF_NONE)) {
			b->base_off = addr;
			b->blk_off = m->intf[i].base;
			b->length = m->intf[i].len;
			b->hwversion = m->hwversion;
			b->log_mask = SDE_DBG_MASK_INTF;
			return &m->intf[i];
		}
	}

	return ERR_PTR(-EINVAL);
}

static void sde_hw_intf_avr_trigger(struct sde_hw_intf *ctx)
{
	struct sde_hw_blk_reg_map *c;

	if (!ctx)
		return;

	c = &ctx->hw;
	SDE_REG_WRITE(c, INTF_AVR_TRIGGER, 0x1);
	SDE_DEBUG("AVR Triggered\n");
}

static int sde_hw_intf_avr_setup(struct sde_hw_intf *ctx,
	const struct intf_timing_params *params,
	const struct intf_avr_params *avr_params)
{
	struct sde_hw_blk_reg_map *c;
	u32 hsync_period, vsync_period;
	u32 min_fps, default_fps, diff_fps;
	u32 vsync_period_slow;
	u32 avr_vtotal;
	u32 add_porches = 0;

	if (!ctx || !params || !avr_params) {
		SDE_ERROR("invalid input parameter(s)\n");
		return -EINVAL;
	}

	c = &ctx->hw;
	min_fps = avr_params->min_fps;
	default_fps = avr_params->default_fps;
	diff_fps = default_fps - min_fps;
	hsync_period = params->hsync_pulse_width +
			params->h_back_porch + params->width +
			params->h_front_porch;
	vsync_period = params->vsync_pulse_width +
			params->v_back_porch + params->height +
			params->v_front_porch;

	if (diff_fps)
		add_porches = mult_frac(vsync_period, diff_fps, min_fps);

	vsync_period_slow = vsync_period + add_porches;
	avr_vtotal = vsync_period_slow * hsync_period;

	SDE_REG_WRITE(c, INTF_AVR_VTOTAL, avr_vtotal);

	return 0;
}

static void sde_hw_intf_avr_ctrl(struct sde_hw_intf *ctx,
	const struct intf_avr_params *avr_params)
{
	struct sde_hw_blk_reg_map *c;
	u32 avr_mode = 0;
	u32 avr_ctrl = 0;

	if (!ctx || !avr_params)
		return;

	c = &ctx->hw;
	if (avr_params->avr_mode) {
		avr_ctrl = BIT(0);
		avr_mode =
		(avr_params->avr_mode == SDE_RM_QSYNC_ONE_SHOT_MODE) ?
			(BIT(0) | BIT(8)) : 0x0;
	}

	SDE_REG_WRITE(c, INTF_AVR_CONTROL, avr_ctrl);
	SDE_REG_WRITE(c, INTF_AVR_MODE, avr_mode);
}

static inline void _check_and_set_comp_bit(struct sde_hw_intf *ctx,
		bool dsc_4hs_merge, bool compression_en, u32 *intf_cfg2)
{
	if (((SDE_HW_MAJOR(ctx->mdss->hwversion) >=
				SDE_HW_MAJOR(SDE_HW_VER_700)) &&
				compression_en) ||
			(IS_SDE_MAJOR_SAME(ctx->mdss->hwversion,
				SDE_HW_VER_600) && dsc_4hs_merge))
		(*intf_cfg2) |= BIT(12);
}

static void sde_hw_intf_setup_timing_engine(struct sde_hw_intf *ctx,
		const struct intf_timing_params *p,
		const struct sde_format *fmt)
{
	struct sde_hw_blk_reg_map *c = &ctx->hw;
	u32 hsync_period, vsync_period;
	u32 display_v_start, display_v_end;
	u32 hsync_start_x, hsync_end_x;
	u32 hsync_data_start_x, hsync_data_end_x;
	u32 active_h_start, active_h_end;
	u32 active_v_start, active_v_end;
	u32 active_hctl, display_hctl, hsync_ctl;
	u32 polarity_ctl, den_polarity, hsync_polarity, vsync_polarity;
	u32 panel_format;
	u32 intf_cfg, intf_cfg2 = 0;
	u32 display_data_hctl = 0, active_data_hctl = 0;
	u32 data_width;
	bool dp_intf = false;

	/* read interface_cfg */
	intf_cfg = SDE_REG_READ(c, INTF_CONFIG);

	if (ctx->cap->type == INTF_EDP || ctx->cap->type == INTF_DP)
		dp_intf = true;

	hsync_period = p->hsync_pulse_width + p->h_back_porch + p->width +
			p->h_front_porch;
	vsync_period = p->vsync_pulse_width + p->v_back_porch + p->height +
			p->v_front_porch;

	display_v_start = ((p->vsync_pulse_width + p->v_back_porch) *
			hsync_period) + p->hsync_skew;
	display_v_end = ((vsync_period - p->v_front_porch) * hsync_period) +
			p->hsync_skew - 1;

	hsync_ctl = (hsync_period << 16) | p->hsync_pulse_width;

	hsync_start_x = p->h_back_porch + p->hsync_pulse_width;
	hsync_end_x = hsync_period - p->h_front_porch - 1;

	/*
	 * DATA_HCTL_EN controls data timing which can be different from
	 * video timing. It is recommended to enable it for all cases, except
	 * if compression is enabled in 1 pixel per clock mode
	 */
	if (!p->compression_en || p->wide_bus_en)
		intf_cfg2 |= BIT(4);

	if (p->wide_bus_en)
		intf_cfg2 |= BIT(0);

	/*
	 * If widebus is disabled:
	 * For uncompressed stream, the data is valid for the entire active
	 * window period.
	 * For compressed stream, data is valid for a shorter time period
	 * inside the active window depending on the compression ratio.
	 *
	 * If widebus is enabled:
	 * For uncompressed stream, data is valid for only half the active
	 * window, since the data rate is doubled in this mode.
	 * p->width holds the adjusted width for DP but unadjusted width for DSI
	 * For compressed stream, data validity window needs to be adjusted for
	 * compression ratio and then further halved.
	 */
	data_width = p->width;

	if (p->compression_en) {
		if (p->wide_bus_en)
			data_width = DIV_ROUND_UP(p->dce_bytes_per_line, 6);
		else
			data_width = DIV_ROUND_UP(p->dce_bytes_per_line, 3);

	} else if (!dp_intf && p->wide_bus_en) {
		data_width = p->width >> 1;
	} else {
		data_width = p->width;
	}

	hsync_data_start_x = hsync_start_x;
	hsync_data_end_x =  hsync_start_x + data_width - 1;

	display_hctl = (hsync_end_x << 16) | hsync_start_x;
	display_data_hctl = (hsync_data_end_x << 16) | hsync_data_start_x;

	if (dp_intf) {
		// DP timing adjustment
		display_v_start += p->hsync_pulse_width + p->h_back_porch;
		display_v_end   -= p->h_front_porch;
	}

	intf_cfg |= BIT(29);	/* ACTIVE_H_ENABLE */
	intf_cfg |= BIT(30);	/* ACTIVE_V_ENABLE */
	active_h_start = hsync_start_x;
	active_h_end = active_h_start + p->xres - 1;
	active_v_start = display_v_start;
	active_v_end = active_v_start + (p->yres * hsync_period) - 1;

	active_hctl = (active_h_end << 16) | active_h_start;

	if (dp_intf) {
		display_hctl = active_hctl;

		if (p->compression_en) {
			active_data_hctl = (hsync_start_x +
					 p->extra_dto_cycles) << 16;
			active_data_hctl += hsync_start_x;

			display_data_hctl = active_data_hctl;
		}
	}

	_check_and_set_comp_bit(ctx, p->dsc_4hs_merge, p->compression_en,
			&intf_cfg2);

	den_polarity = 0;
	if (ctx->cap->type == INTF_HDMI) {
		hsync_polarity = p->yres >= 720 ? 0 : 1;
		vsync_polarity = p->yres >= 720 ? 0 : 1;
	} else if (ctx->cap->type == INTF_DP) {
		hsync_polarity = p->hsync_polarity;
		vsync_polarity = p->vsync_polarity;
	} else {
		hsync_polarity = 0;
		vsync_polarity = 0;
	}
	polarity_ctl = (den_polarity << 2) | /*  DEN Polarity  */
		(vsync_polarity << 1) | /* VSYNC Polarity */
		(hsync_polarity << 0);  /* HSYNC Polarity */

	if (!SDE_FORMAT_IS_YUV(fmt))
		panel_format = (fmt->bits[C0_G_Y] |
				(fmt->bits[C1_B_Cb] << 2) |
				(fmt->bits[C2_R_Cr] << 4) |
				(0x21 << 8));
	else
		/* Interface treats all the pixel data in RGB888 format */
		panel_format = (COLOR_8BIT |
				(COLOR_8BIT << 2) |
				(COLOR_8BIT << 4) |
				(0x21 << 8));

	if (p->wide_bus_en)
		intf_cfg2 |= BIT(0);

	/* Synchronize timing engine enable to TE */
	if ((ctx->cap->features & BIT(SDE_INTF_TE_ALIGN_VSYNC))
			&& p->poms_align_vsync)
		intf_cfg2 |= BIT(16);

	if (ctx->cfg.split_link_en)
		SDE_REG_WRITE(c, INTF_REG_SPLIT_LINK, 0x3);

	SDE_REG_WRITE(c, INTF_HSYNC_CTL, hsync_ctl);
	SDE_REG_WRITE(c, INTF_VSYNC_PERIOD_F0, vsync_period * hsync_period);
	SDE_REG_WRITE(c, INTF_VSYNC_PULSE_WIDTH_F0,
			p->vsync_pulse_width * hsync_period);
	SDE_REG_WRITE(c, INTF_DISPLAY_HCTL, display_hctl);
	SDE_REG_WRITE(c, INTF_DISPLAY_V_START_F0, display_v_start);
	SDE_REG_WRITE(c, INTF_DISPLAY_V_END_F0, display_v_end);
	SDE_REG_WRITE(c, INTF_ACTIVE_HCTL,  active_hctl);
	SDE_REG_WRITE(c, INTF_ACTIVE_V_START_F0, active_v_start);
	SDE_REG_WRITE(c, INTF_ACTIVE_V_END_F0, active_v_end);
	SDE_REG_WRITE(c, INTF_BORDER_COLOR, p->border_clr);
	SDE_REG_WRITE(c, INTF_UNDERFLOW_COLOR, p->underflow_clr);
	SDE_REG_WRITE(c, INTF_HSYNC_SKEW, p->hsync_skew);
	SDE_REG_WRITE(c, INTF_POLARITY_CTL, polarity_ctl);
	SDE_REG_WRITE(c, INTF_FRAME_LINE_COUNT_EN, 0x3);
	SDE_REG_WRITE(c, INTF_CONFIG, intf_cfg);
	SDE_REG_WRITE(c, INTF_PANEL_FORMAT, panel_format);
	SDE_REG_WRITE(c, INTF_CONFIG2, intf_cfg2);
	SDE_REG_WRITE(c, INTF_DISPLAY_DATA_HCTL, display_data_hctl);
	SDE_REG_WRITE(c, INTF_ACTIVE_DATA_HCTL, active_data_hctl);
}

static void sde_hw_intf_enable_timing_engine(
		struct sde_hw_intf *intf,
		u8 enable)
{
	struct sde_hw_blk_reg_map *c = &intf->hw;
	/* Note: Display interface select is handled in top block hw layer */
	SDE_REG_WRITE(c, INTF_TIMING_ENGINE_EN, enable != 0);
}

static void sde_hw_intf_setup_prg_fetch(
		struct sde_hw_intf *intf,
		const struct intf_prog_fetch *fetch)
{
	struct sde_hw_blk_reg_map *c = &intf->hw;
	int fetch_enable;

	/*
	 * Fetch should always be outside the active lines. If the fetching
	 * is programmed within active region, hardware behavior is unknown.
	 */

	fetch_enable = SDE_REG_READ(c, INTF_CONFIG);
	if (fetch->enable) {
		fetch_enable |= BIT(31);
		SDE_REG_WRITE(c, INTF_PROG_FETCH_START,
				fetch->fetch_start);
	} else {
		fetch_enable &= ~BIT(31);
	}

	SDE_REG_WRITE(c, INTF_CONFIG, fetch_enable);
}

static void sde_hw_intf_bind_pingpong_blk(
		struct sde_hw_intf *intf,
		bool enable,
		const enum sde_pingpong pp)
{
	struct sde_hw_blk_reg_map *c;
	u32 mux_cfg;

	if (!intf)
		return;

	c = &intf->hw;

	mux_cfg = SDE_REG_READ(c, INTF_MUX);
	mux_cfg &= ~0xf;

	if (enable) {
		mux_cfg |= (pp - PINGPONG_0) & 0x7;
		if (intf->cfg.split_link_en)
			mux_cfg = 0x60000;
	} else {
		mux_cfg = 0xf000f;
	}

	SDE_REG_WRITE(c, INTF_MUX, mux_cfg);
}

static void sde_hw_intf_get_status(
		struct sde_hw_intf *intf,
		struct intf_status *s)
{
	struct sde_hw_blk_reg_map *c = &intf->hw;

	s->is_en = SDE_REG_READ(c, INTF_TIMING_ENGINE_EN);
	if (s->is_en) {
		s->frame_count = SDE_REG_READ(c, INTF_FRAME_COUNT);
		s->line_count = SDE_REG_READ(c, INTF_LINE_COUNT);
	} else {
		s->line_count = 0;
		s->frame_count = 0;
	}
}

static void sde_hw_intf_v1_get_status(
		struct sde_hw_intf *intf,
		struct intf_status *s)
{
	struct sde_hw_blk_reg_map *c = &intf->hw;

	s->is_en = SDE_REG_READ(c, INTF_STATUS) & BIT(0);
	if (s->is_en) {
		s->frame_count = SDE_REG_READ(c, INTF_FRAME_COUNT);
		s->line_count = SDE_REG_READ(c, INTF_LINE_COUNT);
	} else {
		s->line_count = 0;
		s->frame_count = 0;
	}
}
static void sde_hw_intf_setup_misr(struct sde_hw_intf *intf,
						bool enable, u32 frame_count)
{
	struct sde_hw_blk_reg_map *c = &intf->hw;
	u32 config = 0;

	SDE_REG_WRITE(c, INTF_MISR_CTRL, MISR_CTRL_STATUS_CLEAR);
	/* clear misr data */
	wmb();

	if (enable)
		config = (frame_count & MISR_FRAME_COUNT_MASK) |
				MISR_CTRL_ENABLE |
				INTF_MISR_CTRL_FREE_RUN_MASK |
				INTF_MISR_CTRL_INPUT_SEL_DATA;

	SDE_REG_WRITE(c, INTF_MISR_CTRL, config);
}

static int sde_hw_intf_collect_misr(struct sde_hw_intf *intf, bool nonblock,
		 u32 *misr_value)
{
	struct sde_hw_blk_reg_map *c = &intf->hw;
	u32 ctrl = 0;

	if (!misr_value)
		return -EINVAL;

	ctrl = SDE_REG_READ(c, INTF_MISR_CTRL);
	if (!nonblock) {
		if (ctrl & MISR_CTRL_ENABLE) {
			int rc;

			rc = readl_poll_timeout(c->base_off + c->blk_off +
					INTF_MISR_CTRL, ctrl,
					(ctrl & MISR_CTRL_STATUS) > 0, 500,
					84000);
			if (rc)
				return rc;
		} else {
			return -EINVAL;
		}
	}

	*misr_value =  SDE_REG_READ(c, INTF_MISR_SIGNATURE);
	return 0;
}

static u32 sde_hw_intf_get_line_count(struct sde_hw_intf *intf)
{
	struct sde_hw_blk_reg_map *c;

	if (!intf)
		return 0;

	c = &intf->hw;

	return SDE_REG_READ(c, INTF_LINE_COUNT);
}

static u32 sde_hw_intf_get_underrun_line_count(struct sde_hw_intf *intf)
{
	struct sde_hw_blk_reg_map *c;
	u32 hsync_period;

	if (!intf)
		return 0;

	c = &intf->hw;
	hsync_period = SDE_REG_READ(c, INTF_HSYNC_CTL);
	hsync_period = ((hsync_period & 0xffff0000) >> 16);

	return hsync_period ?
		SDE_REG_READ(c, INTF_UNDERRUN_COUNT) / hsync_period :
		0xebadebad;
}

static u32 sde_hw_intf_get_intr_status(struct sde_hw_intf *intf)
{
	if (!intf)
		return -EINVAL;

	return SDE_REG_READ(&intf->hw, INTF_INTR_STATUS);
}

static int sde_hw_intf_setup_te_config(struct sde_hw_intf *intf,
		struct sde_hw_tear_check *te)
{
	struct sde_hw_blk_reg_map *c;
	u32 cfg = 0;
	spinlock_t tearcheck_spinlock;

	if (!intf)
		return -EINVAL;

	spin_lock_init(&tearcheck_spinlock);
	c = &intf->hw;

	if (te->hw_vsync_mode)
		cfg |= BIT(20);

	cfg |= te->vsync_count;

	/*
	 * Local spinlock is acquired here to avoid pre-emption
	 * as below register programming should be completed in
	 * less than 2^16 vsync clk cycles.
	 */
	spin_lock(&tearcheck_spinlock);
	SDE_REG_WRITE(c, INTF_TEAR_SYNC_WRCOUNT,
			(te->start_pos + te->sync_threshold_start + 1));
	SDE_REG_WRITE(c, INTF_TEAR_SYNC_CONFIG_VSYNC, cfg);
	wmb(); /* disable vsync counter before updating single buffer registers */
	SDE_REG_WRITE(c, INTF_TEAR_SYNC_CONFIG_HEIGHT, te->sync_cfg_height);
	SDE_REG_WRITE(c, INTF_TEAR_VSYNC_INIT_VAL, te->vsync_init_val);
	SDE_REG_WRITE(c, INTF_TEAR_RD_PTR_IRQ, te->rd_ptr_irq);
	SDE_REG_WRITE(c, INTF_TEAR_WR_PTR_IRQ, te->wr_ptr_irq);
	SDE_REG_WRITE(c, INTF_TEAR_START_POS, te->start_pos);
	SDE_REG_WRITE(c, INTF_TEAR_SYNC_THRESH,
			((te->sync_threshold_continue << 16) |
			 te->sync_threshold_start));
	cfg |= BIT(19); /* VSYNC_COUNTER_EN */
	SDE_REG_WRITE(c, INTF_TEAR_SYNC_CONFIG_VSYNC, cfg);
	spin_unlock(&tearcheck_spinlock);

	return 0;
}

static int sde_hw_intf_setup_autorefresh_config(struct sde_hw_intf *intf,
		struct sde_hw_autorefresh *cfg)
{
	struct sde_hw_blk_reg_map *c;
	u32 refresh_cfg;

	if (!intf || !cfg)
		return -EINVAL;

	c = &intf->hw;

	refresh_cfg = SDE_REG_READ(c, INTF_TEAR_AUTOREFRESH_CONFIG);
	if (cfg->enable)
		refresh_cfg = BIT(31) | cfg->frame_count;
	else
		refresh_cfg &= ~BIT(31);

	SDE_REG_WRITE(c, INTF_TEAR_AUTOREFRESH_CONFIG, refresh_cfg);

	return 0;
}

static int sde_hw_intf_get_autorefresh_config(struct sde_hw_intf *intf,
		struct sde_hw_autorefresh *cfg)
{
	struct sde_hw_blk_reg_map *c;
	u32 val;

	if (!intf || !cfg)
		return -EINVAL;

	c = &intf->hw;
	val = SDE_REG_READ(c, INTF_TEAR_AUTOREFRESH_CONFIG);
	cfg->enable = (val & BIT(31)) >> 31;
	cfg->frame_count = val & 0xffff;

	return 0;
}

static int sde_hw_intf_poll_timeout_wr_ptr(struct sde_hw_intf *intf,
		u32 timeout_us)
{
	struct sde_hw_blk_reg_map *c;
	u32 val;
	int rc;

	if (!intf)
		return -EINVAL;

	c = &intf->hw;
	rc = readl_poll_timeout(c->base_off + c->blk_off + INTF_TEAR_LINE_COUNT,
			val, (val & 0xffff) >= 1, 10, timeout_us);

	return rc;
}

static int sde_hw_intf_enable_te(struct sde_hw_intf *intf, bool enable)
{
	struct sde_hw_blk_reg_map *c;

	if (!intf)
		return -EINVAL;

	c = &intf->hw;
	SDE_REG_WRITE(c, INTF_TEAR_TEAR_CHECK_EN, enable);
	return 0;
}

static void sde_hw_intf_update_te(struct sde_hw_intf *intf,
		struct sde_hw_tear_check *te)
{
	struct sde_hw_blk_reg_map *c;
	int cfg;

	if (!intf || !te)
		return;

	c = &intf->hw;
	cfg = SDE_REG_READ(c, INTF_TEAR_SYNC_THRESH);
	cfg &= ~0xFFFF;
	cfg |= te->sync_threshold_start;
	SDE_REG_WRITE(c, INTF_TEAR_SYNC_THRESH, cfg);
}

static int sde_hw_intf_connect_external_te(struct sde_hw_intf *intf,
		bool enable_external_te)
{
	struct sde_hw_blk_reg_map *c = &intf->hw;
	u32 cfg;
	int orig;

	if (!intf)
		return -EINVAL;

	c = &intf->hw;
	cfg = SDE_REG_READ(c, INTF_TEAR_SYNC_CONFIG_VSYNC);
	orig = (bool)(cfg & BIT(20));
	if (enable_external_te)
		cfg |= BIT(20);
	else
		cfg &= ~BIT(20);
	SDE_REG_WRITE(c, INTF_TEAR_SYNC_CONFIG_VSYNC, cfg);

	return orig;
}

static int sde_hw_intf_get_vsync_info(struct sde_hw_intf *intf,
		struct sde_hw_pp_vsync_info *info)
{
	struct sde_hw_blk_reg_map *c = &intf->hw;
	u32 val;

	if (!intf || !info)
		return -EINVAL;

	c = &intf->hw;

	val = SDE_REG_READ(c, INTF_TEAR_VSYNC_INIT_VAL);
	info->rd_ptr_init_val = val & 0xffff;

	val = SDE_REG_READ(c, INTF_TEAR_INT_COUNT_VAL);
	info->rd_ptr_frame_count = (val & 0xffff0000) >> 16;
	info->rd_ptr_line_count = val & 0xffff;

	val = SDE_REG_READ(c, INTF_TEAR_LINE_COUNT);
	info->wr_ptr_line_count = val & 0xffff;

	val = SDE_REG_READ(c, INTF_FRAME_COUNT);
	info->intf_frame_count = val;

	return 0;
}

static int sde_hw_intf_v1_check_and_reset_tearcheck(struct sde_hw_intf *intf,
		struct intf_tear_status *status)
{
	struct sde_hw_blk_reg_map *c = &intf->hw;
	u32 start_pos;

	if (!intf || !status)
		return -EINVAL;

	c = &intf->hw;

	status->read_count = SDE_REG_READ(c, INTF_TEAR_INT_COUNT_VAL);
	start_pos = SDE_REG_READ(c, INTF_TEAR_START_POS);
	status->write_count = SDE_REG_READ(c, INTF_TEAR_SYNC_WRCOUNT);
	status->write_count &= 0xffff0000;
	status->write_count |= start_pos;
	SDE_REG_WRITE(c, INTF_TEAR_SYNC_WRCOUNT, status->write_count);

	return 0;
}

static void sde_hw_intf_vsync_sel(struct sde_hw_intf *intf,
		u32 vsync_source)
{
	struct sde_hw_blk_reg_map *c;

	if (!intf)
		return;

	c = &intf->hw;

	SDE_REG_WRITE(c, INTF_TEAR_MDP_VSYNC_SEL, (vsync_source & 0xf));
}

static void sde_hw_intf_enable_compressed_input(struct sde_hw_intf *intf,
		bool compression_en, bool dsc_4hs_merge)
{
	struct sde_hw_blk_reg_map *c;
	u32 intf_cfg2;

	if (!intf)
		return;

	/*
	 * callers can either call this function to enable/disable the 64 bit
	 * compressed input or this configuration can be applied along
	 * with timing generation parameters
	 */

	c = &intf->hw;
	intf_cfg2 = SDE_REG_READ(c, INTF_CONFIG2);

	_check_and_set_comp_bit(intf, dsc_4hs_merge, compression_en,
			&intf_cfg2);

	SDE_REG_WRITE(c, INTF_CONFIG2, intf_cfg2);
}

static void sde_hw_intf_enable_wide_bus(struct sde_hw_intf *intf,
		bool enable)
{
	struct sde_hw_blk_reg_map *c;
	u32 intf_cfg2;

	if (!intf)
		return;

	c = &intf->hw;
	intf_cfg2 = SDE_REG_READ(c, INTF_CONFIG2);
	intf_cfg2 &= ~BIT(0);

	intf_cfg2 |= enable ? BIT(0) : 0;

	SDE_REG_WRITE(c, INTF_CONFIG2, intf_cfg2);
}

static void _setup_intf_ops(struct sde_hw_intf_ops *ops,
		unsigned long cap)
{
	ops->setup_timing_gen = sde_hw_intf_setup_timing_engine;
	ops->setup_prg_fetch  = sde_hw_intf_setup_prg_fetch;
	ops->enable_timing = sde_hw_intf_enable_timing_engine;
	ops->setup_misr = sde_hw_intf_setup_misr;
	ops->collect_misr = sde_hw_intf_collect_misr;
	ops->get_line_count = sde_hw_intf_get_line_count;
	ops->get_underrun_line_count = sde_hw_intf_get_underrun_line_count;
	ops->get_intr_status = sde_hw_intf_get_intr_status;
	ops->avr_setup = sde_hw_intf_avr_setup;
	ops->avr_trigger = sde_hw_intf_avr_trigger;
	ops->avr_ctrl = sde_hw_intf_avr_ctrl;
	ops->enable_compressed_input = sde_hw_intf_enable_compressed_input;
	ops->enable_wide_bus = sde_hw_intf_enable_wide_bus;

	if (cap & BIT(SDE_INTF_STATUS))
		ops->get_status = sde_hw_intf_v1_get_status;
	else
		ops->get_status = sde_hw_intf_get_status;

	if (cap & BIT(SDE_INTF_INPUT_CTRL))
		ops->bind_pingpong_blk = sde_hw_intf_bind_pingpong_blk;

	if (cap & BIT(SDE_INTF_TE)) {
		ops->setup_tearcheck = sde_hw_intf_setup_te_config;
		ops->enable_tearcheck = sde_hw_intf_enable_te;
		ops->update_tearcheck = sde_hw_intf_update_te;
		ops->connect_external_te = sde_hw_intf_connect_external_te;
		ops->get_vsync_info = sde_hw_intf_get_vsync_info;
		ops->setup_autorefresh = sde_hw_intf_setup_autorefresh_config;
		ops->get_autorefresh = sde_hw_intf_get_autorefresh_config;
		ops->poll_timeout_wr_ptr = sde_hw_intf_poll_timeout_wr_ptr;
		ops->vsync_sel = sde_hw_intf_vsync_sel;
		ops->check_and_reset_tearcheck =
			sde_hw_intf_v1_check_and_reset_tearcheck;
	}
}

static struct sde_hw_blk_ops sde_hw_ops = {
	.start = NULL,
	.stop = NULL,
};

struct sde_hw_intf *sde_hw_intf_init(enum sde_intf idx,
		void __iomem *addr,
		struct sde_mdss_cfg *m)
{
	struct sde_hw_intf *c;
	struct sde_intf_cfg *cfg;
	int rc;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	cfg = _intf_offset(idx, m, addr, &c->hw);
	if (IS_ERR_OR_NULL(cfg)) {
		kfree(c);
		pr_err("failed to create sde_hw_intf %d\n", idx);
		return ERR_PTR(-EINVAL);
	}

	/*
	 * Assign ops
	 */
	c->idx = idx;
	c->cap = cfg;
	c->mdss = m;
	_setup_intf_ops(&c->ops, c->cap->features);

	rc = sde_hw_blk_init(&c->base, SDE_HW_BLK_INTF, idx, &sde_hw_ops);
	if (rc) {
		SDE_ERROR("failed to init hw blk %d\n", rc);
		goto blk_init_error;
	}

	sde_dbg_reg_register_dump_range(SDE_DBG_NAME, cfg->name, c->hw.blk_off,
			c->hw.blk_off + c->hw.length, c->hw.xin_id);

	return c;

blk_init_error:
	kzfree(c);

	return ERR_PTR(rc);
}

void sde_hw_intf_destroy(struct sde_hw_intf *intf)
{
	if (intf)
		sde_hw_blk_destroy(&intf->base);
	kfree(intf);
}

