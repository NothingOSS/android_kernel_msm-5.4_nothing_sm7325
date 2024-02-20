/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 */

#ifndef __AQT1000_IRQ_H_
#define __AQT1000_IRQ_H_

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>

enum {
	/* INTR_CTRL_INT_MASK_2 */
	AQT1000_IRQ_MBHC_BUTTON_RELEASE_DET = 0,
	AQT1000_IRQ_MBHC_BUTTON_PRESS_DET,
	AQT1000_IRQ_MBHC_ELECT_INS_REM_DET,
	AQT1000_IRQ_MBHC_ELECT_INS_REM_LEG_DET,
	AQT1000_IRQ_MBHC_SW_DET,
	AQT1000_IRQ_HPH_PA_OCPL_FAULT,
	AQT1000_IRQ_HPH_PA_OCPR_FAULT,
	AQT1000_IRQ_HPH_PA_CNPL_COMPLETE,

	/* INTR_CTRL_INT_MASK_3 */
	AQT1000_IRQ_HPH_PA_CNPR_COMPLETE,
	AQT1000_CDC_HPHL_SURGE,
	AQT1000_CDC_HPHR_SURGE,
	AQT1000_NUM_IRQS,
};

int aqt_request_irq(struct aqt1000 *aqt, int irq, const char *name,
			irq_handler_t handler, void *data);
void aqt_free_irq(struct aqt1000 *aqt, int irq, void *data);
int aqt_irq_init(struct aqt1000 *aqt);
int aqt_irq_exit(struct aqt1000 *aqt);
void aqt_enable_irq(struct aqt1000 *aqt, int irq);
void aqt_disable_irq(struct aqt1000 *aqt, int irq);

#endif /* __AQT1000_IRQ_H_ */
