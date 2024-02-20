/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2014-2015, 2021, The Linux Foundation. All rights reserved.
 */

#ifndef __QPIC_DISPLAY_PANEL_H__
#define __QPIC_DISPLAY_PANEL_H__

#include <linux/delay.h>

/**
 * Macros for coding MIPI commands
 */
#define INV_SIZE             0xFFFF
/* Size of argument to MIPI command is variable */
#define OP_SIZE_PAIR(op, size)    ((op<<16) | size)
/* MIPI {command, argument size} tuple */
#define LCDC_EXTRACT_OP_SIZE(op_identifier)    ((op_identifier&0xFFFF))
/* extract size from command identifier */
#define LCDC_EXTRACT_OP_CMD(op_identifier)    (((op_identifier>>16)&0xFFFF))
/* extract command id from command identifier */

/* MIPI standard efinitions */
#define LCDC_ADDRESS_MODE_ORDER_BOTTOM_TO_TOP                0x80
#define LCDC_ADDRESS_MODE_ORDER_RIGHT_TO_LEFT                0x40
#define LCDC_ADDRESS_MODE_ORDER_REVERSE                      0x20
#define LCDC_ADDRESS_MODE_ORDER_REFRESH_BOTTOM_TO_TOP        0x10
#define LCDC_ADDRESS_MODE_ORDER_BGER_RGB                     0x08
#define LCDC_ADDRESS_MODE_ORDER_REFERESH_RIGHT_TO_LEFT       0x04
#define LCDC_ADDRESS_MODE_FLIP_HORIZONTAL                    0x02
#define LCDC_ADDRESS_MODE_FLIP_VERTICAL                      0x01

#define LCDC_PIXEL_FORMAT_3_BITS_PER_PIXEL    0x1
#define LCDC_PIXEL_FORMAT_8_BITS_PER_PIXEL    0x2
#define LCDC_PIXEL_FORMAT_12_BITS_PER_PIXEL   0x3
#define LCDC_PIXEL_FORMAT_16_BITS_PER_PIXEL   0x5
#define LCDC_PIXEL_FORMAT_18_BITS_PER_PIXEL   0x6
#define LCDC_PIXEL_FORMAT_24_BITS_PER_PIXEL   0x7

#define LCDC_CREATE_PIXEL_FORMAT(dpi_format, dbi_format) \
	(dpi_format | (dpi_format<<4))

#define POWER_MODE_IDLE_ON       0x40
#define POWER_MODE_PARTIAL_ON    0x20
#define POWER_MODE_SLEEP_ON      0x10
#define POWER_MODE_NORMAL_ON     0x08
#define POWER_MODE_DISPLAY_ON    0x04

#define LCDC_DISPLAY_MODE_SCROLLING_ON       0x80
#define LCDC_DISPLAY_MODE_INVERSION_ON       0x20
#define LCDC_DISPLAY_MODE_GAMMA_MASK         0x07

/**
 * LDCc MIPI Type B supported commands
 */
#define	OP_ENTER_IDLE_MODE      0x39
#define	OP_ENTER_INVERT_MODE    0x21
#define	OP_ENTER_NORMAL_MODE    0x13
#define	OP_ENTER_PARTIAL_MODE   0x12
#define	OP_ENTER_SLEEP_MODE     0x10
#define	OP_EXIT_INVERT_MODE     0x20
#define	OP_EXIT_SLEEP_MODE      0x11
#define	OP_EXIT_IDLE_MODE       0x38
#define	OP_GET_ADDRESS_MODE     0x0B /* size 1 */
#define	OP_GET_BLUE_CHANNEL     0x08 /* size 1 */
#define	OP_GET_DIAGNOSTIC       0x0F /* size 2 */
#define	OP_GET_DISPLAY_MODE     0x0D /* size 1 */
#define	OP_GET_GREEN_CHANNEL    0x07 /* size 1 */
#define	OP_GET_PIXEL_FORMAT     0x0C /* size 1 */
#define	OP_GET_POWER_MODE       0x0A /* size 1 */
#define	OP_GET_RED_CHANNEL      0x06 /* size 1 */
#define	OP_GET_SCANLINE         0x45 /* size 1 */
#define	OP_GET_SIGNAL_MODE      0x0E /* size 1 */
#define	OP_NOP                  0x00
#define	OP_READ_DDB_CONTINUE    0xA8 /* size not fixed */
#define	OP_READ_DDB_START       0xA1 /* size not fixed */
#define	OP_READ_MEMORY_CONTINUE 0x3E /* size not fixed */
#define	OP_READ_MEMORY_START    0x2E /* size not fixed */
#define	OP_SET_ADDRESS_MODE     0x36 /* size 1 */
#define	OP_SET_COLUMN_ADDRESS   0x2A /* size 4 */
#define	OP_SET_DISPLAY_OFF      0x28
#define	OP_SET_DISPLAY_ON       0x29
#define	OP_SET_GAMMA_CURVE      0x26 /* size 1 */
#define	OP_SET_PAGE_ADDRESS     0x2B /* size 4 */
#define	OP_SET_PARTIAL_COLUMNS  0x31 /* size 4 */
#define	OP_SET_PARTIAL_ROWS     0x30 /* size 4 */
#define	OP_SET_PIXEL_FORMAT     0x3A /* size 1 */
#define	OP_SOFT_RESET           0x01
#define	OP_WRITE_MEMORY_CONTINUE  0x3C /* size not fixed */
#define	OP_WRITE_MEMORY_START   0x2C /* size not fixed */
#endif
