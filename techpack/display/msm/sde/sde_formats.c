// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__

#include <drm/drm_fourcc.h>
#include <media/mmm_color_fmt.h>

#include "sde_kms.h"
#include "sde_formats.h"

#define SDE_UBWC_META_MACRO_W_H		16
#define SDE_UBWC_META_BLOCK_SIZE	256
#define SDE_UBWC_PLANE_SIZE_ALIGNMENT	4096

#define SDE_TILE_HEIGHT_DEFAULT	1
#define SDE_TILE_HEIGHT_TILED	4
#define SDE_TILE_HEIGHT_UBWC	4
#define SDE_TILE_HEIGHT_NV12	8

#define SDE_MAX_IMG_WIDTH		0x3FFF
#define SDE_MAX_IMG_HEIGHT		0x3FFF

/**
 * SDE supported format packing, bpp, and other format
 * information.
 * SDE currently only supports interleaved RGB formats
 * UBWC support for a pixel format is indicated by the flag,
 * there is additional meta data plane for such formats
 */

#define INTERLEAVED_RGB_FMT(fmt, a, r, g, b, e0, e1, e2, e3, uc, alpha,   \
bp, flg, fm, np)                                                          \
{                                                                         \
	.base.pixel_format = DRM_FORMAT_ ## fmt,                          \
	.fetch_planes = SDE_PLANE_INTERLEAVED,                            \
	.alpha_enable = alpha,                                            \
	.element = { (e0), (e1), (e2), (e3) },                            \
	.bits = { g, b, r, a },                                           \
	.chroma_sample = SDE_CHROMA_RGB,                                  \
	.unpack_align_msb = 0,                                            \
	.unpack_tight = 1,                                                \
	.unpack_count = uc,                                               \
	.bpp = bp,                                                        \
	.fetch_mode = fm,                                                 \
	.flag = {(flg)},                                                  \
	.num_planes = np,                                                 \
	.tile_height = SDE_TILE_HEIGHT_DEFAULT                            \
}

#define INTERLEAVED_RGB_FMT_TILED(fmt, a, r, g, b, e0, e1, e2, e3, uc,    \
alpha, bp, flg, fm, np, th)                                               \
{                                                                         \
	.base.pixel_format = DRM_FORMAT_ ## fmt,                          \
	.fetch_planes = SDE_PLANE_INTERLEAVED,                            \
	.alpha_enable = alpha,                                            \
	.element = { (e0), (e1), (e2), (e3) },                            \
	.bits = { g, b, r, a },                                           \
	.chroma_sample = SDE_CHROMA_RGB,                                  \
	.unpack_align_msb = 0,                                            \
	.unpack_tight = 1,                                                \
	.unpack_count = uc,                                               \
	.bpp = bp,                                                        \
	.fetch_mode = fm,                                                 \
	.flag = {(flg)},                                                  \
	.num_planes = np,                                                 \
	.tile_height = th                                                 \
}


#define INTERLEAVED_YUV_FMT(fmt, a, r, g, b, e0, e1, e2, e3,              \
alpha, chroma, count, bp, flg, fm, np)                                    \
{                                                                         \
	.base.pixel_format = DRM_FORMAT_ ## fmt,                          \
	.fetch_planes = SDE_PLANE_INTERLEAVED,                            \
	.alpha_enable = alpha,                                            \
	.element = { (e0), (e1), (e2), (e3)},                             \
	.bits = { g, b, r, a },                                           \
	.chroma_sample = chroma,                                          \
	.unpack_align_msb = 0,                                            \
	.unpack_tight = 1,                                                \
	.unpack_count = count,                                            \
	.bpp = bp,                                                        \
	.fetch_mode = fm,                                                 \
	.flag = {(flg)},                                                  \
	.num_planes = np,                                                 \
	.tile_height = SDE_TILE_HEIGHT_DEFAULT                            \
}

#define PSEUDO_YUV_FMT(fmt, a, r, g, b, e0, e1, chroma, flg, fm, np)      \
{                                                                         \
	.base.pixel_format = DRM_FORMAT_ ## fmt,                          \
	.fetch_planes = SDE_PLANE_PSEUDO_PLANAR,                          \
	.alpha_enable = false,                                            \
	.element = { (e0), (e1), 0, 0 },                                  \
	.bits = { g, b, r, a },                                           \
	.chroma_sample = chroma,                                          \
	.unpack_align_msb = 0,                                            \
	.unpack_tight = 1,                                                \
	.unpack_count = 2,                                                \
	.bpp = 2,                                                         \
	.fetch_mode = fm,                                                 \
	.flag = {(flg)},                                                  \
	.num_planes = np,                                                 \
	.tile_height = SDE_TILE_HEIGHT_DEFAULT                            \
}

#define PSEUDO_YUV_FMT_TILED(fmt, a, r, g, b, e0, e1, chroma,             \
flg, fm, np, th)                                                          \
{                                                                         \
	.base.pixel_format = DRM_FORMAT_ ## fmt,                          \
	.fetch_planes = SDE_PLANE_PSEUDO_PLANAR,                          \
	.alpha_enable = false,                                            \
	.element = { (e0), (e1), 0, 0 },                                  \
	.bits = { g, b, r, a },                                           \
	.chroma_sample = chroma,                                          \
	.unpack_align_msb = 0,                                            \
	.unpack_tight = 1,                                                \
	.unpack_count = 2,                                                \
	.bpp = 2,                                                         \
	.fetch_mode = fm,                                                 \
	.flag = {(flg)},                                                  \
	.num_planes = np,                                                 \
	.tile_height = th                                                 \
}

#define PSEUDO_YUV_FMT_LOOSE(fmt, a, r, g, b, e0, e1, chroma, flg, fm, np)\
{                                                                         \
	.base.pixel_format = DRM_FORMAT_ ## fmt,                          \
	.fetch_planes = SDE_PLANE_PSEUDO_PLANAR,                          \
	.alpha_enable = false,                                            \
	.element = { (e0), (e1), 0, 0 },                                  \
	.bits = { g, b, r, a },                                           \
	.chroma_sample = chroma,                                          \
	.unpack_align_msb = 1,                                            \
	.unpack_tight = 0,                                                \
	.unpack_count = 2,                                                \
	.bpp = 2,                                                         \
	.fetch_mode = fm,                                                 \
	.flag = {(flg)},                                                  \
	.num_planes = np,                                                 \
	.tile_height = SDE_TILE_HEIGHT_DEFAULT                            \
}

#define PSEUDO_YUV_FMT_LOOSE_TILED(fmt, a, r, g, b, e0, e1, chroma,       \
flg, fm, np, th)                                                          \
{                                                                         \
	.base.pixel_format = DRM_FORMAT_ ## fmt,                          \
	.fetch_planes = SDE_PLANE_PSEUDO_PLANAR,                          \
	.alpha_enable = false,                                            \
	.element = { (e0), (e1), 0, 0 },                                  \
	.bits = { g, b, r, a },                                           \
	.chroma_sample = chroma,                                          \
	.unpack_align_msb = 1,                                            \
	.unpack_tight = 0,                                                \
	.unpack_count = 2,                                                \
	.bpp = 2,                                                         \
	.fetch_mode = fm,                                                 \
	.flag = {(flg)},                                                  \
	.num_planes = np,                                                 \
	.tile_height = th                                                 \
}


#define PLANAR_YUV_FMT(fmt, a, r, g, b, e0, e1, e2, alpha, chroma, bp,    \
flg, fm, np)                                                      \
{                                                                         \
	.base.pixel_format = DRM_FORMAT_ ## fmt,                          \
	.fetch_planes = SDE_PLANE_PLANAR,                                 \
	.alpha_enable = alpha,                                            \
	.element = { (e0), (e1), (e2), 0 },                               \
	.bits = { g, b, r, a },                                           \
	.chroma_sample = chroma,                                          \
	.unpack_align_msb = 0,                                            \
	.unpack_tight = 1,                                                \
	.unpack_count = 1,                                                \
	.bpp = bp,                                                        \
	.fetch_mode = fm,                                                 \
	.flag = {(flg)},                                                  \
	.num_planes = np,                                                 \
	.tile_height = SDE_TILE_HEIGHT_DEFAULT                            \
}

/*
 * struct sde_media_color_map - maps drm format to media format
 * @format: DRM base pixel format
 * @color: Media API color related to DRM format
 */
struct sde_media_color_map {
	uint32_t format;
	uint32_t color;
};

static const struct sde_format sde_format_map[] = {
	INTERLEAVED_RGB_FMT(ARGB8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C3_ALPHA, 4,
		true, 4, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(ABGR8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 4, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(XBGR8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 4, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(RGBA8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C1_B_Cb, C0_G_Y, C2_R_Cr, 4,
		true, 4, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(BGRA8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C2_R_Cr, C0_G_Y, C1_B_Cb, 4,
		true, 4, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(BGRX8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C2_R_Cr, C0_G_Y, C1_B_Cb, 4,
		false, 4, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(XRGB8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C3_ALPHA, 4,
		false, 4, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(RGBX8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C1_B_Cb, C0_G_Y, C2_R_Cr, 4,
		false, 4, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(RGB888,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, 0, 3,
		false, 3, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(BGR888,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, 0, 3,
		false, 3, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(RGB565,
		0, COLOR_5BIT, COLOR_6BIT, COLOR_5BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, 0, 3,
		false, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(BGR565,
		0, COLOR_5BIT, COLOR_6BIT, COLOR_5BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, 0, 3,
		false, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(ARGB1555,
		COLOR_ALPHA_1BIT, COLOR_5BIT, COLOR_5BIT, COLOR_5BIT,
		C3_ALPHA, C2_R_Cr, C0_G_Y, C1_B_Cb, 4,
		true, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(ABGR1555,
		COLOR_ALPHA_1BIT, COLOR_5BIT, COLOR_5BIT, COLOR_5BIT,
		C3_ALPHA, C1_B_Cb, C0_G_Y, C2_R_Cr, 4,
		true, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(RGBA5551,
		COLOR_ALPHA_1BIT, COLOR_5BIT, COLOR_5BIT, COLOR_5BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(BGRA5551,
		COLOR_ALPHA_1BIT, COLOR_5BIT, COLOR_5BIT, COLOR_5BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C3_ALPHA, 4,
		true, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(XRGB1555,
		COLOR_ALPHA_1BIT, COLOR_5BIT, COLOR_5BIT, COLOR_5BIT,
		C3_ALPHA, C2_R_Cr, C0_G_Y, C1_B_Cb, 4,
		false, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(XBGR1555,
		COLOR_ALPHA_1BIT, COLOR_5BIT, COLOR_5BIT, COLOR_5BIT,
		C3_ALPHA, C1_B_Cb, C0_G_Y, C2_R_Cr, 4,
		false, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(RGBX5551,
		COLOR_ALPHA_1BIT, COLOR_5BIT, COLOR_5BIT, COLOR_5BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		false, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(BGRX5551,
		COLOR_ALPHA_1BIT, COLOR_5BIT, COLOR_5BIT, COLOR_5BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C3_ALPHA, 4,
		false, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(ARGB4444,
		COLOR_ALPHA_4BIT, COLOR_4BIT, COLOR_4BIT, COLOR_4BIT,
		C3_ALPHA, C2_R_Cr, C0_G_Y, C1_B_Cb, 4,
		true, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(ABGR4444,
		COLOR_ALPHA_4BIT, COLOR_4BIT, COLOR_4BIT, COLOR_4BIT,
		C3_ALPHA, C1_B_Cb, C0_G_Y, C2_R_Cr, 4,
		true, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(RGBA4444,
		COLOR_ALPHA_4BIT, COLOR_4BIT, COLOR_4BIT, COLOR_4BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(BGRA4444,
		COLOR_ALPHA_4BIT, COLOR_4BIT, COLOR_4BIT, COLOR_4BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C3_ALPHA, 4,
		true, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(XRGB4444,
		COLOR_ALPHA_4BIT, COLOR_4BIT, COLOR_4BIT, COLOR_4BIT,
		C3_ALPHA, C2_R_Cr, C0_G_Y, C1_B_Cb, 4,
		false, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(XBGR4444,
		COLOR_ALPHA_4BIT, COLOR_4BIT, COLOR_4BIT, COLOR_4BIT,
		C3_ALPHA, C1_B_Cb, C0_G_Y, C2_R_Cr, 4,
		false, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(RGBX4444,
		COLOR_ALPHA_4BIT, COLOR_4BIT, COLOR_4BIT, COLOR_4BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		false, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(BGRX4444,
		COLOR_ALPHA_4BIT, COLOR_4BIT, COLOR_4BIT, COLOR_4BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C3_ALPHA, 4,
		false, 2, 0,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(BGRA1010102,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C2_R_Cr, C0_G_Y, C1_B_Cb, 4,
		true, 4, SDE_FORMAT_FLAG_DX,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(RGBA1010102,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C1_B_Cb, C0_G_Y, C2_R_Cr, 4,
		true, 4, SDE_FORMAT_FLAG_DX,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(ABGR2101010,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 4, SDE_FORMAT_FLAG_DX,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(ARGB2101010,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C3_ALPHA, 4,
		true, 4, SDE_FORMAT_FLAG_DX,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(XRGB2101010,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C3_ALPHA, 4,
		false, 4, SDE_FORMAT_FLAG_DX,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(BGRX1010102,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C2_R_Cr, C0_G_Y, C1_B_Cb, 4,
		false, 4, SDE_FORMAT_FLAG_DX,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(XBGR2101010,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		false, 4, SDE_FORMAT_FLAG_DX,
		SDE_FETCH_LINEAR, 1),

	INTERLEAVED_RGB_FMT(RGBX1010102,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C1_B_Cb, C0_G_Y, C2_R_Cr, 4,
		false, 4, SDE_FORMAT_FLAG_DX,
		SDE_FETCH_LINEAR, 1),

	PSEUDO_YUV_FMT(NV12,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C2_R_Cr,
		SDE_CHROMA_420, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_LINEAR, 2),

	PSEUDO_YUV_FMT(NV21,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C1_B_Cb,
		SDE_CHROMA_420, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_LINEAR, 2),

	PSEUDO_YUV_FMT(NV16,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C2_R_Cr,
		SDE_CHROMA_H2V1, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_LINEAR, 2),

	PSEUDO_YUV_FMT(NV61,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C1_B_Cb,
		SDE_CHROMA_H2V1, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_LINEAR, 2),

	INTERLEAVED_YUV_FMT(VYUY,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C0_G_Y,
		false, SDE_CHROMA_H2V1, 4, 2, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_LINEAR, 2),

	INTERLEAVED_YUV_FMT(UYVY,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C0_G_Y,
		false, SDE_CHROMA_H2V1, 4, 2, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_LINEAR, 2),

	INTERLEAVED_YUV_FMT(YUYV,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C0_G_Y, C1_B_Cb, C0_G_Y, C2_R_Cr,
		false, SDE_CHROMA_H2V1, 4, 2, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_LINEAR, 2),

	INTERLEAVED_YUV_FMT(YVYU,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C0_G_Y, C2_R_Cr, C0_G_Y, C1_B_Cb,
		false, SDE_CHROMA_H2V1, 4, 2, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_LINEAR, 2),

	PLANAR_YUV_FMT(YUV420,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C1_B_Cb, C0_G_Y,
		false, SDE_CHROMA_420, 1, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_LINEAR, 3),

	PLANAR_YUV_FMT(YVU420,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C2_R_Cr, C0_G_Y,
		false, SDE_CHROMA_420, 1, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_LINEAR, 3),
};

/*
 * A5x tile formats tables:
 * These tables hold the A5x tile formats supported.
 */
static const struct sde_format sde_format_map_tile[] = {
	INTERLEAVED_RGB_FMT_TILED(BGR565,
		0, COLOR_5BIT, COLOR_6BIT, COLOR_5BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, 0, 3,
		false, 2, 0,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	INTERLEAVED_RGB_FMT_TILED(ARGB8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C2_R_Cr, C0_G_Y, C1_B_Cb, 4,
		true, 4, 0,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	INTERLEAVED_RGB_FMT_TILED(ABGR8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C1_B_Cb, C0_G_Y, C2_R_Cr, 4,
		true, 4, 0,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	INTERLEAVED_RGB_FMT_TILED(XBGR8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		false, 4, 0,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	INTERLEAVED_RGB_FMT_TILED(RGBA8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 4, 0,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	INTERLEAVED_RGB_FMT_TILED(BGRA8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C3_ALPHA, 4,
		true, 4, 0,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	INTERLEAVED_RGB_FMT_TILED(BGRX8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C0_G_Y, C2_R_Cr, C3_ALPHA, 4,
		false, 4, 0,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	INTERLEAVED_RGB_FMT_TILED(XRGB8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C3_ALPHA, C2_R_Cr, C0_G_Y, C1_B_Cb, 4,
		false, 4, 0,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	INTERLEAVED_RGB_FMT_TILED(RGBX8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		false, 4, 0,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	INTERLEAVED_RGB_FMT_TILED(ABGR2101010,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 4, SDE_FORMAT_FLAG_DX,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	INTERLEAVED_RGB_FMT_TILED(XBGR2101010,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 4, SDE_FORMAT_FLAG_DX,
		SDE_FETCH_UBWC, 1, SDE_TILE_HEIGHT_TILED),

	PSEUDO_YUV_FMT_TILED(NV12,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C2_R_Cr,
		SDE_CHROMA_420, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_UBWC, 2, SDE_TILE_HEIGHT_NV12),

	PSEUDO_YUV_FMT_TILED(NV21,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C1_B_Cb,
		SDE_CHROMA_420, SDE_FORMAT_FLAG_YUV,
		SDE_FETCH_UBWC, 2, SDE_TILE_HEIGHT_NV12),
};

static const struct sde_format sde_format_map_p010_tile[] = {
	PSEUDO_YUV_FMT_LOOSE_TILED(NV12,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C2_R_Cr,
		SDE_CHROMA_420, (SDE_FORMAT_FLAG_YUV | SDE_FORMAT_FLAG_DX),
		SDE_FETCH_UBWC, 2, SDE_TILE_HEIGHT_NV12),
};

static const struct sde_format sde_format_map_tp10_tile[] = {
	PSEUDO_YUV_FMT_TILED(NV12,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C2_R_Cr,
		SDE_CHROMA_420, (SDE_FORMAT_FLAG_YUV | SDE_FORMAT_FLAG_DX),
		SDE_FETCH_UBWC, 2, SDE_TILE_HEIGHT_NV12),
};

/*
 * UBWC formats table:
 * This table holds the UBWC formats supported.
 * If a compression ratio needs to be used for this or any other format,
 * the data will be passed by user-space.
 */
static const struct sde_format sde_format_map_ubwc[] = {
	INTERLEAVED_RGB_FMT_TILED(BGR565,
		0, COLOR_5BIT, COLOR_6BIT, COLOR_5BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, 0, 3,
		false, 2, SDE_FORMAT_FLAG_COMPRESSED,
		SDE_FETCH_UBWC, 2, SDE_TILE_HEIGHT_UBWC),

	INTERLEAVED_RGB_FMT_TILED(ABGR8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 4, SDE_FORMAT_FLAG_COMPRESSED,
		SDE_FETCH_UBWC, 2, SDE_TILE_HEIGHT_UBWC),

	INTERLEAVED_RGB_FMT_TILED(XBGR8888,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		false, 4, SDE_FORMAT_FLAG_COMPRESSED,
		SDE_FETCH_UBWC, 2, SDE_TILE_HEIGHT_UBWC),

	INTERLEAVED_RGB_FMT_TILED(ABGR2101010,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 4, SDE_FORMAT_FLAG_DX | SDE_FORMAT_FLAG_COMPRESSED,
		SDE_FETCH_UBWC, 2, SDE_TILE_HEIGHT_UBWC),

	INTERLEAVED_RGB_FMT_TILED(XBGR2101010,
		COLOR_8BIT, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C2_R_Cr, C0_G_Y, C1_B_Cb, C3_ALPHA, 4,
		true, 4, SDE_FORMAT_FLAG_DX | SDE_FORMAT_FLAG_COMPRESSED,
		SDE_FETCH_UBWC, 2, SDE_TILE_HEIGHT_UBWC),

	PSEUDO_YUV_FMT_TILED(NV12,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C2_R_Cr,
		SDE_CHROMA_420, SDE_FORMAT_FLAG_YUV |
				SDE_FORMAT_FLAG_COMPRESSED,
		SDE_FETCH_UBWC, 4, SDE_TILE_HEIGHT_NV12),
};

static const struct sde_format sde_format_map_p010[] = {
	PSEUDO_YUV_FMT_LOOSE(NV12,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C2_R_Cr,
		SDE_CHROMA_420, (SDE_FORMAT_FLAG_YUV | SDE_FORMAT_FLAG_DX),
		SDE_FETCH_LINEAR, 2),
};

static const struct sde_format sde_format_map_p010_ubwc[] = {
	PSEUDO_YUV_FMT_LOOSE_TILED(NV12,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C2_R_Cr,
		SDE_CHROMA_420, (SDE_FORMAT_FLAG_YUV | SDE_FORMAT_FLAG_DX |
				SDE_FORMAT_FLAG_COMPRESSED),
		SDE_FETCH_UBWC, 4, SDE_TILE_HEIGHT_NV12),
};

static const struct sde_format sde_format_map_tp10_ubwc[] = {
	PSEUDO_YUV_FMT_TILED(NV12,
		0, COLOR_8BIT, COLOR_8BIT, COLOR_8BIT,
		C1_B_Cb, C2_R_Cr,
		SDE_CHROMA_420, (SDE_FORMAT_FLAG_YUV | SDE_FORMAT_FLAG_DX |
				SDE_FORMAT_FLAG_COMPRESSED),
		SDE_FETCH_UBWC, 4, SDE_TILE_HEIGHT_NV12),
};

bool sde_format_is_tp10_ubwc(const struct sde_format *fmt)
{
	if (SDE_FORMAT_IS_YUV(fmt) && SDE_FORMAT_IS_DX(fmt) &&
			SDE_FORMAT_IS_UBWC(fmt) &&
			(fmt->num_planes == 4) && fmt->unpack_tight)
		return true;
	else
		return false;
}

/* _sde_get_v_h_subsample_rate - Get subsample rates for all formats we support
 *   Note: Not using the drm_format_*_subsampling since we have formats
 */
static void _sde_get_v_h_subsample_rate(
	enum sde_chroma_samp_type chroma_sample,
	uint32_t *v_sample,
	uint32_t *h_sample)
{
	if (!v_sample || !h_sample)
		return;

	switch (chroma_sample) {
	case SDE_CHROMA_H2V1:
		*v_sample = 1;
		*h_sample = 2;
		break;
	case SDE_CHROMA_H1V2:
		*v_sample = 2;
		*h_sample = 1;
		break;
	case SDE_CHROMA_420:
		*v_sample = 2;
		*h_sample = 2;
		break;
	default:
		*v_sample = 1;
		*h_sample = 1;
		break;
	}
}

static int _sde_format_get_media_color_ubwc(const struct sde_format *fmt)
{
	static const struct sde_media_color_map sde_media_ubwc_map[] = {
		{DRM_FORMAT_ABGR8888, MMM_COLOR_FMT_RGBA8888_UBWC},
		{DRM_FORMAT_XBGR8888, MMM_COLOR_FMT_RGBA8888_UBWC},
		{DRM_FORMAT_ABGR2101010, MMM_COLOR_FMT_RGBA1010102_UBWC},
		{DRM_FORMAT_XBGR2101010, MMM_COLOR_FMT_RGBA1010102_UBWC},
		{DRM_FORMAT_BGR565, MMM_COLOR_FMT_RGB565_UBWC},
	};
	int color_fmt = -1;
	int i;

	if (fmt->base.pixel_format == DRM_FORMAT_NV12) {
		if (SDE_FORMAT_IS_DX(fmt)) {
			if (fmt->unpack_tight)
				color_fmt = MMM_COLOR_FMT_NV12_BPP10_UBWC;
			else
				color_fmt = MMM_COLOR_FMT_P010_UBWC;
		} else
			color_fmt = MMM_COLOR_FMT_NV12_UBWC;
		return color_fmt;
	}

	for (i = 0; i < ARRAY_SIZE(sde_media_ubwc_map); ++i)
		if (fmt->base.pixel_format == sde_media_ubwc_map[i].format) {
			color_fmt = sde_media_ubwc_map[i].color;
			break;
		}
	return color_fmt;
}

static int _sde_format_get_plane_sizes_ubwc(
		const struct sde_format *fmt,
		const uint32_t width,
		const uint32_t height,
		struct sde_hw_fmt_layout *layout)
{
	int i;
	int color;
	bool meta = SDE_FORMAT_IS_UBWC(fmt);

	memset(layout, 0, sizeof(struct sde_hw_fmt_layout));
	layout->format = fmt;
	layout->width = width;
	layout->height = height;
	layout->num_planes = fmt->num_planes;

	color = _sde_format_get_media_color_ubwc(fmt);
	if (color < 0) {
		DRM_ERROR("UBWC format not supported for fmt: %4.4s\n",
			(char *)&fmt->base.pixel_format);
		return -EINVAL;
	}

	if (SDE_FORMAT_IS_YUV(layout->format)) {
		uint32_t y_sclines, uv_sclines;
		uint32_t y_meta_scanlines = 0;
		uint32_t uv_meta_scanlines = 0;

		layout->num_planes = 2;
		layout->plane_pitch[0] = MMM_COLOR_FMT_Y_STRIDE(color, width);
		y_sclines = MMM_COLOR_FMT_Y_SCANLINES(color, height);
		layout->plane_size[0] =
			MMM_COLOR_FMT_ALIGN(layout->plane_pitch[0] *
			y_sclines, SDE_UBWC_PLANE_SIZE_ALIGNMENT);

		layout->plane_pitch[1] = MMM_COLOR_FMT_UV_STRIDE(color, width);
		uv_sclines = MMM_COLOR_FMT_UV_SCANLINES(color, height);
		layout->plane_size[1] =
			MMM_COLOR_FMT_ALIGN(layout->plane_pitch[1] *
			uv_sclines, SDE_UBWC_PLANE_SIZE_ALIGNMENT);

		if (!meta)
			goto done;

		layout->num_planes += 2;
		layout->plane_pitch[2] =
			MMM_COLOR_FMT_Y_META_STRIDE(color, width);
		y_meta_scanlines =
			MMM_COLOR_FMT_Y_META_SCANLINES(color, height);
		layout->plane_size[2] =
			MMM_COLOR_FMT_ALIGN(layout->plane_pitch[2] *
			y_meta_scanlines, SDE_UBWC_PLANE_SIZE_ALIGNMENT);

		layout->plane_pitch[3] =
			MMM_COLOR_FMT_UV_META_STRIDE(color, width);
		uv_meta_scanlines =
			MMM_COLOR_FMT_UV_META_SCANLINES(color, height);
		layout->plane_size[3] =
			MMM_COLOR_FMT_ALIGN(layout->plane_pitch[3] *
			uv_meta_scanlines, SDE_UBWC_PLANE_SIZE_ALIGNMENT);

	} else {
		uint32_t rgb_scanlines, rgb_meta_scanlines;

		layout->num_planes = 1;

		layout->plane_pitch[0] =
			MMM_COLOR_FMT_RGB_STRIDE(color, width);
		rgb_scanlines = MMM_COLOR_FMT_RGB_SCANLINES(color, height);
		layout->plane_size[0] =
			MMM_COLOR_FMT_ALIGN(layout->plane_pitch[0] *
			rgb_scanlines, SDE_UBWC_PLANE_SIZE_ALIGNMENT);

		if (!meta)
			goto done;
		layout->num_planes += 2;
		layout->plane_pitch[2] =
			MMM_COLOR_FMT_RGB_META_STRIDE(color, width);
		rgb_meta_scanlines =
			MMM_COLOR_FMT_RGB_META_SCANLINES(color, height);
		layout->plane_size[2] =
			MMM_COLOR_FMT_ALIGN(layout->plane_pitch[2] *
			rgb_meta_scanlines, SDE_UBWC_PLANE_SIZE_ALIGNMENT);
	}

done:
	for (i = 0; i < SDE_MAX_PLANES; i++)
		layout->total_size += layout->plane_size[i];

	return 0;
}

static int _sde_format_get_plane_sizes_linear(
		const struct sde_format *fmt,
		const uint32_t width,
		const uint32_t height,
		struct sde_hw_fmt_layout *layout,
		const uint32_t *pitches)
{
	int i;

	memset(layout, 0, sizeof(struct sde_hw_fmt_layout));
	layout->format = fmt;
	layout->width = width;
	layout->height = height;
	layout->num_planes = fmt->num_planes;

	/* Due to memset above, only need to set planes of interest */
	if (fmt->fetch_planes == SDE_PLANE_INTERLEAVED) {
		layout->num_planes = 1;
		layout->plane_size[0] = width * height * layout->format->bpp;
		layout->plane_pitch[0] = width * layout->format->bpp;
	} else {
		uint32_t v_subsample, h_subsample;
		uint32_t chroma_samp;
		uint32_t bpp = 1;

		chroma_samp = fmt->chroma_sample;
		_sde_get_v_h_subsample_rate(chroma_samp, &v_subsample,
				&h_subsample);

		if (width % h_subsample || height % v_subsample) {
			DRM_ERROR("mismatch in subsample vs dimensions\n");
			return -EINVAL;
		}

		if ((fmt->base.pixel_format == DRM_FORMAT_NV12) &&
			(SDE_FORMAT_IS_DX(fmt)))
			bpp = 2;
		layout->plane_pitch[0] = width * bpp;
		layout->plane_pitch[1] = layout->plane_pitch[0] / h_subsample;
		layout->plane_size[0] = layout->plane_pitch[0] * height;
		layout->plane_size[1] = layout->plane_pitch[1] *
				(height / v_subsample);

		if (fmt->fetch_planes == SDE_PLANE_PSEUDO_PLANAR) {
			layout->num_planes = 2;
			layout->plane_size[1] *= 2;
			layout->plane_pitch[1] *= 2;
		} else {
			/* planar */
			layout->num_planes = 3;
			layout->plane_size[2] = layout->plane_size[1];
			layout->plane_pitch[2] = layout->plane_pitch[1];
		}
	}

	/*
	 * linear format: allow user allocated pitches if they are greater than
	 * the requirement.
	 * ubwc format: pitch values are computed uniformly across
	 * all the components based on ubwc specifications.
	 */
	for (i = 0; i < layout->num_planes && i < SDE_MAX_PLANES; ++i) {
		if (pitches && layout->plane_pitch[i] < pitches[i])
			layout->plane_pitch[i] = pitches[i];
	}

	for (i = 0; i < SDE_MAX_PLANES; i++)
		layout->total_size += layout->plane_size[i];

	return 0;
}

int sde_format_get_plane_sizes(
		const struct sde_format *fmt,
		const uint32_t w,
		const uint32_t h,
		struct sde_hw_fmt_layout *layout,
		const uint32_t *pitches)
{
	if (!layout || !fmt) {
		DRM_ERROR("invalid pointer\n");
		return -EINVAL;
	}

	if ((w > SDE_MAX_IMG_WIDTH) || (h > SDE_MAX_IMG_HEIGHT)) {
		DRM_ERROR("image dimensions outside max range\n");
		return -ERANGE;
	}

	if (SDE_FORMAT_IS_UBWC(fmt) || SDE_FORMAT_IS_TILE(fmt))
		return _sde_format_get_plane_sizes_ubwc(fmt, w, h, layout);

	return _sde_format_get_plane_sizes_linear(fmt, w, h, layout, pitches);
}

int sde_format_get_block_size(const struct sde_format *fmt,
		uint32_t *w, uint32_t *h)
{
	if (!fmt || !w || !h) {
		DRM_ERROR("invalid pointer\n");
		return -EINVAL;
	}

	/* TP10 is 96x96 and all others are 128x128 */
	if (SDE_FORMAT_IS_YUV(fmt) && SDE_FORMAT_IS_DX(fmt) &&
			(fmt->num_planes == 2) && fmt->unpack_tight)
		*w = *h = 96;
	else
		*w = *h = 128;

	return 0;
}

uint32_t sde_format_get_framebuffer_size(
		const uint32_t format,
		const uint32_t width,
		const uint32_t height,
		const uint32_t *pitches,
		const uint64_t modifier)
{
	const struct sde_format *fmt;
	struct sde_hw_fmt_layout layout;

	fmt = sde_get_sde_format_ext(format, modifier);
	if (!fmt)
		return 0;

	if (!pitches)
		return -EINVAL;

	if (sde_format_get_plane_sizes(fmt, width, height, &layout, pitches))
		layout.total_size = 0;

	return layout.total_size;
}

static int _sde_format_populate_addrs_ubwc(
		struct msm_gem_address_space *aspace,
		struct drm_framebuffer *fb,
		struct sde_hw_fmt_layout *layout)
{
	uint32_t base_addr;
	bool meta;

	if (!fb || !layout) {
		DRM_ERROR("invalid pointers\n");
		return -EINVAL;
	}

	if (aspace)
		base_addr = msm_framebuffer_iova(fb, aspace, 0);
	else
		base_addr = msm_framebuffer_phys(fb, 0);
	if (!base_addr) {
		DRM_ERROR("failed to retrieve base addr\n");
		return -EFAULT;
	}

	meta = SDE_FORMAT_IS_UBWC(layout->format);

	/* Per-format logic for verifying active planes */
	if (SDE_FORMAT_IS_YUV(layout->format)) {
		/************************************************/
		/*      UBWC            **                      */
		/*      buffer          **      SDE PLANE       */
		/*      format          **                      */
		/************************************************/
		/* -------------------  ** -------------------- */
		/* |      Y meta     |  ** |    Y bitstream   | */
		/* |       data      |  ** |       plane      | */
		/* -------------------  ** -------------------- */
		/* |    Y bitstream  |  ** |  CbCr bitstream  | */
		/* |       data      |  ** |       plane      | */
		/* -------------------  ** -------------------- */
		/* |   Cbcr metadata |  ** |       Y meta     | */
		/* |       data      |  ** |       plane      | */
		/* -------------------  ** -------------------- */
		/* |  CbCr bitstream |  ** |     CbCr meta    | */
		/* |       data      |  ** |       plane      | */
		/* -------------------  ** -------------------- */
		/************************************************/

		/* configure Y bitstream plane */
		layout->plane_addr[0] = base_addr + layout->plane_size[2];

		/* configure CbCr bitstream plane */
		layout->plane_addr[1] = base_addr + layout->plane_size[0]
			+ layout->plane_size[2] + layout->plane_size[3];

		if (!meta)
			goto done;

		/* configure Y metadata plane */
		layout->plane_addr[2] = base_addr;

		/* configure CbCr metadata plane */
		layout->plane_addr[3] = base_addr + layout->plane_size[0]
			+ layout->plane_size[2];

	} else {
		/************************************************/
		/*      UBWC            **                      */
		/*      buffer          **      SDE PLANE       */
		/*      format          **                      */
		/************************************************/
		/* -------------------  ** -------------------- */
		/* |      RGB meta   |  ** |   RGB bitstream  | */
		/* |       data      |  ** |       plane      | */
		/* -------------------  ** -------------------- */
		/* |  RGB bitstream  |  ** |       NONE       | */
		/* |       data      |  ** |                  | */
		/* -------------------  ** -------------------- */
		/*                      ** |     RGB meta     | */
		/*                      ** |       plane      | */
		/*                      ** -------------------- */
		/************************************************/

		layout->plane_addr[0] = base_addr + layout->plane_size[2];
		layout->plane_addr[1] = 0;

		if (!meta)
			goto done;

		layout->plane_addr[2] = base_addr;
		layout->plane_addr[3] = 0;
	}
done:
	return 0;
}

static int _sde_format_populate_addrs_linear(
		struct msm_gem_address_space *aspace,
		struct drm_framebuffer *fb,
		struct sde_hw_fmt_layout *layout)
{
	unsigned int i;

	/* Can now check the pitches given vs pitches expected */
	for (i = 0; i < layout->num_planes; ++i) {
		if (layout->plane_pitch[i] > fb->pitches[i]) {
			DRM_ERROR("plane %u expected pitch %u, fb %u\n",
				i, layout->plane_pitch[i], fb->pitches[i]);
			return -EINVAL;
		}
	}

	/* Populate addresses for simple formats here */
	for (i = 0; i < layout->num_planes; ++i) {
		if (aspace)
			layout->plane_addr[i] =
				msm_framebuffer_iova(fb, aspace, i);
		else
			layout->plane_addr[i] = msm_framebuffer_phys(fb, i);
		if (!layout->plane_addr[i]) {
			DRM_ERROR("failed to retrieve base addr\n");
			return -EFAULT;
		}
	}

	return 0;
}

int sde_format_populate_layout(
		struct msm_gem_address_space *aspace,
		struct drm_framebuffer *fb,
		struct sde_hw_fmt_layout *layout)
{
	uint32_t plane_addr[SDE_MAX_PLANES];
	int i, ret;

	if (!fb || !layout) {
		DRM_ERROR("invalid arguments\n");
		return -EINVAL;
	}

	if ((fb->width > SDE_MAX_IMG_WIDTH) ||
			(fb->height > SDE_MAX_IMG_HEIGHT)) {
		DRM_ERROR("image dimensions outside max range\n");
		return -ERANGE;
	}

	layout->format = to_sde_format(msm_framebuffer_format(fb));

	/* Populate the plane sizes etc via get_format */
	ret = sde_format_get_plane_sizes(layout->format, fb->width, fb->height,
			layout, fb->pitches);
	if (ret)
		return ret;

	for (i = 0; i < SDE_MAX_PLANES; ++i)
		plane_addr[i] = layout->plane_addr[i];

	/* Populate the addresses given the fb */
	if (SDE_FORMAT_IS_UBWC(layout->format) ||
			SDE_FORMAT_IS_TILE(layout->format))
		ret = _sde_format_populate_addrs_ubwc(aspace, fb, layout);
	else
		ret = _sde_format_populate_addrs_linear(aspace, fb, layout);

	/* check if anything changed */
	if (!ret && !memcmp(plane_addr, layout->plane_addr, sizeof(plane_addr)))
		ret = -EAGAIN;

	return ret;
}

int sde_format_check_modified_format(
		const struct msm_kms *kms,
		const struct msm_format *msm_fmt,
		const struct drm_mode_fb_cmd2 *cmd,
		struct drm_gem_object **bos)
{
	const struct drm_format_info *info;
	const struct sde_format *fmt;
	struct sde_hw_fmt_layout layout;
	uint32_t bos_total_size = 0;
	int ret, i;

	if (!msm_fmt || !cmd || !bos) {
		DRM_ERROR("invalid arguments\n");
		return -EINVAL;
	}

	fmt = to_sde_format(msm_fmt);
	info = drm_format_info(fmt->base.pixel_format);
	if (!info)
		return -EINVAL;

	ret = sde_format_get_plane_sizes(fmt, cmd->width, cmd->height,
			&layout, cmd->pitches);
	if (ret)
		return ret;

	for (i = 0; i < info->num_planes; i++) {
		if (!bos[i]) {
			DRM_ERROR("invalid handle for plane %d\n", i);
			return -EINVAL;
		}
		if ((i == 0) || (bos[i] != bos[0]))
			bos_total_size += bos[i]->size;
	}

	if (bos_total_size < layout.total_size) {
		DRM_ERROR("buffers total size too small %u expected %u\n",
				bos_total_size, layout.total_size);
		return -EINVAL;
	}

	return 0;
}

const struct sde_format *sde_get_sde_format_ext(
		const uint32_t format,
		const uint64_t modifier)
{
	uint32_t i = 0;
	const struct sde_format *fmt = NULL;
	const struct sde_format *map = NULL;
	ssize_t map_size = 0;

	/*
	 * Currently only support exactly zero or one modifier.
	 * All planes use the same modifier.
	 */
	SDE_DEBUG("plane format modifier 0x%llX\n", modifier);

	switch (modifier) {
	case 0:
		map = sde_format_map;
		map_size = ARRAY_SIZE(sde_format_map);
		break;
	case DRM_FORMAT_MOD_QCOM_COMPRESSED:
	case DRM_FORMAT_MOD_QCOM_COMPRESSED | DRM_FORMAT_MOD_QCOM_TILE:
		map = sde_format_map_ubwc;
		map_size = ARRAY_SIZE(sde_format_map_ubwc);
		SDE_DEBUG("found fmt: %4.4s  DRM_FORMAT_MOD_QCOM_COMPRESSED\n",
				(char *)&format);
		break;
	case DRM_FORMAT_MOD_QCOM_DX:
		map = sde_format_map_p010;
		map_size = ARRAY_SIZE(sde_format_map_p010);
		SDE_DEBUG("found fmt: %4.4s DRM_FORMAT_MOD_QCOM_DX\n",
				(char *)&format);
		break;
	case (DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_COMPRESSED):
	case (DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_COMPRESSED |
			DRM_FORMAT_MOD_QCOM_TILE):
		map = sde_format_map_p010_ubwc;
		map_size = ARRAY_SIZE(sde_format_map_p010_ubwc);
		SDE_DEBUG(
			"found fmt: %4.4s DRM_FORMAT_MOD_QCOM_COMPRESSED/DX\n",
				(char *)&format);
		break;
	case (DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_COMPRESSED |
		DRM_FORMAT_MOD_QCOM_TIGHT):
	case (DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_COMPRESSED |
		DRM_FORMAT_MOD_QCOM_TIGHT | DRM_FORMAT_MOD_QCOM_TILE):
		map = sde_format_map_tp10_ubwc;
		map_size = ARRAY_SIZE(sde_format_map_tp10_ubwc);
		SDE_DEBUG(
			"found fmt: %4.4s DRM_FORMAT_MOD_QCOM_COMPRESSED/DX/TIGHT\n",
				(char *)&format);
		break;
	case DRM_FORMAT_MOD_QCOM_TILE:
		map = sde_format_map_tile;
		map_size = ARRAY_SIZE(sde_format_map_tile);
		SDE_DEBUG("found fmt: %4.4s DRM_FORMAT_MOD_QCOM_TILE\n",
				(char *)&format);
		break;
	case (DRM_FORMAT_MOD_QCOM_TILE | DRM_FORMAT_MOD_QCOM_DX):
		map = sde_format_map_p010_tile;
		map_size = ARRAY_SIZE(sde_format_map_p010_tile);
		SDE_DEBUG("found fmt: %4.4s DRM_FORMAT_MOD_QCOM_TILE/DX\n",
				(char *)&format);
		break;
	case (DRM_FORMAT_MOD_QCOM_TILE | DRM_FORMAT_MOD_QCOM_DX |
			DRM_FORMAT_MOD_QCOM_TIGHT):
		map = sde_format_map_tp10_tile;
		map_size = ARRAY_SIZE(sde_format_map_tp10_tile);
		SDE_DEBUG(
			"found fmt: %4.4s DRM_FORMAT_MOD_QCOM_TILE/DX/TIGHT\n",
				(char *)&format);
		break;
	default:
		SDE_ERROR("unsupported format modifier %llX\n", modifier);
		return NULL;
	}

	for (i = 0; i < map_size; i++) {
		if (format == map[i].base.pixel_format) {
			fmt = &map[i];
			break;
		}
	}

	if (fmt == NULL)
		SDE_ERROR("unsupported fmt: %4.4s modifier 0x%llX\n",
				(char *)&format, modifier);
	else
		SDE_DEBUG("fmt %4.4s mod 0x%llX ubwc %d yuv %d\n",
				(char *)&format, modifier,
				SDE_FORMAT_IS_UBWC(fmt),
				SDE_FORMAT_IS_YUV(fmt));

	return fmt;
}

const struct msm_format *sde_get_msm_format(
		struct msm_kms *kms,
		const uint32_t format,
		const uint64_t modifier)
{
	const struct sde_format *fmt = sde_get_sde_format_ext(format,
			modifier);
	if (fmt)
		return &fmt->base;
	return NULL;
}

uint32_t sde_populate_formats(
		const struct sde_format_extended *format_list,
		uint32_t *pixel_formats,
		uint64_t *pixel_modifiers,
		uint32_t pixel_formats_max)
{
	uint32_t i, fourcc_format;

	if (!format_list || !pixel_formats)
		return 0;

	for (i = 0, fourcc_format = 0;
			format_list->fourcc_format && i < pixel_formats_max;
			++format_list) {
		/* verify if listed format is in sde_format_map? */

		/* optionally return modified formats */
		if (pixel_modifiers) {
			/* assume same modifier for all fb planes */
			pixel_formats[i] = format_list->fourcc_format;
			pixel_modifiers[i++] = format_list->modifier;
		} else {
			/* assume base formats grouped together */
			if (fourcc_format != format_list->fourcc_format) {
				fourcc_format = format_list->fourcc_format;
				pixel_formats[i++] = fourcc_format;
			}
		}
	}

	return i;
}

int sde_format_validate_fmt(struct msm_kms *kms,
	const struct sde_format *sde_fmt,
	const struct sde_format_extended *fmt_list)
{
	const struct sde_format *fmt_tmp;
	bool valid_format = false;
	int ret = 0;

	if (!sde_fmt || !fmt_list) {
		SDE_ERROR("invalid fmt:%d list:%d\n",
			!sde_fmt, !fmt_list);
		ret = -EINVAL;
		goto exit;
	}

	while (fmt_list->fourcc_format) {
		fmt_tmp = sde_get_sde_format_ext(fmt_list->fourcc_format,
					fmt_list->modifier);
		if (fmt_tmp
		  && (fmt_tmp->base.pixel_format == sde_fmt->base.pixel_format)
		  && (fmt_tmp->fetch_mode == sde_fmt->fetch_mode)) {
			valid_format = true;
			break;
		}
		++fmt_list;
	}

	if (!valid_format) {
		SDE_ERROR("fmt:%d mode:%d not found within the list!\n",
			sde_fmt->base.pixel_format, sde_fmt->fetch_mode);
		ret = -EINVAL;
	}
exit:
	return ret;
}
