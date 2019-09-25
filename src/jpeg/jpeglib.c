/*
 * This software is based in part on the work of the Independent JPEG Group.
 *
 * The authors make NO WARRANTY or representation, either express or implied,
 * with respect to this software, its quality, accuracy, merchantability, or
 * fitness for a particular purpose.  This software is provided "AS IS", and
 * you, its user, assume the entire risk as to its quality and accuracy.
 *
 * This software is copyright (C) 1994-1996, Thomas G. Lane.
 * All Rights Reserved except as specified below.
 *
 * Permission is hereby granted to use, copy, modify, and distribute this
 * software (or portions thereof) for any purpose, without fee, subject to
 * these conditions:
 * (1) If any part of the source code for this software is distributed, then
 * this README file must be included, with this copyright and no-warranty
 * notice unaltered; and any additions, deletions, or changes to the original
 * files must be clearly indicated in accompanying documentation.
 * (2) If only executable code is distributed, then the accompanying
 * documentation must state that "this software is based in part on the work
 * of the Independent JPEG Group".
 * (3) Permission for use of this software is granted only if the user accepts
 * full responsibility for any undesirable consequences; the authors accept
 * NO LIABILITY for damages of any kind.
 *
 * These conditions apply to any software derived from or based on the IJG
 * code, not just to the unmodified library.  If you use our work, you ought
 * to acknowledge us.
 *
 * Permission is NOT granted for the use of any IJG author's name or company
 * name in advertising or publicity relating to this software or products
 * derived from it.  This software may be referred to only as "the Independent
 * JPEG Group's software".
 *
 * We specifically permit and encourage the use of this software as the basis
 * of commercial products, provided that all warranty or liability claims are
 * assumed by the product vendor.
 */

/**
 * @file
 * reference JPEG Group's jcparam.c, jcmarker.c.
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "jpeglib.h"

#define SCALEBITS       (16)
#define FIX(X)          ((1L << SCALEBITS) / (X) + 1)

static const unsigned char jpeg_natural_order[DCTSIZE2] = {
	0,  1,  8,  16,  9,  2,  3, 10,
	17, 24, 32, 25, 18, 11,  4,  5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13,  6,  7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63
};

void emit_dqt(JpegCtx *JpegEncCtx, int index)
{
	int i;

	emit_marker(JpegEncCtx, M_DQT);
	emit_2bytes(JpegEncCtx, DCTSIZE2 + 1 + 2);
	emit_byte(JpegEncCtx, index + (0 << 4));

	for (i = 0; i < DCTSIZE2; i++) {
		/* The table entries must be emitted in zigzag order. */
		unsigned char qval = (unsigned char)JpegEncCtx->quant_tbl[index][jpeg_natural_order[i]];

		emit_byte(JpegEncCtx, qval);
	}
}

void emit_sof(JpegCtx *JpegEncCtx)
{
	emit_marker(JpegEncCtx, M_SOF0);
	emit_2bytes(JpegEncCtx, 3 * 3 + 2 + 5 + 1); /* length */

	emit_byte(JpegEncCtx, 8);//data_precision
	emit_2bytes(JpegEncCtx, JpegEncCtx->image_height);
	emit_2bytes(JpegEncCtx, JpegEncCtx->image_width);

	emit_byte(JpegEncCtx, 3);//num_components
	emit_byte(JpegEncCtx, 1);
	if(JpegEncCtx->JpgColorFormat == JpgYUV420)
		emit_byte(JpegEncCtx, (2 << 4) + 2);
	else if(JpegEncCtx->JpgColorFormat == JpgYUV422)
		emit_byte(JpegEncCtx, (2 << 4) + 1);
	else if(JpegEncCtx->JpgColorFormat == JpgYUV444)
		emit_byte(JpegEncCtx, (1 << 4) + 1);

	emit_byte(JpegEncCtx, 0);
	emit_byte(JpegEncCtx, 2);
	emit_byte(JpegEncCtx, (1 << 4) + 1);
	emit_byte(JpegEncCtx, 1);
	emit_byte(JpegEncCtx, 3);
	emit_byte(JpegEncCtx, (1 << 4) + 1);
	emit_byte(JpegEncCtx, 1);
}

void emit_sos(JpegCtx *JpegEncCtx)
{
	emit_marker(JpegEncCtx, M_SOS);
	emit_2bytes(JpegEncCtx, 2 * 3 + 2 + 1 + 3); /* length */
	emit_byte(JpegEncCtx, 3);//jpgenc->comps_in_scan

	emit_byte(JpegEncCtx, 1);
	emit_byte(JpegEncCtx, (0 << 4) + 0);

	emit_byte(JpegEncCtx, 2);
	emit_byte(JpegEncCtx, (1 << 4) + 1);

	emit_byte(JpegEncCtx, 3);
	emit_byte(JpegEncCtx, (1 << 4) + 1);

	emit_byte(JpegEncCtx, 0);
	emit_byte(JpegEncCtx, 63);
	emit_byte(JpegEncCtx, 0);
}

static void jpeg_add_quant_table(JpegCtx *JpegEncCtx, int which_tbl,
                                 const unsigned char *basic_table,
                                 int scale_factor)
{
	unsigned short *qtblptr;
	int i;

	qtblptr = JpegEncCtx->quant_tbl[which_tbl];//quant_tbl[0] for lum,quant_tbl[1] for cum

	for (i = 0; i < DCTSIZE2; i++) {
		int temp = (basic_table[i] * scale_factor + 50L) / 100L;//scale_factor

		if (temp <= 0L) {
			temp = 1L;
		}
		if (temp > 255L) {
			temp = 255L;        /* limit to baseline range if requested */
		}
		*(qtblptr + i) = (unsigned short) temp; //8*8=64,
		//JpegEncCtx->quant_ori_tbl[which_tbl*64+i] = temp;
	}
}

void jpeg_set_quant_tbl(void *handle, int quality)
{
	int i, j;
	int temp;

	JpegCtx *JpegEncCtx = (JpegCtx *)handle;

	/* These are the sample quantization tables given in JPEG spec section K.1.
	* The spec says that the values given produce "good" quality, and
	* when divided by 2, "very good" quality.
	    */
	static const unsigned char std_luminance_quant_tbl[DCTSIZE2] = {
		16,  11,  10,  16,  24,  40,  51,  61,
		12,  12,  14,  19,  26,  58,  60,  55,
		14,  13,  16,  24,  40,  57,  69,  56,
		14,  17,  22,  29,  51,  87,  80,  62,
		18,  22,  37,  56,  68, 109, 103,  77,
		24,  35,  55,  64,  81, 104, 113,  92,
		49,  64,  78,  87, 103, 121, 120, 101,
		72,  92,  95,  98, 112, 100, 103,  99
	};
	static const unsigned char std_chrominance_quant_tbl[DCTSIZE2] = {
		17,  18,  24,  47,  99,  99,  99,  99,
		18,  21,  26,  66,  99,  99,  99,  99,
		24,  26,  56,  99,  99,  99,  99,  99,
		47,  66,  99,  99,  99,  99,  99,  99,
		99,  99,  99,  99,  99,  99,  99,  99,
		99,  99,  99,  99,  99,  99,  99,  99,
		99,  99,  99,  99,  99,  99,  99,  99,
		99,  99,  99,  99,  99,  99,  99,  99
	};

	if (quality <= 0) {
		quality = 1;
	}
	if (quality > 100) {
		quality = 100;
	}
	if (quality < 50) {
		quality = 5000 / quality;
	} else {
		quality = 200 - quality * 2;//Quality
	}
	jpeg_add_quant_table(JpegEncCtx, 0, std_luminance_quant_tbl, quality);
	jpeg_add_quant_table(JpegEncCtx, 1, std_chrominance_quant_tbl, quality);

	//convert it to aw quant table
	for (j = 0; j < 2; j++) {
		for (i = 0; i < 64; i++) {
			temp = FIX(JpegEncCtx->quant_tbl[j][i]);
			temp = temp > 0xffff ? 0xffff : temp;

			//attention: After Process, these table format in block also
			//             Interlace qp/2,1/qp.
			JpegEncCtx->quant_tbl_aw[128 * j + 2 * i + 0] = temp;
			JpegEncCtx->quant_tbl_aw[128 * j + 2 * i + 1] = JpegEncCtx->quant_tbl[j][i] >> 1;
		}
	}
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

