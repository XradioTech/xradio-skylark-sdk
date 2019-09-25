/**
  * @file	hal_csi.h
  * @author  XRADIO IOT WLAN Team
  */

/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _DRIVER_CHIP_HAL_CSI_JPEG_H_
#define _DRIVER_CHIP_HAL_CSI_JPEG_H_

#include "driver/chip/hal_def.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief The register for CSI.
  */
typedef struct {
	__IO uint32_t CSI_EN_REG;			/*!< 0x00 CSI enable register */
	__IO uint32_t CSI_CFG_REG;			/*!< 0x04 CSI configuration register */
	__IO uint32_t CSI_CAP_REG;			/*!< 0x08 CSI capture control register */
	__I  uint32_t CSI_SIGNAL_STA_REG;	/*!< 0x0C CSI signal status register */
	__IO uint32_t CSI_C0_HSIZE_REG; 	/*!< 0x10 CSI horizontal size register */
	__IO uint32_t CSI_C0_VSIZE_REG; 	/*!< 0x14 CSI vertical size register */
	__IO uint32_t CSI_C0_IN_SIZE_REG; 	/*!< 0x18 CSI input size register */
	__IO uint32_t CSI_C0_INT_EN_REG;	/*!< 0x1C CSI interrupt enable register */
	__IO uint32_t CSI_C0_INT_STA_REG;	/*!< 0x20 CSI interrupt status register */
} CSI_T;

#define	CSI	((CSI_T *)CSI_BASE)

#if (__CONFIG_CHIP_ARCH_VER == 2)
/*
 * Bits definition for CSI enable register (0x0000)
 */
#define CSI_PCLK_EN						HAL_BIT(2)
#define CSI_NCSIC_EN					HAL_BIT(1)
#define CSI_PRS_EN						HAL_BIT(0)

/*
 * Bits definition for CSI configuration register (0x0004)
 */
#define CSI_YUV420_MASK_SHIFT			(12) 	/*!< valid when output mode set YUV422 to YUV420 */
#define CSI_YUV420_MASK_MASK			(0x1U << CSI_YUV420_MASK_SHIFT)
typedef enum {
	CSI_YUV420_MASK_UV_ODD,
	CSI_YUV420_MASK_UV_EVEN,
} CSI_YUV420Mask;

#define CSI_OUTPUT_MODE_SHIFT			(11)	/*!< 0: original output 1: YUV422 to YUV420 */
#define CSI_OUTPUT_MODE_MASK			(0x1U << CSI_OUTPUT_MODE_SHIFT)
typedef enum {
	CSI_OUT_MODE_ORIGINAL,
	CSI_OUT_MODE_YUV422_TO_YUV420,
} CSI_OutputMode;

#define CSI_YUV420_LINE_ORDER_SHIFT		(10)	/*!< YUV420 line order */
#define CSI_YUV420_LINE_ORDER_MASK		(0x1U << CSI_YUV420_LINE_ORDER_SHIFT)
typedef enum {
	CSI_LINE_ORDER_Y_YC_Y_YC,
	CSI_LINE_ORDER_YC_Y_YC_Y,
} CSI_YUV420LineOrder;

#define CSI_INPUT_SEQ_SHIFT				(8)	/*!< input data sequence, only valid for YUV422 and YUV420 input format */
#define CSI_INPUT_SEQ_MASK				(0x3U << CSI_INPUT_SEQ_SHIFT)
typedef enum {
	CSI_IN_SEQ_YUYV,
	CSI_IN_SEQ_YVYU,
	CSI_IN_SEQ_UYVY,
	CSI_IN_SEQ_VYUY,
} CSI_InputSeq;

#define CSI_INPUT_FMT_SHIFT				(6)	/*!< input data format */
#define CSI_INPUT_FMT_MASK				(0x3U << CSI_INPUT_FMT_SHIFT)
typedef enum {
	CSI_IN_FMT_RAW,
	CSI_IN_FMT_RAW1,
	CSI_IN_FMT_YUV422,
	CSI_IN_FMT_YUV420,
} CSI_InputFmt;

#define CSI_VREF_POL_SHIFT				(3)
#define CSI_VREF_POL_MASK				(0x1U << CSI_VREF_POL_SHIFT)	/*!< Vref polarity */
#define CSI_HREF_POL_SHIFT				(2)
#define CSI_HREF_POL_MASK				(0x1U << CSI_HREF_POL_SHIFT)	/*!< Href polarity */
#define CSI_CLK_POL_SHIFT				(1)
#define CSI_CLK_POL_MASK				(0x1U << CSI_CLK_POL_SHIFT)		/*!< data clock type */
typedef enum {
	CSI_POL_NEGATIVE,    /*!< Negative*/
	CSI_POL_POSITIVE,    /*!< Positive*/
}CSI_SignalPol;

#define CSI_SYNC_TYPE_SHIFT				(0)
#define CSI_SYNC_TYPE_MASK				(0x1U << CSI_SYNC_TYPE_SHIFT)	/*!< sync type */
typedef enum {
	CSI_SYNC_SEPARARE,    /*!< Negative*/
	CSI_SYNC_CCIR656,    /*!< Positive*/
}CSI_SyncType;

/*
 * Bits definition for CSI capture control register (0x0008)
 */
#define CSI_C0_FRATE_HALF_SHIFT			(6) /*!< frame rate half down */
#define CSI_C0_FRATE_HALF_MASK			(0x1U << CSI_C0_FRATE_HALF_SHIFT)

#define CSI_C0_FRAME_MASK_SHIFT			(2)
#define CSI_C0_FRAME_MASK_MASK			(0xFU << CSI_C0_FRAME_MASK_SHIFT)	/*!< frame number masked before capture */

#define CSI_C0_VCAP_EN					HAL_BIT(1) /*!< video capture control:  capture the video image data stream on channel 0 */
#define CSI_C0_SCAP_EN					HAL_BIT(0) /*!< still capture control:  capture a single still image frame on channel 0 */
typedef enum {
	CSI_CAP_STILL,
	CSI_CAP_VIDEO,
} CSI_CapType;

/*
 * Bits definition for CSI signal status register (0x000c)
 */
#define CSI_SIGNAL_PCLK_STA_SHIFT		(24)
#define CSI_SIGNAL_PCLK_MASK			(0xFU << CSI_SIGNAL_PCLK_STA_SHIFT)
#define CSI_SIGNAL_DATA_STA_SHIFT		(0)
#define CSI_SIGNAL_DATA_MASK			(0xFFFFFFU << CSI_SIGNAL_DATA_STA_SHIFT)

/*
 * Bits definition for CSI horizontal size register (0x0010)
 */
#define CSI_HOR_LEN_SHIFT				(16)
#define CSI_HOR_LEN_MASK				(0x3FFFU << CSI_HOR_LEN_SHIFT) /*!< horizontal pixel unit length. valid pixel of a line */
#define CSI_HOR_START_SHIFT				(0)
#define CSI_HOR_START_MASK				(0x3FFFU << CSI_HOR_START_SHIFT) /*!< horizontal pixel unit start. pixel is valid from this pixel */

/*
 * Bits definition for CSI vertical size register (0x0014)
 */
#define CSI_VER_LEN_SHIFT				(16)
#define CSI_VER_LEN_MASK				(0x1FFFU << CSI_VER_LEN_SHIFT) /*!< valid line number of a line */
#define CSI_VER_START_SHIFT				(0)
#define CSI_VER_START_MASK				(0x1FFFU << CSI_VER_START_SHIFT) /*!< vertical line start. data is valid from this line */

/*
 * Bits definition for CSI input size register (0x0018)
 */
#define CSI_C0_INPUT_SIZE_Y_SHIFT		(16)
#define CSI_C0_INPUT_SIZE_Y_MASK		(0x1FFFU << CSI_C0_INPUT_SIZE_Y_SHIFT)
#define CSI_C0_INPUT_SIZE_X_SHIFT		(16)
#define CSI_C0_INPUT_SIZE_X_MASK		(0x1FFFU << CSI_C0_INPUT_SIZE_X_SHIFT)

/*
 * Bits definition for CSI interrupt enable register (0x001c)
 */
#define CSI_C0_INT_FRAME_END_EN			HAL_BIT(1)
#define CSI_C0_INT_INPUT_SIZE_CHG_EN	HAL_BIT(0)

/*
 * Bits definition for CSI interrupt status register (0x0020)
 */
#define CSI_C0_INT_STA_FRM_END_PD		HAL_BIT(1)
#define CSI_C0_INT_STA_INPUT_SIZE_CHG	HAL_BIT(0)
typedef enum {
	CSI_IRQ_FRM_END,
	CSI_IRQ_INPUT_SIZE_CHG,
	CSI_IRQ_NONE,
} CSI_IRQState;

/** @brief Type define of CSI interrupt callback function */
typedef void (*CSI_IRQCallback)(void *arg);

/**
 * @brief CSI initialization parameters
 */
typedef struct {
	CSI_IRQCallback	cb;
} CSI_InitParam;

typedef enum {
	CSI_STATE_INVALID	= 0,
	CSI_STATE_INIT		= 1, /* Initializing		*/
	CSI_STATE_DEINIT	= 2, /* Deinitializing	*/
	CSI_STATE_READY		= 3,
	CSI_STATE_BUSY		= 4
} CSI_State;

/**
  * @brief Enable or disable func.
  */
typedef enum {
	CSI_DISABLE,    /*!< Enable*/
	CSI_ENABLE,     /*!< Disable*/
} CSI_CTRL;

typedef struct {
	CSI_YUV420Mask 		yuv420_mask;
	CSI_OutputMode 		out_mode;
	CSI_YUV420LineOrder yuv420_line_order;
	CSI_InputSeq		input_seq;
	CSI_InputFmt		input_fmt;
	CSI_SignalPol		vref_pol;
	CSI_SignalPol		href_pol;
	CSI_SignalPol		clk_pol;
	CSI_SyncType		sync_type;

	uint16_t			hor_len;
	uint16_t			hor_start;
	uint16_t			ver_len;
	uint16_t			ver_start;

	uint8_t 			frame_half_down;
	uint8_t				frame_mask;
} CSI_ConfigParam;

////////////////////////////////////////////////////////////////////////////////////////////

/**
  * @brief The register for JPEG.
  */
typedef struct {
	__IO uint32_t VE_MODE_REG;			/*!< 0x00 JPEG VE mode control register */
	__IO uint32_t VE_RESET_REG; 		/*!< 0x04 JPEG VE reset control register */
	__IO uint32_t ENC_COUNTER_REG;		/*!< 0x08 JPEG VE counter control register */
	__IO uint32_t ENC_OVERTIME_REG; 	/*!< 0x0C JPEG VE overtime register */
		 uint32_t RESERVED0[3];
	__IO uint32_t VE_INT_STA_REG;		/*!< 0x1C JPEG interrupt status register */
	__IO uint32_t CSI_OUTPUT_ADDR_Y;	/*!< 0x20 JPEG CSI output Y address register */
	__IO uint32_t CSI_OUTPUT_ADDR_UV;	/*!< 0x24 JPEG CSI output UV address register */
	__IO uint32_t CSI_OUTPUT_STRIDE;	/*!< 0x28 JPEG CSI output stride control register */

		 uint32_t RESERVED1[245];
	__IO uint32_t INPUT_PIC_SIZE;		/*!< 0x400 JPEG input size register */
	__IO uint32_t JPE_STRIDE_CTRL;		/*!< 0x404 JPEG input stride control register */
		 uint32_t RESERVED2[3];
	__IO uint32_t JPE_STRIDE_CTRL_1;	/*!< 0x414 JPEG input stride1 control register */
		 uint32_t RESERVED3[24];
	__IO uint32_t JPE_INPUT_ADDR_Y; 	/*!< 0x478 JPEG input Y address register */
	__IO uint32_t JPE_INPUT_ADDR_C; 	/*!< 0x47C JPEG input UV address register */

		 uint32_t RESERVED4[225];
	__IO uint32_t JPEG_PARA0_REG;		/*!< 0x804 JPEG para control register */
	__IO uint32_t JPEG_BITRATE_CTRL;	/*!< 0x808 JPEG bitrate control registe */
		 uint32_t RESERVED5[2];
	__IO uint32_t VE_INT_EN_REG;		/*!< 0x814 JPEG VE interrupt enable register */
	__IO uint32_t VE_START_REG; 		/*!< 0x818 JPEG start trigger register */
	__IO uint32_t VE_STA_REG;			/*!< 0x81C JPEG VE status register */
	__IO uint32_t VE_PUTBITS_DATA;		/*!< 0x820 JPEG putbits data register */
	__IO uint32_t MELEVEL_OVERTIME; 	/*!< 0x824 JPEG macroblock level overtime register */
		 uint32_t RESERVED6[22];
	__IO uint32_t OUTSTM_START_ADDR;	/*!< 0x880 JPEG output stream start address register */
	__IO uint32_t OUTSTM_END_ADDR;		/*!< 0x884 JPEG output stream end address register */
	__IO uint32_t OUTSTM_OFFSET;		/*!< 0x888 JPEG output stream offset register */
	__IO uint32_t OUTSTM_VSIZE; 		/*!< 0x88C JPEG output stream valid size */
	__IO uint32_t HARDWARE_OFFSET;		/*!< 0x890 JPEG output stream length register */
		 uint32_t RESERVED7[19];
	__IO uint32_t QM_INDEX; 			/*!< 0x8E0 JPEG VE quantiser matrix index register */
	__IO uint32_t QM_DATA;				/*!< 0x8E4 JPEG VE quantiser matrix input data register */
} JPEG_T;

#define	JPEG	((JPEG_T *)JPEG_BASE)

/*
 * Bits definition for JPEG VE mode control register (0x00)
 */
#define JPEG_MEM_PART_TAKE			HAL_BIT(17)
#define JPEG_MEM_PART_MODE			HAL_BIT(16)

#define JPEG_MEM_PART_NUM_SHIFT		(14)
#define JPEG_MEM_PART_NUM_MASK		(0x3U << JPEG_MEM_PART_NUM_SHIFT)
typedef enum {
	JPEG_MEM_BLOCK2,
	JPEG_MEM_BLOCK4,
	JPEG_MEM_BLOCK8,
} JPEG_MemPartNum;

#define JPEG_AHB_WRITE_BL_8W		HAL_BIT(13)
#define JPEG_AHB_READ_BL_8W			HAL_BIT(12)
#define JPEG_HEIGHT_HALF			HAL_BIT(11)
#define JPEG_WIDTH_HALF				HAL_BIT(10)
#define JPEG_ONLINE_MODE_EN			HAL_BIT(9)	/*!< 0: offline mode 1: online mode */
#define JPEG_INPUT_FMT				HAL_BIT(8)	/*!< 0: JPEG input data format is yuv420 data(NV12) 1: jpeg data */
#define JPEG_ENC_CLK_EN				HAL_BIT(7)
#define JPEG_JPE_CLK_EN				HAL_BIT(6)
#define JPEG_CLK_GATING_DISEN		HAL_BIT(5)

/*
 * Bits definition for JPEG VE reset control register (0x04)
 */
#define JPEG_VE_RESET				HAL_BIT(0)

/*
 * Bits definition for JPEG VE counter control register (0x08)
 */
#define JPEG_VE_CNT_EN				HAL_BIT(31)

#define JPEG_VE_CNT_SHIFT			(0)
#define JPEG_VE_CNT_MASK			(0x7FFFFFFFU << JPEG_VE_CNT_SHIFT)

/*
 * Bits definition for JPEG VE overtime register (0x0c)
 */
#define JPEG_VE_OVERTIME_SHIFT		(8)
#define JPEG_VE_OVERTIME_MASK		(0x7FFFFFU << JPEG_VE_CNT_SHIFT)

/*
 * Bits definition for JPEG interrupt status register (0x1c)
 */
#define JPEG_AHB_SYNC_IDLE			HAL_BIT(25)
#define JPEG_CSI_WB_FINISH			HAL_BIT(24)
#define JPEG_FIFO_OVERFLOW			HAL_BIT(23)
#define JPEG_CSI_ERROR				HAL_BIT(22)
#define JPEG_CSI_TIMEOUT			HAL_BIT(21)
#define JPEG_MEM_PART_OVERFLOW		HAL_BIT(10)
#define JPEG_MEM_PART_INT			HAL_BIT(9)
#define JPEG_VE_TIMEOUT_EN			HAL_BIT(8)
#define JPEG_MB_OVERTIME			HAL_BIT(7)
#define JPEG_BS_STALL				HAL_BIT(6)
#define JPEG_ENC_FINISH				HAL_BIT(3)
#define JPEG_CSI_FRAME_END			HAL_BIT(2)
#define JPEG_CSI_SIZE_CHG			HAL_BIT(1)
#define JPEG_VE_TIMEOUT				HAL_BIT(0)

/*
 * Bits definition for JPEG CSI output Y address register (0x20)
 */

/*
 * Bits definition for JPEG CSI output UV address register (0x24)
 */

/*
 * Bits definition for JPEG CSI output stride control register (0x28)
 */
#define JPEG_CSI_OUT_Y_STRIDE_DIV16_SHIFT			(16)
#define JPEG_CSI_OUT_Y_STRIDE_DIV16_MASK			(0x7FFU << JPEG_CSI_OUT_Y_STRIDE_DIV16_SHIFT)

#define JPEG_CSI_OUT_UV_STRIDE_DIV8_SHIFT			(0)
#define JPEG_CSI_OUT_UV_STRIDE_DIV8_MASK			(0xFFFU << JPEG_CSI_OUT_UV_STRIDE_DIV8_SHIFT)

/*
 * Bits definition for JPEG input size register (0x400)
 */
#define JPEG_PIC_WIDTH_IN_8X8B_SHIFT			(16)
#define JPEG_PIC_WIDTH_IN_8X8B_MASK				(0x7FFU << JPEG_PIC_WIDTH_IN_8X8B_SHIFT)

#define JPEG_PIC_HEIGHT_IN_8X8B_SHIFT			(0)
#define JPEG_PIC_HEIGHT_IN_8X8B_MASK			(0x7FFU << JPEG_PIC_HEIGHT_IN_8X8B_SHIFT)

/*
 * Bits definition for JPEG input stride control register (0x404)
 */
#define JPEG_IN_STRIDE_DIV16_SHIFT				(16)
#define JPEG_IN_STRIDE_DIV16_MASK				(0x7FFU << JPEG_IN_STRIDE_DIV16_SHIFT)

/*
 * Bits definition for JPEG input stride1 control register (0x414)
 */
#define JPEG_IN_C_STRIDE_DIV8_SHIFT				(0)
#define JPEG_IN_C_STRIDE_DIV8_MASK				(0xFFFU << JPEG_IN_C_STRIDE_DIV8_SHIFT)

/*
 * Bits definition for JPEG input Y address register (0x478)
 */
//16B align

/*
 * Bits definition for JPEG input UV address register (0x47c)
 */
//8B align

/*
 * Bits definition for JPEG para control register (0x804)
 */
#define JPEG_STUFF_ZERO_EN						HAL_BIT(30)

#define JPEG_DC_CHRO_SHIFT						(16)
#define JPEG_DC_CHRO_MASK						(0x7FFU << JPEG_DC_CHRO_SHIFT)

#define JPEG_DC_LUMA_SHIFT						(0)
#define JPEG_DC_LUMA_MASK						(0x7FFU << JPEG_DC_LUMA_SHIFT)

/*
 * Bits definition for JPEG bitrate control register (0x808)
 */
#define JPEG_RUN_LENGTH_OPT_DISEN				HAL_BIT(31)
#define JPEG_COEF_DOWN_SAMPLE_DISEN				HAL_BIT(30)

#define JPEG_RUN_LENGTH_TH_SHIFT				(8)
#define JPEG_RUN_LENGTH_TH_MASK					(0xFU << JPEG_RUN_LENGTH_TH_SHIFT)

#define JPEG_CLASSFY_TH_SHIFT					(0)
#define JPEG_CLASSFY_TH_MASK					(0xFFU << JPEG_CLASSFY_TH_SHIFT)

/*
 * Bits definition for JPEG VE interrupt enable register (0x814)
 */
#define JPEG_MB_X_CURR_SHIFT					(24)
#define JPEG_MB_X_CURR_MASK						(0xFU << JPEG_MB_X_CURR_SHIFT)

#define JPEG_MB_Y_CURR_SHIFT					(16)
#define JPEG_MB_Y_CURR_MASK						(0xFU << JPEG_MB_Y_CURR_SHIFT)

#define JPEG_OVERTIME_INT_EN					HAL_BIT(2)
#define JPEG_BS_STALL_INT_EN					HAL_BIT(1)
#define JPEG_VE_FINISH_INT_EN					HAL_BIT(0)

/*
 * Bits definition for JPEG start trigger register (0x818)
 */
#define JPEG_PUTBITS_LEN_SHIFT					(8)
#define JPEG_PUTBITS_LEN_MASK					(0x3FU << JPEG_PUTBITS_LEN_SHIFT)

#define JPEG_VE_ENC_START_SHIFT					(0)	/*!< 1000: VE encode start */
#define JPEG_VE_ENC_START_MASK					(0xFU << JPEG_VE_ENC_START_SHIFT)

/*
 * Bits definition for JPEG VE status register (0x81c)
 */
#define JPEG_PIC_ENC_BUSY						HAL_BIT(31)
#define JPEG_READ_MB_BUSY						HAL_BIT(23)
#define JPEG_IPTIT_BUSY							HAL_BIT(13)
#define JPEG_VLC_BUSY							HAL_BIT(12)
#define JPEG_PUTBITS_STATUS						HAL_BIT(9)
#define JPEG_ENC_MEM_BUSY						HAL_BIT(7)
#define JPEG_ENC_ISP							HAL_BIT(6)

/*
 * Bits definition for JPEG putbits data register (0x820)
 */

/*
 * Bits definition for JPEG macroblock level overtime register (0x824)
 */
#define JPEG_MBOVERTIME_EN						HAL_BIT(15)

#define JPEG_MBOVERTIME_THRESHOLD_SHIFT			(12)
#define JPEG_MBOVERTIME_THRESHOLD_MASK			(0x7U << JPEG_MBOVERTIME_THRESHOLD_SHIFT)

/*
 * Bits definition for JPEG output stream start address register (0x880)
 */

/*
 * Bits definition for JPEG output stream end address register (0x884)
 */

/*
 * Bits definition for JPEG output stream offset register (0x888)
 */
#define JPEG_STREAM_OFFSET_SHIFT				(0)
#define JPEG_STREAM_OFFSET_MASK					(0xFFFFFFFU << JPEG_STREAM_OFFSET_SHIFT)

/*
 * Bits definition for JPEG output stream valid size register (0x88c)
 */
#define JPEG_STREAM_VALID_SIZE_SHIFT			(16)
#define JPEG_STREAM_VALID_SIZE_MASK				(0xFFFU << JPEG_STREAM_VALID_SIZE_SHIFT)

/*
 * Bits definition for JPEG output stream length register (0x890)
 */
#define JPEG_OUTSTM_LEN_SHIFT					(0)
#define JPEG_OUTSTM_LEN_MASK					(0xFFFFFFFU << JPEG_STREAM_OFFSET_SHIFT)

/*
 * Bits definition for JPEG VE quantiser matrix index register (0x8e0)
 */
#define JPEG_QM_INDEX_SHIFT						(0)
#define JPEG_QM_INDEX_MASK						(0x7FU << JPEG_QM_INDEX_SHIFT)

/*
 * Bits definition for JPEG VE quantiser matrix input data register (0x8e4)
 */

typedef struct {
	uint8_t 		jpeg_mode;
	uint8_t 		sensor_out_type;
	uint8_t 		top_clk_en;
	uint8_t 		jpe_clk_en;

	uint32_t		csi_output_addr_y;
	uint32_t		csi_output_addr_uv;

	uint32_t		pic_size_width;
	uint32_t		pic_size_height;

	uint32_t		jpe_input_addr_y;
	uint32_t		jpe_input_addr_uv;

	uint8_t 		marvolvl_ovtime_int_en;
	uint8_t 		bitstream_stall_int_en;
	uint8_t 		ve_finish_int_en;

	uint32_t		outstream_start_addr;
	uint32_t		outstream_end_addr;
	uint32_t		outstream_offset;
	uint32_t		outstream_mem_size;

	uint32_t		quality;

	uint8_t			jpeg_en;
	uint8_t 		mem_part_en;
	JPEG_MemPartNum mem_part_num;
	uint8_t			*mem_part_buf;
} JPEG_ConfigParam;

typedef enum {
	JPEG_MOD_OFFLINE,
	JPEG_MOD_ONLINE,
} JPEG_Mode;

typedef enum {
	JPEG_MEM_PART_DIS,
	JPEG_MEM_PART_EN,
} JPEG_MemPartMode;

HAL_Status HAL_CSI_JPEG_Init(CSI_InitParam *param);
HAL_Status HAL_CSI_JPEG_Deinit(void);
HAL_Status HAL_CSI_Config(CSI_ConfigParam *cfg);

HAL_Status HAL_CSI_StartCapture(CSI_CapType mode);
HAL_Status HAL_CSI_StopCapture(void);

HAL_Status HAL_JPEG_Config(JPEG_ConfigParam *cfg);
void HAL_JPEG_Scale(uint8_t height_scale, uint8_t width_scale);
void HAL_JPEG_Reset(void);
void HAL_JPEG_WriteHeader(uint8_t *baseAddr);

#endif /* (__CONFIG_CHIP_ARCH_VER == 2) */

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_CSI_JPEG_H_ */

