/**
  * @file  hal_csi.c
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

#if (__CONFIG_CHIP_ARCH_VER == 2)

#include "hal_base.h"
#include "driver/chip/hal_gpio.h"

#include "sys/io.h"
#include "driver/chip/hal_sysctl.h"
#include "driver/chip/hal_csi_jpeg.h"

#include "jpeg/jpegenc.h"

#define CSI_JPEG_DBG_ON		1
#define CSI_JPEG_ERR_ON		1

#if (CSI_JPEG_DBG_ON == 1)
#define CSI_JPEG_DBG(fmt, arg...)    HAL_LOG(CSI_JPEG_DBG_ON, "[CSI_JPEG] "fmt, ##arg)
#else
#define CSI_JPEG_DBG(fmt, arg...)
#endif

#if (CSI_JPEG_ERR_ON == 1)
#define CSI_JPEG_ERR(fmt, arg...)    HAL_LOG(CSI_JPEG_ERR_ON, "[CSI_JPEG] "fmt, ##arg)
#else
#define CSI_JPEG_ERR(fmt, arg...)
#endif

#define CSI_JPEG_REG_DBG_ON			1

#define JPEG_SRAM_SWITCH_THR_W		(720)
#define JPEG_SRAM_SWITCH_THR_H		(576)

#define	VE_COUNT_THRESHOLD 			10
#define	MEM_PARTITION_EN 			1

typedef struct {
	CSI_CapType 		capMode;
	JPEG_Mode			encMode;
	uint8_t * 			jpgOutStmAddr;
	uint32_t 			jpgOutStmSize;
	uint32_t			jpgMemSize;

	uint8_t				memPartEn;
	uint8_t 			memPartNum;
	uint32_t			memPartSize;
	uint32_t			memPartCnt;
	uint32_t			memPartOffSet;
	uint32_t			memCurSize;
	uint8_t 			*memPartDstBuf;

	uint8_t 			jpgVeEn;
	uint8_t 			capRun;
	CSI_State 			state;
	CSI_IRQCallback		cb;

	uint32_t 			ve_finish_count;

	JpegCtx 			*jpegCtx;
} CSI_JPEG_Priv;

static CSI_JPEG_Priv gCsiJpegPriv;

static void CSI_JPEG_IRQHandler(void)
{
	CSI_JPEG_Priv *priv = &gCsiJpegPriv;

	uint32_t csi_irq=0, jpe_irq=0;
	uint8_t ve_overrun = 0;

	csi_irq = CSI->CSI_C0_INT_STA_REG;
	jpe_irq = JPEG->VE_INT_STA_REG;

	CSI_JPEG_DBG("csi:%x jpe:%x\n",csi_irq, jpe_irq);

	if (jpe_irq & (0xE00182)) {
		CSI_JPEG_ERR("excption\n");
		priv->state = CSI_STATE_INVALID;
		HAL_CLR_BIT(CSI->CSI_CAP_REG, CSI_C0_VCAP_EN | CSI_C0_SCAP_EN);
		priv->cb(NULL);
		CSI->CSI_C0_INT_STA_REG = csi_irq;
		JPEG->VE_INT_STA_REG = jpe_irq;
		return;
	}

	if (csi_irq & 0x02) {
		CSI_JPEG_DBG("f\n");
		if (priv->jpgVeEn && priv->encMode == JPEG_MOD_OFFLINE && priv->capRun) {
			JPEG->OUTSTM_OFFSET = 0;
			JPEG->OUTSTM_START_ADDR = (uint32_t)priv->jpgOutStmAddr;
			JPEG->OUTSTM_END_ADDR = (uint32_t)priv->jpgOutStmAddr+ (priv->jpgMemSize-1);
			JPEG->VE_START_REG = 0x08;
		}
		else if (!priv->jpgVeEn || (priv->capMode == CSI_CAP_STILL && !priv->capRun)) {
			priv->cb(NULL);
		}

		if (!priv->jpgVeEn && (priv->capMode == CSI_CAP_STILL || !priv->capRun))
			priv->state = CSI_STATE_READY;
		if (priv->encMode == JPEG_MOD_OFFLINE && !priv->capRun)
			priv->state = CSI_STATE_READY;
	}

#if MEM_PARTITION_EN
	uint32_t cur_part_len;

	if (jpe_irq & JPEG_MEM_PART_INT) {
		CSI_JPEG_DBG("part\n");
		cur_part_len = (priv->memPartCnt + 1) * priv->memPartSize - priv->memPartOffSet;

		memcpy(priv->memPartDstBuf + priv->memCurSize, priv->jpgOutStmAddr + priv->memPartOffSet, cur_part_len);
		priv->memPartOffSet += cur_part_len;
		priv->memCurSize += cur_part_len;

		priv->memPartCnt++;
		if (priv->memPartCnt >= priv->memPartNum) {
			priv->memPartCnt = 0;
			priv->memPartOffSet = 0;
		}

		HAL_SET_BIT(JPEG->VE_MODE_REG, 0x20000);
	}
#endif

	if (jpe_irq & 0x08 && priv->jpgVeEn) {
		CSI_JPEG_DBG("ve\n");
		priv->ve_finish_count ++;
		if (priv->ve_finish_count >= VE_COUNT_THRESHOLD) {
			ve_overrun = 1;
			CSI_JPEG_ERR("overrun\n");
		}

		uint32_t encode_len = ((JPEG->HARDWARE_OFFSET)+7)/8;//unit is bytes
	    encode_len = (encode_len+3)/4*4;

#if MEM_PARTITION_EN
		if (encode_len >= priv->memCurSize) {
			cur_part_len = encode_len - priv->memCurSize;

			memcpy(priv->memPartDstBuf + priv->memCurSize, priv->jpgOutStmAddr + priv->memPartOffSet, cur_part_len);
			priv->memCurSize = 0;
			priv->memPartOffSet += cur_part_len;
			priv->memPartOffSet = ALIGN_32B(priv->memPartOffSet);
		}
#endif
		if (priv->encMode == JPEG_MOD_ONLINE) {
			#if 0
			if (priv->capMode == CSI_CAP_VIDEO) {
				priv->jpgOutStmAddr = last_addr;

				if (last_addr == base_addr) {
					last_addr = base_addr + 40960;// 30720;
				} else {
					last_addr = base_addr;
				}
			}
			#endif
			JPEG->OUTSTM_OFFSET = 0;
			JPEG->OUTSTM_START_ADDR = (uint32_t)priv->jpgOutStmAddr;
#if !MEM_PARTITION_EN
			JPEG->OUTSTM_END_ADDR = (uint32_t)priv->jpgOutStmAddr+ (priv->jpgMemSize - 1);
#endif
			if (priv->capRun && !ve_overrun)
				JPEG->VE_START_REG = 0x08;
		}

		if (priv->cb)
			priv->cb(&encode_len);

		priv->ve_finish_count--;

		if (!priv->capRun || priv->capMode == CSI_CAP_STILL || ve_overrun) {
			priv->state = CSI_STATE_READY;
			HAL_CLR_BIT(CSI->CSI_CAP_REG, CSI_C0_VCAP_EN | CSI_C0_SCAP_EN);
		}
	}

	CSI->CSI_C0_INT_STA_REG = csi_irq;
	JPEG->VE_INT_STA_REG = jpe_irq;
}

__STATIC_INLINE void CSI_EnableIRQ(void)
{
	//HAL_SET_BIT(CSI->CSI_C0_INT_EN_REG, CSI_C0_INT_STA_FRM_END_PD | CSI_C0_INT_STA_INPUT_SIZE_CHG);
	HAL_SET_BIT(CSI->CSI_C0_INT_EN_REG, CSI_C0_INT_STA_FRM_END_PD);
}

__STATIC_INLINE void CSI_DisableIRQ(void)
{
	HAL_CLR_BIT(CSI->CSI_C0_INT_EN_REG, CSI_C0_INT_STA_FRM_END_PD | CSI_C0_INT_STA_INPUT_SIZE_CHG);
}

static void CSI_EnableCSI(void)
{
	HAL_SET_BIT(CSI->CSI_EN_REG, CSI_PCLK_EN | CSI_NCSIC_EN | CSI_PRS_EN);
}

static void CSI_DisableCSI(void)
{
	HAL_CLR_BIT(CSI->CSI_EN_REG, CSI_PCLK_EN | CSI_NCSIC_EN | CSI_PRS_EN);
}

__STATIC_INLINE HAL_Status CSI_PINS_Init()
{
	return HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_CSI, 0), 0);
}

__STATIC_INLINE HAL_Status CSI_PINS_Deinit()
{
	return HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_CSI, 0), 0);
}

HAL_Status HAL_CSI_JPEG_Init(CSI_InitParam *param)
{
	if (!param)
		return HAL_INVALID;

	CSI_JPEG_Priv *priv = &gCsiJpegPriv;

	if (priv->state != CSI_STATE_INVALID)
		return HAL_ERROR;

	HAL_Memset(priv, 0, sizeof(*priv));

	priv->jpegCtx = JpegEncCreate();
	if (!priv->jpegCtx) {
		CSI_JPEG_ERR("jpeg encode create fail\n");
		return HAL_ERROR;
	}

	CSI_PINS_Init();

	/*init and enable clk*/
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_CSI_JPEG);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_CSI_JPEG);

	HAL_CCM_CSI_JPEG_SetDevClock(CCM_AHB_PERIPH_CLK_SRC_DEVCLK, CCM_PERIPH_CLK_DIV_N_1, CCM_PERIPH_CLK_DIV_M_2);
	HAL_CCM_CSI_JPEG_EnableDevClock();

	//HAL_CCM_CSI_SetMClock(CCM_AHB_PERIPH_CLK_SRC_DEVCLK, CCM_PERIPH_CLK_DIV_N_1, CCM_PERIPH_CLK_DIV_M_1);
	HAL_CCM_CSI_SetMClock(CCM_AHB_PERIPH_CLK_SRC_DEVCLK, CCM_PERIPH_CLK_DIV_N_1, CCM_PERIPH_CLK_DIV_M_8);
	HAL_CCM_CSI_EnableMClock();

	priv->cb = param->cb;
	CSI_EnableIRQ();
	/* enable NVIC IRQ */
	HAL_NVIC_ConfigExtIRQ(CSI_JPEG_IRQn, CSI_JPEG_IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);

#if CSI_JPEG_REG_DBG_ON
	CSI_JPEG_DBG("CCM->BUS_PERIPH_CLK_CTRL: 0x%x\n", CCM->BUS_PERIPH_CLK_CTRL);
	CSI_JPEG_DBG("CCM->BUS_PERIPH_RST_CTRL: 0x%x\n", CCM->BUS_PERIPH_RST_CTRL);
	CSI_JPEG_DBG("CCM->CSI_MCLK_CTRL: 0x%x\n", CCM->CSI_MCLK_CTRL);
	CSI_JPEG_DBG("CCM->CSI_JPE_DEV_CLK_CTRL: 0x%x\n", CCM->CSI_JPE_DEV_CLK_CTRL);
#endif

	priv->state = CSI_STATE_INIT;

	return HAL_OK;
}

HAL_Status HAL_CSI_JPEG_Deinit(void)
{
	CSI_JPEG_Priv *priv = &gCsiJpegPriv;

	if (priv->state != CSI_STATE_INIT && priv->state != CSI_STATE_READY)
		return HAL_ERROR;

	if (!priv->jpegCtx)
		JpegEncDestory(priv->jpegCtx);

	CSI_PINS_Deinit();

	HAL_CCM_CSI_DisableMClock();
	HAL_CCM_CSI_JPEG_DisableDevClock();
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_CSI_JPEG);

	priv->state = CSI_STATE_INVALID;

	return HAL_OK;
}

HAL_Status HAL_CSI_Config(CSI_ConfigParam *cfg)
{
	uint32_t reg_val;

	CSI_JPEG_Priv *priv = &gCsiJpegPriv;
	if (priv->state != CSI_STATE_INIT && priv->state != CSI_STATE_READY)
		return HAL_ERROR;

	priv->state = CSI_STATE_READY;

	CSI_DisableCSI();

	reg_val = ((cfg->yuv420_mask		& 0x1) << CSI_YUV420_MASK_SHIFT)		|\
			  ((cfg->out_mode			& 0x1) << CSI_OUTPUT_MODE_SHIFT)		|\
			  ((cfg->yuv420_line_order  & 0x1) << CSI_YUV420_LINE_ORDER_SHIFT)	|\
			  ((cfg->input_seq  		& 0x3) << CSI_INPUT_SEQ_SHIFT)			|\
			  ((cfg->input_fmt  		& 0x3) << CSI_INPUT_FMT_SHIFT)			|\
			  ((cfg->vref_pol  			& 0x1) << CSI_VREF_POL_SHIFT)			|\
			  ((cfg->href_pol  			& 0x1) << CSI_HREF_POL_SHIFT)			|\
			  ((cfg->clk_pol  			& 0x1) << CSI_CLK_POL_SHIFT)			|\
			  ((cfg->sync_type  		& 0x1) << CSI_SYNC_TYPE_SHIFT);
	CSI->CSI_CFG_REG = reg_val;


	reg_val = ((cfg->hor_len			& 0x3fff) << CSI_HOR_LEN_SHIFT)			|\
			  ((cfg->hor_start			& 0x3fff) << CSI_HOR_START_SHIFT);
	CSI->CSI_C0_HSIZE_REG = reg_val;

	reg_val = ((cfg->ver_len			& 0x1fff) << CSI_VER_LEN_SHIFT)			|\
			  ((cfg->ver_start			& 0x1fff) << CSI_VER_START_SHIFT);
	CSI->CSI_C0_VSIZE_REG = reg_val;


	reg_val = ((cfg->frame_half_down	& 0x1) << CSI_C0_FRATE_HALF_SHIFT)		|\
			  ((cfg->frame_mask			& 0xf) << CSI_C0_FRAME_MASK_SHIFT);
	CSI->CSI_CAP_REG = reg_val;

#if CSI_JPEG_REG_DBG_ON
	CSI_JPEG_DBG("(%p)CSI->CSI_C0_INT_EN_REG: 0x%x\n", (uint32_t*)&CSI->CSI_C0_INT_EN_REG, CSI->CSI_C0_INT_EN_REG);
	CSI_JPEG_DBG("(%p)CSI->CSI_CFG_REG: 0x%x\n", (uint32_t*)&CSI->CSI_CFG_REG, CSI->CSI_CFG_REG);
	CSI_JPEG_DBG("(%p)CSI->CSI_C0_HSIZE_REG: 0x%x\n", (uint32_t*)&CSI->CSI_C0_HSIZE_REG, CSI->CSI_C0_HSIZE_REG);
	CSI_JPEG_DBG("(%p)CSI->CSI_C0_VSIZE_REG: 0x%x\n", (uint32_t*)&CSI->CSI_C0_VSIZE_REG, CSI->CSI_C0_VSIZE_REG);
	CSI_JPEG_DBG("(%p)CSI->CSI_CAP_REG: 0x%x\n", (uint32_t*)&CSI->CSI_CAP_REG, CSI->CSI_CAP_REG);
#endif

	CSI_EnableCSI();

	return HAL_OK;
}

HAL_Status HAL_CSI_StartCapture(CSI_CapType mode)
{
	CSI_JPEG_Priv *priv = &gCsiJpegPriv;

	if (priv->state != CSI_STATE_READY) {
		CSI_JPEG_ERR("invalid state: %d %d\n", priv->state, __LINE__);
		return HAL_ERROR;
	}

	unsigned long flags = HAL_EnterCriticalSection();
	priv->capRun = 1;
	priv->state = CSI_STATE_BUSY;
	priv->capMode = mode;
	HAL_ExitCriticalSection(flags);

	HAL_CLR_BIT(CSI->CSI_CAP_REG, CSI_C0_VCAP_EN | CSI_C0_SCAP_EN);

	if (CSI_CAP_STILL == mode)
		HAL_SET_BIT(CSI->CSI_CAP_REG, CSI_C0_SCAP_EN);
	else
		HAL_SET_BIT(CSI->CSI_CAP_REG, CSI_C0_VCAP_EN);

	if (priv->encMode == JPEG_MOD_ONLINE)
		JPEG->VE_START_REG = 0x08;

#if CSI_JPEG_REG_DBG_ON
	CSI_JPEG_DBG("%s, CSI->CSI_CAP_REG: %08x\n", __func__, CSI->CSI_CAP_REG);
#endif

	return HAL_OK;
}

HAL_Status HAL_CSI_StopCapture(void)
{
	CSI_JPEG_Priv *priv = &gCsiJpegPriv;

	if (priv->state != CSI_STATE_BUSY) {
		CSI_JPEG_ERR("invalid state: %d %d\n", priv->state, __LINE__);
		return HAL_ERROR;
	}

	unsigned long flags = HAL_EnterCriticalSection();
	priv->capRun = 0;
	HAL_ExitCriticalSection(flags);

	while(1) {
		if (priv->state == CSI_STATE_READY ||priv->state == CSI_STATE_INVALID ) {
			break;
		} else {
			OS_MSleep(10);
		}
	}

	CSI_JPEG_DBG("csi capture stop..\n");

	return HAL_OK;
}

static void JPEG_SetQtab(uint32_t quality)
{
	CSI_JPEG_Priv *priv = &gCsiJpegPriv;

	priv->jpegCtx->ctl_ops->setQuantTbl(priv->jpegCtx, quality);

	priv->jpegCtx->dc_value[0] = DefaultDC / priv->jpegCtx->quant_tbl[0][0];
	priv->jpegCtx->dc_value[1] = DefaultDC / priv->jpegCtx->quant_tbl[1][0];
	priv->jpegCtx->dc_value[2] = DefaultDC / priv->jpegCtx->quant_tbl[1][0];
}

static void JPEG_WriteQtab(uint32_t base_addr)
{
	uint8_t i = 0;

	CSI_JPEG_Priv *priv = &gCsiJpegPriv;

	JPEG->QM_INDEX = base_addr;

	uint32_t *tbl = (uint32_t *)&priv->jpegCtx->quant_tbl_aw;
	for (i = 0; i < 128; i++) {
		JPEG->QM_DATA = *tbl++;
	}
}

HAL_Status HAL_JPEG_Config(JPEG_ConfigParam *cfg)
{
	uint32_t reg_val;
	SYSCTL_CSI_JPE_ShareSramType type = SYSCTL_CSI_JPE_SHARE_32K;

	CSI_JPEG_Priv *priv = &gCsiJpegPriv;
	if (priv->state != CSI_STATE_INIT && priv->state != CSI_STATE_READY)
		return HAL_ERROR;

	priv->encMode = cfg->jpeg_mode & 0x1;
	priv->jpgVeEn = cfg->jpeg_en;
	priv->jpgMemSize = cfg->outstream_mem_size;

	priv->jpegCtx->JpgColorFormat = JpgYUV420;
	priv->jpegCtx->quality = cfg->quality;
	priv->jpegCtx->image_height = cfg->pic_size_height;
	priv->jpegCtx->image_width = cfg->pic_size_width;

	if (cfg->pic_size_width > JPEG_SRAM_SWITCH_THR_W ||
				cfg->pic_size_height > JPEG_SRAM_SWITCH_THR_H) {
		type = SYSCTL_CSI_JPE_SHARE_64K;
	}
	HAL_SYSCTL_SetCSIJPEGSramShare(type);

#if MEM_PARTITION_EN
	priv->memPartEn = cfg->mem_part_en;
	if (priv->memPartEn) {
		priv->memPartNum = 1 << (cfg->mem_part_num + 1);
		priv->memPartSize = priv->jpgMemSize / priv->memPartNum;
		priv->memPartOffSet = 0;
		priv->memPartDstBuf = cfg->mem_part_buf;
	}
#endif

	CSI_JPEG_DBG("priv->memPartNum: %d\n", priv->memPartNum);
	CSI_JPEG_DBG("priv->memPartSize: %d\n", priv->memPartSize);

	reg_val = ((cfg->mem_part_en		& 0x1) << 16)		|\
			  ((cfg->mem_part_num		& 0x3) << 14)		|\
			  ((0						& 0x1) << 11)		|\
			  ((0						& 0x1) << 10)		|\
			  ((cfg->jpeg_mode			& 0x1) << 9)		|\
			  ((cfg->sensor_out_type	& 0x1) << 8)		|\
			  ((cfg->top_clk_en  		& 0x1) << 7)		|\
			  ((cfg->jpe_clk_en  		& 0x1) << 6);
	JPEG->VE_MODE_REG = reg_val;

	reg_val = ((cfg->pic_size_width/8	& 0x7ff) << 16)		|\
			  ((cfg->pic_size_height/8	& 0x7ff) << 0);
	JPEG->INPUT_PIC_SIZE = reg_val;

	if (JPEG_MOD_OFFLINE == cfg->jpeg_mode) {
		JPEG->CSI_OUTPUT_ADDR_Y = cfg->csi_output_addr_y;
		JPEG->CSI_OUTPUT_ADDR_UV = cfg->csi_output_addr_uv;

		reg_val = ((cfg->pic_size_width	/16		& 0x7ff) << 16)		|\
			      ((cfg->pic_size_width/16  	& 0x7ff) << 0);
		JPEG->CSI_OUTPUT_STRIDE = reg_val;

		reg_val = ((cfg->pic_size_width /16 	& 0x7ff) << 16);
		JPEG->JPE_STRIDE_CTRL = reg_val;

		reg_val = ((cfg->pic_size_width /16 	& 0xfff) << 0);
		JPEG->JPE_STRIDE_CTRL_1 = reg_val;

		JPEG->JPE_INPUT_ADDR_Y = cfg->jpe_input_addr_y;
		JPEG->JPE_INPUT_ADDR_C = cfg->jpe_input_addr_uv;
	}

	reg_val = ((cfg->marvolvl_ovtime_int_en			& 0x1) << 2)		|\
			  ((cfg->bitstream_stall_int_en			& 0x1) << 1)		|\
			  ((cfg->ve_finish_int_en			    & 0x1) << 0);
	JPEG->VE_INT_EN_REG = reg_val;

	JPEG_SetQtab(cfg->quality);

	JPEG->JPEG_PARA0_REG = 3<<30 | priv->jpegCtx->dc_value[1]<<16 | priv->jpegCtx->dc_value[0];
	JPEG->JPEG_BITRATE_CTRL = 0x00000840;
	JPEG->OUTSTM_OFFSET = cfg->outstream_offset;
	JPEG->OUTSTM_START_ADDR = cfg->outstream_start_addr;
	JPEG->OUTSTM_END_ADDR = cfg->outstream_end_addr;

	priv->jpgOutStmAddr =  (uint8_t*)JPEG->OUTSTM_START_ADDR;

	JPEG_WriteQtab(0x0);

#if CSI_JPEG_REG_DBG_ON
	CSI_JPEG_DBG("(%p)JPEG->VE_MODE_REG: 0x%x\n", (uint32_t*)&JPEG->VE_MODE_REG, JPEG->VE_MODE_REG);
	CSI_JPEG_DBG("(%p)JPEG->INPUT_PIC_SIZE: 0x%x\n", (uint32_t*)&JPEG->INPUT_PIC_SIZE, JPEG->INPUT_PIC_SIZE);
	CSI_JPEG_DBG("(%p)JPEG->VE_INT_EN_REG: 0x%x\n", (uint32_t*)&JPEG->VE_INT_EN_REG, JPEG->VE_INT_EN_REG);
	CSI_JPEG_DBG("(%p)JPEG->CSI_OUTPUT_ADDR_Y: 0x%x\n", (uint32_t*)&JPEG->CSI_OUTPUT_ADDR_Y, JPEG->CSI_OUTPUT_ADDR_Y);
	CSI_JPEG_DBG("(%p)JPEG->CSI_OUTPUT_ADDR_UV: 0x%x\n", (uint32_t*)&JPEG->CSI_OUTPUT_ADDR_UV, JPEG->CSI_OUTPUT_ADDR_UV);
	CSI_JPEG_DBG("(%p)JPEG->CSI_OUTPUT_STRIDE: 0x%x\n", (uint32_t*)&JPEG->CSI_OUTPUT_STRIDE, JPEG->CSI_OUTPUT_STRIDE);
	CSI_JPEG_DBG("(%p)JPEG->JPE_STRIDE_CTRL: 0x%x\n", (uint32_t*)&JPEG->JPE_STRIDE_CTRL, JPEG->JPE_STRIDE_CTRL);
	CSI_JPEG_DBG("(%p)JPEG->JPE_STRIDE_CTRL_1: 0x%x\n", (uint32_t*)&JPEG->JPE_STRIDE_CTRL_1, JPEG->JPE_STRIDE_CTRL_1);
	CSI_JPEG_DBG("(%p)JPEG->JPE_INPUT_ADDR_Y: 0x%x\n", (uint32_t*)&JPEG->JPE_INPUT_ADDR_Y, JPEG->JPE_INPUT_ADDR_Y);
	CSI_JPEG_DBG("(%p)JPEG->JPE_INPUT_ADDR_C: 0x%x\n", (uint32_t*)&JPEG->JPE_INPUT_ADDR_C, JPEG->JPE_INPUT_ADDR_C);
	CSI_JPEG_DBG("(%p)JPEG->JPEG_PARA0_REG: 0x%x\n", (uint32_t*)&JPEG->JPEG_PARA0_REG, JPEG->JPEG_PARA0_REG);
	CSI_JPEG_DBG("(%p)JPEG->JPEG_BITRATE_CTRL: 0x%x\n", (uint32_t*)&JPEG->JPEG_BITRATE_CTRL, JPEG->JPEG_BITRATE_CTRL);
	CSI_JPEG_DBG("(%p)JPEG->OUTSTM_OFFSET: 0x%x\n", (uint32_t*)&JPEG->OUTSTM_OFFSET, JPEG->OUTSTM_OFFSET);
	CSI_JPEG_DBG("(%p)JPEG->OUTSTM_START_ADDR: 0x%x\n", (uint32_t*)&JPEG->OUTSTM_START_ADDR, JPEG->OUTSTM_START_ADDR);
	CSI_JPEG_DBG("(%p)JPEG->OUTSTM_END_ADDR: 0x%x\n", (uint32_t*)&JPEG->OUTSTM_END_ADDR, JPEG->OUTSTM_END_ADDR);
	CSI_JPEG_DBG("(%p)JPEG->VE_START_REG: 0x%x\n", (uint32_t*)&JPEG->VE_START_REG, JPEG->VE_START_REG);
#endif

	priv->state = CSI_STATE_READY;

	return HAL_OK;
}

void HAL_JPEG_Scale(uint8_t height_scale, uint8_t width_scale)
{
	uint32_t reg_val;
	CSI_JPEG_Priv *priv = &gCsiJpegPriv;

	uint32_t width, height;

	if (height_scale && width_scale) {
		width = 320;
		height = 240;
	} else if (!height_scale && !width_scale) {
		width = 640;
		height = 480;
	} else {
		CSI_JPEG_ERR("invalid arg\n");
		return;
	}

	reg_val = ((width/8		& 0x7ff) << 16)		|\
			  ((height/8	& 0x7ff) << 0);
	JPEG->INPUT_PIC_SIZE = reg_val;

	if (JPEG_MOD_OFFLINE == priv->encMode) {
		reg_val = ((width	/16		& 0x7ff) << 16)		|\
			      ((height  /16  	& 0x7ff) << 0);
		JPEG->CSI_OUTPUT_STRIDE = reg_val;

		reg_val = ((width /16 	& 0x7ff) << 16);
		JPEG->JPE_STRIDE_CTRL = reg_val;

		reg_val = ((width /16 	& 0xfff) << 0);
		JPEG->JPE_STRIDE_CTRL_1 = reg_val;
	}

	reg_val = ((height_scale			& 0x1) << 11)		|\
			  ((width_scale				& 0x1) << 10);
	HAL_MODIFY_REG(JPEG->VE_MODE_REG, (1<<11)|(1<<10), reg_val);
}

void HAL_JPEG_Reset(void)
{
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_CSI_JPEG);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_CSI_JPEG);
}

void HAL_JPEG_WriteHeader(uint8_t *baseAddr)
{
	CSI_JPEG_Priv *priv = &gCsiJpegPriv;

	if (baseAddr == NULL) {
		CSI_JPEG_ERR("invalid handle\n");
		return;
	}

	priv->jpegCtx->BaseAddr = (char*)baseAddr;
	priv->jpegCtx->ctl_ops->writeHeader(priv->jpegCtx);
}

#endif
