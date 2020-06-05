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
#include "driver/chip/hal_dcache.h"

#include "sys/io.h"
#include "driver/chip/hal_sysctl.h"
#include "driver/chip/hal_csi_jpeg.h"

#include "jpeg/jpegenc.h"

#define CSI_JPEG_DBG_ON		0
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

#define CSI_JPEG_REG_DBG_ON    		 0
#if (CSI_JPEG_REG_DBG_ON == 1)
#define CSI_JPEG_REG_DBG(REG)		 CSI_JPEG_DBG("(%p): 0x%8x\n", (uint32_t*)&REG, REG)
#else
#define CSI_JPEG_REG_DBG(fmt, arg...)
#endif

#define CSI_REG_MASK				 (0x01)
#define JPEG_REG_MASK				 (0x02)
#define CSI_JPEG_REG_ALL(mask)		 do {			\
	if (mask & CSI_REG_MASK) {						\
		CSI_JPEG_REG_DBG(CSI->CSI_EN_REG);			\
		CSI_JPEG_REG_DBG(CSI->CSI_CFG_REG);			\
		CSI_JPEG_REG_DBG(CSI->CSI_C0_INT_EN_REG);	\
		CSI_JPEG_REG_DBG(CSI->CSI_CAP_REG);			\
		CSI_JPEG_REG_DBG(CSI->CSI_C0_HSIZE_REG);	\
		CSI_JPEG_REG_DBG(CSI->CSI_C0_VSIZE_REG);	\
	}												\
	if (mask & JPEG_REG_MASK) {						\
		CSI_JPEG_REG_DBG(JPEG->VE_MODE_REG);		\
		CSI_JPEG_REG_DBG(JPEG->INPUT_PIC_SIZE);		\
		CSI_JPEG_REG_DBG(JPEG->VE_INT_EN_REG);		\
		CSI_JPEG_REG_DBG(JPEG->VE_MODE_REG);		\
		CSI_JPEG_REG_DBG(JPEG->CSI_OUTPUT_ADDR_Y);	\
		CSI_JPEG_REG_DBG(JPEG->CSI_OUTPUT_ADDR_UV);	\
		CSI_JPEG_REG_DBG(JPEG->JPE_INPUT_ADDR_Y);	\
		CSI_JPEG_REG_DBG(JPEG->JPE_INPUT_ADDR_C);	\
		CSI_JPEG_REG_DBG(JPEG->JPEG_PARA0_REG);		\
		CSI_JPEG_REG_DBG(JPEG->JPEG_BITRATE_CTRL);	\
		CSI_JPEG_REG_DBG(JPEG->OUTSTM_OFFSET);		\
		CSI_JPEG_REG_DBG(JPEG->OUTSTM_START_ADDR);	\
		CSI_JPEG_REG_DBG(JPEG->OUTSTM_END_ADDR);	\
		CSI_JPEG_REG_DBG(JPEG->VE_START_REG);		\
	}												\
} while (0)

#define JPEG_SRAM_SWITCH_THR_W		(720)
#define JPEG_SRAM_SWITCH_THR_H		(576)

typedef struct {
	CSI_CapType 		capMode;
	JPEG_Mode			encMode;
	uint8_t				jpgOutBufId;
	uint8_t * 			jpgOutStmAddr[JPEG_BUFF_CNT_MAX];
	uint8_t 			jpgOutBufNum;
	uint32_t 			jpgOutStmSize;
	uint32_t			jpgMemSize;
	uint32_t			jpgOutStmOffset;

	uint8_t				memPartEn;
	uint8_t 			memPartNum;
	uint32_t			memPartSize;
	uint32_t			memPartCnt;
	uint32_t			memPartOffSet;
	uint32_t			memCurSize;

	uint8_t 			jpgVeEn;
	uint8_t 			capRun;
	CSI_State 			state;
	CSI_JPEG_IRQCallback cb;

	JpegCtx 			*jpegCtx;
#ifdef CONFIG_PM
	uint8_t suspend;
	uint8_t *baseAddr;
	CSI_JPEG_InitParam initParam;
	CSI_ConfigParam csiParam;
	JPEG_ConfigParam jpegParam;
	struct soc_device dev;
#endif

} CSI_JPEG_Priv;

static CSI_JPEG_Priv *_CsiJpegPriv;

static CSI_JPEG_Priv *CSI_JPEG_GetPriv()
{
	return _CsiJpegPriv;
}

static void CSI_JPEG_SetPriv(CSI_JPEG_Priv *priv)
{
	_CsiJpegPriv = priv;
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

__STATIC_INLINE void CSI_EnableCap(CSI_CapType mode)
{
	if (CSI_CAP_STILL == mode)
		HAL_SET_BIT(CSI->CSI_CAP_REG, CSI_C0_SCAP_EN);
	else
		HAL_SET_BIT(CSI->CSI_CAP_REG, CSI_C0_VCAP_EN);
}

__STATIC_INLINE void CSI_DisableCap(void)
{
	HAL_CLR_BIT(CSI->CSI_CAP_REG, CSI_C0_VCAP_EN | CSI_C0_SCAP_EN);
}

__STATIC_INLINE void JPEG_EncStart(void)
{
	JPEG->VE_START_REG = 0x08;
}

__STATIC_INLINE void JPEG_MemPartTake(void)
{
	HAL_SET_BIT(JPEG->VE_MODE_REG, 0x20000);
}

static void CSI_JPEG_IRQHandler(void)
{
	uint32_t csi_int, jpe_int;
	//CSI_JPEG_StreamInfo stream;
	JPEG_MpartBuffInfo mpart_info;
	JPEG_BuffInfo jpeg_info;
	CSI_JPEG_Priv *priv = CSI_JPEG_GetPriv();

	csi_int = CSI->CSI_C0_INT_STA_REG;
	jpe_int = JPEG->VE_INT_STA_REG;

	CSI_JPEG_DBG("csi:%x jpe:%x\n",csi_int, jpe_int);

	if (jpe_int & JPEG_INT_ERR) {
		CSI_JPEG_ERR("excption\n");
		priv->state = CSI_STATE_INIT;
		CSI_DisableCap();
		if (priv->cb)
			priv->cb(CSI_JPEG_EVENT_EXCP, NULL);
		CSI->CSI_C0_INT_STA_REG = csi_int;
		JPEG->VE_INT_STA_REG = jpe_int;
		return;
	}

	if (csi_int & CSI_C0_INT_STA_FRM_END_PD) {
		CSI_JPEG_DBG("f\n");
#if 0
		if (priv->jpgVeEn && priv->encMode == JPEG_MOD_OFFLINE) {
			JPEG->OUTSTM_OFFSET = 0;
			JPEG->OUTSTM_START_ADDR = (uint32_t)priv->jpgOutStmAddr[0];
			JPEG->OUTSTM_END_ADDR = (uint32_t)priv->jpgOutStmAddr[0]+ (priv->jpgMemSize-1);
			JPEG->VE_START_REG = 0x08;
		}
#else
		if (priv->jpgVeEn && priv->encMode == JPEG_MOD_OFFLINE)
			JPEG_EncStart();
#endif
		if (priv->cb)
			priv->cb(CSI_JPEG_EVENT_FRM_END, NULL);

		if (!priv->jpgVeEn && (priv->capMode != CSI_CAP_STILL || !priv->capRun))
			priv->state = CSI_STATE_READY;
	}

	uint32_t len;
	if (priv->memPartEn && (jpe_int & JPEG_MEM_PART_INT)) {
		CSI_JPEG_DBG("part\n");

		len = (priv->memPartCnt + 1) * priv->memPartSize - priv->memPartOffSet;

		mpart_info.buff_index = priv->jpgOutBufId;
		mpart_info.buff_offset = priv->memPartOffSet;
		mpart_info.size = len;
		mpart_info.tail = 0;
		priv->memPartOffSet += len;
		priv->memCurSize += len;

		priv->memPartCnt++;
		if (priv->memPartCnt >= priv->memPartNum) {
			priv->memPartCnt = 0;
			priv->memPartOffSet = 0;
		}

		if (priv->cb)
			priv->cb(CSI_JPEG_EVENT_MPART, &mpart_info);
		JPEG_MemPartTake();
	}

	if (jpe_int & JPEG_ENC_FINISH) {
		CSI_JPEG_DBG("ve\n");
		uint32_t encode_len = 0;

		encode_len = ((JPEG->HARDWARE_OFFSET - priv->jpgOutStmOffset)+7)/8;//unit is bytes
	    encode_len = (encode_len+3)/4*4;

		jpeg_info.buff_index = priv->jpgOutBufId;
		jpeg_info.size = encode_len;
		mpart_info.buff_index = priv->jpgOutBufId;
		mpart_info.buff_offset = (uint32_t)priv->jpgOutStmAddr[priv->jpgOutBufId];
		mpart_info.size = encode_len;
		mpart_info.tail = 1;
		priv->jpgOutBufId++;
		if (priv->jpgOutBufId >= priv->jpgOutBufNum)
			priv->jpgOutBufId = 0;

		if (priv->memPartEn && encode_len >= priv->memCurSize) {
			len = encode_len - priv->memCurSize;
			mpart_info.buff_offset = priv->memPartOffSet;
			mpart_info.size = len;
			priv->memCurSize = 0;

			priv->memPartCnt++;
			if (priv->memPartCnt >= priv->memPartNum)
				priv->memPartCnt = 0;
			priv->memPartOffSet = priv->memPartSize * priv->memPartCnt;
			priv->memPartOffSet = ALIGN_32B(priv->memPartOffSet);
			priv->jpgOutStmOffset = priv->memPartOffSet * 8;

			if (priv->cb)
				priv->cb(CSI_JPEG_EVENT_MPART, &mpart_info);
		}

		JPEG->OUTSTM_OFFSET = priv->memPartEn ? priv->jpgOutStmOffset : 0;
		JPEG->OUTSTM_START_ADDR = (uint32_t)priv->jpgOutStmAddr[priv->jpgOutBufId];
		JPEG->OUTSTM_END_ADDR = (uint32_t)priv->jpgOutStmAddr[priv->jpgOutBufId] + (priv->jpgMemSize - 1);

		if (priv->capRun && priv->encMode == JPEG_MOD_ONLINE)
			JPEG_EncStart();

		if (priv->cb)
			priv->cb(CSI_JPEG_EVENT_VE_END, &jpeg_info);

		if (!priv->capRun || priv->capMode == CSI_CAP_STILL) {
			priv->state = CSI_STATE_READY;
			CSI_DisableCap();
		}
	}

	CSI->CSI_C0_INT_STA_REG = csi_int;
	JPEG->VE_INT_STA_REG = jpe_int;
}

#ifdef CONFIG_PM
static int csi_jpeg_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	CSI_JPEG_Priv *priv = (CSI_JPEG_Priv*)dev->platform_data;
	priv->suspend = 1;

	switch (state) {
	case PM_MODE_SLEEP:
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_CSI_JPEG_Deinit();
		break;
	default:
		break;
	}

	return 0;
}

static int csi_jpeg_resume(struct soc_device *dev, enum suspend_state_t state)
{
	CSI_JPEG_Priv *priv = (CSI_JPEG_Priv*)dev->platform_data;

	switch (state) {
	case PM_MODE_SLEEP:
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_CSI_JPEG_Init(&priv->initParam);
		HAL_JPEG_Config(&priv->jpegParam);
		HAL_JPEG_WriteHeader(priv->baseAddr);
		HAL_CSI_Config(&priv->csiParam);
		break;
	default:
		break;
	}
	priv->suspend = 0;

	return 0;
}

static const struct soc_device_driver csi_jpeg_drv = {
	.name = "csi_jpeg",
	.suspend = csi_jpeg_suspend,
	.resume = csi_jpeg_resume,
};
#endif

HAL_Status HAL_CSI_JPEG_Init(CSI_JPEG_InitParam *param)
{
	if (!param)
		return HAL_INVALID;

	CSI_JPEG_Priv *priv = CSI_JPEG_GetPriv();

#ifdef CONFIG_PM
	if (priv && priv->suspend) {
		;
	} else {
#endif
		if (!priv) {
			priv = HAL_Malloc(sizeof(CSI_JPEG_Priv));
			if (!priv) {
				CSI_JPEG_ERR("csi jpeg malloc faild\n");
				return HAL_ERROR;
			}
			HAL_Memset(priv, 0, sizeof(CSI_JPEG_Priv));
			CSI_JPEG_SetPriv(priv);
		}
#ifdef CONFIG_PM
	}
#endif

	if (priv->state != CSI_STATE_INVALID) {
		CSI_JPEG_ERR("invalid state\n");
		return HAL_ERROR;
	}

#ifdef CONFIG_PM
	if (!priv->suspend) {
		priv->dev.name = "csi_jpeg";
		priv->dev.driver = &csi_jpeg_drv;
		HAL_Memcpy(&priv->initParam, param, sizeof(CSI_JPEG_InitParam));
		priv->dev.platform_data = priv;
		pm_register_ops(&priv->dev);
	}
#endif

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

	CSI_JPEG_REG_DBG(CCM->BUS_PERIPH_CLK_CTRL);
	CSI_JPEG_REG_DBG(CCM->BUS_PERIPH_RST_CTRL);
	CSI_JPEG_REG_DBG(CCM->CSI_MCLK_CTRL);
	CSI_JPEG_REG_DBG(CCM->CSI_JPE_DEV_CLK_CTRL);

	priv->state = CSI_STATE_INIT;

	return HAL_OK;
}

HAL_Status HAL_CSI_JPEG_Deinit(void)
{
	CSI_JPEG_Priv *priv = CSI_JPEG_GetPriv();

	if (!priv || priv->state == CSI_STATE_INVALID) {
		CSI_JPEG_ERR("%s has deinit\n", __func__);
		return HAL_ERROR;
	}

#ifdef CONFIG_PM
	if (!priv->suspend)
		pm_unregister_ops(&priv->dev);
#endif

	if (priv->jpegCtx) {
		JpegEncDestory(priv->jpegCtx);
		priv->jpegCtx = NULL;
	}

	CSI_PINS_Deinit();

	HAL_CCM_CSI_DisableMClock();
	HAL_CCM_CSI_JPEG_DisableDevClock();
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_CSI_JPEG);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_CSI_JPEG);

	priv->state = CSI_STATE_INVALID;

#ifdef CONFIG_PM
	if (!priv->suspend)
#endif
	{
		CSI_JPEG_SetPriv(NULL);
		HAL_Free(priv);
		priv = NULL;
	}

	return HAL_OK;
}

HAL_Status HAL_CSI_Config(CSI_ConfigParam *cfg)
{
	uint32_t reg_val;

	CSI_JPEG_Priv *priv = CSI_JPEG_GetPriv();
	if (priv->state != CSI_STATE_INIT && priv->state != CSI_STATE_READY)
		return HAL_ERROR;

#ifdef CONFIG_PM
	if (!priv->suspend) {
		HAL_Memcpy(&(priv->csiParam), cfg, sizeof(CSI_ConfigParam));
	}
#endif

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


	reg_val = ((0						& 0x1) << CSI_C0_FRATE_HALF_SHIFT)		|\
			  ((0						& 0xf) << CSI_C0_FRAME_MASK_SHIFT);
	CSI->CSI_CAP_REG = reg_val;

	CSI_JPEG_REG_ALL(CSI_REG_MASK);

	CSI_EnableCSI();

	return HAL_OK;
}

HAL_Status HAL_CSI_StartCapture(CSI_CapType mode)
{
	CSI_JPEG_Priv *priv = CSI_JPEG_GetPriv();

	if (priv->state != CSI_STATE_READY) {
		CSI_JPEG_ERR("invalid state: %d %d\n", priv->state, __LINE__);
		return HAL_ERROR;
	}

	unsigned long flags = HAL_EnterCriticalSection();
	priv->capRun = 1;
	priv->state = CSI_STATE_BUSY;
	priv->capMode = mode;
	HAL_ExitCriticalSection(flags);

	CSI_EnableCap(mode);

	if (priv->encMode == JPEG_MOD_ONLINE)
		JPEG_EncStart();

	return HAL_OK;
}

HAL_Status HAL_CSI_StopCapture(void)
{
	CSI_JPEG_Priv *priv = CSI_JPEG_GetPriv();

	if (priv->state != CSI_STATE_BUSY) {
		CSI_JPEG_ERR("invalid state: %d %d\n", priv->state, __LINE__);
		return HAL_ERROR;
	}

	unsigned long flags = HAL_EnterCriticalSection();
	priv->capRun = 0;
	HAL_ExitCriticalSection(flags);

	CSI_JPEG_DBG("csi capture stop..\n");

	uint8_t cnt, timeout = 1;
	for (cnt = 0; cnt < 5; cnt++) {
		if (priv->state == CSI_STATE_READY || priv->state == CSI_STATE_INVALID) {
			timeout = 0;
			break;
		} else {
			OS_MSleep(100);
		}
	}
	if (timeout)
		return HAL_ERROR;

	return HAL_OK;
}

static void JPEG_SetQtab(uint32_t quality)
{
	CSI_JPEG_Priv *priv = CSI_JPEG_GetPriv();

	priv->jpegCtx->ctl_ops->setQuantTbl(priv->jpegCtx, quality);

	priv->jpegCtx->dc_value[0] = DefaultDC / priv->jpegCtx->quant_tbl[0][0];
	priv->jpegCtx->dc_value[1] = DefaultDC / priv->jpegCtx->quant_tbl[1][0];
	priv->jpegCtx->dc_value[2] = DefaultDC / priv->jpegCtx->quant_tbl[1][0];
}

static void JPEG_WriteQtab(uint32_t base_addr)
{
	uint8_t i = 0;

	CSI_JPEG_Priv *priv = CSI_JPEG_GetPriv();

	JPEG->QM_INDEX = base_addr;

	uint32_t *tbl = (uint32_t *)&priv->jpegCtx->quant_tbl_aw;
	for (i = 0; i < 128; i++) {
		JPEG->QM_DATA = *tbl++;
	}
}

HAL_Status HAL_JPEG_Config(JPEG_ConfigParam *cfg)
{
	uint32_t reg_val;
    if(cfg == NULL) {
        CSI_JPEG_ERR("ERROR: cfg NULL\n");
        return HAL_ERROR;
    }

#if ((__CONFIG_CACHE_POLICY & 0xF) != 0)
    if ((HAL_Dcache_IsCacheable((uint32_t)cfg->csi_output_addr_y, 4))
        || (HAL_Dcache_IsCacheable((uint32_t)cfg->csi_output_addr_uv, 4))) {
        HAL_ERR("CSI_JPEG: Data buf MUST NOT CACHEABLE!!!\n");
        return HAL_ERROR;
    }
#endif

	CSI_JPEG_Priv *priv = CSI_JPEG_GetPriv();
	if (priv->state != CSI_STATE_INIT && priv->state != CSI_STATE_READY)
		return HAL_ERROR;

#ifdef CONFIG_PM
	if (!priv->suspend) {
		HAL_Memcpy(&priv->jpegParam, cfg, sizeof(JPEG_ConfigParam));
	}
#endif

	priv->jpgMemSize = cfg->outstream_buff_size;
	priv->jpgOutBufNum = cfg->outstream_buff_num ? cfg->outstream_buff_num : 1;
	for (int i = 0; i < priv->jpgOutBufNum; i++) {
#if ((__CONFIG_CACHE_POLICY & 0xF) != 0)
	if (HAL_Dcache_IsCacheable((uint32_t)cfg->outstream_buff_addr[i], cfg->outstream_buff_size)) {
		HAL_ERR("CSI_JPEG: Data buf MUST NOT CACHEABLE!!!\n");
		return HAL_ERROR;
	}
#endif
		priv->jpgOutStmAddr[i] = (uint8_t*)cfg->outstream_buff_addr[i];
	}
	//priv->jpgOutStmSize = priv->jpgMemSize / priv->jpgOutBufNum;

	priv->jpgVeEn = cfg->jpeg_en;
	priv->encMode = cfg->jpeg_en ? cfg->jpeg_mode : JPEG_MOD_OFFLINE;

	priv->jpegCtx->JpgColorFormat = JpgYUV420;
	priv->jpegCtx->quality = cfg->quality;
	priv->jpegCtx->image_height = cfg->pic_size_height;
	priv->jpegCtx->image_width = cfg->pic_size_width;

	priv->memPartEn = cfg->mem_part_en;
	if (priv->memPartEn) {
		priv->memPartNum = 1 << (cfg->mem_part_num + 1);
		priv->memPartSize = priv->jpgMemSize / priv->memPartNum;
		priv->memPartOffSet = 0;
	}

	if (cfg->jpeg_en) {
#ifdef __CONFIG_JPEG
		SYSCTL_CSI_JPEG_ShareSramType type = SYSCTL_CSI_JPEG_SHARE_32K;

		if (cfg->pic_size_width > JPEG_SRAM_SWITCH_THR_W ||
					cfg->pic_size_height > JPEG_SRAM_SWITCH_THR_H) {
  #ifdef __CONFIG_JPEG_SHARE_64K
			type = SYSCTL_CSI_JPEG_SHARE_64K;
  #else
			CSI_JPEG_ERR("JPEG share ram not enough\n");
			return HAL_ERROR;
  #endif
		}
		HAL_SYSCTL_SetCSIJPEGSramShare(type);
#else
		CSI_JPEG_ERR("JPEG share ram not exist\n");
		return HAL_ERROR;
#endif
	}

	reg_val = ((cfg->mem_part_en		& 0x1) << 16)		|\
			  ((cfg->mem_part_num		& 0x3) << 14)		|\
			  ((0						& 0x1) << 13)		|\
			  ((0						& 0x1) << 12)		|\
			  ((0						& 0x1) << 11)		|\
			  ((0						& 0x1) << 10)		|\
			  ((cfg->jpeg_mode			& 0x1) << 9)		|\
			  ((cfg->sensor_out_type	& 0x1) << 8)		|\
			  ((1  						& 0x1) << 7)		|\
			  ((1				  		& 0x1) << 6);
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

		JPEG->JPE_INPUT_ADDR_Y = cfg->jpeg_input_addr_y;
		JPEG->JPE_INPUT_ADDR_C = cfg->jpeg_input_addr_uv;
	}

	if (cfg->jpeg_scale) {
		reg_val = ((1	& 0x1) << 11)	|\
				  ((1	& 0x1) << 10);
		HAL_MODIFY_REG(JPEG->VE_MODE_REG, (1<<11)|(1<<10), reg_val);
	}

	reg_val = ((1	& 0x1) << 2)		|\
			  ((1	& 0x1) << 1)		|\
			  ((1	& 0x1) << 0);
	JPEG->VE_INT_EN_REG = reg_val;

	JPEG_SetQtab(cfg->quality);

	JPEG->JPEG_PARA0_REG = 3<<30 | priv->jpegCtx->dc_value[1]<<16 | priv->jpegCtx->dc_value[0];
	JPEG->JPEG_BITRATE_CTRL = cfg->jpeg_bitrate_en ? 0xC0000840 : 0x00000840;
	JPEG->OUTSTM_OFFSET = cfg->outstream_buff_offset;
	JPEG->OUTSTM_START_ADDR = cfg->outstream_buff_addr[0];
	JPEG->OUTSTM_END_ADDR = cfg->outstream_buff_addr[0] + cfg->outstream_buff_size - 1;

	//priv->jpgOutStmAddr =  (uint8_t*)JPEG->OUTSTM_START_ADDR;
	JPEG_WriteQtab(0x0);

	CSI_JPEG_REG_ALL(JPEG_REG_MASK);

	priv->state = CSI_STATE_READY;

	return HAL_OK;
}

void HAL_JPEG_Reset(void)
{
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_CSI_JPEG);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_CSI_JPEG);
}

void HAL_JPEG_WriteHeader(uint8_t *baseAddr)
{
	CSI_JPEG_Priv *priv = CSI_JPEG_GetPriv();

	if (baseAddr == NULL) {
		CSI_JPEG_ERR("invalid handle\n");
		return;
	}
#ifdef CONFIG_PM
	if (!priv->suspend) {
		priv->baseAddr = baseAddr;
	}
#endif
	priv->jpegCtx->BaseAddr = (char*)baseAddr;
	priv->jpegCtx->ctl_ops->writeHeader(priv->jpegCtx);
}

#endif
