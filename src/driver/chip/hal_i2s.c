/**
  * @file  hal_i2s.c
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

#ifndef __CONFIG_BOOTLOADER

#include <stdbool.h>
#include "pm/pm.h"
#include "hal_base.h"
#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_i2s.h"

#include "./codec/audio_arch.h"


//Debug config
#define XRADIO_I2S_DBG_EN               0
#define XRADIO_I2S_ERR_EN               1

#define XRADIO_I2S_DBG(fmt, arg...)    	HAL_LOG(XRADIO_I2S_DBG_EN, "[I2S] "fmt, ##arg)
#define XRADIO_I2S_ERR(fmt, arg...)    	HAL_LOG(XRADIO_I2S_ERR_EN, "[I2S] "fmt, ##arg)
#define XRADIO_I2S_ALWAYS(fmt, arg...)  HAL_LOG(1, "[I2S] "fmt, ##arg)
#define XRADIO_I2S_IT_ERR(fmt, arg...)	HAL_IT_LOG(XRADIO_I2S_ERR_EN, "[I2S] "fmt, ##arg)

//Xradio I2S config
#define XRADIO_I2S_DEF_UNDERRUN_THRES	3
#define XRADIO_I2S_DEF_OVERRUN_THRES	3
#define XRADIO_I2S_MODULE_CLK_SRC		CCM_DAUDIO_MCLK_SRC_1X

//Interface define
#define XRADIO_I2S_MALLOC				HAL_Malloc
#define XRADIO_I2S_FREE					HAL_Free
#define XRADIO_I2S_MEMCPY				HAL_Memcpy
#define XRADIO_I2S_MEMSET				HAL_Memset


//Xradio I2S priv struct
struct Xradio_I2S_Priv{
	//i2s status control
    bool isI2SInit;
    bool isTxInit;
    bool isRxInit;
    volatile bool txRunning;
    volatile bool rxRunning;

	//buffer control
    uint8_t *txBuf;
    uint8_t *rxBuf;
    uint8_t *readPointer;
    uint8_t *writePointer;
	uint32_t rxBufSize;
	uint32_t txBufSize;

	//DMA control
    uint8_t *txDmaPointer;
    uint8_t *rxDmaPointer;
    DMA_Channel txDMAChan;
    DMA_Channel rxDMAChan;
	DMA_DataWidth tx_data_width;
	DMA_DataWidth rx_data_width;
    volatile uint8_t txHalfCallCount;
    volatile uint8_t rxHalfCallCount;
    volatile uint8_t txEndCallCount;
    volatile uint8_t rxEndCallCount;

	//Semaphore control
    bool isTxSemaphore;
    bool isRxSemaphore;
    HAL_Semaphore txReady;
    HAL_Semaphore rxReady;

	//Misc control
	uint8_t mclk_div;
	uint8_t i2s_mode;
	uint8_t slot_width;
	uint8_t loop_back_en;
	uint8_t pcm_frame_mode;
	uint32_t lrck_period;
	uint32_t daudio_clk_freq;
	uint16_t tx_underrun_threshold;
	uint16_t rx_overrun_threshold;
};

static struct Xradio_I2S_Priv *xradio_i2s_priv;


//const array define
struct real_val_to_reg_val {
	uint32_t real_val;
	uint32_t reg_val;
};

struct pll_param {
	uint32_t hosc_freq;
	Audio_Clk_Freq pll_freq_out;
	uint32_t pllParam;
	uint32_t pllPatParam;
};

static const struct real_val_to_reg_val xradio_i2s_bclk_div[] = {
	{0,  0},
	{1,  1},
	{2,  2},
	{4,  3},
	{6,  4},
	{8,  5},
	{12, 6},
	{16, 7},
	{24, 8},
	{32, 9},
	{48, 10},
	{64, 11},
	{96, 12},
	{128,13},
	{176,14},
	{192,15},
};

static const struct real_val_to_reg_val xradio_i2s_mclk_div[] = {
	{0,  0},
	{1,  1},
	{2,  2},
	{4,  3},
	{6,  4},
	{8,  5},
	{12, 6},
	{16, 7},
	{24, 8},
	{32, 9},
	{48, 10},
	{64, 11},
	{96, 12},
	{128,13},
	{176,14},
	{192,15},
};

static const struct pll_param xradio_i2s_pll_params[] = {
    {HOSC_CLOCK_26M, AUDIO_CLK_24M, PRCM_AUD_PLL24M_PARAM_HOSC26M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC26M},
    {HOSC_CLOCK_26M, AUDIO_CLK_22M, PRCM_AUD_PLL22M_PARAM_HOSC26M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC26M},
    {HOSC_CLOCK_24M, AUDIO_CLK_24M, PRCM_AUD_PLL24M_PARAM_HOSC24M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC24M},
    {HOSC_CLOCK_24M, AUDIO_CLK_22M, PRCM_AUD_PLL22M_PARAM_HOSC24M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC24M},
    {HOSC_CLOCK_40M, AUDIO_CLK_24M, PRCM_AUD_PLL24M_PARAM_HOSC40M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC40M},
    {HOSC_CLOCK_40M, AUDIO_CLK_22M, PRCM_AUD_PLL22M_PARAM_HOSC40M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC40M},
    {HOSC_CLOCK_52M, AUDIO_CLK_24M, PRCM_AUD_PLL24M_PARAM_HOSC52M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC52M},
    {HOSC_CLOCK_52M, AUDIO_CLK_22M, PRCM_AUD_PLL22M_PARAM_HOSC52M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC52M},
};


//Base read/write Interface
__nonxip_text
static int xradio_i2s_reg_read(uint32_t reg)
{
	return HAL_REG_32BIT(reg+I2S_BASE);
}

__nonxip_text
static int xradio_i2s_reg_write(uint32_t reg, uint32_t val)
{
	HAL_REG_32BIT(reg+I2S_BASE) = val;

	return HAL_OK;
}

__nonxip_text
static int xradio_i2s_reg_update_bits(uint32_t reg, uint32_t mask, uint32_t val)
{
	uint32_t val_old,val_new;

	val_old = xradio_i2s_reg_read(reg);
	val_new = (val_old & ~mask) | (val & mask);

	if(val_new != val_old){
		xradio_i2s_reg_write(reg, val_new);
	}

	return HAL_OK;
}


/*************************** XRADIO I2S DMA Control ****************************/
__nonxip_text
static void xradio_i2s_dma_trigger(Audio_Stream_Dir dir, bool enable)
{
	uint32_t flags;
	uint8_t doProtection = !HAL_IsISRContext();

	if (doProtection) {
		flags = HAL_EnterCriticalSection();
	}

	if (enable) {
		if (dir == PCM_OUT) {
			//Flush TX_FIFO
			xradio_i2s_reg_update_bits(DA_FCTL, 0x1<<I2S_FTX_BIT, 0x1<<I2S_FTX_BIT);
			if(xradio_i2s_priv->txDMAChan != DMA_CHANNEL_INVALID){
				//TX DRQ Enable
				xradio_i2s_reg_update_bits(DA_INT, 0x1<<I2S_TXE_DRQ_EN_BIT, 0x1<<I2S_TXE_DRQ_EN_BIT);
				//DMA Start
				HAL_DMA_Start(xradio_i2s_priv->txDMAChan, (uint32_t)xradio_i2s_priv->txBuf, (uint32_t)(DA_TXFIFO+I2S_BASE), xradio_i2s_priv->txBufSize);
			}
			xradio_i2s_priv->txRunning = true;
		} else {
			//Flush RX_FIFO
			xradio_i2s_reg_update_bits(DA_FCTL, 0x1<<I2S_FRX_BIT, 0x1<<I2S_FRX_BIT);
			if(xradio_i2s_priv->rxDMAChan != DMA_CHANNEL_INVALID){
				//RX DRQ Enable
				xradio_i2s_reg_update_bits(DA_INT, 0x1<<I2S_RXA_DRQ_EN_BIT, 0x1<<I2S_RXA_DRQ_EN_BIT);
				//DMA Start
				HAL_DMA_Start(xradio_i2s_priv->rxDMAChan, (uint32_t)(DA_RXFIFO+I2S_BASE), (uint32_t)xradio_i2s_priv->rxBuf, xradio_i2s_priv->rxBufSize);
			}
			xradio_i2s_priv->rxRunning = true;
		}
	} else {
		if (dir == PCM_OUT) {
			//Flush TX_FIFO
			xradio_i2s_reg_update_bits(DA_FCTL, 0x1<<I2S_FTX_BIT, 0x1<<I2S_FTX_BIT);
			if(xradio_i2s_priv->txDMAChan != DMA_CHANNEL_INVALID){
				//TX DRQ Disable
				xradio_i2s_reg_update_bits(DA_INT, 0x1<<I2S_TXE_DRQ_EN_BIT, 0x0<<I2S_TXE_DRQ_EN_BIT);
				//DMA Stop
				HAL_DMA_Stop(xradio_i2s_priv->txDMAChan);
			}
			xradio_i2s_priv->txRunning = false;
		} else {
			//Flush RX_FIFO
			xradio_i2s_reg_update_bits(DA_FCTL, 0x1<<I2S_FRX_BIT, 0x1<<I2S_FRX_BIT);
			if(xradio_i2s_priv->rxDMAChan != DMA_CHANNEL_INVALID){
				//RX DRQ Disable
				xradio_i2s_reg_update_bits(DA_INT, 0x1<<I2S_RXA_DRQ_EN_BIT, 0x0<<I2S_RXA_DRQ_EN_BIT);
				//DMA Stop
				HAL_DMA_Stop(xradio_i2s_priv->rxDMAChan);
			}
			xradio_i2s_priv->rxRunning = false;
		}
	}

	if (doProtection) {
		HAL_ExitCriticalSection(flags);
	}
}

__nonxip_text
static int xradio_i2s_dma_threshold_check(Audio_Stream_Dir dir)
{
	if (dir == PCM_OUT) {
		if (xradio_i2s_priv->txHalfCallCount >= xradio_i2s_priv->tx_underrun_threshold ||
			xradio_i2s_priv->txEndCallCount  >= xradio_i2s_priv->tx_underrun_threshold) {
			XRADIO_I2S_IT_ERR("Tx : underrun and stop dma tx...\n");
			xradio_i2s_dma_trigger(PCM_OUT, false);
			//xradio_i2s_priv->txRunning = false;	//has been config in trigger interface
			xradio_i2s_priv->writePointer = NULL;
			xradio_i2s_priv->txDmaPointer = NULL;
			xradio_i2s_priv->txHalfCallCount = 0;
			xradio_i2s_priv->txEndCallCount = 0;
			return HAL_ERROR;
		}
	} else {
		if (xradio_i2s_priv->rxHalfCallCount >= xradio_i2s_priv->rx_overrun_threshold ||
			xradio_i2s_priv->rxEndCallCount  >= xradio_i2s_priv->rx_overrun_threshold) {
			XRADIO_I2S_IT_ERR("Rx : overrun and stop dma rx...\n");
			xradio_i2s_dma_trigger(PCM_IN, false);
			//xradio_i2s_priv->rxRunning = false;	//has been config in trigger interface
			xradio_i2s_priv->readPointer = NULL;
			xradio_i2s_priv->rxDmaPointer = NULL;
			xradio_i2s_priv->rxHalfCallCount = 0;
			xradio_i2s_priv->rxEndCallCount = 0;
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}

__nonxip_text
static void xradio_i2s_dma_half_callback(void *arg)
{
	if (arg == &xradio_i2s_priv->txReady) {
		xradio_i2s_priv->txHalfCallCount++;
		if(xradio_i2s_priv->isTxSemaphore){
			xradio_i2s_priv->isTxSemaphore = false;
			HAL_SemaphoreRelease((HAL_Semaphore *)arg);
		}
		if(xradio_i2s_dma_threshold_check(PCM_OUT))
			return;
		xradio_i2s_priv->txDmaPointer = xradio_i2s_priv->txBuf + xradio_i2s_priv->txBufSize/2;
	} else {
		xradio_i2s_priv->rxHalfCallCount++;
		if(xradio_i2s_priv->isRxSemaphore){
			xradio_i2s_priv->isRxSemaphore = false;
			HAL_SemaphoreRelease((HAL_Semaphore *)arg);
		}
		if(xradio_i2s_dma_threshold_check(PCM_IN))
			return;
		xradio_i2s_priv->rxDmaPointer = xradio_i2s_priv->rxBuf + xradio_i2s_priv->rxBufSize/2;
	}
}

__nonxip_text
static void xradio_i2s_dma_end_callback(void *arg)
{
	if (arg == &xradio_i2s_priv->txReady) {
		xradio_i2s_priv->txEndCallCount++;
		if (xradio_i2s_priv->isTxSemaphore) {
			xradio_i2s_priv->isTxSemaphore = false;
			HAL_SemaphoreRelease((HAL_Semaphore *)arg);
		}
		if (xradio_i2s_dma_threshold_check(PCM_OUT))
			return;
		xradio_i2s_priv->txDmaPointer = xradio_i2s_priv->txBuf;
	} else {
		xradio_i2s_priv->rxEndCallCount++;
		if (xradio_i2s_priv->isRxSemaphore) {
			xradio_i2s_priv->isRxSemaphore = false;
			HAL_SemaphoreRelease((HAL_Semaphore *)arg);
		}
		if (xradio_i2s_dma_threshold_check(PCM_IN))
			return;
		xradio_i2s_priv->rxDmaPointer = xradio_i2s_priv->rxBuf;
	}
}

static int xradio_i2s_dma_init(Audio_Stream_Dir dir, DMA_Channel channel)
{
#if HAL_DMA_OPT_TRANSFER_HALF_IRQ

	DMA_ChannelInitParam dmaParam;
	HAL_Memset(&dmaParam, 0, sizeof(dmaParam));

	if (dir == PCM_OUT) {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(
						DMA_WORK_MODE_CIRCULAR,
						DMA_WAIT_CYCLE_2,
						DMA_BYTE_CNT_MODE_REMAIN,
						xradio_i2s_priv->tx_data_width,
						DMA_BURST_LEN_1,
						DMA_ADDR_MODE_FIXED,
						DMA_PERIPH_DAUDIO,
						xradio_i2s_priv->tx_data_width,
						DMA_BURST_LEN_1,
						DMA_ADDR_MODE_INC,
						DMA_PERIPH_SRAM);

		dmaParam.endArg = &xradio_i2s_priv->txReady;
		dmaParam.halfArg = &xradio_i2s_priv->txReady;
	} else {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(
						DMA_WORK_MODE_CIRCULAR,
						DMA_WAIT_CYCLE_2,
						DMA_BYTE_CNT_MODE_REMAIN,
						xradio_i2s_priv->rx_data_width,
						DMA_BURST_LEN_1,
						DMA_ADDR_MODE_INC,
						DMA_PERIPH_SRAM,
						xradio_i2s_priv->rx_data_width,
						DMA_BURST_LEN_1,
						DMA_ADDR_MODE_FIXED,
						DMA_PERIPH_DAUDIO);

		dmaParam.endArg = &xradio_i2s_priv->rxReady;
		dmaParam.halfArg = &xradio_i2s_priv->rxReady;
	}

	dmaParam.irqType = DMA_IRQ_TYPE_BOTH;
	dmaParam.halfCallback = xradio_i2s_dma_half_callback;
	dmaParam.endCallback = xradio_i2s_dma_end_callback;

	return HAL_DMA_Init(channel, &dmaParam);

#else

	XRADIO_I2S_ERR("DMA don't support Half IRQ, dma init Fail!\n");
	return HAL_ERROR;

#endif
}

/*****************************************************************************/


static void xradio_i2s_reset(void)
{
	XRADIO_I2S_DBG("%s, reset all register to their default value\n",__FUNCTION__);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_DAUDIO);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_DAUDIO);
}

static void xradio_i2s_hw_common_init(Audio_Stream_Dir dir)
{
	//MCLK Output Enable
	xradio_i2s_reg_update_bits(DA_CLKD, 0x1<<I2S_MCLK_OUT_EN_BIT, 0x1<<I2S_MCLK_OUT_EN_BIT);

	//PCM frame mode and LRCK Period and slot width config
	xradio_i2s_reg_update_bits(DA_FMT0, 0x1<<I2S_LRCK_WIDTH_BIT | 0x3ff<<I2S_LRCK_PERIOD_BIT | 0x7<<I2S_SLOT_WIDTH_BIT,\
		!!xradio_i2s_priv->pcm_frame_mode<<I2S_LRCK_WIDTH_BIT | (xradio_i2s_priv->lrck_period-1)<<I2S_LRCK_PERIOD_BIT | (xradio_i2s_priv->slot_width/4-1)<<I2S_SLOT_WIDTH_BIT);

	//MSB/LSB and PCM Data mode config
	xradio_i2s_reg_update_bits(DA_FMT1, 0x1<<I2S_RX_MLS_BIT | 0x1<<I2S_TX_MLS_BIT | 0x3<<I2S_RX_PDM_BIT | 0x3<<I2S_TX_PDM_BIT,\
										0x0<<I2S_RX_MLS_BIT | 0x0<<I2S_TX_MLS_BIT | 0x0<<I2S_RX_PDM_BIT | 0x0<<I2S_TX_PDM_BIT);

	//TX/RX trigger level and FIFO mode config
	xradio_i2s_reg_update_bits(DA_FCTL, 0x7f<<I2S_TXTL_BIT | 0x3f<<I2S_RXTL_BIT, 0x40<<I2S_TXTL_BIT | 0x0f<<I2S_RXTL_BIT);

	/* I2S Loopback test enable */
	xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_LOOP_EN_BIT, !!xradio_i2s_priv->loop_back_en<<I2S_LOOP_EN_BIT);

	if(dir == PCM_OUT){
		//SDO0 Output Enable
		xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_SDO0_EN_BIT, 0x1<<I2S_SDO0_EN_BIT);
		//TX channel map
		xradio_i2s_reg_write(DA_TX0CHMAP, 0x76543210);
		//TX block Enable
		xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_TX_EN_BIT, 0x1<<I2S_TX_EN_BIT);
	} else {
		//RX channel map
		xradio_i2s_reg_write(DA_RXCHMAP, 0x76543210);
		//RX block Enable
		xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_RX_EN_BIT, 0x1<<I2S_RX_EN_BIT);
	}
}

static void xradio_i2s_hw_common_deinit(Audio_Stream_Dir dir)
{
	if(!xradio_i2s_priv->isTxInit && !xradio_i2s_priv->isRxInit){
		//MCLK Output Disable
		xradio_i2s_reg_update_bits(DA_CLKD, 0x1<<I2S_MCLK_OUT_EN_BIT, 0x0<<I2S_MCLK_OUT_EN_BIT);
		//I2S Globle Disable
		xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_GEN_BIT, 0x0<<I2S_GEN_BIT);
	}

	if(dir == PCM_OUT){
		//SDO0 Output Disable
		xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_SDO0_EN_BIT, 0x0<<I2S_SDO0_EN_BIT);
		//TX channel map
		xradio_i2s_reg_write(DA_TX0CHMAP, 0x0);
		//TX block Disable
		xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_TX_EN_BIT, 0x0<<I2S_TX_EN_BIT);
	} else {
		//RX channel map
		xradio_i2s_reg_write(DA_RXCHMAP, 0x0);
		//RX block Disable
		xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_RX_EN_BIT, 0x0<<I2S_RX_EN_BIT);
	}
}

static int xradio_i2s_set_audio_pll(Audio_Clk_Freq pll_freq_out)
{
	uint32_t i,hosc_freq,audioPllParam,audioPllPatParam;

	if(pll_freq_out != AUDIO_CLK_24M && pll_freq_out != AUDIO_CLK_22M){
		return HAL_INVALID;
	}

	hosc_freq = HAL_GetHFClock();

	for(i=0; i<HAL_ARRAY_SIZE(xradio_i2s_pll_params); i++){
		if(xradio_i2s_pll_params[i].hosc_freq == hosc_freq && xradio_i2s_pll_params[i].pll_freq_out == pll_freq_out){
			audioPllParam = xradio_i2s_pll_params[i].pllParam;
			audioPllPatParam = xradio_i2s_pll_params[i].pllPatParam;
			XRADIO_I2S_DBG("Xradio I2S PLL freq_in match:%u, freq_out:%u\n",hosc_freq,pll_freq_out);
			break;
		}
	}

	if(i == HAL_ARRAY_SIZE(xradio_i2s_pll_params)){
		XRADIO_I2S_ERR("Match audio pll input/output freq failed\n");
		return HAL_ERROR;
	}

	HAL_PRCM_SetAudioPLLParam(audioPllParam);
	HAL_PRCM_SetAudioPLLPatternParam(audioPllPatParam);
	HAL_PRCM_EnableAudioPLL();
	HAL_PRCM_EnableAudioPLLPattern();

	return HAL_OK;
}


static int xradio_i2s_dai_set_fmt(uint32_t fmt)
{
	uint8_t tx_rx_offset, i2s_mode, sign_ext, lrck_polarity, brck_polarity;
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	/* I2S config Master/Slave mode */
	switch (fmt & I2S_ROLE_MASK) {
		case DAIFMT_CBM_CFM:	//I2S Slave
			XRADIO_I2S_DBG("I2S set to work as Slave\n");
			xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_BCLK_OUT_BIT | 0x1<<I2S_LRCK_OUT_BIT, 0x0<<I2S_BCLK_OUT_BIT | 0x0<<I2S_LRCK_OUT_BIT);	//BCLK & LRCK input
			break;
		case DAIFMT_CBS_CFS:	//I2S Master
			XRADIO_I2S_DBG("I2S set to work as Master\n");
			xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_BCLK_OUT_BIT | 0x1<<I2S_LRCK_OUT_BIT, 0x1<<I2S_BCLK_OUT_BIT | 0x1<<I2S_LRCK_OUT_BIT); 	//BCLK & LRCK output
			break;
		default:
			XRADIO_I2S_ERR("I2S Master/Slave mode config error:%u\n\n",(fmt & I2S_ROLE_MASK)>>0);
			return HAL_INVALID;
	}

	/* I2S config I2S/LJ/RJ/PCM format */
	xradio_i2s_priv->i2s_mode = 1;
	switch (fmt & I2S_FORMAT_MASK) {
		case DAIFMT_I2S:
			XRADIO_I2S_DBG("I2S config I2S format\n");
			i2s_mode = LEFT_JUSTIFIED_FORMAT;
			tx_rx_offset = 1;
			sign_ext = TRANSFER_ZERO_AFTER;
			break;
		case DAIFMT_RIGHT_J:
			XRADIO_I2S_DBG("I2S config RIGHT-JUSTIFIED format\n");
			i2s_mode = RIGHT_JUSTIFIED_FORMAT;
			tx_rx_offset = 0;
			sign_ext = SIGN_EXTENSION_MSB;
			break;
		case DAIFMT_LEFT_J:
			XRADIO_I2S_DBG("I2S config LEFT-JUSTIFIED format\n");
			i2s_mode = LEFT_JUSTIFIED_FORMAT;
			tx_rx_offset = 0;
			sign_ext = TRANSFER_ZERO_AFTER;
			break;
		case DAIFMT_DSP_A:
			XRADIO_I2S_DBG("I2S config PCM-A format\n");
			i2s_mode = PCM_FORMAT;
			tx_rx_offset = 1;
			sign_ext = TRANSFER_ZERO_AFTER;
			xradio_i2s_priv->i2s_mode = 0;
			break;
		case DAIFMT_DSP_B:
			XRADIO_I2S_DBG("I2S config PCM-B format\n");
			i2s_mode = PCM_FORMAT;
			tx_rx_offset = 0;
			sign_ext = TRANSFER_ZERO_AFTER;
			xradio_i2s_priv->i2s_mode = 0;
			break;
		default:
			XRADIO_I2S_ERR("I2S I2S format config error:%u\n\n",(fmt & I2S_FORMAT_MASK)>>4);
			return HAL_INVALID;
	}
	xradio_i2s_reg_update_bits(DA_CTL, 0x3<<I2S_MODE_SEL_BIT, i2s_mode<<I2S_MODE_SEL_BIT);
	xradio_i2s_reg_update_bits(DA_TX0CHSEL, 0x1<<I2S_TX0_OFFSET_BIT, tx_rx_offset<<I2S_TX0_OFFSET_BIT);
	xradio_i2s_reg_update_bits(DA_RXCHSEL, 0x1<<I2S_RX_OFFSET_BIT, tx_rx_offset<<I2S_RX_OFFSET_BIT);
	xradio_i2s_reg_update_bits(DA_FMT1, 0x3<<I2S_SEXT_BIT, sign_ext<<I2S_SEXT_BIT);

	/* I2S config BCLK&LRCK polarity */
	switch (fmt & I2S_POLARITY_MASK) {
		case DAIFMT_NB_NF:
			XRADIO_I2S_DBG("I2S config BCLK&LRCK polarity: BCLK_normal,LRCK_normal\n");
			brck_polarity = BCLK_NORMAL_DRIVE_N_SAMPLE_P;
			lrck_polarity = LRCK_LEFT_LOW_RIGHT_HIGH;
			break;
		case DAIFMT_NB_IF:
			XRADIO_I2S_DBG("I2S config BCLK&LRCK polarity: BCLK_normal,LRCK_invert\n");
			brck_polarity = BCLK_NORMAL_DRIVE_N_SAMPLE_P;
			lrck_polarity = LRCK_LEFT_HIGH_RIGHT_LOW;
			break;
		case DAIFMT_IB_NF:
			XRADIO_I2S_DBG("I2S config BCLK&LRCK polarity: BCLK_invert,LRCK_normal\n");
			brck_polarity = BCLK_INVERT_DRIVE_P_SAMPLE_N;
			lrck_polarity = LRCK_LEFT_LOW_RIGHT_HIGH;
			break;
		case DAIFMT_IB_IF:
			XRADIO_I2S_DBG("I2S config BCLK&LRCK polarity: BCLK_invert,LRCK_invert\n");
			brck_polarity = BCLK_INVERT_DRIVE_P_SAMPLE_N;
			lrck_polarity = LRCK_LEFT_HIGH_RIGHT_LOW;
			break;
		default:
			XRADIO_I2S_ERR("I2S config BCLK/LRCLK polarity error:%u\n\n",(fmt & I2S_POLARITY_MASK)>>8);
			return HAL_INVALID;
	}
	xradio_i2s_reg_update_bits(DA_FMT0,  0x1<<I2S_LRCK_POLARITY_BIT | 0x1<<I2S_BCLK_POLARITY_BIT, lrck_polarity<<I2S_LRCK_POLARITY_BIT | brck_polarity<<I2S_BCLK_POLARITY_BIT);

	return HAL_OK;
}

static int xradio_i2s_set_clkdiv(uint32_t sample_rate)
{
	int ret;
	uint8_t i, bclk_div;
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	//Config Audio PLL
	ret = xradio_i2s_set_audio_pll(xradio_i2s_priv->daudio_clk_freq = sample_rate%1000 ? AUDIO_CLK_22M : AUDIO_CLK_24M);
	if(ret != HAL_OK){
		XRADIO_I2S_ERR("Config Audio PLL Fail!\n");
		return HAL_ERROR;
	}

	//Set MCLK Div
	for(i=0; i<HAL_ARRAY_SIZE(xradio_i2s_mclk_div); i++){
		if(xradio_i2s_mclk_div[i].real_val == xradio_i2s_priv->mclk_div){
			xradio_i2s_reg_update_bits(DA_CLKD, 0xf<<I2S_MCLK_DIV_BIT, xradio_i2s_mclk_div[i].reg_val<<I2S_MCLK_DIV_BIT);
			break;
		}
	}

	if(i == HAL_ARRAY_SIZE(xradio_i2s_mclk_div)){
		XRADIO_I2S_ERR("Don't match MCLK DIV-%d!\n",xradio_i2s_priv->mclk_div);
		return HAL_ERROR;
	}

	//Set BCLK Div
	bclk_div = xradio_i2s_priv->daudio_clk_freq / ((xradio_i2s_priv->i2s_mode+1)*xradio_i2s_priv->lrck_period) / sample_rate;
	for(i=0; i<HAL_ARRAY_SIZE(xradio_i2s_bclk_div); i++){
		if(xradio_i2s_bclk_div[i].real_val == bclk_div){
			xradio_i2s_reg_update_bits(DA_CLKD, 0xf<<I2S_BCLK_DIV_BIT, xradio_i2s_bclk_div[i].reg_val<<I2S_BCLK_DIV_BIT);
			break;
		}
	}

	if(i == HAL_ARRAY_SIZE(xradio_i2s_bclk_div)){
		XRADIO_I2S_ERR("Don't match BCLK DIV-%d!\n",bclk_div);
		return HAL_ERROR;
	}

	return HAL_OK;
}

static int xradio_i2s_dai_hw_params(Audio_Stream_Dir dir, struct pcm_config *pcm_cfg)
{
	uint32_t dma_buf_size;
	uint8_t sample_resolution, dma_data_width;
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	/* I2S common init */
	xradio_i2s_hw_common_init(dir);

	sample_resolution = pcm_format_to_sampleresolution(pcm_cfg->format);
	dma_data_width = sample_resolution<=8 ? DMA_DATA_WIDTH_8BIT : (sample_resolution<=16 ? DMA_DATA_WIDTH_16BIT : DMA_DATA_WIDTH_32BIT);
	dma_buf_size = pcm_frames_to_bytes(pcm_cfg, pcm_config_to_frames(pcm_cfg));

	/* I2S set channels */
	if(pcm_cfg->channels<1 || pcm_cfg->channels>8){
		XRADIO_I2S_ERR("Invalid channel numbers: %d!",pcm_cfg->channels);
		return HAL_INVALID;
	}

	if(dir == PCM_OUT){
		//TX slot num between CPU/DMA and FIFO
		xradio_i2s_reg_update_bits(DA_CHCFG, 0x7<<I2S_TX_SLOT_NUM, (pcm_cfg->channels-1)<<I2S_TX_SLOT_NUM);
		//TX slot num for each output
		xradio_i2s_reg_update_bits(DA_TX0CHSEL, 0x7<<I2S_TX0_CHSEL_BIT, (pcm_cfg->channels-1)<<I2S_TX0_CHSEL_BIT);
		//TX channel enable
		xradio_i2s_reg_update_bits(DA_TX0CHSEL, 0xff<<I2S_TX0_CHEN_BIT, ((1<<pcm_cfg->channels)-1)<<I2S_TX0_CHEN_BIT);
		//TX FIFO mode config
		xradio_i2s_reg_update_bits(DA_FCTL, 0x1<<I2S_TXIM_BIT, (dma_data_width == DMA_DATA_WIDTH_32BIT ? 0x0 : 0x1)<<I2S_TXIM_BIT);

		//dma tx params config
		xradio_i2s_priv->txBufSize = dma_buf_size;
		xradio_i2s_priv->tx_data_width = dma_data_width;

	} else {

		//RX slot num between CPU/DMA and FIFO
		xradio_i2s_reg_update_bits(DA_CHCFG, 0x7<<I2S_RX_SLOT_NUM, (pcm_cfg->channels-1)<<I2S_RX_SLOT_NUM);
		//RX slot num for each input
		xradio_i2s_reg_update_bits(DA_RXCHSEL, 0x7<<I2S_RX_CHSEL_BIT, (pcm_cfg->channels-1)<<I2S_RX_CHSEL_BIT);
		//RX FIFO mode config
		xradio_i2s_reg_update_bits(DA_FCTL, 0x3<<I2S_RXOM_BIT, (dma_data_width == DMA_DATA_WIDTH_32BIT ? 0x0 : 0x1)<<I2S_RXOM_BIT);

		//dma rx params config
		xradio_i2s_priv->rxBufSize = dma_buf_size;
		xradio_i2s_priv->rx_data_width = dma_data_width;
	}

	/* I2S set sample resorution */
	if(sample_resolution<8 || sample_resolution>32){
		XRADIO_I2S_ERR("Invalid sample resolution: %d!",sample_resolution);
		return HAL_INVALID;
	}
	xradio_i2s_reg_update_bits(DA_FMT0, 0x7<<I2S_SAMPLE_RES_BIT, (sample_resolution/4-1)<<I2S_SAMPLE_RES_BIT);

	/* I2S Globle Enable */
	xradio_i2s_reg_update_bits(DA_CTL, 0x1<<I2S_GEN_BIT, 0x1<<I2S_GEN_BIT);


	XRADIO_I2S_DBG("Sample rate-[%d], channel numbers-[%d], sample resolution-[%d]\n",pcm_cfg->rate,pcm_cfg->channels,sample_resolution);

	return HAL_OK;
}

static int xradio_i2s_dai_hw_free(Audio_Stream_Dir dir)
{
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	xradio_i2s_hw_common_deinit(dir);

	if(!xradio_i2s_priv->isTxInit && !xradio_i2s_priv->isRxInit){
		xradio_i2s_reset();
		HAL_PRCM_DisableAudioPLL();
        HAL_PRCM_DisableAudioPLLPattern();
	}

	return HAL_OK;
}


static HAL_Status xradio_i2s_open(Audio_Stream_Dir dir)
{
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	if (dir == PCM_OUT) {
		if (xradio_i2s_priv->isTxInit == true) {
			XRADIO_I2S_DBG("I2S TX block has opened already\n");
			return HAL_OK;
		}
	} else {
		if (xradio_i2s_priv->isRxInit == true) {
			XRADIO_I2S_DBG("I2S RX block has opened already\n");
			return HAL_OK;
		}
	}

	if (dir == PCM_OUT) {
		xradio_i2s_priv->txDMAChan = DMA_CHANNEL_INVALID;
		xradio_i2s_priv->writePointer = NULL;
		xradio_i2s_priv->txHalfCallCount = 0;
		xradio_i2s_priv->txEndCallCount  = 0;

		/* malloc tx buf for DMA */
		xradio_i2s_priv->txBuf = XRADIO_I2S_MALLOC(xradio_i2s_priv->txBufSize);
		if(xradio_i2s_priv->txBuf == NULL){
			XRADIO_I2S_ERR("Malloc I2S tx buf for DMA failed!\n");
			xradio_i2s_priv->txBufSize = 0;
			return HAL_ERROR;
		}
		XRADIO_I2S_MEMSET(xradio_i2s_priv->txBuf, 0, xradio_i2s_priv->txBufSize);

		/* request DMA channel */
		xradio_i2s_priv->txDMAChan = HAL_DMA_Request();
		if (xradio_i2s_priv->txDMAChan == DMA_CHANNEL_INVALID) {
			XRADIO_I2S_ERR("Obtain I2S tx DMA channel failed!\n");
			XRADIO_I2S_FREE(xradio_i2s_priv->txBuf);
			xradio_i2s_priv->txBuf = NULL;
			xradio_i2s_priv->txBufSize = 0;
			return HAL_ERROR;
		}

		/* init DMA */
		xradio_i2s_dma_init(PCM_OUT, xradio_i2s_priv->txDMAChan);

		/* init semaphore */
		HAL_SemaphoreInitBinary(&xradio_i2s_priv->txReady);

		xradio_i2s_priv->isTxInit = true;

	} else {

		xradio_i2s_priv->rxDMAChan = DMA_CHANNEL_INVALID;
		xradio_i2s_priv->readPointer = NULL;
		xradio_i2s_priv->rxHalfCallCount = 0;
		xradio_i2s_priv->rxEndCallCount = 0;

		/* malloc rx buf for DMA */
		xradio_i2s_priv->rxBuf = XRADIO_I2S_MALLOC(xradio_i2s_priv->rxBufSize);
		if(xradio_i2s_priv->rxBuf == NULL){
			XRADIO_I2S_ERR("Malloc I2S rx buf for DMA failed!\n");
			xradio_i2s_priv->rxBufSize = 0;
			return HAL_ERROR;
		}
		XRADIO_I2S_MEMSET(xradio_i2s_priv->rxBuf, 0, xradio_i2s_priv->rxBufSize);

		/* request DMA channel */
		xradio_i2s_priv->rxDMAChan = HAL_DMA_Request();
		if (xradio_i2s_priv->rxDMAChan == DMA_CHANNEL_INVALID) {
			XRADIO_I2S_ERR("Obtain I2S rx DMA channel failed!\n");
			XRADIO_I2S_FREE(xradio_i2s_priv->rxBuf);
			xradio_i2s_priv->rxBuf = NULL;
			xradio_i2s_priv->rxBufSize = 0;
			return HAL_ERROR;
		}

		/* init DMA */
		xradio_i2s_dma_init(PCM_IN, xradio_i2s_priv->rxDMAChan);

		/* init semaphore */
		HAL_SemaphoreInitBinary(&xradio_i2s_priv->rxReady);

		xradio_i2s_priv->isRxInit = true;
	}

#if XRADIO_I2S_DBG_EN
	uint32_t i;
	printf("\nXradio I2S Reg:");
	for(i=0; i<92; i+=4){
		if(!(i%16)) printf("\n");
		printf("Reg[0x%02x] :0x%08x;  ",i,HAL_REG_32BIT(I2S_BASE+i));
	}
	printf("\n\n");
#endif

	return HAL_OK;
}

static HAL_Status xradio_i2s_close(Audio_Stream_Dir dir)
{
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	if (dir == PCM_OUT) {
		if (xradio_i2s_priv->isTxInit == false) {
			XRADIO_I2S_DBG("I2S TX block has closed already\n");
			return HAL_OK;
		}
	} else {
		if (xradio_i2s_priv->isRxInit == false) {
			XRADIO_I2S_DBG("I2S RX block has closed already\n");
			return HAL_OK;
		}
	}

	if (dir == PCM_OUT) {
		xradio_i2s_dma_trigger(PCM_OUT, false);
		//xradio_i2s_priv->txRunning = false;
		xradio_i2s_priv->writePointer = NULL;
		xradio_i2s_priv->txHalfCallCount = 0;
		xradio_i2s_priv->txEndCallCount  = 0;

		/* deinit semaphore */
		HAL_SemaphoreDeinit(&xradio_i2s_priv->txReady);

		/* deinit DMA */
		HAL_DMA_DeInit(xradio_i2s_priv->txDMAChan);

		/* release DMA channel */
		HAL_DMA_Release(xradio_i2s_priv->txDMAChan);
		xradio_i2s_priv->txDMAChan = DMA_CHANNEL_INVALID;
		xradio_i2s_priv->tx_data_width = 0;

		/* free tx buf for DMA */
		XRADIO_I2S_FREE(xradio_i2s_priv->txBuf);
		xradio_i2s_priv->txBuf = NULL;
		xradio_i2s_priv->txBufSize = 0;

		xradio_i2s_priv->isTxInit = false;

	} else {

		xradio_i2s_dma_trigger(PCM_IN, false);
		//xradio_i2s_priv->rxRunning = false;
		xradio_i2s_priv->readPointer = NULL;
		xradio_i2s_priv->rxHalfCallCount = 0;
		xradio_i2s_priv->rxEndCallCount  = 0;

		/* deinit semaphore */
		HAL_SemaphoreDeinit(&xradio_i2s_priv->rxReady);

		/* deinit DMA */
		HAL_DMA_DeInit(xradio_i2s_priv->rxDMAChan);

		/* release DMA channel */
		HAL_DMA_Release(xradio_i2s_priv->rxDMAChan);
		xradio_i2s_priv->rxDMAChan = DMA_CHANNEL_INVALID;
		xradio_i2s_priv->rx_data_width = 0;

		/* free tx buf for DMA */
		XRADIO_I2S_FREE(xradio_i2s_priv->rxBuf);
		xradio_i2s_priv->rxBuf = NULL;
		xradio_i2s_priv->rxBufSize = 0;

		xradio_i2s_priv->isRxInit = false;
	}

	return HAL_OK;
}

static int xradio_i2s_pcm_read(uint8_t *buf, uint32_t size)
{
	uint8_t xrun_flag;
	uint8_t *pdata = buf;
	uint32_t read_total = 0;
	uint8_t *read_poiter_cur = NULL;
	uint32_t read_single = xradio_i2s_priv->rxBufSize/2;

	if (!buf || size <= 0){
		XRADIO_I2S_ERR("Rx record buf|size NULL, buf-[0x%08x], size-[%d]!\n",(uint32_t)buf, size);
		return HAL_INVALID;
	}

	if (read_single == 0) {
		XRADIO_I2S_ERR("RxBuf not exist\n");
		return HAL_ERROR;
	}

	if (size < read_single) {
		XRADIO_I2S_ERR("Read size too small\n");
		return HAL_INVALID;
	}

	for(; size/read_single; pdata += read_single, read_total += read_single, size -= read_single)
	{
		if (xradio_i2s_priv->rxRunning == false) {
			/* trigger record, start DMA */
			xradio_i2s_dma_trigger(PCM_IN, true);
			XRADIO_I2S_DBG("Rx: record start...\n");
		}

		{
			xrun_flag = 0;
			HAL_DisableIRQ();

			//read_poiter_cur = xradio_i2s_priv->readPointer;

			/* check DMA state */
			if (xradio_i2s_priv->rxHalfCallCount && xradio_i2s_priv->rxEndCallCount) {
				xrun_flag = 1;
				xradio_i2s_priv->rxHalfCallCount = 0;		//overrun
				xradio_i2s_priv->rxEndCallCount  = 0;
			} else if (xradio_i2s_priv->rxHalfCallCount) {	//DMA half end
				xradio_i2s_priv->rxHalfCallCount --;
			} else if (xradio_i2s_priv->rxEndCallCount) {	//DMA end
				xradio_i2s_priv->rxEndCallCount --;
			} else {										//DMA transporting

				/* wait DMA transport end */
				xradio_i2s_priv->isRxSemaphore = true;
				HAL_EnableIRQ();
				HAL_SemaphoreWait(&(xradio_i2s_priv->rxReady), HAL_WAIT_FOREVER);
				HAL_DisableIRQ();

				/* check DMA state */
				if (xradio_i2s_priv->rxHalfCallCount && xradio_i2s_priv->rxEndCallCount) {
					xrun_flag = 1;
					xradio_i2s_priv->rxHalfCallCount = 0;
					xradio_i2s_priv->rxEndCallCount  = 0;
				} else if (xradio_i2s_priv->rxHalfCallCount) {
					xradio_i2s_priv->rxHalfCallCount --;
				} else if (xradio_i2s_priv->rxEndCallCount) {
					xradio_i2s_priv->rxEndCallCount --;
				}
			}

			if (xradio_i2s_priv->rxDmaPointer == xradio_i2s_priv->rxBuf) {
				read_poiter_cur = xradio_i2s_priv->rxBuf + read_single;
			} else {
				read_poiter_cur = xradio_i2s_priv->rxBuf;
			}
			XRADIO_I2S_MEMCPY(pdata, read_poiter_cur, read_single);
			HAL_EnableIRQ();

			if (xrun_flag) {
				XRADIO_I2S_ERR("Rx overrun...\n");
			}
		}
	}

	return read_total;
}

static int xradio_i2s_pcm_write(uint8_t *buf, uint32_t size)
{
	uint8_t xrun_flag;
	uint8_t *pdata = buf;
	uint32_t write_total = 0;
	uint8_t *write_poiter_cur = NULL;
	uint32_t write_single = xradio_i2s_priv->txBufSize/2;

	if (!buf || size <= 0) {
		XRADIO_I2S_ERR("Tx play buf|size NULL, buf-[0x%08x], size-[%d]!\n",(uint32_t)buf, size);
		return HAL_INVALID;
	}

	if (write_single == 0) {
		XRADIO_I2S_ERR("TxBuf not exist\n");
		return HAL_ERROR;
	}

	if (size < write_single) {
		XRADIO_I2S_ERR("Write size too small\n");
		return HAL_INVALID;
	}

	for (; size/write_single; pdata += write_single, write_total += write_single, size -= write_single)
	{
		if (xradio_i2s_priv->txRunning == false) {
			if (!xradio_i2s_priv->writePointer){
				xradio_i2s_priv->writePointer = xradio_i2s_priv->txBuf;
				if(size >= write_single*2){
					write_poiter_cur = xradio_i2s_priv->txBuf;
					XRADIO_I2S_MEMCPY(write_poiter_cur, pdata, write_single);
					pdata += write_single;
					write_total += write_single;
					size -= write_single;
					xradio_i2s_priv->writePointer = xradio_i2s_priv->txBuf + write_single;
				}
			}
			write_poiter_cur = xradio_i2s_priv->writePointer;
			XRADIO_I2S_MEMCPY(write_poiter_cur, pdata, write_single);

			/* trigger play, start DMA */
			xradio_i2s_dma_trigger(PCM_OUT, true);
			XRADIO_I2S_DBG("Tx: play start...\n");

		} else {

			xrun_flag = 0;
			HAL_DisableIRQ();

			/* check DMA state */
			if (xradio_i2s_priv->txHalfCallCount && xradio_i2s_priv->txEndCallCount) {
				xrun_flag = 1;
				xradio_i2s_priv->txHalfCallCount = 0;		//underrun
				xradio_i2s_priv->txEndCallCount  = 0;
			} else if (xradio_i2s_priv->txHalfCallCount) {	//DMA half end
				xradio_i2s_priv->txHalfCallCount --;
			} else if (xradio_i2s_priv->txEndCallCount) {	//DMA end
				xradio_i2s_priv->txEndCallCount --;
			} else {										//DMA transporting

				/* wait DMA transport end */
				xradio_i2s_priv->isTxSemaphore = true;
				HAL_EnableIRQ();
				HAL_SemaphoreWait(&(xradio_i2s_priv->txReady), HAL_WAIT_FOREVER);
				HAL_DisableIRQ();

				/* check DMA state */
				if (xradio_i2s_priv->txHalfCallCount && xradio_i2s_priv->txEndCallCount) {
					xrun_flag = 1;
					xradio_i2s_priv->txHalfCallCount = 0;
					xradio_i2s_priv->txEndCallCount  = 0;
				} else if (xradio_i2s_priv->txHalfCallCount){
					xradio_i2s_priv->txHalfCallCount --;
				} else if (xradio_i2s_priv->txEndCallCount){
					xradio_i2s_priv->txEndCallCount --;
				}
			}

			if (xradio_i2s_priv->txDmaPointer == xradio_i2s_priv->txBuf) {
				write_poiter_cur = xradio_i2s_priv->txBuf + write_single;
				xradio_i2s_priv->writePointer = xradio_i2s_priv->txBuf;
			} else {
				write_poiter_cur = xradio_i2s_priv->txBuf;
				xradio_i2s_priv->writePointer =  xradio_i2s_priv->txBuf + write_single;
			}
			XRADIO_I2S_MEMCPY(write_poiter_cur, pdata, write_single);
			HAL_EnableIRQ();

			if (xrun_flag) {
				XRADIO_I2S_ERR("Tx : underrun....\n");
			}
		}
	}

	return write_total;
}

static int xradio_i2s_ioctl(uint32_t cmd, uint32_t cmd_param[], uint32_t cmd_param_len)
{
	int ret = HAL_ERROR;
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	switch(cmd){
		case PLATFORM_IOCTL_HW_CONFIG:
			if(cmd_param_len != 3)	return HAL_INVALID;
			xradio_i2s_priv->mclk_div = cmd_param[0] & 0xff;
			xradio_i2s_priv->slot_width = cmd_param[0]>>8  & 0xff;
			xradio_i2s_priv->pcm_frame_mode = cmd_param[0]>>16  & 0xff;
			xradio_i2s_priv->loop_back_en = cmd_param[0]>>24  & 0xff;
			xradio_i2s_priv->lrck_period = cmd_param[1];
			xradio_i2s_priv->daudio_clk_freq = cmd_param[2];
			XRADIO_I2S_DBG("\n/*** Xradio I2S hardware params config ***/\n");
			XRADIO_I2S_DBG("mclk_div: %d;\nslot_width: %d;\npcm_frame_mode: %d;\nloop_back_en: %d;\nlrck_period: %d;\ndaudio_clk_freq: %d;\n",\
							xradio_i2s_priv->mclk_div, xradio_i2s_priv->slot_width, xradio_i2s_priv->pcm_frame_mode,\
							xradio_i2s_priv->loop_back_en, xradio_i2s_priv->lrck_period, xradio_i2s_priv->daudio_clk_freq);
			XRADIO_I2S_DBG("/********************************************/");
			ret = HAL_OK;
			break;

		case PLATFORM_IOCTL_SW_CONFIG:
			if(cmd_param_len != 1)	return HAL_INVALID;
			xradio_i2s_priv->tx_underrun_threshold = cmd_param[0] & 0xffff;
			xradio_i2s_priv->rx_overrun_threshold = cmd_param[0]>>16 & 0xffff;
			XRADIO_I2S_DBG("tx_underrun_threshold: %d; rx_overrun_threshold: %d\n",\
							xradio_i2s_priv->tx_underrun_threshold, xradio_i2s_priv->rx_overrun_threshold);
			ret = HAL_OK;
			break;

		default:
			XRADIO_I2S_ERR("Invalid ioctl command!\n");
			return HAL_INVALID;
	}

	return ret;
}


static HAL_Status xradio_i2s_init(void)
{
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	if(xradio_i2s_priv == NULL){
		XRADIO_I2S_ERR("Xradio I2S hasn't been registered, please register first!");
		return HAL_ERROR;
	}

	if (xradio_i2s_priv->isI2SInit == true){
		XRADIO_I2S_DBG("I2S has init already\n");
		return HAL_OK;
	}

	/* I2S pinmux init */
	HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_I2S, 0), 0);

	/* I2S module BUS CLK&reset and module CLK init */
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_DAUDIO);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_DAUDIO);
	HAL_CCM_DAUDIO_SetMClock(XRADIO_I2S_MODULE_CLK_SRC);
	HAL_CCM_DAUDIO_EnableMClock();

	xradio_i2s_priv->isI2SInit = true;

	return HAL_OK;
}

static void xradio_i2s_deinit(void)
{
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	if (xradio_i2s_priv->isI2SInit == false){
		XRADIO_I2S_DBG("I2S has deinit already\n");
		return;
	}

	/* I2S pinmux deinit */
	HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_I2S, 0), 0);

	/* I2S module BUS CLK&reset and module CLK deinit */
	HAL_CCM_DAUDIO_DisableMClock();
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_DAUDIO);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_DAUDIO);

	xradio_i2s_priv->isI2SInit = false;
}


/*** platform dai ops ***/
static const struct platform_dai_ops xradio_i2s_dai_ops = {
	.set_fmt = xradio_i2s_dai_set_fmt,
	.set_clkdiv = xradio_i2s_set_clkdiv,
	.hw_params = xradio_i2s_dai_hw_params,
	.hw_free = xradio_i2s_dai_hw_free,
};

/*** platform ops ***/
static const struct platform_ops xradio_i2s_ops = {
	.open = xradio_i2s_open,
	.close = xradio_i2s_close,

	.pcm_read = xradio_i2s_pcm_read,
	.pcm_write = xradio_i2s_pcm_write,

	.ioctl = xradio_i2s_ioctl,
};

/*** platform driver ***/
static struct platform_driver xradio_i2s_drv = {
	.name = XRADIO_PLATFORM_I2S_NAME,
	.platform_attr = XRADIO_PLATFORM_I2S,

	.init = xradio_i2s_init,
	.deinit = xradio_i2s_deinit,

	.dai_ops = &xradio_i2s_dai_ops,
	.platform_ops = &xradio_i2s_ops,
};


HAL_Status xradio_i2s_register(void)
{
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	/* Malloc xradio_i2s_priv buffer */
	xradio_i2s_priv = (struct Xradio_I2S_Priv *)XRADIO_I2S_MALLOC(sizeof(struct Xradio_I2S_Priv));
	if(xradio_i2s_priv == NULL){
		XRADIO_I2S_ERR("Malloc struct Xradio_I2S_Priv buffer Fail!\n");
		return HAL_ERROR;
	}

	XRADIO_I2S_MEMSET(xradio_i2s_priv, 0, sizeof(struct Xradio_I2S_Priv));
	xradio_i2s_priv->mclk_div = 2;
	xradio_i2s_priv->slot_width = 32;
	xradio_i2s_priv->lrck_period = 32;
	xradio_i2s_priv->loop_back_en = 0;
	xradio_i2s_priv->pcm_frame_mode = PCM_SHORT_FRAME;
	xradio_i2s_priv->daudio_clk_freq = AUDIO_CLK_24M;
	xradio_i2s_priv->tx_underrun_threshold = XRADIO_I2S_DEF_UNDERRUN_THRES;
	xradio_i2s_priv->rx_overrun_threshold = XRADIO_I2S_DEF_OVERRUN_THRES;

	/* Platform list add */
	list_add(&xradio_i2s_drv.node, &hal_snd_platform_list);

	return HAL_OK;
}

HAL_Status xradio_i2s_unregister(void)
{
	struct platform_driver *i2s_drv_ptr;
	XRADIO_I2S_DBG("--->%s\n",__FUNCTION__);

	/* Check snd platform list empty or not */
	if(list_empty(&hal_snd_platform_list)){
		XRADIO_I2S_DBG("Hal snd platform list is empty, don't need to unregister\n");
		return HAL_OK;
	}

	/* Get platform to unregister */
	list_for_each_entry(i2s_drv_ptr, &hal_snd_platform_list, node){
		if(i2s_drv_ptr == &xradio_i2s_drv){
			list_del(&xradio_i2s_drv.node);
			break;
		}
	}

	/* Free xradio_i2s_priv buffer */
	if(xradio_i2s_priv){
		XRADIO_I2S_FREE(xradio_i2s_priv);
		xradio_i2s_priv = NULL;
	}

	return HAL_OK;
}


#endif /* __CONFIG_BOOTLOADER */


