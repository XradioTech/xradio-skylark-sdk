/**
  * @file  xradio_internal_codec.c
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

#include <stdbool.h>
#include "audio_arch.h"
#include "driver/chip/hal_dma.h"

#include "xradio_internal_codec.h"


//Debug config
#define XRADIO_CODEC_DBG_EN				0
#define XRADIO_CODEC_ERR_EN				1

#define XRADIO_CODEC_DBG(fmt, arg...)	HAL_LOG(XRADIO_CODEC_DBG_EN, "[XRADIO_INTERNAL_CODEC] "fmt, ##arg)
#define XRADIO_CODEC_ERR(fmt, arg...)	HAL_LOG(XRADIO_CODEC_ERR_EN, "[XRADIO_INTERNAL_CODEC] "fmt, ##arg)
#define XRADIO_CODEC_ALWAYS(fmt,arg...) HAL_LOG(1, "[XRADIO_INTERNAL_CODEC] "fmt, ##arg)

#define XRADIO_CODEC_IT_ERR(fmt, arg...)	HAL_IT_LOG(XRADIO_CODEC_ERR_EN, "[XRADIO_INTERNAL_CODEC] "fmt, ##arg)


//Xradio Codec config
#define XRADIO_DEFAULT_PLAY_VOLUME		VOLUME_LEVEL31
#define XRADIO_DEFAULT_RECORD_GAIN		VOLUME_GAIN_0dB

#define XRADIO_PLAY_UNDERRUN_THRESHOLD  3//256//
#define XRADIO_RECORD_OVERRUN_THRESHOLD 3//256//

//Interface define
#define XRADIO_CODEC_MALLOC             HAL_Malloc
#define XRADIO_CODEC_FREE               HAL_Free
#define XRADIO_CODEC_MEMCPY             HAL_Memcpy
#define XRADIO_CODEC_MEMSET             HAL_Memset


//Xradio codec priv struct
struct Xradio_Codec_Priv {
	//codec status contrl
	bool isCodecInit;
	bool isTxInit;
	bool isRxInit;
	volatile bool txRunning;
	volatile bool rxRunning;

	//buffer control
	uint8_t *txBuf;
	uint8_t *rxBuf;
	uint8_t *writePointer;
	uint8_t *readPointer;
	uint32_t txBufSize;
	uint32_t rxBufSize;

	//DMA control
	uint8_t *txDmaPointer;
	uint8_t *rxDmaPointer;
	DMA_Channel	txDMAChan;
	DMA_Channel	rxDMAChan;
	DMA_DataWidth tx_data_width;
	DMA_DataWidth rx_data_width;
	volatile uint8_t txHalfCallCount;
	volatile uint8_t rxHalfCallCount;
	volatile uint8_t txEndCallCount;
	volatile uint8_t rxEndCallCount;

	//Semaphore control
	HAL_Semaphore txReady;
	HAL_Semaphore rxReady;
	bool isTxSemaphore;
	bool isRxSemaphore;
} ;

static struct Xradio_Codec_Priv *xradio_codec_priv;


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

static const struct pll_param xradio_pll_params[] = {
	{HOSC_CLOCK_26M, AUDIO_CLK_24M, PRCM_AUD_PLL24M_PARAM_HOSC26M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC26M},
	{HOSC_CLOCK_26M, AUDIO_CLK_22M, PRCM_AUD_PLL22M_PARAM_HOSC26M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC26M},
	{HOSC_CLOCK_24M, AUDIO_CLK_24M, PRCM_AUD_PLL24M_PARAM_HOSC24M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC24M},
	{HOSC_CLOCK_24M, AUDIO_CLK_22M, PRCM_AUD_PLL22M_PARAM_HOSC24M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC24M},
	{HOSC_CLOCK_40M, AUDIO_CLK_24M, PRCM_AUD_PLL24M_PARAM_HOSC40M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC40M},
	{HOSC_CLOCK_40M, AUDIO_CLK_22M, PRCM_AUD_PLL22M_PARAM_HOSC40M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC40M},
	{HOSC_CLOCK_52M, AUDIO_CLK_24M, PRCM_AUD_PLL24M_PARAM_HOSC52M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC52M},
	{HOSC_CLOCK_52M, AUDIO_CLK_22M, PRCM_AUD_PLL22M_PARAM_HOSC52M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC52M},
};

static const struct real_val_to_reg_val xradio_sample_rate[] = {
	{48000, 0},
	{32000, 1},
	{24000, 2},
	{16000, 3},
	{12000, 4},
	{8000,  5},

	{44100, 0},
	{22050, 2},
	{11025, 4},

	{192000,6},
	{96000, 7},
};

static const struct real_val_to_reg_val xradio_maic_mic_pga_gain[] = {
	{VOLUME_GAIN_0dB,  0},
	{VOLUME_GAIN_21dB, 1},
	{VOLUME_GAIN_24dB, 2},
	{VOLUME_GAIN_27dB, 3},
	{VOLUME_GAIN_30dB, 4},
	{VOLUME_GAIN_33dB, 5},
	{VOLUME_GAIN_36dB, 6},
	{VOLUME_GAIN_39dB, 7},
};

static const struct real_val_to_reg_val xradio_linein_pga_gain[] = {
	{VOLUME_GAIN_MINUS_3dB,  0},
	{VOLUME_GAIN_0dB,  1},
	{VOLUME_GAIN_3dB,  2},
	{VOLUME_GAIN_6dB,  3},
	{VOLUME_GAIN_9dB,  4},
	{VOLUME_GAIN_12dB, 5},
	{VOLUME_GAIN_18dB, 6},
	{VOLUME_GAIN_24dB, 7},
};

static const struct real_val_to_reg_val xradio_lineout_gain[] = {
	{VOLUME_GAIN_0dB, 		 31},
	{VOLUME_GAIN_MINUS_3dB,  29},
	{VOLUME_GAIN_MINUS_6dB,  27},
	{VOLUME_GAIN_MINUS_9dB,  25},
	{VOLUME_GAIN_MINUS_12dB, 23},
	{VOLUME_GAIN_MINUS_15dB, 21},
	{VOLUME_GAIN_MINUS_18dB, 19},
	{VOLUME_GAIN_MINUS_21dB, 17},
	{VOLUME_GAIN_MINUS_24dB, 15},
	{VOLUME_GAIN_MINUS_27dB, 13},
	{VOLUME_GAIN_MINUS_30dB, 11},
	{VOLUME_GAIN_MINUS_33dB, 9},
	{VOLUME_GAIN_MINUS_36dB, 7},
	{VOLUME_GAIN_MINUS_39dB, 5},
	{VOLUME_GAIN_MINUS_42dB, 3},
};


#if 0
#define xradio_codec_reg_read(reg)						(HAL_REG_32BIT(reg+CODEC_BASE))
#define xradio_codec_reg_write(reg, val)				(HAL_REG_32BIT(reg+CODEC_BASE) = val)
#define xradio_codec_reg_update_bits(reg, mask ,val)	(HAL_MODIFY_REG(HAL_REG_32BIT(reg+CODEC_BASE), mask ,val))
#endif

//Base read/write Interface
__sram_text
static int xradio_codec_reg_read(uint32_t reg)
{
	return HAL_REG_32BIT(reg+CODEC_BASE);
}

__sram_text
static int xradio_codec_reg_write(uint32_t reg, uint32_t val)
{
	HAL_REG_32BIT(reg+CODEC_BASE) = val;

	return HAL_OK;
}

__sram_text
static int xradio_codec_reg_update_bits(uint32_t reg, uint32_t mask, uint32_t val)
{
	uint32_t val_old,val_new;

	val_old = xradio_codec_reg_read(reg);
	val_new = (val_old & ~mask) | (val & mask);

	if(val_new != val_old){
		xradio_codec_reg_write(reg, val_new);
	}

	return HAL_OK;
}


/*************************** XRADIO Codec DMA Control ****************************/
__sram_text
static void xradio_codec_dma_trigger(Audio_Stream_Dir dir, bool enable)
{
	uint32_t flags;
	uint8_t doProtection = !HAL_IsISRContext();

	if (doProtection) {
		flags = HAL_EnterCriticalSection();
	}

	if(enable){
		if(dir == PCM_OUT){
			//Flush TX_FIFO
			xradio_codec_reg_update_bits(AC_DAC_FIFO_CTRL, 0x1<<DAC_FIFO_FLUSH_BIT, 0x1<<DAC_FIFO_FLUSH_BIT);
			if(xradio_codec_priv->txDMAChan != DMA_CHANNEL_INVALID){
				//DAC DRQ Enable
				xradio_codec_reg_update_bits(AC_DAC_FIFO_CTRL, 0x1<<DAC_DRQ_EN_BIT, 0x1<<DAC_DRQ_EN_BIT);
				//DMA Start
				HAL_DMA_Start(xradio_codec_priv->txDMAChan, (uint32_t)xradio_codec_priv->txBuf, (uint32_t)(AC_DAC_TXDATA+CODEC_BASE), xradio_codec_priv->txBufSize);
			}
			xradio_codec_priv->txRunning = true;
		} else{
			//Flush RX_FIFO
			xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_FIFO_FLUSH_BIT, 0x1<<ADC_FIFO_FLUSH_BIT);
			if(xradio_codec_priv->rxDMAChan != DMA_CHANNEL_INVALID){
				//ADC DRQ Enable
				xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_DRQ_EN_BIT, 0x1<<ADC_DRQ_EN_BIT);
				HAL_DMA_Start(xradio_codec_priv->rxDMAChan, (uint32_t)(AC_ADC_RXDATA+CODEC_BASE), (uint32_t)xradio_codec_priv->rxBuf, xradio_codec_priv->rxBufSize);
			}
			xradio_codec_priv->rxRunning = true;
		}
	} else {
		if(dir == PCM_OUT){
			//Flush TX_FIFO
			xradio_codec_reg_update_bits(AC_DAC_FIFO_CTRL, 0x1<<DAC_FIFO_FLUSH_BIT, 0x1<<DAC_FIFO_FLUSH_BIT);
			if(xradio_codec_priv->txDMAChan != DMA_CHANNEL_INVALID){
				//DAC DRQ Disable
				xradio_codec_reg_update_bits(AC_DAC_FIFO_CTRL, 0x1<<DAC_DRQ_EN_BIT, 0x0<<DAC_DRQ_EN_BIT);
				//DMA Stop
				HAL_DMA_Stop(xradio_codec_priv->txDMAChan);
			}
			xradio_codec_priv->txRunning = false;
		} else{
			//Flush RX_FIFO
			xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_FIFO_FLUSH_BIT, 0x1<<ADC_FIFO_FLUSH_BIT);
			if(xradio_codec_priv->rxDMAChan != DMA_CHANNEL_INVALID){
				//ADC DRQ Disable
				xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_DRQ_EN_BIT, 0x0<<ADC_DRQ_EN_BIT);
				HAL_DMA_Stop(xradio_codec_priv->rxDMAChan);
			}
			xradio_codec_priv->rxRunning = false;
		}
	}

	if (doProtection) {
		HAL_ExitCriticalSection(flags);
	}
}

__sram_text
static int xradio_codec_dma_threshold_check(Audio_Stream_Dir dir)
{
	if(dir == PCM_OUT){
		if(xradio_codec_priv->txHalfCallCount >= XRADIO_PLAY_UNDERRUN_THRESHOLD ||\
		   xradio_codec_priv->txEndCallCount  >= XRADIO_PLAY_UNDERRUN_THRESHOLD){
		   XRADIO_CODEC_IT_ERR("Tx : underrun and stop dma tx...\n");
		   xradio_codec_dma_trigger(PCM_OUT, false);
		   //xradio_codec_priv->txRunning = false;	//has been config in trigger interface
		   xradio_codec_priv->writePointer = NULL;
		   xradio_codec_priv->txDmaPointer = NULL;
		   xradio_codec_priv->txHalfCallCount = 0;
		   xradio_codec_priv->txEndCallCount = 0;
		   return HAL_ERROR;
		}
	} else {
		if(xradio_codec_priv->rxHalfCallCount >= XRADIO_RECORD_OVERRUN_THRESHOLD ||\
		   xradio_codec_priv->rxEndCallCount  >= XRADIO_RECORD_OVERRUN_THRESHOLD){
		   XRADIO_CODEC_IT_ERR("Rx : overrun and stop dma rx...\n");
		   xradio_codec_dma_trigger(PCM_IN, false);
		   //xradio_codec_priv->rxRunning = false;	//has been config in trigger interface
		   xradio_codec_priv->readPointer = NULL;
		   xradio_codec_priv->rxDmaPointer = NULL;
		   xradio_codec_priv->rxHalfCallCount = 0;
		   xradio_codec_priv->rxEndCallCount = 0;
		   return HAL_ERROR;
		}
	}

	return 0;
}

__sram_text
static void xradio_codec_dma_half_callback(void *arg)
{
	if(arg == &xradio_codec_priv->txReady){
		xradio_codec_priv->txHalfCallCount++;
		if(xradio_codec_priv->isTxSemaphore){
			xradio_codec_priv->isTxSemaphore = false;
			HAL_SemaphoreRelease((HAL_Semaphore *)arg);
		}
		if(xradio_codec_dma_threshold_check(PCM_OUT))
			return;
		xradio_codec_priv->txDmaPointer = xradio_codec_priv->txBuf + xradio_codec_priv->txBufSize/2;
	} else {
		xradio_codec_priv->rxHalfCallCount++;
		if(xradio_codec_priv->isRxSemaphore){
			xradio_codec_priv->isRxSemaphore = false;
			HAL_SemaphoreRelease((HAL_Semaphore *)arg);
		}
		if(xradio_codec_dma_threshold_check(PCM_IN))
			return;
		xradio_codec_priv->rxDmaPointer = xradio_codec_priv->rxBuf + xradio_codec_priv->rxBufSize/2;
	}
}

__sram_text
static void xradio_codec_dma_end_callback(void *arg)
{
	if(arg == &xradio_codec_priv->txReady){
		xradio_codec_priv->txEndCallCount++;
		if(xradio_codec_priv->isTxSemaphore){
			xradio_codec_priv->isTxSemaphore = false;
			HAL_SemaphoreRelease((HAL_Semaphore *)arg);
		}
		if(xradio_codec_dma_threshold_check(PCM_OUT))
			return;
		xradio_codec_priv->txDmaPointer = xradio_codec_priv->txBuf;
	} else {
		xradio_codec_priv->rxEndCallCount++;
		if(xradio_codec_priv->isRxSemaphore){
			xradio_codec_priv->isRxSemaphore = false;
			HAL_SemaphoreRelease((HAL_Semaphore *)arg);
		}
		if(xradio_codec_dma_threshold_check(PCM_IN))
			return;
		xradio_codec_priv->rxDmaPointer = xradio_codec_priv->rxBuf;
	}
}

static int xradio_codec_dma_init(Audio_Stream_Dir dir, DMA_Channel channel)
{
#if HAL_DMA_OPT_TRANSFER_HALF_IRQ

	DMA_ChannelInitParam dmaParam;
	XRADIO_CODEC_MEMSET(&dmaParam, 0, sizeof(dmaParam));

	if (dir == PCM_OUT) {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(
						DMA_WORK_MODE_CIRCULAR,
						DMA_WAIT_CYCLE_2,
						DMA_BYTE_CNT_MODE_REMAIN,
						xradio_codec_priv->tx_data_width,
						DMA_BURST_LEN_1,
						DMA_ADDR_MODE_FIXED,
						DMA_PERIPH_AUDIO_CODEC,
						xradio_codec_priv->tx_data_width,
						DMA_BURST_LEN_1,
						DMA_ADDR_MODE_INC,
						DMA_PERIPH_SRAM
						);
		dmaParam.halfArg = &(xradio_codec_priv->txReady);
		dmaParam.endArg  = &(xradio_codec_priv->txReady);
	} else {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(
						DMA_WORK_MODE_CIRCULAR,
						DMA_WAIT_CYCLE_2,
						DMA_BYTE_CNT_MODE_REMAIN,
						xradio_codec_priv->rx_data_width,
						DMA_BURST_LEN_1,
						DMA_ADDR_MODE_INC,
						DMA_PERIPH_SRAM,
						xradio_codec_priv->rx_data_width,
						DMA_BURST_LEN_1,
						DMA_ADDR_MODE_FIXED,
						DMA_PERIPH_AUDIO_CODEC
						);
		dmaParam.halfArg = &(xradio_codec_priv->rxReady);
		dmaParam.endArg  = &(xradio_codec_priv->rxReady);
	}

	dmaParam.irqType = DMA_IRQ_TYPE_BOTH;
	dmaParam.halfCallback = xradio_codec_dma_half_callback;
	dmaParam.endCallback  = xradio_codec_dma_end_callback;

	HAL_DMA_Init(channel, &dmaParam);
	return HAL_OK;

#else

	XRADIO_CODEC_ERR("DMA don't support Half IRQ, dma init Fail!\n");
	return HAL_ERROR;

#endif
}

/*****************************************************************************/


static void xradio_codec_reset(void)
{
	XRADIO_CODEC_DBG("%s, reset all register to their default value\n",__FUNCTION__);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);
}

static void xradio_codec_hw_common_init(Audio_Stream_Dir dir)
{
	//Analog voltage VRA1/ALDO/ADDA_BIAS enable
	xradio_codec_reg_update_bits(AC_POWER_CTRL, 0x1<<ADLDO_EN_BIT | 0x1<<VRA1_EN_BIT, 0x1<<ADLDO_EN_BIT | 0x1<<VRA1_EN_BIT);
	xradio_codec_reg_update_bits(AC_DAC_ANA_CTRL, 0x1<<ADDA_BIAS_EN_BIT, 0x1<<ADDA_BIAS_EN_BIT);

	if(dir == PCM_OUT){
		//DAC LINEOUT Output Select Single_ended mode
		xradio_codec_reg_update_bits(AC_DAC_ANA_CTRL, 0x1<<LINEOUT_DIFF_EN_BIT, 0x0<<LINEOUT_DIFF_EN_BIT);
	} else {
		//Wait analog voltage to be stable output
		HAL_MSleep(200);
		//ADC Digital Part Enable
		xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_DIG_EN_BIT, 0x1<<ADC_DIG_EN_BIT);

		//ADC HPF Enable
		xradio_codec_reg_update_bits(AC_ADC_HPF_CTRL, 0x1<<ADC_HPF_EN_BIT, 0x1<<ADC_HPF_EN_BIT);
	}
}

static void xradio_codec_hw_common_deinit(Audio_Stream_Dir dir)
{
	//Analog voltage VRA1/ALDO/ADDA_BIAS disable
	if(!xradio_codec_priv->isTxInit && !xradio_codec_priv->isRxInit){
		xradio_codec_reg_update_bits(AC_POWER_CTRL, 0x1<<ADLDO_EN_BIT | 0x1<<VRA1_EN_BIT, 0x0<<ADLDO_EN_BIT | 0x0<<VRA1_EN_BIT);
		xradio_codec_reg_update_bits(AC_DAC_ANA_CTRL, 0x1<<ADDA_BIAS_EN_BIT, 0x0<<ADDA_BIAS_EN_BIT);
	}

	if(dir == PCM_OUT){
		//DAC LINEOUT Output recover default Differential mode
		xradio_codec_reg_update_bits(AC_DAC_ANA_CTRL, 0x1<<LINEOUT_DIFF_EN_BIT, 0x1<<LINEOUT_DIFF_EN_BIT);
	} else {
		//ADC Digital Part Disable
		xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_DIG_EN_BIT, 0x0<<ADC_DIG_EN_BIT);

		//ADC HPF Disable
		xradio_codec_reg_update_bits(AC_ADC_HPF_CTRL, 0x1<<ADC_HPF_EN_BIT, 0x0<<ADC_HPF_EN_BIT);
	}
}

static int xradio_codec_set_audio_pll(Audio_Clk_Freq pll_freq_out)
{
	uint32_t i,hosc_freq,audioPllParam;

	if(pll_freq_out != AUDIO_CLK_24M && pll_freq_out != AUDIO_CLK_22M){
		return HAL_INVALID;
	}

	hosc_freq = HAL_GetHFClock();

	for(i=0; i<HAL_ARRAY_SIZE(xradio_pll_params); i++){
		if(xradio_pll_params[i].hosc_freq == hosc_freq && xradio_pll_params[i].pll_freq_out == pll_freq_out){
			audioPllParam = xradio_pll_params[i].pllParam;
			XRADIO_CODEC_DBG("Xradio Codec PLL freq_in match:%u, freq_out:%u\n",hosc_freq,pll_freq_out);
			break;
		}
	}

	if(i == HAL_ARRAY_SIZE(xradio_pll_params)){
		XRADIO_CODEC_ERR("Match audio pll input/output freq failed\n");
		return HAL_ERROR;
	}

	HAL_PRCM_SetAudioPLLParam(audioPllParam);
	HAL_PRCM_EnableAudioPLL();

	return HAL_OK;
}

static void xradio_codec_set_main_mic(bool enable)
{
	XRADIO_CODEC_ALWAYS("Route(cap): main mic %s\n",enable ? "Enable" : "Disable");

	//MIC PGA & ADCL Analog enable/disable
	xradio_codec_reg_update_bits(AC_ADC_ANA_CTRL, 0x1<<MIC_PGA_EN_BIT | 0x1<<ADCL_ANA_EN_BIT, !!enable<<MIC_PGA_EN_BIT | !!enable<<ADCL_ANA_EN_BIT);
	//Wait analog to be stable
	HAL_MSleep(20);
	//ADCL FIFO enable/disable
	xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADCL_FIFO_EN_BIT, !!enable<<ADCL_FIFO_EN_BIT);
}

__nonxip_text
static void xradio_codec_set_linein(bool enable)
{
	//XRADIO_CODEC_ALWAYS("Route(cap): linein %s\n",enable ? "Enable" : "Disable");

	//LINEIN PGA & ADCR Analog enable/disable
	xradio_codec_reg_update_bits(AC_ADC_ANA_CTRL, 0x1<<LINEIN_PGA_EN_BIT | 0x1<<ADCR_ANA_EN_BIT, !!enable<<LINEIN_PGA_EN_BIT | !!enable<<ADCR_ANA_EN_BIT);
	//Wait analog to be stable
	HAL_MSleep(20);
	//ADCR FIFO enable/disable
	xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADCR_FIFO_EN_BIT, !!enable<<ADCR_FIFO_EN_BIT);
}

static void xradio_codec_set_dmic(bool enable)
{
	XRADIO_CODEC_ALWAYS("Route(cap): dmic %s\n",enable ? "Enable" : "Disable");

	//xradio DMIC Pin Mux config
	HAL_BoardIoctl(enable ? HAL_BIR_PINMUX_INIT : HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_DMIC, 0), 0);

	//DMIC Digital & DMICL/R FIFO enable/disable
	xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<DMIC_DIG_EN_BIT | 0x1<<DMICL_FIFO_EN_BIT | 0x1<<DMICR_FIFO_EN_BIT,\
								!!enable<<DMIC_DIG_EN_BIT | !!enable<<DMICL_FIFO_EN_BIT | !!enable<<DMICR_FIFO_EN_BIT);
	//DMIC HPF enable/disable
	xradio_codec_reg_update_bits(AC_ADC_HPF_CTRL, 0x1<<DMIC_HPF_EN_BIT, !!enable<<DMIC_HPF_EN_BIT);
}

static void xradio_codec_set_lineout(bool enable)
{
	XRADIO_CODEC_ALWAYS("Route(play): lineout %s\n",enable ? "Enable" : "Disable");

	//DAC Digital Part enable/disable
	xradio_codec_reg_update_bits(AC_DAC_DIG_CTRL, 0x1<<DAC_DIG_EN_BIT, !!enable<<DAC_DIG_EN_BIT);
	//HAL_MSleep(20);

	//Playback Path Analog Part enable
	xradio_codec_reg_update_bits(AC_DAC_ANA_CTRL, 0x1<<PLAY_ANA_EN_BIT, !!enable<<PLAY_ANA_EN_BIT);
	//wait lineout ramp to stable, default ramp time: 24us*4096=98ms
	if(enable)	HAL_MSleep(150);
}


static int xradio_dai_set_sysclk(Codec_Sysclk_Src sysclk_src, Codec_Pllclk_Src pllclk_src, uint32_t pll_freq_in, uint32_t sample_rate)
{
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);
	int ret = HAL_OK;

	switch(sysclk_src){
		case SYSCLK_SRC_PLL:
			XRADIO_CODEC_DBG("SYSCLK source select AUDIO_PLL\n");
			xradio_codec_reg_update_bits(AC_DAC_DEBUG, 0x1<<CODEC_SYSCLK_SEL_BIT, 0x0<<CODEC_SYSCLK_SEL_BIT);
			ret = xradio_codec_set_audio_pll(sample_rate%1000 ? AUDIO_CLK_22M : AUDIO_CLK_24M);
			break;
		case SYSCLK_SRC_OSC:
			XRADIO_CODEC_DBG("SYSCLK source select OSC\n");
			xradio_codec_reg_update_bits(AC_DAC_DEBUG, 0x1<<CODEC_SYSCLK_SEL_BIT, 0x1<<CODEC_SYSCLK_SEL_BIT);
			break;
		default:
			XRADIO_CODEC_ERR("SYSCLK source select Error!\n");
			return HAL_INVALID;
	}

	return ret;
}

static int xradio_dai_set_fmt(uint32_t fmt)
{
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	return HAL_OK;
}

static int xradio_dai_set_volume(Audio_Device device, uint16_t volume)
{
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);
	uint32_t i,reg_val=0;
	uint16_t vol_set_flag, vol_set_value;

	if(volume == VOLUME_INVALID){
		if(device & AUDIO_IN_DEV_ALL){
			vol_set_flag  = VOLUME_SET_GAIN;
			vol_set_value = XRADIO_DEFAULT_RECORD_GAIN;
		} else {
			vol_set_flag  = VOLUME_SET_LEVEL;
			vol_set_value = XRADIO_DEFAULT_PLAY_VOLUME;
		}
	} else {
		vol_set_flag  = volume & VOLUME_SET_MASK;
		vol_set_value = volume & ~VOLUME_SET_MASK;
	}

	switch(device){
		case AUDIO_IN_DEV_AMIC:
			if(vol_set_flag == VOLUME_SET_LEVEL){
				if (vol_set_value > VOLUME_LEVEL7){
					XRADIO_CODEC_ERR("Invalid main mic volume level: %d!\n",vol_set_value);
					return HAL_INVALID;
				}
				reg_val = vol_set_value;
				XRADIO_CODEC_ALWAYS("AMIC set volume Level-[%d]\n",vol_set_value);
			} else if (vol_set_flag == VOLUME_SET_GAIN){
				for(i=0; i<HAL_ARRAY_SIZE(xradio_maic_mic_pga_gain); i++){
					if(xradio_maic_mic_pga_gain[i].real_val == vol_set_value){
						reg_val = xradio_maic_mic_pga_gain[i].reg_val;
						XRADIO_CODEC_ALWAYS("AMIC set volume Gain-[%d]\n",vol_set_value);
						break;
					}
				}
				if(i == HAL_ARRAY_SIZE(xradio_maic_mic_pga_gain)){
					XRADIO_CODEC_ERR("Invalid main mic volume gain: %d!\n",vol_set_value);
					return HAL_INVALID;
				}
			}

			xradio_codec_reg_update_bits(AC_ADC_ANA_CTRL, 0x7<<MIC_PGA_GAIN_BIT, reg_val<<MIC_PGA_GAIN_BIT);
			break;

		case AUDIO_IN_DEV_LINEIN:
			if(vol_set_flag == VOLUME_SET_LEVEL){
				if (vol_set_value > VOLUME_LEVEL7){
					XRADIO_CODEC_ERR("Invalid linein volume level: %d!\n",vol_set_value);
					return HAL_INVALID;
				}
				reg_val = vol_set_value;
				XRADIO_CODEC_ALWAYS("LINEIN set volume Level-[%d]\n",vol_set_value);
			} else if (vol_set_flag == VOLUME_SET_GAIN){
				for(i=0; i<HAL_ARRAY_SIZE(xradio_linein_pga_gain); i++){
					if(xradio_linein_pga_gain[i].real_val == vol_set_value){
						reg_val = xradio_linein_pga_gain[i].reg_val;
						XRADIO_CODEC_ALWAYS("LINEIN set volume Gain-[%d]\n",vol_set_value);
						break;
					}
				}
				if(i == HAL_ARRAY_SIZE(xradio_linein_pga_gain)){
					XRADIO_CODEC_ERR("Invalid linein volume gain: %d!\n",vol_set_value);
					return HAL_INVALID;
				}
			}

			xradio_codec_reg_update_bits(AC_ADC_ANA_CTRL, 0x7<<LINEIN_PGA_GAIN_BIT, reg_val<<LINEIN_PGA_GAIN_BIT);
			break;

		case AUDIO_IN_DEV_DMIC:
			XRADIO_CODEC_ERR("DMIC don't support set volume\n");
			return HAL_INVALID;

		case AUDIO_OUT_DEV_SPK:
			if(vol_set_flag == VOLUME_SET_LEVEL){
				if (vol_set_value > VOLUME_LEVEL31){
					XRADIO_CODEC_ERR("Invalid lineout volume level: %d!\n",vol_set_value);
					return HAL_INVALID;
				}
				reg_val = vol_set_value;
				XRADIO_CODEC_ALWAYS("LINEOUT set volume Level-[%d]\n",vol_set_value);
			} else if (vol_set_flag == VOLUME_SET_GAIN){
				for(i=0; i<HAL_ARRAY_SIZE(xradio_lineout_gain); i++){
					if(xradio_lineout_gain[i].real_val == vol_set_value){
						reg_val = xradio_lineout_gain[i].reg_val;
						XRADIO_CODEC_ALWAYS("LINEOUT set volume Gain-[%d]\n",vol_set_value);
						break;
					}
				}
				if(i == HAL_ARRAY_SIZE(xradio_lineout_gain)){
					XRADIO_CODEC_ERR("Invalid lineout volume gain: %d!\n",vol_set_value);
					return HAL_INVALID;
				}
			}

			xradio_codec_reg_update_bits(AC_DAC_ANA_CTRL, 0x1f<<LINEOUT_GAIN_BIT, reg_val<<LINEOUT_GAIN_BIT);
			break;

		default:
			XRADIO_CODEC_ERR("Invalid Audio Device-[0x%08x]!\n",device);
			return HAL_INVALID;
	}

	return HAL_OK;
}

__nonxip_text
static int xradio_dai_set_route(Audio_Device device, Audio_Dev_State state)
{
	//XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);
	bool enable = (state==AUDIO_DEV_EN) ? 1 : 0;

	switch(device){
		case AUDIO_IN_DEV_AMIC:
			xradio_codec_set_main_mic(enable);
			break;
		case AUDIO_IN_DEV_LINEIN:
			xradio_codec_set_linein(enable);
			break;
		case AUDIO_IN_DEV_DMIC:
			xradio_codec_set_dmic(enable);
			break;
		case AUDIO_OUT_DEV_SPK:
			xradio_codec_set_lineout(enable);
			break;
		default:
			XRADIO_CODEC_ERR("Invalid Audio Device-[0x%08x]!\n",device);
			return HAL_INVALID;
	}

	return HAL_OK;
}

static int xradio_dai_hw_params(Audio_Stream_Dir dir, struct pcm_config *pcm_cfg)
{
	//uint8_t slot_width;
	uint32_t i,dma_buf_size;
	uint8_t sample_resolution, dma_data_width;
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	/* Codec common init */
	xradio_codec_hw_common_init(dir);

	sample_resolution = pcm_format_to_sampleresolution(pcm_cfg->format);
	//slot_width = sample_resolution<=8 ? 8 : (sample_resolution<=16 ? 16 : 32);
	dma_data_width = sample_resolution<=8 ? DMA_DATA_WIDTH_8BIT : (sample_resolution<=16 ? DMA_DATA_WIDTH_16BIT : DMA_DATA_WIDTH_32BIT);
	dma_buf_size = pcm_frames_to_bytes(pcm_cfg, pcm_config_to_frames(pcm_cfg));

	if(dir == PCM_OUT){
		/* DAC sample rate config */
		for(i=0; i<HAL_ARRAY_SIZE(xradio_sample_rate); i++){
			if(xradio_sample_rate[i].real_val == pcm_cfg->rate){
				xradio_codec_reg_update_bits(AC_DAC_FIFO_CTRL, 0x7<<DAC_FS_BIT, xradio_sample_rate[i].reg_val<<DAC_FS_BIT);
				break;
			}
		}
		if(i == HAL_ARRAY_SIZE(xradio_sample_rate)){
			XRADIO_CODEC_ERR("Invalid play sample rate:%d!\n",pcm_cfg->rate);
			return HAL_INVALID;
		}

		/* DAC channel nums config */
		if(pcm_cfg->channels<=0 || pcm_cfg->channels>2){
			XRADIO_CODEC_ERR("Invalid play channel nums:%d!\n",pcm_cfg->channels);
			return HAL_INVALID;
		}

		if(pcm_cfg->channels == 1){
			xradio_codec_reg_update_bits(AC_DAC_FIFO_CTRL, 0x1<<DAC_MONO_EN_BIT, 0x1<<DAC_MONO_EN_BIT);
		} else {
			xradio_codec_reg_update_bits(AC_DAC_FIFO_CTRL, 0x1<<DAC_MONO_EN_BIT, 0x0<<DAC_MONO_EN_BIT);
		}
		xradio_codec_reg_update_bits(AC_DAC_DIG_CTRL, 0x3<<TX_MIX_CTRL_BIT, 0x3<<TX_MIX_CTRL_BIT);

		/* DAC sample resolution config */
		if(sample_resolution !=16 && sample_resolution != 24){
			XRADIO_CODEC_ERR("Invalid play sample resolution:%d!\n",sample_resolution);
			return HAL_INVALID;
		}
		xradio_codec_reg_update_bits(AC_DAC_FIFO_CTRL, 0x1<<DAC_SAMPLE_BITS_BIT, (sample_resolution/8-2)<<DAC_SAMPLE_BITS_BIT);

		/* dma tx params config */
		xradio_codec_priv->txBufSize = dma_buf_size;
		xradio_codec_priv->tx_data_width = dma_data_width;

	} else {

		/* ADC sample rate config */
		for(i=0; i<HAL_ARRAY_SIZE(xradio_sample_rate)-2; i++){
			if(xradio_sample_rate[i].real_val == pcm_cfg->rate){
				xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x7<<ADC_FS_BIT, xradio_sample_rate[i].reg_val<<ADC_FS_BIT);
				break;
			}
		}
		if(i == HAL_ARRAY_SIZE(xradio_sample_rate)-2){
			XRADIO_CODEC_ERR("Invalid record sample rate:%d!\n",pcm_cfg->rate);
			return HAL_INVALID;
		}

		/* ADC channel nums config */
		if(pcm_cfg->channels<=0 || pcm_cfg->channels>4){
			XRADIO_CODEC_ERR("Invalid record channel nums:%d!\n",pcm_cfg->channels);
			return HAL_INVALID;
		}
		//default channel enable order: ADCL->ADCR->DMICL->DMICR
		//xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0xf<<ADCL_FIFO_EN_BIT, ((1<<pcm_cfg->channels)-1)<<ADCL_FIFO_EN_BIT);

		/* ADC sample resolution config */
		if(sample_resolution !=16 && sample_resolution != 24){
			XRADIO_CODEC_ERR("Invalid record sample resolution:%d!\n",sample_resolution);
			return HAL_INVALID;
		}
		xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_SAMPLE_BITS_BIT, (sample_resolution/8-2)<<ADC_SAMPLE_BITS_BIT);

		/* dma rx params config */
		xradio_codec_priv->rxBufSize = dma_buf_size;
		xradio_codec_priv->rx_data_width = dma_data_width;
	}

	XRADIO_CODEC_DBG("Sample rate-[%d], channel numbers-[%d], sample resolution-[%d]\n",pcm_cfg->rate,pcm_cfg->channels,sample_resolution);
	return HAL_OK;
}

static int xradio_dai_hw_free(Audio_Stream_Dir dir)
{
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	xradio_codec_hw_common_deinit(dir);

	if(!xradio_codec_priv->isTxInit && !xradio_codec_priv->isRxInit){
		xradio_codec_reset();
		HAL_PRCM_DisableAudioPLL();
	}

	return HAL_OK;
}


static int xradio_codec_ioctl_pcm_read(uint8_t *buf, uint32_t size)
{
	uint8_t  xrun_flag;
	uint8_t *pdata = buf;
	uint32_t read_total = 0;
	uint8_t *read_poiter_cur = NULL;
	uint32_t read_single = xradio_codec_priv->rxBufSize/2;

	if (!buf || !size){
		//XRADIO_CODEC_ERR("Rx record buf|size NULL, buf-[0x%08x], size-[%d]!\n",(uint32_t)buf, size);
		return HAL_INVALID;
	}

	for(; size/read_single; pdata += read_single, read_total += read_single, size -= read_single)
	{
		if (xradio_codec_priv->rxRunning == false) {
			/* trigger record, start DMA */
			xradio_codec_dma_trigger(PCM_IN, true);
			XRADIO_CODEC_DBG("Rx: record start...\n");
		}

		{
			xrun_flag = 0;
			HAL_DisableIRQ();

			//read_poiter_cur = xradio_codec_priv->readPointer;

			/* check DMA state */
			if (xradio_codec_priv->rxHalfCallCount && xradio_codec_priv->rxEndCallCount) {
				xrun_flag = 1;
				xradio_codec_priv->rxHalfCallCount = 0;			//overrun
				xradio_codec_priv->rxEndCallCount  = 0;
			} else if (xradio_codec_priv->rxHalfCallCount) {	//DMA half end
				xradio_codec_priv->rxHalfCallCount --;
			} else if (xradio_codec_priv->rxEndCallCount) {		//DMA end
				xradio_codec_priv->rxEndCallCount --;
			} else {											//DMA transporting

				/* wait DMA transport end */
				xradio_codec_priv->isRxSemaphore = true;
				HAL_EnableIRQ();
				HAL_SemaphoreWait(&(xradio_codec_priv->rxReady), HAL_WAIT_FOREVER);
				HAL_DisableIRQ();

				if (xradio_codec_priv->rxHalfCallCount && xradio_codec_priv->rxEndCallCount) {
					xrun_flag = 1;
					xradio_codec_priv->rxHalfCallCount = 0;
					xradio_codec_priv->rxEndCallCount  = 0;
				} else if (xradio_codec_priv->rxHalfCallCount) {
					xradio_codec_priv->rxHalfCallCount --;
				} else if (xradio_codec_priv->rxEndCallCount) {
					xradio_codec_priv->rxEndCallCount --;
				}
			}

			if (xradio_codec_priv->rxDmaPointer == xradio_codec_priv->rxBuf) {
				read_poiter_cur = xradio_codec_priv->rxBuf + read_single;
			} else {
				read_poiter_cur = xradio_codec_priv->rxBuf;
			}
			XRADIO_CODEC_MEMCPY(pdata, read_poiter_cur, read_single);
			HAL_EnableIRQ();

			if (xrun_flag) {
				XRADIO_CODEC_ERR("Rx overrun, (H:%u,F:%u)\n",xradio_codec_priv->rxHalfCallCount, xradio_codec_priv->rxEndCallCount);
			}
		}
	}

	return read_total;
}


static int xradio_codec_ioctl_pcm_write(uint8_t *buf, uint32_t size)
{
	uint8_t  xrun_flag;
	uint8_t *pdata = buf;
	uint32_t write_total = 0;
	uint8_t *write_poiter_cur = NULL;
	uint32_t write_single = xradio_codec_priv->txBufSize/2;

	if (!buf || !size){
		//XRADIO_CODEC_ERR("Tx play buf|size NULL, buf-[0x%08x], size-[%d]!\n",(uint32_t)buf, size);
		return HAL_INVALID;
	}

	for(; size/write_single; pdata += write_single, write_total += write_single, size -= write_single)
	{
		if(xradio_codec_priv->txRunning == false){
			if (!xradio_codec_priv->writePointer){
				xradio_codec_priv->writePointer = xradio_codec_priv->txBuf;
				if(size >= write_single*2){
					write_poiter_cur = xradio_codec_priv->txBuf;
					XRADIO_CODEC_MEMCPY(write_poiter_cur, pdata, write_single);
					pdata += write_single;
					write_total += write_single;
					size -= write_single;
					xradio_codec_priv->writePointer = xradio_codec_priv->txBuf + write_single;
				}
			}
			write_poiter_cur = xradio_codec_priv->writePointer;
			XRADIO_CODEC_MEMCPY(write_poiter_cur, pdata, write_single);

			/* trigger play, start DMA */
			xradio_codec_dma_trigger(PCM_OUT, true);
			XRADIO_CODEC_DBG("Tx: play start...\n");

		} else {

			xrun_flag = 0;
			HAL_DisableIRQ();

			/* check DMA state */
			if (xradio_codec_priv->txHalfCallCount && xradio_codec_priv->txEndCallCount) {
				xrun_flag = 1;
				xradio_codec_priv->txHalfCallCount = 0;			//underrun
				xradio_codec_priv->txEndCallCount  = 0;
			} else if (xradio_codec_priv->txHalfCallCount) {	//DMA half end
				xradio_codec_priv->txHalfCallCount --;
			} else if (xradio_codec_priv->txEndCallCount) {		//DMA end
				xradio_codec_priv->txEndCallCount --;
			} else {											//DMA transporting

				/* wait DMA transport end */
				xradio_codec_priv->isTxSemaphore = true;
				HAL_EnableIRQ();
				HAL_SemaphoreWait(&(xradio_codec_priv->txReady), HAL_WAIT_FOREVER);
				HAL_DisableIRQ();

				/* check DMA state */
				if (xradio_codec_priv->txHalfCallCount && xradio_codec_priv->txEndCallCount) {
					xrun_flag = 1;
					xradio_codec_priv->txHalfCallCount = 0;
					xradio_codec_priv->txEndCallCount  = 0;
				} else if (xradio_codec_priv->txHalfCallCount) {
					xradio_codec_priv->txHalfCallCount --;
				} else if (xradio_codec_priv->txEndCallCount) {
					xradio_codec_priv->txEndCallCount --;
				}
			}

			/* write data to play  */
			if (xradio_codec_priv->txDmaPointer == xradio_codec_priv->txBuf) {
				write_poiter_cur = xradio_codec_priv->txBuf + write_single;
				xradio_codec_priv->writePointer = xradio_codec_priv->txBuf;
			} else {
				write_poiter_cur = xradio_codec_priv->txBuf;
				xradio_codec_priv->writePointer =  xradio_codec_priv->txBuf + write_single;
			}
			XRADIO_CODEC_MEMCPY(write_poiter_cur, pdata, write_single);
			HAL_EnableIRQ();

			if(xrun_flag){
				XRADIO_CODEC_ERR("Tx underrun, (H:%u,F:%u)\n",xradio_codec_priv->txHalfCallCount, xradio_codec_priv->txEndCallCount);
			}
		}
	}

	return write_total;
}

static int xradio_codec_ioctl_set_adda_direct(Audio_Device device, Audio_Dev_State state)
{
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	if (state == AUDIO_DEV_EN) {
		//DAC Mixer Select ADC_DAT
		xradio_codec_reg_update_bits(AC_DAC_DIG_CTRL, 0x3<<DAC_MIX_CTRL_BIT, 0x1<<DAC_MIX_CTRL_BIT);
		xradio_codec_set_lineout(1);

		switch(device) {
			case AUDIO_IN_DEV_AMIC:
				xradio_codec_set_main_mic(1);
				xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_DMIC_EN_BIT, 0x0<<ADC_DMIC_EN_BIT);				 //ADC_DMIC MUX Select ADC
				xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_TO_DAC_MUX_SEL_BIT, 0x0<<ADC_TO_DAC_MUX_SEL_BIT);//ADC_TO_DAC MUX Select ADC Left Channel
				break;
			case AUDIO_IN_DEV_LINEIN:
				xradio_codec_set_linein(1);
				xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_DMIC_EN_BIT, 0x0<<ADC_DMIC_EN_BIT);				 //ADC_DMIC MUX Select ADC
				xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_TO_DAC_MUX_SEL_BIT, 0x1<<ADC_TO_DAC_MUX_SEL_BIT);//ADC_TO_DAC MUX Select ADC Right Channel
				break;
			case AUDIO_IN_DEV_DMIC:
				//xradio_codec_set_dmic(1);
				HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_DMIC, 0), 0);
				xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_DMIC_EN_BIT, 0x1<<ADC_DMIC_EN_BIT);				 //ADC_DMIC MUX Select DMIC
				xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_TO_DAC_MUX_SEL_BIT, 0x0<<ADC_TO_DAC_MUX_SEL_BIT);//ADC_TO_DAC MUX Select DMIC Left Channel
				break;
			default:
				XRADIO_CODEC_ERR("Invalid Audio Device-[0x%08x]!\n",device);
				return HAL_INVALID;
		}
	} else if (state == AUDIO_DEV_DIS) {
		//DAC Mixer && ADC_DMIC MUX && ADC_TO_DAC MUX recover to default status
		xradio_codec_reg_update_bits(AC_DAC_DIG_CTRL, 0x3<<DAC_MIX_CTRL_BIT, 0x0<<DAC_MIX_CTRL_BIT);				//DAC Mixer Select TX_MIX_OUT
		xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_DMIC_EN_BIT, 0x0<<ADC_DMIC_EN_BIT);					//ADC_DMIC MUX Select ADC
		xradio_codec_reg_update_bits(AC_ADC_FIFO_CTRL, 0x1<<ADC_TO_DAC_MUX_SEL_BIT, 0x0<<ADC_TO_DAC_MUX_SEL_BIT);	//ADC_TO_DAC MUX Select ADC Left Channel
		xradio_codec_set_lineout(0);

		switch(device) {
			case AUDIO_IN_DEV_AMIC:
				xradio_codec_set_main_mic(0);
				break;
			case AUDIO_IN_DEV_LINEIN:
				xradio_codec_set_linein(0);
				break;
			case AUDIO_IN_DEV_DMIC:
				//xradio_codec_set_dmic(0);
				HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_DMIC, 0), 0);
				break;
			default:
				XRADIO_CODEC_ERR("Invalid Audio Device-[0x%08x]!\n",device);
				return HAL_INVALID;
		}
	}

	return HAL_OK;
}

static int xradio_codec_ioctl(uint32_t cmd, uint32_t cmd_param[], uint32_t cmd_param_len)
{
	int ret = HAL_ERROR;
	//XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	switch(cmd){
		case CODEC_IOCTL_PCM_READ:
			if(cmd_param_len != 2)	return HAL_INVALID;
			ret = xradio_codec_ioctl_pcm_read((uint8_t *)cmd_param[0], cmd_param[1]);
			break;
		case CODEC_IOCTL_PCM_WRITE:
			if(cmd_param_len != 2)	return HAL_INVALID;
			ret = xradio_codec_ioctl_pcm_write((uint8_t *)cmd_param[0], cmd_param[1]);
			break;
		case CODEC_IOCTL_SET_ADDA_DIRECT:
			if(cmd_param_len != 2)	return HAL_INVALID;
			ret = xradio_codec_ioctl_set_adda_direct((Audio_Device)cmd_param[0], (Audio_Dev_State)cmd_param[1]);
			break;
		default:
			XRADIO_CODEC_ERR("Invalid ioctl command!\n");
			return HAL_INVALID;
	}

	return ret;
}


static int xradio_codec_open(Audio_Stream_Dir dir)
{
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	if(dir == PCM_OUT){
		if(xradio_codec_priv->isTxInit == true){
			XRADIO_CODEC_DBG("Codec play device has opened already\n");
			return HAL_OK;
		}
		if(!xradio_codec_priv->txBufSize){
			XRADIO_CODEC_ERR("TX DMA buffer size is NULL, please set it through IOCTL cmd before codec open!\n");
			return HAL_ERROR;
		}
	} else {
		if(xradio_codec_priv->isRxInit == true){
			XRADIO_CODEC_DBG("Codec record device has opened already\n");
			return HAL_OK;
		}
		if(!xradio_codec_priv->rxBufSize){
			XRADIO_CODEC_ERR("RX DMA buffer size is NULL, please set it through IOCTL cmd before codec open!\n");
			return HAL_ERROR;
		}
	}

	if(dir == PCM_OUT){
		xradio_codec_priv->txDMAChan = DMA_CHANNEL_INVALID;
		//xradio_codec_priv->txBufSize = param->bufSize;
		xradio_codec_priv->writePointer = NULL;
		xradio_codec_priv->txHalfCallCount = 0;
		xradio_codec_priv->txEndCallCount  = 0;

		/* malloc tx buf for DMA */
		xradio_codec_priv->txBuf = XRADIO_CODEC_MALLOC(xradio_codec_priv->txBufSize);
		if(xradio_codec_priv->txBuf == NULL){
			XRADIO_CODEC_ERR("Malloc Tx buf for DMA faild!\n");
			xradio_codec_priv->txBufSize = 0;
			return HAL_ERROR;
		}
		XRADIO_CODEC_MEMSET(xradio_codec_priv->txBuf, 0, xradio_codec_priv->txBufSize);

		/* request DMA channel */
		xradio_codec_priv->txDMAChan = HAL_DMA_Request();
		if(xradio_codec_priv->txDMAChan == DMA_CHANNEL_INVALID){
			XRADIO_CODEC_ERR("Request tx DMA channel faild!\n");
			XRADIO_CODEC_FREE(xradio_codec_priv->txBuf);
			xradio_codec_priv->txBuf = NULL;
			xradio_codec_priv->txBufSize = 0;
			return HAL_ERROR;
		}

		/* init DMA */
		xradio_codec_dma_init(PCM_OUT, xradio_codec_priv->txDMAChan);

		/* init semaphore */
		HAL_SemaphoreInitBinary(&xradio_codec_priv->txReady);

		xradio_codec_priv->isTxInit = true;

	} else {

		xradio_codec_priv->rxDMAChan = DMA_CHANNEL_INVALID;
		//xradio_codec_priv->rxBufSize = param->bufSize;
		xradio_codec_priv->readPointer = NULL;
		xradio_codec_priv->rxHalfCallCount = 0;
		xradio_codec_priv->rxEndCallCount  = 0;

		/* malloc rx buf for DMA */
		xradio_codec_priv->rxBuf = XRADIO_CODEC_MALLOC(xradio_codec_priv->rxBufSize);
		if(xradio_codec_priv->rxBuf == NULL){
			XRADIO_CODEC_ERR("Malloc Rx buf for DMA faild!\n");
			xradio_codec_priv->rxBufSize = 0;
			return HAL_ERROR;
		}
		XRADIO_CODEC_MEMSET(xradio_codec_priv->rxBuf, 0, xradio_codec_priv->rxBufSize);

		/* request DMA channel */
		xradio_codec_priv->rxDMAChan = HAL_DMA_Request();
		if(xradio_codec_priv->rxDMAChan == DMA_CHANNEL_INVALID){
			XRADIO_CODEC_ERR("Request rx DMA channel faild!\n");
			XRADIO_CODEC_FREE(xradio_codec_priv->rxBuf);
			xradio_codec_priv->rxBuf = NULL;
			xradio_codec_priv->rxBufSize = 0;
			return HAL_ERROR;
		}

		/* init DMA */
		xradio_codec_dma_init(PCM_IN, xradio_codec_priv->rxDMAChan);

		/* init semaphore */
		HAL_SemaphoreInitBinary(&xradio_codec_priv->rxReady);

		xradio_codec_priv->isRxInit = true;
	}

#if XRADIO_CODEC_DBG_EN
	uint32_t i;
	printf("\nXradio Codec Reg:");
	for(i=0; i<72; i+=4){
		if(!(i%16))	printf("\n");
		printf("Reg[0x%02x] :0x%08x;  ",i,HAL_REG_32BIT(CODEC_BASE+i));
	}
	printf("\n\n");
#endif

	return HAL_OK;
}

static int xradio_codec_close(Audio_Stream_Dir dir)
{
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	if(dir == PCM_OUT){
		if(xradio_codec_priv->isTxInit == false){
			XRADIO_CODEC_DBG("Codec play device has closed already\n");
			return HAL_OK;
		}
	} else {
		if(xradio_codec_priv->isRxInit == false){
			XRADIO_CODEC_DBG("Codec record device has closed already\n");
			return HAL_OK;
		}
	}

	if(dir == PCM_OUT){
		xradio_codec_dma_trigger(PCM_OUT, false);
		//xradio_codec_priv->txRunning = false;	//has been config in trigger interface
        xradio_codec_priv->writePointer = NULL;
        xradio_codec_priv->txHalfCallCount = 0;
        xradio_codec_priv->txEndCallCount  = 0;

		/* deinit semaphore */
		HAL_SemaphoreDeinit(&xradio_codec_priv->txReady);

		/* deinit DMA */
		HAL_DMA_DeInit(xradio_codec_priv->txDMAChan);

		/* release DMA channel */
		HAL_DMA_Release(xradio_codec_priv->txDMAChan);
		xradio_codec_priv->txDMAChan = DMA_CHANNEL_INVALID;
		xradio_codec_priv->tx_data_width = 0;

		/* free tx buf for DMA */
		XRADIO_CODEC_FREE(xradio_codec_priv->txBuf);
        xradio_codec_priv->txBuf = NULL;
        xradio_codec_priv->txBufSize = 0;

		xradio_codec_priv->isTxInit = false;

	} else {

		xradio_codec_dma_trigger(PCM_IN, false);
		//xradio_codec_priv->rxRunning = false;	//has been config in trigger interface
        xradio_codec_priv->readPointer = NULL;
        xradio_codec_priv->rxHalfCallCount = 0;
        xradio_codec_priv->rxEndCallCount  = 0;

		/* deinit semaphore */
		HAL_SemaphoreDeinit(&xradio_codec_priv->rxReady);

		/* deinit DMA */
		HAL_DMA_DeInit(xradio_codec_priv->rxDMAChan);

		/* release DMA channel */
		HAL_DMA_Release(xradio_codec_priv->rxDMAChan);
		xradio_codec_priv->rxDMAChan = DMA_CHANNEL_INVALID;
		xradio_codec_priv->rx_data_width = 0;

		/* free tx buf for DMA */
		XRADIO_CODEC_FREE(xradio_codec_priv->rxBuf);
        xradio_codec_priv->rxBuf = NULL;
        xradio_codec_priv->rxBufSize = 0;

		xradio_codec_priv->isRxInit = false;
	}

	return HAL_OK;
}


static int xradio_internal_codec_init(void)
{
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	if(xradio_codec_priv->isCodecInit == true){
		XRADIO_CODEC_DBG("codec has init already\n");
		return HAL_OK;
	}

	//CODEC module CLK gating & reset & MCLK release
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);
	HAL_CCM_AudioCodec_EnableMClock();

	xradio_codec_priv->isCodecInit = true;

	return HAL_OK;
}

static void xradio_internal_codec_deinit(void)
{
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	if(xradio_codec_priv->isCodecInit == false){
		XRADIO_CODEC_DBG("codec has deinit already\n");
		return;
	}

	//CODEC module CLK gating & reset & MCLK lock
	HAL_CCM_AudioCodec_DisableMClock();
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_AUDIO_CODEC);

	xradio_codec_priv->isCodecInit = false;
}


/*** codec dai ops ****/
static const struct codec_dai_ops xradio_codec_dai_ops = {
	.set_sysclk = xradio_dai_set_sysclk,
	.set_fmt    = xradio_dai_set_fmt,
	.set_volume = xradio_dai_set_volume,
	.set_route  = xradio_dai_set_route,
	.hw_params  = xradio_dai_hw_params,
	.hw_free    = xradio_dai_hw_free,
};

/*** codec ops ****/
static const struct codec_ops xradio_codec_ops = {
	.open  = xradio_codec_open,
	.close = xradio_codec_close,

	.reg_read  = xradio_codec_reg_read,
	.reg_write = xradio_codec_reg_write,

	.ioctl = xradio_codec_ioctl,
};


/*** codec driver ****/
static struct codec_driver xradio_internal_codec_drv = {
	.name = XRADIO_INTERNAL_CODEC_NAME,
	.codec_attr = XRADIO_CODEC_INTERNAL,

	.init = xradio_internal_codec_init,
	.deinit = xradio_internal_codec_deinit,

	.dai_ops = &xradio_codec_dai_ops,
	.codec_ops = &xradio_codec_ops,
};


HAL_Status xradio_internal_codec_register(void)
{
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	/* Malloc xradio codec priv buffer */
	xradio_codec_priv = (struct Xradio_Codec_Priv *)XRADIO_CODEC_MALLOC(sizeof(struct Xradio_Codec_Priv));
	if(xradio_codec_priv == NULL){
		XRADIO_CODEC_ERR("Malloc Xradio_Codec_Priv buffer Fail!\n");
		return HAL_ERROR;
	}
	XRADIO_CODEC_MEMSET(xradio_codec_priv, 0, sizeof(struct Xradio_Codec_Priv));

	/* Codec list add */
	list_add(&xradio_internal_codec_drv.node, &hal_snd_codec_list);

	return HAL_OK;
}

HAL_Status xradio_internal_codec_unregister(void)
{
	struct codec_driver *codec_drv_ptr;
	XRADIO_CODEC_DBG("--->%s\n",__FUNCTION__);

	/* Check snd codec list empty or not */
	if(list_empty(&hal_snd_codec_list)){
		XRADIO_CODEC_DBG("Hal snd codec list is empty, don't need to unregister\n");
		return HAL_OK;
	}

	/* Get codec to unregister */
	list_for_each_entry(codec_drv_ptr, &hal_snd_codec_list, node){
		if(codec_drv_ptr == &xradio_internal_codec_drv){
			list_del(&xradio_internal_codec_drv.node);
			break;
		}
	}

	/* Free  xradio codec priv buffer */
	if(xradio_codec_priv){
		XRADIO_CODEC_FREE(xradio_codec_priv);
		xradio_codec_priv = NULL;
	}

	return HAL_OK;
}


