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

#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_i2s.h"
#include "hal_base.h"
#include "pm/pm.h"
#include "./codec/audio_arch.h"

#define I2D_DBG_ON                0
#define LOOP_EN					  0

#if (I2D_DBG_ON == 1)
#define I2S_DEBUG(fmt, arg...)    HAL_LOG(I2D_DBG_ON, "[I2S] "fmt, ##arg)
#else
#define I2S_DEBUG(fmt, arg...)
#endif
#define I2S_ERROR(fmt, arg...)    HAL_LOG(1, "[I2S] "fmt, ##arg)

/* debug in interrupt handler */
#ifdef __CONFIG_SECTION_ATTRIBUTE_NONXIP
#define I2S_IT_ERROR(fmt, arg...) HAL_IT_LOG(1, "[I2S] "fmt, ##arg)
#else
#define I2S_IT_ERROR              I2S_ERROR
#endif /* __CONFIG_SECTION_ATTRIBUTE_NONXIP */

struct I2S_Private{
    volatile bool               isHwInit;
    volatile bool               txRunning;
    volatile bool               rxRunning;

    uint8_t                     *txBuf;
    uint8_t                     *rxBuf;
    uint8_t                     *readPointer;
    uint8_t                     *writePointer;
	uint32_t					rxBufSize;
	uint32_t					txBufSize;

    DMA_Channel                 txDMAChan;
    DMA_Channel                 rxDMAChan;
    I2S_HWParam                 *hwParam;
    I2S_DataParam               pdataParam;
    I2S_DataParam               cdataParam;
    volatile uint8_t           txHalfCallCount;
    volatile uint8_t           rxHalfCallCount;
    volatile uint8_t           txEndCallCount;
    volatile uint8_t           rxEndCallCount;
    uint8_t                     *txDmaPointer;
    uint8_t                     *rxDmaPointer;

    HAL_Semaphore               txReady;
    HAL_Semaphore               rxReady;
    bool                        isTxSemaphore;
    bool                        isRxSemaphore;
    bool                        isTxInitiate;
    bool                        isRxInitiate;

    HAL_Mutex                   devSetLock;

    uint32_t                    audioPllParam;
    uint32_t                    audioPllPatParam;
#ifdef CONFIG_PM
	uint32_t                    suspend;
	struct soc_device           dev;
#endif
};

typedef struct {
        uint32_t        hosc;
        uint32_t        audio;
        uint32_t        pllParam;
        uint32_t        pllPatParam;
} HOSC_I2S_Type;

typedef enum {
        I2S_PLL_24M        = 0U,
        I2S_PLL_22M        = 1U,
} I2S_PLLMode;

typedef struct {
	uint8_t       clkDiv;
	I2S_BCLKDIV   bregVal;
	I2S_MCLKDIV   mregVal;
} CLK_DIVRegval;

#define AUDIO_PLL_SRC               (CCM_DAUDIO_MCLK_SRC_1X)
#define AUDIO_DEVICE_PLL             AUDIO_CLK_22M

#define I2S_MEMCPY                   HAL_Memcpy
#define I2S_MALLOC                   HAL_Malloc
#define I2S_FREE                     HAL_Free
#define I2S_MEMSET                   HAL_Memset

#define UNDERRUN_THRESHOLD           (LOOP_EN ? 256 :3)
#define OVERRUN_THRESHOLD            (LOOP_EN ? 256 :3)

#ifdef RESERVERD_MEMORY_FOR_I2S_TX
static uint8_t I2STX_BUF[I2S_BUF_LENGTH];
#endif
#ifdef RESERVERD_MEMORY_FOR_I2S_RX
static uint8_t I2SRX_BUF[I2S_BUF_LENGTH];
#endif

void HAL_I2S_Trigger(bool enable,Audio_Stream_Dir dir);
static struct I2S_Private *gI2sPrivate;

/*
 *default hw configuration
 */
static I2S_HWParam gHwParam = {
        0,                  /*0:I2S,1:PCM*/
        DAIFMT_CBS_CFS,
        DAIFMT_I2S,
        DAIFMT_NB_NF,
        32,                 /*16,32,64,128,256*/
        I2S_SHORT_FRAME,
        I2S_TX_MSB_FIRST,
        I2S_RX_MSB_FIRST,
        0x40,
        0xF,
        {AUDIO_DEVICE_PLL,1},
        1,
};

static const CLK_DIVRegval DivRegval[] = {
        {1,   I2S_BCLKDIV_1,   I2S_MCLKDIV_1},
        {2,   I2S_BCLKDIV_2,   I2S_MCLKDIV_2},
        {4,   I2S_BCLKDIV_4,   I2S_MCLKDIV_4},
        {6,   I2S_BCLKDIV_6,   I2S_MCLKDIV_6},
        {8,   I2S_BCLKDIV_8,   I2S_MCLKDIV_8},
        {12,  I2S_BCLKDIV_12,  I2S_MCLKDIV_12},
        {16,  I2S_BCLKDIV_16,  I2S_MCLKDIV_16},
        {24,  I2S_BCLKDIV_24,  I2S_MCLKDIV_24},
        {32,  I2S_BCLKDIV_32,  I2S_MCLKDIV_32},
        {48,  I2S_BCLKDIV_48,  I2S_MCLKDIV_48},
        {64,  I2S_BCLKDIV_64,  I2S_MCLKDIV_64},
        {96,  I2S_BCLKDIV_96,  I2S_MCLKDIV_96},
        {128, I2S_BCLKDIV_128, I2S_MCLKDIV_128},
        {176, I2S_BCLKDIV_176, I2S_MCLKDIV_176},
        {192, I2S_BCLKDIV_192, I2S_MCLKDIV_192},
};

static const HOSC_I2S_Type i2s_hosc_aud_type[] = {
        {HOSC_CLOCK_26M, I2S_PLL_24M, PRCM_AUD_PLL24M_PARAM_HOSC26M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC26M},
        {HOSC_CLOCK_26M, I2S_PLL_22M, PRCM_AUD_PLL22M_PARAM_HOSC26M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC26M},
        {HOSC_CLOCK_24M, I2S_PLL_24M, PRCM_AUD_PLL24M_PARAM_HOSC24M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC24M},
        {HOSC_CLOCK_24M, I2S_PLL_22M, PRCM_AUD_PLL22M_PARAM_HOSC24M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC24M},
        {HOSC_CLOCK_40M, I2S_PLL_24M, PRCM_AUD_PLL24M_PARAM_HOSC40M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC40M},
        {HOSC_CLOCK_40M, I2S_PLL_22M, PRCM_AUD_PLL22M_PARAM_HOSC40M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC40M},
        {HOSC_CLOCK_52M, I2S_PLL_24M, PRCM_AUD_PLL24M_PARAM_HOSC52M, PRCM_AUD_PLL24M_PAT_PARAM_HOSC52M},
        {HOSC_CLOCK_52M, I2S_PLL_22M, PRCM_AUD_PLL22M_PARAM_HOSC52M, PRCM_AUD_PLL22M_PAT_PARAM_HOSC52M},
};

/**
  * @internal
  * @brief Set i2s clock frequency
  * @pll I2S clock identifier
  *         This parameter can be one of the following values:
  *            @arg @ref I2S_PLL_22M
  *            @arg @ref I2S_PLL_24M
  * @retval return 0 means success otherwise fail
  */
uint32_t I2S_PLLAUDIO_Update(I2S_PLLMode pll)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;

        if (pll != I2S_PLL_24M &&  pll != I2S_PLL_22M)
                return -1;

        uint32_t hoscClock = HAL_GetHFClock();

        int i = 0;
        for (i = 0; i < HAL_ARRAY_SIZE(i2s_hosc_aud_type); i++) {
                if ((i2s_hosc_aud_type[i].hosc == hoscClock) && (i2s_hosc_aud_type[i].audio == pll)) {
                        i2sPrivate->audioPllParam = i2s_hosc_aud_type[i].pllParam;
                        i2sPrivate->audioPllPatParam = i2s_hosc_aud_type[i].pllPatParam;
                        break;
                }
        }
        if (i == HAL_ARRAY_SIZE(i2s_hosc_aud_type)) {
                I2S_ERROR("Update audio pll failed....\n");
                return -1;
        }
        return 0;
}

/**
  * @internal
  * @brief Enable/disable I2S tx
  * @retval None
  */
static void I2S_DisableTx()
{
        HAL_CLR_BIT(I2S->DA_CTL, I2S_TX_EN_BIT);
}

static void I2S_EnableTx()
{
        HAL_SET_BIT(I2S->DA_CTL, I2S_TX_EN_BIT);
}

/**
  * @internal
  * @brief Enable/disable I2S rx
  * @retval None
  */
static void I2S_DisableRx()
{
        HAL_CLR_BIT(I2S->DA_CTL, I2S_RX_EN_BIT);
}

static void I2S_EnableRx()
{
        HAL_SET_BIT(I2S->DA_CTL, I2S_RX_EN_BIT);
}

/**
  * @internal
  * @brief set the i2s module clock frequency for a given peripheral clk
  * @param isEnable: flag for i2s mclk output
  * @param clkSource: Peripheral clock freq
  * @param pll: the freq of mclk
  * @retval HAL status
  */
static HAL_Status I2S_SET_Mclk(uint32_t isEnable, uint32_t clkSource, uint32_t pll)
{
        if (isEnable == 0) {
                HAL_CLR_BIT(I2S->DA_CLKD, I2S_MCLK_OUT_EN_BIT);
        } else {
                uint32_t mclkDiv;
                const CLK_DIVRegval *divRegval;

                if (clkSource == 0) {
                        I2S_ERROR("invalid clkSource %u\n", clkSource);
                        return HAL_INVALID;
                }
                mclkDiv = clkSource / pll;
                divRegval = DivRegval;

                do {
                        if (divRegval->clkDiv == mclkDiv) {
                                HAL_MODIFY_REG(I2S->DA_CLKD, I2S_MCLKDIV_MASK, divRegval->mregVal);
                                break;
                        }
                        divRegval++;
                } while (divRegval->mregVal < I2S_MCLKDIV_192);
                HAL_SET_BIT(I2S->DA_CLKD, I2S_MCLK_OUT_EN_BIT);
        }
        return HAL_OK;
}

/**
  * @internal
  * @brief set the Sample Resolution
  * @param param: pointer to a I2S_DataParam structure that contains
  *        data format information
  * @retval HAL status
  */
static HAL_Status I2S_SET_SampleResolution(I2S_DataParam *param)
{
        if (!param)
                return HAL_INVALID;
        uint8_t sample_resolution = pcm_format_to_sampleresolution(param->resolution);	//I2S_SR16BIT;//temporary test, pjw

        if (8 <= sample_resolution &&  sample_resolution <= 32)
                HAL_MODIFY_REG(I2S->DA_FMT0, I2S_SR_MASK, (sample_resolution/4-1)<<I2S_SR_SHIFT);
        else {
                I2S_ERROR("Invalid sample resolution (%d) failed\n",sample_resolution);
                return HAL_ERROR;
        }
        return HAL_OK;
}

/**
  * @internal
  * @brief set the i2s bclk lrck freq
  * @param param: pointer to a I2S_DataParam structure that contains
  *        data format information
  * @param hwParam: pointer to a I2S_HWParam structure that contains
  *        the configuration for clk/mode/format.
  * @retval HAL status
  */
static HAL_Status I2S_SET_ClkDiv(I2S_DataParam *param,  I2S_HWParam *hwParam)
{
        int32_t ret = HAL_OK;
        if (!param || !hwParam)
                return HAL_INVALID;
        uint32_t rate = param->sampleRate;
       // I2S_SampleRate SR = param->sampleRate;
        uint32_t Period = hwParam->lrckPeriod;
        uint16_t bclkDiv = 0;
        uint32_t audioPll = AUDIO_CLK_24M;

		switch (rate) {
			case 48000:
			case 44100:
			case 8000:
			case 12000:
			case 16000:
			case 24000:
			case 32000:
			case 11025:
			case 22050:
				break;
			default:
				I2S_ERROR("Invalid sample rate(%x) failed...\n",rate);
                return HAL_INVALID;
		}

        I2S_DEBUG("SAMPLE RATE:%d...\n",rate);
        if ((rate % 1000) != 0)
                audioPll = AUDIO_CLK_22M;

        struct I2S_Private *i2sPrivate = gI2sPrivate;

        /*set sysclk*/
        if (audioPll == AUDIO_CLK_24M) {	//48KHZ series
                I2S_PLLAUDIO_Update(I2S_PLL_24M);
                HAL_PRCM_SetAudioPLLParam(i2sPrivate->audioPllParam);
                HAL_PRCM_SetAudioPLLPatternParam(i2sPrivate->audioPllPatParam);
        } else { //44.1KHZ series
                I2S_PLLAUDIO_Update(I2S_PLL_22M);
                HAL_PRCM_SetAudioPLLParam(i2sPrivate->audioPllParam);
                HAL_PRCM_SetAudioPLLPatternParam(i2sPrivate->audioPllPatParam);
        }

        /*config bclk pll div*/
        if (!hwParam->i2sFormat)
                bclkDiv = audioPll/(2*Period*rate);
        else
                bclkDiv = audioPll/(Period*rate);

        const CLK_DIVRegval *divRegval = DivRegval;
        do {
                if (divRegval->clkDiv == bclkDiv) {
                        HAL_MODIFY_REG(I2S->DA_CLKD, I2S_BCLKDIV_MASK, divRegval->bregVal);
                        break;
                }
                divRegval++;
        } while (divRegval->bregVal < I2S_BCLKDIV_192);

        return ret;
}

/**
  * @internal
  * @brief set the i2s transfer format
  * @param param: pointer to a I2S_HWParam structure that contains
  *        the configuration for clk/mode/format.
  * @retval HAL status
  */
static HAL_Status I2S_SET_Format(I2S_HWParam *param)
{
        int32_t ret = HAL_OK;
        if (!param)
                return HAL_INVALID;

        /* config clk mode*/
        switch (param->clkMode) {
                case DAIFMT_CBM_CFM:
                        HAL_MODIFY_REG(I2S->DA_CTL, I2S_BCLK_OUT_MASK|I2S_LRCK_OUT_MASK|I2S_SDO0_EN_BIT,
                                        I2S_BCLK_INPUT|I2S_LRCK_INPUT|I2S_SDO0_EN_BIT);

                        break;
                case DAIFMT_CBS_CFS:
                        HAL_MODIFY_REG(I2S->DA_CTL, I2S_BCLK_OUT_MASK|I2S_LRCK_OUT_MASK|I2S_SDO0_EN_BIT,
                                        I2S_BCLK_OUTPUT|I2S_LRCK_OUTPUT|I2S_SDO0_EN_BIT);
                        break;
                default:
                        ret = HAL_INVALID;
                        I2S_ERROR("Invalid DAI format,failed (%d)...\n",param->clkMode);
                        break;
        }

        /* config transfer format */
        switch (param->transferFormat) {
                case DAIFMT_I2S:
                        HAL_MODIFY_REG(I2S->DA_CTL, I2S_MODE_SEL_MASK, I2S_LEFT_MODE);
                        HAL_MODIFY_REG(I2S->DA_TX0CHSEL, I2S_TXN_OFFSET_MASK, I2S_TX_ONEBCLK_OFFSET);
                        HAL_MODIFY_REG(I2S->DA_RXCHSEL, I2S_RXN_OFFSET_MASK, I2S_RX_ONEBCLK_OFFSET);
                        break;
                case DAIFMT_RIGHT_J:
                        HAL_MODIFY_REG(I2S->DA_CTL, I2S_MODE_SEL_MASK, I2S_RIGHT_MODE);
                        break;
                case DAIFMT_LEFT_J:
                        HAL_MODIFY_REG(I2S->DA_CTL, I2S_MODE_SEL_MASK, I2S_LEFT_MODE);
                        HAL_MODIFY_REG(I2S->DA_TX0CHSEL, I2S_TXN_OFFSET_MASK, I2S_TX_NO_OFFSET);
                        HAL_MODIFY_REG(I2S->DA_RXCHSEL, I2S_RXN_OFFSET_MASK, I2S_RX_NO_OFFSET);
                        break;
                case DAIFMT_DSP_A:
                        HAL_MODIFY_REG(I2S->DA_CTL, I2S_MODE_SEL_MASK, I2S_PCM_MODE);
                        HAL_MODIFY_REG(I2S->DA_TX0CHSEL, I2S_TXN_OFFSET_MASK, I2S_TX_ONEBCLK_OFFSET);
                        HAL_MODIFY_REG(I2S->DA_RXCHSEL, I2S_RXN_OFFSET_MASK, I2S_RX_ONEBCLK_OFFSET);
                        break;
                case DAIFMT_DSP_B:
                        HAL_MODIFY_REG(I2S->DA_CTL, I2S_MODE_SEL_MASK, I2S_PCM_MODE);
                        HAL_MODIFY_REG(I2S->DA_TX0CHSEL, I2S_TXN_OFFSET_MASK, I2S_TX_NO_OFFSET);
                        HAL_MODIFY_REG(I2S->DA_RXCHSEL, I2S_RXN_OFFSET_MASK, I2S_RX_NO_OFFSET);
                        break;
                default:
                        ret = HAL_INVALID;
                        I2S_ERROR("Invalid transfer format,failed (%d)...\n",param->transferFormat);
                        break;
        }

        /* config signal interval */
        switch (param->signalInterval) {
                case DAIFMT_NB_NF:
                        HAL_CLR_BIT(I2S->DA_FMT0, I2S_LRCK_POLARITY_MASK);
                        HAL_CLR_BIT(I2S->DA_FMT0, I2S_BCLK_POLARITY_MASK);
                        break;
                case DAIFMT_NB_IF:
                        HAL_SET_BIT(I2S->DA_FMT0, I2S_LRCK_POLARITY_MASK);
                        HAL_CLR_BIT(I2S->DA_FMT0, I2S_BCLK_POLARITY_MASK);
                        break;
                case DAIFMT_IB_NF:
                        HAL_CLR_BIT(I2S->DA_FMT0, I2S_LRCK_POLARITY_MASK);
                        HAL_SET_BIT(I2S->DA_FMT0, I2S_BCLK_POLARITY_MASK);
                        break;
                case DAIFMT_IB_IF:
                        HAL_SET_BIT(I2S->DA_FMT0, I2S_LRCK_POLARITY_MASK);
                        HAL_SET_BIT(I2S->DA_FMT0, I2S_BCLK_POLARITY_MASK);
                        break;
                default:
                        ret = HAL_INVALID;
                        I2S_ERROR("Invalid signal Interval,failed (%d)...\n",param->signalInterval);
                        break;
        }
        return ret;
}

/**
  * @internal
  * @brief set the number channels of i2s transfer
  * @param param: pointer to a I2S_DataParam structure that contains
  *         data format information.
  * @retval HAL status
  */
static HAL_Status I2S_SET_Channels(I2S_DataParam *param)
{
        uint8_t channel = 0;
        if (!param)
                return HAL_INVALID;

        if (param->direction == PCM_OUT) {/*play*/
                if (param->channels < I2S_TX_SLOT_NUM1 || param->channels > I2S_TX_SLOT_NUM8) {
                        I2S_ERROR("Invalid usr tx channels num,failed (%d)...\n",param->channels);
                        return HAL_INVALID;
                }
                HAL_MODIFY_REG(I2S->DA_CHCFG, I2S_TX_SLOT_NUM_MASK, ((param->channels - 1) << I2S_TX_SLOT_NUM_SHIFT));
                HAL_MODIFY_REG(I2S->DA_TX0CHSEL, I2S_TXN_CHANNEL_SEL_MASK,
                                                 ((param->channels -1) << I2S_TXN_CHANNEL_SEL_SHIFT));
                HAL_MODIFY_REG(I2S->DA_TX0CHSEL, I2S_TXN_CHANNEL_SLOT_ENABLE_MASK,
                                                 I2S_TXN_CHANNEL_SLOTS_ENABLE(param->channels));
                for (channel = 0; channel < param->channels; channel++) {
                        HAL_MODIFY_REG(I2S->DA_TX0CHMAP, I2S_TXN_CHX_MAP_MASK(channel),I2S_TXN_CHX_MAP(channel));
                }

        } else {/*record*/
                if (param->channels < I2S_RX_SLOT_NUM1 || param->channels > I2S_RX_SLOT_NUM8){
                        I2S_ERROR("Invalid usr rx channels num,failed (%d)...\n",param->channels);
                        return HAL_INVALID;
                }

                HAL_MODIFY_REG(I2S->DA_CHCFG, I2S_RX_SLOT_NUM_MASK, ((param->channels - 1) << I2S_RX_SLOT_NUM_SHIFT));
                HAL_MODIFY_REG(I2S->DA_RXCHSEL, I2S_RXN_CHANNEL_SEL_MASK,
                                                ((param->channels - 1) << I2S_RXN_CHANNEL_SEL_SHIFT));
                for (channel = 0; channel < param->channels; channel++) {
                        HAL_MODIFY_REG(I2S->DA_RXCHMAP, I2S_RXN_CHX_MAP_MASK(channel), I2S_RXN_CHX_MAP(channel));
                }
        }
        return HAL_OK;
}

__nonxip_text
static int I2S_DMA_BUFFER_CHECK_Threshold(Audio_Stream_Dir dir)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;
        if (dir == PCM_OUT) {
                if (i2sPrivate->txHalfCallCount >= UNDERRUN_THRESHOLD ||
                        i2sPrivate->txEndCallCount >= UNDERRUN_THRESHOLD) {
                        I2S_IT_ERROR("Tx : underrun and stop dma tx...\n");
                        HAL_I2S_Trigger(false,PCM_OUT);/*stop*/
                        i2sPrivate->txRunning = false;
                        i2sPrivate->writePointer = NULL;
                        i2sPrivate->txHalfCallCount = 0;
                        i2sPrivate->txEndCallCount = 0;
                        i2sPrivate->txDmaPointer = NULL;
                        return -1;
                }
        } else {
                if (i2sPrivate->rxHalfCallCount >= OVERRUN_THRESHOLD ||
                        i2sPrivate->rxEndCallCount >= OVERRUN_THRESHOLD) {
                        I2S_IT_ERROR("Rx : overrun and stop dma rx...\n");
                        HAL_I2S_Trigger(false,PCM_IN);/*stop*/
                        i2sPrivate->rxRunning = false;
                        i2sPrivate->rxHalfCallCount = 0;
                        i2sPrivate->rxEndCallCount = 0;
                        i2sPrivate->readPointer = NULL;
                        i2sPrivate->rxDmaPointer = NULL;
                        return -1;
                }
        }
        return 0;
}

/**
  * @internal
  * @brief DMA I2S transmit/receive process half complete callback
  * @param arg: pointer to a HAL_Semaphore structure that contains
  *             sem to synchronous data.
  * @retval None
  */
__nonxip_text
static void I2S_DMAHalfCallback(void *arg)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;
        if (arg == &(i2sPrivate->txReady)) {
                i2sPrivate->txHalfCallCount ++;
				if (i2sPrivate->isTxSemaphore) {
                        i2sPrivate->isTxSemaphore = false;
                        HAL_SemaphoreRelease((HAL_Semaphore *)arg);
                }
                if (I2S_DMA_BUFFER_CHECK_Threshold(0) != 0)
                        return;
                i2sPrivate->txDmaPointer = i2sPrivate->txBuf + i2sPrivate->txBufSize/2;
        } else {
                i2sPrivate->rxHalfCallCount ++;
				if (i2sPrivate->isRxSemaphore) {
				        i2sPrivate->isRxSemaphore = false;
				        HAL_SemaphoreRelease((HAL_Semaphore *)arg);
				}

                if (I2S_DMA_BUFFER_CHECK_Threshold(1) != 0)
                        return;
				i2sPrivate->rxDmaPointer = i2sPrivate->rxBuf + i2sPrivate->rxBufSize/2;
        }
}

/**
  * @internal
  * @brief DMA I2S transmit/receive process complete callback
  * @param arg: pointer to a HAL_Semaphore structure that contains
  *             sem to synchronous data.
  * @retval None
  */
__nonxip_text
static void I2S_DMAEndCallback(void *arg)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;
        if (arg == &(i2sPrivate->txReady)) {
                i2sPrivate->txEndCallCount ++;
				if (i2sPrivate->isTxSemaphore) {
                        i2sPrivate->isTxSemaphore = false;
                        HAL_SemaphoreRelease((HAL_Semaphore *)arg);
                }
                if (I2S_DMA_BUFFER_CHECK_Threshold(0) != 0)
                        return;
                i2sPrivate->txDmaPointer = i2sPrivate->txBuf;
        } else {
                i2sPrivate->rxEndCallCount ++;
				if (i2sPrivate->isRxSemaphore) {
					i2sPrivate->isRxSemaphore = false;
					HAL_SemaphoreRelease((HAL_Semaphore *)arg);
				}
                if (I2S_DMA_BUFFER_CHECK_Threshold(1) != 0)
                        return;
                i2sPrivate->rxDmaPointer = i2sPrivate->rxBuf;
        }
}

/**
  * @internal
  * @brief Start the DMA Transfer.
  * @param chan: the specified DMA Channel.
  * @param srcAddr: The source memory Buffer address
  * @param dstAddr: The destination memory Buffer address
  * @param datalen: The length of data to be transferred from source to destination
  * @retval none
  */
__nonxip_text
static void I2S_DMAStart(DMA_Channel chan, uint32_t srcAddr, uint32_t dstAddr, uint32_t datalen)
{
        HAL_DMA_Start(chan, srcAddr, dstAddr, datalen);
}

/**
  * @internal
  * @brief stop the DMA Transfer.
  * @param chan: the specified DMA Channel.
  * @retval none
  */
__nonxip_text
static void I2S_DMAStop(DMA_Channel chan)
{
        HAL_DMA_Stop(chan);
}

/**
  * @internal
  * @brief Sets the DMA Transfer parameter.
  * @param channel: the specified DMA Channel.
  * @param dir: Data transfer direction
  * @retval none
  */
static void I2S_DMASet(DMA_Channel channel,Audio_Stream_Dir dir)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;
        DMA_ChannelInitParam dmaParam;
        HAL_Memset(&dmaParam, 0, sizeof(dmaParam));
        if (dir == PCM_OUT) {

                dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_CIRCULAR,
                                DMA_WAIT_CYCLE_2,
                                DMA_BYTE_CNT_MODE_REMAIN,
                                LOOP_EN ? DMA_DATA_WIDTH_32BIT : DMA_DATA_WIDTH_16BIT,
                                DMA_BURST_LEN_1,
                                DMA_ADDR_MODE_FIXED,
                                DMA_PERIPH_DAUDIO,
                                LOOP_EN ? DMA_DATA_WIDTH_32BIT : DMA_DATA_WIDTH_16BIT,
                                DMA_BURST_LEN_1,
                                DMA_ADDR_MODE_INC,
                                DMA_PERIPH_SRAM);

                dmaParam.endArg = &(i2sPrivate->txReady);
                dmaParam.halfArg = &(i2sPrivate->txReady);
        } else {

                dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_CIRCULAR,
                                DMA_WAIT_CYCLE_2,
                                DMA_BYTE_CNT_MODE_REMAIN,
                                LOOP_EN ? DMA_DATA_WIDTH_32BIT : DMA_DATA_WIDTH_16BIT,
                                DMA_BURST_LEN_1,
                                DMA_ADDR_MODE_INC,
                                DMA_PERIPH_SRAM,
                                LOOP_EN ? DMA_DATA_WIDTH_32BIT : DMA_DATA_WIDTH_16BIT,
                                DMA_BURST_LEN_1,
                                DMA_ADDR_MODE_FIXED,
                                DMA_PERIPH_DAUDIO);

                dmaParam.endArg = &(i2sPrivate->rxReady);
                dmaParam.halfArg = &(i2sPrivate->rxReady);
        }
        dmaParam.irqType = DMA_IRQ_TYPE_BOTH;
        dmaParam.endCallback = I2S_DMAEndCallback;
        dmaParam.halfCallback = I2S_DMAHalfCallback;
        HAL_DMA_Init(channel, &dmaParam);
}

/**
  * @internal
  * @brief Enable or disable the specified i2s tx module.
  * @param enable: specifies enable or disable.
  * @retval None
  */
__nonxip_text
static void tx_enable(bool enable)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;
        /*clear tx tifo*/
        HAL_SET_BIT(I2S->DA_FCTL, I2S_TXFIFO_RESET_BIT);

        if (enable) {
                if (i2sPrivate->txDMAChan != DMA_CHANNEL_INVALID)
                        HAL_SET_BIT(I2S->DA_INT, I2S_TXFIFO_DMA_ITEN_BIT);
        } else {
                if (i2sPrivate->txDMAChan != DMA_CHANNEL_INVALID)
                        HAL_CLR_BIT(I2S->DA_INT, I2S_TXFIFO_DMA_ITEN_BIT);
        }
}

/**
  * @internal
  * @brief Enable or disable the specified i2s rx module.
  * @param enable: specifies enable or disable.
  * @retval None
  */
__nonxip_text
static void rx_enable(bool enable)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;
        /*clear rx tifo*/
		//if (!LOOP_EN)
        	HAL_SET_BIT(I2S->DA_FCTL, I2S_RXFIFO_RESET_BIT);

        if (enable) {
                if (i2sPrivate->rxDMAChan != DMA_CHANNEL_INVALID)
                        HAL_SET_BIT(I2S->DA_INT, I2S_RXFIFO_DMA_ITEN_BIT);
        } else {

                if (i2sPrivate->rxDMAChan != DMA_CHANNEL_INVALID)
                        HAL_CLR_BIT(I2S->DA_INT, I2S_RXFIFO_DMA_ITEN_BIT);
        }
}

/**
  * @internal
  * @brief Enable or disable the specified i2s module with DMA module.
  * @param enable: specifies enable or disable.
  * @param dir: the direction of stream.
  * @retval None
  */
__nonxip_text
void HAL_I2S_Trigger(bool enable,Audio_Stream_Dir dir)
{
        unsigned long flags;
        struct I2S_Private *i2sPrivate = gI2sPrivate;
        int doProtection = !HAL_IsISRContext();

        if (doProtection) {
                flags = HAL_EnterCriticalSection();
        }

        if (enable) {
                if (dir == PCM_OUT) {
                        /*trigger tx*/
                        tx_enable(enable);

                        /* start dma*/
                        if (i2sPrivate->txDMAChan != DMA_CHANNEL_INVALID) {
                                I2S_DMAStart(i2sPrivate->txDMAChan, (uint32_t)i2sPrivate->txBuf,
                                                (uint32_t)&(I2S->DA_TXFIFO), i2sPrivate->txBufSize);
                        }
                        i2sPrivate->txRunning = true;

						//if (LOOP_EN) {
						//	HAL_SET_BIT(I2S->DA_FCTL, I2S_RXFIFO_RESET_BIT);
						//}
                } else {
                        rx_enable(enable);
                        if (i2sPrivate->rxDMAChan != DMA_CHANNEL_INVALID)
                                I2S_DMAStart(i2sPrivate->rxDMAChan, (uint32_t)&(I2S->DA_RXFIFO),
                                                (uint32_t)i2sPrivate->rxBuf, i2sPrivate->rxBufSize);
                        i2sPrivate->rxRunning = true;
                }
        } else {
                if (dir == PCM_OUT) {
                        tx_enable(enable);
                        if (i2sPrivate->txDMAChan != DMA_CHANNEL_INVALID)
                                I2S_DMAStop(i2sPrivate->txDMAChan);
                        i2sPrivate->txRunning = false;
                } else {
                        rx_enable(enable);
                        if (i2sPrivate->rxDMAChan != DMA_CHANNEL_INVALID)
                                I2S_DMAStop(i2sPrivate->rxDMAChan);
                        i2sPrivate->rxRunning = false;
                }
        }
        #if 0
        int i =0 ;
        printf("\n");
        for (i = 0; i < 0x68; i = i+4) {
                printf("0x%x: 0x%08x",i,(*(uint32_t *)(0x40042c00+i)));
                printf("\n");
        }
        #endif
        if (doProtection) {
                HAL_ExitCriticalSection(flags);
        }
}

/**
  * @brief Transmit an amount of data with DMA module
  * @param buf: pointer to the Transmit data buffer.
  * @param Size: number of data sample to be sent:
  * @note if the data size less than half of dmabuf, return HAL_INVALID. when the size
  *       several times greater than half of dmabuf, only transfer its(half length of
  *       dmabuf)integer multiple
  * @retval length of data transmited
  */
int32_t HAL_I2S_Write_DMA(uint8_t *buf, uint32_t size)
{
    if (!buf || size <= 0)
            return HAL_INVALID;

    struct I2S_Private *i2sPrivate = gI2sPrivate;

    uint8_t *pdata = buf;
    uint8_t *lastWritePointer = NULL;
    uint32_t toWrite = 0, writeSize = i2sPrivate->txBufSize / 2;
    uint8_t err_flag; /* temp solution to avoid outputing debug message when irq disabled */
#ifdef CONFIG_PM
	if (i2sPrivate->suspend) {
		I2S_ERROR("I2S has suspend, should stop write I2S!\n");
		return 0;
	}
#endif
	if (writeSize == 0) {
		I2S_ERROR("TxBuf not exist\n");
		return -1;
	}

	if (size < writeSize) {
		//I2S_ERROR("Write size too small\n");
		return -1;
	}

	for ( ; size >= writeSize; pdata += writeSize, toWrite += writeSize, size -= writeSize)
	{
		if (i2sPrivate->txRunning == false) {
			if (!i2sPrivate->writePointer){
				i2sPrivate->writePointer = i2sPrivate->txBuf;
				if(size >= writeSize*2){
					lastWritePointer = i2sPrivate->txBuf;
					I2S_MEMCPY(lastWritePointer, pdata, writeSize);
					pdata += writeSize;
					toWrite += writeSize;
					size -= writeSize;
					i2sPrivate->writePointer = i2sPrivate->txBuf + writeSize;
				}
			}
			lastWritePointer = i2sPrivate->writePointer;

			I2S_MEMCPY(lastWritePointer, pdata, writeSize);
			I2S_DEBUG("Tx: play start...\n");
			HAL_I2S_Trigger(true,PCM_OUT);/*play*/
			i2sPrivate->txRunning =true;
		} else {
			err_flag = 0;
			HAL_DisableIRQ();
			if (i2sPrivate->txHalfCallCount && i2sPrivate->txEndCallCount) {
				err_flag = 1;
				i2sPrivate->txHalfCallCount = 0;
				i2sPrivate->txEndCallCount = 0;
			} else if (i2sPrivate->txHalfCallCount) {
				i2sPrivate->txHalfCallCount --;
			} else if (i2sPrivate->txEndCallCount) {
				i2sPrivate->txEndCallCount --;
			} else {
				i2sPrivate->isTxSemaphore = true;
				HAL_EnableIRQ();
				HAL_SemaphoreWait(&(i2sPrivate->txReady), HAL_WAIT_FOREVER);
				HAL_DisableIRQ();

				if (i2sPrivate->txHalfCallCount && i2sPrivate->txEndCallCount) {
					err_flag = 1;
					i2sPrivate->txHalfCallCount = 0;
					i2sPrivate->txEndCallCount = 0;
				} else {
					if (i2sPrivate->txHalfCallCount)
						i2sPrivate->txHalfCallCount --;
					if (i2sPrivate->txEndCallCount)
						i2sPrivate->txEndCallCount --;
				}
			}

			if (i2sPrivate->txDmaPointer == i2sPrivate->txBuf) {
				lastWritePointer = i2sPrivate->txBuf + writeSize;
				i2sPrivate->writePointer = i2sPrivate->txBuf;
			} else {
				lastWritePointer = i2sPrivate->txBuf;
				i2sPrivate->writePointer =  i2sPrivate->txBuf + writeSize;
			}
			I2S_MEMCPY(lastWritePointer, pdata, writeSize);
			HAL_EnableIRQ();

			if (err_flag) {
				I2S_ERROR("TxCount:(H:%u,F:%u)\n",i2sPrivate->txHalfCallCount,
                                                  i2sPrivate->txEndCallCount);
				I2S_ERROR("Tx : underrun....\n");
			}
		}
	}

    return toWrite;
}

/**
  * @brief receive an amount of data with DMA module
  * @param buf: pointer to the receive data buffer.
  * @param Size: number of data sample to be receive:
  * @note if the data size less than half of dmabuf, return HAL_INVALID. when the size
  *       several times greater than half of dmabuf, only transfer its(half length of
  *       dmabuf)integer multiple
  * @retval length of data received
  */
int32_t HAL_I2S_Read_DMA(uint8_t *buf, uint32_t size)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;
        if (!buf || size <= 0)
                return HAL_INVALID;
        uint8_t *pdata = buf;
        uint8_t *lastReadPointer = NULL;
        uint32_t readSize = i2sPrivate->rxBufSize / 2;
        uint32_t toRead = 0;
        uint8_t err_flag; /* temp solution to avoid outputing debug message when irq disabled */

#ifdef CONFIG_PM
	if (i2sPrivate->suspend) {
		I2S_ERROR("I2S has suspend, should stop read I2S!\n");
		return 0;
	}
#endif
		if (readSize == 0) {
			I2S_ERROR("RxBuf not exist\n");
			return -1;
		}

		if (size < readSize) {
			I2S_ERROR("Read size too small\n");
			return -1;
        }

		while (size >= readSize) {
			if (i2sPrivate->rxRunning == false) {
			    I2S_DEBUG("Rx: record start...\n");
			    HAL_I2S_Trigger(true,PCM_IN);
			} else {
				err_flag = 0;
				/*disable irq*/
				HAL_DisableIRQ();
				lastReadPointer = i2sPrivate->readPointer;
				if (i2sPrivate->rxHalfCallCount && i2sPrivate->rxEndCallCount) {
					err_flag = 1;
					i2sPrivate->rxHalfCallCount = 0;
					i2sPrivate->rxEndCallCount = 0;
				} else if (i2sPrivate->rxHalfCallCount) {
					i2sPrivate->rxHalfCallCount --;
				} else if (i2sPrivate->rxEndCallCount) {
					i2sPrivate->rxEndCallCount --;
				} else {
					/**enable irq**/
					i2sPrivate->isRxSemaphore = true;
					HAL_EnableIRQ();
					HAL_SemaphoreWait(&(i2sPrivate->rxReady), HAL_WAIT_FOREVER);
					/**disable irq**/
					HAL_DisableIRQ();
					if (i2sPrivate->rxHalfCallCount && i2sPrivate->rxEndCallCount) {
						err_flag = 1;
						i2sPrivate->rxHalfCallCount = 0;
						i2sPrivate->rxEndCallCount = 0;
					} else {
						if (i2sPrivate->rxHalfCallCount)
								i2sPrivate->rxHalfCallCount --;
						if (i2sPrivate->rxEndCallCount)
								i2sPrivate->rxEndCallCount --;
					}
				}

				if (i2sPrivate->rxDmaPointer == i2sPrivate->rxBuf) {
					lastReadPointer = i2sPrivate->rxBuf + readSize;
				} else {
					lastReadPointer = i2sPrivate->rxBuf;
				}
				I2S_MEMCPY(pdata, lastReadPointer, readSize);
				pdata += readSize;
				//i2sPrivate->readPointer = lastReadPointer;
				/**enable irq**/
				HAL_EnableIRQ();
				size -= readSize;
				toRead += readSize;

				if (err_flag) {
					I2S_ERROR("Rx overrun, (H:%u,F:%u)\n",
					          i2sPrivate->rxHalfCallCount,
					          i2sPrivate->rxEndCallCount);
				}
			}
		}
        return toRead;
}

/**
  * @brief Open the I2S module according to the specified parameters
  *         in the I2S_DataParam.
  * @param param: pointer to a I2S_DataParam structure that contains
  *         data format information
  * @retval HAL status
  */
HAL_Status HAL_I2S_Open(void *param)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;

        I2S_DataParam *dataParam = (I2S_DataParam *)param;

        if (dataParam->direction == PCM_OUT) {
                if (i2sPrivate->isTxInitiate == true) {
                        I2S_ERROR("Tx device opened already.open faied...\n");
                        return HAL_ERROR;
                }
                i2sPrivate->isTxInitiate = true;

                I2S_MEMCPY(&(i2sPrivate->pdataParam), dataParam, sizeof(*dataParam));
        } else {
                if (i2sPrivate->isRxInitiate == true) {
                        I2S_ERROR("Rx device opened already.open faied...\n");
                        return HAL_ERROR;
                }
                i2sPrivate->isRxInitiate = true;

                I2S_MEMCPY(&(i2sPrivate->cdataParam), dataParam, sizeof(*dataParam));
        }

        if (dataParam->direction == PCM_OUT) {
                i2sPrivate->txDMAChan = DMA_CHANNEL_INVALID;
                i2sPrivate->txBufSize = dataParam->bufSize;
                i2sPrivate->txHalfCallCount = 0;
                i2sPrivate->txEndCallCount = 0;
#ifdef RESERVERD_MEMORY_FOR_I2S_TX
                i2sPrivate->txBuf = I2STX_BUF;
#else
                i2sPrivate->txBuf = I2S_MALLOC(i2sPrivate->txBufSize);
                if(i2sPrivate->txBuf)
                        I2S_MEMSET(i2sPrivate->txBuf, 0, i2sPrivate->txBufSize);
                else {
                        I2S_ERROR("Malloc tx buf(for DMA),faild...\n");
                        return HAL_ERROR;
                }
#endif
                /*request DMA channel*/
                i2sPrivate->txDMAChan = HAL_DMA_Request();
                if (i2sPrivate->txDMAChan == DMA_CHANNEL_INVALID) {
                        I2S_ERROR("Obtain I2S tx DMA channel,faild...\n");
#ifndef RESERVERD_MEMORY_FOR_I2S_TX
                        I2S_FREE(i2sPrivate->txBuf);
#endif
                        return HAL_ERROR;
                } else
                I2S_DMASet(i2sPrivate->txDMAChan, PCM_OUT);
                HAL_SemaphoreInitBinary(&i2sPrivate->txReady);
        } else {
                i2sPrivate->rxDMAChan = DMA_CHANNEL_INVALID;
                i2sPrivate->rxBufSize = dataParam->bufSize;
                i2sPrivate->rxHalfCallCount = 0;
                i2sPrivate->rxEndCallCount = 0;
#ifdef RESERVERD_MEMORY_FOR_I2S_RX
                i2sPrivate->rxBuf = I2SRX_BUF;
#else
                i2sPrivate->rxBuf = I2S_MALLOC(i2sPrivate->rxBufSize);
                if(i2sPrivate->rxBuf)
                        I2S_MEMSET(i2sPrivate->rxBuf, 0, i2sPrivate->rxBufSize);
                else {
                        I2S_ERROR("Malloc rx buf(for DMA),faild...\n");
                        return HAL_ERROR;
                }
#endif
                i2sPrivate->rxDMAChan = HAL_DMA_Request();
                if (i2sPrivate->rxDMAChan == DMA_CHANNEL_INVALID) {
                        I2S_ERROR("Obtain I2S rx DMA channel,faild...\n");
#ifndef RESERVERD_MEMORY_FOR_I2S_TX
                        I2S_FREE(i2sPrivate->rxBuf);
#endif
                        return HAL_ERROR;
                } else
                        I2S_DMASet(i2sPrivate->rxDMAChan, PCM_IN);
                HAL_SemaphoreInitBinary(&i2sPrivate->rxReady);
        }

        HAL_MutexLock(&i2sPrivate->devSetLock, OS_WAIT_FOREVER);
        /*set bclk*/
        I2S_SET_ClkDiv(dataParam, i2sPrivate->hwParam);

        /*set sample resolution*/
        I2S_SET_SampleResolution(dataParam);

        /*set channel*/
        I2S_SET_Channels(dataParam);

        if (dataParam->direction == PCM_OUT) {
				I2S_EnableTx();
        } else {
				I2S_EnableRx();
        }
        HAL_MutexUnlock(&i2sPrivate->devSetLock);

        return HAL_OK;
}

/**
  * @brief Close the I2S module
  * @note The module is closed at the end of transaction to avoid power consumption
  * @retval none
  */
HAL_Status HAL_I2S_Close(Audio_Stream_Dir dir)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;

        if (dir == PCM_OUT) {
                HAL_I2S_Trigger(false,PCM_OUT);
                I2S_DisableTx();
                i2sPrivate->txRunning = false;
                i2sPrivate->isTxInitiate = false;
                if (i2sPrivate->txDMAChan != DMA_CHANNEL_INVALID) {
                        HAL_DMA_DeInit(i2sPrivate->txDMAChan);
                        HAL_DMA_Release(i2sPrivate->txDMAChan);
                        i2sPrivate->txDMAChan = DMA_CHANNEL_INVALID;

                }
                I2S_MEMSET(&(i2sPrivate->pdataParam), 0, sizeof(I2S_DataParam));
#ifndef RESERVERD_MEMORY_FOR_I2S_TX
                I2S_FREE(i2sPrivate->txBuf);
#endif
                HAL_SemaphoreDeinit(&i2sPrivate->txReady);
                i2sPrivate->txBuf = NULL;
                i2sPrivate->txBufSize = 0;
                i2sPrivate->writePointer = NULL;
                i2sPrivate->txHalfCallCount = 0;
                i2sPrivate->txEndCallCount = 0;
        } else {
                HAL_I2S_Trigger(false,PCM_IN);
                I2S_DisableRx();
                i2sPrivate->isRxInitiate = false;
                i2sPrivate->rxRunning = false;
                if (i2sPrivate->rxDMAChan != DMA_CHANNEL_INVALID) {
                        HAL_DMA_DeInit(i2sPrivate->rxDMAChan);
                        HAL_DMA_Release(i2sPrivate->rxDMAChan);
                        i2sPrivate->rxDMAChan = DMA_CHANNEL_INVALID;

                }
                I2S_MEMSET(&(i2sPrivate->cdataParam), 0, sizeof(I2S_DataParam));
#ifndef RESERVERD_MEMORY_FOR_I2S_TX
                I2S_FREE(i2sPrivate->rxBuf);
#endif
                HAL_SemaphoreDeinit(&i2sPrivate->rxReady);
                i2sPrivate->rxBuf = NULL;
                i2sPrivate->rxBufSize = 0;
                i2sPrivate->readPointer = NULL;

                i2sPrivate->rxHalfCallCount = 0;
                i2sPrivate->rxEndCallCount = 0;
        }
        return HAL_OK;
}

/**
  * @internal
  * @brief I2S PINS Init
  * @retval HAL status
  */
static inline HAL_Status I2S_PINS_Init()
{
        return HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_I2S, 0), 0);
}

/**
  * @internal
  * @brief I2S PINS DeInit
  * @retval HAL status
  */
static inline HAL_Status I2S_PINS_Deinit()
{
        return HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_I2S, 0), 0);
}

/**
  * @internal
  * @brief I2S hardware Init
  * @param param: pointer to a I2S_HWParam structure that contains
  *         the configuration for clk/mode/format.
  * @retval HAL status
  */
static inline HAL_Status I2S_HwInit(I2S_HWParam *param)
{
        if (!param)
                return HAL_INVALID;

        /*config device clk source*/
        if (param->codecClk.isDevclk != 0) {
                I2S_SET_Mclk(true, param->codecClk.clkSource, AUDIO_DEVICE_PLL / param->codecClkDiv);
        }

        /* set lrck period /frame mode */
        HAL_MODIFY_REG(I2S->DA_FMT0, I2S_LRCK_PERIOD_MASK|I2S_LRCK_WIDTH_MASK|
                                     I2S_SW_SEC_MASK,
                                      I2S_LRCK_PERIOD(param->lrckPeriod)|
                                      param->frameMode|I2S_SLOT_WIDTH_BIT32);

        /* set first transfer bit */
        HAL_MODIFY_REG(I2S->DA_FMT1, I2S_TX_MLS_MASK|I2S_RX_MLS_MASK|
                        I2S_PCM_TXMODE_MASK|I2S_PCM_RXMODE_MASK|I2S_SEXT_MASK,
                        param->txMsbFirst|param->rxMsbFirst|
                        I2S_TX_LINEAR_PCM|I2S_RX_LINEAR_PCM|I2S_ZERO_SLOT);
        /* global enable */
        HAL_SET_BIT(I2S->DA_CTL, I2S_GLOBE_EN_BIT);

        HAL_MODIFY_REG(I2S->DA_FCTL, I2S_RXFIFO_LEVEL_MASK|I2S_TXFIFO_LEVEL_MASK|
                                     I2S_RX_FIFO_MODE_SHIFT|I2S_TX_FIFO_MODE_MASK,
                                     I2S_RXFIFO_TRIGGER_LEVEL(param->rxFifoLevel)|
                                     I2S_TXFIFO_TRIGGER_LEVEL(param->txFifoLevel)|
                                     (LOOP_EN ? I2S_RX_FIFO_MODE0|I2S_TX_FIFO_MODE0 : I2S_RX_FIFO_MODE3|I2S_TX_FIFO_MODE1));
        return I2S_SET_Format(param);
}

/**
  * @internal
  * @brief I2S hardware DeInit
  * @param param: pointer to a I2S_HWParam structure
  * @retval HAL status
  */
static inline HAL_Status I2S_HwDeInit(I2S_HWParam *param)
{
        if (!param)
                return HAL_INVALID;
        /* global disable */
        HAL_CLR_BIT(I2S->DA_CTL, I2S_GLOBE_EN_BIT);
        I2S_SET_Mclk(false, 0, 0);
        return HAL_OK;
}

#ifdef CONFIG_PM
__nonxip_text
static int i2s_suspend(struct soc_device *dev, enum suspend_state_t state)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;

        i2sPrivate->suspend = 1;
        switch (state) {
                case PM_MODE_SLEEP:
                case PM_MODE_STANDBY:
                case PM_MODE_HIBERNATION:
                        HAL_MutexLock(&i2sPrivate->devSetLock, OS_WAIT_FOREVER);
                        I2S_HwDeInit(i2sPrivate->hwParam);
                        HAL_MutexUnlock(&i2sPrivate->devSetLock);

                        I2S_PINS_Deinit();
                        HAL_CCM_DAUDIO_DisableMClock();
                        HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_DAUDIO);
                        HAL_PRCM_DisableAudioPLL();
                        HAL_PRCM_DisableAudioPLLPattern();
                        break;
                default:
                        break;
        }
        return 0;
}

__nonxip_text
static int i2s_resume(struct soc_device *dev, enum suspend_state_t state)
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;

        switch (state) {
                case PM_MODE_SLEEP:
                case PM_MODE_STANDBY:
                case PM_MODE_HIBERNATION:
                        I2S_PINS_Init();
                        /*init and enable clk*/
                        I2S_PLLAUDIO_Update(I2S_PLL_22M);
                        HAL_PRCM_SetAudioPLLParam(i2sPrivate->audioPllParam);
                        HAL_PRCM_SetAudioPLLPatternParam(i2sPrivate->audioPllPatParam);
                        HAL_PRCM_EnableAudioPLL();
                        HAL_PRCM_EnableAudioPLLPattern();

                        HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_DAUDIO);
                        HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_DAUDIO);
                        HAL_CCM_DAUDIO_SetMClock(AUDIO_PLL_SRC);
                        HAL_CCM_DAUDIO_EnableMClock();

                        HAL_MutexLock(&i2sPrivate->devSetLock, OS_WAIT_FOREVER);
                        I2S_HwInit(i2sPrivate->hwParam);
                        HAL_MutexUnlock(&i2sPrivate->devSetLock);
                        break;
                default:
                        break;
        }
        i2sPrivate->suspend = 0;
        return 0;
}

static const struct soc_device_driver i2s_drv = {
        .name = "I2S",
        .suspend = i2s_suspend,
        .resume = i2s_resume,
};
#endif

/**
  * @brief Initializes the I2S module according to the specified parameters
  *         in the I2S_Param.
  * @param param: pointer to a I2S_Param structure that contains
  *         the configuration information for I2S
  * @retval HAL status
  */
HAL_Status HAL_I2S_Init(void *param)
{
        int32_t ret = 0;
		I2S_Param *i2s_param = (I2S_Param *)param;
        if (!i2s_param)
                return HAL_INVALID;
        struct I2S_Private *i2sPrivate = gI2sPrivate;

        if (i2sPrivate->isHwInit == true)
                return HAL_OK;
        I2S_MEMSET(i2sPrivate, 0, sizeof(*i2sPrivate));
        i2sPrivate->isHwInit = true;

        if (i2s_param->hwParam == NULL) {
                i2sPrivate->hwParam = &gHwParam;
        } else {
                i2sPrivate->hwParam = i2s_param->hwParam;
	}
	i2sPrivate->hwParam->codecClkDiv = i2s_param->mclkDiv;

        HAL_MutexInit(&i2sPrivate->devSetLock);

        I2S_PINS_Init();

        /*init and enable clk*/
        I2S_PLLAUDIO_Update(I2S_PLL_22M);
        HAL_PRCM_SetAudioPLLParam(i2sPrivate->audioPllParam);
        HAL_PRCM_EnableAudioPLL();
        HAL_PRCM_SetAudioPLLPatternParam(i2sPrivate->audioPllPatParam);
        HAL_PRCM_EnableAudioPLLPattern();
        HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_DAUDIO);
        HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_DAUDIO);
        HAL_CCM_DAUDIO_SetMClock(AUDIO_PLL_SRC);
        HAL_CCM_DAUDIO_EnableMClock();

        HAL_MutexLock(&i2sPrivate->devSetLock, OS_WAIT_FOREVER);
        ret = I2S_HwInit(i2sPrivate->hwParam);

#ifdef CONFIG_PM
	i2sPrivate->suspend = 0;
	i2sPrivate->dev.name = "I2S";
	i2sPrivate->dev.driver = &i2s_drv;
	pm_register_ops(&i2sPrivate->dev);
#endif
        HAL_MutexUnlock(&i2sPrivate->devSetLock);
        return ret;
}

#if 0
void HAL_I2S_REG_DEBUG()
{
        int i = 0;
        for (i = 0; i < 0x58; i = i+4)
                printf("REG:0X%x,VAL:0X%x\n",i,(*((__IO uint32_t *)(0x40042c00+i))));
}
#endif

/**
  * @brief DeInitializes the I2S module
  *
  * @retval none
  */
void HAL_I2S_DeInit()
{
        struct I2S_Private *i2sPrivate = gI2sPrivate;

#ifdef CONFIG_PM
	pm_unregister_ops(&i2sPrivate->dev);
#endif
        HAL_MutexLock(&i2sPrivate->devSetLock, OS_WAIT_FOREVER);
        i2sPrivate->isHwInit = false;
        I2S_HwDeInit(i2sPrivate->hwParam);
        HAL_MutexUnlock(&i2sPrivate->devSetLock);

        HAL_MutexDeinit(&i2sPrivate->devSetLock);
        I2S_PINS_Deinit();

        HAL_CCM_DAUDIO_DisableMClock();
        HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_DAUDIO);
        HAL_PRCM_DisableAudioPLL();
        HAL_PRCM_DisableAudioPLLPattern();

        I2S_MEMSET(i2sPrivate, 0, sizeof(struct I2S_Private));
}

void I2S_Loop_En(uint8_t en)
{
#if LOOP_EN
   	if (en)
		HAL_SET_BIT(I2S->DA_CTL, I2S_LOOP_TSET_EN_EN_BIT);
	else
		HAL_CLR_BIT(I2S->DA_CTL, I2S_LOOP_TSET_EN_EN_BIT);
#endif
}
void I2S_RXFIFO_Flush(void)
{
#if LOOP_EN
	HAL_SET_BIT(I2S->DA_FCTL, I2S_RXFIFO_RESET_BIT);
#endif
}


/* platform ops */
static const struct platform_ops xr872_i2s_ops = {
	.open = HAL_I2S_Open,
	.close = HAL_I2S_Close,
	.pcm_read = HAL_I2S_Read_DMA,
	.pcm_write = HAL_I2S_Write_DMA,
};

/* platform driver */
static struct platform_driver xr872_i2s_drv = {
	.name = XRADIO_PLATFORM_I2S_NAME,
	.platform_attr = XRADIO_PLATFORM_I2S,

	.init = HAL_I2S_Init,
	.deinit = HAL_I2S_DeInit,

	.platform_ops = &xr872_i2s_ops,
};


HAL_Status xradio_i2s_register(void)
{
	I2S_DEBUG("--->%s\n",__FUNCTION__);

	/* Malloc gI2sPrivate buffer */
	gI2sPrivate = (struct I2S_Private *)I2S_MALLOC(sizeof(struct I2S_Private));
	if(gI2sPrivate == NULL){
		I2S_ERROR("Malloc struct I2S_Private buffer Fail!\n");
		return HAL_ERROR;
	}
	I2S_MEMSET(gI2sPrivate, 0, sizeof(struct I2S_Private));

	/* Platform list add */
	list_add(&xr872_i2s_drv.node, &hal_snd_platform_list);

	return HAL_OK;
}

HAL_Status xradio_i2s_unregister(void)
{
	struct platform_driver *i2s_drv_ptr;
	I2S_DEBUG("--->%s\n",__FUNCTION__);

	/* Check snd platform list empty or not */
	if(list_empty(&hal_snd_platform_list)){
		I2S_DEBUG("Hal snd platform list is empty, don't need to unregister\n");
		return HAL_OK;
	}

	/* Get platform to unregister */
	list_for_each_entry(i2s_drv_ptr, &hal_snd_platform_list, node){
		if(i2s_drv_ptr == &xr872_i2s_drv){
			list_del(&xr872_i2s_drv.node);
			break;
		}
	}

	/* Free gI2sPrivate buffer */
	if(gI2sPrivate){
		I2S_FREE(gI2sPrivate);
		gI2sPrivate = NULL;
	}

	return HAL_OK;
}

#endif /* __CONFIG_BOOTLOADER */


