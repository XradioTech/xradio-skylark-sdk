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

#include "board_debug.h"
#include "board.h"
#include "driver/chip/hal_snd_card.h"
#include <string.h>

#include "pm/pm.h"

/* uart */
#if PRJCONF_UART_EN
__sram_text
HAL_Status board_uart_init(UART_ID uart_id)
{
	__sram_rodata static const UART_InitParam board_uart_param = {
		.baudRate		  = BOARD_UART_BAUD_RATE,
		.parity 		  = BOARD_UART_PARITY,
		.stopBits		  = BOARD_UART_STOP_BITS,
		.dataBits		  = BOARD_UART_DATA_BITS,
		.isAutoHwFlowCtrl = BOARD_UART_HW_FLOW_CTRL,
	};

	return HAL_UART_Init(uart_id, &board_uart_param);
}
#endif /* PRJCONF_UART_EN */

/* spi */
#if PRJCONF_SPI_EN
__weak HAL_Status board_spi_init(SPI_Port spi)
{
	static const SPI_Global_Config board_spi_param = {
		.mclk	  = BOARD_SPI_MCLK,
		.cs_level = BOARD_SPI_CS_LEVEL
	};

	return HAL_SPI_Init(spi, &board_spi_param);
}

__weak HAL_Status board_spi_deinit(SPI_Port spi)
{
	return HAL_SPI_Deinit(spi);
}
#endif /* PRJCONF_SPI_EN */

#if PRJCONF_INTERNAL_SOUNDCARD_EN || PRJCONF_AC107_SOUNDCARD_EN
__weak HAL_Status board_soundcard_init(void)
{
	/* Codec register */
#if PRJCONF_INTERNAL_SOUNDCARD_EN
	HAL_SndCard_CodecRegisterInternal();
#endif
#if PRJCONF_AC107_SOUNDCARD_EN
	HAL_SndCard_CodecRegisterAc107();
#endif
	//Add other codec register here

	/* Platform register */
#if PRJCONF_AC107_SOUNDCARD_EN	//or other codec that need to use I2S
	HAL_SndCard_PlatformRegisterI2S();
#endif
	//Add other platform register here

	/* Snd Card register*/
	HAL_SndCard_Register();

	return HAL_OK;
}

__weak HAL_Status board_soundcard_deinit(void)
{
	/* Snd Card unregister*/
	HAL_SndCard_Unregister();

	/* Platform unregister */
#if PRJCONF_AC107_SOUNDCARD_EN	//or other codec that need to use I2S
	HAL_SndCard_PlatformUnregisterI2S();
#endif
	//Add other platform unregister here

	/* Codec unregister */
#if PRJCONF_INTERNAL_SOUNDCARD_EN
	HAL_SndCard_CodecUnregisterInternal();
#endif
#if PRJCONF_AC107_SOUNDCARD_EN
	HAL_SndCard_CodecUnregisterAc107();
#endif
	//Add other codec unregister here

	return HAL_OK;
}
#endif

/* mmc card */
#if PRJCONF_MMC_EN
__weak HAL_Status board_sdcard_init(card_detect_cb cb)
{
	struct mmc_host *host;
	SDC_InitTypeDef sdc_param;

#ifdef CONFIG_DETECT_CARD
	sdc_param.cd_mode = PRJCONF_MMC_DETECT_MODE;
	sdc_param.cd_cb = cb;
#endif
	sdc_param.debug_mask = ROM_WRN_MASK | ROM_ERR_MASK | ROM_ANY_MASK;
	sdc_param.dma_use = 1;
	host = HAL_SDC_Create(0, &sdc_param);
	if (HAL_SDC_Init(host) == NULL) {
		return HAL_ERROR;
	}

	return HAL_OK;
}
#endif

