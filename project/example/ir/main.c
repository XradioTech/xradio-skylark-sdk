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

#include <stdio.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_clock.h"
#include "driver/chip/ir_nec.h"
#include "driver/chip/hal_irrx.h"
#include "driver/chip/hal_irtx.h"

#define IR_IMS_TEST_EN	(1)

static void irrx_rxcplt_callback(uint32_t addr, uint32_t key)
{
	printf("treceived ir code addr:0x%02x key:0x%02x\n", addr, key);
}

static IRRX_HandleTypeDef *irrx_init()
{
	IRRX_InitTypeDef rx_param;
	IRRX_HandleTypeDef *rx_handle_param;
	rx_param.PulsePolariyInvert = IRRX_RPPI_INVERT;
	rx_param.rxCpltCallback = irrx_rxcplt_callback;
	rx_handle_param = HAL_IRRX_Init(&rx_param);
	return rx_handle_param;
}

static IRTX_HandleTypeDef *irtx_init()
{
	IRTX_InitTypeDef tx_param;
	IRTX_HandleTypeDef *tx_handle_param;
	tx_param.CyclicalCnt = 3;

	uint32_t clk = HAL_GetHFClock();

	if (clk == HOSC_CLOCK_26M) {
		tx_param.IdleDurationCnt = IRTX_26M_NEC_IDC_VALUE;
	} else if (clk == HOSC_CLOCK_24M) {
		tx_param.IdleDurationCnt = IRTX_24M_NEC_IDC_VALUE;
	} else {
		printf("%s unknow clk type(%d)!\n", __func__, clk);
	}

	tx_param.ModulateDutyLevel = IRTX_DRMC_TIME_1;
	tx_param.PulsePolarity = IRTX_TPPI_NONE;
#if IR_IMS_TEST_EN
	tx_param.SendModeType = IRTX_TTS_NONE;
	tx_param.InternalModulation = IRTX_IMS_ENABLE;
#else
	tx_param.SendModeType = IRTX_TTS_CYCLICAL;
	tx_param.InternalModulation = IRTX_IMS_DISABLE;
#endif
	tx_handle_param = HAL_IRTX_Init(&tx_param);
	return tx_handle_param;
}

static void irtx_nec_transmit(IRTX_HandleTypeDef *irtx, uint8_t addr, uint8_t key)
{
	HAL_IRTX_Transmit(irtx, IRTX_NEC_PROTO
						, IR_NEC_CODE(addr, key));
}

static void irrx_deinit( IRRX_HandleTypeDef *irrx)
{
	if (irrx)
		HAL_IRRX_DeInit(irrx);
}

static void irtx_deinit( IRTX_HandleTypeDef *irtx)
{
	if (irtx)
		HAL_IRTX_DeInit(irtx);
}

/*Run this demo, please connect the XR872AT_VER_V1_0 board.*/
int main(void)
{
	printf("ir demo started\n\n");

	IRRX_HandleTypeDef *rx_param;
	IRTX_HandleTypeDef *tx_param;

	rx_param = irrx_init();
	tx_param = irtx_init();

	irtx_nec_transmit(tx_param, 0x01, 0x02);

	OS_MSleep(10000);

	irrx_deinit(rx_param);
	irtx_deinit(tx_param);

	printf("\nir demo over\n");

	while (1)
		OS_MSleep(1000);

	return 0;
}

