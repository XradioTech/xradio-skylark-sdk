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

#include "etf_uart.h"

#define SERIAL1			(UART_T *)UART1_BASE

int etf_port_uart_sent(char *buf, int count)
{
	char *ptr;
	int left;

	ptr = buf;
	left = count;
	while (left > 0) {
		while (!HAL_UART_IsTxReady(SERIAL1)) {
			; /* wait FIFO become not full */
		}

		HAL_UART_PutTxData(SERIAL1, *ptr);
		++ptr;
		--left;
	}

	return count;
}

int etf_port_uart_getc(void)
{
	if (HAL_UART_IsRxReady(SERIAL1)) {
		return (int)HAL_UART_GetRxData(SERIAL1);
	} else {
		return -1;
	}
}

int etf_port_uart_init(void)
{
	UART_InitParam uart_param;

	uart_param.baudRate = 115200;
	uart_param.parity = UART_PARITY_NONE;
	uart_param.stopBits = UART_STOP_BITS_1;
	uart_param.dataBits = UART_DATA_BITS_8;
	uart_param.isAutoHwFlowCtrl = 0;
	HAL_UART_Init(UART1_ID, &uart_param);

	return 0;
}
