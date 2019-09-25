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
#include <string.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_uart.h"

#define UART_DMA_MODE
//#define UART_IT_MODE
//#define UART_POLL_MODE

#define UARTID UART1_ID

int uart_init(void)
{
	HAL_Status status = HAL_ERROR;
	UART_InitParam param;

	param.baudRate = 115200;
	param.dataBits = UART_DATA_BITS_8;
	param.stopBits = UART_STOP_BITS_1;
	param.parity = UART_PARITY_NONE;
	param.isAutoHwFlowCtrl = 0;

	status = HAL_UART_Init(UARTID, &param);
	if (status != HAL_OK)
		printf("uart init error %d\n", status);
	return status;
}

#if (defined(UART_DMA_MODE))
/**@bref DMA Mode:use DMA receive & transmit data**/
void uart_dma_mode(void)
{
	int32_t len = 0;
	uint8_t rx_data;

	char buffer[20];
	uint8_t len_str;
	HAL_Status status = HAL_ERROR;

	printf("uart%d dma mode\n", UARTID);
	status = HAL_UART_EnableTxDMA(UARTID);
	if (status != HAL_OK)
		printf("uart TX enable DMA error %d\n", status);

	status = HAL_UART_EnableRxDMA(UARTID);
	if (status != HAL_OK)
		printf("uart RX enable DMA error %d\n", status);

	len_str = snprintf(buffer, 20, "uart%d DMA mode.\n", UARTID);
	HAL_UART_Transmit_DMA(UARTID, (uint8_t *)buffer, len_str);

	while (1) {
		len = HAL_UART_Receive_DMA(UARTID,	/*uartID*/
								&rx_data,	/*buf Pointer to the data buffer*/
								1,			/*size The maximum number of bytes to be received*/
								10000);		/*Timeout value in millisecond to receive data, HAL_WAIT_FOREVER for no timeout*/
		if (len)
			HAL_UART_Transmit_DMA(UARTID,	/*uartID*/
								&rx_data,	/*buf Pointer to the data buffer*/
								len);		/*size Number of bytes to be transmitted*/
		len = 0;
	}

}

void uart_dma_mode_deinit(void)
{
	HAL_Status status = HAL_ERROR;

	status = HAL_UART_DisableTxDMA(UARTID);
	if (status != HAL_OK)
		printf("uart TX disenable DMA error %d\n", status);

	status = HAL_UART_DisableRxDMA(UARTID);
	if (status != HAL_OK)
		printf("uart RX disenable DMA error %d\n", status);

	status = HAL_UART_DeInit(UARTID);
	if (status != HAL_OK)
		printf("uart deinit error %d\n", status);
}
#endif

#if (defined(UART_IT_MODE))
/**@bref Interrupt Mode:use interrupt receive & transmit data**/
void uart_it_mode(void)
{
	int32_t len;
	uint8_t rx_data;
	char buffer[20];
	uint8_t len_str;

	printf("uart%d it mode\n", UARTID);
	len_str = snprintf(buffer, 20, "uart%d it mode.\n", UARTID);
	HAL_UART_Transmit_IT(UARTID, (uint8_t *)buffer, len_str);

	while (1) {
		len = HAL_UART_Receive_IT(UARTID,	/*uartID*/
								&rx_data,	/*buf Pointer to the data buffer*/
								1,			/*size The maximum number of bytes to be received*/
								10000);		/*Timeout value in millisecond to receive data, HAL_WAIT_FOREVER for no timeout*/
		if (len)
			HAL_UART_Transmit_IT(UARTID,	/*uartID*/
								&rx_data,	/*buf Pointer to the data buffer*/
								len);		/*size Number of bytes to be transmitted*/
		len = 0;
	}

}

void uart_it_mode_deinit(void)
{
	HAL_Status status = HAL_ERROR;

	status = HAL_UART_DisableRxCallback(UARTID);
	if (status != HAL_OK)
		printf("uart RX disenable callback error %d\n", status);

	status = HAL_UART_DeInit(UARTID);
	if (status != HAL_OK)
		printf("uart deinit error %d\n", status);
}
#endif

#if (defined(UART_POLL_MODE))
/**@bref Poll Mode:poll for receive & transmit data**/
void uart_poll_mode(void)
{
	int32_t len = 0;
	uint8_t rx_data;
	char buffer[20];
	uint8_t len_str;

	printf("uart%d poll mode\n", UARTID);
	len_str = snprintf(buffer, 20, "uart%d poll mode.\n", UARTID);
	HAL_UART_Transmit_Poll(UARTID, (uint8_t *)buffer, len_str);

	while (1) {
		len = HAL_UART_Receive_Poll(UARTID,	/*uartID*/
								&rx_data,	/*buf Pointer to the data buffer*/
								1,			/*size The maximum number of bytes to be received*/
								10000);		/*Timeout value in millisecond to receive data, HAL_WAIT_FOREVER for no timeout*/
		if (len)
			HAL_UART_Transmit_Poll(UARTID,	/*uartID*/
								&rx_data,	/*buf Pointer to the data buffer*/
								len);		/*size Number of bytes to be transmitted*/
	}
}

void uart_poll_mode_deinit(void)
{
	HAL_Status status = HAL_ERROR;

	status = HAL_UART_DeInit(UARTID);
	if (status != HAL_OK)
		printf("uart deinit error %d\n", status);
}
#endif

/*Run this example, please connect the uart0 and uart 1*/
int main(void)
{
	printf("uart example started.\n\n");
	printf("uart%d will be used for echo.\n", UARTID);

	uart_init();

#if (defined(UART_DMA_MODE))
	uart_dma_mode();
#elif (defined(UART_IT_MODE))
	uart_it_mode();
#elif (defined(UART_POLL_MODE))
	uart_poll_mode();
#endif

	while (1) {
		OS_MSleep(10000);
	}

#if (defined(UART_DMA_MODE))
	uart_dma_mode_deinit();
#elif (defined(UART_IT_MODE))
	uart_it_mode_deinit();
#elif (defined(UART_POLL_MODE))
	uart_poll_mode_deinit();
#endif

	printf("uart example over.\n");

	return 0;
}
