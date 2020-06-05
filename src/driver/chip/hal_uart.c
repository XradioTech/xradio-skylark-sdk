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

#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_uart.h"
#include "pm/pm.h"
#include "hal_base.h"

#if (defined(__CONFIG_ROM) && defined(__CONFIG_PSRAM_ALL_CACHEABLE))

#if 1 //HAL_UART_OPT_DMA
typedef struct {
	DMA_ChannelInitParam	param;
	DMA_Channel             chan;
	HAL_Semaphore           sem;
} UART_DMAPrivate;
#endif

#if 1 //HAL_UART_OPT_IT
typedef struct {
	uint8_t                *buf;
	int32_t                 bufSize;
	HAL_Semaphore           sem;
} UART_ITPrivate;
#endif

typedef struct {
	UART_T                 *uart;
#if 1 //HAL_UART_OPT_DMA
	UART_DMAPrivate        *txDMA;
	UART_DMAPrivate        *rxDMA;
#endif
#if 1 //HAL_UART_OPT_IT
	UART_ITPrivate         *txIT;
	UART_ITPrivate         *rxIT;

	UART_RxReadyCallback    rxReadyCallback;
	void                   *arg;
#endif
	/* UARTx->IIR_FCR.FIFO_CTRL is write only, shadow its value */
	uint8_t                IIR_FCR_FIFO_CTRL;
#if 1 //def CONFIG_PM
	uint8_t                 bypassPmMode;
	uint8_t                 txDelay;
	UART_InitParam          param;
	struct soc_device       dev;
#endif
} UART_Private;

extern UART_Private *gUartPrivate[UART_NUM];

__STATIC_INLINE UART_Private *UART_GetUartPriv(UART_ID uartID)
{
	if (uartID < UART_NUM) {
		return gUartPrivate[uartID];
	} else {
		return NULL;
	}
}

__STATIC_INLINE UART_T *UART_GetInstance(UART_Private *priv)
{
	return priv->uart;
}

#if 1 //HAL_UART_OPT_DMA

HAL_Status HAL_UART_StartTransmit_DMA(UART_ID uartID, const uint8_t *buf, int32_t size)
{
	UART_T *uart;
	UART_Private *priv;

	if (buf == NULL || size <= 0) {
		return HAL_ERROR;
	}

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

	if (priv->txDMA == NULL) {
		HAL_WRN("tx dma not enable\n");
		return HAL_ERROR;
	}

	uart = UART_GetInstance(priv);
	HAL_DMA_Start(priv->txDMA->chan,
	              (uint32_t)buf,
		          (uint32_t)&uart->RBR_THR_DLL.TX_HOLD,
		          size);
	return HAL_OK;
}

int32_t HAL_UART_StopTransmit_DMA(UART_ID uartID)
{
	UART_Private *priv;
	DMA_Channel chan;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return -1;
	}

	if (priv->txDMA == NULL) {
		HAL_WRN("tx dma not enable\n");
		return -1;
	}

	chan = priv->txDMA->chan;
	HAL_DMA_Stop(chan);

	return HAL_DMA_GetByteCount(chan);
}

HAL_Status HAL_UART_StartReceive_DMA(UART_ID uartID, uint8_t *buf, int32_t size)
{
	UART_T *uart;
	UART_Private *priv;

	if (buf == NULL || size <= 0) {
		return HAL_ERROR;
	}

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return HAL_ERROR;
	}

#if HAL_UART_OPT_IT
	if (priv->rxReadyCallback != NULL) {
		HAL_WRN("rx cb is enabled\n");
		return HAL_ERROR;
	}
#endif

	if (priv->rxDMA == NULL) {
		HAL_WRN("rx dma not enable\n");
		return HAL_ERROR;
	}

	uart = UART_GetInstance(priv);
	HAL_DMA_Start(priv->rxDMA->chan,
	              (uint32_t)&uart->RBR_THR_DLL.RX_BUF,
	              (uint32_t)buf,
	              size);
	return HAL_OK;
}

int32_t HAL_UART_StopReceive_DMA(UART_ID uartID)
{
	UART_Private *priv;
	DMA_Channel chan;

	priv = UART_GetUartPriv(uartID);
	if (priv == NULL) {
		HAL_DBG("uart %d not inited\n", uartID);
		return -1;
	}

	if (priv->rxDMA == NULL) {
		HAL_WRN("rx dma not enable\n");
		return -1;
	}

	chan = priv->rxDMA->chan;
	HAL_DMA_Stop(chan);

	return HAL_DMA_GetByteCount(chan);
}

/**
 * @brief Transmit an amount of data in DMA mode
 *
 * Steps to transmit data in DMA mode:
 *     - use HAL_UART_EnableTxDMA() to enable UART transmit DMA mode
 *     - use HAL_UART_Transmit_DMA() to transmit data, it can be called
 *       repeatedly after HAL_UART_EnableTxDMA()
 *     - use HAL_UART_DisableTxDMA() to disable UART transmit DMA mode if needed
 *
 * @param[in] uartID ID of the specified UART
 * @param[in] buf Pointer to the data buffer
 * @param[in] size Number of bytes to be transmitted
 * @return Number of bytes transmitted, -1 on error
 *
 * @note This function is not thread safe. If using the UART transmit series
 *       functions in multi-thread, make sure they are executed exclusively.
 * @note To transmit data in DMA mode, HAL_UART_EnableTxDMA() MUST be executed
 *       before calling this function.
 */
int32_t HAL_UART_Transmit_DMA(UART_ID uartID, const uint8_t *buf, int32_t size)
{
	UART_Private *priv;
	int32_t left;

	if (HAL_UART_StartTransmit_DMA(uartID, buf, size) != HAL_OK) {
		return -1;
	}

	priv = UART_GetUartPriv(uartID);
	HAL_SemaphoreWait(&priv->txDMA->sem, HAL_WAIT_FOREVER);

	left = HAL_UART_StopTransmit_DMA(uartID);
	if (left < 0) {
		return -1;
	}

	return (size - left);
}

/**
 * @brief Receive an amount of data in DMA mode
 *
 * Steps to receive data in DMA mode:
 *     - use HAL_UART_EnableRxDMA() to enable UART receive DMA mode
 *     - use HAL_UART_Receive_DMA() to receive data, it can be called
 *       repeatedly after HAL_UART_EnableRxDMA()
 *     - use HAL_UART_DisableRxDMA() to disable UART receive DMA mode if needed
 *
 * @param[in] uartID ID of the specified UART
 * @param[out] buf Pointer to the data buffer
 * @param[in] size The maximum number of bytes to be received.
 *                 The actual received bytes can be less than this.
 * @param[in] msec Timeout value in millisecond to receive data.
 *                 HAL_WAIT_FOREVER for no timeout.
 * @return Number of bytes received, -1 on error
 *
 * @note This function is not thread safe. If using the UART receive series
 *       functions in multi-thread, make sure they are executed exclusively.
 * @note To receive data in DMA mode, HAL_UART_EnableRxDMA() MUST be executed
 *       before calling this function.
 */
int32_t HAL_UART_Receive_DMA(UART_ID uartID, uint8_t *buf, int32_t size, uint32_t msec)
{
	UART_Private *priv;
	int32_t left;

	if (HAL_UART_StartReceive_DMA(uartID, buf, size) != HAL_OK) {
		return -1;
	}

	priv = UART_GetUartPriv(uartID);
	HAL_SemaphoreWait(&priv->rxDMA->sem, msec);

	left = HAL_UART_StopReceive_DMA(uartID);
	if (left < 0) {
		return -1;
	}

	return (size - left);
}

#endif /* HAL_UART_OPT_DMA */

#endif /* (defined(__CONFIG_ROM) && defined(__CONFIG_PSRAM_ALL_CACHEABLE)) */
