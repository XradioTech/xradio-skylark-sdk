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
#include "driver/chip/private/hal_debug.h"
#include "driver/chip/hal_uart.h"
#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_dcache.h"
#include "driver/chip/psram/psram.h"

#ifdef __CONFIG_ROM
extern int32_t __HAL_UART_Transmit_DMA(UART_ID uartID, const uint8_t *buf, int32_t size);
extern int32_t __HAL_UART_Receive_DMA(UART_ID uartID, uint8_t *buf, int32_t size, uint32_t msec);
extern HAL_Status __HAL_UART_StartTransmit_DMA(UART_ID uartID, const uint8_t *buf, int32_t size);
extern HAL_Status __HAL_UART_StartReceive_DMA(UART_ID uartID, uint8_t *buf, int32_t size);

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
#if ((__CONFIG_CACHE_POLICY & 0xF) != 0)
    if(HAL_Dcache_IsCacheable((uint32_t)buf, size)) {
        HAL_ERR("UART: buf addr 0x%08x MUST NOT CACHEABLE!!!\n", (uint32_t)buf);
        return -1;
    }
#endif

	return __HAL_UART_Transmit_DMA(uartID, buf, size);
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
#if ((__CONFIG_CACHE_POLICY & 0xF) != 0)
    if(HAL_Dcache_IsCacheable((uint32_t)buf, size)) {
        HAL_ERR("UART: buf addr 0x%08x MUST NOT CACHEABLE!!!\n", (uint32_t)buf);
        return -1;
    }
#endif

	return __HAL_UART_Receive_DMA(uartID, buf, size, msec);
}

HAL_Status HAL_UART_StartTransmit_DMA(UART_ID uartID, const uint8_t *buf, int32_t size)
{
#if ((__CONFIG_CACHE_POLICY & 0xF) != 0)
    if(HAL_Dcache_IsCacheable((uint32_t)buf, size)) {
        HAL_ERR("UART: buf addr 0x%08x MUST NOT CACHEABLE!!!\n", (uint32_t)buf);
        return HAL_ERROR;
    }
#endif

    return __HAL_UART_StartTransmit_DMA(uartID, buf, size);

}

HAL_Status HAL_UART_StartReceive_DMA(UART_ID uartID, uint8_t *buf, int32_t size)
{
#if ((__CONFIG_CACHE_POLICY & 0xF) != 0)
    if(HAL_Dcache_IsCacheable((uint32_t)buf, size)) {
        HAL_ERR("UART: buf addr 0x%08x MUST NOT CACHEABLE!!!\n", (uint32_t)buf);
        return HAL_ERROR;
    }
#endif

    return __HAL_UART_StartReceive_DMA(uartID, buf, size);
}

#endif /*__CONFIG_ROM*/
