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
#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_icache.h"
#include "driver/chip/hal_dcache.h"
#ifdef __CONFIG_ROM
HAL_Status __HAL_DMA_Init(DMA_Channel chan, const DMA_ChannelInitParam *param);
HAL_Status HAL_DMA_Init(DMA_Channel chan, const DMA_ChannelInitParam *param)
{
    HAL_Status ret;
    ret = __HAL_DMA_Init(chan, param);
    if(ret != HAL_OK) {
        return ret;
    }
    HAL_SET_BIT(DMA->CONFIG, DMA_AUTO_CLK_GAT_BIT);
    return HAL_OK;
}


/**
 * @brief Start the DMA transfer of the specified DMA channel
 * @param[in] chan DMA channel
 * @param[in] srcAddr The source address of DMA transfer
 * @param[in] dstAddr The destination address of DMA transfer
 * @param[in] datalen The length of data to be transferred from source to destination
 * @retval HAL_Status, HAL_OK on success
 *
 * @note The source/destination address MUST be aligned to the
 *       source/destination DMA transaction data width defined by DMA_DataWidth.
 * @note The date length MUST not be more than DMA_DATA_MAX_LEN.
 * @note The srcAddr and dstAddr not spupport reentrant
         Before dma start, clean src address part of dcache to sram;
         After dma end, flush dest address part of dcache.
 */
HAL_Status HAL_DMA_Start(DMA_Channel chan, uint32_t srcAddr, uint32_t dstAddr, uint32_t datalen)
{
	if (chan >= DMA_CHANNEL_NUM) {
		printf("invalid dma chan %d\n", chan);
		return HAL_INVALID;
	}
	if ((datalen > DMA_DATA_MAX_LEN) || (datalen == 0)) {
		printf("invalid dma data len %u\n", datalen);
		return HAL_INVALID;
	}

	/* TODO: check alignment of @srcAddr and @dstAddr */
#if (((__CONFIG_CACHE_POLICY & 0xF) != 0) && (defined __CONFIG_PSRAM))
    if((srcAddr >= IDCACHE_START_ADDR) && (srcAddr < IDCACHE_END_ADDR)) {
        HAL_Dcache_Clean(srcAddr, datalen);
    }
    if((srcAddr >= IDCACHE_START_ADDR) && (srcAddr < IDCACHE_END_ADDR)) {
        HAL_Dcache_FlushClean(dstAddr, datalen);
    }
#endif

	DMA->CHANNEL[chan].SRC_ADDR = srcAddr;
	DMA->CHANNEL[chan].DST_ADDR = dstAddr;
	DMA->CHANNEL[chan].BYTE_CNT = datalen & DMA_BYTE_CNT_VMASK;
	HAL_SET_BIT(DMA->CHANNEL[chan].CTRL, DMA_START_BIT);

	return HAL_OK;
}

#endif/*__CONFIG_ROM*/

