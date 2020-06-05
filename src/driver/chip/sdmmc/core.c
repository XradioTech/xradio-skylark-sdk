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
#include "sys/dma_heap.h"
#include "driver/chip/sdmmc/hal_sdhost.h"
#include "driver/chip/sdmmc/sdmmc.h"
#ifdef CONFIG_USE_SDIO
#include "driver/chip/sdmmc/sdio.h"
#endif
#include "driver/chip/private/hal_debug.h"
#include "driver/chip/hal_dcache.h"

#include "../hal_base.h"

#include "_sd_define.h"

#include "_sdhost.h"
#include "_core.h"
#ifdef CONFIG_USE_SDIO
#include "_sdio.h"
#endif
#ifdef CONFIG_USE_SD
#include "_sd.h"
#endif
#ifdef CONFIG_USE_MMC
#include "_mmc.h"
#endif

#ifdef __CONFIG_ROM
extern int32_t _mmc_block_read(struct mmc_card *card, uint8_t *buf, uint64_t sblk, uint32_t nblk);
extern int32_t _mmc_block_write(struct mmc_card *card, const uint8_t *buf, uint64_t sblk, uint32_t nblk);

int32_t mmc_block_read(struct mmc_card *card, uint8_t *buf, uint64_t sblk, uint32_t nblk)
{
	int32_t err;
    uint8_t *dma_buf = buf;
#if ((defined __CONFIG_PSRAM_ALL_CACHEABLE) && (defined __CONFIG_PSRAM))
    uint8_t bufIsCacheable = HAL_Dcache_IsCacheable((uint32_t)buf, (uint32_t)(nblk*512));
    if(bufIsCacheable) {
        dma_buf = dma_malloc(nblk*512, DMAHEAP_PSRAM);
        if(dma_buf == NULL) {
            HAL_ERR("dma_malloc failed\n");
            return -1;
        }
    }
#endif

	HAL_SDC_Claim_Host(card->host);
	err = _mmc_block_read(card, dma_buf, sblk, nblk);
	HAL_SDC_Release_Host(card->host);
#if ((defined __CONFIG_PSRAM_ALL_CACHEABLE) && (defined __CONFIG_PSRAM))
    if(bufIsCacheable) {
        HAL_Memcpy(buf, dma_buf, nblk*512);
        dma_free(dma_buf, DMAHEAP_PSRAM);
    }
#endif
	return err;
}

int32_t mmc_block_write(struct mmc_card *card, const uint8_t *buf, uint64_t sblk, uint32_t nblk)
{
	int32_t err;
    uint8_t *dma_buf = (uint8_t *)buf;
#if ((defined __CONFIG_PSRAM_ALL_CACHEABLE) && (defined __CONFIG_PSRAM))
    uint8_t bufIsCacheable = HAL_Dcache_IsCacheable((uint32_t)buf, (uint32_t)(nblk*512));
    if(bufIsCacheable) {
        dma_buf = dma_malloc(nblk*512, DMAHEAP_PSRAM);
        if(dma_buf == NULL) {
            HAL_ERR("dma_malloc failed\n");
            return -1;
        }
        HAL_Memcpy(dma_buf, buf, nblk*512);
    }
#endif
	HAL_SDC_Claim_Host(card->host);
	err = _mmc_block_write(card, dma_buf, sblk, nblk);
	HAL_SDC_Release_Host(card->host);
#if ((defined __CONFIG_PSRAM_ALL_CACHEABLE) && (defined __CONFIG_PSRAM))
    if(bufIsCacheable) {
        dma_free(dma_buf, DMAHEAP_PSRAM);
    }
#endif
	return err;
}
#endif

uint32_t mmc_get_capacity(struct mmc_card *card)
{
	if (!card) {
		HAL_ERR("card not exist\n");
		return 0;
	}
	return card->csd.capacity;
}

