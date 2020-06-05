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
#include <stdlib.h>
#include <string.h>
#include "sys/dma_heap.h"
#include "driver/chip/private/hal_debug.h"
#include "driver/chip/private/hal_os.h"
#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_icache.h"
#include "driver/chip/hal_dcache.h"
#include "driver/chip/hal_nvic.h"

#ifdef __CONFIG_ROM

#if (defined __CONFIG_PSRAM_ALL_CACHEABLE)

enum DMAHEAP_RECORD_FLAG
{
    DMAHEAP_RECORD_SRC              = (1 << 0),
    DMAHEAP_RECORD_DST              = (1 << 1),
    DMAHEAP_RECORD_DST_HALF_COPIED  = (1 << 2),
    DMAHEAP_RECORD_DST_END_COPIED   = (1 << 3),
};

struct _dmaheap_record
{
    uint32_t flag;
    uint32_t dataLen;
    uint8_t *srcAddr;
    uint8_t *dstAddr;
    uint8_t *outAddr;
};

typedef struct {
    DMA_IRQCallback     endCallback;
    void               *endArg;
#if HAL_DMA_OPT_TRANSFER_HALF_IRQ
    DMA_IRQCallback     halfCallback;
    void               *halfArg;
#endif
} DMA_Private;

#define DMAHEAP_SRC_SET_RECORD(chan, addr) \
    do { \
        g_dmaheap_record[chan].srcAddr = addr; \
        g_dmaheap_record[chan].flag |= DMAHEAP_RECORD_SRC; \
    } while(0)

#define DMAHEAP_DST_SET_RECORD(chan, dstaddr, outaddr, datalen) \
    do { \
        g_dmaheap_record[chan].dstAddr = dstaddr; \
        g_dmaheap_record[chan].outAddr = (uint8_t*)outaddr; \
        g_dmaheap_record[chan].dataLen =  datalen; \
        g_dmaheap_record[chan].flag |= DMAHEAP_RECORD_DST; \
    } while(0)

#define DMAHEAP_SRC_CLR_RECORD(chan) \
    do { \
        g_dmaheap_record[chan].srcAddr = NULL; \
        g_dmaheap_record[chan].flag &= (~DMAHEAP_RECORD_SRC); \
    } while(0)

#define DMAHEAP_DST_CLR_RECORD(chan) \
    do { \
        g_dmaheap_record[chan].dstAddr = NULL; \
        g_dmaheap_record[chan].outAddr = NULL; \
        g_dmaheap_record[chan].dataLen = 0; \
        g_dmaheap_record[chan].flag &= (~(DMAHEAP_RECORD_DST | DMAHEAP_RECORD_DST_END_COPIED | DMAHEAP_RECORD_DST_HALF_COPIED)); \
    } while(0)

#define DMAHEAP_SRC_IS_RECORD(chan) (g_dmaheap_record[chan].flag & DMAHEAP_RECORD_SRC)
#define DMAHEAP_DST_IS_RECORD(chan) (g_dmaheap_record[chan].flag & DMAHEAP_RECORD_DST)
#define DMA_IRQ_ALL_BITS            ((1 << (DMA_CHANNEL_NUM << 1)) - 1)

extern DMA_Private gDMAPrivate[DMA_CHANNEL_NUM];
static struct _dmaheap_record g_dmaheap_record[DMA_CHANNEL_NUM];

__STATIC_INLINE DMA_WorkMode DMA_GetWorkMode(DMA_Channel chan)
{
    return HAL_GET_BIT_VAL(DMA->CHANNEL[chan].CTRL, DMA_WORK_MODE_SHIFT, DMA_WORK_MODE_VMASK);
}

__STATIC_INLINE DMA_ByteCntMode DMA_GetByteCntMode(DMA_Channel chan)
{
    return HAL_GET_BIT_VAL(DMA->CHANNEL[chan].CTRL, DMA_BYTE_CNT_MODE_SHIFT, DMA_BYTE_CNT_MODE_VMASK);
}

void DMA_IRQHandler(void)
{
    uint32_t i;
    uint32_t irqStatus;
#if HAL_DMA_OPT_TRANSFER_HALF_IRQ
    uint32_t isHalfPending;
#endif
    uint32_t isEndPending;
    DMA_Private *priv;

    irqStatus = DMA->IRQ_STATUS & DMA->IRQ_EN & DMA_IRQ_ALL_BITS; /* get pending bits */
    DMA->IRQ_STATUS = irqStatus; /* clear pending bits */
    priv = gDMAPrivate;

    for (i = DMA_CHANNEL_0; i < DMA_CHANNEL_NUM && irqStatus != 0; ++i) {
#if HAL_DMA_OPT_TRANSFER_HALF_IRQ
        isHalfPending = irqStatus & HAL_BIT(0);
        if (isHalfPending) {
#if (defined __CONFIG_PSRAM_ALL_CACHEABLE)
            if (DMAHEAP_DST_IS_RECORD(i)) {
                HAL_Memcpy(g_dmaheap_record[i].outAddr,
                           g_dmaheap_record[i].dstAddr,
                           g_dmaheap_record[i].dataLen / 2);
                g_dmaheap_record[i].flag |= DMAHEAP_RECORD_DST_HALF_COPIED;
                g_dmaheap_record[i].flag &= ~DMAHEAP_RECORD_DST_END_COPIED;
            }
#endif /*(defined __CONFIG_PSRAM_ALL_CACHEABLE)*/
            if (priv[i].halfCallback) {
                priv[i].halfCallback(priv[i].halfArg);
            }
        }
#endif /* HAL_DMA_OPT_TRANSFER_HALF_IRQ */
        isEndPending = irqStatus & HAL_BIT(1);
        if (isEndPending) {
#if (defined __CONFIG_PSRAM_ALL_CACHEABLE)
            if (DMAHEAP_DST_IS_RECORD(i)) {
#if HAL_DMA_OPT_TRANSFER_HALF_IRQ
                if((g_dmaheap_record[i].flag & DMAHEAP_RECORD_DST_HALF_COPIED)) {
                    uint32_t halfLen = g_dmaheap_record[i].dataLen / 2;
                    HAL_Memcpy(g_dmaheap_record[i].outAddr + halfLen,
                               g_dmaheap_record[i].dstAddr + halfLen,
                               g_dmaheap_record[i].dataLen - halfLen);
                    g_dmaheap_record[i].flag &= ~DMAHEAP_RECORD_DST_HALF_COPIED;
                } else
#endif /* HAL_DMA_OPT_TRANSFER_HALF_IRQ */
                {
                    HAL_Memcpy(g_dmaheap_record[i].outAddr,
                               g_dmaheap_record[i].dstAddr,
                               g_dmaheap_record[i].dataLen);
                }
                g_dmaheap_record[i].flag |= DMAHEAP_RECORD_DST_END_COPIED;
            }
#endif /* (defined __CONFIG_PSRAM_ALL_CACHEABLE) */
            if (priv[i].endCallback) {
                priv[i].endCallback(priv[i].endArg);
            }
        }
        irqStatus >>= 2;
    }
}

/**
 * @brief Stop the DMA transfer of the specified DMA channel
 * @param[in] chan DMA channel
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_DMA_Stop(DMA_Channel chan)
{
    if (chan >= DMA_CHANNEL_NUM) {
        HAL_DBG("invalid dma chan %d\n", chan);
        return HAL_INVALID;
    }
    HAL_CLR_BIT(DMA->CHANNEL[chan].CTRL, DMA_START_BIT); /* NB: it will reset the channel */
#if (defined __CONFIG_PSRAM_ALL_CACHEABLE)
    if (DMAHEAP_SRC_IS_RECORD(chan)) {
        dma_free(g_dmaheap_record[chan].srcAddr, DMAHEAP_PSRAM);
        DMAHEAP_SRC_CLR_RECORD(chan);
    }

    if (DMAHEAP_DST_IS_RECORD(chan)) {
        uint8_t *copySrc;
        uint8_t *copyDst;
        int32_t copyLen;
        uint32_t remainCnt;
        if (DMA_GetByteCntMode(chan) == DMA_BYTE_CNT_MODE_REMAIN) {
            remainCnt = DMA->CHANNEL[chan].BYTE_CNT & DMA_BYTE_CNT_VMASK;
        } else {
            remainCnt = 0;
        }

        if (g_dmaheap_record[chan].flag & DMAHEAP_RECORD_DST_END_COPIED) {
            copyLen = 0;
        }
#if HAL_DMA_OPT_TRANSFER_HALF_IRQ
        else if (g_dmaheap_record[chan].flag & DMAHEAP_RECORD_DST_HALF_COPIED) {
            uint32_t halfLen = g_dmaheap_record[chan].dataLen / 2;
            copySrc = g_dmaheap_record[chan].dstAddr + halfLen;
            copyDst = g_dmaheap_record[chan].outAddr + halfLen;
            copyLen = g_dmaheap_record[chan].dataLen - halfLen - remainCnt;
        }
#endif
        else {
            copySrc = g_dmaheap_record[chan].dstAddr;
            copyDst = g_dmaheap_record[chan].outAddr;
            copyLen = g_dmaheap_record[chan].dataLen - remainCnt;
        }

        if (copyLen > 0) {
            HAL_Memcpy(copyDst, copySrc, copyLen);
        }
        dma_free(g_dmaheap_record[chan].dstAddr, DMAHEAP_PSRAM);
        DMAHEAP_DST_CLR_RECORD(chan);
    }
#endif
    return HAL_OK;
}

#endif /* (defined __CONFIG_PSRAM_ALL_CACHEABLE) */

HAL_Status __HAL_DMA_Init(DMA_Channel chan, const DMA_ChannelInitParam *param);
HAL_Status HAL_DMA_Init(DMA_Channel chan, const DMA_ChannelInitParam *param)
{
    HAL_Status ret;
    ret = __HAL_DMA_Init(chan, param);
    if (ret != HAL_OK) {
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
    uint32_t dma_srcAddr = srcAddr;
    uint32_t dma_dstAddr = dstAddr;

    if (chan >= DMA_CHANNEL_NUM) {
        HAL_ERR("invalid dma chan %d\n", chan);
        return HAL_INVALID;
    }
    if ((datalen > DMA_DATA_MAX_LEN) || (datalen == 0)) {
        HAL_ERR("invalid dma data len %u\n", datalen);
        return HAL_INVALID;
    }

    /* TODO: check alignment of @srcAddr and @dstAddr */
#if (defined __CONFIG_PSRAM_ALL_CACHEABLE)
    if (HAL_Dcache_IsCacheable(srcAddr, datalen)) {
        if (DMA_GetWorkMode(chan) == DMA_WORK_MODE_CIRCULAR) {
            HAL_ERR("DMA repeat transfer NOT SUPPORTED\n");
            goto err;
        }
        uint8_t *tmp = dma_malloc(datalen, DMAHEAP_PSRAM);
        if (tmp == NULL) {
            HAL_ERR("dma_malloc failed, datalen = %u\n", datalen);
            goto err;
        }
        HAL_Memcpy(tmp, (uint8_t *)srcAddr, datalen);
        DMAHEAP_SRC_SET_RECORD(chan, tmp);
        dma_srcAddr = (uint32_t)tmp;
    }

    if (HAL_Dcache_IsCacheable(dstAddr, datalen)) {
        if (DMA_GetWorkMode(chan) == DMA_WORK_MODE_CIRCULAR) {
            HAL_ERR("DMA repeat transfer NOT SUPPORTED\n");
            goto err;
        }
        uint8_t *tmp = dma_malloc(datalen, DMAHEAP_PSRAM);
        if (tmp == NULL) {
            HAL_ERR("dma_malloc failed, datalen = %u\n", datalen);
            goto err;
        }
        DMAHEAP_DST_SET_RECORD(chan, tmp, dstAddr, datalen);
        dma_dstAddr = (uint32_t)tmp;
    }
#endif

    DMA->CHANNEL[chan].SRC_ADDR = dma_srcAddr;
    DMA->CHANNEL[chan].DST_ADDR = dma_dstAddr;
    DMA->CHANNEL[chan].BYTE_CNT = datalen & DMA_BYTE_CNT_VMASK;
    HAL_SET_BIT(DMA->CHANNEL[chan].CTRL, DMA_START_BIT);
    return HAL_OK;

#if (defined __CONFIG_PSRAM_ALL_CACHEABLE)
err:
    if (dma_srcAddr != srcAddr) {
        DMAHEAP_SRC_CLR_RECORD(chan);
        dma_free((void *)dma_srcAddr, DMAHEAP_PSRAM);
    }
    if (dma_dstAddr != dstAddr) {
        DMAHEAP_DST_CLR_RECORD(chan);
        dma_free((void *)dma_dstAddr, DMAHEAP_PSRAM);
    }
    return HAL_ERROR;
#endif
}

#if (defined __CONFIG_PSRAM_ALL_CACHEABLE)
static void internal_dma_config(DMA_Channel chan)
{
    if (HAL_DMA_RequestSpecified(chan) == DMA_CHANNEL_INVALID) {
        HAL_ERR("Request dma %d Failed\n", chan);
        return;
    }
    DMA->IRQ_STATUS = DMA_IRQ_ALL_BITS;
    HAL_NVIC_ConfigExtIRQ(DMA_IRQn, DMA_IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);
    DMA_EnableIRQ(chan, DMA_IRQ_TYPE_END);
}

static void internal_dma_deconfig(DMA_Channel chan)
{
    DMA_DisableIRQ(chan, DMA_IRQ_TYPE_END);
    if ((DMA->IRQ_EN & DMA_IRQ_ALL_BITS) == 0) {
        HAL_NVIC_DisableIRQ(DMA_IRQn);
    }
    HAL_DMA_Release(chan);
}

#ifdef CONFIG_PM
static int internal_dma_suspend(struct soc_device *dev, enum suspend_state_t state)
{
    switch (state) {
    case PM_MODE_SLEEP:
        break;
    case PM_MODE_STANDBY:
    case PM_MODE_HIBERNATION:
        internal_dma_deconfig(DMA_CHANNEL_7);
        break;
    default:
        break;
    }
    return 0;
}

static int internal_dma_resume(struct soc_device *dev, enum suspend_state_t state)
{
    switch (state) {
    case PM_MODE_SLEEP:
        break;
    case PM_MODE_STANDBY:
    case PM_MODE_HIBERNATION:
        internal_dma_config(DMA_CHANNEL_7);
        break;
    default:
        break;
    }
    return 0;
}

static const struct soc_device_driver internal_dma_drv = {
    .name = "internal_dma",
    .suspend_noirq = internal_dma_suspend,
    .resume_noirq = internal_dma_resume,
};

static struct soc_device internal_dma_dev = {
    .name = "internal_dma",
    .driver = &internal_dma_drv,
};
#endif /* CONFIG_PM */

void internal_dma_init(void)
{
    internal_dma_config(DMA_CHANNEL_7);
#ifdef CONFIG_PM
    pm_register_ops(&internal_dma_dev);
#endif
}

#endif /*(defined __CONFIG_PSRAM_ALL_CACHEABLE)*/

#endif/*__CONFIG_ROM*/
