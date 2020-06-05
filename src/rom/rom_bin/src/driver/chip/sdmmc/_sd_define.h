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

#ifndef _ROM_DRIVER_CHIP_SDMMC__SD_DEFINE_H_
#define _ROM_DRIVER_CHIP_SDMMC__SD_DEFINE_H_

#include "rom/driver/chip/hal_util.h"
#include "rom/rom_debug.h"
#include "rom/driver/chip/private/hal_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SD_DEBUG        1
#define SDC_DEBUG       1
#define SD_ABORT_ON     0

#if SD_DEBUG
#define SD_LOGD(format, args...) ROM_DBG(card->debug_mask, format, ##args)
#define SD_LOGN(format, args...) ROM_INF(card->debug_mask, format, ##args)
#define SD_LOGW(format, args...) ROM_WRN(card->debug_mask, format, ##args)
#define SD_LOGW_RAW(mask, format, args...) ROM_WRN(mask, format, ##args)
#define SD_LOGE(format, args...) ROM_ERR(card->debug_mask, format, ##args)
#define SD_LOGE_RAW(mask, format, args...) ROM_ERR(mask, format, ##args)
#define SD_LOGA(format, args...) ROM_ANY(card->debug_mask, format, ##args)
#define sd_hex_dump_bytes(a, l) ROM_DUMP_BYTES(card->debug_mask, a, l)
#else
#define SD_LOGD(x...)
#define SD_LOGN(x...)
#define SD_LOGW(x...)
#define SD_LOGW_RAW(x...)
#define SD_LOGE(x...)
#define SD_LOGE_RAW(x...)
#define SD_LOGA(x...)
#define sd_hex_dump_bytes(a, l)
#endif
#if SD_ABORT_ON
#define SD_ABORT() while (1)
#else
#define SD_ABORT()
#endif

#define SD_BUG_ON(v) do {if(v) {printf("BUG at %s:%d!\n", __func__, __LINE__); SD_ABORT();}} while (0)
#define SD_WARN_ON(v) do {if(v) {printf("WARN at %s:%d!\n", __func__, __LINE__);}} while (0)

#define MMC_CMD_RETRIES        3

#if SDC_DEBUG
#define SDC_LOGD(format, args...) ROM_DBG(host->debug_mask, format, ##args)
#define SDC_LOGN(format, args...) ROM_INF(host->debug_mask, format, ##args)
#define SDC_LOGW(format, args...) ROM_WRN(host->debug_mask, format, ##args)
#define SDC_LOGW_RAW(mask, format, args...) ROM_WRN(mask, format, ##args)
#define SDC_LOGE(format, args...) ROM_ERR(host->debug_mask, format, ##args)
#define SDC_LOGE_RAW(mask, format, args...) ROM_ERR(mask, format, ##args)
#define SDC_LOGA(format, args...) ROM_ANY(host->debug_mask, format, ##args)
#else
#define SDC_LOGD(x...)
#define SDC_LOGN(x...)
#define SDC_LOGW(x...)
#define SDC_LOGW_RAW(x...)
#define SDC_LOGE(x...)
#define SDC_LOGE_RAW(x...)
#define SDC_LOGA(x...)
#endif

#define SDC_BUG_ON(v) ROM_BUG_ON(v)
#define SDC_WARN_ON(v) ROM_WARN_ON(v)

/* debug in interrupt handler */
#ifdef __CONFIG_XIP_SECTION_FUNC_LEVEL
#define SDC_IT_LOGD(fmt, arg...)	ROM_IT_DBG(0, fmt, ##arg)
#define SDC_IT_LOGN(fmt, arg...)	ROM_IT_WRN(SDC_DEBUG, fmt, ##arg)
#define SDC_IT_LOGE(fmt, arg...)	ROM_IT_ERR(SDC_DEBUG, fmt, ##arg)
#define SDC_IT_LOGE_RAW(mask, fmt, arg...)	ROM_IT_ERR(mask, fmt, ##arg)
#else /* __CONFIG_XIP_SECTION_FUNC_LEVEL */
#define SDC_IT_LOGD	SDC_LOGD
#define SDC_IT_LOGN	SDC_LOGN
#define SDC_IT_LOGE	SDC_LOGE
#define SDC_IT_LOGE_RAW	SDC_LOGE_RAW
#endif /* __CONFIG_XIP_SECTION_FUNC_LEVEL */

#define SDC_Memset(d, c, l) HAL_Memset(d, c, l)

#define mmc_mdelay(ms) HAL_MSleep(ms)
#define mmc_udelay(us) HAL_UDelay(us)

#if ((defined CONFIG_USE_SD) || (defined CONFIG_USE_MMC))
#define SDC_DMA_TIMEOUT         2000 /* not much data to write on this platform */
#else
#define SDC_DMA_TIMEOUT         300
#endif
#define SDC_THREAD_TIMEOUT      (SDC_DMA_TIMEOUT + 50)

#define SDC_SemCreate(l, n)     OS_SemaphoreCreate(l, n, OS_SEMAPHORE_MAX_COUNT)
#define SDC_SemDel(l)           OS_SemaphoreDelete(l)
#define SDC_SemPend(l, t)       OS_SemaphoreWait(l, t)
#define SDC_SemPost(l)          OS_SemaphoreRelease(l)

#define SDC_MutexCreate(m)      OS_MutexCreate(m)
#define SDC_MutexDelete(m)      OS_MutexDelete(m)
#define SDC_MutexLock(m, t)     OS_MutexLock(m, t)
#define SDC_MutexUnlock(m)      OS_MutexUnlock(m);

#define SDC_InitTimer(t, cb, arg, pms)  OS_TimerCreate(t, OS_TIMER_ONCE, cb, arg, pms)
#define SDC_StartTimer(t)               OS_TimerStart(t)
#define SDC_StopTimer(t)                OS_TimerStop(t)
#define SDC_DelTimer(t)                 OS_TimerDelete(t)
#define SDC_ModTimer(t, ms)             do {if (!ms) SDC_BUG_ON(1); \
                                            OS_TimerChangePeriod(t, ms);} while (0)
#define SDC_TimerIsActive(t)            OS_TimerIsActive(t)

#ifdef __CONFIG_XIP_SECTION_FUNC_LEVEL
#define SDC_IT_ModTimer(t, ms)          OS_TimerChangePeriod(t, ms)
#else
#define SDC_IT_ModTimer(t, ms)          SDC_ModTimer(t, ms)
#endif


#ifdef __cplusplus
}
#endif

#endif /* _ROM_DRIVER_CHIP_SDMMC__SD_DEFINE_H_ */
