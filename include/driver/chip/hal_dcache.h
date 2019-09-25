/**
  * @file	hal_dcache.h
  * @author  XRADIO IOT WLAN Team
  */

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

#ifndef _DRIVER_CHIP_HAL_DCACHE_H_
#define _DRIVER_CHIP_HAL_DCACHE_H_

#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (__CONFIG_CHIP_ARCH_VER == 2)

#define DCACHE_ADDR_MAX         3

typedef struct
{
	__IO uint32_t START_ADDR;       /* ,        Address offset: N * 0x4 + 0x00        */
	__IO uint32_t END_ADDR;         /* ,        Address offset: N * 0x4 + 0x04        */
	__I  uint32_t RESERVE08[2];
} WRITE_THROUGH_ADDR_T;

typedef struct {
	__IO uint32_t DCACHE_COM_CFG;   /* ,     Address offset: 0x000   */
	__I uint32_t RESERVE04[3];
	WRITE_THROUGH_ADDR_T WT_ADDR[DCACHE_ADDR_MAX];  /* ,     Address offset: 0x010   */
	__I uint32_t RESERVE40[24];
	__I uint32_t MISS_COUNT_H;      /* ,     Address offset: 0x0A0   */
	__I uint32_t MISS_COUNT_L;      /* ,     Address offset: 0x0A4   */
	__I uint32_t HIT_COUNT_H;       /* ,     Address offset: 0x0A8   */
	__I uint32_t HIT_COUNT_L;       /* ,     Address offset: 0x0AC   */
	__I uint32_t DCACHE_STA;        /* ,     Address offset: 0x0B0   */
    __I uint32_t RESERVEB4[3];
	__IO uint32_t CLEAN_FLUSH_SADDR; /* Address offset: 0x0C0, clean or flush start addr */
	__IO uint32_t CLEAN_FLUSH_LEN;   /* Address offset: 0x0C4, clean or flush length   */
} DCACHE_T;

#define DCACHE_CTRL ((DCACHE_T *)PSRAM_DCACHE_BASE)

/* DCACHE_COM_CFG */
#define DCACHE_ENABLE_SHIFT             (0)
#define DCACHE_ENABLE_MASK              (0x1U << DCACHE_ENABLE_SHIFT)

#define DCACHE_ASSOCIATE_MODE_SHIFT     (3)
#define DCACHE_ASSOCIATE_MODE_MASK      (0x3U << DCACHE_ASSOCIATE_MODE_SHIFT)
#define DCACHE_ASSOCIATE_DIRECT_MOD     (0 << DCACHE_ASSOCIATE_MODE_SHIFT)
#define DCACHE_ASSOCIATE_TWO_WAY_MOD    (1 << DCACHE_ASSOCIATE_MODE_SHIFT)
#define DCACHE_ASSOCIATE_FOUR_WAY_MOD   (2 << DCACHE_ASSOCIATE_MODE_SHIFT)

#define DCACHE_EN_VICTIM_SHIFT          (5)
#define DCACHE_EN_VICTIM_MASK           (0x1U << DCACHE_EN_VICTIM_SHIFT)
#define DCACHE_EN_VICTIM                (0x1U << DCACHE_EN_VICTIM_SHIFT)

#define DCACHE_EN_RD_WRAP_SHIFT         (6)
#define DCACHE_EN_RD_WRAP_MASK          (0x1U << DCACHE_EN_RD_WRAP_SHIFT)
#define DCACHE_EN_RD_WRAP               (0x1U << DCACHE_EN_RD_WRAP_SHIFT)

#define DCACHE_COUNTER_EN_SHIFT         (7)
#define DCACHE_COUNTER_EN_MASK          (0x1U << DCACHE_COUNTER_EN_SHIFT)

#define DCACHE_CLEAN_START_SHIFT        (8)
#define DCACHE_CLEAN_START_MASK         (0x1U << DCACHE_CLEAN_START_SHIFT)

#define DCACHE_FLUSH_START_SHIFT        (9)
#define DCACHE_FLUSH_START_MASK         (0x1U << DCACHE_FLUSH_START_SHIFT)

#define DCACHE_FLUSH_CLEAN_START_SHIFT  (10)
#define DCACHE_FLUSH_CLEAN_START_MASK   (0x1U << DCACHE_FLUSH_CLEAN_START_SHIFT)

#define DCACHE_MIXED_IDBUS_EN_SHIFT     (11)
#define DCACHE_MIXED_IDBUS_EN           (0x1U << DCACHE_MIXED_IDBUS_EN_SHIFT)

#define DCACHE_MIXED_MODE_IDSHIFT       (12)
#define DCACHE_MIXED_MODE_ICACHE        (0x0U << DCACHE_MIXED_MODE_IDSHIFT)
#define DCACHE_MIXED_MODE_DCACHE        (0x1U << DCACHE_MIXED_MODE_IDSHIFT)

#define DCACHE_MIXED_MODE_SHIFT         (DCACHE_MIXED_IDBUS_EN_SHIFT)
typedef enum {
    DCACHE_MIXED_MODE_ID        = 0 << DCACHE_MIXED_MODE_SHIFT, /* both use ICache cache Instruction, use DCache cache Data */
    DCACHE_MIXED_MODE_I         = 1 << DCACHE_MIXED_MODE_SHIFT, /* only use ICache cache Instruction */
    DCACHE_MIXED_MODE_D         = 3 << DCACHE_MIXED_MODE_SHIFT, /* only use DCache cache Instruction and Data */
} DCACHE_MIXED_MODE;

#define DCACHE_ASSOCIATE_MODE_DIRECT    0
#define DCACHE_ASSOCIATE_MODE_TWO_WAY   1
#define DCACHE_ASSOCIATE_MODE_FOUR_WAY  2

#ifdef __CONFIG_PSRAM
extern char *__PSRAM_Base[];
extern uint8_t __PSRAM_BASE[];
extern uint8_t __PSRAM_LENGTH[];
#define IDCACHE_START_ADDR      ((uint32_t)(__PSRAM_Base))
#define IDCACHE_END_ADDR        ((uint32_t)(0x1400000) + (uint32_t)__PSRAM_LENGTH)
#else
#define IDCACHE_START_ADDR      ((uint32_t)(0x1400000))
#endif

typedef struct {
	uint8_t way_mode;
	uint8_t vc_en;
	uint8_t wrap_en;
	uint32_t mixed_mode;
} DCache_Config;

void HAL_Dcache_FlushAll(void);
void HAL_Dcache_FlushCleanAll(void);
void HAL_Dcache_FlushClean(uint32_t sadd, uint32_t len);
void HAL_Dcache_Clean(uint32_t sadd, uint32_t len);
void HAL_Dcache_Flush(uint32_t sadd, uint32_t len);
void HAL_Dcache_SetWriteThrough(uint32_t idx, uint32_t en, uint32_t sadd, uint32_t eadd);
void HAL_Dcache_DUMP_MissHit(void);
void HAL_Dcache_Config(DCache_Config *cfg);
void HAL_Dcache_DeConfig(void);
void HAL_Dcache_Init(DCache_Config *cfg);
void HAL_Dcache_DeInit(void);

#endif /* (__CONFIG_CHIP_ARCH_VER == 2) */

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_DCACHE_H_ */
