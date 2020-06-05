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

#include "rom/sys/io.h"
//#include "rom/sys/xr_debug.h"
#include "rom/driver/chip/hal_def.h"
#include "rom/driver/chip/hal_util.h"
#include "rom/driver/chip/hal_ccm.h"
#include "rom/driver/chip/hal_dcache.h"

#include "rom/driver/chip/private/hal_debug.h"

//#define DCACHE_DBG

#ifdef DCACHE_DBG
#define DCACHE_DUMP_REGS() \
    { \
        printf("dcache base addr:[0x%08x]\n", (uint32_t)(DCACHE_CTRL)); \
        printf("DCACHE_COM_CFG:0x%08x\n", DCACHE_CTRL->DCACHE_COM_CFG);\
        printf("MISS_COUNT_H:\t0x%08x\n", DCACHE_CTRL->MISS_COUNT_H);\
        printf("MISS_COUNT_L:\t0x%08x\n", DCACHE_CTRL->MISS_COUNT_L);\
        printf("HIT_COUNT_H:\t0x%08x\n", DCACHE_CTRL->HIT_COUNT_H);\
        printf("HIT_COUNT_L:\t0x%08x\n", DCACHE_CTRL->HIT_COUNT_L);\
        printf("DCACHE_STA:\t0x%08x\n", DCACHE_CTRL->DCACHE_STA);\
        printf("CLEAN_FLUSH_SADDR:0x%08x\n", DCACHE_CTRL->CLEAN_FLUSH_SADDR);\
        printf("CLEAN_FLUSH_LEN:0x%08x\n", DCACHE_CTRL->CLEAN_FLUSH_LEN);\
    }
#else
#define DCACHE_DUMP_REGS()
#endif

void HAL_Dcache_FlushAll(void)
{
	HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG,
	               (DCACHE_FLUSH_START_MASK | DCACHE_FLUSH_CLEAN_START_MASK),
	               (DCACHE_FLUSH_START_MASK | DCACHE_FLUSH_CLEAN_START_MASK));
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_FLUSH_START_MASK)
		;
}

void HAL_Dcache_FlushCleanAll(void)
{
	HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG,
	               (DCACHE_FLUSH_START_MASK | DCACHE_CLEAN_START_MASK |
	                DCACHE_FLUSH_CLEAN_START_MASK),
	               (DCACHE_FLUSH_START_MASK | DCACHE_CLEAN_START_MASK |
	                DCACHE_FLUSH_CLEAN_START_MASK));
	while (DCACHE_CTRL->DCACHE_COM_CFG &
	       (DCACHE_FLUSH_START_MASK | DCACHE_CLEAN_START_MASK))
		;
}

void HAL_Dcache_FlushClean(uint32_t sadd, uint32_t len)
{
	HAL_ASSERT_PARAM(sadd >= IDCACHE_START_ADDR && sadd < IDCACHE_END_ADDR);
	HAL_ASSERT_PARAM(len > 0);

	DCACHE_CTRL->CLEAN_FLUSH_SADDR = sadd;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = len;

	HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG,
	               (DCACHE_FLUSH_START_MASK | DCACHE_CLEAN_START_MASK),
	               (DCACHE_FLUSH_START_MASK | DCACHE_CLEAN_START_MASK));
	while (DCACHE_CTRL->DCACHE_COM_CFG &
	       (DCACHE_FLUSH_START_MASK | DCACHE_CLEAN_START_MASK))
		;
}

void HAL_Dcache_Clean(uint32_t sadd, uint32_t len)
{
	HAL_ASSERT_PARAM(sadd >= IDCACHE_START_ADDR && sadd < IDCACHE_END_ADDR);
	HAL_ASSERT_PARAM(len > 0);

	DCACHE_CTRL->CLEAN_FLUSH_SADDR = sadd;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = len;

	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_CLEAN_START_MASK);
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_CLEAN_START_MASK)
		;
}

void HAL_Dcache_SetWriteThrough(uint32_t idx, uint32_t en, uint32_t sadd, uint32_t eadd)
{
	HAL_ASSERT_PARAM(idx < 3);
	HAL_ASSERT_PARAM(!(sadd & 0x0F));
	HAL_ASSERT_PARAM(sadd >= IDCACHE_START_ADDR && sadd < IDCACHE_END_ADDR);

	if (en) {
		HAL_ASSERT_PARAM(!(eadd & 0x0F));
		HAL_ASSERT_PARAM(eadd > IDCACHE_START_ADDR && eadd <= IDCACHE_END_ADDR);
		HAL_ASSERT_PARAM(eadd > sadd);

		HAL_Dcache_FlushClean(sadd, eadd - sadd);
		DCACHE_CTRL->WT_ADDR[idx].START_ADDR = sadd;
		DCACHE_CTRL->WT_ADDR[idx].END_ADDR = eadd;
	} else {
		DCACHE_CTRL->WT_ADDR[idx].START_ADDR = 0;
		DCACHE_CTRL->WT_ADDR[idx].END_ADDR = 0;
	}
}

void HAL_Dcache_DUMP_MissHit(void)
{
	printf("MISS_COUNT_H:\t0x%08x\n", DCACHE_CTRL->MISS_COUNT_H);
	printf("MISS_COUNT_L:\t0x%08x\n", DCACHE_CTRL->MISS_COUNT_L);
	printf("HIT_COUNT_H:\t0x%08x\n", DCACHE_CTRL->HIT_COUNT_H);
	printf("HIT_COUNT_L:\t0x%08x\n", DCACHE_CTRL->HIT_COUNT_L);
}

void HAL_Dcache_Init(DCache_Config *cfg)
{
	HAL_ASSERT_PARAM(cfg);

	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_DCACHE);
	HAL_UDelay(100);
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_ENABLE_MASK);
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_COUNTER_EN_MASK);

	if (cfg->vc_en)
		HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG,
		               DCACHE_EN_VICTIM_MASK, DCACHE_EN_VICTIM);
	else
		HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_EN_VICTIM);

	if (cfg->wrap_en)
		HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG,
		               DCACHE_EN_RD_WRAP_MASK, DCACHE_EN_RD_WRAP);
	else
		HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_EN_RD_WRAP);

	HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_ASSOCIATE_MODE_MASK,
	               (cfg->way_mode << DCACHE_ASSOCIATE_MODE_SHIFT));

	HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG,
	               DCACHE_MIXED_MODE_DCACHE | DCACHE_MIXED_IDBUS_EN,
	               cfg->mixed_mode);

	HAL_Dcache_FlushAll();

	DCACHE_DUMP_REGS();
}

void HAL_Dcache_Deinit(void)
{
	HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_COUNTER_EN_MASK);
	HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_ENABLE_MASK);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_DCACHE);
}
