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

#include "hal_base.h"
#include "rom/driver/chip/hal_icache.h"

#include "sys/xr_debug.h"

#if 1
#define FCC_DEBUG(msg, arg...) XR_DEBUG((DBG_OFF | XR_LEVEL_ALL), NOEXPAND, "[ICACHE] " msg, ##arg)
#define FCC_ERROR(msg, arg...) XR_DEBUG((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[ICACHE] " msg, ##arg)
#else
#define FCC_DEBUG(msg, arg...) XR_DEBUG((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[ICACHE] " msg, ##arg)
#define FCC_ERROR(msg, arg...) XR_DEBUG((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[ICACHE] " msg, ##arg)
#endif

#if defined(__CONFIG_CHIP_XR871)

#define FCC_REG_ALL(cfg) { \
		FCC_DEBUG("flash cache reg:\n"); \
		FCC_DEBUG("COM_CTRL" ":0x%x.\n", FLASH_CACHE->COM_CTRL); \
		FCC_DEBUG("PREFETCH_CTRL" ":0x%x.\n", FLASH_CACHE->PREFETCH_CTRL); \
		FCC_DEBUG("PREFETCH_START_ADDR" ":0x%x.\n", FLASH_CACHE->PREFETCH_START_ADDR); \
		FCC_DEBUG("IV_PREFETCH_CTRL" ":0x%x.\n", FLASH_CACHE->IV_PREFETCH_CTRL); \
		FCC_DEBUG("MIN_ADDR" ":0x%x.\n", FLASH_CACHE->MIN_ADDR); \
		FCC_DEBUG("MAX_ADDR" ":0x%x.\n", FLASH_CACHE->MAX_ADDR); \
		FCC_DEBUG("READ_BIAS_ADDR" ":0x%x.\n", FLASH_CACHE->READ_BIAS_ADDR); \
	}

HAL_Status HAL_ICache_EnablePrefetch(ICache_PrefetchConfig *cfg)
{
	/*	 uint32_t cache_line = (cfg->prefetch_cache_size * 8 + 127) / 128;
		if (cache_line >= CACHE_LINE_MAX)
			return HAL_INVALID;
	*/

	HAL_MODIFY_REG(FLASH_CACHE->PREFETCH_START_ADDR,
			FLASH_CACHE_PREFETCH_START_ADDR_MASK,
			cfg->addr_prefetch_start << FLASH_CACHE_PREFETCH_START_ADDR_SHIFT);
	HAL_MODIFY_REG(FLASH_CACHE->PREFETCH_CTRL,
 //			FLASH_CACHE_PREFETCH_CTRL_SIZE_MASK |
			FLASH_CACHE_PREFETCH_CTRL_BRANCH_MASK |
			FLASH_CACHE_PREFETCH_CTRL_STOP_MASK,
 //			(cache_line << FLASH_CACHE_PREFETCH_CTRL_SIZE_SHIFT) |
			(cfg->prefetch_2nd_branch << FLASH_CACHE_PREFETCH_CTRL_BRANCH_SHIFT) |
			(FLASH_CACHE_PREFETCH_ENABLE << FLASH_CACHE_PREFETCH_CTRL_STOP_SHIFT));

	return HAL_OK;
}

HAL_Status HAL_ICache_DisablePrefetch(void)
{
	HAL_MODIFY_REG(FLASH_CACHE->PREFETCH_CTRL,
			FLASH_CACHE_PREFETCH_CTRL_STOP_MASK,
			(FLASH_CACHE_PREFETCH_DISABLE << FLASH_CACHE_PREFETCH_CTRL_STOP_SHIFT));

	return HAL_OK;
}

/*
HAL_Status HAL_ICache_PrefetchInt(FlashCache_PrefetchIntConfig * cfg)
{
	uint32_t cnt = 0;



	HAL_MODIFY_REG(FLASH_CACHE->IV_PREFETCH_CTRL,
			FLASH_CACHE_IV_PREFETCH_CTRL_IV_NUM_MASK,
			cnt << FLASH_CACHE_IV_PREFETCH_CTRL_IV_NUM_SHIFT);

	if (cnt != 0)
		HAL_SET_BIT(FLASH_CACHE->IV_PREFETCH_CTRL,
			    FLASH_CACHE_IV_PREFETCH_CTRL_RELEASE_MASK |
			    FLASH_CACHE_IV_PREFETCH_CTRL_ENABLE_MASK);
	else
		HAL_CLR_BIT(FLASH_CACHE->IV_PREFETCH_CTRL,
			    FLASH_CACHE_IV_PREFETCH_CTRL_RELEASE_MASK |
			    FLASH_CACHE_IV_PREFETCH_CTRL_ENABLE_MASK);
}
*/

HAL_Status HAL_ICache_Init(ICache_Config *cfg)
{
//	uint32_t cache_line = (cfg->cache_size * 8 + 127) / 128;
//	uint32_t cache_line = (CACHE_SIZE * 8 + 127) / 128;

	/* check param */
	if (/*(cache_line > 0x1FF) || */(cfg->addr_bias & (~FLASH_CACHE_READ_BIAS_ADDR_MASK))) {
		return HAL_INVALID;
	}

	/* CCMU Enable */

 //	HAL_MODIFY_REG(FLASH_CACHE, FLASH_CACHE_COM_CTRL_SIZE_MASK, cache_line << FLASH_CACHE_COM_CTRL_SIZE_SHIFT);
	HAL_MODIFY_REG(FLASH_CACHE->MIN_ADDR,
			FLASH_CACHE_MIN_ADDR_MASK,
			FLASH_ICACHE_START_ADDR << FLASH_CACHE_MIN_ADDR_SHIFT);
	HAL_MODIFY_REG(FLASH_CACHE->MAX_ADDR,
			FLASH_CACHE_MAX_ADDR_MASK,
			FLASH_ICACHE_END_ADDR << FLASH_CACHE_MAX_ADDR_SHIFT);
	HAL_MODIFY_REG(FLASH_CACHE->READ_BIAS_ADDR,
			FLASH_CACHE_READ_BIAS_ADDR_MASK,
			cfg->addr_bias << FLASH_CACHE_READ_BIAS_ADDR_SHIFT);

	return HAL_OK;
}

HAL_Status HAL_ICache_Deinit(void)
{
	return HAL_OK;
}

#elif defined(__CONFIG_CHIP_XR875)

#define ICACHE_REG_ALL() \
     { \
	 FCC_DEBUG("icache reg: base addr   0x%8x\n.", (uint32_t)&(ICACHE_CTRL->CACHE_COM_CFG)); \
	 FCC_DEBUG("CACHE_COM_CFG;	    0x%8x\n.", ICACHE_CTRL->CACHE_COM_CFG);\
	 FCC_DEBUG("MISS_COUNT_H;	    0x%8x\n.", ICACHE_CTRL->MISS_COUNT_H);\
	 FCC_DEBUG("MISS_COUNT_L;	    0x%8x\n.", ICACHE_CTRL->MISS_COUNT_L);\
	 FCC_DEBUG("HIT_COUNT_H;	    0x%8x\n.", ICACHE_CTRL->HIT_COUNT_H);\
	 FCC_DEBUG("HIT_COUNT_L;	    0x%8x\n.", ICACHE_CTRL->HIT_COUNT_L);\
	 FCC_DEBUG("CACHE_STA;		    0x%8x\n.", ICACHE_CTRL->CACHE_STA);\
	 FCC_DEBUG("INSTR_WAIT_H;	    0x%8x\n.", ICACHE_CTRL->INSTR_WAIT_H);\
	 FCC_DEBUG("INSTR_WAIT_L;	    0x%8x\n.", ICACHE_CTRL->INSTR_WAIT_L);\
     }
static inline void Icache_Reg_All()
{
	//ICACHE_REG_ALL();
}

HAL_Status HAL_ICache_EnablePrefetch(ICache_PrefetchConfig *cfg)
{

	return HAL_OK;
}

HAL_Status HAL_ICache_DisablePrefetch(void)
{

	return HAL_OK;
}

void HAL_ICache_Flush(void)
{
	HAL_SET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_FLUSH_ALL_MASK);
	while (HAL_GET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_FLUSH_ALL_MASK))
		;
}

HAL_Status HAL_ICache_Init(ICache_Config *cfg)
{
	FCC_DEBUG("%s: vc:%d wrap:%d mode:%d bypass:%d\n",
		  __func__, cfg->vc_en, cfg->wrap_en, cfg->way_mode, cfg->bypass);

	/* CCMU Enable */
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_ICACHE);
	HAL_UDelay(100);

	/* cache hm count enable */
	HAL_SET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_COUNTER_EN_MASK);

	HAL_ICache_Flush();

	if (cfg->vc_en) { /* cache victim enable */
		HAL_SET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_EN_VICTIM_CACHE_MASK);
	} else { /* cache victim disable */
		HAL_CLR_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_EN_VICTIM_CACHE_MASK);
	}

	if (cfg->wrap_en) { /* cache wrap enable */
		HAL_SET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_EN_ICACHE_WRAP_MASK);
	} else { /* cache wrap disable */
		HAL_CLR_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_EN_ICACHE_WRAP_MASK);
	}

	/* cache set asso mode */
	HAL_MODIFY_REG(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_ASSOCIATE_MODE_MASK,
	               (cfg->way_mode << ICACHE_ASSOCIATE_MODE_SHIFT));

	/* cache enable */
	HAL_SET_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_ENABLE_MASK);

	HAL_ICache_Flush();
	Icache_Reg_All();

	return HAL_OK;
}

HAL_Status HAL_ICache_Deinit(void)
{
	/* cache hm count disable */
	HAL_CLR_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_COUNTER_EN_MASK);

	/* cache disable */
	HAL_CLR_BIT(ICACHE_CTRL->CACHE_COM_CFG, ICACHE_ENABLE_MASK);

	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_ICACHE);

	return HAL_OK;
}

#endif /* __CONFIG_CHIP_XXX */
