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
#if (__CONFIG_CHIP_ARCH_VER == 2)
#include "sys/io.h"
#include "driver/chip/hal_def.h"
#include "driver/chip/hal_util.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_dcache.h"
#include "driver/chip/hal_xip.h"

#include "driver/chip/private/hal_debug.h"
#include "driver/chip/private/hal_os.h"
#include "pm/pm.h"

int32_t HAL_Dcache_IsBypass(uint32_t addr, uint32_t len)
{
    return ((RANGEOF_CACHEBYPASS(addr, len, DCACHE_CTRL->WT_ADDR[0].START_ADDR, DCACHE_CTRL->WT_ADDR[0].END_ADDR|0xF))
           || (RANGEOF_CACHEBYPASS(addr, len, DCACHE_CTRL->WT_ADDR[1].START_ADDR, DCACHE_CTRL->WT_ADDR[1].END_ADDR|0xF))
           || (RANGEOF_CACHEBYPASS(addr, len, DCACHE_CTRL->WT_ADDR[2].START_ADDR, DCACHE_CTRL->WT_ADDR[2].END_ADDR|0xF)));
}

int32_t HAL_Dcache_IsCacheable(uint32_t addr, uint32_t len)
{
    return ((0
#ifdef __CONFIG_PSRAM
        || (RANGEOF_CACHEBYPASS(addr, len, PSRAM_START_ADDR, PSRAM_END_ADDR))
#endif
#ifdef __CONFIG_XIP
        || (RANGEOF_CACHEBYPASS(addr, len, XIP_START_ADDR, XIP_END_ADDR))
#endif
        ) && !HAL_Dcache_IsBypass(addr, len));
}

int32_t HAL_Dcache_Enable_WriteThrough(uint32_t sadd, uint32_t eadd)
{
    int8_t idx;
    unsigned long flag;
    HAL_ASSERT_PARAM(!(sadd & 0x0F));
	HAL_ASSERT_PARAM(!(eadd & 0x0F));
	HAL_ASSERT_PARAM(eadd > sadd);
    HAL_ASSERT_PARAM(0
#ifdef __CONFIG_PSRAM
        ||((sadd >= PSRAM_START_ADDR) && (eadd <= (PSRAM_END_ADDR)))
#endif
#ifdef __CONFIG_XIP
        || ((sadd >= XIP_START_ADDR) && (eadd <= (XIP_END_ADDR)))
#endif
        );

    flag = HAL_EnterCriticalSection();
    for(idx = 0; idx<ARRAY_SIZE(DCACHE_CTRL->WT_ADDR); idx++) {
        if((DCACHE_CTRL->WT_ADDR[idx].START_ADDR == 0x0) && (DCACHE_CTRL->WT_ADDR[idx].END_ADDR == 0x0))
            break;
    }

    if(idx == ARRAY_SIZE(DCACHE_CTRL->WT_ADDR)) {
        idx = -1;
        goto out;
    }

	DCACHE_CTRL->WT_ADDR[idx].START_ADDR = sadd;
	DCACHE_CTRL->WT_ADDR[idx].END_ADDR = eadd;

out:
    HAL_ExitCriticalSection(flag);

    return (int32_t)idx;
}

int32_t HAL_Dcache_Disable_WriteThrough(int32_t idx)
{
    unsigned long flag;

	if(idx >= ARRAY_SIZE(DCACHE_CTRL->WT_ADDR))
        return -1;

    flag = HAL_EnterCriticalSection();
	DCACHE_CTRL->WT_ADDR[idx].START_ADDR = 0;
	DCACHE_CTRL->WT_ADDR[idx].END_ADDR = 0;
    HAL_ExitCriticalSection(flag);

    return 0;
}

#ifdef __CONFIG_ROM

void HAL_Dcache_Flush(uint32_t sadd, uint32_t len)
{
	HAL_ASSERT_PARAM(len > 0);

    unsigned long flag;
    flag = HAL_EnterCriticalSection();

    HAL_Dcache_WaitIdle();
	DCACHE_CTRL->CLEAN_FLUSH_SADDR = sadd;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = len;
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_FLUSH_START_MASK);
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_FLUSH_START_MASK)
		;

    DCACHE_CTRL->CLEAN_FLUSH_SADDR = 0;
    DCACHE_CTRL->CLEAN_FLUSH_LEN = 0;
    HAL_ExitCriticalSection(flag);
}

void HAL_Dcache_FlushAll(void)
{
    unsigned long flag;
    flag = HAL_EnterCriticalSection();

    HAL_Dcache_WaitIdle();
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, (DCACHE_FLUSH_START_MASK | DCACHE_FLUSH_CLEAN_START_MASK));
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_FLUSH_START_MASK)
		;
    HAL_CLR_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_FLUSH_CLEAN_START_MASK);
    HAL_ExitCriticalSection(flag);
}

#ifdef __CONFIG_PSRAM
void HAL_Dcache_CleanAll(void)
{
    HAL_Dcache_Clean(PSRAM_START_ADDR, PSRAM_LENGTH);
}

void HAL_Dcache_Clean(uint32_t sadd, uint32_t len)
{
	HAL_ASSERT_PARAM((sadd >= DCACHE_START_ADDR) && (len > 0) && ((sadd+len-1) <= DCACHE_END_ADDR));

    unsigned long flag;
    flag = HAL_EnterCriticalSection();

    HAL_Dcache_WaitIdle();
	DCACHE_CTRL->CLEAN_FLUSH_SADDR = sadd;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = len;
	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_CLEAN_START_MASK);
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_CLEAN_START_MASK)
		;
	DCACHE_CTRL->CLEAN_FLUSH_SADDR = 0;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = 0;
    HAL_ExitCriticalSection(flag);
}
#endif

#ifdef CONFIG_PM
static DCache_Config _dcache_cfg;
static uint32_t _dcache_addr[ARRAY_SIZE(DCACHE_CTRL->WT_ADDR) * 2];

static int dcache_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
#ifdef __CONFIG_PSRAM
		HAL_Dcache_CleanAll();
		HAL_Dcache_FlushAll();
#endif
		for (int i = 0; i < ARRAY_SIZE(DCACHE_CTRL->WT_ADDR); i++) {
			_dcache_addr[i * 2] = DCACHE_CTRL->WT_ADDR[i].START_ADDR;
			_dcache_addr[i * 2 + 1] = DCACHE_CTRL->WT_ADDR[i].END_ADDR;
		}
		HAL_Dcache_DeConfig();
		break;
	default:
		break;
	}
	return 0;
}

static int dcache_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_Dcache_Config(&_dcache_cfg);
		for (int i = 0; i < ARRAY_SIZE(DCACHE_CTRL->WT_ADDR); i++) {
			DCACHE_CTRL->WT_ADDR[i].START_ADDR = _dcache_addr[i * 2];
			DCACHE_CTRL->WT_ADDR[i].END_ADDR = _dcache_addr[i * 2 + 1];
		}
		break;
	default:
		break;
	}
	return 0;
}

static const struct soc_device_driver dcache_drv = {
	.name = "dcache",
	.suspend_noirq = dcache_suspend,
	.resume_noirq = dcache_resume,
};

static struct soc_device dcache_dev = {
	.name = "dcache",
	.driver = &dcache_drv,
};

#define DCACHE_DEV (&dcache_dev)

#endif/* CONFIG_PM */

void HAL_Dcache_Init(DCache_Config *cfg)
{
	HAL_ASSERT_PARAM(cfg);

	HAL_Dcache_Config(cfg);

#ifdef CONFIG_PM
	HAL_Memcpy(&_dcache_cfg, cfg, sizeof(DCache_Config));
	pm_register_ops(DCACHE_DEV);
#endif
}

void HAL_Dcache_DeInit(void)
{
#ifdef CONFIG_PM
	pm_unregister_ops(DCACHE_DEV);
	HAL_Memset(&_dcache_cfg, 0, sizeof(DCache_Config));
#endif
	HAL_Dcache_DeConfig();
}
#endif/* (__CONFIG_ROM) */

#endif/* (__CONFIG_CHIP_ARCH_VER == 2) */
