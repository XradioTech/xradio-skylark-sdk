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

#include "driver/chip/private/hal_debug.h"
#include "driver/chip/private/hal_os.h"
#include "pm/pm.h"

#ifdef __CONFIG_ROM
#ifdef __CONFIG_PSRAM
void HAL_Dcache_SetWriteThrough(uint32_t idx, uint32_t en, uint32_t sadd, uint32_t eadd)
{
	HAL_ASSERT_PARAM(idx < 3);
	if (en) {
        HAL_ASSERT_PARAM(!(sadd & 0x0F));
		HAL_ASSERT_PARAM(!(eadd & 0x0F));
		HAL_ASSERT_PARAM(eadd > sadd);
        HAL_ASSERT_PARAM((sadd >= IDCACHE_START_ADDR) && (eadd <= (IDCACHE_END_ADDR)));
		HAL_Dcache_FlushClean(sadd, eadd - sadd);
		DCACHE_CTRL->WT_ADDR[idx].START_ADDR = sadd;
		DCACHE_CTRL->WT_ADDR[idx].END_ADDR = eadd;
	} else {
		DCACHE_CTRL->WT_ADDR[idx].START_ADDR = 0;
		DCACHE_CTRL->WT_ADDR[idx].END_ADDR = 0;
	}
}

void HAL_Dcache_FlushClean(uint32_t sadd, uint32_t len)
{
	HAL_ASSERT_PARAM((sadd >= IDCACHE_START_ADDR) && (len > 0) && ((sadd-len) < IDCACHE_END_ADDR));

	DCACHE_CTRL->CLEAN_FLUSH_SADDR = sadd;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = len;

    unsigned long flag;
    flag = HAL_EnterCriticalSection();

	HAL_MODIFY_REG(DCACHE_CTRL->DCACHE_COM_CFG,
	               (DCACHE_FLUSH_START_MASK | DCACHE_CLEAN_START_MASK),
	               (DCACHE_FLUSH_START_MASK | DCACHE_CLEAN_START_MASK));
	while (DCACHE_CTRL->DCACHE_COM_CFG &
	       (DCACHE_FLUSH_START_MASK | DCACHE_CLEAN_START_MASK))
		;
    HAL_ExitCriticalSection(flag);
}

void HAL_Dcache_Clean(uint32_t sadd, uint32_t len)
{
	HAL_ASSERT_PARAM((sadd >= IDCACHE_START_ADDR) && (len > 0) && ((sadd-len) < IDCACHE_END_ADDR));

	DCACHE_CTRL->CLEAN_FLUSH_SADDR = sadd;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = len;

    unsigned long flag;
    flag = HAL_EnterCriticalSection();

	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_CLEAN_START_MASK);
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_CLEAN_START_MASK)
		;

    HAL_ExitCriticalSection(flag);
}

#endif

void HAL_Dcache_Flush(uint32_t sadd, uint32_t len)
{
	HAL_ASSERT_PARAM(len > 0);

	DCACHE_CTRL->CLEAN_FLUSH_SADDR = sadd;
	DCACHE_CTRL->CLEAN_FLUSH_LEN = len;

    unsigned long flag;
    flag = HAL_EnterCriticalSection();

	HAL_SET_BIT(DCACHE_CTRL->DCACHE_COM_CFG, DCACHE_FLUSH_START_MASK);
	while (DCACHE_CTRL->DCACHE_COM_CFG & DCACHE_FLUSH_START_MASK)
		;

    HAL_ExitCriticalSection(flag);
}

#ifdef CONFIG_PM
static DCache_Config _dcache_cfg;
static uint32_t _dcache_addr[DCACHE_ADDR_MAX * 2];

static int dcache_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_Dcache_FlushCleanAll();
		for (int i = 0; i < DCACHE_ADDR_MAX; i++) {
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
		for (int i = 0; i < DCACHE_ADDR_MAX; i++) {
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
