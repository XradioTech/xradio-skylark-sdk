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
#include "driver/chip/hal_prcm.h"

#include "hal_base.h"


#if (__CONFIG_CHIP_ARCH_VER == 2)

uint32_t HAL_PRCM_GetSysPowerEnableFlags(void)
{
	return HAL_GET_BIT(PRCM->SYS_LDO_SW_CTRL,
	                   PRCM_SW4_EN_BIT | PRCM_SW5_EN_BIT | PRCM_SR_SW3_EN_BIT);
}

uint8_t HAL_PRCM_GetSW1Status(void)
{
	return !!HAL_SET_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_SW1_STATUS_BIT);
}

uint8_t HAL_PRCM_GetLDO1Status()
{
	return !!HAL_GET_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_STATUS_BIT);
}

void HAL_PRCM_SetLDO1Voltage(PRCM_LDO1Volt volt)
{
	HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_VOLT_MASK, volt);
}

#ifdef __CONFIG_ROM

void HAL_PRCM_SetLDO1Volt(PRCM_LDO1Volt work_volt, PRCM_LDO1RetVolt ret_volt)
{
    HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_VOLT_MASK | PRCM_LDO1_RET_VOLT_MASK, work_volt | ret_volt);
}

/*
 *fix that inter32kfreq regs is 0 before 24M clock complete checking the inter32k clock.
 *32768 is recommended value.
 */
uint32_t HAL_PRCM_GetLFClock(void)
{
	uint32_t val = HAL_GET_BIT(PRCM->SYS_LFCLK_CTRL, PRCM_LFCLK_SRC_MASK);

	if (val == PRCM_LFCLK_SRC_INTER32K &&
	    HAL_GET_BIT(PRCM->SYS_RCOSC_CALIB_CTRL, PRCM_RCOSC_CALIB_EN_BIT)) {
	    if (!HAL_PRCM_GetInter32KFreq()) {
			return SYS_LFCLOCK;
	    } else {
			return HAL_PRCM_GetInter32KFreq();
		}
	} else {
		return SYS_LFCLOCK;
	}
}

uint32_t HAL_PRCM_GetDev2Clock(void)
{
	uint32_t divm, divn;

	divm = HAL_GET_BIT_VAL(PRCM->DEV_CLK_CTRL,
	                       PRCM_DEV2_FACTOR_M_SHIFT,
	                       PRCM_DEV2_FACTOR_M_VMASK) + 5;
	divn = HAL_GET_BIT_VAL(PRCM->DEV_CLK_CTRL,
	                       PRCM_DEV2_FACTOR_N_SHIFT,
	                       PRCM_DEV2_FACTOR_N_VMASK) + 1;
	return (SYS_PLL_CLOCK / divm / divn);
}

void HAL_PRCM_EnableLDOModeSWSelEnable(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_LDO_MODE_SW_SEL_EN_BIT);
	else
		HAL_CLR_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_LDO_MODE_SW_SEL_EN_BIT);
}

void HAL_PRCM_SetDigSWRefTime(uint32_t val)
{
    HAL_MODIFY_REG(PRCM->DIG_SWITCH_REF_TIME, PRCM_RESET_UP_REF_TIME_MASK, val << PRCM_RESET_UP_REF_TIME_SHIFT);
}

void HAL_PRCM_EnableSysLDOLQModeEnable(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_SYS_LDO_LQ_MODE_BIT);
	else
		HAL_CLR_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_SYS_LDO_LQ_MODE_BIT);
}

void HAL_PRCM_EnableTOPLDOLQModeEnable(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_TOP_LDO_LQ_MODE_BIT);
	else
		HAL_CLR_BIT(PRCM->LDO_MODE_SW_SEL, PRCM_TOP_LDO_LQ_MODE_BIT);
}

void HAL_PRCM_SetSys1SleepPowerFlags(uint32_t flags)
{
    HAL_MODIFY_REG(PRCM->SYS1_SLEEP_CTRL, PRCM_SYS_WS_PWR_FLAGS_MASK, flags & PRCM_SYS_WS_PWR_FLAGS_MASK);
}

#endif /*__CONFIG_ROM */

#endif /*__CONFIG_CHIP_ARCH_VER */
