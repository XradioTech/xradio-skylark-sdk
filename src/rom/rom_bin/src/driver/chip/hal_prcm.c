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

#include "rom/driver/chip/hal_prcm.h"

#include "hal_base.h"

/*
 * Power
 *   - DCDC
 *   - Power switch
 *   - LDO
 */
#ifdef __CONFIG_CHIP_XR871
void HAL_PRCM_SetDCDCVoltage(PRCM_DCDCVolt volt)
{
	HAL_MODIFY_REG(PRCM->SYS_DCDC_CTRL, PRCM_DCDC_VOLT_MASK, volt);
}

uint32_t HAL_PRCM_GetSysPowerEnableFlags(void)
{
	return HAL_GET_BIT(PRCM->SYS_LDO_SW_CTRL,
					   PRCM_SYS1_PWR1_EN_BIT |
					   PRCM_SYS1_PWR2_EN_BIT |
					   PRCM_SYS2_PWR3_EN_BIT |
					   PRCM_SYS3_PWR4_EN_BIT |
					   PRCM_SYS3_PWR5_EN_BIT |
	  				   PRCM_SYS1_SRAM_PWR1_EN_BIT |
	  				   PRCM_SYS2_SRAM_PWR2_EN_BIT |
	  				   PRCM_SYS3_SRAM_PWR3_EN_BIT);
}

__rom_xip_text
void HAL_PRCM_EnableSys2SysPower(void)
{
	HAL_SET_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_SYS2_PWR3_EN_BIT);
}

__rom_xip_text
void HAL_PRCM_DisableSys2SysPower(void)
{
	HAL_CLR_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_SYS2_PWR3_EN_BIT);
}

__rom_xip_text
void HAL_PRCM_EnableSys2Power(void)
{
	HAL_SET_BIT(PRCM->SYS_LDO_SW_CTRL,
		        PRCM_SYS2_PWR3_EN_BIT | PRCM_SYS2_SRAM_PWR2_EN_BIT);
}

__rom_xip_text
void HAL_PRCM_DisableSys2Power(void)
{
	HAL_CLR_BIT(PRCM->SYS_LDO_SW_CTRL,
		        PRCM_SYS2_PWR3_EN_BIT | PRCM_SYS2_SRAM_PWR2_EN_BIT);
}

void HAL_PRCM_SetLDO1Voltage(PRCM_LDO1Volt volt)
{
	HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_VOLT_MASK, volt);
}

uint32_t HAL_PRCM_GetLDOEnableFlags(void)
{
	return HAL_GET_BIT(PRCM->SYS_LDO_SW_CTRL,
					   PRCM_LDO1_EN_BIT |
					   PRCM_PLL_LDO_EN_BIT |
					   PRCM_SRAM_LDO_EN_BIT);
}
#elif (defined __CONFIG_CHIP_XR875)
void HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDOVolt volt)
{
	HAL_MODIFY_REG(PRCM->SYS_TOP_LDO_CTRL, PRCM_TOPLDO_VOLT_MASK, volt);
}

void HAL_PRCM_SetTOPLDOForceActive(uint8_t active)
{
    if (active)
        HAL_SET_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_TOPLDO_FORCE_ACTIVE_MASK);
    else
        HAL_CLR_BIT(PRCM->SYS_TOP_LDO_CTRL, PRCM_TOPLDO_FORCE_ACTIVE_MASK);
}

void  HAL_PRCM_SetLDO1RETVolt(PRCM_LDO1RetVolt volt)
{
	HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_RET_VOLT_MASK, volt);
}

void HAL_PRCM_SelectEXTLDOVolt(PRCM_EXTLDOVolt volt)
{
	if (volt)
		HAL_SET_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_EXT_LDO_VOLT_BIT);
	else
		HAL_CLR_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_EXT_LDO_VOLT_BIT);
}

void HAL_PRCM_SetEXTLDOMdoe(PRCM_ExtLDOMode mode)
{
	HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_EXT_LDO_MODE_MASK, mode);
}

void HAL_PRCM_SetPadClkOut(uint8_t enable)
{
    if (enable)
        HAL_SET_BIT(PRCM->SYS_LFCLK_CTRL, PRCM_PAD_CLK_OUT_EN_BIT);
    else
        HAL_CLR_BIT(PRCM->SYS_LFCLK_CTRL, PRCM_PAD_CLK_OUT_EN_BIT);
}

void HAL_PRCM_SetPadClkOutSource(PRCM_PadClkOutSource source)
{
    HAL_MODIFY_REG(PRCM->SYS_LFCLK_CTRL, PRCM_PAD_CLK_OUT_SOURCE_MASK, source);
}

void HAL_PRCM_SetPadClkOutFactorM(uint16_t value)
{
    HAL_MODIFY_REG(PRCM->SYS_LFCLK_CTRL, PRCM_PAK_CLK_OUT_FACTOR_M_MASK,
                   PRCM_PAK_CLK_OUT_FACTOR_M_VAL(value));
}

#endif

/*
 * Clock
 */
void HAL_PRCM_SetLFCLKSource(PRCM_LFCLKSrc src)
{
	/* always enable inter 32K for external 32K is not ready at startup */
	uint32_t clr_mask = PRCM_LFCLK_SRC_MASK | PRCM_LFCLK_EXT32K_EN_BIT;
	uint32_t set_mask = src | PRCM_LFCLK_INTER32K_EN_BIT;
	if (src == PRCM_LFCLK_SRC_EXT32K) {
		set_mask |= PRCM_LFCLK_EXT32K_EN_BIT;
	}
	HAL_MODIFY_REG(PRCM->SYS_LFCLK_CTRL, clr_mask, set_mask);
}

void HAL_PRCM_SetHOSCType(PRCM_HOSCType type)
{
	HAL_MODIFY_REG(PRCM->SYS_HOSC_CTRL, PRCM_HOSC_TYPE_MASK, type);
}

uint32_t HAL_PRCM_GetHOSCType(void)
{
	return HAL_GET_BIT(PRCM->SYS_HOSC_CTRL, PRCM_HOSC_TYPE_MASK);
}

uint32_t rom_HAL_PRCM_GetHFClock(void)
{
	static const uint32_t PRCM_HOSCClock[] =
		{ HOSC_CLOCK_26M, HOSC_CLOCK_40M, HOSC_CLOCK_24M, HOSC_CLOCK_52M };

	uint32_t val;

	val = HAL_GET_BIT_VAL(PRCM->SYS_HOSC_CTRL,
	                      PRCM_HOSC_TYPE_SHIFT,
	                      PRCM_HOSC_TYPE_VMASK);
	return PRCM_HOSCClock[val];
}

uint32_t HAL_PRCM_GetInter32KFreq(void)
{
	return (10 * HAL_GET_BIT_VAL(PRCM->SYS_RCOSC_CALIB_CTRL,
		                         PRCM_RCOSC_CALIB_FREQ_SHIFT,
		                         PRCM_RCOSC_CALIB_FREQ_VMASK));
}

uint32_t HAL_PRCM_EnableInter32KCalib(void)
{
	return HAL_SET_BIT(PRCM->SYS_RCOSC_CALIB_CTRL, PRCM_RCOSC_CALIB_EN_BIT);
}

uint32_t HAL_PRCM_DisableInter32KCalib(void)
{
	return HAL_CLR_BIT(PRCM->SYS_RCOSC_CALIB_CTRL, PRCM_RCOSC_CALIB_EN_BIT);
}

int HAL_PRCM_IsInter32KCalibEnabled(void)
{
	return HAL_GET_BIT(PRCM->SYS_RCOSC_CALIB_CTRL, PRCM_RCOSC_CALIB_EN_BIT);
}

uint32_t rom_HAL_PRCM_GetLFClock(void)
{
	uint32_t val = HAL_GET_BIT(PRCM->SYS_LFCLK_CTRL, PRCM_LFCLK_SRC_MASK);

	if (val == PRCM_LFCLK_SRC_INTER32K &&
	    HAL_GET_BIT(PRCM->SYS_RCOSC_CALIB_CTRL, PRCM_RCOSC_CALIB_EN_BIT)) {
		return HAL_PRCM_GetInter32KFreq();
	} else {
		return SYS_LFCLOCK;
	}
}

#if 0
void HAL_PRCM_SetSysPLLParam(PRCM_SysPLLParam param)
{
	HAL_MODIFY_REG(PRCM->SYS_PLL_CTRL, PRCM_SYS_PLL_PARAM_MASK, param);
}

void HAL_PRCM_EnableSysPLL(void)
{
	HAL_SET_BIT(PRCM->SYS_PLL_CTRL, PRCM_SYS_PLL_EN_BIT);
}
#else
void HAL_PRCM_SetSysPLL(PRCM_SysPLLParam param)
{
	PRCM->SYS_PLL_CTRL = PRCM_SYS_PLL_EN_BIT | param; /* NB: enable system PLL */
}
#endif

void HAL_PRCM_DisableSysPLL(void)
{
	HAL_CLR_BIT(PRCM->SYS_PLL_CTRL, PRCM_SYS_PLL_EN_BIT);
}

#if defined(__CONFIG_CHIP_XR871)

static void HAL_PRCM_SetCPUClk(__IO uint32_t *sysClkCtrlReg,
                               PRCM_CPUClkSrc src, PRCM_SysClkFactor factor)
{
	switch (src) {
	case PRCM_CPU_CLK_SRC_HFCLK:
	case PRCM_CPU_CLK_SRC_LFCLK:
		HAL_MODIFY_REG(*sysClkCtrlReg, PRCM_CPU_CLK_SRC_MASK, src);
		HAL_MODIFY_REG(*sysClkCtrlReg,
		               PRCM_SYS_CLK_FACTOR_MASK | PRCM_SYS_CLK_EN_BIT,
		               PRCM_SYS_CLK_FACTOR_80M); /* disable system clock */
		break;
	case PRCM_CPU_CLK_SRC_SYSCLK:
	default:
		HAL_MODIFY_REG(*sysClkCtrlReg, PRCM_SYS_CLK_FACTOR_MASK, factor);
		HAL_SET_BIT(*sysClkCtrlReg, PRCM_SYS_CLK_EN_BIT);
		HAL_MODIFY_REG(*sysClkCtrlReg, PRCM_CPU_CLK_SRC_MASK, src);
		break;
	}
}

void HAL_PRCM_SetSys2SramClk(PRCM_CPUClkSrc src, PRCM_SysClkFactor factor)
{
	HAL_PRCM_SetCPUClk(&PRCMN->SYS_CLK2_CTRL, src, factor);
}

void HAL_PRCM_SetCPUAClk(PRCM_CPUClkSrc src, PRCM_SysClkFactor factor)
{
	HAL_PRCM_SetCPUClk(&PRCM->SYS_CLK1_CTRL, src, factor);
}

#elif defined(__CONFIG_CHIP_XR875)

void HAL_PRCM_SetROMCycleMode(uint8_t mode)
{
    if (mode)
        HAL_SET_BIT(PRCM->SYS_CLK1_CTRL, PRCM_ROM_READ_CYCLE_MODE_BIT);
    else
        HAL_CLR_BIT(PRCM->SYS_CLK1_CTRL, PRCM_ROM_READ_CYCLE_MODE_BIT);
}

void HAL_PRCM_SetCPUAClk(PRCM_CPUClkSrc src, PRCM_SysClkFactor factor)
{
	/* TODO: change factor m and n seperately, add DSB, ISB */
	switch (src) {
	case PRCM_CPU_CLK_SRC_HFCLK:
	case PRCM_CPU_CLK_SRC_LFCLK:
		HAL_MODIFY_REG(PRCM->SYS_CLK1_CTRL, PRCM_CPU_CLK_SRC_MASK, src);
		HAL_MODIFY_REG(PRCM->SYS_CLK1_CTRL,
		               PRCM_SYS_CLK_FACTOR_MASK, /* PRCM_SYS_CLK_EN_BIT removed */
		               PRCM_SYS_CLK_FACTOR_80M); /* disable system clock */
		break;
	case PRCM_CPU_CLK_SRC_SYSCLK:
	default:
		HAL_MODIFY_REG(PRCM->SYS_CLK1_CTRL, PRCM_SYS_CLK_FACTOR_MASK, factor);
		/* HAL_SET_BIT(PRCM->SYS_CLK1_CTRL, PRCM_SYS_CLK_EN_BIT); */ /* removed */
		HAL_MODIFY_REG(PRCM->SYS_CLK1_CTRL, PRCM_CPU_CLK_SRC_MASK, src);
		break;
	}
}

#endif /* __CONFIG_CHIP_XXX */

uint32_t HAL_PRCM_GetCPUAClk(void)
{
	uint32_t reg = PRCM->SYS_CLK1_CTRL;
	uint32_t freq;

	switch (reg & PRCM_CPU_CLK_SRC_MASK) {
	case PRCM_CPU_CLK_SRC_HFCLK:
		freq = HAL_GetHFClock();
		break;
	case PRCM_CPU_CLK_SRC_LFCLK:
		freq = HAL_GetLFClock();
		break;
	case PRCM_CPU_CLK_SRC_SYSCLK:
	default: {
		uint32_t div;
#if defined(__CONFIG_CHIP_XR871)
		div = HAL_GET_BIT_VAL(reg,
		                      PRCM_SYS_CLK_FACTOR_SHIFT,
		                      PRCM_SYS_CLK_FACTOR_VMASK) + 1;
#elif defined(__CONFIG_CHIP_XR875)
		uint32_t divm, divn;

		divm = HAL_GET_BIT_VAL(reg,
		                       PRCM_SYS_CLK_FACTORM_SHIFT,
		                       PRCM_SYS_CLK_FACTORM_VMASK) + 5;
		divn = HAL_GET_BIT_VAL(reg,
		                       PRCM_SYS_CLK_FACTORN_SHIFT,
		                       PRCM_SYS_CLK_FACTORN_VMASK) + 1;
		div = divm * divn;
#else
#error "chip undefined!"
#endif
		freq = SYS_PLL_CLOCK / div;
	}
		break;
	}
	return freq;
}

#if (defined(__CONFIG_CHIP_XR875_ON_XR871) || defined(__CONFIG_ARCH_NET_CORE))
void HAL_PRCM_SetCPUNClk(PRCM_CPUClkSrc src, PRCM_SysClkFactor factor)
{
	HAL_PRCM_SetCPUClk(&PRCMN->SYS_CLK2_CTRL, src, factor);
}

uint32_t HAL_PRCM_GetCPUNClk(void)
{
	uint32_t reg = PRCM->SYS_CLK2_CTRL;
	uint32_t freq, div;

	switch (reg & PRCM_CPU_CLK_SRC_MASK) {
	case PRCM_CPU_CLK_SRC_HFCLK:
		freq = HAL_GetHFClock();
		break;
	case PRCM_CPU_CLK_SRC_LFCLK:
		freq = HAL_GetLFClock();
		break;
	case PRCM_CPU_CLK_SRC_SYSCLK:
	default:
		div = HAL_GET_BIT_VAL(reg,
		                      PRCM_SYS_CLK_FACTOR_SHIFT,
		                      PRCM_SYS_CLK_FACTOR_VMASK) + 1;
		freq = SYS_PLL_CLOCK / div;
		break;
	}
	return freq;
}

void HAL_PRCM_EnableCPUWClk(uint32_t enable)
{
	if (enable) {
		HAL_SET_BIT(PRCMN->SYS_CLK3_CTRL, PRCM_SYS_CLK_EN_BIT);
	} else {
		HAL_CLR_BIT(PRCMN->SYS_CLK3_CTRL, PRCM_SYS_CLK_EN_BIT);
	}
}
#elif defined(__CONFIG_CHIP_XR875)
void HAL_PRCM_EnableCPUWClk(uint32_t enable)
{
	if (enable) {
		HAL_SET_BIT(PRCM->SYS_CLK3_CTRL, PRCM_SYS_CLK_EN_BIT);
	} else {
		HAL_CLR_BIT(PRCM->SYS_CLK3_CTRL, PRCM_SYS_CLK_EN_BIT);
	}
}
#endif /* __CONFIG_CHIP_XR875_ON_XR871 or __CONFIG_ARCH_NET_CORE */

__rom_xip_text
void HAL_PRCM_SetAudioPLLParam(PRCM_AudPLLParam param)
{
	PRCM->AUD_PLL_CTRL = param; /* NB: it will disable system PLL */
}

__rom_xip_text
void HAL_PRCM_EnableAudioPLL(void)
{
	HAL_SET_BIT(PRCM->AUD_PLL_CTRL, PRCM_AUD_PLL_EN_BIT);
}

__rom_xip_text
void HAL_PRCM_DisableAudioPLL(void)
{
	HAL_CLR_BIT(PRCM->AUD_PLL_CTRL, PRCM_AUD_PLL_EN_BIT);
}

#if defined(__CONFIG_CHIP_XR871)
void HAL_PRCM_SetDevClock(PRCM_DevClkFactor factor)
{
	/* TODO: change factor m and n seperately */
	PRCM->DEV_CLK_CTRL = factor;
}

uint32_t HAL_PRCM_GetDevClock(void)
{
	uint32_t div;

	div = HAL_GET_BIT_VAL(PRCM->DEV_CLK_CTRL,
	                      PRCM_DEV_CLK_FACTOR_SHIFT,
	                      PRCM_DEV_CLK_FACTOR_VMASK) + 1;

	return (SYS_PLL_CLOCK / div);
}

#elif (defined __CONFIG_CHIP_XR875)

void HAL_PRCM_SetDevClock(PRCM_DevClkFactor factor)
{
	HAL_MODIFY_REG(PRCM->DEV_CLK_CTRL, PRCM_DEV1_FACTOR_N_MASK | PRCM_DEV1_FACTOR_M_MASK, factor);
}

uint32_t rom_HAL_PRCM_GetDevClock(void)
{
	uint32_t div;
	uint32_t divm, divn;

	divm = HAL_GET_BIT_VAL(PRCM->DEV_CLK_CTRL,
	                       PRCM_DEV_CLK_FACTORM_SHIFT,
	                       PRCM_DEV_CLK_FACTORM_VMASK) + 5;
	divn = HAL_GET_BIT_VAL(PRCM->DEV_CLK_CTRL,
	                       PRCM_DEV_CLK_FACTORN_SHIFT,
	                       PRCM_DEV_CLK_FACTORN_VMASK) + 1;
	div = divm * divn;

	return (SYS_PLL_CLOCK / div);
}

void HAL_PRCM_SetDev2Clock(PRCM_Dev2ClkFactor factor)
{
	HAL_MODIFY_REG(PRCM->DEV_CLK_CTRL, PRCM_DEV2_FACTOR_N_MASK | PRCM_DEV2_FACTOR_M_MASK, factor);
}

void HAL_PRCM_EnableDev2Clock(void)
{
	HAL_SET_BIT(PRCM->DEV_CLK_CTRL, PRCM_DEV2_CLK_EN_BIT);
}

void HAL_PRCM_DisableDev2Clock(void)
{
	HAL_CLR_BIT(PRCM->DEV_CLK_CTRL, PRCM_DEV2_CLK_EN_BIT);
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
	return (SYS_PLL_CLOCK / divm * divn);
}

#endif

__rom_xip_text
void HAL_PRCM_SetAudioPLLPatternParam(PRCM_AudPLLPatParam param)
{
	PRCM->AUD_PLL_PAT_CTRL = param; /* NB: it will disable system PLL */
}

__rom_xip_text
void HAL_PRCM_EnableAudioPLLPattern(void)
{
	HAL_SET_BIT(PRCM->AUD_PLL_PAT_CTRL, PRCM_AUD_DIG_DELT_PAT_EN_BIT);
}

__rom_xip_text
void HAL_PRCM_DisableAudioPLLPattern(void)
{
	HAL_CLR_BIT(PRCM->AUD_PLL_PAT_CTRL, PRCM_AUD_DIG_DELT_PAT_EN_BIT);
}

int HAL_PRCM_IsCPUAResetRelease(void)
{
	return HAL_GET_BIT(PRCM->SYS1_CTRL, PRCM_CPUA_RESET_BIT);
}

int HAL_PRCM_IsSys1ResetRelease(void)
{
	return HAL_GET_BIT(PRCM->SYS1_CTRL, PRCM_SYS1_RESET_BIT);
}

#ifdef __CONFIG_CHIP_XR875
void HAL_PRCM_EnableHXTALOUT(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->HXTALOUT_CTRL, PRCM_HXTALOUT_EN_BIT);
	else
		HAL_CLR_BIT(PRCM->HXTALOUT_CTRL, PRCM_HXTALOUT_EN_BIT);
}
#endif

__rom_xip_text
void HAL_PRCM_AllowCPUNDeepSleep(void)
{
	HAL_CLR_BIT(PRCM->SYS1_STATUS, PRCM_CPUN_DEEPSLEEP_LOCK_BIT);
}

__rom_xip_text
void HAL_PRCM_DisallowCPUNDeepSleep(void)
{
	HAL_SET_BIT(PRCM->SYS1_STATUS, PRCM_CPUN_DEEPSLEEP_LOCK_BIT);
}

int HAL_PRCM_IsCPUNDeepSleepAllowed(void)
{
	return !HAL_GET_BIT(PRCM->SYS1_STATUS, PRCM_CPUN_DEEPSLEEP_LOCK_BIT);
}

int HAL_PRCM_IsCPUASleep(void)
{
	return HAL_GET_BIT(PRCM->SYS1_STATUS, PRCM_CPUA_SLEEP_STATUS_BIT);
}

int HAL_PRCM_IsCPUADeepSleep(void)
{
	return HAL_GET_BIT(PRCM->SYS1_STATUS, PRCM_CPUA_DEEPSLEEP_STATUS_BIT);
}

int HAL_PRCM_IsSys1Alive(void)
{
	return HAL_GET_BIT(PRCM->SYS1_STATUS, PRCM_SYS1_ALIVE_BIT);
}

__rom_xip_text
void HAL_PRCM_DisableSys2(void)
{
	HAL_CLR_BIT(PRCM->SYS2_CTRL, PRCM_SYS2_ISOLATION_EN_BIT |
	                             PRCM_CPUN_RESET_BIT |
	                             PRCM_SYS2_RESET_BIT);
}

__rom_xip_text
void HAL_PRCM_EnableSys2Isolation(void)
{
	HAL_CLR_BIT(PRCM->SYS2_CTRL, PRCM_SYS2_ISOLATION_EN_BIT);
}

__rom_xip_text
void HAL_PRCM_DisableSys2Isolation(void)
{
	HAL_SET_BIT(PRCM->SYS2_CTRL, PRCM_SYS2_ISOLATION_EN_BIT);
}

__rom_xip_text
void HAL_PRCM_ForceCPUNReset(void)
{
	HAL_CLR_BIT(PRCM->SYS2_CTRL, PRCM_CPUN_RESET_BIT);
}

__rom_xip_text
void HAL_PRCM_ReleaseCPUNReset(void)
{
	HAL_SET_BIT(PRCM->SYS2_CTRL, PRCM_CPUN_RESET_BIT);
}

int HAL_PRCM_IsCPUNReleased(void)
{
	return HAL_GET_BIT(PRCM->SYS2_CTRL, PRCM_CPUN_RESET_BIT);
}

__rom_xip_text
void HAL_PRCM_ForceSys2Reset(void)
{
	HAL_CLR_BIT(PRCM->SYS2_CTRL, PRCM_SYS2_RESET_BIT);
}

__rom_xip_text
void HAL_PRCM_ReleaseSys2Reset(void)
{
	HAL_SET_BIT(PRCM->SYS2_CTRL, PRCM_SYS2_RESET_BIT);
}

void HAL_PRCM_ForceSys3Reset(void)
{
#if defined(__CONFIG_CHIP_XR875_ON_XR871)
	HAL_CLR_BIT(PRCMN->SYS3_CTRL, PRCM_SYS3_RESET_BIT);
#elif defined(__CONFIG_CHIP_XR875)
	HAL_CLR_BIT(PRCM->SYS3_CTRL, PRCM_SYS3_RESET_BIT);
#endif
}

void HAL_PRCM_ReleaseSys3Reset(void)
{
#if defined(__CONFIG_CHIP_XR875_ON_XR871)
	HAL_SET_BIT(PRCMN->SYS3_CTRL, PRCM_SYS3_RESET_BIT);
#elif defined(__CONFIG_CHIP_XR875)
	HAL_SET_BIT(PRCM->SYS3_CTRL, PRCM_SYS3_RESET_BIT);
#endif
}

int HAL_PRCM_IsSys3Release(void)
{
#if defined(__CONFIG_CHIP_XR875_ON_XR871)
	return HAL_GET_BIT(PRCMN->SYS3_CTRL, PRCM_SYS3_RESET_BIT);
#elif defined(__CONFIG_CHIP_XR875)
	return HAL_GET_BIT(PRCM->SYS3_CTRL, PRCM_SYS3_RESET_BIT);
#endif
}

#ifdef __CONFIG_CHIP_XR871
int HAL_PRCM_IsCPUADeepSleepAllowed(void)
{
	return !HAL_GET_BIT(PRCM->SYS2_STATUS, PRCM_CPUA_DEEPSLEEP_LOCK_BIT);
}

__rom_xip_text
int HAL_PRCM_IsCPUNSleep(void)
{
	return HAL_GET_BIT(PRCM->SYS2_STATUS, PRCM_CPUN_SLEEP_STATUS_BIT);
}

__rom_xip_text
int HAL_PRCM_IsCPUNDeepSleep(void)
{
	return HAL_GET_BIT(PRCM->SYS2_STATUS, PRCM_CPUN_DEEPSLEEP_STATUS_BIT);
}

int HAL_PRCM_IsSys2Alive(void)
{
	return HAL_GET_BIT(PRCM->SYS2_STATUS, PRCM_SYS2_ALIVE_BIT);
}
#endif

__rom_xip_text
int HAL_PRCM_IsSys3Alive(void)
{
	return HAL_GET_BIT(PRCM->SYS3_STATUS, PRCM_SYS3_ALIVE_BIT);
}

void HAL_PRCM_SetSys1WakeupPowerFlags(uint32_t flags)
{
	PRCM->SYS1_WAKEUP_CTRL = flags & PRCM_SYS_WS_PWR_FLAGS_MASK;
}

uint32_t HAL_PRCM_GetSys1WakeupPowerFlags(void)
{
	return (PRCM->SYS1_WAKEUP_CTRL & PRCM_SYS_WS_PWR_FLAGS_MASK);
}

#if (defined(__CONFIG_CHIP_XR875_ON_XR871) || defined(__CONFIG_ARCH_NET_CORE))
void HAL_PRCM_SetSys2WakeupPowerFlags(uint32_t flags)
{
	PRCMN->SYS2_WAKEUP_CTRL = flags & PRCM_SYS_WS_PWR_FLAGS_MASK;
}
#endif

#ifdef __CONFIG_CHIP_XR871
uint32_t HAL_PRCM_GetSys2WakeupPowerFlags(void)
{
	return (PRCM->SYS2_WAKEUP_CTRL & PRCM_SYS_WS_PWR_FLAGS_MASK);
}
#endif

uint32_t HAL_PRCM_GetSys1SleepPowerFlags(void)
{
	return (PRCM->SYS1_SLEEP_CTRL & PRCM_SYS_WS_PWR_FLAGS_MASK);
}

void HAL_PRCM_SetSys1SleepPowerFlags(uint32_t flags)
{
	PRCM->SYS1_SLEEP_CTRL = flags & PRCM_SYS_WS_PWR_FLAGS_MASK;
}

#ifdef __CONFIG_CHIP_XR871
uint32_t HAL_PRCM_GetSys2SleepPowerFlags(void)
{
	return (PRCM->SYS2_SLEEP_CTRL & PRCM_SYS_WS_PWR_FLAGS_MASK);
}

void HAL_PRCM_SetSRAMVoltage(PRCM_SRAMVolt workVolt, PRCM_SRAMVolt retenVolt)
{
	PRCM->SRAM_VOLT_CTRL =
		((workVolt << PRCM_SRAM_WORK_VOLT_SHIFT) & PRCM_SRAM_WORK_VOLT_MASK) |
		((retenVolt << PRCM_SRAM_RETEN_VOLT_SHIFT) & PRCM_SRAM_RETEN_VOLT_MASK);
}
#endif

void HAL_PRCM_SetBANDGAPSTABLE_TIME(uint32_t time)
{
	PRCM->BANDGAP_STABLE_REF_TIME = (time & PRCM_BANDGAP_STABLE_REF_TIME_MASK);
}

#ifdef __CONFIG_CHIP_XR875
void HAL_PRCM_SetRTCLDOVoltage(PRCM_RTCLDORetentionVolt retenVolt, PRCM_RTCLDOWorkVolt workVolt)
{
	PRCM->RTC_LDO_VOLT_CTRL = retenVolt | workVolt;
}
#endif

uint32_t HAL_PRCM_GetBANDGAPSTABLE_TIME(void)
{
	return (PRCM->BANDGAP_STABLE_REF_TIME & PRCM_BANDGAP_STABLE_REF_TIME_MASK);
}

void HAL_PRCM_SetDCDCSTABLE_TIME(uint32_t time)
{
	PRCM->DCDC_STABLE_REF_TIME = (time & PRCM_DCDC_STABLE_REF_TIME_MASK);
}

uint32_t HAL_PRCM_GetDCDCSTABLE_TIME(void)
{
	return (PRCM->DCDC_STABLE_REF_TIME & PRCM_DCDC_STABLE_REF_TIME_MASK);
}

void HAL_PRCM_SetCPUABootFlag(PRCM_CPUABootFlag flag)
{
	PRCM->CPUA_BOOT_FLAG = PRCM_CPUA_BOOT_FLAG_WR_LOCK | flag;
}

uint32_t HAL_PRCM_GetCPUABootFlag(void)
{
	return HAL_GET_BIT(PRCM->CPUA_BOOT_FLAG, PRCM_CPUA_BOOT_FLAG_MASK);
}

void HAL_PRCM_SetCPUABootAddr(uint32_t addr)
{
	PRCM->CPUA_BOOT_ADDR = addr;
}

uint32_t HAL_PRCM_GetCPUABootAddr(void)
{
	return PRCM->CPUA_BOOT_ADDR;
}

void HAL_PRCM_SetCPUABootArg(uint32_t arg)
{
	PRCM->CPUA_BOOT_ARG = arg;
}

uint32_t HAL_PRCM_GetCPUABootArg(void)
{
	return PRCM->CPUA_BOOT_ARG;
}

#if (defined(__CONFIG_CHIP_XR875_ON_XR871) || defined(__CONFIG_ARCH_NET_CORE))
void HAL_PRCM_SetCPUNBootFlag(PRCM_CPUNBootFlag flag)
{
	PRCMN->CPUN_BOOT_FLAG = PRCM_CPUN_BOOT_FLAG_WR_LOCK | flag;
}

uint32_t HAL_PRCM_GetCPUNBootFlag(void)
{
	return HAL_GET_BIT(PRCM->CPUN_BOOT_FLAG, PRCM_CPUN_BOOT_FLAG_MASK);
}

void HAL_PRCM_SetCPUNBootAddr(uint32_t addr)
{
	PRCMN->CPUN_BOOT_ADDR = addr;
}

uint32_t HAL_PRCM_GetCPUNBootAddr(void)
{
	return PRCM->CPUN_BOOT_ADDR;
}

void HAL_PRCM_SetCPUNBootArg(uint32_t arg)
{
	PRCMN->CPUN_BOOT_ARG = arg;
}

uint32_t HAL_PRCM_GetCPUNBootArg(void)
{
	return PRCM->CPUN_BOOT_ARG;
}
#endif /* __CONFIG_CHIP_XR875_ON_XR871 or __CONFIG_ARCH_NET_CORE */

void HAL_PRCM_SetCPUAPrivateData(uint32_t id, uint32_t data)
{
#if defined(__CONFIG_CHIP_XR871)
	if (id == 0) {
		PRCM->CPUA_PRIV_REG = data;
	}
#elif defined(__CONFIG_CHIP_XR875)
	if (id < PRCM_CPUA_PRIV_DATA_ID_NUM) {
		PRCM->CPUA_PRIV_REG_0T3[id] = data;
	}
#endif
}

uint32_t HAL_PRCM_GetCPUAPrivateData(uint32_t id)
{
#if defined(__CONFIG_CHIP_XR871)
	if (id == 0) {
		return PRCM->CPUA_PRIV_REG;
	}
#elif defined(__CONFIG_CHIP_XR875)
	if (id < PRCM_CPUA_PRIV_DATA_ID_NUM) {
		return PRCM->CPUA_PRIV_REG_0T3[id];
	}
#endif
	else {
		return 0;
	}
}

#ifdef __CONFIG_CHIP_XR875
void HAL_PRCM_SetPrivateTime(uint64_t val)
{
	PRCM->CPUA_PRIV_TIME_L = (uint32_t)(val & 0xffffffff);
	PRCM->CPUA_PRIV_TIME_H = (uint32_t)((val >> 32) & 0xffffffff);
}

uint64_t HAL_PRCM_GetPrivateTime(void)
{
	return (((uint64_t)PRCM->CPUA_PRIV_TIME_H << 32) | PRCM->CPUA_PRIV_TIME_L);
}
#endif

uint32_t HAL_PRCM_GetWakeupTimerEnable(void)
{
	return (PRCM->CPUA_WAKE_TIMER_CNT & PRCM_CPUx_WAKE_TIMER_EN_BIT);
}

void HAL_PRCM_WakeupTimerEnable(void)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_TIMER_CNT, PRCM_CPUx_WAKE_TIMER_EN_BIT);
}

void HAL_PRCM_WakeupTimerDisable(void)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_TIMER_CNT, PRCM_CPUx_WAKE_TIMER_EN_BIT);
}

uint32_t HAL_PRCM_WakeupTimerGetCurrentValue(void)
{
	return (PRCM->CPUA_WAKE_TIMER_CNT & PRCM_CPUx_WAKE_TIMER_CUR_VAL_MASK);
}

uint32_t HAL_PRCM_GetWakeupTimerPending(void)
{
	return (PRCM->CPUA_WAKE_TIMER_CMP & PRCM_CPUx_WAKE_TIMER_PENDING_BIT);
}

void HAL_PRCM_ClearWakeupTimerPending(void)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_TIMER_CMP, PRCM_CPUx_WAKE_TIMER_PENDING_BIT);
}

void HAL_PRCM_WakeupTimerSetCompareValue(uint32_t val)
{
	PRCM->CPUA_WAKE_TIMER_CMP = val & PRCM_CPUx_WAKE_TIMER_CMP_VAL_MASK;
}

uint32_t HAL_PRCM_WakeupTimerGetCompareValue(void)
{
	return (PRCM->CPUA_WAKE_TIMER_CMP & PRCM_CPUx_WAKE_TIMER_CMP_VAL_MASK);
}

#if 0
uint32_t HAL_PRCM_GetCPUNWakeupTimerCurrentValue(void)
{
	return (PRCM->CPUN_WAKE_TIMER_CNT & PRCM_CPUx_WAKE_TIMER_CUR_VAL_MASK);
}

void HAL_PRCM_SetCPUNWakeupTimerCompareValue(uint32_t val)
{
	PRCM->CPUN_WAKE_TIMER_CMP = val & PRCM_CPUx_WAKE_TIMER_CMP_VAL_MASK);
}

uint32_t HAL_PRCM_GetCPUNWakeupTimerCompareValue(void)
{
	return (PRCM->CPUN_WAKE_TIMER_CMP & PRCM_CPUx_WAKE_TIMER_CMP_VAL_MASK);
}
#endif

void HAL_PRCM_WakeupIOEnable(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_EN, ioMask);
}

void HAL_PRCM_WakeupIODisable(uint32_t ioMask)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_EN, ioMask);
}

void HAL_PRCM_WakeupIOSetRisingEvent(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_MODE, ioMask);
}

void HAL_PRCM_WakeupIOSetFallingEvent(uint32_t ioMask)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_MODE, ioMask);
}

uint32_t HAL_PRCM_WakeupIOGetEventStatus(void)
{
	return HAL_GET_BIT(PRCM->CPUA_WAKE_IO_STATUS, PRCM_WAKE_IO_MASK);
}

int HAL_PRCM_WakeupIOIsEventDetected(uint32_t ioMask)
{
	return HAL_GET_BIT(PRCM->CPUA_WAKE_IO_STATUS, ioMask);
}

void HAL_PRCM_WakeupIOClearEventDetected(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_STATUS, ioMask);
}

void HAL_PRCM_WakeupIOEnableCfgHold(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_HOLD, ioMask);
}

void HAL_PRCM_WakeupIODisableCfgHold(uint32_t ioMask)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_HOLD, ioMask);
}

void HAL_PRCM_WakeupIOEnableGlobal(void)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_GLOBAL_EN, PRCM_WAKE_IO_GLOBAL_EN_BIT);
}

void HAL_PRCM_WakeupIODisableGlobal(void)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_GLOBAL_EN, PRCM_WAKE_IO_GLOBAL_EN_BIT);
}

#ifdef __CONFIG_CHIP_XR875

void HAL_PRCM_EnableWakeupIOx(uint8_t ioIndex, uint8_t enable)
{
	HAL_ASSERT_PARAM(ioIndex < 10);

	if (enable)
		HAL_SET_BIT(PRCM->CPUA_WAKE_IO_EN, 1 << ioIndex);
	else
		HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_EN, 1 << ioIndex);
}

void HAL_PRCM_SetWakeupIOxDebouce(uint8_t ioIndex, uint8_t val)
{
	HAL_ASSERT_PARAM(ioIndex < 10);

	if (ioIndex < 4) {
		HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_EN,
		               PRCM_WAKEUP_IO0T3_DEDOUNCE_CYCLE_MASK(ioIndex),
		               val << PRCM_WAKEUP_IO0T3_DEDOUNCE_CYCLE_SHIFT(ioIndex));
	} else if (ioIndex < 10) {
		HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_GLOBAL_EN,
		               PRCM_WAKEUP_IO4T9_DEDOUNCE_CYCLE_MASK(ioIndex),
		               val << PRCM_WAKEUP_IO4T9_DEDOUNCE_CYCLE_SHIFT(ioIndex));
	}
}

void HAL_PRCM_SetWakeupIOxDebSrc(uint8_t ioIndex, uint8_t val)
{
	HAL_ASSERT_PARAM(ioIndex < 10);

	if (val)
		HAL_SET_BIT(PRCM->CPUA_WAKE_IO_MODE, 1 << (ioIndex + PRCM_WAKEUP_IOX_DEB_CLK_SRC_SHIFT));
	else
		HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_MODE, 1 << (ioIndex + PRCM_WAKEUP_IOX_DEB_CLK_SRC_SHIFT));
}

void HAL_PRCM_SetWakeupDebClk0(uint8_t val)
{
	HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_MODE, PRCM_WKAEUP_DEB_CLK0_MASK, val << PRCM_WKAEUP_DEB_CLK0_SHIFT);
}

void HAL_PRCM_SetWakeupDebClk1(uint8_t val)
{
	HAL_MODIFY_REG(PRCM->CPUA_WAKE_IO_MODE, PRCM_WKAEUP_DEB_CLK1_MASK, val << PRCM_WKAEUP_DEB_CLK1_SHIFT);
}

#endif

int HAL_PRCM_IsFlashSip(void)
{
	return HAL_GET_BIT(PRCM->BONDING_IO, PRCM_FLASH_SIP_EN_BIT);
}

uint32_t HAL_PRCM_GetFlashSipMode(void)
{
	return HAL_GET_BIT(PRCM->BONDING_IO, PRCM_FLASH_SIP_MODE_MASK);
}

void HAL_PRCM_Start(void)
{
	HAL_CLR_BIT(PRCM->CPUA_PRCM_REG, PRCM_CPUA_PRCM_REG_BIT);
}

#if 0
void HAL_PRCM_EnableCPUAWakeupTimer(void)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_TIMER_CNT, PRCM_CPUx_WAKE_TIMER_EN_BIT);
}

void HAL_PRCM_DisableCPUAWakeupTimer(void)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_TIMER_CNT, PRCM_CPUx_WAKE_TIMER_EN_BIT);
}

uint32_t HAL_PRCM_GetCPUAWakeupTimerCurrentValue(void)
{
	return (PRCM->CPUA_WAKE_TIMER_CNT & PRCM_CPUx_WAKE_TIMER_CUR_VAL_MASK);
}

void HAL_PRCM_SetCPUAWakeupTimerCompareValue(uint32_t val)
{
	PRCM->CPUA_WAKE_TIMER_CMP = val & PRCM_CPUx_WAKE_TIMER_CMP_VAL_MASK;
}

uint32_t HAL_PRCM_GetCPUAWakeupTimerCompareValue(void)
{
	return (PRCM->CPUA_WAKE_TIMER_CMP & PRCM_CPUx_WAKE_TIMER_CMP_VAL_MASK);
}

#if 0
uint32_t HAL_PRCM_GetCPUNWakeupTimerCurrentValue(void)
{
	return (PRCM->CPUN_WAKE_TIMER_CNT & PRCM_CPUx_WAKE_TIMER_CUR_VAL_MASK);
}

void HAL_PRCM_SetCPUNWakeupTimerCompareValue(uint32_t val)
{
	PRCM->CPUN_WAKE_TIMER_CMP = val & PRCM_CPUx_WAKE_TIMER_CMP_VAL_MASK);
}

uint32_t HAL_PRCM_GetCPUNWakeupTimerCompareValue(void)
{
	return (PRCM->CPUN_WAKE_TIMER_CMP & PRCM_CPUx_WAKE_TIMER_CMP_VAL_MASK);
}
#endif

void HAL_PRCM_EnableWakeupIO(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_EN, ioMask);
}

void HAL_PRCM_DisableWakeupIO(uint32_t ioMask)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_EN, ioMask);
}

void HAL_PRCM_SetWakeupIORisingEvent(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_MODE, ioMask);
}

void HAL_PRCM_SetWakeupIOFallingEvent(uint32_t ioMask)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_MODE, ioMask);
}

uint32_t HAL_PRCM_GetWakeupIOEventStatus(void)
{
	return HAL_GET_BIT(PRCM->CPUA_WAKE_IO_STATUS, PRCM_WAKE_IO_MASK);
}

int HAL_PRCM_IsWakeupIOEventDetected(uint32_t ioMask)
{
	return HAL_GET_BIT(PRCM->CPUA_WAKE_IO_STATUS, ioMask);
}

void HAL_PRCM_ClearWakeupIOEventDetected(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_STATUS, ioMask);
}

void HAL_PRCM_EnableWakeupIOCfgHold(uint32_t ioMask)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_HOLD, ioMask);
}

void HAL_PRCM_DisableWakeupIOCfgHold(uint32_t ioMask)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_HOLD, ioMask);
}

void HAL_PRCM_EnableWakeupIOGlobal(void)
{
	HAL_SET_BIT(PRCM->CPUA_WAKE_IO_GLOBAL_EN, PRCM_WAKE_IO_GLOBAL_EN_BIT);
}

void HAL_PRCM_DisableWakeupIOGlobal(void)
{
	HAL_CLR_BIT(PRCM->CPUA_WAKE_IO_GLOBAL_EN, PRCM_WAKE_IO_GLOBAL_EN_BIT);
}
#endif

#ifdef __CONFIG_CHIP_XR875
uint32_t HAL_PRCM_GetResetSource(void)
{
	return PRCM->CPU_RESET_SOURCE;
}

void HAL_PRCM_ClrResetSource(void)
{
	HAL_SET_BIT(PRCM->CPU_RESET_SOURCE, PRCM_CPU_IS_PWRON_RST_BIT | PRCM_CPU_IS_PMU_RST_BIT \
			| PRCM_CPU_IS_WATCHDOG_ALL_RST_BIT | PRCM_CPU_IS_WATCHDOG_CPU_RST_MASK);
}

void HAL_PRCM_SetWdgNoResetPeriph(uint32_t periphMask, int8_t enable)
{
	if (enable) {
		HAL_SET_BIT(PRCM->WDG_NORESET_PERIPH, periphMask);
	} else {
		HAL_CLR_BIT(PRCM->WDG_NORESET_PERIPH, periphMask);
	}

	if (PRCM->WDG_NORESET_PERIPH & PRCM_WDG_NORESET_PERIPH_MASK) {
		HAL_SET_BIT(PRCM->WDG_NORESET_PERIPH, PRCM_WDG_NORESET_PERIPH_EN_BIT);
	} else {
		HAL_CLR_BIT(PRCM->WDG_NORESET_PERIPH, PRCM_WDG_NORESET_PERIPH_EN_BIT);
	}
}

void HAL_PRCM_EnableTOPLDODeepsleep(uint8_t enable)
{
	if (enable)
		HAL_SET_BIT(PRCM->DCDC_PARAM_CTRL, PRCM_TOP_LDO_DEEPSLEEP_EN_BIT);
	else
		HAL_CLR_BIT(PRCM->DCDC_PARAM_CTRL, PRCM_TOP_LDO_DEEPSLEEP_EN_BIT);
}

void HAL_PRCM_EnableWlanCPUClk(uint8_t enable)
{
	if (enable)
		HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_DISABLE_CPU_CLK_BIT); /* 0 is enable */
	else
		HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_DISABLE_CPU_CLK_BIT);
}

void HAL_PRCM_ReleaseWlanCPUReset(void)
{
	HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_RESET_CPU_BIT);
}

void HAL_PRCM_ForceWlanCPUReset(void)
{
	HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_RESET_CPU_BIT);
}

void HAL_PRCM_WakeUpWlan(uint8_t wakeup)
{
	if (wakeup)
		HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_WUP_BIT);
	else
		HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_WUP_BIT);
}

void HAL_PRCM_EnableWlanCPUClkOvrHIF(void)
{
	HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_DISABLE_CPU_CLK_OVR_HIF_BIT);
}

void HAL_PRCM_DisableWlanCPUClkOvrHIF(void)
{
	HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_DISABLE_CPU_CLK_OVR_HIF_BIT);
}

void HAL_PRCM_ReleaseWlanCPUOvrHIF(void)
{
	HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_RESET_CPU_OVR_HIF_BIT);
}

void HAL_PRCM_ResetWlanCPUOvrHIF(void)
{
	HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_RESET_CPU_OVR_HIF_BIT);
}

void HAL_PRCM_EnableWlanWUPOvrHIF(void)
{
	HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_WUP_OVR_HIF_BIT);
}

void HAL_PRCM_DisableWlanWUPOvrHIF(void)
{
	HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_WUP_OVR_HIF_BIT);
}

void HAL_PRCM_EnableWlanIRQOvrHIF(void)
{
	HAL_SET_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_IRQ_OVR_HIF_BIT);
}

void HAL_PRCM_DisableWlanIRQOvrHIF(void)
{
	HAL_CLR_BIT(PRCM->WLAN_HIF_OVERRIDE_CTRL, PRCM_WLAN_IRQ_OVR_HIF_BIT);
}
#endif
