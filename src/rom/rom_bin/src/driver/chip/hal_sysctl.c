/**
  * @file  hal_sysctrl.c
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

#include "rom/driver/chip/hal_sysctl.h"

#include "hal_base.h"

void HAL_SYSCTL_SetMcuMode(uint32_t en)
{
	if (en)
		HAL_SET_BIT(SYSCTL->SRAM_SHARE, SYSCTL_PIN_MCU_MODE_EN);
	else
		HAL_CLR_BIT(SYSCTL->SRAM_SHARE, SYSCTL_PIN_MCU_MODE_EN);
}

void HAL_SYSCTL_SetCSIJPEGSramShare(SYSCTL_CSI_JPE_ShareSramType mode)
{
	HAL_MODIFY_REG(SYSCTL->SRAM_SHARE, SYSCTL_CSI_JPE_SHARE_SRAM_MASK, mode);
}

void HAL_SYSCTL_SetWlanSramShare(SYSCTL_WLAN_ShareSramType type)
{
	HAL_MODIFY_REG(SYSCTL->SRAM_SHARE, SYSCTL_WLAN_SHARE_SRAM_MASK, type);
}

void HAL_SYSCTL_SetPsensorControl(SYSCTL_PsensorId id, SYSCTL_OSCSelect osc_sel,
                              uint32_t ps_n, uint32_t en)
{
	if (en)
		SYSCTL->PS_CTL_REG = id | osc_sel | ps_n | SYSCTL_PS_EN_MASK;
	else
		SYSCTL->PS_CTL_REG = id | osc_sel | ps_n;
}

void HAL_SYSCTL_WaitPsensorRdyAndClean(void)
{
	while (!(SYSCTL->PS_CTL_REG & SYSCTL_CLK250M_CNT_RDY_MASK))
		;
	HAL_CLR_BIT(SYSCTL->PS_CTL_REG, SYSCTL_CLK250M_CNT_RDY_MASK);
}

uint32_t HAL_SYSCTL_GetPsensorCnt(void)
{
	return SYSCTL->PS_CNT_REG;
}

void HAL_SYSCTL_SetDbgData(uint32_t id, uint32_t data)
{
	HAL_ASSERT_PARAM(id < 2);
	SYSCTL->GENRAL_DBG_REG[id] = data;
}

uint32_t HAL_SYSCTL_GetDegData(uint32_t id)
{
	HAL_ASSERT_PARAM(id < 2);
	return SYSCTL->GENRAL_DBG_REG[id];
}
