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

#ifdef __CONFIG_CPU_SUPPORT_349MHZ

#include "hal_base.h"

#ifdef __CONFIG_ROM

/**
 * @brief Get AHB1 clock
 * @return AHB1 clock in Hz
 */
uint32_t HAL_CCM_BusGetAHB1Clock(void)
{
	 return HAL_GetCPUClock();
}

/**
 * @brief Get AHB2 clock
 * @return AHB2 clock in Hz
 */
uint32_t HAL_CCM_BusGetAHB2Clock(void)
{
	 uint32_t div = HAL_GET_BIT_VAL(CCM->CPU_BUS_CLKCFG,
									CCM_AHB2_CLK_DIV_SHIFT,
									CCM_AHB2_CLK_DIV_VMASK) + 1;
	 return HAL_GetCPUClock() / div;
}

/**
 * @brief Get APB clock
 * @return APB clock in Hz
 */
uint32_t HAL_CCM_BusGetAPBClock(void)
{
	 uint32_t reg = CCM->CPU_BUS_CLKCFG;
	 uint32_t freq, div;

	 switch (reg & CCM_APB_CLK_SRC_MASK) {
	 case CCM_APB_CLK_SRC_HFCLK:
		 freq = HAL_GetHFClock();
		 break;
	 case CCM_APB_CLK_SRC_LFCLK:
		 freq = HAL_GetLFClock();
		 break;
	 case CCM_APB_CLK_SRC_AHB2CLK:
	 default:
		 freq = HAL_GetAHB2Clock();
		 break;
	 }

	 div = HAL_GET_BIT_VAL(reg, CCM_APB_CLK_DIV_SHIFT, CCM_APB_CLK_DIV_VMASK);
	 return (freq >> div);
}

#endif /* __CONFIG_ROM */
#endif /* __CONFIG_CPU_SUPPORT_349MHZ */
