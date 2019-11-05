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
#include "driver/chip/hal_sysctl.h"
uint32_t HAL_SYSCTL_GetPsensor(uint32_t id)
{
    uint32_t value;
    if( id > 1 ) return 0;
    HAL_SYSCTL_SetPsensorControl(id == 0 ? SYSCTL_PSENSOR_0 : SYSCTL_PSENSOR_1,
	                             SYSCTL_OSC_LVT_CASECODE,
	                             SYSCTL_PS_N_PRD_VAL(7),
	                             1);

	HAL_SYSCTL_WaitPsensorRdyAndClean();
	HAL_SET_BIT(SYSCTL->PS_CTL_REG, SYSCTL_CLK250M_CNT_RDY_MASK); // clear ready bit

	value = HAL_SYSCTL_GetPsensorCnt();
	HAL_CLR_BIT(SYSCTL->PS_CTL_REG, SYSCTL_PS_EN_MASK); // disable psensor

    return value;
}

