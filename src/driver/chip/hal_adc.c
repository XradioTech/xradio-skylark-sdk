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
#include "driver/chip/hal_adc.h"

#ifdef __CONFIG_ROM
/**
 *@brief modify adc vref voltage
 *@param Mode Mode 0 usage is verf inside soc, and input voltage range of 0~1.4v.
 *@           Mode 1 usage is verf inside soc, and input voltage range of 0~2.5v.
 *@           Mode 2 usage is verf outside pin, and input voltage range of 0~2.5v.
 *@return None
 *@NOTE: Mode 0 and Mode 1 can calculate virtual voltage according to ADC Value But Mode 2 and Mode 3 NOT.
         Mode 2 and Mode 3 adc value will not expand when vddio and channel voltage change at the same time.
 */
__xip_text
void HAL_ADC_Set_VrefMode(ADC_VrefMode mode)
{
    uint8_t ext_ldo = !!HAL_GET_BIT(PRCM->SYS_LDO_SW_CTRL, PRCM_EXT_LDO_VOLT_BIT);
    if(mode < 2) {
        HAL_MODIFY_REG(ADC->CTRL, ADC_VREF_MODE_SEL_MASK, mode << ADC_VREF_MODE_SEL_SHIFT);
    } else if(ext_ldo) {
        HAL_MODIFY_REG(ADC->CTRL, ADC_VREF_MODE_SEL_MASK, ADC_VREF_MODE_3 << ADC_VREF_MODE_SEL_SHIFT);
    } else {
        HAL_MODIFY_REG(ADC->CTRL, ADC_VREF_MODE_SEL_MASK, ADC_VREF_MODE_2 << ADC_VREF_MODE_SEL_SHIFT);
    }
}

extern HAL_Status __HAL_ADC_Init(ADC_InitParam *initParam);
/**
 * @brief Initialize the ADC according to the specified parameters.
 * @param[in] initParam Pointer to ADC_InitParam structure.
 * @retval HAL_Status, HAL_OK on success.
 */
__xip_text
HAL_Status HAL_ADC_Init(ADC_InitParam *initParam)
{
    HAL_Status ret;
    ret = __HAL_ADC_Init(initParam);
    HAL_ADC_Set_VrefMode(initParam->vref_mode);
    return ret;
}
#endif /*__CONFIG_ROM*/


