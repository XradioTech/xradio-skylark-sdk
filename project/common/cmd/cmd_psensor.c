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
#include "cmd_util.h"
#include "cmd_psensor.h"
#include "sys/io.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_psensor.h"

static enum cmd_status cmd_bg_set_exec(char *cmd)
{
	int32_t val;
	int32_t cnt;
	cnt = cmd_sscanf(cmd, " %d", &val);
	if (cnt != 1) {
		CMD_ERR("cmd = %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}
	if((val > 15) || (val < -15)) {
		CMD_ERR("(%d > 15) || (%d < -15)\n", val, val);
		return CMD_STATUS_INVALID_ARG;
	}
	if(val >= 0) {
		HAL_MODIFY_REG(PRCM->DCDC_PARAM_CTRL,
		               PRCM_DCDC_BANDGAP_TRIM_MASK,
		               (uint32_t)val << PRCM_DCDC_BANDGAP_TRIM_SHIFT);
	} else {
		HAL_MODIFY_REG(PRCM->DCDC_PARAM_CTRL,
		               PRCM_DCDC_BANDGAP_TRIM_MASK,
		               (uint32_t)((val+16) | 0x10) << PRCM_DCDC_BANDGAP_TRIM_SHIFT);
	}
	return CMD_STATUS_OK;
}

const static uint32_t digldoVolt[] = {PRCM_LDO1_VOLT_1375MV, PRCM_LDO1_VOLT_1325MV, PRCM_LDO1_VOLT_1275MV,
							PRCM_LDO1_VOLT_1225MV, PRCM_LDO1_VOLT_1175MV, PRCM_LDO1_VOLT_1125MV,
							PRCM_LDO1_VOLT_1025MV, PRCM_LDO1_VOLT_925MV, PRCM_LDO1_VOLT_825MV,
							PRCM_LDO1_VOLT_725MV, PRCM_LDO1_VOLT_625MV, };
const static float digldoVoltFloat[] = {1.375, 1.325, 1.275, 1.225, 1.175, 1.125,
                            1.025, 0.925, 0.825, 0.725, 0.625};

static enum cmd_status cmd_digldo_set_exec(char *cmd)
{
	int32_t cnt;
    float voltFloat;
	cnt = cmd_sscanf(cmd, " %f", &voltFloat);
	if (cnt != 1) {
		CMD_ERR("cmd = %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

    for(int i=0; i<ARRAY_SIZE(digldoVolt); i++) {
        if(voltFloat == digldoVoltFloat[i]) {
            HAL_MODIFY_REG(PRCM->SYS_LDO_SW_CTRL, PRCM_LDO1_VOLT_MASK, digldoVolt[i]);
            CMD_SYSLOG("set digldo = %3fv\n", digldoVoltFloat[i]);
            return CMD_STATUS_OK;
        }
    }

    CMD_ERR("invaild difldo volt = %s\n", cmd);
	return CMD_STATUS_OK;
}


static enum cmd_status cmd_psensor_get_exec(char *cmd)
{
    uint32_t value = HAL_Psensor_GetValue();
    CMD_SYSLOG("psensor value = %d\n", value);
	return CMD_STATUS_OK;
}

static const struct cmd_data g_psensor_cmds[] = {
	{ "get", 	    cmd_psensor_get_exec },
	{ "bg",	    cmd_bg_set_exec },
	{ "digldo",	    cmd_digldo_set_exec },
};

enum cmd_status cmd_psensor_exec(char *cmd)
{
	return cmd_exec(cmd, g_psensor_cmds, cmd_nitems(g_psensor_cmds));
}

#endif /* __CONFIG_CHIP_ARCH_VER == 2 */
