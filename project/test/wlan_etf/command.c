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

#include "common/cmd/cmd_util.h"
#include "common/cmd/cmd.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_gpio.h"

enum cmd_status cmd_prcm_exec(char *cmd)
{
  char *argv[3];

  cmd_parse_argv(cmd, argv, cmd_nitems(argv));

  if (!strcmp("set_top_ldo", argv[0])) {
    if (!strcmp("1.8", argv[1]))
      HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V8_DEFAULT);
    else if (!strcmp("1.4", argv[1]))
      HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V4);
    else if (!strcmp("1.7", argv[1]))
      HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V7);
    else if (!strcmp("1.9", argv[1]))
      HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V9);
    else if (!strcmp("2.0", argv[1]))
      HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_2V0);
  }

  return CMD_STATUS_OK;
}

/*
 * main commands
 */
static const struct cmd_data g_main_cmds[] = {
	{ "mem",	cmd_mem_exec },
	{ "upgrade",    cmd_upgrade_exec },
	{ "reboot",     cmd_reboot_exec },
	{ "etf",	cmd_etf_exec },
	{ "efpg",	cmd_efpg_exec },
	{ "flash",	cmd_flash_exec },
    { "gpio",   cmd_gpio_exec },
    { "prcm",   cmd_prcm_exec },
};

void main_cmd_exec(char *cmd)
{
	cmd_main_exec(cmd, g_main_cmds, cmd_nitems(g_main_cmds));
}
