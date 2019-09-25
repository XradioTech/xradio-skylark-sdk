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

#include <string.h>

#include "cmd_util.h"
#include "console/console.h"

static enum cmd_status cmd_console_enable_exec(char *cmd)
{
	console_enable();

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_console_disable_exec(char *cmd)
{
	int argc;
	char *argv[2];

	argc = cmd_parse_argv(cmd, argv, 2);
	if (argc < 1) {
		CMD_ERR("invalid cmd, argc %d\n", argc);
		return CMD_STATUS_FAIL;
	}
	uint32_t sec = cmd_atoi(argv[0]);

	console_disable();

	OS_Sleep(sec);

	console_enable();

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_console_get_exec(char *cmd)
{
	if (cmd_strcmp(cmd, "uart_id") != 0) {
		CMD_ERR("invalid cmd %s\n", cmd);
		return CMD_STATUS_FAIL;
	}

	UART_ID id = console_get_uart_id();
	if (id == UART_INVALID_ID) {
		CMD_ERR("invalid uart ID\n");
		return CMD_STATUS_FAIL;
	}
	CMD_DBG("uart ID is %d\n", id);

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_console_write_exec(char *cmd)
{
	uint32_t len = strlen(cmd);
	uint8_t *buf = (uint8_t *)cmd;

	if (console_write(buf, len) != len) {
		CMD_ERR("write fail\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static const struct cmd_data g_console_cmds[] = {
	{ "enable",			cmd_console_enable_exec },
	{ "disable",		cmd_console_disable_exec },
	{ "get",			cmd_console_get_exec },
	{ "write",			cmd_console_write_exec },
};

enum cmd_status cmd_console_exec(char *cmd)
{
	return cmd_exec(cmd, g_console_cmds, cmd_nitems(g_console_cmds));
}
