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

#include "cmd_util.h"
#include "cmd_gpio.h"
#include "driver/chip/hal_gpio.h"

static void _gpio_cd_irq(void *arg)
{
	GPIO_PinState state;
	uint32_t pin_config = (uint32_t)arg;
	uint32_t port, pin, event;

	port = (pin_config >> 16) & 0x0ff;
	pin = (pin_config >> 8) & 0x0ff;
	event = pin_config & 0x0ff;
	state = HAL_GPIO_ReadPin(port, pin);

	CMD_DBG("gpio irq state:%d event:%d\n", state, event);
}

/*
 * drv gpio config <gpio> m=<mode> p=<pull> l=<level> [e=<event>]
 * eg. drv gpio config pa6 m=1 p=1 l=1
 * eg. drv gpio config pa6 m=1 p=1 l=1 e=0
 */
static enum cmd_status cmd_gpio_config_exec(char *cmd)
{
	int32_t cnt;
	char port;
	char gpio[3];
	uint32_t pin, mode, pull, level, event;
	GPIO_InitParam param;

	cnt = cmd_sscanf(cmd, "%2s%d m=%d p=%d l=%d e=%d", gpio, &pin, &mode, &pull, &level, &event);
	if (cnt != 6 || pull > GPIO_CTRL_PULL_MAX) {
		cnt = cmd_sscanf(cmd, "%2s%d m=%d p=%d l=%d", gpio, &pin, &mode, &pull, &level);
		if (cnt != 5 || pull > GPIO_CTRL_PULL_MAX) {
			goto err;
		}
	}

	if (mode == GPIOx_Pn_F6_EINT && cnt != 6) {
		goto err;
	}

	if (cmd_strcmp(gpio, "PA") == 0 || cmd_strcmp(gpio, "pa")  == 0)
		port = GPIO_PORT_A;
	else if (cmd_strcmp(gpio, "PB") == 0 || cmd_strcmp(gpio, "pb")  == 0)
		port = GPIO_PORT_B;
#if (__CONFIG_CHIP_ARCH_VER == 2)
	else if (cmd_strcmp(gpio, "PC") == 0 || cmd_strcmp(gpio, "pc")  == 0)
		port = GPIO_PORT_C;
#endif
	else
		goto err;
	param.mode = mode;
	param.driving = level;
	param.pull = pull;
	HAL_GPIO_Init(port, pin, &param);

	if (mode == GPIOx_Pn_F6_EINT) {
		GPIO_IrqParam Irq_param;

		Irq_param.event = event;
		Irq_param.callback = _gpio_cd_irq;
		Irq_param.arg = (void *)((port << 16) | (pin << 8) | event);
		HAL_GPIO_EnableIRQ(port, pin, &Irq_param);
	}

	return CMD_STATUS_OK;

err:
	CMD_ERR("err cmd:%s, expect: <GPIO> m=<Mode> p=<Pull> l=<Level> [e=<Event>]\n", cmd);
	return CMD_STATUS_INVALID_ARG;}

/*
 * drv gpio deconfig <gpio> [m=<mode>]
 * eg. drv gpio deconfig pa6
 * eg. drv gpio deconfig pa6 m=6
 */
static enum cmd_status cmd_gpio_deconfig_exec(char *cmd)
{
	int32_t cnt;
	char port;
	char gpio[3];
	uint32_t pin, mode;

	cnt = cmd_sscanf(cmd, "%2s%d m=%d", gpio, &pin, &mode);
	if (cnt != 3) {
		cnt = cmd_sscanf(cmd, "%2s%d", gpio, &pin);
		if (cnt != 2) {
			CMD_ERR("err cmd:%s, expect: <gpio> m=<Mode>\n", cmd);
			return CMD_STATUS_INVALID_ARG;
		}
	}

	if (cmd_strcmp(gpio, "PA") == 0 || cmd_strcmp(gpio, "pa")  == 0)
		port = GPIO_PORT_A;
	else if (cmd_strcmp(gpio, "PB") == 0 || cmd_strcmp(gpio, "pb")	== 0)
		port = GPIO_PORT_B;
#if (__CONFIG_CHIP_ARCH_VER == 2)
	else if (cmd_strcmp(gpio, "PC") == 0 || cmd_strcmp(gpio, "pc")	== 0)
		port = GPIO_PORT_C;
#endif
	else
		return CMD_STATUS_INVALID_ARG;

	HAL_GPIO_DeInit(port, pin);

	if (mode == GPIOx_Pn_F6_EINT) {
		HAL_GPIO_DisableIRQ(port, pin);
	}

	return CMD_STATUS_OK;
}

/*
 * drv gpio read <gpio>
 * eg. drv gpio read pa6
 */
static enum cmd_status cmd_gpio_read_exec(char *cmd)
{
	enum cmd_status ret = CMD_STATUS_OK;
	int32_t cnt;
	char port;
	char gpio[3];
	uint32_t pin, state;

	cnt = cmd_sscanf(cmd, "%2s%d", gpio, &pin);

	if (cmd_strcmp(gpio, "PA") == 0 || cmd_strcmp(gpio, "pa")  == 0)
		port = GPIO_PORT_A;
	else if (cmd_strcmp(gpio, "PB") == 0 || cmd_strcmp(gpio, "pb")	== 0)
		port = GPIO_PORT_B;
#if (__CONFIG_CHIP_ARCH_VER == 2)
	else if (cmd_strcmp(gpio, "PC") == 0 || cmd_strcmp(gpio, "pc")	== 0)
		port = GPIO_PORT_C;
#endif
	else
		ret = CMD_STATUS_INVALID_ARG;

	if (cnt != 2 || ret == CMD_STATUS_INVALID_ARG) {
		CMD_ERR("err cmd:%s, expect: <gpio>\n", cmd);
		return ret;
	}

	state = HAL_GPIO_ReadPin(port, pin);
	CMD_DBG("read gpio value:%d\n", state);

	return CMD_STATUS_OK;
}

/*
 * drv gpio write <gpio> v=<value>
 * eg. drv gpio write pa6 v=0
 */
static enum cmd_status cmd_gpio_write_exec(char *cmd)
{
	int32_t cnt;
	char port;
	char gpio[3];
	uint32_t pin, state;

	cnt = cmd_sscanf(cmd, "%2s%d v=%d", gpio, &pin, &state);
	if (cnt != 3) {
		CMD_ERR("err cmd:%s, expect: <gpio> v=<value>\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	if (cmd_strcmp(gpio, "PA") == 0 || cmd_strcmp(gpio, "pa")  == 0)
		port = GPIO_PORT_A;
	else if (cmd_strcmp(gpio, "PB") == 0 || cmd_strcmp(gpio, "pb")	== 0)
		port = GPIO_PORT_B;
#if (__CONFIG_CHIP_ARCH_VER == 2)
	else if (cmd_strcmp(gpio, "PC") == 0 || cmd_strcmp(gpio, "pc")	== 0)
		port = GPIO_PORT_C;
#endif
	else
		return CMD_STATUS_INVALID_ARG;

	HAL_GPIO_WritePin(port, pin, state);

	return CMD_STATUS_OK;
}

static const struct cmd_data g_gpio_cmds[] = {
	{ "config",     cmd_gpio_config_exec },
	{ "deconfig",   cmd_gpio_deconfig_exec },
	{ "read",       cmd_gpio_read_exec },
	{ "write",      cmd_gpio_write_exec },
};

enum cmd_status cmd_gpio_exec(char *cmd)
{
	return cmd_exec(cmd, g_gpio_cmds, cmd_nitems(g_gpio_cmds));
}
