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

#include <stdio.h>
#include "cmd_util.h"
#include "kernel/os/os.h"
#include "driver/chip/hal_def.h"
#include "driver/component/oled/drv_oled.h"
//#include "oled_bmp_image.h"
#include "cmd_oled.h"

#define OLED_IMG_SHOW_EN		1

#define OLED_SPI_ID  			SPI1
#define OLED_SPI_MCLK  			(6000000)
#define OLED_SPI_CS  			SPI_TCTRL_SS_SEL_SS0
#define OLED_SPT_DS_PIN			GPIO_PIN_20
#define OLED_SPT_DS_PORT		GPIO_PORT_A
#define OLED_SPT_RESET_PIN		GPIO_PIN_8
#define OLED_SPT_RESET_PORT		GPIO_PORT_A

static int oled_init()
{
	Oled_Config oled_cfg;
	oled_cfg.oled_SPI_ID	 = OLED_SPI_ID;
	oled_cfg.oled_SPI_MCLK	 = OLED_SPI_MCLK;
	oled_cfg.oled_SPI_CS	 = OLED_SPI_CS;
	oled_cfg.oled_dsPin		 = OLED_SPT_DS_PIN;
	oled_cfg.oled_dsPort	 = OLED_SPT_DS_PORT;
	oled_cfg.oled_reset_Pin	 = OLED_SPT_RESET_PIN;
	oled_cfg.oled_reset_Port = OLED_SPT_RESET_PORT;

	if (DRV_Oled_Init(&oled_cfg) != COMP_OK) {
		printf("oled init error\n");
		return -1;
	}
	return 0;
}

static void oled_deinit()
{
	DRV_Oled_DeInit();
}

static enum cmd_status cmd_oled_init_exec(char *cmd)
{
	if (oled_init() != 0) {
		CMD_ERR("oled init fail\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_oled_control_exec(char *cmd)
{
	int32_t cnt;
	char mode[8];
	char value[65];

	cnt = cmd_sscanf(cmd, "m=%7s v=%64s", mode, value);
	if (cnt != 2) {
		return CMD_STATUS_INVALID_ARG;
	}

	if (cmd_strcmp(mode, "lightup") == 0) {
		uint8_t light_en = 0;
		if (cmd_strcmp(value, "1") == 0) {
			light_en = 1;
			CMD_DBG("light up\n");
		}
		//DRV_Oled_OnOff(light_en);
		(void)light_en;
		//DRV_Oled_Set_Brightness(100);
		DRV_Oled_ScreenLightUp(light_en);
	} else if (cmd_strcmp(mode, "display") == 0) {
		if (strlen(value) > 64) {
			CMD_ERR("invalid value:%s\n", value);
			return CMD_STATUS_INVALID_ARG;
		}
		DRV_Oled_Show_Str_1608(0, 0, value);
		CMD_DBG("%s\n",value);
	} else {
		CMD_ERR("invalid mode:%s\n", mode);
		return CMD_STATUS_INVALID_ARG;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_oled_deinit_exec(char *cmd)
{
	oled_deinit();

	return CMD_STATUS_OK;
}

static const struct cmd_data g_oled_cmds[] = {
	{ "init",     	cmd_oled_init_exec },
	{ "control",   	cmd_oled_control_exec },
	{ "deinit",    	cmd_oled_deinit_exec },
};

enum cmd_status cmd_oled_exec(char *cmd)
{
	return cmd_exec(cmd, g_oled_cmds, cmd_nitems(g_oled_cmds));
}
