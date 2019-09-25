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
#include "string.h"
#include "cmd_util.h"
#include "cmd_spi.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_spi.h"
#include "common/board/board.h"

 #define CMD_SPI_PORT SPI1

/*
 * spi config
 */
static enum cmd_status cmd_spi_config_exec(char *cmd)
{
	int ret;
	static const SPI_Global_Config board_spi_param = {
		.mclk	  = BOARD_SPI_MCLK,
		.cs_level = BOARD_SPI_CS_LEVEL
	};

	ret = HAL_SPI_Init(CMD_SPI_PORT, &board_spi_param);
    if(ret != 0) {
        CMD_ERR("HAL_SPI_Init failed\n");
        return CMD_STATUS_FAIL;
    }

    return CMD_STATUS_OK;
}

/*
 * spi deconfig
 */
static enum cmd_status cmd_spi_deconfig_exec(char *cmd)
{
	HAL_SPI_Deinit(CMD_SPI_PORT);

	return CMD_STATUS_OK;
}

/*spi start m=<mode> l=<line> s=<speed>*/
static enum cmd_status cmd_spi_start_exec(char *cmd)
{
    int ret;
    int cnt;
    uint32_t mode,line,speed;
    SPI_Config config;
    uint8_t dataBuf[16];

    cnt = cmd_sscanf(cmd, "m=%u l=%u s=%u", &mode, &line, &speed);
    if (cnt != 3) {
        CMD_ERR("invalid cnt %u\n", cnt);
        return CMD_STATUS_INVALID_ARG;
    }

    if (mode < 0 || mode > 3) {
		CMD_ERR("invalid line %u\n", mode);
		return CMD_STATUS_INVALID_ARG;
	}

    if (line < 1 || line > 2) {
		CMD_ERR("invalid line %u\n", line);
		return CMD_STATUS_INVALID_ARG;
	}

    if (speed < 1 || speed > 48) {
		CMD_ERR("invalid speed %u\n", speed);
		return CMD_STATUS_INVALID_ARG;
	}

    cmd_memset(&config, 0, sizeof(config));
    config.firstBit = SPI_TCTRL_FBS_MSB;
    config.mode = SPI_CTRL_MODE_MASTER;
    config.opMode = SPI_OPERATION_MODE_DMA;
    config.sclk = speed * 1000 * 1000;
    config.sclkMode = mode;

    ret = HAL_SPI_Open(CMD_SPI_PORT, SPI_TCTRL_SS_SEL_SS0, &config, 10);
    if(ret != 0) {
        CMD_ERR("HAL_SPI_Open failed\n");
        return CMD_STATUS_FAIL;
    }

    HAL_SPI_CS(CMD_SPI_PORT, 1);
    cmd_memcpy(dataBuf, "SPI Test Data", sizeof("SPI Test Data"));
    ret = HAL_SPI_Transmit(CMD_SPI_PORT, (uint8_t*)dataBuf, 16);
    if(ret != 0) {
        CMD_ERR("HAL_SPI_Open failed\n");
        goto close;
    }
    if (line == 2)
        HAL_SPI_Config(CMD_SPI_PORT, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_DUAL_RX);
    else
        HAL_SPI_Config(CMD_SPI_PORT, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_NORMAL);

    HAL_SPI_Receive(CMD_SPI_PORT, dataBuf, 16);
    if(ret != 0) {
        CMD_ERR("HAL_SPI_Receive failed\n");
        goto close;
    }

    HAL_SPI_CS(CMD_SPI_PORT, 0);
    cmd_print_uint8_array(dataBuf, 16);

close:
    HAL_SPI_Close(CMD_SPI_PORT);
    return (ret == 0) ? CMD_STATUS_OK : ret;
}



static const struct cmd_data g_spi_cmds[] = {
	{ "config",         cmd_spi_config_exec },
    { "deconfig",       cmd_spi_deconfig_exec },
    { "start",          cmd_spi_start_exec },
};

enum cmd_status cmd_spi_exec(char *cmd)
{
	return cmd_exec(cmd, g_spi_cmds, cmd_nitems(g_spi_cmds));
}
