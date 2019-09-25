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
#include "kernel/os/os.h"
#include "driver/chip/hal_spi.h"

/* spi */
#define DEMO_SPI_MCLK          (48 * 1000 * 1000)
#define DEMO_SPI_CS_LEVEL      0
#define DEMO_SPI_PORT          SPI0
#define DEMO_SPI_CS            SPI_TCTRL_SS_SEL_SS0
#define DEMO_FLASH_INSTRUCTION_RDID  0x9F		/* flash jedec id */

/* FLASH_PN25F16BSSIUG  flashid  0x17701c test ok.
 * You can check flashid in the flash spec.
 */

static int spi_demo(void)
{
	SPI_Config spi_Config;
	HAL_Status ret = HAL_OK;
	uint8_t cmd = DEMO_FLASH_INSTRUCTION_RDID; /* read flash id cmd */
	uint32_t data = 0;

	spi_Config.firstBit = SPI_TCTRL_FBS_MSB;
	spi_Config.mode = SPI_CTRL_MODE_MASTER;
	spi_Config.opMode = SPI_OPERATION_MODE_POLL;
	spi_Config.sclk = 24000000;
	spi_Config.sclkMode = SPI_SCLK_Mode0;

	printf("spi open...\n");
	ret = HAL_SPI_Open(DEMO_SPI_PORT, DEMO_SPI_CS, &spi_Config, 5000);
	if (ret != HAL_OK) {
		printf("spi open failed");
		return ret;
	}

	HAL_SPI_Config(DEMO_SPI_PORT, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_NORMAL);

	HAL_SPI_CS(DEMO_SPI_PORT, 1);

	printf("spi write...\n");
	ret = HAL_SPI_Transmit(DEMO_SPI_PORT, &cmd, 1);
	if (ret != HAL_OK) {
		printf("spi write failed");
		goto failed;
	}

	printf("spi read...\n");
	ret = HAL_SPI_Receive(DEMO_SPI_PORT, (uint8_t *)(&data), 3); /* flash id 3 bytes */
	if (ret != HAL_OK) {
		printf("spi read failed");
		goto failed;
	}

	printf("spi read flashid: 0x%x\n", data);

	HAL_SPI_CS(DEMO_SPI_PORT, 0);

failed:
	printf("spi close.\n");
	HAL_SPI_Close(DEMO_SPI_PORT);
	return ret;
}

int main(void)
{
	printf("spi demo started.\n");

	SPI_Global_Config spi_param;

	spi_param.cs_level = DEMO_SPI_CS_LEVEL;
	spi_param.mclk = DEMO_SPI_MCLK;

	HAL_SPI_Init(DEMO_SPI_PORT, &spi_param);

	spi_demo();

	HAL_SPI_Deinit(DEMO_SPI_PORT);

	printf("spi demo over.\n");
	return 0;
}

