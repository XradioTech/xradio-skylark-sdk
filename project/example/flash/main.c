
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
#include "driver/chip/hal_flash.h"

#define MFLASH 0

static void flash_demo(void)
{
	uint32_t addr;
	FlashEraseMode size_type;
	uint8_t wdata[256];
	uint8_t rdata[256] = {0};
	int i;

	for (i = 0; i < 256; i++) {
		wdata[i] = i;
	}

	addr = 0x108000;
	size_type = FLASH_ERASE_4KB;

	printf("flash driver open...\n");
	if (HAL_Flash_Open(MFLASH, 5000) != HAL_OK) {
		printf("flash driver open failed\n");
		return;
	}

	printf("flash erase...\n");
	if (HAL_Flash_Erase(MFLASH, size_type, addr, 1) != HAL_OK) {
		printf("flash erase failed\n");
		goto fail;
	}

	printf("flash write...\n");
	if (HAL_Flash_Write(MFLASH, addr, wdata, sizeof(wdata)) != HAL_OK) {
		printf("flash write failed\n");
		goto fail;
	}

	printf("flash read...\n");
	if (HAL_Flash_Read(MFLASH, addr, rdata, 256) != HAL_OK) {
		printf("flash read failed\n");
		goto fail;
	}

	printf("flash read result:\n");
	for (i = 0; i < 256; i++) {
		printf("%d ", rdata[i]);
		if (i != 0 && (i % 16) == 0)
			printf("\n");
	}
	printf("\n");

	if(memcmp(wdata, rdata, sizeof(wdata)) == 0) {
		printf("flash erase write read suscess\n");
	} else
		printf("flash erase write read fail\n");

fail:
	printf("flash driver close.\n");
	if (HAL_Flash_Close(MFLASH) != HAL_OK) {
		printf("flash driver close failed\n");
		return;
	}
}


int main(void)
{
	printf("flash demo started.\n");

	HAL_Flash_Init(MFLASH);

	flash_demo();

	HAL_Flash_Deinit(MFLASH);

	printf("flash demo over.\n");
	return 0;
}

