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

#include "debug.h"

#ifdef SAVE_RECORD_PCM_DATA

#define SAVE_TO_SDCARD
//#define SAVE_TO_FLASH

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef SAVE_TO_SDCARD

#include "fs/fatfs/ff.h"
#include "common/framework/fs_ctrl.h"
#include "kernel/os/os_time.h"

#define SAVE_RECORD_DATA_DURATION_MS   10000
struct recordContext {
	FIL file;
	int status;
	unsigned int startTime;
};

static struct recordContext rcdContext;

int record_data_save_start()
{
	unsigned int tick;
	struct recordContext *context = &rcdContext;

	memset(context, 0, sizeof(*context));

	if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
		printf("mount fail\n");
	}
	f_unlink("record/micaec.pcm");
	f_open(&context->file, "record/micaec.pcm", FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	tick = OS_GetTicks();
	context->startTime = OS_TicksToMSecs(tick);
	context->status = 1;
	printf("===start save record data===\n");
	return 0;
}

int record_data_save_put(void *data, unsigned int len)
{
	unsigned int tick;
	unsigned int nowTime;
	unsigned int writeLen;
	struct recordContext *context = &rcdContext;

	if (context->status == 0) {
		return 0;
	}

	f_write(&context->file, data, len, &writeLen);
	tick = OS_GetTicks();
	nowTime = OS_TicksToMSecs(tick);
	if ((nowTime - context->startTime) > SAVE_RECORD_DATA_DURATION_MS) {
		f_close(&context->file);
		context->status = 0;
		printf("===stop save record data===\n");
	}
	return writeLen;
}

int record_data_save_stop()
{
	struct recordContext *context = &rcdContext;

	if (context->status == 1) {
		f_close(&context->file);
		context->status = 0;
		printf("===stop save record data===\n");
	}
	return 0;
}

#else

#include "driver/chip/hal_flash.h"

#define SAVE_BUFFER_LEN  (40 * 32 * 1024)

#define FLASH_START_ADDR  (2048 * 1024)
#define FLASH_DEVICE_NUM  0

struct recordContext {
	int validLen;
	int status;
};

static struct recordContext rcdContext;


int record_data_save_start()
{
	uint32_t blk_cnt;
	struct recordContext *context = &rcdContext;

	memset(context, 0, sizeof(*context));

	context->status = 1;

	if (HAL_Flash_Open(FLASH_DEVICE_NUM, 3000) != HAL_OK) {
        printf("flash open fail\n");
		context->status = 0;
        return -1;
    }
	blk_cnt = SAVE_BUFFER_LEN / (1024 * 4);
	HAL_Flash_Erase(FLASH_DEVICE_NUM, FLASH_ERASE_4KB, FLASH_START_ADDR, blk_cnt);
	printf("===start save record data===\n");
	return 0;
}

int record_data_save_put(void *data, unsigned int len)

{
	int cpy_len;
	int avail_len;
	uint32_t write_addr;
	struct recordContext *context = &rcdContext;

	if (context->status == 0) {
		return 0;
	}

	avail_len = SAVE_BUFFER_LEN - context->validLen;
	cpy_len = len > avail_len ? avail_len : len;
	write_addr = FLASH_START_ADDR + context->validLen;
	HAL_Flash_Write(FLASH_DEVICE_NUM, write_addr, (uint8_t *)data, cpy_len);
	context->validLen += cpy_len;
	if (context->validLen == SAVE_BUFFER_LEN) {
		context->status = 0;
		HAL_Flash_Close(FLASH_DEVICE_NUM);
		printf("===stop save record data===\n");
	}
	return cpy_len;
}

int record_data_save_stop()
{
	struct recordContext *context = &rcdContext;

	if (context->status == 1) {
		HAL_Flash_Close(FLASH_DEVICE_NUM);
		context->status = 0;
		printf("===stop save record data===\n");
	}
	return 0;
}

#endif
#endif

#ifdef RUNNING_TIME_DEBUG

#include "driver/chip/hal_rtc.h"
#include <stdio.h>

#define STATISTICS_TIME 1000

struct timeContext {
	uint64_t start;
	uint64_t stop;
	uint64_t total;
	uint32_t count;
};

static struct timeContext tmContext;

int start_running()
{
	tmContext.start = HAL_RTC_GetFreeRunTime();
	return 0;
}

int stop_running()
{
	tmContext.stop = HAL_RTC_GetFreeRunTime();
	tmContext.count++;
	tmContext.total += tmContext.stop - tmContext.start;
	if (tmContext.count == STATISTICS_TIME) {
		printf("%uus each time\n", (int)tmContext.total / STATISTICS_TIME);
		tmContext.count = 0;
		tmContext.total = 0;
	}
	return 0;
}

#endif

