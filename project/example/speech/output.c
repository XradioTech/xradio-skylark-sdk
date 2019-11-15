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

#include "output.h"

#ifdef SUPPORT_RESULT_OUTPUT

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "kernel/os/os_time.h"
#include "pcmFifo.h"

typedef enum {
	AI_START,
	AI_STOP,
} AI_STATUS;

#define KEYWORD_TO_VAD_THRESHOLD  250
#define MIN_TIME_TO_RESTART       2000
#define MIN_RECORD_TIME           1500
#define MAX_NO_SOUND_TIME         10000
#define MAX_NO_SOUND_TIME2        500

static AI_STATUS ai_status = AI_STOP;
static struct timeval keywordStartTime;
static struct timeval lastVadTime;

extern struct PcmFifoS *pcmFifo;

int result_output_start()
{
	return 0;
}

static int ai_start()
{
	printf("=================start record=================\n");
	ai_status = AI_START;
	gettimeofday(&keywordStartTime, NULL);
	gettimeofday(&lastVadTime, NULL);

	return 0;
}

static int ai_stop()
{
	printf("=================stop  record=================\n");
	ai_status = AI_STOP;

	return 0;
}

static int result_param_solve(struct resultParam *param)
{
	long startDurationMs;
	long keywordToVadDurationMs;
	long noSoundDurationMs;
	struct timeval nowTime;

	if (param->vad == 1) {
		gettimeofday(&lastVadTime, NULL);
	}

	if (ai_status == AI_START) {
		gettimeofday(&nowTime, NULL);
		startDurationMs = (nowTime.tv_sec - keywordStartTime.tv_sec) * 1000 + (nowTime.tv_usec - keywordStartTime.tv_usec) / 1000;
		if (param->keyword) {
			if (startDurationMs > MIN_TIME_TO_RESTART) {
				ai_stop();
				OS_MSleep(100);
				ai_start();
			} else {
				printf("ignore this keyword\n");
			}
		} else {
			keywordToVadDurationMs = (lastVadTime.tv_sec - keywordStartTime.tv_sec) * 1000 + (lastVadTime.tv_usec - keywordStartTime.tv_usec) / 1000;
			noSoundDurationMs = (nowTime.tv_sec - lastVadTime.tv_sec) * 1000 + (nowTime.tv_usec - lastVadTime.tv_usec) / 1000;
			if ((keywordToVadDurationMs <= KEYWORD_TO_VAD_THRESHOLD) && (startDurationMs > MAX_NO_SOUND_TIME)) {
				ai_stop();  /* after keyword, no sound is detected, and duration MAX_NO_SOUND_TIME */
			} else if ((keywordToVadDurationMs > KEYWORD_TO_VAD_THRESHOLD) && (noSoundDurationMs > MAX_NO_SOUND_TIME2) && (startDurationMs > MIN_RECORD_TIME)) {
				ai_stop();
			}
		}
	} else {
		if (param->keyword) {
			ai_start();
		}
	}

	return 0;
}

static int media_data_solve(struct mediaData *mData)
{
	PcmFifoLock(pcmFifo);
	PcmFifoIn(pcmFifo, mData->LOut, mData->loutLen, 1);
	PcmFifoUnlock(pcmFifo);

	return 0;
}

int result_output_data(struct resultParam *param, struct mediaData *mData)
{
	result_param_solve(param);
	media_data_solve(mData);

	return 0;
}

void result_output_stop()
{
	return;
}

#endif

