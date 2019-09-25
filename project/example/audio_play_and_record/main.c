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
#include "common/framework/platform_init.h"
#include "audio_play.h"
#include "audio_record.h"

static int get_group_number(int rate)
{
	int group_num;

	switch (rate) {
	case 8000:
	case 12000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
		group_num = 1;
		printf("sample rate %d belong to group 1\n", rate);
		break;
	case 11025:
	case 22050:
	case 44100:
		group_num = 2;
		printf("sample rate %d belong to group 2\n", rate);
		break;
	default:
		group_num = 0;
		printf("invalid sample rate %d\n", rate);
		break;
	}
	return group_num;
}

static int check_sample_rate(int record_rate, int play_rate)
{
	int record_num;
	int play_num;

	record_num = get_group_number(record_rate);
	play_num = get_group_number(play_rate);

	if ((record_num == 0) || (play_num == 0)) {
		printf("invalid sample rate, record_rate:%d, play_rate:%d\n", record_rate, play_rate);
		return -1;
	}

	if (record_num != play_num) {
		printf("sample rate of record and sample rate of play should belong to the same group.\n");
		return -1;
	}
	printf("it's ok to use %d to record, use %d to play\n", record_rate, play_rate);
	return 0;
}

int main(void)
{
	platform_init();

	printf("audio play + record start.\n");

	check_sample_rate(SAMPLE_RATE_OF_RECORD, SAMPLE_RATE_OF_PLAY);

	audio_play_start();

	audio_record_start();

	return 0;
}
