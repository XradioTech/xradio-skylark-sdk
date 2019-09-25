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
#include <string.h>
#include <stdlib.h>
#include "bgm.h"
#include "kernel/os/os.h"
#include "audio/manager/audio_manager.h"
#include "audio/reverb/mixer.h"
#include "soundStreamControl.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/reverb/reverb.h"

#define KARAOK_THREAD_STACK_SIZE    (2 * 1024)

static OS_Thread_t karaok_thread;
static int karaok_run;

#define AUDIO_CARD                AUDIO_SND_CARD_DEFAULT
#define AUDIO_SAMPLERATE          (16000)
#define AUDIO_CHANNELS            (1)
#define AUDIO_PERIOD_SIZE         (1024)
#define AUDIO_PERIOD_COUNT        (2)

struct recordContext {
	void *buffer;
	unsigned int buf_len;
};

struct reverbContext {
	reverb_info info;
};

struct bgmContext {
	SoundStreamCtrl ssc;
	void *buffer;
	unsigned int buf_len;
};

struct mixerContext {
	void *buffer;
	unsigned int buf_len;
};

struct outputContext {
	int reserve;
};

struct karaokContext {
	struct recordContext record;
	struct reverbContext reverb;
	struct bgmContext bgm;
	struct mixerContext mixer;
	struct outputContext output;
};

static int karaok_record_init(struct recordContext *record)
{
	struct pcm_config config;

	config.channels = AUDIO_CHANNELS;
	config.format = PCM_FORMAT_S16_LE;
	config.period_size = AUDIO_PERIOD_SIZE;
	config.period_count = AUDIO_PERIOD_COUNT;
	config.rate = AUDIO_SAMPLERATE;

	if (snd_pcm_open(AUDIO_CARD, PCM_IN, &config) != 0) {
		printf("snd_pcm_open fail.\n");
		return -1;
	}

	audio_manager_handler(AUDIO_SND_CARD_DEFAULT, AUDIO_MANAGER_SET_VOLUME_LEVEL, AUDIO_IN_DEV_AMIC, 3);

	record->buf_len = AUDIO_CHANNELS * AUDIO_PERIOD_COUNT * AUDIO_PERIOD_SIZE;
	record->buffer = malloc(record->buf_len);
	if (record->buffer == NULL) {
		printf("malloc fail.\n");
		snd_pcm_close(AUDIO_CARD, PCM_IN);
		return -1;
	}
	memset(record->buffer, 0, record->buf_len);

	return 0;
}

static int karaok_record_get_data(struct recordContext *record)
{
	return snd_pcm_read(AUDIO_CARD, record->buffer, record->buf_len);
}

static void karaok_record_deinit(struct recordContext *record)
{
	free(record->buffer);
	snd_pcm_close(AUDIO_CARD, PCM_IN);
}

static int karaok_reverb_init(struct reverbContext *reverb)
{
	int ret;
	float d_time;

	d_time = 0.25f;
	reverb->info.BitsPerSample = 16;
	reverb->info.SampleRate = AUDIO_SAMPLERATE;
	reverb->info.feed_ratio = 3;
	reverb->info.NumChannels = AUDIO_CHANNELS;
	reverb->info.delay_sample = (int)(d_time * AUDIO_SAMPLERATE * AUDIO_CHANNELS);
	ret = reverb_init(&reverb->info);
	if (ret) {
		printf("reverb init fail.\n");
		return -1;
	}

	return 0;
}

static int karaok_reverb_process(struct reverbContext *reverb, void *data, unsigned int len)
{
	reverb_comb(&reverb->info, data, len);
	return 0;
}

static void karaok_reverb_deinit(struct reverbContext *reverb)
{
	reverb_release(&reverb->info);
}

static int karaok_bgm_init(struct bgmContext *bgm)
{
	int block = 1;

	bgm->buf_len = AUDIO_CHANNELS * AUDIO_PERIOD_COUNT * AUDIO_PERIOD_SIZE;
	bgm->buffer = malloc(bgm->buf_len);
	if (bgm->buffer == NULL) {
		printf("malloc fail.\n");
		return -1;
	}

	bgm->ssc = snd_stream_create(STREAM_TYPE_REVERB_PCM);
	if (bgm->ssc == NULL) {
		printf("create STREAM_TYPE_REVERB_PCM fail.\n");
		free(bgm->buffer);
		return -1;
	}

	background_music_pause();

	snd_stream_open(bgm->ssc, STREAM_TYPE_REVERB_PCM);
	snd_stream_control(bgm->ssc, STREAM_TYPE_REVERB_PCM, STREAM_CMD_SET_BLOCK_MODE, &block);

	background_music_resume();

	return 0;
}

static int karaok_bgm_get_data(struct bgmContext *bgm)
{
	memset(bgm->buffer, 0, bgm->buf_len);
	return snd_stream_read(bgm->ssc, STREAM_TYPE_REVERB_PCM, bgm->buffer, bgm->buf_len);
}

static void karaok_bgm_deinit(struct bgmContext *bgm)
{
	int block = 0;

	snd_stream_control(bgm->ssc, STREAM_TYPE_REVERB_PCM, STREAM_CMD_SET_BLOCK_MODE, &block);

	background_music_pause();
	snd_stream_close(bgm->ssc, STREAM_TYPE_REVERB_PCM);
	background_music_resume();

	snd_stream_destroy(bgm->ssc, STREAM_TYPE_REVERB_PCM);
	free(bgm->buffer);
}

static int karaok_mixer_init(struct mixerContext *mixer)
{
	mixer->buf_len = AUDIO_CHANNELS * AUDIO_PERIOD_COUNT * AUDIO_PERIOD_SIZE;
	mixer->buffer = malloc(mixer->buf_len);
	if (mixer->buffer == NULL) {
		printf("malloc fail.\n");
		return -1;
	}
	return 0;
}

static int karaok_mixer_process(struct mixerContext *mixer, void *data1, void *data2)
{
	char *data[2];

	data[0] = (char *)data1;
	data[1] = (char *)data2;
	mixer_process(data, 2, (char*)mixer->buffer, mixer->buf_len);
	return 0;
}

static void karaok_mixer_deinit(struct mixerContext *mixer)
{
	free(mixer->buffer);
}

static int karaok_output_init(struct outputContext *output)
{
	struct pcm_config config;

	config.channels = AUDIO_CHANNELS;
	config.format = PCM_FORMAT_S16_LE;
	config.period_size = AUDIO_PERIOD_SIZE;
	config.period_count = AUDIO_PERIOD_COUNT;
	config.rate = AUDIO_SAMPLERATE;

	if (snd_pcm_open(AUDIO_CARD, PCM_OUT, &config) != 0) {
		printf("snd_pcm_open fail.\n");
		return -1;
	}
	return 0;
}

static int karaok_output_process(struct outputContext *output, void *data, unsigned int len)
{
	return snd_pcm_write(AUDIO_CARD, data, len);
}

static void karaok_output_deinit(struct outputContext *output)
{
	snd_pcm_close(AUDIO_CARD, PCM_OUT);
}

static void karaok_task()
{
	int ret;
	struct karaokContext karaok;
	struct recordContext *record = &karaok.record;
	struct reverbContext *reverb = &karaok.reverb;
	struct bgmContext *bgm = &karaok.bgm;
	struct mixerContext *mixer = &karaok.mixer;
	struct outputContext *output = &karaok.output;

	ret = karaok_record_init(record);
	if (ret) {
		goto exit;
	}

	ret = karaok_reverb_init(reverb);
	if (ret) {
		goto exit1;
	}

	ret = karaok_bgm_init(bgm);
	if (ret) {
		goto exit2;
	}

	ret = karaok_mixer_init(mixer);
	if (ret) {
		goto exit3;
	}

	ret = karaok_output_init(output);
	if (ret) {
		goto exit4;
	}

	while (karaok_run) {
		karaok_record_get_data(record);
		karaok_reverb_process(reverb, record->buffer, record->buf_len);
		karaok_bgm_get_data(bgm);
		karaok_mixer_process(mixer, record->buffer, bgm->buffer);
		karaok_output_process(output, mixer->buffer, mixer->buf_len);
	}

	karaok_output_deinit(output);
exit4:
	karaok_mixer_deinit(mixer);
exit3:
	karaok_bgm_deinit(bgm);
exit2:
	karaok_reverb_deinit(reverb);
exit1:
	karaok_record_deinit(record);
exit:
	OS_ThreadDelete(&karaok_thread);
}

int karaok_start(void)
{
	karaok_run = 1;
	if (OS_ThreadIsValid(&karaok_thread)) {
		printf("task is still running");
		return -1;
	}

	if (OS_ThreadCreate(&karaok_thread,
                        "karaok",
                        karaok_task,
                        NULL,
                        OS_THREAD_PRIO_APP,
                        KARAOK_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create fail.\n");
		return -1;
	}
	return 0;
}

int karaok_stop(void)
{
	karaok_run = 0;
	return 0;
}

