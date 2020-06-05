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
#include "kernel/os/os.h"
#include "common/framework/platform_init.h"
#include "common/framework/fs_ctrl.h"
#include "fs/fatfs/ff.h"
#include "common/apps/player_app.h"
#include "audio/eq/eq.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/manager/audio_manager.h"

#define PLAYER_THREAD_STACK_SIZE    (1024 * 2)
#define PCM_PLAY_EN					1

static OS_Thread_t player_thread;
static eq_core_prms_t core_prms[10] =
{
	{0, 100, 2, HIGHPASS},
	{-1, 200, 1, BANDPASS_PEAK},
	{-3, 400, 1, BANDPASS_PEAK},
	{3, 800, 2, BANDPASS_PEAK},
	{0, 1000, 1, BANDPASS_PEAK},
	{5, 2000, 1, BANDPASS_PEAK},
	{-3, 4000, 2, BANDPASS_PEAK},
	{5, 8000, 1, BANDPASS_PEAK},
	{0, 12000, 1, BANDPASS_PEAK},
	{0, 16000, 1, BANDPASS_PEAK},
};

static int fs_init()
{
	if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
		printf("mount fail\n");
		return -1;
	}

	printf("mount success\n");

	return 0;
}

static int fs_deinit()
{
	if (fs_ctrl_unmount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
		printf("unmount fail\n");
		return -1;
	}
	printf("\nunmount success\n");

	return 0;
}

#if PCM_PLAY_EN

#define PCM_INT_URL		"eq_test/eq_input.pcm"

static struct pcm_config config = {
	.rate = 44100,
	.channels = 1,
	.period_size = 1024,
	.period_count = 2,
	.format = PCM_FORMAT_S16_LE,
};

static int music_pcm_play()
{
	void* equalizer;

	int res;
	FIL fin;
	uint32_t br;

	res = f_open(&fin, PCM_INT_URL, FA_READ|FA_OPEN_EXISTING);
	if (res != FR_OK) {
		printf("open file error %d\n", res);
		return -1;
	}

	uint32_t pcm_buf_size = (config.channels)*2*(config.period_size);
	char *pcm_data = malloc(pcm_buf_size);

	eq_prms_t prms;
	prms.biq_num = 10;
	prms.chan = config.channels;
	prms.sampling_rate = config.rate;
	prms.core_prms = core_prms;
	equalizer = eq_create(&prms);
	if (equalizer == NULL) {
		printf("create equalizer handle error!\n");
		return -1;
	}

	if (snd_pcm_open(AUDIO_SND_CARD_DEFAULT, PCM_OUT, &config) != 0) {
		printf("sound card open err\n");
		return -1;
    }
	audio_manager_handler(AUDIO_SND_CARD_DEFAULT, AUDIO_MANAGER_SET_VOLUME_LEVEL, AUDIO_OUT_DEV_SPK, 24);

	uint32_t radio = (config.channels > 1 ) ? 4 : 2;
	while (1) {
		res = f_read(&fin, pcm_data, pcm_buf_size, &br);
		if (res != FR_OK) {
			printf("read fail, %d\n", res);
			break;
		}
		eq_process(equalizer, (short*)pcm_data, br/radio);
		snd_pcm_write(AUDIO_SND_CARD_DEFAULT, pcm_data, br);
		if (br < pcm_buf_size) {
			printf("read not enough\n");
			break;
		}
	}
	snd_pcm_flush(AUDIO_SND_CARD_DEFAULT);
    snd_pcm_close(AUDIO_SND_CARD_DEFAULT, PCM_OUT);
	eq_destroy(equalizer);
	f_close(&fin);
	return 0;
}

#else
#define MUSIC_URL	"file://eq_test/1.mp3"

static OS_Semaphore_t sem;
static player_base *player;

static void music_task_callback(player_events event, void *data, void *arg)
{
	switch (event) {
	case PLAYER_EVENTS_MEDIA_PREPARED:
		break;
	case PLAYER_EVENTS_MEDIA_STOPPED:
		break;
	case PLAYER_EVENTS_MEDIA_ERROR:
		printf("error occur\n");
		OS_SemaphoreRelease(&sem);
		break;
	case PLAYER_EVENTS_MEDIA_PLAYBACK_COMPLETE:
		printf("media play is complete\n");
		OS_SemaphoreRelease(&sem);
		break;
	default:
		break;
	}
}

static int music_player_play()
{
	OS_Status ret;
	eq_prms_t prm_config;

	ret = OS_SemaphoreCreate(&sem, 0, OS_SEMAPHORE_MAX_COUNT);
	if (ret != OS_OK) {
		printf("sem create fail\n");
		return -1;
	}

	player = player_create();
	if (player == NULL) {
		printf("player create fail.\n");
		OS_SemaphoreDelete(&sem);
		return -1;
	}

	prm_config.core_prms = core_prms;
	prm_config.biq_num = 10;
	player->control(player, PLAYER_CMD_SET_EQ_MODE, &prm_config);
	player->setvol(player, 24);
	player->set_callback(player, music_task_callback, NULL);
	player->play(player, MUSIC_URL);
	OS_SemaphoreWait(&sem, OS_WAIT_FOREVER);
	player->stop(player);

	OS_SemaphoreDelete(&sem);
	player_destroy(player);
	return 0;
}

void music_eq_stop()
{
	if (player)
		player->control(player, PLAYER_CMD_CLEAR_EQ_MODE, NULL);
}

void music_eq_start()
{
	eq_prms_t prm_config;
	prm_config.core_prms = core_prms;
	prm_config.biq_num = 10;

	if (player)
		player->control(player, PLAYER_CMD_SET_EQ_MODE, &prm_config);
}

void music_player_pause()
{
	if (player)
		player->pause(player);
}

void music_eq_resume()
{
	if (player)
		player->resume(player);
}

#endif

static void play_task(void *arg)
{
	fs_init();

#if PCM_PLAY_EN
	music_pcm_play();
#else
	music_player_play();
#endif

	fs_deinit();

	OS_ThreadDelete(&player_thread);
}

int music_start()
{
	if (OS_ThreadCreate(&player_thread,
                        "player_task",
                        play_task,
                        NULL,
                        OS_THREAD_PRIO_APP,
                        PLAYER_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create fail.exit\n");
		return -1;
	}
	return 0;
}

