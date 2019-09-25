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
#include "common/framework/fs_ctrl.h"
#include "common/apps/player_app.h"
#include "common/framework/platform_init.h"
#include "fs/fatfs/ff.h"
#include "audiofifo.h"

#define PLAYER_THREAD_STACK_SIZE    (1024 * 2)

static OS_Thread_t play_thread;
static player_base *player;
static int isCompleted = 0;

static void player_demo_callback(player_events event, void *data, void *arg)
{
    switch (event) {
    case PLAYER_EVENTS_MEDIA_PREPARED:
        printf("media is prepared. play media now.\n");
        printf("you can use player to seek, get duration(by size), get current position(by tell) from now on.\n");
        break;
    case PLAYER_EVENTS_MEDIA_STOPPED:
        printf("media is stopped by user.\n");
        isCompleted = 1;
        break;
    case PLAYER_EVENTS_MEDIA_ERROR:
        printf("error occur\n");
        isCompleted = 1;
        break;
    case PLAYER_EVENTS_MEDIA_PLAYBACK_COMPLETE:
        printf("media play is completed\n");
        isCompleted = 1;
        break;
    default:
        break;
    }
}

static int play_file_music()
{
    int ret;
    int duration;
    int position;

    /*
     * play media in sd/tf card.
     * 1. in this example, you should create a folder named music in you sd/tf card
     * 2. add 1.mp3 to this folder
     */
    isCompleted = 0;
    player->set_callback(player, player_demo_callback, NULL);
    ret = player->play(player, "file://music/1.mp3");
    if (ret != 0) {
        printf("music play fail.\n");
        return -1;
    }

    OS_Sleep(15);
    printf("music pause now, last for 10s\n");
    player->pause(player);
    OS_Sleep(10);

    printf("music resume now.\n");
    player->resume(player);

    OS_Sleep(20);
    duration = player->size(player);
    position = player->tell(player);
    printf("this song is %dms in length, and it has play %dms\n", duration, position);

    printf("we will seek to the half of this song now.\n");
    player->seek(player, duration / 2);

    position = player->tell(player);
    printf("now this song has play %dms\n", position);

    /* wait for playback complete */
    while (!isCompleted) {
        OS_MSleep(100);
    }

    /* stop it */
    player->stop(player);
    return 0;
}

static int play_flash_music()
{
    int ret;

    /*
     * play media in flash.
     * 1. in this example, we play 1.amr(in image/xr872)
     * 2. we should add 1.amr to our image.cfg, so we can write it to flash
     * 3. construct the url according to the offset and size of 1.amr.
     *    in this example, the offset of 1.amr is (1024 * 1024 + 64) = 1048640,
     *    and the size of 1.amr is 363750 bytes. so url is "flash://0?addr=1048640&length=363750"
     * be attention, we will add 64 bytes before the bin in image.cfg, so the offset of 1.amr is (1024 * 1024 + 64)
     */
    isCompleted = 0;
    player->set_callback(player, player_demo_callback, NULL);
    ret = player->play(player, "flash://0?addr=1048640&length=363750");
    if (ret != 0) {
        printf("music play fail.\n");
        return -1;
    }

    /* wait for playback complete */
    while (!isCompleted) {
        OS_MSleep(100);
    }

    /* stop it */
    player->stop(player);
    return 0;
}

static int play_fifo_music()
{
    FIL fp;
    int ret = 0;
    FRESULT result;
    void *file_buffer;
    unsigned int act_read;
    struct AudioFifoS *audiofifo;

    /*
     * play media by putting media data to player
     * 1. only support mp3/amr/wav
     */
    audiofifo = audio_fifo_create();
    if (audiofifo == NULL) {
        return -1;
    }

    file_buffer = (void *)malloc(1024);
    if (file_buffer == NULL) {
        ret = -1;
        goto err1;
    }

    result = f_open(&fp, "music/1.mp3", FA_READ);
    if (result != FR_OK) {
        ret = -1;
        goto err2;
    }

    AudioFifoSetPlayer(audiofifo, player);
    AudioFifoStart(audiofifo);
    while (1) {
        f_read(&fp, file_buffer, 1024, &act_read);
        AudioFifoPutData(audiofifo, file_buffer, act_read);
        if (act_read != 1024)
            break;
    }
    AudioFifoStop(audiofifo, false);

    f_close(&fp);
err2:
    free(file_buffer);
err1:
    audio_fifo_destroy(audiofifo);
    return ret;
}

static void play_task(void *arg)
{
    if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
        printf("mount fail\n");
        goto exit;
    }

    player = player_create();
    if (player == NULL) {
        printf("player create fail.\n");
        goto exit;
    }

    printf("player create success.\n");
    printf("you can use it to play, pause, resume, set volume and so on.\n");

    printf("player set volume to 8. valid volume value is from 0~31\n");
    player->setvol(player, 8);

    while (1) {
        printf("===try to play media in sd/tf card===\n");
        play_file_music();

        printf("===try to play media in flash===\n");
        play_flash_music();

        printf("===try to play media by fifo===\n");
        play_fifo_music();
    }

    player_destroy(player);

exit:
    OS_ThreadDelete(&play_thread);
}

int audio_play_start()
{
    if (OS_ThreadCreate(&play_thread,
                        "play_task",
                        play_task,
                        NULL,
                        OS_THREAD_PRIO_APP,
                        PLAYER_THREAD_STACK_SIZE) != OS_OK) {
        printf("thread create fail.exit\n");
        return -1;
    }
    return 0;
}

