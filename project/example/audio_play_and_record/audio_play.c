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
#include "audio_play.h"
#include "soundStreamControl.h"

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

    /* wait for playback complete */
    while (!isCompleted) {
        OS_MSleep(100);
    }

    /* stop it */
    player->stop(player);
    return 0;
}

static void play_task(void *arg)
{
    if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
        printf("mount fail\n");
        goto exit;
    }

    printf("create player.\n");
    player = player_create();
    if (player == NULL) {
        printf("player create fail.\n");
        goto exit;
    }

    struct SscPcmConfig config;
    config.channels = 1;
    config.rate = SAMPLE_RATE_OF_PLAY;
    player->control(player, PLAYER_CMD_SET_OUTPUT_CONFIG, &config);

    player->setvol(player, 8);

    while (1) {
        play_file_music();
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

