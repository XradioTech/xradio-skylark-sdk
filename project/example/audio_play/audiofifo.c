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

#include <stdlib.h>
#include <unistd.h>
#include "string.h"
#include "kfifoqueue.h"
#include "audiofifo.h"
#include "common/apps/player_app.h"

typedef enum audio_status
{
    AUDIO_STATUS_PLAYING,
    AUDIO_STATUS_STOPPED,
    AUDIO_STATUS_ERROR,
} audio_status;

struct AudioFifoImpl {
    struct AudioFifoS base; /* it must be first */
    struct CdxFifoStreamS *fifobase;
    OS_Mutex_t   audio_mutex;
    OS_Thread_t  start_thread;
    audio_status status;
    player_base *player;
    unsigned int in_size;
    char has_play;
    char has_free;
};

static void player_play_callback(player_events event, void *data, void *arg)
{
    struct AudioFifoImpl *impl;
    impl = (struct AudioFifoImpl *)arg;

    printf("%s event:%d\n", __func__, event);

    switch (event) {
    case PLAYER_EVENTS_MEDIA_STOPPED:
    case PLAYER_EVENTS_MEDIA_ERROR:
    case PLAYER_EVENTS_MEDIA_PLAYBACK_COMPLETE:
        impl->status = AUDIO_STATUS_STOPPED;
        break;
    default:
        break;
    }
}

static int player_play_start(struct AudioFifoImpl *impl)
{
    char url[32];

    sprintf(url, "fifo://%p", impl->fifobase);
    impl->player->set_callback(impl->player, player_play_callback, impl);
    return impl->player->play(impl->player, (const char *)url);
}

static int player_play_stop(struct AudioFifoImpl *impl)
{
    impl->player->stop(impl->player);
    return 0;
}

static void audio_play_task(void *arg)
{
    int ret;
    struct AudioFifoImpl *impl;

    impl = (struct AudioFifoImpl *)arg;

    ret = player_play_start(impl);
    if (ret) {
        impl->status = AUDIO_STATUS_ERROR;
    }
    OS_ThreadDelete(&impl->start_thread);
}

static int audio_fifo_set_player(struct AudioFifoS *audiofifo, player_base *player)
{
    struct AudioFifoImpl *impl;

    impl = (struct AudioFifoImpl *)audiofifo;
    impl->player = player;
    return 0;
}

static int audio_fifo_start(struct AudioFifoS *audiofifo)
{
    struct AudioFifoImpl *impl;

    impl = (struct AudioFifoImpl *)audiofifo;

    OS_MutexLock(&impl->audio_mutex, OS_WAIT_FOREVER);
    if (impl->status == AUDIO_STATUS_PLAYING) {
        goto err;
    }

    impl->fifobase = kfifo_stream_create();
    if (impl->fifobase == NULL) {
        goto err;
    }

    impl->status = AUDIO_STATUS_PLAYING;
    impl->has_play = 0;
    impl->in_size = 0;
    impl->has_free = 0;
    OS_MutexUnlock(&impl->audio_mutex);

    return 0;

err:
    OS_MutexUnlock(&impl->audio_mutex);
    return -1;
}

static int audio_fifo_put_data(struct AudioFifoS *audiofifo, void *inData, int dataLen)
{
    OS_Status ret;
    uint32_t avail;
    uint32_t in_len;
    uint32_t reserve_len;
    uint32_t has_in_len = 0;
    struct AudioFifoImpl *impl;

    impl = (struct AudioFifoImpl *)audiofifo;

    OS_MutexLock(&impl->audio_mutex, OS_WAIT_FOREVER);
    if (impl->status != AUDIO_STATUS_PLAYING) {
        OS_MutexUnlock(&impl->audio_mutex);
        return 0;
    }

    while ((has_in_len != dataLen) && (impl->status == AUDIO_STATUS_PLAYING)) {
        reserve_len = dataLen - has_in_len;
        CdxFifoStreamLock(impl->fifobase);
        avail = CdxFifoStreamAvail(impl->fifobase);
        in_len = avail > reserve_len ? reserve_len : avail;
        CdxFifoStreamIn(impl->fifobase, (char *)inData + has_in_len, in_len);
        has_in_len += in_len;
        impl->in_size += in_len;
        CdxFifoStreamUnlock(impl->fifobase);
        if ((impl->in_size >= (4 * 1024)) && (impl->has_play == 0)) {  /* it must be 4k */
            ret = OS_ThreadCreate(&impl->start_thread, "", audio_play_task, impl, OS_THREAD_PRIO_APP, 1024);
            if (ret != OS_OK) {
                impl->status = AUDIO_STATUS_ERROR;
            }
            impl->has_play = 1;
        }
        if (has_in_len != dataLen)
            OS_MSleep(10);
    }
    OS_MutexUnlock(&impl->audio_mutex);
    return has_in_len;
}

static int audio_fifo_stop(struct AudioFifoS *audiofifo, bool stop_immediately)
{
    int ret;
    unsigned int waittime = 0;
    struct AudioFifoImpl *impl;

    impl = (struct AudioFifoImpl *)audiofifo;

    OS_MutexLock(&impl->audio_mutex, OS_WAIT_FOREVER);
    if (impl->status != AUDIO_STATUS_PLAYING) {
        goto err;
    }

    CdxFifoStreamSeteos(impl->fifobase);

    if (stop_immediately) {
        printf("stop immediately\n");
        player_play_stop(impl);
    } else {
        if (impl->has_play == 0) {
            ret = player_play_start(impl);
            if (ret) {
                impl->status = AUDIO_STATUS_ERROR;
            }
        }
        while ((impl->status == AUDIO_STATUS_PLAYING) && (CdxFifoStreamValid(impl->fifobase) != 0))
            OS_MSleep(100);
        while ((impl->status == AUDIO_STATUS_PLAYING) && (waittime < 5000)) {
            OS_MSleep(200);
            waittime += 200;
        }
        if (impl->status == AUDIO_STATUS_PLAYING)
            player_play_stop(impl);
    }

err:
    if (!impl->has_free) {
        impl->status = AUDIO_STATUS_STOPPED;
        impl->has_free = 1;
        kfifo_stream_destroy(impl->fifobase);
    }
    OS_MutexUnlock(&impl->audio_mutex);

    return 0;
}

static const struct AudioFifoOpsS AudioFifoOps = {
    .set_player = audio_fifo_set_player,
    .start      = audio_fifo_start,
    .put_data   = audio_fifo_put_data,
    .stop       = audio_fifo_stop,
};

struct AudioFifoS *audio_fifo_create()
{
    struct AudioFifoImpl *impl;

    impl = malloc(sizeof(*impl));
    if (impl == NULL) {
        printf("AudioFifo create fail\n");
        return NULL;
    }
    memset(impl, 0, sizeof(*impl));

    impl->status = AUDIO_STATUS_STOPPED;
    impl->has_free = 1;
    impl->base.ops = &AudioFifoOps;
    if (OS_MutexCreate(&impl->audio_mutex) != OS_OK) {
        free(impl);
        return NULL;
    }

    return &impl->base;
}

int audio_fifo_destroy(struct AudioFifoS *audiofifo)
{
    struct AudioFifoImpl *impl;

    impl = (struct AudioFifoImpl *)audiofifo;

    OS_MutexDelete(&impl->audio_mutex);
    free(impl);

    return 0;
}

