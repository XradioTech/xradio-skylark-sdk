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

#ifdef __CONFIG_XPLAYER

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "list.h"
#include "sound_log.h"
#include "soundStreamControl.h"
#include "kernel/os/os_mutex.h"

#define SSC_ASSERT(x)          do {                   \
                                   if (!x) {          \
                                       printf("sound stream control assertion!<%s, %d>\n", __func__, __LINE__);    \
                                       return -1;     \
                                   }                  \
                               } while (0)            \

#define SSC_ASSERT2(x)         do {                   \
                                   if (!x) {          \
                                       printf("sound stream control assertion!<%s, %d>\n", __func__, __LINE__);    \
                                       return;        \
                                   }                  \
                               } while (0)            \


typedef struct SoundStreamContext {
    struct SscPcmConfig config;
    OS_Mutex_t mutex;
    SoundStreamT *stream;
} SoundStreamContext;

static SoundStreamContext *stream_context = NULL;

struct SoundStreamListS
{
    ListT list;
    int size;
};

static struct SoundStreamListS streamList;

struct SoundStreamNodeS
{
    ListNodeT node;
    SoundStreamType type;
    SoundStreamPriority priority;
    const struct SoundStreamCreatorS *creator;
    SoundStreamT *stream;
    int create_refs;
    int open_refs;
};

int SoundStreamListInit(void)
{
    ListInit(&streamList.list);
    streamList.size = 0;
    return 0;
}

static int SoundStreamRegister(const void *creator, SoundStreamType type, SoundStreamPriority priority)
{
    struct SoundStreamNodeS *streamNode;

    streamNode = malloc(sizeof(*streamNode));
    memset(streamNode, 0, sizeof(*streamNode));
    streamNode->creator = (const struct SoundStreamCreatorS *)creator;
    streamNode->type = type;
    streamNode->priority = priority;

    ListAddTail(&streamNode->node, &streamList.list);
    streamList.size++;
    return 0;
}

extern const struct SoundStreamCreatorS CardStreamCtor;
extern const struct SoundStreamCreatorS ReverbStreamCtor;

int SoundStreamRegisterCard(void)
{
    return SoundStreamRegister(&CardStreamCtor, STREAM_TYPE_SOUND_CARD, STREAM_PRIORITY_LEVEL1);
}

int SoundStreamRegisterReverb(void)
{
    return SoundStreamRegister(&ReverbStreamCtor, STREAM_TYPE_REVERB_PCM, STREAM_PRIORITY_LEVEL2);
}

static struct SoundStreamNodeS *FindStreamNodeByType(SoundStreamType type)
{
    struct SoundStreamNodeS *streamNode;
    ListForEachEntry(streamNode, &streamList.list, node) {
        if (streamNode->type == type) {
            return streamNode;
        }
    }
    return NULL;
}

static int FindCreateRefsByType(SoundStreamType type)
{
    struct SoundStreamNodeS *streamNode;
    ListForEachEntry(streamNode, &streamList.list, node) {
        if (streamNode->type == type) {
            return streamNode->create_refs;
        }
    }
    return 0;
}

static int FindOpenRefsByType(SoundStreamType type)
{
    struct SoundStreamNodeS *streamNode;
    ListForEachEntry(streamNode, &streamList.list, node) {
        if (streamNode->type == type) {
            return streamNode->open_refs;
        }
    }
    return 0;
}

static int AllSoundStreamIsDestroyed()
{
    struct SoundStreamNodeS *streamNode;
    ListForEachEntry(streamNode, &streamList.list, node) {
        if (streamNode->create_refs != 0) {
            return 0;
        }
    }
    return 1;
}

static SoundStreamT *FindCurrentSoundStream(void)
{
    SoundStreamT *stream = NULL;
    SoundStreamPriority priority = 0;
    struct SoundStreamNodeS *streamNode;

    ListForEachEntry(streamNode, &streamList.list, node) {
        if (streamNode->open_refs != 0) {
            if (streamNode->priority >= priority) {
                priority = streamNode->priority;
                stream = streamNode->stream;
            }
        }
    }

    return stream;
}

static SoundStreamPriority FindCurrentSoundStreamPriority(void)
{
    SoundStreamPriority priority = 0;
    struct SoundStreamNodeS *streamNode;

    ListForEachEntry(streamNode, &streamList.list, node) {
        if (streamNode->open_refs != 0) {
            if (streamNode->priority >= priority) {
                priority = streamNode->priority;
            }
        }
    }

    return priority;
}

static int SoundStreamCreate(SoundStreamType type)
{
    struct SoundStreamNodeS *streamNode;

    streamNode = FindStreamNodeByType(type);
    if (streamNode == NULL) {
        SND_LOGE("unsupport type(%d)\n", type);
        return -1;
    }

    streamNode->create_refs++;
    if (streamNode->create_refs > 1) {
        return 0; /* there is no need to create it more than one time */
    }

    streamNode->stream = streamNode->creator->create();
    if (streamNode->stream == NULL) {
        SND_LOGE("create stream fail, type(%d)\n", type);
        streamNode->create_refs--;
        return -1;
    }

    return 0;
}

static int SoundStreamDestroy(SoundStreamType type)
{
    struct SoundStreamNodeS *streamNode;

    streamNode = FindStreamNodeByType(type);
    if (streamNode == NULL) {
        SND_LOGE("unsupport type(%d)\n", type);
        return -1;
    }

    streamNode->create_refs--;
    if (streamNode->create_refs != 0) {
        return 0;
    }

    streamNode->creator->destroy(streamNode->stream);
    streamNode->stream = NULL;

    return 0;
}

static int SoundStreamSelectOpen(SoundStreamType type)
{
    int ret;
    SoundStreamT *stream;
    SoundStreamPriority priority;
    struct SoundStreamNodeS *streamNode;

    stream = FindCurrentSoundStream();
    priority = FindCurrentSoundStreamPriority();

    streamNode = FindStreamNodeByType(type);
    if (streamNode == NULL) {
        SND_LOGE("unsupport type(%d)\n", type);
        return -1;
    }

    streamNode->open_refs++;
    if (streamNode->open_refs > 1) {
        return 0;
    }

    /*
     * if priority of new stream is lower than priority of current stream,
     * we do nothing just return
     * else, we will open new stream, and close current stream.
     */
    if (streamNode->priority < priority) {
        return 0;
    }

    ret = SoundStreamOpen(streamNode->stream);
    if (ret) {
        streamNode->open_refs--;
        SND_LOGE("open sound stream fail. type(%d)\n", type);
        return -1;
    }

    if (stream) {
        SoundStreamClose(stream);
    }

    return 0;
}

static int SoundStreamSelectClose(SoundStreamType type)
{
    SoundStreamT *stream;
    SoundStreamPriority priority;
    struct SoundStreamNodeS *streamNode;

    streamNode = FindStreamNodeByType(type);
    if (streamNode == NULL) {
        SND_LOGE("unsupport type(%d)\n", type);
        return -1;
    }

    streamNode->open_refs--;
    if (streamNode->open_refs != 0) {
        return 0;
    }

    priority = FindCurrentSoundStreamPriority();

    /* if priority of closing stream is not highest in opened list,
     * we will do nothing just return
     * else, we will close this stream, and find a stream to open if we can.
     */
    if (streamNode->priority < priority) {
        return 0; /* we will not close this stream, because this stream is not using, which mean not opened */
    }

    SoundStreamClose(streamNode->stream);

    stream = FindCurrentSoundStream();
    if (stream) {
        SoundStreamOpen(stream);
    }

    return 0;
}

static int SoundStreamControl(SoundStreamType type, SoundStreamCmd cmd, void *param)
{
    struct SoundStreamNodeS *streamNode;

    streamNode = FindStreamNodeByType(type);

    return SoundStreamIoctl(streamNode->stream, cmd, param);
}

static SoundStreamContext *snd_stream_context_create()
{
    SoundStreamContext *context;

    context = (SoundStreamContext *)malloc(sizeof(SoundStreamContext));
    if (context == NULL) {
        SND_LOGE("malloc fail.\n");
        return NULL;
    }
    memset(context, 0, sizeof(SoundStreamContext));
    OS_RecursiveMutexCreate(&context->mutex);
    stream_context = context;
    return context;
}

static void snd_stream_context_destroy(SoundStreamContext *context)
{
    OS_RecursiveMutexDelete(&context->mutex);
    free(context);
    stream_context = NULL;
}

SoundStreamCtrl snd_stream_create(SoundStreamType type)
{
    int ret;
    SoundStreamContext *context = stream_context;

    if (AllSoundStreamIsDestroyed()) {
        context = snd_stream_context_create();
        if (context == NULL) {
            return NULL;
        }
    }

    ret = SoundStreamCreate(type);

    if (ret && AllSoundStreamIsDestroyed()) {
        snd_stream_context_destroy(context);
        context = NULL;
    }

    return (SoundStreamCtrl)context;
}

void snd_stream_destroy(SoundStreamCtrl ssc, SoundStreamType type)
{
    SoundStreamContext *context = stream_context;

    SSC_ASSERT2(FindCreateRefsByType(type) != 0);

    SoundStreamDestroy(type);

    if (AllSoundStreamIsDestroyed()) {
        snd_stream_context_destroy(context);
    }
}

int snd_stream_open(SoundStreamCtrl ssc, SoundStreamType type)
{
    int ret = 0;
    SoundStreamContext *context = ssc;

    SSC_ASSERT(FindCreateRefsByType(type) != 0);

    OS_RecursiveMutexLock(&context->mutex, OS_WAIT_FOREVER);

    ret = SoundStreamSelectOpen(type);
    context->stream = FindCurrentSoundStream();

    OS_RecursiveMutexUnlock(&context->mutex);
    return ret;
}

int snd_stream_close(SoundStreamCtrl ssc, SoundStreamType type)
{
    int ret = 0;
    SoundStreamContext *context = ssc;

    SSC_ASSERT(FindOpenRefsByType(type) != 0);

    OS_RecursiveMutexLock(&context->mutex, OS_WAIT_FOREVER);

    ret = SoundStreamSelectClose(type);
    context->stream = FindCurrentSoundStream();

    OS_RecursiveMutexUnlock(&context->mutex);
    return ret;
}

int snd_stream_flush(SoundStreamCtrl ssc, SoundStreamType type)
{
    int ret = 0;
    SoundStreamContext *context = ssc;

    SSC_ASSERT(FindOpenRefsByType(type) != 0);

    OS_RecursiveMutexLock(&context->mutex, OS_WAIT_FOREVER);
    ret = SoundStreamFlush(context->stream);
    OS_RecursiveMutexUnlock(&context->mutex);
    return ret;
}

int snd_stream_write(SoundStreamCtrl ssc, SoundStreamType type, void* pData, int nDataSize)
{
    int ret = 0;
    SoundStreamContext *context = ssc;

    SSC_ASSERT(FindOpenRefsByType(type) != 0);

    /* can not lock SoundStreamWrite, because it may block, which case dead lock */
    ret = SoundStreamWrite(context->stream, &(context->config), pData, nDataSize);
    return ret;
}

int snd_stream_read(SoundStreamCtrl ssc, SoundStreamType type, void* pData, int nDataSize)
{
    int ret = 0;
    SoundStreamContext *context = ssc;

    SSC_ASSERT(FindOpenRefsByType(type) != 0);

    OS_RecursiveMutexLock(&context->mutex, OS_WAIT_FOREVER);
    ret = SoundStreamRead(context->stream, pData, nDataSize);
    OS_RecursiveMutexUnlock(&context->mutex);

    return ret;
}

int snd_stream_control(SoundStreamCtrl ssc, SoundStreamType type, SoundStreamCmd cmd, void *param)
{
    SoundStreamContext *context = ssc;

    SSC_ASSERT(FindCreateRefsByType(type) != 0);

    OS_RecursiveMutexLock(&context->mutex, OS_WAIT_FOREVER);

    if (cmd == STREAM_CMD_SET_CONFIG) {
        struct SscPcmConfig *config = (struct SscPcmConfig *)param;
        context->config.channels = config->channels;
        context->config.rate = config->rate;
    }

    SoundStreamControl(type, cmd, param);

    OS_RecursiveMutexUnlock(&context->mutex);
    return 0;
}

#endif