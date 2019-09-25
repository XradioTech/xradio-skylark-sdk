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
#include "kernel/os/os.h"
#include "reverb_buffer.h"
#include "soundStream.h"

#define SUPPORT_FIXED_OUTPUT_CONFIG

#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
#include "audio/reverb/mixer.h"
#include "audio/reverb/resample.h"
#endif

#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
#define DEFAULT_OUTPUT_SAMPLE_RATE       (16000)
#define DEFAULT_OUTPUT_CHANNEL           (1)
#endif

#define RING_BUF_SIZE           (4096)

/* Semaphore */
#define RG_SemaphoreInitBinary(sem) \
    (OS_SemaphoreCreateBinary(sem) == OS_OK ? 0 : -1)

#define RG_SemaphoreDeinit(sem) \
    (OS_SemaphoreDelete(sem) == OS_OK ? 0 : -1)

#define RG_SemaphoreWait(sem) \
    (OS_SemaphoreWait(sem, OS_WAIT_FOREVER) == OS_OK ? 0 : -1)

#define RG_SemaphoreRelease(sem) \
    (OS_SemaphoreRelease(sem) == OS_OK ? 0 : -1)

struct ReverbPcmConfig {
    unsigned int  channels;
    unsigned int  rate;
};

typedef struct ReverbContext {
    SoundStreamT base;
    reverb_buffer* bufferImpl;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    resample_info res_info;
    struct ReverbPcmConfig input_info; /* sample rate and channels of pcm before resample */
    struct ReverbPcmConfig output_info; /* sample rate and channels of pcm after resample */
#endif
    uint8_t put_block;
    uint8_t waitingSem;
    OS_Semaphore_t sem;
} ReverbContext;

static int reverb_pcm_open(SoundStreamT *stream)
{
    return 0;
}

static int reverb_pcm_close(SoundStreamT *stream)
{
    return 0;
}

static int reverb_pcm_write(SoundStreamT *stream, struct SscPcmConfig *config, void *buffer, unsigned int size)
{
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    int has_resample = 0;
    resample_info *res_info;
    uint8_t channel = config->channels;
    uint32_t rate = config->rate;
#endif
    uint8_t *data_ptr = buffer;
    uint32_t count = size;
    uint32_t len = 0;
    ReverbContext *context = (ReverbContext *)stream;

#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    res_info = &context->res_info;

    /* convert channel */
    if ((context->output_info.channels != 0) && (context->output_info.channels != channel)) {
        /* infact, we only support convert 2 channels to 1 channels */
        char *src_data[2];
        src_data[0] = (char *)buffer;
        src_data[1] = NULL;
        mixer_process(src_data, 2, (char *)buffer, count);
        channel = channel / 2 ;
        count = count / 2;
    }

    /* convert sample rate */
    if ((context->output_info.rate != 0) && (context->output_info.rate != rate)) {
        has_resample = 1;

        if (context->input_info.channels != channel || context->input_info.rate != rate) {
            res_info->BitsPerSample = 16;
            res_info->in_SampleRate = rate;
            res_info->NumChannels = channel;
            res_info->out_SampleRate = context->output_info.rate;
            resample_init(res_info);
            context->input_info.channels = channel;
            context->input_info.rate = rate;
        }
        resample(res_info, (short*)data_ptr, count);

        data_ptr = (uint8_t*)res_info->out_buffer;
        count = res_info->out_frame_indeed * (res_info->BitsPerSample / 8) * res_info->NumChannels;
    }
#endif

    while (count) {
        len = reverb_buffer_put(context->bufferImpl, data_ptr, count);
        data_ptr += len;
        count -= len;
        if ((count != 0) && (context->put_block)) {
            context->waitingSem = 1;
            RG_SemaphoreWait(&context->sem);
            context->waitingSem = 0;
        } else {
            break;
        }
    }

#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    if (has_resample) {
        resample_release(res_info);
    }
#endif

    return size;
}

static int reverb_pcm_read(SoundStreamT *stream, void *buffer, unsigned int size)
{
    uint32_t ret;
    ReverbContext *context = (ReverbContext *)stream;

    ret = reverb_buffer_get(context->bufferImpl, buffer, size);

    if (context->waitingSem) {
        RG_SemaphoreRelease(&context->sem);
    }

    return ret;
}

static int reverb_pcm_flush(SoundStreamT *stream)
{
    /* not support now */
    return 0;
}

static int reverb_pcm_ioctl(SoundStreamT *stream, SoundStreamCmd cmd, void *param)
{
    int *block;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    struct SscPcmConfig *config;
#endif
    ReverbContext *context = (ReverbContext *)stream;

    switch (cmd) {
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    case STREAM_CMD_SET_OUTPUT_CONFIG:
        config = (struct SscPcmConfig *)param;
        if (config->channels > 1) {
            printf("invalid output config. channel:%u\n", config->channels);
            break;
        }
        context->output_info.channels = config->channels;
        context->output_info.rate     = config->rate;
        break;
#endif
    case STREAM_CMD_SET_BLOCK_MODE:
        block = param;
        context->put_block = *block;
        if (*block == 0) {
            if (context->waitingSem) {
                RG_SemaphoreRelease(&context->sem);
            }
        }
        break;
    default:
        break;
    }
    return 0;
}

static const struct SoundStreamOpsS reverbStreamOps =
{
    .soundOpen  = reverb_pcm_open,
    .soundClose = reverb_pcm_close,
    .soundWrite = reverb_pcm_write,
    .soundRead  = reverb_pcm_read,
    .soundFlush = reverb_pcm_flush,
    .soundIoctl = reverb_pcm_ioctl,
};

static SoundStreamT * reverb_pcm_create(void)
{
    ReverbContext *context;

    context = (ReverbContext *)malloc(sizeof(ReverbContext));
    if (context == NULL) {
        return NULL;
    }
    memset(context, 0, sizeof(ReverbContext));

    context->base.ops = &reverbStreamOps;
    context->bufferImpl = reverb_buffer_init(RING_BUF_SIZE);
    if (context->bufferImpl == NULL) {
        free(context);
        return NULL;
    }
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    context->output_info.channels = DEFAULT_OUTPUT_CHANNEL;
    context->output_info.rate = DEFAULT_OUTPUT_SAMPLE_RATE;
#endif
    RG_SemaphoreInitBinary(&context->sem);

    return &context->base;
}

static void reverb_pcm_destroy(SoundStreamT *stream)
{
    ReverbContext *context = (ReverbContext *)stream;
    reverb_buffer_free(context->bufferImpl);
    RG_SemaphoreDeinit(&context->sem);
    free(context);
}

const struct SoundStreamCreatorS ReverbStreamCtor =
{
    .create  = reverb_pcm_create,
    .destroy = reverb_pcm_destroy,
};

#endif