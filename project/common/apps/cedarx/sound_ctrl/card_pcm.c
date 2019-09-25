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
#include "audio/pcm/audio_pcm.h"
#include "audio/manager/audio_manager.h"
#include "soundStream.h"

#define SUPPORT_FIXED_OUTPUT_CONFIG

#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
#include "audio/reverb/mixer.h"
#include "audio/reverb/resample.h"
#endif

#define AUDIO_PERIOD_SIZE           1024
#define AUDIO_PERIOD_COUNT          2

struct CardPcmConfig {
    unsigned int  channels;
    unsigned int  rate;
};

typedef struct CardContext {
    SoundStreamT base;
    struct pcm_config input_config;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    struct CardPcmConfig output_cfg;
    struct pcm_config *output_config;
    resample_info res_info;
    unsigned int rate;
    unsigned int channel;
#endif
} CardContext;

static int card_pcm_open(SoundStreamT *stream)
{
    CardContext *context = (CardContext *)stream;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    if (context->output_config) {
        if (context->output_cfg.channels == 0) {
            context->output_config->channels = context->input_config.channels;
        }
        if (context->output_cfg.rate == 0) {
            context->output_config->rate = context->input_config.rate;
        }
        return snd_pcm_open(AUDIO_SND_CARD_DEFAULT, PCM_OUT, context->output_config);
    }
#endif
    return snd_pcm_open(AUDIO_SND_CARD_DEFAULT, PCM_OUT, &(context->input_config));
}

static int card_pcm_close(SoundStreamT *stream)
{
    return snd_pcm_close(AUDIO_SND_CARD_DEFAULT, PCM_OUT);
}

static int card_pcm_flush(SoundStreamT *stream)
{
    CardContext *context = (CardContext *)stream;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    if (context->output_config) {
        if (context->output_cfg.channels == 0) {
            context->output_config->channels = context->input_config.channels;
        }
        if (context->output_cfg.rate == 0) {
            context->output_config->rate = context->input_config.rate;
        }
        return snd_pcm_flush(AUDIO_SND_CARD_DEFAULT);
    }
#endif
    return snd_pcm_flush(AUDIO_SND_CARD_DEFAULT);
}

static int card_pcm_write(SoundStreamT *stream, struct SscPcmConfig *config, void *data, unsigned int count)
{
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    int has_resample = 0;
    resample_info *res_info;
#endif
    void *outData = data;
    unsigned int dataLen = count;
    CardContext *context = (CardContext *)stream;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    res_info = &context->res_info;
    if (context->output_config) {
        /* convert channel */
        if ((context->output_cfg.channels == 0) || (context->output_cfg.channels == context->input_config.channels)) {
            context->output_config->channels = context->input_config.channels;
        } else { /* request output channel is not equal to channel of this audio */
            /* infact, we only support convert 2 channels to 1 channels */
            char *src_data[2];
            src_data[0] = (char *)outData;
            src_data[1] = NULL;
            mixer_process(src_data, 2, (char *)outData, dataLen);
            dataLen = dataLen / 2;
        }

        /* convert sample rate */
        if ((context->output_cfg.rate == 0) || (context->output_cfg.rate == context->input_config.rate)) {
            context->output_config->rate = context->input_config.rate;
        } else {
            unsigned int out_channel;
            unsigned int in_rate;

            has_resample = 1;
            out_channel = context->output_config->channels;
            in_rate = context->input_config.rate;
            if (context->channel != out_channel || context->rate != in_rate) {
                res_info->BitsPerSample = 16;
                res_info->in_SampleRate = in_rate;
                res_info->NumChannels = out_channel;
                res_info->out_SampleRate = context->output_config->rate;
                resample_init(res_info);
                context->channel = out_channel;
                context->rate = in_rate;
            }
            resample(res_info, (short*)outData, dataLen);
            outData = res_info->out_buffer;
            dataLen = res_info->out_frame_indeed * (res_info->BitsPerSample / 8) * res_info->NumChannels;
        }

        snd_pcm_write(AUDIO_SND_CARD_DEFAULT, outData, dataLen);

        if (has_resample) {
            resample_release(res_info);
        }
        return count;
    }
#endif
    return snd_pcm_write(AUDIO_SND_CARD_DEFAULT, outData, dataLen);
}

static int card_pcm_read(SoundStreamT *stream, void *data, unsigned int count)
{
    CardContext *context = (CardContext *)stream;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    if (context->output_config) {
        if (context->output_cfg.channels == 0) {
            context->output_config->channels = context->input_config.channels;
        }
        if (context->output_cfg.rate == 0) {
            context->output_config->rate = context->input_config.rate;
        }
        return snd_pcm_read(AUDIO_SND_CARD_DEFAULT, data, count);
    }
#endif
    return snd_pcm_read(AUDIO_SND_CARD_DEFAULT, data, count);
}

static int card_pcm_ioctl(SoundStreamT *stream, SoundStreamCmd cmd, void *param)
{
    struct SscPcmConfig *config;
    CardContext *context = (CardContext *)stream;

    switch (cmd) {
    case STREAM_CMD_SET_CONFIG:
        config = (struct SscPcmConfig *)param;
        context->input_config.channels     = config->channels;
        context->input_config.rate         = config->rate;
        context->input_config.format       = PCM_FORMAT_S16_LE;
        context->input_config.period_count = AUDIO_PERIOD_COUNT;
        context->input_config.period_size  = AUDIO_PERIOD_SIZE;
        break;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    case STREAM_CMD_SET_OUTPUT_CONFIG:
        config = (struct SscPcmConfig *)param;
        if (config->channels > 1) {
            printf("invalid output config. channel:%u\n", config->channels);
            break;
        }

        if (context->output_config == NULL) {
            context->output_config = (struct pcm_config *)malloc(sizeof(struct pcm_config));
            if (context->output_config == NULL) {
                break;
            }
        }
        context->output_cfg.channels = config->channels;
        context->output_cfg.rate     = config->rate;
        context->output_config->channels     = config->channels;
        context->output_config->rate         = config->rate;
        context->output_config->format       = PCM_FORMAT_S16_LE;
        context->output_config->period_count = AUDIO_PERIOD_COUNT;
        context->output_config->period_size  = AUDIO_PERIOD_SIZE;
        break;
    case STREAM_CMD_CLEAR_OUTPUT_CONFIG:
        free(context->output_config);
        context->output_config = NULL;
        break;
#endif
    default:
        break;
    }
    return 0;
}

static const struct SoundStreamOpsS cardStreamOps =
{
    .soundOpen  = card_pcm_open,
    .soundClose = card_pcm_close,
    .soundWrite = card_pcm_write,
    .soundRead  = card_pcm_read,
    .soundFlush = card_pcm_flush,
    .soundIoctl = card_pcm_ioctl,
};

static SoundStreamT * card_pcm_create(void)
{
    CardContext *context;

    context = (CardContext *)malloc(sizeof(CardContext));
    if (context == NULL) {
        return NULL;
    }
    memset(context, 0, sizeof(CardContext));

    context->base.ops = &cardStreamOps;
    return &context->base;
}

static void card_pcm_destroy(SoundStreamT * stream)
{
    CardContext *context = (CardContext *)stream;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    free(context->output_config);
#endif
    free(context);
}

const struct SoundStreamCreatorS CardStreamCtor =
{
    .create  = card_pcm_create,
    .destroy = card_pcm_destroy,
};

#endif