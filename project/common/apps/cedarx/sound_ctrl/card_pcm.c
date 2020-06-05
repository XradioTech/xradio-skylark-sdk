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

#define SUPPORT_EQ
#ifdef SUPPORT_EQ
#include "kernel/os/os_mutex.h"
#include "audio/eq/eq.h"
#endif

#define AUDIO_PERIOD_SIZE           1024
#define AUDIO_PERIOD_COUNT          2

#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
/* 8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000 */
#define SAMPLE_RATE_MAX_NUM         9
struct CardPcmConfig {
    unsigned int  channels;
    unsigned int  rate;
    unsigned int  valid;
};
#endif

typedef struct CardContext {
    SoundStreamT base;
    struct pcm_config input_config;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    struct CardPcmConfig support_cfg[SAMPLE_RATE_MAX_NUM + 1];
    struct pcm_config *output_config;
    resample_info res_info;
    unsigned int rate;
    unsigned int channel;
#endif
#ifdef SUPPORT_EQ
    eq_prms_t prms_config;
    unsigned int radio;
    void* equalizer;
    OS_Mutex_t eq_lock;
#endif
} CardContext;

static int card_pcm_open(SoundStreamT *stream)
{
    CardContext *context = (CardContext *)stream;
#ifdef SUPPORT_EQ
    if (context->equalizer) {
        uint32_t chan, sampleRate;
        chan = context->input_config.channels;
        sampleRate = context->input_config.rate;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
        if (context->output_config) {
            chan = context->output_config->channels;
            sampleRate = context->output_config->rate;
        }
#endif
        if (context->prms_config.chan != chan ||
                context->prms_config.sampling_rate != sampleRate) {
            context->prms_config.chan = chan;
            context->prms_config.sampling_rate = sampleRate;
            context->radio = (context->prms_config.chan > 1) ? 4 : 2;
            eq_destroy(context->equalizer);
            context->equalizer = eq_create(&context->prms_config);
            if (context->equalizer == NULL) {
                printf("eq create fail.\n");
            }
        }
    }
#endif
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    if (context->output_config) {
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
#if (defined(SUPPORT_FIXED_OUTPUT_CONFIG) || defined(SUPPORT_EQ))
    CardContext *context = (CardContext *)stream;
#endif

#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    res_info = &context->res_info;
    if (context->output_config) {
        /* convert channel */
        if (context->output_config->channels != context->input_config.channels) {
            /* request output channel is not equal to channel of this audio */
            /* infact, we only support convert 2 channels to 1 channels */
            char *src_data[2];
            src_data[0] = (char *)outData;
            src_data[1] = NULL;
            mixer_process(src_data, 2, (char *)outData, dataLen);
            dataLen = dataLen / 2;
        }

        /* convert sample rate */
        if (context->output_config->rate != context->input_config.rate) {
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

#ifdef SUPPORT_EQ
        if (context->equalizer) {
            OS_MutexLock(&context->eq_lock, OS_WAIT_FOREVER);
            eq_process(context->equalizer, (short*)outData, dataLen/context->radio);
            OS_MutexUnlock(&context->eq_lock);
        }
#endif
        snd_pcm_write(AUDIO_SND_CARD_DEFAULT, outData, dataLen);

        if (has_resample) {
            resample_release(res_info);
        }
        return count;
    }
#endif
#ifdef SUPPORT_EQ
    if (context->equalizer) {
        OS_MutexLock(&context->eq_lock, OS_WAIT_FOREVER);
        eq_process(context->equalizer, (short*)outData, dataLen/context->radio);
        OS_MutexUnlock(&context->eq_lock);
    }
#endif
    return snd_pcm_write(AUDIO_SND_CARD_DEFAULT, outData, dataLen);
}

static int card_pcm_read(SoundStreamT *stream, void *data, unsigned int count)
{
    return snd_pcm_read(AUDIO_SND_CARD_DEFAULT, data, count);
}

#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
static int check_config_param(CardContext *context, struct SscPcmConfig *config)
{
    int i;

    if (config->channels > 1) {
        return -1;
    }
    for (i = 0; i < SAMPLE_RATE_MAX_NUM + 1; i++) {
        if (context->support_cfg[i].rate == config->rate) {
            return 0;
        }
    }
    return -1;
}
#endif

static int card_pcm_ioctl(SoundStreamT *stream, SoundStreamCmd cmd, void *param)
{
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    int i;
    int select_num = 0;
#endif
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
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
        if (context->output_config) {
            for (i = 0; i < SAMPLE_RATE_MAX_NUM + 1; i++) {
                if (context->support_cfg[i].valid) {
                    if (context->support_cfg[i].rate >= config->rate) {
                        select_num = i;
                        break;
                    }
                    select_num = i;
                }
            }
            if (context->support_cfg[select_num].channels == 0) {
                context->output_config->channels = config->channels;
            } else {
                context->output_config->channels = context->support_cfg[select_num].channels;
            }
            if (context->support_cfg[select_num].rate == 0) {
                context->output_config->rate = config->rate;
            } else {
                context->output_config->rate = context->support_cfg[select_num].rate;
            }
        }
#endif
        break;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    case STREAM_CMD_SET_OUTPUT_CONFIG:
    case STREAM_CMD_ADD_OUTPUT_CONFIG:
        config = (struct SscPcmConfig *)param;
        if (check_config_param(context, config) == -1) {
            printf("invalid output config. channel:%u, rate:%u\n", config->channels, config->rate);
            break;
        }

        if (context->output_config == NULL) {
            context->output_config = (struct pcm_config *)malloc(sizeof(struct pcm_config));
            if (context->output_config == NULL) {
                break;
            }
        }
        for (i = 0; i < SAMPLE_RATE_MAX_NUM + 1; i++) {
            if (context->support_cfg[i].rate == config->rate) {
                context->support_cfg[i].channels = config->channels;
                context->support_cfg[i].valid = 1;
                break;
            }
        }

        context->output_config->channels     = config->channels;
        context->output_config->rate         = config->rate;
        context->output_config->format       = PCM_FORMAT_S16_LE;
        context->output_config->period_count = AUDIO_PERIOD_COUNT;
        context->output_config->period_size  = AUDIO_PERIOD_SIZE;
        break;
    case STREAM_CMD_CLEAR_OUTPUT_CONFIG:
        free(context->output_config);
        context->output_config = NULL;
        for (i = 0; i < SAMPLE_RATE_MAX_NUM + 1; i++) {
            context->support_cfg[i].valid = 0;
        }
        break;
#endif
#ifdef SUPPORT_EQ
    case STREAM_CMD_SET_EQ_MODE:
        if (context->equalizer)
            return 0;

        OS_MutexCreate(&context->eq_lock);
        eq_prms_t *prms_config = (eq_prms_t*)param;
        context->prms_config.chan = context->input_config.channels;
        context->prms_config.sampling_rate = context->input_config.rate;
#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
        if (context->output_config) {
            context->prms_config.chan = context->output_config->channels;
            context->prms_config.sampling_rate = context->output_config->rate;
        }
#endif
        if (context->prms_config.chan <= 0)
            context->prms_config.chan = 1;

        context->radio = (context->prms_config.chan > 1) ? 4 : 2;
        if (prms_config && prms_config->core_prms) {
            context->prms_config.biq_num = prms_config->biq_num;
            context->prms_config.core_prms = prms_config->core_prms;
        }
        context->equalizer = eq_create(&context->prms_config);
        if (context->equalizer == NULL) {
            OS_MutexDelete(&context->eq_lock);
            return -1;
        }
        break;
    case STREAM_CMD_CLEAR_EQ_MODE:
        if (context->equalizer) {
            OS_MutexLock(&context->eq_lock, OS_WAIT_FOREVER);
            eq_destroy(context->equalizer);
            context->equalizer = NULL;
            OS_MutexUnlock(&context->eq_lock);
            OS_MutexDelete(&context->eq_lock);
        }
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

#ifdef SUPPORT_FIXED_OUTPUT_CONFIG
    /* from minimum value to maximum value */
    context->support_cfg[0].rate = 8000;
    context->support_cfg[1].rate = 11025;
    context->support_cfg[2].rate = 12000;
    context->support_cfg[3].rate = 16000;
    context->support_cfg[4].rate = 22050;
    context->support_cfg[5].rate = 24000;
    context->support_cfg[6].rate = 32000;
    context->support_cfg[7].rate = 44100;
    context->support_cfg[8].rate = 48000;
    context->support_cfg[9].rate = 0;  /* zero mean using original rate of the media */
#endif

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
