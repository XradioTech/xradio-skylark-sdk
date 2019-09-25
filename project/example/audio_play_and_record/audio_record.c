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
#include "kernel/os/os.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/manager/audio_manager.h"

#include "audio_record.h"
#include "record_output.h"

#define RECORDER_THREAD_STACK_SIZE    (1024 * 2)

#define SAMPLE_CHANNELS   4     /* we sample 4 channels as example */
#define SAMPLE_SIZE       2     /* 2 bytes each sample */
#define SAMPLES_16K_10MS  160   /* for 16 kHz */
#define AUDIO_CARD_ID   AUDIO_SND_CARD_DEFAULT

static OS_Thread_t record_thread;

struct audioContext {
    struct pcm_config pcm_cfg;
    unsigned int read_len;
    void *read_data;
    short *channel_1_data;
    short *channel_2_data;
    short *channel_3_data;
    short *channel_4_data;
};

static int _audio_record_start(struct audioContext *pAudio)
{
    int ret = 0;

    memset(pAudio, 0, sizeof(*pAudio));

    printf("here we record %d channels. also, we can record 1/2/4 channels\n", SAMPLE_CHANNELS);

    /* enable 4 channel */
    audio_manager_handler(AUDIO_SND_CARD_DEFAULT, AUDIO_MANAGER_SET_ROUTE, AUDIO_IN_DEV_AMIC, 1);
    audio_manager_handler(AUDIO_SND_CARD_DEFAULT, AUDIO_MANAGER_SET_ROUTE, AUDIO_IN_DEV_LINEIN, 1);
    audio_manager_handler(AUDIO_SND_CARD_DEFAULT, AUDIO_MANAGER_SET_ROUTE, AUDIO_IN_DEV_DMIC, 1);

    pAudio->pcm_cfg.channels = SAMPLE_CHANNELS;
    pAudio->pcm_cfg.format = PCM_FORMAT_S16_LE;
    pAudio->pcm_cfg.period_count = 2;
    pAudio->pcm_cfg.period_size = SAMPLES_16K_10MS;
    pAudio->pcm_cfg.rate = SAMPLE_RATE_OF_RECORD;

    ret = snd_pcm_open(AUDIO_CARD_ID, PCM_IN, &pAudio->pcm_cfg);
    if (ret != 0) {
        printf("pcm open error\n");
        goto err1;
    }

    pAudio->read_len = SAMPLES_16K_10MS * SAMPLE_SIZE * SAMPLE_CHANNELS;
    pAudio->read_data = (void *)malloc(pAudio->read_len);
    if (pAudio->read_data == NULL) {
        goto err2;
    }

    pAudio->channel_1_data = (short *)malloc(SAMPLES_16K_10MS * SAMPLE_SIZE);
    if (pAudio->channel_1_data == NULL) {
        goto err2;
    }

    pAudio->channel_2_data = (short *)malloc(SAMPLES_16K_10MS * SAMPLE_SIZE);
    if (pAudio->channel_2_data == NULL) {
        goto err2;
    }

    pAudio->channel_3_data = (short *)malloc(SAMPLES_16K_10MS * SAMPLE_SIZE);
    if (pAudio->channel_3_data == NULL) {
        goto err2;
    }

    pAudio->channel_4_data = (short *)malloc(SAMPLES_16K_10MS * SAMPLE_SIZE);
    if (pAudio->channel_4_data == NULL) {
        goto err2;
    }

    return 0;

err2:
    free(pAudio->channel_4_data);
    free(pAudio->channel_3_data);
    free(pAudio->channel_2_data);
    free(pAudio->channel_1_data);
    free(pAudio->read_data);
    snd_pcm_close(AUDIO_CARD_ID, PCM_IN);
err1:
    return -1;
}

static int _audio_record_get_data(struct audioContext *pAudio)
{
    int i;
    int ret;

    ret = snd_pcm_read(AUDIO_CARD_ID, pAudio->read_data, pAudio->read_len);
    if (ret != pAudio->read_len) {
        printf("snd_pcm_read fail.\n");
        return -1;
    }

    /* we separate pcm data, so we can get pcm data of each channel */
    for (i = 0; i < SAMPLES_16K_10MS; i++) {
        pAudio->channel_1_data[i] = ((short *)pAudio->read_data)[4 * i];
        pAudio->channel_2_data[i] = ((short *)pAudio->read_data)[4 * i + 1];
        pAudio->channel_3_data[i] = ((short *)pAudio->read_data)[4 * i + 2];
        pAudio->channel_4_data[i] = ((short *)pAudio->read_data)[4 * i + 3];
    }

    return 0;
}

static void _audio_record_stop(struct audioContext *pAudio)
{
    free(pAudio->channel_4_data);
    free(pAudio->channel_3_data);
    free(pAudio->channel_2_data);
    free(pAudio->channel_1_data);
    free(pAudio->read_data);
    snd_pcm_close(AUDIO_CARD_ID, PCM_IN);
}

static void record_task(void *arg)
{
    int ret;
    struct audioContext aContext;

    printf("start to init output module\n");
    ret = record_output_init();
    if (ret) {
        goto exit;
    }

    printf("start to record\n");
    ret = _audio_record_start(&aContext);
    if (ret) {
        goto err1;
    }

    printf("init success\n");

    while (1) {
        _audio_record_get_data(&aContext);
        record_output_put_data(aContext.read_data, aContext.read_len);
    }

    _audio_record_stop(&aContext);

err1:
    record_output_deinit();
exit:
    OS_ThreadDelete(&record_thread);
}

int audio_record_start()
{
    if (OS_ThreadCreate(&record_thread,
                        "record_task",
                        record_task,
                        NULL,
                        OS_THREAD_PRIO_APP,
                        RECORDER_THREAD_STACK_SIZE) != OS_OK) {
        printf("thread create fail.exit\n");
        return -1;
    }
    return 0;
}

