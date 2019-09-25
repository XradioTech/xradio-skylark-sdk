/*
 * Copyright (c) 2017-2020 Allwinner Technology Co. Ltd.
 * All rights reserved.
 *
 * File : CaptureControl_rtos.c
 * Description : CaptureControl for rtos
 * History :
 *
 */

#ifdef __CONFIG_XPLAYER

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "CaptureControl.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/manager/audio_manager.h"

enum SDSTATUS
{
    SD_STATUS_STOPPED = 0,
    SD_STATUS_STARTED,
    SD_STATUS_PAUSED
};

typedef struct {
    CaptureCtrl              base;
    struct pcm_config        config;
    enum SDSTATUS            eStatus;
    OS_Mutex_t               mutex;
} CaptureCtrlContext;

#define CAPTURE_PERIOD_SIZE             512
#define CAPTURE_PERIOD_COUNT            2
#define CAPTURE_SOUND_CARD              AUDIO_SND_CARD_DEFAULT

static int CaptureDeviceStop_l(CaptureCtrlContext* cc);

static void __Release(CaptureCtrl* c)
{
    CaptureCtrlContext* cc;

    cc = (CaptureCtrlContext*)c;

    OS_RecursiveMutexDelete(&cc->mutex);

    free(cc);
    cc = NULL;
    return;
}

static void __SetFormat(CaptureCtrl* c, CdxCapbkCfg* cfg)
{
    CaptureCtrlContext* cc;

    cc = (CaptureCtrlContext*)c;

    OS_RecursiveMutexLock(&cc->mutex, OS_WAIT_FOREVER);

    if(cc->eStatus != SD_STATUS_STOPPED)
    {
        OS_RecursiveMutexUnlock(&cc->mutex);
        return ;
    }
    CdxCapbkCfg *param = (CdxCapbkCfg *)cfg;

    if (cfg != NULL) {
        cc->config.channels = param->nChannels;
        cc->config.rate = param->nSamplerate;
    } else {
        printf("audio params is NULL\n");
    }

    cc->config.format = PCM_FORMAT_S16_LE;
    cc->config.period_count = CAPTURE_PERIOD_COUNT;
    cc->config.period_size = CAPTURE_PERIOD_SIZE;

    OS_RecursiveMutexUnlock(&cc->mutex);

    return;
}

static int __Start(CaptureCtrl* c)
{
    CaptureCtrlContext* cc;

    cc = (CaptureCtrlContext*)c;

    OS_RecursiveMutexLock(&cc->mutex, OS_WAIT_FOREVER);
    if(cc->eStatus == SD_STATUS_STARTED)
    {
        OS_RecursiveMutexUnlock(&cc->mutex);
        return -1;
    }

    if(cc->eStatus == SD_STATUS_STOPPED)
    {
        snd_pcm_open(CAPTURE_SOUND_CARD, PCM_IN, &(cc->config));
    }
    cc->eStatus = SD_STATUS_STARTED;
    OS_RecursiveMutexUnlock(&cc->mutex);

    return 0;
}

static int __Stop(CaptureCtrl* c)
{
    int ret;

    CaptureCtrlContext* cc;
    cc = (CaptureCtrlContext*)c;

    OS_RecursiveMutexLock(&cc->mutex, OS_WAIT_FOREVER);

    ret = CaptureDeviceStop_l(cc);

    OS_RecursiveMutexUnlock(&cc->mutex);

    return ret;
}

static int CaptureDeviceStop_l(CaptureCtrlContext* cc)
{
    int err = 0;

    if(cc->eStatus == SD_STATUS_STOPPED)
    {
        return 0;
    }

    if(cc->eStatus != SD_STATUS_STOPPED)
    {
        int err;
        if ((err = snd_pcm_close(CAPTURE_SOUND_CARD, PCM_IN)) < 0)
        {
            printf("MSGTR_AO_ALSA_PcmCloseError");
        }

        cc->eStatus = SD_STATUS_STOPPED;
    }
    return err;
}

static int __Read(CaptureCtrl* c, void* pData, int nDataSize)
{
    int ret;
    CaptureCtrlContext* cc;

    cc = (CaptureCtrlContext*)c;

    OS_RecursiveMutexLock(&cc->mutex, OS_WAIT_FOREVER);

    if(cc->eStatus == SD_STATUS_STOPPED || cc->eStatus == SD_STATUS_PAUSED)
    {
        OS_RecursiveMutexUnlock(&cc->mutex);
        return 0;
    }

    ret = snd_pcm_read(CAPTURE_SOUND_CARD, pData, nDataSize);

    if (ret != nDataSize) {
        OS_RecursiveMutexUnlock(&cc->mutex);
        return -1;
    }

    OS_RecursiveMutexUnlock(&cc->mutex);
    return ret;
}

static const struct CaptureControlOpsS mCaptureControlOps =
{
    .destroy   =     __Release,
    .setFormat =     __SetFormat,
    .start     =     __Start,
    .stop      =     __Stop,
    .read      =     __Read,
};

CaptureCtrl* CaptureDeviceCreate()
{
    CaptureCtrlContext* cc;

    cc = (CaptureCtrlContext*)malloc(sizeof(CaptureCtrlContext));
    if(cc == NULL)
    {
        printf("malloc memory fail.");
        return NULL;
    }
    memset(cc, 0, sizeof(CaptureCtrlContext));

    cc->base.ops = &mCaptureControlOps;

    cc->eStatus = SD_STATUS_STOPPED;

    OS_RecursiveMutexCreate(&cc->mutex);
    return (CaptureCtrl*)cc;
}

#endif
