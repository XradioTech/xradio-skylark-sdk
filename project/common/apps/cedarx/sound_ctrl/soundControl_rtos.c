/*
 * Copyright (c) 2008-2016 Allwinner Technology Co. Ltd.
 * All rights reserved.
 *
 * File : soundControl_rtos.c
 * Description : soundControl for rtos
 * History :
 *
 */

#ifdef __CONFIG_XPLAYER

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sound_log.h"
#include "soundControl.h"
#include "soundStreamControl.h"

enum SDSTATUS
{
    SD_STATUS_STOPPED = 0,
    SD_STATUS_STARTED,
    SD_STATUS_PAUSED
};

typedef struct SoundCtrlContext
{
    SoundCtrl          base;
    SoundStreamCtrl    ssc;
    enum SDSTATUS      eStatus;
    OS_Mutex_t         mutex;
} SoundCtrlContext;

static int SoundDeviceStop_l(SoundCtrlContext* sc);

static void __Release(SoundCtrl* s)
{
    SoundCtrlContext* sc;
    sc = (SoundCtrlContext*)s;

    OS_RecursiveMutexDelete(&sc->mutex);
    snd_stream_destroy(sc->ssc, STREAM_TYPE_SOUND_CARD);

    free(sc);
    sc = NULL;
    return;
}

static void __SetFormat(SoundCtrl* s, CdxPlaybkCfg* cfg)
{
    SoundCtrlContext* sc;
    struct SscPcmConfig config;

    sc = (SoundCtrlContext*)s;

    OS_RecursiveMutexLock(&sc->mutex, OS_WAIT_FOREVER);

    if(sc->eStatus != SD_STATUS_STOPPED)
    {
        SND_LOGD("Sound device not int stop status, can not set audio params.");
        OS_RecursiveMutexUnlock(&sc->mutex);
        return ;
    }

    config.channels = cfg->nChannels;
    config.rate = cfg->nSamplerate;
    snd_stream_control(sc->ssc, STREAM_TYPE_SOUND_CARD, STREAM_CMD_SET_CONFIG, &config);

    OS_RecursiveMutexUnlock(&sc->mutex);

    return;
}

static int __Start(SoundCtrl* s)
{
    SoundCtrlContext* sc;
    sc = (SoundCtrlContext*)s;

    OS_RecursiveMutexLock(&sc->mutex, OS_WAIT_FOREVER);
    if(sc->eStatus == SD_STATUS_STARTED)
    {
        SND_LOGW("Sound device already started.");
        OS_RecursiveMutexUnlock(&sc->mutex);
        return -1;
    }

    if(sc->eStatus == SD_STATUS_STOPPED || sc->eStatus == SD_STATUS_PAUSED)
    {
        snd_stream_open(sc->ssc, STREAM_TYPE_SOUND_CARD);
    }
    sc->eStatus = SD_STATUS_STARTED;
    OS_RecursiveMutexUnlock(&sc->mutex);

    return 0;
}

static int __Stop(SoundCtrl* s)
{
    int ret;
    SoundCtrlContext* sc;

    sc = (SoundCtrlContext*)s;

    OS_RecursiveMutexLock(&sc->mutex, OS_WAIT_FOREVER);
    ret = SoundDeviceStop_l(sc);
    OS_RecursiveMutexUnlock(&sc->mutex);

    return ret;
}

static int SoundDeviceStop_l(SoundCtrlContext* sc)
{
    int err = 0;

    if(sc->eStatus == SD_STATUS_STOPPED)
    {
        SND_LOGW("Sound device already stopped.");
        return 0;
    }
    if(sc->eStatus != SD_STATUS_PAUSED)
    {
        int err;
        if ((err = snd_stream_close(sc->ssc, STREAM_TYPE_SOUND_CARD)) < 0)
        {
            SND_LOGE("Pcm close err..");
            return -1;
        }
    }
    sc->eStatus = SD_STATUS_STOPPED;
    return err;
}

static int __Pause(SoundCtrl* s)
{
    SoundCtrlContext* sc;
    int err;
    sc = (SoundCtrlContext*)s;

    OS_RecursiveMutexLock(&sc->mutex, OS_WAIT_FOREVER);
    if(sc->eStatus == SD_STATUS_STOPPED || sc->eStatus == SD_STATUS_PAUSED)
    {
        SND_LOGW("Sound device already stopped or paused.");
        OS_RecursiveMutexUnlock(&sc->mutex);
        return 0;
    }
    if ((err = snd_stream_close(sc->ssc, STREAM_TYPE_SOUND_CARD)) < 0)
    {
        SND_LOGE("Pcm close err..");
        OS_RecursiveMutexUnlock(&sc->mutex);
        return -1;
    }
    sc->eStatus = SD_STATUS_PAUSED;

    OS_RecursiveMutexUnlock(&sc->mutex);

    return 0;
}

static int __Flush(SoundCtrl* s, void* block)
{
    SoundCtrlContext* sc;
    int err;
    sc = (SoundCtrlContext*)s;

    OS_RecursiveMutexLock(&sc->mutex, OS_WAIT_FOREVER);
    if(sc->eStatus == SD_STATUS_STOPPED || sc->eStatus == SD_STATUS_PAUSED)
    {
        SND_LOGW("Sound device already stopped.");
        OS_RecursiveMutexUnlock(&sc->mutex);
        return 0;
    }
    if ((err = snd_stream_flush(sc->ssc, STREAM_TYPE_SOUND_CARD)) < 0)
    {
        OS_RecursiveMutexUnlock(&sc->mutex);
        SND_LOGE("Flush pcm data err..");
        return -1;
    }
    OS_RecursiveMutexUnlock(&sc->mutex);
    return 0;
}

static int __Write(SoundCtrl* s, void* pData, int nDataSize)
{
    int ret = 0;
    SoundCtrlContext* sc;

    SND_LOGD("nDataSize: %d", nDataSize);

    sc = (SoundCtrlContext*)s;

    OS_RecursiveMutexLock(&sc->mutex, OS_WAIT_FOREVER);
    if(sc->eStatus == SD_STATUS_STOPPED || sc->eStatus == SD_STATUS_PAUSED)
    {
        OS_RecursiveMutexUnlock(&sc->mutex);
        SND_LOGW("__Write:Sound device already stopped.");
        return 0;
    }

    ret = snd_stream_write(sc->ssc, STREAM_TYPE_SOUND_CARD, pData, nDataSize);
    if (ret != nDataSize) {
        OS_RecursiveMutexUnlock(&sc->mutex);
        return -1;
    }
    OS_RecursiveMutexUnlock(&sc->mutex);

    return ret;
}

/* called at player seek operation. */
static int __Reset(SoundCtrl* s)
{
    return SoundDeviceStop(s);
}

static int __GetCachedTime(SoundCtrl* s)
{
#if 0
    int ret;
    SoundCtrlContext* sc;

    sc = (SoundCtrlContext*)s;

    if(sc->eStatus == SD_STATUS_STOPPED || sc->eStatus == SD_STATUS_PAUSED)
    {
        return 0;
    }
    unsigned int delay = 0;

    if (snd_pcm_delay(sc->alsa_handler, &delay) < 0)
        return 0;

    if (sc->alsa_handler)
    {
        snd_pcm_sframes_t delay;

        if (snd_pcm_delay(sc->alsa_handler, &delay) < 0)
            return 0;

    if (delay < 0)
    {
        /* underrun - move the application pointer forward to catch up */
        #if SND_LIB_VERSION >= 0x000901 /* snd_pcm_forward() exists since 0.9.0rc8 */
        snd_pcm_forward(sc->alsa_handler, -delay);
        #endif
        delay = 0;
    }
    return ((int)((float) delay * 1000000 / (float) pcm_params.samplerate));
    }
    else
    {
        return 0;
    }
#else
    /* return 0 will mislead cedarx believed driver had play all audio data out,*/
    /* but in fact it's not. cedarx will close driver and lose the data. */
    return 0;
#endif

}

static int __GetFrameCount(SoundCtrl* s)
{
    return 0;
}

static int __Control(SoundCtrl* s, int cmd, void* para)
{
    SoundCtrlContext* sc;
    int ret = 0;
    sc = (SoundCtrlContext*)s;
    OS_RecursiveMutexLock(&sc->mutex, OS_WAIT_FOREVER);

    switch(cmd)
    {
        case SOUND_CONTROL_SET_CLBK_EOS:
            SND_LOGD("set can't set clbk eos for now");
            ret = -1;
            break;
        case SOUND_CONTROL_QUERY_IF_GAPLESS_PLAY:
            {
                /* need driver pad 0 to codec. */
                int answer = 0;
                SND_LOGD("set can't set gapless for now");
                ret = answer;
                SND_LOGD("query if gapless play (%s)", answer?"yes":"no");
                break;
            }
        case SOUND_CONTROL_SET_OUTPUT_CONFIG:
            snd_stream_control(sc->ssc, STREAM_TYPE_SOUND_CARD, STREAM_CMD_SET_OUTPUT_CONFIG, para);
            break;
        case SOUND_CONTROL_CLEAR_OUTPUT_CONFIG:
            snd_stream_control(sc->ssc, STREAM_TYPE_SOUND_CARD, STREAM_CMD_CLEAR_OUTPUT_CONFIG, para);
            break;
        default:
            SND_LOGD("unknown command (%d)...", cmd);
            ret = -1;
            break;
    }
    OS_RecursiveMutexUnlock(&sc->mutex);
    return ret;
}

static const SoundControlOpsT mSoundControlOps =
{
    .destroy       = __Release,
    .setFormat     = __SetFormat,
    .start         = __Start,
    .stop          = __Stop,
    .pause         = __Pause,
    .flush         = __Flush,
    .write         = __Write,
    .reset         = __Reset,
    .getCachedTime = __GetCachedTime,
    .getFrameCount = __GetFrameCount,
    .control       = __Control,
};

SoundCtrl* SoundDeviceCreate()
{
    SoundCtrlContext* s;
    SND_LOGD("SoundDeviceInit");
    s = (SoundCtrlContext*)malloc(sizeof(SoundCtrlContext));
    if(s == NULL)
    {
        SND_LOGE("malloc memory fail.");
        return NULL;
    }
    memset(s, 0, sizeof(SoundCtrlContext));

    s->base.ops = &mSoundControlOps;
    s->eStatus = SD_STATUS_STOPPED;
    s->ssc = snd_stream_create(STREAM_TYPE_SOUND_CARD);
    if (s->ssc == NULL)
    {
        SND_LOGE("snd_stream_create fail.");
        free(s);
        return NULL;
    }

    OS_RecursiveMutexCreate(&s->mutex);
    return (SoundCtrl*)s;
}

#endif