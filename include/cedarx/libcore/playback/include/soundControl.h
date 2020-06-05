/*
 * Copyright (c) 2008-2016 Allwinner Technology Co. Ltd.
 * All rights reserved.
 *
 * File : soundControl.h
 * Description : soundControl
 * History :
 *
 */

#ifndef SOUND_CONTROL_H
#define SOUND_CONTROL_H
#include "cdx_log.h"
#include "adecoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum XAudioTimestretchStretchMode  {
    AUDIO_TIMESTRETCH_STRETCH_DEFAULT            = 0,
    AUDIO_TIMESTRETCH_STRETCH_SPEECH             = 1,
} XAudioTimestretchStretchMode;

typedef enum XAudioTimestretchFallbackMode  {
    XAUDIO_TIMESTRETCH_FALLBACK_CUT_REPEAT     = -1,
    XAUDIO_TIMESTRETCH_FALLBACK_DEFAULT        = 0,
    XAUDIO_TIMESTRETCH_FALLBACK_MUTE           = 1,
    XAUDIO_TIMESTRETCH_FALLBACK_FAIL           = 2,
} XAudioTimestretchFallbackMode;

typedef struct XAudioPlaybackRate {
    float mSpeed;
    float mPitch;
    enum XAudioTimestretchStretchMode  mStretchMode;
    enum XAudioTimestretchFallbackMode mFallbackMode;
}XAudioPlaybackRate;

typedef struct SoundCtrl SoundCtrl;

typedef struct SoundControlOpsS SoundControlOpsT;

struct SoundControlOpsS
{
    void (*cdxDestroy)(SoundCtrl* s);

    void (*cdxSetFormat)(SoundCtrl* s, CdxPlaybkCfg* cfg);

    int (*cdxStart)(SoundCtrl* s);

    int (*cdxStop)(SoundCtrl* s);

    int (*cdxPause)(SoundCtrl* s);

    int (*cdxFlush)(SoundCtrl* s, void *block);

    int (*cdxWrite)(SoundCtrl* s, void* pData, int nDataSize);

    int (*cdxReset)(SoundCtrl* s);

    int (*cdxGetCachedTime)(SoundCtrl* s);

    int (*cdxGetFrameCount)(SoundCtrl* s);

    int (*cdxSetPlaybackRate)(SoundCtrl* s,const XAudioPlaybackRate *rate);

    int (*cdxControl)(SoundCtrl* s, int cmd, void* para);
};

struct SoundCtrl
{
    const struct SoundControlOpsS* ops;
};

typedef enum _SoundCtrlCmd
{
/*
    Set para area...
*/
    SOUND_CONTROL_SET_OPTION_START = 100,
    SOUND_CONTROL_SET_CLBK_EOS,
/*
    Get para area...
*/
    SOUND_CONTROL_GET_OPTION_START = 200,
/*
    Query area...
*/
    SOUND_CONTROL_QUERY_OPTION_START = 300,
    SOUND_CONTROL_QUERY_IF_GAPLESS_PLAY,
/*
    Sound Stream Control area...
*/
    SOUND_CONTROL_SET_OUTPUT_CONFIG = 400,
    SOUND_CONTROL_ADD_OUTPUT_CONFIG,
    SOUND_CONTROL_CLEAR_OUTPUT_CONFIG,
    SOUND_CONTROL_SET_EQ_MODE,
    SOUND_CONTROL_CLEAR_EQ_MODE,
}SoundCtrlCmd;

static inline void SoundDeviceDestroy(SoundCtrl* s)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxDestroy);
    return s->ops->cdxDestroy(s);
}

static inline void SoundDeviceSetFormat(SoundCtrl* s, CdxPlaybkCfg* cfg)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxSetFormat);
    return s->ops->cdxSetFormat(s, cfg);
}

static inline int SoundDeviceStart(SoundCtrl* s)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxStart);
    return s->ops->cdxStart(s);
}

static inline int SoundDeviceStop(SoundCtrl* s)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxStop);
    return s->ops->cdxStop(s);
}

static inline int SoundDevicePause(SoundCtrl* s)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxPause);
    return s->ops->cdxPause(s);
}

static inline int SoundDeviceFlush(SoundCtrl* s, void* block)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxFlush);
    return s->ops->cdxFlush(s, block);
}

static inline int SoundDeviceWrite(SoundCtrl* s, void* pData, int nDataSize)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxWrite);
    return s->ops->cdxWrite(s, pData, nDataSize);
}

static inline int SoundDeviceReset(SoundCtrl* s)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxReset);
    return s->ops->cdxReset(s);
}

static inline int SoundDeviceGetCachedTime(SoundCtrl* s)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxGetCachedTime);
    return s->ops->cdxGetCachedTime(s);
}

static inline int SoundDeviceGetFrameCount(SoundCtrl* s)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxGetFrameCount);
    return s->ops->cdxGetFrameCount(s);
}

static inline int SoundDeviceSetPlaybackRate(SoundCtrl* s,const XAudioPlaybackRate *rate)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxSetPlaybackRate);
    return s->ops->cdxSetPlaybackRate(s,rate);
}

static inline int SoundDeviceControl(SoundCtrl* s, int cmd, void* para)
{
    CDX_CHECK(s);
    CDX_CHECK(s->ops);
    CDX_CHECK(s->ops->cdxControl);
    return s->ops->cdxControl(s, cmd, para);
}

SoundCtrl* SoundDeviceCreate();

#ifdef __cplusplus
}
#endif

#endif

