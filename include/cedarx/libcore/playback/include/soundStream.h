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

#ifndef _SOUND_STREAM_H
#define _SOUND_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

#define SS_CHECK(x)     do {                   \
                            if (!x) {          \
                                return -1;     \
                            }                  \
                        } while (0)            \

typedef enum {
    STREAM_TYPE_SOUND_CARD   = 0,
    STREAM_TYPE_REVERB_PCM   = 1,
    STREAM_TYPE_CUSTOMER     = 2,
} SoundStreamType;

typedef enum {
    STREAM_CMD_SET_CONFIG,
    STREAM_CMD_SET_OUTPUT_CONFIG,
    STREAM_CMD_CLEAR_OUTPUT_CONFIG,
    STREAM_CMD_SET_BLOCK_MODE,
} SoundStreamCmd;

typedef enum {
    STREAM_PRIORITY_LEVEL1 = 1,
    STREAM_PRIORITY_LEVEL2,
    STREAM_PRIORITY_LEVEL3,
    STREAM_PRIORITY_LEVEL4,
} SoundStreamPriority;

struct SscPcmConfig {
    unsigned int  channels;
    unsigned int  rate;
};

typedef struct SoundStreamS SoundStreamT;

struct SoundStreamOpsS {
    int (*soundOpen)(SoundStreamT *);
    int (*soundClose)(SoundStreamT *);
    int (*soundWrite)(SoundStreamT *, struct SscPcmConfig *config, void *buffer, unsigned int len);
    int (*soundRead)(SoundStreamT *, void *buffer, unsigned int len);
    int (*soundFlush)(SoundStreamT *);
    int (*soundIoctl)(SoundStreamT *, SoundStreamCmd cmd, void *param);
};

struct SoundStreamS {
    const struct SoundStreamOpsS *ops;
};

static inline int SoundStreamOpen(struct SoundStreamS *stream)
{
    SS_CHECK(stream);
    SS_CHECK(stream->ops);
    SS_CHECK(stream->ops->soundOpen);
    return stream->ops->soundOpen(stream);
}

static inline int SoundStreamClose(struct SoundStreamS *stream)
{
    SS_CHECK(stream);
    SS_CHECK(stream->ops);
    SS_CHECK(stream->ops->soundClose);
    return stream->ops->soundClose(stream);
}

static inline int SoundStreamWrite(struct SoundStreamS *stream, struct SscPcmConfig *config, void *buffer, unsigned int len)
{
    SS_CHECK(stream);
    SS_CHECK(stream->ops);
    SS_CHECK(stream->ops->soundWrite);
    return stream->ops->soundWrite(stream, config, buffer, len);
}

static inline int SoundStreamRead(struct SoundStreamS *stream, void *buffer, unsigned int len)
{
    SS_CHECK(stream);
    SS_CHECK(stream->ops);
    SS_CHECK(stream->ops->soundRead);
    return stream->ops->soundRead(stream, buffer, len);
}

static inline int SoundStreamFlush(struct SoundStreamS *stream)
{
    SS_CHECK(stream);
    SS_CHECK(stream->ops);
    SS_CHECK(stream->ops->soundFlush);
    return stream->ops->soundFlush(stream);
}

static inline int SoundStreamIoctl(struct SoundStreamS *stream, SoundStreamCmd cmd, void *param)
{
    SS_CHECK(stream);
    SS_CHECK(stream->ops);
    SS_CHECK(stream->ops->soundIoctl);
    return stream->ops->soundIoctl(stream, cmd, param);
}

struct SoundStreamCreatorS
{
    SoundStreamT *(*create)(void);
    void (*destroy)(SoundStreamT *);
};

#ifdef __cplusplus
}
#endif

#endif