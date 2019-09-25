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

#ifndef _AUDIO_FIFO_HEADER_
#define _AUDIO_FIFO_HEADER_

#include <stdbool.h>
#include "common/apps/player_app.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FIFO_CHECK(e)

typedef struct AudioFifoS AudioFifoT;

struct AudioFifoOpsS {
    int (*set_player)(AudioFifoT *, player_base *player);
    int (*start)(AudioFifoT *);
    int (*put_data)(AudioFifoT *, void *inData, int dataLen);
    int (*stop)(AudioFifoT *, bool stop_immediately);
};

struct AudioFifoS {
    const struct AudioFifoOpsS *ops;
};

static inline int AudioFifoSetPlayer(struct AudioFifoS *audiofifo, player_base *player)
{
    FIFO_CHECK(audiofifo);
    FIFO_CHECK(audiofifo->ops);
    FIFO_CHECK(audiofifo->ops->set_player);
    return audiofifo->ops->set_player(audiofifo, player);
}

static inline int AudioFifoStart(struct AudioFifoS *audiofifo)
{
    FIFO_CHECK(audiofifo);
    FIFO_CHECK(audiofifo->ops);
    FIFO_CHECK(audiofifo->ops->start);
    return audiofifo->ops->start(audiofifo);
}

static inline int AudioFifoPutData(struct AudioFifoS *audiofifo, void *inData, int dataLen)
{
    FIFO_CHECK(audiofifo);
    FIFO_CHECK(audiofifo->ops);
    FIFO_CHECK(audiofifo->ops->put_data);
    return audiofifo->ops->put_data(audiofifo, inData, dataLen);
}

static inline int AudioFifoStop(struct AudioFifoS *audiofifo, bool stop_immediately)
{
    FIFO_CHECK(audiofifo);
    FIFO_CHECK(audiofifo->ops);
    FIFO_CHECK(audiofifo->ops->stop);
    return audiofifo->ops->stop(audiofifo, stop_immediately);
}

struct AudioFifoS *audio_fifo_create();
int audio_fifo_destroy(struct AudioFifoS *audiofifo);

#ifdef __cplusplus
}
#endif

#endif /*_AUDIO_HEADER_*/

