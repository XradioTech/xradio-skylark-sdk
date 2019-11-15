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

#ifndef _PCM_FIFO_H_
#define _PCM_FIFO_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum PcmFifoCmd {
	SAVE_PCM_DATA,
	DROP_PCM_DATA,
} PcmFifoCmd;

typedef struct PcmFifoS PcmFifoT;

struct PcmFifoOpsS {
    void (*lock)(PcmFifoT *);
    void (*unlock)(PcmFifoT *);
    int (*in)(PcmFifoT *, void *buf, unsigned int len, int cover_write);
    int (*out)(PcmFifoT *, void *buf, unsigned int len);
    int (*valid)(PcmFifoT *);    /* get the size of valid data in the fifo */
    int (*avail)(PcmFifoT *);    /* get the remain room of the fifo */
	int (*control)(PcmFifoT *, PcmFifoCmd cmd, void *param);
};

struct PcmFifoS {
    const struct PcmFifoOpsS *ops;
};

static inline void PcmFifoLock(struct PcmFifoS *fifo)
{
    fifo->ops->lock(fifo);
}

static inline void PcmFifoUnlock(struct PcmFifoS *fifo)
{
    fifo->ops->unlock(fifo);
}

static inline int PcmFifoIn(struct PcmFifoS *fifo, void *buf, unsigned int len, int cover_write)
{
    return fifo->ops->in(fifo, buf, len, cover_write);
}

static inline int PcmFifoOut(struct PcmFifoS *fifo, void *buf, unsigned int len)
{
    return fifo->ops->out(fifo, buf, len);
}

static inline int PcmFifoValid(struct PcmFifoS *fifo)
{
    return fifo->ops->valid(fifo);
}

static inline int PcmFifoAvail(struct PcmFifoS *fifo)
{
    return fifo->ops->avail(fifo);
}

static inline int PcmFifoControl(struct PcmFifoS *fifo, PcmFifoCmd cmd, void *param)
{
    return fifo->ops->control(fifo, cmd, param);
}

/* return: NON-NULL for success, NULL for error. */
extern struct PcmFifoS *pcm_fifo_create(int fifo_size);

/* return: 0 for success, other for error. */
extern int pcm_fifo_destroy(struct PcmFifoS *fifo);

#ifdef __cplusplus
}
#endif

#endif

