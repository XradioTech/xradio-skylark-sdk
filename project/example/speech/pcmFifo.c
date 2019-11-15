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

#include <stdlib.h>
#include <string.h>
#include "kernel/os/os_mutex.h"
#include "kfifo.h"
#include "pcmFifo.h"

struct PcmFifoImpl {
    struct PcmFifoS base; /* must put it first */
    struct kfifo fifo;
    OS_Mutex_t mutex;
	int save; /* default not save */
};

static void PcmKfifoLock(struct PcmFifoS *fifo)
{
    struct PcmFifoImpl *impl;
	impl = (struct PcmFifoImpl *)fifo;
    OS_MutexLock(&impl->mutex, OS_WAIT_FOREVER);
}

static void PcmKfifoUnlock(struct PcmFifoS *fifo)
{
    struct PcmFifoImpl *impl;
	impl = (struct PcmFifoImpl *)fifo;
    OS_MutexUnlock(&impl->mutex);
}

static int PcmKfifoIn(struct PcmFifoS *fifo, void *buf, unsigned int len, int cover_write)
{
	int putLen = len;
	char buffer[256];
    struct PcmFifoImpl *impl;
	impl = (struct PcmFifoImpl *)fifo;

	if (impl->save) {
		if (cover_write) {
			int avail;
			avail = kfifo_avail(&impl->fifo);
			if (avail < len) {
				int outLen = len - avail;
				int tmpLen;
				while (outLen > 0) {
					tmpLen = (outLen > 256) ? 256 : outLen;
					kfifo_out(&impl->fifo, buffer, tmpLen);
					outLen -= tmpLen;
				}
			}
		}
		putLen = kfifo_in(&impl->fifo, (char *)buf, len);
	}
    return putLen;
}

static int PcmKfifoOut(struct PcmFifoS *fifo, void *buf, unsigned int len)
{
    struct PcmFifoImpl *impl;
	impl = (struct PcmFifoImpl *)fifo;
    return kfifo_out(&impl->fifo, (char *)buf, len);
}

static int PcmKfifoValid(struct PcmFifoS *fifo)
{
    struct PcmFifoImpl *impl;
	impl = (struct PcmFifoImpl *)fifo;
    return kfifo_len(&impl->fifo);
}

static int PcmKfifoAvail(struct PcmFifoS *fifo)
{
    struct PcmFifoImpl *impl;
    impl = (struct PcmFifoImpl *)fifo;
    return kfifo_avail(&impl->fifo);
}

static int PcmKfifoControl(struct PcmFifoS *fifo, PcmFifoCmd cmd, void *param)
{
    struct PcmFifoImpl *impl;
    impl = (struct PcmFifoImpl *)fifo;

	switch (cmd) {
	case SAVE_PCM_DATA:
		impl->save = 1;
		break;
	case DROP_PCM_DATA:
		impl->save = 0;
		break;
	default:
		break;
	}
    return 0;
}

static const struct PcmFifoOpsS KfifoOps = {
    .lock = PcmKfifoLock,
    .unlock = PcmKfifoUnlock,
    .in = PcmKfifoIn,
    .out = PcmKfifoOut,
    .valid = PcmKfifoValid,
    .avail = PcmKfifoAvail,
    .control = PcmKfifoControl,
};

struct PcmFifoS *pcm_fifo_create(int fifo_size)
{
    int ret;
    struct PcmFifoImpl *impl;

    impl = malloc(sizeof(*impl));
    if (impl == NULL) {
        printf("PcmFifoImpl malloc fail\n");
        goto err1;
    }
    memset(impl, 0, sizeof(*impl));

    ret = kfifo_alloc(&impl->fifo, fifo_size);
    if (ret) {
        printf("kfifo alloc fail\n");
        goto err2;
    }

    if (OS_MutexCreate(&impl->mutex) != OS_OK) {
        printf("mutex create fail\n");
        goto err3;
    }

    impl->base.ops = &KfifoOps;
    return &impl->base;

err3:
    kfifo_free(&impl->fifo);
err2:
    free(impl);
err1:
    return NULL;
}

int pcm_fifo_destroy(struct PcmFifoS *fifo)
{
    struct PcmFifoImpl *impl;

    impl = (struct PcmFifoImpl *)fifo;

    OS_MutexDelete(&impl->mutex);
    kfifo_free(&impl->fifo);
    free(impl);

    return 0;
}

