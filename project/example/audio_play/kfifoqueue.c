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

#include <CdxTypes.h>
#include <stdlib.h>
#include "kernel/os/os_mutex.h"
#include "kfifo.h"
#include "CdxFifoStream.h"

#define FIFOBUFFERSIZE   (4 * 1024)  /* at least 4k */

struct KfifoImpl {
    struct CdxFifoStreamS base;
    struct kfifo fifo;
    OS_Mutex_t mutex;
    int eos;
};

static void CdxKfifoLock(struct CdxFifoStreamS *streambase)
{
    struct KfifoImpl *impl;
    impl = CdxContainerOf(streambase, struct KfifoImpl, base);
    OS_MutexLock(&impl->mutex, OS_WAIT_FOREVER);
}

static void CdxKfifoUnlock(struct CdxFifoStreamS *streambase)
{
    struct KfifoImpl *impl;
    impl = CdxContainerOf(streambase, struct KfifoImpl, base);
    OS_MutexUnlock(&impl->mutex);
}

static cdx_int32 CdxKfifoIn(struct CdxFifoStreamS *streambase, void *buf, cdx_uint32 len)
{
    struct KfifoImpl *impl;
    impl = CdxContainerOf(streambase, struct KfifoImpl, base);
    return kfifo_in(&impl->fifo, (char *)buf, len);
}

static cdx_int32 CdxKfifoOut(struct CdxFifoStreamS *streambase, void *buf, cdx_uint32 len)
{
    struct KfifoImpl *impl;
    impl = CdxContainerOf(streambase, struct KfifoImpl, base);
    return kfifo_out(&impl->fifo, (char *)buf, len);
}

static cdx_int32 CdxKfifoValid(struct CdxFifoStreamS *streambase)
{
    struct KfifoImpl *impl;
    impl = CdxContainerOf(streambase, struct KfifoImpl, base);
    return kfifo_len(&impl->fifo);
}

static cdx_int32 CdxKfifoAvail(struct CdxFifoStreamS *streambase)
{
    struct KfifoImpl *impl;
    impl = CdxContainerOf(streambase, struct KfifoImpl, base);
    return kfifo_avail(&impl->fifo);
}

static cdx_int32 CdxKfifoSeteos(struct CdxFifoStreamS *streambase)
{
    struct KfifoImpl *impl;
    impl = CdxContainerOf(streambase, struct KfifoImpl, base);
    impl->eos = 1;
    return 0;
}

static cdx_bool CdxKfifoIseos(struct CdxFifoStreamS *streambase)
{
    struct KfifoImpl *impl;
    impl = CdxContainerOf(streambase, struct KfifoImpl, base);
    return (impl->eos == 1);
}

static const struct CdxFifoStreamOpsS KfifoOps = {
    .lock = CdxKfifoLock,
    .unlock = CdxKfifoUnlock,
    .in = CdxKfifoIn,
    .out = CdxKfifoOut,
    .valid = CdxKfifoValid,
    .avail = CdxKfifoAvail,
    .seteos = CdxKfifoSeteos,
    .iseos = CdxKfifoIseos,
};

struct CdxFifoStreamS *kfifo_stream_create()
{
    int ret;
    struct KfifoImpl *impl;

    impl = malloc(sizeof(*impl));
    if (impl == NULL) {
        printf("KfifoImpl malloc fail\n");
        goto err1;
    }
    memset(impl, 0, sizeof(*impl));

    ret = kfifo_alloc(&impl->fifo, FIFOBUFFERSIZE);
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

int kfifo_stream_destroy(struct CdxFifoStreamS *streambase)
{
    struct KfifoImpl *impl;

    impl = CdxContainerOf(streambase, struct KfifoImpl, base);

    OS_MutexDelete(&impl->mutex);
    kfifo_free(&impl->fifo);
    free(impl);

    return 0;
}

