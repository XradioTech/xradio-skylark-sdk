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
#include "reverb_buffer.h"

//判断x是否是2的次方
#define is_power_of_2(x) ((x) != 0 && (((x) & ((x) - 1)) == 0))
//取a和b中最小值
#define min(a, b) (((a) < (b)) ? (a) : (b))

//初始化缓冲区
reverb_buffer* reverb_buffer_init(unsigned int size)
{
    reverb_buffer *ring_buf = NULL;

    if (!is_power_of_2(size)) {
        printf("size must be power of 2.\n");
        return NULL;
    }
    ring_buf = (reverb_buffer *)malloc(sizeof(reverb_buffer));
    if (!ring_buf) {
        goto err;
    }
    memset(ring_buf, 0, sizeof(reverb_buffer));

    ring_buf->buffer = (unsigned char *)malloc(size);
    if (!ring_buf->buffer) {
        goto err;
    }
    memset(ring_buf->buffer, 0, size);

    ring_buf->size = size;
    ring_buf->in = 0;
    ring_buf->out = 0;
    return ring_buf;

err:
    printf("failed to malloc memory\n");
    free(ring_buf);
    return NULL;
}

//释放缓冲区
void reverb_buffer_free(reverb_buffer *ring_buf)
{
    if (ring_buf) {
        if (ring_buf->buffer) {
            free(ring_buf->buffer);
            ring_buf->buffer = NULL;
        }
        free(ring_buf);
        ring_buf = NULL;
    }
}

//从缓冲区中取数据
static unsigned int __reverb_buffer_get(reverb_buffer *ring_buf, unsigned char * buffer, unsigned int size)
{
    if (!ring_buf || !buffer)
        return 0;
    unsigned int len = 0;
    size  = min(size, ring_buf->in - ring_buf->out);
    /* first get the data from fifo->out until the end of the buffer */
    len = min(size, ring_buf->size - (ring_buf->out & (ring_buf->size - 1)));
    memcpy(buffer, ring_buf->buffer + (ring_buf->out & (ring_buf->size - 1)), len);
    /* then get the rest (if any) from the beginning of the buffer */
    memcpy(buffer + len, ring_buf->buffer, size - len);
    ring_buf->out += size;
    return size;
}

//向缓冲区中存放数据
static unsigned int __reverb_buffer_put(reverb_buffer *ring_buf, unsigned char *buffer, unsigned int size)
{
    if (!ring_buf || !buffer)
        return 0;
    unsigned int len = 0;
    size = min(size, ring_buf->size - ring_buf->in + ring_buf->out);
    /* first put the data starting from fifo->in to buffer end */
    len = min(size, ring_buf->size - (ring_buf->in & (ring_buf->size - 1)));
    memcpy(ring_buf->buffer + (ring_buf->in & (ring_buf->size - 1)), buffer, len);
    /* then put the rest (if any) at the beginning of the buffer */
    memcpy(ring_buf->buffer, buffer + len, size - len);
    ring_buf->in += size;
    return size;
}

unsigned int reverb_buffer_get(reverb_buffer *ring_buf, unsigned char *buffer, unsigned int size)
{
    unsigned int ret;
    ret = __reverb_buffer_get(ring_buf, buffer, size);
    //buffer中没有数据
    if (ring_buf->in == ring_buf->out)
      ring_buf->in = ring_buf->out = 0;
    return ret;
}

unsigned int reverb_buffer_put(reverb_buffer *ring_buf, unsigned char *buffer, unsigned int size)
{
    unsigned int ret;
    ret = __reverb_buffer_put(ring_buf, buffer, size);
    return ret;
}

#endif