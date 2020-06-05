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
#include "types.h"

enum DMAHEAP_FLAG {
    DMAHEAP_SYSTEM_RAM      = (1 << 0),
    DMAHEAP_SRAM            = (1 << 1),
    DMAHEAP_PSRAM           = (1 << 2),
};

#if ((defined __CONFIG_PSRAM) && (__CONFIG_DMAHEAP_PSRAM_SIZE != 0))
void *_dma_malloc( size_t size , uint32_t flag);
void _dma_free( void *ptr , uint32_t flag );
void *_dma_calloc( size_t nmemb, size_t size, uint32_t flag );
void *_dma_realloc(    void *ptr, size_t size, uint32_t flag );

__inline void *dma_malloc( size_t size , uint32_t flag)
{
    return _dma_malloc(size, flag);
}
__inline void dma_free( void *ptr , uint32_t flag )
{
    return _dma_free(ptr, flag);
}
__inline void *dma_calloc( size_t nmemb, size_t size, uint32_t flag )
{
    return _dma_calloc(nmemb, size, flag);
}
__inline void *dma_realloc(    void *ptr, size_t size, uint32_t flag )
{
    return _dma_realloc(ptr, size, flag);
}

#else

__inline void *dma_malloc( size_t size, uint32_t flag )
{
    return malloc(size);
}

__inline void dma_free( void *ptr, uint32_t flag )
{
    return free(ptr);
}

__inline void *dma_calloc( size_t nmemb, size_t size, uint32_t flag )
{
    return calloc(nmemb, size);
}

__inline void *dma_realloc(    void *ptr, size_t size, uint32_t flag )
{
    return realloc(ptr, size);
}
#endif /*((defined __CONFIG_PSRAM) && (__CONFIG_DMAHEAP_PSRAM_SIZE != 0))*/
