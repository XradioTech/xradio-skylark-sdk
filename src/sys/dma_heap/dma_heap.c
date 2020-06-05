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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys/param.h"
#include "sys/dma_heap.h"
#include "sys/sys_heap.h"
#include "driver/chip/hal_dcache.h"

#if ((defined __CONFIG_PSRAM) && (__CONFIG_DMAHEAP_PSRAM_SIZE != 0))

static sys_heap_t *g_dmaPsramHeap = NULL;

#define sram_malloc     malloc
#define sram_free       free
#define sram_calloc     calloc
#define sram_realloc    realloc

static int32_t dma_PsramHeap_init()
{
    g_dmaPsramHeap = (sys_heap_t *)malloc(sizeof(sys_heap_t));
    memset(g_dmaPsramHeap, 0, sizeof(sys_heap_t));
    HAL_Dcache_Enable_WriteThrough(DMAHEAP_PSRAM_BASE, rounddown2(DMAHEAP_PSRAM_END, 16));
    SYSHEAP_DEFAULT_INIT(g_dmaPsramHeap, (uint8_t *)DMAHEAP_PSRAM_BASE, DMAHEAP_PSRAM_LENGTH);
    sys_heap_init(g_dmaPsramHeap);
    return 0;
}

void *_dma_malloc( size_t size , uint32_t flag )
{
    void* ptr;
    if(flag == DMAHEAP_PSRAM) {
        if(g_dmaPsramHeap == NULL) {
            if(dma_PsramHeap_init() != 0) {
                return NULL;
            }
        }
        ptr = sys_heap_malloc(g_dmaPsramHeap, size);
    } else {
        ptr = sram_malloc(size);
    }
    return ptr;
}

void _dma_free( void *ptr , uint32_t flag )
{
    if(flag == DMAHEAP_PSRAM) {
        if(g_dmaPsramHeap == NULL) {
            return;
        }
        sys_heap_free(g_dmaPsramHeap, ptr);
    } else {
        sram_free(ptr);
    }
}

void *_dma_calloc( size_t nmemb, size_t size, uint32_t flag )
{
    void* ptr;
    if(flag == DMAHEAP_PSRAM) {
        if(g_dmaPsramHeap == NULL) {
            if(dma_PsramHeap_init() != 0) {
                return NULL;
            }
        }
        ptr = sys_heap_calloc(g_dmaPsramHeap, nmemb, size);
    } else {
        ptr = sram_calloc(nmemb, size);
    }
    return ptr;
}

void *_dma_realloc(    void *ptr, size_t size, uint32_t flag )
{
    if (flag == DMAHEAP_PSRAM) {
        if (ptr == NULL) {
            ptr = dma_malloc(size, flag);
        } else if (size == 0) {
            dma_free(ptr, flag);
        } else {
            ptr = sys_heap_realloc(g_dmaPsramHeap, ptr, size);
        }
    } else {
        ptr = sram_realloc(ptr, size);
    }
    return ptr;
}

#endif /* ((defined __CONFIG_PSRAM) && (__CONFIG_DMAHEAP_PSRAM_SIZE != 0)) */
