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

#ifndef _SYS_HEAP_H_
#define _SYS_HEAP_H_

#include <stdint.h>

/* Define the linked list structure.  This is used to link free blocks in order
of their memory address. */
typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK *pxNextFreeBlock;	/*<< The next free block in the list. */
    size_t xBlockSize;						/*<< The size of the free block. */
} BlockLink_t;

typedef struct sys_heap
{
    size_t heapBits_Per_Byte;
    uint8_t portByte_Alignment;
    uint8_t portByte_Alignment_Mask;
    size_t configTotal_Heap_Size;
    size_t heapMinmun_Block_Size;
    size_t xHeapStructSize;
    uint8_t *ucHeap;
    BlockLink_t xStart;
    BlockLink_t *pxEnd;
    size_t xFreeBytesRemaining;
    size_t xMinimumEverFreeBytesRemaining;
    size_t xBlockAllocatedBit;
} sys_heap_t;

#define SYSHEAP_DEFAULT_INIT(sysHeap, baseAddr, total_size) (sysHeap)->heapBits_Per_Byte = ( size_t ) 8; \
    (sysHeap)->portByte_Alignment = 8; \
    (sysHeap)->portByte_Alignment_Mask = 0x7; \
    (sysHeap)->xHeapStructSize = ( sizeof( BlockLink_t ) + ( ( size_t ) ( (sysHeap)->portByte_Alignment - 1 ) ) ) & ~( ( size_t ) (sysHeap)->portByte_Alignment_Mask ); \
    (sysHeap)->heapMinmun_Block_Size = ( ( size_t ) ( (sysHeap)->xHeapStructSize << 1 ) ); \
    (sysHeap)->xFreeBytesRemaining = 0; \
    (sysHeap)->xMinimumEverFreeBytesRemaining = 0; \
    (sysHeap)->xBlockAllocatedBit = 0; \
    (sysHeap)->pxEnd = NULL; \
    (sysHeap)->configTotal_Heap_Size = total_size; \
    (sysHeap)->ucHeap = baseAddr;

void sys_heap_free( sys_heap_t *sysHeap, void *ptr );
void *sys_heap_malloc( sys_heap_t *sysHeap, size_t size );
void *sys_heap_realloc( sys_heap_t *sysHeap, uint8_t *ptr,size_t size );
void *sys_heap_calloc( sys_heap_t *sysHeap, size_t nmemb, size_t size );

int sys_heap_init( sys_heap_t *sysHeap );

void *psram_malloc( size_t xWantedSize );
void *psram_realloc( void *pv, size_t xWantedSize );
void *psram_calloc( size_t xNmemb, size_t xMembSize );
void psram_free( void *pv );
char *psram_strdup(const char *s);

typedef void *(*sys_malloc_fn)(size_t size);
typedef void *(*sys_realloc_fn)(void *ptr, size_t size);
typedef void *(*sys_calloc_fn)(size_t nmemb, size_t size);
typedef void (*sys_free_fn)(void *ptr);

void wpa_set_heap_fn(sys_malloc_fn malloc_fn, sys_realloc_fn realloc_fn, sys_free_fn free_fn);
void umac_set_heap_fn(sys_malloc_fn malloc_fn, sys_free_fn free_fn);
void lmac_set_heap_fn(sys_malloc_fn malloc_fn, sys_free_fn free_fn);

#endif /*_SYS_HEAP_H_*/
