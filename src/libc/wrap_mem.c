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
#if (defined __CONFIG_PSRAM_ALL_CACHEABLE)
#include <stdio.h>
#include "driver/chip/chip.h"
extern void *___wrap_memmove(void *dst0, const void *src0, size_t len);
extern void *___wrap_memset(void *s, int c, size_t n);
extern void *___wrap_memcpy(void *dest, const void *src, size_t n);

#define rangof_invalid_addr(start, end) ( (((uint32_t)(start))>PRCM_BASE?((uint32_t)(start)):PRCM_BASE) \
                                        <= (((uint32_t)(end))>UART0_BASE?UART0_BASE:((uint32_t)(end))) )

void *__wrap_memmove(void *dst0, const void *src0, size_t len)
{
    if(rangof_invalid_addr((uint32_t)dst0, (uint32_t)dst0+len)) {
        printf("memm: %p, %p, %d\n", dst0, src0, len);
        return NULL;
    }
    return ___wrap_memmove(dst0, src0, len);
}
void *__wrap_memset(void *s, int c, size_t n)
{
    if(rangof_invalid_addr((uint32_t)s, (uint32_t)s+n)) {
        printf("mems: %p, %d, %d\n", s, c, n);
        return NULL;
    }
    return ___wrap_memset(s,c,n);
}

void *__wrap_memcpy(void *dest, const void *src, size_t n)
{
    if(rangof_invalid_addr((uint32_t)dest, (uint32_t)dest+n)) {
        printf("memc: %p, %p, %d\n", dest, src, n);
        return NULL;
    }
    return ___wrap_memcpy(dest, src, n);
}

#endif /*__CONFIG_PSRAM_ALL_CACHEABLE*/
