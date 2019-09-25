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

#ifndef _MBUF_0_MEM_H_
#define _MBUF_0_MEM_H_

#if (__CONFIG_MBUF_IMPL_MODE == 0)

#include <stdlib.h>

#define MB0_MEM_TRACE_SUM       0 /* trace memory usage sum */
#define MB0_MEM_TRACE_DETAIL    0 /* trace memory usage detail */

#if (MB0_MEM_TRACE_SUM || MB0_MEM_TRACE_DETAIL)

void *mbuf_malloc(size_t size);
void mbuf_free(void *ptr);

#define MB_MALLOC(l)    mbuf_malloc(l)
#define MB_FREE(p)      mbuf_free(p)

#else /* (MB0_MEM_TRACE_SUM || MB0_MEM_TRACE_DETAIL) */

#define MB_MALLOC(l)    malloc(l)
#define MB_FREE(p)      free(p)

#endif /* (MB0_MEM_TRACE_SUM || MB0_MEM_TRACE_DETAIL) */

#endif /* (__CONFIG_MBUF_IMPL_MODE == 0) */
#endif /* _MBUF_0_MEM_H_ */
