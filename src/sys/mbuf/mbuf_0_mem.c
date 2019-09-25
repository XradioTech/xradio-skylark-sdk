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

#if (__CONFIG_MBUF_IMPL_MODE == 0)

#include "kernel/os/os_thread.h"
#include "sys/mbuf_0.h"
#include "mbuf_0_mem.h"
#include "mbuf_debug.h"

#if MB0_MEM_TRACE_DETAIL
#define MBUF_MEM_MAX_CNT    256

struct mbuf_mem {
	size_t   size;
	uint16_t cnt;
	uint16_t cnt_max;
};

static struct mbuf_mem m_mem[MBUF_MEM_MAX_CNT];
#endif /* MB0_MEM_TRACE_DETAIL */

#if MB0_MEM_TRACE_SUM
static uint32_t m_mem_sum = 0;
static uint32_t m_mem_sum_max = 0;
#endif /* MB0_MEM_TRACE_SUM */

static __inline void mbuf_mutex_lock(void)
{
	OS_ThreadSuspendScheduler();
}

static __inline void mbuf_mutex_unlock(void)
{
	OS_ThreadResumeScheduler();
}

void mbuf_mem_info(int verbose)
{
	mbuf_mutex_lock();

#if MB0_MEM_TRACE_SUM
	MBUF_LOG(1, "<<< mbuf mem info >>>\n"
	            "m_mem_sum     %5u (%2u KB)\n"
	            "m_mem_sum_max %5u (%2u KB)\n",
	         m_mem_sum, m_mem_sum / 1024,
	         m_mem_sum_max, m_mem_sum_max / 1024);
#endif /* MB0_MEM_TRACE_SUM */

#if MB0_MEM_TRACE_DETAIL
	if (!verbose) {
		mbuf_mutex_unlock();
		return;
	}

	int i;
#if MB0_MEM_TRACE_SUM
	uint32_t total = 0;
#endif
	for (i = 0; i < MBUF_MEM_MAX_CNT; ++i) {
		if (m_mem[i].size != 0) {
#if MB0_MEM_TRACE_SUM
			total += m_mem[i].size * m_mem[i].cnt;
#endif
			MBUF_LOG(1, "%04d. %4u, %8u, %8u\n", i + 1,
			         m_mem[i].size, m_mem[i].cnt, m_mem[i].cnt_max);
		}
	}
#if MB0_MEM_TRACE_SUM
	if (total != m_mem_sum) {
		MBUF_ERR("total %u != used %u\n", total, m_mem_sum);
	}
#endif /* MB0_MEM_TRACE_SUM */
#endif /* MB0_MEM_TRACE_DETAIL */

	mbuf_mutex_unlock();
}

void *mbuf_malloc(size_t size)
{
	void *ptr = malloc(size);

	mbuf_mutex_lock();
	if (ptr) {
#if MB0_MEM_TRACE_SUM
		m_mem_sum += size;
		if (m_mem_sum_max < m_mem_sum) {
			m_mem_sum_max = m_mem_sum;
		}
#endif /* MB0_MEM_TRACE_SUM */

#if MB0_MEM_TRACE_DETAIL
		int i;
		for (i = 0; i < MBUF_MEM_MAX_CNT; ++i) {
			if (m_mem[i].size == size) {
				if (++m_mem[i].cnt > m_mem[i].cnt_max) {
					m_mem[i].cnt_max = m_mem[i].cnt;
				}
				break;
			} else if (m_mem[i].size == 0) {
				m_mem[i].size = size;
				m_mem[i].cnt = 1;
				m_mem[i].cnt_max = 1;
				break;
			}
		}
		if (i >= MBUF_MEM_MAX_CNT) {
			MBUF_WRN("count exceed %u\n", MBUF_MEM_MAX_CNT);
		}
#endif /* MB0_MEM_TRACE_DETAIL */
	} else {
#if MBUF_OPT_LIMIT_MEM
		MBUF_WRN("malloc %u fail\n", size);
#else
		MBUF_DBG("malloc %u fail\n", size);
#endif
	}
	mbuf_mutex_unlock();
	return ptr;
}

void mbuf_free(void *ptr)
{
	mbuf_mutex_lock();
	if (ptr) {
#if (MB0_MEM_TRACE_SUM || MB0_MEM_TRACE_DETAIL)
		struct mbuf *m = ptr;
		size_t size = sizeof(struct mbuf) + m->m_len + m->m_headspace + m->m_tailspace;
#endif /* (MB0_MEM_TRACE_SUM || MB0_MEM_TRACE_DETAIL) */

#if MB0_MEM_TRACE_SUM
		m_mem_sum -= size;
#endif /* MB0_MEM_TRACE_SUM */

#if MB0_MEM_TRACE_DETAIL
		int i;
		for (i = 0; i < MBUF_MEM_MAX_CNT; ++i) {
			if (m_mem[i].size == size) {
				if (m_mem[i].cnt == 0) {
					MBUF_ERR("free (%p, %u), cnt 0, i %d\n", ptr, size, i);
				} else {
					--m_mem[i].cnt;
				}
				break;
			} else if (m_mem[i].size == 0) {
				MBUF_ERR("free (%p, %u), missed, i %d\n", ptr, size, i);
				break;
			}
		}
		if (i >= MBUF_MEM_MAX_CNT) {
			MBUF_ERR("free (%p, %u), missed, i %d\n", ptr, size, i);
		}
#endif /* MB0_MEM_TRACE_DETAIL */
	}
	mbuf_mutex_unlock();
	free(ptr);
}

#endif /* (__CONFIG_MBUF_IMPL_MODE == 0) */
