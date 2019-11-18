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

#ifdef __CONFIG_PSRAM

#include <stdlib.h>
#include <string.h>

extern void *_psram_malloc( size_t xWantedSize );
extern void _psram_free( void *pv );
extern void *_psram_realloc( void *pv, size_t xWantedSize );

#define PSRAM_MALLOC_MEM_TRACE	defined(__CONFIG_PSRAM_MALLOC_TRACE)

#if PSRAM_MALLOC_MEM_TRACE

#include <stdio.h>

#define PSRAM_HEAP_MEM_DBG_ON         0
#define PSRAM_HEAP_MEM_ERR_ON         1

#define PSRAM_HEAP_MEM_DBG_MIN_SIZE   100
#define PSRAM_HEAP_MEM_MAX_CNT        1024
#define PSRAM_HEAP_SYSLOG             printf

#define PSRAM_HEAP_MEM_IS_TRACED(size)    (size > PSRAM_HEAP_MEM_DBG_MIN_SIZE)

#define PSRAM_HEAP_MEM_LOG(flags, fmt, arg...)    \
    do {                                          \
        if (flags)                                \
            PSRAM_HEAP_SYSLOG(fmt, ##arg);        \
    } while (0)

#define PSRAM_HEAP_MEM_DBG(fmt, arg...) \
	PSRAM_HEAP_MEM_LOG(PSRAM_HEAP_MEM_DBG_ON, "[psram heap] "fmt, ##arg)

#define PSRAM_HEAP_MEM_ERR(fmt, arg...) \
	PSRAM_HEAP_MEM_LOG(PSRAM_HEAP_MEM_ERR_ON, "[psram heap ERR] %s():%d, "fmt, \
	                              __func__, __LINE__, ##arg);

struct psram_heap_mem {
	void *ptr;
	size_t size;
};

static struct psram_heap_mem g_psram_mem[PSRAM_HEAP_MEM_MAX_CNT];

static int g_psram_mem_entry_cnt = 0;
static int g_psram_mem_entry_cnt_max = 0;
static int g_psram_mem_empty_idx = 0; /* beginning idx to do the search for new one */

static size_t g_psram_mem_sum = 0;
static size_t g_psram_mem_sum_max = 0;

#define PSRAM_MEM_MAGIC_LEN	4

#if (PSRAM_MEM_MAGIC_LEN)
static const char g_psram_mem_magic[PSRAM_MEM_MAGIC_LEN] = {0x4a, 0x5b, 0x6c, 0x7f};
#define PSRAM_MEM_SET_MAGIC(p, l)	memcpy((((char *)(p)) + (l)), g_psram_mem_magic, 4)
#define PSRAM_MEM_CHK_MAGIC(p, l)	memcmp((((char *)(p)) + (l)), g_psram_mem_magic, 4)
#else
#define PSRAM_MEM_SET_MAGIC(p, l)	do { } while (0)
#define PSRAM_MEM_CHK_MAGIC(p, l)	0
#endif

#ifdef __CONFIG_OS_FREERTOS
#include "kernel/os/os_thread.h"
static __inline void psram_malloc_mutex_lock(void)
{
	OS_ThreadSuspendScheduler();
}

static __inline void psram_malloc_mutex_unlock(void)
{
	OS_ThreadResumeScheduler();
}
#endif /* __CONFIG_OS_FREERTOS */

uint32_t psram_malloc_heap_info(int verbose)
{
	psram_malloc_mutex_lock();

	PSRAM_HEAP_SYSLOG("<<< psram heap info >>>\n"
	            "g_psram_mem_sum       %u (%u KB)\n"
	            "g_psram_mem_sum_max   %u (%u KB)\n"
	            "g_psram_mem_entry_cnt %u, max %u\n",
	            g_psram_mem_sum, g_psram_mem_sum / 1024,
	            g_psram_mem_sum_max, g_psram_mem_sum_max / 1024,
	            g_psram_mem_entry_cnt, g_psram_mem_entry_cnt_max);

	int i, j = 0;
	for (i = 0; i < PSRAM_HEAP_MEM_MAX_CNT; ++i) {
		if (g_psram_mem[i].ptr != NULL) {
			if (verbose) {
				PSRAM_HEAP_SYSLOG("%03d. %03d, %p, %u\n",
				            ++j, i, g_psram_mem[i].ptr, g_psram_mem[i].size);
			}

			if (PSRAM_MEM_CHK_MAGIC(g_psram_mem[i].ptr, g_psram_mem[i].size)) {
				PSRAM_HEAP_MEM_ERR("psram mem (%p) corrupt\n", g_psram_mem[i].ptr);
			}
		}
	}

	uint32_t ret = g_psram_mem_sum;
	psram_malloc_mutex_unlock();

	return ret;
}

/* Note: @ptr != NULL */
static void psram_malloc_add_entry(void *ptr, size_t size)
{
	int i;

	PSRAM_MEM_SET_MAGIC(ptr, size);

	psram_malloc_mutex_lock();

	for (i = g_psram_mem_empty_idx; i < PSRAM_HEAP_MEM_MAX_CNT; ++i) {
		if (g_psram_mem[i].ptr == NULL) {
			g_psram_mem[i].ptr = ptr;
			g_psram_mem[i].size = size;
			g_psram_mem_entry_cnt++;
			g_psram_mem_empty_idx = i + 1;
			g_psram_mem_sum += size;
			if (g_psram_mem_sum > g_psram_mem_sum_max)
				g_psram_mem_sum_max = g_psram_mem_sum;
			if (g_psram_mem_entry_cnt > g_psram_mem_entry_cnt_max)
				g_psram_mem_entry_cnt_max = g_psram_mem_entry_cnt;
			break;
		}
	}
	psram_malloc_mutex_unlock();

	if (i >= PSRAM_HEAP_MEM_MAX_CNT) {
		PSRAM_HEAP_MEM_ERR("psram heap mem count exceed %d\n", PSRAM_HEAP_MEM_MAX_CNT);
	}
}

/* Note: @ptr != NULL */
static size_t psram_malloc_delete_entry(void *ptr)
{
	int i;
	size_t size;

	psram_malloc_mutex_lock();

	for (i = 0; i < PSRAM_HEAP_MEM_MAX_CNT; ++i) {
		if (g_psram_mem[i].ptr == ptr) {
			size = g_psram_mem[i].size;
			if (PSRAM_MEM_CHK_MAGIC(ptr, size)) {
				PSRAM_HEAP_MEM_ERR("psram mem f (%p, %u) corrupt\n", ptr, size);
			}
			g_psram_mem_sum -= size;
			g_psram_mem[i].ptr = NULL;
			g_psram_mem[i].size = 0;
			g_psram_mem_entry_cnt--;
			if (i < g_psram_mem_empty_idx)
				g_psram_mem_empty_idx = i;
			break;
		}
	}
	psram_malloc_mutex_unlock();

	if (i >= PSRAM_HEAP_MEM_MAX_CNT) {
		PSRAM_HEAP_MEM_ERR("psram heap mem entry (%p) missed\n", ptr);
		size = -1;
	}

	return size;
}

/* Note: @old_ptr != NULL, @new_ptr != NULL, @new_size != 0 */
static size_t psram_malloc_update_entry(void *old_ptr,
                                        void *new_ptr, size_t new_size)
{
	int i;
	size_t old_size;

	PSRAM_MEM_SET_MAGIC(new_ptr, new_size);

	psram_malloc_mutex_lock();

	for (i = 0; i < PSRAM_HEAP_MEM_MAX_CNT; ++i) {
		if (g_psram_mem[i].ptr == old_ptr) {
			old_size = g_psram_mem[i].size;
			g_psram_mem_sum = g_psram_mem_sum - old_size + new_size;
			g_psram_mem[i].ptr = new_ptr;
			g_psram_mem[i].size = new_size;
			if (g_psram_mem_sum > g_psram_mem_sum_max)
				g_psram_mem_sum_max = g_psram_mem_sum;
			break;
		}
	}
	psram_malloc_mutex_unlock();

	if (i >= PSRAM_HEAP_MEM_MAX_CNT) {
		PSRAM_HEAP_MEM_ERR("psram heap mem entry (%p) missed\n", new_ptr);
		old_size = -1;
	}

	return old_size;
}


void *psram_malloc(size_t size)
{
	void *ptr;

	size_t real_size = size + PSRAM_MEM_MAGIC_LEN;
	ptr = _psram_malloc(real_size);
	if (PSRAM_HEAP_MEM_IS_TRACED(size)) {
		PSRAM_HEAP_MEM_DBG("psram malloc (%p, %u)\n", ptr, size);
	}

	if (ptr) {
		psram_malloc_add_entry(ptr, size);
	} else {
		PSRAM_HEAP_MEM_ERR("psram heap mem exhausted (%u)\n", size);
	}

	return ptr;
}

void psram_free(void *ptr)
{
	if (ptr) {
		size_t size = psram_malloc_delete_entry(ptr);
		if (PSRAM_HEAP_MEM_IS_TRACED(size)) {
			PSRAM_HEAP_MEM_DBG("psram f (%p, %u)\n", ptr, size);
		}
	}

	_psram_free(ptr);
}

void *psram_realloc(void *ptr, size_t size)
{
	void *new_ptr;
	size_t old_size;
	size_t real_size;

	real_size = size;

	if (size != 0) { /* (size == 0) means free it */
		real_size += PSRAM_MEM_MAGIC_LEN;
	}

	new_ptr = _psram_realloc(ptr, real_size);

	if (ptr == NULL) {
		old_size = 0;
		if (new_ptr != NULL) {
			psram_malloc_add_entry(new_ptr, size);
		} else {
			if (size != 0) {
				PSRAM_HEAP_MEM_ERR("psram heap mem exhausted (%p, %u)\n", ptr, size);
				goto out;
			}
		}
	} else {
		if (size == 0) {
			if (new_ptr != NULL) {
				PSRAM_HEAP_MEM_ERR("psram realloc (%p, %u) return %p\n", ptr, size, new_ptr);
			}
			old_size = psram_malloc_delete_entry(ptr);
		} else {
			if (new_ptr != NULL) {
				old_size = psram_malloc_update_entry(ptr, new_ptr, size);
			} else {
				PSRAM_HEAP_MEM_ERR("psram heap mem exhausted (%p, %u)\n", ptr, size);
				goto out;
			}
		}
	}

	if (PSRAM_HEAP_MEM_IS_TRACED(size) || PSRAM_HEAP_MEM_IS_TRACED(old_size)) {
		PSRAM_HEAP_MEM_DBG("psram r (%p, %u) <- (%p, %u)\n", new_ptr, size, ptr, old_size);
	}

out:
	return new_ptr;
}

#else

void *psram_malloc(size_t xWantedSize)
{
	return _psram_malloc(xWantedSize);
}

void psram_free(void *pv)
{
	_psram_free(pv);
}

void *psram_realloc(void *pv, size_t xWantedSize)
{
	return _psram_realloc(pv, xWantedSize);
}

#endif

void *psram_calloc(size_t xNmemb, size_t xMembSize)
{
	void *ptr = psram_malloc(xNmemb * xMembSize);
	if(ptr != NULL) {
		memset(ptr, 0, xNmemb * xMembSize);
	}

	return ptr;
}

char *psram_strdup(const char *s)
{
	char *res;
	size_t len;

	if (s == NULL) {
		return NULL;
	}

	len = strlen(s);
	res = psram_malloc(len + 1);
	if (res) {
		memcpy(res, s, len + 1);
	}

	return res;
}

#endif
