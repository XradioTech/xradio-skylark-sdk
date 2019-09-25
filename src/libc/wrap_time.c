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

#include <time.h>
#include <sys/time.h>
#include "driver/chip/hal_rtc.h"
#include "driver/chip/hal_prcm.h"


/* save time baseline of h/w timer in register or not */
#if (__CONFIG_CHIP_ARCH_VER == 2)
#define TIMEOFDAY_BASELINE_SAVE_IN_REG  1
#else
#define TIMEOFDAY_BASELINE_SAVE_IN_REG  0
#endif

/* microseconds per second */
#define TIMEOFDAY_USEC_PER_SEC          1000000L

/* default baseline for timeofday is 2018-01-01 00:00:00 */
#define TIMEOFDAY_DEFAULT_BASELINE      1514764800000000LL

#if TIMEOFDAY_BASELINE_SAVE_IN_REG

static __always_inline void timeofday_set_baseline(int64_t tm)
{
	HAL_PRCM_SetPrivateTime(tm);
}

static __always_inline int64_t timeofday_get_baseline(void)
{
	return HAL_PRCM_GetPrivateTime();
}

#else /* TIMEOFDAY_BASELINE_SAVE_IN_REG */

/* variable to save time baseline of h/w timer */
static int64_t s_tmbl = TIMEOFDAY_DEFAULT_BASELINE;

static __always_inline void timeofday_set_baseline(int64_t tm)
{
	s_tmbl = tm;
}

static __always_inline int64_t timeofday_get_baseline(void)
{
	return s_tmbl;
}

#ifndef __CONFIG_BOOTLOADER
void timeofday_save(void)
{
	int64_t tm = timeofday_get_baseline();
	HAL_PRCM_SetCPUAPrivateData(0, (uint32_t)(tm & 0xffffffff));
	HAL_PRCM_SetCPUABootAddr((uint32_t)((tm >> 32) & 0xffffffff));
}

void timeofday_restore(void)
{
	uint32_t high = HAL_PRCM_GetCPUABootAddr();
	uint32_t low = HAL_PRCM_GetCPUAPrivateData(0);
	if (high != 0 || low != 0) {
		timeofday_set_baseline(((int64_t)high << 32) | low);
	}
}
#endif /* __CONFIG_BOOTLOADER */

#endif /* TIMEOFDAY_BASELINE_SAVE_IN_REG */

int __wrap_settimeofday(const struct timeval *tv, const struct timezone *tz)
{
	int64_t tm;

	if (tv) {
		tm = HAL_RTC_GetFreeRunTime();
		timeofday_set_baseline((int64_t)tv->tv_sec * TIMEOFDAY_USEC_PER_SEC +
		                       tv->tv_usec - tm);
	}

	return 0;
}

int __wrap_gettimeofday(struct timeval *tv, void *tz)
{
	int64_t tm, baseline;

	if (tv) {
		baseline = timeofday_get_baseline();
#if TIMEOFDAY_BASELINE_SAVE_IN_REG
		if (baseline == 0ULL) {
			baseline = TIMEOFDAY_DEFAULT_BASELINE;
			timeofday_set_baseline(baseline);
		}
#endif
		tm = HAL_RTC_GetFreeRunTime() + baseline;
		tv->tv_sec = tm / TIMEOFDAY_USEC_PER_SEC;
		tv->tv_usec = tm % TIMEOFDAY_USEC_PER_SEC;
	}

	return 0;
}

time_t __wrap_time(time_t *t)
{
	struct timeval tv;

	__wrap_gettimeofday(&tv, NULL);
	if (t) {
		*t = tv.tv_sec;
	}

	return tv.tv_sec;
}
