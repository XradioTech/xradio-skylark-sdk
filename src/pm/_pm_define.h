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

#ifndef __PM_DEFINE_H
#define __PM_DEFINE_H

#include "sys/xr_debug.h"

#define CONFIG_PM_DEBUG
//#define CONFIG_PM_THREAD

#define PM_LOGD_ON 1
#define PM_LOGN_ON 1
#define PM_LOGW_ON 1
#define PM_LOGE_ON 1
#define PM_ABORT_ON 0

#define PM_DEBUG(flags, fmt, arg...)			\
	do {						\
		if(flags) 				\
			printf("PMA: "fmt, ##arg);	\
	} while (0)

#define PM_LOGD(format, args...) PM_DEBUG(PM_LOGD_ON, format, ##args)
#define PM_LOGN(format, args...) PM_DEBUG(PM_LOGN_ON, format, ##args)
#define PM_LOGW(format, args...) PM_DEBUG(PM_LOGW_ON, format, ##args)
#define PM_LOGE(format, args...) PM_DEBUG(PM_LOGE_ON, format, ##args)
#define PM_LOGA(format, args...) do {printf("PMA: "format, ##args);} while (0)
#if PM_ABORT_ON
#define PM_ABORT() while (1)
#else
#define PM_ABORT()
#endif

#define PM_BUG_ON(d, v)                                                         \
	do {                                                                    \
		if (v) {                                                        \
			printf("PMA: BUG at %s:%d dev:%s(%p)!\n", __func__,     \
			       __LINE__, (d && ((struct soc_device *)d)->name) ?\
			       ((struct soc_device *)d)->name : "NULL", d);     \
			PM_ABORT();                                             \
		}                                                               \
	} while (0)
#define PM_WARN_ON(d, v)                                                        \
	do {                                                                    \
		if(v) {                                                         \
			printf("PMA: WARN at %s:%d dev:%s(%p)!\n", __func__,    \
			       __LINE__, (d && ((struct soc_device *)d)->name) ?\
			       ((struct soc_device *)d)->name : "NULL", d);     \
		}                                                               \
	} while (0)

#endif /* __PM_DEFINE_H */
