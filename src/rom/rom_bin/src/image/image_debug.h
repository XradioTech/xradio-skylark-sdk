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

#ifndef _IMAGE_DEBUG_H_
#define _IMAGE_DEBUG_H_

#include "rom/libc/stdio.h"
#include "rom/sys/xr_util.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __CONFIG_ROM

extern uint16_t image_dbg_mask;

#define IMAGE_DBG_FLAG  (1 << 0)
#define IMAGE_WRN_FLAG  (1 << 1)
#define IMAGE_ERR_FLAG  (1 << 2)

#define FDCM_DBG_FLAG   (1 << 4)
#define FDCM_WRN_FLAG   (1 << 5)
#define FDCM_ERR_FLAG   (1 << 6)

#define FLASH_DBG_FLAG  (1 << 8)
#define FLASH_WRN_FLAG  (1 << 9)
#define FLASH_ERR_FLAG  (1 << 10)

#define IMAGE_DBG_ON    (image_dbg_mask & IMAGE_DBG_FLAG)
#define IMAGE_WRN_ON    (image_dbg_mask & IMAGE_WRN_FLAG)
#define IMAGE_ERR_ON    (image_dbg_mask & IMAGE_ERR_FLAG)
#define IMAGE_ABORT_ON  0

#define FDCM_DBG_ON     (image_dbg_mask & FDCM_DBG_FLAG)
#define FDCM_WRN_ON     (image_dbg_mask & FDCM_WRN_FLAG)
#define FDCM_ERR_ON     (image_dbg_mask & FDCM_ERR_FLAG)
#define FDCM_ABORT_ON   0

#define FLASH_DBG_ON    (image_dbg_mask & FLASH_DBG_FLAG)
#define FLASH_WRN_ON    (image_dbg_mask & FLASH_WRN_FLAG)
#define FLASH_ERR_ON    (image_dbg_mask & FLASH_ERR_FLAG)
#define FLASH_ABORT_ON  0

#else /* __CONFIG_ROM */

#define IMAGE_DBG_ON    0
#define IMAGE_WRN_ON    0
#define IMAGE_ERR_ON    1
#define IMAGE_ABORT_ON  0

#define FDCM_DBG_ON     0
#define FDCM_WRN_ON     0
#define FDCM_ERR_ON     1
#define FDCM_ABORT_ON   0

#define FLASH_DBG_ON    0
#define FLASH_WRN_ON    0
#define FLASH_ERR_ON    1
#define FLASH_ABORT_ON  0

#endif /* __CONFIG_ROM */

#define IMAGE_SYSLOG    printf
#define IMAGE_ABORT()   sys_abort()

#define IMAGE_LOG(flags, fmt, arg...)   \
    do {                                \
        if (flags)                      \
            IMAGE_SYSLOG(fmt, ##arg);   \
    } while (0)

#define IMAGE_DBG(fmt, arg...)  IMAGE_LOG(IMAGE_DBG_ON, "[img] "fmt, ##arg)
#define IMAGE_WRN(fmt, arg...)  IMAGE_LOG(IMAGE_WRN_ON, "[img W] "fmt, ##arg)
#define IMAGE_ERR(fmt, arg...)                          \
    do {                                                \
        IMAGE_LOG(IMAGE_ERR_ON, "[img E] %s():%d, "fmt,	\
                  __func__, __LINE__, ##arg);           \
        if (IMAGE_ABORT_ON)                             \
            IMAGE_ABORT();                              \
    } while (0)

#define FDCM_SYSLOG     printf
#define FDCM_ABORT()    sys_abort()

#define FDCM_LOG(flags, fmt, arg...)    \
    do {                                \
        if (flags)                      \
            FDCM_SYSLOG(fmt, ##arg);    \
    } while (0)

#define FDCM_DBG(fmt, arg...)   FDCM_LOG(FDCM_DBG_ON, "[FDCM] "fmt, ##arg)
#define FDCM_WRN(fmt, arg...)   FDCM_LOG(FDCM_WRN_ON, "[FDCM W] "fmt, ##arg)
#define FDCM_ERR(fmt, arg...)                           \
    do {                                                \
        FDCM_LOG(FDCM_ERR_ON, "[FDCM E] %s():%d, "fmt,  \
                 __func__, __LINE__, ##arg);            \
        if (FDCM_ABORT_ON)                              \
            FDCM_ABORT();                               \
    } while (0)

#define FLASH_SYSLOG    printf
#define FLASH_ABORT()   sys_abort()

#define FLASH_LOG(flags, fmt, arg...)   \
    do {                                \
        if (flags)                      \
            FLASH_SYSLOG(fmt, ##arg);   \
    } while (0)

#define FLASH_DBG(fmt, arg...)  FLASH_LOG(FLASH_DBG_ON, "[flash] "fmt, ##arg)
#define FLASH_WRN(fmt, arg...)  FLASH_LOG(FLASH_WRN_ON, "[flash W] "fmt, ##arg)
#define FLASH_ERR(fmt, arg...)                              \
    do {                                                    \
        FLASH_LOG(FLASH_ERR_ON, "[flash E] %s():%d, "fmt,   \
                  __func__, __LINE__, ##arg);               \
        if (FLASH_ABORT_ON)                                 \
            FLASH_ABORT();                                  \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif /* _IMAGE_DEBUG_H_ */
