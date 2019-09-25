/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions
 *	are met:
 *	  1. Redistributions of source code must retain the above copyright
 *		 notice, this list of conditions and the following disclaimer.
 *	  2. Redistributions in binary form must reproduce the above copyright
 *		 notice, this list of conditions and the following disclaimer in the
 *		 documentation and/or other materials provided with the
 *		 distribution.
 *	  3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *		 its contributors may be used to endorse or promote products derived
 *		 from this software without specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *	OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SOFT_AP_CONFIG_H_
#define _SOFT_AP_CONFIG_H_

#define SAC_DBG_ON     0
#define SAC_WRN_ON     1
#define SAC_ERR_ON     1

#define SAC_LOG(flags, fmt, arg...)  \
    do {                            \
        if (flags)                  \
            printf(fmt, ##arg);  \
    } while (0)

#define SAC_DBG(fmt, arg...) \
    do {                                            \
        SAC_LOG(SAC_DBG_ON, "[SAC DBG] <%s():%d> "fmt,  \
               __func__, __LINE__, ##arg);          \
    } while (0)

#define SAC_WRN(fmt, arg...) \
    do {                                            \
        SAC_LOG(SAC_WRN_ON, "[SAC WRN] <%s():%d> "fmt,  \
               __func__, __LINE__, ##arg);          \
    } while (0)

#define SAC_ERR(fmt, arg...)                         \
    do {                                            \
        SAC_LOG(SAC_ERR_ON, "[SAC ERR] <%s():%d> "fmt,  \
               __func__, __LINE__, ##arg);          \
    } while (0)

typedef struct {
	char ssid[32];
	char psk[32];
} soft_ap_config_result;

typedef enum {
	SOFT_AP_CONFIG_STOP,
	SOFT_AP_CONFIG_START,
	SOFT_AP_CONFIG_COMPLETE,
} SOFT_AP_CONFIG_STA;

typedef void (*soft_ap_config_cb)(soft_ap_config_result *result,
										SOFT_AP_CONFIG_STA state);

int soft_ap_config_start(void);
int soft_ap_config_stop(void);
int soft_ap_config_set_cb(soft_ap_config_cb cb);
int soft_ap_config_get_result(soft_ap_config_result *result);
int soft_ap_config_get_state(void);

#endif
