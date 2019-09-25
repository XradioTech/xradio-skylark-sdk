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
#include "smartlink/voice_print/voice_print.h"
#include "common/framework/platform_init.h"
#include "common/framework/net_ctrl.h"

#include "smartlink/sc_assistant.h"
#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"
#include "lwip/netif.h"

#define VP_TIME_OUT_MS 120000

uint8_t vp_result_checksum(uint8_t * buf, int len)
{
	int i;
	uint8_t cs = 0;

	for (i = 0;i < len;i++) {
		cs += buf[i];
	}
	return cs;
}

static int vp_result_parser(char *result_str, int result_len, wlan_voiceprint_result_t *vp_result)
{
	const char *str_find;
	char temp[3];
	int len_temp;

	/* ssid length */
	temp[2] = '\0';
	str_find = result_str;
	memcpy(temp, str_find, 2);
	len_temp = (int)strtol(temp, NULL, 16);
	if (len_temp > WLAN_SSID_MAX_LEN) {
		printf("invalid ssid len %d\n", len_temp);
		return -1;
	}
	vp_result->ssid_len = len_temp;
	str_find += 2;
	memcpy(vp_result->ssid, str_find, len_temp);
	str_find += vp_result->ssid_len;

#if (VOICE_PRINT_POLICY == 1)
	/* passphrase length */
	memcpy(temp, str_find, 2);
	len_temp = (int)strtol(temp, NULL, 16);
	if (len_temp != 0) {
		if (len_temp < WLAN_PASSPHRASE_MIN_LEN || len_temp > WLAN_PASSPHRASE_MAX_LEN) {
			printf("invalid psk len %d\n", len_temp);
			return -1;
		}
	}
	str_find += 2;
	memcpy(vp_result->passphrase, str_find, len_temp);
	vp_result->passphrase[len_temp] = '\0';

	/* checksum */
	str_find += len_temp;
	memcpy(temp, str_find, 2);
	uint8_t cs = (uint8_t)strtol(temp, NULL, 16);
	len_temp = 2 + vp_result->ssid_len + 2 + len_temp;
	cs += vp_result_checksum((uint8_t *)result_str, len_temp);
	if (0xFF != cs) {
		printf("cs err: 0x%x\n", cs);
		return -1;
	}
#elif (VOICE_PRINT_POLICY == 2)
	/* passphrase */
	len_temp = result_len - 2 - vp_result->ssid_len;
	if (len_temp != 0) {
		if (len_temp < WLAN_PASSPHRASE_MIN_LEN || len_temp > WLAN_PASSPHRASE_MAX_LEN) {
			printf("invalid psk len %d\n", len_temp);
			return -1;
		}
	}
	memcpy(vp_result->passphrase, str_find, len_temp);
	vp_result->passphrase[len_temp] = '\0';
#endif

	return 0;
}

int voice_print_example()
{
	int ret;
	int len;
	voiceprint_ret_t vp_ret;
	voiceprint_status_t status;
	wlan_voiceprint_result_t vp_result;
	voiceprint_param_t vp_param;
	sc_assistant_fun_t sca_fun;
	sc_assistant_time_config_t config;
	uint8_t *psk;
	char result[128];

	printf("try to init sc_assistant.\n");
	sc_assistant_get_fun(&sca_fun);
	config.time_total = VP_TIME_OUT_MS;
	config.time_sw_ch_long = 0;
	config.time_sw_ch_short = 0;
	ret = sc_assistant_init(g_wlan_netif, &sca_fun, &config);
	if (ret) {
		printf("sc_assistant_init fail.\n");
		return -1;
	}

	printf("try to start voiceprint.\n");
	vp_param.audio_card = AUDIO_SND_CARD_DEFAULT;
	vp_param.nif = g_wlan_netif;
	ret = voiceprint_start(&vp_param);
	if (ret) {
		printf("voiceprint_start fail.\n");
		goto err0;
	}

	printf("waiting for result.\n");
	vp_ret = voiceprint_wait(VP_TIME_OUT_MS);
	if (vp_ret != WLAN_VOICEPRINT_SUCCESS) {
		printf("voiceprint_wait fail.\n");
		goto err1;
	}

	status = voiceprint_get_status();
	if (status != VP_STATUS_COMPLETE) {
		printf("voiceprint status is not completed.\n");
		goto err1;
	}

	printf("try to get raw result.\n");
	len = sizeof(result) - 1;
	vp_ret = wlan_voiceprint_get_raw_result(result, &len);
	if (vp_ret != WLAN_VOICEPRINT_SUCCESS) {
		printf("voiceprint get raw result fail.");
		goto err1;
	}

	printf("try to parser raw result.\n");
	ret = vp_result_parser(result, len, &vp_result);
	if (ret) {
		printf("parser raw result fail.\n");
		goto err1;
	}
	printf("result: [ssid:%s] [password:%s]\n", vp_result.ssid, vp_result.passphrase);

	g_wlan_netif = sc_assistant_open_sta();
	if (vp_result.passphrase[0] != '\0') {
		psk = vp_result.passphrase;
	} else {
		psk = NULL;
	}
	printf("try to connect ap.\n");
	ret = sc_assistant_connect_ap(vp_result.ssid, vp_result.ssid_len, psk, VP_TIME_OUT_MS);
	if (ret < 0) {
		printf("connect ap time out\n");
		goto err1;
	}

	return 0;

err1:
	voiceprint_stop(0);
err0:
	sc_assistant_deinit(g_wlan_netif);
	return -1;
}

int main(void)
{
	platform_init();

	printf("voice print start.\n");

	voice_print_example();

	return 0;
}
