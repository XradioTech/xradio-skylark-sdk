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
#include <string.h>
#include "sys/fdcm.h"
#include "common/framework/platform_init.h"

#define FLASH_DEVICE_NUM         0
#define FDCM_FLASH_START_ADDR (1024 * 1024)
#define FDCM_SIZE             (4 * 1024)

struct wifi_info {
	char ssid[64];
	char password[64];
};

static int fdcm_example()
{
	fdcm_handle_t *fdcm;
	struct wifi_info wifi;
	struct wifi_info info;

	fdcm = fdcm_open(FLASH_DEVICE_NUM, FDCM_FLASH_START_ADDR, FDCM_SIZE);
	if (fdcm == NULL) {
		printf("fdcm open fail.\n");
		return -1;
	}
	printf("fdcm open success, flash addr:0x%x, flash size:%d\n", FDCM_FLASH_START_ADDR, FDCM_SIZE);
	printf("we can use fdcm to save info to flash.\n");

	printf("====fdcm write/read example1====\n");
	strcpy(wifi.ssid, "first_ssid");
	strcpy(wifi.password, "first_password");
	fdcm_write(fdcm, &wifi, sizeof(wifi));
	fdcm_read(fdcm, &info, sizeof(info));
	printf("[write ssid:%s, write password:%s] [read ssid:%s, read password:%s]\n", wifi.ssid, wifi.password, info.ssid, info.password);

	printf("====fdcm write/read example2====\n");
	strcpy(wifi.ssid, "second_ssid");
	strcpy(wifi.password, "second_password");
	fdcm_write(fdcm, &wifi, sizeof(wifi));
	fdcm_read(fdcm, &info, sizeof(info));
	printf("[write ssid:%s, write password:%s] [read ssid:%s, read password:%s]\n", wifi.ssid, wifi.password, info.ssid, info.password);

	printf("====fdcm write/read example3====\n");
	strcpy(wifi.ssid, "third_ssid");
	strcpy(wifi.password, "third_password");
	fdcm_write(fdcm, &wifi, sizeof(wifi));
	fdcm_read(fdcm, &info, sizeof(info));
	printf("[write ssid:%s, write password:%s] [read ssid:%s, read password:%s]\n", wifi.ssid, wifi.password, info.ssid, info.password);

	fdcm_close(fdcm);

	return 0;
}

int main(void)
{
	platform_init();

	fdcm_example();

	return 0;
}
