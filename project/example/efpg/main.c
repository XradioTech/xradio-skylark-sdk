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
#include <stdint.h>
#include "efpg/efpg.h"
#include "common/framework/platform_init.h"

#if (__CONFIG_CHIP_ARCH_VER == 1)
#define EFPG_USER_AREA_LEN 601
#elif (__CONFIG_CHIP_ARCH_VER == 2)
#define EFPG_USER_AREA_LEN 259
#endif

static void efpg_log(uint8_t *buf, uint8_t size)
{
	int i;

	for (i = 0; i < size; i++)
		printf("[%02d] 0x%02x\n", i, buf[i]);
	printf("\n");
}

static void efpg_ua_log(uint8_t *buf, int len_bits)
{
	int len_bytes;

extern void print_hex_dump_bytes(const void *addr, size_t len);

	len_bytes = len_bits / 8;
	print_hex_dump_bytes(buf, len_bytes);
}

static int efpg_read_example()
{
	uint16_t ret;
	int len_bits;
	uint8_t buf[32];
	uint8_t ua_buf[128];

	ret = efpg_read_hosc(buf);
	if (ret == EFPG_ACK_OK) {
		printf("efpg read hosc success.\n");
		efpg_log(buf, 1);
	} else {
		printf("efpg read hosc fail.ret:%u\n", ret);
	}

	ret = efpg_read_boot(buf);
	if (ret == EFPG_ACK_OK) {
		printf("efpg read boot success.\n");
		efpg_log(buf, 32);
	} else {
		printf("efpg read boot fail.ret:%u\n", ret);
	}

	ret = efpg_read_mac(buf);
	if (ret == EFPG_ACK_OK) {
		printf("efpg read mac success.\n");
		efpg_log(buf, 6);
	} else {
		printf("efpg read mac fail.ret:%u\n", ret);
	}

	ret = efpg_read_chipid(buf);
	if (ret == EFPG_ACK_OK) {
		printf("efpg read chipid success.\n");
		efpg_log(buf, 16);
	} else {
		printf("efpg read chipid fail.ret:%u\n", ret);
	}

	ret = efpg_read_dcxo(buf);
	if (ret == EFPG_ACK_OK) {
		printf("efpg read dcxo success.\n");
		efpg_log(buf, 1);
	} else {
		printf("efpg read dcxo fail.ret:%u\n", ret);
	}

	ret = efpg_read_pout(buf);
	if (ret == EFPG_ACK_OK) {
		printf("efpg read pout success.\n");
		efpg_log(buf, 3);
	} else {
		printf("efpg read pout fail.ret:%u\n", ret);
	}

	len_bits = EFPG_USER_AREA_LEN;
	ret = efpg_read_user_area(0, len_bits, ua_buf);
	if (ret == EFPG_ACK_OK) {
		printf("efpg read user area success.\n");
		efpg_ua_log(ua_buf, len_bits);
	} else {
		printf("efpg read user area fail.ret:%u\n", ret);
	}

	return 0;
}

int main(void)
{
	platform_init();

	efpg_read_example();

	return 0;
}
