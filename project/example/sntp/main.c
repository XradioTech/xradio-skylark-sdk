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
#include "kernel/os/os.h"
#include "common/framework/platform_init.h"
#include "net/wlan/wlan.h"
#include "common/framework/net_ctrl.h"
#include "net/sntp/sntp.h"

#define SNTP_YEAR_OFFSET			(2000)

static uint8_t wlan_isConnect = 0;

static void wlan_msg_recv(uint32_t event, uint32_t data, void *arg)
{
    uint16_t type = EVENT_SUBTYPE(event);

    switch (type) {
    case NET_CTRL_MSG_NETWORK_UP:
        wlan_isConnect = 1;
		printf("<sntp> network connected\n");
        break;
    case NET_CTRL_MSG_NETWORK_DOWN:
		wlan_isConnect = 0;
		printf("<sntp> network disconnect\n");
        break;
    default:
        break;
    }
}

static int wlan_msg_init(void)
{
    observer_base *ob = sys_callback_observer_create(CTRL_MSG_TYPE_NETWORK,
                        NET_CTRL_MSG_ALL,
                        wlan_msg_recv,
                        NULL);
    if (ob == NULL) {
		printf("<sntp> net observer create fail\n");
        return -1;
    }
    if (sys_ctrl_attach(ob) != 0)
        return -1;

    return 0;
}

static void sntp_test()
{
	wlan_msg_init();

	printf("<sntp> please connect the network firstly!\n");

	do {
		if (wlan_isConnect) {
			printf("\n<sntp> sntp request...\n");
			if (sntp_request(NULL) != 0) {
				printf("<sntp> sntp request fail\n");
				break;
			}
			sntp_time *time = (sntp_time *)sntp_obtain_time();
			printf("<sntp> sntp request success\n");
			printf("<sntp> %u-%02u-%02u ", time->year + SNTP_YEAR_OFFSET, time->mon, time->day);
        	printf("%02u:%02u:%02u\n", time->hour, time->min, time->sec);
		}
		OS_MSleep(5000);
	} while(1);
}

int main(void)
{
	platform_init();

	printf("sntp demo start\n\n");

	sntp_test();

	printf("\nsntp demo over\n");
	return 0;
}

