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

#include "common/framework/platform_init.h"
#include "common/framework/sys_ctrl/sys_ctrl.h"
#include "common/framework/net_ctrl.h"
#include "soft_ap_config.h"
#include "kernel/os/os.h"

static char *softap_ssid = "XRADIO_SOFT_AP_CONFIG_TEST";
static int net_state = -1;
static soft_ap_config_result soft_ap_result;
static SOFT_AP_CONFIG_STA soft_ap_state;

static void net_cb(uint32_t event, uint32_t data, void *arg)
{
	uint16_t type = EVENT_SUBTYPE(event);

	net_state = type;
}

static void soft_ap_config_callback(soft_ap_config_result *result,
												SOFT_AP_CONFIG_STA state)
{
	/* copy the result and state */
	memcpy(&soft_ap_result, result, sizeof(soft_ap_result));
	soft_ap_state = state;

	printf("ssid:%s psk:%s state:%d\n", result->ssid, result->psk, state);
}

int main(void)
{
	int soft_ap_has_start = 0;

	platform_init();

	/* create an observer to monitor the net work state */
	static observer_base *net_ob;
	net_ob = sys_callback_observer_create(CTRL_MSG_TYPE_NETWORK,
									     NET_CTRL_MSG_ALL,
									     net_cb,
									     NULL);
	if(net_ob == NULL)
		return -1;
	if(sys_ctrl_attach(net_ob) != 0)
		return -1;

	/* set to ap mode */
	net_switch_mode(WLAN_MODE_HOSTAP);
	wlan_ap_disable();
	wlan_ap_set((unsigned char*)softap_ssid, strlen(softap_ssid), NULL);
	wlan_ap_enable();

	/* set soft_ap_config callback */
	soft_ap_config_set_cb(soft_ap_config_callback);
	while (1) {
		if (net_state == NET_CTRL_MSG_NETWORK_UP && !soft_ap_has_start) {
			/* if the network is up, start the soft_ap_config */
			soft_ap_config_start();
			soft_ap_has_start = 1;
		}

		OS_MSleep(100);
	}

	return 0;
}
