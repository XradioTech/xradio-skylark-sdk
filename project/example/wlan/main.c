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
#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"
#include "common/framework/net_ctrl.h"
#include "common/framework/platform_init.h"
#include "lwip/inet.h"

#define STA_MODE_TEST		1
#define AP_MODE_TEST		1
#define MON_MODE_TEST		1

#if STA_MODE_TEST
char * sta_ssid = "your_ap_name";
char * sta_psk = "your_ap_password";
void sta_test(void)
{
	/* switch to sta mode */
	net_switch_mode(WLAN_MODE_STA);

	/* set ssid and password to wlan */
	wlan_sta_set((uint8_t *)sta_ssid, strlen(sta_ssid), (uint8_t *)sta_psk);
	
	/* start scan and connect to ap automatically */
	wlan_sta_enable();

	OS_Sleep(60);

	/* After one minute, disconnect from AP */
	wlan_sta_disable();
}
#endif

#if AP_MODE_TEST
char * ap_ssid = "XR872_AP";
char * ap_psk = "12345678";
void ap_test(void)
{
	/* switch to ap mode */
	net_switch_mode(WLAN_MODE_HOSTAP);

	/* disable AP to set params*/
	wlan_ap_disable();
	
	/* set ap's ssid and password */
	wlan_ap_set((uint8_t *)ap_ssid, strlen(ap_ssid), (uint8_t *)ap_psk);
	
	/* enable ap mode again */
	wlan_ap_enable();

	OS_Sleep(60);

	/* After one minute, disable AP mode */
	wlan_ap_disable();
}
#endif

#if MON_MODE_TEST
void rx_cb(uint8_t *data, uint32_t len, void *info)
{
	if (!info) {
		printf("%s(), info NULL\n", __func__);
		return;
	}

	struct ieee80211_frame *wh;
	char * str_frm_type;
	wh = (struct ieee80211_frame *)data;

	switch (wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK)
	{
	case IEEE80211_FC0_TYPE_MGT:
		switch (wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK)
		{
		case IEEE80211_FC_STYPE_ASSOC_REQ:
			str_frm_type = "association req";
			break;
		case IEEE80211_FC_STYPE_ASSOC_RESP:
			str_frm_type = "association resp";
			break;
		case IEEE80211_FC_STYPE_REASSOC_REQ:
			str_frm_type = "reassociation req";
			break;
		case IEEE80211_FC_STYPE_REASSOC_RESP:
			str_frm_type = "reassociation resp";
			break;
		case IEEE80211_FC_STYPE_PROBE_REQ:
			str_frm_type = "probe req";
			break;
		case IEEE80211_FC_STYPE_PROBE_RESP:
			str_frm_type = "probe resp";
			break;
		case IEEE80211_FC_STYPE_BEACON:
			str_frm_type = "beacon";
			break;
		case IEEE80211_FC_STYPE_ATIM:
			str_frm_type = "atim";
			break;
		case IEEE80211_FC_STYPE_DISASSOC:
			str_frm_type = "disassociation";
			break;
		case IEEE80211_FC_STYPE_AUTH:
			str_frm_type = "authentication";
			break;
		case IEEE80211_FC_STYPE_DEAUTH:
			str_frm_type = "deauthentication";
			break;
		case IEEE80211_FC_STYPE_ACTION:
			str_frm_type = "action";
			break;
		default:
			str_frm_type = "unknown mgmt";
			break;
		}
		break;
	case IEEE80211_FC0_TYPE_CTL:
		str_frm_type = "control";
		break;
	case IEEE80211_FC0_TYPE_DATA:
		str_frm_type = "data";
		break;
	default:
		str_frm_type = "unknown";
		break;
	}
	printf("recv pack type:%s\n", str_frm_type);
}

void monitor_test(void)
{
	/* register rx callback first */
	wlan_monitor_set_rx_cb(g_wlan_netif, rx_cb);

	/* switch to monitor mode */
	net_switch_mode(WLAN_MODE_MONITOR);

	OS_Sleep(60);

	/* unregister rx callback */
	wlan_monitor_set_rx_cb(g_wlan_netif, NULL);

	/* After one minute, switch back to sta mode */
	net_switch_mode(WLAN_MODE_STA);
}
#endif

int main(void)
{
	platform_init();
#if	STA_MODE_TEST
	sta_test();
#endif
#if AP_MODE_TEST
	ap_test();
#endif
#if MON_MODE_TEST
	monitor_test();
#endif

	return 0;
}
