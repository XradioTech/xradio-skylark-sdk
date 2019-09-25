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
#include "lwip/inet.h"
#include "net/udhcp/usr_dhcpd.h"

#define AP_SSID					"XR872_AP"
#define AP_PSK					"12345678"

#define DHCPD_IP_ADDR_START		"192.168.51.100"
#define DHCPD_IP_ADDR_END		"192.168.51.150"
#define DHCPD_LEASE_TIME		60 * 60
#define DHCPD_LEASES_MAX		5

static void wlan_set_ap_mode(void)
{
	/* switch to ap mode */
	net_switch_mode(WLAN_MODE_HOSTAP);

	/* disable AP to set params*/
	wlan_ap_disable();

	/* set ap's ssid and password */
	wlan_ap_set((uint8_t *)AP_SSID, strlen(AP_SSID), (uint8_t *)AP_PSK);

	/* enable ap mode again */
	wlan_ap_enable();

	OS_MSleep(100);
}

static void dhcpd_restart(const struct dhcp_server_info *arg)
{
	printf("<dhcp> dhcp stop.\n");

	dhcp_server_stop();

	printf("<dhcp> dhcp start.\n");

	dhcp_server_start(arg);
}

static void dhcpd_demo()
{
	printf("<dhcp> wlan set AP mode...\n");

	wlan_set_ap_mode();

	printf("<dhcp> dhcp restart...\n");
	struct dhcp_server_info dhcpd_info;
	dhcpd_info.addr_start = inet_addr(DHCPD_IP_ADDR_START);
	dhcpd_info.addr_end	= inet_addr(DHCPD_IP_ADDR_END);
	dhcpd_info.lease_time = DHCPD_LEASE_TIME;
	dhcpd_info.max_leases	= DHCPD_LEASES_MAX;
	dhcpd_restart(&dhcpd_info);

	printf("<dhcp> please connect the AP...\n");
}

int main(void)
{
	platform_init();

	printf("dhcp demo start\n\n");

	dhcpd_demo();

	while (1)
		OS_MSleep(500);

	printf("\ndhcp demo over\n");
	return 0;
}

