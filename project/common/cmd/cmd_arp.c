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

#if PRJCONF_NET_EN

#include "cmd_util.h"
#include "common/framework/net_ctrl.h"
#include "lwip/netifapi.h"
#include "netif/etharp.h"

enum cmd_status arp_clean_table(char *cmd)
{
	if (g_wlan_netif == NULL) {
		return CMD_STATUS_FAIL;
	}
	return netifapi_netif_common(g_wlan_netif, etharp_cleanup_netif, NULL);
}

enum cmd_status arp_state(char *cmd)
{
/*	void etharp_printtable_netif(struct netif *netif);
	return netifapi_netif_common(netif, etharp_printtable_netif, NULL);
*/
	return 0;
}

enum cmd_status arp_del(char *cmd)
{
	return 0;
}

enum cmd_status arp_set(char *cmd)
{
	return 0;
}

static enum cmd_status cmd_arp_help_exec(char *cmd);

static const struct cmd_data g_arp_cmds[] = {
	{ "clean",	    arp_clean_table, CMD_DESC("clean the arp table") },
	{ "state",	    arp_state, CMD_DESC("not support") },
	{ "del",	    arp_del, CMD_DESC("not support") },
	{ "set",	    arp_set, CMD_DESC("not support") },
	{ "help",	    cmd_arp_help_exec, CMD_DESC(CMD_HELP_DESC) },
};

static enum cmd_status cmd_arp_help_exec(char *cmd)
{
	cmd_help_exec(g_arp_cmds, cmd_nitems(g_arp_cmds), 8);
	return 0;
}

enum cmd_status cmd_arp_exec(char *cmd)
{
	int ret;

	ret = cmd_exec(cmd, g_arp_cmds, cmd_nitems(g_arp_cmds));
	return (ret == 0 ? CMD_STATUS_OK : CMD_STATUS_FAIL);
}

#endif /* PRJCONF_NET_EN */
