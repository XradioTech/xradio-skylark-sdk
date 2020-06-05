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

#include "common/cmd/cmd_util.h"
#include "common/cmd/cmd.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_gpio.h"

enum cmd_status cmd_prcm_exec(char *cmd)
{
  char *argv[3];

  cmd_parse_argv(cmd, argv, cmd_nitems(argv));

  if (!strcmp("set_top_ldo", argv[0])) {
    if (!strcmp("1.8", argv[1]))
      HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V8_DEFAULT);
    else if (!strcmp("1.4", argv[1]))
      HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V4);
    else if (!strcmp("1.7", argv[1]))
      HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V7);
    else if (!strcmp("1.9", argv[1]))
      HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V9);
    else if (!strcmp("2.0", argv[1]))
      HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_2V0);
  }

  return CMD_STATUS_OK;
}

#if PRJCONF_NET_EN

#define COMMAND_IPERF       1
#define COMMAND_PING        1
#define COMMAND_HTTPC       0
#define COMMAND_TLS         0
#define COMMAND_HTTPD       0
#define COMMAND_MQTT        0
#define COMMAND_NOPOLL      0
#define COMMAND_SNTP        0
#define COMMAND_DHCPD       0
#define COMMAND_BRROADCAST  0
#define COMMAND_ARP         0
#define COMMAND_WLAN        1


/*
 * net commands
 */
static enum cmd_status cmd_net_help_exec(char *cmd);

static const struct cmd_data g_net_cmds[] = {
	{ "mode",		cmd_wlan_mode_exec, CMD_DESC("mode command") },
#ifdef __CONFIG_WLAN_AP
	{ "ap", 		cmd_wlan_ap_exec, CMD_DESC("ap command") },
#endif
#ifdef __CONFIG_WLAN_STA
	{ "sta",		cmd_wlan_sta_exec, CMD_DESC("sta command") },
#endif
	{ "ifconfig",	cmd_ifconfig_exec, CMD_DESC("ifconfig command") },
	{ "smartconfig",cmd_smart_config_exec, CMD_DESC("smartconfig command") },
	{ "airkiss",	cmd_airkiss_exec, CMD_DESC("airkiss command") },
	{ "smartlink",	cmd_smartlink_exec, CMD_DESC("smartlink command") },

#if COMMAND_IPERF
	{ "iperf",		cmd_iperf_exec, CMD_DESC("iperf command") },
#endif

#if COMMAND_PING
	{ "ping",		cmd_ping_exec, CMD_DESC("ping command") },
#endif

#if COMMAND_HTTPC
	{ "httpc",		cmd_httpc_exec, CMD_DESC("httpc command") },
#endif

#if COMMAND_TLS
	{ "tls",		cmd_tls_exec, CMD_DESC("tls command") },
#endif

#if COMMAND_HTTPD
	{ "httpd",		cmd_httpd_exec, CMD_DESC("httpd command") },
#endif

#if COMMAND_SNTP
	{ "sntp",		cmd_sntp_exec, CMD_DESC("sntp command") },
#endif

#if COMMAND_NOPOLL
	{ "nopoll",		cmd_nopoll_exec, CMD_DESC("nopoll command") },
#endif

#if COMMAND_MQTT
	{ "mqtt",		cmd_mqtt_exec, CMD_DESC("mqtt command") },
#endif

#if COMMAND_DHCPD
	{ "dhcpd",		cmd_dhcpd_exec, CMD_DESC("dhcpd command") },
#endif

#if COMMAND_BRROADCAST
	{ "broadcast",  cmd_broadcast_exec, CMD_DESC("broadcast command") },
#endif

#if COMMAND_ARP
	{ "arp",        cmd_arp_exec, CMD_DESC("arp command") },
#endif

#if COMMAND_WLAN
	{ "wlan",       cmd_wlan_exec, CMD_DESC("wlan command") },
#endif
	{ "help",       cmd_net_help_exec, CMD_DESC(CMD_HELP_DESC) },
};

static enum cmd_status cmd_net_help_exec(char *cmd)
{
	return cmd_help_exec(g_net_cmds, cmd_nitems(g_net_cmds), 16);
}

static enum cmd_status cmd_net_exec(char *cmd)
{
	return cmd_exec(cmd, g_net_cmds, cmd_nitems(g_net_cmds));
}

#endif /* PRJCONF_NET_EN */

/*
 * driver commands
 */
static enum cmd_status cmd_drv_help_exec(char *cmd);

static const struct cmd_data g_drv_cmds[] = {
	{ "help",        cmd_drv_help_exec, CMD_DESC(CMD_HELP_DESC) },
};

static enum cmd_status cmd_drv_help_exec(char *cmd)
{
	return cmd_help_exec(g_drv_cmds, cmd_nitems(g_drv_cmds), 8);
}

static enum cmd_status cmd_drv_exec(char *cmd)
{
	return cmd_exec(cmd, g_drv_cmds, cmd_nitems(g_drv_cmds));
}

/*
 * main commands
 */
static enum cmd_status cmd_main_help_exec(char *cmd);

static const struct cmd_data g_main_cmds[] = {
#if PRJCONF_NET_EN
	{ "net",	cmd_net_exec, CMD_DESC("network command") },
	{ "rf",     cmd_rf_exec, CMD_DESC("radio frequency command") },
#endif
	{ "drv",	cmd_drv_exec, CMD_DESC("driver command") },
	{ "echo",	cmd_echo_exec, CMD_DESC("echo command") },
	{ "mem",	cmd_mem_exec, CMD_DESC("memory command") },
	{ "heap",	cmd_heap_exec, CMD_DESC("heap use information command") },
	{ "thread",	cmd_thread_exec, CMD_DESC("thread information command") },
	{ "upgrade",cmd_upgrade_exec, CMD_DESC("upgrade command") },
	{ "reboot", cmd_reboot_exec, CMD_DESC("reboot command") },
#ifdef __CONFIG_OTA
	{ "ota",    cmd_ota_exec, CMD_DESC("over the airtechnology upgrade commands") },
	{ "etf",	cmd_etf_exec, CMD_DESC("etf command") },
#endif
	{ "pm",		cmd_pm_exec, CMD_DESC("power management command") },
	{ "efpg",	cmd_efpg_exec, CMD_DESC("efpg command") },
	{ "flash",	cmd_flash_exec, CMD_DESC("flash control command") },
#if PRJCONF_NET_EN
	{ "lmac",	cmd_lmac_exec, CMD_DESC("low mac command") },
#endif
	{ "sysinfo",cmd_sysinfo_exec, CMD_DESC("system information command") },
    { "gpio",   cmd_gpio_exec, CMD_DESC("gpio command") },
    { "prcm",   cmd_prcm_exec, CMD_DESC("power reset and clock manager command") },
    { "help",   cmd_main_help_exec, CMD_DESC(CMD_HELP_DESC) },
};

static enum cmd_status cmd_main_help_exec(char *cmd)
{
	return cmd_help_exec(g_main_cmds, cmd_nitems(g_main_cmds), 8);
}

void main_cmd_exec(char *cmd)
{
	cmd_main_exec(cmd, g_main_cmds, cmd_nitems(g_main_cmds));
}
