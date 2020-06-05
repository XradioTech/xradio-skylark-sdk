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

#if PRJCONF_NET_EN
#define COMMAND_IPERF	1
#define COMMAND_PING	1

/*
 * net commands
 */
static enum cmd_status cmd_net_help_exec(char *cmd);

static const struct cmd_data g_net_cmds[] = {
	{ "sta",		cmd_wlan_sta_exec, CMD_DESC("sta command") },
	{ "ifconfig",	cmd_ifconfig_exec, CMD_DESC("ifconfig command") },
	{ "smartconfig",cmd_smart_config_exec, CMD_DESC("smartconfig command") },
	{ "airkiss",	cmd_airkiss_exec, CMD_DESC("airkiss command") },
	{ "voiceprint",	cmd_voice_print_exec, CMD_DESC("voice print command")  },
	{ "smartlink",	cmd_smartlink_exec, CMD_DESC("smartlink command") },
#if COMMAND_IPERF
	{ "iperf",		cmd_iperf_exec, CMD_DESC("iperf command") },
#endif
#if COMMAND_PING
	{ "ping",		cmd_ping_exec, CMD_DESC("ping command") },
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
#endif

/*
 * driver commands
 */
static enum cmd_status cmd_drv_help_exec(char *cmd);

static const struct cmd_data g_drv_cmds[] = {
	{ "sd",		cmd_sd_exec, CMD_DESC("SD card") },
#ifdef __CONFIG_PSRAM
	{ "psram",	cmd_psram_exec, CMD_DESC("psram command") },
#endif
	{ "help",   cmd_drv_help_exec, CMD_DESC(CMD_HELP_DESC) },
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
#endif
	{ "drv",	cmd_drv_exec, CMD_DESC("driver command") },
	{ "echo",	cmd_echo_exec, CMD_DESC("echo command") },
	{ "mem",	cmd_mem_exec, CMD_DESC("memory command") },
	{ "upgrade",cmd_upgrade_exec, CMD_DESC("upgrade command") },
	{ "reboot", cmd_reboot_exec, CMD_DESC("reboot command") },
	{ "pm",		cmd_pm_exec, CMD_DESC("power management command") },
	{ "heap",	cmd_heap_exec, CMD_DESC("heap use information command") },
	{ "cedarx", cmd_cedarx_exec, CMD_DESC("cedarx player command") },
	{ "fs",     cmd_fs_exec, CMD_DESC("file systems command") },
	{ "audio",  cmd_audio_exec, CMD_DESC("audio command") },
	{ "auddbg", cmd_auddbg_exec, CMD_DESC("audio debug command") },
	{ "sysinfo",cmd_sysinfo_exec, CMD_DESC("system information command") },
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
