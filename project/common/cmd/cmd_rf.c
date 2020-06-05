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
#include "net/wlan/wlan_ext_req.h"
#include "net/wlan/wlan_defs.h"
#include "driver/chip/hal_prcm.h"

char * rate_tab_name[11] = {
	"DSSS         1,2",
	"CCK       5.5,11",
	"BPSK1/2    6,6.5",
	"BPSK3/4        9",
	"QPSK1/2    12,13",
	"QPSK3/4  18,19.5",
	"16QAM1/2   24,26",
	"16QAM3/4   36,39",
	"64QAM2/3   48,52",
	"64QAM3/4 54,58.5",
	"64QAM5/6      65"
};

enum cmd_status cmd_rf_set_freq_offset_exec(char *cmd)
{
	int cnt;
	uint32_t value;
	/* get param */
	cnt = cmd_sscanf(cmd, "%d", &value);

	/* check param */
	if (cnt != 1) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}
	if (value > 127) {
		CMD_ERR("invalid value %d\n", value);
		return CMD_STATUS_INVALID_ARG;
	}

	HAL_MODIFY_REG(PRCM->DCXO_CTRL, PRCM_FREQ_OFFSET_MASK, value << PRCM_FREQ_OFFSET_SHIFT);
	CMD_LOG(1, "freq offset is set to %d!\n", value);

	return CMD_STATUS_OK;
}

enum cmd_status cmd_rf_get_freq_offset_exec(char *cmd)
{
	CMD_LOG(1, "freq offset is :%d\n", (PRCM->DCXO_CTRL & PRCM_FREQ_OFFSET_MASK) >> PRCM_FREQ_OFFSET_SHIFT);
	return CMD_STATUS_OK;
}

enum cmd_status cmd_rf_set_sdd_freq_offset_exec(char *cmd)
{
	int ret, cnt;
	uint32_t value;
	/* get param */
	cnt = cmd_sscanf(cmd, "%d", &value);

	/* check param */
	if (cnt != 1) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}
	if (value > 127) {
		CMD_ERR("invalid value %d\n", value);
		return CMD_STATUS_INVALID_ARG;
	}
	uint16_t value16 = (uint16_t)value;
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_SDD_FREQ_OFFSET, (int)(&value16));
	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	CMD_LOG(1, "sdd freq offset is set to %d!\n", value);

	return CMD_STATUS_OK;
}

enum cmd_status cmd_rf_get_sdd_freq_offset_exec(char *cmd)
{
	int ret;
	uint16_t value;
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_SDD_FREQ_OFFSET, (int)(&value));
	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}
	CMD_LOG(1, "sdd freq offset is :%d\n", value);
	return CMD_STATUS_OK;
}

/*
 * print_type:0-HEX type, 1-ASCII type
 */
enum cmd_status cmd_rf_get_sdd_file_exec(char *cmd)
{
	int ret;
	int print_type;
	if (!strcmp("hex",cmd)) {
		print_type = 0;
	} else if (!strcmp("ascii",cmd)) {
		print_type = 1;
	} else {
		CMD_LOG(1, "Invalid params input!Should be [hex | ascii] !\n");
		return -1;
	}

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_SDD_FILE, (int)(&print_type));
	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}
	return CMD_STATUS_OK;
}

enum cmd_status cmd_rf_get_power_exec(char *cmd)
{
	int ret, i;
	int begin_rate, end_rate;
	wlan_ext_power_level_tab_get_t param;
	if (!strcmp("11b",cmd)) {
		begin_rate = 0;
		end_rate = 1;
	} else if (!strcmp("11gn",cmd)) {
		begin_rate = 2;
		end_rate = 9;
	} else if (!strcmp("11n_mcs7",cmd)) {
		begin_rate = 10;
		end_rate = 10;
	} else if (!strcmp("all",cmd)) {
		begin_rate = 0;
		end_rate = 10;
	} else {
		CMD_LOG(1, "Invalid params input!Should be [11b | 11gn | 11n_mcs7 | all] !\n");
		return CMD_STATUS_INVALID_ARG;
	}

	CMD_LOG(1, "------------ power level tab current ------------\n");
	param.PowerTabType = POWER_LEVEL_TAB_TYPE_CUR;
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_POWER_LEVEL_TAB, (int)(&param));
	for (i = begin_rate; i <= end_rate; i++) {
		CMD_LOG(1, "%s: %d\n", rate_tab_name[i], param.PowerTab[i]*4/10+2);
	}
	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}
	return CMD_STATUS_ACKED;
}

enum cmd_status cmd_rf_set_power_exec(char *cmd)
{
	int ret, cnt, i;
	int begin_rate, end_rate;
	int power_b = 0, power_gn = 0, power_mcs7 = 0;
	wlan_ext_power_level_tab_get_t param_get;
	wlan_ext_power_level_tab_set_t param_set;
	if (!strncmp("11b ",cmd,4)) {
		begin_rate = 0;
		end_rate = 1;
		cnt = cmd_sscanf(cmd + 4, "%d", &power_b);
		if (cnt != 1) {
			CMD_ERR("cnt %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
	} else if (!strncmp("11gn ",cmd,5)) {
		begin_rate = 2;
		end_rate = 9;
		cnt = cmd_sscanf(cmd + 5, "%d", &power_gn);
		if (cnt != 1) {
			CMD_ERR("cnt %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
	} else if (!strncmp("11n_mcs7 ",cmd,9)) {
		begin_rate = 10;
		end_rate = 10;
		cnt = cmd_sscanf(cmd + 9, "%d", &power_mcs7);
		if (cnt != 1) {
			CMD_ERR("cnt %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
	} else if (!strncmp("all ",cmd, 4)) {
		begin_rate = 0;
		end_rate = 10;
		cnt = cmd_sscanf(cmd + 4, "%d %d %d",
			&power_b, &power_gn, &power_mcs7);
		if (cnt != 3) {
			CMD_ERR("cnt %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
	} else {
		CMD_LOG(1, "Invalid params input!Should be [11b | 11gn | 11n_mcs7 | all] !\n");
		return -1;
	}

	param_get.PowerTabType = POWER_LEVEL_TAB_TYPE_CUR;
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_POWER_LEVEL_TAB, (int)(&param_get));
	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	for (i = begin_rate; i <= end_rate; i++) {
		if (i >= 0 && i < 2)
			param_get.PowerTab[i] = (power_b - 2) * 10 / 4;
		else if (i >= 2 && i < 10)
			param_get.PowerTab[i] = (power_gn - 2) * 10 / 4;
		else
			param_get.PowerTab[i] = (power_mcs7 - 2) * 10 / 4;
	}
	cmd_memcpy(param_set.PowerTab, param_get.PowerTab, POWER_LEVEL_TAB_USE_LENGTH * sizeof(uint16_t));
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_POWER_LEVEL_TAB, (int)(&param_set));
	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}
	CMD_LOG(1, "Set user power level:\n");
	for (i = 0; i < 11; i++) {
		CMD_LOG(1, "%s\t%d\n",rate_tab_name[i], (int)(param_set.PowerTab[i]*4/10+2));
	}

	return CMD_STATUS_ACKED;
}

enum cmd_status cmd_rf_get_sdd_power_exec(char *cmd)
{
	int ret;
	int begin_rate, end_rate;
	uint16_t user_power[13];
	if (!strcmp("11b",cmd)) {
		begin_rate = 0;
		end_rate = 1;
	} else if (!strcmp("11gn",cmd)) {
		begin_rate = 2;
		end_rate = 9;
	} else if (!strcmp("11n_mcs7",cmd)) {
		begin_rate = 10;
		end_rate = 10;
	} else if (!strcmp("all",cmd)) {
		begin_rate = 0;
		end_rate = 10;
	} else {
		CMD_LOG(1, "Invalid params input!Should be [11b | 11gn | 11n_mcs7 | all] !\n");
		return -1;
	}

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_SDD_POWER, (int)(user_power));
	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}
	CMD_LOG(1, "Get sdd user power level:\n");
	for (int i = begin_rate; i <= end_rate; i++) {
		CMD_LOG(1, "%s\t%d\n",rate_tab_name[i], (int)(user_power[i]*4/16+2));
	}
	return CMD_STATUS_OK;
}

enum cmd_status cmd_rf_set_sdd_power_exec(char *cmd)
{
	int ret, cnt, i;
	int begin_rate, end_rate;
	uint16_t user_power[13];
	int power_b = 0, power_gn = 0, power_mcs7 = 0;
	if (!strncmp("11b ",cmd,4)) {
		begin_rate = 0;
		end_rate = 1;
		cnt = cmd_sscanf(cmd + 4, "%d", &power_b);
		if (cnt != 1) {
			CMD_ERR("cnt %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
	} else if (!strncmp("11gn ",cmd,5)) {
		begin_rate = 2;
		end_rate = 9;
		cnt = cmd_sscanf(cmd + 5, "%d", &power_gn);
		if (cnt != 1) {
			CMD_ERR("cnt %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
	} else if (!strncmp("11n_mcs7 ",cmd,9)) {
		begin_rate = 10;
		end_rate = 10;
		cnt = cmd_sscanf(cmd + 9, "%d", &power_mcs7);
		if (cnt != 1) {
			CMD_ERR("cnt %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
	} else if (!strncmp("all ",cmd, 4)) {
		begin_rate = 0;
		end_rate = 10;
		cnt = cmd_sscanf(cmd + 4, "%d %d %d",
			&power_b, &power_gn, &power_mcs7);
		if (cnt != 3) {
			CMD_ERR("cnt %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
	} else {
		CMD_LOG(1, "Invalid params input!Should be [11b | 11gn | 11n_mcs7 | all] !\n");
		return -1;
	}

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_SDD_POWER, (int)(user_power));
	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	for (i = begin_rate; i <= end_rate; i++) {
		if (i >= 0 && i < 2)
			user_power[i] = (power_b - 2) * 16 / 4;
		else if (i >= 2 && i < 10)
			user_power[i] = (power_gn - 2) * 16 / 4;
		else
			user_power[i] = (power_mcs7 - 2) * 16 / 4;
	}

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_SDD_POWER, (int)(user_power));
	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}
	CMD_LOG(1, "Set sdd user power level:\n");
	for (i = 0; i < 11; i++) {
		CMD_LOG(1, "%s\t%d\n",rate_tab_name[i], (int)(user_power[i]*4/16+2));
	}
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_rf_set_channel_fec(char *cmd)
{
	int ret, cnt;
	int ch1, ch7, ch13;
	wlan_ext_channel_fec_set_t param;

	cnt = cmd_sscanf(cmd, "%d %d %d", &ch1, &ch7, &ch13);
	if (cnt != 3) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	param.FecChannel1 = ch1 * 2;
	param.FecChannel7 = ch7 * 2;
	param.FecChannel13 = ch13 * 2;
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_CHANNEL_FEC, (int)(&param));

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

/*
 * rf commands
 */
static enum cmd_status cmd_rf_help_exec(char *cmd);

static const struct cmd_data g_rf_cmds[] = {
	{ "set_freq_offset",		cmd_rf_set_freq_offset_exec, CMD_DESC("set the frequency offset") },
	{ "get_freq_offset",		cmd_rf_get_freq_offset_exec, CMD_DESC("get the frequency offset") },
	{ "set_sdd_freq_offset",	cmd_rf_set_sdd_freq_offset_exec, CMD_DESC("set the frequency offset in sdd") },
	{ "get_sdd_freq_offset",	cmd_rf_get_sdd_freq_offset_exec, CMD_DESC("get the frequency offset in sdd") },
	{ "set_sdd_power",			cmd_rf_set_sdd_power_exec, CMD_DESC("set the transmit power in sdd") },
	{ "get_sdd_power",			cmd_rf_get_sdd_power_exec, CMD_DESC("get the transmit power in sdd") },
	{ "set_power",				cmd_rf_set_power_exec, CMD_DESC("set the transmit power") },
	{ "get_power",				cmd_rf_get_power_exec, CMD_DESC("get the transmit power") },
	{ "get_sdd_file",			cmd_rf_get_sdd_file_exec, CMD_DESC("get the sdd file") },
	{ "set_channel_fec",		cmd_rf_set_channel_fec, CMD_DESC("set the channel fec") },
	{ "help",					cmd_rf_help_exec, CMD_DESC(CMD_HELP_DESC) },
};

static enum cmd_status cmd_rf_help_exec(char *cmd)
{
	return cmd_help_exec(g_rf_cmds, cmd_nitems(g_rf_cmds), 24);
}

enum cmd_status cmd_rf_exec(char *cmd)
{
	return cmd_exec(cmd, g_rf_cmds, cmd_nitems(g_rf_cmds));
}
#endif

