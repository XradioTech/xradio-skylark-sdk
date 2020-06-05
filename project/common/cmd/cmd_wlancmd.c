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

#include "cmd_util.h"
#include "common/framework/net_ctrl.h"
#include "net/wlan/wlan_ext_req.h"

#if PRJCONF_NET_EN
static enum cmd_status cmd_wlan_set_pm_dtim(char *cmd)
{
	int ret, cnt;
	uint32_t period;

	cnt = cmd_sscanf(cmd, "p=%d", &period);
	if (cnt != 1) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_PM_DTIM, period);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_set_ps_mode(char *cmd)
{
	int ret, cnt;
	uint32_t ps_mode, ps_ip, ps_cp;
	wlan_ext_ps_cfg_t ps_cfg;

	if (cmd_strncmp(cmd, "enable", 6) == 0) {
		ps_mode = 1;
		cnt = cmd_sscanf((cmd + 6), " ip=%d cp=%d", &ps_ip, &ps_cp);
		if (cnt != 2) {
			ps_ip = 0;
			ps_cp = 0;
			CMD_ERR("cnt %d\n", cnt);
		}
	} else if (cmd_strncmp(cmd, "disable", 7) == 0) {
		ps_mode = 0;
	} else {
		CMD_ERR("invalid argument '%s'\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	cmd_memset(&ps_cfg, 0, sizeof(wlan_ext_ps_cfg_t));
	ps_cfg.ps_mode = ps_mode;
	ps_cfg.ps_idle_period = ps_ip;
	ps_cfg.ps_change_period = ps_cp;
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_PS_CFG, (uint32_t)&ps_cfg);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_set_ampdu(char *cmd)
{
	int ret, cnt;
	int num;

	cnt = cmd_sscanf(cmd, "l=%d", &num);
	if (cnt != 1) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_AMPDU_TXNUM, num);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_set_retry(char *cmd)
{
	int ret;
	int retry_cnt, cnt;

	cnt = cmd_sscanf(cmd, "n=%d", &retry_cnt);
	if (cnt != 1) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_TX_RETRY_CNT, retry_cnt);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_set_pm_tx_null_period(char *cmd)
{
	int ret;
	int period, cnt;

	cnt = cmd_sscanf(cmd, "p=%d", &period);
	if (cnt != 1) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_PM_TX_NULL_PERIOD, period);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_get_bcn_stat(char *cmd)
{
	int ret, i;
	unsigned int sum_cnt = 0;
	int sum_avg = 0;
	wlan_ext_bcn_status_t bcn_status;

	char dly_info[][20] = {
		"(<0      )",
		"(<500us  )",
		"(<1000us )",
		"(<2000us )",
		"(<4000us )",
		"(<8000us )",
		"(<16000us)",
		"(>16000us)",
	};

	cmd_memset(&bcn_status, 0, sizeof(wlan_ext_bcn_status_t));
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_BCN_STATUS, (uint32_t)&bcn_status);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	CMD_LOG(1, "\nAPP AP Beacon Delay Stat Info=================\n");
	for (i = 0; i < 8; i++) {
		CMD_LOG(1, "cnt %d %s: %d\n",
			i, dly_info[i],
			bcn_status.bcn_delay_cnt[i]);
		sum_cnt += bcn_status.bcn_delay_cnt[i];
	}

	if (sum_cnt)
		sum_avg = bcn_status.bcn_delay_sum / sum_cnt;

	CMD_LOG(1, "Dur: %d, Max: 0x%X, Rxed: %d, Missed: 0x%X\n",
		bcn_status.bcn_duration,
		bcn_status.bcn_delay_max,
		bcn_status.bcn_rx_cnt,
		bcn_status.bcn_miss_cnt);
	CMD_LOG(1, "Sum: %d, Cnt: %d, Ava: %d\n",
		bcn_status.bcn_delay_sum,
		sum_cnt,
		sum_avg);
	CMD_LOG(1, "APP AP Beacon Delay Stat Info=================\n");

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_set_bcn_win_us(char *cmd)
{
	int ret;
	int bcn_win, cnt;

	cnt = cmd_sscanf(cmd, "w=%d", &bcn_win);
	if (cnt != 1) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_BCN_WIN_US, bcn_win);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_get_cur_tx_rate(char *cmd)
{
	int ret;
	int rate;

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_TX_RATE, (int)(&rate));
	CMD_LOG(1, "current rate:%d\n", rate);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_get_pm_dtim(char *cmd)
{
	int ret;
	wlan_ext_pm_dtim_t dtim;

	cmd_memset(&dtim, 0, sizeof(wlan_ext_pm_dtim_t));
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_PM_DTIM, (int)(&dtim));
	CMD_LOG(1, "AP DTIM set:%d, STA DTIM set:%d\n",
		dtim.pm_join_dtim_period, dtim.pm_dtim_period_extend);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_get_cur_signal(char *cmd)
{
	int ret;
	wlan_ext_signal_t signal;

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_SIGNAL, (int)(&signal));

	CMD_LOG(1, "current rssi:%d, noise:%d\n", signal.rssi, signal.noise);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_set_scan_param(char *cmd)
{
	int ret, cnt;
	int num_probes;
	int probe_delay;
	int min_dwell;
	int max_dwell;
	wlan_ext_scan_param_t param;

	cnt = cmd_sscanf(cmd, "n=%d d=%d min=%d max=%d",
		&num_probes, &probe_delay, &min_dwell, &max_dwell);
	if (cnt != 4) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	memset(&param, 0, sizeof(wlan_ext_scan_param_t));
	param.num_probes = num_probes;
	param.probe_delay = probe_delay;
	param.min_dwell = min_dwell;
	param.max_dwell = max_dwell;
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_SCAN_PARAM, (int)(&param));

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_set_listen_interval(char *cmd)
{
	int ret, cnt;
	uint32_t listen_interval;

	cnt = cmd_sscanf(cmd, "l=%d", &listen_interval);
	if (cnt != 1) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_LISTEN_INTERVAL, listen_interval);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_set_bcn_win_cfg(char *cmd)
{
	int ret, cnt;
	int start_num, stop_num, amp_us, max_num;
	wlan_ext_bcn_win_param_set_t param;

	cnt = cmd_sscanf(cmd, "start=%d stop=%d amp=%d max=%d",
		&start_num, &stop_num, &amp_us, &max_num);
	if (cnt != 4) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	param.BeaconWindowAdjStartNum = start_num;
	param.BeaconWindowAdjStopNum = stop_num;
	param.BeaconWindowAdjAmpUs = amp_us;
	param.BeaconWindowMaxStartNum = max_num;
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_BCN_WIN_CFG, (int)(&param));

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_scan_freq(char *cmd)
{
	int ret, cnt;
	int num;
	int chn[14];
	int freq[14];
	wlan_ext_scan_freq_t param;

	cnt = cmd_sscanf(cmd, "n=%d c=%d %d %d %d %d %d %d %d %d %d %d %d %d %d", &num,
					&chn[0], &chn[1], &chn[2], &chn[3], &chn[4], &chn[5], &chn[6],
					&chn[7], &chn[8], &chn[9], &chn[10], &chn[11], &chn[12], &chn[13]);
	if (cnt != 15) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (num > 14) {
		CMD_ERR("%s: invalid num:%d\n", __func__, num);
		return CMD_STATUS_ACKED;
	}
	for (int i = 0;i < num;i++) {
		freq[i] = 2407 + 5 * chn[i];
	}
	param.freq_num = num;
	param.freq_list = freq;
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_SET_SCAN_FREQ, (int)(&param));

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_pm(char *cmd)
{
	int ret, cnt;
	int enable;

	cnt = cmd_sscanf(cmd, "e=%d", &enable);
	if (cnt != 1) {
		CMD_ERR("cnt %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}
	ret = wlan_set_ps_mode(g_wlan_netif, enable);

	if (ret == -2) {
		CMD_ERR("%s: command '%s' invalid arg\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	} else if (ret == -1) {
		CMD_ERR("%s: command '%s' exec failed\n", __func__, cmd);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_wlan_get_temp_volt(char *cmd)
{
	int ret;
	wlan_ext_temp_volt_get_t param;

	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_TEMP_VOLT, (int)(&param));
	CMD_LOG(1, "Temperature:%.02f°C\n", (float)param.Temperature / 16);
	CMD_LOG(1, "Voltage:%.02fV\n", (float)param.Voltage / 16);

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
 * wlan commands
 */

static enum cmd_status cmd_wlan_help_exec(char *cmd);

static const struct cmd_data g_wlan_cmds[] = {
	{ "pm_dtim",			cmd_wlan_set_pm_dtim },
	{ "get_pm_dtim",		cmd_wlan_get_pm_dtim },
	{ "set_lsn_intv",		cmd_wlan_set_listen_interval },
	{ "ps_mode",			cmd_wlan_set_ps_mode },
	{ "pm",					cmd_wlan_pm },
	{ "ampdu",				cmd_wlan_set_ampdu },
	{ "retry",				cmd_wlan_set_retry },
	{ "null_prd",			cmd_wlan_set_pm_tx_null_period },
	{ "bcn_win_cfg",		cmd_wlan_set_bcn_win_cfg },
	{ "bcn_win",			cmd_wlan_set_bcn_win_us },
	{ "get_bcn_stat",		cmd_wlan_get_bcn_stat },
	{ "get_cur_rate",		cmd_wlan_get_cur_tx_rate },
	{ "get_cur_signal",		cmd_wlan_get_cur_signal },
	{ "scan_freq",			cmd_wlan_scan_freq },
	{ "set_scan_param",		cmd_wlan_set_scan_param },
	{ "get_temp_volt",		cmd_wlan_get_temp_volt },
	{ "help",				cmd_wlan_help_exec },
};

static enum cmd_status cmd_wlan_help_exec(char *cmd)
{
	return cmd_help_exec(g_wlan_cmds, cmd_nitems(g_wlan_cmds), 16);
}

enum cmd_status cmd_wlan_exec(char *cmd)
{
	if (g_wlan_netif == NULL) {
		return CMD_STATUS_FAIL;
	}
	return cmd_exec(cmd, g_wlan_cmds, cmd_nitems(g_wlan_cmds));
}
#endif
