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
#include <stdio.h>
#include "kernel/os/os.h"

#include "driver/chip/hal_clock.h"
#include "board_config.h"

#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_wakeup.h"

#include "pm/pm.h"

#include "common/framework/net_ctrl.h"
#include "net/wlan/wlan_ext_req.h"
#include "net/wlan/wlan_defs.h"

void wlp_pup_pa23(void)
{
	char port;
	uint32_t pin, mode, pull, level;
	GPIO_InitParam param;

	port = GPIO_PORT_A;
	pin = GPIO_PIN_23;
	mode = GPIOA_P23_F2_DCXO_PUP_OUT;
	pull = GPIO_PULL_NONE;
	level = GPIO_DRIVING_LEVEL_1;

	param.mode = mode;
	param.driving = level;
	param.pull = pull;

	HAL_GPIO_Init(port, pin, &param);
	return;
}

void wlp_set_ldo_to_low_volt(void)
{
	HAL_PRCM_SetTOPLDOVoltage(PRCM_TOPLDO_VOLT_1V4);
	return;
}

char *sta_ssid = "wlan_low_power";
char *sta_psk = "12345678";
uint8_t wlp_ap_connected = 0;
void wlp_net_ctrl_msg_proc(uint32_t event, uint32_t data, void *arg)
{
	uint16_t type = EVENT_SUBTYPE(event);

	switch (type) {
	case NET_CTRL_MSG_WLAN_CONNECTED:
		wlp_ap_connected = 1;
		break;
	case NET_CTRL_MSG_WLAN_DISCONNECTED:
		wlp_ap_connected = 0;
		break;
	case NET_CTRL_MSG_WLAN_SCAN_SUCCESS:
		break;
	case NET_CTRL_MSG_WLAN_SCAN_FAILED:
		break;
	case NET_CTRL_MSG_WLAN_4WAY_HANDSHAKE_FAILED:
		wlp_ap_connected = 0;
		break;
	case NET_CTRL_MSG_WLAN_CONNECT_FAILED:
		wlp_ap_connected = 0;
		break;
	case NET_CTRL_MSG_CONNECTION_LOSS:
		wlp_ap_connected = 0;
		break;
	case NET_CTRL_MSG_NETWORK_UP:
		break;
	case NET_CTRL_MSG_NETWORK_DOWN:
		wlp_ap_connected = 0;
		break;
	default:
		printf("unknown msg (%u, %u)\n", type, data);
		break;
	}
}

int wlp_net_ctrl_init(void)
{
	observer_base *ob = sys_callback_observer_create(CTRL_MSG_TYPE_NETWORK,
	                                                 NET_CTRL_MSG_ALL,
	                                                 wlp_net_ctrl_msg_proc,
	                                                 NULL);
	if (ob == NULL)
		return -1;
	if (sys_ctrl_attach(ob) != 0)
		return -1;

	return 0;
}

void wlp_connect_ap(void)
{
	/* create net ctrl for wlan low power examle */
	wlp_net_ctrl_init();

	/* switch to sta mode */
	net_switch_mode(WLAN_MODE_STA);

	/* set ssid and password to wlan */
	wlan_sta_set((uint8_t *)sta_ssid, strlen(sta_ssid), (uint8_t *)sta_psk);

	/* start scan and connect to ap automatically */
	wlan_sta_enable();

	return;
}

void wlp_set_wlan_to_active_mode(void)
{
	int ret;
	uint32_t ps_mode, ps_ip, ps_cp;
	wlan_ext_ps_cfg_t ps_cfg;

	ps_mode = 0;
	ps_ip = 0;
	ps_cp = 0;

	memset(&ps_cfg, 0, sizeof(wlan_ext_ps_cfg_t));
	ps_cfg.ps_mode = ps_mode;
	ps_cfg.ps_idle_period = ps_ip;
	ps_cfg.ps_change_period = ps_cp;
	ret = wlan_ext_request(g_wlan_netif,
		WLAN_EXT_CMD_SET_PS_CFG, (uint32_t)&ps_cfg);

	if (ret == -2) {
		printf("%s: invalid arg\n", __func__);
		return;
	} else if (ret == -1) {
		printf("%s: exec failed\n", __func__);
		return;
	}

	return;
}

void wlp_set_beacon_window(void)
{
	int ret;
	int bcn_win;

	bcn_win = 2300;

	ret = wlan_ext_request(g_wlan_netif,
		WLAN_EXT_CMD_SET_BCN_WIN_US, bcn_win);

	if (ret == -2) {
		printf("%s: invalid arg\n", __func__);
		return;
	} else if (ret == -1) {
		printf("%s: exec failed\n", __func__);
		return;
	}

	return;
}

void wlp_count_ap_beacon_delay(void)
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

	/* get counters for reset */
	memset(&bcn_status, 0, sizeof(wlan_ext_bcn_status_t));
	ret = wlan_ext_request(g_wlan_netif,
		WLAN_EXT_CMD_GET_BCN_STATUS, (uint32_t)&bcn_status);

	/* wait 10s for stats */
	OS_Sleep(10);

	/* get counters */
	memset(&bcn_status, 0, sizeof(wlan_ext_bcn_status_t));
	ret = wlan_ext_request(g_wlan_netif,
		WLAN_EXT_CMD_GET_BCN_STATUS, (uint32_t)&bcn_status);

	if (ret == -2) {
		printf("%s: invalid arg\n", __func__);
		return;
	} else if (ret == -1) {
		printf("%s: exec failed\n", __func__);
		return;
	}

	printf("\nAPP AP Beacon Delay Stat Info=================\n");
	for (i = 0; i < 8; i++) {
		printf("cnt %d %s: %d\n",
			i, dly_info[i],
			bcn_status.bcn_delay_cnt[i]);
		sum_cnt += bcn_status.bcn_delay_cnt[i];
	}

	if (sum_cnt)
		sum_avg = bcn_status.bcn_delay_sum / sum_cnt;

	printf("beacon duration: %d, rxed beacon: %d, missed beacon: %d\n",
		bcn_status.bcn_duration,
		bcn_status.bcn_rx_cnt,
		bcn_status.bcn_miss_cnt);
	printf("beacon delay stat: max %d us, avg %d us, sum %d us, cnt %d\n",
		bcn_status.bcn_delay_max,
		sum_avg, bcn_status.bcn_delay_sum, sum_cnt
		);
	printf("APP AP Beacon Delay Stat Info=================\n");

	return;
}

void wlp_set_dtim_period(void)
{
	int ret;
	uint32_t period;

	period = 1;

	ret = wlan_ext_request(g_wlan_netif,
		WLAN_EXT_CMD_SET_PM_DTIM, period);

	if (ret == -2) {
		printf("%s: invalid arg\n", __func__);
		return;
	} else if (ret == -1) {
		printf("%s: exec failed\n", __func__);
		return;
	}

	return;
}

void wlp_set_listen_interval(void)
{
	int ret;
	uint32_t listen_interval;

	listen_interval = 8;

	ret = wlan_ext_request(g_wlan_netif,
		WLAN_EXT_CMD_SET_LISTEN_INTERVAL, listen_interval);

	if (ret == -2) {
		printf("%s: invalid arg\n", __func__);
		return;
	} else if (ret == -1) {
		printf("%s: exec failed\n", __func__);
		return;
	}

	return;
}

void wlp_set_tx_null_period(void)
{
	int ret;
	int period;

	period = 48;

	ret = wlan_ext_request(g_wlan_netif,
		WLAN_EXT_CMD_SET_PM_TX_NULL_PERIOD, period);

	if (ret == -2) {
		printf("%s: invalid arg\n", __func__);
		return;
	} else if (ret == -1) {
		printf("%s: exec failed\n", __func__);
		return;
	}

	return;
}

void wlp_set_ps_mode_with_little_change_time(void)
{
	int ret;
	uint32_t ps_mode, ps_ip, ps_cp;
	wlan_ext_ps_cfg_t ps_cfg;

	ps_mode = 1;
	ps_ip = 10;
	ps_cp = 10;

	memset(&ps_cfg, 0, sizeof(wlan_ext_ps_cfg_t));
	ps_cfg.ps_mode = ps_mode;
	ps_cfg.ps_idle_period = ps_ip;
	ps_cfg.ps_change_period = ps_cp;
	ret = wlan_ext_request(g_wlan_netif,
		WLAN_EXT_CMD_SET_PS_CFG, (uint32_t)&ps_cfg);

	if (ret == -2) {
		printf("%s: invalid arg\n", __func__);
		return;
	} else if (ret == -1) {
		printf("%s: exec failed\n", __func__);
		return;
	}

	return;
}

void wlp_goto_standby(void)
{
	HAL_Wakeup_SetTimer_Sec(120);

	pm_enter_mode(PM_MODE_STANDBY);

	return;
}

int main(void)
{
	platform_init();

	/* wlan low power example info */
	printf("wlan low power example base on XR808ST demo board.\n\n");
	printf("\trecommend to read readme.md and\n"
			"\tXRADIO_Wlan_Low_Power_Developer_Guide-CN.doc for more information.\n");
	printf("\trecommend to use DC current Analyzer(e.g. N6705B) which connecting V_BAT\n"
		"\tin XR808ST demo borad to measure power.\n\n");

	OS_Sleep(2);
	/* use low cpu frequency */
	printf("use low cpu freq:\n\n");

	printf("\tcpu freq is %d Hz, recommended value is 160MHz or lower.\n",
		HAL_GetCPUClock());

	printf("\tmodify BOARD_CPU_CLK_FACTOR in board_config.h to set cpu freq.\n");

	printf("\n");

	OS_Sleep(2);
	/* use external low frequency XTAL */
	printf("use accurate external low frequency XTAL:\n\n");

	printf("\tBOARD_LOSC_EXTERNAL is %d, recommended value is 1\n",
		BOARD_LOSC_EXTERNAL);

	printf("\tmodify BOARD_LOSC_EXTERNAL in board_config.h to use ext LF XTAL.\n");

	printf("\n");

	OS_Sleep(2);
	/* use DC-DC mode */
	printf("use DC-DC mode:\n\n");

	printf("\tpull up PA23 pin to enable SY8088, the output VDD_ANA will be 1.8v \n");

	wlp_pup_pa23();

	printf("\tif the output of LDO module in XR808ST is lower than VDD_ANA,\n"
			"\tXR808ST will use VDD_ANA.\n");

	printf("\tset the output of LDO module in XR808ST to be 1.4v\n");

	wlp_set_ldo_to_low_volt();

	printf("\n");

	OS_Sleep(2);
	/* connect AP */
	printf("connect ap:\n\n");
	wlp_connect_ap();

	/* wait to be connected */
	while (wlp_ap_connected == 0) {
		printf("\n\tthis example need ap(2.4GHz band), please set your ap in wpa2:\n"
			"\t\tssid: %s, password: %s\n", sta_ssid, sta_psk);
		printf("\tor use this cmd to connect your ap with your own ssid and password:\n"
			"\t\t\"net sta config your_ssid your_password\"\n\n");
		OS_Sleep(10);
	}

	OS_Sleep(2);
	/* count ap beacon delay */
	printf("\ncount ap beacon delay:\n\n");

	printf("\tfirst, set wlan to be active mode.\n");
	wlp_set_wlan_to_active_mode();

	printf("\tsecond, get ap beacon delay stats in 10s or more as you wanted.\n");
	wlp_count_ap_beacon_delay();

	OS_Sleep(2);
	/* set beacon window */
	printf("\nset beacon window:\n\n");

	printf("\treduce this value will save wait time when missing beacon,\n"
			"\tbut will increase the probability of beacon loss\n\n");

	printf("\tthis value is based on ap beacon delay, this example use 2300us.\n");
	wlp_set_beacon_window();

	OS_Sleep(2);
	/* set dtim period and listen interval */
	printf("\nset dtim period and listen interval:\n\n");
	printf("\tincrease dtim period or listen interval will reduce wlan wakeup time\n");
	printf("\tthe unit of increase dtim period and listen interval is beacon period.\n");
	printf("\tdtim period determine the min value of wlan wakeup period when mcu in standby mode\n");
	printf("\tlisten interval determine the max value of wlan wakeup period when mcu in standby mode\n");
	printf("\tif listen interval is 0, wlan wakeup period will equal to dtim period\n\n");

	printf("\tset dtim period to 1 and set listen interval to 8 in this example.\n");
	wlp_set_dtim_period();
	wlp_set_listen_interval();

	OS_Sleep(2);
	/* set tx null period */
	printf("\nset tx null period:\n\n");
	printf("\tincrease tx null period will reduce wlan tx time\n");
	printf("\tthis value is based on ap endurance.\n");
	printf("\tif tx null period is too long, ap will kick off this sta.\n\n");

	printf("\tset tx null period to 48s in this example.\n");
	wlp_set_tx_null_period();

	OS_Sleep(2);
	/* set ps mode with little change time */
	printf("\nset ps mode with little change time:\n\n");
	printf("\treduce ps idle period and change period will reduce ps mode change time.\n\n");
	printf("\tset ps idle period and change period to 10ms in this example\n");
	wlp_set_ps_mode_with_little_change_time();

	OS_Sleep(2);
	/* set mcu into standby mode */
	printf("\nset mcu into standby mode:\n\n");
	printf("\tin this mode, use DC current Analyzer to measure the power.\n\n");

	printf("\tset wakeup timer and goto standby, will wakeup after 120s.\n\n");
	printf("\tplease measure power now.\n\n\n");
	wlp_goto_standby();

	OS_Sleep(2);
	/* wakeup now */
	printf("\nwakeup now.\n\n");

	OS_Sleep(2);
	/* pm cmd info */
	printf("\npm cmd info:\n\n");
	printf("\tuse cmd \"pm wk_timer 120\" to set wakeup timer\n");
	printf("\tuse cmd \"pm standby\" to goto standby mode again.\n\n");

	return 0;
}
