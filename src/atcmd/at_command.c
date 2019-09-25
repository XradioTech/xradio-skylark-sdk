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

#include "atcmd/at_command.h"
#include "at_private.h"
#include "at_debug.h"
#include "kernel/os/os.h"
#include "net/wlan/wlan_defs.h"
#include "net/wlan/wlan.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"

#include "../../project/common/framework/net_ctrl.h"
#define PRJCONF_NET_EN                  1
#define PRJCONF_SYSINFO_SAVE_TO_FLASH	1

#include "../../project/common/framework/sysinfo.h"

#define CMD_CACHE_MAX_LEN 1024
#define CMD_SEND_DATA_MAX_LEN 1024
#define CMD_SEND_TIMEOUT	10
char *c_atCmdRspBuf[1024];

typedef struct cmd_cache {
	u32 cnt;
	u8 buf[CMD_CACHE_MAX_LEN];
} cmd_cache_t;

typedef struct cmd_send_cache {
	u32 status;
	u32 cnt;
	u32 linkID;
	u32 length;
	u8 buf[CMD_SEND_DATA_MAX_LEN];
}cmd_send_cache_t;

typedef struct {
	char *ptr;
} at_para_t;

typedef struct {
	const char *cmd;
	AT_ERROR_CODE (*handler)(at_para_t *at_para);
	const char *usage;
} at_command_handler_t;

typedef struct {
	at_text_t key[AT_PARA_MAX_SIZE];
} at_getcfg_t;

typedef struct {
	at_text_t key[AT_PARA_MAX_SIZE];
} config_key_t;

typedef struct {
	at_text_t ssid[AT_PARA_MAX_SIZE];
} at_ssidtxt_para_t;

typedef struct {
	at_text_t sts_var[AT_PARA_MAX_SIZE];
} at_status_para_t;

typedef struct {
	at_di_t peer_number;
	at_text_t peer_var[AT_PARA_MAX_SIZE];
} at_peers_para_t;

typedef struct {
	at_text_t hostname[AT_PARA_MAX_SIZE];
} at_ping_para_t;

typedef struct {
	at_text_t hostname[AT_PARA_MAX_SIZE];
	at_di_t port;
	at_text_t protocol[1+1];
	at_text_t ind[3+1];
} at_sockon_para_t;

typedef struct {
	at_text_t id[AT_PARA_MAX_SIZE];
	at_di_t len;
} at_sockw_para_t;

typedef struct {
	at_text_t id[AT_PARA_MAX_SIZE];
} at_sockq_para_t;

typedef struct {
	at_text_t id[AT_PARA_MAX_SIZE];
	at_di_t len;
} at_sockr_para_t;

typedef struct {
	at_text_t id[AT_PARA_MAX_SIZE];
} at_sockc_para_t;

typedef struct {
	at_di_t port;
	at_text_t protocol[1+1];
} at_sockd_para_t;

typedef struct {
	at_di_t value;
} at_wifi_para_t;

typedef struct {
	at_di_t num;
	at_text_t direction[3+1];
	at_text_t interrupt[1+1];
} at_gpioc_para_t;

typedef struct {
	at_di_t num;
} at_gpior_para_t;

typedef struct {
	at_di_t num;
	at_di_t value;
} at_gpiow_para_t;

typedef struct {
	at_text_t hostname[AT_PARA_MAX_SIZE];
	at_text_t path[1+1];
	at_di_t port;
} at_upgrade_para_t;

typedef struct {
    uint8_t ssid[WLAN_SSID_MAX_LEN];
} at_lapscan_para_t;

typedef struct {
	at_text_t mode[1+1];
	at_text_t repeat[1+1];
} at_scan_para_t;

extern s32 at_get_value(char *strbuf, s32 pt, void *pvar, s32 vsize);
extern s32 at_set_value(s32 pt, void *pvar, s32 vsize, at_value_t *value);
extern int getWifiStationStatus(void);
extern int is_netconnet_ap(void);

extern AT_ERROR_CODE at_get_parameters(char **ppara, at_para_descriptor_t *list, s32 lsize, s32 *pcnt);
extern void at_response(AT_ERROR_CODE aec);

extern AT_ERROR_CODE at_act(void);
extern AT_ERROR_CODE at_reset(void);
extern AT_ERROR_CODE at_help(void);
extern AT_ERROR_CODE at_getcfg(char *key);
extern AT_ERROR_CODE at_typecfg(char *key);
extern AT_ERROR_CODE at_setcfg(char *key,at_value_t *value);
extern AT_ERROR_CODE at_ssidtxt(char *ssid);
extern AT_ERROR_CODE at_config(void);
extern AT_ERROR_CODE at_factory(void);
extern AT_ERROR_CODE at_save(void);
extern AT_ERROR_CODE at_status(char *sts_var);
extern AT_ERROR_CODE at_peers(s32 pn,char *pv);
extern AT_ERROR_CODE at_ping(char *hostname);
extern AT_ERROR_CODE at_sockon(char *hostname, s32 port, char *protocol, char *ind);
extern AT_ERROR_CODE at_sockw(char *id, s32 len);
extern AT_ERROR_CODE at_sockq(char *id);
extern AT_ERROR_CODE at_sockr(char *id, s32 len);
extern AT_ERROR_CODE at_sockc(char *id);
extern AT_ERROR_CODE at_sockd(s32 port, char *protocol);
extern AT_ERROR_CODE at_mode(AT_MODE mode);
extern AT_ERROR_CODE at_wifi(s32 value);
extern AT_ERROR_CODE at_reassociate(void);
extern AT_ERROR_CODE at_gpioc(s32 num, char *direction, char *interrupt);
extern AT_ERROR_CODE at_gpior(s32 num);
extern AT_ERROR_CODE at_gpiow(s32 num, s32 value);
extern AT_ERROR_CODE at_upgrade(char *hostname, char *path, s32 port);
extern AT_ERROR_CODE at_scan(char *mode, char *repeat);

static AT_ERROR_CODE attention_handler(at_para_t *at_para);
static AT_ERROR_CODE act_handler(at_para_t *at_para);
static AT_ERROR_CODE reset_handler(at_para_t *at_para);
static AT_ERROR_CODE help_handler(at_para_t *at_para);
static AT_ERROR_CODE getcfg_handler(at_para_t *at_para);
static AT_ERROR_CODE setcfg_handler(at_para_t *at_para);
static AT_ERROR_CODE ssidtxt_handler(at_para_t *at_para);
static AT_ERROR_CODE config_handler(at_para_t *at_para);
static AT_ERROR_CODE factory_handler(at_para_t *at_para);
static AT_ERROR_CODE save_handler(at_para_t *at_para);
static AT_ERROR_CODE status_handler(at_para_t *at_para);
static AT_ERROR_CODE ping_handler(at_para_t *at_para);
static AT_ERROR_CODE sockon_handler(at_para_t *at_para);
static AT_ERROR_CODE sockw_handler(at_para_t *at_para);
static AT_ERROR_CODE sockq_handler(at_para_t *at_para);
static AT_ERROR_CODE sockr_handler(at_para_t *at_para);
static AT_ERROR_CODE sockc_handler(at_para_t *at_para);
static AT_ERROR_CODE sockd_handler(at_para_t *at_para);
static AT_ERROR_CODE mode_handler(at_para_t *at_para);
static AT_ERROR_CODE wifi_handler(at_para_t *at_para);
static AT_ERROR_CODE reassociate_handler(at_para_t *at_para);
static AT_ERROR_CODE scan_handler(at_para_t *at_para);

typedef struct {
    at_di_t uart_id;
    at_di_t baudrate;
    at_di_t data_bits;
    at_di_t stop_bits;
    at_di_t parity;
    at_di_t hwfc;
} at_uart_para_t;

typedef struct {
    int sleepMode;
} at_sleep_para_t;

typedef struct {
    int powersaveMode;
} at_powersave_para_t;

typedef struct {
    int echoSwitch;
} at_echoswitch_para_t;

typedef struct {
    int autoconnectSwitch;
} at_autoconnect_para_t;

typedef struct {
    int IPDinfoSwitch;
} at_ipdinfo_para_t;

typedef struct {
    at_di_t gpioId;
	at_di_t edge;
} at_gpiowakeup_para_t;

typedef struct {
    at_di_t wifi_mode;
} at_wifiMode_para_t;

typedef struct {
    at_text_t ssid[AT_PARA_MAX_SIZE];
    at_text_t pwd[AT_PARA_MAX_SIZE];
} at_joinAp_para_t;

#define CMD_WLAN_MAX_BSS_CNT	50

enum atc_scanap_ecntype {
    ATC_SCANAP_ECN_OPEN = 0,
    ATC_SCANAP_ECN_WEP = 1,
    ATC_SCANAP_ECN_WPA_PSK = 2,
    ATC_SCANAP_ECN_WPA_WPA2_PSK = 3,
};

typedef struct {
    at_text_t ssid[AT_PARA_MAX_SIZE];
} at_getscan_para_t;

typedef struct {
    at_di_t  dhcpmode;
    at_di_t  dhcpen;
} at_dhcp_para_t;

typedef struct {
    at_hex_t mac[6];
} at_mac_para_t;

typedef struct {
    at_ip_t  ip;
    at_ip_t  getway;
    at_ip_t  netmask;
} at_ip_para_t;

typedef struct {
    at_text_t hostname[AT_PARA_MAX_SIZE];
} at_hostname_para_t;

static AT_ERROR_CODE version_handler(at_para_t *at_para);
static AT_ERROR_CODE echo_handler(at_para_t *at_para);
static AT_ERROR_CODE restore_handler(at_para_t *at_para);
static AT_ERROR_CODE uart_config_handler(at_para_t *at_para);
static AT_ERROR_CODE uart_config_save_handler(at_para_t *at_para);
static AT_ERROR_CODE sleep_handler(at_para_t *at_para);
static AT_ERROR_CODE powersave_handler(at_para_t *at_para);
static AT_ERROR_CODE wakeupgpio_handler(at_para_t *at_para);
static AT_ERROR_CODE wifi_mode_handler(at_para_t *at_para);
static AT_ERROR_CODE join_ap_handler(at_para_t *at_para);
static AT_ERROR_CODE join_ap_save_handler(at_para_t *at_para);
static AT_ERROR_CODE scan_attr_handler(at_para_t *at_para);
static AT_ERROR_CODE disconnect_handler(at_para_t *at_para);
static AT_ERROR_CODE setautoconnect_handler(at_para_t *at_para);
static AT_ERROR_CODE set_dhcp_handler(at_para_t *at_para);
static AT_ERROR_CODE set_mac_handler(at_para_t *at_para);
static AT_ERROR_CODE set_ip_handler(at_para_t *at_para);
static AT_ERROR_CODE set_hostname_handler(at_para_t *at_para);

static AT_ERROR_CODE network_status_handler(at_para_t *at_para);
static AT_ERROR_CODE dns_handler(at_para_t *at_para);
static AT_ERROR_CODE create_tcp_udp_handler(at_para_t *at_para);
static AT_ERROR_CODE send_tcp_handler(at_para_t *at_para);
static AT_ERROR_CODE close_network_handler(at_para_t *at_para);
static AT_ERROR_CODE set_multi_connect_handler(at_para_t *at_para);
static AT_ERROR_CODE set_transport_mode_handler(at_para_t *at_para);
static AT_ERROR_CODE set_dns_handler(at_para_t *at_para);
static AT_ERROR_CODE read_tcp_data_handler(at_para_t *at_para);
static AT_ERROR_CODE ipdinfo_handler(at_para_t *at_para);
static AT_ERROR_CODE set_recv_data_mode_handler(at_para_t *at_para);
static AT_ERROR_CODE set_hostname_handler(at_para_t *at_para);
static AT_ERROR_CODE send_data_handler(at_para_t *at_para);

extern AT_ERROR_CODE at_version(char * version);
extern AT_ERROR_CODE at_restore(char * address);
extern AT_ERROR_CODE at_uart_config(int uartId, int uartBaud, int dataBit, int parity, int stopBit, int hwfc);
extern AT_ERROR_CODE at_sleep(int sleepMode);
extern AT_ERROR_CODE at_wakeupgpio(int gpioId);
extern AT_ERROR_CODE at_wifi_mode(int wifiMode);
extern AT_ERROR_CODE at_setwupio(int wifiMode,int edge);
extern AT_ERROR_CODE at_join_ap(char * ssid, char * pwd);
extern AT_ERROR_CODE at_get_apinfo(void);
extern AT_ERROR_CODE at_scan_attr(char* at_para);
extern AT_ERROR_CODE at_disconnect(char * at_para);
extern AT_ERROR_CODE at_set_dhcp(unsigned char * at_para);
extern AT_ERROR_CODE at_set_mac(unsigned char * at_para);
extern AT_ERROR_CODE at_set_ip(unsigned char * at_para);
extern AT_ERROR_CODE at_set_hostname(char * at_para);
static AT_ERROR_CODE CWLAPscan_handler(at_para_t *at_para);
static AT_ERROR_CODE set_gpio_cfg_handler(at_para_t *at_para);
static AT_ERROR_CODE get_gpio_cfg_handler(at_para_t *at_para);
static AT_ERROR_CODE set_gpio_dir_handler(at_para_t *at_para);
static AT_ERROR_CODE set_gpio_handler(at_para_t *at_para);
static AT_ERROR_CODE read_gpio_handler(at_para_t *at_para);

at_callback_t at_callback;
static cmd_cache_t cache;
static cmd_send_cache_t send_cache;
static int reconnect_enable = 1;
extern struct netif *g_wlan_netif;

#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"

static const at_command_handler_t at_command_table[] = {
	{"AT",					attention_handler,	" -- Null cmd, always returns OK"},
	{"AT+ACT",				act_handler,		" -- Switch WiFi work mode"},
	{"AT+RST",				reset_handler,		" -- Reset the module"},
	{"AT+S.HELP",			help_handler,		" -- This text"},
	{"AT+S.GCFG",			getcfg_handler,		" =<key> -- Get config key"},
	{"AT+S.SCFG",			setcfg_handler,		" =<key>,<value> -- Set config key"},
	{"AT+S.SSIDTXT",		ssidtxt_handler,	" [=<ssidtxt>] -- Set a textual SSID (not hex), otherwise prints current SSID",},
	{"AT&V",				config_handler,		" -- Dump all settings",	},
	{"AT&F",				factory_handler,	" -- Restore factory default settings"},
	{"AT&W",				save_handler,		" -- Save current settings to flash",},
	{"AT+S.STS",			status_handler,		" [=<sts_var>] -- Report current status/statistics",},
	{"AT+S.PING",			ping_handler,		" =<hostname> -- Send a ping to a specified host"},
	{"AT+S.SOCKON",			sockon_handler,		" =<hostname>,<port>,<t|u> -- Open a network socket"},
	{"AT+S.SOCKW",			sockw_handler,		" =<id>,<len> -- Write data to socket"},
	{"AT+S.SOCKQ",			sockq_handler,		" =<id> -- Query socket for pending data",},
	{"AT+S.SOCKR",			sockr_handler,		" =<id>,<len> -- Read data from socket"},
	{"AT+S.SOCKC",			sockc_handler,		" =<id> -- Close socket"},
	{"AT+S.SOCKD",			sockd_handler,		" =<0|port>,<t|u> -- Disable/Enable socket server. Default is TCP"},
	{"AT+S.",				mode_handler,		" -- Switch to data mode",},
	{"AT+S.WIFI",			wifi_handler,		" =<0|1> -- Disable/Enable WiFi"},
	{"AT+S.ROAM",			reassociate_handler," -- Trigger a WiFi Roam"},
	{"AT+S.SCAN",			scan_handler,		" -- Perform a scan"},

    {"AT+ATE",				echo_handler,			" -- echo on/off"},
    {"AT+GMR",				version_handler,			" -- version info"},
    {"AT+UART_CUR",			uart_config_handler,		" -- config uart don't save into flash"},
    {"AT+UART_DEF",			uart_config_save_handler,		" -- config uart save into flash"},
    {"AT+RESTORE", 		  restore_handler,			  " -- erase all infomation"},
	{"AT+CWJAP",		join_ap_handler,			" -- join ap and do not save into flash"},
    {"AT+CWJAP_CUR",		join_ap_handler,			" -- join ap and do not save into flash"},
    {"AT+CWJAP_DEF",		join_ap_save_handler,			" -- join ap and save into flash"},
    {"AT+CWLAP",			CWLAPscan_handler,			" -- scan the ap near"},
    {"AT+CWQAP",			disconnect_handler,		" -- disconnect ap "},
    {"AT+CWAUTOCONN",			setautoconnect_handler,		" -- setautoconnect funciton "},
    {"AT+CWDHCP",		set_dhcp_handler,		" -- set dhcp address"},
	{"AT+CWDHCP_CUR",		set_dhcp_handler,		" -- set dhcp address"},
	{"AT+CIPSTAMAC_CUR",	set_mac_handler,		" -- set mac address"},
	{"AT+CIPSTAMAC",	set_mac_handler,		" -- set mac address"},
    {"AT+PING", 				 ping_handler,			 " -- ping work"},
	{"AT+CIPSTA_CUR",		set_ip_handler, 		" -- set ip address "},
    {"AT+CIPSTA",		set_ip_handler,			" -- set ip address "},

    {"AT+CIPSTART",         	create_tcp_udp_handler,	" -- create udp or tcp connection"},
    {"AT+CIPCLOSE", 			 close_network_handler,  " -- close the network"},
    {"AT+CIPSEND",          		send_data_handler,	" -- send the data"},
    {"AT+CIPSTATUS",          	network_status_handler,	" -- query network status"},

    {"AT+PS",				powersave_handler,			" -- enter into powersave mode"},
    {"AT+SLEEP",				sleep_handler,			" -- enter into sleep mode"},
    {"AT+WAKEUPGPIO",		wakeupgpio_handler,	" -- wake up by gpio"},
	{"AT+CWMODE",		wifi_mode_handler,		" -- set the wifi mode save into flash"},
    {"AT+CWMODE_CUR",		wifi_mode_handler,		" -- set the wifi mode save into flash"},
    {"AT+CWLAPOPT",			scan_attr_handler,		" -- set scan result attr"},
    {"AT+CWHOSTNAME",          set_hostname_handler,	" -- set network hostname"},

    {"AT+CIPDOMAIN",          	dns_handler,				" -- dns parse"},
    {"AT+CIPSENDBUF",          	send_tcp_handler,			" -- write data into tcp buffer "},
    {"AT+CIPMUX",          		set_multi_connect_handler,	" -- set mutil connection mode"},
    {"AT+CIPMODE",          	set_transport_mode_handler,	" -- set trasport mode "},
   	{"AT+CIPDNS",           set_dns_handler,	" -- set dns"},
    {"AT+CIPRECVDATA",         read_tcp_data_handler,	" -- read the data from tcp buffer"},
    {"AT+CIPDINFO",         ipdinfo_handler,	" --set whether disp the IPD char"},
    {"AT++CIPRECVMODE",     set_recv_data_mode_handler,	" -- set the tcp recv data mode"},
	{"AT+SYSIOSETCFG",			set_gpio_cfg_handler,	"-- set the gpio work mode"},
	{"AT+SYSIOGETCFG",			get_gpio_cfg_handler,	"-- get the gpio work mode"},
	{"AT+SYSGPIODIR",			set_gpio_dir_handler,	"-- set the gpio dir mode"},
	{"AT+SYSGPIOWRITE",			set_gpio_handler,		"-- set the gpio"},
	{"AT+SYSGPIOREAD",			read_gpio_handler,		"-- read the gpio"},

};

AT_ERROR_CODE at_help(void)
{
	s32 i;

	for (i = 0; i < TABLE_SIZE(at_command_table); i++) {
		at_dump("# %s%s\r\n", at_command_table[i].cmd, at_command_table[i].usage);
	}

	return AEC_OK;
}

static s32 at_match(char *cmd)
{
	s32 i;

	if (cmd == NULL) {
		return -2;
	}

	for (i = 0; i < TABLE_SIZE(at_command_table); i++) {
		if (!strcmp(cmd, at_command_table[i].cmd)) {
			return i;
		}
	}

	return -1;
}

/**
  * @brief  AT command initializer.
  * @param	cb: AT command callback function
  * @retval AEC_OK: succeed		Other: fail
  */
AT_ERROR_CODE at_init(at_callback_t *cb)
{
	at_callback_para_t para;

	memset(&at_callback, 0, sizeof(at_callback));

	if (cb ==NULL) {
		return AEC_UNDEFINED;
	}

	at_callback = *cb;

	para.cfg = &at_cfg;

	if (at_callback.handle_cb != NULL) {
		if (at_callback.handle_cb(ACC_LOAD, &para, NULL) != AEC_OK) {
			at_callback.handle_cb(ACC_FACTORY, &para, NULL);
		}
	}

	memset(&cache, 0, sizeof(cache));

	return AEC_OK;
}

static AT_ERROR_CODE at_parse_cmd(char *cmdline, s32 size)
{
	char at_cmd[AT_CMD_MAX_SIZE+1];
	at_para_t at_para;
	char *cptr;
	s32 idx;
	s32 cnt = 0;
	s32 i;

	cptr = cmdline;

	for (i = 0; i < size; i++) {
		if (*cptr == AT_EQU) {
			break;
        } else if (*cptr == AT_LF) {
			break;
        } else if (*cptr == AT_QUE) {
            break;
        } else if (*cptr == AT_CR) {
			if (((i+1) < size) && (*(cptr+1) == AT_LF)) {
				break;
            } else {
				break;
			}
        } else {
			if (cnt < sizeof(at_cmd)-1) {
				at_cmd[cnt++] = *cptr++;
            } else {
				return AEC_CMD_ERROR;
			}
		}
	}

	at_cmd[cnt] = '\0'; /* add string termination */

	if (cnt == 0) { /* skip blank line */
		return AEC_BLANK_LINE;
	}

	idx = at_match(at_cmd);

	if (idx >= 0) {
		if (at_command_table[idx].handler != NULL) {
			at_para.ptr = cptr;
			return at_command_table[idx].handler(&at_para);
        } else {
			return AEC_CMD_ERROR;
		}
    } else {
		return AEC_CMD_ERROR;
	}
}

/**
  * @brief  AT command parser.
  * @param	none
  * @retval AEC_OK: succeed		Other: fail
  */
AT_ERROR_CODE at_parse(void)
{
	AT_ERROR_CODE aec;
	AT_QUEUE_ERROR_CODE aqec;
	u8 tmp;
	u8 send_timeout_flag = 0;
	u32 flag = 0;
	u32 t0 = 0;

	memset(&send_cache,0,sizeof(cmd_send_cache_t));
	while(1) {

		aqec = at_queue_get(&tmp);

		if(aqec == AQEC_OK) {
			if(send_cache.status == 0){
			if (tmp == AT_LF) {
				if(cache.cnt < CMD_CACHE_MAX_LEN) {
					cache.buf[cache.cnt++] = tmp;
	                } else {
					cache.cnt = 0;
					AT_DBG("command is discarded!\n");
					continue; /* command is discarded */
				}

				flag = 1;
	            } else if (tmp == AT_CR) {
				cache.buf[cache.cnt++] = tmp;

				aqec = at_queue_peek(&tmp);
				if(aqec == AQEC_OK && tmp == AT_LF) {
					aqec = at_queue_peek(&tmp);
					if(cache.cnt < CMD_CACHE_MAX_LEN) {
						cache.buf[cache.cnt++] = tmp;
	                    } else {
						cache.cnt = 0;
						AT_DBG("command is discarded!\n");
						continue; /* command is discarded */
					}
				}
	                flag = 1;
	            }
			}else if(send_cache.status == 1)
	        {
	        	send_timeout_flag = 1;
				t0 = OS_GetTicks();
				send_cache.buf[send_cache.cnt++] = tmp;
	        	if(send_cache.cnt >= send_cache.length || cache.cnt > CMD_CACHE_MAX_LEN)
        		{
					memset(&cache,0,sizeof(cmd_cache_t));
					memcpy(cache.buf,"AT+CIPSEND=0,1,u",sizeof("AT+CIPSEND=0,1,u"));
					cache.cnt = sizeof("AT+CIPSEND=0,1,u");
				flag = 1;
					continue;
			}
	        }

			if(cache.cnt < CMD_CACHE_MAX_LEN) {
				cache.buf[cache.cnt++] = tmp;
            } else {
				cache.cnt = 0;
				AT_DBG("command is discarded!\n");
				continue; /* command is discarded */
			}
		}

		if(send_timeout_flag == 1)
		{
			if(OS_GetTicks() - t0 > CMD_SEND_TIMEOUT*1000)
			{
				send_timeout_flag = 0;
				memset(&send_cache,0,sizeof(cmd_send_cache_t));
				memset(&cache,0,sizeof(cmd_cache_t));
				at_response(AEC_SEND_TIMEOUT);
			}
		}
		if (flag) {
			/* echo */
			if (at_cfg.localecho1) {
				cache.buf[cache.cnt] = '\0';
				at_dump("%s", cache.buf);
			}

			aec = at_parse_cmd((char *)cache.buf, cache.cnt);

			if(send_cache.status == 1 && aec != AEC_SEND_READY)
			{
				send_timeout_flag = 0;
				memset(&send_cache,0,sizeof(cmd_send_cache_t));
			}
			at_response(aec);

			cache.cnt = 0;

			flag = 0;
		}

	}

	return AEC_OK;
}

static AT_ERROR_CODE attention_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	return res;
}

static AT_ERROR_CODE act_handler(at_para_t *at_para)
{
	AT_ERROR_CODE res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
    } else {
		return at_act();
	}
}

static AT_ERROR_CODE reset_handler(at_para_t *at_para)
{
	AT_ERROR_CODE res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
    } else {
		return at_reset();
	}
}

static AT_ERROR_CODE help_handler(at_para_t *at_para)
{
	AT_ERROR_CODE res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
    } else {
		return at_help();
	}
}

static AT_ERROR_CODE getcfg_handler(at_para_t *at_para)
{
	at_getcfg_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.key,  AET_LINE | SIZE_LIMIT(sizeof(cmd_para.key))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		return at_getcfg(cmd_para.key);
	}
}

static AT_ERROR_CODE setcfg_handler(at_para_t *at_para)
{
	config_key_t config_key;
	at_value_t config_value;
    at_para_descriptor_t cmd_key_list[] = {
		{APT_TEXT,	&config_key.key,  AET_PARA | SIZE_LIMIT(sizeof(config_key.key))},
    };
	at_para_descriptor_t cmd_value_list[] = {
		{APT_TEXT,	&config_value.text,		AET_LINE | SIZE_LIMIT(sizeof(config_value.text))},
		{APT_HEX,	&config_value.hex,		AET_LINE | SIZE_LIMIT(sizeof(config_value.hex))},
		{APT_DI,	&config_value.di,		AET_LINE | SIZE_LIMIT(sizeof(config_value.di))},
		{APT_HI,	&config_value.hi,		AET_LINE | SIZE_LIMIT(sizeof(config_value.di))},
		{APT_IP,	&config_value.ip,		AET_LINE | SIZE_LIMIT(sizeof(config_value.ip))},
    };
	s32 paracnt;
	s32 res;
	s32 type;
	s32 idx;
	s32 match = 0;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */
		memset(&config_key, 0, sizeof(config_key)); /* default value */
		res = at_get_parameters(&at_para->ptr, cmd_key_list, TABLE_SIZE(cmd_key_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		type = at_typecfg(config_key.key);

		for (idx = 0; idx < TABLE_SIZE(cmd_value_list); idx++) {
			if (cmd_value_list[idx].pt == type ) {
				match = 1;
				break;
			}
		}

		if (!match) {
			return AEC_NOT_FOUND;
        } else {
			memset(&config_value, 0, sizeof(config_value)); /* default value */
			res = at_get_parameters(&at_para->ptr, &cmd_value_list[idx], 1, &paracnt);

			if (res != AEC_OK)	{
				return AEC_PARA_ERROR;
			}

			if (paracnt != 1) {
				return AEC_PARA_ERROR;
			}

			return at_setcfg(config_key.key, &config_value);
		}
	}
}

static AT_ERROR_CODE ssidtxt_handler(at_para_t *at_para)
{
	at_ssidtxt_para_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.ssid,  AET_LINE | SIZE_LIMIT(sizeof(cmd_para.ssid))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		return at_ssidtxt(cmd_para.ssid);
	}
}

static AT_ERROR_CODE config_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr,NULL,0,NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
    } else {
		return at_config();
	}
}

static AT_ERROR_CODE factory_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
    } else {
		return at_factory();
	}
}

static AT_ERROR_CODE save_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
	}

	return at_save();
}

static AT_ERROR_CODE status_handler(at_para_t *at_para)
{
	at_status_para_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.sts_var,  AET_LINE | SIZE_LIMIT(sizeof(cmd_para.sts_var))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
        } else {
			return at_status(NULL);
		}
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK)	{
			return AEC_PARA_ERROR;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		return at_status(cmd_para.sts_var);
	}
}

static AT_ERROR_CODE ping_handler(at_para_t *at_para)
{
	at_ping_para_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.hostname,		AET_LINE | SIZE_LIMIT(sizeof(cmd_para.hostname))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		AT_WRN("%s\n",cmd_para.hostname);
		return at_ping(cmd_para.hostname);
	}
}

static AT_ERROR_CODE sockon_handler(at_para_t *at_para)
{
	at_sockon_para_t cmd_para =	{ /* default value */
		"",
		0,
		"t",
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.hostname,	AET_PARA | SIZE_LIMIT(sizeof(cmd_para.hostname))},
		{APT_DI,	&cmd_para.port,		AET_PARA | SIZE_LIMIT(sizeof(cmd_para.port))},
		{APT_TEXT,	&cmd_para.protocol,	AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(cmd_para.protocol))},
		{APT_TEXT,	&cmd_para.ind,		AET_LINE | SIZE_LIMIT(sizeof(cmd_para.ind))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 3) {
			return AEC_PARA_ERROR;
		}

		if (cmd_para.port < 0 || cmd_para.port > 65535) {
			return AEC_OUT_OF_RANGE;
		}

		if (!(!strcmp(cmd_para.protocol,"t") || !strcmp(cmd_para.protocol,"u"))) {
			return AEC_PARA_ERROR;
		}

		if (paracnt == 4) {
			if (!(!strcmp(cmd_para.ind, "ind"))) {
				return AEC_PARA_ERROR;
			}
		}

		return at_sockon(cmd_para.hostname, cmd_para.port, cmd_para.protocol, cmd_para.ind);
	}
}

static AT_ERROR_CODE sockw_handler(at_para_t *at_para)
{
	at_sockw_para_t cmd_para = { /* default value */
		"",
		0
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.id,	AET_PARA | SIZE_LIMIT(sizeof(cmd_para.id))},
		{APT_DI,	&cmd_para.len,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.len))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 2) {
			return AEC_PARA_ERROR;
		}

		return at_sockw(cmd_para.id, cmd_para.len);
	}
}

static AT_ERROR_CODE sockq_handler(at_para_t *at_para)
{
	at_sockq_para_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.id,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.id))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 1) {
			return AEC_PARA_ERROR;
		}

		return at_sockq(cmd_para.id);
	}
}

static AT_ERROR_CODE sockr_handler(at_para_t *at_para)
{
	at_sockr_para_t cmd_para = { /* default value */
		"",
		0
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.id,	AET_PARA | SIZE_LIMIT(sizeof(cmd_para.id))},
		{APT_DI,	&cmd_para.len,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.len))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 2) {
			return AEC_PARA_ERROR;
		}

		return at_sockr(cmd_para.id, cmd_para.len);
	}
}

static AT_ERROR_CODE sockc_handler(at_para_t *at_para)
{
	at_sockc_para_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.id,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.id))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 1) {
			return AEC_PARA_ERROR;
		}

		return at_sockc(cmd_para.id);
	}
}

static AT_ERROR_CODE sockd_handler(at_para_t *at_para)
{
	at_sockd_para_t cmd_para = { /* default value */
		0,
		"t"
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_DI,	&cmd_para.port,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(cmd_para.port))},
		{APT_TEXT,	&cmd_para.protocol,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.protocol))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 1) {
			return AEC_PARA_ERROR;
		}

		if (cmd_para.port < 0 || cmd_para.port > 65535) {
			return AEC_OUT_OF_RANGE;
		}

		if (!(!strcmp(cmd_para.protocol, "t") || !strcmp(cmd_para.protocol, "u"))) {
			return AEC_PARA_ERROR;
		}

		return at_sockd(cmd_para.port, cmd_para.protocol);
	}
}

static AT_ERROR_CODE mode_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
    } else {
		return at_mode(AM_DATA);
	}
}

static AT_ERROR_CODE wifi_handler(at_para_t *at_para)
{
	at_wifi_para_t cmd_para = { /* default value */
		0
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_DI,	&cmd_para.value,   AET_LINE | SIZE_LIMIT(sizeof(cmd_para.value))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
    } else {
		at_para->ptr++; /* skip '=' */
		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		if (cmd_para.value >= 0 && cmd_para.value <= 1) {
			return at_wifi(cmd_para.value);
        } else {
			return AEC_OUT_OF_RANGE;
		}
	}
}

static AT_ERROR_CODE reassociate_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
    } else {
		return at_reassociate();
	}
}

static AT_ERROR_CODE scan_handler(at_para_t *at_para)
{
	at_scan_para_t cmd_para = { /* default value */
		"",
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.mode,		AET_PARA | SIZE_LIMIT(sizeof(cmd_para.mode))},
		{APT_TEXT,	&cmd_para.repeat,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.repeat))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {

		res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
        } else {
			return at_scan(NULL, NULL);
		}
    } else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 2) {
			return AEC_PARA_ERROR;
		}

		if (!(!strcmp(cmd_para.mode, "a") || !strcmp(cmd_para.mode, "p"))) {
			return AEC_PARA_ERROR;
		}

		if (!(!strcmp(cmd_para.repeat, "r"))) {
			return AEC_PARA_ERROR;
		}

		return at_scan(cmd_para.mode, cmd_para.repeat);
	}
}

static AT_ERROR_CODE CWLAPscan_handler(at_para_t *at_para)
{
    at_lapscan_para_t scanParam ;
    at_para_descriptor_t cmd_para_list[] = {
        {APT_TEXT,	scanParam.ssid,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_lapscan_para_t))}
    };

    s32 paracnt;
    int res;
    u32 matchssid = 0;
    int i;

    if (*at_para->ptr == AT_EQU) {
        at_para->ptr++; /* skip '=' */
        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);
        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }
        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        matchssid = 1;
    } else if ((*at_para->ptr != AT_CR) && (*at_para->ptr != AT_LF)) {
        return AEC_PARA_ERROR;
    }

    wlan_sta_scan_results_t results;
    results.ap = malloc(CMD_WLAN_MAX_BSS_CNT * sizeof(wlan_sta_ap_t));
    if (results.ap == NULL) {
        AT_ERR("no mem\n");
        return AEC_CMD_FAIL;
    }
    res = wlan_sta_scan_once();
    OS_MSleep(2000);
    results.size = CMD_WLAN_MAX_BSS_CNT;
    res = wlan_sta_scan_result(&results);

    if (res == 0) {
        if(matchssid == 1) {
            for (i = 0; i < results.num; ++i) {
                if(strcmp((const char *)scanParam.ssid, (const char *)results.ap[i].ssid.ssid) == 0) {
                    at_dump("+CWLAP:\"%s\",%d,\"%x:%x:%x:%x:%x:%x\",\"%d\"\r\n",
                            results.ap[i].ssid.ssid,
                            results.ap[i].rssi,
                            results.ap[i].bssid[0],
                            results.ap[i].bssid[1],
                            results.ap[i].bssid[2],
                            results.ap[i].bssid[3],
                            results.ap[i].bssid[4],
                            results.ap[i].bssid[5],
                            (int)results.ap[i].channel);
                    break;
                }
            }
        } else {
            for (i = 0; i < results.num; ++i) {
                at_dump( "+CWLAP:\"%s\",%d,\"%x:%x:%x:%x:%x:%x\",\"%d\"\r\n",
                        results.ap[i].ssid.ssid,
                        results.ap[i].rssi,
                        results.ap[i].bssid[0],
                        results.ap[i].bssid[1],
                        results.ap[i].bssid[2],
                        results.ap[i].bssid[3],
                        results.ap[i].bssid[4],
                        results.ap[i].bssid[5],
                        (int)results.ap[i].channel);
            }
        }
        free(results.ap);
        return AEC_OK;
    }
    free(results.ap);
    return AEC_CMD_FAIL;
}

static AT_ERROR_CODE echo_handler(at_para_t *at_para)
{
    at_echoswitch_para_t  echoParam= {0};
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&echoParam.echoSwitch,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(echoParam.echoSwitch))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;
    } else {
        at_para->ptr++; /* skip '=' */
        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);
        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }
        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>echo = %d \n",echoParam.echoSwitch);
		if(echoParam.echoSwitch == 1)
		{
			at_cfg.localecho1 = 1;
		}
		else if(echoParam.echoSwitch == 0)
		{
			at_cfg.localecho1 = 0;
		}
		else
		{
			return AEC_PARA_ERROR;
		}
		return AEC_OK;
    }
}

static AT_ERROR_CODE version_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    if (*at_para->ptr != AT_EQU) {
        return at_version(NULL);
    }
    return AEC_OK;
}

static AT_ERROR_CODE restore_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    if (*at_para->ptr != AT_EQU) {
        return at_restore(NULL);
    }
    return AEC_OK;
}

static AT_ERROR_CODE uart_config_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_uart_para_t uartParam;
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&uartParam.uart_id,			AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(uartParam.uart_id))},
        {APT_DI,	&uartParam.baudrate,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(uartParam.baudrate))},
		{APT_DI,	&uartParam.data_bits,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(uartParam.stop_bits))},
        {APT_DI,	&uartParam.parity,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(uartParam.data_bits))},
        {APT_DI,	&uartParam.stop_bits,			AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(uartParam.parity))},
        {APT_DI,	&uartParam.hwfc,			AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(uartParam.hwfc))},
    };
    s32 paracnt;
    int res;
    if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;
    } else {
        at_para->ptr++; /* skip '=' */
        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);
        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }
        if (paracnt < 6) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>uart_id = %d \n",uartParam.uart_id);
        AT_WRN("------>baudrate = %d \n",uartParam.baudrate);
        AT_WRN("------>data_bits = %d \n",uartParam.data_bits);
        AT_WRN("------>parity = %d \n",uartParam.parity);
        AT_WRN("------>stop_bits = %d \n",uartParam.stop_bits);
        AT_WRN("------>hwfc = %d \n",uartParam.hwfc);
        return at_uart_config(uartParam.uart_id,uartParam.baudrate,uartParam.data_bits,uartParam.parity,uartParam.stop_bits,uartParam.hwfc);
    }
}

static AT_ERROR_CODE uart_config_save_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
	return AEC_OK;
}


static AT_ERROR_CODE powersave_handler(at_para_t *at_para)
{
    at_powersave_para_t  powersaveParam= {0};
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&powersaveParam.powersaveMode,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(powersaveParam.powersaveMode))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {

        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>powersaveMode = %d \n",powersaveParam.powersaveMode);

		if(powersaveParam.powersaveMode == 1)
		{
			wlan_set_ps_mode(g_wlan_netif, 1);
		}
		else if(powersaveParam.powersaveMode == 0)
		{
			wlan_set_ps_mode(g_wlan_netif, 0);
		}
		else
		{
			return AEC_PARA_ERROR;
		}
		return AEC_OK;
    }

}


static AT_ERROR_CODE sleep_handler(at_para_t *at_para)
{
    at_sleep_para_t  sleepParam= {0};
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&sleepParam.sleepMode,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(sleepParam.sleepMode))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {

        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>sleepMode = %d \n",sleepParam.sleepMode);
        return at_sleep(sleepParam.sleepMode);
    }

}

static AT_ERROR_CODE wakeupgpio_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);

    at_gpiowakeup_para_t wakeupgpioParam = {0};
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&wakeupgpioParam.gpioId,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(wakeupgpioParam.gpioId))},
        {APT_DI,	&wakeupgpioParam.edge,			AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(wakeupgpioParam.edge))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;
    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 2) {
            return AEC_PARA_ERROR;
        }

		if(wakeupgpioParam.gpioId <=0 && wakeupgpioParam.gpioId >=9)
			return AEC_PARA_ERROR;
		if(wakeupgpioParam.gpioId == 4)
			return AEC_PARA_ERROR;
		if(wakeupgpioParam.edge != 0 && wakeupgpioParam.edge != 1)
		{
			return AEC_PARA_ERROR;
		}

        AT_DBG("------>wakeupgpio = %d EDGE = %d \n",wakeupgpioParam.gpioId,wakeupgpioParam.edge);
        return at_setwupio(wakeupgpioParam.gpioId,wakeupgpioParam.edge);
    }

    return AEC_OK;
}


static AT_ERROR_CODE wifi_mode_handler(at_para_t *at_para)
{
    at_wifiMode_para_t wifiParam = {0};
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&wifiParam.wifi_mode,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(wifiParam.wifi_mode))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;
    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>wifi mode = %d \n",wifiParam.wifi_mode);
        return at_wifi_mode(wifiParam.wifi_mode);
    }

}

void set_reconnect_enable(void)
{
	reconnect_enable = 1;
}
void set_reconnect_disable(void)
{
	reconnect_enable = 0;
}
int get_reconnect_enable_status(void)
{
	return reconnect_enable;
}

AT_ERROR_CODE at_get_ssid_psk(char **ppara,char *ssid,char *pwd)
{
	char *para = NULL;
	char *ptr = NULL;
	int i;

	para = *ppara;
	ptr = ssid;
	i = 0;
	while(i < (SYSINFO_SSID_LEN_MAX+SYSINFO_PSK_LEN_MAX+1))
	{
		if(*para == '\\')
		{
			para++;
			i++;
			*ptr = *para;
			para++;
			ptr++;
		}
		else if(*para == '"')
		{
			para++;
			i++;
		}
		else if(*para == ',')
		{
			ptr = pwd;
			para++;
			i++;
		}
		else if(*para == AT_CR)
		{
			para++; /* skip <CR> */
			if (*para == AT_LF)
			{
				para++; /* skip <LF> */
			}
			break;
		}
		else
		{
			*ptr = *para;
			para++;
			ptr++;
		}
		i++;
	}
	return i<(SYSINFO_SSID_LEN_MAX+SYSINFO_PSK_LEN_MAX+1)?AEC_OK:AEC_PARA_ERROR;
}

static AT_ERROR_CODE join_ap_handler(at_para_t *at_para)
{
    AT_WRN("------>\n");

    AT_ERROR_CODE ret;

    at_joinAp_para_t joinParam ;
	memset(&joinParam.ssid[0], 0x00, sizeof(at_joinAp_para_t));

    int res;

    if (*at_para->ptr == AT_QUE) {
	     return at_get_apinfo();

    } else if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

		res = at_get_ssid_psk(&at_para->ptr,(char *)&joinParam.ssid,(char *)&joinParam.pwd);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        AT_WRN("------> ssid = %s, pwd = %s \n", joinParam.ssid,joinParam.pwd);
		set_reconnect_disable();
        ret = at_join_ap(joinParam.ssid,joinParam.pwd);
        if(ret == AEC_OK)
    	{
       		return AEC_OK; /* succeed */
    	}

        else {
            at_dump("\r\n+CWJAP_CUR:%d \r\n", at_get_errorcode());
            return AEC_CMD_FAIL;
        }
    }

}

static AT_ERROR_CODE join_ap_save_handler(at_para_t *at_para)
{
    AT_WRN("------>\n");

    AT_ERROR_CODE ret;

    at_joinAp_para_t joinParam ;
	memset(&joinParam.ssid[0], 0x00, sizeof(at_joinAp_para_t));

    int res;

    if (*at_para->ptr == AT_QUE) {
	    return at_get_apinfo();

    } else if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

		res = at_get_ssid_psk(&at_para->ptr,(char *)&joinParam.ssid,(char *)&joinParam.pwd);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        AT_WRN("------> ssid = %s, pwd = %s \n", joinParam.ssid,joinParam.pwd);
		set_reconnect_disable();
        ret = at_join_ap(joinParam.ssid,joinParam.pwd);

        if(ret == AEC_OK)
    	{
       		return AEC_OK; /* succeed */
    	}

        else {
            at_dump("\r\n+CWJAP_CUR:%d \r\n", at_get_errorcode());
            return AEC_CMD_FAIL;
        }
    }

}

static AT_ERROR_CODE scan_attr_handler(at_para_t *at_para)
{
    AT_WRN("------>\n");

    return AEC_OK;
}

static AT_ERROR_CODE disconnect_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);

    if (*at_para->ptr != AT_EQU) {
        at_disconnect(NULL);
		set_reconnect_disable();
        OS_MSleep(200);
        return AEC_OK;
    }

    return AEC_OK;

}

static AT_ERROR_CODE setautoconnect_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_autoconnect_para_t  autoParam= {0};
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&autoParam.autoconnectSwitch,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(autoParam.autoconnectSwitch))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {

        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>autoParam = %d \n",autoParam.autoconnectSwitch);

		if(autoParam.autoconnectSwitch == 1)
		{
			set_reconnect_enable();
		}
		else if(autoParam.autoconnectSwitch == 0)
		{
			set_reconnect_disable();
		}
		else
		{
			return AEC_PARA_ERROR;
		}
		return AEC_OK;

    }


    return AEC_OK;

}


static AT_ERROR_CODE set_dhcp_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_dhcp_para_t  dhcpParam ;
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&dhcpParam.dhcpmode,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_dhcp_para_t))},
        {APT_DI,	&dhcpParam.dhcpen,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_dhcp_para_t))},
    };

    s32 paracnt;
    int res;
    int tmp = 0;

    struct sysinfo *sysinfo = sysinfo_get();

    if(*at_para->ptr == AT_QUE) {
        if(at_cfg.apDhcp == 1)
            tmp |= 0x01;
        if(at_cfg.staDhcp == 1)
            tmp |= 0x02;

        at_dump("\r\n+CWDHCP:%02x \r\n", tmp);
        return AEC_OK;
    } else if (*at_para->ptr == AT_EQU) {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        if(dhcpParam.dhcpmode == 0) {
            at_cfg.apDhcp = dhcpParam.dhcpen;
        } else if(dhcpParam.dhcpmode == 1) {
            if(dhcpParam.dhcpen < 2) {
                at_cfg.staDhcp = dhcpParam.dhcpen;
                sysinfo->sta_use_dhcp = dhcpParam.dhcpen;
            }
        } else if(dhcpParam.dhcpmode == 2) {
            if(dhcpParam.dhcpen < 2) {
                at_cfg.staDhcp = dhcpParam.dhcpen;
                at_cfg.apDhcp = dhcpParam.dhcpen;
                sysinfo->sta_use_dhcp = dhcpParam.dhcpen;
            }
        } else
            return AEC_PARA_ERROR;
    } else {
        return AEC_PARA_ERROR;
    }

    return AEC_OK;
}


static AT_ERROR_CODE set_mac_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_mac_para_t  macParam;
    at_para_descriptor_t cmd_para_list[] = {
        {APT_HEX,	&macParam.mac[0],		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(unsigned char))},
        {APT_HEX,	&macParam.mac[1],		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(unsigned char))},
        {APT_HEX,	&macParam.mac[2],		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(unsigned char))},
        {APT_HEX,	&macParam.mac[3],		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(unsigned char))},
        {APT_HEX,	&macParam.mac[4],		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(unsigned char))},
        {APT_HEX,	&macParam.mac[5],		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(unsigned char))},
    };

    s32 paracnt;
    int res;
    struct sysinfo *sysinfo = sysinfo_get();

    if (*at_para->ptr == AT_QUE) {
        at_dump("+CIPSTAMAC:\"%02x:%02x:%02x:%02x:%02x:%02x\"\r\n",
                sysinfo->mac_addr[0],
                sysinfo->mac_addr[1],
                sysinfo->mac_addr[2],
                sysinfo->mac_addr[3],
                sysinfo->mac_addr[4],
                sysinfo->mac_addr[5]);
        return AEC_OK;
    } else if (*at_para->ptr == AT_EQU) {

        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 5) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>macParam.mac = %02x:%02x:%02x:%02x:%02x:%02x \n",macParam.mac[0],macParam.mac[1],macParam.mac[2],macParam.mac[3],macParam.mac[4],macParam.mac[5]);

        memcpy(sysinfo->mac_addr, macParam.mac, 6);
        sysinfo_save();

        return AEC_OK;
    }

    return AEC_OK;
}

static AT_ERROR_CODE set_ip_handler(at_para_t *at_para)
{

    AT_WRN("------>%s\n",__func__);
    at_ip_para_t  ipParam;
    at_para_descriptor_t cmd_para_list[] = {
        {APT_IP,	&ipParam.ip,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(ipParam.ip))},
        {APT_IP,   &ipParam.getway, 	   AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(ipParam.ip))},
        {APT_IP,	&ipParam.netmask,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(ipParam.ip))},

    };

    s32 paracnt;
    int res;

    struct sysinfo *sysinfo = sysinfo_get();


    if (*at_para->ptr == AT_QUE) {
        res = is_netconnet_ap();
        if(res == 1 ) {
		at_dump("+CIPSTA:ip:\"%s\"\r\n",ipaddr_ntoa(&g_wlan_netif->ip_addr));
		at_dump("+CIPSTA:gateway:\"%s\"\r\n",ipaddr_ntoa(&g_wlan_netif->gw));
		at_dump("+CIPSTA:netmask:\"%s\"\r\n",ipaddr_ntoa(&g_wlan_netif->netmask));
		}

    } else if (*at_para->ptr == AT_EQU) {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }

        memcpy((char *)(&sysinfo->netif_sta_param.ip_addr), ipParam.ip, sizeof(at_ip_t));
        memcpy((char *)(&sysinfo->netif_sta_param.gateway), ipParam.getway, sizeof(at_ip_t));
        memcpy((char *)(&sysinfo->netif_sta_param.net_mask), ipParam.netmask, sizeof(at_ip_t));

        sysinfo->sta_use_dhcp = 0;

        AT_WRN("------>ipParam = %d,%d,%d,%d \n",at_cfg.ip_ipaddr[0],at_cfg.ip_ipaddr[1],at_cfg.ip_ipaddr[2],at_cfg.ip_ipaddr[3]);
        return AEC_OK;
    }

    return AEC_OK;
}

static AT_ERROR_CODE set_hostname_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_hostname_para_t  hostParam;
    at_para_descriptor_t cmd_para_list[] = {
        {APT_TEXT,	&hostParam.hostname,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(hostParam.hostname))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {

        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>hostParam = %s \n",hostParam.hostname);
        return at_set_hostname(hostParam.hostname);
    }


    return AEC_OK;
}

typedef struct {
    at_text_t address[AT_PARA_MAX_SIZE];
} at_domain_para_t;



typedef struct {
    at_text_t buffer[AT_PARA_MAX_SIZE];
} at_tcp_data_para_t;

typedef struct {
    at_di_t linkId;
} at_close_para_t;

typedef struct {
    at_di_t mux;
} at_mux_para_t;

typedef struct {
    at_di_t mode;
} at_trans_mode_para_t;


typedef struct {
    at_ip_t dns;
} at_dns_para_t;


extern AT_ERROR_CODE at_network_status(char* param,       at_callback_rsp_t *rsp);
extern AT_ERROR_CODE at_domain_query(char* dnsAdress) ;
extern AT_ERROR_CODE at_create_network_connect(at_network_para_t netpara, at_callback_rsp_t *rsp);
extern AT_ERROR_CODE at_send_tcp_buffer(char* tcpBuffer);
extern AT_ERROR_CODE at_close_network(int linkId);
extern AT_ERROR_CODE at_mux_network(int mux);
extern AT_ERROR_CODE at_set_trans_mode(int mode);
extern AT_ERROR_CODE at_set_dns(char* dnsAdress);
extern AT_ERROR_CODE at_send_data(int linkId, char* dataBuffer, s32 dataBufferLen);
extern AT_ERROR_CODE at_io_cfg(int ID, int mode,int driving, int pull);
extern AT_ERROR_CODE at_get_io_cfg(void);
extern AT_ERROR_CODE at_set_iodir_cfg(int ID, int mode);
extern AT_ERROR_CODE at_write_io_data(int ID, int data);
extern AT_ERROR_CODE at_read_io_data(int GPIOnum);
extern AT_ERROR_CODE at_delete_ap(int apnum);

static AT_ERROR_CODE network_status_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);

    at_callback_rsp_t rsp;

	int res;

    rsp.vptr = c_atCmdRspBuf;
    memset(c_atCmdRspBuf, 0, 1024);

    if (*at_para->ptr != AT_EQU) {

		res = at_network_status(NULL, &rsp);

		if(rsp.status == 1) {
			at_dump("%s \n", rsp.vptr);
		}

        return res;
    }
    return AEC_OK;
}

static AT_ERROR_CODE dns_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_domain_para_t  domainParam;
    at_para_descriptor_t cmd_para_list[] = {
        {APT_TEXT,	&domainParam.address,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(domainParam.address))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {

        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>dns address = %s \n",domainParam.address);
        return at_domain_query(domainParam.address);
    }

    return AEC_OK;
}

static AT_ERROR_CODE create_tcp_udp_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_network_para_t networkPara= {
        .linkId = -1,
        .keepAlive = 0,
    };
    at_callback_rsp_t rsp;

    rsp.vptr = c_atCmdRspBuf;
    memset(c_atCmdRspBuf, 0, 1024);

    at_para_descriptor_t cmd_para_type[] = {
        {APT_DI,	&networkPara.linkId,			AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_network_para_t))},
        {APT_TEXT,	networkPara.type,				AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_network_para_t))},
    };

    at_para_descriptor_t cmd_para_TCP_list[] = {
        {APT_TEXT,		networkPara.hostname,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_network_para_t))},
        {APT_DI,		&networkPara.port,				AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_network_para_t))},
        {APT_DI,		&networkPara.keepAlive, 	AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_network_para_t))},
    };

    at_para_descriptor_t cmd_para_UDP_list[] = {
        {APT_TEXT,		networkPara.hostname,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_network_para_t))},
        {APT_DI,		&networkPara.port,				AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_network_para_t))},
        {APT_DI,		&networkPara.localport, 	AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_network_para_t))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {

        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_type, TABLE_SIZE(cmd_para_type), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }
        if(networkPara.linkId > 4)
            return AEC_PARA_ERROR;

        if(strcmp(networkPara.type, "TCP") == 0) {
            res = at_get_parameters(&at_para->ptr, cmd_para_TCP_list, TABLE_SIZE(cmd_para_TCP_list), &paracnt);
            if (res != AEC_OK) {
                return AEC_PARA_ERROR;
            }
        } else if(strcmp(networkPara.type, "UDP") == 0) {
            res = at_get_parameters(&at_para->ptr, cmd_para_UDP_list, TABLE_SIZE(cmd_para_UDP_list), &paracnt);
            if (res != AEC_OK) {
                return AEC_PARA_ERROR;
            }
        } else if(strcmp(networkPara.type, "SSL") == 0) {
            return AEC_PARA_ERROR;

        } else {
            return AEC_PARA_ERROR;
        }

        AT_WRN("------>networkPara.linkId = %d, \n",networkPara.linkId);
        AT_WRN("------>networkPara.type = %s, \n",networkPara.type);
        AT_WRN("------>networkPara.hostname = %s \n",networkPara.hostname);
        AT_WRN("------>networkPara.port = %d, \n",networkPara.port);
        AT_WRN("------>networkPara.localport = %d, \n",networkPara.localport);
        AT_WRN("------>networkPara.keepAlive = %d \n",networkPara.keepAlive);
        res = at_create_network_connect(networkPara, &rsp);

        if(rsp.status == 1) {
            at_dump("%s \n", rsp.vptr);
        }
        return res;
    }

    return AEC_OK;
}

static AT_ERROR_CODE send_tcp_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_tcp_data_para_t  tcpBuffer ;
    at_para_descriptor_t cmd_para_list[] = {
        {APT_TEXT,	&tcpBuffer.buffer,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_tcp_data_para_t))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {

        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>tcpBuffer.buffer= %s \n",tcpBuffer.buffer);
        return at_send_tcp_buffer(tcpBuffer.buffer);
    }
    return AEC_OK;
}

static AT_ERROR_CODE close_network_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_close_para_t closePara = {-1};
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&closePara.linkId,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_close_para_t))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {

        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>closePara.linkId = %d \n",closePara.linkId);
        return at_close_network(closePara.linkId);
    }

    return AEC_OK;
}


static AT_ERROR_CODE set_multi_connect_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_mux_para_t muxPara = {-1};
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&muxPara.mux,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_mux_para_t))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr == AT_QUE) {

        if(at_cfg.CIPMUX > 1)
            at_cfg.CIPMUX = 0;
        at_dump("\r\n+ CIPMUX:%d\r\n", at_cfg.CIPMUX);
        return AEC_OK;

    } else if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>muxPara.mux = %d \n",muxPara.mux);
        return at_mux_network(muxPara.mux);
    }
    return AEC_OK;
}

static AT_ERROR_CODE set_transport_mode_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_trans_mode_para_t transModePara = {-1};
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&transModePara.mode,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_trans_mode_para_t))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {

        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>transModePara.mode = %d \n",transModePara.mode);
        return at_set_trans_mode(transModePara.mode);
    }
    return AEC_OK;
}

static AT_ERROR_CODE set_dns_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_dns_para_t dnsPara;

    at_para_descriptor_t cmd_para_list[] = {
        {APT_IP,	&dnsPara.dns,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_dns_para_t))},
    };

    s32 paracnt;
    int res;
	ip_addr_t getdnsip;

	 if(*at_para->ptr == AT_QUE) {
	    	//memcpy(&getdnsip,dns_getserver(0),sizeof(ip_addr_t));
	    	getdnsip = dns_getserver(0);
			at_dump("+CIPDNS:\"%s\"\r\n",ipaddr_ntoa(&getdnsip));
	        return AEC_OK;

	    }
	 else if (*at_para->ptr != AT_EQU) {

		 return AEC_PARA_ERROR;

	 }else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>dnsPara.dns = %d.%d.%d.%d \n",dnsPara.dns[0],dnsPara.dns[1],dnsPara.dns[2],dnsPara.dns[3]);
        return at_set_dns((char*)dnsPara.dns);
    }
    return AEC_OK;
}

static AT_ERROR_CODE read_tcp_data_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    return AEC_OK;
}

extern int is_disp_ipd;
static AT_ERROR_CODE ipdinfo_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_ipdinfo_para_t  IPDParam= {0};
    at_para_descriptor_t cmd_para_list[] = {
        {APT_DI,	&IPDParam.IPDinfoSwitch,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(IPDParam.IPDinfoSwitch))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {

        return AEC_PARA_ERROR;

    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 1) {
            return AEC_PARA_ERROR;
        }
        AT_WRN("------>IPDParam = %d \n",IPDParam.IPDinfoSwitch);

		if(IPDParam.IPDinfoSwitch == 1)
		{
			is_disp_ipd = 1;
		}
		else if(IPDParam.IPDinfoSwitch == 0)
		{
			is_disp_ipd = 0;
		}
		else
		{
			return AEC_PARA_ERROR;
		}
		return AEC_OK;

    }

    return AEC_OK;

}


static AT_ERROR_CODE set_recv_data_mode_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);

    return AEC_OK;
}

static AT_ERROR_CODE send_data_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_sendData_para_t sendDataPara= {-1};

    at_para_descriptor_t cmd_para_list_single[] = {
        {APT_DI,	&sendDataPara.linkId,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_sendData_para_t))},
        {APT_DI,	&sendDataPara.bufferlen,	AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_sendData_para_t))},
    };

    s32 paracnt;
    int res;

    AT_WRN("at_cfg.CIPMUX== %d\r\n", at_cfg.CIPMUX);
    if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;
    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list_single, TABLE_SIZE(cmd_para_list_single), &paracnt);
        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }

        if (paracnt < 2) {
            return AEC_PARA_ERROR;
        }
		if(send_cache.status == 0)
		{
			if(sendDataPara.linkId >= 4 || sendDataPara.bufferlen > CMD_SEND_DATA_MAX_LEN)
			{
				memset(&send_cache,0,sizeof(cmd_send_cache_t));
				return AEC_PARA_ERROR;
			}
			else
			{
				send_cache.linkID = sendDataPara.linkId;
				send_cache.length = sendDataPara.bufferlen;
				send_cache.status = 1;
				return AEC_SEND_READY;
			}
		}
		else
		{
			sendDataPara.linkId = send_cache.linkID;
			sendDataPara.bufferlen = send_cache.length;
			sendDataPara.buffer = (at_text_t *)&send_cache.buf;
		}
        AT_WRN("------>sendDataPara.buffer = %d ,%s \n",sendDataPara.bufferlen, sendDataPara.buffer);

        return at_send_data(sendDataPara.linkId,  sendDataPara.buffer, sendDataPara.bufferlen);
    }

    return AEC_OK;
}

static AT_ERROR_CODE set_gpio_cfg_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_setgpio_para_t gpioPara= {-1};

    at_para_descriptor_t cmd_para_list_single[] = {
        {APT_DI,	&gpioPara.ID,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_setgpio_para_t))},
        {APT_DI,	&gpioPara.mode,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_setgpio_para_t))},
		{APT_DI,	&gpioPara.driving,	AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_setgpio_para_t))},
		{APT_DI,	&gpioPara.pull,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_setgpio_para_t))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;
    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list_single, TABLE_SIZE(cmd_para_list_single), &paracnt);
        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }
        if (paracnt < 4) {
            return AEC_PARA_ERROR;
        }
		if(gpioPara.ID > 3)
			return AEC_PARA_ERROR;

		if(gpioPara.mode >= 2)
			return AEC_PARA_ERROR;

		if(gpioPara.driving >= 1)
			return AEC_PARA_ERROR;

		if(gpioPara.pull >= 4)
			return AEC_PARA_ERROR;

        return at_io_cfg(gpioPara.ID,  gpioPara.mode,gpioPara.driving,gpioPara.pull);
    }

    return AEC_OK;
}

static AT_ERROR_CODE get_gpio_cfg_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    AT_ERROR_CODE res;

    res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

    if (res != AEC_OK) {
        return AEC_PARA_ERROR;
    } else {
        return at_get_io_cfg();
    }

}

static AT_ERROR_CODE set_gpio_dir_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_setgpiodir_para_t gpioPara= {-1};

    at_para_descriptor_t cmd_para_list_single[] = {
        {APT_DI,	&gpioPara.ID,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_setgpiodir_para_t))},
        {APT_DI,	&gpioPara.mode,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_setgpiodir_para_t))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;
    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list_single, TABLE_SIZE(cmd_para_list_single), &paracnt);
        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }
        if (paracnt < 2) {
            return AEC_PARA_ERROR;
        }

		if(gpioPara.ID > 3)
			return AEC_PARA_ERROR;

		if(gpioPara.mode >= 2)
			return AEC_PARA_ERROR;

        return at_set_iodir_cfg(gpioPara.ID,  gpioPara.mode);
    }

    return AEC_OK;
}

static AT_ERROR_CODE set_gpio_handler(at_para_t *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_writegpio_para_t gpioPara= {-1};

    at_para_descriptor_t cmd_para_list_single[] = {
        {APT_DI,	&gpioPara.ID,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_writegpio_para_t))},
        {APT_DI,	&gpioPara.data,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_writegpio_para_t))},
    };

    s32 paracnt;
    int res;

    if (*at_para->ptr != AT_EQU) {
        return AEC_PARA_ERROR;
    } else {
        at_para->ptr++; /* skip '=' */

        res = at_get_parameters(&at_para->ptr, cmd_para_list_single, TABLE_SIZE(cmd_para_list_single), &paracnt);
        if (res != AEC_OK) {
            return AEC_PARA_ERROR;
        }
        if (paracnt < 2) {
            return AEC_PARA_ERROR;
        }

		if(gpioPara.ID > 3)
			return AEC_PARA_ERROR;

		if(gpioPara.data >= 2)
			return AEC_PARA_ERROR;

        return at_write_io_data(gpioPara.ID,  gpioPara.data);
    }

    return AEC_OK;
}
static AT_ERROR_CODE read_gpio_handler(at_para_t *at_para)
{
	AT_WRN("------>%s\n",__func__);
	at_di_t GPIOnum = -1;

	at_para_descriptor_t cmd_para_list_single[] = {
		{APT_DI,	&GPIOnum,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(at_di_t))},
	};

	s32 paracnt = 1;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_PARA_ERROR;
	} else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list_single, TABLE_SIZE(cmd_para_list_single), &paracnt);
		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}
		if (paracnt < 1) {
			return AEC_PARA_ERROR;
		}

		if(GPIOnum > 3)
			return AEC_PARA_ERROR;

		return at_read_io_data(GPIOnum);
	}

    return AEC_OK;
}

