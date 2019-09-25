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


AT_ERROR_CODE at_version(char* version)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_GMR, &para, NULL);
    }

    return AEC_OK; /* succeed */
}

AT_ERROR_CODE at_restore(char* address)
{
    AT_WRN("------>%s\n",__func__);

    at_callback_para_t para;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_RESTORE, &para, NULL);
    }

    return AEC_OK; /* succeed */

}

AT_ERROR_CODE at_uart_config(int uartId,int uartBaud,int dataBit,int parity, int stopBit, int hwfc)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

    para.u.uart.uartId = uartId;
    para.u.uart.uartBaud = uartBaud;
    para.u.uart.dataBit = dataBit;
    para.u.uart.parity = parity;
    para.u.uart.stopBit = stopBit;
    para.u.uart.hwfc = hwfc;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_UART_DEF, &para, NULL);
    }

    return AEC_OK; /* succeed */

}

AT_ERROR_CODE at_sleep(int sleepMode)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
    para.u.sleep.sleepMode = sleepMode;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_SLEEP, &para, NULL);
    }

    return AEC_OK; /* succeed */
}

AT_ERROR_CODE at_wakeupgpio(int gpioId)
{
    AT_WRN("------>%s\n",__func__);

    at_callback_para_t para;
    para.u.wakeupgpio.gpioId = gpioId;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_WAKEUPGPIO, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

AT_ERROR_CODE at_wifi_mode(int wifiMode)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
    para.u.wifiMode.mode = wifiMode;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CWMODE_CUR, &para, NULL);
    }

    return AEC_OK; /* succeed */
}

AT_ERROR_CODE at_setwupio(int gpioId,int edge)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
    para.u.wakeupgpio.gpioId = gpioId;
	para.u.wakeupgpio.edge   = edge;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_WAKEUPGPIO, &para, NULL);
    }

    return AEC_OK; /* succeed */
}


AT_ERROR_CODE at_get_apinfo(void)
{
	return (at_callback.handle_cb != NULL)? at_callback.handle_cb(ACC_CWJAP_INFO, NULL, NULL) : AEC_CMD_FAIL;
}

AT_ERROR_CODE at_join_ap(char*ssid, char* pwd)
{
    AT_ERROR_CODE ret = AEC_CMD_FAIL;

    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
    strcpy(para.u.joinParam.ssid,ssid);
    strcpy(para.u.joinParam.pwd,pwd);
	
    para.cfg = &at_cfg;
	
    para.cfg->wifi_ssid_len =  strlen(ssid);
    if(para.cfg->wifi_ssid_len >= sizeof(para.cfg->wifi_ssid))
        return AEC_LIMITED;
	
    memset(para.cfg->wifi_ssid, 0, sizeof(para.cfg->wifi_ssid));
    memset(para.cfg->wifi_wpa_psk_text, 0, sizeof(para.cfg->wifi_wpa_psk_text));
    strcpy((char *)para.cfg->wifi_ssid,ssid);
    strcpy((char *)para.cfg->wifi_wpa_psk_text,pwd);
	
    if (at_callback.handle_cb != NULL) {
        ret = at_callback.handle_cb(ACC_CWJAP_CUR, &para, NULL);
    }

    if(ret == AEC_OK)
        return AEC_OK; /* succeed */
    else
        return AEC_CMD_FAIL;
}

AT_ERROR_CODE at_scan_attr(char *at_para)
{
    AT_WRN("------>%s\n",__func__);

    return AEC_OK; /* succeed */
}

AT_ERROR_CODE at_disconnect(char *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CWQAP, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

AT_ERROR_CODE at_set_dhcp(unsigned char *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CWQAP, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

AT_ERROR_CODE at_set_mac(unsigned char *at_para)
{
    AT_WRN("------>%s\n",__func__);
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CWQAP, &para, NULL);
    }
    return AEC_OK; /* succeed */

}

AT_ERROR_CODE at_set_ip(unsigned char *at_para)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CWQAP, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

AT_ERROR_CODE at_set_hostname(char *hostname)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
    strcpy(para.u.iphostname.hostname,hostname);
    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CWQAP, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

extern AT_ERROR_CODE at_network_status(char* param,       at_callback_rsp_t *rsp)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CIPSTATUS, &para, rsp);
    }
    return AEC_OK; /* succeed */
}
extern AT_ERROR_CODE at_domain_query(char* dnsAdress)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
	int res = AEC_OK;

    para.cfg = &at_cfg;
	memcpy(&para.u.dns_parse.hostname,dnsAdress,sizeof(at_cfg.ip_hostname));

    if (at_callback.handle_cb != NULL) {
        res = at_callback.handle_cb(ACC_CIPDOMAIN, &para, NULL);
    }
    return res; /* succeed */
}


extern AT_ERROR_CODE at_create_network_connect(at_network_para_t netpara,
        at_callback_rsp_t *rsp)
{

    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
	int res = AEC_OK;

    para.u.net_param.linkId = netpara.linkId ;
    strcpy(para.u.net_param.networkType, netpara.type);
    strcpy(para.u.net_param.hostname, netpara.hostname);
    para.u.net_param.port = netpara.port;
    para.u.net_param.localport = netpara.localport;
    para.u.net_param.keepAlive = netpara.keepAlive;
	strcpy(para.u.net_param.type, netpara.type);

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        res = at_callback.handle_cb(ACC_CIPSTART, &para, rsp);
    }
	
    return res; /* succeed */
}

extern AT_ERROR_CODE at_send_tcp_buffer(char* tcpBuffer)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CWQAP, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

extern AT_ERROR_CODE at_close_network(int linkId)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
    para.u.close_id.id = linkId;
    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CIPCLOSE, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

extern AT_ERROR_CODE at_mux_network(int mux)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

    para.cfg = &at_cfg;

    at_cfg.CIPMUX = mux;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CWQAP, &para, NULL);
    }
    return AEC_OK; /* succeed */
}


extern AT_ERROR_CODE at_set_trans_mode(int mode)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CWQAP, &para, NULL);
    }
    return AEC_OK; /* succeed */
}


extern AT_ERROR_CODE at_set_dns(char* dnsAdress)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;

	memcpy((char *)(&para.u.set_dns.setdnsip), dnsAdress, sizeof(at_ip_t));
    para.cfg = &at_cfg;
	
    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CIPDNS_CUR, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

extern AT_ERROR_CODE at_send_data(int linkId, char* dataBuffer, s32 dataBufferLen)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
	
    para.u.sockw.id = linkId;
    para.u.sockw.len = dataBufferLen;
    para.u.sockw.buf = (u8 *)dataBuffer;
	
    para.cfg = &at_cfg;
	
    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_CIPSEND, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

extern AT_ERROR_CODE at_io_cfg(int ID, int mode,int driving, int pull)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
	
    para.u.setgpio_para.ID = ID;
    para.u.setgpio_para.mode = mode;
    para.u.setgpio_para.driving = driving;
	para.u.setgpio_para.pull = pull;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_SYSIOSETCFG, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

AT_ERROR_CODE at_get_io_cfg(void)
{
	if (at_callback.handle_cb != NULL) {
		at_callback.handle_cb(ACC_SYSIOGETCFG, NULL, NULL);
	}

	return AEC_OK;
}

extern AT_ERROR_CODE at_set_iodir_cfg(int ID, int mode,int driving, int pull)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
	
    para.u.setiodir_para.ID = ID;
    para.u.setiodir_para.mode = mode;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_SYSGPIODIR, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

extern AT_ERROR_CODE at_write_io_data(int ID, int data)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
	
    para.u.writeiodata_para.ID = ID;
    para.u.writeiodata_para.data = data;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_SYSGPIOWRITE, &para, NULL);
    }
    return AEC_OK; /* succeed */
}

extern AT_ERROR_CODE at_read_io_data(int ID, int data)
{
    AT_WRN("------>%s\n",__func__);
    at_callback_para_t para;
	
    para.u.readiodata_para.ID = ID;

    para.cfg = &at_cfg;

    if (at_callback.handle_cb != NULL) {
        at_callback.handle_cb(ACC_SYSGPIOREAD, &para, NULL);
    }
    return AEC_OK; /* succeed */
}


