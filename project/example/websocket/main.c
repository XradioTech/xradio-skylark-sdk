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
#include "net/nopoll/nopoll.h"
#include "lwip/sockets.h"

#define WEBSOCKET_DEMO_THREAD_STACK_SIZE (8 * 1024) /* ssl need more stack */
static OS_Thread_t websocket_demo_thread;

//ws://123.207.167.163:9010/ajaxchattest
static char *host_name1 = "123.207.167.163";
static char *host_port1 = "9010";
static char *get_url1 = "/ajaxchattest";

//wss://echo.websocket.org
static char *host_name2 = "echo.websocket.org";
static char *host_port2 = "443";
static char *get_url2 = "/";

/* this CA only use in wss://echo.websocket.org */
static char *websocket_demo_ca =
"-----BEGIN CERTIFICATE-----\r\n"
"MIIDSjCCAjKgAwIBAgIQRK+wgNajJ7qJMDmGLvhAazANBgkqhkiG9w0BAQUFADA/\r\n"
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\r\n"
"DkRTVCBSb290IENBIFgzMB4XDTAwMDkzMDIxMTIxOVoXDTIxMDkzMDE0MDExNVow\r\n"
"PzEkMCIGA1UEChMbRGlnaXRhbCBTaWduYXR1cmUgVHJ1c3QgQ28uMRcwFQYDVQQD\r\n"
"Ew5EU1QgUm9vdCBDQSBYMzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEB\r\n"
"AN+v6ZdQCINXtMxiZfaQguzH0yxrMMpb7NnDfcdAwRgUi+DoM3ZJKuM/IUmTrE4O\r\n"
"rz5Iy2Xu/NMhD2XSKtkyj4zl93ewEnu1lcCJo6m67XMuegwGMoOifooUMM0RoOEq\r\n"
"OLl5CjH9UL2AZd+3UWODyOKIYepLYYHsUmu5ouJLGiifSKOeDNoJjj4XLh7dIN9b\r\n"
"xiqKqy69cK3FCxolkHRyxXtqqzTWMIn/5WgTe1QLyNau7Fqckh49ZLOMxt+/yUFw\r\n"
"7BZy1SbsOFU5Q9D8/RhcQPGX69Wam40dutolucbY38EVAjqr2m7xPi71XAicPNaD\r\n"
"aeQQmxkqtilX4+U9m5/wAl0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNV\r\n"
"HQ8BAf8EBAMCAQYwHQYDVR0OBBYEFMSnsaR7LHH62+FLkHX/xBVghYkQMA0GCSqG\r\n"
"SIb3DQEBBQUAA4IBAQCjGiybFwBcqR7uKGY3Or+Dxz9LwwmglSBd49lZRNI+DT69\r\n"
"ikugdB/OEIKcdBodfpga3csTS7MgROSR6cz8faXbauX+5v3gTt23ADq1cEmv8uXr\r\n"
"AvHRAosZy5Q6XkjEGB5YGV8eAlrwDPGxrancWYaLbumR9YbK+rlmM6pZW87ipxZz\r\n"
"R8srzJmwN0jP41ZL9c8PDHIyh8bwRLtTcm1D9SZImlJnt1ir/md2cXjbDaJWFBM5\r\n"
"JDGFoqgCWjBH4d1QB7wCCZAA62RjYJsWvIjJEubSfZGL+T0yjWW06XyxV3bqxbYo\r\n"
"Ob8VZRzI9neWagqNdwvYkQsEjgfbKbYK7p2CNTUQ\r\n"
"-----END CERTIFICATE-----\r\n";

static noPollCtx *nopoll_client_ctx = NULL;
static noPollConn *nopoll_conn = NULL;
static noPollConnOpts *nopoll_conn_opts = NULL;

#define WEBSOCKET_DEMO_TEXT "websocket demo test"

static int nopoll_complete_pending_write(noPollConn *conn)
{
	int tries = 0;
	while (tries < 5 && errno == NOPOLL_EWOULDBLOCK &&
			nopoll_conn_pending_write_bytes (conn) > 0) {
		nopoll_sleep(50000);
		if (nopoll_conn_complete_pending_write (conn) == 0)
			return 0;
		tries++;
	}
	return 1;
}

static int nopoll_client_demo_test(char *host_name, char *host_port,
											char *get_url, int use_ssl)
{
	struct hostent *host_entry;
	int ret = -1;
	char *host_ip;

	/* 1. init nopoll */

	/* new client context */
	nopoll_client_ctx = nopoll_ctx_new();
	if (nopoll_client_ctx == NULL) {
		printf("nopoll_ctx_new failed\n");
		ret = -1;
		goto exit;
	}
	/* open the nopoll debug log */
	//nopoll_log_enable(nopoll_client_ctx, nopoll_true);

	/* new connect options */
	nopoll_conn_opts = nopoll_conn_opts_new();
	if (nopoll_conn_opts == NULL) {
		printf("nopoll_conn_opts_new failed\n");
		ret = -1;
		goto exit;
	}

	/* get host ip, nopoll must use host ip */
	host_entry = gethostbyname(host_name);
	if (host_entry != NULL) {
		host_ip = inet_ntoa(*(struct in_addr*)host_entry->h_addr);
		printf("host ip:%s\n", host_ip);
	} else {
		printf("get host name err\n");
		goto exit;
	}

	/* 2. connect to server */

	if (use_ssl == 0) {
		/* if not use ssl */
		nopoll_conn = nopoll_conn_new_opts(nopoll_client_ctx, nopoll_conn_opts,
											host_ip, host_port,
											host_name, get_url,
											NULL, NULL);
	} else {
		/* if use ssl, then set the certs */
		if (!nopoll_conn_opts_set_ssl_certs(nopoll_conn_opts,
											NULL, 0,
											NULL, 0,
											NULL, 0,
											websocket_demo_ca, strlen(websocket_demo_ca) + 1)) {
			printf("nopoll_conn_opts_set_ssl_certs failed\n");
			goto exit;
		}

		/* set ssl verfy */
		nopoll_conn_opts_ssl_peer_verify(nopoll_conn_opts, nopoll_true);
		nopoll_conn = nopoll_conn_tls_new(nopoll_client_ctx, nopoll_conn_opts,
											host_ip, host_port,
											host_name, get_url,
											NULL, NULL);
	}

	if (nopoll_conn == NULL) {
		printf("nopoll_conn create err\n");
		goto exit;
	}

	/* wait 10s until the connection ready  */
	if (nopoll_conn_wait_until_connection_ready(nopoll_conn, 10))
		printf("the connection is ready\n");
	else {
		printf("connection timeout\n");
		goto exit;
	}

	/* 3. send data to server */
	int len = strlen(WEBSOCKET_DEMO_TEXT);
	printf("send data to %s: %s\n", host_name, WEBSOCKET_DEMO_TEXT);
	ret = nopoll_conn_send_text(nopoll_conn, WEBSOCKET_DEMO_TEXT, len);
	if (ret != strlen(WEBSOCKET_DEMO_TEXT)) {
		/* if the actual data sent and the data want sent are not equal */
		if (nopoll_complete_pending_write(nopoll_conn))
			printf("size = %u, but nopoll_conn_send_text ret = %d\n", len, ret);
	}

	/* 4. recv data from server */

	noPollMsg *msg;
	const char *content;
	int times = 0;
	while (1) {
		if (!nopoll_conn_is_ok(nopoll_conn)) {
			printf("received websocket connection close\n");
			break;
		}
		msg = nopoll_conn_get_msg(nopoll_conn);
		if (msg) {
			content = (const char *)nopoll_msg_get_payload(msg);
			printf("rece data from %s: %s\n", host_name, content);

			nopoll_msg_unref(msg);
			break;
		} else {
			nopoll_sleep(100000); // 1s
			times++;
			if (times > 10) // try 10 times
				break;
		}
	}

exit:

	/* 5. close/free connect */

	if (nopoll_conn != NULL) {
		nopoll_conn_close(nopoll_conn);
		nopoll_conn = NULL;
	}

	if (nopoll_conn_opts != NULL) {
		nopoll_conn_opts_free(nopoll_conn_opts);
		nopoll_conn_opts = NULL;
	}

	if (nopoll_client_ctx != NULL) {
		nopoll_ctx_unref(nopoll_client_ctx);
		nopoll_client_ctx = NULL;
	}

	return ret;
}

static void websocket_demo_fun(void *arg)
{

	while (1) {
		nopoll_client_demo_test(host_name1, host_port1, get_url1, 0);
		nopoll_client_demo_test(host_name2, host_port2, get_url2, 1);
		OS_MSleep(1000);
	}
	OS_ThreadDelete(&websocket_demo_thread);
}

static void net_cb(uint32_t event, uint32_t data, void *arg)
{
	uint16_t type = EVENT_SUBTYPE(event);

	switch (type) {
		case NET_CTRL_MSG_NETWORK_UP:
			if (!OS_ThreadIsValid(&websocket_demo_thread)) {
				OS_ThreadCreate(&websocket_demo_thread,
									"websocket_demo_thread",
									websocket_demo_fun,
									(void *)NULL,
									OS_THREAD_PRIO_APP,
									WEBSOCKET_DEMO_THREAD_STACK_SIZE);
			}
			break;

		case NET_CTRL_MSG_NETWORK_DOWN:
			break;

		default:
			break;
	}
}

int main(void)
{
	observer_base *net_ob;

	platform_init();

	printf("websocket demo start\n\n");

	printf("use these commands to connect ap:\n\n");
	printf("1. config ssid and password : net sta config ssid password\n");
	printf("2. enable sta to connect ap : net sta enable\n\n");

	/* create an observer to monitor the net work state */
	net_ob = sys_callback_observer_create(CTRL_MSG_TYPE_NETWORK,
									     NET_CTRL_MSG_ALL,
									     net_cb,
									     NULL);
	if(net_ob == NULL)
		return -1;

	if(sys_ctrl_attach(net_ob) != 0)
		return -1;

	return 0;
}

