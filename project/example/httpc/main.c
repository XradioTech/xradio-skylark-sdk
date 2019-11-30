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
#include "net/HTTPClient/HTTPCUsr_api.h"
#include "mbedtls/mbedtls.h"

#define HTTPC_DEMO_THREAD_STACK_SIZE (8 * 1024) /* ssl need more stack */
static OS_Thread_t httpc_demo_thread;

#define HTTPC_DEMO_URL0 "http://hc.apache.org/httpclient-3.x"
#define HTTPC_DEMO_URL1 "https://tls.mbed.org"

static HTTPParameters httpc_demo_param;
char httpc_demo_buf[1024];

/* this CA only use in https://tls.mbed.org */
static char *httpc_demo_ca =
"-----BEGIN CERTIFICATE-----\r\n"
"MIIDdTCCAl2gAwIBAgILBAAAAAABFUtaw5QwDQYJKoZIhvcNAQEFBQAwVzELMAkG\r\n"
"A1UEBhMCQkUxGTAXBgNVBAoTEEdsb2JhbFNpZ24gbnYtc2ExEDAOBgNVBAsTB1Jv\r\n"
"b3QgQ0ExGzAZBgNVBAMTEkdsb2JhbFNpZ24gUm9vdCBDQTAeFw05ODA5MDExMjAw\r\n"
"MDBaFw0yODAxMjgxMjAwMDBaMFcxCzAJBgNVBAYTAkJFMRkwFwYDVQQKExBHbG9i\r\n"
"YWxTaWduIG52LXNhMRAwDgYDVQQLEwdSb290IENBMRswGQYDVQQDExJHbG9iYWxT\r\n"
"aWduIFJvb3QgQ0EwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDaDuaZ\r\n"
"jc6j40+Kfvvxi4Mla+pIH/EqsLmVEQS98GPR4mdmzxzdzxtIK+6NiY6arymAZavp\r\n"
"xy0Sy6scTHAHoT0KMM0VjU/43dSMUBUc71DuxC73/OlS8pF94G3VNTCOXkNz8kHp\r\n"
"1Wrjsok6Vjk4bwY8iGlbKk3Fp1S4bInMm/k8yuX9ifUSPJJ4ltbcdG6TRGHRjcdG\r\n"
"snUOhugZitVtbNV4FpWi6cgKOOvyJBNPc1STE4U6G7weNLWLBYy5d4ux2x8gkasJ\r\n"
"U26Qzns3dLlwR5EiUWMWea6xrkEmCMgZK9FGqkjWZCrXgzT/LCrBbBlDSgeF59N8\r\n"
"9iFo7+ryUp9/k5DPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNVHRMBAf8E\r\n"
"BTADAQH/MB0GA1UdDgQWBBRge2YaRQ2XyolQL30EzTSo//z9SzANBgkqhkiG9w0B\r\n"
"AQUFAAOCAQEA1nPnfE920I2/7LqivjTFKDK1fPxsnCwrvQmeU79rXqoRSLblCKOz\r\n"
"yj1hTdNGCbM+w6DjY1Ub8rrvrTnhQ7k4o+YviiY776BQVvnGCv04zcQLcFGUl5gE\r\n"
"38NflNUVyRRBnMRddWQVDf9VMOyGj/8N7yy5Y0b2qvzfvGn9LhJIZJrglfCm7ymP\r\n"
"AbEVtQwdpf5pLGkkeB6zpxxxYu7KyJesF12KwvhHhm4qxFYxldBniYUr+WymXUad\r\n"
"DKqC5JlR3XC321Y9YeRq4VzW9v493kHMB65jUr9TU/Qr6cf9tveCX4XSQRjbgbME\r\n"
"HMUfpIBvFSDJ3gyICh3WZlXi/EjJKSZp4A==\r\n"
"-----END CERTIFICATE-----\r\n";

static void* httpc_demo_set_client_cert(void)
{
	static security_client httpc_demo_cert;
	memset(&httpc_demo_cert, 0, sizeof(httpc_demo_cert));

	httpc_demo_cert.pCa = httpc_demo_ca;
	httpc_demo_cert.nCa = strlen(httpc_demo_ca) + 1;

	return &httpc_demo_cert;
}

static void httpc_demo_download(char *url, int use_ssl)
{
	uint32_t download_len = 0;
	INT32 recv_len = 0;
	int32_t ret;
	uint32_t download_time = 0;

	memset(&httpc_demo_param, 0, sizeof(httpc_demo_param));
	memcpy(httpc_demo_param.Uri, url, strlen(url));
	httpc_demo_param.nTimeout = 30; //timeout 30s for every get

	if (use_ssl) {
		/* set CA cert */
		HTTPC_Register_user_certs(httpc_demo_set_client_cert);
		/* set ssl verify mode */
		HTTPC_set_ssl_verify_mode(2);
	}

	download_time = OS_GetTicks();

	while (1) {
		ret = HTTPC_get(&httpc_demo_param, httpc_demo_buf, sizeof(httpc_demo_buf), &recv_len);
		if (ret != HTTP_CLIENT_SUCCESS) {
			break;
		}
		download_len += recv_len;
		printf("%.*s", sizeof(httpc_demo_buf), httpc_demo_buf);
	}
	if (ret == HTTP_CLIENT_EOS) {
		download_time = OS_GetTicks() - download_time;
		printf("\n\ndownload complete, download_len %d, time %ds\n",
					download_len, download_time/1000);
	} else {
		printf("\n\ndownload error, ret=%d\n", ret);
	}
}

static void httpc_demo_fun(void *arg)
{
	httpc_demo_download(HTTPC_DEMO_URL0, 0);
	httpc_demo_download(HTTPC_DEMO_URL1, 1);

	OS_ThreadDelete(&httpc_demo_thread);
}

static void net_cb(uint32_t event, uint32_t data, void *arg)
{
	uint16_t type = EVENT_SUBTYPE(event);

	switch (type) {
		case NET_CTRL_MSG_NETWORK_UP:
			if (!OS_ThreadIsValid(&httpc_demo_thread)) {
				OS_ThreadCreate(&httpc_demo_thread,
									"httpc_demo_thread",
									httpc_demo_fun,
									(void *)NULL,
									OS_THREAD_PRIO_APP,
									HTTPC_DEMO_THREAD_STACK_SIZE);
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

	printf("httpc demo start\n\n");

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

