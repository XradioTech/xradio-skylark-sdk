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
#include <string.h>
#include <stdlib.h>

#include "kernel/os/os.h"
#include "net/wlan/wlan.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"

#include "soft_ap_config.h"
#include "mbedtls/base64.h"

#include "kernel/os/os.h"

#define APP_INFO			"XRADIO Demo: Web server and OTA"
#define HTTP_DATA_MAX_LEN	2048

typedef struct
{
	char *pToken1;	//HTTP request
	char *pToken2;	//HTTP path
	char *pToken3;  //URL function
} http_token;

typedef enum {
	HTTP_DISPLAY_STANDARD,
	HTTP_DISPLAY_IOS,
	HTTP_DISPLAY_ANDROID
} httpDisplayType;

static char userName[] = "admin";
static char usrPassword[] = "admin";
static char *auth_str = NULL;
static char *http_buffer = NULL;
static int xr_server_fd = -1;
static int xr_client_fd = -1;
static SOFT_AP_CONFIG_STA soft_ap_run_state = SOFT_AP_CONFIG_STOP;
static soft_ap_config_cb soft_ap_callback;

#define SOFT_AP_CONFIG_THREAD_STACK_SIZE	(2 * 1024)
static OS_Thread_t soft_ap_thread;

static soft_ap_config_result soft_ap_result;

static const char headerPage[] = {
"HTTP/1.1 200 OK\r\n\
Server: MySocket Server\r\n\
Date: TEST\r\n\
Content-Type: text/html\r\n\
Content-Length: %d\r\n\
Connection: close\r\n\
Accept-Ranges: bytes\r\n\r\n"
};

static const char HTTPSaveResponse[] = {
"HTTP/1.1 200 OK\r\n\
Server: MySocket Server\r\n\
Date: TEST\r\n\
Content-Type: text/html\r\n\
Content-Length: %d\r\n\
Accept-Ranges: bytes\r\n\
Connection: close\r\n\r\n\
%s"
};

static const char authrized[] = {
"HTTP/1.1 401 Authorization Required\r\n"
"Server: MySocket Server\r\n"
"WWW-Authenticate: Basic realm=\"XRADIO\"\r\n"
"Content-Type: text/html\r\n"
"Content-Length: 169\r\n\r\n"
"<HTML>\r\n<HEAD>\r\n<TITLE>Error</TITLE>\r\n"
"<META HTTP-EQUIV=\"Content-Type\" CONTENT=\"text/html; charset=ISO-8859-1\">\r\n"
"</HEAD>\r\n<BODY><H1>401 Unauthorized.</H1></BODY>\r\n</HTML>"
};

static const char not_found[] = {
"HTTP/1.1 200 OK\r\n\
Server: MySocket Server\r\n\
Content-Length: 145\r\n\
Connection: close\r\n\
Content-Type: text/html\r\n\r\n\
<html><head><title>404 Not Found</title></head><body>\r\n\
<h1>Not Found</h1>\r\n\
<p>The requested URL was not found on this server.</p>\r\n\
</body></html>"
};

static const char systemPage[] = {
"<html><head><title>System Setting</title>\r\n\
</head>\r\n\
<body>\
<br /><font size=\"6\" color=\"red\">XRADIO HTTP server and OTA demo</font><br /><br />\r\n\
<br />Firmware Version:&nbsp;%s&nbsp;<br />\r\n\
<form action=\"settings.htm\" method=\"post\">\r\n\
<table id=\"displayme\" border=\"0\" width=\"500\" cellspacing=\"2\">\r\n\
<col align=\"right\" /> <col align=\"left\" />\r\n\
<tbody><tr><td>SSID:&nbsp;</td>\r\n\
<td><Input type=\"text\" name=\"SSID\" value = \"%s\"/></td></tr>\r\n\
<tr><td>Key:&nbsp;</td> <td><Input type=\"text\" name=\"PSK\" value= \"%s\"/></td></tr>\r\n\
</tbody></table><br />\r\n\
<INPUT type=\"submit\" name=\"save\" value=\"Save\">\
<INPUT type=\"submit\" name=\"reset\" value=\"Reset\" ><br />\
</FORM>\r\n\
<FORM ENCTYPE=\"multipart/form-data\" action=\"update.htm\" METHOD=POST>\r\n\
<label>Update firmware: <input type=\"file\" name=\"imagefile\" accept=\"bin\"></label>\
<input type=\"submit\" name=\"upload\" value=\"Upload\">\
</FORM>\
</body></html>\r\n"
};

static const char systemResponseSucc[] = {
"<html>\r\n\
<head>\r\n\
<title>XRADIO Wi-Fi module</title>\r\n\
</head>\r\n\
<body>\r\n\
<p>Firmware update success, system reboot...please wait 5 seconds and refresh</p>\r\n\
</body>\r\n\
</html>"};

static const char systemResponseError[] = {
"<html>\r\n\
<head>\r\n\
<title>XRADIO Wi-Fi module</title>\r\n\
</head>\r\n\
<body>\r\n\
<p>Firmware update fail, system reboot...please wait 5 seconds and refresh</p>\r\n\
</body>\r\n\
</html>"};


static const char SaveResponseSucc[] = {
"<html>\r\n\
<head>\r\n\
<title>XRADIO Wi-Fi module</title>\r\n\
</head>\r\n\
<body>\r\n\
<p>Save Config Done!<a href=\"/system.htm\">Return</a></p>\r\n\
</body>\r\n\
</html>"};

static const char SaveResponseError[] = {
"<html>\r\n\
<head>\r\n\
<title>XRADIO Wi-Fi module</title>\r\n\
</head>\r\n\
<body>\r\n\
<p>Save Config Error, please retry<a href=\"/system.htm\">Return</a></p>\r\n\
</body>\r\n\
</html>"};

static const char ResponseReset[] = {
"<html>\r\n\
<head>\r\n\
<title>XRADIO Wi-Fi module</title>\r\n\
</head>\r\n\
<body>\r\n\
<p>Reset system, please wait 5 seconds and refresh</p>\r\n\
</body>\r\n\
</html>"};

static int auth_init(char *name, char *passwd)
{
	int ret = 0;
	uint32_t len, outlen;
	char *src_str = NULL;

	len = strlen(name) + strlen(passwd) + 2;

	if (auth_str)
		free(auth_str);

	auth_str = malloc(len * 2);
	if (auth_str == NULL) {
		ret = -1;
		goto out;
	}
	memset(auth_str, 0, len * 2);

	src_str = malloc(len);
	if (src_str == 0) {
		ret = -1;
		goto out;
	}

	sprintf(src_str, "%s:%s", name, passwd);
	ret = mbedtls_base64_encode((unsigned char *)auth_str, len * 2,
				&outlen, (const unsigned char *)src_str, strlen(src_str));
	if (ret != 0) {
		SAC_ERR("base64 encode err! ret:%d\n", ret);
		goto out;
	}

	SAC_DBG("auth_str:%s\n", auth_str);

out:
	if (src_str)
		free(src_str);

	return ret;
}

static int soft_ap_http_init(void)
{
	int ret = 0;

	xr_client_fd = -1;
	xr_server_fd = -1;

	http_buffer = malloc(HTTP_DATA_MAX_LEN);
	if (http_buffer == NULL) {
		SAC_ERR("malloc http_buffer failed\n");
		return -1;
	}

	if ((ret = auth_init(userName, usrPassword)) != 0) {
		free(http_buffer);
		ret = -1;
	}

	return -1;
}

static int soft_ap_http_deinit(void)
{
	if (auth_str) {
		free(auth_str);
		auth_str = NULL;
	}

	if (http_buffer) {
		free(http_buffer);
		http_buffer = NULL;
	}

	return 0;
}

static void stop_http_server(void)
{
	if (xr_client_fd >= 0) {
		closesocket(xr_client_fd);
		xr_client_fd = -1;
	}

	if (xr_server_fd >= 0) {
		closesocket(xr_server_fd);
		xr_server_fd = -1;
	}
}

static int send_http_data(int fd, char *data, int len)
{
	int w_size = 0, pos = 0;

	while (1) {
		pos = send(fd, (uint8_t*)data + w_size, len - w_size, 0);
		if (pos < 0) {
			SAC_ERR("send data err: %d\n", errno);
			return pos;
		}
		w_size += pos;
		if (w_size == len)
			break;
	}

	SAC_DBG("\n%.*s\n", len, data);

	return w_size;
}

static int http_parse(char* pStr, http_token *httpToken)
{
	char* pch = strchr(pStr, ' ');
	char* pch2 = strchr(pStr, '/');
	char* pch3;
	httpToken->pToken1 = pStr;

	if (pch) {
		*pch='\0';
		pch++;
		pch3 = strchr(pch, ' ');
		if (pch2 && pch2 < pch3) {
			httpToken->pToken2 = pch2;
			pch2++;
			pch2 = strchr(pch2, '/');
			if (pch2 && pch2 < pch3) {
				pch2++;
				httpToken->pToken3 = pch2;
			} else {
				httpToken->pToken3 = NULL;
			}
		} else {
			httpToken->pToken2 = NULL;
		}
		return 1;
	}
	return 0;
}

static void html_decode(char *p, int len)
{
	int i, j, val;
	char assic[4];

	for (i=0; i<len; i++) {
		if (p[i] == '+') {
			p[i] = ' ';
		}
		if (p[i] == '%') {
			if ((i+2) >= len) {
				return;
			}

			assic[2] = 0;
			assic[0] = p[i+1];
			assic[1] = p[i+2];
			val = strtol(assic, NULL, 16);

			p[i] = val;
			for (j = i+1; j< len;j++) {
				p[j] = p[j+2];
			}
		}
	}
}

static uint8_t http_post_parse(char** ppStr, const char* pFlag, char** ppValue)
{
	char* pch=strstr(*ppStr, pFlag);
	char* pch2=NULL;

	if (pch) {
		pch2 = strchr(pch, '=');
		if (!pch2) return 0;
		pch2++;
		*ppValue = pch2;
		if (!*ppValue) return 0;
		pch = strchr(pch2, '&');
		if (pch) {
			*pch='\0';
			html_decode(pch2, strlen(pch2));
			*ppStr=pch+1;
			return 1;
		}
	}

	return 0;
}

static void save_reset_response(uint8_t result, int fd)
{
	if (result == 1) {
		sprintf(http_buffer, HTTPSaveResponse, strlen(SaveResponseSucc), SaveResponseSucc);
	} else if (result == 0) {
		sprintf(http_buffer, HTTPSaveResponse, strlen(SaveResponseError), SaveResponseError);

	} else if (result == 2) {
		sprintf(http_buffer, HTTPSaveResponse, strlen(ResponseReset), ResponseReset);
	}

	send_http_data(fd, http_buffer, strlen(http_buffer));
}

static void send_system_page(int fd)
{
	char *body;
	uint32_t len = 0;

	memset(http_buffer, 0, HTTP_DATA_MAX_LEN);
	body = http_buffer;
	len = sprintf(body, systemPage, APP_INFO, soft_ap_result.ssid, soft_ap_result.psk);

	len = sprintf(http_buffer, headerPage, len);
	body = http_buffer + len;

	sprintf(body, systemPage, APP_INFO, soft_ap_result.ssid, soft_ap_result.psk);

	send_http_data(fd, http_buffer, strlen(http_buffer));
}

static void get_settings_post(int fd, char *postdata, int len)
{
	char* pToken1, *pValue;
	pToken1 = postdata;

	if(!(http_post_parse(&pToken1, "SSID", &pValue))) {
		SAC_DBG("save_reset_response ssid\n");
		save_reset_response(0, fd);
	 	return;
	}

	strcpy(soft_ap_result.ssid, pValue);
	SAC_DBG("get ssid : %s\r\n", soft_ap_result.ssid);

	if(!http_post_parse(&pToken1, "PSK", &pValue)) {
		SAC_DBG("save_reset_response PSK\n");
		save_reset_response(0, fd);
	  	return;
	}

	strcpy(soft_ap_result.psk, pValue);
	SAC_DBG("get psk : %s\r\n", soft_ap_result.psk);

	if(strstr(pToken1, "save")) {
		SAC_DBG("save_reset_response save\n");
		if (soft_ap_callback) {
			soft_ap_callback(&soft_ap_result, SOFT_AP_CONFIG_COMPLETE);
		}
		save_reset_response(1, fd);
		return;
	}

	if(strstr(pToken1, "reset")) {
		SAC_DBG("save_reset_response reset\n");
		save_reset_response(2, fd);
		return;
	}

	if(strstr(pToken1, "upload") == 0) {
		SAC_DBG("save_reset_response upload\n");
		return;
	}
}

/* Find dst string from src string. return the first place */
static char *memmem(char *src, int src_len, const char *dst, int dst_len)
{
    int i, j;

    for (i=0; i<=src_len-dst_len; i++) {
        if (src[i] == dst[0]) {
            for (j=1; j<dst_len; j++) {
                if (src[i+j] != dst[j])
                    break;
            }
            if (j == dst_len)
                return &src[i];
        }
    }

    return NULL;
}

static int http_recv_requst(int fd)
{
	int length = 0;
	int ret = 0;
	int content_length = 0;
	int pos = 0;

	SAC_DBG("recv request\n");

	memset(http_buffer, 0, HTTP_DATA_MAX_LEN);

	while (1) {
		ret = recv(fd, http_buffer + length, 1, 0);
		if(ret <= 0) {
			if (ret < 0)
				SAC_ERR("recv err! ret:%d errno:%d\n", ret, errno);
			return -1;
		}
		length += ret;
		/* find the header of the response, find the "\r\n" */
		if (length >= 4)
			if (memcmp(http_buffer + length - 4, "\r\n\r\n", 4) == 0)
				break;

		if (length >= HTTP_DATA_MAX_LEN)
			return -1;
	}

	sscanf(strstr(http_buffer, "Content-Length:"), "%*s %d", &content_length);

	SAC_DBG("content length: %d\n", content_length);

	if (content_length >= HTTP_DATA_MAX_LEN - length)
		return -1;

	pos = length;
	length = 0;

	if (content_length > 0) {
		while (1) {
			ret = recv(fd, http_buffer + pos + length, content_length - length, 0);
			if (ret <= 0) {
				if (ret < 0)
					SAC_ERR("recv err! ret:%d errno:%d\n", ret, errno);
				return -1;
			}
			length += ret;
			if(length >= content_length)
				break;
		}
	}
	http_buffer[pos + length] = '\0';

	SAC_DBG("\n%s\n", http_buffer);

	return 0;
}


static void get_update_post(int fd, char *postdata, int len)
{
	static const char endline[] = {'\r', '\n', '\r', '\n'};
	static const char lengthstr[] = "Content-Length: ";
	static const char boundarystr[] = "boundary=";
	char *boundcp = NULL;
	char *boundary, *p;
	char *read_buffer = postdata, *end_pos = NULL, *lengthpos;
	int read_buffer_size = len, time = 5000;
	int bytes_received, read_len, content_len = 0, total_read;
	const char *resp;

	/* Firmware update: OTA from webserver*/
    setsockopt(fd, 0, SO_RCVTIMEO, &time, 4);

	SAC_DBG("[get_update_post]...\n");

    /* Get the content length & boundary & begin of content data */
    do {
        end_pos = (char*)memmem(read_buffer, read_buffer_size, endline, sizeof(endline));

        if ((lengthpos = (char*)memmem(read_buffer,  read_buffer_size, lengthstr, strlen(lengthstr))) != NULL) {
            content_len = atoi(lengthpos + sizeof( lengthstr)-1);
        }
        if ((boundary = (char*)memmem(read_buffer,  read_buffer_size, boundarystr, strlen(boundarystr))) != NULL) {
            boundary += strlen(boundarystr);
            p = boundary;
            while (*p != 0x0d)
                p++;
            *p++ = 0;
            // now, we have found out the boundary, copy out.
            boundcp = (char*)malloc(strlen(boundary) + 1);
            if (boundcp != NULL) {
                strcpy(boundcp, boundary);
            }
        }

        if (end_pos == NULL) {
            read_buffer = http_buffer;
            bytes_received = recv(fd, http_buffer, HTTP_DATA_MAX_LEN, 0 );
            if (bytes_received <= 0) {
                break;
            } else {
                total_read += bytes_received;
                read_buffer_size = bytes_received;
            }
        }
    } while (end_pos == NULL);
    if (boundcp == NULL || content_len == 0) {
        resp = systemResponseError;
        goto EXIT;
    }

    end_pos += sizeof(endline);
    read_buffer_size = read_buffer_size - (end_pos-read_buffer);
    content_len -= read_buffer_size;
    read_buffer = end_pos;
    /* Get the begin of file data and write to flash */
    do {
        end_pos = (char*)memmem(read_buffer, read_buffer_size, endline, sizeof(endline));

        if (end_pos == NULL) {
            read_buffer = http_buffer;
            bytes_received = recv(fd, http_buffer, HTTP_DATA_MAX_LEN, 0);
            if ( bytes_received <= 0 ) {
                break;
            } else {
                content_len -= bytes_received;
                read_buffer_size = bytes_received;
                if (content_len <= 0)
                    break;
            }
        }

    } while (end_pos == NULL);
    if (end_pos == NULL) {
        resp = systemResponseError;
        goto EXIT;
    }

    end_pos += sizeof(endline);
    read_buffer_size = read_buffer_size - (end_pos-read_buffer);
    if (read_buffer_size > 0) {
		//
    }

    content_len -= strlen(boundcp) - 4; // last string is '--'+boudnary+'--'
    /* Recv file and write to flash, if it's last package, find the end of file to write */
    while(content_len > 0) {
        if (content_len > HTTP_DATA_MAX_LEN)
            read_len = HTTP_DATA_MAX_LEN;
        else
            read_len = content_len;

        bytes_received = recv(fd, http_buffer, read_len, 0);
        if ( bytes_received <= 0 ) {
            break;
        }

        content_len -= bytes_received;
    }

    if (content_len == 0) {
      resp = systemResponseSucc;
    } else
      resp = systemResponseError;
EXIT:
    sprintf(http_buffer, HTTPSaveResponse, strlen(resp), resp);
    send_http_data(fd, http_buffer, strlen(http_buffer));
}

static void http_client_handle(int fd ,int data_len)
{
	http_token httpToken ={0,0,0};
	char *p_auth = NULL;

	if (!http_parse(http_buffer, &httpToken))
		goto EXIT;

    p_auth = strstr(httpToken.pToken2, auth_str);
	if (p_auth == NULL) {
		SAC_DBG("un-authrized\n");
        send_http_data(fd, (char*)authrized, strlen(authrized));
        goto EXIT;
	} else {
		SAC_DBG("authrized\n");
    }

	if (!strcmp(httpToken.pToken1, "GET"))
	{
		SAC_DBG("get\n");
		if(!strncmp(httpToken.pToken2, "/system.htm", strlen("/system.htm"))) {
			SAC_DBG("get system.htm\n");
			send_system_page(fd);
		} else if (!strncmp(httpToken.pToken2, "/ ", 2)) {
			SAC_DBG("get /\n");
			send_system_page(fd);
		} else {
			SAC_DBG("get not found\n");
			send_http_data(fd, (char *)not_found, strlen(not_found));
		}
	} else if (!strcmp(httpToken.pToken1, "POST")) {
		SAC_DBG("post\n");
		if (!strncmp(httpToken.pToken2, "/update.htm", strlen("/update.htm")))  {
			SAC_DBG("post update.htm\n");
			get_update_post(fd, httpToken.pToken2, data_len - (httpToken.pToken2 - http_buffer));
		} else if (!strncmp(httpToken.pToken2, "/settings.htm", strlen("/settings.htm"))) {
			SAC_DBG("post settings.htm\n");
			get_settings_post(fd, httpToken.pToken2, data_len - (httpToken.pToken2 - http_buffer));
		}
	}

EXIT:

	return;
}

static int soft_ap_http_websever_task(void)
{
    struct sockaddr_in saddr = { 0 };
    socklen_t socklen = 0;
    int ret = 0;
 	int option;

	xr_client_fd = -1;
	xr_server_fd = -1;

	SAC_DBG("socket begin create...\n");
    if (xr_server_fd < 0) {
        xr_server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (xr_server_fd < 0) {
			SAC_ERR("socket create err!\n");
            goto out;
        }

        saddr.sin_family		= AF_INET;
        saddr.sin_port			= htons(80);
        saddr.sin_addr.s_addr	= htonl(INADDR_ANY);

		option = 1;
		if ((ret = setsockopt(xr_server_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&option, sizeof(option))) < 0) {
			SAC_ERR("failed to setsockopt sock_fd!\n");
			goto out;
		}

		option = 1;
		if ((ret = setsockopt(xr_server_fd, SOL_SOCKET, SO_REUSEPORT, (char *)&option, sizeof(option))) < 0) {
		 	SAC_ERR("failed to setsockopt sock_fd!\n");
			goto out;
		}

        if ((ret = bind(xr_server_fd, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in))) < 0) {
            SAC_ERR( "Failed to bind tcp socket! errno:%d\n", errno);
            goto out;
        }

        if ((ret = listen(xr_server_fd, 8)) != 0) {
            SAC_ERR("listen failed \n");
            goto out;
        }

		/* Set the recv timeout to set the accept timeout */
		option = 100;
		if ((ret = setsockopt(xr_server_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&option, sizeof(option))) < 0) {
			SAC_ERR("set socket option err! errno:%d\n", errno);
			goto out;
		}

		soft_ap_run_state = SOFT_AP_CONFIG_START;

		SAC_DBG("socket listen success!\n");
    }

    while (soft_ap_run_state) {
		int rs;
		fd_set fdr;
		struct timeval tv;

        socklen = sizeof(saddr);
		while (soft_ap_run_state) {
	        xr_client_fd = accept(xr_server_fd, (struct sockaddr *)&saddr, &socklen);
			if (xr_client_fd >= 0) {
				SAC_DBG("socket accept\n");
				option = 1000;
				setsockopt(xr_client_fd, 0, SO_SNDTIMEO, &option, sizeof(option));
				break;
			}
		}

		while (soft_ap_run_state) {
			FD_ZERO(&fdr);
			FD_SET(xr_client_fd, &fdr);

			tv.tv_sec = 0;
			tv.tv_usec = 0;

			rs = select(xr_client_fd + 1, &fdr, NULL, NULL, &tv);
			if (rs > 0) {
				if (FD_ISSET(xr_client_fd, &fdr)) {
					rs = http_recv_requst(xr_client_fd);
					if (rs != 0) {
						closesocket(xr_client_fd);
						xr_client_fd = -1;
						break;
					}
					http_client_handle(xr_client_fd, strlen(http_buffer));
				}
			} else if (rs < 0) {
				SAC_ERR("socket select err! errno:%d\n", errno);
				closesocket(xr_client_fd);
				xr_client_fd = -1;
				break;
			} else if (rs == 0) {
				continue;
			}
		}
    }

out:
	if (xr_server_fd >= 0) {
		closesocket(xr_server_fd);
		xr_server_fd = -1;
	}
	if (xr_client_fd >= 0) {
		closesocket(xr_client_fd);
		xr_client_fd = -1;
	}

	return ret;
}

static void xr_softap_task(void *arg)
{
	soft_ap_http_init();

	soft_ap_http_websever_task();

	OS_ThreadDelete(&soft_ap_thread);
}

int soft_ap_config_start(void)
{
	if (OS_ThreadIsValid(&soft_ap_thread)) {
		SAC_ERR("soft ap config has already start!\n");
		return -1;
	}

	SAC_DBG("soft ap config start!\n");

	if (OS_ThreadCreate(&soft_ap_thread,
						"xr_softap_task",
						xr_softap_task,
						NULL,
						OS_THREAD_PRIO_APP,
						SOFT_AP_CONFIG_THREAD_STACK_SIZE) != OS_OK) {
		SAC_ERR("thread create error\n");
		return -1;
	}
	return 0;
}

int soft_ap_config_stop(void)
{
	soft_ap_run_state = SOFT_AP_CONFIG_STOP;
	soft_ap_http_deinit();

	SAC_DBG("soft ap config stop!\n");

	while (OS_ThreadIsValid(&soft_ap_thread))
		OS_MSleep(10);

	return 0;
}

int soft_ap_config_get_state(void)
{
	return soft_ap_run_state;
}

int soft_ap_config_set_cb(soft_ap_config_cb cb)
{
	soft_ap_callback = cb;
	return 0;
}

int soft_ap_config_get_result(soft_ap_config_result *result)
{
	if (soft_ap_run_state != SOFT_AP_CONFIG_COMPLETE)
		return -1;

	memcpy(result, &soft_ap_result, sizeof(soft_ap_config_result));

	return 0;
}

