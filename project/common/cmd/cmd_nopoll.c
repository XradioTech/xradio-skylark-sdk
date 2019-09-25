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
#include "net/nopoll/nopoll.h"

static const char websockets_ca_cert[] =
"-----BEGIN CERTIFICATE-----\r\n"
"MIIC6zCCAdMCFELncsTSBAsyNRBc/4V2/QXlLdQ8MA0GCSqGSIb3DQEBBQUAMDIx\r\n"
"MDAuBgNVBAoMJ1RMUyBQcm9qZWN0IERvZGd5IENlcnRpZmljYXRlIEF1dGhvcml0\r\n"
"eTAeFw0xOTA3MTUwNjMzMzhaFw0zMzAzMjMwNjMzMzhaMDIxMDAuBgNVBAoMJ1RM\r\n"
"UyBQcm9qZWN0IERvZGd5IENlcnRpZmljYXRlIEF1dGhvcml0eTCCASIwDQYJKoZI\r\n"
"hvcNAQEBBQADggEPADCCAQoCggEBAKXfNjrZaQgbQgHnf+8fnkVou9g6iRofZcjy\r\n"
"HT97qUQrInSUo062+2nqFyfhsrDJU5oGHRUPaCXS9SyEregqa+6XqbMRLvL9Qf/8\r\n"
"NO0rsroqz+fQIIW8i6kxE/9s9/4csJDk7WPxV/ncG2bcHoYf9wiN6AR6hA+h4bZj\r\n"
"Cjt4qGQ23BhwIOLWrIBHq9yTfgQ2Nys01m35FvIjzSwzjQXaEcMJNy20fcmhvAr9\r\n"
"CXhPCm8np7zhbgs6SS7jUpg+9tmLypJ6JUB4GeSIAk62VlixJe+9ku4SxP9beO1/\r\n"
"YpxbcKE9Ubl01z1A6KDOzox7XReqv9iAgu03x+nDv5j9p+0SEi8CAwEAATANBgkq\r\n"
"hkiG9w0BAQUFAAOCAQEAoBYa7wGU5buBQbXRW7HYsu47mOAd2gd/ezMunV+L/ENQ\r\n"
"9IAfFJy9p816fpHShoQuQsf7RV6R/UTu/2c272/clfqanyEDFL1MupvQ7SDY+lyK\r\n"
"ofwz1H7ajWVeGBtPKtFc5btsgW6xtQIwOlHKRNI8BPP5qOrsvcwfipJ4jYlEBSGS\r\n"
"l+tIr0xpuBO0hEcG97jdoe3KpFZ0APW2C4xh8ctW1CArRVnezdCULnWFdKpdhQAY\r\n"
"ssuB2CL76qRa8l6ZZj5gUEa3IgRCj3UisBvNCMV63EhfJd/0mLv1OWqx9GPwQgMa\r\n"
"A3VSOLvHqch6Da88e6wE5irwZoCZLrUMCuS5mrOQTQ==\r\n"
"-----END CERTIFICATE-----\r\n";

/* use for two-phase verification */
static const char websockets_client_cert[] =
"-----BEGIN CERTIFICATE-----\r\n"
"MIIC/DCCAeQCFFk4HVa42niSzTZbbPwh49UZW3rHMA0GCSqGSIb3DQEBBQUAMDIx\r\n"
"MDAuBgNVBAoMJ1RMUyBQcm9qZWN0IERvZGd5IENlcnRpZmljYXRlIEF1dGhvcml0\r\n"
"eTAeFw0xOTA3MTUwNjMzMzhaFw0zMzAzMjMwNjMzMzhaMEMxJzAlBgNVBAoMHlRM\r\n"
"UyBQcm9qZWN0IERldmljZSBDZXJ0aWZpY2F0ZTEYMBYGA1UEAwwPMTkyLjE2OC4x\r\n"
"MTEuMTAxMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAxY05JdapYr6B\r\n"
"ohID4gFxh5ANvqGtTqTdfmeZSQIPzA3b6kTHF+HbMw0FGaKCd2Minah4IlWHq5mE\r\n"
"9jpggj3UxqEcdmCDzBV2bbj+ZA8XdtB7oqeb+6Bt+Gqxi83RDEQpDI/nwwWrGahu\r\n"
"6YXF+PXlJQC3YasW/JrQDJY9VMf1nyGf3FN5TJQ4HJ6eHWsXhwNUCSwE6e4ol+V6\r\n"
"GO2coFukOmQDQ2KgK7MYMKF9YTbusvaSIx1AbmJCfszoudUolbnlVIYIh98g2z7X\r\n"
"p1ki0Vtr0UnrK3qBYVlPXWq+d4hbtgL9rNoR4mSrp9k4QvdChwkZ4szblwkMZ7Xf\r\n"
"wB2qBYO2LwIDAQABMA0GCSqGSIb3DQEBBQUAA4IBAQAl0ui4I0cCtZ9q1el7ACS4\r\n"
"y9ph8aD0WsosQlHl6sjIQOMIU0dH9xSyELaST7UzGauOXx4sWnxWb6NhtB1huC0l\r\n"
"eRZgHqnHMqioxDTb2LPB1rA/B4ntzVkOBzoi44xuL3LFhQap8UMm+1oYJi+WSGsF\r\n"
"3qwIkju1VmL6b85P6xKBoC8N9jLKGVVUFBZ0gCUT+vCsb8fcoiQWqAGXkrz1RWEp\r\n"
"9CEFH1AQYHiRlZOb0lIPizI5Zo29GsPWbj0yai4/vI+q7uYs67Q3Yjd6V8tSu8AC\r\n"
"npskXlMqLl76F1SkD4SEBj57eB40Llm7WSfXPx182naZdHzsKpFzDkjod6MlCWCS\r\n"
"-----END CERTIFICATE-----\r\n";

/* use for two-phase verification */
static const char websockets_client_key[] =
"-----BEGIN RSA PRIVATE KEY-----\r\n"
"MIIEpAIBAAKCAQEAxY05JdapYr6BohID4gFxh5ANvqGtTqTdfmeZSQIPzA3b6kTH\r\n"
"F+HbMw0FGaKCd2Minah4IlWHq5mE9jpggj3UxqEcdmCDzBV2bbj+ZA8XdtB7oqeb\r\n"
"+6Bt+Gqxi83RDEQpDI/nwwWrGahu6YXF+PXlJQC3YasW/JrQDJY9VMf1nyGf3FN5\r\n"
"TJQ4HJ6eHWsXhwNUCSwE6e4ol+V6GO2coFukOmQDQ2KgK7MYMKF9YTbusvaSIx1A\r\n"
"bmJCfszoudUolbnlVIYIh98g2z7Xp1ki0Vtr0UnrK3qBYVlPXWq+d4hbtgL9rNoR\r\n"
"4mSrp9k4QvdChwkZ4szblwkMZ7XfwB2qBYO2LwIDAQABAoIBAQCtbkjwdh7oqHTo\r\n"
"EsbD4B6KM6ZNaGTcuRIWyd6hYKT7sGMTrOPYjJjCnbiPg8LkCu012dP12H6t4K4A\r\n"
"+MkHLj8hTgnNxveN8H2y4Ai9UR55WZhg+KKQ6owA3bIXGU5gZWpgM+n0pYJLmTod\r\n"
"2yotYbqUnKdhoDEi/MqPckpPpuh0lSVXOGBf88Cl1A3WGqhPn/fvMAGAMEuayCU9\r\n"
"NxsK9K05OBox9+CuJX9Gfhx0vcC4cB0HQfLzpAOe5j6W/FhYb39sZCalNSx/9sKt\r\n"
"uxTGiiQrmPJ9mGtY6x0ksPvmZGIKmh6YdNIZ+MQlE5iDf1mYr4SuzuLBpuU6rDEI\r\n"
"1zKV6Ih5AoGBAPL5cNV9QZNSGg/bS5hBzKnyOdubSg0nkFXPA2lqfz2uwQ9c0DYl\r\n"
"5TF2JExGy7YpUWiJCuXjmcNO6ck5QTo8K3M17zikcthOxEtM0cZDHnSWcWLz+EiO\r\n"
"XiK9VMyJZqb2ZivEHPyzjxffE6rvPVN/vYMaKt5Mdw6Jzh8cGPzZjIqFAoGBANAk\r\n"
"Z6rXzrEwbNv9YGiXFj9gnbLTcfVLDlropL9Hf/dxWP8PQxBeyvZ3XfvFxpWI/MU5\r\n"
"hdj3c+AvkB//5AUcHU0K5vE8Sv+M2e7bsFxyMYuVybXlpvqwt9zOpzw5x7xlreRE\r\n"
"/xVpKbUO3YKCjrqeOsJtVOeEhYwK4hAcygD/y44jAoGAAKYX3goSlcEfXrF4NzTd\r\n"
"xgpmiyaUAQr9AK2n1a06H8EKtO7Lg4mAXixxll6OBrN/iybqh4ifDX11dFsZyH0G\r\n"
"pK0dMWqG//rd3VGcMcpWF3ubW+dI33C54Z/dzRoE0ydPSIiihy43kJnA5LD02fc6\r\n"
"W1JDkQplOv21NjIOAwbVsD0CgYBeG0efz8kNBecCI/I197HAX++NDdrlW9UWtz7d\r\n"
"mPc7qkzhrUXWHfXIL7oXfplFvNUEWviwW1lR0E9qmGjBArAgyEAYa/lAx681NrDr\r\n"
"a1oJUWUMz9OKXuISfIDSUxGClbpkjemDBbQsv5bZTiw4JhhNFd+geaNj6PvC6zFN\r\n"
"+FlRXwKBgQDBWxw6FPL/q+AP+B+RNde9pueVHQFx0hZM57K7jjddoVOTo1f98TM/\r\n"
"51b2QXR65Rl08c4tkvDqtf3jT3LeRsvgazfXNyDTp0C2BQ+ocXhyvQ2Har8z+ujs\r\n"
"0h9L+vHw1lUzfFrda7igD2XZDvJEgOlTQ4FQRcTxRSu7A5QSi8WQOg==\r\n"
"-----END RSA PRIVATE KEY-----\r\n";

static const char websockets_server_cert[] =
"-----BEGIN CERTIFICATE-----\r\n"
"MIIC6DCCAdACFFk4HVa42niSzTZbbPwh49UZW3rGMA0GCSqGSIb3DQEBBQUAMDIx\r\n"
"MDAuBgNVBAoMJ1RMUyBQcm9qZWN0IERvZGd5IENlcnRpZmljYXRlIEF1dGhvcml0\r\n"
"eTAeFw0xOTA3MTUwNjMzMzhaFw0zMzAzMjMwNjMzMzhaMC8xFDASBgNVBAoMC1RM\r\n"
"UyBQcm9qZWN0MRcwFQYDVQQDDA4xOTIuMTY4LjUxLjEwMDCCASIwDQYJKoZIhvcN\r\n"
"AQEBBQADggEPADCCAQoCggEBANdysGO5RWrQnzmBQbA7GfopxWXrdtNGMqOp4410\r\n"
"jGVyAd8r5Md8JVK2uUBAZrOEiiFj8LcSaR1T5oPnE5pOEWCxD3wziGNJCjEAeeQx\r\n"
"MASL5Ua2YUk5164qiUqOJcNIJHMXfQ+5BDCIfDCRAqAdJBHFcw6r0c+HKd9FTWMU\r\n"
"JyFHrAemcfCJFbZqPSFej/oYesqV06bP1ox30X31qM1hLn0olUnIEkFAU2p4jL7u\r\n"
"rJfWT3GgEpwjp0FXq8ByMg7ib/zzh8A/JruboHk4wj5uFhbsdDluCXD25AjCtJDI\r\n"
"N/DnnmHMFcvM/KVP0t98iNkOPy50vrErMEKTefA/WqGYpecCAwEAATANBgkqhkiG\r\n"
"9w0BAQUFAAOCAQEAJiGCUmrBTc0GnxIM3VmZEKK64uUHyhndLE1lSDO/gAi6S7Di\r\n"
"5Sh2Q7OQ9d8h/vV8B31BeYZKA0tLSRYWznbYtvGVszVnCJpQ1rBnq+Wk670F/4iM\r\n"
"Nihae7FHHAZ3tHUhL5NPMjK/m+0fp19KvNYIEBecm/E7Am06nCzKV7rDgLPMmWNC\r\n"
"CW07/ViXCQSYUOQoSijaxcjxzD+mCCf38vD0qQ3XR2un/+Ny9tZwzqj6VcaDSvy+\r\n"
"JVaFJFwMIe0P+hHbOTSrpaJ8C0H+Tem72UNRImwe5KX6SP+VFlOQV31HFj3G5rKb\r\n"
"zT81oxV8CdSZSxbsWWXDtiaCEGr6ioq78t7Kqw==\r\n"
"-----END CERTIFICATE-----\r\n";

static const char websockets_server_key[] =
"-----BEGIN RSA PRIVATE KEY-----\r\n"
"MIIEpAIBAAKCAQEA13KwY7lFatCfOYFBsDsZ+inFZet200Yyo6njjXSMZXIB3yvk\r\n"
"x3wlUra5QEBms4SKIWPwtxJpHVPmg+cTmk4RYLEPfDOIY0kKMQB55DEwBIvlRrZh\r\n"
"STnXriqJSo4lw0gkcxd9D7kEMIh8MJECoB0kEcVzDqvRz4cp30VNYxQnIUesB6Zx\r\n"
"8IkVtmo9IV6P+hh6ypXTps/WjHfRffWozWEufSiVScgSQUBTaniMvu6sl9ZPcaAS\r\n"
"nCOnQVerwHIyDuJv/POHwD8mu5ugeTjCPm4WFux0OW4JcPbkCMK0kMg38OeeYcwV\r\n"
"y8z8pU/S33yI2Q4/LnS+sSswQpN58D9aoZil5wIDAQABAoIBAFOr6u0PyvHMy8md\r\n"
"dVFn4pLRHiSS6bbrkEcXd5Q8KzpKqIdvmI4QnL4e0JsvZ7NYSfzlv1qZ/9CwANpB\r\n"
"J8/Jed4/ZXAq41EL0LVvJeeFYsyhOA3aTrYNjMHLKz6VLf1FS0KXYK52gDYJC5Ig\r\n"
"/xAHwTZVpHWSLQ2XxJ89XuyXqwq1AKJ3dLK8RdDAeIbaZ50cR4OfLUU04uuBJ6zC\r\n"
"QKpfiDjf0BM4VGLJZP1uMTvfHFZhUU8b/FQjaHZ7My1EBSffpCnVy0edJnkxKn3E\r\n"
"xs++0nb7xjDhUrp9CkTTfMxrvRmUWSni09P1SCD26TUnBXqjQ9QX3DewAGKLOJaT\r\n"
"JbeMDOECgYEA7MPT6d6f2iBOr3qdx9VREQsbljXtGSCxlwWnfCS5nrC93csXZ95e\r\n"
"5NDd4S9hf328FTYszT7Z+HjJvL8az1owT8IFhLDwa62N1c0rlcwuLNGkzX5WsxgF\r\n"
"5Hzn38u3nUzOVV7ltFeVeMrVobrXf8p5IegAiARjzi/JWnyHlSRWxlsCgYEA6POE\r\n"
"aIFtEcY+OJD1LhkShWOE2jk46rgN+KufbhIaKIRwvftoNev5sczeZSII7fZHXx8I\r\n"
"dYWF56c+x4hKH+u52aW6Qvmod/aur11ncyE0bDWlqb3fQcpcAmJRLdh95pY+g1N8\r\n"
"ZAqDwUQY99nuMSb0uPELFSpsS7f9EOj1UwpqbGUCgYAFsnyh7wp3rDlYfaHYUii+\r\n"
"OT8zsR6AcUn0sV0jXprc17Hp6V4p0K8F4ITGS2aUM8lX1VLkqXODSBzKnuqdPLVW\r\n"
"5ftOAxf171ovX92BoEUoLO9DRpv9eUGDCwJlXziO329DnKH4YfclzfOwDZfr6kpZ\r\n"
"54RDwZ9JMdstgzub+iDgawKBgQCusWfbYyvjB3LDwtlK75x3EZGggQcQ5mP71uBh\r\n"
"kAuz1NYjcrTMsyD2WDdzShm+MlGFoOUcFLy9YytuQOOtmn4uHN1Yupa/F+waMIaS\r\n"
"zRTeDUEl0PDrCpEDbK2KHu8TjBpnK6V99HCn76R1wiWhEKC3THRONBkwk1KhasIG\r\n"
"uyniDQKBgQDFL9bjpQFUoJLn07Cs5gkdDSupScohz1RNjTGKq+6cT7ARL3yh+kyU\r\n"
"lE6MWCw9iONopE6I+hShP2+s+t6DqyTgnrVK1l7EWyE59Ei4g9QLYMLcOaKayyum\r\n"
"3zcjhbjyyfgmnsDSi9hFpjIaNw9pZKmyLrSHnsQzd1HEqeGuaRJ8KQ==\r\n"
"-----END RSA PRIVATE KEY-----\r\n";

#define CMD_NOPOLL_MSG_EXIT		(0)
#define CMD_NOPOLL_MSG_OPEN		(1)
#define CMD_NOPOLL_MSG_SEND		(2)
#define CMD_NOPOLL_MSG_RECV		(3)
#define CMD_NOPOLL_MSG_CLOSE	(4)

struct cmd_nopoll_open_data {
	uint32_t tls;
	uint32_t verify;
	uint32_t cer;
};

struct cmd_nopoll_send_data {
	uint8_t *buf;
	uint32_t size;
};

struct cmd_nopoll_msg {
	uint32_t	type;
	void	   *data;
};

#define CMD_NOPOLL_QUEUE_WAIT_TIME (5000)
#define CMD_NOPOLL_THREAD_STACK_SIZE (4 * 1024)
static OS_Thread_t g_nopoll_server_thread;
static OS_Thread_t g_nopoll_client_thread;

static char g_server_host[32] = {0};
static char g_server_port[8] = {0};
static noPollCtx *g_server_ctx = NULL;
static noPollConn *g_listener = NULL;
static noPollConnOpts *g_listener_opts = NULL;

static OS_Queue_t g_client_queue;
static char g_client_host[32] = {0};
static char g_client_port[8] = {0};
static noPollCtx *g_client_ctx = NULL;
static noPollConn *g_conn = NULL;
static noPollConnOpts *g_conn_opts = NULL;

static int ag_nopoll_complete_pending_write(noPollConn *conn)
{
	int tries = 0;
	while (tries < 5 && errno == NOPOLL_EWOULDBLOCK && nopoll_conn_pending_write_bytes (conn) > 0) {
		nopoll_sleep(50000);
		if (nopoll_conn_complete_pending_write (conn) == 0)
			return 0;
		tries++;
	}
	return 1;
}

static void cmd_nopoll_server_task(void *arg)
{
	nopoll_loop_wait(g_server_ctx, 0);

	CMD_DBG("%s() end\n", __func__);
	OS_ThreadDelete(&g_nopoll_server_thread);
}

static void cmd_nopoll_server_on_close(noPollCtx *ctx, noPollConn *conn, noPollPtr user_data)
{
	CMD_LOG(1, "called connection close\n");
}

static nopoll_bool cmd_nopoll_server_on_open(noPollCtx *ctx, noPollConn *conn, noPollPtr user_data)
{
	CMD_LOG(1, "called connection open\n");

	nopoll_conn_set_on_close(conn, cmd_nopoll_server_on_close, NULL);

	if (!nopoll_conn_set_sock_block(nopoll_conn_socket(conn), nopoll_false)) {
		CMD_ERR("ERROR: failed to configure non-blocking state to connection..\n");
		return nopoll_false;
	}

	return nopoll_true;
}

static void cmd_nopoll_server_on_message(noPollCtx *ctx, noPollConn *conn, noPollMsg *msg, noPollPtr user_data)
{
	int ret;
	int size = nopoll_msg_get_payload_size(msg);
	const char *content = (const char *)nopoll_msg_get_payload(msg);

	CMD_LOG(1, "server receive message:\n");
	CMD_LOG(1, "%s\n", content);

	ret = nopoll_conn_send_text(conn, content, size);
	if (ret != size) {
		if (ag_nopoll_complete_pending_write(conn))
			CMD_ERR("size = %u, but nopoll_conn_send_text ret = %d\n", size, ret);
	}

	if (nopoll_cmp(content, "nopoll test stop running")) {
		nopoll_loop_stop(g_server_ctx);
	}
}

static enum cmd_status cmd_nopoll_server_init_exec(char *cmd)
{
	int cnt;
	uint32_t dbg;

	cnt = cmd_sscanf(cmd, "d=%u", &dbg);
	if (cnt != 1) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (g_server_ctx != NULL) {
		CMD_ERR("already init\n");
		return CMD_STATUS_FAIL;
	}

	g_server_ctx = nopoll_ctx_new();
	if (g_server_ctx == NULL) {
		CMD_ERR("nopoll_ctx_new failed\n");
		return CMD_STATUS_FAIL;
	}

	if (dbg == 0)
		nopoll_log_enable(g_server_ctx, nopoll_false);
	else
		nopoll_log_enable(g_server_ctx, nopoll_true);

	/* set on open */
	nopoll_ctx_set_on_open(g_server_ctx, cmd_nopoll_server_on_open, NULL);

	/* set on message received */
	nopoll_ctx_set_on_msg(g_server_ctx, cmd_nopoll_server_on_message, NULL);

	OS_ThreadSetInvalid(&g_nopoll_server_thread);

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_nopoll_server_open_exec(char *cmd)
{
	int cnt;
	uint32_t tls;
	uint32_t verify;

	cnt = cmd_sscanf(cmd, "h=%s p=%s t=%u v=%u", g_server_host, g_server_port, &tls, &verify);
	if (cnt != 4) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (g_server_ctx == NULL) {
		CMD_ERR("without init\n");
		return CMD_STATUS_FAIL;
	}

	if ((g_listener != NULL) || (g_listener_opts != NULL)) {
		CMD_ERR("already open\n");
		return CMD_STATUS_FAIL;
	}

	g_listener_opts = nopoll_conn_opts_new();
	if (g_listener_opts == NULL) {
		CMD_ERR("nopoll_conn_opts_new failed\n");
		return CMD_STATUS_FAIL;
	}

	if (tls == 0)
		g_listener = nopoll_listener_new_opts(g_server_ctx, g_listener_opts, g_server_host, g_server_port);
	else {
		if (!nopoll_conn_opts_set_ssl_certs(g_listener_opts,
											websockets_server_cert, sizeof(websockets_server_cert),
											websockets_server_key, sizeof(websockets_server_key),
											websockets_ca_cert, sizeof(websockets_ca_cert),
											websockets_ca_cert, sizeof(websockets_ca_cert))) {
			CMD_ERR("nopoll_conn_opts_set_ssl_certs failed\n");
			nopoll_conn_opts_free(g_listener_opts);
			g_listener_opts = NULL;
			return CMD_STATUS_FAIL;
		}

		if (verify == 0)
			nopoll_conn_opts_ssl_peer_verify (g_listener_opts, nopoll_false);
		else
			nopoll_conn_opts_ssl_peer_verify (g_listener_opts, nopoll_true);

		g_listener = nopoll_listener_tls_new_opts(g_server_ctx, g_listener_opts, g_server_host, g_server_port);
	}

	if (g_listener == NULL) {
		CMD_ERR("nopoll_listener(_tls)_new_opts failed\n");
		nopoll_conn_opts_free(g_listener_opts);
		g_listener_opts = NULL;
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_nopoll_server_start_exec(char *cmd)
{
	if ((g_listener == NULL) || (g_listener_opts == NULL)) {
		CMD_ERR("without open\n");
		return CMD_STATUS_FAIL;
	}

	if (OS_ThreadIsValid(&g_nopoll_server_thread)) {
		CMD_ERR("already start\n");
		return CMD_STATUS_FAIL;
	}

	if (OS_ThreadCreate(&g_nopoll_server_thread,
						"cmd_nopoll_serv",
						cmd_nopoll_server_task,
						NULL,
						OS_THREAD_PRIO_CONSOLE,
						CMD_NOPOLL_THREAD_STACK_SIZE) != OS_OK) {
		CMD_ERR("thread create failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_nopoll_server_stop_exec(char *cmd)
{
	if (!OS_ThreadIsValid(&g_nopoll_server_thread)) {
		CMD_ERR("without start\n");
		return CMD_STATUS_FAIL;
	}

	nopoll_loop_stop(g_server_ctx);

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_nopoll_server_close_exec(char *cmd)
{
	if ((g_listener == NULL) || (g_listener_opts == NULL)) {
		CMD_ERR("without open\n");
		return CMD_STATUS_FAIL;
	}

	if (OS_ThreadIsValid(&g_nopoll_server_thread)) {
		CMD_ERR("without stop\n");
		return CMD_STATUS_FAIL;
	}

	nopoll_conn_close(g_listener);
	g_listener = NULL;
	g_listener_opts = NULL;

	cmd_memset(g_server_host, 0, sizeof(g_server_host));
	cmd_memset(g_server_port, 0, sizeof(g_server_port));

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_nopoll_server_deinit_exec(char *cmd)
{
	if (g_server_ctx == NULL) {
		CMD_ERR("without init\n");
		return CMD_STATUS_FAIL;
	}

	nopoll_ctx_unref(g_server_ctx);
	nopoll_cleanup_library();
	g_server_ctx = NULL;

	return CMD_STATUS_OK;
}

static void cmd_nopoll_client_open_task(struct cmd_nopoll_open_data *data)
{
	g_conn_opts = nopoll_conn_opts_new();
	if (g_conn_opts == NULL) {
		CMD_ERR("nopoll_conn_opts_new failed\n");
		goto open_task_exit;
	}

	if (data->tls == 0)
		g_conn = nopoll_conn_new_opts(g_client_ctx, g_conn_opts, g_client_host, g_client_port, NULL, NULL, NULL, NULL);
	else {
		if (data->cer != 0) {
			if (!nopoll_conn_opts_set_ssl_certs(g_conn_opts,
												websockets_client_cert, sizeof(websockets_client_cert),
												websockets_client_key, sizeof(websockets_client_key),
												NULL, 0,
												websockets_ca_cert, sizeof(websockets_ca_cert))) {
				CMD_ERR("nopoll_conn_opts_set_ssl_certs failed\n");
				nopoll_conn_opts_free(g_conn_opts);
				g_conn_opts = NULL;
				goto open_task_exit;
			}
		}

		if (data->verify == 0)
			nopoll_conn_opts_ssl_peer_verify (g_conn_opts, nopoll_false);
		else
			nopoll_conn_opts_ssl_peer_verify (g_conn_opts, nopoll_true);

		g_conn = nopoll_conn_tls_new(g_client_ctx, g_conn_opts, g_client_host, g_client_port, NULL, NULL, NULL, NULL);
	}

	if (g_conn == NULL) {
		CMD_ERR("nopoll_conn(_tls)_new(_opts) failed\n");
		goto open_task_exit;
	}

	if (nopoll_conn_wait_until_connection_ready(g_conn, 10))
		CMD_LOG(1, "the connection is ready\n");
	else
		CMD_ERR("connection timeout\n");

open_task_exit:
	cmd_free(data);
	data = NULL;
	return;
}

static void cmd_nopoll_client_send_task(struct cmd_nopoll_send_data *data)
{
	int ret = nopoll_conn_send_text(g_conn, (char *)data->buf, data->size);
	if (ret != (int)data->size) {
		if (ag_nopoll_complete_pending_write(g_conn))
			CMD_ERR("size = %u, but nopoll_conn_send_text ret = %d\n", data->size, ret);
	}

	cmd_free(data->buf);
	data->buf = NULL;
	cmd_free(data);
	data = NULL;
	return;
}

static void cmd_nopoll_client_recv_task(void *data)
{
	uint32_t msec = 10 * 1000;
	noPollMsg *msg;
	const char *content;

	while (1) {
		if (!nopoll_conn_is_ok(g_conn)) {
			CMD_DBG("received websocket connection close\n");
			break;
		}
		msg = nopoll_conn_get_msg(g_conn);
		if (msg) {
			content = (const char *)nopoll_msg_get_payload(msg);
			CMD_LOG(1, "receive message:\n");
			CMD_LOG(1, "%s\n", content);

			nopoll_msg_unref (msg);
		} else {
			if (msec == 0) {
				CMD_ERR("get message timeout\n");
				break;
			}

			nopoll_sleep(100000);
			msec -= 100;
		}
	}

}

static void cmd_nopoll_client_close_task(void *data)
{
	nopoll_conn_close(g_conn);
	g_conn = NULL;
	g_conn_opts = NULL;

	cmd_memset(g_client_host, 0, sizeof(g_client_host));
	cmd_memset(g_client_port, 0, sizeof(g_client_port));

	return;
}

static void cmd_nopoll_client_task(void *arg)
{
	uint8_t task_exit = 0;
	struct cmd_nopoll_msg *msg;

	while (task_exit == 0) {

		if (OS_MsgQueueReceive(&g_client_queue, (void **)&msg, OS_WAIT_FOREVER) != OS_OK) {
			CMD_ERR("msg queue receive failed\n");
			break;
		}

//		CMD_DBG("recv msg type %u\n", msg->type);

		switch (msg->type) {
		case CMD_NOPOLL_MSG_EXIT:
			task_exit = 1;
			break;
		case CMD_NOPOLL_MSG_OPEN:
			cmd_nopoll_client_open_task(msg->data);
			break;
		case CMD_NOPOLL_MSG_SEND:
			cmd_nopoll_client_send_task(msg->data);
			break;
		case CMD_NOPOLL_MSG_RECV:
			cmd_nopoll_client_recv_task(msg->data);
			break;
		case CMD_NOPOLL_MSG_CLOSE:
			cmd_nopoll_client_close_task(msg->data);
			break;
		default:
			CMD_WRN("unknown msg\n");
			break;
		}

		cmd_free(msg);
		msg = NULL;
	}

	CMD_DBG("%s() end\n", __func__);
	OS_ThreadDelete(&g_nopoll_client_thread);
}

static enum cmd_status cmd_nopoll_client_init_exec(char *cmd)
{
	int cnt;
	uint32_t dbg;

	cnt = cmd_sscanf(cmd, "d=%u", &dbg);
	if (cnt != 1) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (g_client_ctx != NULL) {
		CMD_ERR("already init\n");
		return CMD_STATUS_FAIL;
	}

	g_client_ctx = nopoll_ctx_new();
	if (g_client_ctx == NULL) {
		CMD_ERR("nopoll_ctx_new failed\n");
		return CMD_STATUS_FAIL;
	}

	if (dbg == 0)
		nopoll_log_enable(g_client_ctx, nopoll_false);
	else
		nopoll_log_enable(g_client_ctx, nopoll_true);

	OS_ThreadSetInvalid(&g_nopoll_client_thread);

	if (OS_MsgQueueCreate(&g_client_queue, 1) != OS_OK) {
		CMD_ERR("msg queue create failed\n");
		return CMD_STATUS_FAIL;
	}

	if (OS_ThreadCreate(&g_nopoll_client_thread,
						"cmd_nopoll_cli",
						cmd_nopoll_client_task,
						NULL,
						OS_THREAD_PRIO_CONSOLE,
						CMD_NOPOLL_THREAD_STACK_SIZE) != OS_OK) {
		CMD_ERR("thread create failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_nopoll_client_open_exec(char *cmd)
{
	int cnt;
	uint32_t tls;
	uint32_t verify;
	uint32_t cer;
	struct cmd_nopoll_open_data *data;
	struct cmd_nopoll_msg *msg;

	cnt = cmd_sscanf(cmd, "h=%s p=%s t=%u v=%u c=%u", g_client_host, g_client_port, &tls, &verify, &cer);
	if (cnt != 5) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (g_client_ctx == NULL) {
		CMD_ERR("without init\n");
		return CMD_STATUS_FAIL;
	}

	if ((g_conn != NULL) || (g_conn_opts != NULL)) {
		CMD_ERR("already open\n");
		return CMD_STATUS_FAIL;
	}

	msg = cmd_malloc(sizeof(struct cmd_nopoll_msg));
	data = cmd_malloc(sizeof(struct cmd_nopoll_open_data));
	if ((msg == NULL) || (data == NULL)) {
		CMD_ERR("malloc failed\n");
		if (msg)
			cmd_free(msg);
		if (data)
			cmd_free(data);
		return CMD_STATUS_FAIL;
	}

	data->tls = tls;
	data->verify = verify;
	data->cer = cer;

	msg->type = CMD_NOPOLL_MSG_OPEN;
	msg->data = data;

	if (OS_MsgQueueSend(&g_client_queue, msg, CMD_NOPOLL_QUEUE_WAIT_TIME) != OS_OK) {
		CMD_ERR("msg queue send failed\n");
		if (msg)
			cmd_free(msg);
		if (data)
			cmd_free(data);
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_nopoll_client_send_exec(char *cmd)
{
	int cnt;
	int ret;
	uint32_t size;
	uint8_t *buf;
	struct cmd_nopoll_send_data *data;
	struct cmd_nopoll_msg *msg;

	cnt = cmd_sscanf(cmd, "s=%u", &size);
	if (cnt != 1) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (size == 0) {
		CMD_ERR("size = 0\n");
		return CMD_STATUS_INVALID_ARG;
	}

	if ((g_conn == NULL) || (g_conn_opts == NULL)) {
		CMD_ERR("without open\n");
		return CMD_STATUS_FAIL;
	}

	msg = cmd_malloc(sizeof(struct cmd_nopoll_msg));
	data = cmd_malloc(sizeof(struct cmd_nopoll_send_data));
	buf = cmd_malloc(size);
	if ((msg == NULL) || (data == NULL) || (buf == NULL)) {
		CMD_ERR("malloc failed\n");
		if (msg)
			cmd_free(msg);
		if (data)
			cmd_free(data);
		if (buf)
			cmd_free(buf);
		return CMD_STATUS_FAIL;
	}

	cmd_write_respond(CMD_STATUS_OK, "OK");

	cmd_raw_mode_enable();
	ret = cmd_raw_mode_read(buf, size, 10000);
	if (ret != (int)size) {
		CMD_ERR("size = %u, but cmd_raw_mode_read ret = %d\n", size, ret);
		cmd_free(msg);
		cmd_free(data);
		cmd_free(buf);
		cmd_raw_mode_write((uint8_t *)"FAIL\n", 5);
		cmd_raw_mode_disable();
		return CMD_STATUS_ACKED;
	}
	cmd_raw_mode_disable();

	data->buf = buf;
	data->size = size;

	msg->type = CMD_NOPOLL_MSG_SEND;
	msg->data = data;

	if (OS_MsgQueueSend(&g_client_queue, msg, CMD_NOPOLL_QUEUE_WAIT_TIME) != OS_OK) {
		CMD_ERR("msg queue send failed\n");
		if (msg)
			cmd_free(msg);
		if (data)
			cmd_free(data);
		if (buf)
			cmd_free(buf);
		return CMD_STATUS_ACKED;
	}

	return CMD_STATUS_ACKED;
}

static enum cmd_status cmd_nopoll_client_recv_exec(char *cmd)
{
	struct cmd_nopoll_msg *msg;

	if ((g_conn == NULL) || (g_conn_opts == NULL)) {
		CMD_ERR("without open\n");
		return CMD_STATUS_FAIL;
	}

	msg = cmd_malloc(sizeof(struct cmd_nopoll_msg));
	if (msg == NULL) {
		CMD_ERR("msg queue send failed\n");
		return CMD_STATUS_FAIL;
	}

	msg->type = CMD_NOPOLL_MSG_RECV;
	msg->data = NULL;

	if (OS_MsgQueueSend(&g_client_queue, msg, CMD_NOPOLL_QUEUE_WAIT_TIME) != OS_OK) {
		CMD_ERR("msg queue send failed\n");
		if (msg)
			cmd_free(msg);
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_nopoll_client_close_exec(char *cmd)
{
	struct cmd_nopoll_msg *msg;

	if ((g_conn == NULL) || (g_conn_opts == NULL)) {
		CMD_ERR("without open\n");
		return CMD_STATUS_FAIL;
	}

	msg = cmd_malloc(sizeof(struct cmd_nopoll_msg));
	if (msg == NULL) {
		CMD_ERR("msg queue send failed\n");
		return CMD_STATUS_FAIL;
	}

	msg->type = CMD_NOPOLL_MSG_CLOSE;
	msg->data = NULL;

	if (OS_MsgQueueSend(&g_client_queue, msg, CMD_NOPOLL_QUEUE_WAIT_TIME) != OS_OK) {
		CMD_ERR("msg queue send failed\n");
		if (msg)
			cmd_free(msg);
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_nopoll_client_deinit_exec(char *cmd)
{
	struct cmd_nopoll_msg *msg;

	if (g_client_ctx == NULL) {
		CMD_ERR("without init\n");
		return CMD_STATUS_FAIL;
	}

	nopoll_ctx_unref(g_client_ctx);
	g_client_ctx = NULL;

	msg = cmd_malloc(sizeof(struct cmd_nopoll_msg));
	if (msg == NULL) {
		CMD_ERR("msg queue send failed\n");
		return CMD_STATUS_FAIL;
	}

	msg->type = CMD_NOPOLL_MSG_EXIT;
	msg->data = NULL;

	if (OS_MsgQueueSend(&g_client_queue, msg, CMD_NOPOLL_QUEUE_WAIT_TIME) != OS_OK) {
		CMD_ERR("msg queue send failed\n");
		if (msg)
			cmd_free(msg);
		return CMD_STATUS_FAIL;
	}

	OS_MSleep(100);

	if (OS_MsgQueueDelete(&g_client_queue) != OS_OK) {
		CMD_ERR("msg queue delete failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static const struct cmd_data g_nopoll_server_cmds[] = {
	{ "init",	cmd_nopoll_server_init_exec },
	{ "open",	cmd_nopoll_server_open_exec },
	{ "start",	cmd_nopoll_server_start_exec },
	{ "stop",	cmd_nopoll_server_stop_exec },
	{ "close",	cmd_nopoll_server_close_exec },
	{ "deinit",	cmd_nopoll_server_deinit_exec },
};

static const struct cmd_data g_nopoll_client_cmds[] = {
	{ "init",	cmd_nopoll_client_init_exec },
	{ "open",	cmd_nopoll_client_open_exec },
	{ "send",	cmd_nopoll_client_send_exec },
	{ "recv",	cmd_nopoll_client_recv_exec },
	{ "close",	cmd_nopoll_client_close_exec },
	{ "deinit",	cmd_nopoll_client_deinit_exec },
};

static enum cmd_status cmd_nopoll_server_exec(char *cmd)
{
	return cmd_exec(cmd, g_nopoll_server_cmds, cmd_nitems(g_nopoll_server_cmds));
}

static enum cmd_status cmd_nopoll_client_exec(char *cmd)
{
	return cmd_exec(cmd, g_nopoll_client_cmds, cmd_nitems(g_nopoll_client_cmds));
}

static const struct cmd_data g_nopoll_cmds[] = {
	{ "srv",	cmd_nopoll_server_exec },
	{ "cli",	cmd_nopoll_client_exec },
};

enum cmd_status cmd_nopoll_exec(char *cmd)
{
	return cmd_exec(cmd, g_nopoll_cmds, cmd_nitems(g_nopoll_cmds));
}

#endif /* PRJCONF_NET_EN */
