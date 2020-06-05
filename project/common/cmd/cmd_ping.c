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
#include "cmd_ping.h"
#include "lwip/netdb.h"
#include "net/ping/ping.h"
#include <getopt.h>

#if CMD_DESCRIBE
#define ping_help_info \
"[*] -c count\n"\
"\tStop after sending count ECHO_REQUEST packets. With deadline option,\n"\
"\tping waits for count ECHO_REPLY packets, until	the timeout expires.\n"\
"[*] -i interval\n"\
"\tWait  interval seconds between sending each packet.\n"\
"\tThe default is to wait for one second between each packet normally.\n"\
"[*] -s packetsize\n"\
"\tSpecifies  the  number  of data bytes to be sent.\n"\
"[*] -W timeout\n"\
"\tTime to wait for a response, in seconds.\n"\
"[*] -w deadline\n"\
"\tSpecify a timeout, in seconds, before ping exits regardless of how many\n"\
"\tpackets have been sent or received.\n"\
"\tIn this case ping  does  not stop after count packet are sent,\n"\
"\tit waits either for deadline expire or until count probes are answered\n"\
"\tor for some error notification from network.\n"\
"[*] -e exit\n"\
"\tExit the ping thread according to the handle."
#endif /* CMD_DESCRIBE */

typedef struct {
	OS_Thread_t ping_thread;
	int handle; 		/* the ping hread handle */

	struct ping_data ping_arg;
} cmd_ping_arg;

#define PING_ARG_HANDLE_MAX		2
cmd_ping_arg *s_cmd_ping_arg_handle[PING_ARG_HANDLE_MAX] = {NULL};

#define CMD_PING_DEFAULT_COUNT (3)
#define CMD_PING_DEFAULT_DATA_LONG (0xffff)
#define CMD_PING_DEFAULT_INTERVAL (1)
#define CMD_PING_DEFAULT_TIMEOUT (5*1000)

static int ping_handle_new(struct ping_data *arg)
{
	if (arg == NULL)
		return -1;
	for (int i = 0; i < PING_ARG_HANDLE_MAX; i++) {
		if (s_cmd_ping_arg_handle[i] == NULL) {
			s_cmd_ping_arg_handle[i] = malloc(sizeof(cmd_ping_arg));
			if (s_cmd_ping_arg_handle[i] == NULL) {
				printf("ping data malloc err!\n");
				return -1;
			}
			memset(s_cmd_ping_arg_handle[i], 0, sizeof(cmd_ping_arg));
			memcpy(&s_cmd_ping_arg_handle[i]->ping_arg, arg, sizeof(*arg));
			s_cmd_ping_arg_handle[i]->handle = i;
			return i;
		}
	}
	return -1;
}

static int ping_handle_free(int handle)
{
	if (handle < 0 || handle >PING_ARG_HANDLE_MAX)
		return -1;

	if (s_cmd_ping_arg_handle[handle] != NULL) {
		free(s_cmd_ping_arg_handle[handle]);
		s_cmd_ping_arg_handle[handle] = NULL;
	}
	return 0;
}

static void ping_thread_fun(void *arg)
{
	cmd_ping_arg *cmd_ping = s_cmd_ping_arg_handle[(int)arg];

	ping(&cmd_ping->ping_arg);

	OS_Thread_t thread = cmd_ping->ping_thread;
	ping_handle_free(cmd_ping->handle);
	OS_ThreadDelete(&thread);
}

static int ping_handle_start(int handle)
{
	OS_Status ret;

	if (handle < 0 || handle > PING_ARG_HANDLE_MAX)
		return -1;

	if (s_cmd_ping_arg_handle[handle] == NULL)
		return -1;

	ret = OS_ThreadCreate(&s_cmd_ping_arg_handle[handle]->ping_thread,
							"ping thread",
							ping_thread_fun,
							(void*)handle,
							OS_PRIORITY_NORMAL,
							1 * 1024);
	if (ret != OS_OK)
		return -1;
	return 0;
}

static int ping_handle_stop(int handle)
{
	if (handle < 0 || handle > PING_ARG_HANDLE_MAX)
		return -1;

	if (s_cmd_ping_arg_handle[handle])
		s_cmd_ping_arg_handle[handle]->ping_arg.run_flag = 0;

	return 0;
}

int ping_parse_argv(int argc, char *argv[])
{
	int temp = 0;
	int handle = -1;
	struct ping_data ping_data_t;
	int opt = 0;
	char *short_opts = "c:i:s:W:w:t:e:";
	memset(&ping_data_t, 0, sizeof(ping_data_t));

	/* set default value */
	ping_data_t.count = CMD_PING_DEFAULT_COUNT;
	ping_data_t.data_long = CMD_PING_DEFAULT_DATA_LONG;
	ping_data_t.interval = CMD_PING_DEFAULT_INTERVAL;
	ping_data_t.timeout = CMD_PING_DEFAULT_TIMEOUT;

	opterr = 0; /* close the "invalid option" warning */
	while ((opt = getopt(argc, argv, short_opts)) != -1) {
		//printf("optind:%d\t opt:%c\t optarg:%s\t argv[optind]:%s\n", optind, opt, optarg, argv[optind]);
		switch (opt) {
			case 'c':
				temp = atoi(optarg);
				ping_data_t.count = temp > 0 ? temp : CMD_PING_DEFAULT_COUNT;
				break;
			case 'i':
				temp = atoi(optarg);
				ping_data_t.interval = temp > 0 ? temp : CMD_PING_DEFAULT_INTERVAL;
				break;
			case 's':
				ping_data_t.data_long = (uint32_t)atoi(optarg);
				if (ping_data_t.data_long > 65500)
					ping_data_t.data_long = 65500;
				break;
			case 'W':
				temp = atoi(optarg);
				ping_data_t.timeout = temp > 0 ? temp : CMD_PING_DEFAULT_TIMEOUT;
				break;
			case 'w':
				temp = atoi(optarg);
				ping_data_t.deadline = temp > 0 ? temp : 1;
				break;
			case 't':
				temp = atoi(optarg);
				ping_data_t.ttl = temp > 0 && temp < 255 ? temp : 255;
				break;
			case 'e':
				temp = atoi(optarg);
				ping_handle_stop(temp);
				goto exit;
				break;
			case '?':
				printf("invalid option -- '%s'\n", argv[optind - 1]);
				goto exit;
				break;
			default:
				goto exit;
				break;
		}
	}

	if (optind >= argc) {
		printf("err: no destination!\n");
		goto exit;
	}

	struct hostent *host_entry;
	unsigned int address = 0;
	host_entry = gethostbyname(argv[optind]);
	if(host_entry) {
		address = *((u_long*)host_entry->h_addr_list[0]);
	} else {
		printf("invalid ping host.\n");
		goto exit;
	}

#ifdef __CONFIG_LWIP_V1
    ip4_addr_set_u32(&ping_data_t.sin_addr, address);
#elif LWIP_IPV4 /* now only for IPv4 */
    ip_addr_set_ip4_u32(&ping_data_t.sin_addr, address);
#else
    #error "IPv4 not support!"
#endif

	handle = ping_handle_new(&ping_data_t);
	if (handle >= 0)
		ping_handle_start(handle);

exit:
	optind = 1; /* reset the index, optind must be 1, the first arg is argv[1] */

	return handle >= 0 ? 0 : -1;
}

static enum cmd_status cmd_ping_help_exec(char *cmd)
{
#if CMD_DESCRIBE
	CMD_LOG(1, "%s\n", ping_help_info);
#endif
	return CMD_STATUS_ACKED;
}

enum cmd_status cmd_ping_exec(char *cmd)
{
	int ret = 0;
	int argc;
	char *argv[20];
	struct ping_data pdata;
	memset((void*) &pdata, 0, sizeof(pdata));

	if (cmd_strcmp(cmd, "help") == 0 || cmd_strcmp(cmd, "-h") == 0) {
		cmd_ping_help_exec(cmd);
		return CMD_STATUS_ACKED;
	}

	argc = cmd_parse_argv(cmd, &argv[1], cmd_nitems(argv) - 1);
	argv[0] = "ping";

	ret = ping_parse_argv(argc + 1, argv);

	if (ret != 0)
		return CMD_STATUS_FAIL;
	else
		return CMD_STATUS_OK;
}

#endif /* PRJCONF_NET_EN */
