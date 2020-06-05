#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <lwip/tcpip.h>
#include <lwip/inet.h>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <lwip/ip.h>
#include <lwip/icmp.h>
#include <lwip/inet_chksum.h>
#include "net/ping/ping.h"
#include "kernel/os/os_thread.h"

#define PING_TO		5000    /* timeout to wait every reponse(ms) */
#define PING_ID		0xABCD
#define PING_DATA_SIZE	100     /* size of send frame buff, not include ICMP frma head */
#define PING_IP_HDR_SIZE	40
#define GET_TICKS	OS_GetTicks

static void generate_ping_echo(u8_t *buf, u32_t len, u16_t seq, u16_t id)
{
	u32_t i;
	u32_t data_len = len - sizeof(struct icmp_echo_hdr);
	struct icmp_echo_hdr *pecho;

	pecho = (struct icmp_echo_hdr *)buf;

	ICMPH_TYPE_SET(pecho, ICMP_ECHO);
	ICMPH_CODE_SET(pecho, 0);

	pecho->chksum = 0;
	pecho->id = id;
	pecho->seqno = htons(seq);

	/* fill the additional data buffer with some data */
	for (i = 0; i < data_len; i++) {
		buf[sizeof(struct icmp_echo_hdr) + i] = (unsigned char)i;
	}
	/* Checksum of icmp header and data */
	pecho->chksum = inet_chksum(buf, len);
}

s32_t ping(struct ping_data *data)
{
	struct sockaddr_in ToAddr;
	struct sockaddr_in FromAddr;
	socklen_t          FromLen;
	int 	           iSockID,iStatus;
	fd_set ReadFds;
	struct timeval Timeout;

	u8_t *ping_buf;
	u32_t buf_size, request_size, reply_size;
	struct ip_hdr *iphdr;
	struct icmp_echo_hdr *pecho;
	u16_t ping_seq_num = 1;
	s32_t ping_pass = 0;
	u16_t ping_fail = 0;
	u32_t i;
	u32_t TimeStart=0,TimeNow=0,TimeElapse=0;
	u32_t start_time = 0, stop_time = 0;
	u16_t ping_ids = 0x1234;
	u32_t min_time = 0, max_time = 0, avg_time = 0;

	memset(&FromAddr, 0, sizeof(FromAddr));
	FromLen = sizeof(FromAddr);

	iSockID = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
	if (iSockID < 0) {
		printf("create socket fail. errno:%d\n", errno);
		return -1;
	}

//	fcntl(iSockID, F_SETFL, O_NONBLOCK);  /* set noblocking */
	int val = 1;
	iStatus = ioctlsocket(iSockID, FIONBIO, (void *)&val);/* set noblocking */
	if (iStatus < 0)
		printf("setsockopt err! errno:%d\n", errno);

	if (data->ttl > 0) {
		val = data->ttl;
		if (val > 255)
			val = 255;
		iStatus = setsockopt(iSockID, IPPROTO_IP, IP_TTL, &val, sizeof(val));
		if (iStatus < 0)
			printf("setsockopt err! errno:%d\n", errno);
	}

	memset(&ToAddr, 0, sizeof(ToAddr));
	ToAddr.sin_len = sizeof(ToAddr);
	ToAddr.sin_family = AF_INET;
	//ToAddr.sin_port = data->port;
#ifdef __CONFIG_LWIP_V1
	inet_addr_from_ipaddr(&ToAddr.sin_addr, &data->sin_addr);
#elif LWIP_IPV4 /* now only for IPv4 */
	inet_addr_from_ip4addr(&ToAddr.sin_addr, ip_2_ip4(&data->sin_addr));
#else
	#error "IPv4 not support!"
#endif
	if (data->data_long != 0xffff) {
		request_size = data->data_long;
		if (request_size > 65500)
			request_size = 65500;
	} else {
		request_size = PING_DATA_SIZE;
	}
	request_size += sizeof(struct icmp_echo_hdr);
	buf_size = request_size + PING_IP_HDR_SIZE;
	ping_buf = malloc(buf_size);
	if (!ping_buf) {
		return -1;
	}

	start_time = GET_TICKS();
	if (data->deadline > 0)
		stop_time = start_time + data->deadline * 1000;

	data->run_flag = 1;

	printf("PING %s %d bytes of data.\n", inet_ntoa(data->sin_addr), request_size);

	for (i = 0; i < data->count && data->run_flag; i++) {
		generate_ping_echo(ping_buf, request_size, ping_seq_num, ping_ids);
		sendto(iSockID, ping_buf, request_size, 0, (struct sockaddr*)&ToAddr, sizeof(ToAddr));
		TimeStart = GET_TICKS();
		while (data->run_flag) {
			FD_ZERO(&ReadFds);
			FD_SET(iSockID, &ReadFds);
			Timeout.tv_sec = 0;
			Timeout.tv_usec = 50*1000;   /* 50ms */
			iStatus = select(FD_SETSIZE, &ReadFds, NULL, NULL, &Timeout);
			if (iStatus > 0 && FD_ISSET(iSockID, &ReadFds)) {
			/* block mode can't be used, we wait here if receiving party has sended,
			 * but we can set select to timeout mode to lower cpu's utilization */
				reply_size = recvfrom(iSockID, ping_buf, buf_size, 0,
				                           (struct sockaddr*)&FromAddr, &FromLen);
				if (reply_size >= (int)(sizeof(struct ip_hdr)+sizeof(struct icmp_echo_hdr))) {
					TimeNow = GET_TICKS();
					if (TimeNow >= TimeStart) {
						TimeElapse = TimeNow - TimeStart;
					} else {
						TimeElapse = 0xffffffffUL - TimeStart + TimeNow;
					}
					iphdr = (struct ip_hdr *)ping_buf;
					pecho = (struct icmp_echo_hdr *)(ping_buf + (IPH_HL(iphdr) * 4));
					if ((pecho->id == ping_ids) && (pecho->seqno == htons(ping_seq_num))) {
						/* do some ping result processing */
						if (ping_pass) {
							if (TimeElapse > max_time)
								max_time = TimeElapse;
							if (TimeElapse < min_time)
								min_time = TimeElapse;
						} else {
							max_time = min_time = TimeElapse;
						}
						avg_time += TimeElapse;

						printf("%d bytes from %s: icmp_seq=%d    time=%d ms\n",
						       (reply_size - (IPH_HL(iphdr) * 4) - sizeof(struct icmp_echo_hdr)),
						       inet_ntoa(FromAddr.sin_addr),
						       htons(pecho->seqno), TimeElapse);
						ping_pass++;
						break;
					}
				}
			}

			TimeNow = GET_TICKS();
			if (TimeNow >= TimeStart) {
				TimeElapse = TimeNow - TimeStart;
			} else {
				TimeElapse = 0xffffffffUL - TimeStart + TimeNow;
			}
			if (TimeElapse >= data->timeout) {  /* giveup this wait, if wait timeout */
				printf("Request timeout for icmp_seq=%d\n", ping_seq_num);
				ping_fail++;
				break;
			}
		}

		ping_seq_num++;
		ping_ids++;
		if (ping_ids == 0x7FFF)
			ping_ids = 0x1234;

		if (data->deadline > 0) {
			if (GET_TICKS() >= stop_time) {
				data->run_flag = 0;
				break;
			}
		}

		OS_Sleep(data->interval);
	}

	printf("\n--- %s ping statistics ---\n"\
           "%d packets transmitted, %d received, %d%% packet loss, time %dms\n"\
           "rtt min/avg/max/mdev = %d/%d/%d/%d ms\n",
           inet_ntoa(data->sin_addr), ping_seq_num - 1, ping_pass,
           ping_fail * 100 / (ping_seq_num - 1), GET_TICKS() - start_time,
           min_time, ping_pass > 0 ? avg_time/ping_pass : 0, max_time, max_time - min_time);

	free(ping_buf);
	closesocket(iSockID);
	if (ping_pass > 0)
		return ping_pass;
	else
		return -1;
}

