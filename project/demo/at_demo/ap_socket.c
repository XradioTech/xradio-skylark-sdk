#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "kernel/os/os.h"
#include "net/wlan/wlan.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"
#include "serial.h"
#include "ap_socket.h"
#include "atcmd/at_command.h"

typedef struct Timer Timer;

struct Timer
{
	unsigned int end_time;
};

#define SOCKET_DATALEN_MAX				1460
#define SOCKET_LISTEN_MAX				8
#define SOCKET_TEST_PORT				8080

#define CONNECT_MAX						5
#define SERVER_THREAD_STACK_SIZE	(2 * 1024)

static int xr_listen_fd;
static int xr_client_fd[CONNECT_MAX];
static uint8_t *socket_revbuffer[CONNECT_MAX];
static struct sockaddr_in saddr = { 0 };
static OS_Thread_t g_server_thread[CONNECT_MAX];
static void ap_connect0_read(void *pvParameters);
static void ap_connect1_read(void *pvParameters);
static void ap_connect2_read(void *pvParameters);
static void ap_connect3_read(void *pvParameters);
static void ap_connect4_read(void *pvParameters);
//static uint8_t Cur_connect_max = 1;

static void xr_countdown_ms(Timer* timer, unsigned int timeout_ms)
{
	timer->end_time = OS_TicksToMSecs(OS_GetTicks()) + timeout_ms;
}

static int xr_left_ms(Timer* timer)
{
	int diff = (int)(timer->end_time) - (int)(OS_TicksToMSecs(OS_GetTicks()));
	return (diff < 0) ? 0 : diff;
}

static char xr_expired(Timer* timer)
{
	return 0 <= (int)OS_TicksToMSecs(OS_GetTicks()) - (int)(timer->end_time); /* is time_now over than end time */
}

int xr_socket_init(void)
{
	int i;

	xr_listen_fd = -1;
	memset(xr_client_fd,-1,CONNECT_MAX*sizeof(int));

	for(i = 0;i < CONNECT_MAX;i++){
		socket_revbuffer[i] = malloc(SOCKET_DATALEN_MAX);
		memset(socket_revbuffer[i],0,SOCKET_DATALEN_MAX);
	}

	OS_ThreadCreate(&g_server_thread[0],
						"",
						ap_connect0_read,
						NULL,
						OS_PRIORITY_NORMAL,
						SERVER_THREAD_STACK_SIZE);
	OS_ThreadCreate(&g_server_thread[1],
						"",
						ap_connect1_read,
						NULL,
						OS_PRIORITY_NORMAL,
						SERVER_THREAD_STACK_SIZE);
	OS_ThreadCreate(&g_server_thread[2],
						"",
						ap_connect2_read,
						NULL,
						OS_PRIORITY_NORMAL,
						SERVER_THREAD_STACK_SIZE);
	OS_ThreadCreate(&g_server_thread[3],
						"",
						ap_connect3_read,
						NULL,
						OS_PRIORITY_NORMAL,
						SERVER_THREAD_STACK_SIZE);
	OS_ThreadCreate(&g_server_thread[4],
						"",
						ap_connect4_read,
						NULL,
						OS_PRIORITY_NORMAL,
						SERVER_THREAD_STACK_SIZE);
	return 0;
}

#if 0
static int xr_socket_create(int port)
{
    int err = -1;
 	int option;

	xr_listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (xr_listen_fd < 0) {
		AP_SOCKET_ERR("socket create err!\r\n");
		goto socket_err;
	}

	if (port) {
		saddr.sin_port = htons(port);
	} else {
		saddr.sin_port = htons(80);
	}
	saddr.sin_addr.s_addr	= htonl(INADDR_ANY);
	saddr.sin_family		= AF_INET;

	option = 1;
	if (setsockopt(xr_listen_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&option, sizeof(option)) < 0) {
		AP_SOCKET_ERR("failed to setsockopt sock_fd!\n");
		closesocket(xr_listen_fd);
		return err;
	}

	option = 1;
	if (setsockopt(xr_listen_fd, SOL_SOCKET, SO_REUSEPORT, (char *)&option, sizeof(option)) < 0) {
		AP_SOCKET_ERR("failed to setsockopt sock_fd!\n");
		closesocket(xr_listen_fd);
		return err;
	}

	err = bind(xr_listen_fd, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
	if (err < 0) {
		AP_SOCKET_ERR("Failed to bind tcp socket. Error %d \n", errno);
		goto socket_err;
	}

	if (listen(xr_listen_fd, SOCKET_LISTEN_MAX) != 0) {
		AP_SOCKET_ERR("listen failed \n");
		goto socket_err;
	}
	err = 0;
	AP_SOCKET_DBG("socket listen success!============\n");

socket_err:
	closesocket(xr_listen_fd);

	return err;
}
#endif

static int xr_socket_send(int fd, unsigned char *buffer, int len, int timeout_ms)
{
    int rc = -1;
	int sentLen = 0;
    fd_set fdset;
    struct timeval tv;

    FD_ZERO(&fdset);
    FD_SET(fd, &fdset);

 	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;

    rc = select(fd + 1, NULL, &fdset, NULL, &tv);
    if (rc > 0) {
		if ((rc = send(fd, buffer, len, 0)) > 0) {
			sentLen = rc;
			AP_SOCKET_DBG("[xr_socket_send] server ok, lrn = %d\r\n", sentLen);
		}
		else if (rc == 0) {
			sentLen = -1; /* disconnected with server */
			AP_SOCKET_ERR("[xr_socket_send] server disconnected!\r\n");
		}
		else {
			AP_SOCKET_ERR("send return %d, errno = %d\n", rc, errno);
			sentLen = -2; /* network error */
		}
	} else if (rc == 0) {
		sentLen = 0; /* timeouted and sent 0 bytes */
	}
	else {
		AP_SOCKET_ERR("select return %d, errno = %d\n", rc, errno);
		sentLen = -2; /* network error */
	}

	return sentLen;
}

static int xr_socket_read(int fd, unsigned char *buffer, int len, int timeout_ms)
{
	int recvLen = 0;
	int leftms;
	int rc = -1;
	struct timeval tv;
	Timer timer;
	fd_set fdset;

	xr_countdown_ms(&timer, timeout_ms);

	do {
		leftms = xr_left_ms(&timer);
		tv.tv_sec = leftms / 1000;
		tv.tv_usec = (leftms % 1000) * 1000;

		FD_ZERO(&fdset);
		FD_SET(fd, &fdset);

		rc = select(fd + 1, &fdset, NULL, NULL, &tv);
		if (rc > 0) {
			rc = recv(fd, buffer + recvLen, len - recvLen, 0);
			if (rc > 0) {
				/* received normally */
				recvLen += rc;
			} else if (rc == 0) {
				/* has disconnected with server */
				recvLen = -1;
				break;
			} else {
				/* network error */
				AP_SOCKET_ERR("recv return %d, errno = %d\n", rc, errno);
				recvLen = -2;
				break;
			}
		} else if (rc == 0) {
			if (recvLen != 0)
				AP_SOCKET_ERR("received timeout and length had received is %d\n", recvLen);
			/* timeouted and return the length received */
			break;
		} else {
			/* network error */
			AP_SOCKET_ERR("select return %d, errno = %d\n", rc, errno);
			recvLen = -2;
			break;
		}
	} while (recvLen < len && !xr_expired(&timer)); /* expired() is redundant? */

	return recvLen;
}

static void ap_connect0_read(void *pvParameters)
{
	int rec_len;

	while (1) {
		OS_MSleep(1);
		if(xr_client_fd[0] >= 0){
			rec_len = xr_socket_read(xr_client_fd[0], socket_revbuffer[0], SOCKET_DATALEN_MAX, 10);
			if (rec_len > 0) {
				at_dump("+IPD,0,%d\r\n",rec_len);
				at_data_output((char *)socket_revbuffer[0],rec_len);
				memset(socket_revbuffer[0], 0, SOCKET_DATALEN_MAX);
			} else if (rec_len < 0) {
				closesocket(xr_client_fd[0]);
				xr_client_fd[0] = -1;
				//break;
			}
		}
	}

}

static void ap_connect1_read(void *pvParameters)
{
	int rec_len;

	while (1) {
		OS_MSleep(1);
		if(xr_client_fd[1] >= 0){
			rec_len = xr_socket_read(xr_client_fd[1], socket_revbuffer[1], SOCKET_DATALEN_MAX, 10);
			if (rec_len > 0) {
				at_dump("+IPD,1,%d\r\n",rec_len);
				at_data_output((char *)socket_revbuffer[1],rec_len);
				memset(socket_revbuffer[1], 0, SOCKET_DATALEN_MAX);
			} else if (rec_len < 0) {
				closesocket(xr_client_fd[1]);
				xr_client_fd[1] = -1;
				//break;
			}
		}
    }

}

void ap_connect2_read(void *pvParameters)
{
	int rec_len;

	while (1) {
		OS_MSleep(1);
		if(xr_client_fd[2] >= 0){
			rec_len = xr_socket_read(xr_client_fd[2], socket_revbuffer[2], SOCKET_DATALEN_MAX, 10);
			if (rec_len > 0) {
				at_dump("+IPD,2,%d\r\n",rec_len);
				at_data_output((char *)socket_revbuffer[2],rec_len);
				memset(socket_revbuffer[2], 0, SOCKET_DATALEN_MAX);
			} else if (rec_len < 0) {
				closesocket(xr_client_fd[2]);
				xr_client_fd[2] = -1;
				//break;
			}
		}
    }

}

static void ap_connect3_read(void *pvParameters)
{
	int rec_len;

	while (1) {
		OS_MSleep(1);
		if(xr_client_fd[3] >= 0){
			rec_len = xr_socket_read(xr_client_fd[3], socket_revbuffer[3], SOCKET_DATALEN_MAX, 10);
			if (rec_len > 0) {
				at_dump("+IPD,3,%d\r\n",rec_len);
				at_data_output((char *)socket_revbuffer[3],rec_len);
				memset(socket_revbuffer[3], 0, SOCKET_DATALEN_MAX);
			} else if (rec_len < 0) {
				closesocket(xr_client_fd[3]);
				xr_client_fd[3] = -1;
				//break;
			}
		}
    }

}

static void ap_connect4_read(void *pvParameters)
{
	int rec_len;

	while (1) {
		OS_MSleep(1);
		if(xr_client_fd[4] >= 0){
			rec_len = xr_socket_read(xr_client_fd[4], socket_revbuffer[4], SOCKET_DATALEN_MAX, 10);
			if (rec_len > 0) {
				at_dump("+IPD,4,%d\r\n",rec_len);
				at_data_output((char *)socket_revbuffer[4],rec_len);
				memset(socket_revbuffer[4], 0, SOCKET_DATALEN_MAX);
			} else if (rec_len < 0) {
				closesocket(xr_client_fd[4]);
				xr_client_fd[4] = -1;
				//break;
			}
		}
    }

}

extern void ap_task_delete(void);
extern void close_server(void);

void ap_socket_task(int port)
{
	int err,i;
	int option;
	int temp_fd = -1;
	socklen_t socklen = sizeof(saddr);

	xr_socket_init();

    if (xr_listen_fd < 0) {
        xr_listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (xr_listen_fd < 0) {
			printf("socket create err!\r\n");
            goto socket_err;
        }

		AP_SOCKET_DBG("->>>>>>>>>>port=%d\r\n",port);
        saddr.sin_family		= AF_INET;
        saddr.sin_port			= htons(port);
        saddr.sin_addr.s_addr	= htonl(INADDR_ANY);

		option = 1;
		if (setsockopt(xr_listen_fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option)) < 0) {
			AP_SOCKET_ERR("failed to setsockopt sock_fd!\n");
			closesocket(xr_listen_fd);
			return;
		}

		option = 1;
		if (setsockopt(xr_listen_fd, SOL_SOCKET, SO_REUSEPORT, &option, sizeof(option)) < 0) {
		 	AP_SOCKET_ERR("failed to setsockopt sock_fd!\n");
			closesocket(xr_listen_fd);
			return;
		}

        err = bind(xr_listen_fd, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
        if (err < 0) {
            AP_SOCKET_ERR( "Failed to bind tcp socket. Error %d \n", errno);
            goto socket_err;
        }

        if (listen(xr_listen_fd, SOCKET_LISTEN_MAX) != 0) {
            AP_SOCKET_ERR("listen failed \n");
            goto socket_err;
        }
		AP_SOCKET_DBG("socket listen success!============\n");
    }

    while(1) {
		option = 1000;
		setsockopt(xr_listen_fd, SOL_SOCKET, SO_RCVTIMEO, &option, sizeof(option));

		temp_fd = accept(xr_listen_fd, (struct sockaddr *)&saddr, &socklen);
		if(temp_fd > 0){

			for(i = 0;i < CONNECT_MAX;i++)
			{
				if(xr_client_fd[i] < 0)
				{
					xr_client_fd[i] = temp_fd;
					AP_SOCKET_DBG("connect [%d] = %d:\r\n",i,temp_fd);
					temp_fd = -1;
					break;
				}
			}
		}
		else{
			OS_MSleep(100);
			//printf("temp_fd = %d:\r\n",temp_fd);
			//continue;
		}
	}

socket_err:
	ap_task_delete();
	close_server();
}

void close_server(void)
{
	int i;

	AP_SOCKET_DBG("colse all socket!!!\r\n");
	closesocket(xr_listen_fd);
    xr_listen_fd = -1;
	for(i = 0;i < CONNECT_MAX;i++)
	{
		closesocket(xr_client_fd[i]);
		xr_client_fd[i] = -1;
		free(socket_revbuffer[i]);
		OS_ThreadDelete(&g_server_thread[i]);
	}
}

