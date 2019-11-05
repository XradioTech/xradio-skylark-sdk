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
#include "net/mqtt/MQTTClient-C/MQTTClient.h"

#define MQTT_DEMO_THREAD_STACK_SIZE (8 * 1024) /* ssl need more stack */
static OS_Thread_t mqtt_demo_thread;

static MQTTPacket_connectData mqtt_demo_connectData = MQTTPacket_connectData_initializer;
static Client mqtt_demo_client;
static Network mqtt_demo_network;

#define MQTT_DEMO_CLIENT_ID "12345|securemode=2,signmethod=hmacsha1,timestamp=789|"
#define MQTT_DEMO_HOST_NAME "iot-as-mqtt.cn-shanghai.aliyuncs.com"
#define MQTT_DEMO_PORT      "1883"
#define MQTT_DEMO_USERNAME  "xr871_senor_set0&vo84Hm3xbUj"
#define MQTT_DEMO_PASSWORD  "4f9ebb2143f01e4489133c1ea55051fbbfc4c44e"
#define MQTT_DEMO_TOPIC     "/vo84Hm3xbUj/xr871_senor_set0/data"

#define MQTT_DEMO_BUF_SIZE (2*1024)

#define MQTT_DEMO_MSG_TEXT "mqtt demo test"

static int mqtt_demo_init(void)
{
	char *send_buf;
	char *recv_buf;

	/* init client id */
	mqtt_demo_connectData.clientID.cstring = MQTT_DEMO_CLIENT_ID;
	/* init keep alive interval */
	mqtt_demo_connectData.keepAliveInterval = 30; // 30s
	/* enable session reuse */
	mqtt_demo_connectData.cleansession = 0;
	/* set mqtt version */
	mqtt_demo_connectData.MQTTVersion = 4; //Version of MQTT 3.1.1

	/* send/recv buffer must free when mqtt deinit */
	send_buf = malloc(MQTT_DEMO_BUF_SIZE);
	if (send_buf == NULL) {
		printf("no memory\n");
		return -1;
	}
	recv_buf = malloc(MQTT_DEMO_BUF_SIZE);
	if (recv_buf == NULL) {
		free(send_buf);
		printf("no memory\n");
		return -1;
	}

	/* init network */
	NewNetwork(&mqtt_demo_network);
	/* init mqtt client object */
	MQTTClient(&mqtt_demo_client, &mqtt_demo_network, 6000,
				(unsigned char*)send_buf, MQTT_DEMO_BUF_SIZE,
				(unsigned char*)recv_buf, MQTT_DEMO_BUF_SIZE);

	/**
	 * set will function, when this client disconnect,
	 * server will sent the message to every client in MQTT_DEMO_TOPIC
	 */
	mqtt_demo_connectData.willFlag = 1;
	mqtt_demo_connectData.will.topicName.cstring = MQTT_DEMO_TOPIC;
	mqtt_demo_connectData.will.message.cstring = "I am disconnected";
	mqtt_demo_connectData.will.retained = 0;
	mqtt_demo_connectData.will.qos = 0;

	/* set username and password */
	mqtt_demo_connectData.username.cstring = MQTT_DEMO_USERNAME;
	mqtt_demo_connectData.password.cstring = MQTT_DEMO_PASSWORD;

	return 0;
}

static int mqtt_demo_connect(char *host_name, char *host_port)
{
	int ret = -1;

	/* need connect the server in tcp level first, if use ssl, use TLSConnectNetwork() */
	ret = ConnectNetwork(&mqtt_demo_network, host_name, atoi(host_port));
	if (ret != 0) {
		printf("mqtt connect faild, ret:%d, host:%s, port:%s\n", ret, host_name, host_port);
		goto exit;
	}

	/* if tcp level connected, then connect mqtt level */
	if ((ret = MQTTConnect(&mqtt_demo_client, &mqtt_demo_connectData)) != 0) {
		printf("mqtt connect faild, ret:%d\n", ret);
		/* disconnect the tcp level */
		mqtt_demo_network.disconnect(&mqtt_demo_network);
		goto exit;
	}
	printf("mqtt connected\n");

exit:
	return ret;
}

static void mqtt_demo_msg_cb(MessageData* data)
{
	printf("get a message, topic: %.*s, msg: %.*s\n", data->topicName->lenstring.len,
					data->topicName->lenstring.data, data->message->payloadlen,
					(char *)data->message->payload);
}

static int mqtt_demo_subscribe(char *topic)
{
	int ret = -1;

	if (mqtt_demo_client.isconnected) {
		/* set the message callback */
		if ((ret = MQTTSubscribe(&mqtt_demo_client, topic, 0, mqtt_demo_msg_cb)) != 0)
			printf("mqtt subscribe faild ret:%d\n", ret);
	}
	return ret;
}

static int mqtt_demo_unsubscribe(char *topic)
{
	int ret = -1;

	if (mqtt_demo_client.isconnected) {
		if ((ret = MQTTUnsubscribe(&mqtt_demo_client, topic)) != 0)
			printf("mqtt unsubscribe faild, ret:%d\n", ret);
	}
	return ret;
}

static int mqtt_demo_publish(char *topic, char *msg)
{
	int ret = -1;

	MQTTMessage message;

	memset(&message, 0, sizeof(message));
	message.qos = 0;
	message.retained = 0; /* disable retain the message in server */
	message.payload = msg;
	message.payloadlen = strlen(msg);

	if ((ret = MQTTPublish(&mqtt_demo_client, topic, &message)) != 0)
		printf("mqtt publish faild, ret:%d\n", ret);

	return ret;
}

static int mqtt_demo_disconnect(void)
{
	int ret = -1;

	if (mqtt_demo_client.isconnected) {
		/* need disconnect mqtt level first */
		if ((ret = MQTTDisconnect(&mqtt_demo_client)) != 0)
			printf("mqtt disconnect fail, ret:%d\n", ret);
		/* then disconnect tcp level */
		mqtt_demo_network.disconnect(&mqtt_demo_network);
	}

	return ret;
}

static void mqtt_demo_deinit(void)
{
	if (mqtt_demo_client.buf) {
		free(mqtt_demo_client.buf);
		mqtt_demo_client.buf = NULL;
	}
	if (mqtt_demo_client.readbuf) {
		free(mqtt_demo_client.readbuf);
		mqtt_demo_client.readbuf = NULL;
	}
}

static void mqtt_demo_fun(void *arg)
{
	int ret;
	int reconnect_times = 0;

	/* mqtt init */
	mqtt_demo_init();

	/* mqtt connect */
	ret = mqtt_demo_connect(MQTT_DEMO_HOST_NAME, MQTT_DEMO_PORT);
	if (ret != 0)
		goto exit;

	/* subscribe topic */
	ret = mqtt_demo_subscribe(MQTT_DEMO_TOPIC);
	if (ret != 0)
		goto exit;

	while (1) {
		/* publish message to topic */
		mqtt_demo_publish(MQTT_DEMO_TOPIC, MQTT_DEMO_MSG_TEXT);
		if ((ret = MQTTYield(&mqtt_demo_client, 300)) != 0) {
			printf("mqtt yield err, ret:%d\n", ret);
reconnect:
			printf("mqtt reconnect\n");
			mqtt_demo_disconnect();
			ret = mqtt_demo_connect(MQTT_DEMO_HOST_NAME, MQTT_DEMO_PORT);
			if (ret != 0) {
				reconnect_times++;
				if (reconnect_times > 5)
					goto exit;
				OS_MSleep(5000); //5s
				goto reconnect;
			}
		}
		OS_MSleep(1000); //1s
	}

exit:
	mqtt_demo_unsubscribe(MQTT_DEMO_TOPIC);
	mqtt_demo_disconnect();
	mqtt_demo_deinit();

	OS_ThreadDelete(&mqtt_demo_thread);
}

static void net_cb(uint32_t event, uint32_t data, void *arg)
{
	uint16_t type = EVENT_SUBTYPE(event);

	switch (type) {
		case NET_CTRL_MSG_NETWORK_UP:
			if (!OS_ThreadIsValid(&mqtt_demo_thread)) {
				OS_ThreadCreate(&mqtt_demo_thread,
									"mqtt_demo_thread",
									mqtt_demo_fun,
									(void *)NULL,
									OS_THREAD_PRIO_APP,
									MQTT_DEMO_THREAD_STACK_SIZE);
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

	printf("mqtt demo start\n\n");

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

