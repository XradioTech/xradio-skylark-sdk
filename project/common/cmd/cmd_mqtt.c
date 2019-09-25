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
#include "cmd_mqtt.h"
#include "net/mqtt/MQTTClient-C/MQTTClient.h"


static MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
static Client client;
static Network network;

#define MQTT_BUF_SIZE (1024)

static char server_address[16];
static char server_port[6];
static int mqtt_ssl = 0;

static char *sub_topic[MAX_MESSAGE_HANDLERS];


#define CMD_MQTT_BG_THREAD_STACK_SIZE (1024 * 6)
static OS_Thread_t mqtt_bg_thread;
static int mqtt_bg_thread_run_flag = 0;

static enum cmd_status cmd_mqtt_init_exec(char *cmd)
{
	int cnt;
	uint32_t alive_interval;
	uint32_t clean;
	char client_id_temp[100] = {0}; //just for test, client id max is 65535 in 3.1.1
	char *client_id;
	char *send_buf;
	char *recv_buf;

	/* get param */
	cnt = cmd_sscanf(cmd, "alive=%u clean=%u clientid=%99s",
						&alive_interval, &clean, client_id_temp);

	/* check param */
	if (cnt != 3) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (clean > 1) {
		CMD_ERR("invalid clean %d\n", clean);
		return CMD_STATUS_INVALID_ARG;
	}

	client_id = cmd_malloc(cmd_strlen(client_id_temp) + 1);

	cmd_memcpy(client_id, client_id_temp, cmd_strlen(client_id_temp) + 1);
	CMD_DBG("client id = %s\n", client_id);

	connectData.clientID.cstring = client_id;
	connectData.keepAliveInterval = alive_interval;
	connectData.cleansession = clean;
	connectData.MQTTVersion = 4; //Version of MQTT 3.1.1

	send_buf = cmd_malloc(MQTT_BUF_SIZE);
	if (send_buf == NULL) {
		CMD_ERR("no memory\n");
		return CMD_STATUS_FAIL;
	}
	recv_buf = cmd_malloc(MQTT_BUF_SIZE);
	if (recv_buf == NULL) {
		cmd_free(send_buf);
		CMD_ERR("no memory\n");
		return CMD_STATUS_FAIL;
	}

	NewNetwork(&network);
	MQTTClient(&client, &network, 6000, (unsigned char*)send_buf, MQTT_BUF_SIZE,
				(unsigned char*)recv_buf, MQTT_BUF_SIZE);

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_mqtt_will_exec(char *cmd)
{
	int cnt;
	uint32_t qos;
	uint32_t retain;

	char topic_temp[50] = {0};//just for test, topic may be longer than 50
	char *topic;

	char message_temp[50] = {0};//just for test, message may be longer than 50
	char *message;

	/* get param */
	cnt = cmd_sscanf(cmd, "qos=%u retain=%u topic=%49s message=%49s",
							&qos, &retain, topic_temp, message_temp);

	/* check param */
	if (cnt != 4) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (qos > 2) {
		CMD_ERR("invalid qos %d\n", qos);
		return CMD_STATUS_INVALID_ARG;
	}

	if (retain > 1) {
		CMD_ERR("invalid retain %d\n", retain);
		return CMD_STATUS_INVALID_ARG;
	}

	CMD_DBG("will topic name = %s\n", topic_temp);
	CMD_DBG("will message = %s\n", message_temp);

	topic = cmd_malloc(cmd_strlen(topic_temp) + 1);
	if (topic == NULL) {
		CMD_ERR("no memory\n");
		return CMD_STATUS_FAIL;
	}
	message = cmd_malloc(cmd_strlen(message_temp) + 1);
	if (message == NULL) {
		CMD_ERR("no memory\n");
		cmd_free(topic);
		return CMD_STATUS_FAIL;
	}
	cmd_memcpy(topic, topic_temp, cmd_strlen(topic_temp) + 1);
	cmd_memcpy(message, message_temp, cmd_strlen(message_temp) + 1);

	//will function
	connectData.willFlag = 1;
	connectData.will.topicName.cstring = topic;
	connectData.will.message.cstring = message;
	connectData.will.retained = retain;
	connectData.will.qos = qos;

	return CMD_STATUS_ACKED;
}

static enum cmd_status cmd_mqtt_user_exec(char *cmd)
{
	char *user_name;

	user_name = cmd_malloc(cmd_strlen(cmd) + 1);
	if (user_name == NULL) {
		CMD_ERR("no memory\n");
		return CMD_STATUS_FAIL;
	}
	cmd_memcpy(user_name, cmd, cmd_strlen(cmd) + 1);

	connectData.username.cstring = user_name;

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_mqtt_password_exec(char *cmd)
{
	char *password;

	password = cmd_malloc(cmd_strlen(cmd) + 1);
	if (password == NULL) {
		CMD_ERR("no memory\n");
		return CMD_STATUS_FAIL;
	}
	cmd_memcpy(password, cmd, cmd_strlen(cmd) + 1);

	connectData.password.cstring = password;

	return CMD_STATUS_OK;
}


static void cmd_mqtt_bg(void *arg)
{
	int ret = 0;

	if (mqtt_ssl) {
		/* this test has no ca cert, no client cert, no client pk, no client pwd */
		ret = TLSConnectNetwork(&network, server_address, server_port, NULL, 0, NULL, 0, NULL, 0, NULL, 0);
		if (ret != 0) {
			CMD_ERR("Return code from network ssl connect is -0x%04x\n", -ret);
			goto exit;
		}
	} else {
		ret = ConnectNetwork(&network, server_address, atoi(server_port));
		if (ret != 0) {
			CMD_ERR("Return code from network connect is %d\n", ret);
			goto exit;
		}
	}

	if ((ret = MQTTConnect(&client, &connectData)) != 0) {
		CMD_ERR("Return code from MQTT connect is %d\n", ret);
		goto exit;
	}
	CMD_DBG("MQTT Connected\n");

	while (mqtt_bg_thread_run_flag) {
		if ((ret = MQTTYield(&client, 3000)) != 0) {
			CMD_WRN("Return code from yield is %d\n", ret);

reconnect:
			/* reconnect */
			if (!mqtt_bg_thread_run_flag)
				break;

			if (client.isconnected) {
				if ((ret = MQTTDisconnect(&client)) != 0)
					CMD_ERR("Return code from MQTT disconnect is %d\n", ret);
				network.disconnect(&network);
			}

			if (mqtt_ssl) {
				ret = TLSConnectNetwork(&network, server_address, server_port, NULL, 0, NULL, 0, NULL, 0, NULL, 0);
				if (ret != 0) {
					CMD_ERR("Return code from network ssl connect is -0x%04x\n", -ret);
					goto reconnect;
				}
			} else {
				ret = ConnectNetwork(&network, server_address, atoi(server_port));
				if (ret != 0) {
					CMD_ERR("Return code from network connect is %d\n", ret);
					goto reconnect;
				}
			}

			if ((ret = MQTTConnect(&client, &connectData)) != 0) {
				CMD_ERR("Return code from MQTT connect is %d\n", ret);
			}
		}
		OS_MSleep(10);
	}

exit:
	CMD_DBG("MQTT mqtt_bg_thread exit\n");
	OS_ThreadDelete(&mqtt_bg_thread);
}

static enum cmd_status cmd_mqtt_connect_exec(char *cmd)
{
	int cnt;

	/* get param */
	cnt = cmd_sscanf(cmd, "server=%15s port=%5s ssl=%u", server_address, server_port, &mqtt_ssl);

	/* check param */
	if (cnt != 3) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (OS_ThreadIsValid(&mqtt_bg_thread)) {
		CMD_ERR(" mqtt background thread has exist\n");
		return CMD_STATUS_FAIL;
	}

	mqtt_bg_thread_run_flag = 1;

	if (OS_ThreadCreate(&mqtt_bg_thread,
		                "mqtt_bg",
		                cmd_mqtt_bg,
		                NULL,
		                OS_PRIORITY_NORMAL,
		                CMD_MQTT_BG_THREAD_STACK_SIZE) != OS_OK) {
		CMD_ERR("create mqtt background failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

void mqtt_msg_cb(MessageData* data)
{
	CMD_DBG("[topic: %.*s] %.*s\n", data->topicName->lenstring.len,
					data->topicName->lenstring.data, data->message->payloadlen,
					(char *)data->message->payload);
}

static enum cmd_status cmd_mqtt_subscribe_exec(char *cmd)
{
	int cnt;
	uint32_t qos;
	int rc;

	char topic_temp[50] = {0};//just for test, topic may be longer than 50
	char *topic;

	/* get param */
	cnt = cmd_sscanf(cmd, "qos=%u topic=%49s", &qos, topic_temp);

	/* check param */
	if (cnt != 2) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (qos > 2) {
		CMD_ERR("invalid qos %d\n", qos);
		return CMD_STATUS_INVALID_ARG;
	}

	topic = cmd_malloc(cmd_strlen(topic_temp) + 1);
	cmd_memcpy(topic, topic_temp, cmd_strlen(topic_temp) + 1);

	/* must save topic when subscribe and free topic when unsubscribe */
	int i = 0;
	for (i = 0; i < MAX_MESSAGE_HANDLERS; i++) {
		if (sub_topic[i] == NULL) {
			sub_topic[i] = topic;
			break;
		}
	}

	if (i >= MAX_MESSAGE_HANDLERS) {
		CMD_ERR("Subscribe topic limit %d\n", MAX_MESSAGE_HANDLERS);
		cmd_free(topic);
		return CMD_STATUS_FAIL;
	}

	if ((rc = MQTTSubscribe(&client, sub_topic[i], qos, mqtt_msg_cb)) != 0) {
		CMD_ERR("Return code from MQTT subscribe is %d\n", rc);
		cmd_free(sub_topic[i]);
		sub_topic[i] = NULL;
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_mqtt_unsubscribe_exec(char *cmd)
{
	int cnt;
	char topic_temp[50] = {0};//just for test, topic may be longer than 50
	int rc;

	/* get param */
	cnt = cmd_sscanf(cmd, "topic=%49s", topic_temp);

	/* check param */
	if (cnt != 1) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if ((rc = MQTTUnsubscribe(&client, topic_temp)) != 0) {
		CMD_ERR("Return code from MQTT unsubscribe is %d\n", rc);
		return CMD_STATUS_FAIL;
	} else {
		/* free topic when unsubscribe */
		for (int i = 0; i < MAX_MESSAGE_HANDLERS; i++) {
			if (sub_topic[i] != NULL && !cmd_strncmp(sub_topic[i], topic_temp, cmd_strlen(topic_temp))) {
				cmd_free(sub_topic[i]);
				sub_topic[i] = NULL;
			}
		}
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_mqtt_publish_exec(char *cmd)
{
	int cnt;
	MQTTMessage message;
	uint32_t qos;
	uint32_t retain;
	int rc;

	char topic_temp[50] = {0};//just for test, topic may be longer than 50
	char message_temp[50] = {0};//just for test, message may be longer than 50

	/* get param */
	cnt = cmd_sscanf(cmd, "qos=%u retain=%u topic=%49s message=%49s",
			 				&qos, &retain, topic_temp, message_temp);

	/* check param */
	if (cnt != 4) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (qos > 2) {
		CMD_ERR("invalid qos %d\n", qos);
		return CMD_STATUS_INVALID_ARG;
	}

	if (retain > 1) {
		CMD_ERR("invalid retain %d\n", retain);
		return CMD_STATUS_INVALID_ARG;
	}

//	CMD_DBG("topic name = %s\n", topic_temp);
//	CMD_DBG("message = %s\n", message_temp);

	message.qos = qos;
	message.retained = retain;
	message.payload = message_temp;
	message.payloadlen = cmd_strlen(message_temp);

	if ((rc = MQTTPublish(&client, topic_temp, &message)) != 0)
		CMD_ERR("Return code from MQTT publish is %d\n", rc);
	else
		CMD_DBG("MQTT publish is success\n");

	return CMD_STATUS_ACKED;
}

static enum cmd_status cmd_mqtt_disconnect_exec(char *cmd)
{
	int rc;

	mqtt_bg_thread_run_flag	= 0;
	while(OS_ThreadIsValid(&mqtt_bg_thread))
		OS_MSleep(10); //wait thread exit

	if (client.isconnected) {
		if ((rc = MQTTDisconnect(&client)) != 0) {
			CMD_ERR("Return code from MQTT disconnect is %d\n", rc);
			network.disconnect(&network);
			return CMD_STATUS_FAIL;
		}

		network.disconnect(&network);
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_mqtt_deinit_exec(char *cmd)
{
	if (client.buf) {
		cmd_free(client.buf);
		client.buf = NULL;
		client.buf_size = 0;
	}

	if (client.readbuf) {
		cmd_free(client.readbuf);
		client.readbuf = NULL;
		client.readbuf_size = 0;
	}

	if (connectData.clientID.cstring) {
		cmd_free(connectData.clientID.cstring);
		connectData.clientID.cstring = NULL;
	}
	if (connectData.clientID.lenstring.len != 0) {
		cmd_free(connectData.clientID.lenstring.data);
		connectData.clientID.lenstring = (MQTTLenString){0, NULL};
	}

	if (connectData.will.topicName.cstring) {
		cmd_free(connectData.will.topicName.cstring);
		connectData.will.topicName.cstring = NULL;
	}
	if (connectData.will.topicName.lenstring.len != 0) {
		cmd_free(connectData.will.topicName.lenstring.data);
		connectData.will.topicName.lenstring = (MQTTLenString){0, NULL};
	}

	if (connectData.username.cstring) {
		cmd_free(connectData.username.cstring);
		connectData.username.cstring = NULL;
	}
	if (connectData.username.lenstring.len != 0) {
		cmd_free(connectData.username.lenstring.data);
		connectData.username.lenstring = (MQTTLenString){0, NULL};
	}

	if (connectData.password.cstring) {
		cmd_free(connectData.password.cstring);
		connectData.password.cstring = NULL;
	}
	if (connectData.password.lenstring.len != 0) {
		cmd_free(connectData.password.lenstring.data);
		connectData.password.lenstring = (MQTTLenString){0, NULL};
	}

	for (int i = 0; i < MAX_MESSAGE_HANDLERS; i++) {
		if (sub_topic[i]) {
			cmd_free(sub_topic[i]);
			sub_topic[i] = NULL;
		}
	}

	connectData = (MQTTPacket_connectData)MQTTPacket_connectData_initializer;

	return CMD_STATUS_OK;
}

static const struct cmd_data g_mqtt_cmds[] = {
	{ "init",        cmd_mqtt_init_exec },
	{ "will",        cmd_mqtt_will_exec },
	{ "username",    cmd_mqtt_user_exec },
	{ "password",    cmd_mqtt_password_exec },
	{ "connect",     cmd_mqtt_connect_exec },
	{ "subscribe",   cmd_mqtt_subscribe_exec },
	{ "unsubscribe", cmd_mqtt_unsubscribe_exec },
	{ "publish",     cmd_mqtt_publish_exec },
	{ "disconnect",  cmd_mqtt_disconnect_exec },
	{ "deinit",      cmd_mqtt_deinit_exec },
};

enum cmd_status cmd_mqtt_exec(char *cmd)
{
	return cmd_exec(cmd, g_mqtt_cmds, cmd_nitems(g_mqtt_cmds));
}

#endif /* PRJCONF_NET_EN */
