/*******************************************************************************
 * Copyright (c) 2014, 2015 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *
 *******************************************************************************/

#ifndef __MQTT_XR_RTOS__
#define __MQTT_XR_RTOS__

#include "kernel/os/os.h"

#include "mbedtls/net.h"
#include "mbedtls/ssl.h"
#include "mbedtls/certs.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/debug.h"

typedef struct Timer Timer;

struct Timer
{
	unsigned int end_time;
};

void InitTimer(Timer*);
char expired(Timer*);
void countdown_ms(Timer*, unsigned int);
void countdown(Timer*, unsigned int);
int left_ms(Timer*);

typedef struct Network Network;

/*
struct Network
{
	int my_socket;
	int (*mqttread) (Network*, unsigned char*, int, int);
	int (*mqttwrite) (Network*, unsigned char*, int, int);
	void (*disconnect) (Network*);
};
*/

struct Network {
	int my_socket;
	int (*mqttread)(Network *, unsigned char *, int, int);
	int (*mqttwrite)(Network *, unsigned char *, int, int);
	void (*disconnect)(Network *);

	mbedtls_net_context *fd;
	mbedtls_ssl_context *ssl;
	mbedtls_ssl_config *conf;
	mbedtls_entropy_context *entropy;
	mbedtls_ctr_drbg_context *ctr_drbg;
	mbedtls_x509_crt *cacertl; //The ca certificate or chain
	mbedtls_x509_crt *clicert; //The own certificate
	mbedtls_pk_context *pkey; //The own public key
};

void NewNetwork(Network*);
int ConnectNetwork(Network*, char*, int);
int TLSConnectNetwork(Network *n, const char *addr, const char *port,
                      const char *ca_crt, size_t ca_crt_len,
                      const char *client_crt,	size_t client_crt_len,
                      const char *client_key,	size_t client_key_len,
                      const char *client_pwd, size_t client_pwd_len);
int mqtt_ssl_establish(Network *n, const char *addr, const char *port, const char *ca_crt, size_t ca_crt_len) ;

#endif
