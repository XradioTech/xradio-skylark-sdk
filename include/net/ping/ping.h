#ifndef PING_H
#define PING_H

#include "lwip/inet.h"
#include "lwip/ip_addr.h"

#ifdef __cplusplus
extern "C" {//}
#endif

struct ping_data {
	ip_addr_t sin_addr; /* server addr */

	u32_t count;		/* number of ping */
	u32_t data_long;	/* the ping packet data long */
	u32_t interval;		/* Wait interval seconds between sending each packet, default 1s */
	u32_t timeout;		/* Time to wait for a response, in seconds */
	u32_t deadline;		/* Specify a timeout, in seconds, ping thread will stop if timeout */
	u32_t ttl;			/* ttl ping only. Set the IP Time to Live. */

	int run_flag;       /* run flag, 0:stop 1:start */
};

s32_t ping(struct ping_data *data);

#ifdef __cplusplus
}
#endif

#endif
