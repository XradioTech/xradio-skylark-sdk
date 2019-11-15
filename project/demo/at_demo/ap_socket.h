#ifndef __AP_SOCKET_H
#define __AP_SOCKET_H

#include "stdio.h"

#define AP_SOCKET_DBG_ON				1
#define AP_SOCKET_WRN_ON				1
#define AP_SOCKET_ERR_ON				1

#define AP_SOCKET_SYSLOG				printf

#define AP_SOCKET_LOG(flags, fmt, arg...)	\
    do {								\
        if (flags)						\
            AP_SOCKET_SYSLOG(fmt, ##arg);	\
    } while (0)

#define AP_SOCKET_DBG(fmt, arg...)	\
    AP_SOCKET_LOG(AP_SOCKET_DBG_ON, "[AP_SOCKET DBG] "fmt, ##arg)

#define AP_SOCKET_WRN(fmt, arg...)	\
    AP_SOCKET_LOG(AP_SOCKET_WRN_ON, "[AP_SOCKET WRN] "fmt, ##arg)

#define AP_SOCKET_ERR(fmt, arg...)								\
    do {														\
		AP_SOCKET_LOG(AP_SOCKET_ERR_ON, "[AP_SOCKET ERR] %s():%d, "fmt,	\
               __func__, __LINE__, ##arg); \
    } while (0)


void ap_socket_task(int port);

#endif
