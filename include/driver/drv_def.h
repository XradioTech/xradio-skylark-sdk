#ifndef __DRV_DEF_H_
#define __DRV_DEF_H_

#define DRV_ALE 0
#define DRV_ERR 1
#define DRV_INF 2
#define DRV_DBG 3

/**
 * @brief Drv Status value
 */
typedef enum
{
    DRV_OK      = 0,	/* success */
    DRV_ERROR   = -1,	/* general error */
    DRV_BUSY    = -2,	/* device or resource busy */
    DRV_TIMEOUT = -3,	/* wait timeout */
    DRV_INVALID = -4	/* invalid argument */
} DRV_Status;

#endif/*__DRV_DEF_H_*/

