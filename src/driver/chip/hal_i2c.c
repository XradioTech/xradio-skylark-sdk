/**
  * @file  hal_i2c.c
  * @author  XRADIO IOT WLAN Team
  */

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

#ifdef __CONFIG_ROM

#include "driver/chip/hal_i2c.h"
#include "hal_base.h"

extern HAL_Status __HAL_I2C_Init(I2C_ID i2cID, const I2C_InitParam *initParam);
extern int32_t __HAL_I2C_Master_Transmit_Mem_IT(I2C_ID i2cID, uint16_t devAddr, uint32_t memAddr, I2C_MemAddrSize memAddrSize, uint8_t *buf, int32_t size);
extern int32_t __HAL_I2C_Master_Receive_Mem_IT(I2C_ID i2cID, uint16_t devAddr, uint32_t memAddr, I2C_MemAddrSize memAddrSize, uint8_t *buf, int32_t size);

extern I2C_T *gI2CInstance[I2C_NUM];

/* I2C_Private.ctrl */
#define I2C_INIT_STATE_BIT	HAL_BIT(0)
#define I2C_7BIT_ADDR_BIT	HAL_BIT(1)
#define I2C_READ_MODE_BIT	HAL_BIT(2)
#define I2C_SCCB_MODE_BIT	HAL_BIT(3)
#define I2C_MEM_MODE_BIT	HAL_BIT(4)
#define I2C_RESTART_BIT		HAL_BIT(5)

typedef struct {
	uint8_t                 ctrl;

	uint8_t                 memAddr;
	uint16_t                devAddr;
	uint8_t                 memAddrSizeCnt;
	uint8_t                 *buf;
	int32_t                 size;

	HAL_Mutex               mtx;
	HAL_Semaphore           sem;
} I2C_Private;

extern void I2C_SCCBIRQHandler(I2C_T *i2c, I2C_Private *priv);

extern I2C_Private gI2CPrivate[I2C_NUM];

static uint32_t gI2CMemaAddr;

#define I2C_MEM_ADD_SEC(addresses, shift)	\
((uint8_t)(((uint32_t)(addresses) >> (8*shift)) & 0xFF))

static __inline I2C_T *I2C_GetI2CInstance(I2C_ID i2cID)
{
	return gI2CInstance[i2cID];
}

__nonxip_text
static __inline uint8_t I2C_Is7BitAddrMode(I2C_Private *priv)
{
	return !!HAL_GET_BIT(priv->ctrl, I2C_7BIT_ADDR_BIT);
}

__nonxip_text
static __inline uint8_t I2C_Get7BitAddrRd(I2C_Private *priv)
{
	return (uint8_t)((priv->devAddr << 1) | 0x1);
}

__nonxip_text
static __inline uint8_t I2C_Get7BitAddrWr(I2C_Private *priv)
{
	return (uint8_t)((priv->devAddr << 1) | 0x0);
}

__nonxip_text
static uint8_t I2C_Get10BitAddr1Rd(I2C_Private *priv)
{
	uint8_t tmp = (uint8_t)(priv->devAddr >> 7);
	tmp &= ~(0x08U);
	tmp |= 0xF1U;
	return tmp;
}

__nonxip_text
static uint8_t I2C_Get10BitAddr1Wr(I2C_Private *priv)
{
	uint8_t tmp = (uint8_t)(priv->devAddr >> 7);
	tmp &= ~(0x09U);
	tmp |= 0xF0U;
	return tmp;
}

static __inline void I2C_DisableBus(I2C_T *i2c)
{
	HAL_CLR_BIT(i2c->I2C_CTRL, I2C_WR_CTRL_MASK | I2C_BUS_EN_BIT);
}

static __inline void I2C_EnableBus(I2C_T *i2c)
{
	HAL_MODIFY_REG(i2c->I2C_CTRL, I2C_WR_CTRL_MASK, I2C_BUS_EN_BIT);
}

__nonxip_text
static __inline uint32_t I2C_GetIRQStatus(I2C_T *i2c)
{
	return HAL_GET_BIT(i2c->I2C_STATUS, I2C_STATUS_MASK);
}

__nonxip_text
static __inline uint8_t I2C_IsMemMode(I2C_Private *priv)
{
	return !!HAL_GET_BIT(priv->ctrl, I2C_MEM_MODE_BIT);
}

__nonxip_text
static __inline void I2C_SetRestartBit(I2C_Private *priv)
{
	HAL_SET_BIT(priv->ctrl, I2C_RESTART_BIT);
}

__nonxip_text
static __inline uint8_t I2C_IsReadMode(I2C_Private *priv)
{
	return !!HAL_GET_BIT(priv->ctrl, I2C_READ_MODE_BIT);
}

__nonxip_text
static __inline uint8_t I2C_Get10BitAddr2(I2C_Private *priv)
{
	return (uint8_t)(HAL_GET_BIT(priv->devAddr, 0xFFU));
}

__nonxip_text
static __inline uint8_t I2C_GetData(I2C_T *i2c)
{
	return (uint8_t)(HAL_GET_BIT(i2c->I2C_DATA, I2C_DATA_MASK));
}

__nonxip_text
static __inline void I2C_PutData(I2C_T *i2c, uint8_t data)
{
	i2c->I2C_DATA = data;
}

__nonxip_text
static __inline void I2C_ClrIRQFlag(I2C_T *i2c)
{
	HAL_MODIFY_REG(i2c->I2C_CTRL, I2C_WR_CTRL_MASK, I2C_IRQ_FLAG_BIT);
}

__nonxip_text
static __inline uint8_t I2C_GetIRQFlag(I2C_T *i2c)
{
	return !!HAL_GET_BIT(i2c->I2C_CTRL, I2C_IRQ_FLAG_BIT);
}

__nonxip_text
static __inline void I2C_DisableACK(I2C_T *i2c)
{
	HAL_CLR_BIT(i2c->I2C_CTRL, I2C_WR_CTRL_MASK | I2C_ACK_EN_BIT);
}

__nonxip_text
static __inline void I2C_SendStop(I2C_T *i2c)
{
	HAL_MODIFY_REG(i2c->I2C_CTRL, I2C_WR_CTRL_MASK, I2C_STOP_BIT);
}

__nonxip_text
static __inline void I2C_SendStart(I2C_T *i2c)
{
	HAL_MODIFY_REG(i2c->I2C_CTRL, I2C_WR_CTRL_MASK, I2C_START_BIT);
}

__nonxip_text
static __inline uint8_t I2C_IsSCCBMode(I2C_Private *priv)
{
	return !!HAL_GET_BIT(priv->ctrl, I2C_SCCB_MODE_BIT);
}

static __inline void I2C_SetClockReg(I2C_T *i2c, uint8_t clkM, uint8_t clkN)
{
	HAL_MODIFY_REG(i2c->I2C_CLK_CTRL, I2C_CLK_M_MASK | I2C_CLK_N_MASK,
				   (clkM << I2C_CLK_M_SHIFT) | (clkN << I2C_CLK_N_SHIFT));
}

static __inline void I2C_SetClockFreq(I2C_T *i2c, uint32_t clockFreq)
{
	uint8_t	clkM 	= 0;
	uint8_t	clkN 	= 0;
	uint8_t	pow2N	= 1;

	uint32_t	APBClkDiv10;
	uint32_t	div;
	uint32_t	clockReal;

	APBClkDiv10 = HAL_GetAPBClock() / 10;
	div = APBClkDiv10 / clockFreq;
	if (div == 0) {
		I2C_SetClockReg(i2c, clkM, clkN);
		return;
	}

	while (clkN <= I2C_CLK_N_MAX) {
		clkM = div / pow2N - 1;
		while (clkM <= I2C_CLK_M_MAX) {
			clockReal = APBClkDiv10 / pow2N / (clkM + 1);
			if (clockReal <= clockFreq) {
				I2C_SetClockReg(i2c, clkM, clkN);
				return;
			} else {
				clkM++;
			}
		}
		clkN++;
		pow2N *= 2;
	}
}

__nonxip_text
static void I2C_IRQHandler(I2C_T *i2c, I2C_Private *priv)
{
	uint8_t		end = 0;
	uint32_t	IRQStatus = I2C_GetIRQStatus(i2c);

#if (defined(__CONFIG_SECTION_ATTRIBUTE_NONXIP) && HAL_ERR_ON)
	__nonxip_rodata static char __s_func[] = "I2C_IRQHandler";
#endif

	HAL_IT_I2C_DBG("IRQ Status: %#x\n", IRQStatus);

	switch (IRQStatus) {
	case I2C_START_TRAN:
		if (I2C_Is7BitAddrMode(priv)) {
			if ((!I2C_IsMemMode(priv)) && (I2C_IsReadMode(priv)))
				I2C_PutData(i2c, I2C_Get7BitAddrRd(priv));
			else
				I2C_PutData(i2c, I2C_Get7BitAddrWr(priv));
		} else {
			if ((!I2C_IsMemMode(priv)) && (I2C_IsReadMode(priv)))
				I2C_PutData(i2c, I2C_Get10BitAddr1Rd(priv));
			else
				I2C_PutData(i2c, I2C_Get10BitAddr1Wr(priv));
		}
		break;
	case I2C_RE_START_TRAN:
		if (I2C_Is7BitAddrMode(priv))
			I2C_PutData(i2c, I2C_Get7BitAddrRd(priv));
		else
			I2C_PutData(i2c, I2C_Get10BitAddr1Rd(priv));
		break;
	case I2C_ADDR_WR_TRAN_ACK:
		if (I2C_Is7BitAddrMode(priv)) {
			if (I2C_IsMemMode(priv))
				I2C_PutData(i2c, I2C_MEM_ADD_SEC(gI2CMemaAddr, --priv->memAddrSizeCnt));
			else {
				I2C_PutData(i2c, *priv->buf);
				priv->buf++;
				priv->size--;
			}
		} else {
			I2C_PutData(i2c, I2C_Get10BitAddr2(priv));
		}
		break;
	case I2C_ADDR_RD_TRAN_ACK:
		if (!I2C_Is7BitAddrMode(priv))
			I2C_PutData(i2c, I2C_Get10BitAddr2(priv));
		if (priv->size == 1)
			I2C_DisableACK(i2c);
		break;
	case I2C_SEC_ADDR_WR_ACK:
		if (I2C_IsMemMode(priv)) {
			I2C_PutData(i2c, I2C_MEM_ADD_SEC(gI2CMemaAddr, --priv->memAddrSizeCnt));
		} else {
			I2C_PutData(i2c, *priv->buf);
			priv->buf++;
			priv->size--;
		}
		break;
	case I2C_MASTER_DATA_TRAN_ACK:
		if(priv->memAddrSizeCnt > 0)
			I2C_PutData(i2c, I2C_MEM_ADD_SEC(gI2CMemaAddr, --priv->memAddrSizeCnt));
		else {
			if (I2C_IsMemMode(priv) && I2C_IsReadMode(priv)) {
				I2C_SendStart(i2c);
				I2C_SetRestartBit(priv);
			} else {
				if (priv->size > 0) {
					I2C_PutData(i2c, *priv->buf);
					priv->buf++;
					priv->size--;
				} else {
					end = 1;
				}
			}
		}
		break;
	case I2C_MASTER_DATA_RECV_ACK:
		*priv->buf = I2C_GetData(i2c);
		priv->buf++;
		priv->size--;
		if (priv->size == 1)
			I2C_DisableACK(i2c);
		break;
	case I2C_MASTER_DATA_RECV_NACK:
		*priv->buf = I2C_GetData(i2c);
		priv->buf++;
		priv->size--;
		end = 1;
		break;
	case I2C_ADDR_WR_TRAN_NACK:
		HAL_IT_ERR("Invalid IIC address\n");
		end = 1;
		break;
	case I2C_ADDR_RD_TRAN_NACK:
		if (!I2C_IsMemMode(priv))
			HAL_IT_ERR("Invalid IIC address\n");
		else
			HAL_IT_ERR("No ACK received after 2nd-address-send\n");
		end = 1;
		break;
	case I2C_MASTER_DATA_TRAN_NACK:
		HAL_IT_ERR("In writing, no ACK received\n");
		end = 1;
		break;
	default:
		end = 1;
		break;
	}

	if (end) {
		I2C_SendStop(i2c);
		HAL_SemaphoreRelease(&priv->sem);
	}

	I2C_ClrIRQFlag(i2c);
	while (I2C_GetIRQFlag(i2c))
		;

	return;
}

__nonxip_text
static void I2C0_IRQHandler(void)
{
	if (I2C_IsSCCBMode(&gI2CPrivate[I2C0_ID]))
		I2C_SCCBIRQHandler(I2C0, &gI2CPrivate[I2C0_ID]);
	else
		I2C_IRQHandler(I2C0, &gI2CPrivate[I2C0_ID]);
}

__nonxip_text
static void I2C1_IRQHandler(void)
{
	if (I2C_IsSCCBMode(&gI2CPrivate[I2C1_ID]))
		I2C_SCCBIRQHandler(I2C1, &gI2CPrivate[I2C1_ID]);
	else
		I2C_IRQHandler(I2C1, &gI2CPrivate[I2C1_ID]);
}

/**
 * @brief Initialize the I2C according to the specified parameters
 * @param[in] i2cID ID of the specified I2C
 * @param[in] initParam Pointer to I2C_InitParam structure
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_I2C_Init(I2C_ID i2cID, const I2C_InitParam *initParam)
{
#ifdef __CONFIG_CPU_SUPPORT_349MHZ
	I2C_T *i2c;
#endif
	HAL_Status status;
	IRQn_Type IRQn;
	NVIC_IRQHandler IRQHandler;

	status = __HAL_I2C_Init(i2cID, initParam);

#ifdef __CONFIG_CPU_SUPPORT_349MHZ
	if ((status == HAL_OK) &&
	    ((CCM->CPU_BUS_CLKCFG & CCM_APB_CLK_SRC_MASK) == CCM_APB_CLK_SRC_AHB2CLK) &&
	    (HAL_GET_BIT(PRCM->SYS_CLK1_CTRL, PRCM_SYS_CLK_FACTORM_MASK) == PRCM_SYS_CLK_FACTORM_5_5)) {
		i2c = I2C_GetI2CInstance(i2cID);
		I2C_DisableBus(i2c);
		I2C_SetClockFreq(i2c, initParam->clockFreq);
		I2C_EnableBus(i2c);
	}
#endif

	/* enable NVIC IRQ */
	if (i2cID == I2C0_ID) {
		IRQn = I2C0_IRQn;
		IRQHandler = I2C0_IRQHandler;
	} else {
		IRQn = I2C1_IRQn;
		IRQHandler = I2C1_IRQHandler;
	}
	HAL_NVIC_ConfigExtIRQ(IRQn, IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);

	return status;
}

int32_t HAL_I2C_Master_Transmit_Mem_IT(I2C_ID i2cID, uint16_t devAddr, uint32_t memAddr, I2C_MemAddrSize memAddrSize, uint8_t *buf, int32_t size)
{
	gI2CMemaAddr = memAddr;
	return __HAL_I2C_Master_Transmit_Mem_IT(i2cID, devAddr, memAddr, memAddrSize, buf, size);
}

int32_t HAL_I2C_Master_Receive_Mem_IT(I2C_ID i2cID, uint16_t devAddr, uint32_t memAddr, I2C_MemAddrSize memAddrSize, uint8_t *buf, int32_t size)
{
	gI2CMemaAddr = memAddr;
	return __HAL_I2C_Master_Receive_Mem_IT(i2cID, devAddr, memAddr, memAddrSize, buf, size);
}

#endif /* __CONFIG_ROM */
