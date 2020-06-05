/**
  * @file  hal_flashctrl.c
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

#include <stdbool.h>
#include "hal_base.h"
#include "rom/driver/chip/hal_flashctrl.h"
#include "rom/driver/chip/hal_icache.h"
#include "rom/driver/chip/hal_dma.h"
#include "rom/pm/pm.h"
#include "flashchip/flash_debug.h"

#ifdef CONFIG_PM
#define FLASHC_TEMP_FIXED
#endif
static inline int FC_Sbus_GetDebugState(void);

#if 1 //(FC_DEBUG_ON == DBG_ON)
#define FC_WHILE_TIMEOUT(cond, i)	\
	i = 0x3FFFFFF;	\
	do	\
	{	\
		if (--i == 0) {	\
			FC_Reg_All();	\
			return HAL_ERROR;	\
		}	\
	} while(cond)
#else
#define FC_WHILE_TIMEOUT(cond, i) \
	(void)i; 	\
	while(cond)
#endif

uint8_t fc_debug_mask = FC_ERR_FLAG;

void HAL_Flashc_SetDbgMask(uint8_t dbg_mask)
{
	fc_debug_mask = dbg_mask;
}

typedef enum FC_Sbus_RW {
	FC_SBUS_READ,
	FC_SBUS_WRITE,
} FC_Sbus_RW;

#if 0 //(FC_DEBUG_ON == DBG_ON) // not robust implemetation, only for debug
#define FC_DebugCheck(state) __FC_DebugCheck(state, __LINE__)
static int __FC_DebugCheck(int state, uint32_t line)
{
	int debug = FC_Sbus_GetDebugState();
	if (debug != state) {
		HAL_UDelay(5000);
		debug = FC_Sbus_GetDebugState();
		if (debug != state) {
			FC_ERROR("line: %d, err stat: 0x%x", line, state);
			Regs_Print();
			return -1;
		}
	}
	return 0;
}
#else
#define FC_DebugCheck(state) __FC_DebugCheck(state)
static inline int __FC_DebugCheck(int state)
{
	while(FC_Sbus_GetDebugState() != state);
	return 0;
}
#endif

#if FLASHCTRL_ON_XR875
#define FC_REG_ALL() \
    { \
        FC_DEBUG("QPI reg: base addr             0x%8x", (uint32_t)&(OPI_MEM_CTRL->MEM_COM_CONFG)); \
        FC_DEBUG("MEM_COM_CONFG:                 0x%8x", OPI_MEM_CTRL->MEM_COM_CONFG);\
        FC_DEBUG("OPI_CTRL_CMM_CONFG:            0x%8x", OPI_MEM_CTRL->OPI_CTRL_CMM_CONFG);\
        FC_DEBUG("CACHE_RLVT_CONFG:              0x%8x", OPI_MEM_CTRL->CACHE_RLVT_CONFG);\
        FC_DEBUG("MEM_AC_CHR_TIMING_CONFG:       0x%8x", OPI_MEM_CTRL->MEM_AC_CHR_TIMING_CONFG);\
        FC_DEBUG("CBUS_RD_OPRT_CONFG:            0x%8x", OPI_MEM_CTRL->CBUS_RD_OPRT_CONFG);\
        FC_DEBUG("CBUS_WR_OPRT_CONFG:            0x%8x", OPI_MEM_CTRL->CBUS_WR_OPRT_CONFG);\
        FC_DEBUG("CBUS_RD_DUMMY_DATA_TOP_HALF:   0x%8x", OPI_MEM_CTRL->CBUS_RD_DUMMY_DATA_TOP_HALF);\
        FC_DEBUG("CBUS_RD_DUMMY_DATA_BUTT_HALF:  0x%8x", OPI_MEM_CTRL->CBUS_RD_DUMMY_DATA_BUTT_HALF);\
        FC_DEBUG("CBUS_WR_DUMMY_DATA_TOP_HALF:   0x%8x", OPI_MEM_CTRL->CBUS_WR_DUMMY_DATA_TOP_HALF);\
        FC_DEBUG("CBUS_WR_DUMMY_DATA_BUTT_HALF:  0x%8x", OPI_MEM_CTRL->CBUS_WR_DUMMY_DATA_BUTT_HALF);\
        FC_DEBUG("CBUS_IO_SW_WAIT_TIME:          0x%8x", OPI_MEM_CTRL->CBUS_IO_SW_WAIT_TIME);\
        FC_DEBUG("SBUS_RW_OPRT_CONFG:            0x%8x", OPI_MEM_CTRL->SBUS_RW_OPRT_CONFG);\
        FC_DEBUG("SBUS_ADDR_CONFG:               0x%8x", OPI_MEM_CTRL->SBUS_ADDR_CONFG);\
        FC_DEBUG("SBUS_DUMMY_DATA_TOP_HALF:      0x%8x", OPI_MEM_CTRL->SBUS_DUMMY_DATA_TOP_HALF);\
        FC_DEBUG("SBUS_DUMMY_DATA_BUTT_HALF:     0x%8x", OPI_MEM_CTRL->SBUS_DUMMY_DATA_BUTT_HALF);\
        FC_DEBUG("SBUS_IO_SW_WAIT_TIME:          0x%8x", OPI_MEM_CTRL->SBUS_IO_SW_WAIT_TIME);\
        FC_DEBUG("SBUS_WR_DATA_BYTE_NUM:         0x%8x", OPI_MEM_CTRL->SBUS_WR_DATA_BYTE_NUM);\
        FC_DEBUG("SBUS_RD_DATA_BYTE_NUM:         0x%8x", OPI_MEM_CTRL->SBUS_RD_DATA_BYTE_NUM);\
        FC_DEBUG("SBUS_START_SEND_REG:           0x%8x", OPI_MEM_CTRL->SBUS_START_SEND_REG);\
        FC_DEBUG("FIFO_TRIGGER_LEVEL:            0x%8x", OPI_MEM_CTRL->FIFO_TRIGGER_LEVEL);\
        FC_DEBUG("FIFO_STATUS_REG:               0x%8x", OPI_MEM_CTRL->FIFO_STATUS_REG);\
        FC_DEBUG("INT_ENABLE_REG:                0x%8x", OPI_MEM_CTRL->INT_ENABLE_REG);\
        FC_DEBUG("INT_STATUS_REG:                0x%8x", OPI_MEM_CTRL->INT_STATUS_REG);\
        FC_DEBUG("XIP_WARP_MODE_EXE_IDCT:        0x%8x", OPI_MEM_CTRL->XIP_WARP_MODE_EXE_IDCT);\
        FC_DEBUG("MEM_CTRL_DEBUG_STATE:          0x%8x", OPI_MEM_CTRL->MEM_CTRL_DEBUG_STATE);\
        FC_DEBUG("DEBUG_CNT_SBUS:                0x%8x", OPI_MEM_CTRL->DEBUG_CNT_SBUS);\
        FC_DEBUG("DEBUG_CNT_CBUS:                0x%8x", OPI_MEM_CTRL->DEBUG_CNT_CBUS);\
        FC_DEBUG("PSRAM_COM_CFG:                 0x%8x", OPI_MEM_CTRL->PSRAM_COM_CFG);\
        FC_DEBUG("PSRAM_LAT_CFG:                 0x%8x", OPI_MEM_CTRL->PSRAM_LAT_CFG);\
        FC_DEBUG("PSRAM_TIM_CFG:                 0x%8x", OPI_MEM_CTRL->PSRAM_TIM_CFG);\
        FC_DEBUG("START_ADDR0:                   0x%8x", OPI_MEM_CTRL->START_ADDR0);\
        FC_DEBUG("END_ADDR0:                     0x%8x", OPI_MEM_CTRL->END_ADDR0);\
        FC_DEBUG("BIAS_ADDR0:                    0x%8x", OPI_MEM_CTRL->BIAS_ADDR0);\
        FC_DEBUG("SBUS_WR_DATA_REG:              0x%8x", OPI_MEM_CTRL->SBUS_WR_DATA_REG);\
        FC_DEBUG("SBUS_RD_DATA_REG:              0x%8x", OPI_MEM_CTRL->SBUS_RD_DATA_REG);\
    }



static inline void FC_Reg_All()
{
	//FC_REG_ALL();
}

static inline void FC_Enable_32BitAddr_Mode(void)
{
	HAL_SET_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, FLASHC_ADDR_SIZE_MODE_MASK);
}

void FC_Ibus_ReadConfig(uint8_t read_cmd,
                        FC_CycleBits cmd,
                        FC_CycleBits addr,
                        FC_CycleBits dummy,
                        FC_CycleBits data,
                        uint8_t dummy_byte)
{
	uint8_t dummy_width;

	if (dummy_byte > 8)
		dummy_byte = 8;
	dummy_width = dummy_byte * 8;

	HAL_MODIFY_REG(OPI_MEM_CTRL->CBUS_RD_OPRT_CONFG,
	               FLASHC_RD_CMD_SEND_MASK
	               | FLASHC_SEND_BIT_CMD_EVERY_CYCLE_MASK
	               | FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_MASK
	               | FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_MASK
	               | FLASHC_SEND_BYTE_DUMMY_NUM_MASK
	               | FLASHC_GET_BIT_DATA_EVERY_CYCLE_MASK,
	               (read_cmd << FLASHC_RD_CMD_SEND_SHIFT)
	               | (cmd << FLASHC_SEND_BIT_CMD_EVERY_CYCLE_SHIFT)
	               | (addr << FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_SHIFT)
	               | (dummy << FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_SHIFT)
	               | (dummy_width << FLASHC_SEND_BYTE_DUMMY_NUM_SHIFT)
	               | (data << FLASHC_GET_BIT_DATA_EVERY_CYCLE_SHIFT));

}


void FC_Ibus_DummyData(uint32_t dummyh, uint32_t dummyl)
{
	OPI_MEM_CTRL->CBUS_RD_DUMMY_DATA_TOP_HALF = dummyh;
	OPI_MEM_CTRL->CBUS_RD_DUMMY_DATA_BUTT_HALF = dummyl;
}

void FC_Ibus_TransmitDelay(Flash_Ctrl_DelayCycle *delay)
{
	HAL_MODIFY_REG(OPI_MEM_CTRL->MEM_AC_CHR_TIMING_CONFG,
	               FLASHC_CBUS_SHSL_MASK
	               | FLASHC_SPI_FLASH_CHSH_MASK
	               | FLASHC_SPI_FLASH_SLCH_MASK,
	               (delay->cs_begin << FLASHC_SPI_FLASH_SLCH_SHIFT)
	               | (delay->cs_over << FLASHC_SPI_FLASH_CHSH_SHIFT)
	               | (delay->cs_deselect << FLASHC_CBUS_SHSL_SHIFT));
	HAL_MODIFY_REG(OPI_MEM_CTRL->CBUS_IO_SW_WAIT_TIME,
	               FLASHC_CBUS_CMD_WAIT_CYCLE_MASK
	               | FLASHC_CBUS_ADDR_WAIT_CYCLE_MASK
	               | FLASHC_CBUS_DUMMY_WAIT_CYCLE_MASK,
	               (delay->cmd_over << FLASHC_CBUS_CMD_WAIT_CYCLE_SHIFT)
	               | (delay->addr_over << FLASHC_CBUS_ADDR_WAIT_CYCLE_SHIFT)
	               | (delay->dummy_over << FLASHC_CBUS_DUMMY_WAIT_CYCLE_SHIFT));
	HAL_MODIFY_REG(OPI_MEM_CTRL->OPI_CTRL_CMM_CONFG,
	               FLASHC_WAIT_HALF_CYCLE_MASK,
	               delay->data << FLASHC_WAIT_HALF_CYCLE_SHIFT);
}

void FC_SetFlash(FC_Cs cs, FC_TCTRL_Fbs fbs, FC_Sclk_Mode mode)
{
	HAL_MODIFY_REG(OPI_MEM_CTRL->OPI_CTRL_CMM_CONFG,
	               FLASHC_FLASH_CS_POL_MASK
	               | FLASHC_FIRST_RCV_BIT_SLT_MASK
	               | FLASHC_SPI_CPOL_CTRL_MASK
	               | FLASHC_SPI_CPHA_CTRL_MASK,
	               cs | fbs | mode);
}

void FC_Sbus_ResetFIFO(bool tx, bool rx)
{
	HAL_MODIFY_REG(OPI_MEM_CTRL->MEM_COM_CONFG,
	               FLASHC_REV_FIFO_RESET_MASK | FLASHC_TRAN_FIFO_RESET_MASK,
	               (tx << FLASHC_TRAN_FIFO_RESET_SHIFT) | (rx << FLASHC_REV_FIFO_RESET_SHIFT));
}

static inline void FC_WrapMode(bool enable)
{
	HAL_MODIFY_REG(OPI_MEM_CTRL->MEM_COM_CONFG, FLASHC_WRAP_AROUND_ENABLE_MASK, enable << FLASHC_WRAP_AROUND_ENABLE_SHIFT);
}

static inline void FC_Sbus_CommandConfig(FC_CycleBits cmd,
                FC_CycleBits addr,
                FC_CycleBits dummy,
                FC_CycleBits data,
                uint8_t dummy_byte)
{
	uint8_t dummy_width;

	if (dummy_byte > 8)
		dummy_byte = 8;
	dummy_width = dummy_byte * 8;

	HAL_MODIFY_REG(OPI_MEM_CTRL->SBUS_RW_OPRT_CONFG,
	               FLASHC_SEND_BIT_CMD_EVERY_CYCLE_MASK
	               | FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_MASK
	               | FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_MASK
	               | FLASHC_SEND_BYTE_DUMMY_MASK
	               | FLASHC_GS_BIT_EVERY_CYCLE_MASK,
	               (cmd << FLASHC_SEND_BIT_CMD_EVERY_CYCLE_SHIFT)
	               | (addr << FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_SHIFT)
	               | (dummy << FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_SHIFT)
	               | (dummy_width << FLASHC_SEND_BYTE_DUMMY_SHIFT)
	               | (data << FLASHC_GS_BIT_EVERY_CYCLE_SHIFT));
}

static inline void FC_Sbus_Command(uint8_t cmd, uint32_t addr, uint32_t dummyh, uint32_t dummyl)
{
	HAL_MODIFY_REG(OPI_MEM_CTRL->SBUS_RW_OPRT_CONFG, FLASHC_RW_CMD_SEND_MASK, cmd << FLASHC_RW_CMD_SEND_SHIFT);
	HAL_MODIFY_REG(OPI_MEM_CTRL->SBUS_ADDR_CONFG, FLASHC_SBUS_ADDR_MASK, addr << FLASHC_SBUS_ADDR_SHIFT);
	HAL_MODIFY_REG(OPI_MEM_CTRL->SBUS_DUMMY_DATA_TOP_HALF, FLASHC_SBUS_DUMMY_TOP_HALF_MASK, dummyh << FLASHC_SBUS_DUMMY_TOP_HALF_SHIFT);
	HAL_MODIFY_REG(OPI_MEM_CTRL->SBUS_DUMMY_DATA_BUTT_HALF, FLASHC_SBUS_DUMMY_BUTTOM_HALF_MASK, dummyl << FLASHC_SBUS_DUMMY_BUTTOM_HALF_SHIFT);
}

static inline void FC_Sbus_WriteSize(uint16_t size)
{
	if (size & (~0x1FF))
		FC_ERROR("wr size error");
	size &= 0x1FF;
	HAL_MODIFY_REG(OPI_MEM_CTRL->SBUS_WR_DATA_BYTE_NUM, FLASHC_SBUS_WR_BYTE_MASK, size << FLASHC_SBUS_WR_BYTE_SHIFT);
}

static inline void FC_Sbus_ReadSize(uint32_t size)
{
	HAL_MODIFY_REG(OPI_MEM_CTRL->SBUS_RD_DATA_BYTE_NUM, FLASHC_SBUS_RD_BYTE_MASK, size << FLASHC_SBUS_RD_BYTE_SHIFT);
}

void FC_Sbus_TransmitDelay(Flash_Ctrl_DelayCycle *delay)
{
	HAL_MODIFY_REG(OPI_MEM_CTRL->MEM_AC_CHR_TIMING_CONFG,
	               FLASHC_SPI_FLASH_CHSH_MASK
	               | FLASHC_SPI_FLASH_SLCH_MASK,
	               (delay->cs_begin << FLASHC_SPI_FLASH_SLCH_SHIFT)
	               | (delay->cs_over << FLASHC_SPI_FLASH_CHSH_SHIFT));

	HAL_MODIFY_REG(OPI_MEM_CTRL->MEM_AC_CHR_TIMING_CONFG,
	               FLASHC_SBUS_SHSL_MASK,
	               delay->cs_deselect << FLASHC_SBUS_SHSL_SHIFT);

	HAL_MODIFY_REG(OPI_MEM_CTRL->SBUS_IO_SW_WAIT_TIME,
	               FLASHC_SBUS_CMD_WAIT_CYCLE_MASK
	               | FLASHC_SBUS_ADDR_WAIT_CYCLE_MASK
	               | FLASHC_SBUS_DUMMY_WAIT_CYCLE_MASK,
	               (delay->cmd_over << FLASHC_SBUS_CMD_WAIT_CYCLE_SHIFT)
	               | (delay->addr_over << FLASHC_SBUS_ADDR_WAIT_CYCLE_SHIFT)
	               | (delay->dummy_over << FLASHC_SBUS_DUMMY_WAIT_CYCLE_SHIFT));

	HAL_MODIFY_REG(OPI_MEM_CTRL->OPI_CTRL_CMM_CONFG,
	               FLASHC_WAIT_HALF_CYCLE_MASK,
	               delay->data << FLASHC_WAIT_HALF_CYCLE_SHIFT);
}

static inline void FC_Sbus_FIFOTriggerLevel(uint8_t txfull, uint8_t txempty, uint8_t rxfull, uint8_t rxempty)
{
	HAL_MODIFY_REG(OPI_MEM_CTRL->FIFO_TRIGGER_LEVEL,
	               FLASHC_RD_FIFO_EMPTY_REQ_MASK
	               | FLASHC_RD_FIFO_FULL_REQ_MASK
	               | FLASHC_WR_FIFO_EMPTY_REQ_MASK
	               | FLASHC_WR_FIFO_FULL_REQ_MASK,
	               (txfull << FLASHC_WR_FIFO_FULL_REQ_SHIFT)
	               | (txempty << FLASHC_WR_FIFO_EMPTY_REQ_SHIFT)
	               | (rxfull << FLASHC_RD_FIFO_FULL_REQ_SHIFT)
	               | (rxempty << FLASHC_RD_FIFO_EMPTY_REQ_SHIFT));
}


bool FC_Sbus_IsAvailable(FC_Sbus_RW rw)
{
	if (rw == FC_SBUS_WRITE)
		return !!HAL_GET_BIT(OPI_MEM_CTRL->FIFO_STATUS_REG, FLASHC_WR_BUFF_RD_STATUS_SHIFT);
	else
		return !!HAL_GET_BIT(OPI_MEM_CTRL->FIFO_STATUS_REG, FLASHC_RD_BUFF_RD_STATUS_SHIFT);
}

int FC_Sbus_GetBufCnt(FC_Sbus_RW rw)
{
	if (rw == FC_SBUS_WRITE)
		return HAL_GET_BIT_VAL(OPI_MEM_CTRL->FIFO_STATUS_REG, FLASHC_WR_BUFF_COUNTER_SHIFT, FLASHC_WR_BUFF_COUNTER_VMASK);
	else
		return HAL_GET_BIT_VAL(OPI_MEM_CTRL->FIFO_STATUS_REG, FLASHC_RD_BUFF_COUNTER_SHIFT, FLASHC_RD_BUFF_COUNTER_VMASK);
}

static int FC_Sbus_GetFIFOCnt(FC_Sbus_RW rw)
{
	if (rw == FC_SBUS_WRITE)
		return HAL_GET_BIT_VAL(OPI_MEM_CTRL->FIFO_STATUS_REG, FLASHC_RD_FIFO_COUNTER_SHIFT, FLASHC_RD_FIFO_COUNTER_VMASK);
	else
		return HAL_GET_BIT_VAL(OPI_MEM_CTRL->FIFO_STATUS_REG, FLASHC_RD_FIFO_COUNTER_SHIFT, FLASHC_RD_FIFO_COUNTER_VMASK);
}

/*
	Debug State:
	0x0 Idle or Sbus complete;
	0x2 Send CMD;
	0x4 Send Address;
	0x6 Send Dummy;
	0x8 Send Data;
	0x9 Get Data;
	0xc Ibus read complete;
*/
static inline int FC_Sbus_GetDebugState(void)
{
	return HAL_GET_BIT_VAL(OPI_MEM_CTRL->MEM_CTRL_DEBUG_STATE, FLASHC_MEM_CTRL_STATUE_DEBUG_SHIFT, FLASHC_MEM_CTRL_STATUE_DEBUG_VMASK);
}


static inline void FC_Sbus_StartSend(void)
{
	HAL_SET_BIT(OPI_MEM_CTRL->SBUS_START_SEND_REG, FLASHC_ENABLE_SBUS_MASK);
}

static inline bool FC_Sbus_isSending(void)
{
	return !!HAL_GET_BIT(OPI_MEM_CTRL->SBUS_START_SEND_REG, FLASHC_ENABLE_SBUS_MASK);
}

typedef enum FC_Sbus_IntType {
	FC_INT_HREADY_TIMEOUT           = 1 << FLASHC_SBUS_HREADY_TIME_OUT_INT_FLAG_SHIFT,
	FC_INT_TC                       = 1 << FLASHC_TRAN_COMPLETED_INT_FLAG_SHIFT,
	FC_INT_WR_FIFO_UNDERFLOW        = 1 << FLASHC_WR_FIFO_UNDERFLOW_INT_FLAG_SHIFT,
	FC_INT_WR_FIFO_OVERFLOW         = 1 << FLASHC_WR_FIFO_OVERFLOW_INT_FLAG_SHIFT,
	FC_INT_RD_FIFO_UNDERFLOW        = 1 << FLASHC_RD_FIFO_UNDERFLOW_INT_FLAG_SHIFT,
	FC_INT_RD_FIFO_OVERFLOW         = 1 << FLASHC_RD_FIFO_OVERFLOW_INT_FLAG_SHIFT,
	FC_INT_WR_FIFO_FULL             = 1 << FLASHC_WR_FIFO_FULL_INT_FLAG_SHIFT,
	FC_INT_WR_FIFO_EMPTY            = 1 << FLASHC_WR_FIFO_EMPTY_INT_FLAG_SHIFT,
	FC_INT_WR_FIFO_READY            = 1 << FLASHC_WR_FIFO_REQ_INT_FLAG_SHIFT,
	FC_INT_RD_FIFO_FULL             = 1 << FLASHC_RD_FIFO_FULL_INT_FLAG_SHIFT,
	FC_INT_RD_FIFO_EMPTY            = 1 << FLASHC_RD_FIFO_EMPTY_INT_FLAG_SHIFT,
	FC_INT_RD_FIFO_READY            = 1 << FLASHC_RD_FIFO_REQ_INT_FLAG_SHIFT,
} FC_Sbus_IntType;

static inline void FC_Sbus_ClrStatus(FC_Sbus_IntType type)
{
	HAL_CLR_BIT(OPI_MEM_CTRL->INT_STATUS_REG, type);
}

static inline void FC_Sbus_Write(uint8_t data)
{
	*((uint8_t *)&OPI_MEM_CTRL->SBUS_WR_DATA_REG) = data;
}

static inline uint8_t FC_Sbus_Read(void)
{
	return *((uint8_t *)&OPI_MEM_CTRL->SBUS_RD_DATA_REG);
}

#else /* FLASHCTRL_ON_XR875 */
#define FC_REG_ALL() \
	{ \
		FC_DEBUG("fc reg: base addr 0x%x", (uint32_t)&(FLASH_CTRL->COMMON_CFG)); \
		FC_DEBUG("COMMON_CFG" ":0x%x.", FLASH_CTRL->COMMON_CFG);\
		FC_DEBUG("I_CMD_CFG" ":0x%x.", FLASH_CTRL->I_CMD_CFG);\
		FC_DEBUG("I_DUMMY_H" ":0x%x.", FLASH_CTRL->I_DUMMY_H);\
		FC_DEBUG("I_DUMMY_L" ":0x%x.", FLASH_CTRL->I_DUMMY_L);\
		FC_DEBUG("I_CS_WAIT" ":0x%x.", FLASH_CTRL->I_CS_WAIT);\
		FC_DEBUG("I_IO_WAIT" ":0x%x.", FLASH_CTRL->I_IO_WAIT);\
		FC_DEBUG("RESERVE0[0]" ":0x%x.", FLASH_CTRL->RESERVE0[0]);\
		FC_DEBUG("FLASH_COMMON_CFG" ":0x%x.", FLASH_CTRL->FLASH_COMMON_CFG);\
		FC_DEBUG("XIP_EXEC" ":0x%x.", FLASH_CTRL->XIP_EXEC);\
		FC_DEBUG("COMMON_ADD_CFG" ":0x%x.", FLASH_CTRL->COMMON_ADD_CFG);\
		FC_DEBUG("S_CMD_CFG" ":0x%x.", FLASH_CTRL->S_CMD_CFG);\
		FC_DEBUG("S_ADDR_CFG" ":0x%x.", FLASH_CTRL->S_ADDR_CFG);\
		FC_DEBUG("S_WR_NUM" ":0x%x.", FLASH_CTRL->S_WR_NUM);\
		FC_DEBUG("S_RD_NUM" ":0x%x.", FLASH_CTRL->S_RD_NUM);\
		FC_DEBUG("S_DUMMY_H" ":0x%x.", FLASH_CTRL->S_DUMMY_H);\
		FC_DEBUG("S_DUMMY_L" ":0x%x.", FLASH_CTRL->S_DUMMY_L);\
		FC_DEBUG("FIFO_TRIG_LEVEL" ":0x%x.", FLASH_CTRL->FIFO_TRIG_LEVEL);\
		FC_DEBUG("FIFO_STATUS" ":0x%x.", FLASH_CTRL->FIFO_STATUS);\
		FC_DEBUG("S_IO_WAIT" ":0x%x.", FLASH_CTRL->S_IO_WAIT);\
		FC_DEBUG("WRAP_MODE" ":0x%x.", FLASH_CTRL->WRAP_MODE);\
		FC_DEBUG("START_SEND" ":0x%x.", FLASH_CTRL->START_SEND);\
		FC_DEBUG("INT_EN" ":0x%x.", FLASH_CTRL->INT_EN);\
		FC_DEBUG("INT_STA" ":0x%x.", FLASH_CTRL->INT_STA);\
	}
static inline void FC_Reg_All()
{
	//FC_REG_ALL();
}


void FC_Ibus_ReadConfig(uint8_t read_cmd,
                        FC_CycleBits cmd,
                        FC_CycleBits addr,
                        FC_CycleBits dummy,
                        FC_CycleBits data,
                        uint8_t dummy_byte)
{
	uint8_t dummy_width;

	if (dummy_byte > 8)
		dummy_byte = 8;
	dummy_width = dummy_byte * 8;

	HAL_MODIFY_REG(FLASH_CTRL->I_CMD_CFG,
	               FC_ICC_CMD_MASK
	               | FC_ICC_CMD_BIT_MASK
	               | FC_ICC_ADDR_BIT_MASK
	               | FC_ICC_DUMMY_BIT_MASK
	               | FC_ICC_DATA_BIT_MASK
	               | FC_ICC_DUMMY_WIDTH_MASK,
	               (read_cmd << FC_ICC_CMD_SHIFT)
	               | (cmd << FC_ICC_CMD_BIT_SHIFT)
	               | (addr << FC_ICC_ADDR_BIT_SHIFT)
	               | (dummy << FC_ICC_DUMMY_BIT_SHIFT)
	               | (data << FC_ICC_DATA_BIT_SHIFT)
	               | (dummy_width << FC_ICC_DUMMY_WIDTH_SHIFT));
}

void FC_Ibus_Enable(uint32_t flash_ctrl_en)
{
	HAL_SET_BIT(FLASH_CTRL->COMMON_CFG, flash_ctrl_en);
}

void FC_Ibus_Disable(uint32_t flash_ctrl_en)
{
	HAL_CLR_BIT(FLASH_CTRL->COMMON_CFG, flash_ctrl_en);
}

static inline bool FC_Ibus_IsXIP()
{
	return HAL_GET_BIT(FLASH_CTRL->XIP_EXEC, FC_XE_MASK);
}

/*typedef enum {
	FLASH_CTRL_READMODE_NORMAL_IO = () | () | (),
	FLASH_CTRL_READMODE_DUAL_OUTPUT,
	FLASH_CTRL_READMODE_DUAL_IO,
	FLASH_CTRL_READMODE_QUAD_OUTPUT,
	FLASH_CTRL_READMODE_QUAD_IO
} Flash_Ctrl_ReadMode;*/

static inline uint32_t FC_DefOutput(uint8_t io_num, FC_Io_Output io)
{
	uint32_t mask, shift;

	if (io_num == 1) {
		mask = FC_CC_IO1_MASK;
		shift = FC_CC_IO1_SHIFT;
	} else if (io_num == 2) {
		mask = FC_CC_IO2_MASK;
		shift = FC_CC_IO2_SHIFT;
	} else if (io_num == 3) {
		mask = FC_CC_IO3_MASK;
		shift = FC_CC_IO3_SHIFT;
	} else
		return -1;

	HAL_MODIFY_REG(FLASH_CTRL->COMMON_CFG, mask, io << shift);

	return 0;
}

void FC_Ibus_DummyData(uint32_t dummyh, uint32_t dummyl)
{
	FLASH_CTRL->I_DUMMY_H = dummyh;
	FLASH_CTRL->I_DUMMY_L = dummyl;
}

void FC_Ibus_TransmitDelay(Flash_Ctrl_DelayCycle *delay)
{
	HAL_MODIFY_REG(FLASH_CTRL->I_CS_WAIT,
	               FC_ICW_BEGIN_MASK
	               | FC_ICW_OVER_MASK
	               | FC_ICW_DESEL_MASK,
	               (delay->cs_begin << FC_ICW_BEGIN_SHIFT)
	               | (delay->cs_over << FC_ICW_OVER_SHIFT)
	               | (delay->cs_deselect << FC_ICW_DESEL_SHIFT));
	HAL_MODIFY_REG(FLASH_CTRL->I_IO_WAIT,
	               FC_IIW_CMD_MASK
	               | FC_IIW_ADDR_MASK
	               | FC_IIW_DUM_MASK,
	               (delay->cmd_over << FC_IIW_CMD_SHIFT)
	               | (delay->addr_over << FC_IIW_ADDR_SHIFT)
	               | (delay->dummy_over << FC_IIW_DUM_SHIFT));
	HAL_MODIFY_REG(FLASH_CTRL->FLASH_COMMON_CFG,
	               FC_FCC_WAIT_DATA_MASK,
	               delay->data << FC_FCC_WAIT_DATA_SHIFT);
}

void FC_SetFlash(FC_Cs cs, FC_TCTRL_Fbs fbs, FC_Sclk_Mode mode)
{
	HAL_MODIFY_REG(FLASH_CTRL->FLASH_COMMON_CFG,
	               FC_FCC_CS_MASK
	               | FC_FCC_FBS_MASK
	               | FC_FCC_CPOL_MASK
	               | FC_FCC_CPHA_MASK,
	               cs | fbs | mode);
}

/*static inline void FC_Ibus_SetCsDelay(uint8_t enable_cyc, uint8_t disable_cyc, uint8_t deselect_cyc)
{
	HAL_MODIFY_REG(FLASH_CTRL->I_CS_WAIT,
				   FC_ICW_BEGIN_MASK
				   | FC_ICW_DESEL_MASK
				   | FC_ICW_OVER_MASK,
				   (enable_cyc << FC_ICW_BEGIN_SHIFT)
				   | (disable_cyc << FC_ICW_OVER_SHIFT)
				   | (deselect_cyc << FC_ICW_DESEL_SHIFT));
}

static inline void FC_Sbus_SetCsDelay(uint8_t enable_cyc, uint8_t disable_cyc, uint8_t deselect_cyc)
{
	HAL_MODIFY_REG(FLASH_CTRL->I_CS_WAIT,
				   FC_ICW_BEGIN_MASK
				   | FC_ICW_OVER_MASK,
				   (enable_cyc << FC_ICW_BEGIN_SHIFT)
				   | (disable_cyc << FC_ICW_OVER_SHIFT));

	HAL_MODIFY_REG(FLASH_CTRL->COMMON_ADD_CFG,
				   FC_CAC_CS_DESEL_WAIT_MASK,
				   deselect_cyc << FC_CAC_CS_DESEL_WAIT_SHIFT);
}*/

void FC_Sbus_ResetFIFO(bool tx, bool rx)
{
	HAL_MODIFY_REG(FLASH_CTRL->COMMON_ADD_CFG,
	               FC_CAC_TX_FIFO_RESET_MASK | FC_CAC_RX_FIFO_RESET_MASK,
	               (tx << FC_CAC_TX_FIFO_RESET_SHIFT) | (rx << FC_CAC_RX_FIFO_RESET_SHIFT));
}

static inline void FC_WrapMode(bool enable)
{
	HAL_MODIFY_REG(FLASH_CTRL->COMMON_ADD_CFG, FC_CAC_WRAP_EN_MASK, enable << FC_CAC_WRAP_EN_SHIFT);
}

static inline void FC_AddressMode(FC_Addr_Mode mode)
{
	HAL_MODIFY_REG(FLASH_CTRL->COMMON_ADD_CFG, FC_CAC_ADDR_SIZE_MODE_MASK, mode << FC_CAC_ADDR_SIZE_MODE_SHIFT);
}

static inline void FC_Sbus_CommandConfig(FC_CycleBits cmd,
                FC_CycleBits addr,
                FC_CycleBits dummy,
                FC_CycleBits data,
                uint8_t dummy_byte)
{
	uint8_t dummy_width;

	if (dummy_byte > 8)
		dummy_byte = 8;
	dummy_width = dummy_byte * 8;

	HAL_MODIFY_REG(FLASH_CTRL->S_CMD_CFG,
	               FC_SCC_CMD_BIT_MASK
	               | FC_SCC_ADDR_BIT_MASK
	               | FC_SCC_DUMMY_BIT_MASK
	               | FC_SCC_DATA_BIT_MASK
	               | FC_SCC_DUMMY_DATA_BIT_MASK,
	               (cmd << FC_SCC_CMD_BIT_SHIFT)
	               | (addr << FC_SCC_ADDR_BIT_SHIFT)
	               | (dummy << FC_SCC_DUMMY_BIT_SHIFT)
	               | (data << FC_SCC_DATA_BIT_SHIFT)
	               | (dummy_width << FC_SCC_DUMMY_DATA_BIT_SHIFT));
}

static inline void FC_Sbus_Command(uint8_t cmd, uint32_t addr, uint32_t dummyh, uint32_t dummyl)
{
	HAL_MODIFY_REG(FLASH_CTRL->S_CMD_CFG, FC_SCC_CMD_MASK, cmd << FC_SCC_CMD_SHIFT);
	HAL_MODIFY_REG(FLASH_CTRL->S_ADDR_CFG, FC_SAC_MASK, addr << FC_SAC_SHIFT);
	HAL_MODIFY_REG(FLASH_CTRL->S_DUMMY_H, FC_SDH_MASK, dummyh << FC_SDH_SHIFT);
	HAL_MODIFY_REG(FLASH_CTRL->S_DUMMY_L, FC_SDL_MASK, dummyl << FC_SDL_SHIFT);
}

static inline void FC_Sbus_WriteSize(uint16_t size)
{
	if (size & (~0x1FF))
		FC_ERROR("wr size error");
	size &= 0x1FF;
	HAL_MODIFY_REG(FLASH_CTRL->S_WR_NUM, FC_SWN_MASK, size << FC_SWN_SHIFT);
}

static inline void FC_Sbus_ReadSize(uint32_t size)
{
	HAL_MODIFY_REG(FLASH_CTRL->S_RD_NUM, FC_SRN_MASK, size << FC_SRN_SHIFT);
}

/*static inline void FC_Sbus_DummyData(uint32_t dummyh, uint32_t dummyl)
{
	FLASH_CTRL->S_DUMMY_H = dummyh;
	FLASH_CTRL->S_DUMMY_L = dummyl;
}*/

void FC_Sbus_TransmitDelay(Flash_Ctrl_DelayCycle *delay)
{
	HAL_MODIFY_REG(FLASH_CTRL->I_CS_WAIT,
	               FC_ICW_BEGIN_MASK
	               | FC_ICW_OVER_MASK,
	               (delay->cs_begin << FC_ICW_BEGIN_SHIFT)
	               | (delay->cs_over << FC_ICW_OVER_SHIFT));

	HAL_MODIFY_REG(FLASH_CTRL->COMMON_ADD_CFG,
	               FC_CAC_CS_DESEL_WAIT_MASK,
	               delay->cs_deselect << FC_CAC_CS_DESEL_WAIT_SHIFT);

	HAL_MODIFY_REG(FLASH_CTRL->S_IO_WAIT,
	               FC_SIW_CMD_MASK
	               | FC_SIW_ADDR_MASK
	               | FC_SIW_DUMMY_MASK,
	               (delay->cmd_over << FC_SIW_CMD_SHIFT)
	               | (delay->addr_over << FC_SIW_ADDR_SHIFT)
	               | (delay->dummy_over << FC_SIW_DUMMY_SHIFT));

	HAL_MODIFY_REG(FLASH_CTRL->FLASH_COMMON_CFG,
	               FC_FCC_WAIT_DATA_MASK,
	               delay->data << FC_FCC_WAIT_DATA_SHIFT);
}

static inline void FC_Sbus_FIFOTriggerLevel(uint8_t txfull, uint8_t txempty, uint8_t rxfull, uint8_t rxempty)
{
	HAL_MODIFY_REG(FLASH_CTRL->FIFO_TRIG_LEVEL,
	               FC_FTL_RD_FIFO_EMPTY_MASK
	               | FC_FTL_RD_FIFO_FULL_MASK
	               | FC_FTL_WR_FIFO_EMPTY_MASK
	               | FC_FTL_WR_FIFO_FULL_MASK,
	               (txfull << FC_FTL_WR_FIFO_FULL_SHIFT)
	               | (txempty << FC_FTL_WR_FIFO_EMPTY_SHIFT)
	               | (rxfull << FC_FTL_RD_FIFO_FULL_SHIFT)
	               | (rxempty << FC_FTL_RD_FIFO_EMPTY_SHIFT));
}


bool FC_Sbus_IsAvailable(FC_Sbus_RW rw)
{
	if (rw == FC_SBUS_WRITE)
		return !!HAL_GET_BIT(FLASH_CTRL->FIFO_STATUS, FC_FS_WR_BUF_VALID_MASK);
	else
		return !!HAL_GET_BIT(FLASH_CTRL->FIFO_STATUS, FC_FS_RD_BUF_VALID_MASK);
}

int FC_Sbus_GetBufCnt(FC_Sbus_RW rw)
{
	if (rw == FC_SBUS_WRITE)
		return HAL_GET_BIT_VAL(FLASH_CTRL->FIFO_STATUS, FC_FS_WR_BUF_CNT_SHIFT, FC_FS_WR_BUF_CNT_VMASK);
	else
		return HAL_GET_BIT_VAL(FLASH_CTRL->FIFO_STATUS, FC_FS_RD_BUF_CNT_SHIFT, FC_FS_RD_BUF_CNT_VMASK);
}

static int FC_Sbus_GetFIFOCnt(FC_Sbus_RW rw)
{
	if (rw == FC_SBUS_WRITE)
		return HAL_GET_BIT_VAL(FLASH_CTRL->FIFO_STATUS, FC_FS_WR_FIFO_CNT_SHIFT, FC_FS_WR_FIFO_CNT_VMASK);
	else
		return HAL_GET_BIT_VAL(FLASH_CTRL->FIFO_STATUS, FC_FS_RD_FIFO_CNT_SHIFT, FC_FS_RD_FIFO_CNT_VMASK);
}

/*
	Debug State:
	0x0 Idle or Sbus complete;
	0x2 Send CMD;
	0x4 Send Address;
	0x6 Send Dummy;
	0x8 Send Data;
	0x9 Get Data;
	0xc Ibus read complete;
*/
static inline int FC_Sbus_GetDebugState(void)
{
	return HAL_GET_BIT_VAL(FLASH_CTRL->FIFO_STATUS, FC_FS_STATUS_DGB_SHIFT, FC_FS_STATUS_DGB_VMASK);
}

static inline bool FC_IsWrapMode(void)
{
	return !!HAL_GET_BIT(FLASH_CTRL->WRAP_MODE, FC_WM_MASK);
}

static inline void FC_Sbus_StartSend(void)
{
	HAL_SET_BIT(FLASH_CTRL->START_SEND, FC_SS_MASK);
}

static inline bool FC_Sbus_isSending(void)
{
	return !!HAL_GET_BIT(FLASH_CTRL->START_SEND, FC_SS_MASK);
}

typedef enum FC_Sbus_IntType {
	FC_INT_HREADY_TIMEOUT           = 1 << 13,
	FC_INT_TC                       = 1 << FC_IE_CMPL_SHIFT,
	FC_INT_WR_FIFO_UNDERFLOW        = 1 << FC_IE_WR_FIFO_UNDERFLOW_SHIFT,
	FC_INT_WR_FIFO_OVERFLOW         = 1 << FC_IE_WR_FIFO_OVERFLOW_SHIFT,
	FC_INT_RD_FIFO_UNDERFLOW        = 1 << FC_IE_RD_FIFO_UNDERFLOW_SHIFT,
	FC_INT_RD_FIFO_OVERFLOW         = 1 << FC_IE_RD_FIFO_OVERFLOW_SHIFT,
	FC_INT_WR_FIFO_FULL             = 1 << FC_IE_WR_FIFO_FULL_SHIFT,
	FC_INT_WR_FIFO_EMPTY            = 1 << FC_IE_WR_FIFO_EMPTY_SHIFT,
	FC_INT_WR_FIFO_READY            = 1 << FC_IE_WR_FIFO_READY_SHIFT,
	FC_INT_RD_FIFO_FULL             = 1 << FC_IE_RD_FIFO_FULL_SHIFT,
	FC_INT_RD_FIFO_EMPTY            = 1 << FC_IE_RD_FIFO_EMPTY_SHIFT,
	FC_INT_RD_FIFO_READY            = 1 << FC_IE_RD_FIFO_READY_SHIFT,
} FC_Sbus_IntType;

static inline void FC_Sbus_EnableInt(FC_Sbus_IntType type)
{
	HAL_SET_BIT(FLASH_CTRL->INT_EN, type);
}

static inline void FC_Sbus_DisableInt(FC_Sbus_IntType type)
{
	HAL_CLR_BIT(FLASH_CTRL->INT_EN, type);
}

static inline int FC_Sbus_GetStatus(FC_Sbus_IntType type)
{
	return !!HAL_GET_BIT(FLASH_CTRL->INT_STA, type);
}

static inline void FC_Sbus_ClrStatus(FC_Sbus_IntType type)
{
	HAL_CLR_BIT(FLASH_CTRL->INT_STA, type);
}

static inline void FC_Sbus_Write(uint8_t data)
{
	*((uint8_t *)&FLASH_CTRL->S_WDATA) = data;
}

static inline uint8_t FC_Sbus_Read(void)
{
	return *((uint8_t *)&FLASH_CTRL->S_RDATA);
}

#endif /* FLASHCTRL_ON_XR875 */

/*
 * @brief
 */
void HAL_Flashc_EnableCCMU(struct flash_controller *ctrl)
{
	if (ctrl->ccmu_on++ != 0)
		return;

	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_FLASH_CTRL);
	HAL_CCM_FLASHC_EnableMClock();
}

/*
 * @brief
 */
void HAL_Flashc_DisableCCMU(struct flash_controller *ctrl)
{
	if (--ctrl->ccmu_on != 0)
		return;

	FC_DEBUG("DISABLE CCMU");
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_FLASH_CTRL);
	HAL_CCM_FLASHC_DisableMClock();
}

void HAL_Flashc_ResetCCMU(void)
{
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_FLASH_CTRL);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_FLASH_CTRL);
}

/*
 * @brief
 */
bool HAL_Flashc_ConfigCCMU(uint32_t clk)
{
	CCM_AHBPeriphClkSrc src;
	uint32_t mclk;
	uint32_t div;
	CCM_PeriphClkDivN div_n = 0;
	CCM_PeriphClkDivM div_m = 0;

	if (clk > HAL_GetHFClock()) {
		mclk = HAL_GetDevClock();
		src = CCM_AHB_PERIPH_CLK_SRC_DEVCLK;
	} else {
		mclk = HAL_GetHFClock();
		src = CCM_AHB_PERIPH_CLK_SRC_HFCLK;
	}

	div = (mclk + clk - 1) / clk;
	div = div==0 ? 1 : div;

	if (div > (16 * 8))
		return 0;

	if (div > 64) {
		div_n = CCM_PERIPH_CLK_DIV_N_8;
		div_m = (CCM_PeriphClkDivM)((div >> 3) - 1);
	} else if (div > 32) {
		div_n = CCM_PERIPH_CLK_DIV_N_4;
		div_m = (CCM_PeriphClkDivM)((div >> 2) - 1);
	} else if (div > 16) {
		div_n = CCM_PERIPH_CLK_DIV_N_2;
		div_m = (CCM_PeriphClkDivM)((div >> 1) - 1);
	} else {
		div_n = CCM_PERIPH_CLK_DIV_N_1;
		div_m = (CCM_PeriphClkDivM)((div >> 0) - 1);
	}

	HAL_CCM_FLASHC_SetMClock(src, div_n, div_m);

	return 1;
}

void HAL_Flashc_PinInit(struct flash_controller *ctrl)
{
	if (ctrl->pin_inited++ == 0) {
		/* open io */
		HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_FLASHC, 0), 0);
	}
}

void HAL_Flashc_PinDeinit(struct flash_controller *ctrl)
{
	if ((ctrl->pin_inited > 0) && (--ctrl->pin_inited == 0)) {
		//close io
		HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_FLASHC, 0), 0);
	}
}

/**
  * @brief Initialize Flash controller IBUS driver (XIP).
  * @param cfg:
  *        @arg cfg->addr: Started address of XIP code in Flash.
  *        @arg cfg->freq: Flash working frequency.
  *        @arg cfg->delay: Delay of hardware.
  *        @arg cfg->ins: Instruction of XIP reading
  *        @arg cfg->cont_mode: Enable continue mode in reading or not.
  * @retval HAL_Status: The status of driver.
  */
HAL_Status rom_HAL_Flashc_Xip_Init(struct flash_controller *ctrl, XIP_Config *cfg)
{
	HAL_ASSERT_PARAM(!(FLASH_ICACHE_START_ADDR & 0x0F));
	HAL_ASSERT_PARAM(FLASH_ICACHE_END_ADDR > FLASH_ICACHE_START_ADDR);

#if (defined CONFIG_PM) || (defined FLASHC_TEMP_FIXED)
	if (!ctrl->suspending)
		HAL_Memcpy(&ctrl->pm_ibus_cfg, cfg, sizeof(XIP_Config));
#endif

	/* enable ccmu */
	HAL_Flashc_EnableCCMU(ctrl);

	/* open io */
	HAL_Flashc_PinInit(ctrl);

#if FLASHCTRL_ON_XR875
	HAL_CLR_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, FLASHC_XIP_ENABLE_MASK | FLASHC_CBUS_RW_ENABLE_MASK);
#else
	FC_Ibus_Disable(FC_EN_CONTINUE | FC_EN_IBUS | FC_EN_PREFETCH);
#endif

	/* config flash controller */
	Flash_Ctrl_DelayCycle delay = {1, 0, 3, 0, 0, 0, 1};
	delay.data = FC_GetDataDelay(cfg->freq);

	FC_Ibus_TransmitDelay(&delay);

	FC_SetFlash(FC_TCTRL_CS_LOW_ENABLE, FC_TCTRL_FBS_MSB, FC_SCLK_Mode0);

	FC_Ibus_ReadConfig(cfg->ins.cmd,
	                   cfg->ins.cmd_line,
	                   cfg->ins.addr_line,
	                   cfg->ins.dummy_line,
	                   cfg->ins.data_line,
	                   cfg->ins.dum_btyes);
	if (cfg->cont_mode) {
		FC_Ibus_DummyData(0x20000000, 0);
#if FLASHCTRL_ON_XR875
		HAL_SET_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, FLASHC_XIP_ENABLE_MASK | FLASHC_CBUS_RW_ENABLE_MASK);
		ctrl->xip_continue = FLASHC_XIP_ENABLE_MASK;
#else
		FC_Ibus_Enable(FC_EN_CONTINUE | FC_EN_IBUS);
		ctrl->xip_continue = FC_EN_CONTINUE;
#endif
	} else {
		FC_Ibus_DummyData(0, 0);
#if FLASHCTRL_ON_XR875
		HAL_SET_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, FLASHC_CBUS_RW_ENABLE_MASK);
#else
		FC_Ibus_Enable(FC_EN_IBUS);
#endif
	}

	ctrl->xip_on = 1;

	/* config flash cache */
#if FLASHCTRL_ON_XR875
	HAL_MODIFY_REG(OPI_MEM_CTRL->START_ADDR0, FLASHC_ADDR_FIELD0_START_MASK, \
	               (FLASH_ICACHE_START_ADDR << FLASHC_ADDR_FIELD0_START_SHIFT));
	HAL_MODIFY_REG(OPI_MEM_CTRL->END_ADDR0, FLASHC_ADDR_FIELD0_END_MASK, \
	               (FLASH_ICACHE_END_ADDR << FLASHC_ADDR_FIELD0_END_SHIFT));
	HAL_MODIFY_REG(OPI_MEM_CTRL->BIAS_ADDR0, FLASHC_ADDR_FIELD0_BIAS_MASK, cfg->addr);
	HAL_SET_BIT(OPI_MEM_CTRL->BIAS_ADDR0, FLASHC_ADDR_FIELD0_BIAS_EN_MASK);
#endif
#if 0
	FC_DEBUG("cfg->freq: %d; cfg->ins.cmd: %d; cfg->ins.cmd_line: %d",
	         cfg->freq, cfg->ins.cmd, cfg->ins.cmd_line);
	FC_DEBUG("cfg->ins.addr_line: %d; cfg->ins.dummy_line: %d; cfg->ins.data_line: %d",
	         cfg->ins.addr_line, cfg->ins.dummy_line, cfg->ins.data_line);
	FC_DEBUG("cfg->ins.dum_btyes: %d; cfg->cont_mode: %d; cfg->addr: %d",
	         cfg->ins.dum_btyes, cfg->cont_mode, cfg->addr);
#endif
	FC_DEBUG("ccmu : %d", ctrl->ccmu_on);

	//Regs_Print();

	return HAL_OK;
}

/**
  * @brief Deinitialize Flash controller IBUS (XIP).
  * @param None
  * @retval HAL_Status: The status of driver.
  */
HAL_Status rom_HAL_Flashc_Xip_Deinit(struct flash_controller *ctrl)
{
	unsigned long flags = HAL_EnterCriticalSection();

	//config flash controller
#if FLASHCTRL_ON_XR875
	HAL_CLR_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, FLASHC_XIP_ENABLE_MASK | FLASHC_CBUS_RW_ENABLE_MASK);
#else
	FC_Ibus_Disable(FC_EN_CONTINUE | FC_EN_IBUS | FC_EN_PREFETCH);
#endif
	ctrl->xip_on = 0;
	HAL_ExitCriticalSection(flags);

	//close io
	HAL_Flashc_PinDeinit(ctrl);

	//disable ccmu
	HAL_Flashc_DisableCCMU(ctrl);

	return HAL_OK;
}

/**
  * @internal
  * @brief Flash controller IBUS (XIP) Enable without Pin initialization.
  * @note Most for Flash controller SBUS. It will resume system schedule.
  * @param None
  * @retval None
  */
void HAL_Flashc_Xip_RawEnable(struct flash_controller *ctrl)
{
	if (!ctrl->xip_on)
		return;

#if FLASHCTRL_ON_XR875
	HAL_SET_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, ctrl->xip_continue);
#else
//	HAL_UDelay(100);
//	FC_Ibus_Enable(FC_EN_IBUS | ctrl->xip_continue);

	FC_Ibus_Enable(ctrl->xip_continue);
#endif
	/* if irq disable, do not resume scheduler in case of system error */
	if (!HAL_IsIRQDisabled())
		HAL_ThreadResumeScheduler();
}

/**
  * @internal
  * @brief Flash controller IBUS (XIP) Enable without Pin deinitialization.
  * @note Most for Flash controller SBUS. It will suspend system schedule.
  * @param None
  * @retval None
  */
void HAL_Flashc_Xip_RawDisable(struct flash_controller *ctrl)
{
	if (!ctrl->xip_on)
		return;

	/* if irq disable, do not suspend scheduler in case of system error */
	if (!HAL_IsIRQDisabled())
		HAL_ThreadSuspendScheduler();

//	HAL_UDelay(100);
//	FC_Ibus_Disable(FC_EN_IBUS | ctrl->xip_continue);
	while((FC_Sbus_GetDebugState() != 0x0c) && (FC_Sbus_GetDebugState() != 0x00));
#if FLASHCTRL_ON_XR875
	HAL_CLR_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, ctrl->xip_continue);
#else
	FC_Ibus_Disable(ctrl->xip_continue);
#endif
}

#ifdef CONFIG_PM
int flashc_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	struct flash_controller *ctrl = dev->platform_data;

	/*
		suspend condition:
			(1) not in sbus opened state
			(2)
	*/
	ctrl->suspending = 1;

	if (ctrl->sbusing)
		return -1;

	while((FC_Sbus_GetDebugState() != 0x0c) && (FC_Sbus_GetDebugState() != 0x00));

	switch (state) {
	case PM_MODE_SLEEP:
#if FLASHCTRL_ON_XR875
		HAL_CLR_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, ctrl->xip_continue);
#else
		FC_Ibus_Disable(ctrl->xip_continue);
#endif
		HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_FLASH_CTRL);
		HAL_CCM_FLASHC_DisableMClock();
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
	case PM_MODE_POWEROFF:
		if (ctrl->xip_on) {
			HAL_Flashc_Xip_Deinit(ctrl);
			FC_DEBUG("ccmu : %d", ctrl->ccmu_on);
			ctrl->pm_xip = 1;
		}
		HAL_Flashc_Deinit(ctrl);
		FC_DEBUG("ccmu : %d", ctrl->ccmu_on);
		break;
	default:
		break;
	}

	FC_Reg_All();
	return 0;
}

int flashc_resume(struct soc_device *dev, enum suspend_state_t state)
{
	struct flash_controller *ctrl = dev->platform_data;

	FC_Reg_All();

	switch (state) {
	case PM_MODE_SLEEP:
#if FLASHCTRL_ON_XR875
		HAL_SET_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, ctrl->xip_continue);
#else
		FC_Ibus_Enable(ctrl->xip_continue);
#endif
		HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_FLASH_CTRL);
		HAL_CCM_FLASHC_EnableMClock();
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_Flashc_Init(ctrl, &ctrl->pm_sbus_cfg);
		if (ctrl->pm_xip) {
			ctrl->pm_xip = 0;
			HAL_Flashc_Xip_Init(ctrl, &ctrl->pm_ibus_cfg);
			HAL_UDelay(300);
		}
		FC_DEBUG("ccmu: %d, pin: %d", ctrl->ccmu_on, ctrl->pin_inited);
		break;
	default:
		break;
	}

	ctrl->suspending = 0;

	return 0;
}
#endif

/**
  * @brief Delay realization in Flash controller IBUS (XIP).
  * @note Delay can be system sleep while it's not in XIP, but must be a while
  *       delay without system interface while it's in XIP.
  * @param us: delay time in microsecond.
  * @retval None
  */
void rom_HAL_Flashc_Delay(struct flash_controller *ctrl, unsigned int us)
{
	if (us == 0)
		return;

	if (ctrl->xip_on || !HAL_ThreadIsSchedulerRunning()) {
		HAL_UDelay(us);
	} else {
		HAL_MSleep((us + 1023) >> 10);
	}
}

/**
 * @brief Initialize Flash controller SBUS.
 * @param cfg:
 * 	   @arg cfg->freq: Flash working frequency.
 * @retval HAL_Status: The status of driver.
 */
HAL_Status rom_HAL_Flashc_Init(struct flash_controller *ctrl, const Flashc_Config *cfg)
{
	Flash_Ctrl_DelayCycle delay = {1, 0, 3, 0, 0, 0, 1};

	/* enable ccmu */
	HAL_Flashc_ResetCCMU();
	HAL_Flashc_ConfigCCMU(cfg->freq);
	HAL_Flashc_EnableCCMU(ctrl);
	/* config flash controller */
	/*delay.cs_deselect = cfg->t_shsl_ns * (cfg->freq / 1000000) / 1000;*/
	delay.data = FC_GetDataDelay(cfg->freq);
	FC_Sbus_TransmitDelay(&delay);

	FC_SetFlash(FC_TCTRL_CS_LOW_ENABLE, FC_TCTRL_FBS_MSB, FC_SCLK_Mode0);
#ifdef __CONFIG_CHIP_XR875
	if(ctrl->externAddr_on == 1) {
		FC_Enable_32BitAddr_Mode();
	}
#endif
	FC_Sbus_ResetFIFO(1, 1);

	FC_DEBUG("ccmu : %d", ctrl->ccmu_on);
	//Regs_Print();

	HAL_Flashc_DisableCCMU(ctrl);
#if (defined CONFIG_PM) || (defined FLASHC_TEMP_FIXED)
	if (!ctrl->suspending) {
		HAL_Memset(&ctrl->flashc_drv, 0, sizeof(struct soc_device_driver));
		ctrl->flashc_drv.name = "flashc";
		ctrl->flashc_drv.suspend_noirq = flashc_suspend;
		ctrl->flashc_drv.resume_noirq = flashc_resume;
		HAL_Memset(&ctrl->flashc_dev, 0, sizeof(struct soc_device));
		ctrl->flashc_dev.name = "flashc";
		ctrl->flashc_dev.driver = &ctrl->flashc_drv;
		ctrl->flashc_dev.platform_data = ctrl;
		ctrl->pm_xip = 0;
		HAL_Memcpy(&ctrl->pm_sbus_cfg, cfg, sizeof(Flashc_Config));
		pm_register_ops(&ctrl->flashc_dev);
	}
#endif

	return HAL_OK;
}

/**
* @brief Deinitialize Flash controller SBUS.
* @param None
* @retval HAL_Status: The status of driver.
*/
HAL_Status rom_HAL_Flashc_Deinit(struct flash_controller *ctrl)
{
#if (defined CONFIG_PM) || (defined FLASHC_TEMP_FIXED)
	if (!ctrl->suspending) {
		pm_unregister_ops(&ctrl->flashc_dev);
		ctrl->flashc_dev.platform_data = NULL;
	}
#endif

	return HAL_OK;
}

/**
 * @brief Open flash controller SBUS.
 * @note At the same time, it will disable XIP and suspend schedule.
 * @param None
 * @retval HAL_Status: The status of driver.
 */
HAL_Status rom_HAL_Flashc_Open(struct flash_controller *ctrl)
{
	ctrl->sbusing = 1;
	HAL_Flashc_Xip_RawDisable(ctrl);

#ifdef FLASHC_TEMP_FIXED	/* temporarily fixed */
	ctrl->suspending = 1;

	if (ctrl->xip_on) {
		HAL_Flashc_Xip_Deinit(ctrl);

		FC_DEBUG("ccmu : %d", ctrl->ccmu_on);
		ctrl->xip_on = 1;
	}

	HAL_Flashc_Deinit(ctrl);
	HAL_Flashc_Init(ctrl, &ctrl->pm_sbus_cfg);
	FC_DEBUG("ccmu : %d", ctrl->ccmu_on);
#endif

	HAL_Flashc_EnableCCMU(ctrl);
	HAL_Flashc_PinInit(ctrl);
	FC_Sbus_ResetFIFO(1, 1);

	return HAL_OK;
}

/**
 * @brief Close flash controller SBUS.
 * @param None
 * @retval HAL_Status: The status of driver.
 */
HAL_Status rom_HAL_Flashc_Close(struct flash_controller *ctrl)
{
	HAL_Flashc_PinDeinit(ctrl);
	HAL_Flashc_DisableCCMU(ctrl);

#ifdef FLASHC_TEMP_FIXED	/* temporarily fixed */
	HAL_Flashc_Deinit(ctrl);
	HAL_Flashc_Init(ctrl, &ctrl->pm_sbus_cfg);
	if (ctrl->xip_on) {
		HAL_Flashc_Xip_Init(ctrl, &ctrl->pm_ibus_cfg);
		HAL_UDelay(300);
	}
	FC_DEBUG("ccmu: %d, pin: %d", ctrl->ccmu_on, ctrl->pin_inited);
	ctrl->suspending = 0;

#endif

	HAL_Flashc_Xip_RawEnable(ctrl);
	ctrl->sbusing = 0;
	return HAL_OK;
}

/**
 * @brief Flash controller ioctl.
 * @note op : arg
 *       nothing support for now.
 * @param op: ioctl command.
 * @param arg: ioctl arguement
 * @retval HAL_Status: The status of driver.
 */
HAL_Status HAL_Flashc_Ioctl(struct flash_controller *ctrl, uint32_t op, void *arg)
{
	switch(op)
	{
#ifdef __CONFIG_CHIP_XR875
		case FC_CMD_ENABLE_32BITADDR_MODE:
			FC_Enable_32BitAddr_Mode();
			break;
#endif
        default:
            break;
	}
	return HAL_INVALID;
}

static void HAL_Flashc_DMARelease(void *arg)
{
	HAL_SemaphoreRelease(&(((struct flash_controller *)arg)->dmaSem));
}

static HAL_Status HAL_Flashc_DMATransfer(struct flash_controller *ctrl, int write, uint8_t *data, uint32_t size)
{
	HAL_Status ret = HAL_OK;
	DMA_ChannelInitParam dma_arg;
	DMA_Channel dma_ch;
	HAL_Memset(&dma_arg, 0, sizeof(dma_arg));
	HAL_Memset(&dma_ch, 0, sizeof(dma_ch));
	HAL_Memset(&ctrl->dmaSem, 0, sizeof(HAL_Semaphore));

	if (size == 0 || data == NULL)
		return HAL_ERROR;

	if ((dma_ch = HAL_DMA_Request()) == DMA_CHANNEL_INVALID) {
		FC_ERROR("DMA request failed");
		ret = HAL_BUSY;
		goto failed;
	}

	HAL_SemaphoreInit(&ctrl->dmaSem, 0, 1);

	dma_arg.irqType = DMA_IRQ_TYPE_END;
	dma_arg.endCallback = (DMA_IRQCallback)HAL_Flashc_DMARelease;
	dma_arg.endArg = ctrl;
	if (write)
		dma_arg.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
		                DMA_WAIT_CYCLE_2,
		                DMA_BYTE_CNT_MODE_REMAIN,
		                DMA_DATA_WIDTH_32BIT,
		                DMA_BURST_LEN_1,
		                DMA_ADDR_MODE_FIXED,
		                (DMA_Periph)(DMA_PERIPH_FLASHC),
		                DMA_DATA_WIDTH_8BIT,
		                DMA_BURST_LEN_4,
		                DMA_ADDR_MODE_INC,
		                DMA_PERIPH_SRAM);
	else
		dma_arg.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
		                DMA_WAIT_CYCLE_2,
		                DMA_BYTE_CNT_MODE_REMAIN,
		                DMA_DATA_WIDTH_8BIT,
		                DMA_BURST_LEN_4,
		                DMA_ADDR_MODE_INC,
		                DMA_PERIPH_SRAM,
		                DMA_DATA_WIDTH_32BIT,
		                DMA_BURST_LEN_1,
		                DMA_ADDR_MODE_FIXED,
		                (DMA_Periph)(DMA_PERIPH_FLASHC));
	HAL_DMA_Init(dma_ch, &dma_arg);
#if FLASHCTRL_ON_XR875
	if (write)
		HAL_DMA_Start(dma_ch, (uint32_t)data, (uint32_t)&OPI_MEM_CTRL->SBUS_WR_DATA_REG, size);
	else
		HAL_DMA_Start(dma_ch, (uint32_t)&OPI_MEM_CTRL->SBUS_RD_DATA_REG, (uint32_t)data, size);
#else
	if (write)
		HAL_DMA_Start(dma_ch, (uint32_t)data, (uint32_t)&FLASH_CTRL->S_WDATA, size);
	else
		HAL_DMA_Start(dma_ch, (uint32_t)&FLASH_CTRL->S_RDATA, (uint32_t)data, size);
#endif

	FC_Sbus_StartSend();
	if ((ret = HAL_SemaphoreWait(&ctrl->dmaSem, 5000)) != HAL_OK)
		FC_ERROR("sem wait failed: %d", ret);

	uint32_t i;
	FC_WHILE_TIMEOUT(FC_Sbus_isSending(), i);
	FC_Sbus_ClrStatus(FC_INT_TC);

	HAL_DMA_Stop(dma_ch);
	HAL_DMA_DeInit(dma_ch);
	HAL_DMA_Release(dma_ch);

	HAL_SemaphoreDeinit(&ctrl->dmaSem);

	if (FC_DebugCheck(0))
		return HAL_ERROR;

failed:
	return ret;
}

static HAL_Status HAL_Flashc_PollTransfer(struct flash_controller *ctrl, int write, uint8_t *data, uint32_t size)
{
	uint32_t i;

	FC_Sbus_StartSend();

	if (write) {
		while (size--) {
			FC_WHILE_TIMEOUT(FC_Sbus_GetFIFOCnt(FC_SBUS_WRITE) > 100, i);
			FC_Sbus_Write(*(data++));
		}
	} else {
		while (size--) {
			FC_WHILE_TIMEOUT(FC_Sbus_GetFIFOCnt(FC_SBUS_READ) == 0, i);
			*(data++) = FC_Sbus_Read();
		}
	}

//	FC_WHILE_TIMEOUT(FC_Sbus_GetDebugState() != 0, i);
	FC_WHILE_TIMEOUT(FC_Sbus_isSending(), i);
	FC_Sbus_ClrStatus(FC_INT_TC);

	if (FC_DebugCheck(0))
		return HAL_ERROR;

	return HAL_OK;
}

/**
 * @brief Write or read flash by flash controller SBUS.
 * @note Send a instruction in command + address + dummy + write or read data.
 * @param cmd: Command of instruction.
 *        @arg cmd->pdata: The data is filled with in this field.
 *        @arg cmd->len: The data len of this field.
 *        @arg cmd->line: The number of line transfering this field data.
 * @param addr: Address of instruction
 * @param dummy: Dummy of instruction
 * @param data: Data of instruction
 * @param dma: Transfer data by DMA or not.
 * @retval HAL_Status: The status of driver.
 */
HAL_Status HAL_Flashc_Transfer(struct flash_controller *ctrl, int write,
                               FC_InstructionField *cmd, FC_InstructionField *addr,
                               FC_InstructionField *dummy, FC_InstructionField *data, bool dma)
{
	HAL_Status ret;
	FC_InstructionField zero;
	HAL_Memset(&zero, 0, sizeof(zero));

	/* instruction check */
	if (cmd == NULL)
		cmd = &zero;
	if (addr == NULL)
		addr = &zero;
	if (dummy == NULL)
		dummy = &zero;
	if (data == NULL)
		data = &zero;

	FC_Sbus_ResetFIFO(1, 1);
	FC_Sbus_CommandConfig(cmd->line, addr->line, dummy->line, data->line, dummy->len);
	FC_Sbus_Command(*(cmd->pdata), *((uint32_t *)addr->pdata), 0, 0);
	FC_Reg_All();
	if (write)
		FC_Sbus_WriteSize(data->len);
	else
		FC_Sbus_ReadSize(data->len);

	if (dma == 1 && data->len != 0 && !ctrl->xip_on && !HAL_IsISRContext()) {
		ret = HAL_Flashc_DMATransfer(ctrl, write, data->pdata, data->len);
	} else {
		ret = HAL_Flashc_PollTransfer(ctrl, write, data->pdata, data->len);
	}

	if (ret != HAL_OK)
		FC_ERROR("err on cmd: 0x%x, data len: %d", *cmd->pdata, data->len);

	return ret;
}

struct flash_controller *HAL_Flashc_Create(uint32_t id) {
	struct flash_controller *ctrl;

	ctrl = HAL_Malloc(sizeof(struct flash_controller));
	if (ctrl == NULL) {
		FC_ERROR("no mem");
	} else {
		HAL_Memset(ctrl, 0, sizeof(struct flash_controller));
	}

	return ctrl;
}

HAL_Status HAL_Flashc_Destory(struct flash_controller *ctrl)
{
	if (ctrl == NULL) {
		FC_ERROR("ctrl %p", ctrl);
	} else {
		HAL_Free(ctrl);
	}

	return HAL_OK;
}

int HAL_Flashc_IncRef(struct flash_controller *ctrl)
{
	return (++ctrl->ref);
}

int HAL_Flashc_DecRef(struct flash_controller *ctrl)
{
	if (ctrl->ref > 0) {
		return (--ctrl->ref);
	} else {
		return -1;
	}
}
