/**
  * @file  hal_flashctrl.h
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

#ifndef _DRIVER_CHIP_HAL_FLASHCTRL_H_
#define _DRIVER_CHIP_HAL_FLASHCTRL_H_

#include <stdbool.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_def.h"
#include "pm/pm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLASHCTRL_ARCH_V2		(__CONFIG_CHIP_ARCH_VER == 2)

#if (!FLASHCTRL_ARCH_V2)

typedef struct
{
	__IO uint32_t COMMON_CFG;               /* ,     Address offset: 0x000   */
	__IO uint32_t I_CMD_CFG;                /* ,     Address offset: 0x004   */
	__IO uint32_t I_DUMMY_H;                /* ,     Address offset: 0x008   */
	__IO uint32_t I_DUMMY_L;                /* ,     Address offset: 0x00C   */
	__IO uint32_t I_CS_WAIT;                /* ,     Address offset: 0x010   */
	__IO uint32_t I_IO_WAIT;                /* ,     Address offset: 0x014   */
	__IO uint32_t RESERVE0[2];
	__IO uint32_t FLASH_COMMON_CFG;         /* ,     Address offset: 0x020   */
	__I  uint32_t XIP_EXEC;                 /* ,     Address offset: 0x024   */

	__IO uint32_t COMMON_ADD_CFG;           /* ,     Address offset: 0x028   */
	__IO uint32_t S_CMD_CFG;                /* ,     Address offset: 0x02C   */
	__IO uint32_t S_ADDR_CFG;               /* ,     Address offset: 0x030   */
	__IO uint32_t S_WR_NUM;                 /* ,     Address offset: 0x034   */
	__IO uint32_t S_RD_NUM;                 /* ,     Address offset: 0x038   */
	__IO uint32_t S_DUMMY_H;                /* ,     Address offset: 0x03C   */
	__IO uint32_t S_DUMMY_L;                /* ,     Address offset: 0x040   */
	__IO uint32_t FIFO_TRIG_LEVEL;          /* ,     Address offset: 0x044   */
	__I  uint32_t FIFO_STATUS;              /* ,     Address offset: 0x048   */
	__IO uint32_t S_IO_WAIT;                /* ,     Address offset: 0x04C   */
	__I  uint32_t WRAP_MODE;                /* ,     Address offset: 0x050   */
	__IO uint32_t START_SEND;               /* ,     Address offset: 0x054   */
	__IO uint32_t INT_EN;                   /* ,     Address offset: 0x058   */
	__IO uint32_t INT_STA;                  /* ,     Address offset: 0x05C   */
	__IO uint32_t RESERVE1[40];
	__IO uint32_t S_WDATA;                  /* ,     Address offset: 0x100   */
	__IO uint32_t RESERVE2[63];
	/* (0X200 - 0X100) * 4 / 16 -1 = 63*/
	__I  uint32_t S_RDATA;                  /* ,     Address offset: 0x200   */
} FLASH_CTRL_T;

#define FLASH_CTRL ((FLASH_CTRL_T *)FLASH_CTRL_BASE)

#define FC_CC_CONT_EN_SHIFT             (20)
#define FC_CC_CONT_EN_MASK              (0x1U << FC_CC_CONT_EN_SHIFT)

#define FC_CC_IO1_SHIFT                 (16)
#define FC_CC_IO1_MASK                  (0x3U << FC_CC_IO1_SHIFT)

#define FC_CC_IO2_SHIFT                 (12)
#define FC_CC_IO2_MASK                  (0x3U << FC_CC_IO2_SHIFT)

#define FC_CC_IO3_SHIFT                 (8)
#define FC_CC_IO3_MASK                  (0x3U << FC_CC_IO2_SHIFT)

typedef enum {
	FC_IO_OUTPUT_0,
	FC_IO_OUTPUT_1,
	FC_IO_OUTPUT_Z
} FC_Io_Output;

#define FC_CC_PREFETCH_EN_SHIFT         (4)
#define FC_CC_PREFETCH_EN_MASK          (0x1U << FC_CC_PREFETCH_EN_SHIFT)

#define FC_CC_IBUS_EN_SHIFT             (0)
#define FC_CC_IBUS_EN_MASK              (0x1U << FC_CC_IBUS_EN_SHIFT)

typedef enum {
	FC_EN_CONTINUE = 1 << FC_CC_CONT_EN_SHIFT,
	FC_EN_PREFETCH = 1 << FC_CC_PREFETCH_EN_SHIFT,
	FC_EN_IBUS = 1 << FC_CC_IBUS_EN_SHIFT
} FC_En;

#define FC_ICC_CMD_SHIFT                (24)
#define FC_ICC_CMD_MASK                 (0xFFU << FC_ICC_CMD_SHIFT)

#define FC_ICC_CMD_BIT_SHIFT            (20)
#define FC_ICC_CMD_BIT_MASK             (0x3U << FC_ICC_CMD_BIT_SHIFT)

#define FC_ICC_ADDR_BIT_SHIFT           (16)
#define FC_ICC_ADDR_BIT_MASK            (0x3U << FC_ICC_ADDR_BIT_SHIFT)

#define FC_ICC_DUMMY_BIT_SHIFT          (12)
#define FC_ICC_DUMMY_BIT_MASK           (0x3U << FC_ICC_DUMMY_BIT_SHIFT)

#define FC_ICC_DUMMY_WIDTH_SHIFT        (4)
#define FC_ICC_DUMMY_WIDTH_MASK         (0x7FU << FC_ICC_DUMMY_WIDTH_SHIFT)

#define FC_ICC_DATA_BIT_SHIFT           (0)
#define FC_ICC_DATA_BIT_MASK            (0x3U << FC_ICC_DATA_BIT_SHIFT)


#define FC_IDH_SHIFT                    (0)
#define FC_IDH_MASK                     (0xFFFFFFFFU << FC_IDH_SHIFT)

#define FC_IDL_SHIFT                    (0)
#define FC_IDL_MASK                     (0xFFFFFFFFU << FC_IDL_SHIFT)

#define FC_ICW_BEGIN_SHIFT              (16)
#define FC_ICW_BEGIN_MASK               (0xFFU << FC_ICW_BEGIN_SHIFT)

#define FC_ICW_OVER_SHIFT               (8)
#define FC_ICW_OVER_MASK                (0xFFU << FC_ICW_OVER_SHIFT)

#define FC_ICW_DESEL_SHIFT              (0)
#define FC_ICW_DESEL_MASK               (0xFFU << FC_ICW_DESEL_SHIFT)

#define FC_IIW_CMD_SHIFT                (16)
#define FC_IIW_CMD_MASK                 (0xFFU << FC_IIW_CMD_SHIFT)

#define FC_IIW_ADDR_SHIFT               (8)
#define FC_IIW_ADDR_MASK                (0xFFU << FC_IIW_ADDR_SHIFT)

#define FC_IIW_DUM_SHIFT                (0)
#define FC_IIW_DUM_MASK                 (0xFFU << FC_IIW_DUM_SHIFT)

#define FC_FCC_WAIT_DATA_SHIFT          (12)
#define FC_FCC_WAIT_DATA_MASK           (0x3U << FC_FCC_WAIT_DATA_SHIFT)

#define FC_FCC_CS_SHIFT                 (8)
#define FC_FCC_CS_MASK                  (0x1U << FC_FCC_CS_SHIFT)


typedef enum {
	FC_TCTRL_CS_LOW_ENABLE = 0 << FC_FCC_CS_SHIFT,
	FC_TCTRL_CS_HIGH_ENABLE = 1 << FC_FCC_CS_SHIFT
} FC_Cs;

#define FC_FCC_FBS_SHIFT                (4)
#define FC_FCC_FBS_MASK                 (0x1U << FC_FCC_FBS_SHIFT)

typedef enum {
	FC_TCTRL_FBS_MSB = 0 << FC_FCC_FBS_SHIFT,
	FC_TCTRL_FBS_LSB = 1 << FC_FCC_FBS_SHIFT
} FC_TCTRL_Fbs;

#define FC_FCC_CPOL_SHIFT               (1)
#define FC_FCC_CPOL_MASK                (0x1U << FC_FCC_CPOL_SHIFT)

#define FC_FCC_CPHA_SHIFT               (0)
#define FC_FCC_CPHA_MASK                (0x1U << FC_FCC_CPHA_SHIFT)

typedef enum {
	FC_SCLK_Mode0 = 0 << FC_FCC_CPHA_SHIFT,
	FC_SCLK_Mode1 = 1 << FC_FCC_CPHA_SHIFT,
	FC_SCLK_Mode2 = 2 << FC_FCC_CPHA_SHIFT,
	FC_SCLK_Mode3 = 3 << FC_FCC_CPHA_SHIFT
} FC_Sclk_Mode;

#define FC_XE_SHIFT                     (0)
#define FC_XE_MASK                      (0x1U << FC_XE_SHIFT)

#define FC_CAC_CS_DESEL_WAIT_SHIFT      (4)
#define FC_CAC_CS_DESEL_WAIT_MASK       (0xFFU << FC_CAC_CS_DESEL_WAIT_SHIFT)

#define FC_CAC_TX_FIFO_RESET_SHIFT      (3)
#define FC_CAC_TX_FIFO_RESET_MASK       (0x1U << FC_CAC_TX_FIFO_RESET_SHIFT)

#define FC_CAC_RX_FIFO_RESET_SHIFT      (2)
#define FC_CAC_RX_FIFO_RESET_MASK       (0x1U << FC_CAC_RX_FIFO_RESET_SHIFT)

#define FC_CAC_WRAP_EN_SHIFT            (1)
#define FC_CAC_WRAP_EN_MASK             (0x1U << FC_CAC_WRAP_EN_SHIFT)

#define FC_CAC_ADDR_SIZE_MODE_SHIFT     (0)
#define FC_CAC_ADDR_SIZE_MODE_MASK      (0x1U << FC_CAC_ADDR_SIZE_MODE_SHIFT)

typedef enum {
	FC_ADDR_MODE_24BIT = 0 << FC_CAC_ADDR_SIZE_MODE_SHIFT,
	FC_ADDR_MODE_32BIT = 1 << FC_CAC_ADDR_SIZE_MODE_SHIFT,
} FC_Addr_Mode;


#define FC_SCC_CMD_SHIFT                (24)
#define FC_SCC_CMD_MASK                 (0xFFU << FC_SCC_CMD_SHIFT)

#define FC_SCC_CMD_BIT_SHIFT            (20)
#define FC_SCC_CMD_BIT_MASK             (0x3U << FC_SCC_CMD_BIT_SHIFT)

#define FC_SCC_ADDR_BIT_SHIFT           (16)
#define FC_SCC_ADDR_BIT_MASK            (0x3U << FC_SCC_ADDR_BIT_SHIFT)

#define FC_SCC_DUMMY_BIT_SHIFT          (12)
#define FC_SCC_DUMMY_BIT_MASK           (0x3U << FC_SCC_DUMMY_BIT_SHIFT)

#define FC_SCC_DUMMY_DATA_BIT_SHIFT     (4)
#define FC_SCC_DUMMY_DATA_BIT_MASK      (0x3FU << FC_SCC_DUMMY_DATA_BIT_SHIFT)

#define FC_SCC_DATA_BIT_SHIFT           (0)
#define FC_SCC_DATA_BIT_MASK            (0x3U << FC_SCC_DATA_BIT_SHIFT)

#define FC_SAC_SHIFT                    (0)
#define FC_SAC_MASK                     (0xFFFFFFFFU << FC_SAC_SHIFT)

#define FC_SWN_SHIFT                    (0)
#define FC_SWN_MASK                     (0xFFU << FC_SWN_SHIFT)

#define FC_SRN_SHIFT                    (0)
#define FC_SRN_MASK                     (0xFFFFFFFFU << FC_SRN_SHIFT)

#define FC_SDH_SHIFT                    (0)
#define FC_SDH_MASK                     (0xFFFFFFFFU << FC_SDH_SHIFT)

#define FC_SDL_SHIFT                    (0)
#define FC_SDL_MASK                     (0xFFFFFFFFU << FC_SDL_SHIFT)

#define FC_FTL_WR_FIFO_FULL_SHIFT       (24)
#define FC_FTL_WR_FIFO_FULL_MASK        (0xFFU << FC_FTL_WR_FIFO_FULL_SHIFT)

#define FC_FTL_WR_FIFO_EMPTY_SHIFT      (16)
#define FC_FTL_WR_FIFO_EMPTY_MASK       (0xFFU << FC_FTL_WR_FIFO_EMPTY_SHIFT)

#define FC_FTL_RD_FIFO_FULL_SHIFT       (8)
#define FC_FTL_RD_FIFO_FULL_MASK        (0xFFU << FC_FTL_RD_FIFO_FULL_SHIFT)

#define FC_FTL_RD_FIFO_EMPTY_SHIFT      (0)
#define FC_FTL_RD_FIFO_EMPTY_MASK       (0xFFU << FC_FTL_RD_FIFO_EMPTY_SHIFT)

#define FC_FS_STATUS_DGB_SHIFT          (24)
#define FC_FS_STATUS_DGB_MASK           (0xFU << FC_FS_STATUS_DGB_SHIFT)
#define FC_FS_STATUS_DGB_VMASK          (0xFU)

#define FC_FS_WR_BUF_VALID_SHIFT        (23)
#define FC_FS_WR_BUF_VALID_MASK         (0x1U << FC_FS_WR_BUF_VALID_SHIFT)

#define FC_FS_WR_BUF_CNT_SHIFT          (20)
#define FC_FS_WR_BUF_CNT_MASK           (0x7U << FC_FS_WR_BUF_CNT_SHIFT)
#define FC_FS_WR_BUF_CNT_VMASK          (0x7U)

#define FC_FS_RD_BUF_VALID_SHIFT        (19)
#define FC_FS_RD_BUF_VALID_MASK         (0x1U << FC_FS_RD_BUF_VALID_SHIFT)

#define FC_FS_RD_BUF_CNT_SHIFT          (16)
#define FC_FS_RD_BUF_CNT_MASK           (0x7U << FC_FS_RD_BUF_CNT_SHIFT)
#define FC_FS_RD_BUF_CNT_VMASK          (0x7U)

#define FC_FS_WR_FIFO_CNT_SHIFT         (8)
#define FC_FS_WR_FIFO_CNT_MASK          (0xFFU << FC_FS_WR_FIFO_CNT_SHIFT)
#define FC_FS_WR_FIFO_CNT_VMASK         (0xFFU)

#define FC_FS_RD_FIFO_CNT_SHIFT         (0)
#define FC_FS_RD_FIFO_CNT_MASK          (0xFFU << FC_FS_RD_FIFO_CNT_SHIFT)
#define FC_FS_RD_FIFO_CNT_VMASK         (0xFFU)

#define FC_SIW_CMD_SHIFT                (16)
#define FC_SIW_CMD_MASK                 (0xFFU << FC_SIW_CMD_SHIFT)

#define FC_SIW_ADDR_SHIFT               (8)
#define FC_SIW_ADDR_MASK                (0xFFU << FC_SIW_ADDR_SHIFT)

#define FC_SIW_DUMMY_SHIFT              (0)
#define FC_SIW_DUMMY_MASK               (0xFFU << FC_SIW_DUMMY_SHIFT)

#define FC_WM_SHIFT                     (0)
#define FC_WM_MASK                      (0x1U << FC_WM_SHIFT)

#define FC_SS_SHIFT                     (0)
#define FC_SS_MASK                      (0x1U << FC_SS_SHIFT)

#define FC_IE_CMPL_SHIFT                (12)
#define FC_IE_CMPL_MASK                 (0x1U << FC_IE_CMPL_SHIFT)

#define FC_IE_WR_FIFO_UNDERFLOW_SHIFT   (11)
#define FC_IE_WR_FIFO_UNDERFLOW_MASK    (0x1U << FC_IE_WR_FIFO_UNDERFLOW_SHIFT)

#define FC_IE_WR_FIFO_OVERFLOW_SHIFT    (10)
#define FC_IE_WR_FIFO_OVERFLOW_MASK     (0x1U << FC_IE_WR_FIFO_OVERFLOW_SHIFT)

#define FC_IE_RD_FIFO_UNDERFLOW_SHIFT   (9)
#define FC_IE_RD_FIFO_UNDERFLOW_MASK    (0x1U << FC_IE_RD_FIFO_UNDERFLOW_SHIFT)

#define FC_IE_RD_FIFO_OVERFLOW_SHIFT    (8)
#define FC_IE_RD_FIFO_OVERFLOW_MASK     (0x1U << FC_IE_RD_FIFO_OVERFLOW_SHIFT)

#define FC_IE_WR_FIFO_FULL_SHIFT        (6)
#define FC_IE_WR_FIFO_FULL_MASK         (0x1U << FC_IE_WR_FIFO_FULL_SHIFT)

#define FC_IE_WR_FIFO_EMPTY_SHIFT       (5)
#define FC_IE_WR_FIFO_EMPTY_MASK        (0x1U << FC_IE_WR_FIFO_EMPTY_SHIFT)

#define FC_IE_WR_FIFO_READY_SHIFT       (4)
#define FC_IE_WR_FIFO_READY_MASK        (0x1U << FC_IE_WR_FIFO_READY_SHIFT)

#define FC_IE_RD_FIFO_FULL_SHIFT        (2)
#define FC_IE_RD_FIFO_FULL_MASK         (0x1U << FC_IE_RD_FIFO_FULL_SHIFT)

#define FC_IE_RD_FIFO_EMPTY_SHIFT       (1)
#define FC_IE_RD_FIFO_EMPTY_MASK        (0x1U << FC_IE_RD_FIFO_EMPTY_SHIFT)

#define FC_IE_RD_FIFO_READY_SHIFT       (0)
#define FC_IE_RD_FIFO_READY_MASK        (0x1U << FC_IE_RD_FIFO_READY_SHIFT)

#define FC_IS_CMPL_SHIFT                (12)
#define FC_IS_CMPL_MASK                 (0x1U << FC_IS_CMPL_SHIFT)

#define FC_IS_WR_FIFO_UNDERFLOW_SHIFT   (11)
#define FC_IS_WR_FIFO_UNDERFLOW_MASK    (0x1U << FC_IS_WR_FIFO_UNDERFLOW_SHIFT)

#define FC_IS_WR_FIFO_OVERFLOW_SHIFT    (10)
#define FC_IS_WR_FIFO_OVERFLOW_MASK     (0x1U << FC_IS_WR_FIFO_OVERFLOW_SHIFT)

#define FC_IS_RD_FIFO_UNDERFLOW_SHIFT   (9)
#define FC_IS_RD_FIFO_UNDERFLOW_MASK    (0x1U << FC_IS_RD_FIFO_UNDERFLOW_SHIFT)

#define FC_IS_RD_FIFO_OVERFLOW_SHIFT    (8)
#define FC_IS_RD_FIFO_OVERFLOW_MASK     (0x1U << FC_IS_RD_FIFO_OVERFLOW_SHIFT)

#define FC_IS_WR_FIFO_FULL_SHIFT        (6)
#define FC_IS_WR_FIFO_FULL_MASK         (0x1U << FC_IS_WR_FIFO_FULL_SHIFT)

#define FC_IS_WR_FIFO_EMPTY_SHIFT       (5)
#define FC_IS_WR_FIFO_EMPTY_MASK        (0x1U << FC_IS_WR_FIFO_EMPTY_SHIFT)

#define FC_IS_WR_FIFO_READY_SHIFT       (4)
#define FC_IS_WR_FIFO_READY_MASK        (0x1U << FC_IS_WR_FIFO_READY_SHIFT)

#define FC_IS_RD_FIFO_FULL_SHIFT        (2)
#define FC_IS_RD_FIFO_FULL_MASK         (0x1U << FC_IS_RD_FIFO_FULL_SHIFT)

#define FC_IS_RD_FIFO_EMPTY_SHIFT       (1)
#define FC_IS_RD_FIFO_EMPTY_MASK        (0x1U << FC_IS_RD_FIFO_EMPTY_SHIFT)

#define FC_IS_RD_FIFO_READY_SHIFT       (0)
#define FC_IS_RD_FIFO_READY_MASK        (0x1U << FC_IS_RD_FIFO_READY_SHIFT)

#define FC_SW_SHIFT                     (0)
#define FC_SW_MASK                      (0xFFFFFFFFU << FC_SW_SHIFT)

#define FC_SR_SHIFT                     (0)
#define FC_SR_MASK                      (0xFFFFFFFFU << FC_SR_SHIFT)

#endif /* FLASHCTRL_ARCH_V2 */

typedef enum {
	FC_CYCLEBITS_0,
	FC_CYCLEBITS_1,
	FC_CYCLEBITS_2,
	FC_CYCLEBITS_4
} FC_CycleBits;

typedef struct {
	uint8_t cs_begin;               /*!< cs active to valid clk edge setup minimum time */
	uint8_t cs_over;                /*!< valid clk edge to cs active hold minimum time */
	uint8_t cs_deselect;            /*!< cs minimum deselect time after read */
	uint8_t cmd_over;
	uint8_t addr_over;
	uint8_t dummy_over;
	uint8_t data;                   /*!< delay n half cycle */
} Flash_Ctrl_DelayCycle;

typedef enum {
	XIP_MODE_NORMAL,
	XIP_MODE_FAST,
	XIP_MODE_DUAL_O,
	XIP_MODE_DUAL_IO,
	XIP_MODE_QUAD_O,
	XIP_MODE_QUAD_IO
} XIP_Mode;


typedef struct XIP_Instruction{
	uint8_t cmd;                    /*!< command */
	FC_CycleBits cmd_line;          /*!< line mode of command */
	FC_CycleBits addr_line;         /*!< line mode of address */
	FC_CycleBits dummy_line;        /*!< line mode of dummy */
	FC_CycleBits data_line;         /*!< line mode of data */
	uint32_t dum_btyes;             /*!< dummy length */
//	uint32_t dummyh;
//	uint32_t dummyl;
} XIP_Instruction;

typedef struct {
	uint32_t addr;                  /*!< XIP code started address in flash */
	uint32_t freq;                  /*!< flash working frequency */
	Flash_Ctrl_DelayCycle delay;    /*!< board delay config */
	XIP_Instruction ins;            /*!< XIP read instruction */
	bool cont_mode;                 /*!< continue mode or not */
} XIP_Config;

typedef struct Flashc_Config
{
	uint32_t freq;                  /*!< flash working frequency */
//	uint32_t t_shsl_ns;             /*!< flash t_shsl parameter. for calculate the cs delay. */
} Flashc_Config;

typedef struct Flashc_Delay
{
	uint32_t t_shsl_ns;             /*!< flash t_shsl parameter. for calculate the cs delay. */
} Flashc_Delay;

typedef struct FC_InstructionField
{
	uint8_t *pdata;                 /*!< instruction field: data */
	uint32_t len;                   /*!< instruction field: data len */
	FC_CycleBits line;              /*!< instruction field: line mode */
} FC_InstructionField;

typedef enum Flashc_Commands
{
	FC_CMD_ENABLE_32BITADDR_MODE = 0,
} Flashc_Commands;

struct flash_controller;

#if FLASHCTRL_ARCH_V2

typedef struct
{
	__IO uint32_t MEM_COM_CONFG;					/* ,	 Address offset: 0x000	 */
	__IO uint32_t OPI_CTRL_CMM_CONFG;				/* ,	 Address offset: 0x004	 */
	__IO uint32_t CACHE_RLVT_CONFG;					/* ,	 Address offset: 0x008	 */
	__IO uint32_t MEM_AC_CHR_TIMING_CONFG;			/* ,	 Address offset: 0x00C	 */
	__IO uint32_t CBUS_RD_OPRT_CONFG;				/* ,	 Address offset: 0x010	 */
	__IO uint32_t CBUS_WR_OPRT_CONFG;				/* ,	 Address offset: 0x014	 */
	__IO uint32_t CBUS_RD_DUMMY_DATA_TOP_HALF;		/* ,	 Address offset: 0x018	 */
	__IO uint32_t CBUS_RD_DUMMY_DATA_BUTT_HALF;		/* ,	 Address offset: 0x01C	 */
	__IO uint32_t CBUS_WR_DUMMY_DATA_TOP_HALF;		/* ,	 Address offset: 0x020	 */
	__IO uint32_t CBUS_WR_DUMMY_DATA_BUTT_HALF;		/* ,	 Address offset: 0x024	 */
	__IO uint32_t CBUS_IO_SW_WAIT_TIME;				/* ,	 Address offset: 0x028	 */
	__IO uint32_t SBUS_RW_OPRT_CONFG;				/* ,	 Address offset: 0x02C	 */
	__IO uint32_t SBUS_ADDR_CONFG;					/* ,	 Address offset: 0x030	 */
	__IO uint32_t SBUS_DUMMY_DATA_TOP_HALF;			/* ,	 Address offset: 0x034	 */
	__IO uint32_t SBUS_DUMMY_DATA_BUTT_HALF;		/* ,	 Address offset: 0x038	 */
	__IO uint32_t SBUS_IO_SW_WAIT_TIME;				/* ,	 Address offset: 0x03C	 */
	__IO uint32_t SBUS_WR_DATA_BYTE_NUM;			/* ,	 Address offset: 0x040	 */
	__IO uint32_t SBUS_RD_DATA_BYTE_NUM;			/* ,	 Address offset: 0x044	 */
	__IO uint32_t SBUS_START_SEND_REG;				/* ,	 Address offset: 0x048	 */
	__IO uint32_t FIFO_TRIGGER_LEVEL;				/* ,	 Address offset: 0x04C	 */
	__I	 uint32_t FIFO_STATUS_REG;					/* ,	 Address offset: 0x050	 */
	__IO uint32_t INT_ENABLE_REG;					/* ,	 Address offset: 0x054	 */
	__IO uint32_t INT_STATUS_REG;					/* ,	 Address offset: 0x058	 */
	__IO uint32_t XIP_WARP_MODE_EXE_IDCT;			/* ,	 Address offset: 0x05C	 */
	__I	 uint32_t MEM_CTRL_DEBUG_STATE;				/* ,	 Address offset: 0x060	 */
	__I	 uint32_t DEBUG_CNT_SBUS;					/* ,	 Address offset: 0x064	 */
	__I	 uint32_t DEBUG_CNT_CBUS;					/* ,	 Address offset: 0x068	 */
	__IO uint32_t RESERVE6C[1];						/* ,	 Address offset: 0x06C	 */
	__I	 uint32_t PSRAM_COM_CFG;					/* ,	 Address offset: 0x070	 */
	__I	 uint32_t PSRAM_LAT_CFG;					/* ,	 Address offset: 0x074	 */
	__I	 uint32_t PSRAM_TIM_CFG;					/* ,	 Address offset: 0x078	 */
	__IO uint32_t RESERVE7C[1];						/* ,	 Address offset: 0x07C	 */
	__IO uint32_t START_ADDR0;						/* ,	 Address offset: 0x080	 */
	__IO uint32_t END_ADDR0;						/* ,	 Address offset: 0x084	 */
	__IO uint32_t BIAS_ADDR0;						/* ,	 Address offset: 0x088	 */
	__IO uint32_t RESERVE8C[1];
	__IO uint32_t START_ADDR1;						/* ,	 Address offset: 0x090	 */
	__IO uint32_t END_ADDR1;						/* ,	 Address offset: 0x094	 */
	__IO uint32_t BIAS_ADDR1;						/* ,	 Address offset: 0x098	 */
	__IO uint32_t RESERVE9C[1];
	__IO uint32_t START_ADDR2;						/* ,	 Address offset: 0x0A0	 */
	__IO uint32_t END_ADDR2;						/* ,	 Address offset: 0x0A4	 */
	__IO uint32_t BIAS_ADDR2;						/* ,	 Address offset: 0x0A8	 */
	__IO uint32_t RESERVEAC[1];
	__IO uint32_t START_ADDR3;						/* ,	 Address offset: 0x0B0	 */
	__IO uint32_t END_ADDR3;						/* ,	 Address offset: 0x0B4	 */
	__IO uint32_t BIAS_ADDR3;						/* ,	 Address offset: 0x0B8	 */
	__IO uint32_t RESERVEBC[1];
	__IO uint32_t START_ADDR4;						/* ,	 Address offset: 0x0C0	 */
	__IO uint32_t END_ADDR4;						/* ,	 Address offset: 0x0C4	 */
	__IO uint32_t BIAS_ADDR4;						/* ,	 Address offset: 0x0C8	 */
	__IO uint32_t RESERVECC[1];
	__IO uint32_t START_ADDR5;						/* ,	 Address offset: 0x0D0	 */
	__IO uint32_t END_ADDR5;						/* ,	 Address offset: 0x0D4	 */
	__IO uint32_t BIAS_ADDR5;						/* ,	 Address offset: 0x0D8	 */
	__IO uint32_t RESERVEDC[1];
	__IO uint32_t START_ADDR6;						/* ,	 Address offset: 0x0E0	 */
	__IO uint32_t END_ADDR6;						/* ,	 Address offset: 0x0E4	 */
	__IO uint32_t BIAS_ADDR6;						/* ,	 Address offset: 0x0E8	 */
	__IO uint32_t RESERVEEC[1];
	__IO uint32_t START_ADDR7;						/* ,	 Address offset: 0x0F0	 */
	__IO uint32_t END_ADDR7;						/* ,	 Address offset: 0x0F4	 */
	__IO uint32_t BIAS_ADDR7;						/* ,	 Address offset: 0x0F8	 */
	__IO uint32_t RESERVEFC[1];
	__IO uint32_t SBUS_WR_DATA_REG;					/* ,	 Address offset: 0x100	 */
	__IO uint32_t RESERVE101[63];
	__I	 uint32_t SBUS_RD_DATA_REG;					/* ,	 Address offset: 0x200	 */
} OPI_MEM_CTRL_T;

#define OPI_MEM_CTRL ((OPI_MEM_CTRL_T *)FLASH_CTRL_BASE)

/* Memory Controller Common Configuration */
#define FLASHC_CBUS_RW_ENABLE_SHIFT 					        0
#define FLASHC_CBUS_RW_ENABLE_MASK 					            (0x1U << FLASHC_CBUS_RW_ENABLE_SHIFT)
#define FLASHC_XIP_DEBUG_SHIFT 						            1
#define FLASHC_XIP_DEBUG_MASK 							        (0x1U << FLASHC_XIP_DEBUG_SHIFT)
#define FLASHC_XIP_ENABLE_SHIFT 						        2
#define FLASHC_XIP_ENABLE_MASK 						            (0x1U << FLASHC_XIP_ENABLE_SHIFT)
#define FLASHC_WRAP_AROUND_ENABLE_SHIFT 				        3
#define FLASHC_WRAP_AROUND_ENABLE_MASK 				            (0x1U << FLASHC_WRAP_AROUND_ENABLE_SHIFT)
#define FLASHC_FLASH_PSRAM_SELECT_SHIFT 				        4
#define FLASHC_FLASH_PSRAM_SELECT_MASK 				            (0x1U << FLASHC_FLASH_PSRAM_SELECT_SHIFT)
#define FLASHC_REV_FIFO_RESET_SHIFT 					        5
#define FLASHC_REV_FIFO_RESET_MASK 					            (0x1U << FLASHC_REV_FIFO_RESET_SHIFT)
#define FLASHC_TRAN_FIFO_RESET_SHIFT 					        6
#define FLASHC_TRAN_FIFO_RESET_MASK 					        (0x1U << FLASHC_TRAN_FIFO_RESET_SHIFT)
#define FLASHC_SBUS_HREADY_WAITTIME_OUT_ENABLE_SHIFT 	        7
#define FLASHC_SBUS_HREADY_WAITTIME_OUT_ENABLE_MASK 	        (0x1U << FLASHC_SBUS_HREADY_WAITTIME_OUT_ENABLE_SHIFT)
#define FLASHC_SBUS_HREADY_ADJ_WAITTIME_OUT_SHIFT 		        8
#define FLASHC_SBUS_HREADY_ADJ_WAITTIME_OUT_MASK 		        (0xFFU<< FLASHC_SBUS_HREADY_ADJ_WAITTIME_OUT_SHIFT)
#define FLASHC_IO7_VACANCY_OUT_SHIFT 					        16
#define FLASHC_IO7_VACANCY_OUT_MASK 					        (0x3U << FLASHC_IO7_VACANCY_OUT_SHIFT)
#define FLASHC_IO6_VACANCY_OUT_SHIFT 					        18
#define FLASHC_IO6_VACANCY_OUT_MASK 					        (0x3U << FLASHC_IO6_VACANCY_OUT_SHIFT)
#define FLASHC_IO5_VACANCY_OUT_SHIFT 					        20
#define FLASHC_IO5_VACANCY_OUT_MASK 					        (0x3U << FLASHC_IO5_VACANCY_OUT_SHIFT)
#define FLASHC_IO4_VACANCY_OUT_SHIFT 					        22
#define FLASHC_IO4_VACANCY_OUT_MASK 					        (0x3U << FLASHC_IO4_VACANCY_OUT_SHIFT)
#define FLASHC_IO3_VACANCY_OUT_SHIFT 					        24
#define FLASHC_IO3_VACANCY_OUT_MASK 					        (0x3U << FLASHC_IO3_VACANCY_OUT_SHIFT)
#define FLASHC_IO2_VACANCY_OUT_SHIFT 					        26
#define FLASHC_IO2_VACANCY_OUT_MASK 					        (0x3U << FLASHC_IO2_VACANCY_OUT_SHIFT)
#define FLASHC_IO1_VACANCY_OUT_SHIFT 					        28
#define FLASHC_IO1_VACANCY_OUT_MASK 					        (0x3U << FLASHC_IO1_VACANCY_OUT_SHIFT)
#define FLASHC_ADDR_SIZE_MODE 					                30
#define FLASHC_ADDR_SIZE_MODE_MASK 					            (0x3U << FLASHC_ADDR_SIZE_MODE)

/* OPI Controller Common Configuration */
#define FLASHC_SPI_CPHA_CTRL_SHIFT 		                        0
#define FLASHC_SPI_CPHA_CTRL_MASK 		                        (0x1U << FLASHC_SPI_CPHA_CTRL_SHIFT)
#define FLASHC_SPI_CPOL_CTRL_SHIFT 		                        1
#define FLASHC_SPI_CPOL_CTRL_MASK 		                        (0x1U << FLASHC_SPI_CPOL_CTRL_SHIFT)
#define FLASHC_FIRST_RCV_BIT_SLT_SHIFT 	                        4
#define FLASHC_FIRST_RCV_BIT_SLT_MASK 	                        (0x1U << FLASHC_FIRST_RCV_BIT_SLT_SHIFT)
#define FLASHC_FLASH_CS_POL_SHIFT 			                    8
#define FLASHC_FLASH_CS_POL_MASK 		                        (0x1U << FLASHC_FLASH_CS_POL_SHIFT)
#define FLASHC_WAIT_HALF_CYCLE_SHIFT 		                    12
#define FLASHC_WAIT_HALF_CYCLE_MASK 		                    (0x3U << FLASHC_WAIT_HALF_CYCLE_SHIFT)

/* Cache Relevant Configuration */
#define FLASHC_CACHE_LINE_LEN_CONFG_SHIFT 				        0
#define FLASHC_CACHE_LINE_LEN_CONFG_MASK 				        (0x3U << FLASHC_CACHE_LINE_LEN_CONFG_SHIFT)
#define FLASHC_CBUS_WR_SIZE_SELECT_SHIFT 				        2
#define FLASHC_CBUS_WR_SIZE_SELECT_MASK 				        (0x1U << FLASHC_CBUS_WR_SIZE_SELECT_SHIFT)
#define FLASHC_CBUS_HREADY_TIME_OUT_SHIFT 				        3
#define FLASHC_CBUS_HREADY_TIME_OUT_MASK 				        (0x1U << FLASHC_CBUS_HREADY_TIME_OUT_SHIFT)
#define FLASHC_CBUS_HREADY_ADY_WAIT_TIME_OUT_SHIFT 		        4
#define FLASHC_CBUS_HREADY_ADY_WAIT_TIME_OUT_MASK 	            (0xFFU << FLASHC_CBUS_HREADY_ADY_WAIT_TIME_OUT_SHIFT)

/* Memory AC Charachter Timing Configuration */
#define FLASHC_CBUS_SHSL_SHIFT 		                            0
#define FLASHC_CBUS_SHSL_MASK 			                        (0xFFU << FLASHC_CBUS_SHSL_SHIFT)
#define FLASHC_SBUS_SHSL_SHIFT 		                            8
#define FLASHC_SBUS_SHSL_MASK 			                        (0xFFU << FLASHC_SBUS_SHSL_SHIFT)
#define FLASHC_SPI_FLASH_CHSH_SHIFT 	                        16
#define FLASHC_SPI_FLASH_CHSH_MASK 		                        (0xFFU << FLASHC_SPI_FLASH_CHSH_SHIFT)
#define FLASHC_SPI_FLASH_SLCH_SHIFT 	                        24
#define FLASHC_SPI_FLASH_SLCH_MASK 		                        (0xFFU << FLASHC_SPI_FLASH_SLCH_SHIFT)

/* Code-bus Read Operation Configuration */
#define FLASHC_GET_BIT_DATA_EVERY_CYCLE_SHIFT 		            0
#define FLASHC_GET_BIT_DATA_EVERY_CYCLE_MASK 	                (0x7U << FLASHC_GET_BIT_DATA_EVERY_CYCLE_SHIFT)
#define FLASHC_SEND_BYTE_DUMMY_NUM_SHIFT 			            4
#define FLASHC_SEND_BYTE_DUMMY_NUM_MASK 			            (0x3FU<< FLASHC_SEND_BYTE_DUMMY_NUM_SHIFT)
#define FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_SHIFT 	            12
#define FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_MASK 	                (0x7U << FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_SHIFT)
#define FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_SHIFT 	                16
#define FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_MASK 	                (0x7U << FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_SHIFT)
#define FLASHC_SEND_BIT_CMD_EVERY_CYCLE_SHIFT 		            20
#define FLASHC_SEND_BIT_CMD_EVERY_CYCLE_MASK 	                (0x7U << FLASHC_SEND_BIT_CMD_EVERY_CYCLE_SHIFT)
#define FLASHC_RD_CMD_SEND_SHIFT 					            24
#define FLASHC_RD_CMD_SEND_MASK 					            (0xFFU<< FLASHC_RD_CMD_SEND_SHIFT)

/* Code-bus Write Operation Configuration */
#define FLASHC_SEND_BIT_DATA_EVERY_CYCLE_SHIFT 	                0
#define FLASHC_SEND_BIT_DATA_EVERY_CYCLE_MASK 	                (0x7U << FLASHC_SEND_BIT_DATA_EVERY_CYCLE_SHIFT)
#define FLASHC_SEND_BYTE_DUMMY_NUM_SHIFT 			            4
#define FLASHC_SEND_BYTE_DUMMY_NUM_MASK 		    	        (0x3FU<< FLASHC_SEND_BYTE_DUMMY_NUM_SHIFT)
#define FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_SHIFT 	            12
#define FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_MASK      	        (0x7U << FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_SHIFT)
#define FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_SHIFT 	                16
#define FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_MASK 	                (0x7U << FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_SHIFT)
#define FLASHC_SEND_BIT_CMD_EVERY_CYCLE_SHIFT 		            20
#define FLASHC_SEND_BIT_CMD_EVERY_CYCLE_MASK 	                (0x7U << FLASHC_SEND_BIT_CMD_EVERY_CYCLE_SHIFT)
#define FLASHC_WR_CMD_SEND_SHIFT 					            24
#define FLASHC_WR_CMD_SEND_MASK 				    	        (0xFFU<< FLASHC_WR_CMD_SEND_SHIFT)

/* Code-bus Read Dummy Data Top Half*/
#define FLASHC_RD_DUMMY_DATA_TOP_HALF_SHIFT                     0
#define FLASHC_RD_DUMMY_DATA_TOP_HALF_MASK                      (0xFFFFFFFFU << FLASHC_RD_DUMMY_DATA_TOP_HALF_SHIFT)

/* Code-bus Read Dummy Data Buttom Half */
#define FLASHC_RD_DUMMY_DATA_BUTTOM_HALF_SHIFT                  0
#define FLASHC_RD_DUMMY_DATA_BUTTOM_HALF_MASK                   (0xFFFFFFFFU << FLASHC_RD_DUMMY_DATA_BUTTOM_HALF_SHIFT)

/* Code-bus Write Dummy Data Top Half */
#define FLASHC_WR_DUMMY_DATA_TOP_HALF_SHIFT                     0
#define FLASHC_WR_DUMMY_DATA_TOP_HALF_MASK                      (0xFFFFFFFFU << FLASHC_WR_DUMMY_DATA_TOP_HALF_SHIFT)

/* Code-bus Write Dummy Data Buttom Half */
#define FLASHC_WR_DUMMY_DATA_BUTTOM_HALF_SHIFT                  0
#define FLASHC_WR_DUMMY_DATA_BUTTOM_HALF_MASK                   (0xFFFFFFFFU << FLASHC_WR_DUMMY_DATA_BUTTOM_HALF_SHIFT)

/* Code-bus IO Switch Wait Time */
#define FLASHC_CBUS_DUMMY_WAIT_CYCLE_SHIFT 	                    0
#define FLASHC_CBUS_DUMMY_WAIT_CYCLE_MASK                      (0xFFU << FLASHC_CBUS_DUMMY_WAIT_CYCLE_SHIFT)
#define FLASHC_CBUS_ADDR_WAIT_CYCLE_SHIFT 	                    8
#define FLASHC_CBUS_ADDR_WAIT_CYCLE_MASK                       (0xFFU << FLASHC_CBUS_ADDR_WAIT_CYCLE_SHIFT)
#define FLASHC_CBUS_CMD_WAIT_CYCLE_SHIFT 		                16
 #define FLASHC_CBUS_CMD_WAIT_CYCLE_MASK 	                    (0xFFU << FLASHC_CBUS_CMD_WAIT_CYCLE_SHIFT)

/* System-bus Read/Write Operation Configuration */
#define FLASHC_GS_BIT_EVERY_CYCLE_SHIFT 			            0
#define FLASHC_GS_BIT_EVERY_CYCLE_MASK 			                (0x7U << FLASHC_GS_BIT_EVERY_CYCLE_SHIFT)
#define FLASHC_SEND_BYTE_DUMMY_SHIFT 				            4
#define FLASHC_SEND_BYTE_DUMMY_MASK 				            (0x3FU<< FLASHC_SEND_BYTE_DUMMY_SHIFT)
#define FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_SHIFT 	            12
#define FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_MASK 	                (0x7U << FLASHC_SEND_BIT_DUMMY_EVERY_CYCLE_SHIFT)
#define FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_SHIFT 	                16
#define FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_MASK 	                (0x7U << FLASHC_SEND_BIT_ADDR_EVERY_CYCLE_SHIFT)
#define FLASHC_SEND_BIT_CMD_EVERY_CYCLE_SHIFT 		            20
#define FLASHC_SEND_BIT_CMD_EVERY_CYCLE_MASK 	                (0x7U << FLASHC_SEND_BIT_CMD_EVERY_CYCLE_SHIFT)
#define FLASHC_RW_CMD_SEND_SHIFT 					            24
#define FLASHC_RW_CMD_SEND_MASK 					            (0xFFU<< FLASHC_RW_CMD_SEND_SHIFT)

/* System-bus Address Configuration*/
#define FLASHC_SBUS_ADDR_SHIFT                                  0
#define FLASHC_SBUS_ADDR_MASK                                   (0xFFFFFFFFU << FLASHC_SBUS_ADDR_SHIFT)

/* System-bus Dummy Data Top half */
#define FLASHC_SBUS_DUMMY_TOP_HALF_SHIFT                        0
#define FLASHC_SBUS_DUMMY_TOP_HALF_MASK                         (0xFFFFFFFFU << FLASHC_SBUS_DUMMY_TOP_HALF_SHIFT)

/* System-bus Dummy Data Buttom half */
#define FLASHC_SBUS_DUMMY_BUTTOM_HALF_SHIFT                     0
#define FLASHC_SBUS_DUMMY_BUTTOM_HALF_MASK                      (0xFFFFFFFFU << FLASHC_SBUS_DUMMY_BUTTOM_HALF_SHIFT)

/* System-bus IO Switch Wait Time */
#define FLASHC_SBUS_DUMMY_WAIT_CYCLE_SHIFT 	                    0
#define FLASHC_SBUS_DUMMY_WAIT_CYCLE_MASK 	                    (0xFFU << FLASHC_SBUS_DUMMY_WAIT_CYCLE_SHIFT)
#define FLASHC_SBUS_ADDR_WAIT_CYCLE_SHIFT 	                    8
#define FLASHC_SBUS_ADDR_WAIT_CYCLE_MASK 	                    (0xFFU << FLASHC_SBUS_ADDR_WAIT_CYCLE_SHIFT)
#define FLASHC_SBUS_CMD_WAIT_CYCLE_SHIFT 		                16
#define FLASHC_SBUS_CMD_WAIT_CYCLE_MASK 	                    (0xFFU << FLASHC_SBUS_CMD_WAIT_CYCLE_SHIFT)

/* System-bus Write Data Byte Number */
#define FLASHC_SBUS_WR_BYTE_SHIFT                               0
#define FLASHC_SBUS_WR_BYTE_MASK                                (0xFFFFFFFFU << FLASHC_SBUS_WR_BYTE_SHIFT)

/* System-bus Read Data Byte Number */
#define FLASHC_SBUS_RD_BYTE_SHIFT                               0
#define FLASHC_SBUS_RD_BYTE_MASK                                (0xFFFFFFFFU << FLASHC_SBUS_RD_BYTE_SHIFT)

/* System-bus Start Send Register */
#define FLASHC_ENABLE_SBUS_SHIFT                                0
#define FLASHC_ENABLE_SBUS_MASK 		                        (0x1U << FLASHC_ENABLE_SBUS_SHIFT)

/* FIFO Trigger Level */
#define FLASHC_RD_FIFO_EMPTY_REQ_SHIFT 		                    0
#define FLASHC_RD_FIFO_EMPTY_REQ_MASK 	                        (0xFFU << FLASHC_RD_FIFO_EMPTY_REQ_SHIFT)
#define FLASHC_RD_FIFO_FULL_REQ_SHIFT 		                    8
#define FLASHC_RD_FIFO_FULL_REQ_MASK 		                    (0xFFU << FLASHC_RD_FIFO_FULL_REQ_SHIFT)
#define FLASHC_WR_FIFO_EMPTY_REQ_SHIFT 		                    16
#define FLASHC_WR_FIFO_EMPTY_REQ_MASK 	                        (0xFFU << FLASHC_WR_FIFO_EMPTY_REQ_SHIFT)
#define FLASHC_WR_FIFO_FULL_REQ_SHIFT 	                        24
#define FLASHC_WR_FIFO_FULL_REQ_MASK 		                    (0xFFU << FLASHC_WR_FIFO_FULL_REQ_SHIFT)

/* FIFO Status Register */
#define FLASHC_RD_FIFO_COUNTER_SHIFT 		                    0
#define FLASHC_RD_FIFO_COUNTER_MASK 		                    (0xFFU <<FLASHC_RD_FIFO_COUNTER_SHIFT)
#define FLASHC_RD_FIFO_COUNTER_VMASK 		                    (0xFFU)
#define FLASHC_WR_FIFO_COUNTER_SHIFT 		                    8
#define FLASHC_WR_FIFO_COUNTER_MASK 		                    (0xFFU <<FLASHC_WR_FIFO_COUNTER_SHIFT)
#define FLASHC_WR_FIFO_COUNTER_VMASK 		                    (0xFFU)
#define FLASHC_RD_BUFF_COUNTER_SHIFT 		                    16
#define FLASHC_RD_BUFF_COUNTER_MASK 		                    (0x7U << FLASHC_RD_BUFF_COUNTER_SHIFT)
#define FLASHC_RD_BUFF_COUNTER_VMASK 		                    (0x7U)
#define FLASHC_RD_BUFF_RD_STATUS_SHIFT 		                    19
#define FLASHC_RD_BUFF_RD_STATUS_MASK                           (0x1U << FLASHC_RD_BUFF_RD_STATUS_SHIFT)
#define FLASHC_WR_BUFF_COUNTER_VMASK 		                    (0x7U)
#define FLASHC_WR_BUFF_COUNTER_SHIFT 		                    20
#define FLASHC_WR_BUFF_COUNTER_MASK 		                    (0x7U << FLASHC_WR_BUFF_COUNTER_SHIFT)
#define FLASHC_RD_BUFF_RD_STATUS_VMASK 	                        (0x1U)
#define FLASHC_WR_BUFF_RD_STATUS_SHIFT 		                    23
#define FLASHC_WR_BUFF_RD_STATUS_MASK 	                        (0x1U << FLASHC_WR_BUFF_RD_STATUS_SHIFT)
#define FLASHC_WR_BUFF_RD_STATUS_VMASK 	                        (0x1U)

/* Interrupt Enable Register */
#define FLASHC_RD_FIFO_REQ_INT_ENABLE_SHIFT 			        0
#define FLASHC_RD_FIFO_REQ_INT_ENABLE_MASK 			            (0x1U << FLASHC_RD_FIFO_REQ_INT_ENABLE_SHIFT)
#define FLASHC_RD_FIFO_EMPTY_INT_ENABLE_SHIFT 			        1
#define FLASHC_RD_FIFO_EMPTY_INT_ENABLE_MASK 			        (0x1U << FLASHC_RD_FIFO_EMPTY_INT_ENABLE_SHIFT)
#define FLASHC_RD_FIFO_FULL_INT_ENABLE_SHIFT 			        2
#define FLASHC_RD_FIFO_FULL_INT_ENABLE_MASK 			        (0x1U << FLASHC_RD_FIFO_FULL_INT_ENABLE_SHIFT)
#define FLASHC_WR_FIFO_REQ_INT_ENABLE_SHIFT 			        4
#define FLASHC_WR_FIFO_REQ_INT_ENABLE_MASK 			            (0x1U << FLASHC_WR_FIFO_REQ_INT_ENABLE_SHIFT)
#define FLASHC_WR_FIFO_EMPTY_INT_ENABLE_SHIFT 			        5
#define FLASHC_WR_FIFO_EMPTY_INT_ENABLE_MASK 			        (0x1U << FLASHC_WR_FIFO_EMPTY_INT_ENABLE_SHIFT)
#define FLASHC_WR_FIFO_FULL_INT_ENABLE_SHIFT 			        6
#define FLASHC_WR_FIFO_FULL_INT_ENABLE_MASK 			        (0x1U << FLASHC_WR_FIFO_FULL_INT_ENABLE_SHIFT)
#define FLASHC_RD_FIFO_OVERFLOW_INT_ENABLE_SHIFT 		        8
#define FLASHC_RD_FIFO_OVERFLOW_INT_ENABLE_MASK		            (0x1U << FLASHC_RD_FIFO_OVERFLOW_INT_ENABLE_SHIFT)
#define FLASHC_RD_FIFO_UNDERFLOW_INT_ENABLE_SHIFT 		        9
#define FLASHC_RD_FIFO_UNDERFLOW_INT_ENABLE_MASK 		        (0x1U << FLASHC_RD_FIFO_UNDERFLOW_INT_ENABLE_SHIFT)
#define FLASHC_WR_FIFO_OVERFLOW_INT_ENABLE_SHIFT 		        10
#define FLASHC_WR_FIFO_OVERFLOW_INT_ENABLE_MASK 		        (0x1U << FLASHC_WR_FIFO_OVERFLOW_INT_ENABLE_SHIFT)
#define FLASHC_WR_FIFO_UNDERFLOW_INT_ENABLE_SHIFT 		        11
#define FLASHC_WR_FIFO_UNDERFLOW_INT_ENABLE_MASK 		        (0x1U << FLASHC_WR_FIFO_UNDERFLOW_INT_ENABLE_SHIFT)
#define FLASHC_TRAN_COMPLETED_INT_ENABLE_SHIFT 			        12
#define FLASHC_TRAN_COMPLETED_INT_ENABLE_MASK 		            (0x1U << FLASHC_TRAN_COMPLETED_INT_ENABLE_SHIFT)
#define FLASHC_SBUS_HREADY_TIME_OUT_INT_ENABLE_SHIFT 	        13
#define FLASHC_SBUS_HREADY_TIME_OUT_INT_ENABLE_MASK 	        (0x1U << FLASHC_SBUS_HREADY_TIME_OUT_INT_ENABLE_SHIFT)

/* Interrupt Status Register */
#define FLASHC_RD_FIFO_REQ_INT_FLAG_SHIFT 				        0
#define FLASHC_RD_FIFO_REQ_INT_FLAG_MASK 				        (0x1U << FLASHC_RD_FIFO_REQ_INT_FLAG_SHIFT)
#define FLASHC_RD_FIFO_EMPTY_INT_FLAG_SHIFT 			        1
#define FLASHC_RD_FIFO_EMPTY_INT_FLAG_MASK 			            (0x1U << FLASHC_RD_FIFO_EMPTY_INT_FLAG_SHIFT)
#define FLASHC_RD_FIFO_FULL_INT_FLAG_SHIFT 				        2
#define FLASHC_RD_FIFO_FULL_INT_FLAG_MASK 			            (0x1U << FLASHC_RD_FIFO_FULL_INT_FLAG_SHIFT)
#define FLASHC_WR_FIFO_REQ_INT_FLAG_SHIFT 				        4
#define FLASHC_WR_FIFO_REQ_INT_FLAG_MASK 				        (0x1U << FLASHC_WR_FIFO_REQ_INT_FLAG_SHIFT)
#define FLASHC_WR_FIFO_EMPTY_INT_FLAG_SHIFT 			        5
#define FLASHC_WR_FIFO_EMPTY_INT_FLAG_MASK 			            (0x1U << FLASHC_WR_FIFO_EMPTY_INT_FLAG_SHIFT)
#define FLASHC_WR_FIFO_FULL_INT_FLAG_SHIFT 				        6
#define FLASHC_WR_FIFO_FULL_INT_FLAG_MASK 			            (0x1U << FLASHC_WR_FIFO_FULL_INT_FLAG_SHIFT)
#define FLASHC_RD_FIFO_OVERFLOW_INT_FLAG_SHIFT 		            8
#define FLASHC_RD_FIFO_OVERFLOW_INT_FLAG_MASK 		            (0x1U << FLASHC_RD_FIFO_OVERFLOW_INT_FLAG_SHIFT)
#define FLASHC_RD_FIFO_UNDERFLOW_INT_FLAG_SHIFT 		        9
#define FLASHC_RD_FIFO_UNDERFLOW_INT_FLAG_MASK 		            (0x1U << FLASHC_RD_FIFO_UNDERFLOW_INT_FLAG_SHIFT)
#define FLASHC_WR_FIFO_OVERFLOW_INT_FLAG_SHIFT 			        10
#define FLASHC_WR_FIFO_OVERFLOW_INT_FLAG_MASK 		            (0x1U << FLASHC_WR_FIFO_OVERFLOW_INT_FLAG_SHIFT)
#define FLASHC_WR_FIFO_UNDERFLOW_INT_FLAG_SHIFT 	            11
#define FLASHC_WR_FIFO_UNDERFLOW_INT_FLAG_MASK 		            (0x1U << FLASHC_WR_FIFO_UNDERFLOW_INT_FLAG_SHIFT)
#define FLASHC_TRAN_COMPLETED_INT_FLAG_SHIFT 			        12
#define FLASHC_TRAN_COMPLETED_INT_FLAG_MASK 			        (0x1U << FLASHC_TRAN_COMPLETED_INT_FLAG_SHIFT)
#define FLASHC_SBUS_HREADY_TIME_OUT_INT_FLAG_SHIFT 		        13
#define FLASHC_SBUS_HREADY_TIME_OUT_INT_FLAG_MASK 	            (0x1U << FLASHC_SBUS_HREADY_TIME_OUT_INT_FLAG_SHIFT)

/* XIP/Wrap Mode Executed Indication*/
#define FLASHC_XIP_NOT_EFFECT_MEM_SHIFT 		                0
#define FLASHC_XIP_NOT_EFFECT_MEM_MASK 		                    (0x1U << FLASHC_XIP_NOT_EFFECT_MEM_SHIFT)
#define FLASHC_WRAP_NOT_EFFECT_MEM_SHIFT 		                1
#define FLASHC_WRAP_NOT_EFFECT_MEM_MASK 	                    (0x1U << FLASHC_WRAP_NOT_EFFECT_MEM_SHIFT)

/* Memory Controller Debug state */
#define FLASHC_MEM_CTRL_STATUE_DEBUG_SHIFT 	                    0
#define FLASHC_MEM_CTRL_STATUE_DEBUG_MASK 	                    (0xFU << FLASHC_MEM_CTRL_STATUE_DEBUG_SHIFT)
#define FLASHC_MEM_CTRL_STATUE_DEBUG_VMASK 	                    (0xFU)

/*Memory Controller SBUS Debug Count*/
#define FLASHC_SBUS_WR_COUNT_SHIFT                              0
#define FLASHC_SBUS_WR_COUNT_MASK                               (0xFFFF << FLASHC_SBUS_WR_COUNT_SHIFT)
#define FLASHC_SBUS_RD_COUNT_SHIFT                              16
#define FLASHC_SBUS_RD_COUNT_MASK                               (0xFFFF << FLASHC_SBUS_RD_COUNT_SHIFT)

/*Memory Controller CBUS Debug Count*/
#define FLASHC_CBUS_WR_COUNT_SHIFT                              0
#define FLASHC_CBUS_WR_COUNT_MASK                               (0xFFFF << FLASHC_CBUS_WR_COUNT_SHIFT)
#define FLASHC_CBUS_RD_COUNT_SHIFT                              16
#define FLASHC_CBUS_RD_COUNT_MASK                               (0xFFFF << FLASHC_CBUS_RD_COUNT_SHIFT)

/*PSRAM Common Configure Register*/
#define FLASHC_CMD_HOLD_HALF_CYCLE_SHIFT  		                0
#define FLASHC_CMD_HOLD_HALF_CYCLE_MASK  		                (0x1 << FLASHC_CMD_HOLD_HALF_CYCLE_SHIFT)
#define FLASHC_WORK_DOUBLE_CLK_EDGE_SHIFT 		                1
#define FLASHC_WORK_DOUBLE_CLK_EDGE_MASK  	                    (0x1 << FLASHC_WORK_DOUBLE_CLK_EDGE_SHIFT)
#define FLASHC_OUTPUT_CLK_SGNL_STATE_SHIFT  	                2
#define FLASHC_OUTPUT_CLK_SGNL_STATE_MASK  	                    (0x1 << FLASHC_OUTPUT_CLK_SGNL_STATE_SHIFT)
#define FLASHC_DQS_WORKS_WR_OPT_SHIFT  			                3
#define FLASHC_DQS_WORKS_WR_OPT_MASK  		                    (0x1 << FLASHC_DQS_WORKS_WR_OPT_SHIFT)
#define FLASHC_SEND_DUMMY_DQS_WR_SHIFT  		                4
#define FLASHC_SEND_DUMMY_DQS_WR_MASK  		                    (0x1 << FLASHC_SEND_DUMMY_DQS_WR_SHIFT)
#define FLASHC_SEDN_DUMMY_DM_WR_SHIFT  			                5
#define FLASHC_SEDN_DUMMY_DM_WR_MASK                            (0x1 << FLASHC_SEDN_DUMMY_DM_WR_SHIFT)
#define FLASHC_CLK_ENABLE_STATE_SHIFT  			                6
#define FLASHC_CLK_ENABLE_STATE_MASK  		                    (0x1 << FLASHC_CLK_ENABLE_STATE_SHIFT)
#define FLASHC_ENABLE_CLK_STOPPED_SHIFT  		                7
#define FLASHC_ENABLE_CLK_STOPPED_MASK  		                (0x1 << FLASHC_ENABLE_CLK_STOPPED_SHIFT)
#define FLASHC_MIN_WR_CYCLE_SHIFT  				                8
#define FLASHC_MIN_WR_CYCLE_MASK  			                    (0x3 << FLASHC_MIN_WR_CYCLE_SHIFT)
#define FLASHC_DUMMY_NUM_SHIFT  				                10
#define FLASHC_DUMMY_NUM_MASK  				                    (0x3 << FLASHC_DUMMY_NUM_SHIFT)
#define FLASHC_DQS_RD_WAIT_CYCLE_SHIFT  		                12
#define FLASHC_DQS_RD_WAIT_CYCLE_MASK  		                    (0x3 << FLASHC_DQS_RD_WAIT_CYCLE_SHIFT)
#define FLASHC_MAX_CE_LOW_CYCLE_SHIFT  			                16
#define FLASHC_MAX_CE_LOW_CYCLE_MASK  	                        (0xFFF << FLASHC_MAX_CE_LOW_CYCLE_SHIFT)
#define FLASHC_MAX_PUSH_OUT_RD_LATENCY_SHIFT  	                28
#define FLASHC_MAX_PUSH_OUT_RD_LATENCY_MASK  	                (0xF << FLASHC_MAX_PUSH_OUT_RD_LATENCY_SHIFT)

/*PSRAM Latency Configure Register*/
#define FLASHC_SBUS_WR_LATENCY_SHIFT  		                    0
#define FLASHC_SBUS_WR_LATENCY_MASK  		                    (0xFF << FLASHC_SBUS_WR_LATENCY_SHIFT)
#define FLASHC_CBUS_WR_LATENCY_SHIFT  		                    8
#define FLASHC_CBUS_WR_LATENCY_MASK  		                    (0xFF << FLASHC_CBUS_WR_LATENCY_SHIFT)
#define FLASHC_RD_LATENCY_TIMEOUT_SHIFT  	                    16
#define FLASHC_RD_LATENCY_TIMEOUT_MASK  	                    (0xFF << FLASHC_RD_LATENCY_TIMEOUT_SHIFT)

/*PSRAM Timing Configure Register*/
#define FLASHC_ADQ_OUTPUT_DELAY_CYCLE_SHIFT  	                0
#define FLASHC_ADQ_OUTPUT_DELAY_CYCLE_MASK  	                (0x3 << FLASHC_ADQ_OUTPUT_DELAY_CYCLE_SHIFT)
#define FLASHC_CLK_OUTPUT_DELAY_CYCLE_SHIFT  	                4
#define FLASHC_CLK_OUTPUT_DELAY_CYCLE_MASK  	                (0x3 << FLASHC_CLK_OUTPUT_DELAY_CYCLE_SHIFT)
#define FLASHC_CS_OUTPUT_DELAY_CYCL_SHIFT  		                8
#define FLASHC_CS_OUTPUT_DELAY_CYCL_MASK  	                    (0x3 << FLASHC_CS_OUTPUT_DELAY_CYCL_SHIFT)
#define FLASHC_DM_OUTPUT_DELAY_CYCL_SHIFT  		                12
#define FLASHC_DM_OUTPUT_DELAY_CYCL_MASK  	                    (0x3 << FLASHC_DM_OUTPUT_DELAY_CYCL_SHIFT)
#define FLASHC_DQS_OUTPUT_DELAY_CYCL_SHIFT  	                16
#define FLASHC_DQS_OUTPUT_DELAY_CYCL_MASK  	                    (0x3 << FLASHC_DQS_OUTPUT_DELAY_CYCL_SHIFT)
#define FLASHC_DQS_IUTPUT_DELAY_CYCL_SHIFT  	                20
#define FLASHC_DQS_IUTPUT_DELAY_CYCL_MASK  	                    (0x3 << FLASHC_DQS_IUTPUT_DELAY_CYCL_SHIFT)

/* Address Field0 Start Postion Configuration */
#define FLASHC_ADDR_FIELD0_START_SHIFT          		        0
#define FLASHC_ADDR_FIELD0_START_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD0_START_SHIFT)

/* Address Field0 End Postion Configuration */
#define FLASHC_ADDR_FIELD0_END_SHIFT          		            0
#define FLASHC_ADDR_FIELD0_END_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD0_END_SHIFT)

/* Address Field0 Bias Configuration */
#define FLASHC_ADDR_FIELD0_BIAS_SHIFT  		                    0
#define FLASHC_ADDR_FIELD0_BIAS_MASK  		                    (0xFFFFFFFU << FLASHC_ADDR_FIELD0_BIAS_SHIFT)
#define FLASHC_ADDR_FIELD0_BIAS_EN_SHIFT  	                    31
#define FLASHC_ADDR_FIELD0_BIAS_EN_MASK  		                (0x1U << FLASHC_ADDR_FIELD0_BIAS_EN_SHIFT)

/* Address Field1 Start Postion Configuration */
#define FLASHC_ADDR_FIELD1_START_SHIFT          		        0
#define FLASHC_ADDR_FIELD1_START_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD1_START_SHIFT)

/* Address Field1 End Postion Configuration */
#define FLASHC_ADDR_FIELD1_END_SHIFT          		            0
#define FLASHC_ADDR_FIELD1_END_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD1_END_SHIFT)

/* Address Field1 Bias Configuration */
#define FLASHC_ADDR_FIELD1_BIAS_SHIFT  		                    0
#define FLASHC_ADDR_FIELD1_BIAS_MASK  		                    (0xFFFFFFFU << FLASHC_ADDR_FIELD1_BIAS_SHIFT)
#define FLASHC_ADDR_FIELD1_BIAS_EN_SHIFT  	                    31
#define FLASHC_ADDR_FIELD1_BIAS_EN_MASK  		                (0x1U << FLASHC_ADDR_FIELD1_BIAS_EN_SHIFT)

/* Address Field2 Start Postion Configuration */
#define FLASHC_ADDR_FIELD2_START_SHIFT          		        0
#define FLASHC_ADDR_FIELD2_START_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD2_START_SHIFT)

/* Address Field2 End Postion Configuration */
#define FLASHC_ADDR_FIELD2_END_SHIFT          		            0
#define FLASHC_ADDR_FIELD2_END_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD2_END_SHIFT)

/* Address Field2 Bias Configuration */
#define FLASHC_ADDR_FIELD2_BIAS_SHIFT  		                    0
#define FLASHC_ADDR_FIELD2_BIAS_MASK  		                    (0xFFFFFFFU << FLASHC_ADDR_FIELD2_BIAS_SHIFT)
#define FLASHC_ADDR_FIELD2_BIAS_EN_SHIFT  	                    31
#define FLASHC_ADDR_FIELD2_BIAS_EN_MASK  		                (0x1U << FLASHC_ADDR_FIELD2_BIAS_EN_SHIFT)

/* Address Field3 Start Postion Configuration */
#define FLASHC_ADDR_FIELD3_START_SHIFT          		        0
#define FLASHC_ADDR_FIELD3_START_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD3_START_SHIFT)

/* Address Field3 End Postion Configuration */
#define FLASHC_ADDR_FIELD3_END_SHIFT          		            0
#define FLASHC_ADDR_FIELD3_END_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD3_END_SHIFT)

/* Address Field3 Bias Configuration */
#define FLASHC_ADDR_FIELD3_BIAS_SHIFT  		                    0
#define FLASHC_ADDR_FIELD3_BIAS_MASK  		                    (0xFFFFFFFU << FLASHC_ADDR_FIELD3_BIAS_SHIFT)
#define FLASHC_ADDR_FIELD3_BIAS_EN_SHIFT  	                    31
#define FLASHC_ADDR_FIELD3_BIAS_EN_MASK  		                (0x1U << FLASHC_ADDR_FIELD3_BIAS_EN_SHIFT)

/* Address Field4 Start Postion Configuration */
#define FLASHC_ADDR_FIELD4_START_SHIFT          		        0
#define FLASHC_ADDR_FIELD4_START_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD4_START_SHIFT)

/* Address Field4 End Postion Configuration */
#define FLASHC_ADDR_FIELD4_END_SHIFT          		            0
#define FLASHC_ADDR_FIELD4_END_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD4_END_SHIFT)

/* Address Field4 Bias Configuration */
#define FLASHC_ADDR_FIELD4_BIAS_SHIFT  		                    0
#define FLASHC_ADDR_FIELD4_BIAS_MASK  		                    (0xFFFFFFFU << FLASHC_ADDR_FIELD4_BIAS_SHIFT)
#define FLASHC_ADDR_FIELD4_BIAS_EN_SHIFT  	                    31
#define FLASHC_ADDR_FIELD4_BIAS_EN_MASK  		                (0x1U << FLASHC_ADDR_FIELD4_BIAS_EN_SHIFT)

/* Address Field5 Start Postion Configuration */
#define FLASHC_ADDR_FIELD5_START_SHIFT          		        0
#define FLASHC_ADDR_FIELD5_START_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD5_START_SHIFT)

/* Address Field5 End Postion Configuration */
#define FLASHC_ADDR_FIELD5_END_SHIFT          		            0
#define FLASHC_ADDR_FIELD5_END_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD5_END_SHIFT)

/* Address Field5 Bias Configuration */
#define FLASHC_ADDR_FIELD5_BIAS_SHIFT  		                    0
#define FLASHC_ADDR_FIELD5_BIAS_MASK  		                    (0xFFFFFFFU << FLASHC_ADDR_FIELD5_BIAS_SHIFT)
#define FLASHC_ADDR_FIELD5_BIAS_EN_SHIFT  	                    31
#define FLASHC_ADDR_FIELD5_BIAS_EN_MASK  		                (0x1U << FLASHC_ADDR_FIELD5_BIAS_EN_SHIFT)

/* Address Field6 Start Postion Configuration */
#define FLASHC_ADDR_FIELD6_START_SHIFT          		        0
#define FLASHC_ADDR_FIELD6_START_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD6_START_SHIFT)

/* Address Field6 End Postion Configuration */
#define FLASHC_ADDR_FIELD6_END_SHIFT          		            0
#define FLASHC_ADDR_FIELD6_END_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD6_END_SHIFT)

/* Address Field6 Bias Configuration */
#define FLASHC_ADDR_FIELD6_BIAS_SHIFT  		                    0
#define FLASHC_ADDR_FIELD6_BIAS_MASK  		                    (0xFFFFFFFU << FLASHC_ADDR_FIELD6_BIAS_SHIFT)
#define FLASHC_ADDR_FIELD6_BIAS_EN_SHIFT  	                    31
#define FLASHC_ADDR_FIELD6_BIAS_EN_MASK  		                (0x1U << FLASHC_ADDR_FIELD6_BIAS_EN_SHIFT)

/* Address Field7 Start Postion Configuration */
#define FLASHC_ADDR_FIELD7_START_SHIFT          		        0
#define FLASHC_ADDR_FIELD7_START_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD7_START_SHIFT)

/* Address Field7 End Postion Configuration */
#define FLASHC_ADDR_FIELD7_END_SHIFT          		            0
#define FLASHC_ADDR_FIELD7_END_MASK  			                (0xFFFFFF0U << FLASHC_ADDR_FIELD7_END_SHIFT)

/* Address Field7 Bias Configuration */
#define FLASHC_ADDR_FIELD7_BIAS_SHIFT  		                    0
#define FLASHC_ADDR_FIELD7_BIAS_MASK  		                    (0xFFFFFFFU << FLASHC_ADDR_FIELD7_BIAS_SHIFT)
#define FLASHC_ADDR_FIELD7_BIAS_EN_SHIFT  	                    31
#define FLASHC_ADDR_FIELD7_BIAS_EN_MASK  		                (0x1U << FLASHC_ADDR_FIELD7_BIAS_EN_SHIFT)

/* System-bus Write Data Register */
#define FLASHC_SBUS_WR_DATA_SHIFT                               0
#define FLASHC_SBUS_WR_DATA_MASK                                (0xFFFFFFFFU << FLASHC_SBUS_WR_DATA_SHIFT)

/* System-bus Read Data Register */
#define FLASHC_SBUS_RD_DATA_SHIFT                               0
#define FLASHC_SBUS_RD_DATA_MASK                                (0xFFFFFFFFU << FLASHC_SBUS_RD_DATA_SHIFT)

typedef enum {
	FC_TCTRL_FBS_MSB = 0 << FLASHC_FIRST_RCV_BIT_SLT_SHIFT,
	FC_TCTRL_FBS_LSB = 1 << FLASHC_FIRST_RCV_BIT_SLT_SHIFT
} FC_TCTRL_Fbs;

typedef enum {
	FC_SCLK_Mode0 = 0 << FLASHC_SPI_CPHA_CTRL_SHIFT,
	FC_SCLK_Mode1 = 1 << FLASHC_SPI_CPHA_CTRL_SHIFT,
	FC_SCLK_Mode2 = 2 << FLASHC_SPI_CPHA_CTRL_SHIFT,
	FC_SCLK_Mode3 = 3 << FLASHC_SPI_CPHA_CTRL_SHIFT
} FC_Sclk_Mode;

typedef enum {
	FC_TCTRL_CS_LOW_ENABLE = 0 << FLASHC_FLASH_CS_POL_SHIFT,
	FC_TCTRL_CS_HIGH_ENABLE = 1 << FLASHC_FLASH_CS_POL_SHIFT
} FC_Cs;

#endif /* FLASHCTRL_ARCH_V2 */

struct flash_controller {
	uint8_t externAddr_on;
	uint8_t xip_on;
	uint8_t ccmu_on;
	uint8_t pin_inited;
	uint8_t sbusing;
	OS_Semaphore_t dmaSem;
	uint8_t ref;
#if FLASHCTRL_ARCH_V2
	uint8_t xip_continue;
#else
	FC_En xip_continue;
#endif
#ifdef CONFIG_PM
	uint8_t pm_xip;
	uint8_t suspending;
	XIP_Config pm_ibus_cfg;
	Flashc_Config pm_sbus_cfg;
	struct soc_device_driver flashc_drv;
	struct soc_device flashc_dev;
#endif
};

HAL_Status HAL_Flashc_Xip_Init(struct flash_controller *ctrl, XIP_Config *cfg);
HAL_Status HAL_Flashc_Xip_Deinit(struct flash_controller *ctrl);
void HAL_Flashc_Xip_RawEnable(struct flash_controller *ctrl);
void HAL_Flashc_Xip_RawDisable(struct flash_controller *ctrl);
void HAL_Flashc_Xip_Enable(struct flash_controller *ctrl);
void HAL_Flashc_Xip_Disable(struct flash_controller *ctrl);

HAL_Status HAL_Flashc_Init(struct flash_controller *ctrl, const Flashc_Config *cfg);
HAL_Status HAL_Flashc_Deinit(struct flash_controller *ctrl);
HAL_Status HAL_Flashc_Open(struct flash_controller *ctrl);
HAL_Status HAL_Flashc_Close(struct flash_controller *ctrl);
HAL_Status HAL_Flashc_Ioctl(struct flash_controller *ctrl, uint32_t op, void *arg);
HAL_Status HAL_Flashc_Transfer(struct flash_controller *ctrl, int write,
                               FC_InstructionField *cmd, FC_InstructionField *addr,
                               FC_InstructionField *dummy, FC_InstructionField *data, bool dma);
struct flash_controller *HAL_Flashc_Create(uint32_t id);
HAL_Status HAL_Flashc_Destory(struct flash_controller *ctrl);
int HAL_Flashc_IncRef(struct flash_controller *ctrl);
int HAL_Flashc_DecRef(struct flash_controller *ctrl);
void HAL_Flashc_SetDbgMask(uint8_t dbg_mask);

void HAL_Flashc_Delay(struct flash_controller *ctrl, unsigned int us);
uint8_t FC_GetDataDelay(uint32_t freq);

void HAL_Flashc_ResetCCMU(void);
bool HAL_Flashc_ConfigCCMU(uint32_t clk);
void HAL_Flashc_EnableCCMU(struct flash_controller *ctrl);
void FC_Sbus_TransmitDelay(Flash_Ctrl_DelayCycle *delay);
void FC_SetFlash(FC_Cs cs, FC_TCTRL_Fbs fbs, FC_Sclk_Mode mode);
void FC_Sbus_ResetFIFO(bool tx, bool rx);
void HAL_Flashc_DisableCCMU(struct flash_controller *ctrl);

void HAL_Flashc_PinInit(struct flash_controller *ctrl);
void FC_Ibus_TransmitDelay(Flash_Ctrl_DelayCycle *delay);
void HAL_Flashc_PinDeinit(struct flash_controller *ctrl);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_FLASHCTRL_H_ */
