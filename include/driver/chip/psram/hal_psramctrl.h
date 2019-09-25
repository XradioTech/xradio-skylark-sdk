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

#ifndef _DRIVER_CHIP_HAL_PSRAMCTRL_H_
#define _DRIVER_CHIP_HAL_PSRAMCTRL_H_

#include <stdbool.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_def.h"
#include "driver/chip/psram/psram.h"
#include "pm/pm.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (__CONFIG_CHIP_ARCH_VER == 2)

#define PSRAMC_PAGE_SZ_512              512
#define PSRAMC_PAGE_SZ_1K               1024

#define PSRAM_CTRL_USE_IRQ              1

    /*
     * bit field definition of PSRAMC->COMMON_CFG 0x00
     */
#define PSRAMC_ADDR_SIZE_SHIFT          (30)
#define PSRAMC_ADDR_SIZE_MASK           (3U << PSRAMC_ADDR_SIZE_SHIFT)
    typedef enum {
        PSRAMC_ADDR_SIZE_24BIT = (2U << PSRAMC_ADDR_SIZE_SHIFT),
        PSRAMC_ADDR_SIZE_32BIT = (3U << PSRAMC_ADDR_SIZE_SHIFT),
    }
    PSRAMC_AddrSize;

#define PSRAMC_IOX_DEF_OUTPUT_SHIFT(x)  (30 - 2 * (x))
#define PSRAMC_IOX_DEF_OUTPUT_VMASK     (3U)
#define PSRAMC_IOX_DEF_OUTPUT_MASK(x)   (PSRAMC_IOX_DEF_OUTPUT_VMASK << PSRAMC_IOX_DEF_OUTPUT_SHIFT(x))
    typedef enum {
        PSRAMC_IOX_DEF_OUTPUT0 = 0U,
        PSRAMC_IOX_DEF_OUTPUT1 = 1U,
        PSRAMC_IOX_DEF_OUTPUTZ = 2U
    } PSRAMC_IoxDefOutPutMode;
#define PSRAMC_IOX_DEF_OUTPUT_MODE(x, m) (((m) & PSRAMC_IOX_DEF_OUTPUT_VMASK) << PSRAMC_IOX_DEF_OUTPUT_SHIFT(x))

#define PSRAMC_SBUS_HRDY_WT_TO_SHIFT    (8)
#define PSRAMC_SBUS_HRDY_WT_TO_VMASK    (0x0FF)
#define PSRAMC_SBUS_HRDY_WT_TO_MASK     (PSRAMC_SBUS_HRDY_WT_TO_VMASK << PSRAMC_SBUS_HRDY_WT_TO_SHIFT)
#define PSRAMC_SBUS_HRDY_WT_TO_VAL(v)   (((v) & PSRAMC_SBUS_HRDY_WT_TO_VMASK) << PSRAMC_SBUS_HRDY_WT_TO_SHIFT)

#define PSRAMC_SBUS_HRDY_TO_EN          HAL_BIT(7)
#define PSRAMC_TRAN_FIFO_RST            HAL_BIT(6)
#define PSRAMC_REV_FIFO_RST             HAL_BIT(5)
#define PSRAM_SELECT                    HAL_BIT(4)
#define PSRAMC_WRAP_AROUND_EN           HAL_BIT(3)
#define PSRAMC_CBUS_RW_EN               HAL_BIT(0)

    /*
     * bit field definition of PSRAMC->CTRL_COM_CFG 0x04
     */
#define PSRAM_DMA_CROSS_OP_DIS          HAL_BIT(16)
#define PSRAM_SPI_RDWAIT_SHIFT          (12)
#define PSRAM_SPI_RDWAIT_VMASK          (3U)
#define PSRAM_SPI_RDWAIT_Cycle(n)       (((n) & PSRAM_SPI_RDWAIT_VMASK) << PSRAM_SPI_RDWAIT_SHIFT)
#define PSRAM_SPI_CS_SHIFT              (8)
#define PSRAM_SPI_CS_L                  (0U << PSRAM_SPI_CS_SHIFT)
#define PSRAM_SPI_CS_H                  (1U << PSRAM_SPI_CS_SHIFT)
#define PSRAM_SPI_FIRST_BIT_SHIFT       (4)
#define PSRAM_SPI_MSB                   (0U << PSRAM_SPI_FIRST_BIT_SHIFT)
#define PSRAM_SPI_LSB                   (1U << PSRAM_SPI_FIRST_BIT_SHIFT)
#define PSRAM_SPI_MODE_SHIFT            (0)
#define PSRAM_SPI_MODE_MASK             (3U << PSRAM_SPI_MODE_SHIFT)
    typedef enum {
        PSRAM_SPI_MODE0 = (0U << PSRAM_SPI_MODE_SHIFT),
        PSRAM_SPI_MODE1 = (1U << PSRAM_SPI_MODE_SHIFT),
        PSRAM_SPI_MODE2 = (2U << PSRAM_SPI_MODE_SHIFT),
        PSRAM_SPI_MODE3 = (3U << PSRAM_SPI_MODE_SHIFT),
    } PSRAM_SpiMode;

    /*
     * bit field definition of PSRAMC->CACHE_RLVT_CFG 0x08
     */
#define PSRAMC_AUTO_FLUSH_TO_SHIFT      (31)
#define PSRAMC_AUTO_FLUSH_TO            (1U << PSRAMC_AUTO_FLUSH_TO_SHIFT)
#define PSRAMC_CBUS_WT_CACHE_LL_TO_SHIFT   (17)
#define PSRAMC_CBUS_WT_CACHE_LL_TO_VMASK   (0x07FFF)
#define PSRAMC_CBUS_WT_CACHE_LL_TO(v)   (((v) & PSRAMC_CBUS_WT_CACHE_LL_TO_VMASK) << PSRAMC_CBUS_WT_CACHE_LL_TO_SHIFT)
#define PSRAMC_CBUS_WT_CACHE_LL_TO_EN   HAL_BIT(16)
#define PSRAMC_AUTO_CROSS_PAGE_EN       HAL_BIT(15)

#define PSRAMC_WR_CACHE_LINE_LEN_SHIFT (12)
#define PSRAMC_WR_CACHE_LINE_LEN_VMASK (0x7)
#define PSRAMC_WR_CACHE_LINE_LEN_MASK  (PSRAMC_WR_CACHE_LINE_LEN_VMASK << PSRAMC_WR_CACHE_LINE_LEN_SHIFT)
#define PSRAMC_WR_CACHE_LINE_LEN(n)    (((n) & PSRAMC_WR_CACHE_LINE_LEN_VMASK) << PSRAMC_WR_CACHE_LINE_LEN_SHIFT)
    typedef enum {
        PSRAMC_WR_CACHE_LINE_LEN_8B   = (0x0U << PSRAMC_WR_CACHE_LINE_LEN_SHIFT),
        PSRAMC_WR_CACHE_LINE_LEN_16B  = (0x1U << PSRAMC_WR_CACHE_LINE_LEN_SHIFT),
        PSRAMC_WR_CACHE_LINE_LEN_32B  = (0x2U << PSRAMC_WR_CACHE_LINE_LEN_SHIFT),
        PSRAMC_WR_CACHE_LINE_LEN_48B  = (0x3U << PSRAMC_WR_CACHE_LINE_LEN_SHIFT),
        PSRAMC_WR_CACHE_LINE_LEN_64B  = (0x4U << PSRAMC_WR_CACHE_LINE_LEN_SHIFT),
        PSRAMC_WR_CACHE_LINE_LEN_80B  = (0x5U << PSRAMC_WR_CACHE_LINE_LEN_SHIFT),
        PSRAMC_WR_CACHE_LINE_LEN_96B  = (0x6U << PSRAMC_WR_CACHE_LINE_LEN_SHIFT),
        PSRAMC_WR_CACHE_LINE_LEN_112B = (0x7U << PSRAMC_WR_CACHE_LINE_LEN_SHIFT)
    } PSRAMC_WrCacheLL;

#define PSRAMC_WAIT_CACHE_LINE_TOUT_CYC_SHIFT (6)
#define PSRAMC_WAIT_CACHE_LINE_TOUT_CYC_VMASK (0x7FFFU)
#define PSRAMC_WAIT_CACHE_LINE_TOUT_CYC_MASK (PSRAMC_WAIT_CACHE_LINE_TOUT_CYC_VMASK << PSRAMC_WAIT_CACHE_LINE_TOUT_CYC_SHIFT)
#define PSRAMC_CBUS_HRDY_TIMOUT_SHIFT   (4)
#define PSRAMC_CBUS_HRDY_TIMOUT_VMASK   (0x0FF)
#define PSRAMC_CBUS_HRDY_TIMOUT(v)      (((v) & PSRAMC_CBUS_HRDY_TIMOUT_VMASK) << PSRAMC_CBUS_HRDY_TIMOUT_SHIFT)
#define PSRAMC_CBUS_HRDY_TIMOUT_EN      HAL_BIT(3)
#define PSRAMC_CBUS_WR_SEL_BUS          HAL_BIT(2)
#define PSRAMC_CACHE_LL_CFG_SHIFT    (0)
#define PSRAMC_CACHE_LL_CFG_VMASK    (0x3)
#define PSRAMC_CACHE_LL_CFG_MASK     (PSRAMC_CACHE_LL_CFG_VMASK << PSRAMC_CACHE_LL_CFG_SHIFT)
    typedef enum {
        PSRAMC_CACHE_LL_64BIT  = (0x0U << PSRAMC_CACHE_LL_CFG_SHIFT),
        PSRAMC_CACHE_LL_128BIT = (0x1U << PSRAMC_CACHE_LL_CFG_SHIFT),
        PSRAMC_CACHE_LL_256BIT = (0x2U << PSRAMC_CACHE_LL_CFG_SHIFT)
    } PSRAMC_CacheLLCfg;

    /*
     * bit field definition of PSRAMC->PSRAMC_CBUS_RD_OP_CFG 0x10
     */
#define PSRAMC_CBUS_RD_CMD_SHIFT        (24)
#define PSRAMC_CBUS_RD_CMD_VMASK        (0x0FF)
#define PSRAMC_CBUS_RD_CMD_MASK(cmd)    (((cmd) & PSRAMC_CBUS_RD_CMD_VMASK) << PSRAMC_CBUS_RD_CMD_SHIFT)
#define PSRAMC_IDBUS_DMA_EN		HAL_BIT(23)

#define PSRAMC_CBUS_CMD_BIT_SHIFT       (20)
#define PSRAMC_CBUS_CMD_BIT_VMASK       (0x7)
    typedef enum {
        PSRAMC_CBUS_CMD_0BIT = (0x0U << PSRAMC_CBUS_CMD_BIT_SHIFT),
        PSRAMC_CBUS_CMD_1BIT = (0x1U << PSRAMC_CBUS_CMD_BIT_SHIFT),
        PSRAMC_CBUS_CMD_2BIT = (0x2U << PSRAMC_CBUS_CMD_BIT_SHIFT),
        PSRAMC_CBUS_CMD_4BIT = (0x3U << PSRAMC_CBUS_CMD_BIT_SHIFT),
        PSRAMC_CBUS_CMD_8BIT = (0x4U << PSRAMC_CBUS_CMD_BIT_SHIFT)
    } PSRAMC_CbusCmdBit;

#define PSRAMC_CBUS_ADDR_BIT_SHIFT      (16)
#define PSRAMC_CBUS_ADDR_BIT_VMASK      (0x7)
    typedef enum {
        PSRAMC_CBUS_ADDR_0BIT = (0x0U << PSRAMC_CBUS_ADDR_BIT_SHIFT),
        PSRAMC_CBUS_ADDR_1BIT = (0x1U << PSRAMC_CBUS_ADDR_BIT_SHIFT),
        PSRAMC_CBUS_ADDR_2BIT = (0x2U << PSRAMC_CBUS_ADDR_BIT_SHIFT),
        PSRAMC_CBUS_ADDR_4BIT = (0x3U << PSRAMC_CBUS_ADDR_BIT_SHIFT),
        PSRAMC_CBUS_ADDR_8BIT = (0x4U << PSRAMC_CBUS_ADDR_BIT_SHIFT)
    } PSRAMC_CbusAddrBit;

#define PSRAMC_CBUS_DUMY_BIT_SHIFT      (12)
#define PSRAMC_CBUS_DUMY_BIT_VMASK      (0x7)
    typedef enum {
        PSRAMC_CBUS_DUMY_0BIT = (0x0U << PSRAMC_CBUS_DUMY_BIT_SHIFT),
        PSRAMC_CBUS_DUMY_1BIT = (0x1U << PSRAMC_CBUS_DUMY_BIT_SHIFT),
        PSRAMC_CBUS_DUMY_2BIT = (0x2U << PSRAMC_CBUS_DUMY_BIT_SHIFT),
        PSRAMC_CBUS_DUMY_4BIT = (0x3U << PSRAMC_CBUS_DUMY_BIT_SHIFT),
        PSRAMC_CBUS_DUMY_8BIT = (0x4U << PSRAMC_CBUS_DUMY_BIT_SHIFT)
    } PSRAMC_CbusDumyBit;

#define PSRAMC_CBUS_DUMY_WID_SHIFT      (4)
#define PSRAMC_CBUS_DUMY_WID_VMASK      (0x7F)
#define PSRAMC_CBUS_DUMY_WID(n)         (((n) & PSRAMC_CBUS_DUMY_WID_VMASK) << PSRAMC_CBUS_DUMY_WID_SHIFT)

#define PSRAMC_CBUS_DATA_BIT_SHIFT      (0)
#define PSRAMC_CBUS_DATA_BIT_VMASK      (0x7)
    typedef enum {
        PSRAMC_CBUS_DATA_0BIT = (0x0U << PSRAMC_CBUS_DATA_BIT_SHIFT),
        PSRAMC_CBUS_DATA_1BIT = (0x1U << PSRAMC_CBUS_DATA_BIT_SHIFT),
        PSRAMC_CBUS_DATA_2BIT = (0x2U << PSRAMC_CBUS_DATA_BIT_SHIFT),
        PSRAMC_CBUS_DATA_4BIT = (0x3U << PSRAMC_CBUS_DATA_BIT_SHIFT),
        PSRAMC_CBUS_DATA_8BIT = (0x4U << PSRAMC_CBUS_DATA_BIT_SHIFT)
    } PSRAMC_CbusDataBit;

    /*
     * bit field definition of PSRAMC->PSRAMC_CBUS_WR_OP_CFG 0x14
     */
#define PSRAMC_CBUS_WR_CMD_SHIFT        (24)
#define PSRAMC_CBUS_WR_CMD_VMASK        (0x0FF)
#define PSRAMC_CBUS_WR_CMD(wcmd)        (((wcmd) & PSRAMC_CBUS_WR_CMD_VMASK) << PSRAMC_CBUS_WR_CMD_SHIFT)

#define PSRAMC_CBUS_WR_OP_CMD_BIT_SHIFT (20)
#define PSRAMC_CBUS_WR_OP_CMD_BIT_VMASK (0x7)
    typedef enum {
        PSRAMC_CBUS_WR_OP_CMD_0BIT = (0x0U << PSRAMC_CBUS_WR_OP_CMD_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_CMD_1BIT = (0x1U << PSRAMC_CBUS_WR_OP_CMD_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_CMD_2BIT = (0x2U << PSRAMC_CBUS_WR_OP_CMD_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_CMD_4BIT = (0x3U << PSRAMC_CBUS_WR_OP_CMD_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_CMD_8BIT = (0x4U << PSRAMC_CBUS_WR_OP_CMD_BIT_SHIFT)
    } PSRAMC_CbusWrOpCmdBit;

#define PSRAMC_CBUS_WR_OP_ADD_BIT_SHIFT (16)
#define PSRAMC_CBUS_WR_OP_ADD_BIT_VMASK (0x7)
    typedef enum {
        PSRAMC_CBUS_WR_OP_ADDR_0BIT = (0x0U << PSRAMC_CBUS_WR_OP_ADD_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_ADDR_1BIT = (0x1U << PSRAMC_CBUS_WR_OP_ADD_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_ADDR_2BIT = (0x2U << PSRAMC_CBUS_WR_OP_ADD_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_ADDR_4BIT = (0x3U << PSRAMC_CBUS_WR_OP_ADD_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_ADDR_8BIT = (0x4U << PSRAMC_CBUS_WR_OP_ADD_BIT_SHIFT)
    } PSRAMC_CbusWrOpAddrBit;

#define PSRAMC_CBUS_WR_OP_DUMY_BIT_SHIFT (12)
#define PSRAMC_CBUS_WR_OP_DUMY_BIT_VMASK (0x7)
    typedef enum {
        PSRAMC_CBUS_WR_OP_DUMY_0BIT = (0x0U << PSRAMC_CBUS_WR_OP_DUMY_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_DUMY_1BIT = (0x1U << PSRAMC_CBUS_WR_OP_DUMY_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_DUMY_2BIT = (0x2U << PSRAMC_CBUS_WR_OP_DUMY_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_DUMY_4BIT = (0x3U << PSRAMC_CBUS_WR_OP_DUMY_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_DUMY_8BIT = (0x4U << PSRAMC_CBUS_WR_OP_DUMY_BIT_SHIFT)
    } PSRAMC_CbusWrOpDumyBit;

#define PSRAMC_CBUS_WR_OP_DUMY_NUM_SHIFT (4)
#define PSRAMC_CBUS_WR_OP_DUMY_NUM_VMASK (0x7F)
#define PSRAMC_CBUS_WR_OP_DUMY_NUM(n)   (((n) & PSRAMC_CBUS_WR_OP_DUMY_NUM_VMASK) << PSRAMC_CBUS_WR_OP_DUMY_NUM_SHIFT)

#define PSRAMC_CBUS_WR_OP_DATA_BIT_SHIFT (0)
#define PSRAMC_CBUS_WR_OP_DATA_BIT_VMASK (0x7)
    typedef enum {
        PSRAMC_CBUS_WR_OP_DATA_0BIT = (0x0U << PSRAMC_CBUS_WR_OP_DATA_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_DATA_1BIT = (0x1U << PSRAMC_CBUS_WR_OP_DATA_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_DATA_2BIT = (0x2U << PSRAMC_CBUS_WR_OP_DATA_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_DATA_4BIT = (0x3U << PSRAMC_CBUS_WR_OP_DATA_BIT_SHIFT),
        PSRAMC_CBUS_WR_OP_DATA_8BIT = (0x4U << PSRAMC_CBUS_WR_OP_DATA_BIT_SHIFT)
    } PSRAMC_CbusWrOpDataBit;

    /*
     * Bit field definition of SBUS_RW_OPRT_CONFG
     */
#define PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT  24

#define PSRAMC_S_RW_CFG_MCLK_SRC_SHIFT  (0)
#define PSRAMC_S_RW_CFG_MCLK_SRC_MASK   (0x3U << PSRAMC_S_RW_CFG_MCLK_SRC_SHIFT)
    typedef enum {
        PSRAMC_S_RW_CFG_MCLK_SRC_8X = (0x0U << PSRAMC_S_RW_CFG_MCLK_SRC_SHIFT),
        PSRAMC_S_RW_CFG_MCLK_SRC_4X = (0x1U << PSRAMC_S_RW_CFG_MCLK_SRC_SHIFT),
        PSRAMC_S_RW_CFG_MCLK_SRC_2X = (0x2U << PSRAMC_S_RW_CFG_MCLK_SRC_SHIFT),
        PSRAMC_S_RW_CFG_MCLK_SRC_1X = (0x3U << PSRAMC_S_RW_CFG_MCLK_SRC_SHIFT)
    } PSRAMC_SbusRwCfgMclkSrc;

    /*
     * bit field definition of PSRAMC->PSRAMC_CBUS_IO_SW_WAIT 0x28
     */
#define PSRAMC_CBUS_RW_LAT_WAIT_SHIFT   (24)
#define PSRAMC_CBUS_RW_LAT_WAIT_VMASK   (0x0FF)
#define PSRAMC_CBUS_RW_LAT_WAIT(n)      (((n) & PSRAMC_CBUS_RW_LAT_WAIT_VMASK) << PSRAMC_CBUS_RW_LAT_WAIT_SHIFT)
#define PSRAMC_CBUS_CMD_WAIT_SHIFT      (16)
#define PSRAMC_CBUS_CMD_WAIT_VMASK      (0x0FF)
#define PSRAMC_CBUS_CMD_WAIT(n)         (((n) & PSRAMC_CBUS_CMD_WAIT_VMASK) << PSRAMC_CBUS_CMD_WAIT_SHIFT)
#define PSRAMC_CBUS_ADR_WAIT_SHIFT      (8)
#define PSRAMC_CBUS_ADR_WAIT_VMASK      (0x0FF)
#define PSRAMC_CBUS_ADR_WAIT(n)         (((n) & PSRAMC_CBUS_ADR_WAIT_VMASK) << PSRAMC_CBUS_ADR_WAIT_SHIFT)
#define PSRAMC_CBUS_DUM_WAIT_SHIFT      (0)
#define PSRAMC_CBUS_DUM_WAIT_VMASK      (0x0FF)
#define PSRAMC_CBUS_DUM_WAIT(n)         (((n) & PSRAMC_CBUS_DUM_WAIT_VMASK) << PSRAMC_CBUS_DUM_WAIT_SHIFT)

    /*
     * bit field definition of PSRAMC->PSRAMC_SBUS_CMD_CFG 0x2c
     */
#define PSRAMC_SBUS_RW_CMD_SHIFT        (24)
#define PSRAMC_SBUS_RW_CMD_SEND_MASK    (0x3 << PSRAMC_SBUS_RW_CMD_SHIFT)
#define PSRAMC_SBUS_CMD_SEND_BIT_SHIFT  (20)
#define PSRAMC_SBUS_CMD_SEND_BIT_VMASK  (0x7)
#define PSRAMC_SBUS_CMD_SEND_BIT_MASK   (PSRAMC_SBUS_CMD_SEND_BIT_VMASK << PSRAMC_SBUS_CMD_SEND_BIT_SHIFT)
    typedef enum {
        PSRAMC_SBUS_CMD_SEND_0BIT = (0x0U << PSRAMC_SBUS_CMD_SEND_BIT_SHIFT),
        PSRAMC_SBUS_CMD_SEND_1BIT = (0x1U << PSRAMC_SBUS_CMD_SEND_BIT_SHIFT),
        PSRAMC_SBUS_CMD_SEND_2BIT = (0x2U << PSRAMC_SBUS_CMD_SEND_BIT_SHIFT),
        PSRAMC_SBUS_CMD_SEND_4BIT = (0x3U << PSRAMC_SBUS_CMD_SEND_BIT_SHIFT),
        PSRAMC_SBUS_CMD_SEND_8BIT = (0x4U << PSRAMC_SBUS_CMD_SEND_BIT_SHIFT)
    } PSRAMC_SbusCmdSendBit;

#define PSRAMC_SBUS_ADDR_SEND_BIT_SHIFT (16)
#define PSRAMC_SBUS_ADDR_SEND_BIT_VMASK (0x7)
#define PSRAMC_SBUS_ADDR_SEND_BIT_MASK  (PSRAMC_SBUS_ADDR_SEND_BIT_VMASK << PSRAMC_SBUS_ADDR_SEND_BIT_SHIFT)
    typedef enum {
        PSRAMC_SBUS_ADDR_SEND_0BIT = (0x0U << PSRAMC_SBUS_ADDR_SEND_BIT_SHIFT),
        PSRAMC_SBUS_ADDR_SEND_1BIT = (0x1U << PSRAMC_SBUS_ADDR_SEND_BIT_SHIFT),
        PSRAMC_SBUS_ADDR_SEND_2BIT = (0x2U << PSRAMC_SBUS_ADDR_SEND_BIT_SHIFT),
        PSRAMC_SBUS_ADDR_SEND_4BIT = (0x3U << PSRAMC_SBUS_ADDR_SEND_BIT_SHIFT),
        PSRAMC_SBUS_ADDR_SEND_8BIT = (0x4U << PSRAMC_SBUS_ADDR_SEND_BIT_SHIFT)
    } PSRAMC_SbusAddrSendBit;

#define PSRAMC_SBUS_DUMY_SEND_BIT_SHIFT  (12)
#define PSRAMC_SBUS_DUMY_SEND_BIT_VMASK  (0x7)
#define PSRAMC_SBUS_DUMY_SEND_BIT_MASK   (PSRAMC_SBUS_DUMY_SEND_BIT_VMASK << PSRAMC_SBUS_DUMY_SEND_BIT_SHIFT)
    typedef enum {
        PSRAMC_SBUS_DUMY_SEND_0BIT = (0x0U << PSRAMC_SBUS_DUMY_SEND_BIT_SHIFT),
        PSRAMC_SBUS_DUMY_SEND_1BIT = (0x1U << PSRAMC_SBUS_DUMY_SEND_BIT_SHIFT),
        PSRAMC_SBUS_DUMY_SEND_2BIT = (0x2U << PSRAMC_SBUS_DUMY_SEND_BIT_SHIFT),
        PSRAMC_SBUS_DUMY_SEND_4BIT = (0x3U << PSRAMC_SBUS_DUMY_SEND_BIT_SHIFT),
        PSRAMC_SBUS_DUMY_SEND_8BIT = (0x4U << PSRAMC_SBUS_DUMY_SEND_BIT_SHIFT)
    } PSRAMC_SbusDumySendBit;

    /* DUMMY Data Bit Number, n/8 Byte */
#define PSRAMC_SBUS_DMY_DATA_WID_SHIFT  (4)
#define PSRAMC_SBUS_DMY_DATA_WID_VMASK  (0x7)
#define PSRAMC_SBUS_DMY_DATA_WID_MASK   (PSRAMC_SBUS_DMY_DATA_WID_VMASK << PSRAMC_SBUS_DMY_DATA_WID_SHIFT)
#define PSRAMC_SBUS_DMY_DATA_WID(n) 	(((n) & PSRAMC_SBUS_DMY_DATA_WID_VMASK) << PSRAMC_SBUS_DMY_DATA_WID_SHIFT)

#define PSRAMC_SBUS_DATA_GW_BIT_SHIFT   (0)
#define PSRAMC_SBUS_DATA_GW_BIT_VMASK   (0x7)
#define PSRAMC_SBUS_DATA_GW_BIT_MASK    (PSRAMC_SBUS_DATA_GW_BIT_VMASK << PSRAMC_SBUS_DATA_GW_BIT_SHIFT)
    typedef enum {
        PSRAMC_SBUS_DATA_GW_0BIT = (0x0U << PSRAMC_SBUS_DATA_GW_BIT_SHIFT),
        PSRAMC_SBUS_DATA_GW_1BIT = (0x1U << PSRAMC_SBUS_DATA_GW_BIT_SHIFT),
        PSRAMC_SBUS_DATA_GW_2BIT = (0x2U << PSRAMC_SBUS_DATA_GW_BIT_SHIFT),
        PSRAMC_SBUS_DATA_GW_4BIT = (0x3U << PSRAMC_SBUS_DATA_GW_BIT_SHIFT),
        PSRAMC_SBUS_DATA_GW_8BIT = (0x4U << PSRAMC_SBUS_DATA_GW_BIT_SHIFT)
    } PSRAMC_SbusDataGwBit;

    /*
     * bit field definition of PSRAMC->PSRAMC_S_IO_SWIT_TIME 0x3C
     */
#define PSRAMC_SBUS_RD_LT_WAIT_SHIFT    (24)
#define PSRAMC_SBUS_RD_LT_WAIT_VMASK    (0x0FF)
#define PSRAMC_SBUS_RD_LT_WAIT(n)       (((n) & PSRAMC_SBUS_RD_LT_WAIT_VMASK) << PSRAMC_SBUS_RD_LT_WAIT_SHIFT)
#define PSRAMC_SBUS_CMD_WAIT_SHIFT      (16)
#define PSRAMC_SBUS_CMD_WAIT_VMASK      (0x0FF)
#define PSRAMC_SBUS_CMD_WAIT(n)         (((n) & PSRAMC_SBUS_CMD_WAIT_VMASK) << PSRAMC_SBUS_CMD_WAIT_SHIFT)
#define PSRAMC_SBUS_ADR_WAIT_SHIFT      (8)
#define PSRAMC_SBUS_ADR_WAIT_VMASK      (0x0FF)
#define PSRAMC_SBUS_ADR_WAIT(n)         (((n) & PSRAMC_SBUS_ADR_WAIT_VMASK) << PSRAMC_SBUS_ADR_WAIT_SHIFT)
#define PSRAMC_SBUS_DUM_WAIT_SHIFT      (0)
#define PSRAMC_SBUS_DUM_WAIT_VMASK      (0x0FF)
#define PSRAMC_SBUS_DUM_WAIT(n)         (((n) & PSRAMC_SBUS_DUM_WAIT_VMASK) << PSRAMC_SBUS_DUM_WAIT_SHIFT)

    /*
     * bit field definition of PSRAMC->PSRAMC_FIFO_TRIG_LEVEL 0x4C
     */
#define PSRAMC_FIFO_WR_FULL_SHIFT       (24)
#define PSRAMC_FIFO_WR_FULL_VMASK       (0x0FF)
#define PSRAMC_FIFO_WR_FULL_TRIG(v)     (((v) & PSRAMC_FIFO_WR_FULL_VMASK) << PSRAMC_FIFO_WR_FULL_SHIFT)
#define PSRAMC_FIFO_WR_EMPT_SHIFT       (16)
#define PSRAMC_FIFO_WR_EMPT_VMASK       (0x0FF)
#define PSRAMC_FIFO_WR_EMPT_TRIG(v)     (((v) & PSRAMC_FIFO_WR_EMPT_VMASK)<< PSRAMC_FIFO_WR_EMPT_SHIFT)
#define PSRAMC_FIFO_RD_FULL_SHIFT       (8)
#define PSRAMC_FIFO_RD_FULL_VMASK       (0x0FF)
#define PSRAMC_FIFO_RD_FULL_TRIG(v)     (((v) & PSRAMC_FIFO_RD_FULL_VMASK) << PSRAMC_FIFO_RD_FULL_SHIFT)
#define PSRAMC_FIFO_RD_EMPT_SHIFT       (0)
#define PSRAMC_FIFO_RD_EMPT_VMASK       (0x0FF)
#define PSRAMC_FIFO_RD_EMPT_TRIG(v)     (((v) & PSRAMC_FIFO_RD_EMPT_VMASK) << PSRAMC_FIFO_RD_EMPT_SHIFT)

    /*
     * bit field definition of PSRAMC->PSRAMC_FIFO_STATUS_REG 0x50
     */
#define PSRAMC_FIFO_STA_RD_BUF_EMP          HAL_BIT(25)
#define PSRAMC_FIFO_STA_WR_BUF_FULL         HAL_BIT(24)
#define PSRAMC_FIFO_STA_WR_BUF_CAN          HAL_BIT(23)
#define PSRAMC_FIFO_STA_WR_BUF_CNT_SHIFT    (20)
#define PSRAMC_FIFO_STA_WR_BUF_CNT_MASK     (7U << PSRAMC_FIFO_STA_WR_BUF_CNT_SHIFT)
#define PSRAMC_FIFO_STA_WR_BUF_CNT(reg)     ((reg) & PSRAMC_FIFO_STA_WR_BUF_CNT_MASK)
#define PSRAMC_FIFO_STA_RD_BUF_CAN          HAL_BIT(19)
#define PSRAMC_FIFO_STA_RD_BUF_CNT_SHIFT    (16)
#define PSRAMC_FIFO_STA_RD_BUF_CNT_MASK     (7U << PSRAMC_FIFO_STA_RD_BUF_CNT_SHIFT)
#define PSRAMC_FIFO_STA_RD_BUF_CNT(reg)     ((reg) & PSRAMC_FIFO_STA_RD_BUF_CNT_MASK)
#define PSRAMC_FIFO_STA_WR_FIFO_CNT_SHIFT   (8)
#define PSRAMC_FIFO_STA_WR_FIFO_CNT_MASK    (0x0FFU << PSRAMC_FIFO_STA_WR_FIFO_CNT_SHIFT)
#define PSRAMC_FIFO_STA_WR_FIFO_CNT(reg)    ((reg) & PSRAMC_FIFO_STA_WR_FIFO_CNT_MASK)
#define PSRAMC_FIFO_STA_RD_FIFO_CNT_SHIFT   (0)
#define PSRAMC_FIFO_STA_RD_FIFO_CNT_MASK    (0x0FFU << PSRAMC_FIFO_STA_RD_FIFO_CNT_SHIFT)
#define PSRAMC_FIFO_STA_RD_FIFO_CNT(reg)    ((reg) & PSRAMC_FIFO_STA_RD_FIFO_CNT_MASK)

    /*
     * bit field definition of PSRAMC->PSRAMC_INT_EN_REG 0x54
     */
#define PSRAMC_DMA_WR_CROSS_INT_EN      HAL_BIT(15)
#define PSRAMC_RD_TOUT_INT_EN           HAL_BIT(14)
#define PSRAMC_ISR_HEDY_TI_EN           HAL_BIT(13)
#define PSRAMC_IER_TRANS_ENB            HAL_BIT(12)
#define PSRAMC_IER_TRANS_ENB_MASK       HAL_BIT(12)
#define PSRAMC_WR_FIFO_OVER_FLOW_EN     HAL_BIT(10)
#define PSRAMC_WR_FIFO_FULL_EN          HAL_BIT(6)
#define PSRAMC_WR_FIFO_EMP_EN           HAL_BIT(5)
#define PSRAMC_WR_FIFO_READY_RQ_EN      HAL_BIT(4)
#define PSRAMC_RD_FIFO_FULL_EN          HAL_BIT(2)
#define PSRAMC_RD_FIFO_READY_RQ_EN      HAL_BIT(0)

    /*
     * bit field definition of PSRAMC->PSRAMC_INT_STA_REG 0x58
     */
#define PSRAMC_DMA_RW_OUT_NCACH_FLAG    HAL_BIT(15)
#define PSRAMC_RD_TOUT_FLAG             HAL_BIT(14)
#define PSRAMC_ISR_HEDY_TIOT            HAL_BIT(13)
#define PSRAMC_ISR_TRANS_END            HAL_BIT(12)
#define PSRAMC_WR_FIFO_OVER_FLOW        HAL_BIT(10)
#define PSRAMC_WR_FIFO_FULL_FLAG        HAL_BIT(6)
#define PSRAMC_WR_FIFO_EMP_FLAG         HAL_BIT(5)
#define PSRAMC_WR_FIFO_REQ_FLAG      	HAL_BIT(4)
#define PSRAMC_RD_FIFO_FULL_FLAG      	HAL_BIT(2)
#define PSRAMC_RD_FIFO_REQ_FLAG      	HAL_BIT(0)

    /*
     * bit field definition of PSRAMC->MEM_CTRL_DBG_STATE 0x60
     */
#define PSRAMC_PCTRL_STATE_SHIFT        (0)
#define PSRAMC_PCTRL_STATE_MASK         (0xF << PSRAMC_PCTRL_STATE_SHIFT)

    /*
     * bit field definition of PSRAMC->PSRAM_FORCE_CFG 0x6C
     */
#define PSRAMC_FORCE_CE_EN              HAL_BIT(31)
#define PSRAMC_FORCE_CYC_NUM_SHIFT      (0)
#define PSRAMC_FORCE_CYC_NUM_VMASK      (0x0FFFFU)
#define PSRAMC_FORCE_CYC_NUM_MASK       (PSRAMC_FORCE_CYC_NUM_VMASK << PSRAMC_FORCE_CYC_NUM_SHIFT)
#define PSRAMC_FORCE_CYC_NUM(n)         (((n) & PSRAMC_FORCE_CYC_NUM_VMASK) << PSRAMC_FORCE_CYC_NUM_SHIFT)

    /*
     * bit field definition of PSRAMC->PSRAM_COM_CFG 0x70
     */
#define PSRAMC_MAX_RD_LATENCY_SHIFT     (28)
#define PSRAMC_MAX_RD_LATENCY_VMASK     (0x0F)
#define PSRAMC_MAX_READ_LATENCY(l)      (((l) & PSRAMC_MAX_RD_LATENCY_VMASK) << PSRAMC_MAX_RD_LATENCY_SHIFT)

#define PSRAMC_MAX_CEN_LOW_CYC_SHIFT    (16)
#define PSRAMC_MAX_CEN_LOW_CYC_VMASK    (0x0FFF)
#define PSRAMC_MAX_CEN_LOW_CYC_MASK     ((PSRAMC_MAX_CEN_LOW_CYC_VMASK) << PSRAMC_MAX_CEN_LOW_CYC_SHIFT)
#define PSRAMC_MAX_CEN_LOW_CYC_NUM(n)   (((n) & PSRAMC_MAX_CEN_LOW_CYC_VMASK) << PSRAMC_MAX_CEN_LOW_CYC_SHIFT)

#define PSRAMC_MR_REG_ADDR_EN           HAL_BIT(14)
#define PSRAMC_COM_DQS_READ_WAIT_SHIFT  (12)
#define PSRAMC_COM_DQS_READ_WAIT_VMASK  (3U)
#define PSRAMC_COM_DQS_READ_WAIT(w)     (((w) & PSRAMC_COM_DQS_READ_WAIT_VMASK) << PSRAMC_COM_DQS_READ_WAIT_SHIFT)

#define PSRAMC_DUMMY_NUM_SHIFT          (10)
#define PSRAMC_DUMMY_NUM_MASK		(0x3 << PSRAMC_DUMMY_NUM_SHIFT)
    typedef enum {
        PSRAMC_DUM_NUM_CLC_1 = (0x0U << PSRAMC_DUMMY_NUM_SHIFT),
        PSRAMC_DUM_NUM_CLC_2 = (0x1U << PSRAMC_DUMMY_NUM_SHIFT),
        PSRAMC_DUM_NUM_CLC_3 = (0x2U << PSRAMC_DUMMY_NUM_SHIFT),
        PSRAMC_DUM_NUM_CLC_4 = (0x3U << PSRAMC_DUMMY_NUM_SHIFT)
    } PSRAMC_DumNumClc;

#define PSRAMC_MIN_WR_CLC_SHIFT         (8)
#define PSRAMC_MIN_WR_CLC_MASK		(0x3 << PSRAMC_MIN_WR_CLC_SHIFT)
    typedef enum {
        PSRAMC_MIN_WR_CLC_1 = (0x0U << PSRAMC_MIN_WR_CLC_SHIFT),
        PSRAMC_MIN_WR_CLC_2 = (0x1U << PSRAMC_MIN_WR_CLC_SHIFT),
        PSRAMC_MIN_WR_CLC_3 = (0x2U << PSRAMC_MIN_WR_CLC_SHIFT),
        PSRAMC_MIN_WR_CLC_4 = (0x3U << PSRAMC_MIN_WR_CLC_SHIFT)
    } PSRAMC_MinWrClc;

//#define PSRAMC_CLK_STOP_CE_LOW          HAL_BIT(7)
#define PSRAMC_CEDIS_CLK_VALID          HAL_BIT(6)
#define PSRAMC_WR_AF_DM_DUMMY           HAL_BIT(5)
#define PSRAMC_WR_AF_DQS_DUMMY          HAL_BIT(4)
#define PSRAMC_WR_NEED_DQS              HAL_BIT(3)
#define PSRAMC_CLK_OUTPUT_HLD           HAL_BIT(2)
#define PSRAMC_DDR_MODE_EN              HAL_BIT(1)
#define PSRAMC_CMD_HLD_THCYC            HAL_BIT(0)

    /*
     * bit field definition of PSRAMC->PSRAM_LC_CFG 0x74
     */
#define PSRAMC_RD_LC_TOUT_SHIFT         (16)
#define PSRAMC_RD_LC_TOUT_MASK          (0x0FF << PSRAMC_RD_LC_TOUT_SHIFT)
#define PSRAMC_DBUS_WR_LC_SHIFT         (8)
#define PSRAMC_DBUS_WR_LC_MASK          (0x0FF << PSRAMC_DBUS_WR_LC_SHIFT)
#define PSRAMC_SBUS_WR_LC_SHIFT         (0)
#define PSRAMC_SBUS_WR_LC_MASK          (0x0FF << PSRAMC_SBUS_WR_LC_SHIFT)

    /*
     * bit field definition of PSRAMC->PSRAM_TIM_CFG 0x78
     */
#define PSRAMC_DQS_OUTP_DHCYC_SHIFT     (20)
#define PSRAMC_DQS_OUTP_DHCYC_VMASK     (0x3)
#define PSRAMC_DQS_OUTP_DHCYC_MASK      (PSRAMC_DQS_OUTP_DHCYC_VMASK << PSRAMC_DQS_OUTP_DHCYC_SHIFT)
#define PSRAMC_DQS_OUTP_DHCYC(n)        (((n) & PSRAMC_DQS_OUTP_DHCYC_VMASK) << PSRAMC_DQS_OUTP_DHCYC_SHIFT)
#define PSRAMC_DQS_INP_DHCYC_SHIFT      (16)
#define PSRAMC_DQS_INP_DHCYC_VMASK      (PSRAMC_DQS_INP_DHCYC_VMASK)
#define PSRAMC_DQS_INP_DHCYC_MASK       (0x3 << PSRAMC_DQS_INP_DHCYC_SHIFT)
#define PSRAMC_DQS_INP_DHCYC(n)         (((n) & PSRAMC_DQS_INP_DHCYC_VMASK) << PSRAMC_DQS_INP_DHCYC_SHIFT)
#define PSRAMC_DM_OUTP_DHCYC_SHIFT      (12)
#define PSRAMC_DM_OUTP_DHCYC_VMASK	(0x3)
#define PSRAMC_DM_OUTP_DHCYC_MASK	(PSRAMC_DM_OUTP_DHCYC_VMASK << PSRAMC_DM_OUTP_DHCYC_SHIFT)
#define PSRAMC_DM_OUTP_DHCYC(n)         (((n) & PSRAMC_DM_OUTP_DHCYC_VMASK) << PSRAMC_DM_OUTP_DHCYC_SHIFT)
#define PSRAMC_CS_OUTP_DHCYC_SHIFT      (8)
#define PSRAMC_CS_OUTP_DHCYC_VMASK      (0x3)
#define PSRAMC_CS_OUTP_DHCYC_MASK       (PSRAMC_CS_OUTP_DHCYC_VMASK << PSRAMC_CS_OUTP_DHCYC_SHIFT)
#define PSRAMC_CS_OUTP_DHCYC(n)         (((n) & PSRAMC_CS_OUTP_DHCYC_VMASK) << PSRAMC_CS_OUTP_DHCYC_SHIFT)
#define PSRAMC_CLK_OUTP_DHCYC_SHIFT     (4)
#define PSRAMC_CLK_OUTP_DHCYC_VMASK     (0x3)
#define PSRAMC_CLK_OUTP_DHCYC_MASK      (PSRAMC_CLK_OUTP_DHCYC_VMASK << PSRAMC_CLK_OUTP_DHCYC_SHIFT)
#define PSRAMC_CLK_OUTP_DHCYC(n)        (((n) & PSRAMC_CLK_OUTP_DHCYC_VMASK) << PSRAMC_CLK_OUTP_DHCYC_SHIFT)
#define PSRAMC_ADQ_OUTP_DHCYC_SHIFT     (0)
#define PSRAMC_ADQ_OUTP_DHCYC_VMASK     (0x3)
#define PSRAMC_ADQ_OUTP_DHCYC_MASK      (PSRAMC_ADQ_OUTP_DHCYC_VMASK<< PSRAMC_ADQ_OUTP_DHCYC_SHIFT)
#define PSRAMC_ADQ_OUTP_DHCYC(n)        (((n) & PSRAMC_ADQ_OUTP_DHCYC_VMASK) << 0)

    /*
     * bit field definition of PSRAMC->PSRAM_DQS_DELAY_CFG 0x7C
     */
#define PSRAMC_OVERWR_CAL               HAL_BIT(24)
#define PSRAMC_OVERWR_CAL_SHIFT         (16)
#define PSRAMC_OVERWR_CAL_VMASK         (0x3F)
#define PSRAMC_OVERWR_CAL_VAL(n)	(((n) & PSRAMC_OVERWR_CAL_VMASK) << PSRAMC_OVERWR_CAL_SHIFT)
#define PSRAMC_CAL_SUCCEED		HAL_BIT(12)
#define PSRAMC_CAL_RESULT_VAL_SHIFT	(4)
#define PSRAMC_CAL_RESULT_VAL_MASK	(0x3F << PSRAMC_CAL_RESULT_VAL_SHIFT)
#define PSRAMC_START_DQS_DELAY_CAL	HAL_BIT(0)

    /*
     * bit field definition of PSRAMC->PSRAM_ADDR
     */
#define PSRAMC_START_POS(addr)          ((addr) & 0xFFFFFFF0)
#define PSRAMC_END_POS(addr)            ((addr) & 0xFFFFFFF0)
#define PSRAMC_ADDR_BIAS_EN             HAL_BIT(31)

    typedef struct {
        __IO uint32_t START_ADDR;       /* ,        Address offset: N * 0x4 + 0x00        */
        __IO uint32_t END_ADDR;         /* ,        Address offset: N * 0x4 + 0x04        */
        __IO uint32_t BIAS_ADDR;        /* ,        Address offset: N * 0x4 + 0x08        */
        __I  uint32_t RESERVE0C;        /* ,        Address offset: N * 0x4 + 0x0C        */
    } ADDR_T;

    typedef struct {
        __IO uint32_t MEM_COM_CFG;		/* ,	 Address offset: 0x000	 */
        __IO uint32_t OPI_CTRL_CMM_CFG; 	/* ,	 Address offset: 0x004	 */
        __IO uint32_t CACHE_RLVT_CFG;   	/* ,	 Address offset: 0x008	 */
        __IO uint32_t MEM_AC_CFG;		/* ,	 Address offset: 0x00C	 */
        __IO uint32_t C_RD_OPRT_CFG;            /* ,	 Address offset: 0x010	 */
        __IO uint32_t C_WD_OPRT_CFG;            /* ,	 Address offset: 0x014	 */
        __IO uint32_t C_RD_DUMMY_DATA_H;	/* ,	 Address offset: 0x018	 */
        __IO uint32_t C_RD_DUMMY_DATA_L;	/* ,	 Address offset: 0x01C	 */
        __IO uint32_t C_WD_DUMMY_DATA_H;	/* ,	 Address offset: 0x020	 */
        __IO uint32_t C_WD_DUMMY_DATA_L;	/* ,	 Address offset: 0x024	 */
        __IO uint32_t C_IO_SW_WAIT_TIME;	/* ,	 Address offset: 0x028	 */
        __IO uint32_t S_RW_OPRT_CFG;            /* ,	 Address offset: 0x02C	 */
        __IO uint32_t S_ADDR_CFG;		/* ,	 Address offset: 0x030	 */
        __IO uint32_t S_DUMMY_DATA_H;           /* ,	 Address offset: 0x034	 */
        __IO uint32_t S_DUMMY_DATA_L;           /* ,	 Address offset: 0x038	 */
        __IO uint32_t S_IO_SW_WAIT_TIME;	/* ,	 Address offset: 0x03C	 */
        __IO uint32_t S_WD_DATA_BYTE_NUM;	/* ,	 Address offset: 0x040	 */
        __IO uint32_t S_RD_DATA_BYTE_NUM;	/* ,	 Address offset: 0x044	 */
        __IO uint32_t S_START_SEND_REG;         /* ,	 Address offset: 0x048	 */
        __IO uint32_t FIFO_TRIGGER_LEVEL;	/* ,	 Address offset: 0x04C	 */
        __I  uint32_t FIFO_STATUS_REG;		/* ,	 Address offset: 0x050	 */
        __IO uint32_t INT_ENABLE_REG;		/* ,	 Address offset: 0x054	 */
        __IO uint32_t INT_STATUS_REG;		/* ,	 Address offset: 0x058	 */
        __IO uint32_t XIP_WARP_MODE_EXE_IDCT;	/* ,	 Address offset: 0x05C	 */
        __I  uint32_t MEM_CTRL_DBG_STATE;	/* ,	 Address offset: 0x060	 */
        __I  uint32_t MEM_CTRL_SBUS_DBG_CNTH;	/* ,	 Address offset: 0x064	 */
        __I  uint32_t MEM_CTRL_SBUS_DBG_CNTL;	/* ,	 Address offset: 0x068	 */
        __IO uint32_t PSRAM_FORCE_CFG;		/* ,	 Address offset: 0x06C	 */
        __IO uint32_t PSRAM_COM_CFG;		/* ,	 Address offset: 0x070	 */
        __IO uint32_t PSRAM_LAT_CFG;		/* ,	 Address offset: 0x074	 */
        __IO uint32_t PSRAM_TIM_CFG;		/* ,	 Address offset: 0x078	 */
        __IO uint32_t PSRAM_DQS_DELAY_CFG;	/* ,	 Address offset: 0x07C	 */
        ADDR_T   PSRAM_ADDR[8];		/* ,	 Address offset: 0x080~0x0FC */
        __IO uint32_t S_WD_DATA_REG;            /* ,	 Address offset: 0x100	 */
        __IO uint32_t RESERVE101[63];
        __I  uint32_t S_RD_DATA_REG;            /* ,	 Address offset: 0x200	 */
    } PSRAM_CTRL_T;

#define PSRAM_CTRL ((PSRAM_CTRL_T *)PSRAM_CTRL_BASE)

    /**
     * @brief PSRAM Controller initialization parameters
     */
    typedef struct {
        uint32_t p_type;
        uint32_t freq;			/*!< PSRAM working frequency */
//	uint32_t t_shsl_ns;		/*!< PSRAM t_shsl parameter. for calculate the cs delay. */
        uint8_t rdata_w;                /*!< PSRAM receive data wait cycle(0~3), base on board line length */
    } PSRAMCtrl_InitParam;

    typedef struct {
        uint8_t data_bits;
    } HAL_PSRAMGPIOCfg;

    struct psram_ctrl;

    int32_t HAL_PsramCtrl_Request(struct psram_ctrl *ctrl, struct psram_request *mrq);
    void HAL_PsramCtrl_Set_DBUS_WR_LATENCY(struct psram_ctrl *ctrl, uint32_t lat);
    void HAL_PsramCtrl_Set_SBUS_WR_LATENCY(struct psram_ctrl *ctrl, uint32_t lat);
    uint32_t HAL_PsramCtrl_Set_BusWidth(struct psram_ctrl *ctrl, uint32_t width);
    void HAL_Psram_SbusCfg(struct psram_ctrl *ctrl, uint32_t opcfg, uint32_t wait, uint32_t waitcfg);
    void HAL_PsramCtrl_IDbusCfg(struct psram_ctrl *ctrl, uint32_t write, uint32_t opcfg, uint32_t wait, uint32_t waitcfg);
    void HAL_PsramCtrl_CacheCfg(struct psram_ctrl *ctrl, uint32_t cbus_wsize_bus);
    void HAL_PsramCtrl_MaxCE_LowCyc(struct psram_ctrl *ctrl, uint32_t clk);

    /**
     * @brief Initialize Psram controller.
     * @param cfg:
     * 	   @arg cfg->freq: Psram working frequency.
     * @retval HAL_Status: The status of driver.
     */
    HAL_Status HAL_PsramCtrl_Init(struct psram_ctrl *ctrl, const PSRAMCtrl_InitParam *cfg);

    /**
    * @brief Deinitialize Psram controller.
    * @param None
    * @retval HAL_Status: The status of driver.
    */
    HAL_Status HAL_PsramCtrl_Deinit(struct psram_ctrl *ctrl);
    /**
     * @brief Open psram controller SBUS.
     * @note At the same time, it will disable XIP and suspend schedule.
     * @param None
     * @retval HAL_Status: The status of driver.
     */
    struct psram_ctrl *HAL_PsramCtrl_Open(uint32_t id);
    /**
     * @brief Close psram controller SBUS.
     * @param None
     * @retval HAL_Status: The status of driver.
     */
    HAL_Status HAL_PsramCtrl_Close(struct psram_ctrl *ctrl);
    struct psram_ctrl *HAL_PsramCtrl_Create(uint32_t id, const PSRAMCtrl_InitParam *cfg);

    HAL_Status HAL_PsramCtrl_Destory(struct psram_ctrl *ctrl);
    void HAL_PsramCtrl_IDBUS_Dma_Enable(uint32_t en);
    void HAL_PsramCtrl_Set_Address_Field(struct psram_ctrl *ctrl, uint32_t id,
            uint32_t startaddr, uint32_t endaddr, uint32_t bias_addr);
    int32_t HAL_PsramCtrl_Sbus_Transfer(struct psram_ctrl *ctrl, struct psram_request *mrq, bool dma);
    void HAL_PsramCtrl_Set_Address_Field(struct psram_ctrl *ctrl, uint32_t id,
            uint32_t startaddr, uint32_t endaddr, uint32_t bias_addr);
    int32_t HAL_PsramCtrl_Set_DQS_Delay_Cal(uint32_t clk);
    void HAL_PsramCtrl_Set_RD_BuffSize(PSRAMC_CacheLLCfg size);
    uint32_t HAL_PsramCtrl_Get_RD_BuffSize();
    void HAL_PsramCtrl_Set_WR_BuffSize(PSRAMC_WrCacheLL size);
    uint32_t HAL_PsramCtrl_Get_WR_BuffSize();
#endif /* (__CONFIG_CHIP_ARCH_VER == 2) */

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_PSRAMCTRL_H_ */
