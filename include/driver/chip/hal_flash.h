/**
  * @file  hal_flash.h
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

#ifndef _DRIVER_CHIP_HAL_FLASH_H_
#define _DRIVER_CHIP_HAL_FLASH_H_
#include "sys/list.h"
#include "driver/chip/private/hal_os.h"
#include "driver/chip/hal_def.h"
#include "driver/chip/flashchip/flash_chip.h"
#include "driver/chip/hal_spi.h"

#ifdef __cplusplus
extern "C" {
#endif
#define FLASH_DMA_TRANSFER_MIN_SIZE (64)

#define FLASH_FLASHC_ENABLE (1)
#define FLASH_SPI_ENABLE (!defined(__CONFIG_BOOTLOADER))

typedef enum FlashControlCmd
{
	FLASH_GET_MIN_ERASE_SIZE,
	FLASH_WRITE_STATUS,
	FLASH_READ_STATUS,
	FLASH_ENABLE_32BIT_ADDR,
	FLASH_SET_READ_MODE,
	FLASH_SET_PAGEPROGRAM_MODE,
	/*TODO: tbc...*/
} FlashControlCmd;

struct FlashDrv
{
	/*
		attribute
	*/
	int dev;
	uint32_t sizeToDma;

	/*
		public interface
	*/
	HAL_Status (*write)(struct FlashChip *chip, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data);
	HAL_Status (*read)(struct FlashChip *chip, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data);
	HAL_Status (*open)(struct FlashChip *chip);
	HAL_Status (*close)(struct FlashChip *chip);
	HAL_Status (*setFreq)(struct FlashChip *chip, uint32_t freq);
	void (*msleep)(struct FlashChip *chip, uint32_t ms);
	void (*destroy)(struct FlashChip *);
	HAL_Status (*ioctl)(struct FlashChip *chip, FlashControlCmd attr, uint32_t arg);
	void *platform_data;
};

/*
	Flash
*/
struct FlashDev
{
	struct list_head node;
	HAL_Mutex lock;

	uint32_t flash;
	struct FlashDrv *drv;
	struct FlashChip *chip;
	FlashReadMode rmode;
	FlashPageProgramMode wmode;
	uint8_t usercnt; /* not thread safe */

#ifdef CONFIG_PM
	struct soc_device *pm;
#endif
};

struct FlashDrv *FlashDriverCreator(int driver);
int FlashDriverDestory(struct FlashDrv *drv);

/*
	Flash Board Config
*/
enum FlashBoardType
{
#if FLASH_FLASHC_ENABLE
	FLASH_DRV_FLASHC,	/*!< flash controller driver */
#endif
#if FLASH_SPI_ENABLE
	FLASH_DRV_SPI        /*!< spi driver */
#endif
};

typedef struct FlashcBoardCfg
{
	uint32_t clk;	/*!< flash clock */
} FlashcBoardCfg;

typedef struct SpiBoardCfg
{
	uint32_t clk;   /*!< flash clock */
	SPI_Port port;	/*!< spi port */
	SPI_CS cs;      /*!< cs pin */
} SpiBoardCfg;

typedef struct FlashBoardCfg
{
	enum FlashBoardType type;   /*!< flash driver */
	FlashReadMode mode;         /*!< read mode to flash */
	union {
		FlashcBoardCfg flashc;	/*!< flash driver controller configuration. Notice!! flashc support all read mode */
		SpiBoardCfg spi; 		/*!< spi driver configuration. Notice!! spi only support normal read, fast read, dual output mode */
	};
} FlashBoardCfg;

typedef struct FlashControlStatus
{
	FlashStatus status;
	uint8_t *data;
} FlashControlStatus;

HAL_Status HAL_Flash_Init(uint32_t flash);

HAL_Status HAL_Flash_Deinit(uint32_t flash);

HAL_Status HAL_Flash_Open(uint32_t flash, uint32_t timeout_ms);

HAL_Status HAL_Flash_Close(uint32_t flash);

HAL_Status HAL_Flash_Ioctl(uint32_t flash, FlashControlCmd attr, uint32_t arg);

HAL_Status HAL_Flash_Overwrite(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size);

HAL_Status HAL_Flash_Write(uint32_t flash, uint32_t addr, const uint8_t *data, uint32_t size);

HAL_Status HAL_Flash_Read(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size);

HAL_Status HAL_Flash_Erase(uint32_t flash, FlashEraseMode blk_size, uint32_t addr, uint32_t blk_cnt);

HAL_Status HAL_Flash_MemoryOf(uint32_t flash, FlashEraseMode size, uint32_t addr, uint32_t *start);

int HAL_Flash_Check(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size);

void HAL_Flash_SetDbgMask(uint8_t dbg_mask);

struct FlashDev;

struct FlashDev *getFlashDev(uint32_t flash);
struct FlashChip *getFlashChip(struct FlashDev *dev);

HAL_Status HAL_Flash_WaitCompl(struct FlashDev *dev, int32_t timeout_ms);

struct FlashDrv *flashcDriverCreate(int dev, FlashBoardCfg *bcfg);

#if FLASH_SPI_ENABLE
struct FlashDrv *spiDriverCreate(int dev, FlashBoardCfg *bcfg);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_FLASH_H_ */
