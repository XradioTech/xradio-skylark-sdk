/**
  * @file  hal_flash.c
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

#include "sys/list.h"
#include "sys/param.h"
#include "sys/defs.h"

#include "rom/pm/pm.h"
#include "driver/chip/hal_spi.h"
#include "rom/driver/chip/hal_flashctrl.h"
#include "rom/driver/chip/hal_flash.h"
#ifndef __CONFIG_ROM
#include "driver/chip/hal_wdg.h" /* for HAL_Alive() */
#endif
#include "hal_base.h"
#include "flashchip/flash_debug.h"

static FlashBoardCfg *getFlashBoardCfg(int minor);
#ifdef CONFIG_PM
static struct soc_device_driver flash_drv;
#endif

struct flash_controller *flash_ctrl;

uint8_t flash_dbg_mask = FLASH_ALE_FLAG | FLASH_ERR_FLAG | FLASH_NWA_FLAG | FD_ERR_FLAG | FD_INF_FLAG;

void HAL_Flash_SetDbgMask(uint8_t dbg_mask)
{
	flash_dbg_mask = dbg_mask;
}

HAL_Status HAL_Flash_WaitCompl(struct FlashDev *dev, int32_t timeout_ms)
{
#define FLASH_WAIT_TIME (1)
	uint32_t loop = 0;

	while (dev->chip->isBusy(dev->chip) > 0)
	{
		dev->drv->msleep(dev->chip, FLASH_WAIT_TIME);
		timeout_ms -= FLASH_WAIT_TIME;
		if (timeout_ms <= 0) {
			FD_ERROR("wait clr busy timeout!");
			return HAL_TIMEOUT;
		}
		if (++loop == 1000) {
			HAL_Alive();
			loop = 0;
		}
	}
	return HAL_OK;
#undef FLASH_WAIT_TIME
}

/**
  * @internal
  * @brief Flash driver open.
  * @param base: driver.
  * @retval HAL_Status: The status of driver
  */
HAL_Status flashcFlashOpen(struct FlashChip *chip)
{
	FD_DEBUG("open");
	return HAL_Flashc_Open(chip->flash_ctrl);
}

/**
  * @internal
  * @brief Flash driver close.
  * @param base: driver.
  * @retval HAL_Status: The status of driver
  */
HAL_Status flashcFlashClose(struct FlashChip *chip)
{
	HAL_Status ret = HAL_Flashc_Close(chip->flash_ctrl);
	FD_DEBUG("close");
	return ret;
}

void insToFcIns(InstructionField *ins, FC_InstructionField *fcins)
{
	if (ins == NULL)
		return;

	if (ins->pdata)
		fcins->pdata = ins->pdata;
	else
		fcins->pdata = (uint8_t *)&ins->data;
	fcins->len = ins->len;
	fcins->line = (FC_CycleBits)((ins->line == 4) ? FC_CYCLEBITS_4 : ins->line);
}

#define INS_CMD  (0)
#define INS_ADDR (1)
#define INS_DUM  (2)
#define INS_DATA (3)

/**
  * @internal
  * @brief Flash driver write.
  * @param base: Driver.
  * @param cmd: Instruction command field.
  * @param addr: Instruction address field.
  * @param dummy: Instruction dummy field.
  * @param data: Instruction data field.
  * @retval HAL_Status: The status of driver
  */
HAL_Status flashcFlashWrite(struct FlashChip *dev, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	int dma = 0;
	FC_InstructionField tmp[4];
	struct FlashDrv *drv = dev->mDriver;

	HAL_Memset(tmp, 0, sizeof(tmp));

	insToFcIns(cmd, &tmp[INS_CMD]);
	insToFcIns(addr, &tmp[INS_ADDR]);
	insToFcIns(dummy, &tmp[INS_DUM]);
	insToFcIns(data, &tmp[INS_DATA]);

	if (tmp[INS_DATA].len >= (drv->sizeToDma - 1) &&  tmp[INS_DATA].len < 128 * 1024)
		dma = 1;

	return HAL_Flashc_Transfer(dev->flash_ctrl, 1, &tmp[INS_CMD], &tmp[INS_ADDR], &tmp[INS_DUM], &tmp[INS_DATA], dma);
}

/**
  * @internal
  * @brief Flash driver read.
  * @param base: Driver.
  * @param cmd: Instruction command field.
  * @param addr: Instruction address field.
  * @param dummy: Instruction dummy field.
  * @param data: Instruction data field.
  * @retval HAL_Status: The status of driver
  */
HAL_Status flashcFlashRead(struct FlashChip *dev, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	int dma = 0;
	FC_InstructionField tmp[4];
	struct FlashDrv *drv = dev->mDriver;

	HAL_Memset(tmp, 0, sizeof(tmp));

	insToFcIns(cmd, &tmp[INS_CMD]);
	insToFcIns(addr, &tmp[INS_ADDR]);
	insToFcIns(dummy, &tmp[INS_DUM]);
	insToFcIns(data, &tmp[INS_DATA]);

	if (tmp[INS_DATA].len >= (drv->sizeToDma - 1) &&  tmp[INS_DATA].len < 128 * 1024)
		dma = 1;

	return HAL_Flashc_Transfer(dev->flash_ctrl, 0, &tmp[INS_CMD], &tmp[INS_ADDR], &tmp[INS_DUM], &tmp[INS_DATA], dma);
}

/**
  * @internal
  * @brief Flash driver set working frequency.
  * @param base: Driver.
  * @param freq: Device and driver working frequency.
  * @retval HAL_Status: The status of driver
  */
HAL_Status flashcFlashSetFreq(struct FlashChip *dev, uint32_t freq)
{
	/* TODO: tbc... */
	return HAL_INVALID;
}

/**
  * @internal
  * @brief Sleep realization by driver.
  * @note For some reason(XIP), system sleep function can't be used in some
  *       case. So the realiztion of sleep should be performed by driver.
  * @param base: Driver.
  * @param ms: Sleep or wait sometime in millisecond.
  * @retval HAL_Status: The status of driver
  */
void flashcFlashMsleep(struct FlashChip *dev, uint32_t ms)
{
	HAL_Flashc_Delay(dev->flash_ctrl, ms * 1000);
}

/**
  * @internal
  * @brief Destroy flash driver.
  * @note This may not be used.
  * @param base: Driver.
  * @retval HAL_Status: The status of driver
  */
void flashcFlashDestroy(struct FlashChip *dev)
{
	HAL_Flashc_Deinit(dev->flash_ctrl);
	HAL_Free(dev->mDriver);
}

/**
  * @internal
  * @brief flash driver control.
  * @param base: Driver.
  * @param attr: flash control cmd.
  * @param arg: flash control arguement
  * @retval HAL_Status: The status of driver
  */
HAL_Status flashcFlashIoctl(struct FlashChip *dev, FlashControlCmd cmd, uint32_t arg)
{
	FC_InstructionField fc_ins;
	uint8_t fc_cmd;
	switch(cmd)
	{
		case FLASH_ENABLE_32BIT_ADDR:
			fc_cmd = 0xB7;
			fc_ins.len = 1;
			fc_ins.line = 1;
			fc_ins.pdata = &fc_cmd;

			HAL_Flashc_Transfer(dev->flash_ctrl, 0, &fc_ins, NULL, NULL, NULL, 1);
			HAL_Flashc_Ioctl(dev->flash_ctrl, FC_CMD_ENABLE_32BITADDR_MODE, (void*)arg);
			break;
		default:
			break;
	}
	return HAL_INVALID;
}

/**
  * @internal
  * @brief Create a flash driver.
  * @param dev: Flash device number, but not minor number.
  * @param bcfg: Config from board config.
  * @retval HAL_Status: The status of driver
  */
struct FlashDrv *rom_flashcDriverCreate(int dev, FlashBoardCfg *bcfg)
{
	struct FlashDrv *drv;

	drv = HAL_Malloc(sizeof(struct FlashDrv));
	if (drv == NULL) {
		FD_ERROR("no mem");
		return drv;
	}
	HAL_Memset(drv, 0, sizeof(struct FlashDrv));

	Flashc_Config cfg;
	cfg.freq = bcfg->flashc.clk;
	HAL_Flashc_Init(flash_ctrl, &cfg);

	drv->dev = dev;
	drv->open = flashcFlashOpen;
	drv->close = flashcFlashClose;
	drv->read = flashcFlashRead;
	drv->write = flashcFlashWrite;
	drv->setFreq = flashcFlashSetFreq;
	drv->msleep = flashcFlashMsleep;
	drv->destroy = flashcFlashDestroy;
	drv->ioctl = flashcFlashIoctl;
	drv->sizeToDma = FLASH_DMA_TRANSFER_MIN_SIZE;

	return drv;
}

/**
  * @internal
  * @brief Create a flash driver according board config.
  * @param minor: flash number = flash minor number, from board config.
  * @retval HAL_Status: The status of driver
  */
static struct FlashDrv *flashDriverCreate(int minor, FlashBoardCfg *cfg)
{
	struct FlashDrv *drv = NULL;
	int dev = HAL_MKDEV(HAL_DEV_MAJOR_FLASH, minor);

	if (cfg->type == FLASH_DRV_FLASHC) {
		drv = flashcDriverCreate(dev, cfg);
	}
#if FLASH_SPI_ENABLE // check when ROM code ###############!!!!!
	else if (cfg->type == FLASH_DRV_SPI) {
		drv = spiDriverCreate(dev, cfg);
	}
#endif

	if (drv == NULL)
		FD_ERROR("create fail");

	/*manage driver gpio. tbc...*/

	return drv;
}

/**
  * @internal
  * @brief Destroy a flash driver.
  * @param base: Driver.
  * @retval HAL_Status: The status of driver
  */
static int flashDriverDestory(struct FlashChip *chip)
{
	if (chip == NULL)
		return -1;

	chip->mDriver->destroy(chip);
	return 0;
}


/*
	FlashBoard
*/
static FlashBoardCfg *getFlashBoardCfg(int minor)
{
	FlashBoardCfg *cfg = NULL;
	int dev = HAL_MKDEV(HAL_DEV_MAJOR_FLASH, minor);

	HAL_BoardIoctl(HAL_BIR_GET_CFG, dev, (uint32_t)&cfg);
	if (cfg == NULL)
		FD_ERROR("error");

	return cfg;
}

struct list_head flashNodeHead = {
		.next = &flashNodeHead,
		.prev = &flashNodeHead
};

struct FlashChip *getFlashChip(struct FlashDev *dev)
{
	return dev->chip;
}

FlashReadMode getFlashMode(struct FlashDev *dev)
{
	return dev->rmode;
}

struct FlashDev *getFlashDev(uint32_t flash)
{
	struct FlashDev *dev = NULL;
	struct FlashDev *itor = NULL;

	list_for_each_entry(itor, &flashNodeHead, node) {
		if (itor->flash == flash) {
			dev = itor;
			break;
		}
	}

	if (dev == NULL)
		FD_ERROR("failed");

	return dev;
}

static int deleteFlashDev(struct FlashDev *dev)
{
	if (dev == NULL) {
		FD_ERROR("NULL flash device");
		return -1;
	}

	list_del(&dev->node);

	return 0;
}

static int addFlashDev(struct FlashDev *dev)
{
	if (dev == NULL) {
		FD_ERROR("NULL flash device");
		return -1;
	}

	list_add_tail(&dev->node, &flashNodeHead);

	return 0;
}

/**
  * @brief Initializes flash Device.
  * @note The flash device configuration is in the board_config g_flash_cfg.
  *       Device number is the g_flash_cfg vector sequency number.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number
  * @retval HAL_Status: The status of driver
  */
HAL_Status HAL_Flash_Init(uint32_t flash)
{
	HAL_Status ret;
	struct FlashDrv *drv = NULL;
	struct FlashChip *chip = NULL;
	struct FlashDev *dev = NULL;
	FlashBoardCfg *cfg;

	/* TODO: flash_ctrl should not init here!!! */
	if (flash_ctrl == NULL) {
		flash_ctrl = HAL_Flashc_Create(flash);
	}
	if (flash_ctrl == NULL) {
		FD_ERROR("flashc create failed");
		goto failed;
	}
	HAL_Flashc_IncRef(flash_ctrl);

	cfg = getFlashBoardCfg(flash);
	if (cfg == NULL) {
		FD_ERROR("getFlashBoardCfg failed");
		goto failed;
	}

	dev = HAL_Malloc(sizeof(*dev));
	if (dev == NULL) {
		FD_ERROR("malloc dev failed");
		goto failed;
	}
	HAL_Memset(dev, 0, sizeof(*dev));

	chip = HAL_Malloc(sizeof(struct FlashChip));
	if (chip == NULL) {
		FD_ERROR("FlashChip Malloc failed");
		goto failed;
	}
	HAL_Memset(chip, 0, sizeof(struct FlashChip));
	chip->flash_ctrl = flash_ctrl;

	drv = flashDriverCreate(flash, cfg);
	if (drv == NULL) {
		FD_ERROR("flashDriverCreate failed");
		goto failed;
	}

	FlashChipEnum(chip, drv);
	/*TODO: get read mode from board, and init*/

	dev->chip = chip;
	dev->drv = drv;
	dev->flash = flash;
	dev->rmode = cfg->mode;
	dev->wmode = FLASH_PAGEPROGRAM;
	dev->usercnt = 0;
	INIT_LIST_HEAD(&dev->node);
	ret = HAL_MutexInit(&dev->lock);
	if (ret != HAL_OK)
	{
		FD_ERROR("mutex init failed: %d", ret);
		goto failed;
	}

	addFlashDev(dev);
	dev->drv->open(dev->chip);
#ifdef __CONFIG_CHIP_XR875
	if(chip->cfg.mSize > 128*1024*1024) {
		printf("Enter 32 Bit Address Mode\n");
		dev->drv->ioctl(dev->chip, FLASH_ENABLE_32BIT_ADDR, 0);
		dev->chip->flash_ctrl->externAddr_on = 1;
	}
#endif
	//	chip->setFreq(chip, );
	if (dev->rmode & (FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE | FLASH_READ_QPI_MODE))
	{
		dev->chip->switchReadMode(dev->chip, dev->rmode);

		HAL_Flash_WaitCompl(dev, 5000);//wait busy

		if (dev->rmode & FLASH_READ_QPI_MODE)
			dev->chip->enableQPIMode(dev->chip);
	}
	dev->drv->close(dev->chip);

	FD_INFO("mode: 0x%x, freq: %dHz, drv: %d", cfg->mode, cfg->flashc.clk, cfg->type);
#ifdef CONFIG_PM
	struct soc_device *flash_pm = HAL_Malloc(sizeof(*flash_pm));
	HAL_Memset(flash_pm, 0, sizeof(*flash_pm));
	flash_pm->name = "Flash";
	flash_pm->driver = &flash_drv;
	flash_pm->platform_data = (void *)flash;
	dev->pm = flash_pm;
	pm_register_ops(flash_pm);
#endif

	return HAL_OK;

failed:
	FD_ERROR("failed");

	if (drv != NULL)
		flashDriverDestory(chip);

	if (chip) {
		HAL_Free(chip);
	}
	if (dev != NULL)
		HAL_Free(dev);

	if (flash_ctrl) {
		if (HAL_Flashc_DecRef(flash_ctrl) == 0) {
			HAL_Flashc_Destory(flash_ctrl);
			flash_ctrl = NULL;
		}
	}

	return HAL_ERROR;
}

/**
  * @brief Deinitializes flash Device.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number
  * @retval HAL_Status: The status of driver
  */
HAL_Status HAL_Flash_Deinit(uint32_t flash)
{
	struct FlashDev *dev = getFlashDev(flash);
	struct FlashDrv *drv = dev->drv;
	struct FlashChip *chip = dev->chip;

	/*not thread safe*/
	if (dev->usercnt != 0)
		return HAL_TIMEOUT;

#ifdef CONFIG_PM
	pm_unregister_ops(dev->pm);
	HAL_Free(dev->pm);
#endif

	drv->open(chip);
	chip->reset(chip);
	drv->close(chip);

	deleteFlashDev(dev);
	HAL_MutexDeinit(&dev->lock);

	if (dev->drv != NULL) {
		flashDriverDestory(dev->chip);
	}

	if (dev->chip) {
		HAL_Free(dev->chip);
	}

	if (dev != NULL) {
		HAL_Free(dev);
	}

	if (flash_ctrl) {
		if (HAL_Flashc_DecRef(flash_ctrl) == 0) {
			HAL_Flashc_Destory(flash_ctrl);
			flash_ctrl = NULL;
		}
	}

	return HAL_OK;
}

/**
  * @brief Open flash Device.
  * @note Opened a flash device, other user can't open again, so please
  *       close it while don't need the flash device.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number.
  * @param timeout_ms: timeout in millisecond.
  * @retval HAL_Status: The status of driver
  */
HAL_Status rom_HAL_Flash_Open(uint32_t flash, uint32_t timeout_ms)
{
	struct FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;

	ret = HAL_MutexLock(&dev->lock, timeout_ms);
	if (ret == HAL_OK)
		dev->usercnt++;

	return ret;
}

/**
  * @brief Close flash Device.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number
  * @retval HAL_Status: The status of driver
  */
HAL_Status rom_HAL_Flash_Close(uint32_t flash)
{
	struct FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;

	ret = HAL_MutexUnlock(&dev->lock);
	if (ret == HAL_OK)
		dev->usercnt--;

	return ret;
}

/**
  * @brief Write flash Device memory, no need to erase first and  other memory
  *        will not be change. Only can be used in the flash supported 4k erase.
  * @note Only the flash supported 4k erase!! FDCM module is much fast than
  *       this function.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number
  * @param addr: the address of memory.
  * @param data: the data needed to write to flash device.
  * @param size: the data size needed to write.
  * @retval HAL_Status: The status of driver
  */
HAL_Status HAL_Flash_Overwrite(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size)
{
	struct FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;
	uint8_t *buf = NULL;
	uint8_t *ptr = data;
	uint32_t paddr = addr;
	int32_t  left = (int32_t)size;
	uint32_t pp_size;
	uint32_t saddr;

	FD_DEBUG("mEraseSizeSupport 0x%x", dev->chip->cfg.mEraseSizeSupport);
	if (!(dev->chip->cfg.mEraseSizeSupport & FLASH_ERASE_4KB))
		return HAL_INVALID;

	buf = HAL_Malloc(FLASH_ERASE_4KB);
	if (buf == NULL)
		goto out;

	while (left > 0)
	{
		HAL_Flash_MemoryOf(flash, FLASH_ERASE_4KB, paddr, &saddr);
		HAL_Flash_Read(flash, saddr, buf, FLASH_ERASE_4KB);
		ret = HAL_Flash_Erase(flash, FLASH_ERASE_4KB, saddr, 1);
		if (ret != HAL_OK)
			goto out;

		pp_size = MIN(left, FLASH_ERASE_4KB - (paddr - saddr));
		HAL_Memcpy(buf + (paddr - saddr), ptr, pp_size);

		ret = HAL_Flash_Write(flash, saddr, buf, FLASH_ERASE_4KB);
		if (ret != HAL_OK)
			goto out;

		ptr += pp_size;
		paddr += pp_size;
		left -= pp_size;
	}

out:
	if (buf != NULL)
		HAL_Free(buf);

	return ret;
}

/**
  * @brief Write flash Device memory, if this memory has been written before,
  *        the memory must be erase first by user. HAL_Flash_Check can check
  *        this memory whether is writable.
  * @note If write a written memory, the memory data will a be error data.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number
  * @param addr: the address of memory.
  * @param data: the data needed to write to flash device.
  * @param size: the data size needed to write.
  * @retval HAL_Status: The status of driver
  */
HAL_Status rom_HAL_Flash_Write(uint32_t flash, uint32_t addr, const uint8_t *data, uint32_t size)
{
	struct FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;
	uint32_t address = addr;
	uint32_t left = size;
	const uint8_t *ptr = data;
	uint32_t pp_size;

	FD_DEBUG("%u: w%u, a: 0x%x", flash, size, addr);

	if ((NULL == dev) || (NULL == dev->chip->pageProgram) || (0 == size) || (addr+size >= dev->chip->cfg.mSize)) {
		FD_ERROR("Invalid param");
		return HAL_INVALID;
	}

	while (left > 0) {
		pp_size = MIN(left, dev->chip->mPageSize - (address % dev->chip->mPageSize));

		dev->drv->open(dev->chip);

		dev->chip->writeEnable(dev->chip);
		//FD_DEBUG("WE");
		ret = dev->chip->pageProgram(dev->chip, dev->wmode, address, ptr, pp_size);
		//FD_DEBUG("PP");
		dev->chip->writeDisable(dev->chip);
		//FD_DEBUG("WD");

		if (ret < 0)
			break;

		ret = HAL_Flash_WaitCompl(dev, 5000);

		dev->drv->close(dev->chip);

		if (ret < 0)
			FD_ERROR("wr failed: %d", ret);

		address += pp_size;
		ptr += pp_size;
		left -= pp_size;
	}

	if (ret != 0)
		FD_ERROR("wr failed");

	return ret;
}

/**
  * @brief Read flash device memory.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number
  * @param addr: the address of memory.
  * @param data: the data needed to write to flash device.
  * @param size: the data size needed to read.
  * @retval HAL_Status: The status of driver
  */
HAL_Status rom_HAL_Flash_Read(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size)
{
	struct FlashDev *dev = getFlashDev(flash);
	HAL_Status ret;

	FD_DEBUG("%u: r%u, a: 0x%x", flash, size, addr);

	if ((NULL == dev) || (NULL == dev->chip->read) || (size == 0) || (addr+size >= dev->chip->cfg.mSize)) {
		FD_ERROR("Invalid param");
		return HAL_INVALID;
	}

	dev->drv->open(dev->chip);
	ret = dev->chip->read(dev->chip, dev->rmode, addr, data, size);
	dev->drv->close(dev->chip);

	if (ret != 0)
		FD_ERROR("read failed");

	return ret;
}

/**
  * @brief Erase flash device memory. Flash can only erase sector or block or
  *        chip.
  * @note Some flash is not support some erase mode, for example: FLASH M25P64
  *       is only support FLASH_ERASE_CHIP and FLASH_ERASE_64KB.
  *       The erase address must be aligned to erase mode size, for example:
  *       the address should be n * 0x1000 in the erase 4kb mode, this address
  *       can be calculated in HAL_Flash_MemoryOf.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number.
  * @param blk_size:
  *        @arg FLASH_ERASE_4KB: 4kbyte erase mode.
  *        @arg FLASH_ERASE_32KB: 32kbtye erase mode.
  *        @arg FLASH_ERASE_64KB: 64kbtye erase mode.
  *        @arg FLASH_ERASE_CHIP: erase whole flash chip.
  * @param addr: the address of memory.
  * @param blk_cnt: erase number of block or sector, no use in FLASH_ERASE_CHIP.
  * @retval HAL_Status: The status of driver
  */
HAL_Status rom_HAL_Flash_Erase(uint32_t flash, FlashEraseMode blk_size, uint32_t addr, uint32_t blk_cnt)
{
	struct FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;
	uint32_t esize = blk_size;
	uint32_t eaddr = addr;

	FD_DEBUG("%u: e%u * %u, a: 0x%x", flash, (uint32_t)blk_size, blk_cnt, addr);

	if ((addr + blk_size * blk_cnt) > dev->chip->cfg.mSize) {
		FD_ERROR("memory is over flash memory\n");
		return HAL_INVALID;
	}
	if ((blk_size == FLASH_ERASE_CHIP) && (blk_cnt != 1)) {
		FD_ERROR("execute more than 1");
		return HAL_INVALID;
	}
	if (addr % blk_size) {
		FD_ERROR("on a incompatible address");
		return HAL_INVALID;
	}

	while (blk_cnt-- > 0)
	{
		dev->drv->open(dev->chip);

		dev->chip->writeEnable(dev->chip);
		ret = dev->chip->erase(dev->chip, blk_size, eaddr);
		dev->chip->writeDisable(dev->chip);

		if (ret < 0)
			FD_ERROR("failed: %d", ret);

		ret = HAL_Flash_WaitCompl(dev, 5000);

		dev->drv->close(dev->chip);

		if (ret < 0)
			break;
		eaddr += esize;
	}

	return ret;
}

/**
  * @brief Calculate which block the flash address belong to, and output a
  *        block address to user.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number.
  * @param blk_size:
  *        @arg FLASH_ERASE_4KB: 4kbyte erase mode.
  *        @arg FLASH_ERASE_32KB: 32kbtye erase mode.
  *        @arg FLASH_ERASE_64KB: 64kbtye erase mode.
  *        @arg FLASH_ERASE_CHIP: erase whole flash chip.
  * @param addr: the address of memory.
  * @param start: the address of the block contained the addr.
  * @retval HAL_Status: The status of driver
  */
HAL_Status rom_HAL_Flash_MemoryOf(uint32_t flash, FlashEraseMode size, uint32_t addr, uint32_t *start)
{
	struct FlashDev *dev = getFlashDev(flash);
	uint32_t page_mask;
	HAL_Status ret = HAL_OK;

	if (!(size & dev->chip->cfg.mEraseSizeSupport))
		return HAL_INVALID;

	page_mask = ~((uint32_t)(size - 1));
	*start = addr & page_mask;

	return ret;
}

/**
  * @brief Check the flash memory whether .
  * @note The flash device configuration is in the board_config g_flash_cfg.
  *       Device number is the g_flash_cfg vector sequency number.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number.
  * @param addr: the address of memory.
  * @param data: the data needed to write to flash device.
  * @param size: the data size needed to write.
  * @retval int: 0: same as data, no need to write or erase;
  *              1: write directly, no need to erase;
  *              2: need to erase first;
  */
int HAL_Flash_Check(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size)
{
#define FLASH_CHECK_BUF_SIZE (128)

	uint8_t *pdata = data;
	uint8_t *pbuf;
	uint8_t *buf;
	uint8_t src;
	uint8_t dst;
	uint32_t left = size;
	uint32_t paddr = addr;
	int32_t ret = 0;

	buf = HAL_Malloc(FLASH_CHECK_BUF_SIZE);
	if (buf == NULL)
		return -1;
	pbuf = buf + FLASH_CHECK_BUF_SIZE;

	while (left > 0)
	{
		if ((pbuf - buf) == FLASH_CHECK_BUF_SIZE) {
			HAL_Flash_Read(flash, paddr, buf, FLASH_CHECK_BUF_SIZE);
			pbuf = buf;
		}

		src = *pbuf++;
		dst = *pdata++;
		left--;
		paddr++;

		dst ^= src;
		if (dst == 0)
			continue; /* src == dst */

		ret = 1; /* src != dst */
		if (dst & src) {
			ret = 2; /* src has bit '1', need to erase */
			break;
		}
	}

	HAL_Free(buf);

	return ret;
}


#ifdef CONFIG_PM
//#define FLASH_POWERDOWN (PM_MODE_POWEROFF)

static int PM_FlashSuspend(struct soc_device *dev, enum suspend_state_t state)
{
	/*
		suspend condition:
			(1)
			(2)
	*/
	struct FlashDev *fdev = getFlashDev((uint32_t)dev->platform_data);
	struct FlashDrv *drv = fdev->drv;
	struct FlashChip *chip = fdev->chip;

	if (fdev->usercnt != 0)
		return -1;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
		/* flash chip do not power down */
		break;
	case PM_MODE_HIBERNATION:
	case PM_MODE_POWEROFF:
		//HAL_Flash_Deinit((uint32_t)dev->platform_data);
		drv->open(chip);
		chip->reset(chip);
		chip->control(chip, DEFAULT_FLASH_POWERDOWN, NULL);
		drv->close(chip);
		break;
	default:
		break;
	}

	return 0;
}

static int PM_FlashResume(struct soc_device *dev, enum suspend_state_t state)
{
	struct FlashDev *fdev = getFlashDev((uint32_t)dev->platform_data);
	struct FlashDrv *drv = fdev->drv;
	struct FlashChip *chip = fdev->chip;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
		break;
	case PM_MODE_HIBERNATION:
	case PM_MODE_POWEROFF:
		//HAL_Flash_Init((uint32_t)dev->platform_data);
		drv->open(chip);
		if (fdev->rmode & (FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE | FLASH_READ_QPI_MODE))
		{
			chip->switchReadMode(chip, fdev->rmode);
			if (fdev->rmode & FLASH_READ_QPI_MODE)
				chip->enableQPIMode(chip);
		}
		drv->close(chip);
		break;
	default:
		break;
	}

	return 0;
}

static struct soc_device_driver flash_drv = {
	.name = "Flash",
	.suspend = PM_FlashSuspend,
	.resume = PM_FlashResume,
};

#endif /* CONFIG_PM */
