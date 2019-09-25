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

#include "pm/pm.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_flashctrl.h"
#include "sys/param.h"

#include "hal_base.h"

#if 0
#define FD_DEBUG(msg, arg...) XR_DEBUG((DBG_OFF | XR_LEVEL_ALL), NOEXPAND, "[Flash DRV DBG] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#define FD_ERROR(msg, arg...) XR_ERROR((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[Flash DRV ERR] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#define FD_INFO(msg, arg...) XR_DEBUG((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[Flash DRV INF] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#endif
#if 1
#define FD_DEBUG(msg, arg...) XR_DEBUG((DBG_OFF | XR_LEVEL_ALL), NOEXPAND, "[FD DBG] %s:" msg "\n", __func__, ##arg)
#define FD_ERROR(msg, arg...) XR_ERROR((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[FD ERR] %s:" msg "\n", __func__, ##arg)
#define FD_INFO(msg, arg...)  XR_INFO((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[FD INF] %s:" msg "\n", __func__, ##arg)
#endif

/**
  * @brief Flash ioctl function.
  * @note attr : arg
  *       others are not support for now.
  * @param flash: the flash device number, same as the g_flash_cfg vector
  *               sequency number
  * @param attr: ioctl command
  * @param arg: ioctl arguement
  * @retval HAL_Status: The status of driver
  */
HAL_Status HAL_Flash_Ioctl(uint32_t flash, FlashControlCmd attr, uint32_t arg)
{
	HAL_Status ret = HAL_ERROR;
	struct FlashDev *dev = getFlashDev(flash);

	switch (attr) {
		/*TODO: 1.return min erase size */
	case FLASH_GET_MIN_ERASE_SIZE: {
		*((FlashEraseMode *)arg) = dev->chip->minEraseSize(dev->chip);
		ret = HAL_OK;
		break;
	}
	case FLASH_WRITE_STATUS: {
		FlashControlStatus *tmp = (FlashControlStatus *)arg;
		dev->drv->open(dev->chip);
		dev->chip->writeEnable(dev->chip);
		ret = dev->chip->writeStatus(dev->chip, tmp->status, tmp->data);
		HAL_Flash_WaitCompl(dev, 5000);
		dev->chip->writeDisable(dev->chip);
		dev->drv->close(dev->chip);
		break;
	}
	case FLASH_READ_STATUS: {
		FlashControlStatus *tmp = (FlashControlStatus *)arg;
		dev->drv->open(dev->chip);
		ret = dev->chip->readStatus(dev->chip, tmp->status, tmp->data);
		HAL_Flash_WaitCompl(dev, 5000);
		dev->drv->close(dev->chip);
		break;
	}
	case FLASH_SET_READ_MODE: {
		FlashReadMode old_rmode = dev->rmode;

		dev->rmode = (FlashReadMode)arg;
		if(!(dev->rmode & dev->chip->cfg.mReadSupport)) {
			FD_ERROR("not support read mode %d\n", dev->rmode);
			return HAL_INVALID;
		}

		if(old_rmode == FLASH_READ_QPI_MODE) {
			dev->drv->open(dev->chip);
			ret = dev->chip->disableQPIMode(dev->chip);
			if(ret != HAL_OK)
				return ret;
			HAL_Flash_WaitCompl(dev, 5000);//wait busy
			dev->drv->close(dev->chip);
		}

		dev->drv->open(dev->chip);
		ret = dev->chip->switchReadMode(dev->chip, dev->rmode);

		if(ret != HAL_OK)
			return ret;

		HAL_Flash_WaitCompl(dev, 5000);//wait busy

		if (dev->rmode & FLASH_READ_QPI_MODE)
			ret = dev->chip->enableQPIMode(dev->chip);

		dev->drv->close(dev->chip);
		break;
	}
	case FLASH_SET_PAGEPROGRAM_MODE: {
		dev->wmode = (FlashPageProgramMode)arg;
		if(!(dev->wmode & dev->chip->cfg.mPageProgramSupport)) {
			FD_ERROR("not support page program mode %d\n", dev->wmode);
			return HAL_INVALID;
		}
		ret = HAL_OK;
		break;
	}

	/*TODO: tbc...*/
	default:
		return HAL_INVALID;
	}

	return ret;
}

#ifdef __CONFIG_ROM
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
HAL_Status HAL_Flash_Write(uint32_t flash, uint32_t addr, const uint8_t *data, uint32_t size)
{
	struct FlashDev *dev = getFlashDev(flash);
	HAL_Status ret = HAL_ERROR;
	uint32_t address = addr;
	uint32_t left = size;
	const uint8_t *ptr = data;
	uint32_t pp_size;

	FD_DEBUG("%u: w%u, a: 0x%x", flash, size, addr);

	if ((NULL == dev) || (NULL == dev->chip->pageProgram) || (0 == size) || (addr+size > dev->chip->cfg.mSize)) {
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
HAL_Status HAL_Flash_Read(uint32_t flash, uint32_t addr, uint8_t *data, uint32_t size)
{
	struct FlashDev *dev = getFlashDev(flash);
	HAL_Status ret;

	FD_DEBUG("%u: r%u, a: 0x%x", flash, size, addr);

	if ((NULL == dev) || (NULL == dev->chip->read) || (size == 0) || (addr+size > dev->chip->cfg.mSize)) {
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

extern HAL_Status __HAL_Flash_Init(uint32_t flash);

#ifdef CONFIG_PM
static int PM_FlashSuspend(struct soc_device *dev, enum suspend_state_t state)
{
    struct FlashDev *fdev;
    struct FlashDrv *drv;
    struct FlashChip *chip;
	switch (state) {
	case PM_MODE_SLEEP:
	case PM_MODE_STANDBY:
		break;
	case PM_MODE_HIBERNATION:
        fdev = getFlashDev((uint32_t)dev->platform_data);
        drv = fdev->drv;
        chip = fdev->chip;
        if (fdev->usercnt != 0)
            return -1;
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
    struct FlashDev *fdev;
    struct FlashDrv *drv;
    struct FlashChip *chip;
	switch (state) {
	case PM_MODE_SLEEP:
	case PM_MODE_STANDBY:
		break;
	case PM_MODE_HIBERNATION:
        fdev = getFlashDev((uint32_t)dev->platform_data);
        drv = fdev->drv;
        chip = fdev->chip;
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
    .suspend_noirq = PM_FlashSuspend,
    .resume_noirq = PM_FlashResume,
};
#endif
HAL_Status HAL_Flash_Init(uint32_t flash)
{
    HAL_Status ret;
    ret = __HAL_Flash_Init(flash);
    if(ret != HAL_OK)
        return ret;
#ifdef CONFIG_PM
    struct FlashDev *dev = getFlashDev(flash);
    pm_unregister_ops(dev->pm);
    dev->pm->driver = &flash_drv;
    pm_register_ops(dev->pm);
#endif/*CONFIG_PM*/
    return HAL_OK;
}

#endif

#if FLASH_SPI_ENABLE // check when ROM code ###############!!!!!

/*
	struct SpiFlashDrv
*/
struct SpiFlashDrv {
	SPI_Port port;
	SPI_CS cs;
	SPI_Config config;
};

static struct SpiFlashDrv *spi_drv;

HAL_Status spiFlashOpen(struct FlashChip *chip)
{
	/*TODO: it should be suspend schedule and deinit xip when spi is using spi0 with xiping*/

	return HAL_OK;
}

HAL_Status spiFlashClose(struct FlashChip *chip)
{
	/*TODO: it should be resume schedule and init xip when spi is using spi0 with xiping*/

	return HAL_OK;
}

#define FD_SPI_WRITE(impl, ins, line_max)	\
	do {	\
		ret = insToSpi(impl, ins, line_max, HAL_SPI_Transmit);	\
		if (ret != HAL_OK) { \
			FD_ERROR("spi instruction param error"); \
			goto failed; \
		} \
	} while (0)

#define FD_SPI_READ(impl, ins, line_max)	\
	do { \
		ret = insToSpi(impl, ins, line_max, HAL_SPI_Receive);	\
		if (ret != HAL_OK) { \
			FD_ERROR("spi instruction param error"); \
			goto failed; \
		} \
	} while (0)

static HAL_Status insToSpi(struct SpiFlashDrv *impl, InstructionField *ins, uint32_t line_max, HAL_Status (*fun)(SPI_Port, uint8_t *, uint32_t))
{
	uint8_t *p;

	if (ins) {
		if (ins->line > line_max)
			return HAL_INVALID;
		if (!ins->pdata && ins->len > 4)
			return HAL_ERROR;

		p = ins->pdata ? ins->pdata : (uint8_t *)&ins->data;
		return fun(impl->port, p, ins->len);
	} else
		return HAL_OK;
}

static HAL_Status spiFlashWrite(struct FlashChip *dev, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	struct FlashDrv *drv = dev->mDriver;
	//struct SpiFlashDrv *spi_drv = drv->platform_data;
	InstructionField naddr = {0};
	HAL_Status ret;

	if (data && data->len >= (drv->sizeToDma - 1))
		spi_drv->config.opMode = SPI_OPERATION_MODE_DMA;
	else
		spi_drv->config.opMode = SPI_OPERATION_MODE_POLL;

	ret = HAL_SPI_Open(spi_drv->port, spi_drv->cs, &spi_drv->config, 5000);
	if (ret != HAL_OK) {
		FD_ERROR("spi open failed");
		return ret;
	}

	HAL_SPI_CS(spi_drv->port, 1);
	FD_SPI_WRITE(spi_drv, cmd, 1);
	if (addr) {
		naddr.len = addr->len;
		naddr.line = addr->line;
		if (!addr->pdata)
			addr->pdata = (uint8_t *)&addr->data;
		naddr.data = ((addr->pdata)[2]) | ((addr->pdata)[1] << 8) | ((addr->pdata)[0] << 16);
		FD_DEBUG("naddr.data: 0x%x", naddr.data);
		FD_SPI_WRITE(spi_drv, &naddr, 1);
	}
	FD_SPI_WRITE(spi_drv, dummy, 1);
	FD_SPI_WRITE(spi_drv, data, 1);
	HAL_SPI_CS(spi_drv->port, 0);

failed:
	HAL_SPI_Close(spi_drv->port);
	return ret;
}

static HAL_Status spiFlashRead(struct FlashChip *dev, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	struct FlashDrv *drv = dev->mDriver;
	//struct SpiFlashDrv *spi_drv = drv->platform_data;
	InstructionField naddr = {0};
	HAL_Status ret;

	if (data && data->len >= (drv->sizeToDma - 1))
		spi_drv->config.opMode = SPI_OPERATION_MODE_DMA;
	else
		spi_drv->config.opMode = SPI_OPERATION_MODE_POLL;

	ret = HAL_SPI_Open(spi_drv->port, spi_drv->cs, &spi_drv->config, 5000);
	if (ret != HAL_OK) {
		FD_ERROR("spi open failed");
		return ret;
	}

	if (data && data->line == 2)
		HAL_SPI_Config(spi_drv->port, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_DUAL_RX);
	else
		HAL_SPI_Config(spi_drv->port, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_NORMAL);

	HAL_SPI_CS(spi_drv->port, 1);
	FD_SPI_WRITE(spi_drv, cmd, 1);
	if (addr) {
		naddr.len = addr->len;
		naddr.line = addr->line;
		if (!addr->pdata)
			addr->pdata = (uint8_t *)&addr->data;
		naddr.data = ((addr->pdata)[2]) | ((addr->pdata)[1] << 8) | ((addr->pdata)[0] << 16);
		FD_DEBUG("naddr.data: 0x%x", naddr.data);
		FD_SPI_WRITE(spi_drv, &naddr, 1);
	}
	FD_SPI_WRITE(spi_drv, dummy, 1);
	FD_SPI_READ(spi_drv, data, 2);
	HAL_SPI_CS(spi_drv->port, 0);

failed:
	HAL_SPI_Close(spi_drv->port);
	return ret;
}

static HAL_Status spiFlashSetFreq(struct FlashChip *dev, uint32_t freq)
{
	/* TODO: tbc... */
	return HAL_INVALID;
}

void spiFlashMsleep(struct FlashChip *dev, uint32_t ms)
{
	HAL_MSleep(ms);
}

void spiFlashMsleepReuseFlashc(struct FlashChip *dev, uint32_t ms)
{
	HAL_Flashc_Delay(dev->flash_ctrl, ms * 1000);
}

static void spiFlashDestroy(struct FlashChip *dev)   //to check
{
	//struct SpiFlashDrv *spi_drv = dev->mDriver->platform_data;
	HAL_Free(spi_drv);
	HAL_Free(dev->mDriver);
}

struct FlashDrv *spiDriverCreate(int dev, FlashBoardCfg *bcfg)
{
#if FLASH_SPI_ENABLE
	/*TODO: check read mode*/
	struct FlashDrv *drv;
	//struct SpiFlashDrv *spi_drv;

	drv = HAL_Malloc(sizeof(struct FlashDrv));
	if (!drv) {
		FD_ERROR("malloc fail\n");
		return NULL;
	}
	HAL_Memset(drv, 0, sizeof(struct FlashDrv));
	spi_drv = HAL_Malloc(sizeof(struct SpiFlashDrv));
	if (!spi_drv) {
		FD_ERROR("malloc fail\n");
		HAL_Free(drv);
		return NULL;
	}
	HAL_Memset(spi_drv, 0, sizeof(struct SpiFlashDrv));
	drv->platform_data = spi_drv;

	spi_drv->port = bcfg->spi.port;
	spi_drv->cs = bcfg->spi.cs;
	spi_drv->config.sclk = bcfg->spi.clk;
	spi_drv->config.firstBit = SPI_TCTRL_FBS_MSB;
	spi_drv->config.mode = SPI_CTRL_MODE_MASTER;
	spi_drv->config.opMode = SPI_OPERATION_MODE_DMA;	/*spi0 must be poll on xip;*/
	spi_drv->config.sclkMode = SPI_SCLK_Mode0;

	//FD_DEBUG("type: %d; port: %d; cs: %d; sclk: %d;",
	//	bcfg->type, spi_drv->port, spi_drv->cs, spi_drv->config.sclk);

	drv->dev = dev;
	drv->open = spiFlashOpen;
	drv->close = spiFlashClose;
	drv->read = spiFlashRead;
	drv->write = spiFlashWrite;
	drv->setFreq = spiFlashSetFreq;
	drv->destroy = spiFlashDestroy;
	drv->sizeToDma = FLASH_DMA_TRANSFER_MIN_SIZE;
	if (bcfg->spi.port == SPI1)
		drv->msleep = spiFlashMsleep;
	else
		drv->msleep = spiFlashMsleepReuseFlashc;

	return drv;
#endif
}

#endif /* FLASH_SPI_ENABLE */
