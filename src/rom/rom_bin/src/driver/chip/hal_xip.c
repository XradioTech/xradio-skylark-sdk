/**
  * @file  hal_xip.c
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

#include "rom/driver/chip/hal_flashctrl.h"
#include "rom/driver/chip/hal_flash.h"
#include "rom/driver/chip/hal_xip.h"
#include "rom/sys/param.h"

#include "rom/pm/pm.h"

#include "hal_base.h"
#include "flashchip/flash_debug.h"

struct FlashChip *getFlashChip(struct FlashDev *dev);
FlashReadMode getFlashMode(struct FlashDev *dev);
//struct FlashDev *getFlashDev(uint32_t flash);
void insToFcIns(InstructionField *ins, FC_InstructionField *fcins);

struct XipDrv
{
	XIP_Config mCfg;
	uint32_t flash;
	struct FlashDev *dev;
#ifdef CONFIG_PM
	struct soc_device_driver xip_drv;
	struct soc_device xip_dev;
#endif
	struct flash_controller *flash_ctrl;

};

uint8_t xip_debug_mask = XR_LEVEL_INFO;

void HAL_Xip_SetDbgMask(uint8_t dbg_mask)
{
	xip_debug_mask = dbg_mask;
}

int HAL_Xip_setCmd(struct XipDrv *xip, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data)
{
	FC_InstructionField tmp;
	XIP_Instruction *ins = &xip->mCfg.ins;

	insToFcIns(cmd, &tmp);
	ins->cmd = *tmp.pdata;
	ins->cmd_line = tmp.line;

	insToFcIns(addr, &tmp);
	ins->addr_line = tmp.line;

	insToFcIns(dummy, &tmp);
	ins->dummy_line = tmp.line;
	ins->dum_btyes = tmp.len;

	insToFcIns(data, &tmp);
	ins->data_line = tmp.line;

	return 0;
}

int HAL_Xip_setContinue(struct XipDrv *xip, uint32_t continueMode, void *arg)
{
	xip->mCfg.cont_mode = continueMode;

	return 0;
}

int HAL_Xip_setDelay(struct XipDrv *xip, Flashc_Delay *delay)
{
	/*TODO: how and where should be the delay calculation*/

	return 0;
}

int HAL_Xip_setAddr(struct XipDrv *xip, uint32_t addr)
{
	xip->mCfg.addr = addr;

	return 0;
}

#ifdef CONFIG_PM
//#define FLASH_POWERDOWN (PM_MODE_POWEROFF)

static int PM_XipSuspend(struct soc_device *dev, enum suspend_state_t state)
{
	struct XipDrv *xip = dev->platform_data;
	struct FlashChip *chip = getFlashChip(xip->dev);

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
	case PM_MODE_POWEROFF:
		chip->disableXIP(chip);
		break;
	default:
		break;
	}

	return 0;
}

static int PM_XipResume(struct soc_device *dev, enum suspend_state_t state)
{
	struct XipDrv *xip = dev->platform_data;
	struct FlashChip *chip = getFlashChip(xip->dev);

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		chip->enableXIP(chip);
		break;
	default:
		break;
	}

	return 0;
}
#endif

/**
  * @brief Initializes XIP module.
  * @note XIP is a module that cpu can run the code in flash but not ram.
  * @param flash: flash number, this flash must have been initialized, and must
  *               be connected to the flash controller pin.
  * @param xaddr: XIP code start address.
  * @retval HAL_Status: The status of driver
  */
HAL_Status HAL_Xip_Init(uint32_t flash, uint32_t xaddr)
{
	HAL_Status ret = HAL_OK;
	struct FlashDev *dev;
	struct FlashChip *chip;
	FlashBoardCfg *cfg;
	int devNum = HAL_MKDEV(HAL_DEV_MAJOR_FLASH, flash);
	struct XipDrv *xip;

	xip = HAL_Malloc(sizeof(struct XipDrv));
	if (xip == NULL) {
		XIP_ERROR("no mem\n");
		return HAL_INVALID;
	} else {
		HAL_Memset(xip, 0, sizeof(struct XipDrv));
		XIP_DEBUG("xip:%p\n", xip);
	}

	HAL_BoardIoctl(HAL_BIR_GET_CFG, devNum, (uint32_t)&cfg);
	if (cfg->type != FLASH_DRV_FLASHC) {
		HAL_Free(xip);
		return HAL_INVALID;
	}

	dev = getFlashDev(flash);
	if (dev == NULL) {
		HAL_Free(xip);
		return HAL_INVALID;
	}

	xip->flash = flash;
	xip->dev = dev;
	HAL_Memset(&xip->mCfg, 0, sizeof(xip->mCfg));

	chip = getFlashChip(dev);

	chip->mXip = xip;

	chip->enableXIP(chip);
	chip->xipDriverCfg(chip, getFlashMode(dev));
	xip->mCfg.freq = cfg->flashc.clk;
	xip->mCfg.addr = xaddr;

	xip->flash_ctrl = chip->flash_ctrl;
	HAL_Flashc_Xip_Init(xip->flash_ctrl, &xip->mCfg);

#ifdef CONFIG_PM
	xip->xip_drv.name = "Xip";
	xip->xip_drv.suspend = PM_XipSuspend;
	xip->xip_drv.resume = PM_XipResume;
	xip->xip_dev.name = "Xip";
	xip->xip_dev.driver = &xip->xip_drv;
	xip->xip_dev.platform_data = xip;
	pm_register_ops(&xip->xip_dev);
#endif

	return ret;
}

/**
  * @brief Deinitializes XIP module.
  * @retval HAL_Status: The status of driver
  */
HAL_Status HAL_Xip_Deinit(uint32_t flash)
{
	HAL_Status ret;
	struct FlashDev *dev;
	struct FlashChip *chip;
	struct XipDrv *xip;

	dev = getFlashDev(flash);
	if (dev == NULL) {
		return HAL_INVALID;
	}

	chip = getFlashChip(dev);
	xip = chip->mXip;
	if (xip == NULL) {
		return HAL_INVALID;
	}

#ifdef CONFIG_PM
	pm_unregister_ops(&xip->xip_dev);
	xip->xip_dev.platform_data = NULL;
	HAL_Memset(&xip->xip_dev, 0, sizeof(struct soc_device));
	HAL_Memset(&xip->xip_drv, 0, sizeof(struct soc_device_driver));
#endif

	ret = HAL_Flashc_Xip_Deinit(xip->flash_ctrl);
	chip->disableXIP(chip);

	XIP_DEBUG("xip:%p\n", xip);
	HAL_Free(xip);

	return ret;
}
