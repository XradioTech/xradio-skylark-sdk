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
#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_xip.h"
#include "sys/param.h"
#include "pm/pm.h"

#include "hal_base.h"

#ifdef __CONFIG_ROM

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

HAL_Status __HAL_Xip_Init(uint32_t flash, uint32_t xaddr);
int PM_XipSuspend(struct soc_device *dev, enum suspend_state_t state);
int PM_XipResume(struct soc_device *dev, enum suspend_state_t state);

HAL_Status HAL_Xip_Init(uint32_t flash, uint32_t xaddr)
{
    HAL_Status ret;
    ret = __HAL_Xip_Init(flash, xaddr);
    if(ret != HAL_OK)
        return ret;

#ifdef CONFIG_PM
	struct FlashDev *dev;
    struct FlashChip *chip;
	struct XipDrv *xip;

    dev = getFlashDev(flash);
	if (dev == NULL) {
		return HAL_INVALID;
	}
    chip = getFlashChip(dev);
	if ((chip == NULL) || (chip->mXip == NULL)) {
		return HAL_INVALID;
	}
	xip = chip->mXip;
    pm_unregister_ops(&xip->xip_dev);
    xip->xip_drv.suspend = NULL;
	xip->xip_drv.resume = NULL;
    xip->xip_drv.suspend_noirq= PM_XipSuspend;
	xip->xip_drv.resume_noirq = PM_XipResume;
    pm_register_ops(&xip->xip_dev);
#endif/*CONFIG_PM*/

    return HAL_OK;
}

#endif/*__CONFIG_ROM*/

