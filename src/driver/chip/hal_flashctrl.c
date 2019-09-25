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
#include "hal_base.h"
#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/hal_icache.h"
#include "driver/chip/hal_dcache.h"
#include "sys/xr_debug.h"

#ifdef CONFIG_PM
#define FLASHC_TEMP_FIXED
#endif

#if 0
#define FC_DEBUG_ON (DBG_OFF)
#define FC_DEBUG(msg, arg...) XR_DEBUG((FC_DEBUG_ON | XR_LEVEL_ALL), NOEXPAND, "[FC DBG] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#define FC_ERROR(msg, arg...) XR_ERROR((DBG_ON | XR_LEVEL_ALL), NOEXPAND, "[FC ERR] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#endif

#if 1
#define FC_DEBUG_ON (DBG_ON)
#define FC_DEBUG(msg, arg...) XR_DEBUG((DBG_ON), NOEXPAND, "[FC DBG] %s:" msg "\n", __func__, ##arg)
#define FC_ERROR(msg, arg...) XR_ERROR((DBG_ON), NOEXPAND, "[FC ERR] %s:" msg "\n", __func__, ##arg)
#endif

#ifdef __CONFIG_ROM
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

void HAL_Flashc_Xip_RawDisable(struct flash_controller *ctrl)
{
	if (!ctrl->xip_on)
		return;

	/* if irq disable, do not suspend scheduler in case of system error */
	if (!HAL_IsIRQDisabled())
		HAL_ThreadSuspendScheduler();

//	HAL_UDelay(100);
//	FC_Ibus_Disable(FC_EN_IBUS | ctrl->xip_continue);
	while(FC_Sbus_GetDebugState() != 0x00);
#if FLASHCTRL_ARCH_V2
	HAL_CLR_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, ctrl->xip_continue);
#else
	FC_Ibus_Disable(ctrl->xip_continue);
#endif
}
#endif
/**
  * @internal
  * @brief Flash controller IBUS (XIP) Enable with Pin initialization.
  * @note Most for SPI. It will resume system schedule.
  * @param None
  * @retval None
  */
void HAL_Flashc_Xip_Enable(struct flash_controller *ctrl)
{
	/* open io */
	HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_FLASHC, 0), 0);
	HAL_Flashc_Xip_RawEnable(ctrl);
}


/**
  * @internal
  * @brief Flash controller IBUS (XIP) Enable with Pin deinitialization.
  * @note Most for SPI. It will suspend system schedule.
  * @param None
  * @retval None
  */
void HAL_Flashc_Xip_Disable(struct flash_controller *ctrl)
{
	HAL_Flashc_Xip_RawDisable(ctrl);

	//close io
	HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_FLASHC, 0), 0);
}

uint8_t FC_GetDataDelay(uint32_t freq)
{
#ifdef __CONFIG_PLATFORM_FPGA
	uint32_t apbs_clk = HAL_GetDevClock();
	if(apbs_clk == HOSC_CLOCK_24M) {
		return 1;
	} else if(apbs_clk == HOSC_CLOCK_48M) {
		return 2;
	} else if(apbs_clk > HOSC_CLOCK_48M) {
		return 3;
	} else {
		return 0;
	}
#else
	if (freq < 48000000)
		return 0;
	else if (freq <= 64000000)
		return 1;
	else
		return 2;

#endif
}

#ifdef __CONFIG_ROM

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

	if (!ctrl->xip_on) {
		HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_FLASH_CTRL);
		HAL_CCM_FLASHC_EnableMClock();
	}

	while(FC_Sbus_GetDebugState() != 0x00);

	if (!ctrl->xip_on) {
		HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_FLASH_CTRL);
		HAL_CCM_FLASHC_DisableMClock();
	}

	switch (state) {
	case PM_MODE_SLEEP:
		if (ctrl->xip_on) {
#if FLASHCTRL_ARCH_V2
			HAL_CLR_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, ctrl->xip_continue);
#else
			FC_Ibus_Disable(ctrl->xip_continue);
#endif
			HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_FLASH_CTRL);
			HAL_CCM_FLASHC_DisableMClock();
		}
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
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

	//FC_Reg_All();
	return 0;
}

int flashc_resume(struct soc_device *dev, enum suspend_state_t state)
{
	struct flash_controller *ctrl = dev->platform_data;

	//FC_Reg_All();

	switch (state) {
	case PM_MODE_SLEEP:
		if (ctrl->xip_on) {
			HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_FLASH_CTRL);
			HAL_CCM_FLASHC_EnableMClock();
#if FLASHCTRL_ARCH_V2
			HAL_SET_BIT(OPI_MEM_CTRL->MEM_COM_CONFG, ctrl->xip_continue);
#else
			FC_Ibus_Enable(ctrl->xip_continue);
#endif
		}
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
#endif/*CONFIG_PM*/

HAL_Status __HAL_Flashc_Xip_Init(struct flash_controller *ctrl, XIP_Config *cfg);

HAL_Status HAL_Flashc_Xip_Init(struct flash_controller *ctrl, XIP_Config *cfg)
{
    HAL_Status ret;
	Flash_Ctrl_DelayCycle delay = {1, 0, 8, 0, 0, 0, 1};

    ret = __HAL_Flashc_Xip_Init(ctrl, cfg);
	if (ret != HAL_OK) {
		FC_ERROR("flashc xip init failed\n");
		return ret;
	}

	HAL_Flashc_EnableCCMU(ctrl);
	delay.data = FC_GetDataDelay(cfg->freq);
	FC_Ibus_TransmitDelay(&delay);
	HAL_Flashc_DisableCCMU(ctrl);

	return ret;
}


/**
 * @brief Initialize Flash controller SBUS.
 * @param cfg:
 * 	   @arg cfg->freq: Flash working frequency.
 * @retval HAL_Status: The status of driver.
 */
HAL_Status __HAL_Flashc_Init(struct flash_controller *ctrl, const Flashc_Config *cfg);

HAL_Status HAL_Flashc_Init(struct flash_controller *ctrl, const Flashc_Config *cfg)
{
    HAL_Status ret;
	Flash_Ctrl_DelayCycle delay = {1, 0, 8, 0, 0, 0, 1};

    ret = __HAL_Flashc_Init(ctrl, cfg);
	if (ret != HAL_OK) {
		FC_ERROR("flashc init failed\n");
		return ret;
	}

	HAL_Flashc_EnableCCMU(ctrl);
	delay.data = FC_GetDataDelay(cfg->freq);
	FC_Sbus_TransmitDelay(&delay);
	HAL_Flashc_DisableCCMU(ctrl);
#if (defined CONFIG_PM) || (defined FLASHC_TEMP_FIXED)
	if (!ctrl->suspending) {
		ctrl->flashc_drv.suspend_noirq = flashc_suspend;
		ctrl->flashc_drv.resume_noirq = flashc_resume;
	}
#endif
	return ret;
}

HAL_Status HAL_Flashc_Open(struct flash_controller *ctrl)
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

#endif
