/**
  * @file  hal_wdg.c
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

#include "rom/libc/sys/time.h" /* for timeofday */
#include "rom/pm/pm.h"
#include "rom/driver/chip/hal_wdg.h"
#include "rom/driver/chip/hal_prcm.h"

#include "hal_base.h"

/* Note: Has no 32000HZ clock source, use WDG_CLK_32768HZ by default  */
#define WDG_CLK_SRC_SHIFT	8
#define WDG_CLK_SRC_MASK	(0x1U << WDG_CLK_SRC_SHIFT)
typedef enum {
	WDG_CLK_32000HZ = (0U << WDG_CLK_SRC_SHIFT),
	WDG_CLK_32768HZ = (1U << WDG_CLK_SRC_SHIFT)
} WDG_ClkSrc;

#define WDG_RELOAD_VAL	((0xA57 << 1) | 1)

#define WDG_TIMEOFDAY_SAVE() timeofday_save()

#if HAL_WDG_INTERRUPT_SUPPORT
typedef struct {
	WDG_IRQCallback callback;
	void *arg;
} WDG_Private;

static WDG_Private gWdgPrivate;

static void WDG_EnableIRQ(void)
{
	HAL_SET_BIT(WDG->IRQ_EN, WDG_IRQ_EN_BIT);
}

static void WDG_DisableIRQ(void)
{
	HAL_CLR_BIT(WDG->IRQ_EN, WDG_IRQ_EN_BIT);
}

__rom_sram_text
static __inline int WDG_IsPendingIRQ(void)
{
	return HAL_GET_BIT(WDG->IRQ_STATUS, WDG_IRQ_PENDING_BIT);
}

__rom_sram_text
static void WDG_ClearPendingIRQ(void)
{
	HAL_SET_BIT(WDG->IRQ_STATUS, WDG_IRQ_PENDING_BIT);
}

__rom_sram_text
void WDG_IRQHandler(void)
{
	if (WDG_IsPendingIRQ()) {
		WDG_ClearPendingIRQ();
		if (gWdgPrivate.callback) {
			gWdgPrivate.callback(gWdgPrivate.arg);
		}
	}
}
#endif /* HAL_WDG_INTERRUPT_SUPPORT */

static void WDG_HwInit(const WDG_HwInitParam *hw)
{
	WDG->MODE = hw->timeout; /* NB: it will clear WDG_IRQ_EN_BIT (stop WDG) */
#if (defined(__CONFIG_CHIP_XR871))
	WDG->CFG = hw->event | WDG_CLK_32768HZ;
#elif (defined(__CONFIG_CHIP_XR875))
	WDG->CFG = hw->event | hw->resetCpuMode | WDG_CLK_32768HZ;
#endif
	WDG->RESET_CTRL = ((hw->resetCycle << WDG_RESET_CYCLE_SHIFT) & WDG_RESET_CYCLE_MASK);

#if HAL_WDG_INTERRUPT_SUPPORT
	if (hw->event == WDG_EVT_INTERRUPT) {
		if (WDG_IsPendingIRQ()) {
			WDG_ClearPendingIRQ();
		}
		WDG_EnableIRQ();
		HAL_NVIC_ConfigExtIRQ(WDG_IRQn, WDG_IRQHandler, NVIC_PERIPH_PRIO_DEFAULT);
	}
#endif
}

static void WDG_HwDeInit(void)
{
	HAL_WDG_Stop();

#if HAL_WDG_INTERRUPT_SUPPORT
	HAL_NVIC_DisableIRQ(WDG_IRQn);
	WDG_DisableIRQ();
	if (WDG_IsPendingIRQ()) {
		WDG_ClearPendingIRQ();
	}
#endif
}

#ifdef CONFIG_PM

typedef struct {
	WDG_HwInitParam hw;
	int8_t isStart;
} WDG_PmPrivate;

static WDG_PmPrivate gWdgPmPrivate;

static int wdg_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	gWdgPmPrivate.isStart = !!HAL_GET_BIT(WDG->MODE, WDG_EN_BIT);
	WDG_HwDeInit();
	return 0;
}

static int wdg_resume(struct soc_device *dev, enum suspend_state_t state)
{
	WDG_HwInit(&gWdgPmPrivate.hw);
	if (gWdgPmPrivate.isStart) {
		HAL_WDG_Start();
	}
	return 0;
}

static const struct soc_device_driver wdg_drv = {
	.name = "wdg",
	.suspend = wdg_suspend,
	.resume = wdg_resume,
};

static struct soc_device wdg_dev = {
	.name = "wdg",
	.driver = &wdg_drv,
};

#define WDG_DEV (&wdg_dev)

#endif /* CONFIG_PM */

/**
 * @brief Initialize the watchdog according to the specified parameters
 * @param[in] param Pointer to WDG_InitParam structure
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_WDG_Init(const WDG_InitParam *param)
{
#if HAL_WDG_INTERRUPT_SUPPORT
	if (param->hw.event == WDG_EVT_INTERRUPT) {
		gWdgPrivate.callback = param->callback;
		gWdgPrivate.arg = param->arg;
	}
#endif
	WDG_HwInit(&param->hw);

#ifdef CONFIG_PM
	HAL_Memcpy(&gWdgPmPrivate.hw, &param->hw, sizeof(WDG_HwInitParam));
	pm_register_ops(WDG_DEV);
#endif

	return HAL_OK;
}

/**
 * @brief DeInitialize the watchdog
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_WDG_DeInit(void)
{
#ifdef CONFIG_PM
	pm_unregister_ops(WDG_DEV);
#endif

	WDG_HwDeInit();
#if HAL_WDG_INTERRUPT_SUPPORT
	gWdgPrivate.callback = NULL;
	gWdgPrivate.arg = NULL;
#endif
	return HAL_OK;
}

/**
 * @brief Feed the watchdog
 * @note When watchdog running, reset system or IRQ event will be triggered if
 *       no feeding executed in the interval configured by HAL_WDG_Init().
 * @return None
 */
__rom_sram_text
void HAL_WDG_Feed(void)
{
	WDG->CTRL = WDG_RELOAD_VAL;
}

/**
 * @brief Start the watchdog
 * @return None
 */
void HAL_WDG_Start(void)
{
	HAL_WDG_Feed();
	HAL_SET_BIT(WDG->MODE, WDG_EN_BIT);
}

/**
 * @brief Stop the watchdog
 * @return None
 */
void HAL_WDG_Stop(void)
{
	HAL_CLR_BIT(WDG->MODE, WDG_EN_BIT);
}

/**
 * @brief Reboot system using the watchdog
 * @return None
 */
void HAL_WDG_Reboot(void)
{
	HAL_DisableIRQ();
	HAL_WDG_Stop();
#if (defined(__CONFIG_CHIP_XR871) && !defined(__CONFIG_CHIP_XR875_ON_XR871))
	HAL_PRCM_DisableSys2();
	HAL_PRCM_DisableSys2Power();
#endif
	WDG_TIMEOFDAY_SAVE();
	WDG->CFG = WDG_EVT_RESET | WDG_CLK_32768HZ;
	WDG->RESET_CTRL = ((WDG_DEFAULT_RESET_CYCLE << WDG_RESET_CYCLE_SHIFT) &
	                   WDG_RESET_CYCLE_MASK);
#if defined(__CONFIG_CHIP_XR871)
	WDG->MODE = WDG_TIMEOUT_500MS | WDG_EN_BIT;
#elif defined(__CONFIG_CHIP_XR875)
	WDG->MODE = WDG_RESET_IMMED_VAL | WDG_EN_BIT;
#endif
	while (1) ;
}

#ifdef __CONFIG_CHIP_XR875
/**
 * @brief Reset cpu using the watchdog
 * @return None
 */
void HAL_WDG_ResetCpu(WDG_ResetCpuMode mode)
{
	HAL_DisableIRQ();
	HAL_WDG_Stop();
	WDG_TIMEOFDAY_SAVE();
	WDG->CFG = WDG_EVT_RESET_CPU | mode | WDG_CLK_32768HZ;
	WDG->RESET_CTRL = ((WDG_DEFAULT_RESET_CYCLE << WDG_RESET_CYCLE_SHIFT) &
	                   WDG_RESET_CYCLE_MASK);
	WDG->MODE = WDG_RESET_IMMED_VAL | WDG_EN_BIT;
	while (1) ;
}
#endif
