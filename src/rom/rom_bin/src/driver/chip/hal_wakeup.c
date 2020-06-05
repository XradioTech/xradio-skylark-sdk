/**
  * @file  hal_wakeup.c
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

#include "rom/sys/param.h"

#include "rom/driver/chip/hal_wakeup.h"
#include "rom/rom_debug.h"

#include "hal_base.h"

#define WK_LOGD_ON 1
#define WK_LOGN_ON 1
#define WK_LOGW_ON 1
#define WK_LOGE_ON 1
#define WK_LOGA_ON 1

#define WK_DBG(format, args...) ROM_DBG(wakeup_debug_mask, format, ##args)
#define WK_INF(format, args...) ROM_INF(wakeup_debug_mask, format, ##args)
#define WK_WAR(format, args...) ROM_WRN(wakeup_debug_mask, format, ##args)
#define WK_ERR(format, args...) ROM_ERR(wakeup_debug_mask, format, ##args)
#define WK_ANY(format, args...) ROM_ANY(wakeup_debug_mask, format, ##args)

#define WAKEUP_IO_MASK  ((1 << WAKEUP_IO_MAX) - 1)

#define WAKEUP_IRQn A_WAKEUP_IRQn
#define WAKEUP_GetTimerPending() HAL_PRCM_GetWakeupTimerPending()
#define WAKEUP_ClearTimerPending() HAL_PRCM_ClearWakeupTimerPending()
#define WAKEUP_GetTimerEnable() HAL_PRCM_GetWakeupTimerEnable()
#define WAKEUP_IRQ_SAVE arch_irq_save
#define WAKEUP_IRQ_RESTORE arch_irq_restore

static uint32_t wakeup_event;

static uint16_t wakeup_debug_mask = ROM_INF_MASK | ROM_WRN_MASK | ROM_ERR_MASK | ROM_ANY_MASK;

void HAL_Wakeup_SetDebugMask(uint16_t debug_mask)
{
	wakeup_debug_mask = debug_mask;
}

#ifdef __CONFIG_ARCH_APP_CORE
static uint32_t wakeup_io_en;
static uint32_t wakeup_io_mode;
#ifdef WAKEUP_IO_CONFIG_GPIO
static uint32_t wakeup_io_pull;
#endif
ct_assert(WAKEUP_IO_MAX <= DIV_ROUND_UP(32, GPIO_CTRL_PULL_BITS));

static uint32_t wakeup_time;

void Wakeup_ClrIO(uint32_t io_mask)
{
	HAL_PRCM_WakeupIODisableCfgHold(io_mask);
	HAL_PRCM_WakeupIODisableGlobal();
	HAL_PRCM_WakeupIOSetFallingEvent(io_mask);
	HAL_PRCM_WakeupIODisable(io_mask);

	HAL_PRCM_WakeupIOClearEventDetected(io_mask);
}
#endif

void Wakeup_DisTimer(void)
{
	HAL_PRCM_WakeupTimerDisable();
	while (HAL_PRCM_WakeupTimerGetCurrentValue())
		;
}

void Wakeup_ClrTimer(void)
{
	if (WAKEUP_GetTimerPending()) {
		WAKEUP_ClearTimerPending();
		while (WAKEUP_GetTimerPending())
			;
	}
}

void Wakeup_Source_Handler(void)
{
	Wakeup_ClrTimer();
	Wakeup_DisTimer();
#ifdef __CONFIG_ARCH_APP_CORE
	Wakeup_ClrIO(wakeup_io_en);
#endif

	HAL_NVIC_ClearPendingIRQ(WAKEUP_IRQn);

	WK_INF("appos wakeup clean\n");
}

int32_t Wakeup_SetTimer(uint32_t count_32k)
{
	Wakeup_DisTimer();
	HAL_PRCM_WakeupTimerSetCompareValue(count_32k);
	HAL_PRCM_WakeupTimerEnable();
	WK_INF("%s %u\n", __func__, count_32k);

	return 0;
}

/**
 * @brief Set wakeup timer.
 * @note This will config wakeup timer counting at final stages of suspend.
 *        Wakeup timer should be setted everytime if you want wake up system
 *        from suspend.
 * @param count_32k:
 *        @arg count_32k-> counter to wakeup system based on 32k counter, from
 *             WAKEUP_TIMER_MIN_TIME*32(WAKEUP_TIMER_MIN_TIME mS) to
 *             2147483647(671088S, about 186.4h).
 * retval  0 if success or other if failed.
 */
int32_t HAL_Wakeup_SetTimer(uint32_t count_32k)
{
	if ((count_32k < (32*WAKEUP_TIMER_MIN_TIME)) ||
	    (count_32k & PRCM_CPUx_WAKE_TIMER_EN_BIT))
		return -1;

	wakeup_time = count_32k;

	return 0;
}

/**
 * @brief Clear wakeup timer.
 * retval  0 if success or other if failed.
 */
int32_t HAL_Wakeup_ClrTimer(void)
{
	Wakeup_DisTimer();
	WK_INF("%s\n", __func__);

	return 0;
}

#ifdef __CONFIG_ARCH_APP_CORE
/**
 * @brief Set wakeup IO enable and mode.
 * @note This won't change IO config immediately, the enabled IO will be setted
 *	  to input and wakeup mode before system enter lowpower mode. And the IO
 *	  will be disabled after wakeup. So reinit IO if you want this IO used
 *	  as other function. The IO will used as wakeup IO until be cleaned.
 * @param pn:
 *	  @arg pn-> 0~9.
 * @param mode:
 *	  @arg mode-> 0:negative edge, 1:positive edge.
 * @param pull:
 *	  @arg pull-> 0:no pull, 1:pull up, 2:pull down.
 * retval  None.
 */
void HAL_Wakeup_SetIO(uint32_t pn, WKUPIO_WK_MODE mode, GPIO_PullType pull)
{
	if (pn >= WAKEUP_IO_MAX || mode > WKUPIO_WK_MODE_RISING_EDGE ||
	    pull > GPIO_CTRL_PULL_MAX) {
		WK_ERR("%s,%d err\n", __func__, __LINE__);
		return;
	}

	/* mode */
	if (mode == WKUPIO_WK_MODE_RISING_EDGE)
		wakeup_io_mode |= HAL_BIT(pn);
	else
		wakeup_io_mode &= ~HAL_BIT(pn);

#ifdef WAKEUP_IO_CONFIG_GPIO
	int shift = pn * GPIO_CTRL_PULL_BITS;
	wakeup_io_pull &= ~(GPIO_CTRL_PULL_VMASK << shift);
	wakeup_io_pull |= pull << shift;
#endif

	/* enable */
	wakeup_io_en |= HAL_BIT(pn);

#ifdef WAKEUP_IO_CONFIG_GPIO
	WK_INF("%s en:%x mode:%x pull:%x\n", __func__, wakeup_io_en,
	       wakeup_io_mode, wakeup_io_pull);
#else
	WK_INF("%s en:%x mode:%x\n", __func__, wakeup_io_en, wakeup_io_mode);
#endif
}

/**
 * @brief Clear wakeup IO enable.
 * @param pn:
 *	  @arg pn-> 0~9.
 * retval  None.
 */
void HAL_Wakeup_ClrIO(uint32_t pn)
{
	wakeup_io_en &= ~HAL_BIT(pn);
}

/* All wakeup io is GPIOA, so not return port info. */
GPIO_Pin rom_WakeIo_To_Gpio(uint32_t wkup_io)
{
	switch (wkup_io) {
	case 0: return WAKEUP_IO0;
	case 1: return WAKEUP_IO1;
	case 2: return WAKEUP_IO2;
	case 3: return WAKEUP_IO3;
	case 4: return WAKEUP_IO4;
	case 5: return WAKEUP_IO5;
	case 6: return WAKEUP_IO6;
	case 7: return WAKEUP_IO7;
	case 8: return WAKEUP_IO8;
	case 9: return WAKEUP_IO9;
	}

	WK_ERR("%s,%d err!\n", __func__, __LINE__);

	return 0;
}

/**
 * @brief Set IO hold.
 * @note Set all IO hold before poweroff to prevent IO output low level voltage.
 * @param hold_io:
 *        @arg hold_io-> IO hold mask.
 * retval  0 if success or other if failed.
 */
int32_t HAL_Wakeup_SetIOHold(uint32_t hold_io)
{
	/* clear */
	HAL_PRCM_WakeupIOClearEventDetected(HAL_PRCM_WakeupIOGetEventStatus());
	/* set hold */
	HAL_PRCM_WakeupIOEnableCfgHold(hold_io);

	return 0;
}
#endif

/**
 * @brief Config and enable wakeup io.
 * retval  0 if success or other if failed.
 */
int32_t HAL_Wakeup_SetSrc(uint32_t en_irq)
{
	/* check wakeup time */
	if (WAKEUP_GetTimerEnable() &&
	    (HAL_PRCM_WakeupTimerGetCompareValue() <= (32*WAKEUP_TIMER_MIN_TIME))) {
		WK_ERR("wakeup timer err:%x\n", HAL_PRCM_WakeupTimerGetCurrentValue());
		return -1;
	}

#ifdef __CONFIG_ARCH_APP_CORE
	/* enable wakeup gpio if configed wakeup io */
	if (wakeup_io_en & WAKEUP_IO_MASK) {
#ifdef WAKEUP_IO_CONFIG_GPIO
		uint32_t i, wkio_input;
		wkio_input = wakeup_io_en;
		for (i = 0; (i < WAKEUP_IO_MAX) && wkio_input; wkio_input >>= 1, i++) {
			if (wkio_input & 0x01) {
				GPIO_InitParam param;
				uint32_t pull, shift;

				param.mode = GPIOx_Pn_F6_EINT;
				param.driving = GPIO_DRIVING_LEVEL_1;
				shift = i * GPIO_CTRL_PULL_BITS;
				pull = (wakeup_io_pull >> shift) & GPIO_CTRL_PULL_VMASK;
				param.pull = pull;
				WK_INF("init io:%d\n", WakeIo_To_Gpio(i));
				HAL_GPIO_Init(GPIO_PORT_A, WakeIo_To_Gpio(i), &param); /* set input */
			}
		}
#endif
		/* clear */
		HAL_PRCM_WakeupIOClearEventDetected(HAL_PRCM_WakeupIOGetEventStatus());
		/* mode */
		HAL_PRCM_WakeupIOSetRisingEvent(wakeup_io_mode);
		/* enable */
		HAL_PRCM_WakeupIOEnable(wakeup_io_en);
		/* enable global wakeup io */
		HAL_PRCM_WakeupIOEnableGlobal();
		/* set hold if this io enable */
		HAL_PRCM_WakeupIOEnableCfgHold(wakeup_io_en);
	}
#endif

	if (wakeup_time)
		Wakeup_SetTimer(wakeup_time);

	if (en_irq)
		HAL_NVIC_EnableIRQ(WAKEUP_IRQn); /* enable when sleep */

	return 0;
}

/** @brief Disable wakeup io. */
void HAL_Wakeup_ClrSrc(uint32_t en_irq)
{
	wakeup_time = 0;

#ifdef __CONFIG_ARCH_APP_CORE
	/* wakeup io event */
	wakeup_event = HAL_PRCM_WakeupIOGetEventStatus();
#else
	wakeup_event = 0;
#endif

	/* wakeup timer event */
	if (WAKEUP_GetTimerPending()) {
		WAKEUP_ClearTimerPending();
		while (WAKEUP_GetTimerPending())
			;
		wakeup_event |= PM_WAKEUP_SRC_WKTIMER;
	}

	/* no events is wlan wakeup */
	if (!wakeup_event)
		wakeup_event = PM_WAKEUP_SRC_WKSEV;

#if (defined(__CONFIG_ARCH_APP_CORE) && defined(WAKEUP_IO_CONFIG_GPIO))
	if (wakeup_io_en & WAKEUP_IO_MASK) {
		uint32_t i, wkio_input;

		wkio_input = wakeup_io_en;
		for (i = 0; (i < WAKEUP_IO_MAX) && wkio_input; wkio_input >>= 1, i++) {
			if (wkio_input & 0x01) {
				HAL_GPIO_DeInit(GPIO_PORT_A, WakeIo_To_Gpio(i));
				WK_INF("deinit io:%u\n", WakeIo_To_Gpio(i));
			}
		}
	}
#endif

	Wakeup_Source_Handler();

	if (en_irq)
		HAL_NVIC_EnableIRQ(WAKEUP_IRQn);
}

#ifdef __CONFIG_ARCH_APP_CORE
/**
 * @brief Read wakeup io value.
 * retval bit0~9, matched with PM_WAKEUP_SRC_WKIO0~9
 */
uint32_t HAL_Wakeup_ReadIO(void)
{
	uint32_t i, wkio_input, ret = 0, status;

	wkio_input = wakeup_io_en;
	for (i = 0; (i < WAKEUP_IO_MAX) && wkio_input; wkio_input >>= 1, i++) {
		if (wkio_input & 0x01) {
			status = HAL_GPIO_ReadPin(GPIO_PORT_A, WakeIo_To_Gpio(i));
			if (((wakeup_io_mode & (1 << i)) && status) ||
			    (!(wakeup_io_mode & (1 << i)) && !status)) {
				WK_INF("read io:%u mode:%x status:%x\n", i, wakeup_io_mode, status);
				ret |= (1 << i);
			}
		}
	}

	return ret;
}

/**
 * @brief Check wakeup io mode, EINT mode has expected before suspend.
 * retval  1 if success or 0 if failed.
 */
uint32_t HAL_Wakeup_CheckIOMode(void)
{
	uint32_t i, wkio_input;
	GPIO_InitParam param;

	wkio_input = wakeup_io_en;
	for (i = 0; (i < WAKEUP_IO_MAX) && wkio_input; wkio_input >>= 1, i++) {
		if (wkio_input & 0x01) {
			HAL_GPIO_GetConfig(GPIO_PORT_A, WakeIo_To_Gpio(i), &param);
			if (param.mode != GPIOx_Pn_F6_EINT)
				return 0;
		}
	}

	return 1;
}
#endif

/**
 * @brief Read wakeup timer pending status.
 */
uint32_t HAL_Wakeup_ReadTimerPending(void)
{
	return WAKEUP_GetTimerPending();
}

/**
 * @brief Get last wakeup event.
 * retval  Events defined in PM_WAKEUP_SRC_XX.
 */
uint32_t HAL_Wakeup_GetEvent(void)
{
	return wakeup_event;
}

void HAL_Wakeup_SetEvent(uint32_t event)
{
	wakeup_event = event;
}

/** @brief Init wakeup IO and Timer as disable mode. */
void HAL_Wakeup_Init(void)
{
	Wakeup_ClrTimer();
	Wakeup_DisTimer();
#ifdef __CONFIG_ARCH_APP_CORE
	Wakeup_ClrIO(WAKEUP_IO_MASK);
#endif

	HAL_NVIC_ClearPendingIRQ(WAKEUP_IRQn);
	HAL_NVIC_ConfigExtIRQ(WAKEUP_IRQn, Wakeup_Source_Handler, NVIC_PERIPH_PRIO_DEFAULT);
}

/** @brief Deinit wakeup IO and Timer. */
void HAL_Wakeup_DeInit(void)
{
	HAL_NVIC_DisableIRQ(WAKEUP_IRQn);

#ifdef __CONFIG_ARCH_APP_CORE
	Wakeup_ClrIO(WAKEUP_IO_MASK);
#endif
	Wakeup_DisTimer();
}
