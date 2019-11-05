/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions
 *	are met:
 *	  1. Redistributions of source code must retain the above copyright
 *		 notice, this list of conditions and the following disclaimer.
 *	  2. Redistributions in binary form must reproduce the above copyright
 *		 notice, this list of conditions and the following disclaimer in the
 *		 documentation and/or other materials provided with the
 *		 distribution.
 *	  3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *		 its contributors may be used to endorse or promote products derived
 *		 from this software without specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *	A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *	OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *	SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *	LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *	THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "kernel/os/os.h"
#include "driver/hal_board.h"
#include "driver/hal_dev.h"
#include "driver/chip/hal_wakeup.h"

#include "buttons.h"
#include "matrix_buttons_low_level.h"

#include "pm/pm.h"

#define MATRIX_BUTTON_DBG  1
#define MATRIX_BUTTON_INTERRUPT_DBG 0 /* the debug macro used in interrupt function */

#if MATRIX_BUTTON_DBG
#define MATRIX_BUTTON_DEBUG(msg, arg...)      printf("[matrix low level button debug] <%s : %d> " msg "\n", __FUNCTION__, __LINE__, ##arg)
#define MATRIX_BUTTON_WARNING(msg, arg...)    printf("[matrix low level button warning] <%s : %d> " msg "\n", __FUNCTION__, __LINE__, ##arg)
#else
#define MATRIX_BUTTON_DEBUG(msg, arg...)
#define MATRIX_BUTTON_WARNING(msg, arg...)
#endif
#define MATRIX_BUTTON_ERR(msg, arg...)        printf("[matrix low level button err] <%s : %d> " msg "\n", __FUNCTION__, __LINE__, ##arg)

#if MATRIX_BUTTON_INTERRUPT_DBG
#define MATRIX_BUTTON_IT_LOG(fmt, arg...)            		 \
    do {                                             \
            __nonxip_data static char __fmt[] = fmt; \
            printf(__fmt, ##arg);                    \
    } while (0)
#else
#define MATRIX_BUTTON_IT_LOG(fmt, arg...)
#endif

#define MATRIX_DEBOUNCE_TIME 50 //ms
static OS_Semaphore_t matrix_buttons_sem;
static matrix_button_info matrix_buttons_info;

static int matrix_buttons_gpio_init(GPIO_IRQCallback cb);

__nonxip_text
static void matrix_buttons_gpio_cb(void *arg) /* can't put this function into xip */
{
	OS_SemaphoreRelease(&matrix_buttons_sem);

	MATRIX_BUTTON_IT_LOG("matrix gpio buttons interrupt callback");
}

static int matrix_buttons_which_wakeup_io(GPIO_Port port, GPIO_Pin pin)
{
	if (port == GPIO_PORT_B)
		return -1;

	switch (pin) {
		case WAKEUP_IO0: return 0;
		case WAKEUP_IO1: return 1;
		case WAKEUP_IO2: return 2;
		case WAKEUP_IO3: return 3;
		case WAKEUP_IO4: return 4;
		case WAKEUP_IO5: return 5;
		case WAKEUP_IO6: return 6;
		case WAKEUP_IO7: return 7;
		case WAKEUP_IO8: return 8;
		case WAKEUP_IO9: return 9;
		default: return -1;
	}
}

#ifdef CONFIG_PM

static int matrix_buttons_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	int n_row;
	int n_col;
	int i;
	int wakeup_io;

	/* get matrix buttons info from board_config.c */
	HAL_BoardIoctl(HAL_BIR_GET_CFG, HAL_MKDEV(HAL_DEV_MAJOR_MATRIX_BUTTON, 0), (uint32_t)&matrix_buttons_info);
	n_row = matrix_buttons_info.count_row;
	n_col = matrix_buttons_info.count_col;

	/* set each row pin */
	for (i = 0; i < n_row; i++) {
		if (matrix_buttons_info.matrix_buttons_row_p[i].enable_wakeup) {
			wakeup_io = matrix_buttons_which_wakeup_io(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
							matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin);
			if (wakeup_io >= 0) {
				HAL_PRCM_SetWakeupDebClk0(0);
				HAL_PRCM_SetWakeupIOxDebSrc(wakeup_io, 0);
				HAL_PRCM_SetWakeupIOxDebounce(wakeup_io, 16); //Debounce 1ms
				HAL_Wakeup_SetIO(wakeup_io, matrix_buttons_info.matrix_buttons_row_p[i].wakeup_mode,
							matrix_buttons_info.matrix_buttons_row_p[i].pull_mode);
			}
		}
	}

	/* set each col pin */
	for (i = 0; i < n_col; i++) {
		if (matrix_buttons_info.matrix_buttons_col_p[i].enable_wakeup) {
			wakeup_io = matrix_buttons_which_wakeup_io(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
							matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin);
			if (wakeup_io >= 0) {
				HAL_PRCM_SetWakeupDebClk0(0);
				HAL_PRCM_SetWakeupIOxDebSrc(wakeup_io, 0);
				HAL_PRCM_SetWakeupIOxDebounce(wakeup_io, 16); //Debounce 1ms
				HAL_Wakeup_SetIO(wakeup_io, matrix_buttons_info.matrix_buttons_col_p[i].wakeup_mode,
							matrix_buttons_info.matrix_buttons_col_p[i].pull_mode);
			}
		}
	}

	return 0;
}

static int matrix_buttons_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		break;
	default:
		break;
	}

	return 0;
}

static const struct soc_device_driver matrix_buttons_drv = {
	.name = "matrix_buttons_drv",
	.suspend = matrix_buttons_suspend,
	.resume = matrix_buttons_resume,
};

static struct soc_device matrix_buttons_dev = {
	.name = "matrix_buttons_dev",
	.driver = &matrix_buttons_drv,
};

#endif

static int matrix_buttons_gpio_init(GPIO_IRQCallback cb)
{
	int n_row;
	int n_col;
	int i;
	GPIO_IrqParam irq_param;

	/* get matrix buttons info from board_config.c */
	HAL_BoardIoctl(HAL_BIR_GET_CFG, HAL_MKDEV(HAL_DEV_MAJOR_MATRIX_BUTTON, 0), (uint32_t)&matrix_buttons_info);
	n_row = matrix_buttons_info.count_row;
	n_col = matrix_buttons_info.count_col;

	irq_param.event = GPIO_IRQ_EVT_BOTH_EDGE;
	irq_param.callback = cb;
	irq_param.arg = NULL;

	/* init each row pin */
	for (i = 0; i < n_row; i++) {
		HAL_GPIO_Init(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin,
						&matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.config);
		HAL_GPIO_WritePin(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
							matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin,
							GPIO_PIN_HIGH);
	}
	/* init each col pin */
	for (i = 0; i < n_col; i++) {
		HAL_GPIO_Init(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin,
						&matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.config);
		HAL_GPIO_EnableIRQ(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin,
						&irq_param);
	}

#ifdef CONFIG_PM
	pm_register_ops(&matrix_buttons_dev);
#endif
	return 0;
}

static int matrix_buttons_gpio_deinit(void)
{
	int n_row = matrix_buttons_info.count_row;
	int n_col = matrix_buttons_info.count_col;
	int i;

	/* deinit each pin */
	for (i = 0; i < n_row; i++) {
		HAL_GPIO_DisableIRQ(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin);
		HAL_GPIO_DeInit(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin);
	}
	for (i = 0; i < n_col; i++) {
		HAL_GPIO_DisableIRQ(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin);
		HAL_GPIO_DeInit(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin);
	}

	return 0;
}

/**
  * @brief Get matrix buttons state.
  * @note Get matrix buttons state.
  * @param void
  * @retval The high 16 bits represent row coordinates,
            the lower 16 bits represent the column coordinates
  */
static uint32_t matrix_buttons_get_state(void)
{
	int n_row = matrix_buttons_info.count_row;
	int n_col = matrix_buttons_info.count_col;
	int i;
	unsigned int matrix_button_state = 0;
	GPIO_PinState pin_state = 0;

	/* row out mode, out high */
	for (i = 0; i < n_row; i++) {
		HAL_GPIO_SetMode(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin,
						GPIOx_Pn_F1_OUTPUT);
		HAL_GPIO_WritePin(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin,
						GPIO_PIN_HIGH);
	}
	/* col input mode, pull down */
	for (i = 0; i < n_col; i++) {
		HAL_GPIO_SetMode(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin,
						GPIOx_Pn_F0_INPUT);
		HAL_GPIO_SetPull(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin,
						GPIO_PULL_DOWN);
	}

	/* read every col pin state */
	for (i = 0; i < n_col; i++) {
		pin_state = HAL_GPIO_ReadPin(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
									matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin);
		if (pin_state == GPIO_PIN_HIGH)
			matrix_button_state |= 1 << i;
		else
			matrix_button_state &= ~(1 << i);
	}

	if (!(matrix_button_state & 0x0000ffff)) {
		matrix_button_state = 0;
		goto out;
	}

	/* row input mode, pull up */
	for (i = 0; i < n_row; i++) {
		HAL_GPIO_SetMode(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin,
						GPIOx_Pn_F0_INPUT);
		HAL_GPIO_SetPull(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin,
						GPIO_PULL_UP);
	}
	/* col output mode, out low */
	for (i = 0; i < n_col; i++) {
		HAL_GPIO_SetMode(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin,
						GPIOx_Pn_F1_OUTPUT);
		HAL_GPIO_WritePin(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin,
						GPIO_PIN_LOW);
	}

	/* read every row pin state */
	for (i = 0; i < n_row; i++) {
		pin_state = HAL_GPIO_ReadPin(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
									matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin);
		if (pin_state == GPIO_PIN_LOW)
			matrix_button_state |= 1 << (i + 16);
		else
			matrix_button_state &= ~(1 << (i + 16));
	}

	if (!(matrix_button_state & 0xffff0000)) {
		matrix_button_state = 0;
		goto out;
	}

out:
	/* Return to the initial state */
	for (i = 0; i < n_row; i++) {
		HAL_GPIO_Init(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin,
						&matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.config);
		HAL_GPIO_WritePin(matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.port,
							matrix_buttons_info.matrix_buttons_row_p[i].gpio_param.pin,
							GPIO_PIN_HIGH);
	}
	for (i = 0; i < n_col; i++) {
		HAL_GPIO_Init(matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.port,
						matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.pin,
						&matrix_buttons_info.matrix_buttons_col_p[i].gpio_param.config);
	}

	return matrix_button_state;
}

/**
  * @brief Initialize the matrix low level buttons.
  * @note This interface will initialize all the pins.
  * @param void
  * @retval 0: success, -1: fail
  */
int matrix_buttons_low_level_init(void)
{
	OS_Status sta;

	matrix_buttons_gpio_init(matrix_buttons_gpio_cb);

	sta = OS_SemaphoreCreate(&matrix_buttons_sem, 0, 1);
	if (sta != OS_OK) {
		MATRIX_BUTTON_ERR("matrix buttons semaphore create error");
		return -1;
	}
	return 0;
}

int matrix_buttons_low_level_deinit(void)
{
	matrix_buttons_gpio_deinit();
	return 0;
}

int matrix_buttons_low_level_get_state(void)
{
	return matrix_buttons_get_state();
}

int matrix_buttons_low_level_wait_semaphore(uint32_t waitms)
{
	return OS_SemaphoreWait(&matrix_buttons_sem, waitms);
}

int matrix_buttons_low_level_release_semaphore(void)
{
	return OS_SemaphoreRelease(&matrix_buttons_sem);
}

