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

#ifndef _APPS_BUTTONS_MATRIX_LOW_LEVEL_H_
#define _APPS_BUTTONS_MATRIX_LOW_LEVEL_H_

#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_wakeup.h"

typedef struct {
	char *name;
	GPIO_PinMuxParam gpio_param;
	uint8_t enable_wakeup;
	WKUPIO_WK_MODE wakeup_mode;
	GPIO_PullType pull_mode;
} matrix_button;

typedef struct {
	const matrix_button *matrix_buttons_row_p;
	uint8_t count_row;

	const matrix_button *matrix_buttons_col_p;
	uint8_t count_col;
} matrix_button_info;

int matrix_buttons_low_level_wait_semaphore(uint32_t waitms);
int matrix_buttons_low_level_release_semaphore(void);
int matrix_buttons_low_level_get_state(void);
int matrix_buttons_low_level_init(void);
int matrix_buttons_low_level_deinit(void);

#endif /* _APPS_BUTTONS_MATRIX_LOW_LEVEL_H_ */

