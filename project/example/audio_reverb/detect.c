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

#include <stdio.h>
#include <string.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_gpio.h"
#include "karaok.h"

#define MICROPHONE_GPIO_PORT      GPIO_PORT_A
#define MICROPHONE_GPIO_PIN       GPIO_PIN_8

static OS_Timer_t detect_timer;

static void detect_timer_cb(void *arg)
{
	uint8_t level = HAL_GPIO_ReadPin(MICROPHONE_GPIO_PORT, MICROPHONE_GPIO_PIN);

	if (level == 1) {
		printf("microphone is removed, stop karaok now.\n");
		karaok_stop();
	} else {
		printf("microphone is inserted, start karaok now.\n");
		karaok_start();
	}
}

static void microphone_det_cb(void* arg)
{
	OS_TimerStart(&detect_timer);
}

int microphone_detect_start(void)
{
	GPIO_InitParam param = {0};
	GPIO_IrqParam irq_param = {0};

	OS_TimerCreate(&detect_timer, OS_TIMER_ONCE, detect_timer_cb, NULL, 300);

	param.driving = GPIO_DRIVING_LEVEL_1;
	param.mode = GPIOx_Pn_F6_EINT;
	param.pull = GPIO_PULL_UP;
	HAL_GPIO_Init(MICROPHONE_GPIO_PORT, MICROPHONE_GPIO_PIN, &param);

	irq_param.event = GPIO_IRQ_EVT_BOTH_EDGE;
	irq_param.callback = microphone_det_cb;
	irq_param.arg = NULL;
	HAL_GPIO_EnableIRQ(MICROPHONE_GPIO_PORT, MICROPHONE_GPIO_PIN, &irq_param);

	OS_TimerStart(&detect_timer);

	return 0;
}
