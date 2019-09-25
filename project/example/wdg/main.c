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
#include "driver/chip/hal_wdg.h"

//#define WDG_RESET_MODE
#define WDG_INTERRUPT_MODE
//#define WDG_RESETCPU_MODE

#define WDG_START(void)	HAL_WDG_Start(void)
#define WDG_STOP(void)	HAL_WDG_Stop(void)

#if (defined(WDG_RESET_MODE))
int WdgInitResetMode(void)
{
	HAL_Status status = HAL_ERROR;
	WDG_HwInitParam hwParam;
	WDG_InitParam param;

	hwParam.event = WDG_EVT_RESET;
	hwParam.timeout = WDG_TIMEOUT_2SEC;
	hwParam.resetCycle = 1;

	param.hw = hwParam;

	status = HAL_WDG_Init(&param);
	if (status != HAL_OK)
		printf("Wdg Init Error %d\n", status);
	else
		printf("WatchDog ResetMode Started.\n");

	return status;
}
#endif

void WdgFeedDog(void)
{
	HAL_WDG_Feed();
	printf("Feed Dog.\n");
}

void WdgDeInit(void)
{
	HAL_Status status = HAL_ERROR;

	status = HAL_WDG_DeInit();
	if (status != HAL_OK)
		printf("WatchDog DeInit Error %d\n", status);
}

/** @brief watchdog Interrupt function */
void WdgIRQCallback(void *arg)
{
	static uint32_t callBackCount;

	callBackCount++;
	printf("In wdg_IRQCallback Times: %u\n", callBackCount);
}

#if (defined(WDG_INTERRUPT_MODE))
int WdgInitInterruptMode(void)
{
	/*  support interrupt  */
	HAL_Status status = HAL_ERROR;
	WDG_HwInitParam hwParam;
	WDG_InitParam param;

	hwParam.event = WDG_EVT_INTERRUPT;
	hwParam.timeout = WDG_TIMEOUT_2SEC;
	hwParam.resetCycle = 1;

	param.hw = hwParam;
	param.callback = WdgIRQCallback;
	param.arg = NULL;

	status = HAL_WDG_Init(&param);
	if (status != HAL_OK)
		printf("Wdg Init Error %d\n", status);
	else
		printf("WatchDog Interrupt Mode Started.\n");

	return status;
}
#endif

#if (defined(WDG_RESETCPU_MODE))
int WdgInitResetCpuMode(void)
{
	HAL_Status status = HAL_ERROR;
	WDG_HwInitParam hwParam;
	WDG_InitParam param;

	hwParam.event = WDG_EVT_RESET_CPU;
	hwParam.resetCpuMode = WDG_RESET_CPU_CORE;
	hwParam.timeout = WDG_TIMEOUT_2SEC;
	hwParam.resetCycle = 1;

	param.hw = hwParam;

	status = HAL_WDG_Init(&param);
	if (status != HAL_OK)
		printf("Wdg Init Error %d\n", status);
	else
		printf("WatchDog ResetCpu Mode Started.\n");

	return status;
}
#endif

void WdgShow(void)
{
	/** "watchdog show over" just show on InterruptMode **/
	static int count_feed = 5;
	static int count_not_feed = 13;
	uint8_t is_show = 1;

	printf("Watchdog Show Start.\n");
	WDG_START();

	while (is_show) {
		if (count_feed > 0) {
			WdgFeedDog();
			count_feed--;
		} else if (count_not_feed > 0) {
			printf("Do't Feed Dog.\n");
			count_not_feed--;
		} else
			is_show = 0;
		OS_Sleep(1);
	}

	WDG_STOP();
	printf("Watchdog Show Over.\n");
}

/*Run this example, please connect the uart0 */
int main(void)
{
	printf("WatchDog Example Started.\n\n");

#if (defined(WDG_RESET_MODE))
	WdgInitResetMode();
#elif (defined(WDG_INTERRUPT_MODE))
	WdgInitInterruptMode();
#elif (defined(WDG_RESETCPU_MODE))
	WdgInitResetCpuMode();
#endif

	WdgShow();

	WdgDeInit();
	printf("Watchdog Example Over.\n");

	return 0;
}
