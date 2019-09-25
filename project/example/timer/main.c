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
#include "kernel/os/os.h"
#include "driver/chip/hal_timer.h"

/** Have TIMER0_ID & TIMER1_ID timer **/
#define TIMERID		TIMER0_ID
//#define TIMERID		TIMER1_ID

#define HFCLOCK			24000000		/*External clock frequency (Hz)*/
#define CLK_PRESCALER	4			/*clock prescale*/
#define COUNT_TIME		1			/*timer count down time (second)*/

uint32_t sec_count;

void timer_callback(void *arg)
{
	sec_count++;
	printf(" timer irq: %u\n", sec_count);
}

int timer_init(void)
{
	HAL_Status status = HAL_ERROR;
	TIMER_InitParam param;

	param.arg = NULL;
	param.callback = timer_callback;
	param.cfg = HAL_TIMER_MakeInitCfg(TIMER_MODE_REPEAT,		/*timer mode*/
									TIMER_CLK_SRC_HFCLK,		/*HFCLOCK*/
									TIMER_CLK_PRESCALER_4);		/*CLK_PRESCALER*/
	param.isEnableIRQ = 1;
	param.period = COUNT_TIME * (HFCLOCK / CLK_PRESCALER);

	status = HAL_TIMER_Init(TIMERID, &param);
	if (status != HAL_OK)
		printf("timer int error %d\n", status);

	return status;
}
void timer_show(void)
{
	HAL_TIMER_Start(TIMERID);

	printf("timer%d start.\n", TIMERID);
	printf("the timer count down 1s\n");
	printf("wait timer irq.\n");

	OS_Sleep(10);
	HAL_TIMER_Pause(TIMERID);
	printf("timer over 10s,timer pause 10s!\n");

	OS_Sleep(10);
	HAL_TIMER_Continue(TIMERID);
	printf("timer over 20s,timer continue.\n");

	OS_Sleep(10);
	HAL_TIMER_Stop(TIMERID);
	printf("timer over 30s, timer stop!\n");

	printf("timer show over.\n");
}

int timer_deinit(void)
{
	HAL_Status status = HAL_ERROR;

	status = HAL_TIMER_DeInit(TIMERID);
	if (status != HAL_OK)
		printf("timer deinit error %d\n", status);

	return status;
}

/* Run this example, please connect the uart0 */
int main(void)
{
	printf("timer example started.\n\n");

	timer_init();
	timer_show();
	timer_deinit();

	printf("timer example over.\n");

	return 0;
}
