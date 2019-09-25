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
#include "driver/chip/hal_pwm.h"

#define PWM_OUTPUT_CHL		PWM_GROUP0_CH0
#define PWM_INPUT_CHL		PWM_GROUP0_CH1
#define PWM_OUTPUT_MODE		PWM_CYCLE_MODE
#define PWM_INPUT_MODE		PWM_CAPTURE_MODE

static void pwm_cycle_irq(void *arg,  PWM_IrqEvent event)
{
	printf("this is pwm output irq!\n");
	HAL_PWM_DisableIRQ(PWM_OUTPUT_CHL);
}

static void pwm_cycle_mode_Set()
{
	HAL_Status status = HAL_ERROR;
	PWM_ClkParam clk_param;
	PWM_ChInitParam ch_param;
	PWM_IrqParam irq_param;
	int max_duty_ratio = 0;

	clk_param.clk = PWM_CLK_HOSC;
	clk_param.div = PWM_SRC_CLK_DIV_1;

	status = HAL_PWM_GroupClkCfg(PWM_OUTPUT_CHL / 2, &clk_param);
	if (status != HAL_OK)
		printf("%s(): %d, PWM group clk config error\n", __func__, __LINE__);

	ch_param.hz = 1000;
	ch_param.mode = PWM_OUTPUT_MODE;
	ch_param.polarity = PWM_HIGHLEVE;
	max_duty_ratio = HAL_PWM_ChInit(PWM_OUTPUT_CHL, &ch_param);
	if (max_duty_ratio == -1)
		printf("%s(): %d, PWM ch init error\n", __func__, __LINE__);

	status = HAL_PWM_ChSetDutyRatio(PWM_OUTPUT_CHL, max_duty_ratio / 2);
	if (status != HAL_OK)
		printf("%s(): %d, PWM set duty ratio error\n", __func__, __LINE__);

	status = HAL_PWM_EnableCh(PWM_OUTPUT_CHL, PWM_OUTPUT_MODE, 1);
	if (status != HAL_OK)
		printf("%s(): %d, PWM ch enable error\n", __func__, __LINE__);

	irq_param.arg = NULL;
	irq_param.callback = pwm_cycle_irq;
	irq_param.event = PWM_IRQ_OUTPUT;
	status = HAL_PWM_EnableIRQ(PWM_OUTPUT_CHL, &irq_param);
	if (status != HAL_OK)
		printf("%s(): %d, PWM enable irq error\n", __func__, __LINE__);
}

static void pwm_capture_mode_set()
{
	HAL_Status status = HAL_ERROR;
	PWM_ClkParam clk_param;
	PWM_ChInitParam ch_param;
	PWM_IrqParam irq_param;

	clk_param.clk = PWM_CLK_HOSC;
	clk_param.div = PWM_SRC_CLK_DIV_1;

	status = HAL_PWM_GroupClkCfg(PWM_INPUT_CHL / 2, &clk_param);
	if (status != HAL_OK)
		printf("%s(): %d, PWM group clk config error\n", __func__, __LINE__);

	ch_param.hz = 1000;
	ch_param.mode = PWM_INPUT_MODE;
	ch_param.polarity = PWM_HIGHLEVE;

	if (HAL_PWM_ChInit(PWM_INPUT_CHL, &ch_param) == -1)
		printf("%s(): %d, PWM ch init error\n", __func__, __LINE__);

	status = HAL_PWM_EnableCh(PWM_INPUT_CHL, PWM_INPUT_MODE, 1);
	if (status != HAL_OK)
		printf("%s(): %d, PWM ch enable error\n", __func__, __LINE__);

	irq_param.arg = NULL;
	irq_param.callback = NULL;
	irq_param.event = PWM_IRQ_BOTHEDGE;
	status = HAL_PWM_EnableIRQ(PWM_INPUT_CHL, &irq_param);
	if (status != HAL_OK)
		printf("%s(): %d, PWM enable irq error\n", __func__, __LINE__);
}

static int pwm_capture_value()
{
	PWM_CapResult data = HAL_PWM_CaptureResult(PWM_CAP_CYCLE, PWM_INPUT_CHL);

	if (data.periodTime) {
		printf("caputre result:\n");
		printf("\thigh level %d\n", data.highLevelTime);
		printf("\tlow level  %d\n", data.lowLevelTime);
		printf("\tpreiod     %d\n", data.periodTime);
		return 1;
	}

	return 0;
}

static void pwm_channel_deinit(uint8_t channel, uint8_t mode)
{
	/*Stop channel*/
	HAL_PWM_EnableCh((PWM_CH_ID)channel, (PWM_Mode)mode, 0);
	/*Deinit channel*/
	HAL_PWM_ChDeinit((PWM_CH_ID)channel);
}

/* Run this demo, please connect the PA8 and PA9. */
int main(void)
{
	printf("PWM demo started\n");

	pwm_cycle_mode_Set();
	OS_MSleep(10);
	pwm_capture_mode_set();

	uint8_t count = 3;
	while (count) {
		if (pwm_capture_value())
			count --;
		OS_MSleep(500);
	}

	pwm_channel_deinit(PWM_INPUT_CHL, PWM_INPUT_MODE);
	pwm_channel_deinit(PWM_OUTPUT_CHL, PWM_OUTPUT_MODE);

	printf("\nPWM demo over.\n");

	return 0;
}

