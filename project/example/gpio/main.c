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
#include "driver/chip/hal_gpio.h"

#define GPIO_MODE_INPUT_TEST	1
#define GPIO_MODE_OUTPUT_TEST	1
#define GPIO_MODE_EINT_TEST		1

#define GPIO_INPUT_PORT			GPIO_PORT_A
#define GPIO_INPUT_PIN			GPIO_PIN_9

#define GPIO_OUTPUT_PORT		GPIO_PORT_A
#define GPIO_OUTPUT_PIN			GPIO_PIN_10

#define GPIO_EINT_PORT			GPIO_PORT_A
#define GPIO_EINT_PIN			GPIO_PIN_11

#if GPIO_MODE_INPUT_TEST
static void gpio_input_init()
{
	GPIO_InitParam param;
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.mode = GPIOx_Pn_F0_INPUT;
	param.pull = GPIO_PULL_NONE;
	HAL_GPIO_Init(GPIO_INPUT_PORT, GPIO_INPUT_PIN, &param);
}

static  uint8_t gpio_input_read()
{
	return (uint8_t)HAL_GPIO_ReadPin(GPIO_INPUT_PORT, GPIO_INPUT_PIN);
}

static void gpio_input_deinit()
{
	HAL_GPIO_DeInit(GPIO_INPUT_PORT, GPIO_INPUT_PIN);
}

static void gpio_input_test()
{
	printf("gpio input test start.\n");

	gpio_input_init();

	uint8_t i;
	for (i = 0; i < 10; i++) {
		printf("gpio level: %d\n", gpio_input_read());
		OS_MSleep(50);
	}

	gpio_input_deinit();

	printf("gpio input test end.\n");
}
#endif

#if GPIO_MODE_OUTPUT_TEST
static void gpio_output_init()
{
	GPIO_InitParam param;
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.mode = GPIOx_Pn_F1_OUTPUT;
	param.pull = GPIO_PULL_NONE;
	HAL_GPIO_Init(GPIO_OUTPUT_PORT, GPIO_OUTPUT_PIN, &param);
}

static void gpio_output_ctl(uint8_t level)
{
	HAL_GPIO_WritePin(GPIO_OUTPUT_PORT, GPIO_OUTPUT_PIN, level ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
}

static void gpio_output_deinit()
{
	HAL_GPIO_DeInit(GPIO_OUTPUT_PORT, GPIO_OUTPUT_PIN);
}

static void gpio_output_test()
{
	printf("gpio output test start.\n");

	gpio_output_init();

	uint8_t i;
	uint8_t level = 1;
	for (i = 0; i < 10; i++) {
		gpio_output_ctl(level);
		printf("gpio level write %d\n", level);
		level = (!level & 0x01);
		OS_MSleep(50);
	}

	gpio_output_deinit();

	printf("gpio output test end.\n");
}
#endif

#if GPIO_MODE_EINT_TEST
static void gpio_eint_callback()
{
	printf("button is pressed\n");
}

static void gpio_eint_init()
{
	GPIO_InitParam param;
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.mode = GPIOx_Pn_F6_EINT;
	param.pull = GPIO_PULL_NONE;
	HAL_GPIO_Init(GPIO_EINT_PORT, GPIO_EINT_PIN, &param);

	GPIO_IrqParam irq_param;
	irq_param.arg = NULL;
	irq_param.callback = gpio_eint_callback;
	irq_param.event = GPIO_IRQ_EVT_RISING_EDGE; /*set pin irq low level trigger*/
	HAL_GPIO_EnableIRQ(GPIO_EINT_PORT, GPIO_EINT_PIN, &irq_param);
}

static void gpio_eint_deinit()
{
	HAL_GPIO_DisableIRQ(GPIO_EINT_PORT, GPIO_EINT_PIN);
	HAL_GPIO_DeInit(GPIO_EINT_PORT, GPIO_EINT_PIN);
}

static void gpio_eint_test()
{
	printf("gpio eint test start.\n");

	printf("please press button...\n");

	gpio_eint_init();

	//wait for gpio irq...

	//gpio_eint_deinit();
}
#endif


/*Run this demo, please connect the xr872et_ver_dig board.*/
int main(void)
{
	printf("gpio demo started.\n");

#if GPIO_MODE_INPUT_TEST
	gpio_input_test();
#endif

#if GPIO_MODE_OUTPUT_TEST
	gpio_output_test();
#endif

#if GPIO_MODE_EINT_TEST
	gpio_eint_test();

	while (1)
		OS_MSleep(100);
#endif

	printf("gpio demo over.\n");

	return 0;
}

