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
#include "driver/chip/hal_adc.h"
#include "driver/chip/private/hal_os.h"

#define ADC_BURST_MODE			(0)
#define ADC_IRQ_MODE 			ADC_IRQ_DATA
#define ADC_CHL 				ADC_CHANNEL_VBAT
#define ADC_FEQ 				500000
#define ADC_FIRST_DELAY 		10

HAL_Semaphore voltage_sem;
#define ADC_VOL_SEM_INIT()		HAL_SemaphoreInitBinary(&voltage_sem)
#define ADC_VOL_SEM_WAIT()		HAL_SemaphoreWait(&voltage_sem, 10000)
#define ADC_VOL_SEM_RELEASE()	HAL_SemaphoreRelease(&voltage_sem)
#define ADC_VOL_SEM_DEINIT()	HAL_SemaphoreDeinit(&voltage_sem)

static uint8_t adc_value_status = 1;
static uint32_t adc_data[10];

static void adc_callback(void *arg)
{
	HAL_Status status = HAL_OK;

	if (adc_value_status)
		return;

	ADC_Channel chl = *((ADC_Channel*)arg);

	static uint32_t count = 0;
#if ADC_BURST_MODE
	for (count = 0; count < 10; count++)
		adc_data[count++] = HAL_ADC_GetFifoData();

	status = HAL_ADC_FifoConfigChannel(chl, ADC_SELECT_DISABLE);

	adc_value_status = 1;
	ADC_VOL_SEM_RELEASE();
#else
	if (HAL_ADC_GetIRQState(chl) == ADC_DATA_IRQ) {
		if (count >= 10) {
			status = HAL_ADC_ConfigChannel(chl, ADC_SELECT_DISABLE, ADC_IRQ_MODE, 0, 0);
			count = 0;
			adc_value_status = 1;
			ADC_VOL_SEM_RELEASE();
		} else
			adc_data[count++] = HAL_ADC_GetValue(chl);
	}
#endif
	if (status != HAL_OK)
		printf("ADC disconfig error--- %d\n", status);
}

static void adc_init()
{
	HAL_Status status = HAL_ERROR;
	ADC_InitParam initParam;

	initParam.delay = ADC_FIRST_DELAY;
	initParam.freq = ADC_FEQ;
#if (__CONFIG_CHIP_ARCH_VER == 2)
	initParam.vref_mode = 1;
#endif

	printf("ADC init...\n");
#if ADC_BURST_MODE
	initParam.mode = ADC_BURST_CONV;
#else
	initParam.mode = ADC_CONTI_CONV;
#endif
	status = HAL_ADC_Init(&initParam);
	if (status != HAL_OK) {
		printf("ADC init error %d\n", status);
		return;
	}
}

static void adc_deinit()
{
	HAL_Status status = HAL_ERROR;
	status = HAL_ADC_DeInit();
	if (status != HAL_OK)
		printf("ADC deinit error %d\n", status);
}

static uint32_t adc_filter(uint32_t adc_data[10])
{
	uint8_t j = 0, i = 0;
	uint32_t sum = 0;
	uint32_t max = 0;

	for (i = 0; i < 10; i++) {
	 for (j = i; j < 9; j++) {
		 if (adc_data[i] >= adc_data[j+1]) {
			 max = adc_data[i];
			 adc_data[i] = adc_data[j+1];
			 adc_data[j+1] = max;
		 }
	 }
	}
	for (i = 0; i < 10; i++) {
	 sum = sum + adc_data[i];
	}
	sum = (sum - adc_data[0] - adc_data[9]) / 8;

	return sum;
}

static void adc_voltage_init()
{
	ADC_VOL_SEM_INIT();

	adc_init();
}

static void adc_voltage_config(uint8_t channel, uint8_t en)
{
	HAL_Status status = HAL_ERROR;

	ADC_Channel chl = (ADC_Channel)channel;

	if (en) {
		printf("ADC channel config...\n");
#if ADC_BURST_MODE
		status = HAL_ADC_FifoConfigChannel(chl, ADC_SELECT_ENABLE);
#else
		status = HAL_ADC_ConfigChannel(chl, ADC_SELECT_ENABLE, ADC_IRQ_MODE, 0, 0);
#endif
		if (status != HAL_OK) {
			printf("ADC config error %d\n", status);
			return;
		}

		printf("ADC callback enable...\n");
		status = HAL_ADC_EnableIRQCallback(chl, adc_callback, &chl);
		if (status != HAL_OK) {
			printf("ADC IRQ Enable error %d\n", status);
			return;
		}

		printf("ADC convert start...\n");
		status = HAL_ADC_Start_Conv_IT();
		if (status != HAL_OK) {
			printf("ADC it mode start error %d\n", status);
			return;
		}
	} else {
		printf("ADC convert stop...\n");
		HAL_ADC_Stop_Conv_IT();
		printf("ADC callback disable...\n");
		HAL_ADC_DisableIRQCallback(chl);
	}
}

/*
 * The relationship between adc value and voltage conversion is as follows:
 * voltage = adc_value * 2500 * ratio / 4096(mv), ratio=1 when adc channel is chl0~chl6,
 * 			ratio=3 when adc channel is chl8,
 */
static uint32_t adc_voltage_get(uint8_t channel)
{
	HAL_Status status = HAL_ERROR;

	adc_value_status = 0;

	ADC_Channel chl = (ADC_Channel)channel;
	uint8_t ratio = (chl == ADC_CHANNEL_VBAT) ? 3 : 1;

#if ADC_BURST_MODE
	status = HAL_ADC_FifoConfigChannel(chl, ADC_SELECT_ENABLE);
#else
	status = HAL_ADC_ConfigChannel(chl, ADC_SELECT_ENABLE, ADC_IRQ_MODE, 0, 0);
#endif
	if (status != HAL_OK) {
		printf("ADC it mode start error %d\n", status);
	}

	ADC_VOL_SEM_WAIT();

	if (adc_value_status) {
		uint32_t adc_value = adc_filter(adc_data);
		return (adc_value * 2500 * ratio / 4096);
	}

	return 0;
}

static void adc_voltage_deinit()
{
	adc_deinit();

	ADC_VOL_SEM_DEINIT();
}

int main(void)
{
	printf("ADC demo started.\n");

	adc_voltage_init();

	adc_voltage_config(ADC_CHL, 1);

	printf("ADC get channel voltage...\n");

	uint32_t voltage;
	while (1) {
		if ((voltage = adc_voltage_get(ADC_CHL)))
			printf("VBAT voltage: %dmV\n", voltage);
		OS_MSleep(500);
	}

	adc_voltage_config(ADC_CHL, 0);

	adc_voltage_deinit();

	printf("ADC demo over.\n");

	return 0;
}

