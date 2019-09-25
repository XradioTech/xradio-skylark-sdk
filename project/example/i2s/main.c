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
#include "driver/chip/hal_i2s.h"
#include "../src/driver/chip/hal_base.h"



#define I2S_TEST_SAMPLE_RATE	16000	//[8000,16000,32000, 12000,24000,48000, 11025,22050,44100]
#define I2S_TEST_CHANNEL_NUMS	2		//[1,2]
#define I2S_TEST_RESOLUTION		32		//[8,12,16,20,24,28,32]


static unsigned char tx_sample_resolution;
void I2S_Loop_En(uint8_t en);

unsigned int tx_data_32bit[32] = {
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
};


static int i2s_init_example(void)
{
	I2S_Param i2s_param;
	HAL_Memset(&i2s_param, 0, sizeof(i2s_param));

	i2s_param.mclkDiv = 1;
	int ret = HAL_I2S_Init(&i2s_param);
	if (ret != HAL_OK) {
		printf("I2S Init failed..\n");
		return HAL_ERROR;
	}

	I2S_Loop_En(1);

	return HAL_OK;
}

static int i2s_deinit_example(void)
{
	I2S_Loop_En(0);
	HAL_I2S_DeInit();

	return HAL_OK;
}

static int i2s_open_example(uint8_t direction, uint32_t samplerate, uint8_t channels, uint8_t resolution)
{
	if (direction != 0 && direction != 1) {
		printf("Invalid direction %d\n", direction);
		return HAL_INVALID;
	}

	if (channels < 1 || channels > 2) {
		printf("Invalid channels %d\n", channels);
		return HAL_INVALID;
	}

	switch(samplerate){
		case 8000:
			break;
		case 16000:
			break;
		case 32000:
			break;

		case 12000:
			break;
		case 24000:
			break;
		case 48000:
			break;

		case 11025:
			break;
		case 22050:
			break;
		case 44100:
			break;

		default:
			printf("Invalid sample rate %u\n",samplerate);
			return HAL_INVALID;
	}

	if(!direction)
		tx_sample_resolution = resolution;

	switch(resolution){
		case 8:
			resolution = I2S_SR8BIT;
			break;
		case 12:
			resolution = I2S_SR12BIT;
			break;
		case 16:
			resolution = I2S_SR16BIT;
			break;
		case 20:
			resolution = I2S_SR20BIT;
			break;
		case 24:
			resolution = I2S_SR24BIT;
			break;
		case 28:
			resolution = I2S_SR28BIT;
			break;
		case 32:
			resolution = I2S_SR32BIT;
			break;

		default:
			printf("Invalid resolution %d\n",resolution);
			return HAL_INVALID;
	}

	I2S_DataParam i2s_data;
	i2s_data.direction = direction;
	i2s_data.channels = channels;
	i2s_data.resolution = resolution;
	i2s_data.sampleRate = samplerate;
	i2s_data.bufSize = sizeof(tx_data_32bit);
	if (HAL_I2S_Open(&i2s_data) != HAL_OK) {
		printf("I2S open failed..\n");
		return HAL_ERROR;
	}

	return HAL_OK;
}

static int i2s_close_example(uint8_t direction)
{
	HAL_Status hal_status;

	if (direction != 0 && direction != 1) {
		printf("Invalid direction %d\n", direction);
		return HAL_INVALID;
	}

	hal_status = HAL_I2S_Close(direction);
	if (hal_status == HAL_OK) {
		return HAL_OK;
	} else {
		printf("HAL_I2S_Close return: hal_status = %d\n", hal_status);
		return HAL_ERROR;
	}
}

static int i2s_write_dma_example(uint8_t *buffer, uint32_t size)
{
	uint8_t *buf;
	uint32_t i,*buf_32,bit_map;

	if(!buffer || !size){
		printf("Invalid write buf|size params error!\n");
		return HAL_INVALID;
	}

	buf = (uint8_t *)HAL_Malloc(size);
	if (buf == NULL) {
		printf("Malloc I2S write buffer Fail\n");
		return HAL_ERROR;
	}

	HAL_Memcpy(buf, buffer, size);
	bit_map = ((1<<tx_sample_resolution)-1)<<(32-tx_sample_resolution);
	buf_32 = (unsigned int *)buf;
	for(i=0; i<size/4; i++){
		buf_32[i] = buf_32[i] & bit_map;
	}

	if(HAL_I2S_Write_DMA(buf, size) != size){
		HAL_Free(buf);
		printf("I2S write error!\n");
		return HAL_ERROR;
	}

	printf("\nwrite buf:\n");
	for(i=0; i<size/4; i++){
		printf("0x%08x ",buf_32[i]);
	}
	printf("\n\n");

	HAL_Free(buf);
	return HAL_OK;
}

static int i2s_read_dma_example(uint32_t size)
{
	uint8_t *buf;
	uint32_t i,*buf_32;

	if(!size){
		printf("Invalid read size params error!\n");
		return HAL_INVALID;
	}

	buf = (uint8_t *)HAL_Malloc(size);
	if (buf == NULL) {
		printf("Malloc I2S read buffer Fail\n");
		return HAL_ERROR;
	}

	HAL_Memset(buf, 0, size);
	if(HAL_I2S_Read_DMA(buf, size) != size){
		HAL_Free(buf);
		printf("I2S read error!\n");
		return HAL_ERROR;
	}

	printf("\nread buf:\n");
	buf_32 = (uint32_t *)buf;
	for(i=0; i<size/4; i++)
		printf("0x%08x ", buf_32[i]);
	printf("\n\n");

	HAL_Free(buf);
	return HAL_OK;
}


static void i2s_test_example(void)
{
	i2s_init_example();

	i2s_open_example(0, I2S_TEST_SAMPLE_RATE, I2S_TEST_CHANNEL_NUMS, I2S_TEST_RESOLUTION);
	i2s_open_example(1, I2S_TEST_SAMPLE_RATE, I2S_TEST_CHANNEL_NUMS, I2S_TEST_RESOLUTION);

	i2s_write_dma_example((uint8_t *)&tx_data_32bit, sizeof(tx_data_32bit));
	i2s_read_dma_example(sizeof(tx_data_32bit));

	i2s_close_example(0);
	i2s_close_example(1);

	i2s_deinit_example();
}


int main(void)
{
	printf("\n\n/*** i2s demo start ***/\n");

	i2s_test_example();

	printf("\n\n/*** i2c demo end ***/\n");
}


