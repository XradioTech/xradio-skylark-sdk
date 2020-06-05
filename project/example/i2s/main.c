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
#include <stdlib.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_snd_card.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/manager/audio_manager.h"
#include "common/framework/platform_init.h"



//I2S Test config
#define I2S_TEST_SND_CARD		SND_CARD_3
#define I2S_TEST_LOOP_BACK_EN	1

#define I2S_TEST_SAMPLE_RATE	16000	//[8000,16000,32000, 12000,24000,48000, 11025,22050,44100]
#define I2S_TEST_CHANNEL_NUMS	2		//[1,2]
#define I2S_TEST_RESOLUTION		32		//[8,12,16,20,24,28,32]


unsigned int tx_data_32bit[32] = {
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
};


static int i2s_open_example(uint8_t direction, uint32_t samplerate, uint8_t channels, uint8_t resolution)
{
	uint32_t cmd_param[3];

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
		case 16000:
		case 32000:

		case 12000:
		case 24000:
		case 48000:

		case 11025:
		case 22050:
		case 44100:
			break;

		default:
			printf("Invalid sample rate %u\n",samplerate);
			return HAL_INVALID;
	}

	switch(resolution){
		case 8:
			resolution = PCM_FORMAT_S8;
			break;
		case 12:
			resolution = PCM_FORMAT_S12_LE;
			break;
		case 16:
			resolution = PCM_FORMAT_S16_LE;
			break;
		case 20:
			resolution = PCM_FORMAT_S20_LE;
			break;
		case 24:
			resolution = PCM_FORMAT_S24_LE;
			break;
		case 28:
			resolution = PCM_FORMAT_S28_LE;
			break;
		case 32:
			resolution = PCM_FORMAT_S32_LE;
			break;

		default:
			printf("Invalid resolution %d\n",resolution);
			return HAL_INVALID;
	}

	struct pcm_config pcm_cfg;
	pcm_cfg.rate = samplerate;
	pcm_cfg.channels = channels;
	pcm_cfg.format = resolution;
	pcm_cfg.period_count = 1;
	pcm_cfg.period_size = sizeof(tx_data_32bit)/(pcm_format_to_bits(pcm_cfg.format)/8*channels)/pcm_cfg.period_count;

	cmd_param[0] = !!I2S_TEST_LOOP_BACK_EN<<24 | 0x0<<16 | 0x20<<8 | 0x2;
	cmd_param[1] = (channels+1)/2*32;
	cmd_param[2] = samplerate%1000 ? 22579200 : 24576000;
	audio_maneger_ioctl(I2S_TEST_SND_CARD, PLATFORM_IOCTL_HW_CONFIG, cmd_param, 3);

	cmd_param[0] = 256<<16 | 256;
	audio_maneger_ioctl(I2S_TEST_SND_CARD, PLATFORM_IOCTL_SW_CONFIG, cmd_param, 1);

	if (snd_pcm_open(I2S_TEST_SND_CARD, (Audio_Stream_Dir)direction, &pcm_cfg)) {
		printf("snd pcm open Fail..\n");
		return HAL_ERROR;
	}

	return HAL_OK;
}

static int i2s_close_example(uint8_t direction)
{
	if (direction != 0 && direction != 1) {
		printf("Invalid direction %d\n", direction);
		return HAL_INVALID;
	}

	if (snd_pcm_close(I2S_TEST_SND_CARD, (Audio_Stream_Dir)direction)) {
		printf("Snd pcm close Fail..\n");
		return HAL_ERROR;
	}

	return HAL_OK;
}

static int i2s_pcm_write_example(uint8_t *buffer, uint32_t size)
{
	uint8_t *buf;
	uint32_t i;

	if(!buffer || !size){
		printf("Invalid write buf|size params error!\n");
		return HAL_INVALID;
	}

	buf = (uint8_t *)malloc(size);
	if (buf == NULL) {
		printf("Malloc I2S write buffer Fail\n");
		return HAL_ERROR;
	}

	memcpy(buf, buffer, size);
	if(snd_pcm_write(I2S_TEST_SND_CARD, buf, size) != size){
		free(buf);
		printf("I2S write error!\n");
		return HAL_ERROR;
	}

	printf("\nwrite buf:\n");
	for(i=0; i<size; i++){
		printf("0x%02x ",buf[i]);
	}
	printf("\n\n");

	free(buf);
	return HAL_OK;
}

static int i2s_pcm_read_example(uint32_t size)
{
	uint8_t *buf;
	uint32_t i;

	if(!size){
		printf("Invalid read size params error!\n");
		return HAL_INVALID;
	}

	buf = (uint8_t *)malloc(size);
	if (buf == NULL) {
		printf("Malloc I2S read buffer Fail\n");
		return HAL_ERROR;
	}

	memset(buf, 0, size);
	if(snd_pcm_read(I2S_TEST_SND_CARD, buf, size) != size){
		free(buf);
		printf("I2S read error!\n");
		return HAL_ERROR;
	}

	printf("\nread buf:\n");
	for(i=0; i<size; i++)
		printf("0x%02x ", buf[i]);
	printf("\n\n");

	free(buf);
	return HAL_OK;
}


static void i2s_test_example(void)
{
	i2s_open_example(0, I2S_TEST_SAMPLE_RATE, I2S_TEST_CHANNEL_NUMS, I2S_TEST_RESOLUTION);
	i2s_open_example(1, I2S_TEST_SAMPLE_RATE, I2S_TEST_CHANNEL_NUMS, I2S_TEST_RESOLUTION);

	i2s_pcm_write_example((uint8_t *)&tx_data_32bit, sizeof(tx_data_32bit));
	i2s_pcm_read_example(sizeof(tx_data_32bit));

	i2s_close_example(0);
	i2s_close_example(1);
}


int main(void)
{
	platform_init();

	printf("\n\n/*** i2s demo start ***/\n");

	i2s_test_example();

	printf("\n\n/*** i2c demo end ***/\n");
}


