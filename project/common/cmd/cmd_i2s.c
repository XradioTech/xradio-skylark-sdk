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

#include "cmd_util.h"
#include "cmd_i2s.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_snd_card.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/manager/audio_manager.h"


//CMD I2S Test config
#define CMD_I2S_SND_CARD		SND_CARD_3
#define CMD_I2S_LOOP_BACK_EN	0

#define CMD_I2S_BUF_SIZE		3360//128

static unsigned char tx_sample_res, rx_sample_res;
#if 0
static const uint8_t tx_data_8bit[128] = {
	0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12,
	0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12,
	0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12,
	0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12,
	0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12,
	0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12,
	0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12,
	0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12,
};

static const uint16_t tx_data_16bit[64] = {
	0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234,
	0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234,
	0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234,
	0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234, 0x1234,
};

static const uint32_t tx_data_32bit[32] = {
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
	0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678, 0x12345678,
};
#endif

static enum cmd_status cmd_i2s_open_exec(char *cmd)
{
	int cnt;
	uint32_t direction, channels, resolution, sampleRate, cmd_param[3];

	cnt = cmd_sscanf(cmd, "d=%u c=%u r=%u s=%u", &direction, &channels, &resolution, &sampleRate);
	if (cnt != 4) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (direction != 0 && direction != 1) {
		CMD_ERR("invalid direction %u\n", direction);
		return CMD_STATUS_INVALID_ARG;
	}

	if (channels < 1 || channels > 8) {
		CMD_ERR("invalid channels %u\n", channels);
		return CMD_STATUS_INVALID_ARG;
	}

	switch (sampleRate) {
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
			CMD_ERR("invalid sample rate %u\n", sampleRate);
			return CMD_STATUS_INVALID_ARG;
	}

	if(!direction){
		tx_sample_res = resolution;
	} else {
		rx_sample_res = resolution;
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
			CMD_ERR("Invalid resolution(%u) failed...\n",resolution);
			return HAL_INVALID;
	}

	struct pcm_config pcm_cfg;
	pcm_cfg.rate = sampleRate;
	pcm_cfg.channels = channels;
	pcm_cfg.format = resolution;
	pcm_cfg.period_count = 1;
	pcm_cfg.period_size = CMD_I2S_BUF_SIZE/(pcm_format_to_bits(pcm_cfg.format)/8*channels)/pcm_cfg.period_count;

	cmd_param[0] = !!CMD_I2S_LOOP_BACK_EN<<24 | 0x0<<16 | 0x20<<8 | 0x2;
	cmd_param[1] = (channels+1)/2*32;
	cmd_param[2] = 24576000;
	audio_maneger_ioctl(CMD_I2S_SND_CARD, PLATFORM_IOCTL_HW_CONFIG, cmd_param, 3);

	cmd_param[0] = 256<<16 | 256;
	audio_maneger_ioctl(CMD_I2S_SND_CARD, PLATFORM_IOCTL_SW_CONFIG, cmd_param, 1);

	if (snd_pcm_open(CMD_I2S_SND_CARD, (Audio_Stream_Dir)direction, &pcm_cfg)) {
		CMD_ERR("snd pcm open Fail..\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_i2s_close_exec(char *cmd)
{
	int cnt;
	uint32_t direction;

	cnt = cmd_sscanf(cmd, "d=%u", &direction);
	if (cnt != 1) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (direction != 0 && direction != 1) {
		CMD_ERR("invalid direction %u\n", direction);
		return CMD_STATUS_INVALID_ARG;
	}

	if (snd_pcm_close(CMD_I2S_SND_CARD, (Audio_Stream_Dir)direction)) {
		CMD_ERR("Snd pcm close Fail..\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_i2s_pcm_write_exec()
{
	uint32_t i,*buf_32,bit_map;
	uint16_t *buf_16;
	uint8_t *buf, *buf_8;
	int32_t size;

	//malloc tx buffer
	buf = (uint8_t *)cmd_malloc(CMD_I2S_BUF_SIZE);
	if (buf == NULL) {
		CMD_ERR("cmd_malloc return NULL.\n");
		return CMD_STATUS_FAIL;
	}

	//copy tx data
	if(tx_sample_res > 16){
		buf_32 = (unsigned int *)buf;
		for(i=0; i<CMD_I2S_BUF_SIZE/4;i++){
			buf_32[i] = 0x12345678;
		}
		//cmd_memcpy(buf, tx_data_32bit, CMD_I2S_BUF_SIZE);
	} else if(tx_sample_res > 8){
		buf_16 = (uint16_t *)buf;
		for(i=0; i<CMD_I2S_BUF_SIZE/2;i++){
			buf_16[i] = 0x1234;
		}
		//cmd_memcpy(buf, tx_data_16bit, CMD_I2S_BUF_SIZE);
	} else {
		buf_8 = (uint8_t *)buf;
		for(i=0; i<CMD_I2S_BUF_SIZE/1;i++){
			buf_8[i] = 0x12;
		}
		//cmd_memcpy(buf, tx_data_8bit, CMD_I2S_BUF_SIZE);
	}

	//write pcm data
	size = snd_pcm_write(CMD_I2S_SND_CARD, buf, CMD_I2S_BUF_SIZE);
	if (size != CMD_I2S_BUF_SIZE) {
		CMD_ERR("len = %u, but I2S WRITE size = %d\n", CMD_I2S_BUF_SIZE, size);
		cmd_free(buf);
		cmd_write_respond(CMD_STATUS_FAIL, "FAIL");
		return CMD_STATUS_ACKED;
	}
	cmd_write_respond(CMD_STATUS_OK, "OK");

	//printf tx data for debug
	CMD_LOG(1, "\nwrite buf:\n");
	if(tx_sample_res > 16){
		bit_map = ((1<<tx_sample_res)-1)<<(32-tx_sample_res);
		buf_32 = (unsigned int *)buf;
		for(i=0; i<CMD_I2S_BUF_SIZE/4/8; i++){
			CMD_LOG(1, "0x%08x ", buf_32[i] & bit_map);
		}
	} else if(tx_sample_res > 8){
		bit_map = (1<<tx_sample_res)-1;
		buf_16 = (uint16_t *)buf;
		for(i=0; i<CMD_I2S_BUF_SIZE/2/16; i++){
			CMD_LOG(1, "0x%04x ", buf_16[i] & bit_map);
		}
	} else {
		bit_map = (1<<tx_sample_res)-1;
		buf_8 = (uint8_t *)buf;
		for(i=0; i<CMD_I2S_BUF_SIZE/1/32; i++){
			CMD_LOG(1, "0x%02x ", buf_8[i] & bit_map);
		}
	}
	CMD_LOG(1, "\n\n");

	//free tx buffer
	cmd_free(buf);

	return CMD_STATUS_ACKED;
}

static enum cmd_status cmd_i2s_pcm_read_exec()
{
	uint32_t i, *buf_32;
	uint16_t *buf_16;
	uint8_t *buf, *buf_8;
	int32_t size;

	//malloc rx buffer
	buf = (uint8_t *)cmd_malloc(CMD_I2S_BUF_SIZE);
	if (buf == NULL) {
		CMD_ERR("cmd_malloc return NULL.\n");
		return CMD_STATUS_FAIL;
	}
	cmd_memset(buf, 0, CMD_I2S_BUF_SIZE);

	//read pcm_data
	size = snd_pcm_read(CMD_I2S_SND_CARD, buf, CMD_I2S_BUF_SIZE);
	if (size != CMD_I2S_BUF_SIZE) {
		CMD_ERR("len = %u, but I2S read size = %d\n", CMD_I2S_BUF_SIZE, size);
		cmd_free(buf);
		return CMD_STATUS_FAIL;
	}
	cmd_write_respond(CMD_STATUS_OK, "OK");

	//printf rx_data for debug
	CMD_LOG(1, "\nread buf:\n");
	if(rx_sample_res > 16){
		buf_32 = (unsigned int *)buf;
		for(i=0; i<CMD_I2S_BUF_SIZE/4/8; i++){
			printf("0x%08x ", buf_32[i]);
		}
	} else if(rx_sample_res > 8){
		buf_16 = (uint16_t *)buf;
		for(i=0; i<CMD_I2S_BUF_SIZE/2/16; i++){
			printf("0x%04x ", buf_16[i]);
		}
	} else {
		buf_8 = (uint8_t *)buf;
		for(i=0; i<CMD_I2S_BUF_SIZE/1/32; i++){
			printf("0x%02x ", buf_8[i]);
		}
	}
	CMD_LOG(1, "\n");

	//free rx buffer
	cmd_free(buf);

	return CMD_STATUS_ACKED;
}


static const struct cmd_data g_i2s_cmds[] = {
	{ "open",		cmd_i2s_open_exec },
	{ "close",		cmd_i2s_close_exec },
	{ "pcm_read",	cmd_i2s_pcm_read_exec },
	{ "pcm_write",	cmd_i2s_pcm_write_exec },
};

enum cmd_status cmd_i2s_exec(char *cmd)
{
	return cmd_exec(cmd, g_i2s_cmds, cmd_nitems(g_i2s_cmds));
}


