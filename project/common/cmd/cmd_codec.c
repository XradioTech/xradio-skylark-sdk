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

#include "fft/fft.h"
#include "cmd_util.h"
#include "cmd_codec.h"
#include "cmd_codec_dat.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/manager/audio_manager.h"


#define CMD_CODEC_SND_CARD			AUDIO_SND_CARD_DEFAULT

#define FFT_DEBUG_EN				0
#define RECORD_PCM_BUF_SIZE_48K		(768)		//max: 48*2*4 * 2 = 768
#define RECORD_PCM_BUF_SIZE_44K		(176400/2)	//max: 11025*2*4 * 2 = 176400


struct Cmd_Codec_Priv{
	u32 play_sample_rate;
	u8  play_channels;
	u8  play_sample_res;

	u32 rec_sample_rate;
	u8  rec_channels;
	u8  rec_sample_res;
} cmd_codec_priv ;

FFT_RESULT Cooley_Tukey_FFT(__s32 *data, __u32 fs);

/* sample_rate,  channels,  sample_resolution */
const unsigned char *play_dat[9][2][2] = {
	{{play_8k16bit1ch,  play_8k32bit1ch},  {play_8k16bit2ch,  play_8k32bit2ch}},
	{{play_11k16bit1ch, play_11k32bit1ch}, {play_11k16bit2ch, play_11k32bit2ch}},
	{{play_12k16bit1ch, play_12k32bit1ch}, {play_12k16bit2ch, play_12k32bit2ch}},
	{{play_16k16bit1ch, play_16k32bit1ch}, {play_16k16bit2ch, play_16k32bit2ch}},
	{{play_22k16bit1ch, play_22k32bit1ch}, {play_22k16bit2ch, play_22k32bit2ch}},
	{{play_24k16bit1ch, play_24k32bit1ch}, {play_24k16bit2ch, play_24k32bit2ch}},
	{{play_32k16bit1ch, play_32k32bit1ch}, {play_32k16bit2ch, play_32k32bit2ch}},
	{{play_44k16bit1ch, play_44k32bit1ch}, {play_44k16bit2ch, play_44k32bit2ch}},
	{{play_48k16bit1ch, play_48k32bit1ch}, {play_48k16bit2ch, play_48k32bit2ch}},
};

/* sample_rate,  channels,  sample_resolution */
const uint16_t play_dat_size[9][2][2] = {
	{{sizeof(play_8k16bit1ch),  sizeof(play_8k32bit1ch)},  {sizeof(play_8k16bit2ch),  sizeof(play_8k32bit2ch)}},
	{{sizeof(play_11k16bit1ch), sizeof(play_11k32bit1ch)}, {sizeof(play_11k16bit2ch), sizeof(play_11k32bit2ch)}},
	{{sizeof(play_12k16bit1ch), sizeof(play_12k32bit1ch)}, {sizeof(play_12k16bit2ch), sizeof(play_12k32bit2ch)}},
	{{sizeof(play_16k16bit1ch), sizeof(play_16k32bit1ch)}, {sizeof(play_16k16bit2ch), sizeof(play_16k32bit2ch)}},
	{{sizeof(play_22k16bit1ch), sizeof(play_22k32bit1ch)}, {sizeof(play_22k16bit2ch), sizeof(play_22k32bit2ch)}},
	{{sizeof(play_24k16bit1ch), sizeof(play_24k32bit1ch)}, {sizeof(play_24k16bit2ch), sizeof(play_24k32bit2ch)}},
	{{sizeof(play_32k16bit1ch), sizeof(play_32k32bit1ch)}, {sizeof(play_32k16bit2ch), sizeof(play_32k32bit2ch)}},
	{{sizeof(play_44k16bit1ch), sizeof(play_44k32bit1ch)}, {sizeof(play_44k16bit2ch), sizeof(play_44k32bit2ch)}},
	{{sizeof(play_48k16bit1ch), sizeof(play_48k32bit1ch)}, {sizeof(play_48k16bit2ch), sizeof(play_48k32bit2ch)}},
};


static uint8_t sample_rate_to_num(uint16_t sample_rate)
{
	switch (sample_rate) {
		case 8000:  return 0;
		case 11025: return 1;
		case 12000: return 2;
		case 16000: return 3;
		case 22050: return 4;
		case 24000: return 5;
		case 32000: return 6;
		case 44100: return 7;
		case 48000: return 8;
		default:
			CMD_ERR("invalid sample rate %u\n", sample_rate);
			return 0;
	}
}

static enum cmd_status cmd_codec_open_exec(char *cmd)
{
	int cnt;
	uint32_t direction, channels, resolution, sampleRate;

	cnt = cmd_sscanf(cmd, "d=%u c=%u r=%u s=%u", &direction, &channels, &resolution, &sampleRate);
	if (cnt != 4) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (direction != 0 && direction != 1) {
		CMD_ERR("invalid direction %u\n", direction);
		return CMD_STATUS_INVALID_ARG;
	}

	if (channels != 1 && channels != 2) {
		CMD_ERR("invalid channels %u\n", channels);
		return CMD_STATUS_INVALID_ARG;
	}

	if (resolution != 16 && resolution != 24) {
		CMD_ERR("invalid resolution %u\n", resolution);
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

	struct pcm_config pcm_cfg;
	pcm_cfg.rate = sampleRate;
	pcm_cfg.channels = channels;
	pcm_cfg.format = (resolution == 16) ? PCM_FORMAT_S16_LE : PCM_FORMAT_S24_LE;
	pcm_cfg.period_count = 1;

	if(direction == PCM_IN){
		pcm_cfg.period_size = (sampleRate%1000? RECORD_PCM_BUF_SIZE_44K : RECORD_PCM_BUF_SIZE_48K)\
			/(pcm_format_to_bits(pcm_cfg.format)/8*channels)/pcm_cfg.period_count;
		audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_VOLUME_GAIN, AUDIO_IN_DEV_AMIC, VOLUME_GAIN_0dB);
		audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_ROUTE,		 AUDIO_IN_DEV_AMIC, AUDIO_DEV_EN);
		if(channels == 2){
			audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_VOLUME_GAIN, AUDIO_IN_DEV_LINEIN, VOLUME_GAIN_0dB);
			audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_ROUTE,		 AUDIO_IN_DEV_LINEIN, AUDIO_DEV_EN);
		}
		cmd_codec_priv.rec_sample_rate = sampleRate;
		cmd_codec_priv.rec_channels = channels;
		cmd_codec_priv.rec_sample_res = resolution;
	} else {
		pcm_cfg.period_size = play_dat_size[sample_rate_to_num(sampleRate)][channels-1][resolution/8-2]\
			/(pcm_format_to_bits(pcm_cfg.format)/8*channels)/pcm_cfg.period_count;
		audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_VOLUME_GAIN, AUDIO_OUT_DEV_SPK, VOLUME_GAIN_0dB);
		audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_ROUTE,		 AUDIO_OUT_DEV_SPK, AUDIO_DEV_EN);
		cmd_codec_priv.play_sample_rate = sampleRate;
		cmd_codec_priv.play_channels = channels;
		cmd_codec_priv.play_sample_res = resolution;
	}

	if (snd_pcm_open(CMD_CODEC_SND_CARD, (Audio_Stream_Dir)direction, &pcm_cfg)) {
		CMD_ERR("snd pcm open Fail..\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_codec_close_exec(char *cmd)
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

	if(direction == PCM_IN){
		audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_ROUTE, AUDIO_IN_DEV_AMIC, AUDIO_DEV_DIS);
		audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_ROUTE, AUDIO_IN_DEV_LINEIN, AUDIO_DEV_DIS);
	} else {
		audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_ROUTE, AUDIO_OUT_DEV_SPK, AUDIO_DEV_DIS);
	}

	if (snd_pcm_close(CMD_CODEC_SND_CARD, (Audio_Stream_Dir)direction)) {
		CMD_ERR("Snd pcm close Fail..\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_codec_pcm_write_exec()
{
	uint8_t *buf;
	int32_t size;
	uint32_t len = play_dat_size[sample_rate_to_num(cmd_codec_priv.play_sample_rate)][cmd_codec_priv.play_channels-1][cmd_codec_priv.play_sample_res/8-2];

	buf = (uint8_t *)cmd_malloc(len);
	if (buf == NULL) {
		CMD_ERR("cmd_malloc return NULL.\n");
		return CMD_STATUS_FAIL;
	}

	cmd_write_respond(CMD_STATUS_OK, "OK");
	cmd_memcpy(buf, play_dat[sample_rate_to_num(cmd_codec_priv.play_sample_rate)][cmd_codec_priv.play_channels-1][cmd_codec_priv.play_sample_res/8-2], len);

	size = snd_pcm_write(CMD_CODEC_SND_CARD, buf, len);
	if (size != len) {
		CMD_ERR("len = %u, but snd pcm write size = %d\n", len, size);
		cmd_free(buf);
		cmd_write_respond(CMD_STATUS_FAIL, "FAIL");
		return CMD_STATUS_ACKED;
	}

	cmd_free(buf);
	cmd_write_respond(CMD_STATUS_OK, "OK");

	return CMD_STATUS_ACKED;
}

static enum cmd_status cmd_codec_pcm_read_exec()
{
	int i;
	uint8_t *buf;
	int32_t size;
	uint32_t len = cmd_codec_priv.rec_sample_rate%1000? RECORD_PCM_BUF_SIZE_44K : RECORD_PCM_BUF_SIZE_48K;

	buf = (uint8_t *)cmd_malloc(len);
	if (buf == NULL) {
		CMD_ERR("cmd malloc return NULL.\n");
		return CMD_STATUS_FAIL;
	}

	cmd_memset(buf, 0, len);

	size = snd_pcm_read(CMD_CODEC_SND_CARD, buf, len);
	if (size != len) {
		CMD_ERR("len = %u, but snd pcm read size = %d\n", len, size);
		cmd_free(buf);
		return CMD_STATUS_FAIL;
	}

	cmd_write_respond(CMD_STATUS_OK, "OK");


	/********************************** FFT Analyse **********************************/
	#define FFT_POINTS		1024

	FFT_RESULT fft_result;
	uint16_t *buf_u16 = (uint16_t *)buf;
	uint32_t *buf_u32 = (uint32_t *)buf;
	uint32_t frame_cnt = len/(cmd_codec_priv.rec_sample_res == 16 ? 2 : 4)/cmd_codec_priv.rec_channels;
	s32 *fft_dat = (s32 *)cmd_malloc(FFT_POINTS*4);

#if FFT_DEBUG_EN
	printf("read original data:\n");
	for(i=0; i<len/4; i++)
		printf("0x%08x ", buf_u32[i]);
	printf("\n");
#endif

	//cmd_memcpy(buf, buf+len/2, len/2);

#if FFT_DEBUG_EN
	printf("\n");
	for(i=0; i<len; i++){
		printf("%02x ",buf[i]);
	}
	printf("\n");
#endif

	if(fft_dat == NULL){
		CMD_ERR("\nMalloc FFT DAT buffer Fail\n\n");
	} else {
		/* AMIC FFT analyse */
		cmd_memset(fft_dat, 0, FFT_POINTS*4);
		//printf("AMIC FFT buf:\n");

		if(cmd_codec_priv.rec_sample_res == 16){
			if(cmd_codec_priv.rec_channels == 1){
				for(i=0; i<FFT_POINTS; i++){
					fft_dat[i] = buf_u16[i%frame_cnt];
					//printf("0x%08x\n",fft_dat[i]);
				}
			} else if(cmd_codec_priv.rec_channels == 2) {
				for(i=0; i<FFT_POINTS; i++){
					fft_dat[i] = buf_u16[2*(i%frame_cnt)];
					//printf("0x%08x\n",fft_dat[i]);
				}
			}

		} else if(cmd_codec_priv.rec_sample_res == 24) {

			if(cmd_codec_priv.rec_channels == 1){
				for(i=0; i<FFT_POINTS; i++){
					fft_dat[i] = buf_u32[i%frame_cnt]>>16;
					//printf("0x%08x\n",fft_dat[i]);
				}
			} else if(cmd_codec_priv.rec_channels == 2) {
				for(i=0; i<FFT_POINTS; i++){
					fft_dat[i] = buf_u32[2*(i%frame_cnt)]>>16;
					//printf("0x%08x\n",fft_dat[i]);
				}
			}
		}

		fft_result = Cooley_Tukey_FFT(fft_dat, cmd_codec_priv.rec_sample_rate);
		printf("\nAMIC FFT Frequency: %f KHz, SNR: %f dB\n\n",fft_result.sig_freq/1000,fft_result.sig_power-fft_result.noise_power);


		/* LINEIN FFT analyse */
		if(cmd_codec_priv.rec_channels == 2) {
			cmd_memset(fft_dat, 0, FFT_POINTS*4);
			//printf("LINEIN FFT buf:\n");

			if(cmd_codec_priv.rec_sample_res == 16){
				for(i=0; i<FFT_POINTS; i++){
					fft_dat[i] = buf_u16[2*(i%frame_cnt)+1];
					//printf("0x%08x\n",fft_dat[i]);
				}
			} else if(cmd_codec_priv.rec_sample_res == 24) {
				for(i=0; i<FFT_POINTS; i++){
					fft_dat[i] = buf_u32[2*(i%frame_cnt)+1]>>16;
					//printf("0x%08x\n",fft_dat[i]);
				}
			}

			fft_result = Cooley_Tukey_FFT(fft_dat, cmd_codec_priv.rec_sample_rate);
			printf("\nLINEIN FFT Frequency: %f KHz, SNR: %f dB\n\n",fft_result.sig_freq/1000,fft_result.sig_power-fft_result.noise_power);
		}

		cmd_free(fft_dat);
	}
	/*********************************************************************************/

	cmd_free(buf);
	return CMD_STATUS_ACKED;
}

static enum cmd_status cmd_codec_set_route_exec(char *cmd)
{
	int cnt;
	uint32_t route,enable;
	HAL_Status hal_status;
	Audio_Device audio_device;

	cnt = cmd_sscanf(cmd, "r=%u e=%u", &route, &enable);
	if (cnt != 2) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	switch(route){
		case 0:
			audio_device = AUDIO_IN_DEV_AMIC;
			break;
		case 1:
			audio_device = AUDIO_IN_DEV_LINEIN;
			break;
		case 2:
			audio_device = AUDIO_IN_DEV_DMIC;
			break;
		case 3:
			audio_device = AUDIO_OUT_DEV_SPK;
			break;
		default:
			CMD_ERR("invalid route %u\n", route);
			return CMD_STATUS_INVALID_ARG;
	}

	if(enable != 0 && enable != 1){
		CMD_ERR("invalid enable %u\n", enable);
		return CMD_STATUS_INVALID_ARG;
	}

	hal_status = audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_ROUTE, audio_device, (Audio_Dev_State)enable);
	if (hal_status == HAL_OK) {
		return CMD_STATUS_OK;
	} else {
		CMD_ERR("Codec set route Fail, return: hal_status = %d\n", hal_status);
		return CMD_STATUS_FAIL;
	}
}

static enum cmd_status cmd_codec_set_gain_exec(char *cmd)
{
	int cnt;
	uint32_t route,vol_level;
	HAL_Status hal_status;
	Audio_Device audio_device;

	cnt = cmd_sscanf(cmd, "r=%u g=%u", &route, &vol_level);
	if (cnt != 2) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	switch(route){
		case 0:
			audio_device = AUDIO_IN_DEV_AMIC;
			break;
		case 1:
			audio_device = AUDIO_IN_DEV_LINEIN;
			break;
		case 3:
			audio_device = AUDIO_OUT_DEV_SPK;
			break;
		case 2:
		default:
			CMD_ERR("invalid route %u\n", route);
			return CMD_STATUS_INVALID_ARG;
	}

	if(route == 3){
		if(vol_level<0 || vol_level>31){
			CMD_ERR("invalid vol_level %u\n", vol_level);
			return CMD_STATUS_INVALID_ARG;
		}
	} else {
		if(vol_level<0 || vol_level>7){
			CMD_ERR("invalid vol_level %u\n", vol_level);
			return CMD_STATUS_INVALID_ARG;
		}
	}

	hal_status = audio_manager_handler(CMD_CODEC_SND_CARD, AUDIO_MANAGER_SET_VOLUME_LEVEL, audio_device, (Volume_Level)vol_level);
	if (hal_status == HAL_OK) {
		return CMD_STATUS_OK;
	} else {
		CMD_ERR("Codec set vol_level Fail, return: hal_status = %d\n", hal_status);
		return CMD_STATUS_FAIL;
	}
}

static enum cmd_status cmd_codec_set_adda_exec(char *cmd)
{
	int cnt;
	uint32_t route,enable;
	HAL_Status hal_status;
	Audio_Device audio_device;

	cnt = cmd_sscanf(cmd, "r=%u e=%u", &route, &enable);
	if (cnt != 2) {
		CMD_ERR("cmd_sscanf return: cnt = %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	switch(route){
		case 0:
			audio_device = AUDIO_IN_DEV_AMIC;
			break;
		case 1:
			audio_device = AUDIO_IN_DEV_LINEIN;
			break;
		case 2:
			audio_device = AUDIO_IN_DEV_DMIC;
			break;
		default:
			CMD_ERR("invalid route %u\n", route);
			return CMD_STATUS_INVALID_ARG;
	}

	if(enable != 0 && enable != 1){
		CMD_ERR("invalid enable %u\n", enable);
		return CMD_STATUS_INVALID_ARG;
	}

	uint32_t cmd_param[2] = {audio_device, (Audio_Dev_State)enable};
	hal_status = audio_maneger_ioctl(CMD_CODEC_SND_CARD, CODEC_IOCTL_SET_ADDA_DIRECT, cmd_param, 2);
	if (hal_status == HAL_OK) {
		return CMD_STATUS_OK;
	} else {
		CMD_ERR("Codec_set_adda Fail, return: hal_status = %d\n", hal_status);
		return CMD_STATUS_FAIL;
	}
}


static const struct cmd_data g_codec_cmds[] = {
	{ "open",			cmd_codec_open_exec },
	{ "close",			cmd_codec_close_exec },
	{ "pcm_read",		cmd_codec_pcm_read_exec },
	{ "pcm_write",		cmd_codec_pcm_write_exec },
	{ "set_route",		cmd_codec_set_route_exec},
	{ "set_gain",		cmd_codec_set_gain_exec},
	{ "set_adda",		cmd_codec_set_adda_exec},
};

enum cmd_status cmd_codec_exec(char *cmd)
{
	return cmd_exec(cmd, g_codec_cmds, cmd_nitems(g_codec_cmds));
}


