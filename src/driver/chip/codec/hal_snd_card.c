/**
  * @file  hal_codec.c
  * @author  XRADIO IOT WLAN Team
  */

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

#include "audio_arch.h"
#include "driver/chip/hal_i2s.h"


//Debug config
#define HAL_SND_CARD_DBG_EN                	0

#define HAL_SND_CARD_DEBUG(fmt, arg...)    	HAL_LOG(HAL_SND_CARD_DBG_EN, "[HAL_SND_CARD] "fmt, ##arg)
#define HAL_SND_CARD_ERROR(fmt, arg...)    	HAL_LOG(1, "[HAL_SND_CARD] "fmt, ##arg)

//snd card config
#define SND_CARD_DEFAULT_PLAY_DEV			AUDIO_OUT_DEV_SPK
#define SND_CARD_DEFAULT_RECORD_DEV			AUDIO_IN_DEV_AMIC


//snd card arch list
LIST_HEAD_DEF(hal_snd_codec_list);
LIST_HEAD_DEF(hal_snd_platform_list);
LIST_HEAD_DEF(hal_snd_card_list);


uint8_t pcm_format_to_bits(enum pcm_format format)
{
	switch (format) {
		case PCM_FORMAT_S8:
			return 8;

		case PCM_FORMAT_S12_LE:
		case PCM_FORMAT_S16_LE:
			return 16;

		case PCM_FORMAT_S20_LE:
		case PCM_FORMAT_S24_LE:
		case PCM_FORMAT_S28_LE:
		case PCM_FORMAT_S32_LE:
			return 32;

		default:
			HAL_SND_CARD_ERROR("invalid pcm format %d\n",(uint8_t)format);
			return 0;
	}
}

uint8_t pcm_format_to_sampleresolution(enum pcm_format format)
{
	switch (format) {
		case PCM_FORMAT_S8:		return 8;
		case PCM_FORMAT_S12_LE: return 12;
		case PCM_FORMAT_S16_LE: return 16;
		case PCM_FORMAT_S20_LE:	return 20;
		case PCM_FORMAT_S24_LE: return 24;
		case PCM_FORMAT_S28_LE: return 28;
		case PCM_FORMAT_S32_LE: return 32;
		default:
			HAL_SND_CARD_ERROR("invalid pcm format %d\n",(uint8_t)format);
			return 0;
	}
}

uint32_t pcm_config_to_frames(struct pcm_config *config)
{
	return config->period_count * config->period_size;
}

uint32_t pcm_frames_to_bytes(struct pcm_config *config, unsigned int frames)
{
	return frames * config->channels *(pcm_format_to_bits(config->format) >> 3);
}

__nonxip_text
static struct snd_card *card_num_to_snd_card(Snd_Card_Num card_num)
{
	struct snd_card *sound_card;

	if(!list_empty(&hal_snd_card_list)){
		list_for_each_entry(sound_card, &hal_snd_card_list, node){
			if(sound_card->card_num == card_num)
				return sound_card;
		}
	}

	return NULL;
}

uint8_t HAL_SndCard_GetCardNums(void)
{
	uint8_t card_nums=0;
	struct snd_card *sound_card;

	if(!list_empty(&hal_snd_card_list)){
		list_for_each_entry(sound_card, &hal_snd_card_list, node){
			card_nums++;
		}
	}

	return card_nums;
}

void HAL_SndCard_GetAllCardNum(uint8_t card_num[])
{
	struct snd_card *sound_card;

	if(!list_empty(&hal_snd_card_list)){
		list_for_each_entry(sound_card, &hal_snd_card_list, node){
			*card_num++ = (uint8_t)sound_card->card_num;
		}
	}
}

int HAL_SndCard_CodecRegRead(Snd_Card_Num card_num, uint32_t reg)
{
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	if(sound_card->codec_drv && sound_card->codec_drv->codec_ops && sound_card->codec_drv->codec_ops->reg_read){
		return sound_card->codec_drv->codec_ops->reg_read(reg);
	}

	return HAL_ERROR;
}

int HAL_SndCard_CodecRegWrite(Snd_Card_Num card_num, uint32_t reg, uint32_t val)
{
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	if(sound_card->codec_drv && sound_card->codec_drv->codec_ops && sound_card->codec_drv->codec_ops->reg_write){
		return sound_card->codec_drv->codec_ops->reg_write(reg, val);
	}

	return HAL_ERROR;
}

HAL_Status HAL_SndCard_SetVolume(Snd_Card_Num card_num, Audio_Device dev, uint16_t volume)
{
	HAL_Status hal_status;
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

	//save snd card play/record volume that will be used in snd card open
	if(dev & AUDIO_IN_DEV_ALL){
		sound_card->codec_record_vol = volume;
	} else if (dev & AUDIO_OUT_DEV_ALL){
		sound_card->codec_play_vol = volume;
	}

	//set volume level or gain
	if(sound_card->codec_drv && sound_card->codec_drv->dai_ops && sound_card->codec_drv->dai_ops->set_volume){
		hal_status = sound_card->codec_drv->dai_ops->set_volume(dev, volume);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] codec set volume Fail!\n",(uint8_t)card_num);
			HAL_MutexUnlock(&sound_card->card_lock);
			return HAL_ERROR;
		}
	}

	HAL_MutexUnlock(&sound_card->card_lock);

	return HAL_OK;
}

__nonxip_text
HAL_Status HAL_SndCard_SetRoute(Snd_Card_Num card_num, Audio_Device dev, Audio_Dev_State state)
{
	HAL_Status hal_status;
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		//HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

	//save snd card play/record device that will be used in snd card open
	if(dev & AUDIO_IN_DEV_ALL){
		if(state == AUDIO_DEV_EN){
			if(sound_card->codec_record_dev & AUDIO_DEFAULT_DEV_MASK){
				sound_card->codec_record_dev = dev;
			} else {
				sound_card->codec_record_dev |= dev;
			}
		} else {
			sound_card->codec_record_dev &= ~dev;
		}
	} else if (dev & AUDIO_OUT_DEV_ALL){
		if(state == AUDIO_DEV_EN){
			if(sound_card->codec_play_dev & AUDIO_DEFAULT_DEV_MASK){
				sound_card->codec_play_dev = dev;
			} else {
				sound_card->codec_play_dev |= dev;
			}
		} else {
			sound_card->codec_play_dev &= ~dev;
		}
	}

	//set route
	if(sound_card->codec_drv && sound_card->codec_drv->dai_ops && sound_card->codec_drv->dai_ops->set_route){
		hal_status = sound_card->codec_drv->dai_ops->set_route(dev, state);
		if(hal_status != HAL_OK){
			//HAL_SND_CARD_ERROR("snd card[%d] codec set route Fail!\n",(uint8_t)card_num);
			HAL_MutexUnlock(&sound_card->card_lock);
			return HAL_ERROR;
		}
	}

#if 0
	//PA switch control
	if(sound_card->pa_switch_ctl && dev == AUDIO_OUT_DEV_SPK){
		HAL_GPIO_WritePin(sound_card->pa_switch_ctl->pin_param->port, sound_card->pa_switch_ctl->pin_param->pin,\
			state == AUDIO_DEV_EN ? sound_card->pa_switch_ctl->on_state : !sound_card->pa_switch_ctl->on_state);
		//if(state == AUDIO_DEV_EN)	HAL_MSleep(sound_card->pa_switch_ctl->on_delay);
	}
#endif

	HAL_MutexUnlock(&sound_card->card_lock);

	return HAL_OK;
}

HAL_Status HAL_SndCard_SetMute(Snd_Card_Num card_num, Audio_Device dev, Audio_Mute_State mute)
{
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

	//PA switch control
	if(sound_card->pa_switch_ctl && (dev | AUDIO_OUT_DEV_SPK)){
		HAL_GPIO_WritePin(sound_card->pa_switch_ctl->pin_param->port, sound_card->pa_switch_ctl->pin_param->pin,\
			mute == AUDIO_MUTE ? !sound_card->pa_switch_ctl->on_state : sound_card->pa_switch_ctl->on_state);
		//if(mute == AUDIO_UNMUTE)	HAL_MSleep(sound_card->pa_switch_ctl->on_delay);
	}

	HAL_MutexUnlock(&sound_card->card_lock);

	return HAL_OK;
}


HAL_Status HAL_SndCard_Ioctl(Snd_Card_Num card_num, Codec_Ioctl_Cmd cmd, uint32_t cmd_param[], uint32_t cmd_param_len)
{
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

	if(sound_card->codec_drv && sound_card->codec_drv->codec_ops && sound_card->codec_drv->codec_ops->ioctl){
		return sound_card->codec_drv->codec_ops->ioctl(cmd, cmd_param, cmd_param_len);
	}

	HAL_MutexUnlock(&sound_card->card_lock);

	return HAL_ERROR;
}



int HAL_SndCard_PcmRead(Snd_Card_Num card_num, uint8_t *buf, uint32_t size)
{
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	if(sound_card->platform_drv && sound_card->platform_drv->platform_ops && sound_card->platform_drv->platform_ops->pcm_read){
		return sound_card->platform_drv->platform_ops->pcm_read(buf, size);
	} else if (sound_card->codec_drv && sound_card->codec_drv->codec_ops && sound_card->codec_drv->codec_ops->ioctl){
		uint32_t cmd_param[2] = {(uint32_t)buf, size};
		return sound_card->codec_drv->codec_ops->ioctl(CODEC_IOCTL_PCM_READ, cmd_param, 2);
	}

	return 0;
}

int HAL_SndCard_PcmWrite(Snd_Card_Num card_num, uint8_t *buf, uint32_t size)
{
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	if(sound_card->platform_drv && sound_card->platform_drv->platform_ops && sound_card->platform_drv->platform_ops->pcm_write){
		return sound_card->platform_drv->platform_ops->pcm_write(buf, size);
	} else if (sound_card->codec_drv && sound_card->codec_drv->codec_ops && sound_card->codec_drv->codec_ops->ioctl){
		uint32_t cmd_param[2] = {(uint32_t)buf, size};
		return sound_card->codec_drv->codec_ops->ioctl(CODEC_IOCTL_PCM_WRITE, cmd_param, 2);
	}

	return 0;
}

HAL_Status HAL_SndCard_Open(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir, struct pcm_config *pcm_cfg)
{
	HAL_SND_CARD_DEBUG("--->%s\n",__FUNCTION__);

	uint32_t i,dma_buf_size;
	HAL_Status hal_status;
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	if(!pcm_cfg || !pcm_cfg->rate){
		HAL_SND_CARD_ERROR("snd card[%d] open pcm_config params Error!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	/* Init hardware DMA buffer size */
	dma_buf_size = pcm_frames_to_bytes(pcm_cfg, pcm_config_to_frames(pcm_cfg));
	if(!dma_buf_size){
		HAL_SND_CARD_ERROR("snd card[%d] open dma buf size is 0!\n",(uint8_t)card_num);
		return HAL_ERROR;
	}

	HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

	/* Codec dai config */
	if(sound_card->codec_drv && sound_card->codec_drv->dai_ops){
		//codec set sysclk
		if(sound_card->codec_drv->dai_ops->set_sysclk){
			hal_status = sound_card->codec_drv->dai_ops->set_sysclk(sound_card->codec_sysclk_src,\
				sound_card->codec_pllclk_src, sound_card->codec_pll_freq_in, pcm_cfg->rate);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] codec set sysclk Fail!\n",(uint8_t)card_num);
				HAL_MutexUnlock(&sound_card->card_lock);
				return HAL_ERROR;
			}
		}

		//codec set i2s fmt
		if(sound_card->codec_drv->dai_ops->set_fmt){
			hal_status = sound_card->codec_drv->dai_ops->set_fmt(sound_card->i2s_fmt);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] codec set fmt Fail!\n",(uint8_t)card_num);
				HAL_MutexUnlock(&sound_card->card_lock);
				return HAL_ERROR;
			}
		}

		//codec hw params config
		if(sound_card->codec_drv->dai_ops->hw_params){
			hal_status = sound_card->codec_drv->dai_ops->hw_params(stream_dir, pcm_cfg);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] codec config hw params Fail!\n",(uint8_t)card_num);
				HAL_MutexUnlock(&sound_card->card_lock);
				return HAL_ERROR;
			}
		}

		//codec set volume & route
		if(sound_card->codec_drv->dai_ops->set_volume || sound_card->codec_drv->dai_ops->set_route){
			uint32_t route_dev,codec_dev,codec_vol;

			if(stream_dir == PCM_OUT){
				codec_dev = sound_card->codec_play_dev;
				codec_vol = sound_card->codec_play_vol;
			} else {
				codec_dev = sound_card->codec_record_dev;
				codec_vol = sound_card->codec_record_vol;
			}

			for(i=0,route_dev=1; i<31; i++,route_dev=1<<i){	//ignore AUDIO_DEFAULT_DEV_MASK BIT
				if(codec_dev & route_dev){
					if(sound_card->codec_drv->dai_ops->set_volume && route_dev != AUDIO_IN_DEV_DMIC){
						hal_status = sound_card->codec_drv->dai_ops->set_volume(route_dev, codec_vol);
						if(hal_status != HAL_OK){
							HAL_SND_CARD_ERROR("snd card[%d] codec set volume Fail!\n",(uint8_t)card_num);
							HAL_MutexUnlock(&sound_card->card_lock);
							return HAL_ERROR;
						}
					}

					if(sound_card->codec_drv->dai_ops->set_route){
						hal_status = sound_card->codec_drv->dai_ops->set_route(route_dev, AUDIO_DEV_EN);
						if(hal_status != HAL_OK){
							HAL_SND_CARD_ERROR("snd card[%d] codec set route Fail!\n",(uint8_t)card_num);
							HAL_MutexUnlock(&sound_card->card_lock);
							return HAL_ERROR;
						}
					}
				}
			}
		}
	}

	/* Codec open */
	if(sound_card->codec_drv && sound_card->codec_drv->codec_ops){
		//codec ioctl
		//related Codec Ioctl place here

		//codec open
		if(sound_card->codec_drv->codec_ops->open){
			hal_status = sound_card->codec_drv->codec_ops->open(stream_dir);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] codec open Fail!\n",(uint8_t)card_num);
				HAL_MutexUnlock(&sound_card->card_lock);
				return HAL_ERROR;
			}
		}
	}

	/* Platform open */
	if(sound_card->platform_drv && sound_card->platform_drv->platform_ops && sound_card->platform_drv->platform_ops->open){
		//I2S open
		I2S_DataParam i2s_config;
		i2s_config.direction = stream_dir;
		i2s_config.sampleRate = pcm_cfg->rate;
		i2s_config.channels = pcm_cfg->channels;
		i2s_config.resolution = pcm_cfg->format;
		i2s_config.bufSize = dma_buf_size;

		hal_status = sound_card->platform_drv->platform_ops->open(&i2s_config);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] platform-I2S open Fail!\n",(uint8_t)card_num);
			HAL_MutexUnlock(&sound_card->card_lock);
			return HAL_ERROR;
		}
	}

	HAL_MutexUnlock(&sound_card->card_lock);

	/* PA unmute */
	if(stream_dir == PCM_OUT){
		if(sound_card->pa_switch_ctl && (sound_card->codec_play_dev | AUDIO_OUT_DEV_SPK)){
			HAL_GPIO_WritePin(sound_card->pa_switch_ctl->pin_param->port, sound_card->pa_switch_ctl->pin_param->pin, sound_card->pa_switch_ctl->on_state);
			//HAL_MSleep(sound_card->pa_switch_ctl->on_delay);
		}
	}

	return HAL_OK;
}

HAL_Status HAL_SndCard_Close(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir)
{
	HAL_SND_CARD_DEBUG("--->%s\n",__FUNCTION__);

	HAL_Status hal_status;
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	/* PA mute */
	if(stream_dir == PCM_OUT){
		if(sound_card->pa_switch_ctl && (sound_card->codec_play_dev | AUDIO_OUT_DEV_SPK)){
			HAL_GPIO_WritePin(sound_card->pa_switch_ctl->pin_param->port, sound_card->pa_switch_ctl->pin_param->pin, !sound_card->pa_switch_ctl->on_state);
		}
	}

	HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

	/* Platform close */
	if(sound_card->platform_drv && sound_card->platform_drv->platform_ops && sound_card->platform_drv->platform_ops->close){
		hal_status = sound_card->platform_drv->platform_ops->close(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] platform close Fail!\n",(uint8_t)card_num);
			HAL_MutexUnlock(&sound_card->card_lock);
			return HAL_ERROR;
		}
	}

	/* Codec close */
	if(sound_card->codec_drv && sound_card->codec_drv->codec_ops && sound_card->codec_drv->codec_ops->close){
		hal_status = sound_card->codec_drv->codec_ops->close(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] codec close Fail!\n",(uint8_t)card_num);
			HAL_MutexUnlock(&sound_card->card_lock);
			return HAL_ERROR;
		}
	}

	/* Codec dai hw_free */
	if(sound_card->codec_drv && sound_card->codec_drv->dai_ops && sound_card->codec_drv->dai_ops->hw_free){
		hal_status = sound_card->codec_drv->dai_ops->hw_free(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] codec hw free Fail!\n",(uint8_t)card_num);
			HAL_MutexUnlock(&sound_card->card_lock);
			return HAL_ERROR;
		}
	}

	HAL_MutexUnlock(&sound_card->card_lock);

	return HAL_OK;
}


#ifdef CONFIG_PM
static struct snd_card *soc_device_to_snd_card(struct soc_device *dev)
{
	struct snd_card *sound_card;

	if(!list_empty(&hal_snd_card_list)){
		list_for_each_entry(sound_card, &hal_snd_card_list, node){
			if(sound_card->pm_dev == dev)
				return sound_card;
		}
	}

	return NULL;
}

static int snd_card_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	struct snd_card *sound_card = soc_device_to_snd_card(dev);

	if(sound_card == NULL){
		HAL_SND_CARD_ERROR("Suspend snd_card pointer NULL\n");
		return HAL_ERROR;
	}

	switch(state){
		case PM_MODE_SLEEP:
		case PM_MODE_STANDBY:
		case PM_MODE_HIBERNATION:
			HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

			if(sound_card->codec_drv && sound_card->codec_drv->deinit){
				sound_card->codec_drv->deinit();
			}
			if(sound_card->platform_drv && sound_card->platform_drv->deinit){
				sound_card->platform_drv->deinit();
			}
			HAL_BoardIoctl(HAL_BIR_PINMUX_DEINIT, HAL_MKDEV(HAL_DEV_MAJOR_AUDIO_CODEC, sound_card->card_num), 0);

			HAL_MutexUnlock(&sound_card->card_lock);
			break;

		default:
			break;
	}

	return HAL_OK;
}

static int snd_card_resume(struct soc_device *dev, enum suspend_state_t state)
{
	struct snd_card *sound_card = soc_device_to_snd_card(dev);

	if(sound_card == NULL){
		HAL_SND_CARD_ERROR("Resume snd_card pointer NULL\n");
		return HAL_ERROR;
	}

	switch(state){
		case PM_MODE_SLEEP:
		case PM_MODE_STANDBY:
		case PM_MODE_HIBERNATION:
			HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

			if(sound_card->codec_drv && sound_card->codec_drv->init){
				sound_card->codec_drv->init();
			}
			if(sound_card->platform_drv && sound_card->platform_drv->init){
				//I2S init
				I2S_Param i2s_param;
				memset(&i2s_param, 0, sizeof(i2s_param));
				i2s_param.mclkDiv = 1;

				sound_card->platform_drv->init(&i2s_param);
			}
			HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_AUDIO_CODEC, sound_card->card_num), 0);

			HAL_MutexUnlock(&sound_card->card_lock);
			break;

		default:
			break;
	}

	return HAL_OK;
}

static const struct soc_device_driver snd_card_pm_drv = {
	.name = "snd_card_pm_drv",
	.suspend = snd_card_suspend,
	.resume = snd_card_resume,
};
#endif

__nonxip_text
static void linein_irq_callback(void *arg)
{
	struct snd_card *sound_card = (struct snd_card *)arg;

	if(sound_card->linein_detect_ctl){
		GPIO_PinState state = HAL_GPIO_ReadPin(sound_card->linein_detect_ctl->pin_param->port, sound_card->linein_detect_ctl->pin_param->pin);
		HAL_SndCard_SetRoute(sound_card->card_num, AUDIO_IN_DEV_LINEIN, state);
	}
}


uint8_t HAL_SndCard_Register(void)
{
	HAL_SND_CARD_DEBUG("--->%s\n",__FUNCTION__);

	HAL_Status hal_status;
	uint8_t i,found,card_nums=0;
	struct snd_card *sound_card;
	struct codec_driver *codec_drv_ptr;
	struct platform_driver *platform_drv_ptr = NULL;
	const struct snd_card_board_config *snd_card_board_cfg = NULL;

	/* Check Codec list empty or not */
	if(list_empty(&hal_snd_codec_list)){
		HAL_SND_CARD_ERROR("Hal snd codec list is empty, snd card register Fail!\n");
		return 0;
	}

	/* Get codec_drv to register snd card */
	list_for_each_entry(codec_drv_ptr, &hal_snd_codec_list, node){
		/* Match board config codec link */
		for(i=0; i<=SND_CARD_MAX; i++){
			hal_status = HAL_BoardIoctl(HAL_BIR_GET_CFG, HAL_MKDEV(HAL_DEV_MAJOR_AUDIO_CODEC, i), (uint32_t)&snd_card_board_cfg);
			if(hal_status == HAL_OK && snd_card_board_cfg && snd_card_board_cfg->codec_link == codec_drv_ptr->codec_attr){
				break;		//snd card codec match board config codec SUCCESS, break to continue register sound card
			}
		}
		if(i>SND_CARD_MAX){
			HAL_SND_CARD_DEBUG("snd card codec-[%d] match board config codec Fail!\n",codec_drv_ptr->codec_attr);
			continue;
		}

		/* Match board config platform link */
		if(snd_card_board_cfg->platform_link == XRADIO_PLATFORM_NULL){
			platform_drv_ptr = NULL;
		} else {
			found = 0;
			if(!list_empty(&hal_snd_platform_list)){
				list_for_each_entry(platform_drv_ptr, &hal_snd_platform_list, node){
					if(snd_card_board_cfg->platform_link == platform_drv_ptr->platform_attr){
						found = 1;
						break;	//snd card codec match board config platform SUCCESS, break to continue register sound card
					}
				}
			}
			if(!found){
				HAL_SND_CARD_ERROR("snd card codec-[%d] match board config platform Fail!\n",codec_drv_ptr->codec_attr);
				continue;
			}
		}

		/* Check the card num to be valid and unique */
		if(snd_card_board_cfg->card_num > SND_CARD_MAX){
			HAL_SND_CARD_ERROR("Invalid snd card number-[%d], while the max card number is %d!\n",snd_card_board_cfg->card_num,SND_CARD_MAX);
			continue;
		}
		if(!list_empty(&hal_snd_card_list)){
			found = 0;
			list_for_each_entry(sound_card, &hal_snd_card_list, node){
				if(sound_card->card_num == snd_card_board_cfg->card_num){
					found = 1;
					break;
				}
			}
			if(found){
				HAL_SND_CARD_ERROR("The snd card number-[%d] has registered\n",snd_card_board_cfg->card_num);
				continue;
			}
		}

		/* Malloc snd card buffer */
		sound_card = (struct snd_card *)HAL_Malloc(sizeof(struct snd_card));
		if(!sound_card){
			HAL_SND_CARD_ERROR("Malloc snd card-[%d] struct snd_card buffer Fail!\n",snd_card_board_cfg->card_num);
			break;//continue;
		}
		HAL_Memset(sound_card, 0, sizeof(struct snd_card));

		/* Bind codec & platform driver  */
		sound_card->codec_drv = codec_drv_ptr;
		sound_card->platform_drv = platform_drv_ptr;

		/* Card lock init */
		if(sound_card->card_lock.handle == NULL){
			hal_status = HAL_MutexInit(&sound_card->card_lock);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] mutex init Fail\n", (uint8_t)sound_card->card_num);
				HAL_Free(sound_card);
				continue;
			}
		}

		/* Board config init */
		sound_card->card_num = snd_card_board_cfg->card_num;
		sound_card->card_name = snd_card_board_cfg->card_name;
		sound_card->codec_play_vol = VOLUME_INVALID;
		sound_card->codec_record_vol = VOLUME_INVALID;
		sound_card->codec_play_dev = SND_CARD_DEFAULT_PLAY_DEV | AUDIO_DEFAULT_DEV_MASK;
		sound_card->codec_record_dev = SND_CARD_DEFAULT_RECORD_DEV | AUDIO_DEFAULT_DEV_MASK;
		sound_card->codec_sysclk_src = snd_card_board_cfg->codec_sysclk_src;
		sound_card->codec_pllclk_src = snd_card_board_cfg->codec_pllclk_src;
		sound_card->codec_pll_freq_in = snd_card_board_cfg->codec_pll_freq_in;
		sound_card->i2s_fmt = snd_card_board_cfg->i2s_fmt;

		sound_card->pa_switch_ctl = snd_card_board_cfg->pa_switch_ctl;
		sound_card->linein_detect_ctl = snd_card_board_cfg->linein_detect_ctl;

		if(sound_card->pa_switch_ctl || sound_card->linein_detect_ctl){
			HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_AUDIO_CODEC, (uint8_t)sound_card->card_num), 0);

			//PA switch init
			if(sound_card->pa_switch_ctl){
				HAL_GPIO_WritePin(sound_card->pa_switch_ctl->pin_param->port,\
					sound_card->pa_switch_ctl->pin_param->pin, !sound_card->pa_switch_ctl->on_state);
			}

			//linein detect init
			if(sound_card->linein_detect_ctl){
				GPIO_IrqParam irq_param;
				irq_param.event = GPIO_IRQ_EVT_BOTH_EDGE;
				irq_param.callback = linein_irq_callback;
				irq_param.arg = sound_card;
				HAL_GPIO_EnableIRQ(sound_card->linein_detect_ctl->pin_param->port,\
					sound_card->linein_detect_ctl->pin_param->pin, &irq_param);
			}
		}

		/* Codec init */
		if(sound_card->codec_drv && sound_card->codec_drv->init){
			hal_status = sound_card->codec_drv->init();
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] codec init Fail\n", (uint8_t)sound_card->card_num);
				HAL_Free(sound_card);
				continue;
			}
		}

		/* Platform init */
		if(sound_card->platform_drv && sound_card->platform_drv->init){
			//I2S init
			I2S_Param i2s_param;
			memset(&i2s_param, 0, sizeof(i2s_param));
			i2s_param.mclkDiv = 2;

			hal_status = sound_card->platform_drv->init(&i2s_param);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] platform-I2S init Fail\n", (uint8_t)sound_card->card_num);
				HAL_Free(sound_card);
				continue;
			}
		}

#ifdef CONFIG_PM
		/* PM init */
		sound_card->pm_dev = (struct soc_device *)HAL_Malloc(sizeof(struct soc_device));
		if(sound_card->pm_dev == NULL){
			HAL_SND_CARD_ERROR("Malloc snd card-[%d] pm_dev buffer Fail!\n",sound_card->card_num);
			HAL_Free(sound_card);
			break;//continue;
		}
		HAL_Memset(sound_card->pm_dev, 0, sizeof(struct soc_device));

		sound_card->pm_dev->name = sound_card->card_name;
		sound_card->pm_dev->driver = &snd_card_pm_drv;
		pm_register_ops(sound_card->pm_dev);
#endif

		/* Snd card list add */
		list_add(&sound_card->node, &hal_snd_card_list);
		card_nums++;

		HAL_SND_CARD_DEBUG("/*** Register snd card[%d]->%s Success ***\\^_^\n\n",sound_card->card_num,sound_card->card_name);
	}

	return card_nums;
}

HAL_Status HAL_SndCard_Unregister(void)
{
	HAL_SND_CARD_DEBUG("--->%s\n",__FUNCTION__);

	struct snd_card *sound_card,*next;

	/* Check snd card list empty or not */
	if(list_empty(&hal_snd_card_list)){
		HAL_SND_CARD_DEBUG("Hal snd card list is empty, don't needt to unregister\n");
		return HAL_OK;
	}

	/* Get snd card to unregister */
	list_for_each_entry_safe(sound_card, next, &hal_snd_card_list, node){

		HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

		/* Codec deinit */
		if(sound_card->codec_drv && sound_card->codec_drv->deinit){
			sound_card->codec_drv->deinit();
		}

		/* Platform deinit */
		if(sound_card->platform_drv && sound_card->platform_drv->deinit){
			sound_card->platform_drv->deinit();
		}

		/* Board config deinit */
		if(sound_card->linein_detect_ctl){
			HAL_GPIO_DisableIRQ(sound_card->linein_detect_ctl->pin_param->port, sound_card->linein_detect_ctl->pin_param->pin);
		}

		HAL_MutexUnlock(&sound_card->card_lock);

		/* Card lock deinit */
		if(sound_card->card_lock.handle){
			HAL_MutexDeinit(&sound_card->card_lock);
		}

#ifdef CONFIG_PM
		/* PM deinit */
		if(sound_card->pm_dev){
			pm_unregister_ops(sound_card->pm_dev);
			HAL_Free(sound_card->pm_dev);
		}
#endif

		/* delete the snd card list and Free sound_card buffer */
		list_del(&sound_card->node);
		HAL_Free(sound_card);
	}

	return HAL_OK;
}


HAL_Status HAL_SndCard_CodecRegisterInternal(void)
{
	return xradio_internal_codec_register();
}

HAL_Status HAL_SndCard_CodecUnregisterInternal(void)
{
	return xradio_internal_codec_unregister();
}

HAL_Status HAL_SndCard_CodecRegisterAc107(void)
{
	return ac107_codec_register();
}

HAL_Status HAL_SndCard_CodecUnregisterAc107(void)
{
	return ac107_codec_unregister();
}

HAL_Status HAL_SndCard_PlatformRegisterI2S(void)
{
	return xradio_i2s_register();
}

HAL_Status HAL_SndCard_PlatformUnregisterI2S(void)
{
	return xradio_i2s_unregister();
}


