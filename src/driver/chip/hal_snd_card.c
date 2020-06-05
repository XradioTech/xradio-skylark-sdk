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

#include "codec/audio_arch.h"


//Debug config
#define HAL_SND_CARD_DBG_EN                	0

#define HAL_SND_CARD_DEBUG(fmt, arg...)    	HAL_LOG(HAL_SND_CARD_DBG_EN, "[HAL_SND_CARD] "fmt, ##arg)
#define HAL_SND_CARD_ERROR(fmt, arg...)    	HAL_LOG(1, "[HAL_SND_CARD] "fmt, ##arg)


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

HAL_Status HAL_SndCard_SetRoute(Snd_Card_Num card_num, Audio_Device dev, Audio_Dev_State state)
{
	HAL_Status hal_status;
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

	//set route
	if(sound_card->codec_drv && sound_card->codec_drv->dai_ops && sound_card->codec_drv->dai_ops->set_route){
		hal_status = sound_card->codec_drv->dai_ops->set_route(dev, state);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] codec set route Fail!\n",(uint8_t)card_num);
			HAL_MutexUnlock(&sound_card->card_lock);
			return HAL_ERROR;
		}
	}

#if 0
	//PA switch control
	if(sound_card->pa_switch_ctl && dev == AUDIO_OUT_DEV_SPK){
		//if(state == AUDIO_DEV_EN && sound_card->pa_switch_ctl->on_delay_before)	HAL_MSleep(sound_card->pa_switch_ctl->on_delay_before);
		HAL_GPIO_WritePin(sound_card->pa_switch_ctl->pin_param->port, sound_card->pa_switch_ctl->pin_param->pin,\
			state == AUDIO_DEV_EN ? sound_card->pa_switch_ctl->on_state : !sound_card->pa_switch_ctl->on_state);
		//if(state == AUDIO_DEV_EN && sound_card->pa_switch_ctl->on_delay_after)	HAL_MSleep(sound_card->pa_switch_ctl->on_delay_after);
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
	}

	HAL_MutexUnlock(&sound_card->card_lock);

	return HAL_OK;
}


HAL_Status HAL_SndCard_Ioctl(Snd_Card_Num card_num, Snd_Card_Ioctl_Cmd cmd, uint32_t cmd_param[], uint32_t cmd_param_len)
{
	HAL_Status ret = HAL_ERROR;
	struct snd_card *sound_card = card_num_to_snd_card(card_num);

	if(!sound_card){
		HAL_SND_CARD_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return HAL_INVALID;
	}

	HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

	if(cmd <= CODEC_IOCTL_CMD_MAX){
		if(sound_card->codec_drv && sound_card->codec_drv->codec_ops && sound_card->codec_drv->codec_ops->ioctl){
			ret = sound_card->codec_drv->codec_ops->ioctl(cmd, cmd_param, cmd_param_len);
		}
	} else {
		if(sound_card->platform_drv && sound_card->platform_drv->platform_ops && sound_card->platform_drv->platform_ops->ioctl){
			ret = sound_card->platform_drv->platform_ops->ioctl(cmd, cmd_param, cmd_param_len);
		}
	}

	HAL_MutexUnlock(&sound_card->card_lock);

	return ret;
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

	uint32_t dma_buf_size;
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

#if 0
	//codec ioctl hw_config place here
	if(sound_card->codec_drv && sound_card->codec_drv->codec_ops && sound_card->codec_drv->codec_ops->ioctl){
		uint32_t cmd_param_codec[] = {0x00};
		hal_status = sound_card->codec_drv->codec_ops->ioctl(CODEC_IOCTL_HW_CONFIG, cmd_param_codec, 1);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] codec IOCTL_HW_CONFIG Fail!\n",(uint8_t)card_num);
			HAL_MutexUnlock(&sound_card->card_lock);
			return HAL_ERROR;
		}
	}

	//platform ioctl hw_config place here
	if(sound_card->platform_drv && sound_card->platform_drv->platform_ops && sound_card->platform_drv->platform_ops->ioctl){
		uint32_t cmd_param_platform[3] = {0x00};
		hal_status = sound_card->platform_drv->platform_ops->ioctl(PLATFORM_IOCTL_HW_CONFIG, cmd_param_platform, 3);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] platform IOCTL_HW_CONFIG Fail!\n",(uint8_t)card_num);
			HAL_MutexUnlock(&sound_card->card_lock);
			return HAL_ERROR;
		}
	}
#endif

	/* Codec dai config */
	if(sound_card->codec_drv && sound_card->codec_drv->dai_ops){
		//codec set sysclk
		if(sound_card->codec_drv->dai_ops->set_sysclk){
			hal_status = sound_card->codec_drv->dai_ops->set_sysclk(sound_card->codec_sysclk_src,\
				sound_card->codec_pllclk_src, sound_card->codec_pll_freq_in, pcm_cfg->rate);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] codec set sysclk Fail!\n",(uint8_t)card_num);
				goto codec_hw_free;
			}
		}

		//codec set i2s fmt
		if(sound_card->codec_drv->dai_ops->set_fmt){
			hal_status = sound_card->codec_drv->dai_ops->set_fmt(sound_card->i2s_fmt);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] codec set fmt Fail!\n",(uint8_t)card_num);
				goto codec_hw_free;
			}
		}

		//codec hw params config
		if(sound_card->codec_drv->dai_ops->hw_params){
			hal_status = sound_card->codec_drv->dai_ops->hw_params(stream_dir, pcm_cfg);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] codec config hw params Fail!\n",(uint8_t)card_num);
				goto codec_hw_free;
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
				goto codec_hw_free;
			}
		}
	}

	/* Platform dai config */
	if(sound_card->platform_drv && sound_card->platform_drv->dai_ops){
		//platform set i2s fmt
		if(sound_card->platform_drv->dai_ops->set_fmt){
			hal_status = sound_card->platform_drv->dai_ops->set_fmt(sound_card->i2s_fmt);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] platform set fmt Fail!\n",(uint8_t)card_num);
				goto platform_hw_free;
			}
		}

		//platfrom set clkdiv
		if(sound_card->platform_drv->dai_ops->set_clkdiv){
			hal_status = sound_card->platform_drv->dai_ops->set_clkdiv(pcm_cfg->rate);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] platform set clkdiv Fail!\n",(uint8_t)card_num);
				goto platform_hw_free;
			}
		}

		//platform hw params config
		if(sound_card->platform_drv->dai_ops->hw_params){
			hal_status = sound_card->platform_drv->dai_ops->hw_params(stream_dir, pcm_cfg);
			if(hal_status != HAL_OK){
				HAL_SND_CARD_ERROR("snd card[%d] platform config hw params Fail!\n",(uint8_t)card_num);
				goto platform_hw_free;
			}
		}
	}

	/* Platform open */
	if(sound_card->platform_drv && sound_card->platform_drv->platform_ops && sound_card->platform_drv->platform_ops->open){
		hal_status = sound_card->platform_drv->platform_ops->open(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] platform-I2S open Fail!\n",(uint8_t)card_num);
			goto platform_hw_free;
		}
	}

	HAL_MutexUnlock(&sound_card->card_lock);

	/* PA unmute */
	if(stream_dir == PCM_OUT){
		if(sound_card->pa_switch_ctl){
			if(sound_card->pa_switch_ctl->on_delay_before)	HAL_MSleep(sound_card->pa_switch_ctl->on_delay_before);
			HAL_GPIO_WritePin(sound_card->pa_switch_ctl->pin_param->port, sound_card->pa_switch_ctl->pin_param->pin, sound_card->pa_switch_ctl->on_state);
			if(sound_card->pa_switch_ctl->on_delay_after)	HAL_MSleep(sound_card->pa_switch_ctl->on_delay_after);
		}
	}

	return HAL_OK;


platform_hw_free:
	if(sound_card->platform_drv && sound_card->platform_drv->dai_ops && sound_card->platform_drv->dai_ops->hw_free){
		hal_status = sound_card->platform_drv->dai_ops->hw_free(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] platform hw free Fail!\n",(uint8_t)card_num);
		}
	}

//codec_close:
	if(sound_card->codec_drv && sound_card->codec_drv->codec_ops && sound_card->codec_drv->codec_ops->close){
		hal_status = sound_card->codec_drv->codec_ops->close(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] codec close Fail!\n",(uint8_t)card_num);
		}
	}

codec_hw_free:
	if(sound_card->codec_drv && sound_card->codec_drv->dai_ops && sound_card->codec_drv->dai_ops->hw_free){
		hal_status = sound_card->codec_drv->dai_ops->hw_free(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] codec hw free Fail!\n",(uint8_t)card_num);
		}
	}

	HAL_MutexUnlock(&sound_card->card_lock);
	return HAL_ERROR;
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
		if(sound_card->pa_switch_ctl){
			HAL_GPIO_WritePin(sound_card->pa_switch_ctl->pin_param->port, sound_card->pa_switch_ctl->pin_param->pin, !sound_card->pa_switch_ctl->on_state);
		}
	}

	HAL_MutexLock(&sound_card->card_lock, OS_WAIT_FOREVER);

	/* Platform close */
	if(sound_card->platform_drv && sound_card->platform_drv->platform_ops && sound_card->platform_drv->platform_ops->close){
		hal_status = sound_card->platform_drv->platform_ops->close(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] platform close Fail!\n",(uint8_t)card_num);
		}
	}

	/* Codec close */
	if(sound_card->codec_drv && sound_card->codec_drv->codec_ops && sound_card->codec_drv->codec_ops->close){
		hal_status = sound_card->codec_drv->codec_ops->close(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] codec close Fail!\n",(uint8_t)card_num);
		}
	}

	/* Codec dai hw_free */
	if(sound_card->codec_drv && sound_card->codec_drv->dai_ops && sound_card->codec_drv->dai_ops->hw_free){
		hal_status = sound_card->codec_drv->dai_ops->hw_free(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] codec hw free Fail!\n",(uint8_t)card_num);
		}
	}

	/* Platform dai hw free */
	if(sound_card->platform_drv && sound_card->platform_drv->dai_ops && sound_card->platform_drv->dai_ops->hw_free){
		hal_status = sound_card->platform_drv->dai_ops->hw_free(stream_dir);
		if(hal_status != HAL_OK){
			HAL_SND_CARD_ERROR("snd card[%d] platform hw free Fail!\n",(uint8_t)card_num);
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
				sound_card->platform_drv->init();
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


uint8_t HAL_SndCard_Register(void)
{
	HAL_SND_CARD_DEBUG("--->%s\n",__FUNCTION__);

	HAL_Status hal_status;
	uint8_t i,found,card_nums=0;
	struct snd_card *sound_card;
	struct codec_driver *codec_drv_ptr = NULL;
	struct platform_driver *platform_drv_ptr = NULL;
	const struct snd_card_board_config *snd_card_board_cfg = NULL;

	/* Register snd card according to snd card board config */
	for(i=0; i<=SND_CARD_MAX; i++,snd_card_board_cfg = NULL){
		/* Get snd card board config */
		hal_status = HAL_BoardIoctl(HAL_BIR_GET_CFG, HAL_MKDEV(HAL_DEV_MAJOR_AUDIO_CODEC, i), (uint32_t)&snd_card_board_cfg);
		if(hal_status != HAL_OK || snd_card_board_cfg == NULL){
			continue;
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

		/* Match board config codec link */
		if(snd_card_board_cfg->codec_link == XRADIO_CODEC_NULL){
			codec_drv_ptr = NULL;
		} else {
			found = 0;
			if(!list_empty(&hal_snd_codec_list)){
				list_for_each_entry(codec_drv_ptr, &hal_snd_codec_list, node){
					if(snd_card_board_cfg->codec_link == codec_drv_ptr->codec_attr){
						found = 1;
						break;	//snd card match board config codec SUCCESS, break to continue register sound card
					}
				}
			}
			if(!found){
				HAL_SND_CARD_ERROR("snd card-[%d] match board config codec Fail!\n",snd_card_board_cfg->card_num);
				continue;
			}
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
						break;	//snd card match board config platform SUCCESS, break to continue register sound card
					}
				}
			}
			if(!found){
				HAL_SND_CARD_ERROR("snd card-[%d] match board config platform Fail!\n",snd_card_board_cfg->card_num);
				continue;
			}
		}

		/* Guarantee at least one codec or platform to be link when register snd card */
		if(codec_drv_ptr == NULL && platform_drv_ptr == NULL){
			HAL_SND_CARD_ERROR("snd card-[%d] must link one codec or platform at least!\n",snd_card_board_cfg->card_num);
			continue;
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
		sound_card->codec_sysclk_src = snd_card_board_cfg->codec_sysclk_src;
		sound_card->codec_pllclk_src = snd_card_board_cfg->codec_pllclk_src;
		sound_card->codec_pll_freq_in = snd_card_board_cfg->codec_pll_freq_in;
		sound_card->i2s_fmt = snd_card_board_cfg->i2s_fmt;

		sound_card->pa_switch_ctl = snd_card_board_cfg->pa_switch_ctl;
		HAL_BoardIoctl(HAL_BIR_PINMUX_INIT, HAL_MKDEV(HAL_DEV_MAJOR_AUDIO_CODEC, (uint8_t)sound_card->card_num), 0);

		if(sound_card->pa_switch_ctl){
			//PA switch init
			HAL_GPIO_WritePin(sound_card->pa_switch_ctl->pin_param->port,\
				sound_card->pa_switch_ctl->pin_param->pin, !sound_card->pa_switch_ctl->on_state);
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
			hal_status = sound_card->platform_drv->init();
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

HAL_Status HAL_SndCard_CodecRegisterAc101(void)
{
	return ac101_codec_register();
}

HAL_Status HAL_SndCard_CodecUnregisterAc101(void)
{
	return ac101_codec_unregister();
}

HAL_Status HAL_SndCard_PlatformRegisterI2S(void)
{
	return xradio_i2s_register();
}

HAL_Status HAL_SndCard_PlatformUnregisterI2S(void)
{
	return xradio_i2s_unregister();
}


