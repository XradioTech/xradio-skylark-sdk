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
#include <stdlib.h>
#include <string.h>
#include "sys/defs.h"
#include "sys/list.h"
#include "kernel/os/os_mutex.h"
#include "audio/manager/audio_manager.h"


#define AUDIO_MANAGER_DEBUG_EN				0

#if AUDIO_MANAGER_DEBUG_EN
#define AUDIO_MANAGER_DEBUG(fmt, arg...)	printf("[AUDIO_MANAGER]"fmt, ##arg)
#else
#define AUDIO_MANAGER_DEBUG(fmt, arg...)
#endif

#define AUDIO_MANAGER_ERROR(fmt, arg...)	printf("[AUDIO_MANAGER]"fmt, ##arg)


#define audio_mgr_lock(m)           		OS_MutexLock(m, OS_WAIT_FOREVER)
#define audio_mgr_unlock         			OS_MutexUnlock
#define audio_mgr_lock_init           		OS_MutexCreate
#define audio_mgr_lock_deinit        		OS_MutexDelete


LIST_HEAD_DEF(audio_manager_list);

struct audio_manager_priv {
	Snd_Card_Num card_num;
	uint16_t     current_dev;
	OS_Mutex_t   lock;
	struct list_head node;
};


static void *audio_manager_zalloc(uint32_t size)
{
	void *p;
	if ((p = malloc(size)) != NULL){
		memset(p, 0, size);
		return p;
	}
	return NULL;
}

static void audio_manager_free(void *p)
{
	if(p){
		free(p);
		p = NULL;
	}
}

static struct audio_manager_priv *card_num_to_audio_manager(Snd_Card_Num card_num)
{
	struct audio_manager_priv *audio_mgr_priv;

	if(!list_empty(&audio_manager_list)){
		list_for_each_entry(audio_mgr_priv, &audio_manager_list, node){
			if(audio_mgr_priv->card_num == card_num){
				return audio_mgr_priv;
			}
		}
	}

	return NULL;
}

int audio_manager_get_current_dev(Snd_Card_Num card_num)
{
	struct audio_manager_priv *audio_mgr_priv;

	audio_mgr_priv = card_num_to_audio_manager(card_num);
	if(audio_mgr_priv == NULL){
		AUDIO_MANAGER_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return -1;
	}

	return audio_mgr_priv->current_dev;
}

int audio_manager_reg_read(Snd_Card_Num card_num, uint32_t reg)
{
	struct audio_manager_priv *audio_mgr_priv;

	audio_mgr_priv = card_num_to_audio_manager(card_num);
	if(audio_mgr_priv == NULL){
		AUDIO_MANAGER_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return -1;
	}

	return HAL_SndCard_CodecRegRead(card_num, reg);
}

int audio_manager_reg_write(Snd_Card_Num card_num, uint32_t reg, uint32_t val)
{
	struct audio_manager_priv *audio_mgr_priv;

	audio_mgr_priv = card_num_to_audio_manager(card_num);
	if(audio_mgr_priv == NULL){
		AUDIO_MANAGER_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return -1;
	}

	return HAL_SndCard_CodecRegWrite(card_num, reg, val);
}

int audio_maneger_ioctl(Snd_Card_Num card_num, Codec_Ioctl_Cmd cmd, uint32_t cmd_param[], uint32_t cmd_param_len)
{
	struct audio_manager_priv *audio_mgr_priv;
	AUDIO_MANAGER_DEBUG("--->%s\n",__FUNCTION__);

	audio_mgr_priv = card_num_to_audio_manager(card_num);
	if(audio_mgr_priv == NULL){
		AUDIO_MANAGER_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return -1;
	}

	return HAL_SndCard_Ioctl(card_num, cmd, cmd_param, cmd_param_len);
}

int audio_manager_handler(Snd_Card_Num card_num, Audio_Manager_Cmd cmd, Audio_Device dev, uint32_t param)
{
	int ret = -1;
	struct audio_manager_priv *audio_mgr_priv;
	AUDIO_MANAGER_DEBUG("--->%s\n",__FUNCTION__);

	audio_mgr_priv = card_num_to_audio_manager(card_num);
	if(audio_mgr_priv == NULL){
		AUDIO_MANAGER_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return -1;
	}

	audio_mgr_lock(&audio_mgr_priv->lock);

	switch(cmd){
		case AUDIO_MANAGER_SET_VOLUME_LEVEL:
			ret = HAL_SndCard_SetVolume(card_num, dev, ((uint16_t)param & ~VOLUME_SET_MASK) | VOLUME_SET_LEVEL);
			break;

		case AUDIO_MANAGER_SET_VOLUME_GAIN:
			ret = HAL_SndCard_SetVolume(card_num, dev, ((uint16_t)param & ~VOLUME_SET_MASK) | VOLUME_SET_GAIN);
			break;

		case AUDIO_MANAGER_SET_ROUTE:
			ret = HAL_SndCard_SetRoute(card_num, dev, (Audio_Dev_State)param);
			if(!ret){
				if((Audio_Dev_State)param == AUDIO_DEV_EN){
					audio_mgr_priv->current_dev |= dev;
				} else {
					audio_mgr_priv->current_dev &= ~dev;
				}
			}
			break;

		case AUDIO_MANAGER_SET_MUTE:
			ret = HAL_SndCard_SetMute(card_num, dev, (Audio_Mute_State)param);
			break;

		default:
			AUDIO_MANAGER_ERROR("Invalid audio manager command!\n");
			break;
	}

	audio_mgr_unlock(&audio_mgr_priv->lock);

	return ret;
}


int audio_manager_init(void)
{
	char *audio_mgr_temp;
	uint8_t *card_num, card_nums, i;
	struct audio_manager_priv *audio_mgr_priv;
	AUDIO_MANAGER_DEBUG("--->%s\n",__FUNCTION__);

	/* Get snd card nums */
	card_nums = HAL_SndCard_GetCardNums();
	if(!card_nums){
		AUDIO_MANAGER_ERROR("card nums is 0!\n");
		return -1;
	}

	/* Malloc buffer and init*/
	audio_mgr_temp = (char *)audio_manager_zalloc(sizeof(struct audio_manager_priv) * card_nums);
	card_num = (uint8_t *)audio_manager_zalloc(sizeof(uint8_t) * card_nums);

	if(!audio_mgr_temp || !card_num){
		AUDIO_MANAGER_ERROR("Malloc audio manager buffer Fail!\n");
		audio_manager_free(audio_mgr_temp);
		audio_manager_free(card_num);
		return -1;
	}

	HAL_SndCard_GetAllCardNum(card_num);

	/* Init struct audio_manager_priv and list add */
	for(i=0; i<card_nums; i++){
		//Init struct audio_manager_priv
		audio_mgr_priv = (struct audio_manager_priv *)(audio_mgr_temp + sizeof(struct audio_manager_priv)*i);
		audio_mgr_priv->card_num = (Snd_Card_Num)card_num[i];
		audio_mgr_lock_init(&audio_mgr_priv->lock);

		//list add
		list_add(&audio_mgr_priv->node, &audio_manager_list);
	}

	/* Free get card num buffer */
	audio_manager_free(card_num);

	return 0;
}

int audio_manager_deinit(void)
{
	AUDIO_MANAGER_DEBUG("--->%s\n",__FUNCTION__);
	struct audio_manager_priv *audio_mgr_priv, *next;

	/* Check audio manager list empty or not */
	if(list_empty(&audio_manager_list)){
		AUDIO_MANAGER_DEBUG(("Audio manager list is empty, don't need to deinit\n"));
		return 0;
	}

	/* Get audio manager priv to deinit */
	list_for_each_entry_safe(audio_mgr_priv, next, &audio_manager_list, node){
		//audio manager priv deinit
		audio_mgr_lock_deinit(&audio_mgr_priv->lock);

		//delete the audio manager list and free audio manager priv buffer
		list_del(&audio_mgr_priv->node);
		audio_manager_free(audio_mgr_priv);
	}

	return 0;
}


