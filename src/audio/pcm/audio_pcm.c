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
#include "driver/chip/hal_i2s.h"
#include "driver/chip/hal_dmic.h"
#include "driver/chip/hal_i2c.h"
#include "audio/pcm/audio_pcm.h"
#include "kernel/os/os_mutex.h"


#define AUDIO_PCM_DEBUG_EN				0


#if AUDIO_PCM_DEBUG_EN
#define AUDIO_PCM_DEBUG(fmt, arg...)	printf("[AUDIO_PCM]"fmt, ##arg)
#else
#define AUDIO_PCM_DEBUG(fmt, arg...)
#endif

#define AUDIO_PCM_ERROR(fmt, arg...)	printf("[AUDIO_PCM]"fmt, ##arg)


#define pcm_lock(m)         			OS_MutexLock(m, OS_WAIT_FOREVER)
#define pcm_unlock       				OS_MutexUnlock
#define pcm_lock_init    				OS_MutexCreate
#define pcm_lock_deinit					OS_MutexDelete


LIST_HEAD_DEF(audio_pcm_list);

struct play_priv {
	uint8_t	 *cache;
	uint32_t length;
	uint32_t half_buf_size;
};

struct cap_priv {
	uint32_t half_buf_size;
};

struct pcm_priv {
	//card num
	Snd_Card_Num card_num;

	//play/cap priv
	struct play_priv play_priv;
	struct cap_priv  cap_priv;

	//mutex
	OS_Mutex_t play_lock;
	OS_Mutex_t cap_lock;
	OS_Mutex_t write_lock;

	//list node
	struct list_head node;
};


void* pcm_zalloc(uint32_t size)
{
	void *p;
	if ((p = malloc(size)) != NULL){
		memset(p, 0, size);
		return p;
	}
	return NULL;
}

void pcm_free(void *p)
{
	if(p){
		free(p);
		p = NULL;
	}
}

static struct pcm_priv *card_num_to_pcm_priv(Snd_Card_Num card_num)
{
	struct pcm_priv *audio_pcm_priv;

	if(!list_empty(&audio_pcm_list)){
		list_for_each_entry(audio_pcm_priv, &audio_pcm_list, node){
			if(audio_pcm_priv->card_num == card_num){
				return audio_pcm_priv;
			}
		}
	}

	return NULL;
}


int snd_pcm_read(Snd_Card_Num card_num, void *data, uint32_t count)
{
	struct pcm_priv *audio_pcm_priv;
	uint32_t half_buf_size, read_size;

	/* Check parms to be valid */
	if(!data || !count){
		AUDIO_PCM_ERROR("Invalid data/count params!\n");
		return -1;
	}

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if(audio_pcm_priv == NULL){
		AUDIO_PCM_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return -1;
	}

	/* Calculate read_size */
	half_buf_size = audio_pcm_priv->cap_priv.half_buf_size;
	if(count < half_buf_size)	return -1;
	read_size = (count / half_buf_size) * half_buf_size;

	/* Pcm read */
	return HAL_SndCard_PcmRead(card_num, data, read_size);
}

int snd_pcm_write(Snd_Card_Num card_num, void *data, uint32_t count)
{
	int ret;
	uint8_t *data_ptr;
	struct play_priv *ppriv;
	struct pcm_priv *audio_pcm_priv;
	uint32_t  half_buf_size, write_size, cache_remain=0;

	/* Check parms to be valid */
	if(!data || !count){
		AUDIO_PCM_ERROR("Invalid data/count params!\n");
		return -1;
	}

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if(audio_pcm_priv == NULL){
		AUDIO_PCM_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return -1;
	}

	/* Check play cache */
	if(audio_pcm_priv->play_priv.cache == NULL){
		AUDIO_PCM_ERROR("Play Cache is NULL!\n");
		return -1;
	}

	/* Get write lock */
	if (pcm_lock(&audio_pcm_priv->write_lock) != OS_OK) {
		AUDIO_PCM_ERROR("Obtain write lock err.\n");
		return -1;
	}

	/* Init misc */
	data_ptr = data;
	write_size = count;
	ppriv = &audio_pcm_priv->play_priv;
	half_buf_size = audio_pcm_priv->play_priv.half_buf_size;

	/* Cache has data to write */
	if (ppriv->length) {
		cache_remain = half_buf_size - ppriv->length;
		if (cache_remain > write_size) {
			memcpy(ppriv->cache + ppriv->length, data_ptr, write_size);
			ppriv->length += write_size;
			pcm_unlock(&audio_pcm_priv->write_lock);
			return count;
		} else {
			memcpy(ppriv->cache + ppriv->length, data_ptr, cache_remain);
			HAL_SndCard_PcmWrite(card_num, ppriv->cache, half_buf_size);
			ppriv->length = 0;
			data_ptr += cache_remain;
			write_size -= cache_remain;
			//if(!write_size)	return count;
		}
	}

	/* Pcm write */
	ret = HAL_SndCard_PcmWrite(card_num, data_ptr, write_size);
	if (ret>0 && ret<write_size) {	//remain some data that not enough half_buf_size data to write, save to cache
		data_ptr += ret;
		ppriv->length = write_size - ret;
		memcpy(ppriv->cache, data_ptr, ppriv->length);
	} else if (ret == write_size) {	//all data write complete
		ppriv->length = 0;
	} else if(ret == 0) {			//ret=0, not enough half_buf_size data to write, save to cache
		ppriv->length = write_size<half_buf_size ? write_size : half_buf_size;
		memcpy(ppriv->cache, data_ptr, ppriv->length);
	} else {						//ret<0, write fail
		pcm_unlock(&audio_pcm_priv->write_lock);
		return cache_remain ? cache_remain : -1;
	}

	pcm_unlock(&audio_pcm_priv->write_lock);
	return count;
}

int snd_pcm_flush(Snd_Card_Num card_num)
{
	uint32_t i, half_buf_size;
	struct play_priv *ppriv;
	struct pcm_priv *audio_pcm_priv;
	AUDIO_PCM_DEBUG("--->%s\n",__FUNCTION__);

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if(audio_pcm_priv == NULL){
		AUDIO_PCM_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return -1;
	}

	/* Check play cache */
	if(audio_pcm_priv->play_priv.cache == NULL){
		AUDIO_PCM_ERROR("Play Cache is NULL!\n");
		return -1;
	}

	/* Get write lock */
	if (pcm_lock(&audio_pcm_priv->write_lock) != OS_OK) {
		AUDIO_PCM_ERROR("Obtain write lock err.\n");
		return -1;
	}

	/* Init misc */
	ppriv = &audio_pcm_priv->play_priv;
	half_buf_size = audio_pcm_priv->play_priv.half_buf_size;

	/* Cache has data to write */
	if(ppriv->length){
		memset(ppriv->cache + ppriv->length, 0, half_buf_size - ppriv->length);
		HAL_SndCard_PcmWrite(card_num, ppriv->cache, half_buf_size);
		ppriv->length = 0;
	}

	/* play void frames */
	memset(ppriv->cache, 0, half_buf_size);
	for(i=0; i<2; i++) {
		HAL_SndCard_PcmWrite(card_num, ppriv->cache, half_buf_size);
	}

	pcm_unlock(&audio_pcm_priv->write_lock);
	return 0;
}

int snd_pcm_open(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir, struct pcm_config *pcm_cfg)
{
	uint32_t buf_size;
	struct pcm_priv *audio_pcm_priv;
	AUDIO_PCM_DEBUG("--->%s\n",__FUNCTION__);

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if(audio_pcm_priv == NULL){
		AUDIO_PCM_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return -1;
	}

	/* Open snd card */
	if (HAL_SndCard_Open(card_num, stream_dir, pcm_cfg) != HAL_OK) {
		AUDIO_PCM_ERROR("Sound card-[%d] open Fai!\n",card_num);
		return -1;
	}

	/* Init audio_pcm_priv */
	if (stream_dir == PCM_OUT) {
		//play lock
		if (pcm_lock(&audio_pcm_priv->play_lock) != OS_OK) {
			AUDIO_PCM_ERROR("Obtain play lock err...\n");
			return -1;
		}

		// Malloc play cache buffer
		buf_size = pcm_frames_to_bytes(pcm_cfg, pcm_config_to_frames(pcm_cfg));
		audio_pcm_priv->play_priv.cache = pcm_zalloc(buf_size/2);
		if (audio_pcm_priv->play_priv.cache == NULL) {
			pcm_unlock(&audio_pcm_priv->play_lock);
			AUDIO_PCM_ERROR("obtain play cache failed...\n");
			return -1;
		}
		audio_pcm_priv->play_priv.length = 0;
		audio_pcm_priv->play_priv.half_buf_size = buf_size/2;
	} else {
		//cap lock
		if (pcm_lock(&audio_pcm_priv->cap_lock) != OS_OK) {
			AUDIO_PCM_ERROR("obtain cap lock err...\n");
			return -1;
		}
		audio_pcm_priv->cap_priv.half_buf_size = pcm_frames_to_bytes(pcm_cfg, pcm_config_to_frames(pcm_cfg))/2;
	}

	return 0;
}

int snd_pcm_close(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir)
{
	struct pcm_priv *audio_pcm_priv;
	AUDIO_PCM_DEBUG("--->%s\n",__FUNCTION__);

	/* Get audio_pcm_priv */
	audio_pcm_priv = card_num_to_pcm_priv(card_num);
	if(audio_pcm_priv == NULL){
		AUDIO_PCM_ERROR("Invalid sound card num [%d]!\n",(uint8_t)card_num);
		return -1;
	}

	/* Close snd card */
	HAL_SndCard_Close(card_num, stream_dir);

	/* deinit audio_pcm_priv */
	if (stream_dir == PCM_OUT) {
		pcm_free(audio_pcm_priv->play_priv.cache);
		memset(&(audio_pcm_priv->play_priv), 0, sizeof(struct play_priv));
		pcm_unlock(&audio_pcm_priv->play_lock);
	} else {
		memset(&(audio_pcm_priv->cap_priv), 0, sizeof(struct cap_priv));
		pcm_unlock(&audio_pcm_priv->cap_lock);
	}

	return 0;
}

int snd_pcm_init(void)
{
	char *pcm_priv_temp;
	uint8_t *card_num, card_nums, i;
	struct pcm_priv* audio_pcm_priv;
	AUDIO_PCM_DEBUG("--->%s\n",__FUNCTION__);

	/* Get snd card nums */
	card_nums = HAL_SndCard_GetCardNums();
	if(!card_nums){
		AUDIO_PCM_ERROR("card nums is 0!\n");
		return -1;
	}

	/* Malloc buffer and init */
	pcm_priv_temp = (char  *)pcm_zalloc(sizeof(struct pcm_priv) * card_nums);
	card_num = (uint8_t *)pcm_zalloc(sizeof(uint8_t) * card_nums);

	if(!pcm_priv_temp || !card_num){
		AUDIO_PCM_ERROR("Malloc audio pcm buffer Fail!\n");
		pcm_free(pcm_priv_temp);
		pcm_free(card_num);
		return -1;
	}

	HAL_SndCard_GetAllCardNum(card_num);

	/* Init struct pcm_priv and list add */
	for(i=0; i<card_nums; i++){
		//Init struct pcm_priv
		audio_pcm_priv = (struct pcm_priv *)(pcm_priv_temp + sizeof(struct pcm_priv)*i);
		audio_pcm_priv->card_num = (Snd_Card_Num)card_num[i];
		pcm_lock_init(&audio_pcm_priv->play_lock);
		pcm_lock_init(&audio_pcm_priv->cap_lock);
		pcm_lock_init(&audio_pcm_priv->write_lock);

		//list add
		list_add(&audio_pcm_priv->node, &audio_pcm_list);
	}

	/* Free get card num buffer */
	pcm_free(card_num);

	return 0;
}

int snd_pcm_deinit(void)
{
	struct pcm_priv *audio_pcm_priv, *next;
	AUDIO_PCM_DEBUG("--->%s\n",__FUNCTION__);

	/* Check audio pcm list empty or not */
	if(list_empty(&audio_pcm_list)){
		AUDIO_PCM_DEBUG("Audio pcm list is empty, don't need to deinit\n");
		return 0;
	}

	/* Get audio pcm priv to deinit */
	list_for_each_entry_safe(audio_pcm_priv, next, &audio_pcm_list, node){
		//audio pcm priv deinit
		pcm_lock_deinit(&audio_pcm_priv->play_lock);
		pcm_lock_deinit(&audio_pcm_priv->cap_lock);
		pcm_lock_deinit(&audio_pcm_priv->write_lock);

		//delete the audio pcm list and free audio pcm priv buffer
		list_del(&audio_pcm_priv->node);
		pcm_free(audio_pcm_priv);
	}

	return 0;
}


