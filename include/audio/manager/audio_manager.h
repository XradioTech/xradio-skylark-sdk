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

#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H


#include "driver/chip/hal_snd_card.h"


#ifdef __cplusplus
extern "C" {
#endif


//Audio default snd card num
#define AUDIO_SND_CARD_DEFAULT		SND_CARD_0


/* Audio manager command */
typedef enum {
	AUDIO_MANAGER_SET_VOLUME_LEVEL,
	AUDIO_MANAGER_SET_VOLUME_GAIN,
	AUDIO_MANAGER_SET_ROUTE,
	AUDIO_MANAGER_SET_MUTE,
	AUDIO_MANAGER_CMD_MAX = AUDIO_MANAGER_SET_MUTE,
} Audio_Manager_Cmd;


int audio_manager_init(void);
int audio_manager_deinit(void);
int audio_manager_handler(Snd_Card_Num card_num, Audio_Manager_Cmd cmd, Audio_Device dev, uint32_t param);
int audio_maneger_ioctl(Snd_Card_Num card_num, Codec_Ioctl_Cmd cmd, uint32_t cmd_param[], uint32_t cmd_param_len);
int audio_manager_reg_read(Snd_Card_Num card_num, uint32_t reg);
int audio_manager_reg_write(Snd_Card_Num card_num, uint32_t reg, uint32_t val);
int audio_manager_get_current_dev(Snd_Card_Num card_num);


#ifdef __cplusplus
}
#endif


#endif


