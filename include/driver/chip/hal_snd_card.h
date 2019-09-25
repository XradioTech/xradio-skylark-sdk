/**
  * @file  hal_codec.h
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

#ifndef _HAL_SND_CARD_H_
#define _HAL_SND_CARD_H_


#include "driver/chip/hal_def.h"
#include "driver/chip/hal_gpio.h"


#ifdef __cplusplus
 extern "C" {
#endif


/* Codec name */
#define XRADIO_INTERNAL_CODEC_NAME		"xradio_internal_codec"
#define AC107_CODEC_NAME				"ac107_codec"

/* Platform name */
#define XRADIO_PLATFORM_I2S_NAME		"xradio_platform_i2s"

/* Snd card suffix name */
#define SND_CARD_SUFFIX					"_sound_card"


#define _HAL_SND_CARD_NAME(codec_name, snd_card_suffix)	(codec_name snd_card_suffix)
#define HAL_SND_CARD_NAME(codec_name, snd_card_suffix)	_HAL_SND_CARD_NAME(codec_name, snd_card_suffix)


/* Codec sysclk src */
typedef enum {
	SYSCLK_SRC_OSC,
	SYSCLK_SRC_MCLK,
	SYSCLK_SRC_BCLK,
	SYSCLK_SRC_PLL,
}Codec_Sysclk_Src;

/* Codec PLL src */
typedef enum {
	PLLCLK_SRC_MCLK,
	PLLCLK_SRC_BCLK,
	PLLCLK_SRC_PDMCLK,
}Codec_Pllclk_Src;

/* Codec ioctl cmd */
typedef enum {
	CODEC_IOCTL_PCM_READ,
	CODEC_IOCTL_PCM_WRITE,
	CODEC_IOCTL_HW_CONFIG,
	CODEC_IOCTL_SET_ADDA_DIRECT,
	CODEC_IOCTL_CMD_MAX = CODEC_IOCTL_SET_ADDA_DIRECT,
}Codec_Ioctl_Cmd;


#define I2S_ROLE_MASK		0x000f
#define I2S_FORMAT_MASK		0x00f0
#define I2S_POLARITY_MASK	0x0f00

/* I2S Role */
typedef enum {
	DAIFMT_CBM_CFM = 1<<0,	/*(codec BCLK & FRM master)*/
	DAIFMT_CBS_CFM = 2<<0,	/*(codec BCLK slave & FRM master)*/
	DAIFMT_CBM_CFS = 3<<0,	/*(codec BCLK master & FRM slave)*/
	DAIFMT_CBS_CFS = 4<<0,	/*(codec BCLK & FRM slave)*/
}I2s_Role;

/* I2S Format */
typedef enum {
	DAIFMT_I2S     = 1<<4,	/*(standard i2s format)*/
	DAIFMT_RIGHT_J = 2<<4,	/*(right justfied format)*/
	DAIFMT_LEFT_J  = 3<<4,	/*(left justfied format)*/
	DAIFMT_DSP_A   = 4<<4,	/*(pcm. MSB is available on 2nd BCLK rising edge after LRCK rising edge)*/
	DAIFMT_DSP_B   = 5<<4,	/*(pcm. MSB is available on 1st  BCLK rising edge after LRCK rising edge*/
}I2s_Format;

/* I2S Polarity */
typedef enum {
	DAIFMT_NB_NF = 1<<8,	/*(normal BCLK clock + FRM)*/
	DAIFMT_NB_IF = 2<<8,	/*(normal BCLK + inv FRM)*/
	DAIFMT_IB_NF = 3<<8,	/*(invert BCLK + nor FRM)*/
	DAIFMT_IB_IF = 4<<8,	/*(invert BCLK + FRM)*/
}I2s_Polarity;


/* Audio Stream direction */
typedef enum {
	PCM_OUT,
	PCM_IN,
}Audio_Stream_Dir;

/* Audio Device state */
typedef enum {
	AUDIO_DEV_DIS,
	AUDIO_DEV_EN,
} Audio_Dev_State;

/* AUDIO Mute state */
typedef enum {
	AUDIO_UNMUTE,
	AUDIO_MUTE,
}Audio_Mute_State;


/* codec driver select */
typedef enum {
	XRADIO_CODEC_NULL,
	XRADIO_CODEC_INTERNAL,
	XRADIO_CODEC_AC107,
}Codec_Attr;

/* platform driver select */
typedef enum {
	XRADIO_PLATFORM_NULL,
	XRADIO_PLATFORM_I2S,
	XRADIO_PLATFORM_DMIC,
}Platform_Attr;

/* Sound Card number */
typedef enum {
	SND_CARD_0,
	SND_CARD_1,
	SND_CARD_MAX = SND_CARD_1,
}Snd_Card_Num;


/* Volume set flag */
#define VOLUME_SET_LEVEL	(0x0000)
#define VOLUME_SET_GAIN		(0x8000)
#define VOLUME_SET_MASK		(0x8000)
#define VOLUME_INVALID		(0x7fff)

/* Volume Level */
typedef enum {
	VOLUME_LEVEL0,
	VOLUME_LEVEL1,
	VOLUME_LEVEL2,
	VOLUME_LEVEL3,
	VOLUME_LEVEL4,
	VOLUME_LEVEL5,
	VOLUME_LEVEL6,
	VOLUME_LEVEL7,
	VOLUME_LEVEL8,
	VOLUME_LEVEL9,
	VOLUME_LEVEL10,
	VOLUME_LEVEL11,
	VOLUME_LEVEL12,
	VOLUME_LEVEL13,
	VOLUME_LEVEL14,
	VOLUME_LEVEL15,
	VOLUME_LEVEL16,
	VOLUME_LEVEL17,
	VOLUME_LEVEL18,
	VOLUME_LEVEL19,
	VOLUME_LEVEL20,
	VOLUME_LEVEL21,
	VOLUME_LEVEL22,
	VOLUME_LEVEL23,
	VOLUME_LEVEL24,
	VOLUME_LEVEL25,
	VOLUME_LEVEL26,
	VOLUME_LEVEL27,
	VOLUME_LEVEL28,
	VOLUME_LEVEL29,
	VOLUME_LEVEL30,
	VOLUME_LEVEL31,
	VOLUME_MAX_LEVEL = VOLUME_LEVEL31,
} Volume_Level;

/* Gain @dB */
typedef enum {
	//plus gain
	VOLUME_GAIN_0dB,
	VOLUME_GAIN_1dB,
	VOLUME_GAIN_2dB,
	VOLUME_GAIN_3dB,
	VOLUME_GAIN_4dB,
	VOLUME_GAIN_5dB,
	VOLUME_GAIN_6dB,
	VOLUME_GAIN_7dB,
	VOLUME_GAIN_8dB,
	VOLUME_GAIN_9dB,
	VOLUME_GAIN_10dB,
	VOLUME_GAIN_11dB,
	VOLUME_GAIN_12dB,
	VOLUME_GAIN_13dB,
	VOLUME_GAIN_14dB,
	VOLUME_GAIN_15dB,
	VOLUME_GAIN_16dB,
	VOLUME_GAIN_17dB,
	VOLUME_GAIN_18dB,
	VOLUME_GAIN_19dB,
	VOLUME_GAIN_20dB,
	VOLUME_GAIN_21dB,
	VOLUME_GAIN_22dB,
	VOLUME_GAIN_23dB,
	VOLUME_GAIN_24dB,
	VOLUME_GAIN_25dB,
	VOLUME_GAIN_26dB,
	VOLUME_GAIN_27dB,
	VOLUME_GAIN_28dB,
	VOLUME_GAIN_29dB,
	VOLUME_GAIN_30dB,
	VOLUME_GAIN_31dB,
	VOLUME_GAIN_32dB,
	VOLUME_GAIN_33dB,
	VOLUME_GAIN_34dB,
	VOLUME_GAIN_35dB,
	VOLUME_GAIN_36dB,
	VOLUME_GAIN_37dB,
	VOLUME_GAIN_38dB,
	VOLUME_GAIN_39dB,
	VOLUME_GAIN_40dB,
	VOLUME_GAIN_41dB,
	VOLUME_GAIN_42dB,
	VOLUME_GAIN_43dB,
	VOLUME_GAIN_44dB,
	VOLUME_GAIN_45dB,
	VOLUME_GAIN_46dB,
	VOLUME_GAIN_47dB,
	VOLUME_GAIN_48dB,
	VOLUME_GAIN_49dB,
	VOLUME_GAIN_50dB,

	//minus gain
	VOLUME_GAIN_MINUS_1dB,
	VOLUME_GAIN_MINUS_2dB,
	VOLUME_GAIN_MINUS_3dB,
	VOLUME_GAIN_MINUS_4dB,
	VOLUME_GAIN_MINUS_5dB,
	VOLUME_GAIN_MINUS_6dB,
	VOLUME_GAIN_MINUS_7dB,
	VOLUME_GAIN_MINUS_8dB,
	VOLUME_GAIN_MINUS_9dB,
	VOLUME_GAIN_MINUS_10dB,
	VOLUME_GAIN_MINUS_11dB,
	VOLUME_GAIN_MINUS_12dB,
	VOLUME_GAIN_MINUS_13dB,
	VOLUME_GAIN_MINUS_14dB,
	VOLUME_GAIN_MINUS_15dB,
	VOLUME_GAIN_MINUS_16dB,
	VOLUME_GAIN_MINUS_17dB,
	VOLUME_GAIN_MINUS_18dB,
	VOLUME_GAIN_MINUS_19dB,
	VOLUME_GAIN_MINUS_20dB,
	VOLUME_GAIN_MINUS_21dB,
	VOLUME_GAIN_MINUS_22dB,
	VOLUME_GAIN_MINUS_23dB,
	VOLUME_GAIN_MINUS_24dB,
	VOLUME_GAIN_MINUS_25dB,
	VOLUME_GAIN_MINUS_26dB,
	VOLUME_GAIN_MINUS_27dB,
	VOLUME_GAIN_MINUS_28dB,
	VOLUME_GAIN_MINUS_29dB,
	VOLUME_GAIN_MINUS_30dB,
	VOLUME_GAIN_MINUS_31dB,
	VOLUME_GAIN_MINUS_32dB,
	VOLUME_GAIN_MINUS_33dB,
	VOLUME_GAIN_MINUS_34dB,
	VOLUME_GAIN_MINUS_35dB,
	VOLUME_GAIN_MINUS_36dB,
	VOLUME_GAIN_MINUS_37dB,
	VOLUME_GAIN_MINUS_38dB,
	VOLUME_GAIN_MINUS_39dB,
	VOLUME_GAIN_MINUS_40dB,
	VOLUME_GAIN_MINUS_41dB,
	VOLUME_GAIN_MINUS_42dB,
	VOLUME_GAIN_MINUS_43dB,
	VOLUME_GAIN_MINUS_44dB,
	VOLUME_GAIN_MINUS_45dB,
	VOLUME_GAIN_MINUS_46dB,
	VOLUME_GAIN_MINUS_47dB,
	VOLUME_GAIN_MINUS_48dB,
	VOLUME_GAIN_MINUS_49dB,
	VOLUME_GAIN_MINUS_50dB,
}Volume_Gain;


/* PCM Format */
enum pcm_format {
	PCM_FORMAT_S8,
	PCM_FORMAT_S12_LE,
	PCM_FORMAT_S16_LE,
	PCM_FORMAT_S20_LE,
    PCM_FORMAT_S24_LE,
	PCM_FORMAT_S28_LE,
    PCM_FORMAT_S32_LE,
    PCM_FORMAT_MAX = PCM_FORMAT_S32_LE,
};

/* pcm config*/
struct pcm_config {
    uint32_t rate;
    uint32_t channels;
    uint32_t period_size;
    uint32_t period_count;
    enum pcm_format format;
};


/* PA Switch Control */
typedef struct {
	uint16_t	  on_delay;
	GPIO_PinState on_state;
	const GPIO_PinMuxParam *pin_param;
	uint8_t		  pin_param_cnt;
} Pa_Switch_Ctl;

/* LINEIN Detect Control */
typedef struct {
	GPIO_PinState insert_state;
	const GPIO_PinMuxParam *pin_param;
	uint8_t		  pin_param_cnt;
} Linein_Detect_Ctl;

/* snd card board config */
struct snd_card_board_config {
	Snd_Card_Num card_num;
	const char  *card_name;

	Codec_Attr    codec_link;
	Platform_Attr platform_link;

	const Pa_Switch_Ctl     *pa_switch_ctl;
	const Linein_Detect_Ctl *linein_detect_ctl;

	Codec_Sysclk_Src codec_sysclk_src;
	Codec_Pllclk_Src codec_pllclk_src;
	uint32_t codec_pll_freq_in;
	uint32_t i2s_fmt;
};


/******************************* Audio device enum definition *********************************/

#define AUDIO_IN_DEV_SHIFT		(0)
#define AUDIO_OUT_DEV_SHIFT		(8)
#define AUDIO_DEFAULT_DEV_MASK	(HAL_BIT(31))

typedef enum {
	AUDIO_IN_DEV_AMIC	= HAL_BIT(AUDIO_IN_DEV_SHIFT),		/*< AMIC > */
	AUDIO_IN_DEV_LINEIN	= HAL_BIT(AUDIO_IN_DEV_SHIFT + 1),	/*< LINEIN > */
	AUDIO_IN_DEV_DMIC 	= HAL_BIT(AUDIO_IN_DEV_SHIFT + 2),	/*< DMIC > */
	AUDIO_IN_DEV_ALL	= (0x07<< AUDIO_IN_DEV_SHIFT),

	AUDIO_OUT_DEV_HP	= HAL_BIT(AUDIO_OUT_DEV_SHIFT),		/*< Headphone >*/
	AUDIO_OUT_DEV_SPK	= HAL_BIT(AUDIO_OUT_DEV_SHIFT + 1),	/*< Speaker >*/
	AUDIO_OUT_DEV_ALL	= (0x03<< AUDIO_OUT_DEV_SHIFT),
} Audio_Device;


/********************************* pcm config interface ***************************************/

uint8_t  pcm_format_to_bits(enum pcm_format format);
uint8_t  pcm_format_to_sampleresolution(enum pcm_format format);
uint32_t pcm_config_to_frames(struct pcm_config *config);
uint32_t pcm_frames_to_bytes(struct pcm_config *config, unsigned int frames);


/********************************* HAL snd card interface *************************************/

uint8_t    HAL_SndCard_Register(void);
HAL_Status HAL_SndCard_Unregister(void);
HAL_Status HAL_SndCard_CodecRegisterInternal(void);
HAL_Status HAL_SndCard_CodecUnregisterInternal(void);
HAL_Status HAL_SndCard_CodecRegisterAc107(void);
HAL_Status HAL_SndCard_CodecUnregisterAc107(void);
HAL_Status HAL_SndCard_PlatformRegisterI2S(void);
HAL_Status HAL_SndCard_PlatformUnregisterI2S(void);

HAL_Status HAL_SndCard_Open(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir, struct pcm_config *pcm_cfg);
HAL_Status HAL_SndCard_Close(Snd_Card_Num card_num, Audio_Stream_Dir stream_dir);
HAL_Status HAL_SndCard_SetVolume(Snd_Card_Num card_num, Audio_Device dev, uint16_t volume);
HAL_Status HAL_SndCard_SetRoute(Snd_Card_Num card_num, Audio_Device dev, Audio_Dev_State state);
HAL_Status HAL_SndCard_SetMute(Snd_Card_Num card_num, Audio_Device dev, Audio_Mute_State mute);
HAL_Status HAL_SndCard_Ioctl(Snd_Card_Num card_num, Codec_Ioctl_Cmd cmd, uint32_t cmd_param[], uint32_t cmd_param_len);

int HAL_SndCard_PcmRead(Snd_Card_Num card_num, uint8_t *buf, uint32_t size);
int HAL_SndCard_PcmWrite(Snd_Card_Num card_num, uint8_t *buf, uint32_t size);
int HAL_SndCard_CodecRegRead(Snd_Card_Num card_num, uint32_t reg);
int HAL_SndCard_CodecRegWrite(Snd_Card_Num card_num, uint32_t reg, uint32_t val);

uint8_t HAL_SndCard_GetCardNums(void);
void    HAL_SndCard_GetAllCardNum(uint8_t card_num[]);


/**********************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif

