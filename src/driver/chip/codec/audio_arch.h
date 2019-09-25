/**
  * @file  audio_arch.h
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

#ifndef _AUDIO_ARCH_H_
#define _AUDIO_ARCH_H_


#include "sys/defs.h"
#include "sys/list.h"
#include "../hal_base.h"
#include "driver/chip/hal_snd_card.h"


#ifdef __cplusplus
 extern "C" {
#endif


/******************************** Codec Driver Architecture ***********************************/
/*** codec dai ops ***/
struct codec_dai_ops {
	int (*set_sysclk)(Codec_Sysclk_Src sysclk_src, Codec_Pllclk_Src pllclk_src, uint32_t pll_freq_in, uint32_t sample_rate);
	int (*set_fmt)(uint32_t fmt);
	int (*set_volume)(Audio_Device device, uint16_t volume);
	int (*set_route)(Audio_Device device, Audio_Dev_State state);
	int (*hw_params)(Audio_Stream_Dir dir, struct pcm_config *pcm_cfg);
	int (*hw_free)(Audio_Stream_Dir dir);
};

/*** codec ops ***/
struct codec_ops {
	int (*open)(Audio_Stream_Dir dir);
	int (*close)(Audio_Stream_Dir dir);

	int (*reg_read)(uint32_t reg);
	int (*reg_write)(uint32_t reg, uint32_t val);

	int (*ioctl)(uint32_t cmd, uint32_t cmd_param[], uint32_t cmd_param_len);
};

/*** codec driver ***/
struct codec_driver {
	const char *name;
	Codec_Attr codec_attr;

	int (*init)(void);
	void (*deinit)(void);

	const struct codec_dai_ops *dai_ops;
	const struct codec_ops *codec_ops;

	struct list_head node;
};

/******************************* Platform Driver Architecture **********************************/

/* platform ops */
struct platform_ops {
	HAL_Status (*open)(void *param);
	HAL_Status (*close)(Audio_Stream_Dir dir);
	int (*pcm_read)(uint8_t *buf, uint32_t size);
	int (*pcm_write)(uint8_t *buf, uint32_t size);
};

/* platform driver */
struct platform_driver {
	const char *name;
	Platform_Attr platform_attr;

	HAL_Status (*init)(void *param);
	void (*deinit)(void);

	const struct platform_ops *platform_ops;

	struct list_head node;
};

/******************************* Hal Driver Architecture **************************************/

/* snd card */
struct snd_card {
	//card information
	Snd_Card_Num card_num;
	const char  *card_name;

	//bind driver
	const struct codec_driver    *codec_drv;
	const struct platform_driver *platform_drv;

	//board config control
	const Pa_Switch_Ctl     *pa_switch_ctl;
	const Linein_Detect_Ctl *linein_detect_ctl;

	//user config
	uint16_t codec_play_vol;
	uint16_t codec_record_vol;
	uint32_t codec_play_dev;
	uint32_t codec_record_dev;
	Codec_Sysclk_Src codec_sysclk_src;
	Codec_Pllclk_Src codec_pllclk_src;
	uint32_t codec_pll_freq_in;
	uint32_t i2s_fmt;

	//misc
	HAL_Mutex card_lock;
	struct list_head node;

#ifdef CONFIG_PM
	//pm device
	struct soc_device *pm_dev;
#endif
};


//Audio CLK Freq
typedef enum {
	AUDIO_CLK_11M = 11289600U,
	AUDIO_CLK_12M = 12288000U,
    AUDIO_CLK_22M = 22579200U,
	AUDIO_CLK_24M = 24576000U,
} Audio_Clk_Freq;


//audio arch list
extern struct list_head hal_snd_codec_list;
extern struct list_head hal_snd_platform_list;
extern struct list_head hal_snd_card_list;


/********************************* Codec and Platform interface *******************************/

HAL_Status xradio_internal_codec_register(void);
HAL_Status xradio_internal_codec_unregister(void);

HAL_Status ac107_codec_register(void);
HAL_Status ac107_codec_unregister(void);
HAL_Status ac107_pdm_init(Audio_Device device, uint16_t volume, uint32_t sample_rate);
HAL_Status ac107_pdm_deinit(void);

HAL_Status xradio_i2s_register(void);
HAL_Status xradio_i2s_unregister(void);


/*****************************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* _AUDIO_ARCH_H_ */

