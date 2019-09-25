/**
  * @file  camera_sensor.h
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

#ifndef __CAMERA_SENSOR_H__
#define __CAMERA_SENSOR_H__

#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_def.h"

typedef enum {
	LIGHT_AUTO,
	LIGHT_SUNNY,
	LIGHT_COLUDY,
	LIGHT_OFFICE,
	LIGHT_HOME,
} SENSOR_LightMode;

typedef enum {
	COLOR_SATURATION_0, /*!< -2 */
	COLOR_SATURATION_1, /*!< -1*/
	COLOR_SATURATION_2, /*!< The default */
	COLOR_SATURATION_3, /*!< 1 */
	COLOR_SATURATION_4, /*!< 2 */
} SENSOR_ColorSaturation;

typedef enum {
	BRIGHT_0,  /*!< birghtness -2 */
	BRIGHT_1,  /*!< -1 */
	BRIGHT_2,  /*!< The default */
	BRIGHT_3,  /*!< 1 */
	BRIGHT_4,  /*!< 2 */
} SENSOR_Brightness;

typedef enum {
	CONTARST_0, /*!< -2 */
	CONTARST_1, /*!< -1 */
	CONTARST_2, /*!< The default */
	CONTARST_3, /*!< 1 */
	CONTARST_4, /*!< 2 */
} SENSOR_Contarst;

/**
  * @brief effects.
  */
typedef enum {
	IMAGE_NOMAL,
	IMAGE_NEGATIVE,
	IMAGE_BLACK_WHITE,
	IMAGE_SLANT_RED,
	IMAGE_SLANT_GREEN,
	IMAGE_SLANT_BLUE,
	IMAGE_VINTAGE,
}SENSOR_SpecailEffects;

typedef struct {
	uint32_t width;
	uint32_t height;
} SENSOR_PixelSize;

typedef enum {
	YUV422_UYVY = 0,
	YUV422_VYUY,
	YUV422_YUYV,
	YUV422_YVYU,
	RGB565,
	RAW_RGB_LSC,
	RAW_RGB_DNDD,
	RAW_RGB_CISCTRL,
} SENSOR_PixelOutFmt;

typedef enum  {
	SENSOR_SET_CLK_POL    = 0,
    SENSOR_SET_OUTPUT_FMT,
    SENSOR_SET_PIXEL_SIZE,

        /* TODO ... */
} SENSOR_IoctrlCmd;

typedef struct {
	GPIO_Port Reset_Port;
	GPIO_Pin Reset_Pin;
	GPIO_Port Pwdn_Port;
	GPIO_Pin Pwdn_Pin;
} SENSOR_PowerCtrlCfg;

typedef struct {
	SENSOR_PowerCtrlCfg pwcfg;
	uint8_t i2c_id;
	SENSOR_PixelSize pixel_size;
	SENSOR_PixelOutFmt pixel_outfmt;
} SENSOR_ConfigParam;

typedef struct {
    HAL_Status (*init)(SENSOR_ConfigParam *cfg);
    void (*deinit)(SENSOR_ConfigParam *cfg);
    void (*suspend)(void);
    void (*resume)(void);
    HAL_Status (*ioctl)(SENSOR_IoctrlCmd attr, uint32_t arg);
} SENSOR_Func;

#endif

