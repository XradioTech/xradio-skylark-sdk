/**
  * @file  camera.h
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

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "driver/chip/hal_csi_jpeg.h"
#include "driver/chip/hal_def.h"
#include "kernel/os/os.h"
#include "driver/component/csi_camera/camera_sensor.h"

#define ALIGN_16B(x)				(((x) + (15)) & ~(15))
#define ALIGN_1K(x)					(((x) + (1023)) & ~(1023))

#define CAMERA_BUFF_CNT_MAX			(3)
#define CAMERA_JPEG_HEADER_LEN		623

typedef struct {
	uint8_t *addr;
	uint32_t size;
} CAMERA_BuffInfo;

typedef struct {
	CAMERA_BuffInfo yuv_buf; /* last yuv420 NV12 data */
	CAMERA_BuffInfo jpeg_buf[CAMERA_BUFF_CNT_MAX];  /* jpeg data, you can use multiple buffers when video*/
} CAMERA_Mgmt;

typedef enum {
	CAMERA_STATUS_FRM_END,
	CAMERA_STATUS_VE_END,
	CAMERA_STATUS_MPART,
	CAMERA_STATUS_EXCP,
} CAMERA_CapStatus;

typedef JPEG_MpartBuffInfo CAMERA_MpartBuffInfo;
typedef JPEG_BuffInfo CAMERA_JpegBuffInfo;

typedef void (*CAMERA_CapStatusCb)(CAMERA_CapStatus status, void *arg);

typedef struct {
	uint8_t i2c_id;
	SENSOR_PowerCtrlCfg pwcfg;
} CAMERA_Sensorcfg;

typedef struct {
	uint32_t csi_clk;
	uint32_t hor_start;
	uint32_t ver_start;
} CAMERA_CsiCfg;

typedef struct {
	uint8_t jpeg_en;
	uint32_t jpeg_clk;
	uint32_t memPartEn;
	uint32_t memPartNum;
	uint32_t quality;
	uint32_t width;
	uint32_t height;
	JPEG_Mode jpeg_mode;
} CAMERA_JpegCfg;

typedef struct {
	CAMERA_JpegCfg jpeg_cfg;
	CAMERA_CsiCfg csi_cfg;
	CAMERA_Sensorcfg sensor_cfg;
	SENSOR_Func sensor_func;
	CAMERA_Mgmt *mgmt;
	CAMERA_CapStatusCb cb;
} CAMERA_Cfg;

typedef enum {
	CAMERA_OUT_YUV420,
	CAMERA_OUT_JPEG,
} CAMERA_OutFmt;

typedef enum  {
	CAMERA_SET_PIXEL_SIZE    = 0,
    CAMERA_SET_JPEG_MODE,
    CAMERA_RESET_CSI_JPEG,
    CAMERA_SET_SENSOR_SUBSAMP,
    CAMERA_SET_JPEG_SCALE,

        /* TODO ... */
} CAMERA_IoctrlCmd;

int HAL_CAMERA_Init(CAMERA_Cfg *cfg);
void HAL_CAMERA_DeInit(void);

int HAL_CAMERA_CaptureMpartStart(uint8_t mode);
int HAL_CAMERA_CaptureMpartStop(void);

int HAL_CAMERA_CaptureImage(CAMERA_OutFmt fmt, CAMERA_JpegBuffInfo* info, uint8_t restart);

int HAL_CAMERA_CaptureVideoStart(void);
int HAL_CAMERA_CaptureVideoStop(void);
uint32_t HAL_CAMERA_CaptureVideoData(CAMERA_JpegBuffInfo *info);

int HAL_CAMERA_IoCtl(CAMERA_IoctrlCmd cmd, uint32_t arg);

#endif
