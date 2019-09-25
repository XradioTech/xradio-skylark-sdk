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

#define CAMERA_JPEG_HEADER_LEN 623

typedef struct {
	uint8_t *org_addr; /* malloc addr */
	uint8_t *online_jpeg_buf;  /* last jpeg data buf location */
	uint8_t *online_jpeg_mempart_tmp_buf; /* mem part mode, temp for hardware rx */
	uint8_t *online_jpeg_mempart_last_buf;  /* last jpeg data buf location */
	uint8_t *offline_jpeg_buf; /*last jpeg data buf location */
	uint8_t *offline_y_buf; /* last yuv420 NV12 y data */
	uint8_t *offline_uv_buf; /* last yuv420 NV12 uv data */
	uint8_t *jpeg_header_buf; /* 623 bytes */
} CAMERA_Mgmt;

typedef struct {
	SENSOR_PowerCtrlCfg pwcfg;
	uint8_t i2c_id;
	SENSOR_PixelSize pixel_size;
	SENSOR_PixelOutFmt pixel_outfmt;
} CAMERA_Sensorcfg;

typedef struct {
	uint32_t csi_clk;
} CAMERA_CsiCfg;

typedef struct {
	uint8_t jpeg_en;
	uint32_t jpeg_clk;
	uint32_t memPartEn;
	uint32_t memPartNum;
	uint32_t quality;
	JPEG_Mode jpeg_mode;
} CAMERA_JpegCfg;

typedef struct {
	CAMERA_JpegCfg jpeg_cfg;
	CAMERA_CsiCfg csi_cfg;
	CAMERA_Sensorcfg sensor_cfg;
	SENSOR_Func sensor_func;
} CAMERA_Cfg;

HAL_Status HAL_CAMERA_Init(CAMERA_Cfg *cfg);
void HAL_CAMERA_DeInit(void);
void HAL_CAMERA_SetImageBuf(CAMERA_Mgmt *mem_mgmt_t);
uint32_t HAL_CAMERA_CaptureOneImage(void);
HAL_Status HAL_CAMERA_CaptureVideoStart(void);
void HAL_CAMERA_CaptureVideoStop(void);
uint32_t HAL_CAMERA_CaptureVideoGetData(void);
#endif
