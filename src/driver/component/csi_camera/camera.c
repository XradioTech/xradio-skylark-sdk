/**
  * @file  camera.c
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kernel/os/os.h"
#include "driver/component/csi_camera/camera.h"

#define CAMERA_JPEG_OUTSTM_VSIZE		(0x10000)

#define CAMERA_DBG_LEVEL INFO
enum LOG_LEVEL{
	OFF = 0,
	ERROR,
	WARN,
	DEBUG,
	INFO,
};
#define CAMERA_DBG(level, fmt, args...) \
	do {		\
		if (level <= CAMERA_DBG_LEVEL)	\
			printf("CAMERA: "fmt,##args);	\
	} while (0)

typedef struct {
	uint32_t encodeLen;
	OS_Semaphore_t sem;
	int8_t sem_count;
	CAMERA_Mgmt *mem_mgmt;
    SENSOR_Func sensor_func;
} CAMERA_Private;

static CAMERA_Private gCameraPrivate = {0};

static JPEG_ConfigParam gJpegCfgParam = {
	.top_clk_en = 1,
	.jpe_clk_en = 1,
	.jpeg_en = 1,
	.quality = 64,
	.jpeg_mode = JPEG_MOD_ONLINE,
	.sensor_out_type = 0,  //0:OUT_YUV420 NV12  1:OUT_JPEG 2:RAW
	.csi_output_addr_y = 0,  //CSI_OUTPUT_Y_ADDR;
	.csi_output_addr_uv = 0,  //CSI_OUTPUT_UV_ADDR;
	.jpe_input_addr_y = 0,//CSI_OUTPUT_Y_ADDR;
	.jpe_input_addr_uv = 0,//CSI_OUTPUT_UV_ADDR;
	.pic_size_width = 640,
	.pic_size_height = 480,
	.mem_part_en = 0,
	.mem_part_num = 0,
	.mem_part_buf = 0,
	.outstream_mem_size = CAMERA_JPEG_OUTSTM_VSIZE,
	.outstream_start_addr = 0, //JPEG_OUTSTM_ADDR;
	.outstream_end_addr = 0, //JPEG_OUTSTM_ADDR + 0xFFFF;
	.outstream_offset = 0,

	.marvolvl_ovtime_int_en = 1,
	.bitstream_stall_int_en = 1,
	.ve_finish_int_en = 1,
};

static CSI_ConfigParam gCsiParam = {
	.out_mode = CSI_OUT_MODE_YUV422_TO_YUV420,
	.yuv420_mask = CSI_YUV420_MASK_UV_ODD,
	.yuv420_line_order = CSI_LINE_ORDER_Y_YC_Y_YC,
	.input_seq = CSI_IN_SEQ_YUYV,
	.input_fmt = CSI_IN_FMT_YUV422,
	.vref_pol = CSI_POL_POSITIVE,
	.href_pol = CSI_POL_POSITIVE,
	.clk_pol = CSI_POL_NEGATIVE,  //CSI_POL_POSITIVE,CSI_POL_NEGATIVE;
	.sync_type = CSI_SYNC_SEPARARE,
	.frame_half_down = 0,
	.frame_mask = 0,
	.hor_len = 640,
	.hor_start = 0,
	.ver_len = 480,
	.ver_start = 0,
};

static SENSOR_ConfigParam gSensorParam;

static void CAMERA_CsiJpegIrq(void *arg)
{
	if(gCameraPrivate.sem_count) { /* avoid more times release sem*/
		if (arg == NULL) {
			gCameraPrivate.encodeLen = 0;
		} else {
			gCameraPrivate.encodeLen = (uint32_t)(*(uint32_t *)arg);

			gCameraPrivate.sem_count--;
			OS_Status sta = OS_SemaphoreRelease(&gCameraPrivate.sem);
			if (sta != OS_OK) {
				CAMERA_DBG(ERROR, "semaphore release error, %d\n", sta);
			}
		}
	}
}

uint32_t CAMERA_GetImage(uint32_t timeout_ms)
{
	gCameraPrivate.sem_count++;

	OS_SemaphoreWait(&gCameraPrivate.sem, timeout_ms);

	uint32_t len = gCameraPrivate.encodeLen;
	gCameraPrivate.encodeLen = 0;

	return len;
}

HAL_Status CAMERA_ConfigCsi(CAMERA_CsiCfg *csi_cfg, SENSOR_PixelSize *pixel_size, SENSOR_PixelOutFmt pixel_outfmt)
{
	HAL_Status status;
	gCsiParam.clk_pol = 0; //CSI_POL_NEGATIVE
	gCsiParam.input_fmt = CSI_IN_FMT_YUV422;
	gCsiParam.out_mode = CSI_OUT_MODE_YUV422_TO_YUV420;

	gCsiParam.hor_start = 0;
	gCsiParam.hor_len = pixel_size->width;
	gCsiParam.ver_start = 0;
	gCsiParam.ver_len = pixel_size->height;

	status = HAL_CSI_Config(&gCsiParam);
	if (status != HAL_OK) {
		CAMERA_DBG(ERROR, "csi config failed\n");
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_Status CAMERA_ConfigJpeg(CAMERA_JpegCfg *jpeg_cfg, SENSOR_PixelSize *pixel_size, SENSOR_PixelOutFmt pixel_outfmt)
{
	uint8_t *jpeg_addr;
	HAL_Status status;

	gJpegCfgParam.jpe_clk_en = 1;
	gJpegCfgParam.top_clk_en = 1;
	gJpegCfgParam.jpeg_en =jpeg_cfg->jpeg_en;
	gJpegCfgParam.jpeg_mode = jpeg_cfg->jpeg_mode;//0: offline 1:online
	gJpegCfgParam.quality = jpeg_cfg->quality;//0~99

	gJpegCfgParam.mem_part_en = jpeg_cfg->memPartEn;
	gJpegCfgParam.mem_part_num = jpeg_cfg->memPartNum;

	gJpegCfgParam.sensor_out_type = 0;//0:OUT_YUV420  1:OUT_JPEG  2:RAW , is csi out type

	gJpegCfgParam.pic_size_width = pixel_size->width;
	gJpegCfgParam.pic_size_height = pixel_size->height;

	if (gJpegCfgParam.mem_part_en) {
		gJpegCfgParam.mem_part_buf = gCameraPrivate.mem_mgmt->online_jpeg_mempart_last_buf;
	}


	if(gJpegCfgParam.jpeg_mode == JPEG_MOD_OFFLINE) {
		gJpegCfgParam.csi_output_addr_y = (uint32_t)gCameraPrivate.mem_mgmt->offline_y_buf;
		gJpegCfgParam.csi_output_addr_uv = (uint32_t)gCameraPrivate.mem_mgmt->offline_uv_buf;
		gJpegCfgParam.jpe_input_addr_y = (uint32_t)gCameraPrivate.mem_mgmt->offline_y_buf;
		gJpegCfgParam.jpe_input_addr_uv = (uint32_t)gCameraPrivate.mem_mgmt->offline_uv_buf;
	}

	if(gJpegCfgParam.jpeg_mode == JPEG_MOD_ONLINE) {
		if(gJpegCfgParam.mem_part_en == 1) {
			jpeg_addr = gCameraPrivate.mem_mgmt->online_jpeg_mempart_tmp_buf;
		} else {
			jpeg_addr = gCameraPrivate.mem_mgmt->online_jpeg_buf;
		}
	} else {
		jpeg_addr = gCameraPrivate.mem_mgmt->offline_jpeg_buf;
	}
	CAMERA_DBG(DEBUG, "jpeg_addr: %p\n", jpeg_addr);
	CAMERA_DBG(DEBUG, "y_addr: %p\n", gCameraPrivate.mem_mgmt->offline_y_buf);
	CAMERA_DBG(DEBUG, "uv_addr: %p\n", gCameraPrivate.mem_mgmt->offline_uv_buf);

	gJpegCfgParam.outstream_mem_size = (gJpegCfgParam.mem_part_en ? 0x2000 : CAMERA_JPEG_OUTSTM_VSIZE);
	gJpegCfgParam.outstream_start_addr = (uint32_t)jpeg_addr;
	gJpegCfgParam.outstream_end_addr = (uint32_t)jpeg_addr + ((gJpegCfgParam.mem_part_en ?
		                                  0x2000 : CAMERA_JPEG_OUTSTM_VSIZE) - 1);
	gJpegCfgParam.outstream_offset = 0;

	gJpegCfgParam.bitstream_stall_int_en  = 1;
	gJpegCfgParam.marvolvl_ovtime_int_en = 1;
	gJpegCfgParam.ve_finish_int_en = 1;

	status = HAL_JPEG_Config(&gJpegCfgParam);
	if (status != HAL_OK) {
		CAMERA_DBG(ERROR, "jpeg config failed\n");
		return HAL_ERROR;
	}

	return HAL_OK;
}

static HAL_Status CAMERA_InitCsiJpeg(void)
{
	HAL_Status status;
	CSI_InitParam initParam;
	initParam.cb = CAMERA_CsiJpegIrq;

	status = HAL_CSI_JPEG_Init(&initParam);
	if (status != HAL_OK) {
		CAMERA_DBG(ERROR, "csi jpeg init failed\n");
		return HAL_ERROR;
	}

	return HAL_OK;
}

static void CAMERA_DeInitCsiJpeg(void)
{
	HAL_CSI_JPEG_Deinit();
}

static HAL_Status CAMERA_InitSensor(CAMERA_Sensorcfg *sensor_cfg)
{
	gSensorParam.i2c_id = sensor_cfg->i2c_id;
	memcpy(&gSensorParam.pwcfg, &sensor_cfg->pwcfg, sizeof(SENSOR_PowerCtrlCfg));
	gSensorParam.pixel_outfmt = sensor_cfg->pixel_outfmt;
	memcpy(&gSensorParam.pixel_size, &sensor_cfg->pixel_size, sizeof(SENSOR_PixelSize));

	return gCameraPrivate.sensor_func.init(&gSensorParam);
}

static void CAMERA_DeInitSensor()
{
	gCameraPrivate.sensor_func.deinit(&gSensorParam);
	memset(&gSensorParam, 0, sizeof(SENSOR_ConfigParam));
}

HAL_Status HAL_CAMERA_Init(CAMERA_Cfg *cfg)
{
	//memset(&gCameraPrivate, 0, sizeof(CAMERA_Private));

    OS_Status sta = OS_SemaphoreCreate(&gCameraPrivate.sem, 0, 1);
    if (sta != OS_OK) {
        CAMERA_DBG(ERROR, "semaphore create error, %d\n", sta);
        return HAL_ERROR;
    }

	/* csi init */
	if (CAMERA_InitCsiJpeg() != HAL_OK) {
		OS_SemaphoreDelete(&gCameraPrivate.sem);
		return HAL_ERROR;
	}

	/* jpeg config */
	if(CAMERA_ConfigJpeg(&cfg->jpeg_cfg, &cfg->sensor_cfg.pixel_size,
		                  cfg->sensor_cfg.pixel_outfmt) != HAL_OK) {
		CAMERA_DeInitCsiJpeg();
		OS_SemaphoreDelete(&gCameraPrivate.sem);
		return HAL_ERROR;
	}

	/* csi config */
	if (CAMERA_ConfigCsi(&cfg->csi_cfg, &cfg->sensor_cfg.pixel_size,
		                    cfg->sensor_cfg.pixel_outfmt) != HAL_OK) {
		CAMERA_DeInitCsiJpeg();
		OS_SemaphoreDelete(&gCameraPrivate.sem);
		return HAL_ERROR;
	}

	/* sensor init */
	memset(&gSensorParam, 0, sizeof(SENSOR_ConfigParam));
	memcpy(&gCameraPrivate.sensor_func, &cfg->sensor_func, sizeof(SENSOR_Func));

	if (CAMERA_InitSensor(&cfg->sensor_cfg) != HAL_OK) {
		CAMERA_DeInitCsiJpeg();
		OS_SemaphoreDelete(&gCameraPrivate.sem);
		return HAL_ERROR;
	}

	return HAL_OK;
}

void HAL_CAMERA_DeInit()
{
	CAMERA_DeInitCsiJpeg();

	CAMERA_DeInitSensor();

	OS_SemaphoreDelete(&gCameraPrivate.sem);

	memset(&gCameraPrivate, 0 ,sizeof(gCameraPrivate));
}

uint32_t HAL_CAMERA_CaptureOneImage(void)
{
	uint32_t size = 0;
	HAL_JPEG_WriteHeader((uint8_t*)gCameraPrivate.mem_mgmt->jpeg_header_buf);

	if (HAL_CSI_StartCapture(CSI_CAP_STILL) != HAL_OK) {
		CAMERA_DBG(ERROR, "start cap still mode fail\n");
		return size;
	}

	size = CAMERA_GetImage(5000);
	if (size == 0 && gJpegCfgParam.jpeg_en) {
		CAMERA_DBG(ERROR, "capture fail...\n");
		return size;
	}

	size = size + CAMERA_JPEG_HEADER_LEN;

	HAL_CSI_StopCapture();

	return size;
}

HAL_Status HAL_CAMERA_CaptureVideoStart(void)
{
	if (HAL_CSI_StartCapture(CSI_CAP_VIDEO) != HAL_OK) {
		CAMERA_DBG(ERROR, "start cap video mode fail\n");
		return HAL_ERROR;
	}

	return HAL_OK;
}

void HAL_CAMERA_CaptureVideoStop(void)
{
	HAL_CSI_StopCapture();
}

uint32_t HAL_CAMERA_CaptureVideoGetData(void)
{
	uint32_t size = 0;

	HAL_JPEG_WriteHeader((uint8_t*)gCameraPrivate.mem_mgmt->jpeg_header_buf);
	size = CAMERA_GetImage(5000);
	if (size == 0 && gJpegCfgParam.jpeg_en) {
		CAMERA_DBG(ERROR, "capture fail...\n");
		return size;
	}

	size = size + CAMERA_JPEG_HEADER_LEN;
	return size;
}

void HAL_CAMERA_SetImageBuf(CAMERA_Mgmt *mem_mgmt_t)
{
	gCameraPrivate.mem_mgmt = mem_mgmt_t;
}
