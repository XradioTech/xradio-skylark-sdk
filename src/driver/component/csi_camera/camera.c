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
#include "driver/component/csi_camera/private/camera_debug.h"
#include "kernel/os/os_mutex.h"

typedef struct {
	uint8_t cap_mode;	/* 0:still	1:video */
	uint8_t cap_start;
	uint8_t encode_buff_id;
	uint32_t encode_len;
	OS_Mutex_t lock;
	OS_Semaphore_t sem;
	uint8_t	sem_wait;
	CAMERA_Mgmt *mem_mgmt;
    SENSOR_Func sensor_func;
	SENSOR_ConfigParam sensor_param;
	CAMERA_OutFmt out_fmt;
	CAMERA_CapStatusCb cb;
} CAMERA_Private;

static CAMERA_Private *gCameraPrivate;

static JPEG_ConfigParam gJpegParam = {
	.jpeg_en = 1,
	.quality = 64,
	.jpeg_mode = JPEG_MOD_ONLINE,
	.sensor_out_type = 0,  		/* 0:OUT_YUV420 NV12  1:OUT_JPEG 2:RAW */
	.jpeg_bitrate_en = 0,
	.jpeg_scale = 0,
	.csi_output_addr_y = 0,		/* CSI_OUTPUT_Y_ADDR */
	.csi_output_addr_uv = 0,	/* CSI_OUTPUT_UV_ADDR */
	.jpeg_input_addr_y = 0,		/* CSI_INPUT_Y_ADDR */
	.jpeg_input_addr_uv = 0,		/* CSI_INPUT_UV_ADDR */
	.pic_size_width = 640,
	.pic_size_height = 480,
	.mem_part_en = 0,
	.mem_part_num = 0,
	.outstream_buff_size = 0,
	.outstream_buff_num = 0,
	.outstream_buff_addr[0] = 0, /* JPEG_OUTSTM_ADDR */
	.outstream_buff_offset = 0,
};

static CSI_ConfigParam gCsiParam = {
	.out_mode = CSI_OUT_MODE_YUV422_TO_YUV420,
	.yuv420_mask = CSI_YUV420_MASK_UV_ODD,
	.yuv420_line_order = CSI_LINE_ORDER_Y_YC_Y_YC,
	.input_seq = CSI_IN_SEQ_YUYV,
	.input_fmt = CSI_IN_FMT_YUV422,
	.vref_pol = CSI_POL_POSITIVE,
	.href_pol = CSI_POL_POSITIVE,
	.clk_pol = CSI_POL_NEGATIVE,  /* CSI_POL_POSITIVE, CSI_POL_NEGATIVE */
	.sync_type = CSI_SYNC_SEPARARE,
	.hor_len = 640,
	.hor_start = 0,
	.ver_len = 480,
	.ver_start = 0,
};

static CAMERA_Private *CAMERA_GetPriv()
{
	return gCameraPrivate;
}

static void CAMERA_SetPriv(CAMERA_Private *priv)
{
	gCameraPrivate = priv;
}

static void CAMERA_CsiJpegIrq(CSI_JPEG_IRQEvent event, void *arg)
{
	uint8_t release = 0;
	CAMERA_MpartBuffInfo *info = (CAMERA_MpartBuffInfo*)arg;
	CAMERA_CapStatus status = CAMERA_STATUS_EXCP;
	CAMERA_Private *priv = CAMERA_GetPriv();

	if (event == CSI_JPEG_EVENT_FRM_END) {
		status = CAMERA_STATUS_FRM_END;
		if (priv->out_fmt == CAMERA_OUT_YUV420)
			release = 1;
	} else if (event == CSI_JPEG_EVENT_VE_END) {
		status = CAMERA_STATUS_VE_END;
		CAMERA_JpegBuffInfo *jpeg_info = (CAMERA_JpegBuffInfo*)arg;
		priv->encode_len = jpeg_info->size;
		priv->encode_buff_id = jpeg_info->buff_index;
		release = 1;
	}  else if (event == CSI_JPEG_EVENT_MPART) {
		status = CAMERA_STATUS_MPART;
	} else {
		CAMERA_ERR("csi jpeg excption\n");
		HAL_JPEG_Reset();
		HAL_JPEG_Config(&gJpegParam);
		HAL_CSI_Config(&gCsiParam);
	}
	if (priv->cb)
		priv->cb(status, info);

	if(release && priv->sem_wait) {
		OS_SemaphoreRelease(&priv->sem);
		priv->sem_wait = 0;
	}
}

static int CAMERA_GetImage(CAMERA_OutFmt fmt, CAMERA_JpegBuffInfo* info, uint8_t restart, uint32_t timeout_ms)
{
	CAMERA_Private *priv = CAMERA_GetPriv();

	int len = gJpegParam.pic_size_width*gJpegParam.pic_size_height*3/2;

	if (!restart) {
		if (fmt == CAMERA_OUT_JPEG) {
			info->buff_index = priv->encode_buff_id;
			info->size = priv->encode_len;
			return 0;
		} else if (fmt == CAMERA_OUT_YUV420)
			return len;
	}

	if (HAL_CSI_StartCapture(CSI_CAP_STILL) != HAL_OK) {
		CAMERA_ERR("start cap still mode fail\n");
		return -1;
	}

	priv->out_fmt = fmt;
	priv->sem_wait = 1;
	if (OS_SemaphoreWait(&priv->sem, timeout_ms) != OS_OK) {
		CAMERA_ERR("sem wait fail\n");
		return -1;
	}

	if (fmt == CAMERA_OUT_JPEG) {
		if (!gJpegParam.mem_part_en && gJpegParam.outstream_buff_size < (priv->encode_len + CAMERA_JPEG_HEADER_LEN))
			CAMERA_WRN("jpeg buffer size must > %dB\n", priv->encode_len + CAMERA_JPEG_HEADER_LEN);
		info->buff_index = priv->encode_buff_id;
		info->size = priv->encode_len;
		return 0;
	}

	return len;
}

HAL_Status CAMERA_ConfigCsi(CAMERA_CsiCfg *csi_cfg)
{
	HAL_Status status;
	gCsiParam.clk_pol = CSI_POL_NEGATIVE;
	gCsiParam.input_fmt = CSI_IN_FMT_YUV422;
	gCsiParam.out_mode = CSI_OUT_MODE_YUV422_TO_YUV420;

	gCsiParam.hor_start = csi_cfg->hor_start;
	gCsiParam.hor_len = gJpegParam.pic_size_width;
	gCsiParam.ver_start = csi_cfg->ver_start;
	gCsiParam.ver_len = gJpegParam.pic_size_height;

	status = HAL_CSI_Config(&gCsiParam);
	if (status != HAL_OK) {
		CAMERA_ERR("csi config failed\n");
		return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_Status CAMERA_ConfigJpeg(CAMERA_JpegCfg *jpeg_cfg)
{
	uint8_t *addr;
	HAL_Status status;
	CAMERA_Private *priv = CAMERA_GetPriv();

	gJpegParam.jpeg_en =jpeg_cfg->jpeg_en;
	gJpegParam.jpeg_mode = jpeg_cfg->jpeg_mode; /* 0: offline 1:online */
	gJpegParam.quality = jpeg_cfg->quality;		/* 0~99 */
	gJpegParam.mem_part_en = jpeg_cfg->memPartEn;
	gJpegParam.mem_part_num = jpeg_cfg->memPartNum;
	gJpegParam.sensor_out_type = 0;	/* 0:OUT_YUV420  1:OUT_JPEG  2:RAW , is csi out type */
	gJpegParam.pic_size_width = jpeg_cfg->width;
	gJpegParam.pic_size_height = jpeg_cfg->height;

	int index;
	addr = priv->mem_mgmt->jpeg_buf[0].addr;
	if (addr) {
		gJpegParam.outstream_buff_offset = 0;
		gJpegParam.outstream_buff_num = 0;
		gJpegParam.outstream_buff_size = priv->mem_mgmt->jpeg_buf[0].size;
		for (index = 0; index < CAMERA_BUFF_CNT_MAX && (addr = priv->mem_mgmt->jpeg_buf[index].addr); index++) {
			CAMERA_DBG("jpeg_addr[%d]: %p\n", index, priv->mem_mgmt->jpeg_buf[index].addr);
			if (addr != (uint8_t *)ALIGN_1K((uint32_t)addr)) {
				CAMERA_ERR("jpeg buff must align 1KB\n");
				return HAL_ERROR;
			}
			if (gJpegParam.outstream_buff_size > priv->mem_mgmt->jpeg_buf[index].size) {
				CAMERA_ERR("jpeg buff size should >= %d\n", gJpegParam.outstream_buff_size);
				return HAL_ERROR;
			}
			gJpegParam.outstream_buff_num++;
			gJpegParam.outstream_buff_addr[index] = (uint32_t)addr;
		}
	}

	if (gJpegParam.jpeg_mode == JPEG_MOD_OFFLINE) {
		uint32_t yuv_size = gJpegParam.pic_size_width*gJpegParam.pic_size_height/2*3;
		if (priv->mem_mgmt->yuv_buf.size < yuv_size) {
			CAMERA_ERR("yuv buff size should >= %dB(1.5*width*height)\n", yuv_size);
			return HAL_ERROR;
		}
		addr = priv->mem_mgmt->yuv_buf.addr;
		if (addr != (uint8_t *)ALIGN_16B((uint32_t)addr)) {
			CAMERA_ERR("yuv buff must align 16B\n");
			return HAL_ERROR;
		}
#if 0
		if (gJpegParam.outstream_buff_num > 1) {
			CAMERA_ERR("not support multiple buff at offline\n");
			return HAL_ERROR;
		}
#endif
		gJpegParam.csi_output_addr_y = (uint32_t)addr;
		gJpegParam.csi_output_addr_uv = (uint32_t)addr + (jpeg_cfg->width * jpeg_cfg->height);
		gJpegParam.jpeg_input_addr_y = gJpegParam.csi_output_addr_y;
		gJpegParam.jpeg_input_addr_uv = gJpegParam.csi_output_addr_uv;
		CAMERA_DBG("yuv_addr: %p\n", addr);
	}

	status = HAL_JPEG_Config(&gJpegParam);
	if (status != HAL_OK) {
		CAMERA_ERR("jpeg config failed\n");
		return HAL_ERROR;
	}

	for (index = 0; index < CAMERA_BUFF_CNT_MAX && priv->mem_mgmt->jpeg_buf[index].addr; index++)
		HAL_JPEG_WriteHeader((uint8_t*)(priv->mem_mgmt->jpeg_buf[index].addr-CAMERA_JPEG_HEADER_LEN));

	return HAL_OK;
}

static HAL_Status CAMERA_InitCsiJpeg(void)
{
	HAL_Status status;
	CSI_JPEG_InitParam initParam;
	initParam.cb = CAMERA_CsiJpegIrq;

	status = HAL_CSI_JPEG_Init(&initParam);
	if (status != HAL_OK) {
		CAMERA_ERR("csi jpeg init failed\n");
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
	CAMERA_Private *priv = CAMERA_GetPriv();

	priv->sensor_param.i2c_id = sensor_cfg->i2c_id;
	memcpy(&priv->sensor_param.pwcfg, &sensor_cfg->pwcfg, sizeof(SENSOR_PowerCtrlCfg));

	return priv->sensor_func.init(&priv->sensor_param);
}

static void CAMERA_DeInitSensor()
{
	CAMERA_Private *priv = CAMERA_GetPriv();
	priv->sensor_func.deinit(&priv->sensor_param);
}

int HAL_CAMERA_Init(CAMERA_Cfg *cfg)
{
	if (!cfg || !cfg->mgmt) {
		CAMERA_ERR("param not exist\n");
		return -1;
	}

	CAMERA_Private *priv = CAMERA_GetPriv();
	if (priv) {
		CAMERA_WRN("%s reinit\n", __func__);
		return -1;
	} else {
		priv = malloc(sizeof(CAMERA_Private));
		if (!priv) {
			CAMERA_ERR("%s malloc faild\n", __func__);
			return -1;
		}
		memset(priv, 0, sizeof(CAMERA_Private));
		CAMERA_SetPriv(priv);
	}

	if (OS_MutexCreate(&priv->lock) != OS_OK) {
		CAMERA_ERR("mutex create fail\n");
		goto exit_4;
	}
	if (OS_SemaphoreCreateBinary(&priv->sem) != OS_OK) {
        CAMERA_ERR("sem create fail\n");
        goto exit_3;
    }

	priv->cb = cfg->cb;
	priv->mem_mgmt = cfg->mgmt;

	/* csi init */
	if (CAMERA_InitCsiJpeg() != HAL_OK) {
		CAMERA_ERR("csi/jpeg init fail\n");
        goto exit_2;
	}

	/* jpeg config */
	if (CAMERA_ConfigJpeg(&cfg->jpeg_cfg) != HAL_OK) {
		CAMERA_ERR("jpeg config fail\n");
		goto exit_1;
	}

	/* csi config */
	if (CAMERA_ConfigCsi(&cfg->csi_cfg) != HAL_OK) {
		CAMERA_ERR("csi config fail\n");
		goto exit_1;
	}

	/* sensor init */
	memcpy(&priv->sensor_func, &cfg->sensor_func, sizeof(SENSOR_Func));

	if (CAMERA_InitSensor(&cfg->sensor_cfg) != HAL_OK) {
		CAMERA_ERR("sensor config fail\n");
		goto exit_1;
	}
	return 0;

exit_1:
	CAMERA_DeInitCsiJpeg();

exit_2:
	OS_SemaphoreDelete(&priv->sem);

exit_3:
	OS_MutexDelete(&priv->lock);

exit_4:
	free(priv);
	CAMERA_SetPriv(NULL);
	return -1;
}

void HAL_CAMERA_DeInit(void)
{
	CAMERA_Private *priv = CAMERA_GetPriv();
	if (!priv) {
		CAMERA_ERR("not init\n");
		return;
	}

	CAMERA_DeInitCsiJpeg();

	CAMERA_DeInitSensor();

	OS_SemaphoreDelete(&priv->sem);

	OS_MutexDelete(&priv->lock);

	free(priv);
	CAMERA_SetPriv(NULL);
}

/**
 * @brief Start to capture mem part jpeg.
 * @param mode: 0-still mode, 1-video mode.
 * @retval 0: on success, EIO: if an error occurred with any step.
 */
int HAL_CAMERA_CaptureMpartStart(uint8_t mode)
{
	if (HAL_CSI_StartCapture(mode ? CSI_CAP_VIDEO : CSI_CAP_STILL) != HAL_OK) {
		CAMERA_ERR("start cap fail\n");
		return -1;
	}
	return 0;
}

/**
 * @brief Stop to capture mem part jpeg.
 * @param null.
 * @retval 0: on success, EIO : if an error occurred with any step.
 */
int HAL_CAMERA_CaptureMpartStop(void)
{
	return ((HAL_CSI_StopCapture() == HAL_OK) ? 0 : -1);
}

/**
 * @brief To get one frame image(jpeg or YUV420) in still mode.
 * @param[in] fmt: Indicate the format of the image you want to get.
 * @param[in] restart: Whether to recapture the image.
 * @param[out] info: Indicate the jpeg image into, vaild only when getting jpeg.
 * @retval not equal to -1 : on success, EIO : if an error occurred with any step.
 */
int HAL_CAMERA_CaptureImage(CAMERA_OutFmt fmt, CAMERA_JpegBuffInfo* info, uint8_t restart)
{
	CAMERA_Private *priv = CAMERA_GetPriv();
	if (!priv) {
		CAMERA_ERR("not init\n");
		return -1;
	}

	if (!priv->cap_start && !restart) {
		CAMERA_ERR("should capture firstly\n");
		return -1;
	}

	OS_MutexLock(&priv->lock, OS_WAIT_FOREVER);

	priv->cap_mode = 0;

	int size = CAMERA_GetImage(fmt, info, restart, 5000);
	if (size < 0) {
		CAMERA_ERR("capture fail...\n");
		OS_MutexUnlock(&priv->lock);
		return -1;
	}
	priv->cap_start = 1;
	OS_MutexUnlock(&priv->lock);

	return size;
}

/**
 * @brief Start to capture jpeg image in video mode.
 * @param null.
 * @retval 0 : on success, EIO : if an error occurred with any step.
 */
int HAL_CAMERA_CaptureVideoStart(void)
{
	CAMERA_Private *priv = CAMERA_GetPriv();
	if (!priv) {
		CAMERA_ERR("not init\n");
		return -1;
	}
	priv->cap_mode = 1;
	priv->out_fmt = CAMERA_OUT_JPEG;

	if (gJpegParam.mem_part_en) {
		CAMERA_WRN("not suppport mem part here\n");
		return -1;
	}
	if (HAL_CSI_StartCapture(CSI_CAP_VIDEO) != HAL_OK) {
		CAMERA_ERR("start cap video mode fail\n");
		return -1;
	}

	return 0;
}

/**
 * @brief Stop to capture jpeg image in video mode.
 * @param null.
 * @retval 0: on success, EIO : if an error occurred with any step.
 */
int HAL_CAMERA_CaptureVideoStop(void)
{
	return ((HAL_CSI_StopCapture() == HAL_OK) ? 0 : -1);
}

/**
 * @brief To get one frame jpeg image in video mode.
 * @param[out] info: Indicate the jpeg image into.
 * @retval 0 : on success, EIO : if an error occurred with any step.
 */
uint32_t HAL_CAMERA_CaptureVideoData(CAMERA_JpegBuffInfo *info)
{
	CAMERA_Private *priv = CAMERA_GetPriv();
	if (!priv) {
		CAMERA_ERR("not init\n");
		return -1;
	}

	priv->sem_wait = 1;
	if (OS_SemaphoreWait(&priv->sem, 5000) != OS_OK) {
		CAMERA_ERR("sem wait fail\n");
		return -1;
	}

	info->buff_index = priv->encode_buff_id;
	info->size = priv->encode_len;
	return 0;
}

int HAL_CAMERA_IoCtl(CAMERA_IoctrlCmd cmd, uint32_t arg)
{
	CAMERA_Private *priv = CAMERA_GetPriv();
	if (!priv) {
		CAMERA_ERR("not init\n");
		return -1;
	}
	OS_MutexLock(&priv->lock, OS_WAIT_FOREVER);

	switch (cmd) {
		case CAMERA_SET_PIXEL_SIZE:
		{
			SENSOR_PixelSize *pixel = (SENSOR_PixelSize*)arg;
			if (!pixel) {
				CAMERA_ERR("invalid handle\n");
				OS_MutexUnlock(&priv->lock);
				return -1;
			}
			if (gCsiParam.hor_len != pixel->width || gCsiParam.ver_len != pixel->height) {
				gJpegParam.csi_output_addr_uv = (uint32_t)priv->mem_mgmt->yuv_buf.addr +
															(pixel->width * pixel->height);
				gJpegParam.jpeg_input_addr_uv = gJpegParam.csi_output_addr_uv;
			}

			gJpegParam.pic_size_width = pixel->width;
			gJpegParam.pic_size_height = pixel->height;
			gCsiParam.hor_len = pixel->width;
			gCsiParam.ver_len = pixel->height;
			HAL_CSI_Config(&gCsiParam);
			break;
		}
		case CAMERA_SET_JPEG_MODE:
		{
			int i = 0;
			JPEG_Mode mode = (JPEG_Mode)arg;
			gJpegParam.jpeg_mode = mode;
			if (mode != JPEG_MOD_ONLINE) {
				uint32_t yuv_size = gJpegParam.pic_size_width*gJpegParam.pic_size_height/2*3;
				if (priv->mem_mgmt->yuv_buf.size < yuv_size) {
					CAMERA_ERR("yuv buffer size should >= %dB(1.5*width*height)\n", yuv_size);
					return -1;
				}
			}
			HAL_JPEG_Config(&gJpegParam);
			for (i = 0; i < gJpegParam.outstream_buff_num; i++)
				HAL_JPEG_WriteHeader((uint8_t*)(priv->mem_mgmt->jpeg_buf[i].addr-CAMERA_JPEG_HEADER_LEN));
			break;
		}
		case CAMERA_RESET_CSI_JPEG:
			HAL_JPEG_Reset();
			break;
		case CAMERA_SET_SENSOR_SUBSAMP:
		{
			uint8_t radio = (uint8_t)arg;
			if (priv->sensor_func.ioctl)
				priv->sensor_func.ioctl(SENSOR_SET_SUBSAMP, radio);
			break;
		}
		case CAMERA_SET_JPEG_SCALE:
		{
			uint8_t scale_en = (uint8_t)arg;
			uint8_t scale_radio = 1;
			gJpegParam.jpeg_scale = scale_en ? 1 : 0;
			if (gJpegParam.jpeg_scale) {
				if (gJpegParam.pic_size_width < gCsiParam.hor_len) {
					CAMERA_ERR("not support multiple scale\n");
					OS_MutexUnlock(&priv->lock);
					return -1;
				}
				scale_radio = 2;
			} else {
				if (gJpegParam.pic_size_width >= gCsiParam.hor_len) {
					CAMERA_ERR("not been scaled\n");
					OS_MutexUnlock(&priv->lock);
					return -1;
				}
			}
			gJpegParam.pic_size_width = gCsiParam.hor_len / scale_radio;
			gJpegParam.pic_size_height = gCsiParam.ver_len / scale_radio;
			gJpegParam.csi_output_addr_uv = (uint32_t)priv->mem_mgmt->yuv_buf.addr +
								(gJpegParam.pic_size_width * gJpegParam.pic_size_height);
			gJpegParam.jpeg_input_addr_uv = gJpegParam.csi_output_addr_uv;
			HAL_JPEG_Config(&gJpegParam);
			for (uint8_t i = 0; i < gJpegParam.outstream_buff_num; i++)
				HAL_JPEG_WriteHeader((uint8_t*)(priv->mem_mgmt->jpeg_buf[i].addr-CAMERA_JPEG_HEADER_LEN));
			break;
		}
		default:break;
	}

	OS_MutexUnlock(&priv->lock);

	return 0;
}

