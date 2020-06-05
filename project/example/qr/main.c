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

#include "sys/dma_heap.h"
#include "common/framework/platform_init.h"
#include "fs/fatfs/ff.h"
#include "common/framework/fs_ctrl.h"
#include "driver/component/csi_camera/camera.h"
#include "driver/component/csi_camera/gc0308/drv_gc0308.h"
#include "driver/component/csi_camera/gc0328c/drv_gc0328c.h"
#include "driver/chip/hal_i2c.h"
#include "zbar/zbar_qr.h"

#define IMAGE_SENSOR_I2CID 		I2C0_ID
#define SENSOR_RESET_PIN        GPIO_PIN_15
#define SENSOR_RESET_PORT       GPIO_PORT_A
#define SENSOR_POWERDOWN_PIN    GPIO_PIN_14
#define SENSOR_POWERDOWN_PORT   GPIO_PORT_A

#define JPEG_SRAM_SIZE 			(200*1024)
#define JPEG_ONLINE_EN			(0)

#define JPEG_PSRAM_EN			(1)
#define JPEG_PSRAM_SIZE			(200*1024)

#define JPEG_BUFF_SIZE  		(50*1024)

#define IMAGE_WIDTH				(320)
#define IMAGE_HEIGHT			(240)

#if 0
#define SENSOR_FUNC_INIT	HAL_GC0308_Init
#define SENSOR_FUNC_DEINIT	HAL_GC0308_DeInit
#define SENSOR_FUNC_IOCTL	HAL_GC0308_IoCtl
#else
#define SENSOR_FUNC_INIT	HAL_GC0328C_Init
#define SENSOR_FUNC_DEINIT	HAL_GC0328C_DeInit
#define SENSOR_FUNC_IOCTL	HAL_GC0328C_IoCtl
#endif

static CAMERA_Cfg camera_cfg = {
	.jpeg_cfg.jpeg_en = 1,
	.jpeg_cfg.quality = 64,
	.jpeg_cfg.jpeg_clk  = 0, //no use
	.jpeg_cfg.memPartEn = 0,
	.jpeg_cfg.memPartNum = 0,
	.jpeg_cfg.jpeg_mode = JPEG_ONLINE_EN ? JPEG_MOD_ONLINE : JPEG_MOD_OFFLINE,
	.jpeg_cfg.width = IMAGE_WIDTH,
	.jpeg_cfg.height = IMAGE_HEIGHT,
	.csi_cfg.csi_clk = 24000000, // no use
	.csi_cfg.hor_start = 0,
	.csi_cfg.ver_start = 0,

	/* sensor config */
	.sensor_cfg.i2c_id = IMAGE_SENSOR_I2CID,
	.sensor_cfg.pwcfg.Pwdn_Port = SENSOR_POWERDOWN_PORT,
	.sensor_cfg.pwcfg.Reset_Port = SENSOR_RESET_PORT,
	.sensor_cfg.pwcfg.Pwdn_Pin = SENSOR_POWERDOWN_PIN,
	.sensor_cfg.pwcfg.Reset_Pin = SENSOR_RESET_PIN,

	.sensor_func.init = SENSOR_FUNC_INIT,
	.sensor_func.deinit = SENSOR_FUNC_DEINIT,
	.sensor_func.ioctl = SENSOR_FUNC_IOCTL,
};

static uint8_t* gmemaddr;
static CAMERA_Mgmt mem_mgmt;

static int camera_mem_create(CAMERA_JpegCfg *jpeg_cfg, CAMERA_Mgmt *mgmt)
{
	uint8_t* addr;

	if (JPEG_PSRAM_EN) {
		addr = (uint8_t*)dma_malloc(JPEG_PSRAM_SIZE, DMAHEAP_PSRAM);
		if (addr == NULL) {
			printf("malloc fail\n");
			return -1;
		}
		memset(addr, 0 , JPEG_PSRAM_SIZE);
		printf("malloc addr: %p -> %p\n", addr, addr + JPEG_PSRAM_SIZE);
	} else {
		addr = (uint8_t*)malloc(JPEG_SRAM_SIZE);
		if (addr == NULL) {
			printf("malloc fail\n");
				return -1;
		}
		memset(addr, 0 , JPEG_SRAM_SIZE);
		printf("malloc addr: %p -> %p\n", addr, addr + JPEG_SRAM_SIZE);
	}
	mgmt->yuv_buf.addr = (uint8_t *)ALIGN_16B((uint32_t)addr);
	mgmt->yuv_buf.size = camera_cfg.jpeg_cfg.width*camera_cfg.jpeg_cfg.height*3/2;
	mgmt->jpeg_buf[0].addr = (uint8_t *)ALIGN_1K((uint32_t)mgmt->yuv_buf.addr + CAMERA_JPEG_HEADER_LEN +
							mgmt->yuv_buf.size);
	mgmt->jpeg_buf[0].size = JPEG_BUFF_SIZE;

	gmemaddr = addr;

	return 0;
}

static void camera_mem_destroy()
{
	if (gmemaddr) {
		if (JPEG_PSRAM_EN)
			dma_free(gmemaddr, DMAHEAP_PSRAM);
		else
			free(gmemaddr);
		gmemaddr = NULL;
	}

}

static int camera_init()
{
    /* malloc mem */
	memset(&mem_mgmt, 0, sizeof(CAMERA_Mgmt));
	if (camera_mem_create(&camera_cfg.jpeg_cfg, &mem_mgmt) != 0)
		return -1;

	/* camera init */
	camera_cfg.mgmt = &mem_mgmt;
	if (HAL_CAMERA_Init(&camera_cfg) != HAL_OK) {
		printf("%s fail, %d\n", __func__, __LINE__);
		return -1;
	}

	return 0;
}

static void camera_deinit()
{
	HAL_CAMERA_DeInit();

	camera_mem_destroy();
}

static int camera_get_image()
{
	CAMERA_JpegBuffInfo info;
	int size = HAL_CAMERA_CaptureImage(CAMERA_OUT_YUV420, &info, 1);
	if (size <= 0) {
		printf("capture fail...\n");
		return -1;
	}

	uint8_t *buf = mem_mgmt.yuv_buf.addr;
	(void)buf;
#if 0
	int ret = 0;

	FIL fp;
	uint32_t br;

	f_unlink("test.yuv");
	ret = f_open(&fp, "test.yuv", FA_WRITE | FA_CREATE_NEW);
	if (ret != FR_OK) {
	   printf("open file error %d\n", ret);
	   return -1;
	}

	ret = f_write(&fp, buf, size, &br);
	if (ret != FR_OK || size > br) {
	   printf("file write fail, require %d bytes, return %d bytes.\n",
		   size, br);
	}

	ret = f_close(&fp);
#endif

	return 0;
}

static int fs_init()
{
	if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
		printf("mount fail\n");
		return -1;
	}

	printf("mount success\n");

	return 0;
}

static int fs_deinit()
{
	if (fs_ctrl_unmount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
		printf("unmount fail\n");
		return -1;
	}
	printf("\nunmount success\n");

	return 0;
}

int qr_scan_test()
{
	camera_init();

    void *qr = zbar_qr_create();
	if (!qr) {
		printf("zbar qr create fail\n");
		return -1;
	}

    uint8_t *raw = mem_mgmt.yuv_buf.addr;
	uint32_t width = IMAGE_WIDTH, height = IMAGE_HEIGHT;
	zbar_qr_set_data(qr, raw, width, height);

	uint32_t n;
	const char *result;

	while (1) {
		if (camera_get_image() == 0) {
			printf("QR start scan..\n");
			n = zbar_qr_scan(qr, &result);
			/* extract results */
			if (n > 0)
				printf("decoded symbol \"%s\"\n", result);
		} else {
			printf("camera take photo fail\n");
		}
		OS_MSleep(1000);
	}

    /* clean up */
 	zbar_qr_destroy(qr);

	camera_deinit();

    return(0);
}

int main(void)
{
	platform_init();

	if (fs_init() != 0)
		return -1;

	qr_scan_test();

	fs_deinit();

	return 0;
}
