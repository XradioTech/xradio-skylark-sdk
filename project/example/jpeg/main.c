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
#include "kernel/os/os.h"
#include "fs/fatfs/ff.h"
#include "common/framework/platform_init.h"
#include "common/framework/fs_ctrl.h"
#include "driver/component/csi_camera/camera.h"
#include "driver/chip/hal_i2c.h"

//#define SENSOR_GC0308
#ifdef SENSOR_GC0308
#include "driver/component/csi_camera/gc0308/drv_gc0308.h"

#define SENSOR_FUNC_INIT		HAL_GC0308_Init
#define SENSOR_FUNC_DEINIT		HAL_GC0308_DeInit
#define SENSOR_FUNC_IOCTL		HAL_GC0308_IoCtl
#else
#include "driver/component/csi_camera/gc0328c/drv_gc0328c.h"

#define SENSOR_FUNC_INIT		HAL_GC0328C_Init
#define SENSOR_FUNC_DEINIT		HAL_GC0328C_DeInit
#define SENSOR_FUNC_IOCTL		HAL_GC0328C_IoCtl
#endif

#define JPEG_PSRAM_EN			(1)
#if JPEG_PSRAM_EN
#include "driver/chip/psram/psram.h"
#include "sys/dma_heap.h"

#define JPEG_PSRAM_SIZE			(620*1024)
#endif

#define SENSOR_I2C_ID 			I2C0_ID
#define SENSOR_RESET_PIN        GPIO_PIN_15
#define SENSOR_RESET_PORT       GPIO_PORT_A
#define SENSOR_POWERDOWN_PIN    GPIO_PIN_14
#define SENSOR_POWERDOWN_PORT   GPIO_PORT_A

#define JPEG_ONLINE_EN			(0)
#define JPEG_SRAM_SIZE 			(180*1024)

#define JPEG_MPART_EN			(0)
#if JPEG_MPART_EN
#define JPEG_BUFF_SIZE 			(8*1024)
#else
#define JPEG_BUFF_SIZE  		(50*1024)
#endif

#define JPEG_IMAGE_WIDTH		(640)
#define JPEG_IMAGE_HEIGHT		(480)

static CAMERA_Cfg camera_cfg = {
	/* jpeg config */
	.jpeg_cfg.jpeg_en = 1,
	.jpeg_cfg.quality = 64,
	.jpeg_cfg.jpeg_clk  = 0, //no use
	.jpeg_cfg.memPartEn = JPEG_MPART_EN,
	.jpeg_cfg.memPartNum = JPEG_MPART_EN ? JPEG_MEM_BLOCK4 : 0,
	.jpeg_cfg.jpeg_mode = JPEG_ONLINE_EN ? JPEG_MOD_ONLINE : JPEG_MOD_OFFLINE,
	.jpeg_cfg.width = JPEG_IMAGE_WIDTH,
	.jpeg_cfg.height = JPEG_IMAGE_HEIGHT,

	/* csi config */
	.csi_cfg.csi_clk = 24000000, // no use

	/* sensor config */
	.sensor_cfg.i2c_id = SENSOR_I2C_ID,
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
static OS_Semaphore_t sem;

#if JPEG_MPART_EN
static uint8_t* gmpartaddr;
static uint32_t gmpartsize;
#endif

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

static int camera_mem_create(CAMERA_JpegCfg *jpeg_cfg, CAMERA_Mgmt *mgmt)
{
	uint8_t* addr;
	uint8_t* end_addr;

	if (JPEG_PSRAM_EN) {
		addr = (uint8_t*)dma_malloc(JPEG_PSRAM_SIZE, DMAHEAP_PSRAM);
		if (addr == NULL) {
			printf("malloc fail\n");
			return -1;
		}
		memset(addr, 0 , JPEG_PSRAM_SIZE);
		end_addr = addr + JPEG_PSRAM_SIZE;
	} else {
		addr = (uint8_t*)malloc(JPEG_SRAM_SIZE);//imgbuf;
		if (addr == NULL) {
			printf("malloc fail\n");
				return -1;
		}
		memset(addr, 0 , JPEG_SRAM_SIZE);
		end_addr = addr + JPEG_SRAM_SIZE;
	}
	printf("malloc addr: %p -> %p\n", addr, end_addr);

	mgmt->yuv_buf.addr = (uint8_t *)ALIGN_16B((uint32_t)addr);
	mgmt->yuv_buf.size = camera_cfg.jpeg_cfg.width*camera_cfg.jpeg_cfg.height*3/2;
#if JPEG_ONLINE_EN
	mgmt->jpeg_buf[0].addr = (uint8_t *)ALIGN_1K((uint32_t)addr + CAMERA_JPEG_HEADER_LEN);
#else
	mgmt->jpeg_buf[0].addr = (uint8_t *)ALIGN_1K((uint32_t)mgmt->yuv_buf.addr + CAMERA_JPEG_HEADER_LEN +
							mgmt->yuv_buf.size);//after yuv data
#endif
	mgmt->jpeg_buf[0].size = JPEG_BUFF_SIZE;

	//mgmt->jpeg_buf[1].addr = (uint8_t *)ALIGN_1K((uint32_t)mgmt->jpeg_buf[0].addr+JPEG_BUFF_SIZE+ CAMERA_JPEG_HEADER_LEN);
	//mgmt->jpeg_buf[1].size = JPEG_BUFF_SIZE;


	if ((mgmt->yuv_buf.addr + mgmt->yuv_buf.size) > end_addr ||
			(mgmt->jpeg_buf[0].addr + mgmt->jpeg_buf[0].size) > end_addr) {
		printf("aadr exceeded\n");
		return -1;
	}
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

#if JPEG_MPART_EN
static void camera_cb(CAMERA_CapStatus status, void *arg)
{
	switch (status) {
		case CAMERA_STATUS_MPART:
		{
			static uint32_t offset = 0;
			uint8_t* addr = gmpartaddr;
			CAMERA_MpartBuffInfo *stream = (CAMERA_MpartBuffInfo*)arg;

			memcpy(addr+offset, mem_mgmt.jpeg_buf[stream->buff_index].addr+(uint32_t)stream->buff_offset, stream->size);
			offset += stream->size;
			if (stream->tail) { /* encode one jpeg finish */
				gmpartsize = offset;
				offset = 0;
				OS_SemaphoreRelease(&sem);
			}
			break;
		}
		default:
			break;
	}
}

#if 0
int camera_get_mpart_jpeg()
{
	int res = 0;
	FIL fp;
	uint32_t bw;
	gmpartaddr = (uint8_t*)dma_malloc(45*1024, DMAHEAP_PSRAM);
	if (gmpartaddr == NULL) {
		printf("malloc fail\n");
		return -1;
	}

	f_unlink("test.jpg");
	res = f_open(&fp, "test.jpg", FA_WRITE | FA_CREATE_NEW);
	if (res != FR_OK) {
		printf("open file error %d\n", res);
		res = -1;
		goto exit_2;
	}

	res = f_write(&fp, mem_mgmt.jpeg_buf[0].addr-CAMERA_JPEG_HEADER_LEN, CAMERA_JPEG_HEADER_LEN, &bw);
	if (res != FR_OK || bw < CAMERA_JPEG_HEADER_LEN) {
		printf("write fail(%d), line%d..\n", res, __LINE__);
		res = -1;
		goto exit_1;
	}

	HAL_CAMERA_CaptureStream();

	if (OS_SemaphoreWait(&sem, 5000) != OS_OK) {
		printf("sem wait fail\n");
		res = -1;
		goto exit_1;
	}

	res = f_write(&fp, gmpartaddr, gmpartsize, &bw);
	if (res != FR_OK || bw < gmpartsize) {
		printf("write fail(%d), line%d..\n", res, __LINE__);
		res = -1;
		goto exit_1;
	}
    printf("write jpeg ok\n");
exit_1:
	f_close(&fp);
exit_2:
	dma_free(gmpartaddr, DMAHEAP_PSRAM);

	return res;
}
#endif

int camera_get_mpart_jpeg()
{
	int res = 0;

	gmpartaddr = (uint8_t*)dma_malloc(45*1024, DMAHEAP_PSRAM);
	if (gmpartaddr == NULL) {
		printf("malloc fail\n");
		return -1;
	}
	memset(gmpartaddr, 0 , 45*1024);

	HAL_CAMERA_CaptureMpartStart(0);

	if (OS_SemaphoreWait(&sem, 5000) != OS_OK) {
		printf("sem wait fail\n");
		dma_free(gmpartaddr, DMAHEAP_PSRAM);
		return -1;
	}

	for (int i =0; i < (CAMERA_JPEG_HEADER_LEN); i++)
		printf("%02x ", *(mem_mgmt.jpeg_buf[0].addr+i-CAMERA_JPEG_HEADER_LEN));
	for (int i =0; i < (gmpartsize); i++)
		printf("%02x ", *(gmpartaddr+i));
	printf("\nprint jpeg ok\n");

	OS_MSleep(300);

	dma_free(gmpartaddr, DMAHEAP_PSRAM);

	return res;
}

#endif

static int camera_init()
{
	if (OS_SemaphoreCreateBinary(&sem) != OS_OK) {
        printf("sem create fail\n");
        return -1;
    }

    /* malloc mem */
	memset(&mem_mgmt, 0, sizeof(CAMERA_Mgmt));
	if (camera_mem_create(&camera_cfg.jpeg_cfg, &mem_mgmt) != 0)
		return -1;

	/* camera init */
	camera_cfg.mgmt = &mem_mgmt;
#if JPEG_MPART_EN
	camera_cfg.cb = &camera_cb;
#endif
	if (HAL_CAMERA_Init(&camera_cfg) != HAL_OK) {
		printf("%s fail, %d\n", __func__, __LINE__);
		return -1;
	}

	return 0;
}

int camera_get_image()
{
	int res;
	FIL fp;
	uint32_t bw;
	CAMERA_JpegBuffInfo jpeg_info;

	f_unlink("test.jpg");
	res = f_open(&fp, "test.jpg", FA_WRITE | FA_CREATE_NEW);
	if (res != FR_OK) {
		printf("open file error %d\n", res);
		return -1;
	}

#if (!JPEG_ONLINE_EN)
	FIL fp_y;
	f_unlink("test.YUV");
	res = f_open(&fp_y, "test.YUV", FA_WRITE | FA_CREATE_NEW);
	if (res != FR_OK) {
		printf("open file error %d\n", res);
		return -1;
	}
#endif

	printf("start to capture image...\n");
	CAMERA_OutFmt fmt;
	fmt = JPEG_ONLINE_EN ? CAMERA_OUT_JPEG : CAMERA_OUT_YUV420;

	uint32_t time = OS_TicksToMSecs(OS_GetTicks());
	int size = HAL_CAMERA_CaptureImage(fmt, &jpeg_info, 1);
	uint32_t cost = OS_TicksToMSecs(OS_GetTicks()) - time;

	if (size < 0) {
		printf("capture image failed\n");
		return -1;
	}
	printf("capture image cost: %dms\n", cost);

	uint8_t *addr;

#if (!JPEG_ONLINE_EN)
	addr = mem_mgmt.yuv_buf.addr;
	res = f_write(&fp_y, addr, size, &bw);
	if (res != FR_OK || bw < size) {
		printf("write fail(%d), line%d..\n", res, __LINE__);
		return -1;
	}
	printf("write YUV image ok\n");
	f_close(&fp_y);
#endif

	if (camera_cfg.jpeg_cfg.jpeg_en) {
		size = HAL_CAMERA_CaptureImage(CAMERA_OUT_JPEG, &jpeg_info, 0);

		printf("jpeg image szie: %dbytes\n", jpeg_info.size);

		/* jpeg data*/
		jpeg_info.size += CAMERA_JPEG_HEADER_LEN;
		addr = mem_mgmt.jpeg_buf[jpeg_info.buff_index].addr - CAMERA_JPEG_HEADER_LEN;

		res = f_write(&fp, addr, jpeg_info.size, &bw);
		if (res != FR_OK || bw < jpeg_info.size) {
			printf("write fail(%d), line%d..\n", res, __LINE__);
			return -1;
		}
		f_close(&fp);
		printf("write jpeg image ok\n");
	}

	return 0;
}

static void camera_deinit()
{
	OS_SemaphoreDelete(&sem);

	HAL_CAMERA_DeInit();

	camera_mem_destroy();
}

int camera_print_image()
{
	CAMERA_JpegBuffInfo jpeg_info;

	for (int cnt = 0; cnt < 20; cnt++) {
		uint32_t size = HAL_CAMERA_CaptureImage(CAMERA_OUT_JPEG, &jpeg_info, 1);
		if (size < 0) {
			printf("capture fail\n");
			return -1;
		}

		for (int i =0; i < (jpeg_info.size+CAMERA_JPEG_HEADER_LEN); i++)
			printf("%02x ", *(mem_mgmt.jpeg_buf[jpeg_info.buff_index].addr+i-CAMERA_JPEG_HEADER_LEN));
		printf("\nprint jpeg ok\n");

		OS_MSleep(30);
	}
	return 0;
}

int main(void)
{
	platform_init();

	printf("jpeg demo started\n\n");

	if (fs_init() != 0)
		return -1;

	camera_init();

#if JPEG_MPART_EN
	camera_get_mpart_jpeg();
#else
	camera_get_image();
	//camera_print_image();
#endif

	camera_deinit();

	fs_deinit();

	printf("jpeg demo over\n");

	return 0;
}

