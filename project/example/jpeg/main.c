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
#include "driver/component/csi_camera/gc0308/drv_gc0308.h"
#include "driver/component/csi_camera/gc0328c/drv_gc0328c.h"

#include "driver/chip/hal_i2c.h"

#include "driver/chip/psram/psram.h"

#define IMAGE_SENSOR_I2CID 		I2C0_ID
#define SENSOR_RESET_PIN        GPIO_PIN_15
#define SENSOR_RESET_PORT       GPIO_PORT_A
#define SENSOR_POWERDOWN_PIN    GPIO_PIN_14
#define SENSOR_POWERDOWN_PORT   GPIO_PORT_A

#define JPEG_MEMPART_EN			(0)
#define JPEG_MPART_SIZE			(0x2000) //8k
#define JPEG_SRAM_SIZE 			(180*1024)
#define JPEG_ONLINE_EN			(0)

#define JPEG_PSRAM_EN			(1)
#define JPEG_PSRAM_SIZE			(1024*1024)

#define IMAGE_WIDTH				(640)
#define IMAGE_HEIGHT			(480)

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
#if JPEG_MEMPART_EN
	.jpeg_cfg.memPartEn = 1,
	.jpeg_cfg.memPartNum = JPEG_MEM_BLOCK2, //0->2 part,1->4 part,2->8 part
#else
	.jpeg_cfg.memPartEn = 0,
	.jpeg_cfg.memPartNum = 0,
#endif
	.jpeg_cfg.jpeg_mode = JPEG_ONLINE_EN ? JPEG_MOD_ONLINE : JPEG_MOD_OFFLINE,

	.csi_cfg.csi_clk = 24000000, // no use

	/* sensor config */
	.sensor_cfg.i2c_id = IMAGE_SENSOR_I2CID,
	.sensor_cfg.pwcfg.Pwdn_Port = SENSOR_POWERDOWN_PORT,
	.sensor_cfg.pwcfg.Reset_Port = SENSOR_RESET_PORT,
	.sensor_cfg.pwcfg.Pwdn_Pin = SENSOR_POWERDOWN_PIN,
	.sensor_cfg.pwcfg.Reset_Pin = SENSOR_RESET_PIN,
	.sensor_cfg.pixel_size.width = IMAGE_WIDTH,//640,
	.sensor_cfg.pixel_size.height = IMAGE_HEIGHT,//480,
	.sensor_cfg.pixel_outfmt = YUV422_YUYV,

	.sensor_func.init = SENSOR_FUNC_INIT,
	.sensor_func.deinit = SENSOR_FUNC_DEINIT,
	.sensor_func.ioctl = SENSOR_FUNC_IOCTL,
};

static CAMERA_Mgmt mem_mgmt;

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

	if (jpeg_cfg->memPartEn) {
		addr = (uint8_t*)malloc(JPEG_MPART_SIZE + 2048);//imgbuf;
		if (addr == NULL) {
			printf("malloc fail\n");
			return -1;
		}
		mgmt->online_jpeg_mempart_tmp_buf = (uint8_t *)ALIGN_1K((uint32_t)addr);
		mgmt->online_jpeg_mempart_last_buf = (uint8_t*)malloc(50*1024);//imgbuf;
		if (mgmt->online_jpeg_mempart_last_buf  == NULL) {
			printf("malloc fail\n");
			return -1;
		}
		memset(mgmt->online_jpeg_mempart_last_buf, 0 , 50*1024);
		printf("malloc online_jpeg_mempart_last_buf: %p -> %p\n", mgmt->online_jpeg_mempart_last_buf,
			mgmt->online_jpeg_mempart_last_buf + 50*1024);
	} else {
		if (jpeg_cfg->jpeg_mode == JPEG_MOD_ONLINE) {
			addr = (uint8_t*)malloc(JPEG_SRAM_SIZE + 2048);//imgbuf;
			if (addr == NULL) {
				printf("malloc fail\n");
				return -1;
			}
			memset(addr, 0 , JPEG_SRAM_SIZE + 2048);
			printf("malloc addr: %p -> %p\n", addr, addr + JPEG_SRAM_SIZE + 2048);
			mgmt->jpeg_buf = (uint8_t *)ALIGN_1K((uint32_t)addr);
		} else {
			if (JPEG_PSRAM_EN) {
				addr = (uint8_t*)psram_malloc(JPEG_PSRAM_SIZE + 2048);//imgbuf;
				if (addr == NULL) {
					printf("malloc fail\n");
					return -1;
				}
				memset(addr, 0 , JPEG_PSRAM_SIZE + 2048);
				printf("malloc addr: %p -> %p\n", addr, addr + JPEG_PSRAM_SIZE + 2048);
			} else {
				addr = (uint8_t*)malloc(JPEG_SRAM_SIZE + 2048);//imgbuf;
				if (addr == NULL) {
					printf("malloc fail\n");
						return -1;
				}
				memset(addr, 0 , JPEG_SRAM_SIZE + 2048);
				printf("malloc addr: %p -> %p\n", addr, addr + JPEG_SRAM_SIZE + 2048);
			}
			mgmt->yuv_buf = (uint8_t *)ALIGN_16B((uint32_t)addr);
			mgmt->jpeg_buf = (uint8_t *)ALIGN_1K((uint32_t)mgmt->yuv_buf + CAMERA_JPEG_HEADER_LEN +
				camera_cfg.sensor_cfg.pixel_size.width * camera_cfg.sensor_cfg.pixel_size.height * 3/2);//after yuv data
		}
	}

	mgmt->org_addr = addr;

	return 0;
}

static void camera_mem_destroy()
{
	if (mem_mgmt.org_addr) {
		if (JPEG_PSRAM_EN)
			psram_free(mem_mgmt.org_addr);
		else
			free(mem_mgmt.org_addr);
		mem_mgmt.org_addr = NULL;
	}

	if (mem_mgmt.online_jpeg_mempart_last_buf) {
		free(mem_mgmt.online_jpeg_mempart_last_buf);
		mem_mgmt.online_jpeg_mempart_last_buf = NULL;
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

int camera_get_image()
{
	int res;
	FIL fp;
	uint32_t bw;

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
	uint32_t size = HAL_CAMERA_CaptureImage(fmt, 1);
	uint32_t cost = OS_TicksToMSecs(OS_GetTicks()) - time;

	if (size <= 0) {
		printf("capture image failed\n");
		return -1;
	}
	printf("capture image cost: %dms\n", cost);

	uint8_t *addr;

#if (!JPEG_ONLINE_EN)
	addr = mem_mgmt.yuv_buf;
	res = f_write(&fp_y, addr, size, &bw);
	if (res != FR_OK || bw < size) {
		printf("write fail(%d), line%d..\n", res, __LINE__);
		return -1;
	}

	printf("write YUV image ok\n");
	f_close(&fp_y);
#endif

	if (camera_cfg.jpeg_cfg.jpeg_en) {
		uint32_t encode_size = HAL_CAMERA_CaptureImage(CAMERA_OUT_JPEG, 0);

		printf("jpeg image szie: %dbytes\n", encode_size);

		/* jpeg data*/
		encode_size += CAMERA_JPEG_HEADER_LEN;
		addr = camera_cfg.jpeg_cfg.memPartEn ?  mem_mgmt.online_jpeg_mempart_last_buf : (mem_mgmt.jpeg_buf - CAMERA_JPEG_HEADER_LEN);
		res = f_write(&fp, addr, encode_size, &bw);
		if (res != FR_OK || bw < encode_size) {
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
	HAL_CAMERA_DeInit();

	camera_mem_destroy();
}

int main(void)
{
	platform_init();

	printf("jpeg demo started\n\n");

	if (fs_init() != 0)
		return -1;

	camera_init();

	camera_get_image();

	camera_deinit();

	fs_deinit();

	printf("jpeg demo over\n");

	return 0;
}

