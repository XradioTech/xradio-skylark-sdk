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

#include "common/cmd/cmd_util.h"
#include "driver/component/csi_camera/camera.h"
#include "cmd_camera.h"
#include "kernel/os/os.h"
#include "driver/chip/hal_i2c.h"

//#include "driver/component/csi_camera/gc0328c/drv_gc0328c.h"
//#include "driver/component/csi_camera/gc0329/drv_gc0329.h"
#include "driver/component/csi_camera/gc0308/drv_gc0308.h"

#define CSI_JPEG_PSRAM_SIZE 	(1024*1024)
#define JPEG_MPART_SIZE			(0x2000) //8k
#define CSI_JPEG_SRAM_SIZE 		(80*1024)

#define ALIGN_1K(x)     (((x) + (1023)) & ~(1023))

static CAMERA_Mgmt mem_mgmt;

#define IMAGE_SENSOR_I2CID 		I2C0_ID
#define SENSOR_RESET_PIN        GPIO_PIN_21
#define SENSOR_RESET_PORT       GPIO_PORT_A
#define SENSOR_POWERDOWN_PIN    GPIO_PIN_22
#define SENSOR_POWERDOWN_PORT   GPIO_PORT_A

#define JPEG_ONLINE_MEMPART_ENABLE
//#define JPEG_ONLINE_MEMPART_DISABLE
//#define JPEG_YUV420_NV12_OFFLINE

static CAMERA_Cfg cfg = {
	.jpeg_cfg.jpeg_en = 1,
	.jpeg_cfg.quality = 64,
	.jpeg_cfg.jpeg_clk  = 0, //no use
#ifdef JPEG_ONLINE_MEMPART_DISABLE   // online mode
	.jpeg_cfg.memPartEn = 0,
	.jpeg_cfg.memPartNum = 0,
	.jpeg_cfg.jpeg_mode = JPEG_MOD_ONLINE,
#endif
#ifdef JPEG_ONLINE_MEMPART_ENABLE  // online mem part mode
	.jpeg_cfg.memPartEn = 1,
	.jpeg_cfg.memPartNum = JPEG_MEM_BLOCK4, //0->2 part,1->4 part,2->8 part
	.jpeg_cfg.jpeg_mode = JPEG_MOD_ONLINE,
#endif
#ifdef JPEG_YUV420_NV12_OFFLINE //offline mode
	.jpeg_cfg.memPartEn = 0,
	.jpeg_cfg.memPartNum = 0,
	.jpeg_cfg.jpeg_mode = JPEG_MOD_OFFLINE,
#endif

	.csi_cfg.csi_clk = 24000000, // no use

	/* sensor config */
	.sensor_cfg.i2c_id = IMAGE_SENSOR_I2CID,
	.sensor_cfg.pwcfg.Pwdn_Port = SENSOR_POWERDOWN_PORT,
	.sensor_cfg.pwcfg.Reset_Port = SENSOR_RESET_PORT,
	.sensor_cfg.pwcfg.Pwdn_Pin = SENSOR_POWERDOWN_PIN,
	.sensor_cfg.pwcfg.Reset_Pin = SENSOR_RESET_PIN,
#ifdef JPEG_ONLINE_MEMPART_DISABLE
	.sensor_cfg.pixel_size.width = 320,
	.sensor_cfg.pixel_size.height = 240,
#endif
#ifdef JPEG_ONLINE_MEMPART_ENABLE
	.sensor_cfg.pixel_size.width = 320,
	.sensor_cfg.pixel_size.height = 240,
#endif
#ifdef JPEG_YUV420_NV12_OFFLINE
	.sensor_cfg.pixel_size.width = 160,
	.sensor_cfg.pixel_size.height = 120,
#endif
	.sensor_cfg.pixel_outfmt = YUV422_YUYV,

	.sensor_func.init = HAL_GC0308_Init,
	.sensor_func.deinit = HAL_GC0308_DeInit,
	.sensor_func.ioctl = HAL_GC0308_IoCtl,
};

static int csi_mem_create(CAMERA_JpegCfg *jpeg_cfg, CAMERA_Mgmt *mgmt, SENSOR_PixelSize *pixel_size, uint8_t psram_en)
{
	uint8_t* addr;
//#ifdef __CONFIG_PSRAM
#if 0
	if (psram_en) { //offline mode use more reasonable, online mode not need
		addr = (uint8_t *)psram_malloc(CSI_JPEG_PSRAM_SIZE);
		if (addr == NULL) {
			printf("malloc fail\n");
			return -1;
		}
		HAL_Dcache_SetWriteThrough(0, 1, (uint32_t)addr, (uint32_t)addr + CSI_JPEG_PSRAM_SIZE);
		memset(addr, 0 , CSI_JPEG_PSRAM_SIZE);
		printf("malloc addr: %p -> %p\n", addr, addr + CSI_JPEG_PSRAM_SIZE);
	}
 else
#endif
	{
		mgmt->jpeg_header_buf = (uint8_t*)malloc(CAMERA_JPEG_HEADER_LEN);
		if (mgmt->jpeg_header_buf == NULL) {
			printf("malloc fail\n");
			return -1;
		}
		memset(mgmt->jpeg_header_buf, 0 , CAMERA_JPEG_HEADER_LEN);

		if(jpeg_cfg->jpeg_mode == JPEG_MOD_ONLINE) {
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
				addr = (uint8_t*)malloc(CSI_JPEG_SRAM_SIZE + 2048);//imgbuf;
				if (addr == NULL) {
					printf("malloc fail\n");
					return -1;
				}
				memset(addr, 0 , CSI_JPEG_SRAM_SIZE + 2048);
				printf("malloc addr: %p -> %p\n", addr, addr + CSI_JPEG_SRAM_SIZE + 2048);
				mgmt->online_jpeg_buf = (uint8_t *)ALIGN_1K((uint32_t)addr);
				printf("online_jpeg_buf %p\n", mgmt->online_jpeg_buf);
			}
		} else {//offline mode, the sram space can use limited, so only can get small image,
			addr = (uint8_t*)malloc(CSI_JPEG_SRAM_SIZE + 2048);//imgbuf;
			if (addr == NULL) {
				printf("malloc fail\n");
				return -1;
			}
			memset(addr, 0 , CSI_JPEG_SRAM_SIZE + 2048);
			printf("malloc addr: %p -> %p\n", addr, addr + CSI_JPEG_SRAM_SIZE + 2048);
			mgmt->offline_y_buf = (uint8_t *)(((uint32_t)addr & 0xfffffff0) + 16) ;
			mgmt->offline_uv_buf = (uint8_t *)((((uint32_t)mgmt->offline_y_buf +
				(pixel_size->width * pixel_size->height))& 0xfffffff8) + 8);
			mgmt->offline_jpeg_buf = (uint8_t *)ALIGN_1K((uint32_t)mgmt->offline_uv_buf +
				(pixel_size->width * pixel_size->height / 2));//after yuv data
		}

		mgmt->org_addr = addr;
	}

	return 0;
}

static void csi_mem_destroy()
{
	if (mem_mgmt.org_addr) {
//#ifdef __CONFIG_PSRAM
#if 0
		if (csiPriv.psram_en)
			psram_free(addr);
		else
#endif
			free(mem_mgmt.org_addr);
		mem_mgmt.org_addr = NULL;
	}

	if (mem_mgmt.online_jpeg_mempart_last_buf) {
		free(mem_mgmt.online_jpeg_mempart_last_buf);
		mem_mgmt.online_jpeg_mempart_last_buf = NULL;
	}

	if (mem_mgmt.jpeg_header_buf)
		free(mem_mgmt.jpeg_header_buf);
}

enum cmd_status cmd_camera_init_exec(char *cmd)
{
    /* malloc mem */
	uint8_t psram_en = 0;
	memset(&mem_mgmt, 0, sizeof(CAMERA_Mgmt));
	if (csi_mem_create(&cfg.jpeg_cfg, &mem_mgmt, &cfg.sensor_cfg.pixel_size, psram_en) != 0)
		return CMD_STATUS_FAIL;

	HAL_CAMERA_SetImageBuf(&mem_mgmt);

	/* camera init */
	if (HAL_CAMERA_Init(&cfg) != HAL_OK) {
		CMD_ERR("%s init fail\n", cmd);
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

enum cmd_status cmd_camera_cap_one_image_exec(char *cmd)
{
	uint32_t fm_size;
	int i;

	fm_size = HAL_CAMERA_CaptureOneImage();

	printf("fm_szie %d\n", fm_size);

	if (fm_size == 0 || fm_size == CAMERA_JPEG_HEADER_LEN) {
		printf("cap one image failed\n");
		return CMD_STATUS_FAIL;
	}
	/* jpeg header data*/
	for (i = 0; i< CAMERA_JPEG_HEADER_LEN; i++)
		printf("%02x ", mem_mgmt.jpeg_header_buf[i]);

	/* jpeg body data*/
	for (i = 0; i< fm_size-CAMERA_JPEG_HEADER_LEN; i++) {
		if(cfg.jpeg_cfg.jpeg_mode == JPEG_MOD_ONLINE) {
			if (cfg.jpeg_cfg.memPartEn) {
				printf("%02x ", mem_mgmt.online_jpeg_mempart_last_buf[i]);
			} else {
				printf("%02x ", mem_mgmt.online_jpeg_buf[i]);
			}
		} else { //offline
			printf("%02x ", mem_mgmt.offline_jpeg_buf[i]);
		}
	}

	printf("\n write jpeg ok\n");

	/* yuv420 NV12 data */
	if(cfg.jpeg_cfg.jpeg_mode == JPEG_MOD_OFFLINE) {
		for(i = 0;i < cfg.sensor_cfg.pixel_size.width * cfg.sensor_cfg.pixel_size.height; i++)
			printf("%02x ", mem_mgmt.offline_y_buf[i]);
		for(i = 0;i <cfg.sensor_cfg.pixel_size.width * cfg.sensor_cfg.pixel_size.height/2; i++)
			printf("%02x ", mem_mgmt.offline_uv_buf[i]);

		printf("\n write yuv420 nv12 ok\n");
	}

	return CMD_STATUS_OK;
}

/* user need quickly take data away, otherwise easy data coverage */
/* need pixel small, the cmd example suppose the data can not bigger
 * than 20k, you can set pixel width 160,height 120;
 */
enum cmd_status cmd_camera_cap_video_exec(char *cmd)
{
	int i;
	uint8_t *p;
	uint8_t *tmp_buf;
	uint32_t fm_size[2];

	tmp_buf = (uint8_t *)malloc(2*20*1024);
	memset(tmp_buf, 0, 2*20*1024);
	p = tmp_buf;

	if (HAL_CAMERA_CaptureVideoStart() != HAL_OK) {
		return CMD_STATUS_FAIL;
	}

	for(i=0; i<2; i++) { /* for example, 2 frame continue */
		fm_size[i] = HAL_CAMERA_CaptureVideoGetData();
		printf("fm_size[%d] %d\n", i, fm_size[i]);
		if (fm_size[i] == 0 || fm_size[i] == CAMERA_JPEG_HEADER_LEN) {
			printf("cap video image failed\n");
			return CMD_STATUS_FAIL;
		}

		memcpy(p, mem_mgmt.jpeg_header_buf, CAMERA_JPEG_HEADER_LEN);
		p = p + CAMERA_JPEG_HEADER_LEN;
		if(cfg.jpeg_cfg.jpeg_mode == JPEG_MOD_ONLINE) {
			if (cfg.jpeg_cfg.memPartEn) {
				memcpy(p, mem_mgmt.online_jpeg_mempart_last_buf, fm_size[i]-CAMERA_JPEG_HEADER_LEN);
			} else {
				memcpy(p, mem_mgmt.online_jpeg_buf, fm_size[i]-CAMERA_JPEG_HEADER_LEN);
			}
		} else { //offline
			memcpy(p, mem_mgmt.offline_jpeg_buf, fm_size[i]-CAMERA_JPEG_HEADER_LEN);
		}
		p = p + fm_size[i]-CAMERA_JPEG_HEADER_LEN;
#if 0  /* if you print here, the data may recovered. need quick take data away. */
	for (i = 0; i< CAMERA_JPEG_HEADER_LEN; i++)
		printf("%02x ", mem_mgmt.jpeg_header_buf[i]);

	for (i = 0; i< fm_size-CAMERA_JPEG_HEADER_LEN; i++) {
		if(cfg.jpeg_cfg.jpeg_mode == JPEG_MOD_ONLINE) {
			if (cfg.jpeg_cfg.memPartEn) {
				printf("%02x ", mem_mgmt.online_jpeg_mempart_last_buf[i]);
			} else {
				printf("%02x ", mem_mgmt.online_jpeg_buf[i]);
			}
		} else { //offline
			printf("%02x ", mem_mgmt.offline_jpeg_buf[i]);
		}
	}
#endif
	}

	HAL_CAMERA_CaptureVideoStop();

	for (i = 0; i< fm_size[0]; i++) { /* first frame */
		printf("%02x ", tmp_buf[i]);
	}

	printf("\n write jpeg ok\n");

	for (i = 0; i< fm_size[1]; i++) { /* sencond frame */
		printf("%02x ", tmp_buf[fm_size[0]+i]);
	}

	printf("\n write jpeg ok\n");

	return CMD_STATUS_OK;
}

enum cmd_status cmd_camera_deinit_exec(char *cmd)
{
	HAL_CAMERA_DeInit();

	csi_mem_destroy();

	return CMD_STATUS_OK;
}

static const struct cmd_data g_camera_cmds[] = {
    { "init", cmd_camera_init_exec},
    { "cap_one_image",     cmd_camera_cap_one_image_exec},
    { "cap_video_image",   cmd_camera_cap_video_exec},
    { "deinit",     cmd_camera_deinit_exec},
};

enum cmd_status cmd_camera_exec(char *cmd)
{
	return cmd_exec(cmd, g_camera_cmds, cmd_nitems(g_camera_cmds));
}
