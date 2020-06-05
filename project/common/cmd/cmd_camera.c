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

#define CSI_JPEG_PSRAM_EN		(1)
#if CSI_JPEG_PSRAM_EN
#include "sys/dma_heap.h"

#define CSI_JPEG_PSRAM_SIZE		(520*1024)
#endif

#define CSI_JPEG_MEMPART_EN		(0)
#if CSI_JPEG_MEMPART_EN
#define CSI_JPEG_BUFF_SIZE		(8*1024)
#else
#define CSI_JPEG_BUFF_SIZE		(45*1024)
#endif

#define CSI_JPEG_VIDEO_EN		(0)
#define CSI_JPEG_ONLINE_EN		(1)
#define CSI_JPEG_SRAM_SIZE		(100*1024)

#define IMAGE_SENSOR_I2CID 		I2C0_ID
#define SENSOR_RESET_PIN        GPIO_PIN_15
#define SENSOR_RESET_PORT       GPIO_PORT_A
#define SENSOR_POWERDOWN_PIN    GPIO_PIN_14
#define SENSOR_POWERDOWN_PORT   GPIO_PORT_A

static uint8_t* gmemaddr;
static CAMERA_Mgmt mem_mgmt;

static CAMERA_Cfg camera_cfg = {
	.jpeg_cfg.jpeg_en = 1,
	.jpeg_cfg.quality = 64,
	.jpeg_cfg.jpeg_clk  = 0, //no use
	.jpeg_cfg.memPartEn = CSI_JPEG_MEMPART_EN,
	.jpeg_cfg.memPartNum = CSI_JPEG_MEMPART_EN ? JPEG_MEM_BLOCK4 : 0,
	.jpeg_cfg.jpeg_mode = CSI_JPEG_ONLINE_EN ? JPEG_MOD_ONLINE : JPEG_MOD_OFFLINE,
	.jpeg_cfg.width = 640,
	.jpeg_cfg.height = 480,

	.csi_cfg.csi_clk = 24000000, // no use

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

static int camera_mem_create(CAMERA_JpegCfg *jpeg_cfg, CAMERA_Mgmt *mgmt)
{
	uint8_t* addr;
	uint8_t* end_addr;

#if CSI_JPEG_PSRAM_EN
	addr = (uint8_t*)dma_malloc(CSI_JPEG_PSRAM_SIZE, DMAHEAP_PSRAM);
	if (addr == NULL) {
		CMD_SYSLOG("malloc fail\n");
		return -1;
	}
	cmd_memset(addr, 0 , CSI_JPEG_PSRAM_SIZE);
	end_addr = addr + CSI_JPEG_PSRAM_SIZE;
#else
	addr = (uint8_t*)cmd_malloc(CSI_JPEG_SRAM_SIZE);//imgbuf;
	if (addr == NULL) {
		CMD_SYSLOG("malloc fail\n");
		return -1;
	}
	cmd_memset(addr, 0 , CSI_JPEG_SRAM_SIZE);
	end_addr = addr + CSI_JPEG_SRAM_SIZE;
#endif

	CMD_SYSLOG("malloc addr: %p -> %p\n", addr, end_addr);

	mgmt->yuv_buf.addr = (uint8_t *)ALIGN_16B((uint32_t)addr);
	mgmt->yuv_buf.size = camera_cfg.jpeg_cfg.width*camera_cfg.jpeg_cfg.height/2*3;

#if CSI_JPEG_ONLINE_EN
	mgmt->jpeg_buf[0].addr = (uint8_t *)ALIGN_1K((uint32_t)addr + CAMERA_JPEG_HEADER_LEN);
#if (CSI_JPEG_VIDEO_EN && !CSI_JPEG_MEMPART_EN)
	mgmt->jpeg_buf[1].addr = (uint8_t *)ALIGN_1K((uint32_t)mgmt->jpeg_buf[0].addr + CSI_JPEG_BUFF_SIZE + CAMERA_JPEG_HEADER_LEN);
	mgmt->jpeg_buf[1].size = CSI_JPEG_BUFF_SIZE;
#endif
#else
	mgmt->jpeg_buf[0].addr = (uint8_t *)ALIGN_1K((uint32_t)mgmt->yuv_buf.addr + CAMERA_JPEG_HEADER_LEN +
							mgmt->yuv_buf.size);//after yuv data
#endif
	mgmt->jpeg_buf[0].size = CSI_JPEG_BUFF_SIZE;

	if ((mgmt->yuv_buf.addr + mgmt->yuv_buf.size) > end_addr ||
			(mgmt->jpeg_buf[0].addr + mgmt->jpeg_buf[0].size) > end_addr) {
		CMD_ERR("aadr exceeded\n");
		return -1;
	}
	gmemaddr = addr;

	return 0;
}

static void camera_mem_destroy()
{
	if (gmemaddr) {
#if CSI_JPEG_PSRAM_EN
		dma_free(gmemaddr, DMAHEAP_PSRAM);
#else
		cmd_free(gmemaddr);
#endif
		gmemaddr = NULL;
	}
}

enum cmd_status cmd_camera_init_exec(char *cmd)
{

    /* malloc mem */
	cmd_memset(&mem_mgmt, 0, sizeof(CAMERA_Mgmt));
	if (camera_mem_create(&camera_cfg.jpeg_cfg, &mem_mgmt) != 0)
		return CMD_STATUS_FAIL;

	/* camera init */
	camera_cfg.mgmt = &mem_mgmt;
	if (HAL_CAMERA_Init(&camera_cfg) != HAL_OK) {
		CMD_ERR("%s init fail\n", cmd);
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

enum cmd_status cmd_camera_cap_one_image_exec(char *cmd)
{
	int i;
	uint8_t *addr;
	int size;
	CAMERA_JpegBuffInfo jpeg_info;

	size = HAL_CAMERA_CaptureImage(CAMERA_OUT_JPEG, &jpeg_info, 1);
	if (size < 0) {
		CMD_ERR("cap one image failed\n");
		return CMD_STATUS_FAIL;
	}
	CMD_SYSLOG("fm_szie %d\n", jpeg_info.size);

	/* jpeg header data*/
	addr = mem_mgmt.jpeg_buf[jpeg_info.buff_index].addr - CAMERA_JPEG_HEADER_LEN;
	for (i = 0; i< CAMERA_JPEG_HEADER_LEN; i++)
		CMD_SYSLOG("%02x ", addr[i]);

	/* jpeg body data*/
	for (i = 0; i< jpeg_info.size; i++) {
		addr = mem_mgmt.jpeg_buf[jpeg_info.buff_index].addr;
		CMD_SYSLOG("%02x ", addr[i]);
	}
	CMD_SYSLOG("\n write jpeg ok\n");

	/* yuv420 NV12 data */
	if(camera_cfg.jpeg_cfg.jpeg_mode == JPEG_MOD_OFFLINE) {
		for(i = 0;i < camera_cfg.jpeg_cfg.width*camera_cfg.jpeg_cfg.height*3/2; i++)
			CMD_SYSLOG("%02x ", mem_mgmt.yuv_buf.addr[i]);
		CMD_SYSLOG("\n write yuv420 nv12 ok\n");
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
	uint8_t *tmp_buf[2];
	uint32_t tmp_size[2];
	CAMERA_JpegBuffInfo jpeg_info;

	tmp_buf[0] = (uint8_t *)cmd_malloc(30*1024);
	if (tmp_buf[0] == NULL) {
		CMD_ERR("malloc tmp_buf[0] failed\n");
		return CMD_STATUS_FAIL;
	}
	tmp_buf[1] = (uint8_t *)cmd_malloc(30*1024);
	if (tmp_buf[1] == NULL) {
		CMD_ERR("malloc tmp_buf[1] failed\n");
		cmd_free(tmp_buf[0]);
		return CMD_STATUS_FAIL;
	}
	cmd_memset(tmp_buf[0], 0, 30*1024);
	cmd_memset(tmp_buf[1], 0, 30*1024);

	if (HAL_CAMERA_CaptureVideoStart() != HAL_OK) {
		CMD_ERR("start video failed\n");
		cmd_free(tmp_buf[0]);
		cmd_free(tmp_buf[1]);
		return CMD_STATUS_FAIL;
	}

	for(i=0; i< 2; i++) { /* for example, 2 frame continue */
		int ret = HAL_CAMERA_CaptureVideoData(&jpeg_info);
		CMD_SYSLOG("frame index=%d size=%d\n", jpeg_info.buff_index, jpeg_info.size);
		if (ret != 0) {
			CMD_ERR("cap video image failed\n");
			cmd_free(tmp_buf[0]);
			cmd_free(tmp_buf[1]);
			return CMD_STATUS_FAIL;
		}
		tmp_size[i] = jpeg_info.size + CAMERA_JPEG_HEADER_LEN;
		cmd_memcpy(tmp_buf[i], mem_mgmt.jpeg_buf[jpeg_info.buff_index].addr-CAMERA_JPEG_HEADER_LEN, jpeg_info.size + CAMERA_JPEG_HEADER_LEN);
	}

	HAL_CAMERA_CaptureVideoStop();

	for (i = 0; i< tmp_size[0]; i++) { /* first frame */
		CMD_SYSLOG("%02x ", *(tmp_buf[0] + i));
	}
	CMD_SYSLOG("\nwrite jpeg ok\n");

	for (i = 0; i< tmp_size[1]; i++) { /* sencond frame */
		CMD_SYSLOG("%02x ", *(tmp_buf[1] + i));
	}
	CMD_SYSLOG("\nwrite jpeg ok\n");

	cmd_free(tmp_buf[0]);
	cmd_free(tmp_buf[1]);

	return CMD_STATUS_OK;
}

enum cmd_status cmd_camera_deinit_exec(char *cmd)
{
	HAL_CAMERA_DeInit();

	camera_mem_destroy();

	return CMD_STATUS_OK;
}

static const struct cmd_data g_camera_cmds[] = {
    { "init",				cmd_camera_init_exec},
    { "cap_one_image",		cmd_camera_cap_one_image_exec},
    { "cap_video_image",	cmd_camera_cap_video_exec},
    { "deinit",     		cmd_camera_deinit_exec},
};

enum cmd_status cmd_camera_exec(char *cmd)
{
	return cmd_exec(cmd, g_camera_cmds, cmd_nitems(g_camera_cmds));
}
