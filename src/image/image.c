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

#include "stdlib.h"

#include "image/image.h"
#include "image/flash.h"
#include "image_debug.h"

#define IMAGE_INVALID_FLASH 	(0xFF)
#define IMAGE_INVALID_OTA_SIZE	(0xFFFFFF)

/* Definition of OTA area parameters in bootloader's section_header_t::priv[] */
#define BLSH_OTA_FLASH(sh)	((sh)->priv[0] & 0xFF)			 	/* flash ID of OTA area */
#define BLSH_OTA_SIZE(sh)	(((sh)->priv[0] >> 8) & 0xFFFFFF) 	/* size of OTA area */
#define BLSH_OTA_ADDR(sh)	((sh)->priv[1])						/* start addr of OTA area */

typedef struct {
	uint32_t	id;
	uint32_t	addr;
} sec_addr_t;

typedef struct {
	image_ota_param_t	iop;
	uint8_t				sec_num[IMAGE_SEQ_NUM];
	sec_addr_t		   *sec_addr[IMAGE_SEQ_NUM];
} image_priv_t;

static void image_clear_sec_addr(image_priv_t *img)
{
	image_seq_t seq;

	for (seq = 0; seq < IMAGE_SEQ_NUM; seq++) {
		img->sec_num[seq] = 0;
		if (img->sec_addr[seq]) {
			free(img->sec_addr[seq]);
			img->sec_addr[seq] = NULL;
		}
	}
}

#ifndef __CONFIG_BOOTLOADER

#if IMAGE_OPT_BL_COMPATIBILITY
/*
 * @brief Get the version of bootloader
 * @return Bootloader's version
 */
static uint32_t image_get_bl_ver(void)
{
	uint32_t ver;
	image_ota_param_t *iop = &image_priv.iop;
	uint32_t addr = IMG_BL_ADDR(iop) + offsetof(section_header_t, version);

	if (flash_read(IMG_BL_FLASH(iop), addr, &ver, sizeof(ver)) != sizeof(ver)) {
		return IMAGE_INVALID_BL_VER;
	}
	IMAGE_DBG("bl ver %u\n", ver);
	return ver;
}
#endif /* IMAGE_OPT_BL_COMPATIBILITY */

static image_seq_t image_init_running_seq(const image_ota_param_t *iop)
{
	image_cfg_t cfg;
	image_seq_t	seq = 0; /* default to the first image seq when error occur */

	if (iop->ota_addr == IMAGE_INVALID_ADDR) {
		IMAGE_DBG("invalid ota addr %#x\n", iop->ota_addr);
		return seq;
	}

	if (image_get_cfg(&cfg) != 0) {
		IMAGE_ERR("read img cfg failed\n");
		return seq;
	}
	IMAGE_DBG("img seq %d, state %d\n", cfg.seq, cfg.state);

#if IMAGE_OPT_BL_COMPATIBILITY
	uint32_t bl_ver = image_get_bl_ver();
	if (bl_ver == IMAGE_INVALID_BL_VER) {
		IMAGE_ERR("invalid bl ver %u\n", bl_ver);
		return seq;
	}

	/* for bl version < 0.3, image sequence MUST < 2 */
	if (bl_ver < 3 && cfg.seq > 1) {
		IMAGE_ERR("invalid img seq %u, bl ver %u\n", cfg.seq, bl_ver);
		return seq;
	}

	if (bl_ver < 2) {
		/* for bl version < 0.2, the verfied state is set by app.bin only */
		seq = cfg.seq;
//		cfg.state = IMAGE_STATE_VERIFIED;
	} else if (bl_ver == 2) {
		/* for bl version == 0.2, only support image sequence 0 and 1 */
		if (cfg.state == IMAGE_STATE_VERIFIED) {
			seq = cfg.seq;
		} else {
			seq = (cfg.seq + 1) % 2;
		}
	} else
#endif /* IMAGE_OPT_BL_COMPATIBILITY */
	{
		/* for bl version >= 0.3, only IMAGE_STATE_VERIFIED will be saved */
		if (cfg.state == IMAGE_STATE_VERIFIED) {
			seq = cfg.seq;
		} else {
			IMAGE_ERR("invalid state %d, seq %d\n", cfg.state, cfg.seq);
		}
	}

	return seq;
}

#endif /* __CONFIG_BOOTLOADER */

/**
 * @brief Initialize the image module
 * @param[in] flash Flash device number of the 1st image region
 * @param[in] addr Start address of the 1st image region (including bootloader)
 * @param[in] size Max size of the image region (including bootloader)
 * @retval 0 on success, -1 on failure
 */
int image_init(uint32_t flash, uint32_t addr, uint32_t max_size)
{
	section_header_t sh;
	image_ota_param_t *iop;
	int i;

	IMAGE_DBG("init: flash %u, addr %#x (%u KB)\n", flash, addr, addr / 1024);

	if (flash_read(flash, addr, &sh, IMAGE_HEADER_SIZE) != IMAGE_HEADER_SIZE) {
		return -1;
	}
	if (((sh.priv[2] & 0xFFFF) == IMAGE_INVALID_SIZE) && (max_size == 0)) {
		IMAGE_ERR("the image max size is invalid\n");
		return -1;
	}

	iop = (image_ota_param_t*)image_get_ota_param();
	iop->bl_size = sh.next_addr;
	if (max_size > 0)
		iop->img_max_size = (max_size - iop->bl_size) / 1024;
	else
		iop->img_max_size = (sh.priv[2] & 0xFFFF) - (iop->bl_size / 1024);
	iop->img_xz_max_size = sh.priv[2] >> 16;
	iop->ota_flash = (BLSH_OTA_FLASH(&sh) == IMAGE_INVALID_FLASH) ?
					 flash : BLSH_OTA_FLASH(&sh);
	iop->ota_size = (BLSH_OTA_SIZE(&sh) == IMAGE_INVALID_OTA_SIZE) ?
					iop->bl_size : BLSH_OTA_SIZE(&sh);
	iop->ota_addr = BLSH_OTA_ADDR(&sh);

	/* no info in section_header_t::priv[] if version < 0.2 */
	if (sh.version < 2) { /* for compatibility only */
		iop->ota_flash = flash;
		iop->ota_size = iop->bl_size;
		iop->ota_addr = (1 << 20); /* default to 1MB */
	}
	IMAGE_DBG("bl size %u KB, image (no bl) max size %u KB, image xz max size %u KB\n",
			  iop->bl_size / 1024, iop->img_max_size, iop->img_xz_max_size);
	IMAGE_DBG("ota cfg: flash %u, size %u KB, addr %#x (%u KB)\n",
			  iop->ota_flash, iop->ota_size / 1024, iop->ota_addr,
			  iop->ota_addr / 1024);

	iop->flash[0] = flash;
	iop->addr[0] = addr + iop->bl_size;
	IMAGE_DBG("image 0: flash %u, addr %#010x (%u KB)\n",
			  iop->flash[0], iop->addr[0], iop->addr[0] / 1024);
	for (i = 1; i < IMAGE_SEQ_NUM; ++i) {
		iop->flash[i] = iop->ota_flash;
		if (iop->img_xz_max_size == IMAGE_INVALID_SIZE) {
			/* ota policy is 0x00 */
			iop->addr[i] = iop->ota_addr + iop->ota_size +
							IMAGE_AREA_SIZE(iop->img_max_size) * (i - 1);
		} else {
			iop->addr[i] = iop->ota_addr + iop->ota_size +
							IMAGE_AREA_SIZE(iop->img_xz_max_size) * (i - 1);
		}
		IMAGE_DBG("image %d: flash %u, addr %#010x (%u KB)\n",
				  i, iop->flash[i], iop->addr[i], iop->addr[i] / 1024);
	}
	image_clear_sec_addr((image_priv_t*)iop);

#ifndef __CONFIG_BOOTLOADER
	iop->running_seq = image_init_running_seq(iop); /* init running sequence */
#else
	iop->running_seq = IMAGE_SEQ_NUM; /* set to invalid running sequence for bl*/
#endif
	IMAGE_DBG("running seq %u\n", iop->running_seq);

	return 0;

}

