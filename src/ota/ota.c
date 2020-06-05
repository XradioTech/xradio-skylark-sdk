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

#include "ota_i.h"
#include "ota_debug.h"
#include "ota_file.h"
#include "ota_http.h"
#include "ota/ota.h"
#include "image/flash.h"
#include "image/image.h"
#include "driver/chip/hal_crypto.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_wdg.h"

#define OTA_IMG_DATA_CORRUPTION_TEST	0 /* make image data corruption, for test only */

#define OTA_UPDATE_DEBUG_SIZE_UNIT		(50 * 1024)

#define OTA_START_PERCENT (0)
#define OTA_DOWNLOAD_FINISH_PERCENT (80)
#define OTA_CHECK_SECTIONS_PERCENT (90)
#define OTA_VERIFY_IMAGE_PERCENT (100)

static ota_priv_t	ota_priv;
static ota_callback ota_cb = NULL;
static int32_t		ota_skip_size = -1;

/* indexed by image_seq_t */
static const image_seq_t ota_update_seq_policy[IMAGE_SEQ_NUM] = {
#if (IMAGE_SEQ_NUM == 2)
	/* update policy: 0 -> 1, 1 -> 0 */
	1, 0
#elif (IMAGE_SEQ_NUM == 3)
	/* update policy: 0 -> 1, 1 -> 2, 2 -> 1 */
	1, 2, 1
#else
	#error "unsupport IMAGE_SEQ_NUM!"
#endif
};

/**
 * @brief Get the image sequence which to be updated by OTA
 * @return Image sequence to be updated
 */
static image_seq_t ota_get_update_seq(void)
{
	image_seq_t seq = IMAGE_SEQ_NUM;
	const image_ota_param_t *iop = ota_priv.iop;

	if (iop == NULL) {
		OTA_ERR("not init");
		return seq;
	}

	if ((IMAGE_AREA_SIZE(iop->img_max_size) == 0) || (iop->bl_size == 0) ||
		(iop->running_seq >= IMAGE_SEQ_NUM)) {
		OTA_ERR("not init, img_max_size %#x, bl_size %#x, running seq %u\n",
				IMAGE_AREA_SIZE(iop->img_max_size), iop->bl_size, iop->running_seq);
		return seq;
	}

	seq = ota_update_seq_policy[iop->running_seq]; /* update sequence */
	OTA_DBG("%s(), update seq %d\n", __func__, seq);
	return seq;
}

/**
 * @brief Initialize the OTA service
 * @retval ota_status_t, OTA_STATUS_OK on success
 */
ota_status_t ota_init(void)
{
	ota_memset(&ota_priv, 0, sizeof(ota_priv));
	ota_priv.iop = image_get_ota_param();
#if (__CONFIG_OTA_POLICY == 0x01)
	if (ota_priv.iop->img_xz_max_size == IMAGE_INVALID_SIZE) {
		OTA_ERR("the compressed image max size is invalid, need set in image.cfg file\n");
		return OTA_STATUS_ERROR;
	}
#endif
	return OTA_STATUS_OK;
}

/**
 * @brief DeInitialize the OTA service
 * @return None
 */
void ota_deinit(void)
{
	ota_memset(&ota_priv, 0, sizeof(ota_priv));
}

static ota_status_t ota_update_image_process(image_seq_t seq, void *url,
											 ota_update_init_t init_cb,
											 ota_update_get_t get_cb)
{
	ota_status_t	status;
	uint32_t		flash;
	uint32_t		addr;
	uint32_t		skip_size;
	uint32_t		recv_size;
	uint32_t		img_max_size;
	uint8_t		   *ota_buf;
	uint8_t			eof_flag;
	uint32_t		debug_size;
	ota_status_t	ret = OTA_STATUS_ERROR;
	const image_ota_param_t *iop = ota_priv.iop;

	flash = iop->flash[seq];
	addr = iop->addr[seq];
#if (__CONFIG_OTA_POLICY == 0x00)
	img_max_size = IMAGE_AREA_SIZE(iop->img_max_size);
#else
	img_max_size = IMAGE_AREA_SIZE(iop->img_xz_max_size);
#endif
	OTA_DBG("%s(), seq %d, flash %u, addr %#x, size %d\n", __func__, seq,
			flash, addr, img_max_size);
	OTA_SYSLOG("OTA: erase flash...\n");

	if (ota_cb)
		ota_cb(OTA_UPGRADE_START, 0, OTA_START_PERCENT);

	if (flash_erase(flash, addr, img_max_size) != 0) {
		return ret;
	}

	OTA_DBG("%s(), erase flash success\n", __func__);

	ota_buf = ota_malloc(OTA_BUF_SIZE);
	if (ota_buf == NULL) {
		OTA_ERR("no mem\n");
		return ret;
	}

	if (init_cb(url) != OTA_STATUS_OK) {
		OTA_ERR("ota update init failed\n");
		goto ota_err;
	}

	OTA_SYSLOG("OTA: start loading image...\n");
	debug_size = OTA_UPDATE_DEBUG_SIZE_UNIT;
	ota_priv.get_size = 0;

	/* skip bootloader */
	if (ota_skip_size < 0) {
#if (__CONFIG_OTA_POLICY == 0x00)
		ota_skip_size = iop->bl_size;
#else
		ota_skip_size = 0;
#endif
	}

	skip_size = ota_skip_size;
	while (skip_size > 0) {
		status = get_cb(ota_buf,
		                (skip_size > OTA_BUF_SIZE) ? OTA_BUF_SIZE : skip_size,
		                &recv_size, &eof_flag);
		if ((status != OTA_STATUS_OK) || eof_flag) {
			OTA_ERR("status %d, eof %d\n", status, eof_flag);
			goto ota_err;
		}
		skip_size -= recv_size;
		ota_priv.get_size += recv_size;

		if (ota_priv.get_size >= debug_size) {
			OTA_SYSLOG("OTA: loading image (%u KB)...\n",
			           ota_priv.get_size / 1024);
			debug_size += OTA_UPDATE_DEBUG_SIZE_UNIT;
		}
		if (ota_cb)
			ota_cb(OTA_UPGRADE_UPDATING, 0, OTA_START_PERCENT);
	}

	OTA_DBG("%s(), skip %d success\n", __func__, ota_skip_size);

	if (HAL_Flash_Open(flash, OTA_FLASH_TIMEOUT) != HAL_OK) {
		OTA_ERR("open flash %u fail\n", flash);
		goto ota_err;
	}

	OTA_DBG("image max size %u\n", img_max_size);
#if OTA_IMG_DATA_CORRUPTION_TEST
	OTA_SYSLOG("ota img data corruption test start, pls power down the device\n");
#endif
	while (img_max_size > 0) {
		status = get_cb(ota_buf, (img_max_size > OTA_BUF_SIZE) ? OTA_BUF_SIZE : img_max_size,
		                &recv_size, &eof_flag);
		if (status != OTA_STATUS_OK) {
			OTA_ERR("status %d\n", status);
			break;
		}
		if (recv_size == 0) {
			OTA_WRN("recv_size %u, status %d, eof_flag %d\n",
			        recv_size, status, eof_flag);
		} else {
			img_max_size -= recv_size;
			ota_priv.get_size += recv_size;

			if (HAL_Flash_Write(flash, addr, ota_buf, recv_size) != HAL_OK) {
				OTA_ERR("write flash fail, flash %u, addr %#x, size %#x\n",
				        flash, addr, recv_size);
				break;
			}
			addr += recv_size;
		}
		if (eof_flag) {
			ret = OTA_STATUS_OK;
			break;
		}

		if (ota_priv.get_size >= debug_size) {
			OTA_SYSLOG("OTA: loading image (%u KB)...\n",
			           ota_priv.get_size / 1024);
			debug_size += OTA_UPDATE_DEBUG_SIZE_UNIT;
		}

		if (ota_cb)
			ota_cb(OTA_UPGRADE_UPDATING, ota_priv.get_size - ota_skip_size,
				(ota_priv.get_size - ota_skip_size) * OTA_DOWNLOAD_FINISH_PERCENT /
#if (__CONFIG_OTA_POLICY == 0x00)
				IMAGE_AREA_SIZE(iop->img_max_size)
#else
				IMAGE_AREA_SIZE(iop->img_xz_max_size)
#endif
				);
	}
#if OTA_IMG_DATA_CORRUPTION_TEST
	OTA_SYSLOG("ota img data corruption test end\n");
#endif

	HAL_Flash_Close(flash);

ota_err:
	if (ota_buf)
		ota_free(ota_buf);

	if (ret != OTA_STATUS_OK) {
		if (img_max_size == 0) {
			/* reach max size, but not end, continue trying to check sections */
			OTA_ERR("download img size %u == %u, but not end\n",
					ota_priv.get_size - ota_skip_size, IMAGE_AREA_SIZE(iop->img_max_size));
		} else {
			return ret;
		}
	}

	OTA_SYSLOG("OTA: finish loading image(%#010x)\n", ota_priv.get_size);

	if (image_check_sections(seq) == IMAGE_INVALID) {
		OTA_ERR("ota check image failed\n");
		return OTA_STATUS_ERROR;
	}

	if (ota_cb)
		ota_cb(OTA_UPGRADE_UPDATING, ota_priv.get_size - ota_skip_size, OTA_CHECK_SECTIONS_PERCENT);

	OTA_SYSLOG("OTA: finish checking image.\n");
	return OTA_STATUS_OK;
}

static ota_status_t ota_update_image(void *url,
									 ota_update_init_t init_cb,
									 ota_update_get_t get_cb)
{
	ota_status_t ret = OTA_STATUS_ERROR;
	const image_ota_param_t *iop = ota_priv.iop;
	image_seq_t	 seq;
	seq = ota_get_update_seq();

	if (seq < IMAGE_SEQ_NUM) {
		ret = ota_update_image_process(seq, url, init_cb, get_cb);
		if (ret != OTA_STATUS_OK && ota_cb != NULL) {
			ota_cb(OTA_UPGRADE_FAIL, ota_priv.get_size - ota_skip_size,
				(ota_priv.get_size - ota_skip_size) * OTA_DOWNLOAD_FINISH_PERCENT /
#if (__CONFIG_OTA_POLICY == 0x00)
				IMAGE_AREA_SIZE(iop->img_max_size)
#else
				IMAGE_AREA_SIZE(iop->img_xz_max_size)
#endif
				);
		}
		return ret;
	} else {
		return OTA_STATUS_ERROR;
	}
}

/**
 * @brief Get the image file with the specified protocol and write to flash
 * @param[in] protocol Pointer to the protocol of getting image file
 * @param[in] url URL of the image file
 * @retval ota_status_t, OTA_STATUS_OK on success
 */
ota_status_t ota_get_image(ota_protocol_t protocol, void *url)
{
	if (url == NULL) {
		OTA_ERR("url %p\n", url);
		return OTA_STATUS_ERROR;
	}

	ota_init();

	switch (protocol) {
#if OTA_OPT_PROTOCOL_FILE
	case OTA_PROTOCOL_FILE:
		return ota_update_image(url, ota_update_file_init, ota_update_file_get);
#endif
#if OTA_OPT_PROTOCOL_HTTP
	case OTA_PROTOCOL_HTTP:
		return ota_update_image(url, ota_update_http_init, ota_update_http_get);
#endif
	default:
		OTA_ERR("invalid protocol %d\n", protocol);
		return OTA_STATUS_ERROR;
	}
}

/**
 * @brief Set the callback function of ota
 * @retval ota_status_t, OTA_STATUS_OK on success
 */
ota_status_t ota_set_cb(ota_callback cb)
{
	ota_cb = cb;

	return OTA_STATUS_OK;
}

/**
 * @brief The init operation of pushing the image file
 * @retval ota_status_t, OTA_STATUS_OK on success
 */
ota_status_t ota_push_init(void)
{
	OTA_SYSLOG("OTA: push init\n");

	return ota_init();
}

/**
 * @brief The begin operation of pushing the image file
 * @retval ota_status_t, OTA_STATUS_OK on success
 */
 ota_status_t ota_push_start(void)
 {
	image_seq_t	 seq;
	uint32_t		 flash;
	uint32_t		 addr;
	uint32_t		 img_max_size;
	const image_ota_param_t *iop = NULL;

	iop = ota_priv.iop;
	seq = ota_get_update_seq();
	flash = iop->flash[seq];
	addr = iop->addr[seq];
#if (__CONFIG_OTA_POLICY == 0x00)
	img_max_size = IMAGE_AREA_SIZE(iop->img_max_size);
#else
	img_max_size = IMAGE_AREA_SIZE(iop->img_xz_max_size);
#endif

	if (ota_skip_size < 0) {
#if (__CONFIG_OTA_POLICY == 0x00)
		ota_skip_size = iop->bl_size;
#else
		ota_skip_size = 0;
#endif
	}

	if (ota_cb)
		ota_cb(OTA_UPGRADE_START, 0, OTA_START_PERCENT);

	OTA_DBG("%s(), seq %d, flash %u, addr %#x\n", __func__, seq, flash, addr);
	OTA_SYSLOG("OTA: erase flash...\n");

	if (flash_erase(flash, addr, img_max_size) != 0) {
		OTA_ERR("OTA: erase fail\n");
		return OTA_STATUS_ERROR;
	}


	return OTA_STATUS_OK;
 }

 /**
  * @brief Set the skip size
  * @param[in] the data size need skip
  * @retval ota_status_t, OTA_STATUS_OK on success
  */
 ota_status_t ota_set_skip_size(int32_t skip_size)
 {
	ota_skip_size = skip_size;
	return OTA_STATUS_OK;
 }

/**
 * @brief Push image file data
 * @param[in] data Pointer of push data buffer
 * @param[in] size Size of push data
 * @retval ota_status_t, OTA_STATUS_OK on success
 */
 ota_status_t ota_push_data(uint8_t *data, uint32_t size)
 {
	image_seq_t	 seq;
	uint32_t		 flash;
	uint32_t		 addr;
	uint32_t		 ret = 0;
	uint8_t			*write_data = NULL;
	uint32_t		 write_size = 0;
	uint32_t		 remain_skip_size = 0;
	uint32_t		 remain_img_size = 0;
	uint32_t		 img_max_size;
	const image_ota_param_t *iop = ota_priv.iop;
	ota_status_t	 status = OTA_STATUS_OK;

#if (__CONFIG_OTA_POLICY == 0x00)
	img_max_size = IMAGE_AREA_SIZE(iop->img_max_size);
#else
	img_max_size = IMAGE_AREA_SIZE(iop->img_xz_max_size);
#endif

	if (size == 0) {
		goto out;
	}

	/* skip size */
	if (ota_priv.get_size < ota_skip_size) {
		remain_skip_size = ota_skip_size - ota_priv.get_size;
		if (remain_skip_size >= size) {
			ota_priv.get_size += size;
			status = OTA_STATUS_OK;
			goto out;
		} else {
			write_data = &data[remain_skip_size];
			write_size = size - remain_skip_size;
			ota_priv.get_size += remain_skip_size;
		}
	} else {
		write_data = data;
		write_size = size;
	}

	/* check remain img size */
	remain_img_size = img_max_size + ota_skip_size - ota_priv.get_size;
	if (write_size > remain_img_size) {
		OTA_ERR("download img size overflow: %u == %u\n",
			ota_priv.get_size - ota_skip_size + write_size, img_max_size);
		status = OTA_STATUS_ERROR;
		goto out;
	}

	/* write to flash */
	seq = ota_get_update_seq();
	flash = iop->flash[seq];
	addr = iop->addr[seq] + ota_priv.get_size - ota_skip_size;

	ret = flash_write(flash, addr, write_data, write_size);
	ota_priv.get_size += ret;
	if (ret != write_size) {
		OTA_ERR("write flash fail, flash %u, addr %#x, size %#x, ret %#x\n",
				 flash, addr, write_size, ret);
		status = OTA_STATUS_ERROR;
		goto out;
	}

 out:
	if (ota_cb)
		ota_cb(status == OTA_STATUS_OK ? OTA_UPGRADE_UPDATING : OTA_UPGRADE_FAIL,
			ota_priv.get_size > ota_skip_size ? ota_priv.get_size - ota_skip_size : 0,
			remain_img_size > 0 ?
			OTA_DOWNLOAD_FINISH_PERCENT - remain_img_size * OTA_DOWNLOAD_FINISH_PERCENT /
			img_max_size : OTA_START_PERCENT);
	return status;
 }

/**
 * @brief The end operation of pushing the image file
 * @retval ota_status_t, OTA_STATUS_OK on success
 */
ota_status_t ota_push_finish(void)
{
	image_seq_t seq;
	uint32_t *verify_value;
	ota_verify_t verify_type;
	ota_verify_data_t verify_data;
	ota_status_t status = OTA_STATUS_OK;
	const image_ota_param_t *iop = ota_priv.iop;
	uint32_t img_max_size;

#if (__CONFIG_OTA_POLICY == 0x00)
	img_max_size = IMAGE_AREA_SIZE(iop->img_max_size);
#else
	img_max_size = IMAGE_AREA_SIZE(iop->img_xz_max_size);
#endif

	OTA_SYSLOG("OTA: pushed image size (%#010x = %u KB)\n",
		ota_priv.get_size, ota_priv.get_size / 1024);

	OTA_SYSLOG("OTA: checking image...\n");
	seq = ota_get_update_seq();
	if (image_check_sections(seq) == IMAGE_INVALID) {
		OTA_ERR("check image failed\n");
		status = OTA_STATUS_ERROR;
		goto out;
	}

	if (ota_cb)
		ota_cb(OTA_UPGRADE_UPDATING, ota_priv.get_size - ota_skip_size, OTA_CHECK_SECTIONS_PERCENT);

	if (ota_get_verify_data(&verify_data) != OTA_STATUS_OK) {
		verify_type = OTA_VERIFY_NONE;
		verify_value = NULL;
	} else {
		verify_type = verify_data.ov_type;
		verify_value = (uint32_t*)(verify_data.ov_data);
	}

	if (ota_verify_image(verify_type, verify_value) != OTA_STATUS_OK) {
		OTA_ERR("verify image failed\n");
		status = OTA_STATUS_ERROR;
		goto out;
	}

	OTA_SYSLOG("OTA: check ok\n");

	OTA_SYSLOG("OTA: push finish\n");

out:
	if (ota_cb)
		ota_cb(status == OTA_STATUS_OK ? OTA_UPGRADE_SUCCESS : OTA_UPGRADE_FAIL,
					ota_priv.get_size - ota_skip_size,
					status == OTA_STATUS_OK ? OTA_VERIFY_IMAGE_PERCENT :
					(ota_priv.get_size - ota_skip_size) * OTA_DOWNLOAD_FINISH_PERCENT / img_max_size);
	return status;
}

/**
 * @brief The stop operation of pushing the image file
 * @retval ota_status_t, OTA_STATUS_OK on success
 */
ota_status_t ota_push_stop(void)
{
	OTA_SYSLOG("OTA: push stop\n");

	if (ota_cb)
		ota_cb(OTA_UPGRADE_STOP, ota_priv.get_size - ota_skip_size, OTA_VERIFY_IMAGE_PERCENT);

	ota_deinit();

	return OTA_STATUS_OK;
}

#if (OTA_OPT_EXTRA_VERIFY_CRC32 || OTA_OPT_EXTRA_VERIFY_MD5 || \
     OTA_OPT_EXTRA_VERIFY_SHA1 || OTA_OPT_EXTRA_VERIFY_SHA256)
/**
 * @brief Get the begin address of verify data
 * @param[in] sequence of image to get verify data
 * @return the begin address of verify data, 0 on bad image
 */
uint32_t ota_get_verify_data_pos(image_seq_t seq)
{
	section_header_t sh;
	uint32_t flash;
	uint32_t addr;
	uint32_t next_addr;
	uint32_t pos = 0;
	const image_ota_param_t *iop = ota_priv.iop;

	if (seq >= IMAGE_SEQ_NUM) {
		OTA_ERR("seq %u\n", seq);
		return pos;
	}

	/* iterate from bootloader */
	flash = IMG_BL_FLASH(iop);
	addr = IMG_BL_ADDR(iop);

	if (HAL_Flash_Open(flash, OTA_FLASH_TIMEOUT) != HAL_OK) {
		OTA_ERR("open flash %u fail\n", flash);
		return pos;
	}
	while (1) {
		if (HAL_Flash_Read(flash, addr, (uint8_t*)&sh, IMAGE_HEADER_SIZE) != HAL_OK) {
			OTA_ERR("read flash %u fail, addr %#x, size %#x\n",
					flash, addr, IMAGE_HEADER_SIZE);
			break;
		}
		if (image_check_header(&sh) == IMAGE_INVALID) {
			OTA_ERR("bad header, flash %u, seq %u, addr %#x, id %#x\n",
					  flash, seq, addr, sh.id);
			break;
		}
		next_addr = sh.next_addr;
		if (next_addr == IMAGE_INVALID_ADDR) {
			pos = addr + IMAGE_HEADER_SIZE + sh.data_size;
			OTA_DBG("%s(), seq %d, verify data position %#x\n", __func__, seq, pos);
			break;
		}
		flash = iop->flash[seq];
		addr = iop->addr[seq] + next_addr - iop->bl_size;
	};
	HAL_Flash_Close(flash);
	return pos;
}
#endif /* (OTA_OPT_EXTRA_VERIFY_CRC32 || OTA_OPT_EXTRA_VERIFY_MD5 || ...) */


/**
 * @brief Get the verify data of image file
 * @param[out] structure for verify data
 * @retval ota_status_t, OTA_STATUS_OK on success
 */
ota_status_t ota_get_verify_data(ota_verify_data_t *data)
{
#if (OTA_OPT_EXTRA_VERIFY_CRC32 || OTA_OPT_EXTRA_VERIFY_MD5 || \
     OTA_OPT_EXTRA_VERIFY_SHA1 || OTA_OPT_EXTRA_VERIFY_SHA256)
	ota_status_t	status;
	image_seq_t		seq;
	uint32_t 		addr, flash;
	uint32_t 		read_size;
	const image_ota_param_t *iop = ota_priv.iop;

	status = OTA_STATUS_ERROR;
#ifndef __CONFIG_BOOTLOADER
	/* in normal ota, need verify the update seq, because the image is download
     * in update seq.
     */
	seq = ota_get_update_seq();
#else
	/* in bootloader, need verify the running seq, because the compressd image
     * is download in update seq, and it will be decompressd in running seq.
     */
	seq = iop->running_seq;
#endif
	if (seq >= IMAGE_SEQ_NUM) {
		return status;
	}

	if (ota_priv.get_size == 0) {
		OTA_ERR("need to get image, get_size %#x\n", ota_priv.get_size);
		return status;
	}

	flash = iop->flash[seq];
	addr = ota_get_verify_data_pos(seq);
	if (addr == 0) {
		OTA_ERR("get image %u verify data position fail\n", seq);
		return status;
	}
	if (HAL_Flash_Open(flash, OTA_FLASH_TIMEOUT) != HAL_OK) {
		OTA_ERR("open flash %u fail\n", flash);
		return status;
	}
	read_size = sizeof(ota_verify_data_t);
	if (HAL_Flash_Read(flash, addr, (uint8_t*)data, read_size) != HAL_OK) {
		OTA_ERR("read flash %u fail, addr %#x, size %#x\n",
		        flash, addr, read_size);
		goto out;
	}
	if (data->ov_magic != OTA_VERIFY_MAGIC) {
		OTA_WRN("invalid verify data magic %#x\n", data->ov_magic);
		goto out;
	}

	status = OTA_STATUS_OK;

out:
	HAL_Flash_Close(flash);
	return status;
#else
	return OTA_STATUS_ERROR;
#endif /* (OTA_OPT_EXTRA_VERIFY_CRC32 || OTA_OPT_EXTRA_VERIFY_MD5 || ...) */
}

#if (OTA_OPT_EXTRA_VERIFY_CRC32 || OTA_OPT_EXTRA_VERIFY_MD5 || \
     OTA_OPT_EXTRA_VERIFY_SHA1 || OTA_OPT_EXTRA_VERIFY_SHA256)
static ota_status_t ota_verify_image_append_process(uint32_t flash,
													uint32_t addr,
													uint32_t size,
													ota_verify_append_t append,
													void *hdl)
{
	uint32_t read_size;
	uint8_t *buf = NULL;
	ota_status_t status = OTA_STATUS_ERROR;

	if (HAL_Flash_Open(flash, OTA_FLASH_TIMEOUT) != HAL_OK) {
		OTA_ERR("open flash %u fail\n", flash);
		return status;
	}

	buf = ota_malloc(OTA_BUF_SIZE);
	if (buf == NULL) {
		OTA_ERR("no mem\n");
		goto out;
	}

	while (size > 0) {
		read_size = size > OTA_BUF_SIZE ? OTA_BUF_SIZE : size;
		if (HAL_Flash_Read(flash, addr, buf, read_size) != HAL_OK) {
			OTA_ERR("read flash %u fail, addr %#x, size %#x\n",
			        flash, addr, read_size);
			break;
		}
		if (append(hdl, buf, read_size) != HAL_OK) {
			break;
		}
		size -= read_size;
		addr += read_size;
	}

	if (size == 0) {
		status = OTA_STATUS_OK;
	}

out:
	if (buf) {
		ota_free(buf);
	}
	HAL_Flash_Close(flash);
	return status;
}

static ota_status_t ota_verify_image_append(image_seq_t seq,
											ota_verify_append_t append,
											void *hdl)
{
	uint32_t size;
	ota_priv_t *ota = &ota_priv;
	const image_ota_param_t *iop = ota->iop;

	OTA_DBG("%s(), seq %d\n", __func__, seq);

#if defined(__CONFIG_BOOTLOADER)
    size = ota_get_verify_data_pos(seq) - iop->addr[seq];
#else
	size = ota->get_size - ota_skip_size - sizeof(ota_verify_data_t);
#endif

	//If there is no verify data in new image, we will use OTA_VERIFY_NONE to verify it.
	//In this case, the size will not be use.
	if (ota_verify_image_append_process(iop->flash[seq],
	                                    iop->addr[seq],
	                                    size,
	                                    append,
	                                    hdl) != OTA_STATUS_OK) {
		OTA_ERR("append image fail, seq %d\n", seq);
		return OTA_STATUS_ERROR;
	}

	return OTA_STATUS_OK;
}
#endif /* (OTA_OPT_EXTRA_VERIFY_CRC32 || OTA_OPT_EXTRA_VERIFY_MD5 || ...) */

static ota_status_t ota_verify_image_none(image_seq_t seq, uint32_t *value)
{
	return OTA_STATUS_OK;
}

#if OTA_OPT_EXTRA_VERIFY_CRC32
static ota_status_t ota_verify_image_crc32(image_seq_t seq, uint32_t *value)
{
	CE_CRC_Handler	hdl;
	uint32_t		crc;

	if (HAL_CRC_Init(&hdl, CE_CRC32, ota_priv.get_size) != HAL_OK) {
		OTA_ERR("CRC init failed\n");
		return OTA_STATUS_ERROR;
	}

	if (ota_verify_image_append(seq,
								(ota_verify_append_t)HAL_CRC_Append,
								(void *)&hdl) != OTA_STATUS_OK) {
		OTA_ERR("CRC append failed\n");
		return OTA_STATUS_ERROR;
	}

	if (HAL_CRC_Finish(&hdl, &crc) != HAL_OK) {
		OTA_ERR("CRC finish failed\n");
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), value %#x, crc %#x\n", __func__, *value, crc);

	if (ota_memcmp(value, &crc, sizeof(crc)) == 0) {
		return OTA_STATUS_OK;
	} else {
		return OTA_STATUS_ERROR;
	}
}
#endif /* OTA_OPT_EXTRA_VERIFY_CRC32 */

#if OTA_OPT_EXTRA_VERIFY_MD5
static ota_status_t ota_verify_image_md5(image_seq_t seq, uint32_t *value)
{
	CE_MD5_Handler	hdl;
	uint32_t		digest[4];

	if (HAL_MD5_Init(&hdl, CE_CTL_IVMODE_SHA_MD5_FIPS180, NULL) != HAL_OK) {
		OTA_ERR("MD5 init failed\n");
		return OTA_STATUS_ERROR;
	}

	if (ota_verify_image_append(seq,
								(ota_verify_append_t)HAL_MD5_Append,
								(void *)&hdl) != OTA_STATUS_OK) {
		OTA_ERR("MD5 append failed\n");
		return OTA_STATUS_ERROR;
	}

	if (HAL_MD5_Finish(&hdl, digest) != HAL_OK) {
		OTA_ERR("MD5 finish failed\n");
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), value[0] %#x, digest[0] %#x\n", __func__, value[0], digest[0]);

	if (ota_memcmp(value, digest, sizeof(digest)) == 0) {
		return OTA_STATUS_OK;
	} else {
		return OTA_STATUS_ERROR;
	}
}
#endif /* OTA_OPT_EXTRA_VERIFY_MD5 */

#if OTA_OPT_EXTRA_VERIFY_SHA1
static ota_status_t ota_verify_image_sha1(image_seq_t seq, uint32_t *value)
{
	CE_SHA1_Handler	hdl;
	uint32_t		digest[5];

	if (HAL_SHA1_Init(&hdl, CE_CTL_IVMODE_SHA_MD5_FIPS180, NULL) != HAL_OK) {
		OTA_ERR("SHA1 init failed\n");
		return OTA_STATUS_ERROR;
	}

	if (ota_verify_image_append(seq,
								(ota_verify_append_t)HAL_SHA1_Append,
								(void *)&hdl) != OTA_STATUS_OK) {
		OTA_ERR("SHA1 append failed\n");
		return OTA_STATUS_ERROR;
	}

	if (HAL_SHA1_Finish(&hdl, digest) != HAL_OK) {
		OTA_ERR("SHA1 finish failed\n");
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), value[0] %#x, digest[0] %#x\n", __func__, value[0], digest[0]);

	if (ota_memcmp(value, digest, sizeof(digest)) == 0) {
		return OTA_STATUS_OK;
	} else {
		return OTA_STATUS_ERROR;
	}
}
#endif /* OTA_OPT_EXTRA_VERIFY_SHA1 */

#if OTA_OPT_EXTRA_VERIFY_SHA256
static ota_status_t ota_verify_image_sha256(image_seq_t seq, uint32_t *value)
{
	CE_SHA256_Handler	hdl;
	uint32_t			digest[8];

	if (HAL_SHA256_Init(&hdl, CE_CTL_IVMODE_SHA_MD5_FIPS180, NULL) != HAL_OK) {
		OTA_ERR("SHA256 init failed\n");
		return OTA_STATUS_ERROR;
	}

	if (ota_verify_image_append(seq,
								(ota_verify_append_t)HAL_SHA256_Append,
								(void *)&hdl) != OTA_STATUS_OK) {
		OTA_ERR("SHA256 append failed\n");
		return OTA_STATUS_ERROR;
	}

	if (HAL_SHA256_Finish(&hdl, digest) != HAL_OK) {
		OTA_ERR("SHA256 finish failed\n");
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), value[0] %#x, digest[0] %#x\n", __func__, value[0], digest[0]);

	if (ota_memcmp(value, digest, sizeof(digest)) == 0) {
		return OTA_STATUS_OK;
	} else {
		return OTA_STATUS_ERROR;
	}
}
#endif /* OTA_OPT_EXTRA_VERIFY_SHA256 */

/**
 * @brief Verify the image file and modify OTA configuration correspondingly
 * @param[in] verify Verification algorithm
 * @param[in] value Pointer to standard value of the verification algorithm
 * @retval ota_status_t, OTA_STATUS_OK on success
 */
ota_status_t ota_verify_image(ota_verify_t verify, uint32_t *value)
{
	int				ret;
	ota_status_t	status;
	image_cfg_t		cfg;
	image_seq_t		seq;

	if ((verify != OTA_VERIFY_NONE) && (value == NULL)) {
		OTA_ERR("invalid args, verify %d, res %p\n", verify, value);
		return OTA_STATUS_ERROR;
	}

#ifndef __CONFIG_BOOTLOADER
	/* in normal ota, need verify the update seq, because the image is download
	 * in update seq.
	 */
	seq = ota_get_update_seq();
#else
	/* in bootloader, need verify the running seq, because the compressd image
	 * is download in update seq, and it will be decompressd in running seq.
	 */
	seq = ota_priv.iop->running_seq;
#endif
	if (seq >= IMAGE_SEQ_NUM) {
		return OTA_STATUS_ERROR;
	}

	if (ota_priv.get_size == 0) {
		OTA_ERR("need to get image, get_size %#x\n", ota_priv.get_size);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), verify %d, size %#x\n", __func__, verify, ota_priv.get_size);

	switch (verify) {
	case OTA_VERIFY_NONE:
		status = ota_verify_image_none(seq, value);
		break;
#if OTA_OPT_EXTRA_VERIFY_CRC32
	case OTA_VERIFY_CRC32:
		status = ota_verify_image_crc32(seq, value);
		break;
#endif
#if OTA_OPT_EXTRA_VERIFY_MD5
	case OTA_VERIFY_MD5:
		status = ota_verify_image_md5(seq, value);
		break;
#endif
#if OTA_OPT_EXTRA_VERIFY_SHA1
	case OTA_VERIFY_SHA1:
		status = ota_verify_image_sha1(seq, value);
		break;
#endif
#if OTA_OPT_EXTRA_VERIFY_SHA256
	case OTA_VERIFY_SHA256:
		status = ota_verify_image_sha256(seq, value);
		break;
#endif
	default:
		OTA_ERR("invalid verify %d\n", verify);
		return OTA_STATUS_ERROR;
	}

	if (status != OTA_STATUS_OK) {
		OTA_ERR("verify fail, status %d, verify %d\n", status, verify);
		goto verify_err;
	}
#if (__CONFIG_OTA_POLICY == 0x01)
	cfg.seq = ota_get_update_seq();
	cfg.state = IMAGE_STATE_VERIFIED;
#elif defined(__CONFIG_BOOTLOADER)
	cfg.seq = ota_get_update_seq();
	cfg.state = IMAGE_STATE_UNVERIFIED;
#else
	cfg.seq = seq;
	cfg.state = IMAGE_STATE_VERIFIED;
#endif
	ret = image_set_cfg(&cfg);
	if (ret == 0) {
		if (ota_cb)
#if (__CONFIG_OTA_POLICY == 0x00)
			ota_cb(OTA_UPGRADE_SUCCESS, ota_priv.get_size - ota_skip_size, OTA_VERIFY_IMAGE_PERCENT);
#else
			ota_cb(OTA_UPGRADE_SUCCESS, ota_priv.get_size, OTA_VERIFY_IMAGE_PERCENT);
#endif
		return OTA_STATUS_OK;
	}

verify_err:
	if (ota_cb)
#if (__CONFIG_OTA_POLICY == 0x00)
		ota_cb(OTA_UPGRADE_FAIL, ota_priv.get_size - ota_skip_size,
			(ota_priv.get_size - ota_skip_size) * OTA_DOWNLOAD_FINISH_PERCENT /
			IMAGE_AREA_SIZE(ota_priv.iop->img_max_size));
#else
		ota_cb(OTA_UPGRADE_FAIL, ota_priv.get_size,
			ota_priv.get_size * OTA_DOWNLOAD_FINISH_PERCENT / IMAGE_AREA_SIZE(ota_priv.iop->img_xz_max_size));
#endif
	return OTA_STATUS_ERROR;
}

/**
 * @brief Reboot system
 * @return None
 */
void ota_reboot(void)
{
	OTA_DBG("OTA reboot.\n");
	HAL_WDG_Reboot();
}
