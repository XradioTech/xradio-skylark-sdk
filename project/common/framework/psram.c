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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "sys/param.h"
#include "sys/io.h"
#include "sys/xr_debug.h"

#include "image/fdcm.h"
#include "image/image.h"
#include "driver/chip/hal_util.h"
#include "driver/chip/hal_icache.h"
#include "driver/chip/psram/psram.h"
#include "driver/chip/hal_dcache.h"
#include "psram.h"
#include "common/board/board.h"

#ifdef __CONFIG_BIN_COMPRESS_APP_PSRAM
#include "xz/xz.h"
#endif

#ifdef __CONFIG_PSRAM

#define PSRAM_DBG_CHECK   0
#define PSRAM_DBG_ON      0
#define PSRAM_WRN_ON      0
#define PSRAM_ERR_ON      1
#define PSRAM_ABORT_ON    1

#define PSRAM_SYSLOG      printf
#define PSRAM_ABORT()     do { } while (0)

#define PSRAM_LOG(flags, fmt, arg...) \
    do {                                \
	if (flags) {				    \
		__sram_rodata static char __fmt[] = fmt;    \
		PSRAM_SYSLOG(__fmt, ##arg);		    \
	    }						    \
	} while (0)

#define PSRAM_DBG(fmt, arg...) \
    PSRAM_LOG(PSRAM_DBG_ON, "[psram] "fmt, ##arg)

#define PSRAM_INF(fmt, arg...) \
    PSRAM_LOG(PSRAM_WRN_ON, "[psram] "fmt, ##arg)

#define PSRAM_WRN(fmt, arg...) \
    PSRAM_LOG(PSRAM_WRN_ON, "[psram WRN] "fmt, ##arg)

#define PSRAM_ERR(fmt, arg...)                                    \
    do {                                                            \
        PSRAM_LOG(PSRAM_ERR_ON, "[psram ERR] "fmt, ##arg);          \
        if (PSRAM_ABORT_ON)                                       \
            PSRAM_ABORT();                                        \
    } while (0)

#if PSRAM_DBG_ON
#define PSRAM_DUMP(a, l) print_hex_dump_bytes(a, l)
#else
#define PSRAM_DUMP(a, l)
#endif

#define PSRAM_FREQ (96000000)
#define LOAD_SIZE (1 * 1024)

//#define LOAD_BY_DBUS_CPU
//#define LOAD_BY_DBUS_DMA
#define LOAD_BY_DBUS_FLASH_DMA
//#define LOAD_BY_SBUS_CPU
//#define LOAD_BY_SBUS_DMA

#if (defined LOAD_BY_DBUS_DMA)
#include "driver/chip/hal_dma.h"
__sram_data
static uint32_t dma_wburst0 = 0, dma_wburst1 = 0;
__sram_data
static uint32_t dma_wwidth0 = 0, dma_wwidth1 = 0;
__sram_data
static uint32_t dma_rburst0 = 0, dma_rburst1 = 0;
__sram_data
static uint32_t dma_rwidth0 = 0, dma_rwidth1 = 0;
__sram_data
static OS_Semaphore_t dmaSem;
__sram_data
uint32_t dma_burst_type[2] = {DMA_BURST_LEN_1, DMA_BURST_LEN_4};
__sram_data
uint32_t dma_width_type[3] = {DMA_DATA_WIDTH_8BIT, DMA_DATA_WIDTH_16BIT, DMA_DATA_WIDTH_32BIT};

__sram_text
static void psram_DMARelease(void *arg)
{
	OS_SemaphoreRelease(&dmaSem);
}

__sram_text
static int psram_dma_read_write(uint32_t write, uint32_t addr,
                               uint8_t *buf, uint32_t len)
{
	OS_Status ret;
	DMA_ChannelInitParam dmaParam;
	DMA_Channel dma_ch;

	dma_ch = HAL_DMA_Request();
	if (dma_ch == DMA_CHANNEL_INVALID) {
		PSRAM_ERR("%s,%d\n", __func__, __LINE__);
		return -1;
	}

	OS_SemaphoreCreate(&dmaSem, 0, 1);

	dmaParam.irqType = DMA_IRQ_TYPE_END;
	dmaParam.endCallback = (DMA_IRQCallback)psram_DMARelease;
	dmaParam.endArg = NULL;
	if (write) {
		if (++dma_wburst0 >= 2) {
			dma_wburst0 = 0;
			if (++dma_wburst1 >= 2) {
				dma_wburst1 = 0;
				if (++dma_wwidth0 >= 3) {
					dma_wwidth0 = 0;
					if (++dma_wwidth1 >= 3)
						dma_wwidth1 = 0;
				}
			}
		}
	} else {
		if (++dma_rburst0 >= 2) {
			dma_rburst0 = 0;
			if (++dma_rburst1 >= 2) {
				dma_rburst1 = 0;
				if (++dma_rwidth0 >= 3) {
					dma_rwidth0 = 0;
					if (++dma_rwidth1 >= 3)
						dma_rwidth1 = 0;
				}
			}
		}
	}
	if (write) {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
				DMA_WAIT_CYCLE_2,
				DMA_BYTE_CNT_MODE_REMAIN,
				dma_width_type[dma_wwidth0],
				dma_burst_type[dma_wburst0],
				DMA_ADDR_MODE_INC,
				(DMA_Periph)(DMA_PERIPH_PSRAMC),
				dma_width_type[dma_wwidth1],
				dma_burst_type[dma_wburst1],
				DMA_ADDR_MODE_INC,
				DMA_PERIPH_SRAM);
	} else {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
				DMA_WAIT_CYCLE_2,
				DMA_BYTE_CNT_MODE_REMAIN,
				dma_width_type[dma_rwidth0],
				dma_burst_type[dma_rburst0],
				DMA_ADDR_MODE_INC,
				DMA_PERIPH_SRAM,
				dma_width_type[dma_rwidth1],
				dma_burst_type[dma_rburst1],
				DMA_ADDR_MODE_INC,
				(DMA_Periph)(DMA_PERIPH_PSRAMC));
	}
	HAL_DMA_Init(dma_ch, &dmaParam);
	//PSRAM_DBG("%s,%d rw:%d\n", __func__, __LINE__, write);
	//PSRAM_DBG("wb0:%d wb1:%d ww0:%d ww1:%d\n", dma_wburst0, dma_wburst1, dma_wwidth0, dma_wwidth1);
	//PSRAM_DBG("rb0:%d rb1:%d rw0:%d rw1:%d\n", dma_rburst0, dma_rburst1, dma_rwidth0, dma_rwidth1);

	if (write)
		HAL_DMA_Start(dma_ch, (uint32_t)buf, (uint32_t)addr, len);
	else
		HAL_DMA_Start(dma_ch, (uint32_t)addr, (uint32_t)buf, len);

	ret = OS_SemaphoreWait(&dmaSem, 5000);
	if (ret != OS_OK)
		PSRAM_ERR("sem wait failed: %d", ret);

	HAL_DMA_Stop(dma_ch);
	HAL_DMA_DeInit(dma_ch);
	HAL_DMA_Release(dma_ch);

	OS_SemaphoreDelete(&dmaSem);

	return 0;
}
#endif

#ifdef __CONFIG_BIN_COMPRESS_APP_PSRAM

#define PSRAM_DEC_B_INBUF_SIZE	(4 * 1024)
#define PSRMM_DEC_BIN_DICT_MAX	(32 * 1024)

__sram_text
static int psram_decompress_bin(const section_header_t *sh, uint32_t max_size)
{
	uint8_t *in_buf;
	uint32_t read_size, len, id, offset, left;
	uint16_t chksum;
	struct xz_dec *s;
	struct xz_buf b;
	enum xz_ret xzret;
	int ret = -1;
	OS_Time_t tm;

	PSRAM_DBG("%s() start\n", __func__);
	tm = OS_GetTicks();

	in_buf = malloc(PSRAM_DEC_B_INBUF_SIZE);
	if (in_buf == NULL) {
		PSRAM_ERR("no mem\n");
		return ret;
	}

	s = xz_dec_init(XZ_DYNALLOC, PSRMM_DEC_BIN_DICT_MAX);
	if (s == NULL) {
		PSRAM_ERR("no mem\n");
		goto out;
	}

	b.in = in_buf;
	b.in_pos = 0;
	b.in_size = 0;
	b.out = (uint8_t *)PSRAM_START_ADDR;
	b.out_pos = 0;
	b.out_size = max_size;

	id = sh->id;
	offset = 0;
	left = sh->body_len;
	chksum = sh->data_chksum;

	while (1) {
		if (b.in_pos == b.in_size) {
			if (left == 0) {
				PSRAM_ERR("no more input data\n");
				break;
			}
			read_size = left > PSRAM_DEC_B_INBUF_SIZE ?
			            PSRAM_DEC_B_INBUF_SIZE : left;
			len = image_read(id, IMAGE_SEG_BODY, offset, in_buf, read_size);
			if (len != read_size) {
				PSRAM_ERR("read img body fail, id %#x, off %u, len %u != %u\n",
				         id, offset, len, read_size);
				break;
			}
			chksum += image_get_checksum(in_buf, len);
			offset += len;
			left -= len;
			b.in_size = len;
			b.in_pos = 0;
		}

		xzret = xz_dec_run(s, &b);

		if (b.out_pos == b.out_size) {
			PSRAM_ERR("decompress size >= %u\n", b.out_size);
			break;
		}

		if (xzret == XZ_OK) {
			continue;
		} else if (xzret == XZ_STREAM_END) {
			tm = OS_GetTicks() - tm;
			PSRAM_DBG("%s() end, size %u --> %u, cost %u ms\n", __func__,
			         sh->body_len, b.out_pos, tm);
			if (chksum != 0xFFFF) {
				PSRAM_ERR("invalid checksum %#x\n", chksum);
			} else {
				ret = 0;
			}
			break;
		} else {
			PSRAM_ERR("xz_dec_run() fail %d\n", xzret);
			break;
		}
	}

out:
	xz_dec_end(s);
	free(in_buf);
	return ret;
}

#endif

__sram_text
static int load_psram_bin_and_set_addr(struct psram_chip *chip)
{

	int ret = 0;
	uint32_t put = 0, len = 0;
	section_header_t sh;
	uint8_t *buf = (uint8_t *)PSRAM_START_ADDR;
	uint32_t i, num_blocks, size;

	if (!buf) {
		PSRAM_ERR("%s malloc faild!\n", __func__);
		return -1;
	}

	if (image_read(IMAGE_APP_PSRAM_ID, IMAGE_SEG_HEADER, 0, &sh,
	               IMAGE_HEADER_SIZE) != IMAGE_HEADER_SIZE) {
		PSRAM_ERR("load section (id: %#08x) header failed\n", IMAGE_APP_PSRAM_ID);
		ret = -1;
		goto out;
	}

	if (image_check_header(&sh) == IMAGE_INVALID) {
		PSRAM_ERR("check section (id: %#08x) header failed\n", IMAGE_APP_PSRAM_ID);
		ret = -1;
		goto out;
	}

	if (sh.body_len == 0) { /* psram only used to store data */
		PSRAM_INF("psram only used store data\n");
		goto clr_bss;
	}

#ifdef __CONFIG_BIN_COMPRESS_APP_PSRAM
	if (sh.attribute & IMAGE_ATTR_FLAG_COMPRESS) {
		if (psram_decompress_bin(&sh, PSRAM_LENGTH) != 0) {
			PSRAM_ERR("psram decompress bin failed\n");
			ret = -1;
			goto out;
		}
	} else
#endif
	{
		num_blocks = (sh.body_len + LOAD_SIZE - 1) / LOAD_SIZE;
		PSRAM_INF("sh.body_le=%d load_times:%d\n", sh.body_len, num_blocks);
		for (i = 0; i < num_blocks; i++) {
			//PSRAM_DBG("psram loading idx:%d addr[0x%x] len:%d\n", i, PSRAM_START_ADDR + len, len);
			size = ((sh.body_len - len) > LOAD_SIZE) ? LOAD_SIZE : (sh.body_len - len);
			len += image_read(IMAGE_APP_PSRAM_ID, IMAGE_SEG_BODY, len, (void *)buf, size);
			buf += size;
			put += size;
		}

		if (len != sh.body_len) {
			PSRAM_ERR("psram body size %u, read %u\n", sh.body_len, len);
			ret = -1;
			goto out;
		}

		if (image_check_data(&sh, (void *)PSRAM_START_ADDR, sh.body_len,
		                     NULL, 0) == IMAGE_INVALID) {
			PSRAM_ERR("invalid psram bin body\n");
			//PSRAM_DUMP((const void *)PSRAM_START_ADDR, sh.body_len);
			ret = -1;
			goto out;
		}
	}

clr_bss:
	memset(__psram_bss_start__, 0, __psram_bss_end__ - __psram_bss_start__);
	PSRAM_INF("__psram_start__\t%p\n", __psram_start__);
	PSRAM_INF("__psram_data_start__\t%p\n", __psram_data_start__);
	PSRAM_INF("__psram_data_end__\t%p\n", __psram_data_end__);
	PSRAM_INF("__psram_bss_start__\t%p\n", __psram_bss_start__);
	PSRAM_INF("__psram_bss_end__\t%p\n", __psram_bss_end__);
	PSRAM_INF("__psram_end__\t\t%p\n", __psram_end__);

out:
	return ret;
}

__sram_data
static struct psram_chip chip = {0};

__sram_text
void platform_psram_init(void)
{
	uint32_t addr, p_type;
	struct psram_ctrl *ctrl = NULL;
	PSRAMCtrl_InitParam cfg;
	PSRAMChip_InitParam psram_chip_cfg;

	addr = image_get_section_addr(IMAGE_APP_PSRAM_ID);
	if (addr == IMAGE_INVALID_ADDR) {
		PSRAM_ERR("no psram section\n");
		return;
	}
	PSRAM_INF("get psram section ok @:0x%x\n", addr);

#if (defined __CONFIG_PSRAM_CHIP_SQPI)
	p_type = PSRAM_CHIP_SQPI;
#elif (defined __CONFIG_PSRAM_CHIP_OPI32)
	p_type = PSRAM_CHIP_OPI_APS32;
#elif (defined __CONFIG_PSRAM_CHIP_OPI64)
	p_type = PSRAM_CHIP_OPI_APS64;
#endif

	cfg.freq = PSRAM_FREQ;
	cfg.p_type = p_type;
	if (cfg.p_type == PSRAM_CHIP_SQPI)
		cfg.rdata_w = 0; /* wait 1 half cycle on fpga */
	else
		cfg.rdata_w = 1; /* wait 1 half cycle on fpga */
	ctrl = HAL_PsramCtrl_Create(0, &cfg);
	if (!ctrl) {
		PSRAM_ERR("psram create faile\n");
		return;
	}

	if (HAL_PsramCtrl_Init(ctrl, &cfg)) {
		PSRAM_ERR("psram ctrl init faild!\n");
		goto out3;
	}

	ctrl = HAL_PsramCtrl_Open(0);
	if (!ctrl) {
		PSRAM_ERR("psram open faile\n");
        goto out2;
	}

	psram_chip_cfg.freq = PSRAM_FREQ;
	psram_chip_cfg.p_type = p_type;
	if (psram_init(&chip, ctrl, &psram_chip_cfg)) {
		PSRAM_ERR("psram chip init faild!\n");
		goto out1;
	}
	PSRAM_DBG("psram chip %s init ok!, freq %d\n", chip.name, chip.freq);

	if (load_psram_bin_and_set_addr(&chip)) {
		PSRAM_ERR("load psram bin faild!\n");
		goto out;
	}
	PSRAM_DBG("load psram bin ok\n");
    return;
out:
    psram_deinit(&chip);
out1:
	HAL_PsramCtrl_Close(ctrl);
out2:
    HAL_PsramCtrl_Deinit(ctrl);
out3:
    HAL_PsramCtrl_Destory(ctrl);
    return;
}

#endif /* __CONFIG_PSRAM */
