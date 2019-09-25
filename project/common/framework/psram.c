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
#include "driver/chip/hal_dcache.h"
#include "psram.h"
#include "common/board/board.h"

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

#define PSRAM_FREQ (192000000)
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

__sram_text
static int load_psram_bin_and_set_addr(struct psram_chip *chip)
{
	extern uint8_t __psram_start__[];
	extern uint8_t __psram_end__[];
	extern uint8_t __psram_data_start__[];
	extern uint8_t __psram_data_end__[];
	extern uint8_t __psram_bss_start__[];
	extern uint8_t __psram_bss_end__[];

	int ret = 0;
	uint32_t put = 0, len = 0;
	section_header_t sh;
#ifdef LOAD_BY_DBUS_FLASH_DMA
	uint8_t *buf = (uint8_t *)IDCACHE_START_ADDR;
#else
	uint8_t *buf = malloc(LOAD_SIZE + PSRAM_DBG_CHECK * LOAD_SIZE);
#endif
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

	HAL_PsramCtrl_Set_Address_Field(NULL, 0, IDCACHE_START_ADDR, IDCACHE_END_ADDR, 0);

	if (sh.body_len == 0) { /* psram only used to store data */
		PSRAM_INF("psram only used store data\n");
		goto clr_bss;
	}

#if ((defined LOAD_BY_DBUS_FLASH_DMA) || (defined LOAD_BY_DBUS_DMA) || (defined LOAD_BY_SBUS_DMA))
	HAL_Dcache_SetWriteThrough(0, 1, IDCACHE_START_ADDR, IDCACHE_END_ADDR);
#endif

	num_blocks = (sh.body_len + LOAD_SIZE - 1) / LOAD_SIZE;
	PSRAM_INF("sh.body_le=%d load_times:%d\n", sh.body_len, num_blocks);
	for (i = 0; i < num_blocks; i++) {
		//PSRAM_DBG("psram loading idx:%d addr[0x%x] len:%d\n", i, IDCACHE_START_ADDR + len, len);
		size = ((sh.body_len - len) > LOAD_SIZE) ? LOAD_SIZE : (sh.body_len - len);
		len += image_read(IMAGE_APP_PSRAM_ID, IMAGE_SEG_BODY, len, (void *)buf, size);
#ifdef LOAD_BY_DBUS_CPU
		memcpy((void *)(IDCACHE_START_ADDR + put), buf, size); /* if support DMA, copy by image */
#elif defined LOAD_BY_DBUS_DMA
		psram_dma_read_write(1, (uint32_t)(IDCACHE_START_ADDR + put), buf, size);
#elif defined LOAD_BY_SBUS_CPU
		psram_sbus_write(chip, put, buf, size);
#elif defined LOAD_BY_SBUS_DMA
		psram_sbus_dma_write(chip, put, buf, size);
#endif
		HAL_Dcache_FlushCleanAll();
#if PSRAM_DBG_CHECK
		//PSRAM_DUMP((const void *)buf, 64);
#ifdef LOAD_BY_DBUS_CPU
		//PSRAM_DUMP((const void *)(IDCACHE_START_ADDR + put), 64);
#elif defined LOAD_BY_DBUS_DMA
		//memset(buf, 0, 64);
		//PSRAM_DUMP(0, (uint32_t)(IDCACHE_START_ADDR + put), buf, 64);
		//PSRAM_DUMP((const void *)buf, 64);
#elif defined LOAD_BY_SBUS_CPU
		memset(buf + LOAD_SIZE, 0, size);
		psram_sbus_read(chip, put, buf + LOAD_SIZE, size);
		if (memcmp(buf, buf + LOAD_SIZE, size)) {
			PSRAM_ERR("%s,%d idx:%d\n", __func__, __LINE__, i);
			//PSRAM_DUMP((const void *)(buf + LOAD_SIZE), 64);
			ret = -1;
			goto out;
		}
		//PSRAM_DUMP((const void *)(buf + LOAD_SIZE), 64);
#elif defined LOAD_BY_SBUS_DMA
		memset(buf + LOAD_SIZE, 0, size);
		psram_sbus_dma_read(chip, put, buf + LOAD_SIZE, size);
		if (memcmp(buf, buf + LOAD_SIZE, size)) {
			PSRAM_ERR("%s,%d idx:%d\n", __func__, __LINE__, i);
			PSRAM_DUMP((const void *)(buf + LOAD_SIZE), 64);
			ret = -1;
			goto out;
		}
		//PSRAM_DUMP((const void *)buf, 64);
#endif
#endif
#ifdef LOAD_BY_DBUS_FLASH_DMA
		buf += size;
#endif
		put += size;
	}

#if ((defined LOAD_BY_DBUS_FLASH_DMA) || (defined LOAD_BY_DBUS_DMA) || (defined LOAD_BY_SBUS_DMA))
	HAL_Dcache_SetWriteThrough(0, 0, 0, 0);
#endif

	if (len != sh.body_len) {
		PSRAM_ERR("psram body size %u, read %u\n", sh.body_len, len);
		ret = -1;
		goto out;
	}

#if ((defined __CONFIG_PSRAM_CHIP_SQPI) && ((defined LOAD_BY_SBUS_CPU) || (defined LOAD_BY_SBUS_DMA)))
	psram_sw_reset(chip, 0);
#endif

	if (image_check_data(&sh, (void *)IDCACHE_START_ADDR, sh.body_len,
	                     NULL, 0) == IMAGE_INVALID) {
		PSRAM_ERR("invalid psram bin body\n");
		//PSRAM_DUMP((const void *)IDCACHE_START_ADDR, sh.body_len);
		ret = -1;
		goto out;
	}

clr_bss:
	memset(__psram_bss_start__, 0, __psram_bss_end__ - __psram_bss_start__);
	HAL_Dcache_FlushCleanAll();
#if PSRAM_DBG_CHECK
	for (uint32_t *addr = (uint32_t *)__psram_bss_start__;
	     addr < (uint32_t *)__psram_bss_end__; addr++) {
		if (readl(addr)) {
			PSRAM_ERR("bss not cleared! at[%p]:0x%x\n", addr, readl(addr));
			break;
		}
	}
#endif
	PSRAM_INF("__psram_start__\t%p\n", __psram_start__);
	PSRAM_INF("__psram_data_start__\t%p\n", __psram_data_start__);
	PSRAM_INF("__psram_data_end__\t%p\n", __psram_data_end__);
	PSRAM_INF("__psram_bss_start__\t%p\n", __psram_bss_start__);
	PSRAM_INF("__psram_bss_end__\t%p\n", __psram_bss_end__);
	PSRAM_INF("__psram_end__\t\t%p\n", __psram_end__);

out:
#ifndef LOAD_BY_DBUS_FLASH_DMA
	free(buf);
#endif
	return ret;
}

__sram_data
static struct psram_chip chip = {0};

__sram_text
void platform_psram_init(void)
{
	extern uint8_t __psram_bss_end__[];

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
	PSRAM_DBG("psram chip %s init ok!, freq %d\n", chip.name, cfg.freq);

	if (load_psram_bin_and_set_addr(&chip)) {
		PSRAM_ERR("load psram bin faild!\n");
		goto out;
	}
	PSRAM_DBG("load psram bin ok\n");
	HAL_Dcache_SetWriteThrough(0, 1, rounddown2((uint32_t)__psram_bss_end__, 16), IDCACHE_END_ADDR);
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
