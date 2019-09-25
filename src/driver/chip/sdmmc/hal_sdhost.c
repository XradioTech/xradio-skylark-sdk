/**
  * @file  hal_sdhost.c
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

#include "sys/io.h"

#include "driver/chip/private/hal_debug.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_nvic.h"
#include "driver/chip/hal_gpio.h"
#include "driver/hal_board.h"

#include "driver/hal_dev.h"
#include "driver/chip/hal_clock.h"

#include "driver/chip/sdmmc/hal_sdhost.h"

#include "_sd_define.h"
#include "_sdhost.h"
#include "_core.h"

#define SDXC_REG_NTSR                   (0x5C)
#define SDXC_REG_DELAY_CTRL             (0x144)

#define mci_readl(host, reg) \
	readl((uint32_t)(host)->reg_base + reg)
#define mci_writel(value, host, reg) \
	writel((value), (uint32_t)(host)->reg_base + reg)

static void HAL_SDC_Calibrate(struct mmc_host *host)
{
	HAL_ASSERT_PARAM(host);

	if (host->sdc_id == 0)
		host->caps &= ~(MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED);

	mci_writel(mci_readl(host, SDXC_REG_NTSR) & (~(1 << 31)), host, SDXC_REG_NTSR);
	mci_writel(mci_readl(host, SDXC_REG_DELAY_CTRL) & (~(1 << 7 )), host, SDXC_REG_DELAY_CTRL);	//disable sample delay
	mci_writel(mci_readl(host, SDXC_REG_DELAY_CTRL) | (1 << 15), host, SDXC_REG_DELAY_CTRL);	//enanle sample delay calibration

	uint32_t time = 0;
	while (!(mci_readl(host, SDXC_REG_DELAY_CTRL) & (1 << 14))) {
		time++;
		if (time == 1024)
			break;
	}
	mci_writel(mci_readl(host, SDXC_REG_DELAY_CTRL) | 0x4, host, SDXC_REG_DELAY_CTRL);
	mci_writel(mci_readl(host, SDXC_REG_DELAY_CTRL) | (1 << 7), host, SDXC_REG_DELAY_CTRL);	//enable sample delay

#ifdef __CONFIG_ROM
	host->clk = 400000;
#endif
}


#ifdef __CONFIG_ROM
extern struct mmc_host *_mci_host[SDC_NUM];
SDC_Semaphore _sdc_lock[SDC_NUM];

int32_t HAL_SDC_Claim_Host(struct mmc_host *host)
{
	HAL_ASSERT_PARAM(host->sdc_id < SDC_NUM);

	return (OS_SemaphoreWait(&_sdc_lock[host->sdc_id], OS_WAIT_FOREVER) == OS_OK ? 0 : -1);
}

void HAL_SDC_Release_Host(struct mmc_host *host)
{
	HAL_ASSERT_PARAM(host->sdc_id < SDC_NUM);
	OS_SemaphoreRelease(&_sdc_lock[host->sdc_id]);
}

struct mmc_host *HAL_SDC_Init(struct mmc_host *host)
{
	HAL_ASSERT_PARAM(host->sdc_id < SDC_NUM);

	OS_SemaphoreCreate(&_sdc_lock[host->sdc_id], 1, OS_SEMAPHORE_MAX_COUNT);

	_HAL_SDC_Init(host);
	HAL_SDC_Calibrate(host);

	return host;
}

int32_t HAL_SDC_Deinit(uint32_t sdc_id)
{
	struct mmc_host *host = _mci_host[sdc_id];

	if (_HAL_SDC_Deinit(sdc_id)) {
		return -1;
	}
	_mci_host[sdc_id] = host;
	host->sdc_id = sdc_id;

	OS_SemaphoreDelete(&_sdc_lock[sdc_id]);

	return 0;
}
#endif
