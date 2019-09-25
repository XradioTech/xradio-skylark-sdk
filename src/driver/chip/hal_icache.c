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

#include "hal_base.h"
#include "driver/chip/hal_icache.h"
#include "pm/pm.h"

#include "sys/xr_debug.h"

#ifdef __CONFIG_ROM
#ifdef CONFIG_PM
static ICache_Config _icache_cfg;

static int icache_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_ICache_Flush();
		HAL_ICache_DeConfig();
		break;
	default:
		break;
	}
	return 0;
}

static int icache_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		HAL_ICache_Config(&_icache_cfg);
		break;
	default:
		break;
	}
	return 0;
}

static const struct soc_device_driver icache_drv = {
	.name = "icache",
	.suspend_noirq = icache_suspend,
	.resume_noirq = icache_resume,
};

static struct soc_device icache_dev = {
	.name = "icache",
	.driver = &icache_drv,
};

#define ICACHE_DEV (&icache_dev)

#endif/*CONFIG_PM*/

HAL_Status HAL_ICache_Init(ICache_Config *cfg)
{
	HAL_ASSERT_PARAM(cfg);

	HAL_ICache_Config(cfg);

#ifdef CONFIG_PM
	HAL_Memcpy(&_icache_cfg, cfg, sizeof(ICache_Config));
	pm_register_ops(ICACHE_DEV);
#endif

	return HAL_OK;
}

HAL_Status HAL_ICache_Deinit(void)
{
#ifdef CONFIG_PM
	pm_unregister_ops(ICACHE_DEV);
	HAL_Memset(&_icache_cfg, 0, sizeof(ICache_Config));
#endif
	HAL_ICache_DeConfig();

	return HAL_OK;
}

#endif/*__CONFIG_ROM*/

