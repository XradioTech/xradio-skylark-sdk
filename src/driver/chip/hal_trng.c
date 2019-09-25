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

#include "stdio.h"
#include "driver/chip/hal_trng.h"
#include "./crypto/drv_trng.h"

#if (__CONFIG_CHIP_ARCH_VER == 2)

#define HAL_TRNG_JITTER_TEST_ENABLE 0
/**
  * @brief get trng random number.
  * @param type    : crypto method.
  * @param random: return random numer
  * @retval HAL_Status:  the status of driver
  */
HAL_Status HAL_TRNG_Extract(uint8_t type, uint32_t random[4])
{
	HAL_Status ret = HAL_OK;
	uint64_t actime = 0;
	uint64_t access_time = 0;

	DRV_Trng_Init(&actime);
	#if HAL_TRNG_JITTER_TEST_ENABLE
	if(DRV_Trng_Jitter_Test(&actime) != 0) {
		printf("HAL TRNG: jitter test failed\n");
		ret = HAL_ERROR;
		goto out;
	}
	#endif
	if(DRV_Trng_Extract(type, random, &actime) != 0) {
		printf("HAL TRNG: jitter extract failed\n");
		ret = HAL_ERROR;
		goto out;
	}
	access_time = DRV_Trng_Reg_Access_Time();
	if(access_time != actime) {
		printf("HAL TRNG: actually time(%lu) != trng_reg_access_time(%lu)\n", (unsigned long)actime, (unsigned long)access_time);
		ret = HAL_ERROR;
		goto out;
	}

out:
	DRV_Trng_Deinit();
	return ret;
}

#endif /* (__CONFIG_CHIP_ARCH_VER == 2) */
