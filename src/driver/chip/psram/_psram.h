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

#ifndef __PSRAM_I_H_
#define __PSRAM_I_H_

#define DBG_PR 1

#if (DBG_PR == 1)
#define PR_LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)
#define PR_DBG(fmt, arg...) PR_LOG(0, fmt, ##arg)
#define PR_INF(fmt, arg...) PR_LOG(DBG_PR, fmt, ##arg)
#define PR_WRN(fmt, arg...) PR_LOG(DBG_PR, "[PR WRN] "fmt, ##arg)
#define PR_ERR(fmt, arg...) PR_LOG(DBG_PR, "[PR ERR] "fmt, ##arg)
#else
#define PR_INF(...)
#define PR_WRN(...)
#define PR_ERR(...)
#endif

#define SQPI_BUSWIDTH1_READ   S_FAST_READ
#define SQPI_BUSWIDTH1_WRITE  S_WRITE

#define SQPI_BUSWIDTH4_READ   Q_FAST_READ_QUAD
#define SQPI_BUSWIDTH4_WRITE  Q_WRITE

int32_t HAL_PsramCtrl_ConfigCCMU(uint32_t clk);
int32_t psram_set_driver_strength(struct psram_chip *chip, uint32_t drv);
void psram_sbus_op_cmd(struct psram_chip *chip, uint32_t opcmd);
int32_t psram_sqpi_init(struct psram_chip *chip, struct psram_ctrl *ctrl);
int32_t psram_aps32_init(struct psram_chip *chip, struct psram_ctrl *ctrl);
int32_t psram_aps64_init(struct psram_chip *chip, struct psram_ctrl *ctrl);

#endif /* __PSRAM_I_H_ */
