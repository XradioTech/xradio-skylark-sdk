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

#if (defined __CONFIG_PSRAM_CHIP_OPI64)

#include <string.h>
#include <stdio.h>

#include "sys/xr_debug.h"
#include "sys/io.h"
#include "pm/pm.h"

#include "../hal_base.h"
#include "driver/chip/hal_dma.h"

#include "driver/chip/psram/psram.h"
#include "driver/chip/psram/hal_psramctrl.h"
#include "_psram.h"

#include "driver/chip/hal_icache.h"
#include "driver/chip/hal_dcache.h"

/* define for tCPH
 * freq./tCPH, WL, Write[Min tCPH, Max BL], Read[Min tCPH, Max BL]
 *   133MHz,   0,          4,        656,           2,        2
 *   166MHz,   0,          4,        768,           2,        2
 *   200MHz,   2,          4,        768,           2,        2
 *   200MHz,   2,          6,        984,           2,        2
 *   233HMz,   2,          8,       1024,           2,        2
 */

/* define for Latency
 * MR0[5]=0b(Variable Latency)
 * MR0[4:2], Latency, Max push out, Standard, Extended
 *   010        2          4           66        66
 *   011        3          6          109       109
 *   100        4(default) 8          133       133
 *   101        5         10          166       166
 *   110        6         12        200/233    200/233
 * MR0[5]=1b(Fixed Latency)
 * MR0[4:2], Latency, Standard, Extended
 *   010        4           66        66
 *   011        6          109       109
 *   100        8(default) 133       133
 *   101       10          166       166
 *   110       12        200/233    200/233
 */

int32_t psram_aps64_init(struct psram_chip *chip, struct psram_ctrl *ctrl)
{
    int32_t id;

    chip->buswidth = 8;
    chip->wrap_len = PSRAMC_PAGE_SZ_1K;

    psram_sw_reset(chip, 0);

    id = Psram_Read_Mr(chip, 0);
    if ((id & 0x0f) != 0x0d) { /* Vendor ID: 01101b(APM) */
        PR_ERR("Get APS64 wrong ID:0x%x\n", id);
        return -1;
    }
    PR_DBG("Get APS64 ID:0x%x\n", id);

    //Psram_Read_Info(chip);
    id = Psram_Read_die_sta(chip);
    if (!id) {
        PR_ERR("APS64 BAD die:0x%x\n", id);
        return -1;
    }

    HAL_PsramCtrl_MaxCE_LowCyc(chip->ctrl, chip->freq);

    psram_set_read_latency(chip, 0, 7); /* see TBL Latency */
    psram_set_write_latency(chip, PSRAM_CHIP_OPI_APS64, 7);
    HAL_PsramCtrl_Set_SBUS_WR_LATENCY(chip->ctrl, 6);
    HAL_PsramCtrl_Set_DBUS_WR_LATENCY(chip->ctrl, 6 << 8);

    psram_idbus_op_cmd(chip, O_SYNC_BURST_WRITE);
    psram_idbus_op_cmd(chip, O_SYNC_BURST_READ);

    HAL_PsramCtrl_CacheCfg(chip->ctrl, 0);
    HAL_PsramCtrl_IDBUS_Dma_Enable(1);

    chip->name = "APS64";
    chip->capacity = 8 * 1024 * 1024;

    HAL_UDelay(1000);

    return 0;
}

#endif /* __CONFIG_PSRAM_CHIP_OPI64 */
