/**
  * @file  hal_xip.h
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

#ifndef _DRIVER_CHIP_HAL_XIP_H_
#define _DRIVER_CHIP_HAL_XIP_H_

#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/flashchip/flash_chip.h"

#ifdef __cplusplus
extern "C" {
#endif

struct XipDrv;

int HAL_Xip_setCmd(struct XipDrv *xip, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data);
int HAL_Xip_setDelay(struct XipDrv *xip, Flashc_Delay *delay);
int HAL_Xip_setAddr(struct XipDrv *xip, uint32_t addr);
int HAL_Xip_setContinue(struct XipDrv *xip, uint32_t continueMode, void *arg);

HAL_Status HAL_Xip_Init(uint32_t flash, uint32_t xaddr);
HAL_Status HAL_Xip_Deinit(uint32_t flash);
void HAL_Xip_SetDbgMask(uint8_t dbg_mask);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_XIP_H_ */
