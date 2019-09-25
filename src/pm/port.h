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

#ifndef __PM_PORT_H
#define __PM_PORT_H

#include "driver/chip/hal_nvic.h"
#include "driver/chip/hal_rtc.h"
#include "driver/chip/hal_util.h"

#if (__CONFIG_CHIP_ARCH_VER >= 1)
#define ktime_t uint64_t
#define ktime_get() (HAL_RTC_GetFreeRunTime() / 1000)
#define ktime_to_msecs(t) (t)
#else
#define ktime_t uint32_t
#define ktime_get() OS_GetTicks()
#define ktime_to_msecs(t) OS_MSecsToJiffies(t)
#endif

#define arch_suspend_disable_irqs __disable_irq
#define arch_suspend_enable_irqs __enable_irq

#define nop() __NOP()
#define isb() __ISB()
#define dsb() __DSB()
#define dmb() __DMB()
#define wfi() __WFI()
#define wfe() __WFE()

#define PM_SYS "appos"
#define PM_SetCPUBootFlag(f) HAL_PRCM_SetCPUABootFlag(f)

/* Internal reg, should not used outside.
 * used record state during hibernation and poweroff,
 * used reocrd resume arg when sleep and standby.
 */
#define PM_SetCPUBootArg(a) HAL_PRCM_SetCPUABootArg(a)
#define PM_SystemDeinit() SystemDeInit(SYSTEM_DEINIT_FLAG_RESET_CLK)
#define pm_udelay(us) HAL_UDelay(us)
#define PM_REBOOT() HAL_NVIC_CPUReset()
#define PM_IRQ_SAVE arch_irq_save
#define PM_IRQ_RESTORE arch_irq_restore
#define PM_IRQ_GET_FLAGS arch_irq_get_flags

#define __set_last_record_step(s) HAL_PRCM_SetCPUAPrivateData(0, s)
#define __get_last_record_step() HAL_PRCM_GetCPUAPrivateData(0)

static __always_inline void __record_dbg_status(int val)
{
	__set_last_record_step(val);
	dsb();
	isb();
}

extern unsigned int nvic_int_mask[];

extern int platform_prepare(enum suspend_state_t state);
extern void platform_wake(enum suspend_state_t state);

#endif /* __PM_PORT_H */
