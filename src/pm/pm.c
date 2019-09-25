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

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h> /* for timeofday */

#include "sys/io.h"
#include "errno.h"
#include "sys/list.h"
#include "sys/interrupt.h"
#include "sys/param.h"
#include "kernel/os/os_thread.h"
#include "kernel/os/os_queue.h"

#include "driver/chip/system_chip.h"
#include "driver/chip/chip.h"
#include "driver/chip/hal_wakeup.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_util.h"
#include "driver/chip/hal_nvic.h"
#if (__CONFIG_CHIP_ARCH_VER == 1)
#include "driver/chip/hal_mbox.h"
#endif

#include "pm/pm.h"
#include "_pm_define.h"
#include "pm_i.h"
#include "port.h"

#ifdef CONFIG_PM

static struct arm_CMX_core_regs vault_arm_registers;

#define PM_TIMEOFDAY_SAVE() timeofday_save()

#ifdef CONFIG_PM_DEBUG
#include "sys/xr_debug.h"

#define PM_DEBUG_DUMP_NUM 2

/* len, addr */
static uint32_t pm_debug_dump_addr[PM_DEBUG_DUMP_NUM][2];

/**
 * @brief Set dump addr and len for debug.
 */
void pm_set_dump_addr(uint32_t addr, uint32_t len, uint32_t idx)
{
	if (idx >= PM_DEBUG_DUMP_NUM) {
		PM_LOGE("only support %d dump\n", PM_DEBUG_DUMP_NUM);
		return ;
	}

	pm_debug_dump_addr[idx][0] = len;
	pm_debug_dump_addr[idx][1] = addr;
}
#else
void pm_set_dump_addr(uint32_t addr, uint32_t len, uint32_t idx)
{
}
#endif

/**
 * @brief shutdown sram region when standby mode, else retention
 *
 * @param sramN pm_sram_N or combination
 * enum pm_sram_N {
 *  PM_SRAM_0   = PRCM_SYS_SRAM_32K_SWM4_BIT,           //sram 0x200000 ~ 0x207FFF
 *  PM_SRAM_1   = PRCM_SYS_SRAM_32K_SWM3_BIT,           //sram 0x208000 ~ 0x20FFFF
 *  PM_SRAM_2   = PRCM_SYS_SRAM_352K_SWM2_BIT,          //sram 0x210000 ~ 0x25FFFF
 *};
 *
 * @retval None.
 * @example pm_standby_sram_retention_only(PM_SRAM_0 | PM_SRAM_1)
 */
void pm_standby_sram_retention_only(uint32_t sramN)
{
    HAL_MODIFY_REG(PRCM->SYS1_SLEEP_CTRL, (0x7 << 25), ((~sramN) & (0x7 << 25)));
}

static int __suspend_begin(enum suspend_state_t state)
{
	/* set SEVONPEND flag */
	SCB->SCR = 0x10;

	return 0;
}

/* hibernation whole system, save user data to flash before call this func.
 * BE SUURE: all hardware has been closed and it's prcm config setted to default value.
 */
static void pm_hibernation()
{
	__record_dbg_status(PM_HIBERNATION | 0);

#ifdef __CONFIG_ARCH_APP_CORE
	/* step 1 & 2 has been done when wlan sys poweroff */
	/* step3: writel(0x0f, GPRCM_SYS1_WAKEUP_CTRL) to tell PMU that turn on
	 * SW1, SW2, SRSW1, LDO before release application system reset signal. */
#if (__CONFIG_CHIP_ARCH_VER == 1)
    HAL_PRCM_SetSys1WakeupPowerFlags(0x0F);
    HAL_PRCM_SetSys1SleepPowerFlags(0x0F);
#else
    HAL_PRCM_SetSys1WakeupPowerFlags(0x03);
	HAL_PRCM_SetSys1SleepPowerFlags(0x3F | PRCM_SYS_SRAM_PWR_CTRL_MASK | PRCM_SYS_CACHE_SRAM_PWR_CTRL_BIT | (0x1FU << 24));
    HAL_PRCM_SetEXTLDOMdoe(PRCM_EXTLDO_ALWAYS_ON);
    HAL_PRCM_SetTOPLDOForceActive(0);
    HAL_PRCM_EnableTOPLDODeepsleep(0);
    HAL_PRCM_EnableLDOModeSWSelEnable(0);
    HAL_PRCM_EnableTOPLDOLQModeEnable(0);
    HAL_PRCM_EnableSysLDOLQModeEnable(0);
#endif
	/* step4: writel(0x0f, GPRCM_SYS1_SLEEP_CTRL) to tell PMU that turn off SW1,
	 * SW3 SRSW1, LDO after pull down application system reset signal. */
	__record_dbg_status(PM_HIBERNATION | 7);

	/* step5: switch to HOSC, close SYS1_CLK. */
	PM_SystemDeinit();
	__record_dbg_status(PM_HIBERNATION | 9);

	/* step6: set nvic deepsleep flag, and enter wfe. */
	SCB->SCR = 0x14;
	PM_SetCPUBootFlag(PRCM_CPUA_BOOT_FROM_COLD_RESET);
	PM_TIMEOFDAY_SAVE();

	__disable_fault_irq();
	__disable_irq();

	if (check_wakeup_irqs()) {
		PM_REBOOT();
	}

	wfe();
	if (check_wakeup_irqs()) {
		PM_REBOOT();
	}
	wfe();
	/* some irq generated when second wfe */
	PM_REBOOT();
	__record_dbg_status(PM_HIBERNATION | 0x0ff);

#else /* net cpu */
	/* check wifi is closed by app? */
	PM_WARN_ON(NULL, HAL_PRCM_IsSys3Release());

	/* step1: cpu to switch to HOSC and turn off SYSCLK2 */
	HAL_PRCM_SetCPUNClk(PRCM_CPU_CLK_SRC_HFCLK, PRCM_SYS_CLK_FACTOR_80M);
	__record_dbg_status(PM_HIBERNATION | 1);

	PM_SystemDeinit();

	/* step3: enter WFI state */
	arch_suspend_disable_irqs();
	while (1)
		wfi();
	__record_dbg_status(PM_HIBERNATION | 0x0ff);
#endif
}

static void __suspend_enter(enum suspend_state_t state)
{
	__record_dbg_status(PM_SUSPEND_ENTER | 5);

	__record_dbg_status(PM_SUSPEND_ENTER | 6);
	if (HAL_Wakeup_SetSrc(1))
		return ;

	PM_LOGN("device info. rst:%x clk:%x\n", CCM->BUS_PERIPH_RST_CTRL,
	        CCM->BUS_PERIPH_CLK_CTRL); /* debug info. */

	PM_SetCPUBootArg((uint32_t)&vault_arm_registers);

#ifdef CONFIG_PM_DEBUG
	for (int i = 0; i < PM_DEBUG_DUMP_NUM; i++) {
		if (pm_debug_dump_addr[i][0])
			print_hex_dump_words((const void *)pm_debug_dump_addr[i][1],
			                     pm_debug_dump_addr[i][0]);
	}
#endif

	if (state == PM_MODE_HIBERNATION) {
		__record_dbg_status(PM_SUSPEND_ENTER | 7);
		PM_SetCPUBootArg(PM_MODE_MAGIC | state);
#ifdef __CONFIG_ARCH_APP_CORE
		HAL_Wakeup_SetIOHold((1 << WAKEUP_IO_MAX) - 1);
#endif
		pm_hibernation(); /* never return */
	} else if (state < PM_MODE_STANDBY) {
		__record_dbg_status(PM_SUSPEND_ENTER | 8);
		/* TODO: set system bus to low freq */
		__cpu_sleep(state);
		/* TODO: restore system bus to normal freq */
	} else {
        HAL_PRCM_SetSys1WakeupPowerFlags(0x6);
        HAL_PRCM_SetSys1SleepPowerFlags(0x1E);
#if ((((__CONFIG_CACHE_POLICY>>4) & 0xF) + (__CONFIG_CACHE_POLICY& 0xF)) == 5)
        HAL_SET_BIT(PRCM->SYS1_SLEEP_CTRL, PRCM_SYS_CACHE_SRAM_SWM1_BIT);
#endif
#ifndef __CONFIG_WLAN
        HAL_SET_BIT(PRCM->SYS1_SLEEP_CTRL, PRCM_SYS_WLAN_SRAM_116K_SWM5_BIT);
#endif
		__record_dbg_status(PM_SUSPEND_ENTER | 9);
		__cpu_suspend(state);
	}

	PM_BUG_ON(NULL, !PM_IRQ_GET_FLAGS());

	__record_dbg_status(PM_SUSPEND_ENTER | 0xa);
	HAL_Wakeup_ClrSrc(1);

	__record_dbg_status(PM_SUSPEND_ENTER | 0xb);
}

static void __suspend_end(enum suspend_state_t state)
{
	/* clear SEVONPEND flag */
	SCB->SCR = 0x0;
}

#ifdef CONFIG_PM_DEBUG
void pm_dump_regs(unsigned int flag)
{
	if (flag & 1<<0) { /* cpu */
		int i, j;

		PM_LOGD("regs:\n");
		PM_LOGD("msp:0x%08x, psp:0x%08x, psr:0x%08x, primask:0x%08x\n",
			vault_arm_registers.msp, vault_arm_registers.psp,
			vault_arm_registers.psr, vault_arm_registers.primask);
		PM_LOGD("faultmask:0x%08x, basepri:0x%08x, control:0x%08x\n",
			vault_arm_registers.faultmask, vault_arm_registers.basepri,
			vault_arm_registers.control);
		for (i = 0; i < 3; i++) {
			for (j = 0; j < 4; j++) {
				PM_LOGD("reg[%d]:0x%08x ", j+i*4,
				        vault_arm_registers.reg12[j+i*4]);
			}
			PM_LOGD("\n");
		}
		PM_LOGD("last step:%x\n", __get_last_record_step());
	}
	if (flag & 1<<1) { /* nvic */
		//nvic_print_regs();
	}
	if (flag & 1<<2) { /* ccmu */
		//ccm_print_regs();
	}
	if (flag & 1<<3) { /* gpio */
		//gpio_print_regs();
	}
}

#else
#define suspend_test_start()
#define suspend_test_finish(x...)
#define suspend_test(l)         0
void pm_set_test_level(enum suspend_test_level_t level) {;}
#define pm_dump_regs(flag)
#endif

/**
 * check if any wake-up interrupts are pending
 */
int pm_check_wakeup_irqs(void)
{
	int i;
	unsigned int *addr;
	unsigned int val, ret = 0;

	/* Check if there are any interrupts pending */
	addr = (unsigned int *)NVIC->ISPR;
	for (i = 0; i < DIV_ROUND_UP(NVIC_PERIPH_IRQ_NUM, 32); i++) {
		val = addr[i];
		if (val & nvic_int_mask[i]) {
			PM_LOGN("nvic[%d]:%x, mask:%x en:%x\n", i, val,
			        nvic_int_mask[i], NVIC->ISER[i]);
			if ((val & (1 << A_WAKEUP_IRQn)) || (val & (1 << GPIOA_IRQn))) {
				ret = PM_WAKEUP_SRC_WKTIMER | (PM_WAKEUP_SRC_WKTIMER - 1);
#if (__CONFIG_CHIP_ARCH_VER == 1)
			} else if ((val & (1 << N_UART_IRQn)) && HAL_MBOX_IsPmPatchEnabled()) { /* mbox has disabled */
#elif (__CONFIG_CHIP_ARCH_VER == 2)
			} else if (val & (1 << WIFIC_IRQn)) {
#endif
				PM_LOGN("wakeup wlan pending\n");
				return PM_WAKEUP_SRC_WLAN;
#if (__CONFIG_CHIP_ARCH_VER == 2)
			} else if (val & (1 << RTC_SEC_ALARM_IRQn)) {
				PM_LOGN("RTC pending\n");
				return PM_WAKEUP_SRC_RTC_SEC;
			} else if (val & (1 << RTC_WDAY_ALARM_IRQn)) {
				PM_LOGN("RTC pending\n");
				return PM_WAKEUP_SRC_RTC_WDAY;
#endif
			} else {
				PM_LOGN("wakeup devices pending\n");
				return PM_WAKEUP_SRC_DEVICES;
			}
		}
	}

	if (HAL_Wakeup_ReadTimerPending()) {
		PM_LOGN("wakeup timer pending\n");
		return PM_WAKEUP_SRC_WKTIMER;
	}

#ifdef __CONFIG_ARCH_APP_CORE
	ret = HAL_Wakeup_ReadIO();
	if (ret) {
		PM_LOGN("wakeup io pending\n");
		return ret;
	}
#endif

	return ret;
}

/**
 * @brief Initialize the PM-related part of a device object.
 * @note not use printf for this fun is called very earlier.
 * @retval  0 if success or other if failed.
 */
int pm_init(void)
{
	uint32_t mode;
	struct platform_suspend_ops suspend_ops;
    //pm_set_debug_mask(ROM_TOTAL_MASKS);

	mode = HAL_PRCM_GetCPUABootArg();
	if (mode == (PM_MODE_MAGIC | PM_MODE_HIBERNATION)) {
		PM_SetCPUBootArg(PM_MODE_MAGIC);
		PM_REBOOT();
	}
	if (mode == PM_MODE_MAGIC)
		HAL_Wakeup_ClrSrc(0);

	HAL_Wakeup_Init();

	suspend_ops.begin = __suspend_begin;
	suspend_ops.prepare = platform_prepare;
	suspend_ops.enter = __suspend_enter;
	suspend_ops.wake = platform_wake;
	suspend_ops.end = __suspend_end;
	suspend_ops_init(&suspend_ops);

#ifdef __CONFIG_ARCH_APP_CORE
#if 0 /* enable this if only APP CPU used */
	HAL_PRCM_EnableSys2Power();
	pm_udelay(10000);
	HAL_PRCM_DisableSys2Power();
	pm_udelay(10000);
#endif

	/* set prcm to default value for prcm keep it's last time value. */
#if (__CONFIG_CHIP_ARCH_VER == 1)
    HAL_PRCM_SetSys1WakeupPowerFlags(0x0F);
    HAL_PRCM_SetSys1SleepPowerFlags(0x0E);
	HAL_PRCM_Start();
#else
    HAL_PRCM_SetSys1WakeupPowerFlags(0x0F);
	HAL_PRCM_SetSys1SleepPowerFlags(0x0);
	HAL_PRCM_EnableLDOModeSWSelEnable(0);
#endif

#endif

	return 0;
}

#ifdef __CONFIG_ARCH_APP_CORE
static pm_wlan_power_onoff pm_wlan_power_onoff_cb = NULL;
static int pm_wlan_mode_platform_config = PM_SUPPORT_HIBERNATION;

/**
 * @brief Select wlan power modes when enter pm.
 * @note Wlan power on/off calback will called when pm enter select modes.
 * @param wlan_power_cb:
 *        @arg wlan_power_cb->Wlan power on/off calback.
 * @param select:
 *        @arg select->The selected modes set.
 * retval  0 if success or other if failed.
 */
int pm_register_wlan_power_onoff(pm_wlan_power_onoff wlan_power_cb, unsigned int select)
{
	if ((select & (PM_SUPPORT_HIBERNATION)) !=
	    (PM_SUPPORT_HIBERNATION)) {
		PM_LOGW("wlan should power off when hibernateion/poweroff!\n");
		return -1;
	}
	pm_wlan_power_onoff_cb = wlan_power_cb;
	pm_wlan_mode_platform_config = select | PM_SUPPORT_HIBERNATION;
	PM_LOGN("wlan mode:%x\n", pm_wlan_mode_platform_config);

	return 0;
}

/** @brief unregister wlan power on/off callback. */
void pm_unregister_wlan_power_onoff(void)
{
	pm_wlan_power_onoff_cb = NULL;
}
#endif

#ifdef __CONFIG_ARCH_APP_CORE
static int pm_mode_platform_config = PM_SUPPORT_SLEEP | PM_SUPPORT_STANDBY | \
	PM_SUPPORT_HIBERNATION;

/**
 * @brief Select pm modes used on this platform.
 * @note Select modes at init for some modes are not used on some platforms.
 *        This will prevent enter unselect modes.
 * @param select:
 *        @arg select->The selected modes set.
 */
void pm_mode_platform_select(unsigned int select)
{
	pm_mode_platform_config = select;
	if (!(select & (PM_SUPPORT_SLEEP | PM_SUPPORT_STANDBY | \
	    PM_SUPPORT_HIBERNATION))) {
		PM_LOGW("slect wrong mode!\n");
		return ;
	}
	PM_LOGN("mode select:%x\n", select);
}

static int pm_wlan_alive_platform_config = PM_SUPPORT_SLEEP | PM_SUPPORT_STANDBY;

int pm_wlan_alive_platform_select(unsigned int select)
{
	if (select & (PM_SUPPORT_HIBERNATION)) {
		PM_LOGW("net can't be alive when hiberantion or poweroff!\n");
		return -1;
	}
	pm_wlan_alive_platform_config = select;
	PM_LOGN("net alive select:%x\n", select);

	return 0;
}

/**
 * @brief Set a magin to synchronize with net.
 */
void pm_set_sync_magic(void)
{
	PM_SetCPUBootArg(PM_SYNC_MAGIC); /* set flag to notify net to run */
}
#endif


/**
 * @brief Set system to a lowpower mode.
 * @param state:
 *        @arg state->The lowpower mode will enter.
 * @retval  0 if success or other if failed.
 */
int pm_enter_mode(enum suspend_state_t state)
{
	int err, record;
	enum suspend_state_t state_use = state;
#ifdef __CONFIG_ARCH_APP_CORE
	int net_alive, loop;

	if (!(pm_mode_platform_config & (1 << state))) {
		for (loop = (1 << state_use); loop; loop >>= 1) {
			if (pm_mode_platform_config & loop) {
				break;
			}
			state_use--;
		}
	}
#endif

	if (state_use >= PM_MODE_MAX) {
		PM_LOGE("%s:%d err mode:%d!\n", __func__, __LINE__, state_use);
		return -1;
	}

	HAL_Wakeup_SetEvent(0);

	if (state_use < PM_MODE_SLEEP)
		return 0;

	pm_select_mode(state_use);
	PM_LOGA(PM_SYS" enter mode: %s\n", pm_states[state_use]);
	record = __get_last_record_step();
	if (record != PM_RESUME_COMPLETE)
		PM_LOGN("last suspend record:%x\n", record);
#ifdef CONFIG_PM_DEBUG
	parse_dpm_list(PM_OP_NORMAL);  /* debug info. */
	parse_dpm_list(PM_OP_NOIRQ);
#endif
#ifdef __CONFIG_ARCH_APP_CORE
#if (__CONFIG_CHIP_ARCH_VER == 1)
	net_alive = HAL_PRCM_IsCPUNReleased();
#else
	net_alive = 1;
#endif
	if (net_alive && (pm_wlan_mode_platform_config & (1 << state_use)) &&
	    pm_wlan_power_onoff_cb) {
		pm_wlan_power_onoff_cb(0);
	}
#endif
	PM_SetCPUBootArg(PM_MODE_MAGIC | state_use);

	/* Disable inter 32K calibration before enter standby, hibernation,
	 * poweroff, etc. to avoid wrong RTC time counting.
	 */
	int is_32k_calib_ena = 0;
	if (state >= PM_MODE_STANDBY) {
        if(state == PM_MODE_STANDBY) {
            uint16_t  u16RrefTime;
            u16RrefTime = 10*HAL_GetHFClock()/HAL_GetLFClock() + 0x200;
            HAL_PRCM_SetDigSWRefTime(u16RrefTime);
        }
		is_32k_calib_ena = HAL_PRCM_IsInter32KCalibEnabled();
		if (is_32k_calib_ena) {
			HAL_PRCM_DisableInter32KCalib();
		}
	}

	err = suspend_devices_and_enter(state_use);

	if (is_32k_calib_ena) {
		HAL_PRCM_EnableInter32KCalib();
	}

#ifdef __CONFIG_ARCH_APP_CORE
	pm_set_sync_magic();

	if (net_alive && (pm_wlan_mode_platform_config & (1 << state_use)) &&
	    pm_wlan_power_onoff_cb) {
		pm_wlan_power_onoff_cb(1);
	}
#endif

	return err;
}

#ifdef CONFIG_PM_THREAD
struct pm_thread_t {
	OS_Thread_t thread;
	OS_Queue_t queue;
};

static struct pm_thread_t *pm_thread;

static void pm_task(void *arg)
{
	OS_Status status;
	uint32_t pm_state;

	while (1) {
		status = OS_MsgQueueReceive(&pm_thread->queue, (void **)&pm_state, OS_WAIT_FOREVER);
		if (status != OS_OK) {
			PM_LOGE("%s %d\n", __func__, status);
			continue;
		}
		if (pm_state >= PM_MODE_MAX) {
			PM_LOGW("%s exit!\n", __func__);
			break;
		}

		pm_enter_mode(pm_state);
	}

	OS_ThreadDelete(&pm_thread->thread);
}
#endif

int pm_enter_mode_thread(enum suspend_state_t state)
{
#ifdef CONFIG_PM_THREAD
	if (state < PM_MODE_SLEEP)
		return 0;

	if (state >= PM_MODE_MAX) {
		PM_LOGN("%s exit!\n", __func__);
		return -1;
	}

	OS_Status ret = OS_MsgQueueSend(&pm_thread->queue, (void *)state, OS_WAIT_FOREVER);
	if (ret != OS_OK) {
		PM_LOGE("%s() ret %d\n", __func__, ret);
		return -1;
	}
#else
	PM_LOGE("not support thread mode\n");
	return -1;
#endif

	return 0;
}

void pm_start(void)
{
#ifdef CONFIG_PM_THREAD
	if (pm_thread) {
		PM_LOGE("thread start again\n");
		return ;
	}
	pm_thread = malloc(sizeof(struct pm_thread_t));
	if (!pm_thread) {
		PM_LOGE("thread malloc failed\n");
		return ;
	}

	if (OS_ThreadIsValid(&pm_thread->thread))
		return ;

	OS_MsgQueueCreate(&pm_thread->queue, 2);
	if (OS_ThreadCreate(&pm_thread->thread,
		       "net_pm",
		       pm_task,
		       NULL,
		       OS_PRIORITY_NORMAL,
		       (2 * 1024)) != 0) {
		PM_LOGE("create thread failed\n");
		return ;
	}
#endif
}

void pm_stop(void)
{
#ifdef CONFIG_PM_THREAD
	if (!pm_thread) {
		PM_LOGE("thread stop again\n");
		return ;
	}

	if (!OS_ThreadIsValid(&pm_thread->thread))
		return ;

	OS_MsgQueueSend(&pm_thread->queue, (void *)PM_MODE_MAX, OS_WAIT_FOREVER);
	while (OS_ThreadIsValid(&pm_thread->thread)) {
		OS_MSleep(1);
	};
	OS_MsgQueueDelete(&pm_thread->queue);
	free(pm_thread);
	pm_thread = NULL;
#endif
}

//#define CONFIG_PM_TEST 1

#ifdef CONFIG_PM_TEST

static int test_suspend_noirq(struct soc_device *dev, enum suspend_state_t state)
{
	PM_LOGD("%s %s okay\n", dev->name, __func__);

	//if (!strcmp(dev->name, "dev_test1"))
	//	return -1;
	//else
		return 0;
}

static int test_resume_noirq(struct soc_device *dev, enum suspend_state_t state)
{
	PM_LOGD("%s %s okay\n", dev->name, __func__);

	return 0;
}

static int test_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	PM_LOGD("%s %s okay\n", dev->name, __func__);

	//if (!strcmp(dev->name, "dev_test1"))
	//	return -1;
	//else
		return 0;
}

static int test_resume(struct soc_device *dev, enum suspend_state_t state)
{
	PM_LOGD("%s %s okay\n", dev->name, __func__);

	return 0;
}

static const struct soc_device_driver drv_test1 = {
	.name = "drv_test1",
	.suspend = &test_suspend,
	.resume = &test_resume,
};
static struct soc_device dev_test1 = {
	.name = "dev_test1",
	.driver = &drv_test1,
};

static const struct soc_device_driver drv_test2 = {
	.name = "drv_test2",
	.suspend = &test_suspend,
	.resume = &test_resume,
};
static struct soc_device dev_test2 = {
	.name = "dev_test2",
	.driver = &drv_test2,
};

static const struct soc_device_driver drv_test3 = {
	.name = "drv_test3",
	.suspend_noirq = &test_suspend_noirq,
	.resume_noirq = &test_resume_noirq,
	.suspend = &test_suspend,
	.resume = &test_resume,
};
static struct soc_device dev_test3 = {
	.name = "dev_test3",
	.driver = &drv_test3,
};

int pm_test(void)
{
	struct soc_device *dev;

	pm_register_ops(&dev_test1);
	pm_register_ops(&dev_test2);
	pm_register_ops(&dev_test2);
	pm_register_ops(&dev_test3);

	parse_dpm_list(PM_OP_NORMAL);
	parse_dpm_list(PM_OP_NOIRQ);

	return 0;
}
#endif

#endif /* CONFIG_PM */
