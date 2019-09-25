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

#ifndef _DRIVER_CHIP_CHIP_H_
#define _DRIVER_CHIP_CHIP_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Configuration of the Cortex-M4/3 Processor and Core Peripherals
 */

#ifdef __CONFIG_CPU_CM4F
#define __CM4_REV               0x0001  /*!< Core revision r0p1 */
#define __FPU_PRESENT           1       /*!< FPU present */
#else
#define __CM3_REV               0x0201  /*!< Core revision r2p1 */
#define __FPU_PRESENT           0       /*!< FPU present */
#endif

#define __MPU_PRESENT           0   /*!< MPU present */
#define __NVIC_PRIO_BITS        3   /*!< uses 3 Bits for the Priority Levels   */
#define __Vendor_SysTickConfig  0   /*!< Set to 1 if different SysTick Config is used */

/*!< Interrupt Number Definition */
typedef enum {
    /* Cortex-M4/3 Processor Exceptions Numbers*/
    NonMaskableInt_IRQn     = -14,    /*!< 2 Non Maskable Interrupt */
    MemoryManagement_IRQn   = -12,    /*!< 4 Cortex-M3 Memory Management Interrupt */
    BusFault_IRQn           = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt */
    UsageFault_IRQn         = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt */
    SVCall_IRQn             = -5,     /*!< 11 Cortex-M3 SV Call Interrupt */
    DebugMonitor_IRQn       = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt */
    PendSV_IRQn             = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt */
    SysTick_IRQn            = -1,     /*!< 15 Cortex-M3 System Tick Interrupt */

    /* Specific Interrupt Numbers */
    DMA_IRQn                = 0,
    GPIOA_IRQn              = 1,
    SDC0_IRQn               = 2,
#if (__CONFIG_CHIP_ARCH_VER == 1)
    MBOX_A_IRQn             = 3,
#endif
    UART0_IRQn              = 4,
    UART1_IRQn              = 5,
    SPI0_IRQn               = 6,
    SPI1_IRQn               = 7,
    I2C0_IRQn               = 8,
    I2C1_IRQn               = 9,
    WDG_IRQn                = 10,
    TIMER0_IRQn             = 11,
    TIMER1_IRQn             = 12,
    RTC_SEC_ALARM_IRQn      = 13,
    RTC_WDAY_ALARM_IRQn     = 14,
#if (__CONFIG_CHIP_ARCH_VER == 1)
    CSI_IRQn                = 15,
#elif (__CONFIG_CHIP_ARCH_VER == 2)
    CSI_JPEG_IRQn           = 15,
#endif
    I2S_IRQn                = 16,
    PWM_ECT_IRQn            = 17,
    CE_IRQn                 = 18,
    GPADC_IRQn              = 19,
    GPIOB_IRQn              = 20,
#if (__CONFIG_CHIP_ARCH_VER == 1)
    DMIC_IRQn               = 21,
#endif
    IRRX_IRQn               = 22,
    IRTX_IRQn               = 23,
#if (__CONFIG_CHIP_ARCH_VER == 1)
    MBOX_N_IRQn             = 24,
#endif
    A_WAKEUP_IRQn           = 25,
    FLASHC_IRQn             = 26,
#if (__CONFIG_CHIP_ARCH_VER == 1)
    N_UART_IRQn             = 27,
    SDC1_IRQn               = 28,       /* fake */
#elif (__CONFIG_CHIP_ARCH_VER == 2)
    UART2_IRQn              = 27,
    SDC1_IRQn               = 28,
    WIFIC_IRQn              = 29,
    CODEC_DAC_IRQn          = 30,
    CODEC_ADC_IRQn          = 31,
    AVS_IRQn                = 32,       /* AVS psensor */
    GPIOC_IRQn              = 33,
    PSRAMC_IRQn             = 34,
#endif
}  IRQn_Type;

#if (__CONFIG_CHIP_ARCH_VER == 1)
#define MAX_IRQn              29
#elif (__CONFIG_CHIP_ARCH_VER == 2)
#define MAX_IRQn              35
#endif

#define NVIC_PERIPH_IRQ_NUM     MAX_IRQn
#define NVIC_PERIPH_IRQ_OFFSET  16
#if (MAX_IRQn <= 32)
#define NVIC_PERIPH_IRQ_MASK0   ((1 << MAX_IRQn) - 1)
#else
#define NVIC_PERIPH_IRQ_MASK0   (0xffffffff)
#define NVIC_PERIPH_IRQ_MASK1   ((1 << (MAX_IRQn - 32)) - 1)
#endif
#define NVIC_VECTOR_TABLE_SIZE  (NVIC_PERIPH_IRQ_OFFSET + NVIC_PERIPH_IRQ_NUM)

#ifdef __CONFIG_CPU_CM4F
#include "driver/cmsis/core_cm4.h"
#else
#include "driver/cmsis/core_cm3.h"
#endif

/*!< Peripheral memory map */
#define PERIPH_BASE         (0x40000000U)
#if (__CONFIG_CHIP_ARCH_VER == 1)
#define N_PERIPH_BASE       (0xA0000000U)
#endif

#define DMA_BASE            (PERIPH_BASE + 0x00001000)
#define SDC0_BASE           (PERIPH_BASE + 0x00002000)
#if (__CONFIG_CHIP_ARCH_VER == 2)
#define SDC1_BASE           (PERIPH_BASE + 0x00003000)
#endif
#define CE_BASE             (PERIPH_BASE + 0x00004000)
#define SPI0_BASE           (PERIPH_BASE + 0x00005000)
#define SPI1_BASE           (PERIPH_BASE + 0x00006000)
#define CSI_BASE            (PERIPH_BASE + 0x00007000)
#define JPEG_BASE           (PERIPH_BASE + 0x00007400)
#if (__CONFIG_CHIP_ARCH_VER == 1)
#define SPINLOCK_BASE       (PERIPH_BASE + 0x00008000)
#define MBOX_A_BASE         (PERIPH_BASE + 0x00009000)
#endif
#define SYSCTL_BASE         (PERIPH_BASE + 0x0000A000)
#define FLASH_CTRL_BASE     (PERIPH_BASE + 0x0000B000)
#define FLASH_CACHE_BASE    (PERIPH_BASE + 0x0000C000)
#if (__CONFIG_CHIP_ARCH_VER == 2)
#define PSRAM_CTRL_BASE     (FLASH_CTRL_BASE + 0x00000800)
#define PSRAM_DCACHE_BASE   (PERIPH_BASE + 0x00009000)
#define WIFIC_BASE          (PERIPH_BASE + 0x0000D000)
#endif
#define PRCM_BASE           (PERIPH_BASE + 0x00040000)
#define CCM_BASE            (PERIPH_BASE + 0x00040400)
#define TIMER_BASE          (PERIPH_BASE + 0x00040800)
#define UART0_BASE          (PERIPH_BASE + 0x00040C00)
#define UART1_BASE          (PERIPH_BASE + 0x00041000)
#if (__CONFIG_CHIP_ARCH_VER == 2)
#define UART2_BASE          (PERIPH_BASE + 0x00041400)
#endif
#define RTC_BASE            (PERIPH_BASE + 0x00041800)
#define I2C0_BASE           (PERIPH_BASE + 0x00041c00)
#define I2C1_BASE           (PERIPH_BASE + 0x00042000)
#if (__CONFIG_CHIP_ARCH_VER == 1)
#define DMIC_BASE           (PERIPH_BASE + 0x00042400)
#endif
#define PWM_BASE            (PERIPH_BASE + 0x00042800)
#define I2S_BASE            (PERIPH_BASE + 0x00042C00)
#define GPADC_BASE          (PERIPH_BASE + 0x00043000)
#define IRTX_BASE           (PERIPH_BASE + 0x00043400)
#define IRRX_BASE           (PERIPH_BASE + 0x00043800)
#define SID_BASE            (PERIPH_BASE + 0x00043C00)
#if (__CONFIG_CHIP_ARCH_VER == 2)
#define CODEC_BASE          (PERIPH_BASE + 0x00044000)
#define TRNG_BASE           (PERIPH_BASE + 0x00044400)
#endif
#define GPIOA_CTRL_BASE     (PERIPH_BASE + 0x00050000)
#define GPIOB_CTRL_BASE     (PERIPH_BASE + 0x00050024)
#if (__CONFIG_CHIP_ARCH_VER == 2)
#define GPIOC_CTRL_BASE     (PERIPH_BASE + 0x00050048)
#endif
#define GPIOA_IRQ_BASE      (PERIPH_BASE + 0x00050200)
#define GPIOB_IRQ_BASE      (PERIPH_BASE + 0x00050220)
#if (__CONFIG_CHIP_ARCH_VER == 2)
#define GPIOC_IRQ_BASE      (PERIPH_BASE + 0x00050240)
#endif

#if (__CONFIG_CHIP_ARCH_VER == 1)
#define N_PRCM_BASE         (N_PERIPH_BASE + 0x00040000)
#define N_UART_BASE         (N_PERIPH_BASE + 0x00042000)
#define MBOX_N_BASE         (N_PERIPH_BASE + 0x00003000)
#endif

/* Value of the External oscillator in Hz */
#define HOSC_CLOCK_24M      (24U * 1000U * 1000U)
#define HOSC_CLOCK_26M      (26U * 1000U * 1000U)
#define HOSC_CLOCK_40M      (40U * 1000U * 1000U)
#define HOSC_CLOCK_48M      (48U * 1000U * 1000U)
#define HOSC_CLOCK_52M      (52U * 1000U * 1000U)
#define HOSC_CLOCK_80M      (80U * 1000U * 1000U)

#define LOSC_CLOCK          (32768U)
#define RCOSC_CLOCK         (37500U)        /* 31K ~ 63K */
#define RCOSC_CLOCK_PPM     (350)           /* ppm/deg */

#if (__CONFIG_CHIP_ARCH_VER == 1)
#define SYS_PLL_CLOCK       (960U * 1000U * 1000U)
#elif (__CONFIG_CHIP_ARCH_VER == 2)
#define SYS_PLL_CLOCK       (1920U * 1000U * 1000U)
#endif
#define SYS_LFCLOCK         LOSC_CLOCK

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_CHIP_H_ */
