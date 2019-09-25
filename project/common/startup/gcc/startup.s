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

  .syntax unified
#ifdef __CONFIG_CPU_CM4F
  .cpu cortex-m4
  .fpu softvfp
#else
  .cpu cortex-m3
#endif
  .thumb

#ifdef __CONFIG_ROM
  .equ RAM_TABLE_VMA,                   (0x00200000 + 0xc00)
  .global memcpy
#endif

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

#ifdef __CONFIG_ROM
/* start address for the .ram_table section. defined in linker script */
.word __ram_table_lma_start__
/* end address for the .ram_table section. defined in linker script */
.word __ram_table_lma_end__
#endif

/*.equ  BootRAM, 0xF108F85F*/
/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function

Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */
#ifdef __CONFIG_ROM
  ldr   r0, =RAM_TABLE_VMA
  ldr   r1, =__ram_table_lma_start__  /* must be align 4 */
  ldr   r2, =__ram_table_lma_end__
  sub   r2, r2, r1
  cmp   r2, #0
  bne   ram_table_cpy
  b     startup

ram_table_cpy:
  cmp   r1, r0
  beq   startup
  ldr r3, =memcpy
  blx   r3
startup:
#endif
  bl   _start

LoopForever:
  b LoopForever
.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
  .extern exception_entry

  .section .cpu_text,"ax",%progbits
  .thumb_func
  .type Default_Handler, %function

Default_Handler:
#ifndef __CONFIG_BOOTLOADER
  CPSID F
  TST LR, #0x04
  ITE EQ
  MRSEQ R0, MSP
  MRSNE R0, PSP
  STMDB.W R0!, {R4-R11}
  MRS R1, MSP
  MRS R2, PSP
  BL exception_entry
#endif
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

/******************************************************************************
*
* The minimal vector table for a Cortex M3.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word Default_Handler // NMI_Handler              // -14
  .word Default_Handler // HardFault_Handler
  .word Default_Handler // MemManage_Handler
  .word Default_Handler // BusFault_Handler
  .word Default_Handler // UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler                                 // -5
  .word Default_Handler // DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler                             // -1

  /* External Interrupts */
  .word Default_Handler // DMA_IRQHandler           // 0
  .word Default_Handler // GPIOA_IRQHandler
  .word Default_Handler // SDC0_IRQHandler
  .word Default_Handler // MBOX_A_IRQHandler or no use
  .word Default_Handler // UART0_IRQHandler
  .word Default_Handler // UART1_IRQHandler
  .word Default_Handler // SPI0_IRQHandler
  .word Default_Handler // SPI1_IRQHandler
  .word Default_Handler // I2C0_IRQHandler
  .word Default_Handler // I2C1_IRQHandler
  .word Default_Handler // WDG_IRQHandler           // 10
  .word Default_Handler // TIMER0_IRQHandler
  .word Default_Handler // TIMER1_IRQHandler
  .word Default_Handler // RTC_SecAlarm_IRQHandler
  .word Default_Handler // RTC_WDayAlarm_IRQHandler
  .word Default_Handler // CSI_IRQHandler
  .word Default_Handler // I2S_IRQHandler
  .word Default_Handler // PWM_ECT_IRQHandler
  .word Default_Handler // CE_IRQHandler
  .word Default_Handler // GPADC_IRQHandler
  .word Default_Handler // GPIOB_IRQHandler         // 20
  .word Default_Handler // DMIC_IRQHandler or no use
  .word Default_Handler // IRRX_IRQHandler
  .word Default_Handler // IRTX_IRQHandler
  .word Default_Handler // MBOX_N_IRQHandler or no use
  .word Default_Handler // A_WAKEUP_IRQHandler
  .word Default_Handler // FLASHC_IRQHandler
  .word Default_Handler // N_UART_IRQHandler or UART2_IRQHandler        // 27
  .word Default_Handler // SDC1_IRQHandler
  .word Default_Handler // WIFIC_IRQHandler
  .word Default_Handler // CODEC_DAC_IRQHandler                         // 30
  .word Default_Handler // CODEC_ADC_IRQHandler
  .word Default_Handler // AVS_IRQHandler
  .word Default_Handler // GPIOC_IRQn
  .word Default_Handler // PSRAMC_IRQn              // 34
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0                                           // 40
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0                                           // 47

