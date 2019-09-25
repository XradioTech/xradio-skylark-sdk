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

  .equ NVIC_SCR,                          (0xe000e000 + 0xd10)
  .equ NVIC_SYSTICK_CTRL,                 (0xe000e000 + 0x010)
  .equ NVIC_ISPR,                         (0xe000e000 + 0x200)
  .equ GPRCM_CPUA_BOOT_FLAG,              (0x40040000 + 0x100)
  .equ GPRCM_CPUA_BOOT_ADDR,              (0x40040000 + 0x104)
  .equ GPRCM_CPUA_BOOT_ARG,               (0x40040000 + 0x108)
  .equ GPRCM_SYSCLK1_CTRLS,               (0x40040000 + 0x024)

#ifdef __CONFIG_ROM

  /*------------------ void __cpu_suspend(int nouse) ------------------------*/
  .thumb_func
  .section .text
  .type   __cpu_suspend, %function
  .global __cpu_suspend

__cpu_suspend:
  .fnstart
  .cantunwind

  PUSH {R0-R12, LR}
  ISB

  LDR R0, =GPRCM_CPUA_BOOT_ARG
  ISB
  LDR R1, [R0]
  MRS R0, MSP
  ISB
  STR R0, [R1]

  MRS R0, PSP
  ISB
  STR R0, [R1, #4]

  MRS R0, PRIMASK
  STR R0, [R1, #12]

  MRS R0, FAULTMASK
  STR R0, [R1, #16]

  MRS R0, BASEPRI
  STR R0, [R1, #20]

  MRS R0, CONTROL
  STR R0, [R1, #24]

  /* set deepsleep mode */
  LDR R0, =0x14
  LDR R1, =NVIC_SCR
  ISB
  STR R0, [R1]

  /* set bootflag */
  LDR R0, =0x429b0001
  LDR R1, =GPRCM_CPUA_BOOT_FLAG
  ISB
  STR R0, [R1]

  /* set resume address in thumb state */
  LDR R0, =resume
  ORR.W R0, R0, #1
  LDR R1, =GPRCM_CPUA_BOOT_ADDR
  ISB
  STR R0, [R1]

  /* switch to 24M/div */
  LDR R1, =GPRCM_SYSCLK1_CTRLS
  ISB
  LDR R0, [R1]
  BIC R0, R0, #0x30000
  ORR R0, R0, #0x10000
  STR R0, [R1]
  DSB
  ISB
  NOP
  NOP
  NOP

  /* the WFE instruction will cause two kinds of CPU actions:
   * 1. EVNET_REGISTER = 1, WFE will clear the EVENT_REGISTER and the
   *	 CPU executes the next instruction.
   * 2. EVENT_REGISTER = 0, WFE will make the CPU go to SLEEP state.
   */
  /* first time executing WFE instruction, there are some different
   * situations as follows:
   * 1. if there are interrupts pending and be cleared already,
   *	 the WFE will only clear the CPU EVENT_REGISTER.
   * 2. if there are new interrupts pending after ar400_deepsleep_lock
   *	 operation, the WFE will only clear the CPU EVENT_REGISTER.
   * 3. if the SEV/NMI/DEBUG events coming before now, WFE will only
   *	 clear the CPU EVENT_REGISTER.
   * 4. if there are no SEV/NMI/DEBUG events before and no interrupts
   *	 pending too, WFE wil make the CPU go to the SLEEP state.
   */
  WFE

  /* read the NVIC SET_PENDING_REGISTER to check whether there are
   *  any new pending interrupts after ar400_deepsleep_lock operation
   *  which make the first WFE executing failed.
   * 1. If ther are some new pending interrupts, jump to the RESUME_ENTRY
   *	 and abandon the next WFE execution.
   * 2. If there is no new pending interrupts, we execute WFE instruction
   *	 twice to ensure the CPU goes to SLEEP state successfully.
   */
  LDR R0, =NVIC_ISPR
  LDR R1, [R0, #0]
  CMP R1, #0
  BNE resume

  LDR R1, [R0, #4]
  AND R1, R1, #0x7
  CMP R1, #0
  BNE resume

  ISB
  NOP
  NOP
  NOP
  WFE
  NOP

resume:
  /* switch cpu clk to pll */
  LDR R1, =GPRCM_SYSCLK1_CTRLS
  ISB
  LDR R0, [R1]
  BIC R0, R0, #0x30000
  ORR R0, R0, #0x20000
  STR R0, [R1]
  DSB
  ISB

  /* remove bootflag */
  LDR R0, =0x429b0000
  LDR R1, =GPRCM_CPUA_BOOT_FLAG
  ISB
  STR R0, [R1]

  /* set normal mode */
  MOV R0, 0
  LDR R1, =NVIC_SCR
  ISB
  STR R0, [R1]

  /* restore cpu contex */
  LDR R0, =GPRCM_CPUA_BOOT_ARG
  ISB
  LDR R1, [R0]

  LDR R0, [R1]
  MSR MSP, R0
  ISB

  LDR R0, [R1,#4]
  MSR PSP, R0
  ISB

  LDR R0, [R1, #12]
  MSR PRIMASK, R0

  LDR R0, [R1, #16]
  MSR FAULTMASK, R0

  LDR R0, [R1, #20]
  MSR BASEPRI, R0

  LDR R0, [R1, #24]
  MSR CONTROL, R0
  ISB
  NOP

  POP {R0-R12, PC}
  NOP

  .fnend
  .size   __cpu_suspend, .-__cpu_suspend

#endif
