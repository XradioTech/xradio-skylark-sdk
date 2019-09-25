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

#include <stdio.h>
#include "compiler.h"
#include "version.h"
#include "pm/pm.h"
#include "image/image.h"

#include "common/board/board.h"
#include "sysinfo.h"
#if PRJCONF_NET_EN
#include "net_ctrl.h"
#endif
#include "fs_ctrl.h"
#include "sys_ctrl/sys_ctrl.h"
#include "fwk_debug.h"

#if PRJCONF_INTERNAL_SOUNDCARD_EN || PRJCONF_AC107_SOUNDCARD_EN
#include "audio/manager/audio_manager.h"
#include "audio/pcm/audio_pcm.h"
#if PRJCONF_AUDIO_CTRL_EN
#include "audio_ctrl.h"
#endif
#endif
#if PRJCONF_CONSOLE_EN
#include "console/console.h"
#include "command.h"
#endif
#if (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED) || (PRJCONF_NET_EN)
#include "efpg/efpg.h"
#endif
#include "driver/chip/hal_icache.h"
#include "driver/chip/hal_dcache.h"
#ifdef __CONFIG_PSRAM
#include "psram.h"
#endif

#ifdef __CONFIG_XIP
#include "driver/chip/hal_xip.h"
#endif

#ifdef __CONFIG_XPLAYER
#include "cedarx/cedarx.h"
#endif

#define PLATFORM_SHOW_DEBUG_INFO	0	/* for internal debug only */

static void platform_show_info(void)
{
	uint8_t dbg_en = 0;

#if PLATFORM_SHOW_DEBUG_INFO
	dbg_en = 1;
#endif /* PLATFORM_SHOW_INFO */

	extern uint8_t __text_start__[];
	extern uint8_t __text_end__[];
	extern uint8_t __etext[];
	extern uint8_t __data_start__[];
	extern uint8_t __data_end__[];
	extern uint8_t __bss_start__[];
	extern uint8_t __bss_end__[];
	extern uint8_t __end__[];
	extern uint8_t end[];
	extern uint8_t __HeapLimit[];
	extern uint8_t __StackLimit[];
	extern uint8_t __StackTop[];
	extern uint8_t __stack[];
	extern uint8_t _estack[];
#ifdef __CONFIG_ROM
	extern uint8_t __ram_table_lma_start__[];
	extern uint8_t __ram_table_lma_end__[];
#endif

#if PRJCONF_NET_EN
	uint8_t mac_addr[6] = {0};
	struct sysinfo *sys_info = NULL;
#endif

	FWK_LOG(1, "\nplatform information ===============================================\n");
	FWK_LOG(1, "XRADIO Skylark SDK "SDK_VERSION_STR" "SDK_STAGE_STR" "__DATE__" "__TIME__"\n\n");

	FWK_LOG(dbg_en, "__text_start__ %p\n", __text_start__);
	FWK_LOG(dbg_en, "__text_end__   %p\n", __text_end__);
	FWK_LOG(dbg_en, "__etext        %p\n", __etext);
	FWK_LOG(dbg_en, "__data_start__ %p\n", __data_start__);
	FWK_LOG(dbg_en, "__data_end__   %p\n", __data_end__);
	FWK_LOG(dbg_en, "__bss_start__  %p\n", __bss_start__);
	FWK_LOG(dbg_en, "__bss_end__    %p\n", __bss_end__);
	FWK_LOG(dbg_en, "__end__        %p\n", __end__);
	FWK_LOG(dbg_en, "end            %p\n", end);
	FWK_LOG(dbg_en, "__HeapLimit    %p\n", __HeapLimit);
	FWK_LOG(dbg_en, "__StackLimit   %p\n", __StackLimit);
	FWK_LOG(dbg_en, "__StackTop     %p\n", __StackTop);
	FWK_LOG(dbg_en, "__stack        %p\n", __stack);
	FWK_LOG(dbg_en, "_estack        %p\n", _estack);
#ifdef __CONFIG_ROM
	FWK_LOG(dbg_en, "__ram_table_lma_start__ %p\n", __ram_table_lma_start__);
	FWK_LOG(dbg_en, "__ram_table_lma_end__ %p\n", __ram_table_lma_end__);
#endif
	FWK_LOG(dbg_en, "\n");

	FWK_LOG(1, "heap space [%p, %p), size %u\n\n",
	           __end__, _estack - PRJCONF_MSP_STACK_SIZE,
	           _estack - __end__ - PRJCONF_MSP_STACK_SIZE);

	FWK_LOG(1,      "cpu  clock %9u Hz\n", HAL_GetCPUClock());
	FWK_LOG(dbg_en, "ahb1 clock %9u Hz\n", HAL_GetAHB1Clock());
	FWK_LOG(dbg_en, "ahb2 clock %9u Hz\n", HAL_GetAHB2Clock());
	FWK_LOG(dbg_en, "apb  clock %9u Hz\n", HAL_GetAPBClock());
	FWK_LOG(dbg_en, "dev  clock %9u Hz\n", HAL_GetDevClock());
#if (__CONFIG_CHIP_ARCH_VER == 2)
	FWK_LOG(dbg_en, "apbs clock %9u Hz\n", HAL_GetAPBSClock());
	FWK_LOG(dbg_en, "dev2 clock %9u Hz\n", HAL_GetDev2Clock());
#endif
	FWK_LOG(1,      "HF   clock %9u Hz\n", HAL_GetHFClock());
	FWK_LOG(dbg_en, "LF   clock %9u Hz\n", HAL_GetLFClock());
	FWK_LOG(1, "\n");

#if ((defined(__CONFIG_XIP)) || (defined(__CONFIG_PSRAM)) || (defined(__CONFIG_SECURE_BOOT)))
	FWK_LOG(1, "sdk option:\n");
#if (defined(__CONFIG_XIP))
	FWK_LOG(1, "    %-14s: enable\n", "XIP");
#endif
#if (defined(__CONFIG_PSRAM))
	FWK_LOG(1, "    %-14s: enable\n", "PSRAM");
#endif
#if (defined(__CONFIG_SECURE_BOOT))
	FWK_LOG(1, "    %-14s: enable\n", "Security Boot");
#endif
	FWK_LOG(1, "\n");
#endif

#if PRJCONF_NET_EN
	FWK_LOG(1, "mac address:\n");
	efpg_read(EFPG_FIELD_MAC, mac_addr);
	FWK_LOG(1, "    %-14s: %02x:%02x:%02x:%02x:%02x:%02x\n", "efuse",
		mac_addr[0], mac_addr[1], mac_addr[2],
		mac_addr[3], mac_addr[4], mac_addr[5]);
	sys_info = sysinfo_get();
	FWK_LOG(1, "    %-14s: %02x:%02x:%02x:%02x:%02x:%02x\n", "in use",
		sys_info->mac_addr[0], sys_info->mac_addr[1], sys_info->mac_addr[2],
		sys_info->mac_addr[3], sys_info->mac_addr[4], sys_info->mac_addr[5]);
#endif
	FWK_LOG(1, "====================================================================\n\n");
}

#ifdef __CONFIG_XIP
__nonxip_text
static void platform_xip_init(void)
{
	uint32_t addr;

	addr = image_get_section_addr(IMAGE_APP_XIP_ID);
	if (addr == IMAGE_INVALID_ADDR) {
		FWK_NX_ERR("no xip section\n");
		return;
	}

	/* TODO: check section's validity */
	HAL_Xip_Init(PRJCONF_IMG_FLASH, addr + IMAGE_HEADER_SIZE);
}
#endif /* __CONFIG_XIP */

#if PRJCONF_WDG_EN
static void platform_wdg_feed(void *arg)
{
	FWK_DBG("feed wdg @ %u sec\n", OS_GetTime());
	HAL_WDG_Feed();
}

static void platform_wdg_start(void)
{
	WDG_InitParam param;
	OS_Timer_t timer;

	/* init watchdog */
	param.hw.event = PRJCONF_WDG_EVENT_TYPE;
#if (__CONFIG_CHIP_ARCH_VER == 2)
	param.hw.resetCpuMode = PRJCONF_WDG_RESET_CPU_MODE;
#endif
	param.hw.timeout = PRJCONF_WDG_TIMEOUT;
	param.hw.resetCycle = WDG_DEFAULT_RESET_CYCLE;
	HAL_WDG_Init(&param);

	/* create OS timer to feed watchdog */
	OS_TimerSetInvalid(&timer);
	if (OS_TimerCreate(&timer, OS_TIMER_PERIODIC, platform_wdg_feed, NULL,
	                   PRJCONF_WDG_FEED_PERIOD) != OS_OK) {
		FWK_WRN("wdg timer create failed\n");
		HAL_WDG_DeInit();
		return;
	}

	HAL_WDG_Start(); /* start watchdog */
	OS_TimerStart(&timer); /* start OS timer to feed watchdog */
}
#endif /* PRJCONF_WDG_EN */

#if (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED)
#define RAND_SYS_TICK() ((SysTick->VAL & 0xffffff) | (OS_GetTicks() << 24))

static void platform_prng_init_seed(void)
{
	uint32_t seed[6];
	HAL_Status status;
	ADC_InitParam initParam;
	uint32_t chksum;

	initParam.delay = 0;
	initParam.freq = 1000000;
	initParam.mode = ADC_CONTI_CONV;
	status = HAL_ADC_Init(&initParam);
	if (status != HAL_OK) {
		FWK_WRN("adc init err %d\n", status);
	} else {
		status = HAL_ADC_Conv_Polling(ADC_CHANNEL_VBAT, &seed[0], 1000);
		if (status != HAL_OK) {
			FWK_WRN("adc conv err %d\n", status);
		}
		HAL_ADC_DeInit();
	}

	if (status != HAL_OK) {
		seed[0] = RAND_SYS_TICK();
	}

	if (image_read(IMAGE_APP_ID, IMAGE_SEG_HEADER,
	               offsetof(section_header_t, header_chksum),
	               &chksum, sizeof(chksum)) == sizeof(chksum)) {
	     seed[0] = (seed[0] << 24) ^ (seed[0] << 12) ^ (seed[0]) ^ chksum;
	}

	efpg_read(EFPG_FIELD_CHIPID, (uint8_t *)&seed[1]); /* 16-byte */

	seed[5] = RAND_SYS_TICK();

	HAL_PRNG_SetSeed(seed);
}
#endif /* (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED) */

#ifdef __CONFIG_XPLAYER
/* initial cedarx default features */
__weak void platform_cedarx_init(void)
{
	/* for media player */
	CedarxStreamListInit();
#if PRJCONF_NET_EN
	CedarxStreamRegisterHttps();
	CedarxStreamRegisterSsl();
	CedarxThreadStackSizeSet(DEMUX_THREAD, 8 * 1024);
	CedarxStreamRegisterHttp();
	CedarxStreamRegisterTcp();
#endif
	CedarxStreamRegisterFlash();
	CedarxStreamRegisterFile();
	CedarxStreamRegisterFifo();
	CedarxStreamRegisterCustomer();

	CedarxParserListInit();
	CedarxParserRegisterM3U();
	CedarxParserRegisterM4A();
	CedarxParserRegisterAAC();
	CedarxParserRegisterAMR();
	CedarxParserRegisterMP3();
	CedarxParserRegisterWAV();

	CedarxDecoderListInit();
	CedarxDecoderRegisterAAC();
	CedarxDecoderRegisterAMR();
	CedarxDecoderRegisterMP3();
	CedarxDecoderRegisterWAV();

	SoundStreamListInit();
	SoundStreamRegisterCard();
	SoundStreamRegisterReverb();

	/* for media recorder */
	CedarxWriterListInit();
	CedarxWriterRegisterFile();
	CedarxWriterRegisterCallback();
	CedarxWriterRegisterCustomer();

	CedarxMuxerListInit();
	CedarxMuxerRegisterAmr();
	CedarxMuxerRegisterPcm();

	CedarxEncoderListInit();
	CedarxEncoderRegisterAmr();
	CedarxEncoderRegisterPcm();
}
#endif

#if (__CONFIG_CHIP_ARCH_VER == 2)
#if ((__CONFIG_CACHE_POLICY & 0xF) != 0)
DCache_Config g_dcache_cfg = {
    .vc_en = 1,
    .wrap_en = 1,
    .way_mode = ((__CONFIG_CACHE_POLICY & 0xF) >> 1),
    #if (((__CONFIG_CACHE_POLICY>>4) & 0xF) != 0) /*icache & dcache both enable*/
    .mixed_mode = DCACHE_MIXED_MODE_ID,
    #elif ((__CONFIG_CACHE_POLICY & 0xF) != 0) /*dcache enable only*/
    .mixed_mode = DCACHE_MIXED_MODE_D,
    #endif
};
#endif

#if (((__CONFIG_CACHE_POLICY>>4) & 0xF) != 0)
ICache_Config g_icache_cfg = {
    .vc_en = 0,
    .wrap_en = 0,
    .way_mode = (((__CONFIG_CACHE_POLICY>>4) & 0xF) >> 1),
};
#endif
#endif

__nonxip_text
void platform_cache_init(void)
{
#if (__CONFIG_CHIP_ARCH_VER == 1)
    #ifdef __CONFIG_XIP
        ICache_Config cache_cfg = { 0 };
        uint32_t addr;
        addr = image_get_section_addr(IMAGE_APP_XIP_ID);
        if (addr == IMAGE_INVALID_ADDR) {
            FWK_NX_ERR("no xip section\n");
            return;
        }
        cache_cfg.addr_bias = addr + IMAGE_HEADER_SIZE;
        HAL_ICache_Init(&cache_cfg);
    #endif
#else
    #if (((__CONFIG_CACHE_POLICY>>4) & 0xF) != 0)
    HAL_ICache_Init(&g_icache_cfg);
    #endif
    #if ((__CONFIG_CACHE_POLICY & 0xF) != 0)
    HAL_Dcache_Init(&g_dcache_cfg);
    #endif
#endif
}

/* init basic platform hardware and services */
__nonxip_text
__weak void platform_init_level0(void)
{
	pm_start();

	HAL_Flash_Init(PRJCONF_IMG_FLASH);
	image_init(PRJCONF_IMG_FLASH, PRJCONF_IMG_ADDR, PRJCONF_IMG_MAX_SIZE);
#if (defined(__CONFIG_XIP) || defined(__CONFIG_PSRAM))
    platform_cache_init();
#endif

/*psram have to enable dcache*/
#if ((defined(__CONFIG_PSRAM)) && ((__CONFIG_CACHE_POLICY & 0xF) != 0))
	platform_psram_init();
#endif
#if (defined(__CONFIG_XIP))
	platform_xip_init();
#endif
}

/* init standard platform hardware and services */
__weak void platform_init_level1(void)
{
#if PRJCONF_CE_EN
	HAL_CE_Init();
#endif

#if PRJCONF_SYS_CTRL_EN
	sys_ctrl_create();
  #if PRJCONF_NET_EN
	net_ctrl_init();
  #endif
#endif

#if (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED)
	platform_prng_init_seed(); /* init prng seed */
#endif

	sysinfo_init();

#if PRJCONF_CONSOLE_EN
	console_param_t cparam;
	cparam.uart_id = BOARD_MAIN_UART_ID;
	cparam.cmd_exec = main_cmd_exec;
	cparam.stack_size = PRJCONF_CONSOLE_STACK_SIZE;
	console_start(&cparam);
#endif

#if PRJCONF_PM_EN
	pm_mode_platform_select(PRJCONF_PM_MODE);
#endif

#if PRJCONF_NET_EN
#ifndef __CONFIG_ETF
	net_sys_init();
#endif

	struct sysinfo *sysinfo = sysinfo_get();
	net_sys_start(sysinfo->wlan_mode);

  #if PRJCONF_NET_PM_EN
	pm_register_wlan_power_onoff(net_sys_onoff, PRJCONF_NET_PM_MODE);
  #endif
#endif /* PRJCONF_NET_EN */

#if PRJCONF_WDG_EN
	platform_wdg_start();
#endif
}

/* init extern platform hardware and services */
__weak void platform_init_level2(void)
{
#if PRJCONF_SPI_EN
	board_spi_init(BOARD_SPI_PORT);
#endif

#if PRJCONF_MMC_EN
	fs_ctrl_init();
 	board_sdcard_init(sdcard_detect_callback);
#endif

#if PRJCONF_INTERNAL_SOUNDCARD_EN || PRJCONF_AC107_SOUNDCARD_EN
	board_soundcard_init();

	audio_manager_init();
	snd_pcm_init();
  #if PRJCONF_AUDIO_CTRL_EN
	audio_ctrl_init();
  #endif
#endif

#ifdef __CONFIG_XPLAYER
	platform_cedarx_init();
#endif
}

__nonxip_text
void platform_init(void)
{
	platform_init_level0();
	platform_init_level1();
	platform_init_level2();
	platform_show_info();
}
