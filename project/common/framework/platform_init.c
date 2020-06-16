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
#include "common/board/board_common.h"
#include "sysinfo.h"
#if PRJCONF_NET_EN
#include "net_ctrl.h"
#endif
#include "fs_ctrl.h"
#include "sys_ctrl/sys_ctrl.h"
#include "fwk_debug.h"

#if PRJCONF_AUDIO_SNDCARD_EN
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
#include "driver/chip/system_chip.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_icache.h"
#include "driver/chip/hal_dcache.h"
#ifdef __CONFIG_PSRAM
#include "psram.h"
#include "driver/chip/psram/psram.h"
#include "sys/sys_heap.h"
#include "sys/param.h"
#endif

#ifdef __CONFIG_XIP
#include "driver/chip/hal_xip.h"
#endif

#ifdef __CONFIG_XPLAYER
#include "cedarx/cedarx.h"
#endif

#if (__CONFIG_CHIP_ARCH_VER == 2)
#include "driver/chip/hal_trng.h"
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
	FWK_LOG(1, "XRADIO Skylark SDK "SDK_VERSION_STR SDK_STAGE_STR" "__DATE__" "__TIME__"\n\n");
    uint8_t vsel = HAL_GlobalGetDigLdoVsel();
    if(((vsel & 0xC) != 0) && ((vsel & 0x3) == 0)) {
        FWK_LOG(1, "WRN: wrong efuse digldo vsel 0x%x\n", vsel);
    }

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

	FWK_LOG(1, "sram heap space [%p, %p), total size %u Bytes\n",
	           __end__, _estack - PRJCONF_MSP_STACK_SIZE,
	           _estack - __end__ - PRJCONF_MSP_STACK_SIZE);
#ifdef __CONFIG_PSRAM
    FWK_LOG(1, "psram heap space [%p, %p), total size %u Bytes\n\n",
	           __psram_end__, (uint32_t*)((uint32_t)__PSRAM_BASE+(uint32_t)__PSRAM_LENGTH),
	           (uint32_t)__PSRAM_BASE + (uint32_t)__PSRAM_LENGTH - (uint32_t)__psram_end__);
#endif

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

#if (BOARD_LOSC_EXTERNAL == 1)
	FWK_LOG(1, "    %-14s: enable\n", "EXT LF OSC");
#elif (BOARD_LOSC_EXTERNAL == 0)
	FWK_LOG(1, "    %-14s: enable\n", "INT LF OSC");
#else
	FWK_LOG(1, "\n%s error(%d)\n", "BOARD_LOSC_EXTERNAL", BOARD_LOSC_EXTERNAL);
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

void platform_set_cpu_clock(PRCM_SysClkFactor factor)
{
	extern uint32_t SystemCoreClock;
	uint32_t clk = HAL_PRCM_SysClkFactor2Hz(factor);

	if (clk != SystemCoreClock) {
		HAL_PRCM_SetCPUAClk(PRCM_CPU_CLK_SRC_SYSCLK, factor);
		SystemCoreClockUpdate();
#ifdef __CONFIG_OS_FREERTOS
		extern void vPortSetupTimerInterrupt(void);
		vPortSetupTimerInterrupt();
#endif
	}
}

#if SYS_AVS_EN
#include "driver/chip/hal_util.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_psensor.h"
#define AVS_MIN_SOC_VOLT    (PRCM_LDO1_VOLT_1225MV >> PRCM_LDO1_VOLT_SHIFT)
#define AVS_MAX_SOC_VOLT    (PRCM_LDO1_VOLT_1375MV >> PRCM_LDO1_VOLT_SHIFT)

static __always_inline uint32_t cpufreq_to_psensor(PRCM_SysClkFactor freq)
{
    switch(freq) {
    case PRCM_SYS_CLK_FACTOR_384M:
        return ((28500 << 16) | (26500 << 0));
    case PRCM_SYS_CLK_FACTOR_320M:
        return ((28100 << 16) | (26100 << 0));
    case PRCM_SYS_CLK_FACTOR_274M:
        return ((27700 << 16) | (25700 << 0));
    case PRCM_SYS_CLK_FACTOR_240M:
        return ((27300 << 16) | (25300 << 0));
    default:
        return ((28500 << 16) | (25300 << 0));
    }
}

static void avs_timer_callback(void *arg)
{
    uint32_t workVolt;
    uint32_t psensor;
    uint32_t temp;
    uint16_t minPsensor;
    uint16_t maxPsensor;
    temp = cpufreq_to_psensor((uint32_t)BOARD_CPU_CLK_FACTOR);
    minPsensor = temp & 0xFFFF;
    maxPsensor = (temp >> 16) & 0xFFFF;

    psensor = HAL_Psensor_GetValue();
    workVolt = HAL_PRCM_GetLDO1WorkVolt() >> PRCM_LDO1_VOLT_SHIFT;
    FWK_DBG("AVS: minPsensor=%d, maxPsensor=%d, workVolt = %d, psensor=%d\n",
            minPsensor, maxPsensor, workVolt, psensor);
    if(psensor < minPsensor) {
        for(int i=workVolt+1; i<=AVS_MAX_SOC_VOLT; i++) {
            HAL_PRCM_SetLDO1WorkVolt( i<< PRCM_LDO1_VOLT_SHIFT);
            HAL_UDelay(10);
            psensor = HAL_Psensor_GetValue();
            if(psensor >= minPsensor) {
                platform_set_cpu_clock(BOARD_CPU_CLK_FACTOR);
                return;
            }
        }
        platform_set_cpu_clock(PRCM_SYS_CLK_FACTOR_240M);
        FWK_WRN("AVS: change cpu frequency to safe value\n");
    } else if(psensor > maxPsensor) {
        for(int i=workVolt-1; i>=AVS_MIN_SOC_VOLT; i--) {
            HAL_PRCM_SetLDO1WorkVolt( i<< PRCM_LDO1_VOLT_SHIFT);
            HAL_UDelay(10);
            psensor = HAL_Psensor_GetValue();
            if((psensor >= minPsensor) && (psensor < maxPsensor)) {
                return;
            } else if(psensor < minPsensor) {
                HAL_PRCM_SetLDO1WorkVolt((i+1) << PRCM_LDO1_VOLT_SHIFT);
                return;
            }
        }
    } else {
        platform_set_cpu_clock(BOARD_CPU_CLK_FACTOR);
    }

    return;
}

int32_t platform_avs_init(void)
{
    OS_Timer_t timer;
    avs_timer_callback(NULL);
    /* create OS timer to avs */
	OS_TimerSetInvalid(&timer);
    if (OS_TimerCreate(&timer, OS_TIMER_PERIODIC, avs_timer_callback,
                        NULL, 5000) != OS_OK) {
        FWK_WRN("AVS: timer create failed\n");
        return -1;
    }
    /* start OS timer to avs */
    OS_TimerStart(&timer);
    return 0;
}

#endif /* PRJCONF_SYS_AVS_EN */

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

#if (__CONFIG_CHIP_ARCH_VER == 1)
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
#elif (__CONFIG_CHIP_ARCH_VER == 2)
static void platform_prng_init_seed(void)
{
	uint32_t seed[6];
	HAL_Status status;
	int i;

	for (i = 0; i < 5; ++i) {
		status = HAL_TRNG_Extract(0, seed);
		if (status == HAL_OK) {
			seed[4] = seed[0] ^ seed[1];
			seed[5] = seed[2] ^ seed[3];
			FWK_DBG("prng seed %08x %08x %08x %08x %08x %08x\n",
					seed[0], seed[1], seed[2], seed[3], seed[4], seed[5]);
			break;
		} else {
			FWK_WRN("gen trng fail %d\n", status);
		}
	}

	if (status != HAL_OK) {
		ADC_InitParam initParam;
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

		seed[0] ^= RAND_SYS_TICK();
		efpg_read(EFPG_FIELD_CHIPID, (uint8_t *)&seed[1]); /* 16-byte */
		seed[5] = RAND_SYS_TICK();
	}

	HAL_PRNG_SetSeed(seed);
}
#endif /* __CONFIG_CHIP_ARCH_VER */
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
	CedarxParserRegisterTS();

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
__sram_rodata
static DCache_Config g_dcache_cfg = {
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
__sram_rodata
static ICache_Config g_icache_cfg = {
    .vc_en = 0,
    .wrap_en = 0,
    .way_mode = (((__CONFIG_CACHE_POLICY>>4) & 0xF) >> 1),
};
#endif
#endif

__sram_text
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
__sram_text
__weak void platform_init_level0(void)
{
	pm_start();
#if (defined __CONFIG_PSRAM_ALL_CACHEABLE)
    internal_dma_init();
#endif
	HAL_Flash_Init(PRJCONF_IMG_FLASH);
#if (__CONFIG_OTA_POLICY == 0x00)
	image_init(PRJCONF_IMG_FLASH, PRJCONF_IMG_ADDR, PRJCONF_IMG_MAX_SIZE);
#else
	image_init(PRJCONF_IMG_FLASH, PRJCONF_IMG_ADDR, 0);
#endif
#if (defined(__CONFIG_XIP))
    platform_xip_init();
#endif
#if ((defined(__CONFIG_PSRAM)) && ((__CONFIG_CACHE_POLICY & 0xF) != 0))
    /*psram have to enable dcache*/
    platform_psram_init();
#endif
#if (defined(__CONFIG_XIP) || defined(__CONFIG_PSRAM))
    platform_cache_init();
#endif

#if (defined(__CONFIG_PSRAM) && (!defined(__CONFIG_PSRAM_ALL_CACHEABLE)))
    HAL_Dcache_Enable_WriteThrough(rounddown2((uint32_t)__psram_bss_end__, 16), rounddown2(PSRAM_END_ADDR - DMAHEAP_PSRAM_LENGTH, 16));
#endif
}

#if PRJCONF_NET_EN
#ifdef __CONFIG_WLAN_AP
__weak const wlan_ap_default_conf_t g_wlan_ap_default_conf = {
	.ssid = "AP-XRADIO",
	.ssid_len = 9,
	.psk = "123456789",
	.hw_mode = WLAN_AP_HW_MODE_IEEE80211G,
	.ieee80211n = 0,
	.key_mgmt = WPA_KEY_MGMT_PSK,
	.wpa_pairwise_cipher = WPA_CIPHER_TKIP,
	.rsn_pairwise_cipher = WPA_CIPHER_CCMP,
	.proto = WPA_PROTO_WPA | WPA_PROTO_RSN,
	.auth_alg = WPA_AUTH_ALG_OPEN,
	.group_rekey = 3600,
	.gmk_rekey = 86400,
	.ptk_rekey = 0,
	.strict_rekey = 1,
	.channel = 1,
	.beacon_int = 100,
	.dtim = 1,
	.max_num_sta = 4,
	.country = {'C', 'N', ' '},
};
#endif
#endif

/* init standard platform hardware and services */
__weak void platform_init_level1(void)
{
#if SYS_AVS_EN
    if(HAL_PRCM_SysClkFactor2Hz(BOARD_CPU_CLK_FACTOR) > 240*1000*1000)
    {
        platform_avs_init();
    }
#endif
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
#if (__CONFIG_WPA_HEAP_MODE == 1)
	wpa_set_heap_fn(psram_malloc, psram_realloc, psram_free);
#endif
#if (__CONFIG_UMAC_HEAP_MODE == 1)
	umac_set_heap_fn(psram_malloc, psram_free);
#endif
#if (__CONFIG_LMAC_HEAP_MODE == 1)
	lmac_set_heap_fn(psram_malloc, psram_free);
#endif
#if (__CONFIG_MBUF_HEAP_MODE == 1)
	wlan_ext_request(NULL, WLAN_EXT_CMD_SET_RX_QUEUE_SIZE, 256);
#endif
#ifdef __CONFIG_WLAN_AP
	wlan_ap_set_default_conf(&g_wlan_ap_default_conf);
#endif

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

#if PRJCONF_AUDIO_SNDCARD_EN
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

__sram_text
void platform_init(void)
{
	platform_init_level0();
	platform_init_level1();
	platform_init_level2();
	platform_show_info();
}
