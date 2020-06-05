#if (defined(__CONFIG_ROM))
INCLUDE rom_symbol.ld
#endif

/* Linker script to configure memory regions. */
MEMORY
{
#if (defined(__CONFIG_CHIP_XR872))
#if (defined(__CONFIG_JPEG))
#if (defined(__CONFIG_JPEG_SHARE_64K))
    RAM (rwx)   : ORIGIN = 0x00201000, LENGTH = 348K
#else
    RAM (rwx)   : ORIGIN = 0x00201000, LENGTH = 380K
#endif
#else
    RAM (rwx)   : ORIGIN = 0x00201000, LENGTH = 412K
#endif
#elif (defined(__CONFIG_CHIP_XR808))
    RAM (rwx)   : ORIGIN = 0x00201000, LENGTH = 252K
#endif

#if (defined(__CONFIG_XIP))
    FLASH (rx)  : ORIGIN = 0x00400000, LENGTH = 16M
#else
    FLASH (rx)  : ORIGIN = 0x00400000, LENGTH = 0K
#endif

#if (defined(__CONFIG_PSRAM))
#if (defined(__CONFIG_PSRAM_CHIP_SQPI))
    PSRAM (rwx) : ORIGIN = 0x01400000, LENGTH = 2048K
#elif (defined(__CONFIG_PSRAM_CHIP_OPI32))
    PSRAM (rwx) : ORIGIN = 0x01400000, LENGTH = 4096K
#elif (defined(__CONFIG_PSRAM_CHIP_OPI64))
    PSRAM (rwx) : ORIGIN = 0x01400000, LENGTH = 8192K
#endif
#endif
}

__RAM_BASE = ORIGIN(RAM);
#if (defined(__CONFIG_XIP))
__XIP_BASE = ORIGIN(FLASH);
__XIP_LENGTH = LENGTH(FLASH);
__XIP_END = ORIGIN(FLASH) + LENGTH(FLASH) -1;
#endif
#if (defined(__CONFIG_PSRAM))
__PSRAM_BASE = ORIGIN(PSRAM);
__PSRAM_LENGTH = LENGTH(PSRAM);
__PSRAM_END = ORIGIN(PSRAM) + LENGTH(PSRAM) -1;
__DMAHEAP_PSRAM_BASE = __PSRAM_BASE + __PSRAM_LENGTH - __CONFIG_DMAHEAP_PSRAM_SIZE*1024;
__DMAHEAP_PSRAM_LENGTH = __CONFIG_DMAHEAP_PSRAM_SIZE*1024;
__DMAHEAP_PSRAM_END = __DMAHEAP_PSRAM_BASE + __DMAHEAP_PSRAM_LENGTH -1;
#endif

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 *   _estack
 */
ENTRY(Reset_Handler)

SECTIONS
{
#if (defined(__CONFIG_XIP))
    .xip :
    {
        . = ALIGN(4);
        __xip_start__ = .;

        /* MUST not put IRQ handler/callback in .xip section */
        *libmp3.a: (.text .text.* .rodata .rodata.*)
        *libaac.a: (.text .text.* .rodata .rodata.*)
        *libwav.a: (.text .text.* .rodata .rodata.*)
        *libamr.a: (.text .text.* .rodata .rodata.*)
        *libamren.a: (.text .text.* .rodata .rodata.*)
        *libreverb.a: (.text .text.* .rodata .rodata.*)
        *libopus.a: (.text .text.* .rodata .rodata.*)
        *libspeex.a: (.text .text.* .rodata .rodata.*)
        *libcedarx.a: (.text .text.* .rodata .rodata.*)
        *libpcm.a: (.text .text.* .rodata .rodata.*)
        *libaudmgr.a: (.text .text.* .rodata .rodata.*)
        *liblwip.a: (.text .text.* .rodata .rodata.*)
        *libmbedtls.a: (.text .text.* .rodata .rodata.*)
        *libhttpcli.a: (.text .text.* .rodata .rodata.*)
        *libhttpd.a: (.text .text.* .rodata .rodata.*)
        *libmqtt.a: (.text .text.* .rodata .rodata.*)
        *libnopoll.a: (.text .text.* .rodata .rodata.*)
        *liblibwebsockets.a: (.text .text.* .rodata .rodata.*)
        *libping.a: (.text .text.* .rodata .rodata.*)
        *libsntp.a: (.text .text.* .rodata .rodata.*)
        *libudhcpd.a: (.text .text.* .rodata .rodata.*)
        *libaliyun.a: (.text .text.* .rodata .rodata.*)
        *libwlan.a: (.text .text.* .rodata .rodata.*)
        *libwpa.a: (.text .text.* .rodata .rodata.*)
        *libwpas.a: (.text .text.* .rodata .rodata.*)
        *libwpas_wps.a: (.text .text.* .rodata .rodata.*)
        *libhostapd.a: (.text .text.* .rodata .rodata.*)
        *libnet80211.a: (.text .text.* .rodata .rodata.*)
        *libxrwireless.a: (.text .text.* .rodata .rodata.*)
        *libethernetif.a: (.text .text.* .rodata .rodata.*)
        *libxretf.a: (.text .text.* .rodata .rodata.*)
        *libxretf_gui.a: (.text .text.* .rodata .rodata.*)
        *libxrsys.a: (.text .text.* .rodata .rodata.*)
        *libsc_assistant.a: (.text .text.* .rodata .rodata.*)
        *libsmartlink.a: (.text .text.* .rodata .rodata.*)
        *libairkiss_aes.a: (.text .text.* .rodata .rodata.*)
        *libadt.a: (.text .text.* .rodata .rodata.*)
        *libcjson.a: (.text .text.* .rodata .rodata.*)
        *libfs.a: (.text .text.* .rodata .rodata.*)
        *libatcmd.a: (.text .text.* .rodata .rodata.*)
        *libefpg.a: (.text .text.* .rodata .rodata.*)
        *libota.a: (.text .text.* .rodata .rodata.*)
        *libxz.a: (.text .text.* .rodata .rodata.*)
        *libconsole.a: (.text .text.* .rodata .rodata.*)
        *libutil.a: (.text .text.* .rodata .rodata.*)
        *libzbar.a: (.text .text.* .rodata .rodata.*)

        *libchip.a:hal_wdg.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_timer.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_rtc.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_crypto.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_sdhost.o (.text .text.* .rodata .rodata.*)
        *libchip.a:sd.o (.text .text.* .rodata .rodata.*)
        *libchip.a:core.o (.text .text.* .rodata .rodata.*)
        *libchip.a:test.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_i2s.o (.text .text.* .rodata .rodata.*)
        *libchip.a:ac107.o (.text .text.* .rodata .rodata.*)
        *libchip.a:xradio_internal_codec.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_snd_card.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_i2c.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_dmic.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_pwm.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_spi.o (.text .text.* .rodata .rodata.*)
        *libchip.a:hal_efuse.o (.text .text.* .rodata .rodata.*)

        *libm.a: (EXCLUDE_FILE (lib_a-s_fpclassify.o lib_a-sf_fpclassify.o) .text
                  EXCLUDE_FILE (lib_a-s_fpclassify.o lib_a-sf_fpclassify.o) .text.*
                  EXCLUDE_FILE (lib_a-s_fpclassify.o lib_a-sf_fpclassify.o) .rodata
                  EXCLUDE_FILE (lib_a-s_fpclassify.o lib_a-sf_fpclassify.o) .rodata.*)

        *libc_nano.a:lib_a-asctime.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-asctime_r.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-ctime.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-gmtime.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-gmtime_r.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-lcltime.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-lcltime_r.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-mktime.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strftime.o (.text .text.* .rodata .rodata.*)

        *libc_nano.a:lib_a-gettzinfo.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-month_lengths.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-timelocal.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-tzcalc_limits.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-tzlock.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-tzset.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-tzset_r.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-tzvars.o (.text .text.* .rodata .rodata.*)

        *libc_nano.a:lib_a-envlock.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-getenv.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-getenv_r.o (.text .text.* .rodata .rodata.*)

        *libc_nano.a:lib_a-rand.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-getopt.o (.text .text.* .rodata .rodata.*)


        /*
         * Before putting the following objs of libc_nano.a to xip,
         * make sure they are not called in IRQ handler/callback.
         */
/*
        *libc_nano.a:lib_a-atoi.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-atol.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-atoll.o (.text .text.* .rodata .rodata.*)

        *libc_nano.a:lib_a-strcasecmp.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strcmp.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strncasecmp.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strncmp.o (.text .text.* .rodata .rodata.*)

        *libc_nano.a:lib_a-strcpy.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strlcpy.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strncpy.o (.text .text.* .rodata .rodata.*)

        *libc_nano.a:lib_a-strcat.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strchr.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strcspn.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strdup.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strdup_r.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strerror.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-u_strerr.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strrchr.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strspn.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strstr.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strtod.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-gdtoa-gethex.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-gdtoa-hexnan.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strtok.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strtok_r.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strtol.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strtoll.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strtoll_r.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-strtoul.o (.text .text.* .rodata .rodata.*)

        *libc_nano.a:lib_a-sscanf.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-stdio.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-nano-svfscanf.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-nano-vfscanf_i.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-nano-vfscanf_float.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-sf_nan.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-sccl.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-ungetc.o (.text .text.* .rodata .rodata.*)
*/

        /*
         * Before putting the following objs of libc_nano.a to xip,
         * make sure float printf is not used in IRQ handler/callback.
         */
/*
        *libc_nano.a:lib_a-nano-vfprintf_float.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-dtoa.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-locale.o (.text .text.* .rodata .rodata.*)
        *libc_nano.a:lib_a-mprec.o (.text .text.* .rodata .rodata.*)
        *libm.a:lib_a-s_fpclassify.o (.text .text.* .rodata .rodata.*)
        *libm.a:lib_a-sf_fpclassify.o (.text .text.* .rodata .rodata.*)
*/

        *project/common/board/board.o (.text .text.* .rodata .rodata.*)
        *project/common/cmd*.o (.text .text.* .rodata .rodata.*)
        *project/common/iperf*.o (.text .text.* .rodata .rodata.*)
        *project/common/framework*.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/player_app.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/recorder_app.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/os_glue/cdx_memory.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/os_glue/atomic.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/os_glue/file.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/os_glue/pthread.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/os_glue/sleep.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/capture_ctrl/captureControl_rtos.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/sound_ctrl/card_pcm.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/sound_ctrl/reverb_buffer.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/sound_ctrl/reverb_pcm.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/sound_ctrl/soundControl_rtos.o (.text .text.* .rodata .rodata.*)
        *project/common/apps/cedarx/sound_ctrl/soundStreamControl.o (.text .text.* .rodata .rodata.*)
        *command.o (.text .text.* .rodata .rodata.*)

        *(.xip_text* .xip_rodata*)

        . = ALIGN(4);
        __xip_end__ = .;
    } > FLASH
#endif /* __CONFIG_XIP */

#if (defined(__CONFIG_PSRAM))
    .psram_text :
    {
        . = ALIGN(4);
        __psram_start__ = .;
        __psram_text_start__ = .;

        *project/common/cmd/cmd_psram.o (.text* .rodata*)
        *(.psram_text* .psram_rodata*)

        . = ALIGN(4);
        __psram_text_end__ = .;
    } > PSRAM

    .psram_data :
    {
        . = ALIGN(4);
        __psram_data_start__ = .;
        *libmp3.a: ( .data .data.* vtable )
        *libaac.a: ( .data .data.* vtable )
        *libwav.a: ( .data .data.* vtable )
        *libamr.a: ( .data .data.* vtable )
        *libamren.a: ( .data .data.* vtable )
        *libreverb.a: ( .data .data.* vtable )
        *libopus.a: ( .data .data.* vtable )
        *libcedarx.a: ( .data .data.* vtable )
        *libpcm.a: ( .data .data.* vtable )
        *libaudmgr.a: ( .data .data.* vtable )
        *liblwip.a: ( .data .data.* vtable )
        *libmbedtls.a: ( .data .data.* vtable )
        *libhttpcli.a: ( .data .data.* vtable )
        *libhttpd.a: ( .data .data.* vtable )
        *libmqtt.a: ( .data .data.* vtable )
        *libnopoll.a: ( .data .data.* vtable )
        *liblibwebsockets.a: ( .data .data.* vtable )
        *libping.a: ( .data .data.* vtable )
        *libsntp.a: ( .data .data.* vtable )
        *libudhcpd.a: ( .data .data.* vtable )
        *libaliyun.a: ( .data .data.* vtable )
        *libwlan.a: ( .data .data.* vtable )
        *libwpa.a: ( .data .data.* vtable )
        *libwpas.a: ( .data .data.* vtable )
        *libwpas_wps.a: ( .data .data.* vtable )
        *libhostapd.a: ( .data .data.* vtable )
        *libnet80211.a: ( .data .data.* vtable )
        *libxrwireless.a: ( .data .data.* vtable )
        *libethernetif.a: ( .data .data.* vtable )
        *libxretf.a: ( .data .data.* vtable )
        *libxretf_gui.a: ( .data .data.* vtable )
        *libxrsys.a: ( .data .data.* vtable )
        *libsc_assistant.a: ( .data .data.* vtable )
        *libsmartlink.a: ( .data .data.* vtable )
        *libairkiss_aes.a: ( .data .data.* vtable )
        *libadt.a: ( .data .data.* vtable )
        *libcjson.a: ( .data .data.* vtable )
        *libfs.a: ( .data .data.* vtable )
        *libatcmd.a: ( .data .data.* vtable )
        *libefpg.a: ( .data .data.* vtable )
        *libota.a: ( .data .data.* vtable )
        *libxz.a: ( .data .data.* vtable )
        *libutil.a: ( .data .data.* vtable )
        *libchip.a:hal_wdg.o ( .data .data.* vtable )
        *libchip.a:hal_timer.o ( .data .data.* vtable )
        *libchip.a:hal_rtc.o ( .data .data.* vtable )
        *libchip.a:hal_crypto.o ( .data .data.* vtable )
        *libchip.a:hal_sdhost.o ( .data .data.* vtable )
        *libchip.a:sd.o ( .data .data.* vtable )
        *libchip.a:core.o ( .data .data.* vtable )
        *libchip.a:test.o ( .data .data.* vtable )
        *libchip.a:hal_i2s.o ( .data .data.* vtable )
        *libchip.a:ac107.o ( .data .data.* vtable )
        *libchip.a:xradio_internal_codec.o ( .data .data.* vtable )
        *libchip.a:hal_snd_card.o ( .data .data.* vtable )
        *libchip.a:hal_i2c.o ( .data .data.* vtable )
        *libchip.a:hal_dmic.o ( .data .data.* vtable )
        *libchip.a:hal_pwm.o ( .data .data.* vtable )
        *libchip.a:hal_spi.o ( .data .data.* vtable )
        *libchip.a:hal_efuse.o ( .data .data.* vtable )
        *libm.a: ( .data .data.* vtable )
        *libc_nano.a:lib_a-asctime.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-asctime_r.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-ctime.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-gmtime.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-gmtime_r.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-lcltime.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-lcltime_r.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-mktime.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-strftime.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-gettzinfo.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-month_lengths.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-timelocal.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-tzcalc_limits.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-tzlock.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-tzset.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-tzset_r.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-tzvars.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-envlock.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-getenv.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-getenv_r.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-rand.o ( .data .data.* vtable )
        *libc_nano.a:lib_a-getopt.o ( .data .data.* vtable )

        *project/common/cmd*.o ( .data .data.* vtable )
        *project/common/iperf*.o ( .data .data.* vtable )
        *project/common/apps/player_app.o ( .data .data.* vtable )
        *project/common/apps/recorder_app.o ( .data .data.* vtable )
        *project/common/apps/cedarx/os_glue/cdx_memory.o ( .data .data.* vtable )
        *project/common/apps/cedarx/os_glue/atomic.o ( .data .data.* vtable )
        *project/common/apps/cedarx/os_glue/file.o ( .data .data.* vtable )
        *project/common/apps/cedarx/os_glue/pthread.o ( .data .data.* vtable )
        *project/common/apps/cedarx/os_glue/sleep.o ( .data .data.* vtable )
        *project/common/apps/cedarx/capture_ctrl/captureControl_rtos.o ( .data .data.* vtable )
        *project/common/apps/cedarx/sound_ctrl/card_pcm.o ( .data .data.* vtable )
        *project/common/apps/cedarx/sound_ctrl/reverb_buffer.o ( .data .data.* vtable )
        *project/common/apps/cedarx/sound_ctrl/reverb_pcm.o ( .data .data.* vtable )
        *project/common/apps/cedarx/sound_ctrl/soundControl_rtos.o ( .data .data.* vtable )
        *project/common/apps/cedarx/sound_ctrl/soundStreamControl.o ( .data .data.* vtable )
        *command.o ( .data .data.* vtable )
        *(.psram_data*)

        . = ALIGN(4);
        __psram_data_end__ = .;
    } > PSRAM

    .psram_bss :
    {
        . = ALIGN(4);
        __psram_bss_start__ = .;
        *libmp3.a: ( .bss .bss.* COMMON )
        *libaac.a: ( .bss .bss.* COMMON )
        *libwav.a: ( .bss .bss.* COMMON )
        *libamr.a: ( .bss .bss.* COMMON )
        *libamren.a: ( .bss .bss.* COMMON )
        *libreverb.a: ( .bss .bss.* COMMON )
        *libopus.a: ( .bss .bss.* COMMON )
        *libcedarx.a: ( .bss .bss.* COMMON )
        *libpcm.a: ( .bss .bss.* COMMON )
        *libaudmgr.a: ( .bss .bss.* COMMON )
        *liblwip.a: ( .bss .bss.* COMMON )
        *libmbedtls.a: ( .bss .bss.* COMMON )
        *libhttpcli.a: ( .bss .bss.* COMMON )
        *libhttpd.a: ( .bss .bss.* COMMON )
        *libmqtt.a: ( .bss .bss.* COMMON )
        *libnopoll.a: ( .bss .bss.* COMMON )
        *liblibwebsockets.a: ( .bss .bss.* COMMON )
        *libping.a: ( .bss .bss.* COMMON )
        *libsntp.a: ( .bss .bss.* COMMON )
        *libudhcpd.a: ( .bss .bss.* COMMON )
        *libaliyun.a: ( .bss .bss.* COMMON )
        *libwlan.a: ( .bss .bss.* COMMON )
        *libwpa.a: ( .bss .bss.* COMMON )
        *libwpas.a: ( .bss .bss.* COMMON )
        *libwpas_wps.a: ( .bss .bss.* COMMON )
        *libhostapd.a: ( .bss .bss.* COMMON )
        *libnet80211.a: ( .bss .bss.* COMMON )
        *libxrwireless.a: ( .bss .bss.* COMMON )
        *libethernetif.a: ( .bss .bss.* COMMON )
        *libxretf.a: ( .bss .bss.* COMMON )
        *libxretf_gui.a: ( .bss .bss.* COMMON )
        *libxrsys.a: ( .bss .bss.* COMMON )
        *libsc_assistant.a: ( .bss .bss.* COMMON )
        *libsmartlink.a: ( .bss .bss.* COMMON )
        *libairkiss_aes.a: ( .bss .bss.* COMMON )
        *libadt.a: ( .bss .bss.* COMMON )
        *libcjson.a: ( .bss .bss.* COMMON )
        *libfs.a: ( .bss .bss.* COMMON )
        *libatcmd.a: ( .bss .bss.* COMMON )
        *libefpg.a: ( .bss .bss.* COMMON )
        *libota.a: ( .bss .bss.* COMMON )
        *libxz.a: ( .bss .bss.* COMMON )
        *libutil.a: ( .bss .bss.* COMMON )
        *libchip.a:hal_wdg.o ( .bss .bss.* COMMON )
        *libchip.a:hal_timer.o ( .bss .bss.* COMMON )
        *libchip.a:hal_rtc.o ( .bss .bss.* COMMON )
        *libchip.a:hal_crypto.o ( .bss .bss.* COMMON )
        *libchip.a:hal_sdhost.o ( .bss .bss.* COMMON )
        *libchip.a:sd.o ( .bss .bss.* COMMON )
        *libchip.a:core.o ( .bss .bss.* COMMON )
        *libchip.a:test.o ( .bss .bss.* COMMON )
        *libchip.a:hal_i2s.o ( .bss .bss.* COMMON )
        *libchip.a:ac107.o ( .bss .bss.* COMMON )
        *libchip.a:xradio_internal_codec.o ( .bss .bss.* COMMON )
        *libchip.a:hal_snd_card.o ( .bss .bss.* COMMON )
        *libchip.a:hal_i2c.o ( .bss .bss.* COMMON )
        *libchip.a:hal_dmic.o ( .bss .bss.* COMMON )
        *libchip.a:hal_pwm.o ( .bss .bss.* COMMON )
        *libchip.a:hal_spi.o ( .bss .bss.* COMMON )
        *libchip.a:hal_efuse.o ( .bss .bss.* COMMON )
        *libm.a: ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-asctime.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-asctime_r.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-ctime.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-gmtime.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-gmtime_r.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-lcltime.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-lcltime_r.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-mktime.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-strftime.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-gettzinfo.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-month_lengths.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-timelocal.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-tzcalc_limits.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-tzlock.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-tzset.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-tzset_r.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-tzvars.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-envlock.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-getenv.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-getenv_r.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-rand.o ( .bss .bss.* COMMON )
        *libc_nano.a:lib_a-getopt.o ( .bss .bss.* COMMON )

        *project/common/cmd*.o ( .bss .bss.* COMMON )
        *project/common/iperf*.o ( .bss .bss.* COMMON )
        *project/common/apps/player_app.o ( .bss .bss.* COMMON )
        *project/common/apps/recorder_app.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/os_glue/cdx_memory.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/os_glue/atomic.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/os_glue/file.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/os_glue/pthread.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/os_glue/sleep.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/capture_ctrl/captureControl_rtos.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/sound_ctrl/card_pcm.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/sound_ctrl/reverb_buffer.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/sound_ctrl/reverb_pcm.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/sound_ctrl/soundControl_rtos.o ( .bss .bss.* COMMON )
        *project/common/apps/cedarx/sound_ctrl/soundStreamControl.o ( .bss .bss.* COMMON )
        *command.o ( .bss .bss.* COMMON )
        *(.psram_bss*)

        . = ALIGN(16);
        __psram_bss_end__ = .;
        __psram_end__ = .;
    } > PSRAM
#endif /* __CONFIG_PSRAM */

    .text :
    {
        . = ALIGN(4);
        __text_start__ = .;
        __VECTOR_BASE = .;
        KEEP(*(.isr_vector))
        KEEP(*(.text.Reset_Handler))
        KEEP(*(.cpu_text))
        *(.text*)
        *(.nonxip_text*)
        *(.sram_text*)
        KEEP(*(.init))
        KEEP(*(.fini))

        /* .ctors */
        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        /* .dtors */
        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.rodata*)
        *(.nonxip_rodata*)
        *(.sram_rodata*)

        KEEP(*(.eh_frame*))

        . = ALIGN(4);
        __text_end__ = .;
    } > RAM

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > RAM

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > RAM
    __exidx_end = .;

    . = ALIGN(4);
    __etext = .;
    _sidata = .;

    .data :
    {
        . = ALIGN(4);
        __data_start__ = .;
        _sdata = .;

        *(vtable)
        *(.data*)
        *(.nonxip_data*)
        *(.sram_data*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))

        . = ALIGN(4);
#if (defined(__CONFIG_ROM))
        *(*.got*)
#endif
        __data_end__ = .;
        _edata = .;
    } > RAM

    .bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        _sbss = .;

        *(.bss*)
        *(COMMON)
        *(.nonxip_bss*)
        *(.sram_bss*)

        . = ALIGN(4);
        __bss_end__ = .;
        _ebss = .;
    } > RAM

    .heap (COPY):
    {
        __end__ = .;
        end = __end__;
        *(.heap*)
        __HeapLimit = .;
    } > RAM

    /* .stack_dummy section doesn't contains any symbols. It is only
     * used for linker to calculate size of stack sections, and assign
     * values to stack symbols later */
    .stack_dummy (COPY):
    {
        *(.stack*)
    } > RAM

#if (defined(__CONFIG_ROM))
    .ram_table : AT(__bss_start__)
    {
        . = ALIGN(4);
        KEEP(*(.ram_table))
        . = ALIGN(4);
    } > RAM
    __ram_table_lma_start__ = __bss_start__;
    __ram_table_lma_end__ = __ram_table_lma_start__ + SIZEOF(.ram_table);
#endif

#if (__CONFIG_CACHE_POLICY == 0x00)
    REMAIN_CACHE_SIZE = 40K;
#elif ((__CONFIG_CACHE_POLICY == 0x01) || (__CONFIG_CACHE_POLICY == 0x10))
    REMAIN_CACHE_SIZE = 32K;
#elif ((__CONFIG_CACHE_POLICY == 0x11) || (__CONFIG_CACHE_POLICY == 0x20) || (__CONFIG_CACHE_POLICY == 0x02))
    REMAIN_CACHE_SIZE = 24K;
#elif ((__CONFIG_CACHE_POLICY == 0x12) || (__CONFIG_CACHE_POLICY == 0x21))
    REMAIN_CACHE_SIZE = 16K;
#elif ((__CONFIG_CACHE_POLICY == 0x22) || (__CONFIG_CACHE_POLICY == 0x40) || (__CONFIG_CACHE_POLICY == 0x04))
    REMAIN_CACHE_SIZE = 8K;
#elif ((__CONFIG_CACHE_POLICY == 0x41) || (__CONFIG_CACHE_POLICY == 0x14))
    REMAIN_CACHE_SIZE = 0;
#else
    REMAIN_CACHE_SIZE = 0;
#endif

    /* Set stack top to end of RAM, and stack limit move down by
     * size of stack_dummy section */
	__StackTop = ORIGIN(RAM) + LENGTH(RAM);
#if ((!(defined(__CONFIG_JPEG))) && (defined(__CONFIG_CHIP_XR872)))
    __StackTop += REMAIN_CACHE_SIZE;
#endif
    _estack = __StackTop;
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "ld: region RAM overflowed with stack")
#if (defined(__CONFIG_PSRAM))
	/* Check if psramdata exceeds PSRAM limit */
    ASSERT(__DMAHEAP_PSRAM_BASE > __psram_end__, "ld: region dmapsramheap overflowed with psram heap")
#endif
}
