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

#include "common/cmd/cmd_util.h"
#include "common/cmd/cmd.h"

/*
 * driver commands
 */
static const struct cmd_data g_drv_cmds[] = {
    { "uart",   cmd_uart_exec },
    { "clock",  cmd_clock_exec },
    { "timer",  cmd_timer_exec },
    { "wdg",    cmd_wdg_exec },
    { "rtc",    cmd_rtc_exec },
    { "irrx",   cmd_irrx_exec },
    { "irtx",   cmd_irtx_exec },
    { "crypto", cmd_crypto_exec },
    { "i2c",    cmd_i2c_exec },
    { "adc",    cmd_adc_exec },
    { "flash",  cmd_flash_exec },
    { "pwm",    cmd_pwm_exec },
    { "sd",     cmd_sd_exec },
    { "audio",  cmd_audio_exec },
    { "gpio",   cmd_gpio_exec },
    { "oled",	cmd_oled_exec },
    { "i2s", 	cmd_i2s_exec},
    { "codec",	cmd_codec_exec},
};

static enum cmd_status cmd_drv_exec(char *cmd)
{
	return cmd_exec(cmd, g_drv_cmds, cmd_nitems(g_drv_cmds));
}

/*
 * main commands
 */
static const struct cmd_data g_main_cmds[] = {
	{ "drv",	cmd_drv_exec },
	{ "fs",     cmd_fs_exec },
	{ "echo",	cmd_echo_exec },
	{ "mem",	cmd_mem_exec },
	{ "upgrade",    cmd_upgrade_exec },
	{ "reboot",     cmd_reboot_exec },
	{ "pm",		cmd_pm_exec },
	{ "sysinfo",cmd_sysinfo_exec },
    { "psensor",cmd_psensor_exec},
	{ "uart",   cmd_uart_exec },
    { "clock",  cmd_clock_exec },
    { "timer",  cmd_timer_exec },
    { "wdg",    cmd_wdg_exec },
    { "rtc",    cmd_rtc_exec },
    { "irrx",   cmd_irrx_exec },
    { "irtx",   cmd_irtx_exec },
    { "crypto", cmd_crypto_exec },
    { "i2c",    cmd_i2c_exec },
    { "adc",    cmd_adc_exec },
    { "flash",  cmd_flash_exec },
    { "pwm",    cmd_pwm_exec },
    { "sd",     cmd_sd_exec },
    { "audio",  cmd_audio_exec },
    { "gpio",   cmd_gpio_exec },
    { "spi",    cmd_spi_exec },
    { "heap",   cmd_heap_exec },
    { "i2s", 	cmd_i2s_exec},
    { "codec",	cmd_codec_exec},
    { "efpg",	cmd_efpg_exec},
#ifdef __CONFIG_PSRAM
    { "psram",	cmd_psram_exec},
#endif
};

void main_cmd_exec(char *cmd)
{
	cmd_main_exec(cmd, g_main_cmds, cmd_nitems(g_main_cmds));
}
