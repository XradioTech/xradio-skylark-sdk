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

#include "cmd_util.h"

#include "image/image.h"
#include "driver/chip/hal_prcm.h"
#include "driver/chip/hal_wdg.h"

enum cmd_status cmd_freq_offset_exec(char *cmd)
{
	enum cmd_status status;

	if (cmd_strncmp(cmd, "set_freq_offset ", 16) == 0) {
		int32_t cnt;
		uint32_t value;
		/* get param */
		cnt = cmd_sscanf(cmd + 16, "%d", &value);

		/* check param */
		if (cnt != 1) {
			CMD_ERR("invalid param number %d\n", cnt);
			return CMD_STATUS_INVALID_ARG;
		}
		if (value > 127) {
			CMD_ERR("invalid value %d\n", value);
			return CMD_STATUS_INVALID_ARG;
		}

		cmd_write_respond(CMD_STATUS_OK, "OK");
		HAL_MODIFY_REG(PRCM->DCXO_CTRL, PRCM_FREQ_OFFSET_MASK, value << PRCM_FREQ_OFFSET_SHIFT);
		CMD_LOG(1, "freq offset is set to %d!\n", value);
		status = CMD_STATUS_ACKED;
	} else if (cmd_strncmp(cmd, "get_freq_offset", 15) == 0) {
		cmd_write_respond(CMD_STATUS_OK, "OK");
		CMD_LOG(1, "freq offset is :%d\n", (PRCM->DCXO_CTRL & PRCM_FREQ_OFFSET_MASK) >> PRCM_FREQ_OFFSET_SHIFT);
		status = CMD_STATUS_ACKED;
	} else if (cmd_strncmp(cmd, "disconnect", 10) == 0) {
		cmd_write_respond(CMD_STATUS_OK, "OK");
		HAL_WDG_Reboot();
		status = CMD_STATUS_ACKED;
	} else if (cmd_strncmp(cmd, "switch_image", 12) == 0) {
		cmd_write_respond(CMD_STATUS_OK, "OK");
		const image_ota_param_t *iop = image_get_ota_param();
		CMD_LOG(1, "iop:%p,addr:0x%08X\n", iop, iop->ota_addr);
		if (iop->ota_addr == IMAGE_INVALID_ADDR) {
			/* ota is disable */
			CMD_LOG(1, "only one image, can't switch!\n");
		} else {
			image_cfg_t img_cfg;
			img_cfg.seq = (image_get_running_seq() + 1) % IMAGE_SEQ_NUM;
			img_cfg.state = IMAGE_STATE_VERIFIED;
			image_set_cfg(&img_cfg);
			CMD_LOG(1, "switch to another image, please reboot!\n");
		}
		status = CMD_STATUS_ACKED;
	} else {
		status = CMD_STATUS_UNKNOWN_CMD;
	}
	return status;
}

enum cmd_status cmd_etf_exec(char *cmd)
{
#ifdef __CONFIG_ETF_CLI
	//compare freq_offset cmd
	enum cmd_status status = cmd_freq_offset_exec(cmd);
	if (status != CMD_STATUS_UNKNOWN_CMD)
		return status;

	extern int etf_cli_main(const char **argv);

	int argc;
	char *argv[8];

	argc = cmd_parse_argv(cmd, argv, cmd_nitems(argv));
	if (argc > 0) {
		etf_cli_main((const char **)argv);
		return CMD_STATUS_ACKED;
	} else {
		HAL_WDG_Reboot();
	}
#else
	image_cfg_t img_cfg;

	cmd_write_respond(CMD_STATUS_OK, "%s", cmd);

	img_cfg.seq = (image_get_running_seq() + 1) % IMAGE_SEQ_NUM;
	img_cfg.state = IMAGE_STATE_VERIFIED;
	image_set_cfg(&img_cfg);
	HAL_WDG_Reboot();
#endif

	return CMD_STATUS_ACKED;
}
