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

#ifdef __CONFIG_PSRAM

#include <string.h>
#include <stdio.h>

#include "sys/param.h"
#include "sys/xr_debug.h"
#include "sys/io.h"
#include "pm/pm.h"
#include "image/image.h"
#include "../hal_base.h"
#include "driver/chip/hal_dma.h"

#include "driver/chip/psram/psram.h"
#include "driver/chip/psram/hal_psramctrl.h"

#include "driver/chip/hal_icache.h"
#include "driver/chip/hal_dcache.h"
#include "_psram.h"


/**
 *	mmc_wait_for_req - start a request and wait for completion
 *	@host: MMC host to start command
 *	@mrq: MMC request to start
 *
 *	Start a new MMC custom command request for a host, and wait
 *	for the command to complete. Does not attempt to parse the
 *	response.
 */
static int32_t psram_wait_for_req(struct psram_chip *chip, struct psram_request *mrq)
{
	return HAL_PsramCtrl_Request(chip->ctrl, mrq);
}

int32_t psram_sw_reset(struct psram_chip *chip, uint32_t step)
{
	uint32_t value = 0;
	struct psram_request mrq = {{0}, {0}};
	struct psram_ctrl *ctrl = chip->ctrl;

	if (chip->type == PSRAM_CHIP_SQPI) {
		if (step == 0) {
			mrq.cmd.opcode = SQ_Reset_Enable;
		} else if (step == 1) {
			mrq.cmd.opcode = SQ_Reset;
		}
		if (chip->buswidth == 1) { /* reset enable */
			HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_DATA_GW_0BIT);
		} else if (chip->buswidth == 4) {
			HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_SBUS_CMD_SEND_4BIT | PSRAMC_SBUS_DATA_GW_0BIT);
		} else {
			PR_ERR("%s wrong buswidth:%d\n", __func__, chip->buswidth);
			return -1;
		}
		mrq.data.blksz = 1; /* byte */
		mrq.data.blocks = 0;
		mrq.data.flags = PSRAM_DATA_WRITE_SHORT;
	} else if (chip->type == PSRAM_CHIP_OPI_APS32 || chip->type == PSRAM_CHIP_OPI_APS64) {
		/* global reset */
		mrq.cmd.opcode = Global_Reaet;
		mrq.data.blksz = 1; /* byte */
		mrq.data.blocks = 2;
		value = 0;
		mrq.data.buff = (uint8_t *)&value;
		mrq.data.flags = PSRAM_DATA_WRITE_SHORT;

	}
	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	if (chip->type == PSRAM_CHIP_SQPI) {
		HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_ADDR_SEND_1BIT | PSRAMC_SBUS_DATA_GW_1BIT);
		chip->buswidth = 1;
	}
	HAL_UDelay(1000); //Min tCPH, 8 tCLK is OK !!#######

	return 0;
}

/**
 * Read psram verdor id
 */
int32_t Psram_Read_Id(struct psram_chip *chip)
{
	uint8_t p_id = 0;
	struct psram_request mrq = {{0}, {0}};

	mrq.cmd.resp = &p_id;
	mrq.data.blksz = 1; /* byte */
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_READ_BYTE;
	mrq.cmd.flags = PSRAM_ADDR_PRESENT;

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		mrq.cmd.opcode = SQ_Read_ID;
		mrq.cmd.addr = MR0;
		break;
	case PSRAM_CHIP_OPI_APS32 :
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.cmd.addr = MR1;
		break;
	case PSRAM_CHIP_OPI_APS64 :
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.cmd.addr = MR1;
		break;
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return p_id;
}

int32_t psram_get_driver_strength(struct psram_chip *chip, uint32_t drv)
{
	uint8_t mr_val;

	struct psram_request mrq = {{0}, {0}};

	mrq.cmd.resp = &mr_val;
	mrq.data.blksz = 1; /* byte */
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_READ_BYTE;
	mrq.cmd.flags = PSRAM_ADDR_PRESENT;

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		mrq.cmd.opcode = SQ_Mode_Reg_Read;
		/* PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_ADR_SEND_1BIT | PSRAMC_SBUS_DATA_GW_1BIT */
		/*(8U << 24)*/ //P_SBUS_IO_WAIT
		mrq.cmd.addr = MR0;
		break;
	case PSRAM_CHIP_OPI_APS32 :
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.cmd.addr = MR0;
		break;
	case PSRAM_CHIP_OPI_APS64 :
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.cmd.addr = MR0;
		break;
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("mr0 vale = 0x%x\n", mr_val);

	return mr_val;
}

/**
* Read psram verdor id
**/
int32_t Psram_Read_Mr(struct psram_chip *chip, uint32_t mreg)
{
	uint16_t p_id[5];
	struct psram_request mrq = {{0}, {0}};

	mrq.cmd.resp = (uint8_t *)p_id;
	mrq.data.blksz = 1; /* byte */
	mrq.cmd.flags = PSRAM_ADDR_PRESENT;

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		mrq.cmd.opcode = SQ_Read_ID;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.data.blocks = 8;
		break;
	case PSRAM_CHIP_OPI_APS32 :
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.addr = mreg;
		mrq.data.blocks = 2;
		break;
	case PSRAM_CHIP_OPI_APS64 :
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.data.flags = PSRAM_DATA_READ_SHORT;
		mrq.cmd.addr = MR1;
		mrq.data.blocks = 2;
		break;
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	if (chip->type == PSRAM_CHIP_SQPI)
		print_hex_dump_bytes((const void *)p_id, 8);
	return p_id[0];
}

int32_t Psram_Read_Info(struct psram_chip *chip)
{
	uint8_t mode_regs[8] = {0};
	struct psram_request mrq = {{0}, {0}};

	mrq.data.blksz = 1; /* byte */
	mrq.data.blocks = 2;
	mrq.data.flags = PSRAM_DATA_READ_SHORT;
	mrq.cmd.flags = PSRAM_ADDR_PRESENT;

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		mrq.cmd.opcode = SQ_Read_ID;
		break;
	case PSRAM_CHIP_OPI_APS32 :
		mrq.cmd.opcode = Mode_Reg_Read;
		break;
	case PSRAM_CHIP_OPI_APS64 :
		mrq.cmd.opcode = Mode_Reg_Read;
		break;
	default:
		break;
	}

	mrq.cmd.flags = PSRAM_ADDR_PRESENT;
	for (int i = 0; i < 4; i++) {
		mrq.cmd.resp = &mode_regs[i*2];
		mrq.cmd.addr = i*2;
		if (psram_wait_for_req(chip, &mrq) != 0)
			return -1;
	}
	//print_hex_dump_bytes((const void *)mode_regs, 8);

	return 0;
}

/**
 * read psram die state
 */
int32_t Psram_Read_die_sta(struct psram_chip *chip)
{
	uint8_t p_id;
	uint16_t p_id_raw = 0;
	struct psram_request mrq = {{0}, {0}};

	mrq.cmd.resp = (uint8_t *)&p_id_raw;
	mrq.data.blksz = 1; /* byte */

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		mrq.cmd.opcode = SQ_Read_ID;
		mrq.data.blocks = 2;
		mrq.data.flags = PSRAM_DATA_READ_SHORT;
		break;
	case PSRAM_CHIP_OPI_APS32 :
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.addr = MR2;
		break;
	case PSRAM_CHIP_OPI_APS64 :
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.addr = MR2;
		break;
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	if (chip->type == PSRAM_CHIP_SQPI) {
		if (((p_id_raw >> 8) & 0x0FF) == 0x5D)
			p_id = 1;
		else
			p_id = 0;
	} else {
		p_id = (p_id_raw >> 7) & 0x1; /* bit7: 1:PASS, 0:FAIL */
	}
	PR_DBG("psram chip die:0x%x\n", p_id_raw);

	return p_id;
}

int32_t psram_set_read_latency(struct psram_chip *chip, uint32_t fixed, uint32_t rlc)
{
	uint8_t rval;
	struct psram_request mrq = {{0}, {0}};

	mrq.cmd.resp = &rval;
	mrq.data.blksz = 1; /* byte */

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		PR_ERR("%s,%d not support now!", __func__, __LINE__);
		break;
	case PSRAM_CHIP_OPI_APS32 :
	case PSRAM_CHIP_OPI_APS64 :
		mrq.cmd.opcode = Mode_Reg_Read;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.addr = MR0;
		break;
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("%s,%d read rval:%x\n", __func__, __LINE__, rval);
	rval &= ~((1 << 5) | (0x7 << 2));
	if (chip->type == PSRAM_CHIP_OPI_APS64)
		rval |= (fixed << 5) | ((rlc - 3) << 2);
	else
		rval |= (fixed << 5) | (rlc << 2);
	PR_DBG("%s,%d set rval:%x\n", __func__, __LINE__, rval);

	mrq.data.buff = &rval;

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		PR_ERR("%s,%d not support now!", __func__, __LINE__);
		break;
	case PSRAM_CHIP_OPI_APS32 :
	case PSRAM_CHIP_OPI_APS64 :
		mrq.cmd.opcode = Mode_Reg_Write;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
		mrq.cmd.addr = MR0;
		break;
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return 0;
}

int32_t psram_set_driver_strength(struct psram_chip *chip, uint32_t drv)
{
	uint8_t mr_val = 0;
	struct psram_request mrq = {{0}, {0}};

	mrq.cmd.resp = &mr_val;
	mrq.data.blksz = 1; /* byte */

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		/* PSRAMC_SBUS_CMD_SEND_4BIT | PSRAMC_SBUS_ADR_SEND_4BIT | PSRAMC_SBUS_DATA_GW_4BIT */
		//psram_sbus_op_cmd(chip, chip->sbus_rcmd);//#########
		//rval |= (6U << 24); IO_WAIT
		if (chip->buswidth == 4) {
		} else if (chip->buswidth == 1) {
			psram_sbus_op_cmd(chip, SQ_Mode_Reg_Read);
		}
		mrq.cmd.opcode = SQ_Mode_Reg_Read;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.addr = MR0;
		break;
	case PSRAM_CHIP_OPI_APS32 :
	case PSRAM_CHIP_OPI_APS64 :
		mrq.cmd.opcode = Mode_Reg_Read;
		/* PSRAMC_SBUS_CMD_SEND_8BIT | PSRAMC_SBUS_DATA_GW_8BIT */
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.addr = MR0;
		break;
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("mr0 vale = 0x%x\n", mr_val);

	mr_val &= 0xFC;
	mr_val |= drv;

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		/* PSRAMC_SBUS_CMD_SEND_4BIT | PSRAMC_SBUS_ADR_SEND_4BIT | PSRAMC_SBUS_DATA_GW_4BIT */
		//psram_sbus_op_cmd(chip, chip->sbus_rcmd);//#########
		//rval = (0 << 24); IO_WAIT
		psram_sbus_op_cmd(chip, SQ_Mode_Reg_Write);
		mrq.cmd.opcode = SQ_Mode_Reg_Write;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
		mrq.cmd.addr = MR0;
		mrq.data.buff = &mr_val;
	case PSRAM_CHIP_OPI_APS32 :
	case PSRAM_CHIP_OPI_APS64 :
		mrq.cmd.opcode = Mode_Reg_Write;
		/* PSRAMC_SBUS_CMD_SEND_8BIT | PSRAMC_SBUS_DATA_GW_8BIT */
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
		mrq.cmd.addr = MR0;
		break;
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		/* PSRAMC_SBUS_CMD_SEND_4BIT | PSRAMC_SBUS_ADR_SEND_4BIT | PSRAMC_SBUS_DATA_GW_4BIT */
		//psram_sbus_op_cmd(chip, chip->sbus_rcmd);//#########
		//rval |= (6U << 24); IO_WAIT
		psram_sbus_op_cmd(chip, SQ_Mode_Reg_Read);
		mrq.cmd.opcode = SQ_Mode_Reg_Read;
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.addr = MR0;
		break;
	case PSRAM_CHIP_OPI_APS32 :
	case PSRAM_CHIP_OPI_APS64 :
		mrq.cmd.opcode = Mode_Reg_Read;
		/* PSRAMC_SBUS_CMD_SEND_8BIT | PSRAMC_SBUS_DATA_GW_8BIT */
		mrq.data.blocks = 1;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
		mrq.cmd.addr = MR0;
		break;
	default:
		break;
	}

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("mr0 vale = 0x%x\n", mr_val);

	return 0;
}

int32_t psram_enter_hsleep_mode(struct psram_chip *chip)
{
	uint8_t rval;
	struct psram_request mrq = {{0}, {0}};

	mrq.data.blksz = 1; /* byte */

	mrq.cmd.opcode = Mode_Reg_Write;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.cmd.addr = MR6;
	rval= 0xF0;
	mrq.data.buff = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("%s,%d success\n", __func__, __LINE__);

	return 0;
}

int32_t psram_enter_dpdown_mode(struct psram_chip *chip)
{
	uint8_t rval;
	struct psram_request mrq = {{0}, {0}};

	rval = 0xC0;
	mrq.data.blksz = 1; /* byte */

	mrq.cmd.opcode = Mode_Reg_Write;
	/* PSRAMC_SBUS_CMD_SEND_8BIT | PSRAMC_SBUS_DATA_GW_8BIT */ //?? SQPI
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.cmd.addr = MR6;
	mrq.cmd.resp = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return 0;
}

int32_t psram_set_write_latency(struct psram_chip *chip, uint32_t p_type, uint32_t wlc)
{
	uint8_t rval;
	uint8_t wlc_tables[5] = {0, 4, 2, 6, 1};
	struct psram_request mrq = {{0}, {0}};

	mrq.data.blksz = 1; /* byte */

	mrq.cmd.opcode = Mode_Reg_Read;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_READ_BYTE;
	mrq.cmd.addr = MR4;
	mrq.cmd.resp = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("%s,%d read rval:%x\n", __func__, __LINE__, rval);
	rval &= ~(0xe0);
	if (p_type == PSRAM_CHIP_OPI_APS32)
		rval |= wlc << 6;
	else if (p_type == PSRAM_CHIP_OPI_APS64)
		rval |= wlc_tables[wlc - 3] << 5;

	mrq.cmd.opcode = Mode_Reg_Write;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.data.buff = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return 0;
}

int32_t psram_set_drv(struct psram_chip *chip, uint32_t drv)
{
	uint8_t rval;
	struct psram_request mrq = {{0}, {0}};

	mrq.data.blksz = 1; /* byte */

	mrq.cmd.opcode = Mode_Reg_Read;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_READ_BYTE;
	mrq.cmd.addr = MR0;
	mrq.cmd.resp = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("%s,%d read rval:%x\n", __func__, __LINE__, rval);
	rval &= ~(0x3);
	rval |= drv;

	mrq.cmd.opcode = Mode_Reg_Write;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.data.buff = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return 0;
}

int32_t psram_set_rf(struct psram_chip *chip, uint32_t fast_en)
{
	uint8_t rval;
	struct psram_request mrq = {{0}, {0}};

	mrq.data.blksz = 1; /* byte */

	mrq.cmd.opcode = Mode_Reg_Read;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_READ_BYTE;
	mrq.cmd.addr = MR4;
	mrq.cmd.resp = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	PR_DBG("%s,%d read rval:%x\n", __func__, __LINE__, rval);
	rval &= ~(1 << 3);
	rval |= fast_en << 3;

	mrq.cmd.opcode = Mode_Reg_Write;
	mrq.data.blocks = 1;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	mrq.data.buff = &rval;

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	return 0;
}

int32_t psram_enter_quad_mode(struct psram_chip *chip)
{
	struct psram_request mrq = {{0}, {0}};
	struct psram_ctrl *ctrl = chip->ctrl;

	mrq.data.blksz = 1; /* byte */

	mrq.cmd.opcode = SQ_Enter_Quad_Mode;
	mrq.data.blocks = 0;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_DATA_GW_0BIT);

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	chip->buswidth = 4;

	return 0;
}

int32_t psram_exit_quad_mode(struct psram_chip *chip)
{
	struct psram_request mrq = {{0}, {0}};
	struct psram_ctrl *ctrl = chip->ctrl;

	mrq.data.blksz = 1; /* byte */

	mrq.cmd.opcode = SQ_Exit_Quad_Mode;
	mrq.data.blocks = 0;
	mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_SBUS_CMD_SEND_4BIT | PSRAMC_SBUS_DATA_GW_0BIT);

	if (psram_wait_for_req(chip, &mrq) != 0)
		return -1;

	HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_DATA_GW_0BIT);

	return 0;
}

int32_t psram_set_wrap_dbt(struct psram_chip *chip, uint32_t m_type)
{
	uint32_t buswidth = 0;
	struct psram_request mrq = {{0}, {0}};
	struct psram_ctrl *ctrl = chip->ctrl;

	mrq.data.blksz = 1; /* byte */

	// reset enable
	if (m_type == S_RST) {
		mrq.cmd.opcode = SQ_Wrap;
		mrq.data.blocks = 0;
		mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
		buswidth = HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_DATA_GW_0BIT);
	} else if (m_type == Q_RST) {
		mrq.cmd.opcode = SQ_Wrap;
		mrq.data.blocks = 0;
		mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
		buswidth = HAL_PsramCtrl_Set_BusWidth(ctrl, PSRAMC_SBUS_CMD_SEND_4BIT | PSRAMC_SBUS_DATA_GW_0BIT);
	}

	if (psram_wait_for_req(chip, &mrq) != 0) {
		return -1;
	}

	if ((m_type == S_RST) || (m_type == Q_RST)) {
		HAL_PsramCtrl_Set_BusWidth(ctrl, buswidth);
	}

	return -1;
}

void psram_sbus_op_cmd(struct psram_chip *chip, uint32_t opcmd)
{
	uint32_t write = 0, opcfg;
	uint32_t wait = 0, waitcfg = 0;

	switch (opcmd) {
	case S_READ :
		opcfg = SQ_Read << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_ADDR_SEND_1BIT | PSRAMC_SBUS_DUMY_SEND_1BIT |
		       PSRAMC_SBUS_DMY_DATA_WID(8) | PSRAMC_SBUS_DATA_GW_1BIT;
		break;
	case S_FAST_READ :
	case SQ_Mode_Reg_Read:
		opcfg = SQ_Fast_Read << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_ADDR_SEND_1BIT | PSRAMC_SBUS_DATA_GW_1BIT;
		wait = 1;
		waitcfg = (8U << 24);
		break;
	case SQ_Mode_Reg_Write:
		write = 1;
		opcfg = SQ_Mode_Reg_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_ADDR_SEND_1BIT | PSRAMC_SBUS_DATA_GW_1BIT;
		wait = 1;
		waitcfg = (0 << 24);
		break;
	case S_FAST_READ_QUAD :
		opcfg = SQ_Fast_Read_Quad << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_ADDR_SEND_4BIT | PSRAMC_SBUS_DATA_GW_4BIT;
		wait = 1;
		waitcfg = (6U << 24);
		break;
	case S_WRITE :
		write = 1;
		opcfg = SQ_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_ADDR_SEND_1BIT | PSRAMC_SBUS_DATA_GW_1BIT;
		break;
	case S_QAUD_WRITE :
		write = 1;
		opcfg = SQ_Quad_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_1BIT | PSRAMC_SBUS_ADDR_SEND_4BIT | PSRAMC_SBUS_DATA_GW_4BIT;
		break;
	case Q_FAST_READ :
		opcfg = SQ_Fast_Read << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_4BIT | PSRAMC_SBUS_ADDR_SEND_4BIT | PSRAMC_SBUS_DATA_GW_4BIT;
		wait = 1;
		waitcfg = (4U << 24);
		break;
	case Q_FAST_READ_QUAD :
		opcfg = SQ_Fast_Read_Quad << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_4BIT | PSRAMC_SBUS_ADDR_SEND_4BIT | PSRAMC_SBUS_DATA_GW_4BIT;
		wait = 1;
		waitcfg = (6U << 24);
		break;
	case Q_WRITE :
		write = 1;
		opcfg = SQ_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_4BIT | PSRAMC_SBUS_ADDR_SEND_4BIT | PSRAMC_SBUS_DATA_GW_4BIT;
		if (chip->type == PSRAM_CHIP_SQPI) {
			wait = 1;
			waitcfg = (6U << 24);
		}
		break;
	case O_SYNC_READ :
		opcfg = Sync_Read << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_8BIT | PSRAMC_SBUS_ADDR_SEND_8BIT | PSRAMC_SBUS_DATA_GW_8BIT;
		break;
	case O_SYNC_WRITE :
		write = 1;
		opcfg = Sync_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_8BIT | PSRAMC_SBUS_ADDR_SEND_8BIT | PSRAMC_SBUS_DATA_GW_8BIT;
		break;
	case O_SYNC_BURST_READ :
		opcfg = Sync_Burst_Read << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_8BIT | PSRAMC_SBUS_ADDR_SEND_8BIT | PSRAMC_SBUS_DATA_GW_8BIT;
		break;
	case O_SYNC_BURST_WRITE :
		write = 1;
		opcfg = Sync_Burst_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		       PSRAMC_SBUS_CMD_SEND_8BIT | PSRAMC_SBUS_ADDR_SEND_8BIT | PSRAMC_SBUS_DATA_GW_8BIT;
		break;
	default:
		PR_ERR("%s,%d unkonw cmd:%d\n", __func__, __LINE__, opcmd);
		return ;
	}
	if (write)
		chip->sbus_wcmd = opcfg >> PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT;
	else
		chip->sbus_rcmd = opcfg >> PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT;

	HAL_Psram_SbusCfg(chip->ctrl, opcfg, wait, waitcfg);
}

void psram_idbus_op_cmd(struct psram_chip *chip, uint32_t opcmd)
{
	uint32_t write = 0, opcfg;
	uint32_t wait = 0, waitcfg = 0;

	switch (opcmd) {
	case S_READ :
		opcfg = SQ_Read << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_1BIT | PSRAMC_CBUS_ADDR_1BIT | PSRAMC_CBUS_DATA_1BIT;
		break;
	case S_FAST_READ :
		opcfg = SQ_Fast_Read << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_1BIT | PSRAMC_CBUS_ADDR_1BIT | PSRAMC_CBUS_DUMY_1BIT |
		        PSRAMC_CBUS_DUMY_WID(8) | PSRAMC_CBUS_DATA_1BIT;
		break;
	case S_FAST_READ_QUAD :
		opcfg = SQ_Fast_Read_Quad << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_1BIT | PSRAMC_CBUS_ADDR_4BIT | PSRAMC_CBUS_DATA_4BIT;
		wait = 1;
		waitcfg = (6U << 24);
		break;
	case S_WRITE :
		write = 1;
		opcfg = SQ_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_1BIT | PSRAMC_CBUS_ADDR_1BIT | PSRAMC_CBUS_DATA_1BIT;
		break;
	case S_QAUD_WRITE :
		write = 1;
		opcfg = SQ_Quad_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_1BIT | PSRAMC_CBUS_ADDR_4BIT | PSRAMC_CBUS_DATA_4BIT;
		break;
	case Q_FAST_READ :
		opcfg = SQ_Fast_Read << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_4BIT | PSRAMC_CBUS_ADDR_4BIT | PSRAMC_CBUS_DATA_4BIT;
		wait = 1;
		waitcfg = (4U << 24);
		break;
	case Q_FAST_READ_QUAD :
		opcfg = SQ_Fast_Read_Quad << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_4BIT | PSRAMC_CBUS_ADDR_4BIT | PSRAMC_CBUS_DATA_4BIT;
		wait = 1;
		waitcfg = (6U << 24);
		break;
	case Q_WRITE :
		write = 1;
		opcfg = SQ_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_4BIT | PSRAMC_CBUS_ADDR_4BIT | PSRAMC_CBUS_DATA_4BIT;
		break;
	case O_SYNC_READ :
		opcfg = Sync_Read << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_8BIT | PSRAMC_CBUS_ADDR_8BIT | PSRAMC_CBUS_DATA_8BIT;
		break;
	case O_SYNC_WRITE :
		write = 1;
		opcfg = Sync_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_8BIT | PSRAMC_CBUS_ADDR_8BIT | PSRAMC_CBUS_DATA_8BIT;
		break;
	case O_SYNC_BURST_READ :
		opcfg = Sync_Burst_Read << 24 |
		        PSRAMC_CBUS_CMD_8BIT | PSRAMC_CBUS_ADDR_8BIT | PSRAMC_CBUS_DATA_8BIT;
		break;
	case O_SYNC_BURST_WRITE :
		write = 1;
		opcfg = Sync_Burst_Write << PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT |
		        PSRAMC_CBUS_CMD_8BIT | PSRAMC_CBUS_ADDR_8BIT | PSRAMC_CBUS_DATA_8BIT;
		break;
	default:
		PR_ERR("%s,%d unkonw cmd:%d\n", __func__, __LINE__, opcmd);
		return ;
	}
	if (write)
		chip->cbus_wcmd = opcfg >> PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT;
	else
		chip->cbus_rcmd = opcfg >> PSRAMC_S_RW_CFG_RW_COM_SEND_SHIFT;
	HAL_PsramCtrl_IDbusCfg(chip->ctrl, write, opcfg, wait, waitcfg);
}

static int32_t psram_sbus_transfer(struct psram_chip *chip, int write, uint32_t addr,
                                   uint8_t *buf, uint32_t len,
                                   uint32_t page_size, bool dma)
{
	int32_t ret = 0;
	uint32_t start_addr = addr, data_len = len;
	uint32_t padding = addr % page_size;
	uint32_t w_len = page_size - padding;
	struct psram_request mrq = {{0}, {0}};

	mrq.data.blksz = 1; /* byte */
	if (write) {
		mrq.cmd.opcode = chip->sbus_wcmd;
		mrq.data.flags = PSRAM_DATA_WRITE_BYTE;
	} else {
		mrq.cmd.opcode = chip->sbus_rcmd;
		mrq.data.flags = PSRAM_DATA_READ_BYTE;
	}

	if (padding && (data_len > w_len)) {
		w_len = page_size - padding;
		mrq.cmd.addr = start_addr;
		mrq.data.blocks = w_len;
		mrq.data.buff = buf;
		ret = HAL_PsramCtrl_Sbus_Transfer(chip->ctrl, &mrq, dma);
		if (ret != 0)
			return -1;
		start_addr += w_len;
		buf += w_len;
		data_len -= w_len;
	}
	while (data_len / page_size) {
		mrq.cmd.addr = start_addr;
		mrq.data.blocks = page_size;
		mrq.data.buff = buf;
		ret = HAL_PsramCtrl_Sbus_Transfer(chip->ctrl, &mrq, dma);
		if (ret != 0)
			return -1;
		start_addr += page_size;
		buf += page_size;
		data_len -= page_size;
	}
	if (data_len) {
		mrq.cmd.addr = start_addr;
		mrq.data.blocks = data_len;
		mrq.data.buff = buf;
		ret = HAL_PsramCtrl_Sbus_Transfer(chip->ctrl, &mrq, dma);
		if (ret != 0)
			return -1;
	}

	return 0;
}

int32_t psram_sbus_read(struct psram_chip *chip, uint32_t addr,
                        uint8_t *buf, uint32_t len)
{
	int32_t ret = -1;

	if (!buf || !len) {
		PR_ERR("%s,%d no data!\n", __func__, __LINE__);
		return -1;
	}

	switch (chip->type) {
	case PSRAM_CHIP_SQPI:
		if (chip->buswidth == 4)
			psram_sbus_op_cmd(chip, SQPI_BUSWIDTH4_READ);
		else
			psram_sbus_op_cmd(chip, SQPI_BUSWIDTH1_READ);
		ret = psram_sbus_transfer(chip, 0, addr, buf, len,
	                                  chip->wrap_len, 0);
		break;
	case PSRAM_CHIP_OPI_APS32:
		psram_sbus_op_cmd(chip, O_SYNC_READ);
		ret = psram_sbus_transfer(chip, 0, addr, buf, len,
	                                  chip->wrap_len, 0);
		break;
	case PSRAM_CHIP_OPI_APS64:
		psram_sbus_op_cmd(chip, O_SYNC_BURST_READ);
		ret = psram_sbus_transfer(chip, 0, addr, buf, len,
	                                  chip->wrap_len, 0);
		break;
	default:
		break;
	}

	return ret;
}

int32_t psram_sbus_write(struct psram_chip *chip, uint32_t addr, uint8_t *buf, uint32_t len)
{
	int32_t ret = -1;

	if (!buf || !len) {
		PR_ERR("%s,%d no data!\n", __func__, __LINE__);
		return -1;
	}

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		if (chip->buswidth == 4)
			psram_sbus_op_cmd(chip, SQPI_BUSWIDTH4_WRITE);
		else
			psram_sbus_op_cmd(chip, SQPI_BUSWIDTH1_WRITE);
		ret = psram_sbus_transfer(chip, 1, addr, buf, len,
	                                  chip->wrap_len, 0);
		break;
	case PSRAM_CHIP_OPI_APS32 :
		psram_sbus_op_cmd(chip, O_SYNC_WRITE);
		ret = psram_sbus_transfer(chip, 1, addr, buf, len,
		                          chip->wrap_len, 0);
		break;
	case PSRAM_CHIP_OPI_APS64 :
		psram_sbus_op_cmd(chip, O_SYNC_BURST_WRITE);
		ret = psram_sbus_transfer(chip, 1, addr, buf, len,
	                                  chip->wrap_len, 0);
		break;
	default:
		break;
	}

	return ret;
}

int32_t psram_sbus_dma_read(struct psram_chip *chip, uint32_t addr,
                            uint8_t *buf, uint32_t len)
{
	int32_t ret = -1;

	if (!buf || !len) {
		PR_ERR("%s,%d no data!\n", __func__, __LINE__);
		return -1;
	}

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		if (chip->buswidth == 4)
			psram_sbus_op_cmd(chip, SQPI_BUSWIDTH4_READ);
		else
			psram_sbus_op_cmd(chip, SQPI_BUSWIDTH1_READ);
		ret = psram_sbus_transfer(chip, 0, addr, buf, len, chip->wrap_len, 1);
		break;
	case PSRAM_CHIP_OPI_APS32 :
		psram_sbus_op_cmd(chip, O_SYNC_READ);
		ret = psram_sbus_transfer(chip, 0, addr, buf, len, chip->wrap_len, 1);
		break;
	case PSRAM_CHIP_OPI_APS64 :
		psram_sbus_op_cmd(chip, O_SYNC_BURST_READ);
		ret = psram_sbus_transfer(chip, 0, addr, buf, len, chip->wrap_len, 1);
		break;
	default:
		break;
	}

	return ret;
}

int32_t psram_sbus_dma_write(struct psram_chip *chip, uint32_t addr,
                             uint8_t *buf, uint32_t len)
{
	int32_t ret = -1;

	if (!buf || !len) {
		PR_ERR("%s,%d no data!\n", __func__, __LINE__);
		return -1;
	}

	switch (chip->type) {
	case PSRAM_CHIP_SQPI :
		if (chip->buswidth == 4)
			psram_sbus_op_cmd(chip, SQPI_BUSWIDTH4_WRITE);
		else
			psram_sbus_op_cmd(chip, SQPI_BUSWIDTH1_WRITE);
		ret = psram_sbus_transfer(chip, 1, addr, buf, len, chip->wrap_len, 1);
		break;
	case PSRAM_CHIP_OPI_APS32 :
		psram_sbus_op_cmd(chip, O_SYNC_WRITE);
		ret = psram_sbus_transfer(chip, 1, addr, buf, len, chip->wrap_len, 1);
		break;
	case PSRAM_CHIP_OPI_APS64 :
		psram_sbus_op_cmd(chip, O_SYNC_BURST_WRITE);
		ret = psram_sbus_transfer(chip, 1, addr, buf, len, chip->wrap_len, 1);
		break;
	default:
		break;
	}

	return ret;
}

static struct psram_chip *_chip_priv;

/**
 * @brief Open psram controller SBUS.
 * @note At the same time, it will disable XIP and suspend schedule.
 * @param None
 * @retval HAL_Status: The status of driver.
 */
struct psram_chip *psram_open(uint32_t id)
{
	if (_chip_priv)
		_chip_priv->ref++;

	return _chip_priv;
}

/**
 * @brief Close psram controller SBUS.
 * @param None
 * @retval HAL_Status: The status of driver.
 */
HAL_Status psram_close(struct psram_chip *chip)
{
	if (chip)
		chip->ref--;

	return HAL_OK;
}

#ifdef CONFIG_PM
static uint16_t u16PsramCheckSum = 0;
static __inline uint16_t psram_data_checksum()
{
    return image_checksum16(__PSRAM_BASE, (uint32_t)__PSRAM_LENGTH);
}

static int psram_suspend(struct soc_device *dev, enum suspend_state_t state)
{
    switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
    case PM_MODE_HIBERNATION:
        u16PsramCheckSum = psram_data_checksum();
        psram_set_write_latency(_chip_priv, PSRAM_CHIP_OPI_APS32, 0);
        HAL_PsramCtrl_Set_SBUS_WR_LATENCY(_chip_priv->ctrl, 0 << 8);
        HAL_PsramCtrl_ConfigCCMU(96000000);
        HAL_UDelay(10);
        HAL_PsramCtrl_Set_DQS_Delay_Cal(96000000);
        HAL_PsramCtrl_MaxCE_LowCyc(_chip_priv->ctrl, 96000000);
        HAL_MODIFY_REG(PSRAM_CTRL->PSRAM_TIM_CFG, PSRAMC_CS_OUTP_DHCYC_MASK, PSRAMC_CS_OUTP_DHCYC(3));
        HAL_UDelay(10);
        psram_enter_hsleep_mode(_chip_priv);
        break;
    default :
            break;
        }
    return 0;
}

static int psram_resume(struct soc_device *dev, enum suspend_state_t state)
{
    int32_t ret = 0;
    switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
    case PM_MODE_HIBERNATION:
        switch (_chip_priv->type) {
        #if (defined __CONFIG_PSRAM_CHIP_SQPI)
        case PSRAM_CHIP_SQPI:
            ret = psram_sqpi_init(_chip_priv, _chip_priv->ctrl);
            break;
        #elif (defined __CONFIG_PSRAM_CHIP_OPI32)
        case PSRAM_CHIP_OPI_APS32:
            ret = psram_aps32_init(_chip_priv, _chip_priv->ctrl);
            break;
        #elif (defined __CONFIG_PSRAM_CHIP_OPI64)
        case PSRAM_CHIP_OPI_APS64:
            ret = psram_aps64_init(_chip_priv, _chip_priv->ctrl);
            break;
        #endif
        default :
            break;
        }

        if(psram_data_checksum() != u16PsramCheckSum) {
            PR_ERR("ERROR: data checksum failed\n");
            return -1;
        }
		break;
	default:
		break;
	}
    return ret;
}

static const struct soc_device_driver psram_drv = {
	.name = "psram",
	.suspend_noirq = psram_suspend,
	.resume_noirq = psram_resume,
};

static struct soc_device psram_dev = {
	.name = "psram",
	.driver = &psram_drv,
};
#endif

int32_t psram_init(struct psram_chip *chip, struct psram_ctrl *ctrl, PSRAMChip_InitParam *param)
{
	int32_t ret = -1;

	HAL_SemaphoreInit(&chip->lock, 1, 1);

	chip->type = param->p_type;
	chip->ctrl = ctrl;
	chip->freq = param->freq;

	switch (chip->type) {
#if (defined __CONFIG_PSRAM_CHIP_SQPI)
	case PSRAM_CHIP_SQPI:
		ret = psram_sqpi_init(chip, ctrl);
		break;
#elif (defined __CONFIG_PSRAM_CHIP_OPI32)
	case PSRAM_CHIP_OPI_APS32:
		ret = psram_aps32_init(chip, ctrl);
		break;
#elif (defined __CONFIG_PSRAM_CHIP_OPI64)
	case PSRAM_CHIP_OPI_APS64:
		ret = psram_aps64_init(chip, ctrl);
		break;
#endif
	default :
		break;
	}
	_chip_priv = chip;
#ifdef CONFIG_PM
    pm_register_ops(&psram_dev);
#endif
	PR_DBG("%s @:%p ret:%d\n", __func__, chip, ret);

	return ret;
}

int32_t psram_deinit(struct psram_chip *chip)
{
	switch (chip->type) {
		case PSRAM_CHIP_SQPI:
		break;
		case PSRAM_CHIP_OPI_APS32:
		break;
		case PSRAM_CHIP_OPI_APS64:
		break;
	}
	_chip_priv = NULL;
	HAL_SemaphoreDeinit(&chip->lock);
	memset(chip, 0, sizeof(struct psram_chip));
#ifdef CONFIG_PM
    pm_unregister_ops(&psram_dev);
#endif

	return 0;
}

void psram_info_dump(struct psram_chip *chip)
{
	if (!chip)
		return ;

	PR_INF("Chip @        : %p\n", chip);
	PR_INF("    Type      : %s\n", chip->name);
	//PR_INF("    Die       : 0x%x\n", chip->die);
	PR_INF("    speed     : %u KHz\n", chip->freq / 1000);
	PR_INF("    capacity  : %dKB\n", chip->capacity/1024);
	PR_INF("    bus_width : %d\n", chip->buswidth);
	PR_INF("    wrap_len  : %d\n", chip->wrap_len);

	return ;
}

#define CONFIG_PSRAM_APS_TEST

#ifdef CONFIG_PSRAM_APS_TEST
#include <string.h>

typedef uint32_t (*c_psram_run_exec)(void *p);

volatile int p_flag = 1;

const unsigned int psram_bin[91] = { /* include text, data, bss */
	0x4903b508,0x60084603,0x47984802,0xbd0820c8,0x01400038,0x01357924,0x22014901,0xb804f000,
	0x0140003c,0x00000000,0x4802b401,0xbc014684,0xbf004760,0x002062b9,0x00000000,0x01400044,
	0x01400001,0x666e6f63,0x00006769,
};

static void test_cb(uint32_t args, void *addr)
{
	printf("%s,%d %x %p\n", __func__, __LINE__, args, addr);
}

int32_t psram_aps_run_test(void)
{
	int32_t ret = 0;
	uint32_t config_test = 0x1400001;

	HAL_PsramCtrl_Set_Address_Field(NULL, 0, IDCACHE_START_ADDR, IDCACHE_END_ADDR, 0);
	HAL_Dcache_FlushCleanAll();

	memcpy((void *)0x1400000, psram_bin, sizeof(psram_bin));
	print_hex_dump_bytes((const void *)0x1400000, sizeof(psram_bin));

	printf("%s print:\n", __func__);
	((c_psram_run_exec)config_test)(test_cb);
	while(p_flag);
	p_flag = 1;

	return ret;
}
#endif /* CONFIG_PSRAM_APS_TEST */

#endif /* __CONFIG_PSRAM */
