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

#include "rom/driver/chip/flashchip/flash_chip.h"
#include "../hal_base.h"
#include "flash_debug.h"

#define EN25QH64A_JEDEC 0x17701C

typedef enum {
	FLASH_INSTRUCTION_RDSR = 0x05,				/* read status register */
	FLASH_INSTRUCTION_WRSR = 0x01,				/* write status register */
	FLASH_INSTRUCTION_RDSR2 = 0x09,				/* read status register-2 */
	FLASH_INSTRUCTION_RDSR3 = 0x95,				/* read status register-3 */
	FLASH_INSTRUCTION_WRSR3 = 0xC0,				/* write status register-3 */
	FLASH_INSTRUCTION_SRWREN = 0x50,
} eSF_Instruction;

/* internal macros for flash chip instruction */
#define FCI_CMD(idx)    instruction[idx]
#define FCI_ADDR(idx)   instruction[idx]
#define FCI_DUMMY(idx)  instruction[idx]
#define FCI_DATA(idx)   instruction[idx]

int EN25QHXXA_ReadStatus(struct FlashChip *chip, FlashStatus reg, uint8_t *status)
{
	PCHECK(chip);
	InstructionField instruction[2];

	if (!(reg & chip->cfg.mReadStausSupport)) {
		FLASH_NOTSUPPORT();
		return -1;
	}

	HAL_Memset(&instruction, 0, sizeof(instruction));

	if (reg == FLASH_STATUS1)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_RDSR;
	}
	else if (reg == FLASH_STATUS2)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_RDSR2;
	}
	else if (reg == FLASH_STATUS3)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_RDSR3;
	}
	else
	{
		FLASH_NOWAY();
	}

	FCI_DATA(1).pdata = (uint8_t *)status;
	FCI_DATA(1).len = 1;
	FCI_DATA(1).line = 1;

	return chip->driverRead(chip, &FCI_CMD(0), NULL, NULL, &FCI_DATA(1));
}

static int EN25QHXXA_WriteStatus(struct FlashChip *chip, FlashStatus reg, uint8_t *status)
{
	int ret;

	PCHECK(chip);
	InstructionField instruction[2];

	if (!(reg & chip->cfg.mWriteStatusSupport)) {
		FLASH_NOTSUPPORT();
		return HAL_INVALID;
	}

	HAL_Memset(&instruction, 0, sizeof(instruction));

	FCI_CMD(0).data = FLASH_INSTRUCTION_SRWREN;
	FCI_CMD(0).line = 1;

	chip->driverWrite(chip, &FCI_CMD(0), NULL, NULL, NULL);

	HAL_Memset(&instruction, 0, sizeof(instruction));

	if (reg == FLASH_STATUS1)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_WRSR;

		FCI_DATA(1).pdata = (uint8_t *)status;
		FCI_DATA(1).len = 1;
		FCI_DATA(1).line = 1;
	}
	else if (reg == FLASH_STATUS3)
	{
		FCI_CMD(0).data = FLASH_INSTRUCTION_WRSR3;

		FCI_DATA(1).pdata = (uint8_t *)status;
		FCI_DATA(1).len = 1;
		FCI_DATA(1).line = 1;
	}
	else
	{
		FLASH_NOWAY();
	}

	chip->writeEnable(chip);

	ret = chip->driverWrite(chip, &FCI_CMD(0), NULL, NULL, &FCI_DATA(1));

	chip->writeDisable(chip);
/*
	while (chip->isBusy(chip)) {
		//printf("busy...\n");
	}
*/
	return ret;
}

static int EN25QHXXA_FlashInit(struct FlashChip *chip)
{
	PCHECK(chip);

	chip->writeEnable = defaultWriteEnable;
	chip->writeDisable = defaultWriteDisable;
	chip->readStatus = EN25QHXXA_ReadStatus;
	chip->erase = defaultErase;
	chip->jedecID = defaultGetJedecID;
	chip->pageProgram = defaultPageProgram;
	chip->read = defaultRead;

	chip->driverWrite = defaultDriverWrite;
	chip->driverRead = defaultDriverRead;
	chip->xipDriverCfg = defaultXipDriverCfg;
	chip->setFreq = defaultSetFreq;
	chip->switchReadMode = defaultSwitchReadMode;
	chip->enableXIP = defaultEnableXIP;
	chip->disableXIP = defaultDisableXIP;
	chip->isBusy = defaultIsBusy;
	chip->control = defaultControl;
	chip->minEraseSize = defaultGetMinEraseSize;
	//chip->writeStatus = defaultWriteStatus;
	chip->writeStatus = EN25QHXXA_WriteStatus;
	chip->enableQPIMode = defaultEnableQPIMode;
	chip->disableQPIMode = defaultDisableQPIMode;
//	chip->enableReset = defaultEnableReset;
	chip->reset = defaultReset;

	chip->suspendErasePageprogram = NULL;
	chip->resumeErasePageprogram = NULL;
	chip->powerDown = NULL;
	chip->releasePowerDown = NULL;
	chip->uniqueID = NULL;
	/*TODO: a NULL interface for showing invalid interface*/

	FLASH_DEBUG("EN25QHXXA_Flash inited");

	return 0;
}

static int EN25QHXXA_FlashDeinit(struct FlashChip *chip)
{
	PCHECK(chip);

//	HAL_Free(chip);

	return 0;
}

static struct FlashChip *EN25QHXXA_FlashCtor(struct FlashChip *chip, uint32_t arg)
{
	uint32_t jedec = arg;
	uint32_t size;
	PCHECK(chip);

	if (jedec == EN25QH64A_JEDEC) {
		size = 128 * 16 * 0x1000;
	}
	else {
		return NULL;
	}

	/* TODO: use HAL_Memcpy() and const to init chip->cfg to save code size */
	chip->cfg.mJedec = jedec;
	chip->cfg.mSize = size;
	chip->cfg.mMaxFreq = 104 * 1000 * 1000;
	chip->cfg.mMaxReadFreq = 83 * 1000 * 1000;
	chip->cfg.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP;
	chip->cfg.mPageProgramSupport = FLASH_PAGEPROGRAM | FLASH_QUAD_PAGEPROGRAM;
	chip->cfg.mReadStausSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3;
	chip->cfg.mWriteStatusSupport = FLASH_STATUS1 | FLASH_STATUS3;//
	chip->cfg.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
				| FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE;
	chip->mPageSize = 256;
	chip->mFlashStatus = 0;
	chip->mDummyCount = 1;

	return chip;
}

FlashChipCtor  EN25QH64A_FlashChip = {
		.mJedecId = EN25QH64A_JEDEC,
		.enumerate = EN25QHXXA_FlashCtor,
		.init = EN25QHXXA_FlashInit,
		.destory = EN25QHXXA_FlashDeinit,
};
