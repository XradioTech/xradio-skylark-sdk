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

#ifndef _DRIVER_CHIP_FLASHCHIP_FLASH_CHIP_H_
#define _DRIVER_CHIP_FLASHCHIP_FLASH_CHIP_H_

#include "driver/chip/hal_def.h"
#include "driver/chip/flashchip/flash_chip_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

struct FlashChip;
struct FlashDrv;

typedef struct InstructionField
{
	uint32_t data;
	uint8_t *pdata;
	uint32_t len;
	uint32_t line;
} InstructionField;

typedef int (*FlashDrvIface)(struct FlashChip *chip, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data);

typedef enum FlashReadMode
{
	FLASH_READ_NORMAL_MODE	= 1 << 0,	/*!< cmd: 1 line, addr + dummy: 1 line, data: 1 line, freq can't too high */
	FLASH_READ_FAST_MODE	= 1 << 1,   /*!< cmd: 1 line, addr + dummy: 1 line, data: 1 line */
	FLASH_READ_DUAL_O_MODE	= 1 << 2,   /*!< cmd: 1 line, addr + dummy: 1 line, data: 2 line */
	FLASH_READ_DUAL_IO_MODE	= 1 << 3,   /*!< cmd: 1 line, addr + dummy: 2 line, data: 2 line */
	FLASH_READ_QUAD_O_MODE	= 1 << 4,   /*!< cmd: 1 line, addr + dummy: 1 line, data: 4 line */
	FLASH_READ_QUAD_IO_MODE	= 1 << 5,   /*!< cmd: 1 line, addr + dummy: 4 line, data: 4 line */
	FLASH_READ_QPI_MODE		= 1 << 6,   /*!< cmd: 4 line, addr + dummy: 4 line, data: 4 line */
} FlashReadMode;

typedef enum FlashEraseMode
{
	FLASH_ERASE_NOSUPPORT 	= 0,
	FLASH_ERASE_4KB			= 1 << 12,	/*!< 4KB  erase mode, command: 0x20 , esize: 4096 */
	FLASH_ERASE_32KB 		= 1 << 15,	/*!< 32KB erase mode, command: 0x52 , esize: 32768 */
	FLASH_ERASE_64KB 		= 1 << 16,	/*!< 64KB erase mode, command: 0xD8 , esize: 65536 */
	FLASH_ERASE_CHIP		= 1 << 1,	/*!< chip erase mode, command: 0xC7 *///0xC7,
} FlashEraseMode;

typedef enum FlashStatus
{
	FLASH_STATUS1 = 1 << 0, /*!< flash status 1 */
	FLASH_STATUS2 = 1 << 1, /*!< flash status 2 */
	FLASH_STATUS3 = 1 << 2,	/*!< flash status 3 */
} FlashStatus;

typedef enum FlashPageProgramMode
{
	FLASH_PAGEPROGRAM		= 1 << 0,   /*!< page program */
	FLASH_QUAD_PAGEPROGRAM	= 1 << 1,	/*!< quad page program */
} FlashPageProgramMode;

typedef enum FlashLockMode
{
	FlashLockMode_NOTHING	/*!< nothing to support for now */
	/*TODO: tbc...*/
} FlashLockMode;

struct FlashChip
{
	/* attribute */
	FlashChipCfg cfg;

	uint32_t mCsEn;				//no need?
	uint32_t mPageSize;			//256bytes

	uint16_t mFlashStatus;
	uint8_t  mDummyCount;
	uint8_t  mLockSupport;

	struct FlashDrv *mDriver;
	struct XipDrv *mXip;

	/* private interface */
	void (*writeEnable)(struct FlashChip *chip);
	void (*writeDisable)(struct FlashChip *chip);
	int (*readStatus)(struct FlashChip *chip, FlashStatus status, uint8_t *data);
	int (*writeStatus)(struct FlashChip *chip, FlashStatus status, uint8_t *data);
	int (*erase)(struct FlashChip *chip, FlashEraseMode mode, uint32_t addr);	//block count is realized in Flash erase.
	int (*suspendErasePageprogram)(struct FlashChip *chip);
	int (*resumeErasePageprogram)(struct FlashChip *chip);
	int (*powerDown)(struct FlashChip *chip);
	int (*releasePowerDown)(struct FlashChip *chip);
	int (*jedecID)(struct FlashChip *chip, uint32_t *data);
	int (*enableQPIMode)(struct FlashChip *chip);
	int (*disableQPIMode)(struct FlashChip *chip);
//	int (*enableReset)(struct FlashChip *chip);
	int (*reset)(struct FlashChip *chip);
	int (*uniqueID)(struct FlashChip *chip, uint8_t uid[8]);
	int (*pageProgram)(struct FlashChip *chip, FlashPageProgramMode mode, uint32_t addr, const uint8_t *data, uint32_t size);
	int (*read)(struct FlashChip *chip, FlashReadMode mode, uint32_t addr, uint8_t *data, uint32_t size);
	// tbc...

	int (*driverWrite)(struct FlashChip *chip, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data);
	int (*driverRead)(struct FlashChip *chip, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data);
	int (*xipDriverCfg)(struct FlashChip *chip, FlashReadMode mode);

	/*
		public interface
	*/
	int (*setFreq)(struct FlashChip *chip, uint32_t freq);
	int (*switchReadMode)(struct FlashChip *chip, FlashReadMode mode);
	int (*enableXIP)(struct FlashChip *chip);
	int (*disableXIP)(struct FlashChip *chip);
	int (*isBusy)(struct FlashChip *chip);
	int (*control)(struct FlashChip *chip, int op, void *param);
	FlashEraseMode (*minEraseSize)(struct FlashChip *chip);
	//void *platform_data; /* for special flash */
	struct flash_controller *flash_ctrl;
};

typedef enum DefaultFlashControlCmds
{
	DEFAULT_FLASH_SET_QPI_READ_P5P4,
	DEFAULT_FLASH_POWERDOWN,
} DefaultFlashControlCmds;

/*
	Default Flash Chip Interface
*/
void defaultWriteEnable(struct FlashChip *chip);

void defaultWriteDisable(struct FlashChip *chip);

int defaultReadStatus(struct FlashChip *chip, FlashStatus status, uint8_t *data);

int defaultWriteStatus(struct FlashChip *chip, FlashStatus reg, uint8_t *status);

int defaultErase(struct FlashChip *chip, FlashEraseMode mode, uint32_t addr);

int defaultSuspendErasePageprogram(struct FlashChip *chip);

int defaultResumeErasePageprogram(struct FlashChip *chip);

int defaultPowerDown(struct FlashChip *chip);

int defaultReleasePowerDown(struct FlashChip *chip);

int defaultGetJedecID(struct FlashChip *chip, uint32_t *data);

int defaultEnableQPIMode(struct FlashChip *chip);

int defaultDisableQPIMode(struct FlashChip *chip);

int defaultEnableReset(struct FlashChip *chip);

int defaultReset(struct FlashChip *chip);

int defaultGetUniqueID(struct FlashChip *, uint8_t uid[8]);

int defaultPageProgram(struct FlashChip *chip, FlashPageProgramMode mode, uint32_t addr, const uint8_t *data, uint32_t size);

int defaultRead(struct FlashChip *chip, FlashReadMode mode, uint32_t addr, uint8_t *data, uint32_t size);

int defaultDriverWrite(struct FlashChip *chip, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data);

int defaultDriverRead(struct FlashChip *chip, InstructionField *cmd, InstructionField *addr, InstructionField *dummy, InstructionField *data);

int defaultSetFreq(struct FlashChip *chip, uint32_t freq);

int defaultSwitchReadMode(struct FlashChip *chip, FlashReadMode mode);

int defaultEnableXIP(struct FlashChip *chip);

int defaultDisableXIP(struct FlashChip *chip);

int defaultIsBusy(struct FlashChip *chip);

int defaultControl(struct FlashChip *chip, int op, void *param);

FlashEraseMode defaultGetMinEraseSize(struct FlashChip *chip);

int defaultXipDriverCfg(struct FlashChip *chip, FlashReadMode mode);


typedef struct FlashChipCtor
{
	uint32_t mJedecId;

	struct FlashChip *(*enumerate)(struct FlashChip *chip, uint32_t arg);
	int (*init)(struct FlashChip *);
	int (*destory)(struct FlashChip *);
} FlashChipCtor;

int FlashChipEnum(struct FlashChip *chip, struct FlashDrv *drv);
int FlashChipDestroy(struct FlashChip *chip);

const FlashChipCtor **FlashChipGetChipList(int *size);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_FLASHCHIP_FLASH_CHIP_H_ */
