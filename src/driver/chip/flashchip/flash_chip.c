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

#include "driver/chip/flashchip/flash_chip.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_xip.h"

typedef enum {
	FLASH_INSTRUCTION_WREN = 0x06,				/* write enable */
	FLASH_INSTRUCTION_WRDI = 0x04,				/* write disable */
	FLASH_INSTRUCTION_RDID = 0x9F,				/* jedec id */
	FLASH_INSTRUCTION_RDSR1 = 0x05, 			/* read status register-1 */
	FLASH_INSTRUCTION_WRSR1 = 0x01, 			/* write status register-1 */
	FLASH_INSTRUCTION_READ = 0x03,				/* read data */
	FLASH_INSTRUCTION_FAST_READ = 0x0B, 		/* fast read */
	FLASH_INSTRUCTION_PP = 0x02,					/* page program */
	FLASH_INSTRUCTION_ERASE_64KB = 0xD8,			/* erase block(sector) 64k */
	FLASH_INSTRUCTION_ERASE_32KB = 0x52,			/* erase block(sector) 32k */
	FLASH_INSTRUCTION_ERASE_4KB = 0x20, 		/* erase sector 4k */
	FLASH_INSTRUCTION_ERASE_CHIP = 0xC7,			/* chip erase */
	FLASH_INSTRUCTION_WRSR = 0X01,				/* write status register */
	FLASH_INSTRUCTION_FAST_READ_DO = 0x3B,		/* fast read dual output */
	FLASH_INSTRUCTION_RDSR2 = 0x35,
	FLASH_INSTRUCTION_RDSR3 = 0x15,
	FLASH_INSTRUCTION_WRSR2 = 0x31,
	FLASH_INSTRUCTION_WRSR3 = 0x11,
	FLASH_INSTRUCTION_SRWREN = 0x50,
	FLASH_INSTRUCTION_CE = 0x60,
	FLASH_INSTRUCTION_EPSP = 0x75,
	FLASH_INSTRUCTION_EPRS = 0x7A,
	FLASH_INSTRUCTION_PWDN = 0xB9,
	FLASH_INSTRUCTION_REL = 0xAB,
	FLASH_INSTRUCTION_FAST_READ_DIO = 0xBB,
	FLASH_INSTRUCTION_FAST_READ_QO = 0x6B,
	FLASH_INSTRUCTION_FAST_READ_QIO = 0xEB,
	FLASH_INSTRUCTION_EN_QPI = 0x38,
	FLASH_INSTRUCTION_DIS_QPI = 0xFF,
	FLASH_INSTRUCTION_RSEN = 0x66,
	FLASH_INSTRUCTION_RESET = 0x99,
	FLASH_INSTRUCTION_QPP = 0x32,
	FLASH_INSTRUCTION_SRP = 0xC0,
} eSF_Instruction;

#ifdef FLASH_DEFAULTCHIP
extern FlashChipCtor DefaultFlashChip;
#endif
#ifdef FLASH_XT25F16B
extern FlashChipCtor  XT25F16B_FlashChip;
#endif
#ifdef FLASH_XT25F32B
extern FlashChipCtor  XT25F32B_FlashChip;
#endif
#ifdef FLASH_XT25F64B
extern FlashChipCtor  XT25F64B_FlashChip;
#endif
#ifdef FLASH_P25Q80H
extern FlashChipCtor  P25Q80H_FlashChip;
#endif
#ifdef FLASH_P25Q40H
extern FlashChipCtor  P25Q40H_FlashChip;
#endif
#ifdef FLASH_P25Q16H
extern FlashChipCtor  P25Q16H_FlashChip;
#endif
#ifdef FLASH_EN25QH64A
extern FlashChipCtor  EN25QH64A_FlashChip;
#endif
#ifdef FLASH_XM25QH64A
extern FlashChipCtor  XM25QH64A_FlashChip;
#endif

FlashChipCtor *flashChipList[] = {
#ifdef FLASH_DEFAULTCHIP
	&DefaultFlashChip, /*default chip must be at the first*/
#endif
#ifdef FLASH_XT25F16B
	&XT25F16B_FlashChip,
#endif
#ifdef FLASH_XT25F32B
	&XT25F32B_FlashChip,
#endif
#ifdef FLASH_XT25F64B
	&XT25F64B_FlashChip,
#endif
#ifdef FLASH_P25Q80H
	&P25Q80H_FlashChip,
#endif
#ifdef FLASH_P25Q40H
	&P25Q40H_FlashChip,
#endif
#ifdef FLASH_P25Q16H
	&P25Q16H_FlashChip,
#endif
#ifdef FLASH_EN25QH64A
	&EN25QH64A_FlashChip,
#endif
#ifdef FLASH_XM25QH64A
	&XM25QH64A_FlashChip,
#endif
};

const FlashChipCtor **FlashChipGetChipList(int *len)
{
	*len = HAL_ARRAY_SIZE(flashChipList);
	return (const FlashChipCtor **)flashChipList;
}
