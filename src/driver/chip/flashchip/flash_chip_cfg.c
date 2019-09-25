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

static const FlashChipCfg simpleFlashChipCfg[] = {
/*
 * #ifdef FLASH_xxxxx
 *	{
 *		.mJedec = 0x ID7-ID0 ID15-ID8 M7-M0 from jedec id,
 *		.mSize = total flash memory size,
 *		.mEraseSizeSupport = the flash erase commands is supported,
 *		.mPageProgramSupport = the flash pageprogram commands is supported,
 *		.mReadStausSupport = the flash status registers can be read,
 *		.mWriteStatusSupport = the flash status registers can be written,
 *		.mReadSupport = the flash read commands (modes) is supported,
 *		.mMaxFreq = max operation frequency to flash,
 *		.mMaxReadFreq = max read command frequency(only read command: 0x03h),
 *	},
 * #endif
 */
	{
		/* default config must be at first */
		.mJedec = 0,	/* ID7-ID0 ID15-ID8 M7-M0 */
		.mSize = 128*1024*1024,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mWriteStatusSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE | FLASH_READ_QPI_MODE,
		.mMaxFreq = -1,
		.mMaxReadFreq = -1,
	},
#ifdef FLASH_PN25F16B
	{
		/* FLASH_PN25F16B */
		.mJedec = 0x15405E,
		.mSize = 32 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1,
		.mWriteStatusSupport = FLASH_STATUS1,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE,
		.mMaxFreq = 100 * 1000 * 1000,
		.mMaxReadFreq = 55 * 1000 * 1000,
	},
#endif
#ifdef FLASH_M25P64
	{
		/* FLASH_M25P64 */
		.mJedec = 0x172020,
		.mSize = 128 * 0x10000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1,
		.mWriteStatusSupport = 0,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE,
		.mMaxFreq = 50 * 1000 * 1000,
		.mMaxReadFreq = 20 * 1000 * 1000,
	},
#endif
#ifdef FLASH_W25Q16FW
	{
		/* FLASH_W25Q16FW */
		.mJedec = 0x1560EF,
		.mSize = 32 *16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mWriteStatusSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 80 * 1000 * 1000,
		.mMaxReadFreq = 50 * 1000 * 1000,
	},
#endif
#ifdef FLASH_PN25F08
	{
		/* FLASH_PN25F08 */
		.mJedec = 0x14405E,
		.mSize = 16 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1 | FLASH_STATUS2,
		.mWriteStatusSupport = FLASH_STATUS1 /* write status2 need to rewrite writeStatus */,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 108 * 1000 * 1000,
		.mMaxReadFreq = 55 * 1000 * 1000,
	},
#endif
#ifdef FLASH_PN25F16
	{
		/* FLASH_PN25F16 */
		.mJedec = 0x1540E0,
		.mSize = 32 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1 | FLASH_STATUS2,
		.mWriteStatusSupport = FLASH_STATUS1 /* write status2 need to rewrite writeStatus */,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 108 * 1000 * 1000,
		.mMaxReadFreq = 55 * 1000 * 1000,
	},
#endif
#ifdef FLASH_MX25L1636E
	{
		/* FLASH_MX25L1636E */
		.mJedec = 0x1525C2,
		.mSize = 32 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM /* QPP need to rewrite pageProgram */,
		.mReadStausSupport = FLASH_STATUS1,
		.mWriteStatusSupport = FLASH_STATUS1,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 108 * 1000 * 1000,
		.mMaxReadFreq = 50 * 1000 * 1000,
	},
#endif
#ifdef FLASH_MX25L1633E
	{
		/* FLASH_MX25L1633E */
		.mJedec = 0x1524C2,
		.mSize = 32 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM /* QPP need to rewrite pageProgram */,
		.mReadStausSupport = FLASH_STATUS1,
		.mWriteStatusSupport = FLASH_STATUS1,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 85 * 1000 * 1000,
		.mMaxReadFreq = 33 * 1000 * 1000,
	},
#endif
#ifdef FLASH_XM25QH16B
	{
		/* FLASH_XM25QH16B */
		.mJedec = 0x154020,
		.mSize = 32 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mWriteStatusSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE | FLASH_READ_QPI_MODE,
		.mMaxFreq = 104 * 1000 * 1000,
		.mMaxReadFreq = 80 * 1000 * 1000,
	},
#endif
#ifdef FLASH_XM25QH32B
	{
		/* FLASH_XM25QH32B */
		.mJedec = 0x164020,
		.mSize = 64 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mWriteStatusSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE | FLASH_READ_QPI_MODE,
		.mMaxFreq = 104 * 1000 * 1000,
		.mMaxReadFreq = 80 * 1000 * 1000,
	},
#endif
#ifdef FLASH_BY25Q64AS
	{
		/* FLASH_BY25Q64AS */
		.mJedec = 0x174068,
		.mSize = 128 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mWriteStatusSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 120 * 1000 * 1000,
		.mMaxReadFreq = 55 * 1000 * 1000,
	},
#endif
#ifdef FLASH_BY25Q32BS
	{
		/* FLASH_BY25Q32BS */
		.mJedec = 0x164068,
		.mSize = 64 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mWriteStatusSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 108 * 1000 * 1000,
		.mMaxReadFreq = 55 * 1000 * 1000,
	},
#endif
#ifdef FLASH_BY25D16
	{
		/* FLASH_BY25D16 */
		.mJedec = 0x154068,
		.mSize = 32 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1,
		.mWriteStatusSupport = FLASH_STATUS1,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE,
		.mMaxFreq = 108 * 1000 * 1000,
		.mMaxReadFreq = 55 * 1000 * 1000,
	},
#endif
#ifdef FLASH_BY25D80
	{
		/* FLASH_BY25D80 */
		.mJedec = 0x144068,
		.mSize = 16 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1,
		.mWriteStatusSupport = FLASH_STATUS1,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE,
		.mMaxFreq = 108 * 1000 * 1000,
		.mMaxReadFreq = 55 * 1000 * 1000,
	},
#endif
#ifdef FLASH_EN25Q80B
	{
		/* FLASH_EN25Q80B */
		.mJedec = 0x14301C,
		.mSize = 16 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1,
		.mWriteStatusSupport = FLASH_STATUS1,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | /*FLASH_READ_QUAD_O_MODE |*/ FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 104 * 1000 * 1000,
		.mMaxReadFreq = 50 * 1000 * 1000,
	},
#endif
#ifdef FLASH_EN25QH16A
	{
		/* FLASH_EN25QH16A */
		.mJedec = 0x15701C,
		.mSize = 32 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1,
		.mWriteStatusSupport = FLASH_STATUS1,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | /*FLASH_READ_QUAD_O_MODE |*/ FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 104 * 1000 * 1000,
		.mMaxReadFreq = 50 * 1000 * 1000,
	},
#endif
#ifdef FLASH_EN25Q32C
	{
		/* FLASH_EN25Q32C */
		.mJedec = 0x16301C,
		.mSize = 64 * 16 * 0x1000,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1,
		.mWriteStatusSupport = FLASH_STATUS1,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 104 * 1000 * 1000,
		.mMaxReadFreq = 50 * 1000 * 1000,
	},
#endif

#ifdef FLASH_GD25Q256D
	{
		/* FLASH_GD25Q256D */
		.mJedec = 0x1940C8,
		.mSize = 256 * 1024 * 1024,
		.mEraseSizeSupport = FLASH_ERASE_64KB | FLASH_ERASE_32KB | FLASH_ERASE_4KB | FLASH_ERASE_CHIP,
		.mPageProgramSupport = FLASH_PAGEPROGRAM,
		.mReadStausSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mWriteStatusSupport = FLASH_STATUS1 | FLASH_STATUS2 | FLASH_STATUS3,
		.mReadSupport = FLASH_READ_NORMAL_MODE | FLASH_READ_FAST_MODE | FLASH_READ_DUAL_O_MODE
		                | FLASH_READ_DUAL_IO_MODE | FLASH_READ_QUAD_O_MODE | FLASH_READ_QUAD_IO_MODE,
		.mMaxFreq = 104 * 1000 * 1000,
		.mMaxReadFreq = 50 * 1000 * 1000,
	},
#endif

};

const FlashChipCfg *FlashChipGetCfgList(int32_t *len)
{
	*len = HAL_ARRAY_SIZE(simpleFlashChipCfg);
	return simpleFlashChipCfg;
}
