/**
  * @file  psram.h
  * @author  XRADIO IOT WLAN Team
  */

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

#ifndef _PSRAM_H
#define _PSRAM_H
#include "driver/chip/hal_def.h"
#include "kernel/os/FreeRTOS/os_semaphore.h"

//---------PSRAM SPI/QPI Command set-------
#define SQ_Read				0x03
#define SQ_Fast_Read			0x0B    /* 66MHz, wait 4 cycle every read */
#define SQ_Fast_Read_Quad		0xEB    /* 144/84MHz, wait 6 cycle every read */
#define SQ_Write			0x02
#define SQ_Quad_Write			0x38
#define SQ_Mode_Reg_Read		0xB5U
#define SQ_Mode_Reg_Write		0xB1U
#define SQ_Wrapped_Read			0x8B
#define SQ_Wrapped_Write		0x82
#define SQ_ModeResister_Read            0xB5
#define SQ_ModeResister_Write           0xB1
#define SQ_Enter_Quad_Mode		0x35
#define SQ_Exit_Quad_Mode		0xF5
#define SQ_Reset_Enable			0x66
#define SQ_Reset			0x99
#define SQ_Wrap				0xC0
#define SQ_Read_ID			0x9F

//--------PSRAM OPI Command set-----------
#define Sync_Read                0x00
#define Sync_Write               0x80
#define Sync_Burst_Read          0x20
#define Sync_Burst_Write         0xA0
#define Mode_Reg_Read            0x40
#define Mode_Reg_Write           0xC0
#define Global_Reaet             0xFF


//--------OPI Mode Register Address---------
#define MR0			0x00
#define MR1			0x01
#define MR2			0x02
#define MR3			0x03
#define MR4			0x04
#define MR5			0x05
#define MR6			0x06
#define MR7			0x07

//--------------define by myself------------------
#define S_READ				0x00
#define S_FAST_READ			0x01
#define S_FAST_READ_QUAD		0x02
#define S_WRITE				0x03
#define S_QAUD_WRITE			0x04
#define Q_FAST_READ			0x05
#define Q_FAST_READ_QUAD		0x06
#define Q_WRITE				0x07
#define O_SYNC_READ			0x08
#define O_SYNC_WRITE			0x09
#define O_SYNC_BURST_READ		0x0A
#define O_SYNC_BURST_WRITE		0x0B


//--------SQPI MODE Reset------------------
#define S_RST					0
#define Q_RST					1

//---------BUS define----------------------
#define S_BUS					0
#define ID_BUS					1

//--------------------------------------------------

#define P_DMA_B1W8			0
#define P_DMA_B1W16			1
#define P_DMA_B1W32			2
#define P_DMA_B4W8			3
#define P_DMA_B4W16			4
#define P_DMA_B4W32			5

#define FLUSH_LEN_HSIZE  32
#define FLUSH_LEN_CSIZE_128 128
#define FLUSH_LEN_CSIZE_256 256
#define FLUSH_LEN_CSIZE_896	896

#define FLUSH_MODE	FLUSH_LEN_CSIZE_128

#define DCACHE_OPEN

//--------------------------------------------------
/*  chip information  */
#define PSRAM_CHIP_SQPI         0
#define PSRAM_CHIP_OPI_APS32    1
#define PSRAM_CHIP_OPI_APS64    2
#define PSRAM_CHIP_MAX          3

/**
 * @brief PSRAM initialization parameters
 */
typedef struct {
	uint32_t p_type;
	uint32_t freq;			/*!< PSRAM Chip working frequency */
} PSRAMChip_InitParam;

struct psram_chip {
	uint8_t                 id;
	uint8_t                 type;
	uint8_t                 ref;
	uint8_t                 suspend;

	uint8_t                 sbus_rcmd;
	uint8_t                 sbus_wcmd;
	uint8_t                 cbus_rcmd;
	uint8_t                 cbus_wcmd;

	uint32_t                buswidth;
	uint32_t                wrap_len;
	uint32_t                capacity;
	uint32_t                freq;
	OS_Semaphore_t          lock;
	char                    *name;
	struct psram_ctrl       *ctrl;
};

#define PSRAM_DATA_WRITE_BYTE   (1 << 0)
#define PSRAM_DATA_WRITE_SHORT  (1 << 1)
#define PSRAM_DATA_WRITE_WORD   (1 << 2)
#define PSRAM_DATA_WRITE_MASK   (0x07 << 0)
#define PSRAM_DATA_READ_BYTE    (1 << 4)
#define PSRAM_DATA_READ_SHORT   (1 << 5)
#define PSRAM_DATA_READ_WORD    (1 << 6)
#define PSRAM_DATA_READ_MASK    (0x07 << 4)

struct psram_data {
	uint32_t blksz; 	/* data block size */
	uint32_t blocks;	/* number of blocks */
	uint32_t flags;
	uint32_t rw_cfg;
	uint8_t *buff;

	//uint32_t                bytes_xfered;
	//uint32_t                sg_len;         /* size of scatter list */
	//struct scatterlist      *sg;            /* I/O scatter list */
};

//#define MMC_RSP_MASK            (0x1f << 0)
#define PSRAM_ADDR_PRESENT        (1 << 0)
//#define MMC_RSP_136             (1 << 1)        /* 136 bit response */
//#define MMC_RSP_CRC             (1 << 2)        /* expect valid crc */
//#define MMC_RSP_BUSY            (1 << 3)        /* card may send busy */
//#define MMC_RSP_OPCODE          (1 << 4)        /* response contains opcode */

struct psram_command {
	uint32_t opcode;
	uint32_t addr;
	uint8_t *resp;
	uint32_t rw_cfg;
	uint32_t flags;                         /* expected response type */

//#define MMC_CMD_MASK            (3 << 5)        /* non-SPI command type */
//#define MMC_CMD_AC              (0 << 5)        /* addressed comamnd without data transfer */
//#define MMC_CMD_ADTC            (1 << 5)        /* addressed command with data transfer */
//#define MMC_CMD_BC              (2 << 5)        /* broadcast command without response */
//#define MMC_CMD_BCR             (3 << 5)        /* broadcast command with response */

//#define MMC_RSP_SPI_S1          (1 << 7)        /* one status byte */
//#define MMC_RSP_SPI_S2          (1 << 8)        /* second byte */
//#define MMC_RSP_SPI_B4          (1 << 9)        /* four data bytes */
//#define MMC_RSP_SPI_BUSY        (1 << 10)       /* card may send busy */

/* These are the native response types, and correspond to valid bit
 * patterns of the above flags.  One additional valid pattern
 * is all zeros, which means we don't expect a response.
 */
//#define MMC_RSP_NONE            (0)
//#define MMC_RSP_R1              (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
//#define MMC_RSP_R1B             (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE|MMC_RSP_BUSY)
//#define MMC_RSP_R2              (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC)
//#define MMC_RSP_R3              (MMC_RSP_PRESENT)
//#define MMC_RSP_R4              (MMC_RSP_PRESENT)
//#define MMC_RSP_R5              (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
//#define MMC_RSP_R6              (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
//#define MMC_RSP_R7              (MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)

//#define mmc_resp_type(cmd)      ((cmd)->flags & MMC_RSP_MASK)

/* These are the command types. */
//#define mmc_cmd_type(cmd)       ((cmd)->flags & MMC_CMD_MASK)
};

struct psram_request {
	struct psram_command    cmd;
	struct psram_data       data;
	//void                    (*done)(struct psram_request *); /* completion function */
};

struct psram_ctrl;

int32_t psram_init(struct psram_chip *chip, struct psram_ctrl *ctrl, PSRAMChip_InitParam *param);
int32_t psram_deinit(struct psram_chip *chip);
int32_t Psram_Read_Id(struct psram_chip *chip);
int32_t Psram_Read_Mr(struct psram_chip *chip, uint32_t mreg);
int32_t Psram_Read_Info(struct psram_chip *chip);
int32_t Psram_Read_die_sta(struct psram_chip *chip);

int32_t psram_set_write_latency(struct psram_chip *chip, uint32_t p_type, uint32_t wlc);
int32_t psram_set_read_latency(struct psram_chip *chip, uint32_t fixed, uint32_t rlc);
int32_t psram_set_drv(struct psram_chip *chip, uint32_t drv);
int32_t psram_set_rf(struct psram_chip *chip, uint32_t fast_en);

int32_t psram_sw_reset(struct psram_chip *chip, uint32_t step);
void psram_idbus_op_cmd(struct psram_chip *chip, uint32_t opcmd);
int32_t psram_enter_quad_mode(struct psram_chip *chip);
int32_t psram_exit_quad_mode(struct psram_chip *chip);
int32_t psram_set_wrap_dbt(struct psram_chip *chip, uint32_t m_type);
int32_t psram_sbus_read(struct psram_chip *chip, uint32_t addr,
                        uint8_t *buf, uint32_t len);
int32_t psram_sbus_write(struct psram_chip *chip, uint32_t addr,
                         uint8_t *buf, uint32_t len);
int32_t psram_sbus_dma_read(struct psram_chip *chip, uint32_t page_addr,
                            uint8_t *buf, uint32_t len);
int32_t psram_sbus_dma_write(struct psram_chip *chip, uint32_t page_addr,
                             uint8_t *buf, uint32_t len);
struct psram_chip *psram_open(uint32_t id);
HAL_Status psram_close(struct psram_chip *chip);
void psram_info_dump(struct psram_chip *chip);

void *psram_malloc( size_t xWantedSize );
void *psram_realloc( uint8_t *srcaddr,size_t xWantedSize );
void *psram_calloc( size_t xNmemb, size_t xMembSize );
void psram_free( void *pv );

#endif /* _PSRAM_H */
