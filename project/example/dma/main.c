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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_flashctrl.h"
#include "data_header.h"

#define DMA_SRAM_MODE
//#define DMA_UART0_MODE

#define MAX_DATAWIDTH       2
#define MAX_BUSTLEN         1

#define DMA_START(chan, srcAddr, dstAddr, datalen)	HAL_DMA_Start(chan, srcAddr, dstAddr, datalen)
/*
 * @return If DMA byte count mode is DMA_BYTE_CNT_MODE_REMAIN,
 *         the DMA_GET_BYTE_COUNT(chan) return value is the length of the remaining data not transferred,
 *         but DMA_BYTE_CNT_MODE_NORMAL mode is the data length set by HAL_DMA_Start().
 */
#define DMA_GET_BYTE_COUNT(chan)					HAL_DMA_GetByteCount(chan)
#if (defined(DMA_SRAM_MODE))
	#define DMA_IRQ_TYPE		DMA_PERIPH_SRAM
#elif (defined(DMA_UART0_MODE))
	#define DMA_IRQ_TYPE		DMA_PERIPH_UART0	/* write to uart0, but no receive by user send to uart0 */
#endif

#define SRAM_SRC_FIFO_ADDR		0x00250000			/* free memory addr */
#define SRAM_DST_FIFO_ADDR		0x00250000			/* free memory addr */
#define UART0_SRC_FIFO_ADDR		0x40040C00			/* uart0 rx addr */
#define UART0_DST_FIFO_ADDR		0x40040C00			/* uart0 tx addr */
#define SRAM_FIFO_SIZE			(64 * 1024)			/* send & receive data size */
#define UART0_FIFO_SIZE			9					/* send & receive data size */

typedef struct dmaPriv {
	uint8_t     workMode;
	uint8_t     waitCycle;
	uint8_t     byteCntMode;
	uint8_t     addrMode;
	uint32_t    srcFifo;
	uint32_t    dstFifo;
	uint32_t    fifoSize;
} dmaPriv_t;

typedef struct dmaChanalPriv {
	DMA_WorkMode workMode;
	DMA_WaitCycle waitCycle;
	DMA_ByteCntMode byteCntMode;
	DMA_DataWidth dstDataWidth;
	DMA_BurstLen dstBurstLen;
	DMA_AddrMode dstAddrMode;
	DMA_Periph dstPeriph;
	DMA_DataWidth srcDataWidth;
	DMA_BurstLen srcBurstLen;
	DMA_AddrMode srcAddrMode;
	DMA_Periph srcPeriph;
} dmaChanalPriv_T;

typedef enum {
	DMA_WRITE_CHANAL,
	DMA_READ_CHANAL
} dmaInitMode;

dmaPriv_t dmaPriv[] = {
	[DMA_PERIPH_SRAM]           = {DMA_WORK_MODE_SINGLE, DMA_WAIT_CYCLE_1, DMA_BYTE_CNT_MODE_REMAIN, DMA_ADDR_MODE_INC, (SRAM_SRC_FIFO_ADDR), (SRAM_DST_FIFO_ADDR), SRAM_FIFO_SIZE},
	[DMA_PERIPH_UART0]          = {DMA_WORK_MODE_SINGLE, DMA_WAIT_CYCLE_1, DMA_BYTE_CNT_MODE_REMAIN, DMA_ADDR_MODE_FIXED, (UART0_SRC_FIFO_ADDR), (UART0_DST_FIFO_ADDR), UART0_FIFO_SIZE},
/*
 *	@note The following are all channels supported by the DMA, but the corresponding configuration is required to be used.
 *	      If you need to use it, you can go to the corresponding sample project to view.
 *	[DMA_PERIPH_SPI0],
 *	[DMA_PERIPH_SPI1],
 *	[DMA_PERIPH_UART1],
 *	[DMA_PERIPH_CE],
 *	[DMA_PERIPH_DAUDIO],
 *	[DMA_PERIPH_FLASHC],
 *	[DMA_PERIPH_DMIC],
 *	#if (__CONFIG_CHIP_ARCH_VER == 2)
 *		[DMA_PERIPH_UART2],
 *		[DMA_PERIPH_AUDIO_CODEC],
 *		[DMA_PERIPH_GPADC],
 *		[DMA_PERIPH_PSRAMC],
 *	#endif
 */
};

void Stop_Dma(void *arg)
{
	DMA_Channel chanal = (DMA_Channel)arg;

	HAL_DMA_Stop(chanal);
	HAL_DMA_Release(chanal);
}

DMA_Channel Dma_Request(dmaChanalPriv_T dmaChanalPriv)
{
	DMA_Channel chanal;
	DMA_ChannelInitParam param;
	/*request a free channel.*/
	chanal = HAL_DMA_Request();
	if (chanal == DMA_CHANNEL_INVALID) {
		return DMA_CHANNEL_INVALID;
	}
	param.cfg =  HAL_DMA_MakeChannelInitCfg(dmaChanalPriv.workMode,
											dmaChanalPriv.waitCycle,
											dmaChanalPriv.byteCntMode,
											dmaChanalPriv.dstDataWidth,
											dmaChanalPriv.dstBurstLen,
											dmaChanalPriv.dstAddrMode,
											dmaChanalPriv.dstPeriph,

											dmaChanalPriv.srcDataWidth,
											dmaChanalPriv.srcBurstLen,
											dmaChanalPriv.srcAddrMode,
											dmaChanalPriv.srcPeriph);
	param.endArg = (void *)chanal;
	param.endCallback = Stop_Dma;
	param.irqType = DMA_IRQ_TYPE_END;
	HAL_DMA_Init(chanal, &param);
	return chanal;
}

DMA_Channel Dma_Init(uint8_t src_DataWidth, uint8_t src_BustLen,
						uint8_t dest_DataWidth, uint8_t dest_BustLen, dmaInitMode mode)
{
	DMA_Channel chanal;
	dmaChanalPriv_T dmaChanalPri;

	dmaChanalPri.workMode = dmaPriv[DMA_IRQ_TYPE].workMode;
	dmaChanalPri.waitCycle = dmaPriv[DMA_IRQ_TYPE].waitCycle;
	dmaChanalPri.byteCntMode = dmaPriv[DMA_IRQ_TYPE].byteCntMode;

	if (mode == DMA_WRITE_CHANAL) {
		dmaChanalPri.dstDataWidth = dest_DataWidth;
		dmaChanalPri.dstBurstLen = dest_BustLen;
		dmaChanalPri.srcDataWidth = src_DataWidth;
		dmaChanalPri.srcBurstLen = src_BustLen;
		dmaChanalPri.dstAddrMode = dmaPriv[DMA_IRQ_TYPE].addrMode;
		dmaChanalPri.srcAddrMode = DMA_ADDR_MODE_INC;
		dmaChanalPri.dstPeriph = DMA_IRQ_TYPE;
		dmaChanalPri.srcPeriph = DMA_PERIPH_SRAM;
	} else if (mode == DMA_READ_CHANAL) {
		dmaChanalPri.dstDataWidth = src_DataWidth;
		dmaChanalPri.dstBurstLen = src_BustLen;
		dmaChanalPri.srcDataWidth = dest_DataWidth;
		dmaChanalPri.srcBurstLen = dest_BustLen;
		dmaChanalPri.dstAddrMode = DMA_ADDR_MODE_INC;
		dmaChanalPri.srcAddrMode = dmaPriv[DMA_IRQ_TYPE].addrMode;
		dmaChanalPri.dstPeriph = DMA_PERIPH_SRAM;
		dmaChanalPri.srcPeriph = DMA_IRQ_TYPE;
	} else {
		printf("dmaInitMode erro!\n");
		return -1;
	}

	chanal = Dma_Request(dmaChanalPri);
	if (chanal == DMA_CHANNEL_INVALID) {
		printf("DMA request error\n");
		return DMA_CHANNEL_INVALID;
	}

	return chanal;
}

void Dma_DeInit(DMA_Channel chan)
{
	HAL_DMA_DeInit(chan);
	HAL_DMA_Release(chan);
}

int Dma_Write_Read(DMA_Channel ch_Write, DMA_Channel ch_Read)
{
	uint32_t writeSrcFifo, writeDstFifo, readSrcFifo, readDstFifo;

	readSrcFifo = writeDstFifo = dmaPriv[DMA_IRQ_TYPE].dstFifo;
#if (defined(DMA_SRAM_MODE))
	uint32_t *dataBuf = NULL;

	dataBuf = malloc(dmaPriv[DMA_IRQ_TYPE].fifoSize);
	if (dataBuf == NULL) {
		printf("malloc failed\n");
		return -1;
	}
	memset(dataBuf, 0, dmaPriv[DMA_IRQ_TYPE].fifoSize);
	writeSrcFifo = (uint32_t)arrData;
	readDstFifo = (uint32_t)dataBuf;
#elif (defined(DMA_UART0_MODE))
	char uartStr[] = {"123456789"};
	char *strdata = NULL;

	strdata = (char *)malloc(dmaPriv[DMA_IRQ_TYPE].fifoSize * sizeof(char));
	if (strdata == NULL) {
		printf("malloc failed\n");
		return -1;
	}
	memset(strdata, 0, dmaPriv[DMA_IRQ_TYPE].fifoSize * sizeof(char));
	writeSrcFifo = (uint32_t)uartStr;
	readDstFifo = (uint32_t)strdata;
#endif

	DMA_START(ch_Write, writeSrcFifo, writeDstFifo, dmaPriv[DMA_IRQ_TYPE].fifoSize);
	while (DMA_GET_BYTE_COUNT(ch_Write))	/* Suitable for DMA_BYTE_CNT_MODE_REMAIN mode */
		;

	DMA_START(ch_Read, readSrcFifo, readDstFifo, dmaPriv[DMA_IRQ_TYPE].fifoSize);
	while (DMA_GET_BYTE_COUNT(ch_Read))		/* Suitable for DMA_BYTE_CNT_MODE_REMAIN mode */
		;

#if (defined(DMA_SRAM_MODE))
	if (memcmp(arrData, dataBuf, dmaPriv[DMA_IRQ_TYPE].fifoSize) != 0)
#elif (defined(DMA_UART0_MODE))
	if (strncmp(&uartStr[0], strdata, dmaPriv[DMA_IRQ_TYPE].fifoSize))
#endif
		printf("Write & Read data compare failed!\n");
	else
		printf("Write & Read data compare OK.\n");
#if (defined(DMA_SRAM_MODE))
	free(dataBuf);
	dataBuf = NULL;
#elif (defined(DMA_UART0_MODE))
	free(strdata);
	strdata = NULL;
#endif

	return 0;
}

int Dma_Show(void)
{
	DMA_Channel ch_Write;
	DMA_Channel ch_Read;
	uint8_t src_DataWidth, src_BustLen, dest_DataWidth, dest_BustLen;

#if (defined(DMA_SRAM_MODE))
	for (src_DataWidth = 0; src_DataWidth <= MAX_DATAWIDTH; src_DataWidth++) {
		for (src_BustLen = 0; src_BustLen <= MAX_BUSTLEN; src_BustLen++) {
			for (dest_DataWidth = 0; dest_DataWidth <= MAX_DATAWIDTH; dest_DataWidth++) {
				for (dest_BustLen = 0; dest_BustLen <= MAX_BUSTLEN; dest_BustLen++) {
					printf("module %d:src_DataWidth=%d, src_BustLen=%d, dest_DataWidth=%d, dest_BustLen=%d\n",
						DMA_IRQ_TYPE, src_DataWidth, src_BustLen, dest_DataWidth, dest_BustLen);
					ch_Write = Dma_Init(src_DataWidth, src_BustLen, dest_DataWidth, dest_BustLen, DMA_WRITE_CHANAL);
					ch_Read = Dma_Init(src_DataWidth, src_BustLen, dest_DataWidth, dest_BustLen, DMA_READ_CHANAL);
					Dma_Write_Read(ch_Write, ch_Read);
					Dma_DeInit(ch_Write);
					Dma_DeInit(ch_Read);
					OS_MSleep(10);
				}
			}
		}
	}
#elif (defined(DMA_UART0_MODE))
	src_DataWidth = 0;
	src_BustLen = 0;
	dest_DataWidth = 0;
	dest_BustLen = 0;

	printf("module %d:src_DataWidth=%d, src_BustLen=%d, dest_DataWidth=%d, dest_BustLen=%d\n",
		DMA_IRQ_TYPE, src_DataWidth, src_BustLen, dest_DataWidth, dest_BustLen);
	ch_Write = Dma_Init(src_DataWidth, src_BustLen, dest_DataWidth, dest_BustLen, DMA_WRITE_CHANAL);
	ch_Read = Dma_Init(src_DataWidth, src_BustLen, dest_DataWidth, dest_BustLen, DMA_READ_CHANAL);
	Dma_Write_Read(ch_Write, ch_Read);
	Dma_DeInit(ch_Write);
	Dma_DeInit(ch_Read);
#endif
	return 0;
}

/* this example just show use SRAM/Uart0 for DMA , other chanal can be view on corresponding example or demo */
/* SRAM mode:using dma to copy data to another memory */
/* Uart0 mode:using dma to send & receive data to uart0 */
int main(void)
{
	printf("dma example started\n\n");
	Dma_Show();
	printf("dma example end\n\n");

	return 0;
}
