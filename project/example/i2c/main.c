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

#include <stdio.h>
#include <string.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_i2c.h"

#define IIC_ID 					I2C0_ID
#define IIC_FREQ				200000
#define IIC_AT24C16_ADDR		0x50
#define IIC_AT24C16_PAGE_SIZE	(16)

static int i2c_init()
{
	HAL_Status status = HAL_ERROR;

	I2C_InitParam initParam;
	initParam.addrMode = I2C_ADDR_MODE_7BIT;
	initParam.clockFreq = IIC_FREQ;

	status = HAL_I2C_Init(IIC_ID, &initParam);
	if (status != HAL_OK) {
		printf("IIC init error %d\n", status);
		return -1;
	}

	return 0;
}

static int i2c_at24c16_write(uint8_t mem_addr, uint8_t *buf, uint32_t size)
{
	if (((mem_addr % 16) + size) > IIC_AT24C16_PAGE_SIZE) {
		printf("roll err\n");
		return -1;
	}

	uint32_t len = HAL_I2C_Master_Transmit_Mem_IT(IIC_ID, IIC_AT24C16_ADDR, mem_addr,
														I2C_MEMADDR_SIZE_8BIT, buf, size);
	if (len != size) {
		printf("to write size=%d, but i2c write size=%d\n", size, len);
		return -1;
	}

	return 0;
}

static int i2c_at24c16_read(uint8_t mem_addr, uint8_t *buf, uint32_t size)
{
	uint32_t len = HAL_I2C_Master_Receive_Mem_IT(IIC_ID, IIC_AT24C16_ADDR, mem_addr,
														I2C_MEMADDR_SIZE_8BIT, buf, size);
	if (len != size) {
		printf("to read size=%d, but i2c read size=%d\n", size, len);
		return -1;
	}

	return 0;
}

static void i2c_deinit()
{
	HAL_Status status = HAL_ERROR;

	status = HAL_I2C_DeInit(IIC_ID);
	if (status != HAL_OK)
		printf("IIC deinit error %d\n", status);
}

static void i2c_test()
{
	#define BUF_SIZE	IIC_AT24C16_PAGE_SIZE

	i2c_init();

	uint16_t i;
	uint16_t value = 0x0;
	uint8_t write_buf[BUF_SIZE];
	uint8_t read_buf[BUF_SIZE];
	for (i = 0; i < BUF_SIZE; i++) {
		write_buf[i] = value++;
	}

	if (i2c_at24c16_write(0, write_buf, BUF_SIZE) == 0) {
		printf("i2c write:");
		for (i = 0; i < BUF_SIZE; i++)
			printf(" %02x", write_buf[i]);
		printf("\n");
	} else
		printf("i2c write fail\n");

	OS_MSleep(5);

	if (i2c_at24c16_read(0, read_buf, BUF_SIZE) == 0) {
		printf("i2c read:");
		for (i = 0; i < BUF_SIZE; i++)
			printf(" %02x", read_buf[i]);
		printf("\n");
	} else
		printf("i2c read fail\n");
}

/* Run this demo, please connect the XR872ET_VER_DIG_V1_0 board. */
int main(void)
{
	printf("i2c demo started\n\n");

	i2c_test();

	printf("i2c demo over\n");

	return 0;
}

