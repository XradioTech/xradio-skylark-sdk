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
#include "string.h"
#include "cmd_util.h"
#include "cmd_spi.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_spi.h"
#include "common/board/board.h"

#define CMD_SPI_PORT SPI1

#define CMD_SPI_DO_OPEN_CLOSE		0
#define CMD_SPI_SLAVE_DELAY_STOP	0

#define CMD_SPI_FIRST_BIT	SPI_TCTRL_FBS_MSB
#define CMD_SPI_SCLK_MODE	SPI_SCLK_Mode0
 //#define CMD_SPI_SCLK 	 (48 * 1000 * 1000)
 //#define CMD_SPI_TEST_TRANSFER_DATA_LEN	 48
 uint32_t testlen;// master and slave must same

#define CMD_SPI_MASTER_BUF_SIZE	(1 * 1024)

/*
 * spi config
 */
static enum cmd_status cmd_spi_config_exec(char *cmd)
{
	int ret;
	static const SPI_Global_Config board_spi_param = {
		.mclk	  = BOARD_SPI_MCLK,
		.cs_level = BOARD_SPI_CS_LEVEL
	};

	ret = HAL_SPI_Init(CMD_SPI_PORT, &board_spi_param);
    if(ret != 0) {
        CMD_ERR("HAL_SPI_Init failed\n");
        return CMD_STATUS_FAIL;
    }

    return CMD_STATUS_OK;
}

/*
 * spi deconfig
 */
static enum cmd_status cmd_spi_deconfig_exec(char *cmd)
{
	HAL_SPI_Deinit(CMD_SPI_PORT);

	return CMD_STATUS_OK;
}

/*spi start m=<mode> l=<line> s=<speed>*/
static enum cmd_status cmd_spi_start_exec(char *cmd)
{
    int ret;
    int cnt;
    uint32_t mode,line,speed;
    SPI_Config config;
    uint8_t dataBuf[16];

    cnt = cmd_sscanf(cmd, "m=%u l=%u s=%u", &mode, &line, &speed);
    if (cnt != 3) {
        CMD_ERR("invalid cnt %u\n", cnt);
        return CMD_STATUS_INVALID_ARG;
    }

    if (mode < 0 || mode > 3) {
		CMD_ERR("invalid line %u\n", mode);
		return CMD_STATUS_INVALID_ARG;
	}

    if (line < 1 || line > 2) {
		CMD_ERR("invalid line %u\n", line);
		return CMD_STATUS_INVALID_ARG;
	}

    if (speed < 1 || speed > 48) {
		CMD_ERR("invalid speed %u\n", speed);
		return CMD_STATUS_INVALID_ARG;
	}

    cmd_memset(&config, 0, sizeof(config));
    config.firstBit = SPI_TCTRL_FBS_MSB;
    config.mode = SPI_CTRL_MODE_MASTER;
    config.opMode = SPI_OPERATION_MODE_DMA;
    config.sclk = speed * 1000 * 1000;
    config.sclkMode = mode;

    ret = HAL_SPI_Open(CMD_SPI_PORT, SPI_TCTRL_SS_SEL_SS0, &config, 10);
    if(ret != 0) {
        CMD_ERR("HAL_SPI_Open failed\n");
        return CMD_STATUS_FAIL;
    }

    HAL_SPI_CS(CMD_SPI_PORT, 1);
    cmd_memcpy(dataBuf, "SPI Test Data", sizeof("SPI Test Data"));
    ret = HAL_SPI_Transmit(CMD_SPI_PORT, (uint8_t*)dataBuf, 16);
    if(ret != 0) {
        CMD_ERR("HAL_SPI_Open failed\n");
        goto close;
    }
    if (line == 2)
        HAL_SPI_Config(CMD_SPI_PORT, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_DUAL_RX);
    else
        HAL_SPI_Config(CMD_SPI_PORT, SPI_ATTRIBUTION_IO_MODE, SPI_IO_MODE_NORMAL);

    HAL_SPI_Receive(CMD_SPI_PORT, dataBuf, 16);
    if(ret != 0) {
        CMD_ERR("HAL_SPI_Receive failed\n");
        goto close;
    }

    HAL_SPI_CS(CMD_SPI_PORT, 0);
    cmd_print_uint8_array(dataBuf, 16);

close:
    HAL_SPI_Close(CMD_SPI_PORT);
    return (ret == 0) ? CMD_STATUS_OK : ret;
}

#if 0
static uint8_t *cmd_spi_master_buf = (uint8_t *)0x7E000;
#else
static uint8_t cmd_spi_master_buf[CMD_SPI_MASTER_BUF_SIZE];
#endif

void cmd_spi_init_data(uint8_t *buf, int32_t len)
{
	int i;
	for (i = 0; i < len; ++i) {
		buf[i] = ((i + 1) & 0xff);
	}
}

int cmd_spi_verify_data(uint8_t *buf, int32_t len)
{
	int i;
#if 0
	uint8_t *ptr = buf;

	for (i = 0; i < len; ++i) {
		if ((i % 16) == 0) {
			printf("\n");
		}
		printf(" %02X", *ptr++);
	}
	printf("\n");
#endif
	for (i = 0; i < len; ++i) {
		if (buf[i] != ((i + 1) & 0xff)) {
			break;
		}
	}
	if (i >= len) {
		CMD_DBG("data correct\n");
		return 1;
	} else {
		CMD_DBG("data wrong, i %d\n", i);
		return 0;
	}
}

#if 0
__nonxip_text
static void cmd_master_irq_cb(uint32_t irq, void *arg)
{
	//CMD_DBG("master spi irq %#x\n", irq);

	//OS_SemaphoreRelease(&cmd_slave_sem);
}
#endif

static enum cmd_status cmd_spi_master_init_exec(char *cmd)
{
	SPI_Device param;
    uint32_t speed;
	uint8_t cnt;
	uint32_t test_len;
    cnt = cmd_sscanf(cmd, "s=%u l=%u", &speed, &test_len);
    if (cnt != 2) {
        CMD_ERR("invalid cnt %u\n", cnt);
        return CMD_STATUS_INVALID_ARG;
    }

    if (speed < 1 || speed > 48) {
		CMD_ERR("invalid speed %u\n", speed);
		return CMD_STATUS_INVALID_ARG;
	}

    if (test_len < 1 || test_len > 1024) {
		CMD_ERR("invalid speed %u\n", test_len);
		return CMD_STATUS_INVALID_ARG;
	}

	cmd_memset(&param, 0, sizeof(SPI_Device));
	param.port = CMD_SPI_PORT;
	param.cs = SPI_TCTRL_SS_SEL_SS0;
	param.config.mode = SPI_CTRL_MODE_MASTER;
	param.config.opMode = SPI_OPERATION_MODE_DMA;
	param.config.firstBit = CMD_SPI_FIRST_BIT;
	param.config.sclk = speed * 1000 * 1000;
	param.config.sclkMode = CMD_SPI_SCLK_MODE;

	testlen = test_len;
	CMD_DBG("sclk %d, test len %d\n", param.config.sclk, testlen);
	if (HAL_SPI_Open(param.port, param.cs, &param.config, 5000) == HAL_OK) {
#if 0
		SPI_IrqParam irq_param;
		irq_param.irqMask = SPI_INT_TRANSFER_COMPLETE; /* or SPI_INT_TRANSFER_COMPLETE */
		irq_param.callback = cmd_master_irq_cb;
		irq_param.arg = NULL;
		HAL_SPI_Slave_EnableIRQ(param.spi_port, &irq_param);
#endif
		return CMD_STATUS_OK;
	} else {
		return CMD_STATUS_FAIL;
	}
}

static enum cmd_status cmd_spi_master_deinit_exec(char *cmd)
{
	HAL_SPI_Close(CMD_SPI_PORT);
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_spi_master_send_exec(char *cmd)
{
	uint8_t *buf = cmd_spi_master_buf;
	int32_t len = testlen;
	enum cmd_status ret = CMD_STATUS_OK;

	cmd_spi_init_data(buf, len);

#if CMD_SPI_DO_OPEN_CLOSE
	cmd_spi_master_init_exec(NULL);
#endif

	HAL_SPI_CS(CMD_SPI_PORT, 1);
	if (HAL_SPI_Transmit(CMD_SPI_PORT, (uint8_t *)buf, len) != HAL_OK) {
		ret = CMD_STATUS_FAIL;
	}
	HAL_SPI_CS(CMD_SPI_PORT, 0);

#if CMD_SPI_DO_OPEN_CLOSE
	cmd_spi_master_deinit_exec(NULL);
#endif

	return ret;
}

static enum cmd_status cmd_spi_master_send_continue_exec(char *cmd)
{
	uint8_t *buf = cmd_spi_master_buf;
	int32_t len = testlen;
	enum cmd_status ret = CMD_STATUS_OK;

	cmd_spi_init_data(buf, len);
	while (1) {
#if CMD_SPI_DO_OPEN_CLOSE
		cmd_spi_master_init_exec(NULL);
#endif

		HAL_SPI_CS(CMD_SPI_PORT, 1);
		if (HAL_SPI_Transmit(CMD_SPI_PORT, (uint8_t *)buf, len) != HAL_OK) {
			ret = CMD_STATUS_FAIL;
		}
		HAL_SPI_CS(CMD_SPI_PORT, 0);

#if CMD_SPI_DO_OPEN_CLOSE
		cmd_spi_master_deinit_exec(NULL);
#endif
		OS_MSleep(1);  /* wait slave recv first exec */
	}
	return ret;
}

static enum cmd_status cmd_spi_master_recv_exec(char *cmd)
{
	uint8_t *buf = cmd_spi_master_buf;
	int32_t len = testlen;
	enum cmd_status ret = CMD_STATUS_OK;

	cmd_memset(buf, 0, len);

#if CMD_SPI_DO_OPEN_CLOSE
	cmd_spi_master_init_exec(NULL);
#endif

	HAL_SPI_CS(CMD_SPI_PORT, 1);
	if (HAL_SPI_Receive(CMD_SPI_PORT, (uint8_t *)buf, len) != HAL_OK) {
		ret = CMD_STATUS_FAIL;
	}
	HAL_SPI_CS(CMD_SPI_PORT, 0);

#if CMD_SPI_DO_OPEN_CLOSE
	cmd_spi_master_deinit_exec(NULL);
#endif

	cmd_spi_verify_data(buf, len);

	return ret;
}

static enum cmd_status cmd_spi_master_recv_continue_exec(char *cmd)
{
	uint8_t *buf = cmd_spi_master_buf;
	int32_t len = testlen;
	enum cmd_status ret = CMD_STATUS_OK;

	uint32_t test_timeout = 1000 * 120;
	uint32_t timeout = OS_GetTicks() + OS_TicksToMSecs(test_timeout);
	uint32_t correct = 0;
	uint32_t wrong = 0;
	uint32_t sum = 0;

	while (OS_TimeBeforeEqual(OS_GetTicks(), timeout)) {
		cmd_memset(buf, 0, len);

#if CMD_SPI_DO_OPEN_CLOSE
		cmd_spi_master_init_exec(NULL);
#endif

		HAL_SPI_CS(CMD_SPI_PORT, 1);
		if (HAL_SPI_Receive(CMD_SPI_PORT, (uint8_t *)buf, len) != HAL_OK) {
			ret = CMD_STATUS_FAIL;
			break;
		}
		HAL_SPI_CS(CMD_SPI_PORT, 0);

#if CMD_SPI_DO_OPEN_CLOSE
		cmd_spi_master_deinit_exec(NULL);
#endif
		sum += 1;

		int i;
		for (i = 0; i < len; ++i) {
			if (buf[i] != ((i + 1) & 0xff)) {
				break;
			}
		}
		if (i >= len) {
			correct += 1;
		} else {
			wrong += 1;
		}
	}
	CMD_DBG("master rx sum %u, correct %u, wrong %u, speed %u(KB/s)\n",
			sum, correct, wrong, (sum * testlen)/((test_timeout / 1000) * 1000));
	return ret;
}

/* after cmd_spi_slave_stable_test_exec exec*/
static enum cmd_status cmd_spi_master_stable_test_exec(char *cmd)
{
	while (1) {
		cmd_spi_master_recv_exec(NULL);
		OS_MSleep(500);
		cmd_spi_master_send_exec(NULL);
		OS_MSleep(500);
	}
	return CMD_STATUS_OK;
}

static const struct cmd_data g_spi_master_cmds[] = {
	{ "init", 	cmd_spi_master_init_exec },
	{ "deinit",	cmd_spi_master_deinit_exec },
	{ "send",	cmd_spi_master_send_exec },
	{ "recv",	cmd_spi_master_recv_exec },
	{ "send_continue", 	cmd_spi_master_send_continue_exec },
	{ "recv_continue", 	cmd_spi_master_recv_continue_exec },
	{ "stable_test", 	cmd_spi_master_stable_test_exec },
};

static enum cmd_status cmd_spi_master_exec(char *cmd)
{
	return cmd_exec(cmd, g_spi_master_cmds, cmd_nitems(g_spi_master_cmds));
}

typedef enum
{
  SPI_SLAVE_INIT,
  SPI_SLAVE_STATUS_TX,
  SPI_SLAVE_STATUS_RX,
  SPI_SLAVE_DEINIT,
} SPI_Slave_Status;

SPI_Slave_Status slave_status = SPI_SLAVE_DEINIT;
OS_Semaphore_t cmd_slave_sem;

__nonxip_text
static void cmd_slave_dma_half_cb(void *arg)
{
}

__nonxip_text
static void cmd_slave_dma_end_cb(void *arg)
{
	//OS_SemaphoreRelease(&cmd_slave_sem);
	//CMD_DBG("dam end\n");
}

__nonxip_text
static void cmd_slave_irq_cb(uint32_t irq, void *arg)
{
	//CMD_DBG("slave spi irq %#x\n", irq);
	/*if spi slave init first, spi slave cs level is high defult,but master cs connect slave and low,
	 *spi slave cs change to low too. then master init, master cs set high defaut and then slave cs low
	 *change to high and the deselect irq is triggered.
    */
	if((slave_status == SPI_SLAVE_STATUS_TX) | (slave_status == SPI_SLAVE_STATUS_RX))
		OS_SemaphoreRelease(&cmd_slave_sem);
}

static enum cmd_status cmd_spi_slave_init_exec(char *cmd)
{
	SPI_Device param;
	uint8_t cnt;
	uint32_t test_len;

    cnt = cmd_sscanf(cmd, "l=%u", &test_len);
    if (cnt != 1) {
        CMD_ERR("invalid cnt %u\n", cnt);
        return CMD_STATUS_INVALID_ARG;
    }

    if (test_len < 1 || test_len > 1024) {
		CMD_ERR("invalid speed %u\n", test_len);
		return CMD_STATUS_INVALID_ARG;
	}

	if (OS_SemaphoreCreateBinary(&cmd_slave_sem) != OS_OK) {
		CMD_ERR("create sem fail\n");
	}

	cmd_memset(&param, 0, sizeof(SPI_Device));
	param.port = CMD_SPI_PORT;
	param.cs = SPI_TCTRL_SS_SEL_SS0;
	param.config.mode = SPI_CTRL_MODE_SLAVE;
	param.config.opMode = SPI_OPERATION_MODE_DMA;
	param.config.firstBit = CMD_SPI_FIRST_BIT;
	//param.config.sclk = CMD_SPI_SCLK;  //no need to set
	param.config.sclkMode = CMD_SPI_SCLK_MODE;

	testlen = test_len;
	CMD_DBG("test len %d\n", testlen);

	/* init spi */
	if (HAL_SPI_Slave_Open(param.port, param.cs, &param.config, 5000) != HAL_OK) {
		CMD_ERR("HAL_SPI_Slave_Open() fail\n");
	}

	DMA_ChannelInitParam dma_param;
	cmd_memset(&dma_param, 0, sizeof(dma_param));
	dma_param.irqType = DMA_IRQ_TYPE_BOTH;
	dma_param.endCallback = cmd_slave_dma_end_cb;
	dma_param.endArg = NULL;
	dma_param.halfCallback = cmd_slave_dma_half_cb;
	dma_param.halfArg = NULL;

	if (HAL_SPI_Slave_InitTxDMA(param.port, &dma_param) != HAL_OK) {
		CMD_ERR("HAL_SPI_Slave_InitTxDMA() fail\n");
	}

	if (HAL_SPI_Slave_InitRxDMA(param.port, &dma_param) != HAL_OK) {
		CMD_ERR("HAL_SPI_Slave_InitRxDMA() fail\n");
	}

#if 1
	SPI_IrqParam irq_param;
	irq_param.irqMask = SPI_INT_CS_DESELECT; /* or SPI_INT_TRANSFER_COMPLETE/SPI_INT_CS_DESELECT */
	irq_param.callback = cmd_slave_irq_cb;
	irq_param.arg = NULL;
	HAL_SPI_Slave_EnableIRQ(param.port, &irq_param);
#endif
	slave_status = SPI_SLAVE_INIT;

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_spi_slave_deinit_exec(char *cmd)
{
	HAL_SPI_Slave_DisableIRQ(CMD_SPI_PORT);
	HAL_SPI_Slave_DeInitRxDMA(CMD_SPI_PORT);
	HAL_SPI_Slave_DeInitTxDMA(CMD_SPI_PORT);
	HAL_SPI_Slave_Close(CMD_SPI_PORT);
	slave_status = SPI_SLAVE_DEINIT;
	OS_SemaphoreDelete(&cmd_slave_sem);
	return CMD_STATUS_OK;
}

static enum cmd_status cmd_spi_slave_send_exec(char *cmd)
{
	uint8_t *buf = cmd_spi_master_buf;
	int32_t len = testlen;
	enum cmd_status ret = CMD_STATUS_OK;

	cmd_spi_init_data(buf, len);

#if CMD_SPI_DO_OPEN_CLOSE
	cmd_spi_slave_init_exec(NULL);
#endif

	if (HAL_SPI_Slave_StartTransmit_DMA(CMD_SPI_PORT, buf, len) != HAL_OK) {
		CMD_ERR("HAL_SPI_Slave_StartTransmit_DMA() fail\n");
	}

	slave_status = SPI_SLAVE_STATUS_TX;
	if (OS_SemaphoreWait(&cmd_slave_sem, 10000) != OS_OK) {
		CMD_ERR("%s(), wait timeout\n", __func__);
	}

#if CMD_SPI_SLAVE_DELAY_STOP
	cmd_msleep(CMD_SPI_SLAVE_DELAY_STOP);
#endif

	int32_t left = HAL_SPI_Slave_StopTransmit_DMA(CMD_SPI_PORT);
	if (left != 0) {
		CMD_ERR("HAL_SPI_Slave_StopTransmit_DMA(), left %d\n", left);
		ret = CMD_STATUS_FAIL;
	}

#if CMD_SPI_DO_OPEN_CLOSE
	cmd_spi_slave_deinit_exec(NULL);
#endif

	return ret;
}

static enum cmd_status cmd_spi_slave_send_continue_exec(char *cmd)
{
	uint8_t *buf = cmd_spi_master_buf;
	int32_t len = testlen;
	enum cmd_status ret = CMD_STATUS_OK;

	cmd_spi_init_data(buf, len);
	while (1) {
#if CMD_SPI_DO_OPEN_CLOSE
		cmd_spi_slave_init_exec(NULL);
#endif

		if (HAL_SPI_Slave_StartTransmit_DMA(CMD_SPI_PORT, buf, len) != HAL_OK) {
			CMD_ERR("HAL_SPI_Slave_StartTransmit_DMA() fail\n");
		}

		slave_status = SPI_SLAVE_STATUS_TX;
		if (OS_SemaphoreWait(&cmd_slave_sem, OS_WAIT_FOREVER) != OS_OK) {
			CMD_ERR("%s(), wait timeout\n", __func__);
		}

#if CMD_SPI_SLAVE_DELAY_STOP
		cmd_msleep(CMD_SPI_SLAVE_DELAY_STOP);
#endif

		int32_t left = HAL_SPI_Slave_StopTransmit_DMA(CMD_SPI_PORT);
		if (left != 0) {
			CMD_ERR("HAL_SPI_Slave_StopTransmit_DMA(), left %d\n", left);
			ret = CMD_STATUS_FAIL;
			break;
		}

#if CMD_SPI_DO_OPEN_CLOSE
		cmd_spi_slave_deinit_exec(NULL);
#endif
	}
	return ret;
}

static enum cmd_status cmd_spi_slave_recv_exec(char *cmd)
{
	uint8_t *buf = cmd_spi_master_buf;
	int32_t len = testlen;
	enum cmd_status ret = CMD_STATUS_OK;

	cmd_memset(buf, 0, len);

#if CMD_SPI_DO_OPEN_CLOSE
	cmd_spi_slave_init_exec(NULL);
#endif

	if (HAL_SPI_Slave_StartReceive_DMA(CMD_SPI_PORT, buf, len) != HAL_OK) {
		CMD_ERR("HAL_SPI_Slave_StartReceive_DMA() fail\n");
	}

	slave_status = SPI_SLAVE_STATUS_RX;
	if (OS_SemaphoreWait(&cmd_slave_sem, 10000) != OS_OK) {
		CMD_ERR("%s(), wait timeout\n", __func__);
	}


#if CMD_SPI_SLAVE_DELAY_STOP
	cmd_msleep(CMD_SPI_SLAVE_DELAY_STOP);
#endif

	int32_t left = HAL_SPI_Slave_StopReceive_DMA(CMD_SPI_PORT);
	if (left != 0) {
		CMD_ERR("HAL_SPI_Slave_StopReceive_DMA(), left %d\n", left);
		ret = CMD_STATUS_FAIL;
	}

#if CMD_SPI_DO_OPEN_CLOSE
	cmd_spi_slave_deinit_exec(NULL);
#endif

	cmd_spi_verify_data(buf, len);

	return ret;
}

static enum cmd_status cmd_spi_slave_recv_continue_exec(char *cmd)
{
	uint8_t *buf = cmd_spi_master_buf;
	int32_t len = testlen;
	enum cmd_status ret = CMD_STATUS_OK;

	uint32_t test_timeout = 1000 * 120;
	uint32_t timeout = OS_GetTicks() + OS_TicksToMSecs(test_timeout);
	uint32_t correct = 0;
	uint32_t wrong = 0;
	uint32_t sum = 0;

	while (OS_TimeBeforeEqual(OS_GetTicks(), timeout)) {

		cmd_memset(buf, 0, len);

#if CMD_SPI_DO_OPEN_CLOSE
		cmd_spi_slave_init_exec(NULL);
#endif

		if (HAL_SPI_Slave_StartReceive_DMA(CMD_SPI_PORT, buf, len) != HAL_OK) {
			CMD_ERR("HAL_SPI_Slave_StartReceive_DMA() fail\n");
		}

		slave_status = SPI_SLAVE_STATUS_RX;
		if (OS_SemaphoreWait(&cmd_slave_sem, OS_WAIT_FOREVER) != OS_OK) {
			CMD_ERR("%s(), wait timeout\n", __func__);
		}


#if CMD_SPI_SLAVE_DELAY_STOP
		cmd_msleep(CMD_SPI_SLAVE_DELAY_STOP);
#endif

		int32_t left = HAL_SPI_Slave_StopReceive_DMA(CMD_SPI_PORT);
		if (left != 0) {
			CMD_ERR("HAL_SPI_Slave_StopReceive_DMA(), left %d\n", left);
			ret = CMD_STATUS_FAIL;
			break;
		}

#if CMD_SPI_DO_OPEN_CLOSE
		cmd_spi_slave_deinit_exec(NULL);
#endif

		sum += 1;

		int i;
		for (i = 0; i < len; ++i) {
			if (buf[i] != ((i + 1) & 0xff)) {
				break;
			}
		}
		if (i >= len) {
			correct += 1;
		} else {
			wrong += 1;
		}
	}
	/* master need to wait slave recv first exec, so the speed is not exact */
	CMD_DBG("slave rx sum %u, correct %u, wrong %u, speed %u(KB/s)\n",
			sum, correct, wrong, (sum * testlen)/((test_timeout / 1000) * 1000));
	return ret;
}

static enum cmd_status cmd_spi_slave_stable_test_exec(char *cmd)
{
	while (1) {
		cmd_spi_slave_send_exec(NULL);
		cmd_spi_slave_recv_exec(NULL);
	}
	return CMD_STATUS_OK;
}

static const struct cmd_data g_spi_slave_cmds[] = {
	{ "init", 	cmd_spi_slave_init_exec },
	{ "deinit",	cmd_spi_slave_deinit_exec },
	{ "send", 	cmd_spi_slave_send_exec },
	{ "recv", 	cmd_spi_slave_recv_exec },
	{ "send_continue",	cmd_spi_slave_send_continue_exec },
	{ "recv_continue",	cmd_spi_slave_recv_continue_exec },
	{ "stable_test", cmd_spi_slave_stable_test_exec},
};

static enum cmd_status cmd_spi_slave_exec(char *cmd)
{
	return cmd_exec(cmd, g_spi_slave_cmds, cmd_nitems(g_spi_slave_cmds));
}

static const struct cmd_data g_spi_cmds[] = {
	{ "config",         cmd_spi_config_exec },
    { "deconfig",       cmd_spi_deconfig_exec },
    { "start",          cmd_spi_start_exec },
	{ "master",         cmd_spi_master_exec },
	{ "slave",	        cmd_spi_slave_exec },
};

enum cmd_status cmd_spi_exec(char *cmd)
{
	return cmd_exec(cmd, g_spi_cmds, cmd_nitems(g_spi_cmds));
}
