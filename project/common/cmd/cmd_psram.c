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
#include <stdlib.h>
#include "sys/io.h"
#include "sys/xr_debug.h"
#include "cmd_util.h"
#include "cmd_psram.h"
#include "image/image.h"

#include "driver/chip/hal_rtc.h"
#include "driver/chip/psram/psram.h"
#include "driver/chip/psram/hal_psramctrl.h"
#include "driver/chip/hal_dcache.h"

#define CMD_PSRAM_EXE_AND_BENCH /* run code and read write data in psram */

#ifdef CMD_PSRAM_EXE_AND_BENCH
#define __CMD_SRAM_RODATA __psram_rodata
#define __CMD_SRAM_DATA __psram_data
#define __CMD_SRAM_TEXT __psram_text
#else
#define __CMD_SRAM_RODATA __sram_rodata
#define __CMD_SRAM_DATA __sram_data
#define __CMD_SRAM_TEXT __sram_text
#endif

#define CMD_PSRAM_DBG(fmt, arg...)                      \
	do {                                            \
		__CMD_SRAM_RODATA static const char __fmt[] = fmt;    \
		printf(__fmt, ##arg);                   \
	} while (0)

#define CMD_SRAM_DBG(fmt, arg...)                       \
	do {                                            \
		__CMD_SRAM_RODATA static const char __fmt[] = fmt;\
		printf(__fmt, ##arg);		        \
	} while (0)

#define CMD_PSRAM_WRN(fmt, arg...)                      \
	CMD_SRAM_DBG("[WRN] "fmt, ##arg)

#define CMD_PSRAM_ERR(fmt, arg...)                      \
	do {                                            \
		CMD_SRAM_DBG("[ERR] %s():%d, "fmt,      \
		             __func__, __LINE__, ##arg);\
	} while (0)


__CMD_SRAM_DATA
static uint32_t dmaPrintFlgCpu = 0;
__CMD_SRAM_DATA
static uint32_t dmaPrintFlgDma = 0;

__CMD_SRAM_DATA
static OS_Semaphore_t dmaSem;

__CMD_SRAM_TEXT
static void psram_DMARelease(void *arg)
{
	OS_SemaphoreRelease(arg);
}

__CMD_SRAM_TEXT
static int psram_dma_read_write(uint32_t write, uint32_t addr,
                               uint8_t *buf, uint32_t len, void *arg)
{
	OS_Status ret;
	DMA_ChannelInitParam dmaParam;
	DMA_Channel dma_ch;
	DMA_DataWidth dma_data_width;
	OS_Semaphore_t *dma_sem;
	uint32_t wt_saddr, wt_eaddr;

	dma_ch = HAL_DMA_Request();
	if (dma_ch == DMA_CHANNEL_INVALID) {
		CMD_PSRAM_ERR("%s,%d\n", __func__, __LINE__);
		return -1;
	}

	if (arg)
		dma_sem = arg;
	else
		dma_sem = &dmaSem;
	OS_SemaphoreCreate(dma_sem, 0, 1);
	if (!dmaPrintFlgDma) {
		CMD_PSRAM_DBG("Flg[%p]:0x%x dmaSem[%x]:0x%x\n",
		              &dmaPrintFlgDma, dmaPrintFlgDma,
		              (uint32_t)dma_sem, (uint32_t)dma_sem->handle);
		dmaPrintFlgDma = 1;
	}

	dmaParam.irqType = DMA_IRQ_TYPE_END;
	dmaParam.endCallback = (DMA_IRQCallback)psram_DMARelease;
	dmaParam.endArg = dma_sem;

	if (addr & 0x1)
		dma_data_width = DMA_DATA_WIDTH_8BIT;
	else if ((addr & 0x3) == 0x2)
		dma_data_width = DMA_DATA_WIDTH_16BIT;
	else
		dma_data_width = DMA_DATA_WIDTH_32BIT;

	if (write) {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
				DMA_WAIT_CYCLE_2,
				DMA_BYTE_CNT_MODE_REMAIN,
				dma_data_width,
				DMA_BURST_LEN_1,
				DMA_ADDR_MODE_INC,
				(DMA_Periph)(DMA_PERIPH_PSRAMC),
				DMA_DATA_WIDTH_8BIT,
				DMA_BURST_LEN_4,
				DMA_ADDR_MODE_INC,
				DMA_PERIPH_SRAM);
	} else {
		dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
				DMA_WAIT_CYCLE_2,
				DMA_BYTE_CNT_MODE_REMAIN,
				DMA_DATA_WIDTH_8BIT,
				DMA_BURST_LEN_4,
				DMA_ADDR_MODE_INC,
				DMA_PERIPH_SRAM,
				dma_data_width,
				DMA_BURST_LEN_1,
				DMA_ADDR_MODE_INC,
				(DMA_Periph)(DMA_PERIPH_PSRAMC));
	}
	HAL_DMA_Init(dma_ch, &dmaParam);

	wt_saddr = rounddown2(addr, 16);
	wt_eaddr = roundup2((uint32_t)addr + len, 16);
	HAL_Dcache_SetWriteThrough(1, 1, wt_saddr, wt_eaddr);

	if (write)
		HAL_DMA_Start(dma_ch, (uint32_t)buf, (uint32_t)addr, len);
	else
		HAL_DMA_Start(dma_ch, (uint32_t)addr, (uint32_t)buf, len);

	ret = OS_SemaphoreWait(dma_sem, 2000);
	if (ret != OS_OK)
		CMD_PSRAM_ERR("sem wait failed: %d\n", ret);

	HAL_Dcache_SetWriteThrough(1, 0, 0, 0);

	HAL_DMA_Stop(dma_ch);
	HAL_DMA_DeInit(dma_ch);
	HAL_DMA_Release(dma_ch);

	OS_SemaphoreDelete(dma_sem);

	return 0;
}

typedef int (*psram_read_write)(uint32_t write, uint32_t addr, uint8_t *buf, uint32_t len, void *arg);

__CMD_SRAM_TEXT
static int psram_dbus_cpu_read_write(uint32_t write, uint32_t addr, uint8_t *buf, uint32_t len, void *arg)
{
	if (!dmaPrintFlgCpu) {
		CMD_PSRAM_DBG("Flg[%p]:0x%x\n", &dmaPrintFlgCpu, dmaPrintFlgCpu);
		dmaPrintFlgCpu = 1;
	}

	if (write)
		memcpy((void *)addr, buf, len);
	else
		memcpy(buf, (void *)addr, len);

	return 0;
}

__CMD_SRAM_TEXT
static int psram_dbus_dma_read_write(uint32_t write, uint32_t addr, uint8_t *buf, uint32_t len, void *arg)
{
    int ret;
	if (write)
		ret = psram_dma_read_write(1, addr, buf, len, arg);
	else
		ret = psram_dma_read_write(0, addr, buf, len, arg);

	return ret;
}

__CMD_SRAM_TEXT
static int psram_sbus_cpu_read_write(uint32_t write, uint32_t addr, uint8_t *buf, uint32_t len, void *arg)
{
	struct psram_chip *chip;

	chip = psram_open(0);
	if (!chip) {
		CMD_PSRAM_ERR("invalid chip\n");
		return -1;
	}

	if (write)
		psram_sbus_write(chip, addr, buf, len);
	else
		psram_sbus_read(chip, addr, buf, len);
	psram_close(chip);

	return 0;
}

__CMD_SRAM_TEXT
static int psram_sbus_dma_read_write(uint32_t write, uint32_t addr, uint8_t *buf, uint32_t len, void *arg)
{
	struct psram_chip *chip;
	uint32_t wt_saddr, wt_eaddr;

	chip = psram_open(0);
	if (!chip) {
		CMD_PSRAM_ERR("invalid chip\n");
		return -1;
	}

	wt_saddr = rounddown2(addr, 16);
	wt_eaddr = roundup2((uint32_t)addr + len, 16);

	HAL_Dcache_SetWriteThrough(0, 1, wt_saddr, wt_eaddr);

	if (write)
		psram_sbus_dma_write(chip, addr, buf, len);
	else
		psram_sbus_dma_read(chip, addr, buf, len);

	HAL_Dcache_SetWriteThrough(0, 0, 0, 0);

	psram_close(chip);

	return 0;
}

__CMD_SRAM_DATA
static psram_read_write psram_rw_op[] =
	{psram_dbus_cpu_read_write,
	 psram_dbus_dma_read_write,
	 psram_sbus_cpu_read_write,
	 psram_sbus_dma_read_write,
	};

__CMD_SRAM_DATA
static volatile uint32_t psram_val = 0x12345;

/*
 * drv psram info
 */
__CMD_SRAM_TEXT
enum cmd_status cmd_info_exec(char *cmd)
{
	struct psram_chip *chip;

	chip = psram_open(0);
	if (!chip) {
		CMD_PSRAM_ERR("invalid chip\n");
		return CMD_STATUS_FAIL;
	}
	psram_info_dump(chip);
	psram_close(0);

	return CMD_STATUS_OK;
}

/*
 * drv psram run
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_run_exec(char *cmd)
{
	printf("%s read:0x%x\n", __func__, psram_val);

	return CMD_STATUS_OK;
}

/*
 * drv psram read <b/w/l or B/W/L> <mode> <0xadd>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_read_exec(char *cmd)
{
	int32_t ret;
	uint32_t *addr;
	char type;
	uint32_t mode;
    uint32_t len;
	uint8_t *pdata;

	ret = cmd_sscanf(cmd, "%c %d 0x%x %d", &type, &mode, (unsigned int *)&addr, &len);
	if (ret != 4 || mode > 3) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}
    if (type == 'w' || type == 'W') {
        len = len << 1;
    } else if (type == 'l' || type == 'L') {
        len = len << 2;
    }

    if (len == 0) {
        CMD_PSRAM_ERR("invalid len = 0 \n");
        return CMD_STATUS_INVALID_ARG;
    }

    if(((uint32_t)addr < (uint32_t)__PSRAM_BASE) || (len > (uint32_t)__PSRAM_LENGTH) \
        || (((uint32_t)addr+len) > ((uint32_t)__PSRAM_BASE + (uint32_t)__PSRAM_LENGTH))) {
        CMD_PSRAM_ERR("Invalid param\n");
        return CMD_STATUS_INVALID_ARG;
    }

    pdata = (uint8_t *)cmd_malloc(len);
    if(pdata == NULL) {
        CMD_PSRAM_ERR("malloc failed\n");
		return CMD_STATUS_INVALID_ARG;
    }

	if (type == 'b' || type == 'B') {
        ret = psram_rw_op[mode](0, (uint32_t)addr, pdata, len, NULL);
	} else if (type == 'w' || type == 'W') {
        ret = psram_rw_op[mode](0, (uint32_t)addr, pdata, len, NULL);
	} else if (type == 'l' || type == 'L') {
        ret = psram_rw_op[mode](0, (uint32_t)addr, pdata, len, NULL);
	}
    if(ret != 0) {
        CMD_PSRAM_ERR("read failed\n");
        return CMD_STATUS_FAIL;
    }

    cmd_print_uint8_array(pdata, len);
    cmd_free(pdata);
	return CMD_STATUS_OK;
}
extern uint8_t __text_start__[];

/*
 * drv psram write <b/w/l or B/W/L> <mode> <0xadd> <0xval>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_write_exec(char *cmd)
{
	int32_t ret;
	uint32_t *addr;
	char type;
	uint32_t mode;
    uint32_t len;
    uint8_t *pdata;
    uint8_t *rdata;

	ret = cmd_sscanf(cmd, "%c %d 0x%x %d", &type, &mode, (unsigned int *)&addr, &len);
	if (ret != 4 || mode > 3) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

    if (type == 'w' || type == 'W') {
        len = len << 1;
    } else if (type == 'l' || type == 'L') {
        len = len << 2;
    }

    if (len == 0) {
        CMD_PSRAM_ERR("invalid len = 0 \n");
        return CMD_STATUS_INVALID_ARG;
    }

    if(((uint32_t)addr < (uint32_t)__PSRAM_BASE) || (len > (uint32_t)__PSRAM_LENGTH) \
        || (((uint32_t)addr+len) > ((uint32_t)__PSRAM_BASE + (uint32_t)__PSRAM_LENGTH))) {
        CMD_PSRAM_ERR("Invalid param\n");
        return CMD_STATUS_INVALID_ARG;
    }

    pdata = __text_start__ + ( (uint32_t)HAL_RTC_GetFreeRunCnt() % 4096);
	if (type == 'b' || type == 'B') {
        ret = psram_rw_op[mode](1, (uint32_t)addr, (uint8_t *)pdata, len, NULL);
	} else if (type == 'w' || type == 'W') {
		ret = psram_rw_op[mode](1, (uint32_t)addr, (uint8_t *)pdata, len, NULL);
	} else if (type == 'l' || type == 'L') {
		ret = psram_rw_op[mode](1, (uint32_t)addr, (uint8_t *)pdata, len, NULL);
	}

    if(ret != 0) {
        CMD_PSRAM_ERR("write failed\n");
        return CMD_STATUS_FAIL;
    }

    rdata = (uint8_t *)cmd_malloc(len);
    if(rdata == NULL) {
        CMD_PSRAM_ERR("malloc failed\n");
		return CMD_STATUS_FAIL;
    }
    //printf("write:");
    //cmd_print_uint8_array(pdata, len);
    psram_rw_op[mode](0, (uint32_t)addr, rdata, len, NULL);
    //printf("read:");
    //cmd_print_uint8_array(rdata, len);
    if(memcmp(rdata, pdata, len) !=0 ) {
        CMD_PSRAM_ERR("memcmp failed\n");
		return CMD_STATUS_FAIL;
    } else {
        printf("write successfully\n");
    }
    cmd_free(rdata);
	return CMD_STATUS_OK;
}

/*
 * drv psram membist m=<mode>
 * eg. drv psram membist m=0
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_membist_exec(char *cmd)
{
	extern uint8_t __psram_end__[];

	int32_t cnt;
	uint32_t mode;
	uint32_t size;
	uint32_t *add = NULL;
	uint32_t i;
	uint32_t *p;
	enum cmd_status ret = CMD_STATUS_OK;

	cnt = cmd_sscanf(cmd, "m=%d", &mode);
	if (cnt != 1 || mode > 1) {
		CMD_SRAM_DBG("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	if (mode == 0) {
		size = IDCACHE_END_ADDR - (uint32_t)__psram_end__;
		while (!add && size >= 1024) {
			add = psram_malloc(size);
			size -= 1024;
			size &= ~(1024 - 1);
		}
		size += 1024;
	} else if (mode == 1) {
		add = (uint32_t *)IDCACHE_START_ADDR;
		size = IDCACHE_END_ADDR - IDCACHE_START_ADDR;
	}
	size /= 4;
	CMD_SRAM_DBG("%s,%d start:%p size:%d KB\n", __func__, __LINE__, add, size / (1024/4));
	if (!add)
		return CMD_STATUS_FAIL;
	for (i = 0, p = add; i < size; i++, p++) {
		*p = 0xA5A5A5A5;
	}
	for (i = 0, p = add; i < size; i++, p++) {
		if (*p != 0xA5A5A5A5) {
			CMD_SRAM_DBG("%s faild at[%p]:%x 0xA5A5A5A5\n",
			              __func__, p, *p);
			ret = CMD_STATUS_FAIL;
			goto out;
		}
	}
	for (i = 0, p = add; i < size; i++, p++) {
		*p = 0x5A5A5A5A;
	}
	for (i = 0, p = add; i < size; i++, p++) {
		if (*p != 0x5A5A5A5A) {
			CMD_SRAM_DBG("%s faild at[%p]:%x 0x5A5A5A5A\n",
			              __func__, p, *p);
			ret = CMD_STATUS_FAIL;
			goto out;
		}
	}
	CMD_SRAM_DBG("%s suscess, start:%p size:%d KB\n", __func__, add, size / (1024/4));
out:
	if (mode == 0) {
		psram_free(add);
	}
	return ret;
}

__CMD_SRAM_TEXT
static void cmd_psram_bench_task(void *arg)
{
	int32_t err;
	char *cmd = (char *)arg;
	uint32_t start_addr, mode;
	uint32_t cnt;
	uint32_t throuth_mb, throuth_kb;
	OS_Time_t tick_start, tick_use1, tick_use2;

	cnt = cmd_sscanf(cmd, "s=0x%x m=%d", &start_addr, &mode);
	if (cnt != 2) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		goto out;
	}

	switch (mode) {
	case 0:
	case 1:
		if (start_addr < (uint32_t)__PSRAM_Base ||
		    start_addr >= ((uint32_t)__PSRAM_Base + 8*1024*1024)) {
			CMD_PSRAM_ERR("invalid argument start_addr:%x, should >= 0x%x and < 0x%x\n",
			        start_addr, (uint32_t)__PSRAM_Base,
			        ((uint32_t)__PSRAM_Base + 8*1024*1024));
			goto out;
		}
		break;
	case 2:
	case 3:
		if (start_addr > 8*1024*1024) {
			CMD_PSRAM_ERR("invalid argument start_addr:%x, should < 8M\n", start_addr);
			goto out;
		}
		break;
	default:
		CMD_PSRAM_ERR("invalid argument mode:%d, only support:0~3\n", mode);
		goto out;
	}

	for (int i = 0; i < 1000; i++) {
		int j;
		uint32_t bench_size = 1024 * (1 << i);
		uint32_t *buf = cmd_malloc(bench_size);
		if (!buf) {
			CMD_PSRAM_ERR("%s test end for malloc buff failed!\n", __func__);
			goto out;
		}

		for (j = 0; j < bench_size/4; j++)
			buf[j] = j;

		//HAL_Dcache_DUMP_MissHit();
		HAL_Dcache_FlushCleanAll();
		tick_start = OS_GetTicks();
		err = psram_rw_op[mode](1, start_addr, (uint8_t *)buf, bench_size, NULL);
		tick_use1 = OS_GetTicks() - tick_start;
		HAL_Dcache_FlushCleanAll();
		tick_use2 = OS_GetTicks() - tick_start;
		//HAL_Dcache_DUMP_MissHit();
		if (!tick_use1)
			tick_use1 = 1;
		if (!tick_use2)
			tick_use2 = 1;
		if (err) {
			CMD_PSRAM_ERR("write err!\n");
			goto next;
		} else {
			throuth_kb = bench_size*1000/1024/(uint32_t)OS_TicksToMSecs(tick_use2);
			throuth_mb = throuth_kb/1000;
			CMD_PSRAM_DBG("%s write ok, ", __func__);
			CMD_PSRAM_DBG("%3d", bench_size/1024);
			CMD_PSRAM_DBG(" KB use:%3d ms, %3d ms, throughput:%d.%d MB/S\n",
			              (uint32_t)OS_TicksToMSecs(tick_use1), (uint32_t)OS_TicksToMSecs(tick_use2),
			              throuth_mb, throuth_kb - throuth_mb);
		}

		for (j = 0; j < bench_size/4; j++)
			buf[j] = 0;

		//HAL_Dcache_DUMP_MissHit();
		HAL_Dcache_FlushCleanAll();
		tick_start = OS_GetTicks();
		err = psram_rw_op[mode](0, start_addr, (uint8_t *)buf, bench_size, NULL);
		tick_use1 = OS_GetTicks() - tick_start;
		HAL_Dcache_FlushCleanAll();
		tick_use2 = OS_GetTicks() - tick_start;
		//HAL_Dcache_DUMP_MissHit();
		if (!tick_use1)
			tick_use1 = 1;
		if (!tick_use2)
			tick_use2 = 1;
		if (err) {
			CMD_PSRAM_ERR("read err!\n");
			goto next;
		} else {
			throuth_kb = bench_size*1000/1024/(uint32_t)OS_TicksToMSecs(tick_use2);
			throuth_mb = throuth_kb/1000;
		}

		err = 0;
		for (j = 0; j < bench_size/4; j++) {
			if (buf[j] != j) {
				err = -1;
				break;
			}
		}
		if (err) {
			CMD_PSRAM_ERR("bench_size:%d write data err:0x%x should:0x%x, idx:%d!\n",
			              bench_size, buf[j], j, j);
			j = j > 16 ? j - 16 : 0;
			print_hex_dump_words((const void *)&buf[j], 256);
			goto next;
		}
		CMD_PSRAM_DBG("%s read ok, ", __func__);
		CMD_PSRAM_DBG("%3d", bench_size/1024);
		CMD_PSRAM_DBG(" KB use:%3d ms, %3d ms, throughput:%d.%d MB/S\n",
			      (uint32_t)OS_TicksToMSecs(tick_use1), (uint32_t)OS_TicksToMSecs(tick_use2),
			      throuth_mb, throuth_kb - throuth_mb);

next:
		cmd_free(buf);
		if (err)
			break;
	}

out:
	CMD_PSRAM_DBG("%s test end\n", __func__);

	cmd_free(arg);
	OS_ThreadDelete(NULL);
}

#define CMD_ARG_LEN  64

/* drv psram bench <s=0xaddr> <m=0/1/2/3>
 * s: from 0~(2/4/8MB) if use SBUS, or from __PSRAM_Base(0x1400000)~(__PSRAM_Base+2/4/8MB) if use DBUS.
 * m: 0:DBUS CPU, 1:DBUS DMA, 2:SBUS CPU, 3:SBUS DMA
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_bench_exec(char *cmd)
{
	OS_Thread_t thread;
	char *param;
	uint32_t len;

	len = strlen(cmd);
	if (len >= CMD_ARG_LEN - 1) {
		CMD_PSRAM_ERR("should adjust CMD_ARG_LEN to %d\n", len);
		return CMD_STATUS_FAIL;
	}

	param = cmd_malloc(CMD_ARG_LEN);
	if (!param)
		return CMD_STATUS_FAIL;
	memcpy(param, cmd, len);
	param[len + 1] = 0;

	OS_ThreadSetInvalid(&thread);
	if (OS_ThreadCreate(&thread,
	                    "",
	                    cmd_psram_bench_task,
	                    param,
	                    OS_THREAD_PRIO_APP,
	                    2 * 1024) != OS_OK) {
		CMD_PSRAM_ERR("create psram bench test task failed\n");
		return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

#define READ_TEST_LEN 256
#define WRITE_TEST_LEN 256
__CMD_SRAM_DATA
static OS_Semaphore_t sem_wait;

struct psram_test_param {
	uint8_t task_idx;
	uint8_t random;
	uint8_t task_num;
	uint8_t mode;
	uint32_t time_sec;
	uint32_t addr;
	uint32_t len;
};

__CMD_SRAM_TEXT
static void cmd_psram_press_read_task(void *arg)
{
	int32_t err;
	struct psram_test_param *param = (struct psram_test_param *)arg;
	OS_Time_t tick_now = 0, tick_print = 0;
	OS_Time_t tick_end = OS_GetTicks() + OS_MSecsToTicks(param->time_sec * 1000);
	uint32_t random_sleep = param->random;
	uint8_t *buf;
	struct psram_chip *chip;
	uint32_t start_addr = param->addr;
	uint32_t round = 0;
	uint32_t loop_num = (param->len + READ_TEST_LEN - 1) / READ_TEST_LEN;
	OS_Semaphore_t dma_sem;

	chip = psram_open(0);
	if (!chip) {
		CMD_ERR("chip open failed!\n");
		goto fail;
	}

	if (param->task_idx == 0) {
		buf = cmd_malloc(READ_TEST_LEN);
		for (int i = 0; i < READ_TEST_LEN; i++) {
			buf[i] = i & 0x0FF;
		}
		CMD_DBG("%s do nothing until chip prepare ok!\n", __func__);
		for (int j = 0; j < loop_num; j++) {
			err = psram_rw_op[param->mode](1, start_addr, buf, READ_TEST_LEN, &dma_sem);
			if (err) {
				CMD_ERR("%s prepare failed!\n", __func__);
				goto out;
			}
			start_addr += READ_TEST_LEN;
		}
		CMD_DBG("%s chip prepared ok!\n", __func__);

		for (int j = 0; j < param->task_num - 1; j++)
			OS_SemaphoreRelease(&sem_wait);
		cmd_free(buf);
	} else {
		OS_SemaphoreWait(&sem_wait, OS_WAIT_FOREVER);
	}

	buf = cmd_malloc(READ_TEST_LEN);
	if (!buf) {
		CMD_ERR("%s malloc failed!\n", __func__);
		goto exit;
	}

	if (!random_sleep)
		random_sleep = 2;
	OS_MSleep(random_sleep);

	start_addr = param->addr;
	CMD_DBG("%s id:%d random:%d start_addr:0x%x len:%d\n", __func__, param->task_idx,
	        random_sleep, start_addr, param->len);

	while (tick_now < tick_end) {
		int j;

		err = psram_rw_op[param->mode](0, start_addr, buf, READ_TEST_LEN, &dma_sem);
		if (err) {
			CMD_ERR("%s read err!\n", __func__);
			goto out;
		}
		err = 0;
		for (j = 0; j < READ_TEST_LEN; j++) {
			if (buf[j] != (j & 0x0FF)) {
				err = -1;
				CMD_ERR("read:0x%x should:0x%x idx:%d\n", buf[j], j & 0x0FF, j);
				break;
			}
		}
		if (err) {
			CMD_ERR("%s len:%d read data err! at addr:0x%x count:%d\n",
			        __func__, param->len, start_addr, j);
			goto out;
		} else
			;//CMD_DBG("%s read data ok! at addr:0x%x\n",
			//         __func__, start_addr);
		OS_MSleep(random_sleep);
		tick_now = OS_GetTicks();
		if (tick_now >= tick_print + 5000) {
			CMD_DBG("%s id:%d testing... at addr:0x%x\n", __func__,
			        param->task_idx, start_addr);
			tick_print = tick_now;
		}
		start_addr += READ_TEST_LEN;
		round++;
		if (start_addr >= param->addr + param->len) {
			start_addr = param->addr;
			round = 0;
		}
	}

out:
	CMD_DBG("%s id:%d test end\n", __func__, param->task_idx);
	cmd_free(buf);
exit:
	psram_close(chip);
fail:
	cmd_free(param);
	if (param->task_idx == 0)
		OS_SemaphoreDelete(&sem_wait);
	OS_ThreadDelete(NULL);
}

__CMD_SRAM_TEXT
static void cmd_psram_press_write_task(void *arg)
{
	int32_t err;
	struct psram_test_param *param = (struct psram_test_param *)arg;
	OS_Time_t tick_now = 0, tick_print = 0;
	OS_Time_t tick_end = OS_GetTicks() + OS_MSecsToTicks(param->time_sec * 1000);
	uint32_t random_sleep = param->random;
	uint8_t *buf;
	uint32_t start_addr = param->addr;
	uint32_t round = 0;
	struct psram_chip *chip;
	uint32_t loop_num = (param->len + WRITE_TEST_LEN - 1) / WRITE_TEST_LEN;
	OS_Semaphore_t dma_sem;

	chip = psram_open(0);
	if (!chip) {
		CMD_ERR("chip open failed!\n");
		goto fail;
	}

	buf = cmd_malloc(WRITE_TEST_LEN);
	if (!buf)
		goto out;

	for (int i = 0; i < WRITE_TEST_LEN; i++)
		buf[i] = i & 0x0FF;

	if (!random_sleep)
		random_sleep = 2;
	OS_MSleep(random_sleep);
	CMD_DBG("%s id:%d random:%d start addr:0x%x len:%d\n", __func__,
	        param->task_idx, random_sleep, param->addr, param->len);

	while (tick_now < tick_end) {
		err = psram_rw_op[param->mode](1, start_addr, buf, WRITE_TEST_LEN, &dma_sem);
		if (err) {
			CMD_ERR("psram write err!\n");
			goto out;
		}
		start_addr += WRITE_TEST_LEN;
		if (start_addr >= param->addr + param->len) {
			start_addr = param->addr;
			round = 1;
		}
		OS_MSleep(random_sleep);
		tick_now = OS_GetTicks();
		if (tick_now >= tick_print + 5000) {
			CMD_DBG("%s id:%d testing... at addr:0x%x\n", __func__,
			        param->task_idx, start_addr);
			tick_print = tick_now;
		}
	}
	CMD_DBG("%s test end. round:%d start_addr:0x%x end_addr:0x%x\n", __func__,
	        round, param->addr, start_addr);

	if (!round) {
		CMD_ERR("test time too short!\n");
			goto out;
	}
	start_addr = param->addr;
	CMD_DBG("%s test checking... start_addr:0x%x loop_num:%d\n", __func__,
	        start_addr, loop_num);
	for (int i = 0; i < loop_num; i++) {
		int j;

		memset(buf, 0, WRITE_TEST_LEN);
		err = psram_rw_op[param->mode](0, start_addr, buf, WRITE_TEST_LEN, &dma_sem);
		if (err) {
			CMD_ERR("psram blocks read err!\n");
			goto out;
		}
		err = 0;
		for (j = 0; j < WRITE_TEST_LEN; j++) {
			if (buf[j] != (j & 0x0FF)) {
				err = -1;
				break;
			}
		}
		if (err) {
			CMD_ERR("read:0x%x should:0x%x idx:%d\n", buf[j], (j & 0x0FF), j);
			CMD_ERR("psram %d write data err! at addr:0x%x idx:%d\n",
			        param->len, start_addr, j);
			goto out;
		} else
			;//CMD_DBG("%s write data ok! at addr:0x%x\n",
			//         __func__, start_addr);
		start_addr += WRITE_TEST_LEN;
	}

out:
	psram_close(chip);
	CMD_DBG("%s id:%d test end\n", __func__, param->task_idx);
	cmd_free(buf);
fail:
	cmd_free(param);
	OS_ThreadDelete(NULL);
}

/*
 * drv psram press r=<threads_num> s=<start_addr> l=<length> m=<mode> w=<threads_num>
 *                 s=<start_addr> l=<length> m=<mode> t=<secons>
 * eg:
 *    drv psram press r=2 s=0x1404000 l=10240 m=0 w=2 s=0x1408000 l=10240 m=1 t=120
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_press_exec(char *cmd)
{
	OS_Thread_t thread;
	struct psram_test_param *param;
	uint32_t r_threads, w_threads;
	uint32_t r_len, w_len;
	uint32_t r_mode, w_mode;
	uint32_t cnt;
	uint32_t time_sec;
	uint32_t start_raddr, start_waddr;

	cnt = cmd_sscanf(cmd, "r=%d s=0x%x l=%d m=%d w=%d s=0x%x l=%d m=%d t=%d",
	                 &r_threads, &start_raddr, &r_len, &r_mode,
	                 &w_threads, &start_waddr, &w_len, &w_mode,
	                 &time_sec);
	if (cnt != 9) {
		CMD_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_FAIL;
	}

	OS_SemaphoreCreate(&sem_wait, 0, OS_SEMAPHORE_MAX_COUNT);

	OS_MSleep(5);

	for (uint32_t i = 0; i < r_threads; i++) {
		param = cmd_malloc(sizeof(struct psram_test_param));
		if (!param)
			return CMD_STATUS_FAIL;
		param->task_idx = i;
		param->task_num = r_threads;
		param->addr = start_raddr;
		param->len = r_len;
		param->mode = r_mode;
		param->time_sec = time_sec;
		param->random = rand() % 8 + i;
		if (!r_len || time_sec < 2) {
			CMD_ERR("%s read l=<length> should not 0 !\n", __func__);
			cmd_free(param);
			goto out;
		}
		OS_ThreadSetInvalid(&thread);
		if (OS_ThreadCreate(&thread,
		                    "",
		                    cmd_psram_press_read_task,
		                    param,
		                    OS_THREAD_PRIO_APP,
		                    2 * 1024) != OS_OK) {
			CMD_ERR("create psram press read task:%d failed\n", i);
			return CMD_STATUS_FAIL;
		}
		(void)cmd_psram_press_read_task;
		OS_MSleep(2);
	}

	for (uint32_t i = 0; i < w_threads; i++) {
		param = cmd_malloc(sizeof(struct psram_test_param));
		if (!param)
			return CMD_STATUS_FAIL;
		param->task_idx = i;
		param->task_num = w_threads;
		param->addr = start_waddr;
		param->len = w_len;
		param->mode = w_mode;
		param->time_sec = time_sec;
		param->random = rand() % 20 + i;
		if (!w_len || time_sec < 2) {
			CMD_ERR("%s write l=<length> should not 0 !\n", __func__);
			cmd_free(param);
			goto out;
		}
		OS_ThreadSetInvalid(&thread);
		if (OS_ThreadCreate(&thread,
				    "",
				    cmd_psram_press_write_task,
				    param,
				    OS_THREAD_PRIO_APP,
				    2 * 1024) != OS_OK) {
			CMD_ERR("create psram press write task:%d failed\n", i);
			return CMD_STATUS_FAIL;
		}
		OS_MSleep(2);
	}

out:
	return CMD_STATUS_OK;
}

/*
 * drv psram malloc <size> <wr>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_malloc_exec(char *cmd)
{
	void *add;
	uint32_t size, wt;
	uint32_t cnt;

	cnt = cmd_sscanf(cmd, "0x%x %d", &size, &wt);
	if (cnt != 2) {
		cnt = cmd_sscanf(cmd, "%d %d", &size, &wt);
		if (cnt != 2) {
			CMD_PSRAM_ERR("invalid argument %s\n", cmd);
			return CMD_STATUS_INVALID_ARG;
		}
	}

	add = psram_malloc(size);
	printf("%s malloc:%p\n", __func__, add);

	if (wt)
		HAL_Dcache_SetWriteThrough(0, 1, (uint32_t)add, (uint32_t)add + size);

	return CMD_STATUS_OK;
}

/*
 * drv psram free <0xaddr> <wt>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_free_exec(char *cmd)
{
	void *add;
	uint32_t cnt, wt;

	cnt = cmd_sscanf(cmd, "0x%x %d", (unsigned int *)&add, &wt);
	if (cnt != 2) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	psram_free(add);
	printf("%s free:%p\n", __func__, add);

	if (wt)
		HAL_Dcache_SetWriteThrough(0, 0, 0, 0);

	return CMD_STATUS_OK;
}

/*
 * drv psram wt <idx> <en> <0xaddr> <size>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_wt_exec(char *cmd)
{
	void *add;
	uint32_t idx, en, size;
	uint32_t cnt;

	cnt = cmd_sscanf(cmd, "%d %d 0x%x 0x%x", &idx, &en, (unsigned int *)&add, &size);
	if (cnt != 4) {
		cnt = cmd_sscanf(cmd, "%d %d 0x%x %d", &idx, &en, (unsigned int *)&add, &size);
		if (cnt != 4) {
			CMD_PSRAM_ERR("invalid argument %s\n", cmd);
			return CMD_STATUS_INVALID_ARG;
		}
	}
	if (idx > 2 || en > 1) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	HAL_Dcache_SetWriteThrough(idx, en, (uint32_t)add, (uint32_t)add + size);

	return CMD_STATUS_OK;
}

/*
 * drv psram flush_clean
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_flush_clean_exec(char *cmd)
{
	HAL_Dcache_FlushCleanAll();

	return CMD_STATUS_OK;
}

/*
 * drv psram clean <0xadd> <len>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_clean_exec(char *cmd)
{
	uint32_t add, len;
	uint32_t cnt;

	cnt = cmd_sscanf(cmd, "0x%x %d", &add, &len);
	if (cnt != 1) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}
	HAL_Dcache_Clean(add, len);

	return CMD_STATUS_OK;
}

static __inline uint16_t cmd_psram_data_checksum()
{
    return image_checksum16(__PSRAM_BASE, (uint32_t)__PSRAM_LENGTH);
}

/*
 * drv psram clean <0xadd> <len>
 */
__CMD_SRAM_TEXT
static enum cmd_status cmd_psram_check_exec(char *cmd)
{
    uint32_t ret;
    char op[16];
    uint32_t j=0;
    uint32_t *u32PsramArr = (uint32_t *)__PSRAM_BASE;
    ret = cmd_sscanf(cmd, "%s", op);
    if (ret != 1) {
		CMD_PSRAM_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

    if(cmd_strcmp(op, "upload") ==0) {
        j=0x12345678;
        for(uint32_t i=0; i<(uint32_t)__PSRAM_LENGTH/4; i++, j++) {
            u32PsramArr[i] = j;
        }
        printf("cmd_psram_data_checksum() = 0x%x\n", cmd_psram_data_checksum());
    } else if (cmd_strcmp(op, "download") ==0) {
        j=0x12345678;
        for(uint32_t i=0; i<(uint32_t)__PSRAM_LENGTH/4; i++, j++) {
            if(j != u32PsramArr[i]) {
                printf("j=0x%x != arrCacheData[i]=0x%x, i=0x%x\n", j, u32PsramArr[i], i);
            }
        }
        printf("cmd_psram_data_checksum() = 0x%x\n", cmd_psram_data_checksum());
    }  else if (cmd_strcmp(op, "checksum") ==0) {
        printf("cmd_psram_data_checksum() = 0x%x\n", cmd_psram_data_checksum());
    } else {
        return CMD_STATUS_INVALID_ARG;
    }

	return CMD_STATUS_OK;
}


static const struct cmd_data g_psram_cmds[] = {
	{ "info",       cmd_info_exec},
	{ "run",        cmd_psram_run_exec },
	{ "read",       cmd_psram_read_exec },
	{ "write",      cmd_psram_write_exec },
	{ "membist",    cmd_psram_membist_exec },
	{ "bench",      cmd_psram_bench_exec },
	{ "press",      cmd_psram_press_exec },
	{ "malloc",     cmd_psram_malloc_exec },
	{ "free",       cmd_psram_free_exec },
	{ "wt",         cmd_psram_wt_exec },
	{ "flush_clean",cmd_psram_flush_clean_exec },
    { "clean",      cmd_psram_clean_exec },
    { "check",      cmd_psram_check_exec },
};

enum cmd_status cmd_psram_exec(char *cmd)
{
	return cmd_exec(cmd, g_psram_cmds, cmd_nitems(g_psram_cmds));
}
#endif
