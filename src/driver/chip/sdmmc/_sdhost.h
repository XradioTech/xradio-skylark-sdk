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

#ifndef _DRIVER_CHIP_SDMMC__SDHOST_H_
#define _DRIVER_CHIP_SDMMC__SDHOST_H_

#include "driver/chip/sdmmc/hal_sdhost.h"
#include "pm/pm.h"

#include "kernel/os/os_semaphore.h"
#include "kernel/os/os_mutex.h"

#define SDC_Semaphore OS_Semaphore_t
#define SDC_Mutex OS_Mutex_t
#define SDC_Timer OS_Timer_t

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_SDC_OS_USED
#define CONFIG_SD_PM

/*
 * the max length of buffer which IDMA description supported is 8192,
 * transport data by several IDMA descriptions if data lenght more than 8192,
 * and the max number IDMA description support is 1024.
 * which meas the mas length transport data is 1024 * 8192 = 8MB in a signal transfer.
 */
#define SDXC_MAX_TRANS_LEN              (1 << 18)       /* max len is 256K */
#define SDXC_DES_NUM_SHIFT              (13)
#define SDXC_DES_BUFFER_MAX_LEN         (1 << SDXC_DES_NUM_SHIFT)  /* 8192 == 1<<13; */
#define SDXC_MAX_DES_NUM                (SDXC_MAX_TRANS_LEN >> SDXC_DES_NUM_SHIFT)      /* 2 is the least */
#define SDXC_DES_MODE                   (0)             /* 0-chain mode, 1-fix length skip */

/* IDMC structure */
typedef struct {
	uint32_t config;

#define SDXC_IDMAC_DES0_DIC             BIT(1) /* disable interrupt on completion */
#define SDXC_IDMAC_DES0_LD              BIT(2) /* 1-this data buffer is the last buffer */
#define SDXC_IDMAC_DES0_FD              BIT(3) /* 1-data buffer is the first buffer, 0-data buffer contained in the next descriptor is the first data buffer */
#define SDXC_IDMAC_DES0_CH              BIT(4) /* 1-the 2nd address in the descriptor is the next descriptor address */
#define SDXC_IDMAC_DES0_ER              BIT(5) /* 1-last descriptor flag when using dual data buffer in descriptor */
#define SDXC_IDMAC_DES0_CES             BIT(30) /* transfer error flag */
#define SDXC_IDMAC_DES0_OWN             BIT(31) /* des owner:1-idma owns it, 0-host owns it */

	uint32_t data_buf1_sz   :16,
	         data_buf2_sz   :16;

	uint32_t buf_addr_ptr1;
	uint32_t buf_addr_ptr2;
} smc_idma_des;

typedef enum
{
	SDC_STATE_RESET        = 0x00, /* Peripheral is not yet Initialized */
	SDC_STATE_READY        = 0x02, /* Peripheral Initialized and ready for use */
	SDC_STATE_BUSY         = 0x04, /* An internal process is ongoing */
	SDC_STATE_ERROR        = 0x08  /* Error */
} SDC_StateTypeDef;

struct __mci_ctrl_regs {
	uint32_t gctrl;
	uint32_t clkc;
	uint32_t timeout;
	uint32_t buswid;
	uint32_t waterlvl;
	uint32_t funcsel;
	uint32_t idmacc;
};

struct mmc_bus_ops {
	int (*suspend)(struct mmc_host *);
	int (*resume)(struct mmc_host *);
};

struct mmc_host {
	volatile void   *reg_base;      /* Mapped address */
	uint8_t         sdc_id;
	uint8_t         pin_ref;
	uint16_t        ref;
	uint16_t        debug_mask;
	uint16_t        dma_use;
	struct mmc_card *card;
#ifdef CONFIG_SDC_SUPPORT_1V8
	uint32_t        voltage;
#define SDC_WOLTAGE_3V3 (0)
#define SDC_WOLTAGE_1V8 (1)
#define SDC_WOLTAGE_1V2 (2)
#define SDC_WOLTAGE_OFF (3)
#define SDC_WOLTAGE_ON  (4)
	uint32_t        voltage_switching;
#endif
	volatile uint32_t present;
	uint16_t        power_on;
	uint16_t        suspend;
	uint32_t        int_err;       /* for Interrupt Controller */

	uint32_t        int_use;       /* Control */
	uint32_t        int_sum;       /* interrupt summary */
	uint16_t        trans_done;
	uint16_t        dma_done;
	uint32_t        buswidth;      /* current card bus width */
	uint32_t        blkcnt;

	/* NOTE: define idma_des here for aligned8! */
	smc_idma_des    idma_des[SDXC_MAX_DES_NUM] __attribute__ ((aligned (8)));
	smc_idma_des    *dma_hdle;

	/* host specific block data */
	uint32_t        max_seg_size;   /* see blk_queue_max_segment_size */
	uint32_t        max_segs;       /* see blk_queue_max_segments */
	uint32_t        max_req_size;   /* maximum number of bytes in one req */
	uint32_t        max_blk_size;   /* maximum size of one mmc block */
	uint32_t        max_blk_count;  /* maximum number of blocks in one req */
	uint32_t        ocr_avail;

#define MMC_VDD_165_195         0x00000080      /* VDD voltage 1.65 - 1.95 */
#define MMC_VDD_20_21           0x00000100      /* VDD voltage 2.0 ~ 2.1 */
#define MMC_VDD_21_22           0x00000200      /* VDD voltage 2.1 ~ 2.2 */
#define MMC_VDD_22_23           0x00000400      /* VDD voltage 2.2 ~ 2.3 */
#define MMC_VDD_23_24           0x00000800      /* VDD voltage 2.3 ~ 2.4 */
#define MMC_VDD_24_25           0x00001000      /* VDD voltage 2.4 ~ 2.5 */
#define MMC_VDD_25_26           0x00002000      /* VDD voltage 2.5 ~ 2.6 */
#define MMC_VDD_26_27           0x00004000      /* VDD voltage 2.6 ~ 2.7 */
#define MMC_VDD_27_28           0x00008000      /* VDD voltage 2.7 ~ 2.8 */
#define MMC_VDD_28_29           0x00010000      /* VDD voltage 2.8 ~ 2.9 */
#define MMC_VDD_29_30           0x00020000      /* VDD voltage 2.9 ~ 3.0 */
#define MMC_VDD_30_31           0x00040000      /* VDD voltage 3.0 ~ 3.1 */
#define MMC_VDD_31_32           0x00080000      /* VDD voltage 3.1 ~ 3.2 */
#define MMC_VDD_32_33           0x00100000      /* VDD voltage 3.2 ~ 3.3 */
#define MMC_VDD_33_34           0x00200000      /* VDD voltage 3.3 ~ 3.4 */
#define MMC_VDD_34_35           0x00400000      /* VDD voltage 3.4 ~ 3.5 */
#define MMC_VDD_35_36           0x00800000      /* VDD voltage 3.5 ~ 3.6 */

	uint32_t        caps;                   /* Host capabilities */

#define MMC_CAP_4_BIT_DATA      (1 << 0)        /* Can the host do 4 bit transfers */
#define MMC_CAP_MMC_HIGHSPEED   (1 << 1)        /* Can do MMC high-speed timing */
#define MMC_CAP_SD_HIGHSPEED    (1 << 2)        /* Can do SD high-speed timing */
#define MMC_CAP_SDIO_IRQ        (1 << 3)        /* Can signal pending SDIO IRQs */
#define MMC_CAP_SPI             (1 << 4)        /* Talks only SPI protocols */
#define MMC_CAP_NEEDS_POLL      (1 << 5)        /* Needs polling for card-detection */
#define MMC_CAP_8_BIT_DATA      (1 << 6)        /* Can the host do 8 bit transfers */

#define MMC_CAP_NONREMOVABLE    (1 << 8)        /* Nonremovable e.g. eMMC */
#define MMC_CAP_WAIT_WHILE_BUSY (1 << 9)        /* Waits while card is busy */
#define MMC_CAP_ERASE           (1 << 10)       /* Allow erase/trim commands */
#define MMC_CAP_1_8V_DDR        (1 << 11)       /* can support */
                                                /* DDR mode at 1.8V */
#define MMC_CAP_1_2V_DDR        (1 << 12)       /* can support */
                                                /* DDR mode at 1.2V */
#define MMC_CAP_POWER_OFF_CARD  (1 << 13)       /* Can power off after boot */
#define MMC_CAP_BUS_WIDTH_TEST  (1 << 14)       /* CMD14/CMD19 bus width ok */
#define MMC_CAP_UHS_SDR12       (1 << 15)       /* Host supports UHS SDR12 mode */
#define MMC_CAP_UHS_SDR25       (1 << 16)       /* Host supports UHS SDR25 mode */
#define MMC_CAP_UHS_SDR50       (1 << 17)       /* Host supports UHS SDR50 mode */
#define MMC_CAP_UHS_SDR104      (1 << 18)       /* Host supports UHS SDR104 mode */
#define MMC_CAP_UHS_DDR50       (1 << 19)       /* Host supports UHS DDR50 mode */
#define MMC_CAP_SET_XPC_330     (1 << 20)       /* Host supports >150mA current at 3.3V */
#define MMC_CAP_SET_XPC_300     (1 << 21)       /* Host supports >150mA current at 3.0V */
#define MMC_CAP_SET_XPC_180     (1 << 22)       /* Host supports >150mA current at 1.8V */
#define MMC_CAP_DRIVER_TYPE_A   (1 << 23)       /* Host supports Driver Type A */
#define MMC_CAP_DRIVER_TYPE_C   (1 << 24)       /* Host supports Driver Type C */
#define MMC_CAP_DRIVER_TYPE_D   (1 << 25)       /* Host supports Driver Type D */
#define MMC_CAP_MAX_CURRENT_200 (1 << 26)       /* Host max current limit is 200mA */
#define MMC_CAP_MAX_CURRENT_400 (1 << 27)       /* Host max current limit is 400mA */
#define MMC_CAP_MAX_CURRENT_600 (1 << 28)       /* Host max current limit is 600mA */
#define MMC_CAP_MAX_CURRENT_800 (1 << 29)       /* Host max current limit is 800mA */
#define MMC_CAP_CMD23           (1 << 30)       /* CMD23 supported. */
#define MMC_CAP_HW_RESET        (1 << 31)       /* Hardware reset */

        uint32_t        caps2;                  /* More host capabilities */

#define MMC_CAP2_BOOTPART_NOACC (1 << 0)        /* Boot partition no access */
#define MMC_CAP2_CACHE_CTRL     (1 << 1)        /* Allow cache control */
#define MMC_CAP2_POWEROFF_NOTIFY (1 << 2)       /* Notify poweroff supported */
#define MMC_CAP2_NO_MULTI_READ  (1 << 3)        /* Multiblock reads don't work */
#define MMC_CAP2_NO_SLEEP_CMD   (1 << 4)        /* Don't allow sleep command */
#define MMC_CAP2_HS200_1_8V_SDR (1 << 5)        /* can support */
#define MMC_CAP2_HS200_1_2V_SDR (1 << 6)        /* can support */
#define MMC_CAP2_HS200          (MMC_CAP2_HS200_1_8V_SDR | MMC_CAP2_HS200_1_2V_SDR)
#define MMC_CAP2_BROKEN_VOLTAGE (1 << 7)        /* Use the broken voltage */
#define MMC_CAP2_DETECT_ON_ERR  (1 << 8)        /* On I/O err check card removal */
#define MMC_CAP2_HC_ERASE_SZ    (1 << 9)        /* High-capacity erase size */

#ifdef CONFIG_SDC_OS_USED
	SDC_Semaphore           lock;
	SDC_Mutex               thread_lock;
#ifdef CONFIG_DETECT_CARD
	SDC_Timer               cd_timer;
#endif
#ifdef CONFIG_SDC_EXCLUSIVE_HOST
	SDC_Semaphore           exclusive_lock; /* lock for claim and bus ops */
#endif
#endif

	//uint8_t                 bus_width;         /* data bus width */
	uint32_t                clk;

#define MMC_BUS_WIDTH_1         0
#define MMC_BUS_WIDTH_4         2
#define MMC_BUS_WIDTH_8         3

	struct mmc_request      *mrq;

#define SDC_WAIT_NONE                   BIT(0)
#define SDC_WAIT_CMD_DONE               BIT(1)
#define SDC_WAIT_DATA_OVER              BIT(2)
#define SDC_WAIT_AUTOCMD_DONE           BIT(3)
#define SDC_WAIT_IDMA_DONE              BIT(4)
#define SDC_WAIT_IDMA_ERR               BIT(5)
#define SDC_WAIT_ERROR                  BIT(6)
#define SDC_WAIT_RXDATA_OVER            (SDC_WAIT_DATA_OVER|SDC_WAIT_IDMA_DONE)
#define SDC_WAIT_RXAUTOCMD_DONE         (SDC_WAIT_AUTOCMD_DONE|SDC_WAIT_IDMA_DONE)
#define SDC_WAIT_SWITCH1V8              BIT(7)
#define SDC_WAIT_FINALIZE               BIT(8)
	volatile uint32_t       smc_cmd;

	uint32_t                wait;
#ifdef CONFIG_SDIO_IRQ_SUPPORT
	uint32_t                sdio_int;
#endif
#ifdef CONFIG_SD_PM
	struct __mci_ctrl_regs  regs_back;
	const struct mmc_bus_ops *bus_ops;      /* current bus driver */
	uint32_t                pm_flags;       /* requested pm features */
	uint32_t                pm_caps;        /* supported pm features */
#endif
#ifdef __CONFIG_ARCH_APP_CORE
	SDC_InitTypeDef         param;
	GPIO_Port               cd_port;
	GPIO_Pin                cd_pin;
	uint16_t                cd_delay;       /* delay interval (in ms) to wait power stable */
	uint8_t                 wait_voltage_stable;	/* card voltage stable*/
	GPIO_PinState           cd_pin_present_val;
#endif
#ifdef CONFIG_SDC_READONLY_USED
	uint32_t                read_only;
	GPIO_PinMuxParam        ro_gpio;
#endif
	SDC_StateTypeDef        State;
};

/* registers define */
#define SMC0_BASE                       (SDC0_BASE)
#if (__CONFIG_CHIP_ARCH_VER == 2)
#define SMC1_BASE                       (SDC1_BASE)
#endif
#define SDXC_REG_GCTRL                  (0x00)      /* SMC Global Control Register */
#define SDXC_REG_CLKCR                  (0x04)      /* SMC Clock Control Register */

/* Clock control */
#define SDXC_CardClkOn                  (0x1U << 16)
#define SDXC_LowPowerOn                 (0x1U << 17)

extern int32_t __mci_program_clk(struct mmc_host *host);
extern int32_t __mci_update_clock(struct mmc_host *host, uint32_t cclk);
extern int32_t HAL_SDC_Claim_Host(struct mmc_host *host);
extern void HAL_SDC_Release_Host(struct mmc_host *host);

#endif /* _DRIVER_CHIP_SDMMC__SDHOST_H_ */
