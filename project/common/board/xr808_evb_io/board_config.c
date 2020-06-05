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
#include "common/board/board_debug.h"
#include "common/board/board_common.h"
#include "board_config.h"

/* Note: Default SWD pins are multiplexing with flash pins.
 *       Using/Enabling SWD may cause flash read/write error.
 */
#define BOARD_SWD_EN		PRJCONF_SWD_EN

static const GPIO_PinMuxParam g_pinmux_uart0[] = {
	{ GPIO_PORT_B, GPIO_PIN_0,  { GPIOB_P0_F2_UART0_TX,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } }, /* TX */
	{ GPIO_PORT_B, GPIO_PIN_1,  { GPIOB_P1_F2_UART0_RX,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } }, /* RX */
};

static const GPIO_PinMuxParam g_pinmux_uart1[] = {
	{ GPIO_PORT_A, GPIO_PIN_13, { GPIOA_P13_F5_UART1_TX,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } }, /* TX */
	{ GPIO_PORT_A, GPIO_PIN_14, { GPIOA_P14_F5_UART1_RX,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } }, /* RX */
	//{ GPIO_PORT_B, GPIO_PIN_6, { GPIOB_P6_F4_UART1_CTS,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_DOWN } }, /* CTS */
	//{ GPIO_PORT_B, GPIO_PIN_7, { GPIOB_P7_F4_UART1_RTS,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_DOWN } }, /* RTS */
};

static const GPIO_PinMuxParam g_pinmux_uart2[] = {
	{ GPIO_PORT_A, GPIO_PIN_22, { GPIOA_P22_F2_UART2_TX,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } }, /* TX */
	{ GPIO_PORT_A, GPIO_PIN_21, { GPIOA_P21_F2_UART2_RX,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } }, /* RX */
	{ GPIO_PORT_A, GPIO_PIN_20, { GPIOA_P20_F2_UART2_CTS,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_DOWN } }, /* CTS */
	{ GPIO_PORT_A, GPIO_PIN_19, { GPIOA_P19_F2_UART2_RTS,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_DOWN } }, /* RTS */
};

__xip_rodata
static const GPIO_PinMuxParam g_pinmux_irrx[] = {
	{ GPIO_PORT_A, GPIO_PIN_17, { GPIOA_P17_F3_IR_RX,     GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};

__xip_rodata
static const GPIO_PinMuxParam g_pinmux_irtx[] = {
	{ GPIO_PORT_A, GPIO_PIN_18, { GPIOA_P18_F3_IR_TX,     GPIO_DRIVING_LEVEL_1, GPIO_PULL_DOWN } },
};

__xip_rodata
static const GPIO_PinMuxParam g_pinmux_i2c0[] = {
	{ GPIO_PORT_A, GPIO_PIN_19,  { GPIOA_P19_F3_I2C0_SCL,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } }, /* SCL */
	{ GPIO_PORT_A, GPIO_PIN_20,  { GPIOA_P20_F3_I2C0_SDA,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } }, /* SDA */
};

__xip_rodata
static const GPIO_PinMuxParam g_pinmux_i2c1[] = {
	{ GPIO_PORT_A, GPIO_PIN_0, { GPIOA_P0_F4_I2C1_SCL,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } }, /* SCL */
	{ GPIO_PORT_A, GPIO_PIN_1, { GPIOA_P1_F4_I2C1_SDA,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } }, /* SDA */
};

__xip_rodata
static const GPIO_PinMuxParam g_pinmux_adc[] = {
	{ GPIO_PORT_A, GPIO_PIN_10, { GPIOA_P10_F2_ADC_CH0, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_11, { GPIOA_P11_F2_ADC_CH1, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_12, { GPIOA_P12_F2_ADC_CH2, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_13, { GPIOA_P13_F2_ADC_CH3, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_14, { GPIOA_P14_F2_ADC_CH4, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
};

__xip_rodata
static const GPIO_PinMuxParam g_pinmux_spi1[] = {
	{ GPIO_PORT_A, GPIO_PIN_19,  { GPIOA_P19_F5_SPI1_MOSI,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_20,  { GPIOA_P20_F5_SPI1_MISO,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_21,  { GPIOA_P21_F5_SPI1_CLK,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
};

__xip_rodata
static const GPIO_PinMuxParam g_pinmux_spi1_cs0[] = {
	{ GPIO_PORT_A, GPIO_PIN_22,  { GPIOA_P22_F5_SPI1_CS0,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};

static const GPIO_PinMuxParam g_pinmux_flashc[] = {
	{ GPIO_PORT_B, GPIO_PIN_4,  { GPIOB_P4_F5_FLASH_MOSI, GPIO_DRIVING_LEVEL_3, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_5,  { GPIOB_P5_F5_FLASH_MISO, GPIO_DRIVING_LEVEL_3, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_6,  { GPIOB_P6_F5_FLASH_CS,   GPIO_DRIVING_LEVEL_3, GPIO_PULL_UP   } },
	{ GPIO_PORT_B, GPIO_PIN_7,  { GPIOB_P7_F5_FLASH_CLK,  GPIO_DRIVING_LEVEL_3, GPIO_PULL_NONE } },
};

#if BOARD_SWD_EN
static const GPIO_PinMuxParam g_pinmux_swd[] = {
	{ GPIO_PORT_B, GPIO_PIN_2,  { GPIOB_P2_F2_SWD_TMS,    GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP   } },
	{ GPIO_PORT_B, GPIO_PIN_3,  { GPIOB_P3_F2_SWD_TCK,    GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP   } },
	/*JTAG PinMux*/
    //{ GPIO_PORT_B, GPIO_PIN_0,  { GPIOB_P0_F3_JTAG_TMS,    GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP   } },
    //{ GPIO_PORT_B, GPIO_PIN_1,  { GPIOB_P1_F3_JTAG_TCK,    GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP   } },
    //{ GPIO_PORT_B, GPIO_PIN_2,  { GPIOB_P2_F3_JTAG_TD0,    GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP   } },
    //{ GPIO_PORT_B, GPIO_PIN_3,  { GPIOB_P3_F3_JTAG_TDI,    GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP   } },
};
#endif

/* flash */
static const FlashBoardCfg g_flash_cfg[] = {
	{
		.type = FLASH_DRV_FLASHC,
		.mode = FLASH_READ_DUAL_O_MODE,
		.flashc.clk = (96 * 1000 * 1000),
	},
};

__xip_rodata
static const GPIO_PinMuxParam g_pinmux_pwm[] = {
	//{ GPIO_PORT_A, GPIO_PIN_19, { GPIOA_P19_F4_PWM0_ECT0, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	//{ GPIO_PORT_A, GPIO_PIN_20, { GPIOA_P20_F4_PWM1_ECT1, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	//{ GPIO_PORT_A, GPIO_PIN_21, { GPIOA_P21_F4_PWM2_ECT2, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_10, { GPIOA_P10_F3_PWM2_ECT2, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_11, { GPIOA_P11_F3_PWM3_ECT3, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_12, { GPIOA_P12_F3_PWM4_ECT4, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_13, { GPIOA_P13_F3_PWM5_ECT5, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_14, { GPIOA_P14_F3_PWM6_ECT6, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	//{ GPIO_PORT_B, GPIO_PIN_3, { GPIOB_P3_F4_PWM7_ECT7, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
};

struct board_pinmux_info {
	const GPIO_PinMuxParam *pinmux;
	uint32_t count;
};

#define BOARD_PINMUX_INFO_MAX	2

static HAL_Status board_get_pinmux_info(uint32_t major, uint32_t minor, uint32_t param,
                                        struct board_pinmux_info info[])
{
	HAL_Status ret = HAL_OK;

	switch (major) {
	case HAL_DEV_MAJOR_UART:
		if (minor == UART0_ID) {
			info[0].pinmux = g_pinmux_uart0;
			info[0].count = HAL_ARRAY_SIZE(g_pinmux_uart0);
		} else if (minor == UART1_ID) {
			info[0].pinmux = g_pinmux_uart1;
			info[0].count = HAL_ARRAY_SIZE(g_pinmux_uart1);
		} else if (minor == UART2_ID) {
			info[0].pinmux = g_pinmux_uart2;
			info[0].count = HAL_ARRAY_SIZE(g_pinmux_uart2);
		} else {
			ret = HAL_INVALID;
		}
		break;
	case HAL_DEV_MAJOR_I2C:
		if (minor == I2C0_ID) {
			info[0].pinmux = g_pinmux_i2c0;
			info[0].count = HAL_ARRAY_SIZE(g_pinmux_i2c0);
		} else if (minor == I2C1_ID) {
			info[0].pinmux = g_pinmux_i2c1;
			info[0].count = HAL_ARRAY_SIZE(g_pinmux_i2c1);
		} else {
			ret = HAL_INVALID;
		}
		break;
	case HAL_DEV_MAJOR_SPI:
		if (minor == SPI1) {
			info[0].pinmux = g_pinmux_spi1;
			info[0].count = HAL_ARRAY_SIZE(g_pinmux_spi1);
			switch (param) {
			case SPI_TCTRL_SS_SEL_SS0:
				info[1].pinmux = g_pinmux_spi1_cs0;
				info[1].count = HAL_ARRAY_SIZE(g_pinmux_spi1_cs0);
				break;
			default:
				ret = HAL_INVALID;
				break;
			}
		} else {
			ret = HAL_INVALID;
		}
		break;
	case HAL_DEV_MAJOR_IRRX:
		info[0].pinmux = g_pinmux_irrx;
		info[0].count = HAL_ARRAY_SIZE(g_pinmux_irrx);
		break;
	case HAL_DEV_MAJOR_IRTX:
		info[0].pinmux = g_pinmux_irtx;
		info[0].count = HAL_ARRAY_SIZE(g_pinmux_irtx);
		break;
	case HAL_DEV_MAJOR_ADC:
		if (minor < HAL_ARRAY_SIZE(g_pinmux_adc)) {
			info[0].pinmux = &g_pinmux_adc[minor];
			info[0].count = 1;
		} else {
			ret = HAL_INVALID;
		}
		break;
	case HAL_DEV_MAJOR_PWM:
		if (minor < HAL_ARRAY_SIZE(g_pinmux_pwm)) {
			info[0].pinmux = &g_pinmux_pwm[minor];
			info[0].count = 1;
		} else if (minor != ADC_CHANNEL_VBAT) {
			ret = HAL_INVALID;
		}
		break;
	case HAL_DEV_MAJOR_FLASHC:
		if (board_get_flashc_sip_pinmux_cfg(&info[0].pinmux,
		                                    &info[0].count) != HAL_OK) {
			info[0].pinmux = g_pinmux_flashc;
			info[0].count = HAL_ARRAY_SIZE(g_pinmux_flashc);
		}
		break;
#ifdef __CONFIG_PSRAM
	case HAL_DEV_MAJOR_PSRAM:
		board_get_psram_sip_pinmux_cfg(&info[0].pinmux, &info[0].count);
		break;
#endif
	case HAL_DEV_MAJOR_SDC:
		if (minor == SDC1) {
			info[0].pinmux = NULL;
			info[0].count = 0;
			info[1].pinmux = NULL;
			info[1].count = 0;
		} else {
			ret = HAL_INVALID;
		}
		break;
#if BOARD_SWD_EN
	case HAL_DEV_MAJOR_SWD:
		info[0].pinmux = g_pinmux_swd;
		info[0].count = HAL_ARRAY_SIZE(g_pinmux_swd);
		break;
#endif
	default:
		BOARD_ERR("unknow major %u\n", major);
		ret = HAL_INVALID;
	}

	return ret;
}

static HAL_Status board_get_cfg(uint32_t major, uint32_t minor, uint32_t param)
{
	HAL_Status ret = HAL_OK;

	switch (major) {
	case HAL_DEV_MAJOR_SDC:
		if (minor == SDC1) {
			*((HAL_SDCGPIOCfg **)param) = NULL;
		} else {
			*((HAL_SDCGPIOCfg **)param) = NULL;
			ret = HAL_INVALID;
		}
		break;
	case HAL_DEV_MAJOR_FLASH:
		if (minor <= (sizeof(g_flash_cfg) / sizeof(FlashBoardCfg)))
			*((FlashBoardCfg **)param) = (FlashBoardCfg *)&g_flash_cfg[minor];
		else
			*((FlashBoardCfg **)param) = NULL;
		break;
	default:
		BOARD_ERR("unknow major %u\n", major);
		ret = HAL_INVALID;
	}

	return ret;
}

#ifdef __CONFIG_PM
void board_flash_pinmux_deinit(const GPIO_PinMuxParam *pinmux, uint32_t count)
{
	uint32_t i;

	for (i = 0; i < count; ++i) {
		/* reset driving and mode, but keep pull to avoid current leakage */
		HAL_GPIO_SetDriving(pinmux[i].port, pinmux[i].pin, GPIO_DRIVING_LEVEL_1);
		HAL_GPIO_SetMode(pinmux[i].port, pinmux[i].pin, GPIOx_Pn_F7_DISABLE);
	}
}
#endif

HAL_Status board_ioctl(HAL_BoardIoctlReq req, uint32_t param0, uint32_t param1)
{
	HAL_Status ret = HAL_OK;
	uint32_t major, minor, i;
	struct board_pinmux_info info[BOARD_PINMUX_INFO_MAX];

	switch (req) {
	case HAL_BIR_PINMUX_INIT:
	case HAL_BIR_PINMUX_DEINIT:
		memset(info, 0, sizeof(info));
		major = HAL_DEV_MAJOR((HAL_Dev_t)param0);
		minor = HAL_DEV_MINOR((HAL_Dev_t)param0);
		ret = board_get_pinmux_info(major, minor, param1, info);
		if (major == HAL_DEV_MAJOR_SDC) {
			if (minor == SDC1) {
				; /* do nothing */
			}
#ifdef __CONFIG_PM
		} else if (major == HAL_DEV_MAJOR_FLASHC && req == HAL_BIR_PINMUX_DEINIT) {
			for (i = 0; i < BOARD_PINMUX_INFO_MAX; ++i) {
				if (info[i].pinmux != NULL && info[i].count != 0) {
					board_flash_pinmux_deinit(info[i].pinmux, info[i].count);
				} else {
					break;
				}
			}
#endif
		} else {
			for (i = 0; i < BOARD_PINMUX_INFO_MAX; ++i) {
				if (info[i].pinmux != NULL && info[i].count != 0) {
					board_pinmux_cfg(req, info[i].pinmux, info[i].count);
				} else {
					break;
				}
			}
		}
		break;
	case HAL_BIR_CHIP_CLOCK_INIT:
		board_chip_clock_init();
		break;
	case HAL_BIR_GET_CFG:
		major = HAL_DEV_MAJOR((HAL_Dev_t)param0);
		minor = HAL_DEV_MINOR((HAL_Dev_t)param0);
		ret = board_get_cfg(major, minor, param1);
		break;
	default:
		BOARD_ERR("req %d not suppport\n", req);
		ret = HAL_INVALID;
		break;
	}

	if (ret != HAL_OK) {
		BOARD_ERR("req %d, param0 %#x, param1 %#x, ret %d\n", req, param0, param1, ret);
	}

	return ret;
}
