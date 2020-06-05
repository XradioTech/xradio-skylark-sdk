/**
  * @file  hal_i2s.h
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

#ifndef _XRADIO_I2S_H_
#define _XRADIO_I2S_H_


#ifdef __cplusplus
 extern "C" {
#endif


/*** XRADIO I2S Register Define***/

#define DA_CTL			0x00
#define DA_FMT0			0x04
#define DA_FMT1			0x08
#define DA_ISTA			0x0C
#define DA_RXFIFO		0x10
#define DA_FCTL			0x14
#define DA_FSTA			0x18
#define DA_INT			0x1C
#define DA_TXFIFO		0x20
#define DA_CLKD			0x24
#define DA_TXCNT		0x28
#define DA_RXCNT		0x2C
#define DA_CHCFG		0x30
#define DA_TX0CHSEL		0x34
#define DA_TX1CHSEL		0x38
#define DA_TX2CHSEL		0x3C
#define DA_TX3CHSEL		0x40
#define DA_TX0CHMAP		0x44
#define DA_TX1CHMAP		0x48
#define DA_TX2CHMAP		0x4C
#define DA_TX3CHMAP		0x50
#define DA_RXCHSEL		0x54
#define DA_RXCHMAP		0x58



/*** XRADIO I2S Register Bit Define***/

//DA_CTL
#define I2S_BCLK_OUT_BIT		18
#define I2S_LRCK_OUT_BIT		17
#define I2S_SDO0_EN_BIT			8
#define I2S_OUT_MUTE_BIT		6
#define I2S_MODE_SEL_BIT		4
#define I2S_LOOP_EN_BIT			3
#define I2S_TX_EN_BIT			2
#define I2S_RX_EN_BIT			1
#define I2S_GEN_BIT				0

//DA_FMT0
#define I2S_LRCK_WIDTH_BIT		30
#define I2S_LRCK_POLARITY_BIT	19
#define I2S_LRCK_PERIOD_BIT		8
#define I2S_BCLK_POLARITY_BIT	7
#define I2S_SAMPLE_RES_BIT		4
#define I2S_EDGE_TRANSFER_BIT	3
#define I2S_SLOT_WIDTH_BIT		0

//DA_FMT1
#define I2S_RX_MLS_BIT			7
#define I2S_TX_MLS_BIT			6
#define I2S_SEXT_BIT			4
#define I2S_RX_PDM_BIT			2
#define I2S_TX_PDM_BIT			0

//DA_ISTA
#define I2S_TXU_INT_BIT			6
#define I2S_TXO_INT_BIT			5
#define I2S_TXE_INT_BIT			4
#define I2S_RXU_INT_BIT			2
#define I2S_RXO_INT_BIT			1
#define I2S_RXA_INT_BIT			0

//DA_FCTL
#define I2S_HUB_EN_BIT			31
#define I2S_FTX_BIT				25
#define I2S_FRX_BIT				24
#define I2S_TXTL_BIT			12
#define I2S_RXTL_BIT			4
#define I2S_TXIM_BIT			2
#define I2S_RXOM_BIT			0

// DA_FSTA
#define I2S_TXE_BIT				28
#define I2S_TXE_CNT_BIT			16
#define I2S_RXA_BIT				8
#define I2S_RXA_CNT_BIT			0

//DA_INT
#define I2S_TXE_DRQ_EN_BIT		7
#define I2S_TXUI_EN_BIT			6
#define I2S_TXOI_EN_BIT			5
#define I2S_TXEI_EN_BIT			4
#define I2S_RXA_DRQ_EN_BIT		3
#define I2S_RXUI_EN_BIT			2
#define I2S_RXOI_EN_BIT			1
#define I2S_RXAI_EN_BIT			0

//DA_CLKD
#define I2S_MCLK_OUT_EN_BIT		8
#define I2S_BCLK_DIV_BIT		4
#define I2S_MCLK_DIV_BIT		0

//DA_CHCFG
#define I2S_TX_SLOT_HIZ_BIT		9
#define I2S_TX_STATE_BIT		8
#define I2S_RX_SLOT_NUM			4
#define I2S_TX_SLOT_NUM			0

//DA_TX0CHSEL
#define I2S_TX0_OFFSET_BIT		12
#define I2S_TX0_CHEN_BIT		4
#define I2S_TX0_CHSEL_BIT		0

//DA_TX1CHSEL
#define I2S_TX1_OFFSET_BIT		12
#define I2S_TX1_CHEN_BIT		4
#define I2S_TX1_CHSEL_BIT		0

//DA_TX2CHSEL
#define I2S_TX2_OFFSET_BIT		12
#define I2S_TX2_CHEN_BIT		4
#define I2S_TX2_CHSEL_BIT		0

//DA_TX3CHSEL
#define I2S_TX3_OFFSET_BIT		12
#define I2S_TX3_CHEN_BIT		4
#define I2S_TX3_CHSEL_BIT		0

//DA_TX0CHMAP
#define I2S_TX0_CH7_MAP_BIT		28
#define I2S_TX0_CH6_MAP_BIT		24
#define I2S_TX0_CH5_MAP_BIT		20
#define I2S_TX0_CH4_MAP_BIT		16
#define I2S_TX0_CH3_MAP_BIT		12
#define I2S_TX0_CH2_MAP_BIT		8
#define I2S_TX0_CH1_MAP_BIT		4
#define I2S_TX0_CH0_MAP_BIT		0

//DA_TX1CHMAP
#define I2S_TX1_CH7_MAP_BIT		28
#define I2S_TX1_CH6_MAP_BIT		24
#define I2S_TX1_CH5_MAP_BIT		20
#define I2S_TX1_CH4_MAP_BIT		16
#define I2S_TX1_CH3_MAP_BIT		12
#define I2S_TX1_CH2_MAP_BIT		8
#define I2S_TX1_CH1_MAP_BIT		4
#define I2S_TX1_CH0_MAP_BIT		0

//DA_TX2CHMAP
#define I2S_TX2_CH7_MAP_BIT		28
#define I2S_TX2_CH6_MAP_BIT		24
#define I2S_TX2_CH5_MAP_BIT		20
#define I2S_TX2_CH4_MAP_BIT		16
#define I2S_TX2_CH3_MAP_BIT		12
#define I2S_TX2_CH2_MAP_BIT		8
#define I2S_TX2_CH1_MAP_BIT		4
#define I2S_TX2_CH0_MAP_BIT		0

//DA_TX3CHMAP
#define I2S_TX3_CH7_MAP_BIT		28
#define I2S_TX3_CH6_MAP_BIT		24
#define I2S_TX3_CH5_MAP_BIT		20
#define I2S_TX3_CH4_MAP_BIT		16
#define I2S_TX3_CH3_MAP_BIT		12
#define I2S_TX3_CH2_MAP_BIT		8
#define I2S_TX3_CH1_MAP_BIT		4
#define I2S_TX3_CH0_MAP_BIT		0

// DA_RXCHSEL
#define I2S_RX_OFFSET_BIT		12
#define I2S_RX_CHSEL_BIT		0

//DA_RXCHMAP
#define I2S_RX_CH7_MAP_BIT		28
#define I2S_RX_CH6_MAP_BIT		24
#define I2S_RX_CH5_MAP_BIT		20
#define I2S_RX_CH4_MAP_BIT		16
#define I2S_RX_CH3_MAP_BIT		12
#define I2S_RX_CH2_MAP_BIT		8
#define I2S_RX_CH1_MAP_BIT		4
#define I2S_RX_CH0_MAP_BIT		0



/*** Some Config Value ***/

//I2S BCLK POLARITY Control
#define BCLK_NORMAL_DRIVE_N_SAMPLE_P	0
#define BCLK_INVERT_DRIVE_P_SAMPLE_N	1

//I2S LRCK POLARITY Control
#define	LRCK_LEFT_LOW_RIGHT_HIGH		0
#define LRCK_LEFT_HIGH_RIGHT_LOW		1

//I2S Format Selection
#define PCM_FORMAT						0
#define LEFT_JUSTIFIED_FORMAT			1
#define RIGHT_JUSTIFIED_FORMAT			2

//I2S Sign Extend in slot
#define ZERO_OR_AUDIIO_GAIN_PADDING_LSB	0
#define SIGN_EXTENSION_MSB				1
#define TRANSFER_ZERO_AFTER				3

//I2S PCM Frame mode
#define PCM_SHORT_FRAME					0
#define PCM_LONG_FRAME					1


#ifdef __cplusplus
}
#endif


#endif	//_XRADIO_I2S_H_


