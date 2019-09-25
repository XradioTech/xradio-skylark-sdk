/**
  * @file  xradio_internal_codec.h
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

#ifndef _XRADIO_INTERNAL_CODEC_H_
#define _XRADIO_INTERNAL_CODEC_H_


#ifdef __cplusplus
extern "C" {
#endif

/*** XRADIO Internal Codec Register Define***/

//DAC Digital Control Register
#define AC_DAC_DIG_CTRL			0x00
#define AC_DAC_FIFO_CTRL		0x04
#define AC_DAC_FIFO_STA			0x08
#define AC_DAC_TXDATA			0x0c
#define AC_DAC_TXCNT			0x10
#define AC_DAC_DEBUG			0x14

//ADC Digital Control Register
#define AC_ADC_FIFO_CTRL		0x18
#define AC_ADC_FIFO_STA			0x1c
#define AC_ADC_RXDATA			0x20
#define AC_ADC_RXCNT			0x24
#define AC_ADC_HPF_CTRL			0x28
#define AC_ADC_HPF_COEF			0x2c
#define AC_ADC_HPF_GAIN			0x30

//Debug Control Register
#define AC_ADDA_DBG_CTRL		0x34

//Power Control Regiter
#define AC_POWER_CTRL			0x38

//ADC/DAC Analog Control Regiter
#define AC_ADC_ANA_CTRL			0x40
#define AC_DAC_ANA_CTRL			0x44



/*** XRADIO Internal Codec Register Bit Define***/

//AC_DAC_DIG_CTRL
#define DAC_DIG_EN_BIT			31
#define DAC_MODQU_BIT			25
#define DAC_DWA_BIT				24
#define DAC_HPF_EN_BIT			18
#define DAC_DIG_VOL_BIT			12
#define DAC_MIX_CTRL_BIT		2
#define TX_MIX_CTRL_BIT			0

//AC_DAC_FIFO_CTRL
#define DAC_FS_BIT				29
#define DAC_FIR_VER_BIT			28
#define DAC_SEND_LASAT_BIT		26
#define DAC_FIFO_MODE_BIT		24
#define DAC_DRQ_CLR_CNT_BIT		21
#define DAC_FIFO_TRIG_LEVEL_BIT	8
#define DAC_MONO_EN_BIT			6
#define DAC_SAMPLE_BITS_BIT		5
#define DAC_DRQ_EN_BIT			4
#define DAC_IRQ_EN_BIT			3
#define DAC_UNDERRUN_IRQ_EN_BIT	2
#define DAC_OVERRUN_IRQ_EN_BIT	1
#define DAC_FIFO_FLUSH_BIT		0

//AC_DAC_FIFO_STA
#define TX_EMPTY_BIT			23
#define TXE_CNT_BIT				8
#define TXE_INT_BIT				3
#define TXU_INT_BIT				2
#define TXO_INT_BIT				1

//AC_DAC_DEBUG
#define DAC_PATTERN_SEL_BIT		9
#define CODEC_SYSCLK_SEL_BIT	8


//AC_ADC_FIFO_CTRL
#define ADC_FS_BIT				29
#define ADC_DIG_EN_BIT			28
#define DMIC_DIG_EN_BIT			27
#define ADC_FIFO_DELAY_TIME_BIT	25
#define ADC_FIFO_DELAY_EN_BIT	24
#define ADC_FIFO_MODE_BIT		23
#define ADC_DMIC_EN_BIT			22
#define ADC_DMIC_MONO_EN_BIT	21
#define ADC_SAMPLE_BITS_BIT		17
#define ADC_TO_DAC_MUX_SEL_BIT	16
#define ADC_OUTPUT_SWAP_BIT		15
#define DMICR_FIFO_EN_BIT		14
#define DMICL_FIFO_EN_BIT		13
#define ADCR_FIFO_EN_BIT		12
#define ADCL_FIFO_EN_BIT		11
#define ACC_FIFO_TRIG_LEVEL_BIT	4
#define ADC_DRQ_EN_BIT			3
#define ADC_IRQ_EN_BIT			2
#define ADC_OVERRUN_IRQ_EN_BIT	1
#define ADC_FIFO_FLUSH_BIT		0

//AC_ADC_FIFO_STA
#define RX_EMPTY_BIT			23
#define RXA_CNT_BIT				8
#define RXA_INT_BIT				3
#define RXO_INT_BIT				1

//AC_ADC_HPF_CTRL
#define DMIC_HPF_EN_BIT			1
#define ADC_HPF_EN_BIT			0

//AC_ADDA_DBG_CTL
#define ADC_PTN_SEL_BIT			4

//AC_POWER_CTRL
#define LINEOUT_RAMP_TIME_BIT	20
#define BG_SPEEDUP_STA_BIT		19
#define ADLDO_EN_BIT			18
#define ADLDO_OUT_VOL_CTRL_BIT	15
#define ADLDO_BYPASS_BIT		14
#define VRA1_EN_BIT				13
#define VRA1_OUT_VOL_CTRL_BIT	11
#define BG_TRIM_BIT				7
#define DAC_IOPVRS_BIT			5
#define DAC_IOPDACS_BIT			3
#define DAC_ILINEOUTS_BIT		1
#define DAC_RSWITCH_BIT			0

// AC_ADC_ANA_CTRL
#define ADC_DITH_AMP_BIT		30
#define LOUT_LOOPB_EN_BIT		29
#define LINEIN_PGA_GAIN_BIT		26
#define LINEIN_PGA_EN_BIT		25
#define ADCR_ANA_EN_BIT			24
#define ADCR_IOPADC1S_BIT		22
#define ADCR_IOPADC2S_BIT		20
#define ADCR_IOPMICS_BIT		18
#define ADCR_IOPMIXS_BIT		16
#define ADC_IOPADC1S_BIT		15
#define ADC_DITHER_EN_BIT		14
#define MIC_MODE_SEL_BIT		13
#define MIC_PGA_GAIN_BIT		10
#define MIC_PGA_EN_BIT			9
#define ADCL_ANA_EN_BIT			8
#define ADCL_IOPADC1S_BIT		6
#define ADCL_IOPADC2S_BIT		4
#define ADCL_IOPMICS_BIT		2
#define ADCL_IOPMIXS_BIT		0

//AC_DAC_ANA_CTRL
#define RAMP_MAN_EN_BIT			17
#define PLAY_ANA_EN_BIT			16
#define DAC_ANA_EN_BIT			15
#define ADDA_BIAS_EN_BIT		9
#define LINEOUT_RAMP_EN_BIT		8
#define LINEOUT_EN_BIT			7
#define LINEOUT_DIFF_EN_BIT		6
#define LINEOUT_MUTE_BIT		5
#define LINEOUT_GAIN_BIT		0


#ifdef __cplusplus
}
#endif


#endif	//_XRADIO_INTERNAL_CODEC_H_
