/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the
 *	 distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *	 its contributors may be used to endorse or promote products derived
 *	 from this software without specific prior written permission.
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

#include <stdbool.h>
#include "audio_arch.h"
#include "driver/chip/hal_i2c.h"

#include "ac107.h"


//Debug config
#define AC107_DBG_EN				0
#define AC107_ERR_EN				1

#define AC107_DBG(fmt, arg...)		HAL_LOG(AC107_DBG_EN, "[AC107_CODEC] "fmt, ##arg)
#define AC107_ERR(fmt, arg...)		HAL_LOG(AC107_ERR_EN, "[AC107_CODEC] "fmt, ##arg)
#define AC107_ALWAYS(fmt, arg...)	HAL_LOG(1, "[AC107_CODEC] "fmt, ##arg)

//AC107 default config
#define AC107_DEFAULT_LRCK_PERIOD	32
#define AC107_DEFAULT_RECORD_GAIN	VOLUME_GAIN_0dB

//Interface define
#define AC107_MALLOC             	HAL_Malloc
#define AC107_FREE               	HAL_Free
#define AC107_MEMCPY             	HAL_Memcpy
#define AC107_MEMSET             	HAL_Memset

#define AC107_I2C_READ				HAL_I2C_Master_Receive_Mem_IT
#define AC107_I2C_WRITE				HAL_I2C_Master_Transmit_Mem_IT


//AC107 codec priv struct
struct ac107_i2c_config {
	I2C_ID i2c_id;
	uint8_t i2c_addr;
};

struct ac107_codec_priv {
	//hw config
	bool pdm_en;
	bool encoding_en;
	bool encoding_fmt;
	uint8_t  encoding_nums;
	uint8_t  adc_pattern;
	uint16_t lrck_period;

	//I2C config
	uint8_t chip_nums;
	struct ac107_i2c_config *ac107_i2c_cfg;
};

struct ac107_codec_priv *ac107_priv;


//const array define
struct real_val_to_reg_val {
	uint32_t real_val;
	uint32_t reg_val;
};

struct pll_div {
	uint32_t freq_in;
	uint32_t freq_out;
	uint32_t m1;
	uint32_t m2;
	uint32_t n;
	uint32_t k1;
	uint32_t k2;
};


static const struct real_val_to_reg_val ac107_sample_rate[] = {
	{8000,  0},
	{11025, 1},
	{12000, 2},
	{16000, 3},
	{22050, 4},
	{24000, 5},
	{32000, 6},
	{44100, 7},
	{48000, 8},
};

static const struct real_val_to_reg_val ac107_bclk_div[] = {
	{0,  0},
	{1,  1},
	{2,  2},
	{4,  3},
	{6,  4},
	{8,  5},
	{12, 6},
	{16, 7},
	{24, 8},
	{32, 9},
	{48, 10},
	{64, 11},
	{96, 12},
	{128,13},
	{176,14},
	{192,15},
};

static const struct real_val_to_reg_val ac107_pga_gain[] = {
	{VOLUME_GAIN_MINUS_6dB,	0},
	{VOLUME_GAIN_0dB, 		1},
	{VOLUME_GAIN_3dB, 		4},
	{VOLUME_GAIN_4dB, 		5},
	{VOLUME_GAIN_5dB, 		6},
	{VOLUME_GAIN_6dB, 		7},
	{VOLUME_GAIN_7dB, 		8},
	{VOLUME_GAIN_8dB, 		9},
	{VOLUME_GAIN_9dB, 		10},
	{VOLUME_GAIN_10dB, 		11},
	{VOLUME_GAIN_11dB, 		12},
	{VOLUME_GAIN_12dB, 		13},
	{VOLUME_GAIN_13dB, 		14},
	{VOLUME_GAIN_14dB, 		15},
	{VOLUME_GAIN_15dB, 		16},
	{VOLUME_GAIN_16dB, 		17},
	{VOLUME_GAIN_17dB, 		18},
	{VOLUME_GAIN_18dB, 		19},
	{VOLUME_GAIN_19dB, 		20},
	{VOLUME_GAIN_20dB, 		21},
	{VOLUME_GAIN_21dB, 		22},
	{VOLUME_GAIN_22dB, 		23},
	{VOLUME_GAIN_23dB, 		24},
	{VOLUME_GAIN_24dB, 		25},
	{VOLUME_GAIN_25dB, 		26},
	{VOLUME_GAIN_26dB, 		27},
	{VOLUME_GAIN_27dB, 		28},
	{VOLUME_GAIN_28dB, 		29},
	{VOLUME_GAIN_29dB, 		30},
	{VOLUME_GAIN_30dB, 		31},
};

//FOUT =(FIN * N) / [(M1+1) * (M2+1)*(K1+1)*(K2+1)] ;	M1[0,31],  M2[0,1],  N[0,1023],  K1[0,31],  K2[0,1]
static const struct pll_div ac107_pll_div[] = {
	{400000,   AUDIO_CLK_12M, 0,  0, 983,  15, 1},	//<out: 12.2875M>
	{512000,   AUDIO_CLK_12M, 0,  0, 960,  19, 1},	//24576000/48
	{768000,   AUDIO_CLK_12M, 0,  0, 640,  19, 1},	//24576000/32
	{800000,   AUDIO_CLK_12M, 0,  0, 768,  24, 1},
	{1024000,  AUDIO_CLK_12M, 0,  0, 480,  19, 1},	//24576000/24
	{1600000,  AUDIO_CLK_12M, 0,  0, 384,  24, 1},
	{2048000,  AUDIO_CLK_12M, 0,  0, 240,  19, 1},	//24576000/12
	{3072000,  AUDIO_CLK_12M, 0,  0, 160,  19, 1},	//24576000/8
	{4096000,  AUDIO_CLK_12M, 0,  0, 120,  19, 1},	//24576000/6
	{6000000,  AUDIO_CLK_12M, 4,  0, 512,  24, 1},
	{6144000,  AUDIO_CLK_12M, 1,  0, 160,  19, 1},	//24576000/4
	{12000000, AUDIO_CLK_12M, 9,  0, 512,  24, 1},
	{13000000, AUDIO_CLK_12M, 12, 0, 639,  25, 1},	//<out: 12.2885M>
	{15360000, AUDIO_CLK_12M, 9,  0, 320,  19, 1},
	{16000000, AUDIO_CLK_12M, 9,  0, 384,  24, 1},
	{19200000, AUDIO_CLK_12M, 11, 0, 384,  24, 1},
	{19680000, AUDIO_CLK_12M, 15, 1, 999,  24, 1},	//<out: 12.2877M>
	{24000000, AUDIO_CLK_12M, 9,  0, 256,  24, 1},

	{400000,   AUDIO_CLK_11M, 0,  0, 1016, 17, 1},	//<out: 11.2889M>
	{512000,   AUDIO_CLK_11M, 0,  0, 882,  19, 1},
	{768000,   AUDIO_CLK_11M, 0,  0, 588,  19, 1},
	{800000,   AUDIO_CLK_11M, 0,  0, 508,  17, 1},	//<out: 11.2889M>
	{1024000,  AUDIO_CLK_11M, 0,  0, 441,  19, 1},
	{1600000,  AUDIO_CLK_11M, 0,  0, 254,  17, 1},	//<out: 11.2889M>
	{2048000,  AUDIO_CLK_11M, 1,  0, 441,  19, 1},
	{3072000,  AUDIO_CLK_11M, 0,  0, 147,  19, 1},
	{4096000,  AUDIO_CLK_11M, 3,  0, 441,  19, 1},
	{6000000,  AUDIO_CLK_11M, 1,  0, 143,  18, 1},	//<out: 11.2895M>
	{6144000,  AUDIO_CLK_11M, 1,  0, 147,  19, 1},
	{12000000, AUDIO_CLK_11M, 3,  0, 143,  18, 1},	//<out: 11.2895M>
	{13000000, AUDIO_CLK_11M, 12, 0, 429,  18, 1},	//<out: 11.2895M>
	{15360000, AUDIO_CLK_11M, 14, 0, 441,  19, 1},
	{16000000, AUDIO_CLK_11M, 24, 0, 882,  24, 1},
	{19200000, AUDIO_CLK_11M, 4,  0, 147,  24, 1},
	{19680000, AUDIO_CLK_11M, 13, 1, 771,  23, 1},	//<out: 11.28964M>
	{24000000, AUDIO_CLK_11M, 24, 0, 588,  24, 1},


	{12288000, AUDIO_CLK_12M, 9,  0, 400,  19, 1},		//24576000/2
	{11289600, AUDIO_CLK_11M, 9,  0, 400,  19, 1},		//22579200/2

	{24576000/1,   AUDIO_CLK_12M, 9, 0, 200, 19, 1},	//24576000
	{24576000/16,  AUDIO_CLK_12M, 0, 0, 320, 19, 1},	//1536000
	{24576000/64,  AUDIO_CLK_12M, 0, 0, 640, 9,  1},	//384000
	{24576000/96,  AUDIO_CLK_12M, 0, 0, 960, 9,  1},	//256000
	{24576000/128, AUDIO_CLK_12M, 0, 0, 512, 3,  1},	//192000
	{24576000/176, AUDIO_CLK_12M, 0, 0, 880, 4,  1},	//140000
	{24576000/192, AUDIO_CLK_12M, 0, 0, 960, 4,  1},	//128000

	{22579200/1,   AUDIO_CLK_11M, 9, 0, 200, 19, 1},	//22579200
	{22579200/4,   AUDIO_CLK_11M, 4, 0, 400, 19, 1},	//5644800
	{22579200/16,  AUDIO_CLK_11M, 0, 0, 320, 19, 1},	//1411200
	{22579200/64,  AUDIO_CLK_11M, 0, 0, 640, 9,  1},	//352800
	{22579200/96,  AUDIO_CLK_11M, 0, 0, 960, 9,  1},	//235200
	{22579200/128, AUDIO_CLK_11M, 0, 0, 512, 3,  1},	//176400
	{22579200/176, AUDIO_CLK_11M, 0, 0, 880, 4,  1},	//128290
	{22579200/192, AUDIO_CLK_11M, 0, 0, 960, 4,  1},	//117600

	{22579200/6,   AUDIO_CLK_11M, 2, 0, 360, 19, 1},	//3763200
	{22579200/8,   AUDIO_CLK_11M, 0, 0, 160, 19, 1}, 	//2822400
	{22579200/12,  AUDIO_CLK_11M, 0, 0, 240, 19, 1},	//1881600
	{22579200/24,  AUDIO_CLK_11M, 0, 0, 480, 19, 1}, 	//940800
	{22579200/32,  AUDIO_CLK_11M, 0, 0, 640, 19, 1}, 	//705600
	{22579200/48,  AUDIO_CLK_11M, 0, 0, 960, 19, 1}, 	//470400
};


//Base read/write Interface
static int ac107_read(uint8_t reg, uint8_t *rt_value, struct ac107_i2c_config *i2c_cfg)
{
	if(i2c_cfg == NULL){
		AC107_ERR("ac107_i2c_config is NULL!\n");
		return HAL_INVALID;
	}

	//return read success nums
	return AC107_I2C_READ(i2c_cfg->i2c_id, i2c_cfg->i2c_addr, reg, I2C_MEMADDR_SIZE_8BIT, rt_value, 1);
}

static int ac107_write(uint8_t reg, uint8_t value, struct ac107_i2c_config *i2c_cfg)
{
	if(i2c_cfg == NULL){
		AC107_ERR("ac107_i2c_config is NULL!\n");
		return HAL_INVALID;
	}

	//return wirte success nums
	return AC107_I2C_WRITE(i2c_cfg->i2c_id, i2c_cfg->i2c_addr, reg, I2C_MEMADDR_SIZE_8BIT, &value, 1);
}

static int ac107_update_bits(uint8_t reg, uint8_t mask, uint8_t value, struct ac107_i2c_config *i2c_cfg)
{
	uint8_t val_old,val_new;

	ac107_read(reg, &val_old, i2c_cfg);
	val_new = (val_old & ~mask) | (value & mask);
	if(val_new != val_old){
		ac107_write(reg, val_new, i2c_cfg);
	}

	return 0;
}

#if 0
static int ac107_multi_chips_read(uint8_t reg, uint8_t *rt_value)
{
	uint8_t i;

	for(i=0; i<ac107_priv->chip_nums; i++){
		ac107_read(reg, rt_value++, &ac107_priv->ac107_i2c_cfg[i]);
	}

	return 0;
}
#endif

static int ac107_multi_chips_write(uint8_t reg, uint8_t value)
{
	uint8_t i;

	for(i=0; i<ac107_priv->chip_nums; i++){
		ac107_write(reg, value, &ac107_priv->ac107_i2c_cfg[i]);
	}

	return 0;
}

static int ac107_multi_chips_update_bits(uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t i;

	for(i=0; i<ac107_priv->chip_nums; i++){
		ac107_update_bits(reg, mask, value, &ac107_priv->ac107_i2c_cfg[i]);
	}

	return 0;
}


static void ac107_hw_init(struct ac107_i2c_config *i2c_cfg)
{
	/* SYSCLK Config */
	ac107_update_bits(SYSCLK_CTRL, 0x1<<SYSCLK_EN, 0x1<<SYSCLK_EN, i2c_cfg);				/*SYSCLK Enable*/
	ac107_write(MOD_CLK_EN, 0x07, i2c_cfg);		/*0x21=0x07: Module clock enable<I2S, ADC digital,  ADC analog>*/
	ac107_write(MOD_RST_CTRL, 0x03, i2c_cfg);	/*0x22=0x03: Module reset de-asserted<I2S, ADC digital>*/

	if(!ac107_priv->pdm_en){
		/* I2S Common Config */
		ac107_update_bits(I2S_CTRL, 0x1<<SDO_EN, 0x1<<SDO_EN, i2c_cfg);						/*SDO enable*/
		ac107_update_bits(I2S_BCLK_CTRL, 0x1<<EDGE_TRANSFER, 0x0<<EDGE_TRANSFER, i2c_cfg);	/*SDO drive data and SDI sample data at the different BCLK edge*/
		ac107_update_bits(I2S_LRCK_CTRL1, 0x3<<LRCK_PERIODH, ((ac107_priv->lrck_period-1) >> 8)<<LRCK_PERIODH, i2c_cfg);
		ac107_write(I2S_LRCK_CTRL2, (uint8_t)(ac107_priv->lrck_period-1), i2c_cfg);			/*config LRCK period*/
		/*Encoding mode format select 0~N-1, Encoding mode enable, Turn to hi-z state (TDM) when not transferring slot*/
		ac107_update_bits(I2S_FMT_CTRL1, 0x1<<ENCD_FMT | 0x1<<ENCD_SEL | 0x1<<TX_SLOT_HIZ | 0x1<<TX_STATE,\
						!!ac107_priv->encoding_fmt<<ENCD_FMT | !!ac107_priv->encoding_en<<ENCD_SEL | 0x0<<TX_SLOT_HIZ | 0x1<<TX_STATE, i2c_cfg);
		/*0x36=0x60: TX MSB first, SDOUT normal, PCM frame type, Linear PCM Data Mode*/
		ac107_update_bits(I2S_FMT_CTRL3, 0x1<<TX_MLS | 0x1<<SDOUT_MUTE | 0x1<<LRCK_WIDTH | 0x3<<TX_PDM,\
										 0x0<<TX_MLS | 0x0<<SDOUT_MUTE | 0x0<<LRCK_WIDTH | 0x0<<TX_PDM, i2c_cfg);

		ac107_update_bits(I2S_TX_CHMP_CTRL1, 0xff, 0xaa, i2c_cfg);	/*0x3c=0xaa: TX CH1/3/5/7 map to adc1, TX CH2/4/6/8 map to adc2*/
		ac107_update_bits(I2S_TX_CHMP_CTRL2, 0xff, 0xaa, i2c_cfg);	/*0x3d=0xaa: TX CH9/11/13/15 map to adc1, TX CH10/12/14/16 map to adc2*/
	} else {
		/*PDM Interface Latch ADC1 data on rising clock edge. Latch ADC2 data on falling clock edge, PDM Enable*/
		ac107_update_bits(PDM_CTRL, 0x1<<PDM_TIMING | 0x1<<PDM_EN, 0x0<<PDM_TIMING | 0x1<<PDM_EN, i2c_cfg);
	}

	/* ADC DIG part Config */
	ac107_update_bits(ADC_DIG_EN, 0x7, 0x7, i2c_cfg);				/*0x61=0x07: Digital part globe enable, ADCs digital part enable*/

	/* ADC pattern select */
	ac107_write(HPF_EN, !ac107_priv->adc_pattern*0x03, i2c_cfg);
	ac107_update_bits(ADC_DIG_DEBUG, 0x7<<ADC_PTN_SEL, (ac107_priv->adc_pattern & 0x7)<<ADC_PTN_SEL, i2c_cfg);

	/* ADC Digital Volume Config */
	ac107_update_bits(ADC1_DVOL_CTRL, 0xff, 0xA0, i2c_cfg);
	ac107_update_bits(ADC2_DVOL_CTRL, 0xff, 0xA0, i2c_cfg);
}

static void ac107_set_amic(bool enable)
{
	AC107_ALWAYS("Route(cap): main mic %s\n",enable ? "Enable" : "Disable");

	/* Analog voltage enable/disable */
	ac107_multi_chips_update_bits(PWR_CTRL1, 0x1<<VREF_ENABLE, !!enable<<VREF_ENABLE);
	ac107_multi_chips_update_bits(PWR_CTRL2, 0x1<<MICBIAS2_EN | 0x1<<MICBIAS1_EN, !!enable<<MICBIAS2_EN | !!enable<<MICBIAS1_EN);

	/* ADCs analog global enable/disable */
	ac107_multi_chips_update_bits(ANA_ADC1_CTRL5, 0x1<<RX1_GLOBAL_EN, !!enable<<RX1_GLOBAL_EN);
	ac107_multi_chips_update_bits(ANA_ADC2_CTRL5, 0x1<<RX2_GLOBAL_EN, !!enable<<RX2_GLOBAL_EN);

	/* VREF Fast Start-up disable/enable */
	HAL_MSleep(20*enable);
	ac107_multi_chips_update_bits(PWR_CTRL1, 0x1<<VREF_FSU_DISABLE, !!enable<<VREF_FSU_DISABLE);
}

static void ac107_set_dmic(bool enable)
{
	AC107_ALWAYS("Route(cap): dmic %s\n",enable ? "Enable" : "Disable");

	/* DMIC enable/disable */
	ac107_multi_chips_update_bits(DMIC_EN, 0x1, !!enable);
}

static int ac107_set_pll(Codec_Pllclk_Src pllclk_src, uint32_t freq_in, uint32_t freq_out)
{
	uint32_t i,m1,m2,n,k1,k2;
	AC107_DBG("%s\n",__FUNCTION__);

	if (!freq_out)	return HAL_INVALID;

	if (freq_in < 128000 || freq_in > 24576000) {
		AC107_ERR("AC107 PLLCLK source input freq only support [128K,24M],while now %u\n\n",freq_in);
		return HAL_INVALID;
	} else if ((freq_in == AUDIO_CLK_12M || freq_in == AUDIO_CLK_11M) && (pllclk_src == PLLCLK_SRC_MCLK || pllclk_src == PLLCLK_SRC_BCLK)) {
		//System Clock Source Select MCLK/BCLK
		i = (pllclk_src == PLLCLK_SRC_MCLK ? 0 : 1);
		AC107_DBG("AC107 don't need to use PLL, SYSCLK source select %s\n\n",i? "BCLK" : "MCLK");
		ac107_multi_chips_update_bits(SYSCLK_CTRL, 0x3<<SYSCLK_SRC, i<<SYSCLK_SRC);
		return HAL_OK;	//Don't need to use PLL
	}

	/* PLL Clock Source Select */
	switch(pllclk_src){
		case PLLCLK_SRC_MCLK:
			AC107_DBG("AC107 PLLCLK input source select MCLK\n");
			ac107_multi_chips_update_bits(SYSCLK_CTRL, 0x3<<PLLCLK_SRC, 0x0<<PLLCLK_SRC);
			break;
		case PLLCLK_SRC_BCLK:
			AC107_DBG("AC107 PLLCLK input source select BCLK\n");
			ac107_multi_chips_update_bits(SYSCLK_CTRL, 0x3<<PLLCLK_SRC, 0x1<<PLLCLK_SRC);
			break;
		case PLLCLK_SRC_PDMCLK:
			AC107_DBG("AC107 PLLCLK input source select PDMCLK\n");
			ac107_multi_chips_update_bits(SYSCLK_CTRL, 0x3<<PLLCLK_SRC, 0x2<<PLLCLK_SRC);
			break;
		default:
			AC107_ERR("AC107 PLLCLK source config error:%d\n\n",pllclk_src);
			return HAL_INVALID;
	}

	//FOUT =(FIN * N) / [(M1+1) * (M2+1)*(K1+1)*(K2+1)] ;
	for(i=0; i<HAL_ARRAY_SIZE(ac107_pll_div); i++){
		if(ac107_pll_div[i].freq_in == freq_in && ac107_pll_div[i].freq_out == freq_out){
			m1 = ac107_pll_div[i].m1;
			m2 = ac107_pll_div[i].m2;
			n = ac107_pll_div[i].n;
			k1 = ac107_pll_div[i].k1;
			k2 = ac107_pll_div[i].k2;
			AC107_DBG("AC107 PLL freq_in match:%u, freq_out:%u\n\n",freq_in,freq_out);
			break;
		}
	}

	if(i == HAL_ARRAY_SIZE(ac107_pll_div)){
		AC107_ERR("AC107 don't match PLLCLK freq_in and freq_out table\n\n");
		return HAL_INVALID;
	}

	/* Config PLL DIV param M1/M2/N/K1/K2 */
	ac107_multi_chips_update_bits(PLL_CTRL2, 0x1f<<PLL_PREDIV1 | 0x1<<PLL_PREDIV2, m1<<PLL_PREDIV1 | m2<<PLL_PREDIV2);
	ac107_multi_chips_update_bits(PLL_CTRL3, 0x3<<PLL_LOOPDIV_MSB, (n>>8)<<PLL_LOOPDIV_MSB);
	ac107_multi_chips_update_bits(PLL_CTRL4, 0xff<<PLL_LOOPDIV_LSB, (uint8_t)n<<PLL_LOOPDIV_LSB);
	ac107_multi_chips_update_bits(PLL_CTRL5, 0x1f<<PLL_POSTDIV1 | 0x1<<PLL_POSTDIV2, k1<<PLL_POSTDIV1 | k2<<PLL_POSTDIV2);

	/* Config PLL module current */
	//ac107_multi_chips_update_bits(PLL_CTRL1, 0x7<<PLL_IBIAS, 0x4<<PLL_IBIAS);
	//ac107_multi_chips_update_bits(PLL_CTRL6, 0x1f<<PLL_CP, 0xf<<PLL_CP);

	/* PLL module enable */
	ac107_multi_chips_update_bits(PLL_LOCK_CTRL, 0x7<<SYSCLK_HOLD_TIME | 0x1<<PLL_LOCK_EN, 0x3<<SYSCLK_HOLD_TIME | 0x1<<PLL_LOCK_EN);
	ac107_multi_chips_update_bits(PLL_CTRL1, 0x1<<PLL_EN | 0x1<<PLL_COM_EN, 0x1<<PLL_EN | 0x1<<PLL_COM_EN);

	/* PLLCLK Enable */
	ac107_multi_chips_update_bits(SYSCLK_CTRL, 0x1<<PLLCLK_EN, 0x1<<PLLCLK_EN);

	return HAL_OK;
}


static int ac107_dai_set_sysclk(Codec_Sysclk_Src sysclk_src, Codec_Pllclk_Src pllclk_src, uint32_t pll_freq_in, uint32_t sample_rate)
{
	AC107_DBG("--->%s\n",__FUNCTION__);
	int ret = HAL_OK;

	switch(sysclk_src){
		case SYSCLK_SRC_MCLK:
			AC107_DBG("SYSCLK source select MCLK\n");
			ac107_multi_chips_update_bits(SYSCLK_CTRL, 0x3<<SYSCLK_SRC, 0x0<<SYSCLK_SRC);
			break;
		case SYSCLK_SRC_BCLK:
			AC107_DBG("SYSCLK source select BCLK\n");
			ac107_multi_chips_update_bits(SYSCLK_CTRL, 0x3<<SYSCLK_SRC, 0x1<<SYSCLK_SRC);
			break;
		case SYSCLK_SRC_PLL:
			AC107_DBG("SYSCLK source select PLL\n");
			ac107_multi_chips_update_bits(SYSCLK_CTRL, 0x3<<SYSCLK_SRC, 0x2<<SYSCLK_SRC);
			ret = ac107_set_pll(pllclk_src, pll_freq_in, sample_rate%1000 ? AUDIO_CLK_11M : AUDIO_CLK_12M);
			break;
		default:
			AC107_ERR("SYSCLK source select Error!\n");
			return HAL_INVALID;
	}

	return ret;
}

static int ac107_dai_set_fmt(uint32_t fmt)
{
	AC107_DBG("--->%s\n",__FUNCTION__);
	uint8_t i, tx_offset, i2s_mode, sign_ext, lrck_polarity, brck_polarity;

	if(!ac107_priv || !ac107_priv->ac107_i2c_cfg){
		AC107_ERR("AC107 hasn't been registered!\n");
		return HAL_ERROR;
	}

	if(ac107_priv->pdm_en){
		AC107_DBG("AC107 use PDM output, don't need to config I2S format\n");
		return HAL_OK;
	}

	/* AC107 config Master/Slave mode */
	switch (fmt & I2S_ROLE_MASK) {
		case DAIFMT_CBM_CFM:	//AC107 Master
			AC107_DBG("AC107 set to work as Master\n");
			ac107_update_bits(I2S_CTRL, 0x3<<LRCK_IOEN, 0x3<<LRCK_IOEN, &ac107_priv->ac107_i2c_cfg[0]);	//BCLK & LRCK output
			break;
		case DAIFMT_CBS_CFS:	//AC107 Slave
			AC107_DBG("AC107 set to work as Slave\n");
			ac107_update_bits(I2S_CTRL, 0x3<<LRCK_IOEN, 0x0<<LRCK_IOEN, &ac107_priv->ac107_i2c_cfg[0]);	//BCLK & LRCK input
			break;
		default:
			AC107_ERR("AC107 Master/Slave mode config error:%u\n\n",(fmt & I2S_ROLE_MASK)>>0);
			return HAL_INVALID;
	}
	for(i=1; i<ac107_priv->chip_nums; i++){	//multi_chips: only one chip set as Master, and the others also need to set as Slave
		ac107_update_bits(I2S_CTRL, 0x3<<LRCK_IOEN, 0x0<<LRCK_IOEN, &ac107_priv->ac107_i2c_cfg[i]);
	}

	/* AC107 config I2S/LJ/RJ/PCM format */
	switch (fmt & I2S_FORMAT_MASK) {
		case DAIFMT_I2S:
			AC107_DBG("AC107 config I2S format\n");
			i2s_mode = LEFT_JUSTIFIED_FORMAT;
			tx_offset = 1;
			sign_ext = TRANSFER_ZERO_AFTER;
			break;
		case DAIFMT_RIGHT_J:
			AC107_DBG("AC107 config RIGHT-JUSTIFIED format\n");
			i2s_mode = RIGHT_JUSTIFIED_FORMAT;
			tx_offset = 0;
			sign_ext = SIGN_EXTENSION_MSB;
			break;
		case DAIFMT_LEFT_J:
			AC107_DBG("AC107 config LEFT-JUSTIFIED format\n");
			i2s_mode = LEFT_JUSTIFIED_FORMAT;
			tx_offset = 0;
			sign_ext = TRANSFER_ZERO_AFTER;
			break;
		case DAIFMT_DSP_A:
			AC107_DBG("AC107 config PCM-A format\n");
			i2s_mode = PCM_FORMAT;
			tx_offset = 1;
			sign_ext = TRANSFER_ZERO_AFTER;
			break;
		case DAIFMT_DSP_B:
			AC107_DBG("AC107 config PCM-B format\n");
			i2s_mode = PCM_FORMAT;
			tx_offset = 0;
			sign_ext = TRANSFER_ZERO_AFTER;
			break;
		default:
			AC107_ERR("AC107 I2S format config error:%u\n\n",(fmt & I2S_FORMAT_MASK)>>4);
			return HAL_INVALID;
	}
	ac107_multi_chips_update_bits(I2S_FMT_CTRL1, 0x3<<MODE_SEL | 0x1<<TX_OFFSET, i2s_mode<<MODE_SEL | tx_offset<<TX_OFFSET);
	ac107_multi_chips_update_bits(I2S_FMT_CTRL3, 0x3<<SEXT, sign_ext<<SEXT);

	/* AC107 config BCLK&LRCK polarity */
	switch (fmt & I2S_POLARITY_MASK) {
		case DAIFMT_NB_NF:
			AC107_DBG("AC107 config BCLK&LRCK polarity: BCLK_normal,LRCK_normal\n");
			brck_polarity = BCLK_NORMAL_DRIVE_N_SAMPLE_P;
			lrck_polarity = LRCK_LEFT_LOW_RIGHT_HIGH;
			break;
		case DAIFMT_NB_IF:
			AC107_DBG("AC107 config BCLK&LRCK polarity: BCLK_normal,LRCK_invert\n");
			brck_polarity = BCLK_NORMAL_DRIVE_N_SAMPLE_P;
			lrck_polarity = LRCK_LEFT_HIGH_RIGHT_LOW;
			break;
		case DAIFMT_IB_NF:
			AC107_DBG("AC107 config BCLK&LRCK polarity: BCLK_invert,LRCK_normal\n");
			brck_polarity = BCLK_INVERT_DRIVE_P_SAMPLE_N;
			lrck_polarity = LRCK_LEFT_LOW_RIGHT_HIGH;
			break;
		case DAIFMT_IB_IF:
			AC107_DBG("AC107 config BCLK&LRCK polarity: BCLK_invert,LRCK_invert\n");
			brck_polarity = BCLK_INVERT_DRIVE_P_SAMPLE_N;
			lrck_polarity = LRCK_LEFT_HIGH_RIGHT_LOW;
			break;
		default:
			AC107_ERR("AC107 config BCLK/LRCLK polarity error:%u\n\n",(fmt & I2S_POLARITY_MASK)>>8);
			return HAL_INVALID;
	}
	ac107_multi_chips_update_bits(I2S_BCLK_CTRL,  0x1<<BCLK_POLARITY, brck_polarity<<BCLK_POLARITY);
	ac107_multi_chips_update_bits(I2S_LRCK_CTRL1, 0x1<<LRCK_POLARITY, lrck_polarity<<LRCK_POLARITY);

	return HAL_OK;
}

static int ac107_dai_set_volume(Audio_Device device, uint16_t volume)
{
	AC107_DBG("--->%s\n",__FUNCTION__);
	uint32_t i,reg_val=0;
	uint16_t vol_set_flag, vol_set_value;

	if(volume == VOLUME_INVALID){
		vol_set_flag  = VOLUME_SET_GAIN;
		vol_set_value = AC107_DEFAULT_RECORD_GAIN;
	} else {
		vol_set_flag  = volume & VOLUME_SET_MASK;
		vol_set_value = volume & ~VOLUME_SET_MASK;
	}

	switch(device){
		case AUDIO_IN_DEV_AMIC:
			if(vol_set_flag == VOLUME_SET_LEVEL){
				if (vol_set_value > VOLUME_LEVEL31){
					AC107_ERR("Invalid Amic volume level: %d!\n",vol_set_value);
					return HAL_INVALID;
				}
				reg_val = vol_set_value;
				AC107_ALWAYS("AMIC set volume Level-[%d]\n",vol_set_value);
			} else if (vol_set_flag == VOLUME_SET_GAIN){
				for(i=0; i<HAL_ARRAY_SIZE(ac107_pga_gain); i++){
					if(ac107_pga_gain[i].real_val == vol_set_value){
						reg_val = ac107_pga_gain[i].reg_val;
						AC107_ALWAYS("AMIC set volume Gain-[%d]\n",vol_set_value);
						break;
					}
				}
				if(i == HAL_ARRAY_SIZE(ac107_pga_gain)){
					AC107_ERR("Invalid main mic volume gain: %d!\n",vol_set_value);
					return HAL_INVALID;
				}
			}

			/* ADCs analog PGA gain Config */
			ac107_multi_chips_update_bits(ANA_ADC1_CTRL3, 0x1f<<RX1_PGA_GAIN_CTRL, reg_val<<RX1_PGA_GAIN_CTRL);
			ac107_multi_chips_update_bits(ANA_ADC2_CTRL3, 0x1f<<RX2_PGA_GAIN_CTRL, reg_val<<RX2_PGA_GAIN_CTRL);
			break;
		case AUDIO_IN_DEV_DMIC:
			AC107_ERR("DMIC don't support set volume\n");
			return HAL_INVALID;
		default:
			AC107_ERR("Invalid Audio Device-[0x%08x]!\n",device);
			return HAL_INVALID;
	}

	return HAL_OK;
}

__nonxip_text
static int ac107_dai_set_route(Audio_Device device, Audio_Dev_State state)
{
	//AC107_DBG("--->%s\n",__FUNCTION__);
	bool enable = (state==AUDIO_DEV_EN) ? 1 : 0;

	switch(device){
		case AUDIO_IN_DEV_AMIC:
			ac107_set_amic(enable);
			break;
		case AUDIO_IN_DEV_DMIC:
			ac107_set_dmic(enable);
			break;
		default:
			//AC107_ERR("Invalid Audio Device-[0x%08x]!\n",device);
			return HAL_INVALID;
	}

	return HAL_OK;
}

static int ac107_dai_hw_params(Audio_Stream_Dir dir, struct pcm_config *pcm_cfg)
{
	AC107_DBG("--->%s\n",__FUNCTION__);
	uint16_t i, channels, channels_en, sample_resolution, slot_width, bclk_div;

	if(dir == PCM_OUT){
		AC107_ERR("Invalid audio stream dir-[%d], AC107 only support capture!\n",dir);
		return HAL_INVALID;
	}

	/* AC107 common init */
	for(i=0; i<ac107_priv->chip_nums; i++){
		ac107_hw_init(&ac107_priv->ac107_i2c_cfg[i]);
	}

	/* AC107 set sample rate */
	for(i=0; i<HAL_ARRAY_SIZE(ac107_sample_rate); i++){
		if(ac107_sample_rate[i].real_val == pcm_cfg->rate / (ac107_priv->encoding_en ? ac107_priv->encoding_nums/2 : 1)){
			ac107_multi_chips_update_bits(ADC_SPRC, 0xf<<ADC_FS_I2S, ac107_sample_rate[i].reg_val<<ADC_FS_I2S);
			break;
		}
	}
	if(i == HAL_ARRAY_SIZE(ac107_sample_rate)){
		AC107_ERR("AC107 set sample rate Fail!\n");
		return HAL_INVALID;
	}

	if(ac107_priv->pdm_en){
		AC107_DBG("Sample rate-[%d]\n",pcm_cfg->rate);
		return HAL_OK;
	}

	/* AC107 set channels */
	channels = pcm_cfg->channels * (ac107_priv->encoding_en ? ac107_priv->encoding_nums/2 : 1);
	for(i=0; i<(channels+1)/2; i++){
		channels_en = (channels >= 2*(i+1)) ? 0x0003<<(2*i) : ((1<<(channels%2))-1)<<(2*i);
		ac107_write(I2S_TX_CTRL1, channels-1, &ac107_priv->ac107_i2c_cfg[i]);
		ac107_write(I2S_TX_CTRL2, (uint8_t)channels_en, &ac107_priv->ac107_i2c_cfg[i]);
		ac107_write(I2S_TX_CTRL3, channels_en>>8, &ac107_priv->ac107_i2c_cfg[i]);
	}
	for(; i<ac107_priv->chip_nums; i++){
		ac107_write(I2S_TX_CTRL1, 0, &ac107_priv->ac107_i2c_cfg[i]);
		ac107_write(I2S_TX_CTRL2, 0, &ac107_priv->ac107_i2c_cfg[i]);
		ac107_write(I2S_TX_CTRL3, 0, &ac107_priv->ac107_i2c_cfg[i]);
	}

	/* AC107 set sample resorution and slot width */
	sample_resolution = pcm_format_to_sampleresolution(pcm_cfg->format);
	slot_width = 32;//sample_resolution<=8 ? 8 : (sample_resolution<=16 ? 16 : 32);
	ac107_multi_chips_update_bits(I2S_FMT_CTRL2, 0x7<<SLOT_WIDTH_SEL | 0x7<<SAMPLE_RESOLUTION,\
		(slot_width/4-1)<<SLOT_WIDTH_SEL | (sample_resolution/4-1)<<SAMPLE_RESOLUTION);

	/* AC107 set BCLK div, only use in master mode */
	bclk_div = (pcm_cfg->rate%100 ? AUDIO_CLK_11M : AUDIO_CLK_12M)/pcm_cfg->rate/(2*ac107_priv->lrck_period);	//default I2S/LJ/RJ format
	for(i=0; i<HAL_ARRAY_SIZE(ac107_bclk_div); i++){
		if(ac107_bclk_div[i].real_val == bclk_div){
			ac107_multi_chips_update_bits(I2S_BCLK_CTRL, 0xf<<BCLKDIV, ac107_bclk_div[i].reg_val<<BCLKDIV);
			AC107_DBG("AC107 set BCLK_DIV_[%u]\n\n",bclk_div);
			break;
		}
	}

	/* AC107 TX enable, Globle enable */
	ac107_multi_chips_update_bits(I2S_CTRL, 0x1<<TXEN | 0x1<<GEN, 0x1<<TXEN | 0x1<<GEN);

	AC107_DBG("Sample rate-[%d], channel numbers-[%d], sample resolution-[%d]\n",pcm_cfg->rate,pcm_cfg->channels,sample_resolution);
	return HAL_OK;
}

static int ac107_dai_hw_free(Audio_Stream_Dir dir)
{
	AC107_DBG("--->%s\n",__FUNCTION__);

	if(dir == PCM_OUT){
		AC107_ERR("Invalid audio stream dir-[%d], AC107 only support capture!\n",dir);
		return HAL_INVALID;
	}

	/* AC107 I2S Globle disable */
	ac107_multi_chips_update_bits(I2S_CTRL, 0x1<<GEN, 0x0<<GEN);

	/* AC107 soft reset */
	AC107_DBG("AC107 reset all register to their default value\n\n");
	ac107_multi_chips_write(CHIP_AUDIO_RST, 0x12);

	return HAL_OK;
}


static int ac107_codec_ioctl(uint32_t cmd, uint32_t cmd_param[], uint32_t cmd_param_len)
{
	int ret = HAL_ERROR;
	AC107_DBG("--->%s\n",__FUNCTION__);

	switch(cmd){
		case CODEC_IOCTL_HW_CONFIG:
			if(cmd_param_len != 2)	return HAL_INVALID;
			ac107_priv->pdm_en 		  = cmd_param[0] & HAL_BIT(0);
			ac107_priv->encoding_en   = cmd_param[0] & HAL_BIT(1);
			ac107_priv->encoding_fmt  = cmd_param[0] & HAL_BIT(2);
			ac107_priv->encoding_nums = cmd_param[1] & 0xff;
			ac107_priv->adc_pattern   = cmd_param[1]>>8  & 0xff;
			ac107_priv->lrck_period   = cmd_param[1]>>16 & 0xffff;
			ret = HAL_OK;
			break;

		default:
			AC107_ERR("Invalid ioctl command!\n");
			return HAL_INVALID;
	}

	return ret;
}

static int ac107_codec_reg_read(uint32_t reg)
{
	int ret;
	uint8_t reg_val;
	AC107_DBG("--->%s\n",__FUNCTION__);

	if(!ac107_priv || !ac107_priv->ac107_i2c_cfg){
		AC107_ERR("AC107 hasn't been registered!\n");
		return HAL_ERROR;
	}

	ret = ac107_read(reg, &reg_val, &ac107_priv->ac107_i2c_cfg[0]);
	if(ret != 1){
		return ret;
	}

	return reg_val;
}

static int ac107_codec_reg_write(uint32_t reg, uint32_t val)
{
	AC107_DBG("--->%s\n",__FUNCTION__);

	if(!ac107_priv || !ac107_priv->ac107_i2c_cfg){
		AC107_ERR("AC107 hasn't been registered!\n");
		return HAL_ERROR;
	}

	ac107_multi_chips_write(reg, val);

	return HAL_OK;
}

static int ac107_codec_open(Audio_Stream_Dir dir)
{
	AC107_DBG("--->%s\n",__FUNCTION__);

	if(dir == PCM_OUT){
		AC107_ERR("Invalid audio stream dir-[%d], AC107 only support capture!\n",dir);
		return HAL_INVALID;
	}

#if AC107_DBG_EN
	uint8_t i, rt_value;
	printf("\nac107 reg:");
	for(i=0; i<0xAB; i++){
		rt_value = 0;
		if(!(i%4))	printf("\n");
		ac107_read(i, &rt_value, &ac107_priv->ac107_i2c_cfg[0]);
		printf("Reg[0x%02x]: 0x%02x;  ",i,rt_value);
	}
	printf("\n\n");
#endif

	return HAL_OK;
}

static int ac107_codec_close(Audio_Stream_Dir dir)
{
	AC107_DBG("--->%s\n",__FUNCTION__);

	if(dir == PCM_OUT){
		AC107_ERR("Invalid audio stream dir-[%d], AC107 only support capture!\n",dir);
		return HAL_INVALID;
	}

	return HAL_OK;
}


static int ac107_codec_init(void)
{
	AC107_DBG("--->%s\n",__FUNCTION__);

	return HAL_OK;
}

static void ac107_codec_deinit(void)
{
	AC107_DBG("--->%s\n",__FUNCTION__);
}


/*** codec dai ops ****/
static const struct codec_dai_ops ac107_codec_dai_ops = {
	.set_sysclk = ac107_dai_set_sysclk,
	.set_fmt    = ac107_dai_set_fmt,
	.set_volume = ac107_dai_set_volume,
	.set_route  = ac107_dai_set_route,
	.hw_params  = ac107_dai_hw_params,
	.hw_free    = ac107_dai_hw_free,
};

/*** codec ops ****/
static const struct codec_ops ac107_codec_ops = {
	.open  = ac107_codec_open,
	.close = ac107_codec_close,

	.reg_read  = ac107_codec_reg_read,
	.reg_write = ac107_codec_reg_write,

	.ioctl = ac107_codec_ioctl,
};

/*** codec driver ****/
static struct codec_driver ac107_codec_drv = {
	.name = AC107_CODEC_NAME,
	.codec_attr = XRADIO_CODEC_AC107,

	.init   = ac107_codec_init,
	.deinit = ac107_codec_deinit,

	.dai_ops   = &ac107_codec_dai_ops,
	.codec_ops = &ac107_codec_ops,
};


HAL_Status ac107_pdm_init(Audio_Device device, uint16_t volume, uint32_t sample_rate)
{
	int ret=0;
	struct pcm_config pcm_cfg;

	pcm_cfg.rate = sample_rate;
	ac107_priv->pdm_en = true;

	ret += ac107_dai_set_sysclk(SYSCLK_SRC_PLL, PLLCLK_SRC_PDMCLK, (sample_rate>=32000 ? 64 : 128)*sample_rate, sample_rate);
	ret += ac107_dai_set_volume(device, volume);
	ret += ac107_dai_set_route(device, AUDIO_DEV_EN);
	ret += ac107_dai_hw_params(PCM_IN, &pcm_cfg);
	if(ret) return HAL_ERROR;

	return HAL_OK;
}

HAL_Status ac107_pdm_deinit(void)
{
	return ac107_dai_hw_free(PCM_IN);
}


HAL_Status ac107_codec_register(void)
{
	AC107_DBG("--->%s\n",__FUNCTION__);
	uint8_t i, i2c_id, chip_id, detect_nums=0;
	const uint8_t ac107_i2c_addr[] = {0x36, 0x37, 0x38, 0x39};
	struct ac107_i2c_config ac107_i2c_cfg_temp[I2C_NUM * HAL_ARRAY_SIZE(ac107_i2c_addr)];

	I2C_InitParam i2c_param;
	i2c_param.addrMode = I2C_ADDR_MODE_7BIT;
	i2c_param.clockFreq = 400000;
	for(i=0; i<I2C_NUM; i++){
		if(HAL_I2C_Init(i, &i2c_param) != HAL_OK){
			AC107_ERR("I2C-[%d] init Fail...\n",i);
		}
	}

	/* Auto detect AC107 */
	for(i2c_id=0; i2c_id<I2C_NUM-1; i2c_id++){
		for(i=0; i<HAL_ARRAY_SIZE(ac107_i2c_addr); i++){
			chip_id = 0;
			AC107_I2C_READ(i2c_id, ac107_i2c_addr[i], CHIP_AUDIO_RST, I2C_MEMADDR_SIZE_8BIT, &chip_id, 1);
			if(chip_id == 0x4B){
				AC107_DBG("AC107-[0x%02x] on I2C-[%d] auto detect success\n",ac107_i2c_addr[i],i2c_id);
				ac107_i2c_cfg_temp[detect_nums].i2c_id = i2c_id;
				ac107_i2c_cfg_temp[detect_nums].i2c_addr = ac107_i2c_addr[i];
				detect_nums++;
			}
		}
	}
	if(!detect_nums){
		AC107_ERR("NO AC107 chip been detect, register Fail!\n");
		return HAL_ERROR;
	}

	/* Malloc ac107_priv buffer */
	ac107_priv = (struct ac107_codec_priv *)AC107_MALLOC(sizeof(struct ac107_codec_priv));
	if(ac107_priv == NULL){
		AC107_ERR("Malloc ac107 codec priv buffer Fail!\n");
		return HAL_ERROR;
	}
	AC107_MEMSET(ac107_priv, 0, sizeof(struct ac107_codec_priv));

	/* Ac107 i2c config malloc buffer and init */
	ac107_priv->ac107_i2c_cfg = (struct ac107_i2c_config *)AC107_MALLOC(sizeof(struct ac107_i2c_config) * detect_nums);
	if(ac107_priv->ac107_i2c_cfg == NULL){
		AC107_ERR("Malloc ac107 i2c config buffer Fail!\n");
		AC107_FREE(ac107_priv);
		return HAL_ERROR;
	}
	AC107_MEMSET(ac107_priv->ac107_i2c_cfg, 0, sizeof(struct ac107_i2c_config) * detect_nums);

	/* Init  ac107_priv */
	ac107_priv->pdm_en = false;
	ac107_priv->encoding_en = false;
	ac107_priv->encoding_fmt = 0;
	ac107_priv->encoding_nums = 4;
	ac107_priv->adc_pattern = ADC_PTN_NORMAL;
	ac107_priv->lrck_period = AC107_DEFAULT_LRCK_PERIOD;
	ac107_priv->chip_nums = detect_nums;
	AC107_MEMCPY(ac107_priv->ac107_i2c_cfg, ac107_i2c_cfg_temp, sizeof(struct ac107_i2c_config) * detect_nums);

	/* Codec list add */
	list_add(&ac107_codec_drv.node, &hal_snd_codec_list);

	return HAL_OK;
}

HAL_Status ac107_codec_unregister(void)
{
	AC107_DBG("--->%s\n",__FUNCTION__);
	struct codec_driver *codec_drv_ptr;

	/* Check snd codec list empty or not */
	if(list_empty(&hal_snd_codec_list)){
		AC107_DBG("Hal snd codec list is empty, don't need to unregister\n");
		return HAL_OK;
	}

	/* Get codec to unregister */
	list_for_each_entry(codec_drv_ptr, &hal_snd_codec_list, node){
		if(codec_drv_ptr == &ac107_codec_drv){
			list_del(&ac107_codec_drv.node);
			break;
		}
	}

	/* Free ac107 priv buffer */
	if(ac107_priv){
		AC107_FREE(ac107_priv->ac107_i2c_cfg);
		AC107_FREE(ac107_priv);
		ac107_priv = NULL;
	}

	return HAL_OK;
}


