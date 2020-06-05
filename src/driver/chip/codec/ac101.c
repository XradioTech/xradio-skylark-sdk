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

#include "ac101.h"

#if (__CONFIG_CODEC_HEAP_MODE == 1)
#include "sys/sys_heap.h"
#else
#include <stdlib.h>
#endif


//Debug config
#define AC101_DBG_EN				0
#define AC101_ERR_EN				1

#define AC101_DBG(fmt, arg...)		HAL_LOG(AC101_DBG_EN, "[AC101_CODEC] "fmt, ##arg)
#define AC101_ERR(fmt, arg...)		HAL_LOG(AC101_ERR_EN, "[AC101_CODEC] "fmt, ##arg)
#define AC101_ALWAYS(fmt, arg...)	HAL_LOG(1, "[AC101_CODEC] "fmt, ##arg)

//AC101 default config
#define AC101_DEFAULT_AMIC_VOL		VOLUME_LEVEL0
#define AC101_DEFAULT_LINEIN_VOL	VOLUME_LEVEL4
#define AC101_DEFAULT_HP_VOL		VOLUME_LEVEL31
#define AC101_DEFAULT_SPK_VOL		VOLUME_LEVEL31

#define AC101_DEFAULT_PLAY_DEV		AUDIO_OUT_DEV_SPK
#define AC101_DEFAULT_RECORD_DEV	AUDIO_IN_DEV_AMIC

#define AC101_DEFAULT_SLOT_WIDTH	32

//Interface define
#if (__CONFIG_CODEC_HEAP_MODE == 1)
#define AC101_MALLOC             	psram_malloc
#define AC101_FREE               	psram_free
#else
#define AC101_MALLOC             	HAL_Malloc
#define AC101_FREE               	HAL_Free
#endif

#define AC101_MEMCPY             	HAL_Memcpy
#define AC101_MEMSET             	HAL_Memset

#define AC101_I2C_READ				HAL_I2C_Master_Receive_Mem_IT
#define AC101_I2C_WRITE				HAL_I2C_Master_Transmit_Mem_IT


//AC101 codec priv struct
struct ac101_codec_priv {
	//codec status contrl
	bool playing;
	bool recording;

	//I2C control
	I2C_ID i2c_id;

	//I2S control
	uint8_t slot_width;

	//volume and route control
	uint16_t amic_vol;
	uint16_t linein_vol;
	uint16_t hpout_vol;
	uint16_t spkout_vol;
	bool play_route_def;
	bool record_route_def;

	//misc control
	uint8_t amic_ch_sel;
	uint8_t spkout_ch_sel;
	bool drc_en;
};

struct ac101_codec_priv *ac101_priv;


//const array define
struct real_val_to_reg_val {
	uint32_t real_val;
	uint32_t reg_val;
};

struct pll_div {
	uint32_t freq_in;
	uint32_t freq_out;
	uint32_t m;
	uint32_t n_i;
	uint32_t n_f;
};


//FOUT =(FIN * N) / (M * (2K+1)) ;		 (N= N_i + 0.2*N_f), K=1;	N_i[0, 1023], N_f[0.2, 1.4], M[1, 64]
static const struct pll_div ac101_pll_div[] = {
	{128000,   AUDIO_CLK_22M, 1,  529, 1},
	{192000,   AUDIO_CLK_22M, 1,  352, 4},
	{256000,   AUDIO_CLK_22M, 1,  264, 3},
	{384000,   AUDIO_CLK_22M, 1,  176, 2},
	{6000000,  AUDIO_CLK_22M, 38, 429, 0},		//<out: 22.5789M>
	{13000000, AUDIO_CLK_22M, 19, 99,  0},		//<out: 22.5789M>
	{19200000, AUDIO_CLK_22M, 25, 88,  1},

	{128000,   AUDIO_CLK_24M, 1,  576, 0},		//24576000/192
	{192000,   AUDIO_CLK_24M, 1,  384, 0},		//24576000/128
	{256000,   AUDIO_CLK_24M, 1,  288, 0},		//24576000/96
	{384000,   AUDIO_CLK_24M, 1,  192, 0},		//24576000/64
	{2048000,  AUDIO_CLK_24M, 1,  36,  0},		//24576000/12
	{6000000,  AUDIO_CLK_24M, 25, 307, 1},
	{13000000, AUDIO_CLK_24M, 42, 238, 1},
	{19200000, AUDIO_CLK_24M, 25, 88,  1},

	{12288000, AUDIO_CLK_24M, 1,  6,   0},		//24576000/2
	{11289600, AUDIO_CLK_22M, 1,  6,   0},		//22579200/2

	{24576000/1,   AUDIO_CLK_24M, 1, 3,   0},	//24576000
	{24576000/4,   AUDIO_CLK_24M, 1, 12,  0},	//6144000
	{24576000/6,   AUDIO_CLK_24M, 1, 18,  0},	//4096000
	{24576000/8,   AUDIO_CLK_24M, 1, 24,  0},	//3072000
	{24576000/16,  AUDIO_CLK_24M, 1, 48,  0},	//1536000
	{24576000/24,  AUDIO_CLK_24M, 1, 72,  0},	//1024000
	{24576000/32,  AUDIO_CLK_24M, 1, 96,  0},	//768000
	{24576000/48,  AUDIO_CLK_24M, 1, 144, 0},	//512000

	{22579200/1,   AUDIO_CLK_22M, 1, 3,   0},	//22579200
	{22579200/4,   AUDIO_CLK_22M, 1, 12,  0},	//5644800
	{22579200/6,   AUDIO_CLK_22M, 1, 18,  0}, 	//3763200
	{22579200/8,   AUDIO_CLK_22M, 1, 24,  0}, 	//2822400
	{22579200/12,  AUDIO_CLK_22M, 1, 36,  0}, 	//1881600
	{22579200/16,  AUDIO_CLK_22M, 1, 48,  0}, 	//1411200
	{22579200/24,  AUDIO_CLK_22M, 1, 72,  0}, 	//940800
	{22579200/32,  AUDIO_CLK_22M, 1, 96,  0}, 	//705600
	{22579200/48,  AUDIO_CLK_22M, 1, 144, 0}, 	//470400
	{22579200/64,  AUDIO_CLK_22M, 1, 192, 0}, 	//352800
	{22579200/96,  AUDIO_CLK_22M, 1, 288, 0}, 	//235200
	{22579200/128, AUDIO_CLK_22M, 1, 384, 0}, 	//176400
	{22579200/192, AUDIO_CLK_22M, 1, 576, 0}, 	//117600
};

static const struct real_val_to_reg_val ac101_sample_rate[] = {
	{8000,   0},
	{11025,  1},
	{12000,  2},
	{16000,  3},
	{22050,  4},
	{24000,  5},
	{32000,  6},
	{44100,  7},
	{48000,  8},
	{96000,  9},
	{192000, 10},
};

static const struct real_val_to_reg_val ac101_sample_res[] = {
	{8,  0},
	{16, 1},
	{20, 2},
	{24, 3},
};

static const struct real_val_to_reg_val ac101_bclk_div[] = {
	{1,   0},
	{2,   1},
	{4,   2},
	{6,   3},
	{8,   4},
	{12,  5},
	{16,  6},
	{24,  7},
	{32,  8},
	{48,  9},
	{64,  10},
	{96,  11},
	{128, 12},
	{192, 13},
};

static const struct real_val_to_reg_val ac101_lrck_bclk_ratio[] = {
	{16,  0},
	{32,  1},
	{64,  2},
	{128, 3},
	{256, 4},
};

static const struct real_val_to_reg_val ac101_amic_vol_level[] = {
	{VOLUME_LEVEL0,  0},
	{VOLUME_LEVEL1,  1},
	{VOLUME_LEVEL2,  2},
	{VOLUME_LEVEL3,  3},
	{VOLUME_LEVEL4,  4},
	{VOLUME_LEVEL5,  5},
	{VOLUME_LEVEL6,  6},
	{VOLUME_LEVEL7,  7},
};

static const struct real_val_to_reg_val ac101_amic_vol_gain[] = {
	{VOLUME_GAIN_0dB,  0},
	{VOLUME_GAIN_30dB, 1},
	{VOLUME_GAIN_33dB, 2},
	{VOLUME_GAIN_36dB, 3},
	{VOLUME_GAIN_39dB, 4},
	{VOLUME_GAIN_42dB, 5},
	{VOLUME_GAIN_45dB, 6},
	{VOLUME_GAIN_48dB, 7},
};

static const struct real_val_to_reg_val ac101_linein_vol_level[] = {
	{VOLUME_LEVEL0,  0},
	{VOLUME_LEVEL1,  1},
	{VOLUME_LEVEL2,  2},
	{VOLUME_LEVEL3,  3},
	{VOLUME_LEVEL4,  4},
	{VOLUME_LEVEL5,  5},
	{VOLUME_LEVEL6,  6},
	{VOLUME_LEVEL7,  7},
};

static const struct real_val_to_reg_val ac101_linein_vol_gain[] = {
	{VOLUME_GAIN_MINUS_12dB, 0},
	{VOLUME_GAIN_MINUS_9dB,  1},
	{VOLUME_GAIN_MINUS_6dB,  2},
	{VOLUME_GAIN_MINUS_3dB,  3},
	{VOLUME_GAIN_0dB,  		 4},
	{VOLUME_GAIN_3dB,  		 5},
	{VOLUME_GAIN_6dB,  		 6},
	{VOLUME_GAIN_9dB,  		 7},
};

static const struct real_val_to_reg_val ac101_hpout_vol_level[] = {
	{VOLUME_LEVEL0,  0},
	{VOLUME_LEVEL1,  2},
	{VOLUME_LEVEL2,  4},
	{VOLUME_LEVEL3,  6},
	{VOLUME_LEVEL4,  8},
	{VOLUME_LEVEL5,  10},
	{VOLUME_LEVEL6,  12},
	{VOLUME_LEVEL7,  14},
	{VOLUME_LEVEL8,  16},
	{VOLUME_LEVEL9,  18},
	{VOLUME_LEVEL10, 20},
	{VOLUME_LEVEL11, 22},
	{VOLUME_LEVEL12, 24},
	{VOLUME_LEVEL13, 26},
	{VOLUME_LEVEL14, 28},
	{VOLUME_LEVEL15, 30},
	{VOLUME_LEVEL16, 32},
	{VOLUME_LEVEL17, 34},
	{VOLUME_LEVEL18, 36},
	{VOLUME_LEVEL19, 38},
	{VOLUME_LEVEL20, 40},
	{VOLUME_LEVEL21, 42},
	{VOLUME_LEVEL22, 44},
	{VOLUME_LEVEL23, 46},
	{VOLUME_LEVEL24, 48},
	{VOLUME_LEVEL25, 50},
	{VOLUME_LEVEL26, 52},
	{VOLUME_LEVEL27, 54},
	{VOLUME_LEVEL28, 56},
	{VOLUME_LEVEL29, 58},
	{VOLUME_LEVEL30, 60},
	{VOLUME_LEVEL31, 63},
};

static const struct real_val_to_reg_val ac101_hpout_vol_gain[] = {
	{VOLUME_GAIN_0dB, 		 63},
	{VOLUME_GAIN_MINUS_1dB,  62},
	{VOLUME_GAIN_MINUS_2dB,  61},
	{VOLUME_GAIN_MINUS_3dB,  60},
	{VOLUME_GAIN_MINUS_4dB,  59},
	{VOLUME_GAIN_MINUS_5dB,  58},
	{VOLUME_GAIN_MINUS_6dB,  57},
	{VOLUME_GAIN_MINUS_7dB,  56},
	{VOLUME_GAIN_MINUS_8dB,  55},
	{VOLUME_GAIN_MINUS_9dB,  54},
	{VOLUME_GAIN_MINUS_10dB, 53},
	{VOLUME_GAIN_MINUS_11dB, 52},
	{VOLUME_GAIN_MINUS_12dB, 51},
	{VOLUME_GAIN_MINUS_13dB, 50},
	{VOLUME_GAIN_MINUS_14dB, 49},
	{VOLUME_GAIN_MINUS_15dB, 48},
	{VOLUME_GAIN_MINUS_16dB, 47},
	{VOLUME_GAIN_MINUS_17dB, 46},
	{VOLUME_GAIN_MINUS_18dB, 45},
	{VOLUME_GAIN_MINUS_19dB, 44},
	{VOLUME_GAIN_MINUS_20dB, 43},
	{VOLUME_GAIN_MINUS_21dB, 42},
	{VOLUME_GAIN_MINUS_22dB, 41},
	{VOLUME_GAIN_MINUS_23dB, 40},
	{VOLUME_GAIN_MINUS_24dB, 39},
	{VOLUME_GAIN_MINUS_25dB, 38},
	{VOLUME_GAIN_MINUS_26dB, 37},
	{VOLUME_GAIN_MINUS_27dB, 36},
	{VOLUME_GAIN_MINUS_28dB, 35},
	{VOLUME_GAIN_MINUS_29dB, 34},
	{VOLUME_GAIN_MINUS_30dB, 33},
	{VOLUME_GAIN_MINUS_31dB, 32},
	{VOLUME_GAIN_MINUS_32dB, 31},
	{VOLUME_GAIN_MINUS_33dB, 30},
	{VOLUME_GAIN_MINUS_34dB, 29},
	{VOLUME_GAIN_MINUS_35dB, 28},
	{VOLUME_GAIN_MINUS_36dB, 27},
	{VOLUME_GAIN_MINUS_37dB, 26},
	{VOLUME_GAIN_MINUS_38dB, 25},
	{VOLUME_GAIN_MINUS_39dB, 24},
	{VOLUME_GAIN_MINUS_40dB, 23},
	{VOLUME_GAIN_MINUS_41dB, 22},
	{VOLUME_GAIN_MINUS_42dB, 21},
	{VOLUME_GAIN_MINUS_43dB, 20},
	{VOLUME_GAIN_MINUS_44dB, 19},
	{VOLUME_GAIN_MINUS_45dB, 18},
	{VOLUME_GAIN_MINUS_46dB, 17},
	{VOLUME_GAIN_MINUS_47dB, 16},
	{VOLUME_GAIN_MINUS_48dB, 15},
	{VOLUME_GAIN_MINUS_49dB, 14},
	{VOLUME_GAIN_MINUS_50dB, 13},
	{VOLUME_GAIN_MINUS_51dB, 12},
	{VOLUME_GAIN_MINUS_52dB, 11},
	{VOLUME_GAIN_MINUS_53dB, 10},
	{VOLUME_GAIN_MINUS_54dB, 9},
	{VOLUME_GAIN_MINUS_55dB, 8},
	{VOLUME_GAIN_MINUS_56dB, 7},
	{VOLUME_GAIN_MINUS_57dB, 6},
	{VOLUME_GAIN_MINUS_58dB, 5},
	{VOLUME_GAIN_MINUS_59dB, 4},
	{VOLUME_GAIN_MINUS_60dB, 3},
	{VOLUME_GAIN_MINUS_61dB, 2},
	{VOLUME_GAIN_MINUS_62dB, 1},
	{VOLUME_GAIN_MUTE,		 0},
};

static const struct real_val_to_reg_val ac101_spkout_vol_level[] = {
	{VOLUME_LEVEL0,  0},
	{VOLUME_LEVEL1,  1},
	{VOLUME_LEVEL2,  2},
	{VOLUME_LEVEL3,  3},
	{VOLUME_LEVEL4,  4},
	{VOLUME_LEVEL5,  5},
	{VOLUME_LEVEL6,  6},
	{VOLUME_LEVEL7,  7},
	{VOLUME_LEVEL8,  8},
	{VOLUME_LEVEL9,  9},
	{VOLUME_LEVEL10, 10},
	{VOLUME_LEVEL11, 11},
	{VOLUME_LEVEL12, 12},
	{VOLUME_LEVEL13, 13},
	{VOLUME_LEVEL14, 14},
	{VOLUME_LEVEL15, 15},
	{VOLUME_LEVEL16, 16},
	{VOLUME_LEVEL17, 17},
	{VOLUME_LEVEL18, 18},
	{VOLUME_LEVEL19, 19},
	{VOLUME_LEVEL20, 20},
	{VOLUME_LEVEL21, 21},
	{VOLUME_LEVEL22, 22},
	{VOLUME_LEVEL23, 23},
	{VOLUME_LEVEL24, 24},
	{VOLUME_LEVEL25, 25},
	{VOLUME_LEVEL26, 26},
	{VOLUME_LEVEL27, 27},
	{VOLUME_LEVEL28, 28},
	{VOLUME_LEVEL29, 29},
	{VOLUME_LEVEL30, 30},
	{VOLUME_LEVEL31, 31},
};

static const struct real_val_to_reg_val ac101_spkout_vol_gain[] = {
	{VOLUME_GAIN_0dB, 		 31},
	{VOLUME_GAIN_MINUS_3dB,  29},
	{VOLUME_GAIN_MINUS_6dB,  27},
	{VOLUME_GAIN_MINUS_9dB,  25},
	{VOLUME_GAIN_MINUS_12dB, 23},
	{VOLUME_GAIN_MINUS_15dB, 21},
	{VOLUME_GAIN_MINUS_18dB, 19},
	{VOLUME_GAIN_MINUS_21dB, 17},
	{VOLUME_GAIN_MINUS_24dB, 15},
	{VOLUME_GAIN_MINUS_27dB, 13},
	{VOLUME_GAIN_MINUS_30dB, 11},
	{VOLUME_GAIN_MINUS_33dB, 9},
	{VOLUME_GAIN_MINUS_36dB, 7},
	{VOLUME_GAIN_MINUS_39dB, 5},
	{VOLUME_GAIN_MINUS_42dB, 3},
	{VOLUME_GAIN_MUTE,		 1},
};

/*
* DRC Control Parameters:  	Threshold = -20dB, Offset = 0, delta_x/delta_y = 2
* RMS Filter (ms): 			Left Average Time = 1, Right Average Time = 1
* Smooth Gain Time(ms): 	Smooth Attack Time = 1, Smooth Decay Time = 100
*	y=0.5x-10,  x=[-20, 0]
*/
static const struct real_val_to_reg_val ac101_drc_params[] = {
	{0xa3, 0x000B},
	{0xa4, 0x77F0},
	{0xa5, 0x000B},
	{0xa6, 0x77F0},
	{0xa7, 0x0000},
	{0xa8, 0x1E08},
	{0xa9, 0x000B},
	{0xaa, 0x77F0},
	{0xab, 0x0352},
	{0xac, 0x69E0},
	{0xad, 0x0780},
	{0xae, 0x0000},
	{0xaf, 0x0100},
	{0xb0, 0x0000},
};


/*** Base read/write Interface ***/
static int ac101_read(uint8_t reg, uint16_t *rt_value, I2C_ID i2c_id)
{
	uint8_t reg_val[2], read_success_bytes_cnt;

	read_success_bytes_cnt =  AC101_I2C_READ(i2c_id, AC101_I2C_ADDR, reg, I2C_MEMADDR_SIZE_8BIT, reg_val, 2);
	if(read_success_bytes_cnt == 2){
		*rt_value = (reg_val[0] << 8) | reg_val[1];
	}

	//return read success bytes
	return read_success_bytes_cnt;
}

static int ac101_write(uint8_t reg, uint16_t value, I2C_ID i2c_id)
{
	uint8_t reg_val[2];

	reg_val[0] = (value & 0xFF00) >> 8;
	reg_val[1] = value & 0xFF;

	//return wirte success bytes
	return AC101_I2C_WRITE(i2c_id, AC101_I2C_ADDR, reg, I2C_MEMADDR_SIZE_8BIT, reg_val, 2);
}

static int ac101_update_bits(uint8_t reg, uint16_t mask, uint16_t value)
{
	uint16_t val_old,val_new;

	if(!ac101_priv || ac101_priv->i2c_id >= I2C_NUM){
		AC101_ERR("AC101 hasn't registered or I2C BUS Number error!\n");
		return -1;
	}

	ac101_read(reg, &val_old, ac101_priv->i2c_id);
	val_new = (val_old & ~mask) | (value & mask);
	if(val_new != val_old){
		ac101_write(reg, val_new, ac101_priv->i2c_id);
	}

	return 0;
}


/*** Common init interface  ***/
static void ac101_codec_reset(void)
{
	/* AC101 soft reset */
	AC101_DBG("AC101 reset all register to their default value\n\n");
	ac101_write(CHIP_AUDIO_RST, 0x0101, ac101_priv->i2c_id);
}

static void ac101_drc_enable(bool enable)
{
	AC101_ALWAYS("Play DRC %s\n", enable ? "enable" : "disable");

	//DAC HPF/DRC Module CLK Enable and Module de-asserted
	ac101_update_bits(MOD_CLK_EN,	0x1<<HPF_DRC_MODCLK_EN_BIT,   !!enable<<HPF_DRC_MODCLK_EN_BIT);
	ac101_update_bits(MOD_RST_CTRL, 0x1<<HPF_DRC_MODRST_CTRL_BIT, !!enable<<HPF_DRC_MODRST_CTRL_BIT);

	//DRC Module Enable
	ac101_update_bits(DAC_DAP_CTRL, 0x1<<DRC_EN_BIT, !!enable<<DRC_EN_BIT);

	//DAC DRC Enable
	ac101_update_bits(DAC_DRC_EN_CTRL, 0x1<<DAC_DRC_EN_BIT, !!enable<<DAC_DRC_EN_BIT);

	//DRC Params Config
	if(enable){
		for(uint32_t i=0; i<HAL_ARRAY_SIZE(ac101_drc_params); i++){
			ac101_write(ac101_drc_params[i].real_val, ac101_drc_params[i].reg_val, ac101_priv->i2c_id);
		}
	}
}

static void ac101_hw_common_init(Audio_Stream_Dir dir)
{
	//I2S1CLK Enable, SYSCLK Enable
	ac101_update_bits(SYSCLK_CTRL, 0x1<<I2S1CLK_EN_BIT | 0x1<<SYSCLK_EN_BIT, 0x1<<I2S1CLK_EN_BIT | 0x1<<SYSCLK_EN_BIT);

	//I2S1 Module CLK Enable and Module de-asserted
	ac101_update_bits(MOD_CLK_EN,	0x1<<I2S1_MODCLK_EN_BIT,   0x1<<I2S1_MODCLK_EN_BIT);
	ac101_update_bits(MOD_RST_CTRL, 0x1<<I2S1_MODRST_CTRL_BIT, 0x1<<I2S1_MODRST_CTRL_BIT);

	//Enable These Bist To Prevent Leakage From LDOIN
	ac101_update_bits(ADDA_TUNE1, 0x1<<VOL_ZERO_CROSS_EN_BIT, 0x0<<VOL_ZERO_CROSS_EN_BIT);
	ac101_update_bits(ADDA_TUNE3, 0x1<<INNER_OSC_EN_BIT, 0x1<<INNER_OSC_EN_BIT);

	if(dir == PCM_OUT){
		//DAC DRC Enable
		if(ac101_priv->drc_en)	ac101_drc_enable(true);

		//DAC DIG Module CLK Enable and Module de-asserted
		ac101_update_bits(MOD_CLK_EN,	0x1<<DAC_DIG_MODCLK_EN_BIT,   0x1<<DAC_DIG_MODCLK_EN_BIT);
		ac101_update_bits(MOD_RST_CTRL, 0x1<<DAC_DIG_MODRST_CTRL_BIT, 0x1<<DAC_DIG_MODRST_CTRL_BIT);

		//I2S1 RX Slot0 Input Mux Source Select
		ac101_update_bits(I2S1_SDIN_CTRL, 0x3<<I2S1_RX_SLOT0_L_SRC_BIT | 0x3<<I2S1_RX_SLOT0_R_SRC_BIT,\
										  0x0<<I2S1_RX_SLOT0_L_SRC_BIT | 0x0<<I2S1_RX_SLOT0_R_SRC_BIT);

		//DAC Mixer Source Config
		ac101_update_bits(DAC_MXR_SRC, 0x1<<DACL_MXR_SRC_I2S1_DAC0L_BIT | 0x1<<DACR_MXR_SRC_I2S1_DAC0R_BIT,\
									   0x1<<DACL_MXR_SRC_I2S1_DAC0L_BIT | 0x1<<DACR_MXR_SRC_I2S1_DAC0R_BIT);

		//Headphone L/R Input Source select Output Mixer L/R
		ac101_update_bits(HPOUT_CTRL, 0x1<<HPOUT_PA_R_SRC_SEL_BIT | 0x1<<HPOUT_PA_L_SRC_SEL_BIT,\
									  0x1<<HPOUT_PA_R_SRC_SEL_BIT | 0x1<<HPOUT_PA_L_SRC_SEL_BIT);
		//Headphone Calibration CLK Freq Select
		ac101_update_bits(SPKOUT_CTRL, 0x7<<HPOUT_CALI_CLK_BIT, 0x7<<HPOUT_CALI_CLK_BIT);

		//DAC Digital Enable
		ac101_update_bits(DAC_DIG_CTRL, 0x1<<DAC_DIG_EN_BIT, 0x1<<DAC_DIG_EN_BIT);

	} else {

		//ADC DIG Module CLK Enable and Module de-asserted
		ac101_update_bits(MOD_CLK_EN,	0x1<<ADC_DIG_MODCLK_EN_BIT,   0x1<<ADC_DIG_MODCLK_EN_BIT);
		ac101_update_bits(MOD_RST_CTRL, 0x1<<ADC_DIG_MODRST_CTRL_BIT, 0x1<<ADC_DIG_MODRST_CTRL_BIT);

		//ADC HPF Module CLK Enable and Module de-asserted
		ac101_update_bits(MOD_CLK_EN,   0x1<<HPF_AGC_MODCLK_EN_BIT,   0x1<<HPF_AGC_MODCLK_EN_BIT);
		ac101_update_bits(MOD_RST_CTRL, 0x1<<HPF_AGC_MODRST_CTRL_BIT, 0x1<<HPF_AGC_MODRST_CTRL_BIT);

		//I2S1 ADC Mixer L/R Source Select ADC L/R
		ac101_update_bits(I2S1_MXR_SRC, 0x1<<I2S1_ADCL0_MXR_SRC_ADCL_BIT | 0x1<<I2S1_ADCR0_MXR_SRC_ADCR_BIT,\
										0x1<<I2S1_ADCL0_MXR_SRC_ADCL_BIT | 0x1<<I2S1_ADCR0_MXR_SRC_ADCR_BIT);
		//I2S1 TX Slot0 Output Mux L/R Source Select I2S1_ADCL/R0
		ac101_update_bits(I2S1_SDOUT_CTRL, 0x3<<I2S1_TX_SLOT0_L_SRC_BIT | 0x3<<I2S1_TX_SLOT0_R_SRC_BIT,\
										   0x0<<I2S1_TX_SLOT0_L_SRC_BIT | 0x0<<I2S1_TX_SLOT0_R_SRC_BIT);

		//ADC Digital Enable
		ac101_update_bits(ADC_DIG_CTRL, 0x1<<ADC_DIG_EN_BIT, 0x1<<ADC_DIG_EN_BIT);

		//ADC HPF Enable
		ac101_update_bits(ADCL_DAP_CTRL, 0x1<<ADCL_HPF_EN_BIT, 0x1<<ADCL_HPF_EN_BIT);
		ac101_update_bits(ADCR_DAP_CTRL, 0x1<<ADCR_HPF_EN_BIT, 0x1<<ADCR_HPF_EN_BIT);
	}
}

static void ac101_hw_common_deinit(Audio_Stream_Dir dir)
{
	if(dir == PCM_OUT){
		//DAC DRC Disable
		if(ac101_priv->drc_en)	ac101_drc_enable(false);

		//DAC Analog and Output Mixer Disable
		ac101_update_bits(DAC_ANA_CTRL, 0x1<<DAC_R_ANA_EN_BIT | 0x1<<DAC_L_ANA_EN_BIT, 0x0<<DAC_R_ANA_EN_BIT | 0x0<<DAC_L_ANA_EN_BIT);
		ac101_update_bits(DAC_ANA_CTRL, 0x1<<OUT_MXR_R_ANA_EN_BIT | 0x1<<OUT_MXR_L_ANA_EN_BIT | 0xf<<HPOUT_DC_OFFSET_BIT,\
										0x0<<OUT_MXR_R_ANA_EN_BIT | 0x0<<OUT_MXR_L_ANA_EN_BIT | 0x0<<HPOUT_DC_OFFSET_BIT);

		//I2S1 RX Slot0 Disable
		ac101_update_bits(I2S1_SDIN_CTRL, 0x1<<I2S1_RX_SLOT0_L_EN_BIT | 0x1<<I2S1_RX_SLOT0_R_EN_BIT,\
										  0x0<<I2S1_RX_SLOT0_L_EN_BIT | 0x0<<I2S1_RX_SLOT0_R_EN_BIT);

		//DAC Mixer Source Config
		ac101_update_bits(DAC_MXR_SRC, 0x1<<DACL_MXR_SRC_I2S1_DAC0L_BIT | 0x1<<DACR_MXR_SRC_I2S1_DAC0R_BIT,\
									   0x0<<DACL_MXR_SRC_I2S1_DAC0L_BIT | 0x0<<DACR_MXR_SRC_I2S1_DAC0R_BIT);

		//DAC Digital Disable
		ac101_update_bits(DAC_DIG_CTRL, 0x1<<DAC_DIG_EN_BIT, 0x0<<DAC_DIG_EN_BIT);

		//DAC DIG Module CLK Disable and Module asserted
		ac101_update_bits(MOD_CLK_EN,	0x1<<DAC_DIG_MODCLK_EN_BIT,   0x0<<DAC_DIG_MODCLK_EN_BIT);
		ac101_update_bits(MOD_RST_CTRL, 0x1<<DAC_DIG_MODRST_CTRL_BIT, 0x0<<DAC_DIG_MODRST_CTRL_BIT);

	} else {

		//I2S1 ADC Mixer L/R Source Disable
		ac101_update_bits(I2S1_MXR_SRC, 0x1<<I2S1_ADCL0_MXR_SRC_ADCL_BIT | 0x1<<I2S1_ADCR0_MXR_SRC_ADCR_BIT,\
										0x0<<I2S1_ADCL0_MXR_SRC_ADCL_BIT | 0x0<<I2S1_ADCR0_MXR_SRC_ADCR_BIT);

		//ADC DIG Module CLK Disable and Module asserted
		ac101_update_bits(MOD_CLK_EN,	0x1<<ADC_DIG_MODCLK_EN_BIT,   0x0<<ADC_DIG_MODCLK_EN_BIT);
		ac101_update_bits(MOD_RST_CTRL, 0x1<<ADC_DIG_MODRST_CTRL_BIT, 0x0<<ADC_DIG_MODRST_CTRL_BIT);

		//ADC HPF Module CLK Disable and Module asserted
		ac101_update_bits(MOD_CLK_EN,   0x1<<HPF_AGC_MODCLK_EN_BIT,   0x0<<HPF_AGC_MODCLK_EN_BIT);
		ac101_update_bits(MOD_RST_CTRL, 0x1<<HPF_AGC_MODRST_CTRL_BIT, 0x0<<HPF_AGC_MODRST_CTRL_BIT);

		//ADC HPF Disable
		ac101_update_bits(ADCL_DAP_CTRL, 0x1<<ADCL_HPF_EN_BIT, 0x0<<ADCL_HPF_EN_BIT);
		ac101_update_bits(ADCR_DAP_CTRL, 0x1<<ADCR_HPF_EN_BIT, 0x0<<ADCR_HPF_EN_BIT);

		//ADC Digital Disable
		ac101_update_bits(ADC_DIG_CTRL, 0x1<<ADC_DIG_EN_BIT, 0x0<<ADC_DIG_EN_BIT);
	}
}

static void ac101_set_amic(bool enable)
{
	AC101_ALWAYS("Route(cap): amic %s\n",enable ? "Enable" : "Disable");

	//ADC/DMIC MUX set ADC
	ac101_update_bits(ADC_DIG_CTRL, 0x1<<ADC_DMIC_SEL_BIT, 0x0<<ADC_DMIC_SEL_BIT);

	if(ac101_priv->amic_ch_sel == 0){			//Use MIC1 & MIC2, connect to ADC Mixer L/R
		//MIC1&2 PGA Enable and MIC2 Pin Enable
		ac101_update_bits(ADC_PGA_CTRL, 0x1<<ADC_MIC1_PGA_EN_BIT | 0x1<<ADC_MIC2_PGA_EN_BIT | 0x1<<ADC_MIC2_PIN_EN_BIT,\
						  !!enable<<ADC_MIC1_PGA_EN_BIT | !!enable<<ADC_MIC2_PGA_EN_BIT | !!enable<<ADC_MIC2_PIN_EN_BIT);
		//ADC Mixer Source select: MIC1/2->Mixer L/R
		ac101_update_bits(ADC_MXR_SRC, 0x1<<ADCR_MXR_SRC_MIC2_BIT | 0x1<<ADCL_MXR_SRC_MIC1_BIT,\
						  !!enable<<ADCR_MXR_SRC_MIC2_BIT | !!enable<<ADCL_MXR_SRC_MIC1_BIT);
		//ADC L/R Analog & MICBIAS Enable
		ac101_update_bits(ADC_ANA_CTRL, 0x1<<ADC_R_ANA_EN_BIT | 0x1<<ADC_L_ANA_EN_BIT | 0x1<<ADC_MICBIAS_EN_BIT,\
						  !!enable<<ADC_R_ANA_EN_BIT | !!enable<<ADC_L_ANA_EN_BIT | !!enable<<ADC_MICBIAS_EN_BIT);

	} else if(ac101_priv->amic_ch_sel == 1){	//Use MIC1, connect to ADC Mixer L

		//MIC1 PGA Enable
		ac101_update_bits(ADC_PGA_CTRL, 0x1<<ADC_MIC1_PGA_EN_BIT, !!enable<<ADC_MIC1_PGA_EN_BIT);
		//ADC Mixer Source select: MIC1->Mixer L
		ac101_update_bits(ADC_MXR_SRC, 0x1<<ADCL_MXR_SRC_MIC1_BIT, !!enable<<ADCL_MXR_SRC_MIC1_BIT);
		//ADC L Analog & MICBIAS Enable
		ac101_update_bits(ADC_ANA_CTRL, 0x1<<ADC_L_ANA_EN_BIT | 0x1<<ADC_MICBIAS_EN_BIT, !!enable<<ADC_L_ANA_EN_BIT | !!enable<<ADC_MICBIAS_EN_BIT);

	} else if(ac101_priv->amic_ch_sel == 2){	//Use MIC2, connect to ADC Mixer L

		//MIC2 PGA & Pin Enable
		ac101_update_bits(ADC_PGA_CTRL, 0x1<<ADC_MIC2_PGA_EN_BIT | 0x1<<ADC_MIC2_PIN_EN_BIT, !!enable<<ADC_MIC2_PGA_EN_BIT | !!enable<<ADC_MIC2_PIN_EN_BIT);
		//ADC Mixer Source select: MIC2->Mixer L
		ac101_update_bits(ADC_MXR_SRC, 0x1<<ADCL_MXR_SRC_MIC2_BIT, !!enable<<ADCL_MXR_SRC_MIC2_BIT);
		//ADC L Analog & MICBIAS Enable
		ac101_update_bits(ADC_ANA_CTRL, 0x1<<ADC_L_ANA_EN_BIT | 0x1<<ADC_MICBIAS_EN_BIT, !!enable<<ADC_L_ANA_EN_BIT | !!enable<<ADC_MICBIAS_EN_BIT);
	}
}

static void ac101_set_linein(bool enable)
{
	if(ac101_priv->amic_ch_sel == 0){
		AC101_ERR("Use MIC1 & MIC2, don't allow to use LINEIN at the same time!\n");
		return;
	}

	AC101_ALWAYS("Route(cap): linein %s\n",enable ? "Enable" : "Disable");

	//ADC/DMIC MUX set ADC
	ac101_update_bits(ADC_DIG_CTRL, 0x1<<ADC_DMIC_SEL_BIT, 0x0<<ADC_DMIC_SEL_BIT);

	//ADC Mixer Source select: LINEIN L-R->Mixer R
	ac101_update_bits(ADC_MXR_SRC, 0x1<<ADCR_MXR_SRC_LINEIN_LR_BIT, !!enable<<ADCR_MXR_SRC_LINEIN_LR_BIT);
	//ADC R Analog Enable
	ac101_update_bits(ADC_ANA_CTRL, 0x1<<ADC_R_ANA_EN_BIT, !!enable<<ADC_R_ANA_EN_BIT);
}

static void ac101_set_dmic(bool enable)
{
	AC101_ALWAYS("Route(cap): dmic %s\n",enable ? "Enable" : "Disable");

	//ADC/DMIC MUX set DMIC
	ac101_update_bits(ADC_DIG_CTRL, 0x1<<ADC_DMIC_SEL_BIT, 0x1<<ADC_DMIC_SEL_BIT);
}

static void ac101_set_headphone(bool enable)
{
	AC101_ALWAYS("Route(play): headphone %s\n",enable ? "Enable" : "Disable");

	//DAC Analog and Output Mixer L&R Enable
	ac101_update_bits(DAC_ANA_CTRL, 0x1<<DAC_R_ANA_EN_BIT | 0x1<<DAC_L_ANA_EN_BIT, !!enable<<DAC_R_ANA_EN_BIT | !!enable<<DAC_L_ANA_EN_BIT);
	ac101_update_bits(DAC_ANA_CTRL, 0x1<<OUT_MXR_R_ANA_EN_BIT | 0x1<<OUT_MXR_L_ANA_EN_BIT | 0xf<<HPOUT_DC_OFFSET_BIT,\
					  !!enable<<OUT_MXR_R_ANA_EN_BIT | !!enable<<OUT_MXR_L_ANA_EN_BIT | (0xf*!!enable)<<HPOUT_DC_OFFSET_BIT);

	//Headphone L/R Unmute and HP PA Enable
	ac101_update_bits(HPOUT_CTRL, 0x1<<HPOUT_PA_R_UNMUTE_BIT | 0x1<<HPOUT_PA_L_UNMUTE_BIT | 0x1<<HPOUT_PA_EN_BIT,\
								  !!enable<<HPOUT_PA_R_UNMUTE_BIT | !!enable<<HPOUT_PA_L_UNMUTE_BIT | !!enable<<HPOUT_PA_EN_BIT);
}

static void ac101_set_speaker(bool enable)
{
	AC101_ALWAYS("Route(play): speaker %s\n",enable ? "Enable" : "Disable");

	if(ac101_priv->spkout_ch_sel == 0){			//dobule channel output
		//DAC Analog and Output Mixer L&R Enable
		ac101_update_bits(DAC_ANA_CTRL, 0x1<<DAC_R_ANA_EN_BIT | 0x1<<DAC_L_ANA_EN_BIT, !!enable<<DAC_R_ANA_EN_BIT | !!enable<<DAC_L_ANA_EN_BIT);
		ac101_update_bits(DAC_ANA_CTRL, 0x1<<OUT_MXR_R_ANA_EN_BIT | 0x1<<OUT_MXR_L_ANA_EN_BIT | 0xf<<HPOUT_DC_OFFSET_BIT,\
						  !!enable<<OUT_MXR_R_ANA_EN_BIT | !!enable<<OUT_MXR_L_ANA_EN_BIT | (0xf*!!enable)<<HPOUT_DC_OFFSET_BIT);

		//Speaker L/R Input Source select Output Mixer L/R
		ac101_update_bits(SPKOUT_CTRL, 0x1<<SPKOUT_R_SRC_SEL_BIT | 0x1<<SPKOUT_L_SRC_SEL_BIT, 0x0<<SPKOUT_R_SRC_SEL_BIT | 0x0<<SPKOUT_L_SRC_SEL_BIT);
		//Speaker L/R Enable
		ac101_update_bits(SPKOUT_CTRL, 0x1<<SPKOUT_R_EN_BIT | 0x1<<SPKOUT_L_EN_BIT, !!enable<<SPKOUT_R_EN_BIT | !!enable<<SPKOUT_L_EN_BIT);

	} else if(ac101_priv->spkout_ch_sel == 1){	//left channnel output

		//DAC Analog and Output Mixer L Enable
		ac101_update_bits(DAC_ANA_CTRL, 0x1<<DAC_L_ANA_EN_BIT, !!enable<<DAC_L_ANA_EN_BIT);
		ac101_update_bits(DAC_ANA_CTRL, 0x1<<OUT_MXR_L_ANA_EN_BIT | 0xf<<HPOUT_DC_OFFSET_BIT,\
						  !!enable<<OUT_MXR_L_ANA_EN_BIT | (0xf*!!enable)<<HPOUT_DC_OFFSET_BIT);

		//Speaker L Input Source select Output Mixer L+R
		ac101_update_bits(SPKOUT_CTRL, 0x1<<SPKOUT_L_SRC_SEL_BIT, 0x1<<SPKOUT_L_SRC_SEL_BIT);
		//Speaker L Enable, R Disable
		ac101_update_bits(SPKOUT_CTRL, 0x1<<SPKOUT_L_EN_BIT, !!enable<<SPKOUT_L_EN_BIT);

	} else if(ac101_priv->spkout_ch_sel == 2){	//right channel output

		//DAC Analog and Output Mixer R Enable
		ac101_update_bits(DAC_ANA_CTRL, 0x1<<DAC_R_ANA_EN_BIT, !!enable<<DAC_R_ANA_EN_BIT);
		ac101_update_bits(DAC_ANA_CTRL, 0x1<<OUT_MXR_R_ANA_EN_BIT | 0xf<<HPOUT_DC_OFFSET_BIT,\
						  !!enable<<OUT_MXR_R_ANA_EN_BIT | (0xf*!!enable)<<HPOUT_DC_OFFSET_BIT);

		//Speaker R Input Source select Output Mixer L+R
		ac101_update_bits(SPKOUT_CTRL, 0x1<<SPKOUT_R_SRC_SEL_BIT, 0x1<<SPKOUT_R_SRC_SEL_BIT);
		//Speaker R Enable, L Disable
		ac101_update_bits(SPKOUT_CTRL, 0x1<<SPKOUT_R_EN_BIT, !!enable<<SPKOUT_R_EN_BIT);
	}
}

static int ac101_set_route(Audio_Device device, bool enable)
{
	switch(device){
		case AUDIO_IN_DEV_AMIC:
			ac101_set_amic(enable);
			break;
		case AUDIO_IN_DEV_LINEIN:
			ac101_set_linein(enable);
			break;
		case AUDIO_IN_DEV_DMIC:
			ac101_set_dmic(enable);
			break;
		case AUDIO_OUT_DEV_HP:
			ac101_set_headphone(enable);
			break;
		case AUDIO_OUT_DEV_SPK:
			ac101_set_speaker(enable);
			break;
		default:
			AC101_ERR("Invalid Audio Device-[0x%08x]!\n",device);
			return HAL_INVALID;
	}

	return HAL_OK;
}

static int ac101_set_pll(Codec_Pllclk_Src pllclk_src, uint32_t freq_in, uint32_t freq_out)
{
	uint16_t i, m, n_f;
	uint32_t n_i;
	AC101_DBG("%s\n",__FUNCTION__);

	if (!freq_out)	return HAL_INVALID;

	if (freq_in < 128000 || freq_in > 24576000) {
		AC101_ERR("AC101 PLLCLK source input freq only support [128K,24M],while now %u\n\n",freq_in);
		return HAL_INVALID;
	} else if ((freq_in == AUDIO_CLK_24M || freq_in == AUDIO_CLK_22M) && pllclk_src == PLLCLK_SRC_MCLK) {
		AC101_DBG("AC101 don't need to use PLL, I2S1CLK source select MCLK, SYSCLK Source default select I2S1CLK\n\n");
		//I2S1CLK Source Select MCLK1
		ac101_update_bits(SYSCLK_CTRL, 0x3<<I2S1CLK_SRC_BIT, 0x0<<I2S1CLK_SRC_BIT);
		return HAL_OK;
	}

	/* PLL Clock Source Select */
	switch(pllclk_src){
		case PLLCLK_SRC_MCLK:
			AC101_DBG("AC101 PLLCLK input source select I2S1 MCLK\n");
			ac101_update_bits(SYSCLK_CTRL, 0x3<<PLLCLK_SRC_BIT, 0x0<<PLLCLK_SRC_BIT);
			break;
		case PLLCLK_SRC_BCLK:
			AC101_DBG("AC101 PLLCLK input source select I2S1 BCLK\n");
			ac101_update_bits(SYSCLK_CTRL, 0x3<<PLLCLK_SRC_BIT, 0x2<<PLLCLK_SRC_BIT);
			break;
		default:
			AC101_ERR("AC101 PLLCLK source config error:%d\n\n",pllclk_src);
			return HAL_INVALID;
	}

	//FOUT =(FIN * N) / (M * (2K+1)) ;		 (N= N_i + 0.2*N_f), K=1;
	for(i=0; i<HAL_ARRAY_SIZE(ac101_pll_div); i++){
		if(ac101_pll_div[i].freq_in == freq_in && ac101_pll_div[i].freq_out == freq_out){
			m = ac101_pll_div[i].m;
			n_i = ac101_pll_div[i].n_i;
			n_f = ac101_pll_div[i].n_f;
			break;
		}
	}
	if(i == HAL_ARRAY_SIZE(ac101_pll_div)){
		AC101_ERR("AC101 don't match PLLCLK freq_in and freq_out table\n\n");
		return HAL_INVALID;
	}

	/* Config PLL DIV param M/N_I/N_F */
	ac101_update_bits(PLL_CTRL1, 0x3f<<PLL_POSTDIV_M_BIT, m<<PLL_POSTDIV_M_BIT);
	ac101_update_bits(PLL_CTRL2, 0x3ff<<PLL_PREDIV_NI_BIT | 0x7<<PLL_POSTDIV_NF_BIT, n_i<<PLL_PREDIV_NI_BIT | n_f<<PLL_POSTDIV_NF_BIT);

	//PLL Module Enable
	ac101_update_bits(PLL_CTRL2, 0x1<<PLL_EN_BIT, 0x1<<PLL_EN_BIT);

	//PLLCLK Enable
	ac101_update_bits(SYSCLK_CTRL, 0x1<<PLLCLK_EN_BIT, 0x1<<PLLCLK_EN_BIT);

	return HAL_OK;
}


/*** codec_dai_ops interface ***/
static int ac101_dai_set_sysclk(Codec_Sysclk_Src sysclk_src, Codec_Pllclk_Src pllclk_src, uint32_t pll_freq_in, uint32_t sample_rate)
{
	int ret = HAL_OK;
	AC101_DBG("--->%s\n",__FUNCTION__);

	switch(sysclk_src){
		case SYSCLK_SRC_MCLK:
			AC101_DBG("SYSCLK source select MCLK\n");
			ac101_update_bits(SYSCLK_CTRL, 0x3<<I2S1CLK_SRC_BIT, 0x0<<I2S1CLK_SRC_BIT);	//I2S1CLK Source Select MCLK1
			break;
		case SYSCLK_SRC_PLL:
			AC101_DBG("SYSCLK source select PLL\n");
			ac101_update_bits(SYSCLK_CTRL, 0x3<<I2S1CLK_SRC_BIT, 0x3<<I2S1CLK_SRC_BIT);	//I2S1CLK Source Select PLL
			ret = ac101_set_pll(pllclk_src, pll_freq_in, sample_rate%1000 ? AUDIO_CLK_22M : AUDIO_CLK_24M);
			break;
		default:
			AC101_ERR("SYSCLK source select Error!\n");
			return HAL_INVALID;
	}

	return ret;
}

static int ac101_dai_set_fmt(uint32_t fmt)
{
	AC101_DBG("--->%s\n",__FUNCTION__);

	/* AC101 config Master/Slave mode */
	switch (fmt & I2S_ROLE_MASK) {
		case DAIFMT_CBM_CFM:	//AC101 Master
			AC101_DBG("AC101 set to work as Master\n");
			ac101_update_bits(I2S1_CLK_CTRL, 0x1<<I2S1_ROLE_BIT, 0x0<<I2S1_ROLE_BIT);	//BCLK & LRCK output
			break;
		case DAIFMT_CBS_CFS:	//AC101 Slave
			AC101_DBG("AC101 set to work as Slave\n");
			ac101_update_bits(I2S1_CLK_CTRL, 0x1<<I2S1_ROLE_BIT, 0x1<<I2S1_ROLE_BIT);	//BCLK & LRCK input
			break;
		default:
			AC101_ERR("AC101 Master/Slave mode config error:%u\n\n",(fmt & I2S_ROLE_MASK)>>0);
			return HAL_INVALID;
	}

	/* AC101 config I2S/LJ/RJ/PCM format */
	switch (fmt & I2S_FORMAT_MASK) {
		case DAIFMT_I2S:
			AC101_DBG("AC101 config I2S format\n");
			ac101_update_bits(I2S1_CLK_CTRL, 0x3<<I2S1_MOD_BIT, 0x0<<I2S1_MOD_BIT);
			break;
		case DAIFMT_LEFT_J:
			AC101_DBG("AC101 config LEFT-JUSTIFIED format\n");
			ac101_update_bits(I2S1_CLK_CTRL, 0x3<<I2S1_MOD_BIT, 0x1<<I2S1_MOD_BIT);
			break;
		case DAIFMT_RIGHT_J:
			AC101_DBG("AC101 config RIGHT-JUSTIFIED format\n");
			ac101_update_bits(I2S1_CLK_CTRL, 0x3<<I2S1_MOD_BIT, 0x2<<I2S1_MOD_BIT);
			break;
		case DAIFMT_DSP_A:
			AC101_DBG("AC101 config PCM-A format\n");
			ac101_update_bits(I2S1_CLK_CTRL, 0x3<<I2S1_MOD_BIT, 0x3<<I2S1_MOD_BIT);
			break;
		case DAIFMT_DSP_B:
		default:
			AC101_ERR("AC101 I2S format config error:%u\n\n",(fmt & I2S_FORMAT_MASK)>>4);
			return HAL_INVALID;
	}

	/* AC101 config BCLK&LRCK polarity */
	switch (fmt & I2S_POLARITY_MASK) {
		case DAIFMT_NB_NF:
			AC101_DBG("AC101 config BCLK&LRCK polarity: BCLK_normal,LRCK_normal\n");
			ac101_update_bits(I2S1_CLK_CTRL, 0x1<<I2S1_BCLK_POLARITY_BIT | 0x1<<I2S1_LRCK_POLARITY_BIT,\
											 0x0<<I2S1_BCLK_POLARITY_BIT | 0x0<<I2S1_LRCK_POLARITY_BIT);
			break;
		case DAIFMT_NB_IF:
			AC101_DBG("AC101 config BCLK&LRCK polarity: BCLK_normal,LRCK_invert\n");
			ac101_update_bits(I2S1_CLK_CTRL, 0x1<<I2S1_BCLK_POLARITY_BIT | 0x1<<I2S1_LRCK_POLARITY_BIT,\
											 0x0<<I2S1_BCLK_POLARITY_BIT | 0x1<<I2S1_LRCK_POLARITY_BIT);
			break;
		case DAIFMT_IB_NF:
			AC101_DBG("AC101 config BCLK&LRCK polarity: BCLK_invert,LRCK_normal\n");
			ac101_update_bits(I2S1_CLK_CTRL, 0x1<<I2S1_BCLK_POLARITY_BIT | 0x1<<I2S1_LRCK_POLARITY_BIT,\
											 0x1<<I2S1_BCLK_POLARITY_BIT | 0x0<<I2S1_LRCK_POLARITY_BIT);
			break;
		case DAIFMT_IB_IF:
			AC101_DBG("AC101 config BCLK&LRCK polarity: BCLK_invert,LRCK_invert\n");
			ac101_update_bits(I2S1_CLK_CTRL, 0x1<<I2S1_BCLK_POLARITY_BIT | 0x1<<I2S1_LRCK_POLARITY_BIT,\
											 0x1<<I2S1_BCLK_POLARITY_BIT | 0x1<<I2S1_LRCK_POLARITY_BIT);
			break;
		default:
			AC101_ERR("AC101 config BCLK/LRCLK polarity error:%u\n\n",(fmt & I2S_POLARITY_MASK)>>8);
			return HAL_INVALID;
	}

	return HAL_OK;
}

static int ac101_dai_set_volume(Audio_Device device, uint16_t volume)
{
	uint32_t i,reg_val=0;
	uint16_t vol_set_flag, vol_set_value, vol_array_size=0;
	const struct real_val_to_reg_val *ac101_vol=NULL;
	AC101_DBG("--->%s\n",__FUNCTION__);

	vol_set_flag  = volume & VOLUME_SET_MASK;
	vol_set_value = volume & ~VOLUME_SET_MASK;

	switch(device){
		case AUDIO_IN_DEV_AMIC:
			if(vol_set_flag == VOLUME_SET_LEVEL){
				ac101_vol = ac101_amic_vol_level;
				vol_array_size = HAL_ARRAY_SIZE(ac101_amic_vol_level);
			} else if (vol_set_flag == VOLUME_SET_GAIN) {
				ac101_vol = ac101_amic_vol_gain;
				vol_array_size = HAL_ARRAY_SIZE(ac101_amic_vol_gain);
			}

			for(i=0; i<vol_array_size; i++){
				if(ac101_vol[i].real_val == vol_set_value){
					reg_val = ac101_vol[i].reg_val;
					break;
				}
			}
			if(i == vol_array_size){
				AC101_ERR("Invalid AMIC volume %s: %d!\n", vol_set_flag ? "Gain" : "Level", vol_set_value);
				return HAL_INVALID;
			}

			/* MIC1&2 analog PGA gain Config */
			ac101_priv->amic_vol = volume;
			ac101_update_bits(ADC_PGA_CTRL, 0x7<<ADC_MIC1_PGA_GAIN_BIT 	   | 0x7<<ADC_MIC2_PGA_GAIN_BIT,\
											reg_val<<ADC_MIC1_PGA_GAIN_BIT | reg_val<<ADC_MIC2_PGA_GAIN_BIT);
			AC101_ALWAYS("AMIC set volume %s-[%d]\n", vol_set_flag ? "Gain" : "Level", vol_set_value);
			break;

		case AUDIO_IN_DEV_LINEIN:
			if(vol_set_flag == VOLUME_SET_LEVEL){
				ac101_vol = ac101_linein_vol_level;
				vol_array_size = HAL_ARRAY_SIZE(ac101_linein_vol_level);
			} else if (vol_set_flag == VOLUME_SET_GAIN) {
				ac101_vol = ac101_linein_vol_gain;
				vol_array_size = HAL_ARRAY_SIZE(ac101_linein_vol_gain);
			}

			for(i=0; i<vol_array_size; i++){
				if(ac101_vol[i].real_val == vol_set_value){
					reg_val = ac101_vol[i].reg_val;
					break;
				}
			}
			if(i == vol_array_size){
				AC101_ERR("Invalid LINEIN volume %s: %d!\n", vol_set_flag ? "Gain" : "Level", vol_set_value);
				return HAL_INVALID;
			}

			/* LINEIN analog PGA gain Config */
			ac101_priv->linein_vol = volume;
			ac101_update_bits(ADC_PGA_CTRL, 0x7<<ADC_LINEIN_LR_PGA_GAIN_BIT, reg_val<<ADC_LINEIN_LR_PGA_GAIN_BIT);
			AC101_ALWAYS("LINEIN set volume %s-[%d]\n", vol_set_flag ? "Gain" : "Level", vol_set_value);
			break;

		case AUDIO_OUT_DEV_HP:
			if(vol_set_flag == VOLUME_SET_LEVEL){
				ac101_vol = ac101_hpout_vol_level;
				vol_array_size = HAL_ARRAY_SIZE(ac101_hpout_vol_level);
			} else if (vol_set_flag == VOLUME_SET_GAIN) {
				ac101_vol = ac101_hpout_vol_gain;
				vol_array_size = HAL_ARRAY_SIZE(ac101_hpout_vol_gain);
			}

			for(i=0; i<vol_array_size; i++){
				if(ac101_vol[i].real_val == vol_set_value){
					reg_val = ac101_vol[i].reg_val;
					break;
				}
			}
			if(i == vol_array_size){
				AC101_ERR("Invalid HeadPhone volume %s: %d!\n", vol_set_flag ? "Gain" : "Level", vol_set_value);
				return HAL_INVALID;
			}

			/* HeadPhone analog PGA gain Config */
			ac101_priv->hpout_vol = volume;
			ac101_update_bits(HPOUT_CTRL, 0x3f<<HPOUT_VOL_BIT, reg_val<<HPOUT_VOL_BIT);
			AC101_ALWAYS("HeadPhone set volume %s-[%d]\n", vol_set_flag ? "Gain" : "Level", vol_set_value);
			break;

		case AUDIO_OUT_DEV_SPK:
			if(vol_set_flag == VOLUME_SET_LEVEL){
				ac101_vol = ac101_spkout_vol_level;
				vol_array_size = HAL_ARRAY_SIZE(ac101_spkout_vol_level);
			} else if (vol_set_flag == VOLUME_SET_GAIN) {
				ac101_vol = ac101_spkout_vol_gain;
				vol_array_size = HAL_ARRAY_SIZE(ac101_spkout_vol_gain);
			}

			for(i=0; i<vol_array_size; i++){
				if(ac101_vol[i].real_val == vol_set_value){
					reg_val = ac101_vol[i].reg_val;
					break;
				}
			}
			if(i == vol_array_size){
				AC101_ERR("Invalid Speaker volume %s: %d!\n", vol_set_flag ? "Gain" : "Level", vol_set_value);
				return HAL_INVALID;
			}

			/* Speaker analog PGA gain Config */
			ac101_priv->spkout_vol = volume;
			ac101_update_bits(SPKOUT_CTRL, 0x1f<<SPKOUT_VOL_BIT, reg_val<<SPKOUT_VOL_BIT);
			AC101_ALWAYS("Speaker set volume %s-[%d]\n", vol_set_flag ? "Gain" : "Level", vol_set_value);
			break;

		case AUDIO_IN_DEV_DMIC:
			AC101_ERR("DMIC don't support set volume\n");
			return HAL_INVALID;

		default:
			AC101_ERR("Invalid Audio Device-[0x%08x]!\n",device);
			return HAL_INVALID;
	}

	return HAL_OK;
}

static int ac101_dai_set_route(Audio_Device device, Audio_Dev_State state)
{
	bool enable = (state==AUDIO_DEV_EN) ? true : false;
	AC101_DBG("--->%s\n",__FUNCTION__);

	if(device & AUDIO_IN_DEV_ALL){
		if(ac101_priv->record_route_def){
			ac101_priv->record_route_def = false;
			ac101_set_route(AC101_DEFAULT_RECORD_DEV, false);
		}
	} else {
		if(ac101_priv->play_route_def){
			ac101_priv->play_route_def = false;
			ac101_set_route(AC101_DEFAULT_PLAY_DEV, false);
		}
	}

	return ac101_set_route(device, enable);
}

static int ac101_dai_hw_params(Audio_Stream_Dir dir, struct pcm_config *pcm_cfg)
{
	uint16_t i, sample_resolution, bclk_div, lrck_bclk_ratio;
	AC101_DBG("--->%s\n",__FUNCTION__);

	/* AC101 common init */
	ac101_hw_common_init(dir);

	/* AC101 set sample rate */
	for(i=0; i<HAL_ARRAY_SIZE(ac101_sample_rate); i++){
		if(ac101_sample_rate[i].real_val == pcm_cfg->rate){
			ac101_update_bits(I2S_SR_CTRL, 0xf<<I2S1_ADDA_FS_BIT, ac101_sample_rate[i].reg_val<<I2S1_ADDA_FS_BIT);
			break;
		}
	}
	if(i == HAL_ARRAY_SIZE(ac101_sample_rate)){
		AC101_ERR("AC101 set sample rate-[%d] fail!\n", pcm_cfg->rate);
		return HAL_INVALID;
	}

	/* AC101 set channels */
	if(pcm_cfg->channels<=0 || pcm_cfg->channels>2){
		AC101_ERR("Invalid %s channel nums:%d!\n", dir == PCM_OUT ? "play" : "record", pcm_cfg->channels);
		return HAL_INVALID;
	}
	if(dir == PCM_OUT){
		if(pcm_cfg->channels == 1){
			//I2S1 RX Slot0 L Enable
			ac101_update_bits(I2S1_SDIN_CTRL, 0x1<<I2S1_RX_SLOT0_L_EN_BIT, 0x1<<I2S1_RX_SLOT0_L_EN_BIT);
			//Output Mixer Source Config
			ac101_update_bits(OUT_MXR_SRC, 0x1<<OUT_MXR_R_SRC_DACL_BIT | 0x1<<OUT_MXR_L_SRC_DACL_BIT,\
										   0x1<<OUT_MXR_R_SRC_DACL_BIT | 0x1<<OUT_MXR_L_SRC_DACL_BIT);
		} else {
			//I2S1 RX Slot0 L&R Enable
			ac101_update_bits(I2S1_SDIN_CTRL, 0x1<<I2S1_RX_SLOT0_L_EN_BIT | 0x1<<I2S1_RX_SLOT0_R_EN_BIT,\
											  0x1<<I2S1_RX_SLOT0_L_EN_BIT | 0x1<<I2S1_RX_SLOT0_R_EN_BIT);
			//Output Mixer Source Config
			ac101_update_bits(OUT_MXR_SRC, 0x1<<OUT_MXR_R_SRC_DACR_BIT | 0x1<<OUT_MXR_L_SRC_DACL_BIT,\
										   0x1<<OUT_MXR_R_SRC_DACR_BIT | 0x1<<OUT_MXR_L_SRC_DACL_BIT);
		}
	} else {
		if(pcm_cfg->channels == 1){
			//I2S1 TX Slot0 L Enable
			ac101_update_bits(I2S1_SDOUT_CTRL, 0x1<<I2S1_TX_SLOT0_L_EN_BIT, 0x1<<I2S1_TX_SLOT0_L_EN_BIT);
		} else {
			//I2S1 TX Slot0 L&R Enable
			ac101_update_bits(I2S1_SDOUT_CTRL, 0x1<<I2S1_TX_SLOT0_L_EN_BIT | 0x1<<I2S1_TX_SLOT0_R_EN_BIT,\
											   0x1<<I2S1_TX_SLOT0_L_EN_BIT | 0x1<<I2S1_TX_SLOT0_R_EN_BIT);
		}
	}

	/* AC101 set sample resorution */
	sample_resolution = pcm_format_to_sampleresolution(pcm_cfg->format);
	for(i=0; i<HAL_ARRAY_SIZE(ac101_sample_res); i++){
		if(ac101_sample_res[i].real_val == sample_resolution){
			ac101_update_bits(I2S1_CLK_CTRL, 0x3<<I2S1_SAMPLE_RES_BIT, ac101_sample_res[i].reg_val<<I2S1_SAMPLE_RES_BIT);
			break;
		}
	}
	if(i == HAL_ARRAY_SIZE(ac101_sample_res)){
		AC101_ERR("AC101 set sample resolution-[%d] fail!\n", sample_resolution);
		return HAL_INVALID;
	}

	/* AC101 set BCLK/LRCK ratio */
	lrck_bclk_ratio = ac101_priv->slot_width * ((pcm_cfg->channels+1)/2*2);
	for(i=0; i<HAL_ARRAY_SIZE(ac101_lrck_bclk_ratio); i++){
		if(ac101_lrck_bclk_ratio[i].real_val == lrck_bclk_ratio){
			ac101_update_bits(I2S1_CLK_CTRL, 0x7<<I2S1_LRCKBCLK_RATIO_BIT, ac101_lrck_bclk_ratio[i].reg_val<<I2S1_LRCKBCLK_RATIO_BIT);
			break;
		}
	}
	if(i == HAL_ARRAY_SIZE(ac101_lrck_bclk_ratio)){
		AC101_ERR("AC101 set BCLK/LRCK ratio-[%d] fail!\n", lrck_bclk_ratio);
	}

	/* AC101 set BCLK div */
	bclk_div = (pcm_cfg->rate%1000 ? AUDIO_CLK_22M : AUDIO_CLK_24M)/pcm_cfg->rate/lrck_bclk_ratio;
	for(i=0; i<HAL_ARRAY_SIZE(ac101_bclk_div); i++){
		if(ac101_bclk_div[i].real_val == bclk_div){
			ac101_update_bits(I2S1_CLK_CTRL, 0xf<<I2S1_BCLK_DIV_BIT, ac101_bclk_div[i].reg_val<<I2S1_BCLK_DIV_BIT);
			break;
		}
	}
	if(i == HAL_ARRAY_SIZE(ac101_bclk_div)){
		AC101_ERR("AC101 set BCLK DIV-[%d] fail!\n", bclk_div);
	}

	AC101_DBG("Sample rate-[%d], channel numbers-[%d], sample resolution-[%d]\n",pcm_cfg->rate,pcm_cfg->channels,sample_resolution);
	return HAL_OK;
}

static int ac101_dai_hw_free(Audio_Stream_Dir dir)
{
	AC101_DBG("--->%s\n",__FUNCTION__);

	ac101_hw_common_deinit(dir);

	if(!ac101_priv->playing && !ac101_priv->recording){
		ac101_codec_reset();
	}

	return HAL_OK;
}


/*** codec_ops interface ***/
static int ac101_codec_ioctl(uint32_t cmd, uint32_t cmd_param[], uint32_t cmd_param_len)
{
	int ret = HAL_ERROR;
	AC101_DBG("--->%s\n",__FUNCTION__);

	switch(cmd){
		case CODEC_IOCTL_HW_CONFIG:
			if(cmd_param_len != 1)	return HAL_INVALID;
			ac101_priv->spkout_ch_sel = cmd_param[0] & 0xff;
			ac101_priv->amic_ch_sel   = cmd_param[0]>>8 & 0xff;
			ac101_priv->slot_width	  = cmd_param[0]>>16 & 0xff;
			ac101_priv->drc_en		  = cmd_param[0]>>24 & 0xff;
			ret = HAL_OK;
			break;
		default:
			AC101_ERR("Invalid ioctl command!\n");
			return HAL_INVALID;
	}

	return ret;
}

static int ac101_codec_reg_read(uint32_t reg)
{
	int ret;
	uint16_t reg_val;
	AC101_DBG("--->%s\n",__FUNCTION__);

	if(!ac101_priv || ac101_priv->i2c_id >= I2C_NUM){
		AC101_ERR("AC101 hasn't registered or I2C BUS Number error!\n");
		return HAL_ERROR;
	}

	ret = ac101_read(reg, &reg_val, ac101_priv->i2c_id);
	if(ret != 2){
		return HAL_ERROR;
	}

	return reg_val;
}

static int ac101_codec_reg_write(uint32_t reg, uint32_t val)
{
	int ret;
	AC101_DBG("--->%s\n",__FUNCTION__);

	if(!ac101_priv || ac101_priv->i2c_id >= I2C_NUM){
		AC101_ERR("AC101 hasn't registered or I2C BUS Number error!\n");
		return HAL_ERROR;
	}

	ret = ac101_write(reg, val, ac101_priv->i2c_id);
	if(ret != 2){
		return HAL_ERROR;
	}

	return HAL_OK;
}

static int ac101_codec_open(Audio_Stream_Dir dir)
{
	AC101_DBG("--->%s\n",__FUNCTION__);

	/*int volume and route*/
	if(dir == PCM_OUT){
		if(ac101_priv->playing == true){
			AC101_DBG("Codec play device has opened already\n");
			return HAL_OK;
		}

		ac101_dai_set_volume(AUDIO_OUT_DEV_HP,  ac101_priv->hpout_vol);
		ac101_dai_set_volume(AUDIO_OUT_DEV_SPK, ac101_priv->spkout_vol);
		if(ac101_priv->play_route_def){
			ac101_set_route(AC101_DEFAULT_PLAY_DEV, true);
		}

		ac101_priv->playing = true;
	} else {
		if(ac101_priv->recording == true){
			AC101_DBG("Codec record device has opened already\n");
			return HAL_OK;
		}

		ac101_dai_set_volume(AUDIO_IN_DEV_AMIC,   ac101_priv->amic_vol);
		ac101_dai_set_volume(AUDIO_IN_DEV_LINEIN, ac101_priv->linein_vol);
		if(ac101_priv->record_route_def){
			ac101_set_route(AC101_DEFAULT_RECORD_DEV, true);
		}

		ac101_priv->recording = true;
	}

#if AC101_DBG_EN
	uint16_t i, rt_value;
	printf("\nac101 reg:");
	for(i=0; i<0x59; i++){
		rt_value = 0;
		if(!(i%4))	printf("\n");
		if(i == 0x7)	i=0x10;
		else if(i == 0x19)	i=0x40;
		ac101_read(i, &rt_value, ac101_priv->i2c_id);
		printf("Reg[0x%02x]: 0x%04x;  ",i,rt_value);
	}
	printf("\n\n");
#endif

	return HAL_OK;
}

static int ac101_codec_close(Audio_Stream_Dir dir)
{
	AC101_DBG("--->%s\n",__FUNCTION__);

	if(dir == PCM_OUT){
		if(ac101_priv->playing == false){
			AC101_DBG("Codec play device has closed already\n");
			return HAL_OK;
		}

		/* close all play route */
		ac101_set_route(AUDIO_OUT_DEV_HP, false);
		ac101_set_route(AUDIO_OUT_DEV_SPK, false);

		ac101_priv->play_route_def = true;
		ac101_priv->playing = false;
	} else {
		if(ac101_priv->recording == false){
			AC101_DBG("Codec record device has closed already\n");
			return HAL_OK;
		}

		/* close all record route */
		ac101_set_route(AUDIO_IN_DEV_AMIC, false);
		ac101_set_route(AUDIO_IN_DEV_LINEIN, false);
		ac101_set_route(AUDIO_IN_DEV_DMIC, false);

		ac101_priv->record_route_def = true;
		ac101_priv->recording = false;
	}

	return HAL_OK;
}


/*** codec_driver interface ***/
static int ac101_codec_init(void)
{
	AC101_DBG("--->%s\n",__FUNCTION__);

	return HAL_OK;
}

static void ac101_codec_deinit(void)
{
	AC101_DBG("--->%s\n",__FUNCTION__);
}


/*** codec dai ops ****/
static const struct codec_dai_ops ac101_codec_dai_ops = {
	.set_sysclk = ac101_dai_set_sysclk,
	.set_fmt    = ac101_dai_set_fmt,
	.set_volume = ac101_dai_set_volume,
	.set_route  = ac101_dai_set_route,
	.hw_params  = ac101_dai_hw_params,
	.hw_free    = ac101_dai_hw_free,
};

/*** codec ops ****/
static const struct codec_ops ac101_codec_ops = {
	.open  = ac101_codec_open,
	.close = ac101_codec_close,

	.reg_read  = ac101_codec_reg_read,
	.reg_write = ac101_codec_reg_write,

	.ioctl = ac101_codec_ioctl,
};

/*** codec driver ****/
static struct codec_driver ac101_codec_drv = {
	.name = AC101_CODEC_NAME,
	.codec_attr = XRADIO_CODEC_AC101,

	.init   = ac101_codec_init,
	.deinit = ac101_codec_deinit,

	.dai_ops   = &ac101_codec_dai_ops,
	.codec_ops = &ac101_codec_ops,
};


/*** Register interface ***/
HAL_Status ac101_codec_register(void)
{
	uint8_t i, i2c_id;
	uint16_t chip_id;
	AC101_DBG("--->%s\n",__FUNCTION__);

	/* Init I2C and wait to be stable */
	I2C_InitParam i2c_param;
	i2c_param.addrMode = I2C_ADDR_MODE_7BIT;
	i2c_param.clockFreq = 400000;
	for(i=0; i<I2C_NUM; i++){
		if(HAL_I2C_Init(i, &i2c_param) != HAL_OK){
			AC101_ERR("I2C-[%d] init Fail...\n",i);
		}
	}
	HAL_MSleep(20);

	/* Auto detect AC101 */
	for(i2c_id=0,chip_id=0; i2c_id<I2C_NUM; i2c_id++,chip_id=0){
		ac101_read(CHIP_AUDIO_RST, &chip_id, i2c_id);
		if(chip_id == 0x0101){
			AC101_DBG("AC101 on I2C-[%d] auto detect success\n",i2c_id);
			break;
		}
	}
	if(i2c_id == I2C_NUM){
		AC101_ERR("NO AC101 chip been detect, register Fail!\n");
		return HAL_ERROR;
	}

	/* Malloc ac101_priv buffer */
	ac101_priv = (struct ac101_codec_priv *)AC101_MALLOC(sizeof(struct ac101_codec_priv));
	if(ac101_priv == NULL){
		AC101_ERR("Malloc ac101 codec priv buffer Fail!\n");
		return HAL_ERROR;
	}
	AC101_MEMSET(ac101_priv, 0, sizeof(struct ac101_codec_priv));

	/* Init  ac101_priv */
	ac101_priv->i2c_id = i2c_id;
	ac101_priv->slot_width = AC101_DEFAULT_SLOT_WIDTH;
	ac101_priv->amic_vol = AC101_DEFAULT_AMIC_VOL;
	ac101_priv->linein_vol = AC101_DEFAULT_LINEIN_VOL;
	ac101_priv->hpout_vol = AC101_DEFAULT_HP_VOL;
	ac101_priv->spkout_vol = AC101_DEFAULT_SPK_VOL;
	ac101_priv->play_route_def = ac101_priv->record_route_def = true;
	ac101_priv->amic_ch_sel = 1;
	ac101_priv->drc_en = 0;

	/* Codec list add */
	list_add(&ac101_codec_drv.node, &hal_snd_codec_list);

	return HAL_OK;
}

HAL_Status ac101_codec_unregister(void)
{
	struct codec_driver *codec_drv_ptr;
	AC101_DBG("--->%s\n",__FUNCTION__);

	/* Check snd codec list empty or not */
	if(list_empty(&hal_snd_codec_list)){
		AC101_DBG("Hal snd codec list is empty, don't need to unregister\n");
		return HAL_OK;
	}

	/* Get codec to unregister */
	list_for_each_entry(codec_drv_ptr, &hal_snd_codec_list, node){
		if(codec_drv_ptr == &ac101_codec_drv){
			list_del(&ac101_codec_drv.node);
			break;
		}
	}

	/* Free ac101 priv buffer */
	if(ac101_priv){
		AC101_FREE(ac101_priv);
		ac101_priv = NULL;
	}

	return HAL_OK;
}


