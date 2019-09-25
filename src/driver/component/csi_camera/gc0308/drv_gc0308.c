/**
  * @file  drv_gc0308.c
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "kernel/os/os.h"
#include "driver/chip/hal_i2c.h"
#include "driver/chip/hal_csi.h"

#include "driver/component/csi_camera/gc0308/drv_gc0308.h"
#include "gc0308_cfg.h"

#define GC0308_SCCB_ID 0x21  			//GC0308 ID
#define GC0308_CHIP_ID 0x9b
#define GC0308_IIC_CLK_FREQ	100000

#define SENSOR_DBG_LEVEL INFO
enum LOG_LEVEL{
	OFF = 0,
	ERROR,
	WARN,
	DEBUG,
	INFO,
};
#define SENSOR_DBG(level, fmt, args...) \
	do {		\
		if (level <= SENSOR_DBG_LEVEL)	\
			printf(fmt,##args);	\
	} while (0)

static uint8_t i2c_id;

static void GC0308_InitSccb(void)
{
    I2C_InitParam initParam;

    initParam.addrMode = I2C_ADDR_MODE_7BIT;
    initParam.clockFreq = GC0308_IIC_CLK_FREQ;
    HAL_I2C_Init(i2c_id, &initParam);
}

static void GC0308_DeInitSccb(void)
{
    HAL_I2C_DeInit(i2c_id);
}

int GC0308_WriteSccb(uint8_t sub_addr, uint8_t data)
{
    return HAL_I2C_SCCB_Master_Transmit_IT(i2c_id, GC0308_SCCB_ID, sub_addr, &data);
}

int GC0308_ReadSccb(uint8_t sub_addr, uint8_t *data)
{
    return HAL_I2C_SCCB_Master_Receive_IT(i2c_id, GC0308_SCCB_ID, sub_addr, data);
}

/**
  * @brief Seclet the light mode.
  * @note This function is used to set the light mode for camera.
  *           The appropriate mode helps to improve the shooting effect.
  * @param light_mode: light mode.
  * @retval None
  */
void GC0308_SetLightMode(SENSOR_LightMode light_mode)
{
    uint8_t reg13val = 0XE7, reg01val = 0, reg02val = 0;
    switch(light_mode) {
    case LIGHT_AUTO:
        reg13val = 0XE7;
        reg01val = 0;
        reg02val = 0;
        break;
    case LIGHT_SUNNY:
        reg13val = 0XE5;
        reg01val = 0X5A;
        reg02val = 0X5C;
        break;
    case LIGHT_COLUDY:
        reg13val = 0XE5;
        reg01val = 0X58;
        reg02val = 0X60;
        break;
    case LIGHT_OFFICE:
        reg13val = 0XE5;
        reg01val = 0X84;
        reg02val = 0X4c;
        break;
    case LIGHT_HOME:
        reg13val = 0XE5;
        reg01val = 0X96;
        reg02val = 0X40;
        break;
    }
    GC0308_WriteSccb(0X13, reg13val);
    GC0308_WriteSccb(0X01, reg01val);
    GC0308_WriteSccb(0X02, reg02val);
}

/**
  * @brief Set the color saturation for camera.
  * @param sat: The color saturation.
  * @retval None
  */
void GC0308_SetColorSaturation(SENSOR_ColorSaturation sat)
{
    uint8_t reg4f5054val = 0X80, reg52val = 0X22, reg53val = 0X5E;
    switch(sat) {
    case COLOR_SATURATION_0://-2
        reg4f5054val = 0X40;
        reg52val = 0X11;
        reg53val = 0X2F;
        break;
    case COLOR_SATURATION_1://-1
        reg4f5054val = 0X66;
        reg52val = 0X1B;
        reg53val = 0X4B;
        break;
    case COLOR_SATURATION_2:
        reg4f5054val = 0X80;
        reg52val = 0X22;
        reg53val = 0X5E;
        break;
    case COLOR_SATURATION_3:
        reg4f5054val = 0X99;
        reg52val = 0X28;
        reg53val = 0X71;
        break;
    case COLOR_SATURATION_4:
        reg4f5054val = 0XC0;
        reg52val = 0X33;
        reg53val = 0X8D;
        break;
    }

    GC0308_WriteSccb(0X4F, reg4f5054val);
    GC0308_WriteSccb(0X50, reg4f5054val);
    GC0308_WriteSccb(0X51, 0X00);
    GC0308_WriteSccb(0X52, reg52val);
    GC0308_WriteSccb(0X53, reg53val);
    GC0308_WriteSccb(0X54, reg4f5054val);

}

/**
  * @brief Set the sensitivity for camera.
  * @param brihgt: The brightness value.
  * @retval None
  */
void GC0308_SetBrightness(SENSOR_Brightness bright)
{
    uint8_t reg55val = 0X00;
    switch(bright) {
    case BRIGHT_0:		//-2
        reg55val = 0XB0;
        break;
    case BRIGHT_1:
        reg55val = 0X98;
        break;
    case BRIGHT_2:
        reg55val = 0X00;
        break;
    case BRIGHT_3:
        reg55val = 0X18;
        break;
    case BRIGHT_4:
        reg55val = 0X30;
        break;
    }

    GC0308_WriteSccb(0X55,reg55val);
}

/**
  * @brief Set the contarst for camera.
  * @param contrast: The contrast value.
  * @retval None
  */
void GC0308_SetContrast(SENSOR_Contarst contrast)
{
    uint8_t reg56val = 0X40;
    switch(contrast) {
    case CONTARST_0:	//-2
        reg56val = 0X30;
        break;
    case CONTARST_1:
        reg56val = 0X38;
        break;
    case CONTARST_2:
        reg56val = 0X40;
        break;
    case CONTARST_3:
        reg56val = 0X50;
        break;
    case CONTARST_4:
        reg56val = 0X60;
        break;
    }
    GC0308_WriteSccb(0X56,reg56val);
}

/**
  * @brief Set the effects for camera.
  * @param eft: effects.
  * @retval None
  */
void GC0308_SetSpecialEffects(SENSOR_SpecailEffects eft)
{
    uint8_t reg3aval = 0X04;
    uint8_t reg67val = 0XC0;
    uint8_t reg68val = 0X80;
    switch(eft) {
    case IMAGE_NOMAL:	//nomal
        reg3aval = 0X04;
        reg67val = 0XC0;
        reg68val = 0X80;
        break;
    case IMAGE_NEGATIVE:
        reg3aval = 0X24;
        reg67val = 0X80;
        reg68val = 0X80;
        break;
    case IMAGE_BLACK_WHITE:
        reg3aval = 0X14;
        reg67val = 0X80;
        reg68val = 0X80;
        break;
    case IMAGE_SLANT_RED:
        reg3aval = 0X14;
        reg67val = 0Xc0;
        reg68val = 0X80;
        break;
    case IMAGE_SLANT_GREEN:
        reg3aval = 0X14;
        reg67val = 0X40;
        reg68val = 0X40;
        break;
    case IMAGE_SLANT_BLUE:
        reg3aval = 0X14;
        reg67val = 0X80;
        reg68val = 0XC0;
        break;
    case IMAGE_VINTAGE:
        reg3aval = 0X14;
        reg67val = 0XA0;
        reg68val = 0X40;
        break;
    }

    GC0308_WriteSccb(0X3A, reg3aval);
    GC0308_WriteSccb(0X68, reg67val);
    GC0308_WriteSccb(0X67, reg68val);
}

/**
  * @brief Set the window for camera.
  * @param sx: Starting coordinates.
  * @param sy: Starting coordinates.
  * @param width: Window width.
  * @param height: Window height.
  * @retval None
  */
void GC0308_SetWindow(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
    uint16_t endx;
    uint16_t endy;
    uint8_t temp;

    endx = sx + width * 2;		//V*2
    endy = sy + height * 2;
    if(endy > 784)
        endy -= 784;

    GC0308_ReadSccb(0X03, &temp);
    temp &= 0XF0;
    temp |= ((endx & 0X03) << 2) | (sx & 0X03);
    GC0308_WriteSccb(0X03, temp);
    GC0308_WriteSccb(0X19, sx>>2);
    GC0308_WriteSccb(0X1A, endx>>2);
    GC0308_ReadSccb(0X32, &temp);
    temp &= 0XC0;
    temp |= ((endy & 0X07) << 3) | (sy&0X07);
    GC0308_WriteSccb(0X32, temp);
    GC0308_WriteSccb(0X17, sy >> 3);
    GC0308_WriteSccb(0X18, endy >> 3);
}

//(140,16,640,480) is good for VGA
//(272,16,320,240) is good for QVGA
/* config_GC0308_window */
void GC0308_ConfigWindow(unsigned int startx,unsigned int starty,unsigned int width, unsigned int height)
{
    unsigned int endx;
    unsigned int endy;// "v*2"±ØÐë
    unsigned char temp_reg1, temp_reg2;
    unsigned char temp=0;

    endx=(startx+width*2)%784;
    endy=(starty+height*2);// "v*2"±ØÐë

    GC0308_ReadSccb(0x32, &temp_reg2 );
    temp_reg2 &= 0xc0;

    GC0308_ReadSccb(0x03, &temp_reg1 );
    temp_reg1 &= 0xf0;

    // Horizontal
    temp = temp_reg2|((endx&0x7)<<3)|(startx&0x7);
    GC0308_WriteSccb(0x32, temp );
    temp = (startx&0x7F8)>>3;
    GC0308_WriteSccb(0x17, temp );
    temp = (endx&0x7F8)>>3;
    GC0308_WriteSccb(0x18, temp );

    // Vertical
    temp =temp_reg1|((endy&0x3)<<2)|(starty&0x3);
    GC0308_WriteSccb(0x03, temp );
    temp = starty>>2;
    GC0308_WriteSccb(0x19, temp );
    temp = endy>>2;
    GC0308_WriteSccb(0x1A, temp );
}

void GC0308_SetPixelOutFmt(SENSOR_PixelOutFmt pixel_out_fmt)
{
    switch (pixel_out_fmt) {
    case YUV422_UYVY:
        GC0308_WriteSccb(0x24, 0xa0);
        break;
    case YUV422_VYUY:
        GC0308_WriteSccb(0x24, 0xa1);
        break;
    case YUV422_YUYV:
        GC0308_WriteSccb(0x24, 0xa2);
        break;
    case YUV422_YVYU:
        GC0308_WriteSccb(0x24, 0xa3);
        break;
    case RGB565:
        GC0308_WriteSccb(0x24, 0xa6);
        break;
    default:
        SENSOR_DBG(ERROR, "GC0308:untest pixel out fmt %d\n", pixel_out_fmt);
        break;
	}
}

/**
  * @brief Init the io for ctrl the camera power.
  * @param cfg: The io info.
  * @retval None
  */
static void GC0308_InitPower(SENSOR_PowerCtrlCfg *cfg)
{
    GPIO_InitParam param;
    param.driving = GPIO_DRIVING_LEVEL_1;
    param.mode = GPIOx_Pn_F1_OUTPUT;
    param.pull = GPIO_PULL_NONE;

    HAL_GPIO_Init(cfg->Pwdn_Port, cfg->Pwdn_Pin, &param);
    HAL_GPIO_Init(cfg->Reset_Port, cfg->Reset_Pin, &param);

    HAL_GPIO_WritePin(cfg->Pwdn_Port, cfg->Pwdn_Pin, GPIO_PIN_LOW);
    HAL_GPIO_WritePin(cfg->Reset_Port, cfg->Reset_Pin, GPIO_PIN_LOW);
    OS_MSleep(3);
    HAL_GPIO_WritePin(cfg->Reset_Port, cfg->Reset_Pin, GPIO_PIN_HIGH);
    OS_MSleep(100);
}

static void GC0308_DeInitPower(SENSOR_PowerCtrlCfg *cfg)
{
    HAL_GPIO_WritePin(cfg->Pwdn_Port, cfg->Pwdn_Pin, GPIO_PIN_HIGH);
    HAL_GPIO_WritePin(cfg->Reset_Port, cfg->Reset_Pin, GPIO_PIN_LOW);
    OS_MSleep(3);
    HAL_GPIO_DeInit(cfg->Pwdn_Port, cfg->Pwdn_Pin);
    HAL_GPIO_DeInit(cfg->Reset_Port, cfg->Reset_Pin);
}

static HAL_Status GC0308_Init(void)
{
    uint8_t chip_id;
    uint16_t i = 0;

    GC0308_WriteSccb(0XFE, 0x80);
    OS_MSleep(100);

    GC0308_WriteSccb(0XFE, 0x00);
    OS_MSleep(100);

    if (GC0308_ReadSccb(0x00, &chip_id) != 1) {
        SENSOR_DBG(ERROR, "GC0308 sccb read error\n");
        return HAL_ERROR;
    } else {
	    if(chip_id!= GC0308_CHIP_ID) {
		    SENSOR_DBG(ERROR, "GC0308 get chip id wrong 0x%02x\n", chip_id);
		    return HAL_ERROR;
	    } else {
		    SENSOR_DBG(INFO, "GC0308 chip id read success 0x%02x\n", chip_id);
	    }
    }

    OS_MSleep(1000);

    for (i = 0; i < sizeof(gc0308_init_reg_tbl) / sizeof(gc0308_init_reg_tbl[0]); i++) {
        if (!GC0308_WriteSccb(gc0308_init_reg_tbl[i][0], gc0308_init_reg_tbl[i][1])) {
            SENSOR_DBG(ERROR, "GC0308 sccb read error\n");
            return HAL_ERROR;
        }
    }

   SENSOR_DBG(DEBUG, "GC0308 Init Done \r\n");

    return HAL_OK;
}

HAL_Status HAL_GC0308_IoCtl(SENSOR_IoctrlCmd attr, uint32_t arg)
{
    switch (attr) {
        case SENSOR_SET_OUTPUT_FMT:
        {
            SENSOR_PixelOutFmt output_fmt;
            output_fmt = *((SENSOR_PixelOutFmt *)arg);
            GC0308_SetPixelOutFmt(output_fmt);
            break;
        }
        default:
            SENSOR_DBG(ERROR, "un support camsensor cmd %d\n", attr);
            return HAL_ERROR;
            break;
    }

    return HAL_OK;
}

/**
  * @brief Init the GC0308.
  * @retval HAL_Status : The driver status.
  */
HAL_Status HAL_GC0308_Init(SENSOR_ConfigParam *cfg)
{
    i2c_id = cfg->i2c_id;

	GC0308_InitSccb();

    GC0308_InitPower(&cfg->pwcfg);

    if (GC0308_Init() != HAL_OK) {
	    SENSOR_DBG(ERROR, "GC0308  Init error!!\n");
	    return HAL_ERROR;
    }

	/* sensor set, windows cfg_set, pixelformat cfg_set and so on */
    GC0308_SetPixelOutFmt(cfg->pixel_outfmt);
    OS_MSleep(500);
    return HAL_OK;
}

/**
  * @brief Deinit the GC0308.
  * @retval HAL_Status : The driver status.
  */
void HAL_GC0308_DeInit(SENSOR_ConfigParam *cfg)
{
    i2c_id = 0;
    GC0308_DeInitPower(&cfg->pwcfg);
	GC0308_DeInitSccb();
}

/**
  * @brief Suspend the GC0308.
  * @retval Void.
  */
void HAL_GC0308_Suspend(void)
{
	uint8_t analog_mode1 = 0;
	uint8_t output_en = 0;

	GC0308_ReadSccb(0x1a, &analog_mode1);
	GC0308_ReadSccb(0x25, &output_en);

	analog_mode1 |= 0x1;
	output_en = 0;

	GC0308_WriteSccb(0x1a, analog_mode1);
	GC0308_WriteSccb(0x25, output_en);
}

/**
  * @brief Resume the GC0308.
  * @retval Void.
  */
void HAL_GC0308_Resume(void)
{
	uint8_t analog_mode1 = 0;
	uint8_t output_en = 0;

	GC0308_ReadSccb(0x1a, &analog_mode1);
	GC0308_ReadSccb(0x25, &output_en);

	analog_mode1 &= (~(0x1));
	output_en = 0x7;

	GC0308_WriteSccb(0x1a, analog_mode1);
	GC0308_WriteSccb(0x25, output_en);
}
