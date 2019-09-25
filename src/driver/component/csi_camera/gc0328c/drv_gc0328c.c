/**
  * @file  drv_gc0328c.c
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
#include "driver/component/csi_camera/gc0328c/drv_gc0328c.h"
#include "gc0328c_cfg.h"

#define GC0328C_SCCB_ID 0x21  			//GC0328C ID
#define GC0328C_CHIP_ID 0x9d
#define GC0328C_IIC_CLK_FREQ	100000

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

static void GC0328C_InitSccb(void)
{
    I2C_InitParam initParam;

    initParam.addrMode = I2C_ADDR_MODE_7BIT;
    initParam.clockFreq = GC0328C_IIC_CLK_FREQ;
    HAL_I2C_Init(i2c_id, &initParam);
}

static void GC0328C_DeInitSccb(void)
{
    HAL_I2C_DeInit(i2c_id);
}

int GC0328C_WriteSccb(uint8_t sub_addr, uint8_t data)
{
    return HAL_I2C_SCCB_Master_Transmit_IT(i2c_id, GC0328C_SCCB_ID, sub_addr, &data);
}

int GC0328C_ReadSccb(uint8_t sub_addr, uint8_t *data)
{
    return HAL_I2C_SCCB_Master_Receive_IT(i2c_id, GC0328C_SCCB_ID, sub_addr, data);
}

/**
  * @brief Set the effects for camera.
  * @param eft: effects.
  * @retval None
  */
void GC0328C_SetSpecialEffects(SENSOR_SpecailEffects eft)
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

    GC0328C_WriteSccb(0X3A, reg3aval);
    GC0328C_WriteSccb(0X68, reg67val);
    GC0328C_WriteSccb(0X67, reg68val);
}

/**
  * @brief Set the window for camera.
  * @param x: Starting coordinates.
  * @param y: Starting coordinates.
  * @param width: Window width.
  * @param height: Window height.
  * @retval None
  */
void GC0328C_SetWindow(uint16_t x,uint16_t y,uint16_t w,uint16_t h)
{
    if (x<0 || x>648 || y<0 || y>488 || w<=0 || w>648 || h<=0 || h>488)
    {
        return ;
    }

    GC0328C_WriteSccb(0xfe, 0x00);            // page0
    GC0328C_WriteSccb(0x09, (y>>8)&0x0001);   // Row_start[8]
    GC0328C_WriteSccb(0x0a, y&0x00FF);        // Row_start[7:0]
    GC0328C_WriteSccb(0x0b, (x>>8)&0x0003);   // Column_start[9:8]
    GC0328C_WriteSccb(0x0c, x&0x00FF);        // Column_start[7:0]
    GC0328C_WriteSccb(0x0d, (h>>8)&0x0001);   // Window_height[8]
    GC0328C_WriteSccb(0x0e, h&0x00FF);        // Window_height[7:0]
    GC0328C_WriteSccb(0x0f, (w>>8)&0x0003);   // Window_width[9:8]
    GC0328C_WriteSccb(0x10, w&0x00FF);        // Window_width[7:0]
}

void GC0328C_SetCropWindow(uint16_t x,uint16_t y,uint16_t w,uint16_t h)
{
    if ((x==0) && (y==0) && (w==0) && (h==0))
    {
        // disable crop window
        GC0328C_WriteSccb(0x50, 0x00);
        return ;
    }
    if (x<0 || x>648 || y<0 || y>488 || w<=0 || w>648 || h<=0 || h>488)
    {
        return ;
    }
    GC0328C_WriteSccb(0xfe, 0x00);            // page0
    GC0328C_WriteSccb(0x51, (y>>8)&0x0003);   // out window y1[8,9]
    GC0328C_WriteSccb(0x52, y&0x00FF);        // out window y1[7:0]
    GC0328C_WriteSccb(0x53, (x>>8)&0x0007);   // out window x1[10:8]
    GC0328C_WriteSccb(0x54, x&0x00FF);        // out window x1[7:0]
    GC0328C_WriteSccb(0x55, (h>>8)&0x0001);   // out Window_height[8]
    GC0328C_WriteSccb(0x56, h&0x00FF);        // out Window_height[7:0]
    GC0328C_WriteSccb(0x57, (w>>8)&0x0003);   // out Window_width[9:8]
    GC0328C_WriteSccb(0x58, w&0x00FF);        // out Window_width[7:0]
    GC0328C_WriteSccb(0x50, 0x01);            // crop out window mode
}

void GC0328C_SetSubsample(uint8_t ratio)
{
    if (ratio > 6)
    {
        return ;
    }
    uint8_t row_col;
    uint8_t row_n1, row_n2, row_n3, row_n4;
    uint8_t col_n1, col_n2, col_n3, col_n4;

    uint8_t tmp;
    GC0328C_WriteSccb(0xfe,0x00); // page0
    GC0328C_ReadSccb(0x5a, &tmp);
    tmp &= ~(3<<4);
    GC0328C_WriteSccb(0x5a, tmp); // sub_mode[5:4]

    if (ratio == 0)
    {
        return ;
    }
    else if (ratio == 1)
    {
        row_col = 0x22;
        row_n1 = row_n2 = row_n3 = row_n4 = 0;
        col_n1 = col_n2 = col_n3 = col_n4 = 0;
    }
    else if (ratio == 2)
    {
        row_col = 0x33;
        row_n1 = row_n2 = row_n3 = row_n4 = 0;
        col_n1 = col_n2 = col_n3 = col_n4 = 0;
    }
    else if (ratio == 3)
    {
        row_col = 0x44;
        row_n1 = row_n2 = row_n3 = row_n4 = 0;
        col_n1 = col_n2 = col_n3 = col_n4 = 0;
    }
    else if (ratio == 4)
    {
        row_col = 0x33;
        row_n2 = row_n3 = row_n4 = 0;
        col_n2 = col_n3 = col_n4 = 0;
        row_n1 = 0x02;
        col_n1 = 0x02;
    }
    else if (ratio == 5)
    {
        row_col = 0x55;
        row_n3 = row_n4 = 0;
        col_n3 = col_n4 = 0;
        row_n1 = 0x02;
        col_n1 = 0x02;
        row_n2 = 0x04;
        col_n2 = 0x04;
    }
    else if (ratio == 6)
    {
        row_col = 0x77;
        row_n1 = 0x02;
        row_n2 = 0x46;
        row_n3 = 0x00;
        row_n4 = 0x40;
        col_n1 = 0x02;
        col_n2 = 0x46;
        col_n3 = 0x00;
        col_n4 = 0x00;
    }
    else
    {
        return ;
    }

    GC0328C_WriteSccb(0x59, row_col); // subsample

    GC0328C_WriteSccb(0x5b, row_n1);   // Sub_row_N1
    GC0328C_WriteSccb(0x5c, row_n2);   // Sub_row_N2
    GC0328C_WriteSccb(0x5d, row_n3);   // Sub_row_N3
    GC0328C_WriteSccb(0x5e, row_n4);   // Sub_row_N4

    GC0328C_WriteSccb(0x5f, col_n1);   // Sub_col_N1
    GC0328C_WriteSccb(0x60, col_n2);   // Sub_col_N2
    GC0328C_WriteSccb(0x61, col_n3);   // Sub_col_N3
    GC0328C_WriteSccb(0x62, col_n4);   // Sub_col_N4
}

/**
 * Meas_win_x0 = Reg[P1:0x06]*4
 * Meas_win_y0 = Reg[P1:0x07]*4
 * Meas_win_x1 = Reg[P1:0x08]*4
 * Meas_win_y1 = Reg[P1:0x09]*4
 */
void GC0328C_SetMeasureWin(uint16_t x0,uint16_t y0,uint16_t w,uint16_t h)
{
    if (x0<0 || x0>648 || y0<0 || y0>488 || w<=0 || w>648 || h<=0 || h>488)
    {
        return ;
    }
    uint16_t x1,y1;
    x1 = x0 + w;
    y1 = y0 + h;
    if (x1 > 640)
        x1 = 640;
    if (y1 > 480)
        y1 = 480;
    GC0328C_WriteSccb(0xfe, 0x01);       // page1
    GC0328C_WriteSccb(0x06, x0/4);       // big_win_x0
    GC0328C_WriteSccb(0x07, y0/4);       // big_win_y0
    GC0328C_WriteSccb(0x08, x1/4);       // big_win_x1
    GC0328C_WriteSccb(0x09, y1/4);       // big_win_y1
}

void GC0328C_SetPixelOutFmt(SENSOR_PixelOutFmt pixel_out_fmt)
{
    GC0328C_WriteSccb(0xfe,0x00); // page0
    switch (pixel_out_fmt) {
    case YUV422_UYVY:
        GC0328C_WriteSccb(0x44, 0xa0);
        break;
    case YUV422_VYUY:
        GC0328C_WriteSccb(0x44, 0xa1);
        break;
    case YUV422_YUYV:
        GC0328C_WriteSccb(0x44, 0xa2);
        break;
    case YUV422_YVYU:
        GC0328C_WriteSccb(0x44, 0xa3);
        break;
    case RGB565:
        GC0328C_WriteSccb(0x44, 0xa6);
        break;
    default:
        SENSOR_DBG(ERROR, "GC0328C:untest pixel out fmt %d\n", pixel_out_fmt);
        break;
	}
}

/**
  * @brief Init the io for ctrl the camera power.
  * @param cfg: The io info.
  * @retval None
  */
static void GC0328C_InitPower(SENSOR_PowerCtrlCfg *cfg)
{
    GPIO_InitParam param;
    param.driving = GPIO_DRIVING_LEVEL_1;
    param.mode = GPIOx_Pn_F1_OUTPUT;
    param.pull = GPIO_PULL_NONE;

    HAL_GPIO_Init(cfg->Pwdn_Port, cfg->Pwdn_Pin, &param);
    HAL_GPIO_WritePin(cfg->Pwdn_Port, cfg->Pwdn_Pin, GPIO_PIN_LOW);
    OS_MSleep(10);
}

static void GC0328C_DeInitPower(SENSOR_PowerCtrlCfg *cfg)
{
    HAL_GPIO_WritePin(cfg->Pwdn_Port, cfg->Pwdn_Pin, GPIO_PIN_HIGH);
    HAL_GPIO_DeInit(cfg->Pwdn_Port, cfg->Pwdn_Pin);
}

HAL_Status HAL_GC0328C_IoCtl(SENSOR_IoctrlCmd attr, uint32_t arg)
{
    switch (attr) {
        case SENSOR_SET_OUTPUT_FMT:
        {
            SENSOR_PixelOutFmt output_fmt;
            output_fmt = *((SENSOR_PixelOutFmt *)arg);
            GC0328C_SetPixelOutFmt(output_fmt);
            break;
        }
        default:
            SENSOR_DBG(ERROR, "un support camsensor cmd %d\n", attr);
            return HAL_ERROR;
            break;
    }

    return HAL_OK;
}

static HAL_Status GC0328C_Init(void)
{
    uint8_t chip_id;
    uint16_t i = 0;

    if (GC0328C_ReadSccb(0xf0, &chip_id) != 1) {
        SENSOR_DBG(ERROR, "GC0328C sccb read error\n");
        return HAL_ERROR;
    } else {
	    if(chip_id!= GC0328C_CHIP_ID) {
		    SENSOR_DBG(ERROR, "GC0328C get chip id wrong 0x%02x\n", chip_id);
		    return HAL_ERROR;
	    } else {
		    SENSOR_DBG(INFO, "GC0328C chip id read success 0x%02x\n", chip_id);
	    }
    }

    for (i = 0; i < sizeof(gc0328c_init_reg_tbl) / sizeof(gc0328c_init_reg_tbl[0]); i++) {
        if (!GC0328C_WriteSccb(gc0328c_init_reg_tbl[i][0], gc0328c_init_reg_tbl[i][1])) {
            SENSOR_DBG(ERROR, "GC0328C sccb read error\n");
            return HAL_ERROR;
        }
    }

    SENSOR_DBG(DEBUG, "GC0328C Init Done \r\n");

    return HAL_OK;
}

/**
  * @brief Init the GC0328C.
  * @retval Component_Status : The driver status.
  */
HAL_Status HAL_GC0328C_Init(SENSOR_ConfigParam *cfg)
{
    i2c_id = cfg->i2c_id;

	GC0328C_InitSccb();

    GC0328C_InitPower(&cfg->pwcfg);

    if (GC0328C_Init() != HAL_OK) {
	    SENSOR_DBG(ERROR, "GC0328C  Init error!!\n");
	    return HAL_ERROR;
    }

	/* sensor set, windows cfg_set, pixelformat cfg_set and so on */
	GC0328C_SetPixelOutFmt(cfg->pixel_outfmt);
	if ((cfg->pixel_size.height == 240) && (cfg->pixel_size.width == 320)) {  //subsample 1/2
    	GC0328C_SetSubsample(1);//1/2  320*240
    	GC0328C_SetCropWindow(0,0,320,240);
	} else if ((cfg->pixel_size.height == 480) && (cfg->pixel_size.width == 640)) {
		GC0328C_SetCropWindow(0,0,640,480);
	} else {
		SENSOR_DBG(ERROR, "GC0328C  untest pixel_size case\n");
	}

    OS_MSleep(1000);

    return HAL_OK;
}

/**
  * @brief Deinit the GC0328C.
  * @retval Component_Status : The driver status.
  */
void HAL_GC0328C_DeInit(SENSOR_ConfigParam *cfg)
{
    i2c_id = 0;
    GC0328C_DeInitPower(&cfg->pwcfg);
	GC0328C_DeInitSccb();
}

/**
  * @brief Suspend the GC0328C.
  * @retval Void.
  */
void HAL_GC0328C_Suspend(void)
{
	uint8_t clock_mode = 0;
	uint8_t pad_setting1 = 0;

	GC0328C_ReadSccb(0xfc, &clock_mode);
	GC0328C_ReadSccb(0xf1, &pad_setting1);

	clock_mode |= 0x1;
	pad_setting1 = 0;

	GC0328C_WriteSccb(0xfc, clock_mode);
	GC0328C_WriteSccb(0xf1, pad_setting1);
}

/**
  * @brief Resume the GC0328C.
  * @retval Void.
  */
void HAL_GC0328C_Resume(void)
{
	uint8_t clock_mode = 0;
	uint8_t pad_setting1 = 0;

	GC0328C_ReadSccb(0xfc, &clock_mode);
	GC0328C_ReadSccb(0xf1, &pad_setting1);

	clock_mode &= (~(0x1));
	pad_setting1 = 0x7;

	GC0328C_WriteSccb(0xfc, clock_mode);
	GC0328C_WriteSccb(0xf1, pad_setting1);
}
