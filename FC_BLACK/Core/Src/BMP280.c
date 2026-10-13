/*
 * BMP280.c
 *
 *  Created on: 29-Jul-2026
 *      Author: GeetikSharma
 */
#include "FreeRTOS.h"
#include "task.h"
#include "bmp280.h"
#include "bmp280_reg.h"
#include <math.h>
#include "i2c.h"
#include "event_groups.h"

// PRIVATE VARIABLES: The rest of the drone cannot see these

static float ground_pressure_pa=0;
static uint8_t ground_captured=0;
static uint16_t warmup_count=0;
#define BMP280_WARMUP_SAMPLES 25

volatile uint8_t chip_id_result = 0;
volatile uint8_t bmp280_ok = 0;

static float t_fine;

uint8_t values[6];
uint32_t pressure;
uint32_t temperature;
uint8_t data_status=1;
float temp_c;
float pressure_c;

float altitude_m;
// Store this in your main driver file

typedef struct
{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
}BMP280_CalibData;

static BMP280_CalibData bmp280_cal;

//static uint8_t I2C_Read(uint8_t reg)
//{
//	uint8_t data=0;
//	HAL_StatusTypeDef status;
//	status=HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR<<1, reg,I2C_MEMADD_SIZE_8BIT,&data,1,pdMS_TO_TICKS(100));
//	if(status == HAL_OK)
//	{
//		return data;
//	}
//	return 0x00;
//}

static void I2C_Write(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(&hi2c1,BMP280_I2C_ADDR<<1, reg, 1, &value, 1, 100);
}

static void BMP280_Read_Calibration(void)
{
	uint8_t calib[24];
	HAL_I2C_Mem_Read(&hi2c1,BMP280_I2C_ADDR<<1 ,BMP280_REG_CALIB00,I2C_MEMADD_SIZE_8BIT,calib,24,pdMS_TO_TICKS(100));

	//storing data to structure
	bmp280_cal.dig_T1=(calib[1]<<8)|calib[0];
	bmp280_cal.dig_T2=(calib[3]<<8)|calib[2];
	bmp280_cal.dig_T3 = (calib[5] << 8) | calib[4];
	bmp280_cal.dig_P1 = (calib[7] << 8) | calib[6];
	bmp280_cal.dig_P2 = (calib[9] << 8) | calib[8];
	bmp280_cal.dig_P3 = (calib[11] << 8) | calib[10];
	bmp280_cal.dig_P4 = (calib[13] << 8) | calib[12];
	bmp280_cal.dig_P5 = (calib[15] << 8) | calib[14];
	bmp280_cal.dig_P6 = (calib[17] << 8) | calib[16];
	bmp280_cal.dig_P7 = (calib[19] << 8) | calib[18];
	bmp280_cal.dig_P8 = (calib[21] << 8) | calib[20];
	bmp280_cal.dig_P9 = (calib[23] << 8) | calib[22];

}

void Read_Raw(void)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR<<1, 0xF7, I2C_MEMADD_SIZE_8BIT, values, 6, pdMS_TO_TICKS(100));
	if(status==HAL_OK)
	{
		temperature=((values[3]<<12)|(values[4]<<4)|(values[5]>>4));
		pressure=((values[0]<<12)|(values[1]<<4)|(values[2]>>4));
	}
	else
	{
		data_status=0;
	}
}

float bmp280_compensate_T_double(int32_t temp)
{
float var1, var2, T;
var1 = (((float)temp)/16384.0f - ((float)bmp280_cal.dig_T1)/1024.0f) * ((float)bmp280_cal.dig_T2);
var2 = ((((float)temp)/131072.0f - ((float)bmp280_cal.dig_T1)/8192.0f) *
(((float)temp)/131072.0f - ((float) bmp280_cal.dig_T1)/8192.0f)) * ((float)bmp280_cal.dig_T3);
t_fine = (float)(var1 + var2);
T = (var1 + var2) / 5120.0f;
return T;
}
// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
float bmp280_compensate_P_double(int32_t pres)
{
float var1, var2, p;
var1 = ((float)t_fine/2.0f) - 64000.0f;
var2 = var1 * var1 * ((float)bmp280_cal.dig_P6) / 32768.0f;
var2 = var2 + var1 * ((float)bmp280_cal.dig_P5) * 2.0f;
var2 = (var2/4.0f)+(((float)bmp280_cal.dig_P4) * 65536.0f);
var1 = (((float)bmp280_cal.dig_P3) * var1 * var1 / 524288.0f + ((float)bmp280_cal.dig_P2) * var1) / 524288.0f;
var1 = (1.0f + var1 / 32768.0f)*((float)bmp280_cal.dig_P1);
if (var1 == 0.0f)
{
return 0; // avoid exception caused by division by zero
}
p = 1048576.0f - (float)pres;
p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
var1 = ((float)bmp280_cal.dig_P9) * p * p / 2147483648.0f;
var2 = p * ((float)bmp280_cal.dig_P8) / 32768.0f;
p = p + (var1 + var2 + ((float)bmp280_cal.dig_P7)) / 16.0f;
return p;
}


float BMP280_Get_Altitude(float pressure_pa, float reference_pa)
{
	return (44330.0f *(1-powf((pressure_pa/reference_pa),0.190295f)));
}

uint8_t BMP280_ReadChip(void)
{
	uint8_t data=0;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(&hi2c1,BMP280_I2C_ADDR<<1,BMP280_REG_ID,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&data,1,pdMS_TO_TICKS(100));
	if(status == HAL_OK)
	{
		return data;
	}
	return 0x01;
}


void BMP280_TASK(void *pvParameters)
{
    chip_id_result = BMP280_ReadChip();
    bmp280_ok = (chip_id_result == 0x58) ? 1 : 0;

    if (bmp280_ok)
    {
        I2C_Write(0xF5, 0x10);
        I2C_Write(0xF4, 0x53);
        BMP280_Read_Calibration();
    }

    for (;;)
    {
        if (bmp280_ok)
        {
            Read_Raw();
            if (data_status)
            {
                temp_c     = bmp280_compensate_T_double((int32_t)temperature);
                pressure_c = bmp280_compensate_P_double((int32_t)pressure);
            }
            if(!ground_captured)
            {
            	warmup_count++;
            	if(warmup_count>=BMP280_WARMUP_SAMPLES)
            	{
            		ground_pressure_pa=pressure_c;
            		ground_captured=1;
            	}
            }
            else
            {
            	altitude_m=BMP280_Get_Altitude(pressure_c,ground_pressure_pa);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}
