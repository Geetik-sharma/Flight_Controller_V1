/*
 * MPU9250.c
 *
 *  Created on: 07-Aug-2026
 *      Author: GeetikSharma
 */
#include "MPU9250_Regi.h"
#include "MPU9250.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "spi.h"
#include "event_groups.h"
#include "app_task.h"

uint8_t mpu_who_am_i=0;
uint8_t mpu_ok=0;
uint8_t Raw_Data[14];
int16_t Data[7];//3accel 1temp 3gyro

float Accel_g[3];   // X, Y, Z in g
float Gyro_dps[3];  // X, Y, Z in degrees/sec
float Temp_C;

float accel_bias[3];
float gyro_bias[3];

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // identity quaternion — "no rotation yet"

float Kp = 2.0f;    // proportional gain — how aggressively accel corrects gyro, immediately
float Ki = 0.005f;  // integral gain — how aggressively persistent bias gets learned out

float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;   // accumulated bias correction

float halfvx, halfvy, halfvz;   // declare as plain globals, no initializer — just reserve the storage
float halfex, halfey, halfez;

float gx, gy, gz;   // corrected gyro rate, in rad/s — used by the integration step next

float Roll, Pitch, Yaw;


uint8_t MPU9250_ReadRegister(uint8_t reg)
{
	uint8_t tx_buf[2];
	uint8_t rx_buf[2];

	tx_buf[0]=reg | 0x80;
	tx_buf[1]=0x00;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,tx_buf,rx_buf,2,pdMS_TO_TICKS(100));
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	return rx_buf[1];
}


void MPU9250_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    tx_buf[0] = reg;
    tx_buf[1] = value;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}


void MPU9250_Init(void)
{
    MPU9250_WriteRegister(PWR_MGMT_01, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    MPU9250_WriteRegister(PWR_MGMT_01, 0x01);
    MPU9250_WriteRegister(PWR_MGMT_02, 0x00);
    vTaskDelay(pdMS_TO_TICKS(200));

    MPU9250_WriteRegister(ACCEL_CONFIG_1, 0x08);   // ACCEL_CONFIG — fill in the address
    MPU9250_WriteRegister(GYRO_CONFIG, 0x10);   // GYRO_CONFIG — fill in the address
}


void MPU9250_Scale(void)
{
    Accel_g[0] = (float)Data[0] / 8192.0f;
    Accel_g[1] = (float)Data[1] / 8192.0f;
    Accel_g[2] = (float)Data[2] / 8192.0f;

    Temp_C = ((float)Data[3] / 333.87f) + 21.0f;   // datasheet formula, confirmed earlier

    Gyro_dps[0] = (float)Data[4]/32.8f;   // <- fill in, same pattern as accel, using the
    Gyro_dps[1] = (float)Data[5]/32.8f;   //    32.8 divisor
    Gyro_dps[2] = (float)Data[6]/32.8f;
}

void MPU9250_ReadRaw(void)
{
    uint8_t tx_buf[15] = {ACCEL_XOUT_H | 0x80, 0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint8_t rx_buf[15];

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 15, pdMS_TO_TICKS(200));
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    for (int i = 0; i < 14; i++)
    {
        Raw_Data[i] = rx_buf[i + 1];   // shift past the garbage address-phase byte
    }
    Data[0]=(Raw_Data[0]<<8)|Raw_Data[1];
    Data[1]=(Raw_Data[2]<<8)|Raw_Data[3];
    Data[2]=(Raw_Data[4]<<8)|Raw_Data[5];
    Data[3]=(Raw_Data[6]<<8)|Raw_Data[7];
    Data[4]=(Raw_Data[8]<<8)|Raw_Data[9];
    Data[5]=(Raw_Data[10]<<8)|Raw_Data[11];
    Data[6]=(Raw_Data[12]<<8)|Raw_Data[13];
}

//calibration for accel and gyro
void MPU9250_Calibrate(uint16_t samples)
{
	float accel_sum[3]={0.0f,0.0f,0.0f};
	float gyro_sum[3]={0.0f,0.0f,0.0f};
	for (uint16_t i=0;i<samples;i++)
	{
		MPU9250_ReadRaw();
		MPU9250_Scale();
		accel_sum[0]+=Accel_g[0];
		accel_sum[1]+=Accel_g[1];
		accel_sum[2]+=Accel_g[2];

		gyro_sum[0]+=Gyro_dps[0];
		gyro_sum[1]+=Gyro_dps[1];
		gyro_sum[2]+=Gyro_dps[2];
		vTaskDelay(pdMS_TO_TICKS(4));
	}
	accel_bias[0]=accel_sum[0]/(float)samples-0.0f;
	accel_bias[1]=accel_sum[1]/(float)samples-0.0f;
	accel_bias[2]=accel_sum[2]/(float)samples-1.0f;

	gyro_bias[0]=gyro_sum[0]/(float)samples-0.0f;
	gyro_bias[1]=gyro_sum[1]/(float)samples-0.0f;
	gyro_bias[2]=gyro_sum[2]/(float)samples-0.0f;
}
void MPU9250_Calibrated(void)
{
	Accel_g[0]-=accel_bias[0];
	Accel_g[1]-=accel_bias[1];
	Accel_g[2]-=accel_bias[2];

	Gyro_dps[0]-=gyro_bias[0];
	Gyro_dps[1]-=gyro_bias[1];
	Gyro_dps[2]-=gyro_bias[2];
}

void MPU9250_EstimateGravity(void)
{
    halfvx = q1*q3 - q0*q2;
    halfvy = q0*q1 + q2*q3;
    halfvz = q0*q0 - 0.5f + q3*q3;
}

void MPU9250_ComputeError(void)
{
    float recipNorm = 1.0f / sqrtf(Accel_g[0]*Accel_g[0] + Accel_g[1]*Accel_g[1] + Accel_g[2]*Accel_g[2]);
    float ax = Accel_g[0] * recipNorm;
    float ay = Accel_g[1] * recipNorm;
    float az = Accel_g[2] * recipNorm;

    halfex = (ay*halfvz - az*halfvy);
    halfey = (az*halfvx - ax*halfvz);
    halfez = (ax*halfvy - ay*halfvx);
}

void MPU9250_ApplyCorrection(float dt)
{
    if (Ki > 0.0f)
    {
        integralFBx += Ki * halfex * dt;
        integralFBy += Ki * halfey * dt;
        integralFBz += Ki * halfez * dt;
    }
    else
    {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    gx = (Gyro_dps[0] * (float)M_PI / 180.0f) + Kp * halfex + integralFBx;
    gy = (Gyro_dps[1] * (float)M_PI / 180.0f) + Kp * halfey + integralFBy;
    gz = (Gyro_dps[2] * (float)M_PI / 180.0f) + Kp * halfez + integralFBz;
}

void MPU9250_IntegrateQuaternion(float dt)
{
    float qa = q0, qb = q1, qc = q2;

    q0 += (-qb*gx - qc*gy - q3*gz) * (0.5f * dt);
    q1 += (qa*gx + qc*gz - q3*gy) * (0.5f * dt);
    q2 += (qa*gy - qb*gz + q3*gx) * (0.5f * dt);
    q3 += (qa*gz + qb*gy - qc*gx) * (0.5f * dt);
}

void MPU9250_NormalizeQuaternion(void)
{
    float recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void MPU9250_UpdateFilter(float dt)
{
	MPU9250_EstimateGravity();
	MPU9250_ComputeError();
	MPU9250_ApplyCorrection(dt);
	MPU9250_IntegrateQuaternion(dt);
	MPU9250_NormalizeQuaternion();
	Roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f / (float)M_PI;
	Pitch = asinf(2.0f*(q0*q2 - q3*q1)) * 180.0f / (float)M_PI;
	Yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f / (float)M_PI;
}

void MPU9250_TASK(void *pvParameters)
{
	// 1. Give the sensor hardware 100ms to stabilize its power rail on cold boot
	    vTaskDelay(pdMS_TO_TICKS(100));

	    // 2. Retry loop in case the sensor takes a moment to respond
	    uint8_t retries = 5;
	    while (retries-- > 0 && !mpu_ok)
	    {
	        mpu_who_am_i = MPU9250_ReadRegister(WHO_AM_I);
	        mpu_ok = (mpu_who_am_i == 0x71 || mpu_who_am_i == 0x70) ? 1 : 0;
	        if (!mpu_ok) {
	            vTaskDelay(pdMS_TO_TICKS(50));
	        }
	    }
	if (mpu_ok)
	{
		MPU9250_Init();
		MPU9250_Calibrate(1000);
	}
	TickType_t xLastWakeTime=xTaskGetTickCount();
	for(;;)
	{
		if(mpu_ok)
			{
				MPU9250_ReadRaw();
				MPU9250_Scale();
				MPU9250_Calibrated();
				MPU9250_UpdateFilter(0.004f);
				xEventGroupSetBits(xSensorEventGroup,ATTITUDE_READY_BIT);
			}
		vTaskDelayUntil(&xLastWakeTime,pdMS_TO_TICKS(4));
	}
}

