/*
 * MPU9250.h
 *
 *  Created on: 07-Aug-2026
 *      Author: GeetikSharma
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_
#include "stm32f4xx_hal.h" // Needed for I2C_HandleTypeDef
#include <stdint.h>

extern float Roll,Pitch,Yaw;
extern float Accel_g[3];   // X, Y, Z in g
extern float Gyro_dps[3];

void MPU9250_TASK(void *pvParameters);
void MPU9250_WriteRegister(uint8_t reg, uint8_t value);
void MPU9250_Init(void);
uint8_t MPU9250_ReadRegister(uint8_t reg);
void MPU9250_Calibrate(uint16_t samples);
void MPU9250_Calibrated(void);

#endif /* INC_MPU9250_H_ */
