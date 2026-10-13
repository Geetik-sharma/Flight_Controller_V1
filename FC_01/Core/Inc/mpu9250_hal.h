/*
 * mpu9250_hal.h
 *
 *  Created on: 28-Jul-2026
 *      Author: GeetikSharma
 */

#ifndef INC_MPU9250_HAL_H_
#define INC_MPU9250_HAL_H_
#include "stm32f1xx_hal.h"

void mpu9250_write_reg(uint8_t reg_add,uint8_t reg_val);
void mpu9250_read_reg(uint8_t reg_add, uint8_t *buffer, uint8_t length);
#endif /* INC_MPU9250_HAL_H_ */
