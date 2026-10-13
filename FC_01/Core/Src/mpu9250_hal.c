/*
 * mpu9250_hal.c
 *
 *  Created on: 28-Jul-2026
 *      Author: GeetikSharma
 */
#include "mpu9250_hal.h"
#include "mpu9250_reg.h"

extern SPI_HandleTypeDef hspi1;

#define CS_LOW() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)

void mpu9250_write_reg(uint8_t reg_add,uint8_t reg_val)
{
	uint8_t tx[2];
	uint8_t rx[2];

	tx[0]=reg_add & 0x7F;
	tx[1]=reg_val;
	CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1,tx,rx,2,HAL_MAX_DELAY);
	CS_HIGH();
}

void mpu9250_read_reg(uint8_t reg_add, uint8_t *buffer, uint8_t length)
{
	uint8_t addr_byte = reg_add | 0x80;   // MSB = 1 -> read
	uint8_t dummy_rx;
	uint8_t tx_dummy[14] = {0};            // enough for our biggest burst read

	CS_LOW();

	    // Step 1: send register address
	HAL_SPI_TransmitReceive(&hspi1, &addr_byte, &dummy_rx, 1, HAL_MAX_DELAY);

	    // Step 2: clock out dummy bytes to pull real data back
	HAL_SPI_TransmitReceive(&hspi1, tx_dummy, buffer, length, HAL_MAX_DELAY);

	CS_HIGH();
}
