/*
 * BMP280_reg.h
 *
 *  Created on: 29-Jul-2026
 *      Author: GeetikSharma
 */

#ifndef INC_BMP280_REG_H_
#define INC_BMP280_REG_H_

// bmp280_regs.h
#define BMP280_I2C_ADDR      0x76

// Core Registers
#define BMP280_REG_ID        0xD0 // Should return 0x58
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5

// Data Registers
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB  0xFA

// Calibration Data Start Address
#define BMP280_REG_CALIB00   0x88


#endif /* INC_BMP280_REG_H_ */
