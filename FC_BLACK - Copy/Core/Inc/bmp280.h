#ifndef BMP280_H
#define BMP280_H

#include "stm32f4xx_hal.h" // Needed for I2C_HandleTypeDef
#include <stdint.h>
extern volatile uint8_t chip_id_result;
extern volatile uint8_t bmp280_ok;
// Public function prototypes
// Pass your configured I2C handle (e.g., &hi2c1) to the init function
uint8_t BMP280_Init(I2C_HandleTypeDef *hi2c);

float BMP280_Get_Temperature(void);
float BMP280_Get_Pressure(void);
uint8_t BMP280_ReadChip(void);
void BMP280_TASK(void *pvParameters);
float BMP280_Get_Altitude(float pressure_pa, float reference_pa);



#endif
