/*
 * Motors.h
 *
 *  Created on: 23-Aug-2026
 *      Author: GeetikSharma
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_
#include <stdint.h>
 #define MOTOR_PWM_MIN 1000
 #define MOTOR_PWM_MAX 2000
extern volatile uint8_t motor_armed_flag;
void Motor_Init(void);
void Set_Motor_Speed(uint16_t M1, uint16_t M2, uint16_t M3, uint16_t M4);
void Motor_Mixer(float roll,float pitch,float yaw,float throttle);
void Motor_Task(void *pvParameters);
#endif /* INC_MOTORS_H_ */
