/*
 * Motors.c
 *
 *  Created on: 23-Aug-2026
 *      Author: GeetikSharma
 */
#include "Motors.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "PID.h"
#include "RC_Capture.h"


volatile uint8_t motor_armed_flag=0;

void Motor_Init(void)
{
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1000);/*i have used 1000 ticks because i set my timer clk at 1MHz so 1 tick = 1us and 1000ticks=1000us*/
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,1000);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,1000);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    vTaskDelay(pdMS_TO_TICKS(3000));
    motor_armed_flag=1;

}

static uint16_t Motors_Clamp(uint16_t value)
{
    if (value < MOTOR_PWM_MIN)
    {
        return MOTOR_PWM_MIN;
    }
    if (value > MOTOR_PWM_MAX)
    {
        return MOTOR_PWM_MAX;
    }
    return value;
}

static float Motors_Clamp_f(float value)
{
    if (value < (float)MOTOR_PWM_MIN) { return (float)MOTOR_PWM_MIN; }
    if (value > (float)MOTOR_PWM_MAX) { return (float)MOTOR_PWM_MAX; }
    return value;
}

void Set_Motor_Speed(uint16_t M1, uint16_t M2, uint16_t M3, uint16_t M4)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Motors_Clamp(M1));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Motors_Clamp(M2));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Motors_Clamp(M3));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Motors_Clamp(M4));
}
float M1_debug=1000;
float M2_debug=1000;
float M3_debug=1000;
float M4_debug=1000;
void Motor_Mixer(float roll,float pitch,float yaw,float throttle)
{
	float M1 = Motors_Clamp_f(throttle - roll - pitch - yaw);
	float M2 = Motors_Clamp_f(throttle + roll + pitch - yaw);
	float M3 = Motors_Clamp_f(throttle + roll - pitch + yaw);
	float M4 = Motors_Clamp_f(throttle - roll + pitch + yaw);
	M1_debug=M1;
	M2_debug=M2;
	M3_debug=M3;
	M4_debug=M4;
	Set_Motor_Speed((uint16_t)M1, (uint16_t)M2, (uint16_t)M3, (uint16_t)M4);

}

void Motor_Task(void *pvParameters)
{
	Motor_Init();
	for(;;)
	{
		vTaskDelay(1000);
	}
}

