///*
// * PID.c
// *
// *  Created on: 25-Aug-2026
// *      Author: GeetikSharma
// */
//
#include "PID.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "RC_Capture.h"
#include "MPU9250.h"
#include "Motors.h"
#include "event_groups.h"
#include "app_task.h"

uint8_t Max_Angle_fixed=30;
uint8_t pilot_armed = 0;


typedef struct
{
	float kp;
	float ki;
	float kd;
	float integral;
	float prev_error;
}PID_Controller;

PID_Controller Roll_PID={2.0f, 0.0f, 0.0f, 0.0f, 0.0f};
PID_Controller Roll_Rate_PID = {0.5f, 0.1f, 0.01f, 0.0f, 0.0f};
PID_Controller Pitch_PID={2.0f, 0.0f, 0.0f, 0.0f, 0.0f};
PID_Controller Pitch_Rate_PID = {0.5f, 0.1f, 0.01f, 0.0f, 0.0f};
PID_Controller Yaw_PID={2.0f, 0.0f, 0.0f, 0.0f, 0.0f};
PID_Controller Yaw_Rate_PID = {0.5f, 0.1f, 0.01f, 0.0f, 0.0f};

//typedef struct
//{
//	float p;
//	float i;
//	float d;
//}PID_Output;
//
//PID_Output roll_pid_output={0.0f, 0.0f, 0.0f};
//PID_Output pitch_pid_output={0.0f, 0.0f, 0.0f};
//PID_Output yaw_pid_output={0.0f, 0.0f, 0.0f};

float target_roll=0;
float roll_error=0;
float roll_pid_output = 0.0f;
float target_pitch=0;
float pitch_error=0;
float pitch_pid_output = 0.0f;
float target_yaw_rate=0;
float yaw_error=0;
float yaw_pid_output = 0.0f;

static float PPM_Clamp(float PPM)
{
	if(PPM<1000)
	{
		return 1000;
	}
	if(PPM>2000)
	{
		return 2000;
	}
	return PPM;
}


float PPM_to_Angle(float PPM, uint8_t Max_Angle)
{
    PPM = PPM_Clamp(PPM);
    return ((PPM - 1500.0f) / 500.0f) * (float)Max_Angle;
}

float PPM_to_Rate(float PPM, float Max_Rate)
{
    PPM = PPM_Clamp(PPM);
    return ((PPM - 1500.0f) / 500.0f) * Max_Rate;
}

float PID_Compute(PID_Controller *pid, float target, float current, float dt)
{
    float error = target - current;

    float p_term = pid->kp * error;

    pid->integral += (error + pid->prev_error) / 2.0f * dt;

    if (pid->integral > 100.0f) { pid->integral = 100.0f; }
    if (pid->integral < -100.0f) { pid->integral = -100.0f; }

    float i_term = pid->ki * pid->integral;

    // TODO before real flight: raw derivative amplifies gyro noise
    // ("derivative kick"). Add low-pass filtering on this term before
    // tuning D gains for actual flight — see project notes.
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;

    pid->prev_error = error;

    float output = p_term + i_term + d_term;

    if (output > 100.0f) { output = 100.0f; }
    if (output < -100.0f) { output = -100.0f; }

    return output;
}



#define ARM_SWITCH_HIGH   1900
#define ARM_SWITCH_LOW    1100
#define ARM_THROTTLE_MAX  1050

void Update_Arm_State(void)
{
	uint16_t arming_value=PPM_Channels[4];
	float throttle=PPM_Channels[2];
    if(!pilot_armed)
    {
    	if(arming_value>=ARM_SWITCH_HIGH && throttle<=ARM_THROTTLE_MAX)
    	{
    		pilot_armed=1;
    	}
    }
    else
    {
    	if(arming_value<=ARM_SWITCH_LOW)
    	{
    		pilot_armed=0;
    	}
    }
}

static TickType_t last_wake_tick = 0;

void Control_Task(void *pvParameters)
{
    static uint8_t flag=1;
    float dt=0.0f;
    for (;;)
    {
       	xEventGroupWaitBits(
        	    xSensorEventGroup,      // which event group
        	    ATTITUDE_READY_BIT,     // which bit(s) to wait for
        	    pdTRUE,                 // clear the bit automatically once received (so next signal must be genuinely new)
        	    pdFALSE,                // wait for ANY of the specified bits (irrelevant with just one bit now; matters once ALTITUDE_READY_BIT joins)
        	    portMAX_DELAY            // block forever if needed — no timeout
        	);

       	TickType_t now = xTaskGetTickCount();
       	Update_Arm_State();
       	if(flag)
       	{
       		dt=0.004f;
       		flag=0;
       	}
       	else{
       		dt=(float)(now - last_wake_tick) * portTICK_PERIOD_MS / 1000.0f;
       	}
       	last_wake_tick=now;
       	target_roll = PPM_to_Angle(PPM_Channels[0], 30);
       	roll_error = target_roll - Roll;
       	float target_roll_rate = PID_Compute(&Roll_PID, target_roll, Roll, dt);
       	roll_pid_output = PID_Compute(&Roll_Rate_PID, target_roll_rate, Gyro_dps[0], dt);

       	target_pitch=PPM_to_Angle(PPM_Channels[1],30);
       	pitch_error=target_pitch - Pitch;
       	float target_pitch_rate = PID_Compute(&Pitch_PID,target_pitch,Pitch,dt);
       	pitch_pid_output=PID_Compute(&Pitch_Rate_PID, target_pitch_rate, Gyro_dps[1],dt);

       	target_yaw_rate=PPM_to_Rate(PPM_Channels[3],30);
       	yaw_error = target_yaw_rate - Gyro_dps[2];
       	yaw_pid_output = PID_Compute(&Yaw_Rate_PID,target_yaw_rate,Gyro_dps[2],dt);

       	if (motor_armed_flag && pilot_armed)
       	{
       	    float throttle = (float)PPM_Channels[2];
       	    Motor_Mixer(roll_pid_output, pitch_pid_output, yaw_pid_output, throttle);
       	}
       	else
       	{
       	    Set_Motor_Speed(MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MIN);
       	}
    }
}
