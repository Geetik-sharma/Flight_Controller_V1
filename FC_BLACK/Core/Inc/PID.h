/*
 * PID.h
 *
 *  Created on: 25-Aug-2026
 *      Author: GeetikSharma
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
extern float target_roll;
extern float roll_error;
extern float roll_pid_output;
extern float pitch_pid_output;
extern float yaw_pid_output;
extern uint8_t pilot_armed;

extern uint8_t Max_Angle_fixed;
//
//typedef struct
//{
//    float kp;
//    float ki;
//    float kd;
//    float integral;
//    float prev_error;
////    float d_lpf;          // Low-pass filtered derivative state
//} PID_Controller;
//
//extern PID_Controller Roll_PID;
//extern PID_Controller Pitch_PID;
//extern PID_Controller Yaw_PID;
//extern PID_Controller Roll_Rate_PID;
//extern PID_Controller Pitch_Rate_PID;
//extern PID_Controller Yaw_Rate_PID;
float PPM_to_Angle(float PPM, uint8_t Max_Angle);
void Control_Task(void *pvParameters);
#endif /* INC_PID_H_ */
