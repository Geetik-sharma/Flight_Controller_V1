/*
 * app_task.h
 *
 *  Created on: 29-Jul-2026
 *      Author: GeetikSharma
 */

#ifndef INC_APP_TASK_H_
#define INC_APP_TASK_H_
#include "event_groups.h"
#define ATTITUDE_READY_BIT   (1 << 0)
#define ALTITUDE_READY_BIT   (1 << 1)
extern EventGroupHandle_t xSensorEventGroup;
void AppTasksInit(void);
void led_toggle(void *pvParameters);
void message(void *pvParameters);
#endif /* INC_APP_TASK_H_ */
