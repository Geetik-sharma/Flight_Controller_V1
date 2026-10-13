/*
 * app_task.c
 *
 *  Created on: 29-Jul-2026
 *      Author: GeetikSharma
 */
#include "main.h"        // pulls in HAL types + (transitively) usart.h's huart1 extern
#include "FreeRTOS.h"
#include "task.h"         // or cmsis_os.h if that's what declares xTaskCreate/vTaskStartScheduler for you
#include <stdio.h>
#include "usart.h"
#include "bmp280.h"
#include "MPU9250.h"
#include "RC_Capture.h"
#include "tim.h"
#include "Motors.h"
#include "PID.h"
#include "event_groups.h"
#include "app_task.h"


EventGroupHandle_t xSensorEventGroup;
int _write(int file, char *ptr, int len)
{
	HAL_StatusTypeDef status=HAL_UART_Transmit(&huart1,(uint8_t*)ptr,len,pdMS_TO_TICKS(100));
	if(status == HAL_OK)
	{
		return len;
	}
	return -1;
}
// forward declare your task functions before AppTasksInit uses them
void led_toggle(void *pvParameters);
void message(void *pvParameters);
uint32_t counter=0;

int  setvbuf(FILE *stream, char *buf, int mode, size_t size);

void AppTasksInit(void)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	xSensorEventGroup=xEventGroupCreate();
	  xTaskCreate(led_toggle,"ledToggle",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	  xTaskCreate(BMP280_TASK,"BMP280",128,NULL,3,NULL);
	  xTaskCreate(MPU9250_TASK,"MPU9250",128,NULL,4,NULL);
	  xTaskCreate(Motor_Task,"Motor Task",128,NULL,2,NULL);
	  xTaskCreate(Control_Task,"PID Controller",128,NULL,5,NULL);
}

void led_toggle(void *pvParameters)
{
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}


