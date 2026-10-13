/*
 * RC_Capture.c
 *
 *  Created on: 14-Aug-2026
 *      Author: GeetikSharma
 */

#include "RC_Capture.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#define PPM_CHANNELS 6
#define PPM_SYNC_THRESHOLD_US 3000

volatile uint16_t PPM_Channels[PPM_CHANNELS];
volatile uint8_t PPM_Channel_Index = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t last_counter=0;

	if(htim->Instance==TIM2 && htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
	{
		uint32_t current_counter=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
		uint32_t interval=current_counter-last_counter;
		last_counter = current_counter;
		if(interval>=PPM_SYNC_THRESHOLD_US)
		{
			PPM_Channel_Index=0;
		}
		else
		{
			if(PPM_Channel_Index < PPM_CHANNELS)
			{
				PPM_Channels[PPM_Channel_Index]=interval;
				PPM_Channel_Index++;
			}
		}
	}
}
