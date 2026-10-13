#ifndef INC_RC_CAPTURE_H_
#define INC_RC_CAPTURE_H_

#include "main.h"

#define PPM_CHANNELS 6

extern volatile uint16_t PPM_Channels[PPM_CHANNELS];//
extern volatile uint8_t PPM_Channel_Index;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);


#endif
