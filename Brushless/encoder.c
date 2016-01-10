#include "6Step_Lib.h"

float speed;
uint32_t clock;

void reset_encoder()
{	
	__HAL_TIM_SetCounter(&HALL_ENCODER_TIMx, 0);
		
		//uint16_t time = __HAL_TIM_GetCounter(&htim15);
		//
		//__HAL_TIM_SetCounter(&htim15, 0);
		
		//speed = (float)clock / 4 / 200 / (float)time;
		
		//El_Speed_Hz = (int32_t)((SIXSTEP_parameters.SYSCLK_frequency) / (prsc)) / (__HAL_TIM_GetAutoreload(&LF_TIMx) * 6);
		
	uint8_t t = 1;
}