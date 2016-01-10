#include "6Step_Lib.h"

_Bool stopped = TRUE;
float speed = 0.0f;
uint32_t clock = 72000000;

void reset_encoder()
{	
	__HAL_TIM_SetCounter(&HALL_ENCODER_TIMx, 0);
		
	uint16_t time = __HAL_TIM_GetCounter(&htim15);
	__HAL_TIM_SetCounter(&htim15, 0);
		
	if (!stopped)
	{		
		speed = (45000.0f / time) * 60.0f;
	}
		
	//El_Speed_Hz = (int32_t)((SIXSTEP_parameters.SYSCLK_frequency) / (prsc)) / (__HAL_TIM_GetAutoreload(&LF_TIMx) * 6);
		
	stopped = FALSE;
}

float get_speed()
{
	return speed;
}

void SpeedTimer_PeriodElapsedCallback()
{
	stopped = TRUE;
	speed = 0.0f;	
}