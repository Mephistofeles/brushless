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
		
	stopped = FALSE;
}

float get_speed()
{
	return speed;
}

float get_magnetic_theta_deg()
{
	return fmodf(__HAL_TIM_GetCounter(&htim2) / 4095.0f * 360.0f, 51.42f);
}

void SpeedTimer_PeriodElapsedCallback()
{
	stopped = TRUE;
	speed = 0.0f;	
}