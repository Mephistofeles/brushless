#pragma once

#include "stm32_nucleo_ihm07m1.h"

void reset_encoder(void);
float get_speed(void);
void SpeedTimer_PeriodElapsedCallback();