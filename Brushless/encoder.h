#pragma once

#include "stm32_nucleo_ihm07m1.h"

void reset_encoder(void);
float get_speed(void);
float get_magnetic_theta_deg(void);
void SpeedTimer_PeriodElapsedCallback();