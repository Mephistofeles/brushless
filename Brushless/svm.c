#include "svm.h"
#include "l6230.h"

void SVPWM_run(float a, float m)
{
	//float X, Y, Z;
	//float PWMa, PWMb, PWMc;
	uint8_t sector;

	float Ualpha = m * cosf(a * 0.0174532925f);
	float Ubeta = m * sinf(a * 0.0174532925f);
	
	float angle = atan2f(Ubeta, Ualpha);
	float magnitude = sqrtf(Ualpha * Ualpha + Ubeta * Ubeta);
	
	if (angle < 0) angle = 2 * PI + angle;
	
	if (angle <= PI / 3) sector = 0;
	else if (angle <= 2 * PI / 3) sector = 1;
	else if (angle <= 3 * PI / 3) sector = 2;
	else if (angle <= 4 * PI / 3) sector = 3;
	else if (angle <= 5 * PI / 3) sector = 4;
	else sector = 5;
	
	float restricted_angle_0_60 = fmodf(angle, PI / 3);
	
	float calculated_Ualpha = magnitude * cosf(restricted_angle_0_60);
	float calculated_Ubeta = magnitude * sinf(restricted_angle_0_60);
	
	float timeA = calculated_Ualpha - calculated_Ubeta / sqrtf(3);
	float timeB = 2 / sqrtf(3) * calculated_Ubeta;
	float time07 = 1 - timeA - timeB;
	float dutyU, dutyV, dutyW;
	
	float t02 = time07 / 2.0f;
	
	switch (sector)
	{
	case 0: // 100 -> 110
		dutyU = t02;
		dutyV = t02 + timeA;
		dutyW = 1 - t02;
		break;
	case 1: // 110 -> 010
		dutyU = t02 + timeB;
		dutyV = t02;
		dutyW = 1 - t02;
		break;
	case 2: // 010 -> 011
		dutyU = 1 - t02;
		dutyV = t02;
		dutyW = t02 + timeA;
		break;
	case 3: // 011 -> 001
		dutyU = 1 - t02;
		dutyV = t02 + timeB;
		dutyW = t02;
		break;
	case 4: // 001 -> 101
		dutyU = t02 + timeA;
		dutyV = 1 - t02;
		dutyW = t02;
		break;
	case 5: // 101 -> 100
		dutyU = t02;
		dutyV = 1 - t02;
		dutyW = t02 + timeB;
		break;
	default:
		break;
	}
	
	uint16_t phase_U_enable_duty_cycle = 718 * dutyU;
	uint16_t phase_V_enable_duty_cycle = 718 * dutyV;
	uint16_t phase_W_enable_duty_cycle = 718 * dutyW;	
	
	L6230_HFTIM_DC_CH1(phase_U_enable_duty_cycle);
	L6230_HFTIM_DC_CH2(phase_V_enable_duty_cycle);
	L6230_HFTIM_DC_CH3(phase_W_enable_duty_cycle);
}