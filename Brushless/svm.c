#include "svm.h"
#include "l6230.h"

void SVPWM_run(float a, float m)
{
	float Ualpha, Ubeta;
	float X, Y, Z;
	float PWMa, PWMb, PWMc;
	uint8_t sector;

	//Ualpha = m * cos(a);
	//Ubeta = m * sin(a);
	
	float angle = a;
	if (angle >= 180.0f) angle -= 360.0f;
		
	arm_sin_cos_f32(angle, &Ubeta, &Ualpha);

	X = Ubeta;
	Y = Ubeta * 0.5f + Ualpha * 0.8660254f; //(Ubeta + Ualpha*sqrt(3))/2
	Z = X - Y;				//(Ubeta - Ualpha*sqrt(3))/2

	if (a < 60) sector = 0;
	else if (a < 120) sector = 1;
	else if (a < 180) sector = 2;
	else if (a < 240) sector = 3;
	else if (a < 300) sector = 4;
	else sector = 5;

	//switch (sector) {
	//case 0:
	//case 3:
		//PWMa = Y;
		//PWMb = X + Z;
		//PWMc = -Y;
		//break;
	//case 1:
	//case 4:
		//PWMa = Y - Z;
		//PWMb = X;
		//PWMc = -X;
		//break;
	//case 2:
	//case 5:
		//PWMa = -Z;
		//PWMb = Z;
		//PWMc = -(Y + X);
		//break;
	//default:
		//break;
	//}
	
	uint8_t restricted_angle = (uint16_t)a % 60;
	
	float restricted_cos = arm_cos_f32((restricted_angle + 30) * 0.0174532925f);
	float restricted_sin = arm_sin_f32((restricted_angle) * 0.0174532925f);
	
	float t1 = 0.8660254f * m * restricted_cos;
	float t2 = 0.8660254f * m * restricted_sin;
	float t0 = 1 - t1 - t2;
	
	uint16_t phase_U_enable_duty_cycle = 0;
	uint16_t phase_V_enable_duty_cycle = 0;
	uint16_t phase_W_enable_duty_cycle = 0;
	
	
	
	switch (sector) {
	case 0:
		phase_U_enable_duty_cycle = (t1 + t2) * 719;
		phase_V_enable_duty_cycle = t2 * 719;
		phase_W_enable_duty_cycle = 0;
		break;
	case 1:
		phase_U_enable_duty_cycle = t1 * 719;
		phase_V_enable_duty_cycle = (t1 + t2) * 719;
		phase_W_enable_duty_cycle = 0;
		break;
	case 2:
		phase_U_enable_duty_cycle = 0;
		phase_V_enable_duty_cycle = (t1 + t2) * 719;
		phase_W_enable_duty_cycle = t2 * 719;
		break;
	case 3:
		phase_U_enable_duty_cycle = 0;
		phase_V_enable_duty_cycle = t1 * 719;
		phase_W_enable_duty_cycle = (t1 + t2) * 719;
		break;
	case 4:
		phase_U_enable_duty_cycle = t2 * 719;
		phase_V_enable_duty_cycle = 0;
		phase_W_enable_duty_cycle = (t1 + t2) * 719;
		break;
	case 5:
		phase_U_enable_duty_cycle = (t1 + t2) * 719;
		phase_V_enable_duty_cycle = 0;
		phase_W_enable_duty_cycle = t1 * 719;
		break;
	default:
		break;
	}
	
	//arm_inv_clarke_f32(Ualpha, Ubeta, &PWMa, &PWMb);
	//
	//PWMc =  -0.5 * Ualpha - (float32_t) 0.8660254039 * Ubeta;
	
	//uint16_t phase_U_enable_duty_cycle = 350 + PWMa * 300;
	//uint16_t phase_V_enable_duty_cycle = 350 + PWMb * 300;
	//uint16_t phase_W_enable_duty_cycle = 350 + PWMc * 300;
	
	uint8_t phase_U_direction = PWMa > 0;
	uint8_t phase_V_direction = PWMb > 0;
	uint8_t phase_W_direction = PWMc > 0;	
	
	L6230_HFTIM_DC_CH1(phase_U_enable_duty_cycle);
	L6230_HFTIM_DC_CH2(phase_V_enable_duty_cycle);
	L6230_HFTIM_DC_CH3(phase_W_enable_duty_cycle);
}