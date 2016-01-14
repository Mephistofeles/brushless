#include "svm.h"
#include "l6230.h"

void SVPWM_run(float a, float m)
{
	//float Ualpha, Ubeta;
	//float X, Y, Z;
	//float PWMa, PWMb, PWMc;
	uint8_t sector;

	//Ualpha = m * cos(a);
	//Ubeta = m * sin(a);
	
	//float angle = a;
	//if (angle >= 180.0f) angle -= 360.0f;
		//
	//arm_sin_cos_f32(angle, &Ubeta, &Ualpha);
//
	//X = Ubeta;
	//Y = Ubeta * 0.5f + Ualpha * 0.8660254f; //(Ubeta + Ualpha*sqrt(3))/2
	//Z = X - Y;				//(Ubeta - Ualpha*sqrt(3))/2

	if (a <= 60) sector = 0;
	else if (a <= 120) sector = 1;
	else if (a <= 180) sector = 2;
	else if (a <= 240) sector = 3;
	else if (a <= 300) sector = 4;
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
	
	//uint8_t restricted_angle = (uint16_t)a % 60;
	//
	//float restricted_cos = arm_cos_f32((restricted_angle + 30) * 0.0174532925f);
	//float restricted_sin = arm_sin_f32((restricted_angle) * 0.0174532925f);
	//
	//float t1 = 0.8660254f * m * restricted_cos;
	//float t2 = 0.8660254f * m * restricted_sin;
	//float t0 = 1 - t1 - t2;
	
	float rad = 0.0174532925f * fmodf(a, 60);
	
	float ta = 1.15470053838f * m * cosf(rad + PI / 6);
	float tb = 1.15470053838f * m * sinf(rad);
	float t07 = 1 - ta - tb;
	
	float t02 = t07 / 4;
	
	float dutyU, dutyV, dutyW;
	
	switch (sector) {
	case 0: // 100 -> 110
		dutyU = t02;
		dutyV = t02 + ta;
		dutyW = 1 - t02;
		break;
	case 1: // 110 -> 010
		dutyU = t02 + tb;
		dutyV = t02;
		dutyW = 1 - t02;
		break;
	case 2: // 010 -> 011
		dutyU = 1 - t02;
		dutyV = t02;
		dutyW = t02 + ta;
		break;
	case 3: // 011 -> 001
		dutyU = 1 - t02;
		dutyV = t02 + tb;
		dutyW = t02;
		break;
	case 4: // 001 -> 101
		dutyU = t02 + ta;
		dutyV = 1 - t02;
		dutyW = t02;
		break;
	case 5: // 101 -> 100
		dutyU = t02;
		dutyV = 1 - t02;
		dutyW = t02 + tb;
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