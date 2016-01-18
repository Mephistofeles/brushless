#include <arm_math.h>
#include "field_oriented_control.h"

arm_pid_instance_f32 speed_omega_pi;
arm_pid_instance_f32 flux_id_pi;
arm_pid_instance_f32 torque_iq_pi;

void init_algorithm()
{
	speed_omega_pi.Kp = 1.0f;
	speed_omega_pi.Ki = 0.0f;
	speed_omega_pi.Kd = 0.0f;
	
	arm_pid_init_f32(&speed_omega_pi, 1);
		
	flux_id_pi.Kp = 1.0f;
	flux_id_pi.Ki = 0.0f;
	flux_id_pi.Kd = 0.0f;
	
	arm_pid_init_f32(&flux_id_pi, 1);
		
	torque_iq_pi.Kp = 1.0f;
	torque_iq_pi.Ki = 0.0f;
	torque_iq_pi.Kd = 0.0f;
	
	arm_pid_init_f32(&torque_iq_pi, 1);
}

void algorithm(float phase_a_current, float phase_b_current, float theta, float speed, float* Ualpha, float* Ubeta)
{
	float reference_speed = 0.0f;
	const float reference_flux_current = 0.0f;
	
	float Ialpha, Ibeta;
	arm_clarke_f32(phase_a_current, phase_b_current, &Ialpha, &Ibeta);
	
	float theta_sin, theta_cos;
	arm_sin_cos_f32(theta, &theta_sin, &theta_cos);
	
	float flux_current_id, torque_current_iq;
	arm_park_f32(Ialpha, Ibeta, &flux_current_id, &torque_current_iq, theta_sin, theta_cos);
	
	float reference_torque_current = arm_pid_f32(&speed_omega_pi, reference_speed - speed);
	float calculated_torque_voltage = arm_pid_f32(&torque_iq_pi, reference_torque_current - torque_current_iq);
	float calculated_flux_voltage = arm_pid_f32(&flux_id_pi, reference_flux_current - flux_current_id);
	
	arm_inv_park_f32(calculated_flux_voltage, calculated_torque_voltage, Ualpha, Ubeta, theta_sin, theta_cos);
}