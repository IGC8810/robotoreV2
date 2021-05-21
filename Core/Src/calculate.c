#include "calculate.h"

float mileage(float mm) {
	return mm * ENC_PULSE_MM;
}

float ComplementaryFilter(float high_cut, float low_cut, float alpha, float complement_before) {
	float complement;

	complement = alpha * (complement_before + high_cut * DELTA_T) + (1.0f - alpha) * low_cut;

	return complement;
}

void posPID(float difference, float* order_posR, float* order_posL) {

	float p_pos, d_pos;
	static float i_pos;
	float kp_pos = 0.10f, ki_pos = 0.004f, kd_pos = 0.008f;
	static float def_pos[] = {0.0f, 0.0f};

	def_pos[0] = difference;

	p_pos = kp_pos * def_pos[0];
	i_pos += ki_pos * def_pos[0] * DELTA_T;
	d_pos = kd_pos * (def_pos[0] - def_pos[1]) / DELTA_T;

	*order_posR = p_pos + i_pos + d_pos;
	*order_posL = -(p_pos + i_pos + d_pos);

	def_pos[1] = def_pos[0];
}

void velPID(float target_vel, float , float* order_velR, float* order_velL) {
	float p_vel, kp_vel = 2.8f, ki_vel = 50.0f;
	float filter_vel_center, acceleration_imu;
	static float i_vel, def_vel, last_vel_center;

	//velR = (float)enc_tim8_ms * ENC_PULSE_MM * 1000.0f;
	//velL = (float)enc_tim1_ms * ENC_PULSE_MM * 1000.0f;
	//vel_center = (velR + velL) / 2.0f;
	acceleration_imu = (float)xa / 16384.0f;
	filter_vel_center = ComplementaryFilter(acceleration_imu, vel, 0.65f, last_vel_center);
	last_vel_center = filter_vel_center;

	def_vel = filter_vel_center - target_vel;

	p_vel = kp_vel * def_vel;
	i_vel += ki_vel * def_vel * DELTA_T;

	*order_velR = p_vel + i_vel;
	*order_velL = p_vel + i_vel;
}
