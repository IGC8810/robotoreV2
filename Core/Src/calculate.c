#include "calculate.h"

float order_posR = 0.0f;
float order_posL = 0.0f;
float order_velR = 0.0f;
float order_velL = 0.0f;

float mileage(float mm) {
	return mm * ENC_PULSE_MM;
}

float ComplementaryFilter(float high_cut, float low_cut, float alpha, float complement_before) {
	float complement;

	complement = alpha * (complement_before + high_cut * DELTA_T) + (1.0f - alpha) * low_cut;

	return complement;
}

void posPID(void) {

	float p_pos, d_pos;
	static float i_pos;
	float kp_pos = 0.10f, ki_pos = 0.004f, kd_pos = 0.008f;
	static float def_pos[] = {0.0f, 0.0f};

	def_pos[0] = ( ((float)line_senLLL * 1.6f) + ((float)line_senLL * 1.25f) + (float)line_senL) - ((float)line_senR + ((float)line_senRR * 1.25f) + ((float)line_senRRR * 1.6f)); //1.25 1.6

	p_pos = kp_pos * def_pos[0]; //P制御
	i_pos += ki_pos * def_pos[0] * DELTA_T; //I制御
	d_pos = kd_pos * (def_pos[0] - def_pos[1]) / DELTA_T; //D制御

	order_posR = p_pos + i_pos + d_pos;
	order_posL = -(p_pos + i_pos + d_pos);

	def_pos[1] = def_pos[0];

}

void velPID(float target) {
	float p_vel, kp_vel = 2.8f, ki_vel = 50.0f;	//2.8 50
	float vel_center, filter_vel_center, acceleration_imu;
	static float i_vel, def_vel, last_vel_center;

	vel_center = (velR + velL) / 2.0f;
	acceleration_imu = (float)xa / 16384.0f;
	filter_vel_center = ComplementaryFilter(acceleration_imu, vel_center, 0.65f, last_vel_center);
	last_vel_center = filter_vel_center;

	def_vel = filter_vel_center - target;

	p_vel = kp_vel * def_vel;
	i_vel += ki_vel * def_vel * DELTA_T;

	order_velR = p_vel + i_vel;
	order_velL = p_vel + i_vel;
}

void Calculation_offset_zg(void){
	offset_zg = sum_zg / calibration_cnt;
}

float Velo_Spline_Curve(float curvature) {
	float velo_spline;

	velo_spline = 0.00000243536328504599f * powf(curvature, 3) + (-0.00768048107979400f) * powf(curvature, 2) + 8.55442953186553f * curvature + 154.785382404022f;

	return velo_spline;
}
