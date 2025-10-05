//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef AHRS_h
#define AHRS_h

typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;
	float integralFBx;
	float integralFBy;
	float integralFBz;
	float accMagP;
	int initialUpdateDone;

	// Parameters
	float acc_confidence_decay;
	float kp;
	float ki;
	float beta;
} ATTITUDE_INFO;

// Function declarations
void ahrs_init_attitude_info(ATTITUDE_INFO *att);
void ahrs_update_all_parameters(ATTITUDE_INFO *att, float confidence_decay, float kp, float ki, float beta);
void ahrs_update_initial_orientation(const float *accelXYZ, const float *magXYZ, ATTITUDE_INFO *att);

void ahrs_update_mahony_imu(const float *gyroXYZ, const float *accelXYZ, float dt, ATTITUDE_INFO *att);
void ahrs_update_madgwick_imu(const float *gyroXYZ, const float *accelXYZ, float dt, ATTITUDE_INFO *att);

float ahrs_get_roll(const ATTITUDE_INFO *att);
float ahrs_get_pitch(const ATTITUDE_INFO *att);
float ahrs_get_yaw(const ATTITUDE_INFO *att);
void ahrs_get_roll_pitch_yaw(float *rpy, ATTITUDE_INFO *att);

#endif
