/*
	Copyright 2019 - 2023 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include "driver/i2c.h"

typedef enum {
	IMU_TYPE_OFF = 0,
	IMU_TYPE_INTERNAL,
	IMU_TYPE_EXTERNAL_MPU9X50,
	IMU_TYPE_EXTERNAL_ICM20948,
	IMU_TYPE_EXTERNAL_BMI160,
	IMU_TYPE_EXTERNAL_LSM6DS3
} IMU_TYPE;

typedef enum {
	AHRS_MODE_MADGWICK = 0,
	AHRS_MODE_MAHONY,
	AHRS_MODE_MADGWICK_FUSION
} AHRS_MODE;

typedef enum {
	IMU_FILTER_LOW = 0,
	IMU_FILTER_MEDIUM,
	IMU_FILTER_HIGH
} IMU_FILTER;

typedef struct {
	IMU_TYPE type;
	AHRS_MODE mode;
	IMU_FILTER filter;
	float accel_lowpass_filter_x;
	float accel_lowpass_filter_y;
	float accel_lowpass_filter_z;
	float gyro_lowpass_filter;
	int sample_rate_hz;
	bool use_magnetometer;
	float accel_confidence_decay;
	float mahony_kp;
	float mahony_ki;
	float madgwick_beta;
	float rot_roll;
	float rot_pitch;
	float rot_yaw;
	float accel_offsets[3];
	float gyro_offsets[3];
} imu_config;

void imu_init(imu_config *set, SemaphoreHandle_t i2c_mutex);
bool imu_i2c_tx_rx(uint8_t addr,
		const uint8_t* write_buffer, size_t write_size,
		uint8_t* read_buffer, size_t read_size);
void imu_reset_orientation(void);
void imu_init_lsm6ds3(void);
void imu_stop(void);
bool imu_startup_done(void);
float imu_get_roll(void);
float imu_get_pitch(void);
float imu_get_yaw(void);
void imu_get_rpy(float *rpy);
void imu_get_accel(float *accel);
void imu_get_gyro(float *gyro);
void imu_get_mag(float *mag);
void imu_derotate(float *input, float *output);
void imu_get_accel_derotated(float *accel);
void imu_get_gyro_derotated(float *gyro);
void imu_get_quaternions(float *q);
void imu_get_calibration(float yaw, float * imu_cal);
void imu_set_read_callback(void (*func)(float *acc, float *gyro, float *mag, float dt));

#endif /* IMU_IMU_H_ */
