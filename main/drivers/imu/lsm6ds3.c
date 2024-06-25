/*
	Copyright 2020 Mitch Lustig
	Copyright 2023 Benjamin Vedder

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

#include "lsm6ds3.h"
#include "terminal.h"
#include "imu.h"
#include "commands.h"
#include "utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>

static volatile uint16_t lsm6ds3_addr;
static int rate_hz = 1000;
static IMU_FILTER filter;
static volatile bool thd_running = false;
static volatile bool should_terminate = false;

static void terminal_read_reg(int argc, const char **argv);
static uint8_t read_single_reg(uint8_t reg);
static void lsm_task(void *arg);

// Function pointers
static void(*read_callback)(float *accel, float *gyro, float *mag) = 0;


void lsm6ds3_set_rate_hz(int hz) {
	rate_hz = hz;
}

void lsm6ds3_set_filter(IMU_FILTER f) {
	filter = f;
}

void lsm6ds3_init(void) {
	read_callback = 0;

	uint8_t txb[2];
	uint8_t rxb[2];

	txb[0] = LSM6DS3_ACC_GYRO_WHO_AM_I_REG;
	lsm6ds3_addr = LSM6DS3_ACC_GYRO_ADDR_A;

	bool res = imu_i2c_tx_rx(lsm6ds3_addr, txb, 1, rxb, 1);

	if (!res || (rxb[0] != 0x69 && rxb[0] != 0x6A && rxb[0] != 0x6C)) {
		commands_printf("LSM6DS3 Address A failed, trying B (rx: %d)", rxb[0]);
		lsm6ds3_addr = LSM6DS3_ACC_GYRO_ADDR_B;
		res = imu_i2c_tx_rx(lsm6ds3_addr, txb, 1, rxb, 1);
		if (!res || (rxb[0] != 0x69 && rxb[0] != 0x6A && rxb[0] != 0x6C)) {
			commands_printf("LSM6DS3 Address B failed (rx: %d)", rxb[0]);
			return;
		}
	}

	bool is_trc = false;
	if (rxb[0] == 0x6A){
		is_trc = true;
	}

	// TRC variant supports configurable hardware filters
	// oversampling is achieved by configuring higher bandwidth + stronger filtering
	#define LSM6DS3TRC_BW0_XL 0x1
	#define LSM6DS3TRC_LPF1_BW_SEL 0x2

	// Configure imu
	// Set all accel speeds
	txb[0] = LSM6DS3_ACC_GYRO_CTRL1_XL;
	txb[1] = LSM6DS3_ACC_GYRO_BW_XL_400Hz | LSM6DS3_ACC_GYRO_FS_XL_16g;
	if (rate_hz <= 13) {
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
	} else if (rate_hz <= 26){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
	} else if (rate_hz <= 52){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
	} else if (rate_hz <= 104){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
	} else if (rate_hz <= 208){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
	} else if (rate_hz <= 416){
		if (is_trc && (filter >= IMU_FILTER_MEDIUM)) {
			// ODR/4 with 833Hz
			txb[1] |= LSM6DS3TRC_LPF1_BW_SEL | LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
		} else {
			// default: ODR/2 with 416Hz
			txb[1] |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
		}
	} else if (rate_hz <= 833){
		if (is_trc && (filter >= IMU_FILTER_MEDIUM)) {
			// ODR/4 with 1660Hz AND Accelerometer Analog Chain Bandwidth = 400Hz
			txb[1] |= LSM6DS3TRC_BW0_XL | LSM6DS3TRC_LPF1_BW_SEL | LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
		} else {
			// default: ODR/2 with 833Hz
			txb[1] |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
		}
	} else if (rate_hz <= 1660){
		if (is_trc && (filter >= IMU_FILTER_MEDIUM)) {
			// ODR/4 with 3330Hz
			txb[1] |= LSM6DS3TRC_LPF1_BW_SEL | LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
			if (filter == IMU_FILTER_HIGH) {
				// Also enable Accelerometer Analog Chain Bandwidth = 400Hz
				txb[1] |= LSM6DS3TRC_BW0_XL;
			}
		} else {
			txb[1] |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
		}
	} else if (rate_hz <= 3330){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
	} else {
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
	}
	res = imu_i2c_tx_rx(lsm6ds3_addr, txb, 2, rxb, 1);
	if (!res){
		commands_printf("LSM6DS3 Accel Config FAILED");
		return;
	}

	// Set all gyro speeds
	txb[0] = LSM6DS3_ACC_GYRO_CTRL2_G;
	txb[1] = LSM6DS3_ACC_GYRO_FS_G_2000dps;
	if (rate_hz <= 13){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
	} else if (rate_hz <= 26){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
	} else if (rate_hz <= 52){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
	} else if (rate_hz <= 104){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
	} else if (rate_hz <= 208){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
	} else if (rate_hz <= 416){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
	} else if (rate_hz <= 833){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
	} else if (rate_hz <= 1660 || is_trc == false){
		txb[1] |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
	} else if (rate_hz <= 3330){
		txb[1] |= LSM6DS3TRC_ACC_GYRO_ODR_G_3330Hz;
	} else {
		txb[1] |= LSM6DS3TRC_ACC_GYRO_ODR_G_6660Hz;
	}
	res = imu_i2c_tx_rx(lsm6ds3_addr, txb, 2, rxb, 1);
	if (!res){
		commands_printf("LSM6DS3 Gyro Config FAILED");
		return;
	}

	// Filtering
	txb[0] = LSM6DS3_ACC_GYRO_CTRL4_C;
	// TRC Variant CTRL4 register is very different from other variants
	if (is_trc) {
		if (filter >= IMU_FILTER_MEDIUM) {
			// Enable gyroscope digital low-pass filter LPF1
			txb[1] = LSM6DS3_ACC_GYRO_LPF1_SEL_G_ENABLED;
		} else {
			txb[1] = 0;
		}
	} else {
		// Standard LSM6DS3 only: Set XL anti-aliasing filter to be manually configured
		txb[1] = LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
	}
	res = imu_i2c_tx_rx(lsm6ds3_addr, txb, 2, rxb, 1);
	if (!res){
		commands_printf("LSM6DS3 ODR Config FAILED");
		return;
	}

	if (is_trc && (filter == IMU_FILTER_HIGH)) {
		// Low-pass filter with ODR/9 data rate
		#define LSM6DS3TRC_LPF2_XL_EN 0x80
		#define LSM6DS3TRC_HPCF_XL_ODR9 0x40
		txb[0] = LSM6DS3_ACC_GYRO_CTRL8_XL;
		txb[1] = LSM6DS3TRC_LPF2_XL_EN | LSM6DS3TRC_HPCF_XL_ODR9;
		res = imu_i2c_tx_rx(lsm6ds3_addr, txb, 2, rxb, 1);
		if (!res) {
			commands_printf("LSM6DS3 Accel Low Pass Config FAILED");
			return;
		}
	}

	terminal_register_command_callback(
				"lsm_read_reg",
				"Read register of the LSM6DS3",
				"[reg]",
				terminal_read_reg);

	should_terminate = false;
	thd_running = true;
	xTaskCreatePinnedToCore(lsm_task, "LSM6DSx", 1536, NULL, 6, NULL, tskNO_AFFINITY);
}

void lsm6ds3_stop(void) {
	should_terminate = true;

	bool thd_was_running = thd_running;

	while (thd_running) {
		vTaskDelay(1);
	}

	// Put IMU in sleep mode
	if (thd_was_running) {
		uint8_t txb[2];
		uint8_t rxb[2];

		txb[0] = LSM6DS3_ACC_GYRO_CTRL1_XL;
		txb[1] = 0;
		imu_i2c_tx_rx(lsm6ds3_addr, txb, 2, rxb, 1);

		txb[0] = LSM6DS3_ACC_GYRO_CTRL2_G;
		txb[1] = 0;
		imu_i2c_tx_rx(lsm6ds3_addr, txb, 2, rxb, 1);
	}

	terminal_unregister_callback(terminal_read_reg);
}

void lsm6ds3_set_read_callback(void(*func)(float *accel, float *gyro, float *mag)) {
	read_callback = func;
}

static uint8_t read_single_reg(uint8_t reg) {
	uint8_t txb[2];
	uint8_t rxb[2];

	txb[0] = reg;
	bool res = imu_i2c_tx_rx(lsm6ds3_addr, txb, 1, rxb, 2);

	if (res) {
		return rxb[0];
	} else {
		return 0;
	}
}

static void terminal_read_reg(int argc, const char **argv) {
	if (argc == 2) {
		int reg = -1;
		sscanf(argv[1], "%d", &reg);

		if (reg >= 0) {
			unsigned int res = read_single_reg(reg);

			char bl[9];
			utils_byte_to_binary(res & 0xFF, bl);

			commands_printf("Reg 0x%02x: %s (0x%02x)\n", reg, bl, res);
		} else {
			commands_printf("Invalid argument(s).\n");
		}
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

static void lsm_task(void *arg) {
	(void)arg;

	while (!should_terminate) {
		uint8_t txb[2];
		uint8_t rxb[12];

		// Disable IMU writing to output registers
		txb[0] = LSM6DS3_ACC_GYRO_CTRL3_C;
		txb[1] = LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE | LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
		imu_i2c_tx_rx(lsm6ds3_addr, txb, 2, rxb, 1);

		// Read IMU output registers
		txb[0] = LSM6DS3_ACC_GYRO_OUTX_L_G;
		bool res = imu_i2c_tx_rx(lsm6ds3_addr, txb, 1, rxb, 12);

		// Parse 6 axis values
		float gx = (float)((int16_t)((uint16_t)rxb[1] << 8) + rxb[0]) * 4.375 * (2000 / 125) / 1000;
		float gy = (float)((int16_t)((uint16_t)rxb[3] << 8) + rxb[2]) * 4.375 * (2000 / 125) / 1000;
		float gz = (float)((int16_t)((uint16_t)rxb[5] << 8) + rxb[4]) * 4.375 * (2000 / 125) / 1000;
		float ax = (float)((int16_t)((uint16_t)rxb[7] << 8) + rxb[6]) * 0.061 * (16 >> 1) / 1000;
		float ay = (float)((int16_t)((uint16_t)rxb[9] << 8) + rxb[8]) * 0.061 * (16 >> 1) / 1000;
		float az = (float)((int16_t)((uint16_t)rxb[11] << 8) + rxb[10]) * 0.061 * (16 >> 1) / 1000;

		if (res && read_callback) {
			float tmp_accel[3] = {ax,ay,az}, tmp_gyro[3] = {gx,gy,gz}, tmp_mag[3] = {1,2,3};
			read_callback(tmp_accel, tmp_gyro, tmp_mag);
		}

		vTaskDelay(portTICK_PERIOD_MS / rate_hz);
	}

	thd_running = false;
	vTaskDelete(NULL);
}

