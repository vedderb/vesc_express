/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Copyright 2022 Jakub Tomczak

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

#ifndef ENC_AS504X_H_
#define ENC_AS504X_H_

#include "stdint.h"
#include "stdbool.h"
#include "spi_bb.h"

typedef struct {
	uint8_t is_connected;
	uint8_t AGC_value;
	uint16_t magnitude;
	uint8_t is_OCF;
	uint8_t is_COF;
	uint8_t is_Comp_low;
	uint8_t is_Comp_high;
	uint16_t serial_diag_flgs;
	uint16_t serial_magnitude;
	uint16_t serial_error_flags;
} AS504x_diag;

typedef struct {
	uint16_t diag_fetch_now_count;
	uint32_t data_last_invalid_counter;
	uint32_t spi_communication_error_count;
	uint8_t spi_data_err_raised;
	AS504x_diag sensor_diag;
	uint16_t spi_val;
	float last_enc_angle;
	uint32_t spi_error_cnt;
	float spi_error_rate;
	uint32_t last_update_time;
} AS504x_state;

typedef struct {
	spi_bb_state sw_spi;
	AS504x_state state;
} AS504x_config_t;

// Functions
bool enc_as504x_init(AS504x_config_t *AS504x_config);
void enc_as504x_deinit(AS504x_config_t *cfg);
void enc_as504x_routine(AS504x_config_t *cfg);
float enc_as504x_read_angle(AS504x_config_t *cfg);

// Macros
#define AS504x_LAST_ANGLE(cfg)		((cfg)->state.last_enc_angle)
#define AS504x_IS_CONNECTED(cfg)	((cfg)->state.sensor_diag.is_connected)
#define AS504x_IS_COMP_HIGH(cfg)	((cfg)->state.sensor_diag.is_Comp_high)
#define AS504x_IS_COMP_LOW(cfg)		((cfg)->state.sensor_diag.is_Comp_low)


#endif /* ENC_AS504X_H_ */
