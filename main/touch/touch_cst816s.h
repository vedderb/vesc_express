/*
	Copyright 2025

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

#ifndef MAIN_TOUCH_TOUCH_CST816S_H_
#define MAIN_TOUCH_TOUCH_CST816S_H_

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "lispif_touch_extensions.h"

#define TOUCH_CST816S_I2C_ADDR 0x15

typedef struct {
	int pin_sda;
	int pin_scl;
	int pin_rst;
	int pin_int;
	i2c_port_t i2c_port;
	uint8_t i2c_addr;
	uint32_t i2c_freq;
	uint32_t timeout_ms;
	bool reset_level;
	bool interrupt_level;
	bool read_id;
} touch_cst816s_config_t;

esp_err_t touch_cst816s_init(const touch_cst816s_config_t *config, lispif_touch_driver_t *driver);

#endif