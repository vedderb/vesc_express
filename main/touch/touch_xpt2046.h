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

#ifndef MAIN_TOUCH_TOUCH_XPT2046_H_
#define MAIN_TOUCH_TOUCH_XPT2046_H_

#include <stdbool.h>
#include <stdint.h>

#include "lispif_touch_extensions.h"

typedef struct {
	int pin_mosi;
	int pin_miso;
	int pin_clk;
	int pin_cs;
	int pin_int;
	uint16_t width;
	uint16_t height;
	uint16_t z_threshold;
	bool interrupt_level;
} touch_xpt2046_config_t;

esp_err_t touch_xpt2046_init(const touch_xpt2046_config_t *config, lispif_touch_driver_t *driver);

#endif