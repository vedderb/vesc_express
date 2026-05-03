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

#ifndef TOUCH_CST836U_H_
#define TOUCH_CST836U_H_

#include <stdbool.h>
#include "driver/i2c.h"
#include "lispif_touch_extensions.h"

esp_err_t touch_cst836u_init(i2c_port_t port, uint16_t width, uint16_t height, lispif_touch_driver_t *driver);
void touch_cst836u_set_transforms(bool swap_xy, bool mirror_x, bool mirror_y);
void touch_cst836u_reset(void);

esp_err_t touch_cst836u_read_data(void);
esp_err_t touch_cst836u_get_data(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt);

#endif
