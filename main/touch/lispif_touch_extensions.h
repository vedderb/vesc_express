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

#ifndef LISPIF_TOUCH_EXTENSIONS_H_
#define LISPIF_TOUCH_EXTENSIONS_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint8_t track_id;
    uint16_t x;
    uint16_t y;
    uint16_t strength;
} lispif_touch_point_data_t;

typedef struct {
    esp_err_t (*deinit)(void);
    esp_err_t (*read_data)(void);
    esp_err_t (*get_data)(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt);
} lispif_touch_driver_t;

void lispif_touch_irq_from_isr(void);
void lispif_touch_shutdown(void);
void lispif_load_touch_extensions(void);

#endif