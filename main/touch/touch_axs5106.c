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

#include "touch_axs5106.h"
#include <string.h>

static i2c_port_num_t i2c_port;
static uint16_t screen_width, screen_height;
static bool swap_xy, mirror_x, mirror_y;
static lispif_touch_point_data_t points[2];
static uint8_t point_count;

static esp_err_t read_data(void);
static esp_err_t get_data(lispif_touch_point_data_t *data, uint8_t *count, uint8_t max_count);

esp_err_t touch_axs5106_init(i2c_port_num_t port, uint16_t width, uint16_t height, lispif_touch_driver_t *driver) {
	if (!driver || !width || !height) {
		return ESP_ERR_INVALID_ARG;
	}
	i2c_port = port;
	screen_width = width;
	screen_height = height;
	swap_xy = false;
	mirror_x = false;
	mirror_y = false;
	point_count = 0;
	memset(points, 0, sizeof(points));
	esp_err_t res = hwi2c_tx_rx(port, 0x63, NULL, 0, NULL, 0, 100);
	if (res == ESP_OK) {
		driver->read_data = read_data;
		driver->get_data = get_data;
	}
	return res;
}

void touch_axs5106_set_transforms(bool swap, bool mx, bool my) {
	swap_xy = swap;
	mirror_x = mx;
	mirror_y = my;
}

static esp_err_t read_data(void) {
	uint8_t reg = 0x01, data[14];
	esp_err_t res = hwi2c_tx_rx(i2c_port, 0x63, &reg, 1, NULL, 0, 20);
	if (res == ESP_OK) {
		res = hwi2c_tx_rx(i2c_port, 0x63, NULL, 0, data, sizeof(data), 20);
	}
	point_count = 0;
	if (res != ESP_OK) {
		return res;
	}
	uint8_t count = data[1] & 0x0F;
	if (count > 2) {
		count = 2;
	}
	for (int i = 0; i < count; i++) {
		int offset = i * 6;
		uint16_t x = ((data[2 + offset] & 0x0F) << 8) | data[3 + offset];
		uint16_t y = ((data[4 + offset] & 0x0F) << 8) | data[5 + offset];
		if (swap_xy) {
			uint16_t tmp = x;
			x = y;
			y = tmp;
		}
		if (x >= screen_width || y >= screen_height) {
			continue;
		}
		if (mirror_x) {
			x = screen_width - 1 - x;
		}
		if (mirror_y) {
			y = screen_height - 1 - y;
		}
		points[point_count++] = (lispif_touch_point_data_t){data[4 + offset] >> 4, x, y, 1};
	}
	return ESP_OK;
}

static esp_err_t get_data(lispif_touch_point_data_t *data, uint8_t *count, uint8_t max_count) {
	if (!data || !count || !max_count) {
		return ESP_ERR_INVALID_ARG;
	}
	*count = point_count < max_count ? point_count : max_count;
	memcpy(data, points, *count * sizeof(*data));
	return ESP_OK;
}
