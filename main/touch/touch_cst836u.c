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

#include "touch_cst836u.h"

#include <string.h>

#include "freertos/FreeRTOS.h"

#define TOUCH_CST836U_I2C_ADDR 0x15
#define TOUCH_CST836U_READ_REG 0x02
#define TOUCH_CST836U_READ_LEN 5

static i2c_port_t cst836u_i2c_port = I2C_NUM_0;
static uint16_t cst836u_width = 0;
static uint16_t cst836u_height = 0;
static uint16_t cst836u_raw_width = 0;
static uint16_t cst836u_raw_height = 0;
static bool cst836u_swap_xy = false;
static bool cst836u_mirror_x = false;
static bool cst836u_mirror_y = false;

static bool cst836u_has_point = false;
static lispif_touch_point_data_t cst836u_point = {0};
static uint16_t cst836u_hist_x[3] = {0};
static uint16_t cst836u_hist_y[3] = {0};
static uint8_t cst836u_hist_cnt = 0;

static void cst836u_filter_reset(void) {
	cst836u_hist_cnt = 0;
	memset(cst836u_hist_x, 0, sizeof(cst836u_hist_x));
	memset(cst836u_hist_y, 0, sizeof(cst836u_hist_y));
}

static uint16_t cst836u_median3(uint16_t a, uint16_t b, uint16_t c) {
	if (a > b) {
		uint16_t t = a;
		a = b;
		b = t;
	}
	if (b > c) {
		uint16_t t = b;
		b = c;
		c = t;
	}
	if (a > b) {
		uint16_t t = a;
		a = b;
		b = t;
	}
	return b;
}

static void cst836u_filter_push(uint16_t x, uint16_t y) {
	if (cst836u_hist_cnt < 3) {
		cst836u_hist_x[cst836u_hist_cnt] = x;
		cst836u_hist_y[cst836u_hist_cnt] = y;
		cst836u_hist_cnt++;
		return;
	}

	cst836u_hist_x[0] = cst836u_hist_x[1];
	cst836u_hist_x[1] = cst836u_hist_x[2];
	cst836u_hist_x[2] = x;

	cst836u_hist_y[0] = cst836u_hist_y[1];
	cst836u_hist_y[1] = cst836u_hist_y[2];
	cst836u_hist_y[2] = y;
}

static void cst836u_filter_apply(lispif_touch_point_data_t *point) {
	if (!point || cst836u_hist_cnt == 0) {
		return;
	}

	if (cst836u_hist_cnt == 1) {
		point->x = cst836u_hist_x[0];
		point->y = cst836u_hist_y[0];
		return;
	}

	if (cst836u_hist_cnt == 2) {
		point->x = (uint16_t)(((uint32_t)cst836u_hist_x[0] + (uint32_t)cst836u_hist_x[1]) / 2U);
		point->y = (uint16_t)(((uint32_t)cst836u_hist_y[0] + (uint32_t)cst836u_hist_y[1]) / 2U);
		return;
	}

	point->x = cst836u_median3(cst836u_hist_x[0], cst836u_hist_x[1], cst836u_hist_x[2]);
	point->y = cst836u_median3(cst836u_hist_y[0], cst836u_hist_y[1], cst836u_hist_y[2]);
}

static uint16_t cst836u_scale_axis(uint16_t value, uint16_t in_max, uint16_t out_max) {
	if (in_max <= 1 || out_max <= 1) {
		return 0;
	}

	if (value >= in_max) {
		value = in_max - 1U;
	}

	uint32_t num = (uint32_t)value * (uint32_t)(out_max - 1U);
	return (uint16_t)(num / (uint32_t)(in_max - 1U));
}

static void cst836u_set_default_raw_geometry(uint16_t width, uint16_t height) {
	if ((width == 320 && height == 240) || (width == 240 && height == 320)) {
		cst836u_raw_width = 240;
		cst836u_raw_height = 320;
		return;
	}

	cst836u_raw_width = width;
	cst836u_raw_height = height;
}

static void cst836u_apply_transforms(lispif_touch_point_data_t *point) {
	if (!point || cst836u_width == 0 || cst836u_height == 0 || cst836u_raw_width == 0 || cst836u_raw_height == 0) {
		return;
	}

	uint16_t x = point->x;
	uint16_t y = point->y;
	uint16_t max_x = cst836u_raw_width;
	uint16_t max_y = cst836u_raw_height;

	if (x >= max_x && max_x > 0) {
		x = max_x - 1U;
	}
	if (y >= max_y && max_y > 0) {
		y = max_y - 1U;
	}

	if (cst836u_swap_xy) {
		uint16_t tmp = x;
		x = y;
		y = tmp;

		tmp = max_x;
		max_x = max_y;
		max_y = tmp;
	}

	if (cst836u_mirror_x && max_x > 0) {
		x = x < max_x ? (uint16_t)((max_x - 1U) - x) : 0;
	}

	if (cst836u_mirror_y && max_y > 0) {
		y = y < max_y ? (uint16_t)((max_y - 1U) - y) : 0;
	}

	if (max_x > 0 && x >= max_x) {
		x = max_x - 1U;
	}
	if (max_y > 0 && y >= max_y) {
		y = max_y - 1U;
	}

	if (cst836u_width > 1 && max_x > 1 && cst836u_width != max_x) {
		x = cst836u_scale_axis(x, max_x, cst836u_width);
	}
	if (cst836u_height > 1 && max_y > 1 && cst836u_height != max_y) {
		y = cst836u_scale_axis(y, max_y, cst836u_height);
	}

	if (x >= cst836u_width && cst836u_width > 0) {
		x = cst836u_width - 1U;
	}
	if (y >= cst836u_height && cst836u_height > 0) {
		y = cst836u_height - 1U;
	}

	point->x = x;
	point->y = y;
}

esp_err_t touch_cst836u_init(i2c_port_t port, uint16_t width, uint16_t height, lispif_touch_driver_t *driver) {
	if (!driver) {
		return ESP_ERR_INVALID_ARG;
	}

	cst836u_i2c_port = port;
	cst836u_width = width;
	cst836u_height = height;
	cst836u_set_default_raw_geometry(width, height);
	cst836u_swap_xy = false;
	cst836u_mirror_x = false;
	cst836u_mirror_y = false;
	cst836u_has_point = false;
	cst836u_filter_reset();
	memset(&cst836u_point, 0, sizeof(cst836u_point));

	driver->read_data = touch_cst836u_read_data;
	driver->get_data = touch_cst836u_get_data;

	return ESP_OK;
}

void touch_cst836u_set_transforms(bool swap_xy, bool mirror_x, bool mirror_y) {
	cst836u_swap_xy = swap_xy;
	cst836u_mirror_x = mirror_x;
	cst836u_mirror_y = mirror_y;
}

void touch_cst836u_reset(void) {
	cst836u_width = 0;
	cst836u_height = 0;
	cst836u_raw_width = 0;
	cst836u_raw_height = 0;
	cst836u_swap_xy = false;
	cst836u_mirror_x = false;
	cst836u_mirror_y = false;
	cst836u_has_point = false;
	cst836u_filter_reset();
	memset(&cst836u_point, 0, sizeof(cst836u_point));
}

esp_err_t touch_cst836u_read_data(void) {
	uint8_t reg = TOUCH_CST836U_READ_REG;
	uint8_t raw[TOUCH_CST836U_READ_LEN] = {0};

	esp_err_t res = i2c_master_write_read_device(
			cst836u_i2c_port,
			TOUCH_CST836U_I2C_ADDR,
			&reg,
			1,
			raw,
			sizeof(raw),
			pdMS_TO_TICKS(20));
	if (res != ESP_OK) {
		cst836u_has_point = false;
		return res;
	}

	uint8_t point_cnt = raw[0] & 0x0F;
	if (point_cnt == 0) {
		cst836u_has_point = false;
		cst836u_filter_reset();
		return ESP_OK;
	}

	cst836u_point.x = (uint16_t)(((raw[1] & 0x0F) << 8) | raw[2]);
	cst836u_point.y = (uint16_t)(((raw[3] & 0x0F) << 8) | raw[4]);
	cst836u_point.track_id = (raw[3] >> 4) & 0x0F;
	cst836u_point.strength = 1;
	cst836u_has_point = true;

	return ESP_OK;
}

esp_err_t touch_cst836u_get_data(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt) {
	if (!data || !point_cnt || max_point_cnt == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	if (!cst836u_has_point) {
		*point_cnt = 0;
		return ESP_OK;
	}

	data[0] = cst836u_point;
	cst836u_apply_transforms(&data[0]);
	cst836u_filter_push(data[0].x, data[0].y);
	cst836u_filter_apply(&data[0]);
	*point_cnt = 1;
	return ESP_OK;
}
