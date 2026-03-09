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

#include "touch_xpt2046.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "spi_bb.h"

#define TOUCH_XPT2046_ADC_LIMIT 4095U
#define TOUCH_XPT2046_SAMPLE_COUNT 5

typedef struct {
	touch_xpt2046_config_t config;
	spi_bb_state spi;
	lispif_touch_point_data_t point;
	uint8_t point_cnt;
	bool initialized;
} touch_xpt2046_state_t;

static touch_xpt2046_state_t touch_xpt2046_state = {0};
static const char *TAG = "touch_xpt2046";

static esp_err_t touch_xpt2046_deinit(void);
static esp_err_t touch_xpt2046_read_data(void);
static esp_err_t touch_xpt2046_get_data(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt);
static esp_err_t touch_xpt2046_read_reg(uint8_t reg, uint16_t *value);
static bool touch_xpt2046_irq_active(void);
static uint8_t touch_xpt2046_make_cmd(uint8_t base);
static void touch_xpt2046_isr(void *arg);

enum {
	TOUCH_XPT2046_REG_Z1 = 0xB0,
	TOUCH_XPT2046_REG_Z2 = 0xC0,
	TOUCH_XPT2046_REG_Y = 0x90,
	TOUCH_XPT2046_REG_X = 0xD0,
};

esp_err_t touch_xpt2046_init(const touch_xpt2046_config_t *config, lispif_touch_driver_t *driver) {
	ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is null");
	ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");
	ESP_RETURN_ON_FALSE(config->pin_mosi >= 0, ESP_ERR_INVALID_ARG, TAG, "invalid MOSI pin");
	ESP_RETURN_ON_FALSE(config->pin_miso >= 0, ESP_ERR_INVALID_ARG, TAG, "invalid MISO pin");
	ESP_RETURN_ON_FALSE(config->pin_clk >= 0, ESP_ERR_INVALID_ARG, TAG, "invalid CLK pin");
	ESP_RETURN_ON_FALSE(config->pin_cs >= 0, ESP_ERR_INVALID_ARG, TAG, "invalid CS pin");
	ESP_RETURN_ON_FALSE(config->width > 0, ESP_ERR_INVALID_ARG, TAG, "invalid width");
	ESP_RETURN_ON_FALSE(config->height > 0, ESP_ERR_INVALID_ARG, TAG, "invalid height");
	ESP_RETURN_ON_FALSE(config->z_threshold > 0, ESP_ERR_INVALID_ARG, TAG, "invalid threshold");

	if (touch_xpt2046_state.initialized) {
		touch_xpt2046_deinit();
	}

	memset(&touch_xpt2046_state, 0, sizeof(touch_xpt2046_state));
	touch_xpt2046_state.config = *config;
	touch_xpt2046_state.spi.mosi_pin = config->pin_mosi;
	touch_xpt2046_state.spi.miso_pin = config->pin_miso;
	touch_xpt2046_state.spi.sck_pin = config->pin_clk;
	touch_xpt2046_state.spi.nss_pin = config->pin_cs;
	spi_bb_init(&touch_xpt2046_state.spi);

	if (config->pin_int >= 0) {
		gpio_config_t int_conf = {
				.mode = GPIO_MODE_INPUT,
				.pull_up_en = GPIO_PULLUP_ENABLE,
				.intr_type = config->interrupt_level ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE,
				.pin_bit_mask = BIT64(config->pin_int)
		};
		ESP_RETURN_ON_ERROR(gpio_config(&int_conf), TAG, "interrupt pin config failed");

		esp_err_t res = gpio_install_isr_service(0);
		if (res != ESP_OK && res != ESP_ERR_INVALID_STATE) {
			ESP_RETURN_ON_ERROR(res, TAG, "gpio_install_isr_service failed");
		}
		ESP_RETURN_ON_ERROR(gpio_intr_enable(config->pin_int), TAG, "gpio_intr_enable failed");
		ESP_RETURN_ON_ERROR(gpio_isr_handler_add(config->pin_int, touch_xpt2046_isr, 0), TAG, "gpio_isr_handler_add failed");
	}

	touch_xpt2046_state.initialized = true;
	driver->deinit = touch_xpt2046_deinit;
	driver->read_data = touch_xpt2046_read_data;
	driver->get_data = touch_xpt2046_get_data;
	return ESP_OK;
}

static esp_err_t touch_xpt2046_deinit(void) {
	if (touch_xpt2046_state.config.pin_int >= 0) {
		gpio_isr_handler_remove(touch_xpt2046_state.config.pin_int);
		gpio_intr_disable(touch_xpt2046_state.config.pin_int);
		gpio_reset_pin(touch_xpt2046_state.config.pin_int);
	}

	spi_bb_deinit(&touch_xpt2046_state.spi);
	memset(&touch_xpt2046_state, 0, sizeof(touch_xpt2046_state));
	return ESP_OK;
}

static esp_err_t touch_xpt2046_read_data(void) {
	ESP_RETURN_ON_FALSE(touch_xpt2046_state.initialized, ESP_ERR_INVALID_STATE, TAG, "driver not initialized");

	if (touch_xpt2046_state.config.pin_int >= 0 && !touch_xpt2046_irq_active()) {
		touch_xpt2046_state.point_cnt = 0;
		memset(&touch_xpt2046_state.point, 0, sizeof(touch_xpt2046_state.point));
		return ESP_OK;
	}

	uint16_t z1 = 0;
	uint16_t z2 = 0;
	ESP_RETURN_ON_ERROR(touch_xpt2046_read_reg(TOUCH_XPT2046_REG_Z1, &z1), TAG, "z1 read failed");
	ESP_RETURN_ON_ERROR(touch_xpt2046_read_reg(TOUCH_XPT2046_REG_Z2, &z2), TAG, "z2 read failed");

	uint16_t z = (uint16_t)((z1 >> 3) + ((TOUCH_XPT2046_ADC_LIMIT + 1U) - (z2 >> 3)));
	if (z < touch_xpt2046_state.config.z_threshold) {
		touch_xpt2046_state.point_cnt = 0;
		memset(&touch_xpt2046_state.point, 0, sizeof(touch_xpt2046_state.point));
		return ESP_OK;
	}

	uint16_t discard = 0;
	ESP_RETURN_ON_ERROR(touch_xpt2046_read_reg(TOUCH_XPT2046_REG_X, &discard), TAG, "discard read failed");

	uint32_t x_acc = 0;
	uint32_t y_acc = 0;
	uint8_t valid = 0;

	for (uint8_t i = 0;i < TOUCH_XPT2046_SAMPLE_COUNT;i++) {
		uint16_t x = 0;
		uint16_t y = 0;
		ESP_RETURN_ON_ERROR(touch_xpt2046_read_reg(TOUCH_XPT2046_REG_X, &x), TAG, "x read failed");
		ESP_RETURN_ON_ERROR(touch_xpt2046_read_reg(TOUCH_XPT2046_REG_Y, &y), TAG, "y read failed");
		x >>= 3;
		y >>= 3;

		if (x >= 50 && x <= (TOUCH_XPT2046_ADC_LIMIT - 50) && y >= 50 && y <= (TOUCH_XPT2046_ADC_LIMIT - 50)) {
			x_acc += x;
			y_acc += y;
			valid++;
		}
	}

	if (valid < ((TOUCH_XPT2046_SAMPLE_COUNT == 1) ? 1 : (TOUCH_XPT2046_SAMPLE_COUNT / 2))) {
		touch_xpt2046_state.point_cnt = 0;
		memset(&touch_xpt2046_state.point, 0, sizeof(touch_xpt2046_state.point));
		return ESP_OK;
	}

	uint32_t x_avg = x_acc / valid;
	uint32_t y_avg = y_acc / valid;
	touch_xpt2046_state.point.x = (uint16_t)((x_avg * (touch_xpt2046_state.config.width - 1U)) / TOUCH_XPT2046_ADC_LIMIT);
	touch_xpt2046_state.point.y = (uint16_t)((y_avg * (touch_xpt2046_state.config.height - 1U)) / TOUCH_XPT2046_ADC_LIMIT);
	touch_xpt2046_state.point.strength = z;
	touch_xpt2046_state.point.track_id = 0;
	touch_xpt2046_state.point_cnt = 1;

	return ESP_OK;
}

static esp_err_t touch_xpt2046_get_data(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt) {
	ESP_RETURN_ON_FALSE(touch_xpt2046_state.initialized, ESP_ERR_INVALID_STATE, TAG, "driver not initialized");
	ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "data is null");
	ESP_RETURN_ON_FALSE(point_cnt != NULL, ESP_ERR_INVALID_ARG, TAG, "point count is null");
	ESP_RETURN_ON_FALSE(max_point_cnt > 0, ESP_ERR_INVALID_ARG, TAG, "max_point_cnt must be > 0");

	*point_cnt = touch_xpt2046_state.point_cnt > max_point_cnt ? max_point_cnt : touch_xpt2046_state.point_cnt;
	if (*point_cnt > 0) {
		data[0] = touch_xpt2046_state.point;
	}

	touch_xpt2046_state.point_cnt = 0;
	return ESP_OK;
}

static esp_err_t touch_xpt2046_read_reg(uint8_t reg, uint16_t *value) {
	ESP_RETURN_ON_FALSE(value != NULL, ESP_ERR_INVALID_ARG, TAG, "value is null");

	uint8_t rx0 = 0;
	uint8_t rx1 = 0;
	spi_bb_begin(&touch_xpt2046_state.spi);
	(void)spi_bb_exchange_8(&touch_xpt2046_state.spi, touch_xpt2046_make_cmd(reg));
	rx0 = spi_bb_exchange_8(&touch_xpt2046_state.spi, 0);
	rx1 = spi_bb_exchange_8(&touch_xpt2046_state.spi, 0);
	spi_bb_end(&touch_xpt2046_state.spi);

	*value = ((uint16_t)rx0 << 8) | rx1;
	return ESP_OK;
}

static bool touch_xpt2046_irq_active(void) {
	return gpio_get_level(touch_xpt2046_state.config.pin_int) == (int)touch_xpt2046_state.config.interrupt_level;
}

static uint8_t touch_xpt2046_make_cmd(uint8_t base) {
	uint8_t pd0 = touch_xpt2046_state.config.pin_int >= 0 ? 0x00 : 0x01;
	uint8_t pd1 = 0x00;
	return base | pd1 | pd0;
}

static void IRAM_ATTR touch_xpt2046_isr(void *arg) {
	(void)arg;
	lispif_touch_irq_from_isr();
}