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

#include "touch_cst816s.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"

#define TOUCH_CST816S_POINT_NUM_MAX 1
#define TOUCH_CST816S_DATA_START_REG 0x02
#define TOUCH_CST816S_CHIP_ID_REG 0xA7

typedef struct {
	touch_cst816s_config_t config;
	lispif_touch_point_data_t points[TOUCH_CST816S_POINT_NUM_MAX];
	uint8_t point_cnt;
	bool owns_i2c_driver;
	bool initialized;
} touch_cst816s_state_t;

static touch_cst816s_state_t touch_cst816s_state = {0};
static const char *TAG = "touch_cst816s";

static esp_err_t touch_cst816s_deinit(void);
static esp_err_t touch_cst816s_read_data(void);
static esp_err_t touch_cst816s_get_data(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt);
static esp_err_t touch_cst816s_reset(void);
static esp_err_t touch_cst816s_read_id(void);
static esp_err_t touch_cst816s_read_reg(uint8_t reg, uint8_t *data, uint8_t len);
static void touch_cst816s_isr(void *arg);

esp_err_t touch_cst816s_init(const touch_cst816s_config_t *config, lispif_touch_driver_t *driver) {
	ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is null");
	ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");
	ESP_RETURN_ON_FALSE(config->pin_sda >= 0, ESP_ERR_INVALID_ARG, TAG, "invalid SDA pin");
	ESP_RETURN_ON_FALSE(config->pin_scl >= 0, ESP_ERR_INVALID_ARG, TAG, "invalid SCL pin");
	ESP_RETURN_ON_FALSE(config->i2c_freq > 0, ESP_ERR_INVALID_ARG, TAG, "invalid I2C frequency");
	ESP_RETURN_ON_FALSE(config->timeout_ms > 0, ESP_ERR_INVALID_ARG, TAG, "invalid timeout");

	if (touch_cst816s_state.initialized) {
		touch_cst816s_deinit();
	}

	memset(&touch_cst816s_state, 0, sizeof(touch_cst816s_state));
	touch_cst816s_state.config = *config;

	i2c_config_t conf = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = config->pin_sda,
			.scl_io_num = config->pin_scl,
			.sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE,
			.master.clk_speed = config->i2c_freq,
	};

	bool reuse_existing_i2c = false;
	esp_err_t cfg_res = i2c_param_config(config->i2c_port, &conf);
	if (cfg_res != ESP_OK) {
		if (cfg_res != ESP_FAIL && cfg_res != ESP_ERR_INVALID_STATE) {
			ESP_RETURN_ON_ERROR(cfg_res, TAG, "i2c_param_config failed");
		}

		reuse_existing_i2c = true;
		ESP_LOGW(TAG, "Reusing existing I2C bus config: %s (%d)", esp_err_to_name(cfg_res), cfg_res);
	}

	esp_err_t res = i2c_driver_install(config->i2c_port, conf.mode, 0, 0, 0);
	if (res == ESP_ERR_INVALID_STATE) {
		reuse_existing_i2c = true;
	} else if (res != ESP_OK) {
		ESP_RETURN_ON_ERROR(res, TAG, "i2c_driver_install failed");
	}
	touch_cst816s_state.owns_i2c_driver = !reuse_existing_i2c;

	if (config->pin_int >= 0) {
		gpio_config_t int_conf = {
				.mode = GPIO_MODE_INPUT,
				.intr_type = config->interrupt_level ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE,
				.pin_bit_mask = BIT64(config->pin_int)
		};
		ESP_RETURN_ON_ERROR(gpio_config(&int_conf), TAG, "interrupt pin config failed");
	}

	if (config->pin_rst >= 0) {
		gpio_config_t rst_conf = {
				.mode = GPIO_MODE_OUTPUT,
				.pin_bit_mask = BIT64(config->pin_rst)
		};
		ESP_RETURN_ON_ERROR(gpio_config(&rst_conf), TAG, "reset pin config failed");
	}

	ESP_RETURN_ON_ERROR(touch_cst816s_reset(), TAG, "touch reset failed");

	if (config->read_id) {
		res = touch_cst816s_read_id();
		if (res != ESP_OK) {
			ESP_LOGW(TAG, "Read ID failed: %s (%d), continuing", esp_err_to_name(res), res);
		}
	}

	if (config->pin_int >= 0) {
		res = gpio_install_isr_service(0);
		if (res != ESP_OK && res != ESP_ERR_INVALID_STATE) {
			ESP_RETURN_ON_ERROR(res, TAG, "gpio_install_isr_service failed");
		}
		ESP_RETURN_ON_ERROR(gpio_intr_enable(config->pin_int), TAG, "gpio_intr_enable failed");
		ESP_RETURN_ON_ERROR(gpio_isr_handler_add(config->pin_int, touch_cst816s_isr, 0), TAG, "gpio_isr_handler_add failed");
	}

	touch_cst816s_state.initialized = true;
	driver->deinit = touch_cst816s_deinit;
	driver->read_data = touch_cst816s_read_data;
	driver->get_data = touch_cst816s_get_data;

	return ESP_OK;
}

static esp_err_t touch_cst816s_deinit(void) {
	if (touch_cst816s_state.config.pin_int >= 0) {
		gpio_isr_handler_remove(touch_cst816s_state.config.pin_int);
		gpio_intr_disable(touch_cst816s_state.config.pin_int);
		gpio_reset_pin(touch_cst816s_state.config.pin_int);
	}

	if (touch_cst816s_state.config.pin_rst >= 0) {
		gpio_reset_pin(touch_cst816s_state.config.pin_rst);
	}

	if (touch_cst816s_state.owns_i2c_driver) {
		i2c_driver_delete(touch_cst816s_state.config.i2c_port);
	}

	memset(&touch_cst816s_state, 0, sizeof(touch_cst816s_state));
	return ESP_OK;
}

static esp_err_t touch_cst816s_read_data(void) {
	typedef struct {
		uint8_t num;
		uint8_t x_h : 4;
		uint8_t : 4;
		uint8_t x_l;
		uint8_t y_h : 4;
		uint8_t : 4;
		uint8_t y_l;
	} touch_cst816s_packet_t;

	ESP_RETURN_ON_FALSE(touch_cst816s_state.initialized, ESP_ERR_INVALID_STATE, TAG, "driver not initialized");

	touch_cst816s_packet_t packet = {0};
	ESP_RETURN_ON_ERROR(touch_cst816s_read_reg(TOUCH_CST816S_DATA_START_REG, (uint8_t*)&packet, sizeof(packet)), TAG, "touch read failed");

	touch_cst816s_state.point_cnt = packet.num > TOUCH_CST816S_POINT_NUM_MAX ? TOUCH_CST816S_POINT_NUM_MAX : packet.num;
	for (uint8_t i = 0;i < touch_cst816s_state.point_cnt;i++) {
		touch_cst816s_state.points[i].x = ((uint16_t)packet.x_h << 8) | packet.x_l;
		touch_cst816s_state.points[i].y = ((uint16_t)packet.y_h << 8) | packet.y_l;
		touch_cst816s_state.points[i].strength = 0;
		touch_cst816s_state.points[i].track_id = 0;
	}

	return ESP_OK;
}

static esp_err_t touch_cst816s_get_data(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt) {
	ESP_RETURN_ON_FALSE(touch_cst816s_state.initialized, ESP_ERR_INVALID_STATE, TAG, "driver not initialized");
	ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "data is null");
	ESP_RETURN_ON_FALSE(point_cnt != NULL, ESP_ERR_INVALID_ARG, TAG, "point count is null");
	ESP_RETURN_ON_FALSE(max_point_cnt > 0, ESP_ERR_INVALID_ARG, TAG, "max_point_cnt must be > 0");

	*point_cnt = touch_cst816s_state.point_cnt > max_point_cnt ? max_point_cnt : touch_cst816s_state.point_cnt;
	for (uint8_t i = 0;i < *point_cnt;i++) {
		data[i] = touch_cst816s_state.points[i];
	}

	touch_cst816s_state.point_cnt = 0;
	return ESP_OK;
}

static esp_err_t touch_cst816s_reset(void) {
	if (touch_cst816s_state.config.pin_rst >= 0) {
		ESP_RETURN_ON_ERROR(gpio_set_level(touch_cst816s_state.config.pin_rst, touch_cst816s_state.config.reset_level), TAG, "reset low failed");
		vTaskDelay(pdMS_TO_TICKS(200));
		ESP_RETURN_ON_ERROR(gpio_set_level(touch_cst816s_state.config.pin_rst, !touch_cst816s_state.config.reset_level), TAG, "reset high failed");
		vTaskDelay(pdMS_TO_TICKS(200));
	}

	return ESP_OK;
}

static esp_err_t touch_cst816s_read_id(void) {
	uint8_t id = 0;
	ESP_RETURN_ON_ERROR(touch_cst816s_read_reg(TOUCH_CST816S_CHIP_ID_REG, &id, 1), TAG, "read id failed");
	ESP_LOGI(TAG, "IC id: %u", (unsigned int)id);
	return ESP_OK;
}

static esp_err_t touch_cst816s_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
	ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "data is null");

	return i2c_master_write_read_device(
		touch_cst816s_state.config.i2c_port,
		touch_cst816s_state.config.i2c_addr,
		&reg,
		1,
		data,
		len,
		pdMS_TO_TICKS(touch_cst816s_state.config.timeout_ms));
}

static void IRAM_ATTR touch_cst816s_isr(void *arg) {
	(void)arg;
	lispif_touch_irq_from_isr();
}