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

#include "touch_gt911.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TOUCH_GT911_MAX_POINTS 5
#define TOUCH_GT911_REG_PRODUCT_ID 0x8140
#define TOUCH_GT911_REG_CONFIG_VERSION 0x8047
#define TOUCH_GT911_REG_STATUS 0x814E

typedef struct {
	touch_gt911_config_t config;
	lispif_touch_point_data_t points[TOUCH_GT911_MAX_POINTS];
	uint8_t point_cnt;
	bool owns_i2c_driver;
	bool initialized;
} touch_gt911_state_t;

static touch_gt911_state_t touch_gt911_state = {0};
static const char *TAG = "touch_gt911";

static esp_err_t touch_gt911_deinit(void);
static esp_err_t touch_gt911_read_data(void);
static esp_err_t touch_gt911_get_data(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt);
static esp_err_t touch_gt911_read_reg(uint16_t reg, uint8_t *data, uint8_t len);
static esp_err_t touch_gt911_write_reg8(uint16_t reg, uint8_t value);
static esp_err_t touch_gt911_reset(void);
static esp_err_t touch_gt911_read_cfg(void);
static void touch_gt911_isr(void *arg);

esp_err_t touch_gt911_init(const touch_gt911_config_t *config, lispif_touch_driver_t *driver) {
	ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is null");
	ESP_RETURN_ON_FALSE(driver != NULL, ESP_ERR_INVALID_ARG, TAG, "driver is null");
	ESP_RETURN_ON_FALSE(config->pin_sda >= 0, ESP_ERR_INVALID_ARG, TAG, "invalid SDA pin");
	ESP_RETURN_ON_FALSE(config->pin_scl >= 0, ESP_ERR_INVALID_ARG, TAG, "invalid SCL pin");
	ESP_RETURN_ON_FALSE(config->i2c_freq > 0, ESP_ERR_INVALID_ARG, TAG, "invalid I2C frequency");
	ESP_RETURN_ON_FALSE(config->timeout_ms > 0, ESP_ERR_INVALID_ARG, TAG, "invalid timeout");

	if (touch_gt911_state.initialized) {
		touch_gt911_deinit();
	}

	memset(&touch_gt911_state, 0, sizeof(touch_gt911_state));
	touch_gt911_state.config = *config;

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
	touch_gt911_state.owns_i2c_driver = !reuse_existing_i2c;

	if (config->pin_rst >= 0) {
		gpio_config_t rst_conf = {
				.mode = GPIO_MODE_OUTPUT,
				.pin_bit_mask = BIT64(config->pin_rst)
		};
		ESP_RETURN_ON_ERROR(gpio_config(&rst_conf), TAG, "reset pin config failed");
	}

	if (config->pin_int >= 0 && config->pin_rst >= 0) {
		gpio_config_t int_out_conf = {
				.mode = GPIO_MODE_OUTPUT,
				.pull_up_en = GPIO_PULLUP_ENABLE,
				.pin_bit_mask = BIT64(config->pin_int)
		};
		ESP_RETURN_ON_ERROR(gpio_config(&int_out_conf), TAG, "interrupt output config failed");

		ESP_RETURN_ON_ERROR(gpio_set_level(config->pin_rst, config->reset_level), TAG, "reset assert failed");
		ESP_RETURN_ON_ERROR(gpio_set_level(config->pin_int,
				config->i2c_addr == TOUCH_GT911_I2C_ADDR_BACKUP ? 1 : 0), TAG, "address select failed");
		vTaskDelay(pdMS_TO_TICKS(10));
		ESP_RETURN_ON_ERROR(gpio_set_level(config->pin_rst, !config->reset_level), TAG, "reset release failed");
		vTaskDelay(pdMS_TO_TICKS(60));
	} else {
		ESP_RETURN_ON_ERROR(touch_gt911_reset(), TAG, "touch reset failed");
	}

	if (config->pin_int >= 0) {
		gpio_config_t int_in_conf = {
				.mode = GPIO_MODE_INPUT,
				.intr_type = config->interrupt_level ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE,
				.pull_up_en = GPIO_PULLUP_ENABLE,
				.pin_bit_mask = BIT64(config->pin_int)
		};
		ESP_RETURN_ON_ERROR(gpio_config(&int_in_conf), TAG, "interrupt input config failed");

		res = gpio_install_isr_service(0);
		if (res != ESP_OK && res != ESP_ERR_INVALID_STATE) {
			ESP_RETURN_ON_ERROR(res, TAG, "gpio_install_isr_service failed");
		}
		ESP_RETURN_ON_ERROR(gpio_intr_enable(config->pin_int), TAG, "gpio_intr_enable failed");
		ESP_RETURN_ON_ERROR(gpio_isr_handler_add(config->pin_int, touch_gt911_isr, 0), TAG, "gpio_isr_handler_add failed");
	}

	if (config->read_cfg) {
		res = touch_gt911_read_cfg();
		if (res != ESP_OK) {
			ESP_LOGW(TAG, "Read config failed: %s (%d), continuing", esp_err_to_name(res), res);
		}
	}

	touch_gt911_state.initialized = true;
	driver->deinit = touch_gt911_deinit;
	driver->read_data = touch_gt911_read_data;
	driver->get_data = touch_gt911_get_data;

	return ESP_OK;
}

static esp_err_t touch_gt911_deinit(void) {
	if (touch_gt911_state.config.pin_int >= 0) {
		gpio_isr_handler_remove(touch_gt911_state.config.pin_int);
		gpio_intr_disable(touch_gt911_state.config.pin_int);
		gpio_reset_pin(touch_gt911_state.config.pin_int);
	}

	if (touch_gt911_state.config.pin_rst >= 0) {
		gpio_reset_pin(touch_gt911_state.config.pin_rst);
	}

	if (touch_gt911_state.owns_i2c_driver) {
		i2c_driver_delete(touch_gt911_state.config.i2c_port);
	}

	memset(&touch_gt911_state, 0, sizeof(touch_gt911_state));
	return ESP_OK;
}

static esp_err_t touch_gt911_read_data(void) {
	ESP_RETURN_ON_FALSE(touch_gt911_state.initialized, ESP_ERR_INVALID_STATE, TAG, "driver not initialized");

	uint8_t status = 0;
	ESP_RETURN_ON_ERROR(touch_gt911_read_reg(TOUCH_GT911_REG_STATUS, &status, 1), TAG, "status read failed");

	touch_gt911_state.point_cnt = 0;

	if ((status & 0x80U) == 0U) {
		return touch_gt911_write_reg8(TOUCH_GT911_REG_STATUS, 0);
	}

	uint8_t touch_cnt = status & 0x0FU;
	if (touch_cnt == 0 || touch_cnt > TOUCH_GT911_MAX_POINTS) {
		return touch_gt911_write_reg8(TOUCH_GT911_REG_STATUS, 0);
	}

	uint8_t raw[TOUCH_GT911_MAX_POINTS * 8] = {0};
	ESP_RETURN_ON_ERROR(touch_gt911_read_reg(TOUCH_GT911_REG_STATUS + 1, raw, touch_cnt * 8), TAG, "points read failed");
	ESP_RETURN_ON_ERROR(touch_gt911_write_reg8(TOUCH_GT911_REG_STATUS, 0), TAG, "status clear failed");

	for (uint8_t i = 0;i < touch_cnt;i++) {
		uint8_t *point = &raw[i * 8];
		touch_gt911_state.points[i].track_id = point[0];
		touch_gt911_state.points[i].x = ((uint16_t)point[2] << 8) | point[1];
		touch_gt911_state.points[i].y = ((uint16_t)point[4] << 8) | point[3];
		touch_gt911_state.points[i].strength = ((uint16_t)point[6] << 8) | point[5];
	}

	touch_gt911_state.point_cnt = touch_cnt;
	return ESP_OK;
}

static esp_err_t touch_gt911_get_data(lispif_touch_point_data_t *data, uint8_t *point_cnt, uint8_t max_point_cnt) {
	ESP_RETURN_ON_FALSE(touch_gt911_state.initialized, ESP_ERR_INVALID_STATE, TAG, "driver not initialized");
	ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "data is null");
	ESP_RETURN_ON_FALSE(point_cnt != NULL, ESP_ERR_INVALID_ARG, TAG, "point count is null");
	ESP_RETURN_ON_FALSE(max_point_cnt > 0, ESP_ERR_INVALID_ARG, TAG, "max_point_cnt must be > 0");

	*point_cnt = touch_gt911_state.point_cnt > max_point_cnt ? max_point_cnt : touch_gt911_state.point_cnt;
	for (uint8_t i = 0;i < *point_cnt;i++) {
		data[i] = touch_gt911_state.points[i];
	}

	touch_gt911_state.point_cnt = 0;
	return ESP_OK;
}

static esp_err_t touch_gt911_reset(void) {
	if (touch_gt911_state.config.pin_rst >= 0) {
		ESP_RETURN_ON_ERROR(gpio_set_level(touch_gt911_state.config.pin_rst, touch_gt911_state.config.reset_level), TAG, "reset low failed");
		vTaskDelay(pdMS_TO_TICKS(10));
		ESP_RETURN_ON_ERROR(gpio_set_level(touch_gt911_state.config.pin_rst, !touch_gt911_state.config.reset_level), TAG, "reset high failed");
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	return ESP_OK;
}

static esp_err_t touch_gt911_read_cfg(void) {
	uint8_t buf[4] = {0};
	ESP_RETURN_ON_ERROR(touch_gt911_read_reg(TOUCH_GT911_REG_PRODUCT_ID, &buf[0], 3), TAG, "product id read failed");
	ESP_RETURN_ON_ERROR(touch_gt911_read_reg(TOUCH_GT911_REG_CONFIG_VERSION, &buf[3], 1), TAG, "config version read failed");
	ESP_LOGI(TAG, "TouchPad_ID:0x%02x,0x%02x,0x%02x", buf[0], buf[1], buf[2]);
	ESP_LOGI(TAG, "TouchPad_Config_Version:%u", (unsigned int)buf[3]);
	return ESP_OK;
}

static esp_err_t touch_gt911_read_reg(uint16_t reg, uint8_t *data, uint8_t len) {
	ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "data is null");
	uint8_t addr[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
	return i2c_master_write_read_device(
		touch_gt911_state.config.i2c_port,
		touch_gt911_state.config.i2c_addr,
		addr,
		2,
		data,
		len,
		pdMS_TO_TICKS(touch_gt911_state.config.timeout_ms));
}

static esp_err_t touch_gt911_write_reg8(uint16_t reg, uint8_t value) {
	uint8_t payload[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value};
	return i2c_master_write_to_device(
		touch_gt911_state.config.i2c_port,
		touch_gt911_state.config.i2c_addr,
		payload,
		sizeof(payload),
		pdMS_TO_TICKS(touch_gt911_state.config.timeout_ms));
}

static void IRAM_ATTR touch_gt911_isr(void *arg) {
	(void)arg;
	lispif_touch_irq_from_isr();
}