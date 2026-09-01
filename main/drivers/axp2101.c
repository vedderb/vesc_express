/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "axp2101.h"

#include "freertos/FreeRTOS.h"

#define AXP2101_ADDRESS        0x34
#define AXP2101_REG_IC_TYPE    0x03
#define AXP2101_IC_TYPE        0x4A
#define AXP2101_REG_LDO_ENABLE 0x90
#define AXP2101_DISPLAY_LDOS   0x0F

static i2c_port_t m_port = I2C_NUM_MAX;
static bool m_ready;

static esp_err_t read_reg(uint8_t reg, uint8_t *value) {
	return i2c_master_write_read_device(
		m_port, AXP2101_ADDRESS, &reg, 1, value, 1, pdMS_TO_TICKS(100));
}

static esp_err_t write_reg(uint8_t reg, uint8_t value) {
	uint8_t data[] = {reg, value};
	return i2c_master_write_to_device(
		m_port, AXP2101_ADDRESS, data, sizeof(data), pdMS_TO_TICKS(100));
}

esp_err_t axp2101_init(i2c_port_t port) {
	m_port = port;
	m_ready = false;

	uint8_t type = 0;
	esp_err_t result = read_reg(AXP2101_REG_IC_TYPE, &type);
	if (result != ESP_OK) {
		return result;
	}
	if (type != AXP2101_IC_TYPE) {
		return ESP_ERR_NOT_FOUND;
	}

	m_ready = true;
	return ESP_OK;
}

esp_err_t axp2101_enable_display_power(void) {
	if (!m_ready) {
		return ESP_ERR_INVALID_STATE;
	}

	uint8_t enable = 0;
	esp_err_t result = read_reg(AXP2101_REG_LDO_ENABLE, &enable);
	if (result != ESP_OK) {
		return result;
	}

	return write_reg(AXP2101_REG_LDO_ENABLE,
		enable | AXP2101_DISPLAY_LDOS);
}

bool axp2101_is_ready(void) {
	return m_ready;
}
