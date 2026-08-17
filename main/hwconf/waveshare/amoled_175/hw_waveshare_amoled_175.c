/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "hw_waveshare_amoled_175.h"

#include "axp2101.h"
#include "disp_co5300.h"
#include "display_backend.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl_display.h"
#include "lispif_lvgl_extensions.h"
#include "lvgl_runtime.h"

#define TAG "waveshare_amoled"

static esp_err_t m_display_init_result = ESP_ERR_INVALID_STATE;

static esp_err_t init_i2c(void) {
	i2c_config_t config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = WAVESHARE_I2C_SDA,
		.scl_io_num = WAVESHARE_I2C_SCL,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = WAVESHARE_I2C_CLOCK_HZ,
		.clk_flags = 0,
	};

	esp_err_t result = i2c_param_config(I2C_NUM_0, &config);
	if (result != ESP_OK) {
		return result;
	}
	result = i2c_driver_install(I2C_NUM_0, config.mode, 0, 0, 0);
	return result == ESP_ERR_INVALID_STATE ? ESP_OK : result;
}

void hw_init(void) {
	esp_err_t result = init_i2c();
	if (result == ESP_OK) {
		result = axp2101_init(I2C_NUM_0);
	}
	if (result == ESP_OK) {
		result = axp2101_enable_display_power();
	}
	if (result != ESP_OK) {
		m_display_init_result = result;
		ESP_LOGE(TAG, "display power initialization failed: %s",
			esp_err_to_name(result));
		return;
	}
	vTaskDelay(pdMS_TO_TICKS(20));

	disp_co5300_config_t panel_config = {
		.host = SPI2_HOST,
		.pin_data0 = WAVESHARE_DISP_DATA0,
		.pin_data1 = WAVESHARE_DISP_DATA1,
		.pin_data2 = WAVESHARE_DISP_DATA2,
		.pin_data3 = WAVESHARE_DISP_DATA3,
		.pin_clock = WAVESHARE_DISP_CLOCK,
		.pin_cs = WAVESHARE_DISP_CS,
		.pin_reset = WAVESHARE_DISP_RESET,
		.clock_hz = WAVESHARE_DISP_CLOCK_HZ,
		.column_offset = WAVESHARE_DISP_COLUMN_OFFSET,
		.row_offset = 0,
	};
	result = disp_co5300_init(&panel_config);
	if (result == ESP_OK) {
		result = lvgl_runtime_start();
	}
	if (result == ESP_OK) {
		const lvgl_display_config_t display_config = {
			.buffer_lines = 40,
			.show_smoke_test = true,
		};
		result = lvgl_display_attach(&display_config);
	}
	if (result == ESP_OK) {
		lispif_lvgl_extensions_init();
	}

	m_display_init_result = result;
	if (result != ESP_OK) {
		ESP_LOGE(TAG, "display initialization failed: %s",
			esp_err_to_name(result));
	}
}

void hw_post_lispif_init(void) {
	if (m_display_init_result != ESP_OK) {
		return;
	}
	esp_err_t result = display_backend_bind_lisp();
	if (result != ESP_OK) {
		ESP_LOGE(TAG, "Lisp display binding failed: %s",
			esp_err_to_name(result));
	}
}
