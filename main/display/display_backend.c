/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "display_backend.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "display_backend"

static SemaphoreHandle_t m_mutex;
static display_backend_ops_t m_ops;
static bool m_ready;

static bool take_backend(void) {
	return m_mutex && xSemaphoreTakeRecursive(m_mutex, portMAX_DELAY) == pdTRUE;
}

static void give_backend(void) {
	xSemaphoreGiveRecursive(m_mutex);
}

esp_err_t display_backend_register(const display_backend_ops_t *ops) {
	if (!ops || !ops->name || !ops->render_image || !ops->clear ||
		!ops->reset || !ops->draw_rgb565 || !ops->get_width ||
		!ops->get_height) {
		return ESP_ERR_INVALID_ARG;
	}

	if (!m_mutex) {
		m_mutex = xSemaphoreCreateRecursiveMutex();
		if (!m_mutex) {
			return ESP_ERR_NO_MEM;
		}
	}

	if (!take_backend()) {
		return ESP_ERR_INVALID_STATE;
	}

	int width = ops->get_width();
	int height = ops->get_height();
	if (width <= 0 || height <= 0) {
		give_backend();
		return ESP_ERR_INVALID_ARG;
	}

	m_ops = *ops;
	m_ready = true;
	give_backend();

	ESP_LOGI(TAG, "active display: %s (%dx%d)", ops->name, width, height);
	return ESP_OK;
}

esp_err_t display_backend_bind_lisp(void) {
	if (!display_backend_is_ready()) {
		return ESP_ERR_INVALID_STATE;
	}

	lbm_display_extensions_set_callbacks(
		display_backend_render_image,
		display_backend_clear,
		display_backend_reset);
	return ESP_OK;
}

bool display_backend_is_ready(void) {
	if (!take_backend()) {
		return false;
	}
	bool ready = m_ready;
	give_backend();
	return ready;
}

esp_err_t display_backend_get_info(display_backend_info_t *info) {
	if (!info) {
		return ESP_ERR_INVALID_ARG;
	}
	if (!take_backend()) {
		return ESP_ERR_INVALID_STATE;
	}
	if (!m_ready) {
		give_backend();
		return ESP_ERR_INVALID_STATE;
	}

	info->name = m_ops.name;
	info->width = m_ops.get_width();
	info->height = m_ops.get_height();
	info->pixel_format = m_ops.pixel_format;
	info->invalidate_x_alignment = m_ops.invalidate_x_alignment ?
		m_ops.invalidate_x_alignment : 1U;
	info->invalidate_y_alignment = m_ops.invalidate_y_alignment ?
		m_ops.invalidate_y_alignment : 1U;
	give_backend();
	return ESP_OK;
}

bool display_backend_render_image(image_buffer_t *img, uint16_t x, uint16_t y,
		color_t *colors) {
	if (!take_backend()) {
		return false;
	}
	bool result = m_ready && m_ops.render_image(img, x, y, colors);
	give_backend();
	return result;
}

void display_backend_clear(uint32_t color) {
	if (!take_backend()) {
		return;
	}
	if (m_ready) {
		m_ops.clear(color);
	}
	give_backend();
}

void display_backend_reset(void) {
	if (!take_backend()) {
		return;
	}
	if (m_ready) {
		m_ops.reset();
	}
	give_backend();
}

esp_err_t display_backend_draw_rgb565(int x_start, int y_start, int x_end,
		int y_end, const void *pixels) {
	if (!pixels || x_start < 0 || y_start < 0 || x_end <= x_start ||
		y_end <= y_start) {
		return ESP_ERR_INVALID_ARG;
	}
	if (!take_backend()) {
		return ESP_ERR_INVALID_STATE;
	}
	if (!m_ready || x_end > m_ops.get_width() || y_end > m_ops.get_height()) {
		give_backend();
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t result = m_ops.draw_rgb565(
		x_start, y_start, x_end, y_end, pixels);
	give_backend();
	return result;
}

esp_err_t display_backend_set_brightness(uint8_t brightness_percent) {
	if (brightness_percent > 100U) {
		return ESP_ERR_INVALID_ARG;
	}
	if (!take_backend()) {
		return ESP_ERR_INVALID_STATE;
	}
	if (!m_ready) {
		give_backend();
		return ESP_ERR_INVALID_STATE;
	}
	if (!m_ops.set_brightness) {
		give_backend();
		return ESP_ERR_NOT_SUPPORTED;
	}

	esp_err_t result = m_ops.set_brightness(brightness_percent);
	give_backend();
	return result;
}
