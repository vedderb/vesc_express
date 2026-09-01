/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "lvgl_display.h"

#include <stddef.h>

#include "display_backend.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lvgl_runtime.h"

#define TAG "lvgl_display"

static lv_display_t *m_display;
static void *m_draw_buffer;
static size_t m_draw_buffer_size;
static esp_err_t m_last_flush_error = ESP_OK;
static uint8_t m_invalidate_x_alignment = 1U;
static uint8_t m_invalidate_y_alignment = 1U;

static lv_coord_t align_area_start(lv_coord_t value, uint8_t alignment) {
	if (value <= 0 || alignment <= 1U) {
		return value < 0 ? 0 : value;
	}
	return (lv_coord_t)((value / alignment) * alignment);
}

static lv_coord_t align_area_end(lv_coord_t value, uint8_t alignment,
		lv_coord_t maximum) {
	if (value < 0) {
		return 0;
	}
	if (alignment > 1U) {
		value = (lv_coord_t)((((value + 1) + alignment - 1U) /
			alignment) * alignment - 1U);
	}
	return value > maximum ? maximum : value;
}

static void round_invalidation_area(lv_event_t *event) {
	if (lv_event_get_code(event) != LV_EVENT_INVALIDATE_AREA) {
		return;
	}
	lv_area_t *area = lv_event_get_param(event);
	if (!area || !m_display) {
		return;
	}

	area->x1 = align_area_start(area->x1, m_invalidate_x_alignment);
	area->x2 = align_area_end(area->x2, m_invalidate_x_alignment,
		(lv_coord_t)(lv_display_get_horizontal_resolution(m_display) - 1));
	area->y1 = align_area_start(area->y1, m_invalidate_y_alignment);
	area->y2 = align_area_end(area->y2, m_invalidate_y_alignment,
		(lv_coord_t)(lv_display_get_vertical_resolution(m_display) - 1));
}

static void flush_display(lv_display_t *display, const lv_area_t *area,
		uint8_t *pixels) {
	(void)display;
	m_last_flush_error = display_backend_draw_rgb565(
		area->x1, area->y1, area->x2 + 1, area->y2 + 1, pixels);
	if (m_last_flush_error != ESP_OK) {
		ESP_LOGE(TAG, "flush failed: %s", esp_err_to_name(m_last_flush_error));
	}
	lv_display_flush_ready(display);
}

static void create_smoke_screen(void) {
	lv_obj_t *screen = lv_screen_active();
	lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
	lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

	lv_obj_t *label = lv_label_create(screen);
	lv_label_set_text(label, "Please install\ndisplay package");
	lv_obj_set_width(label, 420);
	lv_obj_set_style_text_font(label, &lv_font_montserrat_32, LV_PART_MAIN);
	lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
	lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
	lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

esp_err_t lvgl_display_attach(const lvgl_display_config_t *config) {
	if (!config || config->buffer_lines == 0 || !lvgl_runtime_is_ready()) {
		return ESP_ERR_INVALID_ARG;
	}
	if (m_display) {
		return ESP_ERR_INVALID_STATE;
	}

	display_backend_info_t backend;
	esp_err_t result = display_backend_get_info(&backend);
	if (result != ESP_OK) {
		return result;
	}
	if (backend.pixel_format != DISPLAY_BACKEND_PIXEL_FORMAT_RGB565_SWAPPED ||
		backend.width <= 0 || backend.height <= 0 ||
		config->buffer_lines > backend.height) {
		return ESP_ERR_NOT_SUPPORTED;
	}

	m_draw_buffer_size =
		(size_t)backend.width * config->buffer_lines * sizeof(uint16_t);
	m_draw_buffer = heap_caps_malloc(m_draw_buffer_size,
		MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	if (!m_draw_buffer) {
		m_draw_buffer_size = 0;
		return ESP_ERR_NO_MEM;
	}

	if (!lvgl_runtime_lock()) {
		heap_caps_free(m_draw_buffer);
		m_draw_buffer = NULL;
		m_draw_buffer_size = 0;
		return ESP_ERR_INVALID_STATE;
	}

	m_display = lv_display_create(backend.width, backend.height);
	if (!m_display) {
		result = ESP_ERR_NO_MEM;
	} else {
		lv_display_set_default(m_display);
		lv_display_set_color_format(
			m_display, LV_COLOR_FORMAT_RGB565_SWAPPED);
		lv_display_set_buffers(m_display, m_draw_buffer, NULL,
			m_draw_buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
		lv_display_set_flush_cb(m_display, flush_display);
		m_invalidate_x_alignment = backend.invalidate_x_alignment;
		m_invalidate_y_alignment = backend.invalidate_y_alignment;
		if (m_invalidate_x_alignment > 1U ||
			m_invalidate_y_alignment > 1U) {
			lv_display_add_event_cb(m_display, round_invalidation_area,
				LV_EVENT_INVALIDATE_AREA, NULL);
		}
		if (config->show_smoke_test) {
			create_smoke_screen();
		}
	}
	lvgl_runtime_unlock();

	if (result != ESP_OK) {
		heap_caps_free(m_draw_buffer);
		m_draw_buffer = NULL;
		m_draw_buffer_size = 0;
		return result;
	}

	ESP_LOGI(TAG, "%s attached with %u-line partial buffer (%u bytes)",
		backend.name, config->buffer_lines, (unsigned)m_draw_buffer_size);
	return ESP_OK;
}

void lvgl_display_detach(void) {
	if (!m_display) {
		return;
	}
	if (lvgl_runtime_lock()) {
		lv_display_delete(m_display);
		m_display = NULL;
		lvgl_runtime_unlock();
	}
	heap_caps_free(m_draw_buffer);
	m_draw_buffer = NULL;
	m_draw_buffer_size = 0;
	m_last_flush_error = ESP_OK;
	m_invalidate_x_alignment = 1U;
	m_invalidate_y_alignment = 1U;
}

bool lvgl_display_is_ready(void) {
	return m_display != NULL;
}

esp_err_t lvgl_display_get_last_flush_error(void) {
	return m_last_flush_error;
}
