/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "disp_co5300.h"

#include "display_backend.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_lcd_co5300.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "CO5300"

#define RENDER_CHUNK_LINES 40
#define PIX_BUF_PIXELS (DISP_CO5300_WIDTH * RENDER_CHUNK_LINES)
#define PIX_BUF_BYTES  (PIX_BUF_PIXELS * sizeof(uint16_t))

static esp_lcd_panel_io_handle_t m_io;
static esp_lcd_panel_handle_t m_panel;
static SemaphoreHandle_t m_refresh_done;
static uint16_t *m_pix_buf;
static int m_column_offset;
static int m_row_offset;

static bool IRAM_ATTR notify_refresh_done(esp_lcd_panel_io_handle_t panel_io,
		esp_lcd_panel_io_event_data_t *event_data, void *user_ctx) {
	(void)panel_io;
	(void)event_data;
	(void)user_ctx;
	BaseType_t need_yield = pdFALSE;
	xSemaphoreGiveFromISR(m_refresh_done, &need_yield);
	return need_yield == pdTRUE;
}

static esp_err_t draw_bitmap_sync(int x_start, int y_start, int x_end,
		int y_end, const void *pixels) {
	if (!m_panel || !m_refresh_done || !pixels) {
		return ESP_ERR_INVALID_STATE;
	}

	esp_err_t result = esp_lcd_panel_draw_bitmap(
		m_panel, x_start, y_start, x_end, y_end, pixels);
	if (result != ESP_OK) {
		return result;
	}
	return xSemaphoreTake(m_refresh_done, portMAX_DELAY) == pdTRUE ?
		ESP_OK : ESP_ERR_TIMEOUT;
}

static inline uint16_t to_disp_color(uint32_t rgb) {
	uint8_t r = (uint8_t)(rgb >> 16) >> 3;
	uint8_t g = (uint8_t)(rgb >> 8) >> 2;
	uint8_t b = (uint8_t)rgb >> 3;
	uint8_t high = (r << 3) | (g >> 3);
	uint8_t low = (g << 5) | b;
	return (uint16_t)high | ((uint16_t)low << 8);
}

static inline uint16_t fetch_rgb565(image_buffer_t *img, uint32_t index,
		color_t *colors) {
	int x = index % img->width;
	int y = index / img->width;
	if (colors && (img->fmt == indexed2 || img->fmt == indexed4 ||
			img->fmt == indexed16)) {
		uint32_t color_index = getpixel(img, x, y);
		return to_disp_color(COLOR_TO_RGB888(colors[color_index], x, y));
	}
	if (img->fmt == rgb565) {
		size_t position = ((size_t)y * img->width + x) * 2U;
		return (uint16_t)img->data[position] |
			((uint16_t)img->data[position + 1] << 8);
	}
	return to_disp_color(getpixel(img, x, y));
}

bool disp_co5300_render_image(image_buffer_t *img, uint16_t x, uint16_t y,
		color_t *colors) {
	if (!img || !m_panel || x + img->width > DISP_CO5300_WIDTH ||
		y + img->height > DISP_CO5300_HEIGHT) {
		return false;
	}

	if (img->fmt == rgb565) {
		for (int line = 0; line < img->height; line += RENDER_CHUNK_LINES) {
			int lines = img->height - line;
			if (lines > RENDER_CHUNK_LINES) {
				lines = RENDER_CHUNK_LINES;
			}
			const uint8_t *source = img->data +
				((size_t)line * img->width * sizeof(uint16_t));
			if (draw_bitmap_sync(x, y + line, x + img->width,
					y + line + lines, source) != ESP_OK) {
				return false;
			}
		}
		return true;
	}

	for (int line = 0; line < img->height; line += RENDER_CHUNK_LINES) {
		int lines = img->height - line;
		if (lines > RENDER_CHUNK_LINES) {
			lines = RENDER_CHUNK_LINES;
		}
		size_t pixel_count = (size_t)img->width * lines;
		for (size_t i = 0; i < pixel_count; i++) {
			m_pix_buf[i] = fetch_rgb565(
				img, (uint32_t)((size_t)line * img->width + i), colors);
		}
		if (draw_bitmap_sync(x, y + line, x + img->width,
				y + line + lines, m_pix_buf) != ESP_OK) {
			return false;
		}
	}
	return true;
}

void disp_co5300_clear(uint32_t color) {
	uint16_t display_color = to_disp_color(color);
	for (size_t i = 0; i < PIX_BUF_PIXELS; i++) {
		m_pix_buf[i] = display_color;
	}

	for (int y = 0; y < DISP_CO5300_HEIGHT; y += RENDER_CHUNK_LINES) {
		int lines = DISP_CO5300_HEIGHT - y;
		if (lines > RENDER_CHUNK_LINES) {
			lines = RENDER_CHUNK_LINES;
		}
		if (draw_bitmap_sync(0, y, DISP_CO5300_WIDTH, y + lines,
				m_pix_buf) != ESP_OK) {
			ESP_LOGE(TAG, "display clear failed at line %d", y);
			return;
		}
	}
}

void disp_co5300_reset(void) {
	if (!m_panel) {
		return;
	}
	esp_err_t result = esp_lcd_panel_reset(m_panel);
	if (result == ESP_OK) {
		result = esp_lcd_panel_init(m_panel);
	}
	if (result == ESP_OK) {
		result = esp_lcd_panel_set_gap(
			m_panel, m_column_offset, m_row_offset);
	}
	if (result == ESP_OK) {
		result = esp_lcd_panel_disp_on_off(m_panel, true);
	}
	if (result == ESP_OK) {
		disp_co5300_clear(0);
	} else {
		ESP_LOGE(TAG, "display reset failed: %s", esp_err_to_name(result));
	}
}

esp_err_t disp_co5300_draw_rgb565(int x_start, int y_start, int x_end,
		int y_end, const void *pixels) {
	if (x_start < 0 || y_start < 0 || x_end <= x_start || y_end <= y_start ||
		x_end > DISP_CO5300_WIDTH || y_end > DISP_CO5300_HEIGHT) {
		return ESP_ERR_INVALID_ARG;
	}
	return draw_bitmap_sync(x_start, y_start, x_end, y_end, pixels);
}

esp_err_t disp_co5300_set_brightness(uint8_t brightness_percent) {
	if (!m_panel) {
		return ESP_ERR_INVALID_STATE;
	}
	return esp_lcd_panel_co5300_set_brightness(m_panel, brightness_percent);
}

int disp_co5300_get_width(void) {
	return DISP_CO5300_WIDTH;
}

int disp_co5300_get_height(void) {
	return DISP_CO5300_HEIGHT;
}

esp_err_t disp_co5300_init(const disp_co5300_config_t *config) {
	if (!config || config->pin_data0 < 0 || config->pin_data1 < 0 ||
		config->pin_data2 < 0 || config->pin_data3 < 0 ||
		config->pin_clock < 0 || config->pin_cs < 0 || config->clock_hz <= 0) {
		return ESP_ERR_INVALID_ARG;
	}
	if (m_panel) {
		return ESP_ERR_INVALID_STATE;
	}

	m_pix_buf = heap_caps_malloc(
		PIX_BUF_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	if (!m_pix_buf) {
		return ESP_ERR_NO_MEM;
	}
	m_refresh_done = xSemaphoreCreateBinary();
	if (!m_refresh_done) {
		heap_caps_free(m_pix_buf);
		m_pix_buf = NULL;
		return ESP_ERR_NO_MEM;
	}

	spi_bus_config_t bus_config = CO5300_PANEL_BUS_QSPI_CONFIG(
		config->pin_clock, config->pin_data0, config->pin_data1,
		config->pin_data2, config->pin_data3, PIX_BUF_BYTES);
	esp_err_t result = spi_bus_initialize(
		config->host, &bus_config, SPI_DMA_CH_AUTO);
	if (result != ESP_OK) {
		return result;
	}

	esp_lcd_panel_io_spi_config_t io_config = CO5300_PANEL_IO_QSPI_CONFIG(
		config->pin_cs, notify_refresh_done, NULL);
	io_config.pclk_hz = config->clock_hz;
	io_config.trans_queue_depth = 1;
	result = esp_lcd_new_panel_io_spi(
		(esp_lcd_spi_bus_handle_t)config->host, &io_config, &m_io);
	if (result != ESP_OK) {
		return result;
	}

	co5300_vendor_config_t vendor_config = {
		.flags = {
			.use_qspi_interface = 1,
		},
	};
	esp_lcd_panel_dev_config_t panel_config = {
		.reset_gpio_num = config->pin_reset,
		.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
		.bits_per_pixel = 16,
		.vendor_config = &vendor_config,
	};
	result = esp_lcd_new_panel_co5300(m_io, &panel_config, &m_panel);
	if (result == ESP_OK) {
		result = esp_lcd_panel_reset(m_panel);
	}
	if (result == ESP_OK) {
		result = esp_lcd_panel_init(m_panel);
	}
	m_column_offset = config->column_offset;
	m_row_offset = config->row_offset;
	if (result == ESP_OK) {
		result = esp_lcd_panel_set_gap(
			m_panel, m_column_offset, m_row_offset);
	}
	if (result == ESP_OK) {
		result = esp_lcd_panel_disp_on_off(m_panel, true);
	}
	if (result != ESP_OK) {
		return result;
	}

	static const display_backend_ops_t backend_ops = {
		.name = "CO5300",
		.render_image = disp_co5300_render_image,
		.clear = disp_co5300_clear,
		.reset = disp_co5300_reset,
		.draw_rgb565 = disp_co5300_draw_rgb565,
		.set_brightness = disp_co5300_set_brightness,
		.get_width = disp_co5300_get_width,
		.get_height = disp_co5300_get_height,
		.pixel_format = DISPLAY_BACKEND_PIXEL_FORMAT_RGB565_SWAPPED,
		/* CO5300 QSPI pixel writes require even starts and extents. LVGL 9
		 * invalidations are otherwise allowed to begin/end on odd pixels,
		 * which shifts the byte stream and appears as tearing. */
		.invalidate_x_alignment = 2,
		.invalidate_y_alignment = 2,
	};
	return display_backend_register(&backend_ops);
}
