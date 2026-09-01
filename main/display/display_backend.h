/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef MAIN_DISPLAY_DISPLAY_BACKEND_H_
#define MAIN_DISPLAY_DISPLAY_BACKEND_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "lispif_disp_extensions.h"

typedef enum {
	/* RGB565 bytes are ordered for transmission on the QSPI panel bus. */
	DISPLAY_BACKEND_PIXEL_FORMAT_RGB565_SWAPPED = 0,
} display_backend_pixel_format_t;

typedef struct {
	const char *name;
	bool (*render_image)(image_buffer_t *img, uint16_t x, uint16_t y,
		color_t *colors);
	void (*clear)(uint32_t color);
	void (*reset)(void);
	esp_err_t (*draw_rgb565)(int x_start, int y_start, int x_end, int y_end,
		const void *pixels);
	esp_err_t (*set_brightness)(uint8_t brightness_percent);
	int (*get_width)(void);
	int (*get_height)(void);
	display_backend_pixel_format_t pixel_format;
	/* LVGL invalidation rectangles are rounded to these pixel boundaries.
	 * Use 1 when the panel has no alignment requirement. */
	uint8_t invalidate_x_alignment;
	uint8_t invalidate_y_alignment;
} display_backend_ops_t;

typedef struct {
	const char *name;
	int width;
	int height;
	display_backend_pixel_format_t pixel_format;
	uint8_t invalidate_x_alignment;
	uint8_t invalidate_y_alignment;
} display_backend_info_t;

esp_err_t display_backend_register(const display_backend_ops_t *ops);
esp_err_t display_backend_bind_lisp(void);
bool display_backend_is_ready(void);
esp_err_t display_backend_get_info(display_backend_info_t *info);

bool display_backend_render_image(image_buffer_t *img, uint16_t x, uint16_t y,
	color_t *colors);
void display_backend_clear(uint32_t color);
void display_backend_reset(void);
esp_err_t display_backend_draw_rgb565(int x_start, int y_start, int x_end,
	int y_end, const void *pixels);
esp_err_t display_backend_set_brightness(uint8_t brightness_percent);

#endif /* MAIN_DISPLAY_DISPLAY_BACKEND_H_ */
