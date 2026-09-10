/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef MAIN_LVGL_LVGL_DISPLAY_H_
#define MAIN_LVGL_LVGL_DISPLAY_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
	uint16_t buffer_lines;
	bool show_smoke_test;
} lvgl_display_config_t;

esp_err_t lvgl_display_attach(const lvgl_display_config_t *config);
void lvgl_display_detach(void);
bool lvgl_display_is_ready(void);
esp_err_t lvgl_display_get_last_flush_error(void);

#endif /* MAIN_LVGL_LVGL_DISPLAY_H_ */
