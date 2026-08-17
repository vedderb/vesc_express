/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef MAIN_LVGL_LVGL_RUNTIME_H_
#define MAIN_LVGL_LVGL_RUNTIME_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
	size_t total_size;
	size_t free_size;
	size_t largest_free_block;
	uint8_t used_percent;
	uint8_t fragmentation_percent;
} lvgl_runtime_memory_stats_t;

esp_err_t lvgl_runtime_start(void);
esp_err_t lvgl_runtime_stop(void);
bool lvgl_runtime_is_ready(void);

/* All LVGL API calls outside LVGL callbacks must use this pair. */
bool lvgl_runtime_lock(void);
void lvgl_runtime_unlock(void);

esp_err_t lvgl_runtime_get_memory_stats(
	lvgl_runtime_memory_stats_t *stats);

#endif /* MAIN_LVGL_LVGL_RUNTIME_H_ */
