/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef MAIN_DISPLAY_DISP_CO5300_H_
#define MAIN_DISPLAY_DISP_CO5300_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "lispif_disp_extensions.h"

#define DISP_CO5300_WIDTH  466
#define DISP_CO5300_HEIGHT 466

typedef struct {
	spi_host_device_t host;
	int pin_data0;
	int pin_data1;
	int pin_data2;
	int pin_data3;
	int pin_clock;
	int pin_cs;
	int pin_reset;
	int clock_hz;
	int column_offset;
	int row_offset;
} disp_co5300_config_t;

esp_err_t disp_co5300_init(const disp_co5300_config_t *config);
bool disp_co5300_render_image(image_buffer_t *img, uint16_t x, uint16_t y,
	color_t *colors);
void disp_co5300_clear(uint32_t color);
void disp_co5300_reset(void);
esp_err_t disp_co5300_draw_rgb565(int x_start, int y_start, int x_end,
	int y_end, const void *pixels);
esp_err_t disp_co5300_set_brightness(uint8_t brightness_percent);
int disp_co5300_get_width(void);
int disp_co5300_get_height(void);

#endif /* MAIN_DISPLAY_DISP_CO5300_H_ */
