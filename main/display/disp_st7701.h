/*
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

#ifndef MAIN_DISPLAY_DISP_ST7701_H_
#define MAIN_DISPLAY_DISP_ST7701_H_

#include <stdint.h>
#include <stdbool.h>
#include "lispif_disp_extensions.h"

#if CONFIG_IDF_TARGET_ESP32P4

void disp_st7701_init(int pin_rst, int lane_mbps);
void disp_st7701_deinit(void);
bool disp_st7701_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors);
void disp_st7701_clear(uint32_t color);
void disp_st7701_reset(void);

#endif

#endif
