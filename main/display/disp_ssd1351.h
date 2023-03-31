/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se
	Copyright 2023 Joel Svensson    svenssonjoel@yahoo.se

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

#ifndef MAIN_DISPLAY_DISP_SSD1351_H_
#define MAIN_DISPLAY_DISP_SSD1351_H_

#include <stdint.h>
#include <stdbool.h>
#include "lispif_disp_extensions.h"

void disp_ssd1351_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int pin_dc, int clock_mhz);
void disp_ssd1351_command(uint8_t command, const uint8_t *args, int argn);
bool disp_ssd1351_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors);
void disp_ssd1351_clear(uint32_t color);
void disp_ssd1351_reset(void);

#endif /* MAIN_DISPLAY_DISP_SSD1351_H_ */
