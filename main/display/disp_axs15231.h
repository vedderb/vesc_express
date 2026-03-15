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

#ifndef MAIN_DISPLAY_DISP_AXS15231_H_
#define MAIN_DISPLAY_DISP_AXS15231_H_

#include <stdint.h>
#include <stdbool.h>
#include "lispif_disp_extensions.h"

void disp_axs15231_init(int pin_sd0, int pin_sd1, int pin_sd2, int pin_sd3, int pin_clk, int pin_cs, int pin_reset, int clock_mhz);
void disp_axs15231_command(uint8_t command, const uint8_t *args, int argn);
bool disp_axs15231_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors);
void disp_axs15231_clear(uint32_t color);
void disp_axs15231_reset(void);

#endif /* MAIN_DISPLAY_DISP_AXS15231_H_ */
