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

#ifndef MAIN_DISPLAY_DISP_JD9165_H_
#define MAIN_DISPLAY_DISP_JD9165_H_

#include <stdint.h>
#include <stdbool.h>
#include "soc/soc_caps.h"
#include "lispif_disp_extensions.h"

#if SOC_MIPI_DSI_SUPPORTED

void disp_jd9165_init(int pin_rst, int lane_mbps);
bool disp_jd9165_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors);
void disp_jd9165_clear(uint32_t color);
void disp_jd9165_reset(void);

#endif

#endif /* MAIN_DISPLAY_DISP_JD9165_H_ */
