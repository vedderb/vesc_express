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

#ifndef LISPIF_DISP_EXTENSIONS_H_
#define LISPIF_DISP_EXTENSIONS_H_

#include <stdint.h>
#include <stdbool.h>

#include "lispbm.h"

typedef enum {
	indexed2 = 0,
	indexed4,
	indexed16,
	rgb332,
	rgb565,
	rgb888,
	format_not_supported
} color_format_t;

typedef struct {
	color_format_t fmt;
	uint16_t width;
	uint16_t height;
	uint8_t  *data;
	uint8_t  *mem_base;
} image_buffer_t;

typedef enum {
	COLOR_REGULAR = 0,
	COLOR_GRADIENT_X,
	COLOR_GRADIENT_Y,
	COLOR_PRE_X,
	COLOR_PRE_Y,
} COLOR_TYPE;

typedef struct {
	int color1;
	int color2;
	uint16_t param1;
	uint16_t param2;
	bool mirrored;
	COLOR_TYPE type;
	uint32_t *precalc;
} color_t;

#define COLOR_PRECALC_LEN	512

static inline uint32_t color_apply_precalc(color_t color, int x, int y) {
	int pos;
	switch (color.type) {
		case COLOR_PRE_X: {
			pos = x;
			break;
		}
		case COLOR_PRE_Y: {
			pos = y;
			break;
		}
		default: {
			return 0;
		}
	}
	
	int i;
	if (color.mirrored) {
		i = (pos - color.param2) % (color.param1 * 2);
		if (i < 0) {
			i += color.param1 * 2;
		}
		if (i >= color.param1) {
			i = color.param1 * 2 - i - 1;
		}
	} else {
		i = (pos - color.param2) % (color.param1);
		if (i < 0) {
			i += color.param1;
		}
	}
	return color.precalc[i];
}

#define COLOR_CHECK_PRE(color, x, y) (color.precalc ? color_apply_precalc(color, x, y) : lispif_disp_rgb888_from_color(color, x, y))
#define COLOR_TO_RGB888(color, x, y) (color.type == COLOR_REGULAR ? color.color1 : COLOR_CHECK_PRE(color, x, y))

// Interface
bool lispif_disp_is_image_buffer(lbm_value v);
bool lispif_disp_is_color(lbm_value v);
uint32_t lispif_disp_rgb888_from_color(color_t color, int x, int y);

void lispif_load_disp_extensions(void);
void lispif_disp_set_callbacks(
		bool(* volatile render_image)(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors),
		void(* volatile clear)(uint32_t color),
		void(* volatile reset)(void)
);

#endif
