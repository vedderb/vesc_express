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
	uint8_t  data_offset;
} image_buffer_t;

// Interface
bool lispif_disp_is_image_buffer(lbm_value v);

// Load extensions
void lispif_load_disp_extensions(void);

#endif
