/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAIN_UTILS_H_
#define MAIN_UTILS_H_

#include <stdint.h>
#include <stdbool.h>

int32_t utils_ms_today(void);
int64_t utils_ms_tot(void);

#define UTILS_AGE_S(x)		((float)(xTaskGetTickCount() - x) / ((float)portTICK_PERIOD_MS * 1000.0))

// Return the sign of the argument. -1.0 if negative, 1.0 if zero or positive.
#define SIGN(x)				(((x) < 0.0) ? -1.0 : 1.0)

// Squared
#define SQ(x)				((x) * (x))

// For double precision literals
#define D(x) 				((double)x##L)

#endif /* MAIN_UTILS_H_ */
