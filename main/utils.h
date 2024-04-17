/*
	Copyright 2022 Benjamin Vedder		benjamin@vedder.se
    Copyright 2023 Rasmus Söderhielm	rasmus.soderhielm@gmail.com

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
#include <math.h>

// Global variables
extern char *string_pin_invalid;

// Functions
int32_t utils_ms_today(void);
int64_t utils_ms_tot(void);
void utils_byte_to_binary(int x, char *b);
void utils_rotate_vector3(float *input, float *rotation, float *output, bool reverse);
bool utils_rmtree(const char *path);
float utils_throttle_curve(float val, float curve_acc, float curve_brake, int mode);

const char *utils_bool_to_str(bool value);
bool utils_gpio_is_valid(int pin);

#define UTILS_AGE_S(x)		((float)(xTaskGetTickCount() - x) / ((float)portTICK_PERIOD_MS * 1000.0))

// Handy conversions for radians/degrees and RPM/radians-per-second
#define DEG2RAD_f(deg) ((deg) * (float)(M_PI / 180.0))
#define RAD2DEG_f(rad) ((rad) * (float)(180.0 / M_PI))
#define RPM2RADPS_f(rpm) ((rpm) * (float)((2.0 * M_PI) / 60.0))
#define RADPS2RPM_f(rad_per_sec) ((rad_per_sec) * (float)(60.0 / (2.0 * M_PI)))

#ifndef M_3PI_2
#define M_3PI_2 4.71238898038469
#endif

// Return the sign of the argument. -1.0 if negative, 1.0 if zero or positive.
#define SIGN(x)				(((x) < 0.0) ? -1.0 : 1.0)

// Squared
#define SQ(x)				((x) * (x))

#ifndef MIN
#define MIN(a,b) (((a) > (b)) ? (b) : (a))
#endif
#ifndef MAX
#define MAX(a,b) (((a) < (b)) ? (b) : (a))
#endif

#ifndef NUMBER_OF
#define NUMBER_OF(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef SIZEOF_MEMBER
// source: https://stackoverflow.com/a/3553321/15507414
#define SIZEOF_MEMBER(type, member) sizeof(((type *)0)->member)
#endif

// For double precision literals
#define D(x) 				((double)x##L)

#ifndef __NOP
#define __NOP()					__asm__ __volatile__ ("nop")
#endif

/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))

static inline void utils_truncate_number(float *number, float min, float max) {
	if (*number > max) {
		*number = max;
	} else if (*number < min) {
		*number = min;
	}
}

static inline void utils_norm_angle_rad(float *angle) {
	while (*angle < -M_PI) { *angle += 2.0 * M_PI; }
	while (*angle >=  M_PI) { *angle -= 2.0 * M_PI; }
}

#endif /* MAIN_UTILS_H_ */
