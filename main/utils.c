/*
	Copyright 2022 Benjamin Vedder      benjamin@vedder.se
	Copyright 2023 Rasmus Söderhielm    rasmus.soderhielm@gmail.com

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

#include "utils.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_vfs.h"

#include <sys/time.h>
#include <dirent.h>
#include <string.h>

// Global variables
char *string_pin_invalid = "Invalid pin";

int32_t utils_ms_today(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((tv.tv_sec % 86400) * 1000 + tv.tv_usec / 1000);
}

int64_t utils_ms_tot(void) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((int64_t)tv.tv_sec * (int64_t)1000000 + (int64_t)tv.tv_usec) / (int64_t)1000.0;
}

void utils_byte_to_binary(int x, char *b) {
	b[0] = '\0';

	int z;
	for (z = 128; z > 0; z >>= 1) {
		strcat(b, ((x & z) == z) ? "1" : "0");
	}
}

void utils_rotate_vector3(float *input, float *rotation, float *output, bool reverse) {
	float s1, c1, s2, c2, s3, c3;

	if (rotation[2] != 0.0) {
		s1 = sinf(rotation[2]);
		c1 = cosf(rotation[2]);
	} else {
		s1 = 0.0;
		c1 = 1.0;
	}

	if (rotation[1] != 0.0) {
		s2 = sinf(rotation[1]);
		c2 = cosf(rotation[1]);
	} else {
		s2 = 0.0;
		c2 = 1.0;
	}

	if (rotation[0] != 0.0) {
		s3 = sinf(rotation[0]);
		c3 = cosf(rotation[0]);
	} else {
		s3 = 0.0;
		c3 = 1.0;
	}

	float m11 = c1 * c2;	float m12 = c1 * s2 * s3 - c3 * s1;	float m13 = s1 * s3 + c1 * c3 * s2;
	float m21 = c2 * s1;	float m22 = c1 * c3 + s1 * s2 * s3;	float m23 = c3 * s1 * s2 - c1 * s3;
	float m31 = -s2; 		float m32 = c2 * s3;				float m33 = c2 * c3;

	if (reverse) {
		output[0] = input[0] * m11 + input[1] * m21 + input[2] * m31;
		output[1] = input[0] * m12 + input[1] * m22 + input[2] * m32;
		output[2] = input[0] * m13 + input[1] * m23 + input[2] * m33;
	} else {
		output[0] = input[0] * m11 + input[1] * m12 + input[2] * m13;
		output[1] = input[0] * m21 + input[1] * m22 + input[2] * m23;
		output[2] = input[0] * m31 + input[1] * m32 + input[2] * m33;
	}
}

bool utils_rmtree(const char *path) {
	struct stat stat_path;
	DIR *dir;

	stat(path, &stat_path);

	if (S_ISDIR(stat_path.st_mode) == 0) {
		return unlink(path) == 0;
	}

	if ((dir = opendir(path)) == NULL) {
		return false;
	}

	size_t path_len = strlen(path);

	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL) {
		if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) {
			continue;
		}
		char *path_full = malloc(path_len + strlen(entry->d_name) + 2);
		strcpy(path_full, path);
		strcat(path_full, "/");
		strcat(path_full, entry->d_name);
		utils_rmtree(path_full);
		free(path_full);
	}

	int res = rmdir(path);
	closedir(dir);

	return res == 0;
}

float utils_throttle_curve(float val, float curve_acc, float curve_brake, int mode) {
	float ret = 0.0;

	if (val < -1.0) {
		val = -1.0;
	}

	if (val > 1.0) {
		val = 1.0;
	}

	float val_a = fabsf(val);

	float curve;
	if (val >= 0.0) {
		curve = curve_acc;
	} else {
		curve = curve_brake;
	}

	// See
	// http://math.stackexchange.com/questions/297768/how-would-i-create-a-exponential-ramp-function-from-0-0-to-1-1-with-a-single-val
	if (mode == 0) { // Exponential
		if (curve >= 0.0) {
			ret = 1.0 - powf(1.0 - val_a, 1.0 + curve);
		} else {
			ret = powf(val_a, 1.0 - curve);
		}
	} else if (mode == 1) { // Natural
		if (fabsf(curve) < 1e-10) {
			ret = val_a;
		} else {
			if (curve >= 0.0) {
				ret = 1.0 - ((expf(curve * (1.0 - val_a)) - 1.0) / (expf(curve) - 1.0));
			} else {
				ret = (expf(-curve * val_a) - 1.0) / (expf(-curve) - 1.0);
			}
		}
	} else if (mode == 2) { // Polynomial
		if (curve >= 0.0) {
			ret = 1.0 - ((1.0 - val_a) / (1.0 + curve * val_a));
		} else {
			ret = val_a / (1.0 - curve * (1.0 - val_a));
		}
	} else { // Linear
		ret = val_a;
	}

	if (val < 0.0) {
		ret = -ret;
	}

	return ret;
}

const char *utils_bool_to_str(bool value) {
	return value ? "true" : "false";
}

bool utils_gpio_is_valid(int pin) {
	switch (pin) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 18:
	case 19:
	case 20:
	case 21:
		return true;

	default:
		return false;
	}
}
