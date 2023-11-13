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

#include "utils.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_vfs.h"

#include <sys/time.h>
#include <dirent.h>
#include <string.h>

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
