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
