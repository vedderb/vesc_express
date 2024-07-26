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

#ifndef MAIN_LOG_H_
#define MAIN_LOG_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

// Functions
bool log_init(void);
bool log_mount_card(int pin_mosi, int pin_miso, int pin_sck, int pin_cs, int freq);
void log_unmount_card(void);
void log_process_packet(unsigned char *data, unsigned int len);

// Global variables
extern char *file_basepath;

#endif /* MAIN_LOG_H_ */
