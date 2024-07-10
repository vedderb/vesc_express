/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se

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

#ifndef FLASH_HELPER_H_
#define FLASH_HELPER_H_

#include <stdint.h>
#include <stdbool.h>

#define CODE_IND_QML	0
#define CODE_IND_LISP	1

bool flash_helper_erase_code(int ind, int size);
bool flash_helper_write_code(int ind, uint32_t offset, uint8_t *data, uint32_t len, uint32_t save_after);
bool flash_helper_code_data(int ind, uint32_t offset, uint8_t *data, uint32_t len);
const uint8_t *flash_helper_code_data_ptr(int ind);
uint8_t* flash_helper_code_data_raw(int ind);
int flash_helper_code_size_raw(int ind);
uint32_t flash_helper_code_size(int ind);
uint16_t flash_helper_code_flags(int ind);
unsigned int flash_helper_write_erase_cnt(void);

#endif /* FLASH_HELPER_H_ */
