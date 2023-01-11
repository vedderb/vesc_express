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

#include "flash_helper.h"
#include "esp_partition.h"
#include "crc.h"
#include "buffer.h"

#include <string.h>

typedef struct {
	bool check_done;
	bool ok;
	uint32_t size;
} _code_checks;

static _code_checks code_checks[2] = {0};

static const esp_partition_t* get_partition(int ind) {
	return esp_partition_find_first(
			ESP_PARTITION_TYPE_ANY,
			ESP_PARTITION_SUBTYPE_ANY,
			ind == CODE_IND_QML ? "qml" : "lisp");
}

static void code_check(int ind) {
	if (code_checks[ind].check_done) {
		return;
	}

	const esp_partition_t *part = get_partition(ind);

	if (!part) {
		return;
	}

	uint8_t base[64];
	esp_partition_read(part, 0, base, 8);

	int32_t index = 0;
	uint32_t code_len = buffer_get_uint32(base, &index);
	uint16_t code_crc = buffer_get_uint16(base, &index);
	uint16_t crc_calc = crc16(base + index, 2);

	if (code_len <= (part->size - 8)) {
		int ind_code = 0;
		while (ind_code < code_len) {
			int data_left = code_len - ind_code;

			if (data_left > 64) {
				esp_partition_read(part, ind_code + 8, base, 64);
				crc_calc = crc16_with_init(base, 64, crc_calc);
				ind_code += 64;
			} else {
				esp_partition_read(part, ind_code + 8, base, data_left);
				crc_calc = crc16_with_init(base, data_left, crc_calc);
				ind_code += data_left;
			}
		}

		code_checks[ind].ok = crc_calc == code_crc;
		code_checks[ind].size = code_len;
	} else {
		code_checks[ind].ok = false;
		code_checks[ind].size = 0;
	}

	code_checks[ind].check_done = true;
}

bool flash_helper_erase_code(int ind) {
	const esp_partition_t *part = get_partition(ind);

	if (!part) {
		return false;
	}

	code_checks[ind].size = 0;
	code_checks[ind].check_done = false;
	code_checks[ind].ok = false;

	return esp_partition_erase_range(part, 0, part->size) == ESP_OK;
}

bool flash_helper_write_code(int ind, uint32_t offset, uint8_t *data, uint32_t len) {
	code_checks[ind].size = 0;
	code_checks[ind].check_done = false;
	code_checks[ind].ok = false;

	const esp_partition_t *part = get_partition(ind);

	if (!part) {
		return false;
	}

	return esp_partition_write(part, offset, data, len) == ESP_OK;
}

bool flash_helper_code_data(int ind, uint32_t offset, uint8_t *data, uint32_t len) {
	code_check(ind);

	if (!code_checks[ind].ok) {
		return false;
	}

	const esp_partition_t *part = get_partition(ind);

	if (!part) {
		return 0;
	}

	return esp_partition_read(part, offset + 8, data, len) == ESP_OK;
}

uint32_t flash_helper_code_size(int ind) {
	code_check(ind);
	return code_checks[ind].size;
}

uint16_t flash_helper_code_flags(int ind) {
	code_check(ind);

	if (!code_checks[ind].ok) {
		return 0;
	}

	const esp_partition_t *part = get_partition(ind);

	if (!part) {
		return 0;
	}

	uint8_t base[4];
	esp_partition_read(part, 0, base, 4);

	int32_t index = 0;
	uint32_t code_len = buffer_get_uint32(base, &index);
	return code_len;
}
