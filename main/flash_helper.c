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
	uint16_t flags;
	bool mmap_done;
	const void *addr;
	esp_partition_mmap_handle_t handle;
} _code_checks;

static _code_checks code_checks[2] = {0};

static const esp_partition_t* get_partition(int ind) {
	return esp_partition_find_first(
			ESP_PARTITION_TYPE_ANY,
			ESP_PARTITION_SUBTYPE_ANY,
			ind == CODE_IND_QML ? "qml" : "lisp");
}

static bool perform_mmap(int ind) {
	if (code_checks[ind].mmap_done)  {
		return true;
	}

	const esp_partition_t *part = get_partition(ind);

	if (!part) {
		return false;
	}

	esp_err_t res = esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA,
			&code_checks[ind].addr, &code_checks[ind].handle);

	code_checks[ind].mmap_done = res == ESP_OK;
	return code_checks[ind].mmap_done;
}

static void code_check(int ind) {
	if (code_checks[ind].check_done) {
		return;
	}

	const esp_partition_t *part = get_partition(ind);

	if (!part) {
		return;
	}

	if (!perform_mmap(ind))  {
		return;
	}

	uint8_t *base = (uint8_t*)code_checks[ind].addr;
	int32_t index = 0;
	uint32_t code_len = buffer_get_uint32(base, &index);
	uint16_t code_crc = buffer_get_uint16(base, &index);

	if (code_len <= (part->size - 8)) {
		uint16_t crc_calc = crc16(base + index, code_len + 2); // CRC includes the 2 byte flags
		code_checks[ind].ok = crc_calc == code_crc;
	} else {
		code_checks[ind].ok = false;
	}

	if (code_checks[ind].ok) {
		code_checks[ind].size = code_len;
		code_checks[ind].flags = buffer_get_uint16(base, &index);
	} else {
		code_checks[ind].size = 0;
		code_checks[ind].flags = 0;
	}

	code_checks[ind].check_done = true;
}

bool flash_helper_erase_code(int ind, int size) {
	(void)size;

	const esp_partition_t *part = get_partition(ind);

	if (!part) {
		return false;
	}

	code_checks[ind].size = 0;
	code_checks[ind].check_done = false;
	code_checks[ind].ok = false;

	if (!perform_mmap(ind)) {
		return false;
	}

	// Always erase the entire partition as that allows using it as constant storage. To speed
	// up the process erase is only performed on sectors that are not already erased.

	esp_partition_erase_range(part, 0, part->erase_size);
	uint8_t *erased_data = malloc(part->erase_size);
	if (!erased_data) {
		return false;
	}

	esp_partition_read(part, 0, erased_data, part->erase_size);

	for (uint32_t i = part->erase_size; i < part->size; i += part->erase_size) {
		if (memcmp(code_checks[ind].addr + i, erased_data, part->erase_size) != 0) {
			esp_partition_erase_range(part, i, part->erase_size);
		}
	}

	free(erased_data);
	return true;
}

bool flash_helper_write_code(int ind, uint32_t offset, uint8_t *data, uint32_t len) {
	if (offset < (code_checks[ind].size + 8)) {
		code_checks[ind].size = 0;
		code_checks[ind].check_done = false;
		code_checks[ind].ok = false;
	}

	const esp_partition_t *part = get_partition(ind);

	if (!part) {
		return false;
	}

	uint8_t *data_old = flash_helper_code_data_raw(ind);
	bool erased = true;
	for (int i = 0;i < len;i++) {
		if (data_old[offset + i] != 0xff && data_old[offset + i] != data[i]) {
			erased = false;
			break;
		}
	}

	// If the old flash already contains something we erase the sector first. The data that was
	// there previously until that location is saved in RAM, then written back. Note that the
	// data after the offset (if any) is not backed up. That is because we assume this only
	// is used with LBM const data, where doing it this way is a better approach. TODO: Update
	// this comment with an explanation on why that is the case.
	if (!erased) {
		uint32_t sector_start = (offset / part->erase_size) * part->erase_size;
		uint32_t buf_len = offset - sector_start;

		if (buf_len > 0) {
			uint8_t *buf = calloc(buf_len, 1);
			if (!buf) {
				return false;
			}
			memcpy(buf, data_old + sector_start, buf_len);
			bool erase_ok = esp_partition_erase_range(part, sector_start, part->erase_size) == ESP_OK;
			bool write_ok = esp_partition_write(part, sector_start, buf, buf_len) == ESP_OK;
			free(buf);


			if (!erase_ok || !write_ok) {
				return false;
			}
		} else {
			bool erase_ok = esp_partition_erase_range(part, sector_start, part->erase_size) == ESP_OK;

			if (!erase_ok) {
				return false;
			}
		}
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

const uint8_t *flash_helper_code_data_ptr(int ind) {
	code_check(ind);

	if (!code_checks[ind].ok) {
		return NULL;
	}

	return (uint8_t*)code_checks[ind].addr + 8;
}

uint8_t* flash_helper_code_data_raw(int ind) {
	perform_mmap(ind);
	return (uint8_t*)code_checks[ind].addr;
}

int flash_helper_code_size_raw(int ind) {
	const esp_partition_t *part = get_partition(ind);

	if (!part) {
		return 0;
	}

	return part->size;
}

uint32_t flash_helper_code_size(int ind) {
	code_check(ind);
	return code_checks[ind].size;
}

uint16_t flash_helper_code_flags(int ind) {
	code_check(ind);
	return code_checks[ind].flags;
}
