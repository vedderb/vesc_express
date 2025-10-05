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

#include "crypto.h"
#include "aes/esp_aes.h"

int aes_ctr_crypt_inplace(const uint8_t *key, size_t key_len,
                              uint8_t counter[16],
                              uint8_t *buf, size_t start, size_t len)
{
    if (!key || !counter || !buf) return -1;
    if (!(key_len == 16 || key_len == 24 || key_len == 32)) return -2;
    if (len == 0) return 0;

    esp_aes_context ctx;
    esp_aes_init(&ctx);

    int rc = esp_aes_setkey(&ctx, key, (int)(key_len * 8));
    if (rc != 0) {
        esp_aes_free(&ctx);
        return -3;
    }

    uint8_t stream_block[16] = {0};
    size_t  nc_off = 0;

    rc = esp_aes_crypt_ctr(&ctx, len, &nc_off, counter, stream_block,
                           buf + start, buf + start);

    esp_aes_free(&ctx);
    return rc == 0 ? 0 : -4;
}