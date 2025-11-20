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

#ifndef MAIN_CRYPTO_H_
#define MAIN_CRYPTO_H_

#include <stddef.h>
#include <stdint.h>

int aes_ctr_crypt_inplace(const uint8_t *key, size_t key_len,
                              uint8_t counter[16],
                              uint8_t *buf, size_t start, size_t len);

#endif /* MAIN_CRYPTO_H_ */
