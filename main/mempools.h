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

#ifndef MEMPOOLS_H_
#define MEMPOOLS_H_

#include "datatypes.h"

// Functions
void mempools_init(void);

uint8_t *mempools_get_packet_buffer(void);
uint8_t *mempools_get_lbm_packet_buffer(void);
void mempools_free_packet_buffer(uint8_t *buffer);

#endif /* MEMPOOLS_H_ */
