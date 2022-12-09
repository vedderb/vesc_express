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

#ifndef MAIN_COMM_CAN_H_
#define MAIN_COMM_CAN_H_

#include "datatypes.h"

// Functions
void comm_can_init(void);
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send);
bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type);

void comm_can_set_duty(uint8_t controller_id, float duty);

#endif /* MAIN_COMM_CAN_H_ */
