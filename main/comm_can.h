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

#define CAN_STATUS_MSGS_TO_STORE	10

// Functions
void comm_can_init(void);
void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send);
bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type);

void comm_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_current(uint8_t controller_id, float current);
void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay);
void comm_can_set_current_brake(uint8_t controller_id, float current);
void comm_can_set_rpm(uint8_t controller_id, float rpm);
void comm_can_set_pos(uint8_t controller_id, float pos);
void comm_can_set_current_rel(uint8_t controller_id, float current_rel);
void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay);
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel);
void comm_can_set_handbrake(uint8_t controller_id, float current);
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);

can_status_msg *comm_can_get_status_msg_index(int index);
can_status_msg *comm_can_get_status_msg_id(int id);
can_status_msg_2 *comm_can_get_status_msg_2_index(int index);
can_status_msg_2 *comm_can_get_status_msg_2_id(int id);
can_status_msg_3 *comm_can_get_status_msg_3_index(int index);
can_status_msg_3 *comm_can_get_status_msg_3_id(int id);
can_status_msg_4 *comm_can_get_status_msg_4_index(int index);
can_status_msg_4 *comm_can_get_status_msg_4_id(int id);
can_status_msg_5 *comm_can_get_status_msg_5_index(int index);
can_status_msg_5 *comm_can_get_status_msg_5_id(int id);
can_status_msg_6 *comm_can_get_status_msg_6_index(int index);
can_status_msg_6 *comm_can_get_status_msg_6_id(int id);

io_board_adc_values *comm_can_get_io_board_adc_1_4_index(int index);
io_board_adc_values *comm_can_get_io_board_adc_1_4_id(int id);
io_board_adc_values *comm_can_get_io_board_adc_5_8_index(int index);
io_board_adc_values *comm_can_get_io_board_adc_5_8_id(int id);
io_board_digial_inputs *comm_can_get_io_board_digital_in_index(int index);
io_board_digial_inputs *comm_can_get_io_board_digital_in_id(int id);
void comm_can_io_board_set_output_digital(int id, int channel, bool on);
void comm_can_io_board_set_output_pwm(int id, int channel, float duty);

psw_status *comm_can_get_psw_status_index(int index);
psw_status *comm_can_get_psw_status_id(int id);
void comm_can_psw_switch(int id, bool is_on, bool plot);
void comm_can_update_pid_pos_offset(int id, float angle_now, bool store);

#endif /* MAIN_COMM_CAN_H_ */
