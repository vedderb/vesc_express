/*
	Copyright 2020 - 2023 Benjamin Vedder	benjamin@vedder.se

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

#include "bms.h"
#include "buffer.h"
#include "utils.h"
#include "datatypes.h"
#include "comm_can.h"
#include "commands.h"
#include "comm_usb.h"
#include "main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <math.h>

// Settings
#define MAX_CAN_AGE_SEC				2.0

// Private variables
static volatile bms_values m_values;
static volatile bms_soc_soh_temp_stat m_stat_temp_max;
static volatile bms_soc_soh_temp_stat m_stat_soc_min;
static volatile bms_soc_soh_temp_stat m_stat_soc_max;

// Function pointers
static void(*cmd_handler)(COMM_PACKET_ID cmd, int param1, int param2) = 0;

void bms_init(void) {
	memset((void*)&m_values, 0, sizeof(m_values));
	memset((void*)&m_stat_temp_max, 0, sizeof(m_stat_temp_max));
	memset((void*)&m_stat_soc_min, 0, sizeof(m_stat_soc_min));
	memset((void*)&m_stat_soc_max, 0, sizeof(m_stat_soc_max));

	m_values.can_id = -1;
	m_stat_temp_max.id = -1;
	m_stat_soc_min.id = -1;
	m_stat_soc_max.id = -1;
}

bool bms_process_can_frame(uint32_t can_id, uint8_t *data8, int len, bool is_ext) {
	bool used_data = false;

	if (is_ext) {
		uint8_t id = can_id & 0xFF;
		CAN_PACKET_ID cmd = can_id >> 8;

		switch (cmd) {
		case CAN_PACKET_BMS_SOC_SOH_TEMP_STAT: {
			used_data = true;

			int32_t ind = 0;
			bms_soc_soh_temp_stat msg;
			msg.id = id;
			msg.rx_time = xTaskGetTickCount();
			msg.v_cell_min = buffer_get_float16(data8, 1e3, &ind);
			msg.v_cell_max = buffer_get_float16(data8, 1e3, &ind);
			msg.soc = ((float)((uint8_t)data8[ind++])) / 255.0;
			msg.soh = ((float)((uint8_t)data8[ind++])) / 255.0;
			msg.t_cell_max = (float)((int8_t)data8[ind++]);
			uint8_t stat = data8[ind++];
			msg.is_charging = (stat >> 0) & 1;
			msg.is_balancing = (stat >> 1) & 1;
			msg.is_charge_allowed = (stat >> 2) & 1;

			if (id == m_values.can_id || m_values.can_id == -1 || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
				m_values.can_id = id;
				m_values.update_time = xTaskGetTickCount();
				m_values.soc = msg.soc;
				m_values.soh = msg.soh;
				m_values.temp_max_cell = msg.t_cell_max;
				m_values.is_charging = msg.is_charging ? 1 : 0;
				m_values.is_balancing = msg.is_balancing ? 1 : 0;
				m_values.is_charge_allowed = msg.is_charge_allowed ? 1 : 0;
			}

			// In case there is more than one BMS, keep track of the limiting
			// values for all of them.

			if (m_stat_temp_max.id < 0 ||
					UTILS_AGE_S(m_stat_temp_max.rx_time) > MAX_CAN_AGE_SEC ||
					m_stat_temp_max.t_cell_max < msg.t_cell_max) {
				m_stat_temp_max = msg;
			} else if (m_stat_temp_max.id == msg.id) {
				m_stat_temp_max = msg;
			}

			if (m_stat_soc_min.id < 0 ||
					UTILS_AGE_S(m_stat_soc_min.rx_time) > MAX_CAN_AGE_SEC ||
					m_stat_soc_min.soc > msg.soc) {
				m_stat_soc_min = msg;
			} else if (m_stat_soc_min.id == msg.id) {
				m_stat_soc_min = msg;
			}

			if (m_stat_soc_max.id < 0 ||
					UTILS_AGE_S(m_stat_soc_max.rx_time) > MAX_CAN_AGE_SEC ||
					m_stat_soc_max.soc < msg.soc) {
				m_stat_soc_max = msg;
			} else if (m_stat_soc_max.id == msg.id) {
				m_stat_soc_max = msg;
			}
		} break;

		case CAN_PACKET_BMS_V_TOT: {
			used_data = true;

			if (id == m_values.can_id || m_values.can_id == -1 || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
				int32_t ind = 0;
				m_values.can_id = id;
				m_values.update_time = xTaskGetTickCount();
				m_values.v_tot = buffer_get_float32_auto(data8, &ind);
				m_values.v_charge = buffer_get_float32_auto(data8, &ind);
			}
		} break;

		case CAN_PACKET_BMS_I: {
			used_data = true;

			if (id == m_values.can_id || m_values.can_id == -1 || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
				int32_t ind = 0;
				m_values.can_id = id;
				m_values.update_time = xTaskGetTickCount();
				m_values.i_in = buffer_get_float32_auto(data8, &ind);
				m_values.i_in_ic = buffer_get_float32_auto(data8, &ind);
			}
		} break;

		case CAN_PACKET_BMS_AH_WH: {
			used_data = true;

			if (id == m_values.can_id || m_values.can_id == -1 || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
				int32_t ind = 0;
				m_values.can_id = id;
				m_values.update_time = xTaskGetTickCount();
				m_values.ah_cnt = buffer_get_float32_auto(data8, &ind);
				m_values.wh_cnt = buffer_get_float32_auto(data8, &ind);
			}
		} break;

		case CAN_PACKET_BMS_V_CELL: {
			used_data = true;

			if (id == m_values.can_id || m_values.can_id == -1 || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
				int32_t ind = 0;
				m_values.can_id = id;
				m_values.update_time = xTaskGetTickCount();
				unsigned int ofs = data8[ind++];
				m_values.cell_num = data8[ind++];

				while(ind < len) {
					if (ofs >= (sizeof(m_values.v_cell) / sizeof(float))) {
						// Out of buffer space
						break;
					}

					m_values.v_cell[ofs++] = buffer_get_float16(data8, 1e3, &ind);
				}
			}
		} break;

		case CAN_PACKET_BMS_BAL: {
			used_data = true;

			if (id == m_values.can_id || m_values.can_id == -1 || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
				int32_t ind = 0;
				m_values.can_id = id;
				m_values.update_time = xTaskGetTickCount();

				int cell_num = data8[0];
				uint64_t bal_state_0 = buffer_get_uint32(data8, &ind);
				bal_state_0 &= 0x00FFFFFF;
				uint64_t bal_state_1 = buffer_get_uint32(data8, &ind);
				uint64_t bal_state = bal_state_0 << 32 | bal_state_1;
				ind = 0;

				while (ind < (int)(sizeof(m_values.bal_state) / sizeof(bool)) && ind < cell_num) {
					m_values.bal_state[ind] = (bal_state >> ind) & 1;
					ind++;
				}
			}
		} break;

		case CAN_PACKET_BMS_TEMPS: {
			used_data = true;

			if (id == m_values.can_id || m_values.can_id == -1 || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
				int32_t ind = 0;
				m_values.can_id = id;
				m_values.update_time = xTaskGetTickCount();
				unsigned int ofs = data8[ind++];
				m_values.temp_adc_num = data8[ind++];

				while(ind < len) {
					if (ofs >= (sizeof(m_values.temps_adc) / sizeof(float))) {
						// Out of buffer space
						break;
					}

					m_values.temps_adc[ofs++] = buffer_get_float16(data8, 1e2, &ind);
				}
			}
		} break;

		case CAN_PACKET_BMS_HUM: {
			used_data = true;

			if (id == m_values.can_id || m_values.can_id == -1 || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
				int32_t ind = 0;
				m_values.can_id = id;
				m_values.update_time = xTaskGetTickCount();
				m_values.temp_hum = buffer_get_float16(data8, 1e2, &ind);
				m_values.hum = buffer_get_float16(data8, 1e2, &ind);
				m_values.temp_ic = buffer_get_float16(data8, 1e2, &ind);
				if (len == 8) {
					m_values.pressure = buffer_get_float16(data8, 1e-1, &ind);
				}
			}
		} break;

		case CAN_PACKET_BMS_AH_WH_CHG_TOTAL: {
			used_data = true;

			if (id == m_values.can_id || m_values.can_id == -1 || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
				int32_t ind = 0;
				m_values.can_id = id;
				m_values.update_time = xTaskGetTickCount();
				m_values.ah_cnt_chg_total = buffer_get_float32_auto(data8, &ind);
				m_values.wh_cnt_chg_total = buffer_get_float32_auto(data8, &ind);
			}
		} break;

		case CAN_PACKET_BMS_AH_WH_DIS_TOTAL: {
			used_data = true;

			if (id == m_values.can_id || m_values.can_id == -1 || UTILS_AGE_S(m_values.update_time) > MAX_CAN_AGE_SEC) {
				int32_t ind = 0;
				m_values.can_id = id;
				m_values.update_time = xTaskGetTickCount();
				m_values.ah_cnt_dis_total = buffer_get_float32_auto(data8, &ind);
				m_values.wh_cnt_dis_total = buffer_get_float32_auto(data8, &ind);
			}
		} break;

		default:
			break;
		}
	}

	return used_data;
}

void bms_process_cmd(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len)) {
	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;

	switch (packet_id) {
	case COMM_BMS_GET_VALUES: {
		int32_t ind = 0;
		uint8_t send_buffer[256];

		send_buffer[ind++] = packet_id;

		buffer_append_float32(send_buffer, m_values.v_tot, 1e6, &ind);
		buffer_append_float32(send_buffer, m_values.v_charge, 1e6, &ind);
		buffer_append_float32(send_buffer, m_values.i_in, 1e6, &ind);
		buffer_append_float32(send_buffer, m_values.i_in_ic, 1e6, &ind);
		buffer_append_float32(send_buffer, m_values.ah_cnt, 1e3, &ind);
		buffer_append_float32(send_buffer, m_values.wh_cnt, 1e3, &ind);

		// Cell voltages
		send_buffer[ind++] = m_values.cell_num;
		for (int i = 0;i < m_values.cell_num;i++) {
			buffer_append_float16(send_buffer, m_values.v_cell[i], 1e3, &ind);
		}

		// Balancing state
		for (int i = 0;i < m_values.cell_num;i++) {
			send_buffer[ind++] = m_values.bal_state[i];
		}

		// Temperatures
		send_buffer[ind++] = m_values.temp_adc_num;
		for (int i = 0;i < m_values.temp_adc_num;i++) {
			buffer_append_float16(send_buffer, m_values.temps_adc[i], 1e2, &ind);
		}
		buffer_append_float16(send_buffer, m_values.temp_ic, 1e2, &ind);

		// Humidity
		buffer_append_float16(send_buffer, m_values.temp_hum, 1e2, &ind);
		buffer_append_float16(send_buffer, m_values.hum, 1e2, &ind);

		// Highest cell temperature
		buffer_append_float16(send_buffer, m_values.temp_max_cell, 1e2, &ind);

		// State of charge and state of health
		buffer_append_float16(send_buffer, m_values.soc, 1e3, &ind);
		buffer_append_float16(send_buffer, m_values.soh, 1e3, &ind);

		// CAN ID
		send_buffer[ind++] = m_values.can_id;

		// Total charge and discharge counters
		buffer_append_float32_auto(send_buffer, m_values.ah_cnt_chg_total, &ind);
		buffer_append_float32_auto(send_buffer, m_values.wh_cnt_chg_total, &ind);
		buffer_append_float32_auto(send_buffer, m_values.ah_cnt_dis_total, &ind);
		buffer_append_float32_auto(send_buffer, m_values.wh_cnt_dis_total, &ind);

		// Pressure
		buffer_append_float16(send_buffer, m_values.pressure, 1e-1, &ind);

		reply_func(send_buffer, ind);
	} break;

	default:
		break;
	}

	switch (packet_id) {
	case COMM_BMS_SET_CHARGE_ALLOWED:
	case COMM_BMS_SET_BALANCE_OVERRIDE:
	case COMM_BMS_RESET_COUNTERS:
	case COMM_BMS_FORCE_BALANCE:
	case COMM_BMS_ZERO_CURRENT_OFFSET: {
		if (cmd_handler) {
			int param1 = -1;
			int param2 = -1;

			if (len >= 1) {
				param1 = data[0];
			}

			if (len >= 2) {
				param2 = data[1];
			}

			cmd_handler(packet_id, param1, param2);
		} else {
			if (UTILS_AGE_S(m_values.update_time) < MAX_CAN_AGE_SEC) {
				comm_can_send_buffer(m_values.can_id, data - 1, len + 1, 0);
			}
		}
	} break;

	default:
		break;
	}
}

void bms_register_cmd_handler(void (*handler)(COMM_PACKET_ID cmd, int param1, int param2)) {
	cmd_handler = handler;
}

volatile bms_values *bms_get_values(void) {
	return &m_values;
}

void bms_send_status_can(void) {
	int32_t send_index = 0;
	uint8_t buffer[8];

	uint8_t id = backup.config.controller_id;

	buffer_append_float32_auto(buffer, m_values.v_tot, &send_index);
	buffer_append_float32_auto(buffer, m_values.v_charge, &send_index);
	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_BMS_V_TOT << 8), buffer, send_index);

	send_index = 0;
	buffer_append_float32_auto(buffer, m_values.i_in, &send_index);
	buffer_append_float32_auto(buffer, m_values.i_in_ic, &send_index);
	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_BMS_I << 8), buffer, send_index);

	send_index = 0;
	buffer_append_float32_auto(buffer, m_values.ah_cnt, &send_index);
	buffer_append_float32_auto(buffer, m_values.wh_cnt, &send_index);
	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_BMS_AH_WH << 8), buffer, send_index);

	int cell_now = 0;
	int cell_max = m_values.cell_num;
	if (cell_max > BMS_MAX_CELLS) {
		cell_max = BMS_MAX_CELLS;
	}

	while (cell_now < cell_max) {
		send_index = 0;
		buffer[send_index++] = cell_now;
		buffer[send_index++] = m_values.cell_num;
		if (cell_now < cell_max) {
			buffer_append_float16(buffer, m_values.v_cell[cell_now++], 1e3, &send_index);
		}
		if (cell_now < cell_max) {
			buffer_append_float16(buffer, m_values.v_cell[cell_now++], 1e3, &send_index);
		}
		if (cell_now < cell_max) {
			buffer_append_float16(buffer, m_values.v_cell[cell_now++], 1e3, &send_index);
		}
		comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_BMS_V_CELL << 8), buffer, send_index);
	}

	send_index = 0;
	buffer[send_index++] = cell_max;
	uint64_t bal_state = 0;
	for (int i = 0;i < cell_max;i++) {
		bal_state |= (uint64_t)m_values.bal_state[i] << i;
	}
	buffer[send_index++] = (bal_state >> 48) & 0xFF;
	buffer[send_index++] = (bal_state >> 40) & 0xFF;
	buffer[send_index++] = (bal_state >> 32) & 0xFF;
	buffer[send_index++] = (bal_state >> 24) & 0xFF;
	buffer[send_index++] = (bal_state >> 16) & 0xFF;
	buffer[send_index++] = (bal_state >> 8) & 0xFF;
	buffer[send_index++] = (bal_state >> 0) & 0xFF;
	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_BMS_BAL << 8), buffer, send_index);

	int temp_now = 0;
	int temp_max = m_values.temp_adc_num;
	if (temp_max > BMS_MAX_TEMPS) {
		temp_max = BMS_MAX_TEMPS;
	}

	while (temp_now < temp_max) {
		send_index = 0;
		buffer[send_index++] = temp_now;
		buffer[send_index++] = temp_max;
		if (temp_now < temp_max) {
			buffer_append_float16(buffer, m_values.temps_adc[temp_now++], 1e2, &send_index);
		}
		if (temp_now < temp_max) {
			buffer_append_float16(buffer, m_values.temps_adc[temp_now++], 1e2, &send_index);
		}
		if (temp_now < temp_max) {
			buffer_append_float16(buffer, m_values.temps_adc[temp_now++], 1e2, &send_index);
		}
		comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_BMS_TEMPS << 8), buffer, send_index);
	}

	send_index = 0;
	buffer_append_float16(buffer, m_values.temp_hum, 1e2, &send_index);
	buffer_append_float16(buffer, m_values.hum, 1e2, &send_index);
	buffer_append_float16(buffer, m_values.temp_ic, 1e2, &send_index); // Put IC temp here instead of making mew msg
	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_BMS_HUM << 8), buffer, send_index);

	/*
	 * CAN_PACKET_BMS_SOC_SOH_TEMP_STAT
	 *
	 * b[0] - b[1]: V_CELL_MIN (mV)
	 * b[2] - b[3]: V_CELL_MAX (mV)
	 * b[4]: SoC (0 - 255)
	 * b[5]: SoH (0 - 255)
	 * b[6]: T_CELL_MAX (-128 to +127 degC)
	 * b[7]: State bitfield:
	 * [B7      B6      B5      B4      B3      B2      B1      B0      ]
	 * [RSV     RSV     RSV     RSV     RSV     CHG_OK  IS_BAL  IS_CHG  ]
	 */
	send_index = 0;
	buffer_append_float16(buffer, -1.0, 1e3, &send_index);
	buffer_append_float16(buffer, -1.0, 1e3, &send_index);
	buffer[send_index++] = (uint8_t)(m_values.soc * 255.0);
	buffer[send_index++] = (uint8_t)(m_values.soh * 255.0);
	buffer[send_index++] = (int8_t)m_values.temp_max_cell;
	buffer[send_index++] =
			((m_values.is_charging ? 1 : 0) << 0) |
			((m_values.is_balancing ? 1 : 0) << 1) |
			((m_values.is_charge_allowed ? 1 : 0) << 2);
	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_BMS_SOC_SOH_TEMP_STAT << 8), buffer, send_index);

	send_index = 0;
	buffer_append_float32_auto(buffer, m_values.ah_cnt_chg_total, &send_index);
	buffer_append_float32_auto(buffer, m_values.wh_cnt_chg_total, &send_index);
	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_BMS_AH_WH_CHG_TOTAL << 8), buffer, send_index);

	send_index = 0;
	buffer_append_float32_auto(buffer, m_values.ah_cnt_dis_total, &send_index);
	buffer_append_float32_auto(buffer, m_values.wh_cnt_dis_total, &send_index);
	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_BMS_AH_WH_DIS_TOTAL << 8), buffer, send_index);
}
