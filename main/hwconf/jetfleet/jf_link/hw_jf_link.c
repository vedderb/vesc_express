/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se

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

#include "hw_jf_link.h"
#include "bms.h"
#include "comm_can.h"
#include "main.h"
#include "utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

#define JFBMS_MAX_SLAVES			8
#define JFBMS_CELLS_PER_SLAVE		32
#define JFBMS_TEMPS_PER_SLAVE		4
#define JFBMS_BAL_SETTLE_TIMEOUT_MS	5000
#define JF_LINK_TASK_MS		100
#define JFBMS_STATUS_MIN_PERIOD_MS	50
#define JFBMS_KEEPALIVE_MS			1000
#define JFBMS_SLAVE_TIMEOUT_S		1.0f

#define CAN_ID_TEMPS(slave_id)			(0x400 | (slave_id))
#define CAN_ID_STATUS(slave_id)			(0x480 | (slave_id))
#define CAN_ID_BAL_CMD(slave_id)		(0x500 | (slave_id))

typedef struct {
	uint16_t cells_mv[JFBMS_CELLS_PER_SLAVE];
	int16_t temps[JFBMS_TEMPS_PER_SLAVE];
	uint32_t cell_rx_time[8];
	uint32_t temp_rx_time;
	uint32_t status_rx_time;
	uint32_t bal_reported;
	uint8_t faults;
	uint8_t cells_ic1;
	uint8_t cells_ic2;
	bool got_cells;
	bool got_temps;
	bool got_status;
} jfbms_slave_t;

typedef enum {
	BAL_STATE_IDLE = 0,
	BAL_STATE_SETTLE,
	BAL_STATE_RUN,
	BAL_STATE_MANUAL,
} bal_state_t;

typedef struct {
	int slave;
	int ic;
	uint32_t bit;
	float delta;
} bal_candidate_t;

static jfbms_slave_t m_slaves[JFBMS_MAX_SLAVES];
static uint32_t m_cmd_masks[JFBMS_MAX_SLAVES];
static volatile bal_state_t m_bal_state = BAL_STATE_IDLE;
static volatile bool m_started = false;
static uint32_t m_settle_start = 0;
static uint32_t m_last_keepalive = 0;
static uint32_t m_last_status = 0;

static void link_task(void *arg);
static bool bms_cmd_hook(COMM_PACKET_ID cmd, int param1, int param2);

static int cfg_num_slaves(void) {
	int num = backup.config.num_slaves;
	if (num < 1) {
		num = 1;
	}
	if (num > JFBMS_MAX_SLAVES) {
		num = JFBMS_MAX_SLAVES;
	}

	return num;
}

static float cfg_timeout_s(void) {
	return JFBMS_SLAVE_TIMEOUT_S;
}

static bool rx_timed_out(uint32_t rx_time) {
	float timeout = cfg_timeout_s();
	return timeout > 0.0f && UTILS_AGE_S(rx_time) > timeout;
}

static int slave_cell_count(const jfbms_slave_t *s) {
	int cells = s->cells_ic1 + s->cells_ic2;
	if (cells < 0) {
		cells = 0;
	}
	if (cells > JFBMS_CELLS_PER_SLAVE) {
		cells = JFBMS_CELLS_PER_SLAVE;
	}

	return cells;
}

static uint32_t cell_to_bal_bit(const jfbms_slave_t *s, int cell) {
	if (cell < s->cells_ic1) {
		return (uint32_t)1 << cell;
	}

	int ic2_cell = cell - s->cells_ic1;
	if (ic2_cell >= 0 && ic2_cell < s->cells_ic2) {
		return (uint32_t)1 << (16 + ic2_cell);
	}

	return 0;
}

static bool cell_valid(uint16_t mv) {
	return mv > 0 && mv < 0xFFFF;
}

static bool slave_active(int ind) {
	const jfbms_slave_t *s = &m_slaves[ind];
	int cells = slave_cell_count(s);
	if (cells <= 0) {
		return false;
	}

	int cell_msgs = (cells + 3) / 4;
	for (int i = 0;i < cell_msgs;i++) {
		if (s->cell_rx_time[i] == 0 || rx_timed_out(s->cell_rx_time[i])) {
			return false;
		}
	}

	return s->got_status && s->got_cells && s->status_rx_time != 0 &&
			!rx_timed_out(s->status_rx_time);
}

static bool slave_settled(int ind) {
	return slave_active(ind) && ((m_slaves[ind].faults & 0x04) != 0);
}

static void send_bal_mask(int slave_id, uint32_t mask) {
	uint8_t buf[4];
	buf[0] = (mask >> 0) & 0xFF;
	buf[1] = (mask >> 8) & 0xFF;
	buf[2] = (mask >> 16) & 0xFF;
	buf[3] = (mask >> 24) & 0xFF;
	comm_can_transmit_sid(CAN_ID_BAL_CMD(slave_id), buf, 4);
}

static void send_all_masks(void) {
	int num = cfg_num_slaves();
	for (int i = 0;i < num;i++) {
		send_bal_mask(i + 1, m_cmd_masks[i]);
	}
	m_last_keepalive = xTaskGetTickCount();
}

static void clear_all_masks(bool transmit) {
	memset(m_cmd_masks, 0, sizeof(m_cmd_masks));

	if (transmit) {
		send_all_masks();
	}
}

static bool any_mask_active(void) {
	for (int i = 0;i < cfg_num_slaves();i++) {
		if (m_cmd_masks[i] != 0) {
			return true;
		}
	}

	return false;
}

static bool set_balance_override_cell(int cell, bool enable) {
	if (cell < 0) {
		return false;
	}

	int base = 0;
	for (int i = 0;i < cfg_num_slaves();i++) {
		jfbms_slave_t *s = &m_slaves[i];
		int cells = slave_cell_count(s);

		if (cell < (base + cells)) {
			uint32_t bit = cell_to_bal_bit(s, cell - base);
			if (bit == 0) {
				return false;
			}

			if (enable) {
				m_cmd_masks[i] |= bit;
			} else {
				m_cmd_masks[i] &= ~bit;
			}

			send_all_masks();
			return true;
		}

		base += cells;
	}

	return false;
}

static bool all_active_slaves_settled(void) {
	int active = 0;
	int num = cfg_num_slaves();

	for (int i = 0;i < num;i++) {
		if (slave_active(i)) {
			active++;
			if (!slave_settled(i)) {
				return false;
			}
		}
	}

	return active > 0;
}

static void update_bms_values(void) {
	volatile bms_values *bms = bms_get_values();

	memset((void*)bms, 0, sizeof(*bms));
	bms->can_id = backup.config.controller_id;
	bms->update_time = xTaskGetTickCount();
	bms->is_charge_allowed = 1;
	bms->soh = 1.0f;
	bms->data_version = 1;

	float v_tot = 0.0f;
	float v_min = 999.0f;
	float v_max = 0.0f;
	float temp_cell_max = -273.0f;
	float temp_ic_max = -273.0f;
	int active = 0;
	int faults = 0;
	bool waiting_settle = false;

	int num = cfg_num_slaves();
	for (int i = 0;i < num;i++) {
		jfbms_slave_t *s = &m_slaves[i];
		if (!slave_active(i)) {
			continue;
		}

		active++;
		faults |= s->faults & 0x03;
		waiting_settle = waiting_settle || !slave_settled(i);

		int cells = slave_cell_count(s);
		for (int c = 0;c < cells && bms->cell_num < BMS_MAX_CELLS;c++) {
			uint16_t mv = s->cells_mv[c];
			if (!cell_valid(mv)) {
				continue;
			}

			float v = (float)mv / 1000.0f;
			bms->v_cell[bms->cell_num] = v;
			bms->bal_state[bms->cell_num] = (m_cmd_masks[i] & cell_to_bal_bit(s, c)) != 0;
			bms->cell_num++;
			v_tot += v;
			if (v < v_min) {
				v_min = v;
			}
			if (v > v_max) {
				v_max = v;
			}
		}

		if (!s->got_temps || rx_timed_out(s->temp_rx_time)) {
			continue;
		}

		for (int t = 0;t < JFBMS_TEMPS_PER_SLAVE && bms->temp_adc_num < BMS_MAX_TEMPS;t++) {
			if (s->temps[t] == 0x7FFF) {
				continue;
			}

			float temp = (float)s->temps[t] / 10.0f;
			bms->temps_adc[bms->temp_adc_num++] = temp;
			if ((t == 0 || t == 2) && temp > temp_ic_max) {
				temp_ic_max = temp;
			} else if ((t == 1 || t == 3) && temp > temp_cell_max) {
				temp_cell_max = temp;
			}
		}
	}

	bms->v_tot = v_tot;
	bms->v_charge = v_tot;
	bms->v_cell_min = bms->cell_num > 0 ? v_min : 0.0f;
	bms->v_cell_max = bms->cell_num > 0 ? v_max : 0.0f;
	bms->temp_max_cell = temp_cell_max > -273.0f ? temp_cell_max : 0.0f;
	bms->temp_ic = temp_ic_max > -273.0f ? temp_ic_max : 0.0f;
	bms->is_balancing = any_mask_active() ? 1 : 0;

	if (bms->cell_num > 0) {
		float avg = v_tot / (float)bms->cell_num;
		float soc = (avg - 3.1f) / 1.1f;
		utils_truncate_number(&soc, 0.0f, 1.0f);
		bms->soc = soc;
	}

	snprintf((char*)bms->status, BMS_STATUS_LEN, "JFBMS %d/%d f%02X%s",
			active, num, faults, waiting_settle ? " settle" : "");
}

static bool collect_balance_candidates(bal_candidate_t *candidates, int *candidate_num,
		float *global_min) {
	*candidate_num = 0;
	*global_min = 999.0f;
	bool all_above_min = true;
	int num = cfg_num_slaves();

	for (int i = 0;i < num;i++) {
		jfbms_slave_t *s = &m_slaves[i];
		if (!slave_active(i)) {
			continue;
		}

		int cells = slave_cell_count(s);
		for (int c = 0;c < cells;c++) {
			if (!cell_valid(s->cells_mv[c])) {
				continue;
			}

			float v = (float)s->cells_mv[c] / 1000.0f;
			if (v < *global_min) {
				*global_min = v;
			}
			if (v < backup.config.vc_balance_min) {
				all_above_min = false;
			}
		}
	}

	if (*global_min > 100.0f || !all_above_min) {
		return false;
	}

	float parity_score[JFBMS_MAX_SLAVES][2][2] = {{{0.0f}}};
	for (int i = 0;i < num;i++) {
		jfbms_slave_t *s = &m_slaves[i];
		if (!slave_active(i)) {
			continue;
		}

		int cells = slave_cell_count(s);
		for (int c = 0;c < cells;c++) {
			uint32_t bit = cell_to_bal_bit(s, c);
			if (bit == 0 || !cell_valid(s->cells_mv[c])) {
				continue;
			}

			float delta = ((float)s->cells_mv[c] / 1000.0f) - *global_min;
			bool was_on = (m_cmd_masks[i] & bit) != 0;
			float threshold = was_on ? backup.config.vc_balance_end : backup.config.vc_balance_start;
			if (delta < threshold) {
				continue;
			}

			int ic = c < s->cells_ic1 ? 0 : 1;
			int local_cell = ic == 0 ? c : c - s->cells_ic1;
			parity_score[i][ic][local_cell & 1] += delta;
		}
	}

	int parity[JFBMS_MAX_SLAVES][2];
	for (int i = 0;i < JFBMS_MAX_SLAVES;i++) {
		for (int ic = 0;ic < 2;ic++) {
			parity[i][ic] = parity_score[i][ic][1] > parity_score[i][ic][0] ? 1 : 0;
		}
	}

	for (int i = 0;i < num;i++) {
		jfbms_slave_t *s = &m_slaves[i];
		if (!slave_active(i)) {
			continue;
		}

		int cells = slave_cell_count(s);
		for (int c = 0;c < cells;c++) {
			uint32_t bit = cell_to_bal_bit(s, c);
			if (bit == 0 || !cell_valid(s->cells_mv[c])) {
				continue;
			}

			float delta = ((float)s->cells_mv[c] / 1000.0f) - *global_min;
			bool was_on = (m_cmd_masks[i] & bit) != 0;
			float threshold = was_on ? backup.config.vc_balance_end : backup.config.vc_balance_start;
			if (delta < threshold) {
				continue;
			}

			int ic = c < s->cells_ic1 ? 0 : 1;
			int local_cell = ic == 0 ? c : c - s->cells_ic1;
			if ((local_cell & 1) != parity[i][ic]) {
				continue;
			}

			if (*candidate_num < (JFBMS_MAX_SLAVES * JFBMS_CELLS_PER_SLAVE)) {
				candidates[*candidate_num].slave = i;
				candidates[*candidate_num].ic = ic;
				candidates[*candidate_num].bit = bit;
				candidates[*candidate_num].delta = delta;
				(*candidate_num)++;
			}
		}
	}

	return true;
}

static void update_balance_masks(void) {
	bal_candidate_t candidates[JFBMS_MAX_SLAVES * JFBMS_CELLS_PER_SLAVE];
	int candidate_num = 0;
	float global_min = 0.0f;

	if (!collect_balance_candidates(candidates, &candidate_num, &global_min)) {
		clear_all_masks(false);
		return;
	}

	clear_all_masks(false);

	for (int i = 0;i < candidate_num - 1;i++) {
		for (int j = i + 1;j < candidate_num;j++) {
			if (candidates[j].delta > candidates[i].delta) {
				bal_candidate_t tmp = candidates[i];
				candidates[i] = candidates[j];
				candidates[j] = tmp;
			}
		}
	}

	int max_bal_per_bq = backup.config.max_bal_ch;
	if (max_bal_per_bq < 1) {
		max_bal_per_bq = 1;
	}
	if (max_bal_per_bq > 8) {
		max_bal_per_bq = 8;
	}

	int bal_per_bq[JFBMS_MAX_SLAVES][2] = {{0}};
	for (int i = 0;i < candidate_num;i++) {
		bal_candidate_t *candidate = &candidates[i];
		if (bal_per_bq[candidate->slave][candidate->ic] >= max_bal_per_bq) {
			continue;
		}

		m_cmd_masks[candidate->slave] |= candidate->bit;
		bal_per_bq[candidate->slave][candidate->ic]++;
	}
}

void hw_init(void) {
	gpio_config_t io_conf = {};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = ((1ULL << LED_RED_PIN) | (1ULL << LED_BLUE_PIN));
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	LED_RED_OFF();
	LED_BLUE_OFF();
}

void hw_jf_link_start(void) {
	if (m_started) {
		return;
	}

	m_started = true;
	for (int i = 0;i < JFBMS_MAX_SLAVES;i++) {
		for (int t = 0;t < JFBMS_TEMPS_PER_SLAVE;t++) {
			m_slaves[i].temps[t] = 0x7FFF;
		}
	}
	bms_register_cmd_hook(bms_cmd_hook);
	xTaskCreatePinnedToCore(link_task, "jf_link", 4096, NULL, 6, NULL, tskNO_AFFINITY);
}

void hw_can_rx_hook(uint32_t id, uint8_t *data, int len, bool is_ext) {
	if (is_ext || len < 1) {
		return;
	}

	uint8_t slave_id = id & 0x7F;
	if (slave_id < 1 || slave_id > cfg_num_slaves()) {
		return;
	}

	jfbms_slave_t *s = &m_slaves[slave_id - 1];
	uint32_t now = xTaskGetTickCount();

	if (id < 0x400) {
		// Cell voltage messages: types 0-7
		uint8_t type = (id >> 7) & 0x0F;
		if (type > 7 || len < 8) {
			return;
		}

		int base = type * 4;
		for (int i = 0;i < 4;i++) {
			s->cells_mv[base + i] = (uint16_t)data[i * 2] |
					((uint16_t)data[(i * 2) + 1] << 8);
		}
		s->cell_rx_time[type] = now;
		s->got_cells = true;
	} else if (id == CAN_ID_TEMPS(slave_id)) {
		if (len < 8) {
			return;
		}

		for (int i = 0;i < JFBMS_TEMPS_PER_SLAVE;i++) {
			s->temps[i] = (int16_t)((uint16_t)data[i * 2] |
					((uint16_t)data[(i * 2) + 1] << 8));
		}
		s->temp_rx_time = now;
		s->got_temps = true;
	} else if (id == CAN_ID_STATUS(slave_id)) {
		if (len < 7) {
			return;
		}

		s->bal_reported = (uint32_t)data[0] |
				((uint32_t)data[1] << 8) |
				((uint32_t)data[2] << 16) |
				((uint32_t)data[3] << 24);
		s->faults = data[4];
		s->cells_ic1 = data[5] > 16 ? 16 : data[5];
		s->cells_ic2 = data[6] > 16 ? 16 : data[6];
		s->status_rx_time = now;
		s->got_status = true;
	}
}

static bool bms_cmd_hook(COMM_PACKET_ID cmd, int param1, int param2) {
	if (cmd == COMM_BMS_SET_BALANCE_OVERRIDE) {
		if (set_balance_override_cell(param1, param2 != 0)) {
			m_bal_state = any_mask_active() ? BAL_STATE_MANUAL : BAL_STATE_IDLE;
			update_bms_values();
		}

		return true;
	}

	if (cmd != COMM_BMS_FORCE_BALANCE) {
		return false;
	}

	if (param1 != 0) {
		clear_all_masks(true);
		m_settle_start = xTaskGetTickCount();
		m_bal_state = BAL_STATE_SETTLE;
	} else {
		m_bal_state = BAL_STATE_IDLE;
		clear_all_masks(true);
		update_bms_values();
	}

	return true;
}

static void link_task(void *arg) {
	(void)arg;

	for (;;) {
		uint32_t now = xTaskGetTickCount();

		switch (m_bal_state) {
		case BAL_STATE_SETTLE: {
			clear_all_masks(false);

			if (UTILS_AGE_S(m_settle_start) >= ((float)JFBMS_BAL_SETTLE_TIMEOUT_MS / 1000.0f) ||
					all_active_slaves_settled()) {
				m_bal_state = BAL_STATE_RUN;
				update_balance_masks();
				if (!any_mask_active()) {
					m_bal_state = BAL_STATE_IDLE;
				}
				send_all_masks();
			} else if (UTILS_AGE_S(m_last_keepalive) >= ((float)JFBMS_KEEPALIVE_MS / 1000.0f)) {
				send_all_masks();
			}
		} break;

		case BAL_STATE_RUN: {
			bool any_active = false;
			for (int i = 0;i < cfg_num_slaves();i++) {
				any_active = any_active || slave_active(i);
			}

			if (!any_active) {
				m_bal_state = BAL_STATE_IDLE;
				clear_all_masks(true);
			} else if (UTILS_AGE_S(m_last_keepalive) >= ((float)JFBMS_KEEPALIVE_MS / 1000.0f)) {
				update_balance_masks();
				if (!any_mask_active()) {
					m_bal_state = BAL_STATE_IDLE;
				}
				send_all_masks();
			}
		} break;

		case BAL_STATE_MANUAL: {
			bool any_active = false;
			for (int i = 0;i < cfg_num_slaves();i++) {
				any_active = any_active || slave_active(i);
			}

			if (!any_active || !any_mask_active()) {
				m_bal_state = BAL_STATE_IDLE;
				clear_all_masks(true);
			} else if (UTILS_AGE_S(m_last_keepalive) >= ((float)JFBMS_KEEPALIVE_MS / 1000.0f)) {
				send_all_masks();
			}
		} break;

		case BAL_STATE_IDLE:
		default:
			break;
		}

		update_bms_values();

		int rate = backup.config.can_status_rate_hz;
		if (rate > 0) {
			int period_ms = 1000 / rate;
			if (period_ms < JFBMS_STATUS_MIN_PERIOD_MS) {
				period_ms = JFBMS_STATUS_MIN_PERIOD_MS;
			}

			if ((now - m_last_status) >= (uint32_t)pdMS_TO_TICKS(period_ms)) {
				bms_send_status_can();
				m_last_status = now;
			}
		}

		vTaskDelay(pdMS_TO_TICKS(JF_LINK_TASK_MS));
	}
}
