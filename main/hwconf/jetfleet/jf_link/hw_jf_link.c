/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se
	Copyright 2026 JetFleet

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
#include "commands.h"
#include "lispbm.h"
#include "lispif.h"
#include "main.h"
#include "utils.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define JFBMS_MAX_SLAVES			8
#define JFBMS_CELLS_PER_SLAVE		32
#define JFBMS_TEMPS_PER_SLAVE		4
#define JFBMS_CAN_BUF_SIZE			128
#define JFBMS_TASK_PERIOD_MS		50
#define JFBMS_STATUS_PERIOD_MS		100
#define JFBMS_BAL_KEEPALIVE_MS		1000
#define JFBMS_BAL_SETTLE_MIN_MS		2000
#define JFBMS_BAL_SETTLE_TIMEOUT_MS	5000
#define JFBMS_BAL_HOLD_MS			30000

#define CAN_ID_TEMPS(slave_id)		(0x400 | (slave_id))
#define CAN_ID_STATUS(slave_id)		(0x480 | (slave_id))
#define CAN_ID_BAL_CMD(slave_id)	(0x500 | (slave_id))

typedef enum {
	BAL_AUTO_IDLE = 0,
	BAL_AUTO_SETTLING,
	BAL_AUTO_HOLD,
} balance_auto_state_t;

typedef struct {
	uint16_t cells_mv[JFBMS_CELLS_PER_SLAVE];
	int16_t temps[JFBMS_TEMPS_PER_SLAVE];
	uint32_t balance_mask;
	uint32_t last_seen_ticks;
	uint32_t last_status_ticks;
	uint32_t status_rx_count;
	uint8_t faults;
	uint8_t status_flags_raw;
	uint8_t cells_ic1;
	uint8_t cells_ic2;
	bool active;
	bool settled;
} jfbms_slave_t;

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint8_t len;
} can_msg_t;

typedef struct {
	lbm_uint num_slaves;
	lbm_uint slave_timeout_s;
	lbm_uint max_bal_ch;
	lbm_uint vc_balance_start;
	lbm_uint vc_balance_end;
	lbm_uint vc_balance_min;
	lbm_uint can_status_rate_hz;
} config_syms_t;

typedef struct {
	int slave_id;
	int cells_ic1;
	int cells_ic2;
	int cell_count;
	float cells[JFBMS_CELLS_PER_SLAVE];
} slave_snapshot_t;

typedef struct {
	int index;
	float voltage;
} balance_candidate_t;

static jfbms_slave_t m_slaves[JFBMS_MAX_SLAVES];
static SemaphoreHandle_t m_data_mutex;

static can_msg_t m_can_rx_buf[JFBMS_CAN_BUF_SIZE];
static volatile int m_can_rx_head = 0;
static volatile int m_can_rx_tail = 0;
static volatile uint32_t m_can_rx_overflow = 0;

static config_syms_t m_syms_cfg = {0};

static uint32_t m_cached_bal_masks[JFBMS_MAX_SLAVES];
static bool m_prev_active[JFBMS_MAX_SLAVES];
static volatile int m_force_bal_pending = -1;
static volatile bool m_manual_bal_pending = false;
static volatile int m_manual_bal_cell = -1;
static volatile int m_manual_bal_enable = 0;
static bool m_bal_request = false;
static bool m_auto_balancing = false;
static bool m_manual_balancing = false;
static bool m_bal_keepalive_kick = false;
static balance_auto_state_t m_bal_auto_state = BAL_AUTO_IDLE;
static uint32_t m_bal_settle_start_ticks = 0;
static uint32_t m_bal_hold_start_ticks = 0;
static uint32_t m_bal_keepalive_ticks = 0;
static TaskHandle_t m_jf_link_task_handle = NULL;

static int process_can_rx_all(void);
static void check_slave_timeouts(float timeout_s);
static void update_vesc_bms_values(void);

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

static bool cell_valid(uint16_t mv) {
	return mv > 0 && mv < 0xFFFF;
}

static bool decode_slave_can_id(uint32_t id, uint8_t *slave_id, uint8_t *msg_type) {
	if (id > 0x7FF) {
		return false;
	}

	uint8_t sid = id & 0x0F;
	uint8_t subtype = (id >> 4) & 0x07;
	uint8_t type = (id >> 7) & 0x0F;

	if (subtype != 0 || sid < 1 || sid > JFBMS_MAX_SLAVES || type > 0x09) {
		return false;
	}

	*slave_id = sid;
	*msg_type = type;
	return true;
}

static bool valid_slave_msg_len(uint8_t msg_type, int len) {
	if (msg_type <= 0x08) {
		return len == 8;
	}

	return msg_type == 0x09 && len >= 5 && len <= 7;
}

static int logical_cell_to_wire_index(const jfbms_slave_t *s, int cell) {
	if (cell < 0) {
		return -1;
	}

	if (cell < s->cells_ic1) {
		return cell;
	}

	int ic2_cell = cell - s->cells_ic1;
	if (ic2_cell >= 0 && ic2_cell < s->cells_ic2) {
		return 16 + ic2_cell;
	}

	return -1;
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

static void init_slave_data(void) {
	memset(m_slaves, 0, sizeof(m_slaves));

	for (int s = 0;s < JFBMS_MAX_SLAVES;s++) {
		for (int t = 0;t < JFBMS_TEMPS_PER_SLAVE;t++) {
			m_slaves[s].temps[t] = 0x7FFF;
		}
	}
}

static void parse_slave_message(uint32_t id, const uint8_t *data, int len) {
	uint8_t slave_id = 0;
	uint8_t msg_type = 0xFF;

	if (!decode_slave_can_id(id, &slave_id, &msg_type)) {
		return;
	}

	if (slave_id < 1 || slave_id > cfg_num_slaves()) {
		return;
	}

	if (!valid_slave_msg_len(msg_type, len)) {
		return;
	}

	jfbms_slave_t *s = &m_slaves[slave_id - 1];

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	uint32_t now = xTaskGetTickCount();
	s->active = true;
	s->last_seen_ticks = now;

	if (msg_type <= 0x07) {
		int base_cell = msg_type * 4;
		for (int i = 0;i < 4 && (base_cell + i) < JFBMS_CELLS_PER_SLAVE;i++) {
			s->cells_mv[base_cell + i] =
					(uint16_t)data[i * 2] |
					((uint16_t)data[(i * 2) + 1] << 8);
		}
	} else if (msg_type == 0x08) {
		for (int i = 0;i < JFBMS_TEMPS_PER_SLAVE;i++) {
			s->temps[i] =
					(int16_t)((uint16_t)data[i * 2] |
					((uint16_t)data[(i * 2) + 1] << 8));
		}
	} else if (msg_type == 0x09) {
		s->balance_mask =
				(uint32_t)data[0] |
				((uint32_t)data[1] << 8) |
				((uint32_t)data[2] << 16) |
				((uint32_t)data[3] << 24);

		if (len >= 7) {
			s->cells_ic1 = data[5] > 16 ? 16 : data[5];
			s->cells_ic2 = data[6] > 16 ? 16 : data[6];
		} else if (len >= 6) {
			uint8_t total = data[5];
			s->cells_ic1 = total > 16 ? 16 : total;
			s->cells_ic2 = total > 32 ? 16 : (total > 16 ? total - 16 : 0);
		} else {
			s->cells_ic1 = 16;
			s->cells_ic2 = 16;
		}

		s->status_flags_raw = data[4];
		s->faults = data[4] & 0x07;
		s->settled = (data[4] & 0x04) != 0;

		s->last_status_ticks = now;
		s->status_rx_count++;
	}

	xSemaphoreGive(m_data_mutex);
}

static bool compare_symbol(lbm_uint sym, lbm_uint *comp) {
	if (*comp == 0) {
		if (comp == &m_syms_cfg.num_slaves) {
			lbm_add_symbol_const("num_slaves", comp);
		} else if (comp == &m_syms_cfg.slave_timeout_s) {
			lbm_add_symbol_const("slave_timeout_s", comp);
		} else if (comp == &m_syms_cfg.max_bal_ch) {
			lbm_add_symbol_const("max_bal_ch", comp);
		} else if (comp == &m_syms_cfg.vc_balance_start) {
			lbm_add_symbol_const("vc_balance_start", comp);
		} else if (comp == &m_syms_cfg.vc_balance_end) {
			lbm_add_symbol_const("vc_balance_end", comp);
		} else if (comp == &m_syms_cfg.vc_balance_min) {
			lbm_add_symbol_const("vc_balance_min", comp);
		} else if (comp == &m_syms_cfg.can_status_rate_hz) {
			lbm_add_symbol_const("can_status_rate_hz", comp);
		}
	}

	return *comp == sym;
}

static lbm_value get_or_set_i(bool set, int *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_i32(*lbm_val);
		return ENC_SYM_TRUE;
	}

	return lbm_enc_i(*val);
}

static lbm_value get_or_set_float(bool set, float *val, lbm_value *lbm_val) {
	if (set) {
		*val = lbm_dec_as_float(*lbm_val);
		return ENC_SYM_TRUE;
	}

	return lbm_enc_float(*val);
}

static lbm_value bms_get_set_param(bool set, lbm_value *args, lbm_uint argn) {
	lbm_value res = ENC_SYM_EERROR;
	lbm_value set_arg = 0;

	if (set && argn >= 1) {
		set_arg = args[argn - 1];
		argn--;

		if (!lbm_is_number(set_arg)) {
			lbm_set_error_reason((char*)lbm_error_str_no_number);
			return ENC_SYM_EERROR;
		}
	}

	if (argn != 1 || lbm_type_of(args[0]) != LBM_TYPE_SYMBOL) {
		return res;
	}

	lbm_uint name = lbm_dec_sym(args[0]);
	main_config_t *cfg = (main_config_t*)&backup.config;

	if (compare_symbol(name, &m_syms_cfg.num_slaves)) {
		res = get_or_set_i(set, &cfg->num_slaves, &set_arg);
	} else if (compare_symbol(name, &m_syms_cfg.slave_timeout_s)) {
		res = get_or_set_float(set, &cfg->slave_timeout_s, &set_arg);
	} else if (compare_symbol(name, &m_syms_cfg.max_bal_ch)) {
		res = get_or_set_i(set, &cfg->max_bal_ch, &set_arg);
	} else if (compare_symbol(name, &m_syms_cfg.vc_balance_start)) {
		res = get_or_set_float(set, &cfg->vc_balance_start, &set_arg);
	} else if (compare_symbol(name, &m_syms_cfg.vc_balance_end)) {
		res = get_or_set_float(set, &cfg->vc_balance_end, &set_arg);
	} else if (compare_symbol(name, &m_syms_cfg.vc_balance_min)) {
		res = get_or_set_float(set, &cfg->vc_balance_min, &set_arg);
	} else if (compare_symbol(name, &m_syms_cfg.can_status_rate_hz)) {
		res = get_or_set_i(set, &cfg->can_status_rate_hz, &set_arg);
	}

	return res;
}

static lbm_value ext_bms_get_param(lbm_value *args, lbm_uint argn) {
	return bms_get_set_param(false, args, argn);
}

static lbm_value ext_bms_set_param(lbm_value *args, lbm_uint argn) {
	return bms_get_set_param(true, args, argn);
}

static lbm_value ext_bms_store_cfg(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;
	main_store_backup_data();
	return ENC_SYM_TRUE;
}

static int process_can_rx_all(void) {
	int count = 0;

	while (m_can_rx_tail != m_can_rx_head) {
		can_msg_t *msg = &m_can_rx_buf[m_can_rx_tail];
		parse_slave_message(msg->id, msg->data, msg->len);
		m_can_rx_tail = (m_can_rx_tail + 1) % JFBMS_CAN_BUF_SIZE;
		count++;
	}

	return count;
}

static lbm_value ext_master_can_read_all(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	return lbm_enc_i(process_can_rx_all());
}

static lbm_value ext_master_can_available(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	int available = m_can_rx_head - m_can_rx_tail;
	if (available < 0) {
		available += JFBMS_CAN_BUF_SIZE;
	}

	return lbm_enc_i(available);
}

static lbm_value ext_master_can_overflow(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	return lbm_enc_u32(m_can_rx_overflow);
}

static lbm_value ext_master_get_slave_cells(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > JFBMS_MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	lbm_value vc_list = ENC_SYM_NIL;
	jfbms_slave_t *s = &m_slaves[slave_id - 1];

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	int num_cells = slave_cell_count(s);
	for (int i = num_cells - 1;i >= 0;i--) {
		int wire_index = logical_cell_to_wire_index(s, i);
		if (wire_index < 0) {
			continue;
		}

		uint16_t mv = s->cells_mv[wire_index];
		if (cell_valid(mv)) {
			vc_list = lbm_cons(lbm_enc_float((float)mv / 1000.0f), vc_list);
		}
	}

	xSemaphoreGive(m_data_mutex);

	return vc_list;
}

static lbm_value ext_master_get_slave_temps(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > JFBMS_MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	lbm_value ts_list = ENC_SYM_NIL;
	jfbms_slave_t *s = &m_slaves[slave_id - 1];

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = JFBMS_TEMPS_PER_SLAVE - 1;i >= 0;i--) {
		int16_t raw = s->temps[i];
		if (raw != 0x7FFF) {
			ts_list = lbm_cons(lbm_enc_float((float)raw / 10.0f), ts_list);
		}
	}

	xSemaphoreGive(m_data_mutex);

	return ts_list;
}

static lbm_value ext_master_get_slave_status(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > JFBMS_MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	jfbms_slave_t *s = &m_slaves[slave_id - 1];

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	lbm_value res = ENC_SYM_NIL;
	res = lbm_cons(lbm_enc_i(s->cells_ic2), res);
	res = lbm_cons(lbm_enc_i(s->cells_ic1), res);
	res = lbm_cons(lbm_enc_i(s->faults), res);
	res = lbm_cons(lbm_enc_u32(s->balance_mask), res);

	xSemaphoreGive(m_data_mutex);

	return res;
}

static lbm_value ext_master_slave_active(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > JFBMS_MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	jfbms_slave_t *s = &m_slaves[slave_id - 1];

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	bool active = s->active;
	xSemaphoreGive(m_data_mutex);

	return active ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_master_get_slave_settled(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > JFBMS_MAX_SLAVES) {
		return ENC_SYM_NIL;
	}

	jfbms_slave_t *s = &m_slaves[slave_id - 1];

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	uint32_t now = xTaskGetTickCount();
	bool status_fresh = s->last_status_ticks != 0 &&
			((now - s->last_status_ticks) * portTICK_PERIOD_MS) <= 1000;
	bool settled = s->active && status_fresh && s->settled && (s->faults & 0x03) == 0;
	xSemaphoreGive(m_data_mutex);

	return settled ? ENC_SYM_TRUE : ENC_SYM_NIL;
}

static lbm_value ext_master_get_active_slaves(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	lbm_value list = ENC_SYM_NIL;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = cfg_num_slaves() - 1;i >= 0;i--) {
		if (m_slaves[i].active) {
			list = lbm_cons(lbm_enc_i(i + 1), list);
		}
	}

	xSemaphoreGive(m_data_mutex);

	return list;
}

static lbm_value ext_master_get_cell_count(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > JFBMS_MAX_SLAVES) {
		return lbm_enc_i(0);
	}

	jfbms_slave_t *s = &m_slaves[slave_id - 1];

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	int count = slave_cell_count(s);
	xSemaphoreGive(m_data_mutex);

	return lbm_enc_i(count);
}

static lbm_value ext_master_get_cells_ic1(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > JFBMS_MAX_SLAVES) {
		return lbm_enc_i(0);
	}

	jfbms_slave_t *s = &m_slaves[slave_id - 1];

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	int count = s->cells_ic1;
	xSemaphoreGive(m_data_mutex);

	return lbm_enc_i(count);
}

static lbm_value ext_master_get_cells_ic2(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	int slave_id = lbm_dec_as_i32(args[0]);
	if (slave_id < 1 || slave_id > JFBMS_MAX_SLAVES) {
		return lbm_enc_i(0);
	}

	jfbms_slave_t *s = &m_slaves[slave_id - 1];

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	int count = s->cells_ic2;
	xSemaphoreGive(m_data_mutex);

	return lbm_enc_i(count);
}

static void send_balance_cmd(uint8_t slave_id, uint32_t mask, uint8_t beep_code) {
	uint8_t buf[5];
	buf[0] = (mask >> 0) & 0xFF;
	buf[1] = (mask >> 8) & 0xFF;
	buf[2] = (mask >> 16) & 0xFF;
	buf[3] = (mask >> 24) & 0xFF;
	buf[4] = beep_code;

	comm_can_transmit_sid(CAN_ID_BAL_CMD(slave_id), buf, sizeof(buf));
}

static bool ticks_elapsed(uint32_t now, uint32_t start, uint32_t ms) {
	return (now - start) >= pdMS_TO_TICKS(ms);
}

static const char *balance_state_name(balance_auto_state_t state) {
	switch (state) {
	case BAL_AUTO_IDLE: return "idle";
	case BAL_AUTO_SETTLING: return "settle";
	case BAL_AUTO_HOLD: return "hold";
	default: return "?";
	}
}

static void clear_cached_balance_masks(void) {
	memset(m_cached_bal_masks, 0, sizeof(m_cached_bal_masks));
}

static bool any_cached_balance_masks(void) {
	for (int i = 0;i < JFBMS_MAX_SLAVES;i++) {
		if (m_cached_bal_masks[i] != 0) {
			return true;
		}
	}

	return false;
}

static void send_cached_balance_masks(uint8_t beep_code) {
	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	bool active[JFBMS_MAX_SLAVES];
	for (int i = 0;i < JFBMS_MAX_SLAVES;i++) {
		active[i] = i < cfg_num_slaves() && m_slaves[i].active;
	}

	xSemaphoreGive(m_data_mutex);

	for (int i = 0;i < cfg_num_slaves();i++) {
		if (active[i]) {
			send_balance_cmd(i + 1, m_cached_bal_masks[i], beep_code);
		}
	}
}

static void stop_all_balancing(void) {
	clear_cached_balance_masks();
	send_cached_balance_masks(0);

	m_bal_request = false;
	m_auto_balancing = false;
	m_manual_balancing = false;
	m_bal_keepalive_kick = false;
	m_bal_auto_state = BAL_AUTO_IDLE;
}

static void sort_candidates(balance_candidate_t *candidates, int count) {
	for (int i = 0;i < count - 1;i++) {
		for (int j = i + 1;j < count;j++) {
			if (candidates[j].voltage > candidates[i].voltage) {
				balance_candidate_t tmp = candidates[i];
				candidates[i] = candidates[j];
				candidates[j] = tmp;
			}
		}
	}
}

static uint32_t balance_ic_group(
		const float *voltages, int cell_count, float cell_min,
		float threshold, int max_channels) {
	if (cell_count <= 0 || max_channels <= 0) {
		return 0;
	}

	if (max_channels > 16) {
		max_channels = 16;
	}

	balance_candidate_t even[JFBMS_CELLS_PER_SLAVE / 2];
	balance_candidate_t odd[JFBMS_CELLS_PER_SLAVE / 2];
	int even_count = 0;
	int odd_count = 0;
	float even_sum = 0.0f;
	float odd_sum = 0.0f;

	for (int i = 0;i < cell_count;i++) {
		float delta = voltages[i] - cell_min;
		if (delta > threshold) {
			if ((i & 1) == 0) {
				even[even_count].index = i;
				even[even_count].voltage = voltages[i];
				even_count++;
				even_sum += delta;
			} else {
				odd[odd_count].index = i;
				odd[odd_count].voltage = voltages[i];
				odd_count++;
				odd_sum += delta;
			}
		}
	}

	sort_candidates(even, even_count);
	sort_candidates(odd, odd_count);

	balance_candidate_t *selected = even;
	int selected_count = even_count;

	if (odd_count > even_count || (odd_count == even_count && odd_sum > even_sum)) {
		selected = odd;
		selected_count = odd_count;
	}

	uint32_t mask = 0;
	for (int i = 0;i < selected_count && i < max_channels;i++) {
		mask |= (uint32_t)1 << selected[i].index;
	}

	return mask;
}

static bool collect_slave_snapshots(
		slave_snapshot_t *snapshots, int *snapshot_count, int *total_cells,
		float *global_min, float *global_max) {
	*snapshot_count = 0;
	*total_cells = 0;
	*global_min = 9.0f;
	*global_max = 0.0f;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int s_ind = 0;s_ind < cfg_num_slaves() && *snapshot_count < JFBMS_MAX_SLAVES;s_ind++) {
		jfbms_slave_t *s = &m_slaves[s_ind];
		if (!s->active) {
			continue;
		}

		int count = slave_cell_count(s);
		if (count <= 0) {
			continue;
		}

		bool valid = true;
		for (int c = 0;c < count;c++) {
			int wire_index = logical_cell_to_wire_index(s, c);
			if (wire_index < 0 || !cell_valid(s->cells_mv[wire_index])) {
				valid = false;
				break;
			}
		}

		if (!valid) {
			continue;
		}

		slave_snapshot_t *dst = &snapshots[*snapshot_count];
		dst->slave_id = s_ind + 1;
		dst->cells_ic1 = s->cells_ic1;
		dst->cells_ic2 = s->cells_ic2;
		dst->cell_count = count;

		for (int c = 0;c < count;c++) {
			int wire_index = logical_cell_to_wire_index(s, c);
			float v = (float)s->cells_mv[wire_index] / 1000.0f;
			dst->cells[c] = v;

			if (v < *global_min) {
				*global_min = v;
			}
			if (v > *global_max) {
				*global_max = v;
			}
		}

		*total_cells += count;
		(*snapshot_count)++;
	}

	xSemaphoreGive(m_data_mutex);

	return *snapshot_count > 0;
}

static bool active_slaves_ready_for_balance(uint32_t now) {
	bool all_settled = true;
	int active_count = 0;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = 0;i < cfg_num_slaves();i++) {
		if (m_slaves[i].active) {
			active_count++;
			bool status_fresh = m_slaves[i].last_status_ticks != 0 &&
					((now - m_slaves[i].last_status_ticks) * portTICK_PERIOD_MS) <= 1000;
			if (!status_fresh || !m_slaves[i].settled ||
					(m_slaves[i].faults & 0x03) != 0) {
				all_settled = false;
			}
		}
	}

	xSemaphoreGive(m_data_mutex);

	return active_count > 0 && all_settled;
}

static void start_auto_settle(uint32_t now) {
	clear_cached_balance_masks();
	send_cached_balance_masks(0);

	m_bal_auto_state = BAL_AUTO_SETTLING;
	m_bal_settle_start_ticks = now;
	commands_printf("BAL: settling, balance off for >=%dms", JFBMS_BAL_SETTLE_MIN_MS);
}

static void compute_auto_balance(uint32_t now) {
	commands_printf("BAL: settle done after %lums",
			(unsigned long)((now - m_bal_settle_start_ticks) * portTICK_PERIOD_MS));

	slave_snapshot_t snapshots[JFBMS_MAX_SLAVES];
	int snapshot_count = 0;
	int total_cells = 0;
	float global_min = 9.0f;
	float global_max = 0.0f;
	float threshold = m_auto_balancing ?
			backup.config.vc_balance_end : backup.config.vc_balance_start;
	float bal_min = backup.config.vc_balance_min;
	int max_ch = backup.config.max_bal_ch;

	bool any_cells = collect_slave_snapshots(
			snapshots, &snapshot_count, &total_cells, &global_min, &global_max);

	if (any_cells) {
		commands_printf("BAL: scan slaves=%d cells=%d min=%.3f max=%.3f delta=%.3f thr=%.3f",
				snapshot_count, total_cells, (double)global_min, (double)global_max,
				(double)(global_max - global_min), (double)threshold);
	}

	if (!any_cells) {
		stop_all_balancing();
		commands_printf("BAL: stopped (no cells available)");
		return;
	}

	if (global_min <= bal_min) {
		stop_all_balancing();
		commands_printf("BAL: stopped (min %.3f <= %.3f)",
				(double)global_min, (double)bal_min);
		return;
	}

	clear_cached_balance_masks();
	bool any_balancing = false;

	for (int i = 0;i < snapshot_count;i++) {
		slave_snapshot_t *s = &snapshots[i];
		uint32_t ic1_mask = balance_ic_group(
				s->cells, s->cells_ic1, global_min, threshold, max_ch);
		uint32_t ic2_mask = 0;

		if (s->cells_ic2 > 0) {
			ic2_mask = balance_ic_group(
					&s->cells[s->cells_ic1], s->cells_ic2,
					global_min, threshold, max_ch);
		}

		uint32_t mask = ic1_mask | (ic2_mask << 16);
		m_cached_bal_masks[s->slave_id - 1] = mask;

		if (mask != 0) {
			any_balancing = true;
			commands_printf("BAL S%d IC1:0x%04lX IC2:0x%04lX min=%.3f",
					s->slave_id, (unsigned long)ic1_mask,
					(unsigned long)ic2_mask, (double)global_min);
		}
	}

	if (any_balancing) {
		m_auto_balancing = true;
		m_manual_balancing = false;
		m_bal_auto_state = BAL_AUTO_HOLD;
		m_bal_hold_start_ticks = now;
		m_bal_keepalive_kick = true;
		send_cached_balance_masks(0);
	} else {
		stop_all_balancing();
		commands_printf("BAL: target reached (delta %.3f <= %.3f)",
				(double)(global_max - global_min), (double)threshold);
	}
}

static void process_manual_balance(int cell, int enable) {
	process_can_rx_all();

	int target_slave = -1;
	int target_ic1 = 0;
	int target_ic2 = 0;
	uint32_t target_mask = 0;
	int base = 0;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = 0;i < cfg_num_slaves();i++) {
		jfbms_slave_t *s = &m_slaves[i];
		if (!s->active) {
			continue;
		}

		int count = slave_cell_count(s);
		if (count <= 0) {
			continue;
		}

		if (cell >= base && cell < (base + count)) {
			int local_cell = cell - base;
			uint32_t bit = cell_to_bal_bit(s, local_cell);
			if (bit != 0) {
				if (enable > 0) {
					m_cached_bal_masks[i] |= bit;
				} else {
					m_cached_bal_masks[i] &= ~bit;
				}

				target_slave = i + 1;
				target_ic1 = s->cells_ic1;
				target_ic2 = s->cells_ic2;
				target_mask = m_cached_bal_masks[i];
			}
			break;
		}

		base += count;
	}

	xSemaphoreGive(m_data_mutex);

	m_bal_request = false;
	m_auto_balancing = false;
	m_bal_auto_state = BAL_AUTO_IDLE;
	m_manual_balancing = any_cached_balance_masks();
	m_bal_keepalive_kick = true;

	if (target_slave > 0) {
		send_balance_cmd(target_slave, target_mask, 0);
		commands_printf("BAL OVR: cell %d %s -> S%d IC1:0x%04lX IC2:0x%04lX",
				cell, enable > 0 ? "on" : "off", target_slave,
				(unsigned long)(target_mask & 0xFFFF),
				(unsigned long)((target_mask >> 16) & 0xFFFF));
		(void)target_ic1;
		(void)target_ic2;
	} else {
		commands_printf("BAL OVR: cell %d not found", cell);
	}
}

static void process_pending_balance_commands(uint32_t now) {
	int force = m_force_bal_pending;
	if (force >= 0) {
		m_force_bal_pending = -1;

		if (force > 0) {
			if (!m_bal_request || m_bal_auto_state == BAL_AUTO_IDLE) {
				m_bal_request = true;
				m_manual_balancing = false;
				commands_printf("BAL CMD: start");
				start_auto_settle(now);
			}
		} else {
			commands_printf("BAL CMD: stop");
			stop_all_balancing();
		}
	}

	if (m_manual_bal_pending) {
		int cell = m_manual_bal_cell;
		int enable = m_manual_bal_enable;
		m_manual_bal_pending = false;
		process_manual_balance(cell, enable);
	}
}

static void run_balance_state(uint32_t now) {
	if (m_bal_request) {
		if (m_bal_auto_state == BAL_AUTO_IDLE) {
			start_auto_settle(now);
		}

		if (m_bal_auto_state == BAL_AUTO_SETTLING) {
			bool min_wait_done = ticks_elapsed(now,
					m_bal_settle_start_ticks, JFBMS_BAL_SETTLE_MIN_MS);
			bool timed_out = ticks_elapsed(now,
					m_bal_settle_start_ticks, JFBMS_BAL_SETTLE_TIMEOUT_MS);

			bool ready = active_slaves_ready_for_balance(now);
			if (min_wait_done && ready) {
				compute_auto_balance(now);
			} else if (timed_out) {
				stop_all_balancing();
				commands_printf("BAL: stopped (slaves did not report fresh settled status)");
			}
		} else if (m_bal_auto_state == BAL_AUTO_HOLD) {
			if (ticks_elapsed(now, m_bal_hold_start_ticks, JFBMS_BAL_HOLD_MS)) {
				start_auto_settle(now);
			}
		}
	}

	bool keepalive_active = m_manual_balancing ||
			(m_bal_request && m_bal_auto_state == BAL_AUTO_HOLD && m_auto_balancing);

	if (keepalive_active &&
			(m_bal_keepalive_kick ||
					ticks_elapsed(now, m_bal_keepalive_ticks, JFBMS_BAL_KEEPALIVE_MS))) {
		send_cached_balance_masks(0);
		m_bal_keepalive_ticks = now;
		m_bal_keepalive_kick = false;
	}
}

static void check_connection_changes(void) {
	bool active[JFBMS_MAX_SLAVES] = {0};

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	for (int i = 0;i < cfg_num_slaves();i++) {
		active[i] = m_slaves[i].active;
	}
	xSemaphoreGive(m_data_mutex);

	for (int i = 0;i < cfg_num_slaves();i++) {
		if (active[i] && !m_prev_active[i]) {
			commands_printf("Slave %d connected", i + 1);
		} else if (!active[i] && m_prev_active[i]) {
			commands_printf("Slave %d disconnected", i + 1);
			stop_all_balancing();
			for (int alert = 0;alert < cfg_num_slaves();alert++) {
				if (alert != i && active[alert]) {
					send_balance_cmd(alert + 1, 0, 0x04);
				}
			}
		}

		m_prev_active[i] = active[i];
	}
}

static bool jf_link_bms_cmd_hook(COMM_PACKET_ID cmd, int param1, int param2) {
	switch (cmd) {
	case COMM_BMS_FORCE_BALANCE:
		m_force_bal_pending = param1 > 0 ? 1 : 0;
		return true;

	case COMM_BMS_SET_BALANCE_OVERRIDE:
		m_manual_bal_cell = param1;
		m_manual_bal_enable = param2;
		m_manual_bal_pending = true;
		return true;

	default:
		return false;
	}
}

static void jf_link_task(void *arg) {
	(void)arg;
	commands_printf("JF Link C controller started");

	uint32_t last_status_ticks = 0;

	for (;;) {
		process_can_rx_all();

		uint32_t now = xTaskGetTickCount();
		process_pending_balance_commands(now);
		run_balance_state(now);

		if (ticks_elapsed(now, last_status_ticks, JFBMS_STATUS_PERIOD_MS)) {
			check_slave_timeouts(backup.config.slave_timeout_s);
			update_vesc_bms_values();
			if (backup.config.can_status_rate_hz != 0) {
				bms_send_status_can();
			}
			check_connection_changes();
			last_status_ticks = now;
		}

		vTaskDelay(pdMS_TO_TICKS(JFBMS_TASK_PERIOD_MS));
	}
}

static lbm_value ext_master_send_balance(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(4);

	int slave_id = lbm_dec_as_i32(args[0]);
	uint32_t ic1_mask = lbm_dec_as_u32(args[1]) & 0xFFFF;
	uint32_t ic2_mask = lbm_dec_as_u32(args[2]) & 0xFFFF;
	uint8_t beep_code = (uint8_t)lbm_dec_as_u32(args[3]);

	if (slave_id < 1 || slave_id > cfg_num_slaves()) {
		return ENC_SYM_NIL;
	}

	send_balance_cmd(slave_id, ic1_mask | (ic2_mask << 16), beep_code);
	return ENC_SYM_TRUE;
}

static void check_slave_timeouts(float timeout_s) {
	if (timeout_s > 100.0f) {
		timeout_s /= 1000.0f;
	}

	uint32_t now = xTaskGetTickCount();

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = 0;i < cfg_num_slaves();i++) {
		if (m_slaves[i].active) {
			float age_s = ((float)(now - m_slaves[i].last_seen_ticks) *
					(float)portTICK_PERIOD_MS) / 1000.0f;
			if (timeout_s > 0.0f && age_s > timeout_s) {
				m_slaves[i].active = false;
				m_cached_bal_masks[i] = 0;
			}
		}
	}

	xSemaphoreGive(m_data_mutex);
}

static lbm_value ext_master_check_timeouts(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_ARGN_NUMBER(1);

	check_slave_timeouts(lbm_dec_as_float(args[0]));
	return ENC_SYM_TRUE;
}

static lbm_value ext_master_reset_slaves(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);
	init_slave_data();
	xSemaphoreGive(m_data_mutex);

	m_can_rx_head = 0;
	m_can_rx_tail = 0;
	m_can_rx_overflow = 0;

	return ENC_SYM_TRUE;
}

static void update_vesc_bms_values(void) {
	uint32_t now = xTaskGetTickCount();
	volatile bms_values *bms = bms_get_values();
	memset((void*)bms, 0, sizeof(*bms));

	bms->can_id = backup.config.controller_id;
	bms->is_charge_allowed = 1;
	bms->soh = 1.0f;
	bms->data_version = 1;

	float v_tot = 0.0f;
	float v_min = 9999.0f;
	float v_max = 0.0f;
	float t_ic_max = -300.0f;
	float t_cell_min = 9999.0f;
	float t_cell_max = -300.0f;
	int active = 0;
	int faults = 0;
	bool waiting_settle = false;
	bool missing_status = false;
	bool any_balancing = false;

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int s_ind = 0;s_ind < cfg_num_slaves() && bms->cell_num < BMS_MAX_CELLS;s_ind++) {
		jfbms_slave_t *s = &m_slaves[s_ind];
		if (!s->active) {
			continue;
		}

		active++;
		bool status_fresh = false;
		if (s->last_status_ticks != 0) {
			uint32_t status_age_ms = (now - s->last_status_ticks) * portTICK_PERIOD_MS;
			status_fresh = status_age_ms <= 1000;
		}

		if (status_fresh) {
			faults |= s->faults & 0x03;
			waiting_settle = waiting_settle || !s->settled;
			any_balancing = any_balancing || s->balance_mask != 0;
		} else {
			missing_status = true;
			waiting_settle = true;
		}

		int num_cells = slave_cell_count(s);
		for (int c = 0;c < num_cells && bms->cell_num < BMS_MAX_CELLS;c++) {
			int wire_index = logical_cell_to_wire_index(s, c);
			if (wire_index < 0) {
				continue;
			}

			uint16_t mv = s->cells_mv[wire_index];
			if (!cell_valid(mv)) {
				continue;
			}

			float v = (float)mv / 1000.0f;
			bms->v_cell[bms->cell_num] = v;
			bms->bal_state[bms->cell_num] =
					(s->balance_mask & cell_to_bal_bit(s, c)) != 0;
			bms->cell_num++;

			v_tot += v;
			if (v < v_min) {
				v_min = v;
			}
			if (v > v_max) {
				v_max = v;
			}
		}

		for (int t = 0;t < JFBMS_TEMPS_PER_SLAVE;t++) {
			int16_t raw = s->temps[t];
			if (raw == 0x7FFF) {
				continue;
			}

			float temp_c = (float)raw / 10.0f;
			if (temp_c > 900.0f || temp_c < -273.0f) {
				continue;
			}

			if (t == 0 || t == 2) {
				if (temp_c > t_ic_max) {
					t_ic_max = temp_c;
				}
			} else {
				if (temp_c < t_cell_min) {
					t_cell_min = temp_c;
				}
				if (temp_c > t_cell_max) {
					t_cell_max = temp_c;
				}
			}
		}
	}

	bms->temps_adc[0] = t_ic_max > -299.0f ? t_ic_max : -300.0f;
	bms->temps_adc[1] = t_cell_min < 9000.0f ? t_cell_min : -300.0f;
	bms->temps_adc[2] = t_cell_max > -299.0f ? t_cell_max : -300.0f;
	bms->temps_adc[3] = -300.0f;
	bms->temps_adc[4] = -300.0f;
	bms->temp_adc_num = 5;

	for (int s_ind = 0;s_ind < cfg_num_slaves() && bms->temp_adc_num < BMS_MAX_TEMPS;s_ind++) {
		jfbms_slave_t *s = &m_slaves[s_ind];
		if (!s->active) {
			continue;
		}

		for (int t = 0;t < JFBMS_TEMPS_PER_SLAVE && bms->temp_adc_num < BMS_MAX_TEMPS;t++) {
			if (t != 1 && t != 3) {
				continue;
			}

			int16_t raw = s->temps[t];
			if (raw == 0x7FFF) {
				continue;
			}

			float temp_c = (float)raw / 10.0f;
			if (temp_c > 900.0f || temp_c < -273.0f) {
				continue;
			}

			bms->temps_adc[bms->temp_adc_num++] = temp_c;
		}
	}

	xSemaphoreGive(m_data_mutex);

	bms->v_tot = v_tot;
	bms->v_charge = v_tot;
	bms->v_cell_min = bms->cell_num > 0 ? v_min : 0.0f;
	bms->v_cell_max = bms->cell_num > 0 ? v_max : 0.0f;
	bms->temp_ic = t_ic_max > -299.0f ? t_ic_max : 0.0f;
	bms->temp_max_cell = t_cell_max > -299.0f ? t_cell_max : 0.0f;
	bms->is_balancing = any_balancing ? 1 : 0;

	if (bms->cell_num > 0) {
		float avg_v = v_tot / (float)bms->cell_num;
		float soc = (avg_v - 3.0f) / (4.2f - 3.0f);
		utils_truncate_number(&soc, 0.0f, 1.0f);
		bms->soc = soc;
	}

	const char *state_txt = "";
	if (any_balancing) {
		state_txt = " bal";
	} else if (waiting_settle) {
		state_txt = " settle";
	}

	snprintf((char*)bms->status, BMS_STATUS_LEN, "JFLink %d/%d f%02X%s%s",
			active, cfg_num_slaves(), faults,
			state_txt,
			missing_status ? " nostat" : "");

	bms->update_time = now;
}

static lbm_value ext_master_update_vesc_bms(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	update_vesc_bms_values();
	return ENC_SYM_TRUE;
}

static lbm_value ext_send_bms_status_can(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	if (backup.config.can_status_rate_hz != 0) {
		bms_send_status_can();
	}

	return ENC_SYM_TRUE;
}

static lbm_value ext_can_debug(lbm_value *args, lbm_uint argn) {
	(void)args;
	(void)argn;

	uint32_t now = xTaskGetTickCount();

	commands_printf_lisp("JF Link CAN buf: head=%d tail=%d overflow=%lu",
			m_can_rx_head, m_can_rx_tail, (unsigned long)m_can_rx_overflow);
	commands_printf_lisp("CAN core: rx_recovery=%d", comm_can_get_rx_recovery_cnt());

	uint32_t settle_age_ms = m_bal_settle_start_ticks == 0 ? 0 :
			(now - m_bal_settle_start_ticks) * portTICK_PERIOD_MS;
	uint32_t hold_age_ms = m_bal_hold_start_ticks == 0 ? 0 :
			(now - m_bal_hold_start_ticks) * portTICK_PERIOD_MS;

	commands_printf_lisp("BAL auto=%s req=%d auto-bal=%d manual=%d settle-age=%lums hold-age=%lums",
			balance_state_name(m_bal_auto_state),
			m_bal_request ? 1 : 0,
			m_auto_balancing ? 1 : 0,
			m_manual_balancing ? 1 : 0,
			(unsigned long)settle_age_ms,
			(unsigned long)hold_age_ms);

	xSemaphoreTake(m_data_mutex, portMAX_DELAY);

	for (int i = 0;i < cfg_num_slaves();i++) {
		jfbms_slave_t *s = &m_slaves[i];
		if (s->active) {
			uint32_t status_age_ms = 0xFFFFFFFF;
			if (s->last_status_ticks != 0) {
				status_age_ms = (now - s->last_status_ticks) * portTICK_PERIOD_MS;
			}

			commands_printf_lisp(
					"Slave %d: ic1=%d ic2=%d flags=0x%02X bq1=%s bq2=%s settled=%d stat=%lu age=%lums bal=0x%08lX cmd=0x%08lX",
					i + 1, s->cells_ic1, s->cells_ic2,
					s->status_flags_raw,
					(s->faults & 0x01) ? "ERR" : "ok",
					s->cells_ic2 == 0 ? "none" : ((s->faults & 0x02) ? "ERR" : "ok"),
					s->settled ? 1 : 0,
					(unsigned long)s->status_rx_count,
					(unsigned long)status_age_ms,
					(unsigned long)s->balance_mask,
					(unsigned long)m_cached_bal_masks[i]);
		}
	}

	xSemaphoreGive(m_data_mutex);

	return ENC_SYM_TRUE;
}

static void load_extensions(bool main_found) {
	(void)main_found;

	lbm_add_extension("bms-get-param", ext_bms_get_param);
	lbm_add_extension("bms-set-param", ext_bms_set_param);
	lbm_add_extension("bms-store-cfg", ext_bms_store_cfg);

	lbm_add_extension("master-can-read-all", ext_master_can_read_all);
	lbm_add_extension("master-can-available", ext_master_can_available);
	lbm_add_extension("master-can-overflow", ext_master_can_overflow);

	lbm_add_extension("master-get-slave-cells", ext_master_get_slave_cells);
	lbm_add_extension("master-get-slave-temps", ext_master_get_slave_temps);
	lbm_add_extension("master-get-slave-status", ext_master_get_slave_status);
	lbm_add_extension("master-slave-active?", ext_master_slave_active);
	lbm_add_extension("master-get-slave-settled?", ext_master_get_slave_settled);
	lbm_add_extension("master-get-active-slaves", ext_master_get_active_slaves);
	lbm_add_extension("master-get-cell-count", ext_master_get_cell_count);
	lbm_add_extension("master-get-cells-ic1", ext_master_get_cells_ic1);
	lbm_add_extension("master-get-cells-ic2", ext_master_get_cells_ic2);

	lbm_add_extension("master-send-balance", ext_master_send_balance);
	lbm_add_extension("master-check-timeouts", ext_master_check_timeouts);
	lbm_add_extension("master-reset-slaves", ext_master_reset_slaves);
	lbm_add_extension("master-update-vesc-bms", ext_master_update_vesc_bms);
	lbm_add_extension("master-send-bms-can", ext_send_bms_status_can);

	lbm_add_extension("can-debug", ext_can_debug);
}

void hw_init(void) {
	m_data_mutex = xSemaphoreCreateMutex();
	init_slave_data();

	gpio_config_t io_conf = {0};
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = ((1ULL << LED_RED_PIN) | (1ULL << LED_BLUE_PIN));
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);

	LED_RED_OFF();
	LED_BLUE_OFF();

	bms_register_cmd_hook(jf_link_bms_cmd_hook);
	lispif_add_ext_load_callback(load_extensions);

	if (!m_jf_link_task_handle) {
		xTaskCreatePinnedToCore(jf_link_task, "jf_link", 4096, NULL, 7,
				&m_jf_link_task_handle, tskNO_AFFINITY);
	}
}

void hw_can_rx_hook(uint32_t id, uint8_t *data, int len, bool is_ext) {
	if (is_ext) {
		return;
	}

	uint8_t slave_id = 0;
	uint8_t msg_type = 0xFF;
	if (!decode_slave_can_id(id, &slave_id, &msg_type) ||
			slave_id > cfg_num_slaves() ||
			!valid_slave_msg_len(msg_type, len)) {
		return;
	}

	int next_head = (m_can_rx_head + 1) % JFBMS_CAN_BUF_SIZE;
	if (next_head == m_can_rx_tail) {
		m_can_rx_overflow++;
		return;
	}

	m_can_rx_buf[m_can_rx_head].id = id;
	m_can_rx_buf[m_can_rx_head].len = len > 8 ? 8 : len;
	memcpy(m_can_rx_buf[m_can_rx_head].data, data, m_can_rx_buf[m_can_rx_head].len);
	m_can_rx_head = next_head;
}
